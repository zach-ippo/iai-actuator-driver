"""Run the driver tests: ``python tests/run_backend.py`` (from ``bindings/python``; ``--backend native`` is the same).

Sets ``IAI_DRIVER_BACKEND`` before the package is imported (it is read once, at the first import; ``native`` and
``auto`` both run on the core, the only implementation since the pure-Python edition was removed after 1.0.x) and
discovers every ``test*.py`` next to this file (or runs the test names given). The package loads the native library
at import (``IAI_ACTUATOR_DRIVER_NATIVE`` or ``core/build``); without it the run ends at once with exit code 2.

Each test, and each class or module fixture between tests, gets a time limit (``--timeout``, default 120 s): past it
``faulthandler`` dumps every thread's stack and ends the process, so a hang shows where it is instead of stalling the
CI job. At the end no thread named ``iad-*`` (the native binding's threads) may still be alive.

Prints the implementation (``backend_info()``), then ran / passed / skipped / expected failures / unexpected successes
/ failures / errors, the reasons of the skips and the slowest tests. Exit code 0 when the run is clean (an unexpected
success is not), 1 when it is not, 2 when the package does not import. No hardware and no serial port: every test
runs on the simulator.

Parallel workers (``--jobs N``; default for a whole-suite run: half the CPUs, at most 6; for named tests 1): the tests
mostly wait on the simulator clock, so they run in several processes at once. The runner splits the tests by module
(a module heavier than half a worker's share by class, a class heavier than that and without class fixtures by
test), balanced by ``tests/test_times.txt`` (seconds per test class; a class not in it counts ``DEFAULT_TEST_S`` per
test), and runs each part in its own process with its own working and temp folders. Each worker keeps the time limit
per test and the thread check; this process checks that each test ran exactly once, prints the output of a worker
that failed and one merged summary. The modules in ``SERIAL`` and the load-sensitive tests (the ``python`` lines of
``tools/load_sensitive.txt``: throughput or real-time latency is their subject) run after the others, alone, in one
process; a load-sensitive test that failed there runs once more, in a process of its own, and its second outcome
counts (the summary names it). ``--jobs 1`` is one process, as before (no retry). ``--save-times`` writes this run's
seconds per class to ``tests/test_times.txt``.
"""

from __future__ import annotations

import argparse
import faulthandler
import functools
import heapq
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from collections import Counter
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any, TextIO

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent  # bindings/python
ENV_VAR = "IAI_DRIVER_BACKEND"
CHOICES = ("native", "auto")
DEFAULT_TIMEOUT_S = 120.0
THREAD_PREFIX = "iad-"  # threads of the native binding: none may outlive the run
THREAD_GRACE_S = 2.0
SLOWEST = 5
MAX_DEFAULT_JOBS = 6
TIMES_FILE = HERE / "test_times.txt"  # seconds per test class on the core, to balance the workers (--save-times)
DEFAULT_TEST_S = 0.3  # a class missing from the time list: this many seconds per test
# Tests that only run alone: after the parallel part, in one process. Module (or module.Class): the reason.
SERIAL = {
    "test_packaging": "reads every text file of the working tree: a file another process writes or removes during "
                      "that walk breaks it",
}
# The tests every runner keeps out of its parallel workers and retries once (read_load_sensitive)
LOAD_SENSITIVE_FILE = PROJECT.parent.parent / "tools" / "load_sensitive.txt"


def parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="run_backend.py", description="Run the driver tests on the core.")
    parser.add_argument("--backend", choices=CHOICES, default=None,
                        help=f"{ENV_VAR} for the run (default: the variable, else auto; both are the core)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"seconds per test before the stacks are dumped and the run ends (0: no limit; "
                             f"default {DEFAULT_TIMEOUT_S:g})")  # fmt: skip
    parser.add_argument("-p", "--pattern", default="test*.py", help="test files to discover (default test*.py)")
    parser.add_argument("-v", "--verbose", action="store_true", help="one line per test")
    parser.add_argument("-j", "--jobs", type=int, default=None,
                        help=f"worker processes (default: {default_jobs()} for every test file, 1 for named tests; "
                             f"1: this process only)")  # fmt: skip
    parser.add_argument("--save-times", action="store_true",
                        help="write the seconds per test class of this run to tests/test_times.txt")
    parser.add_argument("--names-from", help=argparse.SUPPRESS)  # a worker: the test names, one per line
    parser.add_argument("--worker-out", help=argparse.SUPPRESS)  # a worker: where its outcomes go (JSON)
    parser.add_argument("tests", nargs="*", help="test names (module, module.Class or module.Class.test); "
                                                 "default: every test file")  # fmt: skip
    return parser.parse_args(argv)


def default_jobs() -> int:
    """About half the CPUs, at most 6: the tests wait on the simulator clock, so more buys little and loads the PC."""
    return max(1, min(MAX_DEFAULT_JOBS, (os.cpu_count() or 2) // 2))


def read_load_sensitive(suite: str, path: Path = LOAD_SENSITIVE_FILE) -> dict[str, str]:
    """``{test id: reason}`` of ``suite`` (``core``, ``python``, ``gui``, ``dotnet``) in the list of load-sensitive
    tests (``tools/load_sensitive.txt``: the suite, a space, the test id, two spaces, the reason; ``#`` starts a
    comment); missing or unreadable: none."""
    entries: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return entries
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        head, _, reason = line.partition("  ")
        kind, _, name = head.partition(" ")
        if kind == suite and name.strip():
            entries[name.strip()] = reason.strip()
    return entries


def iter_tests(suite: unittest.TestSuite | unittest.TestCase) -> Iterable[unittest.TestCase]:
    if isinstance(suite, unittest.TestSuite):
        for item in suite:
            yield from iter_tests(item)
    else:
        yield suite


def name_of(test: Any) -> str:
    """``module.Class.test`` of a test case, else the test's own id."""
    if isinstance(test, unittest.TestCase) and hasattr(test, "_testMethodName"):
        return f"{type(test).__module__}.{type(test).__qualname__}.{test._testMethodName}"
    return str(test.id())


class BackendResult(unittest.TextTestResult):
    """A text result that holds each test to ``timeout_s`` (0: no limit) and times it.
    ``TextTestRunner(resultclass=functools.partial(BackendResult, timeout_s=...))``."""

    def __init__(self, *args: Any, timeout_s: float = 0.0, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.timeout_s = timeout_s
        self.passed: list[str] = []
        self.durations: list[tuple[float, str]] = []
        self.walls: list[tuple[float, str]] = []  # each test with the fixtures before it (for tests/test_times.txt)
        self._started = 0.0
        self._last_stop = time.perf_counter()

    def _arm(self) -> None:
        """(Re)start the time limit. It stays armed from the start of the run to its end: the class and module
        fixtures (setUpClass, setUpModule, tearDownClass, tearDownModule) run between stopTest and the next startTest,
        and get a full limit of their own from the stopTest before them."""
        if self.timeout_s > 0:
            faulthandler.dump_traceback_later(self.timeout_s, exit=True)

    def startTestRun(self) -> None:
        super().startTestRun()
        self._last_stop = time.perf_counter()
        self._arm()

    def stopTestRun(self) -> None:
        if self.timeout_s > 0:
            faulthandler.cancel_dump_traceback_later()
        super().stopTestRun()

    def startTest(self, test: unittest.TestCase) -> None:
        self._arm()
        self._started = time.perf_counter()
        super().startTest(test)

    def stopTest(self, test: unittest.TestCase) -> None:
        super().stopTest(test)
        self._arm()
        now = time.perf_counter()
        self.durations.append((now - self._started, name_of(test)))
        self.walls.append((now - self._last_stop, name_of(test)))
        self._last_stop = now

    def addSuccess(self, test: unittest.TestCase) -> None:
        super().addSuccess(test)
        self.passed.append(name_of(test))


def check_result(result: Any) -> list[str]:
    """What fails a run beside its failures and errors, one line each: a test marked as an expected failure that
    passed (its mark is out of date)."""
    return [f"{name_of(test)}: passed, but is marked as an expected failure" for test in result.unexpectedSuccesses]


def live_threads(prefix: str = THREAD_PREFIX, grace_s: float = THREAD_GRACE_S) -> list[str]:
    """Names of the threads starting with ``prefix`` still alive after up to ``grace_s`` seconds."""
    deadline = time.monotonic() + grace_s
    while True:
        alive = [t for t in threading.enumerate() if t.name.startswith(prefix) and t.is_alive()]
        if not alive or time.monotonic() >= deadline:
            return sorted(t.name for t in alive)
        alive[0].join(min(0.1, max(0.0, deadline - time.monotonic())))


def describe_backend(info: dict[str, Any]) -> str:
    text = f"backend {info.get('backend')} ({info.get('reason')})"
    for key, label in (("core_version", "core"), ("abi", "ABI"), ("library", "library")):
        value = info.get(key)
        if value is not None:
            text += f"; {label} {'.'.join(map(str, value)) if isinstance(value, (tuple, list)) else value}"
    return text


def summarise(result: BackendResult, elapsed: float, out: TextIO) -> None:
    """The summary line (the form tools/unittest_jobs.py prints too), the reasons of the skips, the slowest tests."""
    print(
        f"ran {result.testsRun} tests in {elapsed:.1f} s: {len(result.passed)} passed, {len(result.skipped)} skipped, "
        f"{len(result.expectedFailures)} expected failures, {len(result.unexpectedSuccesses)} unexpected successes, "
        f"{len(result.failures)} failures, {len(result.errors)} errors",
        file=out,
    )
    reasons = Counter(reason for _, reason in result.skipped)
    for reason, count in sorted(reasons.items()):
        print(f"  skipped {count}: {reason}", file=out)
    if result.durations:
        slowest = sorted(result.durations, reverse=True)[:SLOWEST]
        print("slowest: " + ", ".join(f"{name} {seconds:.1f} s" for seconds, name in slowest), file=out)


# ---------------------------------------------------------------------------------------------- parallel workers


def unit_of(test: Any) -> tuple[str, str]:
    """(module, module.Class) of a loaded test; for a module that failed to import (or was skipped at import) the
    module name twice."""
    if type(test).__module__ == "unittest.loader" and hasattr(test, "_testMethodName"):
        return test._testMethodName, test._testMethodName
    module = type(test).__module__
    return module, f"{module}.{type(test).__qualname__}"


def read_times(path: Path = TIMES_FILE) -> dict[str, float]:
    """``module.Class seconds`` per line (``#`` starts a comment); missing or unreadable: empty."""
    times: dict[str, float] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return times
    for line in lines:
        parts = line.split("#", 1)[0].split()
        if len(parts) == 2:
            try:
                times[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return times


def save_times(walls: Iterable[tuple[float, str]], full: bool, path: Path = TIMES_FILE,
               runner: str = "tests/run_backend.py") -> None:  # fmt: skip
    """Write the seconds per test class of this run (``walls``: seconds and ``module.Class.test`` names) into ``path``;
    the other classes keep their line unless the run was ``full`` (then the classes that no longer exist go)."""
    measured: dict[str, float] = {}
    for seconds, name in walls:
        cls = name.rsplit(".", 1)[0]
        measured[cls] = measured.get(cls, 0.0) + seconds
    times = {} if full else read_times(path)
    times.update(measured)
    lines = [f"{name}  {seconds:.2f}" for name, seconds in sorted(times.items())]
    header = (f"# Seconds per test class: {runner} splits the suite into balanced worker processes with them.\n"
              f"# Rewritten by --save-times; a class missing here counts {DEFAULT_TEST_S} s per test.\n")  # fmt: skip
    path.write_text(header + "\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {len(measured)} class times to {path}")


def class_fixtures(test: Any) -> bool:
    """Whether the test's class has a setUpClass or tearDownClass of its own (shared state: its tests stay together)."""
    cls = type(test)
    return any(getattr(getattr(cls, n, None), "__func__", None) is not getattr(unittest.TestCase, n).__func__
               for n in ("setUpClass", "tearDownClass"))  # fmt: skip


def plan(tests: Sequence[Any], jobs: int, times: dict[str, float],
         serial: dict[str, str] | None = None) -> tuple[list[list[str]], list[str]]:  # fmt: skip
    """Split ``tests`` (loaded test cases, in their run order) into at most ``jobs`` lists of names to load, balanced
    by ``times``, and the names that run alone afterwards (``serial``: module, module.Class or module.Class.test; in
    run order). A module is one part, unless it weighs more than half a worker's share: then each of its classes is
    one, and a class that weighs more than half a share on its own, without class fixtures, is split into its tests
    (each an equal part of the class's time); a class with a test that runs alone is one part of its other tests.
    Deterministic: the heaviest part goes to the least loaded worker first (ties: the earlier one); each worker loads
    its names in run order."""
    serial = SERIAL if serial is None else serial
    order: dict[str, int] = {}
    classes: dict[str, list[str]] = {}  # module -> its classes, in run order
    members: dict[str, list[str]] = {}  # class -> its tests, in run order
    fixtures: dict[str, bool] = {}
    alone: list[str] = []
    held: set[str] = set()  # the modules and classes of a test that runs alone: never loaded whole by a worker
    for test in tests:
        module, cls = unit_of(test)
        name = name_of(test)
        key = module if module in serial else cls if cls in serial else name if name in serial else None
        if key is not None:
            if key not in alone:
                alone.append(key)
            if key == name:
                held |= {module, cls}
            continue
        for unit in (module, cls, name):
            order.setdefault(unit, len(order))
        if cls not in classes.setdefault(module, []):
            classes[module].append(cls)
            fixtures[cls] = class_fixtures(test)
        members.setdefault(cls, []).append(name)
    weight = {cls: times.get(cls, DEFAULT_TEST_S * len(names)) for cls, names in members.items()}
    share = sum(weight.values()) / max(1, jobs)
    parts: list[tuple[float, list[str]]] = []  # (seconds, the names to load)
    for module, in_module in classes.items():
        module_weight = sum(weight[c] for c in in_module)
        whole = module_weight <= share / 2 or (len(in_module) == 1 and len(members[in_module[0]]) == 1)
        if whole and module not in held:
            parts.append((module_weight, [module]))
            continue
        for cls in in_module:
            names = members[cls]
            if weight[cls] > share / 2 and len(names) > 1 and not fixtures[cls] and cls != module:
                parts += [(weight[cls] / len(names), [n]) for n in names]
            else:
                parts.append((weight[cls], names if cls in held else [cls]))
    bins: list[tuple[float, int, list[str]]] = [(0.0, k, []) for k in range(max(1, jobs))]
    heapq.heapify(bins)
    for seconds, part in sorted(parts, key=lambda p: (-p[0], order[p[1][0]])):
        load, k, names = heapq.heappop(bins)
        names += part
        heapq.heappush(bins, (load + seconds, k, names))
    workers = [sorted(names, key=order.__getitem__) for _, _, names in sorted(bins, key=lambda b: b[1]) if names]
    return workers, alone


class _Named:
    """A test known by its name only (an outcome a worker reported): what ``name_of`` and the summary need."""

    def __init__(self, name: str) -> None:
        self.name = name

    def id(self) -> str:
        return self.name

    def __str__(self) -> str:
        return self.name


class MergedResult:
    """The outcomes of every worker, in the shape ``check_result`` and ``summarise`` read."""

    def __init__(self) -> None:
        self.testsRun = 0
        self.passed: list[str] = []
        self.skipped: list[tuple[_Named, str]] = []
        self.expectedFailures: list[tuple[_Named, str]] = []
        self.unexpectedSuccesses: list[_Named] = []
        self.failures: list[tuple[_Named, str]] = []
        self.errors: list[tuple[_Named, str]] = []
        self.durations: list[tuple[float, str]] = []
        self.walls: list[tuple[float, str]] = []

    def add(self, data: dict[str, Any]) -> None:
        self.testsRun += data["ran"]
        self.passed += data["passed"]
        self.skipped += [(_Named(n), r) for n, r in data["skipped"]]
        self.expectedFailures += [(_Named(n), "") for n in data["expected_failures"]]
        self.unexpectedSuccesses += [_Named(n) for n in data["unexpected_successes"]]
        self.failures += [(_Named(n), "") for n in data["failures"]]
        self.errors += [(_Named(n), "") for n in data["errors"]]
        self.durations += [(s, n) for s, n in data["durations"]]
        self.walls += [(s, n) for s, n in data["walls"]]

    def wasSuccessful(self) -> bool:
        return not self.failures and not self.errors and not self.unexpectedSuccesses


def take_retry(results: list[dict[str, Any]], retry: dict[str, Any]) -> dict[str, Any]:
    """The outcomes of a retry, to add to ``results`` (the outcomes of the run before it, as workers report them):
    the retried tests' failures and errors there give way (in place), and the retry adds no test run or time."""
    again = set(retry["loaded"])
    for data in results:
        for key in ("failures", "errors"):
            data[key] = [name for name in data[key] if name not in again]
    return dict(retry, ran=0, loaded=[], durations=[], walls=[])


def worker_data(result: BackendResult, loaded: Sequence[str], problems: Sequence[str]) -> dict[str, Any]:
    return {
        "loaded": list(loaded),
        "ran": result.testsRun,
        "passed": result.passed,
        "skipped": [[name_of(t), reason] for t, reason in result.skipped],
        "expected_failures": [name_of(t) for t, _ in result.expectedFailures],
        "unexpected_successes": [name_of(t) for t in result.unexpectedSuccesses],
        "failures": [name_of(t) for t, _ in result.failures],
        "errors": [name_of(t) for t, _ in result.errors],
        "durations": result.durations,
        "walls": result.walls,
        "problems": list(problems),
    }


def start_worker(args: argparse.Namespace, backend: str, names: list[str], home: Path) -> subprocess.Popen:
    """One worker: this script on ``names``, in ``home`` (its working folder and temp folder: nothing a test writes by
    a relative path or under a fixed temp name meets another worker's file or lands in the checkout)."""
    (home / "tmp").mkdir(parents=True)
    (home / "names.txt").write_text("\n".join(names) + "\n", encoding="utf-8")
    temp = str(home / "tmp")
    env = dict(os.environ, **{ENV_VAR: backend, "TMP": temp, "TEMP": temp, "TMPDIR": temp, "PYTHONUNBUFFERED": "1"})
    cmd = [sys.executable, str(Path(__file__).resolve()), "--backend", backend, "--jobs", "1", "--timeout",
           str(args.timeout), "--names-from", str(home / "names.txt"), "--worker-out", str(home / "result.json"),
           *(["-v"] if args.verbose else [])]  # fmt: skip
    with open(home / "log.txt", "wb") as log:
        return subprocess.Popen(cmd, cwd=home, env=env, stdout=log, stderr=subprocess.STDOUT)


def run_workers(args: argparse.Namespace, backend: str, workers: list[list[str]], alone: list[str], out: TextIO,
                alone_label: str = "alone") -> tuple[list[dict[str, Any]], list[str], list[str]]:  # fmt: skip
    """Run each list of names in its own process, all at once, then ``alone`` in one more process (its label:
    ``alone_label``). Prints a line per worker as it ends (and its output when it failed, or with -v). Returns the
    outcomes of the workers that reported, the names they loaded, and the problems (a worker that ended without a
    result)."""
    folder = Path(tempfile.mkdtemp(prefix="iad-run-backend-"))
    results: list[dict[str, Any]] = []
    loaded: list[str] = []
    problems: list[str] = []
    batches = [[(f"worker {k}/{len(workers)}", names) for k, names in enumerate(workers, 1)]]
    if alone:
        batches.append([(f"{alone_label} ({', '.join(alone)})", alone)])
    running: list[tuple[str, Path, subprocess.Popen, float]] = []
    try:
        for number, batch in enumerate(batches):
            for index, (label, names) in enumerate(batch):
                home = folder / f"{number}-{index}"
                running.append((label, home, start_worker(args, backend, names, home), time.perf_counter()))
            while running:
                for item in list(running):
                    label, home, proc, started = item
                    code = proc.poll()
                    if code is None:
                        continue
                    running.remove(item)
                    seconds = time.perf_counter() - started
                    try:
                        data = json.loads((home / "result.json").read_text(encoding="utf-8"))
                    except (OSError, ValueError):
                        data = None
                    if data is None:
                        problems.append(f"{label} ended without a result (exit code {code})")
                    else:
                        results.append(data)
                        loaded += data["loaded"]
                        if code not in (0, 1):
                            problems.append(f"{label} ended with exit code {code}")
                    clean = data is not None and code == 0
                    ran = data["ran"] if data else 0
                    print(f"{label}: {ran} tests in {seconds:.1f} s: {'OK' if clean else 'FAILED'}", file=out,
                          flush=True)  # fmt: skip
                    if not clean or args.verbose:
                        text = (home / "log.txt").read_text(encoding="utf-8", errors="replace").rstrip("\n")
                        print(f"--- output of {label}\n{text}\n--- end of {label}", file=out, flush=True)
                if running:
                    time.sleep(0.05)
    finally:
        for _, _, proc, _ in running:  # Ctrl+C or an error here: leave no worker behind
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        shutil.rmtree(folder, ignore_errors=True)
    return results, loaded, problems


def run_parallel(args: argparse.Namespace, info: dict[str, Any], tests: list[Any], full: bool,
                 workers: list[list[str]], alone: list[str], out: TextIO,
                 retry: Iterable[str] = ()) -> int:  # fmt: skip
    """The parallel run; a test in ``retry`` (the load-sensitive tests) that failed runs once more, and its second
    outcome counts."""
    names = [name_of(t) for t in tests]
    print(f"{len(names)} tests in {len(workers)} worker processes (--jobs 1: one process)"
          + (f", then alone: {', '.join(alone)}" if alone else ""), file=out, flush=True)  # fmt: skip
    started = time.perf_counter()
    results, loaded, problems = run_workers(args, info["backend"], workers, alone, out)
    failed = {name for data in results for name in data["failures"] + data["errors"]} & set(retry)
    again = [name for name in names if name in failed]
    second: list[str] = []
    if again:
        retried, _, more = run_workers(args, info["backend"], [], again, out, alone_label="retry")
        problems += more
        for data in retried:
            results.append(take_retry(results, data))
            second += data["passed"]
    elapsed = time.perf_counter() - started

    merged = MergedResult()
    for data in results:
        merged.add(data)
        problems += data["problems"]
    expected, got = Counter(names), Counter(loaded)
    missing = sorted((expected - got).elements())
    extra = sorted((got - expected).elements())
    if missing:
        more = " ..." if len(missing) > 20 else ""
        problems.append(f"{len(missing)} tests did not run: " + ", ".join(missing[:20]) + more)
    if extra:
        problems.append("loaded by a worker but not by this run (or twice): " + ", ".join(extra[:20]))
    problems = check_result(merged) + problems
    summarise(merged, elapsed, out)  # type: ignore[arg-type]
    for name in second:
        print(f"passed on its retry (load-sensitive): {name}", file=out)
    for problem in problems:
        print(f"FAIL {problem}", file=out)
    clean = merged.wasSuccessful() and not problems
    print(f"{'OK' if clean else 'FAILED'} on the {info['backend']} backend", file=out)
    if args.save_times and clean:
        save_times(merged.walls, full)
    return 0 if clean else 1


def main(argv: Sequence[str] | None = None, out: TextIO | None = None) -> int:
    args = parse_args(argv)
    out = out or sys.stdout
    backend = args.backend or (os.environ.get(ENV_VAR) or "auto").strip().lower()
    if backend not in CHOICES:  # python: the removed pure-Python edition (the package says so at its import)
        print(f"{ENV_VAR}={backend}: not a choice any more; choose {' or '.join(CHOICES)} (both are the core)",
              file=out)  # fmt: skip
        return 2
    os.environ[ENV_VAR] = backend  # read once, at the package's first import: before anything imports it
    for folder in (PROJECT, HERE):  # the tests, then this checkout's package (as unittest run from bindings/python)
        if str(folder) not in sys.path:
            sys.path.insert(0, str(folder))
    try:
        import iai_actuator_driver
    except (ImportError, NotImplementedError) as exc:
        print(f"{ENV_VAR}={backend}: the package does not import: {type(exc).__name__}: {exc}", file=out)
        return 2
    info = iai_actuator_driver.backend_info()
    print(describe_backend(info), file=out)

    loader = unittest.TestLoader()
    names = list(args.tests)
    if args.names_from:  # a worker of a parallel run
        names = [n.strip() for n in Path(args.names_from).read_text(encoding="utf-8").splitlines() if n.strip()]
    if names:
        suite = loader.loadTestsFromNames(names)
    else:
        suite = loader.discover(str(HERE), pattern=args.pattern, top_level_dir=str(HERE))
    full = not names and args.pattern == "test*.py"
    jobs = args.jobs if args.jobs is not None else (1 if names else default_jobs())
    if jobs > 1 and not args.worker_out:
        tests = list(iter_tests(suite))
        tail = read_load_sensitive("python")
        workers, alone = plan(tests, jobs, read_times(), {**SERIAL, **tail})
        if len(workers) + bool(alone) > 1:
            return run_parallel(args, info, tests, full, workers, alone, out, retry=tail)
    loaded = [name_of(t) for t in iter_tests(suite)]
    resultclass = functools.partial(BackendResult, timeout_s=max(0.0, args.timeout))
    runner = unittest.TextTestRunner(stream=out, verbosity=2 if args.verbose else 1, resultclass=resultclass)
    started = time.perf_counter()
    result = runner.run(suite)
    elapsed = time.perf_counter() - started
    assert isinstance(result, BackendResult)

    threads = [f"thread {name} is still alive after the run" for name in live_threads()]
    problems = check_result(result) + threads
    summarise(result, elapsed, out)
    for problem in problems:
        print(f"FAIL {problem}", file=out)
    clean = result.wasSuccessful() and not problems
    print(f"{'OK' if clean else 'FAILED'} on the {info['backend']} backend", file=out)
    if args.worker_out:  # the parent checks the whole run; it needs each outcome
        data = worker_data(result, loaded, threads)
        Path(args.worker_out).write_text(json.dumps(data), encoding="utf-8")
    elif args.save_times and clean:
        save_times(result.walls, full)
    return 0 if clean else 1


if __name__ == "__main__":
    sys.exit(main())
