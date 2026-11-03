#!/usr/bin/env python3
"""Run a unittest suite as several processes at once, split by test class: the GUI suite (apps/gui).

    python tools/unittest_jobs.py -s apps/gui -t apps [--jobs N] [--expect-backend native] [--timeout S]
                                  [--load-sensitive gui] [-v]

From the repository root, with PYTHONPATH naming bindings/python (the driver) as for
``python -m unittest discover -s apps/gui -t apps``. The tests mostly wait on the simulator clock, so they run in
parallel processes almost as fast as one alone. The runner discovers the suite, splits it by module (a module heavier
than its share by class) balanced by a time list (``--times``, default ``<start>/test_times.txt``: seconds per test
class; a class not in it counts 0.3 s per test), and runs each part in its own process with its own working and temp
folders. Each worker holds every test to ``--timeout`` seconds (default 120; past it the worker dumps its stacks and
ends). The runner prints the output of a worker that failed, checks that every discovered test ran exactly once, and
prints one summary: ``ran N tests in X s: P passed, S skipped, E expected failures, U unexpected successes,
F failures, R errors``, then ``OK`` or ``FAILED``. Exit code 0 only when every worker passed and ended with the exit
code its result calls for (a crash after the result fails the run), 1 otherwise, 5 when no test was discovered (as
``unittest discover``). ``--jobs 1`` runs the suite in one worker process. ``--save-times`` rewrites the time list from
this run. ``--load-sensitive SUITE``: the tests of that suite in ``tools/load_sensitive.txt`` (throughput or real-time
latency is their subject) run after the workers, one after another in one more process, and the failed ones once more
in another; their second outcome counts (the summary names a test that passed then).

No test opens a serial port: every test runs on the simulator.
"""

from __future__ import annotations

import argparse
import faulthandler
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from collections import Counter
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any, TextIO

ROOT = Path(__file__).resolve().parent.parent
RUN_BACKEND = ROOT / "bindings" / "python" / "tests" / "run_backend.py"
DEFAULT_TIMEOUT_S = 120.0
SLOWEST = 5
NO_TESTS_EXIT = 5  # as ``python -m unittest discover`` when it finds no test


def _run_backend() -> Any:
    """bindings/python/tests/run_backend.py: its test planner (by module or class, balanced by a time list)."""
    spec = importlib.util.spec_from_file_location("iad_run_backend", RUN_BACKEND)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


rb = _run_backend()


def iter_tests(suite: unittest.TestSuite | unittest.TestCase) -> Iterable[unittest.TestCase]:
    return rb.iter_tests(suite)


class Result(unittest.TextTestResult):
    """Times each test (with the fixtures before it) and holds it to ``timeout_s`` (0: no limit)."""

    def __init__(self, *args: Any, timeout_s: float = 0.0, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.timeout_s = timeout_s
        self.passed: list[str] = []
        self.durations: list[tuple[float, str]] = []
        self.walls: list[tuple[float, str]] = []
        self._started = self._last_stop = time.perf_counter()

    def _arm(self) -> None:
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
        self.durations.append((now - self._started, rb.name_of(test)))
        self.walls.append((now - self._last_stop, rb.name_of(test)))
        self._last_stop = now

    def addSuccess(self, test: unittest.TestCase) -> None:
        super().addSuccess(test)
        self.passed.append(rb.name_of(test))


def worker(args: argparse.Namespace) -> int:
    """One part of the suite, in this process; its outcomes go to ``--worker-out`` as JSON."""
    names = [n.strip() for n in Path(args.names_from).read_text(encoding="utf-8").splitlines() if n.strip()]
    suite = unittest.TestLoader().loadTestsFromNames(names)
    loaded = [rb.name_of(t) for t in iter_tests(suite)]
    runner = unittest.TextTestRunner(stream=sys.stdout, verbosity=2 if args.verbose else 1,
                                     resultclass=lambda *a, **k: Result(*a, timeout_s=args.timeout, **k))
    result = runner.run(suite)
    data = {
        "loaded": loaded,
        "ran": result.testsRun,
        "passed": result.passed,
        "skipped": [[rb.name_of(t), reason] for t, reason in result.skipped],
        "expected_failures": [rb.name_of(t) for t, _ in result.expectedFailures],
        "unexpected_successes": [rb.name_of(t) for t in result.unexpectedSuccesses],
        "failures": [rb.name_of(t) for t, _ in result.failures],
        "errors": [rb.name_of(t) for t, _ in result.errors],
        "durations": result.durations,
        "walls": result.walls,
    }
    Path(args.worker_out).write_text(json.dumps(data), encoding="utf-8")
    return 0 if result.wasSuccessful() else 1


def start_worker(args: argparse.Namespace, names: list[str], home: Path) -> subprocess.Popen:
    (home / "tmp").mkdir(parents=True)
    (home / "names.txt").write_text("\n".join(names) + "\n", encoding="utf-8")
    temp = str(home / "tmp")
    path = os.pathsep.join(p for p in (str(Path(args.top_level).resolve()), os.environ.get("PYTHONPATH", "")) if p)
    env = dict(os.environ, TMP=temp, TEMP=temp, TMPDIR=temp, PYTHONUNBUFFERED="1", PYTHONPATH=path)
    cmd = [sys.executable, str(Path(__file__).resolve()), "--worker", "-t", str(Path(args.top_level).resolve()),
           "--names-from", str(home / "names.txt"), "--worker-out", str(home / "result.json"),
           "--timeout", str(args.timeout), *(["-v"] if args.verbose else [])]  # fmt: skip
    with open(home / "log.txt", "wb") as log:
        # its own working folder: nothing a test writes by a relative path lands in the checkout
        return subprocess.Popen(cmd, cwd=home, env=env, stdout=log, stderr=subprocess.STDOUT)


def run_parts(args: argparse.Namespace, parts: list[tuple[str, list[str]]], folder: Path,
              out: TextIO) -> tuple[list[dict[str, Any]], list[str]]:  # fmt: skip
    """Run each part (a label and the names to load) in its own process, all at once, in a folder of its own under
    ``folder``; prints a line per part as it ends (and its output when it failed, or with -v). Returns the outcomes of
    the parts that reported and the problems (a part that ended without a result, or with an exit code its result does
    not call for)."""
    running = []
    results: list[dict[str, Any]] = []
    problems: list[str] = []
    try:
        for label, names in parts:
            home = folder / "-".join(label.replace("/", " ").split())
            running.append((label, home, start_worker(args, names, home), time.perf_counter()))
        while running:
            for item in list(running):
                label, home, proc, t0 = item
                code = proc.poll()
                if code is None:
                    continue
                running.remove(item)
                try:
                    data = json.loads((home / "result.json").read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    data = None
                if data is None:
                    problems.append(f"{label} ended without a result (exit code {code})")
                else:
                    results.append(data)
                    # the worker exits 1 exactly when a test failed: any other code after its result is a crash at
                    # exit (a native finalizer, an atexit handler, a Tk thread abort)
                    failed = data["failures"] or data["errors"] or data["unexpected_successes"]
                    if code != (1 if failed else 0):
                        problems.append(f"{label} ended with exit code {code} after its result")
                clean = data is not None and code == 0
                print(f"{label}: {data['ran'] if data else 0} tests in {time.perf_counter() - t0:.1f} s: "
                      f"{'OK' if clean else 'FAILED'}", file=out, flush=True)  # fmt: skip
                if not clean or args.verbose:
                    text = (home / "log.txt").read_text(encoding="utf-8", errors="replace").rstrip("\n")
                    print(f"--- output of {label}\n{text}\n--- end of {label}", file=out, flush=True)
            if running:
                time.sleep(0.05)
    finally:
        for _, _, proc, _ in running:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
    return results, problems


def run(args: argparse.Namespace, out: TextIO) -> int:
    top = Path(args.top_level).resolve()
    if str(top) not in sys.path:
        sys.path.insert(0, str(top))
    try:
        import iai_actuator_driver
        backend = getattr(iai_actuator_driver, "BACKEND", "?")
    except Exception as exc:  # noqa: BLE001 (reported, then the run goes on: the suite reports it again)
        backend = f"none ({type(exc).__name__}: {exc})"
    print(f"driver backend {backend}", file=out)
    if args.expect_backend and backend != args.expect_backend:
        print(f"FAILED: the driver runs on {backend}, not {args.expect_backend} (IAI_ACTUATOR_DRIVER_NATIVE, "
              f"PYTHONPATH)", file=out)  # fmt: skip
        return 2
    suite = unittest.TestLoader().discover(str(Path(args.start).resolve()), pattern=args.pattern,
                                           top_level_dir=str(top))  # fmt: skip
    tests = list(iter_tests(suite))
    names = [rb.name_of(t) for t in tests]
    if not names:  # as unittest: nothing discovered is not a pass (a test file that no longer matches the pattern)
        print(f"FAIL no test discovered in {args.start} (pattern {args.pattern})\nFAILED", file=out, flush=True)
        return NO_TESTS_EXIT
    times_file = Path(args.times) if args.times else Path(args.start) / "test_times.txt"
    tail = rb.read_load_sensitive(args.load_sensitive, Path(args.load_sensitive_file)) if args.load_sensitive else {}
    workers, alone = rb.plan(tests, max(1, args.jobs), rb.read_times(times_file), serial=tail)
    print(f"{len(names)} tests in {len(workers)} worker processes"
          + (f", then alone: {', '.join(alone)}" if alone else ""), file=out, flush=True)  # fmt: skip

    folder = Path(tempfile.mkdtemp(prefix="iad-unittest-jobs-"))
    started = time.perf_counter()
    second: list[str] = []
    try:
        results, problems = run_parts(args, [(f"worker {k}/{len(workers)}", part)
                                             for k, part in enumerate(workers, 1)], folder, out)  # fmt: skip
        if alone:  # the load-sensitive tests: after the workers, one after another; a failed one once more
            first, more = run_parts(args, [("tail (load-sensitive)", alone)], folder, out)
            results, problems = results + first, problems + more
            failed = {n for data in first for n in data["failures"] + data["errors"]}
            again = [n for n in alone if n in failed]
            if again:
                retried, more = run_parts(args, [("retry", again)], folder, out)
                problems += more
                for data in retried:
                    results.append(rb.take_retry(results, data))
                    second += data["passed"]
    finally:
        shutil.rmtree(folder, ignore_errors=True)
    elapsed = time.perf_counter() - started

    total: Counter[str] = Counter()
    durations: list[tuple[float, str]] = []
    walls: list[tuple[float, str]] = []
    loaded: Counter[str] = Counter()
    for data in results:
        loaded.update(data["loaded"])
        total["ran"] += data["ran"]
        for key in ("passed", "skipped", "expected_failures", "unexpected_successes", "failures", "errors"):
            total[key] += len(data[key])
        durations += [(s, n) for s, n in data["durations"]]
        walls += [(s, n) for s, n in data["walls"]]
    expected = Counter(names)
    missing = sorted((expected - loaded).elements())
    extra = sorted((loaded - expected).elements())
    if missing:
        more = " ..." if len(missing) > 20 else ""
        problems.append(f"{len(missing)} tests did not run: " + ", ".join(missing[:20]) + more)
    if extra:
        problems.append("ran more than once or not discovered: " + ", ".join(extra[:20]))
    print(f"ran {total['ran']} tests in {elapsed:.1f} s: {total['passed']} passed, {total['skipped']} skipped, "
          f"{total['expected_failures']} expected failures, {total['unexpected_successes']} unexpected successes, "
          f"{total['failures']} failures, {total['errors']} errors", file=out)  # fmt: skip
    if durations:
        slowest = sorted(durations, reverse=True)[:SLOWEST]
        print("slowest: " + ", ".join(f"{name} {seconds:.1f} s" for seconds, name in slowest), file=out)
    for name in second:
        print(f"passed on its retry (load-sensitive): {name}", file=out)
    for problem in problems:
        print(f"FAIL {problem}", file=out)
    clean = not problems and not (total["failures"] or total["errors"] or total["unexpected_successes"])
    print("OK" if clean else "FAILED", file=out, flush=True)
    if clean and args.save_times:
        rb.save_times(walls, True, times_file, runner="tools/unittest_jobs.py")
    return 0 if clean else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="unittest_jobs.py", description="Run a unittest suite in parallel processes.")
    parser.add_argument("-s", "--start", default=".", help="the folder to discover (as unittest discover -s)")
    parser.add_argument("-t", "--top-level", default=None, help="the top-level folder (as unittest discover -t)")
    parser.add_argument("-p", "--pattern", default="test*.py")
    parser.add_argument("-j", "--jobs", type=int, default=rb.default_jobs(),
                        help=f"worker processes (default {rb.default_jobs()}: half the CPUs, at most 6)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"seconds per test before its worker dumps the stacks and ends (0: no limit; default "
                             f"{DEFAULT_TIMEOUT_S:g})")  # fmt: skip
    parser.add_argument("--times", help="seconds per test class (default: <start>/test_times.txt)")
    parser.add_argument("--save-times", action="store_true", help="rewrite the time list from this run")
    parser.add_argument("--expect-backend", help="fail unless iai_actuator_driver.BACKEND is this (e.g. native)")
    parser.add_argument("--load-sensitive", metavar="SUITE",
                        help="run this suite's tests in tools/load_sensitive.txt after the workers, with one retry")
    parser.add_argument("--load-sensitive-file", default=str(rb.LOAD_SENSITIVE_FILE), help=argparse.SUPPRESS)
    parser.add_argument("-v", "--verbose", action="store_true", help="every worker's output, one line per test")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--names-from", help=argparse.SUPPRESS)
    parser.add_argument("--worker-out", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    args.top_level = args.top_level or args.start
    if args.worker:
        top = str(Path(args.top_level).resolve())
        if top not in sys.path:
            sys.path.insert(0, top)
        return worker(args)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass
    return run(args, sys.stdout)


if __name__ == "__main__":
    sys.exit(main())
