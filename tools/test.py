#!/usr/bin/env python3
"""Build once, then run the test suites a change needs, at the same time, and print one table.

    python tools/test.py                    the suites the changes since the merge base with main need (--changed)
    python tools/test.py --changed [BASE]   the same against BASE (a commit, branch or tag)
    python tools/test.py --all              every suite
    python tools/test.py --suites core,gui  these suites
    options: -j N (worker processes of the parallel suites in all, default: two thirds of the CPUs, at most 8),
             --no-build, --dry-run, --timeout S (per suite; ends the suite's whole process tree), -v (--changed: the
             suites of each changed path)

Suites (simulator only: no test opens a serial port):
    core    the C++ tests (core/tests/run_shards.py: ec_tests in parallel shards)
    smoke   the C smoke test (core/build.py smoke: tests/c_smoke.c as C99 against the DLL)
    python  the driver tests on the core (bindings/python/tests/run_backend.py --backend native, parallel workers)
    gui     the GUI tests (tools/unittest_jobs.py -s apps/gui -t apps, parallel workers, on the core)
    dotnet  the C# tests (dotnet test of bindings/dotnet/IaiActuatorDriver.sln against the DLL)
    docs    the documentation, version and wording checks (test_packaging) and the
            tests of the runners themselves (test_runners)
    hw      the hardware acceptance harness rehearsed on the simulator (tools/unittest_jobs.py -s tools/hw/tests)
    release the tests of the release scripts (.github/release/test_local_release.py: no build, no Windows or WSL)

--changed maps each changed path (committed since BASE, uncommitted, or new and not ignored) to the suites in PATH_MAP;
a path no rule names selects every suite, a change to documents only selects docs (and gui for the ones its About
panel serves). A text file anywhere (.py, .md, .txt, ...) also selects docs: the wording check reads it. The release
suite runs when .github/release/, .github/workflows/, core/CMakeLists.txt or core/build.py change, or a file its
tests read: LICENSE, licenses/, apps/gui/packaging/, apps/gui/iai-gui.spec and the NuGet project. A moved file counts
with its old path too.

--all runs every suite; docs runs inside python when both are selected (the python suite runs the same tests, so no
docs row shows).

The build (core/build.py with IAD_BUILD_DIR, dotnet build) runs first, one step after another, then smoke; then the
other selected suites run at once. The parallel suites (core, python, gui, hw) share -j worker processes, one
each and the rest to the suite that would take longest; dotnet and docs are one process each beside them. Each suite's
output goes to a log in <build>/test-logs/<time>/ (the last 10 runs are kept); the table gives suite, result,
workers, seconds and counts, and the end of a failed suite's log is printed. Exit code 0 when every selected suite
passed, 1 otherwise.

The load-sensitive tests (tools/load_sensitive.txt: throughput or real-time latency is their subject) run after the
parallel part of their suite, one after another, and a failed one runs once more (its second outcome counts): the
core, python and gui runners do that themselves (this script passes them the core names and the gui suite); the
dotnet ones run in a dotnet test of their own (--filter) after every suite has ended, and once more when it failed.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "core"
PY = ROOT / "bindings" / "python"
SLN = ROOT / "bindings" / "dotnet" / "IaiActuatorDriver.sln"
BUILD = Path(os.environ["IAD_BUILD_DIR"]).resolve() if os.environ.get("IAD_BUILD_DIR") else CORE / "build"
EXE = ".exe" if os.name == "nt" else ""
DLL = BUILD / ("iai_actuator_driver.dll" if os.name == "nt" else "libiai_actuator_driver.so")
MAX_WORKERS = 6  # per suite
MAX_BUDGET = 8  # worker processes in all, by default (-j sets more)
KEEP_LOGS = 10  # runs of test-logs kept
SUITE_TIMEOUT_S = 3600.0
CORE_CWD = "core-cwd"  # in each run's log folder

LOAD_SENSITIVE = ROOT / "tools" / "load_sensitive.txt"  # the runners' serial tail (read_load_sensitive)
# The text files test_packaging reads for its wording checks (its TEXT_SUFFIXES; test_runners keeps the two in step) and
# the version check: a change to one of them selects docs, whatever else its rule says.
WORDING_SUFFIXES = {".py", ".md", ".toml", ".js", ".html", ".css", ".json", ".txt", ".cfg", ".gitignore"}

ALL = ("smoke", "core", "python", "gui", "dotnet", "docs", "hw", "release")
NATIVE = {"core", "smoke", "python", "gui", "dotnet", "docs", "hw"}  # what a change to the core can break
# seconds each suite takes alone in one process (shares out the worker budget); dotnet, docs, release and smoke stay
# one process
WEIGHT = {"core": 430.0, "python": 350.0, "gui": 180.0, "dotnet": 80.0, "docs": 5.0, "hw": 86.0,
          "release": 14.0, "smoke": 2.0}  # fmt: skip
PARALLEL = {"core", "python", "gui", "hw"}

# (path prefix, suites): the first rule that matches a changed path decides. A path no rule matches selects ALL.
PATH_MAP: list[tuple[str, set[str]]] = [
    ("core/tests/c_smoke.c", {"smoke"}),
    ("core/tests/", {"core"}),  # docs too for its text files, see WORDING_SUFFIXES (run_shards.py, ...)
    # the customers' build; core/build.py does not read it (docs: the version check; release: the Linux packaging)
    ("core/CMakeLists.txt", {"core", "release"}),
    ("core/include/iai_actuator_driver/version.hpp", NATIVE),
    ("core/build.py", (NATIVE - {"docs"}) | {"release"}),  # the Windows build; the release scripts call it
    ("core/", NATIVE - {"docs"}),  # the library everything binds to
    ("bindings/python/iai_actuator_driver/", {"python", "gui", "hw"}),  # the binding the GUI and the harness run on
    ("bindings/python/ec_driver/", {"python"}),
    ("bindings/python/tests/test_packaging.py", {"docs"}),
    # the planner of the GUI and hw runners too (tools/unittest_jobs.py imports plan(), name_of() and the time list)
    ("bindings/python/tests/run_backend.py", {"python", "gui", "hw", "docs"}),
    ("bindings/python/tests/test_times.txt", {"python"}),
    ("bindings/python/tests/", {"python"}),
    ("bindings/python/", {"python", "docs"}),  # pyproject.toml, setup.py, README.md
    # release: test_local_release reads these (the GUI zip's notices, the spec they describe, the NuGet metadata)
    ("apps/gui/packaging/", {"gui", "docs", "release"}),
    ("apps/gui/iai-gui.spec", {"gui", "docs", "release"}),
    ("apps/gui/", {"gui", "docs"}),  # docs: test_packaging checks the GUI's pyproject.toml and pages
    ("bindings/dotnet/src/IaiActuatorDriver/IaiActuatorDriver.csproj", {"dotnet", "docs", "release"}),
    ("bindings/dotnet/", {"dotnet", "docs"}),  # docs: the version check reads the C# projects
    ("tools/hw/", {"hw"}),
    ("tools/unittest_jobs.py", {"gui", "hw", "docs"}),  # the runner of both
    ("tools/load_sensitive.txt", {"core", "python", "gui", "dotnet", "docs"}),  # the suites that run its tests
    ("tools/test.py", {"docs"}),  # docs: test_runners checks this file
    # the documents the GUI's About panel serves (apps/gui/server.py ABOUT_DOCS; test_runners keeps the list in step)
    ("docs/GETTING_STARTED.md", {"docs", "gui"}),
    ("docs/COORDINATED.md", {"docs", "gui"}),
    ("docs/COMPATIBILITY.md", {"docs", "gui"}),
    ("docs/DRIVER.md", {"docs", "gui"}),
    ("docs/REFERENCE.md", {"docs", "gui"}),
    ("README.md", {"docs", "gui"}),
    ("CHANGELOG.md", {"docs", "gui"}),
    ("docs/", {"docs"}),
    ("examples/python/", {"python", "docs"}),  # python: test_examples runs each one on the simulator
    ("examples/", {"docs"}),
    (".github/release/", {"docs", "release"}),  # docs: the wording check; release: its tests (and their own file)
    (".github/workflows/", {"docs", "release"}),  # test_local_release reads ci.yml and release.yml
    (".github/", {"docs"}),  # CONTRIBUTING.md, the issue forms
    (".gitignore", {"docs"}),
    (".gitattributes", {"docs"}),
    (".editorconfig", {"docs"}),
    # docs: test_packaging compares the package copies with them; release: the release scripts pack and check them
    ("LICENSE", {"docs", "release"}),
    ("licenses/", {"docs", "release"}),  # NOTICE, NATIVE_THIRD_PARTY.txt
]


def suites_for(path: str) -> set[str] | None:
    """The suites a change to ``path`` needs (None: no rule names it)."""
    for prefix, suites in PATH_MAP:
        if path == prefix or (prefix.endswith("/") and path.startswith(prefix)):
            found = set(suites)
            name = Path(path).name
            if Path(path).suffix.lower() in WORDING_SUFFIXES or name == ".gitignore":
                found.add("docs")  # the wording check reads every text file of the tree
            return found
    return None


def changed_paths(base: str | None, root: Path = ROOT) -> tuple[str, list[str]] | None:
    """(the base commit, the paths changed since it: committed, uncommitted, new); None when git cannot say."""
    def git(*args: str) -> str | None:
        proc = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True)
        return proc.stdout if proc.returncode == 0 else None

    ref = base or (git("merge-base", "HEAD", "main") or "").strip()
    if not ref:
        return None
    # --no-renames: a moved file lists its old path too (a module moved out of a folder still selects its suites)
    diff = git("diff", "--no-renames", "--name-only", ref, "--")
    new = git("ls-files", "--others", "--exclude-standard")
    if diff is None or new is None:
        return None
    paths = sorted({p.strip() for p in (diff + new).splitlines() if p.strip()})
    return ref, paths


def share_workers(selected: list[str], budget: int) -> dict[str, int]:
    """One worker each, then the rest of ``budget`` one at a time to the parallel suite that would take longest
    (the one-process suites do not count: they mostly wait, beside the others)."""
    workers = {s: 1 for s in selected}
    left = budget - len([s for s in selected if s in PARALLEL])
    while left > 0:
        open_ = [s for s in selected if s in PARALLEL and workers[s] < MAX_WORKERS]
        if not open_:
            break
        slowest = max(open_, key=lambda s: (WEIGHT[s] / workers[s], -ALL.index(s)))
        workers[slowest] += 1
        left -= 1
    return workers


def default_budget() -> int:
    """Two thirds of the CPUs, at most MAX_BUDGET: the tests wait on the simulator clock, so a worker takes a fraction
    of a CPU, and the suites then end together (``share_workers``) instead of the longest one running alone."""
    return max(1, min(MAX_BUDGET, (os.cpu_count() or 2) * 2 // 3))


@dataclass
class Suite:
    name: str
    cmd: list[str]
    cwd: Path
    env: dict[str, str]
    workers: int = 1
    tail: list[str] | None = None  # the command of its load-sensitive tests, run after every suite (run_tail)
    log: Path | None = None
    proc: subprocess.Popen | None = None
    started: float = 0.0
    seconds: float = 0.0
    code: int | None = None
    counts: str = ""
    timed_out: bool = False
    retried: list[str] = field(default_factory=list)  # load-sensitive tests that passed only on their retry


def suite_env(gui: bool = False) -> dict[str, str]:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(PY), *([str(ROOT / "apps")] if gui else [])])
    env["IAI_ACTUATOR_DRIVER_NATIVE"] = str(DLL)
    env["PYTHONUNBUFFERED"] = "1"
    env["IAI_DRIVER_BACKEND"] = "native"
    return env


def dotnet_env() -> dict[str, str]:
    return dict(os.environ, MSBUILDDISABLENODEREUSE="1", DOTNET_CLI_USE_MSBUILD_SERVER="0", DOTNET_NOLOGO="1",
                DOTNET_CLI_TELEMETRY_OPTOUT="1", DOTNET_SKIP_FIRST_TIME_EXPERIENCE="1")  # fmt: skip


def read_load_sensitive(suite: str) -> list[str]:
    """The suite's tests in tools/load_sensitive.txt, in its order (run_backend.read_load_sensitive reads the file for
    every runner)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("iad_run_backend", PY / "tests" / "run_backend.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.read_load_sensitive(suite, LOAD_SENSITIVE))


def make_suite(name: str, workers: int, logs: Path) -> Suite:
    py = sys.executable
    if name == "core":
        scratch = logs / CORE_CWD  # the tests' working folder: nothing lands in the checkout (main() creates it)
        tail = [arg for test in read_load_sensitive("core") for arg in ("--tail", test)]
        return Suite(name, [py, str(CORE / "tests" / "run_shards.py"), str(BUILD / f"ec_tests{EXE}"), "--quiet",
                            "--shards", str(workers), "--cwd", str(scratch), *tail], ROOT, dict(os.environ), workers)
    if name == "python":
        return Suite(name, [py, "tests/run_backend.py", "--backend", "native", "--jobs", str(workers)], PY,
                     suite_env(), workers)  # fmt: skip
    if name == "docs":
        return Suite(name, [py, "tests/run_backend.py", "--backend", "native", "test_packaging", "test_runners"], PY,
                     suite_env())  # fmt: skip
    if name == "gui":
        return Suite(name, [py, str(ROOT / "tools" / "unittest_jobs.py"), "-s", "apps/gui", "-t", "apps", "--jobs",
                            str(workers), "--expect-backend", "native", "--load-sensitive", "gui"], ROOT,
                     suite_env(gui=True), workers)  # fmt: skip
    if name == "hw":
        return Suite(name, [py, str(ROOT / "tools" / "unittest_jobs.py"), "-s", "tools/hw/tests", "-t", ".", "--jobs",
                            str(workers), "--expect-backend", "native"], ROOT, suite_env(), workers)
    if name == "dotnet":
        cmd = ["dotnet", "test", str(SLN), "--no-build", f"-p:NativeLibraryPath={DLL}", "--results-directory",
               str(logs / "dotnet-results"), "--nologo"]  # fmt: skip
        last = read_load_sensitive("dotnet")
        if not last:
            return Suite(name, cmd, ROOT, dotnet_env())
        return Suite(name, [*cmd, "--filter", "&".join(f"FullyQualifiedName!={t}" for t in last)], ROOT, dotnet_env(),
                     tail=[*cmd, "--filter", "|".join(f"FullyQualifiedName={t}" for t in last)])  # fmt: skip
    if name == "release":  # reads files and runs scratch scripts: needs no build and no DLL
        return Suite(name, [py, str(ROOT / ".github" / "release" / "test_local_release.py")], ROOT,
                     dict(os.environ, PYTHONUNBUFFERED="1"))  # fmt: skip
    if name == "smoke":
        return Suite(name, [py, "build.py", "smoke"], CORE, dict(os.environ))
    raise ValueError(name)


# ------------------------------------------------------------------------------------------------- result parsing

def parse_counts(name: str, text: str) -> str:
    def last(pattern: str) -> re.Match[str] | None:
        found = list(re.finditer(pattern, text, re.M))
        return found[-1] if found else None

    if name == "core":
        m = last(r"^(\d+) tests, (\d+) passed, (\d+) failed, \d+ checks, [\d.]+ s$")
        return f"{m[1]} tests, {m[2]} passed, {m[3]} failed" if m else ""
    if name in ("python", "docs", "gui", "hw"):  # run_backend.py and unittest_jobs.py print the same summary
        m = last(r"^ran (\d+) tests in [\d.]+ s: (\d+) passed, (\d+) skipped, \d+ expected failures, (\d+) unexpected "
                 r"successes, (\d+) failures, (\d+) errors")
        return f"{m[1]} tests, {m[2]} passed, {m[3]} skipped, {int(m[5]) + int(m[6])} failed" if m else ""
    if name == "dotnet":
        m = last(r"(Passed|Failed)!\s+-\s+Failed:\s+(\d+), Passed:\s+(\d+), Skipped:\s+(\d+), Total:\s+(\d+)")
        return f"{m[5]} tests, {m[3]} passed, {m[4]} skipped, {m[2]} failed" if m else ""
    if name == "release":  # unittest: "Ran 39 tests in 13.6s", then OK or FAILED (failures=1, errors=2, skipped=3)
        ran, end = last(r"^Ran (\d+) tests? in [\d.]+s$"), last(r"^(?:OK|FAILED)\b(.*)$")
        if not ran:
            return ""
        n = {k: int(v) for k, v in re.findall(r"(?<!expected )(failures|errors|skipped)=(\d+)", end[1] if end else "")}
        failed, skipped = n.get("failures", 0) + n.get("errors", 0), n.get("skipped", 0)
        return f"{ran[1]} tests, {int(ran[1]) - failed - skipped} passed, {skipped} skipped, {failed} failed"
    if name == "smoke":
        return "c_smoke"
    return ""


def retried_tests(text: str) -> list[str]:
    """The tests a runner's log names as passed only on their retry (run_shards.py, run_backend.py, unittest_jobs.py):
    a load-sensitive test that failed once. The table shows them, so an intermittent timing regression is seen."""
    return re.findall(r"passed on its retry(?: \(load-sensitive\))?: (\S+)", text)


def add_counts(a: str, b: str) -> str:
    """Two ``N tests, P passed, S skipped, F failed`` counts added up (the other one when one is not that)."""
    pattern = r"^(\d+) tests, (\d+) passed, (\d+) skipped, (\d+) failed$"
    ma, mb = re.match(pattern, a), re.match(pattern, b)
    if not (ma and mb):
        return a or b
    n, p, s, f = (int(x) + int(y) for x, y in zip(ma.groups(), mb.groups()))
    return f"{n} tests, {p} passed, {s} skipped, {f} failed"


# ------------------------------------------------------------------------------------------------------- the run

def run_step(label: str, cmd: list[str], cwd: Path, env: dict[str, str], log: Path) -> tuple[bool, float]:
    print(f"build: {label}", flush=True)
    t0 = time.monotonic()
    with open(log, "wb") as f:
        code = subprocess.run(cmd, cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT).returncode
    seconds = time.monotonic() - t0
    if code != 0:
        print(f"build: {label} FAILED (exit code {code}, {seconds:.1f} s); the end of {log}:")
        print_tail(log, 40)
    return code == 0, seconds


def print_tail(log: Path, lines: int) -> None:
    try:
        text = log.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return
    for line in text[-lines:]:
        print(f"    {line}")


def build(selected: list[str], logs: Path) -> tuple[bool, float, list[str]]:
    """The build steps the suites need, one after another (two builders never share a build folder)."""
    steps: list[tuple[str, list[str], Path, dict[str, str]]] = []
    py = sys.executable
    if set(selected) & {"python", "gui", "dotnet", "docs", "hw", "smoke"}:
        steps.append(("core shared (the DLL)", [py, "build.py", "shared"], CORE, dict(os.environ)))
    if "core" in selected:
        steps.append(("core tests (ec_tests)", [py, "build.py", "tests"], CORE, dict(os.environ)))
    if "dotnet" in selected:
        steps.append(("dotnet build", ["dotnet", "build", str(SLN), f"-p:NativeLibraryPath={DLL}", "--nologo",
                                       "-v", "q"], ROOT, dotnet_env()))  # fmt: skip
    t0 = time.monotonic()
    done = []
    for i, (label, cmd, cwd, env) in enumerate(steps):
        ok, _ = run_step(label, cmd, cwd, env, logs / f"build-{i + 1}.log")
        if not ok:
            return False, time.monotonic() - t0, done
        done.append(label.split(" (")[0])
    return True, time.monotonic() - t0, done


def kill_tree(proc: subprocess.Popen) -> None:
    """Kill ``proc`` and every process it started (a runner's workers and their test processes): ``proc.kill()``
    alone ends only the runner and leaves the rest running."""
    if proc.poll() is None:
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        else:
            try:
                os.killpg(proc.pid, signal.SIGKILL)  # the suite started its own process group (run_suites)
            except OSError:
                pass
    try:
        proc.kill()  # in case the tree kill missed it
    except OSError:
        pass
    proc.wait()


def run_suites(suites: list[Suite], timeout_s: float) -> None:
    for s in suites:
        assert s.log is not None
        s.started = time.monotonic()
        with open(s.log, "wb") as f:
            s.proc = subprocess.Popen(s.cmd, cwd=s.cwd, env=s.env, stdout=f, stderr=subprocess.STDOUT,
                                      start_new_session=os.name != "nt")  # POSIX: a process group to kill as one
    running = list(suites)
    try:
        while running:
            for s in list(running):
                assert s.proc is not None and s.log is not None
                code = s.proc.poll()
                if code is None and time.monotonic() - s.started > timeout_s:
                    kill_tree(s.proc)
                    code = s.proc.wait()
                    s.timed_out = True
                if code is None:
                    continue
                running.remove(s)
                s.code, s.seconds = code, time.monotonic() - s.started
                text = s.log.read_text(encoding="utf-8", errors="replace")
                s.counts, s.retried = parse_counts(s.name, text), retried_tests(text)
                state = "OK" if code == 0 else ("killed after the time limit" if s.timed_out else "FAILED")
                print(f"{s.name}: {state} in {s.seconds:.1f} s" + (f" ({s.counts})" if s.counts else ""), flush=True)
            if running:
                time.sleep(0.1)
    finally:
        for s in running:  # Ctrl+C or an error here: leave no test process behind
            if s.proc is not None and s.proc.poll() is None:
                kill_tree(s.proc)


def run_tail(s: Suite, timeout_s: float) -> None:
    """The suite's load-sensitive tests (``s.tail``), alone, and once more when they failed: the outcome of the last
    run joins the suite's (counts, seconds, result), its log is <suite>-tail.log or <suite>-retry.log. Tests that passed
    only on the retry join ``s.retried``."""
    assert s.tail is not None and s.log is not None
    first = ""
    for label in ("tail", "retry"):
        log = s.log.with_name(f"{s.name}-{label}.log")
        started = time.monotonic()
        with open(log, "wb") as f:
            proc = subprocess.Popen(s.tail, cwd=s.cwd, env=s.env, stdout=f, stderr=subprocess.STDOUT,
                                    start_new_session=os.name != "nt")  # fmt: skip
            try:
                code = proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                kill_tree(proc)
                code = proc.returncode
            except BaseException:  # Ctrl+C: leave no test process behind
                kill_tree(proc)
                raise
        seconds = time.monotonic() - started
        text = log.read_text(encoding="utf-8", errors="replace")
        counts = parse_counts(s.name, text)
        s.retried += retried_tests(text)
        print(f"{s.name} {label} (load-sensitive): {'OK' if code == 0 else 'FAILED'} in {seconds:.1f} s"
              + (f" ({counts})" if counts else ""), flush=True)  # fmt: skip
        s.seconds += seconds
        if code == 0:
            if label == "retry":  # dotnet test: "  Failed <name> [12 ms]" in the first run's log
                s.retried += re.findall(r"^\s*Failed (\S+) \[", first, re.M) or ["load-sensitive tail"]
            break
        first = text
    s.counts = add_counts(s.counts, counts)
    if code != 0 and s.code == 0:
        s.code, s.log = code, log  # the end of its log is printed with the failed suites


def prune_logs(parent: Path) -> None:
    runs = sorted(p for p in parent.glob("*") if p.is_dir())
    for old in runs[:-KEEP_LOGS]:
        shutil.rmtree(old, ignore_errors=True)


def select(args: argparse.Namespace) -> tuple[list[str], str]:
    if args.all:
        return list(ALL), "--all"
    if args.suites:
        names = [n.strip() for n in args.suites.split(",") if n.strip()]
        bad = [n for n in names if n not in ALL]
        if bad:
            raise SystemExit(f"unknown suite(s): {', '.join(bad)} (suites: {', '.join(ALL)})")
        return [n for n in ALL if n in names], "--suites"
    found = changed_paths(args.changed)
    if found is None:
        return list(ALL), "git could not list the changes: every suite"
    base, paths = found
    selected: set[str] = set()
    unknown = []
    for path in paths:
        suites = suites_for(path)
        if suites is None:
            unknown.append(path)
        selected |= set(ALL) if suites is None else suites
    why = f"{len(paths)} paths changed since {base[:12]}"
    if unknown:
        why += f" ({len(unknown)} not in the path map, so every suite: {', '.join(unknown[:5])}" + (
            " ..." if len(unknown) > 5 else "") + ")"
    if args.verbose:
        for path in paths:
            suites = suites_for(path)
            print(f"  {path}: {', '.join(n for n in ALL if n in suites) if suites is not None else 'every suite'}")
    return [n for n in ALL if n in selected], why


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="tools/test.py", description="Build once, then run the test suites a "
                                     "change needs, at the same time.")  # fmt: skip
    pick = parser.add_mutually_exclusive_group()
    pick.add_argument("--all", action="store_true", help="every suite")
    pick.add_argument("--suites", help=f"a comma-separated list of: {', '.join(ALL)}")
    pick.add_argument("--changed", nargs="?", const=None, default=None, metavar="BASE",
                      help="the suites the changes since BASE need (default: the merge base with main)")
    parser.add_argument("-j", "--jobs", type=int, default=default_budget(),
                        help=f"worker processes of core, python, gui and hw in all (default {default_budget()})")
    parser.add_argument("--no-build", action="store_true", help="use what is built already")
    parser.add_argument("--dry-run", action="store_true", help="show the suites and commands, run nothing")
    parser.add_argument("--timeout", type=float, default=SUITE_TIMEOUT_S, help="seconds per suite (default 3600)")
    parser.add_argument("-v", "--verbose", action="store_true", help="--changed: the suites of each changed path")
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass

    selected, why = select(args)
    if "python" in selected and "docs" in selected:
        selected.remove("docs")  # the python suite runs the same checks
    print(f"suites: {', '.join(selected) or 'none'} ({why})")
    if not selected:
        print("nothing to test")
        return 0
    stamp = time.strftime("%Y%m%d-%H%M%S")
    logs = BUILD / "test-logs" / stamp
    workers = share_workers(selected, max(1, args.jobs))
    suites = [make_suite(name, workers[name], logs) for name in selected]
    if args.dry_run:
        for s in suites:
            print(f"  {s.name} ({s.workers} worker{'s' * (s.workers > 1)}): {' '.join(s.cmd)}  [in {s.cwd}]")
            if s.tail:
                print(f"  {s.name} load-sensitive, after every suite: {' '.join(s.tail)}  [in {s.cwd}]")
        return 0
    logs.mkdir(parents=True, exist_ok=True)
    if "core" in selected:
        (logs / CORE_CWD).mkdir(exist_ok=True)
    prune_logs(logs.parent)
    t0 = time.monotonic()

    rows: list[tuple[str, str, str, float, str]] = []
    if args.no_build:
        needed = [DLL] if set(selected) - {"core", "release"} else []
        needed += [BUILD / f"ec_tests{EXE}"] if "core" in selected else []
        missing = [str(p) for p in needed if not p.exists()]
        if missing:
            print(f"--no-build: not built: {', '.join(missing)}")
            return 1
    else:
        ok, seconds, done = build(selected, logs)
        rows.append(("build", "ok" if ok else "FAILED", ", ".join(done), seconds, ""))
        if not ok:
            print_table(rows, time.monotonic() - t0, logs, False)
            return 1

    serial = [s for s in suites if s.name == "smoke"]  # a builder: never alongside another build.py
    for s in suites:
        s.log = logs / f"{s.name}.log"
    if serial:
        run_suites(serial, args.timeout)
    run_suites([s for s in suites if s not in serial], args.timeout)
    for s in suites:
        if s.tail:
            run_tail(s, args.timeout)

    ok = all(s.code == 0 for s in suites)
    for s in suites:
        state = "OK" if s.code == 0 else ("TIMEOUT" if s.timed_out else "FAILED")
        if s.code == 0 and s.retried:
            state += f" ({len(s.retried)} retried)"
        rows.append((s.name, state, s.counts, s.seconds, str(s.workers)))
    for s in suites:
        if s.code != 0 and s.log is not None:
            print(f"\n--- {s.name} FAILED (exit code {s.code}); the end of {s.log}:")
            print_tail(s.log, 60)
    print_table(rows, time.monotonic() - t0, logs, ok)
    for s in suites:
        for name in s.retried:  # passed, but failed once: an intermittent timing regression shows here first
            print(f"retried: {s.name} {name}")
    return 0 if ok else 1


def print_table(rows: list[tuple[str, str, str, float, str]], total: float, logs: Path, ok: bool) -> None:
    print()
    width = max([8, *(len(row[1]) for row in rows)])  # "OK (1 retried)"
    print(f"{'suite':<8} {'result':<{width}} {'workers':>7}  {'seconds':>8}  counts")
    for name, state, counts, seconds, workers in rows:
        print(f"{name:<8} {state:<{width}} {workers:>7}  {seconds:>8.1f}  {counts}")
    print(f"{'total':<8} {'OK' if ok else 'FAILED':<{width}} {'':>7}  {total:>8.1f}  logs: {logs}")


if __name__ == "__main__":
    sys.exit(main())
