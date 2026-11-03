"""The test runners themselves: ``tools/test.py`` (what a change selects, the suite time limit, --dry-run),
``tools/unittest_jobs.py`` (the GUI and hw runner), ``core/tests/run_shards.py`` (the C++ runner) and the serial tail
of the load-sensitive tests (``tools/load_sensitive.txt``) that they and ``run_backend.py`` share.

No hardware and no serial port: scratch test files and a stand-in test executable (a Python script), run in
temporary folders; git is used on a scratch repository only.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
import io
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import test_packaging

ROOT = Path(__file__).resolve().parents[3]  # the repository root
TEST_PY = ROOT / "tools" / "test.py"
UNITTEST_JOBS = ROOT / "tools" / "unittest_jobs.py"
RUN_SHARDS = ROOT / "core" / "tests" / "run_shards.py"
SERVER_PY = ROOT / "apps" / "gui" / "server.py"
RUN_BACKEND = ROOT / "bindings" / "python" / "tests" / "run_backend.py"
LOAD_SENSITIVE = ROOT / "tools" / "load_sensitive.txt"


def load(name: str, path: Path):  # noqa: ANN201 (a module)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses look the module up by name
    spec.loader.exec_module(module)
    return module


def pid_alive(pid: int) -> bool:
    if os.name == "nt":
        import ctypes

        kernel = ctypes.windll.kernel32
        handle = kernel.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return False
        code = ctypes.c_ulong()
        kernel.GetExitCodeProcess(handle, ctypes.byref(code))
        kernel.CloseHandle(handle)
        return code.value == 259  # STILL_ACTIVE
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def kill_pid(pid: int) -> None:
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(pid)], capture_output=True)
    else:
        try:
            os.kill(pid, 9)
        except OSError:
            pass


_TOOL: list = []  # tools/test.py, loaded once


@unittest.skipUnless(TEST_PY.is_file(), "the runners are not part of this source tree")
class RunnerCase(unittest.TestCase):
    """A scratch folder per test, and the runner modules."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="iad-runners-"))
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    @staticmethod
    def tool():  # noqa: ANN205 (the module tools/test.py)
        if not _TOOL:
            _TOOL.append(load("iad_tools_test", TEST_PY))
        return _TOOL[0]


class PathMapTest(RunnerCase):
    def suites(self, path: str) -> set[str]:
        found = self.tool().suites_for(path)
        self.assertIsNotNone(found, f"no rule names {path}")
        return found

    def test_the_runners_select_every_suite_that_runs_through_them(self) -> None:
        # tools/unittest_jobs.py runs the gui and hw suites and imports run_backend's planner and time list
        self.assertTrue({"gui", "hw"} <= self.suites("tools/unittest_jobs.py"))
        self.assertTrue({"python", "gui", "hw"} <= self.suites("bindings/python/tests/run_backend.py"))
        self.assertIn("core", self.suites("core/tests/run_shards.py"))

    def test_a_text_file_anywhere_selects_the_wording_and_version_checks(self) -> None:
        for path in ("core/CMakeLists.txt", "core/tests/run_shards.py", "tools/hw/README.md", "tools/unittest_jobs.py",
                     "tools/hw/tests/test_times.txt", "core/build.py"):  # fmt: skip
            self.assertIn("docs", self.suites(path), path)
        self.assertEqual(self.suites("core/tests/test_cell.cpp"), {"core"})  # not a text file for those checks

    def test_the_release_suite_runs_for_the_release_scripts_and_the_builds_they_use(self) -> None:
        for path in (".github/release/local_release.py", ".github/release/test_local_release.py",
                     ".github/release/sdk/CMakeLists.txt", ".github/workflows/release.yml", ".github/workflows/ci.yml",
                     "core/CMakeLists.txt", "core/build.py"):  # fmt: skip
            self.assertIn("release", self.suites(path), path)
            self.assertIn("docs", self.suites(path), path)  # what these paths selected before the suite existed
        self.assertIn("core", self.suites("core/CMakeLists.txt"))
        self.assertGreaterEqual(self.suites("core/build.py"), self.tool().NATIVE)
        for path in ("core/src/foo.cpp", "core/tests/test_cell.cpp", "docs/DRIVER.md", "examples/python/x.py",
                     "tools/test.py", "apps/gui/server.py", "bindings/dotnet/src/IaiActuatorDriver/Foo.cs"):  # fmt: skip
            self.assertNotIn("release", self.suites(path), path)

    def test_the_release_suite_runs_for_the_files_its_tests_read(self) -> None:
        """test_local_release.py reads the notices, the GUI spec and the NuGet project: a change to one of them must
        run it (and the suite of the part it belongs to), not only the broad rule of its folder."""
        for path, suite in (("apps/gui/packaging/THIRD_PARTY_NOTICES.txt", "gui"), ("apps/gui/packaging/README.txt", "gui"),
                            ("apps/gui/iai-gui.spec", "gui"),
                            ("bindings/dotnet/src/IaiActuatorDriver/IaiActuatorDriver.csproj", "dotnet"),
                            ("LICENSE", "docs"), ("licenses/NOTICE", "docs"),
                            ("licenses/NATIVE_THIRD_PARTY.txt", "docs")):  # fmt: skip
            self.assertTrue({suite, "docs", "release"} <= self.suites(path), path)

    def test_the_release_suite_runs_the_tests_of_the_release_scripts_and_needs_no_build(self) -> None:
        tool = self.tool()
        suite = tool.make_suite("release", 1, self.tmp)
        script = ROOT / ".github" / "release" / "test_local_release.py"
        self.assertEqual(suite.cmd, [sys.executable, str(script)])
        self.assertTrue(script.is_file())
        self.assertNotIn("release", tool.PARALLEL)
        text = "......\n" + "-" * 70 + "\nRan 39 tests in 13.610s\n\nOK\n"
        self.assertEqual(tool.parse_counts("release", text), "39 tests, 39 passed, 0 skipped, 0 failed")
        text = "Ran 39 tests in 13.610s\n\nFAILED (failures=2, errors=1, skipped=3, expected failures=4)\n"
        self.assertEqual(tool.parse_counts("release", text), "39 tests, 33 passed, 3 skipped, 3 failed")
        self.assertEqual(tool.parse_counts("release", "Ran 1 test in 0.001s\n\nOK (skipped=1)\n"),
                         "1 tests, 0 passed, 1 skipped, 0 failed")  # fmt: skip
        self.assertEqual(tool.parse_counts("release", "Traceback: no tests ran\n"), "")

    def test_a_load_sensitive_test_that_passed_only_on_its_retry_is_named(self) -> None:
        tool = self.tool()  # run_shards.py (core), then run_backend.py and unittest_jobs.py
        text = ("12 tests, 12 passed, 0 failed, 40 checks, 3.1 s\n"
                "  passed on its retry: axis_fast_stop_stays_immediate\n"
                "passed on its retry (load-sensitive): test_cell.ScanTest.test_scan\n")  # fmt: skip
        self.assertEqual(tool.retried_tests(text), ["axis_fast_stop_stays_immediate", "test_cell.ScanTest.test_scan"])
        self.assertEqual(tool.retried_tests("ran 3 tests in 0.1 s: 3 passed\n"), [])

    def test_the_wording_suffixes_are_those_test_packaging_scans(self) -> None:
        self.assertEqual(self.tool().WORDING_SUFFIXES, test_packaging.TEXT_SUFFIXES)

    @unittest.skipUnless(SERVER_PY.is_file(), "no GUI in this source tree")
    def test_every_document_the_gui_serves_selects_the_gui_suite(self) -> None:
        tree = ast.parse(SERVER_PY.read_text(encoding="utf-8"))
        assigned = [n for n in tree.body
                    if isinstance(n, ast.AnnAssign) and getattr(n.target, "id", "") == "ABOUT_DOCS"]
        self.assertEqual(len(assigned), 1)
        documents = [row[2] for row in ast.literal_eval(assigned[0].value)]
        self.assertGreaterEqual(len(documents), 5)
        for rel in documents:
            self.assertTrue({"gui", "docs"} <= self.suites(rel), rel)


class SelectTest(RunnerCase):
    """What --all, --suites and --changed choose: select() on a stand-in for the changed paths."""

    def select(self, found: tuple[str, list[str]] | None, **values) -> list[str]:
        tool = self.tool()
        args = argparse.Namespace(**{"all": False, "suites": None, "changed": None, "verbose": False, **values})
        with mock.patch.object(tool, "changed_paths", return_value=found), redirect_stdout(io.StringIO()):
            return tool.select(args)[0]

    def test_all_runs_every_suite(self) -> None:
        every = list(self.tool().ALL)
        self.assertEqual(self.select(("base", ["README.md"]), all=True), every)
        self.assertEqual(self.select(None, all=True), every)  # git cannot say
        for suite in ("release", "hw", "docs", "dotnet", "smoke", "python", "gui", "core"):
            self.assertIn(suite, every)
        self.assertNotIn("pure", every)  # the pure-Python edition was removed after 1.0.x, and its suite with it

    def test_the_help_and_the_docstring_say_what_all_runs(self) -> None:
        said = "--all every suite"
        self.assertIn(said, " ".join(self.tool().__doc__.split()))
        proc = subprocess.run([sys.executable, str(TEST_PY), "--help"], cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn(said, " ".join(proc.stdout.split()))

    def test_a_change_to_the_release_scripts_selects_their_tests(self) -> None:
        self.assertEqual(self.select(("base", [".github/release/local_release.py"])), ["docs", "release"])
        self.assertEqual(self.select(("base", ["core/CMakeLists.txt"])), ["core", "docs", "release"])
        self.assertIn("release", self.select(("base", ["core/build.py"])))
        self.assertNotIn("release", self.select(("base", ["core/src/foo.cpp"])))

    def test_the_release_suite_can_be_named_and_a_dry_run_shows_its_command(self) -> None:
        self.assertEqual(self.select(None, suites="release"), ["release"])
        build = self.tmp / "build"
        build.mkdir()
        env = dict(os.environ, IAD_BUILD_DIR=str(build))
        proc = subprocess.run([sys.executable, str(TEST_PY), "--dry-run", "--suites", "release"], cwd=ROOT, env=env,
                              text=True, capture_output=True)  # fmt: skip
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("suites: release", proc.stdout)
        self.assertIn("test_local_release.py", proc.stdout)
        self.assertEqual(list(build.iterdir()), [], "a dry run left folders in the build folder")


class ChangedPathsTest(RunnerCase):
    """A moved file lists its old path too: the suites of the place it left run as well."""

    def git(self, *args: str) -> str:
        env = dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@example.test", GIT_COMMITTER_NAME="t",
                   GIT_COMMITTER_EMAIL="t@example.test")  # fmt: skip
        proc = subprocess.run(["git", "-c", "commit.gpgsign=false", *args], cwd=self.tmp, env=env, text=True,
                              capture_output=True)  # fmt: skip
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout.strip()

    def setUp(self) -> None:
        super().setUp()
        if shutil.which("git") is None:
            self.skipTest("git is not installed")
        self.git("init", "-q")
        for rel in ("bindings/python/iai_actuator_driver/helper.py", "bindings/python/tests/test_cell.py",
                    "core/src/foo.cpp"):  # fmt: skip
            path = self.tmp / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("x = 1\n" * 20, encoding="utf-8")
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "start")
        self.base = self.git("rev-parse", "HEAD")

    def move(self, old: str, new: str) -> tuple[list[str], set[str]]:
        (self.tmp / new).parent.mkdir(parents=True, exist_ok=True)
        self.git("mv", old, new)
        found = self.tool().changed_paths(self.base, self.tmp)
        self.assertIsNotNone(found)
        paths = found[1]
        suites: set[str] = set()
        for path in paths:
            suites |= self.tool().suites_for(path) or set(self.tool().ALL)
        return paths, suites

    def test_a_module_that_leaves_the_package_still_selects_its_suites(self) -> None:
        paths, suites = self.move("bindings/python/iai_actuator_driver/helper.py", "apps/gui/helper.py")
        self.assertEqual(paths, ["apps/gui/helper.py", "bindings/python/iai_actuator_driver/helper.py"])
        self.assertTrue({"python", "gui", "hw"} <= suites, suites)

    def test_a_test_that_moves_to_another_folder_selects_the_python_suites(self) -> None:
        paths, suites = self.move("bindings/python/tests/test_cell.py", "tools/hw/tests/test_cell.py")
        self.assertIn("bindings/python/tests/test_cell.py", paths)
        self.assertTrue({"python", "hw"} <= suites, suites)

    def test_a_source_file_that_moves_into_docs_selects_the_core_suites(self) -> None:
        paths, suites = self.move("core/src/foo.cpp", "docs/foo.cpp")
        self.assertEqual(paths, ["core/src/foo.cpp", "docs/foo.cpp"])
        self.assertTrue({"core", "python", "dotnet"} <= suites, suites)


class RunSuitesTest(RunnerCase):
    def test_the_time_limit_ends_the_suite_and_everything_it_started(self) -> None:
        tool = self.tool()
        pidfile = self.tmp / "child.pid"
        script = self.tmp / "runner.py"
        script.write_text(
            "import os, pathlib, subprocess, sys, time\n"
            "child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)'])\n"
            f"pathlib.Path({str(pidfile) + '.tmp'!r}).write_text(str(child.pid))\n"
            f"os.replace({str(pidfile) + '.tmp'!r}, {str(pidfile)!r})\n"
            "time.sleep(120)\n", encoding="utf-8")  # fmt: skip
        suite = tool.Suite("hang", [sys.executable, str(script)], self.tmp, dict(os.environ), log=self.tmp / "hang.log")

        class Clock:  # the run_suites' clock jumps past the limit as soon as the runner has started its child
            sleep = staticmethod(time.sleep)

            @staticmethod
            def monotonic() -> float:
                return time.monotonic() + (1e6 if pidfile.is_file() else 0.0)

        try:
            with redirect_stdout(io.StringIO()), mock.patch.object(tool, "time", Clock):
                tool.run_suites([suite], 600.0)
            self.assertTrue(suite.timed_out)
            self.assertNotEqual(suite.code, 0)
            child = int(pidfile.read_text())
            deadline = time.monotonic() + 15
            while pid_alive(child) and time.monotonic() < deadline:
                time.sleep(0.1)
            self.assertFalse(pid_alive(child), "the child of the killed runner is still running")
        finally:
            if pidfile.is_file():
                kill_pid(int(pidfile.read_text()))

    def test_a_dry_run_creates_nothing(self) -> None:
        build = self.tmp / "build"
        build.mkdir()
        env = dict(os.environ, IAD_BUILD_DIR=str(build))
        proc = subprocess.run([sys.executable, str(TEST_PY), "--dry-run", "--suites", "core,docs"], cwd=ROOT, env=env,
                              text=True, capture_output=True)  # fmt: skip
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("core (", proc.stdout)
        self.assertEqual(list(build.iterdir()), [], "a dry run left folders in the build folder")


class WorkerShareTest(RunnerCase):
    def test_the_workers_go_where_the_slowest_suite_is(self) -> None:
        tool = self.tool()
        every = [n for n in tool.ALL]
        parallel = [n for n in every if n in tool.PARALLEL]
        for budget in range(len(parallel), 13):
            shares = tool.share_workers(every, budget)
            self.assertEqual(sum(shares[n] for n in parallel), budget, shares)
            for n in every:
                self.assertTrue(1 <= shares[n] <= tool.MAX_WORKERS, shares)
            slowest = max(tool.WEIGHT[n] / shares[n] for n in parallel)
            for give in parallel:  # no single move of a worker makes the slowest suite faster
                for take in parallel:
                    if give != take and shares[take] > 1 and shares[give] < tool.MAX_WORKERS:
                        moved = dict(shares, **{give: shares[give] + 1, take: shares[take] - 1})
                        self.assertGreaterEqual(max(tool.WEIGHT[n] / moved[n] for n in parallel), slowest - 1e-9)

    def test_the_default_budget_is_two_thirds_of_the_cpus_up_to_eight(self) -> None:
        tool = self.tool()
        for cpus, expected in ((None, 1), (1, 1), (2, 1), (4, 2), (8, 5), (12, 8), (16, 8), (64, 8)):
            with mock.patch.object(tool.os, "cpu_count", return_value=cpus):
                self.assertEqual(tool.default_budget(), expected, cpus)

    def test_the_default_budget_gives_the_core_tests_more_than_one_process(self) -> None:
        tool = self.tool()
        with mock.patch.object(tool.os, "cpu_count", return_value=12):
            shares = tool.share_workers([n for n in tool.ALL], tool.default_budget())
        self.assertGreater(shares["core"], 1, shares)
        self.assertGreater(shares["python"], 1, shares)


class UnittestJobsTest(RunnerCase):
    def run_jobs(self, *tests: tuple[str, str], jobs: int = 2, load_sensitive: str = "") -> tuple[int, str]:
        """Run the runner on scratch test files; ``load_sensitive``: the text of its load-sensitive list (suite
        ``scratch``)."""
        folder = self.tmp / "suite"
        folder.mkdir()
        for name, text in tests:
            (folder / name).write_text(text, encoding="utf-8")
        env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
        cmd = [sys.executable, str(UNITTEST_JOBS), "-s", str(folder), "-t", str(folder), "--jobs", str(jobs),
               "--timeout", "60"]  # fmt: skip
        if load_sensitive:
            (self.tmp / "load_sensitive.txt").write_text(load_sensitive, encoding="utf-8")
            cmd += ["--load-sensitive", "scratch", "--load-sensitive-file", str(self.tmp / "load_sensitive.txt")]
        proc = subprocess.run(cmd, cwd=self.tmp, env=env, text=True, capture_output=True)
        # stderr first: a warning printed at import (a DeprecationWarning, say) must not come after the runner's last
        # line, which the tests look at
        return proc.returncode, proc.stderr + proc.stdout

    OK = "import unittest\nclass T(unittest.TestCase):\n    def test_a(self):\n        pass\n"
    CRASH = ("import atexit, os, unittest\nclass T(unittest.TestCase):\n    def test_a(self):\n"
             "        atexit.register(os._exit, 3)\n")  # fmt: skip
    FAIL = "import unittest\nclass T(unittest.TestCase):\n    def test_a(self):\n        self.fail('no')\n"
    # fails the first time in a scratch folder, passes once its mark is there (the tests run in the runner's cwd)
    FLAKY = ("import unittest\nfrom pathlib import Path\nMARK = Path(__file__).with_name('ran-once')\n"
             "class T(unittest.TestCase):\n    def test_a(self):\n        first = not MARK.exists()\n"
             "        MARK.touch()\n        self.assertFalse(first, 'the first run fails')\n")  # fmt: skip

    def test_a_clean_run_passes(self) -> None:
        code, text = self.run_jobs(("test_ok.py", self.OK))
        self.assertEqual(code, 0, text)
        self.assertTrue(text.rstrip().endswith("OK"), text)

    def test_a_failed_test_fails_the_run(self) -> None:
        code, text = self.run_jobs(("test_fail.py", self.FAIL))
        self.assertEqual(code, 1, text)
        self.assertTrue(text.rstrip().endswith("FAILED"), text)

    def test_a_worker_that_crashes_after_writing_its_result_fails_the_run(self) -> None:
        code, text = self.run_jobs(("test_crash.py", self.CRASH), ("test_ok.py", self.OK))
        self.assertNotEqual(code, 0, text)
        self.assertIn("ended with exit code 3 after its result", text)
        self.assertTrue(text.rstrip().endswith("FAILED"), text)

    def test_a_load_sensitive_test_runs_after_the_workers_and_passes_on_its_retry(self) -> None:
        code, text = self.run_jobs(("test_flaky.py", self.FLAKY), ("test_ok.py", self.OK),
                                   load_sensitive="scratch test_flaky.T.test_a  measures a time\n")  # fmt: skip
        self.assertEqual(code, 0, text)
        self.assertIn("then alone: test_flaky.T.test_a", text)
        self.assertIn("tail (load-sensitive): 1 tests", text)
        self.assertIn("retry: 1 tests", text)
        self.assertIn("ran 2 tests", text)  # the retry is not a second test
        self.assertIn("2 passed", text)
        self.assertIn("passed on its retry (load-sensitive): test_flaky.T.test_a", text)
        self.assertTrue(text.rstrip().endswith("OK"), text)

    def test_a_load_sensitive_test_that_fails_twice_fails_the_run(self) -> None:
        code, text = self.run_jobs(("test_fail.py", self.FAIL), ("test_ok.py", self.OK),
                                   load_sensitive="scratch test_fail.T.test_a  measures a time\n")  # fmt: skip
        self.assertEqual(code, 1, text)
        self.assertIn("retry: 1 tests", text)
        self.assertIn("ran 2 tests", text)
        self.assertIn("1 failures", text)
        self.assertNotIn("passed on its retry", text)
        self.assertTrue(text.rstrip().endswith("FAILED"), text)

    def test_no_test_discovered_is_a_failure_as_in_unittest(self) -> None:
        code, text = self.run_jobs(("gui_tests.py", self.OK))  # a file the test*.py pattern does not match
        self.assertEqual(code, 5, text)
        self.assertIn("no test discovered", text)
        self.assertTrue(text.rstrip().endswith("FAILED"), text)


FAKE_EXE = '''\
import os, sys, time
from pathlib import Path
NAMES = [f"t{i:02d}" for i in range(1, 11)]
marks = Path(os.environ["FAKE_MARKS"])  # files that tell the test where this process has got to
args = sys.argv[1:]
if args and args[0] == "-l":
    print("\\n".join(NAMES))
    sys.exit(0)
k, n = 1, 1
if "--shard" in args:
    k, n = map(int, args.pop(args.index("--shard") + 1).split("/"))
excluded = [args.pop(i + 1) for i in reversed(range(len(args))) if args[i] == "--exclude"]
named = [a for a in args if not a.startswith("-")]  # with --exact: whole names
selected = [t for t in NAMES if (not named or t in named) and t not in excluded]
mode = os.environ.get("FAKE_MODE", "")
passed = failed = 0
for name in selected[k - 1::n]:
    if mode == "hang:" + name:
        (marks / "hang").touch()
        time.sleep(120)
    if mode == "crash:" + name:
        os._exit(3)
    once = mode == "flaky:" + name and not (marks / name).exists()  # fails its first run only
    if mode == "fail:" + name or once:
        (marks / name).touch()
        print(f"  FAIL {name} 0.01 s", flush=True)
        failed += 1
        continue
    if "--quiet" not in args or "--shard" in args:
        print(f"  ok   {name} 0.01 s", flush=True)
    passed += 1
print(f"\\n{passed + failed} tests, {passed} passed, {failed} failed, {passed} checks, 0.01 s", flush=True)
(marks / f"done{k}").touch()
sys.exit(1 if failed else 0)
'''


class ShardsTest(RunnerCase):
    """core/tests/run_shards.py on a stand-in for ec_tests: a script that lists ten tests, runs its shard's tests in
    order and, by FAKE_MODE, hangs or crashes at one of them."""

    def setUp(self) -> None:
        super().setUp()
        self.script = self.tmp / "fake_ec_tests.py"
        self.script.write_text(FAKE_EXE, encoding="utf-8")
        self.shards = load("iad_run_shards", RUN_SHARDS)
        real = subprocess

        class Fake:  # the runner's own view of subprocess: the executable it starts is the script
            TimeoutExpired, STDOUT, PIPE = real.TimeoutExpired, real.STDOUT, real.PIPE

            @staticmethod
            def Popen(cmd, **kw):  # noqa: N802
                return real.Popen([sys.executable, str(self.script), *cmd[1:]], **kw)

            @staticmethod
            def run(cmd, **kw):
                return real.run([sys.executable, str(self.script), *cmd[1:]], **kw)

        patch = mock.patch.object(self.shards, "subprocess", Fake)
        patch.start()
        self.addCleanup(patch.stop)

    def run_shards(self, mode: str, shards: int, timeout_s: float = 600.0, past: tuple[str, ...] = (),
                   tail: tuple[str, ...] = ()) -> tuple[int, str]:  # fmt: skip
        """Run the stand-in. ``past``: marker files (see FAKE_EXE); once they all exist the runner's clock jumps beyond
        its time limit, so a hang is cut off at once and the shards that are fine have ended by then."""
        marks = self.tmp / "marks"
        marks.mkdir(exist_ok=True)

        class Clock:
            sleep = staticmethod(time.sleep)

            @staticmethod
            def monotonic() -> float:
                return time.monotonic() + (1e6 if past and all((marks / n).exists() for n in past) else 0.0)

        out = io.StringIO()
        with mock.patch.dict(os.environ, FAKE_MODE=mode, FAKE_MARKS=str(marks)), \
                mock.patch.object(self.shards, "time", Clock):  # fmt: skip
            code = self.shards.run(self.script, shards=shards, quiet=True, timeout_s=timeout_s, cwd=self.tmp, out=out,
                                   tail=tail)  # fmt: skip
        return code, out.getvalue()

    def test_a_clean_run_counts_every_test(self) -> None:
        code, text = self.run_shards("", 3)
        self.assertEqual(code, 0, text)
        self.assertIn("10 tests, 10 passed, 0 failed", text)

    def test_a_crashed_shard_keeps_the_tests_it_passed_in_the_totals_and_names_the_test_it_was_in(self) -> None:
        code, text = self.run_shards("crash:t07", 3)  # shard 1 runs t01 t04 t07 t10: t01 and t04 passed first
        self.assertEqual(code, 1, text)
        self.assertIn("8 tests, 8 passed, 0 failed", text)  # 8 reported + the 2 below = the 10 selected
        self.assertIn("2 of 10 selected tests did not run: t07, t10", text)
        self.assertIn("shard 1/3: ended without a summary (exit code 3) while running t07", text)
        self.assertIn("3 shards: FAILED", text)

    def test_a_shard_past_the_time_limit_names_the_test_it_was_running(self) -> None:
        # shard 2 runs t02 t05 t08 and never gets past t02; the time limit comes once shards 1 and 3 have ended
        code, text = self.run_shards("hang:t02", 3, past=("hang", "done1", "done3"))
        self.assertEqual(code, 1, text)
        self.assertRegex(text, r"shard 2/3: killed after the time limit \(\d+ s\) while running t02")
        self.assertIn("7 tests, 7 passed, 0 failed", text)  # the other two shards'
        self.assertIn("3 of 10 selected tests did not run: t02, t05, t08", text)

    def test_the_tail_runs_after_the_shards_and_a_test_that_fails_once_passes_on_its_retry(self) -> None:
        code, text = self.run_shards("flaky:t05", 3, tail=("t05", "t09"))
        self.assertEqual(code, 0, text)
        self.assertIn("8 tests in 3 shards (--shards 1: one process), then 2 alone (load-sensitive)", text)
        self.assertIn("10 tests, 10 passed, 0 failed", text)  # t05 once: its retry is not a second test
        self.assertIn("passed on its retry: t05", text)
        self.assertIn("3 shards and the tail: OK", text)

    def test_a_tail_test_that_fails_twice_fails_the_run(self) -> None:
        code, text = self.run_shards("fail:t05", 3, tail=("t05",))
        self.assertEqual(code, 1, text)
        self.assertIn("10 tests, 9 passed, 1 failed", text)
        self.assertIn("failed: t05", text)
        self.assertNotIn("passed on its retry", text)
        self.assertIn("3 shards and the tail: FAILED", text)

    def test_one_process_is_held_to_the_time_limit_too(self) -> None:
        started = time.monotonic()
        code, text = self.run_shards("hang:t01", 1, timeout_s=1.0)
        self.assertEqual(code, 1, text)
        self.assertIn("killed after the time limit (1 s)", text)
        self.assertLess(time.monotonic() - started, 60)

    def test_one_process_without_a_limit_or_a_hang_returns_its_exit_code(self) -> None:
        code, _ = self.run_shards("", 1)
        self.assertEqual(code, 0)
        code, _ = self.run_shards("crash:t03", 1)
        self.assertEqual(code, 3)


class LoadSensitiveTest(RunnerCase):
    """tools/load_sensitive.txt: every entry names a test of its suite, and run_backend's planner and retry."""

    def setUp(self) -> None:
        super().setUp()
        self.rb = load("iad_run_backend_tail", RUN_BACKEND)

    def test_the_list_is_read_per_suite_in_its_order(self) -> None:
        path = self.tmp / "load_sensitive.txt"
        path.write_text("# a comment\n\ncore b_test  a reason\npython m.C.test_x  another reason\ncore a_test  r\n",
                        encoding="utf-8")  # fmt: skip
        self.assertEqual(list(self.rb.read_load_sensitive("core", path)), ["b_test", "a_test"])
        self.assertEqual(self.rb.read_load_sensitive("python", path), {"m.C.test_x": "another reason"})
        self.assertEqual(self.rb.read_load_sensitive("python", self.tmp / "missing.txt"), {})

    def test_every_entry_names_a_test_of_its_suite_and_gives_a_reason(self) -> None:
        def text(folder: Path, pattern: str) -> str:
            return "".join(f.read_text(encoding="utf-8") for f in sorted(folder.rglob(pattern)))

        found = {  # suite: the sources of its tests, and how one of them names a test
            "core": (text(ROOT / "core" / "tests", "test_*.cpp"), "TEST({})"),
            "python": (text(ROOT / "bindings" / "python" / "tests", "test*.py"), "def {}("),
            "gui": (text(ROOT / "apps" / "gui", "test*.py"), "def {}("),
            "dotnet": (text(ROOT / "bindings" / "dotnet" / "tests", "*Tests.cs"), " {}("),
        }
        listed = 0
        for suite, (source, form) in found.items():
            for name, reason in self.rb.read_load_sensitive(suite, LOAD_SENSITIVE).items():
                listed += 1
                self.assertTrue(reason, name)
                self.assertIn(form.format(name.rsplit(".", 1)[-1]), source, f"{suite} {name}: no such test")
        self.assertGreater(listed, 0)

    def test_the_planner_runs_a_listed_test_alone_after_the_workers(self) -> None:
        class Sample(unittest.TestCase):
            def test_a(self) -> None:
                pass

            def test_b(self) -> None:
                pass

        tests = [Sample("test_a"), Sample("test_b")]
        names = [self.rb.name_of(t) for t in tests]
        workers, alone = self.rb.plan(tests, 2, {}, {names[1]: "a reason"})
        self.assertEqual(alone, [names[1]])
        self.assertEqual(workers, [[names[0]]])  # by its name: its module or class would load the listed test too

    def test_a_retry_replaces_the_first_outcome_and_runs_no_second_test(self) -> None:
        first = [{"ran": 2, "loaded": ["m.T.a", "m.T.b"], "passed": ["m.T.a"], "failures": ["m.T.b"], "errors": []}]
        retry = {"ran": 1, "loaded": ["m.T.b"], "passed": ["m.T.b"], "failures": [], "errors": [], "durations": [[1.0,
                 "m.T.b"]], "walls": [[1.0, "m.T.b"]], "skipped": []}  # fmt: skip
        added = self.rb.take_retry(first, retry)
        self.assertEqual(first[0]["failures"], [])
        self.assertEqual((added["ran"], added["loaded"], added["passed"], added["durations"]), (0, [], ["m.T.b"], []))


if __name__ == "__main__":
    unittest.main()
