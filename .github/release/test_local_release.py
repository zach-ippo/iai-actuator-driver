"""Checks of the release scripts that need neither Windows, WSL nor a build: the choice of targets and the WSL kit path in
local_release.py, its --resume across a change of .github/release/, the gui step's helpers and its agreement with
release.yml, the SDK smoke steps with the frozen ABI 1.0 C program, the README of the SDK (make_sdk.py: the compile line
of the Linux one, the pointer to the hardware evidence), the path filters of ci.yml, and LICENSE and NOTICE and
NATIVE_THIRD_PARTY.txt in every kind of release asset (make_sdk.py, make_gui_zip.py, check_package.py,
collect_release.py), and the monotonic clock of the Linux library's timed waits (check_package.py's gate, the core's
sources).

    python -m unittest discover -s .github/release -p "test_*.py"        (from the repository root)
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import http.server
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import unittest
import zipfile
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import check_package  # noqa: E402
import collect_release  # noqa: E402
import local_release  # noqa: E402
import make_gui_zip  # noqa: E402
import make_sdk  # noqa: E402


class FakeWslList:
    """subprocess.run for `wsl.exe --list --quiet`: one distro, Ubuntu-24.04."""

    returncode = 0
    stdout = b"Ubuntu-24.04\r\n"


def args_of(**values) -> argparse.Namespace:
    base = {"targets": None, "wsl_kit": "kit", "wsl_distro": "Ubuntu-24.04"}
    return argparse.Namespace(**{**base, **values})


@contextlib.contextmanager
def in_folder(path: Path):
    before = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(before)


class WslKit(unittest.TestCase):
    def choose(self, **values):
        with mock.patch.object(local_release.shutil, "which", lambda name: name), \
                mock.patch.object(local_release.subprocess, "run", lambda *a, **k: FakeWslList()):
            return local_release.choose_targets(args_of(**values))

    def test_relative_kit_is_resolved_once(self) -> None:
        """A relative --wsl-kit (or IAD_LINUX_VM) found run.ps1 from the caller's folder, but run_wsl starts powershell.exe in
        <out>/work, where the relative path does not exist."""
        with tempfile.TemporaryDirectory() as tmp, tempfile.TemporaryDirectory() as elsewhere:
            kit = Path(tmp) / "kit"
            kit.mkdir()
            (kit / "run.ps1").write_text("", encoding="utf-8")
            for targets in (None, local_release.LINUX_RID):
                with in_folder(Path(tmp)):
                    found, wsl, notes = self.choose(wsl_kit="kit", targets=targets)
                self.assertIn(local_release.LINUX_RID, found)
                self.assertIsNotNone(wsl)
                self.assertEqual(notes, [])
                self.assertTrue(wsl.kit.is_absolute(), wsl.kit)
                self.assertEqual(wsl.kit, kit.resolve())
                with in_folder(Path(elsewhere)):  # where run_wsl runs powershell.exe
                    self.assertTrue(wsl.run_ps1.is_file(), wsl.run_ps1)

    def test_missing_kit_says_so(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, in_folder(Path(tmp)):
            found, wsl, notes = self.choose(wsl_kit="no-such-kit")
            self.assertEqual(found, [local_release.RID])
            self.assertIsNone(wsl)
            self.assertIn("no run.ps1", notes[0])
            with self.assertRaises(local_release.Failure):
                self.choose(wsl_kit="no-such-kit", targets=local_release.LINUX_RID)


class Targets(unittest.TestCase):
    def test_a_list_without_a_target_is_refused(self) -> None:
        """--targets "," once passed the argument check, chose no target and ended in a KeyError at the collect step."""
        for value in ("", ",", ",,"):
            err = io.StringIO()
            with contextlib.redirect_stderr(err), self.assertRaises(SystemExit) as raised:
                local_release.main(["--targets", value])
            self.assertEqual(raised.exception.code, 2, value)
            self.assertIn("names no target", err.getvalue(), value)

    def test_an_unknown_target_is_refused(self) -> None:
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit) as raised:
            local_release.main(["--targets", "win-x64,linux-arm64"])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("linux-arm64", err.getvalue())

    def test_choose_targets_does_not_return_an_empty_list(self) -> None:
        for value in ("", ","):
            with self.assertRaises(local_release.Failure):
                local_release.choose_targets(args_of(targets=value))

    def test_windows_only_needs_no_wsl(self) -> None:
        found, wsl, notes = local_release.choose_targets(args_of(targets=local_release.RID + ","))
        self.assertEqual((found, wsl, notes), ([local_release.RID], None, []))


def make_ctx(out: Path, commit: str, targets: list[str] | None = None) -> local_release.Ctx:
    return local_release.Ctx(
        args=argparse.Namespace(verbose=False, gui_tests=False), commit=commit, branch="main", version="1.0.0", nuget_version="1.0.0",
        tag="v1.0.0", prerelease=False, out=out, tools=out.parent / "_tools", python=sys.executable,
        zig_version="0.16.0", dotnet_version="8.0.x", targets=targets or [local_release.RID], wsl=None,
        pyinstaller_version="6.22.3", release_python="3.12")


def git_in(repo: Path, *args: str) -> str:
    cmd = ["git", "-c", "user.name=Test", "-c", "user.email=test@example.invalid", "-c", "commit.gpgsign=false", *args]
    return subprocess.run(cmd, cwd=repo, check=True, capture_output=True, text=True).stdout.strip()


def make_repo(repo: Path) -> tuple[str, str, str]:
    """A repository of three commits: c1; c2 changes .github/release/ only; c3 changes README.md."""
    repo.mkdir(parents=True)
    git_in(repo, "init", "-q")

    def commit(message: str) -> str:
        git_in(repo, "add", "-A")
        git_in(repo, "commit", "-q", "-m", message)
        return git_in(repo, "rev-parse", "HEAD")

    (repo / ".github" / "release").mkdir(parents=True)
    (repo / ".github" / "release" / "a.py").write_text("old\n", encoding="utf-8")
    (repo / "README.md").write_text("one\n", encoding="utf-8")
    c1 = commit("c1")
    (repo / ".github" / "release" / "a.py").write_text("new\n", encoding="utf-8")
    (repo / ".github" / "release" / "sdk").mkdir()
    (repo / ".github" / "release" / "sdk" / "x.txt").write_text("x\n", encoding="utf-8")
    c2 = commit("c2")
    (repo / "README.md").write_text("two\n", encoding="utf-8")
    return c1, c2, commit("c3")


class ResumeAcrossScripts(unittest.TestCase):
    """--resume (and --only, --skip) go on with the steps that passed when the only change since the recorded commit is in
    .github/release/ (the fix of the script that failed); any other change needs a clean run."""

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.repo = self.tmp / "repo"
        self.c1, self.c2, self.c3 = make_repo(self.repo)
        patcher = mock.patch.object(local_release, "REPO", self.repo)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.out = self.tmp / "out"
        self.save_run(self.c1)

    def save_run(self, commit: str, targets: list[str] | None = None, built: str | None = None) -> None:
        """An output folder of a run at `commit` that passed export; its work/src is the export of that commit."""
        local_release.rmtree(self.out)
        (self.out / "work" / "src" / ".github" / "release").mkdir(parents=True)
        (self.out / "work" / "src" / "core").mkdir()
        (self.out / "work" / "src" / ".github" / "release" / "a.py").write_text("old\n", encoding="utf-8")
        (self.out / "work" / "src" / "core" / "keep.txt").write_text("built from this\n", encoding="utf-8")
        (self.out / "work" / "src.commit").write_text(commit + "\n", encoding="utf-8")
        (self.out / local_release.MARKER).write_text("1.0.0 " + commit + "\n", encoding="utf-8")
        state = {"commit": commit, "built": built or commit, "targets": targets or [local_release.RID],
                 "steps": {"export": {"detail": "1 files", "secs": 1.0, "facts": []}}}
        (self.out / local_release.STATE).write_text(json.dumps(state), encoding="utf-8")

    def prepare(self, commit: str, keep: bool = True, targets: list[str] | None = None) -> local_release.Ctx:
        ctx = make_ctx(self.out, commit, targets)
        with contextlib.redirect_stdout(io.StringIO()):
            local_release.prepare_out(ctx, keep)
        return ctx

    def test_only_release_scripts_changed_is_carried_over(self) -> None:
        ctx = self.prepare(self.c2)
        self.assertIn("export", ctx.done)  # the recorded steps are kept
        self.assertEqual(ctx.built, self.c1)
        self.assertEqual(ctx.built_at, self.c1)
        self.assertEqual(len(ctx.notes), 1)
        self.assertIn(self.c1[:7], ctx.notes[0])
        self.assertIn(self.c2[:7], ctx.notes[0])
        src = self.out / "work" / "src"
        self.assertEqual((src / ".github" / "release" / "a.py").read_text(encoding="utf-8"), "new\n")  # the fixed script
        self.assertTrue((src / ".github" / "release" / "sdk" / "x.txt").is_file())
        self.assertTrue((src / "core" / "keep.txt").is_file())  # everything else is left as it was
        self.assertEqual(local_release.need_src(ctx), src)  # now the export of the new commit, as every step asks
        local_release.save_state(ctx)
        saved = json.loads((self.out / local_release.STATE).read_text(encoding="utf-8"))
        self.assertEqual((saved["commit"], saved["built"]), (self.c2, self.c1))

    def test_the_next_resume_still_says_where_the_steps_were_built(self) -> None:
        self.prepare(self.c2)  # no save_state here: a resume that runs no step never calls it (main does, per step)
        again = self.prepare(self.c2)
        self.assertEqual(again.built, self.c1)
        self.assertEqual(len(again.notes), 1)
        self.assertIn("export", again.done)

    def commit_on(self, base: str, name: str, text: str) -> str:
        """A commit on top of `base` that changes one file (in .github/release/ or not), on a side branch."""
        git_in(self.repo, "checkout", "-q", "-B", "side", base)
        path = self.repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        git_in(self.repo, "add", "-A")
        git_in(self.repo, "commit", "-q", "-m", "side")
        return git_in(self.repo, "rev-parse", "HEAD")

    def test_scripts_only_commits_carry_over_one_after_the_other(self) -> None:
        """A --resume that runs no step (every step passed earlier) still records the commit it carried the steps to, so
        the next change of .github/release/ is carried over too, not refused as 'not an export of' it."""
        c4 = self.commit_on(self.c2, ".github/release/a.py", "newest\n")
        self.prepare(self.c2)  # --resume at c2: nothing runs, nothing else saves the state
        saved = json.loads((self.out / local_release.STATE).read_text(encoding="utf-8"))
        self.assertEqual((saved["commit"], saved["built"]), (self.c2, self.c1))
        ctx = self.prepare(c4)  # --only <step> at c4
        self.assertEqual((ctx.built, "export" in ctx.done), (self.c1, True))
        src = self.out / "work" / "src"
        self.assertEqual(local_release.need_src(ctx), src)  # an export of c4: the step would not be told to export again
        self.assertEqual((src / ".github" / "release" / "a.py").read_text(encoding="utf-8"), "newest\n")
        self.assertTrue((src / "core" / "keep.txt").is_file())

    def test_a_file_moved_into_the_scripts_folder_is_not_a_scripts_only_change(self) -> None:
        """git diff pairs a deleted and an added file as a rename and names only the new path: the deletion outside
        .github/release/ must still count (the wheel, DLL and SDK of the recorded commit still hold the old file)."""
        for source in ("docs/guide.md", "core/engine.cpp", "bindings/python/tests/test_x.py"):
            with self.subTest(source=source):
                before = self.commit_on(self.c2, source, "a file that is moved into the scripts folder\n")
                git_in(self.repo, "mv", source, ".github/release/moved.txt")
                git_in(self.repo, "commit", "-q", "-m", "move")
                after = git_in(self.repo, "rev-parse", "HEAD")
                self.assertEqual(sorted(local_release.changed_files(before, after) or []),
                                 sorted([source, ".github/release/moved.txt"]))
                self.assertFalse(local_release.only_release_scripts(local_release.changed_files(before, after) or []))
                self.save_run(before)
                with self.assertRaises(local_release.Failure) as raised:
                    self.prepare(after)
                self.assertIn(source, str(raised.exception))
                src = self.out / "work" / "src" / ".github" / "release"
                self.assertEqual((src / "a.py").read_text(encoding="utf-8"), "old\n")  # nothing was refreshed

    def test_the_same_commit_has_no_note(self) -> None:
        ctx = self.prepare(self.c1)
        self.assertEqual((ctx.notes, ctx.built_at), ([], self.c1))

    def test_another_change_is_refused_and_leaves_the_export_alone(self) -> None:
        with self.assertRaises(local_release.Failure) as raised:
            self.prepare(self.c3)  # c1 -> c3 changes README.md too
        self.assertIn("README.md", str(raised.exception))
        src = self.out / "work" / "src" / ".github" / "release" / "a.py"
        self.assertEqual(src.read_text(encoding="utf-8"), "old\n")

    def test_a_commit_git_does_not_know_is_refused(self) -> None:
        self.save_run("0" * 39 + "1")
        with self.assertRaises(local_release.Failure):
            self.prepare(self.c2)

    def test_other_targets_are_refused(self) -> None:
        with self.assertRaises(local_release.Failure) as raised:
            self.prepare(self.c2, targets=[local_release.RID, local_release.LINUX_RID])
        self.assertIn("targets", str(raised.exception))
        src = self.out / "work" / "src" / ".github" / "release" / "a.py"
        self.assertEqual(src.read_text(encoding="utf-8"), "old\n")

    def test_without_the_export_nothing_is_refreshed(self) -> None:
        local_release.rmtree(self.out / "work" / "src")
        ctx = self.prepare(self.c2)
        self.assertEqual(ctx.built, self.c1)
        with self.assertRaises(local_release.Failure):
            local_release.need_src(ctx)  # says: run the step 'export'

    def test_a_clean_run_starts_empty(self) -> None:
        ctx = self.prepare(self.c3, keep=False)
        self.assertEqual((ctx.done, ctx.notes), ({}, []))
        self.assertFalse((self.out / "work" / "src").exists())

    def test_which_paths_count(self) -> None:
        only = local_release.only_release_scripts
        self.assertTrue(only([]))
        self.assertTrue(only([".github/release/local_release.py", ".github/release/sdk/CMakeLists.txt"]))
        self.assertFalse(only([".github/release/a.py", "README.md"]))
        self.assertFalse(only([".github/workflows/release.yml"]))  # the workflow is not a script of this folder
        self.assertFalse(only([".github/releases/a.py"]))
        self.assertEqual(local_release.changed_files(self.c1, self.c2),
                         [".github/release/a.py", ".github/release/sdk/x.txt"])
        self.assertIsNone(local_release.changed_files("0" * 39 + "1", self.c2))


class Handler(http.server.BaseHTTPRequestHandler):
    """A stand-in for iai-gui on 127.0.0.1: GET / is 200, /denied is 403, a POST answers with the access key it was given."""

    def do_GET(self) -> None:  # noqa: N802
        self.reply(403 if self.path == "/denied" else 200, b"<html></html>")

    def do_POST(self) -> None:  # noqa: N802
        length = int(self.headers.get("Content-Length") or 0)
        sent = {"key": self.headers.get(local_release.GUI_TOKEN_HEADER), "type": self.headers.get("Content-Type"),
                "body": self.rfile.read(length).decode("utf-8")}
        self.reply(200, json.dumps(sent).encode("utf-8"))

    def reply(self, status: int, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args) -> None:
        pass


class GuiStep(unittest.TestCase):
    def test_gui_is_the_last_step_before_collect_and_a_win_x64_step(self) -> None:
        """Last but collect: what it makes is only an input of collect, so running it again costs no other step."""
        self.assertEqual(local_release.STEPS[-2:], ["gui", "collect"])
        self.assertEqual(set(local_release.STEP_FUNCS), set(local_release.STEPS))
        self.assertEqual(local_release.STEP_TARGET["gui"], local_release.RID)
        with tempfile.TemporaryDirectory() as tmp:
            both = make_ctx(Path(tmp), "a" * 40, list(local_release.TARGET_NAMES))
            linux = make_ctx(Path(tmp), "a" * 40, [local_release.LINUX_RID])
            windows = make_ctx(Path(tmp), "a" * 40, [local_release.RID])
        self.assertIn("gui", both.steps)
        self.assertIn("gui", windows.steps)
        self.assertNotIn("gui", linux.steps)

    def test_it_does_what_release_yml_does(self) -> None:
        """The paths and the pin of the step are those of the gui job of release.yml (and of the GUI itself)."""
        workflow = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")
        for text in (local_release.GUI_SPEC, local_release.GUI_ZIP, local_release.GUI_BUNDLED_DLL,
                     "--noconfirm --clean", "unittest discover -s apps/gui -t apps"):
            self.assertIn(text, workflow)
        self.assertEqual(make_gui_zip.zip_name("1.0.0"), "iai-gui-1.0.0-win-x64.zip")
        self.assertRegex(local_release.release_yml_pin("PYINSTALLER_VERSION"), r"^\d+\.\d+(\.\d+)?$")
        self.assertRegex(local_release.release_yml_pin("PYTHON_VERSION"), r"^3\.\d+$")
        self.assertTrue((ROOT / local_release.GUI_SPEC).is_file())
        self.assertTrue((ROOT / local_release.GUI_ZIP).is_file())
        for rel in make_gui_zip.FROM_REPOSITORY.values():
            self.assertTrue((ROOT / rel).is_file(), rel)
        server = (ROOT / "apps" / "gui" / "server.py").read_text(encoding="utf-8")
        self.assertEqual(re.search(r'^TOKEN_HEADER = "([^"]+)"', server, re.M).group(1), local_release.GUI_TOKEN_HEADER)

    def test_skip_reasons(self) -> None:
        self.assertIsNone(local_release.gui_skip_reason("win32", True))
        self.assertIn("Windows only", local_release.gui_skip_reason("linux", True))
        self.assertIn("linux", local_release.gui_skip_reason("linux", True))
        self.assertIn("iai-gui.spec", local_release.gui_skip_reason("win32", False))

    def skipped(self, platform: str) -> tuple[str, local_release.Ctx, Path]:
        """step_gui on `platform` with a stale zip of an earlier run in work/dist/gui and no wheel."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        out = Path(tmp.name) / "out"
        ctx = make_ctx(out, "a" * 40)
        stale = ctx.work / "dist" / "gui" / "iai-gui-0.0.1-win-x64.zip"
        stale.parent.mkdir(parents=True)
        stale.write_bytes(b"old")
        with mock.patch.object(local_release.sys, "platform", platform), contextlib.redirect_stdout(io.StringIO()):
            return local_release.step_gui(ctx), ctx, stale

    def test_not_windows_skips_with_a_note_and_drops_a_stale_zip(self) -> None:
        detail, ctx, stale = self.skipped("linux")
        self.assertTrue(detail.startswith("skipped: "), detail)
        self.assertIn("Windows only", detail)
        self.assertEqual([what for what, _ in ctx.facts], ["GUI (iai-gui.exe)"])
        self.assertIn("not built", ctx.facts[0][1])
        self.assertFalse(stale.exists())  # collect would otherwise ship it

    def test_no_spec_skips_with_a_note(self) -> None:
        detail, ctx, stale = self.skipped("win32")  # work/src has no apps/gui/iai-gui.spec
        self.assertTrue(detail.startswith("skipped: "), detail)
        self.assertIn("iai-gui.spec", ctx.facts[0][1])
        self.assertFalse(stale.exists())

    @contextlib.contextmanager
    def on_windows(self, fake_run):
        """Windows, with `run` replaced and nothing printed."""
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(local_release.sys, "platform", "win32"))
            stack.enter_context(mock.patch.object(local_release, "run", fake_run))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            yield

    def test_gui_python_makes_the_environment_and_a_test_failure_names_the_python(self) -> None:
        """The exe embeds the Python its environment is made from: --gui-python, else --python. The GUI tests (--gui-tests)
        run on it, and when they fail the message says which Python that was."""
        with tempfile.TemporaryDirectory() as tmp:
            ctx = make_ctx(Path(tmp) / "out", "a" * 40)
            ctx.gui_python = "C:/Python313/python.exe"
            ctx.args.gui_tests = True
            (ctx.src / "apps" / "gui").mkdir(parents=True)
            (ctx.src / local_release.GUI_SPEC).write_text("", encoding="utf-8")
            (ctx.work / "dist" / "wheel").mkdir(parents=True)
            (ctx.work / "dist" / "wheel" / "iai_actuator_driver-1.0.0-py3-none-win_amd64.whl").write_bytes(b"")
            commands: list[list[str]] = []

            def fake_run(ctx, label, cmd, cwd, env=None, timeout=1200):
                commands.append([label, *[str(c) for c in cmd]])
                if label == "gui-python":
                    return "3.13.15\n"
                if label == "gui-tests":
                    raise local_release.Failure("gui-tests failed (exit 1)")
                return ""

            with self.on_windows(fake_run):
                with self.assertRaises(local_release.Failure) as raised:
                    local_release.step_gui(ctx)
            venv = next(c for c in commands if c[0] == "gui-venv")
            self.assertEqual(venv[1], "C:/Python313/python.exe")
            install = next(c for c in commands if c[0] == "gui-install")
            self.assertIn("pyinstaller==6.22.3", install)
            self.assertEqual(commands[-1][0], "gui-tests")  # nothing is built from a GUI that fails its tests
            self.assertIn("Python 3.13.15", str(raised.exception))
            self.assertIn("timing-sensitive", str(raised.exception))
            ctx.gui_python = ""
            commands.clear()
            with self.on_windows(fake_run):
                with self.assertRaises(local_release.Failure):
                    local_release.step_gui(ctx)
            self.assertEqual(next(c for c in commands if c[0] == "gui-venv")[1], sys.executable)
            # without --gui-tests the step goes on to PyInstaller and says that the tests were not run
            ctx.args.gui_tests = False
            commands.clear()
            ctx.facts = []
            with self.on_windows(fake_run):
                with self.assertRaises(local_release.Failure):  # fake_run builds no exe
                    local_release.step_gui(ctx)
            labels = [c[0] for c in commands]
            self.assertNotIn("gui-tests", labels)
            self.assertIn("gui-pyinstaller", labels)
            self.assertIn(("GUI tests (apps/gui)", "not run (--gui-tests runs them, as the gui job of release.yml does)"),
                          [tuple(f) for f in ctx.facts])

    def test_the_summary_names_where_the_gui_python_comes_from(self) -> None:
        """--gui-python, else --python, else the interpreter that runs the script: the fact of summary.txt and the console
        note say which of the three made the environment (iai-gui.exe embeds its Python)."""
        cases = [("C:/Python313/python.exe", "C:/Python312/python.exe", "--gui-python"),
                 ("", "C:/Python312/python.exe", "--python"),
                 ("", "", "the interpreter that ran this build")]
        for gui_python, python, source in cases:
            with self.subTest(source=source), tempfile.TemporaryDirectory() as tmp:
                ctx = make_ctx(Path(tmp) / "out", "a" * 40)
                ctx.gui_python = gui_python
                if python:
                    ctx.args.python = python
                    ctx.python = python
                (ctx.src / local_release.GUI_SPEC).parent.mkdir(parents=True)
                (ctx.src / local_release.GUI_SPEC).write_text("", encoding="utf-8")
                (ctx.work / "dist" / "wheel").mkdir(parents=True)
                (ctx.work / "dist" / "wheel" / "iai_actuator_driver-1.0.0-py3-none-win_amd64.whl").write_bytes(b"")
                commands: list[list[str]] = []

                def fake_run(ctx, label, cmd, cwd, env=None, timeout=1200):
                    commands.append([label, *[str(c) for c in cmd]])
                    return "3.13.15\n" if label == "gui-python" else ""

                console = io.StringIO()
                with self.on_windows(fake_run), contextlib.redirect_stdout(console):
                    with self.assertRaises(local_release.Failure):  # fake_run builds no exe
                        local_release.step_gui(ctx)
                self.assertEqual(next(c for c in commands if c[0] == "gui-venv")[1], gui_python or python or sys.executable)
                fact = next(f for f in ctx.facts if f[0] == "Python inside iai-gui.exe")
                self.assertEqual(fact[1], f"3.13.15 ({source}; release.yml builds the GUI with 3.12)")
                self.assertIn(f"iai-gui.exe embeds Python 3.13.15 ({source}; release.yml: 3.12)", console.getvalue())

    def test_the_bundle_check(self) -> None:
        """The inline check reads the table of contents of the exe: the library must be inside, at the path the spec gives."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            archive = folder / "PyInstaller" / "archive"
            archive.mkdir(parents=True)
            (folder / "PyInstaller" / "__init__.py").write_text("", encoding="utf-8")
            (archive / "__init__.py").write_text("", encoding="utf-8")
            (archive / "readers.py").write_text(
                "import json\n"
                "class CArchiveReader:\n"
                "    def __init__(self, path):\n"
                "        self.toc = json.load(open('toc.json', encoding='utf-8'))\n", encoding="utf-8")

            def check(toc: list[str]) -> subprocess.CompletedProcess:
                (folder / "toc.json").write_text(json.dumps(toc), encoding="utf-8")
                return subprocess.run([sys.executable, "-c", local_release.GUI_BUNDLE_CHECK, "iai-gui.exe",
                                       local_release.GUI_BUNDLED_DLL], cwd=folder, capture_output=True, text=True)

            windows_name = local_release.GUI_BUNDLED_DLL.replace("/", chr(92))  # a table may use backslashes
            for name in (local_release.GUI_BUNDLED_DLL, windows_name):
                found = check(["base_library.zip", name])
                self.assertEqual(found.returncode, 0, found.stderr)
                self.assertIn("iai_actuator_driver.dll", found.stdout)
            missing = check(["base_library.zip", "iai_actuator_driver/_native/linux-x64/libiai_actuator_driver.so"])
            self.assertEqual(missing.returncode, 1)
            self.assertIn("no " + local_release.GUI_BUNDLED_DLL + " in the bundle", missing.stdout)

    def test_requests_to_the_gui(self) -> None:
        server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.addCleanup(server.server_close)
        threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_address[1]}"
        self.assertEqual(local_release.gui_request(base + "/"), (200, b"<html></html>"))
        self.assertEqual(local_release.gui_request(base + "/denied")[0], 403)
        status, body = local_release.gui_request(base + "/api/quit", token="abc123", body=b"{}")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), {"key": "abc123", "type": "application/json", "body": "{}"})
        server.shutdown()
        server.server_close()
        self.assertEqual(local_release.gui_request(base + "/"), (0, b""))  # nothing listens: no exception

    def test_free_port_is_free(self) -> None:
        port = local_release.free_port()
        self.assertGreater(port, 0)
        server = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
        server.server_close()


class LoadSensitiveRetry(unittest.TestCase):
    """The wheel, linux-wheel and gui test runs go through the runners of tools/test.py (run_backend.py,
    unittest_jobs.py), so a load-sensitive test of tools/load_sensitive.txt that failed runs once more, and the summary
    names it."""

    # fails the first time in its folder, passes once its mark is there
    FLAKY = ("import unittest\nfrom pathlib import Path\nMARK = Path(__file__).with_name('ran-once')\n"
             "class T(unittest.TestCase):\n    def test_a(self):\n        first = not MARK.exists()\n"
             "        MARK.touch()\n        self.assertFalse(first, 'the first run fails')\n")  # fmt: skip
    OK = "import unittest\nclass T(unittest.TestCase):\n    def test_a(self):\n        pass\n"

    def test_the_steps_run_the_runners_of_tools_test(self) -> None:
        source = (HERE / "local_release.py").read_text(encoding="utf-8")
        self.assertIn('[py, "tests/run_backend.py", "--backend", "native"]', source)  # wheel
        self.assertIn('"tools/unittest_jobs.py", "-s", "apps/gui", "-t", "apps"', source)  # gui (--gui-tests)
        self.assertIn('"--load-sensitive", "gui"', source)
        wsl = (ROOT / local_release.WSL_SCRIPT).read_text(encoding="utf-8")
        self.assertIn('"$venv/bin/python" tests/run_backend.py --backend native', wsl)  # linux-wheel
        self.assertNotIn("unittest discover", wsl)
        for runner in ("tools/unittest_jobs.py", "bindings/python/tests/run_backend.py", "tools/load_sensitive.txt"):
            self.assertTrue((ROOT / runner).is_file(), runner)

    def test_the_summary_is_read_as_tools_test_reads_it(self) -> None:
        text = ("backend native (auto: iai_actuator_driver.dll (C ABI 1.1))\n"
                "ran 512 tests in 301.4 s: 509 passed, 3 skipped, 0 expected failures, 0 unexpected successes, "
                "0 failures, 0 errors\nslowest: test_x.T.test_a 9.0 s\n"
                "passed on its retry (load-sensitive): test_cell.ScanTest.test_scan\nOK on the native backend\n")
        self.assertEqual(local_release.suite_summary("python", text),
                         "512 tests, 509 passed, 3 skipped, 0 failed in 301 s; passed only on its retry "
                         "(load-sensitive): test_cell.ScanTest.test_scan")  # fmt: skip
        clean = text.replace("passed on its retry (load-sensitive): test_cell.ScanTest.test_scan\n", "")
        self.assertEqual(local_release.suite_summary("python", clean),
                         "512 tests, 509 passed, 3 skipped, 0 failed in 301 s")
        self.assertEqual(local_release.suite_summary("gui", "no summary\n"), "ran")

    def test_a_load_sensitive_gui_test_that_fails_once_is_named_in_the_summary(self) -> None:
        """unittest_jobs.py as the gui step runs it, on scratch tests with a list of their own: the flaky test passes on
        its retry, the run passes, and the step's summary names the test."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "suite"
            folder.mkdir()
            (folder / "test_flaky.py").write_text(self.FLAKY, encoding="utf-8")
            (folder / "test_ok.py").write_text(self.OK, encoding="utf-8")
            listed = Path(tmp) / "load_sensitive.txt"
            listed.write_text("gui test_flaky.T.test_a  measures a time\n", encoding="utf-8")
            env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
            proc = subprocess.run([sys.executable, str(ROOT / "tools" / "unittest_jobs.py"), "-s", str(folder), "-t",
                                   str(folder), "--jobs", "2", "--load-sensitive", "gui", "--load-sensitive-file",
                                   str(listed)], cwd=tmp, env=env, capture_output=True, text=True)  # fmt: skip
        text = proc.stderr + proc.stdout
        self.assertEqual(proc.returncode, 0, text)
        summary = local_release.suite_summary("gui", text)
        self.assertTrue(summary.startswith("2 tests, 2 passed, 0 skipped, 0 failed in "), summary)
        self.assertTrue(summary.endswith("; passed only on its retry (load-sensitive): test_flaky.T.test_a"), text)


class DefaultTargets(unittest.TestCase):
    def test_both_platforms_when_wsl_is_there(self) -> None:
        """The NuGet package carries both runtimes only when linux-x64 is built: that is what a bare run does with WSL."""
        with tempfile.TemporaryDirectory() as tmp:
            kit = Path(tmp)
            (kit / "run.ps1").write_text("", encoding="utf-8")
            with mock.patch.object(local_release.shutil, "which", lambda name: name), \
                    mock.patch.object(local_release.subprocess, "run", lambda *a, **k: FakeWslList()):
                found, wsl, notes = local_release.choose_targets(args_of(wsl_kit=str(kit)))
            self.assertEqual(found, [local_release.RID, local_release.LINUX_RID])
            self.assertIsNotNone(wsl)
            self.assertEqual(notes, [])
            steps = make_ctx(kit / "out", "a" * 40, found).steps
            self.assertEqual(steps, local_release.STEPS)  # every step, the linux ones and gui among them
            self.assertEqual(",".join(found), "win-x64,linux-x64")  # the --rids of the nupkg check

    def test_without_wsl_it_is_windows_with_a_note_that_says_why(self) -> None:
        with mock.patch.object(local_release.shutil, "which", lambda name: None):
            found, wsl, notes = local_release.choose_targets(args_of())
        self.assertEqual((found, wsl), ([local_release.RID], None))
        self.assertEqual(len(notes), 1)
        self.assertIn("wsl.exe is not installed", notes[0])
        self.assertIn("--targets win-x64", notes[0])


class CiPathFilters(unittest.TestCase):
    """.github/workflows/ci.yml: the Python package runs on the core only (the pure-Python edition was removed after
    1.0.x), so there is no pure-Python job or area, and every job that imports the package gets a native library."""

    PACKAGE = "bindings/python/iai_actuator_driver/"

    def setUp(self) -> None:
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML is not installed")
        self.text = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
        self.ci = yaml.safe_load(self.text)  # valid YAML
        steps = self.ci["jobs"]["changes"]["steps"]
        self.script = next(step["run"] for step in steps if step.get("id") == "areas")
        self.areas = re.search(r"^\s*for area in ([\w ]+); do$", self.script, re.M).group(1).split()

    def area(self, name: str) -> re.Pattern[str]:
        """The path filter of an area, with the shell variables it names filled in."""
        raw = {k: v for k, _, v in re.findall(r"^\s*(\w+)=(['\"])(.+)\2$", self.script, re.M)}
        pattern = re.sub(r"\$(\w+)", lambda m: raw[m.group(1)], raw[name])
        return re.compile(pattern)

    def test_every_output_is_declared_and_computed(self) -> None:
        declared = set(self.ci["jobs"]["changes"]["outputs"])
        used = set(re.findall(r"needs\.changes\.outputs\.(\w+)", self.text))
        self.assertLessEqual(used, declared)
        self.assertEqual(declared, set(self.areas))

    def test_no_pure_python_job_or_area(self) -> None:
        self.assertNotIn("pure", self.areas)
        self.assertNotIn("python", self.ci["jobs"])  # the job of the pure-Python suite
        self.assertNotIn("--backend python", self.text)
        self.assertNotIn("_pure", self.text)

    def test_every_job_that_imports_the_package_gets_a_library(self) -> None:
        jobs = self.ci["jobs"]
        for name, artifacts in (("gui", {"native-windows-x64", "native-linux-x64"}), ("build", {"native-windows-x64"}),
                                ("python-native", {"native-windows-x64"})):  # fmt: skip
            with self.subTest(job=name):
                text = str(jobs[name])
                self.assertTrue(all(a in text or "matrix.native" in text for a in artifacts), name)
                self.assertIn("IAI_ACTUATOR_DRIVER_NATIVE", text)
        self.assertLessEqual({"core-windows", "core-linux"}, set(jobs["gui"]["needs"]))
        for core in ("core-windows", "core-linux"):  # they run when the GUI's area changed, to make its libraries
            self.assertIn("needs.changes.outputs.gui == 'true'", jobs[core]["if"], core)
        include = jobs["gui"]["strategy"]["matrix"]["include"]
        self.assertEqual({i["os"]: i["native"] for i in include},
                         {"windows-latest": "native-windows-x64", "ubuntu-latest": "native-linux-x64"})

    def test_the_driver_suite_runs_on_windows_and_linux(self) -> None:
        """python-native runs the whole suite against the DLL, core-linux against the .so."""
        for job, library in (("python-native", "iai_actuator_driver.dll"), ("core-linux", "libiai_actuator_driver.so")):
            with self.subTest(job=job):
                steps = [s for s in self.ci["jobs"][job]["steps"] if "tests/run_backend.py" in str(s.get("run", ""))]
                self.assertEqual(len(steps), 1, job)
                self.assertIn(library, steps[0]["env"]["IAI_ACTUATOR_DRIVER_NATIVE"])

    def test_the_build_checks_that_the_driver_needs_its_library(self) -> None:
        build = "\n".join(str(step.get("run", "")) for step in self.ci["jobs"]["build"]["steps"])
        self.assertIn("needs its native library", build)
        self.assertIn("iai-actuator-driver --sim status", build)

    def test_the_package_is_in_the_core_area(self) -> None:
        core = self.area("core")
        for path in ("ec.py", "native.py", "_native/__init__.py", "_shared/coord.py", "_backend.py", "cli.py"):
            self.assertRegex(self.PACKAGE + path, core, path)
        for path in ("bindings/python/tests/run_backend.py", "bindings/python/tests/test_native_binding.py",
                     "bindings/python/tests/test_backend.py", "core/src/x.cpp"):
            self.assertRegex(path, core, path)
        for path in ("docs/GETTING_STARTED.md", "apps/gui/server.py", "bindings/python/tests/test_cell.py"):
            self.assertNotRegex(path, core, path)

    def test_build_waits_for_every_test_job(self) -> None:
        """The sdists and wheels are built after the jobs that tested them: a skipped job (its area did not change) does
        not hold the build back, a failed one does (!failure() in its condition)."""
        build = self.ci["jobs"]["build"]
        self.assertLessEqual({"changes", "gui", "core-windows", "python-native"}, set(build["needs"]))
        self.assertIn("!failure()", build["if"])
        self.assertIn("!cancelled()", build["if"])

    def test_the_native_suite_sees_the_documents_examples_and_release_scripts(self) -> None:
        """python-native runs the whole suite (test_packaging reads the documents, test_examples runs the examples)."""
        native = self.area("native")
        for path in ("core/src/x.cpp", "bindings/python/x.py", "docs/GETTING_STARTED.md", "examples/python/x.py",
                     "README.md", ".github/CONTRIBUTING.md", ".github/SECURITY.md", "licenses/NOTICE",
                     ".github/release/local_release.py"):
            self.assertRegex(path, native, path)


class WorkflowTriggers(unittest.TestCase):
    """Nothing starts on GitHub by itself on a push or a tag: ci.yml runs on a pull request or by hand, release.yml by hand
    only (its release job replaces the assets of a published release). Releases are built with local_release.py."""

    def workflow(self, name: str) -> dict:
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML is not installed")
        return yaml.safe_load((ROOT / ".github" / "workflows" / name).read_text(encoding="utf-8"))

    def triggers(self, name: str) -> set[str]:
        """The workflow's events, read as text (the keys two spaces in under the top-level ``on:``, up to the next
        top-level key), so this guard needs no YAML library and never skips."""
        lines = (ROOT / ".github" / "workflows" / name).read_text(encoding="utf-8").splitlines()
        start = next((i for i, line in enumerate(lines) if line.startswith("on:")), -1)
        self.assertGreaterEqual(start, 0, f"{name}: no top-level on:")
        self.assertEqual(lines[start][3:].split("#")[0].strip(), "", f"{name}: write the events as a block under on:")
        events: set[str] = set()
        for line in lines[start + 1:]:
            if re.match(r"[^\s#]", line):
                break
            if m := re.match(r"  ([A-Za-z_]+):", line):
                events.add(m[1])
        return events

    def test_release_runs_by_hand_only(self) -> None:
        self.assertEqual(self.triggers("release.yml"), {"workflow_dispatch"})

    def test_a_manual_release_run_publishes_nothing(self) -> None:
        """Only a tag push reaches the release job; a manual run of a tag does not."""
        condition = self.workflow("release.yml")["jobs"]["release"]["if"]
        self.assertIn("github.event_name == 'push'", condition)
        self.assertIn("github.ref_type == 'tag'", condition)

    def test_ci_runs_on_a_pull_request_or_by_hand(self) -> None:
        self.assertEqual(self.triggers("ci.yml"), {"pull_request", "workflow_dispatch"})

    def test_no_job_of_ci_waits_for_a_push(self) -> None:
        """The quick job runs on every pull request and manual run; no job is limited to the push event."""
        jobs = self.workflow("ci.yml")["jobs"]
        self.assertNotIn("if", jobs["quick"])
        for name, job in jobs.items():
            self.assertNotIn("push", str(job.get("if", "")), name)


class SdkReadme(unittest.TestCase):
    def readme(self, rid: str) -> str:
        with tempfile.TemporaryDirectory() as tmp:
            native = Path(tmp) / "native"
            native.mkdir()
            lib = "iai_actuator_driver.dll" if rid.startswith("win") else "libiai_actuator_driver.so"
            (native / lib).write_bytes(b"\0")
            if rid.startswith("win"):
                (native / "iai_actuator_driver.lib").write_bytes(b"\0")
            out = Path(tmp) / "out"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(make_sdk.main(["--rid", rid, "--version", "0.0.1", "--native", str(native),
                                                "--root", str(ROOT), "--out", str(out)]), 0)
            with zipfile.ZipFile(next(out.glob("*.zip"))) as zf:
                return zf.read(f"iai_actuator_driver-sdk-0.0.1-{rid}/README.txt").decode("utf-8")

    def test_linux_compile_line_writes_the_program_to_bin(self) -> None:
        """The line is typed in the SDK folder: its program must land in bin/, where $ORIGIN/../lib is the SDK's lib/
        (local_release_wsl.sh sdk-smoke runs the line as written, from this README)."""
        lines = [line.strip() for line in self.readme("linux-x64").splitlines() if line.startswith("  cc -Iinclude")]
        self.assertEqual(len(lines), 1, lines)
        line = lines[0]
        self.assertIn("-Wl,-rpath,'$ORIGIN/../lib'", line)
        self.assertIn("-liai_actuator_driver", line)
        self.assertTrue(line.endswith("-o bin/app"), line)

    def test_windows_readme_has_no_linux_line(self) -> None:
        self.assertNotIn("$ORIGIN", self.readme("win-x64"))

    def test_hardware_evidence_points_to_the_page_of_the_repository(self) -> None:
        """The SDK ships NATIVE_ABI.md, README.md and CHANGELOG.md but not COMPATIBILITY.md, where the evidence per unit
        and feature lives: the README links that page on GitHub."""
        page = "https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/COMPATIBILITY.md"
        self.assertTrue((ROOT / "docs" / "COMPATIBILITY.md").is_file())
        for rid in ("win-x64", "linux-x64"):
            text = self.readme(rid)
            self.assertIn(f"Hardware evidence per unit and feature: {page}\n", text, rid)
            self.assertNotIn("README.md and CHANGELOG.md", text, rid)


def ctest_output(live: str = "Passed", frozen: str = "Passed") -> str:
    """What ctest prints for sdk_consumer (its two tests); `live` and `frozen` are the result columns."""
    return ("Test project /build/sdk_consumer\n"
            f"    Start 1: c_smoke\n1/2 Test #1: c_smoke ......................   {live}    0.31 sec\n"
            f"    Start 2: c_smoke_1_0\n2/2 Test #2: c_smoke_1_0 .................   {frozen}    0.30 sec\n\n"
            "100% tests passed out of 2\n")


NO_FROZEN = ctest_output().split("    Start 2")[0]  # the frozen program was not built: ctest never ran it


class SdkSmokeSteps(unittest.TestCase):
    """step_sdk_smoke and step_linux_sdk_smoke build the frozen C program of ABI 1.0 (core/abi/1.0) beside the live one
    in sdk_consumer, and fail unless ctest reports both as passed."""

    def test_the_files_the_steps_name_exist(self) -> None:
        source = ROOT / local_release.FROZEN_SMOKE
        self.assertEqual(local_release.FROZEN_SMOKE, "core/abi/1.0/c_smoke.c")
        self.assertTrue(source.is_file())
        self.assertTrue((source.parent / "iai_actuator_driver.h").is_file())  # the 1.0 header, beside it
        cmake = (ROOT / ".github" / "release" / "sdk_consumer" / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in local_release.SMOKE_TESTS:  # the test names ctest reports
            self.assertRegex(cmake, rf"(?m)^iad_smoke\({name} ", name)

    def test_the_wsl_script_and_the_workflow_pass_the_same_source(self) -> None:
        wsl = (ROOT / local_release.WSL_SCRIPT).read_text(encoding="utf-8")
        self.assertIn(f"frozen=$SRC/{local_release.FROZEN_SMOKE}", wsl)
        self.assertIn('-DIAD_FROZEN_SMOKE_SOURCE="$frozen"', wsl)
        workflow = (ROOT / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")
        self.assertIn(f'-DIAD_FROZEN_SMOKE_SOURCE="$GITHUB_WORKSPACE/{local_release.FROZEN_SMOKE}"', workflow)

    def test_ctest_must_report_both_programs_as_passed(self) -> None:
        local_release.ctest_passed(ctest_output())
        for text, missing in ((ctest_output(frozen="***Failed"), "c_smoke_1_0"), (NO_FROZEN, "c_smoke_1_0"),
                              (ctest_output(live="***Failed"), "c_smoke"), ("0 tests failed out of 0\n", "c_smoke")):
            with self.assertRaises(local_release.Failure, msg=text) as raised:
                local_release.ctest_passed(text)
            self.assertIn(f"report {missing} as", str(raised.exception))

    def windows_step(self, ctest: str) -> tuple[local_release.Ctx, dict[str, list[str]], str]:
        """step_sdk_smoke with `run` replaced: the commands it ran by label, and ctest printing `ctest`."""
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            ctx = make_ctx(Path(tmp), "a" * 40)
            (ctx.work / "dist" / "sdk").mkdir(parents=True)
            with zipfile.ZipFile(ctx.work / "dist" / "sdk" / f"{ctx.sdk_name}.zip", "w") as zf:
                zf.writestr(f"{ctx.sdk_name}/README.txt", "x")
            commands: dict[str, list[str]] = {}

            def fake_run(_ctx, label, cmd, cwd, env=None, timeout=1200):
                commands[label] = [str(c) for c in cmd]
                return ctest if label == "sdk-ctest" else ""

            with mock.patch.object(local_release, "run", fake_run):
                detail = local_release.step_sdk_smoke(ctx)
            return ctx, commands, detail

    def test_the_windows_step_builds_both_programs_and_says_so(self) -> None:
        ctx, commands, detail = self.windows_step(ctest_output())
        frozen = (ctx.src / "core" / "abi" / "1.0" / "c_smoke.c").as_posix()
        self.assertIn(f"-DIAD_SMOKE_SOURCE={(ctx.src / 'core' / 'tests' / 'c_smoke.c').as_posix()}", commands["sdk-cmake"])
        self.assertIn(f"-DIAD_FROZEN_SMOKE_SOURCE={frozen}", commands["sdk-cmake"])
        self.assertIn("c_smoke_1_0", detail)
        self.assertIn("frozen ABI 1.0", ctx.facts[0][0])

    def test_the_windows_step_fails_when_the_frozen_program_does_not_pass(self) -> None:
        for text in (ctest_output(frozen="***Failed"), NO_FROZEN):
            with self.assertRaises(local_release.Failure):
                self.windows_step(text)

    def linux_step(self, text: str) -> tuple[local_release.Ctx, str]:
        """step_linux_sdk_smoke with the staging and the WSL run replaced: the run prints `text`."""
        with tempfile.TemporaryDirectory() as tmp:
            ctx = make_ctx(Path(tmp), "a" * 40, [local_release.RID, local_release.LINUX_RID])
            sdk = ctx.work / "dist" / f"sdk-{local_release.LINUX_RID}"
            sdk.mkdir(parents=True)
            (sdk / f"{ctx.linux_sdk_name}.zip").write_bytes(b"")
            with mock.patch.object(local_release, "linux_stage", lambda *a, **k: None), \
                    mock.patch.object(local_release, "run_wsl", lambda *a, **k: text):
                return ctx, local_release.step_linux_sdk_smoke(ctx)

    def test_the_linux_step_reads_the_same_ctest_output(self) -> None:
        manual = "c_smoke: 107 checks, 0 failed\n"  # the README's compile line, run by the WSL script after ctest
        ctx, detail = self.linux_step(ctest_output() + manual)
        self.assertIn("c_smoke_1_0", detail)
        self.assertIn("frozen ABI 1.0", ctx.facts[0][0])
        self.assertIn("c_smoke: 107 checks, 0 failed", ctx.facts[1][1])
        for text in (ctest_output(frozen="***Failed") + manual, NO_FROZEN + manual, manual):
            with self.assertRaises(local_release.Failure):
                self.linux_step(text)


NOTICE_FILES = {name: (ROOT / rel).read_bytes() for name, rel in check_package.NOTICES.items()}


def write_zip(path: Path, files: dict[str, bytes]) -> Path:
    with zipfile.ZipFile(path, "w") as zf:
        for name, data in files.items():
            zf.writestr(name, data)
    return path


def write_tar(path: Path, files: dict[str, bytes]) -> Path:
    with tarfile.open(path, "w:gz") as tf:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tf.addfile(info, io.BytesIO(data))
    return path


def fake_assets(folder: Path, drop: str = "", version: str = "1.2.3") -> dict[str, Path]:
    """One asset of each kind, with the NOTICES files where its kind keeps them; `drop` leaves that file out."""
    def notices(prefix: str) -> dict[str, bytes]:
        return {prefix + name: data for name, data in NOTICE_FILES.items() if name != drop}
    meta = b"Metadata-Version: 2.4\nName: iai-actuator-driver\nLicense-Expression: Apache-2.0\n"
    dist_info = f"iai_actuator_driver-{version}.dist-info"
    sdist = f"iai_actuator_driver-{version}"
    sdk = f"iai_actuator_driver-sdk-{version}-win-x64"
    nuspec = b'<package><metadata><id>IaiActuatorDriver</id><license type="expression">Apache-2.0</license>' \
             b'</metadata></package>'
    return {
        "wheel": write_zip(folder / f"{sdist}-py3-none-win_amd64.whl",
                           {f"{dist_info}/METADATA": meta, **notices(f"{dist_info}/licenses/")}),
        "sdist": write_tar(folder / f"{sdist}.tar.gz", {f"{sdist}/PKG-INFO": meta, **notices(f"{sdist}/")}),
        "nupkg": write_zip(folder / f"IaiActuatorDriver.{version}.nupkg",
                           {"IaiActuatorDriver.nuspec": nuspec, **notices("")}),
        "sdk": write_zip(folder / f"{sdk}.zip",
                         {f"{sdk}/README.txt": b"sdk", **notices(f"{sdk}/share/doc/iai_actuator_driver/")}),
        "gui": write_zip(folder / f"iai-gui-{version}-win-x64.zip", {"iai-gui.exe": b"MZ", **notices("")}),
    }


class LicenceFiles(unittest.TestCase):
    """Every release asset carries the repository's LICENSE, NOTICE and NATIVE_THIRD_PARTY.txt: the packaging scripts
    refuse to package without them, check_package.py checks the wheel, sdist and nupkg, and collect_release.py refuses an asset that
    lacks one."""

    def test_every_kind_of_asset_has_them_where_its_kind_keeps_them(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            for kind, path in fake_assets(Path(tmp)).items():
                self.assertEqual(check_package.missing_notices(path), [], kind)

    def test_a_missing_or_different_file_is_named(self) -> None:
        for drop in check_package.NOTICES:
            with tempfile.TemporaryDirectory() as tmp:
                for kind, path in fake_assets(Path(tmp), drop=drop).items():
                    lacking = check_package.missing_notices(path)
                    self.assertEqual(len(lacking), 1, (kind, lacking))
                    self.assertTrue(lacking[0].endswith(drop), (kind, lacking))
        with tempfile.TemporaryDirectory() as tmp:
            other = write_zip(Path(tmp) / "iai-gui-1.2.3-win-x64.zip",
                              {**NOTICE_FILES, "LICENSE": b"MIT"})
            self.assertEqual(check_package.missing_notices(other), ["LICENSE (not the repository's LICENSE)"])

    def test_check_package_checks_the_wheel_sdist_and_nupkg(self) -> None:
        commands = {"wheel": ["--rid", "win-x64"], "sdist": [], "nupkg": ["--rids", ""]}
        for drop, verdict in (("", "ok    "), ("NOTICE", "FAILED")):
            with tempfile.TemporaryDirectory() as tmp:
                assets = fake_assets(Path(tmp), drop=drop)
                for kind, extra in commands.items():
                    out = io.StringIO()
                    with contextlib.redirect_stdout(out):
                        check_package.main([kind, str(assets[kind]), *extra])
                    self.assertIn(f"{verdict} carries the repository's {', '.join(check_package.NOTICES)}",
                                  out.getvalue(), kind)
                    self.assertRegex(out.getvalue(), r"ok     (METADATA|PKG-INFO|nuspec): .*Apache-2\.0", kind)

    def test_collect_refuses_an_asset_without_them(self) -> None:
        def collect(drop: str) -> tuple[int, str]:
            with tempfile.TemporaryDirectory() as tmp:
                artifacts = Path(tmp) / "artifacts"
                artifacts.mkdir()
                fake_assets(artifacts, drop=drop)
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    code = collect_release.main([
                        "--artifacts", str(artifacts), "--out", str(Path(tmp) / "release"), "--version", "1.2.3",
                        "--tag", "v1.2.3", "--changelog", str(Path(tmp) / "none.md"), "--minimum", "Python wheels=1",
                        "--minimum", "C/C++ SDK=1"])
                return code, out.getvalue()

        self.assertEqual(collect("")[0], 0)
        code, text = collect("LICENSE")
        self.assertEqual(code, 1)
        self.assertEqual(text.count("lacks"), 5, text)

    def test_the_sdk_zip_carries_them_and_is_not_made_without_them(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            native = Path(tmp) / "native"
            native.mkdir()
            (native / "libiai_actuator_driver.so").write_bytes(b"\0")
            args = ["--rid", "linux-x64", "--version", "1.2.3", "--native", str(native)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(make_sdk.main([*args, "--root", str(ROOT), "--out", str(Path(tmp) / "out")]), 0)
            self.assertEqual(check_package.missing_notices(next((Path(tmp) / "out").glob("*.zip"))), [])

            for lacking in check_package.NOTICES:  # a root with the headers and every notice file but one
                bare = Path(tmp) / f"bare-{lacking}"
                (bare / "core").mkdir(parents=True)
                shutil.copytree(ROOT / "core" / "include", bare / "core" / "include")
                for name, rel in check_package.NOTICES.items():
                    if name != lacking:
                        (bare / rel).parent.mkdir(parents=True, exist_ok=True)
                        (bare / rel).write_bytes(NOTICE_FILES[name])
                out = Path(tmp) / f"bare-out-{lacking}"
                with self.assertRaises(SystemExit) as caught:
                    make_sdk.main([*args, "--root", str(bare), "--out", str(out)])
                self.assertIn(lacking, str(caught.exception.code))
                self.assertFalse(out.exists(), lacking)

    def test_the_gui_zip_carries_them_and_is_not_made_without_them(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            exe, python_license = Path(tmp) / "iai-gui.exe", Path(tmp) / "LICENSE.txt"
            exe.write_bytes(b"MZ")
            python_license.write_text("PSF\n", encoding="utf-8")
            path = make_gui_zip.make(exe, "1.2.3", Path(tmp) / "out", python_license)
            self.assertEqual(path.name, "iai-gui-1.2.3-win-x64.zip")
            with zipfile.ZipFile(path) as zf:
                self.assertEqual(sorted(zf.namelist()), ["LICENSE", "NATIVE_THIRD_PARTY.txt", "NOTICE",
                                                         "PYTHON_LICENSE.txt", "README.txt", "THIRD_PARTY_NOTICES.txt",
                                                         "iai-gui.exe"])
            self.assertEqual(check_package.missing_notices(path), [])

            for lacking in make_gui_zip.FROM_REPOSITORY:  # the repository files of the zip without one of them
                bare = Path(tmp) / f"bare-{lacking}"
                for name, rel in make_gui_zip.FROM_REPOSITORY.items():
                    if name != lacking:
                        (bare / rel).parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(ROOT / rel, bare / rel)
                out = Path(tmp) / f"bare-out-{lacking}"
                with self.assertRaises(SystemExit) as caught:
                    make_gui_zip.make(exe, "1.2.3", out, python_license, root=bare)
                self.assertIn(f"{lacking} (", str(caught.exception.code))
                self.assertFalse(out.exists(), lacking)

    def test_the_gui_zip_docstring_lists_its_contents(self) -> None:
        """make_gui_zip.py's docstring is the one place that lists what the GUI zip holds (iai-gui.spec, release.yml and
        local_release.py point to it or build the list from CONTENTS): it names every file."""
        for name in make_gui_zip.CONTENTS:
            self.assertIn(name, make_gui_zip.__doc__)

    def test_the_gui_notices_name_what_the_spec_bundles(self) -> None:
        """THIRD_PARTY_NOTICES.txt names what the exe carries and holds the texts that PYTHON_LICENSE.txt lacks (each
        with a line only its text has); NATIVE_THIRD_PARTY.txt does the same for what is linked into the library. What
        the notices say is not in the exe holds only while the spec leaves it out."""
        notices = (ROOT / make_gui_zip.FROM_REPOSITORY["THIRD_PARTY_NOTICES.txt"]).read_text(encoding="utf-8")
        native = (ROOT / check_package.NOTICES["NATIVE_THIRD_PARTY.txt"]).read_text(encoding="utf-8")
        spec = (ROOT / local_release.GUI_SPEC).read_text(encoding="utf-8")
        self.assertIn('"serial"', spec)
        excludes = re.search(r"^\s*excludes=(\[[^\]]*\])", spec, re.M)
        self.assertIsNotNone(excludes, "excludes=[...] in iai-gui.spec")
        self.assertGreaterEqual(set(ast.literal_eval(excludes.group(1))), {"ssl", "_ssl", "_hashlib", "tkinter"})
        self.assertIn("Not in the exe: OpenSSL", notices)
        self.assertIn("Tcl/Tk", notices)
        for needle in ("PSF-2.0", "pySerial", "BSD-3-Clause", "PyInstaller", "Bootloader-exception", "PYTHON_LICENSE.txt",
                       "VCRUNTIME140.dll", "NATIVE_THIRD_PARTY.txt", "libffi - Copyright (c) 1996-2022  Anthony Green",
                       "Thai Open Source Software Center", "Stefan Krah", "Jean-loup Gailly",
                       "INRIA, CMU and Microsoft Corporation", "Daan Leijen"):
            self.assertIn(needle, notices)
        for needle in ("LLVM-exception", "LLVM Exceptions to the Apache 2.0 License", "Zig compiler_rt",
                       "Copyright (c) Zig contributors", "mingw-w64", "ZPL-2.1", "Zope Public License"):
            self.assertIn(needle, native)
        for name, text in (("THIRD_PARTY_NOTICES.txt", notices), ("NATIVE_THIRD_PARTY.txt", native)):
            for line in text.splitlines():  # zig links its own compiler_rt: LLVM's compiler-rt is not in the library
                self.assertFalse("LLVM" in line and "compiler-rt" in line, (name, line))

    def test_the_nuget_package_declares_them(self) -> None:
        csproj = (ROOT / "bindings" / "dotnet" / "src" / "IaiActuatorDriver" / "IaiActuatorDriver.csproj").read_text(
            encoding="utf-8")
        includes = [rel.replace("/", "\\") for rel in check_package.NOTICES.values()]  # the repository files, as the
        # csproj names them (from its folder, with backslashes)
        for needle in ("<PackageLicenseExpression>Apache-2.0</PackageLicenseExpression>",
                       "<Authors>Zachary Ippolito</Authors>", "<Copyright>Copyright 2026 Zachary Ippolito</Copyright>",
                       *(f'<None Include="..\\..\\..\\..\\{rel}" Pack="true" PackagePath="\\"'
                         for rel in includes)):
            self.assertIn(needle, csproj)


def fake_elf(imports: tuple[str, ...], exports: tuple[str, ...] = ("iad_version",)) -> bytes:
    """A 64-bit little-endian ELF shared object with only a .dynstr and a .dynsym section: `imports` undefined global
    functions, `exports` defined ones (what check_package.elf_info reads of the symbols)."""
    import struct

    names = [*imports, *exports]
    dynstr = b"\0" + b"".join(n.encode() + b"\0" for n in names)
    offsets, at = [], 1
    for n in names:
        offsets.append(at)
        at += len(n) + 1
    dynsym = bytes(24)  # entry 0: the null symbol
    for i, _ in enumerate(names):
        defined = i >= len(imports)
        dynsym += struct.pack("<IBBHQQ", offsets[i], (1 << 4) | 2, 0, 1 if defined else 0, 0x1000 if defined else 0, 0)
    dynstr_off = 64
    dynsym_off = (dynstr_off + len(dynstr) + 7) // 8 * 8
    shoff = (dynsym_off + len(dynsym) + 7) // 8 * 8
    header = b"\x7fELF" + bytes([2, 1, 1]) + bytes(9)
    header += struct.pack("<HHIQQQIHHHHHH", 3, 62, 1, 0, 0, shoff, 0, 64, 0, 0, 64, 3, 0)
    sections = bytes(64)  # section 0: null
    sections += struct.pack("<IIQQQQIIQQ", 0, 3, 0, 0, dynstr_off, len(dynstr), 0, 0, 1, 0)  # SHT_STRTAB
    sections += struct.pack("<IIQQQQIIQQ", 0, 11, 0, 0, dynsym_off, len(dynsym), 1, 1, 8, 24)  # SHT_DYNSYM
    data = bytearray(shoff + len(sections))
    data[:64] = header
    data[dynstr_off:dynstr_off + len(dynstr)] = dynstr
    data[dynsym_off:dynsym_off + len(dynsym)] = dynsym
    data[shoff:] = sections
    return bytes(data)


class MonotonicWaits(unittest.TestCase):
    """The Linux library's timed waits run on the monotonic clock. Built against glibc 2.17 (manylinux2014), libc++ has
    no pthread_cond_clockwait: std::condition_variable waits with a timeout on an absolute CLOCK_REALTIME deadline
    (pthread_cond_timedwait), which a backward step of the system clock stretches (1.0.0: a coordinated stream stopped
    with LateFrame in WSL). check_package.py's gate refuses a library that imports pthread_cond_timedwait without
    pthread_condattr_setclock or pthread_cond_clockwait; the core waits on its own condition variable
    (core/src/monotonic_cv.hpp), never on std::condition_variable or another timed wait of the standard library."""

    def clock_check(self, imports: tuple[str, ...]) -> list[str]:
        info = check_package.elf_info(fake_elf(imports))
        c = check_package.Checker("lib")
        with contextlib.redirect_stdout(io.StringIO()):
            check_package.check_clock(c, info, "lib")
        return c.errors

    def test_elf_info_reads_the_imported_symbols(self) -> None:
        info = check_package.elf_info(fake_elf(("pthread_cond_timedwait", "clock_gettime"), ("iad_version", "iad_open")))
        self.assertEqual(sorted(info["imports"]), ["clock_gettime", "pthread_cond_timedwait"])
        self.assertEqual(sorted(info["exports"]), ["iad_open", "iad_version"])

    def test_the_gate_refuses_a_wall_clock_timed_wait(self) -> None:
        # the 1.0.0 library: libc++ (glibc 2.17) waits on pthread_cond_timedwait and nothing sets the clock
        errors = self.clock_check(("pthread_cond_wait", "pthread_cond_timedwait", "pthread_cond_broadcast"))
        self.assertEqual(len(errors), 1)
        self.assertIn("timed waits on the monotonic clock", errors[0])
        self.assertIn("imports pthread_cond_timedwait)", errors[0])

    def test_the_gate_takes_a_monotonic_condition_variable_or_clockwait(self) -> None:
        self.assertEqual(self.clock_check(("pthread_cond_timedwait", "pthread_condattr_setclock")), [])
        self.assertEqual(self.clock_check(("pthread_cond_clockwait",)), [])  # libc++ / libstdc++ with glibc 2.30+
        self.assertEqual(self.clock_check(("pthread_cond_wait", "nanosleep")), [])  # no timed wait at all

    def test_the_library_check_runs_the_gate_on_linux_only(self) -> None:
        source = (HERE / "check_package.py").read_text(encoding="utf-8")
        body = source[source.index("def check_library("):source.index("class Checker")]
        linux = body[body.index("else:"):body.index("exports = sorted(")]
        self.assertIn("check_clock(c, info, label)", linux)

    def test_the_core_has_no_wall_clock_timed_wait(self) -> None:
        # every condition variable of the core is detail::MonotonicCv; no other timed wait of the standard library
        banned = re.compile(r"\bstd::(condition_variable(_any)?|(recursive_|shared_)?timed_mutex|(shared_)?future|"
                            r"counting_semaphore|binary_semaphore)\b|this_thread::sleep_until|pthread_cond_timedwait")
        found = []
        for folder in ("src", "include", "cli"):
            for path in sorted((ROOT / "core" / folder).rglob("*")):
                if path.suffix not in (".cpp", ".hpp", ".h") or path.name == "monotonic_cv.hpp":
                    continue
                for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                    code = line.split("//")[0]
                    if banned.search(code):
                        found.append(f"{path.relative_to(ROOT).as_posix()}:{number}: {line.strip()}")
        self.assertEqual(found, [])
        self.assertIn("pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)",
                      (ROOT / "core" / "src" / "monotonic_cv.hpp").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
