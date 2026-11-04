#!/usr/bin/env python3
"""Build, test and collect the win-x64 and linux-x64 deliverables of a release on this PC, without GitHub Actions.

    python .github/release/local_release.py                  (the version of bindings/python; output: see --out)
    python .github/release/local_release.py --out D:\\release\\X.Y.Z
    python .github/release/local_release.py --resume          (after a failed step: continue the run of this commit)
    python .github/release/local_release.py --only wheel,nuget     (run these steps again, and those after them)
    python .github/release/local_release.py --targets win-x64      (Windows only: no WSL needed)

It does what .github/workflows/release.yml does, with the same scripts of this folder, and stops at the first failure:

    export        the committed tree (git archive HEAD) into <out>/work/src: everything below builds from it, so
                  nothing untracked or left over from earlier builds reaches a package. The working tree must be clean.
    tools         a tools environment with the pinned ziglang of release.yml, build, cmake and ninja (kept between runs)
    native        build_native.py --target win-x64 --run: iai_actuator_driver.dll, the command line, the C++ tests
                  and the C smoke test, which run
    library       check_package.py binary: target, exports, no C++ runtime of its own
    sdk           make_sdk.py: the C/C++ SDK zip; sdk-smoke: unpacks it and builds and runs the C smoke test and the
                  frozen C program of ABI 1.0 (core/abi/1.0, with its own 1.0 header) against it with CMake
                  (find_package), the compiler being zig cc (release.yml uses MSVC)
    linux-native  build_native.py --target linux-x64 (zig cross-compiles here: x86_64-linux-gnu.2.17), then ec_tests,
                  c_smoke and the command line run in WSL
    linux-library check_package.py binary: ELF x86-64, exports, no C++ runtime of its own, glibc 2.17 at most, timed
                  waits on the monotonic clock (pthread_cond_timedwait only with pthread_condattr_setclock)
    linux-sdk     make_sdk.py for linux-x64; linux-sdk-smoke: in WSL, the C smoke test and the frozen ABI 1.0 program
                  built against it with CMake (find_package, gcc), and the C smoke test with the compile line of its
                  README
    sdist         the sdist (source, needs the native library: none inside), checked, installed in a new environment;
                  without a library its import is refused, and with the DLL its command line runs on the simulator
    wheel         the py3-none-win_amd64 wheel with the DLL inside, checked, installed in a new environment, the whole
                  Python suite run against it with the source package moved out of the way (tests/run_backend.py)
    linux-wheel   the same in WSL for the py3-none-manylinux_2_17_x86_64 wheel with the .so inside, plus auditwheel show
    nuget         the C# tests against the DLL, then the NuGet package IaiActuatorDriver with runtimes/win-x64/native
                  and runtimes/linux-x64/native, checked
    nuget-smoke   new net8.0 and net48 programs reference the package from a local feed and load the packaged library
    linux-nuget-smoke  in WSL: a new net8.0 program does the same with the package's linux-x64 library
    gui           the gui job of release.yml: a new environment with the win-x64 wheel just built and the pinned
                  PyInstaller, apps/gui/iai-gui.spec, a check that the bundle carries iai_actuator_driver.dll,
                  iai-gui.exe --version, a short --sim-only --no-browser run on a free port that answers HTTP 200 and is
                  stopped (POST /api/quit), then iai-gui-<version>-win-x64.zip (make_gui_zip.py) in <out>/work/dist/gui.
                  The exe embeds the Python its environment is made from (--gui-python, else --python, else the one that
                  runs this script; the summary says which). The GUI tests run only with --gui-tests (about 6 minutes;
                  a few are timing-sensitive). Skipped with a note when this is not Windows or there is no spec
    collect       collect_release.py: the assets in <out>/release, the notes (the CHANGELOG.md section of the version
                  and an asset table) in <out>/release-notes.md

Then it prints the gh release create command that would publish them. It never runs gh, never pushes and never tags.
No step opens a serial port: every test runs on the simulator.

The Python and GUI suites run through the runners of tools/test.py and honour tools/load_sensitive.txt as it does: a
listed test runs after the others, alone, and once more when it failed; the summary names a test that passed only on
that retry.

--targets chooses what is built (default: both when WSL and the Linux test kit are available, else win-x64 and a note
says why). The linux-x64 steps that must run on Linux run in a WSL 2 distro through the kit (see --wsl-kit); the
distro needs python3 with venv, gcc, cmake, rsync and the .NET 8 SDK, and network access for pip and NuGet packages.

Output (--out, default build/release-local/<version> in the repository; an existing folder of an earlier run is
replaced):
    release/            the files to attach: wheels, sdist, nupkg, SDK zips, the GUI zip
    release-notes.md    the release notes
    logs/               one log per command; summary.txt, gh-release-create.txt
    work/               the exported tree, build folders and environments (can be deleted after the release)

A step that passed is recorded in <out>/state.json with the commit and the targets. --resume skips the recorded steps, so
a failure late in the run (some C++, C# and timing-sensitive tests can fail on a busy PC) costs only the failed step and
those after it. Running a step again drops the record of the steps after it. A commit made after the failure may change
.github/release/ only (the fix of the script that failed): --resume, --only and --skip then refresh
work/src/.github/release and the summary says which commit the earlier steps were built at. The command is printed only
when every step has passed for the commit.

The notes say so when the release has one platform only and, if the gui step was skipped, that the GUI is not in it.
Needs: Windows x64, Python 3.11 or later, git, the .NET 8 SDK (and .NET Framework 4.8 for the net48 smoke), and network
access for pip and NuGet packages; for linux-x64 also WSL 2 with the distro and the kit.
"""

from __future__ import annotations

import argparse
import functools
import importlib.util
import io
import json
import os
import re
import shlex
import shutil
import socket
import stat
import subprocess
import sys
import tarfile
import threading
import time
import urllib.error
import urllib.request
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import build_native  # noqa: E402  (same folder)
import check_package  # noqa: E402
import collect_release  # noqa: E402
import make_gui_zip  # noqa: E402

TOOLS_TEST = REPO / "tools" / "test.py"  # parse_counts and retried_tests read what the test runners print

RID = "win-x64"
WHEEL_PLATFORM = "win_amd64"
LIB = "iai_actuator_driver.dll"
CLI = "iai-actuator-driver-cpp.exe"
LINUX_RID = "linux-x64"
LINUX_WHEEL_PLATFORM = "manylinux_2_17_x86_64.manylinux2014_x86_64"  # release.yml: matrix plat of linux-x64
LINUX_LIB = "libiai_actuator_driver.so"
LINUX_CLI = "iai-actuator-driver-cpp"
TARGET_NAMES = [RID, LINUX_RID]
DEFAULT_OUT_ROOT = REPO / "build" / "release-local"  # build/ is ignored by git
WSL_KIT_ENV = "IAD_LINUX_VM"  # the Linux test kit when --wsl-kit is not given
DEFAULT_WSL_DISTRO = "Ubuntu-24.04"
WSL_SCRIPT = ".github/release/local_release_wsl.sh"
STAGE_NAME = "iai-release-linux"  # the folder run.ps1 copies into the distro (~/work/<this name>)
MARKER = ".local_release"
STATE = "state.json"
# gui is the last step before collect: it only makes an input of collect, so running it again drops no other record
STEPS = ["export", "tools", "native", "library", "sdk", "sdk-smoke", "linux-native", "linux-library", "linux-sdk",
         "linux-sdk-smoke", "sdist", "wheel", "linux-wheel", "nuget", "nuget-smoke", "linux-nuget-smoke", "gui",
         "collect"]
# the steps that belong to one target; the others (export, tools, sdist, nuget, collect) run whatever the targets are
STEP_TARGET = {"native": RID, "library": RID, "sdk": RID, "sdk-smoke": RID, "wheel": RID, "nuget-smoke": RID,
               "linux-native": LINUX_RID, "linux-library": LINUX_RID, "linux-sdk": LINUX_RID,
               "linux-sdk-smoke": LINUX_RID, "linux-wheel": LINUX_RID, "linux-nuget-smoke": LINUX_RID, "gui": RID}
RELEASE_SCRIPTS = ".github/release/"  # the only folder a --resume may find changed since the recorded commit
FROZEN_SMOKE = "core/abi/1.0/c_smoke.c"  # the frozen C program of ABI 1.0 (its 1.0 header is beside it)
SMOKE_TESTS = ("c_smoke", "c_smoke_1_0")  # the ctest names of .github/release/sdk_consumer: the live and the frozen one
GUI_SPEC = "apps/gui/iai-gui.spec"
GUI_ZIP = ".github/release/make_gui_zip.py"  # the GUI zip: exe, README.txt and the licence files (release.yml too)
GUI_BUNDLED_DLL = "iai_actuator_driver/_native/win-x64/iai_actuator_driver.dll"  # inside iai-gui.exe (the spec's layout)
GUI_TOKEN_HEADER = "X-EC-Token"  # apps/gui/server.py TOKEN_HEADER: the per-run access key of an /api request
GUI_START_SECS = 90  # onefile exe: it unpacks itself first
GUI_STOP_SECS = 30
# what the child programs must not inherit: they would point a test at another library or another backend
DROP_ENV = {"IAI_ACTUATOR_DRIVER_NATIVE", "IAI_DRIVER_BACKEND", "PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV",
            "IAD_WHEEL_PLATFORM", "IAD_BUILD_DIR"}
# the note of the release notes when it has one platform only
PLATFORM_NOTES = {
    (RID,): ("Windows x64 only: this release has no Linux x64 wheel, SDK or NuGet runtime. "
             "The NuGet package carries `runtimes/win-x64/native` only."),
    (LINUX_RID,): ("Linux x64 only: this release has no Windows x64 wheel, SDK or NuGet runtime. "
                   "The NuGet package carries `runtimes/linux-x64/native` only."),
}


class Failure(Exception):
    """A step failed; the message says what and where the log is."""


@dataclass(frozen=True)
class Wsl:
    """The WSL distro and the Linux test kit (see --wsl-kit) the linux-x64 steps use."""
    kit: Path
    distro: str

    @property
    def run_ps1(self) -> Path:
        return self.kit / "run.ps1"


@dataclass
class Ctx:
    args: argparse.Namespace
    commit: str
    branch: str
    version: str
    nuget_version: str
    tag: str
    prerelease: bool
    out: Path
    tools: Path
    python: str  # the interpreter the tools environment and the test environments are made from
    zig_version: str
    dotnet_version: str
    targets: list[str]  # TARGET_NAMES that are built (--targets)
    wsl: Wsl | None  # the distro and kit of the linux-x64 steps; None when linux-x64 is not built
    notes: list[str] = field(default_factory=list)  # what the summary says about the choice of targets
    pyinstaller_version: str = ""  # release.yml PYINSTALLER_VERSION
    release_python: str = ""  # release.yml PYTHON_VERSION: the Python its gui job puts inside iai-gui.exe
    gui_python: str = ""  # --gui-python: the interpreter the GUI environment is made from (and iai-gui.exe embeds); "": python
    built: str = ""  # the commit the recorded steps were built at, when a --resume went on past a change of .github/release/
    logs: Path = field(init=False)
    work: Path = field(init=False)
    src: Path = field(init=False)
    counter: int = 0
    facts: list[list[str]] = field(default_factory=list)  # [what, result] of the step running now
    done: dict[str, dict] = field(default_factory=dict)  # the steps that passed for this commit (state.json)

    def __post_init__(self) -> None:
        self.logs = self.out / "logs"
        self.work = self.out / "work"
        self.src = self.work / "src"

    @property
    def built_at(self) -> str:
        return self.built or self.commit

    @property
    def tools_python(self) -> str:
        return str(self.tools / "Scripts" / "python.exe")

    def tool(self, name: str) -> str:
        """A program of the tools environment (Popen looks programs up on this process's PATH, not the child's)."""
        return str(self.tools / "Scripts" / f"{name}.exe")

    @property
    def sdk_name(self) -> str:
        return f"iai_actuator_driver-sdk-{self.version}-{RID}"

    @property
    def linux_sdk_name(self) -> str:
        return f"iai_actuator_driver-sdk-{self.version}-{LINUX_RID}"

    @property
    def steps(self) -> list[str]:
        """The steps of this run: those of the chosen targets and the ones every run has."""
        return [s for s in STEPS if STEP_TARGET.get(s, "") in ("", *self.targets)]

    @property
    def native_dist(self) -> Path:
        return self.work / "native" / RID

    @property
    def linux_native_dist(self) -> Path:
        return self.work / "native" / LINUX_RID

    def env(self, **extra: str) -> dict[str, str]:
        env = {k: v for k, v in os.environ.items() if k.upper() not in DROP_ENV}
        env.update({"PYTHONUTF8": "1", "PIP_DISABLE_PIP_VERSION_CHECK": "1", "DOTNET_CLI_TELEMETRY_OPTOUT": "1",
                    "DOTNET_NOLOGO": "1", "DOTNET_SKIP_FIRST_TIME_EXPERIENCE": "1", "MSBUILDDISABLENODEREUSE": "1",
                    "DOTNET_CLI_USE_MSBUILD_SERVER": "0"})
        env.update(extra)
        return env

    def fact(self, name: str, text: str) -> None:
        self.facts.append([name, text])


def say(text: str = "") -> None:
    print(text, flush=True)


def git(*args: str) -> str:
    proc = subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, encoding="utf-8",
                          errors="replace")
    if proc.returncode != 0:
        raise Failure(f"git {' '.join(args)} failed: {proc.stderr.strip()}")
    return proc.stdout


def quote(args: list[str]) -> str:
    return subprocess.list2cmdline([str(a) for a in args])


def run(ctx: Ctx, label: str, cmd: list[str], cwd: Path, env: dict[str, str] | None = None,
        timeout: int = 1200) -> str:
    """Run one command; its output goes to logs/NN-<label>.log (and the console with --verbose). Returns the output.
    A non-zero exit, or more than `timeout` seconds, fails the step."""
    ctx.counter += 1
    log = ctx.logs / f"{ctx.counter:02d}-{label}.log"
    lines: list[str] = []
    t0 = time.monotonic()
    with log.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(f"$ {quote(cmd)}\n  in {cwd}\n\n")
        try:
            proc = subprocess.Popen([str(c) for c in cmd], cwd=cwd, env=env or ctx.env(), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        except OSError as exc:
            raise Failure(f"{label}: cannot start {cmd[0]}: {exc}") from exc

        def pump() -> None:
            assert proc.stdout is not None
            for line in proc.stdout:
                lines.append(line)
                fh.write(line)
                fh.flush()
                if ctx.args.verbose:
                    print("    " + line.rstrip(), flush=True)

        reader = threading.Thread(target=pump, daemon=True)
        reader.start()
        timed_out = False
        try:
            code = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
            code = proc.wait()
        reader.join(30)
        secs = time.monotonic() - t0
        fh.write(f"\n[exit {code}{', timed out after %d s' % timeout if timed_out else ''}; {secs:.1f} s]\n")
    text = "".join(lines)
    if code != 0:
        tail = "".join(lines[-40:]).rstrip()
        why = f"timed out after {timeout} s" if timed_out else f"exit {code}"
        raise Failure(f"{label} failed ({why}): {quote(cmd)}\n{tail}\n(full output: {log})")
    return text


@functools.cache
def tools_test():
    """tools/test.py, loaded by path (``test`` is the name of the standard library's own package)."""
    spec = importlib.util.spec_from_file_location("iad_tools_test", TOOLS_TEST)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look the module up by name
    spec.loader.exec_module(module)
    return module


def suite_summary(suite: str, text: str) -> str:
    """What the output of a test runner of tools/test.py (``suite``: python, gui) says, as tools/test.py reads it: the
    counts, the seconds, and the load-sensitive tests that passed only on their retry."""
    tool = tools_test()
    summary = tool.parse_counts(suite, text) or "ran"
    took = last(text, r"^ran \d+ tests in ([\d.]+) s:", re.M)
    if took:
        summary += f" in {float(took.group(1)):.0f} s"
    retried = tool.retried_tests(text)
    if retried:
        summary += "; passed only on its retry (load-sensitive): " + ", ".join(retried)
    return summary


def rmtree(path: Path) -> None:
    def onerror(func, p, _exc):  # read-only files (git objects, NuGet caches)
        os.chmod(p, stat.S_IWRITE)
        func(p)

    if path.exists():
        shutil.rmtree(path, onerror=onerror)


def need(path: Path, step: str) -> Path:
    if not path.exists():
        raise Failure(f"{path} is missing: run the step '{step}' first")
    return path


def need_src(ctx: Ctx) -> Path:
    """The exported tree, which must be the one of HEAD (a partial run can find an older export)."""
    marker = ctx.work / "src.commit"
    if not ctx.src.is_dir() or not marker.is_file() or marker.read_text(encoding="utf-8").strip() != ctx.commit:
        raise Failure(f"{ctx.src} is not an export of {ctx.commit[:7]}: run the step 'export' first")
    return ctx.src


def one(folder: Path, pattern: str) -> Path:
    found = sorted(folder.glob(pattern))
    if len(found) != 1:
        raise Failure(f"expected one {pattern} in {folder}, found {[p.name for p in found]}")
    return found[0]


def size(path: Path) -> str:
    n = path.stat().st_size
    return f"{n / 1048576:.2f} MiB" if n >= 1048576 else f"{n / 1024:.1f} KiB"


def last(text: str, pattern: str, flags: int = 0) -> re.Match | None:
    found = None
    for found in re.finditer(pattern, text, flags):
        pass
    return found


# ------------------------------------------------------------------------------------------------------------------ WSL

def find_wsl(kit: Path | None, distro: str) -> tuple[Wsl | None, str]:
    """The distro and kit when the linux-x64 steps can use them; otherwise None and the reason."""
    wsl_exe = shutil.which("wsl.exe")
    if not wsl_exe:
        return None, "wsl.exe is not installed (WSL 2 is not set up on this PC)"
    if kit is None:
        return None, f"no Linux test kit given (--wsl-kit or {WSL_KIT_ENV})"
    if not shutil.which("powershell.exe"):
        return None, "powershell.exe was not found (the kit's run.ps1 needs Windows PowerShell)"
    if not (kit / "run.ps1").is_file():
        return None, f"the Linux test kit is not at {kit}: no run.ps1 (--wsl-kit)"
    try:
        proc = subprocess.run([wsl_exe, "--list", "--quiet"], capture_output=True, timeout=60,
                              env={**os.environ, "WSL_UTF8": "1"})
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"wsl.exe --list failed: {exc}"
    # wsl.exe answers in UTF-16 unless WSL_UTF8 is honoured: dropping the NULs reads both
    text = proc.stdout.decode("utf-8", "replace").replace("\x00", "").replace("\ufeff", "")
    names = [line.strip() for line in text.splitlines() if line.strip()]
    if proc.returncode != 0 or distro not in names:
        return None, f"WSL has no distro named {distro} (--wsl-distro; wsl --list: {', '.join(names) or 'none'})"
    return Wsl(kit, distro), ""


def wsl_path(ctx: Ctx, path: Path) -> str:
    """A Windows path as the distro sees it (/mnt/c/...), converted by wslpath in the distro."""
    assert ctx.wsl is not None
    proc = subprocess.run(["wsl.exe", "-d", ctx.wsl.distro, "--exec", "wslpath", "-a", "-u", str(path)],
                          capture_output=True, text=True, encoding="utf-8", errors="replace",
                          env={**os.environ, "WSL_UTF8": "1"})
    out = proc.stdout.replace("\x00", "").strip()
    if proc.returncode != 0 or not out:
        raise Failure(f"wslpath could not convert {path} in {ctx.wsl.distro}: {out} {proc.stderr.strip()}")
    return out


def linux_stage(ctx: Ctx, inputs: dict[str, Path], consumer: bool = False) -> Path:
    """The folder the kit copies into the distro: src/ (the exported tree), in/ (the files this
    step takes from the steps before it, by the names given), consumer/ (the NuGet smoke program) and release.env (the
    values local_release_wsl.sh reads). Everything but src/ is made again for every step."""
    stage = ctx.work / STAGE_NAME
    marker = stage / "src.commit"
    if not ((stage / "src").is_dir() and marker.is_file() and marker.read_text(encoding="utf-8").strip() == ctx.commit):
        rmtree(stage)
        stage.mkdir(parents=True)
        shutil.copytree(need_src(ctx), stage / "src")
        marker.write_text(ctx.commit + "\n", encoding="utf-8")
    for name in ("in", "consumer"):
        rmtree(stage / name)
    (stage / "in").mkdir()
    for name, source in inputs.items():
        dest = stage / "in" / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if source.is_dir():
            shutil.copytree(source, dest)
        else:
            shutil.copy2(source, dest)
    if consumer:
        (stage / "consumer").mkdir()
        # @FEED@: the folder of the package, which local_release_wsl.sh knows
        (stage / "consumer" / "nuget.config").write_text(CONSUMER_CONFIG.format(feed="@FEED@"), encoding="utf-8",
                                                         newline="\n")
        (stage / "consumer" / "Consumer.csproj").write_text(
            CONSUMER_PROJECT.format(tfm="net8.0", version=ctx.nuget_version), encoding="utf-8", newline="\n")
        (stage / "consumer" / "Program.cs").write_text(CONSUMER_PROGRAM, encoding="utf-8", newline="\n")
    values = {"VERSION": ctx.version, "NUGET_VERSION": ctx.nuget_version, "RID": LINUX_RID, "LIB": LINUX_LIB,
              "WHEEL_PLATFORM": LINUX_WHEEL_PLATFORM, "SDK_NAME": ctx.linux_sdk_name, "OUT": wsl_path(ctx, ctx.work)}
    (stage / "release.env").write_text("".join(f"{k}={shlex.quote(v)}\n" for k, v in values.items()), encoding="utf-8",
                                       newline="\n")
    return stage


def run_wsl(ctx: Ctx, label: str, step: str, timeout: int = 1800) -> str:
    """Run one step of local_release_wsl.sh in the distro through the kit. Returns the output; the kit's own log of the
    run is named in it."""
    assert ctx.wsl is not None
    cmd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ctx.wsl.run_ps1), "-Project",
           str(ctx.work / STAGE_NAME), "-Profile", str(ctx.src / WSL_SCRIPT), "-ProfileArgs", step, "-Distro",
           ctx.wsl.distro]
    return run(ctx, label, cmd, ctx.work, timeout=timeout)


def wsl_environment(text: str) -> str:
    m = last(text, r"^WSL-ENV: (.*)$", re.M)
    return m.group(1).strip() if m else "?"


# ---------------------------------------------------------------------------------------------------------------- setup

def release_yml_pin(name: str) -> str:
    text = (REPO / ".github" / "workflows" / "release.yml").read_text(encoding="utf-8")
    m = re.search(rf'^\s*{name}:\s*"([^"]+)"', text, re.M)
    if not m:
        raise Failure(f"{name} not found in .github/workflows/release.yml")
    return m.group(1)


def choose_targets(args: argparse.Namespace) -> tuple[list[str], Wsl | None, list[str]]:
    """The targets to build, the WSL distro and kit when linux-x64 is one of them, and what the summary says about the
    choice. Without --targets: both when WSL and the kit are available, else win-x64 and the reason."""
    # absolute: run_wsl starts powershell.exe in <out>/work
    kit, distro = (Path(args.wsl_kit).resolve() if args.wsl_kit else None), args.wsl_distro
    if args.targets is not None:
        targets = [t for t in TARGET_NAMES if t in args.targets.split(",")]
        if not targets:
            raise Failure(f"--targets {args.targets!r} names no target; targets: {', '.join(TARGET_NAMES)}")
        wsl, why = find_wsl(kit, distro) if LINUX_RID in targets else (None, "")
        if LINUX_RID in targets and not wsl:
            raise Failure(f"--targets {args.targets} needs WSL for {LINUX_RID}, but {why}. Set WSL and the kit up "
                          f"(--wsl-kit), or build Windows only with --targets {RID}.")
        return targets, wsl, []
    wsl, why = find_wsl(kit, distro)
    if wsl:
        return list(TARGET_NAMES), wsl, []
    return [RID], None, [f"{LINUX_RID} not built: {why}. Only {RID} is built; set WSL up and run again, or pass "
                         f"--targets {RID} to say that Windows only is meant."]


def preflight(args: argparse.Namespace) -> Ctx:
    if sys.platform != "win32" or build_native.host_rid() != RID:
        raise Failure(f"this script builds the {RID} deliverables and runs on Windows x64 "
                      f"(this machine: {sys.platform}, {build_native.host_rid()})")
    if sys.version_info < (3, 11):
        raise Failure("Python 3.11 or later is needed")
    if not (REPO / ".git").exists():
        raise Failure(f"{REPO} is not a git checkout")

    dirty = git("status", "--porcelain", "--untracked-files=all").strip()
    if dirty:
        shown = "\n".join("  " + line for line in dirty.splitlines()[:20])
        raise Failure("the working tree is not clean; commit or remove these first (a release is built from one "
                      f"commit):\n{shown}")
    commit = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()

    proc = subprocess.run([sys.executable, str(HERE / "check_version.py"), "--root", str(REPO)], cwd=REPO,
                          capture_output=True, text=True, encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        raise Failure("check_version.py failed:\n" + proc.stdout + proc.stderr)
    out = dict(line.split("=", 1) for line in proc.stdout.splitlines() if re.match(r"^[a-z_]+=", line))
    version, nuget_version, tag = out["version"], out["nuget_version"], out["tag"]

    if not collect_release.changelog_section(REPO / "CHANGELOG.md", version):
        raise Failure(f"CHANGELOG.md has no section for {version} (a '## ' heading naming it): the release notes come "
                      "from it")

    gui_python = ""
    if getattr(args, "gui_python", None):
        gui_python = shutil.which(args.gui_python) or ""
        if not gui_python:
            raise Failure(f"--gui-python {args.gui_python}: no such program (a path to python.exe)")
    root = Path(args.out).resolve() if args.out else (DEFAULT_OUT_ROOT / version)
    ignored = REPO / "build"  # inside the repository, only build/ (ignored by git) may hold the output
    if root == REPO or root in REPO.parents or (REPO in root.parents and ignored not in root.parents):
        raise Failure(f"--out {root} must not be above the repository, or inside it outside build/")
    tools = Path(args.tools).resolve() if args.tools else root.parent / "_tools"

    targets, wsl, notes = choose_targets(args)
    zig_version = release_yml_pin("ZIGLANG_VERSION")
    dotnet_major = release_yml_pin("DOTNET_VERSION").split(".")[0]
    if not shutil.which("dotnet"):
        raise Failure("dotnet not found: install the .NET SDK " + release_yml_pin("DOTNET_VERSION"))
    sdks = subprocess.run(["dotnet", "--list-sdks"], capture_output=True, text=True).stdout.split()
    if not any(s.startswith(dotnet_major + ".") for s in sdks[::2]):
        raise Failure(f"no .NET SDK {dotnet_major}.x installed (dotnet --list-sdks: {sdks[::2] or 'none'})")
    return Ctx(args=args, commit=commit, branch=branch, version=version, nuget_version=nuget_version, tag=tag,
               prerelease=out["prerelease"] == "true", out=root, tools=tools, python=args.python or sys.executable,
               zig_version=zig_version, dotnet_version=release_yml_pin("DOTNET_VERSION"), targets=targets, wsl=wsl,
               notes=notes, pyinstaller_version=release_yml_pin("PYINSTALLER_VERSION"),
               release_python=release_yml_pin("PYTHON_VERSION"), gui_python=gui_python)


def changed_files(old: str, new: str) -> list[str] | None:
    """The paths that differ between two commits; None when git cannot say (a commit this repository does not have).
    --no-renames: a moved file is its old path (deleted) and its new path (added), not the new path alone: a file moved
    into .github/release/ from elsewhere is a change outside it."""
    try:
        out = git("diff", "--name-only", "--no-renames", "-z", old, new)
    except Failure:
        return None
    return [p for p in out.split(chr(0)) if p]


def only_release_scripts(paths: list[str]) -> bool:
    """True when every path is under .github/release/ (the scripts of this run: a fix of the one that failed)."""
    return all(p.startswith(RELEASE_SCRIPTS) for p in paths)


def refresh_release_scripts(ctx: Ctx, recorded: str) -> None:
    """The exported tree work/src is the one of `recorded`; HEAD differs from it in .github/release/ only. Put the
    scripts of HEAD there (every step runs the scripts of work/src), and say the tree is the one of HEAD. Without an
    export of `recorded` nothing is done: need_src then says to run the step 'export'."""
    marker = ctx.work / "src.commit"
    if not (ctx.src.is_dir() and marker.is_file() and marker.read_text(encoding="utf-8").strip() == recorded):
        return
    tar = subprocess.run(["git", "archive", "--format=tar", ctx.commit, RELEASE_SCRIPTS], cwd=REPO, capture_output=True)
    if tar.returncode != 0:
        raise Failure("git archive failed: " + tar.stderr.decode("utf-8", "replace"))
    rmtree(ctx.src / ".github" / "release")
    with tarfile.open(fileobj=io.BytesIO(tar.stdout)) as tf:
        if hasattr(tarfile, "data_filter"):
            tf.extractall(ctx.src, filter="data")
        else:
            tf.extractall(ctx.src)
    marker.write_text(ctx.commit + "\n", encoding="utf-8")


def prepare_out(ctx: Ctx, keep: bool) -> None:
    """A clean run starts with an empty output folder. A run that continues one (--resume, --only, --skip) keeps what
    the steps before it made, and only if that was made from this commit, or from one that differs from it in
    .github/release/ only (the scripts of the failed step were fixed and committed)."""
    out = ctx.out
    if out.exists() and any(out.iterdir()) and not (out / MARKER).exists():
        raise Failure(f"{out} exists and is not an output folder of this script (no {MARKER}); not using it")
    state = out / STATE
    if keep and state.is_file():
        data = json.loads(state.read_text(encoding="utf-8"))
        recorded = data.get("commit")
        carried = recorded != ctx.commit
        if carried:
            changed = changed_files(recorded, ctx.commit) if recorded else None
            if changed is None or not only_release_scripts(changed):
                other = [p for p in changed or [] if not p.startswith(RELEASE_SCRIPTS)]
                why = f" ({len(other)} file(s) outside {RELEASE_SCRIPTS} differ, e.g. {other[0]})" if other else ""
                raise Failure(f"{out} holds a run of commit {str(recorded)[:7]}, this is {ctx.commit[:7]}{why}: "
                              "start a clean run (no --resume, --only or --skip) or give another --out. Only a change "
                              f"of {RELEASE_SCRIPTS} can be carried over.")
        if data.get("targets", [RID]) != ctx.targets:  # a state file of the Windows-only script has no targets
            raise Failure(f"{out} holds a run with targets {', '.join(data.get('targets', [RID]))}, this is "
                          f"{', '.join(ctx.targets)}: start a clean run (no --resume, --only or --skip) or give "
                          "another --out")
        ctx.done = data.get("steps", {})
        ctx.built = data.get("built") or recorded
        if carried:
            refresh_release_scripts(ctx, recorded)
            # work/src now says it is the export of this commit: say so in the state too, also when no step runs (a
            # --resume of steps that all passed), or the next change of .github/release/ finds the two disagreeing
            save_state(ctx)
        if ctx.built != ctx.commit:
            text = (f"the steps that passed were built at {ctx.built[:7]}; this commit {ctx.commit[:7]} differs from it "
                    f"in {RELEASE_SCRIPTS} only, so they are kept and work/src holds the scripts of {ctx.commit[:7]}. "
                    "The libraries and packages are those of the earlier commit (build-info.json in the SDKs names "
                    "it); run a step again with --only if a script change should apply to it.")
            ctx.notes.append(text)
            say(f"note: {text}")
    elif not keep:
        rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / MARKER).write_text(f"{ctx.version} {ctx.commit}\n", encoding="utf-8")
    ctx.logs.mkdir(exist_ok=True)
    ctx.work.mkdir(exist_ok=True)
    ctx.counter = len(list(ctx.logs.glob("*.log")))  # a continued run numbers its logs after the earlier ones


def save_state(ctx: Ctx) -> None:
    # "built": the commit the steps were built at; it differs from "commit" after a --resume past a change of
    # .github/release/ (see prepare_out)
    (ctx.out / STATE).write_text(json.dumps({"commit": ctx.commit, "built": ctx.built_at, "targets": ctx.targets,
                                             "steps": ctx.done}, indent=1) + "\n", encoding="utf-8", newline="\n")


# ----------------------------------------------------------------------------------------------------------- the steps

def step_export(ctx: Ctx) -> str:
    rmtree(ctx.src)
    tar = subprocess.run(["git", "archive", "--format=tar", ctx.commit], cwd=REPO, capture_output=True)
    if tar.returncode != 0:
        raise Failure("git archive failed: " + tar.stderr.decode("utf-8", "replace"))
    with tarfile.open(fileobj=io.BytesIO(tar.stdout)) as tf:
        if hasattr(tarfile, "data_filter"):
            tf.extractall(ctx.src, filter="data")
        else:
            tf.extractall(ctx.src)
    (ctx.work / "src.commit").write_text(ctx.commit + "\n", encoding="utf-8")
    lacking = [rel for rel in check_package.NOTICES.values() if not (ctx.src / rel).is_file()]
    if lacking:
        raise Failure(f"{', '.join(lacking)} missing in {ctx.commit[:7]}: every package carries them")
    files = sum(1 for p in ctx.src.rglob("*") if p.is_file())
    return f"{files} files of {ctx.commit[:7]}"


def step_tools(ctx: Ctx) -> str:
    py = Path(ctx.tools_python)
    if not py.is_file():
        ctx.tools.parent.mkdir(parents=True, exist_ok=True)
        run(ctx, "tools-venv", [ctx.python, "-m", "venv", str(ctx.tools)], ctx.work)
    run(ctx, "tools-pip", [str(py), "-m", "pip", "install", "--quiet", f"ziglang=={ctx.zig_version}", "build", "cmake",
                           "ninja"], ctx.work, timeout=900)
    zig = run(ctx, "tools-zig", [str(py), "-m", "ziglang", "version"], ctx.work).strip().splitlines()[-1]
    if zig != ctx.zig_version:
        raise Failure(f"zig {zig} in {ctx.tools}, release.yml pins ziglang {ctx.zig_version}")
    versions = run(ctx, "tools-versions", [str(py), "-m", "pip", "list", "--format=json"], ctx.work)
    installed = {p["name"].lower(): p["version"] for p in json.loads(versions[versions.index("["):])}
    return (f"zig {zig}, build {installed.get('build', '?')}, cmake {installed.get('cmake', '?')}, "
            f"ninja {installed.get('ninja', '?')}")


def step_native(ctx: Ctx) -> str:
    build_dir = ctx.work / "native-build" / RID
    cmd = [ctx.tools_python, ".github/release/build_native.py", "--target", RID, "--core", "core", "--out",
           str(build_dir), "--run"]
    if ctx.args.jobs:
        cmd += ["--jobs", str(ctx.args.jobs)]
    # build-info.json (in the SDK zip too) records the commit and the tag, as the workflow's GITHUB_SHA and
    # GITHUB_REF_NAME do on a runner
    text = run(ctx, "native", cmd, ctx.src, env=ctx.env(GITHUB_SHA=ctx.commit, GITHUB_REF_NAME=ctx.tag), timeout=2400)
    rmtree(ctx.native_dist)
    shutil.copytree(build_dir / "dist", ctx.native_dist)
    tests = last(text, r"^\d+ tests, .*$", re.M)
    ctx.fact("C++ tests (ec_tests) and C smoke test", (tests.group(0) if tests else "passed") + "; c_smoke passed")
    return f"{LIB} {size(ctx.native_dist / LIB)}"


def step_library(ctx: Ctx) -> str:
    need(ctx.native_dist / LIB, "native")
    check = ".github/release/check_package.py"
    run(ctx, "check-library", [ctx.tools_python, check, "binary", str(ctx.native_dist / LIB), "--rid", RID,
                               "--library"], ctx.src)
    run(ctx, "check-cli", [ctx.tools_python, check, "binary", str(ctx.native_dist / CLI), "--rid", RID], ctx.src)
    return "library and command line"


def step_sdk(ctx: Ctx) -> str:
    need(ctx.native_dist / LIB, "native")
    out = ctx.work / "dist" / "sdk"
    run(ctx, "sdk", [ctx.tools_python, ".github/release/make_sdk.py", "--rid", RID, "--version", ctx.version,
                     "--native", str(ctx.native_dist), "--out", str(out)], ctx.src)
    return one(out, "*.zip").name


def ctest_passed(text: str) -> None:
    """The ctest output of sdk_consumer names every test that passed: both programs must be there, so that a target
    left out of the build cannot pass as a smoke test."""
    for name in SMOKE_TESTS:
        if not re.search(rf"^\s*\d+/\d+ Test\s+#\d+: {name} \.+\s*Passed", text, re.M):
            raise Failure(f"ctest did not report {name} as passed")


def step_sdk_smoke(ctx: Ctx) -> str:
    zip_path = need(one(ctx.work / "dist" / "sdk", "*.zip"), "sdk")
    unpacked = ctx.work / "sdk-unpacked"
    rmtree(unpacked)
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(unpacked)
    prefix = need(unpacked / ctx.sdk_name, "sdk")
    # the compiler of this check is zig cc: CMake runs the wrapper like any other C compiler
    wrapper = ctx.work / "zigcc" / "zig-cc.cmd"
    wrapper.parent.mkdir(exist_ok=True)
    wrapper.write_text(f'@echo off\r\n"{ctx.tools_python}" -m ziglang cc -target x86_64-windows-gnu %*\r\n',
                       encoding="utf-8", newline="")
    env = ctx.env(PATH=str(ctx.tools / "Scripts") + os.pathsep + os.environ.get("PATH", ""))
    build = ctx.work / "sdk_consumer"
    rmtree(build)
    run(ctx, "sdk-cmake", [ctx.tool("cmake"), "-S", ".github/release/sdk_consumer", "-B", str(build), "-G", "Ninja",
                           f"-DCMAKE_C_COMPILER={wrapper}", "-DCMAKE_BUILD_TYPE=Release",
                           f"-DCMAKE_PREFIX_PATH={prefix.as_posix()}",
                           f"-DIAD_SMOKE_SOURCE={(ctx.src / 'core' / 'tests' / 'c_smoke.c').as_posix()}",
                           f"-DIAD_FROZEN_SMOKE_SOURCE={(ctx.src / FROZEN_SMOKE).as_posix()}"],
        ctx.src, env=env)
    run(ctx, "sdk-build", [ctx.tool("cmake"), "--build", str(build), "--config", "Release"], ctx.src, env=env)
    text = run(ctx, "sdk-ctest", [ctx.tool("ctest"), "--test-dir", str(build), "-C", "Release", "--output-on-failure"], ctx.src,
               env=env)
    ctest_passed(text)
    m = last(text, r"^\d+% tests passed.*$", re.M)
    ctx.fact("C smoke test and the frozen ABI 1.0 C program built against the unpacked SDK (CMake find_package, zig cc)",
             m.group(0) if m else "passed")
    return "c_smoke and c_smoke_1_0 ran against the unpacked SDK"


def step_linux_native(ctx: Ctx) -> str:
    build_dir = ctx.work / "native-build" / LINUX_RID
    cmd = [ctx.tools_python, ".github/release/build_native.py", "--target", LINUX_RID, "--core", "core", "--out",
           str(build_dir)]
    if ctx.args.jobs:
        cmd += ["--jobs", str(ctx.args.jobs)]
    # zig cross-compiles on this PC (x86_64-linux-gnu.2.17, as on release.yml's ubuntu runner); --run is left out because
    # the tests do not run on Windows: they run in WSL below
    run(ctx, "linux-native", cmd, ctx.src, env=ctx.env(GITHUB_SHA=ctx.commit, GITHUB_REF_NAME=ctx.tag), timeout=2400)
    rmtree(ctx.linux_native_dist)
    shutil.copytree(build_dir / "dist", ctx.linux_native_dist)
    need(ctx.linux_native_dist / LINUX_LIB, "linux-native")
    linux_stage(ctx, {"native-build/test": build_dir / "test", "native-build/dist": build_dir / "dist"})
    text = run_wsl(ctx, "linux-native-tests", "native")
    tests = last(text, r"^\d+ tests, .*$", re.M)
    smoke = last(text, r"^c_smoke: .*$", re.M)
    ctx.fact("Linux test environment (WSL)", wsl_environment(text))
    ctx.fact("C++ tests (ec_tests) and C smoke test, run in WSL",
             f"{tests.group(0) if tests else 'passed'}; {smoke.group(0) if smoke else 'c_smoke passed'}")
    return f"{LINUX_LIB} {size(ctx.linux_native_dist / LINUX_LIB)}"


def step_linux_library(ctx: Ctx) -> str:
    need(ctx.linux_native_dist / LINUX_LIB, "linux-native")
    check = ".github/release/check_package.py"
    run(ctx, "check-linux-library", [ctx.tools_python, check, "binary", str(ctx.linux_native_dist / LINUX_LIB), "--rid",
                                     LINUX_RID, "--library"], ctx.src)
    run(ctx, "check-linux-cli", [ctx.tools_python, check, "binary", str(ctx.linux_native_dist / LINUX_CLI), "--rid",
                                 LINUX_RID], ctx.src)
    return "library and command line"


def step_linux_sdk(ctx: Ctx) -> str:
    need(ctx.linux_native_dist / LINUX_LIB, "linux-native")
    out = ctx.work / "dist" / f"sdk-{LINUX_RID}"
    rmtree(out)
    run(ctx, "linux-sdk", [ctx.tools_python, ".github/release/make_sdk.py", "--rid", LINUX_RID, "--version", ctx.version,
                           "--native", str(ctx.linux_native_dist), "--out", str(out)], ctx.src)
    return one(out, "*.zip").name


def step_linux_sdk_smoke(ctx: Ctx) -> str:
    zip_path = need(one(ctx.work / "dist" / f"sdk-{LINUX_RID}", "*.zip"), "linux-sdk")
    if zip_path.name != f"{ctx.linux_sdk_name}.zip":
        raise Failure(f"unexpected SDK zip {zip_path.name}")
    linux_stage(ctx, {f"sdk/{zip_path.name}": zip_path})
    text = run_wsl(ctx, "linux-sdk-smoke", "sdk-smoke")
    ctest_passed(text)
    ctest = last(text, r"^\d+% tests passed.*$", re.M)
    manual = last(text, r"^c_smoke: .*$", re.M)
    ctx.fact("C smoke test and the frozen ABI 1.0 C program built against the SDK unpacked in WSL (CMake find_package, gcc)",
             ctest.group(0) if ctest else "passed")
    ctx.fact("C smoke test built with the compile line of the SDK's README.txt as written, run from the SDK folder",
             manual.group(0) if manual else "passed")
    return "c_smoke and c_smoke_1_0 ran against the unpacked SDK"


SDIST_NO_LIBRARY = (  # run by the sdist's Python with no library named: the import must be refused, saying why
    'try:\n'
    '    import iai_actuator_driver\n'
    'except ImportError as exc:\n'
    "    assert 'needs its native library' in str(exc), exc\n"
    "    print('without the library: ImportError:', str(exc).splitlines()[0])\n"
    'else:\n'
    "    raise SystemExit('the driver imported without its native library')\n"
)


def step_sdist(ctx: Ctx) -> str:
    dist = ctx.work / "dist" / "sdist"
    rmtree(dist)
    # a copy of bindings/python: the build leaves build/ and egg-info folders behind, and the wheel is built apart
    stage = ctx.work / "sdist-src"
    rmtree(stage)
    shutil.copytree(ctx.src / "bindings" / "python", stage, ignore=shutil.ignore_patterns("__pycache__"))
    run(ctx, "sdist-build", [ctx.tools_python, "-m", "build", "--sdist", "--outdir", str(dist), str(stage)], ctx.work,
        timeout=900)
    sdist = one(dist, "*.tar.gz")
    run(ctx, "sdist-check", [ctx.tools_python, ".github/release/check_package.py", "sdist", str(sdist), "--version",
                             ctx.version], ctx.src)
    venv = ctx.work / "venv-sdist"
    rmtree(venv)
    run(ctx, "sdist-venv", [ctx.python, "-m", "venv", str(venv)], ctx.work)
    py = venv / "Scripts" / "python.exe"
    cwd = ctx.work / "tmp-sdist"  # away from every source package
    cwd.mkdir(exist_ok=True)
    run(ctx, "sdist-install", [str(py), "-m", "pip", "install", "--quiet", str(sdist)], cwd, timeout=900)
    # source, needs the native library (D8): none inside, so the import is refused until one is named
    run(ctx, "sdist-no-library", [str(py), "-c", SDIST_NO_LIBRARY], cwd)
    lib = need(ctx.native_dist / LIB, "native")
    with_lib = ctx.env(IAI_ACTUATOR_DRIVER_NATIVE=str(lib))
    exe = venv / "Scripts" / "iai-actuator-driver.exe"
    version = run(ctx, "sdist-version", [str(exe), "--version"], cwd, env=with_lib).strip()
    run(ctx, "sdist-sim-status", [str(exe), "--sim", "status"], cwd, env=with_lib)
    ctx.fact("sdist installed in a new environment", f"no native library inside, import refused without one; with "
                                                      f"the DLL the command line reports {version!r}; --sim status ran")
    return f"{sdist.name} {size(sdist)}"


def step_wheel(ctx: Ctx) -> str:
    need(ctx.native_dist / LIB, "native")
    dist = ctx.work / "dist" / "wheel"
    rmtree(dist)
    # the library goes into a copy of bindings/python (release.yml: into the checkout, which is clean there)
    stage = ctx.work / "wheel-src"
    rmtree(stage)
    shutil.copytree(ctx.src / "bindings" / "python", stage, ignore=shutil.ignore_patterns("__pycache__"))
    native_dir = stage / "iai_actuator_driver" / "_native" / RID
    native_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ctx.native_dist / LIB, native_dir / LIB)
    run(ctx, "wheel-build", [ctx.tools_python, "-m", "build", "--wheel", "--outdir", str(dist), str(stage)], ctx.work,
        env=ctx.env(IAD_WHEEL_PLATFORM=WHEEL_PLATFORM), timeout=900)
    wheel = one(dist, "*.whl")
    if f"-py3-none-{WHEEL_PLATFORM}.whl" not in wheel.name:
        raise Failure(f"unexpected wheel name {wheel.name}")
    check = ".github/release/check_package.py"
    run(ctx, "wheel-check", [ctx.tools_python, check, "wheel", str(wheel), "--rid", RID, "--version", ctx.version],
        ctx.src)

    venv = ctx.work / "venv-wheel"
    rmtree(venv)
    run(ctx, "wheel-venv", [ctx.python, "-m", "venv", str(venv)], ctx.work)
    py = str(venv / "Scripts" / "python.exe")
    run(ctx, "wheel-install", [py, "-m", "pip", "install", "--quiet", str(wheel)], ctx.work, timeout=900)

    # the tests must import the installed wheel, not the source package next to them
    pkg_dir = ctx.src / "bindings" / "python"
    source_pkg = pkg_dir / "iai_actuator_driver"
    parked = ctx.work / "source-package"
    rmtree(parked)
    shutil.move(str(source_pkg), str(parked))
    try:
        run(ctx, "wheel-installed-check", [py, str(ctx.src / ".github" / "release" / "check_package.py"), "installed",
                                           "--rid", RID, "--version", ctx.version], pkg_dir)
        # the runner of tools/test.py: the load-sensitive tests after the others, a failed one once more
        text = run(ctx, "wheel-tests", [py, "tests/run_backend.py", "--backend", "native"], pkg_dir, timeout=1800)
    finally:
        shutil.move(str(parked), str(source_pkg))
    ctx.fact("Python suite against the installed wheel (source package out of the way)", suite_summary("python", text))
    return f"{wheel.name} {size(wheel)}"


def step_linux_wheel(ctx: Ctx) -> str:
    lib = need(ctx.linux_native_dist / LINUX_LIB, "linux-native")
    dist = ctx.work / "dist" / f"wheel-{LINUX_RID}"
    rmtree(dist)
    dist.mkdir(parents=True)
    linux_stage(ctx, {f"native/{LINUX_RID}/{LINUX_LIB}": lib})
    # in WSL (release.yml: the ubuntu job): build, check_package.py wheel, auditwheel show, a new venv, the whole Python
    # suite against the installed wheel with the source package out of the way; the wheel is copied to `dist`
    text = run_wsl(ctx, "linux-wheel", "wheel", timeout=3000)
    wheel = one(dist, "*.whl")
    if f"-py3-none-{LINUX_WHEEL_PLATFORM}.whl" not in wheel.name:
        raise Failure(f"unexpected wheel name {wheel.name}")
    audit = last(text, r"^AUDITWHEEL: (.*)$", re.M)
    ctx.fact("manylinux check of the wheel (auditwheel show)", audit.group(1) if audit else "?")
    ctx.fact("Python suite in WSL against the installed linux-x64 wheel (source package out of the way)",
             suite_summary("python", text))
    return f"{wheel.name} {size(wheel)}"


def step_nuget(ctx: Ctx) -> str:
    if RID in ctx.targets:
        need(ctx.native_dist / LIB, "native")
        dll = (ctx.native_dist / LIB).as_posix()
        text = run(ctx, "dotnet-test", ["dotnet", "test", "bindings/dotnet/IaiActuatorDriver.sln", "--configuration",
                                        "Release", f"-p:NativeLibraryPath={dll}"], ctx.src, timeout=1800)
        totals = re.findall(r"(Passed|Failed)!\s+-\s+Failed:\s*(\d+),\s*Passed:\s*(\d+),\s*Skipped:\s*(\d+),\s*Total:\s*(\d+)",
                            text)
        if not totals:
            raise Failure("dotnet test printed no result line (see logs/*-dotnet-test.log)")
        ctx.fact("C# tests against the DLL (dotnet test, IaiActuatorDriver.sln)",
                 "; ".join(f"{p} passed, {f} failed, {s} skipped of {t}" for _, f, p, s, t in totals))
    if LINUX_RID in ctx.targets:
        need(ctx.linux_native_dist / LINUX_LIB, "linux-native")

    dist = ctx.work / "dist" / "nuget"
    rmtree(dist)
    # the package carries the libraries of the chosen targets (work/native/<rid>): packed without IadRequireNative,
    # check_package.py --rids checks that each of them is in it
    run(ctx, "dotnet-pack", ["dotnet", "pack", "bindings/dotnet/src/IaiActuatorDriver/IaiActuatorDriver.csproj",
                             "--configuration", "Release", "--output", str(dist), f"-p:Version={ctx.nuget_version}",
                             f"-p:IadNativeDir={(ctx.work / 'native').as_posix()}/", "-p:IadRequireNative=false"],
        ctx.src, timeout=900)
    nupkg = dist / f"IaiActuatorDriver.{ctx.nuget_version}.nupkg"
    need(nupkg, "nuget")
    run(ctx, "nupkg-check", [ctx.tools_python, ".github/release/check_package.py", "nupkg", str(nupkg), "--version",
                             ctx.nuget_version, "--rids", ",".join(ctx.targets)], ctx.src)
    return f"{nupkg.name} {size(nupkg)}"


CONSUMER_CONFIG = """<configuration>
  <packageSources>
    <clear />
    <add key="release" value="{feed}" />
    <add key="nuget.org" value="https://api.nuget.org/v3/index.json" />
  </packageSources>
</configuration>
"""
CONSUMER_PROJECT = """<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>Exe</OutputType>
    <TargetFramework>{tfm}</TargetFramework>
    <PlatformTarget Condition="'$(TargetFramework)' == 'net48'">x64</PlatformTarget>
  </PropertyGroup>
  <ItemGroup>
    <PackageReference Include="IaiActuatorDriver" Version="{version}" />
  </ItemGroup>
</Project>
"""
CONSUMER_PROGRAM = """using System;
using IaiActuatorDriver.Native;

internal static class Program
{
    private static int Main()
    {
        NativeDriver.EnsureLoaded();
        Console.WriteLine("iai_actuator_driver " + NativeDriver.LibraryVersion);
        return 0;
    }
}
"""


def step_nuget_smoke(ctx: Ctx) -> str:
    feed = need(ctx.work / "dist" / "nuget", "nuget")
    # a package folder of its own: a version restored by an earlier run must not stand in for this package
    packages = ctx.work / "nuget-packages"
    rmtree(packages)
    seen = []
    for tfm in ("net8.0", "net48"):
        folder = ctx.work / f"consumer-{tfm}"
        rmtree(folder)
        folder.mkdir(parents=True)
        (folder / "nuget.config").write_text(CONSUMER_CONFIG.format(feed=feed), encoding="utf-8", newline="\n")
        (folder / "Consumer.csproj").write_text(CONSUMER_PROJECT.format(tfm=tfm, version=ctx.nuget_version),
                                                encoding="utf-8", newline="\n")
        (folder / "Program.cs").write_text(CONSUMER_PROGRAM, encoding="utf-8", newline="\n")
        text = run(ctx, f"nuget-smoke-{tfm}", ["dotnet", "run", "--project", str(folder / "Consumer.csproj"),
                                               "--configuration", "Release"], folder,
                   env=ctx.env(NUGET_PACKAGES=str(packages)), timeout=900)
        m = last(text, r"^iai_actuator_driver (\S+)\s*$", re.M)
        if not m:
            raise Failure(f"the {tfm} consumer did not print the library version")
        seen.append(f"{tfm}: library {m.group(1)}")
    ctx.fact("NuGet install smoke (new programs, local feed)", "; ".join(seen))
    return "; ".join(seen)


def step_linux_nuget_smoke(ctx: Ctx) -> str:
    nupkg = need(ctx.work / "dist" / "nuget" / f"IaiActuatorDriver.{ctx.nuget_version}.nupkg", "nuget")
    linux_stage(ctx, {f"nuget/{nupkg.name}": nupkg}, consumer=True)
    text = run_wsl(ctx, "linux-nuget-smoke", "nuget-smoke", timeout=1800)
    m = last(text, r"^iai_actuator_driver (\S+)\s*$", re.M)
    packaged = last(text, r"^PACKAGED-LIBRARY: (.*)$", re.M)
    if not m or not packaged:
        raise Failure("the net8.0 consumer in WSL did not print the library version (see logs/*-linux-nuget-smoke.log)")
    ctx.fact("NuGet install smoke in WSL (new net8.0 program, local feed)",
             f"net8.0 on Linux: library {m.group(1)}, {packaged.group(1)}")
    return f"net8.0 on Linux: library {m.group(1)}"


# run by the Python of the GUI environment: is the native library in iai-gui.exe? (release.yml: the same check, inline)
GUI_BUNDLE_CHECK = """import sys
from PyInstaller.archive.readers import CArchiveReader
names = [n.replace(chr(92), "/") for n in CArchiveReader(sys.argv[1]).toc]
hits = [n for n in names if n.endswith(sys.argv[2])]
print(chr(10).join(hits) or "no " + sys.argv[2] + " in the bundle")
raise SystemExit(0 if hits else 1)
"""


def gui_skip_reason(platform: str, spec_exists: bool) -> str | None:
    """Why the gui step builds nothing (None: it builds). iai-gui.exe is a Windows program; release.yml leaves the gui
    job out, and the release has no GUI zip, while apps/gui/iai-gui.spec does not exist."""
    if platform != "win32":
        return f"iai-gui.exe is built on Windows only (this is {platform})"
    if not spec_exists:
        return f"{GUI_SPEC} does not exist (release.yml skips its gui job then too)"
    return None


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def gui_request(url: str, token: str | None = None, body: bytes | None = None) -> tuple[int, bytes]:
    """One request to the GUI on 127.0.0.1, never through a proxy: (status, body); (0, b"") when nothing answers."""
    headers = {}
    if token:
        headers[GUI_TOKEN_HEADER] = token
    if body is not None:
        headers["Content-Type"] = "application/json"
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(urllib.request.Request(url, data=body, headers=headers), timeout=5) as reply:
            return reply.status, reply.read()
    except urllib.error.HTTPError as exc:
        with exc:
            return exc.code, exc.read()
    except (urllib.error.URLError, OSError):
        return 0, b""


def gui_sim_run(ctx: Ctx, exe: Path) -> tuple[int, str, float]:
    """Start `iai-gui.exe --sim-only --no-browser` on a free port, wait until its page answers HTTP 200, read the Python
    inside it (/api/about), and stop it the way the page's Quit button does (POST /api/quit: the safe shutdown, then the
    process ends). Returns (port, the Python inside the exe, seconds until the page answered). Nothing but the simulator
    runs: --sim-only lists and opens no serial port."""
    port = free_port()
    base = f"http://127.0.0.1:{port}"
    ctx.counter += 1
    log = ctx.logs / f"{ctx.counter:02d}-gui-sim-only.log"
    cmd = [str(exe), "--sim-only", "--no-browser", "--port", str(port)]
    lines: list[str] = []
    with log.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(f"$ {quote(cmd)}\n  in {ctx.work}\n\n")
        try:
            proc = subprocess.Popen(cmd, cwd=ctx.work, env=ctx.env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding="utf-8", errors="replace")
        except OSError as exc:
            raise Failure(f"gui-sim-only: cannot start {exe}: {exc}") from exc

        def pump() -> None:
            assert proc.stdout is not None
            for line in proc.stdout:
                lines.append(line)
                fh.write(line)
                fh.flush()
                if ctx.args.verbose:
                    print("    " + line.rstrip(), flush=True)

        reader = threading.Thread(target=pump, daemon=True)
        reader.start()

        def fail(why: str) -> Failure:
            return Failure(f"{why}: {quote(cmd)}\n{''.join(lines[-30:]).rstrip()}\n(full output: {log})")

        try:
            t0 = time.monotonic()
            status = 0
            while time.monotonic() - t0 < GUI_START_SECS:
                if proc.poll() is not None:
                    raise fail(f"iai-gui.exe ended with exit {proc.returncode} before it answered")
                status, _page = gui_request(base + "/")
                if status == 200:
                    break
                time.sleep(0.5)
            if status != 200:
                raise fail(f"iai-gui.exe did not answer HTTP 200 on {base}/ within {GUI_START_SECS} s (last status "
                           f"{status})")
            answered = time.monotonic() - t0
            key = None
            for _ in range(50):  # the launch address is printed before the server starts: the reader may be a moment behind
                key = re.search(r"#k=([0-9a-f]+)", "".join(lines))
                if key:
                    break
                time.sleep(0.1)
            if not key:
                raise fail("iai-gui.exe printed no access key (#k=...)")
            status, body = gui_request(base + "/api/about", token=key.group(1))
            about = json.loads(body.decode("utf-8", "replace")) if status == 200 else {}
            if status != 200 or not about.get("python"):
                raise fail(f"GET /api/about answered {status}")
            status, _ = gui_request(base + "/api/quit", token=key.group(1), body=b"{}")
            if status != 200:
                raise fail(f"POST /api/quit answered {status}")
            try:
                code = proc.wait(timeout=GUI_STOP_SECS)
            except subprocess.TimeoutExpired:
                raise fail(f"iai-gui.exe did not end within {GUI_STOP_SECS} s of /api/quit") from None
            if code != 0:
                raise fail(f"iai-gui.exe ended with exit {code} after /api/quit")
        finally:
            if proc.poll() is None:  # a failure above: the whole process tree (the onefile exe has a parent)
                subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
                proc.wait()
            reader.join(10)
            fh.write(f"\n[exit {proc.returncode}]\n")
    if not any(line.startswith("GUI stopped.") for line in lines):
        raise Failure(f"iai-gui.exe ended without printing 'GUI stopped.' (full output: {log})")
    return port, str(about["python"]), answered


def gui_python_source(ctx: Ctx) -> str:
    """Where the Python inside iai-gui.exe comes from: the gui environment is made from --gui-python, else --python, else
    the interpreter that runs this script (ctx.python: --python or sys.executable)."""
    if ctx.gui_python:
        return "--gui-python"
    if getattr(ctx.args, "python", None):
        return "--python"
    return "the interpreter that ran this build"


def step_gui(ctx: Ctx) -> str:
    """What the gui job of release.yml does, on the win-x64 wheel just built (work/dist/wheel): a new environment with the
    wheel and PyInstaller, (with --gui-tests) the GUI tests, PyInstaller with apps/gui/iai-gui.spec, the check of the
    bundle, --version, a short --sim-only run, and the zip in work/dist/gui."""
    gui_dist = ctx.work / "dist" / "gui"
    rmtree(gui_dist)  # a zip of an earlier run must not stand in for this one (collect takes what is in this folder)
    why = gui_skip_reason(sys.platform, (ctx.src / GUI_SPEC).is_file())
    if why:
        say(f"      no GUI zip: {why}")
        ctx.fact("GUI (iai-gui.exe)", f"not built: {why}; the release has no GUI zip")
        return f"skipped: {why}"
    wheel = one(need(ctx.work / "dist" / "wheel", "wheel"), "*.whl")
    venv = ctx.work / "venv-gui"
    rmtree(venv)
    run(ctx, "gui-venv", [ctx.gui_python or ctx.python, "-m", "venv", str(venv)], ctx.work)
    py = str(venv / "Scripts" / "python.exe")
    run(ctx, "gui-install", [py, "-m", "pip", "install", "--quiet", str(wheel),
                             f"pyinstaller=={ctx.pyinstaller_version}"], ctx.work, timeout=900)
    python_version = run(ctx, "gui-python", [py, "-c", "import platform; print(platform.python_version())"],
                         ctx.work).strip().splitlines()[-1]
    chosen = gui_python_source(ctx)
    ctx.fact("Python inside iai-gui.exe",
             f"{python_version} ({chosen}; release.yml builds the GUI with {ctx.release_python})")
    if python_version.split(".")[:2] != ctx.release_python.split(".")[:2]:
        say(f"      note: iai-gui.exe embeds Python {python_version} ({chosen}; release.yml: {ctx.release_python})")

    if ctx.args.gui_tests:
        # the GUI source of the exported tree against the installed wheel (the source package is not on the path here),
        # through the runner of tools/test.py: the load-sensitive tests after the others, a failed one once more
        try:
            cmd = [py, "tools/unittest_jobs.py", "-s", "apps/gui", "-t", "apps", "--expect-backend", "native",
                   "--load-sensitive", "gui"]
            text = run(ctx, "gui-tests", cmd, ctx.src, timeout=1800)
        except Failure as exc:
            raise Failure(f"{exc}\n(the GUI tests ran on Python {python_version}, the one iai-gui.exe would embed; some "
                          "of them are timing-sensitive and fail on a busy PC: run the step again)") from None
        ctx.fact("GUI tests (apps/gui) against the installed wheel", suite_summary("gui", text))
    else:
        ctx.fact("GUI tests (apps/gui)", "not run (--gui-tests runs them, as the gui job of release.yml does)")

    build = ctx.work / "gui-build"
    rmtree(build)
    # IAI_ACTUATOR_DRIVER_NATIVE is not set (ctx.env drops it): the spec takes the library from the installed wheel
    run(ctx, "gui-pyinstaller", [py, "-m", "PyInstaller", "--noconfirm", "--clean", "--distpath", str(build / "dist"),
                                 "--workpath", str(build / "build"), GUI_SPEC], ctx.src, timeout=1800)
    exe = need(build / "dist" / "iai-gui.exe", "gui")
    bundle = run(ctx, "gui-bundle-check", [py, "-c", GUI_BUNDLE_CHECK, str(exe), GUI_BUNDLED_DLL], ctx.work).strip()
    ctx.fact("iai-gui.exe carries the native library", bundle.splitlines()[-1])
    out = run(ctx, "gui-version", [str(exe), "--version"], ctx.work).strip().splitlines()[-1]
    if out != f"iai-gui {ctx.version}":
        raise Failure(f"iai-gui.exe --version printed {out!r}, expected 'iai-gui {ctx.version}'")
    port, inside, secs = gui_sim_run(ctx, exe)
    ctx.fact("iai-gui.exe --version, and --sim-only --no-browser run",
             f"{out!r}; the page answered HTTP 200 on 127.0.0.1:{port} after {secs:.0f} s, Python {inside} reported, "
             "stopped with POST /api/quit (exit 0)")

    # run by the Python of the exe, as in release.yml: the zip carries that Python's LICENSE.txt (make_gui_zip.py)
    run(ctx, "gui-zip", [py, GUI_ZIP, "--exe", str(exe), "--version", ctx.version, "--out", str(gui_dist)], ctx.src)
    zip_path = need(gui_dist / make_gui_zip.zip_name(ctx.version), "gui")
    ctx.fact("GUI zip", f"{zip_path.name} {size(zip_path)}: {', '.join(make_gui_zip.CONTENTS)} (iai-gui.exe {size(exe)})")
    return f"{zip_path.name} {size(zip_path)}; Python {inside} inside"


def step_collect(ctx: Ctx) -> str:
    artifacts = ctx.work / "artifacts"
    rmtree(artifacts)
    artifacts.mkdir(parents=True)
    # (what, folder of work/dist, the step that makes it): one wheel and one SDK zip per target
    assets = [("*.whl", "wheel", "wheel"), ("*.tar.gz", "sdist", "sdist"), ("*.nupkg", "nuget", "nuget"),
              ("*.zip", "sdk", "sdk")]
    if RID not in ctx.targets:
        assets = [a for a in assets if a[1] not in ("wheel", "sdk")]
    if LINUX_RID in ctx.targets:
        assets += [("*.whl", f"wheel-{LINUX_RID}", "linux-wheel"), ("*.zip", f"sdk-{LINUX_RID}", "linux-sdk")]
    for pattern, folder, step in assets:
        src = one(need(ctx.work / "dist" / folder, step), pattern)
        shutil.copy2(src, artifacts / src.name)
    # the gui step makes the GUI zip in dist/gui (and removes an old one first); when it is there the release requires it,
    # when the step was skipped (not Windows, no apps/gui/iai-gui.spec) or left out with --skip the GUI is optional
    gui_zips = sorted((ctx.work / "dist" / "gui").glob("iai-gui-*.zip"))
    if gui_zips:
        gui = one(ctx.work / "dist" / "gui", "iai-gui-*.zip")
        shutil.copy2(gui, artifacts / gui.name)
    release = ctx.out / "release"
    rmtree(release)
    notes = ctx.out / "release-notes.md"
    count = str(len(ctx.targets))  # a wheel and an SDK per target (collect_release.py expects two of each)
    cmd = [ctx.tools_python, ".github/release/collect_release.py", "--artifacts", str(artifacts), "--out",
           str(release), "--version", ctx.version, "--tag", ctx.tag, "--minimum", f"Python wheels={count}", "--minimum",
           f"C/C++ SDK={count}"]
    if tuple(ctx.targets) in PLATFORM_NOTES:
        cmd += ["--note", PLATFORM_NOTES[tuple(ctx.targets)]]
    if not gui_zips:
        # the notes say it is not in this release
        cmd += ["--optional", "GUI"]
        say("      the GUI zip was not built (no iai-gui-*.zip in " + str(ctx.work / "dist" / "gui") + "): "
            "the release will have no iai-gui.exe")
    run(ctx, "collect", cmd, ctx.src)
    need(notes, "collect")
    return f"{len(list(release.iterdir()))} files in {release}"


STEP_FUNCS = {"export": step_export, "tools": step_tools, "native": step_native, "library": step_library,
              "sdk": step_sdk, "sdk-smoke": step_sdk_smoke, "linux-native": step_linux_native,
              "linux-library": step_linux_library, "linux-sdk": step_linux_sdk, "linux-sdk-smoke": step_linux_sdk_smoke,
              "sdist": step_sdist, "wheel": step_wheel, "linux-wheel": step_linux_wheel, "nuget": step_nuget,
              "nuget-smoke": step_nuget_smoke, "linux-nuget-smoke": step_linux_nuget_smoke, "gui": step_gui,
              "collect": step_collect}


# ------------------------------------------------------------------------------------------------------------ the end

def github_repo() -> str | None:
    url = git("remote", "get-url", "origin").strip()
    m = re.search(r"github\.com[:/]([^/\s]+/[^/\s]+?)(?:\.git)?$", url)
    return m.group(1) if m else None


def release_command(ctx: Ctx) -> list[str]:
    release = ctx.out / "release"
    assets = sorted(p for p in release.iterdir() if p.is_file())
    cmd = ["gh", "release", "create", ctx.tag]
    repo = github_repo()
    if repo:
        cmd += ["--repo", repo]
    cmd += ["--target", ctx.commit, "--title", f"IAI Actuator Driver {ctx.version}", "--notes-file",
            str(ctx.out / "release-notes.md")]
    if ctx.prerelease:
        cmd.append("--prerelease")
    cmd.append("--latest" if ctx.args.latest else "--latest=false")
    return cmd + [str(p) for p in assets]


def finish(ctx: Ctx) -> None:
    release = ctx.out / "release"
    missing = [s for s in ctx.steps if s not in ctx.done]
    lines = [f"Local release {ctx.version} ({ctx.tag}), commit {ctx.commit} on {ctx.branch}", "",
             f"Targets: {', '.join(ctx.targets)}"]
    lines += [f"Note: {note}" for note in ctx.notes]
    lines += ["", "Steps"]
    for name in STEPS:
        if name in ctx.done:
            lines.append(f"  {name:<18} {ctx.done[name]['secs']:7.1f} s  {ctx.done[name]['detail']}")
    lines += ["", "Tests and checks"]
    lines += [f"  {what}: {result}" for name in STEPS for what, result in ctx.done.get(name, {}).get("facts", [])] \
        or ["  (none run)"]
    if release.is_dir() and not missing:
        lines += ["", f"Assets in {release}"]
        lines += [f"  {p.name}  {size(p)}" for p in sorted(release.iterdir()) if p.is_file()]
    if missing:
        lines += ["", f"Not done yet: {', '.join(missing)}. No command is printed until every step has passed for this "
                      "commit (--resume continues this run)."]
    else:
        cmd = quote(release_command(ctx))
        remote = git("branch", "-r", "--contains", ctx.commit).strip()
        lines += ["", "Command that would publish this release (not run):", "", "  " + cmd, ""]
        lines += ["Before running it:",
                  f"  - the commit {ctx.commit[:7]} must be on GitHub: "
                  + ("it is on " + remote.splitlines()[0].strip() + "." if remote
                     else "it is on no remote branch yet (push it first)."),
                  f"  - the tag {ctx.tag} must not exist yet: the command creates it at that commit (gh release view "
                  f"{ctx.tag} should say 'release not found').",
                  "  - gh uploads the files while the release is still a draft, then publishes it.",
                  "  - a new tag starts no GitHub workflow: the Release workflow (release.yml) runs by hand only."]
        (ctx.out / "gh-release-create.txt").write_text(cmd + "\n", encoding="utf-8", newline="\n")
    text = "\n".join(lines) + "\n"
    (ctx.out / "summary.txt").write_text(text, encoding="utf-8", newline="\n")
    say()
    say(text)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build, test and collect the win-x64 and linux-x64 release on this PC.")
    parser.add_argument("--out", help="output folder (default: build/release-local/<version> in the repository)")
    parser.add_argument("--tools", help="the tools environment (default: _tools next to the output folder)")
    parser.add_argument("--python", help="the Python the tools and test environments are made from (default: this one)")
    parser.add_argument("--gui-tests", action="store_true",
                        help="the gui step also runs the GUI tests (apps/gui) against the wheel, as the gui job of release.yml "
                             "does: about 6 minutes, and some of them are timing-sensitive (a busy PC fails a few)")
    parser.add_argument("--gui-python", help="the Python the GUI environment is made from, which iai-gui.exe embeds "
                                             "(default: --python; release.yml builds the GUI with its PYTHON_VERSION)")
    parser.add_argument("--resume", action="store_true",
                        help="continue the run of this commit in the output folder: steps that passed are not run "
                             "again (the step that failed, and those after it, run). A commit that differs from the "
                             f"recorded one in {RELEASE_SCRIPTS} only is accepted too")
    parser.add_argument("--targets", help=f"the platforms to build, comma separated ({', '.join(TARGET_NAMES)}; default: "
                                          f"both when WSL and the kit are available, else {RID} and a note says why)")
    parser.add_argument("--wsl-kit", default=os.environ.get(WSL_KIT_ENV) or None,
                        help=f"the Linux test kit of the linux-x64 steps (default: %%{WSL_KIT_ENV}%%; without either, "
                             "only win-x64 is built): a folder with a run.ps1 that takes -Project <folder> -Profile "
                             "<script> -ProfileArgs <step> -Distro <name>, copies <folder> to ~/work/<folder name> in "
                             "that WSL distro, runs <script> <step> there and exits with the script's exit code")
    parser.add_argument("--wsl-distro", default=DEFAULT_WSL_DISTRO, help=f"the WSL distro (default: {DEFAULT_WSL_DISTRO})")
    parser.add_argument("--only", help=f"run only these steps, comma separated ({', '.join(STEPS)})")
    parser.add_argument("--skip", help="skip these steps, comma separated")
    parser.add_argument("--jobs", type=int, help="compile jobs of the native build (default: one per CPU thread)")
    parser.add_argument("--latest", action="store_true",
                        help="print --latest instead of --latest=false (a final release that is to become Latest)")
    parser.add_argument("-v", "--verbose", action="store_true", help="also print every command's output")
    args = parser.parse_args(argv)
    for opt in (args.only, args.skip):
        unknown = [s for s in (opt or "").split(",") if s and s not in STEPS]
        if unknown:
            parser.error(f"unknown step {unknown}; steps: {', '.join(STEPS)}")
    if args.targets is not None:
        named = [s for s in args.targets.split(",") if s]
        unknown = [s for s in named if s not in TARGET_NAMES]
        if unknown or not named:  # "" and "," name no target at all
            what = f"unknown target {unknown}" if unknown else f"--targets {args.targets!r} names no target"
            parser.error(f"{what}; targets: {', '.join(TARGET_NAMES)}")
    try:
        sys.stdout.reconfigure(errors="replace", line_buffering=True)
        sys.stderr.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass

    ctx = None
    try:
        ctx = preflight(args)
        say(f"local release {ctx.version} ({ctx.tag}{', pre-release' if ctx.prerelease else ''}) of commit "
            f"{ctx.commit[:7]} on {ctx.branch}; output {ctx.out}")
        say(f"zig {ctx.zig_version} (release.yml), .NET SDK {ctx.dotnet_version} (release.yml), Python "
            f"{sys.version.split()[0]}")
        say(f"targets: {', '.join(ctx.targets)}" + (f" (linux-x64 in WSL {ctx.wsl.distro}, kit {ctx.wsl.kit})"
                                                   if ctx.wsl else ""))
        for note in ctx.notes:
            say(f"note: {note}")
        wanted = [s for s in args.only.split(",") if s] if args.only else []
        off = [s for s in wanted if s not in ctx.steps]
        if off:
            raise Failure(f"--only {','.join(off)}: {'that step belongs' if len(off) == 1 else 'those steps belong'} to "
                          f"a target that is not built (targets: {', '.join(ctx.targets)}; --targets)")
        selected = [s for s in ctx.steps if (not wanted or s in wanted) and s not in (args.skip or "").split(",")]
        prepare_out(ctx, keep=bool(args.resume or args.only or args.skip))
        for number, name in enumerate(selected, 1):
            say(f"[{number}/{len(selected)}] {name} ...")
            if args.resume and name in ctx.done:
                say(f"      passed in an earlier run ({ctx.done[name]['secs']:.0f} s) {ctx.done[name]['detail']}")
                continue
            # what a step makes is the input of the steps after it: those must run again
            for later in ([name] if name == "tools" else STEPS[STEPS.index(name):]):
                ctx.done.pop(later, None)
            save_state(ctx)
            t0 = time.monotonic()
            if name not in ("export", "tools"):
                need_src(ctx)
            ctx.facts = []
            detail = STEP_FUNCS[name](ctx)
            secs = time.monotonic() - t0
            ctx.done[name] = {"detail": detail, "secs": round(secs, 1), "facts": ctx.facts}
            save_state(ctx)
            say(f"      ok ({secs:.0f} s) {detail}")
        finish(ctx)
        return 0
    except Failure as exc:
        say(f"\nFAILED: {exc}")
        return 1
    finally:
        # msbuild and the compiler server may keep files of the output folder open
        if ctx is not None and shutil.which("dotnet"):
            subprocess.run(["dotnet", "build-server", "shutdown"], capture_output=True)


if __name__ == "__main__":
    sys.exit(main())
