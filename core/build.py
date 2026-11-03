#!/usr/bin/env python3
"""Build the C++ edition on this PC (no CMake / MSVC here): zig's clang + libc++ via ``python -m ziglang c++``.

Customers use CMakeLists.txt instead. Output goes to core/build/:

    build/obj/...            objects (+ .d dependency files)
    build/libiai_actuator_driver.a     the library (every src/*.cpp)
    build/ec_tests.exe       the tests (tests/*.cpp + the library + cli/*.cpp except main.cpp)
    build/ec_tests_<unit>.exe  one unit's tests (with --unit)
    build/iai-actuator-driver-cpp.exe  the command line (cli/*.cpp + the library)
    build/iai_actuator_driver.dll      the C ABI (include/iai_actuator_driver.h) as a DLL: every src/*.cpp, libc++
    build/iai_actuator_driver.lib      linked statically (no runtime DLL beyond Windows' own), + its import library;
                                       a version resource (src/iai_actuator_driver.rc.in) names both versions
    build/c_smoke.exe        tests/c_smoke.c: a C99 program against the DLL (smoke)
    build/c_smoke_1_0.exe    abi/1.0/c_smoke.c: the frozen 1.0 program, built with the frozen 1.0 header (smoke)

Usage:
    python build.py lib   [--unit U]          compile the library (or one unit's sources)
    python build.py tests [--unit U]          build the test executable
    python build.py cli                       build the command line
    python build.py shared                    build the C ABI DLL and its import library
    python build.py smoke                     build the DLL, compile tests/c_smoke.c and the frozen 1.0 program
                                              (abi/1.0/c_smoke.c) as C99 against it and run both
    python build.py test  [--unit U] [--shards N] [PREFIX ...]   build and run the tests (PREFIX: test-name prefixes)
    python build.py check [FILE ...]          syntax-only check (default: every header; the C ABI headers as C99)
    python build.py manifest [--update]       the C ABI manifest (abi/manifest.py): the headers against
                                              abi/manifest.txt, the Python and the C# bindings (--update rewrites it)
Options: -j N (parallel jobs, default: CPUs), --release (-O2; default -O1), --force (rebuild all), -v (show commands),
--shards N (test: run the tests as N processes at once through tests/run_shards.py; default half the CPUs, at most 6;
--shards 1 runs the test executable directly, one test after another).

Units (disjoint file ownership; see DESIGN.md): bus, model, sim, axis, capi. ``--unit bus|model|sim`` builds and tests
that unit alone (they only share header-only code); ``--unit axis`` needs the other three, ``--unit capi`` all four.
Incremental by mtime: an object is rebuilt when its source or any header in its .d file is newer, or the flags
changed. The first compile after a zig install takes about a minute (zig builds its libc++ cache); then it is fast.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import fnmatch
import importlib.util
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
# IAD_BUILD_DIR moves the whole output (objects, library, tests) elsewhere, so parallel builds of one checkout don't clash
BUILD = Path(os.environ["IAD_BUILD_DIR"]).resolve() if os.environ.get("IAD_BUILD_DIR") else ROOT / "build"
OBJ = BUILD / "obj"
TARGET = "x86_64-windows-gnu"
EXE = ".exe"
ZIG = [sys.executable, "-m", "ziglang"]
LINK_LIBS = ["-lwinmm", "-lsetupapi", "-ladvapi32"]

# Disjoint file ownership of the four implementation units (patterns relative to src/, tests/).
UNITS: dict[str, dict[str, list[str]]] = {
    "bus": {
        "src": ["serial_*.cpp", "transport.cpp", "modbus*.cpp", "baud.cpp", "commands.cpp", "fake_transport.cpp",
                "port.cpp", "bus_*.cpp", "line.cpp", "gateway.cpp"],
        "tests": ["test_bus*.cpp"],
    },
    "model": {
        "src": ["registers.cpp", "alarm*.cpp", "status.cpp", "directory.cpp", "device_info.cpp", "limits.cpp",
                "position.cpp", "*parameter*.cpp", "to_json.cpp", "model_*.cpp"],
        "tests": ["test_model*.cpp"],
    },
    "sim": {
        "src": ["sim_*.cpp"],
        "tests": ["test_sim*.cpp"],
    },
    "axis": {
        "src": ["axis_*.cpp", "cell.cpp", "cell_*.cpp", "monitor.cpp", "move_handle.cpp", "recovery*.cpp", "info.cpp", "check.cpp"],
        "tests": ["test_axis*.cpp", "test_cell*.cpp", "test_cli*.cpp", "test_recovery*.cpp", "test_info*.cpp",
                  "test_check*.cpp"],
    },
    "capi": {
        "src": ["c_api*.cpp"],
        "tests": ["test_capi*.cpp", "test_c_api*.cpp"],
    },
    "coord": {  # coordinated motion (coord.hpp): on top of the axis unit (it reaches Axis::Impl through AxisAccess)
        "src": ["coord_*.cpp"],
        "tests": ["test_coord*.cpp"],
    },
}
UNIT_DEPS = {"bus": [], "model": [], "sim": [], "axis": ["bus", "model", "sim"],
             "capi": ["bus", "model", "sim", "axis", "coord"], "coord": ["bus", "model", "sim", "axis"]}
# The shared library: its name, and the define that makes IAD_API dllexport (only c_api*.cpp is compiled with it).
SHARED_NAME = "iai_actuator_driver"
SHARED_DEFINE = "-DIAD_BUILD_SHARED"
HARNESS_TESTS = ["test_main.cpp", "test_harness.cpp"]

# zig's c++ driver reports "<file>:1:1: error: FileNotFound" after a successful -fsyntax-only (it looks for an output
# file that syntax-only never writes); those lines, and the "-c unused" warning, are not real diagnostics.
QUIRK_RE = re.compile(r"(:1:1: error: FileNotFound\s*$)|(argument unused during compilation: '-c')")


def base_flags(release: bool) -> list[str]:
    return [
        "-target", TARGET, "-std=c++17", "-O2" if release else "-O1",
        "-Iinclude", "-Isrc", "-Icli", "-Itests",
        "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-command-line-argument",
    ]


def rel(path: Path) -> str:
    # absolute for outputs under an IAD_BUILD_DIR outside core/
    return path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else path.as_posix()


def matches(name: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatch(name, p) for p in patterns)


def unit_of_src(name: str) -> str | None:
    for unit, spec in UNITS.items():
        if matches(name, spec["src"]):
            return unit
    return None


def unit_of_test(name: str) -> str | None:
    for unit, spec in UNITS.items():
        if matches(name, spec["tests"]):
            return unit
    return None


def lib_sources(unit: str | None) -> list[Path]:
    files = sorted((ROOT / "src").glob("*.cpp"))
    if unit is None:
        for f in files:
            if unit_of_src(f.name) is None:
                print(f"note: src/{f.name} belongs to no unit (built with the full library)")
        return files
    wanted = {unit, *UNIT_DEPS[unit]}
    return [f for f in files if unit_of_src(f.name) in wanted]


def cli_sources(with_main: bool) -> list[Path]:
    files = sorted((ROOT / "cli").glob("*.cpp"))
    return files if with_main else [f for f in files if f.name != "main.cpp"]


def test_sources(unit: str | None) -> list[Path]:
    files = sorted((ROOT / "tests").glob("*.cpp"))
    out = [f for f in files if f.name in HARNESS_TESTS]
    for f in files:
        if f.name in HARNESS_TESTS:
            continue
        owner = unit_of_test(f.name)
        if unit is None or owner == unit:
            out.append(f)
        if owner is None:
            print(f"note: tests/{f.name} belongs to no unit")
    return out


def obj_path(src: Path, obj_dir: Path = OBJ) -> Path:
    return obj_dir / src.parent.name / (src.stem + ".o")


def depfile_deps(dep: Path) -> list[Path] | None:
    """The prerequisites listed in a clang .d file (None when unreadable)."""
    try:
        text = dep.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    text = text.replace("\\\r\n", " ").replace("\\\n", " ")
    sep = text.find(": ")
    if sep < 0:
        return None
    parts = re.split(r"(?<!\\)\s+", text[sep + 2:].strip())
    deps = []
    for p in parts:
        if not p:
            continue
        p = p.replace("\\ ", " ")
        path = Path(p)
        deps.append(path if path.is_absolute() else ROOT / path)
    return deps


def needs_build(src: Path, obj: Path, force: bool) -> bool:
    if force or not obj.exists():
        return True
    dep = obj.with_suffix(".d")
    deps = depfile_deps(dep)
    if deps is None:
        return True
    out_time = obj.stat().st_mtime
    for d in [src, *deps]:
        try:
            if d.stat().st_mtime > out_time:
                return True
        except OSError:
            return True
    return False


_FLAGS_CHANGED: dict[str, bool] = {}


def flags_changed(flags: list[str], obj_dir: Path = OBJ) -> bool:
    """True (for the whole run) when the flags differ from the last build's in obj_dir: its objects are then rebuilt.
    Each object directory keeps its own stamp (the shared build's c_api objects have one flag more)."""
    text = " ".join(flags)
    key = f"{obj_dir}|{text}"
    if key not in _FLAGS_CHANGED:
        stamp = obj_dir / "flags.txt"
        old = stamp.read_text(encoding="utf-8") if stamp.exists() else None
        changed = old is not None and old != text
        obj_dir.mkdir(parents=True, exist_ok=True)
        stamp.write_text(text, encoding="utf-8")
        _FLAGS_CHANGED[key] = changed
    return _FLAGS_CHANGED[key]


def run(cmd: list[str], verbose: bool) -> tuple[int, str]:
    if verbose:
        print(" ".join(cmd))
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def compile_all(sources: list[Path], flags: list[str], jobs: int, force: bool, verbose: bool,
                obj_dir: Path = OBJ) -> list[Path]:
    """Compile what is out of date (in parallel) into obj_dir; return every object. Exits on a compile error."""
    force = flags_changed(flags, obj_dir) or force
    todo = [s for s in sources if needs_build(s, obj_path(s, obj_dir), force)]
    objects = [obj_path(s, obj_dir) for s in sources]
    if not todo:
        return objects
    for s in todo:
        obj_path(s, obj_dir).parent.mkdir(parents=True, exist_ok=True)

    def one(src: Path) -> tuple[Path, int, str, float]:
        obj = obj_path(src, obj_dir)
        cmd = [*ZIG, "c++", *flags, "-MMD", "-MF", str(obj.with_suffix(".d")), "-c", rel(src), "-o", str(obj)]
        t0 = time.monotonic()
        code, out = run(cmd, verbose)
        return src, code, out, time.monotonic() - t0

    failed = []
    done = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        for src, code, out, secs in pool.map(one, todo):
            done += 1
            status = "ok" if code == 0 else "FAILED"
            print(f"[{done}/{len(todo)}] {rel(src)} {status} ({secs:.1f} s)")
            text = "\n".join(line for line in out.splitlines() if not QUIRK_RE.search(line)).strip()
            if text:
                print(text)
            if code != 0:
                failed.append(src)
                try:
                    obj_path(src, obj_dir).unlink()
                except OSError:
                    pass
    if failed:
        print(f"\n{len(failed)} file(s) failed: " + ", ".join(rel(f) for f in failed))
        sys.exit(1)
    return objects


def up_to_date(output: Path, inputs: list[Path]) -> bool:
    if not output.exists():
        return False
    t = output.stat().st_mtime
    return all(i.stat().st_mtime <= t for i in inputs)


def link(output: Path, objects: list[Path], verbose: bool, force: bool, extra: list[str] | None = None) -> None:
    """Link an executable (or, with extra = ["-shared", ...], a DLL)."""
    if not force and up_to_date(output, objects):
        print(f"{rel(output)} up to date")
        return
    BUILD.mkdir(parents=True, exist_ok=True)
    cmd = [*ZIG, "c++", "-target", TARGET, *(extra or []), *[str(o) for o in objects], "-o", str(output), *LINK_LIBS]
    t0 = time.monotonic()
    code, out = run(cmd, verbose)
    text = "\n".join(line for line in out.splitlines() if not QUIRK_RE.search(line)).strip()
    if text:
        print(text)
    if code != 0:
        print(f"link of {rel(output)} FAILED")
        sys.exit(1)
    print(f"linked {rel(output)} ({time.monotonic() - t0:.1f} s)")


def archive(output: Path, objects: list[Path], verbose: bool, force: bool) -> None:
    if not objects:
        print("no library sources yet (src/*.cpp): library not built")
        return
    if not force and up_to_date(output, objects):
        print(f"{rel(output)} up to date")
        return
    if output.exists():
        output.unlink()  # a build artifact, rebuilt from the objects (drops objects of removed sources)
    code, out = run([*ZIG, "ar", "rcs", str(output), *[str(o) for o in objects]], verbose)
    if code != 0:
        print(out)
        print(f"archive {rel(output)} FAILED")
        sys.exit(1)
    print(f"archived {rel(output)} ({len(objects)} objects)")


def build_lib(args: argparse.Namespace) -> list[Path]:
    flags = base_flags(args.release)
    objects = compile_all(lib_sources(args.unit), flags, args.jobs, args.force, args.verbose)
    name = "libiai_actuator_driver.a" if args.unit is None else f"libiai_actuator_driver_{args.unit}.a"
    archive(BUILD / name, objects, args.verbose, args.force)
    return objects


def build_tests(args: argparse.Namespace) -> Path:
    flags = base_flags(args.release)
    lib_objs = compile_all(lib_sources(args.unit), flags, args.jobs, args.force, args.verbose)
    with_cli = args.unit in (None, "axis")
    cli_objs = compile_all(cli_sources(with_main=False), flags, args.jobs, args.force, args.verbose) if with_cli else []
    test_objs = compile_all(test_sources(args.unit), flags, args.jobs, args.force, args.verbose)
    exe = BUILD / (f"ec_tests{EXE}" if args.unit is None else f"ec_tests_{args.unit}{EXE}")
    link(exe, [*test_objs, *cli_objs, *lib_objs], args.verbose, args.force)
    return exe


def build_cli(args: argparse.Namespace) -> Path | None:
    flags = base_flags(args.release)
    sources = cli_sources(with_main=True)
    if not sources:
        print("no CLI sources yet (cli/*.cpp): command line not built")
        return None
    lib_objs = compile_all(lib_sources(None), flags, args.jobs, args.force, args.verbose)
    cli_objs = compile_all(sources, flags, args.jobs, args.force, args.verbose)
    exe = BUILD / f"iai-actuator-driver-cpp{EXE}"
    link(exe, [*cli_objs, *lib_objs], args.verbose, args.force)
    return exe


def version_resource(out_dir: Path, core: Path = ROOT) -> Path:
    """src/iai_actuator_driver.rc.in with the core's version (version.hpp) and the C ABI versions (the two headers)
    filled in, written to out_dir/iai_actuator_driver.rc when its text changed (CMake does the same with
    configure_file). zig compiles an .rc file given to the link (resinator) into the DLL."""
    version = re.search(r'kVersion = "([^"]+)"', (core / "include" / "iai_actuator_driver" / "version.hpp")
                        .read_text(encoding="utf-8"))
    header = (core / "include" / "iai_actuator_driver.h").read_text(encoding="utf-8")
    testing = (core / "include" / "iai_actuator_driver_testing.h").read_text(encoding="utf-8")
    major = re.search(r"#define IAD_ABI_VERSION_MAJOR (\d+)", header)
    minor = re.search(r"#define IAD_ABI_VERSION_MINOR (\d+)", header)
    test = re.search(r"#define IAD_TESTING_ABI_VERSION (\d+)", testing)
    if not (version and major and minor and test):
        sys.exit("version_resource: a version is missing from version.hpp or the C headers")
    numbers = re.match(r"(\d+)\.(\d+)\.(\d+)", version.group(1))
    if not numbers:
        sys.exit(f"version_resource: {version.group(1)!r} does not start with X.Y.Z")
    values = {
        "IAD_VERSION": version.group(1),
        "IAD_RC_VERSION": ",".join([*numbers.groups(), "0"]),
        "IAD_ABI_VERSION": f"{major.group(1)}.{minor.group(1)}",
        "IAD_TESTING_ABI_VERSION": test.group(1),
    }
    text = (core / "src" / "iai_actuator_driver.rc.in").read_text(encoding="utf-8")
    for key, value in values.items():
        text = text.replace(f"@{key}@", value)
    out = out_dir / f"{SHARED_NAME}.rc"
    if not out.exists() or out.read_text(encoding="utf-8") != text:
        out_dir.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
    return out


def build_shared(args: argparse.Namespace) -> Path:
    """The C ABI as a DLL: every library object (shared with ``lib``) plus c_api*.cpp compiled again with
    IAD_BUILD_SHARED into build/obj/shared/ (so only the iad_ functions are exported). zig links libc++ and libunwind
    statically, so the DLL imports only Windows' own DLLs (KERNEL32, WINMM, SETUPAPI, ADVAPI32 and the UCRT
    api-ms-win-crt-*, part of Windows 10 and later). The import library is written next to it (COFF; MSVC and MinGW
    linkers both take it)."""
    flags = base_flags(args.release)
    capi = UNITS["capi"]["src"]
    sources = lib_sources(None)
    core_objs = compile_all([s for s in sources if not matches(s.name, capi)], flags, args.jobs, args.force,
                            args.verbose)
    capi_objs = compile_all([s for s in sources if matches(s.name, capi)], [*flags, SHARED_DEFINE], args.jobs,
                            args.force, args.verbose, obj_dir=OBJ / "shared")
    dll = BUILD / f"{SHARED_NAME}.dll"
    implib = BUILD / f"{SHARED_NAME}.lib"
    resource = version_resource(OBJ / "shared")
    link(dll, [*capi_objs, *core_objs, resource], args.verbose, args.force,
         extra=["-shared", f"-Wl,--out-implib,{implib}"])
    return dll


def run_manifest(args: argparse.Namespace) -> int:
    """abi/manifest.py: check the headers against abi/manifest.txt and both bindings, or (--update) rewrite it. Runs
    in a child Python; it reads the Python binding's tables without loading a library (manifest.python_binding)."""
    cmd = [sys.executable, str(ROOT / "abi" / "manifest.py"), "update" if args.update else "check"]
    return subprocess.run(cmd, cwd=ROOT).returncode


def run_smoke(args: argparse.Namespace) -> int:
    """The C smoke tests: tests/c_smoke.c compiled as plain C99 (-pedantic, ``zig cc``) against the DLL's import
    library, then run from build/ next to the DLL. It proves the header is valid C and that a C program drives the
    exports (simulator only, no COM port). Then the frozen 1.0 program, abi/1.0/c_smoke.c, built with the frozen 1.0
    header only (never include/): a C program built for 1.0 must still run on every later 1.x library."""
    build_shared(args)
    implib = BUILD / f"{SHARED_NAME}.lib"
    programs = [(ROOT / "tests" / "c_smoke.c", "include", f"c_smoke{EXE}"),
                (ROOT / "abi" / "1.0" / "c_smoke.c", "abi/1.0", f"c_smoke_1_0{EXE}")]
    failed = 0
    for src, include, name in programs:
        exe = BUILD / name
        header = ROOT / include / "iai_actuator_driver.h"
        if args.force or not up_to_date(exe, [src, implib, header]):
            cmd = [*ZIG, "cc", "-target", TARGET, "-std=c99", "-pedantic", "-Wall", "-Wextra", f"-I{include}", rel(src),
                   str(implib), "-o", str(exe)]
            code, out = run(cmd, args.verbose)
            text = "\n".join(line for line in out.splitlines() if not QUIRK_RE.search(line)).strip()
            if text:
                print(text)
            if code != 0:
                print(f"build of {rel(exe)} FAILED")
                return 1
            print(f"built {rel(exe)}")
        print(f"\nrunning {rel(exe)}")
        failed |= subprocess.run([str(exe)], cwd=BUILD).returncode
    return failed


def shard_runner():
    """tests/run_shards.py: runs the test executable as several processes at once and merges their output."""
    spec = importlib.util.spec_from_file_location("run_shards", ROOT / "tests" / "run_shards.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look the module up by name
    spec.loader.exec_module(module)
    return module


def run_tests(args: argparse.Namespace) -> int:
    exe = build_tests(args)
    if args.unit is None:
        build_cli(args)
    print(f"\nrunning {rel(exe)} {' '.join(args.prefixes)}".rstrip())
    return shard_runner().run(exe, args.prefixes, shards=args.shards, cwd=ROOT)


def default_check_files() -> list[Path]:
    files = sorted((ROOT / "include").glob("*.h"))
    files += sorted((ROOT / "include" / "iai_actuator_driver").glob("*.hpp"))
    files += sorted((ROOT / "src").glob("*.hpp"))
    files += sorted((ROOT / "cli").glob("*.hpp"))
    files += [ROOT / "tests" / "harness.hpp"]
    return [f for f in files if f.exists()]


def check(args: argparse.Namespace) -> int:
    files = [Path(f).resolve() for f in args.files] or default_check_files()
    flags = base_flags(args.release)

    def one(path: Path) -> tuple[Path, bool, str]:
        header = path.suffix in (".hpp", ".h")
        if path.suffix == ".h":  # the C ABI header: plain C99 (src/c_api.cpp compiles it as C++)
            cmd = [*ZIG, "cc", "-target", TARGET, "-std=c99", "-pedantic", "-Wall", "-Wextra", "-fsyntax-only",
                   "-x", "c"]
        else:
            cmd = [*ZIG, "c++", *flags, "-fsyntax-only"]
            if header:
                cmd += ["-Wno-pragma-once-outside-header", "-x", "c++"]
        cmd.append(str(path))
        code, out = run(cmd, args.verbose)
        lines = [line for line in out.splitlines() if not QUIRK_RE.search(line)]
        real_error = any(": error:" in line or line.startswith("error:") for line in lines)
        ok = code == 0 or not real_error
        return path, ok, "\n".join(lines).strip()

    bad = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        for path, ok, text in pool.map(one, files):
            name = rel(path) if ROOT in path.parents else str(path)
            print(f"{'ok    ' if ok else 'FAILED'} {name}")
            if text:
                print(text)
            bad += not ok
    print(f"\n{len(files)} file(s) checked, {bad} failed")
    return 1 if bad else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build the IAI Actuator Driver C++ edition with zig (python -m ziglang).")
    parser.add_argument("command", choices=["lib", "tests", "cli", "shared", "smoke", "test", "check", "manifest"])
    parser.add_argument("rest", nargs="*", help="test: test-name prefixes; check: files")
    parser.add_argument("--unit", choices=sorted(UNITS), help="one unit only (lib / tests / test)")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    parser.add_argument("--release", action="store_true", help="-O2 instead of -O1")
    parser.add_argument("--force", action="store_true", help="rebuild everything")
    parser.add_argument("-v", "--verbose", action="store_true", help="print every command")
    parser.add_argument("--update", action="store_true", help="manifest: rewrite abi/manifest.txt from the headers")
    parser.add_argument("--shards", type=int, default=None,
                        help="test: processes at once (default: half the CPUs, at most 6; 1: one process)")
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(line_buffering=True)  # keep our lines in order with the test executable's output
    except (AttributeError, OSError):
        pass
    args.prefixes = args.rest if args.command == "test" else []
    args.files = args.rest if args.command == "check" else []
    if args.rest and args.command not in ("test", "check"):
        parser.error(f"{args.command} takes no positional arguments")
    t0 = time.monotonic()
    if args.command == "lib":
        build_lib(args)
        code = 0
    elif args.command == "tests":
        build_tests(args)
        code = 0
    elif args.command in ("cli", "shared", "smoke"):
        if args.unit:
            parser.error(f"{args.command} builds the whole library; --unit does not apply")
        code = 0
        if args.command == "cli":
            build_cli(args)
        elif args.command == "shared":
            build_shared(args)
        else:
            code = run_smoke(args)
    elif args.command == "test":
        code = run_tests(args)
    elif args.command == "manifest":
        code = run_manifest(args)
    else:
        code = check(args)
    print(f"({time.monotonic() - t0:.1f} s)")
    return code


if __name__ == "__main__":
    sys.exit(main())
