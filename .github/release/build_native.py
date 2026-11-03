#!/usr/bin/env python3
"""Release build of the native core (core/) for one target, with zig's clang and libc++ (``python -m ziglang``).

core/build.py is the development build and targets Windows only. This script is the release build: the same sources
and warning flags, -O2, and a ``--target`` so one runner can build any of the shipped platforms (zig cross-compiles,
no sysroot needed). It calls ``python -m ziglang c++ -target <triple>`` directly; CMake is not used here (CMake is what
customers use with their own compiler, see core/CMakeLists.txt).

    python .github/release/build_native.py --target win-x64     [--core core] [--out build/native/win-x64] [--run]
    python .github/release/build_native.py --target linux-x64
    python .github/release/build_native.py --target linux-arm64

Targets (zig triples):
    win-x64      x86_64-windows-gnu       iai_actuator_driver.dll + iai_actuator_driver.lib (import library)
                                          (the DLL carries a version resource: core/src/iai_actuator_driver.rc.in)
    linux-x64    x86_64-linux-gnu.2.17    libiai_actuator_driver.so (glibc 2.17 symbols: manylinux2014)
    linux-arm64  aarch64-linux-gnu.2.17   libiai_actuator_driver.so

Output (--out, default build/native/<target>):
    dist/   what ships: the shared library (+ import library on Windows), the command line, build-info.json
    test/   ec_tests and c_smoke (tests/c_smoke.c as C99 against the shared library), with a copy of the library
            (on Linux also under its SONAME, the name c_smoke looks for)
    obj/    objects; obj-shared/ the c_api*.cpp objects compiled again with IAD_BUILD_SHARED

The C++ runtime (libc++, libunwind) is linked statically, so the library needs nothing beyond the OS: Windows' own
DLLs and the UCRT on win-x64, glibc (libc, libpthread, libm) on Linux. Only the iad_ functions are exported (IAD_API;
everything else is hidden; on Linux a linker version script also keeps the static C++ runtime out of the exports).
The Linux library's SONAME is libiai_actuator_driver.so.<ABI major> (IAD_ABI_VERSION_MAJOR, read from the header, as
core/build.py and the CMake build do): a program linked against it records that name, so a library of another ABI major
is never found for it. The file itself is still called libiai_actuator_driver.so in dist/ (the wheel and NuGet load it by
path); make_sdk.py ships it as lib/libiai_actuator_driver.so.<major> with a linker script named
libiai_actuator_driver.so for -liai_actuator_driver. Shipped files are stripped (-s); the tests keep symbols.

--run runs ec_tests and c_smoke when the target runs on this machine (same OS and CPU); otherwise it says so and only
builds. ec_tests runs as several processes at once (core/tests/run_shards.py; --shards N, default half the CPUs, at
most 6; --shards 1: one process). No test opens a serial port: every test runs on the simulator.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ZIG = [sys.executable, "-m", "ziglang"]
NAME = "iai_actuator_driver"
CLI_NAME = "iai-actuator-driver-cpp"

# zig's c++ driver prints "<file>:1:1: error: FileNotFound" after some successful runs and warns about "-c" unused;
# neither is a real diagnostic (same filter as core/build.py).
QUIRK_RE = re.compile(r"(:1:1: error: FileNotFound\s*$)|(argument unused during compilation: '-c')")


@dataclass(frozen=True)
class Target:
    rid: str
    triple: str
    os: str  # "windows" or "linux"
    arch: str  # "x64" or "arm64"

    @property
    def exe(self) -> str:
        return ".exe" if self.os == "windows" else ""

    @property
    def shared_lib(self) -> str:
        return f"{NAME}.dll" if self.os == "windows" else f"lib{NAME}.so"

    def soname(self, abi_major: int) -> str | None:
        """The SONAME of the Linux shared library (the ABI major is its version); None on Windows."""
        return None if self.os == "windows" else f"lib{NAME}.so.{abi_major}"

    def linux_shared_flags(self, abi_major: int, version_script: Path) -> list[str]:
        """Link flags of the Linux shared library. -z defs: fail here, not at load time, on an unresolved symbol. The
        version script exports the iad_ functions only, so nothing of the static C++ runtime leaks into the export table
        (zig's linker driver does not take --exclude-libs)."""
        return ["-shared", f"-Wl,-soname,{self.soname(abi_major)}", "-Wl,-z,defs", f"-Wl,--version-script,{version_script}"]

    @property
    def import_lib(self) -> str | None:
        return f"{NAME}.lib" if self.os == "windows" else None

    @property
    def link_libs(self) -> list[str]:
        if self.os == "windows":
            return ["-lwinmm", "-lsetupapi", "-ladvapi32"]
        return ["-lpthread"]

    @property
    def compile_extra(self) -> list[str]:
        if self.os == "windows":
            return []
        # position-independent code for the .so; hidden by default so only IAD_API (visibility default) is exported
        return ["-fPIC", "-fvisibility=hidden", "-fvisibility-inlines-hidden"]


TARGETS = {
    "win-x64": Target("win-x64", "x86_64-windows-gnu", "windows", "x64"),
    "linux-x64": Target("linux-x64", "x86_64-linux-gnu.2.17", "linux", "x64"),
    "linux-arm64": Target("linux-arm64", "aarch64-linux-gnu.2.17", "linux", "arm64"),
}


def abi_major(core: Path) -> int:
    """IAD_ABI_VERSION_MAJOR of the C header under `core` (the shared library's SONAME version)."""
    header = core / "include" / f"{NAME}.h"
    m = re.search(r"^#[ \t]*define[ \t]+IAD_ABI_VERSION_MAJOR[ \t]+(\d+)", header.read_text(encoding="utf-8"), flags=re.M)
    if m is None:
        sys.exit(f"IAD_ABI_VERSION_MAJOR not found in {header}")
    return int(m.group(1))


def host_rid() -> str | None:
    machine = platform.machine().lower()
    arch = {"amd64": "x64", "x86_64": "x64", "arm64": "arm64", "aarch64": "arm64"}.get(machine)
    system = {"win32": "win", "linux": "linux"}.get(sys.platform)
    return f"{system}-{arch}" if arch and system else None


def run(cmd: list[str], cwd: Path, verbose: bool) -> tuple[int, str]:
    if verbose:
        print(" ".join(str(c) for c in cmd))
    proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, "\n".join(line for line in out.splitlines() if not QUIRK_RE.search(line)).strip()


class Builder:
    def __init__(self, target: Target, core: Path, out: Path, jobs: int, verbose: bool,
                 shards: int | None = None) -> None:  # fmt: skip
        self.t = target
        self.core = core
        self.out = out
        self.jobs = jobs
        self.verbose = verbose
        self.shards = shards
        self.dist = out / "dist"
        self.test = out / "test"
        self.abi_major = abi_major(core)
        self.cxx_flags = [
            "-target", target.triple, "-std=c++17", "-O2",
            f"-I{core / 'include'}", f"-I{core / 'src'}", f"-I{core / 'cli'}", f"-I{core / 'tests'}",
            "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-command-line-argument",
            *target.compile_extra,
        ]

    def compile(self, sources: list[Path], obj_dir: Path, extra: list[str] | None = None) -> list[Path]:
        obj_dir.mkdir(parents=True, exist_ok=True)
        flags = [*self.cxx_flags, *(extra or [])]

        def one(src: Path) -> tuple[Path, Path, int, str, float]:
            obj = obj_dir / f"{src.parent.name}_{src.stem}.o"
            t0 = time.monotonic()
            code, text = run([*ZIG, "c++", *flags, "-c", str(src), "-o", str(obj)], self.core, self.verbose)
            return src, obj, code, text, time.monotonic() - t0

        objects, failed = [], []
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, self.jobs)) as pool:
            for i, (src, obj, code, text, secs) in enumerate(pool.map(one, sources), 1):
                rel = src.relative_to(self.core).as_posix()
                print(f"[{i}/{len(sources)}] {rel} {'ok' if code == 0 else 'FAILED'} ({secs:.1f} s)")
                if text:
                    print(text)
                (objects if code == 0 else failed).append(obj if code == 0 else src)
        if failed:
            sys.exit(f"{len(failed)} file(s) failed to compile for {self.t.rid}")
        return objects

    def link(self, output: Path, inputs: list[Path], extra: list[str]) -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        cmd = [*ZIG, "c++", "-target", self.t.triple, *extra, *[str(i) for i in inputs], "-o", str(output),
               *self.t.link_libs]
        t0 = time.monotonic()
        code, text = run(cmd, self.core, self.verbose)
        if text:
            print(text)
        if code != 0:
            sys.exit(f"link of {output.name} FAILED for {self.t.rid}")
        print(f"linked {output.relative_to(self.out).as_posix()} ({time.monotonic() - t0:.1f} s)")

    def build(self) -> None:
        src = sorted((self.core / "src").glob("*.cpp"))
        capi = [s for s in src if s.name.startswith("c_api")]
        cli_all = sorted((self.core / "cli").glob("*.cpp"))
        cli_lib = [s for s in cli_all if s.name != "main.cpp"]
        tests = sorted((self.core / "tests").glob("*.cpp"))
        if not src or not capi:
            sys.exit(f"no core sources under {self.core} (src/*.cpp, src/c_api*.cpp)")

        print(f"== {self.t.rid} ({self.t.triple}): {len(src)} library, {len(cli_all)} cli, {len(tests)} test sources")
        lib_objs = self.compile(src, self.out / "obj" / "src")
        capi_shared = self.compile(capi, self.out / "obj-shared", ["-DIAD_BUILD_SHARED"])
        core_objs = [o for o in lib_objs if not o.name.startswith("src_c_api")]
        cli_objs = self.compile(cli_all, self.out / "obj" / "cli")
        test_objs = self.compile(tests, self.out / "obj" / "tests")
        cli_lib_objs = [o for o in cli_objs if o.name != "cli_main.o"]

        # the shared library (the C ABI)
        lib = self.dist / self.t.shared_lib
        resources: list[Path] = []
        if self.t.os == "windows":
            extra = ["-shared", f"-Wl,--out-implib,{self.dist / self.t.import_lib}"]
            resources.append(self.version_resource())  # the core's and the ABI's versions in the DLL's Properties
        else:
            script = self.out / "exports.map"
            script.write_text("{\n  global: iad_*;\n  local: *;\n};\n", encoding="utf-8")
            extra = self.t.linux_shared_flags(self.abi_major, script)
        # -s: what ships carries no debug information (zig emits it by default: a .pdb on Windows, DWARF in the .so)
        self.link(lib, [*capi_shared, *core_objs, *resources], ["-s", *extra])

        # the command line and the tests (static library objects, no IAD_BUILD_SHARED)
        if cli_all:
            self.link(self.dist / f"{CLI_NAME}{self.t.exe}", [*cli_objs, *lib_objs], ["-s"])
        self.link(self.test / f"ec_tests{self.t.exe}", [*test_objs, *cli_lib_objs, *lib_objs], [])

        # the C smoke test: plain C99 against the shared library, run next to a copy of it
        self.test.mkdir(parents=True, exist_ok=True)
        shutil.copy2(lib, self.test / lib.name)
        soname = self.t.soname(self.abi_major)
        if soname:  # c_smoke records the SONAME, so the loader looks for that file name next to it ($ORIGIN)
            shutil.copy2(lib, self.test / soname)
        smoke_src = self.core / "tests" / "c_smoke.c"
        if smoke_src.is_file():
            cmd = [*ZIG, "cc", "-target", self.t.triple, "-std=c99", "-pedantic", "-Wall", "-Wextra",
                   f"-I{self.core / 'include'}", str(smoke_src)]
            if self.t.os == "windows":
                cmd += [str(self.dist / self.t.import_lib)]
            else:
                cmd += [f"-L{self.test}", f"-l{NAME}", "-Wl,-rpath,$ORIGIN", "-lm"]
            cmd += ["-o", str(self.test / f"c_smoke{self.t.exe}")]
            code, text = run(cmd, self.core, self.verbose)
            if text:
                print(text)
            if code != 0:
                sys.exit(f"build of c_smoke FAILED for {self.t.rid}")
            print("built test/c_smoke")

        self.write_build_info()

    def version_resource(self) -> Path:
        """core/src/iai_actuator_driver.rc.in filled in by core/build.py's version_resource (one template, one rule)."""
        import importlib.util

        spec = importlib.util.spec_from_file_location("iad_core_build", self.core / "build.py")
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.version_resource(self.out / "obj-shared", self.core)

    def write_build_info(self) -> None:
        _, zig_version = run([*ZIG, "version"], self.core, False)
        info = {
            "target": self.t.rid,
            "zig_triple": self.t.triple,
            "zig_version": zig_version.strip(),
            "cxx_flags": [f for f in self.cxx_flags if not f.startswith("-I")],
            "commit": os.environ.get("GITHUB_SHA", ""),
            "ref": os.environ.get("GITHUB_REF_NAME", ""),
            "files": sorted(p.name for p in self.dist.iterdir() if p.is_file()),
        }
        (self.dist / "build-info.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")

    def run_tests(self) -> int:
        here = host_rid()
        if here != self.t.rid:
            print(f"--run: {self.t.rid} does not run on this machine ({here}); built only")
            return 0
        print(f"\n== running test/ec_tests{self.t.exe} --quiet")
        runner = self.core / "tests" / "run_shards.py"
        if runner.is_file():  # the tests as several processes at once, merged into one summary
            spec = importlib.util.spec_from_file_location("run_shards", runner)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module  # dataclasses look the module up by name
            spec.loader.exec_module(module)
            code = module.run(self.test / f"ec_tests{self.t.exe}", shards=self.shards, quiet=True, cwd=self.core)
        else:
            code = subprocess.run([str(self.test / f"ec_tests{self.t.exe}"), "--quiet"], cwd=self.core).returncode
        if code != 0:
            return code
        smoke = self.test / f"c_smoke{self.t.exe}"
        if smoke.is_file():
            print(f"\n== running test/{smoke.name}")
            code = subprocess.run([str(smoke)], cwd=self.test).returncode
        return code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Release build of the native core for one target (zig).")
    parser.add_argument("--target", required=True, choices=sorted(TARGETS))
    parser.add_argument("--core", type=Path, default=Path("core"), help="the core folder (default: core)")
    parser.add_argument("--out", type=Path, help="output folder (default: build/native/<target>)")
    parser.add_argument("--run", action="store_true", help="run ec_tests and c_smoke when the target runs here")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    parser.add_argument("--shards", type=int, default=None,
                        help="--run: ec_tests processes at once (default: half the CPUs, at most 6; 1: one process)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass
    target = TARGETS[args.target]
    core = args.core.resolve()
    out = (args.out or Path("build") / "native" / target.rid).resolve()
    if out.exists():
        shutil.rmtree(out)  # a release build always starts clean
    t0 = time.monotonic()
    builder = Builder(target, core, out, args.jobs, args.verbose, args.shards)
    builder.build()
    code = builder.run_tests() if args.run else 0
    print(f"({time.monotonic() - t0:.1f} s) {out}")
    return code


if __name__ == "__main__":
    sys.exit(main())
