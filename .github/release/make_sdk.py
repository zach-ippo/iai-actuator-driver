#!/usr/bin/env python3
"""Zip the C/C++ SDK of one platform: the C ABI header, the prebuilt shared library (+ import library on Windows), the
command line, a CMake package config and the documents.

    python .github/release/make_sdk.py --rid win-x64 --version X.Y.Z --native build/native/win-x64/dist --out dist

writes dist/iai_actuator_driver-sdk-<version>-<rid>.zip, holding one folder of the same name:

    include/iai_actuator_driver.h                       the C ABI (C99; docs/dev/NATIVE_ABI.md)
    include/iai_actuator_driver_testing.h               the test hooks the library also exports (no stability promise)
    bin/iai_actuator_driver.dll                         Windows: the library
    bin/iai-actuator-driver-cpp[.exe]                   the command line (simulator: --sim)
    lib/iai_actuator_driver.lib                         Windows: import library (MSVC and MinGW linkers)
    lib/libiai_actuator_driver.so.<abi>                 Linux: the library; its SONAME is this name (<abi> = the ABI major)
    lib/libiai_actuator_driver.so                       Linux: a linker script (INPUT) naming it, for -liai_actuator_driver
    lib/cmake/iai_actuator_driver/*.cmake               find_package(iai_actuator_driver): iai_actuator_driver::shared
    examples/c/                                         the C examples of the repository
    share/doc/iai_actuator_driver/                      NATIVE_ABI.md, README.md, CHANGELOG.md, LICENSE, NOTICE,
                                                        NATIVE_THIRD_PARTY.txt (what is linked into the library),
                                                        build-info.json

Only the C ABI ships prebuilt: the C++ classes of core/include/iai_actuator_driver/*.hpp have no stable binary
interface across compilers, so C++ projects that want them build core/ from source (add_subdirectory).
The zip is not written when the repository's LICENSE, licenses/NOTICE or licenses/NATIVE_THIRD_PARTY.txt is missing.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import zipfile
from pathlib import Path

from check_package import NOTICES  # this folder: the script's own; the licence files every release asset carries

HERE = Path(__file__).resolve().parent
NAME = "iai_actuator_driver"
CLI = "iai-actuator-driver-cpp"
OS_OF = {"win-x64": "windows", "linux-x64": "linux", "linux-arm64": "linux"}

README = """\
IAI Actuator Driver C/C++ SDK {version} ({rid})
Unofficial driver for IAI actuators over serial Modbus RTU. Not an IAI product.

Contents
  include/iai_actuator_driver.h   the C ABI (C99, iad_ prefix); reference: share/doc/iai_actuator_driver/NATIVE_ABI.md
  include/iai_actuator_driver_testing.h   test hooks of the same library: no stability promise, never for production
  {lib_line}
  bin/{cli}   the command line; '{cli} --sim status' runs on the built-in simulator
  lib/cmake/iai_actuator_driver/  CMake package config

CMake
  find_package(iai_actuator_driver CONFIG REQUIRED)    # -DCMAKE_PREFIX_PATH=<this folder>
  target_link_libraries(your_app PRIVATE iai_actuator_driver::shared)

Without CMake
  {manual_line}

Requirements
  {requirements}

Built with zig {zig} (clang, libc++ linked statically) for {triple}.
The C++ classes of the core are not part of this SDK; build core/ from the source archive for those.
Hardware evidence per unit and feature: https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/COMPATIBILITY.md
Licence: Apache License 2.0, see share/doc/iai_actuator_driver/LICENSE and NOTICE. What is linked into the library
(libc++, zig's compiler_rt, on Windows the mingw-w64 runtime) has its own licences: NATIVE_THIRD_PARTY.txt, same folder.
"""


def abi_major(header: Path) -> int:
    """IAD_ABI_VERSION_MAJOR of the C header: the version of the Linux library's SONAME."""
    m = re.search(r"^#[ \t]*define[ \t]+IAD_ABI_VERSION_MAJOR[ \t]+(\d+)", header.read_text(encoding="utf-8"), flags=re.M)
    if m is None:
        sys.exit(f"IAD_ABI_VERSION_MAJOR not found in {header}")
    return int(m.group(1))


def linker_script(soname: str) -> str:
    """lib/libiai_actuator_driver.so on Linux: a GNU ld / lld script, so -liai_actuator_driver finds the library under its
    SONAME (a program then records the SONAME, which carries the ABI major). Text, so it survives every unzip tool (a
    symbolic link would not)."""
    return f"/* GNU ld script: the library is {soname} (its SONAME; the ABI major is the version) */\nINPUT({soname})\n"


def add_file(zf: zipfile.ZipFile, src: Path, arc: str, executable: bool = False) -> None:
    info = zipfile.ZipInfo.from_file(src, arc)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.create_system = 3  # Unix: without it a zip made on Windows has its mode bits ignored when unpacked on Linux
    info.external_attr = ((0o755 if executable else 0o644) | 0o100000) << 16
    zf.writestr(info, src.read_bytes())


def add_text(zf: zipfile.ZipFile, text: str, arc: str) -> None:
    info = zipfile.ZipInfo(arc, date_time=(2020, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    info.create_system = 3
    info.external_attr = (0o644 | 0o100000) << 16
    zf.writestr(info, text.encode("utf-8"))


def render(template: Path, values: dict[str, str]) -> str:
    text = template.read_text(encoding="utf-8")
    for key, value in values.items():
        text = text.replace(f"@{key}@", value)
    left = re.findall(r"@[A-Z_]+@", text)
    if left:
        sys.exit(f"{template.name}: unreplaced {sorted(set(left))}")
    return text


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Zip the C/C++ SDK of one platform.")
    parser.add_argument("--rid", required=True, choices=sorted(OS_OF))
    parser.add_argument("--version", required=True, help="PEP 440 form, X.Y.Z or X.Y.ZaN")
    parser.add_argument("--native", type=Path, required=True, help="the dist/ folder of build_native.py")
    parser.add_argument("--root", type=Path, default=Path("."), help="repository root")
    parser.add_argument("--out", type=Path, default=Path("dist"))
    args = parser.parse_args(argv)

    m = re.match(r"^(\d+)\.(\d+)\.(\d+)", args.version)
    if not m:
        sys.exit(f"--version {args.version!r} does not start with X.Y.Z")
    os_name = OS_OF[args.rid]
    arch = args.rid.split("-", 1)[1]
    windows = os_name == "windows"
    root, native = args.root.resolve(), args.native.resolve()

    header = root / "core" / "include" / f"{NAME}.h"
    testing_header = root / "core" / "include" / f"{NAME}_testing.h"
    lib = native / (f"{NAME}.dll" if windows else f"lib{NAME}.so")
    implib = native / f"{NAME}.lib"
    cli = native / (f"{CLI}.exe" if windows else CLI)
    notices = {name: root / rel for name, rel in NOTICES.items()}  # the licence files every package carries
    required = [header, testing_header, lib] + ([implib] if windows else []) + list(notices.values())
    missing = [str(p) for p in required if not p.is_file()]
    if missing:
        sys.exit("missing: " + ", ".join(missing))

    info_file = native / "build-info.json"
    info = json.loads(info_file.read_text(encoding="utf-8")) if info_file.is_file() else {}

    major_abi = abi_major(header)
    soname = f"lib{NAME}.so.{major_abi}"
    base = f"{NAME}-sdk-{args.version}-{args.rid}"
    args.out.mkdir(parents=True, exist_ok=True)
    zip_path = args.out / f"{base}.zip"
    values = {"VERSION": args.version, "CMAKE_VERSION": ".".join(m.groups()), "RID": args.rid, "OS": os_name,
              "ARCH": arch, "MAJOR": m.group(1), "MINOR": m.group(2), "ABI_MAJOR": str(major_abi)}

    with zipfile.ZipFile(zip_path, "w") as zf:
        add_file(zf, header, f"{base}/include/{header.name}")
        add_file(zf, testing_header, f"{base}/include/{testing_header.name}")
        if windows:
            add_file(zf, lib, f"{base}/bin/{lib.name}", executable=True)
            add_file(zf, implib, f"{base}/lib/{implib.name}")
        else:
            # the library under its SONAME, and the name -l looks for as a text script that points at it
            add_file(zf, lib, f"{base}/lib/{soname}", executable=True)
            add_text(zf, linker_script(soname), f"{base}/lib/{lib.name}")
        if cli.is_file():
            add_file(zf, cli, f"{base}/bin/{cli.name}", executable=True)
        cmake_dir = f"{base}/lib/cmake/{NAME}"
        add_text(zf, render(HERE / "sdk" / f"{NAME}Config.cmake.in", values), f"{cmake_dir}/{NAME}Config.cmake")
        add_text(zf, render(HERE / "sdk" / f"{NAME}ConfigVersion.cmake.in", values),
                 f"{cmake_dir}/{NAME}ConfigVersion.cmake")

        examples = root / "examples" / "c"
        if examples.is_dir():
            for path in sorted(p for p in examples.rglob("*") if p.is_file()):
                add_file(zf, path, f"{base}/examples/c/{path.relative_to(examples).as_posix()}")

        doc = f"{base}/share/doc/{NAME}"
        for rel in ("docs/dev/NATIVE_ABI.md", "README.md", "CHANGELOG.md"):
            path = root / rel
            if path.is_file():
                add_file(zf, path, f"{doc}/{path.name}")
        for name, path in notices.items():
            add_file(zf, path, f"{doc}/{name}")
        if info_file.is_file():
            add_file(zf, info_file, f"{doc}/build-info.json")

        if windows:
            lib_line = "bin/iai_actuator_driver.dll + lib/iai_actuator_driver.lib (import library)"
            manual = ("cl /I include app.c lib\\iai_actuator_driver.lib   (or: gcc -Iinclude app.c "
                      "lib/iai_actuator_driver.lib); put the DLL next to app.exe")
            req = "Windows 10 or later, x64 (the UCRT is part of Windows); 64-bit programs only"
        else:
            lib_line = (f"lib/{soname} (the library, SONAME {soname}) + lib/libiai_actuator_driver.so "
                        "(a linker script that names it, for -liai_actuator_driver)")
            manual = ("cc -Iinclude app.c -Llib -liai_actuator_driver -Wl,-rpath,'$ORIGIN/../lib' -lm -o bin/app\n"
                      "  (run from this folder: the program in bin/ finds lib/ through the rpath)")
            req = (f"Linux {'x86-64' if arch == 'x64' else 'ARM64 (aarch64)'} with glibc 2.17 or later; "
                   "serial ports through the kernel's tty drivers")
        add_text(zf, README.format(version=args.version, rid=args.rid, lib_line=lib_line, cli=cli.name,
                                   manual_line=manual, requirements=req, zig=info.get("zig_version", "?"),
                                   triple=info.get("zig_triple", "?")), f"{base}/README.txt")

    with zipfile.ZipFile(zip_path) as zf:
        names = zf.namelist()
    print(f"{zip_path} ({zip_path.stat().st_size // 1024} KiB, {len(names)} files)")
    for name in names:
        print(f"  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
