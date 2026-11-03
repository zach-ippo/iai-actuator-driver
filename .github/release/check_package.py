#!/usr/bin/env python3
"""Check what a release package carries before it is attached to the release.

    python .github/release/check_package.py wheel dist/x.whl --rid win-x64 --version 1.0.0
    python .github/release/check_package.py sdist dist/x.tar.gz --version 1.0.0
    python .github/release/check_package.py nupkg nupkg/IaiActuatorDriver.1.0.0.nupkg --version 1.0.0
    python .github/release/check_package.py binary build/native/linux-x64/dist/libiai_actuator_driver.so --rid linux-x64

wheel: tag py3-none-<platform of rid> in the file name and in WHEEL, Root-Is-Purelib false, the library at
       iai_actuator_driver/_native/<rid>/ and built for that OS and CPU, no other native file, listed in RECORD; no
       pure-Python edition (_pure/).
sdist: source, needs the native library: no native library or binary of any platform inside and no pure-Python
       edition (_pure/, removed after 1.0.x); PKG-INFO carries the version and says that the sdist needs the library
       (SDIST_TEXT, from bindings/python/README.md).
nupkg: runtimes/<rid>/native/ for win-x64, linux-x64 and linux-arm64 (each built for its OS and CPU), the managed
       assembly under lib/, the .NET Framework copy targets under build/ and buildTransitive/, the nuspec version.
wheel, sdist and nupkg also: LICENSE, NOTICE and NATIVE_THIRD_PARTY.txt (the notices of what is linked into the
       native library), each equal to the repository's file (NOTICES: LICENSE, licenses/NOTICE,
       licenses/NATIVE_THIRD_PARTY.txt), inside (missing_notices(), which collect_release.py applies to every release
       asset) and the licence in the metadata (License-Expression / <license>: Apache-2.0).
binary: the file is built for that OS and CPU (PE x64, ELF x86-64, ELF aarch64); for the shared library (--library)
       also: it exports only iad_ functions, it needs no C++ runtime of its own (no libc++ / libstdc++ / libgcc /
       libunwind / winpthread import), and on Linux it needs glibc 2.17 at most (manylinux2014) and its timed waits run
       on the monotonic clock: it imports pthread_cond_timedwait only together with pthread_condattr_setclock or
       pthread_cond_clockwait (with glibc 2.17 symbols, libc++'s std::condition_variable waits with a timeout on an
       absolute wall-clock deadline, which a backward step of the system clock stretches; the core's own condition
       variable, core/src/monotonic_cv.hpp, sets CLOCK_MONOTONIC). This is a necessary condition only: a library that
       also keeps a std::condition_variable timed wait would pass, and the source scan of test_local_release.py
       (test_the_core_has_no_wall_clock_timed_wait) covers that; the two together are the guard. The headers are read
       here in Python, so the check is the same on every runner and for every target.
installed: run with the wheel installed (and the source package out of the way): iai_actuator_driver is imported from
       site-packages, its native module finds the library in _native/<rid>/ of the installed package, loads it, and
       the library reports the release version.

    python .github/release/check_package.py installed --rid linux-x64 --version 1.0.0
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
import tarfile
import zipfile
from pathlib import Path, PurePosixPath

LIB = {"win-x64": "iai_actuator_driver.dll", "linux-x64": "libiai_actuator_driver.so",
       "linux-arm64": "libiai_actuator_driver.so"}
WHEEL_PLATFORMS = {
    "win-x64": {"win_amd64"},
    "linux-x64": {"manylinux_2_17_x86_64", "manylinux2014_x86_64"},
    "linux-arm64": {"manylinux_2_17_aarch64", "manylinux2014_aarch64"},
}
NATIVE_SUFFIXES = (".dll", ".so", ".dylib", ".pyd", ".lib", ".a", ".exe")
ROOT = Path(__file__).resolve().parents[2]  # the repository (or the exported tree of a release build)
HEADER = ROOT / "core" / "include" / "iai_actuator_driver.h"
# the files every release asset carries (the library ships in the wheels, the nupkg, the SDK zips and, inside the exe,
# the GUI zip; NATIVE_THIRD_PARTY.txt holds the notices of what is linked into it): {name in the package: the repository
# file it is copied from}. LICENSE stays at the repository root (where GitHub finds the licence), the notices are in
# licenses/; every package keeps them under these names, side by side.
NOTICES = {
    "LICENSE": "LICENSE",
    "NOTICE": "licenses/NOTICE",
    "NATIVE_THIRD_PARTY.txt": "licenses/NATIVE_THIRD_PARTY.txt",
}
LICENSE_EXPRESSION = "Apache-2.0"
SDIST_TEXT = re.compile(r"The sdist is source, and needs the native\s+library")
"""What the sdist's description (PKG-INFO, from bindings/python/README.md) says about it (decision D8)."""
PURE = "iai_actuator_driver/_pure/"  # the pure-Python edition, removed after 1.0.x: in no package since


def notice_folder(asset: str, names: list[str]) -> str:
    """Where a release asset keeps the NOTICES files ('' for its root), from its file name: a wheel in
    <name>.dist-info/licenses/, an sdist in its top folder, an SDK zip in share/doc/iai_actuator_driver/ of its top
    folder, a nupkg and the GUI zip at the root."""
    if asset.endswith(".whl"):
        dist_info = next((n.split("/")[0] for n in names if n.split("/")[0].endswith(".dist-info")), "?.dist-info")
        return f"{dist_info}/licenses/"
    if asset.endswith(".tar.gz"):
        return asset[:-len(".tar.gz")] + "/"
    if asset.startswith("iai_actuator_driver-sdk-") and asset.endswith(".zip"):
        return asset[:-len(".zip")] + "/share/doc/iai_actuator_driver/"
    return ""


def notice_files(path: Path) -> tuple[list[str], dict[str, bytes]]:
    """The member names of a wheel, sdist, nupkg or zip, and {name: contents} of its NOTICES files."""
    wanted = lambda n: PurePosixPath(n).name in NOTICES  # noqa: E731
    if path.name.endswith(".tar.gz"):
        with tarfile.open(path) as tf:
            members = [m for m in tf.getmembers() if m.isfile()]
            return [m.name for m in members], {m.name: tf.extractfile(m).read() for m in members if wanted(m.name)}
    with zipfile.ZipFile(path) as zf:
        names = zf.namelist()
        return names, {n: zf.read(n) for n in names if wanted(n)}


def missing_notices(path: Path) -> list[str]:
    """What a release asset lacks of the NOTICES files, [] when it carries all where its kind keeps them; a copy that
    differs from the repository's file (line endings aside) counts as missing."""
    names, files = notice_files(path)
    folder = notice_folder(path.name, names)
    lacking = []
    for name, rel in NOTICES.items():
        data = files.get(folder + name)
        own = ROOT / rel
        if data is None:
            lacking.append(folder + name)
        elif own.is_file() and data.splitlines() != own.read_bytes().splitlines():
            lacking.append(f"{folder}{name} (not the repository's {rel})")
    return lacking


def check_notices(c: Checker, path: Path) -> None:
    lacking = missing_notices(path)
    c.expect(not lacking, f"carries the repository's {', '.join(NOTICES)} ({', '.join(lacking) or 'all there'})")


def abi_major() -> int | None:
    """IAD_ABI_VERSION_MAJOR of this checkout's C header (None when the header is not next to these scripts)."""
    try:
        text = HEADER.read_text(encoding="utf-8")
    except OSError:
        return None
    m = re.search(r"^#[ \t]*define[ \t]+IAD_ABI_VERSION_MAJOR[ \t]+(\d+)", text, flags=re.M)
    return int(m.group(1)) if m else None


def soname_ok(rid: str, soname: str | None) -> bool:
    """The Linux library's SONAME is libiai_actuator_driver.so.<ABI major>; the dll has none."""
    if soname is None:
        return False
    major = abi_major()
    if major is None:  # no header to read: any version number
        return re.fullmatch(re.escape(LIB[rid]) + r"\.\d+", soname) is not None
    return soname == f"{LIB[rid]}.{major}"


def binary_kind(data: bytes) -> str | None:
    """The rid a binary was built for (PE / ELF header), None when unknown."""
    if data[:2] == b"MZ" and len(data) > 0x40:
        pe = struct.unpack_from("<I", data, 0x3C)[0]
        if data[pe:pe + 4] == b"PE\0\0":
            machine = struct.unpack_from("<H", data, pe + 4)[0]
            return {0x8664: "win-x64", 0xAA64: "win-arm64", 0x14C: "win-x86"}.get(machine)
    if data[:4] == b"\x7fELF" and len(data) > 20:
        endian = "<" if data[5] == 1 else ">"
        machine = struct.unpack_from(endian + "H", data, 18)[0]
        if data[4] == 2:  # ELFCLASS64
            return {0x3E: "linux-x64", 0xB7: "linux-arm64"}.get(machine)
    return None


def _cstr(data: bytes, offset: int) -> str:
    end = data.find(b"\0", offset)
    return data[offset:end if end >= 0 else len(data)].decode("utf-8", "replace")


def elf_info(data: bytes) -> dict:
    """Exported (defined, global or weak, default visibility) and imported (undefined, global or weak) dynamic symbols,
    NEEDED, SONAME and the GLIBC_ versions a 64-bit little-endian ELF file needs."""
    shoff, = struct.unpack_from("<Q", data, 0x28)
    shentsize, shnum = struct.unpack_from("<HH", data, 0x3A)
    sections = [struct.unpack_from("<IIQQQQIIQQ", data, shoff + i * shentsize) for i in range(shnum)]
    # (name, type, flags, addr, offset, size, link, info, addralign, entsize)
    info: dict = {"exports": [], "imports": [], "needed": [], "soname": None, "glibc": []}
    for sec in sections:
        stype, offset, size, link = sec[1], sec[4], sec[5], sec[6]
        strtab = sections[link][4] if link < len(sections) else 0
        if stype == 11:  # SHT_DYNSYM
            for off in range(offset + 24, offset + size, 24):  # entry 0 is the null symbol
                name, st_info, st_other, shndx = struct.unpack_from("<IBBH", data, off)
                if shndx != 0 and (st_info >> 4) in (1, 2) and (st_other & 3) == 0 and (st_info & 0xF) != 4:
                    info["exports"].append(_cstr(data, strtab + name))
                elif shndx == 0 and (st_info >> 4) in (1, 2) and name:
                    info["imports"].append(_cstr(data, strtab + name).split("@")[0])
        elif stype == 6:  # SHT_DYNAMIC
            for off in range(offset, offset + size, 16):
                tag, val = struct.unpack_from("<qQ", data, off)
                if tag == 1:
                    info["needed"].append(_cstr(data, strtab + val))
                elif tag == 14:
                    info["soname"] = _cstr(data, strtab + val)
        elif stype == 0x6FFFFFFE:  # SHT_GNU_verneed
            off = offset
            for _ in range(sec[7]):
                _ver, cnt, _file, aux, nxt = struct.unpack_from("<HHIII", data, off)
                a = off + aux
                for _ in range(cnt):
                    _hash, _flags, _other, name, anext = struct.unpack_from("<IHHII", data, a)
                    version = _cstr(data, strtab + name)
                    if version.startswith("GLIBC_"):
                        info["glibc"].append(version)
                    a += anext
                off += nxt
    return info


def pe_info(data: bytes) -> dict:
    """Exported names and imported DLLs of a PE32+ file."""
    pe, = struct.unpack_from("<I", data, 0x3C)
    nsec, = struct.unpack_from("<H", data, pe + 6)
    optsize, = struct.unpack_from("<H", data, pe + 20)
    opt = pe + 24
    secs = [struct.unpack_from("<8sIIII", data, opt + optsize + i * 40) for i in range(nsec)]

    def off(rva: int) -> int:
        for _name, vsize, va, rawsize, rawptr in secs:
            if va <= rva < va + max(vsize, rawsize):
                return rva - va + rawptr
        raise ValueError(f"RVA {rva:#x} outside every section")

    export_rva, _ = struct.unpack_from("<II", data, opt + 112)
    import_rva, _ = struct.unpack_from("<II", data, opt + 120)
    info: dict = {"exports": [], "imports": []}
    if export_rva:
        e = off(export_rva)
        count, names = struct.unpack_from("<I", data, e + 24)[0], struct.unpack_from("<I", data, e + 32)[0]
        for i in range(count):
            info["exports"].append(_cstr(data, off(struct.unpack_from("<I", data, off(names) + 4 * i)[0])))
    if import_rva:
        d = off(import_rva)
        while True:
            name_rva, = struct.unpack_from("<I", data, d + 12)
            if not name_rva:
                break
            info["imports"].append(_cstr(data, off(name_rva)))
            d += 20
    return info


# A timed wait on a condition variable whose deadline can only be on the wall clock (CLOCK_REALTIME, the default of a
# pthread condition variable): imported without either way to wait on CLOCK_MONOTONIC.
WALL_CLOCK_WAIT = "pthread_cond_timedwait"
MONOTONIC_WAITS = ("pthread_condattr_setclock", "pthread_cond_clockwait")


def check_clock(c: Checker, info: dict, label: str) -> None:
    """The timed waits of a Linux library run on the monotonic clock (see the module docstring)."""
    imports = set(info["imports"])
    found = sorted(imports & {WALL_CLOCK_WAIT, *MONOTONIC_WAITS})
    ok = WALL_CLOCK_WAIT not in imports or any(name in imports for name in MONOTONIC_WAITS)
    c.expect(ok, f"{label}: timed waits on the monotonic clock ({WALL_CLOCK_WAIT} imported only with "
                 f"{' or '.join(MONOTONIC_WAITS)}; imports {', '.join(found) or 'none of them'})")


RUNTIME_RE = re.compile(r"(libc\+\+|libstdc\+\+|libgcc|libunwind|winpthread|msvcp|vcruntime)", re.I)


def check_library(c: Checker, data: bytes, rid: str, label: str, max_glibc: str) -> None:
    if rid.startswith("win"):
        info = pe_info(data)
        deps = info["imports"]
        print(f"       imports: {', '.join(deps)}")
    else:
        info = elf_info(data)
        deps = info["needed"]
        print(f"       NEEDED: {', '.join(deps)}; SONAME: {info['soname']}")
        major = abi_major()
        c.expect(soname_ok(rid, info["soname"]),
                 f"{label}: SONAME {info['soname']} is {LIB[rid]}.{'<ABI major>' if major is None else major}")
        newest = max(info["glibc"], key=lambda v: tuple(int(x) for x in v[6:].split(".")), default="GLIBC_0")
        limit = tuple(int(x) for x in max_glibc.split("."))
        c.expect(tuple(int(x) for x in newest[6:].split(".")) <= limit,
                 f"{label}: needs {newest} at most GLIBC_{max_glibc}")
        check_clock(c, info, label)
    exports = sorted(set(info["exports"]))
    others = [e for e in exports if not e.startswith("iad_")]
    c.expect(bool(exports) and not others,
             f"{label}: exports {len(exports)} iad_ function(s) and nothing else ({others[:8] or 'none other'})")
    runtime = [d for d in deps if RUNTIME_RE.search(d)]
    c.expect(not runtime, f"{label}: needs no C++ runtime library ({runtime or 'none'})")


class Checker:
    def __init__(self, what: str) -> None:
        self.what = what
        self.errors: list[str] = []

    def expect(self, ok: bool, message: str) -> None:
        print(f"{'ok    ' if ok else 'FAILED'} {message}")
        if not ok:
            self.errors.append(message)

    def done(self) -> int:
        if self.errors:
            print(f"\n{self.what}: {len(self.errors)} check(s) failed")
            return 1
        print(f"\n{self.what}: all checks passed")
        return 0


def check_binary(c: Checker, data: bytes, rid: str, label: str) -> None:
    kind = binary_kind(data)
    c.expect(kind == rid, f"{label} is built for {rid} (found {kind or 'unknown format'})")


def cmd_wheel(args: argparse.Namespace) -> int:
    path = Path(args.file)
    c = Checker(path.name)
    rid = args.rid
    parts = path.name[:-4].split("-")
    c.expect(len(parts) >= 5 and parts[-3] == "py3" and parts[-2] == "none",
             f"file name tag py3-none-* ({'-'.join(parts[-3:])})")
    plats = set(parts[-1].split(".")) if parts else set()
    c.expect(plats == WHEEL_PLATFORMS[rid], f"file name platform {sorted(plats)} is {sorted(WHEEL_PLATFORMS[rid])}")
    if args.version:
        c.expect(len(parts) >= 2 and parts[1] == args.version, f"file name version {parts[1]} is {args.version}")
    with zipfile.ZipFile(path) as zf:
        names = zf.namelist()
        wheel_meta = next((n for n in names if n.endswith(".dist-info/WHEEL")), None)
        record = next((n for n in names if n.endswith(".dist-info/RECORD")), None)
        c.expect(wheel_meta is not None, "has .dist-info/WHEEL")
        if wheel_meta:
            text = zf.read(wheel_meta).decode("utf-8")
            c.expect("Root-Is-Purelib: false" in text, "WHEEL: Root-Is-Purelib: false")
            tags = set(re.findall(r"^Tag: (\S+)\s*$", text, re.M))
            c.expect(tags == {f"py3-none-{p}" for p in WHEEL_PLATFORMS[rid]}, f"WHEEL tags {sorted(tags)}")
        lib = f"iai_actuator_driver/_native/{rid}/{LIB[rid]}"
        c.expect(lib in names, f"carries {lib}")
        if lib in names:
            check_binary(c, zf.read(lib), rid, lib)
            if record:
                c.expect(lib in zf.read(record).decode("utf-8"), f"RECORD lists {lib}")
        others = [n for n in names if n != lib and n.lower().endswith(NATIVE_SUFFIXES)]
        c.expect(not others, f"no other native file ({others[:5] or 'none'})")
        pure = [n for n in names if n.startswith(PURE)]
        c.expect(not pure, f"no pure-Python edition, {PURE} ({pure[:3] or 'none'})")
        metadata = next((n for n in names if n.endswith(".dist-info/METADATA")), None)
        text = zf.read(metadata).decode("utf-8") if metadata else ""
        c.expect(re.search(rf"^License-Expression: {LICENSE_EXPRESSION}\s*$", text, re.M) is not None,
                 f"METADATA: License-Expression: {LICENSE_EXPRESSION}")
    check_notices(c, path)
    return c.done()


def cmd_sdist(args: argparse.Namespace) -> int:
    path = Path(args.file)
    c = Checker(path.name)
    with tarfile.open(path) as tf:
        members = [m for m in tf.getmembers() if m.isfile()]
        native = [m.name for m in members if m.name.lower().endswith(NATIVE_SUFFIXES)]
        c.expect(not native, f"no native file inside ({native[:5] or 'none'})")
        binaries = []
        for m in members:
            fh = tf.extractfile(m)
            head = fh.read(4) if fh else b""
            if head[:2] == b"MZ" or head == b"\x7fELF" or head[:4] in (b"\xcf\xfa\xed\xfe", b"\xca\xfe\xba\xbe"):
                binaries.append(m.name)
        c.expect(not binaries, f"no executable binary inside ({binaries[:5] or 'none'})")
        pure = [m.name for m in members if f"/{PURE}" in m.name]
        c.expect(not pure, f"no pure-Python edition, {PURE} ({pure[:3] or 'none'})")
        pkg_info = next((m for m in members if PurePosixPath(m.name).name == "PKG-INFO"
                         and len(PurePosixPath(m.name).parts) == 2), None)
        c.expect(pkg_info is not None, "has PKG-INFO")
        text = tf.extractfile(pkg_info).read().decode("utf-8") if pkg_info else ""
        if pkg_info and args.version:
            m = re.search(r"^Version: (\S+)\s*$", text, re.M)  # setuptools on Windows writes PKG-INFO with CRLF
            c.expect(bool(m) and m.group(1) == args.version, f"PKG-INFO version {m.group(1) if m else None}")
        c.expect(re.search(rf"^License-Expression: {LICENSE_EXPRESSION}\s*$", text, re.M) is not None,
                 f"PKG-INFO: License-Expression: {LICENSE_EXPRESSION}")
        c.expect(SDIST_TEXT.search(text) is not None, "PKG-INFO says the sdist is source and needs the native library")
    check_notices(c, path)
    return c.done()


def cmd_nupkg(args: argparse.Namespace) -> int:
    path = Path(args.file)
    c = Checker(path.name)
    with zipfile.ZipFile(path) as zf:
        names = zf.namelist()
        nuspec = next((n for n in names if n.endswith(".nuspec") and "/" not in n), None)
        c.expect(nuspec is not None, "has a .nuspec")
        package_id = ""
        if nuspec:
            text = zf.read(nuspec).decode("utf-8")
            m = re.search(r"<version>([^<]+)</version>", text)
            if args.version:
                c.expect(bool(m) and m.group(1) == args.version, f"nuspec version {m.group(1) if m else None}")
            idm = re.search(r"<id>([^<]+)</id>", text)
            package_id = idm.group(1) if idm else ""
            c.expect(f'<license type="expression">{LICENSE_EXPRESSION}</license>' in text,
                     f'nuspec: <license type="expression">{LICENSE_EXPRESSION}</license>')
        c.expect(any(re.match(r"lib/[^/]+/[^/]+\.dll$", n) for n in names), "managed assembly under lib/<tfm>/")
        rids = [r for r in args.rids.split(",") if r]
        for rid in rids:
            lib = f"runtimes/{rid}/native/{LIB[rid]}"
            c.expect(lib in names, f"carries {lib}")
            if lib in names:
                check_binary(c, zf.read(lib), rid, lib)
        if "win-x64" in rids:
            for folder in ("build", "buildTransitive"):
                targets = f"{folder}/{package_id}.targets"
                c.expect(targets in names, f"carries {targets} (.NET Framework copy of the DLL)")
    check_notices(c, path)
    return c.done()


def cmd_binary(args: argparse.Namespace) -> int:
    path = Path(args.file)
    c = Checker(path.name)
    data = path.read_bytes()
    check_binary(c, data, args.rid, path.name)
    if args.library and binary_kind(data) == args.rid:
        check_library(c, data, args.rid, path.name, args.max_glibc)
    return c.done()


def cmd_installed(args: argparse.Namespace) -> int:
    c = Checker(f"installed wheel ({args.rid})")
    import importlib

    package = importlib.import_module("iai_actuator_driver")
    native = importlib.import_module("iai_actuator_driver.native")
    where = Path(package.__file__).resolve().parent
    c.expect("site-packages" in where.parts or "dist-packages" in where.parts,
             f"iai_actuator_driver imported from the installed wheel ({where})")
    found = native.find_library()
    want = where / "_native" / args.rid / LIB[args.rid]
    c.expect(found is not None and Path(found).resolve() == want, f"native library found at {want} ({found})")
    if found is not None:
        native.load()
        version = native.library_version()
        print(f"       loaded: library {version}, ABI {native.abi_version()}")
        if args.version:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from check_version import parse

            c.expect(parse(version) is not None and parse(version) == parse(args.version),
                     f"library version {version} is the release version {args.version}")
    return c.done()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check a release package.")
    sub = parser.add_subparsers(dest="command", required=True)
    w = sub.add_parser("wheel")
    w.add_argument("file")
    w.add_argument("--rid", required=True, choices=sorted(LIB))
    w.add_argument("--version", default="")
    s = sub.add_parser("sdist")
    s.add_argument("file")
    s.add_argument("--version", default="")
    n = sub.add_parser("nupkg")
    n.add_argument("file")
    n.add_argument("--version", default="")
    n.add_argument("--rids", default="win-x64,linux-x64,linux-arm64")
    b = sub.add_parser("binary")
    b.add_argument("file")
    b.add_argument("--rid", required=True, choices=sorted(LIB))
    b.add_argument("--library", action="store_true", help="the shared library: check exports and dependencies")
    b.add_argument("--max-glibc", default="2.17", help="newest glibc symbol version allowed (default 2.17)")
    i = sub.add_parser("installed")
    i.add_argument("--rid", required=True, choices=sorted(LIB))
    i.add_argument("--version", default="")
    args = parser.parse_args(argv)
    commands = {"wheel": cmd_wheel, "sdist": cmd_sdist, "nupkg": cmd_nupkg, "binary": cmd_binary,
                "installed": cmd_installed}
    return commands[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
