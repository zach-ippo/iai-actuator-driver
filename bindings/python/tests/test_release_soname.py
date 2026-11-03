"""The Linux shared library of a release carries the ABI major in its SONAME (docs/NATIVE_ABI.md, "The library file").

build_native.py links the release library with ``-Wl,-soname,libiai_actuator_driver.so.<IAD_ABI_VERSION_MAJOR>``
(the header's major, as core/build.py and the CMake build do); make_sdk.py ships it as
``lib/libiai_actuator_driver.so.<major>`` with a text linker script ``lib/libiai_actuator_driver.so`` that names it, so
``-liai_actuator_driver`` works and a program records the versioned name; check_package.py refuses another SONAME. These
tests link a tiny library for linux-x64 with zig (no compile of the core, no serial port) and need the release scripts of
a source checkout.
"""

from __future__ import annotations

import importlib.util
import re
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import ModuleType

ROOT = Path(__file__).resolve().parent.parent
REPO = ROOT.parents[1]
RELEASE = REPO / ".github" / "release"
HEADER = REPO / "core" / "include" / "iai_actuator_driver.h"
TRIPLE = "x86_64-linux-gnu.2.17"


def _load(name: str) -> ModuleType:
    if str(RELEASE) not in sys.path:
        sys.path.insert(0, str(RELEASE))  # the release scripts import each other, as when run from their folder
    spec = importlib.util.spec_from_file_location(f"iad_release_{name}", RELEASE / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(f"iad_release_{name}", module)
    spec.loader.exec_module(module)
    return module


def _zig_ok() -> bool:
    try:
        return subprocess.run([sys.executable, "-m", "ziglang", "version"], capture_output=True).returncode == 0
    except OSError:
        return False


def _zig(*args: str, cwd: Path) -> None:
    done = subprocess.run([sys.executable, "-m", "ziglang", "cc", "-target", TRIPLE, *args], cwd=cwd,
                          capture_output=True, text=True)
    if done.returncode != 0:
        raise AssertionError(f"zig cc {' '.join(args)} failed:\n{done.stdout}{done.stderr}")


@unittest.skipUnless(RELEASE.is_dir() and HEADER.is_file(), "the release scripts are in a source checkout only")
class ReleaseSonameTest(unittest.TestCase):
    build: ModuleType
    sdk: ModuleType
    package: ModuleType
    major: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.build = _load("build_native")
        cls.sdk = _load("make_sdk")
        cls.package = _load("check_package")
        found = re.search(r"^#define IAD_ABI_VERSION_MAJOR (\d+)", HEADER.read_text(encoding="utf-8"), flags=re.M)
        assert found is not None
        cls.major = int(found.group(1))

    def test_the_soname_is_the_header_s_abi_major(self) -> None:
        self.assertEqual(self.build.abi_major(REPO / "core"), self.major)
        self.assertEqual(self.sdk.abi_major(HEADER), self.major)
        self.assertEqual(self.package.abi_major(), self.major)
        for rid in ("linux-x64", "linux-arm64"):
            self.assertEqual(self.build.TARGETS[rid].soname(self.major), f"libiai_actuator_driver.so.{self.major}")
            self.assertIn(f"-Wl,-soname,libiai_actuator_driver.so.{self.major}",
                          self.build.TARGETS[rid].linux_shared_flags(self.major, Path("exports.map")))
        self.assertIsNone(self.build.TARGETS["win-x64"].soname(self.major))
        # the checker takes that name only
        lib = "libiai_actuator_driver.so"
        self.assertTrue(self.package.soname_ok("linux-x64", f"{lib}.{self.major}"))
        self.assertFalse(self.package.soname_ok("linux-x64", lib))
        self.assertFalse(self.package.soname_ok("linux-x64", f"{lib}.{self.major + 1}"))
        self.assertFalse(self.package.soname_ok("linux-x64", None))

    @unittest.skipUnless(_zig_ok(), "zig (python -m ziglang) is needed to link a Linux library")
    def test_a_library_linked_with_the_release_flags_carries_the_versioned_soname(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            (work / "lib.c").write_text("unsigned iad_abi_version(void) { return 7u; }\n", encoding="utf-8")
            script = work / "exports.map"
            script.write_text("{\n  global: iad_*;\n  local: *;\n};\n", encoding="utf-8")
            flags = self.build.TARGETS["linux-x64"].linux_shared_flags(self.major, script)
            _zig("-fPIC", *flags, "lib.c", "-o", "libiai_actuator_driver.so", cwd=work)
            info = self.package.elf_info((work / "libiai_actuator_driver.so").read_bytes())
            self.assertEqual(info["soname"], f"libiai_actuator_driver.so.{self.major}")
            self.assertTrue(self.package.soname_ok("linux-x64", info["soname"]))
            self.assertEqual(info["exports"], ["iad_abi_version"])

    @unittest.skipUnless(_zig_ok(), "zig (python -m ziglang) is needed to link a Linux program")
    def test_the_sdk_ships_the_library_under_its_soname_and_a_program_linked_with_l_records_it(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            native = work / "native"
            native.mkdir()
            (work / "lib.c").write_text("unsigned iad_abi_version(void) { return 7u; }\n", encoding="utf-8")
            (work / "app.c").write_text(
                "unsigned iad_abi_version(void);\nint main(void) { return (int)iad_abi_version() - 7; }\n",
                encoding="utf-8")
            script = work / "exports.map"
            script.write_text("{\n  global: iad_*;\n  local: *;\n};\n", encoding="utf-8")
            flags = self.build.TARGETS["linux-x64"].linux_shared_flags(self.major, script)
            _zig("-fPIC", *flags, "lib.c", "-o", str(native / "libiai_actuator_driver.so"), cwd=work)
            out = work / "out"
            self.assertEqual(self.sdk.main(["--rid", "linux-x64", "--version", "1.0.0", "--native", str(native),
                                            "--root", str(REPO), "--out", str(out)]), 0)
            base = "iai_actuator_driver-sdk-1.0.0-linux-x64"
            soname = f"libiai_actuator_driver.so.{self.major}"
            with zipfile.ZipFile(out / f"{base}.zip") as zf:
                names = set(zf.namelist())
                self.assertIn(f"{base}/lib/{soname}", names)
                self.assertIn(f"{base}/lib/libiai_actuator_driver.so", names)
                self.assertEqual(zf.read(f"{base}/lib/{soname}"), (native / "libiai_actuator_driver.so").read_bytes())
                self.assertEqual(zf.read(f"{base}/lib/libiai_actuator_driver.so").decode(),
                                 f"/* GNU ld script: the library is {soname} (its SONAME; the ABI major is the version) */\n"
                                 f"INPUT({soname})\n")
                config = zf.read(f"{base}/lib/cmake/iai_actuator_driver/iai_actuator_driverConfig.cmake").decode()
                self.assertIn(f'IMPORTED_LOCATION "${{_IAD_PREFIX}}/lib/{soname}"', config)
                self.assertIn(f'IMPORTED_SONAME "{soname}"', config)
                self.assertNotIn("@ABI_MAJOR@", config)
                self.assertIn(soname, zf.read(f"{base}/README.txt").decode())
                zf.extractall(work / "unpacked")  # the way the release smoke test unpacks it: no symbolic links
            lib_dir = work / "unpacked" / base / "lib"
            _zig("app.c", f"-L{lib_dir}", "-liai_actuator_driver", "-o", "app", cwd=work)
            needed = self.package.elf_info((work / "app").read_bytes())["needed"]
            self.assertIn(soname, needed)
            self.assertNotIn("libiai_actuator_driver.so", needed)

    def test_a_windows_sdk_has_no_versioned_library(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            native = work / "native"
            native.mkdir()
            for name in ("iai_actuator_driver.dll", "iai_actuator_driver.lib"):
                (native / name).write_bytes(b"MZ")
            out = work / "out"
            self.assertEqual(self.sdk.main(["--rid", "win-x64", "--version", "1.0.0", "--native", str(native),
                                            "--root", str(REPO), "--out", str(out)]), 0)
            with zipfile.ZipFile(out / "iai_actuator_driver-sdk-1.0.0-win-x64.zip") as zf:
                names = zf.namelist()
            self.assertFalse([n for n in names if ".so" in n])
            self.assertTrue(any(n.endswith("/bin/iai_actuator_driver.dll") for n in names))


if __name__ == "__main__":
    unittest.main()
