"""Platform wheels that carry the native library (release builds); everything else is in pyproject.toml.

Without IAD_WHEEL_PLATFORM this file changes nothing: ``python -m build`` makes the sdist (source, needs the native
library) and a py3-none-any wheel, both without the library: they look for it elsewhere (iai_actuator_driver/native.py:
the environment variable IAI_ACTUATOR_DRIVER_NATIVE, then a source checkout), and the package does not import without
one (the pure-Python edition that ran without it was removed; 1.0.x is the last release with it).

With IAD_WHEEL_PLATFORM set to a wheel platform tag, the wheel is tagged py3-none-<tag> and marked not pure
(Root-Is-Purelib: false). ctypes loads the library, so one wheel serves every Python 3.11+ of that platform:

    win_amd64                                       iai_actuator_driver/_native/win-x64/iai_actuator_driver.dll
    manylinux_2_17_x86_64.manylinux2014_x86_64      iai_actuator_driver/_native/linux-x64/libiai_actuator_driver.so
    manylinux_2_17_aarch64.manylinux2014_aarch64    iai_actuator_driver/_native/linux-arm64/libiai_actuator_driver.so

The release workflow copies the library into that folder first. The build stops when it is missing, or when another
platform's folder holds a library too, so a wheel never carries a wrong or extra binary.

Every build stops when LICENSE, NOTICE or NATIVE_THIRD_PARTY.txt (the license-files of pyproject.toml) is missing next
to this file.
"""

from __future__ import annotations

import os
from pathlib import Path

from setuptools import setup
from setuptools.dist import Distribution

try:  # setuptools 70.1 and later
    from setuptools.command.bdist_wheel import bdist_wheel
except ImportError:  # older setuptools: the wheel package's command
    from wheel.bdist_wheel import bdist_wheel  # type: ignore[no-redef]

PLATFORM = os.environ.get("IAD_WHEEL_PLATFORM", "").strip()
HERE = Path(__file__).resolve().parent
NATIVE = HERE / "iai_actuator_driver" / "_native"
LIBRARY_SUFFIXES = (".dll", ".so", ".dylib")

# the license-files of pyproject.toml: setuptools only warns when one is missing, and no package ships without them
MISSING = [name for name in ("LICENSE", "NOTICE", "NATIVE_THIRD_PARTY.txt") if not (HERE / name).is_file()]
if MISSING:
    raise SystemExit(f"{', '.join(MISSING)} missing in {HERE}: copy the repository's file(s) here (LICENSE from its "
                     "root, NOTICE and NATIVE_THIRD_PARTY.txt from licenses/)")


def rid_of(tag: str) -> tuple[str, str]:
    """(rid, library file name) of a wheel platform tag; the native loader's folder names."""
    first = tag.split(".")[0]
    if first == "win_amd64":
        return "win-x64", "iai_actuator_driver.dll"
    if first.startswith(("manylinux", "musllinux", "linux")):
        if first.endswith("_x86_64"):
            return "linux-x64", "libiai_actuator_driver.so"
        if first.endswith("_aarch64"):
            return "linux-arm64", "libiai_actuator_driver.so"
    if first.startswith("macosx"):
        if first.endswith("_arm64"):
            return "osx-arm64", "libiai_actuator_driver.dylib"
        if first.endswith("_x86_64"):
            return "osx-x64", "libiai_actuator_driver.dylib"
    raise SystemExit(f"IAD_WHEEL_PLATFORM={tag!r}: no native library folder for this platform tag")


class PlatformDistribution(Distribution):
    """A platform-specific distribution when IAD_WHEEL_PLATFORM is set: the files install to platlib, so the wheel
    holds the package at its root (not under .data/purelib/)."""

    def has_ext_modules(self) -> bool:
        return bool(PLATFORM) or super().has_ext_modules()


class PlatformWheel(bdist_wheel):
    """bdist_wheel with the py3-none-<platform> tag when IAD_WHEEL_PLATFORM is set."""

    def finalize_options(self) -> None:
        super().finalize_options()
        if PLATFORM:
            self.root_is_pure = False

    def get_tag(self) -> tuple[str, str, str]:
        if PLATFORM:
            return "py3", "none", PLATFORM
        return super().get_tag()

    def run(self) -> None:
        if PLATFORM:
            rid, name = rid_of(PLATFORM)
            wanted = NATIVE / rid / name
            if not wanted.is_file():
                raise SystemExit(f"IAD_WHEEL_PLATFORM={PLATFORM}: {wanted} is missing (copy the {rid} build there)")
            extra = [p for p in NATIVE.rglob("*") if p.is_file() and p.suffix in LIBRARY_SUFFIXES and p != wanted]
            if extra:
                raise SystemExit(f"IAD_WHEEL_PLATFORM={PLATFORM}: other native files would be packed: "
                                 + ", ".join(str(p) for p in extra))
        super().run()


setup(distclass=PlatformDistribution, cmdclass={"bdist_wheel": PlatformWheel})
