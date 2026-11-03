# PyInstaller spec for iai-gui.exe: the web GUI as one Windows program with Python inside (one file, console).
#
# Build, from this folder, in a Python 3.11+ environment that has pyserial and PyInstaller:
#     pyinstaller iai-gui.spec              (writes dist\iai-gui.exe; build\ holds the intermediate files)
#     dist\iai-gui.exe --version            (quick check)
#     dist\iai-gui.exe --sim-only           (the GUI with simulators only: no serial port, no unit)
#
# The exe carries the driver's native library (iai_actuator_driver.dll), which is where the driver finds it at run time:
# iai_actuator_driver/_native/<platform>/ inside the bundle. native_library() below takes it from, in this order:
#   1. the environment variable IAI_ACTUATOR_DRIVER_NATIVE (the library file, or the folder that holds it): use it for a
#      build from a source checkout, e.g. the folder a "python core/build.py" run wrote to;
#   2. the installed driver package (the release workflow installs the win-x64 wheel first);
#   3. core/build/ in the repository.
# The build stops with a message when none of them has the library: the driver does not import without it.
#
# GitHub Release: .github/release/make_gui_zip.py (its docstring lists what the zip holds) makes
# iai-gui-<version>-win-x64.zip, for example iai-gui-1.0.0-win-x64.zip, from dist\iai-gui.exe and the licence files
# (release.yml job "gui" and local_release.py step gui run it). Something new in the bundle (a hidden import, a binary,
# a vendored web file) gets a line in packaging\THIRD_PARTY_NOTICES.txt; what is linked into the DLL, in the
# repository's licenses/NATIVE_THIRD_PARTY.txt.
# The exe is never committed: dist\ and build\ are in .gitignore.
#
# Bundle layout (relative to sys._MEIPASS, read through gui/resources.py): gui/*.html, gui/*.js, gui/*.css, the
# documents the About panel links (DOCS below, README.md, CHANGELOG.md), and the packages gui and iai_actuator_driver.
# Backups go next to iai-gui.exe, or to the user's own folder when that folder cannot be written to.

import importlib.util
import os
import platform
import struct
import sys

from PyInstaller.utils.hooks import collect_submodules

ROOT = os.path.normpath(os.path.join(SPECPATH, "..", ".."))  # noqa: F821 (SPECPATH: apps/gui, set by PyInstaller; ROOT: the repository root)


def here(*parts):
    return os.path.join(ROOT, *parts)


def platform_tag():
    """The folder name of a packaged build, as iai_actuator_driver.native names it: win-x64, linux-x64, osx-arm64..."""
    machine = platform.machine().lower()
    bits = struct.calcsize("P") * 8
    if machine in ("amd64", "x86_64", "x64"):
        arch = "x64" if bits == 64 else "x86"
    elif machine in ("arm64", "aarch64"):
        arch = "arm64"
    elif machine in ("x86", "i386", "i686"):
        arch = "x86"
    else:
        arch = machine or "unknown"
    os_name = {"win32": "win", "darwin": "osx"}.get(sys.platform, "linux")
    return f"{os_name}-{arch}"


def native_library():
    """[(library file, folder inside the bundle)] for Analysis(binaries=...); stops the build when there is none."""
    name = {"win32": "iai_actuator_driver.dll", "darwin": "libiai_actuator_driver.dylib"}.get(
        sys.platform, "libiai_actuator_driver.so")
    tried = []
    env = os.environ.get("IAI_ACTUATOR_DRIVER_NATIVE", "").strip()
    if env:  # an explicit choice: used alone, as the driver does at run time
        path = os.path.join(env, name) if os.path.isdir(env) else env
        tried.append(path)
        found = path if os.path.isfile(path) else None
    else:
        found = None
        package = importlib.util.find_spec("iai_actuator_driver")  # before sys.path is changed below: the installed one
        folders = list(package.submodule_search_locations or []) if package else []
        candidates = [os.path.join(f, "_native", platform_tag(), name) for f in folders]
        candidates += [os.path.join(f, "_native", name) for f in folders]
        candidates.append(here("core", "build", name))
        for path in candidates:
            tried.append(path)
            if os.path.isfile(path):
                found = path
                break
    if found is None:
        raise SystemExit(
            f"iai-gui.spec: the native library {name} was not found. Tried: {', '.join(tried) or '(nothing)'}. "
            "Install the win-x64 wheel of bindings/python, build the core (python core/build.py) or set "
            "IAI_ACTUATOR_DRIVER_NATIVE to the library or its folder.")
    return [(found, "/".join(("iai_actuator_driver", "_native", platform_tag())))]


binaries = native_library()
# So that collect_submodules finds gui and iai_actuator_driver without an install (after native_library(): an installed
# wheel is looked at first there).
sys.path[:0] = [os.path.join(ROOT, "apps"), os.path.join(ROOT, "bindings", "python")]


# The documents of docs/ that the About panel links (ABOUT_DOCS in apps/gui/server.py; test_gui.py checks the two lists
# against each other). Listed one by one on purpose: a document added to docs/ later must not ship in the exe by itself.
DOCS = ("GETTING_STARTED.md", "COORDINATED.md", "COMPATIBILITY.md", "DRIVER.md", "REFERENCE.md")

datas = [
    (here("apps", "gui", "*.html"), "gui"),
    (here("apps", "gui", "*.js"), "gui"),
    (here("apps", "gui", "*.css"), "gui"),
    *[(here("docs", name), "docs") for name in DOCS],
    (here("README.md"), "."),
    (here("CHANGELOG.md"), "."),
]

hiddenimports = (
    collect_submodules("iai_actuator_driver")
    + collect_submodules("gui", filter=lambda name: not name.split(".")[-1].startswith("test"))
    + [
        "serial",
        "serial.tools.list_ports",
        "serial.tools.list_ports_common",
        "serial.tools.list_ports_windows" if sys.platform == "win32" else "serial.tools.list_ports_posix",
        "serial.serialwin32" if sys.platform == "win32" else "serial.serialposix",
    ]
)

a = Analysis(  # noqa: F821
    [here("apps", "gui", "packaging", "iai_gui_entry.py")],
    pathex=[os.path.join(ROOT, "apps"), os.path.join(ROOT, "bindings", "python")],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Nothing in the GUI uses these (it speaks plain HTTP on 127.0.0.1 and serial). ssl and _ssl would put libssl-3.dll and
    # libcrypto-3.dll in the exe, which customers' vulnerability scanners then report for a program that never uses TLS;
    # _hashlib (OpenSSL's hashes, only imported by hashlib and hmac, which fall back to Python's own implementations)
    # would put libcrypto-3.dll in it again. gui/test_gui.py is left out above.
    excludes=["tkinter", "unittest", "ssl", "_ssl", "_hashlib"],
    noarchive=False,
)
pyz = PYZ(a.pure)  # noqa: F821

exe = EXE(  # noqa: F821
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="iai-gui",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
