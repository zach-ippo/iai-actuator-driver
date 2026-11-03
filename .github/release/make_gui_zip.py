#!/usr/bin/env python3
"""Zip iai-gui.exe for the release, with its README and the licence files.

    python .github/release/make_gui_zip.py --exe dist/gui/iai-gui.exe --version 1.0.0 --out dist

writes <out>/iai-gui-<version>-win-x64.zip:

    iai-gui.exe                 the GUI (PyInstaller, one file, Python inside)
    README.txt                  apps/gui/packaging/README.txt
    LICENSE, NOTICE             this project's licence (Apache License 2.0) and notice: the repository's LICENSE and
                                licenses/NOTICE
    NATIVE_THIRD_PARTY.txt      licenses/NATIVE_THIRD_PARTY.txt: what is linked into iai_actuator_driver.dll, with its
                                licence (every release asset carries these three: check_package.NOTICES)
    THIRD_PARTY_NOTICES.txt     apps/gui/packaging/THIRD_PARTY_NOTICES.txt: what else the exe carries, with its licence
    PYTHON_LICENSE.txt          the LICENSE.txt of the Python inside the exe (--python-license; by default that of the
                                Python running this script, so run it with the Python that built the exe)

release.yml (job gui) and local_release.py (step gui) both call it. Nothing is written when a file is missing.
"""

from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path

from check_package import NOTICES  # this folder: the script's own; the files every release asset carries

ROOT = Path(__file__).resolve().parents[2]
EXE = "iai-gui.exe"
PYTHON_LICENSE = "PYTHON_LICENSE.txt"
FROM_REPOSITORY = {  # name in the zip: file of the repository
    "README.txt": "apps/gui/packaging/README.txt",
    "THIRD_PARTY_NOTICES.txt": "apps/gui/packaging/THIRD_PARTY_NOTICES.txt",
    **NOTICES,  # LICENSE (the root's), NOTICE and NATIVE_THIRD_PARTY.txt (from licenses/)
}
CONTENTS = sorted([EXE, PYTHON_LICENSE, *FROM_REPOSITORY])


def zip_name(version: str) -> str:
    return f"iai-gui-{version}-win-x64.zip"


def make(exe: Path, version: str, out: Path, python_license: Path, root: Path = ROOT) -> Path:
    """Write the zip and return its path; SystemExit naming what is missing, and no zip, when a file is not there."""
    files = {EXE: exe, PYTHON_LICENSE: python_license, **{name: root / rel for name, rel in FROM_REPOSITORY.items()}}
    missing = [f"{name} ({path})" for name, path in sorted(files.items()) if not path.is_file()]
    if missing:
        raise SystemExit("make_gui_zip.py: missing " + ", ".join(missing) + "; no zip written")
    out.mkdir(parents=True, exist_ok=True)
    path = out / zip_name(version)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        for name in CONTENTS:
            zf.write(files[name], name)
    with zipfile.ZipFile(path) as zf:
        names = sorted(zf.namelist())
    if names != CONTENTS:
        raise SystemExit(f"make_gui_zip.py: unexpected content of {path.name}: {names}")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Zip iai-gui.exe with its README and the licence files.")
    parser.add_argument("--exe", type=Path, required=True, help="the iai-gui.exe PyInstaller wrote")
    parser.add_argument("--version", required=True, help="PEP 440 form, e.g. 1.0.0")
    parser.add_argument("--out", type=Path, default=Path("dist"))
    parser.add_argument("--python-license", type=Path, default=Path(sys.base_prefix) / "LICENSE.txt",
                        help="LICENSE.txt of the Python inside the exe (default: of the Python running this script)")
    args = parser.parse_args(argv)
    path = make(args.exe, args.version, args.out, args.python_license)
    print(f"{path} ({path.stat().st_size // 1024} KiB): {', '.join(CONTENTS)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
