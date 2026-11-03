"""Where the GUI finds the files it serves and where it writes its own files.

Three ways to run it:

* a source checkout (the repository root, which holds ``apps/gui`` and ``bindings/python``): files next to the code,
  backups in the checkout's ``backups`` folder;
* an installed package (``pip install ./bindings/python ./apps/gui``): the pages come with the ``gui`` package, the documents are not installed,
  backups go to a folder of the user's own (see :func:`data_dir`), never into site-packages and never into whatever
  folder the GUI happened to be started from (a shortcut can start it in the Windows system folder);
* the ``iai-gui.exe`` bundle (PyInstaller one-file): everything is unpacked under ``sys._MEIPASS`` (``gui/*.html``,
  ``gui/*.js``, ``gui/*.css``, ``docs/*.md``, ``README.md``, ``CHANGELOG.md``); backups go next to ``iai-gui.exe``, or to
  the user's own folder when that folder cannot be written to (an exe under Program Files).

Every path is worked out when it is asked for, so a test can patch ``sys.frozen`` / ``sys._MEIPASS``.
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

__all__ = ["frozen", "bundle_root", "gui_file", "doc_file", "user_dir", "data_dir", "run_as"]

_GUI_DIR = Path(__file__).resolve().parent


def frozen() -> bool:
    """True inside the iai-gui.exe bundle (``sys.frozen`` and ``sys._MEIPASS`` set)."""
    return bool(getattr(sys, "frozen", False)) and bool(getattr(sys, "_MEIPASS", None))


def _source_root() -> Path:
    """The repository root of a source checkout (two folders above this one; nothing useful when installed)."""
    return _GUI_DIR.parents[1]  # apps/gui -> the repository root


def _checkout() -> bool:
    return (_source_root() / "bindings" / "python" / "pyproject.toml").is_file()


def bundle_root() -> Path:
    """``sys._MEIPASS`` in the bundle, else the repository root of a source checkout."""
    if frozen():
        return Path(str(getattr(sys, "_MEIPASS")))
    return _source_root()


def gui_file(name: str) -> Path:
    """A page, script or stylesheet of the GUI (``gui/<name>`` in the bundle, next to this file otherwise)."""
    return (bundle_root() / "gui" if frozen() else _GUI_DIR) / name


def doc_file(rel: str) -> Path:
    """A document the About panel links (``README.md``, ``CHANGELOG.md``, ``docs/*.md``), relative to the root."""
    return bundle_root() / rel


def _writable(folder: Path) -> bool:
    """Whether a file can be created in ``folder``: tried, not guessed (``os.access`` says nothing about the access
    rights of Program Files on Windows)."""
    try:
        with tempfile.TemporaryFile(dir=folder):
            return True
    except OSError:
        return False


def data_dir() -> Path:
    """The user's own folder for what the GUI writes: ``%LOCALAPPDATA%/IAI Actuator Driver`` on Windows,
    ``~/Library/Application Support/IAI Actuator Driver`` on macOS, else ``$XDG_DATA_HOME/iai-actuator-driver``
    (default ``~/.local/share/iai-actuator-driver``). Only the path: nothing is created here."""
    if sys.platform == "win32":
        base = os.environ.get("LOCALAPPDATA")
        return (Path(base) if base else Path.home() / "AppData" / "Local") / "IAI Actuator Driver"
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Application Support" / "IAI Actuator Driver"
    base = os.environ.get("XDG_DATA_HOME")
    return (Path(base) if base else Path.home() / ".local" / "share") / "iai-actuator-driver"


def user_dir() -> Path:
    """Where the GUI writes (backups): next to iai-gui.exe in the bundle when that folder can be written to, the
    repository root in a source checkout, else the user's own folder (:func:`data_dir`)."""
    if frozen():
        beside = Path(sys.executable).resolve().parent
        return beside if _writable(beside) else data_dir()
    if _checkout():
        return _source_root()
    return data_dir()


def run_as() -> str:
    """How the GUI runs: ``"iai-gui.exe"``, ``"source checkout"`` or ``"installed package"``."""
    if frozen():
        return "iai-gui.exe"
    return "source checkout" if _checkout() else "installed package"
