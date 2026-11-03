"""Entry point of the iai-gui.exe bundle (PyInstaller, see iai-gui.spec): starts the GUI server."""

from gui.server import main

if __name__ == "__main__":
    raise SystemExit(main())
