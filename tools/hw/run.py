"""``python tools/hw/run.py``: runs the harness from a checkout (the same entry point as ``python -m tools.hw``)."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[0] = str(ROOT)  # the repository root instead of this folder, so that ``tools.hw`` is found and nothing in here shadows a module

from tools.hw.main import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
