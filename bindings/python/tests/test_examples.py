"""Every Python example runs with its simulator option on the default backend, the one a user gets (the core when the
library is found): a public name an example uses cannot be missing on that backend unnoticed. No hardware."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EXAMPLES = ROOT / "examples" / "python"
ARGS = {"draw_circle.py": ["--radius", "3", "--speed", "5", "--chord-error", "0.2"]}
"""Options beside ``--sim``. The circle: smaller, slower and coarser than its default, whose steps a busy PC can leave
too short for the simulated line it measures (an answer, exit 2, not the subject here)."""
TIMEOUT_S = 300.0
WORKERS = 3
BACKEND_VARS = ("IAI_DRIVER_BACKEND",)  # unset: as a user runs them


class ExamplesTest(unittest.TestCase):
    def test_every_example_runs_on_the_simulator(self) -> None:
        scripts = sorted(EXAMPLES.glob("*.py"))
        self.assertGreaterEqual(len(scripts), 9)
        env = {k: v for k, v in os.environ.items() if k not in BACKEND_VARS}
        cwd = self.enterContext(tempfile.TemporaryDirectory())  # an example that writes a file writes it here

        def run(script: Path) -> subprocess.CompletedProcess[str]:
            command = [sys.executable, str(script), "--sim", *ARGS.get(script.name, [])]
            return subprocess.run(command, capture_output=True, text=True, timeout=TIMEOUT_S, env=env, cwd=cwd)

        with ThreadPoolExecutor(max_workers=WORKERS) as pool:
            done = list(pool.map(run, scripts))
        for script, proc in zip(scripts, done):
            with self.subTest(script.name):
                self.assertEqual(proc.returncode, 0, f"{proc.stdout[-1500:]}\n{proc.stderr[-1500:]}")
                self.assertNotIn("Traceback", proc.stderr)


if __name__ == "__main__":
    unittest.main()
