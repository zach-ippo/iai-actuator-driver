"""Every Python example runs with its simulator option on the default backend, the one a user gets (the core when the
library is found): a public name an example uses cannot be missing on that backend unnoticed. No hardware."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import os
import subprocess
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import ModuleType, SimpleNamespace

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


def example(name: str) -> ModuleType:
    """An example imported as a module (its main() is not run)."""
    spec = importlib.util.spec_from_file_location(f"example_{name}", EXAMPLES / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status(**bits: object) -> SimpleNamespace:
    """A status with every flag off but `bits`."""
    names = ("servo_on", "homed", "in_position", "moving", "pushing", "push_contact", "push_missed", "modbus_enabled",
             "emergency_stop", "alarm", "warning")
    return SimpleNamespace(**{**{n: False for n in names}, "position_mm": 1.0, "alarm_code": 0, **bits})


class NoCodeTest(unittest.TestCase):
    """An alarm or warning bit with code 0 (a PCON keeps its warning bit set with no code) is shown, never hidden or
    printed as 0x000."""

    def test_the_one_line_status(self) -> None:
        for name in ("first_move", "push_press", "stored_positions"):
            with self.subTest(name):
                describe = example(name).describe
                self.assertTrue(describe(status(warning=True), "mm").endswith("; warning (no code)"))
                self.assertTrue(describe(status(alarm=True), "mm").endswith("; alarm (no code)"))
                self.assertNotIn("no code", describe(status(), "mm"))

    def test_the_machine_sequence_report(self) -> None:
        module = example("machine_sequence")
        cell = SimpleNamespace(status_all=lambda **kw: {"x": status(warning=True), "y": status(alarm=True)},
                               units_of=lambda name: "mm")
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            module.report_axes(cell)
        self.assertEqual(out.getvalue().splitlines(), ["  x: at 1.00 mm, servo OFF, warning (no code)",
                                                       "  y: at 1.00 mm, servo OFF, alarm (no code)"])


if __name__ == "__main__":
    unittest.main()
