"""The implementation the package runs on (``_backend.py``): the core, loaded at import; ``IAI_DRIVER_BACKEND``; the
module layout.

No hardware and no serial port.
"""

from __future__ import annotations

import ast
import importlib
import os
import pickle
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import iai_actuator_driver
from iai_actuator_driver import _backend as driver_backend
from iai_actuator_driver import native

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = Path(iai_actuator_driver.__file__).resolve().parent  # the imported one: the source tree, or the installed wheel
REMOVED = "1.0.x is the last release with it"


def run_python(code: str, **env: str) -> subprocess.CompletedProcess[str]:
    """``python -c code`` in a fresh process (the variable is read at the first import) with ``env`` added."""
    full = {**os.environ, "PYTHONPATH": str(ROOT), **env}
    if "IAI_DRIVER_BACKEND" not in env:
        full.pop("IAI_DRIVER_BACKEND", None)
    return subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=full, capture_output=True, text=True,
                          timeout=120)  # fmt: skip


def last_line(done: subprocess.CompletedProcess[str]) -> str:
    return done.stderr.strip().splitlines()[-1] if done.stderr.strip() else ""


class BackendTest(unittest.TestCase):
    def test_unset_auto_and_native_are_the_core(self) -> None:
        for value, start in ((None, "auto (the default): "), ("", "auto (the default): "), ("auto", "auto: "),
                             (" Native ", "native: ")):  # fmt: skip
            with self.subTest(value=value):
                backend, reason = driver_backend.resolve(value)
                self.assertEqual(backend, "native")
                self.assertTrue(reason.startswith(start), reason)
                self.assertIn(str(native.library_path()), reason)

    def test_python_is_a_one_line_import_error(self) -> None:
        for value in ("python", " Python "):
            with self.subTest(value=value), self.assertRaises(ImportError) as caught:
                driver_backend.resolve(value)
            text = str(caught.exception)
            self.assertNotIn("\n", text)
            for needle in ("IAI_DRIVER_BACKEND=python", "pure-Python edition", "was removed", REMOVED):
                self.assertIn(needle, text)
        for code in ("import iai_actuator_driver", "import ec_driver", "import iai_actuator_driver.ec"):
            with self.subTest(code=code):
                done = run_python(code, IAI_DRIVER_BACKEND="python")
                self.assertNotEqual(done.returncode, 0)
                self.assertEqual(last_line(done), f"ImportError: {text}", done.stderr)

    def test_an_unknown_value_names_the_valid_ones(self) -> None:
        with self.assertRaises(ImportError) as caught:
            driver_backend.resolve("fast")
        self.assertIn("native, auto", str(caught.exception))
        self.assertNotIn("\n", str(caught.exception))

    def test_the_package_reports_its_backend(self) -> None:
        info = iai_actuator_driver.backend_info()
        self.assertEqual(info["backend"], iai_actuator_driver.BACKEND)
        self.assertEqual(iai_actuator_driver.BACKEND, "native")
        self.assertIn("backend_info", iai_actuator_driver.__all__)
        self.assertEqual(info["native_modules"], list(driver_backend.MODULES))
        self.assertTrue(info["library"] and info["abi"] and info["core_version"] and info["reason"])

    def test_without_the_library_the_import_says_why(self) -> None:
        missing = str(Path(tempfile.gettempdir()) / "no-such-dir" / native.library_name())
        for env in ({}, {"IAI_DRIVER_BACKEND": "native"}, {"IAI_DRIVER_BACKEND": "auto"}):
            with self.subTest(env=env):
                done = run_python("import iai_actuator_driver", IAI_ACTUATOR_DRIVER_NATIVE=missing, **env)
                self.assertNotEqual(done.returncode, 0)
                self.assertIn("ImportError: iai_actuator_driver needs its native library", done.stderr)
                self.assertIn(missing, done.stderr)  # where it looked
                self.assertIn(REMOVED, done.stderr)

    def test_read_at_import(self) -> None:
        code = "import iai_actuator_driver as d; print(d.BACKEND, d.modbus.__name__, d.backend_info()['requested'])"
        for env, requested in (({"IAI_DRIVER_BACKEND": "native"}, "native"), ({}, "None")):
            with self.subTest(env=env):
                done = run_python(code, **env)
                self.assertEqual(done.returncode, 0, done.stderr)
                self.assertEqual(done.stdout.split(), ["native", "iai_actuator_driver.modbus", requested])


class LayoutTest(unittest.TestCase):
    """Each driver module is its own file, the implementation over the core: no facade, no second edition."""

    def test_each_driver_module_is_its_own_file(self) -> None:
        for name in driver_backend.MODULES:
            with self.subTest(name=name):
                module = importlib.import_module(f"iai_actuator_driver.{name}")
                self.assertEqual(Path(module.__file__).resolve(), PACKAGE / f"{name}.py")
                self.assertIs(sys.modules[f"iai_actuator_driver.{name}"], module)
                if name not in iai_actuator_driver.__all__:  # the package exports the function sim_line()
                    self.assertIs(getattr(iai_actuator_driver, name), module)
        self.assertEqual(sorted(p.stem for p in (PACKAGE / "_native").glob("*.py")), ["__init__", "_abi_tables"])

    def test_classes_functions_and_loggers_keep_the_public_module_name(self) -> None:
        from iai_actuator_driver import cell, ec, gateway, modbus, sim

        self.assertEqual(ec.Elecylinder.__module__, "iai_actuator_driver.ec")
        self.assertEqual(modbus.ModbusRTU.__module__, "iai_actuator_driver.modbus")
        self.assertEqual(gateway.UnsupportedLineError.__module__, "iai_actuator_driver.gateway")
        self.assertEqual(sim.SimulatedEC.__module__, "iai_actuator_driver.sim")
        self.assertEqual(cell.scan.__module__, "iai_actuator_driver.cell")
        self.assertIs(pickle.loads(pickle.dumps(ec.Elecylinder)), ec.Elecylinder)
        for name in ("cell", "ec", "gateway", "modbus", "monitor", "recovery"):
            with self.subTest(name=name):
                self.assertEqual(importlib.import_module(f"iai_actuator_driver.{name}").log.name,
                                 f"iai_actuator_driver.{name}")

    def test_shared_parts_are_one_object_under_the_old_and_the_new_name(self) -> None:
        from iai_actuator_driver import _protocol, cell, ec, errors, modbus, recovery, values

        for name in values.__all__:
            with self.subTest(name=name):
                self.assertIs(getattr(ec, name), getattr(values, name))
        for name in _protocol.__all__:
            with self.subTest(name=name):
                self.assertIs(getattr(modbus, name), getattr(_protocol, name))
        self.assertIs(modbus.TransportError, errors.TransportError)
        self.assertIs(recovery.RecoveryError, errors.RecoveryError)
        self.assertIs(recovery.UnitReplacedError, errors.UnitReplacedError)
        self.assertIs(cell.CellError, errors.CellError)
        self.assertEqual([n for n in modbus.__all__ if not hasattr(modbus, n)], [])
        self.assertEqual([n for n in ec.__all__ if not hasattr(ec, n)], [])

    def test_the_pure_edition_is_gone(self) -> None:
        """Removed after 1.0.x (``_pure/``): not in the package, and nothing imports it."""
        self.assertFalse((PACKAGE / "_pure").exists())
        for path in sorted(PACKAGE.rglob("*.py")):
            rel = path.relative_to(PACKAGE).as_posix()
            with self.subTest(path=rel):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                found = [n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
                found += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
                found += [a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and not n.module
                          for a in n.names]  # fmt: skip
                self.assertEqual([m for m in found if "_pure" in m.split(".")], [])


if __name__ == "__main__":
    unittest.main()
