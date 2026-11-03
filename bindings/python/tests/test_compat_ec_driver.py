"""The deprecated ``ec_driver`` import name gives the same module objects as ``iai_actuator_driver``.

rename-project: keep-file
"""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

PROBE = r"""
import importlib, pkgutil, sys, warnings


def own_spec(module):
    # the module keeps its real spec, not the alias's (iai_actuator_driver.ec: the backend's implementation module)
    name = module.__spec__.name
    return name.startswith("iai_actuator_driver.") and sys.modules[name] is module


with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    import ec_driver
    import ec_driver.ec
    from ec_driver.sim import SimulatedEC
    import ec_driver as again
import iai_actuator_driver
import iai_actuator_driver.ec
import iai_actuator_driver.sim
assert ec_driver is iai_actuator_driver is again
assert ec_driver.ec is iai_actuator_driver.ec
assert ec_driver.ec.Elecylinder is iai_actuator_driver.ec.Elecylinder
assert SimulatedEC is iai_actuator_driver.sim.SimulatedEC
assert own_spec(iai_actuator_driver.ec), iai_actuator_driver.ec.__spec__
for info in pkgutil.iter_modules(iai_actuator_driver.__path__):
    if info.name == "__main__":
        continue
    old = importlib.import_module("ec_driver." + info.name)
    new = importlib.import_module("iai_actuator_driver." + info.name)
    assert old is new, info.name
    assert sys.modules["ec_driver." + info.name] is new, info.name
    assert own_spec(new), new.__spec__
try:
    importlib.import_module("ec_driver.no_such_module")
except ModuleNotFoundError as exc:
    assert "ec_driver.no_such_module" in str(exc), exc
else:
    raise AssertionError("ec_driver.no_such_module imported")
deprecations = [w for w in caught if issubclass(w.category, DeprecationWarning) and "ec_driver" in str(w.message)]
assert len(deprecations) == 1, [str(w.message) for w in caught]
print("ok")
"""


class EcDriverAliasTest(unittest.TestCase):
    def test_old_import_name_maps_to_the_same_modules_and_warns_once(self) -> None:
        # A fresh interpreter: the warning is raised on the first import only.
        result = subprocess.run([sys.executable, "-c", PROBE], cwd=ROOT, capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "ok")


if __name__ == "__main__":
    unittest.main()
