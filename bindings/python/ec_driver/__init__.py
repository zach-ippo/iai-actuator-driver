"""Deprecated import name: ``ec_driver`` is now ``iai_actuator_driver`` (the project is the IAI Actuator Driver).

Kept so existing code keeps working: ``import ec_driver`` and ``from ec_driver.<module> import ...`` (any submodule)
give the very same module objects as ``iai_actuator_driver`` (no copies), so for example
``ec_driver.ec.Elecylinder is iai_actuator_driver.ec.Elecylinder``. A DeprecationWarning is raised once, on the first
import. Please import ``iai_actuator_driver`` instead; this alias will be removed in 2.0.
"""

import importlib
import importlib.abc
import importlib.util
import sys
import warnings

import iai_actuator_driver as _package

_OLD = "ec_driver"
_NEW = "iai_actuator_driver"


class _AliasLoader(importlib.abc.Loader):
    """Returns the already imported ``iai_actuator_driver.<name>`` module instead of creating a new one."""

    def __init__(self, new_name):
        self._new_name = new_name
        self._real_spec = None

    def create_module(self, spec):
        module = importlib.import_module(self._new_name)
        self._real_spec = module.__spec__
        return module

    def exec_module(self, module):
        # The import system has just set __spec__ to the alias spec: give the module its own spec back so it keeps
        # its real name for reload(), tracebacks and tools. Nothing is executed a second time.
        if self._real_spec is not None:
            module.__spec__ = self._real_spec


class _AliasFinder(importlib.abc.MetaPathFinder):
    """Maps ``ec_driver.<name>`` to ``iai_actuator_driver.<name>`` (runs before the normal path finder)."""

    alias_of = _OLD

    def find_spec(self, fullname, path=None, target=None):
        if not fullname.startswith(_OLD + ".") or fullname.endswith(".__main__"):
            return None
        new_name = _NEW + fullname[len(_OLD):]
        try:
            real = importlib.util.find_spec(new_name)
        except (ImportError, ValueError):
            return None
        if real is None:
            return None
        return importlib.util.spec_from_loader(fullname, _AliasLoader(new_name),
                                               is_package=real.submodule_search_locations is not None)


warnings.warn("the ec_driver package was renamed to iai_actuator_driver; import iai_actuator_driver instead "
              "(the ec_driver alias will be removed in 2.0)", DeprecationWarning, stacklevel=2)

if not any(getattr(finder, "alias_of", None) == _OLD for finder in sys.meta_path):
    sys.meta_path.insert(0, _AliasFinder())
for _name, _module in list(sys.modules.items()):
    if _name.startswith(_NEW + "."):
        sys.modules.setdefault(_OLD + _name[len(_NEW):], _module)
sys.modules[_OLD] = _package
