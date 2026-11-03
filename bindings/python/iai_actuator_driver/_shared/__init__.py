"""The Python parts of the driver modules: what runs in Python, beside the core's calls.

``_shared/<module>.py`` holds the part of ``iai_actuator_driver.<module>`` that has no I/O of its own (constants,
records, value types, the coordinated-motion planner, the simulator's profiles) and, for ``info`` and ``check``, the
shared Python functions (tables, decoders, the readers and the self-test, which reach a unit only through an axis's
public methods) that those modules re-bind (:func:`rebound`, :func:`adopt`). The driver modules import them from here,
so the public module has the very same objects.

A module here imports other driver modules by their public names (``from ..errors import ...``) or from this folder.
The public classes and functions defined here carry the public module name (:func:`publish`), so their reprs, pickles
and documentation name ``iai_actuator_driver.<module>``; the private ones keep this folder's name, under which they can
always be found.
"""

from __future__ import annotations

import types
from typing import Any

PACKAGE = __name__.rpartition(".")[0]  # "iai_actuator_driver"


def publish(namespace: dict[str, Any], module: str) -> None:
    """Give the public classes and functions defined in a shared module (``namespace``: its globals) the name of the
    public module they belong to, ``iai_actuator_driver.<module>``. Called at the end of each shared module."""
    own = namespace["__name__"]
    public = f"{PACKAGE}.{module}"
    for name, value in list(namespace.items()):
        if name.startswith("_") or not isinstance(value, (type, types.FunctionType)):
            continue
        if getattr(value, "__module__", None) == own:
            try:
                value.__module__ = public
            except (AttributeError, TypeError):  # a type that does not take it: keeps its own name
                pass


def rebound(function: types.FunctionType, namespace: dict[str, Any]) -> types.FunctionType:
    """``function`` of a shared module with ``namespace`` (a backend module's globals) as its globals: the names it
    calls are looked up there, so ``mock.patch("iai_actuator_driver.<module>.<name>")`` reaches the code that runs."""
    copy = types.FunctionType(function.__code__, namespace, function.__name__, function.__defaults__,
                              function.__closure__)  # fmt: skip
    copy.__kwdefaults__ = function.__kwdefaults__
    copy.__dict__.update(function.__dict__)
    copy.__doc__, copy.__qualname__, copy.__module__ = function.__doc__, function.__qualname__, namespace["__name__"]
    copy.__annotations__ = dict(function.__annotations__)
    return copy


def adopt(source: types.ModuleType, namespace: dict[str, Any], skip: tuple[str, ...] = ()) -> None:
    """Put every name of the shared module ``source`` into ``namespace`` (a backend module's globals), its functions
    :func:`rebound` to that namespace; the dunder names and ``skip`` are left out."""
    own = vars(source)
    for name, value in own.items():
        if name.startswith("__") or name in skip:
            continue
        if isinstance(value, types.FunctionType) and value.__globals__ is own:
            value = rebound(value, namespace)
        namespace[name] = value
