"""The core's stateless functions (alarm and CON parameter tables and rules, the info decoders) for the native
``alarms``, ``con_params`` and ``info`` modules: one entry point, ``iad_call_json(function, args_json, buffer...)``.

The function and its JSON conventions are in ``core/src/c_api_tables.cpp`` (exported by the shared library; asked of
the header owner for ``core/include/iai_actuator_driver.h``):

    iad_result iad_call_json(const char* function, const char* args_json, char* buffer, size_t buffer_size,
                             size_t* required);

``function`` is ``"<module>.<name>"`` (``"alarms.alarm_info"``, ``"con_params.parse"``, ``"info.decode_bits"``;
``"functions"`` lists them), ``args_json`` an object of named arguments or an array of such objects (one result per
object, in order). An unknown function is ``IAD_ERR_UNSUPPORTED``. Integers and floats keep their Python types both
ways (``250`` and ``250.0`` differ), so the results are Python's own values. A value JSON cannot carry is sent
as ``{"float": "inf"}`` or ``{"repr": "<text>"}`` (:func:`value`): the core names it as Python's ``repr()`` does.
"""

from __future__ import annotations

import ctypes
import json
import math
import threading
from collections.abc import Iterable, Mapping
from typing import Any

from .. import native as _n

_P = ctypes.POINTER
_R = ctypes.c_int32
_Z = ctypes.c_size_t
_STR = ctypes.c_char_p

FUNCTION = "iad_call_json"
PROTOTYPES: dict[str, tuple[Any, list[Any]]] = {
    FUNCTION: (_R, [_STR, _STR, _P(ctypes.c_char), _Z, _P(_Z)]),
}
"""Name -> (restype, argtypes) of the functions this area uses beyond what ``native.py`` binds."""

_START = 4096  # first output buffer: most results are small; tables grow it once (the size the core asks for)

_lock = threading.Lock()
_resolved: dict[str, bool] = {}


def has(name: str = FUNCTION) -> bool:
    """Whether the loaded library exports ``name`` with this signature (then bound in the library's function table)."""
    found = _resolved.get(name)
    if found is not None:
        return found
    with _lock:
        lib = _n.load()
        restype, argtypes = PROTOTYPES[name]
        if lib.has(name):  # bound by native.py itself
            found = len(getattr(lib.fn(name), "argtypes", None) or ()) == len(argtypes)
        else:
            try:
                function = getattr(lib.dll, name)
            except AttributeError:
                found = False
            else:
                function.restype = restype
                function.argtypes = argtypes
                lib.functions[name] = function
                found = True
        _resolved[name] = found
        return found


def _require() -> _n._Library:
    if not has(FUNCTION):
        raise NotImplementedError(
            f"the core library {_n.library_path()} does not export {FUNCTION} (the alarm and parameter tables and "
            "decoders of the driver); rebuild it with 'python build.py shared' in core/"
        )
    return _n.load()


def value(v: Any) -> Any:
    """A value as the core takes it, with its Python type: JSON for None, bool, int, float, str; ``{"float": "inf"}``
    for a float JSON cannot carry; ``{"repr": repr(v)}`` for anything else."""
    if v is None or isinstance(v, (bool, int, str)):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else {"float": "nan" if math.isnan(v) else ("inf" if v > 0 else "-inf")}
    return {"repr": repr(v)}


def call(function: str, args: Mapping[str, Any] | Iterable[Mapping[str, Any]] | None = None) -> Any:
    """``function`` of the core with ``args`` (an object, or a list of objects: a list of results). Raises the
    package's exception for the core's error (``ValueError`` for a refused value, ``NotImplementedError`` for a
    function this library lacks)."""
    lib = _require()
    if args is None:
        text = b"{}"
    elif isinstance(args, Mapping):
        text = json.dumps(args, allow_nan=False).encode("utf-8")
    else:
        text = json.dumps(list(args), allow_nan=False).encode("utf-8")
    name = function.encode("utf-8")
    size = _START
    for _ in range(3):
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        code, details = _n._invoke(lib, FUNCTION, name, text, buffer, size, ctypes.byref(required))
        if code == _n.IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
            size = required.value
            continue
        if code != _n.IAD_OK:
            raise _n._make_error(code, details or {})
        return json.loads(buffer.value.decode("utf-8"))
    raise RuntimeError(f"{FUNCTION}({function}): the output kept growing; try again")


def functions() -> list[str]:
    """The functions the loaded library offers through :func:`call`."""
    return list(call("functions"))
