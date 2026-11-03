"""The implementation the package runs on: the shared C++ core, through its C ABI, loaded once at the first import.

The driver modules (``iai_actuator_driver.ec``, ``modbus``, ``cell`` ... :data:`MODULES`) are bindings over the core
(``native.py``), and importing the package loads the native library and checks its ABI version. When the library is
not found, does not load or has another ABI version, importing the package raises ``ImportError`` with the reason
(the text of :class:`~iai_actuator_driver.native.NativeLibraryError`, which says where it looked). A platform wheel
carries the library; the sdist and a plain install of the source folder need one: built with ``python build.py
shared`` in ``core/``, or named by ``IAI_ACTUATOR_DRIVER_NATIVE``.

``IAI_DRIVER_BACKEND``, read once at the first import: unset, ``auto`` or ``native`` (the core, the only
implementation). ``python`` chose the pure-Python edition, which was removed: it raises a one-line ``ImportError`` that
says so and names the last release with it (:data:`~iai_actuator_driver.native.PURE_PYTHON_REMOVED`). Any other value
raises ``ImportError`` naming the valid ones.

:func:`backend_info` says what was loaded and why.
"""

from __future__ import annotations

import importlib
import os
from typing import Any

ENV_VAR = "IAI_DRIVER_BACKEND"
CHOICES = ("native", "auto")
REMOVED = "python"
"""The removed choice: the pure-Python edition (1.0.x is the last release with it)."""

PACKAGE = __name__.rpartition(".")[0]  # "iai_actuator_driver"
BACKEND = "native"
"""The implementation in use: always the core (kept for code that reads ``iai_actuator_driver.BACKEND``)."""

MODULES: tuple[str, ...] = ("modbus", "sim", "sim_line", "alarms", "con_params", "info", "ec", "monitor", "gateway",
                            "cell", "coord", "recovery", "check")  # fmt: skip
"""The driver modules over the core (``backend_info()["native_modules"]``)."""


def _choice(value: str | None) -> str:
    """The ``IAI_DRIVER_BACKEND`` value (``None`` or empty: ``auto``), or ``ImportError`` on one line."""
    choice = (value or "").strip().lower() or "auto"
    if choice == REMOVED:
        native = importlib.import_module(f"{PACKAGE}.native")  # imports no driver module and loads nothing
        raise ImportError(f"{ENV_VAR}=python is no longer a choice. {native.PURE_PYTHON_REMOVED} Unset {ENV_VAR} "
                          "to run on the native core.")
    if choice not in CHOICES:
        raise ImportError(f"{ENV_VAR}={value!r} is not a backend: choose one of {', '.join(CHOICES)} or leave it unset")
    return choice


def resolve(value: str | None) -> tuple[str, str]:
    """``("native", reason)`` for an ``IAI_DRIVER_BACKEND`` value (``None`` or empty: the default) once the native
    library is loaded (``native.load()``: its search order and ABI check). Raises ``ImportError`` for the removed or an
    unknown value, and when the library is not usable."""
    choice = _choice(value)
    native = importlib.import_module(f"{PACKAGE}.native")  # imports no driver module: a failure leaves nothing behind
    try:
        lib = native.load()
        major, minor = native.abi_version()
    except Exception as exc:  # NativeLibraryError: not found, does not load, other ABI
        raise ImportError(f"{PACKAGE} needs its native library, which is not usable: {type(exc).__name__}: "
                          f"{exc}") from exc
    default = " (the default)" if not (value or "").strip() else ""
    return BACKEND, f"{choice}{default}: {lib.path} (C ABI {major}.{minor})"


def backend_info() -> dict[str, Any]:
    """The implementation in use and why: ``backend`` (``"native"``), ``requested`` (``IAI_DRIVER_BACKEND`` as read at
    import, ``None`` when unset), ``reason``, ``native_modules`` (:data:`MODULES`), ``library`` (its path), ``abi``
    (the loaded library's ``"major.minor"``, e.g. ``"1.1"``) and ``core_version``."""
    native = importlib.import_module(f"{PACKAGE}.native")
    major, minor = native.abi_version()
    return {
        "backend": BACKEND,
        "requested": _REQUESTED,
        "reason": _REASON,
        "native_modules": list(MODULES),
        "library": str(native.library_path()),
        "abi": f"{major}.{minor}",
        "core_version": native.library_version(),
    }


_REQUESTED = os.environ.get(ENV_VAR)
_, _REASON = resolve(_REQUESTED)
importlib.import_module(f"{PACKAGE}._native").install_log_bridge()
