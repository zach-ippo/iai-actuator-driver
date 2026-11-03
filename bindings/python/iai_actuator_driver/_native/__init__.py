"""What the driver modules share over the core: the log bridge, the ABI's table calls (``_abi_tables``) and the folder
of the native library a platform wheel carries (``_native/<platform>/``, ``native.py``'s search order).

The driver modules themselves are the public names (``iai_actuator_driver.modbus`` ...): bindings over the shared C++
core. The ctypes layer is ``iai_actuator_driver.native`` (loader, ABI check, structs, result codes -> exceptions,
callback trampolines); the modules convert arguments and results, the core decides. Nothing here touches the library
at import: ``_backend`` loads it, then calls :func:`install_log_bridge`.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

from .. import native as _n

LOG_SOURCES: dict[str, str] = {
    "iai_actuator_driver.transport": "iai_actuator_driver.modbus",
    "iai_actuator_driver.axis": "iai_actuator_driver.ec",
    "iai_actuator_driver.limits": "iai_actuator_driver.ec",
    "iai_actuator_driver.device_info": "iai_actuator_driver.ec",
    "iai_actuator_driver.port": "iai_actuator_driver.cell",
}
"""Core log source -> the Python logger of the driver module that logs it (the others keep their names, which already
are the public module loggers)."""

_bridge_lock = threading.Lock()
_bridge_installed = False


def _forward(level: int, source: str, message: str) -> None:
    name = LOG_SOURCES.get(source, source) or "iai_actuator_driver"
    logger = logging.getLogger(name)
    if logger.isEnabledFor(level):
        logger.log(level, "%s", message)


PACKAGE_LOGGER = "iai_actuator_driver"
_LEVEL_PROBE = logging.getLogger(f"{PACKAGE_LOGGER}._core_log_level")
"""Its :mod:`logging` cache is emptied whenever a logger's level changes (``Logger.setLevel``, ``basicConfig``,
``dictConfig``, ``logging.disable``): then the core's level is worked out again."""
_synced_generation = -1  # native._log_sink_generation at the last look


def _wanted_level() -> int:
    """The lowest level any of the package's loggers lets through (``logging.disable`` counted)."""
    manager = logging.Logger.manager
    level = logging.getLogger(PACKAGE_LOGGER).getEffectiveLevel()
    for name, logger in list(manager.loggerDict.items()):
        if isinstance(logger, logging.Logger) and name.startswith(PACKAGE_LOGGER + "."):
            level = min(level, logger.getEffectiveLevel())
    if manager.disable:
        level = max(level, manager.disable + 1)
    return level


def _follow_logging_levels() -> None:
    """Before each call into the core (``native._invoke``), while this bridge is the core's log sink and
    ``native.set_log_level`` was never called: when a :mod:`logging` level changed since the last look, the core's
    level (iad_set_log_level) becomes the lowest level the package's loggers let through, so a line no logger would
    keep never crosses into Python (a core thread would wait for the interpreter's lock for it)."""
    global _synced_generation
    if _n._log_level_explicit or _n._log_sink_function is not _forward:
        return
    cache = getattr(_LEVEL_PROBE, "_cache", None)
    if cache is None or (cache and _synced_generation == _n._log_sink_generation):
        return  # unchanged since the last look (or a logging without the cache: every line, as before)
    with _n._log_level_lock:
        if _n._log_level_explicit or _n._log_sink_function is not _forward:
            return
        _LEVEL_PROBE.isEnabledFor(logging.DEBUG)  # fills the cache first: a change from now on is seen next time
        _synced_generation = _n._log_sink_generation
        _n.load().fn("iad_set_log_level")(_n._log_level_code(_wanted_level()))  # not through _invoke (runs this)


def install_log_bridge() -> None:
    """Send every log line of the core to the :mod:`logging` logger of its module (:data:`LOG_SOURCES`), on the thread
    that logs. The core passes on only the lines some logger of the package lets through: its level follows the
    :mod:`logging` levels (until ``native.set_log_level`` is called or another sink is installed). Once per process;
    idempotent."""
    global _bridge_installed
    with _bridge_lock:
        if _bridge_installed:
            return
        _n.set_log_sink(_forward)
        _n._log_level_check = _follow_logging_levels
        _follow_logging_levels()
        _bridge_installed = True


def library_info() -> dict[str, Any]:
    """``library`` (path), ``abi`` (``"0.3"``) and ``core_version`` of the loaded native library."""
    major, minor = _n.abi_version()
    return {"library": str(_n.library_path()), "abi": f"{major}.{minor}", "core_version": _n.library_version()}

