"""Test hooks: the sanctioned way for tests to reach behind the public API of the driver.

Every hook here is part of the driver, so a test written with them keeps working as the implementation behind the
public API changes. The simulator's own hooks are methods of
:class:`~iai_actuator_driver.sim.SimulatedEC` (``on_request`` / ``handle``, ``word_overrides``, ``unreadable``,
``coil_state`` ...); what is not tied to one simulated unit is here:

* :func:`virtual_clock`: the bus and the axis run on one virtual clock (a wait advances it at once).
* :func:`timings`: shorter driver time limits (a home that must start, the PARW handshake).
* :func:`fake_ports`: the serial ports :func:`~iai_actuator_driver.cell.list_ports` sees, instead of the PC's.
* :func:`forget_nonvolatile_state`: what a new process starts with for the non-volatile write guard.

Nothing here opens a serial port. Not for production code.
"""

from __future__ import annotations

import contextlib
import json
import time as _time
from collections.abc import Iterable, Iterator, Mapping
from typing import Any

from . import cell as _cell
from . import ec as _ec
from . import modbus as _modbus

__all__ = ["VirtualClock", "virtual_clock", "timings", "TIMINGS", "fake_ports", "forget_nonvolatile_state"]


class VirtualClock:
    """A clock that only moves when told: :meth:`sleep` and :meth:`advance` move it at once, no real time passes.

    It stands in for the ``time`` functions of the bus and the axis inside :func:`virtual_clock`; pass
    ``clock=clock.perf_counter`` to a :class:`~iai_actuator_driver.sim.SimulatedEC` so the simulated axis moves in the
    same time. Other ``time`` functions (wall clock, formatting) are the real ones."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = float(start)

    def perf_counter(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)

    def advance(self, seconds: float) -> None:
        """Move the clock forward by ``seconds`` (an hour later: ``advance(3600)``)."""
        self.sleep(seconds)

    def __getattr__(self, name: str) -> Any:
        return getattr(_time, name)


_CLOCKED_MODULES = (_modbus, _ec)  # the bus (timeouts, gaps, retries) and the axis (polls, waits, the NV window)


@contextlib.contextmanager
def _core_clock(clock: VirtualClock) -> Iterator[None]:
    """The core's clock seam (bus timeouts, gaps and retries, the axis's polls and waits) runs on ``clock`` for the
    block."""
    from . import native

    native.use_clock(clock.perf_counter, clock.sleep)
    try:
        yield
    finally:
        native.use_real_clock()


@contextlib.contextmanager
def virtual_clock(start: float = 1000.0) -> Iterator[VirtualClock]:
    """Run the bus and the axis on one new :class:`VirtualClock` for the block: every timeout, gap, poll and wait of
    :class:`~iai_actuator_driver.modbus.ModbusRTU` and :class:`~iai_actuator_driver.ec.Elecylinder` uses it, so a
    timeout costs no real time and a run is the same on any PC. Status monitors and other threads keep real time:
    use it for single-threaded tests. Not reentrant. The core's clock seam runs on it, and the bus and axis modules'
    own ``time`` is replaced too (their waits in Python)."""
    clock = VirtualClock(start)
    saved = [(module, module.time) for module in _CLOCKED_MODULES if hasattr(module, "time")]
    try:
        for module, _ in saved:
            module.time = clock  # type: ignore[attr-defined]
        with _core_clock(clock):
            yield clock
    finally:
        for module, original in saved:
            module.time = original  # type: ignore[attr-defined]


TIMINGS: Mapping[str, str] = {
    "home_start_s": "_HOME_START_S",  # a home return must visibly start within this time
    "parw_ready_s": "PARW_READY_S",  # wait for the parameter-write handshake to be idle before a write
    "parw_clear_s": "PARW_CLEAR_S",  # the PARW handshake must end within this after the coil
}
"""The driver time limits :func:`timings` can change (name -> what it bounds)."""


@contextlib.contextmanager
def timings(**seconds: float) -> Iterator[None]:
    """Change driver time limits for the block (names in :data:`TIMINGS`), e.g. ``timings(home_start_s=0.2)``: a
    test of a limit that runs out does not wait the full production value."""
    unknown = sorted(set(seconds) - set(TIMINGS))
    if unknown:
        raise TypeError(f"unknown timing(s) {', '.join(unknown)}; choose from {', '.join(TIMINGS)}")
    saved = {name: getattr(_ec, TIMINGS[name]) for name in seconds}
    core = getattr(_ec, "_set_core_timing", None)  # the core's time limits
    try:
        for name, value in seconds.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise ValueError(f"{name} must be a number of seconds >= 0, got {value!r}")
            setattr(_ec, TIMINGS[name], float(value))
            if core is not None:
                core(name, float(value))
        yield
    finally:
        for name, value in saved.items():
            setattr(_ec, TIMINGS[name], value)
            if core is not None:
                core(name, None)


@contextlib.contextmanager
def fake_ports(ports: Iterable[Mapping[str, Any]]) -> Iterator[None]:
    """Replace the PC's serial-port enumeration for the block: :func:`~iai_actuator_driver.cell.list_ports` (and so
    ``Cell.open()``, ``scan()`` and the ``ports`` command; the core's enumeration too, so a recovery's port check)
    sees ``ports``, each a mapping with ``port`` (the name) and optionally ``vid``, ``pid``, ``description`` and
    ``manufacturer``. No port is opened. Not reentrant."""
    entries = [dict(p) for p in ports]
    for entry in entries:
        if not isinstance(entry.get("port"), str):
            raise ValueError(f"each fake port needs a 'port' name, got {entry!r}")
    saved = _cell._PORT_LIST
    _cell._PORT_LIST = entries
    try:
        with _core_ports(entries):
            yield
    finally:
        _cell._PORT_LIST = saved


_PORT_FIELDS = ("port", "description", "manufacturer", "vid", "pid")


@contextlib.contextmanager
def _core_ports(entries: list[dict[str, Any]]) -> Iterator[None]:
    """The core's port enumeration sees ``entries`` too for the block (iad_testing_set_port_list: a recovery's port
    check, the cell's port discovery); nothing with a library without it."""
    from . import native

    lib = native.load()
    if not lib.has("iad_testing_set_port_list"):
        yield
        return
    listed = [{key: entry[key] for key in _PORT_FIELDS if key in entry} for entry in entries]
    native._check(lib, "iad_testing_set_port_list", json.dumps(listed).encode("utf-8"))
    try:
        yield
    finally:
        native._invoke(lib, "iad_testing_set_port_list", None)


def forget_nonvolatile_state(transport: Any) -> None:
    """Forget what this process knows about the units reached through ``transport`` (a simulator, a line or a serial
    port, known by its name) for the non-volatile write guard: the write counter, the rate window and the parameters
    written to RAM and not yet committed. What a new process (another CLI run) starts with."""
    _ec._nv_forget(transport)
