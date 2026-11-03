"""Fast status path: a background status monitor per axis, the one-sample completion rule and a 1 ms timer.

:class:`AxisMonitor` is an optional status monitor per axis: a thread of the core (C++ ``AxisMonitor``) that reads a
*lean* status block as fast as the link allows. The block holds PNOW, ALMC, DSS1 and DSSE in the smallest contiguous
range: 0x9000-0x9007, 8 words (``01 03 90 00 00 08``; every word VERIFIED readable on the EC). Each poll is one FC03
transaction with no retries under the port lock; between two polls the lock is free and the thread pauses ``gap_s``
(2 ms), so a STOP or a command from another thread, and the other axes of a shared port (each with its own monitor or
none), get the port at once. It publishes the latest :class:`~iai_actuator_driver.ec.Status` with its timestamps
(:attr:`AxisMonitor.latest`), serves the waits (:meth:`AxisMonitor.wait_for`, :meth:`AxisMonitor.wait_update`,
:meth:`AxisMonitor.wait_done`) and queues the ``on_change`` / ``on_done`` callbacks, which run on the binding's event
thread of the axis (``iad-events-...``, below). A timeout / CRC / frame error is counted and retried after a short
back-off; a transport error (the USB adapter unplugged: the COM port is gone) or a closed bus ends the thread and every
waiter raises it. :meth:`AxisMonitor.stop` ends it cleanly (also done by ``Elecylinder.close``). A baud change, a
software reset and ``axis.recover()`` hold every monitor of the port meanwhile (:func:`suspend_bus`): on a shared line
they may change the rate under every axis.

Link watch (:mod:`iai_actuator_driver.recovery`): the monitor also tells the application when the link fails or the unit
restarted, through ``on_event`` callbacks that get a :class:`~iai_actuator_driver.recovery.RecoveryEvent`. A transport error, or
:data:`SILENT_AFTER` failed polls in a row, sets the axis's link state (``"port_vanished"`` / ``"unit_silent"``: event
``"link_lost"``). Every ``link_check_s`` (1 s) it runs the core's link check (what
:func:`iai_actuator_driver.recovery.check_link` does): one full status read, compared with the axis's session for
restart signs (uptime dropped, PC control or HEND lost: event ``"unit_restarted"``). The check only runs when the
axis has a session (:func:`iai_actuator_driver.recovery.track`, or one made without I/O from the identity
``identify()`` / ``connect()`` read). The monitor never recovers anything: call ``axis.recover()`` (VERIFIED on the
EC-GRBP8M gripper and the PCON-CB) when it reports a fault. The monitor's own link
events are UNVERIFIED on hardware (simulator only); polling, ``on_change`` and ``on_done`` are VERIFIED on the PCON-CB.

The monitor only reads. It never decides anything for the axis: :meth:`Elecylinder.move_to` uses its samples (a
status younger than :data:`FRESH_S` replaces the pre-move status read; the wait takes its samples instead of polling)
and :meth:`Elecylinder.stop` never waits for it.

:func:`fast_done` is the ``completion="fast"`` rule on one sample: PEND, ``|position - target| <= band`` and the DSSE
MOVE bit clear (not homing, not paused), confirmed once (no two-poll settle). On hardware PEND rose inside the band
while the axis was still settling (EC-GRBP8M gripper: 3.04 mm, then 3.00 mm): the fast rule accepts that sample by
design (the position is already inside the band the caller asked for); the default ``completion="safe"`` keeps the
settle rule.

:func:`timer_resolution` is a no-op, kept for code that calls it: the core itself raises the Windows timer resolution to
1 ms (``winmm.timeBeginPeriod`` / ``timeEndPeriod``, reference-counted) while a wait or a monitor runs. The default
granularity is ~15.6 ms (VERIFIED on the test PC), which every ``sleep`` of a polling loop would otherwise round up to.

What this module converts between the core and Python:

* the published state (:attr:`AxisMonitor.latest`, the counters, the errors) is read from the core when asked (no
  I/O); sample times are on ``time.perf_counter()``'s clock, and reads this close together (2 ms) see one sample;
* the waits (:meth:`~AxisMonitor.wait_update`, :meth:`~AxisMonitor.wait_for`, :meth:`~AxisMonitor.wait_done`) wait in
  short slices on the calling thread, so Ctrl+C reaches Python; ``wait_for`` evaluates its predicate in Python;
* the callbacks (``on_change``, ``on_done``, ``on_event``) run on the binding's event thread of the axis
  (``iad-events-...``), which drains the core's event queue of the axis: in the order the core queued them, a little
  later than the poll. Events are queued only for the callbacks given; :meth:`AxisMonitor.stop` delivers what was
  queued before it returns, and a monitor that ended (port gone) has delivered its events once :attr:`running` says so;
* :func:`suspend_bus` holds the core's monitors of a bus.

:func:`fast_done`, :class:`MonitorSample` and the constants are the shared ones (``_shared/monitor.py``, no I/O).
"""

from __future__ import annotations

import contextlib
import ctypes
import json
import logging
import math
import queue
import threading
import time
import weakref
from collections.abc import Callable, Iterable, Iterator
from typing import TYPE_CHECKING, Any

from . import native as _n
from ._shared.monitor import (  # the shared names (no I/O)
    BACKOFF_MAX_S,
    BACKOFF_MIN_S,
    FRESH_S,
    LEAN_STATUS_WORDS,
    LINK_CHECK_S,
    POLL_GAP_S,
    SILENT_AFTER,
    STALL_S,
    MonitorSample,
    fast_done,
)
from .errors import ModbusCRCError, ModbusError, ModbusException, ModbusFrameError, ModbusTimeout  # noqa: F401

if TYPE_CHECKING:
    from .values import Status

__all__ = [
    "AxisMonitor",
    "MonitorSample",
    "fast_done",
    "suspend_bus",
    "timer_resolution",
    "LEAN_STATUS_WORDS",
    "FRESH_S",
    "POLL_GAP_S",
    "LINK_CHECK_S",
    "SILENT_AFTER",
]

log = logging.getLogger("iai_actuator_driver.monitor")  # the public module name, on every backend

StatusCallback = Callable[..., Any]

_SLICE_S = 0.05  # longest wait of the calling thread between two looks at Ctrl+C (CC2)
_SAME_READ_S = 0.002  # reads of the published state this close together see one sample (one read, the same object)
_FLUSH_S = 0.5  # how long stop() / the end of a monitor wait for the event thread to deliver what was queued
_EVENTS_STATUS = 0x01  # IAD_EVENTS_*: the kinds the axis's queue takes
_EVENTS_DONE = 0x02
_EVENTS_LINK = 0x04
_EVENTS_CHECK = 0x08
_EVENTS_PER_CALL = _EVENTS_LINK | _EVENTS_CHECK  # always queued: per-call callbacks (check_link, recover, run_check)


# =========================================================================== 1 ms timer resolution (the core's)


class _TimerResolution:
    """The reference-counted winmm wrapper of 1.0, kept as a name: the core raises the
    Windows timer resolution itself while a wait or a monitor runs, so nothing here is ever raised."""

    @property
    def active(self) -> bool:
        return False

    @property
    def users(self) -> int:
        return 0

    def acquire(self, ms: int = 1) -> bool:
        return False

    def release(self) -> None:
        pass


TIMER = _TimerResolution()


@contextlib.contextmanager
def timer_resolution(ms: int = 1) -> Iterator[bool]:
    """A no-op (the core holds the 1 ms timer resolution while it waits or polls); yields
    ``False``."""
    yield False


# =========================================================================== helpers


def _callbacks(value: StatusCallback | Iterable[StatusCallback] | None) -> list[StatusCallback]:
    if value is None:
        return []
    if callable(value):
        return [value]
    return list(value)


def _clock_offset() -> float:
    """``time.perf_counter()`` minus the core's clock: 0 when they are one clock (the steady clock and perf_counter
    both read QueryPerformanceCounter / CLOCK_MONOTONIC), so a sample's times never jitter between two reads."""
    offset = time.perf_counter() - _n.clock_now()
    return 0.0 if abs(offset) < 1e-3 else offset


def _exception(detail: Any) -> BaseException | None:
    """The package's exception for an error the core describes as iad_last_error_json does (``None`` for none)."""
    if not isinstance(detail, dict) or not detail.get("code"):
        return None
    return _n._make_error(int(detail["code"]), detail)


_MONITORS: weakref.WeakSet[AxisMonitor] = weakref.WeakSet()  # every monitor made, for suspend_bus()
_MONITORS_LOCK = threading.Lock()


@contextlib.contextmanager
def suspend_bus(bus: Any, timeout: float = 1.0) -> Iterator[list[AxisMonitor]]:
    """Suspend every running monitor of an axis on ``bus`` inside the block (the core's suspend_bus): for a broadcast
    baud change or a recovery, which change the line's rate or reopen the port under every axis of a shared line.
    Yields the monitors held."""
    with _MONITORS_LOCK:
        mons = list(_MONITORS)
    held = [m for m in mons if getattr(m.axis, "bus", None) is bus and m.running]
    handle = getattr(bus, "_handle", None)
    if not handle:  # not a bus of the core: no monitor of the core polls it
        yield held
        return
    lib = _n.load()
    _n._check(lib, "iad_bus_suspend_monitors", handle, float(timeout), None)
    try:
        yield held
    finally:
        code, details = _n._invoke(lib, "iad_bus_resume_monitors", handle)
        if code != _n.IAD_OK:  # the bus handle went meanwhile: its holds went with it
            log.debug("monitors of the bus not resumed: %s", (details or {}).get("message"))


# =========================================================================== the axis's events


class _AxisEvents:
    """The event thread of one axis (``native._EventPump`` over iad_axis_next_event), the only reader of the core's
    event queue of the axis: status and done events go to the monitor started on the axis; link events by what
    queued them (iad_event.source): the monitor's own to its ``on_event``, those of a call (check_link, recover,
    set_link_state) to the listener of that call in progress (:func:`_link_events`; the call is known by the thread it
    runs on, iad_event.caller_thread), else nowhere (the core logged them). Holds the axis and the monitor weakly: a
    forgotten axis is closed by its finalizer, and the close ends the thread."""

    def __init__(self, axis: Any) -> None:
        self._axis = weakref.ref(axis)
        self._lock = threading.Lock()
        self._pump: Any = None
        self._handle: int | None = None
        self._monitor: weakref.ref[AxisMonitor] | None = None
        # (source, callback, call) of the calls in progress
        self._listeners: list[tuple[int, Callable[[Any], Any], _CallEvents | None]] = []

    def feed(self, monitor: AxisMonitor | None) -> None:
        self._monitor = None if monitor is None else weakref.ref(monitor)

    def monitor(self) -> AxisMonitor | None:
        ref = self._monitor
        return None if ref is None else ref()

    def on_thread(self) -> bool:
        pump = self._pump
        return pump is not None and pump._thread is threading.current_thread()

    def ensure(self, handle: int, name: str) -> Any:
        """The running event thread of ``handle`` (started first, before anything is queued for it)."""
        with self._lock:
            pump = self._pump
            if pump is None or not pump.alive or self._handle != handle:
                if pump is not None:
                    pump.stop(0.0)
                pump = _n._EventPump(_n.load(), "iad_axis_next_event", handle, self._dispatch, name).start()
                self._pump, self._handle = pump, handle
            return pump

    def flush(self, timeout: float = _FLUSH_S) -> None:
        """Wait until what is queued now has been delivered (not on the event thread itself)."""
        pump = self._pump
        if pump is not None and pump.alive and not self.on_thread():
            pump.flush(timeout)

    def release(self) -> None:
        """Deliver what is queued, then end the thread unless a call still listens."""
        self.flush()
        with self._lock:
            pump = self._pump
            if pump is None or self._listeners:
                return
            self._pump = self._handle = None
        if not self.on_thread():
            pump.stop(_FLUSH_S)
        else:
            pump._stop.set()

    def attach(
        self, handle: int, callback: Callable[[Any], Any], source: int, name: str, call: _CallEvents | None = None
    ) -> Any:
        """Hand the link events of ``source`` (IAD_EVENT_SOURCE_*; 0: every one not the monitor's) to ``callback``
        from the event thread of ``handle``, started for it when none runs (before the call queues anything). With
        ``call``, those queued on the thread the call runs on (:meth:`_CallEvents.bind`) and no other call's. Returns
        the event thread."""
        with self._lock:
            pump = self._pump
            if pump is None or not pump.alive or self._handle != handle:
                if pump is not None:
                    pump.stop(0.0)
                pump = _n._EventPump(_n.load(), "iad_axis_next_event", handle, self._dispatch, name).start()
                self._pump, self._handle = pump, handle
            self._listeners.append((source, callback, call))
            return pump

    def detach(self, callback: Callable[[Any], Any]) -> None:
        """``callback`` listens no more; the event thread ends when nothing else needs it (no call listens, no monitor
        with callbacks runs on the axis)."""
        with self._lock:
            self._listeners = [entry for entry in self._listeners if entry[1] is not callback]
            pump = self._pump
            mon = self.monitor()
            needed = bool(self._listeners) or (mon is not None and mon._wants_events() and mon._attached() is not None)
            if pump is None or needed:
                return
            self._pump = self._handle = None
        if pump.on_thread():
            pump._stop.set()
        else:
            pump.stop(_FLUSH_S)

    def _dispatch(self, e: _n.Event) -> None:
        axis = self._axis()
        if axis is None:
            return
        mon = self.monitor()
        if e.kind in (_n.IAD_EVENT_STATUS, _n.IAD_EVENT_DONE):
            if mon is not None:
                mon._deliver(int(e.kind), e.status)
        elif e.kind in (_n.IAD_EVENT_LINK, _n.IAD_EVENT_FAULT, _n.IAD_EVENT_LINK_STATE):
            event = _n._recovery_event(e)
            source = int(e.source)
            thread = int(e.caller_thread)
            with self._lock:
                listeners = [(c, call) for s, c, call in self._listeners if source != _n.IAD_EVENT_SOURCE_MONITOR
                             and (s in (0, source) or source == 0)]  # fmt: skip
            if listeners and thread:  # the thread that queued it says which call it is: that call's listener only
                for cb in [c for c, call in listeners if call is not None and call.thread == thread]:
                    cb(event)
            elif listeners:  # a core that does not say: every listener of its source
                for cb, _ in listeners:
                    cb(event)
            elif mon is not None and source in (0, _n.IAD_EVENT_SOURCE_MONITOR):  # 0: a library that does not say
                for cb in list(mon.on_event):
                    mon._call(cb, event)


_EVENTS: weakref.WeakKeyDictionary[Any, _AxisEvents] = weakref.WeakKeyDictionary()
_EVENTS_LOCK = threading.Lock()


def _events_of(axis: Any) -> _AxisEvents:
    with _EVENTS_LOCK:
        events = _EVENTS.get(axis)
        if events is None:
            events = _EVENTS[axis] = _AxisEvents(axis)
        return events


class _CallEvents:
    """The link events of one call in progress (:func:`_link_events`), those of its ``source`` only, as the axis's
    event thread routes them (iad_event.source): the only reader of the core's queue of the axis, so two calls (or a
    call and a monitor) never take each other's events."""

    def __init__(self, source: int, events: _AxisEvents) -> None:
        self._source = source
        self._events = events
        self._queue: queue.SimpleQueue[Any] = queue.SimpleQueue()
        self.thread: int | None = None  # the OS id of the thread the call runs on: its events carry it

    def bind(self) -> None:
        """Name the calling thread as the one the call runs on (call it on that thread, before the call): the link
        events the call queues are queued on it, so they are this call's and no other call's of the same source."""
        self.thread = threading.get_native_id()

    def _pump(self) -> Any:
        pump = self._events._pump
        return pump if pump is not None and pump.on_thread() else None

    def get(self, timeout: float) -> Any | None:
        """The next event of the call, ``None`` when none came within ``timeout``. On the event thread itself (a
        callback of a monitor that runs a call with ``on_event``) the events queued so far are dispatched first."""
        pump = self._pump()
        if pump is not None:
            pump._drain()
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                time.sleep(min(timeout, 0.005))
                return None
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def rest(self) -> list[Any]:
        """The events the call queued and that were not got yet (call it once the call has returned)."""
        self._events.flush()
        pump = self._pump()
        if pump is not None:
            pump._drain()
        out: list[Any] = []
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                return out


@contextlib.contextmanager
def _link_events(axis: Any, source: int = 0) -> Iterator[_CallEvents]:
    """The link events (:class:`~iai_actuator_driver.recovery.RecoveryEvent`) the core queues on ``axis``'s handle
    for a call inside the block (``source``: IAD_EVENT_SOURCE_* of that call), from the object yielded, so that the
    caller's thread hands them to the call's ``on_event`` while the call runs. They come from the axis's event thread
    (started for the block when no monitor with callbacks runs, ended after it), which routes each event by what
    queued it and on which thread: they never reach the monitor's ``on_event`` nor another call's (not even one of
    the same source), nor does a monitor's event reach the call. The call must run on a thread that has called
    :meth:`_CallEvents.bind` first. What was queued before the block (nobody listened) is not the call's."""
    handle = axis._h(False)
    events = _events_of(axis)
    call = _CallEvents(source, events)
    put = call._queue.put  # one bound method: detach() compares it by identity
    events.attach(handle, put, source, f"call-{getattr(axis, 'slave', '?')}", call)
    try:
        yield call
    finally:
        events.detach(put)


# =========================================================================== the monitor


class AxisMonitor:
    """Background status thread for one axis: the core's monitor of the axis (see the module docstring). Usually made
    by :meth:`Elecylinder.start_monitor`; read-only.

    Args:
        axis: the :class:`~iai_actuator_driver.ec.Elecylinder` to watch.
        gap_s: pause between two polls, with the port lock free (default 2 ms; 0 polls back to back).
        on_change: callable(s) ``f(status, previous)`` called when PNOW / ALMC / DSS1 / DSS2 / DSSE change.
        on_done: callable(s) ``f(status)`` called when the axis becomes done by :meth:`is_done` (a rising edge).
        retries: retries per poll (default 0: one attempt, then the lock is released).
        name: the name of this monitor (``repr`` and log lines).
        on_event: callable(s) ``f(event)`` getting each :class:`~iai_actuator_driver.recovery.RecoveryEvent` of the
            axis's link (``"link_lost"``, ``"unit_restarted"``, ...).
        link_check_s: how often the link check runs (default 1 s; ``None`` or 0: never; a lost link is still
            reported from the polls).

    Callbacks run on the binding's event thread of the axis: keep them short, never call a blocking axis command from
    them. An exception in a callback is logged and does not stop the monitor. Callbacks are added with
    :meth:`Elecylinder.start_monitor` (the events of a kind are queued only while a callback of that kind is set).
    """

    def __init__(
        self,
        axis: Any,
        *,
        gap_s: float = POLL_GAP_S,
        on_change: StatusCallback | Iterable[StatusCallback] | None = None,
        on_done: StatusCallback | Iterable[StatusCallback] | None = None,
        retries: int = 0,
        name: str | None = None,
        on_event: Callable[[Any], Any] | Iterable[Callable[[Any], Any]] | None = None,
        link_check_s: float | None = LINK_CHECK_S,
    ) -> None:
        if isinstance(gap_s, bool) or not isinstance(gap_s, (int, float)) or not 0.0 <= gap_s <= 10.0:
            raise ValueError(f"gap_s must be 0..10 s, got {gap_s!r}")
        if link_check_s is not None and (
            isinstance(link_check_s, bool) or not isinstance(link_check_s, (int, float)) or not 0 <= link_check_s <= 3600
        ):
            raise ValueError(f"link_check_s must be 0..3600 s or None, got {link_check_s!r}")
        if isinstance(retries, bool) or not isinstance(retries, int) or retries < 0:
            raise ValueError(f"retries must be >= 0, got {retries!r}")
        if not callable(getattr(axis, "_h", None)):
            raise TypeError(f"axis must be an Elecylinder of iai_actuator_driver.ec, got {type(axis).__name__}")
        self.axis = axis
        self.gap_s = float(gap_s)
        self.retries = retries
        self.on_change: list[StatusCallback] = _callbacks(on_change)
        self.on_done: list[StatusCallback] = _callbacks(on_done)
        self.on_event: list[Callable[[Any], Any]] = _callbacks(on_event)
        self.link_check_s: float | None = float(link_check_s) if link_check_s else None
        self.name = name or f"ec-monitor-{getattr(axis, 'slave', '?')}"
        self._lib = _n.load()
        self._lock = threading.Lock()
        self._handle: int | None = None  # the core axis handle while this object is its monitor
        self._sample: MonitorSample | None = None  # the last sample read (one object per seq)
        self._read_at = -math.inf
        self._kept: dict[str, Any] | None = None  # the monitor's state at stop(): read after it
        self._ended = False  # the end of this run was seen (and its events delivered)
        self._previous: Status | None = None  # the last status given to on_change
        self._errors: dict[str, BaseException] = {}  # the exception object of each error the core described
        self._failed_callbacks: set[int] = set()  # id() of callbacks that raised once (logged in full once)
        with _MONITORS_LOCK:
            _MONITORS.add(self)

    # ------------------------------------------------------------------ the core monitor

    def _attached(self) -> int | None:
        """The core axis handle this monitor runs on; ``None`` once stopped (or when the axis moved to a new one)."""
        handle = self._handle
        return handle if handle is not None and getattr(self.axis, "_handle", None) == handle else None

    def _wants_events(self) -> bool:
        return bool(self.on_change or self.on_done or self.on_event)

    def _options(self) -> _n.MonitorOptions:
        o = _n.MonitorOptions()
        _n._check(self._lib, "iad_monitor_options_init", ctypes.byref(o))
        o.gap_s = self.gap_s
        o.link_check_s = self.link_check_s or 0.0
        o.retries = self.retries
        o.events = (_EVENTS_PER_CALL | (_EVENTS_STATUS if self.on_change else 0)
                    | (_EVENTS_DONE if self.on_done else 0))  # fmt: skip
        return o

    def _read(self, handle: int) -> _n.MonitorSample | None:
        """The core's last sample and counters (no I/O); ``None`` when the axis has no monitor (any more)."""
        out = _n.MonitorSample()
        code, _ = _n._invoke(self._lib, "iad_axis_monitor_latest", handle, ctypes.byref(out))
        return out if code == _n.IAD_OK else None

    def _info(self) -> dict[str, Any] | None:
        """iad_axis_monitor_info_json while running on the core, else what was kept at stop()."""
        handle = self._attached()
        if handle is not None:
            try:
                info = json.loads(_n._read_text(self._lib, "iad_axis_monitor_info_json", handle))
            except (ValueError, ModbusError):  # the handle went meanwhile
                info = None
            if info is not None:
                return info  # type: ignore[no-any-return]
        return None if self._kept is None else self._kept.get("info")

    def _remember(self, out: _n.MonitorSample) -> MonitorSample | None:
        """The sample of ``out`` (the same object for the same seq), noted as the last one read."""
        seq = int(out.seq)
        with self._lock:
            sample = self._sample
        if seq <= 0:
            return sample
        if sample is None or sample.seq != seq:
            status = self.axis._status(out.status)
            if status is None:
                return sample
            offset = _clock_offset()
            sample = MonitorSample(status, seq, float(out.t_start_s) + offset, float(out.t_s) + offset)
        with self._lock:
            if self._sample is None or self._sample.seq <= seq:
                self._sample = sample
            self._read_at = time.perf_counter()
        return sample

    def _note_end(self) -> None:
        """The run ended (port gone, bus closed): deliver its queued events before anyone hears of the end."""
        if self._ended:
            return
        self._ended = True
        _events_of(self.axis).flush()

    def _deliver(self, kind: int, c_status: _n.Status) -> None:
        """A status / done event from the event thread."""
        st = self.axis._status(c_status)
        if st is None:
            return
        if kind == _n.IAD_EVENT_STATUS:
            previous, self._previous = self._previous, st
            for cb in list(self.on_change):
                self._call(cb, st, previous)
        else:
            for cb in list(self.on_done):
                self._call(cb, st)

    def _add(self, on_change: Any, on_done: Any, on_event: Any) -> None:
        """Callbacks added to the running monitor (:meth:`Elecylinder.start_monitor`); the queue then takes their
        events too."""
        for name, extra in (("on_change", on_change), ("on_done", on_done), ("on_event", on_event)):
            if extra is not None:
                getattr(self, name).extend([extra] if callable(extra) else list(extra))
        handle = self._attached()
        if handle is not None:
            if self._wants_events():
                _events_of(self.axis).ensure(handle, self.name)
            _n._check(self._lib, "iad_axis_start_monitor", handle, ctypes.byref(self._options()))

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> AxisMonitor:
        """Start the core's monitor of the axis (no-op while it runs). Returns ``self``."""
        if self.running:
            return self
        handle = self.axis._h()
        events = _events_of(self.axis)
        events.feed(self)
        with self._lock:
            self._handle = handle  # before ensure(): a call that ends meanwhile sees a monitor that needs the thread
        try:
            if self._wants_events():
                events.ensure(handle, self.name)  # before the start: no event of this run is taken for backlog
            _n._check(self._lib, "iad_axis_start_monitor", handle, ctypes.byref(self._options()))
        except BaseException:
            with self._lock:
                self._handle = None
            events.release()  # the thread this start made, when nothing else needs it
            raise
        with self._lock:
            self._kept = None
            self._ended = False
        return self

    def stop(self, timeout: float = 1.0) -> bool:
        """Stop the monitor and wake every waiter (they get ``None``); the callbacks get what was queued before.
        Returns True once the core's thread has ended."""
        handle = self._attached()
        ended = True
        if handle is not None:
            out = self._read(handle)
            kept: dict[str, Any] = {"info": self._info(), "polls": 0, "error_count": 0, "poll_rate_hz": None}
            if out is not None:
                self._remember(out)
                kept.update(polls=int(out.polls), error_count=int(out.error_count),
                            poll_rate_hz=_n._opt(out.poll_rate_hz))  # fmt: skip
            code, details = _n._invoke(self._lib, "iad_axis_stop_monitor", handle, float(timeout))
            if code != _n.IAD_OK:
                ended = code == _n.IAD_ERR_INVALID_ARGUMENT  # the axis was closed: its monitor went with it
                if not ended:
                    log.warning("%s: stopping the status monitor failed: %s", self.name, (details or {}).get("message"))
            with self._lock:
                self._kept, self._handle = kept, None
        events = _events_of(self.axis)
        if events.monitor() is self:
            events.release()
        return ended

    @property
    def running(self) -> bool:
        handle = self._attached()
        if handle is None:
            return False
        out = self._read(handle)
        if out is not None and out.running:
            return True
        self._note_end()
        return False

    @property
    def suspended(self) -> bool:
        """Polls are held (inside :meth:`suspend` or :func:`suspend_bus`, e.g. during a recovery or a baud change)."""
        info = self._info() if self._attached() is not None else None
        return bool(info and info.get("suspended"))

    def __enter__(self) -> AxisMonitor:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    @contextlib.contextmanager
    def suspend(self, timeout: float = 1.0) -> Iterator[None]:
        """No polls inside the block (waits up to ``timeout`` for a poll in flight): for a baud change or a software
        reset, where a poll at the wrong moment would only cost a timeout. Nests."""
        handle = self._attached()
        held = False
        if handle is not None:
            code, details = _n._invoke(self._lib, "iad_axis_suspend_monitor", handle, float(timeout))
            if code == _n.IAD_OK:
                held = True
            elif code not in (_n.IAD_ERR_NOT_READY, _n.IAD_ERR_INVALID_ARGUMENT):  # no monitor / handle any more
                raise _n._make_error(code, details or {})
        try:
            yield
        finally:
            if held:
                _n._invoke(self._lib, "iad_axis_resume_monitor", handle)

    def __repr__(self) -> str:
        state = "running" if self.running else ("failed" if self.error is not None else "stopped")
        return f"AxisMonitor({self.name!r}, {state}, polls={self.polls}, errors={self.error_count})"

    # ------------------------------------------------------------------ the published state

    @property
    def latest(self) -> MonitorSample | None:
        """The last published sample (status + timestamps), ``None`` before the first poll."""
        with self._lock:
            sample, read_at = self._sample, self._read_at
        handle = self._attached()
        if handle is None or time.perf_counter() - read_at < _SAME_READ_S:
            return sample
        out = self._read(handle)
        return sample if out is None else self._remember(out)

    @property
    def status(self) -> Status | None:
        sample = self.latest
        return None if sample is None else sample.status

    @property
    def timestamp(self) -> float | None:
        """``time.perf_counter()`` when the last status was read."""
        sample = self.latest
        return None if sample is None else sample.t

    @property
    def age_s(self) -> float | None:
        sample = self.latest
        return None if sample is None else sample.age_s

    def fresh(self, max_age_s: float = FRESH_S) -> Status | None:
        """The latest status if the monitor runs and it is younger than ``max_age_s``, else ``None``."""
        sample = self.latest
        if sample is None or not self.running or sample.age_s > max_age_s:
            return None
        return sample.status

    def _counter(self, name: str) -> Any:
        handle = self._attached()
        out = None if handle is None else self._read(handle)
        if out is not None:
            return _n._opt(out.poll_rate_hz) if name == "poll_rate_hz" else int(getattr(out, name))
        kept = self._kept or {}
        return kept.get(name, None if name == "poll_rate_hz" else 0)

    @property
    def polls(self) -> int:
        """Answered polls."""
        return self._counter("polls")  # type: ignore[no-any-return]

    @property
    def error_count(self) -> int:
        """Transient errors in total."""
        return self._counter("error_count")  # type: ignore[no-any-return]

    @property
    def poll_rate_hz(self) -> float | None:
        """Answered polls per second since the last :meth:`start`."""
        return self._counter("poll_rate_hz")  # type: ignore[no-any-return]

    @property
    def link_checks(self) -> int:
        """Link checks run (:func:`iai_actuator_driver.recovery.check_link`)."""
        info = self._info()
        return int(info.get("link_checks") or 0) if info else 0

    @property
    def started_at(self) -> float | None:
        """``time.perf_counter()`` of the last :meth:`start`."""
        info = self._info()
        started = None if not info else info.get("started_at_s")
        return None if started is None else float(started) + _clock_offset()

    def _error(self, key: str) -> BaseException | None:
        info = self._info()
        detail = None if not info else info.get(key)
        if not detail:
            return None
        text = json.dumps(detail, sort_keys=True)
        with self._lock:
            exc = self._errors.get(text)
            if exc is None:
                exc = self._errors[text] = _exception(detail)  # type: ignore[assignment]
        return exc

    @property
    def error(self) -> BaseException | None:
        """The error that ended the monitor (port gone, bus closed), ``None`` otherwise."""
        return self._error("error_detail")

    @property
    def last_error(self) -> BaseException | None:
        """The last transient error (a timeout, a CRC error ...)."""
        return self._error("last_error_detail")

    @property
    def link_state(self) -> str:
        """The axis's link state (:data:`iai_actuator_driver.recovery.LINK_STATES`: ``"ok"``, ``"unit_silent"`` ...)."""
        return _n._read_text(self._lib, "iad_axis_link_state", self.axis._h(False))

    def is_done(self, st: Status) -> bool:
        """:func:`fast_done` against the axis's last commanded target and band (when the axis knows them)."""
        target, band = ctypes.c_double(math.nan), ctypes.c_double(math.nan)
        _n._check(self._lib, "iad_axis_monitor_target", self.axis._h(False), ctypes.byref(target), ctypes.byref(band))
        return fast_done(st, _n._opt(target.value), _n._opt(band.value))

    # ------------------------------------------------------------------ waiting (in slices on the calling thread)

    def wait_update(
        self, after_seq: int | None = None, *, since: float | None = None, timeout: float | None = None
    ) -> MonitorSample | None:
        """The first sample with ``seq > after_seq`` (default: the next one) whose request went out at or after
        ``since`` (``time.perf_counter()``). ``None`` on timeout or when the monitor stops; raises when it failed."""
        if after_seq is None:
            handle = self._attached()
            out = None if handle is None else self._read(handle)
            if out is not None:
                after_seq = int(out.seq)
            else:
                sample = self._sample
                after_seq = 0 if sample is None else sample.seq
        return self._wait(None, int(after_seq), since, timeout)

    def wait_for(
        self, predicate: Callable[[Status], bool], timeout: float | None = None, *, since: float | None = None
    ) -> Status | None:
        """Block until a sample (the latest one included) satisfies ``predicate``; returns its status. ``None`` on
        timeout or when the monitor stops; raises when it failed (port gone) or the predicate raised."""
        sample = self._wait(predicate, 0, since, timeout)
        return None if sample is None else sample.status

    def wait_done(self, timeout: float | None = None, *, since: float | None = None) -> Status | None:
        """:meth:`wait_for` with :meth:`is_done`."""
        if since is not None:
            return self.wait_for(self.is_done, timeout, since=since)
        end = None if timeout is None else time.perf_counter() + max(0.0, float(timeout))
        while True:
            handle = self._attached()
            if handle is None:
                self._raise_failed()
                return None
            remaining = None if end is None else end - time.perf_counter()
            done, out = _n._c_bool(0), _n._status_out()
            code, details = _n._invoke(self._lib, "iad_axis_monitor_wait_done", handle,
                                       _SLICE_S if remaining is None else max(0.0, min(_SLICE_S, remaining)),
                                       ctypes.byref(done), ctypes.byref(out))  # fmt: skip
            if code == _n.IAD_ERR_NOT_READY:  # the axis's monitor went (stopped elsewhere)
                return None
            if code != _n.IAD_OK:
                raise _n._make_error(code, details or {})
            if done.value:
                return self.axis._status(out)  # type: ignore[no-any-return]
            if not self.running or (end is not None and time.perf_counter() >= end):
                return None

    def _accepts(self, sample: MonitorSample, after: int, predicate: Callable[[Status], bool] | None,
                 since: float | None) -> bool:  # fmt: skip
        if sample.seq <= after or (since is not None and sample.t_start < since):
            return False
        return predicate is None or bool(predicate(sample.status))

    def _wait(
        self, predicate: Callable[[Status], bool] | None, after: int, since: float | None, timeout: float | None
    ) -> MonitorSample | None:
        handle = self._attached()
        if handle is None:  # stopped: only the last sample can still match
            self._raise_failed()
            sample = self._sample
            return sample if sample is not None and self._accepts(sample, after, predicate, since) else None
        end = None if timeout is None else time.perf_counter() + max(0.0, float(timeout))
        seq = after
        out = self._read(handle)
        if out is not None and out.seq > seq:  # the latest sample first
            sample = self._remember(out)
            if sample is not None:
                if self._accepts(sample, after, predicate, since):
                    return sample
                seq = sample.seq
        while True:
            remaining = None if end is None else end - time.perf_counter()
            got, out = _n._c_bool(0), _n.MonitorSample()
            code, details = _n._invoke(self._lib, "iad_axis_monitor_wait_update", handle, seq,
                                       _SLICE_S if remaining is None else max(0.0, min(_SLICE_S, remaining)),
                                       ctypes.byref(got), ctypes.byref(out))  # fmt: skip
            if code in (_n.IAD_ERR_NOT_READY, _n.IAD_ERR_INVALID_ARGUMENT) and self._read(handle) is None:
                return None  # the axis's monitor went (stopped elsewhere, the axis closed)
            if code != _n.IAD_OK:
                raise _n._make_error(code, details or {})
            if got.value:
                sample = self._remember(out)
                if sample is not None:
                    if self._accepts(sample, after, predicate, since):
                        return sample
                    seq = sample.seq
            elif not self.running:
                return None
            if end is not None and time.perf_counter() >= end:
                return None

    def _raise_failed(self) -> None:
        error = self.error
        if error is not None:
            raise ModbusError(f"status monitor stopped: {error}") from error

    # ------------------------------------------------------------------ callbacks

    def _call(self, cb: StatusCallback, *args: Any) -> None:
        try:
            cb(*args)
        except Exception as exc:  # noqa: BLE001 - a broken callback must never stop the monitor
            if id(cb) in self._failed_callbacks:
                log.debug("%s: callback %r failed again: %s", self.name, cb, exc)
            else:
                self._failed_callbacks.add(id(cb))
                log.warning("%s: callback %r failed (logged once)", self.name, cb, exc_info=True)
