"""Power-loss and link-loss recovery: find out what happened to an axis's link, bring it back, say what changed.

Three things take an axis away from the PC:

* ``"port_vanished"``: the USB adapter was pulled; the COM port disappears from the PC (VERIFIED) and every call on
  the open handle fails: :class:`~iai_actuator_driver.modbus.TransportError` (from the core's serial port; no pyserial
  underneath), or a silent unit whose port is no longer listed.
* ``"unit_silent"``: the port is there but the unit does not answer after the retries (24 V off, cable, or a unit
  that came back at another baud rate).
* ``"unit_restarted"``: the unit answers but restarted since :func:`track`: its uptime counter 0x9010 dropped (only a
  power-up resets it, a software reset does not: VERIFIED), a new 0x0FFF power-up record is in its alarm history
  (VERIFIED; none was written after a 3.2 s outage on the gripper, 2026-10-30), Modbus control (PMSS) dropped while the
  PC held it (0 after every power-up: VERIFIED), HEND was lost (an
  incremental unit: VERIFIED at a software reset, assumed at a power-up), or its baud rate fell back to 38400 (the
  power-up rate: VERIFIED).

Typical use::

    from iai_actuator_driver import Axis
    from iai_actuator_driver.recovery import check_link, note_target, recover, track

    ax = Axis.open("COM3"); ax.connect(); ax.set_baudrate(115200)
    track(ax)                        # remember identity, baud rate, PC control, safety speed, HEND, uptime
    ax.move_to(50.0); note_target(ax, 50.0)
    chk = check_link(ax)             # one status read: cheap enough for a monitor thread
    if not chk.ok:
        report = recover(ax, timeout=60)     # never moves the axis
        if report.needs_home:
            ax.home()                         # the application decides what to do next

:func:`recover`, in order: wait for the port to be listed again (by name), reopen it in place
(:meth:`~iai_actuator_driver.modbus.ModbusRTU.reopen`), find the unit's baud rate (38400 first when power was lost), re-identify
the unit and REQUIRE the same serial number and model (else :class:`UnitReplacedError`: nothing is written to the
other unit), restore the session (the baud rate the application had set, PC control, the safety-speed choice),
re-read the ranges, then assess: alarms (only message-level ones such as a PCON's 0x069 are reset automatically),
servo, HEND (``needs_home``), the position against the last commanded target, and on request the stored rows
(checksums) and the parameters against a cached copy. **It never starts motion**: no servo ON, no home, no move; a move
cut off by the fault is not resumed.

Events (``on_event`` callbacks get a :class:`RecoveryEvent`): ``"link_lost"``, ``"unit_restarted"``,
``"unit_replaced"``, ``"recovered"``, each once per change of the axis's link state. :class:`~iai_actuator_driver.cell.Cell`
adds ``on_fault`` (by default every other axis is stopped), ``check_links()``, ``monitor_links()`` and
``recover_all()``.

Evidence: the power-up facts and the vanishing COM port above are VERIFIED. :func:`recover` is VERIFIED on the
EC-GRBP8M gripper after a real power cycle (a fresh process, :func:`session_from_backup`) and after a line break
(found at 9600, put back on 38400 / 230400; also through ``Cell.monitor_links`` / ``recover_all``) and on the PCON-CB
after a 250 ms line break (found at 9600, put back on 38400 or 230400, 6.2 s with the row and parameter checks); a power
cycle seen live by a running process is VERIFIED on the same gripper (2026-10-30, the Python package on the native core):
the 24 V supply was off for 3.2 s; :func:`check_link` reported the link lost (``unit_silent``) and then
``unit_restarted`` (the uptime counter dropped, 6481.8 s to 0.8 s, and PMSS dropped while the PC held it);
:func:`recover` took PC control back in 1.44 s; the servo stayed on, HEND was kept, the 3 rows and 102 parameter items
were unchanged and nothing was written to non-volatile memory. No new 0x0FFF power-up record was written after that
3.2 s outage. The simulator covers the rest (:meth:`~iai_actuator_driver.sim.SimulatedEC.power_cycle`,
:meth:`~iai_actuator_driver.sim.SimulatedEC.unplug`). It adapts to the unit: on a CON unit after a power cycle (whose
software reset keeps PMSS and the uptime) and behind a gateway (RCON / ROBONET: no broadcast, so the baud rate is not
restored; identity from the 0x9400 serial code) the same steps run where the line carries them (UNVERIFIED).

On the core: the core decides and waits: the per-unit bookkeeping (session, link state, pending
target, kept per bus and slave), the restart signs, the link check, the wait for the port, the baud-rate search, the
identity rule, what is restored and how the result is assessed, and the hold of the status monitors meanwhile
(recovery::check_link / recover_axis). This module converts:

* :class:`Session` is a live view of the core's session of the unit: each field read asks the core
  (iad_axis_session_json, no I/O), and setting ``recovering``, ``uptime_at``, ``modbus_control``, ``homed``... writes
  it there (iad_axis_session_update). :func:`session_of` returns the same object while the core keeps that session;
  one made again (:func:`track`, a backup) is a new object, and the old one keeps the last values it read;
* :class:`LinkCheck`, :class:`RecoveryReport`, :class:`RecoveryEvent` and :class:`RecoveryPolicy` are the shared
  records (``_shared/recovery.py``), built from the core's results; errors are the package's exceptions with their
  ``report`` (a :class:`RecoveryReport`), ``expected`` and ``found``;
* a per-call ``on_event`` (:func:`check_link`, :func:`recover`, :func:`set_link_state`) gets the events of that call
  on the calling thread while the call runs: the call runs on a helper thread (``iad-call:<caller's name>``) and the
  events are told from a status monitor's by what queued them (iad_event.source). :meth:`Elecylinder.recover` keeps
  the axis's status monitor held until its events have been delivered;
* :func:`classify_failure` is a rule over Python exceptions; :func:`port_present` asks the core
  (iad_axis_port_present) unless a ``lister`` is given; :class:`LinkMonitor` is a Python thread over
  :func:`check_link`, and ``Cell.monitor_links`` returns a view of the core cell's own monitor.

What a Python callable cannot do in the core: a ``RecoveryPolicy.list_ports`` (and the ``lister`` of
:func:`check_link`) does not reach it, so :func:`recover` waits for the port in the core's port list
(``testing.fake_ports`` replaces it in tests); ``info=`` of :func:`track` is not passed (the core reads the identity
itself, read-only either way).
"""

from __future__ import annotations

import ctypes
import json
import logging
import threading
import time
import weakref
from collections.abc import Callable, Iterable, Mapping
from dataclasses import asdict, dataclass, field, fields, replace
from pathlib import Path
from typing import Any

from . import native as _n
from . import registers as R  # noqa: F401 (re-exported)
from ._shared.recovery import (  # the shared names (records, constants)
    _IDENTITY_KEYS,
    _LINK_ERRORS,
    BOOT_BAUDRATE,
    EVENT_KINDS,
    EVENT_LINK_LOST,
    EVENT_RECOVERED,
    EVENT_UNIT_REPLACED,
    EVENT_UNIT_RESTARTED,
    LINK_OK,
    LINK_STATES,
    PARAMETER_CLASSES,
    PORT_POLL_S,
    PORT_VANISHED,
    POSITION_TOLERANCE_MM,
    RECOVER_TIMEOUT_S,
    RECOVERING,
    UNIT_REPLACED,
    UNIT_RESTARTED,
    UNIT_SILENT,
    UPTIME_RATE_MIN,
    UPTIME_SLACK_S,
    AlarmKey,
    EventCallback,
    LinkCheck,
    RecoveryEvent,
    RecoveryPolicy,
    RecoveryReport,
    _chain,
    _load_backup,
)
from ._shared.recovery import restart_signs as _restart_signs_of_a_record
from .alarms import LEVEL_MESSAGE, alarm_level, alarm_name  # noqa: F401 (re-exported)
from .ec import Elecylinder
from .errors import EcNotReadyError as _EcNotReadyError
from .errors import (  # noqa: F401 (re-exported)
    EcDriverError,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    RecoveryError,
    TransportError,
    UnitReplacedError,
)
from .modbus import BAUD_DETECT_ORDER, VERIFIED_BAUD_RATES  # noqa: F401 (re-exported)
from .values import DeviceInfo, PositionEntry, Status
from .ec import _error, _record, _status_from_json, _status_words_of
from .monitor import _clock_offset, _link_events

__all__ = [
    "LINK_OK",
    "PORT_VANISHED",
    "UNIT_SILENT",
    "UNIT_RESTARTED",
    "UNIT_REPLACED",
    "RECOVERING",
    "LINK_STATES",
    "EVENT_LINK_LOST",
    "EVENT_UNIT_RESTARTED",
    "EVENT_UNIT_REPLACED",
    "EVENT_RECOVERED",
    "EVENT_KINDS",
    "BOOT_BAUDRATE",
    "PARAMETER_CLASSES",
    "RecoveryError",
    "UnitReplacedError",
    "RecoveryEvent",
    "Session",
    "LinkCheck",
    "RecoveryPolicy",
    "RecoveryReport",
    "LinkMonitor",
    "track",
    "session_of",
    "session_from_info",
    "session_from_backup",
    "forget",
    "note_target",
    "note_control",
    "link_state",
    "set_link_state",
    "classify_failure",
    "port_present",
    "port_name",
    "restart_signs",
    "check_link",
    "recover",
]

log = logging.getLogger("iai_actuator_driver.recovery")  # the public module name, on every backend

_SLICE_S = 0.01  # longest wait of the calling thread between two looks at Ctrl+C and at the call's events (CC3)
_NO_SESSION = ("no session for this axis: call recovery.track(axis) while the link is up (the identity to compare with "
               "is unknown)")  # fmt: skip
_CORE_TYPES: dict[str, type[Exception]] = {
    "TransportError": TransportError, "ModbusTimeout": ModbusTimeout, "ModbusCrcError": ModbusCRCError,
    "ModbusFrameError": ModbusFrameError, "ModbusError": ModbusError, "EcNotReadyError": _EcNotReadyError,
    "RecoveryError": RecoveryError,
}  # fmt: skip
"""The core's error type names (iad_link_check.error "<Type>: <message>") -> the package's classes."""


# ====================================================================== calls and their events


def _deliver(on_event: EventCallback, event: RecoveryEvent, label: str | None) -> None:
    if label is not None and event.axis != label:
        event = replace(event, axis=label)
    try:
        on_event(event)
    except Exception:  # a broken callback must never break the recovery
        log.exception("recovery event callback raised")


def _run(
    axis: Elecylinder, function: str, args: tuple[Any, ...], *, source: int, on_event: EventCallback | None,
    label: str | None = None, io: bool = True, waiting: bool = False, kept: bool = False,
) -> str | None:  # fmt: skip
    """``function(axis, *args)``. With ``on_event`` the call runs on a helper thread while this thread hands the link
    events it queues (``source``, IAD_EVENT_SOURCE_*) to ``on_event`` as they come, then the ones queued before it
    returned; Ctrl+C latches the core's cancel of the axis (the call ends) and is raised. Without one: a plain call
    (``waiting``: on the main thread on a helper, Ctrl+C as above). ``kept``: returns the result the call kept, read
    on the thread that made it (``native._invoke_kept``: a check of another thread meanwhile does not replace it)."""
    if on_event is None:
        if waiting:
            return axis._call_waiting(function, *args, kept=kept)
        if not kept:
            axis._call(function, *args, io=io)
            return None
        code, details, text = _n._invoke_kept(axis._lib, function, axis._h(io), *args)
        if code != _n.IAD_OK:
            raise _error(code, details or {})
        return text
    handle = axis._h(io)
    lib = axis._lib
    box: list[Any] = []
    invoke = _n._invoke_kept if kept else _n._invoke

    def run() -> None:  # holds ``args`` (and so the structs behind byref) until the call returns
        try:
            call.bind()  # the events the core queues for this call are queued on this thread
            box.append(invoke(lib, function, handle, *args))
        except BaseException as exc:  # handed to the waiting thread
            box.append(exc)

    with _link_events(axis, source) as call:
        helper = _n._Helper(f"iad-call:{threading.current_thread().name}", run)
        try:
            while not helper.ended:
                event = call.get(_SLICE_S)
                if event is not None:
                    _deliver(on_event, event, label)
        except KeyboardInterrupt as interrupt:
            try:
                _n._invoke(lib, "iad_axis_cancel_latch", handle)
            except Exception as exc:  # never hide the interrupt
                interrupt.add_note(f"iai_actuator_driver: cancelling {function} failed: {exc}")
            else:
                interrupt.add_note(f"iai_actuator_driver: {function} cancelled on Ctrl+C")
            helper.wait(10.0)  # the core ends the wait at its next check; a second Ctrl+C leaves at once
            if helper.ended:
                _n._invoke(lib, "iad_axis_cancel_reset", handle)
            raise
        for event in call.rest():  # the events queued before the call returned
            _deliver(on_event, event, label)
    outcome = box[0]
    if isinstance(outcome, BaseException):
        raise outcome
    if outcome[0] != _n.IAD_OK:
        raise _error(outcome[0], outcome[1] or {})
    return outcome[2] if kept else None


def _report(data: Mapping[str, Any] | None) -> RecoveryReport | None:
    """A :class:`RecoveryReport` from the core's JSON of one (RecoveryReport::to_json)."""
    if not isinstance(data, Mapping):
        return None
    names = {f.name for f in fields(RecoveryReport)} - {"status"}
    values: dict[str, Any] = {k: v for k, v in data.items() if k in names}
    values["identity"] = dict(values.get("identity") or {})
    for key in ("restart_signs", "limits_changed", "alarms_cleared", "rows_checked", "row_problems",
                "parameter_differences", "actions", "notes"):  # fmt: skip
        values[key] = list(values.get(key) or [])
    values["new_alarm_records"] = [dict(r) for r in values.get("new_alarm_records") or []]
    return RecoveryReport(**values, status=_status_from_json(data.get("status")))


def _with_report(exc: BaseException) -> BaseException:
    """A :class:`RecoveryError` from the core with its ``report`` as a :class:`RecoveryReport` (not the JSON)."""
    if isinstance(exc, RecoveryError) and isinstance(exc.report, Mapping):
        exc.report = _report(exc.report)
    return exc


def _exception_from_text(text: str) -> BaseException | None:
    """The exception a link check reports as "<Type>: <message>" (iad_link_check.error)."""
    if not text:
        return None
    kind, _, message = text.partition(": ")
    cls = _CORE_TYPES.get(kind)
    if cls is None:
        return EcDriverError(text)
    return cls(message)


def _link_check(axis: Elecylinder, c: _n.LinkCheck, label: str | None) -> LinkCheck:
    text = _n._text(c.reasons)
    reasons = () if not text else (text,) if c.reason_count <= 1 else tuple(text.split("; "))
    status = axis._status(c.status) if c.status.raw_words > 0 else None
    return LinkCheck(_n._text(c.state), reasons, status, _exception_from_text(_n._text(c.error)),
                     round(float(c.elapsed_s), 3), label)  # fmt: skip


def _link_check_from_json(data: Mapping[str, Any]) -> LinkCheck:
    """A :class:`LinkCheck` from the core's JSON of one (no status: the JSON carries only its position)."""
    return LinkCheck(str(data.get("state") or LINK_OK), tuple(str(r) for r in data.get("reasons") or ()), None,
                     _exception_from_text(str(data.get("error") or "")), float(data.get("elapsed_s") or 0.0),
                     data.get("axis"))  # fmt: skip


def _policy_of(policy: Any) -> RecoveryPolicy:
    """A RecoveryPolicy from itself or from a mapping of its fields (as the native binding takes it); a TypeError for
    anything else, or a field it does not have."""
    if isinstance(policy, RecoveryPolicy):
        return policy
    if isinstance(policy, Mapping):
        try:
            return RecoveryPolicy(**policy)
        except TypeError as exc:
            raise TypeError(f"policy: not a field of RecoveryPolicy ({exc})") from None
    raise TypeError(f"policy must be a RecoveryPolicy or a mapping of its fields, not {type(policy).__name__}")


def _policy(
    policy: RecoveryPolicy | Mapping[str, Any], require_session: bool
) -> tuple[_n.CRecoveryPolicy, str | None, str | None, str | None]:
    """iad_recovery_policy, the row list and the backup's JSON text of a policy, and the report's note when its backup
    could not be read (then none is compared)."""
    policy = _policy_of(policy)
    c = _n._recovery_policy(_n.load(), policy)
    c.require_session = int(bool(require_session))
    rows = policy.check_rows
    row_text = None if rows is False else "all" if rows is True else ",".join(str(int(n)) for n in rows)
    backup = note = None
    if policy.backup is not None:
        try:
            backup = json.dumps(_load_backup(policy.backup))
        except (OSError, ValueError) as exc:
            note = f"backup not readable: {exc}"
    return c, row_text, backup, note


# ====================================================================== session (the core's, per unit)


def _span(value: Any) -> tuple[float, float] | None:
    if isinstance(value, Mapping):
        value = (value.get("min"), value.get("max"))
    if isinstance(value, (list, tuple)) and len(value) == 2 and None not in value:
        return float(value[0]), float(value[1])
    return None


def _entry(data: Any) -> Any:
    try:
        return _record(PositionEntry, data)
    except (TypeError, ValueError):
        return data


def _session_json(axis: Elecylinder) -> dict[str, Any] | None:
    """The core's session of the axis's unit (no I/O), ``None`` when it has none."""
    try:
        return json.loads(axis._text("iad_axis_session_json", io=False))  # type: ignore[no-any-return]
    except _EcNotReadyError:
        return None


class _Live:
    """The core session a :class:`Session` object shows: that of the unit of ``axis`` made at ``taken`` (the core's
    ``taken_at``, which tells one session from the next)."""

    def __init__(self, axis: Elecylinder, taken: Any) -> None:
        self.axis = weakref.ref(axis)
        self.taken = taken

    def data(self) -> dict[str, Any] | None:
        axis = self.axis()
        if axis is None:
            return None
        data = _session_json(axis)
        return data if data is not None and data.get("taken_at") == self.taken else None

    def update(self, values: Mapping[str, Any]) -> None:
        axis = self.axis()
        if axis is None or self.data() is None:
            raise _EcNotReadyError("this session is no longer the unit's (forgotten, or tracked again)")
        axis._call("iad_axis_session_update", _n._utf8(json.dumps(values)), io=False)


_WRITABLE = {"name": "name", "baudrate": "baudrate", "modbus_control": "modbus_control",
             "safety_speed": "safety_speed", "servo_on": "servo_on", "homed": "homed", "uptime_ms": "uptime_ms",
             "uptime_at": "uptime_age_s", "last_target_mm": "last_target_mm", "recovering": "recovering"}  # fmt: skip
"""Session fields the package's code sets -> the key of iad_axis_session_update."""


@dataclass
class Session:
    """What the application had established on an axis while its link was up (:func:`track`): the unit's identity,
    the baud rate, PC control (PMSS), the safety-speed choice, servo, HEND, the uptime and alarm history used to spot
    a restart, the last commanded target, and optional copies of the stored rows / parameters for integrity checks.
    :func:`check_link` refreshes the volatile fields while the link is healthy; :func:`recover` compares against them
    and restores them. On this backend the object made by :func:`track` / :func:`session_of` shows the core's session
    of the unit: each field read asks the core, and setting ``recovering``, ``uptime_at`` (``time.perf_counter()``'s
    clock), ``modbus_control``, ``safety_speed``, ``servo_on``, ``homed``, ``baudrate``, ``uptime_ms``,
    ``last_target_mm`` or ``name`` writes it there."""

    name: str | None
    slave: int
    port: str | None
    baudrate: int | None
    serial_number: str | None = None
    model_name: str | None = None
    system_code: int | None = None
    unit_code: int | None = None
    firmware: str | None = None
    product_family: str | None = None
    soft_limits_mm: tuple[float, float] | None = None
    max_speed_mm_s: float | None = None
    units: str = "mm"
    modbus_control: bool | None = None
    safety_speed: bool | None = None
    servo_on: bool | None = None
    homed: bool | None = None
    uptime_ms: int | None = None
    uptime_at: float | None = None  # time.perf_counter() of the uptime reading
    alarm_history: tuple[AlarmKey, ...] | None = None
    last_target_mm: float | None = None
    rows: dict[int, PositionEntry] | None = None
    parameters: dict[str, str] | None = None
    backup: Mapping[str, Any] | None = field(default=None, repr=False)
    taken_at: float = field(default_factory=time.perf_counter)
    recovering: bool = False
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False, compare=False)
    _recover_lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def __getattribute__(self, name: str) -> Any:
        if name in _CORE_FIELDS:
            live = object.__getattribute__(self, "__dict__").get("_live")
            if live is not None:
                data = live.data()
                if data is not None:
                    object.__getattribute__(self, "_take")(data)
        return object.__getattribute__(self, name)

    def __setattr__(self, name: str, value: Any) -> None:
        live = self.__dict__.get("_live")
        if live is not None and name in _CORE_FIELDS:
            key = _WRITABLE.get(name)
            if key is None:
                raise AttributeError(f"Session.{name} is the core's: it is set by track() / check_link() / recover() "
                                     f"(writable here: {', '.join(_WRITABLE)})")  # fmt: skip
            if name == "uptime_at":
                core = None if value is None else time.perf_counter() - float(value)  # an age: the clocks differ
            elif name in ("modbus_control", "safety_speed", "servo_on", "homed", "recovering"):
                core = None if value is None else bool(value)
            else:
                core = value
            live.update({key: core})
        object.__setattr__(self, name, value)

    def _take(self, data: Mapping[str, Any]) -> None:
        """The fields of the core's session JSON (Session::to_json), times on ``time.perf_counter()``'s clock."""
        offset = _clock_offset()
        history = data.get("alarm_history")
        rows = data.get("rows")
        values = {key: data.get(key) for key in _CORE_FIELDS}
        values.update(
            slave=int(data.get("slave") or 1), units=str(data.get("units") or "mm"),
            soft_limits_mm=_span(data.get("soft_limits_mm")),
            uptime_at=None if data.get("uptime_at") is None else float(data["uptime_at"]) + offset,
            alarm_history=None if history is None else tuple(tuple(a) for a in history),
            rows=None if rows is None else {int(n): _entry(e) for n, e in rows.items()},
            parameters=None if data.get("parameters") is None else dict(data["parameters"]),
            taken_at=float(data.get("taken_at") or 0.0) + offset, recovering=bool(data.get("recovering")),
        )  # fmt: skip
        for key, value in values.items():
            object.__setattr__(self, key, value)
        if not data.get("has_backup"):
            object.__setattr__(self, "backup", None)

    @property
    def identity(self) -> dict[str, Any]:
        """The fields :func:`recover` requires to match (serial number, model, system / unit code) and the firmware."""
        values = self._fresh()
        return {key: values[key] for key in (*_IDENTITY_KEYS, "firmware")}

    def _fresh(self) -> dict[str, Any]:
        """The fields, read from the core once."""
        live = self.__dict__.get("_live")
        if live is not None:
            data = live.data()
            if data is not None:
                self._take(data)
        return dict(self.__dict__)

    def to_dict(self) -> dict[str, Any]:
        values = self._fresh()
        data = {f.name: values[f.name] for f in fields(self) if not f.name.startswith("_")}
        data.pop("backup", None)
        limits, history, rows = data["soft_limits_mm"], data["alarm_history"], data["rows"]
        data["soft_limits_mm"] = list(limits) if limits is not None else None
        data["alarm_history"] = None if history is None else [list(a) for a in history]
        data["rows"] = None if rows is None else {n: asdict(e) if hasattr(e, "__dataclass_fields__") else e
                                                  for n, e in rows.items()}  # fmt: skip
        data["parameters"] = None if data["parameters"] is None else dict(data["parameters"])
        data["has_backup"] = values.get("backup") is not None
        return data


_CORE_FIELDS = frozenset(f.name for f in fields(Session)) - {"backup", "_lock", "_recover_lock"}

_VIEWS: dict[tuple[int, int], Session] = {}  # (core bus handle, slave) -> the view of the unit's present session
_VIEWS_LOCK = threading.Lock()


def _unit(axis: Elecylinder) -> tuple[int, int]:
    axis._h(False)
    return int(axis._bus_handle or 0), int(axis.slave)


def port_name(axis: Elecylinder) -> str | None:
    """The name of the axis's port (``"COM3"``, a simulated line's name), ``None`` for a transport without one."""
    handle = getattr(axis.bus, "_handle", None)
    if handle:
        try:
            name = _n._read_json(axis._lib, "iad_bus_info_json", handle).get("name")
        except (ValueError, EcDriverError):  # the bus handle went: what the transport object says
            name = None
        if isinstance(name, str) and name:
            return name
    transport = axis.bus.transport
    for attr in ("port", "name"):
        value = getattr(transport, attr, None)
        if isinstance(value, str) and value:
            return value
    return None


def track(
    axis: Elecylinder,
    *,
    name: str | None = None,
    info: DeviceInfo | None = None,
    alarm_history: bool = True,
    rows: bool | Iterable[int] = False,
    parameters: bool = False,
    backup: Mapping[str, Any] | str | Path | None = None,
) -> Session:
    """Remember what the application has set up on ``axis`` while its link is up, for :func:`check_link` and
    :func:`recover` (recovery::track: identity, baud rate, PC control, safety speed, servo, HEND, uptime, alarm history
    and optionally rows / parameters). READ-ONLY (FC03 / FC11). ``rows``: ``True`` (every row) or row numbers;
    ``backup``: an :func:`iai_actuator_driver.info.backup` dict or file to compare with instead. ``name`` ``None``
    keeps the earlier session's name. ``info`` is not passed to the core: it reads the identity itself. Replaces an
    earlier session of the unit. Raises the link error when the axis does not answer."""
    o = _n.TrackOptions()
    _n._check(axis._lib, "iad_track_options_init", ctypes.byref(o))
    o.alarm_history = int(bool(alarm_history))
    o.parameters = int(bool(parameters))
    row_text = None if rows is False else "all" if rows is True else ",".join(
        str(int(n)) for n in rows)  # type: ignore[union-attr]
    data = None if backup is None else _load_backup(backup)
    axis._call_waiting("iad_axis_track", ctypes.byref(o), _n._utf8(name), _n._utf8(row_text),
                       _n._utf8(None if data is None else json.dumps(data)))  # fmt: skip
    sess = session_of(axis)
    if sess is None:  # forgotten meanwhile by another thread
        raise RecoveryError("the session was dropped while it was made")
    if data is not None:
        object.__setattr__(sess, "backup", data)
    return sess


def session_from_info(axis: Elecylinder, info: DeviceInfo, *, name: str | None = None) -> Session:
    """A session from an identity already read (e.g. ``cell.info``), without any I/O: :func:`recover` can then check
    the unit is the same one, but has no PC-control / safety-speed / uptime baseline to restore or compare (those
    steps are skipped with a note). Registered for the axis unless it already has one (that one is returned)."""
    data = {key: getattr(info, key, None) for key in ("serial_number", "model_name", "system_code", "unit_code",
                                                      "firmware", "product_family", "max_speed_mm_s")}  # fmt: skip
    data["units"] = getattr(info, "units", None) or "mm"
    limits = getattr(info, "soft_limits_mm", None)
    data["soft_limits_mm"] = None if limits is None else [float(limits[0]), float(limits[1])]
    axis._call("iad_axis_session_from_info", _n._utf8(json.dumps(data)), _n._utf8(name), io=False)
    sess = session_of(axis)
    assert sess is not None
    return sess


def session_from_backup(
    axis: Elecylinder, backup: Mapping[str, Any] | str | Path, *, name: str | None = None
) -> Session:
    """A session from an :func:`iai_actuator_driver.info.backup` dict or file, without any I/O: :func:`recover` then
    requires the unit to be the one the backup was taken from (serial number, model, system / unit code) and compares
    the stored rows / parameters with the backup when the policy asks for it. Like :func:`session_from_info` it has no
    PC-control / safety-speed / uptime baseline (state them with :func:`note_control`); the baud rate is the one in use
    now. Replaces an earlier session of the axis. Raises ``ValueError`` when the backup holds no identity."""
    data = _load_backup(backup)
    axis._call("iad_axis_session_from_backup", _n._utf8(json.dumps(data)), _n._utf8(name), io=False)
    sess = session_of(axis)
    assert sess is not None
    object.__setattr__(sess, "backup", data)
    return sess


def session_of(axis: Elecylinder) -> Session | None:
    """The axis's session (:func:`track`), or ``None``: the same object while the core keeps that session."""
    key = _unit(axis)
    data = _session_json(axis)
    with _VIEWS_LOCK:
        if data is None:
            _VIEWS.pop(key, None)
            return None
        view = _VIEWS.get(key)
        live = None if view is None else view.__dict__.get("_live")
        if live is not None and live.taken == data.get("taken_at"):
            if live.axis() is None:
                live.axis = weakref.ref(axis)
            return view
        view = Session(name=None, slave=int(axis.slave), port=None, baudrate=None)
        view._take(data)
        object.__setattr__(view, "_live", _Live(axis, data.get("taken_at")))
        _VIEWS[key] = view
        return view


def forget(axis: Elecylinder) -> None:
    """Drop the axis's session, link state and pending target."""
    axis._call("iad_axis_forget", io=False)
    with _VIEWS_LOCK:
        _VIEWS.pop(_unit(axis), None)


def note_target(axis: Elecylinder, target_mm: float | None) -> None:
    """Record the last commanded target (axis units) so :func:`recover` can compare the position with it (``None``:
    no target, e.g. after a home return). The axis records its own moves;
    :meth:`~iai_actuator_driver.cell.Cell.move_all` its targets."""
    axis._call("iad_axis_note_target", _n._num(target_mm), io=False)


def note_control(
    axis: Elecylinder,
    *,
    modbus_control: bool | None = None,
    safety_speed: bool | None = None,
    baudrate: int | None = None,
    homed: bool | None = None,
) -> None:
    """Tell the session about a deliberate change (``None``: unchanged), e.g. ``modbus_control=False`` after
    ``disable_modbus_control()``, so that :func:`check_link` does not take it for a restart and :func:`recover`
    restores the new choice. Without a session nothing happens."""

    def tri(value: bool | None) -> int:
        return _n.IAD_UNKNOWN if value is None else int(bool(value))

    axis._call("iad_axis_note_control", tri(modbus_control), tri(safety_speed), int(baudrate or 0), tri(homed),
               io=False)  # fmt: skip


def link_state(axis: Elecylinder) -> str:
    """The axis's last known link state (:data:`LINK_STATES`; ``"ok"`` when nothing was seen)."""
    return axis._text("iad_axis_link_state", io=False)


def set_link_state(
    axis: Elecylinder,
    state: str,
    *,
    message: str = "",
    on_event: EventCallback | None = None,
    name: str | None = None,
) -> bool:
    """Record the axis's link state and emit the matching event when it changed: ``"link_lost"`` on entering
    ``"port_vanished"`` / ``"unit_silent"`` from a working link, ``"unit_restarted"``, ``"unit_replaced"`` and
    ``"recovered"`` (back to ``"ok"`` from a fault). Returns True when the state changed."""
    if state not in LINK_STATES:
        raise ValueError(f"state must be one of {LINK_STATES}, got {state!r}")
    changed = ctypes.c_int32(0)
    _run(axis, "iad_axis_set_link_state", (_n._utf8(state), _n._utf8(message or None), _n._utf8(name),
                                           ctypes.byref(changed)),
         source=_n.IAD_EVENT_SOURCE_SET_LINK_STATE, on_event=on_event, io=False)  # fmt: skip
    return bool(changed.value)


def _default_name(axis: Elecylinder) -> str:
    sess = session_of(axis)
    if sess is not None and sess.name:
        return sess.name
    port = port_name(axis)
    return f"{port}:{axis.slave}" if port else f"slave {axis.slave}"


# ====================================================================== detection


def port_present(axis: Elecylinder, *, lister: Callable[[], list[dict[str, Any]]] | None = None) -> bool | None:
    """Whether the axis's port is listed by the PC right now: a transport's own ``present`` flag (the simulator), else
    its port name in the core's port list (``lister`` replaces that list). ``None`` when it cannot be told (a
    transport without a port name). READ-ONLY: no port is opened."""
    if lister is None:
        present = ctypes.c_int32(_n.IAD_UNKNOWN)
        axis._call("iad_axis_port_present", ctypes.byref(present), io=False)
        return None if present.value == _n.IAD_UNKNOWN else bool(present.value)
    transport = axis.bus.transport
    own = getattr(transport, "present", None)
    if isinstance(own, bool):
        return own
    name = getattr(transport, "port", None)
    if not isinstance(name, str) or not name:
        return None
    try:
        ports = lister()
    except Exception as exc:  # listing must never break a recovery
        log.debug("listing the ports failed: %s", exc)
        return None
    return name.upper() in {str(p.get("port", "")).upper() for p in ports}


def classify_failure(
    exc: BaseException, axis: Elecylinder | None = None, *, lister: Callable[[], list[dict[str, Any]]] | None = None
) -> str | None:
    """The link state an exception means: ``"port_vanished"`` (a :class:`~iai_actuator_driver.modbus.TransportError` /
    ``OSError`` underneath, or no answer while ``axis``'s port is no longer listed), ``"unit_silent"`` (no valid answer
    after the retries: timeout, CRC or frame errors), ``"unit_replaced"`` (:class:`UnitReplacedError`), or ``None`` for
    anything that is not a link fault (a refused command, an alarm, a soft limit, a closed bus ...)."""
    if isinstance(exc, UnitReplacedError):
        return UNIT_REPLACED
    chain = _chain(exc)
    if any(isinstance(e, TransportError) for e in chain):
        return PORT_VANISHED
    for e in chain:
        if isinstance(e, _LINK_ERRORS):
            if axis is not None and port_present(axis, lister=lister) is False:
                return PORT_VANISHED
            return UNIT_SILENT
        if isinstance(e, (ModbusError, EcDriverError)):
            continue  # a driver error of another kind: look at its cause
        if isinstance(e, OSError) and not isinstance(e, TimeoutError):
            return PORT_VANISHED  # e.g. a raw OS error from a baud-rate switch on a dead handle
    return None


def restart_signs(
    session: Session,
    status: Status,
    *,
    new_records: Iterable[AlarmKey] = (),
    baudrate_found: int | None = None,
    boot_baudrate: int = BOOT_BAUDRATE,
    now: float | None = None,
) -> list[str]:
    """The signs in ``status`` (and the alarm records / baud rate found) that the unit restarted since ``session`` was
    refreshed, in words; empty when none (recovery::restart_signs against the unit's session). ``now``:
    ``time.perf_counter()``'s clock."""
    live = session.__dict__.get("_live") if isinstance(session, Session) else None
    axis = None if live is None or live.data() is None else live.axis()
    if axis is None:  # a session that is not the core's (made by hand, forgotten): the same rule on its fields
        return _restart_signs_of_a_record(session, status, new_records=new_records, baudrate_found=baudrate_found,
                                          boot_baudrate=boot_baudrate, now=now)  # type: ignore[arg-type]
    options: dict[str, Any] = {"new_records": [list(r) for r in new_records], "baud_rate_found": baudrate_found,
                               "boot_baud_rate": int(boot_baudrate)}  # fmt: skip
    if now is not None:
        options["now_s"] = _n.clock_now() + (float(now) - time.perf_counter())
    words = _status_words_of(status)
    block = (ctypes.c_uint16 * len(words))(*words)
    text = axis._text("iad_axis_restart_signs", block, len(words), _n._utf8(json.dumps(options)), io=False)
    return [str(s) for s in json.loads(text)]


def check_link(
    axis: Elecylinder,
    *,
    session: Session | None = None,
    on_event: EventCallback | None = None,
    name: str | None = None,
    lister: Callable[[], list[dict[str, Any]]] | None = None,
) -> LinkCheck:
    """One cheap link check, safe from a monitor thread (the bus lock serialises it with the application's commands):
    one status read (22 words), compared with the unit's session (:func:`track`) for restart signs. READ-ONLY.

    Returns ``"ok"`` (the session's volatile baseline of uptime, PC control, safety speed, servo, HEND and baud rate
    is moved forward), ``"port_vanished"`` / ``"unit_silent"`` (the read failed; ``error``), ``"unit_restarted"``
    (``reasons``; the baseline is kept for :func:`recover`) or ``"recovering"`` (a :func:`recover` is running).
    Emits ``"link_lost"`` / ``"unit_restarted"`` to ``on_event`` when the state changes. Without a session only the
    link itself is checked. An error that is not a link fault is raised. ``session`` must be the unit's own (the core
    checks against it); ``lister`` does not reach the core (its port list is used)."""
    label = name or _default_name(axis)
    out = _n.LinkCheck()
    _run(axis, "iad_axis_check_link", (ctypes.byref(out),), source=_n.IAD_EVENT_SOURCE_CHECK_LINK, on_event=on_event,
         label=name)  # fmt: skip
    return _link_check(axis, out, label)


class LinkMonitor:
    """A background thread running :func:`check_link` on each axis every ``interval_s``.

    ``on_change(name, check)`` is called when an axis's state changes (the first check counts as a change only when it
    is not ``"ok"``); ``states`` holds the last :class:`LinkCheck` per axis. ``check(name, axis)`` replaces
    :func:`check_link`. The monitor never recovers or moves anything itself: call :func:`recover` (or
    ``Cell.recover_all``) from the application when it reports a fault. Use it as a context manager or call
    :meth:`start` / :meth:`stop`. (``Cell.monitor_links`` returns a view of the core cell's own monitor.)"""

    def __init__(
        self,
        axes: Mapping[str, Elecylinder] | Iterable[Elecylinder] | Elecylinder,
        *,
        interval_s: float = 1.0,
        on_change: Callable[[str, LinkCheck], Any] | None = None,
        on_event: EventCallback | None = None,
        check: Callable[[str, Elecylinder], LinkCheck] | None = None,
    ) -> None:
        if not interval_s > 0:
            raise ValueError("interval_s must be > 0")
        if isinstance(axes, Elecylinder):
            axes = [axes]
        self.axes: dict[str, Elecylinder] = (
            dict(axes) if isinstance(axes, Mapping) else {_default_name(a): a for a in axes}
        )
        self.interval_s = interval_s
        self.on_change = on_change
        self.on_event = on_event
        self._check = check if check is not None else (
            lambda name, ax: check_link(ax, name=name, on_event=self.on_event)
        )  # fmt: skip
        self.states: dict[str, LinkCheck] = {}
        self.passes = 0
        self.errors: list[str] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def check_now(self) -> dict[str, LinkCheck]:
        """One pass over every axis, in the calling thread. Returns the checks of this pass."""
        out: dict[str, LinkCheck] = {}
        for name, ax in self.axes.items():
            try:
                chk = self._check(name, ax)
            except Exception as exc:  # e.g. the bus was closed: keep monitoring the others
                self.errors.append(f"{name}: {type(exc).__name__}: {exc}")
                del self.errors[:-50]
                continue
            previous = self.states.get(name)
            self.states[name] = chk
            out[name] = chk
            if (previous.state if previous is not None else LINK_OK) != chk.state and self.on_change is not None:
                try:
                    self.on_change(name, chk)
                except Exception:
                    log.exception("link monitor callback raised")
        self.passes += 1
        return out

    def start(self) -> LinkMonitor:
        if self.running:
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ec-link-monitor", daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
        self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            self.check_now()
            self._stop.wait(self.interval_s)

    def __enter__(self) -> LinkMonitor:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()


class _CellLinkMonitor(LinkMonitor):
    """``Cell.monitor_links``: a view of the core cell's link monitor (iad_cell_monitor_links): its thread, checks and
    fault handling are the core's; ``states``, ``passes``, ``errors`` and ``running`` are read from it
    (iad_cell_link_monitor_json, no I/O), and ``on_change`` gets its state changes from the cell's event thread."""

    def __init__(self, cell: Any, axes: Mapping[str, Elecylinder], interval_s: float,
                 on_change: Callable[[str, LinkCheck], Any] | None) -> None:  # fmt: skip
        self.axes = dict(axes)
        self.interval_s = interval_s
        self.on_change = on_change
        self.on_event = None
        self._cell = weakref.ref(cell)
        self._kept: dict[str, Any] | None = None  # the core's last JSON once stopped
        self._local: dict[str, LinkCheck] | None = None  # the states once stopped and checked again (check_now)

    def _json(self) -> dict[str, Any]:
        cell = self._cell()
        if self._kept is None and cell is not None and cell._handle is not None and not cell._closed:
            data = _n._read_json(cell._lib, "iad_cell_link_monitor_json", cell._handle)
            if isinstance(data, dict) and cell._link_monitor is self:
                return data
        return self._kept or {}

    @property
    def running(self) -> bool:
        return bool(self._json().get("running"))

    @property
    def states(self) -> dict[str, LinkCheck]:  # type: ignore[override]
        if self._local is not None and self._kept is not None:  # stopped, then checked by check_now()
            return dict(self._local)
        return {str(n): _link_check_from_json(c) for n, c in (self._json().get("states") or {}).items()}

    @property
    def passes(self) -> int:  # type: ignore[override]
        return int(self._json().get("passes") or 0)

    @property
    def errors(self) -> list[str]:  # type: ignore[override]
        return [str(e) for e in self._json().get("errors") or ()]

    def check_now(self) -> dict[str, LinkCheck]:
        """One pass over every axis, in the calling thread: while the monitor runs, a pass of the core's monitor
        (iad_cell_link_monitor_check_now: its ``states`` and ``passes``, and ``on_change`` for each state change before
        this returns); once stopped, the cell's checks against the states it kept. Returns the checks of this pass."""
        cell = self._cell()
        names = list(self.axes)
        if cell is None or cell._closed or not names:
            return {}
        if self._kept is None and cell._link_monitor is self:
            checks = (_n.LinkCheck * len(names))()
            count = ctypes.c_size_t(0)
            args = (cell._h(), _n._utf8(",".join(names)), checks, len(names), ctypes.byref(count))
            code, details = _n._invoke_interruptible(cell._lib, "iad_cell_link_monitor_check_now", args, None)
            if code == _n.IAD_OK:
                out: dict[str, LinkCheck] = {}
                for i, name in enumerate(names[: count.value]):
                    if _n._text(checks[i].state) != "error":  # "error": its check threw (the monitor's errors)
                        out[name] = _link_check(self.axes[name], checks[i], name)
                cell._flush_events()  # on_change of this pass's changes, from the cell's event thread
                return out
            if code != _n.IAD_ERR_NOT_READY:  # NOT_READY: the core's monitor went (stopped by the core): as stopped
                raise _error(code, details or {})
            self._kept = dict(self._json(), running=False)
        return self._check_kept(cell, names)

    def _check_kept(self, cell: Any, names: list[str]) -> dict[str, LinkCheck]:
        """A pass of a stopped monitor: the cell's checks, compared with (and kept as) the states it showed."""
        kept = self._kept if self._kept is not None else {}
        if self._local is None:
            self._local = self.states
        errors = list(kept.get("errors") or ())
        out: dict[str, LinkCheck] = {}
        for name, chk in cell.check_links(names).items():
            if not isinstance(chk, LinkCheck):  # e.g. the bus was closed: keep monitoring the others
                errors.append(f"{name}: {type(chk).__name__}: {chk}")
                del errors[:-50]
                continue
            previous = self._local.get(name)
            self._local[name] = out[name] = chk
            if (previous.state if previous is not None else LINK_OK) != chk.state and self.on_change is not None:
                try:
                    self.on_change(name, chk)
                except Exception:
                    log.exception("link monitor callback raised")
        self._kept = dict(kept, errors=errors, passes=int(kept.get("passes") or 0) + 1, running=False)
        return out

    def start(self) -> LinkMonitor:
        cell = self._cell()
        if cell is not None and not self.running:
            _n._check(cell._lib, "iad_cell_monitor_links", cell._h(), float(self.interval_s),
                      _n._utf8(",".join(self.axes)))  # fmt: skip
            self._kept = self._local = None
            cell._link_monitor = self
        return self

    def stop(self, timeout: float = 5.0) -> None:
        cell = self._cell()
        if self._kept is None:
            self._kept = dict(self._json(), running=False)
            if cell is not None and cell._handle is not None and not cell._closed and cell._link_monitor is self:
                _n._check(cell._lib, "iad_cell_stop_link_monitor", cell._handle)

    def _changed(self, e: Any) -> None:
        """A state change the core's monitor queued (IAD_EVENT_LINK_STATE), on the cell's event thread."""
        if self.on_change is None:
            return
        message = _n._text(e.message)
        state = _n._text(e.name)
        reasons = () if not message else (message,) if state not in (LINK_OK, UNIT_RESTARTED) else tuple(
            message.split("; "))  # fmt: skip
        error = _exception_from_text(message) if state in (PORT_VANISHED, UNIT_SILENT) else None
        name = _n._text(e.axis)
        try:
            self.on_change(name, LinkCheck(state, reasons, None, error, 0.0, name))
        except Exception:
            log.exception("link monitor callback raised")


# ====================================================================== recovery


def _recover(
    axis: Elecylinder, *, timeout: float, policy: RecoveryPolicy | None, session: Session | None,
    on_event: EventCallback | None, name: str | None, require_session: bool,
) -> RecoveryReport:  # fmt: skip
    """recovery::recover_axis (``require_session``: without a session, recover()'s RecoveryError instead of the
    identity the axis read last as the baseline)."""
    policy = policy if policy is not None else RecoveryPolicy()
    if timeout < 0:
        raise ValueError("timeout must be >= 0")
    if getattr(axis.bus, "closed", False):  # the bus refuses I/O: these errors, in this order
        if require_session and session is None and session_of(axis) is None:
            raise RecoveryError(_NO_SESSION)
        raise RecoveryError("the bus is closed (close() was called): open the axis again instead")
    c_policy, row_text, backup, note = _policy(policy, require_session)
    out = _n.RecoverySummary()
    args = (float(timeout), ctypes.byref(c_policy), _n._utf8(row_text), _n._utf8(backup), _n._utf8(name),
            ctypes.byref(out))  # fmt: skip
    mon = axis.monitor if on_event is not None else None
    try:
        if mon is None:
            text = _run(axis, "iad_axis_recover", args, source=_n.IAD_EVENT_SOURCE_RECOVER, on_event=on_event,
                        waiting=True, kept=True)  # fmt: skip
        else:  # held until the call's events are delivered: on_event sees it held
            with mon.suspend():
                text = _run(axis, "iad_axis_recover", args, source=_n.IAD_EVENT_SOURCE_RECOVER, on_event=on_event,
                            kept=True)  # fmt: skip
    except RecoveryError as exc:
        _with_report(exc)
        raise
    if text is None:  # a library before 0.7: what the handle kept (a link check meanwhile may have replaced it)
        text = axis._text("iad_axis_last_result_json", io=False)
    data = json.loads(text)
    report = _report(data) if isinstance(data, Mapping) and "same_unit" in data else None
    if report is None:  # only the handle's result of an older library: another call's
        raise RecoveryError("the recovery's report was replaced on the axis handle before it was read; call it again")
    if note is not None:
        report.notes.append(note)
    return report


def recover(
    axis: Elecylinder,
    *,
    timeout: float = RECOVER_TIMEOUT_S,
    policy: RecoveryPolicy | None = None,
    session: Session | None = None,
    on_event: EventCallback | None = None,
    name: str | None = None,
) -> RecoveryReport:
    """Bring ``axis`` back after a power or link loss and report what changed (recovery::recover). **Never starts
    motion.**

    Needs the session :func:`track` recorded while the link was up (the identity to compare with). Steps: find out
    what happened (one status read); if the port vanished, wait until it is listed again (by name, every
    ``policy.poll_s``) and reopen it in place; if the unit was silent, find it again until ``timeout`` (by its baud
    rate: 38400 first after a power loss, then the session's, then the usual rates; or, without a baud rate, by a
    status read); re-identify it and require the same serial number / model / unit code (:class:`UnitReplacedError`
    otherwise, raised before anything is written); look for restart signs (:func:`restart_signs`, the alarm
    history); restore the session (baud rate, PC control, safety speed; per ``policy``); compare the ranges re-read
    by ``identify()``; assess the alarm (only a message-level one is reset), servo, HEND (``needs_home``) and the
    position against the last target; optionally check the stored rows and parameters. The session's baseline is
    then moved forward and the link state is ``"ok"`` (event ``"recovered"``). The status monitors of the port are
    held meanwhile.

    Raises :class:`RecoveryError` (with ``report``) when the port / unit does not come back within ``timeout`` or the
    identity cannot be confirmed, and :class:`UnitReplacedError` for another unit. Only one recovery runs per axis at a
    time; :func:`check_link` answers ``"recovering"`` meanwhile. ``session`` must be the unit's own.
    """
    return _recover(axis, timeout=timeout, policy=policy, session=session, on_event=on_event, name=name,
                    require_session=True)  # fmt: skip
