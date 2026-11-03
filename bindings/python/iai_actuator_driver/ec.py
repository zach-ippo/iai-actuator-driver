"""High-level axis API for IAI actuators: status, identify, servo, home, moves, push, stop, position table, parameters.

One generic axis model for every IAI unit on the bus, ELECYLINDER (EC) and CON-family controllers (PCON, ACON,
DCON, SCON ...) alike, whatever the actuator kind (slider, rod, gripper, rotary, wire cylinder ...): the same commands
everywhere; only the unit label (``mm``, or ``deg`` on a rotary axis, AKID item 7.33 = 1) and the unit's own ranges
differ. Ranges and capabilities are read from the unit itself (directory and ID maps, soft limits 7.1 / 7.2, MAXV 7.4,
MAXA 7.5, MAXD 7.6, MXPP 7.9, MNPP 7.58, AKID 7.33, MAXP 3.2, the support words), never from a table of tested
models: :meth:`Elecylinder.limits` reports them. Every product is driven the same way. The ECs tested and a PCON-CB
controller (CON family) answered the same protocol on hardware.

Evidence words used in this package: VERIFIED (confirmed on hardware), ON-WIRE (observed on the serial
line, e.g. a USB capture of IA-OS talking to the unit), DOC (public IAI manuals), UNVERIFIED (expected
behaviour, not yet confirmed on hardware). Hardware evidence comes from the EC-DS4H and EC-S6HH sliders, the
EC-GRBP8M gripper, the EC-R6S rod and a PCON-CB controller with an RCP6 actuator.

Safety defaults:
* Targets are checked against the *effective stroke* (the configured ``stroke`` intersected with
  the unit's own soft limits read from 0x0700), and speed / acceleration / deceleration / push force
  against min(the configured cap, the unit's own limit read from class 7 by item ID: MAXV 7.4, MAXA 7.5,
  MAXD 7.6, MXPP 7.9; 0x0812 is only the default speed, item 8.10, read as the fallback), *before*
  anything is written; violations raise :class:`SoftLimitError`. The configured caps default to the unit's
  own maxima (``None``). A push below the unit's recommended minimum (MNPP 7.58, or 20 %) is allowed with a
  warning, and the unit's own existing row values are always accepted below the caps. A rotary axis
  (AKID 7.33 = 1) works like any other, in degrees.
* Alarm levels (``alarms.alarm_level``): a message-level code (0x011-0x07F, e.g. a maintenance warning) is
  a *warning* (:attr:`Status.warning`) and never blocks motion; a fault (:attr:`Status.alarm`) is DSS1 ALMH
  or a code of level operation cancelled / cold start.
* Every move writes the full 9-word 0x9900 block, CTLF included, so a stale CTLF bit left by
  another tool can never turn an absolute move into a relative move or a push.
* Moves require: Modbus control, servo on, homed, no alarm / emergency stop, not paused, not homing.
  Modbus control (PMSL coil) is taken automatically, but only while the axis is at rest: switching
  PMSL on mid-move stops the move (VERIFIED on the EC-DS4H slider), so it is never toggled during motion.
* Completion is PEND (or PSFL for a push) *and* position within the band *and* PNOW unchanged over
  ``settle_s`` (two polls ~0.1 s apart): on hardware PEND and MEND rise inside the band while the axis
  is still moving (3.04 -> 3.00 mm). The DSSE MOVE bit is not trusted because it clears while the EC
  is still decelerating. A push result (contact or miss) must hold, with the position unchanged, for
  ``push_settle_s``: on the EC-GRBP8M gripper a push that missed showed the full contact fingerprint for a
  moment first.
* ``completion="fast"`` (per axis or per call) trades that settle window for latency: done on the first sample with
  PEND, the position inside the band and DSSE MOVE clear (``monitor.fast_done``); never before the position is inside
  the band. The optional status monitor (``monitor.AxisMonitor``: a background thread reading the 8-word lean block
  0x9000-0x9007 with the port lock free between polls) feeds the waits and replaces the pre-move status read when
  its sample is younger than 50 ms. Both are off by default; ``Axis.open(port, fast=True)`` turns them on with 230400
  baud. STOP never waits for the monitor.
* Every stop (:meth:`Elecylinder.stop` and the automatic stop after an error, timeout, alarm or
  Ctrl+C) uses the hardware-verified STOP coil sequence: 0x042C ON, wait until the axis has
  stopped (PNOW unchanged over ``settle_s``), 0x042C OFF. With PMSS = 0 it stopped a move commanded
  over Modbus (VERIFIED on the EC-DS4H slider); on a move a PLC commanded over PIO it is untested (a PLC holding
  its start input might restart the move once STOP is released), so stop those from the PLC or with
  the hardware stop. The IAI 0x45/0x48 strategies are never used to stop. With the ``"register"`` strategy (FC06
  writes of DRG1 0x0D00, for lines without FC05 coils such as an RCON gateway; DOC ME0162-12A p.5-124 to 5-128,
  ME0413 p.5-11; UNVERIFIED on an EC) the stop is the coil-free substitute: pause (DRG1 STP), wait until the axis
  stands still, an ALRS edge while paused (cancels the remaining travel, DOC ME0384 p.6-76, ME0162-12A p.4-11,
  5-85), then release the pause. The driver switches to it by itself when the line forbids FC05 or an FC05 write
  is answered with exception 01.
* Stopping over the serial link is not a safety function (no communication watchdog, below): an external
  emergency stop / drive cut-off is required, as for any PC-commanded axis (EN ISO 13849-1).
* IAI 0x45 commands: a rejection raises :class:`IaiCommandRejected` naming the reason
  (``registers.IAI_REJECTIONS``) and is never retried; the never-send list (NV writes, axis number,
  alarm-list clear, initialisation, ...) needs an explicit ``allow_never_send``. The 0x45 forms are
  offered as options: :meth:`Elecylinder.home` ``method="iai"`` (VERIFIED on the EC-R6S rod),
  :meth:`Elecylinder.reset_alarm` ``method="iai"`` (frame accepted on hardware with no alarm active; clearing a
  real alarm UNVERIFIED) and :meth:`Elecylinder.set_action_mode` (VERIFIED on the EC-R6S rod); the coils stay the
  default.
* Position markers (backward end / forward end / middle) come from ZONS 0x9013 b12/b13/b14, which
  means the same on every EC tested; the PIO image DOPM differs between models and is only kept raw.
* Position-table writes (non-volatile memory) need no opt-in (``allow_nonvolatile`` / ``allow_unsafe`` are accepted
  and ignored: no longer needed). A row past the unit's stored rows (0 .. MAXP 3.2 - 1, else the directory's row count;
  3 on an EC, 64 on the PCON-CB), a row the position mode does not run, a push on the 3-position middle row and unknown
  control-flag bits are written with a WARNING; the EC's simple-data rows 256-267 are never written.
* Non-volatile write guard: the units' non-volatile memory lasts about 100,000 write cycles (IAI). A parameter commit,
  a stored-row write or a teach first compares with what the unit holds and writes nothing when nothing would change
  (``skipped``; ``force=True`` writes anyway). A parameter written to RAM by this process (``commit=False``, or a
  commit that failed or was not confirmed) is committed by the next commit instead of skipped; one written RAM only by
  another program or process is not seen. Per axis (unit) this process counts its non-volatile writes over every
  axis object on it (:attr:`Elecylinder.nonvolatile_writes`) and logs one WARNING per hour while more than
  :data:`NV_WARN_PER_HOUR` fall in the last hour; it never refuses a write for the rate. Direct moves (0x9900,
  :meth:`Elecylinder.move_to`) are RAM only (VERIFIED) and cost no write cycle.
* Stored-position moves (:meth:`Elecylinder.move_to_position`, FC06 0x9800 = n, or 0x45 cmd 6 with a speed
  override) are limited to the same rows, check the row (stroke; stored speed / accel / push force against the caps unless
  ``allow_above_caps``) before anything is written, take Modbus control first (the EC silently
  ignores 0x9800 while PMSS = 0, VERIFIED on the EC-GRBP8M gripper) and raise instead of hanging when no motion
  starts. Completion uses the settle rules (a transient PEND during a push never ends the wait).
* :meth:`Elecylinder.move_to` with a deceleration different from the acceleration uses the command-profile path
  (FC17 write 0x0F00 x20 + read 0x9000 x34, then the PCRF coil 0x042E): VERIFIED on the EC-GRBP8M gripper and
  the PCON-CB (other EC models are driven the same way).
* Parameters: :meth:`Elecylinder.parameters` lists the unit's named parameters (EC user parameters P1-P18 on an EC,
  the numbered CON parameters on a CON controller, each where the unit's ID map has its item);
  :meth:`Elecylinder.read_parameter` / :meth:`Elecylinder.write_parameter` read and write any of them, or any
  class.item the unit exposes. A dangerous write (home, motor / encoder, lead / axis type, servo gains, PIO /
  fieldbus / baud configuration, widening a soft limit, raw items of unknown meaning) goes through with a warning
  (``allow_dangerous`` / ``allow_home_params`` / ``allow_unlisted`` are accepted and ignored: no longer needed).
* User-parameter writes (:meth:`Elecylinder.write_user_parameter`): only parameters confirmed on
  hardware (changed one at a time in IA-OS, registers diffed), found by class + item ID through the
  EC's directory, read-modify-write for shared items, verified by read-back, then committed with coil
  0x0428 (PARW; the frame IA-OS sends, ON-WIRE; persistence VERIFIED on the EC-R6S rod). A committed change
  takes effect only after :meth:`Elecylinder.software_reset` or a power cycle. P3 / P4 (home) are written
  with the home warning (ABS error 0x0EE / home lost). Every unit is accepted and written the same way, whatever its
  model.
* :meth:`Elecylinder.software_reset` sends the frames IA-OS sends (ON-WIRE; servo OFF with 0x45 cmd 9,
  then 0x45 cmd 5 op1 1; VERIFIED on the EC-R6S rod and the PCON-CB), never the SFRS coil 0x0420 (alarm 0x090
  with the servo on), and re-syncs the link in place. It is confirmed by PMSS dropping to 0 (EC) or by the unit
  going silent for a moment and coming back (the PCON-CB keeps PMSS 1, its baud rate and its uptime).
* :meth:`Elecylinder.set_baudrate` sends the broadcast baud change IA-OS sends (ON-WIRE; 0x45 cmd 4 to
  address 0) and switches the open port in place; the change is volatile (38400 again after power-up).

The EC has no communication watchdog: if the PC goes silent, a move in progress is completed.

On the core: :class:`Elecylinder` / :data:`Axis` is a C++ ``Axis`` (iad_axis) on the core bus of the
:class:`~iai_actuator_driver.modbus.ModbusRTU` it is given, :class:`MoveHandle` a C++ ``MoveHandle`` (iad_move_handle).
The core decides and waits: every limit check, the completion rules, the STOP sequences, the command strategies, the
Modbus control rules, the non-volatile write guard (unchanged writes skipped, the per-unit counter, one warning above
:data:`NV_WARN_PER_HOUR` writes an hour), the parameter rules, the baud change and the software reset. This module
converts arguments and results:

* statuses are the shared :class:`~iai_actuator_driver.values.Status`, decoded from the raw words the core read
  (iad_axis_last_status_words) with the core's position and velocity; the other records come from the core's JSON;
* the attributes (``soft_limits_mm``, ``ec_max_speed_mm_s``, ``axis_limits``, ``unit_code``,
  the caps ...) are read from the core's state (iad_axis_state_json, no I/O); the settings (``settle_s``,
  ``max_speed_mm_s``, ``completion`` ...) are the core axis's options;
* a call that can wait long runs, on the main thread, on a helper thread (``iad-call:MainThread``) joined in short
  slices: Ctrl+C then latches the core's cancel (a decel stop when a motion wait was running) and raises
  ``KeyboardInterrupt``; a move waits in ``iad_move_handle_poll`` slices on the calling thread.

The command strategy classes, :data:`STRATEGIES` and the constants are the shared ones (``_shared/ec.py``; names the
core's strategies carry). The status monitor (:meth:`Elecylinder.start_monitor`) is the core axis's
(``monitor.AxisMonitor``), and the link-loss calls (:meth:`~Elecylinder.track`, :meth:`~Elecylinder.check_link`,
:meth:`~Elecylinder.recover`) are those of :mod:`iai_actuator_driver.recovery` on the core axis.
"""

from __future__ import annotations

import ctypes
import json
import logging
import math
import os
import signal
import threading
import time
import weakref
from collections import deque
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import fields, replace
from typing import Any

from . import native as _n
from . import registers as R
from ._shared.ec import (  # the shared names (strategies, constants, helpers)
    ASOF_WAKE_S,
    BAUD_SETTLE_S,
    BREAK_S,
    COMPLETION_MODES,
    FALLBACK_ACCEL_CAP_G,
    FALLBACK_PUSH_CAP_PCT,
    FALLBACK_SPEED_CAP,
    FAST_BAUDRATE,
    FAST_POLL_S,
    MOVE_BAND_MAX,
    NV_WRITE_CYCLES,
    P1_MIN_MM,
    P2_RANGE_MM,
    P4_RANGE_MM,
    PARW_CLEAR_S,
    PARW_OBSERVE_S,
    PARW_READY_S,
    PUSH_CREEP_MM,
    PUSH_SETTLE_S,
    RESET_GAP_PROBE_S,
    RESET_TIMEOUT_S,
    RESET_WAIT_S,
    SERVO_FIRST_ON_MAX_S,
    SERVO_TIMEOUT_S,
    SETTLE_S,
    SMOOTH_ACCEL_MAX_TIME_S,
    STORED_START_S,
    STRATEGIES,
    TEACH_WRITE_S,
    WRITABLE_PARAMETER_CLASSES,
    CommandStrategy,
    IaiCommand2Strategy,
    IaiCommandStrategy,
    RegisterCommandStrategy,
    StandardCoilStrategy,
    _check_completion,
    _check_stroke,
    info_item_note,
)
from .alarms import alarm_level, alarm_name  # noqa: F401 (re-exported)
from .errors import (  # noqa: F401 (re-exported)
    NO_ANSWER_HINT,
    EcAlarmError,
    EcNotReadyError,
    IaiCommandRejected,
    LineRuleError,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    NonVolatileWriteError,
    ServoBlockedError,
    SoftLimitError,
    UnsafeCommandError,
    VerificationError,
)
from .gateway import UnsupportedLineError, detect_line, read_serial_code  # noqa: F401 (re-exported)
from .modbus import (  # noqa: F401 (re-exported)
    BAUD_BY_CODE,
    BAUD_DETECT_ORDER,
    BAUD_PROBE_ALL,
    DEFAULT_BAUDRATE,
    IAI_BAUD_CODES,
    RECOMMENDED_RETRIES,
    UNIT_DEPENDENT_BAUD_RATES,
    VERIFIED_BAUD_RATES,
    FrameLogger,
    ModbusRTU,
    SerialTransport,
    Transport,
)
from .monitor import (  # noqa: F401 (re-exported)
    FRESH_S,
    LEAN_STATUS_WORDS,
    STALL_S,
    AxisMonitor,
    fast_done,
    suspend_bus,
    timer_resolution,
)
from .values import (  # the shared records and notes (re-exported here)
    BAUD_VOLATILE_NOTE,
    NV_SKIP_NOTE,
    NV_WARN_PER_HOUR,
    NV_WINDOW_S,
    AlarmRecord,
    AxisLimits,
    BaudChangeResult,
    DeviceInfo,
    IndexCategory,
    IndexHeader,
    ParameterWriteResult,
    PositionEntry,
    PushResult,
    SoftwareResetResult,
    Status,
)
from .modbus import _error as _bus_error

__all__ = [
    "Elecylinder",
    "Axis",
    "Status",
    "DeviceInfo",
    "AxisLimits",
    "IndexHeader",
    "IndexCategory",
    "PositionEntry",
    "PushResult",
    "AlarmRecord",
    "ParameterWriteResult",
    "SoftwareResetResult",
    "BaudChangeResult",
    "CommandStrategy",
    "StandardCoilStrategy",
    "IaiCommandStrategy",
    "IaiCommand2Strategy",
    "RegisterCommandStrategy",
    "STRATEGIES",
    "MoveHandle",
    "AxisMonitor",
    "COMPLETION_MODES",
    "FAST_BAUDRATE",
]

log = logging.getLogger("iai_actuator_driver.ec")  # the public module name, on every backend

_NOTE_PREFIX = "iai_actuator_driver: "
_SLICE_S = 0.05  # longest wait of the calling thread between two looks at Ctrl+C (CC2 / CC3)
_STATUS_WORDS = R.STATUS_READ_WORDS  # 22: the most iad_axis_last_status_words gives
_ERROR_STOP_HOLD_S = 3.0  # the STOP on leaving a ``with`` block after an error holds at most this long
_METHODS = {"coil": 1, "iai": 2, "iai2": 3, "register": 4}  # IAD_METHOD_* (0.8: IAD_METHOD_DEFAULT is 0)
_METHOD_DEFAULT = 0
_METHOD_NAMES = {v: k for k, v in _METHODS.items()}
_COMPLETIONS = {"safe": 1, "fast": 2}  # IAD_COMPLETION_*
_ACCEL_PATTERNS = {"trapezoid": 1, "s_motion": 2, "filter": 3}  # IAD_ACCEL_*
_BAUD_METHODS = {"direct": 0, "break": 1}
_TEACH_METHODS = {"table": 0, "coil": 1}
_FLOAT_ROW_FIELDS = ("position_mm", "band_mm", "speed_mm_s", "zone_plus_mm", "zone_minus_mm", "accel_g", "decel_g")


# ====================================================================== calls and errors


def _error(code: int, details: Mapping[str, Any]) -> BaseException:
    """The package's exception for a failed call (the bus facade's mapping: the core's notes, the line's refusal); the
    core's notes on what the axis did ("decel stop sent after this error") carry the package name."""
    details = dict(details)
    notes = [str(n) for n in details.get("notes") or ()]
    details["notes"] = [n if n.startswith(_NOTE_PREFIX) or n.startswith("gave up after") else _NOTE_PREFIX + n
                        for n in notes]  # fmt: skip
    return _bus_error(code, details)


def _finite(name: str, value: Any) -> float:
    """A motion input as the core takes it; a value that is no finite number is refused
    (:class:`SoftLimitError`, nothing sent; the C ABI takes finite numbers only)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SoftLimitError(f"invalid {name} {value!r}")
    return float(value)


def _sigint_is_python() -> bool:
    """CC7: the helper thread only while Python's own SIGINT handler runs and ``IAI_DRIVER_SIGINT`` is not ``0``."""
    if os.environ.get("IAI_DRIVER_SIGINT", "").strip() == "0":
        return False
    try:
        return signal.getsignal(signal.SIGINT) is signal.default_int_handler
    except (ValueError, OSError):  # not the main interpreter
        return False


def _nan(value: float | None) -> float:
    return math.nan if value is None else float(value)


def _opt(value: float) -> float | None:
    return None if math.isnan(value) else float(value)


def _tuple(value: Any) -> Any:
    return tuple(_tuple(v) for v in value) if isinstance(value, list) else value


def _choice(value: str, choices: Mapping[str, int], what: str) -> int:
    try:
        return choices[str(value)]
    except KeyError:
        raise ValueError(f"unknown {what} {value!r}; choose from {sorted(choices)}") from None


def _status_from_json(data: Mapping[str, Any] | None) -> Status | None:
    """A :class:`Status` from the core's JSON of one (its raw words by name)."""
    if not data:
        return None
    names = {f.name for f in fields(Status)}
    values = {k: v for k, v in data.items() if k in names}
    values["raw"] = dict(data.get("raw") or {})
    return Status(**values)


def _record(cls: type, data: Mapping[str, Any], **override: Any) -> Any:
    """A shared record (``values.py``) from the core's JSON of it: the fields it has, lists as tuples."""
    names = [f.name for f in fields(cls)]
    values = {k: _tuple(data[k]) for k in names if k in data}
    values.update(override)
    return cls(**values)


def _device_info(data: Mapping[str, Any]) -> DeviceInfo:
    raw = data.get("slave_id_raw")
    header = data.get("index_header")
    return _record(
        DeviceInfo, data,
        slave_id_raw=bytes.fromhex(raw.replace(" ", "")) if isinstance(raw, str) else raw,
        index_header=_record(IndexHeader, header) if isinstance(header, Mapping) else None,
        categories=tuple(_record(IndexCategory, c) for c in data.get("categories") or ()),
        soft_limits_mm=_tuple(data.get("soft_limits_mm")),
        errors=tuple(data.get("errors") or ()),
    )


def _axis_limits(data: Mapping[str, Any] | None) -> AxisLimits | None:
    return None if not data else _record(AxisLimits, data)


# ---- the non-volatile write guard's bookkeeping per unit (the core keeps it under a key this binding chooses)

_NV_WATCHED: dict[int, Any] = {}  # id(transport) -> the finalizer that notes its collection
_NV_GONE: deque[int] = deque()  # ids of collected transports whose keys are still to be forgotten
_NV_LOCK = threading.Lock()


def _nv_drop(ident: int) -> None:
    """A transport was collected (a finalizer: it may run inside any call, even one of the library, so it only notes
    the id; :func:`_nv_drain` forgets its keys before the id can be taken again)."""
    _NV_GONE.append(ident)


def _nv_drain() -> None:
    """Forget the keys of the collected transports (with ``_NV_LOCK`` held)."""
    while _NV_GONE:
        ident = _NV_GONE.popleft()
        _NV_WATCHED.pop(ident, None)
        _n._invoke(_n.load(), "iad_nv_forget", _n._utf8(f"obj:{ident}:"))


def _nv_prefix(transport: Any) -> str | None:
    """The key prefix of the units reached through ``transport``: a serial port by its name (a new handle on the same
    port reaches the same unit), any other transport by the object itself; ``None`` for one that cannot be weakly
    referenced (it keeps a bookkeeping of its own)."""
    if isinstance(transport, SerialTransport) and isinstance(getattr(transport, "port", None), str):
        return f"port:{os.path.normcase(transport.port)}:"
    with _NV_LOCK:
        _nv_drain()  # an id of a collected transport may be this one's now
        if id(transport) not in _NV_WATCHED:
            try:
                _NV_WATCHED[id(transport)] = weakref.finalize(transport, _nv_drop, id(transport))
            except TypeError:  # not weakly referenceable
                return None
    return f"obj:{id(transport)}:"


def _nv_forget(transport: Any) -> None:
    """Drop the bookkeeping of every axis reached through ``transport`` (a serial port: by its name), as a new
    process starts without it (:func:`iai_actuator_driver.testing.forget_nonvolatile_state`)."""
    if isinstance(transport, SerialTransport) and isinstance(getattr(transport, "port", None), str):
        prefix: str | None = f"port:{os.path.normcase(transport.port)}:"
    else:
        with _NV_LOCK:
            _nv_drain()
            prefix = f"obj:{id(transport)}:" if id(transport) in _NV_WATCHED else None
    if prefix is not None:
        _n._check(_n.load(), "iad_nv_forget", _n._utf8(prefix))


# ---- testing.timings on the core (home_start_s, parw_ready_s, parw_clear_s)

_HOME_START_S = 2.0  # a home return must visibly start within this time (the core's; testing.timings)


def _set_core_timing(name: str, seconds: float | None) -> None:
    """:func:`iai_actuator_driver.testing.timings` on the core: ``name`` shortened to ``seconds`` (``None``: back to the
    default), process-wide."""
    _n._check(_n.load(), "iad_testing_set_timing", _n._utf8(name), math.nan if seconds is None else float(seconds))


def _status_words_of(st: Status) -> list[int]:
    """The 0x9000 words a :class:`Status` was decoded from (its ``raw`` words by name, in block order)."""
    raw = st.raw

    def u32(name: str) -> list[int]:
        return list(R.to_u32_words(int(raw.get(name, 0)) & 0xFFFFFFFF))

    words = [*u32("PNOW"), *(int(raw.get(n, 0)) for n in ("ALMC", "DIPM", "DOPM", "DSS1", "DSS2", "DSSE"))]
    if "STAT" in raw:
        words += [*u32("STAT"), *u32("VNOW"), *u32("CNOW")]
    if "ZONS" in raw:
        words += [*u32("DEVI"), *u32("STIM"), int(raw.get("SIPM", 0)), int(raw["ZONS"])]
    if "SSSE" in raw:
        words += [int(raw.get("POSS", 0)), int(raw["SSSE"])]
    return words


# ====================================================================== moves started without waiting


class MoveHandle:
    """A move started with ``wait=False`` (:meth:`Elecylinder.move_to`, :meth:`~Elecylinder.move_by`,
    :meth:`~Elecylinder.jog`): the command has been sent and every limit check has passed (the core's
    ``MoveHandle``). :meth:`wait` blocks until done (in short slices: Ctrl+C stops the axis, decel), :meth:`done`
    checks once, :attr:`result` is the final status once known, :meth:`cancel` is a decel stop."""

    def __init__(
        self, axis: Elecylinder, target_mm: float, band_mm: float, timeout: float, completion: str, sent_at: float
    ) -> None:
        self.axis = axis
        self.target_mm = round(target_mm, 3)
        self.band_mm = band_mm
        self.timeout = timeout
        self.completion = completion
        self.sent_at = sent_at
        self.deadline = sent_at + timeout
        self.cancelled = False
        self._result: Status | None = None
        self._handle: int | None = None
        self._finalizer: Any = None

    @classmethod
    def _started(cls, axis: Elecylinder, handle: int) -> MoveHandle:
        """The handle of a move the core just started (iad_axis_start_*)."""
        info = _n.MoveInfo()
        try:
            axis._check("iad_move_handle_result", handle, ctypes.byref(info))
        except BaseException:
            _n._close_quietly(axis._lib, "iad_move_handle_close", handle)
            raise
        completion = _n._name_of(int(info.completion), _COMPLETIONS, "safe") or "safe"
        move = cls(axis, float(info.target), float(info.band), float(info.timeout_s), completion,
                   float(info.sent_at_s))  # fmt: skip
        move.deadline = float(info.deadline_s)
        move._handle = handle
        move._finalizer = weakref.finalize(move, _n._release, axis._lib, "iad_move_handle_close", handle)
        return move

    def _h(self) -> int:
        if self._handle is None:
            raise ValueError("the move handle is closed")
        return self._handle

    def _finish(self, out: _n.Status) -> Status:
        st = self.axis._status(out)
        if st is not None:
            self._result = st
        return st  # type: ignore[return-value]

    @property
    def result(self) -> Status | None:
        if self._result is None and self._handle is not None:
            info = _n.MoveInfo()
            self.axis._check("iad_move_handle_result", self._h(), ctypes.byref(info))
            if info.has_result:
                self._result = self.axis._status(info.result)
        return self._result

    def done(self) -> bool:
        """True once the move is done (checked on one sample; ``completion="safe"`` needs the settle window over
        several calls). Raises like :meth:`wait` on an alarm, an emergency stop or the servo dropping (axis stopped)."""
        if self._result is not None:
            return True
        flag, out = _n._c_bool(0), _n._status_out()
        self.axis._check("iad_move_handle_done", self._h(), ctypes.byref(flag), ctypes.byref(out))
        if flag.value:
            self._finish(out)
            return True
        return False

    def wait(self, timeout: float | None = None) -> Status:
        """Block until done; returns the final status (on a timeout or an alarm a decel stop is sent first)."""
        if self._result is not None:
            return self._result
        out = _n._status_out()
        end = None if timeout is None else time.perf_counter() + float(timeout)
        try:
            while True:
                if end is not None and end - time.perf_counter() <= _SLICE_S:  # the last part: stop on timeout
                    self.axis._check("iad_move_handle_wait", self._h(), max(end - time.perf_counter(), 0.0),
                                     ctypes.byref(out))  # fmt: skip
                    return self._finish(out)
                flag = _n._c_bool(0)
                self.axis._check("iad_move_handle_poll", self._h(), _SLICE_S, ctypes.byref(flag), ctypes.byref(out))
                if flag.value:
                    return self._finish(out)
        except KeyboardInterrupt as interrupt:
            if getattr(interrupt, "iad_code", None) is None:  # Ctrl+C here, not a cancel from another thread
                try:
                    self.cancel()
                except Exception as exc:  # never hide the interrupt
                    interrupt.add_note(f"iai_actuator_driver: stopping the move on Ctrl+C failed: {exc}")
                else:
                    interrupt.add_note("iai_actuator_driver: decel stop sent on Ctrl+C")
            raise

    def cancel(self, *, wait: bool = False, timeout: float = 5.0) -> Status | None:
        """Decel stop (:meth:`Elecylinder.stop`); the status where it stopped becomes :attr:`result`."""
        out = _n._status_out()
        self.axis._check("iad_move_handle_cancel", self._h(), int(bool(wait)), float(timeout), ctypes.byref(out))
        self.cancelled = True
        st = self.axis._status(out)
        if self._result is None:
            self._result = st
        return st

    def to_dict(self) -> dict[str, Any]:
        result = self._result
        return {
            "target_mm": self.target_mm, "band_mm": self.band_mm, "completion": self.completion,
            "timeout_s": round(self.timeout, 3), "done": result is not None, "cancelled": self.cancelled,
            "position_mm": None if result is None else result.position_mm,
        }  # fmt: skip

    def __repr__(self) -> str:
        state = "cancelled" if self.cancelled else ("done" if self._result is not None else "running")
        return f"MoveHandle(target={self.target_mm}, band={self.band_mm}, {self.completion}, {state})"


# ====================================================================== the axis


def _close_core(lib: Any, handle: int) -> None:
    """Finalizer of a collected axis: the core axis goes, the bus stays (it is the caller's)."""
    _n._release(lib, "iad_axis_close", handle)


class Elecylinder:
    """One IAI axis on a Modbus RTU bus: an ELECYLINDER, or an actuator on a CON-family controller (PCON, ACON,
    DCON, SCON ...). :data:`Axis` is the same class under a generic name.

    Positions are in the unit's own unit (:attr:`units`: ``"mm"``, or ``"deg"`` on a rotary axis), speeds in units/s,
    accelerations in G; the ``_mm`` / ``_mm_s`` names of the arguments are kept for compatibility.

    Args:
        bus: an open :class:`ModbusRTU`.
        slave: Modbus address (axis number + 1).
        stroke: soft limits (min, max) for this application, or ``None`` to use the unit's own
            soft limits (0x0700, minus the unit's 0.30 margin). Either way the limits actually
            enforced are the intersection with the unit's limits (:attr:`effective_stroke`).
        max_speed_mm_s: optional cap for any commanded speed; ``None`` (the default) = the unit's own maximum
            speed MAXV, item 7.4. The cap actually enforced is min(this, MAXV) (:attr:`speed_cap_mm_s`; the 0x0812
            default speed, item 8.10, is only the fallback when MAXV cannot be read, and
            :data:`FALLBACK_SPEED_CAP` applies only while neither is known).
        max_accel_g: optional cap for any commanded acceleration / deceleration; ``None`` = the unit's MAXA /
            MAXD (items 7.5 / 7.6; 0.30 G on the gripper) (:attr:`accel_cap_g`).
        strategy: ``"coil"`` (FC05, default), ``"iai"`` (0x45: servo cmd 9 and home cmd 0x0A with op1 =
            ORG, VERIFIED on the EC-R6S rod; alarm reset cmd 5 op1 0, frame accepted with no alarm active, clearing
            a real alarm UNVERIFIED), the EXPERIMENTAL ``"iai2"`` (0x48, UNVERIFIED) or ``"register"`` (FC06 writes of
            DRG1, no coils: servo / home / alarm reset / pause / safety speed, and the coil-free STOP substitute; DOC
            ME0162-12A p.5-124 to 5-128, ME0413 p.5-11; UNVERIFIED on an EC) for servo / home / alarm reset, or a
            :class:`CommandStrategy`. :meth:`home` / :meth:`reset_alarm` also take a per-call ``method``. The
            coil strategy switches to ``"register"`` by itself when the line forbids FC05 (``bus.line_rules``) or an
            FC05 write is answered with exception 01; an IAI strategy on a line without 0x45 / 0x48 falls back to
            ``"coil"`` (or ``"register"``).
        poll_interval_s: status polling period while waiting.
        max_push_pct: optional cap for the push force (1..100 %); ``None`` = the unit's MXPP (item 7.9: the
            hard cap; 60 % on the slider, 70 % on the rod) (:attr:`push_cap_pct`). A force below the unit's
            recommended minimum MNPP (or 20 %) is allowed with a warning.
        auto_modbus_control: take Modbus control (PMSL coil) automatically before a command when
            the EC reports PMSS = 0 and the axis is at rest (the default). With ``False`` such
            commands raise :class:`EcNotReadyError` until :meth:`enable_modbus_control` is called.
            PMSL is never written while the axis moves (switching it on mid-move stops the move).
        settle_s: a move, home return or STOP counts as done only once PNOW has stayed unchanged
            this long (PEND / MEND can rise before standstill).
        push_settle_s: a push outcome (contact or miss) must hold this long, position unchanged
            (within ``PUSH_CREEP_MM`` for a contact), before it is reported (see :meth:`push`).
        completion: when a move counts as done: ``"safe"`` (default: PEND, in the band and PNOW unchanged over
            ``settle_s``) or ``"fast"`` (one sample: PEND, in the band, DSSE MOVE clear; :func:`monitor.fast_done`).
            ``move_to`` / ``move_by`` / ``jog`` / ``wait_idle`` / ``move_sequence`` take it per call too.

    Fast path (all optional): :meth:`start_monitor` (a background status thread; a fresh sample replaces the pre-move
    status read and feeds the waits), ``completion="fast"``, ``wait=False`` returning a :class:`MoveHandle`,
    :meth:`move_sequence` with ``blend_band``, and ``Axis.open(port, fast=True)`` / ``connect(fast=True)`` for all of
    them plus 230400 baud (:meth:`enable_fast`; :meth:`disable_fast` turns it off).

    Power / link loss (:mod:`iai_actuator_driver.recovery`): :meth:`track` remembers the session, :meth:`check_link` checks the
    link in one status read, :meth:`recover` brings the axis back after a power or link loss and reports what changed
    (never moving it). The status monitor runs the link check itself and reports ``"link_lost"`` /
    ``"unit_restarted"`` to its ``on_event`` callbacks.

    On the core: the core's ``Axis``.
    """

    def __init__(
        self,
        bus: ModbusRTU,
        slave: int = 1,
        stroke: tuple[float, float] | None = None,
        max_speed_mm_s: float | None = None,
        max_accel_g: float | None = None,
        *,
        strategy: CommandStrategy | str = "coil",
        poll_interval_s: float = 0.02,
        owns_bus: bool = False,
        max_push_pct: float | None = None,
        auto_modbus_control: bool = True,
        settle_s: float = SETTLE_S,
        push_settle_s: float = PUSH_SETTLE_S,
        completion: str = "safe",
    ) -> None:
        _check_completion(completion)
        if max_speed_mm_s is not None and not 0 < max_speed_mm_s <= 100_000:
            raise ValueError("max_speed_mm_s must be > 0 (or None: the unit's own maximum)")
        if max_accel_g is not None and not 0 < max_accel_g <= R.ACMD_MAX_RAW * R.ACCEL_SCALE:
            raise ValueError("max_accel_g must be in (0, 3.0] (or None: the unit's own maximum)")
        if max_push_pct is not None and not 0.0 < max_push_pct <= 100.0:
            raise ValueError("max_push_pct must be in (0, 100] (or None: the unit's own maximum)")
        if isinstance(strategy, str):
            try:
                strategy = STRATEGIES[strategy]()
            except KeyError:
                raise ValueError(f"unknown strategy {strategy!r}; choose from {sorted(STRATEGIES)}") from None
        if getattr(bus, "_handle", None) is None:
            raise TypeError(f"bus must be a ModbusRTU of iai_actuator_driver.modbus, got {type(bus).__name__}")
        self._lib = _n.load()
        self._bus = bus
        self.slave = slave
        self.owns_bus = owns_bus
        self.baud_detected = False  # open() found the baud rate by probing
        self.device_info: DeviceInfo | None = None  # the last identify() result
        self._strategy = strategy
        self._options: dict[str, Any] = {
            "stroke": None if stroke is None else _check_stroke(stroke), "max_speed_mm_s": max_speed_mm_s,
            "max_accel_g": max_accel_g, "max_push_pct": max_push_pct, "poll_interval_s": float(poll_interval_s),
            "settle_s": float(settle_s), "method": _METHODS.get(getattr(strategy, "name", "coil"), _METHODS["coil"]),
            "auto_modbus_control": bool(auto_modbus_control), "push_settle_s": float(push_settle_s),
            "completion": completion,
        }  # fmt: skip
        self._handle: int | None = None
        self._bus_handle: int | None = None
        self._finalizer: Any = None
        self._state_lock = threading.Lock()
        self._monitor: AxisMonitor | None = None  # the status monitor, once started
        self._open_core()

    # ------------------------------------------------------------------ the core axis

    def _axis_options(self) -> _n.AxisOptions:
        o = _n.AxisOptions()
        _n._check(self._lib, "iad_axis_options_init", ctypes.byref(o))
        opts = self._options
        stroke = opts["stroke"]
        o.stroke_min, o.stroke_max = (math.nan, math.nan) if stroke is None else stroke
        o.max_speed_mm_s = _nan(opts["max_speed_mm_s"])
        o.max_accel_g = _nan(opts["max_accel_g"])
        o.max_push_pct = _nan(opts["max_push_pct"])
        o.poll_interval_s = opts["poll_interval_s"]
        o.settle_s = opts["settle_s"]
        o.method = opts["method"]
        o.auto_modbus_control = int(opts["auto_modbus_control"])
        o.push_settle_s = opts["push_settle_s"] if opts["push_settle_s"] > 0 else -1.0  # negative: no settling
        o.completion = _COMPLETIONS[opts["completion"]]
        return o

    def _open_core(self) -> None:
        """The core axis on the bus's present core bus (a new one after the bus moved to another transport)."""
        bus_handle = self._bus._handle
        o = self._axis_options()
        out = ctypes.c_void_p()
        adopted = self.__dict__.pop("_adopt_handle", None)  # (py-cell) a cell's own axis (iad_cell_axis_ref)
        if adopted is not None:
            out.value = adopted
            _n._check(self._lib, "iad_axis_set_options", adopted, ctypes.byref(o))
        else:
            _n._check(self._lib, "iad_axis_open_bus", bus_handle, int(self.slave), ctypes.byref(o), ctypes.byref(out))
        old, self._handle, self._bus_handle = self._finalizer, int(out.value or 0), bus_handle
        self._finalizer = weakref.finalize(self, _close_core, self._lib, self._handle)
        if old is not None:
            old()
        if o.method == _METHODS["coil"]:  # (py-gateway) "coil" given or by default is this axis's own:
            _n._check(self._lib, "iad_axis_set_strategy", self._handle, o.method)  # a core axis without a
            # strategy takes its line's (open() and Port.axis pass the line's); a line without coils switches it later
        # the non-volatile write guard's bookkeeping is the unit's: every axis object of this process on it shares it
        transport = getattr(self._bus, "transport", None)
        prefix = _nv_prefix(transport)
        port = getattr(transport, "port", None)
        label = f"{port} slave {self.slave}" if isinstance(port, str) else f"slave {self.slave}"
        _n._check(self._lib, "iad_axis_set_nv_unit", self._handle,
                  None if prefix is None else _n._utf8(f"{prefix}{self.slave}"), _n._utf8(label))  # fmt: skip

    def _h(self, io: bool = True) -> int:
        """The core axis handle. ``io``: the call talks to the unit (refused once the bus is closed; the core
        axis follows the bus to a new transport); otherwise the call only reads what
        the axis keeps, which stays readable after :meth:`close`."""
        bus = self._bus
        if io and getattr(bus, "closed", False):
            raise ModbusError("bus is closed")
        if io and getattr(bus, "_adapter", None) is not None:  # a Python transport's timeout changed meanwhile
            bus._sync_timeout()
        if self._handle is None or (io and bus._handle != self._bus_handle):
            if getattr(bus, "closed", False):
                raise ModbusError("bus is closed")
            with self._state_lock:
                if self._handle is None or bus._handle != self._bus_handle:
                    self._open_core()
        return self._handle  # type: ignore[return-value]

    def _check(self, name: str, *args: Any) -> None:
        code, details = _n._invoke(self._lib, name, *args)
        if code != _n.IAD_OK:
            raise _error(code, details or {})

    def _call(self, name: str, *args: Any, io: bool = True) -> None:
        self._check(name, self._h(io), *args)

    def _call_waiting(self, name: str, *args: Any, kept: bool = False) -> str | None:
        """A call that may wait long. On the main thread it runs on a helper thread (``iad-call:MainThread``) joined
        in slices, so Ctrl+C reaches Python: the core's cancel latch then ends the wait (a decel stop when motion was
        waited for), the call returns and ``KeyboardInterrupt`` is raised (CC3). Elsewhere it runs directly.
        ``kept``: returns the result the call kept (``native._invoke_kept``, read on the thread that made it)."""
        handle = self._h()
        lib = self._lib
        caller = threading.current_thread()
        invoke = _n._invoke_kept if kept else _n._invoke
        if caller is not threading.main_thread() or not _sigint_is_python():
            outcome = invoke(lib, name, handle, *args)
            if outcome[0] != _n.IAD_OK:
                raise _error(outcome[0], outcome[1] or {})
            return outcome[2] if kept else None
        box: list[Any] = []

        def run() -> None:  # holds ``args`` (and so the structs behind byref) until the call returns
            try:
                box.append(invoke(lib, name, handle, *args))
            except BaseException as exc:  # handed to the waiting thread
                box.append(exc)

        helper = _n._Helper(f"iad-call:{caller.name}", run)
        try:
            while not helper.wait(_SLICE_S):
                pass
        except KeyboardInterrupt as interrupt:
            try:
                _n._invoke(lib, "iad_axis_cancel_latch", handle)
            except Exception as exc:  # never hide the interrupt
                interrupt.add_note(f"iai_actuator_driver: cancelling {name} failed: {exc}")
            else:
                interrupt.add_note("iai_actuator_driver: wait cancelled on Ctrl+C (a decel stop was sent)")
            helper.wait(10.0)  # the core ends the wait right after the stop; a second Ctrl+C leaves at once
            if helper.ended:
                _n._invoke(lib, "iad_axis_cancel_reset", handle)
            raise
        outcome = box[0]
        if isinstance(outcome, BaseException):
            raise outcome
        if outcome[0] != _n.IAD_OK:
            raise _error(outcome[0], outcome[1] or {})
        return outcome[2] if kept else None

    def _json(self, name: str, *args: Any, io: bool = True) -> Any:
        return json.loads(self._text(name, *args, io=io))

    def _text(self, name: str, *args: Any, io: bool = True) -> str:
        handle = self._h(io)
        size = 64 * 1024
        for _ in range(3):
            buffer = ctypes.create_string_buffer(size)
            required = ctypes.c_size_t(0)
            code, details = _n._invoke(self._lib, name, handle, *args, buffer, size, ctypes.byref(required))
            if code == _n.IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
                size = required.value + max(4096, required.value // 4)  # a fresh read (unit info) may come out longer
                continue
            if code != _n.IAD_OK:
                raise _error(code, details or {})
            return buffer.value.decode("utf-8", errors="replace")
        raise RuntimeError(f"{name}: the output kept growing; try again")

    def _state(self) -> dict[str, Any]:
        """What the core axis knows without I/O (iad_axis_state_json)."""
        return self._json("iad_axis_state_json", io=False)  # type: ignore[no-any-return]

    def _last_result(self) -> dict[str, Any]:
        return self._json("iad_axis_last_result_json", io=False)  # type: ignore[no-any-return]

    def _kept_result(self, text: str | None) -> dict[str, Any]:
        """The result a call kept: its own (``text``, read on the thread that made the call: a link check on another
        thread replaces the handle's), else the handle's (a core without ``iad_last_result_json``)."""
        return self._last_result() if text is None else json.loads(text)  # type: ignore[no-any-return]

    def _status(self, out: _n.Status) -> Status | None:
        """The shared :class:`Status` of an iad_status the core returned: decoded from the raw words of that sample
        (iad_axis_last_status_words), with the core's position (an RCON unit counting 0.001 mm), velocity and line;
        ``None`` when the call wrote none."""
        if out.raw_words <= 0 or math.isnan(out.position_mm):
            return None
        words = (ctypes.c_uint16 * _STATUS_WORDS)()
        count = ctypes.c_size_t(0)
        code, _ = _n._invoke(self._lib, "iad_axis_last_status_words", self._h(False), ctypes.byref(out), words,
                             _STATUS_WORDS, ctypes.byref(count))  # fmt: skip
        if code == _n.IAD_OK and count.value >= R.STATUS_BLOCK_MIN_WORDS:
            st = Status.from_registers(list(words[: count.value]))
        else:  # a sample no longer kept: the 8 words the struct carries
            pnow = R.to_u32_words(int(round(out.position_mm / R.POSITION_SCALE)) & 0xFFFFFFFF)
            st = Status.from_registers([*pnow, out.alarm_code & 0xFFFF, out.raw_dipm, out.raw_dopm, out.raw_dss1,
                                        out.raw_dss2, out.raw_dsse])  # fmt: skip
        velocity = _opt(out.velocity_mm_s)
        line = self.line_name
        return replace(st, position_mm=float(out.position_mm), velocity_mm_s=None if velocity is None
                       else round(velocity, 2), line=line)  # fmt: skip

    def _status_call(self, name: str, *args: Any, waiting: bool = False) -> Status | None:
        out = _n._status_out()
        (self._call_waiting if waiting else self._call)(name, *args, ctypes.byref(out))
        return self._status(out)

    # ------------------------------------------------------------------ construction

    @classmethod
    def open(
        cls,
        port: str | Transport,
        *,
        baudrate: int | None = None,
        auto_baud: bool | None = None,
        timeout: float = 0.2,
        retries: int = RECOMMENDED_RETRIES,
        frame_logger: FrameLogger | None = None,
        line: str = "auto",
        position_unit_mm: float | None = None,
        fast: bool = False,
        **kwargs: Any,
    ) -> Elecylinder:
        """Open a bus on ``port`` (name or Transport) and return an Elecylinder that owns it.

        ``fast=True`` then runs :meth:`enable_fast`: 230400 baud (a verified rate; volatile, 38400 again after a power
        cycle), ``completion="fast"`` and the status monitor on.

        The port is opened once. With ``auto_baud`` (the default when no ``baudrate`` is given) the
        baud rate the EC answers at is found by probing in place: ``baudrate`` (default 115200)
        first, then 38400 (the EC's power-up rate, VERIFIED), 9600, 230400, 57600, 19200.
        The rate is changed on the open port; it is never closed and reopened. A transport without
        a baud rate (e.g. the simulator) is used as it is. Raises :class:`ModbusTimeout` (port
        closed again) when no rate answers.

        ``retries`` defaults to 3, the manufacturer's recommended count (DOC ME0162-12A p.4-2).

        The line is worked out once the baud rate is settled (:func:`~iai_actuator_driver.gateway.detect_line`: two read-only
        probes, about 0.1 s on a plain line). ``line`` is ``"auto"`` (default), or states it: ``"direct"`` /
        ``"robonet"`` send nothing, ``"rcon"`` waits for the gateway's first contact. Behind an RCON gateway the axis
        takes the ``"register"`` strategy (FC06 DRG1, STOP without a coil), identifies itself by the 0x9400 serial
        code instead of FC11, treats a silent 0x0000 directory as "no directory" and needs ``position_unit_mm``
        (0.01 or 0.001, parameter No.207) before any mm-based command (DOC ME0413, ME0384; UNVERIFIED on hardware).
        A ROBONET gateway in gateway mode gives no axis access: :class:`~iai_actuator_driver.gateway.UnsupportedLineError`.
        """
        if auto_baud is None:
            auto_baud = baudrate is None
        start = DEFAULT_BAUDRATE if baudrate is None else baudrate
        bus = ModbusRTU(port, start, timeout, retries, frame_logger=frame_logger)
        try:
            detected = False
            if auto_baud and bus.can_change_baudrate:
                try:
                    bus.detect_baudrate(kwargs.get("slave", 1), BAUD_DETECT_ORDER)
                except ModbusTimeout as exc:
                    exc.add_note(f"iai_actuator_driver: {NO_ANSWER_HINT}")
                    raise
                detected = True
            found = detect_line(bus, line, position_unit_mm=position_unit_mm)
            if not found.rules.axis_access:
                raise UnsupportedLineError(f"{port if isinstance(port, str) else 'transport'}: {found.explain()}")
            wanted = found.command_strategy
            if "strategy" not in kwargs and wanted in STRATEGIES:
                kwargs["strategy"] = wanted
            ec = cls(bus, owns_bus=True, **kwargs)
        except BaseException:
            bus.close()
            raise
        ec.baud_detected = detected
        if fast:
            try:
                ec.enable_fast()
            except BaseException:
                ec.close()
                raise
        return ec

    @property
    def bus(self) -> ModbusRTU:
        """The :class:`ModbusRTU` this axis talks through (the object passed in)."""
        return self._bus

    @bus.setter
    def bus(self, value: ModbusRTU) -> None:
        if getattr(value, "_handle", None) is None:
            raise TypeError(f"bus must be a ModbusRTU of iai_actuator_driver.modbus, got {type(value).__name__}")
        with self._state_lock:
            self._bus = value
            self._options.update(self._core_options())
            self._open_core()

    @property
    def baudrate(self) -> int | None:
        """The serial baud rate in use (``None`` for a transport without one, e.g. the simulator)."""
        return self.bus.baudrate

    def connect(self, *, take_control: bool = True, fast: bool = False) -> DeviceInfo:
        """Start a session: identity, directory, soft limits and speed limit, then Modbus control when
        ``take_control`` and the axis is at rest (never while it moves). ``fast=True`` then :meth:`enable_fast`."""
        identity = _n.Identity()
        self._call_waiting("iad_axis_connect", int(bool(take_control)), ctypes.byref(identity))
        info = self._device_info()
        if fast:
            self.enable_fast()
        return info

    def _device_info(self) -> DeviceInfo:
        info = _device_info(self._json("iad_axis_identity_json"))
        self.device_info = info
        return info

    def enable_fast(self, *, baudrate: int | None = FAST_BAUDRATE, monitor: bool = True) -> dict[str, Any]:
        """The fast path in one call: ``completion="fast"``, the status monitor on and the link raised to
        ``baudrate`` (the core's enable_fast; the reasons a part was not done are in ``notes``). The monitor is
        :meth:`start_monitor` (the one already started is kept, or restarted with its callbacks)."""
        out = _n.FastSummary()
        self._call("iad_axis_set_port_owner", int(bool(self.owns_bus)))  # the baud change needs the port to itself
        self._call_waiting("iad_axis_enable_fast", 0 if baudrate is None else int(baudrate), 0, ctypes.byref(out))
        if monitor:
            self.start_monitor()
        return self._fast_summary(self._live_monitor() is not None)

    def disable_fast(self) -> dict[str, Any]:
        """Undo :meth:`enable_fast`: ``completion="safe"`` again and the status monitor stopped. The baud rate is
        kept."""
        self.stop_monitor()
        out = _n.FastSummary()
        self._call("iad_axis_disable_fast", ctypes.byref(out))
        return self._fast_summary(bool(out.monitor))

    def _fast_summary(self, monitor: bool) -> dict[str, Any]:
        state = self._state()
        notes = [str(n) for n in state.get("fast_notes") or ()]
        return {"baudrate": self.bus.baudrate, "completion": self.completion, "monitor": monitor,
                "notes": notes}  # fmt: skip

    @property
    def fast_mode(self) -> bool:
        """:meth:`enable_fast` ran (and no :meth:`disable_fast` since)."""
        return bool(self._state().get("fast_mode"))

    @property
    def fast_notes(self) -> list[str]:
        """What :meth:`enable_fast` could not do (the baud rate kept ...)."""
        return [str(n) for n in self._state().get("fast_notes") or ()]

    # ------------------------------------------------------------------ status monitor

    @property
    def monitor(self) -> AxisMonitor | None:
        """The status monitor (:meth:`start_monitor`), ``None`` when none was started."""
        return self._monitor

    def start_monitor(self, **kwargs: Any) -> AxisMonitor:
        """Start the background status monitor (:class:`~iai_actuator_driver.monitor.AxisMonitor`, the core axis's;
        ``kwargs``: ``gap_s``, ``on_change``, ``on_done``, ``on_event``, ``link_check_s``, ``retries``, ``name``).
        Already running: the callbacks given are added and the running monitor is returned; a monitor that ended on
        an error (port gone) is restarted with its callbacks when no ``kwargs`` are given. Read-only; stopped by
        :meth:`stop_monitor` and :meth:`close`."""
        mon = self._live_monitor()
        if mon is not None:
            mon._add(kwargs.get("on_change"), kwargs.get("on_done"), kwargs.get("on_event"))
            return mon
        if self._monitor is not None and not kwargs:
            return self._monitor.start()
        mon = AxisMonitor(self, **kwargs)
        self._monitor = mon
        return mon.start()

    def stop_monitor(self, timeout: float = 1.0) -> None:
        """Stop the status monitor (no-op without one)."""
        mon, self._monitor = self._monitor, None
        if mon is not None:
            mon.stop(timeout)
        elif self._handle is not None:  # one the core started without this object (none on this backend so far)
            _n._invoke(self._lib, "iad_axis_stop_monitor", self._handle, float(timeout))

    def _live_monitor(self) -> AxisMonitor | None:
        mon = self._monitor
        return mon if mon is not None and mon.running else None

    # ------------------------------------------------------------------ state (read-only)

    @property
    def status_words(self) -> int:
        """Words :meth:`status` reads at 0x9000: 22 (..SSSE), 20 (..ZONS) or 8 (..DSSE)."""
        return int(self._state()["status_words"])

    @property
    def motion_pending(self) -> bool:
        """A motion command (move, push, home, jog) was sent and has not been seen to finish."""
        return bool(self._state()["motion_pending"])

    @property
    def stop_held(self) -> bool:
        """The STOP this axis wrote ON (:meth:`begin_stop`, a stop that could not be released) is still held."""
        return bool(self._state()["stop_held"])

    @property
    def directory_absent(self) -> bool:
        """The directory read timed out on a line where that means the units have none (RCON / ROBONET axes)."""
        return bool(self._state()["directory_absent"])

    @property
    def soft_limits_mm(self) -> tuple[float, float] | None:
        """The unit's own soft limits (min, max) as stored, once read."""
        return _tuple(self._state()["soft_limits_mm"])  # type: ignore[no-any-return]

    @property
    def soft_limits_note(self) -> str:
        """Where the soft limits were read from, or why not."""
        return str(self._state()["soft_limits_note"])

    @property
    def ec_max_speed_mm_s(self) -> float | None:
        """MAXV (item 7.4), else the 0x0812 default speed (item 8.10), once read."""
        return self._state()["ec_max_speed_mm_s"]  # type: ignore[no-any-return]

    @property
    def ec_max_speed_note(self) -> str:
        """Where :attr:`ec_max_speed_mm_s` came from, or why it is not known."""
        return str(self._state()["ec_max_speed_note"])

    @property
    def axis_limits(self) -> AxisLimits | None:
        """The unit's class-7 limits (:meth:`read_axis_limits`), once read. Settable: every later check uses the limits
        given until the next read."""
        return _axis_limits(self._state()["axis_limits"])

    @axis_limits.setter
    def axis_limits(self, value: AxisLimits) -> None:
        data = {f.name: getattr(value, f.name) for f in fields(AxisLimits)}
        self._call("iad_axis_set_axis_limits_json", _n._utf8(json.dumps(data)), io=False)

    @property
    def system_code(self) -> int | None:
        """FC11 / CTCL system code once read."""
        return self._state()["system_code"]  # type: ignore[no-any-return]

    @property
    def unit_code(self) -> int | None:
        """FC11 unit code once read (identify() or a parameter command)."""
        return self._state()["unit_code"]  # type: ignore[no-any-return]

    @property
    def model_name(self) -> str | None:
        """The 0x0600 model string once read."""
        return self._state()["model_name"]  # type: ignore[no-any-return]

    @property
    def boot_baudrate(self) -> int | None:
        """The unit's power-up rate from BRSL (item 8.3), once read."""
        return self._state()["boot_baud_rate"]  # type: ignore[no-any-return]

    # ------------------------------------------------------------------ settings (the core axis's options)

    def _core_options(self) -> dict[str, Any]:
        o = _n.AxisOptions()
        self._call("iad_axis_get_options", ctypes.byref(o), io=False)
        return {
            "stroke": None if math.isnan(o.stroke_min) or math.isnan(o.stroke_max) else (o.stroke_min, o.stroke_max),
            "max_speed_mm_s": _opt(o.max_speed_mm_s), "max_accel_g": _opt(o.max_accel_g),
            "max_push_pct": _opt(o.max_push_pct), "poll_interval_s": float(o.poll_interval_s),
            "settle_s": float(o.settle_s), "method": int(o.method), "auto_modbus_control": bool(o.auto_modbus_control),
            "push_settle_s": max(float(o.push_settle_s), 0.0), "completion": _n._name_of(int(o.completion),
                                                                                          _COMPLETIONS, "safe"),
        }  # fmt: skip

    def _set_option(self, name: str, value: Any) -> None:
        with self._state_lock:
            self._options[name] = value
            if self._handle is not None:
                self._call("iad_axis_set_options", ctypes.byref(self._axis_options()), io=False)

    def _option(self, name: str) -> Any:
        return self._options[name]

    max_speed_mm_s = property(lambda self: self._option("max_speed_mm_s"),
                              lambda self, v: self._set_option("max_speed_mm_s", None if v is None else float(v)),
                              doc="Configured speed cap (units/s); ``None`` = the unit's own maximum.")  # fmt: skip
    max_accel_g = property(lambda self: self._option("max_accel_g"),
                           lambda self, v: self._set_option("max_accel_g", None if v is None else float(v)),
                           doc="Configured accel / decel cap (G); ``None`` = the unit's MAXA / MAXD.")  # fmt: skip
    max_push_pct = property(lambda self: self._option("max_push_pct"),
                            lambda self, v: self._set_option("max_push_pct", None if v is None else float(v)),
                            doc="Configured push force cap (%); ``None`` = the unit's MXPP.")  # fmt: skip
    poll_interval_s = property(lambda self: self._option("poll_interval_s"),
                               lambda self, v: self._set_option("poll_interval_s", float(v)),
                               doc="Status polling period while waiting.")  # fmt: skip
    settle_s = property(lambda self: self._option("settle_s"), lambda self, v: self._set_option("settle_s", float(v)),
                        doc="A move, home return or STOP is done once PNOW stayed unchanged this long.")  # fmt: skip
    push_settle_s = property(lambda self: self._option("push_settle_s"),
                             lambda self, v: self._set_option("push_settle_s", float(v)),
                             doc="A push outcome must hold this long before it is reported.")  # fmt: skip
    auto_modbus_control = property(lambda self: self._option("auto_modbus_control"),
                                   lambda self, v: self._set_option("auto_modbus_control", bool(v)),
                                   doc="Take Modbus control (PMSL) automatically at rest before a command.")  # fmt: skip

    @property
    def completion(self) -> str:
        """``"safe"`` / ``"fast"``: the default completion rule of this axis's moves (:meth:`enable_fast` sets fast)."""
        return str(self._core_options()["completion"]) if self._handle is not None else self._options["completion"]

    @completion.setter
    def completion(self, value: str) -> None:
        self._set_option("completion", _check_completion(value))

    @property
    def strategy(self) -> CommandStrategy:
        """The command strategy for servo / home / alarm reset (the core's, which adapts to the line and to a unit
        without coils). Settable (a name or a :class:`CommandStrategy`)."""
        code = ctypes.c_int32(0)
        self._call("iad_axis_strategy", ctypes.byref(code), io=False)
        name = _METHOD_NAMES.get(int(code.value), "coil")
        if getattr(self._strategy, "name", None) != name:
            self._strategy = STRATEGIES[name]()
        return self._strategy

    @strategy.setter
    def strategy(self, value: CommandStrategy | str) -> None:
        if isinstance(value, str):
            if value not in STRATEGIES:
                raise ValueError(f"unknown strategy {value!r}; choose from {sorted(STRATEGIES)}")
            value = STRATEGIES[value]()
        self._call("iad_axis_set_strategy", _choice(getattr(value, "name", ""), _METHODS, "strategy"), io=False)
        self._strategy = value

    # ------------------------------------------------------------------ power / link loss (recovery)

    def track(self, **kwargs: Any) -> Any:
        """:func:`iai_actuator_driver.recovery.track` on this axis: remember its identity, baud rate, PC control,
        safety speed, HEND and uptime while the link is up (the baseline :meth:`check_link` and :meth:`recover` compare
        with). READ-ONLY. ``kwargs``: ``name``, ``info``, ``alarm_history``, ``rows``, ``parameters``, ``backup``.
        Returns the :class:`~iai_actuator_driver.recovery.Session`."""
        from . import recovery

        return recovery.track(self, **kwargs)

    def check_link(self, **kwargs: Any) -> Any:
        """:func:`iai_actuator_driver.recovery.check_link` on this axis: one status read, compared with the session for
        restart signs. Returns a :class:`~iai_actuator_driver.recovery.LinkCheck` (``state``: ``"ok"``,
        ``"port_vanished"``, ``"unit_silent"``, ``"unit_restarted"``). READ-ONLY. ``kwargs``: ``on_event``, ``name``."""
        from . import recovery

        return recovery.check_link(self, **kwargs)

    @property
    def link_state(self) -> str:
        """The last known link state (:data:`iai_actuator_driver.recovery.LINK_STATES`; ``"ok"`` when nothing was
        seen)."""
        from . import recovery

        return recovery.link_state(self)

    def recover(
        self,
        *,
        timeout: float | None = None,
        policy: Any = None,
        session: Any = None,
        on_event: Any = None,
        name: str | None = None,
    ) -> Any:
        """Bring this axis back after a power or link loss (recovery::recover_axis: wait for the port, reopen it, find
        the baud rate, require the same unit, restore baud rate / PC control / safety speed, assess alarm, HEND and
        position). **Never starts motion.** Returns the :class:`~iai_actuator_driver.recovery.RecoveryReport`.

        Without a :meth:`track` session the identity this axis read last (``identify()`` / ``connect()``) is the
        baseline (no I/O). The status monitor is held while the recovery runs (with every other running monitor on this
        port) and restarted afterwards when it was running or ended on the fault (the port vanished); with
        ``on_event`` it stays held until the recovery's events have been delivered. ``timeout``: seconds to wait for
        the port / unit (default 30). Raises :class:`~iai_actuator_driver.recovery.RecoveryError` /
        :class:`~iai_actuator_driver.recovery.UnitReplacedError` as the function does (the monitor is then left as it
        was)."""
        from . import recovery

        return recovery._recover(self, timeout=recovery.RECOVER_TIMEOUT_S if timeout is None else timeout,
                                 policy=policy, session=session, on_event=on_event, name=name,
                                 require_session=False)  # fmt: skip

    # ------------------------------------------------------------------ closing

    def close(self) -> None:
        """Stop the status monitor, release a STOP (or a register STOP's pause) left on (best effort), then close the
        bus if this object opened it. What the axis read stays readable."""
        try:
            self.stop_monitor()
            if self._handle is not None and not getattr(self._bus, "closed", False):
                code, details = _n._invoke(self._lib, "iad_axis_release_held", self._handle)
                if code != _n.IAD_OK:
                    log.warning("could not release the STOP on close: %s", (details or {}).get("message"))
        finally:
            if self.owns_bus:
                self.bus.close()

    def __enter__(self) -> Elecylinder:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, tb: object) -> None:
        try:
            # Only stop if this object commanded motion: read-only use must never write to the unit.
            if exc is not None and not getattr(exc, "stop_attempted", False) and self._still_moving():
                self._try_stop(exc, "on leaving the Elecylinder context")
        finally:
            self.close()

    def _still_moving(self) -> bool:
        try:
            return self._handle is not None and self.motion_pending
        except (ModbusError, OSError, ValueError):
            return False

    def _try_stop(self, exc: BaseException, when: str) -> None:
        exc.stop_attempted = True  # type: ignore[attr-defined]
        what = "decel stop (DRG1 pause + cancel)" if self._stop_path_now() == "register" else "decel stop"
        try:
            self.stop(timeout=_ERROR_STOP_HOLD_S)
        except (Exception, KeyboardInterrupt) as stop_exc:
            exc.add_note(f"iai_actuator_driver: {what} {when} failed too: {stop_exc}")
        else:
            exc.add_note(f"iai_actuator_driver: {what} sent {when}")

    def _stop_path_now(self) -> str:
        try:
            return self.stop_path
        except (ModbusError, OSError, ValueError):
            return "coil"

    def __repr__(self) -> str:
        try:
            soft = self.soft_limits_mm
            strategy = self.strategy.name
        except (ModbusError, OSError, ValueError):
            soft, strategy = None, getattr(self._strategy, "name", "coil")
        return f"Elecylinder(slave={self.slave}, stroke={self.stroke}, soft_limits={soft}, strategy={strategy!r})"

    # ------------------------------------------------------------------ stroke / soft limits

    @property
    def stroke(self) -> tuple[float, float] | None:
        """The configured stroke (mm), or ``None`` to use the unit's soft limits. Settable."""
        return self._options["stroke"]  # type: ignore[no-any-return]

    @stroke.setter
    def stroke(self, value: tuple[float, float] | None) -> None:
        self._set_option("stroke", _check_stroke(value) if value is not None else None)

    @property
    def ec_stroke(self) -> tuple[float, float] | None:
        """The unit's soft limits minus its 0.30 mm margin (its nominal stroke), once read."""
        soft = self.soft_limits_mm
        if soft is None:
            return None
        lo, hi = soft
        m = R.SOFT_LIMIT_MARGIN_MM
        if hi - lo > 2 * m + 0.01:
            return round(lo + m, 2), round(hi - m, 2)
        return lo, hi

    @property
    def effective_stroke(self) -> tuple[float, float] | None:
        """Limits every target is checked against: configured stroke and unit stroke intersected (no I/O)."""
        cfg, unit = self.stroke, self.ec_stroke
        if cfg is None or unit is None:
            return cfg or unit
        lo, hi = max(cfg[0], unit[0]), min(cfg[1], unit[1])
        return (lo, hi) if lo < hi else None

    def read_soft_limits(self) -> tuple[float, float] | None:
        """Read the unit's own soft limits (min, max) from items 7.1 / 7.2; ``None`` with the reason in
        :attr:`soft_limits_note`. READ-ONLY."""
        return _tuple(self._json("iad_axis_read_soft_limits_json")["soft_limits_mm"])  # type: ignore[no-any-return]

    # ------------------------------------------------------------------ caps (no I/O)

    @property
    def speed_cap_mm_s(self) -> float:
        """Highest speed the driver commands: min(``max_speed_mm_s``, MAXV), each only when known."""
        return float(self._state()["speed_cap_mm_s"])

    @property
    def ec_max_accel_g(self) -> float | None:
        """The unit's MAXA (item 7.5) once read."""
        limits = self.axis_limits
        return None if limits is None else limits.max_accel_g

    @property
    def ec_max_decel_g(self) -> float | None:
        """The unit's MAXD (item 7.6) once read."""
        limits = self.axis_limits
        return None if limits is None else limits.max_decel_g

    @property
    def accel_cap_g(self) -> float:
        """Highest acceleration (= deceleration on a 0x9900 move): min(``max_accel_g``, MAXA, MAXD), when known."""
        return float(self._state()["accel_cap_g"])

    @property
    def accel_only_cap_g(self) -> float:
        """Highest acceleration on a move with its own deceleration: min(``max_accel_g``, MAXA)."""
        return float(self._state()["accel_only_cap_g"])

    @property
    def decel_cap_g(self) -> float:
        """Highest deceleration on a move with its own deceleration: min(``max_accel_g``, MAXD)."""
        return float(self._state()["decel_cap_g"])

    @property
    def push_cap_pct(self) -> float:
        """Highest push force: min(``max_push_pct``, MXPP 7.9), each only when known."""
        return float(self._state()["push_cap_pct"])

    # ------------------------------------------------------------------ generic axis: units, kind, ranges

    def _kind(self) -> dict[str, Any]:
        return self._json("iad_axis_kind_json")  # type: ignore[no-any-return]

    def _limits_kind(self, name: str, fallback: str) -> str:
        """``units`` / ``kind`` of the unit's limits: once read, no I/O (also after close); else read once, and
        ``fallback`` when they are not readable."""
        limits = self.axis_limits
        if limits is not None:
            return str(getattr(limits, name))
        try:
            return str(self._kind()[name])
        except (ModbusError, OSError) as exc:
            log.info("axis limits not readable: %s", exc)
            return fallback

    @property
    def units(self) -> str:
        """``"deg"`` on a rotary axis, else ``"mm"``. Reads the unit's limits once."""
        return self._limits_kind("units", "mm")

    @property
    def kind(self) -> str:
        """Actuator kind in words from AKID (``"unknown"`` when not readable). Reads the limits once."""
        return self._limits_kind("kind", "unknown")

    @property
    def product_family(self) -> str:
        """Family of the FC11 system code: ``"EC"``, ``"PCON"``, ``"ACON/DCON"``, ``"SCON"`` or ``"IAI controller"``."""
        code = self.system_code
        return R.product_family(code) if code is not None else str(self._kind()["product_family"])

    @property
    def is_ec(self) -> bool:
        """The unit is an ELECYLINDER (system 0xEC / 0xED); otherwise a CON-family controller."""
        code = self.system_code
        return R.is_ec_family(code) if code is not None else bool(self._kind()["is_ec"])

    def max_positions(self) -> int:
        """Number of stored rows the unit uses (rows 0 .. n - 1; an EC reports at least 3). Read once. READ-ONLY."""
        count = ctypes.c_int32(0)
        self._call("iad_axis_max_positions", ctypes.byref(count))
        return int(count.value)

    def limits(self) -> dict[str, Any]:
        """Every range the driver enforces on this axis, as a JSON-able dict, read from the unit. READ-ONLY."""
        return self._json("iad_axis_limits_json")  # type: ignore[no-any-return]

    def read_axis_limits(self) -> AxisLimits:
        """Read the unit's own limits from class 7 by item ID (and SUPT from class 1). READ-ONLY."""
        return _record(AxisLimits, self._json("iad_axis_read_axis_limits_json"))  # type: ignore[no-any-return]

    def read_speed_limit(self) -> float | None:
        """Read the unit's maximum speed: MAXV (item 7.4), else the 0x0812 default speed; ``None`` with the reason in
        :attr:`ec_max_speed_note`. READ-ONLY."""
        speed = ctypes.c_double(0.0)
        self._call("iad_axis_read_speed_limit", ctypes.byref(speed))
        return _opt(speed.value)

    def read_support_words(self) -> dict[str, int] | None:
        """Class 1 support-function words by name (SUPT, DIAG, MVPT ... SUP3); ``None`` when not readable. READ-ONLY."""
        return self._json("iad_axis_read_support_words_json")  # type: ignore[no-any-return]

    def recommended_min_speed(self) -> tuple[float | None, str]:
        """(documented minimum speed in units/s, source); the driver warns below it, never refuses. READ-ONLY."""
        caps = _n.MotionCaps()
        self._call("iad_axis_motion_caps", ctypes.byref(caps))
        return _opt(caps.min_speed), _n._text(caps.min_speed_source)

    def push_speed_mm_s(self, speed: float | None = None) -> tuple[float, str]:
        """(push speed in units/s, source): min(``speed``, PSHV 8.15 / CON No.34, MXPH 7.8). READ-ONLY."""
        data = self._json("iad_axis_push_speed_json", _nan(speed))
        return float(data["speed_mm_s"]), str(data["source"])

    @property
    def line_name(self) -> str | None:
        """The line kind (``"rcon"``, ``"robonet"``, ``"robonet-gateway"``), ``None`` on a direct line."""
        return self._text("iad_axis_line_name", io=False) or None

    @property
    def stop_path(self) -> str:
        """``"coil"`` (the STOP coil sequence) or ``"register"`` (the coil-free substitute)."""
        return self._text("iad_axis_stop_path", io=False)

    # ------------------------------------------------------------------ reading (READ-ONLY)

    def status(self) -> Status:
        """Read and decode the monitor block (22 words; 20, then 8 when the unit refuses more)."""
        return self._status_call("iad_axis_status")  # type: ignore[return-value]

    def read_lean_status(self, *, retries: int | None = None) -> Status:
        """The lean status: one read of 0x9000-0x9007 (8 words). Read-only."""
        if retries is not None and (not isinstance(retries, int) or retries < 0):
            raise ValueError("retries must be >= 0")
        return self._status_call("iad_axis_read_lean_status",  # type: ignore[return-value]
                                 _n.IAD_INT_UNKNOWN if retries is None else retries)  # fmt: skip

    def identify(self) -> DeviceInfo:
        """Read the unit's identity, directory, soft limits and limits (READ-ONLY; FC03 + FC11)."""
        out = _n.Identity()
        self._call_waiting("iad_axis_identify", ctypes.byref(out))
        return self._device_info()

    def read_registers(self, address: int, count: int) -> list[int]:
        """Raw FC03 read from this axis (diagnostics)."""
        return self.bus.read_registers(self.slave, address, count)

    @property
    def categories(self) -> tuple[IndexCategory, ...]:
        """The unit's directory records (index table at 0x0000) as read so far (empty before)."""
        return tuple(_record(IndexCategory, c) for c in self._state()["categories"])

    def unit_info(self, *, sections: Iterable[str] | None = None) -> dict[str, Any]:
        """Everything the unit tells about itself, decoded, as one JSON-serialisable dict. READ-ONLY."""
        names = None if sections is None else ",".join(sections)
        return self._json("iad_axis_read_unit_info_json", _n._utf8(names))  # type: ignore[no-any-return]

    def read_position_table(self, n: int) -> PositionEntry:
        """Read stored row ``n`` (in the axis's units)."""
        out = _n.CPositionEntry()
        self._call("iad_axis_read_position_table", int(n), ctypes.byref(out))
        return _n._entry_from_c(out)

    def read_alarm_history(self, *, include_empty: bool = False) -> list[AlarmRecord]:
        """Alarm history, newest first; empty slots left out unless ``include_empty``."""
        records = self._json("iad_axis_read_alarm_history_json", int(bool(include_empty)))
        return [_record(AlarmRecord, r) for r in records]

    # ------------------------------------------------------------------ control

    def enable_modbus_control(self, verify: bool = True) -> Status | None:
        """Give Modbus command authority (PMSL); nothing written when PMSS is already 1. Refused while the axis
        moves, homes or pushes; a pause (DSS1 STP) at a standstill with the servo off is no refusal, a pause with the
        servo on is (a PLC holding a stop input on a move). A PCON in PIO mode with nothing on its PIO connector
        pauses and turns the servo off (VERIFIED on the PCON-CB, 2026-11-02); 1.0.1 taking control there: VERIFIED
        (PCON-CB, 2026-11-02)."""
        return self._status_call("iad_axis_enable_modbus_control", int(bool(verify)))

    def disable_modbus_control(self) -> None:
        """Hand command authority back to PIO (PMSL = 0). Refused while the axis moves."""
        self._call("iad_axis_modbus_control", 0)

    def servo(self, on: bool, *, wait: bool = True, timeout: float | None = None) -> Status | None:
        """Servo on / off; a servo ON the status shows cannot work is refused before anything is sent
        (:class:`ServoBlockedError` with the reasons). Returns the status when ``wait``."""
        o = _n.ServoOptions(timeout_s=_nan(timeout), wait=int(bool(wait)))
        return self._status_call("iad_axis_servo", int(bool(on)), ctypes.byref(o), waiting=True)

    def servo_on_timeout(self, st: Status | None = None) -> tuple[float, str]:
        """(seconds, why) to wait for a servo ON (3 s, or a CON unit's first servo ON after power-up). READ-ONLY."""
        if st is None:
            data = self._json("iad_axis_servo_on_timeout_json", None, 0)
        else:
            words = _status_words_of(st)
            data = self._json("iad_axis_servo_on_timeout_json", (ctypes.c_uint16 * len(words))(*words), len(words))
        return float(data["timeout_s"]), str(data["reason"])

    def reset_alarm(self, *, method: str | None = None) -> Status:
        """Reset the current alarm (not a software reset) and return the new status."""
        code = _METHOD_DEFAULT if method is None else _choice(method, _METHODS, "method")
        return self._status_call("iad_axis_reset_alarm", code)  # type: ignore[return-value]

    def set_action_mode(self, pc: bool = True, safety: bool | None = None, *, verify: bool = True) -> Status:
        """The action mode: ``pc`` = PC / Modbus control (the PMSL coil, and the SFTY coil when the safety speed
        changes) or PIO (0x45 cmd 0x0C op1 = 0), ``safety`` the safety speed (``None`` keeps the present one)."""
        tri = _n.IAD_UNKNOWN if safety is None else int(bool(safety))
        return self._status_call("iad_axis_set_action_mode", int(bool(pc)), tri,  # type: ignore[return-value]
                                 int(bool(verify)))  # fmt: skip

    def read_home_pattern(self) -> int | None:
        """The unit's home pattern ORG (item 9.16 by item ID); ``None`` when not in the directory. READ-ONLY."""
        known, pattern = _n._c_bool(0), ctypes.c_uint32(0)
        self._call("iad_axis_read_home_pattern", ctypes.byref(known), ctypes.byref(pattern))
        return int(pattern.value) if known.value else None

    def read_boot_baudrate(self) -> int | None:
        """The unit's power-up rate from BRSL (item 8.3); ``None`` when not readable. READ-ONLY."""
        rate = ctypes.c_int32(0)
        self._call("iad_axis_read_boot_baud_rate", ctypes.byref(rate))
        return None if rate.value in (0, _n.IAD_INT_UNKNOWN) else int(rate.value)

    def set_safety_speed(self, on: bool, *, verify: bool = True) -> Status:
        """Switch the safety-speed mode (SFTY coil, or DRG1 b14 on the register path; volatile)."""
        return self._status_call("iad_axis_set_safety_speed", int(bool(on)), int(bool(verify)))  # type: ignore[return-value]

    def home(self, *, wait: bool = True, timeout: float | None = None, method: str | None = None) -> Status | None:
        """Home return; with ``wait`` waits for HEND and standstill and returns the status. With ``wait=False`` call
        :meth:`finish_home` once the axis is homed."""
        code = _METHOD_DEFAULT if method is None else _choice(method, _METHODS, "method")
        o = _n.HomeOptions(timeout_s=_nan(timeout), wait=int(bool(wait)), method=code)
        return self._status_call("iad_axis_home", ctypes.byref(o), waiting=True)

    def finish_home(self) -> None:
        """Release what a home start left set (the HOME coil, the DRG1 HOME bit): needed after ``home(wait=False)``."""
        self._call("iad_axis_finish_home")

    # ------------------------------------------------------------------ motion

    def _move_options(
        self, init: str, speed: float, accel: float, band: float, wait: bool, timeout: float | None, *,
        decel_g: float | None = None, accel_pattern: str | None = None, completion: str | None = None,
    ) -> _n.MoveOptions:
        o = _n.MoveOptions()
        _n._check(self._lib, init, ctypes.byref(o))
        o.speed, o.accel_g, o.band = _finite("speed", speed), _finite("accel", accel), _finite("band", band)
        o.timeout_s = _nan(timeout)
        o.wait = int(bool(wait))
        o.decel_g = _nan(decel_g)
        if accel_pattern is not None and accel_pattern not in _ACCEL_PATTERNS:
            raise SoftLimitError(f"accel_pattern must be one of {sorted(_ACCEL_PATTERNS)} (or None), got "
                                 f"{accel_pattern!r}")  # fmt: skip
        o.accel_pattern = 0 if accel_pattern is None else _ACCEL_PATTERNS[accel_pattern]
        o.completion = 0 if completion is None else _COMPLETIONS[_check_completion(completion)]
        return o

    def _move(self, name: str, value: float, o: _n.MoveOptions, wait: bool) -> Status | MoveHandle | None:
        if not wait:  # the move handle of iad_axis_start_move_to / _by / start_jog
            handle = ctypes.c_void_p()
            self._call(name.replace("iad_axis_", "iad_axis_start_"), float(value), ctypes.byref(o),
                       ctypes.byref(handle))  # fmt: skip
            return MoveHandle._started(self, int(handle.value or 0))
        return self._status_call(name, float(value), ctypes.byref(o), waiting=True)

    def check_move(
        self,
        mm: float,
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        *,
        decel_g: float | None = None,
        accel_pattern: str | None = None,
    ) -> Status:
        """Every check :meth:`move_to` runs, with nothing sent; raises what it would and returns the status read."""
        o = self._move_options("iad_move_options_init", speed_mm_s, accel_g, band_mm, True, None, decel_g=decel_g,
                               accel_pattern=accel_pattern)  # fmt: skip
        return self._status_call("iad_axis_check_move", _finite("target", mm),  # type: ignore[return-value]
                                 ctypes.byref(o))  # fmt: skip

    def move_to(
        self,
        mm: float,
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        decel_g: float | None = None,
        accel_pattern: str | None = None,
        completion: str | None = None,
    ) -> Status | MoveHandle | None:
        """Absolute move, checked against the unit's ranges before anything is sent (:class:`SoftLimitError`). With
        ``wait`` blocks until done (Ctrl+C stops the axis, decel); with ``wait=False`` returns a :class:`MoveHandle`."""
        o = self._move_options("iad_move_options_init", speed_mm_s, accel_g, band_mm, wait, timeout, decel_g=decel_g,
                               accel_pattern=accel_pattern, completion=completion)  # fmt: skip
        return self._move("iad_axis_move_to", _finite("target", mm), o, wait)

    def move_by(
        self,
        delta_mm: float,
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        accel_pattern: str | None = None,
        completion: str | None = None,
    ) -> Status | MoveHandle | None:
        """Move relative to the measured position, sent as an absolute target. As :meth:`move_to`."""
        o = self._move_options("iad_move_options_init", speed_mm_s, accel_g, band_mm, wait, timeout,
                               accel_pattern=accel_pattern, completion=completion)  # fmt: skip
        return self._move("iad_axis_move_by", _finite("distance", delta_mm), o, wait)

    def jog(
        self,
        delta_mm: float,
        speed_mm_s: float = 10.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        completion: str | None = None,
    ) -> Status | MoveHandle | None:
        """A small relative step at a low default speed (limits ``jog_step``). As :meth:`move_to`."""
        o = self._move_options("iad_jog_options_init", speed_mm_s, accel_g, band_mm, wait, timeout,
                               completion=completion)  # fmt: skip
        return self._move("iad_axis_jog", _finite("distance", delta_mm), o, wait)

    def push(
        self,
        target_mm: float,
        band_mm: float,
        force_pct: float,
        direction: int = 1,
        speed_mm_s: float = 10.0,
        accel_g: float = 0.1,
        *,
        wait: bool = True,
        timeout: float | None = None,
    ) -> PushResult | None:
        """Push move: approach ``target_mm``, then push over ``band_mm`` at ``force_pct``. With ``wait`` returns the
        outcome (``"contact"`` / ``"missed"``) with its warnings; otherwise ``None`` right after the command."""
        if direction not in (1, -1):
            raise SoftLimitError(f"direction must be +1 (away from home) or -1 (toward home), got {direction!r}")
        if isinstance(force_pct, bool) or not isinstance(force_pct, (int, float)) or not math.isfinite(force_pct):
            raise SoftLimitError(f"invalid push force {force_pct!r}")
        o = _n.PushOptions()
        _n._check(self._lib, "iad_push_options_init", ctypes.byref(o))
        o.direction = int(direction)
        o.speed = float(speed_mm_s)
        o.accel_g = float(accel_g)
        o.timeout_s = _nan(timeout)
        o.wait = int(bool(wait))
        out = _n.PushResult()
        kept = self._call_waiting("iad_axis_push", float(target_mm), float(band_mm), float(force_pct), ctypes.byref(o),
                                  ctypes.byref(out), kept=True)  # fmt: skip
        if not wait:
            return None
        record = self._kept_result(kept)
        status = self._status(out.status) or _status_from_json(record.get("status"))
        return PushResult(outcome=str(record.get("outcome") or _n._text(out.outcome_name)),
                          position_mm=float(record.get("position_mm", out.position_mm)), status=status,  # type: ignore[arg-type]
                          warnings=tuple(str(w) for w in record.get("warnings") or ()))  # fmt: skip

    def move_to_position(
        self,
        n: int,
        wait: bool = True,
        timeout: float | None = None,
        *,
        allow_above_caps: bool = False,
        speed_pct: float | None = None,
    ) -> Status | None:
        """Run stored row ``n`` (read and checked first; Modbus control taken at rest). ``speed_pct`` 1..100 % of the
        row's speed; ``allow_above_caps``: only the unit's own limits apply to the row."""
        o = _n.PositionMoveOptions()
        _n._check(self._lib, "iad_position_move_options_init", ctypes.byref(o))
        o.timeout_s = _nan(timeout)
        o.speed_pct = _nan(speed_pct)
        o.wait = int(bool(wait))
        o.allow_above_caps = int(bool(allow_above_caps))
        return self._status_call("iad_axis_move_to_position", int(n), ctypes.byref(o), waiting=True)

    def stop(self, *, wait: bool = False, timeout: float = 5.0) -> Status | None:
        """Decel stop (the STOP coil held until standstill, or the register substitute); blocks for the deceleration.
        With ``wait``, :class:`MotionTimeoutError` when still moving after ``timeout``. Returns the last status."""
        o = _n.StopOptions(timeout_s=float(timeout), wait=int(bool(wait)))
        return self._status_call("iad_axis_stop", ctypes.byref(o))

    def begin_stop(self) -> None:
        """Send only the first frame of :meth:`stop` (STOP ON, or the DRG1 pause) and return at once; a following
        :meth:`stop` / :meth:`close` releases it."""
        self._call("iad_axis_begin_stop")

    def pause(self) -> None:
        """Decelerate and hold, keeping the remaining travel (STP)."""
        self._call("iad_axis_pause")

    def resume(self) -> None:
        """Release the pause; a paused move continues by itself."""
        self._call("iad_axis_resume")

    def cancel_paused_move(self, *, timeout: float = 5.0) -> Status:
        """Drop the remaining travel of a paused move and release the pause; returns the status after the release."""
        o = _n.CancelPausedMoveOptions(timeout_s=float(timeout))
        return self._status_call("iad_axis_cancel_paused_move", ctypes.byref(o),  # type: ignore[return-value]
                                 waiting=True)  # fmt: skip

    def change_speed(self, speed_mm_s: float, *, wait: bool = False, timeout: float | None = None) -> Status | None:
        """Change the speed of the 0x9900 move in progress (the full block re-sent). With ``wait`` waits for it."""
        o = _n.ChangeOptions(timeout_s=_nan(timeout), wait=int(bool(wait)))
        return self._status_call("iad_axis_change_speed", float(speed_mm_s), ctypes.byref(o), waiting=True)

    def change_accel(self, accel_g: float, *, wait: bool = False, timeout: float | None = None) -> Status | None:
        """Change the acceleration (= deceleration) of the 0x9900 move in progress."""
        o = _n.ChangeOptions(timeout_s=_nan(timeout), wait=int(bool(wait)))
        return self._status_call("iad_axis_change_accel", float(accel_g), ctypes.byref(o), waiting=True)

    def teach_position(
        self,
        n: int,
        *,
        allow_nonvolatile: bool = False,
        method: str = "table",
        allow_unsafe: bool = False,
        force: bool = False,
    ) -> PositionEntry:
        """Store the current position as row ``n`` (non-volatile). ``method`` ``"table"`` or the EXPERIMENTAL
        ``"coil"``. Returns the row read back (``skipped`` when it already held the position). ``allow_nonvolatile`` /
        ``allow_unsafe``: accepted and ignored (no longer needed); a row the position mode does not run is taught with
        a WARNING."""
        code = _choice(method, _TEACH_METHODS, "teach method")
        o = self._nv_options(allow_nonvolatile, force, allow_unsafe)
        out = _n.PositionWriteResult()
        self._call_waiting("iad_axis_teach_position", int(n), code, ctypes.byref(o), ctypes.byref(out))
        return _n._entry_from_c(out, skipped=bool(out.skipped))

    def wait_idle(
        self,
        target_mm: float | None = None,
        band_mm: float | None = None,
        timeout: float = 10.0,
        *,
        completion: str | None = None,
        since: float | None = None,
    ) -> Status:
        """Block until the move is done by the completion rule, at ``target_mm`` within ``band_mm`` when given.
        :class:`MotionIncompleteError` when the axis settles away from the target."""
        mode = 0 if completion is None else _COMPLETIONS[_check_completion(completion)]  # 0: the axis option
        o = _n.WaitIdleOptions(target=_nan(target_mm), band=_nan(band_mm), timeout_s=float(timeout), completion=mode)
        return self._status_call("iad_axis_wait_idle", ctypes.byref(o), waiting=True)  # type: ignore[return-value]

    def move_sequence(
        self,
        points: Iterable[float | Mapping[str, float]],
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        *,
        blend_band: float | None = None,
        completion: str | None = None,
        timeout: float | None = None,
    ) -> list[Status]:
        """Absolute moves through ``points`` in order; one status per point. Every point is checked before the first
        command (:class:`SoftLimitError` names the point; nothing sent)."""
        if blend_band is not None and (isinstance(blend_band, bool) or not isinstance(blend_band, (int, float))
                                       or not math.isfinite(blend_band)):  # fmt: skip
            raise SoftLimitError(f"invalid blend_band {blend_band!r}")
        plan: list[_n.SequencePoint] = []
        for i, point in enumerate(points):
            p = _n.SequencePoint()
            p.speed = p.accel_g = p.band = math.nan
            if isinstance(point, Mapping):
                unknown = set(point) - {"mm", "speed_mm_s", "accel_g", "band_mm"}
                if "mm" not in point or unknown:
                    raise SoftLimitError(f"point {i}: a mapping needs 'mm' and only speed_mm_s / accel_g / band_mm")
                p.position = float(point["mm"])
                p.speed = _n._slot("speed_mm_s", point.get("speed_mm_s"))
                p.accel_g = _n._slot("accel_g", point.get("accel_g"))
                p.band = _n._slot("band_mm", point.get("band_mm"))
            else:
                p.position = float(point)
            plan.append(p)
        o = _n.SequenceOptions()
        _n._check(self._lib, "iad_sequence_options_init", ctypes.byref(o))
        o.speed, o.accel_g, o.band = float(speed_mm_s), float(accel_g), float(band_mm)
        o.blend_band = _nan(blend_band)
        o.timeout_s = _nan(timeout)
        o.completion = 0 if completion is None else _COMPLETIONS[_check_completion(completion)]
        points_c = (_n.SequencePoint * max(len(plan), 1))(*plan)
        statuses = (_n.Status * max(len(plan), 1))()
        count = ctypes.c_size_t(0)
        self._call_waiting("iad_axis_move_sequence", points_c, len(plan), ctypes.byref(o), statuses, len(plan),
                           ctypes.byref(count))  # fmt: skip
        return [st for st in (self._status(s) for s in statuses[: min(count.value, len(plan))]) if st is not None]

    # ------------------------------------------------------------------ non-volatile write guard

    def _nv_counts(self) -> tuple[int, int, bool]:
        total, last_hour, uncommitted = ctypes.c_int64(0), ctypes.c_int32(0), _n._c_bool(0)
        self._call("iad_axis_nonvolatile_writes", ctypes.byref(total), ctypes.byref(last_hour),
                   ctypes.byref(uncommitted), io=False)  # fmt: skip
        return int(total.value), int(last_hour.value), bool(uncommitted.value)

    @property
    def nonvolatile_writes(self) -> int:
        """Non-volatile write cycles this unit took from this process (parameter commits + position-table writes);
        skipped writes are not counted. About 100,000 cycles in a unit's life (IAI)."""
        return self._nv_counts()[0]

    @property
    def nonvolatile_writes_last_hour(self) -> int:
        """The same, counted over the last hour (one WARNING per hour above :data:`NV_WARN_PER_HOUR`)."""
        return self._nv_counts()[1]

    def _nv_options(self, allow_nonvolatile: bool, force: bool, allow_unsafe: bool) -> _n.NvWriteOptions:
        o = _n.NvWriteOptions()
        _n._check(self._lib, "iad_nv_write_options_init", ctypes.byref(o))
        o.allow_nonvolatile = int(bool(allow_nonvolatile))
        o.force = int(bool(force))
        o.allow_unsafe = int(bool(allow_unsafe))
        return o

    def write_position_table(
        self,
        n: int,
        entry: PositionEntry,
        allow_nonvolatile: bool = False,
        *,
        write_checksum: bool = True,
        allow_unsafe: bool = False,
        allow_unverified_model: bool = False,
        force: bool = False,
    ) -> PositionEntry:
        """Write stored row ``n`` with FC10 and verify it by reading it back (non-volatile; an unchanged row is
        skipped unless ``force``). A row outside the unit's rows or its position mode, a push on the middle row and
        unknown control-flag bits are written with a WARNING; push power and load threshold must be in the unit's
        range. ``allow_nonvolatile`` / ``allow_unsafe`` / ``allow_unverified_model``: accepted and ignored (no longer
        needed)."""
        c_entry = _n.CPositionEntry()
        _n._check(self._lib, "iad_position_entry_init", ctypes.byref(c_entry))
        for name in _FLOAT_ROW_FIELDS:
            value = getattr(entry, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise SoftLimitError(f"row {n} {name} {value!r} is not a finite number")
            setattr(c_entry, name, float(value))
        for name in ("push_power", "load_threshold", "control_flags"):
            value = getattr(entry, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise SoftLimitError(f"position {n}: {name} must be an int, got {value!r}")
            setattr(c_entry, name, int(value))
        c_entry.checksum = _n.IAD_INT_UNKNOWN  # ignored in inputs: the core writes the checksum of the data
        o = self._nv_options(allow_nonvolatile, force, allow_unsafe)
        out = _n.PositionWriteResult()
        self._call_waiting("iad_axis_write_position_row", int(n), ctypes.byref(c_entry), ctypes.byref(o),
                           int(bool(write_checksum)), ctypes.byref(out))  # fmt: skip
        return _n._entry_from_c(out, skipped=bool(out.skipped))

    # ------------------------------------------------------------------ parameters

    def user_parameters(self) -> list[dict[str, Any]]:
        """The EC user parameters for this unit (IAI's numbering), decoded. READ-ONLY (FC03, FC11 once)."""
        return self._json("iad_axis_user_parameters_json")  # type: ignore[no-any-return]

    def write_user_parameter(
        self,
        no: int,
        value: Any,
        *,
        commit: bool = True,
        allow_home_params: bool = False,
        allow_unlisted: bool = False,
        allow_unverified_model: bool = False,
        force: bool = False,
    ) -> ParameterWriteResult:
        """Write user parameter P``no`` = ``value``, verify it by read-back and (``commit``) save it. P3 / P4 and a
        parameter the unit's list does not show are written with a warning (``allow_home_params`` /
        ``allow_unlisted`` / ``allow_unverified_model``: accepted and ignored, no longer needed)."""
        if isinstance(no, bool) or not isinstance(no, int):
            raise UnsafeCommandError(f"parameter number must be an int, got {no!r}; nothing written")
        if isinstance(value, bool):  # P7 uses 1 = Disable: a bool would be ambiguous
            raise SoftLimitError(f"P{no}: pass the number or label IA-OS shows, not {value!r} (P7 uses 1 = Disable); "
                                 "nothing written")  # fmt: skip
        return self._write_parameter(f"P{no}", value, commit=commit, allow_dangerous=False,
                                     allow_unlisted=allow_unlisted, allow_home_params=allow_home_params,
                                     force=force)  # fmt: skip

    def parameter_model(self, *, visibility: bool = False) -> dict[str, Any]:
        """This unit's parameter list and whether it is verified for it (FC11 + 0x0600; ``visibility``: also the
        unit's runtime visibility rules). READ-ONLY."""
        return self._json("iad_axis_parameter_model_json", int(bool(visibility)))  # type: ignore[no-any-return]

    def software_reset(
        self, *, timeout: float = RESET_TIMEOUT_S, reset_wait_s: float = RESET_WAIT_S
    ) -> SoftwareResetResult:
        """Restart the unit's firmware (servo off first; at rest only) and wait until it answers again; committed
        parameters apply afterwards."""
        o = _n.ResetOptions()
        _n._check(self._lib, "iad_reset_options_init", ctypes.byref(o))
        o.timeout_s = float(timeout)
        o.reset_wait_s = float(reset_wait_s)
        out = _n.ResetResult()
        kept = self._call_waiting("iad_axis_software_reset", ctypes.byref(o), ctypes.byref(out), kept=True)
        record = self._kept_result(kept)
        status = self._status(out.status) or _status_from_json(record.get("status"))
        return _record(SoftwareResetResult, record, status=status, notes=tuple(record.get("notes") or ()))  # type: ignore[no-any-return]

    def read_monitor(self) -> dict[str, Any]:
        """The live monitor values (position, speed, current, load, alarm ...) decoded. READ-ONLY."""
        return self._json("iad_axis_read_monitor_json")  # type: ignore[no-any-return]

    def parameters(self) -> list[dict[str, Any]]:
        """The unit's named parameters, decoded (EC: P1-P18, CON: No.x; READ-ONLY)."""
        return self._json("iad_axis_parameters_json")  # type: ignore[no-any-return]

    def read_parameter(self, key: int | str) -> dict[str, Any]:
        """One parameter (READ-ONLY): a number (``2``, ``"P2"``, ``"No.35"``) or a raw item (``"8.15"``)."""
        return self._json("iad_axis_read_parameter_json", _n._utf8(str(key)))  # type: ignore[no-any-return]

    def write_parameter(
        self,
        key: int | str,
        value: Any,
        *,
        commit: bool = True,
        allow_dangerous: bool = False,
        allow_unlisted: bool = False,
        allow_home_params: bool = False,
        force: bool = False,
    ) -> ParameterWriteResult:
        """Write one parameter (verified by read-back; ``commit`` saves it with PARW; an unchanged value is skipped
        unless ``force``). Read-only, unmapped and out-of-range values are refused before anything is written; a
        dangerous, home or unlisted parameter is written with a warning (``allow_dangerous`` / ``allow_unlisted`` /
        ``allow_home_params``: accepted and ignored, no longer needed). At rest only."""
        return self._write_parameter(key, value, commit=commit, allow_dangerous=allow_dangerous,
                                     allow_unlisted=allow_unlisted, allow_home_params=allow_home_params,
                                     force=force)  # fmt: skip

    def _write_parameter(
        self, key: int | str, value: Any, *, commit: bool, allow_dangerous: bool, allow_unlisted: bool,
        allow_home_params: bool, force: bool,
    ) -> ParameterWriteResult:
        o = _n.ParamWriteOptions()
        _n._check(self._lib, "iad_param_write_options_init", ctypes.byref(o))
        o.commit = int(bool(commit))
        o.allow_dangerous = int(bool(allow_dangerous))
        o.allow_unlisted = int(bool(allow_unlisted))
        o.allow_home_params = int(bool(allow_home_params))
        o.force = int(bool(force))
        out = _n.ParamWriteResult()
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            kept = self._call_waiting("iad_axis_write_parameter_number", _n._utf8(str(key)), float(value),
                                      ctypes.byref(o), ctypes.byref(out), kept=True)  # fmt: skip
        else:
            kept = self._call_waiting("iad_axis_write_parameter", _n._utf8(str(key)), _n._utf8(str(value)),
                                      ctypes.byref(o), ctypes.byref(out), kept=True)  # fmt: skip
        record = self._kept_result(kept)
        return _record(ParameterWriteResult, record, warnings=tuple(record.get("warnings") or ()))  # type: ignore[no-any-return]

    def set_baudrate(
        self,
        rate: int,
        *,
        method: str = "direct",
        settle_s: float = BAUD_SETTLE_S,
        allow_unverified_code: bool = False,
    ) -> BaudChangeResult:
        """Change the unit's and the port's baud rate (volatile: 38400 again after power-up). At rest only, and only
        when this axis is the only one on the bus. A rate whose code is not verified on hardware is sent with a
        WARNING (``allow_unverified_code``: accepted and ignored, no longer needed)."""
        o = _n.BaudOptions()
        _n._check(self._lib, "iad_baud_options_init", ctypes.byref(o))
        o.method = _choice(method, _BAUD_METHODS, "method")
        o.settle_s = float(settle_s)
        o.allow_unverified_code = int(bool(allow_unverified_code))
        out = _n.BaudResult()
        kept = self._call_waiting("iad_axis_set_baud_rate", int(rate), ctypes.byref(o), ctypes.byref(out), kept=True)
        return _record(BaudChangeResult, self._kept_result(kept))  # type: ignore[no-any-return]


Axis = Elecylinder  # the generic name: one axis of any IAI unit
