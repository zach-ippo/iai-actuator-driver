"""Thread-safe controller between the HTTP handlers and one connected axis.

* One lock (``_driver_lock``) serialises every driver call, including the poller.
* A background thread polls status at ~5 Hz and records state changes as events.
* STOP never queues: it bumps a stop epoch (so commands still waiting for the
  lock are dropped), waits at most ``STOP_LOCK_WAIT_S`` for the lock, then sends
  the stop anyway (ModbusRTU is thread-safe per the driver contract), and the
  following status reads must show the axis idle within ``STOP_CONFIRM_S``.
  On hardware a Modbus STOP stopped a move commanded over Modbus with PMSS = 0
  (test 19); it has never been tried on a move a PLC commanded over PIO. So while
  PIO has control, the axis is watched for ``PIO_RESTART_WATCH_S`` after STOP and
  "AXIS MOVED AGAIN AFTER STOP" is raised if it moves (e.g. a PLC still holding
  its start input once STOP is released).
* Modbus control (PMSL) is taken before the first command, and only while the axis is
  at rest: switching PMSL on mid-move stops the move (hardware test 18).
* Every motion request is range-checked here, independently of the browser, against the
  stroke in use: by default the EC's own soft limits (minus their 0.30 mm margin) read at
  connect; a stroke typed in the connect form can only narrow it. Speeds are capped at
  min(100 mm/s, the EC's own maximum speed MAXV, item 7.4). The driver checks again (also MAXA / MAXD and the
  push cap MXPP, item 7.9).
* Jog is a relative move through the driver (the EC's JOG coils are never used). A push is
  sent without waiting; its outcome (contact / missed) is read from the status flags and
  reported only once it has held for ``PUSH_RESULT_SETTLE_S`` with the position unchanged
  (hardware test G5: the contact fingerprint appeared for a moment during a push that missed).
* Warnings logged by ``iai_actuator_driver`` (e.g. "axis still moving after STOP") appear in the event log.
* Unit info (read-only, FC03 + FC11 only): ``unit_info`` reads ``Elecylinder.unit_info`` in a few
  groups of sections; ``read_block`` reads one directory block for the register browser; ``identify``
  and ``positions`` re-read identity / table rows. These read-only calls never hold the bus lock for
  the whole call: it is taken per Modbus request (``_preemptible``), each request is short
  (``backend.GATED_READ_WORDS`` words, retried outside the lock), and STOP ends the read before its
  next request. So STOP's coil goes out after at most one short request, and the status poll,
  commands and STOP's own status reads get the bus between any two requests. ``unit_live`` decodes
  the poller's latest status words (no bus access). None of them can write to the EC.
* STOP has priority over the status poll while it waits for the bus lock, and arms the restart
  watch with the standstill position its own hold read (``AxisSession.stop`` returns it), even when
  another command held the bus. Status reads that began before the watch was armed are ignored by it.
* Configuration commands (each confirmed in the browser first): ``write_param`` (user parameter,
  committed with PARW, the frames IA-OS sends (ON-WIRE); applies after a software reset),
  ``software_reset`` (servo OFF + 0x45 cmd 5; the snapshot's ``swreset`` keeps the outcome, e.g. "needs
  home" on an incremental unit), ``set_baud`` (broadcast baud change 0x45 cmd 4, volatile) and
  ``run_stored`` (0x9800 = n at the row's stored speed). They go through ``_command`` like every other command, so STOP still bypasses
  the queue (it waits at most ``STOP_LOCK_WAIT_S`` for the bus, then is sent anyway).
"""

from __future__ import annotations

import contextlib
import logging
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Callable, Iterator

try:
    from .backend import (
        DEFAULT_SIM_PROFILE,
        DEFAULT_STROKE,
        SIM_PROFILE_NAMES,
        TABLE_LABELS,
        VERIFIED_BAUD_RATES,
        AxisSession,
        BackendError,
        check_port_name,
        decode_live,
        error_status,
        open_session,
    )
except ImportError:  # run as a script from apps/gui/
    from backend import (  # type: ignore[no-redef]
        DEFAULT_SIM_PROFILE,
        DEFAULT_STROKE,
        SIM_PROFILE_NAMES,
        TABLE_LABELS,
        VERIFIED_BAUD_RATES,
        AxisSession,
        BackendError,
        check_port_name,
        decode_live,
        error_status,
        open_session,
    )

SessionFactory = Callable[..., AxisSession]  # (port, baud, slave, simulate, *, stroke, sim_profile) -> AxisSession

# Unit info is read in these groups of iai_actuator_driver.info.SECTIONS (identity and parameters share the
# parameter classes; alarms reuse the monitor words). A group that fails or is ended by STOP leaves
# the groups read before it in the result. The bus lock is taken per request, not per group.
UNIT_INFO_GROUPS: tuple[tuple[str, ...], ...] = (
    ("identity", "parameters"), ("monitor", "alarms"), ("maintenance", "positions"),
)

MAX_SPEED_MM_S = 100.0  # GUI cap; lowered per connection to the EC's own MAXV, item 7.4 (gripper: 53 mm/s)
MAX_ACCEL_G = 0.3  # the test unit's own position table uses 0.30 G
DEFAULT_ACCEL_G = 0.1
PUSH_FORCE_MIN_PCT = 20.0  # the GUI's floor (the recommended minimum when the EC has no MNPP; the driver itself
#                             only warns below the EC's MNPP)
PUSH_FORCE_MAX_PCT = 70.0  # the GUI's cap; lowered per connection to the EC's MXPP (item 7.9: 60 % on the slider)
PUSH_SPEED_MAX_MM_S = 20.0  # the EC pushes at min(speed, 20 mm/s); the approach runs at the full speed
PUSH_START_S = 1.0  # a push must visibly start within this time
PUSH_SETTLE_S = 0.5  # stopped this long without contact / missed flags: the push was ended (STOP, alarm...)
PUSH_RESULT_SETTLE_S = 0.15  # contact / missed must hold this long, position unchanged (= the driver's rule)
PUSH_CREEP_MM = 0.05  # a holding contact may creep this much meanwhile (test 14: 0.01 mm in 0.26 s)
MOTION_SETTLE_S = 0.1  # "Motion finished at X mm" once idle with the position unchanged this long (= driver settle_s)
SAME_POSITION_MM = 0.005  # positions are 0.01 mm counts: closer than this = unchanged
MAX_STROKE_MM = 5000.0  # sanity bound for a typed stroke, not a property of any EC
STROKE_SLACK_MM = 1.0  # table max this far below the stroke max triggers the "shorter EC?" warning
BAUD_RATES = (9600, 19200, 38400, 57600, 115200, 230400)  # connect: the rate tried first (auto-detect follows)
BAUD_CHANGE_RATES = (*VERIFIED_BAUD_RATES, 38400)  # baud change: 115200 0x14 and 230400 0x15 (verified), and 38400 0x06:
#   the EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod IGNORED it; the driver checks after the switch;
#   the other codes are UNVERIFIED
BAUD_METHODS = ("direct", "break")
PARAM_NUMBERS = range(1, 13)  # P1-P12; the driver decides which ones this unit lists / the driver may write
MAX_PARAM_TEXT = 40

POLL_INTERVAL_S = 0.2
POLL_LOCK_WAIT_S = 0.5
COMMAND_LOCK_WAIT_S = 3.0
STOP_LOCK_WAIT_S = 0.25
READ_YIELD_S = 0.005  # a preemptible read pauses this long between requests (= the bus's own min gap) so
#                       that a waiting poll / command gets the bus lock (Python locks are not fair)
STOP_CONFIRM_S = 3.0  # MOVE must clear this soon after STOP, or "STOP NOT CONFIRMED" is raised
PIO_RESTART_WATCH_S = 5.0  # with PIO in control, watch this long after STOP for the axis moving again
RESTART_DRIFT_MM = 0.1  # an "idle" position this far from where the axis stopped counts as moving again
DISCONNECT_CONFIRM_POLL_S = 0.1  # status read period while confirming a disconnect STOP
COMM_LOST_AFTER = 3  # consecutive failed polls
EVENT_LIMIT = 300

PIO_NOTE = (
    "PIO has control (PMSS = 0, as after every power-up). On hardware a Modbus STOP stopped a move commanded "
    "over Modbus with PMSS = 0; it is untested on a move a PLC commanded over PIO, so stop those from the PLC "
    "or with the hardware stop. The first command takes Modbus control while the axis is at rest."
)
PIO_HOLD_REFUSAL = (
    "PIO control was chosen with the PC / PIO switch: the unit follows its PIO inputs, so this command was not sent. "
    "Switch to PC first, then repeat it."
)
PIO_STOP_NOTE = (
    "PIO has control: a Modbus STOP is verified only on Modbus-commanded moves; watching "
    "{watch:g} s for the axis moving again - use the PLC / hardware stop if it does"
)


class ReadInterrupted(BackendError):
    """A read-only read (Unit info, register browser, identify, table rows) ended by STOP (409)."""

    def __init__(self, label: str) -> None:
        super().__init__(f"{label} interrupted by STOP (nothing more was read). Try again.", 409)


@dataclass(frozen=True)
class Limits:
    """Soft limits enforced by the server (and mirrored by the browser). Stroke None: not known yet."""

    stroke_min_mm: float | None
    stroke_max_mm: float | None
    max_speed_mm_s: float = MAX_SPEED_MM_S
    max_accel_g: float = MAX_ACCEL_G
    max_speed_source: str = "GUI cap"
    max_push_pct: float = PUSH_FORCE_MAX_PCT
    min_push_pct: float = PUSH_FORCE_MIN_PCT
    units: str = "mm"  # the axis's position unit: "deg" on a rotary axis (speeds in units/s)


def require_number(value: Any, name: str) -> float:
    """Return ``value`` as a finite float or raise a 400 BackendError naming the field."""
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise BackendError(f"{name} must be a number.", 400)
    try:
        number = float(value)
    except ValueError:
        raise BackendError(f"{name} must be a number.", 400) from None
    if not math.isfinite(number):
        raise BackendError(f"{name} must be a finite number.", 400)
    return number


def check_stroke(min_mm: Any, max_mm: Any) -> tuple[float, float]:
    """Validate a stroke (soft limits) in mm; raise a 400 BackendError if it is not usable."""
    lo, hi = require_number(min_mm, "Stroke min"), require_number(max_mm, "Stroke max")
    if not 0 <= lo < hi <= MAX_STROKE_MM:
        raise BackendError(f"Stroke must satisfy 0 <= min < max <= {MAX_STROKE_MM:g} mm (got {lo:g}-{hi:g}).", 400)
    return lo, hi


def parse_stroke(text: str) -> tuple[float, float]:
    """Parse ``"MIN,MAX"`` in mm (e.g. ``"0,150"``)."""
    parts = text.split(",")
    if len(parts) != 2:
        raise BackendError(f"Stroke must be MIN,MAX in mm, e.g. 0,150 (got {text!r}).", 400)
    return check_stroke(parts[0].strip(), parts[1].strip())


class Controller:
    """Owns at most one AxisSession and every access to it.

    Args:
        session_factory: opens an axis (default :func:`backend.open_session`; tests pass a fake).
        poll_interval_s: status polling period.
        stroke: default stroke override (mm) for connections that do not give their own; ``None``
            (the default) uses each EC's own soft limits.
        sim_only: demo mode (``--sim-only``): only simulated connections are accepted, no serial port is opened.
        stop_confirm_s: how long after STOP the axis may still report MOVE before an alert.
        pio_restart_watch_s: with PIO in control, how long after STOP a renewed motion raises
            "AXIS MOVED AGAIN AFTER STOP" (a Modbus STOP is untested on PLC-commanded moves).
    """

    def __init__(
        self,
        *,
        session_factory: SessionFactory = open_session,
        poll_interval_s: float = POLL_INTERVAL_S,
        stroke: tuple[float, float] | None = DEFAULT_STROKE,
        stop_confirm_s: float = STOP_CONFIRM_S,
        pio_restart_watch_s: float = PIO_RESTART_WATCH_S,
        sim_only: bool = False,
    ) -> None:
        self._session_factory = session_factory
        self._sim_only = sim_only
        self._poll_interval_s = poll_interval_s
        self._default_stroke = None if stroke is None else check_stroke(*stroke)
        self._stop_confirm_s = stop_confirm_s
        self._pio_restart_watch_s = pio_restart_watch_s
        self._driver_lock = threading.Lock()
        self._state_lock = threading.Lock()  # guards the fields below, never held during I/O
        self._session: AxisSession | None = None
        self._conn: dict[str, Any] = {}
        self._identity: dict[str, Any] | None = None
        self._status: dict[str, Any] | None = None
        self._status_time = 0.0
        self._last_error: str | None = None
        self._poll_failures = 0
        self._control_enabled = False  # this GUI turned "Modbus control on" (verified) during this connection
        self._pmsl_written = False  # this GUI wrote the PMSL coil, even if the EC did not take it
        self._control_at_connect: bool | None = None  # as found, restored on disconnect
        self._pio_hold = False  # the operator chose PIO control: no command of this GUI takes it back by itself
        self._motion_commanded = False  # this GUI sent a motion command during this connection
        self._motion_block: str | None = None  # why move/jog are refused for this connection, if they are
        self._stop_epoch = 0
        self._stop_waiting = 0  # STOPs waiting for the bus lock: the poller leaves it to them
        self._stop_watch: tuple[AxisSession, float] | None = None  # session + deadline of an unconfirmed STOP
        # After a STOP with PIO in control: session, when armed, end of the watch, position once seen stopped
        # (None: not yet). Status reads that began before it was armed are ignored by it.
        self._restart_watch: tuple[AxisSession, float, float, float | None] | None = None
        self._stop_alert: str | None = None
        self._stop_alert_kind: str | None = None  # "unconfirmed" (clears once idle) or "restart" (stays)
        self._finish_watch: tuple[float, float | None] | None = None  # (since, position) of an idle axis settling
        self._push: dict[str, Any] | None = None  # last push of this connection and its outcome
        self._unit_mode: dict[str, Any] | None = None  # position mode from the last unit info read
        self._last_reset: dict[str, Any] | None = None  # outcome of this connection's last software reset
        self._busy_op: str | None = None  # a software reset / baud change holding the bus (for STOP's message)
        self._events: deque[dict[str, Any]] = deque(maxlen=EVENT_LIMIT)
        self._shutdown = threading.Event()
        self._poller = threading.Thread(target=self._poll_loop, name="ec-status-poller", daemon=True)
        self._driver_log = _EventLogHandler(self)

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        """Start the background status poller."""
        self.log("info", "GUI server started")
        logging.getLogger("iai_actuator_driver").addHandler(self._driver_log)
        self._poller.start()

    def close(self) -> None:
        """Stop polling and disconnect (same STOP / release rules as :meth:`disconnect`)."""
        self._shutdown.set()
        if self._session is not None:
            try:
                self.disconnect()
            except BackendError as exc:
                self.log("error", f"Disconnect at shutdown failed: {exc}")
                try:
                    self.stop()  # at least try to leave the axis stopped
                except BackendError:
                    pass
        if self._poller.is_alive():
            self._poller.join(timeout=2)
        logging.getLogger("iai_actuator_driver").removeHandler(self._driver_log)

    @property
    def limits(self) -> Limits:
        session = self._session
        stroke = session.stroke if session else self._default_stroke
        lo, hi = stroke if stroke is not None else (None, None)
        speed, source = MAX_SPEED_MM_S, "GUI cap"
        if session is not None and session.max_speed_mm_s < speed:
            speed, source = session.max_speed_mm_s, "this EC's maximum speed (MAXV, item 7.4)"
        # Explicit None checks: a session cap of 0 is a real value (0 % push = this unit cannot push), not "unknown".
        session_accel = None if session is None else getattr(session, "accel_cap_g", None)
        accel = MAX_ACCEL_G if session_accel is None else min(MAX_ACCEL_G, max(0.0, float(session_accel)))
        session_push = None if session is None else getattr(session, "max_push_pct", None)
        push = PUSH_FORCE_MAX_PCT if session_push is None else min(PUSH_FORCE_MAX_PCT, max(0.0, float(session_push)))
        units = getattr(session, "units", "mm") if session is not None else "mm"
        return Limits(lo, hi, max_speed_mm_s=speed, max_accel_g=accel, max_speed_source=source, max_push_pct=push,
                      min_push_pct=min(PUSH_FORCE_MIN_PCT, push), units=units if units in ("mm", "deg") else "mm")

    # ------------------------------------------------------------------ events / snapshot

    def log(self, level: str, message: str) -> None:
        """Append an event to the in-memory log shown in the GUI."""
        now = time.time()
        stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now % 1 * 1000):03d}"
        with self._state_lock:
            self._events.append({"ts": now, "t": stamp, "level": level, "msg": message})

    def is_connected(self) -> bool:
        """Whether an axis is connected on the single-axis page."""
        return self._session is not None

    def snapshot(self, event_count: int = 80) -> dict[str, Any]:
        """Everything GET /api/status returns."""
        with self._state_lock:
            connected = self._session is not None
            age = time.monotonic() - self._status_time if self._status is not None else None
            return {
                "connected": connected,
                "connection": dict(self._conn) if connected else None,
                "status": dict(self._status) if connected and self._status else None,
                "status_age_s": round(age, 3) if age is not None else None,
                "comm_ok": connected and self._poll_failures < COMM_LOST_AFTER,
                "last_error": self._last_error,
                "stop_alert": self._stop_alert if connected else None,
                "motion_block": self._motion_block if connected else None,
                "pio_hold": connected and self._pio_hold,
                "push": _public(self._push) if connected and self._push else None,
                "swreset": dict(self._last_reset) if connected and self._last_reset else None,
                "limits": asdict(self.limits),
                "default_stroke": list(self._default_stroke) if self._default_stroke else None,
                "events": list(self._events)[-event_count:][::-1],
            }

    # ------------------------------------------------------------------ connect

    def connect(
        self, port: str, baud: int, slave: int, simulate: bool, stroke: tuple[float, float] | None = None,
        sim_profile: str = DEFAULT_SIM_PROFILE,
    ) -> dict[str, Any]:
        """Open the axis and read status, identity, soft limits, speed limit and position table. Read-only.

        ``stroke`` (mm) is an optional override, default the server's ``--stroke`` (itself default
        None: the EC's own soft limits minus their 0.30 mm margin). An override can only narrow the
        EC's limits, and is cross-checked against the position table. ``sim_profile`` picks the
        simulated unit (``"slider"`` / ``"gripper"``) when ``simulate`` is set.
        """
        port = port.strip()
        if not simulate and self._sim_only:
            raise BackendError("Simulator-only mode (--sim-only): serial ports are not opened. Tick Simulate.", 409)
        if not simulate:
            port = check_port_name(port)
        if baud not in BAUD_RATES:
            raise BackendError(f"Baud must be one of {', '.join(map(str, BAUD_RATES))}.", 400)
        if not 1 <= slave <= 247:
            raise BackendError("Slave address must be 1-247.", 400)
        if sim_profile not in SIM_PROFILE_NAMES:
            raise BackendError(f"Simulator profile must be one of {', '.join(SIM_PROFILE_NAMES)}.", 400)
        configured = self._default_stroke if stroke is None else check_stroke(*stroke)
        with self._acquire("Connect"):
            if self._session is not None:
                raise BackendError("Already connected. Disconnect first.", 409)
            target = f"simulator ({sim_profile} profile)" if simulate else f"{port} @ {baud} baud"
            stroke_text = ("the EC's own soft limits" if configured is None
                           else f"override {configured[0]:g}-{configured[1]:g} mm")
            self.log("info", f"Connecting to {target}, slave {slave}, stroke: {stroke_text}")
            try:
                session = self._session_factory(port, baud, slave, simulate, stroke=configured,
                                                sim_profile=sim_profile)
            except BackendError as exc:
                self._fail(str(exc))
                raise
            except Exception as exc:
                raise self._fail(f"Could not open {target}: {_describe(exc)}", 502) from exc
            try:
                status = session.status()
            except Exception as exc:
                _close_quietly(session)
                raise self._fail(f"No valid status reply from slave {slave} on {target}: {_describe(exc)}", 502) from exc
            try:
                identity: dict[str, Any] | None = session.identify()
            except Exception as exc:
                identity = None
                self.log("warn", f"Identify failed (continuing): {_describe(exc)}")
            stroke_info, block = _cross_check_stroke(session, configured)
            with self._state_lock:
                self._session = session
                self._conn = {"port": None if simulate else port, "baud": session.baud, "requested_baud": baud,
                              "line_baud": getattr(session, "line_baud", session.baud),
                              "slave": slave, "simulate": simulate, "sim_profile": sim_profile if simulate else None,
                              "driver": session.description, "stroke": stroke_info}
                self._last_reset = None
                self._identity = identity
                self._unit_mode = None
                self._status, self._status_time = status, time.monotonic()
                self._poll_failures, self._last_error = 0, None
                self._control_enabled = self._pmsl_written = self._pio_hold = False
                self._control_at_connect = status.get("modbus_control")
                self._motion_commanded = False
                self._motion_block = block
                self._stop_watch, self._restart_watch, self._finish_watch = None, None, None
                self._stop_alert, self._stop_alert_kind = None, None
                self._push = None
            self.log("info", f"Connected: {session.description}")
            self.log("info", _describe_stroke(stroke_info))
            for warning in stroke_info["warnings"]:
                self.log("warn", warning)
            if status.get("modbus_control") is False:
                self.log("info", PIO_NOTE)
            return {"connection": dict(self._conn), "identity": identity}

    def disconnect(self) -> None:
        """Close the axis, leaving it stopped and as found.

        A fresh status is read first. STOP is sent if the axis is moving, if that read fails, or if
        this GUI commanded motion during the connection (a decel stop is harmless when idle), and
        the axis must then report idle within ``stop_confirm_s`` or "STOP NOT CONFIRMED" is logged
        (the poller cannot check it after disconnect). Then Modbus control is handed back to PIO
        if this GUI took it, and a raised HOME coil is released.
        """
        with self._acquire("Disconnect"):
            session = self._require_session()
            self._stop_before_disconnect(session)
            if self._pmsl_written and self._control_at_connect is False:
                # Leave the EC as found: hand command authority back to PIO.
                try:
                    session.release_control()
                    self.log("info", "Modbus control released (as found at connect)")
                except Exception as exc:
                    self.log("error", f"Releasing Modbus control failed: {_describe(exc)}")
            with self._state_lock:
                self._session = None
                self._pio_hold = False
                self._status, self._identity, self._conn, self._unit_mode = None, None, {}, None
                self._last_reset = None
                self._motion_block, self._stop_watch, self._restart_watch = None, None, None
                self._stop_alert, self._stop_alert_kind = None, None
                self._push = None
            try:
                session.close()
            except Exception as exc:
                self.log("error", f"Closing the connection failed: {_describe(exc)}")
            self.log("info", "Disconnected")

    def _stop_before_disconnect(self, session: AxisSession) -> None:
        """Send STOP unless a fresh read shows the axis idle and this GUI never moved it."""
        status: dict[str, Any] | None
        try:
            status = session.status()
        except Exception as exc:
            status, reason = None, f"status read failed ({_describe(exc)})"
        else:
            moving = status.get("moving")
            reason = None if moving is False else ("axis is moving" if moving else "motion state unknown")
        if reason is None and self._motion_commanded:
            reason = "this GUI commanded motion during the connection"
        if reason is None:
            return
        # Sent with PIO in control too: STOP stopped a Modbus-commanded move with PMSS = 0 (test 19). On a
        # PLC-commanded move it is untested; the confirmation below and its message cover that case.
        try:
            session.stop()
        except Exception as exc:
            self.log("error", f"STOP before disconnect failed: {_describe(exc)}. Use the hardware stop if the axis moves.")
            return
        idle = status is not None and status.get("moving") is False
        self.log("info" if idle else "warn", f"STOP sent before disconnect: {reason}")
        if not idle:
            self._confirm_stopped_before_disconnect(session)

    def _confirm_stopped_before_disconnect(self, session: AxisSession) -> None:
        """Poll until the axis reports idle after a disconnect STOP; log STOP NOT CONFIRMED otherwise."""
        deadline = time.monotonic() + self._stop_confirm_s
        failures, problem = 0, "no status read"
        while True:
            try:
                status = session.status()
            except Exception as exc:
                failures += 1
                problem = f"no status reply ({_describe(exc)})"
                if failures >= COMM_LOST_AFTER:
                    break
            else:
                if status.get("moving") is False:
                    self.log("info", "Axis stopped before disconnect")
                    return
                pos = status.get("position_mm")
                where = f" at {pos:.2f} mm" if isinstance(pos, (int, float)) and not isinstance(pos, bool) else ""
                problem = ("the axis is still moving" if status.get("moving") else
                           "the axis does not report whether it moves") + where
            if time.monotonic() >= deadline:
                break
            time.sleep(DISCONNECT_CONFIRM_POLL_S)
        message = (f"STOP NOT CONFIRMED before disconnect: {problem} {self._stop_confirm_s:g} s after STOP. "
                   "Use the PIO / hardware stop if the axis moves.")
        with self._state_lock:
            self._last_error = message
        self.log("error", message)

    # ------------------------------------------------------------------ read-only queries

    def identify(self) -> dict[str, Any]:
        """Re-read identity from the axis (read-only; the bus lock is taken per request, STOP ends it)."""
        try:
            with self._preemptible("Identify") as session:
                identity = session.identify()
        except BackendError:
            raise  # not connected, connection changed, bus busy or STOP: nothing to blame the EC for
        except Exception as exc:
            raise self._fail(f"Identify failed: {_describe(exc)}", 502) from exc
        with self._state_lock:
            if self._session is session:
                self._identity = identity
        return identity

    def positions(self) -> list[dict[str, Any]]:
        """Position table rows 0-2 (Backward / Forward / Middle). Read-only (bus lock per request)."""
        rows = []
        with self._preemptible("Read position table") as session:
            for n, label in TABLE_LABELS.items():
                try:
                    row = session.table_row(n)
                except BackendError:
                    raise
                except Exception as exc:
                    raise self._fail(f"Reading position table row {n} failed: {_describe(exc)}", 502) from exc
                rows.append({"n": n, "label": label, **row})
        return rows

    def unit_info(self) -> dict[str, Any]:
        """Everything the EC tells about itself (``Elecylinder.unit_info``). Read-only: FC03 + FC11 only.

        Read in ``UNIT_INFO_GROUPS`` and merged into one unit_info dict (``meta.gui`` says how). The
        bus lock is taken per Modbus request (:meth:`_preemptible`), so the status poll, commands and
        STOP get the bus between any two requests, and STOP ends the read. A failure (or STOP) in the
        first group is an error; a later one ends the read and is listed in ``errors`` with the
        sections read so far.
        """
        started = time.monotonic()
        with self._state_lock:
            epoch = self._stop_epoch  # one STOP ends the whole read, also between two groups
        parts: list[dict[str, Any]] = []
        failures: list[str] = []
        first: AxisSession | None = None
        for group in UNIT_INFO_GROUPS:
            try:
                with self._preemptible("Unit info", epoch=epoch, expected=first) as session:
                    first = session
                    parts.append(session.unit_info(group))
            except BackendError as exc:  # STOP, not connected, connection changed or bus busy
                if isinstance(exc, ReadInterrupted):
                    self.log("info", f"Unit info read interrupted by STOP after {len(parts)} of "
                                     f"{len(UNIT_INFO_GROUPS)} groups")
                if not parts:
                    raise
                failures.append(f"{', '.join(group)}: {exc}")
                break
            except Exception as exc:
                if not parts:
                    raise self._fail(f"Unit info failed: {_describe(exc)}", error_status(exc)) from exc
                failures.append(f"{', '.join(group)}: {_describe(exc)}")
                break
        wall = time.monotonic() - started
        info = _merge_unit_info(parts, failures, wall)
        mode = (info.get("identity") or {}).get("position_mode")
        with self._state_lock:
            if self._session is first and isinstance(mode, dict):
                self._unit_mode = mode  # the live view decodes DIPM / DOPM with this layout
        errors = len(info["errors"])
        self.log("warn" if errors else "info",
                 f"Unit info read (read-only, {len(parts)} of {len(UNIT_INFO_GROUPS)} groups, bus lock per "
                 f"request, {wall:.2f} s)" + (f" with {errors} error(s): {info['errors'][0]}" if errors else ""))
        return info

    def unit_live(self) -> dict[str, Any]:
        """The live Unit info sections (monitor, status words, I/O) from the poller's latest status words.

        No bus access: the words come from the ~5 Hz status poll and are decoded with the driver's bit
        tables (DIPM / DOPM with the position mode from the last unit info read). Words the poll does
        not read (POSS, SSSE, OLLV, ...) are only in the full unit info.
        """
        with self._state_lock:
            if self._session is None:
                raise BackendError("Not connected.", 409)
            status = self._status
            age = time.monotonic() - self._status_time if status is not None else None
            mode, model = self._unit_mode, (self._identity or {}).get("model")
            comm_ok = self._poll_failures < COMM_LOST_AFTER
        raw = (status or {}).get("raw") or {}
        live = decode_live(raw, mode, model if isinstance(model, str) else None) if raw else None
        return {
            "live": live,
            "status_age_s": round(age, 3) if age is not None else None,
            "comm_ok": comm_ok,
            "mode_source": "unit info" if mode is not None else "model string / assumed (Refresh to read P9)",
        }

    def read_block(self, class_id: int, offset: int = 0, count: int | None = None) -> dict[str, Any]:
        """Raw words of one directory block for the register browser. Read-only: FC03 inside the EC's directory.

        The bus lock is taken per request (:meth:`_preemptible`): STOP ends a long read after one short request.
        """
        try:
            with self._preemptible("Read registers") as session:
                return session.read_block(class_id, offset, count)
        except BackendError:
            raise  # refused before anything was read (not in the directory / out of range), or STOP / busy
        except Exception as exc:
            raise self._fail(f"Reading class 0x{class_id:04X} failed: {_describe(exc)}", error_status(exc)) from exc

    @contextlib.contextmanager
    def _preemptible(
        self, label: str, *, epoch: int | None = None, expected: AxisSession | None = None
    ) -> Iterator[AxisSession]:
        """Run read-only session calls with the bus lock taken per Modbus request, not for the whole call.

        Every request of the call (``AxisSession.preemptible``) is sent inside ``gate()``: it raises
        :class:`ReadInterrupted` once STOP was pressed after ``epoch`` (default: now), so after STOP
        not one more request is sent, then takes the bus lock for that one short request only. STOP
        therefore waits for at most one in-flight request, and the poll and commands get the bus
        between two requests. ``expected``: the session an earlier part of the same read used.
        """
        with self._state_lock:
            session = self._session
            if epoch is None:
                epoch = self._stop_epoch
        if session is None:
            raise BackendError("Not connected.", 409)
        if expected is not None and session is not expected:
            raise BackendError(f"{label}: the connection changed while reading. Try again.", 409)

        def check() -> None:
            if self._stop_epoch != epoch:
                raise ReadInterrupted(label)
            if self._session is not session:
                raise BackendError(f"{label}: the connection changed while reading. Try again.", 409)

        @contextlib.contextmanager
        def gate() -> Iterator[None]:
            check()
            if not self._driver_lock.acquire(timeout=COMMAND_LOCK_WAIT_S):
                raise BackendError(f"{label}: driver busy, try again.", 409)
            try:
                check()  # STOP may have been pressed while this request waited for the bus
                yield
            finally:
                self._driver_lock.release()
                time.sleep(READ_YIELD_S)  # let a waiting poll / command / STOP take the bus first

        with session.preemptible(gate):
            yield session

    # ------------------------------------------------------------------ commands

    def servo(self, on: bool) -> None:
        """Servo ON/OFF."""
        self._command(f"Servo {'ON' if on else 'OFF'}", lambda s, _st: s.servo(on))

    def home(self) -> None:
        """Start a home return (does not wait for completion)."""

        def run(session: AxisSession, status: dict[str, Any]) -> None:
            _require_ready(status, need_homed=False)
            self._end_push("superseded by Home")
            session.home()

        self._command("Home", run, motion=True)

    def set_control(self, pc: bool) -> dict[str, Any]:
        """Choose who has command authority: ``pc`` True = Modbus (PMSL on, PMSS follows), False = PIO (PMSL off).

        Only at rest: a fresh status is read first and the switch is refused while the axis moves, homes, is paused or
        pushes (PMSL ON mid-move stops the EC's move, hardware test 18). Taking PC control from PIO allows a pause at a
        standstill with the servo off (a PCON in PIO mode with nothing on its PIO connector pauses at rest; the driver
        checks that nothing moves); a pause with the servo on is a hold and stays a refusal. Choosing PIO stays until
        PC is chosen again: no other command of this GUI takes control back by itself while it is on (they are
        refused with the reason).
        Returns the control state read back afterwards; it is an error when the unit does not follow."""
        label = "Control -> PC" if pc else "Control -> PIO"
        with self._acquire(label):
            session = self._require_session()
            try:
                status = session.status()
            except Exception as exc:
                raise self._fail(f"{label} failed: {_describe(exc)}", error_status(exc)) from exc
            # PIO to PC: a pause at a standstill with the servo off is no refusal (a PCON with nothing on its PIO connector)
            pause_ok = pc and status.get("modbus_control") is False and status.get("servo_on") is False
            busy = _motion_in_progress(status, pause_ok=pause_ok)
            if busy:
                raise self._fail(
                    f"{label} refused: the axis is {busy}. The control mode is only switched at rest "
                    "(switching it during motion stops or changes the move). Press STOP, or wait.", 409)
            was = status.get("modbus_control")
            try:
                if pc:
                    if was is not True:
                        self._pmsl_written = True  # before writing: a failed verify may still have latched PMSL
                        session.enable_control()
                        self._control_enabled = True
                else:
                    if was is not False:
                        session.release_control()
                        self._control_enabled = False
                        self._pmsl_written = False  # nothing is left for disconnect to hand back
            except Exception as exc:
                raise self._fail(f"{label} failed: {_describe(exc)}", error_status(exc)) from exc
            with self._state_lock:
                self._pio_hold = not pc
            try:
                now = session.status()
            except Exception as exc:
                raise self._fail(f"{label} was sent but the status read after it failed: {_describe(exc)}", 502) from exc
            self._record_status(session, dict(now))
            state = now.get("modbus_control")
            if state is not None and state != pc:
                raise self._fail(
                    f"{label}: the unit still reports {'PC' if state else 'PIO'} control (PMSS = {int(bool(state))}) "
                    "after the switch.", 502)
            self.log("info", f"Control mode: {'PC (Modbus)' if pc else 'PIO'}" + (" (already)" if was == pc else ""))
            return {"modbus_control": state, "pio_hold": not pc, "changed": was != pc}

    def reset_alarm(self) -> None:
        """Alarm reset."""
        self._command("Alarm reset", lambda s, _st: s.reset_alarm())

    def run_stored(self, n: int, allow_above_caps: bool = False) -> None:
        """Run stored position ``n`` (0-2) with 0x9800 = n at the row's own stored speed (does not wait).

        The driver refuses a row outside the stroke, and (unless ``allow_above_caps``, which the
        browser asks for in its confirm dialog) one whose stored speed / acceleration exceed the
        driver's caps; it raises if the EC ignores the command (no motion within 0.5 s).
        """
        self._require_motion_allowed()
        if isinstance(n, bool) or n not in TABLE_LABELS:
            raise BackendError("Stored position must be 0, 1 or 2.", 400)

        def run(session: AxisSession, status: dict[str, Any]) -> None:
            _require_ready(status, need_homed=True)
            self._end_push("superseded by a stored-position move")
            session.run_stored(n, allow_above_caps)

        extra = " at its own speed / accel (above the caps, confirmed)" if allow_above_caps else ""
        self._command(f"Run stored position {n} ({TABLE_LABELS[n]}){extra}", run, motion=True)

    def write_param(
        self, no: int, value: Any, allow_home_params: bool = False, allow_unverified_model: bool = False
    ) -> dict[str, Any]:
        """Write user parameter P``no`` = ``value``, verified and committed (PARW, the commit IA-OS sends).

        Permanent (non-volatile); the EC applies it only after a software reset or a power cycle. The
        driver refuses parameters it has not confirmed; P3 / P4 (home) and one not on this unit's parameter list are
        written with a warning in the result. Every unit's parameters are written the same way (``allow_home_params``
        and ``allow_unverified_model`` are kept for compatibility and ignored). A value the unit already holds is
        neither written nor committed (``skipped``; the log says so: the non-volatile memory lasts about 100,000 write
        cycles, IAI). Returns ``{"result": ParameterWriteResult dict}``.
        """
        if isinstance(no, bool) or not isinstance(no, int) or no not in PARAM_NUMBERS:
            raise BackendError("Parameter number must be 1-12.", 400)
        if isinstance(value, bool) or not isinstance(value, (int, float, str)) or len(str(value)) > MAX_PARAM_TEXT:
            raise BackendError("Parameter value must be a number or a label such as 1:Forward.", 400)

        def run(session: AxisSession, status: dict[str, Any]) -> dict[str, Any]:
            _require_at_rest(status, "Parameters are written")
            if allow_unverified_model:
                return session.write_param(no, value, allow_home_params, allow_unverified_model=True)
            return session.write_param(no, value, allow_home_params)

        result = self._command(f"Write P{no} = {value} (committed)", run)
        if result.get("skipped"):
            self.log("info", f"P{no} {result.get('name', '')} = {result.get('after')}: {result.get('note')}")
        elif result.get("commit_confirmed", True):
            self.log("warn", f"P{no} {result.get('name', '')}: {result.get('before')} -> {result.get('after')}, committed "
                             "to non-volatile memory; it takes effect after a software reset or a power cycle")
        else:
            self.log("error", f"P{no} {result.get('name', '')}: {result.get('before')} -> {result.get('after')}, but the "
                              "commit was NOT confirmed (the EC never showed PARW busy): it may be RAM only")
        for warning in result.get("warnings") or []:
            self.log("warn", f"P{no}: {warning}")
        return {"result": result}

    def software_reset(self) -> dict[str, Any]:
        """Software reset with the frames IA-OS sends (servo OFF, then 0x45 cmd 5), link re-synced in place.

        Committed parameters take effect. Afterwards PMSS = 0 (the next command takes control again),
        safety speed off, the servo back on, and an incremental unit has lost its home ("needs home",
        kept in the snapshot's ``swreset`` until a home return). Returns ``{"result": ...}``.
        """

        def run(session: AxisSession, status: dict[str, Any]) -> dict[str, Any]:
            _require_at_rest(status, "A software reset is done")
            self._end_push("ended by a software reset")
            self._busy_op = "a software reset"
            try:
                result = session.software_reset()
            finally:
                self._busy_op = None
            with self._state_lock:
                if self._session is session:
                    self._conn["line_baud"] = getattr(session, "line_baud", session.baud)
                    if not self._conn.get("simulate"):
                        self._conn["baud"] = session.baud
            return result

        result = self._command("Software reset", run)
        stamp = time.strftime("%H:%M:%S")
        summary = {k: result.get(k) for k in ("accepted", "reply_lost", "baudrate", "modbus_enabled", "safety_speed",
                                              "servo_on", "homed", "needs_home", "alarm_code", "elapsed_s")}
        with self._state_lock:
            self._last_reset = {**summary, "at": stamp, "notes": list(result.get("notes") or [])}
        self.log("warn" if result.get("needs_home") else "info",
                 f"Software reset done: PMSS {int(bool(result.get('modbus_enabled')))}, SFTY "
                 f"{int(bool(result.get('safety_speed')))}, servo {'ON' if result.get('servo_on') else 'OFF'}, HEND "
                 f"{int(bool(result.get('homed')))}" + (" - NEEDS HOME before the next move" if result.get("needs_home")
                                                         else ""))
        return {"result": result}

    def set_baud(self, rate: int, method: str = "direct") -> dict[str, Any]:
        """Change the EC's baud rate with the broadcast 0x45 cmd 4 (the frame IA-OS sends); the port follows.

        Volatile: the EC answers at 38400 again after a power-up. Every EC on the line switches.
        ``method``: "direct" (broadcast at the current rate; VERIFIED) or "break" (200 ms break, 9600,
        broadcast; UNVERIFIED). Returns ``{"result": BaudChangeResult dict}``.
        """
        if isinstance(rate, bool) or rate not in BAUD_CHANGE_RATES:
            raise BackendError(
                f"New baud must be one of {', '.join(map(str, BAUD_CHANGE_RATES))} (115200 and 230400 are verified on "
                "hardware; the EC-GRBP8M gripper and the PCON-CB take the 38400 code, the EC-R6S rod ignores it and "
                "the driver checks after the switch; the other codes are untested). A power cycle brings a unit back "
                "to 38400.", 400)
        if method not in BAUD_METHODS:
            raise BackendError("Method must be 'direct' or 'break'.", 400)

        def run(session: AxisSession, status: dict[str, Any]) -> dict[str, Any]:
            _require_at_rest(status, "The baud rate is changed")
            self._busy_op = "a baud-rate change"
            try:
                result = session.set_baudrate(rate, method)
            finally:
                self._busy_op = None
            with self._state_lock:
                if self._session is session:
                    self._conn["line_baud"] = getattr(session, "line_baud", rate)
                    if not self._conn.get("simulate"):
                        self._conn["baud"] = session.baud
            return result

        result = self._command(f"Baud rate -> {rate} ({method})", run)
        self.log("warn", f"Baud rate now {rate} (volatile: 38400 again after a power-up; every EC on the line switched)")
        return {"result": result}

    def move(self, position_mm: float, speed_mm_s: float, accel_g: float) -> None:
        """Absolute move (does not wait). Validated against stroke, speed and accel limits."""
        self._require_motion_allowed()
        limits = self.limits
        _check_position(position_mm, limits)
        _check_profile(speed_mm_s, accel_g, limits)

        def run(session: AxisSession, status: dict[str, Any]) -> None:
            _require_ready(status, need_homed=True)
            self._end_push("superseded by a move")
            session.move_to(position_mm, speed_mm_s, accel_g)

        self._command(f"Move to {position_mm:.2f} mm @ {speed_mm_s:g} mm/s, {accel_g:g} G", run, motion=True)

    def jog(self, delta_mm: float, speed_mm_s: float, accel_g: float = DEFAULT_ACCEL_G) -> None:
        """Relative move (driver ``jog``, never the JOG coils); rejected if the target leaves the stroke."""
        self._require_motion_allowed()
        limits = self.limits
        lo, hi = _stroke_of(limits)
        if delta_mm == 0 or abs(delta_mm) > hi - lo:
            raise BackendError("Jog distance must be non-zero and smaller than the stroke.", 400)
        _check_profile(speed_mm_s, accel_g, limits)

        def run(session: AxisSession, status: dict[str, Any]) -> None:
            _require_ready(status, need_homed=True)
            current = status.get("position_mm")
            if not isinstance(current, (int, float)):
                raise BackendError("Current position is unknown; cannot jog.", 409)
            target = round(current + delta_mm, 2)
            if not lo <= target <= hi:
                raise BackendError(
                    f"Jog {delta_mm:+g} mm from {current:.2f} mm would reach {target:.2f} mm, outside the stroke "
                    f"{lo:g}-{hi:g} mm.", 400)
            self._end_push("superseded by a jog")
            session.jog(delta_mm, speed_mm_s, accel_g)
            self.log("info", f"Jog target {target:.2f} mm")

        self._command(f"Jog {delta_mm:+g} mm @ {speed_mm_s:g} mm/s", run, motion=True)

    def push(
        self, target_mm: float, band_mm: float, force_pct: float, direction: int, speed_mm_s: float,
        accel_g: float = DEFAULT_ACCEL_G,
    ) -> None:
        """Push move (does not wait): approach ``target_mm``, then push over ``band_mm`` at ``force_pct``.

        ``direction`` +1 pushes away from home, -1 toward home. The outcome (contact / missed) is
        read from the status flags as the poller sees them and shown in the snapshot's ``push``.
        """
        self._require_motion_allowed()
        limits = self.limits
        lo, hi = _stroke_of(limits)
        if direction not in (1, -1):
            raise BackendError("Direction must be +1 (away from home) or -1 (toward home).", 400)
        if limits.max_push_pct <= 0:
            raise BackendError("Push refused: this unit cannot push (its push cap is 0 %). Nothing was sent.", 400)
        if not limits.min_push_pct <= force_pct <= limits.max_push_pct:
            cap = " (this EC's MXPP, item 7.9)" if limits.max_push_pct < PUSH_FORCE_MAX_PCT else ""
            raise BackendError(f"Push force must be {limits.min_push_pct:g}-{limits.max_push_pct:g} %{cap}.", 400)
        if not 0 < band_mm <= hi - lo:
            raise BackendError("Push band must be > 0 and no longer than the stroke.", 400)
        _check_position(target_mm, limits)
        end = round(target_mm + direction * band_mm, 2)
        if not lo <= end <= hi:
            raise BackendError(f"Push band end {end:.2f} mm is outside the stroke {lo:g}-{hi:g} mm.", 400)
        _check_profile(speed_mm_s, accel_g, limits)
        toward = "away from home" if direction > 0 else "toward home"

        def run(session: AxisSession, status: dict[str, Any]) -> None:
            _require_ready(status, need_homed=True)
            self._end_push("superseded by a new push")
            session.push(target_mm, band_mm, force_pct, direction, speed_mm_s, accel_g)
            with self._state_lock:
                self._push = {
                    "state": "running", "target_mm": target_mm, "band_mm": band_mm, "end_mm": end,
                    "force_pct": force_pct, "direction": direction, "speed_mm_s": speed_mm_s,
                    "start_mm": status.get("position_mm"), "position_mm": None, "message": "Pushing...",
                    "_started": time.monotonic(), "_seen_start": False, "_idle_since": None, "_outcome": None,
                }

        self._command(
            f"Push {toward}: approach {target_mm:.2f} mm, band {band_mm:g} mm to {end:.2f} mm, {force_pct:g} %, "
            f"{speed_mm_s:g} mm/s (pushing at {min(speed_mm_s, PUSH_SPEED_MAX_MM_S):g} mm/s)", run, motion=True)

    def stop(self, take_control: bool = False) -> None:
        """Decelerate to stop. Bypasses the command queue and is retried once on failure.

        STOP is sent whether or not Modbus has command authority and does not touch PMSL. With
        PMSS = 0 it stopped a move commanded over Modbus on hardware (test 19); on a move a PLC
        commanded over PIO it is untested. While PIO has control the axis is therefore watched for
        ``pio_restart_watch_s`` after STOP, and "AXIS MOVED AGAIN AFTER STOP" is raised if it moves
        (for example a PLC still holding its start input once STOP is released). With
        ``take_control``, command authority is switched to Modbus *after* the stop, once the axis
        is at rest (PMSL is never switched during motion: that stops a move too); it is handed back
        to PIO on disconnect. The following status reads must show the axis idle within
        ``stop_confirm_s``, or "STOP NOT CONFIRMED" is raised.

        Read-only reads (Unit info, register browser) never hold the bus lock for more than one short
        request and stop at once; the status poll leaves the lock to a waiting STOP. The restart watch
        is armed with the standstill the driver's STOP hold itself read (``AxisSession.stop`` returns
        it), so it works even when another command kept the bus lock.
        """
        with self._state_lock:
            self._stop_epoch += 1  # queued commands are dropped, running read-only reads end
            self._stop_waiting += 1  # the poller leaves the bus lock to this STOP
            session = self._session
            control = (self._status or {}).get("modbus_control")  # who had control when STOP was pressed
        try:
            if session is None:
                self.log("warn", "STOP pressed while not connected")
                raise BackendError("Not connected: nothing to stop.", 409)
            locked = self._driver_lock.acquire(timeout=STOP_LOCK_WAIT_S)
        finally:
            with self._state_lock:
                self._stop_waiting -= 1
        control_error: str | None = None
        try:
            errors = []
            final: dict[str, Any] | None = None
            for _attempt in range(2):
                try:
                    final = session.stop()  # blocks until the axis has stopped (at most backend.STOP_HOLD_S)
                    break
                except Exception as exc:
                    errors.append(_describe(exc))
            else:
                busy = f" ({self._busy_op} was in progress: the EC may not answer for a moment)" if self._busy_op else ""
                raise self._fail("STOP FAILED: " + " | ".join(errors) + busy + ". Use the hardware stop / power off.", 502)
            final = final if isinstance(final, dict) else None
            with self._state_lock:
                self._stop_watch = (session, time.monotonic() + self._stop_confirm_s)
                self._stop_alert, self._stop_alert_kind = None, None
                self._restart_watch = None  # re-armed below for this STOP
            self._end_push("stopped by STOP", state="stopped")
            if final is not None:
                if control is None and final.get("modbus_control") is False:
                    control = False
                self._record_status(session, dict(final))  # where STOP left the axis, also without the bus lock
            stopped_at = _standstill(final)
            if control is False:
                with self._state_lock:
                    self._arm_restart_watch(session, stopped_at)
            if locked:
                # Fresh authority / motion state; a failed read falls back to the STOP hold's last status.
                try:
                    read_at = time.monotonic()
                    fresh = session.status()
                    if control is None and fresh.get("modbus_control") is False:
                        control = False
                        with self._state_lock:
                            self._arm_restart_watch(session, stopped_at)
                    self._record_status(session, fresh, read_at)
                except Exception:
                    pass
                if take_control:
                    control_error = self._take_control_at_rest(session)
        finally:
            if locked:
                self._driver_lock.release()
        note = "" if locked else " (while another command held the bus)"
        if control is False:
            note += f" ({PIO_STOP_NOTE.format(watch=self._pio_restart_watch_s)})"
        self.log("warn", "STOP sent" + note)
        if take_control and not locked:
            control_error = "the bus was busy"
        if control_error:
            self.log("warn", f"Taking Modbus control after STOP failed: {control_error}")

    # ------------------------------------------------------------------ internals

    def _arm_restart_watch(self, session: AxisSession, stopped_at: float | None = None) -> None:
        """Watch for the axis moving again after a STOP sent while PIO has control (call under _state_lock).

        Armed only after the driver's STOP sequence has returned, i.e. once the axis was seen stopped
        (or the hold timed out, which "STOP NOT CONFIRMED" also reports), so any motion reported during
        the watch is motion after STOP. ``stopped_at``: the standstill position the STOP hold read
        (None: taken from the first idle status). Status reads that began before arming are ignored.
        """
        if self._pio_restart_watch_s > 0:
            now = time.monotonic()
            self._restart_watch = (session, now, now + self._pio_restart_watch_s, stopped_at)

    def _check_restart(
        self, session: AxisSession, status: dict[str, Any] | None, read_at: float | None = None
    ) -> str | None:
        """Restart watch (call under _state_lock): the alert message if the axis moves again, else None.

        ``read_at``: when the status read began (None: just now). A read that began before the watch
        was armed may show the motion STOP ended, not a restart, and is ignored.
        """
        watch = self._restart_watch
        if watch is None or watch[0] is not session or status is None:
            return None
        _, armed_at, until, stopped_at = watch
        if read_at is not None and read_at < armed_at:
            return None
        if time.monotonic() >= until:
            self._restart_watch = None
            return None
        pos = status.get("position_mm")
        known = isinstance(pos, (int, float)) and not isinstance(pos, bool)
        moving = status.get("moving")
        if moving is False:
            if stopped_at is None:
                if known:
                    self._restart_watch = (session, armed_at, until, float(pos))
                return None
            if not known or abs(pos - stopped_at) <= RESTART_DRIFT_MM:
                return None
        elif moving is not True:
            return None
        self._restart_watch = self._stop_watch = None  # this alert supersedes "STOP NOT CONFIRMED"
        where = f" at {pos:.2f} mm" if known else ""
        since = f" (stopped at {stopped_at:.2f} mm)" if stopped_at is not None else ""
        message = (f"AXIS MOVED AGAIN AFTER STOP{where}{since}: PIO has control (PMSS = 0) and a PLC may still be "
                   "commanding it, e.g. holding its start input. A Modbus STOP is untested on PLC-commanded moves: "
                   "use the PLC / hardware stop.")
        self._stop_alert, self._stop_alert_kind, self._last_error = message, "restart", message
        return message

    def _take_control_at_rest(self, session: AxisSession) -> str | None:
        """Switch command authority to Modbus if the axis is at rest; returns an error text instead of raising."""
        try:
            status = session.status()
        except Exception as exc:
            return _describe(exc)
        if status.get("modbus_control") is True:
            return None
        if status.get("moving") is not False:
            return "the axis is not at rest (PMSL is never switched during motion)"
        self._pmsl_written = True  # before writing: a failed verify may still have latched PMSL
        try:
            session.enable_control()
        except Exception as exc:
            return _describe(exc)
        self._control_enabled, self._pio_hold = True, False
        self.log("info", "Modbus control taken from PIO after STOP, at rest (handed back on disconnect)")
        try:
            self._record_status(session, session.status())
        except Exception:
            pass
        return None

    def _end_push(self, reason: str, state: str = "superseded") -> None:
        """Close the watch on a running push (STOP, or a new motion command)."""
        with self._state_lock:
            push = self._push
            if push is None or push["state"] != "running":
                return
            push.update(state=state, message=f"Push {reason}")
        self.log("warn" if state == "stopped" else "info", f"Push {reason}")

    def _check_push(self, status: dict[str, Any]) -> None:
        """Decide a running push's outcome from the status flags (the rules of ``Elecylinder.push``).

        Contact (PEND + DSSE b0, PSFL 0) and miss (PSFL, stopped) are reported only after the same
        flags have held for ``PUSH_RESULT_SETTLE_S`` with the position unchanged (a contact may creep
        ``PUSH_CREEP_MM``): hardware test G5 showed the contact fingerprint for a moment at 6.76 mm
        during a push that then missed at 7.00 mm.
        """
        with self._state_lock:
            push = self._push
            if push is None or push["state"] != "running":
                return
            now = time.monotonic()
            pos = status.get("position_mm")
            push["position_mm"] = pos
            moving = bool(status.get("moving"))
            push["_seen_start"] = push["_seen_start"] or moving or bool(status.get("move_bit")) or (
                not status.get("in_position") and not status.get("push_missed"))
            seen = push["_seen_start"]
            known = isinstance(pos, (int, float)) and not isinstance(pos, bool)
            where = f"{pos:.2f} mm" if known else "?"
            outcome: str | None = None
            if seen or now - push["_started"] >= PUSH_START_S:
                if status.get("push_missed") and not moving:
                    outcome = "missed"
                elif status.get("push_contact") and not status.get("push_missed"):
                    outcome = "contact"
            candidate = push.get("_outcome")  # (outcome, since, position) still settling
            if outcome is None:
                push["_outcome"] = None
            elif (candidate is None or candidate[0] != outcome or not known or candidate[2] is None
                    or abs(pos - candidate[2]) > PUSH_CREEP_MM):
                push["_outcome"] = (outcome, now, pos if known else None)
            if outcome is not None:
                push["_idle_since"] = None
                if now - push["_outcome"][1] >= PUSH_RESULT_SETTLE_S:
                    if outcome == "missed":
                        push.update(state="missed", message=f"No contact over the band; stopped at {where}")
                    else:
                        push.update(state="contact", message=f"Contact at {where}. The motor keeps pushing until "
                                                              "the next move command: move or jog away to release")
            elif seen or now - push["_started"] >= PUSH_START_S:
                if not seen:
                    push.update(state="failed", message=f"The push did not start within {PUSH_START_S:g} s")
                elif status.get("alarm") or status.get("servo_on") is False:
                    push.update(state="failed", message=f"Push ended by an alarm / servo off at {where}")
                elif not moving:
                    push["_idle_since"] = push["_idle_since"] or now
                    if now - push["_idle_since"] >= PUSH_SETTLE_S:
                        push.update(state="stopped", message=f"Push ended at {where} without contact or miss")
                else:
                    push["_idle_since"] = None
            state, text = push["state"], push["message"]
        if state != "running":
            self.log("info" if state in ("contact", "missed") else "warn", f"Push result: {state.upper()} - {text}")

    def _require_motion_allowed(self) -> None:
        block = self._motion_block
        if block:
            raise BackendError(block, 409)

    def _command(
        self, label: str, action: Callable[[AxisSession, dict[str, Any]], Any], *, motion: bool = False
    ) -> Any:
        """Run one state-changing command under the driver lock, with a fresh status read first.

        Returns what ``action`` returns."""
        with self._state_lock:
            epoch = self._stop_epoch
        with self._acquire(label):
            if self._stop_epoch != epoch:
                self.log("warn", f"{label}: cancelled by STOP")
                raise BackendError(f"{label} cancelled by STOP.", 409)
            session = self._require_session()
            try:
                status = session.status()
                control = status.get("modbus_control")
                if control is False or (control is None and not self._control_enabled):
                    if self._pio_hold:
                        raise BackendError(PIO_HOLD_REFUSAL, 409)
                    if status.get("moving") is not False:
                        # PMSL ON mid-move stops the move on the EC (hardware test 18): never switch it now.
                        raise BackendError(
                            "PIO has control and the axis is moving: Modbus control is only taken while the axis "
                            "is at rest (switching it now would stop the move). Wait, or stop it from the PLC / "
                            "hardware stop (a Modbus STOP is untested on PLC-commanded moves).", 409)
                    self._pmsl_written = True  # before writing: a failed verify may still have latched PMSL
                    session.enable_control()
                    self._control_enabled = True
                    self.log("info", "Modbus control enabled")
                    status = session.status()
                if motion:
                    self._motion_commanded = True  # before sending: a failed send may still have landed
                    with self._state_lock:  # this GUI's own motion is not a restart after STOP
                        self._restart_watch = None
                        if self._stop_alert_kind == "restart":
                            self._stop_alert = self._stop_alert_kind = None
                result = action(session, status)
            except BackendError as exc:
                self.log("error", f"{label}: {exc}")
                raise
            except Exception as exc:
                raise self._fail(f"{label} failed: {_describe(exc)}", error_status(exc)) from exc
            self.log("info", label)
            self._refresh_after_command(session)
            if self._stop_epoch != epoch:
                # STOP arrived while this command held the bus and may have landed before it.
                try:
                    session.stop()
                    self.log("warn", f"STOP re-sent: it was pressed while '{label}' was being sent")
                except Exception as exc:
                    raise self._fail(f"STOP after {label} FAILED: {_describe(exc)}", 502) from exc
            return result

    def _refresh_after_command(self, session: AxisSession) -> None:
        """Re-read status so the GUI shows the command's effect without waiting for the next poll."""
        try:
            read_at = time.monotonic()
            self._record_status(session, session.status(), read_at)
        except Exception as exc:
            self._record_poll_error(exc, session)

    def _acquire(self, label: str) -> "_LockGuard":
        return _LockGuard(self._driver_lock, label, COMMAND_LOCK_WAIT_S)

    def _require_session(self) -> AxisSession:
        if self._session is None:
            raise BackendError("Not connected.", 409)
        return self._session

    def _fail(self, message: str, http_status: int = 409) -> BackendError:
        with self._state_lock:
            self._last_error = message
        self.log("error", message)
        return BackendError(message, http_status)

    def _poll_loop(self) -> None:
        while not self._shutdown.wait(self._poll_interval_s):
            session = self._session
            if session is None or self._stop_waiting or not self._driver_lock.acquire(timeout=POLL_LOCK_WAIT_S):
                continue  # not connected, a STOP is waiting for the bus, or a command is using it: skip this tick
            try:
                if self._session is not session or self._stop_waiting:
                    continue  # a STOP started waiting meanwhile: it goes first
                read_at = time.monotonic()
                try:
                    status = session.status()
                except Exception as exc:
                    self._record_poll_error(exc, session)
                    continue
                # Recorded while the bus is still held, so an older poll can never overwrite the fresher
                # status a command records right after it (the lamps would flicker back for a tick).
                self._record_status(session, status, read_at)
            finally:
                self._driver_lock.release()

    def _record_status(self, session: AxisSession, status: dict[str, Any], read_at: float | None = None) -> None:
        """Make ``status`` the current one; ``read_at`` = when its read began (None: just now)."""
        notes = status.pop("notes", None) or ()
        with self._state_lock:
            if self._session is not session:
                return  # disconnected (or reconnected) while this status was being read
            previous, self._status = self._status, status
            self._status_time = time.monotonic()
            recovered = self._poll_failures >= COMM_LOST_AFTER
            self._poll_failures = 0
            finished = self._motion_finished(previous or {}, status)
        if recovered:
            self.log("info", "Communication restored")
        for note in notes:
            self.log("warn", note)
        for message in _describe_changes(previous or {}, status):
            self.log("warn" if "ALARM" in message or "EMERGENCY" in message else "info", message)
        if finished:
            self.log("info", finished)
        self._check_push(status)
        self._check_stop(session, status, read_at)

    def _motion_finished(self, previous: dict[str, Any], status: dict[str, Any]) -> str | None:
        """'Motion finished at X mm' once the axis is idle *and* its position has settled (call under _state_lock).

        The EC reports PEND / MEND (so ``moving`` False) inside the band before standstill (3.04 -> 3.00 mm
        on the gripper), so the first idle sample can show a position the axis is still leaving.
        """
        moving, pos = status.get("moving"), status.get("position_mm")
        known = isinstance(pos, (int, float)) and not isinstance(pos, bool)
        now = time.monotonic()
        if moving is not False:
            self._finish_watch = None
            return None
        if previous.get("moving") is True:
            self._finish_watch = (now, float(pos) if known else None)
            return None
        watch = self._finish_watch
        if watch is None:
            return None
        if not known or watch[1] is None:
            self._finish_watch = None
            return "Motion finished"
        if abs(pos - watch[1]) > SAME_POSITION_MM:
            self._finish_watch = (now, float(pos))
            return None
        if now - watch[0] < MOTION_SETTLE_S:
            return None
        self._finish_watch = None
        return f"Motion finished at {pos:.2f} mm"

    def _record_poll_error(self, exc: Exception, session: AxisSession | None = None) -> None:
        message = f"Status read failed: {_describe(exc)}"
        with self._state_lock:
            self._poll_failures += 1
            failures = self._poll_failures
            repeated = message == self._last_error
            self._last_error = message
        if not repeated:
            self.log("error", message)
        if failures == COMM_LOST_AFTER:
            self.log("error", f"Communication lost ({failures} failed polls in a row)")
        if session is not None:
            self._check_stop(session, None)

    def _check_stop(self, session: AxisSession, status: dict[str, Any] | None, read_at: float | None = None) -> None:
        """After STOP: done once a status shows the axis idle; alert if that has not happened in time.

        With PIO in control, also alert if the axis moves again during the restart watch.
        """
        with self._state_lock:
            if self._session is not session:
                return
            restarted = self._check_restart(session, status, read_at)
        if restarted:
            self.log("error", restarted)
            return
        with self._state_lock:
            if self._session is not session:
                return
            if status is not None and status.get("moving") is False:
                self._stop_watch = None
                if self._stop_alert_kind != "restart":
                    self._stop_alert = self._stop_alert_kind = None
                return
            watch = self._stop_watch
            if watch is None or watch[0] is not session or time.monotonic() < watch[1]:
                return
            self._stop_watch = None
            waited = f"{self._stop_confirm_s:g} s after STOP"
            if status is None:
                message = f"STOP NOT CONFIRMED: no status reply {waited}. Use the hardware stop if the axis moves."
            else:
                pos = status.get("position_mm")
                where = f" at {pos:.2f} mm" if isinstance(pos, (int, float)) and not isinstance(pos, bool) else ""
                state = "still moving" if status.get("moving") else "not reporting whether it moves"
                message = f"STOP NOT CONFIRMED: the axis is {state}{where} {waited}. Use the PIO / hardware stop."
            self._stop_alert = self._last_error = message
            self._stop_alert_kind = "unconfirmed"
        self.log("error", message)


class _LockGuard:
    """Context manager: acquire with a timeout or raise a clear 'busy' error."""

    def __init__(self, lock: threading.Lock, label: str, timeout: float) -> None:
        self._lock, self._label, self._timeout = lock, label, timeout

    def __enter__(self) -> None:
        if not self._lock.acquire(timeout=self._timeout):
            raise BackendError(f"{self._label}: driver busy, try again.", 409)

    def __exit__(self, *_exc: object) -> None:
        self._lock.release()


def _describe(exc: BaseException) -> str:
    text = str(exc).strip()
    return f"{type(exc).__name__}: {text}" if text else type(exc).__name__


def _standstill(status: dict[str, Any] | None) -> float | None:
    """The position of a status that shows the axis at rest, else None."""
    if not status or status.get("moving") is not False:
        return None
    pos = status.get("position_mm")
    return float(pos) if isinstance(pos, (int, float)) and not isinstance(pos, bool) else None


def _close_quietly(session: AxisSession) -> None:
    try:
        session.close()
    except Exception:
        pass


def _cross_check_stroke(
    session: AxisSession, configured: tuple[float, float] | None
) -> tuple[dict[str, Any], str | None]:
    """Work out the stroke in use from the EC (read-only) and an optional override.

    * Without an override the stroke in use is the EC's own stroke (its soft limits minus their
      0.30 mm margin, exactly what the driver enforces). If those cannot be read, motion is disabled
      for the connection (returned as the block message) until a stroke is entered.
    * An override can only narrow the EC's stroke, never widen it; no overlap at all disables motion.
    * Position-table rows 0-2 outside the stroke in use, or a table that stops well short of the
      stroke when the soft limits are unknown, produce warnings.
    """
    warnings: list[str] = []
    block: str | None = None
    try:
        ec_limits, ec_note = session.ec_limits()
    except Exception as exc:
        ec_limits, ec_note = None, f"not readable ({_describe(exc)})"
    if ec_limits is not None and configured is not None:
        lo, hi = configured
        (ec_lo, ec_hi), (soft_lo, soft_hi) = ec_limits.stroke_mm, ec_limits.soft_limits_mm
        ec_text = f"this EC's own stroke {ec_lo:g}-{ec_hi:g} mm (soft limits {soft_lo:g} / {soft_hi:g} mm)"
        use_lo, use_hi = max(lo, ec_lo), min(hi, ec_hi)
        if use_lo >= use_hi:
            block = (f"Motion disabled: the stroke {lo:g}-{hi:g} mm lies outside {ec_text}. "
                     "Disconnect and set the right stroke (or leave it empty to use the EC's own).")
            warnings.append(block)
        elif (use_lo, use_hi) != (lo, hi):
            session.set_stroke((use_lo, use_hi))
            warnings.append(f"Stroke narrowed to {use_lo:g}-{use_hi:g} mm: the override {lo:g}-{hi:g} mm goes "
                            f"beyond {ec_text}.")
    elif ec_limits is None and configured is None:
        block = (f"Motion disabled: this EC's soft limits could not be read ({ec_note}) and no stroke was entered. "
                 "Disconnect and enter the stroke (min / max) for this EC.")
        warnings.append(block)
    in_use = session.stroke
    positions: list[float] = []
    for n, label in TABLE_LABELS.items():
        try:
            pos = session.table_row(n).get("position_mm")
        except Exception as exc:
            warnings.append(f"Position table could not be read to cross-check the stroke: {_describe(exc)}")
            break
        if not isinstance(pos, (int, float)) or isinstance(pos, bool):
            continue
        positions.append(float(pos))
        if in_use is not None and not in_use[0] <= pos <= in_use[1]:
            warnings.append(f"Position table {label} ({n}) is {pos:.2f} mm, outside the stroke in use "
                            f"{in_use[0]:g}-{in_use[1]:g} mm. Check the stroke for this EC; that quick-move button "
                            "is disabled.")
    if positions and in_use is not None and ec_limits is None and max(positions) < in_use[1] - STROKE_SLACK_MM:
        warnings.append(f"The position table reaches only {max(positions):.2f} mm but the stroke in use goes to "
                        f"{in_use[1]:g} mm, and this EC's soft limits are unknown. If its stroke is shorter, "
                        "disconnect and set the right stroke.")
    info = {
        "configured_mm": list(configured) if configured is not None else None,
        "in_use_mm": list(in_use) if in_use is not None and block is None else None,
        "ec_soft_limits_mm": list(ec_limits.soft_limits_mm) if ec_limits else None,
        "ec_stroke_mm": list(ec_limits.stroke_mm) if ec_limits else None,
        "ec_soft_limits_note": ec_note,
        "ec_max_speed_mm_s": ec_limits.max_speed_mm_s if ec_limits else None,
        "table_mm": [min(positions), max(positions)] if positions else None,
        "warnings": warnings,
    }
    return info, block


def _describe_stroke(info: dict[str, Any]) -> str:
    in_use = info["in_use_mm"]
    configured = info["configured_mm"]
    parts = ["no override (the EC's own)" if configured is None
             else f"override {configured[0]:g}-{configured[1]:g} mm"]
    ec, soft = info["ec_stroke_mm"], info["ec_soft_limits_mm"]
    parts.append(f"EC stroke {ec[0]:g}-{ec[1]:g} mm, soft limits {soft[0]:g} / {soft[1]:g} mm" if ec and soft
                 else f"EC soft limits {info['ec_soft_limits_note']}")
    if info.get("ec_max_speed_mm_s") is not None:
        parts.append(f"EC max speed {info['ec_max_speed_mm_s']:g} mm/s")
    table = info["table_mm"]
    if table:
        parts.append(f"position table {table[0]:.2f}-{table[1]:.2f} mm")
    used = f"{in_use[0]:g}-{in_use[1]:g} mm" if in_use else "none (motion disabled)"
    return f"Stroke in use {used} ({'; '.join(parts)})"


def _stroke_of(limits: Limits) -> tuple[float, float]:
    if limits.stroke_min_mm is None or limits.stroke_max_mm is None:
        raise BackendError("The stroke is unknown (the EC's soft limits were not read): disconnect and enter the "
                           "stroke for this EC.", 409)
    return limits.stroke_min_mm, limits.stroke_max_mm


def _check_position(position_mm: float, limits: Limits) -> None:
    lo, hi = _stroke_of(limits)
    if not lo <= position_mm <= hi:
        u = limits.units
        raise BackendError(f"Target {position_mm:g} {u} is outside the stroke {lo:g}-{hi:g} {u}.", 400)


def _check_profile(speed_mm_s: float, accel_g: float, limits: Limits) -> None:
    if not 0 < speed_mm_s <= limits.max_speed_mm_s:
        source = f" ({limits.max_speed_source})" if limits.max_speed_source != "GUI cap" else ""
        raise BackendError(f"Speed must be > 0 and <= {limits.max_speed_mm_s:g} {limits.units}/s{source}.", 400)
    if not 0 < accel_g <= limits.max_accel_g:
        raise BackendError(f"Acceleration must be > 0 and <= {limits.max_accel_g:g} G.", 400)


def _require_ready(status: dict[str, Any], *, need_homed: bool) -> None:
    """Refuse motion in states where the EC would ignore it or alarm. Unknown (None) fields don't block."""
    code = status.get("alarm_code")
    alarm = status.get("alarm")
    active = alarm if isinstance(alarm, bool) else code not in (None, 0, False)
    if active:
        shown = f"0x{code:03X}" if isinstance(code, int) and not isinstance(code, bool) else str(code)
        raise BackendError(f"Alarm {shown} is active. Reset the alarm first.", 409)
    if status.get("emergency_stop") is True:
        raise BackendError("Emergency stop is active.", 409)
    if status.get("servo_on") is False:
        raise BackendError("Servo is OFF. Turn the servo ON first.", 409)
    if status.get("homing") is True:
        raise BackendError("A home return is in progress. Wait for it to finish or press STOP.", 409)
    if need_homed and status.get("homed") is False:
        raise BackendError("Axis is not homed. Run Home first.", 409)
    if status.get("paused") is True:
        raise BackendError("Axis is paused (STP is on, set by another master). Release the pause first.", 409)
    if status.get("moving") is True:
        raise BackendError("Axis is moving. Wait for it to finish or press STOP.", 409)


def _require_at_rest(status: dict[str, Any], what: str) -> None:
    """Refuse configuration commands while the axis moves, pushes, homes or is paused (unknown fields don't block)."""
    for key, text in (("moving", "moving"), ("homing", "homing"), ("pushing", "pushing"), ("paused", "paused")):
        if status.get(key) is True:
            raise BackendError(f"{what} only at rest: the axis is {text}. Wait, or press STOP.", 409)


def _motion_in_progress(status: dict[str, Any], pause_ok: bool = False) -> str | None:
    """What the axis is doing when it is not at rest (``None`` when it is). An unknown motion flag counts as moving:
    the control mode is not switched on a guess. ``pause_ok`` (taking PC control from PIO with the servo off): a
    pause at a standstill does not count; the driver refuses it while anything moves."""
    if status.get("moving") is not False:
        return "moving" if status.get("moving") is True else "in an unknown motion state (moving cannot be read)"
    for key, text in (("homing", "homing"), ("pushing", "pushing"), ("paused", "paused")):
        if status.get(key) is True and not (pause_ok and key == "paused"):
            return text
    return None


def _describe_changes(old: dict[str, Any], new: dict[str, Any]) -> list[str]:
    """Human-readable transitions between two status snapshots."""
    messages = []
    for key, on_text, off_text in (
        ("servo_on", "Servo is ON", "Servo is OFF"),
        ("homed", "Home complete", "Home not complete"),
        ("modbus_control", "Modbus has control", "Modbus control released"),
        ("emergency_stop", "EMERGENCY STOP active", "Emergency stop cleared"),
        ("safety_speed", "Safety speed ON (the EC caps speed at ~100 mm/s)", "Safety speed OFF"),
        ("paused", "Paused (STP on)", "Pause released"),
    ):
        if key in old and old.get(key) != new.get(key) and new.get(key) is not None:
            messages.append(on_text if new[key] else off_text)
    if old.get("alarm_code") != new.get("alarm_code") and "alarm_code" in old:
        code = new.get("alarm_code")
        messages.append("Alarm cleared" if code in (0, None) else f"ALARM {code!r} raised")
    return messages  # "Motion finished at X mm" is logged by Controller._motion_finished once settled


def _public(data: dict[str, Any]) -> dict[str, Any]:
    """Copy without the internal ``_`` keys."""
    return {k: v for k, v in data.items() if not k.startswith("_")}


def _merge_unit_info(parts: list[dict[str, Any]], failures: list[str], wall_s: float) -> dict[str, Any]:
    """One unit_info dict from the per-group reads (each section comes from the group that read it)."""
    merged: dict[str, Any] = dict(parts[0])
    meta = dict(parts[0].get("meta") or {})
    meta["sections"], merged["errors"] = [], []
    duration = 0.0
    for part in parts:
        for key, value in part.items():
            if key not in ("meta", "errors") and value is not None and merged.get(key) is None:
                merged[key] = value
        part_meta = part.get("meta") or {}
        meta["sections"] += list(part_meta.get("sections") or [])
        duration += part_meta.get("duration_s") or 0.0
        merged["errors"] += list(part.get("errors") or [])
    merged["errors"] += failures
    meta["duration_s"] = round(duration, 3)
    meta["gui"] = {
        "groups": [list(group) for group in UNIT_INFO_GROUPS[: len(parts)]],
        "groups_read": len(parts),
        "bus_lock": "per request",
        "wall_s": round(wall_s, 3),
        "note": "read group by group with the bus lock taken per Modbus request: the status poll, commands and "
                "STOP get the bus between any two requests, and STOP ends the read",
    }
    merged["meta"] = meta
    return merged


class _EventLogHandler(logging.Handler):
    """Forwards ``iai_actuator_driver`` warnings and errors into the GUI event log."""

    def __init__(self, controller: Controller) -> None:
        super().__init__(level=logging.WARNING)
        self._controller = controller

    def emit(self, record: logging.LogRecord) -> None:
        try:
            level = "error" if record.levelno >= logging.ERROR else "warn"
            self._controller.log(level, f"Driver: {record.getMessage()}")
        except Exception:  # logging must never break a command
            self.handleError(record)
