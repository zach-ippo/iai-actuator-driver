"""The Python part of ``iai_actuator_driver.recovery``: the link states, event kinds and timings, the
records (:class:`LinkCheck`, :class:`RecoveryReport`, :class:`RecoveryEvent`, :class:`RecoveryPolicy`), the restart
signs of a record and the backup reader. No unit I/O.

``iai_actuator_driver.recovery`` (where the steps, the rules and the evidence are) imports these names from here.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .. import registers as R
from ..alarms import alarm_level, alarm_name
from ..errors import ModbusCRCError, ModbusFrameError, ModbusTimeout
from . import publish

if TYPE_CHECKING:
    from ..recovery import Session
    from ..values import Status

LINK_OK = "ok"
PORT_VANISHED = "port_vanished"
UNIT_SILENT = "unit_silent"
UNIT_RESTARTED = "unit_restarted"
UNIT_REPLACED = "unit_replaced"
RECOVERING = "recovering"
LINK_STATES = (LINK_OK, PORT_VANISHED, UNIT_SILENT, UNIT_RESTARTED, UNIT_REPLACED, RECOVERING)
"""States of :class:`LinkCheck` / :func:`link_state`. ``"recovering"``: :func:`recover` is running on the axis."""
EVENT_LINK_LOST = "link_lost"
EVENT_UNIT_RESTARTED = "unit_restarted"
EVENT_UNIT_REPLACED = "unit_replaced"
EVENT_RECOVERED = "recovered"
EVENT_KINDS = (EVENT_LINK_LOST, EVENT_UNIT_RESTARTED, EVENT_UNIT_REPLACED, EVENT_RECOVERED)

BOOT_BAUDRATE = 38400
"""The rate every unit tested powers up at (BRSL item 8.3 = code 6; VERIFIED): probed first after a power loss."""
RECOVER_TIMEOUT_S = 30.0
PORT_POLL_S = 0.5
"""How often :func:`recover` looks for the port / the unit again while waiting."""
POSITION_TOLERANCE_MM = 0.1
UPTIME_SLACK_S = 2.0
UPTIME_RATE_MIN = 0.9
"""The uptime counter must have advanced at least ``UPTIME_RATE_MIN`` x the PC's elapsed time minus
``UPTIME_SLACK_S`` since the last reading; less means the unit restarted meanwhile (it counts ms since power-up and
tracks the wall clock: VERIFIED)."""
_UPTIME_WRAP_MS = 1 << 32
_HOUR_MS = 3_600_000
PARAMETER_CLASSES: tuple[int, ...] = (0x0007, 0x0008, 0x0009, 0x000C)
"""Directory classes compared by the parameter integrity check (the writable parameter classes; classes 3 / 4 carry
live values such as the present alarm and are left out)."""
_IDENTITY_KEYS = ("serial_number", "model_name", "system_code", "unit_code")
_LINK_ERRORS = (ModbusTimeout, ModbusCRCError, ModbusFrameError)


@dataclass(frozen=True)
class RecoveryEvent:
    """One change of an axis's link state: ``kind`` is one of :data:`EVENT_KINDS`; ``cause`` the link state behind
    it (``"port_vanished"`` ...); ``time`` is ``time.time()``."""

    kind: str
    axis: str | None
    message: str
    cause: str | None = None
    time: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


EventCallback = Callable[[RecoveryEvent], Any]
AlarmKey = tuple[int, int, int, "int | None"]
"""(code, detail, address, time stamp ms or None) of one alarm-history slot."""


def _uptime_ms(st: Status | None) -> int | None:
    if st is None:
        return None
    value = st.raw.get("STIM")
    return None if value is None else int(value)


def _load_backup(backup: Mapping[str, Any] | str | Path) -> Mapping[str, Any]:
    if isinstance(backup, Mapping):
        return backup
    return json.loads(Path(backup).read_text(encoding="utf-8"))


def _chain(exc: BaseException) -> list[BaseException]:
    out: list[BaseException] = []
    current: BaseException | None = exc
    while current is not None and current not in out and len(out) < 8:
        out.append(current)
        current = current.__cause__ or current.__context__
    return out


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
    refreshed, in words; empty when none."""
    signs: list[str] = []
    up = _uptime_ms(status)
    before = session.uptime_ms
    if up is not None and before is not None:
        wrapped = before > _UPTIME_WRAP_MS - _HOUR_MS and up < _HOUR_MS
        if not wrapped:
            if up < before:
                signs.append(f"uptime counter dropped ({before / 1000:.1f} s -> {up / 1000:.1f} s)")
            elif session.uptime_at is not None:
                elapsed = (time.perf_counter() if now is None else now) - session.uptime_at
                expected = before + 1000.0 * (UPTIME_RATE_MIN * elapsed - UPTIME_SLACK_S)
                if up < expected:
                    signs.append(
                        f"uptime counter {up / 1000:.1f} s is behind the {expected / 1000:.1f} s it should show at "
                        "least (restarted while the PC was away)"
                    )
    if any(rec[0] == R.ALARM_POWER_UP for rec in new_records):
        signs.append("new power-up record (0x0FFF) in the alarm history")
    if session.modbus_control and not status.modbus_enabled:
        signs.append("Modbus control (PMSS) dropped while the PC held it")
    if session.homed and not status.homed and not status.homing:
        signs.append("home (HEND) lost")
    if (
        baudrate_found is not None
        and session.baudrate is not None
        and baudrate_found != session.baudrate
        and baudrate_found == boot_baudrate
    ):
        signs.append(f"baud rate fell back to {boot_baudrate} (the power-up rate) from {session.baudrate}")
    return signs


@dataclass(frozen=True)
class LinkCheck:
    """Result of :func:`check_link`: ``state`` (:data:`LINK_STATES`), ``reasons`` in words, the ``status`` read (when
    the unit answered), the ``error`` (when it did not) and how long the check took."""

    state: str
    reasons: tuple[str, ...] = ()
    status: Status | None = None
    error: BaseException | None = None
    elapsed_s: float = 0.0
    axis: str | None = None

    @property
    def ok(self) -> bool:
        return self.state == LINK_OK

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis, "state": self.state, "ok": self.ok, "reasons": list(self.reasons),
            "error": None if self.error is None else f"{type(self.error).__name__}: {self.error}",
            "elapsed_s": self.elapsed_s,
            "position_mm": None if self.status is None else self.status.position_mm,
        }  # fmt: skip


@dataclass
class RecoveryPolicy:
    """What :func:`recover` restores and checks. Motion is never part of it.

    ``restore_baudrate``: switch the unit back to the rate the application had set (only a verified baud-change code;
    a broadcast: every unit of a shared line switches; never behind a gateway). ``restore_control``: take PC control
    (PMSL) again when the application held it. ``restore_safety_speed``: put the safety-speed mode back as it was.
    ``clear_message_alarms``: reset a message-level alarm (a warning such as 0x069); other alarms are left for the
    operator. ``check_rows``: compare the stored rows with the session's copy / the backup and check their checksums
    (``True``: the copied rows, else rows 0 .. ``max_positions() - 1``; or row numbers). ``check_parameters``: compare
    the parameter classes with the session's copy / the backup. ``backup``: an :func:`iai_actuator_driver.info.backup` dict or
    file to compare with (used when the session has no copy). ``position_tolerance_mm``: |position - last target| that
    still counts as at the target. ``poll_s``: how often to look for the port / the unit while waiting.
    ``boot_baudrate``: the power-up rate probed first after a power loss. ``list_ports``: replaces
    :func:`iai_actuator_driver.cell.list_ports` for the "port listed again?" check.
    """

    restore_baudrate: bool = True
    restore_control: bool = True
    restore_safety_speed: bool = True
    clear_message_alarms: bool = True
    check_rows: bool | Iterable[int] = False
    check_parameters: bool = False
    backup: Mapping[str, Any] | str | Path | None = None
    position_tolerance_mm: float = POSITION_TOLERANCE_MM
    poll_s: float = PORT_POLL_S
    boot_baudrate: int = BOOT_BAUDRATE
    list_ports: Callable[[], list[dict[str, Any]]] | None = None


@dataclass
class RecoveryReport:
    """What :func:`recover` found and did. ``cause``: the link state it started from; ``restart_signs`` why the unit is
    taken to have restarted; ``actions`` what was sent / done, in order; ``notes`` what the application should know or
    do (nothing in a note was done automatically). ``needs_attention`` sums up whether anything needs a decision before
    the next move; ``ready_to_move`` whether the status allows one (servo ON, homed, PC control, no alarm).
    ``control_restored``: PC control was taken again (PMSS had dropped); ``control_kept``: the application held it before
    the fault and the unit still did (a line break does not drop it), so nothing was written."""

    axis: str | None
    cause: str = LINK_OK
    restarted: bool = False
    restart_signs: list[str] = field(default_factory=list)
    port_reopened: bool = False
    baudrate_found: int | None = None
    baudrate: int | None = None
    baud_restored: bool = False
    same_unit: bool = False
    identity: dict[str, Any] = field(default_factory=dict)
    control_restored: bool = False
    control_kept: bool = False
    safety_speed_restored: bool = False
    limits_changed: list[str] = field(default_factory=list)
    alarm_code: int = 0
    alarm_name: str | None = None
    alarm_level: str | None = None
    alarms_cleared: list[int] = field(default_factory=list)
    new_alarm_records: list[dict[str, Any]] = field(default_factory=list)
    modbus_control: bool | None = None
    safety_speed: bool | None = None
    servo_on: bool | None = None
    servo_was_on: bool | None = None
    homed: bool | None = None
    needs_home: bool = False
    position_mm: float | None = None
    last_target_mm: float | None = None
    position_error_mm: float | None = None
    at_target: bool | None = None
    rows_checked: list[int] = field(default_factory=list)
    row_problems: list[str] = field(default_factory=list)
    parameters_checked: int = 0
    parameter_differences: list[str] = field(default_factory=list)
    actions: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0
    status: Status | None = field(default=None, repr=False)

    @property
    def fault_alarm(self) -> bool:
        """An alarm that blocks motion is still active."""
        return bool(self.status.alarm) if self.status is not None else bool(self.alarm_code)

    @property
    def needs_attention(self) -> bool:
        """Anything the application must decide before the next move: home lost, a fault alarm, a move cut off (not at
        its target) or still running, changed ranges, row / parameter differences, or the servo off where it was on."""
        return bool(
            self.needs_home or self.fault_alarm or self.at_target is False or self.limits_changed or self.row_problems
            or self.parameter_differences or (self.servo_was_on and not self.servo_on)
            or (self.status is not None and self.status.moving)
        )  # fmt: skip

    @property
    def ready_to_move(self) -> bool:
        """The status allows a move now (servo ON, homed, PC control, no alarm, no E-stop)."""
        return self.status is not None and self.status.ready

    def summary(self) -> str:
        parts = [f"{self.axis or 'axis'}: recovered from {self.cause}"]
        if self.restarted:
            parts.append("unit restarted")
        if self.port_reopened:
            parts.append("port reopened")
        if self.baud_restored:
            parts.append(f"baud {self.baudrate_found} -> {self.baudrate}")
        if self.needs_home:
            parts.append("needs home")
        if self.fault_alarm:
            parts.append(f"alarm 0x{self.alarm_code:03X}")
        if self.status is not None and self.status.moving:
            parts.append("still moving")
        if self.at_target is False:
            parts.append(f"{self.position_error_mm:+.2f} from the last target")
        return ", ".join(parts)

    def to_dict(self) -> dict[str, Any]:
        data = {f.name: getattr(self, f.name) for f in fields(self) if f.name != "status"}
        data.update(
            status=None if self.status is None else self.status.to_dict(), fault_alarm=self.fault_alarm,
            needs_attention=self.needs_attention, ready_to_move=self.ready_to_move, summary=self.summary(),
        )  # fmt: skip
        return data


publish(globals(), "recovery")
