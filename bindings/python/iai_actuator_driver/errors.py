"""Exception hierarchy for iai_actuator_driver.

Every error raised on purpose by this package derives from :class:`EcDriverError`, so callers can
catch one type. Communication problems derive from :class:`ModbusError`; problems with what the
actuator was asked to do (soft limits, alarms, missing preconditions) derive from :class:`EcError`.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .recovery import RecoveryReport

NO_ANSWER_HINT = (
    "check the 24 V supply, the cable, the slave address (axis number + 1) and that IA-OS (or any other program) has "
    "disconnected. Not supported, whatever the wiring: the controllers with the SEL protocol (XSEL, SSEL, ASEL, PSEL, "
    "MSEL and the TTA table-top robots) do not speak Modbus RTU, and an ELECYLINDER on an RCON-EC / REC connection "
    "unit has its serial port disabled"
)
"""What to check when a unit does not answer at all (appended to the no-answer errors of ``open`` / ``scan`` / the GUI)."""

EXCEPTION_NAMES: dict[int, str] = {
    0x01: "Illegal Function",
    0x02: "Illegal Data Address",
    0x03: "Illegal Data Value",
    0x04: "Slave Device Failure",
}


class EcDriverError(Exception):
    """Base class for all errors raised by iai_actuator_driver."""


# --------------------------------------------------------------------------- Modbus / link layer


class ModbusError(EcDriverError):
    """A Modbus transaction failed."""


class ModbusTimeout(ModbusError, TimeoutError):
    """No (complete) response arrived within the timeout, after all retries."""


class ModbusCRCError(ModbusError):
    """A response arrived but its CRC did not match, after all retries."""


class ModbusFrameError(ModbusError):
    """A response arrived but did not match the request (wrong slave, function or echo)."""


class TransportError(ModbusError):
    """The transport itself failed (the core's serial port: a USB adapter pulled, the port gone or closed underneath
    the bus). Never retried; when a Python transport raised, ``__cause__`` is its error. A
    :class:`~iai_actuator_driver.errors.ModbusError`, so existing handlers still catch it; ``iai_actuator_driver.recovery`` classifies it
    as ``"port_vanished"``. Also importable from :mod:`iai_actuator_driver.modbus`."""


class ModbusException(ModbusError):
    """The slave answered with a Modbus exception response (function code | 0x80).

    Attributes:
        code: exception code (1 = illegal function, 2 = illegal address, 3 = illegal value, 4 = failure).
        function: the function code of the request that was rejected.
    """

    def __init__(self, code: int, function: int, message: str | None = None) -> None:
        self.code = code
        self.function = function
        name = EXCEPTION_NAMES.get(code, "Unknown exception")
        super().__init__(message or f"Modbus exception {code:02X}h ({name}) for function {function:02X}h")


class IaiCommandRejected(ModbusError):
    """An IAI control command (function 0x45 / 0x48) was answered with a non-zero rejection code.

    The EC understood the frame and refused the command: it is never retried (a rejection is a valid
    answer). ``rejection`` is the code (0x45: one byte 0x01-0x3F; 0x48: a word 0x1000-0x13FF), ``reason``
    its name (0x11 "1st operand error" is VERIFIED on hardware; other names such as 0x23 "motion command
    while the servo is OFF", 0x25 "software reset while the servo is ON" or 0x3A "command prohibited in
    monitor (PIO) mode" are UNVERIFIED) and ``hint`` the driver's short advice ("" if none). Tables:
    ``registers.IAI_REJECTIONS`` / ``IAI2_REJECTIONS``.
    """

    def __init__(self, function: int, command: int, rejection: int, status: object = None) -> None:
        from .registers import iai_rejection_hint, iai_rejection_name

        self.function = function
        self.command = command
        self.rejection = rejection
        self.status = status
        self.reason = iai_rejection_name(function, rejection)
        self.hint = iai_rejection_hint(function, rejection, command)
        width = 6 if function == 0x48 else 4
        text = (
            f"IAI command {command:#04x} (function {function:02X}h) rejected with code {rejection:#0{width}x} "
            f"({self.reason})"
        )
        super().__init__(text + (f": {self.hint}" if self.hint else ""))


class UnsafeCommandError(EcDriverError):
    """Refused to send a command that could damage configuration (baud/axis change, init, NV write...).

    Nothing was sent. Commands on the never-send lists (``registers.IAI_NEVER_SEND`` / ``IAI2_NEVER_SEND``)
    need ``allow_never_send=True``; ``allow_unsafe`` alone is not enough for them.
    """


# --------------------------------------------------------------------------- actuator layer


class EcError(EcDriverError):
    """The actuator cannot do what was asked."""


class SoftLimitError(EcError, ValueError):
    """A target or motion parameter is outside the configured soft limits / caps. Nothing was sent."""


class EcNotReadyError(EcError):
    """A precondition for the command is not met (servo off, not homed, Modbus control disabled...)."""


class ServoBlockedError(EcNotReadyError):
    """The servo cannot (or did not) come on, with the reasons the unit's status bits give.

    ``reasons`` lists them in words (teaching-tool enable switch released, motor drive power cut, emergency stop
    input, alarm, PIO / AUTO in control ...; DOC ME0162-12A p.4-11, 4-19 to 4-21, 5-83); it may be empty when the
    status shows no known cause (e.g. the servo-ON delay or the excitation detection of a first SON still running).
    """

    def __init__(self, message: str, reasons: tuple[str, ...] = ()) -> None:
        self.reasons = tuple(reasons)
        super().__init__(message)


class LineRuleError(UnsafeCommandError):
    """The line the unit sits on does not carry this command (e.g. no FC05 coils, no IAI 0x45 / 0x48 commands or no
    broadcast behind a gateway). Nothing was sent."""


class EcAlarmError(EcError):
    """The actuator reports an alarm."""

    def __init__(self, alarm_code: int, message: str | None = None) -> None:
        self.alarm_code = alarm_code
        super().__init__(message or f"actuator alarm {alarm_code:#05x}")


class MotionTimeoutError(EcError, TimeoutError):
    """Motion did not complete in time. A decel stop was requested before this was raised."""


class MotionIncompleteError(EcError):
    """The axis settled (in position, not moving) away from the commanded target.

    Typical causes: a STOP from another master, the EC ignoring the command (STOP held, PIO in
    control) or the EC clamping the target to its own soft limit.
    """


class NonVolatileWriteError(EcError, PermissionError):
    """A write to non-volatile memory (position table) was refused: a protected row (the EC's simple-data rows
    256-267) or one the table cannot address. Nothing was written."""


class VerificationError(EcError):
    """Read-back after a write did not match what was written."""


# --------------------------------------------------------------------------- recovery and cell


class RecoveryError(EcDriverError):
    """:func:`~iai_actuator_driver.recovery.recover` could not bring the axis back (the port or the unit did not return
    in time, the identity could not be confirmed, no session to compare with). ``report`` holds what was found so far
    (or ``None``). Also importable from :mod:`iai_actuator_driver.recovery`."""

    def __init__(self, message: str, report: RecoveryReport | None = None) -> None:
        super().__init__(message)
        self.report = report


class UnitReplacedError(RecoveryError):
    """Another unit answers where the tracked one was (a different serial number or model). Nothing was written to it:
    check the machine, then :func:`~iai_actuator_driver.recovery.track` the new unit (and back it up) before using it.
    ``expected`` / ``found``: the identity fields compared. Also importable from :mod:`iai_actuator_driver.recovery`."""

    def __init__(
        self,
        message: str,
        *,
        expected: Mapping[str, Any],
        found: Mapping[str, Any],
        report: RecoveryReport | None = None,
    ) -> None:
        super().__init__(message, report)
        self.expected = dict(expected)
        self.found = dict(found)


class CellError(EcDriverError):
    """One or more axes failed in a cell operation. ``errors`` / ``results`` are keyed by axis name. Also importable
    from :mod:`iai_actuator_driver.cell`."""

    def __init__(self, operation: str, errors: Mapping[str, BaseException], results: Mapping[str, Any]) -> None:
        self.operation = operation
        self.errors = dict(errors)
        self.results = dict(results)
        detail = "; ".join(f"{name}: {type(exc).__name__}: {exc}" for name, exc in self.errors.items())
        super().__init__(f"{operation} failed on {len(self.errors)} axis/axes: {detail}")
