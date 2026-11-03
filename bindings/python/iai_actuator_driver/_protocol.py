"""The Modbus RTU / IAI protocol facts: frame building, CRC, the IAI command replies and the
protocol constants (function codes, frame limits, the safe IAI commands, baud codes and rates, timeouts, retries).

Pure Python with no I/O: test doubles build frames with it and the core is cross-checked against it. The bus itself
(:class:`~iai_actuator_driver.modbus.ModbusRTU`) is in :mod:`iai_actuator_driver.modbus`, which re-exports every name
here (``from iai_actuator_driver.modbus import crc16`` keeps working). Evidence words as in the README: DOC, ON-WIRE,
VERIFIED, UNVERIFIED.
"""

from __future__ import annotations

import struct
from collections.abc import Callable
from typing import NamedTuple

__all__ = [
    "FC_READ_HOLDING",
    "FC_WRITE_COIL",
    "FC_WRITE_REGISTER",
    "FC_WRITE_REGISTERS",
    "FC_REPORT_SLAVE_ID",
    "FC_READ_WRITE_REGISTERS",
    "FC_IAI_COMMAND",
    "FC_IAI_COMMAND2",
    "MAX_READ_REGISTERS",
    "MAX_FC03_REGISTERS",
    "MAX_WRITE_REGISTERS",
    "COIL_ON",
    "COIL_OFF",
    "IAI_SAFE_COMMANDS",
    "IAI_SAFE_COMMANDS2",
    "FrameLogger",
    "DEFAULT_BAUDRATE",
    "SLAVE_ID_TIMEOUT_S",
    "SCAN_TIMEOUT_S",
    "RECOMMENDED_RETRIES",
    "MIN_FRAME_TIMEOUT_S",
    "USB_MARGIN_S",
    "DEFAULT_RESPONSE_DELAY_MS",
    "BROADCAST_FUNCTIONS",
    "BROADCAST_TURNAROUND_S",
    "IAI_BAUD_CODES",
    "VERIFIED_BAUD_RATES",
    "UNIT_DEPENDENT_BAUD_RATES",
    "BAUD_DETECT_ORDER",
    "BAUD_BY_CODE",
    "BAUD_PROBE_ALL",
    "crc16",
    "build_frame",
    "check_crc",
    "IaiResponse",
    "IaiResponse2",
]

FC_READ_HOLDING = 0x03
FC_WRITE_COIL = 0x05
FC_WRITE_REGISTER = 0x06
FC_WRITE_REGISTERS = 0x10
FC_REPORT_SLAVE_ID = 0x11
FC_READ_WRITE_REGISTERS = 0x17
FC_IAI_COMMAND = 0x45
FC_IAI_COMMAND2 = 0x48

MAX_READ_REGISTERS = 60
"""Largest single FC03 read this driver issues by default; bigger reads are split (a conservative size).
:meth:`ModbusRTU.read_registers` takes ``chunk=`` up to :data:`MAX_FC03_REGISTERS` for a block that must come in
one frame (the RCON all-axes monitor: 120 words)."""
MAX_FC03_REGISTERS = 125
"""Modbus limit of one FC03 read."""
MAX_WRITE_REGISTERS = 123
"""FC10 frame must fit the controller's 256-byte receive buffer (ME0162 p.5-4)."""

COIL_ON = 0xFF00
COIL_OFF = 0x0000

# IAI control commands that are allowed without ``allow_unsafe=True``. Anything else (parameter /
# position non-volatile writes, baud or axis-number change, parameter or absolute-encoder
# initialisation, alarm-list clear, software reset, factory inspection commands...) is refused.
# Codes: IAI's control-command numbering. 0x45 cmd 4 / 5 / 9 / 0x0C are ON-WIRE (USB capture of IA-OS);
# 0x45 cmd 5 / 9 / 0x0A / 0x0B / 0x0C are VERIFIED on an EC (cmd 6 on the EC-GRBP8M gripper and the PCON-CB);
# the others, and every 0x48 code, are UNVERIFIED.
IAI_SAFE_COMMANDS: dict[int, str] = {
    0x05: "reset (alarm reset only: op1 must be 0)",
    0x06: "move to position index with override",
    0x07: "move to position index with speed",
    0x09: "servo on/off",
    0x0A: "return home",
    0x0B: "decel stop",
    0x0C: "change action mode",
    0x0D: "change speed while moving",
    0x0E: "change acceleration while moving",
}
IAI_SAFE_COMMANDS2: dict[int, str] = {
    0x22: "change action mode",
    0x23: "alarm reset",
    0x30: "servo on/off",
    0x31: "return home",
    0x32: "move to position index with override",
    0x33: "move to position index with speed",
    0x34: "decel stop",
    0x35: "change speed while moving",
    0x36: "change acceleration while moving",
}

FrameLogger = Callable[[str, bytes], None]
"""Callback ``(direction, frame)``; direction is ``"tx"``, ``"rx"`` or ``"rx-partial"``."""

DEFAULT_BAUDRATE = 115200
"""Rate a port is opened at when none is given (IA-OS switches ECs to it; volatile)."""

SLAVE_ID_TIMEOUT_S = 0.65
"""How long :meth:`ModbusRTU.report_slave_id` waits for the answer by default. VERIFIED: the EC-R6S-150 rod
answered FC11 only after 0.2-0.4 s (no reply within 0.2 s), the other units at once."""

SCAN_TIMEOUT_S = 0.05
"""How long :meth:`ModbusRTU.probe` waits for an answer (one FC03 read, no retries). An answering unit
replies within a few ms at 9600..230400 baud, so 50 ms per address keeps a 16-address scan at ~1 s.
Gateway lines wait longer (``bus.line.rules.probe_timeout_s``: 0.65 s RCON first contact, 0.2 s ROBONET)."""

RECOMMENDED_RETRIES = 3
"""Retries IAI's Modbus manual asks for: "Nrt = 3", "setting of the number of retries is mandatory" (DOC ME0162-12A
p.4-2). The default of :class:`~iai_actuator_driver.cell.Port` and ``Cell.open``; idempotent requests only."""

MIN_FRAME_TIMEOUT_S = 0.2
"""Floor of :meth:`ModbusRTU.response_timeout`: the read timeout every verified EC / PCON session used."""

USB_MARGIN_S = 0.05
"""Added to the manual's Tout for the USB-serial adapter's latency and the PC's scheduling (driver choice)."""

DEFAULT_RESPONSE_DELAY_MS = 5.0
"""RTIM, the unit's minimum delay before it answers: EC item 8.4 / CON parameter No.17, default 5 ms (DOC
ME0162-12A p.4-1). Tell the bus a unit's own value with :meth:`ModbusRTU.set_response_delay`."""

BROADCAST_FUNCTIONS: frozenset[int] = frozenset({0x05, 0x06, 0x10})
"""Function codes that may be broadcast (slave 0): every FC05 coil, FC06 and FC10, never a read (DOC ME0162-12A
p.1-1, 5-3, 5-5 to 5-8, 5-80)."""

BROADCAST_TURNAROUND_S = 0.02
"""Silence kept after a broadcast frame before the next request, so every unit has acted on it (driver choice,
UNVERIFIED: no unit has been tested on a multi-drop line)."""

IAI_BAUD_CODES: dict[int, int] = {
    9600: 0x04,  # UNVERIFIED
    19200: 0x05,  # UNVERIFIED
    38400: 0x06,  # VERIFIED on the EC-GRBP8M gripper (direct, from 230400 and 9600); IGNORED by the EC-R6S rod
    #               when sent direct (the rod stayed at 230400)
    57600: 0x13,  # UNVERIFIED
    115200: 0x14,  # ON-WIRE (IA-OS capture) + VERIFIED: EC-R6S rod direct, both ways (230400 -> 115200, twice)
    230400: 0x15,  # VERIFIED: EC-R6S rod direct (115200 -> 230400)
    14400: 0x11,  # UNVERIFIED; never probed at connect
    28800: 0x12,  # UNVERIFIED; never probed at connect
    76800: 0x07,  # UNVERIFIED; never probed at connect
}
"""op1 code of the IAI baud-rate change (0x45 cmd 4) per rate, in IAI's baud-code numbering. Only 0x14 (115200)
and 0x15 (230400) are verified on hardware (capture ``00 45 00 04 00 14 00 05``; the EC-R6S rod switched both
ways when they were sent at the current rate). **0x06 ("38400") was ignored by the EC-R6S rod** when sent direct,
so this table is not the EC's for every code: see :data:`VERIFIED_BAUD_RATES`."""

VERIFIED_BAUD_RATES: tuple[int, ...] = (115200, 230400)
"""Rates whose baud-change code was verified on hardware (the EC-R6S rod, sent direct at the current rate).
:meth:`Elecylinder.set_baudrate` sends any other rate with an IAI code with a WARNING (``allow_unverified_code`` is
accepted and ignored: no longer needed)."""

UNIT_DEPENDENT_BAUD_RATES: tuple[int, ...] = (38400,)
"""Rates whose code some units take and others ignore: 0x06 (38400) switched the EC-GRBP8M gripper (VERIFIED, sent
direct from 230400 and from 9600) and the PCON-CB (VERIFIED, from 230400) and was IGNORED by the EC-R6S rod. :meth:`Elecylinder.set_baudrate` sends them
without a warning because it checks the result: a unit that ignores the code keeps answering at its rate, and the
port is left there (:class:`~iai_actuator_driver.errors.VerificationError`)."""

BAUD_DETECT_ORDER: tuple[int, ...] = (115200, 38400, 9600, 230400, 57600, 19200)
"""Rates :meth:`ModbusRTU.detect_baudrate` tries after the current one. VERIFIED-LIVE: the three tested ECs
boot at 38400 (their BRSL, item 8.3 = code 6) and IA-OS switches them to 115200 (volatile, ON-WIRE)."""

BAUD_BY_CODE: dict[int, int] = {code: rate for rate, code in IAI_BAUD_CODES.items()}
"""IAI baud code -> rate (e.g. BRSL, item 8.3: 6 -> 38400)."""

BAUD_PROBE_ALL: tuple[int, ...] = BAUD_DETECT_ORDER + tuple(r for r in IAI_BAUD_CODES if r not in BAUD_DETECT_ORDER)
"""Every rate an IAI baud code names: what :meth:`Elecylinder.set_baudrate` probes when the EC does not
answer at the new rate (an EC whose code table differs from :data:`IAI_BAUD_CODES` may have gone to any of
them)."""


def crc16(data: bytes) -> int:
    """Return the CRC-16/MODBUS of ``data`` (poly 0xA001 reflected, init 0xFFFF).

    On the wire the result is sent low byte first: ``struct.pack("<H", crc16(body))``.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_frame(slave: int, pdu: bytes) -> bytes:
    """Return the RTU frame ``[slave][pdu...][crc lo][crc hi]``."""
    body = bytes([slave]) + pdu
    return body + struct.pack("<H", crc16(body))


def check_crc(frame: bytes) -> bool:
    """True if the last two bytes of ``frame`` are the CRC of the rest."""
    return len(frame) >= 4 and struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2])


class IaiResponse(NamedTuple):
    """Response to function 0x45: rejection code (0 = accepted) and status byte."""

    rejection: int
    status: int


class IaiResponse2(NamedTuple):
    """Response to function 0x48: 16-bit rejection code (0 = accepted) and four 32-bit status words."""

    rejection: int
    status: tuple[int, int, int, int]
