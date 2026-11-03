"""The Python part of ``iai_actuator_driver.gateway``: the line kinds, the gateway addresses and
timings, the operation names of the line rules, the gateway errors and the decoded gateway records. No I/O.

``iai_actuator_driver.gateway`` (where the line rules, the gateway protocols and the evidence are) imports these names
from here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .. import registers as R
from ..errors import EcNotReadyError
from . import publish


# ====================================================================== names and constants


LINE_DIRECT = "direct"
LINE_RCON = "rcon"
LINE_ROBONET = "robonet"
LINE_ROBONET_GATEWAY = "robonet-gateway"
LINE_KINDS: tuple[str, ...] = (LINE_DIRECT, LINE_RCON, LINE_ROBONET, LINE_ROBONET_GATEWAY)

# Operations an axis can ask about (Line.allows / Line.require).
OP_BROADCAST = "broadcast"  # any frame to address 0
OP_BAUD_CHANGE = "baud_change"  # the broadcast baud change (0x45 cmd 4 to address 0)
OP_IAI_COMMAND = "iai_command"  # function 0x45
OP_IAI_COMMAND2 = "iai_command2"  # function 0x48
OP_SOFTWARE_RESET = "software_reset"  # 0x45 cmd 5 op1 1
OP_ACTION_MODE = "action_mode"  # 0x45 cmd 0x0C
OP_HOME_IAI = "home_iai"  # 0x45 cmd 0x0A
OP_COIL = "coil"  # FC05 (servo / home / STOP / pause coils, PMSL)
OP_REPORT_SLAVE_ID = "report_slave_id"  # FC11
OP_AXIS_CONTROL = "axis_control"  # any command to an axis at all
OPERATIONS: tuple[str, ...] = (
    OP_BROADCAST, OP_BAUD_CHANGE, OP_IAI_COMMAND, OP_IAI_COMMAND2, OP_SOFTWARE_RESET, OP_ACTION_MODE, OP_HOME_IAI,
    OP_COIL, OP_REPORT_SLAVE_ID, OP_AXIS_CONTROL,
)  # fmt: skip

_FC_COIL = 0x05
_FC_SLAVE_ID = 0x11
RCON_GATEWAY_SLAVE = 0xD0
"""RCON-GW unit identifier (D0h; also D1h or FFh). DOC ME0413 p.4-1."""
RCON_GATEWAY_ADDRESSES: tuple[int, ...] = (0xD0, 0xD1, 0xFF)
RCON_GW_STATUS = 0x3700
"""GWSR (0x3700) .. NPLK (0x370A): 11 words. DOC ME0413 p.5-2, 5-3."""
RCON_GW_STATUS_WORDS = 0x0B
RCON_GW_ALARM = 0x3708
"""Gateway alarm code, u32 high word first (e.g. 0x000008BA). DOC ME0413 p.5-2; ME0425 p.70."""
RCON_MONITOR_BASE = 0x2900
"""All-axes monitor: 15 words per axis at 0x2900 + 15 x axis (axes 0-15). DOC ME0413 p.5-2."""
RCON_MONITOR_STRIDE = 15
RCON_MONITOR_AXES_PER_READ = 8
"""8 axes = 120 words per FC03 frame (``D0 03 29 00 00 78 5F F5``), so 16 axes take two frames."""
RCON_MAX_AXES = 16
RCON_SERIAL_BASE = 0x9400
"""Serial codes CTS1..CTS4, ASCII, 8 words, on each driver unit. DOC ME0413 p.5-10 to 5-23 (SCON-CB-RC from V000D)."""
RCON_SERIAL_WORDS = 8
ROBONET_GATEWAY_SLAVE = 0x3F
"""ROBONET RGW-SIO in gateway mode (SW2 OFF): fixed slave 3Fh. DOC ME0208 p.26, p.119."""
ROBONET_GW_STATUS = 0xF700
"""Gateway status 0 (RUN, counts) and status 1 (LNK15..LNK0). DOC ME0208 p.40, p.48, p.123, p.128-129."""
ROBONET_GW_STATUS_WORDS = 2

GATEWAY_PROBE_TIMEOUT_S = 0.05
"""Wait per detection probe (D0h, 3Fh). On a line without a gateway both cost ~0.1 s together, once per port."""
GATEWAY_FRAME_TIMEOUT_S = 0.3
"""Minimum response timeout per frame on a gateway line: a teaching-tool frame can be delayed by >= 40 ms per
exchange, more with EC units (DOC ME0394 p.5-17; ME0384 p.6-118)."""
GATEWAY_FIRST_CONTACT_S = 0.65
"""Minimum wait for the first contact with an axis behind an RCON gateway (probe / scan of a linked axis)."""
ROBONET_PROBE_TIMEOUT_S = 0.2
"""Probe wait in ROBONET through mode: the bytes are relayed over the 230.4 kbps internal link (UNVERIFIED)."""
GWSR_CACHE_S = 2.0
"""How long a read of the RCON gateway status (MANU flag) is trusted before it is read again."""
POSITION_UNITS_MM: tuple[float, ...] = (0.01, 0.001)
"""RCON parameter No.207: 0 = 0.01 mm, 1 = 0.001 mm (target, band, zones, monitored position). DOC ME0384 p.8-56."""

EC_BEHIND_GATEWAY = (
    "an ELECYLINDER behind an EC connection unit (RCON-EC / REC) is not reachable over IAI's public Modbus "
    "(DOC ME0413 p.4-1: 'RCON-EC unit is not applicable'), and its own SIO port is disabled there (DOC ME0394 "
    "p.4-56; ME0384 p.5-67): not supported by this driver"
)
ROBONET_SW2 = (
    "the ROBONET gateway is in gateway mode (SW2 OFF, slave 3Fh): set SW2 ON (SIO through mode) for direct access "
    "to the axes at axis number + 1, then open the port with line='robonet' (DOC ME0208 p.30, p.118)"
)


# ====================================================================== errors


class GatewayModeError(EcNotReadyError):
    """The RCON gateway is not in MANU mode (or did not answer), so Modbus control is not possible (ME0413 p.6-1 #5)."""


class PositionUnitError(EcNotReadyError):
    """An mm-based command on an RCON driver unit whose position unit (No.207) is not known. Nothing was sent."""


class UnsupportedLineError(EcNotReadyError):
    """The line gives no axis access the driver supports (ROBONET gateway mode, an EC behind an RCON-EC / REC unit)."""


# ====================================================================== line rules


@dataclass(frozen=True)
class LineRules:
    """What one kind of line carries. ``function_codes`` is the documented list (``None``: no restriction known).

    The bus refuses only **writes** (and broadcasts) outside the list; a read of an undocumented function (e.g. FC11
    on RCON) still goes out: a read is harmless and the unit may answer it.
    """

    kind: str
    title: str
    broadcast: bool
    iai_commands: bool
    function_codes: frozenset[int] | None
    command_strategy: str | None
    requires_manu: bool
    requires_position_unit: bool
    axis_access: bool
    max_write_registers: int
    probe_timeout_s: float
    min_frame_timeout_s: float
    baud_range: tuple[int, int] | None
    gateway_addresses: tuple[int, ...]
    directory_timeout_means_absent: bool
    evidence: str

    def allows_function(self, function: int) -> bool:
        """True if ``function`` is documented on this line (or the line has no documented list)."""
        return self.function_codes is None or function in self.function_codes

    @property
    def coils(self) -> bool:
        """FC05 is carried (False on RCON and in ROBONET gateway mode)."""
        return self.allows_function(_FC_COIL)

    @property
    def report_slave_id(self) -> bool:
        """FC11 is documented (False on RCON: identify the unit by :func:`read_serial_code` instead)."""
        return self.allows_function(_FC_SLAVE_ID)

    def baud_allowed(self, rate: int) -> bool:
        """``rate`` is inside the line's documented host baud range (always True when none is documented)."""
        return self.baud_range is None or self.baud_range[0] <= rate <= self.baud_range[1]

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "title": self.title,
            "broadcast": self.broadcast,
            "iai_commands": self.iai_commands,
            "function_codes": None if self.function_codes is None else sorted(self.function_codes),
            "command_strategy": self.command_strategy,
            "requires_manu": self.requires_manu,
            "requires_position_unit": self.requires_position_unit,
            "axis_access": self.axis_access,
            "max_write_registers": self.max_write_registers,
            "probe_timeout_s": self.probe_timeout_s,
            "min_frame_timeout_s": self.min_frame_timeout_s,
            "baud_range": self.baud_range,
            "gateway_addresses": list(self.gateway_addresses),
            "directory_timeout_means_absent": self.directory_timeout_means_absent,
            "evidence": self.evidence,
        }


# ====================================================================== RCON gateway


@dataclass(frozen=True)
class RconGatewayStatus:
    """The RCON gateway status block 0x3700-0x370A (DOC ME0413 p.5-2, 5-3), as read. READ-ONLY.

    ``linked_axes`` decodes NLNK (0x3703, active low) as bit n = axis n: the bit order is not stated in the manual
    (UNVERIFIED). ``auto_manu_status`` is GSR2 b2 ATST (its polarity is not stated: UNVERIFIED).
    """

    words: tuple[int, ...]
    read_at: float = 0.0

    def _w(self, offset: int) -> int:
        return self.words[offset] if offset < len(self.words) else 0

    @property
    def gwsr(self) -> int:
        return self._w(0)

    @property
    def gsr2(self) -> int:
        return self._w(1)

    @property
    def nlnk(self) -> int:
        return self._w(3)

    @property
    def manu(self) -> bool:
        """GWSR b15 GWMD: 1 = MANU (Modbus control possible), 0 = AUTO (field network / PLC)."""
        return bool(self.gwsr & 0x8000)

    @property
    def monitor_active(self) -> bool:
        """GWSR b14 MONS: field-network monitoring active."""
        return bool(self.gwsr & 0x4000)

    @property
    def link_error(self) -> bool:
        """GWSR b13 ERRT or b12 ERRC (communication errors; ERRC from firmware V00006)."""
        return bool(self.gwsr & 0x3000)

    @property
    def auto_manu_status(self) -> bool:
        """GSR2 b2 ATST (UNVERIFIED polarity)."""
        return bool(self.gsr2 & 0x0004)

    @property
    def emergency_stop(self) -> bool:
        """GSR2 b1 EMST: the emergency-stop input is active."""
        return bool(self.gsr2 & 0x0002)

    @property
    def system_timer_ms(self) -> int:
        """STIM 0x3704 (u32)."""
        return R.from_u32_words(self._w(4), self._w(5))

    @property
    def alarm_code(self) -> int:
        """Gateway alarm code ALMC 0x3708 (u32), e.g. 0x8BA "axes / operation mode mismatch"; 0 = none."""
        return R.from_u32_words(self._w(8), self._w(9))

    @property
    def alarm_name(self) -> str | None:
        if not self.alarm_code:
            return None
        from ..alarms import alarm_name

        return alarm_name(self.alarm_code)

    @property
    def linked_axes(self) -> tuple[int, ...]:
        """Axis numbers whose NLNK bit is 0 (linked). Bit order UNVERIFIED (bit n = axis n assumed)."""
        return tuple(n for n in range(RCON_MAX_AXES) if not self.nlnk & (1 << n))

    @property
    def linked_slaves(self) -> tuple[int, ...]:
        """Slave addresses (axis number + 1) of :attr:`linked_axes`."""
        return tuple(n + 1 for n in self.linked_axes)

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": LINE_RCON,
            "words": [f"0x{w:04X}" for w in self.words],
            "manu": self.manu,
            "monitor_active": self.monitor_active,
            "link_error": self.link_error,
            "emergency_stop": self.emergency_stop,
            "linked_axes": list(self.linked_axes),
            "alarm_code": self.alarm_code,
            "alarm_name": self.alarm_name,
            "system_timer_ms": self.system_timer_ms,
        }


@dataclass(frozen=True)
class RconAxisMonitor:
    """One axis's record of the RCON all-axes monitor (15 words at 0x2900 + 15 x axis, DOC ME0413 p.5-2).

    Positions are raw counts in the unit's position unit (No.207: 0.01 or 0.001 mm); speeds are 0.01 mm/s.
    The record has no DSS2, ZONS or POSS: read the axis's own 0x9000 block for those.
    """

    axis: int
    words: tuple[int, ...]

    @property
    def slave(self) -> int:
        return self.axis + 1

    @property
    def position_counts(self) -> int:
        """PNOW (s32), in the unit's position unit."""
        return R.from_i32_words(self.words[0], self.words[1])

    @property
    def velocity_counts(self) -> int:
        """VNOW (s32), 0.01 mm/s."""
        return R.from_i32_words(self.words[2], self.words[3])

    @property
    def current_ma(self) -> int:
        """CNOW (mA; command current on RCON-PC, feedback current on AC/DC/SC)."""
        return R.from_i32_words(self.words[4], self.words[5])

    @property
    def alarm_code(self) -> int:
        return self.words[6]

    @property
    def dss1(self) -> int:
        return self.words[7]

    @property
    def dsse(self) -> int:
        return self.words[8]

    @property
    def sipm(self) -> int:
        return self.words[9]

    @property
    def overload_pct(self) -> int:
        """OLLV, overload level in %."""
        return self.words[10]

    @property
    def deviation_pulses(self) -> int:
        """DEVI (s32), pulses."""
        return R.from_i32_words(self.words[11], self.words[12])

    @property
    def stat(self) -> int:
        """STAT (u32)."""
        return R.from_u32_words(self.words[13], self.words[14])

    def status_words(self) -> list[int]:
        """The record in the 14-word layout of the axis's own 0x9000 block (PNOW .. CNOW), for ``Status`` decoding.

        DIPM, DOPM and DSS2 are not in the gateway record and are given as 0.
        """
        w = self.words
        return [w[0], w[1], w[6], 0, 0, w[7], 0, w[8], w[13], w[14], w[2], w[3], w[4], w[5]]

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis,
            "slave": self.slave,
            "position_counts": self.position_counts,
            "velocity_counts": self.velocity_counts,
            "current_ma": self.current_ma,
            "alarm_code": self.alarm_code,
            "dss1": self.dss1,
            "dsse": self.dsse,
            "sipm": self.sipm,
            "overload_pct": self.overload_pct,
            "deviation_pulses": self.deviation_pulses,
            "stat": self.stat,
        }


# ====================================================================== ROBONET gateway (gateway mode)


@dataclass(frozen=True)
class RoboNetGatewayStatus:
    """ROBONET gateway status words 0xF700 / 0xF701 (DOC ME0208 p.40-52, p.128-129). READ-ONLY.

    Example reply ``3F 03 04 80 21 00 03 1C 3B``: RUN, one 4-word axis, one 8-word axis, LNK0 + LNK1.
    """

    words: tuple[int, ...]
    read_at: float = 0.0

    @property
    def status0(self) -> int:
        return self.words[0] if self.words else 0

    @property
    def link(self) -> int:
        return self.words[1] if len(self.words) > 1 else 0

    @property
    def running(self) -> bool:
        """b15 RUN: the gateway runs normally."""
        return bool(self.status0 & 0x8000)

    @property
    def manu(self) -> bool:
        """b12 MOD: the front switch is at MANU."""
        return bool(self.status0 & 0x1000)

    @property
    def direct_axes(self) -> int:
        """b9-b5: number of direct-numerical (8-word) axes."""
        return (self.status0 >> 5) & 0x1F

    @property
    def positioner_axes(self) -> int:
        """b4-b0: number of positioner 1 / simple direct (4-word) axes."""
        return self.status0 & 0x1F

    @property
    def linked_axes(self) -> tuple[int, ...]:
        """Axis numbers whose LNK bit is set (bit n = axis n linked)."""
        return tuple(n for n in range(16) if self.link & (1 << n))

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": LINE_ROBONET_GATEWAY,
            "words": [f"0x{w:04X}" for w in self.words],
            "running": self.running,
            "manu": self.manu,
            "direct_axes": self.direct_axes,
            "positioner_axes": self.positioner_axes,
            "linked_axes": list(self.linked_axes),
            "explanation": ROBONET_SW2,
        }


publish(globals(), "gateway")
