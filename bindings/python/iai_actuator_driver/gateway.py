"""Gateway lines: what may be sent on a serial line, and the read-only gateway objects (RCON, ROBONET).

Every :class:`~iai_actuator_driver.modbus.ModbusRTU` carries a :class:`Line` (``bus.line``). On a plain line (an EC or a CON
controller on its own SIO port, or an SIO daisy chain) everything the driver knows is allowed. Behind a gateway the
line carries less, and the bus refuses the rest **before anything is sent** (:class:`LineRuleError`):

=====================  =========  =========  =====  ================================================================
line kind              broadcast  0x45/0x48  FC05   other rules
=====================  =========  =========  =====  ================================================================
``"direct"``           yes        yes        yes    one unit per port VERIFIED (EC, PCON-CB); daisy chain UNVERIFIED
``"rcon"``             no         no         no     FC 01/03/04/06/10/17 only; gateway in MANU before control and
                                                    motion; position unit (No.207) known before mm commands;
                                                    command strategy ``"register"`` (DRG1 over FC06)
``"robonet"``          no         no         yes    SIO through mode (SW2 ON): FC 01-07/0F/10/11/17; FC10 <= 75
                                                    registers (160-byte buffer); host 38.4-230.4 kbps; strategy
                                                    ``"coil"``; a timed-out 0x0000 directory read = no directory
``"robonet-gateway"``  no         no         no     gateway mode (SW2 OFF, slave 3Fh): no axis access at all
=====================  =========  =========  =====  ================================================================

Sources: RCON DOC ME0413 p.3-1 (function codes), p.4-1 (addresses; "RCON-EC unit is not applicable"), p.6-1 #2 (no
broadcast), #5 (Modbus control in MANU mode); ME0384 p.8-56 (No.207 position unit). ROBONET DOC ME0208 p.26 (through
mode: FC list, 38.4-230.4 kbps, 160-byte buffer; gateway mode: 3Fh, FC 03/06/10), p.30 / p.118 (SW2), p.340 (fixed
230.4 kbps internal link). Every gateway rule is DOC only: no RCON or ROBONET has been tested with this driver
(UNVERIFIED on hardware).

Detection (:func:`detect_line`, run by :class:`~iai_actuator_driver.cell.Port` at the first scan or axis): two read-only FC03
probes, each waiting at most :data:`GATEWAY_PROBE_TIMEOUT_S` (0.05 s), so a normal line pays about 0.1 s once:

* ``D0 03 37 00 00 0B 18 38`` - the RCON gateway status block 0x3700 (GWSR .. NPLK, ME0413 p.5-2, 5-3);
* ``3F 03 F7 00 00 02 F2 A1`` - the ROBONET gateway status words 0xF700 (ME0208 p.128-129).

Only a normal reply (not an exception) counts. ROBONET **through** mode is transparent and cannot be probed: open the
port with ``line="robonet"`` (:func:`robonet_hint` gives an UNVERIFIED hint).

Interface for the axis class (``ax.bus.line`` is always a :class:`Line`)::

    line = ax.bus.line
    line.kind                              # "direct" | "rcon" | "robonet" | "robonet-gateway"
    line.allows(OP_SOFTWARE_RESET)         # bool: is it carried here
    line.require(OP_SOFTWARE_RESET)        # LineRuleError with a line-specific message (nothing sent)
    line.command_strategy                  # "register" (RCON) | "coil" (ROBONET) | None (the axis's own choice)
    line.require_control(slave)            # before taking control: RCON gateway in MANU, ROBONET gateway mode refused
    line.require_motion(slave) -> float    # before every motion command: control + the position unit (mm per count)
    line.position_unit_mm(slave)           # 0.01 / 0.001 / None (RCON, not stated)
    line.rules.report_slave_id             # False on RCON: use read_serial_code(bus, slave) (0x9400) instead of FC11
    line.rules.directory_timeout_means_absent  # ROBONET / RCON: a timeout on the 0x0000 read = "no directory"

On the core: :class:`Line` is a C++ ``Line`` (iad_line), :class:`RconGateway`
and :class:`RoboNetGateway` C++ gateway objects (iad_gateway) reading through the core bus of the
:class:`~iai_actuator_driver.modbus.ModbusRTU` they were given. The core decides: the line rules and their refusals,
the request guard, the MANU check and its cache, the position units, the probes and the detection. This module
converts arguments and results:

* ``bus.line = line`` attaches the core line itself (iad_bus_attach_line): its gateway, its position units and the
  axes' line rules then serve every core axis on that bus, so ``bus.line.set_position_unit(...)`` reaches the axes;
* :func:`detect_line` runs the core's detection on the bus and hands back the line it attached, with its gateway;
* the rules (:data:`RULES`) are the core's, read once (iad_line_rules_json); the status records
  (:class:`RconGatewayStatus`, :class:`RconAxisMonitor`, :class:`RoboNetGatewayStatus`), the constants and the error
  classes are the shared ones (``_shared/gateway.py``, no I/O in them);
* ``bus.line_rules`` (the duck-typed rules a caller states for the axes) becomes the core line's stated
  rules (iad_bus_state_line_rules, read by :func:`_stated_rules`).

Every gateway rule is DOC only (ME0413, ME0208): UNVERIFIED on hardware.
"""

from __future__ import annotations

import ctypes
import json
import logging
import math
import weakref
from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING, Any

from . import native as _n
from ._shared.gateway import (  # the shared names (constants, records, errors)
    EC_BEHIND_GATEWAY,
    GATEWAY_FIRST_CONTACT_S,
    GATEWAY_FRAME_TIMEOUT_S,
    GATEWAY_PROBE_TIMEOUT_S,
    GWSR_CACHE_S,
    LINE_DIRECT,
    LINE_KINDS,
    LINE_RCON,
    LINE_ROBONET,
    LINE_ROBONET_GATEWAY,
    OP_ACTION_MODE,
    OP_AXIS_CONTROL,
    OP_BAUD_CHANGE,
    OP_BROADCAST,
    OP_COIL,
    OP_HOME_IAI,
    OP_IAI_COMMAND,
    OP_IAI_COMMAND2,
    OP_REPORT_SLAVE_ID,
    OP_SOFTWARE_RESET,
    OPERATIONS,
    POSITION_UNITS_MM,
    RCON_GATEWAY_ADDRESSES,
    RCON_GATEWAY_SLAVE,
    RCON_GW_ALARM,
    RCON_GW_STATUS,
    RCON_GW_STATUS_WORDS,
    RCON_MAX_AXES,
    RCON_MONITOR_AXES_PER_READ,
    RCON_MONITOR_BASE,
    RCON_MONITOR_STRIDE,
    RCON_SERIAL_BASE,
    RCON_SERIAL_WORDS,
    ROBONET_GATEWAY_SLAVE,
    ROBONET_GW_STATUS,
    ROBONET_GW_STATUS_WORDS,
    ROBONET_PROBE_TIMEOUT_S,
    ROBONET_SW2,
    GatewayModeError,
    LineRules,
    PositionUnitError,
    RconAxisMonitor,
    RconGatewayStatus,
    RoboNetGatewayStatus,
    UnsupportedLineError,
)
from .errors import (  # noqa: F401 (re-exported)
    EcNotReadyError,
    LineRuleError,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
)

if TYPE_CHECKING:
    from .modbus import ModbusRTU

__all__ = [
    "LINE_DIRECT",
    "LINE_RCON",
    "LINE_ROBONET",
    "LINE_ROBONET_GATEWAY",
    "LINE_KINDS",
    "OP_BROADCAST",
    "OP_BAUD_CHANGE",
    "OP_IAI_COMMAND",
    "OP_IAI_COMMAND2",
    "OP_SOFTWARE_RESET",
    "OP_ACTION_MODE",
    "OP_HOME_IAI",
    "OP_COIL",
    "OP_REPORT_SLAVE_ID",
    "OP_AXIS_CONTROL",
    "OPERATIONS",
    "RCON_GATEWAY_SLAVE",
    "RCON_GATEWAY_ADDRESSES",
    "RCON_GW_STATUS",
    "RCON_MONITOR_BASE",
    "RCON_SERIAL_BASE",
    "ROBONET_GATEWAY_SLAVE",
    "ROBONET_GW_STATUS",
    "ROBONET_SW2",
    "GWSR_CACHE_S",
    "GATEWAY_PROBE_TIMEOUT_S",
    "GATEWAY_FRAME_TIMEOUT_S",
    "GATEWAY_FIRST_CONTACT_S",
    "ROBONET_PROBE_TIMEOUT_S",
    "POSITION_UNITS_MM",
    "EC_BEHIND_GATEWAY",
    "LineRuleError",
    "GatewayModeError",
    "PositionUnitError",
    "UnsupportedLineError",
    "LineRules",
    "RULES",
    "Line",
    "RconGatewayStatus",
    "RconAxisMonitor",
    "RconGateway",
    "RoboNetGatewayStatus",
    "RoboNetGateway",
    "probe_rcon_gateway",
    "probe_robonet_gateway",
    "detect_line",
    "read_serial_code",
    "robonet_hint",
    "line_rules_table",
]

log = logging.getLogger("iai_actuator_driver.gateway")  # the public module name, on every backend

_SHARED = (GatewayModeError, PositionUnitError, UnsupportedLineError, LineRules, RconGatewayStatus, RconAxisMonitor,
           RoboNetGatewayStatus)  # fmt: skip
for _shared in _SHARED:  # reported (and pickled) under the public module name
    _shared.__module__ = "iai_actuator_driver.gateway"

_n.NOT_READY_KINDS.update(  # the core's not-ready kinds of a line: raised as these classes by every module
    GatewayModeError=GatewayModeError, PositionUnitError=PositionUnitError, UnsupportedLineError=UnsupportedLineError
)

_KIND_CODES = {LINE_DIRECT: 1, LINE_RCON: 2, LINE_ROBONET: 3, LINE_ROBONET_GATEWAY: 4}  # IAD_LINE_*
_GATEWAY_WORDS = 16  # room for either gateway's status (RCON 11 words, ROBONET 2)


def _rules() -> dict[str, LineRules]:
    """Every kind's rules as the core states them (iad_line_rules_json, in IAD_LINE_DIRECT .. order)."""
    out: dict[str, LineRules] = {}
    for r in _n._read_json(_n.load(), "iad_line_rules_json"):
        codes, baud = r.get("function_codes"), r.get("baud_range")
        out[str(r["kind"])] = LineRules(
            kind=str(r["kind"]), title=str(r["title"]), broadcast=bool(r["broadcast"]),
            iai_commands=bool(r["iai_commands"]), function_codes=None if codes is None else frozenset(codes),
            command_strategy=r.get("command_strategy"), requires_manu=bool(r["requires_manu"]),
            requires_position_unit=bool(r["requires_position_unit"]), axis_access=bool(r["axis_access"]),
            max_write_registers=int(r["max_write_registers"]), probe_timeout_s=float(r["probe_timeout_s"]),
            min_frame_timeout_s=float(r["min_frame_timeout_s"]), baud_range=None if baud is None else tuple(baud),
            gateway_addresses=tuple(r["gateway_addresses"]),
            directory_timeout_means_absent=bool(r["directory_timeout_means_absent"]), evidence=str(r["evidence"]),
        )  # fmt: skip
    return {k: out[k] for k in LINE_KINDS}


RULES: dict[str, LineRules] = _rules()


def _bus_handle(bus: Any) -> int:
    """The core bus of a :class:`~iai_actuator_driver.modbus.ModbusRTU`; a closed one refuses."""
    handle = getattr(bus, "_handle", None)
    if handle is None:
        raise TypeError(f"bus must be a ModbusRTU of iai_actuator_driver.modbus, got {type(bus).__name__}")
    if getattr(bus, "closed", False):
        raise ModbusError("bus is closed")
    return int(handle)


def _slaves_text(slaves: Iterable[int]) -> bytes | None:
    return _n._utf8(",".join(str(int(s)) for s in slaves))


def _words_out(lib: Any, name: str, *args: Any) -> tuple[tuple[int, ...], float] | None:
    """A gateway status the core read: (words, read_at), ``None`` when it wrote none (a probe nobody answered)."""
    words = (ctypes.c_uint16 * _GATEWAY_WORDS)()
    count, read_at = ctypes.c_size_t(0), ctypes.c_double(0.0)
    _n._check(lib, name, *args, words, _GATEWAY_WORDS, ctypes.byref(count), ctypes.byref(read_at))
    if not count.value:
        return None
    return tuple(int(w) for w in words[: count.value]), float(read_at.value)


# ====================================================================== RCON gateway


class _GatewayHandle:
    """What both gateway objects share: the core gateway (iad_gateway) and its last status."""

    _handle: int

    def _adopt(self, lib: Any, handle: int) -> None:
        self._lib = lib
        self._handle = handle
        self._finalizer = weakref.finalize(self, _n._release, lib, "iad_gateway_close", handle)

    def _info(self) -> dict[str, Any]:
        return _n._read_json(self._lib, "iad_gateway_info_json", self._handle)  # type: ignore[no-any-return]

    def _last(self) -> tuple[tuple[int, ...], float] | None:
        info = self._info()
        words = info.get("words")
        if words is None:
            return None
        read_at = info.get("read_at")
        return tuple(int(w) for w in words), float(read_at if read_at is not None else 0.0)

    def _read(self, max_age_s: float | None) -> tuple[tuple[int, ...], float]:
        _bus_handle(self.bus)  # type: ignore[attr-defined]
        got = _words_out(self._lib, "iad_gateway_status", self._handle, _n._num(max_age_s))
        if got is None:  # pragma: no cover - the core always returns a status or raises
            raise ModbusError(f"{self!r}: no status read")
        return got


class RconGateway(_GatewayHandle):
    """The RCON gateway (slave D0h) as a read-only object: status, MANU flag, alarm, link map, all-axes monitor.

    Never written by this driver (DOC ME0413 p.5-2 to 5-9; UNVERIFIED on hardware). The MANU flag is cached for
    :data:`GWSR_CACHE_S` seconds; a refusal always re-reads it first.
    """

    kind = LINE_RCON

    def __init__(
        self, bus: ModbusRTU, address: int = RCON_GATEWAY_SLAVE, *, status: RconGatewayStatus | None = None,
        cache_s: float = GWSR_CACHE_S,
    ) -> None:  # fmt: skip
        lib = _n.load()
        known = () if status is None else tuple(status.words)
        words = (ctypes.c_uint16 * max(len(known), 1))(*known)
        read_at = math.nan if status is None else float(status.read_at)
        out = ctypes.c_void_p()
        _n._check(lib, "iad_gateway_open", _bus_handle(bus), _KIND_CODES[LINE_RCON], int(address), words, len(known),
                  read_at, float(cache_s), ctypes.byref(out))  # fmt: skip
        self.bus = bus
        self.address = address
        self._adopt(lib, int(out.value or 0))

    @classmethod
    def _of(cls, lib: Any, handle: int, bus: ModbusRTU) -> RconGateway:
        """The object of a core gateway the line already has (detection)."""
        gw = cls.__new__(cls)
        gw.bus = bus
        gw._adopt(lib, handle)
        gw.address = int(gw._info()["address"])
        return gw

    def __repr__(self) -> str:
        return f"RconGateway(0x{self.address:02X})"

    @property
    def cache_s(self) -> float:
        """How long a read of the gateway status (the MANU flag) is trusted before it is read again (s)."""
        return float(self._info()["cache_s"])

    @cache_s.setter
    def cache_s(self, value: float) -> None:
        _n._check(self._lib, "iad_gateway_set_cache_s", self._handle, float(value))

    @property
    def last_status(self) -> RconGatewayStatus | None:
        """The last status read (``None`` before the first read)."""
        last = self._last()
        return None if last is None else RconGatewayStatus(*last)

    def read_status(self) -> RconGatewayStatus:
        """FC03 ``D0 03 37 00 00 0B``: GWSR .. NPLK. READ-ONLY."""
        return RconGatewayStatus(*self._read(-1.0))

    def status(self, max_age_s: float | None = None) -> RconGatewayStatus:
        """The cached status when younger than ``max_age_s`` (default :attr:`cache_s`), else a fresh read."""
        return RconGatewayStatus(*self._read(max_age_s))

    def read_alarm(self) -> int:
        """FC03 ``D0 03 37 08 00 02``: the gateway alarm code (u32; 0 = none). READ-ONLY."""
        _bus_handle(self.bus)
        code = ctypes.c_uint32(0)
        _n._check(self._lib, "iad_gateway_read_alarm", self._handle, ctypes.byref(code))
        return int(code.value)

    def require_manu(self) -> RconGatewayStatus:
        """Raise :class:`GatewayModeError` unless the gateway reports MANU (GWSR b15 GWMD = 1).

        A cached AUTO is re-read before refusing; a gateway that does not answer is a refusal too.
        """
        got = _words_out(self._lib, "iad_gateway_require_manu", self._handle)
        if got is None:  # pragma: no cover - the core always returns a status or raises
            raise GatewayModeError(f"{self!r}: no status read")
        return RconGatewayStatus(*got)

    def read_monitor(self, axes: Iterable[int] | None = None) -> dict[int, RconAxisMonitor]:
        """Read the all-axes monitor for ``axes`` (axis numbers 0-15; default all 16) in as few frames as possible.

        At most 8 axes (120 words) per FC03 frame, e.g. ``D0 03 29 00 00 78 5F F5`` + ``D0 03 29 78 00 78 DF EC``
        for all 16. Returns ``{axis: RconAxisMonitor}``. READ-ONLY.
        """
        wanted = sorted(set(range(RCON_MAX_AXES) if axes is None else axes))
        for a in wanted:
            if not 0 <= a < RCON_MAX_AXES:
                raise ValueError(f"RCON axis number must be 0..{RCON_MAX_AXES - 1}, got {a}")
        if not wanted:
            return {}
        _bus_handle(self.bus)
        records = _n._read_json(self._lib, "iad_gateway_read_monitor_json", self._handle, _slaves_text(wanted))
        return {int(a): RconAxisMonitor(int(a), tuple(int(w) for w in words)) for a, words in records.items()}


# ====================================================================== ROBONET gateway (gateway mode)


class RoboNetGateway(_GatewayHandle):
    """A ROBONET gateway in gateway mode (3Fh): read-only status only. The driver never writes it (UNVERIFIED)."""

    kind = LINE_ROBONET_GATEWAY

    def __init__(self, bus: ModbusRTU, *, status: RoboNetGatewayStatus | None = None) -> None:
        lib = _n.load()
        known = () if status is None else tuple(status.words)
        words = (ctypes.c_uint16 * max(len(known), 1))(*known)
        read_at = math.nan if status is None else float(status.read_at)
        out = ctypes.c_void_p()
        _n._check(lib, "iad_gateway_open", _bus_handle(bus), _KIND_CODES[LINE_ROBONET_GATEWAY], 0, words, len(known),
                  read_at, math.nan, ctypes.byref(out))  # fmt: skip
        self.bus = bus
        self.address = ROBONET_GATEWAY_SLAVE
        self._adopt(lib, int(out.value or 0))

    @classmethod
    def _of(cls, lib: Any, handle: int, bus: ModbusRTU) -> RoboNetGateway:
        """The object of a core gateway the line already has (detection)."""
        gw = cls.__new__(cls)
        gw.bus = bus
        gw.address = ROBONET_GATEWAY_SLAVE
        gw._adopt(lib, handle)
        return gw

    def __repr__(self) -> str:
        return "RoboNetGateway(0x3F)"

    @property
    def last_status(self) -> RoboNetGatewayStatus | None:
        last = self._last()
        return None if last is None else RoboNetGatewayStatus(*last)

    def read_status(self) -> RoboNetGatewayStatus:
        """FC03 ``3F 03 F7 00 00 02``. READ-ONLY."""
        return RoboNetGatewayStatus(*self._read(-1.0))

    def status(self, max_age_s: float | None = None) -> RoboNetGatewayStatus:
        return RoboNetGatewayStatus(*self._read(max_age_s))


Gateway = RconGateway | RoboNetGateway


def _gateway_of(lib: Any, line_handle: int, bus: ModbusRTU) -> Gateway | None:
    """The object of the gateway a core line has (``None`` without one)."""
    out = ctypes.c_void_p()
    _n._check(lib, "iad_line_gateway", line_handle, ctypes.byref(out))
    handle = int(out.value or 0)
    if not handle:
        return None
    kind = _n._read_json(lib, "iad_gateway_info_json", handle)["kind"]
    return (RconGateway if kind == LINE_RCON else RoboNetGateway)._of(lib, handle, bus)


# ====================================================================== the line


class Line:
    """What one serial line carries, and the gateway behind it (if any). Attached to the bus as ``bus.line``.

    Args:
        kind: one of :data:`LINE_KINDS`.
        gateway: the gateway object found by :func:`detect_line` (``None`` when none answered).
        detected: True when a probe found the gateway; False when the kind was stated or is the default.
        position_unit_mm: the position unit of every axis on the line (RCON: 0.01 or 0.001), if stated.
    """

    def __init__(
        self,
        kind: str = LINE_DIRECT,
        *,
        gateway: Gateway | None = None,
        detected: bool = False,
        position_unit_mm: float | None = None,
    ) -> None:
        if kind not in RULES:
            raise ValueError(f"line kind must be one of {', '.join(LINE_KINDS)}, got {kind!r}")
        lib = _n.load()
        out = ctypes.c_void_p()
        _n._check(lib, "iad_line_open", _KIND_CODES[kind], getattr(gateway, "_handle", None), int(bool(detected)),
                  _n._num(position_unit_mm), ctypes.byref(out))  # fmt: skip
        self._adopt(lib, int(out.value or 0))
        self.rules: LineRules = RULES[kind]
        self.gateway = gateway
        self.detected = detected

    def _adopt(self, lib: Any, handle: int) -> None:
        self._lib = lib
        self._handle = handle
        self._finalizer = weakref.finalize(self, _n._release, lib, "iad_line_close", handle)

    @classmethod
    def _of(cls, lib: Any, handle: int, bus: ModbusRTU) -> Line:
        """The object of a core line (the one detection attached to the bus), with its gateway."""
        line = cls.__new__(cls)
        line._adopt(lib, handle)
        state = line._snapshot()
        line.rules = RULES[str(state["kind"])]
        line.gateway = _gateway_of(lib, handle, bus)
        line.detected = bool(state["detected"])
        return line

    def _attach(self, bus_handle: int) -> None:
        """Make this the line of a core bus (``ModbusRTU.line = line``): its guard, gateway and position units serve
        every axis on it."""
        _n._check(self._lib, "iad_bus_attach_line", bus_handle, self._handle)

    def _snapshot(self) -> dict[str, Any]:
        return _n._read_json(self._lib, "iad_line_json", self._handle)  # type: ignore[no-any-return]

    def __repr__(self) -> str:
        how = "detected" if self.detected else "stated"
        return f"Line({self.kind!r}, {how}, gateway={self.gateway!r})"

    # ------------------------------------------------------------------ basics

    @property
    def kind(self) -> str:
        return self.rules.kind

    @property
    def is_gateway(self) -> bool:
        """True on every line except ``"direct"``."""
        return self.kind != LINE_DIRECT

    @property
    def command_strategy(self) -> str | None:
        """The command strategy the line requires: ``"register"`` (RCON), ``"coil"`` (ROBONET) or ``None``."""
        return self.rules.command_strategy

    @property
    def max_write_registers(self) -> int:
        return self.rules.max_write_registers

    def _operation(self, operation: str) -> bytes | None:
        if operation not in OPERATIONS:
            raise ValueError(f"unknown line operation {operation!r}; choose from {', '.join(OPERATIONS)}")
        return _n._utf8(operation)

    def allows(self, operation: str) -> bool:
        """Whether ``operation`` (an ``OP_*`` name) is carried / documented on this line."""
        flag = _n._c_bool(0)
        _n._check(self._lib, "iad_line_allows", self._handle, self._operation(operation), ctypes.byref(flag))
        return bool(flag.value)

    def require(self, operation: str, *, what: str | None = None) -> None:
        """Raise :class:`LineRuleError` (nothing sent) with a line-specific message if ``operation`` is not carried."""
        _n._check(self._lib, "iad_line_require", self._handle, self._operation(operation), _n._utf8(what))

    def check_request(self, slave: int, pdu: bytes) -> None:
        """The bus's guard, run before a frame is sent: raise :class:`LineRuleError` if the line does not carry it.

        Refused on a gateway line: any broadcast; IAI functions 0x45 / 0x48; a write function outside the documented
        list (FC05 on RCON); any write in ROBONET gateway mode; any write to the gateway's own address; an FC10
        longer than the line's buffer allows. Reads always pass.
        """
        data = bytes(pdu)
        buffer = (ctypes.c_uint8 * max(len(data), 1)).from_buffer_copy(data or b"\0")
        _n._check(self._lib, "iad_line_check_request", self._handle, int(slave), buffer, len(data))

    # ------------------------------------------------------------------ control and motion preconditions

    def require_control(self, slave: int | None = None) -> None:
        """Run before taking control of an axis and before every motion command.

        ROBONET gateway mode: :class:`UnsupportedLineError` (no axis access). RCON: the gateway must report MANU
        (:class:`GatewayModeError`; the status read is cached for :data:`GWSR_CACHE_S`, an AUTO is re-read before
        refusing). Other lines: nothing to check.
        """
        _n._check(self._lib, "iad_line_require_control", self._handle, 0 if slave is None else int(slave))

    def position_unit_mm(self, slave: int | None = None) -> float | None:
        """The position unit (mm per count) of ``slave``: stated / read for that slave, else for the whole line.

        ``direct`` and ``robonet`` lines: 0.01 (the CON / EC unit, ``registers.POSITION_SCALE``) unless stated.
        RCON: ``None`` until stated (No.207 is not readable over Modbus).
        """
        return self._unit(slave)[0]

    def position_unit_source(self, slave: int | None = None) -> str | None:
        """Where :meth:`position_unit_mm` comes from: ``"stated"``, ``"read"``, ``"default"`` or ``None`` (unknown)."""
        return self._unit(slave)[1]

    def _unit(self, slave: int | None) -> tuple[float | None, str | None]:
        unit = ctypes.c_double(math.nan)
        buffer = ctypes.create_string_buffer(64)
        required = ctypes.c_size_t(0)
        _n._check(self._lib, "iad_line_position_unit", self._handle, 0 if slave is None else int(slave),
                  ctypes.byref(unit), buffer, 64, ctypes.byref(required))  # fmt: skip
        source = buffer.value.decode("utf-8", errors="replace")
        return _n._opt(unit.value), source or None

    def set_position_unit(self, unit_mm: float, slave: int | None = None, *, source: str = "stated") -> None:
        """Record the position unit (0.01 or 0.001 mm) of ``slave`` (``None`` = every axis on the line).

        ``source`` is ``"stated"`` (by the caller) or ``"read"`` (the axis read No.207 itself). A direct line refuses
        any unit but the unit's own 0.01 (``ValueError``; ON-WIRE on a PCON-CB, a stated 0.001 read 94.99 mm as 9.499).
        """
        _n._check(self._lib, "iad_line_set_position_unit", self._handle, float(unit_mm),
                  0 if slave is None else int(slave), _n._utf8(source))  # fmt: skip

    def require_position_unit(self, slave: int) -> float:
        """The position unit of ``slave``; :class:`PositionUnitError` on RCON when it is not known (nothing sent)."""
        return self._require_unit(slave, control=False)

    def require_motion(self, slave: int) -> float:
        """:meth:`require_control` plus :meth:`require_position_unit`; returns the position unit (mm per count)."""
        return self._require_unit(slave, control=True)

    def _require_unit(self, slave: int, *, control: bool) -> float:
        unit = ctypes.c_double(math.nan)
        _n._check(self._lib, "iad_line_require_position_unit", self._handle, int(slave), int(control),
                  ctypes.byref(unit))  # fmt: skip
        return float(unit.value)

    # ------------------------------------------------------------------ explanations

    def note_unreachable(self, slaves: Iterable[int]) -> None:
        """Record axes the gateway reports as linked that do not answer Modbus (e.g. ECs behind an RCON-EC unit)."""
        items = list(slaves)
        if items:
            _n._check(self._lib, "iad_line_note_unreachable", self._handle, _slaves_text(items))

    @property
    def unreachable_slaves(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self._snapshot()["unreachable_slaves"])

    def explain_unreachable(self, slaves: Iterable[int] | None = None) -> str | None:
        """Why linked axes (default: :attr:`unreachable_slaves`) do not answer, or ``None`` on a non-gateway line."""
        if slaves is not None:
            slaves = list(slaves)
            if not slaves:
                return None
        text = _n._read_text(self._lib, "iad_line_explain_unreachable", self._handle,
                             None if slaves is None else _slaves_text(slaves))  # fmt: skip
        return text or None

    def explain(self) -> str | None:
        """A one-line explanation of what this line does not support (``None`` on a direct line)."""
        explanation = self._snapshot()["explanation"]
        return None if explanation is None else str(explanation)

    def to_dict(self) -> dict[str, Any]:
        state = self._snapshot()
        state["rules"] = self.rules.to_dict()
        return state


# ====================================================================== the rules a caller states for the axes

_STATED_NAMES: dict[str, tuple[str, ...]] = {  # the names a caller's rules use for "may this line carry ..."
    "coils": ("fc05", "coils", "allow_coils", "allow_fc05"),
    "iai_commands": ("iai", "fc45", "iai_commands", "allow_iai", "allow_0x45"),
    "broadcast": ("broadcast", "allow_broadcast"),
    "report_slave_id": ("report_slave_id", "slave_id", "allow_slave_id"),
}
_STATED_OPERATIONS: dict[str, tuple[str, ...]] = {
    "coils": ("coil", "fc05"), "iai_commands": ("iai_command", "iai"), "broadcast": ("broadcast",),
    "report_slave_id": ("report_slave_id", "slave_id"),
}  # fmt: skip
_STATED_FUNCTIONS: dict[str, tuple[int, ...]] = {"coils": (0x05,), "iai_commands": (0x45, 0x48),
                                                 "report_slave_id": (0x11,)}  # fmt: skip


def _stated_rules(rules: Any) -> str | None:
    """The core's stated line rules (iad_bus_state_line_rules JSON) of a duck-typed
    ``bus.line_rules``: an object or a mapping with a callable ``allows(operation)``, boolean attributes / keys
    (``coils``, ``iai_commands``, ``broadcast`` ...), a collection ``function_codes`` and a name (``kind``, ``name``,
    ``line`` or ``gateway``); a nested ``rules`` is read too. What it does not state stays the line kind's. ``None``
    clears."""
    if rules is None:
        return None
    layers = [rules]
    nested = rules.get("rules") if isinstance(rules, Mapping) else getattr(rules, "rules", None)
    if nested is not None and not callable(nested):
        layers.append(nested)

    def get(obj: Any, name: str) -> Any:
        return obj.get(name) if isinstance(obj, Mapping) else getattr(obj, name, None)

    stated: dict[str, Any] = {}
    for key in _STATED_NAMES:
        for obj in layers:
            verdict = None
            allows = getattr(obj, "allows", None)
            if callable(allows):
                for operation in _STATED_OPERATIONS[key]:
                    try:
                        verdict = allows(operation)
                    except (TypeError, ValueError, KeyError, LookupError):
                        continue
                    if verdict is not None:
                        break
            if verdict is None:
                verdict = next((v for v in (get(obj, n) for n in _STATED_NAMES[key]) if isinstance(v, bool)), None)
            codes = get(obj, "function_codes")
            if verdict is None and codes is not None and _STATED_FUNCTIONS.get(key):
                try:
                    verdict = all(fc in codes for fc in _STATED_FUNCTIONS[key])
                except TypeError:
                    verdict = None
            if verdict is not None:
                stated[key] = bool(verdict)
                break
    for obj in layers:
        name = next((v for v in (get(obj, n) for n in ("kind", "name", "line", "gateway")) if isinstance(v, str) and v),
                    None)  # fmt: skip
        if name is not None:
            stated["name"] = name.lower()
            break
    return json.dumps(stated)


# ====================================================================== probes and detection


def probe_rcon_gateway(
    bus: ModbusRTU, *, address: int = RCON_GATEWAY_SLAVE, timeout: float = GATEWAY_PROBE_TIMEOUT_S
) -> RconGatewayStatus | None:
    """``D0 03 37 00 00 0B 18 38``: the RCON gateway status block, or ``None`` when no gateway answers. READ-ONLY."""
    got = _words_out(_n.load(), "iad_bus_probe_gateway", _bus_handle(bus), _KIND_CODES[LINE_RCON], int(address),
                     float(timeout))  # fmt: skip
    return None if got is None else RconGatewayStatus(*got)


def probe_robonet_gateway(bus: ModbusRTU, *, timeout: float = GATEWAY_PROBE_TIMEOUT_S) -> RoboNetGatewayStatus | None:
    """``3F 03 F7 00 00 02 F2 A1``: the ROBONET gateway status words, or ``None`` when none answers. READ-ONLY."""
    got = _words_out(_n.load(), "iad_bus_probe_gateway", _bus_handle(bus), _KIND_CODES[LINE_ROBONET_GATEWAY], 0,
                     float(timeout))  # fmt: skip
    return None if got is None else RoboNetGatewayStatus(*got)


def detect_line(
    bus: ModbusRTU,
    kind: str = "auto",
    *,
    timeout: float = GATEWAY_PROBE_TIMEOUT_S,
    position_unit_mm: float | None = None,
) -> Line:
    """Work out the line ``bus`` is on, attach it as ``bus.line`` and return it. READ-ONLY (FC03 probes only).

    ``kind``: ``"auto"`` (default) probes for an RCON gateway (D0h) and then a ROBONET gateway in gateway mode (3Fh);
    when neither answers the line is ``"direct"``. ``"rcon"`` / ``"robonet-gateway"`` probe that gateway only (an
    RCON probe then waits :data:`GATEWAY_FIRST_CONTACT_S`); ``"direct"`` and ``"robonet"`` (through mode, which
    cannot be probed) send nothing. Call it once the baud rate is settled.
    """
    if kind != "auto" and kind not in RULES:
        raise ValueError(f"line must be 'auto' or one of {', '.join(LINE_KINDS)}, got {kind!r}")
    lib = _n.load()
    handle = _bus_handle(bus)
    found = ctypes.c_int32(0)
    _n._check(lib, "iad_bus_detect_line", handle, _KIND_CODES.get(kind, 0), float(timeout),
              _n._num(position_unit_mm), ctypes.byref(found))  # fmt: skip
    out = ctypes.c_void_p()
    _n._check(lib, "iad_bus_line", handle, ctypes.byref(out))
    line = Line._of(lib, int(out.value or 0), bus)
    bus.line = line
    return line


def read_serial_code(bus: ModbusRTU, slave: int) -> str | None:
    """The unit's serial code, ``01 03 94 00 00 08 68 3C`` (8 words ASCII): identity where FC11 is not documented.

    DOC ME0413 p.5-10 to 5-23 (RCON driver units; SCON-CB-RC from V000D). ``None`` when the read is refused or
    the text is empty. READ-ONLY.
    """
    text = _n._read_text(_n.load(), "iad_bus_read_serial_code", _bus_handle(bus), int(slave))
    return text or None


def robonet_hint(bus: ModbusRTU, slave: int, *, directory_timed_out: bool) -> str | None:
    """An UNVERIFIED hint that a "direct" line may be a ROBONET in SIO through mode.

    Given when IAI command 0x45 was answered with exception 01 (``bus.unsupported_functions(slave)``) and the 0x0000
    directory read timed out on an axis that answers 0x9000.
    """
    text = _n._read_text(_n.load(), "iad_bus_robonet_hint", _bus_handle(bus), int(slave),
                         int(bool(directory_timed_out)))  # fmt: skip
    return text or None


def line_rules_table() -> list[dict[str, Any]]:
    """Every line kind's rules, as dicts (for documentation and the GUI)."""
    return [RULES[k].to_dict() for k in LINE_KINDS]
