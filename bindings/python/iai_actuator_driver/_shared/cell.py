"""The Python part of ``iai_actuator_driver.cell``: the adapter's USB IDs and the scan defaults, the scan
and port records (:class:`ScanHit`, :class:`PortBaudResult`), the axis names and small helpers.

``iai_actuator_driver.cell`` (where the rules and the evidence are) imports these names from here.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any

from .. import registers as R
from ..values import BaudChangeResult, Status
from . import publish
from .gateway import LINE_DIRECT, LINE_RCON

if TYPE_CHECKING:
    from ..cell import AxisKey, Cell
    from ..gateway import RconAxisMonitor

IAI_USB_VID = 0x10C4
"""USB vendor ID of the IAI USB-RS485 adapter (Silicon Labs CP210x). VERIFIED."""
IAI_USB_PID = 0x81D7
"""USB product ID of the IAI adapter ("IAI USB to UART Bridge Controller", maker "IAI Corporation"). VERIFIED."""
DEFAULT_SLAVES: tuple[int, ...] = tuple(range(1, 17))
"""Addresses scanned by default: axis numbers 0..15 (slave = axis number + 1)."""
MAX_PORTS = 16
"""The port limit of :meth:`Cell.open` up to 1.0, kept for compatibility: since 1.1 a cell takes any number of ports
(one thread each) and every axis on them (``max_axes`` caps it)."""
MONITOR_STATUS_AGE_S = 0.1  # status_all takes a running monitor's sample when it is younger than this


def axis_name(port: str, slave: int) -> str:
    """The name of the axis at ``slave`` on ``port`` in a :class:`Cell`, e.g. ``"COM3:1"``."""
    return f"{port}:{slave}"


def _port_sort_key(device: str) -> tuple[str, int, str]:
    head = device.rstrip("0123456789")
    tail = device[len(head):]
    return (head, int(tail) if tail else -1, device)


@dataclass(frozen=True)
class ScanHit:
    """One address that answered a scan (FC03 probe), with what Report Slave ID (FC11) says about it."""

    port: str
    slave: int
    slave_id_raw: bytes | None = None
    system_code: int | None = None
    unit_code: int | None = None
    firmware: str | None = None
    error: str | None = None  # FC11 not answered / rejected (the address still answered the probe)
    line: str = LINE_DIRECT  # the line kind of the port (gateway.LINE_KINDS)
    serial_code: str | None = None  # 0x9400 serial code: the identity on an RCON line, where FC11 is not documented

    @property
    def name(self) -> str:
        """``"<port>:<slave>"``, the axis name a :class:`Cell` gives it."""
        return axis_name(self.port, self.slave)

    @property
    def axis_number(self) -> int:
        """Axis number = slave address - 1."""
        return self.slave - 1

    @property
    def system_name(self) -> str | None:
        if self.system_code is None:
            return None
        return R.SYSTEM_CODES.get(self.system_code, f"system 0x{self.system_code:02X}")

    @property
    def product_family(self) -> str | None:
        """Product family of the FC11 system code (``"EC"``, ``"PCON"``, ``"ACON/DCON"``, ``"SCON"``, else
        ``"IAI controller"``); ``"RCON"`` for a driver unit behind an RCON gateway; ``None`` when FC11 was not
        answered. Every family is driven the same way."""
        if self.system_code is None:
            return "RCON" if self.line == LINE_RCON else None
        return R.product_family(self.system_code)

    @property
    def family(self) -> str | None:
        """EC family name from the unit code (``None`` for a CON-family unit or an EC unit code the driver has no
        name for; it is still driven)."""
        return R.ec_family(self.system_code, self.unit_code)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "port": self.port,
            "slave": self.slave,
            "axis_number": self.axis_number,
            "slave_id_raw": self.slave_id_raw.hex(" ") if self.slave_id_raw is not None else None,
            "system_code": self.system_code,
            "system_name": self.system_name,
            "unit_code": self.unit_code,
            "firmware": self.firmware,
            "product_family": self.product_family,
            "family": self.family,
            "error": self.error,
            "line": self.line,
            "serial_code": self.serial_code,
        }


@dataclass(frozen=True)
class PortBaudResult:
    """Outcome of :meth:`Port.set_baudrate`: the switch (verified on the first axis) and who answers after it."""

    port: str
    rate: int
    change: BaudChangeResult
    answered: dict[int, bool] = field(default_factory=dict)

    @property
    def all_answered(self) -> bool:
        return all(self.answered.values())

    def to_dict(self) -> dict[str, Any]:
        return {"port": self.port, "rate": self.rate, "change": self.change.to_dict(), "answered": dict(self.answered)}


def _status_from_monitor(mon: RconAxisMonitor, unit_mm: float | None) -> Status:
    """A :class:`Status` from one RCON all-axes monitor record, positions scaled by the line's position unit."""
    st = Status.from_registers(mon.status_words())
    unit = R.POSITION_SCALE if unit_mm is None else unit_mm
    raw = dict(st.raw)
    raw.update({"SIPM": mon.sipm, "OLLV": mon.overload_pct, "DEVI": mon.deviation_pulses & 0xFFFFFFFF,
                "GATEWAY_MONITOR": 1, "POSITION_UNIT_UM": 0 if unit_mm is None else round(unit_mm * 1000)})  # fmt: skip
    return replace(st, position_mm=round(mon.position_counts * unit, 3), raw=raw)


def _pick(value: float | Mapping[AxisKey, float], key: AxisKey, name: str, cell: Cell) -> float:
    if not isinstance(value, Mapping):
        return float(value)
    for k, v in value.items():
        if k == key or cell._name(k) == name:
            return float(v)
    raise KeyError(f"no value for axis {name}")


def _ranges(values: Iterable[int]) -> str:
    items = sorted(set(values))
    if not items:
        return "-"
    out: list[str] = []
    start = prev = items[0]
    for v in items[1:] + [None]:  # type: ignore[list-item]
        if v is not None and v == prev + 1:
            prev = v
            continue
        out.append(str(start) if start == prev else f"{start}-{prev}")
        if v is not None:
            start = prev = v
    return ",".join(out)


publish(globals(), "cell")
