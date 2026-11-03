"""Simulated RS-485 lines carrying several simulated units, for multi-axis tests without hardware.

:class:`SimLine` is a :class:`~iai_actuator_driver.modbus.Transport` that hosts several
:class:`~iai_actuator_driver.sim.SimulatedEC` instances by slave address, the way an RCON gateway or an SIO daisy
chain puts several axes on one serial line (slave = axis number + 1; UNVERIFIED on hardware)::

    from iai_actuator_driver.cell import Cell
    from iai_actuator_driver.sim_line import SimLine, sim_ports

    line = SimLine.with_slaves([1, 2, 3], time_scale=20)      # three axes on one line
    with Cell.open([line], slaves="scan") as cell:
        cell.move_all({0: 50.0, 1: 60.0, 2: 70.0}, speed_mm_s=50)

    ports = sim_ports(4)                                       # four lines, one axis each
    with Cell.open(ports) as cell: ...

Every request frame is delivered to every unit on the line: the addressed unit answers, the others stay
silent, a frame to the broadcast address 0 reaches all of them and none answers, and an address nobody
has stays silent (the master times out). Two units cannot share an address (that would garble every
reply on a real line). A baud-rate change of the line (``set_baudrate``) and a line BREAK reach every
unit, like the shared wire. :attr:`SimLine.requests` records ``(time, frame)`` for every request, so tests
can check how the frames of several axes interleave.

``latency_s`` delays every reply by that long and gives the line a read ``timeout``, so the bus applies its
response-timeout formula as on a real port (``ModbusRTU.response_timeout``).

Fault injection (:mod:`iai_actuator_driver.recovery`): :meth:`SimLine.unplug` / :meth:`SimLine.replug` model a pulled USB
adapter (the port vanishes from the PC, ``present`` False, and the open handle raises OSError on every call until
:meth:`SimLine.reopen`); :meth:`SimLine.power_cycle` (or ``line[slave].power_off()`` / ``power_on()``) models a
24 V power loss of the units. ``remove`` / ``add`` while unplugged model a unit swapped for another.

:class:`SimGatewayLine` models a line behind a gateway (DOC only, UNVERIFIED on hardware)::

    line = sim_rcon_line([1, 2, 3])                     # RCON gateway at D0h + three driver units
    line = sim_rcon_line([1], position_unit_mm=0.001)   # No.207 = 1: positions in 0.001 mm on the wire
    line = sim_robonet_line([1, 2])                     # ROBONET in SIO through mode (SW2 ON)
    line = sim_robonet_line([1, 2], gateway_mode=True)  # gateway mode (SW2 OFF): only 3Fh answers

* ``"rcon"``: a :class:`SimulatedRconGateway` answers at D0h (status block 0x3700 with the MANU flag, alarm 0x3708,
  link map NLNK, all-axes monitor 0x2900 + 15 x axis); the units answer FC 01/03/04/06/10/17 and exception 01 to
  anything else (FC05, FC11, 0x45, 0x48: ME0413 p.3-1); a broadcast reaches nobody (ME0413 p.6-1 #2); 0x9400
  serves a serial code; with ``position_unit_mm=0.001`` PNOW / PCMD / INP are 0.001 mm on the wire (ME0384 p.8-56).
* ``"robonet"``: FC 01-07/0F/10/11/17 answered, 0x45 / 0x48 exception 01 (ME0208 p.26); no broadcast; reads below
  0x0D00 (directory and class items) get no answer (RCM-era firmware, modelled); frames above 160 bytes get none.
* ``"robonet-gateway"``: only a :class:`SimulatedRoboNetGateway` at 3Fh answers (status words 0xF700).

A :class:`SimLine` here is a line of the core (``iad_sim_line``, C++ ``SimLine``) carrying native
:class:`~iai_actuator_driver.sim.SimulatedEC` units: every frame reaches every unit, the addressed one answers, a
broadcast reaches all and none answers, and unplug / replug / reopen / power cycle are the core's (the behaviour is
documented above). ``line[slave]`` is the very ``SimulatedEC`` object that was put on the
line. A native bus reaches the line straight in the core (``iad_sim_line_port``, offered by ``_iad_transport``);
``latency_s`` (replies held back, a read ``timeout``) is applied here, on this object's own ``write`` / ``read``, which
a bus then drives (on the native bus as a callback transport).

The gateway lines (:class:`SimGatewayLine`, ``sim_rcon_line``, ``sim_robonet_line``) are the core's gateway lines
(``iad_sim_gateway_line_open``, C++ ``SimGatewayLine``): the gateway's own answers, the routing and the conversions run
there, and each frame the gateway passes on reaches the addressed unit through its port.
:attr:`SimGatewayLine.gateway` is a view of the core's gateway (DOC only, UNVERIFIED on hardware).
"""

from __future__ import annotations

import ctypes
import json
import threading
import time
from collections.abc import Callable, Iterable, Iterator, Mapping
from typing import Any

from . import errors as _errors
from . import native as _n
from ._protocol import build_frame, check_crc  # noqa: F401 (re-exported)
from .sim import SimulatedEC
from .sim import _replaces_transport

__all__ = [
    "SimLine",
    "SimGatewayLine",
    "SimulatedRconGateway",
    "SimulatedRoboNetGateway",
    "sim_line",
    "sim_ports",
    "sim_rcon_line",
    "sim_robonet_line",
]


def _os_error(exc: BaseException) -> OSError:
    return OSError(str(exc))


class SimLine:
    """One simulated serial line with several :class:`SimulatedEC` units on it (a core line).

    Args:
        units: the simulated units; each answers to its own ``slave`` address (unique on the line).
        name: port name shown by :class:`~iai_actuator_driver.cell.Cell` (e.g. ``"SIM1"``; axis names ``"SIM1:2"``).
        latency_s: delay every reply by this long (real seconds) and expose a read ``timeout`` (0), so the bus
            waits its computed response timeout as on a serial port. ``None`` (default): replies at once.
    """

    def __init__(self, units: Iterable[SimulatedEC] = (), *, name: str = "SIM", latency_s: float | None = None) -> None:
        if latency_s is not None and latency_s < 0:
            raise ValueError("latency_s must be >= 0")
        self.name = name
        self.latency_s = latency_s
        self._units: dict[int, SimulatedEC] = {}
        self._lock = threading.Lock()
        self._last_write = 0.0
        self._read_timeout = 0.0
        self._line = self._open_line(name)
        self._port = self._line.transport()
        for unit in units:
            self.add(unit)

    def _open_line(self, name: str) -> _n.NativeSimLine:
        """The core line behind this object (a subclass opens another kind)."""
        return _n.NativeSimLine.open([], name)

    @classmethod
    def with_slaves(cls, slaves: Iterable[int], *, name: str = "SIM", **sim_kwargs: Any) -> SimLine:
        """A line with one new ``SimulatedEC(slave=n, **sim_kwargs)`` per address in ``slaves``."""
        return cls((SimulatedEC(slave=n, **sim_kwargs) for n in slaves), name=name)

    # ------------------------------------------------------------------ units

    def add(self, unit: SimulatedEC) -> SimulatedEC:
        """Put ``unit`` on the line (its ``slave`` must be free) and return it."""
        if not 1 <= unit.slave <= 247:
            raise ValueError(f"slave address must be 1..247, got {unit.slave}")
        with self._lock:
            if unit.slave in self._units:
                raise ValueError(f"slave {unit.slave} is already on line {self.name}")
            self._line.add(unit._unit)
            self._units[unit.slave] = unit
        return unit

    def remove(self, slave: int) -> SimulatedEC:
        """Take the unit at ``slave`` off the line (e.g. to model a cable pulled) and return it."""
        with self._lock:
            unit = self._units.pop(slave)
            self._line.remove(slave)
            return unit

    @property
    def units(self) -> dict[int, SimulatedEC]:
        """The units on the line by slave address (a copy)."""
        with self._lock:
            return dict(self._units)

    @property
    def slaves(self) -> list[int]:
        """The slave addresses on the line, in order."""
        return sorted(self.units)

    def __getitem__(self, slave: int) -> SimulatedEC:
        return self._units[slave]

    def __iter__(self) -> Iterator[SimulatedEC]:
        return iter(self.units[s] for s in self.slaves)

    def __len__(self) -> int:
        return len(self._units)

    @property
    def requests(self) -> list[tuple[float, bytes]]:
        """``(time, frame)`` of every request on the line, oldest first."""
        return [(t, frame) for t, _slave, frame in self._line.requests()]

    def request_slaves(self) -> list[int]:
        """The address byte of every request sent so far, in order (0 = broadcast)."""
        return [frame[0] for _, frame in self.requests if frame]

    # ------------------------------------------------------------------ state and fault injection

    @property
    def present(self) -> bool:
        """The port is listed by the PC (:meth:`unplug` / :meth:`replug`)."""
        return bool(self._line.state()["present"])

    @property
    def closed(self) -> bool:
        """The line was closed and not reopened."""
        return not self._line.state()["open"]

    @property
    def reopens(self) -> int:
        return int(self._line.state()["reopens"])

    def unplug(self) -> None:
        """Test hook: the USB adapter is pulled: the port disappears from the PC and the open handle raises OSError
        on every call until :meth:`reopen` after :meth:`replug`. The units keep their power, state and baud rate."""
        self._line.unplug()

    def replug(self) -> None:
        """Test hook: the adapter is plugged back in (the old handle stays dead until :meth:`reopen`)."""
        self._line.replug()

    def reopen(self, baudrate: int | None = None) -> None:
        """Open the line again after :meth:`close` or an :meth:`unplug` (OSError while unplugged). ``baudrate``: the
        PC side's rate afterwards (units that model one; ``None``: unchanged)."""
        try:
            self._port.reopen(baudrate)
        except _errors.TransportError as exc:
            raise _os_error(exc) from exc

    def power_cycle(self, slaves: Iterable[int] | None = None, *, lose_home: bool | None = None) -> None:
        """Test hook: a 24 V power cycle of the units at ``slaves`` (default: every unit on the line)."""
        self._line.power_cycle(None if slaves is None else list(slaves), lose_home=lose_home)

    # ------------------------------------------------------------------ Transport protocol

    @property
    def _iad_transport(self) -> Callable[[], tuple[int, bool]] | None:
        """How a native bus reaches this line: straight on the core's line (:meth:`_bus_port`); ``None`` (the bus then
        drives this object's methods) with ``latency_s`` or when its class or the object replaces one of them."""
        if self.latency_s is not None or _replaces_transport(self, SimLine) or not _n.load().has("iad_sim_line_port"):
            return None
        return self._bus_port

    def _bus_port(self) -> tuple[int, bool]:
        """(a new transport handle on the core's line, True: the caller releases it once its bus has it)."""
        out = ctypes.c_void_p()
        _n._check(_n.load(), "iad_sim_line_port", self._line._h(), ctypes.byref(out))
        return int(out.value or 0), True

    def write(self, data: bytes) -> None:
        try:
            self._port.write(bytes(data))
        except _errors.TransportError as exc:
            raise _os_error(exc) from exc
        self._last_write = time.perf_counter()

    def _reply_due(self) -> bool:
        return self.latency_s is None or time.perf_counter() >= self._last_write + self.latency_s

    def read(self, n: int) -> bytes:
        if not self._reply_due():
            return b""
        try:
            return self._port.read(n)
        except _errors.TransportError as exc:
            raise _os_error(exc) from exc

    @property
    def timeout(self) -> float:
        """Read timeout, only on a line with ``latency_s`` (reads never block: the bus waits its own deadline)."""
        if self.latency_s is None:
            raise AttributeError("timeout")
        return self._read_timeout

    @timeout.setter
    def timeout(self, value: float) -> None:
        self._read_timeout = float(value)

    def reset_input_buffer(self) -> None:
        try:
            self._port.reset_input_buffer()
        except _errors.TransportError as exc:
            raise _os_error(exc) from exc

    def close(self) -> None:
        self._line.transport().close()  # closes the line (its units too); this object's handle stays for reopen()

    @property
    def baudrate(self) -> int | None:
        """The line's (PC side) baud rate: the units' own model of it, ``None`` when they do not model one."""
        rate = self._line.state()["baud_rate"]
        return None if rate is None else int(rate)

    def set_baudrate(self, baudrate: int) -> None:
        """Change the PC side's rate on the shared wire (every unit hears the line at the new rate)."""
        if not self._units or self.baudrate is None:
            raise TypeError(f"simulated line {self.name} has no baud rate (pass baudrate= to the units)")
        try:
            self._port.set_baudrate(int(baudrate))
        except _errors.TransportError as exc:
            raise _os_error(exc) from exc

    def send_break(self, duration: float) -> None:
        """A line BREAK reaches every unit on the wire."""
        self._port.send_break(float(duration))

    def __repr__(self) -> str:
        return f"SimLine({self.name!r}, slaves={self.slaves})"


# ================================================================== gateway lines (DOC only; UNVERIFIED on hardware)

_RCON_FUNCTIONS = frozenset({0x01, 0x03, 0x04, 0x06, 0x10, 0x17})  # ME0413 p.3-1
_ROBONET_FUNCTIONS = frozenset({0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0F, 0x10, 0x11, 0x17})  # ME0208 p.26


def _frames(hex_frames: Iterable[str]) -> list[bytes]:
    return [bytes.fromhex(f) for f in hex_frames]


def _open_gateway_line(kind: str, name: str, options: dict[str, Any]) -> _n.NativeSimLine:
    """A core gateway line without units yet (iad_sim_gateway_line_open)."""
    lib = _n.load()
    out = ctypes.c_void_p()
    _n._check(lib, "iad_sim_gateway_line_open", _n._utf8(kind), _n._utf8(json.dumps({"name": name, **options})), None,
              0, ctypes.byref(out))  # fmt: skip
    return _n.NativeSimLine(lib, int(out.value or 0), name)


class _GatewayView:
    """The gateway station of a core gateway line: its state is read and changed there (iad_sim_line_control_json)."""

    _native: _n.NativeSimLine
    line: SimGatewayLine | None

    @classmethod
    def _on(cls, line: SimGatewayLine) -> Any:
        view = cls.__new__(cls)
        view._native = line._line
        view.line = line
        return view

    def _state(self) -> dict[str, Any]:
        return dict(self._native.control("gateway")["gateway"])

    def _set(self, **values: Any) -> None:
        self._native.control("set_gateway", **values)

    @property
    def slave(self) -> int:
        return int(self._state()["slave"])

    @slave.setter
    def slave(self, value: int) -> None:
        self._set(slave=int(value))

    @property
    def requests(self) -> list[bytes]:
        """Every frame addressed to the gateway, oldest first."""
        return _frames(self._state()["requests"])

    def handle(self, frame: bytes) -> bytes | None:
        """The gateway's answer to one request frame (``None``: no answer)."""
        reply = self._native.control("gateway_handle", frame=bytes(frame).hex(" "))["reply"]
        return None if reply is None else bytes.fromhex(reply)


class SimulatedRconGateway(_GatewayView):
    """A simulated RCON gateway (slave D0h), run by the core: answers FC03 / FC04 reads only (the status block
    0x3700, the all-axes monitor 0x2900 and TIMC 0x3300). On a :class:`SimGatewayLine` it is
    :attr:`SimGatewayLine.gateway`; made on its own it has no line and no axes."""

    def __init__(
        self,
        slave: int = 0xD0,
        *,
        manu: bool = True,
        alarm_code: int = 0,
        emergency_stop: bool = False,
        linked_only: Iterable[int] = (),
    ) -> None:
        options = {"manu": bool(manu), "gateway_alarm": int(alarm_code), "emergency_stop": bool(emergency_stop),
                   "linked_only": sorted({int(s) for s in linked_only})}  # fmt: skip
        self._native = _open_gateway_line("rcon", "GW", options)
        self.line = None
        if slave != 0xD0:
            self.slave = slave

    @property
    def manu(self) -> bool:
        """The AUTO / MANU switch (GWSR b15 GWMD, GSR2 b2 ATST)."""
        return bool(self._state()["manu"])

    @manu.setter
    def manu(self, value: bool) -> None:
        self._set(manu=bool(value))

    @property
    def alarm_code(self) -> int:
        return int(self._state()["alarm_code"])

    @alarm_code.setter
    def alarm_code(self, value: int) -> None:
        self._set(alarm_code=int(value))

    @property
    def emergency_stop(self) -> bool:
        return bool(self._state()["emergency_stop"])

    @emergency_stop.setter
    def emergency_stop(self, value: bool) -> None:
        self._set(emergency_stop=bool(value))

    @property
    def linked_only(self) -> set[int]:
        """Slave addresses linked at the gateway that never answer (ECs on RCON-EC)."""
        return {int(s) for s in self._state()["linked_only"]}

    @linked_only.setter
    def linked_only(self, slaves: Iterable[int]) -> None:
        self._set(linked_only=sorted({int(s) for s in slaves}))

    def linked_slaves(self) -> list[int]:
        """The units on the line plus :attr:`linked_only`, ascending."""
        return [int(s) for s in self._state()["linked_slaves"]]

    def status_block(self) -> list[int]:
        """The 11 words of 0x3700."""
        return [int(w) for w in self._state()["status_block"]]


class SimulatedRoboNetGateway(_GatewayView):
    """A simulated ROBONET gateway in gateway mode (slave 3Fh), run by the core: FC03 of the status area 0xF700-0xF707.
    On a :class:`SimGatewayLine` it is :attr:`SimGatewayLine.gateway`; made on its own it has no line and no axes."""

    def __init__(self, slave: int = 0x3F, *, running: bool = True) -> None:
        self._native = _open_gateway_line("robonet-gateway", "GW", {})
        self.line = None
        if slave != 0x3F:
            self.slave = slave
        if not running:
            self.running = False

    @property
    def running(self) -> bool:
        return bool(self._state()["running"])

    @running.setter
    def running(self, value: bool) -> None:
        self._set(running=bool(value))


class SimGatewayLine(SimLine):
    """A simulated line behind a gateway: ``kind`` ``"rcon"``, ``"robonet"`` (through mode) or ``"robonet-gateway"``
    (this module's documentation says what each kind answers). The gateway runs in
    the core; each frame it passes on reaches the addressed unit through its port.

    :attr:`dropped_broadcasts` lists the broadcast frames nobody received; :attr:`rejected` the frames a unit answered
    with exception 01 because the gateway does not carry the function.
    """

    KINDS = ("rcon", "robonet", "robonet-gateway")

    def __init__(
        self,
        units: Iterable[SimulatedEC] = (),
        *,
        kind: str = "rcon",
        name: str = "GW",
        manu: bool = True,
        gateway_alarm: int = 0,
        emergency_stop: bool = False,
        linked_only: Iterable[int] = (),
        position_unit_mm: float | Mapping[int, float] = 0.01,
        serials: Mapping[int, str] | None = None,
        latency_s: float | None = None,
        silent_no_access: bool = False,
    ) -> None:
        if kind not in self.KINDS:
            raise ValueError(f"kind must be one of {self.KINDS}, got {kind!r}")
        for value in ([position_unit_mm] if not isinstance(position_unit_mm, Mapping) else position_unit_mm.values()):
            if value not in (0.01, 0.001):
                raise ValueError(f"position_unit_mm must be 0.01 or 0.001, got {value!r}")
        self.kind = kind
        self._unit_mm = position_unit_mm
        self._serials = dict(serials or {})
        self._options: dict[str, Any] = {
            "manu": bool(manu), "gateway_alarm": int(gateway_alarm), "emergency_stop": bool(emergency_stop),
            "linked_only": sorted({int(s) for s in linked_only}),
            "position_unit_mm": ({str(int(s)): float(u) for s, u in position_unit_mm.items()}
                                 if isinstance(position_unit_mm, Mapping) else float(position_unit_mm)),
            "serials": {str(int(s)): str(code) for s, code in self._serials.items()},
            "silent_no_access": bool(silent_no_access),
        }  # fmt: skip
        super().__init__(units, name=name, latency_s=latency_s)
        self.gateway: SimulatedRconGateway | SimulatedRoboNetGateway | None
        if kind == "rcon":
            self.gateway = SimulatedRconGateway._on(self)
        elif kind == "robonet-gateway":
            self.gateway = SimulatedRoboNetGateway._on(self)
        else:
            self.gateway = None
        self.function_codes = _RCON_FUNCTIONS if kind == "rcon" else _ROBONET_FUNCTIONS

    def _open_line(self, name: str) -> _n.NativeSimLine:
        return _open_gateway_line(self.kind, name, self._options)

    def __repr__(self) -> str:
        return f"SimGatewayLine({self.name!r}, kind={self.kind!r}, slaves={self.slaves})"

    def position_unit(self, slave: int) -> float:
        unit = self._unit_mm
        return float(unit.get(slave, 0.01)) if isinstance(unit, Mapping) else float(unit)

    def position_factor(self, slave: int) -> int:
        """Wire counts per simulated count: 10 for a 0.001 mm unit, else 1 (RCON only)."""
        return 10 if self.kind == "rcon" and self.position_unit(slave) == 0.001 else 1

    def serial_code(self, slave: int) -> str:
        return self._serials.get(slave, f"SIMRCON-{slave - 1:02d}")

    def _gateway_state(self) -> dict[str, Any]:
        return dict(self._line.control("gateway"))

    @property
    def silent_no_access(self) -> bool:
        """The axes never answer an FC03 read of the RCON "system, no access" areas (the directory times out)."""
        return bool(self._gateway_state()["silent_no_access"])

    @silent_no_access.setter
    def silent_no_access(self, value: bool) -> None:
        self._line.control("set_gateway", silent_no_access=bool(value))

    @property
    def dropped_broadcasts(self) -> list[bytes]:
        """The broadcast frames nobody received, oldest first."""
        return _frames(self._gateway_state()["dropped_broadcasts"])

    @property
    def rejected(self) -> list[bytes]:
        """The frames a unit answered with exception 01 (a function the gateway does not carry), oldest first."""
        return _frames(self._gateway_state()["rejected"])


def sim_rcon_line(
    slaves: Iterable[int] = (1,),
    *,
    name: str = "RCON",
    manu: bool = True,
    gateway_alarm: int = 0,
    emergency_stop: bool = False,
    linked_only: Iterable[int] = (),
    position_unit_mm: float | Mapping[int, float] = 0.01,
    serials: Mapping[int, str] | None = None,
    latency_s: float | None = None,
    profile: str = "pcon",
    silent_no_access: bool = False,
    **sim_kwargs: Any,
) -> SimGatewayLine:
    """An RCON line: a simulated gateway at D0h and one ``SimulatedEC(slave=n, profile=profile)`` per address."""
    units = [SimulatedEC(slave=n, profile=profile, **sim_kwargs) for n in slaves]
    return SimGatewayLine(units, kind="rcon", name=name, manu=manu, gateway_alarm=gateway_alarm,
                          emergency_stop=emergency_stop, linked_only=linked_only, position_unit_mm=position_unit_mm,
                          serials=serials, latency_s=latency_s, silent_no_access=silent_no_access)  # fmt: skip


def sim_robonet_line(
    slaves: Iterable[int] = (1,),
    *,
    name: str = "ROBONET",
    gateway_mode: bool = False,
    latency_s: float | None = None,
    profile: str = "pcon",
    **sim_kwargs: Any,
) -> SimGatewayLine:
    """A ROBONET line in SIO through mode (default) or, with ``gateway_mode``, in gateway mode (only 3Fh answers)."""
    units = [SimulatedEC(slave=n, profile=profile, **sim_kwargs) for n in slaves]
    return SimGatewayLine(units, kind="robonet-gateway" if gateway_mode else "robonet", name=name,
                          latency_s=latency_s)  # fmt: skip


def sim_line(slaves: Iterable[int] = (1,), *, name: str = "SIM", **sim_kwargs: Any) -> SimLine:
    """A :class:`SimLine` named ``name`` with one ``SimulatedEC(slave=n, **sim_kwargs)`` per address."""
    return SimLine.with_slaves(slaves, name=name, **sim_kwargs)


def sim_ports(
    n: int, slaves: Iterable[int] = (1,), *, prefix: str = "SIM", **sim_kwargs: Any
) -> dict[str, SimLine]:
    """``n`` simulated ports named ``SIM1`` .. ``SIM<n>``, each a line with one unit per address in ``slaves``.

    The default (one unit at slave 1 per port) is the normal setup: one axis per COM port. Returns
    ``{name: line}``, ready for ``Cell.open(ports)``.
    """
    if not 1 <= n <= 64:
        raise ValueError("n must be 1..64")
    addresses = list(slaves)
    return {f"{prefix}{i}": SimLine.with_slaves(addresses, name=f"{prefix}{i}", **sim_kwargs) for i in range(1, n + 1)}
