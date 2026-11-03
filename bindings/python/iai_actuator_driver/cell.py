"""Several axes at once: port discovery, address scan, and a :class:`Cell` of axes driven together.

The normal setup is **one axis per COM port** (one IAI USB adapter per actuator, all driven in parallel;
a cell takes every axis it finds, ``max_axes`` caps it). Several axes can also share one port by axis
number (an RCON gateway or an SIO daisy chain, up to 16 per port, slave address = axis number + 1;
UNVERIFIED on hardware), and the two can be mixed::

    from iai_actuator_driver.cell import Cell, list_ports

    print(list_ports())                         # IAI USB adapters (CP210x 10C4:81D7) first
    with Cell.open() as cell:                   # every IAI adapter, every answering address 1..16
        print(cell.names)                       # ("COM3:1", "COM4:1", ...)
        cell.servo_all(True)
        cell.home_all()
        cell.move_all({"COM3:1": 50.0, "COM4:1": 20.0}, speed_mm_s=50)
        print(cell.status_all())
        cell.servo_all(False)

Rules:

* A port is opened **once** (:class:`Port`: one :class:`~iai_actuator_driver.modbus.ModbusRTU` with its own lock) and
  its baud rate is detected once, in place. A baud change on a port is a broadcast: **every** axis on that
  line switches (:meth:`Port.set_baudrate`).
* Ports work in parallel threads; the axes of one port take turns on its lock. Waiting loops
  (``wait_idle``, homing, servo) sleep between polls without holding the lock, so axes sharing a line
  interleave their frames and move at the same time.
* Each axis is an ordinary :class:`~iai_actuator_driver.ec.Elecylinder` (built on the port's bus and its slave
  address), with its own state, ranges and capabilities read from the unit itself: every unit is driven the
  same way, whatever its model.
* :meth:`Cell.stop_all` sends STOP to every axis first (ports in parallel, axes of a port back to back),
  then waits for each to stop. A failure on one axis during :meth:`Cell.move_all` / :meth:`Cell.home_all` /
  :meth:`Cell.wait_all` stops the others too (``stop_others_on_error``) and leaving a ``with Cell`` block on
  an exception after motion stops every axis.
* Operations on many axes collect per-axis results; any failure raises :class:`CellError` (with every
  result and error) after all axes were handled, or, with ``return_exceptions=True``, is returned in place
  of that axis's result.
* Every port knows its line (:attr:`Port.line`, :mod:`iai_actuator_driver.gateway`): at the first scan it probes once for an
  RCON gateway (D0h) and a ROBONET gateway in gateway mode (3Fh), read-only, ~0.1 s on a line without one. On a
  gateway line nothing is broadcast (no baud change, no broadcast STOP), 0x45 / 0x48 are not sent, and on RCON
  FC05 is not sent, the gateway must be in MANU and the position unit (No.207) must be stated before mm commands.
  A ROBONET gateway in gateway mode gives no axis: set its SW2 ON (through mode) and use ``line="robonet"``.
* Broadcast STOP / pause / servo off (:meth:`Port.broadcast_stop`, ``Cell.stop_all(broadcast=True)``) are for SIO
  daisy chains only.
* Power / link loss (:mod:`iai_actuator_driver.recovery`): a link fault on one axis during any group command (its port vanished,
  it stopped answering) or seen by :meth:`Cell.check_links` / :meth:`Cell.monitor_links` (also a restarted unit) calls
  ``cell.on_fault(name, kind, error)`` and, unless ``stop_on_fault`` is False, stops every other axis once;
  ``cell.faults`` lists the faulted axes and ``cell.events`` the ``link_lost`` / ``unit_restarted`` / ``unit_replaced``
  / ``recovered`` events. :meth:`Cell.recover_all` brings the axes back (ports in parallel, the axes of a port one
  after another) and never moves anything.
* Status monitors (:mod:`iai_actuator_driver.monitor`): :meth:`Cell.start_monitors` starts each axis's background status thread
  with its link events routed to the fault handling above; while an axis's monitor runs, :meth:`Cell.status_all` takes
  its latest sample (no bus traffic) and :meth:`Cell.recover_all` holds it during the recovery and restarts it after.

Evidence: one axis per port is VERIFIED (each tested unit alone on its own USB adapter). Several axes on
one line, the broadcast baud change reaching several units, broadcast STOP / pause, and the CP210x identification
of other adapters are UNVERIFIED; the IAI adapter's USB ID 10C4:81D7 ("IAI USB to UART Bridge Controller") is
VERIFIED. RCON and ROBONET lines are DOC only (ME0413, ME0208), UNVERIFIED on hardware.

On the core: :class:`Port` is a C++ ``Port`` over the core bus of its :class:`~iai_actuator_driver.modbus.ModbusRTU`
(iad_bus_set_port), :class:`Cell` a C++ ``Cell`` over those ports (iad_cell_open_buses) or over axes that exist
already (iad_cell_open_axes). The core decides and waits: the baud rate found once, the scan and the identities, the
listed slaves that must answer, ``max_axes``, the line-wide baud change, every group command (ports in parallel, the
axes of a port one after another, the checks before any move starts, the STOPs first, the stop of the others on a
failure, the broadcasts of a daisy chain, the fault handling of a link loss). This module converts arguments and
results:

* the line of a port is worked out by the active :mod:`iai_actuator_driver.gateway` (its ``detect_line`` attaches it
  to the bus), which the core port calls where it would probe; the line's own objects (``port.line``,
  ``port.gateway``, the RCON all-axes monitor of :meth:`Cell.status_all`) are that module's;
* the axes are ordinary :class:`~iai_actuator_driver.ec.Elecylinder` objects: those of :meth:`Cell.open` are the
  cell's own core axes (iad_cell_axis_ref), with the settings the axis keyword arguments give;
* per-axis results are the shared :class:`~iai_actuator_driver.values.Status` and records, per-axis errors the
  package's exceptions (with the core's notes), a failure :class:`~iai_actuator_driver.errors.CellError`;
* a group command that can wait long runs, on the main thread, on a helper thread joined in short slices: Ctrl+C then
  latches the core's cancel on those axes (a decel stop) and raises ``KeyboardInterrupt``.

The link-loss calls of a cell (:meth:`Cell.track_all`, :meth:`~Cell.check_links`, :meth:`~Cell.monitor_links`,
:meth:`~Cell.recover_all`) are the core cell's: its fault handling (``on_fault``, the stop of the other axes), its
link monitor and its recovery (ports in parallel); their reports, checks and sessions are those of
:mod:`iai_actuator_driver.recovery`, and their events have reached ``on_fault`` / ``on_event`` / :attr:`Cell.events`
when the call returns.
"""

from __future__ import annotations

import collections
import ctypes
import itertools
import json
import logging
import threading
import weakref
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from pathlib import Path
from typing import Any

from . import native as _n
from . import recovery as _recovery
from ._shared.cell import (  # the shared names (records, helpers, constants)
    DEFAULT_SLAVES,
    IAI_USB_PID,
    IAI_USB_VID,
    MAX_PORTS,
    MONITOR_STATUS_AGE_S,
    PortBaudResult,
    ScanHit,
    _pick,
    _port_sort_key,
    _ranges,
    _status_from_monitor,
    axis_name,
)
from .ec import Elecylinder
from .errors import (  # noqa: F401 (re-exported)
    NO_ANSWER_HINT,
    CellError,
    EcDriverError,
    EcNotReadyError,
    ModbusError,
    ModbusException,
    ModbusTimeout,
)
from .gateway import (  # noqa: F401 (re-exported)
    LINE_DIRECT,
    LINE_KINDS,
    LINE_RCON,
    LINE_ROBONET,
    LINE_ROBONET_GATEWAY,
    OP_BAUD_CHANGE,
    OP_BROADCAST,
    RCON_GATEWAY_SLAVE,
    RCON_GW_STATUS,
    ROBONET_GATEWAY_SLAVE,
    ROBONET_GW_STATUS,
    Line,
    LineRuleError,
    RconAxisMonitor,
    RconGateway,
    RconGatewayStatus,
    RoboNetGateway,
    RoboNetGatewayStatus,
    UnsupportedLineError,
    detect_line,
    read_serial_code,
)
from .modbus import (  # noqa: F401 (re-exported)
    BAUD_DETECT_ORDER,
    DEFAULT_BAUDRATE,
    RECOMMENDED_RETRIES,
    SCAN_TIMEOUT_S,
    FrameLogger,
    ModbusRTU,
    Transport,
)
from .values import BaudChangeResult, DeviceInfo, Status  # noqa: F401 (re-exported)
from .ec import _BAUD_METHODS, _METHOD_DEFAULT, _METHODS, _choice, _device_info, _error, _record

__all__ = [
    "IAI_USB_VID",
    "IAI_USB_PID",
    "DEFAULT_SLAVES",
    "MAX_PORTS",
    "list_ports",
    "scan",
    "ScanHit",
    "Port",
    "PortBaudResult",
    "Cell",
    "CellError",
    "axis_name",
]

log = logging.getLogger("iai_actuator_driver.cell")  # the public module name, on every backend

_LINE_CODES = {"auto": 0, LINE_DIRECT: 1, LINE_RCON: 2, LINE_ROBONET: 3, LINE_ROBONET_GATEWAY: 4}  # IAD_LINE_*
_BAUD_OPTIONS = ("method", "settle_s", "allow_unverified_code")  # Port.set_baudrate's options (iad_baud_options)


def _csv(values: Iterable[int]) -> bytes:
    return ",".join(str(int(v)) for v in values).encode("ascii")


def _json_call(lib: Any, name: str, *args: Any) -> Any:
    """A JSON output of the core (64 KiB first, then the size it asked for), its error raised as the package's."""
    size = 64 * 1024
    for _ in range(3):
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        code, details = _n._invoke(lib, name, *args, buffer, size, ctypes.byref(required))
        if code == _n.IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
            size = required.value + 4096
            continue
        if code != _n.IAD_OK:
            raise _error(code, details or {})
        return json.loads(buffer.value.decode("utf-8", errors="replace"))
    raise RuntimeError(f"{name}: the output kept growing; try again")


def _json_or_none(text: str | None) -> dict[str, Any]:
    """A call's kept result (``native._invoke_kept``) as a dict; ``{}`` for none."""
    if not text:
        return {}
    try:
        data = json.loads(text)
    except ValueError:
        return {}
    return data if isinstance(data, dict) else {}


def _waiting(lib: Any, name: str, *args: Any) -> None:
    """A port call that may wait long (the baud detection, a scan): on the main thread it runs on a helper thread so
    that Ctrl+C reaches Python once it has ended on its own timeouts (nothing can end it early)."""
    code, details = _n._invoke_interruptible(lib, name, args, None)
    if code != _n.IAD_OK:
        raise _error(code, details or {})


# ====================================================================== discovery


_PORT_LIST: list[dict[str, Any]] | None = None  # set by testing.fake_ports(): replaces the OS enumeration
_PORT_DEFAULTS: dict[str, Any] = {"vid": None, "pid": None, "description": "", "manufacturer": ""}


def list_ports() -> list[dict[str, Any]]:
    """The serial ports of this PC: ``[{port, description, vid, pid, is_iai}]``, IAI adapters first (the core's
    enumeration; no port is opened)."""
    if _PORT_LIST is None:
        found = _n._read_json(_n.load(), "iad_list_ports_json")
        return [{"port": p["port"], "description": str(p.get("description") or ""), "vid": p.get("vid"),
                 "pid": p.get("pid"), "is_iai": bool(p.get("is_iai"))} for p in found]  # fmt: skip
    entries: list[dict[str, Any]] = []
    for given in _PORT_LIST:  # the fake enumeration of the tests, judged and sorted as the core does
        p = {**_PORT_DEFAULTS, **given}
        maker = str(p.get("manufacturer") or "")
        is_iai = (p["vid"] == IAI_USB_VID and p["pid"] == IAI_USB_PID) or maker.upper().startswith("IAI")
        entries.append({"port": p["port"], "description": str(p.get("description") or ""), "vid": p["vid"],
                        "pid": p["pid"], "is_iai": bool(is_iai)})  # fmt: skip
    entries.sort(key=lambda e: (not e["is_iai"], _port_sort_key(e["port"])))
    return entries


def _iai_port_names() -> list[str]:
    return [p["port"] for p in list_ports() if p["is_iai"]]


# ====================================================================== scan


def _scan_hit(data: Mapping[str, Any]) -> ScanHit:
    raw = data.get("slave_id_raw")
    return ScanHit(
        str(data["port"]), int(data["slave"]),
        slave_id_raw=bytes.fromhex(raw.replace(" ", "")) if isinstance(raw, str) else None,
        system_code=data.get("system_code"), unit_code=data.get("unit_code"), firmware=data.get("firmware"),
        error=data.get("error"), line=str(data.get("line") or LINE_DIRECT), serial_code=data.get("serial_code"),
    )  # fmt: skip


# ---- the line of a port: the core port asks its owner (iad_port_line_fn), on the thread doing the work

_LINE_OWNERS: dict[int, Any] = {}  # key -> weak reference to the Port whose line the core asks for
_LINE_KEYS = itertools.count(1)


def _port_line(user: Any) -> int:
    try:
        port = _LINE_OWNERS.get(int(user or 0), lambda: None)()
        if port is None:
            return _n.IAD_ERR_DRIVER
        port._work_out_line()
        return _n.IAD_OK
    except BaseException as exc:  # never into C: the core logs it and asks again next time
        _n._interrupted(exc)
        if not isinstance(exc, KeyboardInterrupt):
            log.warning("line detection failed: %s: %s", type(exc).__name__, exc)
        return _n.IAD_ERR_DRIVER


_PORT_LINE_CB = _n.PORT_LINE(_port_line)  # referenced for the life of the process (rule B4)


class Port:
    """One serial port, opened once, with its own bus lock; the axes on it share that bus.

    Args:
        port: a port name (``"COM3"``) or a :class:`~iai_actuator_driver.modbus.Transport` (e.g. a
            :class:`~iai_actuator_driver.sim_line.SimLine`).
        name: display name (default: the port name, or the transport's ``name``, or ``"PORT"``).
        baudrate: fixed rate; ``None`` (default) detects it once, in place (:meth:`ensure_baudrate`).
        timeout / retries / frame_logger: as for :class:`~iai_actuator_driver.modbus.ModbusRTU` (retries default 3, as
            ME0162-12A p.4-2 asks).
        line: ``"auto"`` (default: probe once for an RCON gateway at D0h and a ROBONET gateway at 3Fh, ~0.1 s on a
            line without one), ``"direct"`` (no probe), ``"rcon"``, ``"robonet"`` (SIO through mode, which cannot be
            probed) or ``"robonet-gateway"``. See :mod:`iai_actuator_driver.gateway`.
        position_unit_mm: the position unit of every axis on an RCON line (No.207: 0.01 or 0.001 mm). Without it
            mm-based motion on an RCON line is refused (``bus.line.require_motion``); see :meth:`set_position_unit`.

    The line (``port.line``, also ``port.bus.line`` for the axes) is detected at the first :meth:`scan`,
    :meth:`axis`, :meth:`set_baudrate` or broadcast, after the baud rate.

    On the core: the core's ``Port``.
    """

    def __init__(
        self,
        port: str | Transport,
        *,
        name: str | None = None,
        baudrate: int | None = None,
        timeout: float = 0.2,
        retries: int = RECOMMENDED_RETRIES,
        frame_logger: FrameLogger | None = None,
        line: str = "auto",
        position_unit_mm: float | None = None,
    ) -> None:
        if line != "auto" and line not in LINE_KINDS:
            raise ValueError(f"line must be 'auto' or one of {', '.join(LINE_KINDS)}, got {line!r}")
        if name is None:
            name = port if isinstance(port, str) else str(getattr(port, "name", "") or "PORT")
        self.name = name
        self.auto_baud = baudrate is None
        self.bus = ModbusRTU(port, DEFAULT_BAUDRATE if baudrate is None else baudrate, timeout, retries,
                             frame_logger=frame_logger)  # fmt: skip
        self._lib = _n.load()
        self._axes: dict[int, Elecylinder] = {}
        self._lock = threading.Lock()
        self._line_kind = line
        self._position_unit_mm = position_unit_mm
        self._line_key = next(_LINE_KEYS)
        try:
            if line in (LINE_DIRECT, LINE_ROBONET):  # nothing to probe
                self.bus.line = Line(line, position_unit_mm=position_unit_mm)
            elif position_unit_mm is not None:
                Line(LINE_RCON, position_unit_mm=position_unit_mm)  # validated now; set at detection (direct: 0.01)
            _LINE_OWNERS[self._line_key] = weakref.ref(self)
            weakref.finalize(self, _LINE_OWNERS.pop, self._line_key, None)
            self._port_handle = 0
            self._h()
        except BaseException:
            self.bus.close()
            raise

    def __enter__(self) -> Port:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"Port({self.name!r}, baud={self.baudrate}, line={self.line.kind!r}, slaves={sorted(self._axes)})"

    def _h(self) -> int:
        """The core bus handle, made this port's (iad_bus_set_port) once, and again after the bus moved to another
        transport."""
        handle = self.bus._handle
        if handle != self._port_handle:
            _n._check(self._lib, "iad_bus_set_port", handle, _n._utf8(self.name), _LINE_CODES[self._line_kind],
                      _n._num(self._position_unit_mm), int(not self.auto_baud), _PORT_LINE_CB,
                      self._line_key)  # fmt: skip
            self._port_handle = handle
        return handle  # type: ignore[no-any-return]

    def _info(self) -> dict[str, Any]:
        return _json_call(self._lib, "iad_bus_port_info_json", self._h())  # type: ignore[no-any-return]

    @property
    def baudrate(self) -> int | None:
        return self.bus.baudrate

    @property
    def baud_detected(self) -> bool:
        """The rate was found by this port's detection (:meth:`ensure_baudrate`)."""
        return bool(self._info()["baud_detected"])

    @property
    def axes(self) -> dict[int, Elecylinder]:
        """The axes created on this port by slave address (a copy)."""
        with self._lock:
            return dict(self._axes)

    # ------------------------------------------------------------------ line and gateway

    @property
    def line(self) -> Line:
        """The line's rules and gateway (``bus.line``); ``"direct"`` until :meth:`detect_line` has run."""
        return self.bus.line

    @property
    def gateway(self) -> RconGateway | RoboNetGateway | None:
        """The gateway object of an RCON / ROBONET gateway-mode line (``None`` otherwise or before detection)."""
        return self.bus.line.gateway

    def _work_out_line(self) -> None:
        """The detection the core port asks for (iad_port_line_fn): the stated kind, or the gateway probes of the active
        gateway module, which attach the line to the bus."""
        kind = self._line_kind
        if kind in (LINE_DIRECT, LINE_ROBONET):
            self.bus.line = Line(kind, position_unit_mm=self._position_unit_mm)
        else:
            detect_line(self.bus, kind, position_unit_mm=self._position_unit_mm)

    def detect_line(self, *, force: bool = False) -> Line:
        """Work out the line once (the rate first, then READ-ONLY FC03 probes) and return it; ``force`` probes
        again."""
        _waiting(self._lib, "iad_bus_port_detect_line", self._h(), int(bool(force)), None)
        return self.bus.line

    def set_position_unit(self, unit_mm: float, slaves: Iterable[int] | None = None) -> None:
        """State the position unit (0.01 or 0.001 mm; RCON No.207) of ``slaves`` (default: every axis on the line)."""
        line = self.detect_line()
        if slaves is None:
            self._position_unit_mm = unit_mm
            line.set_position_unit(unit_mm)
        else:
            for s in slaves:
                line.set_position_unit(unit_mm, s)

    def gateway_status(self) -> RconGatewayStatus | RoboNetGatewayStatus | None:
        """A fresh read of the gateway's status (``None`` on a line without one). READ-ONLY."""
        gw = self.detect_line().gateway
        return None if gw is None else gw.read_status()

    def gateway_monitor(self, slaves: Iterable[int] | None = None) -> dict[int, RconAxisMonitor]:
        """The RCON all-axes monitor for ``slaves`` (default: this port's axes), ``{slave: RconAxisMonitor}``.
        READ-ONLY."""
        gw = self.detect_line().gateway
        if not isinstance(gw, RconGateway):
            raise EcNotReadyError(f"{self.name}: no RCON gateway on this line (line {self.line.kind!r})")
        wanted = sorted(self.axes) if slaves is None else sorted(set(slaves))
        for s in wanted:
            if not 1 <= s <= 16:
                raise ValueError(f"an axis behind an RCON gateway is slave 1..16, got {s}")
        return {m.slave: m for m in gw.read_monitor(s - 1 for s in wanted).values()}

    # ------------------------------------------------------------------ baud rate and scan

    def ensure_baudrate(self, slaves: Iterable[int] = DEFAULT_SLAVES) -> int | None:
        """Detect the baud rate once (in place) by asking ``slaves`` at each rate; later calls return at once."""
        _waiting(self._lib, "iad_bus_ensure_baud_rate", self._h(), _csv(slaves), None, None)
        return self.baudrate

    def scan(
        self, slaves: Iterable[int] = DEFAULT_SLAVES, *, timeout: float = SCAN_TIMEOUT_S, identify: bool = True,
        use_link_map: bool = True,
    ) -> list[ScanHit]:  # fmt: skip
        """Return the addresses in ``slaves`` that answer, with their identity. READ-ONLY."""
        addresses = list(slaves)
        if not addresses:
            return []
        size = 64 * 1024
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        args = (self._h(), _csv(addresses), float(timeout), int(bool(identify)), int(bool(use_link_map)),
                buffer, size, ctypes.byref(required))  # fmt: skip
        code, details = _n._invoke_interruptible(self._lib, "iad_bus_scan_units_json", args, None)
        if code == _n.IAD_ERR_BUFFER_TOO_SMALL:  # 247 identities do not fit: the scan again (READ-ONLY)
            buffer = ctypes.create_string_buffer(required.value + 4096)
            args = (*args[:5], buffer, len(buffer), ctypes.byref(required))
            code, details = _n._invoke_interruptible(self._lib, "iad_bus_scan_units_json", args, None)
        if code != _n.IAD_OK:
            raise _error(code, details or {})
        return [_scan_hit(h) for h in json.loads(buffer.value.decode("utf-8", errors="replace"))]

    def _identify_hit(self, slave: int) -> ScanHit:
        return _scan_hit(_json_call(self._lib, "iad_bus_identify_json", self._h(), int(slave)))

    def axis(self, slave: int = 1, **kwargs: Any) -> Elecylinder:
        """The :class:`~iai_actuator_driver.ec.Elecylinder` at ``slave`` on this port (created once, on the shared bus;
        without a ``strategy`` the line's required one)."""
        return self._axis(slave, kwargs, None)

    def _axis(self, slave: int, kwargs: dict[str, Any], handle: int | None) -> Elecylinder:
        """:meth:`axis`; ``handle``: a cell's own core axis for it (iad_cell_axis_ref), taken over when the axis is
        new, released otherwise."""
        try:
            if not 1 <= slave <= 247:
                raise ValueError(f"slave address must be 1..247, got {slave}")
            line = self.detect_line()
            if not line.rules.axis_access:
                raise UnsupportedLineError(f"{self.name}: {line.explain()}")
            with self._lock:
                ec = self._axes.get(slave)
                if ec is None:
                    wanted = line.command_strategy
                    if "strategy" not in kwargs and wanted is not None:
                        from .ec import STRATEGIES

                        if wanted in STRATEGIES:
                            kwargs = {**kwargs, "strategy": wanted}
                        else:
                            log.warning("%s: the %s needs command strategy %r, which this axis class does not have",
                                        self.name, line.rules.title, wanted)  # fmt: skip
                    ec = Elecylinder.__new__(Elecylinder)
                    if handle is not None:
                        ec._adopt_handle = handle  # type: ignore[attr-defined]  # taken by Elecylinder._open_core
                    ec.__init__(self.bus, slave=slave, owns_bus=False, **kwargs)  # type: ignore[misc]
                    handle = None
                    self._axes[slave] = ec
                return ec
        finally:
            if handle is not None:
                _n._release(self._lib, "iad_axis_close", handle)

    # ------------------------------------------------------------------ broadcast (daisy chains only)

    def _broadcast_coil(self, coil: int, on: bool) -> bytes:
        self.detect_line().require(OP_BROADCAST, what=f"{self.name}: broadcast")
        return self.bus.broadcast_coil(coil, on)

    def broadcast_stop(self, on: bool = True) -> bytes:
        """STOP coil 0x042C to **every unit on the line** at once (``00 05 04 2C FF 00 4D 12`` / release
        ``00 05 04 2C 00 00 0C E2``). Refused on a gateway line. UNVERIFIED on hardware."""
        from . import registers as R

        return self._broadcast_coil(R.COIL_STOP, on)

    def broadcast_pause(self, on: bool = True) -> bytes:
        """Pause coil STP 0x040A to every unit: ``00 05 04 0A FF 00 AC D9`` / release ``00 05 04 0A 00 00 ED 29``."""
        from . import registers as R

        return self._broadcast_coil(R.COIL_STP, on)

    def broadcast_servo_off(self) -> bytes:
        """Servo OFF (coil SON 0x0403 = 0) to every unit: ``00 05 04 03 00 00 3D 2B``. Refused on a gateway line."""
        from . import registers as R

        return self._broadcast_coil(R.COIL_SON, False)

    def set_baudrate(self, rate: int, *, slaves: Iterable[int] | None = None, **kwargs: Any) -> PortBaudResult:
        """Change the line's baud rate with the broadcast baud change: **every unit on this port switches** (the core's
        Port::set_baud_rate; options ``method``, ``settle_s``; a code not verified on hardware is sent with a warning,
        ``allow_unverified_code`` is accepted and ignored, no longer needed)."""
        unknown = sorted(set(kwargs) - set(_BAUD_OPTIONS))
        if unknown:
            raise TypeError(f"set_baudrate() got unexpected keyword argument(s) {', '.join(unknown)}")
        self.detect_line().require(OP_BAUD_CHANGE, what=f"{self.name}: baud change")
        addresses = sorted(self.axes) if slaves is None else sorted(set(slaves))
        if not addresses:
            raise EcNotReadyError(f"{self.name}: no axis on this port to verify a baud change with")
        o = _n.BaudOptions()
        _n._check(self._lib, "iad_baud_options_init", ctypes.byref(o))
        if "method" in kwargs:
            o.method = _choice(kwargs["method"], _BAUD_METHODS, "method")
        if "settle_s" in kwargs:
            o.settle_s = float(kwargs["settle_s"])
        o.allow_unverified_code = int(bool(kwargs.get("allow_unverified_code", False)))
        size = 64 * 1024
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        _waiting(self._lib, "iad_bus_set_baud_rate_all", self._h(), int(rate), _csv(addresses),
                 ctypes.byref(o), buffer, size, ctypes.byref(required))  # fmt: skip
        data = json.loads(buffer.value.decode("utf-8", errors="replace"))
        return PortBaudResult(str(data["port"]), int(data["rate"]), _record(BaudChangeResult, data["change"]),
                              {int(k): bool(v) for k, v in data["answered"].items()})  # fmt: skip

    def close(self) -> None:
        """Release any STOP coil left ON by an axis (best effort), then close the port. Safe to call twice."""
        for ec in self.axes.values():
            try:
                ec.close()
            except Exception as exc:  # never keep the port open because one axis failed to close
                log.warning("%s: closing slave %d: %s", self.name, ec.slave, exc)
        self.bus.close()


def scan(
    port: str | Transport | Port,
    slaves: Iterable[int] = DEFAULT_SLAVES,
    *,
    timeout: float = SCAN_TIMEOUT_S,
    identify: bool = True,
    baudrate: int | None = None,
    line: str = "auto",
) -> list[ScanHit]:
    """Scan one port for answering addresses (READ-ONLY; see :meth:`Port.scan`). A port name or transport is opened
    for the scan and closed again; a :class:`Port` is left open."""
    if isinstance(port, Port):
        return port.scan(slaves, timeout=timeout, identify=identify)
    with Port(port, baudrate=baudrate, line=line) as p:
        return p.scan(slaves, timeout=timeout, identify=identify)


# ====================================================================== cell


AxisKey = str | int | Elecylinder
PortSpec = str | Transport | Port


class _Faults(dict):  # type: ignore[type-arg]
    """``Cell.faults``: the core cell's faulted axes and their kind; removing an axis (``pop``, ``del``) clears its
    fault in the core cell too (iad_cell_clear_fault)."""

    def __init__(self, cell: Cell, faults: Mapping[str, str]) -> None:
        super().__init__(faults)
        self._cell = cell

    def _clear(self, key: Any) -> None:
        if key in self and self._cell._handle is not None and not self._cell._closed:
            _n._check(self._cell._lib, "iad_cell_clear_fault", self._cell._handle, _n._utf8(str(key)))

    def pop(self, key: Any, *default: Any) -> Any:
        self._clear(key)
        return super().pop(key, *default)

    def __delitem__(self, key: Any) -> None:
        self._clear(key)
        super().__delitem__(key)


def _release_cell(lib: Any, handle: int) -> None:
    """Finalizer of a collected cell: the handle goes, what it uses elsewhere stays open (iad_cell_release)."""
    _n._release(lib, "iad_cell_release", handle)


class Cell:
    """Several axes on one or more ports, driven together. Build it with :meth:`open`.

    ``cell.names`` / ``cell.axes`` are in port order, then slave order; ``cell["COM3:1"]`` or ``cell[0]``
    is one axis (an ordinary :class:`~iai_actuator_driver.ec.Elecylinder`). Methods taking ``axes`` accept names,
    indices or axis objects (default: all).

    Faults (:mod:`iai_actuator_driver.recovery`): ``on_fault(name, kind, error)`` is called once per axis when its link fails
    (``kind``: ``"port_vanished"``, ``"unit_silent"``, ``"unit_restarted"``, ``"unit_replaced"``; ``error`` the
    exception or ``None``); with ``stop_on_fault`` (default True) every other axis is then stopped (:meth:`stop_all`).
    ``faults`` maps the faulted axes to their kind until :meth:`recover_all` brings them back; ``events`` keeps the last
    :class:`~iai_actuator_driver.recovery.RecoveryEvent` objects and ``on_event(event)`` receives each one.

    On the core: the core's ``Cell``.
    """

    def __init__(
        self,
        ports: Mapping[str, Port],
        axes: Sequence[tuple[str, Elecylinder]],
        *,
        on_fault: Callable[[str, str, BaseException | None], Any] | None = None,
        stop_on_fault: bool = True,
        on_event: Callable[[_recovery.RecoveryEvent], Any] | None = None,
    ) -> None:
        self._setup(ports, axes, on_fault, stop_on_fault, on_event)
        if not self._axes:
            return
        handles = (ctypes.c_void_p * len(self._axes))(*(ec._h(False) for ec in self._axes.values()))
        o = self._options(identify=False)  # max_axes 0: no limit
        out = ctypes.c_void_p()
        _n._check(self._lib, "iad_cell_open_axes", handles, len(self._axes), _n._utf8(",".join(self._axes)),
                  ctypes.byref(o), ctypes.byref(out))  # fmt: skip
        self._attach(int(out.value or 0))

    def _setup(
        self,
        ports: Mapping[str, Port],
        axes: Sequence[tuple[str, Elecylinder]],
        on_fault: Callable[[str, str, BaseException | None], Any] | None,
        stop_on_fault: bool,
        on_event: Callable[[_recovery.RecoveryEvent], Any] | None,
    ) -> None:
        self._lib = _n.load()
        self.ports: dict[str, Port] = dict(ports)
        self._axes: dict[str, Elecylinder] = dict(axes)
        self._port_of: dict[str, str] = {}
        for name, ec in self._axes.items():
            for pname, port in self.ports.items():
                if port.bus is ec.bus:
                    self._port_of[name] = pname
                    break
            else:
                raise ValueError(f"axis {name} is not on any port of this cell")
        self.info: dict[str, DeviceInfo] = {}
        self.port_errors: dict[str, BaseException] = {}
        self._motion_commanded = False
        self._closed = False
        self.on_fault = on_fault
        self.stop_on_fault = stop_on_fault
        self.on_event = on_event
        self.events: collections.deque[_recovery.RecoveryEvent] = collections.deque(maxlen=200)
        self._handle: int | None = None
        self._finalizer: Any = None
        self._pump: Any = None
        self._link_monitor: Any = None  # the view of the core cell's link monitor (monitor_links)

    def _options(self, **values: Any) -> _n.CellOptions:
        o = _n.CellOptions()
        _n._check(self._lib, "iad_cell_options_init", ctypes.byref(o))
        o.stop_on_fault = int(bool(self.stop_on_fault))
        for key, value in values.items():
            setattr(o, key, value)
        return o

    def _attach(self, handle: int) -> None:
        """Hold the core cell: released when this object is collected; its fault and link events reach
        :attr:`on_fault`, :attr:`on_event` and :attr:`events` on an event thread of its own."""
        self._handle = handle
        self._finalizer = weakref.finalize(self, _release_cell, self._lib, handle)
        method = weakref.WeakMethod(self._dispatch)  # the event thread does not keep the cell alive

        def dispatch(e: Any) -> None:
            bound = method()
            if bound is not None:
                bound(e)

        self._pump = _n._EventPump(self._lib, "iad_cell_next_event", handle, dispatch, "cell").start()

    def _dispatch(self, e: Any) -> None:
        if e.kind == _n.IAD_EVENT_LINK and e.source == _n.IAD_EVENT_SOURCE_MONITOR:
            return  # an axis monitor's own copy: the core cell passes each monitor event on itself (source CELL)
        if e.kind == _n.IAD_EVENT_LINK:
            event = _n._recovery_event(e)
            self.events.append(event)
            if self.on_event is not None:
                try:
                    self.on_event(event)
                except Exception:  # a broken callback must never break the cell
                    log.exception("cell on_event callback raised")
        elif e.kind == _n.IAD_EVENT_FAULT and self.on_fault is not None:
            message = _n._text(e.message)
            try:
                self.on_fault(_n._text(e.axis), _n._text(e.name), EcDriverError(message) if message else None)
            except Exception:
                log.exception("cell on_fault callback raised")
        elif e.kind == _n.IAD_EVENT_LINK_STATE and self._link_monitor is not None:
            self._link_monitor._changed(e)

    def _flush_events(self) -> None:
        """Let the event thread deliver what the core queued so far (faults, link events: to the callbacks and to
        :attr:`events`) before a call returns; on the event thread itself (a callback that calls the cell) they are
        delivered here, at once."""
        if self._pump is not None:
            self._pump.flush(0.5)

    def _h(self) -> int:
        if self._closed or self._handle is None:
            raise EcNotReadyError("cell is closed")
        return self._handle

    # ------------------------------------------------------------------ construction

    @classmethod
    def open(
        cls,
        ports: PortSpec | Iterable[PortSpec] | Mapping[str, str | Transport] | None = None,
        slaves: str | Iterable[int] | Mapping[str, Iterable[int]] = "scan",
        *,
        max_axes: int | None = None,
        baudrate: int | None = None,
        timeout: float = 0.2,
        retries: int = RECOMMENDED_RETRIES,
        scan_timeout: float = SCAN_TIMEOUT_S,
        scan_slaves: Iterable[int] = DEFAULT_SLAVES,
        identify: bool = True,
        frame_logger: FrameLogger | None = None,
        line: str | Mapping[str, str] = "auto",
        position_unit_mm: float | Mapping[str, float] | None = None,
        on_fault: Callable[[str, str, BaseException | None], Any] | None = None,
        stop_on_fault: bool = True,
        on_event: Callable[[_recovery.RecoveryEvent], Any] | None = None,
        **axis_kwargs: Any,
    ) -> Cell:
        """Open the ports, find the axes and return the cell (READ-ONLY: nothing is written to any unit). The core
        scans (or probes the listed slaves), applies ``max_axes`` (``None``: no limit; a scan keeps the first
        ``max_axes`` axes, listed slaves beyond it are refused) and closes the ports left without an axis."""
        if max_axes is not None and max_axes < 1:
            raise ValueError("max_axes must be >= 1 (or None for no limit)")
        explicit: Mapping[str, Iterable[int]] | list[int] | None
        if isinstance(slaves, str):
            if slaves != "scan":
                raise ValueError("slaves must be 'scan', a list of slave addresses or {port: [slaves]}")
            explicit = None
        elif isinstance(slaves, Mapping):
            explicit = {str(k): list(v) for k, v in slaves.items()}
        else:
            explicit = list(slaves)
        specs = cls._port_specs(ports, explicit)
        if not specs:
            raise EcNotReadyError("no IAI USB adapter found (list_ports); pass ports=[...] to name the ports")
        strict = explicit is not None

        # the ports: a Port given is used as it is, the others are opened here (one that cannot be opened is a port
        # error, as a port where nothing answers)
        opened: dict[str, Port] = {}
        made: list[Port] = []
        errors: dict[str, BaseException] = {}
        for name, spec in specs.items():
            if isinstance(spec, Port):
                opened[name] = spec
                continue
            try:
                port = Port(spec, name=name, baudrate=baudrate, timeout=timeout, retries=retries,
                            frame_logger=frame_logger,
                            line=line.get(name, "auto") if isinstance(line, Mapping) else line,
                            position_unit_mm=position_unit_mm.get(name) if isinstance(position_unit_mm, Mapping)
                            else position_unit_mm)  # fmt: skip
            except (EcDriverError, OSError, ValueError) as exc:
                errors[name] = exc
                continue
            opened[name] = port
            made.append(port)

        def close_made() -> None:
            for port in made:
                port.close()

        if errors and (strict or not opened):
            close_made()
            if strict:
                first = next(iter(errors.values()))
                if len(errors) > 1:
                    first.add_note("iai_actuator_driver: other ports failed too: "
                                   + "; ".join(f"{n}: {e}" for n, e in errors.items()))  # fmt: skip
                raise first
            for name, exc in errors.items():
                log.warning("port %s skipped: %s", name, exc)
            raise EcNotReadyError("no axis found (" + "; ".join(f"{n}: {e}" for n, e in errors.items()) + ")")
        for name, exc in errors.items():
            log.warning("port %s skipped: %s", name, exc)

        # the core: rates, lines, the scan or the listed probes, max_axes, the axes
        if explicit is None:
            slaves_json = None
        elif isinstance(explicit, Mapping):
            slaves_json = json.dumps({n: [int(s) for s in explicit.get(n, [])] for n in opened})
        else:
            slaves_json = json.dumps([int(s) for s in explicit])
        lib = _n.load()
        o = _n.CellOptions()
        _n._check(lib, "iad_cell_options_init", ctypes.byref(o))
        o.max_axes, o.scan_timeout_s = 0 if max_axes is None else int(max_axes), float(scan_timeout)  # 0: no limit
        o.identify, o.stop_on_fault = 0, int(bool(stop_on_fault))
        buses = (ctypes.c_void_p * len(opened))(*(p._h() for p in opened.values()))
        out = ctypes.c_void_p()
        code, details = _n._invoke_interruptible(
            lib, "iad_cell_open_buses",
            (buses, len(opened), _n._utf8(slaves_json), _csv(scan_slaves) or None, ctypes.byref(o), None,
             ctypes.byref(out)),
            None,
        )  # fmt: skip
        if code != _n.IAD_OK:
            close_made()
            exc = _error(code, details or {})
            if errors and isinstance(exc, EcNotReadyError):
                exc.add_note("iai_actuator_driver: ports not opened: "
                             + "; ".join(f"{n}: {e}" for n, e in errors.items()))  # fmt: skip
            raise exc
        handle = int(out.value or 0)
        by_name = {p.name: p for p in opened.values()}
        try:
            info = _json_call(lib, "iad_cell_info_json", handle)
            for name, data in _json_call(lib, "iad_cell_port_errors_json", handle).items():
                errors[name] = _error(int(data.get("code") or _n.IAD_ERR_DRIVER), data)
            axes: list[tuple[str, Elecylinder]] = []
            for name in info["axes"]:
                pname, _, slave = str(name).rpartition(":")
                ref = ctypes.c_void_p()
                _n._check(lib, "iad_cell_axis_ref", handle, _n._utf8(name), ctypes.byref(ref))
                axes.append((name, by_name[pname]._axis(int(slave), dict(axis_kwargs), int(ref.value or 0))))
        except BaseException:
            _n._release(lib, "iad_cell_release", handle)
            close_made()
            raise
        used = {n.rpartition(":")[0] for n, _ in axes}
        for port in opened.values():
            if port.name not in used:
                port.close()  # the core closed its bus already
        cell = cls.__new__(cls)
        cell._setup({p.name: p for p in opened.values() if p.name in used}, axes, on_fault, stop_on_fault, on_event)
        cell._attach(handle)
        cell.port_errors = errors
        if identify:
            try:
                cell.identify_all()
            except BaseException:
                cell.close()
                raise
        return cell

    @staticmethod
    def _port_specs(
        ports: PortSpec | Iterable[PortSpec] | Mapping[str, str | Transport] | None,
        explicit: Mapping[str, Iterable[int]] | list[int] | None,
    ) -> dict[str, PortSpec]:
        if ports is None:
            if isinstance(explicit, Mapping):
                return {name: name for name in explicit}
            return {name: name for name in _iai_port_names()}
        if isinstance(ports, Mapping):
            return {str(name): spec for name, spec in ports.items()}
        if isinstance(ports, (str, Port)) or not isinstance(ports, Iterable):
            ports = [ports]  # type: ignore[list-item]
        specs: dict[str, PortSpec] = {}
        for i, spec in enumerate(ports):  # type: ignore[union-attr]
            if isinstance(spec, str):
                name = spec
            elif isinstance(spec, Port):
                name = spec.name
            else:
                name = str(getattr(spec, "name", "") or f"PORT{i + 1}")
            if name in specs:
                name = f"{name}#{i + 1}"
            specs[name] = spec
        return specs

    # ------------------------------------------------------------------ access

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._axes)

    @property
    def axes(self) -> tuple[Elecylinder, ...]:
        return tuple(self._axes.values())

    def items(self) -> list[tuple[str, Elecylinder]]:
        return list(self._axes.items())

    def __len__(self) -> int:
        return len(self._axes)

    def __iter__(self) -> Iterator[Elecylinder]:
        return iter(self.axes)

    def __contains__(self, key: object) -> bool:
        try:
            self._name(key)  # type: ignore[arg-type]
        except (KeyError, IndexError, TypeError):
            return False
        return True

    def __getitem__(self, key: AxisKey) -> Elecylinder:
        return self._axes[self._name(key)]

    def __repr__(self) -> str:
        return f"Cell({list(self._axes)})"

    def name_of(self, axis: Elecylinder) -> str:
        """The cell name of ``axis``."""
        return self._name(axis)

    def port_of(self, key: AxisKey) -> Port:
        """The :class:`Port` the axis is on."""
        return self.ports[self._port_of[self._name(key)]]

    @property
    def lines(self) -> dict[str, str]:
        """The line kind of every port (``"direct"``, ``"rcon"``, ``"robonet"``, ``"robonet-gateway"``)."""
        return {name: port.line.kind for name, port in self.ports.items()}

    @property
    def gateways(self) -> dict[str, RconGateway | RoboNetGateway]:
        """The gateway object of every port that has one (read-only objects)."""
        return {name: port.gateway for name, port in self.ports.items() if port.gateway is not None}

    @property
    def faults(self) -> dict[str, str]:
        """The faulted axes and their kind (``"port_vanished"``, ``"unit_silent"`` ...) until they are recovered."""
        if self._handle is None or self._closed:
            return _Faults(self, {})
        return _Faults(self, _json_call(self._lib, "iad_cell_info_json", self._handle).get("faults") or {})

    def _name(self, key: AxisKey) -> str:
        if isinstance(key, Elecylinder):
            for name, ec in self._axes.items():
                if ec is key:
                    return name
            raise KeyError(f"{key!r} is not in this cell")
        if isinstance(key, bool):
            raise TypeError("axis key must be a name, an index or an axis")
        if isinstance(key, int):
            return self.names[key]
        if isinstance(key, str):
            if key not in self._axes:
                raise KeyError(f"no axis {key!r} in this cell (axes: {', '.join(self._axes)})")
            return key
        raise TypeError(f"axis key must be a name, an index or an axis, got {key!r}")

    def _select(self, axes: Iterable[AxisKey] | AxisKey | None) -> list[str]:
        if axes is None:
            return list(self._axes)
        if isinstance(axes, (str, int, Elecylinder)):
            axes = [axes]
        return list(dict.fromkeys(self._name(k) for k in axes))

    def _select_known(self, axes: Iterable[AxisKey] | AxisKey | None) -> tuple[list[str], dict[str, BaseException]]:
        """:meth:`_select` of the stop path (stop_all, pause_all): the listed axes this cell has, and each key it does not
        have as an error of its own (keyed by the name, or the key's text), never a refusal of the whole command: a STOP
        never waits on an unrelated bad name (an axis that dropped out of a reopened cell)."""
        if axes is None:
            return list(self._axes), {}
        if isinstance(axes, (str, int, Elecylinder)):
            axes = [axes]
        names: list[str] = []
        unknown: dict[str, BaseException] = {}
        for key in axes:
            try:
                name = self._name(key)
            except (KeyError, IndexError) as exc:
                unknown.setdefault(key if isinstance(key, str) else repr(key), exc)
                continue
            if name not in names:
                names.append(name)
        return names, unknown

    @staticmethod
    def _with_unknown(
        operation: str, out: dict[str, Any], unknown: dict[str, BaseException], return_exceptions: bool
    ) -> dict[str, Any]:
        """The stop path's result: the axes' outcomes, then the unknown keys' errors; one :class:`CellError` for all
        unless ``return_exceptions`` (raised after every known axis was handled)."""
        result = {**out, **{k: v for k, v in unknown.items() if k not in out}}
        errors = {n: v for n, v in result.items() if isinstance(v, BaseException)}
        if errors and not return_exceptions:
            raise CellError(operation, errors, {n: v for n, v in result.items() if n not in errors})
        return result

    # ------------------------------------------------------------------ execution helpers

    def _check_open(self) -> None:
        if self._closed:
            raise EcNotReadyError("cell is closed")

    def _group(
        self, operation: str, function: str, names: list[str], args: tuple[Any, ...], return_exceptions: bool, *,
        waits: bool = True, statuses: bool = True, kept: list[str | None] | None = None, core_operation: str = "",
    ) -> dict[str, Any]:  # fmt: skip
        """One group command of the core: ``function(cell, *args, results, capacity, count)``. ``waits``: on the main
        thread Ctrl+C latches the core's cancel on ``names`` (a decel stop of a motion wait). The per-axis outcomes as
        ``_finish`` gives them. ``kept``: gets the result the call kept (``native._invoke_kept``).
        ``core_operation``: the core's name of the operation when it is not ``operation`` (move_all runs the core's
        move_each): the errors name ``operation``."""
        if not names:
            return {}
        handle = self._h()
        capacity = len(names)
        results = (_n.CellResult * capacity)()
        count = ctypes.c_size_t(0)
        call = (handle, *args, results, capacity, ctypes.byref(count))
        lib = self._lib
        invoke = _n._invoke if kept is None else _n._invoke_kept
        if waits:
            axes_arg = _n._utf8(",".join(names))
            outcome = _n._invoke_interruptible(
                lib, function, call, lambda: _n._invoke(lib, "iad_cell_cancel_latch", handle, axes_arg),
                lambda: _n._invoke(lib, "iad_cell_cancel_reset", handle, axes_arg), call=invoke,
            )  # fmt: skip
        else:
            outcome = invoke(lib, function, *call)
        code, details = outcome[0], outcome[1] or {}
        core_op = str(details.get("operation") or "")
        if core_operation and core_op.startswith(core_operation):
            details = {**details, "operation": operation + core_op[len(core_operation):]}
        if kept is not None:
            kept.append(outcome[2])
        if code not in (_n.IAD_OK, _n.IAD_ERR_CELL):
            raise _error(code, details)
        per_axis = details.get("axes") if isinstance(details.get("axes"), Mapping) else {}
        values: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        for r in results[: min(count.value, capacity)]:
            name = _n._text(r.axis)
            if name not in names and 0 <= r.index < len(names):
                name = names[r.index]  # (0.8) a name longer than the field: the index is the key
            if r.code == _n.IAD_OK:
                ec = self._axes.get(name)
                values[name] = ec._status(r.status) if statuses and ec is not None else None
            else:
                entry = dict(per_axis.get(name) or {}) if isinstance(per_axis.get(name), Mapping) else {}
                entry["code"] = int(r.code)
                entry["message"] = str(entry.get("message") or "") or _n._text(r.message)
                errors[name] = _error(int(r.code), entry)
        if code == _n.IAD_ERR_CELL and not errors:  # no per-axis outcome came back: the error of the call
            raise _error(code, details)
        if errors and not return_exceptions:
            raise CellError(str(details.get("operation") or operation), {n: errors[n] for n in names if n in errors},
                            values)  # fmt: skip
        return {n: errors[n] if n in errors else values.get(n) for n in names}

    def _read_all(self, what: str, operation: str, names: list[str], return_exceptions: bool) -> dict[str, Any]:
        """A read-only group read of the core (iad_cell_read_all_json): ``{name: value}``, the failures as above."""
        if not names:
            return {}
        handle = self._h()
        size = 256 * 1024
        for _ in range(3):
            buffer = ctypes.create_string_buffer(size)
            required = ctypes.c_size_t(0)
            args = (handle, _n._utf8(what), _n._utf8(",".join(names)), buffer, size, ctypes.byref(required))
            code, details = _n._invoke_interruptible(self._lib, "iad_cell_read_all_json", args, None)
            if code == _n.IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
                size = required.value + 4096
                continue
            break
        if code != _n.IAD_OK:
            raise _error(code, details or {})
        data = json.loads(buffer.value.decode("utf-8", errors="replace"))
        results = dict(data.get("results") or {})
        errors = {n: _error(int(e.get("code") or _n.IAD_ERR_DRIVER), e) for n, e in (data.get("errors") or {}).items()}
        if errors and not return_exceptions:
            raise CellError(operation, {n: errors[n] for n in names if n in errors}, results)
        return {n: errors[n] if n in errors else results.get(n) for n in names}

    # ------------------------------------------------------------------ operations

    def identify_all(
        self, axes: Iterable[AxisKey] | None = None, *, return_exceptions: bool = False
    ) -> dict[str, DeviceInfo | BaseException]:
        """:meth:`Elecylinder.identify` on every axis (ports in parallel). READ-ONLY. Stored in :attr:`info`."""
        self._check_open()
        names = self._select(axes)
        out = self._read_all("identify", "identify_all", names, True)
        results: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        for name, value in out.items():
            if isinstance(value, BaseException):
                errors[name] = value
            else:
                results[name] = self._axes[name].device_info = _device_info(value)
        self.info.update(results)
        if errors and not return_exceptions:
            raise CellError("identify_all", errors, results)
        return {n: errors[n] if n in errors else results[n] for n in names}

    def status_all(
        self,
        axes: Iterable[AxisKey] | None = None,
        *,
        return_exceptions: bool = False,
        via_gateway: bool | None = None,
        monitor_max_age_s: float | None = MONITOR_STATUS_AGE_S,
    ) -> dict[str, Status | BaseException]:
        """Read every axis's status: ports in parallel threads, the axes of a port one after another; an axis whose
        status monitor has a sample younger than ``monitor_max_age_s`` is not read. On an RCON line (``via_gateway``
        ``None`` or True) the axes of the port are read from the gateway's all-axes monitor."""
        self._check_open()
        names = self._select(axes)
        results: dict[str, Any] = {}
        rest = names
        if via_gateway is not False:
            rest = []
            groups: dict[str, list[str]] = {}
            for name in names:
                groups.setdefault(self._port_of[name], []).append(name)
            for pname, group in groups.items():
                port = self.ports[pname]
                if not isinstance(port.gateway, RconGateway):
                    rest += group
                    continue
                to_read = []
                for n in group:  # a running monitor's fresh sample first, as for the other ports
                    mon = self._axes[n].monitor if monitor_max_age_s is not None else None
                    st = mon.fresh(monitor_max_age_s) if mon is not None else None
                    if st is not None:
                        results[n] = st
                    else:
                        to_read.append(n)
                try:
                    mons = port.gateway_monitor([self._axes[n].slave for n in to_read]) if to_read else {}
                    for n in to_read:
                        slave = self._axes[n].slave
                        results[n] = _status_from_monitor(mons[slave], port.line.position_unit_mm(slave))
                except Exception as exc:  # fall back to the per-axis reads
                    log.info("%s: gateway monitor not used (%s): reading each axis", pname, exc)
                    rest += to_read
        rest = [n for n in names if n in rest]
        out = self._group("status_all", "iad_cell_status_all", rest,
                          (_n._utf8(",".join(rest)), _n._num(monitor_max_age_s)), True, waits=False)  # fmt: skip
        errors = {n: v for n, v in out.items() if isinstance(v, BaseException)}
        results.update({n: v for n, v in out.items() if not isinstance(v, BaseException)})
        if errors and not return_exceptions:
            raise CellError("status_all", {n: errors[n] for n in names if n in errors}, results)
        return {n: errors[n] if n in errors else results.get(n) for n in names}

    def gateway_status_all(self, *, return_exceptions: bool = False) -> dict[str, Any]:
        """A fresh status read of every port's gateway, ``{port name: RconGatewayStatus | RoboNetGatewayStatus}``
        (ports without a gateway are left out). READ-ONLY."""
        self._check_open()
        results: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        for name, port in self.ports.items():
            gw = port.gateway
            if gw is None:
                continue
            try:
                results[name] = gw.read_status()
            except Exception as exc:
                errors[name] = exc
        if errors and not return_exceptions:
            raise CellError("gateway_status_all", errors, results)
        return {**results, **errors}

    def limits_all(
        self, axes: Iterable[AxisKey] | None = None, *, return_exceptions: bool = False
    ) -> dict[str, dict[str, Any] | BaseException]:
        """:meth:`Elecylinder.limits` of every axis: each unit's own ranges, units (mm / deg), kind and family.
        READ-ONLY (ports in parallel, the axes of a port one after another)."""
        self._check_open()
        return self._read_all("limits", "limits_all", self._select(axes), return_exceptions)

    def monitor_all(
        self, axes: Iterable[AxisKey] | None = None, *, return_exceptions: bool = False
    ) -> dict[str, dict[str, Any] | BaseException]:
        """:meth:`Elecylinder.read_monitor` of every axis (position, velocity, current, deviation, alarm, uptime).
        READ-ONLY (ports in parallel, the axes of a port one after another)."""
        self._check_open()
        return self._read_all("monitor", "monitor_all", self._select(axes), return_exceptions)

    def backup(
        self, directory: str | Path, axes: Iterable[AxisKey] | None = None, *, rows: str | Iterable[int] = "used"
    ) -> dict[str, str]:
        """Read-only backup of every axis (:func:`iai_actuator_driver.info.backup`) into ``directory``, one
        ``<axis>.json`` each. Returns ``{axis name: file path}``, or ``{name: "error: ..."}``."""
        self._check_open()
        from .info import backup_axes

        return backup_axes({n: self._axes[n] for n in self._select(axes)}, directory, rows=rows)

    def units_of(self, key: AxisKey) -> str:
        """Position unit of one axis: ``"deg"`` on a rotary axis, else ``"mm"`` (read from the unit once)."""
        return self[key].units

    def servo_all(
        self, on: bool, axes: Iterable[AxisKey] | None = None, *, timeout: float = 3.0, return_exceptions: bool = False
    ) -> dict[str, Status | None | BaseException]:
        """Servo ON / OFF on every axis and wait for each to confirm (axes in parallel)."""
        self._check_open()
        names = self._select(axes)
        return self._group("servo_all", "iad_cell_servo_all", names,
                           (int(bool(on)), _n._utf8(",".join(names)), float(timeout)), return_exceptions)  # fmt: skip

    def home_all(
        self,
        axes: Iterable[AxisKey] | None = None,
        *,
        timeout: float | None = None,
        method: str | None = None,
        stop_others_on_error: bool = True,
        return_exceptions: bool = False,
    ) -> dict[str, Status | None | BaseException]:
        """Home return on every axis at once and wait for all (each axis polls in its own thread)."""
        self._check_open()
        names = self._select(axes)
        code = _METHOD_DEFAULT if method is None else _choice(method, _METHODS, "method")
        self._motion_commanded = True
        return self._group("home_all", "iad_cell_home_all", names,
                           (_n._utf8(",".join(names)), _n._num(timeout), code, int(bool(stop_others_on_error))),
                           return_exceptions)  # fmt: skip

    def move_all(
        self,
        targets: Mapping[AxisKey, float],
        speed_mm_s: float | Mapping[AxisKey, float] = 50.0,
        accel_g: float | Mapping[AxisKey, float] = 0.1,
        band_mm: float | Mapping[AxisKey, float] = 0.1,
        *,
        decel_g: float | Mapping[AxisKey, float] | None = None,
        wait: bool = True,
        timeout: float | None = None,
        stop_others_on_error: bool = True,
        return_exceptions: bool = False,
    ) -> dict[str, Status | None | BaseException] | None:
        """Start an absolute move on every axis in ``targets`` (``{axis: position}``), then (``wait``) wait for all.
        Every move is checked against its own unit's ranges before any axis starts (one bad value refuses the whole
        move, nothing sent); a start that fails stops the axes already started (``stop_others_on_error``)."""
        self._check_open()
        plan: dict[str, tuple[float, float, float, float, float | None]] = {}
        for key, mm in targets.items():
            name = self._name(key)
            decel = None if decel_g is None else _pick(decel_g, key, name, self)
            plan[name] = (float(mm), _pick(speed_mm_s, key, name, self), _pick(accel_g, key, name, self),
                          _pick(band_mm, key, name, self), decel)  # fmt: skip
        names = list(plan)
        if not names:
            return None if not wait else {}
        moves = _n._sized_array(_n.AxisMove, len(names))
        for m, name in zip(moves, names):
            mm, speed, accel, band, decel = plan[name]
            # (0.8) the move's own fields (NaN: the default); the call's wait and timeout apply
            m.speed, m.accel_g = _n._slot("speed_mm_s", speed), _n._slot("accel_g", accel)
            m.band = _n._slot("band_mm", band)
            m.timeout_s, m.decel_g = _n._num(None), _n._num(decel)
            m.position = mm
            m.axis = name.encode("utf-8")
        try:
            out = self._group("move_all", "iad_cell_move_each", names,
                              (moves, len(names), int(bool(wait)), _n._num(timeout), int(bool(stop_others_on_error))),
                              return_exceptions, core_operation="move_each")  # fmt: skip
        except CellError as exc:
            if "nothing started" not in exc.operation:
                self._motion_commanded = True
            raise
        self._motion_commanded = True
        return out if wait else None

    def wait_all(
        self,
        axes: Iterable[AxisKey] | None = None,
        *,
        timeout: float | None = None,
        stop_others_on_error: bool = True,
        return_exceptions: bool = False,
    ) -> dict[str, Status | BaseException]:
        """Wait until every axis (default: all) is in position and settled; each axis polls in its own thread. An axis
        started by :meth:`move_all` is checked against its target and band."""
        self._check_open()
        names = self._select(axes)
        return self._group("wait_all", "iad_cell_wait_all", names,
                           (_n._utf8(",".join(names)), _n._num(timeout), int(bool(stop_others_on_error))),
                           return_exceptions)  # fmt: skip

    def coordinated(self, *axes: AxisKey | Mapping[str, AxisKey], **named: AxisKey) -> Any:
        """A :class:`~iai_actuator_driver.coord.CoordinatedGroup` of cell axes, for coordinated (interpolated) motion:
        lines, arcs and circles over several axes, planned, run and stopped by the core on this cell (alpha). No I/O."""
        from .coord import CoordinatedGroup

        mapping: dict[str, str] = {}
        for item in axes:
            if isinstance(item, Mapping):
                for alias, key in item.items():
                    mapping[str(alias)] = self._name(key)
            else:
                name = self._name(item)
                mapping[name] = name
        for alias, key in named.items():
            mapping[alias] = self._name(key)
        if not mapping:
            mapping = {name: name for name in self._axes}
        if len(set(mapping.values())) != len(mapping):
            raise ValueError(f"an axis is named twice in {mapping}")
        return CoordinatedGroup({alias: self._axes[name] for alias, name in mapping.items()}, cell=self,
                                cell_names=mapping)  # fmt: skip

    def stop_all(
        self,
        axes: Iterable[AxisKey] | None = None,
        *,
        timeout: float = 5.0,
        return_exceptions: bool = False,
        broadcast: bool = False,
    ) -> dict[str, Status | None | BaseException]:
        """Decel STOP on every axis (default: all), as fast as possible, then wait until each has stopped
        (``broadcast``: one broadcast STOP first on a daisy chain whose axes are all selected; UNVERIFIED). A listed
        key this cell does not have never holds back the STOP: every listed axis it has is stopped, then each unknown
        key is an error of its own (``KeyError``; :class:`CellError` unless ``return_exceptions``)."""
        if self._closed:
            raise EcNotReadyError("cell is closed")
        names, unknown = self._select_known(axes)  # an unknown name never holds back the STOP of the others
        out = self._group("stop_all", "iad_cell_stop_all", names,
                          (_n._utf8(",".join(names)), float(timeout), int(bool(broadcast))), True,
                          waits=False) if names else {}  # fmt: skip
        return self._with_unknown("stop_all", out, unknown, return_exceptions)

    def pause_all(
        self, axes: Iterable[AxisKey] | None = None, *, broadcast: bool = False, return_exceptions: bool = False
    ) -> dict[str, None | BaseException]:
        """Pause every axis (keeps the remaining travel; :meth:`Elecylinder.pause`), ports in parallel
        (``broadcast``: a broadcast pause first on a daisy chain whose axes are all selected; UNVERIFIED)."""
        self._check_open()
        names, unknown = self._select_known(axes)  # as stop_all: the known axes are paused, unknown keys reported
        out = self._group("pause_all", "iad_cell_pause_all", names, (_n._utf8(",".join(names)), int(bool(broadcast))),
                          True, waits=False, statuses=False) if names else {}  # fmt: skip
        return self._with_unknown("pause_all", out, unknown, return_exceptions)

    def resume_all(
        self, axes: Iterable[AxisKey] | None = None, *, broadcast: bool = False, return_exceptions: bool = False
    ) -> dict[str, None | BaseException]:
        """Release the pause on every axis (:meth:`Elecylinder.resume`: the paused moves continue; ``broadcast``: a
        broadcast release afterwards on a daisy chain whose axes are all selected)."""
        self._check_open()
        names = self._select(axes)
        return self._group("resume_all", "iad_cell_resume_all", names,
                           (_n._utf8(",".join(names)), int(bool(broadcast))), return_exceptions, waits=False,
                           statuses=False)  # fmt: skip

    # ------------------------------------------------------------------ power / link loss (recovery)

    def track_all(
        self, axes: Iterable[AxisKey] | None = None, *, return_exceptions: bool = False, **track_kwargs: Any
    ) -> dict[str, _recovery.Session | BaseException]:
        """:func:`iai_actuator_driver.recovery.track` on every axis (Cell::track_all: ports in parallel). READ-ONLY.
        ``track_kwargs``: ``rows``, ``parameters``, ``alarm_history``, ``backup`` (with a backup each axis is tracked
        on its own, one after another; ``info`` is not needed: the core reads the identity)."""
        self._check_open()
        names = self._select(axes)
        unknown = sorted(set(track_kwargs) - {"rows", "parameters", "alarm_history", "backup", "info"})
        if unknown:
            raise TypeError(f"track() got an unexpected keyword argument {unknown[0]!r}")
        results: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        if track_kwargs.get("backup") is not None:
            for name in names:
                try:
                    results[name] = _recovery.track(self._axes[name], name=name, **track_kwargs)
                except Exception as exc:
                    errors[name] = exc
        else:
            o = _n.TrackOptions()
            _n._check(self._lib, "iad_track_options_init", ctypes.byref(o))
            o.alarm_history = int(bool(track_kwargs.get("alarm_history", True)))
            o.parameters = int(bool(track_kwargs.get("parameters", False)))
            rows = track_kwargs.get("rows", False)
            row_text = None if rows is False else "all" if rows is True else ",".join(str(int(n)) for n in rows)
            out = self._group("track_all", "iad_cell_track_all", names,
                              (_n._utf8(",".join(names)), ctypes.byref(o), _n._utf8(row_text)), True,
                              statuses=False)  # fmt: skip
            for name, value in out.items():
                if isinstance(value, BaseException):
                    errors[name] = value
                else:
                    sess = _recovery.session_of(self._axes[name])
                    if sess is None:
                        errors[name] = EcNotReadyError(f"{name}: no session after track_all")
                    else:
                        results[name] = sess
        if errors and not return_exceptions:
            raise CellError("track_all", {n: errors[n] for n in names if n in errors}, results)
        return {n: errors[n] if n in errors else results[n] for n in names}

    def check_links(self, axes: Iterable[AxisKey] | None = None) -> dict[str, _recovery.LinkCheck | BaseException]:
        """:func:`iai_actuator_driver.recovery.check_link` on every axis (Cell::check_links: one status read each,
        ports in parallel). An axis not tracked yet is tracked first (READ-ONLY). A fault goes to ``on_fault`` (and
        stops the other axes, see :class:`Cell`). Returns ``{name: LinkCheck}`` (an unexpected error in place of a
        check)."""
        self._check_open()
        names = self._select(axes)
        if not names:
            return {}
        checks = (_n.LinkCheck * len(names))()
        count = ctypes.c_size_t(0)
        args = (self._h(), _n._utf8(",".join(names)), checks, len(names), ctypes.byref(count))
        code, details, kept = _n._invoke_interruptible(self._lib, "iad_cell_check_links", args, None,
                                                       call=_n._invoke_kept)  # fmt: skip
        if code != _n.IAD_OK:
            raise _error(code, details or {})
        records = _json_or_none(kept).get("error_details") or {}  # (0.7) each failure as the core describes it
        out: dict[str, Any] = {}
        for i, name in enumerate(names[: count.value]):
            c = checks[i]
            if _n._text(c.state) == "error":  # the check itself failed (not a link fault): that error, its own class
                record = records.get(name)
                if isinstance(record, Mapping) and record.get("code"):
                    out[name] = _error(int(record["code"]), dict(record))
                else:
                    out[name] = _recovery._exception_from_text(_n._text(c.error)) or EcDriverError("check failed")
            else:
                out[name] = _recovery._link_check(self._axes[name], c, name)
        self._flush_events()
        return {n: out[n] for n in names if n in out}

    def monitor_links(
        self,
        interval_s: float = 1.0,
        *,
        axes: Iterable[AxisKey] | None = None,
        on_change: Callable[[str, _recovery.LinkCheck], Any] | None = None,
    ) -> _recovery.LinkMonitor:
        """Start the core cell's background link monitor (Cell::monitor_links: :meth:`check_links`' check on every
        axis each ``interval_s``; faults go to ``on_fault`` as above; ``on_change(name, check)`` on every state change,
        from the cell's event thread). It replaces one started earlier and is stopped by :meth:`close`; it never
        recovers or moves anything. Returns a :class:`~iai_actuator_driver.recovery.LinkMonitor` showing it
        (``states``, ``passes``, ``running``, ``stop()``)."""
        self._check_open()
        if not interval_s > 0:
            raise ValueError("interval_s must be > 0")
        selected = {n: self._axes[n] for n in self._select(axes)}
        previous = self._link_monitor
        if previous is not None and previous._kept is None:
            previous._kept = dict(previous._json(), running=False)  # replaced: it keeps what it last showed
        monitor = _recovery._CellLinkMonitor(self, selected, interval_s, on_change)
        return monitor.start()

    def recover_all(
        self,
        axes: Iterable[AxisKey] | None = None,
        *,
        timeout: float = _recovery.RECOVER_TIMEOUT_S,
        policy: _recovery.RecoveryPolicy | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, _recovery.RecoveryReport | BaseException]:
        """:func:`iai_actuator_driver.recovery.recover` on every axis (Cell::recover_all; default: all): ports in
        parallel, the axes of one port one after another (the first reopens a shared port and restores its baud rate
        for all). An axis never tracked is checked against the identity read at :meth:`open`. Recovered axes leave
        :attr:`faults`; each axis's status monitor is held meanwhile and restarted afterwards. Never moves anything: no
        servo ON, no home, no move; a move cut off by the fault is not resumed (the report's ``at_target`` /
        ``needs_home`` say what is left). A failed or replaced axis raises :class:`CellError` after every axis was
        handled (``return_exceptions``: returned in its place). The policy's ``backup`` and ``list_ports`` do not
        reach the cell's recovery."""
        self._check_open()
        names = self._select(axes)
        policy = policy if policy is not None else _recovery.RecoveryPolicy()
        c_policy, row_text, _backup, _note = _recovery._policy(policy, False)
        args = (_n._utf8(",".join(names)), float(timeout), ctypes.byref(c_policy), _n._utf8(row_text))
        kept: list[str | None] = []
        out = self._group("recover_all", "iad_cell_recover_all", names, args, True, statuses=False, kept=kept)
        reports: dict[str, Any] = {}
        if names:
            if kept and kept[0] is not None:  # (0.7) the call's own, whatever another thread's check kept since
                result = _json_or_none(kept[0])
            else:
                result = _json_call(self._lib, "iad_cell_last_result_json", self._h())
            reports = dict(result.get("reports") or {}) if isinstance(result, Mapping) else {}
        results: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        for name, value in out.items():
            if isinstance(value, BaseException):
                errors[name] = _recovery._with_report(value)
            else:
                results[name] = _recovery._report(reports.get(name))
        self._flush_events()
        if errors and not return_exceptions:
            raise CellError("recover_all", {n: errors[n] for n in names if n in errors}, results)
        return {n: errors[n] if n in errors else results[n] for n in names}

    # ------------------------------------------------------------------ status monitors

    def start_monitors(self, axes: Iterable[AxisKey] | None = None, **monitor_kwargs: Any) -> dict[str, Any]:
        """Start the status monitor of every axis (:meth:`Elecylinder.start_monitor`; ``monitor_kwargs``: ``gap_s``,
        ``link_check_s``, ``on_change`` ...). Each monitor's link events go to :attr:`events` / ``on_event`` and to the
        fault handling of the core cell (``on_fault``; with ``stop_on_fault`` every other axis is stopped). READ-ONLY.
        Returns ``{name: AxisMonitor}``; stopped by :meth:`stop_monitors` and :meth:`close`."""
        self._check_open()
        names = self._select(axes)
        out: dict[str, Any] = {name: self._axes[name].start_monitor(**monitor_kwargs) for name in names}
        if names:  # the running monitors' link events reach the cell (their settings stay as started)
            o = _n._monitor_options(self._lib, {})
            _n._check(self._lib, "iad_cell_start_monitors", self._h(), _n._utf8(",".join(names)), ctypes.byref(o))
        return out

    def stop_monitors(self, axes: Iterable[AxisKey] | None = None) -> None:
        """Stop the status monitor of every axis (no-op where none runs)."""
        for name in self._select(axes):
            self._axes[name].stop_monitor()

    # ------------------------------------------------------------------ lifecycle

    def close(self) -> None:
        """Stop the monitors, release STOP coils left ON (best effort) and close every port. Safe to call twice."""
        if self._closed:
            return
        monitor = self._link_monitor
        if monitor is not None:
            try:
                monitor.stop()
            except Exception as exc:
                log.warning("stopping the link monitor: %s", exc)
        self._closed = True
        for port in self.ports.values():
            try:
                port.close()
            except Exception as exc:
                log.warning("closing %s: %s", port.name, exc)
        if self._finalizer is not None:
            if self._pump is not None:
                self._pump.flush(0.5)  # the events of the work done so far reach the callbacks and events first
            self._finalizer()  # iad_cell_release: the axes and ports are closed already
            if self._pump is not None:
                self._pump.stop()

    def __enter__(self) -> Cell:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, tb: object) -> None:
        try:
            if exc is not None and self._motion_commanded and not self._closed:
                try:
                    self.stop_all(return_exceptions=True)
                except Exception as stop_exc:  # never hide the original error
                    exc.add_note(f"iai_actuator_driver: stop_all on leaving the cell failed: {stop_exc}")
                else:
                    exc.add_note("iai_actuator_driver: stop_all sent on leaving the cell")
        finally:
            self.close()

