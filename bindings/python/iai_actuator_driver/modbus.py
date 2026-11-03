"""Modbus RTU master for IAI controllers (ELECYLINDER) over a pluggable byte transport.

Framing follows IAI manual ME0162 (Modbus RTU, CRC-16/MODBUS low byte first, big-endian data).
Besides the standard function codes this client speaks IAI's control-command functions 0x45 and
0x48. 0x45 is ON-WIRE (USB capture of IA-OS talking to an EC: servo off cmd 9, software
reset cmd 5 op1 1, broadcast baud change cmd 4, action mode cmd 0x0C) and VERIFIED on hardware; 0x48
(``addr 48 group cmd(u16) op1..op4(u32)``) is UNVERIFIED: none of the ECs tested uses it.

Design rules:
* The serial port is opened once and kept open. Opening/closing in a loop, or a line break of
  >= 150 ms, drops an EC back to 9600 baud. :class:`SerialTransport` never sends a break.
* One transaction at a time (a lock), at least ``min_gap_s`` (default 5 ms) between requests.
* Timeouts and CRC errors are retried. Exception responses are not retried. IAI's Modbus manual makes 3 retries
  mandatory (DOC ME0162-12A p.4-2: :data:`RECOMMENDED_RETRIES`); :class:`~iai_actuator_driver.cell.Port` and ``Cell`` use 3.
  The bare :class:`ModbusRTU` default is 2, which existing callers rely on.
  On hardware, the read sent right after a mid-move write once got no reply, so the gap and the
  retries are required, not optional. Every driver write is idempotent (absolute targets, coil
  levels; incremental moves are never used), so a retry after a lost response is harmless; if you
  send non-idempotent commands yourself (incremental moves, IAI commands), pass ``retries=0``.
* Response timeout per request (:meth:`ModbusRTU.response_timeout`, DOC ME0162-12A p.4-1, 4-2):
  Tout = 3 x Tint + RTIM + 10 x (response bytes + 8) / kbps [ms] + a USB margin, Tint = 1 ms per register, RTIM =
  the unit's response delay (default 5 ms; :meth:`ModbusRTU.set_response_delay`), never below 0.2 s
  (:data:`MIN_FRAME_TIMEOUT_S`, the VERIFIED EC timing) or the line's minimum (0.3 s on a gateway line). It applies
  to a transport with a real read timeout (a serial port, or a simulated line with latency); a transport without
  one (the plain simulator) answers at once and gets no extra wait.
* Every bus carries a :class:`~iai_actuator_driver.gateway.Line` (``bus.line``, default ``"direct"``). Before a frame is sent
  the line checks it: on an RCON or ROBONET gateway line broadcasts, IAI functions 0x45 / 0x48 and the write
  functions the gateway does not carry are refused (:class:`~iai_actuator_driver.gateway.LineRuleError`, nothing sent).
* Report Slave ID (FC11) waits up to :data:`SLAVE_ID_TIMEOUT_S` (0.65 s) for its answer: the EC-R6S
  rod (unit 0x00, fw 0x0002) answers it only after 0.2-0.4 s (VERIFIED).
* Several axes can share one line (RCON gateway / SIO daisy chain, slave = axis number + 1; UNVERIFIED on
  hardware): one :class:`ModbusRTU` per port, its lock serialises the axes' transactions. :meth:`ModbusRTU.probe`
  asks one address once with a short timeout (:data:`SCAN_TIMEOUT_S`) and no retries, so a scan of 16
  addresses takes about a second (``iai_actuator_driver.cell``).
* Broadcast (slave 0) is never answered. :meth:`ModbusRTU.broadcast_iai_command` sends the IAI baud-rate change in
  the frame IA-OS sends (ON-WIRE: ``00 45 00 04 00 <code> 00 05``); :meth:`ModbusRTU.broadcast` /
  :meth:`ModbusRTU.broadcast_coil` send FC05 / FC06 / FC10 to every unit of a daisy chain (DOC ME0162-12A p.1-1,
  5-3; never FC03), e.g. STOP ``00 05 04 2C FF 00 4D 12``. Refused on gateway lines (DOC ME0413 p.6-1 #2).
* A port that fails underneath the bus (a USB adapter pulled: the COM port vanishes, VERIFIED) raises
  :class:`TransportError` (a :class:`~iai_actuator_driver.errors.ModbusError`, never retried). :meth:`ModbusRTU.reopen` opens
  the same port again in place once it is back (``iai_actuator_driver.recovery`` waits for it and re-syncs the unit).
* IAI commands (0x45 / 0x48): a reply with a non-zero rejection code raises
  :class:`~iai_actuator_driver.errors.IaiCommandRejected` naming the rejection (``registers.IAI_REJECTIONS``) and is
  never retried.
  Commands on the never-send lists (``registers.IAI_NEVER_SEND`` / ``IAI2_NEVER_SEND``: non-volatile writes,
  axis number, baud, alarm-list clear, initialisation, maintenance writes, firmware update, inspection) are
  refused unless ``allow_never_send=True``; any other command not on the safe list needs ``allow_unsafe=True``.

On the core: :class:`ModbusRTU` is a C++ ``ModbusRtu`` (iad_bus) and :class:`SerialTransport` a C++ serial port
(iad_transport). The core runs every transaction: framing, the CRC, the response timeout, the minimum gap, the retries, the echo and byte
count checks, the IAI command safety lists, baud detection and the bus lock. This module converts arguments and
results and keeps the Python objects the caller handed over:

* ``ModbusRTU("COM3")`` opens the port through the module-level :class:`SerialTransport` (the name tests patch).
* Any other transport object (a simulator written in Python, a test double) is driven by the core through
  callbacks on the thread doing the bus I/O, with the bus lock held. A transport with a numeric ``timeout`` is a real
  line (each read may block up to it, and the response timeout formula applies); one without answers at once.
* ``frame_logger`` is the bus's frame sink: every request, response and partial response, on the thread doing the I/O
  (a core thread for a cell's group command). An exception in it is logged and never breaks the bus.
* ``bus.line`` is the line object of ``iai_actuator_driver.gateway``; its kind goes to the core (iad_bus_set_line),
  whose guard refuses what the line does not carry before anything is sent.
* ``exclusive()`` holds the core's bus lock (iad_bus_lock).

The C ABI 0.4 bus functions (``native.py`` binds them); a partial build of the library that lacks one raises
``NotImplementedError`` naming it, and nothing is sent.
"""

from __future__ import annotations

import contextlib
import ctypes
import logging
import math
import struct
import threading
import weakref
from collections.abc import Iterator, Sequence
from typing import Any, Protocol, runtime_checkable

from . import native as _n
from ._protocol import (  # the protocol facts (re-exported here)
    BAUD_BY_CODE,
    BAUD_DETECT_ORDER,
    BAUD_PROBE_ALL,
    BROADCAST_FUNCTIONS,
    BROADCAST_TURNAROUND_S,
    COIL_OFF,
    COIL_ON,
    DEFAULT_BAUDRATE,
    DEFAULT_RESPONSE_DELAY_MS,
    FC_IAI_COMMAND,
    FC_IAI_COMMAND2,
    FC_READ_HOLDING,
    FC_READ_WRITE_REGISTERS,
    FC_REPORT_SLAVE_ID,
    FC_WRITE_COIL,
    FC_WRITE_REGISTER,
    FC_WRITE_REGISTERS,
    IAI_BAUD_CODES,
    IAI_SAFE_COMMANDS,
    IAI_SAFE_COMMANDS2,
    MAX_FC03_REGISTERS,
    MAX_READ_REGISTERS,
    MAX_WRITE_REGISTERS,
    MIN_FRAME_TIMEOUT_S,
    RECOMMENDED_RETRIES,
    SCAN_TIMEOUT_S,
    SLAVE_ID_TIMEOUT_S,
    UNIT_DEPENDENT_BAUD_RATES,
    USB_MARGIN_S,
    VERIFIED_BAUD_RATES,
    FrameLogger,
    IaiResponse,
    IaiResponse2,
    build_frame,
    check_crc,
    crc16,
)
from .errors import (  # re-exported, the same objects
    IaiCommandRejected,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    TransportError,
    UnsafeCommandError,
)
from .gateway import OP_BAUD_CHANGE, OP_BROADCAST, OP_IAI_COMMAND, OP_IAI_COMMAND2, Line  # noqa: F401 (re-exported)
from .registers import IAI2_NEVER_SEND, IAI_NEVER_SEND  # re-exported

__all__ = [
    "crc16",
    "build_frame",
    "check_crc",
    "Transport",
    "SerialTransport",
    "TransportError",
    "ModbusRTU",
    "IaiResponse",
    "IaiResponse2",
    "FrameLogger",
    "MAX_READ_REGISTERS",
    "MAX_FC03_REGISTERS",
    "MAX_WRITE_REGISTERS",
    "IAI_SAFE_COMMANDS",
    "IAI_SAFE_COMMANDS2",
    "BAUD_DETECT_ORDER",
    "BAUD_PROBE_ALL",
    "DEFAULT_BAUDRATE",
    "SLAVE_ID_TIMEOUT_S",
    "SCAN_TIMEOUT_S",
    "IAI_BAUD_CODES",
    "BAUD_BY_CODE",
    "VERIFIED_BAUD_RATES",
    "UNIT_DEPENDENT_BAUD_RATES",
    "RECOMMENDED_RETRIES",
    "MIN_FRAME_TIMEOUT_S",
    "USB_MARGIN_S",
    "DEFAULT_RESPONSE_DELAY_MS",
    "BROADCAST_FUNCTIONS",
    "BROADCAST_TURNAROUND_S",
]

log = logging.getLogger("iai_actuator_driver.modbus")  # the public module name, on every backend

_LINE_KINDS = {"direct": 1, "rcon": 2, "robonet": 3, "robonet-gateway": 4}  # IAD_LINE_*


@runtime_checkable
class Transport(Protocol):
    """Byte pipe used by :class:`ModbusRTU`. :class:`SerialTransport` and ``sim.SimulatedEC`` implement it."""

    def write(self, data: bytes) -> None:
        """Send all of ``data``."""

    def read(self, n: int) -> bytes:
        """Return up to ``n`` bytes, blocking at most the transport's timeout; fewer means timeout."""

    def reset_input_buffer(self) -> None:
        """Discard any received bytes not yet read."""

    def close(self) -> None:
        """Release the underlying resource."""


# ====================================================================== calls and errors


def _error(code: int, details: dict[str, Any]) -> BaseException:
    """The package's exception for a failed call (``native._make_error``), with the core's notes."""
    exc = _n._make_error(code, details)
    for note in details.get("notes") or ():
        if str(note) not in getattr(exc, "__notes__", ()):
            exc.add_note(str(note))
    return exc


def _check(lib: Any, name: str, *args: Any) -> None:
    code, details = _n._invoke(lib, name, *args)
    if code != _n.IAD_OK:
        raise _error(code, details or {})


def _check_u16(name: str, value: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be an int in 0..65535, got {value!r}")
    return value


def _retries_arg(retries: int | None) -> int:
    if retries is None:
        return _n.IAD_INT_UNKNOWN
    if not isinstance(retries, int) or retries < 0:
        raise ValueError("retries must be >= 0")
    return retries


def _nan(value: float | None) -> float:
    return math.nan if value is None else float(value)


def _words(name: str, values: Sequence[int]) -> Any:
    words = [_check_u16(name, v) for v in values]
    return (ctypes.c_uint16 * max(len(words), 1))(*words), len(words)


def _flags(allow_unsafe: bool, allow_never_send: bool, check: bool = True) -> int:
    """IAD_IAI_* opt-ins of an IAI command (with ``check`` the core raises a rejection, with the reply's status)."""
    return _n._iai_flags(allow_unsafe, allow_never_send, check)


def _bytes_in(data: bytes) -> Any:
    raw = bytes(data)
    return (ctypes.c_uint8 * max(len(raw), 1)).from_buffer_copy(raw or b"\0"), len(raw)


# ====================================================================== transports


def _has_read_timeout(transport: Any) -> bool:
    """A numeric ``timeout`` makes a real line: reads may block, the response timeout formula applies."""
    own = getattr(transport, "timeout", None)
    return isinstance(own, (int, float)) and not isinstance(own, bool)


class _Adapter:
    """The object behind one callback transport (``native._keep_callback``): ``target`` (the transport, which
    ``native.py``'s trampolines call), ``read(n, timeout)`` and ``set_timeout(seconds)``. A settable ``timeout`` of the
    transport gets the core's read timeout before the request when the bus shortens it (a probe, ``max_wait``), and
    before a read otherwise. The transport is closed only by :meth:`ModbusRTU.close`: a
    collected bus lets go of it without closing it (iad_bus_release)."""

    __slots__ = ("target", "timeout", "settable")

    def __init__(self, transport: Any) -> None:
        self.target = transport
        self.timeout: float | None = getattr(transport, "timeout", None) if _has_read_timeout(transport) else None
        self.settable = _has_read_timeout(transport)

    def set_timeout(self, seconds: float) -> None:
        if self.settable and seconds != self.timeout:
            try:
                self.target.timeout = seconds
            except (AttributeError, TypeError, ValueError, OSError) as exc:
                log.debug("cannot change the transport timeout: %s", exc)
                self.settable = False
            self.timeout = seconds

    def read(self, n: int, timeout: float) -> bytes:
        self.set_timeout(timeout)
        return bytes(self.target.read(n) or b"")


def _callback_transport(lib: Any, transport: Any) -> tuple[int, _Adapter]:
    """A core transport driving the Python object ``transport`` (iad_transport_open_callbacks): a new handle the
    caller releases, and the adapter behind it."""
    cb = _n.TransportCallbacks()
    cb.struct_size = ctypes.sizeof(cb)  # 0.5: the whole struct (set_read_timeout)
    _check(lib, "iad_transport_callbacks_init", ctypes.byref(cb))
    real = _has_read_timeout(transport)
    if real and transport.timeout >= 0:
        cb.read_timeout_s = float(transport.timeout)
    cb.real_timeout = int(real)
    if real:
        cb.set_read_timeout = _n._c_py_set_read_timeout
    cb.write, cb.read, cb.release = _n._c_py_write, _n._c_py_read, _n._c_py_release
    if callable(getattr(transport, "reset_input_buffer", None)) or callable(getattr(transport, "reset_input", None)):
        cb.reset_input = _n._c_py_reset_input
    if callable(getattr(transport, "set_baudrate", None)):
        cb.set_baud_rate = _n._c_py_set_baud_rate
        rate = getattr(transport, "baudrate", None)
        cb.baud_rate = rate if isinstance(rate, int) and not isinstance(rate, bool) and rate > 0 else 0
    if callable(getattr(transport, "send_break", None)):
        cb.send_break = _n._c_py_send_break
    if hasattr(transport, "present"):
        cb.present = _n._c_py_present
    if callable(getattr(transport, "reopen", None)):
        cb.reopen = _n._c_py_reopen
    cb.close = _n._c_py_close
    label = str(getattr(transport, "port", None) or getattr(transport, "name", None) or "") or None
    adapter = _Adapter(transport)
    key = _n._keep_callback(adapter)
    out = ctypes.c_void_p()
    try:
        _check(lib, "iad_transport_open_callbacks", ctypes.byref(cb), _n._utf8(label), key, ctypes.byref(out))
    except BaseException:
        _n._py_release(key)  # nothing was taken
        raise
    return int(out.value or 0), adapter


class SerialTransport:
    """A serial port of the core (C++ ``SerialTransport``). Opens the port once at construction; never toggles break.

    Args:
        port: e.g. ``"COM3"``.
        baudrate: 115200 for an EC that IA-OS has switched up; ECs boot at 38400 (VERIFIED after a 24 V power cycle).
            :meth:`ModbusRTU.detect_baudrate` / ``Elecylinder.open`` find the rate in place.
        timeout: per-read timeout in seconds.

    Raises :class:`~iai_actuator_driver.errors.TransportError` when the port is busy (another program owns it) or
    absent.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2) -> None:
        self.port = port
        self._lib = _n.load()
        out = ctypes.c_void_p()
        _check(self._lib, "iad_transport_open_serial", _n._utf8(port), int(baudrate), float(timeout),
               ctypes.byref(out))  # fmt: skip
        self._handle = int(out.value or 0)
        self.reopens = 0
        self._finalizer = weakref.finalize(self, _n._release, self._lib, "iad_transport_release", self._handle)

    def _iad_transport(self) -> tuple[int, bool]:
        """(the core transport handle, whether the caller releases it): the bus is built on this port directly."""
        return self._handle, False

    def _call(self, name: str, *args: Any) -> None:
        _check(self._lib, name, self._handle, *args)

    def _info(self) -> dict[str, Any]:
        return _n._read_json(self._lib, "iad_transport_info_json", self._handle)

    def reopen(self, baudrate: int | None = None) -> None:
        """Open the same port again in place, e.g. after the USB adapter was pulled and plugged back in. Same
        settings; ``baudrate`` (default: the rate in use before) and the read timeout carry over. Raises
        :class:`~iai_actuator_driver.errors.TransportError` while the port is still missing."""
        self._call("iad_transport_reopen", int(baudrate or 0))
        self.reopens += 1

    def write(self, data: bytes) -> None:
        buffer, size = _bytes_in(data)
        self._call("iad_transport_write", buffer, size)

    def read(self, n: int) -> bytes:
        buffer = (ctypes.c_uint8 * max(int(n), 1))()
        got = ctypes.c_size_t(0)
        self._call("iad_transport_read", buffer, int(n), ctypes.byref(got))
        return bytes(buffer[: got.value])

    def reset_input_buffer(self) -> None:
        self._call("iad_transport_reset_input")

    @property
    def timeout(self) -> float:
        """Per-read timeout in seconds (:meth:`ModbusRTU.probe` shortens it for one request)."""
        return float(self._info()["read_timeout_s"])

    @timeout.setter
    def timeout(self, value: float) -> None:
        self._call("iad_transport_set_read_timeout", float(value))

    @property
    def baudrate(self) -> int:
        """Current baud rate."""
        return int(self._info().get("baud_rate") or 0)

    def set_baudrate(self, baudrate: int) -> None:
        """Change the baud rate in place. The port stays open, so no line break reaches the EC."""
        self._call("iad_transport_set_baud_rate", int(baudrate))
        self._call("iad_transport_reset_input")

    def send_break(self, duration: float) -> None:
        """Hold the line in BREAK for ``duration`` s. A break of >= 150 ms drops an EC to 9600 baud (ME0162);
        only :meth:`iai_actuator_driver.ec.Elecylinder.set_baudrate` with ``method="break"`` uses it."""
        self._call("iad_transport_send_break", float(duration))

    def close(self) -> None:
        """Close the port (:meth:`reopen` opens it again)."""
        if self._info().get("open"):
            # iad_transport_close also releases the handle it is given: close the port through a bus of its own, so
            # this object keeps its handle for reopen()
            bus = ctypes.c_void_p()
            _check(self._lib, "iad_bus_open", self._handle, None, ctypes.byref(bus))
            _check(self._lib, "iad_bus_close", bus)

    def __repr__(self) -> str:
        try:
            rate = self.baudrate
        except Exception:  # a dead handle: the name is enough
            rate = None
        return f"SerialTransport({self.port!r}, {rate})"


# ====================================================================== the master


class _FrameSink:
    """The bus's frame sink: forwards to the facade's ``frame_logger`` (looked up at call time), logs what it
    raises."""

    __slots__ = ("bus",)

    def __init__(self, bus: ModbusRTU) -> None:
        self.bus = weakref.ref(bus)

    def __call__(self, direction: str, frame: bytes) -> None:
        bus = self.bus()
        logger = None if bus is None else bus._frame_logger
        if logger is None:
            return
        try:
            logger(direction, frame)
        except Exception:  # a broken logger must never break communication
            log.exception("frame logger raised")


def _release_bus(lib: Any, handle: int) -> None:
    """Finalizer of a collected bus: the handle goes and nothing is closed (the
    caller's transport, a Python object or a :class:`SerialTransport`, stays open)."""
    _n._release(lib, "iad_bus_release", handle)


class ModbusRTU:
    """Thread-safe Modbus RTU master (the core's ``ModbusRtu``).

    Args:
        port: a serial port name (opened once via :class:`SerialTransport`) or a :class:`Transport`
            instance such as ``iai_actuator_driver.sim.SimulatedEC()``.
        baudrate: serial baud rate (ignored for a Transport instance).
        timeout: serial read timeout in seconds (ignored for a Transport instance). Each request waits at least
            :meth:`response_timeout` (the manual's Tout, 0.2 s floor) on a transport with a read timeout.
        retries: extra attempts after a timeout / CRC error / mismatched frame (default 2; IAI asks for
            :data:`RECOMMENDED_RETRIES` = 3, which :class:`~iai_actuator_driver.cell.Port` and ``Cell`` use).
        min_gap_s: minimum silence between the end of one transaction and the next request.
        frame_logger: optional ``(direction, frame)`` callback for debugging and captures.

    All request methods take the slave address first (1..247; EC axis n is slave n + 1).
    Broadcast (slave 0) goes through :meth:`broadcast` / :meth:`broadcast_coil` (FC05 / 06 / 10) and
    :meth:`broadcast_iai_command` (the baud change); none is answered.

    :attr:`line` (a :class:`~iai_actuator_driver.gateway.Line`) says what the line carries; ``cell.Port`` sets it after
    probing for a gateway (``gateway.detect_line``). Requests the line does not carry raise
    :class:`~iai_actuator_driver.gateway.LineRuleError` before anything is sent.
    """

    def __init__(
        self,
        port: str | Transport,
        baudrate: int = 115200,
        timeout: float = 0.2,
        retries: int = 2,
        *,
        min_gap_s: float = 0.005,
        frame_logger: FrameLogger | None = None,
    ) -> None:
        if retries < 0:
            raise ValueError("retries must be >= 0")
        if min_gap_s < 0:
            raise ValueError("min_gap_s must be >= 0")
        self._lib = _n.load()
        self._retries = int(retries)
        self._min_gap_s = float(min_gap_s)
        self._closed = False
        self._state = threading.Lock()  # close / reopen / a new transport
        self._line: Line = Line()
        self._sink: _FrameSink | None = None
        self._sink_key: int | None = None
        self._frame_logger: FrameLogger | None = None
        self._handle = 0
        self._transport: Any = None
        self._finalizer: Any = None
        self.transport = SerialTransport(port, baudrate, timeout) if isinstance(port, str) else port
        self.frame_logger = frame_logger

    def _core_transport(self, transport: Any) -> tuple[int, bool, _Adapter | None]:
        """The core transport of ``transport``: (its handle, whether the caller releases it, the adapter of a Python
        transport)."""
        own = getattr(transport, "_iad_transport", None)
        if callable(own):
            handle, release = own()
            return handle, release, None
        handle, adapter = _callback_transport(self._lib, transport)
        return handle, True, adapter

    def _open_bus(self, transport: Any) -> tuple[int, _Adapter | None]:
        """A core bus on ``transport`` with this bus's settings: (its handle, the adapter of a Python transport)."""
        handle, release, adapter = self._core_transport(transport)
        options = _n.BusOptions()
        _check(self._lib, "iad_bus_options_init", ctypes.byref(options))
        options.retries = self._retries
        options.min_gap_s = self._min_gap_s
        out = ctypes.c_void_p()
        try:
            _check(self._lib, "iad_bus_open", handle, ctypes.byref(options), ctypes.byref(out))
        finally:
            if release:  # the bus keeps the transport (on a failure it goes with this handle: released, not closed)
                _n._close_quietly(self._lib, "iad_transport_release", handle)
        return int(out.value or 0), adapter

    @property
    def transport(self) -> Transport:
        """The byte pipe under the bus (a :class:`SerialTransport`, a simulator, a test double). Assigning another
        one moves the bus onto it in place (iad_bus_set_transport): the same core bus keeps its settings, the line,
        the response delays, the frame logger and what is keyed by it (the axes on it, their recovery sessions and
        non-volatile write counts); the old transport is left open."""
        return self._transport  # type: ignore[no-any-return]

    @transport.setter
    def transport(self, value: Transport) -> None:
        if value is self._transport:
            return
        if self._handle:  # 0.6 (fixes): the same core bus moves onto the new pipe
            handle, release, adapter = self._core_transport(value)
            try:
                with self._state:
                    _check(self._lib, "iad_bus_set_transport", self._handle, handle)
                    self._adapter, self._transport = adapter, value
            finally:
                if release:  # the bus keeps its own reference
                    _n._close_quietly(self._lib, "iad_transport_release", handle)
            return
        handle, adapter = self._open_bus(value)
        with self._state:
            self._handle, self._adapter, self._transport = handle, adapter, value
            self._finalizer = weakref.finalize(self, _release_bus, self._lib, handle)
        if callable(getattr(self._line, "_attach", None)):  # (0.5 py-gateway) the first core bus carries this bus's
            self.line = self._line  # line object (a line of the core), so what is stated on it reaches the axes

    # ------------------------------------------------------------------ plumbing

    def _call(self, name: str, *args: Any) -> None:
        if self._closed:  # a plain ModbusError, not the TransportError of a vanished port
            raise ModbusError("bus is closed")
        self._sync_timeout()
        _check(self._lib, name, self._handle, *args)

    def _sync_timeout(self) -> None:
        """A Python transport whose ``timeout`` the application changed since the core last set or read it: the core
        takes the new value (iad_transport_set_read_timeout), so a probe's or ``max_wait``'s shorter wait and the
        response timeout follow it, as if the transport's timeout were read at each request."""
        adapter = getattr(self, "_adapter", None)
        if adapter is None or not adapter.settable:
            return
        value = getattr(adapter.target, "timeout", None)
        if value == adapter.timeout or isinstance(value, bool) or not isinstance(value, (int, float)) or not value >= 0:
            return
        transport = ctypes.c_void_p()
        code, _ = _n._invoke(self._lib, "iad_bus_transport", self._handle, ctypes.byref(transport))
        if code != _n.IAD_OK or not transport.value:
            return
        try:  # the core calls the transport's set_read_timeout back: the adapter then knows the value
            code, details = _n._invoke(self._lib, "iad_transport_set_read_timeout", transport.value, float(value))
            if code != _n.IAD_OK:
                log.debug("the transport's new timeout did not reach the core: %s", (details or {}).get("message"))
        finally:
            _n._close_quietly(self._lib, "iad_transport_release", transport.value)

    def _call_long(self, name: str, *args: Any) -> None:
        """:meth:`_call` of a call that can take seconds (baud detection): on the main thread it runs on the binding's
        helper thread, so a Ctrl+C reaches Python, and is raised once the call has ended on its own timeouts (nothing
        can end it early: it holds the bus)."""
        if self._closed:
            raise ModbusError("bus is closed")
        self._sync_timeout()
        code, details = _n._invoke_interruptible(self._lib, name, (self._handle, *args), None)
        if code != _n.IAD_OK:
            raise _error(code, details or {})

    def _info(self) -> dict[str, Any]:
        return _n._read_json(self._lib, "iad_bus_info_json", self._handle)

    # ------------------------------------------------------------------ settings

    @property
    def retries(self) -> int:
        """Extra attempts after a timeout / CRC error / mismatched frame."""
        return self._retries

    @retries.setter
    def retries(self, value: int) -> None:
        if not isinstance(value, int) or value < 0:
            raise ValueError("retries must be >= 0")
        _check(self._lib, "iad_bus_set_retries", self._handle, int(value))
        self._retries = int(value)

    @property
    def min_gap_s(self) -> float:
        """Minimum silence between the end of one transaction and the next request (s)."""
        return self._min_gap_s

    @min_gap_s.setter
    def min_gap_s(self, value: float) -> None:
        if not value >= 0:
            raise ValueError("min_gap_s must be >= 0")
        _check(self._lib, "iad_bus_set_min_gap", self._handle, float(value))
        self._min_gap_s = float(value)

    @property
    def frame_logger(self) -> FrameLogger | None:
        """Optional ``(direction, frame)`` callback: ``"tx"``, ``"rx"`` and ``"rx-partial"`` frames, on the thread
        doing the I/O. An exception it raises is logged and ignored."""
        return self._frame_logger

    @frame_logger.setter
    def frame_logger(self, logger: FrameLogger | None) -> None:
        if logger is not None and self._sink_key is None:
            self._sink = _FrameSink(self)
            self._sink_key = _n._keep_callback(self._sink)
        self._frame_logger = logger
        if logger is None:
            _check(self._lib, "iad_bus_set_frame_sink", self._handle, _n.FRAME_SINK(), None)
        else:
            _check(self._lib, "iad_bus_set_frame_sink", self._handle, _n._c_py_frame, self._sink_key)

    @property
    def line(self) -> Line:
        """What the line carries (a :class:`~iai_actuator_driver.gateway.Line`; ``"direct"`` until detected)."""
        return self._line

    @line.setter
    def line(self, value: Line) -> None:
        attach = getattr(value, "_attach", None)  # (0.5 py-gateway) a line of the core: attached itself, with its
        if callable(attach):  # gateway and position units
            attach(self._handle)
        else:
            kind = _LINE_KINDS.get(str(getattr(value, "kind", "")))
            if kind is not None:  # the kind: the core's request guard, timeouts and command rules follow it
                _check(self._lib, "iad_bus_set_line", self._handle, kind, math.nan)
        self._line = value
        rules = getattr(self, "_line_rules", None)
        if rules is not None:  # what the caller stated for the axes stays over the new line's kind
            self.line_rules = rules

    # ---- 0.5 (py-gateway): the rules a caller states for the axes on the line
    @property
    def line_rules(self) -> Any:
        """What the axes on this line may send over the line kind's rules (a duck-typed
        ``bus.line_rules``: an object or mapping with ``coils``, ``iai_commands``, ``broadcast``, ``report_slave_id``,
        ``function_codes``, ``allows(operation)``, a ``kind`` / ``name``); ``None`` (default): the kind's. The bus's
        request guard keeps the kind's rules."""
        return getattr(self, "_line_rules", None)

    @line_rules.setter
    def line_rules(self, value: Any) -> None:
        from .gateway import _stated_rules  # the native gateway module reads the duck-typed rules

        _check(self._lib, "iad_bus_state_line_rules", self._handle, _n._utf8(_stated_rules(value)))
        self._line_rules = value

    # ------------------------------------------------------------------ line facts

    def set_response_delay(self, slave: int, delay_ms: float) -> None:
        """Tell the bus the unit's response delay RTIM (EC item 8.4 / CON No.17, ms) for :meth:`response_timeout`."""
        _check(self._lib, "iad_bus_set_response_delay", self._handle, int(slave), float(delay_ms))

    def response_delay_ms(self, slave: int) -> float:
        """The RTIM used for ``slave`` (:data:`DEFAULT_RESPONSE_DELAY_MS` unless told otherwise)."""
        value = ctypes.c_double(0.0)
        _check(self._lib, "iad_bus_response_delay", self._handle, int(slave), ctypes.byref(value))
        return value.value

    def unsupported_functions(self, slave: int) -> frozenset[int]:
        """Function codes ``slave`` answered with exception 01 (Illegal Function) so far on this bus."""
        table = self._info().get("unsupported_functions") or {}
        return frozenset(int(f) for f in table.get(str(int(slave)), ()))

    def response_timeout(self, slave: int, pdu: bytes, expected_len: int | None = None) -> float:
        """How long to wait for the answer to ``pdu`` (seconds), from IAI's formula (DOC ME0162-12A p.4-1, 4-2):
        Tout = 3 x Tint + RTIM + 10 x (response bytes + 8) / kbps [ms] + :data:`USB_MARGIN_S`, never below
        :data:`MIN_FRAME_TIMEOUT_S` (0.2 s), the slow-rate floors or the line's minimum (0.3 s on a gateway line)."""
        buffer, size = _bytes_in(pdu)
        value = ctypes.c_double(0.0)
        expected = 0 if expected_len is None else int(expected_len)  # 0: 64 bytes assumed
        _check(self._lib, "iad_bus_response_timeout", self._handle, int(slave), buffer, size, expected,
               ctypes.byref(value))  # fmt: skip
        return value.value

    # ------------------------------------------------------------------ lifecycle

    @contextlib.contextmanager
    def exclusive(self) -> Iterator[ModbusRTU]:
        """Hold the bus lock for a block: no request of another thread (a status monitor, another axis on the line)
        goes out until the block ends. The calling thread must not send through this bus inside the block."""
        locked = ctypes.c_int32(0)
        _check(self._lib, "iad_bus_lock", self._handle, math.nan, ctypes.byref(locked))
        try:
            yield self
        finally:
            _check(self._lib, "iad_bus_unlock", self._handle)

    def close(self) -> None:
        """Close the transport. Safe to call twice."""
        with self._state:
            if self._closed:
                return
            self._closed = True
        # as ModbusRtu::close: the bus's status monitors stop, the transaction in flight ends, then the transport
        # closes; the handle stays for reopen()
        _check(self._lib, "iad_bus_shutdown", self._handle)

    @property
    def closed(self) -> bool:
        """True after :meth:`close` (until :meth:`reopen`)."""
        return self._closed

    @property
    def can_reopen(self) -> bool:
        """True if the transport can be opened again in place (:meth:`reopen`)."""
        return callable(getattr(self.transport, "reopen", None))

    def reopen(self, *, baudrate: int | None = None) -> None:
        """Open the transport again in place after its port vanished (USB adapter pulled, VERIFIED: the COM port
        disappears) and came back, or after :meth:`close`. Every object holding this bus (axes, a ``cell.Port``) keeps
        working with it; nothing is sent. ``baudrate``: the PC side's rate after the reopen (default: as before).

        Raises:
            TransportError: the port is still missing (``__cause__``: the transport's OSError).
            TypeError: the transport cannot be reopened (e.g. a scripted test transport).
        """
        if not self.can_reopen:
            raise TypeError(f"{self.transport!r} cannot be reopened")
        with self._state:
            _check(self._lib, "iad_bus_reopen", self._handle, int(baudrate or 0))
            self._closed = False

    def __enter__(self) -> ModbusRTU:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ------------------------------------------------------------------ baud rate

    @property
    def baudrate(self) -> int | None:
        """The transport's baud rate, or ``None`` if it has none (e.g. the simulator)."""
        value = getattr(self.transport, "baudrate", None)
        return value if isinstance(value, int) and not isinstance(value, bool) else None

    @property
    def can_change_baudrate(self) -> bool:
        """True if the transport has a baud rate that :meth:`detect_baudrate` can change in place."""
        return self.baudrate is not None and callable(getattr(self.transport, "set_baudrate", None))

    @property
    def can_send_break(self) -> bool:
        """True if the transport can hold the line in BREAK (:meth:`send_break`)."""
        return callable(getattr(self.transport, "send_break", None))

    def set_baudrate(self, baudrate: int) -> None:
        """Change the transport's baud rate in place (the port is never closed). Raises TypeError if it has none."""
        if not self.can_change_baudrate:
            raise TypeError(f"{self.transport!r} has no baud rate to change")
        self._call("iad_bus_set_baud_rate", int(baudrate))

    def send_break(self, duration: float) -> None:
        """Hold the line in BREAK for ``duration`` s (>= 150 ms drops an EC to 9600 baud). Raises TypeError if
        the transport cannot. Nothing else is sent meanwhile (the bus lock is held)."""
        if not self.can_send_break:
            raise TypeError(f"{self.transport!r} cannot send a line break")
        self._call("iad_bus_send_break", float(duration))

    def detect_baudrate(
        self,
        slave: int | Sequence[int | tuple[int, int]],
        candidates: Sequence[int] = BAUD_DETECT_ORDER,
        *,
        probe_address: int = 0x9000,
        current_first: bool = True,
        probe_timeout: float | None = None,
    ) -> int:
        """Find the baud rate ``slave`` answers at and leave the transport set to it (the core's baud detection: the
        current rate first unless ``current_first`` is False, one FC03 read of ``probe_address`` per address and rate,
        no retries; an exception response counts as an answer). ``slave`` may be several addresses, and an entry
        ``(slave, register)`` probes that register (``cell.Port`` adds the gateway probes ``(0xD0, 0x3700)`` and
        ``(0x3F, 0xF700)``). ``probe_timeout`` (default: the transport's timeout for one slave, :data:`SCAN_TIMEOUT_S`
        for several or for a register entry) caps the wait per address. The rate is changed in place: the port is never
        closed or reopened.

        Raises:
            ModbusTimeout: no answer at any rate (the starting rate is restored).
            TypeError: the transport cannot change its baud rate (e.g. the simulator).
        """
        if not self.can_change_baudrate:
            raise TypeError(f"{self.transport!r} has no baud rate to detect")
        entries = [slave] if isinstance(slave, int) else list(slave)
        if not entries:
            raise ValueError("no slave address to probe")
        _check_u16("probe_address", probe_address)
        targets = ",".join(str(int(e)) if isinstance(e, int) else f"{int(e[0]):#04x}@{int(e[1]):#06x}" for e in entries)
        # no candidates: the current rate only (NULL or "" would be the core's default order)
        rates = ",".join(str(int(b)) for b in candidates) or str(self.baudrate)
        rate = ctypes.c_int32(0)
        self._call_long("iad_bus_detect_baud_rate", _n._utf8(targets), _n._utf8(rates), probe_address,
                        int(bool(current_first)), _nan(probe_timeout), ctypes.byref(rate))  # fmt: skip
        return rate.value

    def probe(self, slave: int, *, timeout: float = SCAN_TIMEOUT_S, address: int = 0x9000) -> bool:
        """Ask ``slave`` once (FC03, one register at ``address``) and return True if a valid frame came back.

        No retries, and at most ``timeout`` s of waiting, so a missing address costs ~``timeout``. An exception
        response counts as an answer. A reply from another address, a bad CRC or silence is False. READ-ONLY.
        """
        _check_u16("address", address)
        if not timeout > 0:
            raise ValueError("probe timeout must be > 0")
        answered = ctypes.c_int32(0)
        self._call("iad_bus_probe", int(slave), float(timeout), int(address), ctypes.byref(answered))
        return bool(answered.value)

    # ------------------------------------------------------------------ public requests

    def read_registers(
        self,
        slave: int,
        address: int,
        count: int,
        *,
        retries: int | None = None,
        max_wait: float | None = None,
        chunk: int | None = None,
    ) -> list[int]:
        """FC03: read ``count`` holding registers starting at ``address``.

        Reads larger than :data:`MAX_READ_REGISTERS` (or ``chunk``, at most :data:`MAX_FC03_REGISTERS`) are split
        into several requests. The caller must not span IAI register categories (e.g. 0x9000 and 0x9800) in one
        call. ``retries`` overrides :attr:`retries` for this call (e.g. 0 so that the bus is released after one
        attempt and the caller retries between other threads' requests). ``max_wait`` caps the wait per attempt
        (a probe: a missing unit costs about ``max_wait``).
        """
        _check_u16("address", address)
        if not isinstance(count, int) or count < 1 or address + count > 0x10000:
            raise ValueError(f"invalid register count {count} at {address:#06x}")
        if chunk is not None and (not isinstance(chunk, int) or not 1 <= chunk <= MAX_FC03_REGISTERS):
            raise ValueError(f"chunk must be 1..{MAX_FC03_REGISTERS}, got {chunk}")
        if max_wait is not None and not max_wait > 0:
            raise ValueError("max_wait must be > 0")
        tries = _retries_arg(retries)
        values = (ctypes.c_uint16 * count)()
        self._call("iad_bus_read_registers", int(slave), address, count, tries, _nan(max_wait),
                   _n.IAD_INT_UNKNOWN if chunk is None else chunk, values)  # fmt: skip
        return list(values)

    def write_coil(self, slave: int, address: int, on: bool, *, retries: int | None = None) -> None:
        """FC05: force a single coil ON (FF00h) or OFF (0000h). ``retries`` as in :meth:`read_registers`."""
        _check_u16("address", address)
        tries = _retries_arg(retries)
        self._call("iad_bus_write_coil", int(slave), address, int(bool(on)), tries)

    def write_register(self, slave: int, address: int, value: int) -> None:
        """FC06: write one holding register."""
        _check_u16("address", address)
        _check_u16("value", value)
        self._call("iad_bus_write_register", int(slave), address, value)

    def write_registers(self, slave: int, address: int, values: Sequence[int]) -> None:
        """FC10: write consecutive holding registers in one frame."""
        _check_u16("address", address)
        n = len(values)
        if not 1 <= n <= MAX_WRITE_REGISTERS:
            raise ValueError(f"can write 1..{MAX_WRITE_REGISTERS} registers per frame, got {n}")
        words, count = _words("value", values)
        self._call("iad_bus_write_registers", int(slave), address, words, count)

    def read_write_registers(
        self, slave: int, read_address: int, read_count: int, write_address: int, values: Sequence[int]
    ) -> list[int]:
        """FC17: write ``values`` at ``write_address`` then read ``read_count`` registers, in one frame."""
        _check_u16("read_address", read_address)
        _check_u16("write_address", write_address)
        if not isinstance(read_count, int) or not 1 <= read_count <= MAX_READ_REGISTERS:
            raise ValueError(f"read_count must be 1..{MAX_READ_REGISTERS}")
        if not 1 <= len(values) <= 60:
            raise ValueError("can write 1..60 registers with FC17")
        words, count = _words("value", values)
        out = (ctypes.c_uint16 * read_count)()
        self._call("iad_bus_read_write_registers", int(slave), read_address, read_count, write_address, words, count,
                   out)  # fmt: skip
        return list(out)

    def report_slave_id(
        self, slave: int, *, retries: int | None = None, timeout: float | None = SLAVE_ID_TIMEOUT_S
    ) -> bytes:
        """FC11: return the data bytes after the byte count.

        For an EC (VERIFIED-LIVE): ``[system code, run status, unit code, firmware hi, firmware lo]``;
        system code 0xEC = EC 24 V, 0xED = EC 200 V. ``retries`` as in :meth:`read_registers`.
        ``timeout`` (default :data:`SLAVE_ID_TIMEOUT_S`, 0.65 s) is how long each attempt waits for the
        answer when that is longer than the transport's own timeout; ``None`` = the transport's timeout only.
        """
        tries = _retries_arg(retries)  # None: the bus's
        buffer = (ctypes.c_uint8 * 256)()
        length = ctypes.c_size_t(0)
        self._call("iad_bus_report_slave_id", int(slave), tries, _nan(timeout), buffer, 256, ctypes.byref(length))
        return bytes(buffer[: length.value])

    def iai_command(
        self,
        slave: int,
        group: int,
        command: int,
        op1: int = 0,
        op2: int = 0,
        *,
        allow_unsafe: bool = False,
        allow_never_send: bool = False,
        check: bool = True,
        retries: int | None = None,
    ) -> IaiResponse:
        """IAI function 0x45 "ControlCommand": ``[group u8][cmd u8][op1 u16][op2 u16]``.

        Only commands in :data:`IAI_SAFE_COMMANDS`, addressed to ID group 0, are sent unless ``allow_unsafe`` is True;
        commands on the never-send list (``registers.IAI_NEVER_SEND``) only with ``allow_never_send``
        (:class:`~iai_actuator_driver.errors.UnsafeCommandError`, nothing sent). With ``check`` (default) a non-zero
        rejection code raises :class:`~iai_actuator_driver.errors.IaiCommandRejected` naming the rejection; a
        rejection is a valid answer and is never retried. ``retries`` (timeouts / CRC errors only) as in
        :meth:`read_registers` (pass 0 for a command that must not be sent twice, e.g. a reset).
        """
        _check_u16("op1", op1)
        _check_u16("op2", op2)
        tries = _retries_arg(retries)
        rejection, status = ctypes.c_int32(0), ctypes.c_uint32(0)
        self._call("iad_bus_iai_command", int(slave), 0x45, int(group), int(command), (ctypes.c_uint32 * 2)(op1, op2),
                   2, _flags(allow_unsafe, allow_never_send, check), tries, ctypes.byref(rejection),
                   ctypes.byref(status), 1)  # fmt: skip
        return IaiResponse(rejection.value, status.value)

    def broadcast_iai_command(
        self,
        group: int,
        command: int,
        op1: int = 0,
        op2: int = 0,
        *,
        allow_unsafe: bool = False,
        allow_never_send: bool = False,
        settle_s: float = 0.2,
    ) -> bytes:
        """Send IAI function 0x45 to the broadcast address 0 (every slave on the line acts, none answers).

        Used only for the baud-rate change, in the frame IA-OS sends (ON-WIRE, USB capture:
        ``00 45 00 04 00 14 00 05`` = 115200). The bus lock is held for ``settle_s`` after the frame. The same
        safe-list / never-send rules as :meth:`iai_command` apply. Returns the frame sent.
        """
        if settle_s < 0:
            raise ValueError("settle_s must be >= 0")
        _check_u16("op1", op1)
        _check_u16("op2", op2)
        frame = (ctypes.c_uint8 * 256)()
        length = ctypes.c_size_t(0)
        self._call("iad_bus_broadcast_iai_command", int(group), int(command), op1, op2,
                   _flags(allow_unsafe, allow_never_send), float(settle_s), frame, 256, ctypes.byref(length))  # fmt: skip
        return bytes(frame[: length.value])

    def broadcast(self, pdu: bytes, *, settle_s: float = BROADCAST_TURNAROUND_S) -> bytes:
        """Send ``pdu`` (FC05 / FC06 / FC10 only) to the broadcast address 0: every unit on the line acts, none answers.

        DOC ME0162-12A p.1-1, 5-3, 5-5 to 5-8 (never FC03). The bus lock is held for ``settle_s`` after the frame.
        Refused on a gateway line (:class:`~iai_actuator_driver.gateway.LineRuleError`). Returns the frame sent.
        """
        if settle_s < 0:
            raise ValueError("settle_s must be >= 0")
        buffer, size = _bytes_in(pdu)
        frame = (ctypes.c_uint8 * 256)()
        length = ctypes.c_size_t(0)
        self._call("iad_bus_broadcast", buffer, size, float(settle_s), frame, 256, ctypes.byref(length))
        return bytes(frame[: length.value])

    def broadcast_coil(self, address: int, on: bool, *, settle_s: float = BROADCAST_TURNAROUND_S) -> bytes:
        """FC05 to every unit on the line (address 0), e.g. STOP ``00 05 04 2C FF 00 4D 12``; see :meth:`broadcast`."""
        _check_u16("address", address)
        return self.broadcast(struct.pack(">BHH", FC_WRITE_COIL, address, COIL_ON if on else COIL_OFF),
                              settle_s=settle_s)  # fmt: skip

    def iai_command2(
        self,
        slave: int,
        command: int,
        operands: Sequence[int] = (),
        *,
        allow_unsafe: bool = False,
        allow_never_send: bool = False,
        check: bool = True,
        group: int = 0,
    ) -> IaiResponse2:
        """IAI function 0x48 "ControlCommand2": ``[group u8][cmd u16][op1..op4 u32]`` (19 data bytes).

        A rejection (0x1000-0x13FF) raises :class:`~iai_actuator_driver.errors.IaiCommandRejected` and is never
        retried. UNVERIFIED: none of the ECs tested uses 0x48. The full 16-bit code must be in
        :data:`IAI_SAFE_COMMANDS2` and ``group`` must be 0 unless ``allow_unsafe`` is True; codes on
        ``registers.IAI2_NEVER_SEND`` need ``allow_never_send``.
        """
        _check_u16("command", command)
        if not isinstance(group, int) or not 0 <= group <= 0xFF:
            raise ValueError(f"group must be an int in 0..255, got {group!r}")
        if len(operands) > 4:
            raise ValueError("at most 4 operands")
        ops = list(operands)
        for i, op in enumerate(ops):
            if not isinstance(op, int) or not 0 <= op <= 0xFFFFFFFF:
                raise ValueError(f"operand {i + 1} must be a u32")
        flags = _flags(allow_unsafe, allow_never_send, check)
        values = (ctypes.c_uint32 * 4)(*ops)
        rejection = ctypes.c_int32(0)
        status = (ctypes.c_uint32 * 4)()
        self._call("iad_bus_iai_command", int(slave), 0x48, group, command, values, len(ops), flags,
                   _n.IAD_INT_UNKNOWN, ctypes.byref(rejection), status, 4)  # fmt: skip
        return IaiResponse2(rejection.value, (status[0], status[1], status[2], status[3]))

