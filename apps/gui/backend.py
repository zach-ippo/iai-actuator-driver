"""Adapter between the test GUI and the ``iai_actuator_driver`` package.

Every call into the driver lives in this module; the controller only sees plain dicts and floats.
A session is either a real EC on a serial port (baud rate auto-detected in place, the port is
opened once) or ``iai_actuator_driver``'s :class:`~iai_actuator_driver.sim.SimulatedEC` (no serial port at all).

Commands are sent without the driver's blocking waits, so the GUI's driver lock is held only for
the request itself and the poller shows progress. Two follow-ups therefore live here:

* Home: ``Elecylinder.home(wait=False)`` leaves the edge-triggered HOME coil ON. It is released
  with ``finish_home()`` when status shows the home return complete (the driver's own rule:
  started, then HEND + PEND and no longer homing), when it did not start within 2 s, on an alarm
  or servo off, on STOP and on close.
* STOP always goes through ``Elecylinder.stop()``: the hardware-verified STOP coil sequence
  (0x042C ON, hold until the axis has stopped, OFF). It never touches PMSL. With PMSS = 0 it
  stopped a Modbus-commanded move (hardware test 19); on a PLC/PIO-commanded move it is untested
  (the controller watches for a restart after STOP while PIO has control). ``stop()`` returns the
  last status of the hold (the standstill), which arms that watch.

Stroke: ``stroke=None`` (the default) means the EC's own soft limits (0x0700 minus the 0.30 mm
margin, as the driver enforces them); a given stroke can only narrow them. Speed: the driver caps
every speed at min(100 mm/s, the EC's own maximum speed MAXV, item 7.4) - 53 mm/s on the gripper.

Unit info (read-only): ``unit_info`` is ``Elecylinder.unit_info`` (FC03 + FC11 only, every block read
bounded by the EC's own directory); ``read_block`` reads raw words of one directory block for the
register browser; ``decode_live`` decodes the status poll's words with the same bit tables, without
any bus access. Inside ``preemptible(gate)`` these reads (and ``identify`` / ``table_row``) go
through ``gate`` one Modbus request at a time, in short requests without in-lock retries: the
controller takes its bus lock per request there, so STOP waits for one short request at most.

Writes beyond motion (each behind a confirm dialog in the browser, all through the driver's own
guards): ``write_param`` (``Elecylinder.write_user_parameter``, committed with PARW, the commit IA-OS
sends; applies after a software reset), ``software_reset`` (``Elecylinder.software_reset``: servo OFF, 0x45
cmd 5, link re-synced in place), ``set_baudrate`` (``Elecylinder.set_baudrate``, broadcast 0x45 cmd 4,
volatile) and ``run_stored`` (``Elecylinder.move_to_position``, 0x9800 = n, without waiting). The
simulator session models the serial line at ``SIM_LINE_BAUD`` so the baud selector works there too
(its ``baud`` stays None: no serial port).
"""

from __future__ import annotations

import contextlib
import re
import sys
import threading
import time
from collections.abc import Callable, Iterable, Iterator, Mapping
from contextlib import AbstractContextManager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

ROOT = Path(__file__).resolve().parents[2]  # the repository root of a source checkout
_DRIVER = ROOT / "bindings" / "python"  # its Python package, used before an installed one
if (_DRIVER / "iai_actuator_driver").is_dir() and str(_DRIVER) not in sys.path:
    sys.path.insert(0, str(_DRIVER))

from iai_actuator_driver import BACKEND  # noqa: E402 (needs the sys.path entry above)
from iai_actuator_driver import errors as ec_errors  # noqa: E402
from iai_actuator_driver import info as ec_info  # noqa: E402
from iai_actuator_driver import registers as R  # noqa: E402
from iai_actuator_driver.alarms import alarm_info, alarm_is_fault, alarm_name  # noqa: E402
from iai_actuator_driver.ec import Elecylinder, Status  # noqa: E402
from iai_actuator_driver.modbus import BAUD_DETECT_ORDER, VERIFIED_BAUD_RATES, ModbusRTU  # noqa: E402,F401 (re-exported)
from iai_actuator_driver.sim import PROFILES as SIM_PROFILES  # noqa: E402
from iai_actuator_driver.sim import SimulatedEC  # noqa: E402

DEFAULT_STROKE: tuple[float, float] | None = None  # None: the EC's own soft limits (minus their 0.30 mm margin)
DEFAULT_SIM_PROFILE = "slider"
SIM_PROFILE_NAMES: tuple[str, ...] = tuple(sorted(SIM_PROFILES))  # gripper, pcon, rod, rotary, slider, unknown
TABLE_LABELS: dict[int, str] = {0: "Backward", 1: "Forward", 2: "Middle"}
DRIVER_MAX_SPEED_MM_S = 100.0  # the GUI's own cap; the driver lowers it to the EC's MAXV (item 7.4)
STOP_HOLD_S = 3.0  # longest the driver holds the STOP coil while the axis decelerates
HOME_START_S = 2.0  # a home return must visibly start within this time (as in Elecylinder.home)
BROWSE_DEFAULT_WORDS = 256  # register browser: words per read when no count is given
BROWSE_MAX_WORDS = 512  # register browser: most words per read (the position table is paged)
# Preemptible reads (Unit info, register browser): words per FC03, so that one request is short
# (32 words = 77 bytes on the line: 20 ms at 38400 baud, 7 ms at 115200).
GATED_READ_WORDS = 32
SIM_LINE_BAUD = 115200  # the simulator session's modelled line rate (the baud selector changes it)

ReadGate = Callable[[], AbstractContextManager[None]]  # entered around every request of a preemptible read


class BackendError(Exception):
    """User-facing error with an HTTP status hint (400 bad input, 409 wrong state, 502 driver). ``extra`` fields
    (e.g. ``kind="in_use"``, ``port="COM3"``) go into the error JSON next to ``error``."""

    def __init__(self, message: str, http_status: int = 409, **extra: Any) -> None:
        super().__init__(message)
        self.http_status = http_status
        self.extra: dict[str, Any] = dict(extra)


# The names a serial port can have, for both pages: COM1..COM999, and on Linux / macOS / BSD a tty, a cu (macOS
# cu.*, BSD cua*), an rfcomm or a serial/by-id / by-path device under /dev. Nothing else reaches pyserial (a UNC name such as \\host\share makes Windows contact that
# host with the user's credentials; "/dev/../x" leaves /dev).
PORT_NAME = re.compile(
    r"^(?:(?i:COM)[1-9][0-9]{0,2}|/dev/(?:tty[\w.-]{1,32}|cu[a.][\w.-]{1,48}|rfcomm[0-9]{1,3}"
    r"|serial/by-(?:id|path)/[\w.:+-]{1,128}))$",
    re.ASCII)


def check_port_name(name: Any) -> str:
    """The port name, stripped, when it is a serial port name (:data:`PORT_NAME`); else a 400 ``BackendError``."""
    text = name.strip() if isinstance(name, str) else ""
    if not PORT_NAME.match(text) or ".." in text:
        shown = text[:40] + ("..." if len(text) > 40 else "")
        raise BackendError(f"Port {shown!r} is not a valid serial port name (e.g. COM3 or /dev/ttyUSB0).", 400)
    return text


class ReadOnlyViolation(RuntimeError):
    """A write was attempted from inside a preemptible read-only read (never happens by design)."""


@dataclass(frozen=True)
class EcLimits:
    """The EC's own limits, read at connect (parameter category 7, 0x0700 on the test EC)."""

    soft_limits_mm: tuple[float, float]  # as stored on the EC, e.g. -0.30 / +150.30
    stroke_mm: tuple[float, float]  # the EC's nominal stroke: soft limits minus its 0.30 mm margin
    max_speed_mm_s: float | None = None  # the EC's maximum speed (MAXV item 7.4), if readable


class AxisSession(Protocol):
    """What the GUI controller needs from one connected axis."""

    description: str
    baud: int | None  # the baud rate the EC answered at; None for the simulator

    @property
    def stroke(self) -> tuple[float, float] | None: ...  # limits in use (None: unknown); the driver enforces them too
    @property
    def max_speed_mm_s(self) -> float: ...  # speed cap in use: min(GUI cap, the EC's MAXV (7.4) once read)
    def identify(self) -> dict[str, Any]: ...
    def status(self) -> dict[str, Any]: ...
    def ec_limits(self) -> tuple[EcLimits | None, str]: ...
    def set_stroke(self, stroke: tuple[float, float]) -> None: ...
    def enable_control(self) -> None: ...
    def release_control(self) -> None: ...
    def servo(self, on: bool) -> None: ...
    def home(self) -> None: ...
    def move_to(self, position_mm: float, speed_mm_s: float, accel_g: float) -> None: ...
    def jog(self, delta_mm: float, speed_mm_s: float, accel_g: float) -> None: ...
    def push(self, target_mm: float, band_mm: float, force_pct: float, direction: int,
             speed_mm_s: float, accel_g: float) -> None: ...
    def stop(self) -> dict[str, Any] | None: ...  # blocks until stopped; the last status of the hold (standstill)
    def reset_alarm(self) -> None: ...
    def run_stored(self, n: int, allow_above_caps: bool) -> None: ...  # 0x9800 = n (stored speed), no wait
    def write_param(  # committed (PARW)
        self, no: int, value: Any, allow_home_params: bool, allow_unverified_model: bool = False
    ) -> dict[str, Any]: ...
    def software_reset(self) -> dict[str, Any]: ...  # servo OFF + 0x45 cmd 5; blocks until the EC answers again
    def set_baudrate(self, rate: int, method: str) -> dict[str, Any]: ...  # broadcast 0x45 cmd 4; volatile
    @property
    def line_baud(self) -> int | None: ...  # the port's current rate (also the simulator's modelled line)
    def table_row(self, n: int) -> dict[str, Any]: ...
    def unit_info(self, sections: Iterable[str]) -> dict[str, Any]: ...  # read-only (FC03 + FC11)
    def read_block(self, class_id: int, offset: int, count: int | None) -> dict[str, Any]: ...  # read-only (FC03)
    # Read-only calls made by this thread inside the context go through ``gate`` one request at a time.
    def preemptible(self, gate: ReadGate) -> AbstractContextManager[None]: ...
    def close(self) -> None: ...


def error_status(exc: BaseException) -> int:
    """HTTP status for a driver exception: 400 bad value, 409 actuator refused/state, 502 communication."""
    if isinstance(exc, ec_errors.SoftLimitError):
        return 400
    if isinstance(exc, (ec_errors.EcError, ec_errors.UnsafeCommandError)):
        return 409
    if isinstance(exc, (ec_errors.ModbusError, OSError, TimeoutError)):
        return 502
    if isinstance(exc, ValueError):
        return 400
    return 502


def open_session(
    port: str, baud: int, slave: int, simulate: bool, *, stroke: tuple[float, float] | None = DEFAULT_STROKE,
    sim_profile: str = DEFAULT_SIM_PROFILE,
) -> AxisSession:
    """Open one axis. ``stroke`` None: the EC's own soft limits; otherwise an override narrowed to them.

    ``sim_profile`` picks the simulated unit (``"slider"`` or ``"gripper"``); ignored for a real port.
    """
    return DriverSession(port, baud, slave, simulate, stroke, sim_profile)


def decode_live(
    raw: Mapping[str, int], mode: Mapping[str, Any] | None = None, model: str | None = None
) -> dict[str, Any]:
    """The live Unit info sections from one status poll's words (``status_dict(...)["raw"]``). No bus access.

    Same decoders and bit tables as ``Elecylinder.unit_info``; ``mode`` is the position mode from the
    last unit info read (else guessed from the ``model`` string, else the 2-position layout, assumed).
    Words the status poll does not read (POSS, SSSE, OLLV, ...) are left out.
    """
    mode = mode if mode is not None else ec_info.position_mode(None, model)
    live = ec_info.decode_monitor(raw, mode)
    code = raw.get("ALMC")
    if code is not None:
        about = alarm_info(code)
        dss1 = raw.get("DSS1", 0)
        fault = alarm_is_fault(code, alarm_bit=bool(dss1 & R.DSS1_ALMH),
                               warning_bit=bool(dss1 & R.DSS1_ALML))  # fmt: skip
        live["alarm"] = {
            "code": code, "hex": f"0x{code:03X}", "name": alarm_name(code), "active": code != 0,
            "level": about["level"] if code else None, "fault": fault, "warning": code != 0 and not fault,
            "remedy": about["remedy"],
        }  # fmt: skip
    live["position_mode"] = dict(mode)
    return live


def status_dict(st: Status) -> dict[str, Any]:
    """The GUI's view of an ``iai_actuator_driver`` Status (JSON-friendly)."""
    return {
        "position_mm": st.position_mm,
        "speed_mm_s": st.velocity_mm_s,
        "current": st.current_ma,
        "alarm_code": st.alarm_code,
        "alarm": st.alarm,  # a fault: ALMH, or alarm level operation cancelled / cold start (blocks motion)
        "warning": st.warning,  # a message-level alarm (0x011-0x07F): the axis keeps running
        "alarm_level": st.alarm_level,
        "servo_on": st.servo_on,
        "homed": st.homed,
        "in_position": st.in_position,
        "moving": st.moving,
        "modbus_control": st.modbus_enabled,
        "emergency_stop": st.emergency_stop,
        "safety_speed": st.safety_speed,
        "paused": st.paused,
        "homing": st.homing,
        "pushing": st.pushing,
        "push_contact": st.push_contact,
        "push_missed": st.push_missed,
        "target_reached": st.target_reached,
        "move_bit": st.move_bit,
        # Position markers from ZONS 0x9013 b12/b13/b14 (None when ZONS was not read), never from DOPM.
        "at_backward_end": st.at_backward_end,
        "at_forward_end": st.at_forward_end,
        "at_middle": st.at_middle,
        "raw": {name: int(value) for name, value in st.raw.items()},
    }


class _GatedBus:
    """The session's ModbusRTU as the driver sees it, with preemptible reads for one thread at a time.

    Inside :meth:`gated` (and only in the thread that entered it) every FC03 / FC11 goes through
    ``gate()`` - the controller's bus lock taken for that one request - in requests of at most
    ``GATED_READ_WORDS`` words and without the bus's in-lock retries (they are retried here, each
    attempt gated again), so another thread's STOP waits for one short request at most. A write
    from that thread is refused (such a read is read-only by construction). Every other thread
    (status poll, commands, STOP) uses the bus directly, exactly as before.
    """

    _WRITES = frozenset({"write_coil", "write_register", "write_registers", "read_write_registers",
                         "iai_command", "iai_command2", "broadcast_iai_command", "set_baudrate", "send_break",
                         "detect_baudrate"})

    def __init__(self, bus: ModbusRTU) -> None:
        self.bus = bus
        self._local = threading.local()

    @property
    def gate(self) -> ReadGate | None:
        """The gate of a preemptible read running in the calling thread, if any."""
        return getattr(self._local, "gate", None)

    @contextlib.contextmanager
    def gated(self, gate: ReadGate) -> Iterator[None]:
        previous = self.gate
        self._local.gate = gate
        try:
            yield
        finally:
            self._local.gate = previous

    def read_registers(self, slave: int, address: int, count: int, *, retries: int | None = None) -> list[int]:
        gate = self.gate
        if gate is None:
            return self.bus.read_registers(slave, address, count, retries=retries)
        if count < 1 or address + count > 0x10000:
            raise ValueError(f"invalid register count {count} at {address:#06x}")
        values: list[int] = []
        while count > 0:
            n = min(count, GATED_READ_WORDS)
            values += self._request(gate, lambda a=address, n=n: self.bus.read_registers(slave, a, n, retries=0))
            address += n
            count -= n
        return values

    def report_slave_id(self, slave: int, *, retries: int | None = None, **kwargs: Any) -> bytes:
        gate = self.gate
        if gate is None:
            return self.bus.report_slave_id(slave, retries=retries, **kwargs)
        if retries == 0:  # sent once (the driver's rule for FC11; it then falls back to CTCL)
            with gate():
                return self.bus.report_slave_id(slave, retries=0, **kwargs)
        return self._request(gate, lambda: self.bus.report_slave_id(slave, retries=0, **kwargs))

    def _request(self, gate: ReadGate, send: Callable[[], Any]) -> Any:
        attempts = self.bus.retries + 1
        for attempt in range(attempts):
            with gate():
                try:
                    return send()
                except (ec_errors.ModbusTimeout, ec_errors.ModbusCRCError, ec_errors.ModbusFrameError) as exc:
                    if attempt + 1 >= attempts:
                        if attempts > 1:
                            exc.add_note(f"gave up after {attempts} attempts")
                        raise
        raise AssertionError("unreachable")

    def __getattr__(self, name: str) -> Any:
        attr = getattr(self.bus, name)  # everything else is the bus itself
        if name not in self._WRITES:
            return attr

        def guarded(*args: Any, **kwargs: Any) -> Any:
            if self.gate is not None:
                raise ReadOnlyViolation(f"{name} refused: a read-only Unit info / register read runs in this thread")
            return attr(*args, **kwargs)

        return guarded


class _CoreReads:
    """Preemptible reads on the core: the core's own scope for the same job as :class:`_GatedBus`.

    ``iad_bus_preemptible_begin`` (``NativeBus.preemptible``) puts the calling thread's FC03 reads of that bus into
    frames of ``GATED_READ_WORDS`` words, takes the bus per attempt after the gate (retries outside the bus lock),
    ends the read with whatever the gate raises, and refuses this thread's writes (``UnsafeCommandError``, where
    :class:`_GatedBus` raises :class:`ReadOnlyViolation`). Unit info, identify, table rows and register reads run
    inside the core's calls, so they cannot be split from Python as :class:`_GatedBus` does. Other threads (status
    poll, commands, STOP) are not affected. The scope belongs to the thread that begins it: the GUI's request threads
    do, and a call made on the main thread of the process would run on the binding's helper thread, outside it.
    """

    def __init__(self, bus: Any) -> None:
        from iai_actuator_driver import native  # only reached for a bus with a core handle

        # A borrowed view of the session bus's own handle (what the driver's modules do as well): the bus alone
        # closes it, so this object must never release or close it.
        self._scope = native.NativeBus(bus._lib, bus._handle, name="gui reads", owned=False)
        self._scope._finalizer.detach()

    def preemptible(self, gate: ReadGate) -> AbstractContextManager[Any]:
        def before_request() -> None:  # the core calls this before every request attempt; raising ends the read
            with gate():
                pass

        return self._scope.preemptible(before_request, words=GATED_READ_WORDS)


class DriverSession:
    """One ``iai_actuator_driver.Elecylinder`` on a serial port or on the simulator."""

    def __init__(
        self, port: str, baud: int, slave: int, simulate: bool, stroke: tuple[float, float] | None,
        sim_profile: str = DEFAULT_SIM_PROFILE,
    ) -> None:
        self.baud: int | None
        if simulate:
            if sim_profile not in SIM_PROFILE_NAMES:
                raise BackendError(f"Simulator profile must be one of {', '.join(SIM_PROFILE_NAMES)}.", 400)
            # A Transport instance: no serial port. The line is modelled (SIM_LINE_BAUD) for the baud selector.
            self._bus = ModbusRTU(SimulatedEC(slave=slave, profile=sim_profile, baudrate=SIM_LINE_BAUD))
            self.baud = None
            self.description = f"iai_actuator_driver simulator (SimulatedEC, {sim_profile} profile, no serial port)"
        else:
            self._bus = _open_serial(port, baud, slave)
            self.baud = self._bus.baudrate
            how = "as selected" if self.baud == baud else f"auto-detected; {baud} did not answer"
            self.description = f"iai_actuator_driver on {port} @ {self.baud} baud ({how})"
        # Preemptible reads: the core's own scope (the bus is the driver's bus as it is); for a bus without a core
        # handle (a test double) a wrapper bus that is direct except inside preemptible().
        self._core_reads = _CoreReads(self._bus) if BACKEND == "native" and getattr(self._bus, "_handle", None) else None
        self._gated = None if self._core_reads is not None else _GatedBus(self._bus)
        try:
            bus = self._bus if self._gated is None else self._gated
            self._ec = Elecylinder(bus, slave=slave, stroke=stroke,  # type: ignore[arg-type]
                                   max_speed_mm_s=DRIVER_MAX_SPEED_MM_S)
        except Exception:
            self._bus.close()
            raise
        self._configured = None if stroke is None else (float(stroke[0]), float(stroke[1]))
        self._home_started: float | None = None  # set while the HOME coil may still be raised
        self._home_seen_start = False

    # ------------------------------------------------------------------ read-only

    @property
    def stroke(self) -> tuple[float, float] | None:
        """Limits every target is checked against: the EC's stroke, narrowed by a configured one."""
        return self._ec.effective_stroke or self._configured

    @property
    def max_speed_mm_s(self) -> float:
        """min(GUI cap, the EC's own maximum speed MAXV (item 7.4) once read): what the driver will accept."""
        return self._ec.speed_cap_mm_s

    @property
    def units(self) -> str:
        """The axis's position unit: ``"deg"`` on a rotary axis, else ``"mm"`` (from the limits read at connect;
        no bus traffic here)."""
        axis = self._ec.axis_limits
        return "mm" if axis is None else axis.units

    @property
    def max_push_pct(self) -> float:
        """min(the driver's push cap, the EC's MXPP (item 7.9) once read); 0 when this unit cannot push
        (SUPT PUSH = 0, MXPP = 0 or TCPC = 0: the driver refuses every push then)."""
        axis = self._ec.axis_limits
        if axis is not None and axis.push_available is False:
            return 0.0
        return self._ec.push_cap_pct

    @property
    def accel_cap_g(self) -> float:
        """min(the driver's accel cap, the EC's MAXA / MAXD (items 7.5 / 7.6) once read)."""
        return self._ec.accel_cap_g

    def identify(self) -> dict[str, Any]:
        info = self._ec.identify()
        limits = info.soft_limits_mm
        ec_stroke = self._ec.ec_stroke
        axis = self._ec.axis_limits
        return {
            "model": info.model,  # the 0x0600 model string when the EC has one
            "controller": info.controller,  # FC11 / CTCL: system, unit code + family name
            "actuator": info.actuator,  # AKID item 7.33, e.g. "gripper (AKID 4)"
            "unit_known": info.unit_known,  # True when the driver has a family name for the unit code (display only)
            "identity_source": info.identity_source,
            "axis_number": info.axis_number,  # AXNO 0x8004
            "user_name": info.user_name,  # 0x061A: the user-editable unit name (set in IA-OS)
            "short_model": info.short_model,  # old name of user_name
            "serial_number": info.serial_number,
            "firmware": info.firmware,
            "system_code": _hex(info.system_code),
            "unit_code": _hex(info.unit_code),
            "run_status": _hex(info.run_status),
            "slave": info.slave,
            "baud": self.baud if self.baud is not None else "simulated",
            "soft_limits_mm": list(limits) if limits else None,
            "ec_stroke_mm": list(ec_stroke) if ec_stroke else None,
            "ec_max_speed_mm_s": info.max_speed_mm_s,  # MAXV item 7.4 (0x0812 only as the fallback)
            "ec_max_accel_g": info.max_accel_g,  # MAXA item 7.5
            "ec_max_decel_g": info.max_decel_g,  # MAXD item 7.6
            "speed_cap_mm_s": self._ec.speed_cap_mm_s,
            "accel_cap_g": self._ec.accel_cap_g,
            "push_cap_pct": self._ec.push_cap_pct,
            "push_min_recommended_pct": None if axis is None else axis.min_push_pct,
            "index_categories": len(info.categories),
            "errors": "; ".join(info.errors) or "none",
        }

    def status(self) -> dict[str, Any]:
        st = self._ec.status()
        result = status_dict(st)
        if self._home_started is not None:
            note = self._follow_home(st)
            if note:
                result["notes"] = [note]
        return result

    def ec_limits(self) -> tuple[EcLimits | None, str]:
        """The EC's soft limits, nominal stroke and speed limit, or None with the reason. Raises on a timeout."""
        limits = self._ec.read_soft_limits()
        stroke = self._ec.ec_stroke
        speed = self._ec.read_speed_limit()
        if limits is None or stroke is None:
            return None, self._ec.soft_limits_note
        return EcLimits(limits, stroke, speed), self._ec.soft_limits_note

    def set_stroke(self, stroke: tuple[float, float] | None) -> None:
        self._configured = None if stroke is None else (float(stroke[0]), float(stroke[1]))
        self._ec.stroke = self._configured  # the driver checks every target against this too

    def table_row(self, n: int) -> dict[str, Any]:
        entry = self._ec.read_position_table(n)
        return {
            "position_mm": entry.position_mm,
            "speed_mm_s": entry.speed_mm_s,
            "accel_g": entry.accel_g,
            "decel_g": entry.decel_g,
            "band_mm": entry.band_mm,
            "push_power": entry.push_power,
            "control_flags": entry.control_flags,
            "checksum_ok": entry.checksum_ok,
        }

    def unit_info(self, sections: Iterable[str]) -> dict[str, Any]:
        """``Elecylinder.unit_info`` for some of its sections. Read-only: FC03 + FC11 only."""
        return self._ec.unit_info(sections=list(sections))

    def preemptible(self, gate: ReadGate) -> AbstractContextManager[None]:
        """Read-only calls of this thread inside the context send each FC03 / FC11 inside ``gate()``.

        Requests are at most ``GATED_READ_WORDS`` words and retried outside the bus lock, so the
        controller can take its bus lock per request and STOP never waits for more than one short
        request. The status poll, commands and STOP (other threads) are not affected.
        """
        if self._core_reads is not None:
            return self._core_reads.preemptible(gate)
        assert self._gated is not None
        return self._gated.gated(gate)

    def read_block(self, class_id: int, offset: int, count: int | None) -> dict[str, Any]:
        """Raw words of directory block ``class_id`` for the register browser. Read-only (FC03).

        Bounded by the block in the EC's own directory - for a block of uncertain size (mixed member
        sizes) its largest possible extent (``words_readable``; the words past ``block_words`` are
        "size uncertain") - and by ``BROWSE_MAX_WORDS`` per read; out-of-range requests raise a 400
        BackendError before anything is sent.
        """
        records = self._ec.categories or ec_info.read_directory(self._ec)[1]
        rec = next((r for r in records if r.category_id == class_id), None)
        if rec is None:
            raise BackendError(f"Class 0x{class_id:04X} is not in this EC's directory.", 404)
        words, exact, words_max = ec_info.block_extent(rec)
        readable = ec_info.readable_words(rec, records)
        if count is None:
            count = min(BROWSE_DEFAULT_WORDS, readable - offset)
        if not 0 <= offset < readable or not 1 <= count <= BROWSE_MAX_WORDS or offset + count > readable:
            size = (f"{words} registers" if readable == words
                    else f"{words} registers for certain, {readable} at most (size uncertain)")
            raise BackendError(
                f"Class 0x{class_id:04X} has {size} at 0x{rec.data_address:04X}: offset {offset} + "
                f"count {count} is outside it (at most {BROWSE_MAX_WORDS} words per read).", 400)
        data = ec_info.read_block(self._ec, class_id, offset, count, records=records)
        return {
            "class_id": class_id,
            "class": f"0x{class_id:04X}",
            "name": R.CATEGORY_NAMES.get(class_id, "?"),
            "block_address": rec.data_address,
            "block_words": words,
            "exact": exact,
            "words_max": words_max,
            "words_readable": readable,  # words at block_words.. are "size uncertain" when exact is False
            "member_size": rec.member_size,
            "members": rec.member_count,
            "entries": rec.list_count,
            "offset": offset,
            "count": len(data),
            "address": rec.data_address + offset,
            "words": list(data),
            "read_only": True,
        }

    # ------------------------------------------------------------------ commands

    def enable_control(self) -> None:
        # Refused by the driver while the axis moves (PMSL on mid-move stops it); raises EcNotReadyError
        # if PMSS does not follow (Monitor mode).
        self._ec.enable_modbus_control()

    def release_control(self) -> None:
        self._ec.disable_modbus_control()

    def servo(self, on: bool) -> None:
        self._ec.servo(on, wait=False)

    def home(self) -> None:
        try:
            self._ec.home(wait=False)
        except ec_errors.EcError:
            raise  # refused by a precondition check: nothing was written
        except BaseException:
            self._home_started, self._home_seen_start = time.monotonic(), False  # the HOME coil may be raised
            raise
        self._home_started, self._home_seen_start = time.monotonic(), False

    def move_to(self, position_mm: float, speed_mm_s: float, accel_g: float) -> None:
        self._ec.move_to(position_mm, speed_mm_s=speed_mm_s, accel_g=accel_g, wait=False)

    def jog(self, delta_mm: float, speed_mm_s: float, accel_g: float) -> None:
        self._ec.jog(delta_mm, speed_mm_s=speed_mm_s, accel_g=accel_g, wait=False)  # a relative move, never JOG coils

    def push(self, target_mm: float, band_mm: float, force_pct: float, direction: int,
             speed_mm_s: float, accel_g: float) -> None:
        self._ec.push(target_mm, band_mm, force_pct, direction, speed_mm_s=speed_mm_s, accel_g=accel_g, wait=False)

    def stop(self) -> dict[str, Any] | None:
        """Driver STOP coil sequence; blocks for the deceleration (at most ``STOP_HOLD_S``).

        Returns the last status the hold read (the settled standstill, or the still-moving status
        when the hold timed out), so the caller knows where STOP left the axis even when it could
        not read status itself.
        """
        try:
            st = self._ec.stop(wait=False, timeout=STOP_HOLD_S)
        finally:
            if self._home_started is not None:
                self._finish_home()
        return None if st is None else status_dict(st)

    def reset_alarm(self) -> None:
        self._ec.reset_alarm()

    @property
    def line_baud(self) -> int | None:
        """The port's current baud rate (the simulator's modelled line rate in a simulator session)."""
        return self._bus.baudrate

    def run_stored(self, n: int, allow_above_caps: bool) -> None:
        """Stored position ``n`` (0x9800 = n) at its stored speed; returns once the motion has started."""
        self._ec.move_to_position(n, wait=False, allow_above_caps=allow_above_caps)

    def write_param(
        self, no: int, value: Any, allow_home_params: bool, allow_unverified_model: bool = False
    ) -> dict[str, Any]:
        """Write + verify + commit (PARW) user parameter P``no``; it applies after a software reset."""
        return self._ec.write_user_parameter(
            no, value, commit=True, allow_home_params=allow_home_params, allow_unverified_model=allow_unverified_model
        ).to_dict()

    def software_reset(self) -> dict[str, Any]:
        """Software reset (servo OFF, 0x45 cmd 5) and link re-sync; the result carries the GUI's status dict."""
        result = self._ec.software_reset()
        if self.baud is not None and result.baudrate is not None:
            self.baud = result.baudrate
        data = result.to_dict()
        data["status"] = status_dict(result.status)
        return data

    def set_baudrate(self, rate: int, method: str) -> dict[str, Any]:
        """Broadcast baud change (0x45 cmd 4); the port follows in place. Volatile (38400 after power-up)."""
        result = self._ec.set_baudrate(rate, method=method)
        if self.baud is not None:
            self.baud = result.baudrate
        return result.to_dict()

    def close(self) -> None:
        """Release a HOME coil this session may have left raised and a STOP coil left ON, then close the port."""
        try:
            if self._home_started is not None:
                self._finish_home()
        finally:
            try:
                self._ec.close()  # releases a STOP coil whose OFF write failed earlier
            finally:
                self._bus.close()

    # ------------------------------------------------------------------ internals

    def _follow_home(self, st: Status) -> str | None:
        """Release the HOME coil once the home return has ended; returns a note for the event log."""
        assert self._home_started is not None
        self._home_seen_start = self._home_seen_start or st.homing or not st.homed or st.move_bit
        if self._home_seen_start and st.homed and st.in_position and not st.homing:
            self._finish_home()
            return None  # "Home complete" is logged by the controller from the HEND change
        if st.alarm or not st.servo_on:
            self._finish_home()
            return "Home return ended by an alarm / servo off; HOME coil released"
        if not self._home_seen_start and time.monotonic() - self._home_started >= HOME_START_S:
            self._finish_home()
            return f"Home return did not start within {HOME_START_S:g} s (HOME ignored?); HOME coil released"
        return None

    def _finish_home(self) -> None:
        # Cleared only after the write succeeds, so a failed release is retried (at the latest by close()).
        self._ec.finish_home()
        self._home_started = None


def _open_serial(port: str, baud: int, slave: int) -> ModbusRTU:
    """Open ``port`` once and find the baud rate the EC answers at, changing it in place."""
    try:
        bus = ModbusRTU(port, baudrate=baud)
    except OSError as exc:
        raise BackendError(
            f"Cannot open {port}: {exc}. Only one program can use the port: disconnect / close IA-OS "
            "(or any other tool using it) first.", 502) from exc
    try:
        bus.detect_baudrate(slave, BAUD_DETECT_ORDER)  # the selected rate first, then the usual ones
    except ec_errors.ModbusTimeout as exc:
        bus.close()
        raise BackendError(
            f"No answer from slave {slave} on {port} at any baud rate ({exc}); {ec_errors.NO_ANSWER_HINT}.",
            502) from exc
    except BaseException:
        bus.close()
        raise
    return bus


def _hex(value: Any) -> Any:
    return f"0x{value:02X}" if isinstance(value, int) and not isinstance(value, bool) else value
