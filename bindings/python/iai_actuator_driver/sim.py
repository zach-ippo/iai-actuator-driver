"""Byte-level ELECYLINDER simulator implementing the :class:`~iai_actuator_driver.modbus.Transport` protocol.

``SimulatedEC`` parses Modbus RTU request frames and answers exactly like a slave would
(FC03/05/06/10/11/17 and IAI 0x45/0x48), so the whole driver stack can be exercised without
hardware::

    from iai_actuator_driver.modbus import ModbusRTU
    from iai_actuator_driver.sim import SimulatedEC
    bus = ModbusRTU(SimulatedEC())

Three EC profiles (``profile=``) model the ECs tested on hardware:

* ``"slider"`` (default): EC-DS4H-150 in 3-position mode: 150 mm stroke, soft limits
  -0.30 / +150.30 mm, maximum speed 700.00 mm/s (MAXV item 7.4 = the 0x0812 default), FC11 unit 0x02 firmware 0x0013,
  model string MWB-EC-DSRR4-WA-MF, the 27-record index table, rows 0.00 / 150.00 / 75.00 mm.
* ``"gripper"``: EC-GRBP8M-20 in 2-position mode: 9.50 mm stroke, soft limits -0.30 / +9.80 mm,
  53.00 mm/s (MAXV / 0x0812; MAXA / MAXD 0.30 G), unit 0x0A firmware 0x0002, model string EC-GRBP8M-20-1-1-PN-WA-WL2, its
  25-record index table, rows 3.00 mm (open) and 4.10 mm + 5.40 mm push (grip), starting at 9.50 mm.
* ``"rod"``: EC-R6S-150-1-PN, 150 mm stroke, soft limits -0.30 / +150.30 mm, 800.00 mm/s (MAXV / 0x0812),
  unit 0x00 firmware 0x0002, its 22-record directory and parameter classes, 2-position DOPM layout,
  rows 0.00 mm (240 mm/s, accel 0.10 / decel 0.47 G) and a push row 30.00 mm + 120 mm away from home
  at PPOW 26 (~10 %), starting at 0.00 mm. **Incremental** (no WA option): a software reset clears
  HEND. FC11 answers only after ``slave_id_delay_s`` (0.3 s; hardware: 0.2-0.4 s).

Each is seeded with the values read live (PNOW, DSS1 0x3098, DSSE, position rows 0-3, the 0x9900
block, the FC11 answer, the alarm history, the identification block with a placeholder serial,
the soft limits, the 0x0800 parameters). Motion is a real-time trapezoidal model (speed VCMD,
accel = decel ACMD). Behaviour follows what the hardware showed:

* DSS1 b0 = commanded target reached; b7 = 1 only when stopped; DSSE b4 set while moving.
* The DSSE MOVE bit clears during the final deceleration (``move_bit_clears_in_decel``).
* PEND and MEND rise as soon as the axis is inside the INP band of a normal move, while it is still
  moving (``early_in_position``; on the gripper DSS1 read 0x3099 at 3.04 mm and the next sample 3.00 mm,
  and an iai-check move "reached" 7.91 mm, then read 7.97 mm), and the last ``settle_tail_mm`` of a
  normal move is covered at no more than ``settle_speed_mm_s`` (the servo's settling tail; values
  chosen to match those readings, not measured). A driver must therefore also wait for PNOW to
  stop changing.
* STOP (0x042C) latches until written OFF (``stop_latches``; whether the EC does is unverified, so
  the latching model is the worst case): while ON, new move / home commands are ignored.
* PMSL / PMSS (VERIFIED on the EC-DS4H slider): 0x9900 moves run and STOP stops them with PMSS = 0;
  switching PMSL ON mid-move stops the move (PEND = 1, MEND = 0). **0x9800 position-number moves are
  silently ignored while PMSS = 0** (write accepted, no motion: VERIFIED on the EC-GRBP8M gripper); with
  PMSS = 1 they run row n at its stored speed / acceleration (VERIFIED on the EC-DS4H slider; a push row
  runs its push). Other coils, DRG writes and IAI commands are ignored / rejected while PMSS = 0
  (unverified on the EC; ME0162 behaviour).
* Parameters (classes 7 / 8 / 9, e.g. the user parameters P1-P12): FC06 / FC10 writes land in RAM
  only: they read back at once and are lost by :meth:`SimulatedEC.power_cycle` (VERIFIED on the
  EC-GRBP8M gripper: P2 reverted after a 24 V power cycle). The simulator does not act on a RAM value
  (soft limits, LS range and home direction stay as committed): the EC applies a change only after a
  reset.
* :meth:`SimulatedEC.power_cycle` (as seen on the EC-DS4H slider and the EC-GRBP8M gripper): PMSS = 0, safety
  speed off, coils released, servo on, homed (WA absolute; an incremental unit comes back without HEND), position
  kept, position table kept (non-volatile), parameter RAM reverted, alarm cleared, a 0x0FFF power-up marker in the
  alarm history, uptime restarted, the line at 38400 baud.
* STP (0x040A) is a level input: it holds a move, and a move commanded while it is ON waits.
* Home: HEND 0, GHMS 1, ~30 mm/s to the mechanical end (-2.00 mm), then a creep back to 0.00
  with MOVE 0 but GHMS still 1, then HEND 1 + PEND 1.
* Push (CTLF bit 1, bit 2 = away from home): approach PCMD, then push over INP at
  min(VCMD, 20 mm/s); a workpiece at ``obstacle_mm`` gives contact (PEND, DSSE b0, b10 held),
  none gives PSFL at the band end. A start inside / past the push zone skips the approach.
  ``push_blip_mm`` reproduces what the EC-GRBP8M gripper showed: the full contact fingerprint (DSS1
  PEND + MEND, DSSE 0x37C3 with MOVE 0) for a moment during the push stroke, which then goes on to a miss.
* Soft limits at 0x0700 (+stroke + 0.30 / -0.30 mm); targets outside raise alarm 0x0D9.
* Parameter commit (coil 0x0428 PARW, ON-WIRE: USB capture of IA-OS): DRGE b7 reads 1 for
  ``parw_busy_s`` real seconds (capture: DRGE 0x0180 at +55 ms, 0x0100 at +235 ms; default 0.1 s), then clears
  itself and the parameter RAM as it was at the coil is in the simulated non-volatile store.
  ``parw_dsse_busy=True`` also drops DSSE b7 to 0 meanwhile (seen on a PCON controller during its commit; on
  the EC the capture's one sample in the window still read 1, so off by default). A software reset or power
  cycle inside the window loses the commit (worst case; unknown on the EC). ``parw_busy_s=0`` commits at once
  and DRGE never shows b7. Ignored while PMSS = 0 like other coils (unverified on the EC): DRGE b7 then stays 0.
* IAI 0x45 commands (forms VERIFIED on the EC-R6S rod): servo cmd 9; home cmd 0x0A with op1 = the home pattern
  ORG (op1 = 0 rejected 0x11 "1st operand error", servo off 0x23, homing 0x24); alarm reset cmd 5 op1 0; action
  mode cmd 0x0C (op1 PC = PMSS, op2 safety speed; accepted with PMSS = 0 too). Other commands with PMSS = 0 are
  rejected 0x3A; unknown commands 0x21. Alarms follow the levels of :mod:`iai_actuator_driver.alarms`: a message-level
  code (0x011-0x07F) sets ALML and the axis keeps running; any other sets ALMH and switches the servo off.
* Class 1 support words (0x0100) are served on every profile (``_SUPPORT_WORDS``: the gripper's read live, the
  rod's partly assumed); ``collision_detection=True`` sets MVPT b13 CDAS.
* Software reset (0x45 cmd 5 op1 1, after servo OFF with 0x45 cmd 9): the reply ``01 45 00 00`` is sent,
  then the EC is silent for ``reset_s`` simulated seconds and comes back like after a power-up, except
  that the uptime 0x9010 keeps counting: parameter RAM reloaded from the non-volatile store (and applied:
  soft limits, LS range), PMSS 0 (as on the EC-R6S rod; the gripper profile keeps PMSS 1, as the EC-GRBP8M
  gripper and the PCON-CB did), safety speed off, coils released, servo ON, alarm cleared; an
  incremental unit (``absolute=False``) loses HEND; an absolute unit whose committed home parameters
  (P3 9.16 / P4 9.21) changed since its last home raises 0x0EE "ABS error (2)" (servo off, HEND 0;
  an alarm reset switches the servo back on, a home return clears it, as on the EC-GRBP8M gripper). With
  the servo ON the command raises alarm 0x090 instead (as the SFRS coil 0x0420 did on the gripper; for
  0x45 cmd 5 that is an assumption). The baud rate after a reset is ``reset_baudrate`` (None: unchanged;
  the hardware's is unknown).
* Baud rate (``baudrate=``, off by default): the simulator then models the line and answers only when
  the PC side (``set_baudrate``) matches the EC's own rate; a line BREAK of >= 150 ms
  (``send_break``) drops the EC to 9600; the broadcast ``00 45 00 04 <code> 00 05`` switches it (codes
  of ``modbus.IAI_BAUD_CODES``; never answered) except the codes in ``ignored_baud_codes`` (default
  0x06 "38400": the EC-R6S rod ignored it sent direct; the simulator also ignores it after a break,
  untested and worst case, and on every profile); a power cycle brings it back to ``boot_baudrate``
  (38400, VERIFIED on the EC-DS4H slider).
* ZONS (0x9013): b0 / b1 zones (both span the soft limits), b12 / b13 / b14 = LS0 / LS1 / LS2 from
  the position alone, moving or not: lit while PNOW is within ``ls_range_mm`` (EC parameter 2,
  0.10 mm on both units) of row 0 / 1 / 2. A push row's marker sits at its push end (PCMD +/- INP),
  not over the whole push zone. Row 2 lights LS2 on the 2-position gripper too. This matches 379 of
  the 380 ZONS samples logged on hardware: gripper 0x0003 at 5.99 / 9.00 mm and during the whole push
  of row 1, 0x2003 only at 9.50 mm, 0x4003 at 2.52 mm and 0x1003 at 2.91 mm while moving; slider
  0x2003 / 0x1003 at 150.10 / -0.10 mm.
* DOPM (0x9004), ``dopm_layout``: ``"3pos"`` b0 LS0, b1 LS1, b2 LS2, b3 *ALM (slider);
  ``"2pos"`` b0 LS0, b1 LS1, b2 *ALM (gripper). *ALM = 1 means no alarm. The LS bits follow the
  same position rule, except that a push row's bit stays off after a missed push (gripper after the
  push of row 1 missed: DOPM 0x0004 at 9.50 mm with PSFL, while ZONS read 0x2003).
* PIO (``pio_move``): a test hook for a move a PLC starts over PIO while PMSS = 0. With
  ``hold_start=True`` the PLC keeps its start input on and the EC restarts the move as soon as a
  Modbus STOP is released. This is a WORST-CASE model: a Modbus STOP on a PIO-commanded move is untested.
* Alarm history at 0x0500 + 6k; new alarms are inserted at k = 0.
* Read-only information (:mod:`iai_actuator_driver.sim_fixtures`): each profile serves its unit's real
  directory, the item-ID maps, the parameter classes 3 / 4 / 7 / 8 / 9 / 0C, the maintenance block
  0x8400 (TLMC, item 1, counts the moves the simulator starts), the identification block (placeholder
  serial), its alarm history and position rows 256-267 (the % page IA-OS shows). Class 3 item 10 follows
  the present alarm, the status image 0x0200 follows the monitor block. Reads are bounded by the
  directory: inside a block without recorded data the simulator answers 0, outside every block (and
  outside the ID maps) it answers exception 02. That is stricter than the EC, which answers 0 everywhere,
  so tests catch reads that stray past a block.

Further profiles (``profile=``):

* ``"pcon"``: a PCON-CB controller with an RCP6-WSA10R slider (FC11 ``AE FF 44 00 00``; the whole unit read
  live): 150 mm, soft limits -0.30 / +150.30 mm, MAXV 840.00 mm/s, MAXA / MAXD 1.00 G, its 22-record directory,
  768 position rows (rows 0-7 as read, the rest the unit's blank row), 64 positions (MAXP 3.2). It starts with the
  message-level alarm 0x069 "RTC oscillation stop", which comes back after every alarm reset and restart while the
  clock is unset (as on the unit; the axis still runs). Its software reset (0x45 cmd 5 op1 1) is silent for
  ``reset_s`` and reloads the parameters from non-volatile memory, but keeps PMSS = 1, the baud rate and the uptime
  counter (VERIFIED on the PCON-CB: a driver cannot confirm it by PMSS there).
* ``"rotary"``: an EC on the rod's blocks with AKID (item 7.33) = 1, i.e. a rotary axis whose positions are degrees
  (0..360, soft limits -0.30 / +360.30); a simulator profile.
* ``"unknown"``: an unknown IAI unit (FC11 system 0x99, unit 0x7E, AKID 42, blank model string) on the rod's blocks,
  to show the driver drives any unit the same way.

On every profile the command profile 0x0F00 (20 words: PCMD, VCMD, ACMD, DCMD, INP, SPOW, DPOW, PPOW, CTLF, LPOW,
CTF2, ZNMP, ZNLP) is writable (FC10 / FC17; read back as written, no motion) and runs with the PCRF coil 0x042E
(self-clearing; separate acceleration and deceleration; CTLF b1 / b2 push, b3 incremental; a new profile + PCRF
retargets; STOP stops it), VERIFIED on the EC-GRBP8M gripper and the PCON-CB. 0x45 cmd 6 runs stored row op1 at op2 /
255 of its stored speed (VERIFIED on the EC-GRBP8M gripper and the PCON-CB); an EC profile rejects a row its position
mode does not run (row 2 in 2-position mode) with 0x11 "1st operand error" and nothing moves (VERIFIED on the
EC-GRBP8M gripper in 2-position mode). 0x9800 with such a row is unverified: the simulator still runs it.

Modelled from the manuals (DOC ME0162-12A unless noted; UNVERIFIED on an EC):

* The monitor serves POSS 0x9014 (the completed stored row: set when a 0x9800 / CSTR / 0x45 cmd 6 row move ends, a
  push row included, 0 after a 0x9900 move; p.4-24) and SSSE 0x9015 (the profile's word plus b11 while a cold-start
  alarm is present; ``rtc=True`` sets b8). STAT carries b0 MPOW, b1 SON, b2 SV, b3 HEND, b4 RMDS, b15, b16 AEEP (while
  a PARW commit runs) and b17 ASOF (p.4-21).
* :meth:`SimulatedEC.set_inputs` (test hook): ``emg`` (DSS1 EMGS + DSSE EMGP, servo off), ``drive_cut`` (STAT MPOW 0 +
  DSSE MPUV, servo off) and ``enable`` (DSS2 ENBS; released = servo off). While any blocks, servo ON is ignored.
* ``auto_servo_off_s``: after a move or push ends, the servo goes off by itself after this many simulated seconds of
  standstill and STAT b17 ASOF is set (PEND drops with it); SON, or a new move command (the CON behaviour, DOC PCON
  p.5-5), switches it back on.
* ``cold_boot=True``: the unit starts (and comes back from :meth:`power_cycle`) with the servo off and DSSE PSNS 0; the
  first servo ON takes the excitation-detection time, on a CON unit 0.006 + (T1 + No.29) x 10 + 0.020 s from its item
  9.1 (No.29 b0-15 ms, No.30 b17-18; T1 0.16 / 0.22 s; DOC PCON p.4-24), on an EC ``ec_first_son_s``.
* DRG1 / STP: an ALRS edge while STP is held cancels the paused move: the axis stays where it stopped and releasing
  the pause does not resume the old target (DOC ME0384 p.6-76; ME0162-12A p.4-11, 5-85).
* Push speed: min(VCMD, the unit's PSHV item 8.15, MXPH 7.8): 5 mm/s on the gripper, 30 on the slider, 20 on the rod.
* CTLF b6-b7 (acceleration pattern) is passed through: :attr:`SimulatedEC.accel_pattern` names the last one started;
  S-motion / filter moves run with half the commanded acceleration (a smoothed profile; a simulator choice).
* Teach coils MOD 0x0414 / TEAC 0x0415 (p.5-97): with MOD on, TEAC writes PNOW into row POSR (0x0D03) and sets DSS2
  b10 PWRT until TEAC is released (alarm 0x093 when TEAC comes before homing; DSS2 b11 MODS follows MOD).
* ``function_codes``: only these function codes are answered (others: exception 01), e.g. ``(0x03, 0x06, 0x10)`` for
  a unit behind an RCON gateway (DOC ME0413 p.3-1); ``broadcast`` (default: True unless ``function_codes`` is given)
  whether frames to address 0 are acted on (RCON: not, ME0413 p.6-1).

Fault injection for power-loss / link-loss recovery (:mod:`iai_actuator_driver.recovery`):

* :meth:`SimulatedEC.power_off` / :meth:`SimulatedEC.power_on` (or :meth:`SimulatedEC.power_cycle`): the unit is silent
  while off, then powers up as above (baud rate back to ``boot_baudrate``, PMSS 0, uptime restarted, HEND lost on an
  incremental unit, the 0x0FFF power-up record; a PCON raises 0x069 again).
* :meth:`SimulatedEC.unplug` / :meth:`SimulatedEC.replug` (also on a :class:`~iai_actuator_driver.sim_line.SimLine`): the PC's
  port vanishes (``present`` False) and the open handle raises OSError on every call, like a pulled USB adapter
  (VERIFIED: the COM port disappears); after ``replug`` the port is listed again and :meth:`SimulatedEC.reopen` opens
  it. The unit keeps its power and state.
* ``serial_number=``: a different controller serial, to model a replaced unit.

What it is NOT: proof of EC behaviour. Where the hardware tests and manuals are silent the
simulator makes a simple choice, noted in comments. Passing tests against it shows the driver is
self-consistent with what was observed, not that every EC behaves this way.

A :class:`SimulatedEC` here is a simulated unit of the core (``iad_sim``, C++ ``HookedSimUnit``): the motion model, the
registers, the protocol answers and the hooks run in the core, and the profiles, behaviour and evidence levels are the
ones documented above and in ``core/include/iai_actuator_driver/sim.hpp``. This
module converts: the constructor options become the core's options JSON, the attributes read its state JSON, the test
hooks are its controls, the transport calls go to its transport handle, and an :meth:`SimulatedEC.on_request` override
becomes its request hook (:meth:`SimulatedEC.handle` is the core's own answer).

A native bus reaches a ``SimulatedEC`` straight on the unit's port in the core (``iad_sim_port``, offered by
``_iad_transport``), so no frame passes through Python; a subclass or an object that replaces one of the transport
methods (``write`` / ``read`` ...) is driven through them instead (a callback transport), as any transport. Either way a
bus that is collected leaves the unit's port open, a closed or unplugged port raises ``TransportError`` (from an
``OSError``), :attr:`SimulatedEC.baud_history` sees every rate change (the core keeps it) and an exception raised in
:meth:`SimulatedEC.on_request` fails the request: an ``OSError`` becomes the cause of the bus's
``TransportError``, anything else reaches the caller as it is.

Read-only data with an exact register form is served through the core's word overrides: ``model_string`` (the
identification block), ``collision_detection`` (MVPT b13), ``row_words`` / ``monitor_record_words`` / ``missing_items``
(the directory and ID maps) and an index table read from ``bkec_path``. A ``clock`` other than the library's becomes
the unit's own clock (``iad_sim_set_clock``); the other options and the controls (setting ``homed`` / ``servo_on`` /
``pmsl`` / ``safety_speed``, changing an option later, ``set_inputs``, ``set_parameter_word``) are passed to the core
by name and raise ``NotImplementedError`` on a library whose core refuses them. Never opens a port.
"""

from __future__ import annotations

import collections.abc
import ctypes
import os
import struct
import time
import weakref
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from pathlib import Path
from typing import Any

from . import errors as _errors
from . import native as _n
from . import registers as R
from ._protocol import IAI_BAUD_CODES, build_frame, check_crc  # noqa: F401 (re-exported)
from ._shared.sim import (  # the shared profile data and constants
    ALARM_ABS_ERROR,
    ALARM_BAD_BAND,
    ALARM_BAD_DECEL,
    ALARM_BAD_SPEED,
    ALARM_MOVE_WHILE_HOMING,
    ALARM_NOT_HOMED_MOVE,
    ALARM_POSITION_NO,
    ALARM_RESET_SERVO_ON,
    ALARM_SERVO_OFF_MOVE,
    ALARM_SOFT_LIMIT,
    ALARM_TEACH_NOT_HOMED,
    DOPM_LAYOUTS,
    FAULT_KINDS,
    HOME_PARAMETER_ITEMS,
    PARAMETER_RAM_CLASSES,
    PROFILES,
    REJECT_COMMAND_CODE,
    REJECT_HOMING,
    REJECT_MONITOR_MODE,
    REJECT_OPERAND1,
    REJECT_SERVO_OFF,
    load_bkec_registers,
)
from .alarms import LEVEL_COLD_START, LEVEL_MESSAGE, alarm_level  # noqa: F401 (re-exported)

__all__ = ["SimulatedEC", "load_bkec_registers", "DEFAULT_BKEC_PATH", "PROFILES"]

DEFAULT_BKEC_PATH = Path(__file__).resolve().parents[4] / "backups" / "Axis No. 0 (2026-10-27 baseline).bkec"

_FIRST_SON_EC_S = 0.3  # the default ec_first_son_s
_DOPM_NAMES = {"3pos": "three_position", "2pos": "two_position"}
# attribute names -> the core's option keys, where they differ
_CORE_OPTION_NAMES = {"dopm_layout": "dopm", "serve_soft_limits": "soft_limits", "boot_baudrate": "boot_baud_rate",
                      "reset_baudrate": "reset_baud_rate"}  # fmt: skip
_LIBRARY_CLOCKS = (time.perf_counter, time.monotonic)
_NOT_IN_CORE = "is not in the core simulator of this library yet"


class SimException(Exception):
    """Raised inside a :meth:`SimulatedEC.on_request` override to answer a Modbus exception ``code`` to the request
    (0x01 illegal function, 0x02 illegal address, 0x03 illegal value)."""

    def __init__(self, code: int) -> None:
        super().__init__(code)
        self.code = code


_SimException = SimException  # the earlier private name


def _refused(exc: BaseException, what: str) -> NotImplementedError:
    return NotImplementedError(f"SimulatedEC: {what} {_NOT_IN_CORE} ({exc})")


def _unknown_to_core(exc: ValueError) -> bool:
    text = str(exc)
    return "unknown simulator option" in text or "unknown simulator command" in text


class _SimLog(collections.abc.Sequence):  # type: ignore[type-arg]
    """A list-like view of one of the unit's logs in the core (read at each use), plus entries a test appended."""

    def __init__(self, read: Callable[[], list[Any]], clear: Callable[[], None]) -> None:
        self._read = read
        self._clear = clear
        self._extra: list[Any] = []

    def _items(self) -> list[Any]:
        return [*self._read(), *self._extra]

    def __len__(self) -> int:
        return len(self._items())

    def __getitem__(self, index: Any) -> Any:
        return self._items()[index]

    def __iter__(self) -> Iterator[Any]:
        return iter(self._items())

    def __contains__(self, item: object) -> bool:
        return item in self._items()

    def __eq__(self, other: object) -> bool:
        return list(self) == list(other) if isinstance(other, Iterable) else NotImplemented

    def __repr__(self) -> str:
        return repr(self._items())

    def append(self, item: Any) -> None:
        self._extra.append(item)

    def clear(self) -> None:
        self._clear()
        self._extra.clear()


def _option(name: str, doc: str) -> property:
    """A constructor option as an attribute: read as given, changed in the core (``set_option``)."""

    def get(self: SimulatedEC) -> Any:
        return self._options[name]

    def set_(self: SimulatedEC, value: Any) -> None:
        self._set_option(name, value)

    return property(get, set_, doc=doc)


def _state_flag(key: str, doc: str) -> property:
    """A live state flag of the unit, settable as a test hook (``set_state``)."""

    def get(self: SimulatedEC) -> bool:
        return bool(self._state()[key])

    def set_(self: SimulatedEC, value: bool) -> None:
        self._control("set_state", f"setting {key}", **{key: bool(value)})

    return property(get, set_, doc=doc)


def _state_value(key: str, doc: str, convert: Callable[[Any], Any] = lambda v: v) -> property:
    def get(self: SimulatedEC) -> Any:
        return convert(self._state()[key])

    return property(get, doc=doc)


class SimulatedEC:
    """Simulated EC on an RS-485 line, run by the native core (the module's documentation describes the
    profiles and the behaviour). Overriding :meth:`on_request` in a subclass changes the answer to chosen
    requests; :meth:`handle` is the simulator's own answer."""

    def __init__(
        self,
        slave: int = 1,
        *,
        profile: str = "slider",
        stroke_mm: float | None = None,
        position_mm: float | None = None,
        time_scale: float = 1.0,
        clock: Callable[[], float] = time.perf_counter,
        bkec_path: str | os.PathLike[str] | None = None,
        max_speed_mm_s: float | None = None,
        max_accel_g: float = 1.0,
        system_code: int | None = None,
        unit_code: int | None = None,
        firmware: tuple[int, int] | None = None,
        dopm_layout: str | None = None,
        pmsl: bool = True,
        recompute_position_checksum: bool = True,
        drop_reply_after_retarget: bool = False,
        stop_latches: bool = True,
        move_bit_clears_in_decel: bool = True,
        safety_speed: bool = False,
        soft_limits: bool = True,
        ls_range_mm: float = R.LS_RANGE_MM,
        early_in_position: bool = True,
        settle_tail_mm: float = 0.05,
        settle_speed_mm_s: float = 0.5,
        push_blip_mm: float | None = None,
        push_blip_len_mm: float = 0.10,
        absolute: bool | None = None,
        baudrate: int | None = None,
        ec_baudrate: int | None = None,
        boot_baudrate: int = 38400,
        reset_baudrate: int | None = None,
        reset_s: float = 1.0,
        slave_id_delay_s: float | None = None,
        parw_busy_s: float = 0.1,
        parw_dsse_busy: bool = False,
        ignored_baud_codes: Iterable[int] = (0x06,),
        collision_detection: bool = False,
        auto_servo_off_s: float | None = None,
        cold_boot: bool = False,
        ec_first_son_s: float = _FIRST_SON_EC_S,
        rtc: bool = False,
        function_codes: Iterable[int] | None = None,
        broadcast: bool | None = None,
        serial_number: str | None = None,
        model_string: str | None = None,
        port_name: str | None = None,
        step_s: float | None = None,
        word_overrides: Mapping[int, int] | None = None,
        unreadable: Iterable[tuple[int, int]] = (),
        parw_latches: bool = True,
        stale_status_polls: int = 0,
        row_words: int | None = None,
        monitor_record_words: int | None = None,
        missing_items: Iterable[tuple[int, int]] = (),
    ) -> None:
        if time_scale <= 0:
            raise ValueError("time_scale must be > 0")
        if step_s is not None and step_s < 0:
            raise ValueError("step_s must be >= 0")
        if profile not in PROFILES:
            raise ValueError(f"profile must be one of {sorted(PROFILES)}")
        defaults = PROFILES[profile]
        dopm_layout = str(defaults["dopm_layout"] if dopm_layout is None else dopm_layout)
        if dopm_layout not in DOPM_LAYOUTS:
            raise ValueError(f"dopm_layout must be one of {DOPM_LAYOUTS}")
        function_codes = None if function_codes is None else frozenset(int(f) for f in function_codes)
        ignored = frozenset(int(c) for c in ignored_baud_codes)
        delay = defaults["slave_id_delay_s"] if slave_id_delay_s is None else slave_id_delay_s

        # index table: a backup file wins over the embedded table
        path = Path(bkec_path) if bkec_path is not None else DEFAULT_BKEC_PATH
        index: dict[int, int] = {}
        if bkec_path is None and isinstance(defaults["index_table"], str):
            self.index_source = f"embedded:{profile}"
        elif path.is_file():
            index = {a: v for a, v in load_bkec_registers(path).items() if a < 0x0100}
            self.index_source = f"bkec:{path}"
        elif bkec_path is not None:
            raise FileNotFoundError(path)
        else:
            self.index_source = "embedded"

        core: dict[str, Any] = {
            "profile": profile, "slave": int(slave), "time_scale": float(time_scale), "stroke_mm": stroke_mm,
            "position_mm": position_mm, "max_speed_mm_s": max_speed_mm_s, "max_accel_g": float(max_accel_g),
            "system_code": system_code, "unit_code": unit_code,
            "firmware": None if firmware is None else [int(firmware[0]), int(firmware[1])],
            "dopm": _DOPM_NAMES[dopm_layout], "pmsl": bool(pmsl),
            "recompute_position_checksum": bool(recompute_position_checksum),
            "drop_reply_after_retarget": bool(drop_reply_after_retarget), "stop_latches": bool(stop_latches),
            "move_bit_clears_in_decel": bool(move_bit_clears_in_decel), "safety_speed": bool(safety_speed),
            "soft_limits": bool(soft_limits), "ls_range_mm": float(ls_range_mm),
            "early_in_position": bool(early_in_position), "settle_tail_mm": float(settle_tail_mm),
            "settle_speed_mm_s": float(settle_speed_mm_s), "push_blip_mm": push_blip_mm,
            "push_blip_len_mm": float(push_blip_len_mm), "absolute": absolute, "baud_rate": baudrate,
            "unit_baud_rate": (ec_baudrate or baudrate) if baudrate is not None else None,
            "boot_baud_rate": int(boot_baudrate), "reset_baud_rate": reset_baudrate, "reset_s": float(reset_s),
            "slave_id_delay_s": float(delay), "parw_busy_s": float(parw_busy_s),  # type: ignore[arg-type]
            "parw_dsse_busy": bool(parw_dsse_busy), "ignored_baud_codes": sorted(ignored),
            "serial_number": serial_number, "unreadable": list(unreadable), "port_name": port_name,
            "word_overrides": None if word_overrides is None else {a: v & 0xFFFF for a, v in word_overrides.items()},
            "stale_status_polls": int(stale_status_polls), "step_s": step_s,
        }  # fmt: skip
        # behaviour the core simulator models under these names (refused while it does not)
        if auto_servo_off_s is not None:
            core["auto_servo_off_s"] = float(auto_servo_off_s)
        if cold_boot:
            core["cold_boot"] = True
            core["ec_first_son_s"] = float(ec_first_son_s)
        if rtc:
            core["rtc"] = True
        if function_codes is not None:
            core["function_codes"] = sorted(function_codes)
        if broadcast is not None:
            core["broadcast"] = bool(broadcast)
        if not parw_latches:
            core["parw_latches"] = False
        try:
            self._unit = _n.NativeSim.open(**core)
        except ValueError as exc:
            if _unknown_to_core(exc):
                raise _refused(exc, "an option") from exc
            raise
        self._port = self._unit.transport()
        if step_s is None and not _library_clock(clock):
            self._use_clock(clock)

        self._options: dict[str, Any] = {
            "max_accel_g": float(max_accel_g), "firmware": defaults["firmware"] if firmware is None else firmware,
            "dopm_layout": dopm_layout, "recompute_position_checksum": recompute_position_checksum,
            "drop_reply_after_retarget": drop_reply_after_retarget, "stop_latches": stop_latches,
            "move_bit_clears_in_decel": move_bit_clears_in_decel, "serve_soft_limits": soft_limits,
            "ls_range_mm": ls_range_mm, "early_in_position": early_in_position, "settle_tail_mm": settle_tail_mm,
            "settle_speed_mm_s": settle_speed_mm_s, "push_blip_mm": push_blip_mm,
            "push_blip_len_mm": push_blip_len_mm, "boot_baudrate": boot_baudrate, "reset_baudrate": reset_baudrate,
            "reset_s": reset_s, "slave_id_delay_s": float(delay),  # type: ignore[arg-type]
            "parw_busy_s": float(parw_busy_s),
            "parw_dsse_busy": parw_dsse_busy, "ignored_baud_codes": ignored, "auto_servo_off_s": auto_servo_off_s,
            "cold_boot": cold_boot, "ec_first_son_s": ec_first_son_s, "function_codes": function_codes,
            "broadcast": (function_codes is None) if broadcast is None else bool(broadcast),
            "parw_latches": parw_latches,
        }  # fmt: skip
        self.con_reset = bool(defaults["con_reset"])
        self.reset_keeps_pmss = self.con_reset or bool(defaults.get("reset_keeps_pmss", False))
        self.rtc_unset = bool(defaults["rtc_alarm"])
        self.port = port_name
        self._baud_cleared = 0  # entries of the core's history before the last baud_history.clear()
        self.baud_history = _SimLog(self._baud_history, self._forget_baud_history)  # the PC side's rates, in order
        fixed = self._state()
        self.profile: str = fixed["profile"]
        self.slave: int = int(fixed["slave"])
        self.stroke_mm: float = float(fixed["stroke_mm"])
        self.time_scale: float = float(fixed["time_scale"])
        self.max_speed_mm_s: float = float(fixed["max_speed_mm_s"])
        self.system_code: int = int(fixed["system_code"])
        self.unit_code: int = int(fixed["unit_code"])
        self.absolute: bool = bool(fixed["absolute"])
        self._position_count = int(fixed["position_count"])
        self.events = _SimLog(self._unit.events, lambda: self._unit.control("clear_events"))
        self.requests = _SimLog(lambda: [frame for _, frame in self._unit.requests()],
                                lambda: self._unit.control("clear_requests"))  # fmt: skip

        # read-only data served through word overrides (exact for every read)
        for address, value in index.items():
            if self._served(address) != value:
                self._unit.set_word_override(address, value)
        for text, (offset, count) in ((model_string, R.IDENT_MODEL),):
            if text is not None:
                raw = text.encode("ascii")[: 2 * count].ljust(2 * count, b" ")
                for i, word in enumerate(struct.unpack(f">{count}H", raw)):
                    self._unit.set_word_override(R.IDENTIFICATION_BASE + offset + i, word)
        if collision_detection:
            self._unit.set_word_override(0x0103, (self._served(0x0103) or 0) | R.MVPT_CDAS)
        self._reshape_directory(row_words, monitor_record_words, missing_items)
        if type(self).on_request is not SimulatedEC.on_request:
            self._unit.set_request_hook(_request_hook(self))

    # ================================================================== core plumbing

    def _state(self) -> dict[str, Any]:
        return self._unit.state()

    def _control(self, command: str, what: str, **arguments: Any) -> dict[str, Any]:
        try:
            return self._unit.control(command, **arguments)
        except ValueError as exc:
            if _unknown_to_core(exc):
                raise _refused(exc, what) from exc
            raise

    def _set_option(self, name: str, value: Any) -> None:
        core = value
        if isinstance(value, (frozenset, set, tuple)):
            core = sorted(value) if not isinstance(value, tuple) else list(value)
        if name == "dopm_layout":
            if value not in DOPM_LAYOUTS:
                raise ValueError(f"dopm_layout must be one of {DOPM_LAYOUTS}")
            core = _DOPM_NAMES[value]
        self._control("set_option", f"changing {name}", name=_CORE_OPTION_NAMES.get(name, name), value=core)
        self._options[name] = value  # read back as given

    def _use_clock(self, clock: Callable[[], float]) -> None:
        lib = _n.load()
        if not lib.has("iad_sim_set_clock"):
            raise NotImplementedError(
                "SimulatedEC(clock=...): a clock of the simulated unit's own needs iad_sim_set_clock, which this "
                "native library does not export; use step_s or testing.virtual_clock()"
            )
        _n._check(lib, "iad_sim_set_clock", self._unit._h(), _n._c_py_clock_now, _n._keep_callback((clock, None)))

    def _baud_history(self) -> list[int]:
        return [int(rate) for rate in self._state()["baud_history"][self._baud_cleared :]]

    def _forget_baud_history(self) -> None:
        self._baud_cleared = len(self._state()["baud_history"])

    @property
    def _iad_transport(self) -> Callable[[], tuple[int, bool]] | None:
        """How a native bus reaches this unit: straight on its port in the core (:meth:`_bus_port`); ``None`` (the bus
        then drives this object's methods) when its class or the object itself replaces one of them."""
        return None if _replaces_transport(self, SimulatedEC) or not _n.load().has("iad_sim_port") else self._bus_port

    def _bus_port(self) -> tuple[int, bool]:
        """(a new transport handle on the unit's port, True: the caller releases it once its bus has it)."""
        out = ctypes.c_void_p()
        _n._check(_n.load(), "iad_sim_port", self._unit._h(), ctypes.byref(out))
        return int(out.value or 0), True

    def _served(self, address: int) -> int | None:
        try:
            return self._unit.register_value(address)
        except ValueError:
            return None

    def _reshape_directory(
        self, row_words: int | None, monitor_words: int | None, missing_items: Iterable[tuple[int, int]]
    ) -> None:
        """``row_words``, ``monitor_record_words`` and ``missing_items`` as word overrides of the directory records
        and the ID maps (what the driver reads)."""
        missing = {(int(c), int(i)) for c, i in missing_items}
        if row_words is None and monitor_words is None and not missing:
            return
        for k in range(self._served(2) or 0):
            base = 8 + 8 * k
            ci, dq, ia, ms = (self._served(base + j) or 0 for j in (0, 2, 5, 7))
            if ci == R.CATEGORY_POSITION_TABLE and row_words is not None:
                self._unit.set_word_override(base + 2, row_words // (ms or 1))
            if ci == R.CATEGORY_MONITOR and monitor_words is not None:
                self._unit.set_word_override(base + 2, monitor_words // (ms or 1))
            if ia and missing:
                for j in range(dq):
                    if (ci, self._served(ia + 2 * j)) in missing:
                        self._unit.set_word_override(ia + 2 * j, 0xFFFF)  # no item id: the item is not served

    # ================================================================== Transport protocol

    def write(self, data: bytes) -> None:
        try:
            self._port.write(bytes(data))
        except _errors.TransportError as exc:
            raise OSError(str(exc)) from exc

    def read(self, n: int) -> bytes:
        try:
            return self._port.read(n)
        except _errors.TransportError as exc:
            raise OSError(str(exc)) from exc

    def reset_input_buffer(self) -> None:
        try:
            self._port.reset_input_buffer()
        except _errors.TransportError as exc:
            raise OSError(str(exc)) from exc

    def close(self) -> None:
        self._unit.transport().close()  # closes the port; this object's own handle stays for reopen()

    def reopen(self, baudrate: int | None = None) -> None:
        """Open the port again after :meth:`close` or an :meth:`unplug` (OSError while unplugged). ``baudrate``: the
        PC side's rate afterwards (a modelled line only; ``None`` keeps it). Pending replies are lost."""
        try:
            self._port.reopen(baudrate)
        except _errors.TransportError as exc:
            raise OSError(str(exc)) from exc

    @property
    def baudrate(self) -> int | None:
        """The PC side's baud rate (``None``: the line is not modelled, as by default)."""
        rate = self._state()["baud_rate"]
        return None if rate is None else int(rate)

    @baudrate.setter
    def baudrate(self, value: int | None) -> None:
        if value is None:
            raise NotImplementedError(f"SimulatedEC: unmodelling the line (baudrate = None) {_NOT_IN_CORE}")
        self._port.set_baudrate(int(value))

    @property
    def ec_baudrate(self) -> int | None:
        """The rate the simulated EC listens at (``None``: not modelled)."""
        rate = self._state()["unit_baud_rate"]
        return None if rate is None else int(rate)

    def set_baudrate(self, baudrate: int) -> None:
        """Change the PC side's rate in place (like ``SerialTransport.set_baudrate``)."""
        if self.baudrate is None:
            raise TypeError("this SimulatedEC has no baud rate (pass baudrate= to model the line)")
        try:
            self._port.set_baudrate(int(baudrate))
        except _errors.TransportError as exc:
            raise OSError(str(exc)) from exc

    def send_break(self, duration: float) -> None:
        """A line BREAK: >= 150 ms drops the EC to 9600 baud."""
        self._port.send_break(float(duration))

    # ================================================================== test hooks

    def inject_fault(self, kind: str, count: int = 1) -> None:
        """Corrupt the next ``count`` responses: ``timeout`` | ``crc`` | ``partial`` | ``wrong_slave``."""
        if kind not in FAULT_KINDS:
            raise ValueError(f"fault kind must be one of {FAULT_KINDS}")
        self._unit.inject_fault(kind, int(count))

    def trigger_alarm(self, code: int) -> None:
        """Raise an alarm as if the actuator detected it (message level: ALML, the axis keeps running; else ALMH and
        the servo off)."""
        self._unit.trigger_alarm(int(code))

    @property
    def position_mm(self) -> float:
        """Current simulated position."""
        return float(self._state()["position_mm"])

    @property
    def moving(self) -> bool:
        """True while a move, push or home return is unfinished (whatever the MOVE bit says)."""
        return bool(self._state()["moving"])

    @property
    def stop_held(self) -> bool:
        """True while a latched STOP blocks new commands."""
        return bool(self._state()["stop_held"])

    def position_row(self, n: int) -> list[int]:
        """Raw 16 words of position-table entry ``n``."""
        return self._unit.position_row(int(n))

    def pio_move(
        self, target_mm: float, speed_mm_s: float = 20.0, accel_g: float = 0.3, *, hold_start: bool = False
    ) -> bool:
        """Test hook: a PLC starts a move over PIO. Ignored (returns False) while Modbus has control."""
        result = self._unit.control("pio_move", target_mm=float(target_mm), speed_mm_s=float(speed_mm_s),
                                    accel_g=float(accel_g), hold_start=bool(hold_start))  # fmt: skip
        return bool(result.get("started"))

    def pio_release(self) -> None:
        """Test hook: the PLC drops a start input held with ``pio_move(..., hold_start=True)``."""
        self._unit.control("pio_release")

    def set_inputs(
        self, *, emg: bool | None = None, drive_cut: bool | None = None, enable: bool | None = None
    ) -> None:
        """Test hook: the emergency-stop input, a motor drive power cut and the teaching-tool enable switch."""
        inputs = {k: bool(v) for k, v in (("emg", emg), ("drive_cut", drive_cut), ("enable", enable)) if v is not None}
        self._control("set_inputs", "set_inputs (EMG, drive cut, enable inputs)", **inputs)

    @property
    def auto_servo_off_active(self) -> bool:
        """STAT b17 ASOF: the servo is off because of the automatic servo OFF."""
        return bool(self._state().get("auto_servo_off_active", False))

    @property
    def parameter_ram(self) -> dict[int, int]:
        """Parameter words written since the last commit / restart (address -> value): lost by a restart."""
        return {int(a): int(v) for a, v in self._state()["parameter_ram"].items()}

    @property
    def parameter_nv(self) -> dict[int, int]:
        """Parameter words committed with PARW (address -> value): kept across restarts."""
        return {int(a): int(v) for a, v in self._state()["parameter_nv"].items()}

    @property
    def parw_busy(self) -> bool:
        """A PARW commit is running (DRGE b7 reads 1)."""
        return bool(self._state()["parw_busy"])

    def power_cycle(self, *, lose_home: bool | None = None) -> None:
        """Test hook: a 24 V power cycle (:meth:`power_off` then :meth:`power_on`)."""
        self._unit.power_cycle(lose_home=lose_home)

    def power_off(self) -> None:
        """Test hook: the 24 V supply is cut; the unit is silent until :meth:`power_on`."""
        self._unit.power_off()

    def power_on(self, *, lose_home: bool | None = None) -> None:
        """Test hook: the 24 V supply returns (``lose_home``: drop HEND; default: on an incremental unit)."""
        self._unit.power_on(lose_home=lose_home)

    def unplug(self) -> None:
        """Test hook: the USB adapter is pulled: the port vanishes and the open handle raises OSError until
        :meth:`reopen` after :meth:`replug`."""
        self._unit.unplug()

    def replug(self) -> None:
        """Test hook: the USB adapter is plugged back in (the old handle stays dead until :meth:`reopen`)."""
        self._unit.replug()

    @property
    def serial_number(self) -> str:
        """The controller serial served at 0x0600 (CTS1-4)."""
        return str(self._state()["serial_number"])

    # ------------------------------------------------------------------ requests and state (test hooks)

    def on_request(self, frame: bytes) -> bytes | None:
        """Override in a subclass to change the answer to a request: return ``NotImplemented`` for the normal
        handling, ``None`` for no answer, or the reply frame. :meth:`handle` runs the simulator on the frame and
        returns its answer; a :class:`SimException` raised here answers that exception. Every frame written to this
        unit comes here (on a line: the frames to other slaves too), after the line checks. It runs on the thread
        doing the bus I/O: no other driver call from it."""
        return NotImplemented  # type: ignore[return-value]

    def handle(self, frame: bytes) -> bytes | None:
        """The simulator's own answer to ``frame`` (``None``: no answer), with its effects (for :meth:`on_request`)."""
        return self._unit.handle(bytes(frame))

    def set_word_override(self, address: int, value: int | None) -> None:
        """Serve ``value`` for register ``address`` instead of the modelled value (``None``: model it again)."""
        self._unit.set_word_override(int(address), None if value is None else int(value) & 0xFFFF)

    def register_value(self, address: int) -> int:
        """The value a read of register ``address`` returns now (raises :class:`SimException` where a read is
        refused)."""
        try:
            return self._unit.register_value(int(address))
        except _n.NativeOutOfRangeError as exc:
            raise SimException(0x02) from exc

    def item_address(self, class_id: int, item_id: int) -> int | None:
        """The first register of 32-bit item (class, item id) in the served directory and ID map; None: not served."""
        for i in range(self._served(2) or 0):
            rec = [self._served(8 + 8 * i + j) or 0 for j in range(8)]
            if rec[0] == class_id and rec[5] and rec[7] == 2:
                for k in range(rec[2]):
                    if self._served(rec[5] + 2 * k) == item_id:
                        return rec[4] + 2 * k
        return None

    def set_parameter_word(self, address: int, value: int, *, nv: bool = False) -> None:
        """Set one parameter word as if written (``nv=False``: in the parameter RAM) or committed (``nv=True``)."""
        self._control("set_parameter_word", "set_parameter_word", address=int(address), value=int(value) & 0xFFFF,
                      nv=bool(nv))  # fmt: skip

    def coil_state(self, address: int) -> bool:
        """The state of coil ``address`` as last written (a self-clearing coil reads OFF)."""
        return self._unit.coil_state(int(address))

    def set_coil_state(self, address: int, on: bool) -> None:
        """Set the state of coil ``address`` without its effect (e.g. a STOP left ON by another program)."""
        self._unit.set_coil_state(int(address), bool(on))

    def set_position_row(self, n: int, words: Sequence[int]) -> None:
        """Store the raw 16 words of position-table entry ``n`` as they are (the checksum word is not recomputed)."""
        if len(words) != R.POSITION_STRIDE:
            raise ValueError(f"a position row has {R.POSITION_STRIDE} words, got {len(words)}")
        self._unit.set_position_row(int(n), [int(w) & 0xFFFF for w in words])

    @property
    def command_speed_mm_s(self) -> float:
        """The speed of the move or push in progress (or the last one)."""
        return float(self._state()["command_speed_mm_s"])

    @property
    def push_speed_mm_s(self) -> float:
        """The speed of the push stroke in progress (or the last one)."""
        return float(self._state()["push_speed_mm_s"])

    @property
    def position_count(self) -> int:
        """Stored rows the unit reports (MAXP, item 3.2)."""
        return self._position_count

    # ------------------------------------------------------------------ live state (the core's)

    servo_on = _state_flag("servo_on", "The servo is ON (settable as a test hook).")
    homed = _state_flag("homed", "HEND: the home position is established (settable as a test hook).")
    pmsl = _state_flag("pmsl", "PMSL / PMSS: Modbus has control (settable as a test hook).")
    safety_speed = _state_flag("safety_speed", "SFTY: the safety speed is on (settable as a test hook).")
    paused = _state_value("paused", "A move is held by STP.", bool)
    alarm_code = _state_value("alarm_code", "The present alarm (0: none).", int)
    powered = _state_value("powered", "24 V present (power_off() / power_on()).", bool)
    present = _state_value("present", "The port is listed by the PC (unplug() / replug()).")
    reopens = _state_value("reopens", "How often the port was reopened.", int)
    nonvolatile_writes = _state_value("nonvolatile_writes", "Position-table row writes.", int)
    parameter_commits = _state_value("parameter_commits", "Completed PARW commits.", int)
    parameter_ram_writes = _state_value("parameter_ram_writes", "Parameter words written to RAM.", int)
    software_resets = _state_value("software_resets", "Software resets done.", int)
    override_moves = _state_value("override_moves", "Moves started with 0x45 cmd 6.", int)
    profile_moves = _state_value("profile_moves", "Moves started with the PCRF coil.", int)
    accel_pattern = _state_value("accel_pattern", "CTLF b6-b7 of the last move started.", str)
    stale_polls_served = _state_value("stale_polls_served", "Status reads answered stale so far.", int)
    soft_limits_mm = _state_value("soft_limits_mm", "The soft limits (min, max) in mm.",
                                  lambda v: (float(v[0]), float(v[1])))  # fmt: skip

    @property
    def closed(self) -> bool:
        """The port was closed (:meth:`close`) and not reopened."""
        return not self._state()["open"]

    @property
    def obstacle_mm(self) -> float | None:
        """Test hook: a workpiece a push makes contact with (``None``: none)."""
        value = self._state()["obstacle_mm"]
        return None if value is None else float(value)

    @obstacle_mm.setter
    def obstacle_mm(self, value: float | None) -> None:
        self._unit.control("set_obstacle", mm=None if value is None else float(value))

    @property
    def stale_status_polls(self) -> int:
        """After each 0x9900 move command, this many status reads still show PEND with MOVE clear (0 = off)."""
        return int(self._state()["stale_status_polls"])

    @stale_status_polls.setter
    def stale_status_polls(self, value: int) -> None:
        self._unit.set_stale_status_polls(int(value))

    @property
    def step_s(self) -> float | None:
        """The simulator's own clock step per request (``None``: the unit runs on its clock)."""
        value = self._state()["step_s"]
        return None if value is None else float(value)

    @step_s.setter
    def step_s(self, value: float) -> None:
        self._unit.control("set_step_s", step_s=float(value))

    # ------------------------------------------------------------------ options (as given; changed in the core)

    max_accel_g = _option("max_accel_g", "ACMD above this raises an alarm.")
    firmware = _option("firmware", "The FC11 firmware bytes.")
    dopm_layout = _option("dopm_layout", "DOPM layout: '3pos' or '2pos'.")
    recompute_position_checksum = _option("recompute_position_checksum", "Word +0xF updated after a row write.")
    drop_reply_after_retarget = _option("drop_reply_after_retarget", "No answer after a mid-move command.")
    stop_latches = _option("stop_latches", "STOP stays ON until written OFF.")
    move_bit_clears_in_decel = _option("move_bit_clears_in_decel", "DSSE MOVE clears in the final deceleration.")
    serve_soft_limits = _option("serve_soft_limits", "The soft-limit block 0x0700 is served.")
    @property
    def ls_range_mm(self) -> float:
        """Half-width of the LS ranges around each row (EC parameter 2) as the unit applies it (a committed change
        applies at the next restart, when the core reports it)."""
        return float(self._state().get("ls_range_mm", self._options["ls_range_mm"]))

    @ls_range_mm.setter
    def ls_range_mm(self, value: float) -> None:
        self._set_option("ls_range_mm", float(value))
    early_in_position = _option("early_in_position", "PEND / MEND rise inside the band while moving.")
    settle_tail_mm = _option("settle_tail_mm", "The settling tail of a normal move.")
    settle_speed_mm_s = _option("settle_speed_mm_s", "The speed over the settling tail.")
    push_blip_mm = _option("push_blip_mm", "Where a push shows the contact fingerprint for a moment.")
    push_blip_len_mm = _option("push_blip_len_mm", "How long that fingerprint lasts (mm).")
    boot_baudrate = _option("boot_baudrate", "The unit's rate after a power cycle.")
    reset_baudrate = _option("reset_baudrate", "The unit's rate after a software reset (None: unchanged).")
    reset_s = _option("reset_s", "Simulated seconds the unit is silent after a software reset.")
    slave_id_delay_s = _option("slave_id_delay_s", "Real seconds before an FC11 answer can be read.")
    parw_busy_s = _option("parw_busy_s", "Real seconds DRGE b7 reads 1 after the PARW coil.")
    parw_dsse_busy = _option("parw_dsse_busy", "DSSE b7 reads 0 while a commit runs.")
    ignored_baud_codes = _option("ignored_baud_codes", "Baud-change codes the unit ignores.")
    auto_servo_off_s = _option("auto_servo_off_s", "Automatic servo OFF after this standstill (None: off).")
    cold_boot = _option("cold_boot", "The unit starts with the servo off and a slow first servo ON.")
    ec_first_son_s = _option("ec_first_son_s", "On an EC with cold_boot, the time the first servo ON takes.")
    function_codes = _option("function_codes", "Only these function codes are answered (None: all).")
    broadcast = _option("broadcast", "Frames to address 0 are acted on.")
    parw_latches = _option("parw_latches", "The PARW coil starts a commit.")


# what a bus calls on its transport: a class or an object that replaces one of these is driven through its methods
_TRANSPORT_NAMES = ("write", "read", "reset_input_buffer", "close", "reopen", "baudrate", "set_baudrate", "send_break",
                    "present", "timeout")  # fmt: skip


def _replaces_transport(obj: Any, base: type) -> bool:
    """``obj``'s class (a subclass of ``base``) or ``obj`` itself replaces one of the transport methods of ``base``."""
    own = getattr(obj, "__dict__", {})
    return any(name in own or getattr(type(obj), name, None) is not getattr(base, name, None) for name in
               _TRANSPORT_NAMES)  # fmt: skip


def _library_clock(clock: Callable[[], float]) -> bool:
    """``clock`` is the clock the core runs on anyway (the real one, or the core's virtual clock of
    ``testing.virtual_clock``)."""
    if clock in _LIBRARY_CLOCKS or clock is _n.clock_now:
        return True
    return isinstance(getattr(clock, "__self__", None), _n.NativeVirtualClock)


def _request_hook(sim: SimulatedEC) -> Callable[[bytes], Any]:
    """The core's request hook for a subclass that overrides :meth:`SimulatedEC.on_request` (holds the unit weakly)."""
    ref = weakref.ref(sim)

    def hook(frame: bytes) -> Any:
        unit = ref()
        if unit is None:
            return NotImplemented
        try:
            answer = unit.on_request(frame)
        except SimException as exc:
            return int(exc.code) if len(frame) > 1 else None
        if answer is NotImplemented or answer is None:
            return answer
        return bytes(answer)

    return hook

