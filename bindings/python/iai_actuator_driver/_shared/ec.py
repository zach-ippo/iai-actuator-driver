"""The Python part of ``iai_actuator_driver.ec``: the command strategies, the timing constants and driver
limits, and the argument checks.

:class:`CommandStrategy` and its four strategies name how servo, home and alarm reset reach a unit: the core's
strategies carry the same names. ``iai_actuator_driver.ec`` (where the protocol notes and the evidence are) imports
these names from here.
"""

from __future__ import annotations

import logging
import math
from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import TYPE_CHECKING, ClassVar

from .. import registers as R
from ..errors import EcNotReadyError, ModbusError
from . import publish

if TYPE_CHECKING:
    from ..modbus import ModbusRTU

log = logging.getLogger("iai_actuator_driver.ec")  # the public module name, on every backend

SETTLE_S = 0.1  # "done" needs PNOW unchanged over two polls this far apart (PEND can rise before standstill)
PUSH_SETTLE_S = 0.15  # a push contact / miss must hold this long, position unchanged (EC-GRBP8M showed a transient)
PUSH_CREEP_MM = 0.05  # a holding contact may creep this much over PUSH_SETTLE_S (EC-DS4H: 0.01 mm in 0.26 s)
STORED_START_S = 0.5  # a 0x9800 position-number move must visibly start within this (on hardware: ~50 ms)
FALLBACK_SPEED_CAP = 100.0  # units/s: the speed cap only while neither the unit's maximum nor a configured cap is known
FALLBACK_ACCEL_CAP_G = 0.30  # G: the same for the acceleration / deceleration
FALLBACK_PUSH_CAP_PCT = 70.0  # %: the same for the push force
MOVE_BAND_MAX = 10.0  # driver sanity limit of a positioning band (units); a push band may span the stroke
RESET_GAP_PROBE_S = 0.05  # after a software reset command: probe this often to see the unit go silent (PCON-CB: ~1 s)
P1_MIN_MM = 1.00  # driver sanity limits for parameter writes (the EC's own valid ranges are not documented)
P2_RANGE_MM = (0.01, 1.00)  # LS detection range (0.10 on both units tested)
P4_RANGE_MM = (0.00, 5.00)  # home position adjustment (gripper 0.50, slider 2.00)
PARW_READY_S = 2.0  # before a parameter write / the PARW coil: wait at most this for DRGE b7 = 0 and DSSE b7 = 1
PARW_CLEAR_S = 2.0  # after the PARW coil: DRGE b7 = 0 and DSSE b7 = 1 again within this (ON-WIRE: 0.24 s)
PARW_OBSERVE_S = 0.3  # with neither DRGE b7 = 1 nor DSSE b7 = 0 seen, keep watching this long before "unconfirmed"
NV_WRITE_CYCLES = 100_000  # the units' non-volatile memory lasts about this many write cycles (IAI)
RESET_WAIT_S = 1.0  # after the software reset: wait this long before probing (ON-WIRE: IA-OS reopened after ~1.1 s)
RESET_TIMEOUT_S = 10.0  # ... and give up when the EC has not answered this long after the reset
BREAK_S = 0.2  # method="break": 200 ms line BREAK (DOC: >= 150 ms drops the unit to 9600 baud; UNVERIFIED on an EC)
BAUD_SETTLE_S = 0.2  # after the broadcast baud change, before the port switches (ON-WIRE: IA-OS waited ~0.2 s)
SERVO_TIMEOUT_S = 3.0  # servo on / off must show in DSS1 within this (a CON unit's first servo ON may take longer)
SERVO_FIRST_ON_MAX_S = 13.3  # first servo ON with No.29 / No.30 unreadable: 0.006 + (0.22 + 0.999) x 10 + 0.020 + 1 s
ASOF_WAKE_S = 1.5  # automatic servo OFF: the servo must come back within this after SON (else a SON off/on cycle)
SMOOTH_ACCEL_MAX_TIME_S = 2.0  # P5 on: accel / decel times above 2 s are not allowed (DOC ME3816-6B p.6-11)
COMPLETION_MODES = ("safe", "fast")  # safe: PEND + band + PNOW still over settle_s; fast: one sample (fast_done)
FAST_BAUDRATE = 230400  # Axis.open(fast=True): the rate raised to (VERIFIED on the EC-R6S rod, the EC-GRBP8M gripper and the PCON-CB)
FAST_POLL_S = 0.002  # completion="fast" without a monitor: pause between two lean status reads
TEACH_WRITE_S = 1.0  # experimental TEAC path: DSS2 b10 PWRT must rise within this (DOC ME0162-12A p.5-97: >= 20 ms)


class CommandStrategy(ABC):
    """How servo / home / alarm-reset commands are sent.

    Moves always use the 0x9900 block. Stops use the STOP coil (``stop_method = "coil"``), except with the
    ``"register"`` strategy, whose stop is the coil-free substitute (``stop_method = "register"``).
    """

    name: ClassVar[str]
    home_needs_org: ClassVar[bool] = False  # start_home needs the EC's home pattern ORG (item 9.16)
    stop_method: ClassVar[str] = "coil"  # how Elecylinder.stop() / pause() / resume() reach the unit
    uses_coils: ClassVar[bool] = True  # servo / home / reset are FC05 writes
    uses_iai: ClassVar[bool] = False  # servo / home / reset are IAI 0x45 / 0x48 commands

    @abstractmethod
    def servo(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        """Turn the servo on or off."""

    @abstractmethod
    def start_home(self, bus: ModbusRTU, slave: int, *, org: int | None = None) -> None:
        """Start a home return (``org``: the EC's home pattern, item 9.16, for strategies that need it)."""

    def finish_home(self, bus: ModbusRTU, slave: int) -> None:  # noqa: B027 - optional hook
        """Called after homing ended or failed (e.g. to release an edge-triggered coil)."""

    @abstractmethod
    def reset_alarm(self, bus: ModbusRTU, slave: int) -> None:
        """Reset the current alarm (not a software reset)."""

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class StandardCoilStrategy(CommandStrategy):
    """FC05 coils (ME0162). SON, ALRS and HOME are VERIFIED-LIVE on the ECs tested. The default."""

    name = "coil"

    def servo(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        bus.write_coil(slave, R.COIL_SON, on)

    def start_home(self, bus: ModbusRTU, slave: int, *, org: int | None = None) -> None:
        bus.write_coil(slave, R.COIL_HOME, False)  # HOME acts on the rising edge
        bus.write_coil(slave, R.COIL_HOME, True)

    def finish_home(self, bus: ModbusRTU, slave: int) -> None:
        bus.write_coil(slave, R.COIL_HOME, False)

    def reset_alarm(self, bus: ModbusRTU, slave: int) -> None:
        bus.write_coil(slave, R.COIL_ALRS, True)  # ALRS acts on the rising edge
        bus.write_coil(slave, R.COIL_ALRS, False)


class IaiCommandStrategy(CommandStrategy):
    """IAI function 0x45 control commands.

    Servo 0x45 cmd 9 op1 1/0: VERIFIED on the EC-R6S rod (servo OFF also ON-WIRE). Home 0x45 cmd 0x0A with op1 =
    the home pattern ORG (item 9.16) & 0xFFFF (``01 45 00 0A 00 07 00 00``): VERIFIED on the EC-R6S rod (op1 = 0 is
    rejected 0x11 "1st operand error"); sent once, without retries (a repeated home frame after a lost reply
    would be rejected 0x24 while the first one runs). Alarm reset 0x45 cmd 5 op1 0: frame accepted on
    hardware with no alarm active; clearing a real alarm UNVERIFIED. A rejection raises
    :class:`IaiCommandRejected` naming the reason.
    """

    name = "iai"
    home_needs_org = True
    uses_coils = False
    uses_iai = True

    def servo(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        bus.iai_command(slave, 0, R.IAI_CMD_SERVO, 1 if on else 0)

    def start_home(self, bus: ModbusRTU, slave: int, *, org: int | None = None) -> None:
        if org is None or org & 0xFFFF == 0:
            raise EcNotReadyError(
                "the 0x45 home needs op1 = the EC's home pattern ORG (item 9.16); none given (op1 = 0 is rejected "
                "0x11 '1st operand error'). Nothing sent"
            )
        # Sent once: a lost reply may hide a home that runs (Elecylinder.home then checks the status).
        bus.iai_command(slave, 0, R.IAI_CMD_HOME, org & 0xFFFF, retries=0)

    def reset_alarm(self, bus: ModbusRTU, slave: int) -> None:
        bus.iai_command(slave, 0, R.IAI_CMD_RESET, R.IAI_RESET_ALARM)  # op1 0 = alarm reset (1 = software reset)


class IaiCommand2Strategy(CommandStrategy):
    """EXPERIMENTAL: IAI function 0x48 control commands. UNVERIFIED: untested on any EC, frame layout included
    (:meth:`ModbusRTU.iai_command2`).

    Expected only on units whose SUP3 IVC1 bit is set, which none of the ECs tested has (they all answer 0x45)."""

    name = "iai2"
    uses_coils = False
    uses_iai = True

    def servo(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        bus.iai_command2(slave, R.IAI2_CMD_SERVO, (1 if on else 0,))

    def start_home(self, bus: ModbusRTU, slave: int, *, org: int | None = None) -> None:
        bus.iai_command2(slave, R.IAI2_CMD_HOME)  # 0x48 home without an operand (UNVERIFIED)

    def reset_alarm(self, bus: ModbusRTU, slave: int) -> None:
        bus.iai_command2(slave, R.IAI2_CMD_ALARM_RESET)


class RegisterCommandStrategy(CommandStrategy):
    """FC06 writes of DRG1 0x0D00 (no FC05 coil, no IAI command): the path RCON documents (DOC ME0413 p.5-11, 5-13)
    and every CON unit too (DOC ME0162-12A p.5-124 to 5-128). On an EC the DRG1 readback is VERIFIED (0x1000 /
    0x5000), an FC06 write to it is UNVERIFIED.

    Every write is a read-modify-write of the unit's own DRG1 image: SON, SFTY and a held STP are carried over
    (:data:`registers.DRG1_KEEP`), EMG and BKRL are never set. Frames (slave 01, servo on, safety speed off):

    * servo on / off ``01 06 0D 00 10 00`` / ``01 06 0D 00 00 00``
    * home (edge) ``10 10``, released ``10 00`` once homing ended
    * alarm reset (servo stays on) ``11 00`` then ``10 00``, not IAI's sample ``01 00`` (it drops SON)
    * pause / resume ``10 20`` / ``10 00``
    * stop substitute (``stop_method = "register"``): pause ``10 20``, wait until the axis stands still, an ALRS
      edge while paused ``11 20`` then ``10 20`` (cancels the remaining travel: DOC ME0384 p.6-76, ME0162-12A
      p.4-11, 5-85), release ``10 00``.

    The image is read with FC03 before each write (one extra frame); when that read fails, the last image read is
    used, never a guess (a guessed SON = 0 would switch the servo off).
    """

    name = "register"
    stop_method = "register"
    uses_coils = False

    def __init__(self) -> None:
        self._images: dict[int, int] = {}  # slave -> DRG1 levels last read / written (DRG1_KEEP bits)

    def image(self, bus: ModbusRTU, slave: int) -> tuple[int, int]:
        """(levels to keep, raw DRG1) read now; on a failed read the last image (raw unknown: = the levels)."""
        try:
            raw = bus.read_registers(slave, R.DRG1, 1)[0]
        except (ModbusError, OSError):
            if slave not in self._images:
                raise
            log.warning("DRG1 not readable: using the last DRG1 image 0x%04X", self._images[slave])
            return self._images[slave], self._images[slave]
        self._images[slave] = raw & R.DRG1_KEEP
        return raw & R.DRG1_KEEP, raw

    def write(self, bus: ModbusRTU, slave: int, value: int) -> None:
        """FC06 DRG1 = ``value`` (EMG and BKRL always cleared)."""
        value &= ~(R.DRG1_EMG | R.DRG1_BKRL) & 0xFFFF
        bus.write_register(slave, R.DRG1, value)
        self._images[slave] = value & R.DRG1_KEEP

    def servo(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        keep, _ = self.image(bus, slave)
        self.write(bus, slave, keep | R.DRG1_SON if on else keep & ~R.DRG1_SON)

    def start_home(self, bus: ModbusRTU, slave: int, *, org: int | None = None) -> None:
        keep, raw = self.image(bus, slave)
        if raw & R.DRG1_HOME:
            self.write(bus, slave, keep)  # HOME acts on the rising edge: drop a level left high first
        self.write(bus, slave, keep | R.DRG1_HOME)

    def finish_home(self, bus: ModbusRTU, slave: int) -> None:
        keep, _ = self.image(bus, slave)
        self.write(bus, slave, keep)

    def reset_alarm(self, bus: ModbusRTU, slave: int) -> None:
        keep, _ = self.image(bus, slave)
        self.write(bus, slave, keep | R.DRG1_ALRS)
        keep, _ = self.image(bus, slave)  # re-read: an alarm reset may bring the servo back by itself
        self.write(bus, slave, keep)

    def pause(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        """STP level: ``on`` pauses (keeps the remaining travel), off resumes. A pause (the first frame of a STOP)
        is always sent: when DRG1 cannot be read and no image is saved, SON / SFTY come from DSS1 (0x9005), else SON
        alone is kept (an axis that moves has its servo on)."""
        try:
            keep, _ = self.image(bus, slave)
        except (ModbusError, OSError):
            if not on:
                raise
            keep = self._levels_from_status(bus, slave)
        self.write(bus, slave, keep | R.DRG1_STP if on else keep & ~R.DRG1_STP)

    @staticmethod
    def _levels_from_status(bus: ModbusRTU, slave: int) -> int:
        try:
            dss1 = bus.read_registers(slave, R.DSS1, 1)[0]
        except (ModbusError, OSError):
            log.warning("DRG1 and DSS1 not readable: pausing with SON kept")
            return R.DRG1_SON
        log.warning("DRG1 not readable: pausing with SON / SFTY taken from DSS1 0x%04X", dss1)
        return (R.DRG1_SON if dss1 & R.DSS1_SV else 0) | (R.DRG1_SFTY if dss1 & R.DSS1_SFTY else 0)

    def safety_speed(self, bus: ModbusRTU, slave: int, on: bool) -> None:
        keep, _ = self.image(bus, slave)
        self.write(bus, slave, keep | R.DRG1_SFTY if on else keep & ~R.DRG1_SFTY)

    def cancel_paused(self, bus: ModbusRTU, slave: int, *, release: bool = True) -> None:
        """ALRS edge with STP held (drops the remaining travel), then, with ``release``, STP off."""
        keep, _ = self.image(bus, slave)
        held = keep | R.DRG1_STP
        self.write(bus, slave, held | R.DRG1_ALRS)
        self.write(bus, slave, held)
        if release:
            self.write(bus, slave, held & ~R.DRG1_STP)


STRATEGIES: dict[str, type[CommandStrategy]] = {
    StandardCoilStrategy.name: StandardCoilStrategy,
    IaiCommandStrategy.name: IaiCommandStrategy,
    IaiCommand2Strategy.name: IaiCommand2Strategy,
    RegisterCommandStrategy.name: RegisterCommandStrategy,
}


def _check_completion(value: str) -> str:
    if value not in COMPLETION_MODES:
        raise ValueError(f"completion must be one of {COMPLETION_MODES}, got {value!r}")
    return value


def _check_stroke(stroke: Sequence[float]) -> tuple[float, float]:
    lo, hi = float(stroke[0]), float(stroke[1])
    if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
        raise ValueError(f"invalid stroke {tuple(stroke)!r}")
    return lo, hi


WRITABLE_PARAMETER_CLASSES: frozenset[int] = frozenset({0x0007, 0x0008, 0x0009, 0x000C})


def info_item_note(class_id: int, item_id: int) -> str:
    """What the driver knows about an item (``info.ITEM_NOTES``), or ``""``."""
    from ..info import ITEM_NOTES

    return ITEM_NOTES.get((class_id, item_id), "")


publish(globals(), "ec")
