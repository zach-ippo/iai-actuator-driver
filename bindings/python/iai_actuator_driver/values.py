"""The records the driver returns: :class:`Status`, :class:`DeviceInfo`,
:class:`AxisLimits`, :class:`PositionEntry`, :class:`PushResult`, :class:`ParameterWriteResult`,
:class:`SoftwareResetResult`, :class:`BaudChangeResult`, :class:`AlarmRecord`, :class:`IndexHeader` and
:class:`IndexCategory`, and the notes of the non-volatile write guard.

Plain frozen dataclasses with their decoders (``Status.from_registers``, ``PositionEntry.from_registers`` /
``to_registers``) and ``to_dict``; no I/O. :mod:`iai_actuator_driver.ec` re-exports every name here
(``from iai_actuator_driver.ec import Status`` keeps working). Evidence words as in the README: DOC, ON-WIRE,
VERIFIED, UNVERIFIED.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from typing import Any, ClassVar

from . import registers as R
from .alarms import LEVEL_MESSAGE, alarm_is_fault, alarm_level, alarm_name

__all__ = [
    "Status",
    "DeviceInfo",
    "AxisLimits",
    "IndexHeader",
    "IndexCategory",
    "PositionEntry",
    "PushResult",
    "AlarmRecord",
    "ParameterWriteResult",
    "SoftwareResetResult",
    "BaudChangeResult",
    "NV_WARN_PER_HOUR",
    "NV_WINDOW_S",
    "NV_SKIP_NOTE",
    "BAUD_VOLATILE_NOTE",
]

NV_WARN_PER_HOUR = 60  # more non-volatile writes than this in the last hour on one axis: one WARNING per hour
NV_WINDOW_S = 3600.0  # the sliding window of the rate warning
NV_SKIP_NOTE = "unchanged: nothing written (one non-volatile write cycle saved)"
BAUD_VOLATILE_NOTE = (
    "volatile: the EC answers at 38400 baud again after every power-up (VERIFIED); a broadcast changes the "
    "rate of every EC on the RS-485 line"
)


# =========================================================================== data classes


@dataclass(frozen=True)
class Status:
    """Decoded monitor block (0x9000...). ``raw`` holds the register words by name.

    ``moving`` is "the axis is in motion": DSSE MOVE, a home return in progress, or servo on
    without PEND / PSFL (and not paused). The raw MOVE bit (:attr:`move_bit`) alone is not
    reliable on the EC: it clears while still decelerating and during the homing creep.

    ``velocity_mm_s`` is 0.0 when not moving; while moving it is derived from successive PNOW
    samples by :meth:`Elecylinder.status`, or without a recent sample it is VNOW (0.01 mm/s, but
    quantized in ~12.5 mm/s steps on the EC; ``None`` if only the short block was read).
    ``current_ma`` is CNOW as documented for CON controllers.

    The condition flags (:attr:`position_complete_no`, :attr:`cold_start_alarm`, :attr:`rtc_in_use`,
    :attr:`drive_power_cut`, :attr:`emg_input`, :attr:`enable_off`, :attr:`auto_servo_off`, :attr:`nv_busy`,
    :attr:`manual_mode`, :attr:`battery_low`) are ``None`` when their word was not read (a short status block) or
    the line's units do not define the bit (``line == "rcon"``: DOC ME0413 p.5-15 to 5-19), never a guessed False.
    Bit meanings DOC ME0162-12A p.4-18 to 4-25; on an EC the meanings are UNVERIFIED (the values are read live).
    """

    position_mm: float
    alarm_code: int
    servo_on: bool
    homed: bool
    in_position: bool
    moving: bool
    alarm: bool
    modbus_enabled: bool
    emergency_stop: bool
    velocity_mm_s: float | None
    current_ma: int | None
    raw: Mapping[str, int] = field(default_factory=dict)
    line: str | None = None  # "rcon" behind an RCON gateway (bits that unit family does not define read as None)

    # (word, mask) of the bits RCON driver units leave undefined ("-", DOC ME0413 p.5-15 to 5-19)
    _RCON_UNDEFINED: ClassVar[dict[str, int]] = {
        "DSSE": (1 << 12) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 3) | (1 << 0),
        "STAT": R.STAT_AEEP,
        "ZONS": R.ZONS_LS0 | R.ZONS_LS1 | R.ZONS_LS2,
    }

    def _bit(self, word: str, mask: int) -> bool | None:
        """``word & mask`` as a bool; ``None`` when the word was not read or the line leaves the bit undefined."""
        value = self.raw.get(word)
        if value is None or (self.line == "rcon" and self._RCON_UNDEFINED.get(word, 0) & mask):
            return None
        return bool(value & mask)

    @property
    def position_complete_no(self) -> int | None:
        """POSS 0x9014: the completed position number (b0-b9; 7 bits on RCON), also valid after a push contact or a
        push miss (DOC ME0162-12A p.4-24, 5-64; ME0413 p.5-10 to 5-23). 0 after a 0x9900 move. ``None`` if not read."""
        poss = self.raw.get("POSS")
        if poss is None:
            return None
        return poss & (R.POSS_MASK_RCON if self.line == "rcon" else R.POSS_MASK)

    @property
    def cold_start_alarm(self) -> bool | None:
        """SSSE b11: an alarm of the cold-start level is present (software reset / power cycle; DOC p.4-25)."""
        return self._bit("SSSE", R.SSSE_ALMC)

    @property
    def rtc_in_use(self) -> bool | None:
        """SSSE b8: the real-time clock is in use (alarm times are then calendar times; DOC p.4-25)."""
        return self._bit("SSSE", R.SSSE_RTC)

    @property
    def drive_power_cut(self) -> bool | None:
        """Motor drive power cut: STAT b0 MPOW = 0 or DSSE b14 MPUV = 1 (DOC ME0162-12A p.4-20, 4-21; e.g. an external
        drive cut-off). ``None`` only when neither word was read."""
        mpuv = self._bit("DSSE", R.DSSE_MPUV)
        mpow = self._bit("STAT", R.STAT_MPOW)
        if mpuv is None and mpow is None:
            return None
        return bool(mpuv) or mpow is False

    @property
    def emg_input(self) -> bool | None:
        """DSSE b15 EMGP: the emergency-stop input is active (DOC ME0162-12A p.4-20)."""
        return self._bit("DSSE", R.DSSE_EMGP)

    @property
    def enable_off(self) -> bool | None:
        """DSS2 b15 ENBS = 0: the enable (dead-man) switch of a connected teaching tool is released, the servo is held
        off (DOC ME0162-12A p.4-19; a servo-ON precondition, p.4-11, 5-83). Fixed 1 in AUTO or without the function."""
        enbs = self._bit("DSS2", R.DSS2_ENBS)
        return None if enbs is None else not enbs

    @property
    def auto_servo_off(self) -> bool | None:
        """STAT b17 ASOF: the unit switched the servo off by itself after a standstill (automatic servo OFF; DOC
        ME0162-12A p.4-21). The next move re-energises it on CON (DOC PCON p.5-5). ``None`` without STAT."""
        return self._bit("STAT", R.STAT_ASOF)

    @property
    def nv_busy(self) -> bool | None:
        """STAT b16 AEEP: a non-volatile memory write is in progress (DOC p.4-21; not defined on RCON)."""
        return self._bit("STAT", R.STAT_AEEP)

    @property
    def manual_mode(self) -> bool | None:
        """DSSE b13 RMDS: the unit's MANU / AUTO mode bit, 1 = MANU, 0 = AUTO (on a unit with a MANU / AUTO mode
        switch, AUTO gives the PIO control; DOC ME0162-12A p.4-20). It does **not** show PC vs PIO control: that is
        :attr:`modbus_enabled` (DSSE PMSS). On the EC it stays 1 in PC and in PIO mode alike (VERIFIED on the
        EC-GRBP8M gripper: True with PMSS 0 and with PMSS 1)."""
        return self._bit("DSSE", R.DSSE_RMDS)

    @property
    def battery_low(self) -> bool | None:
        """STAT b31 BATL: absolute-encoder battery voltage low (SCON; operation continues; DOC p.4-21)."""
        return self._bit("STAT", R.STAT_BATL)

    @property
    def excitation_done(self) -> bool:
        """DSSE b9 PSNS: excitation detection complete (0 before the first servo ON after power-up on CON, and during
        the EC's homing creep)."""
        return bool(self.raw.get("DSSE", 0) & R.DSSE_PSNS)

    @property
    def warning(self) -> bool:
        """A message-level alarm is active (a warning: the axis keeps running), not a fault.

        True when :attr:`alarm` is False and either the code is of the message level (0x011-0x07F, e.g.
        a maintenance warning 0x04E) or DSS1 b9 ALML is set. Shown as a warning lamp, never the red one."""
        if self.alarm:
            return False
        return bool(self.raw.get("DSS1", 0) & R.DSS1_ALML) or (
            self.alarm_code != 0 and alarm_level(self.alarm_code) == LEVEL_MESSAGE
        )

    @property
    def alarm_level(self) -> str | None:
        """The level of the present alarm code (``alarms.alarm_level``), ``None`` when no code is set."""
        return alarm_level(self.alarm_code) if self.alarm_code else None

    @property
    def fault(self) -> bool:
        """Same as :attr:`alarm`: a fault that blocks motion (ALMH, or level operation cancelled / cold start)."""
        return self.alarm

    @property
    def move_bit(self) -> bool:
        """Raw DSSE MOVE bit (clears early during deceleration; prefer :attr:`moving`)."""
        return bool(self.raw.get("DSSE", 0) & R.DSSE_MOVE)

    @property
    def homing(self) -> bool:
        """Home return in progress (DSSE GHMS, still set during the final creep)."""
        return bool(self.raw.get("DSSE", 0) & R.DSSE_GHMS)

    @property
    def paused(self) -> bool:
        """Pause (STP) active (DSS1 bit 5)."""
        return bool(self.raw.get("DSS1", 0) & R.DSS1_STP)

    @property
    def target_reached(self) -> bool:
        """DSS1 bit 0: the last commanded move ended at its target (0 after a STOP)."""
        return bool(self.raw.get("DSS1", 0) & R.DSS1_TARGET_REACHED)

    @property
    def push_missed(self) -> bool:
        """DSS1 bit 11 (PSFL): the last push went over its whole band without contact."""
        return bool(self.raw.get("DSS1", 0) & R.DSS1_PSFL)

    @property
    def pushing(self) -> bool:
        """DSSE bit 10: push in progress, or holding after contact."""
        return bool(self.raw.get("DSSE", 0) & R.DSSE_PUSH)

    @property
    def push_contact(self) -> bool:
        """The contact fingerprint (PEND and DSSE bit 0) in this one sample.

        One sample is not proof of contact: on the EC-GRBP8M gripper a push that then missed showed it for a
        moment while still moving. :meth:`Elecylinder.push` waits for it to hold. Behind an RCON gateway DSSE b0
        is not defined (DOC ME0413 p.5-15): there it is PEND with DSSE b10 PUSH and no PSFL (UNVERIFIED).
        """
        dsse = self.raw.get("DSSE", 0)
        if self.line == "rcon":
            return self.in_position and bool(dsse & R.DSSE_PUSH) and not self.push_missed
        return self.in_position and bool(dsse & R.DSSE_PUSH_CONTACT)

    @property
    def safety_speed(self) -> bool:
        """DSS1 bit 14 (SFTY): safety speed enabled on the EC (set externally, e.g. IA-OS Teach mode)."""
        return bool(self.raw.get("DSS1", 0) & R.DSS1_SFTY)

    def _zons(self, bit: int) -> bool | None:
        return self._bit("ZONS", bit)

    @property
    def at_backward_end(self) -> bool | None:
        """ZONS bit 12 (LS0): within the LS range of position row 0, the backward end. ``None`` if ZONS was not read.

        VERIFIED-LIVE on the 3-position EC-DS4H slider and the 2-position EC-GRBP8M gripper. The
        LS bits follow the position, also while moving (EC parameter 2, 0.10 mm on both units);
        for a push row the marker is at its push end. DOPM is not used: its bit layout differs
        between models (DOPM b2 is *ALM on the gripper).
        """
        return self._zons(R.ZONS_LS0)

    @property
    def at_forward_end(self) -> bool | None:
        """ZONS bit 13 (LS1): within the LS range of row 1 (a push row: its push end). ``None`` if not read."""
        return self._zons(R.ZONS_LS1)

    @property
    def at_middle(self) -> bool | None:
        """ZONS bit 14 (LS2): within the LS range of row 2 (also on the 2-position gripper). ``None`` if not read."""
        return self._zons(R.ZONS_LS2)

    @property
    def vnow_mm_s(self) -> float | None:
        """VNOW in mm/s (coarse: quantized in ~12.5 mm/s steps), if the long status block was read."""
        vnow = self.raw.get("VNOW")
        if vnow is None:
            return None
        return round((vnow - (1 << 32) if vnow & 0x80000000 else vnow) * R.VELOCITY_SCALE, 2)

    @property
    def idle(self) -> bool:
        """Not moving, not paused, not homing: safe for relative decisions and non-volatile writes."""
        return not self.moving and not self.paused and not self.homing

    @property
    def ready(self) -> bool:
        """True when a move would be accepted (as far as the status bits tell)."""
        return (
            self.servo_on
            and self.homed
            and self.modbus_enabled
            and not self.alarm
            and not self.emergency_stop
            and not self.paused
            and not self.homing
        )

    @classmethod
    def from_registers(cls, words: Sequence[int]) -> Status:
        """Decode words read from 0x9000: 8 (PNOW..DSSE), 14 (..CNOW), 20 (..ZONS) or 22 (..SSSE, the default read).

        The raw DOPM and ZONS words are kept in ``raw``; the position markers are taken from ZONS
        only and are ``None`` when fewer than 20 words were read; POSS and SSSE need 22.
        """
        if len(words) < R.STATUS_BLOCK_MIN_WORDS:
            raise ValueError(f"need at least {R.STATUS_BLOCK_MIN_WORDS} words")
        alarm_code, dss1, dss2, dsse = words[2], words[5], words[6], words[7]
        raw = {
            "PNOW": R.from_u32_words(words[0], words[1]),
            "ALMC": alarm_code,
            "DIPM": words[3],
            "DOPM": words[4],
            "DSS1": dss1,
            "DSS2": dss2,
            "DSSE": dsse,
        }
        current: int | None = None
        vnow: float | None = None
        if len(words) >= 14:
            raw["STAT"] = R.from_u32_words(words[8], words[9])
            raw["VNOW"] = R.from_u32_words(words[10], words[11])
            raw["CNOW"] = R.from_u32_words(words[12], words[13])
            vnow = round(R.from_i32_words(words[10], words[11]) * R.VELOCITY_SCALE, 2)
            current = R.from_i32_words(words[12], words[13])
        if len(words) >= R.STATUS_BLOCK_WORDS:
            raw["DEVI"] = R.from_u32_words(words[14], words[15])
            raw["STIM"] = R.from_u32_words(words[16], words[17])
            raw["SIPM"] = words[18]
            raw["ZONS"] = words[19]
        if len(words) >= R.STATUS_READ_WORDS:
            raw["POSS"] = words[20]
            raw["SSSE"] = words[21]
        # Fault rule (alarm levels, alarms.alarm_is_fault): ALMH, or a code of level operation cancelled /
        # cold start. A message-level code (0x011-0x07F: maintenance / deviation / collision warnings) is a
        # warning (the axis keeps running) and never blocks motion. An unknown-level code without the
        # warning bit ALML is a fault (fail safe).
        alarm = alarm_is_fault(
            alarm_code, alarm_bit=bool(dss1 & R.DSS1_ALMH), warning_bit=bool(dss1 & R.DSS1_ALML)
        )
        servo_on = bool(dss1 & R.DSS1_SV)
        # "Settled" states in which PEND legitimately stays 0 fall back to the raw bits. A warning does not
        # settle the axis: it keeps running.
        settled = (
            not servo_on
            or bool(dss1 & (R.DSS1_PEND | R.DSS1_PSFL | R.DSS1_STP))
            or alarm
            or not dss1 & R.DSS1_HEND
        )
        moving = bool(dsse & (R.DSSE_MOVE | R.DSSE_GHMS)) or not settled
        if not moving:
            vnow = 0.0
        return cls(
            position_mm=round(R.from_i32_words(words[0], words[1]) * R.POSITION_SCALE, 2),
            alarm_code=alarm_code,
            servo_on=servo_on,
            homed=bool(dss1 & R.DSS1_HEND),
            in_position=bool(dss1 & R.DSS1_PEND),
            moving=moving,
            alarm=alarm,
            modbus_enabled=bool(dsse & R.DSSE_PMSS),
            emergency_stop=bool(dss1 & R.DSS1_EMGS),
            velocity_mm_s=vnow,
            current_ma=current,
            raw=raw,
        )

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict including the derived flags."""
        data = asdict(self)
        data["raw"] = dict(self.raw)
        data.update(
            warning=self.warning,
            alarm_level=self.alarm_level,
            move_bit=self.move_bit,
            homing=self.homing,
            paused=self.paused,
            target_reached=self.target_reached,
            push_missed=self.push_missed,
            pushing=self.pushing,
            push_contact=self.push_contact,
            safety_speed=self.safety_speed,
            at_backward_end=self.at_backward_end,
            at_forward_end=self.at_forward_end,
            at_middle=self.at_middle,
            vnow_mm_s=self.vnow_mm_s,
            idle=self.idle,
            ready=self.ready,
            position_complete_no=self.position_complete_no,
            cold_start_alarm=self.cold_start_alarm,
            rtc_in_use=self.rtc_in_use,
            drive_power_cut=self.drive_power_cut,
            emg_input=self.emg_input,
            enable_off=self.enable_off,
            auto_servo_off=self.auto_servo_off,
            nv_busy=self.nv_busy,
            manual_mode=self.manual_mode,
            battery_low=self.battery_low,
            excitation_done=self.excitation_done,
        )
        return data


def _at_rest_for_control(st: Status) -> bool:
    """At rest for taking Modbus control from PIO (PMSL ON, the action mode PC): ``idle``, or paused (DSS1 STP) with the
    servo off while standing still: no MOVE, no home return, no push, VNOW read and 0. A PCON in PIO mode with nothing
    on its PIO connector turns the servo off and shows STP at rest (the unwired *STP input; VERIFIED on the PCON-CB,
    2026-11-02); 1.0.1 taking control there: VERIFIED on the core (PCON-CB, 2026-11-02); through this function (the
    Python command line) UNVERIFIED (tested on the simulator). A pause with the servo on is a hold, for example a PLC
    keeping *STP on a move: taking control would cancel it, so it stays a refusal. A PLC pause with the servo off is
    still indistinguishable from an unwired *STP. Handing control back to PIO keeps ``idle``.
    The core's detail::at_rest_for_control."""
    if st.idle:
        return True
    return (st.paused and not st.servo_on and not st.moving and not st.homing and not st.pushing
            and st.raw.get("VNOW") == 0)


@dataclass(frozen=True)
class IndexHeader:
    """Index table header at 0x0000 (MODL, DQNT, LQNT, BFRD, BFTD)."""

    model_code: int
    words_per_record: int
    record_count: int
    rx_buffer: int
    tx_buffer: int


@dataclass(frozen=True)
class IndexCategory:
    """One 8-word category record of the index table (CI, MD, DQ, LQ, DA, IA, BA, MS)."""

    category_id: int
    model_code: int
    member_count: int
    list_count: int
    data_address: int
    id_list_address: int
    coil_base: int
    member_size: int


@dataclass(frozen=True)
class DeviceInfo:
    """What the EC says about itself. Fields are ``None`` when the EC did not answer them.

    Identity comes from three places: the Report Slave ID (FC11) or, when that is silent, CTCL (class 3
    item 1 at 0x0300; the same fields as FC11 on every unit tested) gives system / unit code / firmware
    (``identity_source``); the unit code gives the EC **family** (``unit_name``: e.g. 0x0A =
    'EC(MS) "2-size EC"', ``unit_known`` True when the driver has a family name for the code; every unit is driven
    the same way; the EC-S6HH-200 slider reports code 0x00); **AKID** (item 7.33) gives the actuator kind
    (``actuator_kind`` / ``actuator_kind_name``: gripper, rod, slider, rotary ...); the 0x0600 block gives the model
    string (``model_name``, :attr:`model`), the controller / actuator serials and ``user_name`` (ANA1-3 at
    0x061A, a name the user can edit in IA-OS, not a model field; ``short_model`` is a compatibility alias).
    ``axis_number`` is AXNO (0x8004, read-only; axis = slave address - 1). ``soft_limits_mm`` is (min, max)
    from 0x0700; ``max_speed_mm_s`` is the EC's maximum speed MAXV (item 7.4; ``speed_limit_source`` says
    where it came from: 0x0812 = the default speed, item 8.10 VCMD, is only the fallback), ``max_accel_g`` /
    ``max_decel_g`` MAXA / MAXD. ``product_family`` is the family of the FC11 system code (``"EC"``, ``"PCON"``,
    ``"ACON/DCON"``, ``"SCON"``, else ``"IAI controller"``); ``controller_name`` the CON controller's name block
    (0x8010, e.g. ``"PCON-CB (FB)"``); ``units`` ``"mm"`` or ``"deg"`` (rotary, AKID 1); ``max_positions`` the stored
    rows the unit uses (MAXP 3.2, else the directory). Every string is printable ASCII; ``unit_known`` is
    display only (no command depends on it).
    """

    slave: int
    system_code: int | None = None
    system_name: str | None = None
    run_status: int | None = None
    unit_code: int | None = None
    unit_name: str | None = None
    firmware: str | None = None
    slave_id_raw: bytes | None = None
    index_header: IndexHeader | None = None
    categories: tuple[IndexCategory, ...] = ()
    serial_number: str | None = None
    user_name: str | None = None
    model_name: str | None = None
    soft_limits_mm: tuple[float, float] | None = None
    max_speed_mm_s: float | None = None
    errors: tuple[str, ...] = ()
    identity_source: str | None = None  # "FC11" or "CTCL 0x0300" (FC11 not answered)
    unit_known: bool | None = None  # the driver has a family name for this (system, unit) code (display only)
    actuator_kind: int | None = None  # AKID item 7.33 bits 0-7
    actuator_kind_name: str | None = None
    rotary_atyp: bool | None = None  # ATYP item 7.7 bit 8 (rotary on units without AKID, e.g. CON controllers)
    axis_number: int | None = None  # AXNO 0x8004
    actuator_serial: str | None = None  # ACS1-4 0x0608
    accl: int | None = None  # ACCL 0x0612 (bits 8-11 = extensibility level)
    max_accel_g: float | None = None  # MAXA item 7.5
    max_decel_g: float | None = None  # MAXD item 7.6
    speed_limit_source: str | None = None
    product_family: str | None = None
    controller_name: str | None = None
    units: str = "mm"
    max_positions: int | None = None

    @property
    def kind(self) -> str:
        """The actuator kind in words (``"slider"``, ``"rotary"`` ... ``"unknown"``)."""
        return R.axis_kind_name(self.actuator_kind, self.rotary_atyp)

    @property
    def short_model(self) -> str | None:
        """Alias of :attr:`user_name`, kept for compatibility (0x061A ANA1-3 is a user-defined unit name)."""
        return self.user_name

    @property
    def family(self) -> str | None:
        """EC family name of the unit code (same as ``unit_name``)."""
        return self.unit_name

    @property
    def extensibility_level(self) -> int | None:
        """ACCL bits 8-11 (0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo)."""
        return None if self.accl is None else (self.accl >> 8) & 0xF

    @property
    def rotary(self) -> bool | None:
        """AKID kind 1: a rotary axis (positions in deg, speeds in deg/s). It is
        commanded like every other axis. Without AKID: ATYP 7.7 bit 8."""
        if self.actuator_kind is None:
            return self.rotary_atyp
        return self.actuator_kind == R.AKID_ROTARY

    @property
    def controller(self) -> str:
        """Report Slave ID in words, e.g. ``'EC 24 V, unit 0x0A (EC(MS) "2-size EC")'`` or ``"EC 24 V, unit 0x05"``."""
        if self.system_code is None:
            return "unknown"
        name = self.system_name or f"system code {self.system_code:#04x}"
        if self.unit_code is not None:
            name += f", unit 0x{self.unit_code:02X}"
            if self.unit_name:
                name += f" ({self.unit_name})"
        return name

    @property
    def actuator(self) -> str | None:
        """Actuator kind from AKID in words, e.g. ``"gripper (AKID 4)"``; ``None`` if AKID was not read."""
        if self.actuator_kind is None:
            return None
        return f"{self.actuator_kind_name or 'unknown kind'} (AKID {self.actuator_kind})"

    @property
    def model(self) -> str:
        """Display name (printable ASCII): the 0x0600 model string if the unit has one (EC), else the controller
        name block (CON, e.g. ``"PCON-CB (FB)"``), else :attr:`controller`."""
        return R.printable(self.model_name or self.controller_name or self.controller)

    def category(self, category_id: int) -> IndexCategory | None:
        """Return the index record for ``category_id``, if present."""
        return next((c for c in self.categories if c.category_id == category_id), None)

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        data = asdict(self)
        data["slave_id_raw"] = self.slave_id_raw.hex(" ") if self.slave_id_raw is not None else None
        data["soft_limits_mm"] = list(self.soft_limits_mm) if self.soft_limits_mm is not None else None
        data.update(
            model=self.model, controller=self.controller, short_model=self.short_model, family=self.family,
            actuator=self.actuator, extensibility_level=self.extensibility_level, rotary=self.rotary, kind=self.kind,
        )
        return data


@dataclass(frozen=True)
class AxisLimits:
    """The EC's own per-model limits, read from class 7 by item ID (no per-model table in the driver).

    ``max_speed_mm_s`` MAXV 7.4, ``max_accel_g`` MAXA 7.5, ``max_decel_g`` MAXD 7.6, ``max_push_speed_mm_s``
    MXPH 7.8, ``max_push_pct`` MXPP 7.9 (the driver's hard push cap; 0 = no push), ``min_push_pct`` MNPP 7.58
    or 20 when absent (only a recommendation: below it the driver warns), ``push_force_factor`` /
    ``push_force_offset`` TCPC / TCOA 7.34 / 7.35 (0 factor = no push), ``actuator_kind`` AKID 7.33 bits 0-7,
    ``default_speed_mm_s`` the default speed 0x0812 (item 8.10 VCMD), read only as the speed fallback.
    ``supports_push`` / ``supports_relative`` are SUPT b11 / b8 (class 1; ``None`` if not read). Read on the EC-R6S
    rod: MAXV 800.00 mm/s, MAXA 1.00 G (VERIFIED); the meaning of the push items is UNVERIFIED.
    """

    max_speed_mm_s: float | None = None
    max_accel_g: float | None = None
    max_decel_g: float | None = None
    max_push_speed_mm_s: float | None = None
    max_push_pct: float | None = None
    min_push_pct: float = R.MNPP_DEFAULT_PCT
    min_push_from_ec: bool = False  # MNPP was read (else the default 20 %)
    push_force_factor: int | None = None
    push_force_offset: int | None = None
    actuator_kind: int | None = None
    rotary_atyp: bool | None = None  # ATYP 7.7 bit 8: rotary on a unit without AKID (CON controllers; UNVERIFIED)
    default_speed_mm_s: float | None = None
    supports_push: bool | None = None
    supports_relative: bool | None = None
    source: str = ""
    max_positions: int | None = None  # stored rows the unit uses (MAXP 3.2, else the directory's row count)
    min_band: float | None = None  # MINI 3.67 when the unit enforces it (SUP2 CINP and ATYP RINP set), else None
    lead_mm: float | None = None  # LEAD 7.3 (screw lead, 0.01 mm; CON No.77), for the minimum-speed recommendation
    move_patterns: int | None = None  # MVPT (class 1): b1 SCUR S-motion, b2 PFIL filter, b8 ASOF auto servo off

    @property
    def units(self) -> str:
        """``"deg"`` on a rotary axis (AKID 1; without AKID, ATYP 7.7 bit 8), else ``"mm"``."""
        return R.axis_units(self.actuator_kind, self.rotary_atyp)

    @property
    def kind(self) -> str:
        return R.axis_kind_name(self.actuator_kind, self.rotary_atyp)

    @property
    def push_available(self) -> bool | None:
        """Push is possible: SUPT PUSH not 0, MXPP > 0 and TCPC > 0 (UNVERIFIED rule; every unit tested can push).
        ``None`` if neither was read."""
        if self.max_push_pct is None and self.push_force_factor is None and self.supports_push is None:
            return None
        return not (
            self.supports_push is False or self.max_push_pct == 0 or self.push_force_factor == 0
        )

    @property
    def rotary(self) -> bool | None:
        if self.actuator_kind is None:
            return self.rotary_atyp
        return self.actuator_kind == R.AKID_ROTARY

    def push_force_estimate(self, pct: float) -> float | None:
        """Push force estimate TCPC x % / MXPP + TCOA (unit probably N; UNVERIFIED)."""
        if not self.max_push_pct or self.push_force_factor is None:
            return None
        return round(self.push_force_factor * pct / self.max_push_pct + (self.push_force_offset or 0), 1)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data.update(push_available=self.push_available, rotary=self.rotary, units=self.units, kind=self.kind)
        return data


@dataclass(frozen=True)
class PositionEntry:
    """One position-table entry (16 words at 0x1000 + 16 * n). Units: mm, mm/s, G.

    To change an entry, start from what the EC holds so the control flags (and the unknown
    0x0010 / 0x0020 role bits of rows 1 and 2) are kept::

        entry = dataclasses.replace(ec.read_position_table(2), position_mm=60.0)
    """

    position_mm: float
    band_mm: float = 0.10
    speed_mm_s: float = 50.0
    zone_plus_mm: float = 0.0
    zone_minus_mm: float = 0.0
    accel_g: float = 0.30
    decel_g: float = 0.30
    push_power: int = 0
    load_threshold: int = 0
    control_flags: int = 0
    checksum: int | None = None  # word +0xF as read (byte-sum checksum of the entry)
    # write_position_table / teach_position: the row already held this data, nothing was written (NV_SKIP_NOTE)
    skipped: bool = field(default=False, compare=False)

    @property
    def checksum_ok(self) -> bool | None:
        """Whether the read checksum matches the entry's data (``None`` if no checksum was read)."""
        if self.checksum is None:
            return None
        return self.checksum == R.position_checksum(self.to_registers())

    @classmethod
    def from_registers(cls, words: Sequence[int]) -> PositionEntry:
        """Decode 15 or 16 words."""
        if len(words) < R.POSITION_WORDS_WRITABLE:
            raise ValueError(f"need at least {R.POSITION_WORDS_WRITABLE} words")
        return cls(
            position_mm=round(R.from_i32_words(words[0], words[1]) * R.POSITION_SCALE, 2),
            band_mm=round(R.from_u32_words(words[2], words[3]) * R.POSITION_SCALE, 2),
            speed_mm_s=round(R.from_u32_words(words[4], words[5]) * R.VELOCITY_SCALE, 2),
            zone_plus_mm=round(R.from_i32_words(words[6], words[7]) * R.POSITION_SCALE, 2),
            zone_minus_mm=round(R.from_i32_words(words[8], words[9]) * R.POSITION_SCALE, 2),
            accel_g=round(words[10] * R.ACCEL_SCALE, 2),
            decel_g=round(words[11] * R.ACCEL_SCALE, 2),
            push_power=words[12],
            load_threshold=words[13],
            control_flags=words[14],
            checksum=words[15] if len(words) > 15 else None,
        )

    def to_registers(self) -> list[int]:
        """Encode the 15 writable words (+0x0..+0xE). Raises ValueError for unencodable values."""
        for name in ("push_power", "load_threshold", "control_flags"):
            value = getattr(self, name)
            if not isinstance(value, int) or not 0 <= value <= 0xFFFF:
                raise ValueError(f"{name} must be an int in 0..65535")
        accel, decel = R.to_counts(self.accel_g, R.ACCEL_SCALE), R.to_counts(self.decel_g, R.ACCEL_SCALE)
        if not 0 <= accel <= 0xFFFF or not 0 <= decel <= 0xFFFF:
            raise ValueError("accel/decel out of range")
        return [
            *R.to_i32_words(R.to_counts(self.position_mm)),
            *R.to_u32_words(R.to_counts(self.band_mm)),
            *R.to_u32_words(R.to_counts(self.speed_mm_s, R.VELOCITY_SCALE)),
            *R.to_i32_words(R.to_counts(self.zone_plus_mm)),
            *R.to_i32_words(R.to_counts(self.zone_minus_mm)),
            accel,
            decel,
            self.push_power,
            self.load_threshold,
            self.control_flags,
        ]


@dataclass(frozen=True)
class PushResult:
    """Outcome of :meth:`Elecylinder.push`: ``"contact"`` (holding against the work) or ``"missed"``.

    ``warnings``: e.g. a force below the EC's recommended minimum (MNPP, or 20 %), which is allowed."""

    outcome: str
    position_mm: float
    status: Status
    warnings: tuple[str, ...] = ()

    @property
    def contact(self) -> bool:
        """The push met the work (PEND and DSSE bit 0); the motor keeps pushing until the next move."""
        return self.outcome == "contact"

    @property
    def missed(self) -> bool:
        """The push went over the whole band without contact (DSS1 PSFL)."""
        return self.outcome == "missed"

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return {"outcome": self.outcome, "position_mm": self.position_mm, "status": self.status.to_dict(),
                "warnings": list(self.warnings)}


@dataclass(frozen=True)
class ParameterWriteResult:
    """Outcome of :meth:`Elecylinder.write_user_parameter` (values as IA-OS shows them)."""

    no: int
    name: str
    key: str  # class.item, e.g. "8.9"
    address: str  # where the item was found on this EC (by item ID), e.g. "0x0810"
    unit: str
    before: str
    after: str
    value: Any
    raw_before: str
    raw_after: str
    changed: bool  # the item's raw value changed (an unchanged one is committed only to save earlier RAM writes, or forced)
    verified: bool  # read back equal to what was written
    committed: bool  # the PARW coil 0x0428 was sent and the handshake ended idle (DRGE b7 = 0, DSSE b7 = 1)
    listed_for_model: bool  # the parameter is in this unit's parameter list
    verified_for_model: bool  # kept for compatibility: an IA-OS parameter list is recorded for the unit code
    home_parameter: bool  # P3 / P4
    warnings: tuple[str, ...] = ()
    note: str = ""
    commit_confirmed: bool = False  # the EC was seen acting on the coil (DRGE b7 read 1 or DSSE b7 read 0 after it)
    # The unit already held the value and no parameter this process wrote to the unit's RAM waits for a commit:
    # nothing written, nothing committed
    skipped: bool = False
    # The unit already held the value, but ``force=True`` wrote / committed it anyway (one non-volatile write cycle)
    forced: bool = False

    @property
    def saved(self) -> bool:
        """The value is in non-volatile memory as far as the EC showed: committed AND the commit was seen, or
        ``skipped`` (the unit already held it and nothing this process wrote is uncommitted), ASSUMING no RAM-only
        write by another program or process (an earlier CLI run with ``--no-commit``: write again with
        ``force=True``)."""
        return self.skipped or (self.committed and self.commit_confirmed)

    @property
    def needs_reset(self) -> bool:
        """True when this write left a value waiting for a software reset or a power cycle (the EC applies a
        parameter only then). False for a skipped write: nothing was written or committed, so there is nothing new to
        apply (a value committed by an earlier call may still wait for its reset: ``takes_effect`` says so)."""
        return not self.skipped

    @property
    def takes_effect(self) -> str:
        """When the new value applies, in words."""
        if self.skipped:
            return ("nothing new: the unit already holds this value (a value committed earlier applies after a "
                    "software reset or a power cycle; one left in RAM only by another program or process is NOT "
                    "saved: write it with force=True)")
        if self.committed and not self.commit_confirmed:
            return (
                "after a software reset or a power cycle IF the commit took; it was NOT confirmed (the EC never "
                "showed PARW busy): check the value after a power cycle"
            )
        if self.committed:
            return "after a software reset (software_reset()) or a power cycle"
        return "RAM only: lost at the next power-up (probably at a software reset too); untested before then"

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        data = asdict(self)
        data.update(warnings=list(self.warnings), saved=self.saved, needs_reset=self.needs_reset,
                    takes_effect=self.takes_effect)
        return data


@dataclass(frozen=True)
class SoftwareResetResult:
    """Outcome of :meth:`Elecylinder.software_reset`: the status once the EC answered again."""

    accepted: bool  # the EC answered the reset command with rejection 0 (01 45 00 00)
    reply_lost: bool  # no (complete) answer to the reset command; the reset was confirmed from the status
    baudrate: int | None  # the rate the EC answered at after the reset (None: transport without one)
    baud_changed: bool
    status: Status  # first status after the reset (servo on / homed / PMSS / SFTY / alarm)
    uptime_before_s: float | None
    uptime_after_s: float | None
    elapsed_s: float
    notes: tuple[str, ...] = ()
    confirmed: bool = True  # the restart was seen (the servo back on by itself, PMSS 1 -> 0, a reply gap, ...)
    confirmed_by: str = ""  # how, in words

    @property
    def modbus_enabled(self) -> bool:
        """PMSS after the reset. It depends on the unit: the EC-R6S rod dropped it to 0 (like a power-up: PIO has
        control again), the EC-GRBP8M gripper and the PCON-CB kept 1 (VERIFIED)."""
        return self.status.modbus_enabled

    @property
    def safety_speed(self) -> bool:
        return self.status.safety_speed

    @property
    def servo_on(self) -> bool:
        return self.status.servo_on

    @property
    def homed(self) -> bool:
        return self.status.homed

    @property
    def needs_home(self) -> bool:
        """HEND is 0 after the reset: an incremental unit lost its home (as the EC-R6S rod does), or an absolute
        unit raised ABS error 0x0EE after a home-parameter change. Run home() (after reset_alarm() if an alarm is
        active)."""
        return not self.status.homed

    @property
    def alarm_code(self) -> int:
        return self.status.alarm_code

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return {
            "accepted": self.accepted, "reply_lost": self.reply_lost, "baudrate": self.baudrate,
            "baud_changed": self.baud_changed, "modbus_enabled": self.modbus_enabled,
            "safety_speed": self.safety_speed, "servo_on": self.servo_on, "homed": self.homed,
            "needs_home": self.needs_home, "alarm_code": self.alarm_code, "uptime_before_s": self.uptime_before_s,
            "uptime_after_s": self.uptime_after_s, "elapsed_s": self.elapsed_s, "notes": list(self.notes),
            "confirmed": self.confirmed, "confirmed_by": self.confirmed_by, "status": self.status.to_dict(),
        }


@dataclass(frozen=True)
class BaudChangeResult:
    """Outcome of :meth:`Elecylinder.set_baudrate`."""

    previous: int
    baudrate: int  # the rate the port is at now and the EC answered at
    method: str  # "direct" or "break"
    code: int  # op1 of the broadcast (IAI baud code)
    frame: str  # the broadcast frame sent, hex
    verified: bool  # the first read at the new rate was answered
    probed: bool  # that read failed and the rate was found by probing instead
    note: str = BAUD_VOLATILE_NOTE
    code_verified: bool = True  # the rate's code is verified on hardware (modbus.VERIFIED_BAUD_RATES)

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)


@dataclass(frozen=True)
class AlarmRecord:
    """One alarm-history entry (0x0500 + 6 * index). Index 0 is the newest (consistent with the time stamps
    read on hardware).

    ``time_raw`` is the time stamp in **milliseconds since the EC was powered on** (the EC has no clock; the
    unit fits the values read on hardware), ``None`` when the EC stores FFFFFFFF. Detail 0000 / address FFFF
    mean "none"."""

    index: int
    code: int
    detail: int
    address: int
    time_raw: int | None  # ms since power-on; None when the EC stores FFFFFFFF

    @property
    def name(self) -> str:
        """Alarm name (``alarms.alarm_name``) or a hex code."""
        return alarm_name(self.code)

    @property
    def power_up(self) -> bool:
        """The 0x0FFF "PowerUP No Error" marker the EC writes at every power-up."""
        return self.code == R.ALARM_POWER_UP

    @property
    def level(self) -> str:
        """The level of the code (``alarms.alarm_level``: message / operation_cancelled / cold_start ...)."""
        return alarm_level(self.code)

    @property
    def time_after_power_on(self) -> str | None:
        """The time stamp as ``[d days ]hh:mm:ss.mmm`` after power-on (``None`` without one)."""
        if self.time_raw is None:
            return None
        seconds, ms = divmod(self.time_raw, 1000)
        days, rest = divmod(seconds, 86400)
        hours, rest = divmod(rest, 3600)
        minutes, secs = divmod(rest, 60)
        return (f"{days} d " if days else "") + f"{hours:02d}:{minutes:02d}:{secs:02d}.{ms:03d}"

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        data = asdict(self)
        data.update(name=self.name, power_up=self.power_up, level=self.level, time_ms=self.time_raw,
                    time_after_power_on=self.time_after_power_on)
        return data
