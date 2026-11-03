"""The Python part of ``iai_actuator_driver.sim``: the unit profiles (:data:`PROFILES`, built on
:mod:`iai_actuator_driver.sim_fixtures`), the alarm and rejection codes the simulator raises, the fault kinds and the
``.bkec`` backup reader.

``iai_actuator_driver.sim`` (the core's simulator, where its behaviour is documented) imports these names from here.
"""

from __future__ import annotations

import os

from .. import registers as R
from .. import sim_fixtures
from . import publish

# Identification block 0x0600-0x0647 as read live, with the serial number replaced by a placeholder.
_EMBEDDED_IDENTIFICATION = (
    "53494D2D30303030303031202020202030202020202020202020202020202020304D3132000101102025092600000018"
    "000000004D57422D45432D44535252344D57422D45432D44535252342D57412D4D462020202020202020202020202020"
    "202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020"
)

# Index table of the EC gripper (EC-GRBP8M-20, 25 records) as read live.
_EMBEDDED_GRIPPER_INDEX_TABLE = (
    "0001000800190100010000000000000000010100000B0001010000000000000000020002000A00010200028001000001"
    "00030004003A000103000380000000020004000200130001040004800000000200050002000300200500580000000002"
    "000600010024000106000000000000000007010200160001070051000000000200080002002200010800520000000002"
    "00090002002A00010900530000000002000C0000000200020C00560000000002000D0002000A00010D000D8004000001"
    "000F0100000D00010F000000000000000010010100080300100050000000000200120002000500017F00000000000000"
    "010000020010000180000000000000000013000000100001810000000000000000140000001000018200000000000000"
    "020000000017000184008480000000020101000300160001900000000000000001090001000E00019100000000000000"
    "010200010003000198000000000000000103000100070001990000000000000001040000000300019A00000000000000"
    "010A0000000600019B0000000000000001110000000500019D00000000000000"
)

# Identification block of the gripper as read live, serial number replaced by a placeholder.
_EMBEDDED_GRIPPER_IDENTIFICATION = (
    "53494D2D30303030303032202020202053494D2D30303030303032202020202030413031000000012024112600000709"
    "0000000045432D47524250384D2D323045432D47524250384D2D32302D312D312D504E2D57412D574C32202020202020"
    "202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020"
)


# Position table rows 0-3 exactly as read live / found in the backup (16 words each).
_SEED_POSITIONS: dict[int, tuple[int, ...]] = {
    0: (0, 0, 0, 10, 0, 0x2A62, 0, 0, 0, 0, 0x1E, 0x1E, 0, 0, 0x0000, 0x002E),  # 0.00 mm, 108.50 mm/s
    1: (0, 0x3A98, 0, 10, 0, 0x6112, 0, 0, 0, 0, 0x1E, 0x1E, 0, 0, 0x0010, 0x0065),  # 150.00 mm, 248.50 mm/s
    2: (0, 0x1D4C, 0, 10, 0, 0x6112, 0, 0, 0, 0, 0x1E, 0x1E, 0, 0, 0x0020, 0x00BE),  # 75.00 mm, 248.50 mm/s
    3: (0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0x1E, 0x1E, 0xB2, 0, 0x0000, 0x0008),
}

# Gripper position rows 0-3 as read live.
_SEED_GRIPPER_POSITIONS: dict[int, tuple[int, ...]] = {
    0: (0, 300, 0, 10, 0, 5300, 0, 0, 0, 0, 30, 30, 0, 0, 0x0400, 0x00C1),  # 3.00 mm (open), 53.00 mm/s
    1: (0, 410, 0, 540, 0, 5300, 0, 0, 0, 0, 30, 30, 128, 0, 0x0416, 0x00A9),  # 4.10 mm + 5.40 mm push 50 % (+)
    2: (0, 250, 0, 10, 0, 5300, 0, 0, 0, 0, 30, 30, 0, 0, 0x0400, 0x00F4),  # 2.50 mm (unused in 2-position mode)
    3: (0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 30, 30, 178, 0, 0x0000, 0x0008),
}
# Defaults shared by the EC profiles (the CON profile overrides them).
_EC_DEFAULTS: dict[str, object] = {
    "system_code": 0xEC, "akid": None, "blank_row": None, "con_reset": False, "rtc_alarm": False,
}  # fmt: skip
# Rows of the simulator-only profiles: 0.00 and 90.00 (deg on the rotary) at 100.00 / s, 0.30 G (checksums computed).
_SIM_ROW0 = (0, 0, 0, 10, 0, 10000, 0, 0, 0, 0, 30, 30, 0, 0, 0x0000)
_SIM_ROW1 = (0, 9000, 0, 10, 0, 10000, 0, 0, 0, 0, 30, 30, 0, 0, 0x0000)
_SIM_ROW0 = (*_SIM_ROW0, R.position_checksum(_SIM_ROW0))
_SIM_ROW1 = (*_SIM_ROW1, R.position_checksum(_SIM_ROW1))

# Per-unit defaults selected with ``SimulatedEC(profile=...)``; explicit arguments override them.
PROFILES: dict[str, dict[str, object]] = {
    "slider": {  # EC-DS4H-150-MOT-WA-WL2, 3-position mode (tested on hardware)
        "stroke_mm": 150.0,
        "position_mm": 73.95,
        "max_speed_mm_s": 700.0,
        "unit_code": 0x02,
        "firmware": (0x00, 0x13),
        "dopm_layout": "3pos",
        "index_table": None,  # the baseline backup, or its embedded copy
        "identification": _EMBEDDED_IDENTIFICATION,
        "positions": _SEED_POSITIONS,
        "ssse": 0x1008,  # live SSSE 0x9015
        "ollv": 0x12,  # live OLLV 0x9020-21
        "absolute": True,  # WA: battery-less absolute encoder
        "slave_id_delay_s": 0.0,
    **_EC_DEFAULTS,
    },
    "gripper": {  # EC-GRBP8M-20-1-1-PN-WA-WL2, 2-position mode (tested on hardware)
        "stroke_mm": 9.5,
        "position_mm": 9.5,
        "max_speed_mm_s": 53.0,
        "unit_code": 0x0A,
        "firmware": (0x00, 0x02),
        "dopm_layout": "2pos",
        "index_table": _EMBEDDED_GRIPPER_INDEX_TABLE,
        "identification": _EMBEDDED_GRIPPER_IDENTIFICATION,
        "positions": _SEED_GRIPPER_POSITIONS,
        "ssse": 0x0000,  # live SSSE 0x9015
        "ollv": 0x00,  # not read on the gripper
        "absolute": True,  # WA
        "reset_keeps_pmss": True,  # VERIFIED: PMSS stayed 1 across a software reset (2026-10-29)
        "slave_id_delay_s": 0.0,
    **_EC_DEFAULTS,
    },
    "rod": {  # EC-R6S-150-1-PN (tested on hardware): no WA option = incremental encoder
        "stroke_mm": 150.0,
        "position_mm": 0.0,
        "max_speed_mm_s": 800.0,
        "unit_code": 0x00,
        "firmware": (0x00, 0x02),
        "dopm_layout": "2pos",  # live DOPM 0x0005 at 0.00 mm: b0 LS0, b2 *ALM
        "index_table": sim_fixtures.ROD_INDEX_TABLE,
        "identification": sim_fixtures.ROD_IDENTIFICATION,
        "positions": {n: row for n, row in sim_fixtures.ROD_POSITIONS.items() if n < 4},
        "ssse": 0x0000,
        "ollv": 0x00,
        "absolute": False,  # loses HEND at a software reset (re-homed on hardware with iai-actuator-driver home)
        "slave_id_delay_s": 0.3,  # FC11 answered after 0.2-0.4 s, not within 0.2 s
        **_EC_DEFAULTS,
    },
    "pcon": {  # PCON-CB + RCP6-WSA10R (tested on hardware): CON-family controller, same protocol
        "stroke_mm": 150.0,
        "position_mm": 60.0,
        "max_speed_mm_s": 840.0,
        "unit_code": 0x44,
        "firmware": (0x00, 0x00),
        "dopm_layout": "2pos",
        "index_table": sim_fixtures.PCON_INDEX_TABLE,
        "identification": sim_fixtures.PCON_IDENTIFICATION,
        "positions": sim_fixtures.PCON_POSITIONS,
        "ssse": 0x0000,
        "ollv": 0x00,
        "absolute": True,  # HEND 1 after power-up
        "slave_id_delay_s": 0.0,
        "system_code": 0xAE,
        "akid": None,
        "blank_row": sim_fixtures.PCON_BLANK_ROW,
        "con_reset": True,
        "rtc_alarm": True,
    },
    "rotary": {  # simulator-only: an EC rotary axis (AKID 1), positions in degrees
        "stroke_mm": 360.0,
        "position_mm": 0.0,
        "max_speed_mm_s": 600.0,
        "unit_code": 0x00,
        "firmware": (0x00, 0x02),
        "dopm_layout": "2pos",
        "index_table": sim_fixtures.ROD_INDEX_TABLE,
        "identification": sim_fixtures.ROTARY_IDENTIFICATION,
        "positions": {0: _SIM_ROW0, 1: _SIM_ROW1},
        "ssse": 0x0000,
        "ollv": 0x00,
        "absolute": True,
        "slave_id_delay_s": 0.0,
        **_EC_DEFAULTS,
        "akid": R.AKID_ROTARY,
    },
    "unknown": {  # simulator-only: a unit whose system / unit code and AKID kind have no name in the driver
        "stroke_mm": 100.0,
        "position_mm": 0.0,
        "max_speed_mm_s": 400.0,
        "unit_code": 0x7E,
        "firmware": (0x01, 0x00),
        "dopm_layout": "2pos",
        "index_table": sim_fixtures.ROD_INDEX_TABLE,
        "identification": sim_fixtures.UNKNOWN_IDENTIFICATION,
        "positions": {0: _SIM_ROW0, 1: _SIM_ROW1},
        "ssse": 0x0000,
        "ollv": 0x00,
        "absolute": True,
        "slave_id_delay_s": 0.0,
        **_EC_DEFAULTS,
        "system_code": 0x99,
        "akid": 42,
    },
}
DOPM_LAYOUTS = ("3pos", "2pos")

# Alarm codes the simulator raises (names from R.ALARM_NAMES). Levels follow alarms.alarm_level:
# 0x011-0x07F are messages (ALML, the servo stays on, the axis keeps running); operation-cancelled and
# cold-start codes (every code from 0x080 up) set ALMH and switch the servo off.
ALARM_SERVO_OFF_MOVE = 0x080  # "Motion command while in SERVO-OFF condition"
ALARM_NOT_HOMED_MOVE = 0x083  # "Absolute positional movement command when Homing is not completed"
ALARM_MOVE_WHILE_HOMING = 0x084  # "Motion command during Homing"
ALARM_POSITION_NO = 0x085  # "Position No. error when moving"
ALARM_BAD_BAND = 0x0A3  # "Positional instruction information error when executing"
ALARM_BAD_DECEL = 0x0A7  # "The instructed deceleration is abnormal."
ALARM_BAD_SPEED = 0x0AB  # "The instructed speed is abnormal."
ALARM_SOFT_LIMIT = 0x0D9  # "Software stroke limit over error"
ALARM_RESET_SERVO_ON = 0x090  # "Software reset command while in SERVO-ON condition" (EC-GRBP8M: major, servo off)
ALARM_ABS_ERROR = 0x0EE  # "ABS error (2)": home lost (gripper after a committed P4 change + software reset)
ALARM_TEACH_NOT_HOMED = 0x093  # TEAC held before homing (DOC ME0162-12A p.5-99)
REJECT_COMMAND_CODE = 0x21  # "Command code error"
REJECT_MONITOR_MODE = 0x3A  # "Prohibition of command in monitor mode"
REJECT_OPERAND1 = 0x11  # "1st operand error": 0x45 home with op1 = 0, and 0x45 cmd 6 with row 2 in 2-position mode
#                         (both VERIFIED on the EC-GRBP8M gripper)
REJECT_SERVO_OFF = 0x23  # "Motion command while servo OFF"
REJECT_HOMING = 0x24  # "Motion command during homing"
FAULT_KINDS = ("timeout", "crc", "partial", "wrong_slave")
PARAMETER_RAM_CLASSES = (0x0007, 0x0008, 0x0009, 0x000C)  # writable parameter classes (RAM until PARW commits them)
HOME_PARAMETER_ITEMS = ((0x0009, 16), (0x0009, 21))  # P3 home direction, P4 home position adjustment


def load_bkec_registers(path: str | os.PathLike[str], max_lines: int = 0x100) -> dict[int, int]:
    """Parse the first ``max_lines`` ``AAAA,VVVV`` data lines of an IA-OS ``.bkec`` backup (read-only)."""
    regs: dict[int, int] = {}
    with open(path, encoding="ascii", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith(";"):
                continue
            addr_s, _, value_s = line.partition(",")
            regs[int(addr_s, 16)] = int(value_s, 16)
            if len(regs) >= max_lines:
                break
    return regs


publish(globals(), "sim")
