"""ELECYLINDER register map: addresses, bit masks and scaling.

Every entry carries an evidence level (see README):

* ``VERIFIED-LIVE`` / ``VERIFIED`` - read (or written) on a real EC and confirmed (EC-DS4H-150 slider,
  EC-GRBP8M-20 gripper and EC-R6S-150 rod).
* ``ON-WIRE``       - observed on the serial line (USB capture of IA-OS talking to an EC).
* ``BACKUP``        - present in the EC's own backup file (.bkec, written by IA-OS), i.e. the EC exposes it.
* ``DOC`` / ``DOC-CON`` - public IAI manuals; DOC-CON = documented only for CON-family controllers (ME0162).
* ``UNVERIFIED``    - expected behaviour (IAI's register / command numbering), not yet confirmed on hardware.

Verified on hardware: the full 9-word direct move block (absolute, incremental and push moves),
stored-position moves, the SON / ALRS / STP / STOP / HOME / PMSL / SFTY coils, FC11, the index table and the
ZONS position markers, on two units (EC-DS4H-150 slider, EC-GRBP8M-20 gripper), plus a third (EC-R6S-150
rod). Position-table writes reach non-volatile memory (a written row survived a 24 V power cycle). A USB
capture of IA-OS (ON-WIRE) showed the parameter commit (coil 0x0428 PARW), the software reset (0x45 cmd 5
op1 1 after servo OFF, 0x45 cmd 9) and the broadcast baud change (0x45 cmd 4); the driver's own replay of the
commit + reset frames was accepted. These 0x45 command forms were verified on the EC-R6S rod: home 0x45 cmd
0x0A with op1 = ORG (item 9.16), alarm reset 0x45 cmd 5 op1 0 and action mode 0x45 cmd 0x0C. The 0x48
commands are UNVERIFIED.
"""

from __future__ import annotations

import math
from typing import Final

# --------------------------------------------------------------------------- scaling

POSITION_SCALE: Final = 0.01  # mm per count (PNOW, PCMD, INP, zones). VERIFIED-LIVE (0x1CE3 = 73.95 mm)
VELOCITY_SCALE: Final = 0.01  # mm/s per count (VCMD; VNOW too, see VNOW). VERIFIED-LIVE (10 mm/s measured ~9.8)
ACCEL_SCALE: Final = 0.01  # G per count (ACMD, DCMD). VERIFIED-LIVE readback (0x001E = 0.30 G) / DOC-CON
G_MM_S2: Final = 9800.0  # 1 G in mm/s^2 as IAI defines it (ME3816)
ACMD_MAX_RAW: Final = 300  # 3.00 G, upper limit of ACMD (DOC ME0162-12A p.5-132)

# --------------------------------------------------------------------------- index table (self-description)

INDEX_HEADER: Final = 0x0000  # MODL, DQNT, LQNT, BFRD, BFTD, RSV x3. VERIFIED-LIVE readable, BACKUP
INDEX_HEADER_WORDS: Final = 8
INDEX_RECORDS: Final = 0x0008  # LQNT records of DQNT words: CI MD DQ LQ DA IA BA MS. VERIFIED-LIVE, BACKUP

# Category ids (CI) in the index table and the base address (DA) the driver hard-codes for each.
# The `check` self-test compares these with what the connected EC reports. VERIFIED-LIVE, BACKUP
CATEGORY_ALARM_HISTORY: Final = 0x0005  # DA 0x0500
CATEGORY_IDENTIFICATION: Final = 0x0006  # DA 0x0600
CATEGORY_LIMIT_PARAMS: Final = 0x0007  # DA 0x0700 (soft limits first)
CATEGORY_MOTION_PARAMS: Final = 0x0008  # DA 0x0800 (0x9900 defaults: INP, VCMD = default/max speed, ACMD)
CATEGORY_IO_CONTROL: Final = 0x000D  # DA 0x0D00, coil base BA 0x0400
CATEGORY_POSITION_TABLE: Final = 0x0010  # DA 0x1000
CATEGORY_MONITOR: Final = 0x0101  # DA 0x9000
CATEGORY_COMMAND1: Final = 0x0102  # DA 0x9800
CATEGORY_COMMAND2: Final = 0x0103  # DA 0x9900
EXPECTED_CATEGORY_BASES: Final[dict[int, int]] = {
    CATEGORY_IO_CONTROL: 0x0D00,
    CATEGORY_POSITION_TABLE: 0x1000,
    CATEGORY_MONITOR: 0x9000,
    CATEGORY_COMMAND1: 0x9800,
    CATEGORY_COMMAND2: 0x9900,
}
EXPECTED_COIL_BASE: Final = 0x0400  # BA of category 0x0D. VERIFIED-LIVE
CATEGORY_STATUS_IMAGE: Final = 0x0002  # DA 0x0200: DSS1, DSS2, DSSE, POSS, ZONS, DIPM, DOPM, SIPM, SSSE (ids 1-9)
CATEGORY_CONTROLLER_MONITOR: Final = 0x0003  # DA 0x0300: CTL (item 1), current alarm (item 10), ...
CATEGORY_MONITOR2: Final = 0x0004  # DA 0x0400
CATEGORY_FACTORY_PARAMS: Final = 0x0009  # DA 0x0900: user parameters 3 and 4 live here
CATEGORY_ZONES: Final = 0x000C  # DA 0x0C00: zone boundaries ZNM / ZNL, 2 sets
CATEGORY_ACTIVE_PROFILE: Final = 0x000F  # DA 0x0F00: the move profile the EC applied
CATEGORY_MAINTENANCE: Final = 0x0200  # DA 0x8400: TLMC (item 1), ODOM (item 2), ...

CATEGORY_SUPPORT: Final = 0x0001  # DA 0x0100: SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... (fixed member order)
CATEGORY_SYSTEM: Final = 0x0100  # DA 0x8000: FLGS, CKID, AXNO, PCB, FVER, ... (16 x 2 words)

# What each directory category holds ("?" = content not understood).
CATEGORY_NAMES: Final[dict[int, str]] = {
    0x0001: "support functions (SUPT, MVPT, SPTS, SUP2, SUP3)",
    0x0002: "status image (DSS1 .. SSSE)",
    0x0003: "controller monitor (CTL, current alarm, ...)",
    0x0004: "second monitor block",
    0x0005: "alarm history",
    0x0006: "identification (serials, ACCL, unit name, model)",
    0x0007: "axis parameters (soft limits = P1, MAXV / MAXA / MAXD, push limits, AKID)",
    0x0008: "parameters (user parameters 2, 5-7, 9-12, 16; BRSL, move defaults, FMNT)",
    0x0009: "parameters (user parameters 3, 4, 13; ORG home pattern)",
    0x000C: "zone boundaries ZNM / ZNL (user parameters 14, 15, 17, 18)",
    0x000D: "control registers DRG1 / DRG2 / DRGE / POSR (coil base)",
    0x000F: "active move profile",
    0x0010: "position table",
    0x0012: "?",
    0x0013: "PIO input map?",
    0x0014: "PIO output map?",
    0x0100: "controller system info (AXNO, PCB, FVER, UNIT)",
    0x0101: "monitor",
    0x0102: "position-number move (POSR)",
    0x0103: "direct numeric move",
    0x0104: "PCMD / PPOS mirror",
    0x0109: "parameter copies",
    0x010A: "?",
    0x0111: "?",
    0x0112: "?",
    0x0114: "compact monitor",
    0x0200: "maintenance counters",
}

# Blocks with MS = 0 (members of mixed sizes): (DQ, register count) where the count is known.
# Member names UNVERIFIED; the counts are confirmed by reads (VERIFIED-LIVE).
KNOWN_MIXED_EXTENTS: Final[dict[int, tuple[int, int]]] = {
    0x0006: (36, 0x48),  # identification 0x0600-0x0647 (BACKUP segment of 72 words on both units)
    0x000F: (13, 20),  # active move profile 0x0F00-0x0F13 (VERIFIED-LIVE values)
    0x0013: (16, 16),  # 0x8100-0x810F, 16 one-word members (PI00..PI15)
    0x0014: (16, 16),  # 0x8200-0x820F (PO00..PO15)
    0x0100: (16, 32),  # 0x8000-0x801F, 16 two-word members (FLGS .. CSUM)
    0x0101: (22, 34),  # monitor 0x9000-0x9021 (0x9022.. reads 0 and holds no data, VERIFIED-LIVE)
    0x0102: (3, 3),  # 0x9800-0x9802
    0x0103: (7, 10),  # 0x9900-0x9909 (0x9909 is the DRG1 mirror: never write it)
    0x0104: (3, 4),  # 0x9A00-0x9A03
    0x0114: (20, 40),  # compact monitor 0x9F00-0x9F27: 9F18 OLLV, 9F1A-1B TLMC, 9F1C-1D ODOM (VERIFIED-LIVE)
}


def category_extent(category_id: int, member_count: int, list_count: int, member_size: int) -> tuple[int, bool, int]:
    """Register extent of a directory block: ``(words, exact, words_max)``.

    ``words`` is a count that is certainly inside the block: DQ x MS x LQ when the members have one
    size (MS != 0), the known count for a mixed-size block of :data:`KNOWN_MIXED_EXTENTS`, otherwise
    DQ x LQ (every member is at least one register; ``exact`` is then False). ``words_max`` is the
    largest extent the record allows (two registers per member for an unknown mixed block): a
    read-only browser may read up to it and mark the words past ``words`` as "size uncertain"
    (live data sits there, e.g. 0x910F-0x9113, 0x7F08, 0x9E14-15). The EC does not reject unmapped
    addresses (it returns 0000), so these bounds are the only guard against reading past a block.
    """
    if member_size:
        words = member_count * member_size * list_count
        return words, True, words
    if category_id == CATEGORY_SUPPORT and 0 < member_count <= len(SUPPORT_MEMBERS):
        # class 1 has a fixed member order with known sizes (18 words for 11 members, 8 for the EC-R6S rod's 6)
        words = sum(size for _, size in SUPPORT_MEMBERS[:member_count]) * list_count
        return words, True, words
    known = KNOWN_MIXED_EXTENTS.get(category_id)
    if known is not None and known[0] == member_count:
        words = known[1] * list_count
        return words, True, words
    return member_count * list_count, False, 2 * member_count * list_count

# --------------------------------------------------------------------------- monitor block (FC03)

MONITOR_BASE: Final = 0x9000
PNOW: Final = 0x9000  # int32, current position, 0.01 mm. VERIFIED-LIVE
ALMC: Final = 0x9002  # present alarm code, 0 = none. VERIFIED-LIVE (reads 0)
DIPM: Final = 0x9003  # PIO input image. DOC-CON
DOPM: Final = 0x9004  # PIO output image. VERIFIED-LIVE values; layout depends on the PIO pattern (below)
# DOPM layout DIFFERS between models / PIO patterns (VERIFIED-LIVE on both units below):
#   3-position slider (EC-DS4H-150, model suffix MF): b0 LS0, b1 LS1, b2 LS2, b3 *ALM (1 = no alarm)
#   2-position gripper (EC-GRBP8M-20):                b0 LS0, b1 LS1, b2 *ALM (1 = no alarm)
# so DOPM b2 is "middle" on one unit and "no alarm" on the other. The driver therefore takes the
# position markers from ZONS (below) and the alarm from ALMC / DSS1, never from DOPM; DOPM is kept in raw.
DOPM_LS0: Final = 1 << 0  # completion output for position row 0 (both layouts)
DOPM_LS1: Final = 1 << 1  # completion output for position row 1 (both layouts)
DOPM_3POS_LS2: Final = 1 << 2  # 3-position layout only: row 2 (middle)
DOPM_3POS_NO_ALARM: Final = 1 << 3  # 3-position layout: *ALM, 1 = no alarm
DOPM_2POS_NO_ALARM: Final = 1 << 2  # 2-position layout: *ALM, 1 = no alarm (gripper: DOPM 0x0006 at 9.50 mm)
DSS1: Final = 0x9005  # device status 1. VERIFIED-LIVE values / DOC-CON bit meanings
DSS2: Final = 0x9006  # device status 2. DOC-CON
DSSE: Final = 0x9007  # expansion device status. VERIFIED-LIVE values / DOC-CON bit meanings
STAT: Final = 0x9008  # uint32 system status. DOC-CON
VNOW: Final = 0x900A  # int32 speed, ~0.01 mm/s but quantized in ~12.48 mm/s steps on the EC
#                       (12.45 / 24.90 / 37.48 / 49.93 / 99.98 seen). VERIFIED-LIVE. Too coarse for control.
CNOW: Final = 0x900C  # int32, motor current command. DOC-CON (mA); live: ~464 while pushing at 30 %
DEVI: Final = 0x900E  # int32, deviation, pulses. DOC-CON
STIM: Final = 0x9010  # uint32, ms since power on. VERIFIED-LIVE (tracks wall clock)
SIPM: Final = 0x9012  # input status bits (0x0300 live). (value)
ZONS: Final = 0x9013  # zone / position-marker status. VERIFIED-LIVE on the EC-DS4H slider and EC-GRBP8M gripper
ZONS_Z1: Final = 1 << 0  # inside zone 1 (both zones span the whole stroke on both units: reads 1)
ZONS_Z2: Final = 1 << 1  # inside zone 2
# The LS bits follow the position alone, moving or not: lit while PNOW is within the LS range (EC parameter 2,
# 0.10 mm on both units) of the row's position, or of a push row's push end (PCMD +/- INP). Consistent with 379 of
# 380 samples logged on hardware, e.g. gripper LS0 at 2.91 mm while moving, LS2 at 2.52 mm.
ZONS_LS0: Final = 1 << 12  # near position row 0, the backward end (slider 0x1003 at 0.00 mm)
ZONS_LS1: Final = 1 << 13  # near position row 1, the forward end (slider 0x2003 at 150.00; gripper 0x2003 at its
#                            push end 9.50 = 4.10 + 5.40 mm only, 0x0003 at 5.99 / 9.00 mm and along the push)
ZONS_LS2: Final = 1 << 14  # near position row 2, the middle (slider 0x4003 at 75.00; also on the 2-position
#                            gripper: 0x4003 at 2.52 mm, row 2 = 2.50 mm)
LS_RANGE_MM: Final = 0.10  # EC parameter 2 "LS signal detection range" on both units (as IA-OS shows it); width
#                            matches the log
POSS: Final = 0x9014  # completed position number, b0-b9 (10 bits; 7 bits on RCON, DOC ME0413 p.5-10 to 5-23);
#                      valid at a push contact and at a push miss too (DOC ME0162-12A p.4-24, 5-64); 0 after 0x9900 moves
SSSE: Final = 0x9015  # system status expansion (DOC ME0162-12A p.4-25)
POSS_MASK: Final = 0x03FF  # POSS b0-b9 (DOC ME0162-12A p.4-24)
POSS_MASK_RCON: Final = 0x007F  # 7 bits on RCON driver units (DOC ME0413 p.5-10 to 5-23)
SSSE_ALMC: Final = 1 << 11  # a cold-start-level alarm is present (DOC ME0162-12A p.4-25)
SSSE_RTC: Final = 1 << 8  # the real-time clock (calendar) is in use (DOC ME0162-12A p.4-25)
STATUS_READ_WORDS: Final = 22  # 0x9000..0x9015 (PNOW..SSSE) in one read: 01 03 90 00 00 16 E9 04 (DOC ME0162-12A
#                                p.4-7, 5-10). The monitor block is 34 registers on all three ECs (VERIFIED). A unit
#                                whose directory record for 0x9000 is shorter gets STATUS_BLOCK_WORDS
STATUS_BLOCK_WORDS: Final = 20  # 0x9000..0x9013 (PNOW..ZONS): the first fallback when 22 words are refused
STATUS_BLOCK_MIN_WORDS: Final = 8  # last fallback 0x9000..0x9007 (PNOW..DSSE), all VERIFIED-LIVE readable

# STAT 0x9008-0x9009 (u32) bits. DOC ME0162-12A p.4-21 (the EC's idle value 0x0000801F is VERIFIED; the meanings
# are UNVERIFIED on an EC)
STAT_MPOW: Final = 1 << 0  # motor drive power on (0 = drive power cut, e.g. an external drive cut-off)
STAT_SON: Final = 1 << 1  # servo-on command
STAT_SV: Final = 1 << 2  # servo on
STAT_HEND: Final = 1 << 3  # home return complete
STAT_RMDS: Final = 1 << 4  # MANU mode
STAT_AEEP: Final = 1 << 16  # non-volatile memory write in progress (RCON: not defined, DOC ME0413 p.5-15 to 5-19)
STAT_ASOF: Final = 1 << 17  # automatic servo OFF in effect (the unit switched the servo off after a standstill)
STAT_BATL: Final = 1 << 31  # absolute battery voltage low (SCON; operation continues while ALMH = 0)

# DSS1 bits. Values VERIFIED-LIVE (idle 0x3098 / 0x7098, after a completed move 0x7099, servo off
# 0x6010); meanings DOC-CON (ME0162 p.4-18) unless marked otherwise.
DSS1_EMGS: Final = 1 << 15  # emergency stop active
DSS1_SFTY: Final = 1 << 14  # safety speed enabled. VERIFIED-LIVE value (set on the EC-DS4H slider, probably by
#                             IA-OS Teach mode); a 50 mm/s move was NOT capped by it
DSS1_PWR: Final = 1 << 13  # controller ready
DSS1_SV: Final = 1 << 12  # servo on. VERIFIED-LIVE (servo off/on test)
DSS1_PSFL: Final = 1 << 11  # push missed the work (no contact over the band). VERIFIED-LIVE
DSS1_ALMH: Final = 1 << 10  # major alarm (operation-cancel / cold-start level). VERIFIED-LIVE with 0x0EE / 0x090
DSS1_ALML: Final = 1 << 9  # minor (message-level) alarm / warning. UNVERIFIED
# Alarm levels (alarms.py, UNVERIFIED on hardware): 0x011-0x07F are MESSAGES and the axis keeps running (maintenance
# warnings 0x048 / 0x04A / 0x04E / 0x04F, warnings 0x047 / 0x049), so a code alone is not a fault. Fault = ALMH, or
# a code of the level "operation cancelled" / "cold start" (alarms.alarm_level).
DSS1_ABER: Final = 1 << 8  # absolute error
DSS1_BKRL: Final = 1 << 7  # brake released (DOC ME0162-12A p.4-18: 1 while the servo is on). Live on the EC: 1 when
#                            stopped, 0 while moving; not used by the driver
DSS1_STP: Final = 1 << 5  # pause active. VERIFIED-LIVE
DSS1_HEND: Final = 1 << 4  # home return complete (stays 1 with servo off on WA; 0 while homing). VERIFIED-LIVE
DSS1_PEND: Final = 1 << 3  # inside the INP band; 0 while paused / servo off / after a missed push,
#                            1 after STOP and after a push made contact. VERIFIED-LIVE. It (and MEND) can rise
#                            while the axis is still moving inside the band (3.04 -> 3.00 mm): "done" also needs
#                            PNOW unchanged over two polls ~0.1 s apart
DSS1_TARGET_REACHED: Final = 1 << 0  # EC only (MEND): 1 after a move ends in its band (or a push makes contact
#                                      or misses), 0 after STOP or servo cycle. VERIFIED-LIVE observation

# DSS2 bits. DOC ME0162-12A p.4-19
DSS2_ENBS: Final = 1 << 15  # teaching-tool enable (dead-man) switch: 0 = released, the servo is held off (a servo-ON
#                             precondition, DOC ME0162-12A p.4-11, 4-19, 5-83); fixed 1 in AUTO mode or without the
#                             enable function (the EC's DSS2 reads 0x8000, VERIFIED)
DSS2_MODS: Final = 1 << 11  # teaching mode
DSS2_PWRT: Final = 1 << 10  # position data write (teach) complete (DOC ME0162-12A p.5-97)

# DSSE bits. Values VERIFIED-LIVE (idle 0x33C2, moving 0x33F2, homing 0x3BF2 / 0x39C2, push contact 0x37C3),
# meanings DOC-CON (ME0162 p.4-20); names not in the manual are UNVERIFIED
DSSE_EMGP: Final = 1 << 15  # emergency stop input (DOC ME0162-12A p.4-20)
DSSE_MPUV: Final = 1 << 14  # motor drive power cut off / low (DOC ME0162-12A p.4-20)
DSSE_RMDS: Final = 1 << 13  # 1 = MANU mode, 0 = AUTO: PIO has control (always 1 without a mode switch; DOC p.4-20)
DSSE_SSON: Final = 1 << 12  # name UNVERIFIED; reads 1 with servo on
DSSE_GHMS: Final = 1 << 11  # home return in progress (also during the final creep, when MOVE is 0). VERIFIED-LIVE
DSSE_PUSH: Final = 1 << 10  # push in progress / holding after contact. VERIFIED-LIVE
DSSE_PSNS: Final = 1 << 9  # excitation detection complete (0 during the homing creep on the EC)
DSSE_PMSS: Final = 1 << 8  # 1 = PIO disabled, Modbus has control. VERIFIED-LIVE value
DSSE_MOVE: Final = 1 << 5  # moving. VERIFIED-LIVE, BUT it clears while still decelerating on fast moves and is
#                            0 while paused and during the homing creep: never use it alone for "move done"
DSSE_PARW: Final = 1 << 7  # parameter write idle / complete: the driver waits for DRGE b7 PARW = 0 AND this = 1,
#                            both before and after the PARW coil. Idle 0x33C2 / 0x32C2 on all three units has it
#                            set (VERIFIED-LIVE). On a PCON controller it read 0 during the commit (VERIFIED); on
#                            the EC the capture's one sample inside the commit window still read 1 (0x33C2, +148 ms)
DSSE_DST1: Final = 1 << 4  # name UNVERIFIED; seen set while moving (live)
DSSE_PUSH_CONTACT: Final = 1 << 0  # EC: 1 while a push holds after contact (0x37C3). VERIFIED-LIVE on the EC-DS4H
#                                    slider, but also seen for a moment during a push that then missed (EC-GRBP8M
#                                    gripper, 6.76 mm): wait for the fingerprint to hold with the position unchanged
#                                    before calling it contact

# --------------------------------------------------------------------------- control registers (FC06)

DRG1: Final = 0x0D00  # device control 1. VERIFIED-LIVE readback 0x1000 / 0x5000 (SON, SFTY) / DOC-CON bits
DRG2: Final = 0x0D01  # device control 2. DOC-CON
DRGE: Final = 0x0D02  # expansion control (bit 8 = PMSL). VERIFIED-LIVE readback 0x0100
DRGE_PARW: Final = 1 << 7  # parameter write (commit): coil 0x0428; self-clearing. Capture: DRGE 0x0180 at +55 ms
#                            after the coil (b7 still set: commit in progress), 0x0100 at +235 ms (done)
DRGE_SFRS: Final = 1 << 15  # software reset coil 0x0420: NEVER USED (with the servo on it raised alarm 0x090)
POSR: Final = 0x0D03  # position number for CSTR. DOC-CON
# DRG1 bits (DOC ME0162-12A p.5-124 to 5-128; RCON ME0413 p.5-11). The "register" command strategy writes DRG1
# with FC06 and keeps SON / SFTY (and a held STP) in every write; it never sets EMG or BKRL.
DRG1_EMG: Final = 1 << 15  # emergency stop: never set by this driver
DRG1_SFTY: Final = 1 << 14
DRG1_SON: Final = 1 << 12
DRG1_ALRS: Final = 1 << 8
DRG1_BKRL: Final = 1 << 7  # brake release: never set by this driver
DRG1_STP: Final = 1 << 5
DRG1_HOME: Final = 1 << 4
DRG1_CSTR: Final = 1 << 3  # CON only ("-" on RCON, DOC ME0413 p.5-11)
DRG1_KEEP: Final = DRG1_SON | DRG1_SFTY | DRG1_STP  # the levels every FC06 write of DRG1 carries over

COMMAND1_BASE: Final = 0x9800  # POSR: FC06 write = immediate move to that position no. VERIFIED-LIVE

# --------------------------------------------------------------------------- direct numeric move (FC10)

DIRECT_MOVE_BASE: Final = 0x9900  # VERIFIED-LIVE: an FC10 write starts the move at once and re-targets a
#                                   move in progress; the block keeps the last written values
PCMD: Final = 0x9900  # int32 target (or signed distance with CTLF_INC), 0.01 mm
INP: Final = 0x9902  # uint32 positioning band (push band with CTLF_PUSH), 0.01 mm
VCMD: Final = 0x9904  # uint32 speed, 0.01 mm/s
ACMD: Final = 0x9906  # uint16 accel = decel, 0.01 G
PPOW: Final = 0x9907  # uint16 push current limit, 255 = 100 %. VERIFIED-LIVE (0 for normal moves, 77 = 30 %)
CTLF: Final = 0x9908  # control flags. Resets to 0 after each move start, but a CTLF written without a move
#                       start stays pending: the driver therefore always writes it (DIRECT_MOVE_WORDS = 9)
DIRECT_MOVE_WORDS: Final = 9  # PCMD, INP, VCMD, ACMD, PPOW, CTLF. VERIFIED-LIVE frame shape
CTLF_ABSOLUTE: Final = 0x0000  # VERIFIED-LIVE
CTLF_PUSH: Final = 1 << 1  # push move: approach PCMD, then push over INP at PPOW. VERIFIED-LIVE
CTLF_PUSH_AWAY: Final = 1 << 2  # with CTLF_PUSH: push away from home (+); without it toward home (-). VERIFIED-LIVE
CTLF_INC: Final = 1 << 3  # incremental: PCMD is the signed distance. VERIFIED-LIVE (not used by the driver)
# CTLF b6-b7 MOD acceleration pattern: 00 trapezoid, 01 S-motion (ratio CON No.56), 10 first-order filter (time
# constant CON No.55); 11 is not allowed (DOC ME0162-12A p.5-134; ME0384 p.6-102 to 6-105). b4-b5 GSL and b12-b13 NTC
# are left 0 by the driver (ACON / SCON only, DOC p.5-134). UNVERIFIED on an EC (offered when MVPT says so).
CTLF_MOD_MASK: Final = 0x00C0
CTLF_MOD_S_MOTION: Final = 0x0040
CTLF_MOD_FILTER: Final = 0x0080
ACCEL_PATTERNS: Final[dict[str, int]] = {"trapezoid": 0x0000, "s_motion": CTLF_MOD_S_MOTION, "filter": CTLF_MOD_FILTER}
ACCEL_PATTERN_MVPT_BITS: Final[dict[str, int]] = {"s_motion": 1 << 1, "filter": 1 << 2}  # MVPT b1 SCUR, b2 PFIL
S_MOTION_MAX_ACCEL_TIME_S: Final = 2.0  # S-motion is invalid for an acceleration time above 2 s (DOC PCON p.4-18)

PPOW_MIN: Final = 0x33  # 20 % = floor(20 x 255 / 100), the CON manuals' push range (DOC ME0162-12A p.5-132). On the
#                         EC the lower bound is MNPP (item 7.58, or 20 % when absent), only a RECOMMENDATION (the
#                         EC-R6S rod ships a 10 % row): the driver warns below it and never refuses the EC's own value
PPOW_MAX: Final = 0xB2  # 70 % = floor(70 x 255 / 100) (DOC ME0162-12A p.5-132, 5-142: 70 % -> B2h). The hard cap on
#                         a unit is its MXPP (item 7.9: slider 60, rod 70, gripper 100 %)
PPOW_RAW_MAX: Final = 0x1FE  # 200 %: RCS2-RA13R / RCS3-RA15R / RA20R take 20-200 % = 0x33-0x1FE (DOC ME0162-12A
#                              p.5-132); the unit's MXPP decides how far a given axis goes
PUSH_SPEED_MAX_MM_S: Final = 20.0  # FALLBACK push speed only, when neither PSHV (item 8.15, CON No.34) nor MXPH (7.8)
#                                    is readable. The push speed "may vary by model" (DOC ME3816-6B p.4-36): 10 / 20 /
#                                    25 / 30 / 38 / 40 mm/s by series (DOC ME3793-10E p.1-27; ME3794-9C p.1-10, 1-43).
#                                    Read per unit: slider 30.00, rod 20.00 (VERIFIED ~20 mm/s), gripper 5.00 mm/s.
#                                    The unit pushes at min(VCMD, PSHV, MXPH) (DOC ME3793-10E p.1-27)

# Documented minimum speeds (units/s) per EC series and lead (mm): a RECOMMENDATION only; below it the unit "may
# cause abnormal noise or unstable speed" (DOC ME3793-10E p.1-24). The driver warns, never refuses. Series from the
# model string (0x0620), lead from the unit (item 7.3). DOC: S3/S4/S6/S7/S8 ME3793-10E p.1-7 to 1-132; R6/R7/RR8/RR10
# ME3794-9C p.1-6 to 1-48; rotary ME3800-5E p.1-5 to 1-27 (deg/s); grippers ME3806-4C p.1-5 to 1-24; wire ME3832-1D
# p.1-6 to 1-12. Anything else: lead x 1.25 (= lead / 800 pulses / 1 ms, DOC ME0155 p.139) when the lead is known.
MIN_SPEED_BY_SERIES_LEAD: Final[dict[str, dict[float, float]]] = {
    "S3": {6.0: 8.0, 4.0: 5.0, 2.0: 3.0},
    "S4": {16.0: 40.0, 10.0: 30.0, 5.0: 7.0, 2.5: 4.0},
    "S6": {20.0: 25.0, 12.0: 15.0, 6.0: 8.0, 3.0: 4.0},
    "S7": {24.0: 30.0, 16.0: 20.0, 8.0: 10.0, 4.0: 5.0},
    "S8": {30.0: 38.0, 20.0: 25.0, 10.0: 13.0, 5.0: 7.0},
    "R6": {20.0: 25.0, 12.0: 15.0, 6.0: 8.0, 3.0: 4.0},
    "R7": {24.0: 30.0, 16.0: 20.0, 8.0: 10.0, 4.0: 5.0},
    "RR8": {10.0: 13.0, 5.0: 7.0},
    "RR10": {5.0: 7.0, 2.5: 4.0},
}
MIN_SPEED_BY_SERIES: Final[dict[str, float]] = {"RTB": 20.0, "RTC": 20.0, "GRB": 5.0, "GRC": 10.0, "WER": 5.0,
                                                 "WEGR": 5.0}
MIN_SPEED_PER_LEAD: Final = 1.25  # generic: lead / 800 pulses / 0.001 s (DOC ME0155 p.139)

# --------------------------------------------------------------------------- position table (FC03 / FC10)

POSITION_TABLE_BASE: Final = 0x1000  # VERIFIED-LIVE rows 0..2 (0.00 / 150.00 / 75.00 mm)
POSITION_STRIDE: Final = 16  # words per position. VERIFIED-LIVE
POSITION_WORDS_WRITABLE: Final = 15  # +0x0..+0xE as in the ME0162 examples; +0xF is the checksum (below)
POSITION_MAX_ENTRIES: Final = 0x300  # LQ of category 0x10 in the index table. BACKUP
POSITION_ACTIVE_ROWS: Final = 3  # an EC uses rows 0 (backward end), 1 (forward end), 2 (middle, 3-position
#                                  mode) only: live table (VERIFIED-LIVE), at most 3 positions
POSITION_SIMPLE_DATA_ROWS: Final = range(256, 268)  # 0x2000 page: the "simple data" store. Never written by this
#                                                      driver. IA-OS displays these rows, not rows 0-2 (UNVERIFIED).
# Row roles (UNVERIFIED: row 263 is the PREVIOUS forward %, the Middle % row is 266). A later IA-OS Transfer
# rewrites rows 0-2 from these rows (UNVERIFIED).
SIMPLE_DATA_ROLES: Final[dict[int, str]] = {
    256: "backward end, present (real)", 257: "forward end, present (real)",
    258: "backward end, previous (real)", 259: "forward end, previous (real)",
    260: "backward end, present (IA-OS %)", 261: "forward end, present (IA-OS %)",
    262: "backward end, previous (IA-OS %)", 263: "forward end, previous (IA-OS %)",
    264: "middle, present (real)", 265: "middle, previous (real)",
    266: "middle, present (IA-OS %)", 267: "middle, previous (IA-OS %)",
}  # fmt: skip
POSITION_CTLF_KNOWN: Final = 0x003E  # PUSH, DIR, INC and the 0x0010 / 0x0020 bits seen on live rows 1 / 2
POSITION_CTLF_ROLE_BITS: Final = 0x0030  # 0x0010 on row 1 (forward end), 0x0020 on row 2 (middle). Meaning
#                                          unknown (CON: servo gain set); preserved unless explicitly allowed
# offsets inside an entry. VERIFIED-LIVE values / DOC-CON names
POS_PCMD: Final = 0x0  # int32 0.01 mm
POS_INP: Final = 0x2  # uint32 0.01 mm
POS_VCMD: Final = 0x4  # uint32 0.01 mm/s
POS_ZNMP: Final = 0x6  # int32 0.01 mm
POS_ZNLP: Final = 0x8  # int32 0.01 mm
POS_ACMD: Final = 0xA  # uint16 0.01 G
POS_DCMD: Final = 0xB  # uint16 0.01 G
POS_PPOW: Final = 0xC  # uint16 push power
POS_LPOW: Final = 0xD  # uint16 load threshold
POS_CTLF: Final = 0xE  # uint16 control flags
POS_CHECKSUM: Final = 0xF  # low byte makes the byte sum of the 32-byte entry 0 mod 256. BACKUP (all 768 rows),
#                           VERIFIED-LIVE values; the driver writes it with the row (VERIFIED: 0x00C1 -> 0x008F).
# An FC10 write to 0x1000 reaches non-volatile memory at once (VERIFIED: row 0 survived a 24 V power cycle) and the
# EC uses it at once. Parameters, unlike rows, need the PARW commit (COIL_PARW) and a software reset.


def position_address(n: int) -> int:
    """Return the first register of position-table entry ``n``."""
    if not 0 <= n < POSITION_MAX_ENTRIES:
        raise ValueError(f"position number must be 0..{POSITION_MAX_ENTRIES - 1}, got {n}")
    return POSITION_TABLE_BASE + POSITION_STRIDE * n


# --------------------------------------------------------------------------- parameter / info blocks (FC03)

ALARM_HISTORY_BASE: Final = 0x0500  # 32 entries x 6 words {detail, address, 0, code, time hi, time lo}.
#                                     VERIFIED-LIVE (stride and count over all 192 words). Index 0 is the NEWEST,
#                                     the list ends at the first code 0, and the time is MILLISECONDS SINCE
#                                     POWER-ON (the EC has no clock); FFFFFFFF = no time (UNVERIFIED)
ALARM_HISTORY_STRIDE: Final = 6
ALARM_HISTORY_ENTRIES: Final = 32
ALARM_POWER_UP: Final = 0x0FFF  # "PowerUP No Error" marker written at every power-up. VERIFIED-LIVE

MAINTENANCE_BASE: Final = 0x8400  # class 0x200 (NV), 32-bit items by ID: 1 TLMC total moves (+1 per move start,
#                                   VERIFIED-LIVE 94 -> 101), 2 ODOM distance in m. Other names UNVERIFIED.
#                                   The item set differs per model (TTIM / TEMP at 0x840A / 0x840E on the slider and
#                                   rod, 0x8408 / 0x840C on the gripper): always map by item ID
MAINTENANCE_TOTAL_MOVES_ID: Final = 1
MAINTENANCE_DISTANCE_ID: Final = 2
CURRENT_ALARM_ITEM: Final = (CATEGORY_CONTROLLER_MONITOR, 10)  # class 3 item 10 = present alarm code (0xEE seen
#                                                               with ALMC 0xEE on the gripper, VERIFIED-LIVE)
POSITION_MODE_ITEM: Final = (CATEGORY_MOTION_PARAMS, 2)  # user parameter 9: 0 Double, 3 Single, 4 on the
#                                                          3-position slider. CONFIRMED (gripper diffs)

IDENTIFICATION_BASE: Final = 0x0600  # BACKUP + VERIFIED-LIVE readable (72 words). Fixed layout (class 6 has no
#                                      ID map, every member is 2 words; text is Shift-JIS, high byte first)
IDENTIFICATION_WORDS: Final = 0x48
IDENT_SERIAL: Final = (0x00, 8)  # (offset, words): CTS1-4 controller serial number. VERIFIED-LIVE offsets
IDENT_ACTUATOR_SERIAL: Final = (0x08, 8)  # ACS1-4 actuator serial number (UNVERIFIED; the slider reads "0")
IDENT_PRDT: Final = 0x10  # PRDT u32 "manufacture information": MREV (first 4 hex digits) + FREV (last 4). UNVERIFIED
IDENT_ACCL: Final = 0x12  # ACCL u32: bits 8-11 FEX1/2/4/8 = extensibility level (0 Normal, 1 Position3, 2 Phase1,
#                           3 Phase2, 4 MultiIo). UNVERIFIED; slider 0x00010110 (level 1), gripper / rod level 0
IDENT_BISD: Final = 0x14  # BISD u32: reads like a BCD date (UNVERIFIED guess)
IDENT_USER_NAME: Final = (0x1A, 6)  # ANA1-3: the USER-EDITABLE unit name (renamed in IA-OS), NOT a model field
#                                     (blank on the EC-R6S rod). "MWB-EC-DSRR4" on the slider
IDENT_SHORT_MODEL: Final = IDENT_USER_NAME  # compatibility alias of IDENT_USER_NAME
IDENT_MODEL: Final = (0x20, 0x28)  # AS01-AS20 actuator model string, e.g. "EC-R6S-150-1-PN". BACKUP + VERIFIED-LIVE

SOFT_LIMITS: Final = 0x0700  # int32 +max, int32 -min, 0.01 mm (class 7 items 1-2). VERIFIED-LIVE read
#                              (+150.30 / -0.30 on a 150 mm EC). Read-only for this driver.
SOFT_LIMIT_MARGIN_MM: Final = 0.30  # the EC stores stroke + 0.30 mm at each end (ME3816 p.323); the driver
#                                     keeps its own targets inside the nominal stroke

MOTION_PARAMS_BASE: Final = 0x0800  # class 8 parameters (NV). BACKUP + VERIFIED-LIVE read
SPEED_LIMIT_OFFSET: Final = 0x12  # 0x0812-0x0813 = item 8.10 VCMD, the DEFAULT command speed (UNVERIFIED), not
#                                   the maximum. It equals MAXV on all three units (700.00 / 53.00 / 800.00 mm/s),
#                                   but that is a factory setting, not a rule: the driver's speed cap comes from MAXV
#                                   (item 7.4) and falls back to this word only when MAXV is not readable.
SPEED_LIMIT: Final = MOTION_PARAMS_BASE + SPEED_LIMIT_OFFSET

# Class 7 "axis information" item IDs. There is no per-model table: the driver reads these from the EC by item ID
# through the class's ID map; addresses differ per model. Values in 0.01 units unless noted.
AXIS_ITEM_LIMM: Final = 1  # + soft limit (P1 + 0.30 mm)
AXIS_ITEM_LIML: Final = 2  # - soft limit
AXIS_ITEM_LEAD: Final = 3  # screw lead, 0.01 mm
AXIS_ITEM_MAXV: Final = 4  # maximum speed, 0.01 mm/s (slider 700.00, gripper 53.00, rod 800.00; VERIFIED on the rod)
AXIS_ITEM_MAXA: Final = 5  # maximum acceleration, 0.01 G (1.00 / 0.30 / 1.00; VERIFIED on the rod)
AXIS_ITEM_MAXD: Final = 6  # maximum deceleration, 0.01 G (1.00 / 0.30 / 1.00)
AXIS_ITEM_ATYP: Final = 7  # axis type bits: TYPE b8, MODE b9, PSDN b12, BUEN b18 (= P8), RINP b24
AXIS_ITEM_MXPH: Final = 8  # maximum push speed, 0.01 mm/s (30.00 / 5.00 / 20.00)
AXIS_ITEM_MXPP: Final = 9  # maximum push current, % (60 / 100 / 70): the HARD cap; 0 = no push
AXIS_ITEM_STN1: Final = 15  # smart tuning actuator / lead numbers (FFFFFFFF = none)
AXIS_ITEM_AKID: Final = 33  # actuator kind, bits 0-7 (ACTUATOR_KINDS); bit 8 return type
AXIS_ITEM_TCPC: Final = 34  # push force conversion factor (0 = no push)
AXIS_ITEM_TCOA: Final = 35  # push force conversion offset
AXIS_ITEM_MNPP: Final = 58  # recommended minimum push, % (slider 20, gripper 50, absent on the rod -> 20)
AXIS_ITEM_TCP2: Final = 62  # second push conversion factor (the driver warns before a P3 change when != 0)
MNPP_DEFAULT_PCT: Final = 20.0  # the push recommendation when MNPP is absent or 0 (a warning, never a refusal).
#                                 UNVERIFIED
ATYP_BUEN: Final = 1 << 18  # P8 power saving: 1 = Disable (EC-R6S rod: 0x05350000 -> "1:Disable" in IA-OS)
ATYP_RINP: Final = 1 << 24  # minimum positioning band applies (with SUP2 CINP)
ATYP_ROTARY: Final = 1 << 8  # TYPE: rotary axis (CON No.78 "Axis action type" 1; units without AKID 7.33), UNVERIFIED

# Class 8 / 9 / 3 / 4 item IDs used by the driver (gripper parameters mapped on hardware; the others UNVERIFIED)
PARAM_ITEM_BRSL: Final = 3  # 8.3 power-up baud code (IAI baud code: 6 = 38400 on all three units)
PARAM_ITEM_RTIM: Final = 4  # 8.4 minimum delay before the EC's transmitter turns on, ms (5)
PARAM_ITEM_ZRMK: Final = 8  # 8.8 (b8 CORG: the driver warns before a P3 change)
PARAM_ITEM_VCMD: Final = 10  # 8.10 default command speed (0x0812)
PARAM_ITEM_CTLF: Final = 14  # 8.14 control flags (P5 b6, P6 b10; default collision set b19-20)
PARAM_ITEM_PSHV: Final = 15  # 8.15 push speed, 0.01 mm/s
PARAM_ITEM_SPIO: Final = 50  # 8.50 (P7 b23, P10 b18, P16 b10, EXFN b13, INIT b15, SLCD b28 collision output)
PARAM_ITEM_FMNT: Final = 54  # 8.54 servo monitor: MODE b0-3 (0 none, 1 = 4 ch, 2 = 8 ch, 3 = 2 ch), BFCY b16-31
COLLISION_ITEMS: Final = tuple(range(119, 128))  # 8.119-8.127: CDC1 CDD1 CDB1 CDC2 CDD2 CDB2 CDC3 CDD3 CDB3
#                                                  (UNVERIFIED)
SPIO_EXFN: Final = 1 << 13
SPIO_SLCD: Final = 1 << 28  # collision detection output: 0 alarm, 1 warning
DRIVER_ITEM_FSTP: Final = 15  # 9.15 (P13 = b10 PSER)
DRIVER_ITEM_ORG: Final = 16  # 9.16 home pattern number (P3 = its parity); 0x45 home sends op1 = ORG & 0xFFFF
CONTROLLER_ITEM_CTCL: Final = 1  # 3.1 b31-24 system, b23-16 unit, b15-0 firmware (same as FC11; faster)
CONTROLLER_ITEM_MAXP: Final = 2  # 3.2 maximum position count (3 on the 3-position slider)
CONTROLLER_ITEM_MXPT: Final = 16  # 3.16 highest allowed P9 value
CONTROLLER_ITEM_SRWT: Final = 52  # 3.52 software reset wait, ms (slider 5500, rod / gripper 1000)
CONTROLLER_ITEM_MINI: Final = 67  # 3.67 minimum positioning band (P2 lower limit with CINP + RINP)
MONITOR2_ITEM_CNOW: Final = 4  # 4.4 motor current (same unit as MCUR)
MONITOR2_ITEM_MCUR: Final = 5  # 4.5 rated current (slider 1200): load % = CNOW / MCUR x 100 (UNVERIFIED)
AXNO_OFFSET: Final = 4  # AXNO u32 at 0x8004 (class 0x0100 member 3): axis number = address - 1. Read-only (set only
#                         with 0x45 cmd 3, which the driver never sends). VERIFIED-LIVE 0 on the EC-R6S rod

# Class 1 "support function information": no ID map, fixed member order (UNVERIFIED); 32-bit items high word first.
# The EC-R6S rod has 6 members (SUPT .. SUP2), the slider and the gripper 11 (up to SUP3). Bit meanings UNVERIFIED.
SUPPORT_MEMBERS: Final[tuple[tuple[str, int], ...]] = (
    ("SUPT", 2), ("DIAG", 1), ("MVPT", 1), ("SPTS", 1), ("CTYP", 1), ("SUP2", 2), ("AEHS", 2), ("ACHS", 2),
    ("ARSF", 2), ("SPTM", 2), ("SUP3", 2),
)  # fmt: skip
SUPT_SRST: Final = 1 << 2  # software reset supported
SUPT_ROAX: Final = 1 << 3  # rotational INDEX MODE capability, not "rotary actuator" (that is AKID = 1)
SUPT_INCM: Final = 1 << 8  # relative (incremental) moves
SUPT_PUSH: Final = 1 << 11  # push motion
MVPT_VCTL: Final = 1 << 10  # vibration control
MVPT_CDAS: Final = 1 << 13  # collision detection (0 on the three tested ECs)
SUP2_CNWL: Final = 1 << 23  # wireless module (P7 shown only with it)
SUP2_CINP: Final = 1 << 24  # minimum positioning band input limit (P2 >= MINI)
SUP3_SORS: Final = 1 << 5  # ACR operation ready signal (P16 shown only with it)
SUP3_IVC1: Final = 1 << 16  # the unit takes function 0x48 instead of 0x45 (0 / absent on the three tested ECs,
#                             which use 0x45)

# --------------------------------------------------------------------------- coils (FC05: FF00 on / 0000 off)
# Coil = coil base 0x0400 + (15 - bit) of DRG1 (0x0410.. DRG2, 0x0420.. DRGE). The base (BA = 0x0400 for
# category 0x0D) is VERIFIED-LIVE in the index table; coil meanings are DOC-CON (ME0162 p.5-80).

COIL_SFTY: Final = 0x0401  # safety speed (level): caps speed at ~100 mm/s; volatile. VERIFIED-LIVE (EC-DS4H)
COIL_SON: Final = 0x0403  # servo on (level). VERIFIED-LIVE
COIL_ALRS: Final = 0x0407  # alarm reset (rising edge: FF00 then 0000). VERIFIED-LIVE pulse (no alarm present)
COIL_STP: Final = 0x040A  # pause (FF00, level: holds while ON) / resume (0000, auto-resumes). VERIFIED-LIVE
COIL_HOME: Final = 0x040B  # home return: 0000 then FF00 (edge), release 0000 after HEND. VERIFIED-LIVE
COIL_CSTR: Final = 0x040C  # start move to POSR (rising edge). DOC-CON, untested on EC
COIL_MOD: Final = 0x0414  # DRG2 b11 MOD teaching mode (DOC ME0162-12A p.5-97). Experimental teach path only
COIL_TEAC: Final = 0x0415  # DRG2 b10 TEAC: write the current position into row POSR (DOC ME0162-12A p.5-97; alarm 093
#                            if held before homing, p.5-99). Experimental teach path only; UNVERIFIED on an EC
COIL_PMSL: Final = 0x0427  # FF00 = Modbus control (DSSE PMSS = 1). VERIFIED-LIVE on the EC-DS4H slider. PMSS is
#                            0 at power-up. It does NOT gate 0x9900 moves or STOP, and switching it ON mid-move
#                            STOPS the move (both VERIFIED-LIVE): write it only while the axis is at rest.
COIL_STOP: Final = 0x042C  # decel stop. VERIFIED-LIVE sequence: FF00, then 0000 after the stop; stopped a
#                            Modbus-commanded move with PMSS = 0 (EC-DS4H). UNTESTED on a move a PLC commanded
#                            over PIO (a held start input might restart it after the release). Whether the EC
#                            releases it by itself (ME0162 says so for CON) is UNVERIFIED, so the driver always
#                            writes 0000 afterwards.
COIL_JOG_PLUS: Final = 0x0416  # NEVER USED: runs at ~100 mm/s from an EC parameter, not the commanded speed.
#                                 VERIFIED-LIVE
COIL_JOG_MINUS: Final = 0x0417  # NEVER USED (as JOG+)
COIL_PCRF: Final = 0x042E  # DRGE b1 PCRF: run the command profile at 0x0F00 (written first, FC17). VERIFIED on the
#                            EC-GRBP8M gripper and the PCON-CB (self-clearing; STOP stops it; a new profile + PCRF
#                            retargets)
COIL_PARW: Final = 0x0428  # DRGE b7 PARW: commit the parameters in RAM to non-volatile memory. ON-WIRE (IA-OS
#                            Transfer, USB capture: FC10 item write, then 01 05 04 28 FF 00; no OFF is sent, DRGE
#                            reads back without b7) and VERIFIED (the driver's replay; a committed P2 survived a
#                            reset). Without it a parameter write is RAM only (VERIFIED: lost at the next power
#                            cycle). Takes effect only after a software reset / power cycle (UNVERIFIED).
COIL_SFRS: Final = 0x0420  # DRGE b15 software reset. NEVER USED: pulsed with the servo on, it raised alarm 0x090
#                            "Software reset command while in SERVO-ON condition" (EC-GRBP8M); the driver uses
#                            0x45 cmd 5

# --------------------------------------------------------------------------- IAI control commands
# Function 0x45 / 0x48 command codes (IAI's control-command numbering; VERIFIED where marked, else UNVERIFIED).
# 0x45 request ``addr 45 group cmd op1 op2`` (u16 big-endian), reply ``addr 45 rejection status`` (ON-WIRE): a
# non-zero rejection is a refusal that this driver never retries. 0x48 request ``addr 48 group cmd(u16)
# op1..op4(u32)``, reply ``addr 48 rejection(u16) status1..4(u32)`` (UNVERIFIED): the command set of units with
# SUP3 IVC1 = 1; all three ECs tested use 0x45. Never used on the safety stop path (the STOP coil is).

IAI_CMD_BAUD: Final = 0x04  # broadcast to address 0: op1 = baud code (modbus.IAI_BAUD_CODES), op2 = 5 (as IA-OS).
#                             ON-WIRE (00 45 00 04 00 14 00 05 = 115200) + VERIFIED; volatile (38400 after power-up)
IAI_BAUD_OP2: Final = 0x0005  # op2 IA-OS sends with the baud change (ON-WIRE; meaning unknown)
IAI_CMD_RESET: Final = 0x05  # op1: 0 = alarm reset, 1 = software reset (Elecylinder.software_reset only)
IAI_RESET_ALARM: Final = 0x0000  # op1 of the alarm reset: 01 45 00 05 00 00 00 00 (VERIFIED on the EC-R6S rod)
IAI_RESET_SOFTWARE: Final = 0x0001  # op1 of the software reset: 01 45 00 05 00 01 00 00 -> 01 45 00 00 (ON-WIRE,
#                                     VERIFIED)
IAI_CMD_MOVE_POSITION: Final = 0x06  # op1 = stored row n, op2 = speed override: the row runs at (op2 + 1) / 256 of
#                                      its stored speed (255 = 100 %; ON-WIRE on the PCON-CB: op2 13 ran an 840 mm/s
#                                      row at 46.0 mm/s). VERIFIED on the EC-GRBP8M gripper and the PCON-CB
IAI_CMD_SERVO: Final = 0x09  # op1: 1 = on, 0 = off. VERIFIED (01 45 00 09 00 00 00 00 = servo OFF -> 01 45 00 00)
IAI_CMD_HOME: Final = 0x0A  # op1 = ORG (item 9.16) & 0xFFFF, op2 0. VERIFIED on the EC-R6S rod
#                             (01 45 00 0A 00 07 00 00); op1 = 0 is rejected 0x11 "1st operand error" (VERIFIED on
#                             the EC-GRBP8M gripper)
IAI_CMD_STOP: Final = 0x0B  # decel stop (VERIFIED; the driver's stop() uses the STOP coil 0x042C instead)
IAI_CMD_ACTION_MODE: Final = 0x0C  # op1 1 = PC (Modbus) / IA-OS "Teaching", 0 = PIO / "Monitoring"; op2 1 = safety
#                                    speed. VERIFIED on the EC-R6S rod: (1,1) (1,0) (0,0); read back DSSE PMSS /
#                                    DSS1 SFTY
IAI2_CMD_ALARM_RESET: Final = 0x0023  # 0x48 command codes are 16 bit. UNVERIFIED
IAI2_CMD_SERVO: Final = 0x0030
IAI2_CMD_HOME: Final = 0x0031
IAI2_CMD_STOP: Final = 0x0034

# Commands the driver never sends unless the caller passes ``allow_never_send=True`` (``allow_unsafe`` alone is not
# enough): non-volatile writes, axis number, baud, alarm-list clear, initialisation, maintenance writes, firmware
# update, inspection. The broadcast baud change (0x04) is sent only by Elecylinder.set_baudrate, which opts in.
IAI_NEVER_SEND: Final[dict[int, str]] = {
    0x01: "write parameters to non-volatile memory (the driver commits with the PARW coil instead)",
    0x02: "write positions to non-volatile memory",
    0x03: "change the axis number (slave address)",
    0x04: "change the baud rate (Elecylinder.set_baudrate opts in)",
    0x08: "change the ID group",
    0x0F: "clear the alarm list (wipes the non-volatile alarm history)",
    0x10: "initialise parameters (factory reset)",
    0x11: "change the DO type",
    0x1D: "initialise the absolute encoder",
    **{code: "write maintenance information" for code in range(0x20, 0x3D)},
    0x4F: "change the maintenance update mode",
    0x73: "set the axis index",
    0x90: "switch to firmware update mode",
    0x91: "switch to firmware update mode",
    **{code: "factory inspection command" for code in range(0xDD, 0x100)},
    0xF3: "clear position data",
}  # fmt: skip
IAI2_NEVER_SEND: Final[dict[int, str]] = {
    0x0001: "update maintenance information", 0x0002: "update maintenance information",
    0x0010: "write parameters to non-volatile memory", 0x0011: "write positions to non-volatile memory",
    0x0012: "change the axis number", 0x0020: "change the baud rate", 0x0021: "change the ID group",
    0x0040: "clear the alarm list", 0x0041: "initialise parameters", 0x0042: "change the DO type",
    0x0061: "load-cell key command", 0x0080: "initialise the absolute encoder",
    0x0090: "safety unit command", 0x0091: "safety unit command", 0x0092: "safety unit command",
    0x0093: "safety unit command", 0x00A1: "change the drive mode", 0x00B3: "change the axis ID (converter)",
    0x1000: "switch to firmware update mode", 0xFFFD: "clear position data",
}  # fmt: skip
IAI47_NEVER_SEND: Final[dict[int, str]] = {
    2: "write the encoder buffer area", 3: "write the encoder FRAM", 4: "clear controller maintenance info",
    5: "clear encoder maintenance info",
}
"""Function 0x47 (encoder AREG) commands, listed for completeness: this driver never builds 0x47 frames."""

# Rejection codes: the 0x45 reply byte (0x01-0x3F) and the 0x48 reply word (0x1000-0x13FF). Names UNVERIFIED
# except 0x11 (VERIFIED: home with op1 = 0 on the EC-GRBP8M gripper).
IAI_REJECTIONS: Final[dict[int, str]] = {
    0x01: "illegal function", 0x02: "illegal data address", 0x03: "illegal data value",
    0x04: "cannot execute: slave recovery disabled", 0x11: "1st operand error", 0x12: "2nd operand error",
    0x1F: "password mismatch", 0x21: "command code error", 0x22: "EEPROM write already in progress",
    0x23: "motion command while the servo is OFF", 0x24: "motion command during homing",
    0x25: "software reset while the servo is ON", 0x26: "motion command in TEACH mode",
    0x27: "invalid HOME command", 0x28: "parameter initialisation before EEPROM init completed",
    0x29: "software reset with ABS unit and Z phase enabled", 0x2A: "load-cell command without a load cell",
    0x2B: "load-cell command with the servo ON", 0x2C: "monitoring data type changed during monitoring",
    0x2D: "monitoring command while monitoring is invalid", 0x2E: "calendar command while the calendar is invalid",
    0x2F: "release of safety speed prohibited", 0x30: "monitoring conditions incorrect",
    0x31: "battery-less absolute encoder command error", 0x32: "encoder FRAM access with the servo ON",
    0x33: "encoder FRAM: bad page", 0x34: "encoder FRAM: bad unique code", 0x35: "encoder FRAM write error",
    0x38: "invalid command during virtual axis", 0x39: "software reset during homing",
    0x3A: "command prohibited in monitor (PIO) mode", 0x3B: "software reset while software reset is inhibited",
    0x3C: "servo ON after encoder FRAM access", 0x3D: "inspection command error", 0x3E: "inspection command error",
    0x3F: "inspection command error",
}  # fmt: skip
IAI2_REJECTIONS: Final[dict[int, str]] = {
    0x1000: "command code error", 0x1001: "1st operand error", 0x1002: "2nd operand error",
    0x1003: "3rd operand error", 0x1004: "4th operand error", 0x1005: "motion command while the servo is OFF",
    0x1006: "motion command during homing", 0x1007: "software reset while the servo is ON",
    0x1008: "motion command in TEACH mode", 0x1009: "invalid HOME command",
    0x100A: "parameter initialisation before EEPROM init completed", 0x100B: "load-cell command without a load cell",
    0x100C: "load-cell command with the servo ON", 0x100D: "monitoring data type changed during monitoring",
    0x100E: "monitoring command while monitoring is invalid", 0x100F: "monitoring conditions incorrect",
    0x1010: "servo monitor not available", 0x1011: "drive recorder not available",
    0x1012: "calendar command while the calendar is invalid", 0x1013: "release of safety speed prohibited",
    0x1014: "invalid command during virtual axis", 0x1015: "software reset during homing",
    0x1016: "inspection command error", 0x1017: "inspection command error",
    0x101C: "monitoring command in drive-recorder mode", 0x101D: "software reset while software reset is inhibited",
    0x101E: "battery-less absolute encoder command error", 0x101F: "movement during collision-detection reversal",
}  # fmt: skip
# Our own short hints for the rejections a driver user can act on. Hints that hold for any command are keyed by
# the rejection code; hints that only make sense for one command by (command, rejection code).
IAI_REJECTION_HINTS: Final[dict[int, str]] = {
    0x23: "turn the servo on first",
    0x24: "wait for the home return to finish",
    0x25: "turn the servo off first (0x45 cmd 9 op1 0)",
    0x26: "the EC is in TEACH mode",
    0x3A: "the EC is in PIO (monitor) mode: take Modbus control first (PMSL coil, or action mode op1 = 1)",
}
IAI2_REJECTION_HINTS: Final[dict[int, str]] = {
    0x1005: IAI_REJECTION_HINTS[0x23], 0x1006: IAI_REJECTION_HINTS[0x24], 0x1007: IAI_REJECTION_HINTS[0x25],
    0x1008: IAI_REJECTION_HINTS[0x26], 0x101F: "wait for the collision-detection reversal to finish",
}  # fmt: skip
_HOME_OPERAND_HINT: Final = "the home return needs op1 = the EC's home pattern ORG (item 9.16)"
IAI_COMMAND_REJECTION_HINTS: Final[dict[tuple[int, int], str]] = {
    (IAI_CMD_HOME, 0x11): _HOME_OPERAND_HINT,  # VERIFIED: op1 = 0 rejected (EC-GRBP8M), op1 = ORG accepted (EC-R6S)
    # VERIFIED: row 2 rejected by the EC-GRBP8M gripper in 2-position mode (P9 = 0); nothing moved, no alarm
    (IAI_CMD_MOVE_POSITION, 0x11): "the unit does not run this stored row in its position mode (an EC runs rows 0 "
    "and 1 in 2-position mode, P9 0 / 3; rows 0-2 in 3-position mode)",
}
IAI2_COMMAND_REJECTION_HINTS: Final[dict[tuple[int, int], str]] = {
    (IAI2_CMD_HOME, 0x1001): _HOME_OPERAND_HINT,  # UNVERIFIED (0x48 untested)
}


def iai_rejection_name(function: int, code: int) -> str:
    """Name of an IAI rejection code: 0x45 byte (0x01-0x3F) or 0x48 word (0x1000-0x13FF)."""
    table = IAI2_REJECTIONS if function == 0x48 else IAI_REJECTIONS
    return table.get(code, "unknown rejection code")


def iai_rejection_hint(function: int, code: int, command: int | None = None) -> str:
    """Our own short advice for a rejection ``code`` of ``command`` ("" when there is none).

    A command-specific hint (e.g. 0x11 on the 0x45 home, cmd 0x0A) is given only when ``command`` is passed
    and matches; the generic hints (servo off, homing, TEACH / PIO mode, ...) hold for any command.
    """
    if function == 0x48:
        specific, generic = IAI2_COMMAND_REJECTION_HINTS, IAI2_REJECTION_HINTS
    else:
        specific, generic = IAI_COMMAND_REJECTION_HINTS, IAI_REJECTION_HINTS
    if command is not None and (command, code) in specific:
        return specific[(command, code)]
    return generic.get(code, "")

# --------------------------------------------------------------------------- report slave id (FC11)
# VERIFIED-LIVE answers: 01 11 05 EC FF 02 00 13 C1 5D = EC 24 V, run status 0xFF, unit code 0x02, firmware
# 0x0013 (slider; backup CTL=EC020013), EC FF 0A 00 02 (gripper) and EC FF 00 00 02 (rod, after 0.2-0.4 s).
# The same identity is in CTCL, class 3 item 1 (0x0300 on all three units): b31-24 system, b23-16 unit, b15-0
# firmware (VERIFIED EC02 0013 / EC0A 0002 / EC00 0002): the driver falls back to it when FC11 is silent.

SYSTEM_CODES: Final[dict[int, str]] = {
    0xEC: "EC 24 V", 0xED: "EC 200 V", 0xAE: "PCON", 0xAC: "ACON / DCON", 0xAA: "SCON",
}  # fmt: skip
# Product family per FC11 system code (byte 0; DOC, from IAI manuals). Any other code is reported as a generic IAI
# controller and works the same way: every range and capability comes from the unit's own directory, never from this
# table.
PRODUCT_FAMILIES: Final[dict[int, str]] = {0xEC: "EC", 0xED: "EC", 0xAE: "PCON", 0xAC: "ACON/DCON", 0xAA: "SCON"}
GENERIC_FAMILY: Final = "IAI controller"
EC_SYSTEM_CODES: Final = frozenset({0xEC, 0xED})
# EC family names per (system, unit) code (from IA-OS); any other code is driven the same way, display only. The
# family is NOT the actuator kind: unit 0x0A is the
# EC(MS) "2-size" family (the EC-GRBP8M gripper); the kind (gripper / rod / slider / rotary) is AKID, item 7.33
# (ACTUATOR_KINDS). The EC-S6HH slider reports unit 0x00 (EC standard) and has no family of its own here.
EC_FAMILIES: Final[dict[tuple[int, int], str]] = {
    (0xEC, 0x00): "EC standard",
    (0xEC, 0x01): "EC slim/small",
    (0xEC, 0x02): "EC dust/drip-proof",
    (0xEC, 0x03): "EC high thrust",
    (0xEC, 0x08): "EC(UM) ultra-compact",
    (0xEC, 0x09): "EC(T) thin",
    (0xEC, 0x0A): 'EC(MS) "2-size EC"',
    (0xEC, 0x0B): "EC(UC) ultra-ultra-compact",
    (0xED, 0x00): "EC(200V)",
}
NOT_AN_ACTUATOR: Final[dict[tuple[int, int], str]] = {(0xEC, 0x10): "CCM conveyor control motor (not an EC actuator)"}
UNIT_CODES: Final[dict[int, str]] = {unit: name for (system, unit), name in EC_FAMILIES.items() if system == 0xEC}
"""EC family name per FC11 unit code of system 0xEC (display only: every unit code is driven the same way)."""
UNIT_CODES_CONFIRMED: Final[frozenset[int]] = frozenset()  # kept for compatibility; no longer used by the driver
UNITS_SEEN_ON_HARDWARE: Final[frozenset[tuple[int, int]]] = frozenset()
"""Kept for compatibility; no longer used by the driver (empty: every unit is driven the same way)."""


def product_family(system_code: int | None) -> str:
    """Product family of an FC11 system code: ``"EC"``, ``"PCON"``, ``"ACON/DCON"``, ``"SCON"`` or ``"IAI controller"``."""
    if system_code is None:
        return GENERIC_FAMILY
    return PRODUCT_FAMILIES.get(system_code, GENERIC_FAMILY)


def is_ec_family(system_code: int | None) -> bool:
    """The FC11 system code is an ELECYLINDER (0xEC / 0xED); every other code is a CON-family controller."""
    return system_code in EC_SYSTEM_CODES


def ec_family(system_code: int | None, unit_code: int | None) -> str | None:
    """EC family name of an FC11 (system, unit) pair, ``None`` if it is not in :data:`EC_FAMILIES`."""
    if system_code is None or unit_code is None:
        return None
    return EC_FAMILIES.get((system_code, unit_code))


# AKID (item 7.33) bits 0-7 -> actuator kind (VERIFIED: slider 9, gripper 4, rod 3; the other values UNVERIFIED).
ACTUATOR_KINDS: Final[dict[int, str]] = {
    0: "slider", 1: "rotary", 2: "arm", 3: "rod", 4: "gripper", 5: "other", 6: "linear", 7: "scalar", 8: "table",
    9: "slider (small)", 10: "rod (small)", 11: "wire",
}  # fmt: skip
AKID_ROTARY: Final = 1  # rotary iff KIND = 1: positions in deg, speeds in deg/s


def axis_units(actuator_kind: int | None, rotary_atyp: bool | None = None) -> str:
    """Position unit of an axis from its AKID kind: ``"deg"`` for a rotary axis (1), ``"mm"`` for everything else
    (an unknown or unread kind included). A unit without AKID (CON controllers) is rotary when its ATYP 7.7 TYPE
    bit 8 is set (``rotary_atyp``); AKID wins when present. Speeds are then in units/s; accelerations stay in G."""
    if actuator_kind is None:
        return "deg" if rotary_atyp else "mm"
    return "deg" if actuator_kind == AKID_ROTARY else "mm"


def axis_kind_name(actuator_kind: int | None, rotary_atyp: bool | None = None) -> str:
    """AKID kind in words (``"unknown"`` when not read, ``"kind <n>"`` for a value outside the table); without AKID,
    ``"rotary"`` when ATYP 7.7 bit 8 says so (``rotary_atyp``)."""
    if actuator_kind is None:
        return ACTUATOR_KINDS[AKID_ROTARY] if rotary_atyp else "unknown"
    return ACTUATOR_KINDS.get(actuator_kind, f"kind {actuator_kind}")

# The user-parameter candidates IA-OS lists per (system, unit) code. Some are then hidden by runtime rules
# (info.parameter_visibility); those rules predict exactly what IA-OS showed on the gripper and on the rod. A code
# without a row gets every mapped item.
PARAMETER_CANDIDATES: Final[dict[tuple[int, int], tuple[int, ...]]] = {
    (0xEC, 0x00): (1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x01): (1, 2, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x02): (1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x03): (1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x08): (1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x09): (1, 2, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x0A): (1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xEC, 0x0B): (1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18),
    (0xED, 0x00): (1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18),
}  # fmt: skip
IAOS_PARAMETER_CANDIDATES: Final = PARAMETER_CANDIDATES  # former name, kept for existing callers

# What IA-OS's "User parameter edit" screen showed on hardware (the lists the visibility rules must reproduce).
MODEL_PARAMETERS: Final[dict[int, tuple[int, ...]]] = {
    0x0A: (1, 2, 3, 4, 5, 6, 7, 9, 10, 12),  # EC-GRBP8M-20 gripper
    0x00: (1, 2, 3, 4, 5, 6, 8, 9, 12),  # EC-R6S-150-1-PN rod
}
MODEL_PARAMETERS_SEEN_ON: Final[dict[int, tuple[str, ...]]] = {}  # kept for compatibility; no longer used by the driver

# Alarm names for codes this driver may meet (IAI's alarm names; 0x0C1, 0x0D9 and 0x0EE were seen on hardware).
# Levels: alarms.alarm_level.
ALARM_NAMES: Final[dict[int, str]] = {
    0x080: "Motion command while in SERVO-OFF condition",
    0x083: "Absolute positional movement command when Homing is not completed",
    0x084: "Motion command during Homing",
    0x085: "Position No. error when moving",
    0x0A3: "Positional instruction information error when executing",
    0x0A7: "The instructed deceleration is abnormal",
    0x0AB: "The instructed speed is abnormal",
    0x0BE: "Time-out error while Homing",
    0x0C0: "Over speed error",
    0x0C1: "Servo error",
    0x0D8: "Deviation error",
    0x0D9: "Software stroke limit over error",
    0x0E0: "Over load",
    0x0EE: "ABS error (2)",
    ALARM_POWER_UP: "PowerUP (no error)",
}

# --------------------------------------------------------------------------- evidence summary

EVIDENCE: Final[dict[str, str]] = {
    "index table @0x0000": "VERIFIED-LIVE readable, BACKUP",
    "PNOW @0x9000": "VERIFIED-LIVE",
    "ALMC @0x9002": "VERIFIED-LIVE",
    "DSS1 @0x9005": "VERIFIED-LIVE values (SV, PEND, HEND, STP, PSFL, SFTY, b0) / DOC-CON other bits",
    "DSSE @0x9007": "VERIFIED-LIVE values (PMSS, MOVE, GHMS, PUSH, b0) / DOC-CON other bits",
    "VNOW @0x900A": "VERIFIED-LIVE: ~0.01 mm/s but quantized in ~12.5 mm/s steps; speed derived from PNOW",
    "CNOW @0x900C": "DOC-CON (mA), plausible live values",
    "DRG1 @0x0D00": "VERIFIED-LIVE readback / DOC-CON bits",
    "direct move @0x9900": "VERIFIED-LIVE (9 words: absolute, incremental, push; re-targets mid-move)",
    "position move @0x9800": "VERIFIED-LIVE (FC06 = position number starts at once)",
    "position table @0x1000": "VERIFIED-LIVE read and write (non-volatile at once); +F checksum BACKUP",
    "alarm history @0x0500": "VERIFIED-LIVE read (32 x 6 words)",
    "identification @0x0600": "BACKUP + VERIFIED-LIVE read (ASCII serial / model)",
    "soft limits @0x0700": "VERIFIED-LIVE read (+150.30 / -0.30 mm slider, +9.80 / -0.30 mm gripper)",
    "default speed @0x0812 (item 8.10 VCMD)": "VERIFIED-LIVE read (700.00 / 53.00 / 800.00 mm/s); equals MAXV on "
    "all three units, but it is the DEFAULT speed (UNVERIFIED): the speed cap comes from MAXV 7.4",
    "MAXV / MAXA / MAXD (items 7.4 / 7.5 / 7.6)": "VERIFIED on the EC-R6S rod (800.00 mm/s, 1.00 G); read by item ID",
    "ZONS @0x9013": "VERIFIED-LIVE: b12/b13/b14 = position markers LS0/LS1/LS2 on both units",
    "DOPM @0x9004": "VERIFIED-LIVE; layout differs per model (b2 = LS2 on the slider, *ALM on the gripper)",
    "coils SON/ALRS/STP/HOME": "VERIFIED-LIVE",
    "coil STOP": "VERIFIED-LIVE ON then OFF, also with PMSS = 0; self-release unverified (driver releases it)",
    "coil PMSL (write ON)": "VERIFIED-LIVE: PMSS follows; does not gate 0x9900 moves or STOP; ON mid-move stops",
    "coil SFTY (write)": "VERIFIED-LIVE: caps speed at ~100 mm/s",
    "coil CSTR": "DOC-CON, untested",
    "coils JOG+/JOG-": "VERIFIED-LIVE: run at ~100 mm/s from a parameter; never used",
    "IAI commands 0x45": "ON-WIRE (IA-OS capture: servo off 9, software reset 5/op1 1, broadcast baud 4, "
    "action mode 0x0C; servo 9 and stop 0x0B also VERIFIED with the driver's frames); VERIFIED on the EC-R6S rod: "
    "home 0x0A op1 = ORG, alarm reset 5 op1 0, action mode 0x0C (1,1) (1,0) (0,0)",
    "IAI commands 0x48": "UNVERIFIED (no EC tested uses 0x48); request addr 48 group cmd(u16) op1..op4(u32)",
    "coil PARW 0x0428": "ON-WIRE (IA-OS parameter Transfer); replayed by the driver (accepted, VERIFIED)",
    "report slave id (FC11)": "VERIFIED-LIVE (EC FF 02 00 13 slider, EC FF 0A 00 02 gripper, EC FF 00 00 02 rod "
    "after 0.2-0.4 s)",
    "position table write": "VERIFIED-LIVE: an FC10 write is non-volatile (survives a power cycle); checksum word "
    "must be set",
    "parameter write (FC10)": "VERIFIED-LIVE: RAM only until PARW 0x0428 commits it (power-cycle test + IA-OS capture)",
}


# --------------------------------------------------------------------------- word helpers


def to_i32_words(value: int) -> tuple[int, int]:
    """Split a signed 32-bit value into (high word, low word), high word at the lower address."""
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError(f"{value} does not fit in int32")
    v = value & 0xFFFFFFFF
    return v >> 16, v & 0xFFFF


def to_u32_words(value: int) -> tuple[int, int]:
    """Split an unsigned 32-bit value into (high word, low word)."""
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{value} does not fit in uint32")
    return value >> 16, value & 0xFFFF


def from_i32_words(high: int, low: int) -> int:
    """Combine (high word, low word) into a signed 32-bit value."""
    v = (high << 16) | low
    return v - (1 << 32) if v & 0x80000000 else v


def from_u32_words(high: int, low: int) -> int:
    """Combine (high word, low word) into an unsigned 32-bit value."""
    return (high << 16) | low


def to_counts(value: float, scale: float = POSITION_SCALE) -> int:
    """Convert engineering units to integer counts, rounding to nearest (e.g. 73.95 mm -> 7395).

    Raises ValueError for NaN / infinity.
    """
    if not math.isfinite(value):
        raise ValueError(f"{value!r} is not a finite number")
    return round(value / scale)


def words_to_ascii(words: list[int] | tuple[int, ...]) -> str:
    """Decode big-endian ASCII register words to printable ASCII, stripping padding (spaces / NULs).

    Every byte outside 0x20-0x7E is dropped, so the result is safe to print on any console (cp1252 included)."""
    raw = b"".join(bytes([(w >> 8) & 0xFF, w & 0xFF]) for w in words)
    return "".join(chr(b) if 0x20 <= b <= 0x7E else " " if b == 0 else "" for b in raw).strip()


def words_to_text(words: list[int] | tuple[int, ...]) -> str | None:
    """Like :func:`words_to_ascii`, but ``None`` when the words are not a text field (any byte other than printable
    ASCII or NUL): a CON controller keeps binary data where an EC keeps its model string."""
    raw = b"".join(bytes([(w >> 8) & 0xFF, w & 0xFF]) for w in words)
    if any(not (0x20 <= b <= 0x7E or b == 0) for b in raw):
        return None
    return words_to_ascii(words) or None


def printable(text: object) -> str:
    """Any value as printable ASCII (other characters dropped): safe to print on every console."""
    return "".join(c for c in str(text) if 0x20 <= ord(c) <= 0x7E)


def push_power(percent: float) -> int:
    """Push current limit percentage -> PPOW counts: ``floor(percent x 255 / 100)``, IAI's truncating scale (255 =
    100 %; DOC ME0162-12A p.5-132, 5-142, 5-144; ME0384 p.6-32): 20 % -> 51 (0x33), 30 % -> 76 (0x4C), 50 % -> 127
    (0x7F), 60 % -> 153 (0x99), 70 % -> 178 (0xB2 = :data:`PPOW_MAX`), 100 % -> 255. Whole percents use integer
    arithmetic; a fraction is truncated the same way (a 1e-9 guard absorbs float noise)."""
    if not math.isfinite(percent):
        raise ValueError(f"{percent!r} is not a finite number")
    if float(percent).is_integer():
        return int(percent) * 255 // 100
    return math.floor(percent * 255 / 100 + 1e-9)


def push_pct(raw: int) -> float:
    """PPOW counts -> percent (the inverse scale, ``raw x 100 / 255``), e.g. 0xB2 -> 69.8 %."""
    return raw * 100 / 255


def ec_series(model: str | None) -> str | None:
    """The EC series of a model string (``"EC-DS4H-150-..."`` -> ``"S4"``, ``"EC-R6S-150"`` -> ``"R6"``, ``"EC-RR8..."``
    -> ``"RR8"``, ``"EC-GRBP8M-20"`` -> ``"GRB"``, ``"EC-RTC12..."`` -> ``"RTC"``); ``None`` when not recognised. Used
    only for the minimum-speed recommendation."""
    import re

    if not model:
        return None
    text = model.upper()
    found = re.search(r"EC-D?(RR\d+|R\d|S\d|RT[BC]|GR[BC]|WEGR|WER)", text)
    return None if found is None else found.group(1)


def recommended_min_speed(model: str | None, lead_mm: float | None) -> tuple[float | None, str]:
    """(documented minimum speed in units/s, source) from the model string and the unit's lead; ``(None, why)`` when
    neither tells. A recommendation only (see :data:`MIN_SPEED_BY_SERIES_LEAD`)."""
    series = ec_series(model)
    if series is not None:
        if series in MIN_SPEED_BY_SERIES:
            return MIN_SPEED_BY_SERIES[series], f"series {series} (DOC)"
        table = MIN_SPEED_BY_SERIES_LEAD.get(series, {})
        if lead_mm is not None:
            for lead, speed in table.items():
                if abs(lead - lead_mm) < 0.005:
                    return speed, f"series {series}, lead {lead:g} mm (DOC)"
    if lead_mm is not None and lead_mm > 0:
        return round(lead_mm * MIN_SPEED_PER_LEAD, 2), f"lead {lead_mm:g} mm x {MIN_SPEED_PER_LEAD} (DOC ME0155 p.139)"
    return None, "no documented minimum known for this unit"


def position_checksum(words: list[int] | tuple[int, ...]) -> int:
    """Return the +0xF word for the 15 words +0x0..+0xE of a position entry.

    Its low byte makes the byte sum of the whole 32-byte entry 0 mod 256 (BACKUP: holds for all
    768 entries of the baseline backup; the live rows 0-3 match).
    """
    if len(words) < POSITION_WORDS_WRITABLE:
        raise ValueError(f"need {POSITION_WORDS_WRITABLE} words")
    total = sum((w >> 8) + (w & 0xFF) for w in words[:POSITION_WORDS_WRITABLE])
    return (-total) & 0xFF


# --------------------------------------------------------------------------- command profile (class 0x0F, 0x0F00)
# 20 words: PCMD s32, VCMD u32, ACMD u32, DCMD u32, INP u32, SPOW, DPOW, PPOW, CTLF, LPOW, CTF2 (u16), ZNMP s32,
# ZNLP s32. Written with FC17 (0x9000 x34 read in the same frame), then started with the PCRF coil. VERIFIED on the
# PCON-CB with an RCP6.
PROFILE_BASE: Final = 0x0F00
PROFILE_WORDS: Final = 20
PROFILE_SPOW: Final = 10  # word offsets inside the block
PROFILE_DPOW: Final = 11
PROFILE_MONITOR_WORDS: Final = 34  # 0x9000 x34 read back in the FC17 frame (the whole monitor block)

# --------------------------------------------------------------------------- CON controller name (class 0x0100)
CONTROLLER_NAME_OFFSET: Final = 0x10  # 0x8010-0x8017 on the PCON-CB: "PCON    " "CB  " "(FB)" (VERIFIED read; the split
CONTROLLER_NAME_WORDS: Final = 8  # into name / type / option is UNVERIFIED). An EC keeps its model string at 0x0620.
ALARM_RTC_STOP: Final = 0x069  # message level: real-time clock oscillation stop (PCON at power-up; re-raised while unset)
