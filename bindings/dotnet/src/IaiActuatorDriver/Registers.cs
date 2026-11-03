using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace IaiActuatorDriver
{
    /// <summary>
    /// Register map of IAI units (ELECYLINDER first; CON-family controllers share most of it): addresses, bit masks,
    /// scaling, IAI command codes and the tables that name them. The names are IAI's register mnemonics (as in the IAI
    /// manuals and the Python edition), so they keep their upper-case spelling.
    /// </summary>
    /// <remarks>
    /// Evidence words: VERIFIED (read or written on a real unit and confirmed: EC-DS4H-150 slider and EC-GRBP8M-20 gripper
    /// 2026-10-27, EC-R6S-150 rod "EC #3" 2026-10-28; "unit backup" = present in the unit's own backup file),
    /// ON-WIRE (observed on the serial line: USB capture of IA-OS talking to an EC, 2026-10-28), DOC (public IAI manuals;
    /// "DOC (CON manuals)" = documented only for CON-family controllers, ME0162) and UNVERIFIED (expected behaviour, not
    /// yet confirmed on hardware). The driver adapts to whatever unit is connected: ranges and capabilities come from the
    /// unit's own directory; these constants are the fallbacks and the names.
    /// </remarks>
    public static partial class Registers
    {
        // ---------------------------------------------------------------- scaling

        /// <summary>mm per count (PNOW, PCMD, INP, zones). VERIFIED (0x1CE3 = 73.95 mm)</summary>
        public const double POSITION_SCALE = 0.01;
        /// <summary>mm/s per count (VCMD; VNOW too, see VNOW). VERIFIED (10 mm/s measured ~9.8)</summary>
        public const double VELOCITY_SCALE = 0.01;
        /// <summary>G per count (ACMD, DCMD). VERIFIED readback (0x001E = 0.30 G) / DOC (CON manuals)</summary>
        public const double ACCEL_SCALE = 0.01;
        /// <summary>1 G in mm/s^2 as IAI defines it (ME3816)</summary>
        public const double G_MM_S2 = 9800.0;
        /// <summary>3.00 G, upper limit of ACMD (ME0162 p.5-131). DOC (CON manuals)</summary>
        public const int ACMD_MAX_RAW = 300;

        // ---------------------------------------------------------------- index table (self-description)

        /// <summary>MODL, DQNT, LQNT, BFRD, BFTD, RSV x3. VERIFIED (read live; also in the unit backup)</summary>
        public const int INDEX_HEADER = 0x0000;
        public const int INDEX_HEADER_WORDS = 8;
        /// <summary>LQNT records of DQNT words: CI MD DQ LQ DA IA BA MS. VERIFIED (read live; also in the unit backup)</summary>
        public const int INDEX_RECORDS = 0x0008;
        // Category ids (CI) in the index table and the base address (DA) the driver hard-codes for each.
        // The check self-test compares these with what the connected EC reports. VERIFIED (read live; also in the unit backup)
        /// <summary>DA 0x0500</summary>
        public const int CATEGORY_ALARM_HISTORY = 0x0005;
        /// <summary>DA 0x0600</summary>
        public const int CATEGORY_IDENTIFICATION = 0x0006;
        /// <summary>DA 0x0700 (soft limits first)</summary>
        public const int CATEGORY_LIMIT_PARAMS = 0x0007;
        /// <summary>DA 0x0800 (0x9900 defaults: INP, VCMD = default/max speed, ACMD)</summary>
        public const int CATEGORY_MOTION_PARAMS = 0x0008;
        /// <summary>DA 0x0D00, coil base BA 0x0400</summary>
        public const int CATEGORY_IO_CONTROL = 0x000D;
        /// <summary>DA 0x1000</summary>
        public const int CATEGORY_POSITION_TABLE = 0x0010;
        /// <summary>DA 0x9000</summary>
        public const int CATEGORY_MONITOR = 0x0101;
        /// <summary>DA 0x9800</summary>
        public const int CATEGORY_COMMAND1 = 0x0102;
        /// <summary>DA 0x9900</summary>
        public const int CATEGORY_COMMAND2 = 0x0103;
        /// <summary>BA of category 0x0D. VERIFIED</summary>
        public const int EXPECTED_COIL_BASE = 0x0400;
        /// <summary>DA 0x0200: DSS1, DSS2, DSSE, POSS, ZONS, DIPM, DOPM, SIPM, SSSE (ids 1-9)</summary>
        public const int CATEGORY_STATUS_IMAGE = 0x0002;
        /// <summary>DA 0x0300: CTL (item 1), current alarm (item 10), ...</summary>
        public const int CATEGORY_CONTROLLER_MONITOR = 0x0003;
        /// <summary>DA 0x0400</summary>
        public const int CATEGORY_MONITOR2 = 0x0004;
        /// <summary>DA 0x0900: user parameters 3 and 4 live here</summary>
        public const int CATEGORY_FACTORY_PARAMS = 0x0009;
        /// <summary>DA 0x0C00: zone boundaries ZNM / ZNL, 2 sets</summary>
        public const int CATEGORY_ZONES = 0x000C;
        /// <summary>DA 0x0F00: the move profile the EC applied</summary>
        public const int CATEGORY_ACTIVE_PROFILE = 0x000F;
        /// <summary>DA 0x8400: TLMC (item 1), ODOM (item 2), ...</summary>
        public const int CATEGORY_MAINTENANCE = 0x0200;
        /// <summary>DA 0x0100: SUPT, DIAG, MVPT, SPTS, CTYP, SUP2, ... (fixed member order)</summary>
        public const int CATEGORY_SUPPORT = 0x0001;
        /// <summary>DA 0x8000: FLGS, CKID, AXNO, PCB, FVER, ... (16 x 2 words)</summary>
        public const int CATEGORY_SYSTEM = 0x0100;
        // What each directory category holds ("?" = content not understood).
        // Blocks with MS = 0 (members of mixed sizes): (DQ, register count) where the count is known.
        // Member names UNVERIFIED; the counts are confirmed by reads (VERIFIED).
        /// <summary>
        /// Register extent of a directory block: (words, exact, words max). <c>Words</c> is a count that is certainly inside
        /// the block: DQ x MS x LQ when the members have one size (MS != 0), the known count for a mixed-size block of
        /// <see cref="KNOWN_MIXED_EXTENTS"/> (class 1 from <see cref="SUPPORT_MEMBERS"/>), otherwise DQ x LQ (every member is
        /// at least one register; <c>Exact</c> is then false). <c>WordsMax</c> is the largest extent the record allows (two
        /// registers per member for an unknown mixed block): a read-only browser may read up to it and mark the words past
        /// <c>Words</c> as "size uncertain" (live data sits there, e.g. 0x910F-0x9113). The unit does not reject unmapped
        /// addresses (it returns 0000), so these bounds are the only guard against reading past a block.
        /// </summary>
        public static BlockExtent CategoryExtent(int categoryId, int memberCount, int listCount, int memberSize)
        {
            if (memberSize != 0)
            {
                var words = memberCount * memberSize * listCount;
                return new BlockExtent(words, true, words);
            }

            if (categoryId == CATEGORY_SUPPORT && memberCount > 0 && memberCount <= SUPPORT_MEMBERS.Count)
            {
                // class 1 has a fixed member order with known sizes (18 words for 11 members, 8 for EC #3's 6)
                var words = SUPPORT_MEMBERS.Take(memberCount).Sum(m => m.Words) * listCount;
                return new BlockExtent(words, true, words);
            }

            if (KNOWN_MIXED_EXTENTS.TryGetValue(categoryId, out var known) && known.MemberCount == memberCount)
            {
                var words = known.Words * listCount;
                return new BlockExtent(words, true, words);
            }

            return new BlockExtent(memberCount * listCount, false, 2 * memberCount * listCount);
        }

        // ---------------------------------------------------------------- monitor block (FC03)

        public const int MONITOR_BASE = 0x9000;
        /// <summary>int32, current position, 0.01 mm. VERIFIED</summary>
        public const int PNOW = 0x9000;
        /// <summary>present alarm code, 0 = none. VERIFIED (reads 0)</summary>
        public const int ALMC = 0x9002;
        /// <summary>PIO input image. DOC (CON manuals)</summary>
        public const int DIPM = 0x9003;
        /// <summary>PIO output image. VERIFIED values; layout depends on the PIO pattern (below)</summary>
        public const int DOPM = 0x9004;
        // DOPM layout DIFFERS between models / PIO patterns (VERIFIED on both test units, tests 20 and G1-G7):
        // 3-position slider (EC-DS4H-150, model suffix MF): b0 LS0, b1 LS1, b2 LS2, b3 *ALM (1 = no alarm)
        // 2-position gripper (EC-GRBP8M-20):                b0 LS0, b1 LS1, b2 *ALM (1 = no alarm)
        // so DOPM b2 is "middle" on one unit and "no alarm" on the other. The driver therefore takes the
        // position markers from ZONS (below) and the alarm from ALMC / DSS1, never from DOPM; DOPM is kept in raw.
        /// <summary>completion output for position row 0 (both layouts)</summary>
        public const int DOPM_LS0 = 1 << 0;
        /// <summary>completion output for position row 1 (both layouts)</summary>
        public const int DOPM_LS1 = 1 << 1;
        /// <summary>3-position layout only: row 2 (middle)</summary>
        public const int DOPM_3POS_LS2 = 1 << 2;
        /// <summary>3-position layout: *ALM, 1 = no alarm</summary>
        public const int DOPM_3POS_NO_ALARM = 1 << 3;
        /// <summary>2-position layout: *ALM, 1 = no alarm (gripper: DOPM 0x0006 at 9.50 mm)</summary>
        public const int DOPM_2POS_NO_ALARM = 1 << 2;
        /// <summary>device status 1. VERIFIED values / DOC (CON manuals) bit meanings</summary>
        public const int DSS1 = 0x9005;
        /// <summary>device status 2. DOC (CON manuals)</summary>
        public const int DSS2 = 0x9006;
        /// <summary>expansion device status. VERIFIED values / DOC (CON manuals) bit meanings</summary>
        public const int DSSE = 0x9007;
        /// <summary>uint32 system status. DOC (CON manuals)</summary>
        public const int STAT = 0x9008;
        /// <summary>int32 speed, ~0.01 mm/s but quantized in ~12.48 mm/s steps on the EC (12.45 / 24.90 / 37.48 / 49.93 / 99.98 seen). VERIFIED. Too coarse for control.</summary>
        public const int VNOW = 0x900A;
        /// <summary>int32, motor current command. DOC (CON manuals) (mA); live: ~464 while pushing at 30 %</summary>
        public const int CNOW = 0x900C;
        /// <summary>int32, deviation, pulses. DOC (CON manuals)</summary>
        public const int DEVI = 0x900E;
        /// <summary>uint32, ms since power on. VERIFIED (tracks wall clock)</summary>
        public const int STIM = 0x9010;
        /// <summary>input status bits (0x0300 live). (value seen)</summary>
        public const int SIPM = 0x9012;
        /// <summary>zone / position-marker status. VERIFIED on both test units (tests 20, G1)</summary>
        public const int ZONS = 0x9013;
        /// <summary>inside zone 1 (both zones span the whole stroke on both units: reads 1)</summary>
        public const int ZONS_Z1 = 1 << 0;
        /// <summary>inside zone 2</summary>
        public const int ZONS_Z2 = 1 << 1;
        // The LS bits follow the position alone, moving or not: lit while PNOW is within the LS range (EC parameter 2,
        // 0.10 mm on both units) of the row's position, or of a push row's push end (PCMD +/- INP). Consistent with 379 of
        // 380 samples logged on hardware, e.g. gripper LS0 at 2.91 mm while moving, LS2 at 2.52 mm.
        /// <summary>near position row 0, the backward end (slider 0x1003 at 0.00 mm)</summary>
        public const int ZONS_LS0 = 1 << 12;
        /// <summary>near position row 1, the forward end (slider 0x2003 at 150.00; gripper 0x2003 at its push end 9.50 = 4.10 + 5.40 mm only, 0x0003 at 5.99 / 9.00 mm and along the push)</summary>
        public const int ZONS_LS1 = 1 << 13;
        /// <summary>near position row 2, the middle (slider 0x4003 at 75.00; also on the 2-position gripper: 0x4003 at 2.52 mm, row 2 = 2.50 mm)</summary>
        public const int ZONS_LS2 = 1 << 14;
        /// <summary>EC parameter 2 "LS signal detection range" on both units (as IA-OS shows it); width matches the log</summary>
        public const double LS_RANGE_MM = 0.10;
        /// <summary>position-complete number. DOC (CON manuals)</summary>
        public const int POSS = 0x9014;
        /// <summary>0x9000..0x9013 in one read (PNOW..ZONS). The monitor block is 34 registers on the slider and class 0x101 (22 members, mixed sizes) on the gripper: 20 fits both</summary>
        public const int STATUS_BLOCK_WORDS = 20;
        /// <summary>fallback 0x9000..0x9007 (PNOW..DSSE), all VERIFIED readable</summary>
        public const int STATUS_BLOCK_MIN_WORDS = 8;
        // DSS1 bits. Values VERIFIED (idle 0x3098 / 0x7098, after a completed move 0x7099, servo off
        // 0x6010); meanings DOC (CON manuals) (ME0162 p.4-18) unless marked otherwise.
        /// <summary>emergency stop active</summary>
        public const int DSS1_EMGS = 1 << 15;
        /// <summary>safety speed enabled. VERIFIED value (set on the test EC, probably by IA-OS Teach mode); a 50 mm/s move was NOT capped by it</summary>
        public const int DSS1_SFTY = 1 << 14;
        /// <summary>controller ready</summary>
        public const int DSS1_PWR = 1 << 13;
        /// <summary>servo on. VERIFIED (servo off/on test)</summary>
        public const int DSS1_SV = 1 << 12;
        /// <summary>push missed the work (no contact over the band). VERIFIED</summary>
        public const int DSS1_PSFL = 1 << 11;
        /// <summary>major alarm (operation-cancel / cold-start level). VERIFIED with 0x0EE / 0x090</summary>
        public const int DSS1_ALMH = 1 << 10;
        /// <summary>minor (message-level) alarm / warning. UNVERIFIED</summary>
        public const int DSS1_ALML = 1 << 9;
        // Alarm levels (Alarms, UNVERIFIED on hardware): 0x011-0x07F are MESSAGES - the axis keeps running (maintenance
        // warnings 0x048 / 0x04A / 0x04E / 0x04F, warnings 0x047 / 0x049) - so a code alone is not a fault. Fault = ALMH, or
        // a code of the level "operation cancelled" / "cold start" (Alarms.AlarmLevelOf).
        /// <summary>absolute error</summary>
        public const int DSS1_ABER = 1 << 8;
        /// <summary>CON: brake released. Live: 1 when stopped, 0 while moving -&gt; meaning unknown, unused</summary>
        public const int DSS1_BKRL = 1 << 7;
        /// <summary>pause active. VERIFIED</summary>
        public const int DSS1_STP = 1 << 5;
        /// <summary>home return complete (stays 1 with servo off on WA; 0 while homing). VERIFIED</summary>
        public const int DSS1_HEND = 1 << 4;
        /// <summary>inside the INP band; 0 while paused / servo off / after a missed push, 1 after STOP and after a push made contact. VERIFIED. It (and MEND) can rise while the axis is still moving inside the band (3.04 -&gt; 3.00 mm): "done" also needs PNOW unchanged over two polls ~0.1 s apart</summary>
        public const int DSS1_PEND = 1 << 3;
        /// <summary>EC only (MEND): 1 after a move ends in its band (or a push makes contact or misses), 0 after STOP or servo cycle. VERIFIED observation</summary>
        public const int DSS1_TARGET_REACHED = 1 << 0;
        // DSS2 bits. DOC (CON manuals)
        /// <summary>teaching mode</summary>
        public const int DSS2_MODS = 1 << 11;
        // DSSE bits. Values VERIFIED (idle 0x33C2, moving 0x33F2, homing 0x3BF2 / 0x39C2, push contact 0x37C3),
        // meanings DOC (CON manuals) (ME0162 p.4-20); names not in the manual are UNVERIFIED
        /// <summary>emergency stop input</summary>
        public const int DSSE_EMGP = 1 << 15;
        /// <summary>motor drive power cut off / low</summary>
        public const int DSSE_MPUV = 1 << 14;
        /// <summary>MANU mode (always 1 without mode switch)</summary>
        public const int DSSE_RMDS = 1 << 13;
        /// <summary>name UNVERIFIED; reads 1 with servo on</summary>
        public const int DSSE_SSON = 1 << 12;
        /// <summary>home return in progress (also during the final creep, when MOVE is 0). VERIFIED</summary>
        public const int DSSE_GHMS = 1 << 11;
        /// <summary>push in progress / holding after contact. VERIFIED</summary>
        public const int DSSE_PUSH = 1 << 10;
        /// <summary>excitation detection complete (0 during the homing creep on the EC)</summary>
        public const int DSSE_PSNS = 1 << 9;
        /// <summary>1 = PIO disabled, Modbus has control. VERIFIED value</summary>
        public const int DSSE_PMSS = 1 << 8;
        /// <summary>moving. VERIFIED, BUT it clears while still decelerating on fast moves and is 0 while paused and during the homing creep: never use it alone for "move done"</summary>
        public const int DSSE_MOVE = 1 << 5;
        /// <summary>parameter write idle / complete: the driver waits for DRGE b7 PARW = 0 AND this = 1, both before and after the PARW coil. Idle 0x33C2 / 0x32C2 on all three units has it set (VERIFIED). On a PCON controller it read 0 during the commit (VERIFIED); on the EC the capture's one sample inside the commit window still read 1 (0x33C2, +148 ms)</summary>
        public const int DSSE_PARW = 1 << 7;
        /// <summary>name UNVERIFIED; seen set while moving (live)</summary>
        public const int DSSE_DST1 = 1 << 4;
        /// <summary>EC: 1 while a push holds after contact (0x37C3). VERIFIED (test 14), but also seen for a moment during a push that then missed (G5, 6.76 mm): wait for the fingerprint to hold with the position unchanged before calling it contact</summary>
        public const int DSSE_PUSH_CONTACT = 1 << 0;

        // ---------------------------------------------------------------- control registers (FC06)

        /// <summary>device control 1. VERIFIED readback 0x1000 / 0x5000 (SON, SFTY) / DOC (CON manuals) bits</summary>
        public const int DRG1 = 0x0D00;
        /// <summary>device control 2. DOC (CON manuals)</summary>
        public const int DRG2 = 0x0D01;
        /// <summary>expansion control (bit 8 = PMSL). VERIFIED readback 0x0100</summary>
        public const int DRGE = 0x0D02;
        /// <summary>parameter write (commit): coil 0x0428; self-clearing. Capture: DRGE 0x0180 at +55 ms after the coil (b7 still set: commit in progress), 0x0100 at +235 ms (done)</summary>
        public const int DRGE_PARW = 1 << 7;
        /// <summary>software reset coil 0x0420: NEVER USED (with the servo on it raised alarm 0x090)</summary>
        public const int DRGE_SFRS = 1 << 15;
        /// <summary>position number for CSTR. DOC (CON manuals)</summary>
        public const int POSR = 0x0D03;
        public const int DRG1_SFTY = 1 << 14;
        public const int DRG1_SON = 1 << 12;
        public const int DRG1_ALRS = 1 << 8;
        public const int DRG1_STP = 1 << 5;
        public const int DRG1_HOME = 1 << 4;
        public const int DRG1_CSTR = 1 << 3;
        /// <summary>POSR: FC06 write = immediate move to that position no. VERIFIED</summary>
        public const int COMMAND1_BASE = 0x9800;

        // ---------------------------------------------------------------- direct numeric move (FC10)

        /// <summary>VERIFIED: an FC10 write starts the move at once and re-targets a move in progress; the block keeps the last written values</summary>
        public const int DIRECT_MOVE_BASE = 0x9900;
        /// <summary>int32 target (or signed distance with CTLF_INC), 0.01 mm</summary>
        public const int PCMD = 0x9900;
        /// <summary>uint32 positioning band (push band with CTLF_PUSH), 0.01 mm</summary>
        public const int INP = 0x9902;
        /// <summary>uint32 speed, 0.01 mm/s</summary>
        public const int VCMD = 0x9904;
        /// <summary>uint16 accel = decel, 0.01 G</summary>
        public const int ACMD = 0x9906;
        /// <summary>uint16 push current limit, 255 = 100 %. VERIFIED (0 for normal moves, 77 = 30 %)</summary>
        public const int PPOW = 0x9907;
        /// <summary>control flags. Resets to 0 after each move start, but a CTLF written without a move start stays pending: the driver therefore always writes it (DIRECT_MOVE_WORDS = 9)</summary>
        public const int CTLF = 0x9908;
        /// <summary>PCMD, INP, VCMD, ACMD, PPOW, CTLF. VERIFIED frame shape (batch 2)</summary>
        public const int DIRECT_MOVE_WORDS = 9;
        /// <summary>VERIFIED</summary>
        public const int CTLF_ABSOLUTE = 0x0000;
        /// <summary>push move: approach PCMD, then push over INP at PPOW. VERIFIED</summary>
        public const int CTLF_PUSH = 1 << 1;
        /// <summary>with CTLF_PUSH: push away from home (+); without it toward home (-). VERIFIED</summary>
        public const int CTLF_PUSH_AWAY = 1 << 2;
        /// <summary>incremental: PCMD is the signed distance. VERIFIED (not used by the driver)</summary>
        public const int CTLF_INC = 1 << 3;
        /// <summary>20 % - the CON manuals' push range (ME0162 p.5-131). DOC (CON manuals). On the EC the lower bound is MNPP (item 7.58, or 20 % when absent), only a RECOMMENDATION (UNVERIFIED; EC #3 ships a 10 % row): the driver warns below it and never refuses the EC's own value</summary>
        public const int PPOW_MIN = 0x33;
        /// <summary>70 % - DOC (CON manuals). The hard cap on the EC is MXPP (item 7.9: slider 60, rod 70, gripper 100 %)</summary>
        public const int PPOW_MAX = 0xB2;
        /// <summary>EC pushes at min(VCMD, 20 mm/s) (ME3816 p.260). DOC; live push at 10 ran at 10. The push speed is PSHV (item 8.15), capped by MXPH (7.8; UNVERIFIED): slider 30.00, rod 20.00 (VERIFIED ~20 mm/s), gripper 5.00 mm/s (the timeouts assume 20)</summary>
        public const double PUSH_SPEED_MAX_MM_S = 20.0;

        // ---------------------------------------------------------------- position table (FC03 / FC10)

        /// <summary>VERIFIED rows 0..2 (0.00 / 150.00 / 75.00 mm)</summary>
        public const int POSITION_TABLE_BASE = 0x1000;
        /// <summary>words per position. VERIFIED</summary>
        public const int POSITION_STRIDE = 16;
        /// <summary>+0x0..+0xE as in the ME0162 examples; +0xF is the checksum (below)</summary>
        public const int POSITION_WORDS_WRITABLE = 15;
        /// <summary>LQ of category 0x10 in the index table. VERIFIED (unit backup)</summary>
        public const int POSITION_MAX_ENTRIES = 0x300;
        /// <summary>an EC uses rows 0 (backward end), 1 (forward end), 2 (middle, 3-position mode) only: live table (VERIFIED), at most 3 positions</summary>
        public const int POSITION_ACTIVE_ROWS = 3;
        /// <summary>0x2000 page: the "simple data" store, rows 256-267. Never written by this driver. IA-OS displays these rows, not rows 0-2 (UNVERIFIED).</summary>
        public const int POSITION_SIMPLE_DATA_FIRST_ROW = 256;

        /// <summary>One past the last simple-data row (268).</summary>
        public const int POSITION_SIMPLE_DATA_END_ROW = 268;
        // Row roles (UNVERIFIED: row 263 is the PREVIOUS forward %, the Middle % row is 266). A later IA-OS Transfer
        // rewrites rows 0-2 from these rows (UNVERIFIED).
        /// <summary>PUSH, DIR, INC and the 0x0010 / 0x0020 bits seen on live rows 1 / 2</summary>
        public const int POSITION_CTLF_KNOWN = 0x003E;
        /// <summary>0x0010 on row 1 (forward end), 0x0020 on row 2 (middle). Meaning unknown (CON: servo gain set); preserved unless explicitly allowed</summary>
        public const int POSITION_CTLF_ROLE_BITS = 0x0030;
        // offsets inside an entry. VERIFIED values / DOC (CON manuals) names
        /// <summary>int32 0.01 mm</summary>
        public const int POS_PCMD = 0x0;
        /// <summary>uint32 0.01 mm</summary>
        public const int POS_INP = 0x2;
        /// <summary>uint32 0.01 mm/s</summary>
        public const int POS_VCMD = 0x4;
        /// <summary>int32 0.01 mm</summary>
        public const int POS_ZNMP = 0x6;
        /// <summary>int32 0.01 mm</summary>
        public const int POS_ZNLP = 0x8;
        /// <summary>uint16 0.01 G</summary>
        public const int POS_ACMD = 0xA;
        /// <summary>uint16 0.01 G</summary>
        public const int POS_DCMD = 0xB;
        /// <summary>uint16 push power</summary>
        public const int POS_PPOW = 0xC;
        /// <summary>uint16 load threshold</summary>
        public const int POS_LPOW = 0xD;
        /// <summary>uint16 control flags</summary>
        public const int POS_CTLF = 0xE;
        /// <summary>low byte makes the byte sum of the 32-byte entry 0 mod 256. VERIFIED (unit backup) (all 768 rows), VERIFIED values; the driver writes it with the row (test N1: 0x00C1 -&gt; 0x008F).</summary>
        public const int POS_CHECKSUM = 0xF;
        // An FC10 write to 0x1000 reaches non-volatile memory at once (test N3: row 0 survived a 24 V power cycle) and the
        // EC uses it at once - unlike parameters, which need the PARW commit (COIL_PARW) and a software reset.
        /// <summary>First register of position-table entry <paramref name="n"/> (0 .. POSITION_MAX_ENTRIES - 1).</summary>
        public static int PositionAddress(int n)
        {
            if (n < 0 || n >= POSITION_MAX_ENTRIES)
            {
                throw new ArgumentOutOfRangeException(nameof(n), n, $"position number must be 0..{POSITION_MAX_ENTRIES - 1}");
            }

            return POSITION_TABLE_BASE + (POSITION_STRIDE * n);
        }

        // ---------------------------------------------------------------- parameter / info blocks (FC03)

        /// <summary>32 entries x 6 words {detail, address, 0, code, time hi, time lo}. VERIFIED (stride and count over all 192 words). Index 0 is the NEWEST, the list ends at the first code 0, and the time is MILLISECONDS SINCE POWER-ON (the EC has no clock); FFFFFFFF = no time (UNVERIFIED)</summary>
        public const int ALARM_HISTORY_BASE = 0x0500;
        public const int ALARM_HISTORY_STRIDE = 6;
        public const int ALARM_HISTORY_ENTRIES = 32;
        /// <summary>"PowerUP No Error" marker written at every power-up. VERIFIED</summary>
        public const int ALARM_POWER_UP = 0x0FFF;
        /// <summary>class 0x200 (NV), 32-bit items by ID: 1 TLMC total moves (+1 per move start, VERIFIED 94 -&gt; 101), 2 ODOM distance in m. Other names UNVERIFIED. The item set differs per model (TTIM / TEMP at 0x840A / 0x840E on the slider and rod, 0x8408 / 0x840C on the gripper): always map by item ID</summary>
        public const int MAINTENANCE_BASE = 0x8400;
        public const int MAINTENANCE_TOTAL_MOVES_ID = 1;
        public const int MAINTENANCE_DISTANCE_ID = 2;
        /// <summary>Class 3 item 10 = present alarm code (0xEE seen with ALMC 0xEE on the gripper, VERIFIED).</summary>
        public static readonly (int ClassId, int ItemId) CURRENT_ALARM_ITEM = (CATEGORY_CONTROLLER_MONITOR, 10);
        /// <summary>User parameter 9: 0 Double, 3 Single, 4 on the 3-position slider. VERIFIED (gripper diffs).</summary>
        public static readonly (int ClassId, int ItemId) POSITION_MODE_ITEM = (CATEGORY_MOTION_PARAMS, 2);
        /// <summary>VERIFIED (read live; also in the unit backup), readable (72 words). Fixed layout (class 6 has no ID map, every member is 2 words; text is Shift-JIS, high byte first)</summary>
        public const int IDENTIFICATION_BASE = 0x0600;
        public const int IDENTIFICATION_WORDS = 0x48;
        /// <summary>(offset, words): CTS1-4 controller serial number. VERIFIED offsets.</summary>
        public static readonly (int Offset, int Words) IDENT_SERIAL = (0x00, 8);
        /// <summary>ACS1-4 actuator serial number (UNVERIFIED; the slider reads "0").</summary>
        public static readonly (int Offset, int Words) IDENT_ACTUATOR_SERIAL = (0x08, 8);
        /// <summary>PRDT u32 "manufacture information": MREV (first 4 hex digits) + FREV (last 4). UNVERIFIED</summary>
        public const int IDENT_PRDT = 0x10;
        /// <summary>ACCL u32: bits 8-11 FEX1/2/4/8 = extensibility level (0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo). UNVERIFIED; slider 0x00010110 (level 1), gripper / rod level 0</summary>
        public const int IDENT_ACCL = 0x12;
        /// <summary>BISD u32: reads like a BCD date (our guess only)</summary>
        public const int IDENT_BISD = 0x14;
        /// <summary>ANA1-3: the USER-EDITABLE unit name (renamed in IA-OS), NOT a model field (EC #3's is blank). "MWB-EC-DSRR4" on the slider.</summary>
        public static readonly (int Offset, int Words) IDENT_USER_NAME = (0x1A, 6);
        /// <summary>AS01-AS20 actuator model string, e.g. "EC-R6S-150-1-PN". VERIFIED (read live; also in the unit backup).</summary>
        public static readonly (int Offset, int Words) IDENT_MODEL = (0x20, 0x28);
        /// <summary>int32 +max, int32 -min, 0.01 mm (class 7 items 1-2). VERIFIED read (+150.30 / -0.30 on the 150 mm test EC). Read-only for this driver.</summary>
        public const int SOFT_LIMITS = 0x0700;
        /// <summary>the EC stores stroke + 0.30 mm at each end (ME3816 p.323); the driver keeps its own targets inside the nominal stroke</summary>
        public const double SOFT_LIMIT_MARGIN_MM = 0.30;
        /// <summary>class 8 parameters (NV). VERIFIED (read live; also in the unit backup)</summary>
        public const int MOTION_PARAMS_BASE = 0x0800;
        /// <summary>0x0812-0x0813 = item 8.10 VCMD, the DEFAULT command speed (UNVERIFIED), not the maximum. It equals MAXV on all three units (700.00 / 53.00 / 800.00 mm/s), but that is a factory setting, not a rule: the driver's speed cap comes from MAXV (item 7.4) and falls back to this word only when MAXV is not readable.</summary>
        public const int SPEED_LIMIT_OFFSET = 0x12;
        /// <summary>0x0812: item 8.10 VCMD, the default command speed (see SPEED_LIMIT_OFFSET).</summary>
        public const int SPEED_LIMIT = MOTION_PARAMS_BASE + SPEED_LIMIT_OFFSET;
        // Class 7 "axis information" item IDs. There is no per-model table: the driver reads these from the EC by item ID
        // through the class's ID map - addresses differ per model. Values in 0.01 units unless noted.
        /// <summary>+ soft limit (P1 + 0.30 mm)</summary>
        public const int AXIS_ITEM_LIMM = 1;
        /// <summary>- soft limit</summary>
        public const int AXIS_ITEM_LIML = 2;
        /// <summary>screw lead, 0.01 mm</summary>
        public const int AXIS_ITEM_LEAD = 3;
        /// <summary>maximum speed, 0.01 mm/s (slider 700.00, gripper 53.00, rod 800.00 - VERIFIED on EC #3)</summary>
        public const int AXIS_ITEM_MAXV = 4;
        /// <summary>maximum acceleration, 0.01 G (1.00 / 0.30 / 1.00 - VERIFIED on EC #3)</summary>
        public const int AXIS_ITEM_MAXA = 5;
        /// <summary>maximum deceleration, 0.01 G (1.00 / 0.30 / 1.00)</summary>
        public const int AXIS_ITEM_MAXD = 6;
        /// <summary>axis type bits: TYPE b8, MODE b9, PSDN b12, BUEN b18 (= P8), RINP b24</summary>
        public const int AXIS_ITEM_ATYP = 7;
        /// <summary>maximum push speed, 0.01 mm/s (30.00 / 5.00 / 20.00)</summary>
        public const int AXIS_ITEM_MXPH = 8;
        /// <summary>maximum push current, % (60 / 100 / 70): the HARD cap; 0 = no push</summary>
        public const int AXIS_ITEM_MXPP = 9;
        /// <summary>smart tuning actuator / lead numbers (FFFFFFFF = none)</summary>
        public const int AXIS_ITEM_STN1 = 15;
        /// <summary>actuator kind, bits 0-7 (ACTUATOR_KINDS); bit 8 return type</summary>
        public const int AXIS_ITEM_AKID = 33;
        /// <summary>push force conversion factor (0 = no push)</summary>
        public const int AXIS_ITEM_TCPC = 34;
        /// <summary>push force conversion offset</summary>
        public const int AXIS_ITEM_TCOA = 35;
        /// <summary>recommended minimum push, % (slider 20, gripper 50, absent on the rod -&gt; 20)</summary>
        public const int AXIS_ITEM_MNPP = 58;
        /// <summary>second push conversion factor (the driver warns before a P3 change when != 0)</summary>
        public const int AXIS_ITEM_TCP2 = 62;
        /// <summary>the push recommendation when MNPP is absent or 0 (a warning, never a refusal). UNVERIFIED</summary>
        public const double MNPP_DEFAULT_PCT = 20.0;
        /// <summary>P8 power saving: 1 = Disable (EC #3 0x05350000 -&gt; "1:Disable", as IA-OS showed it)</summary>
        public const int ATYP_BUEN = 1 << 18;
        /// <summary>minimum positioning band applies (with SUP2 CINP)</summary>
        public const int ATYP_RINP = 1 << 24;
        /// <summary>ATYP 7.7 bit 8 TYPE: rotary axis (CON No.78 "Axis action type" 1; units without AKID 7.33) - UNVERIFIED</summary>
        public const int ATYP_ROTARY = 1 << 8;
        // Class 8 / 9 / 3 / 4 item IDs used by the driver (gripper parameters mapped on hardware; the others UNVERIFIED)
        /// <summary>8.3 power-up baud code (IAI baud code: 6 = 38400 on all three units)</summary>
        public const int PARAM_ITEM_BRSL = 3;
        /// <summary>8.4 minimum delay before the EC's transmitter turns on, ms (5)</summary>
        public const int PARAM_ITEM_RTIM = 4;
        /// <summary>8.8 (b8 CORG: the driver warns before a P3 change)</summary>
        public const int PARAM_ITEM_ZRMK = 8;
        /// <summary>8.10 default command speed (0x0812)</summary>
        public const int PARAM_ITEM_VCMD = 10;
        /// <summary>8.14 control flags (P5 b6, P6 b10; default collision set b19-20)</summary>
        public const int PARAM_ITEM_CTLF = 14;
        /// <summary>8.15 push speed, 0.01 mm/s</summary>
        public const int PARAM_ITEM_PSHV = 15;
        /// <summary>8.50 (P7 b23, P10 b18, P16 b10, EXFN b13, INIT b15, SLCD b28 collision output)</summary>
        public const int PARAM_ITEM_SPIO = 50;
        /// <summary>8.54 servo monitor: MODE b0-3 (0 none, 1 = 4 ch, 2 = 8 ch, 3 = 2 ch), BFCY b16-31</summary>
        public const int PARAM_ITEM_FMNT = 54;
        /// <summary>8.119-8.127: CDC1 CDD1 CDB1 CDC2 CDD2 CDB2 CDC3 CDD3 CDB3 (UNVERIFIED).</summary>
        public static readonly IReadOnlyList<int> COLLISION_ITEMS = Array.AsReadOnly(Enumerable.Range(119, 9).ToArray());
        public const int SPIO_EXFN = 1 << 13;
        /// <summary>collision detection output: 0 alarm, 1 warning</summary>
        public const int SPIO_SLCD = 1 << 28;
        /// <summary>9.15 (P13 = b10 PSER)</summary>
        public const int DRIVER_ITEM_FSTP = 15;
        /// <summary>9.16 home pattern number (P3 = its parity); 0x45 home sends op1 = ORG &amp; 0xFFFF</summary>
        public const int DRIVER_ITEM_ORG = 16;
        /// <summary>3.1 b31-24 system, b23-16 unit, b15-0 firmware (same as FC11; faster)</summary>
        public const int CONTROLLER_ITEM_CTCL = 1;
        /// <summary>3.2 maximum position count (3 on the 3-position slider)</summary>
        public const int CONTROLLER_ITEM_MAXP = 2;
        /// <summary>3.16 highest allowed P9 value</summary>
        public const int CONTROLLER_ITEM_MXPT = 16;
        /// <summary>3.52 software reset wait, ms (slider 5500, rod / gripper 1000)</summary>
        public const int CONTROLLER_ITEM_SRWT = 52;
        /// <summary>3.67 minimum positioning band (P2 lower limit with CINP + RINP)</summary>
        public const int CONTROLLER_ITEM_MINI = 67;
        /// <summary>4.4 motor current (same unit as MCUR)</summary>
        public const int MONITOR2_ITEM_CNOW = 4;
        /// <summary>4.5 rated current (slider 1200): load % = CNOW / MCUR x 100 (UNVERIFIED)</summary>
        public const int MONITOR2_ITEM_MCUR = 5;
        /// <summary>AXNO u32 at 0x8004 (class 0x0100 member 3): axis number = address - 1. Read-only (set only with 0x45 cmd 3, which the driver never sends). VERIFIED 0 on EC #3</summary>
        public const int AXNO_OFFSET = 4;
        // Class 1 "support function information": no ID map, fixed member order (UNVERIFIED); 32-bit items high word first.
        // EC #3 has 6 members (SUPT .. SUP2), the slider and the gripper 11 (up to SUP3). Bit meanings UNVERIFIED.
        /// <summary>software reset supported</summary>
        public const int SUPT_SRST = 1 << 2;
        /// <summary>rotational INDEX MODE capability - not "rotary actuator" (that is AKID = 1)</summary>
        public const int SUPT_ROAX = 1 << 3;
        /// <summary>relative (incremental) moves</summary>
        public const int SUPT_INCM = 1 << 8;
        /// <summary>push motion</summary>
        public const int SUPT_PUSH = 1 << 11;
        /// <summary>vibration control</summary>
        public const int MVPT_VCTL = 1 << 10;
        /// <summary>collision detection (0 on EC #1-#3)</summary>
        public const int MVPT_CDAS = 1 << 13;
        /// <summary>wireless module (P7 shown only with it)</summary>
        public const int SUP2_CNWL = 1 << 23;
        /// <summary>minimum positioning band input limit (P2 &gt;= MINI)</summary>
        public const int SUP2_CINP = 1 << 24;
        /// <summary>ACR operation ready signal (P16 shown only with it)</summary>
        public const int SUP3_SORS = 1 << 5;
        /// <summary>the unit takes function 0x48 instead of 0x45 (0 / absent on EC #1-#3, which use 0x45)</summary>
        public const int SUP3_IVC1 = 1 << 16;

        // ---------------------------------------------------------------- coils (FC05: FF00 on / 0000 off)

        // Coil = coil base 0x0400 + (15 - bit) of DRG1 (0x0410.. DRG2, 0x0420.. DRGE). The base (BA = 0x0400 for
        // category 0x0D) is VERIFIED in the index table; coil meanings are DOC (CON manuals) (ME0162 p.5-80).
        /// <summary>safety speed (level): caps speed at ~100 mm/s; volatile. VERIFIED (test 23)</summary>
        public const int COIL_SFTY = 0x0401;
        /// <summary>servo on (level). VERIFIED</summary>
        public const int COIL_SON = 0x0403;
        /// <summary>alarm reset (rising edge: FF00 then 0000). VERIFIED pulse (no alarm present)</summary>
        public const int COIL_ALRS = 0x0407;
        /// <summary>pause (FF00, level: holds while ON) / resume (0000, auto-resumes). VERIFIED</summary>
        public const int COIL_STP = 0x040A;
        /// <summary>home return: 0000 then FF00 (edge), release 0000 after HEND. VERIFIED</summary>
        public const int COIL_HOME = 0x040B;
        /// <summary>start move to POSR (rising edge). DOC (CON manuals), untested on EC</summary>
        public const int COIL_CSTR = 0x040C;
        /// <summary>FF00 = Modbus control (DSSE PMSS = 1). VERIFIED (test 19). PMSS is 0 at power-up. It does NOT gate 0x9900 moves or STOP (tests 18, 19), and switching it ON mid-move STOPS the move (test 18): write it only while the axis is at rest.</summary>
        public const int COIL_PMSL = 0x0427;
        /// <summary>decel stop. VERIFIED sequence: FF00, then 0000 after the stop; stopped a Modbus-commanded move with PMSS = 0 (test 19). UNTESTED on a move a PLC commanded over PIO (a held start input might restart it after the release). Whether the EC releases it by itself (ME0162 says so for CON) is UNVERIFIED, so the driver always writes 0000 afterwards.</summary>
        public const int COIL_STOP = 0x042C;
        /// <summary>NEVER USED: runs at ~100 mm/s from an EC parameter, not our speed. VERIFIED</summary>
        public const int COIL_JOG_PLUS = 0x0416;
        /// <summary>NEVER USED (as JOG+)</summary>
        public const int COIL_JOG_MINUS = 0x0417;
        /// <summary>DRGE b1 PCRF: run the command profile at 0x0F00 (written first, FC17). VERIFIED on the PCON (self-clearing; STOP stops it; a new profile + PCRF retargets); UNVERIFIED on an EC</summary>
        public const int COIL_PCRF = 0x042E;
        /// <summary>DRGE b7 PARW: commit the parameters in RAM to non-volatile memory. ON-WIRE (IA-OS Transfer, USB capture 2026-10-28: FC10 item write, then 01 05 04 28 FF 00; no OFF is sent, DRGE reads back without b7) and VERIFIED (our replay; a committed P2 survived a reset). Without it a parameter write is RAM only (test N3). Takes effect only after a software reset / power cycle (UNVERIFIED).</summary>
        public const int COIL_PARW = 0x0428;
        /// <summary>DRGE b15 software reset. NEVER USED: pulsed with the servo on it raised alarm 0x090 "Software reset command while in SERVO-ON condition" (test R1); the driver uses 0x45 cmd 5</summary>
        public const int COIL_SFRS = 0x0420;

        // ---------------------------------------------------------------- IAI control commands

        // Function 0x45 / 0x48 command codes (IAI's control-command numbering; VERIFIED where marked, else UNVERIFIED).
        // 0x45 request addr 45 group cmd op1 op2 (u16 big-endian), reply addr 45 rejection status (ON-WIRE): a
        // non-zero rejection is a refusal that this driver never retries. 0x48 request addr 48 group cmd(u16)
        // op1..op4(u32), reply addr 48 rejection(u16) status1..4(u32) (UNVERIFIED): the command set of units with
        // SUP3 IVC1 = 1; all three ECs tested use 0x45. Never used on the safety stop path (the STOP coil is).
        /// <summary>broadcast to address 0: op1 = baud code (ModbusRtu.IaiBaudCodes), op2 = 5 (as IA-OS). ON-WIRE (00 45 00 04 00 14 00 05 = 115200) + VERIFIED; volatile (38400 after power-up)</summary>
        public const int IAI_CMD_BAUD = 0x04;
        /// <summary>op2 IA-OS sends with the baud change (ON-WIRE; meaning unknown)</summary>
        public const int IAI_BAUD_OP2 = 0x0005;
        /// <summary>op1: 0 = alarm reset, 1 = software reset (the software-reset call only)</summary>
        public const int IAI_CMD_RESET = 0x05;
        /// <summary>op1 of the alarm reset: 01 45 00 05 00 00 00 00 (VERIFIED on EC #3)</summary>
        public const int IAI_RESET_ALARM = 0x0000;
        /// <summary>op1 of the software reset: 01 45 00 05 00 01 00 00 -&gt; 01 45 00 00 (ON-WIRE, VERIFIED)</summary>
        public const int IAI_RESET_SOFTWARE = 0x0001;
        /// <summary>op1 = stored row n, op2 = speed override: the row runs at (op2 + 1) / 256 of its stored speed (255 = 100 %; ON-WIRE on the PCON-CB: op2 13 ran an 840 mm/s row at 46.0 mm/s). VERIFIED on the PCON; UNVERIFIED on an EC</summary>
        public const int IAI_CMD_MOVE_POSITION = 0x06;
        /// <summary>op1: 1 = on, 0 = off. VERIFIED (01 45 00 09 00 00 00 00 = servo OFF -&gt; 01 45 00 00)</summary>
        public const int IAI_CMD_SERVO = 0x09;
        /// <summary>op1 = ORG (item 9.16) &amp; 0xFFFF, op2 0. VERIFIED on EC #3 (01 45 00 0A 00 07 00 00); op1 = 0 is rejected 0x11 "1st operand error" (test I3)</summary>
        public const int IAI_CMD_HOME = 0x0A;
        /// <summary>decel stop (tested by us; the driver stops with the STOP coil 0x042C instead)</summary>
        public const int IAI_CMD_STOP = 0x0B;
        /// <summary>op1 1 = PC (Modbus) / IA-OS "Teaching", 0 = PIO / "Monitoring"; op2 1 = safety speed. VERIFIED on EC #3: (1,1) (1,0) (0,0); read back DSSE PMSS / DSS1 SFTY</summary>
        public const int IAI_CMD_ACTION_MODE = 0x0C;
        /// <summary>0x48 command codes are 16 bit. UNVERIFIED</summary>
        public const int IAI2_CMD_ALARM_RESET = 0x0023;
        public const int IAI2_CMD_SERVO = 0x0030;
        public const int IAI2_CMD_HOME = 0x0031;
        public const int IAI2_CMD_STOP = 0x0034;
        // Commands the driver never sends unless the caller passes allowNeverSend (allowUnsafe alone is not
        // enough): non-volatile writes, axis number, baud, alarm-list clear, initialisation, maintenance writes, firmware
        // update, inspection. The broadcast baud change (0x04) is sent only by the baud-change call, which opts in.
        // Rejection codes: the 0x45 reply byte (0x01-0x3F) and the 0x48 reply word (0x1000-0x13FF). Names UNVERIFIED
        // except 0x11 (VERIFIED: home op1 = 0, test I3).
        // Our own short hints for the rejections a driver user can act on. Hints that hold for any command are keyed by
        // the rejection code; hints that only make sense for one command by (command, rejection code).
        /// <summary>Name of an IAI rejection code: 0x45 byte (0x01-0x3F) or 0x48 word (0x1000-0x13FF).</summary>
        public static string IaiRejectionName(int function, int code)
        {
            var table = function == 0x48 ? IAI2_REJECTIONS : IAI_REJECTIONS;
            return table.TryGetValue(code, out var name) ? name : "unknown rejection code";
        }
        /// <summary>
        /// The driver's own short advice for a rejection <paramref name="code"/> of <paramref name="command"/> ("" when there
        /// is none). A command-specific hint (e.g. 0x11 on the 0x45 home, cmd 0x0A) is given only when
        /// <paramref name="command"/> is passed and matches; the generic hints hold for any command.
        /// </summary>
        public static string IaiRejectionHint(int function, int code, int? command = null)
        {
            var specific = function == 0x48 ? IAI2_COMMAND_REJECTION_HINTS : IAI_COMMAND_REJECTION_HINTS;
            var generic = function == 0x48 ? IAI2_REJECTION_HINTS : IAI_REJECTION_HINTS;
            if (command.HasValue && specific.TryGetValue((command.Value, code), out var hint))
            {
                return hint;
            }

            return generic.TryGetValue(code, out var any) ? any : string.Empty;
        }

        // ---------------------------------------------------------------- report slave id (FC11)

        // VERIFIED answers: 01 11 05 EC FF 02 00 13 C1 5D = EC 24 V, run status 0xFF, unit code 0x02, firmware
        // 0x0013 (slider; backup CTL=EC020013), EC FF 0A 00 02 (gripper) and EC FF 00 00 02 (rod, after 0.2-0.4 s).
        // The same identity is in CTCL, class 3 item 1 (0x0300 on all three units): b31-24 system, b23-16 unit, b15-0
        // firmware (VERIFIED EC02 0013 / EC0A 0002 / EC00 0002): the driver falls back to it when FC11 is silent.
        // Product family per FC11 system code (byte 0; DOC, from IAI manuals). Any other code is reported as a generic
        // IAI controller and works the same way: every range and capability comes from the unit's own directory, never
        // from this table.
        /// <summary>Family of a system code that is not in <see cref="PRODUCT_FAMILIES"/>.</summary>
        public const string GENERIC_FAMILY = "IAI controller";
        /// <summary>FC11 system codes of an ELECYLINDER (0xEC 24 V, 0xED 200 V); every other code is a CON-family controller.</summary>
        public static readonly IReadOnlyCollection<int> EC_SYSTEM_CODES = Array.AsReadOnly(new[] { 0xEC, 0xED });
        // EC family names per (system, unit) code (from IA-OS); any other code is driven the same way. The family is
        // NOT the actuator kind: unit 0x0A is the EC(MS) "2-size" family (our gripper); the kind - gripper / rod /
        // slider / rotary - is AKID, item 7.33 (ACTUATOR_KINDS). There is no "SP" family in this table.
        /// <summary>Kept for compatibility; no longer used by the driver (empty).</summary>
        public static readonly IReadOnlyCollection<int> UNIT_CODES_CONFIRMED = Array.AsReadOnly(Array.Empty<int>());
        /// <summary>Product family of an FC11 system code: "EC", "PCON", "ACON/DCON", "SCON" or "IAI controller".</summary>
        public static string ProductFamily(int? systemCode) =>
            systemCode.HasValue && PRODUCT_FAMILIES.TryGetValue(systemCode.Value, out var family) ? family : GENERIC_FAMILY;
        /// <summary>The FC11 system code is an ELECYLINDER (0xEC / 0xED); every other code is a CON-family controller.</summary>
        public static bool IsEcFamily(int? systemCode) => systemCode.HasValue && EC_SYSTEM_CODES.Contains(systemCode.Value);
        /// <summary>EC family name of an FC11 (system, unit) pair, <c>null</c> if it is not in <see cref="EC_FAMILIES"/>.</summary>
        public static string? EcFamily(int? systemCode, int? unitCode)
        {
            if (!systemCode.HasValue || !unitCode.HasValue)
            {
                return null;
            }

            return EC_FAMILIES.TryGetValue((systemCode.Value, unitCode.Value), out var name) ? name : null;
        }
        // AKID (item 7.33) bits 0-7 -> actuator kind (VERIFIED: slider 9, gripper 4, rod 3; the other values UNVERIFIED).
        /// <summary>rotary iff KIND = 1: positions in deg, speeds in deg/s</summary>
        public const int AKID_ROTARY = 1;
        /// <summary>
        /// Position unit of an axis from its AKID kind: "deg" for a rotary axis (1), "mm" for everything else (an unknown or
        /// unread kind included). Speeds are then in units/s; accelerations stay in G.
        /// </summary>
        public static string AxisUnits(int? actuatorKind) => actuatorKind == AKID_ROTARY ? "deg" : "mm";
        /// <summary>
        /// As <see cref="AxisUnits(int?)"/>; a unit without AKID (CON controllers) is rotary when its ATYP 7.7 TYPE bit 8 is
        /// set (<paramref name="rotaryAtyp"/>). AKID wins when present.
        /// </summary>
        public static string AxisUnits(int? actuatorKind, bool? rotaryAtyp) =>
            actuatorKind.HasValue ? AxisUnits(actuatorKind) : (rotaryAtyp == true ? "deg" : "mm");
        /// <summary>As <see cref="AxisKindName(int?)"/>; without AKID, "rotary" when ATYP 7.7 bit 8 says so.</summary>
        public static string AxisKindName(int? actuatorKind, bool? rotaryAtyp) =>
            actuatorKind.HasValue ? AxisKindName(actuatorKind) : (rotaryAtyp == true ? ACTUATOR_KINDS[AKID_ROTARY] : "unknown");
        /// <summary>AKID kind in words ("unknown" when not read, "kind &lt;n&gt;" for a value outside the table).</summary>
        public static string AxisKindName(int? actuatorKind)
        {
            if (!actuatorKind.HasValue)
            {
                return "unknown";
            }

            return ACTUATOR_KINDS.TryGetValue(actuatorKind.Value, out var name)
                ? name
                : "kind " + actuatorKind.Value.ToString(CultureInfo.InvariantCulture);
        }
        // The user-parameter candidates IA-OS lists per (system, unit) code. Some are then hidden by runtime rules (the
        // parameter visibility rules); those rules predict exactly what IA-OS showed on the gripper and on the rod. A
        // code without a row gets every mapped item.
        // What IA-OS's "User parameter edit" screen showed (the lists the visibility rules must reproduce).
        // MODEL_PARAMETERS_SEEN_ON is kept for compatibility; no longer used by the driver.
        // Alarm names for codes this driver may meet (IAI's alarm names; 0x0C1, 0x0D9 and 0x0EE were seen on hardware).
        // Levels: Alarms.AlarmLevelOf.

        // ---------------------------------------------------------------- evidence summary

        // ---------------------------------------------------------------- word helpers

        /// <summary>Split a signed 32-bit value into (high word, low word), high word at the lower address.</summary>
        public static (ushort High, ushort Low) ToI32Words(long value)
        {
            if (value < int.MinValue || value > int.MaxValue)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "does not fit in int32");
            }

            var v = unchecked((uint)(int)value);
            return ((ushort)(v >> 16), (ushort)(v & 0xFFFF));
        }
        /// <summary>Split an unsigned 32-bit value into (high word, low word).</summary>
        public static (ushort High, ushort Low) ToU32Words(long value)
        {
            if (value < 0 || value > uint.MaxValue)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "does not fit in uint32");
            }

            return ((ushort)(value >> 16), (ushort)(value & 0xFFFF));
        }
        /// <summary>Combine (high word, low word) into a signed 32-bit value.</summary>
        public static int FromI32Words(int high, int low) => unchecked((int)FromU32Words(high, low));
        /// <summary>Combine (high word, low word) into an unsigned 32-bit value.</summary>
        public static uint FromU32Words(int high, int low) => ((uint)(high & 0xFFFF) << 16) | (uint)(low & 0xFFFF);
        /// <summary>
        /// Convert engineering units to integer counts, rounding to nearest (half to even, e.g. 73.95 mm -> 7395).
        /// Throws for NaN / infinity.
        /// </summary>
        public static long ToCounts(double value, double scale = POSITION_SCALE)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "not a finite number");
            }

            return (long)Math.Round(value / scale, MidpointRounding.ToEven);
        }
        /// <summary>
        /// Decode big-endian ASCII register words to printable ASCII, stripping padding (spaces / NULs). Every byte outside
        /// 0x20-0x7E is dropped (NUL becomes a space before trimming), so the result is safe to print on any console.
        /// </summary>
        public static string WordsToAscii(IEnumerable<ushort> words)
        {
            var sb = new StringBuilder();
            foreach (var w in words)
            {
                AppendPrintable(sb, (byte)(w >> 8));
                AppendPrintable(sb, (byte)(w & 0xFF));
            }

            return sb.ToString().Trim(' ');
        }
        /// <summary>
        /// Like <see cref="WordsToAscii"/>, but <c>null</c> when the words are not a text field (any byte other than printable
        /// ASCII or NUL) or are blank: a CON controller keeps binary data where an EC keeps its model string.
        /// </summary>
        public static string? WordsToText(IEnumerable<ushort> words)
        {
            var list = words.ToList();
            foreach (var w in list)
            {
                if (!IsTextByte((byte)(w >> 8)) || !IsTextByte((byte)(w & 0xFF)))
                {
                    return null;
                }
            }

            var text = WordsToAscii(list);
            return text.Length > 0 ? text : null;
        }
        /// <summary>Any text as printable ASCII (other characters dropped): safe to print on every console.</summary>
        public static string Printable(string? text)
        {
            if (string.IsNullOrEmpty(text))
            {
                return string.Empty;
            }

            var sb = new StringBuilder(text!.Length);
            foreach (var c in text)
            {
                if (c >= 0x20 && c <= 0x7E)
                {
                    sb.Append(c);
                }
            }

            return sb.ToString();
        }
        /// <summary>
        /// Push current limit percentage -> PPOW counts (255 = 100 %), rounded half to even like the Python edition:
        /// 20 % -> 51, 70 % -> 178, 30 % -> 76 (76.5; a unit was seen holding 77 for 30 %).
        /// </summary>
        public static int PushPower(double percent) => (int)Math.Round(percent * 255 / 100, MidpointRounding.ToEven);
        /// <summary>
        /// The +0xF word for the 15 words +0x0..+0xE of a position entry: its low byte makes the byte sum of the whole 32-byte
        /// entry 0 mod 256 (holds for all 768 entries of a unit backup; the live rows match).
        /// </summary>
        public static int PositionChecksum(IReadOnlyList<ushort> words)
        {
            if (words is null || words.Count < POSITION_WORDS_WRITABLE)
            {
                throw new ArgumentException($"need {POSITION_WORDS_WRITABLE} words", nameof(words));
            }

            var total = 0;
            for (var i = 0; i < POSITION_WORDS_WRITABLE; i++)
            {
                total += (words[i] >> 8) + (words[i] & 0xFF);
            }

            return -total & 0xFF;
        }

        private static bool IsTextByte(byte b) => (b >= 0x20 && b <= 0x7E) || b == 0;

        private static void AppendPrintable(StringBuilder sb, byte b)
        {
            if (b >= 0x20 && b <= 0x7E)
            {
                sb.Append((char)b);
            }
            else if (b == 0)
            {
                sb.Append(' ');
            }
        }

        // ---------------------------------------------------------------- command profile (class 0x0F, 0x0F00)

        // 20 words: PCMD s32, VCMD u32, ACMD u32, DCMD u32, INP u32, SPOW, DPOW, PPOW, CTLF, LPOW, CTF2 (u16), ZNMP s32,
        // ZNLP s32. Written with FC17 (0x9000 x34 read in the same frame), then started with the PCRF coil. VERIFIED on the PCON.
        public const int PROFILE_BASE = 0x0F00;
        public const int PROFILE_WORDS = 20;
        /// <summary>word offsets inside the block</summary>
        public const int PROFILE_SPOW = 10;
        public const int PROFILE_DPOW = 11;
        /// <summary>0x9000 x34 read back in the FC17 frame (the whole monitor block)</summary>
        public const int PROFILE_MONITOR_WORDS = 34;

        // ---------------------------------------------------------------- CON controller name (class 0x0100)

        /// <summary>0x8010-0x8017 on the PCON: "PCON    " "CB  " "(FB)" (VERIFIED read; the split</summary>
        public const int CONTROLLER_NAME_OFFSET = 0x10;
        /// <summary>into name / type / option is UNVERIFIED). An EC keeps its model string at 0x0620.</summary>
        public const int CONTROLLER_NAME_WORDS = 8;
        /// <summary>message level: real-time clock oscillation stop (PCON at power-up; re-raised while unset)</summary>
        public const int ALARM_RTC_STOP = 0x069;
    }

    /// <summary>Register extent of a directory block (see <see cref="Registers.CategoryExtent"/>).</summary>
    /// <param name="Words">A register count certainly inside the block.</param>
    /// <param name="Exact">True when <paramref name="Words"/> is the block's exact size.</param>
    /// <param name="WordsMax">The largest extent the record allows.</param>
    public readonly record struct BlockExtent(int Words, bool Exact, int WordsMax);
}
