using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace IaiActuatorDriver
{
    // Lookup tables (generated from the Python edition's data; keep the two in step).
    public static partial class Registers
    {
        /// <summary>Base address (DA) the driver expects per category id. VERIFIED (index table, also in the unit backup)</summary>
        public static readonly IReadOnlyDictionary<int, int> EXPECTED_CATEGORY_BASES = Ro(new Dictionary<int, int>
        {
            [0x000D] = 0x0D00,
            [0x0010] = 0x1000,
            [0x0101] = 0x9000,
            [0x0102] = 0x9800,
            [0x0103] = 0x9900,
        });

        /// <summary>What each directory category holds ("?" = content not understood).</summary>
        public static readonly IReadOnlyDictionary<int, string> CATEGORY_NAMES = Ro(new Dictionary<int, string>
        {
            [0x0001] = "support functions (SUPT, MVPT, SPTS, SUP2, SUP3)",
            [0x0002] = "status image (DSS1 .. SSSE)",
            [0x0003] = "controller monitor (CTL, current alarm, ...)",
            [0x0004] = "second monitor block",
            [0x0005] = "alarm history",
            [0x0006] = "identification (serials, ACCL, unit name, model)",
            [0x0007] = "axis parameters (soft limits = P1, MAXV / MAXA / MAXD, push limits, AKID)",
            [0x0008] = "parameters (user parameters 2, 5-7, 9-12, 16; BRSL, move defaults, FMNT)",
            [0x0009] = "parameters (user parameters 3, 4, 13; ORG home pattern)",
            [0x000C] = "zone boundaries ZNM / ZNL (user parameters 14, 15, 17, 18)",
            [0x000D] = "control registers DRG1 / DRG2 / DRGE / POSR (coil base)",
            [0x000F] = "active move profile",
            [0x0010] = "position table",
            [0x0012] = "?",
            [0x0013] = "PIO input map?",
            [0x0014] = "PIO output map?",
            [0x0100] = "controller system info (AXNO, PCB, FVER, UNIT)",
            [0x0101] = "monitor",
            [0x0102] = "position-number move (POSR)",
            [0x0103] = "direct numeric move",
            [0x0104] = "PCMD / PPOS mirror",
            [0x0109] = "parameter copies",
            [0x010A] = "?",
            [0x0111] = "?",
            [0x0112] = "?",
            [0x0114] = "compact monitor",
            [0x0200] = "maintenance counters",
        });

        /// <summary>Blocks with MS = 0 (members of mixed sizes): (DQ, register count) where the count is known. Member names UNVERIFIED; the counts are confirmed by reads (VERIFIED).</summary>
        public static readonly IReadOnlyDictionary<int, (int MemberCount, int Words)> KNOWN_MIXED_EXTENTS = Ro(new Dictionary<int, (int MemberCount, int Words)>
        {
            [0x0006] = (36, 72),
            [0x000F] = (13, 20),
            [0x0013] = (16, 16),
            [0x0014] = (16, 16),
            [0x0100] = (16, 32),
            [0x0101] = (22, 34),
            [0x0102] = (3, 3),
            [0x0103] = (7, 10),
            [0x0104] = (3, 4),
            [0x0114] = (20, 40),
        });

        /// <summary>Row roles of the simple-data rows 256-267 (UNVERIFIED: row 263 is the PREVIOUS forward %, the Middle % row is 266).</summary>
        public static readonly IReadOnlyDictionary<int, string> SIMPLE_DATA_ROLES = Ro(new Dictionary<int, string>
        {
            [256] = "backward end, present (real)",
            [257] = "forward end, present (real)",
            [258] = "backward end, previous (real)",
            [259] = "forward end, previous (real)",
            [260] = "backward end, present (IA-OS %)",
            [261] = "forward end, present (IA-OS %)",
            [262] = "backward end, previous (IA-OS %)",
            [263] = "forward end, previous (IA-OS %)",
            [264] = "middle, present (real)",
            [265] = "middle, previous (real)",
            [266] = "middle, present (IA-OS %)",
            [267] = "middle, previous (IA-OS %)",
        });

        /// <summary>Class 1 "support function information": no ID map, fixed member order (UNVERIFIED); 32-bit items high word first. EC #3 has 6 members (SUPT .. SUP2), the slider and the gripper 11 (up to SUP3).</summary>
        public static readonly IReadOnlyList<(string Name, int Words)> SUPPORT_MEMBERS = new (string, int)[]
        {
            ("SUPT", 2),
            ("DIAG", 1),
            ("MVPT", 1),
            ("SPTS", 1),
            ("CTYP", 1),
            ("SUP2", 2),
            ("AEHS", 2),
            ("ACHS", 2),
            ("ARSF", 2),
            ("SPTM", 2),
            ("SUP3", 2),
        };

        /// <summary>0x45 commands the driver never sends unless the caller passes allowNeverSend (allowUnsafe alone is not enough): non-volatile writes, axis number, baud, alarm-list clear, initialisation, maintenance writes, firmware update, inspection. The broadcast baud change (0x04) is sent only by an explicit opt-in.</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI_NEVER_SEND = Ro(new Dictionary<int, string>
        {
            [0x01] = "write parameters to non-volatile memory (the driver commits with the PARW coil instead)",
            [0x02] = "write positions to non-volatile memory",
            [0x03] = "change the axis number (slave address)",
            [0x04] = "change the baud rate (Elecylinder.set_baudrate opts in)",
            [0x08] = "change the ID group",
            [0x0F] = "clear the alarm list (wipes the non-volatile alarm history)",
            [0x10] = "initialise parameters (factory reset)",
            [0x11] = "change the DO type",
            [0x1D] = "initialise the absolute encoder",
            [0x20] = "write maintenance information",
            [0x21] = "write maintenance information",
            [0x22] = "write maintenance information",
            [0x23] = "write maintenance information",
            [0x24] = "write maintenance information",
            [0x25] = "write maintenance information",
            [0x26] = "write maintenance information",
            [0x27] = "write maintenance information",
            [0x28] = "write maintenance information",
            [0x29] = "write maintenance information",
            [0x2A] = "write maintenance information",
            [0x2B] = "write maintenance information",
            [0x2C] = "write maintenance information",
            [0x2D] = "write maintenance information",
            [0x2E] = "write maintenance information",
            [0x2F] = "write maintenance information",
            [0x30] = "write maintenance information",
            [0x31] = "write maintenance information",
            [0x32] = "write maintenance information",
            [0x33] = "write maintenance information",
            [0x34] = "write maintenance information",
            [0x35] = "write maintenance information",
            [0x36] = "write maintenance information",
            [0x37] = "write maintenance information",
            [0x38] = "write maintenance information",
            [0x39] = "write maintenance information",
            [0x3A] = "write maintenance information",
            [0x3B] = "write maintenance information",
            [0x3C] = "write maintenance information",
            [0x4F] = "change the maintenance update mode",
            [0x73] = "set the axis index",
            [0x90] = "switch to firmware update mode",
            [0x91] = "switch to firmware update mode",
            [0xDD] = "factory inspection command",
            [0xDE] = "factory inspection command",
            [0xDF] = "factory inspection command",
            [0xE0] = "factory inspection command",
            [0xE1] = "factory inspection command",
            [0xE2] = "factory inspection command",
            [0xE3] = "factory inspection command",
            [0xE4] = "factory inspection command",
            [0xE5] = "factory inspection command",
            [0xE6] = "factory inspection command",
            [0xE7] = "factory inspection command",
            [0xE8] = "factory inspection command",
            [0xE9] = "factory inspection command",
            [0xEA] = "factory inspection command",
            [0xEB] = "factory inspection command",
            [0xEC] = "factory inspection command",
            [0xED] = "factory inspection command",
            [0xEE] = "factory inspection command",
            [0xEF] = "factory inspection command",
            [0xF0] = "factory inspection command",
            [0xF1] = "factory inspection command",
            [0xF2] = "factory inspection command",
            [0xF3] = "clear position data",
            [0xF4] = "factory inspection command",
            [0xF5] = "factory inspection command",
            [0xF6] = "factory inspection command",
            [0xF7] = "factory inspection command",
            [0xF8] = "factory inspection command",
            [0xF9] = "factory inspection command",
            [0xFA] = "factory inspection command",
            [0xFB] = "factory inspection command",
            [0xFC] = "factory inspection command",
            [0xFD] = "factory inspection command",
            [0xFE] = "factory inspection command",
            [0xFF] = "factory inspection command",
        });

        /// <summary>0x48 commands on the never-send list (same rule as IAI_NEVER_SEND).</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI2_NEVER_SEND = Ro(new Dictionary<int, string>
        {
            [0x0001] = "update maintenance information",
            [0x0002] = "update maintenance information",
            [0x0010] = "write parameters to non-volatile memory",
            [0x0011] = "write positions to non-volatile memory",
            [0x0012] = "change the axis number",
            [0x0020] = "change the baud rate",
            [0x0021] = "change the ID group",
            [0x0040] = "clear the alarm list",
            [0x0041] = "initialise parameters",
            [0x0042] = "change the DO type",
            [0x0061] = "load-cell key command",
            [0x0080] = "initialise the absolute encoder",
            [0x0090] = "safety unit command",
            [0x0091] = "safety unit command",
            [0x0092] = "safety unit command",
            [0x0093] = "safety unit command",
            [0x00A1] = "change the drive mode",
            [0x00B3] = "change the axis ID (converter)",
            [0x1000] = "switch to firmware update mode",
            [0xFFFD] = "clear position data",
        });

        /// <summary>Function 0x47 (encoder AREG) commands: listed for completeness - this driver never builds 0x47 frames at all.</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI47_NEVER_SEND = Ro(new Dictionary<int, string>
        {
            [2] = "write the encoder buffer area",
            [3] = "write the encoder FRAM",
            [4] = "clear controller maintenance info",
            [5] = "clear encoder maintenance info",
        });

        /// <summary>0x45 rejection codes (reply byte 0x01-0x3F). Names UNVERIFIED except 0x11 (VERIFIED: home op1 = 0).</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI_REJECTIONS = Ro(new Dictionary<int, string>
        {
            [0x01] = "illegal function",
            [0x02] = "illegal data address",
            [0x03] = "illegal data value",
            [0x04] = "cannot execute: slave recovery disabled",
            [0x11] = "1st operand error",
            [0x12] = "2nd operand error",
            [0x1F] = "password mismatch",
            [0x21] = "command code error",
            [0x22] = "EEPROM write already in progress",
            [0x23] = "motion command while the servo is OFF",
            [0x24] = "motion command during homing",
            [0x25] = "software reset while the servo is ON",
            [0x26] = "motion command in TEACH mode",
            [0x27] = "invalid HOME command",
            [0x28] = "parameter initialisation before EEPROM init completed",
            [0x29] = "software reset with ABS unit and Z phase enabled",
            [0x2A] = "load-cell command without a load cell",
            [0x2B] = "load-cell command with the servo ON",
            [0x2C] = "monitoring data type changed during monitoring",
            [0x2D] = "monitoring command while monitoring is invalid",
            [0x2E] = "calendar command while the calendar is invalid",
            [0x2F] = "release of safety speed prohibited",
            [0x30] = "monitoring conditions incorrect",
            [0x31] = "battery-less absolute encoder command error",
            [0x32] = "encoder FRAM access with the servo ON",
            [0x33] = "encoder FRAM: bad page",
            [0x34] = "encoder FRAM: bad unique code",
            [0x35] = "encoder FRAM write error",
            [0x38] = "invalid command during virtual axis",
            [0x39] = "software reset during homing",
            [0x3A] = "command prohibited in monitor (PIO) mode",
            [0x3B] = "software reset while software reset is inhibited",
            [0x3C] = "servo ON after encoder FRAM access",
            [0x3D] = "inspection command error",
            [0x3E] = "inspection command error",
            [0x3F] = "inspection command error",
        });

        /// <summary>0x48 rejection codes (reply word 0x1000-0x13FF). Names UNVERIFIED.</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI2_REJECTIONS = Ro(new Dictionary<int, string>
        {
            [0x1000] = "command code error",
            [0x1001] = "1st operand error",
            [0x1002] = "2nd operand error",
            [0x1003] = "3rd operand error",
            [0x1004] = "4th operand error",
            [0x1005] = "motion command while the servo is OFF",
            [0x1006] = "motion command during homing",
            [0x1007] = "software reset while the servo is ON",
            [0x1008] = "motion command in TEACH mode",
            [0x1009] = "invalid HOME command",
            [0x100A] = "parameter initialisation before EEPROM init completed",
            [0x100B] = "load-cell command without a load cell",
            [0x100C] = "load-cell command with the servo ON",
            [0x100D] = "monitoring data type changed during monitoring",
            [0x100E] = "monitoring command while monitoring is invalid",
            [0x100F] = "monitoring conditions incorrect",
            [0x1010] = "servo monitor not available",
            [0x1011] = "drive recorder not available",
            [0x1012] = "calendar command while the calendar is invalid",
            [0x1013] = "release of safety speed prohibited",
            [0x1014] = "invalid command during virtual axis",
            [0x1015] = "software reset during homing",
            [0x1016] = "inspection command error",
            [0x1017] = "inspection command error",
            [0x101C] = "monitoring command in drive-recorder mode",
            [0x101D] = "software reset while software reset is inhibited",
            [0x101E] = "battery-less absolute encoder command error",
            [0x101F] = "movement during collision-detection reversal",
        });

        /// <summary>The driver's own short hints for 0x45 rejections a user can act on (any command).</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI_REJECTION_HINTS = Ro(new Dictionary<int, string>
        {
            [0x23] = "turn the servo on first",
            [0x24] = "wait for the home return to finish",
            [0x25] = "turn the servo off first (0x45 cmd 9 op1 0)",
            [0x26] = "the EC is in TEACH mode",
            [0x3A] = "the EC is in PIO (monitor) mode: take Modbus control first (PMSL coil, or action mode op1 = 1)",
        });

        /// <summary>The driver's own short hints for 0x48 rejections (any command).</summary>
        public static readonly IReadOnlyDictionary<int, string> IAI2_REJECTION_HINTS = Ro(new Dictionary<int, string>
        {
            [0x1005] = "turn the servo on first",
            [0x1006] = "wait for the home return to finish",
            [0x1007] = "turn the servo off first (0x45 cmd 9 op1 0)",
            [0x1008] = "the EC is in TEACH mode",
            [0x101F] = "wait for the collision-detection reversal to finish",
        });

        /// <summary>Hints that only hold for one 0x45 command: (command, rejection). The home hint is VERIFIED (op1 = 0 rejected 0x11, op1 = ORG accepted).</summary>
        public static readonly IReadOnlyDictionary<(int Command, int Rejection), string> IAI_COMMAND_REJECTION_HINTS = Ro(new Dictionary<(int Command, int Rejection), string>
        {
            [(0x0A, 0x11)] = "the home return needs op1 = the EC's home pattern ORG (item 9.16)",
        });

        /// <summary>Hints that only hold for one 0x48 command: (command, rejection). UNVERIFIED (0x48 untested).</summary>
        public static readonly IReadOnlyDictionary<(int Command, int Rejection), string> IAI2_COMMAND_REJECTION_HINTS = Ro(new Dictionary<(int Command, int Rejection), string>
        {
            [(0x0031, 0x1001)] = "the home return needs op1 = the EC's home pattern ORG (item 9.16)",
        });

        /// <summary>FC11 system code (byte 0) in words.</summary>
        public static readonly IReadOnlyDictionary<int, string> SYSTEM_CODES = Ro(new Dictionary<int, string>
        {
            [0xEC] = "EC 24 V",
            [0xED] = "EC 200 V",
            [0xAE] = "PCON",
            [0xAC] = "ACON / DCON",
            [0xAA] = "SCON",
        });

        /// <summary>Product family per FC11 system code (DOC, from IAI manuals). Any other code is a generic IAI controller and works the same way: every range and capability comes from the unit's own directory.</summary>
        public static readonly IReadOnlyDictionary<int, string> PRODUCT_FAMILIES = Ro(new Dictionary<int, string>
        {
            [0xEC] = "EC",
            [0xED] = "EC",
            [0xAE] = "PCON",
            [0xAC] = "ACON/DCON",
            [0xAA] = "SCON",
        });

        /// <summary>EC family names per (system, unit) code (from IA-OS); any other code is driven the same way. The family is NOT the actuator kind (that is AKID, item 7.33).</summary>
        public static readonly IReadOnlyDictionary<(int System, int Unit), string> EC_FAMILIES = Ro(new Dictionary<(int System, int Unit), string>
        {
            [(0xEC, 0x00)] = "EC standard",
            [(0xEC, 0x01)] = "EC slim/small",
            [(0xEC, 0x02)] = "EC dust/drip-proof",
            [(0xEC, 0x03)] = "EC high thrust",
            [(0xEC, 0x08)] = "EC(UM) ultra-compact",
            [(0xEC, 0x09)] = "EC(T) thin",
            [(0xEC, 0x0A)] = "EC(MS) \"2-size EC\"",
            [(0xEC, 0x0B)] = "EC(UC) ultra-ultra-compact",
            [(0xED, 0x00)] = "EC(200V)",
        });

        /// <summary>(system, unit) codes of IAI units that are not EC actuators.</summary>
        public static readonly IReadOnlyDictionary<(int System, int Unit), string> NOT_AN_ACTUATOR = Ro(new Dictionary<(int System, int Unit), string>
        {
            [(0xEC, 0x10)] = "CCM conveyor control motor (not an EC actuator)",
        });

        /// <summary>EC family name per FC11 unit code of system 0xEC (from IA-OS); any other code is driven the same way.</summary>
        public static readonly IReadOnlyDictionary<int, string> UNIT_CODES = Ro(new Dictionary<int, string>
        {
            [0x00] = "EC standard",
            [0x01] = "EC slim/small",
            [0x02] = "EC dust/drip-proof",
            [0x03] = "EC high thrust",
            [0x08] = "EC(UM) ultra-compact",
            [0x09] = "EC(T) thin",
            [0x0A] = "EC(MS) \"2-size EC\"",
            [0x0B] = "EC(UC) ultra-ultra-compact",
        });

        /// <summary>AKID (item 7.33) bits 0-7 -> actuator kind (VERIFIED: slider 9, gripper 4, rod 3; the other values UNVERIFIED).</summary>
        public static readonly IReadOnlyDictionary<int, string> ACTUATOR_KINDS = Ro(new Dictionary<int, string>
        {
            [0] = "slider",
            [1] = "rotary",
            [2] = "arm",
            [3] = "rod",
            [4] = "gripper",
            [5] = "other",
            [6] = "linear",
            [7] = "scalar",
            [8] = "table",
            [9] = "slider (small)",
            [10] = "rod (small)",
            [11] = "wire",
        });

        /// <summary>The user-parameter candidates IA-OS lists per (system, unit) code; a code without a row gets every mapped item.</summary>
        public static readonly IReadOnlyDictionary<(int System, int Unit), IReadOnlyList<int>> PARAMETER_CANDIDATES = Ro(new Dictionary<(int System, int Unit), IReadOnlyList<int>>
        {
            [(0xEC, 0x00)] = new[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x01)] = new[] { 1, 2, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x02)] = new[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x03)] = new[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x08)] = new[] { 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x09)] = new[] { 1, 2, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x0A)] = new[] { 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xEC, 0x0B)] = new[] { 1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
            [(0xED, 0x00)] = new[] { 1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18 },
        });

        /// <summary>What IA-OS's "User parameter edit" screen showed per unit code (the lists the visibility rules reproduce).</summary>
        public static readonly IReadOnlyDictionary<int, IReadOnlyList<int>> MODEL_PARAMETERS = Ro(new Dictionary<int, IReadOnlyList<int>>
        {
            [0x0A] = new[] { 1, 2, 3, 4, 5, 6, 7, 9, 10, 12 },
            [0x00] = new[] { 1, 2, 3, 4, 5, 6, 8, 9, 12 },
        });

        /// <summary>Kept for compatibility; no longer used by the driver (empty).</summary>
        public static readonly IReadOnlyDictionary<int, IReadOnlyList<string>> MODEL_PARAMETERS_SEEN_ON = Ro(new Dictionary<int, IReadOnlyList<string>>());

        /// <summary>The driver's curated alarm names (0x0C1, 0x0D9 and 0x0EE were seen on hardware). Levels: Alarms.AlarmLevelOf.</summary>
        public static readonly IReadOnlyDictionary<int, string> ALARM_NAMES = Ro(new Dictionary<int, string>
        {
            [0x080] = "Motion command while in SERVO-OFF condition",
            [0x083] = "Absolute positional movement command when Homing is not completed",
            [0x084] = "Motion command during Homing",
            [0x085] = "Position No. error when moving",
            [0x0A3] = "Positional instruction information error when executing",
            [0x0A7] = "The instructed deceleration is abnormal",
            [0x0AB] = "The instructed speed is abnormal",
            [0x0BE] = "Time-out error while Homing",
            [0x0C0] = "Over speed error",
            [0x0C1] = "Servo error",
            [0x0D8] = "Deviation error",
            [0x0D9] = "Software stroke limit over error",
            [0x0E0] = "Over load",
            [0x0EE] = "ABS error (2)",
            [0xFFF] = "PowerUP (no error)",
        });

        /// <summary>Evidence summary per register / command.</summary>
        public static readonly IReadOnlyDictionary<string, string> EVIDENCE = Ro(new Dictionary<string, string>
        {
            ["index table @0x0000"] = "VERIFIED readable, unit backup",
            ["PNOW @0x9000"] = "VERIFIED",
            ["ALMC @0x9002"] = "VERIFIED",
            ["DSS1 @0x9005"] = "VERIFIED values (SV, PEND, HEND, STP, PSFL, SFTY, b0) / DOC (CON manuals) other bits",
            ["DSSE @0x9007"] = "VERIFIED values (PMSS, MOVE, GHMS, PUSH, b0) / DOC (CON manuals) other bits",
            ["VNOW @0x900A"] = "VERIFIED: ~0.01 mm/s but quantized in ~12.5 mm/s steps; speed derived from PNOW",
            ["CNOW @0x900C"] = "DOC (CON manuals) (mA), plausible live values",
            ["DRG1 @0x0D00"] = "VERIFIED readback / DOC (CON manuals) bits",
            ["direct move @0x9900"] = "VERIFIED (9 words: absolute, incremental, push; re-targets mid-move)",
            ["position move @0x9800"] = "VERIFIED (FC06 = position number starts at once)",
            ["position table @0x1000"] = "VERIFIED read and write (non-volatile at once, test N3); +F checksum unit backup",
            ["alarm history @0x0500"] = "VERIFIED read (32 x 6 words)",
            ["identification @0x0600"] = "unit backup + VERIFIED read (ASCII serial / model)",
            ["soft limits @0x0700"] = "VERIFIED read (+150.30 / -0.30 mm slider, +9.80 / -0.30 mm gripper)",
            ["default speed @0x0812 (item 8.10 VCMD)"] = "VERIFIED read (700.00 / 53.00 / 800.00 mm/s); equals MAXV on all three units, but it is the DEFAULT speed (UNVERIFIED): the speed cap comes from MAXV 7.4",
            ["MAXV / MAXA / MAXD (items 7.4 / 7.5 / 7.6)"] = "VERIFIED on EC #3 (800.00 mm/s, 1.00 G); read by item ID",
            ["ZONS @0x9013"] = "VERIFIED: b12/b13/b14 = position markers LS0/LS1/LS2 on both units",
            ["DOPM @0x9004"] = "VERIFIED; layout differs per model (b2 = LS2 on the slider, *ALM on the gripper)",
            ["coils SON/ALRS/STP/HOME"] = "VERIFIED",
            ["coil STOP"] = "VERIFIED ON then OFF, also with PMSS = 0; self-release unverified (driver releases it)",
            ["coil PMSL (write ON)"] = "VERIFIED: PMSS follows; does not gate 0x9900 moves or STOP; ON mid-move stops",
            ["coil SFTY (write)"] = "VERIFIED: caps speed at ~100 mm/s (test 23)",
            ["coil CSTR"] = "DOC (CON manuals), untested",
            ["coils JOG+/JOG-"] = "VERIFIED: run at ~100 mm/s from a parameter; never used",
            ["IAI commands 0x45"] = "ON-WIRE (IA-OS capture 2026-10-28: servo off 9, software reset 5/op1 1, broadcast baud 4, action mode 0x0C; servo 9 and stop 0x0B also tested by us); VERIFIED on EC #3: home 0x0A op1 = ORG, alarm reset 5 op1 0, action mode 0x0C (1,1) (1,0) (0,0)",
            ["IAI commands 0x48"] = "UNVERIFIED (no EC tested uses 0x48); request addr 48 group cmd(u16) op1..op4(u32)",
            ["coil PARW 0x0428"] = "ON-WIRE (IA-OS parameter Transfer); replayed by us (accepted, VERIFIED)",
            ["report slave id (FC11)"] = "VERIFIED (EC FF 02 00 13 slider, EC FF 0A 00 02 gripper, EC FF 00 00 02 rod after 0.2-0.4 s)",
            ["position table write"] = "VERIFIED: an FC10 write is non-volatile (test N3); checksum word must be set",
            ["parameter write (FC10)"] = "VERIFIED: RAM only until PARW 0x0428 commits it (tests N2/N3 + capture)",
        });

        private static IReadOnlyDictionary<TKey, TValue> Ro<TKey, TValue>(IDictionary<TKey, TValue> d)
            where TKey : notnull => new ReadOnlyDictionary<TKey, TValue>(d);
    }
}
