using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver
{
    /// <summary>
    /// An EC user parameter (numbered as IA-OS shows it) and where it lives (class.item; kind "mm" / "bit" / "parity" /
    /// "enum" / "multi_io"). <see cref="ReadOnly"/> parameters are decoded but never written by this driver;
    /// <see cref="Entry"/> picks a multi-entry class's entry (zone sets); <see cref="ItemId2"/> is the second item of a pair (P11).
    /// </summary>
    public sealed record ParamDef(int No, string Name, int ClassId, int ItemId, string Kind)
    {
        public string Unit { get; init; } = string.Empty;

        public int? Bit { get; init; }

        public double OffsetMm { get; init; }

        public IReadOnlyDictionary<int, string>? Choices { get; init; }

        public string Encoding { get; init; } = string.Empty;

        public bool ReadOnly { get; init; }

        public int Entry { get; init; }

        public int? ItemId2 { get; init; }

        public string Evidence { get; init; } = string.Empty;

        public string Note { get; init; } = string.Empty;

        /// <summary>"class.item" (class in hex above 9), "[entry]" for a second entry, "+class.item2" for a pair.</summary>
        public string Key
        {
            get
            {
                var cls = ClassId > 9 ? ClassId.ToString("X", CultureInfo.InvariantCulture) : ClassId.ToString(CultureInfo.InvariantCulture);
                return $"{cls}.{ItemId}" + (Entry != 0 ? $"[{Entry}]" : string.Empty) + (ItemId2.HasValue ? $"+{cls}.{ItemId2.Value}" : string.Empty);
            }
        }
    }

    /// <summary>One EC user parameter decoded from the unit's items, with the per-model flags (<see cref="UserParameters.Annotate"/>).</summary>
    public sealed record DecodedParameter
    {
        public int No { get; init; }

        public string Name { get; init; } = string.Empty;

        public string Key { get; init; } = string.Empty;

        public string Unit { get; init; } = string.Empty;

        public string Encoding { get; init; } = string.Empty;

        public string Evidence { get; init; } = string.Empty;

        public bool ReadOnly { get; init; }

        public string Note { get; init; } = string.Empty;

        /// <summary>The item is in this unit's ID map.</summary>
        public bool Found { get; init; }

        public string? Address { get; init; }

        public string? RawHex { get; init; }

        /// <summary>double (mm) or int (number / choice); <c>null</c> when not found or not settable.</summary>
        public object? Value { get; init; }

        public string Text { get; init; } = string.Empty;

        /// <summary>P3: the ORG home pattern (item 9.16).</summary>
        public uint? Pattern { get; init; }

        // ---- per-model flags (Annotate)

        /// <summary>IA-OS's screen lists it for this unit.</summary>
        public bool Listed { get; init; }

        /// <summary>Kept for compatibility: an IA-OS parameter list is recorded for the unit code (= <see cref="ParameterModel.Known"/>).</summary>
        public bool VerifiedForModel { get; init; }

        /// <summary>P3 / P4: written with the home warning.</summary>
        public bool HomeParameter { get; init; }

        /// <summary>Found, mapped, listed, not read-only and settable.</summary>
        public bool Writable { get; init; }

        public IReadOnlyDictionary<string, string> Choices { get; init; } = new Dictionary<string, string>();

        public bool? ShownByIaos { get; init; }

        public string HiddenReason { get; init; } = string.Empty;

        public string ModelNote { get; init; } = string.Empty;
    }

    /// <summary>Whether IA-OS's parameter screen shows a parameter on this unit (<c>null</c>: an input was not readable).</summary>
    public sealed record ParameterVisibility(bool? Shown, string Reason);

    /// <summary>Which user parameters IA-OS's screen lists for this unit (<see cref="UserParameters.ModelParameterList"/>).</summary>
    public sealed record ParameterModel
    {
        public int? UnitCode { get; init; }

        public int? SystemCode { get; init; }

        public string? ModelName { get; init; }

        public string? Family { get; init; }

        /// <summary>An IA-OS parameter list is recorded for the (system, unit) code (<see cref="Registers.PARAMETER_CANDIDATES"/>).</summary>
        public bool Known { get; init; }

        public IReadOnlyList<int> Candidates { get; init; } = Array.Empty<int>();

        public IReadOnlyList<int> Listed { get; init; } = Array.Empty<int>();

        public IReadOnlyList<(int No, string Reason)> Hidden { get; init; } = Array.Empty<(int, string)>();

        /// <summary>Kept for compatibility: equals <see cref="Known"/>.</summary>
        public bool Verified { get; init; }

        /// <summary>Kept for compatibility; no longer used by the driver (always empty).</summary>
        public IReadOnlyList<string> SeenOn { get; init; } = Array.Empty<string>();

        public string Note { get; init; } = string.Empty;

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["unit_code"] = UnitCode,
            ["system_code"] = SystemCode,
            ["model_name"] = ModelName,
            ["family"] = Family,
            ["known"] = Known,
            ["candidates"] = Candidates.ToList(),
            ["listed"] = Listed.ToList(),
            ["hidden"] = Hidden.Select(h => (object?)new Dictionary<string, object?> { ["no"] = h.No, ["reason"] = h.Reason }).ToList(),
            ["verified"] = Verified,
            ["seen_on"] = SeenOn.ToList(),
            ["note"] = Note,
        };
    }

    /// <summary>
    /// The EC user parameters P1-P18: definitions, decoding, encoding and the per-model rules (no I/O). P1-P7, P9, P10, P12
    /// VERIFIED on the EC gripper (changed one at a time in IA-OS, registers compared by class + item ID; the same items
    /// decode on the rod); P3 decodes by the PARITY of the ORG home pattern; P8, P11, P13-P18 are UNVERIFIED locations
    /// and read-only here.
    /// </summary>
    public static class UserParameters
    {
        private const string ParamEvidence =
            "VERIFIED on the EC gripper (fw 0002): changed one at a time in IA-OS and the registers compared; same item IDs assumed elsewhere";

        private const string UnverifiedItem = "UNVERIFIED: location expected, never tested on hardware (IA-OS hid it on all units tested)";
        private const string ReadOnlyUnverified = "read-only in this driver: location expected, never tested on hardware";

        private static readonly IReadOnlyDictionary<int, string> Enable = new Dictionary<int, string> { [0] = "0:Disable", [1] = "1:Enable" };

        /// <summary>P11 "Multi I/O function" (UNVERIFIED): (byte 1 of FI00 8.25, byte 1 of FO00 8.29) -&gt; the number IA-OS shows.</summary>
        public static readonly IReadOnlyDictionary<(int, int), int> MULTI_IO_PAIRS = new Dictionary<(int, int), int>
        {
            [(0, 0)] = 0, [(15, 15)] = 1, [(13, 0)] = 2, [(0, 18)] = 3, [(0, 32)] = 4, [(0, 87)] = 5, [(22, 28)] = 6, [(26, 43)] = 7,
        };

        /// <summary>P1-P18.</summary>
        public static readonly IReadOnlyList<ParamDef> USER_PARAMETERS = new[]
        {
            new ParamDef(1, "Operation range adjustment", 7, 1, "mm") { Unit = "mm", OffsetMm = -0.30, Encoding = "s32 0.01 mm, raw = (P1 + 0.30 mm) x 100 (the upper soft limit)" },
            new ParamDef(2, "Auto-switch \"LS\" signal detection range adjustment", 8, 9, "mm") { Unit = "mm", Encoding = "s32 0.01 mm" },
            new ParamDef(3, "HOME direction change", 9, 16, "parity")
            {
                Choices = new Dictionary<int, string> { [1] = "1:Forward", [0] = "0:Reverse" },
                Encoding = "ORG home pattern number (item 9.16): parity = direction (odd = 1:Forward, even = 0:Reverse); a change writes ORG -1 / +1; not settable when ORG is 0 or FFFFFFFF",
                Evidence = "VERIFIED on the gripper (IA-OS's 1:Forward <-> 0:Reverse changed ORG 7 <-> 8); the parity rule for other patterns is UNVERIFIED (EC #3 holds 7)",
            },
            new ParamDef(4, "HOME position adjustment", 9, 21, "mm") { Unit = "mm", Encoding = "s32 0.01 mm" },
            new ParamDef(5, "Smooth accel/decel setting", 8, 14, "bit") { Bit = 6, Choices = Enable, Encoding = "bit 6: 1 = Enable" },
            new ParamDef(6, "Current control setting while stop", 8, 14, "bit") { Bit = 10, Choices = Enable, Encoding = "bit 10: 1 = Enable" },
            new ParamDef(7, "Wireless function setting", 8, 50, "bit") { Bit = 23, Choices = new Dictionary<int, string> { [0] = "0:Enable", [1] = "1:Disable" }, Encoding = "bit 23, inverted sense: 0 = Enable, 1 = Disable" },
            new ParamDef(8, "Power-saving setting", 7, 7, "bit")
            {
                Bit = 18,
                Choices = new Dictionary<int, string> { [0] = "0:Enable", [1] = "1:Disable" },
                Encoding = "ATYP item 7.7 bit 18 BUEN: 0 = Enable, 1 = Disable",
                ReadOnly = true,
                Evidence = "UNVERIFIED location; fits EC #3 (ATYP 0x05350000, IA-OS showed 1:Disable)",
                Note = "read-only until hardware-verified: a P8 change is expected to also rewrite the STN1 (7.15) tuning data, set 8.50 b15 INIT and reset CLD0 / CLD1 to FFFFFFFF (speeds / accelerations re-initialised)",
            },
            new ParamDef(9, "Solenoid operation type selection", 8, 2, "enum")
            {
                Choices = new Dictionary<int, string> { [0] = "0:Double", [3] = "3:Single", [4] = "4:3-position", [5] = "5:Double (Multi I/O)" },
                Encoding = "enum = the number IA-OS shows (valid set by extensibility level, then <= MXPT 3.16)",
            },
            new ParamDef(10, "LED lighting method auto SW setting", 8, 50, "bit") { Bit = 18, Choices = Enable, Encoding = "bit 18: 1 = Enable" },
            new ParamDef(11, "Multi I/O function selection", 8, 25, "multi_io")
            {
                ItemId2 = 29,
                Choices = new Dictionary<int, string>
                {
                    [0] = "0:Not exist", [1] = "1:Homing", [2] = "2:Pause", [3] = "3:Zone 1", [4] = "4:Miss-pressing", [5] = "5:Minor fault", [6] = "6:AUTO/MANU switching", [7] = "7:Move middle point",
                },
                Encoding = "byte 1 (bits 8-15) of FI00 8.25 and FO00 8.29 as a pair",
                ReadOnly = true,
                Evidence = UnverifiedItem,
                Note = ReadOnlyUnverified,
            },
            new ParamDef(12, "Move command type", 8, 1, "bit") { Bit = 16, Choices = new Dictionary<int, string> { [0] = "0:Continuous energization", [1] = "1:Instantaneous energization" }, Encoding = "bit 16: 1 = instantaneous (momentary)" },
            new ParamDef(13, "Push back error", 9, 15, "bit") { Bit = 10, Choices = Enable, Encoding = "FSTP item 9.15 bit 10 PSER", ReadOnly = true, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
            new ParamDef(14, "Zone output position (1) +", 0x0C, 1, "mm") { Unit = "mm", Encoding = "ZNM 0xC.1 entry 0, s32 0.01 mm", ReadOnly = true, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
            new ParamDef(15, "Zone output position (1) -", 0x0C, 2, "mm") { Unit = "mm", Encoding = "ZNL 0xC.2 entry 0, s32 0.01 mm", ReadOnly = true, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
            new ParamDef(16, "ACR operation ready signal selection", 8, 50, "bit") { Bit = 10, Choices = Enable, Encoding = "SPIO item 8.50 bit 10 ORSS", ReadOnly = true, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
            new ParamDef(17, "Zone output position (2) +", 0x0C, 1, "mm") { Unit = "mm", Encoding = "ZNM 0xC.1 entry 1, s32 0.01 mm", ReadOnly = true, Entry = 1, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
            new ParamDef(18, "Zone output position (2) -", 0x0C, 2, "mm") { Unit = "mm", Encoding = "ZNL 0xC.2 entry 1, s32 0.01 mm", ReadOnly = true, Entry = 1, Evidence = UnverifiedItem, Note = ReadOnlyUnverified },
        };

        /// <summary>P-number -&gt; definition.</summary>
        public static readonly IReadOnlyDictionary<int, ParamDef> BY_NO = USER_PARAMETERS.ToDictionary(p => p.No);

        /// <summary>Home direction / home offset (ABS error 0x0EE after a reset on an absolute unit; VERIFIED on the gripper).</summary>
        public static readonly IReadOnlyCollection<int> HOME_PARAMETERS = new[] { 3, 4 };

        /// <summary>Classes holding named parameters: 7, 8, 9, 0x0C.</summary>
        public static readonly IReadOnlyList<int> PARAMETER_CLASSES_NAMED = USER_PARAMETERS.Select(p => p.ClassId).Distinct().OrderBy(c => c).ToArray();

        /// <summary>The classes the parameter rules read: class 3 (MXPT, MINI) + the named classes.</summary>
        public static readonly IReadOnlyList<int> VISIBILITY_CLASSES = new[] { Registers.CATEGORY_CONTROLLER_MONITOR }.Concat(PARAMETER_CLASSES_NAMED).ToArray();

        public const string RAM_ONLY_NOTE =
            "RAM ONLY - NOT SAVED (commit=false): a parameter written over Modbus (FC10) changes the unit's RAM only and reverts to the committed value at the next power-up (hardware test N3: P2 0.20 mm went back to 0.10 mm after a 24 V power cycle); probably also at a software reset (untested). A later commit (PARW) saves it too.";

        public const string COMMIT_NOTE =
            "Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the coil IA-OS's Transfer writes (ON-WIRE). PARW saves every parameter value in RAM, including earlier uncommitted writes. The new value is in the register at once, but the unit applies it only after a software reset or a power cycle (IA-OS also asks for a software reset after a parameter change).";

        public const string HOME_WARNING =
            "P3 / P4 are home parameters. After a committed P4 change + software reset the WA (battery-less absolute) gripper came up with alarm 0x0EE 'ABS error (2)' (home lost); only an alarm reset + home return cleared it. Expect the same for P3 / P4 on any absolute (WA) unit; an incremental unit loses its home at every software reset anyway. Either way: alarm reset (if any) + home return before the next move.";

        /// <summary>Items the driver knows something about beyond the user parameters.</summary>
        public static readonly IReadOnlyDictionary<(int ClassId, int ItemId), string> ITEM_NOTES = new Dictionary<(int, int), string>
        {
            [(0x0003, 1)] = "CTCL: system / unit code + firmware (same as FC11)",
            [(0x0003, 2)] = "MAXP: maximum position count (3 = 3-position)",
            [(0x0003, 10)] = "present alarm code (followed ALMC 0x0EE on the gripper)",
            [(0x0003, 16)] = "MXPT: highest allowed P9 value",
            [(0x0003, 52)] = "SRWT: software reset wait, ms",
            [(0x0003, 67)] = "MINI: minimum positioning band (P2 lower limit with CINP + RINP)",
            [(0x0004, 4)] = "CNOW: motor current",
            [(0x0004, 5)] = "MCUR: rated current (IA-OS shows CNOW / MCUR x 100 %)",
            [(0x0007, 1)] = "LIMM: upper soft limit = P1 + 0.30 mm (0.01 mm)",
            [(0x0007, 2)] = "LIML: lower soft limit (0.01 mm)",
            [(0x0007, 3)] = "LEAD: screw lead (0.01 mm)",
            [(0x0007, 4)] = "MAXV: maximum speed (0.01 mm/s; the driver's speed cap)",
            [(0x0007, 5)] = "MAXA: maximum acceleration (0.01 G; the driver's accel cap)",
            [(0x0007, 6)] = "MAXD: maximum deceleration (0.01 G; the driver's decel cap)",
            [(0x0007, 7)] = "ATYP: axis type bits (b8 rotary, b18 BUEN = P8, b24 RINP)",
            [(0x0007, 8)] = "MXPH: maximum push speed (0.01 mm/s)",
            [(0x0007, 9)] = "MXPP: maximum push current % (the hard push cap; 0 = no push)",
            [(0x0007, 15)] = "STN1: smart tuning actuator / lead numbers",
            [(0x0007, 33)] = "AKID: actuator kind (bits 0-7)",
            [(0x0007, 34)] = "TCPC: push force conversion factor (0 = no push)",
            [(0x0007, 35)] = "TCOA: push force conversion offset",
            [(0x0007, 58)] = "MNPP: recommended minimum push % (a warning, not a limit)",
            [(0x0008, 3)] = "BRSL: power-up baud code (6 = 38400)",
            [(0x0008, 4)] = "RTIM: min. delay before the unit's transmitter turns on, ms",
            [(0x0008, 10)] = "VCMD: default command speed (0x0812; not the maximum - that is MAXV 7.4)",
            [(0x0008, 11)] = "ACMD: default acceleration (0.01 G)",
            [(0x0008, 15)] = "PSHV: push speed (0.01 mm/s, capped by MXPH)",
            [(0x0008, 54)] = "FMNT: servo monitor mode (b0-3, 0 = off) / buffering cycle (b16-31)",
            [(0x0009, 16)] = "ORG: home pattern number (P3 = parity); 0x45 home op1",
            [(0x000C, 1)] = "zone boundary ZNM (0.01 mm)",
            [(0x000C, 2)] = "zone boundary ZNL (0.01 mm)",
        };

        private static readonly IReadOnlyDictionary<int, int[]> P9Valid = new Dictionary<int, int[]>
        {
            [0] = new[] { 0, 3 }, [1] = new[] { 0, 3, 4 }, [2] = new[] { 0, 3, 4, 5 }, [3] = new[] { 0, 3, 4 }, [4] = new[] { 0, 3, 4, 5 },
        };

        /// <summary>P3 can be changed only when the ORG pattern (item 9.16) is neither 0 nor FFFFFFFF (UNVERIFIED).</summary>
        public static bool HomePatternSettable(uint? raw) => raw.HasValue && raw.Value != 0 && raw.Value != 0xFFFFFFFF;

        /// <summary>ACCL (0x0612) bits 8-11: 0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo (UNVERIFIED).</summary>
        public static int? ExtensibilityLevel(uint? accl) => accl.HasValue ? (int)((accl.Value >> 8) & 0xF) : (int?)null;

        /// <summary>The valid P9 values on this unit (level / EXFN / MXPT; <c>null</c> when the level is unknown; UNVERIFIED).</summary>
        public static IReadOnlyList<int>? P9ValidValues(int? level, bool? exfn, uint? mxpt)
        {
            if (!level.HasValue)
            {
                return null;
            }

            IEnumerable<int> valid = P9Valid.TryGetValue(level.Value, out var v) ? v : new[] { 0, 3 };
            if (level.Value >= 2 && exfn == false)
            {
                valid = new[] { 0, 3 };
            }

            if (mxpt.HasValue)
            {
                valid = valid.Where(x => x <= mxpt.Value);
            }

            return valid.ToArray();
        }

        /// <summary>Number -&gt; label of a choice parameter (empty for mm).</summary>
        public static IReadOnlyDictionary<int, string> ChoicesOf(ParamDef p) =>
            p.Kind == "mm" || p.Choices is null ? new Dictionary<int, string>() : p.Choices;

        /// <summary>One parameter decoded from its class item(s) (no flags; <see cref="Annotate"/> adds them).</summary>
        public static DecodedParameter Decode(ParamDef p, ClassItem? item, ClassItem? item2 = null)
        {
            var d = new DecodedParameter
            {
                No = p.No,
                Name = p.Name,
                Key = p.Key,
                Unit = p.Unit,
                Encoding = p.Encoding,
                Evidence = p.Evidence.Length > 0 ? p.Evidence : ParamEvidence,
                ReadOnly = p.ReadOnly,
                Note = p.Note,
            };
            if (item is null || (p.Kind == "multi_io" && item2 is null))
            {
                return d with { Found = false, Text = "not found on this unit" };
            }

            d = d with { Found = true, Address = Hex4(item.Address), RawHex = item.Hex };
            var choices = p.Choices ?? new Dictionary<int, string>();
            switch (p.Kind)
            {
                case "mm":
                    var mm = Math.Round((item.Signed * Registers.POSITION_SCALE) + p.OffsetMm, 2);
                    return d with { Value = mm, Text = mm.ToString("F2", CultureInfo.InvariantCulture) + " mm" };
                case "bit":
                    var bit = (int)((item.Raw >> (p.Bit ?? 0)) & 1);
                    return d with { Value = bit, Text = choices.TryGetValue(bit, out var bt) ? bt : bit.ToString(CultureInfo.InvariantCulture) };
                case "parity":
                    if (!HomePatternSettable(item.Raw))
                    {
                        return d with { Pattern = item.Raw, Value = null, Text = $"not settable (home pattern ORG = 0x{item.Raw:X8})" };
                    }

                    var parity = (int)(item.Raw & 1);
                    return d with { Pattern = item.Raw, Value = parity, Text = choices.TryGetValue(parity, out var pt) ? pt : parity.ToString(CultureInfo.InvariantCulture) };
                case "multi_io":
                    var pair = ((int)((item.Raw >> 8) & 0xFF), (int)((item2!.Raw >> 8) & 0xFF));
                    d = d with { RawHex = item.Hex + "/" + item2.Hex, Address = Hex4(item.Address) + "/" + Hex4(item2.Address) };
                    if (MULTI_IO_PAIRS.TryGetValue(pair, out var number))
                    {
                        return d with { Value = number, Text = choices.TryGetValue(number, out var mt) ? mt : string.Empty };
                    }

                    return d with { Value = null, Text = $"unknown pair ({pair.Item1}, {pair.Item2})" };
                default:
                    object value = item.Raw <= int.MaxValue ? (object)(int)item.Raw : (long)item.Raw;
                    var text = item.Raw <= int.MaxValue && choices.TryGetValue((int)item.Raw, out var et) ? et : $"{item.Raw}: unknown";
                    return d with { Value = value, Text = text };
            }
        }

        /// <summary>The named user parameters P1-P18 decoded from read classes.</summary>
        public static IReadOnlyList<DecodedParameter> DecodeAll(IReadOnlyDictionary<int, ClassBlock> classes)
        {
            var result = new List<DecodedParameter>();
            foreach (var p in USER_PARAMETERS)
            {
                classes.TryGetValue(p.ClassId, out var block);
                var item = block?.Item(p.ItemId, p.Entry);
                var item2 = block != null && p.ItemId2.HasValue ? block.Item(p.ItemId2.Value, p.Entry) : null;
                result.Add(Decode(p, item, item2));
            }

            return result;
        }

        /// <summary>
        /// Which parameters IA-OS's parameter screen shows on this unit (no I/O). Inputs: ACCL extensibility level, EXFN
        /// (8.50 b13), SUP2 CNWL (P7), SUP3 SORS (P16), STN1 (7.15; P8 - an approximation), the present P9 against the valid
        /// set and MXPT (3.16), FI00 / FO00 (8.25 / 8.29, P11). These rules reproduce IA-OS's screen on the gripper and the
        /// rod (VERIFIED there, UNVERIFIED on other units).
        /// </summary>
        public static IReadOnlyDictionary<int, ParameterVisibility> Visibility(
            IReadOnlyDictionary<int, ClassBlock> classes, IReadOnlyDictionary<string, uint>? support, uint? accl)
        {
            ClassItem? Item(int ci, int itemId) => classes.TryGetValue(ci, out var b) ? b.Item(itemId) : null;

            var level = ExtensibilityLevel(accl);
            var spio = Item(Registers.CATEGORY_MOTION_PARAMS, Registers.PARAM_ITEM_SPIO);
            bool? exfn = spio is null ? (bool?)null : (spio.Raw & Registers.SPIO_EXFN) != 0;
            uint? sup2 = support != null && support.TryGetValue("SUP2", out var s2) ? s2 : (uint?)null;
            uint sup3 = support != null && support.TryGetValue("SUP3", out var s3) ? s3 : 0;
            var stn1 = Item(Registers.CATEGORY_LIMIT_PARAMS, Registers.AXIS_ITEM_STN1);
            var p9 = Item(Registers.CATEGORY_MOTION_PARAMS, 2);
            var mxpt = Item(Registers.CATEGORY_CONTROLLER_MONITOR, Registers.CONTROLLER_ITEM_MXPT);
            var lvl = level.HasValue ? level.Value.ToString(CultureInfo.InvariantCulture) : "unknown";
            bool? ext = level.HasValue && exfn.HasValue ? level.Value > 1 && exfn.Value : (bool?)null;

            var outp = USER_PARAMETERS.ToDictionary(p => p.No, p => new ParameterVisibility(true, string.Empty));
            outp[2] = new ParameterVisibility(ext.HasValue ? !(level!.Value >= 2 && exfn!.Value) : (bool?)null, "hidden when level >= Phase1 and EXFN = 1");
            outp[7] = new ParameterVisibility(sup2.HasValue ? (sup2.Value & Registers.SUP2_CNWL) != 0 : (bool?)null, "shown only with a wireless module (SUP2 CNWL)");
            outp[8] = new ParameterVisibility(stn1 is null ? (bool?)null : stn1.Raw != 0xFFFFFFFF, "shown only with smart-tuning data (STN1 7.15 set; an approximation)");
            var valid = P9ValidValues(level, exfn, mxpt?.Raw);
            var validText = valid is null ? "None" : "(" + string.Join(", ", valid) + ")";
            outp[9] = new ParameterVisibility(valid is null || p9 is null ? (bool?)null : p9.Raw <= int.MaxValue && valid.Contains((int)p9.Raw), $"the present value must be in the valid set {validText} (level {lvl}, <= MXPT)");
            var fi = Item(Registers.CATEGORY_MOTION_PARAMS, 25);
            var fo = Item(Registers.CATEGORY_MOTION_PARAMS, 29);
            bool? knownPair = fi is null || fo is null ? (bool?)null : MULTI_IO_PAIRS.ContainsKey(((int)((fi.Raw >> 8) & 0xFF), (int)((fo.Raw >> 8) & 0xFF)));
            bool? p11 = ext.HasValue ? ext.Value && level != 3 && knownPair == true : (bool?)null;
            outp[11] = new ParameterVisibility(p11, $"needs level > Position3, not Phase2, EXFN = 1 and a known FI00/FO00 pair (level {lvl})");
            foreach (var no in new[] { 13, 14, 15 })
            {
                outp[no] = new ParameterVisibility(ext, $"needs level > Position3 and EXFN = 1 (level {lvl})");
            }

            outp[16] = new ParameterVisibility(support is null ? (bool?)null : (sup3 & Registers.SUP3_SORS) != 0, "shown only with SUP3 SORS");
            foreach (var no in new[] { 17, 18 })
            {
                outp[no] = new ParameterVisibility(ext.HasValue ? ext.Value && level > 2 : (bool?)null, $"needs level > Phase1 and EXFN = 1 (level {lvl})");
            }

            return outp;
        }

        /// <summary>
        /// Which user parameters IA-OS's screen lists for this unit (no I/O). A (system, unit) code with an IA-OS list
        /// (<see cref="Registers.PARAMETER_CANDIDATES"/>) is <c>Known</c>: its candidates filtered by
        /// <paramref name="visibility"/>, whatever the model string. Any other code (or an unreadable unit code): every
        /// mapped parameter. Every unit's parameters are written the same way.
        /// </summary>
        public static ParameterModel ModelParameterList(int? unitCode, string? modelName, int? systemCode = null, IReadOnlyDictionary<int, ParameterVisibility>? visibility = null)
        {
            var system = systemCode ?? 0xEC;
            IReadOnlyList<int>? candidates = null;
            if (unitCode.HasValue && Registers.PARAMETER_CANDIDATES.TryGetValue((system, unitCode.Value), out var c))
            {
                candidates = c;
            }

            var family = Registers.EcFamily(system, unitCode);
            var model = new ParameterModel { UnitCode = unitCode, SystemCode = systemCode, ModelName = modelName, Family = family };
            if (candidates != null)
            {
                var hidden = candidates.Where(no => visibility != null && visibility.TryGetValue(no, out var v) && v.Shown == false)
                    .Select(no => (no, visibility![no].Reason)).ToList();
                var hiddenNos = new HashSet<int>(hidden.Select(h => h.no));
                var listed = candidates.Where(no => !hiddenNos.Contains(no)).ToList();
                var note = $"the parameters IA-OS lists for unit 0x{unitCode!.Value:X2} ({family ?? "?"})";
                return model with { Known = true, Candidates = candidates.ToList(), Listed = listed, Hidden = hidden, Verified = true, Note = note };
            }

            var noteUnknown = unitCode is null
                ? "unit code not readable (FC11 / CTCL): every mapped item found on the unit is shown"
                : $"unit code 0x{unitCode.Value:X2}: every mapped item found on the unit is shown";
            return model with { Known = false, Listed = BY_NO.Keys.OrderBy(n => n).ToList(), Verified = false, Note = noteUnknown };
        }

        /// <summary>The decoded named parameters with the per-model flags added (no I/O).</summary>
        public static IReadOnlyList<DecodedParameter> Annotate(
            IEnumerable<DecodedParameter> named, int? unitCode, string? modelName, int? systemCode = null, IReadOnlyDictionary<int, ParameterVisibility>? visibility = null)
        {
            var model = ModelParameterList(unitCode, modelName, systemCode, visibility);
            var result = new List<DecodedParameter>();
            foreach (var q in named)
            {
                BY_NO.TryGetValue(q.No, out var p);
                var listed = model.Listed.Contains(q.No) && (model.Known || q.Found);
                var settable = !(p != null && p.Kind == "parity" && q.Found && q.Value is null);
                var readOnly = p != null && p.ReadOnly;
                ParameterVisibility? vis = null;
                visibility?.TryGetValue(q.No, out vis);
                var hiddenReason = vis != null && vis.Shown == false ? vis.Reason : string.Empty;
                var modelNote = string.Empty;
                if (model.Known && !listed)
                {
                    modelNote = $"not listed by IA-OS for unit 0x{model.UnitCode!.Value:X2}" + (hiddenReason.Length > 0 ? $" ({hiddenReason})" : string.Empty);
                }
                else if (readOnly)
                {
                    modelNote = p != null && p.Note.Length > 0 ? p.Note : "read-only in this driver";
                }
                else if (!settable)
                {
                    modelNote = "not settable: home pattern ORG (item 9.16) is 0 / FFFFFFFF";
                }

                result.Add(q with
                {
                    Listed = listed,
                    VerifiedForModel = model.Verified,
                    HomeParameter = HOME_PARAMETERS.Contains(q.No),
                    Writable = q.Found && p != null && listed && !readOnly && settable,
                    Choices = p is null ? new Dictionary<string, string>() : ChoicesOf(p).ToDictionary(kv => kv.Key.ToString(CultureInfo.InvariantCulture), kv => kv.Value),
                    ReadOnly = readOnly,
                    ShownByIaos = vis?.Shown,
                    HiddenReason = hiddenReason,
                    ModelNote = modelNote,
                });
            }

            return result;
        }

        /// <summary>
        /// A value for parameter <paramref name="p"/> as IA-OS shows it, as a number. mm parameters: a finite number (or
        /// numeric string), rounded to 0.01 mm. Choice parameters: the number or the label, case-insensitive ("1:Forward",
        /// "forward", "Disable"; a unique prefix such as "instant" is accepted). Booleans are refused (P7 uses 1 = Disable).
        /// Throws <see cref="ArgumentException"/>.
        /// </summary>
        public static double Parse(ParamDef p, object? value)
        {
            if (value is bool)
            {
                throw new ArgumentException($"P{p.No}: pass the number or label IA-OS shows, not {value} (P7 uses 1 = Disable)");
            }

            if (p.Kind == "mm")
            {
                double number;
                if (value is string s)
                {
                    if (!double.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out number))
                    {
                        throw new ArgumentException($"P{p.No} needs a number in mm, got '{s}'");
                    }
                }
                else if (!TryNumber(value, out number))
                {
                    throw new ArgumentException($"P{p.No} needs a finite number in mm, got {Show(value)}");
                }

                if (double.IsNaN(number) || double.IsInfinity(number))
                {
                    throw new ArgumentException($"P{p.No} needs a finite number in mm, got {Show(value)}");
                }

                return Math.Round(number, 2);
            }

            var choices = ChoicesOf(p);
            var allowed = string.Join(", ", choices.Values);
            int chosen;
            if (value is string text)
            {
                var t = text.Trim().ToLowerInvariant();
                if (int.TryParse(t, NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out var n))
                {
                    chosen = n;
                }
                else
                {
                    var names = choices.ToDictionary(kv => kv.Key, kv => kv.Value.Split(new[] { ':' }, 2).Last().Trim().ToLowerInvariant());
                    var exact = choices.Where(kv => t == kv.Value.ToLowerInvariant() || t == names[kv.Key]).Select(kv => kv.Key).ToList();
                    var prefix = names.Where(kv => t.Length > 0 && kv.Value.StartsWith(t, StringComparison.Ordinal)).Select(kv => kv.Key).ToList();
                    var found = exact.Count > 0 ? exact : prefix;
                    if (found.Count != 1)
                    {
                        throw new ArgumentException($"P{p.No} ({p.Name}): '{text}' is not one of {allowed}");
                    }

                    chosen = found[0];
                }
            }
            else if (TryNumber(value, out var d) && d == Math.Floor(d) && Math.Abs(d) < int.MaxValue)
            {
                chosen = (int)d;
            }
            else
            {
                throw new ArgumentException($"P{p.No} ({p.Name}): {Show(value)} is not one of {allowed}");
            }

            if (!choices.ContainsKey(chosen))
            {
                throw new ArgumentException($"P{p.No} ({p.Name}): {Show(value)} is not one of {allowed}");
            }

            return chosen;
        }

        /// <summary>
        /// The new 32-bit item for P<c>p.No</c> = <paramref name="number"/> (from <see cref="Parse"/>), every other bit of
        /// <paramref name="raw"/> kept (read-modify-write: P5 / P6 share item 8.14, P7 / P10 8.50, P12 is one bit of 8.1).
        /// mm items s32 0.01 mm (P1 = upper soft limit - 0.30 mm); bits 1 / 0 (P7 inverted in meaning); P3 the PARITY of
        /// the ORG home pattern (an even pattern becomes pattern - 1 for 1:Forward, an odd one pattern + 1 for 0:Reverse;
        /// not settable when ORG is 0 / FFFFFFFF); P9 the whole item. Read-only kinds throw <see cref="ArgumentException"/>.
        /// </summary>
        public static uint Encode(ParamDef p, uint raw, double number)
        {
            if (p.ReadOnly || p.Kind == "multi_io")
            {
                throw new ArgumentException($"P{p.No} ({p.Name}) is read-only in this driver");
            }

            if (p.Kind == "mm")
            {
                var counts = Registers.ToCounts(number - p.OffsetMm);
                if (counts < int.MinValue || counts > int.MaxValue)
                {
                    throw new ArgumentException($"P{p.No}: {number} mm does not fit a 32-bit item");
                }

                return unchecked((uint)(int)counts);
            }

            var choices = ChoicesOf(p);
            var n = (int)number;
            if (n != number || !choices.ContainsKey(n))
            {
                throw new ArgumentException($"P{p.No}: {number} is not one of its numbers ({string.Join(", ", choices.Values)})");
            }

            switch (p.Kind)
            {
                case "bit":
                    var mask = 1u << (p.Bit ?? 0);
                    return (raw & ~mask) | (n != 0 ? mask : 0u);
                case "parity":
                    if (!HomePatternSettable(raw))
                    {
                        throw new ArgumentException($"P{p.No}: the home pattern ORG is 0x{raw:X8}: the home direction is not settable");
                    }

                    if ((raw & 1) == n)
                    {
                        return raw; // already the requested direction
                    }

                    return n == 1 ? raw - 1 : raw + 1;
                default:
                    return unchecked((uint)n); // enum: the whole item
            }
        }

        internal static bool TryNumber(object? value, out double number)
        {
            switch (value)
            {
                case int i:
                    number = i;
                    return true;
                case long l:
                    number = l;
                    return true;
                case uint u:
                    number = u;
                    return true;
                case short sh:
                    number = sh;
                    return true;
                case byte b:
                    number = b;
                    return true;
                case double d:
                    number = d;
                    return true;
                case float f:
                    number = f;
                    return true;
                case decimal m:
                    number = (double)m;
                    return true;
                default:
                    number = double.NaN;
                    return false;
            }
        }

        internal static string Show(object? value) => value switch
        {
            null => "null",
            string s => "'" + s + "'",
            IFormattable f => f.ToString(null, CultureInfo.InvariantCulture),
            _ => value.ToString() ?? string.Empty,
        };

        private static string Hex4(int address) => "0x" + address.ToString("X4", CultureInfo.InvariantCulture);
    }
}
