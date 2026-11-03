using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver
{
    /// <summary>Level of a controller alarm (<see cref="Alarms.AlarmLevelOf"/>).</summary>
    public enum AlarmLevel
    {
        /// <summary>0x000 or the 0x0FFF power-up marker.</summary>
        None,

        /// <summary>A warning: the axis keeps running.</summary>
        Message,

        /// <summary>Stop + servo off; an alarm reset clears it.</summary>
        OperationCancelled,

        /// <summary>Stop + servo off; only a software reset or a power cycle clears it.</summary>
        ColdStart,

        /// <summary>No known level: a fault unless the warning bit ALML is set (fail safe).</summary>
        Unknown,
    }

    /// <summary>
    /// Controller alarm names, levels and groups (ALMC 0x9002, alarm history 0x0500).
    /// </summary>
    /// <remarks>
    /// <para>Names (<see cref="AlarmName"/>): the driver's curated names (<see cref="Registers.ALARM_NAMES"/>), else
    /// <see cref="CONTROLLER_ALARM_NAMES"/> (IAI's English controller alarm names, as IA-OS shows them). Which of these
    /// codes an EC can actually raise is not documented.</para>
    /// <para>Levels, UNVERIFIED unless marked:</para>
    /// <list type="bullet">
    /// <item><description><b>message</b> = a warning: the axis keeps running; it never blocks motion and never lights the
    /// red alarm lamp. Only the codes of <see cref="MESSAGE_CODES"/> (the named codes of 0x011-0x07F, e.g. maintenance
    /// warnings 0x048 / 0x04A / 0x04E / 0x04F, deviation warning 0x047, collision warning 0x049). No message-level code
    /// has been seen on an EC yet.</description></item>
    /// <item><description><b>operation cancelled</b>: the unit decelerates, stops and switches the servo off (DSS1 b10
    /// ALMH); an alarm reset clears it (<see cref="OPERATION_CANCELLED_CODES"/>). VERIFIED for 0x090 (software reset sent
    /// with the servo on: ALMH, servo off; an alarm reset cleared it).</description></item>
    /// <item><description><b>cold start</b>: every other code of 0x080-0x0FF (and 0x001-0x010). The same stop, but only
    /// a software reset (servo off first) or a 24 V power cycle clears it, and an incremental unit then needs a home
    /// return.</description></item>
    /// <item><description>Device-dependent: <b>0x0EE</b> "ABS error (2)" is operation cancelled on an absolute unit
    /// (VERIFIED on the WA battery-less absolute gripper: ALMH, servo off; an alarm reset and then a home return cleared
    /// it) and cold start on an incremental unit (UNVERIFIED). <see cref="AbsoluteEncoderFromSupport"/> tells the two
    /// apart; unknown, the texts name both. <b>0x0CE / 0x0D2</b> (supply voltage) count as cold start on every unit
    /// (conservative).</description></item>
    /// <item><description><b>unknown</b>: every other code, including a code of 0x011-0x07F that is not in
    /// <see cref="MESSAGE_CODES"/>. It counts as a fault unless the warning bit DSS1 b9 ALML is set (fail
    /// safe).</description></item>
    /// </list>
    /// <para>Groups (<see cref="AlarmGroup"/>): the EC alarm groups A-F, "Warning" (maintenance) and "Other"
    /// (UNVERIFIED; IA-OS's alarm window showed 0x0EE as group D). The remedies are the driver's own short wording.</para>
    /// </remarks>
    public static partial class Alarms
    {
        /// <summary>Level texts.</summary>
        public static readonly IReadOnlyDictionary<AlarmLevel, string> LEVEL_TEXT = new Dictionary<AlarmLevel, string>
        {
            [AlarmLevel.None] = "no alarm",
            [AlarmLevel.Message] = "message (warning: the axis keeps running)",
            [AlarmLevel.OperationCancelled] = "operation cancelled (stop, servo off; alarm reset)",
            [AlarmLevel.ColdStart] = "cold start (stop, servo off; software reset or power cycle)",
            [AlarmLevel.Unknown] = "unknown level (treated as a fault unless the EC sets the warning bit ALML)",
        };

        /// <summary>The message-level codes (the named codes of 0x011-0x07F).</summary>
        public static readonly IReadOnlyCollection<int> MESSAGE_CODES = Codes(
            Range(0x040, 0x041), Range(0x047, 0x04A), Range(0x04C, 0x04F), Range(0x05A, 0x05E), Range(0x061, 0x065),
            Range(0x068, 0x06B), Range(0x070, 0x07C), new[] { 0x07F });

        /// <summary>
        /// The 0x080-0x0FF codes that only cancel the operation (0x0EE is handled apart; 0x0CE / 0x0D2 are cold start; 0x0F1
        /// fieldbus link error: operation cancelled on every CON family, DOC ME0342-4H p.8-21).
        /// </summary>
        public static readonly IReadOnlyCollection<int> OPERATION_CANCELLED_CODES = Codes(
            new[] { 0x080 }, Range(0x082, 0x08A), Range(0x090, 0x096), Range(0x0A2, 0x0A5), new[] { 0x0A7 },
            Range(0x0AB, 0x0AE), Range(0x0B5, 0x0B6), new[] { 0x0BA }, Range(0x0BE, 0x0C2), new[] { 0x0C5, 0x0CF, 0x0D6 },
            Range(0x0D8, 0x0DC), Range(0x0DE, 0x0DF), new[] { 0x0E1, 0x0ED, 0x0EF, 0x0F1 });

        // Class-1 support bits that mark an absolute encoder. Both are set on the two WA (battery-less absolute) units read
        // on hardware (SUPT 0x60002BFF / 0x60002BD7, SUP2 0x838C009A / 0x4384009A); no incremental unit's words were read.
        private const uint SuptAbse = 1u << 13; // SUPT b13 ABSE: absolute encoder
        private const uint Sup2Mabs = 1u << 18; // SUP2 b18 MABS: battery-less absolute

        private const string LevelText0EEUnknown = "operation cancelled on absolute units / cold start on incremental units";

        private static readonly HashSet<int> MessageSet = new HashSet<int>(MESSAGE_CODES);
        private static readonly HashSet<int> OperationCancelledSet = new HashSet<int>(OPERATION_CANCELLED_CODES);

        /// <summary>The message codes of the CON-family units (ME0342-4H p.8-14), named on every unit as the core and the Python edition do.</summary>
        private static readonly IReadOnlyDictionary<int, string> ConMessageNames = new Dictionary<int, string>
        {
            [0x02C] = "Monitoring data type change command during monitoring",
            [0x02D] = "Monitoring related command in monitoring function invalid status",
            [0x02E] = "RTC related command in RTC invalid status",
        };

        /// <summary>
        /// Name of controller alarm <paramref name="code"/>: the driver's curated name, else IAI's
        /// (<see cref="CONTROLLER_ALARM_NAMES"/>, the CON message codes 0x02C-0x02E), else "alarm 0x...".
        /// </summary>
        public static string AlarmName(int code)
        {
            if (Registers.ALARM_NAMES.TryGetValue(code, out var curated))
            {
                return curated;
            }

            if (CONTROLLER_ALARM_NAMES.TryGetValue(code, out var name) || ConMessageNames.TryGetValue(code, out name))
            {
                return name;
            }

            return "alarm 0x" + code.ToString("x3", CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// Whether the unit has an absolute encoder, from its class-1 support words (SUPT / SUP2 by name, see
        /// <see cref="UnitDirectory.ReadSupportWords"/>). No I/O. True when SUPT b13 ABSE or SUP2 b18 MABS is set (both
        /// are set on the WA units read on hardware); false when SUPT was read and neither bit is set (UNVERIFIED: no
        /// incremental unit's support words were read); <c>null</c> when it cannot be told.
        /// </summary>
        public static bool? AbsoluteEncoderFromSupport(IReadOnlyDictionary<string, uint>? support)
        {
            if (support is null || support.Count == 0)
            {
                return null;
            }

            var hasSupt = support.TryGetValue("SUPT", out var supt);
            var hasSup2 = support.TryGetValue("SUP2", out var sup2);
            if ((hasSupt && (supt & SuptAbse) != 0) || (hasSup2 && (sup2 & Sup2Mabs) != 0))
            {
                return true;
            }

            return hasSupt ? false : (bool?)null;
        }

        /// <summary>
        /// Level of controller alarm <paramref name="code"/> (rules in the class remarks; UNVERIFIED unless marked there).
        /// <paramref name="absoluteEncoder"/> only matters for 0x0EE: true (absolute unit) or <c>null</c> (unknown) ->
        /// operation cancelled, as on the absolute units it was seen on; false (incremental unit) -> cold start.
        /// </summary>
        public static AlarmLevel AlarmLevelOf(int code, bool? absoluteEncoder = null)
        {
            if (code == 0 || code == Registers.ALARM_POWER_UP)
            {
                return AlarmLevel.None;
            }

            if (code >= 0x001 && code <= 0x010)
            {
                return AlarmLevel.ColdStart; // Modbus exception / reject range (not raised as ALMC on the EC)
            }

            if (code >= 0x011 && code <= 0x07F)
            {
                return MessageSet.Contains(code) ? AlarmLevel.Message : AlarmLevel.Unknown;
            }

            if (code >= 0x080 && code <= 0x0FF)
            {
                if (code == 0x0EE)
                {
                    return absoluteEncoder == false ? AlarmLevel.ColdStart : AlarmLevel.OperationCancelled;
                }

                return OperationCancelledSet.Contains(code) ? AlarmLevel.OperationCancelled : AlarmLevel.ColdStart;
            }

            return AlarmLevel.Unknown;
        }

        /// <summary>The level as the lower-case key the Python edition uses ("none", "message", "operation_cancelled", "cold_start", "unknown").</summary>
        public static string LevelKey(AlarmLevel level) => level switch
        {
            AlarmLevel.None => "none",
            AlarmLevel.Message => "message",
            AlarmLevel.OperationCancelled => "operation_cancelled",
            AlarmLevel.ColdStart => "cold_start",
            _ => "unknown",
        };

        /// <summary>
        /// The driver's fault rule: DSS1 b10 ALMH (<paramref name="alarmBit"/>), or a code of level operation cancelled /
        /// cold start. A message-level code is never a fault by itself. A code of unknown level is a fault unless the
        /// warning bit ALML (<paramref name="warningBit"/>) is set (fail safe).
        /// </summary>
        public static bool AlarmIsFault(int code, bool alarmBit = false, bool warningBit = false)
        {
            if (alarmBit)
            {
                return true;
            }

            if (code == 0)
            {
                return false;
            }

            var level = AlarmLevelOf(code);
            if (level == AlarmLevel.Message || level == AlarmLevel.None)
            {
                return false;
            }

            if (level == AlarmLevel.OperationCancelled || level == AlarmLevel.ColdStart)
            {
                return true;
            }

            return !warningBit;
        }

        /// <summary>EC alarm group letter (A-F), "Warning" (maintenance) or "Other" (UNVERIFIED).</summary>
        public static string AlarmGroup(int code) => ALARM_GROUPS.TryGetValue(code, out var group) ? group : "Other";

        /// <summary>
        /// Decode alarm-history words (0x0500: 6 words per slot - detail, address, 0, code, time high, time low; VERIFIED
        /// layout). No I/O. Index 0 is the newest; the time (ms since power-on) is <c>null</c> when the unit stores
        /// FFFFFFFF. The list ends at the first empty slot (code 0) unless <paramref name="includeEmpty"/>. Power-up
        /// markers (0x0FFF) are kept.
        /// </summary>
        public static IReadOnlyList<AlarmRecord> AlarmHistorySlots(IReadOnlyList<ushort> words, bool includeEmpty = false)
        {
            var stride = Registers.ALARM_HISTORY_STRIDE;
            var slots = new List<AlarmRecord>();
            for (var k = 0; k < words.Count / stride; k++)
            {
                var o = k * stride;
                int code = words[o + 3];
                if (code == 0 && !includeEmpty)
                {
                    break;
                }

                var stamp = Registers.FromU32Words(words[o + 4], words[o + 5]);
                slots.Add(new AlarmRecord(k, code, words[o], words[o + 1], stamp == 0xFFFFFFFF ? (long?)null : stamp));
            }

            return slots.AsReadOnly();
        }

        /// <summary>
        /// Decode an alarm's detail / address words where their meaning is known ("" otherwise; UNVERIFIED). 0x0A2: detail
        /// = position row, address = 0x1000 + 16 * row + field. 0x0A3: the address names the direct-move field. 0x0A1: the
        /// address names the parameter. Detail 0000 / address FFFF mean "none".
        /// </summary>
        public static string DescribeAlarmDetail(int code, int? detail, int? address)
        {
            if (code == 0x0A2 && address.HasValue && address.Value >= 0x1000 && address.Value < 0x4000)
            {
                var row = (address.Value - 0x1000) / 16;
                var field = (address.Value - 0x1000) % 16;
                var fieldName = RowFields.TryGetValue(field, out var f) ? f : "+0x" + field.ToString("X", CultureInfo.InvariantCulture);
                return string.Format(CultureInfo.InvariantCulture, "row {0}, field {1}", detail ?? row, fieldName);
            }

            if (code == 0x0A3 && address.HasValue && CommandFields.TryGetValue(address.Value, out var commandField))
            {
                return string.Format(CultureInfo.InvariantCulture, "direct-move field {0} (0x{1:X4})", commandField, address.Value);
            }

            if (code == 0x0A1 && address.HasValue && address.Value != 0xFFFF)
            {
                return string.Format(CultureInfo.InvariantCulture, "parameter at 0x{0:X4}", address.Value);
            }

            return string.Empty;
        }

        /// <summary>
        /// Everything the driver knows about <paramref name="code"/> (no I/O). <paramref name="absoluteEncoder"/>
        /// (<see cref="AbsoluteEncoderFromSupport"/>) picks the 0x0EE wording; <c>null</c> names both cases.
        /// </summary>
        public static AlarmInfo AlarmInfoOf(int code, bool? absoluteEncoder = null, int? detail = null, int? address = null)
        {
            var level = AlarmLevelOf(code, absoluteEncoder);
            var group = AlarmGroup(code);
            var levelText = LEVEL_TEXT[level];
            string reset;
            string remedy;
            if (level == AlarmLevel.None)
            {
                reset = string.Empty;
                remedy = string.Empty;
            }
            else if (level == AlarmLevel.Message)
            {
                reset = "alarm reset (the axis keeps running meanwhile)";
                remedy = "warning only: the axis keeps running; alarm reset once the cause is dealt with";
            }
            else if (code == 0x0EE && absoluteEncoder is null)
            {
                levelText = LevelText0EEUnknown;
                reset = "absolute units: alarm reset, then servo on; incremental units: software reset (servo off first) or 24 V power cycle";
                remedy = "the EC stopped and switched the servo off: alarm reset; if it stays, software reset or power cycle";
            }
            else if (level == AlarmLevel.OperationCancelled)
            {
                reset = "alarm reset, then servo on";
                remedy = "the EC stopped and switched the servo off: remove the cause, alarm reset, servo on";
            }
            else if (level == AlarmLevel.ColdStart)
            {
                reset = "software reset (servo off first) or 24 V power cycle; an incremental unit then needs a home return";
                remedy = "an alarm reset will probably not clear it: remove the cause, then software reset or power cycle";
            }
            else
            {
                reset = "alarm reset; if it stays, software reset (servo off first) or power cycle";
                remedy = "unknown alarm level: treated as a fault unless the EC sets the warning bit ALML";
            }

            var advice = SpecificRemedy.TryGetValue(code, out var specific) ? specific : GroupAdvice.TryGetValue(group, out var g) ? g : string.Empty;
            if (advice.Length > 0 && level != AlarmLevel.None)
            {
                remedy = $"{remedy}. {char.ToUpperInvariant(advice[0])}{advice.Substring(1)}";
            }

            string home;
            if (code == 0x0EE)
            {
                home = absoluteEncoder switch
                {
                    true => "home lost: alarm reset, then a home return",
                    false => "home lost: software reset (servo off first) or power cycle, then a home return",
                    null => "home lost: a home return after the reset (absolute units: alarm reset; incremental units: software reset or power cycle)",
                };
            }
            else
            {
                home = HOME_NEEDED.TryGetValue(code, out var h) ? h : string.Empty;
            }

            if (home.Length == 0 && level == AlarmLevel.ColdStart)
            {
                home = "incremental units: home return after the reset";
            }

            return new AlarmInfo
            {
                Code = code,
                Hex = "0x" + code.ToString("X3", CultureInfo.InvariantCulture),
                Name = AlarmName(code),
                Level = level,
                LevelText = levelText,
                Group = group,
                GroupName = GROUP_NAMES.TryGetValue(group, out var groupName) ? groupName : group,
                Fault = level == AlarmLevel.OperationCancelled || level == AlarmLevel.ColdStart || level == AlarmLevel.Unknown,
                Warning = level == AlarmLevel.Message,
                Reset = reset,
                HomeNeeded = home,
                Remedy = remedy,
                DetailText = DescribeAlarmDetail(code, detail, address),
            };
        }

        private static int[] Range(int first, int last) => Enumerable.Range(first, last - first + 1).ToArray();

        private static IReadOnlyCollection<int> Codes(params int[][] parts) =>
            Array.AsReadOnly(parts.SelectMany(p => p).OrderBy(c => c).ToArray());
    }

    /// <summary>Everything the driver knows about one alarm code (<see cref="Alarms.AlarmInfoOf"/>).</summary>
    public sealed record AlarmInfo
    {
        public int Code { get; init; }

        /// <summary>e.g. "0x0EE".</summary>
        public string Hex { get; init; } = string.Empty;

        public string Name { get; init; } = string.Empty;

        public AlarmLevel Level { get; init; }

        public string LevelText { get; init; } = string.Empty;

        /// <summary>A-F, "Warning" or "Other".</summary>
        public string Group { get; init; } = string.Empty;

        public string GroupName { get; init; } = string.Empty;

        /// <summary>Blocks motion and lights the red lamp: level operation cancelled / cold start / unknown.</summary>
        public bool Fault { get; init; }

        /// <summary>Message level: the axis keeps running.</summary>
        public bool Warning { get; init; }

        /// <summary>How to clear it.</summary>
        public string Reset { get; init; } = string.Empty;

        /// <summary>Home-return advice ("" if none).</summary>
        public string HomeNeeded { get; init; } = string.Empty;

        /// <summary>The driver's own short advice.</summary>
        public string Remedy { get; init; } = string.Empty;

        /// <summary><see cref="Alarms.DescribeAlarmDetail"/>.</summary>
        public string DetailText { get; init; } = string.Empty;
    }

    /// <summary>
    /// One alarm-history entry (0x0500 + 6 * index). Index 0 is the newest (consistent with the time stamps read on
    /// hardware). <see cref="TimeMs"/> is the time stamp in milliseconds since the unit was powered on (it has no clock),
    /// <c>null</c> when it stores FFFFFFFF. Detail 0000 / address FFFF mean "none".
    /// </summary>
    public sealed record AlarmRecord(int Index, int Code, int Detail, int Address, long? TimeMs)
    {
        /// <summary>Alarm name (<see cref="Alarms.AlarmName"/>).</summary>
        public string Name => Alarms.AlarmName(Code);

        /// <summary>The 0x0FFF "PowerUP No Error" marker the unit writes at every power-up.</summary>
        public bool PowerUp => Code == Registers.ALARM_POWER_UP;

        /// <summary>The level of the code.</summary>
        public AlarmLevel Level => Alarms.AlarmLevelOf(Code);

        /// <summary>The time stamp as "[d d ]hh:mm:ss.mmm" after power-on (<c>null</c> without one).</summary>
        public string? TimeAfterPowerOn
        {
            get
            {
                if (!TimeMs.HasValue)
                {
                    return null;
                }

                var ms = TimeMs.Value % 1000;
                var seconds = TimeMs.Value / 1000;
                var days = seconds / 86400;
                var rest = seconds % 86400;
                var text = string.Format(CultureInfo.InvariantCulture, "{0:00}:{1:00}:{2:00}.{3:000}", rest / 3600, (rest % 3600) / 60, rest % 60, ms);
                return days > 0 ? string.Format(CultureInfo.InvariantCulture, "{0} d {1}", days, text) : text;
            }
        }
    }
}
