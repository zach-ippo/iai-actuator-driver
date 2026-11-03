using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The core's messages name the API in the Python edition's identifiers (<c>pass allow_nonvolatile</c>,
    /// <c>call enable_modbus_control()</c>, <c>(max_speed_mm_s)</c>); a C# caller reads this library's names instead
    /// (<c>pass allowNonvolatile: true</c>, <c>call EnableModbusControl()</c>, <c>(MaxSpeedMmS)</c>). Applied to the error
    /// messages and notes of the error map and to the log lines passed on to <see cref="DriverLog"/>. Pure: no library call.
    /// </summary>
    internal static class NameMap
    {
        // Longer forms first: "pass allow_dangerous=True" before "pass allow_dangerous" before "allow_dangerous".
        private static readonly (string Core, string Managed)[] Pairs =
        {
            ("pass allow_nonvolatile=True", "pass allowNonvolatile: true"),
            ("pass allow_nonvolatile", "pass allowNonvolatile: true"),
            ("pass allow_unsafe=True", "pass allowUnsafe: true"),
            ("pass allow_unsafe", "pass allowUnsafe: true"),
            ("pass allow_dangerous=True", "pass allowDangerous: true"),
            ("pass allow_dangerous", "pass allowDangerous: true"),
            ("pass allow_unlisted=True", "pass allowUnlisted: true"),
            ("pass allow_unlisted", "pass allowUnlisted: true"),
            ("pass allow_never_send", "pass allowNeverSend: true"),
            ("Pass allow_above_caps", "Pass allowAboveCaps: true"),
            ("allow_home_params=True", "allowHomeParams: true"),
            ("allow_unverified_code=True", "allowUnverifiedCode: true"),
            ("write_checksum=True", "writeChecksum: true"),
            ("force=True", "force: true"),
            ("allow_nonvolatile", "allowNonvolatile"),
            ("allow_unsafe", "allowUnsafe"),
            ("allow_dangerous", "allowDangerous"),
            ("allow_unlisted", "allowUnlisted"),
            ("allow_home_params", "allowHomeParams"),
            ("allow_above_caps", "allowAboveCaps"),
            ("allow_never_send", "allowNeverSend"),
            ("allow_unverified_code", "allowUnverifiedCode"),
            ("write_checksum", "writeChecksum"),
            ("max_speed_mm_s", "MaxSpeedMmS"),
            ("max_accel_g", "MaxAccelG"),
            ("max_push_pct", "MaxPushPct"),
            ("enable_modbus_control()", "EnableModbusControl()"),
            ("set_safety_speed()", "SetSafetySpeed()"),
            ("software_reset()", "SoftwareReset()"),
            ("reset_alarm()", "ResetAlarm()"),
            ("identify()", "Identify()"),
            ("move_to()", "MoveTo()"),
            ("use move_to (", "use MoveTo ("),
            ("move_to_position(", "MoveToPosition("),
            ("read_position_table(", "ReadPositionTable("),
            ("read_parameter(", "ReadParameter("),
            ("resume()", "Resume()"),
            ("pause()", "Pause()"),
            ("home()", "Home()"),
            ("stop()", "Stop()"),
        };

        // One pass over the text, each identifier only where it is not part of a longer one.
        private static readonly Regex Pattern = new Regex(
            string.Join("|", Pairs.Select(p => "(?<![A-Za-z0-9_])" + Regex.Escape(p.Core) + (char.IsLetterOrDigit(p.Core[p.Core.Length - 1]) ? "(?![A-Za-z0-9_])" : string.Empty))),
            RegexOptions.CultureInvariant);

        private static readonly Dictionary<string, string> ByCore = Pairs.ToDictionary(p => p.Core, p => p.Managed);

        /// <summary><paramref name="text"/> with the core's API identifiers in this library's spelling.</summary>
        internal static string Managed(string text) =>
            string.IsNullOrEmpty(text) || text.IndexOfAny(new[] { '_', '(' }) < 0 ? text : Pattern.Replace(text, m => ByCore[m.Value]);
    }
}
