using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace IaiActuatorDriver.Cli
{
    /// <summary>The tool's text output (<c>--json</c> carries every field instead).</summary>
    internal static class CliOutput
    {
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        public static string Hex(IEnumerable<byte> bytes) => string.Join(" ", bytes.Select(b => b.ToString("x2", Inv)));

        public static void PrintPorts(IReadOnlyList<SerialPortInfo> entries, TextWriter output)
        {
            if (entries.Count == 0)
            {
                output.WriteLine("no serial port found");
                return;
            }

            output.WriteLine(I($"{"PORT",-10} {"USB ID",-10} {"IAI",-4} DESCRIPTION"));
            foreach (var e in entries)
            {
                var usb = e.Vid.HasValue && e.Pid.HasValue ? I($"{e.Vid.Value:X4}:{e.Pid.Value:X4}") : "-";
                output.WriteLine(I($"{e.Port,-10} {usb,-10} {(e.IsIai ? "yes" : "no"),-4} {e.Description}"));
            }
        }

        public static void PrintScan(string name, int? baud, IReadOnlyList<ScanHit> hits, string? error, TextWriter output)
        {
            if (error != null)
            {
                output.WriteLine($"{name}: error: {error}");
                return;
            }

            if (hits.Count == 0)
            {
                output.WriteLine($"{name}: no axis answered");
                return;
            }

            var rate = baud.HasValue ? I($"{baud.Value} baud") : "no baud rate (simulated)";
            output.WriteLine(I($"{name}: {hits.Count} axis/axes at {rate}"));
            foreach (var h in hits)
            {
                string what;
                if (h.Error != null)
                {
                    what = h.Error;
                }
                else
                {
                    var family = h.Family ?? h.ProductFamily ?? "-";
                    what = I($"{h.SystemName}, unit 0x{h.UnitCode ?? 0:X2}, firmware {h.Firmware}: {family}");
                }

                output.WriteLine(I($"  {h.Name,-12} axis {h.AxisNumber,-3} {what}"));
            }
        }

        public static void PrintStatus(Status st, TextWriter output, string units = "mm")
        {
            var flags = new (string Name, bool On)[]
            {
                ("SERVO", st.ServoOn), ("HOMED", st.Homed), ("IN-POS", st.InPosition), ("MOVING", st.Moving), ("PAUSED", st.Paused),
                ("HOMING", st.Homing), ("PUSHING", st.Pushing), ("PUSH-MISSED", st.PushMissed), ("MODBUS", st.ModbusEnabled),
                ("SAFETY-SPEED", st.SafetySpeed), ("ALARM", st.Alarm), ("WARNING", st.Warning), ("EMG", st.EmergencyStop),
            }.Where(f => f.On).Select(f => f.Name).ToList();
            var markers = new (string Name, bool? On)[]
            {
                ("BACKWARD-END", st.AtBackwardEnd), ("MIDDLE", st.AtMiddle), ("FORWARD-END", st.AtForwardEnd),
            }.Where(m => m.On == true).Select(m => m.Name).ToList();
            output.WriteLine(I($"position   {st.PositionMm,9:F2} {units}"));
            if (st.VelocityMmS.HasValue)
            {
                output.WriteLine(I($"velocity   {st.VelocityMmS.Value,9:F2} {units}/s"));
            }

            if (st.CurrentMa.HasValue)
            {
                output.WriteLine(I($"current    {st.CurrentMa.Value,9}"));
            }

            output.WriteLine("alarm code " + ("0x" + st.AlarmCode.ToString("x", Inv)).PadLeft(9));
            output.WriteLine("flags      " + (flags.Count > 0 ? string.Join(" ", flags) : "-"));
            var markerText = st.AtBackwardEnd is null ? "not read (ZONS)" : (markers.Count > 0 ? string.Join(" ", markers) : "-");
            output.WriteLine("markers    " + markerText);
            output.WriteLine("raw        " + string.Join(" ", st.Raw.Select(kv => kv.Key + "=0x" + kv.Value.ToString("x4", Inv))));
        }

        public static void PrintInfo(DeviceInfo info, Axis axis, TextWriter output)
        {
            output.WriteLine($"model      {info.Model}");
            output.WriteLine($"controller {info.Controller}");
            output.WriteLine($"family     {info.ProductFamily ?? "-"}" + (info.ControllerName != null ? $" ({info.ControllerName})" : string.Empty));
            if (info.IdentitySource != null && info.IdentitySource != "FC11")
            {
                output.WriteLine($"identity   from {info.IdentitySource} (FC11 not answered)");
            }

            output.WriteLine($"actuator   {info.Actuator ?? "-"}");
            var kind = info.Kind.StartsWith("kind ", StringComparison.Ordinal) ? info.Kind : "kind " + info.Kind;
            output.WriteLine($"units      {info.Units} ({kind})");
            if (info.MaxPositions is int rows && rows > 0)
            {
                output.WriteLine(I($"rows       {rows} stored positions (0..{rows - 1})"));
            }

            output.WriteLine($"model name {info.ModelName ?? "-"}");
            output.WriteLine($"unit name  {info.UserName ?? "-"} (0x061A, user-defined)");
            output.WriteLine($"serial     {info.SerialNumber ?? "-"}");
            output.WriteLine($"firmware   {info.Firmware ?? "-"}");
            if (info.AxisNumber.HasValue)
            {
                output.WriteLine(I($"axis no.   {info.AxisNumber.Value} (AXNO 0x8004; slave {axis.Slave})"));
            }

            if (info.SlaveIdRaw != null)
            {
                output.WriteLine("slave id   " + Hex(info.SlaveIdRaw));
            }

            if (axis.BaudRate.HasValue)
            {
                output.WriteLine(I($"baud       {axis.BaudRate.Value}{(axis.BaudDetected ? " (auto-detected)" : string.Empty)}"));
            }

            var u = info.Units;
            output.WriteLine("soft limit " + Range(info.SoftLimitsMm, u, "-"));
            output.WriteLine("stroke     " + Range(axis.EffectiveStroke, u, "unknown") + " (used for soft-limit checks)");
            var source = (info.SpeedLimitSource ?? string.Empty).StartsWith("MAXV", StringComparison.Ordinal) ? "MAXV 7.4" : "0x0812 fallback";
            var unitSpeed = info.MaxSpeedMmS.HasValue ? I($"{info.MaxSpeedMmS.Value:F2} {u}/s ({source})") : "not readable";
            output.WriteLine(I($"max speed  unit max {unitSpeed}; driver cap {axis.SpeedCapMmS:F2} {u}/s"));
            if (info.MaxAccelG.HasValue || info.MaxDecelG.HasValue)
            {
                var a = info.MaxAccelG.HasValue ? info.MaxAccelG.Value.ToString("F2", Inv) : "n/a";
                var d = info.MaxDecelG.HasValue ? info.MaxDecelG.Value.ToString("F2", Inv) : "n/a";
                output.WriteLine(I($"max accel  unit max {a} / {d} G (MAXA / MAXD); driver cap {axis.AccelCapG:F2} G"));
            }

            var limits = axis.AxisLimits;
            if (limits?.MaxPushPct != null)
            {
                var push = limits.PushAvailable == false
                    ? "not available"
                    : $"{G(limits.MinPushPct)} % (recommended) .. {G(limits.MaxPushPct.Value)} % (MXPP); driver cap {G(axis.PushCapPct)} %";
                output.WriteLine("push       " + push);
            }

            if (info.IndexHeader is IndexHeader h)
            {
                output.WriteLine(I($"index      MODL=0x{h.ModelCode:x4} records={h.RecordCount} rx/tx buffer={h.RxBuffer}/{h.TxBuffer}"));
            }

            foreach (var c in info.Categories)
            {
                output.WriteLine(I($"  CI=0x{c.CategoryId:x4} DA=0x{c.DataAddress:x4} DQ={c.MemberCount,-4} LQ={c.ListCount,-4} BA=0x{c.CoilBase:x4}"));
            }

            foreach (var err in info.Errors)
            {
                output.WriteLine("note       " + err);
            }
        }

        public static void PrintAlarms(IReadOnlyList<AlarmRecord> records, TextWriter output)
        {
            if (records.Count == 0)
            {
                output.WriteLine("alarm history empty");
            }

            foreach (var r in records)
            {
                var stamp = r.TimeAfterPowerOn ?? "-";
                output.WriteLine(I($"#{r.Index,-2} code 0x{r.Code:x4} {Alarms.LevelKey(r.Level),-19} detail {r.Detail,-5} {stamp,17} after power-on  {r.Name}"));
            }
        }

        public static void PrintLimits(AxisRanges limits, TextWriter output)
        {
            var kind = limits.Kind.StartsWith("kind ", StringComparison.Ordinal) ? limits.Kind : "kind " + limits.Kind;
            output.WriteLine($"units      {limits.Units} ({kind}, family {limits.Family})");
            var rows = new (string Name, LimitRange Range)[]
            {
                ("position", limits.Position), ("speed", limits.Speed), ("accel", limits.Accel), ("decel", limits.Decel),
                ("accel_decel", limits.AccelDecel), ("push", limits.Push), ("band", limits.Band), ("push_band", limits.PushBand),
                ("stored_rows", limits.StoredRows), ("jog_step", limits.JogStep), ("speed_override", limits.SpeedOverride),
            };
            foreach (var (name, r) in rows)
            {
                var range = $"{(r.Min.HasValue ? G(r.Min.Value) : "-")} .. {(r.Max.HasValue ? G(r.Max.Value) : "-")}";
                var extra = new List<string>();
                if (r.Available.HasValue)
                {
                    extra.Add(r.Available.Value ? "available" : "NOT available");
                }

                if (r.RecommendedMin.HasValue)
                {
                    extra.Add("recommended min " + G(r.RecommendedMin.Value));
                }

                if (r.Source.Length > 0)
                {
                    extra.Add(r.Source);
                }

                output.WriteLine($"{name,-14} {range,-22} {r.Unit,-6} {string.Join("; ", extra)}".TrimEnd());
            }
        }

        public static void PrintMonitor(MonitorReading m, TextWriter output)
        {
            var u = m.Units;
            var pct = m.CurrentPct.HasValue ? $" ({G(m.CurrentPct.Value)} % of rated)" : string.Empty;
            var alarm = m.Alarm != 0 ? ("0x" + m.Alarm.ToString("X3", Inv) + " " + m.AlarmName).TrimEnd() : "-";
            output.WriteLine(
                $"position {Num(m.Position, "F2")} {u}  velocity {Num(m.Velocity, "F2")} {u}/s  current {Num(m.Current)}{pct}  " +
                $"deviation {Num(m.Deviation)}  alarm {alarm}  uptime {Num(m.UptimeS, "F1")} s");
        }

        public static void PrintUserParameters(IReadOnlyList<DecodedParameter> parameters, ParameterModel model, TextWriter output, string units)
        {
            output.WriteLine($"user parameters ({model.Note})");
            foreach (var p in parameters)
            {
                var text = units == "mm" ? p.Text : p.Text.Replace(" mm", " " + units);
                if (!p.Found)
                {
                    output.WriteLine(I($"  P{p.No,-3} {p.Name,-52} {"not located",-34} read-only"));
                    continue;
                }

                var flags = new List<string>();
                if (p.Writable)
                {
                    flags.Add("writable");
                }

                if (p.ReadOnly)
                {
                    flags.Add("READ-ONLY");
                }

                if (p.HomeParameter)
                {
                    flags.Add("HOME, DANGEROUS");
                }

                if (p.ModelNote.Length > 0)
                {
                    flags.Add(p.ModelNote);
                }

                output.WriteLine(I($"  P{p.No,-3} {p.Name,-52} {text,-34} {p.Key} @{p.Address} = {p.RawHex}  {string.Join("; ", flags)}").TrimEnd());
            }
        }

        public static void PrintGenericParameters(IReadOnlyList<ParameterInfo> parameters, TextWriter output)
        {
            if (parameters.Count == 0)
            {
                output.WriteLine("no named parameter found in this unit's ID map");
            }

            foreach (var p in parameters)
            {
                var flags = new List<string> { p.Writable ? "writable" : "READ-ONLY" };
                if (p.Dangerous)
                {
                    flags.Add("DANGEROUS");
                }

                var range = p.Min.HasValue && p.Max.HasValue ? $"[{G(p.Min.Value)}..{G(p.Max.Value)}]" : string.Empty;
                output.WriteLine(I($"  No.{p.Number,-4} {p.Name,-44} {p.Text ?? "-",-22} {range,-18} {p.Item} @{p.Address} = {p.Raw}  {string.Join("; ", flags)}"));
            }
        }

        /// <summary>Print a parameter write; <paramref name="again"/>: the command that commits this value later (<c>param set KEY VALUE</c>).</summary>
        public static void PrintParameterWrite(ParameterWriteResult result, TextWriter output, string prefix, string again = "")
        {
            var label = result.Number != 0 ? prefix + result.Number.ToString(Inv) : "item";
            if (result.Skipped)
            {
                output.WriteLine($"{label} {result.Name} ({result.Key} @{result.Address}) = {result.After} (raw {result.RawAfter}): {result.Note}; --force writes it anyway "
                    + "(needed when an earlier --no-commit run left this value in RAM only: this run cannot see that)");
                foreach (var warning in result.Warnings)
                {
                    output.WriteLine("warning: " + warning);
                }

                return;
            }

            if (!result.Changed && !result.Committed)
            {
                // --no-commit with the value the unit already holds: nothing was sent
                output.WriteLine($"{label} {result.Name} ({result.Key} @{result.Address}) = {result.After} (raw {result.RawAfter}): unchanged, the unit already holds this value; nothing written (--no-commit)");
                foreach (var warning in result.Warnings)
                {
                    output.WriteLine("warning: " + warning);
                }

                return;
            }

            output.WriteLine($"{label} {result.Name} ({result.Key} @{result.Address}): {result.Before} -> {result.After} (raw {result.RawBefore} -> {result.RawAfter}, read back OK)");
            if (result.Committed && result.CommitConfirmed)
            {
                output.WriteLine("committed to non-volatile memory (PARW 0x0428: the unit showed the commit, then idle again)");
            }
            else if (result.Committed)
            {
                output.WriteLine("PARW 0x0428 sent, but the commit was NOT CONFIRMED (the unit never showed it busy): the value may be RAM only - check it after a power cycle");
            }
            else
            {
                output.WriteLine($"RAM ONLY (--no-commit): lost at the next power-up. To save it later run '{(again.Length > 0 ? again : "param set KEY VALUE")} --force' "
                    + "(without --force a new run sees the value already in the unit and commits nothing)");
            }

            output.WriteLine($"takes effect {result.TakesEffect}; run 'swreset' to apply it now");
            foreach (var warning in result.Warnings)
            {
                output.WriteLine("warning: " + warning);
            }
        }

        public static void PrintReset(SoftwareResetResult result, TextWriter output)
        {
            var reply = result.Accepted ? "accepted (45 00 00)" : "answer lost; confirmed from the status";
            output.WriteLine(I($"software reset {reply}, unit answering again after {result.ElapsedS:F2} s{(result.BaudRate.HasValue ? I($" at {result.BaudRate.Value} baud") : string.Empty)}"));
            output.WriteLine(I($"PMSS {(result.ModbusEnabled ? 1 : 0)}  SFTY {(result.SafetySpeed ? 1 : 0)}  servo {(result.ServoOn ? "ON" : "OFF")}  HEND {(result.Homed ? 1 : 0)}  alarm 0x{result.AlarmCode:x3}"));
            output.WriteLine(result.Confirmed
                ? $"restart confirmed by {(result.ConfirmedBy.Length > 0 ? result.ConfirmedBy : "the status")}"
                : "restart NOT confirmed (no PMSS drop, reply gap, uptime restart or HEND loss seen): check the unit");
            if (result.NeedsHome)
            {
                output.WriteLine("NEEDS HOME: run 'home' (after 'alarm-reset' if an alarm is active) before the next move");
            }

            foreach (var note in result.Notes)
            {
                output.WriteLine("note: " + note);
            }
        }

        public static void PrintBaud(BaudChangeResult result, TextWriter output)
        {
            var how = result.Verified ? "verified" : "found by probing";
            output.WriteLine(I($"baud {result.Previous} -> {result.BaudRate} ({result.Method.ToString().ToLowerInvariant()}; broadcast {result.Frame}; {how})"));
            if (!result.CodeVerified)
            {
                output.WriteLine(I($"warning: code 0x{result.Code:X2} is not verified on hardware (sent anyway)"));
            }

            output.WriteLine("note: " + result.Note);
        }

        /// <summary>
        /// <see cref="Axis.UnitInfo"/> as indented text: one block per section, nested dictionaries indented, short lists of
        /// plain values on one line (<c>--json</c> carries the same data).
        /// </summary>
        public static void PrintUnitInfo(IDictionary<string, object?> data, TextWriter output)
        {
            var meta = data.TryGetValue("meta", out var m) ? m as IDictionary : null;
            output.WriteLine($"{CliApp.Banner} - unit info, read-only (FC03 + FC11), {Scalar(meta?["taken_at"])}, {Scalar(meta?["duration_s"])} s");
            foreach (var kv in data)
            {
                if (kv.Key == "meta" || kv.Value is null)
                {
                    continue;
                }

                if (kv.Value is IList list && list.Count == 0)
                {
                    continue;
                }

                output.WriteLine();
                output.WriteLine($"[{kv.Key}]");
                Tree(output, kv.Value, 2);
            }
        }

        private static void Tree(TextWriter output, object? value, int indent)
        {
            var pad = new string(' ', indent);
            switch (value)
            {
                case IDictionary dict:
                    foreach (DictionaryEntry e in dict)
                    {
                        var key = Convert.ToString(e.Key, Inv);
                        if (IsScalar(e.Value) || IsFlatList(e.Value))
                        {
                            output.WriteLine($"{pad}{key,-22} {Inline(e.Value)}".TrimEnd());
                        }
                        else
                        {
                            output.WriteLine($"{pad}{key}:");
                            Tree(output, e.Value, indent + 2);
                        }
                    }

                    break;
                case IEnumerable items when !(value is string):
                    var i = 0;
                    foreach (var item in items)
                    {
                        if (IsScalar(item) || IsFlatList(item))
                        {
                            output.WriteLine($"{pad}- {Inline(item)}");
                        }
                        else
                        {
                            output.WriteLine(I($"{pad}#{i}"));
                            Tree(output, item, indent + 2);
                        }

                        i++;
                    }

                    break;
                default:
                    output.WriteLine(pad + Scalar(value));
                    break;
            }
        }

        /// <summary>
        /// What <see cref="Axis.Recover"/> found and did, as the Python edition's <c>recover</c> prints it: the summary, the restart signs,
        /// uptime, baud rate, PC control, servo, home, position, alarm, what was done, differences, notes and the verdict (nothing was moved).
        /// </summary>
        public static void PrintRecovery(RecoveryReport report, string units, TextWriter output)
        {
            output.WriteLine(report.Summary);
            if (report.RestartSigns.Count > 0)
            {
                output.WriteLine("restart signs: " + string.Join("; ", report.RestartSigns));
            }

            if (report.ToDictionary().TryGetValue("status", out var status) && status is IDictionary<string, object?> st
                && st.TryGetValue("raw", out var raw) && raw is IDictionary<string, object?> words && words.TryGetValue("STIM", out var stim) && stim != null)
            {
                output.WriteLine(I($"uptime     {Convert.ToDouble(stim, Inv) / 1000.0,9:F1} s (the unit's power-on time; a power-up restarts it)"));
            }

            output.WriteLine("baud rate  " + (report.BaudRate.HasValue && report.BaudRate.Value != 0 ? report.BaudRate.Value.ToString(Inv) : "n/a") + (report.BaudRestored ? " (restored)" : string.Empty));
            var control = report.ControlRestored ? " (taken again)" : report.ControlKept ? " (kept)" : string.Empty;
            output.WriteLine("PC control " + (report.ModbusControl == true ? "yes" : "no") + control);
            output.WriteLine("servo      " + (report.ServoOn == true ? "ON" : "OFF"));
            output.WriteLine("homed      " + (report.Homed == true ? "yes" : "NO: home() before the next move"));
            if (report.PositionMm.HasValue)
            {
                var where = I($"position   {report.PositionMm.Value,9:F2} {units}");
                if (report.LastTargetMm.HasValue && report.PositionErrorMm.HasValue)
                {
                    where += I($" (target {report.LastTargetMm.Value:F2}, {report.PositionErrorMm.Value:+0.00;-0.00;+0.00})");
                }

                output.WriteLine(where);
            }

            if (report.AlarmCode != 0)
            {
                output.WriteLine(I($"alarm      0x{report.AlarmCode:X3} {report.AlarmName ?? string.Empty} ({report.AlarmLevel})"));
            }

            foreach (var action in report.Actions)
            {
                output.WriteLine("done:      " + action);
            }

            foreach (var problem in report.LimitsChanged.Concat(report.RowProblems).Concat(report.ParameterDifferences))
            {
                output.WriteLine("differs:   " + problem);
            }

            foreach (var note in report.Notes)
            {
                output.WriteLine("note:      " + note);
            }

            output.WriteLine("ready to move: " + (report.ReadyToMove ? "yes" : "no") + "; needs attention: " + (report.NeedsAttention ? "YES" : "no") + " (nothing was moved)");
        }

        private static bool IsScalar(object? value) => value is null || value is string || !(value is IEnumerable);

        private static bool IsFlatList(object? value) =>
            value is IList list && !(value is string) && list.Count <= 8 && list.Cast<object?>().All(IsScalar);

        private static string Inline(object? value) =>
            value is IList list && !(value is string) ? "[" + string.Join(", ", list.Cast<object?>().Select(Scalar)) + "]" : Scalar(value);

        private static string Scalar(object? value) => value switch
        {
            null => "-",
            bool b => b ? "true" : "false",
            double d => G(d),
            float f => G(f),
            IFormattable x => x.ToString(null, Inv),
            _ => value.ToString() ?? string.Empty,
        };

        private static string Range((double Min, double Max)? r, string units, string missing) =>
            r.HasValue ? I($"{r.Value.Min:F2}..{r.Value.Max:F2} {units}") : missing;

        private static string Num(double? v, string format) => v.HasValue ? v.Value.ToString(format, Inv) : "-";

        private static string Num(int? v) => v.HasValue ? v.Value.ToString(Inv) : "-";

        private static string G(double v) => v.ToString("0.######", Inv);

        private static string I(FormattableString text) => text.ToString(Inv);
    }
}
