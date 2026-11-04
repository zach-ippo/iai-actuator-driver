using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace IaiActuatorDriver.Cli
{
    /// <summary>
    /// The commands <c>check</c>, <c>backup</c> and <c>recover</c>: the compatibility self-test, a read-only JSON backup of the unit and
    /// the recovery after a power or link loss (the Python edition's commands, same options and output).
    /// </summary>
    internal static class CliTools
    {
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        // ------------------------------------------------------------------ check

        /// <summary>
        /// <c>check [--motion] [--report FILE]</c>: the self-test (read-only unless <c>--motion</c>), each line printed as it is
        /// finished, then the JSON report written (default <c>iai-check-YYYYMMDD-HHMMSS.json</c>). Exit code 0 when no line failed, else 1.
        /// </summary>
        public static int Check(Axis axis, string label, CliArgs args, TextWriter output, TextWriter error, System.Threading.CancellationToken ct)
        {
            output.WriteLine(CliApp.Banner);
            output.WriteLine(string.Format(Inv, "compatibility check on {0}, slave {1}, strategy {2}", label, axis.Slave, axis.Strategy.Method.ToString().ToLowerInvariant()));
            // Ctrl+C ends the test (a decel stop when it moves) and the tool exits with 130, no report (as the Python
            // edition); before, check --motion kept moving and exited 0 (ON-WIRE on a PCON-CB)
            var report = axis.RunCheck(
                new CheckOptions { Motion = args.Has("--motion"), PortLabel = label },
                item => output.WriteLine(item.ToLine()),
                ct);
            ct.ThrowIfCancellationRequested();  // a test the cancel ended returns its report (the motion line failed)
            var path = args.Value("--report") ?? string.Format(Inv, "iai-check-{0:yyyyMMdd-HHmmss}.json", DateTime.Now);
            var where = SaveReport(report, path, output, error);
            output.WriteLine(string.Format(Inv, "RESULT {0} - {1}", report.Passed ? "PASS" : "FAIL", where));
            return report.Passed ? 0 : 1;
        }

        /// <summary>Write the JSON report; a reporting failure falls back to the temp folder, then to the output.</summary>
        private static string SaveReport(CheckReport report, string path, TextWriter output, TextWriter error)
        {
            try
            {
                return "report written to " + report.WriteJson(path);
            }
            catch (Exception exc) when (exc is IOException || exc is UnauthorizedAccessException)
            {
                error.WriteLine($"warning: could not write the report to {path}: {exc.Message}");
            }

            var fallback = Path.Combine(Path.GetTempPath(), Path.GetFileName(path));
            try
            {
                return "report written to " + report.WriteJson(fallback) + " (fallback location)";
            }
            catch (Exception exc) when (exc is IOException || exc is UnauthorizedAccessException)
            {
                error.WriteLine($"warning: could not write the report to {fallback} either: {exc.Message}");
            }

            output.WriteLine(report.ToJson());
            return "report printed above";
        }

        // ------------------------------------------------------------------ backup

        /// <summary>The <c>--rows</c> option: <c>used</c> (default), <c>all</c> or row numbers such as <c>0,1,2</c>; <see cref="ArgumentException"/> with the message the Python edition prints.</summary>
        public static string ParseRows(string? text)
        {
            var value = text ?? "used";
            if (value == "used" || value == "all")
            {
                return value;
            }

            var bad = $"--rows must be used, all or row numbers such as 0,1,2 (got '{value}')";
            var parts = value.Replace(" ", string.Empty).Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 0 || parts.Any(p => !int.TryParse(p, NumberStyles.Integer, Inv, out var n) || n < 0))
            {
                throw new ArgumentException(bad);
            }

            return string.Join(",", parts.Select(p => int.Parse(p, Inv).ToString(Inv)));
        }

        /// <summary>
        /// <c>backup [FILE] [--rows used|all|0,1,2]</c>: a read-only JSON backup of this unit (identity, directory, parameters, rows, alarms,
        /// maintenance counters; FC03 and FC11 only) written to FILE (default <c>ec-backup-&lt;port&gt;-s&lt;slave&gt;-YYYYMMDD-HHMMSS.json</c>).
        /// </summary>
        public static int Backup(Axis axis, string label, IReadOnlyList<string> positional, CliArgs args, bool json, TextWriter output, Action<TextWriter, object?> writeJson)
        {
            var rows = ParseRows(args.Value("--rows"));
            var safe = new string(label.Select(c => char.IsLetterOrDigit(c) ? c : '-').ToArray());
            var path = positional.Count > 0 ? positional[0] : string.Format(Inv, "ec-backup-{0}-s{1}-{2:yyyyMMdd-HHmmss}.json", safe, axis.Slave, DateTime.Now);
            var data = axis.Backup(path, rows);
            if (json)
            {
                writeJson(output, data);
                return 0;
            }

            var meta = Dict(data, "meta");
            var parameters = Dict(data, "parameters");
            var items = List(parameters, "classes").OfType<IDictionary<string, object?>>().Sum(block => List(block, "items").Count);
            var named = List(parameters, "named").Count;
            var positions = List(Dict(data, "positions"), "rows").Count;
            var history = List(Dict(data, "alarms"), "history").Count;
            var written = meta.TryGetValue("path", out var p) && p != null ? Convert.ToString(p, Inv) : path;
            output.WriteLine(string.Format(Inv, "backup written to {0} (read-only: FC03 / FC11 only; {1} s)", written, Convert.ToString(meta.TryGetValue("duration_s", out var d) ? d : 0, Inv)));
            output.WriteLine(string.Format(Inv, "contents   identity, directory, {0} parameter items ({1} named), {2} position rows, {3} alarm entries, maintenance counters", items, named, positions, history));
            foreach (var err in List(data, "errors"))
            {
                output.WriteLine("not read   " + Convert.ToString(err, Inv));
            }

            return 0;
        }

        private static IDictionary<string, object?> Dict(IDictionary<string, object?> data, string key) =>
            data.TryGetValue(key, out var value) && value is IDictionary<string, object?> d ? d : new Dictionary<string, object?>();

        private static IList<object?> List(IDictionary<string, object?> data, string key) =>
            data.TryGetValue(key, out var value) && value is IList<object?> l ? l : new List<object?>();

        // ------------------------------------------------------------------ monitor --watch, move --change-*, teach

        /// <summary>
        /// <c>monitor --watch [--interval S] [--duration S]</c>: the status monitor with the link watch. A line for the status changes (at most
        /// one per <c>--interval</c>, the newest) and one for every link event, until Ctrl+C or <c>--duration</c>; then a summary. Read-only.
        /// With <c>--json</c> one JSON object per line. Exit code 2 when the monitor ended on an error (the port gone: run <c>recover</c>).
        /// </summary>
        public static int Watch(Axis axis, string label, CliArgs args, bool json, TextWriter output, TextWriter error, System.Threading.CancellationToken ct)
        {
            try
            {
                axis.Track(label);  // read-only: the baseline restart detection compares with
            }
            catch (EcDriverException exc)
            {
                error.WriteLine($"note: not tracked ({exc.GetType().Name}: {exc.Message}): a restart is not detected, a lost link still is");
            }

            var items = new System.Collections.Concurrent.BlockingCollection<(string Kind, object Item)>();
            axis.StatusChanged += (_, e) => items.Add(("status", e.Status));
            axis.LinkChanged += (_, e) => items.Add(("event", e.Event));
            var monitor = axis.StartMonitor();
            var clock = System.Diagnostics.Stopwatch.StartNew();
            var limit = args.Double("--duration") is double seconds ? Math.Max(0.0, seconds) : (double?)null;
            var interval = Math.Max(0.0, args.Double("--interval") ?? 0.5);
            Status? pending = null;
            var lastPrint = double.NegativeInfinity;
            var code = 0;

            void Emit(string kind, IDictionary<string, object?> payload, string text)
            {
                var t = Math.Round(clock.Elapsed.TotalSeconds, 3);
                if (json)
                {
                    var line = new Dictionary<string, object?> { ["t"] = t, ["type"] = kind };
                    foreach (var kv in payload)
                    {
                        line[kv.Key] = kv.Value;
                    }

                    output.WriteLine(System.Text.Json.JsonSerializer.Serialize(line));
                }
                else
                {
                    output.WriteLine(string.Format(Inv, "{0,8:F3} s  {1}", t, text));
                }

                output.Flush();
            }

            void Show(Status st) =>
                Emit("status", new Dictionary<string, object?> { ["status"] = AxisJson.Status(st) }, StatusLine(st, axis.Units));

            try
            {
                while (!ct.IsCancellationRequested && (limit is null || clock.Elapsed.TotalSeconds < limit))
                {
                    if (!items.TryTake(out var next, 50))
                    {
                        next = (string.Empty, null!);
                    }

                    if (next.Kind == "event" && next.Item is RecoveryEvent link)
                    {
                        Emit("event", link.ToDictionary(), link.Kind.ToUpperInvariant() + ": " + link.Message);
                    }
                    else if (next.Kind == "status" && next.Item is Status st)
                    {
                        pending = st;
                    }

                    if (pending != null && clock.Elapsed.TotalSeconds - lastPrint >= interval)
                    {
                        Show(pending);
                        pending = null;
                        lastPrint = clock.Elapsed.TotalSeconds;
                    }

                    if (!monitor.Running && items.Count == 0)
                    {
                        if (monitor.Error != null)
                        {
                            Emit("stopped", new Dictionary<string, object?> { ["error"] = monitor.Error }, $"status monitor stopped: {monitor.Error} (run 'recover')");
                            code = 2;
                        }

                        break;
                    }
                }
            }
            finally
            {
                StopMonitorCounted(axis, monitor);
            }

            if (pending != null)
            {
                Show(pending);
            }

            var rate = monitor.PollRateHz;
            var summary = new Dictionary<string, object?>
            {
                ["polls"] = monitor.Polls,
                ["poll_rate_hz"] = rate is null ? (double?)null : Math.Round(rate.Value, 1),
                ["errors"] = monitor.ErrorCount,
                ["link_checks"] = monitor.LinkChecks,
                ["link_state"] = axis.LinkState,
            };
            Emit("summary", summary, string.Format(Inv, "{0} polls ({1:0.######} /s), {2} errors, {3} link checks, link {4}", monitor.Polls, rate is null ? 0.0 : Math.Round(rate.Value, 1), monitor.ErrorCount, monitor.LinkChecks, summary["link_state"]));
            return code;
        }

        /// <summary>
        /// Stop the monitor so that the summary can still read its counters (the core drops the monitor with them, and
        /// <see cref="Axis.StopMonitor"/> keeps what the monitor shows just before). The monitor is held first, which waits for a poll or
        /// link check in flight: a check that ends after the last look is counted, as the C++ command line, which reads after the stop, counts it.
        /// </summary>
        private static void StopMonitorCounted(Axis axis, AxisMonitor monitor)
        {
            IDisposable? hold = null;
            try
            {
                hold = monitor.Suspend();
            }
            catch (Exception exc) when (exc is EcDriverException || exc is ObjectDisposedException)
            {
                // the monitor ended on an error or the axis is closed: what was seen last stays
            }

            try
            {
                axis.StopMonitor();
            }
            finally
            {
                hold?.Dispose();
            }
        }

        /// <summary>One compact line of a (lean) status for <c>monitor --watch</c>.</summary>
        private static string StatusLine(Status st, string units)
        {
            var flags = new List<string>();
            foreach (var (name, on) in new[]
            {
                ("SERVO", st.ServoOn), ("HOMED", st.Homed), ("IN-POS", st.InPosition), ("MOVING", st.Moving), ("PAUSED", st.Paused),
                ("HOMING", st.Homing), ("PUSHING", st.Pushing), ("MODBUS", st.ModbusEnabled), ("ALARM", st.Alarm), ("WARNING", st.Warning),
                ("EMG", st.EmergencyStop),
            })
            {
                if (on)
                {
                    flags.Add(name);
                }
            }

            var velocity = st.VelocityMmS is double v ? string.Format(Inv, "  velocity {0,8:F2} {1}/s", v, units) : string.Empty;
            var alarm = st.AlarmCode != 0 ? string.Format(Inv, "  alarm 0x{0:X3}", st.AlarmCode) : string.Empty;
            return string.Format(Inv, "position {0,9:F2} {1}{2}  {3}{4}", st.PositionMm, units, velocity, flags.Count == 0 ? "-" : string.Join(" ", flags), alarm);
        }

        /// <summary>
        /// <c>move POS --change-speed V / --change-accel G [--change-after S]</c>: start the move, wait <c>--change-after</c> seconds, change the running
        /// move and wait for it to finish. VERIFIED on the EC-GRBP8M gripper (the speed change also on the PCON-CB).
        /// </summary>
        public static Status? MoveAndChange(Axis axis, double target, CliArgs args, TextWriter error, System.Threading.CancellationToken ct)
        {
            axis.MoveTo(target, args.Double("--speed") ?? 20.0, args.Double("--accel") ?? 0.1, args.Double("--band") ?? 0.1, wait: false);
            if (ct.WaitHandle.WaitOne(TimeSpan.FromSeconds(Math.Max(0.0, args.Double("--change-after") ?? 1.0))))
            {
                ct.ThrowIfCancellationRequested();
            }

            Status? result = null;
            if (args.Double("--change-speed") is double speed)
            {
                error.WriteLine(string.Format(Inv, "changing the speed of the running move to {0:0.######} {1}/s", speed, axis.Units));
                result = axis.ChangeSpeed(speed, wait: !args.Has("--change-accel"), cancellationToken: ct);
            }

            if (args.Double("--change-accel") is double accel)
            {
                error.WriteLine(string.Format(Inv, "changing the acceleration of the running move to {0:0.######} G", accel));
                result = axis.ChangeAccel(accel, wait: true, cancellationToken: ct);
            }

            return result;
        }

        /// <summary>
        /// <c>teach N [--force] [--method table|coil]</c>: store the current position as stored row N (non-volatile;
        /// --allow-nonvolatile is accepted and ignored).
        /// </summary>
        public static int Teach(Axis axis, int n, CliArgs args, bool json, TextWriter output, Action<TextWriter, object?> writeJson)
        {
            var viaCoils = (args.Value("--method") ?? "table") switch
            {
                "table" => false,
                "coil" => true,
                var other => throw new UsageException($"--method: table or coil, got '{other}'"),
            };
            var entry = axis.TeachPosition(n, viaCoils: viaCoils, force: args.Has("--force"));
            if (json)
            {
                writeJson(output, entry.ToDictionary());
            }
            else if (entry.Skipped)
            {
                output.WriteLine(string.Format(Inv, "row {0} already holds the current position {1:F2} {2}: {3}; --force writes it anyway", n, entry.PositionMm, axis.Units, Axis.NvSkipNote));
            }
            else
            {
                output.WriteLine(string.Format(Inv, "row {0} taught from the current position: {1:F2} {2}, speed {3:F2}, accel / decel {4:F2} / {5:F2} G, band {6:F2} (non-volatile, read back)", n, entry.PositionMm, axis.Units, entry.SpeedMmS, entry.AccelG, entry.DecelG, entry.BandMm));
            }

            return 0;
        }

        // ------------------------------------------------------------------ recover

        /// <summary>
        /// <c>recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]</c>: bring the axis back after a power or
        /// link loss (same unit, PC control, baud rate, message alarms) and report home, alarm and position. It never moves the axis.
        /// </summary>
        public static int Recover(Axis axis, string label, CliArgs args, bool json, TextWriter output, TextWriter error, Action<TextWriter, object?> writeJson, System.Threading.CancellationToken ct)
        {
            var backupFile = args.Value("--backup");
            if (backupFile != null)
            {
                try
                {
                    axis.TrackBackupFile(backupFile, label);
                }
                catch (Exception exc) when (exc is IOException || exc is ArgumentException || exc is UnauthorizedAccessException)
                {
                    error.WriteLine($"error: backup {backupFile}: {exc.Message}");
                    return 2;
                }
            }
            else
            {
                axis.Forget();
                axis.Identify();
                axis.TrackIdentity(label);
            }

            axis.NoteControl(modbusControl: !args.Has("--no-control"), baudRate: args.Int("--restore-baud"));
            if (args.Double("--target") is double target)
            {
                axis.NoteTarget(target);
            }

            var policy = new RecoveryPolicy
            {
                CheckRows = backupFile != null ? new int[0] : null,
                CheckParameters = backupFile != null,
                BackupFile = backupFile,
            };
            var report = axis.Recover(Math.Max(0.0, args.Double("--wait") ?? 30.0), policy, label, ct);
            if (json)
            {
                writeJson(output, report.ToDictionary());
            }
            else
            {
                CliOutput.PrintRecovery(report, axis.Units, output);
            }

            return 0;
        }
    }
}
