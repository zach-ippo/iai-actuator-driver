using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;

namespace IaiActuatorDriver.Cli
{
    /// <summary>
    /// <c>iai-actuator-driver-cs [--port COM3 | --sim PROFILE] [--slave N | --axis N] COMMAND</c>, plus the port-level commands
    /// <c>ports</c> and <c>scan</c> (the command line of the Python edition, same commands, opt-ins and exit codes).
    /// </summary>
    /// <remarks>
    /// Any IAI unit is driven the same way (ELECYLINDER or a CON-family controller such as a PCON): positions are in mm, or
    /// in deg on a rotary axis, and every range is read from the unit itself (<c>limits</c> prints them); <c>--max-speed</c>
    /// / <c>--max-accel</c> / <c>--max-push</c> only narrow them further. Commands that change the unit's state first read
    /// identity and ranges and take Modbus control while the axis is at rest; read-only commands write nothing. STOP is sent
    /// with or without Modbus control. Exit codes: 0 ok, 1 nothing found by <c>scan</c>, 2 error, 130 interrupted.
    /// </remarks>
    internal static class CliApp
    {
        public const string Banner = "iai-actuator-driver-cs (UNOFFICIAL - not an IAI product)";

        /// <summary>The tool's conservative push cap without --max-push / --unit-max (as in the Python edition).</summary>
        public const double DefaultMaxPushPct = 70.0;

        public const string DeprecatedResetNote =
            "note: 'reset' is deprecated and means ALARM reset; use 'alarm-reset'. The software reset is 'swreset'.";

        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private static readonly string[] ConnectionOptions =
        {
            "--port", "--sim", "--slave", "--axis", "--baud", "--timeout", "--retries", "--strategy", "--stroke", "--max-speed",
            "--max-accel", "--max-push", "--unit-max", "--log-frames", "--json",
        };

        /// <summary>Command -&gt; (positional arguments after the command, extra options).</summary>
        private static readonly Dictionary<string, (int Args, string[] Options)> AxisCommands = new Dictionary<string, (int, string[])>(StringComparer.Ordinal)
        {
            ["status"] = (0, new string[0]),
            ["identify"] = (0, new string[0]),
            ["info"] = (0, new[] { "--section" }),
            ["limits"] = (0, new string[0]),
            ["monitor"] = (0, new[] { "--count", "--interval", "--watch", "--duration" }),
            ["alarms"] = (0, new string[0]),
            ["servo"] = (1, new string[0]),
            ["home"] = (0, new[] { "--method" }),
            ["move"] = (1, new[] { "--speed", "--accel", "--decel", "--band", "--no-wait", "--fast", "--change-speed", "--change-accel", "--change-after" }),
            ["jog"] = (1, new[] { "--speed", "--fast" }),
            ["push"] = (3, new[] { "--toward-home", "--speed" }),
            ["goto"] = (1, new[] { "--speed-pct", "--allow-fast", "--no-wait" }),
            ["stop"] = (0, new string[0]),
            ["pause"] = (0, new string[0]),
            ["resume"] = (0, new string[0]),
            ["cancel-pause"] = (0, new string[0]),
            ["teach"] = (1, new[] { "--allow-nonvolatile", "--force", "--method" }),  // --allow-*: hidden no-ops
            ["alarm-reset"] = (0, new[] { "--method" }),
            ["reset"] = (0, new[] { "--method" }),
            ["mode"] = (1, new[] { "--safety" }),
            ["swreset"] = (0, new string[0]),
            ["baud"] = (1, new[] { "--method", "--allow-unverified-code" }),
            ["param"] = (-1, new[] { "--no-commit", "--allow-dangerous", "--allow-home-params", "--allow-unlisted", "--force" }),
            ["check"] = (0, new[] { "--motion", "--report" }),
            ["backup"] = (-1, new[] { "--rows" }),
            ["recover"] = (0, new[] { "--backup", "--target", "--restore-baud", "--no-control", "--wait" }),
        };

        private static readonly HashSet<string> StateChanging = new HashSet<string>(StringComparer.Ordinal)
        {
            "servo", "home", "move", "jog", "push", "goto", "reset", "alarm-reset", "swreset", "baud", "mode",
            "pause", "resume", "cancel-pause", "teach",
        };

        private static readonly JsonSerializerOptions JsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
            NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals,
            Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
        };

        private static readonly JsonSerializerOptions JsonLineOptions = new JsonSerializerOptions(JsonOptions) { WriteIndented = false };

        /// <summary>
        /// The tool's version as the project sets it: its informational version, e.g. "1.0.0" or "0.12.0-beta.1".
        /// </summary>
        public static string Version => VersionText(
            typeof(CliApp).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion,
            typeof(CliApp).Assembly.GetName().Version);

        /// <summary>
        /// The informational version without its build metadata (the "+commit" the SDK appends), so a pre-release keeps
        /// its label; the assembly version (major.minor.patch) only when there is none.
        /// </summary>
        internal static string VersionText(string? informational, Version? assembly)
        {
            if (!string.IsNullOrWhiteSpace(informational))
            {
                var plus = informational.IndexOf('+');
                return (plus >= 0 ? informational.Substring(0, plus) : informational).Trim();
            }

            return assembly?.ToString(3) ?? "0.0.0";
        }

        /// <summary>Run one command line; returns the exit code.</summary>
        public static int Run(IReadOnlyList<string> argv, TextWriter output, TextWriter error, CliEnvironment env, CancellationToken cancellationToken = default)
        {
            CliArgs args;
            try
            {
                args = CliArgs.Parse(argv);
            }
            catch (UsageException exc)
            {
                return Usage(error, exc.Message);
            }

            if (args.Has("--version"))
            {
                output.WriteLine("iai-actuator-driver-cs " + Version);
                return 0;
            }

            if (args.Has("--help"))
            {
                output.Write(HelpText);
                return 0;
            }

            if (args.Positionals.Count == 0)
            {
                return Usage(error, "a command is required");
            }

            try
            {
                var command = args.Positionals[0];
                switch (command)
                {
                    case "ports":
                        return RunPorts(args, output, env);
                    case "scan":
                        return RunScan(args, output, error, env);
                    default:
                        return RunAxisCommand(args, command, output, error, env, cancellationToken);
                }
            }
            catch (UsageException exc)
            {
                return Usage(error, exc.Message);
            }
        }

        // ------------------------------------------------------------------ port commands

        private static int RunPorts(CliArgs args, TextWriter output, CliEnvironment env)
        {
            args.Allow("ports", new[] { "--json" });
            if (args.Positionals.Count != 1)
            {
                throw new UsageException("'ports' takes no argument");
            }

            var entries = env.ListPorts();
            if (args.Has("--json"))
            {
                WriteJson(output, entries.Select(e => e.ToDictionary()).ToList());
            }
            else
            {
                CliOutput.PrintPorts(entries, output);
            }

            return 0;
        }

        private static int RunScan(CliArgs args, TextWriter output, TextWriter error, CliEnvironment env)
        {
            args.Allow("scan", new[] { "--json", "--ports", "--port", "--sim", "--slaves", "--baud", "--scan-timeout", "--timeout", "--log-frames", "--sim-slaves" });
            if (args.Positionals.Count != 1)
            {
                throw new UsageException("'scan' takes no argument (use --ports and --slaves)");
            }

            var portList = args.Value("--ports") ?? args.Value("--port");
            var profile = args.Value("--sim");
            if (portList != null && profile != null)
            {
                throw new UsageException("--ports and --sim exclude each other");
            }

            var slaves = args.Value("--slaves") is string s ? CliArgs.SlaveList("--slaves", s) : PortManager.DefaultSlaves;
            var scanTimeout = args.Double("--scan-timeout") ?? ModbusRtu.ScanTimeout.TotalSeconds;
            if (scanTimeout <= 0)
            {
                throw new UsageException("--scan-timeout must be > 0");
            }

            var baud = args.Int("--baud");
            var readTimeout = Seconds(args.Double("--timeout"));
            var logger = args.Has("--log-frames") ? FrameLogger(error) : null;
            var targets = new List<(string Name, Func<ITransport> Open, bool AutoBaud)>();
            if (profile != null)
            {
                CheckProfile(profile, env);
                var simSlaves = args.Value("--sim-slaves") is string ss ? CliArgs.SlaveList("--sim-slaves", ss) : new[] { 1 };
                targets.Add(("sim", () => env.CreateSim(profile, simSlaves, baud), true));
            }
            else if (portList != null)
            {
                foreach (var name in portList.Split(',').Select(n => n.Trim()).Where(n => n.Length > 0))
                {
                    targets.Add((name, () => env.OpenSerial(name, baud ?? ModbusRtu.DefaultBaudRate, readTimeout), baud is null));
                }
            }
            else
            {
                var names = env.ListPorts().Where(p => p.IsIai).Select(p => p.Port).ToList();
                if (names.Count == 0)
                {
                    error.WriteLine("error: no IAI USB adapter found; name the ports with --ports COM3,COM4");
                    return 2;
                }

                foreach (var name in names)
                {
                    targets.Add((name, () => env.OpenSerial(name, baud ?? ModbusRtu.DefaultBaudRate, readTimeout), baud is null));
                }
            }

            if (targets.Count == 0)
            {
                throw new UsageException("--ports: no port named");
            }

            var report = new Dictionary<string, object?>();
            var found = 0;
            var failed = 0;
            foreach (var (name, open, autoBaud) in targets)
            {
                int? rate = null;
                IReadOnlyList<ScanHit> hits = Array.Empty<ScanHit>();
                string? problem = null;
                try
                {
                    using var port = new Port(open(), name, frameLogger: logger, autoBaud: autoBaud);
                    hits = port.ScanUnits(slaves, TimeSpan.FromSeconds(scanTimeout));
                    rate = port.BaudRate;
                }
                catch (Exception exc) when (IsReportable(exc))
                {
                    problem = exc.GetType().Name + ": " + exc.Message;
                    failed++;
                }

                found += hits.Count;
                report[name] = new Dictionary<string, object?>
                {
                    ["baud"] = rate,
                    ["axes"] = hits.Select(h => (object?)h.ToDictionary()).ToList(),
                    ["error"] = problem,
                };
                if (!args.Has("--json"))
                {
                    CliOutput.PrintScan(name, rate, hits, problem, output);
                }
            }

            if (args.Has("--json"))
            {
                WriteJson(output, report);
            }

            if (failed == targets.Count)
            {
                return 2;
            }

            return found > 0 ? 0 : 1;
        }

        // ------------------------------------------------------------------ axis commands

        private static int RunAxisCommand(CliArgs args, string command, TextWriter output, TextWriter error, CliEnvironment env, CancellationToken ct)
        {
            if (!AxisCommands.TryGetValue(command, out var spec))
            {
                throw new UsageException($"unknown command '{command}'");
            }

            args.Allow(command, ConnectionOptions.Concat(spec.Options));
            var positional = args.Positionals.Skip(1).ToList();
            if (spec.Args >= 0 && positional.Count != spec.Args)
            {
                throw new UsageException(spec.Args == 0 ? $"'{command}' takes no argument" : $"'{command}' takes {spec.Args} argument(s), got {positional.Count}");
            }

            if (command == "param")
            {
                CheckParamArgs(positional, args);
            }
            else if (command == "servo" && positional[0] != "on" && positional[0] != "off")
            {
                throw new UsageException("servo: on or off");
            }
            else if (command == "mode" && positional[0] != "pc" && positional[0] != "pio")
            {
                throw new UsageException("mode: pc or pio");
            }

            CheckNumbers(command, positional, args);
            if (command == "backup")
            {
                if (positional.Count > 1)
                {
                    throw new UsageException("'backup' takes at most one argument (the output file)");
                }

                try
                {
                    CliTools.ParseRows(args.Value("--rows"));
                }
                catch (ArgumentException exc)
                {
                    error.WriteLine("error: " + exc.Message);
                    return 2;
                }
            }
            else if (command == "move" && (args.Has("--change-speed") || args.Has("--change-accel")) && (args.Has("--no-wait") || args.Has("--decel")))
            {
                error.WriteLine("error: --change-speed / --change-accel need a plain move that waits (no --no-wait, no --decel)");
                return 2;
            }
            else if (command == "recover" && args.Int("--restore-baud") is int restoreBaud && !ModbusRtu.VerifiedBaudRates.Contains(restoreBaud))
            {
                throw new UsageException("--restore-baud: choose from " + string.Join(", ", ModbusRtu.VerifiedBaudRates.OrderBy(r => r)));
            }

            var portName = args.Value("--port");
            var profile = args.Value("--sim");
            if ((portName is null) == (profile is null))
            {
                throw new UsageException("give exactly one of --port NAME and --sim PROFILE");
            }

            if (args.Has("--slave") && args.Has("--axis"))
            {
                throw new UsageException("--slave and --axis exclude each other");
            }

            var slave = args.Has("--axis") ? args.Int("--axis")!.Value + 1 : args.Int("--slave") ?? 1;
            if (slave < 1 || slave > 247)
            {
                error.WriteLine(I($"error: slave address must be 1..247 (axis 0..246), got {slave}"));
                return 2;
            }

            var options = AxisOptionsFrom(args);
            var baud = args.Int("--baud");
            var retries = args.Int("--retries") ?? ModbusRtu.DefaultRetries;
            if (retries < 0)
            {
                throw new UsageException("--retries must be >= 0");
            }

            var logger = args.Has("--log-frames") ? FrameLogger(error) : null;
            Axis axis;
            try
            {
                ITransport transport;
                bool autoBaud;
                if (profile != null)
                {
                    CheckProfile(profile, env);
                    transport = env.CreateSim(profile, new[] { slave }, baud);
                    autoBaud = true;
                }
                else
                {
                    transport = env.OpenSerial(portName!, baud ?? ModbusRtu.DefaultBaudRate, Seconds(args.Double("--timeout")));
                    autoBaud = baud is null;
                }

                axis = Axis.Open(transport, slave, options, autoBaud, retries, logger);
            }
            catch (Exception exc) when (IsReportable(exc))
            {
                ReportError(error, exc);
                return 2;
            }

            if (profile is null)
            {
                error.WriteLine(I($"note: {portName} at {axis.BaudRate} baud ({(axis.BaudDetected ? "auto-detected" : "fixed by --baud")})"));
            }

            try
            {
                using (axis)
                {
                    return Execute(axis, command, positional, args, output, error, ct, profile != null ? "sim" : portName!);
                }
            }
            catch (OperationCanceledException)
            {
                error.WriteLine("interrupted (a decel stop was sent if motion had been commanded)");
                return 130;
            }
            catch (Exception exc) when (IsReportable(exc))
            {
                ReportError(error, exc);
                return 2;
            }
        }

        private static int Execute(Axis axis, string command, IReadOnlyList<string> positional, CliArgs args, TextWriter output, TextWriter error, CancellationToken ct, string label)
        {
            var json = args.Has("--json");
            if (command == "reset")
            {
                error.WriteLine(DeprecatedResetNote);
            }

            if (StateChanging.Contains(command) || (command == "param" && positional[0] == "set"))
            {
                // Session start: identity, directory, soft limits and speed limit; Modbus control only at rest.
                var before = axis.Status();
                // a pause at a standstill with the servo off (a PCON in PIO mode with nothing on its PIO connector) does not stop Connect
                bool still = before.Paused && !before.ServoOn && !before.Moving && !before.Homing && !before.Pushing && before.Raw.TryGetValue("VNOW", out var vnow) && vnow == 0;
                if (!before.ModbusEnabled && (before.Idle || still))
                {
                    (json ? error : output).WriteLine("enabling Modbus control (PMSL) while the axis is at rest");  // stdout stays one JSON document
                }

                axis.Connect(takeControl: true);
                if (args.Has("--fast"))
                {
                    axis.EnableFast(baudRate: null);  // fast completion and the status monitor; the baud rate is kept
                }
            }

            switch (command)
            {
                case "param":
                    return ExecuteParam(axis, positional, args, output);
                case "limits":
                {
                    var limits = axis.Limits();
                    if (json)
                    {
                        WriteJson(output, limits.ToDictionary());
                    }
                    else
                    {
                        CliOutput.PrintLimits(limits, output);
                    }

                    return 0;
                }

                case "monitor" when args.Has("--watch"):
                    return CliTools.Watch(axis, label, args, json, output, error, ct);
                case "monitor":
                {
                    var count = Math.Max(1, args.Int("--count") ?? 1);
                    var interval = Math.Max(0.0, args.Double("--interval") ?? 0.5);
                    for (var i = 0; i < count; i++)
                    {
                        if (i > 0 && ct.WaitHandle.WaitOne(TimeSpan.FromSeconds(interval)))
                        {
                            ct.ThrowIfCancellationRequested();
                        }

                        var m = axis.ReadMonitor();
                        if (json)
                        {
                            var d = m.ToDictionary();
                            d.Remove("words");
                            output.WriteLine(JsonSerializer.Serialize(d, JsonLineOptions));
                        }
                        else
                        {
                            CliOutput.PrintMonitor(m, output);
                        }
                    }

                    return 0;
                }

                case "swreset":
                {
                    var done = axis.SoftwareReset();
                    if (json)
                    {
                        WriteJson(output, done.ToDictionary());
                    }
                    else
                    {
                        CliOutput.PrintReset(done, output);
                    }

                    return 0;
                }

                case "baud":
                {
                    var rate = CliArgs.ParseInt("baud", positional[0]);
                    if (!ModbusRtu.IaiBaudCodes.ContainsKey(rate))
                    {
                        throw new UsageException("baud: choose from " + string.Join(", ", ModbusRtu.IaiBaudCodes.Keys.OrderBy(r => r)));
                    }

                    var method = (args.Value("--method") ?? "direct") switch
                    {
                        "direct" => BaudChangeMethod.Direct,
                        "break" => BaudChangeMethod.Break,
                        var other => throw new UsageException($"--method: direct or break, got '{other}'"),
                    };
                    var done = axis.SetBaudRate(rate, method);  // --allow-unverified-code: ignored (no longer needed)
                    if (json)
                    {
                        WriteJson(output, done.ToDictionary());
                    }
                    else
                    {
                        CliOutput.PrintBaud(done, output);
                    }

                    return 0;
                }

                case "check":
                    return CliTools.Check(axis, label, args, output, error);
                case "backup":
                    return CliTools.Backup(axis, label, positional, args, json, output, WriteJson);
                case "recover":
                    return CliTools.Recover(axis, label, args, json, output, error, WriteJson, ct);
                case "info":
                {
                    var sections = args.Values("--section");
                    var data = axis.UnitInfo(sections.Count > 0 ? sections : null);
                    if (json)
                    {
                        WriteJson(output, data);
                    }
                    else
                    {
                        CliOutput.PrintUnitInfo(data, output);
                    }

                    return 0;
                }

                case "status":
                    return Show(axis.Status(), axis, json, output);
                case "identify":
                {
                    var info = axis.Identify();
                    if (json)
                    {
                        WriteJson(output, AxisJson.DeviceInfo(info));
                    }
                    else
                    {
                        CliOutput.PrintInfo(info, axis, output);
                    }

                    return 0;
                }

                case "alarms":
                {
                    var records = axis.ReadAlarmHistory();
                    if (json)
                    {
                        WriteJson(output, records.Select(r => (object?)AxisJson.AlarmRecord(r)).ToList());
                    }
                    else
                    {
                        CliOutput.PrintAlarms(records, output);
                    }

                    return 0;
                }

                case "servo":
                    return Show(axis.Servo(positional[0] == "on"), axis, json, output);
                case "home":
                    return Show(axis.Home(method: Method(args), cancellationToken: ct), axis, json, output);
                case "move":
                {
                    var target = CliArgs.ParseDouble("move POS", positional[0]);
                    var wait = !args.Has("--no-wait");
                    if (args.Has("--change-speed") || args.Has("--change-accel"))
                    {
                        return Show(CliTools.MoveAndChange(axis, target, args, error, ct), axis, json, output);
                    }

                    var st = axis.MoveTo(
                        target,
                        args.Double("--speed") ?? 20.0,
                        args.Double("--accel") ?? 0.1,
                        args.Double("--band") ?? 0.1,
                        wait,
                        decelG: args.Double("--decel"),
                        cancellationToken: ct);
                    if (st is null)
                    {
                        output.WriteLine(I($"move to {target:F2} {axis.Units} sent"));
                        return 0;
                    }

                    return Show(st, axis, json, output);
                }

                case "jog":
                    return Show(axis.Jog(CliArgs.ParseDouble("jog DELTA", positional[0]), args.Double("--speed") ?? 10.0, cancellationToken: ct), axis, json, output);
                case "push":
                {
                    var result = axis.Push(
                        CliArgs.ParseDouble("push POS", positional[0]),
                        CliArgs.ParseDouble("push BAND", positional[1]),
                        CliArgs.ParseDouble("push PCT", positional[2]),
                        args.Has("--toward-home") ? -1 : 1,
                        args.Double("--speed") ?? 10.0,
                        cancellationToken: ct);
                    if (result is null)
                    {
                        return 0;
                    }

                    if (json)
                    {
                        WriteJson(output, result.ToDictionary());
                    }
                    else
                    {
                        output.WriteLine(I($"push {(result.Contact ? "contact" : "missed")} at {result.PositionMm:F2} {axis.Units}"));
                        foreach (var warning in result.Warnings)
                        {
                            output.WriteLine("warning: " + warning);
                        }

                        CliOutput.PrintStatus(result.Status, output, axis.Units);
                    }

                    return 0;
                }

                case "goto":
                {
                    var n = CliArgs.ParseInt("goto N", positional[0]);
                    var pct = args.Double("--speed-pct");
                    if (pct is double p && p > 0 && p <= 100) // out of range: refused by MoveToPosition
                    {
                        // the unit runs (code + 1) / 256 of the row's speed (ON-WIRE on the PCON-CB)
                        var code = (int)Math.Max(1.0, Math.Min(255.0, Math.Round(256.0 * p / 100.0) - 1.0));
                        var runs = (code + 1) * 100.0 / 256.0;
                        if (Math.Abs(runs - p) > 0.05 * p)
                            error.WriteLine(I($"note: --speed-pct {p}: the row runs at {runs:0.00} % of its stored speed ")
                                            + I($"(override code {code}; the unit runs (code + 1) / 256)"));
                    }

                    var st = axis.MoveToPosition(n, pct, !args.Has("--no-wait"), allowAboveCaps: args.Has("--allow-fast"), cancellationToken: ct);
                    if (st is null)
                    {
                        output.WriteLine(I($"stored position {n} started"));
                        return 0;
                    }

                    return Show(st, axis, json, output);
                }

                case "stop":
                    return Show(axis.Stop(wait: true), axis, json, output);
                case "pause":
                    axis.Pause();
                    output.WriteLine("paused: the move is held and keeps its remaining travel; 'resume' continues it, 'cancel-pause' drops it (the pause stays set after this command ends)");
                    return 0;
                case "resume":
                    axis.Resume();
                    output.WriteLine("pause released: a paused move continues by itself");
                    return 0;
                case "cancel-pause":
                    return Show(axis.CancelPausedMove(), axis, json, output);
                case "teach":
                    return CliTools.Teach(axis, CliArgs.ParseInt("teach N", positional[0]), args, json, output, WriteJson);
                case "alarm-reset":
                case "reset":
                    return Show(axis.ResetAlarm(Method(args)), axis, json, output);
                case "mode":
                    return Show(axis.SetActionMode(positional[0] == "pc", args.Has("--safety")), axis, json, output);
                default:
                    throw new UsageException($"unknown command '{command}'");
            }
        }

        private static int ExecuteParam(Axis axis, IReadOnlyList<string> positional, CliArgs args, TextWriter output)
        {
            var json = args.Has("--json");
            var isEc = axis.IsEc;
            switch (positional[0])
            {
                case "list":
                    if (isEc)
                    {
                        var parameters = axis.UserParameterList();
                        if (json)
                        {
                            WriteJson(output, parameters);
                        }
                        else
                        {
                            CliOutput.PrintUserParameters(parameters, axis.ReadParameterModel(), output, axis.Units);
                        }
                    }
                    else
                    {
                        var parameters = axis.Parameters();
                        if (json)
                        {
                            WriteJson(output, parameters.Select(p => (object?)p.ToDictionary()).ToList());
                        }
                        else
                        {
                            output.WriteLine($"{axis.ProductFamily} parameters (offered when the item is in the unit's ID map)");
                            CliOutput.PrintGenericParameters(parameters, output);
                        }
                    }

                    return 0;
                case "get":
                    WriteJson(output, axis.ReadParameter(positional[1].Trim()).ToDictionary());
                    return 0;
                default:
                {
                    var key = positional[1].Trim();
                    object value = positional[2];
                    if (IsRawItemKey(key) && TryRawInteger(positional[2], out var raw))
                    {
                        value = raw;
                    }

                    // --allow-dangerous / --allow-unlisted / --allow-home-params: ignored (no longer needed)
                    var written = axis.WriteParameter(
                        key,
                        value,
                        commit: !args.Has("--no-commit"),
                        force: args.Has("--force"));
                    if (json)
                    {
                        WriteJson(output, written.ToDictionary());
                    }
                    else
                    {
                        CliOutput.PrintParameterWrite(written, output, isEc ? "P" : "No.", $"param set {key} {positional[2]}");
                    }

                    return 0;
                }
            }
        }

        // ------------------------------------------------------------------ helpers

        private static void CheckParamArgs(IReadOnlyList<string> positional, CliArgs args)
        {
            if (positional.Count == 0)
            {
                throw new UsageException("param: list, get KEY or set KEY VALUE");
            }

            var (count, sub) = positional[0] switch
            {
                "list" => (1, "list"),
                "get" => (2, "get KEY"),
                "set" => (3, "set KEY VALUE"),
                _ => throw new UsageException($"param: list, get KEY or set KEY VALUE (got '{positional[0]}')"),
            };
            if (positional.Count != count)
            {
                throw new UsageException("usage: param " + sub);
            }

            if (positional[0] != "set")
            {
                var setOnly = new[] { "--no-commit", "--allow-dangerous", "--allow-home-params", "--allow-unlisted", "--force" }.Where(args.Has).ToList();
                if (setOnly.Count > 0)
                {
                    throw new UsageException($"'param {positional[0]}' does not take {string.Join(", ", setOnly)}");
                }
            }
        }

        /// <summary>Every number on the line is parsed before any port is opened: a typo never costs a connection.</summary>
        private static void CheckNumbers(string command, IReadOnlyList<string> positional, CliArgs args)
        {
            foreach (var name in new[] { "--timeout", "--max-speed", "--max-accel", "--max-push", "--speed", "--accel", "--decel", "--band", "--interval", "--speed-pct", "--target", "--wait", "--duration", "--change-speed", "--change-accel", "--change-after" })
            {
                args.Double(name);
            }

            foreach (var name in new[] { "--slave", "--axis", "--baud", "--retries", "--count", "--restore-baud" })
            {
                args.Int(name);
            }

            switch (command)
            {
                case "move":
                    CliArgs.ParseDouble("move POS", positional[0]);
                    break;
                case "jog":
                    CliArgs.ParseDouble("jog DELTA", positional[0]);
                    break;
                case "push":
                    CliArgs.ParseDouble("push POS", positional[0]);
                    CliArgs.ParseDouble("push BAND", positional[1]);
                    CliArgs.ParseDouble("push PCT", positional[2]);
                    break;
                case "goto":
                    CliArgs.ParseInt("goto N", positional[0]);
                    break;
                case "baud":
                    CliArgs.ParseInt("baud", positional[0]);
                    break;
                case "teach":
                    CliArgs.ParseInt("teach N", positional[0]);
                    break;
            }
        }

        private static AxisOptions AxisOptionsFrom(CliArgs args)
        {
            (double Min, double Max)? stroke = null;
            if (args.Tuple("--stroke") is string[] s)
            {
                var lo = CliArgs.ParseDouble("--stroke MIN", s[0]);
                var hi = CliArgs.ParseDouble("--stroke MAX", s[1]);
                if (!(lo < hi))
                {
                    throw new UsageException("--stroke: MIN must be below MAX");
                }

                stroke = (lo, hi);
            }

            var strategy = (args.Value("--strategy") ?? "coil") switch
            {
                "coil" => CommandMethod.Coil,
                "iai" => CommandMethod.Iai,
                "iai2" => CommandMethod.Iai2,
                var other => throw new UsageException($"--strategy: coil, iai or iai2, got '{other}'"),
            };

            // The configured caps: the explicit options, else the conservative defaults (100 units/s, 0.3 G, 70 %) unless
            // --unit-max. The driver always lowers each to the unit's own maximum (MAXV / MAXA / MAXD / MXPP).
            var unitMax = args.Has("--unit-max");
            var speed = args.Double("--max-speed");
            var accel = args.Double("--max-accel");
            var push = args.Double("--max-push");
            try
            {
                return new AxisOptions
                {
                    Stroke = stroke,
                    Strategy = strategy,
                    MaxSpeedMmS = speed ?? (unitMax ? (double?)null : Axis.FallbackSpeedCap),
                    MaxAccelG = accel ?? (unitMax ? (double?)null : Axis.FallbackAccelCapG),
                    MaxPushPct = push ?? (unitMax ? (double?)null : DefaultMaxPushPct),
                };
            }
            catch (ArgumentException exc)
            {
                throw new UsageException(exc.Message);
            }
        }

        private static CommandMethod? Method(CliArgs args) => args.Value("--method") switch
        {
            null => null,
            "coil" => CommandMethod.Coil,
            "iai" => CommandMethod.Iai,
            var other => throw new UsageException($"--method: coil or iai, got '{other}'"),
        };

        private static void CheckProfile(string profile, CliEnvironment env)
        {
            if (!env.SimProfiles.Contains(profile))
            {
                throw new UsageException($"--sim: choose a profile from {string.Join(", ", env.SimProfiles)}, got '{profile}'");
            }
        }

        private static bool IsRawItemKey(string key) =>
            key.Contains('.') && !key.StartsWith("No.", StringComparison.OrdinalIgnoreCase);

        private static bool TryRawInteger(string text, out long value)
        {
            var t = text.Trim();
            if (t.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            {
                return long.TryParse(t.Substring(2), NumberStyles.AllowHexSpecifier, Inv, out value);
            }

            return long.TryParse(t, NumberStyles.AllowLeadingSign, Inv, out value);
        }

        private static TimeSpan? Seconds(double? seconds)
        {
            if (seconds is null)
            {
                return null;
            }

            if (seconds.Value <= 0)
            {
                throw new UsageException("--timeout must be > 0");
            }

            return TimeSpan.FromSeconds(seconds.Value);
        }

        private static Action<FrameDirection, byte[]> FrameLogger(TextWriter error)
        {
            var clock = Stopwatch.StartNew();
            var gate = new object();
            return (direction, frame) =>
            {
                lock (gate)
                {
                    error.WriteLine(I($"{clock.Elapsed.TotalSeconds:F4} {direction,-10} {CliOutput.Hex(frame)}"));
                }
            };
        }

        private static int Show(Status? st, Axis axis, bool json, TextWriter output)
        {
            if (st is null)
            {
                return 0;
            }

            if (json)
            {
                WriteJson(output, AxisJson.Status(st));
            }
            else
            {
                CliOutput.PrintStatus(st, output, axis.Units);
            }

            return 0;
        }

        internal static void WriteJson(TextWriter output, object? value) => output.WriteLine(JsonSerializer.Serialize(value, JsonOptions));

        /// <summary>Errors the tool reports with exit code 2 (anything else is a bug and keeps its stack trace).</summary>
        private static bool IsReportable(Exception exc) =>
            exc is EcDriverException || exc is IOException || exc is UnauthorizedAccessException || exc is ArgumentException
            || exc is InvalidOperationException || exc is TimeoutException || exc is NotSupportedException;

        private static void ReportError(TextWriter error, Exception exc)
        {
            error.WriteLine($"error: {exc.GetType().Name}: {exc.Message}");
            if (exc.Data["IaiActuatorDriver.Notes"] is string notes)
            {
                error.WriteLine("       " + notes);
            }
        }

        private static int Usage(TextWriter error, string message)
        {
            error.WriteLine("error: " + message);
            error.WriteLine("usage: iai-actuator-driver-cs (--port NAME | --sim PROFILE) [--slave N | --axis N] [options] COMMAND [args]");
            error.WriteLine("       iai-actuator-driver-cs ports | scan [--ports COM3,COM4] [--slaves 1-16]   (--help for every command)");
            return 2;
        }

        private static string I(FormattableString text) => text.ToString(Inv);

        private const string HelpText =
@"iai-actuator-driver-cs (UNOFFICIAL - not an IAI product)

usage: iai-actuator-driver-cs (--port NAME | --sim PROFILE) [--slave N | --axis N] [options] COMMAND [args]
       iai-actuator-driver-cs ports [--json]
       iai-actuator-driver-cs scan [--ports COM3,COM4 | --sim PROFILE] [--slaves 1-16] [--baud RATE] [--scan-timeout S] [--json]

Port commands (READ-ONLY, no --port needed):
  ports                        list the serial ports, IAI USB adapters (10C4:81D7) first; opens nothing
  scan                         find the axes that answer on each port (default: every IAI adapter, slaves 1-16)

Axis commands (positions in mm, or deg on a rotary axis; every range is read from the unit):
  status                       position and status bits (read-only)
  identify                     identity, directory, soft limits, unit maxima (read-only)
  info [--section S]...        everything the unit tells about itself (read-only, FC03 + FC11;
                               sections: identity, parameters, monitor, alarms, maintenance, positions, and
                               limits (an extra); no section: every one)
  limits                       every range the driver enforces, with its source (read-only)
  check [--motion] [--report FILE]
                               compatibility self-test (read-only unless --motion: also a short move out and
                               back); each line as it finishes, JSON report written (default
                               iai-check-<time>.json); exit code 1 when a line fails
  backup [FILE] [--rows used|all|0,1,2]
                               read-only JSON backup of this unit: identity, directory, parameters, rows,
                               alarms (default ec-backup-<port>-s<slave>-<time>.json; FC03 + FC11 only)
  recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]
                               bring the axis back after a power or link loss: same unit, PC control, baud
                               rate, message alarms, and a report of home, alarm and position (never moves;
                               --backup compares the stored rows and parameters with a 'backup' file;
                               --restore-baud switches the unit back to RATE (115200 or 230400; a broadcast);
                               --wait S waits that long for the unit to answer again, default 30)
  monitor [--count N] [--interval S] [--watch [--duration S]]
                               position, velocity, current, deviation, alarm, uptime (read-only); --watch runs
                               the status monitor with the link watch: a line per status change (at most one per
                               --interval) and every link event until Ctrl+C or --duration
  alarms                       alarm history (read-only)
  servo on|off                 servo on / off (on also takes Modbus control at rest)
  home [--method coil|iai]     home return and wait
  move POS [--speed V] [--accel G] [--decel G] [--band B] [--no-wait] [--fast]
       [--change-speed V] [--change-accel G] [--change-after S]
                               absolute move (default 20 units/s, 0.1 G, band 0.1); --decel different from
                               --accel uses the command-profile move; --fast: fast completion with the status
                               monitor feeding the wait (the baud rate is kept); --change-speed / --change-accel
                               change the running move after --change-after seconds (default 1; plain moves only)
  jog DELTA [--speed V] [--fast]
                               relative move by DELTA (default 10 units/s; never the unit's JOG coils)
  push POS BAND PCT [--toward-home] [--speed V]
                               approach POS, then push over BAND at PCT % force (capped by --max-push and MXPP)
  goto N [--speed-pct P] [--allow-fast] [--no-wait]
                               run stored row N at its stored speed (or P % of it; 0x45 cmd 6, in steps of
                               1/256, at least 0.78 %)
  stop                         decel stop (STOP coil on, wait until stopped, off); works without Modbus control
  pause | resume | cancel-pause
                               hold the move and keep its remaining travel (stays set after this command ends) /
                               release the pause / drop the remaining travel and release the pause
  teach N [--force] [--method table|coil]
                               store the current position as stored row N (non-volatile; a row that would not
                               change is not written, --force writes it anyway; coil is EXPERIMENTAL)
  alarm-reset [--method coil|iai]
                               alarm reset (not a software reset; 'reset' is a deprecated alias)
  mode pc|pio [--safety]       action mode: Modbus control (PMSL coil) or PIO (0x45 cmd 0x0C), safety speed on / off
  swreset                      software reset (servo off, then 0x45 cmd 5): applies committed parameters;
                               an incremental unit loses its home
  baud RATE [--method direct|break]
                               change the baud rate (broadcast 0x45 cmd 4); volatile: power-up rate after a
                               power cycle; verified codes: 115200, 230400; 38400 is sent and checked (some
                               units ignore its code); another code is sent with a warning
  param list                   the unit's parameters, decoded (EC: P-numbers; CON: No.-numbers; read-only)
  param get KEY                one parameter or raw item (5, P5, No.35, 8.15, C.1[1]; read-only)
  param set KEY VALUE [--no-commit] [--force]
                               write, verify and commit (PARW); applies after swreset or a power cycle; a
                               home, dangerous or unlisted parameter is written with a warning.
                               A value the unit already holds is neither written nor committed (the
                               non-volatile memory lasts about 100,000 write cycles, IAI); --force commits it.
                               Each run is a new process: a value an earlier --no-commit run left in RAM
                               looks unchanged, so commit it with 'param set KEY VALUE --force'

Connection and safety options:
  --port NAME                  serial port, e.g. COM3 (close IA-OS first: one program per port)
  --sim PROFILE                the simulator instead of hardware (slider, gripper, rod, pcon, rotary, unknown)
  --slave N | --axis N         Modbus address = axis number + 1 (default slave 1)
  --baud RATE                  fixed baud rate (default: detected in place: 115200, 38400, 9600, 230400, 57600, 19200)
  --timeout S                  response timeout in s (default 0.2)
  --retries N                  retries on timeout / CRC error (default 3, IAI's recommendation)
  --strategy coil|iai|iai2     servo / home / reset command path (default coil; iai2 is EXPERIMENTAL)
  --stroke MIN MAX             narrow the unit's soft limits (never widened)
  --max-speed V                speed cap (default 100 units/s), always lowered to the unit's MAXV (7.4)
  --max-accel G                acceleration cap (default 0.3 G), always lowered to MAXA / MAXD (7.5 / 7.6)
  --max-push PCT               push force cap (default 70 %), always lowered to MXPP (7.9)
  --unit-max                   lift the default caps up to the unit's own maxima
  --log-frames                 print every Modbus frame to stderr, with the time it was printed (a request
                               and its reply come together once the request has ended: not the reply time)
  --json                       machine-readable output

Commands that change the unit's state first read identity and ranges and take Modbus control while the axis
is at rest; read-only commands write nothing. Ctrl+C during a move or home sends the decel STOP sequence.
This tool is not a safety function: keep a hardware emergency stop.
Exit codes: 0 ok, 1 nothing found by scan, 2 error, 130 interrupted.
";
    }
}
