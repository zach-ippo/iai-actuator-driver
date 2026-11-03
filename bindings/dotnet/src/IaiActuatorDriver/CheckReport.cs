using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>The result of one line of the self-test.</summary>
    public enum CheckResult
    {
        /// <summary>The check passed.</summary>
        Pass,

        /// <summary>The check failed: the unit or the setup is not as the driver expects.</summary>
        Fail,

        /// <summary>Something to look at; does not fail the test.</summary>
        Warn,

        /// <summary>Not run (nothing to check, or an earlier line failed).</summary>
        Skip,
    }

    /// <summary>One line of the self-test (<see cref="Axis.RunCheck"/>).</summary>
    /// <param name="Name">The check ("link", "slave_id", "index_table", "identification", "soft_limits", ...).</param>
    /// <param name="Result">What it found.</param>
    /// <param name="Detail">In words.</param>
    /// <param name="Data">The values behind it (JSON-friendly: strings, numbers, booleans, lists, dictionaries).</param>
    public sealed record CheckItem(string Name, CheckResult Result, string Detail, IReadOnlyDictionary<string, object?> Data)
    {
        /// <summary>"PASS", "FAIL", "WARN" or "SKIP", as the report writes it.</summary>
        public string ResultText => Result.ToString().ToUpperInvariant();

        /// <summary>The line as the command line prints it: <c>PASS  link               answered FC03 ...</c>.</summary>
        public string ToLine() => string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0,-4}  {1,-18} {2}", ResultText, Name, Detail);

        /// <summary>The line as a JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["name"] = Name,
            ["result"] = ResultText,
            ["detail"] = Detail,
            ["data"] = Data.ToDictionary(kv => kv.Key, kv => kv.Value),
        };

        internal static CheckItem From(JsonObject o)
        {
            var result = (o.String("result") ?? "FAIL").ToUpperInvariant();
            return new CheckItem(
                o.String("name") ?? string.Empty,
                result == "PASS" ? CheckResult.Pass : result == "WARN" ? CheckResult.Warn : result == "SKIP" ? CheckResult.Skip : CheckResult.Fail,
                o.String("detail") ?? string.Empty,
                o.Object("data") is { } data ? JsonPlain.Dict(data) : new Dictionary<string, object?>());
        }
    }

    /// <summary>Settings of the self-test (<see cref="Axis.RunCheck"/>).</summary>
    public sealed record CheckOptions
    {
        /// <summary>Also do a short move out and back (up to <see cref="StepMm"/>, at most a quarter of the stroke). Without it the test only reads.</summary>
        public bool Motion { get; init; }

        /// <summary>Length of the move (units).</summary>
        public double StepMm { get; init; } = 5.0;

        /// <summary>Speed of the move (units/s).</summary>
        public double SpeedMmS { get; init; } = 10.0;

        /// <summary>Acceleration of the move (G).</summary>
        public double AccelG { get; init; } = 0.1;

        /// <summary>The port's name for the report; default the axis's own.</summary>
        public string? PortLabel { get; init; }
    }

    /// <summary>
    /// Everything the self-test found (<see cref="Axis.RunCheck"/>); the layout of the Python edition's <c>iai-check</c> report. Read-only by
    /// default (nothing is written to the unit): link, slave id, index table, identification, soft limits against the configured
    /// stroke, speed limit against the caps, status, alarm, alarm history, position table, direct-move block, Modbus control, safety speed,
    /// the unit information and the parameter ranges. With <see cref="CheckOptions.Motion"/> a short move out and back, after which the
    /// servo and Modbus-control state it found is restored.
    /// </summary>
    public sealed record CheckReport
    {
        /// <summary>ISO 8601 time the test started.</summary>
        public string Timestamp { get; init; } = string.Empty;

        /// <summary>The port's name.</summary>
        public string Port { get; init; } = string.Empty;

        /// <summary>The unit's Modbus address.</summary>
        public int Slave { get; init; }

        /// <summary>The command strategy ("coil", "iai", "iai2", "register").</summary>
        public string Strategy { get; init; } = string.Empty;

        /// <summary>The driver's version.</summary>
        public string DriverVersion { get; init; } = string.Empty;

        /// <summary>The effective stroke the checks used.</summary>
        public (double Min, double Max)? Stroke { get; init; }

        /// <summary>A motion check was requested.</summary>
        public bool MotionRequested { get; init; }

        /// <summary>The stroke the application configured; <c>null</c> = the unit's own.</summary>
        public (double Min, double Max)? ConfiguredStroke { get; init; }

        /// <summary>The unit's own soft limits.</summary>
        public (double Min, double Max)? SoftLimitsMm { get; init; }

        /// <summary>The unit's maximum speed (MAXV, item 7.4).</summary>
        public double? MaxSpeedMmS { get; init; }

        /// <summary>The speed the driver allows: the smaller of the configured cap and the unit's maximum.</summary>
        public double? SpeedCapMmS { get; init; }

        /// <summary>The baud rate; <c>null</c> on a transport without one.</summary>
        public int? BaudRate { get; init; }

        /// <summary>The unit's display name.</summary>
        public string? Model { get; init; }

        /// <summary>The controller (from the Report Slave ID).</summary>
        public string? Controller { get; init; }

        /// <summary>The model name string.</summary>
        public string? ModelName { get; init; }

        /// <summary>The serial number.</summary>
        public string? SerialNumber { get; init; }

        /// <summary>FC11 system code.</summary>
        public int? SystemCode { get; init; }

        /// <summary>FC11 unit code.</summary>
        public int? UnitCode { get; init; }

        /// <summary>The firmware version.</summary>
        public string? Firmware { get; init; }

        /// <summary>The lines, in the order they ran.</summary>
        public IReadOnlyList<CheckItem> Items { get; init; } = Array.Empty<CheckItem>();

        /// <summary>The unit information (<see cref="Axis.UnitInfo"/>), <c>null</c> when it could not be read.</summary>
        public IDictionary<string, object?>? UnitInfo { get; init; }

        /// <summary>No line failed (WARN and SKIP do not fail the test).</summary>
        public bool Passed => Items.All(i => i.Result != CheckResult.Fail);

        /// <summary>The report as a JSON-friendly dictionary (the Python edition's layout, plus <c>result</c> and <c>tool</c>).</summary>
        public IDictionary<string, object?> ToDictionary()
        {
            var d = new Dictionary<string, object?>(Details ?? new Dictionary<string, object?>(), StringComparer.Ordinal)
            {
                ["result"] = Passed ? "PASS" : "FAIL",
                ["tool"] = "iai-check (unofficial)",
            };
            return d;
        }

        /// <summary>The report as JSON text (indented by two spaces).</summary>
        public string ToJson() => JsonPlain.Write(ToDictionary());

        /// <summary>Write <see cref="ToJson"/> to <paramref name="path"/> (UTF-8); returns the path.</summary>
        public string WriteJson(string path)
        {
            File.WriteAllText(path, ToJson(), new UTF8Encoding(false));
            return path;
        }

        private IDictionary<string, object?>? Details { get; init; }

        internal static CheckReport From(string json)
        {
            var o = MiniJson.TryParse(json) as JsonObject ?? new JsonObject();
            var details = JsonPlain.Dict(o);
            details.Remove("result");
            details.Remove("tool");
            return new CheckReport
            {
                Timestamp = o.String("timestamp") ?? string.Empty,
                Port = o.String("port") ?? string.Empty,
                Slave = o.Int("slave") ?? 0,
                Strategy = o.String("strategy") ?? string.Empty,
                DriverVersion = o.String("driver_version") ?? string.Empty,
                Stroke = Span(o, "stroke"),
                MotionRequested = o.Bool("motion_requested") ?? false,
                ConfiguredStroke = Span(o, "configured_stroke"),
                SoftLimitsMm = Span(o, "ec_soft_limits_mm"),
                MaxSpeedMmS = o.Number("ec_max_speed_mm_s"),
                SpeedCapMmS = o.Number("speed_cap_mm_s"),
                BaudRate = o.Int("baud"),
                Model = o.String("model"),
                Controller = o.String("controller"),
                ModelName = o.String("model_name"),
                SerialNumber = o.String("serial_number"),
                SystemCode = o.Int("system_code"),
                UnitCode = o.Int("unit_code"),
                Firmware = o.String("firmware"),
                Items = (o.Array("items") ?? new List<object?>()).OfType<JsonObject>().Select(CheckItem.From).ToList(),
                UnitInfo = o.Object("unit_info") is { } info ? JsonPlain.Dict(info) : null,
                Details = details,
            };
        }

        private static (double Min, double Max)? Span(JsonObject o, string key) =>
            o.Array(key) is { Count: 2 } a && a[0] is double lo && a[1] is double hi ? (lo, hi) : ((double, double)?)null;
    }
}
