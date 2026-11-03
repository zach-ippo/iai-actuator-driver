using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;

namespace IaiActuatorDriver
{
    /// <summary>The states of an axis's link (<see cref="Axis.LinkState"/>, <see cref="LinkCheck.State"/>).</summary>
    public static class LinkStates
    {
        /// <summary>The link answers and nothing suggests a restart.</summary>
        public const string Ok = "ok";

        /// <summary>The serial port is gone (the adapter unplugged).</summary>
        public const string PortVanished = "port_vanished";

        /// <summary>The port is there but the unit does not answer (power lost, cable broken).</summary>
        public const string UnitSilent = "unit_silent";

        /// <summary>The unit answers but restarted (its uptime dropped, or PC control or HEND was lost).</summary>
        public const string UnitRestarted = "unit_restarted";

        /// <summary>Another unit answers where the tracked one was.</summary>
        public const string UnitReplaced = "unit_replaced";

        /// <summary>A <see cref="Axis.Recover"/> is running.</summary>
        public const string Recovering = "recovering";

        /// <summary>Every state.</summary>
        public static IReadOnlyList<string> All { get; } = new[] { Ok, PortVanished, UnitSilent, UnitRestarted, UnitReplaced, Recovering };
    }

    /// <summary>Result of <see cref="Axis.CheckLink"/>: one status read compared with what the axis's session remembers.</summary>
    /// <param name="State">One of <see cref="LinkStates"/>.</param>
    /// <param name="Reasons">The restart signs or the failure, in words.</param>
    /// <param name="Status">The status read; <c>null</c> when the unit did not answer.</param>
    /// <param name="Error">"Type: message" when the unit did not answer.</param>
    /// <param name="ElapsedS">How long the check took.</param>
    /// <param name="Axis">The axis ("COM3:1").</param>
    public sealed record LinkCheck(string State, IReadOnlyList<string> Reasons, Status? Status, string? Error, double ElapsedS, string? Axis)
    {
        /// <summary>The state is <see cref="LinkStates.Ok"/>.</summary>
        public bool Ok => State == LinkStates.Ok;

        /// <summary>The check as a JSON-friendly dictionary (the Python edition's keys).</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["axis"] = Axis,
            ["state"] = State,
            ["ok"] = Ok,
            ["reasons"] = Reasons.Cast<object?>().ToList(),
            ["error"] = Error,
            ["elapsed_s"] = ElapsedS,
            ["position_mm"] = Status?.PositionMm,
        };

        internal static LinkCheck From(NativeLinkCheck c, string? axis = null)
        {
            var json = MiniJson.TryParse(c.Json) as JsonObject;
            return new LinkCheck(
                c.State,
                c.Reasons,
                c.Status?.ToRecord(),
                c.Error.Length == 0 ? null : c.Error,
                c.ElapsedS ?? 0.0,
                axis ?? json?.String("axis"));
        }
    }

    /// <summary>What a recovery restores and checks (<see cref="Axis.Recover"/>). A recovery never starts motion.</summary>
    public sealed record RecoveryPolicy
    {
        /// <summary>
        /// Switch the unit back to the baud rate the application had set (only a verified baud-change code; a broadcast: every unit
        /// of a shared line switches; never behind a gateway).
        /// </summary>
        public bool RestoreBaudRate { get; init; } = true;

        /// <summary>Take PC control (PMSL) again when the application held it.</summary>
        public bool RestoreControl { get; init; } = true;

        /// <summary>Put the safety-speed mode back as it was.</summary>
        public bool RestoreSafetySpeed { get; init; } = true;

        /// <summary>Reset a message-level alarm (a warning such as 0x069); other alarms are left for the operator.</summary>
        public bool ClearMessageAlarms { get; init; } = true;

        /// <summary>
        /// Compare these stored rows with the session's copy or the backup and check their checksums; <c>null</c> = none. An empty
        /// list means every row of the session's copy (or rows 0 .. MaxPositions - 1).
        /// </summary>
        public IReadOnlyList<int>? CheckRows { get; init; }

        /// <summary>Compare the parameter classes with the session's copy or the backup.</summary>
        public bool CheckParameters { get; init; }

        /// <summary>A backup (<see cref="Axis.BackupJson"/> as JSON text) to compare with when the session has no copy.</summary>
        public string? BackupJson { get; init; }

        /// <summary>A backup file (UTF-8 JSON, as <see cref="Axis.BackupJson"/> writes it) to compare with; used when <see cref="BackupJson"/> is null.</summary>
        public string? BackupFile { get; init; }

        /// <summary>|position - last target| that still counts as at the target (units).</summary>
        public double PositionToleranceMm { get; init; } = 0.1;

        /// <summary>How often to look for the port and the unit while waiting (s).</summary>
        public double PollS { get; init; } = 0.5;

        /// <summary>The power-up baud rate probed first after a power loss.</summary>
        public int BootBaudRate { get; init; } = 38400;

        /// <summary>Refuse (<see cref="RecoveryException"/>) when no session is tracked, instead of using the identity read last.</summary>
        public bool RequireSession { get; init; }

        internal NativeRecoveryPolicy ToNative() => new NativeRecoveryPolicy
        {
            PositionToleranceMm = PositionToleranceMm,
            PollS = PollS,
            BootBaudRate = BootBaudRate,
            RestoreBaudRate = RestoreBaudRate,
            RestoreControl = RestoreControl,
            RestoreSafetySpeed = RestoreSafetySpeed,
            ClearMessageAlarms = ClearMessageAlarms,
            CheckParameters = CheckParameters,
            RequireSession = RequireSession,
        };
    }

    /// <summary>
    /// What <see cref="Axis.Recover"/> found and did. <see cref="Cause"/> is the link state it started from, <see cref="RestartSigns"/>
    /// why the unit is taken to have restarted, <see cref="Actions"/> what was sent or done, in order, <see cref="Notes"/> what the
    /// application should know or do (nothing in a note was done automatically). Nothing was moved.
    /// </summary>
    public sealed record RecoveryReport
    {
        /// <summary>The axis ("COM3:1").</summary>
        public string? Axis { get; init; }

        /// <summary>The link state the recovery started from.</summary>
        public string Cause { get; init; } = LinkStates.Ok;

        /// <summary>The unit restarted (power cycle).</summary>
        public bool Restarted { get; init; }

        /// <summary>Why: the restart signs, in words.</summary>
        public IReadOnlyList<string> RestartSigns { get; init; } = Array.Empty<string>();

        /// <summary>The port was reopened.</summary>
        public bool PortReopened { get; init; }

        /// <summary>The rate the unit was found at; <c>null</c> when not searched.</summary>
        public int? BaudRateFound { get; init; }

        /// <summary>The rate in use afterwards.</summary>
        public int? BaudRate { get; init; }

        /// <summary>The unit was switched back to the application's rate.</summary>
        public bool BaudRestored { get; init; }

        /// <summary>The unit is the one that was tracked (serial number, model).</summary>
        public bool SameUnit { get; init; }

        /// <summary>PC control (PMSL) was taken again.</summary>
        public bool ControlRestored { get; init; }

        /// <summary>The application held PC control before the fault and the unit still did (a line break does not drop it): nothing was written.</summary>
        public bool ControlKept { get; init; }

        /// <summary>The safety-speed mode was put back.</summary>
        public bool SafetySpeedRestored { get; init; }

        /// <summary>The unit's ranges that differ from the session's.</summary>
        public IReadOnlyList<string> LimitsChanged { get; init; } = Array.Empty<string>();

        /// <summary>ALMC afterwards, 0 = none.</summary>
        public int AlarmCode { get; init; }

        /// <summary>The alarm's name.</summary>
        public string? AlarmName { get; init; }

        /// <summary>The alarm's level ("message", "operation", "cold_start"...).</summary>
        public string? AlarmLevel { get; init; }

        /// <summary>Message alarms the recovery reset.</summary>
        public IReadOnlyList<int> AlarmsCleared { get; init; } = Array.Empty<int>();

        /// <summary>PC control (PMSS) afterwards.</summary>
        public bool? ModbusControl { get; init; }

        /// <summary>Safety speed afterwards.</summary>
        public bool? SafetySpeed { get; init; }

        /// <summary>Servo on afterwards.</summary>
        public bool? ServoOn { get; init; }

        /// <summary>The servo was on when the session was taken.</summary>
        public bool? ServoWasOn { get; init; }

        /// <summary>HEND afterwards.</summary>
        public bool? Homed { get; init; }

        /// <summary>HEND was lost: home before the next move.</summary>
        public bool NeedsHome { get; init; }

        /// <summary>The position read afterwards.</summary>
        public double? PositionMm { get; init; }

        /// <summary>The last commanded target the session knew.</summary>
        public double? LastTargetMm { get; init; }

        /// <summary>Position minus last target.</summary>
        public double? PositionErrorMm { get; init; }

        /// <summary>The axis stands within the tolerance of its last target; <c>null</c> when none was known.</summary>
        public bool? AtTarget { get; init; }

        /// <summary>The stored rows compared.</summary>
        public IReadOnlyList<int> RowsChecked { get; init; } = Array.Empty<int>();

        /// <summary>Rows that differ from the session's copy or the backup, or whose checksum is wrong.</summary>
        public IReadOnlyList<string> RowProblems { get; init; } = Array.Empty<string>();

        /// <summary>Parameter items compared.</summary>
        public int ParametersChecked { get; init; }

        /// <summary>Parameters that differ from the session's copy or the backup.</summary>
        public IReadOnlyList<string> ParameterDifferences { get; init; } = Array.Empty<string>();

        /// <summary>What was sent or done, in order.</summary>
        public IReadOnlyList<string> Actions { get; init; } = Array.Empty<string>();

        /// <summary>What the application should know or do.</summary>
        public IReadOnlyList<string> Notes { get; init; } = Array.Empty<string>();

        /// <summary>How long the recovery took.</summary>
        public double ElapsedS { get; init; }

        /// <summary>The last status read; <c>null</c> when none.</summary>
        public Status? Status { get; init; }

        /// <summary>An alarm that blocks motion is still active.</summary>
        public bool FaultAlarm { get; init; }

        /// <summary>
        /// Anything to decide before the next move: home lost, a fault alarm, a move cut off (not at its target) or still running,
        /// changed ranges, row or parameter differences, or the servo off where it was on.
        /// </summary>
        public bool NeedsAttention { get; init; }

        /// <summary>The status allows a move now: servo ON, homed, PC control, no alarm, no E-stop.</summary>
        public bool ReadyToMove { get; init; }

        /// <summary>"G: recovered from unit_silent, unit restarted, needs home".</summary>
        public string Summary { get; init; } = string.Empty;

        /// <summary>The whole report as a JSON-friendly dictionary (the Python edition's keys).</summary>
        public IDictionary<string, object?> ToDictionary() => Details ?? new Dictionary<string, object?>();

        /// <summary>The whole report as JSON text (indented by two spaces).</summary>
        public string ToJson() => JsonPlain.Write(ToDictionary());

        /// <inheritdoc />
        public override string ToString() => Summary;

        private IDictionary<string, object?>? Details { get; init; }

        internal static RecoveryReport From(NativeRecoverySummary s) => From(s.Json, s);

        internal static RecoveryReport? FromJson(string? json) => json is null || json.Length == 0 ? null : From(json, null);

        internal static RecoveryReport? FromDetails(IDictionary<string, object?>? report) =>
            report is null ? null : From(JsonPlain.Write(report, indented: false), null);

        private static RecoveryReport From(string json, NativeRecoverySummary? s)
        {
            var o = MiniJson.TryParse(json) as JsonObject ?? new JsonObject();
            var status = s?.Status?.ToRecord();
            return new RecoveryReport
            {
                Axis = o.String("axis") ?? (s?.Axis.Length > 0 ? s.Axis : null),
                Cause = o.String("cause") ?? s?.Cause ?? LinkStates.Ok,
                Restarted = o.Bool("restarted") ?? s?.Restarted ?? false,
                RestartSigns = o.Strings("restart_signs"),
                PortReopened = o.Bool("port_reopened") ?? s?.PortReopened ?? false,
                BaudRateFound = o.Int("baudrate_found") ?? s?.BaudRateFound,
                BaudRate = o.Int("baudrate") ?? s?.BaudRate,
                BaudRestored = o.Bool("baud_restored") ?? s?.BaudRestored ?? false,
                SameUnit = o.Bool("same_unit") ?? s?.SameUnit ?? false,
                ControlRestored = o.Bool("control_restored") ?? s?.ControlRestored ?? false,
                ControlKept = o.Bool("control_kept") ?? s?.ControlKept ?? false,
                SafetySpeedRestored = o.Bool("safety_speed_restored") ?? s?.SafetySpeedRestored ?? false,
                LimitsChanged = o.Strings("limits_changed"),
                AlarmCode = o.Int("alarm_code") ?? s?.AlarmCode ?? 0,
                AlarmName = o.String("alarm_name"),
                AlarmLevel = o.String("alarm_level"),
                AlarmsCleared = Ints(o, "alarms_cleared"),
                ModbusControl = o.Bool("modbus_control") ?? s?.ModbusControl,
                SafetySpeed = o.Bool("safety_speed") ?? s?.SafetySpeed,
                ServoOn = o.Bool("servo_on") ?? s?.ServoOn,
                ServoWasOn = o.Bool("servo_was_on"),
                Homed = o.Bool("homed") ?? s?.Homed,
                NeedsHome = o.Bool("needs_home") ?? s?.NeedsHome ?? false,
                PositionMm = o.Number("position_mm") ?? s?.PositionMm,
                LastTargetMm = o.Number("last_target_mm") ?? s?.LastTargetMm,
                PositionErrorMm = o.Number("position_error_mm") ?? s?.PositionErrorMm,
                AtTarget = o.Bool("at_target") ?? s?.AtTarget,
                RowsChecked = Ints(o, "rows_checked"),
                RowProblems = o.Strings("row_problems"),
                ParametersChecked = o.Int("parameters_checked") ?? 0,
                ParameterDifferences = o.Strings("parameter_differences"),
                Actions = o.Strings("actions"),
                Notes = o.Strings("notes"),
                ElapsedS = o.Number("elapsed_s") ?? s?.ElapsedS ?? 0.0,
                Status = status,
                FaultAlarm = o.Bool("fault_alarm") ?? s?.FaultAlarm ?? false,
                NeedsAttention = o.Bool("needs_attention") ?? s?.NeedsAttention ?? false,
                ReadyToMove = o.Bool("ready_to_move") ?? s?.ReadyToMove ?? false,
                Summary = o.String("summary") ?? s?.Summary ?? string.Empty,
                Details = JsonPlain.Dict(o),
            };
        }

        private static IReadOnlyList<int> Ints(JsonObject o, string key) =>
            (o.Array(key) ?? new List<object?>()).OfType<double>().Select(d => (int)d).ToList();
    }

    /// <summary>
    /// What an axis's session remembers (<see cref="Axis.Track"/>): the identity and the volatile state a link check and a recovery compare
    /// with.
    /// </summary>
    public sealed record RecoverySession
    {
        /// <summary>The session's name (the axis label: "COM3:1").</summary>
        public string? Name { get; init; }

        /// <summary>The unit's Modbus address.</summary>
        public int Slave { get; init; }

        /// <summary>The port's name.</summary>
        public string? Port { get; init; }

        /// <summary>The baud rate the link ran at.</summary>
        public int? BaudRate { get; init; }

        /// <summary>The unit's serial number.</summary>
        public string? SerialNumber { get; init; }

        /// <summary>The unit's model name.</summary>
        public string? ModelName { get; init; }

        /// <summary>FC11 system code.</summary>
        public int? SystemCode { get; init; }

        /// <summary>FC11 unit code.</summary>
        public int? UnitCode { get; init; }

        /// <summary>The firmware version.</summary>
        public string? Firmware { get; init; }

        /// <summary>PC control (PMSS) when the session was taken.</summary>
        public bool? ModbusControl { get; init; }

        /// <summary>Safety speed when the session was taken.</summary>
        public bool? SafetySpeed { get; init; }

        /// <summary>Servo on when the session was taken.</summary>
        public bool? ServoOn { get; init; }

        /// <summary>HEND when the session was taken.</summary>
        public bool? Homed { get; init; }

        /// <summary>The unit's uptime (ms) when the session was taken or last refreshed.</summary>
        public long? UptimeMs { get; init; }

        /// <summary>The alarm-history slots copied (up to 32); 0 when none.</summary>
        public int AlarmHistoryCount { get; init; }

        /// <summary>The last commanded target.</summary>
        public double? LastTargetMm { get; init; }

        /// <summary>The stored rows copied (row numbers); empty when none.</summary>
        public IReadOnlyList<int> Rows { get; init; } = Array.Empty<int>();

        /// <summary>The parameters were copied.</summary>
        public bool HasParameters { get; init; }

        /// <summary>A recovery is running.</summary>
        public bool Recovering { get; init; }

        /// <summary>The whole session as a JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => Details ?? new Dictionary<string, object?>();

        private IDictionary<string, object?>? Details { get; init; }

        internal static RecoverySession From(string json)
        {
            var o = MiniJson.TryParse(json) as JsonObject ?? new JsonObject();
            return new RecoverySession
            {
                Name = o.String("name"),
                Slave = o.Int("slave") ?? 0,
                Port = o.String("port"),
                BaudRate = o.Int("baudrate"),
                SerialNumber = o.String("serial_number"),
                ModelName = o.String("model_name"),
                SystemCode = o.Int("system_code"),
                UnitCode = o.Int("unit_code"),
                Firmware = o.String("firmware"),
                ModbusControl = o.Bool("modbus_control"),
                SafetySpeed = o.Bool("safety_speed"),
                ServoOn = o.Bool("servo_on"),
                Homed = o.Bool("homed"),
                UptimeMs = o.Long("uptime_ms"),
                AlarmHistoryCount = o.Array("alarm_history")?.Count ?? 0,
                LastTargetMm = o.Number("last_target_mm"),
                Rows = (o.Object("rows")?.Keys ?? Array.Empty<string>())
                    .Select(k => int.TryParse(k, NumberStyles.Integer, CultureInfo.InvariantCulture, out var n) ? n : -1)
                    .Where(n => n >= 0)
                    .ToList(),
                HasParameters = o.Object("parameters") != null,
                Recovering = o.Bool("recovering") ?? false,
                Details = JsonPlain.Dict(o),
            };
        }
    }
}
