using System;
using System.Collections.Generic;
using System.Globalization;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // Read-only views of the C ABI's output structs and JSON records. Units: positions in mm (deg on a rotary axis),
    // speeds in units/s, accelerations in G, times in seconds. A value the library does not know (NaN, IAD_INT_UNKNOWN,
    // IAD_UNKNOWN) is null. Property names follow the managed edition where it has the same field.

    /// <summary>Conversions of the ABI's "unknown" values.</summary>
    internal static class Known
    {
        internal static double? Double(double value) => double.IsNaN(value) ? (double?)null : value;

        internal static int? Int(int value) => value == int.MinValue ? (int?)null : value;

        internal static bool? Tri(int value) => value == NativeMethods.TriUnknown ? (bool?)null : value != 0;

        internal static bool Bool(int value) => value != 0;

        internal static int Flag(bool value) => value ? 1 : 0;

        /// <summary>The input form of an iad_tribool: null is IAD_UNKNOWN (2), false 0, true 1.</summary>
        internal static int Tri(bool? value) => value is null ? NativeMethods.TriUnknown : (value.Value ? 1 : 0);

        /// <summary>The input form of an optional double: NaN means "not set" (the default).</summary>
        internal static double OrNaN(double? value) => value ?? double.NaN;

        /// <summary>
        /// A speed, acceleration or band for a struct without an _init function (iad_axis_move, iad_sequence_point):
        /// the core reads a 0 there as "the default" (0.8), so an explicit 0 is refused here, as the managed API always
        /// did. null is NaN (the default).
        /// </summary>
        internal static double Slot(string name, double? value) =>
            value is { } v && v == 0.0 ? throw new SoftLimitException($"{name} must be > 0, got 0") : OrNaN(value);

        internal static AlarmLevel Level(int value)
        {
            switch (value)
            {
                case 0: return AlarmLevel.None;
                case 1: return AlarmLevel.Message;
                case 2: return AlarmLevel.OperationCancelled;
                case 3: return AlarmLevel.ColdStart;
                default: return AlarmLevel.Unknown;
            }
        }

        internal static AlarmLevel Level(string? key)
        {
            switch (key)
            {
                case "none": return AlarmLevel.None;
                case "message": return AlarmLevel.Message;
                case "operation_cancelled": return AlarmLevel.OperationCancelled;
                case "cold_start": return AlarmLevel.ColdStart;
                default: return AlarmLevel.Unknown;
            }
        }
    }

    // ------------------------------------------------------------------------------------------ status

    /// <summary>One decoded status sample (iad_status; C++ Status).</summary>
    internal sealed class NativeStatus
    {
        internal NativeStatus(in IadStatus s)
        {
            PositionMm = s.PositionMm;
            VelocityMmS = Known.Double(s.VelocityMmS);
            CurrentMa = Known.Double(s.CurrentMa);
            UptimeS = Known.Double(s.UptimeS);
            AlarmCode = s.AlarmCode;
            AlarmLevel = s.AlarmCode != 0 ? Known.Level(s.AlarmLevel) : (AlarmLevel?)null;
            ServoOn = Known.Bool(s.ServoOn);
            Homed = Known.Bool(s.Homed);
            InPosition = Known.Bool(s.InPosition);
            Moving = Known.Bool(s.Moving);
            Alarm = Known.Bool(s.Alarm);
            Warning = Known.Bool(s.Warning);
            ModbusEnabled = Known.Bool(s.ModbusEnabled);
            EmergencyStop = Known.Bool(s.EmergencyStop);
            Paused = Known.Bool(s.Paused);
            Homing = Known.Bool(s.Homing);
            SafetySpeed = Known.Bool(s.SafetySpeed);
            Ready = Known.Bool(s.Ready);
            Idle = Known.Bool(s.Idle);
            TargetReached = Known.Bool(s.TargetReached);
            PushMissed = Known.Bool(s.PushMissed);
            Pushing = Known.Bool(s.Pushing);
            EmgInput = Known.Tri(s.EmgInput);
            DrivePowerCut = Known.Tri(s.DrivePowerCut);
            EnableOff = Known.Tri(s.EnableOff);
            RawWords = s.RawWords;
            RawDss1 = s.RawDss1;
            RawDss2 = s.RawDss2;
            RawDsse = s.RawDsse;
            RawDipm = s.RawDipm;
            RawDopm = s.RawDopm;
            Rcon = Known.Bool(s.Rcon);
        }

        /// <summary>PNOW (mm, or deg on a rotary axis), 0.01 resolution.</summary>
        public double PositionMm { get; }

        /// <summary>0 when not moving; while moving from PNOW samples; null when not known.</summary>
        public double? VelocityMmS { get; }

        /// <summary>CNOW (signed, mA); null with fewer than 14 words read.</summary>
        public double? CurrentMa { get; }

        /// <summary>STIM / 1000 (s); null when not read.</summary>
        public double? UptimeS { get; }

        /// <summary>ALMC, 0 = none.</summary>
        public int AlarmCode { get; }

        /// <summary>Level of <see cref="AlarmCode"/>; null without an alarm.</summary>
        public AlarmLevel? AlarmLevel { get; }

        /// <summary>DSS1 SV.</summary>
        public bool ServoOn { get; }

        /// <summary>DSS1 HEND.</summary>
        public bool Homed { get; }

        /// <summary>DSS1 PEND.</summary>
        public bool InPosition { get; }

        /// <summary>In motion: DSSE MOVE, a home return in progress, or servo on without PEND while homed.</summary>
        public bool Moving { get; }

        /// <summary>A fault that blocks motion.</summary>
        public bool Alarm { get; }

        /// <summary>Same as <see cref="Alarm"/> (the managed edition's name).</summary>
        public bool Fault => Alarm;

        /// <summary>A message-level alarm (or ALML) and no fault.</summary>
        public bool Warning { get; }

        /// <summary>DSSE PMSS: Modbus has control.</summary>
        public bool ModbusEnabled { get; }

        /// <summary>DSS1 EMGS.</summary>
        public bool EmergencyStop { get; }

        /// <summary>DSS1 STP.</summary>
        public bool Paused { get; }

        /// <summary>DSSE GHMS.</summary>
        public bool Homing { get; }

        /// <summary>DSS1 SFTY.</summary>
        public bool SafetySpeed { get; }

        /// <summary>Servo on, homed, Modbus control, no fault / EMG / pause / homing.</summary>
        public bool Ready { get; }

        /// <summary>Not moving, not paused, not homing.</summary>
        public bool Idle { get; }

        /// <summary>DSS1 MEND.</summary>
        public bool TargetReached { get; }

        /// <summary>DSS1 PSFL.</summary>
        public bool PushMissed { get; }

        /// <summary>DSSE b10.</summary>
        public bool Pushing { get; }

        /// <summary>DSSE EMGP (DOC, UNVERIFIED on an EC); null when not known.</summary>
        public bool? EmgInput { get; }

        /// <summary>STAT MPOW = 0 or DSSE MPUV; null when not known.</summary>
        public bool? DrivePowerCut { get; }

        /// <summary>DSS2 ENBS = 0; null when not known.</summary>
        public bool? EnableOff { get; }

        /// <summary>Words decoded (8, 14, 20 or 22).</summary>
        public int RawWords { get; }

        public uint RawDss1 { get; }

        public uint RawDss2 { get; }

        public uint RawDsse { get; }

        public uint RawDipm { get; }

        public uint RawDopm { get; }

        /// <summary>(0.2) Read behind an RCON gateway: the bits RCON leaves undefined are not decoded.</summary>
        public bool Rcon { get; }

        /// <inheritdoc />
        public override string ToString() => string.Format(
            CultureInfo.InvariantCulture,
            "pos={0:F2} servo={1} homed={2} moving={3} alarm=0x{4:x3}",
            PositionMm,
            ServoOn ? "on" : "off",
            Homed ? "yes" : "no",
            Moving ? "yes" : "no",
            AlarmCode);
    }

    // ------------------------------------------------------------------------------------------ limits, identity

    /// <summary>One range the driver enforces (iad_range; C++ LimitRange). Sources in words: <see cref="NativeAxis.LimitsJson"/>.</summary>
    internal sealed class NativeRange
    {
        internal unsafe NativeRange(IadRange r)
        {
            Min = Known.Double(r.Min);
            Max = Known.Double(r.Max);
            UnitMax = Known.Double(r.UnitMax);
            Configured = Known.Double(r.Configured);
            RecommendedMin = Known.Double(r.RecommendedMin);
            Available = Known.Tri(r.Available);
            Count = Known.Int(r.Count);
            Unit = Utf8.FromFixed(r.Unit, 8);
        }

        public double? Min { get; }

        public double? Max { get; }

        /// <summary>The unit's own maximum.</summary>
        public double? UnitMax { get; }

        /// <summary>The cap configured on the axis.</summary>
        public double? Configured { get; }

        /// <summary>Push: MNPP; speed: the documented minimum speed.</summary>
        public double? RecommendedMin { get; }

        /// <summary>Push: the unit can push.</summary>
        public bool? Available { get; }

        /// <summary>Stored rows: how many.</summary>
        public int? Count { get; }

        /// <summary>"mm", "mm/s", "G", "%"...</summary>
        public string Unit { get; }

        /// <inheritdoc />
        public override string ToString() =>
            string.Format(CultureInfo.InvariantCulture, "{0} .. {1} {2}", Min?.ToString("G", CultureInfo.InvariantCulture) ?? "?", Max?.ToString("G", CultureInfo.InvariantCulture) ?? "?", Unit).TrimEnd();
    }

    /// <summary>Every range the driver enforces on one axis (iad_limits; C++ AxisRanges, Axis::limits()).</summary>
    internal sealed class NativeLimits
    {
        internal unsafe NativeLimits(IadLimits l)
        {
            Position = new NativeRange(l.Position);
            Speed = new NativeRange(l.Speed);
            Accel = new NativeRange(l.Accel);
            Decel = new NativeRange(l.Decel);
            AccelDecel = new NativeRange(l.AccelDecel);
            Push = new NativeRange(l.Push);
            Band = new NativeRange(l.Band);
            PushBand = new NativeRange(l.PushBand);
            StoredRows = new NativeRange(l.StoredRows);
            JogStep = new NativeRange(l.JogStep);
            SpeedOverride = new NativeRange(l.SpeedOverride);
            Units = Utf8.FromFixed(l.Units, 8);
            Kind = Utf8.FromFixed(l.Kind, 24);
            Family = Utf8.FromFixed(l.Family, 24);
            PushSpeed = Known.Double(l.PushSpeed);
        }

        public NativeRange Position { get; }

        public NativeRange Speed { get; }

        public NativeRange Accel { get; }

        public NativeRange Decel { get; }

        /// <summary>One value for both on a 0x9900 move: min(MAXA, MAXD).</summary>
        public NativeRange AccelDecel { get; }

        public NativeRange Push { get; }

        public NativeRange Band { get; }

        public NativeRange PushBand { get; }

        public NativeRange StoredRows { get; }

        public NativeRange JogStep { get; }

        /// <summary>1..100 % (0x45 cmd 6).</summary>
        public NativeRange SpeedOverride { get; }

        /// <summary>"mm" or "deg".</summary>
        public string Units { get; }

        /// <summary>"slider", "rod", "gripper", "rotary"... "unknown".</summary>
        public string Kind { get; }

        /// <summary>"EC", "PCON", ...</summary>
        public string Family { get; }

        /// <summary>
        /// (0.2) The push speed, min(PSHV 8.15 / CON No.34, MXPH 7.8) (units/s); null when not readable (its source is in
        /// <see cref="NativeAxis.LimitsJson"/>).
        /// </summary>
        public double? PushSpeed { get; }
    }

    /// <summary>Identity of one unit (iad_identity; C++ DeviceInfo). The full record: <see cref="NativeAxis.UnitInfoJson"/>.</summary>
    internal sealed class NativeIdentity
    {
        internal unsafe NativeIdentity(IadIdentity d)
        {
            SoftMinMm = Known.Double(d.SoftMinMm);
            SoftMaxMm = Known.Double(d.SoftMaxMm);
            MaxSpeedMmS = Known.Double(d.MaxSpeedMmS);
            MaxAccelG = Known.Double(d.MaxAccelG);
            MaxDecelG = Known.Double(d.MaxDecelG);
            Slave = d.Slave;
            SystemCode = Known.Int(d.SystemCode);
            UnitCode = Known.Int(d.UnitCode);
            RunStatus = Known.Int(d.RunStatus);
            AxisNumber = Known.Int(d.AxisNumber);
            MaxPositions = Known.Int(d.MaxPositions);
            ActuatorKind = Known.Int(d.ActuatorKind);
            UnitKnown = Known.Tri(d.UnitKnown);
            Rotary = Known.Tri(d.Rotary);
            ProductFamily = Utf8.FromFixed(d.ProductFamily, 16);
            Model = Utf8.FromFixed(d.Model, 64);
            ModelName = Utf8.FromFixed(d.ModelName, 64);
            ControllerName = Utf8.FromFixed(d.ControllerName, 32);
            Controller = Utf8.FromFixed(d.Controller, 64);
            UnitName = Utf8.FromFixed(d.UnitName, 48);
            SystemName = Utf8.FromFixed(d.SystemName, 48);
            Firmware = Utf8.FromFixed(d.Firmware, 8);
            SerialNumber = Utf8.FromFixed(d.SerialNumber, 32);
            ActuatorSerial = Utf8.FromFixed(d.ActuatorSerial, 32);
            UserName = Utf8.FromFixed(d.UserName, 48);
            Units = Utf8.FromFixed(d.Units, 8);
            Kind = Utf8.FromFixed(d.Kind, 24);
            IdentitySource = Utf8.FromFixed(d.IdentitySource, 16);
        }

        /// <summary>Soft limit 7.1 as stored (with the 0.30 margin).</summary>
        public double? SoftMinMm { get; }

        /// <summary>Soft limit 7.2 as stored (with the 0.30 margin).</summary>
        public double? SoftMaxMm { get; }

        /// <summary>MAXV 7.4 (or the default speed 8.10).</summary>
        public double? MaxSpeedMmS { get; }

        public double? MaxAccelG { get; }

        public double? MaxDecelG { get; }

        public int Slave { get; }

        /// <summary>FC11 byte 0 (0xEC EC, 0xAE PCON...).</summary>
        public int? SystemCode { get; }

        /// <summary>FC11 byte 2.</summary>
        public int? UnitCode { get; }

        /// <summary>FC11 byte 1.</summary>
        public int? RunStatus { get; }

        /// <summary>AXNO (= slave - 1).</summary>
        public int? AxisNumber { get; }

        /// <summary>Stored rows.</summary>
        public int? MaxPositions { get; }

        /// <summary>AKID 7.33.</summary>
        public int? ActuatorKind { get; }

        /// <summary>The driver has a family name for this unit code (display only, never a gate).</summary>
        public bool? UnitKnown { get; }

        public bool? Rotary { get; }

        /// <summary>"EC", "PCON", ...</summary>
        public string ProductFamily { get; }

        /// <summary>The model string, else the controller name, else the controller description.</summary>
        public string Model { get; }

        /// <summary>e.g. "EC-R6S-150-1-PN" ("" when the unit keeps none).</summary>
        public string ModelName { get; }

        /// <summary>"PCON-CB" on a CON controller.</summary>
        public string ControllerName { get; }

        /// <summary>"EC 24 V unit 0x02 fw 0013"-style description.</summary>
        public string Controller { get; }

        public string UnitName { get; }

        public string SystemName { get; }

        /// <summary>4 hex digits.</summary>
        public string Firmware { get; }

        public string SerialNumber { get; }

        public string ActuatorSerial { get; }

        /// <summary>The user-editable unit name.</summary>
        public string UserName { get; }

        /// <summary>"mm" or "deg".</summary>
        public string Units { get; }

        /// <summary>"slider", "rod", "gripper", "rotary"...</summary>
        public string Kind { get; }

        /// <summary>"FC11" or "CTCL 0x0300".</summary>
        public string IdentitySource { get; }

        /// <inheritdoc />
        public override string ToString() => $"{Model} ({ProductFamily}, slave {Slave})";
    }

    // ------------------------------------------------------------------------------------------ position table

    /// <summary>
    /// One stored row of the position table, in the axis's units (iad_position_entry; C++ PositionEntry). Input of
    /// <see cref="NativeAxis.WritePositionTable"/> and output of <see cref="NativeAxis.ReadPositionTable"/>. A new entry
    /// has the C++ defaults: band 0.10, speed 50, accel / decel 0.30, the rest 0.
    /// </summary>
    internal sealed class NativePositionEntry
    {
        public NativePositionEntry()
        {
        }

        /// <summary>(0.8) The row read back of a write result (its own fields, those of iad_position_entry).</summary>
        internal NativePositionEntry(in IadPositionWriteResult e)
        {
            PositionMm = e.PositionMm;
            BandMm = e.BandMm;
            SpeedMmS = e.SpeedMmS;
            ZonePlusMm = e.ZonePlusMm;
            ZoneMinusMm = e.ZoneMinusMm;
            AccelG = e.AccelG;
            DecelG = e.DecelG;
            PushPower = e.PushPower;
            LoadThreshold = e.LoadThreshold;
            ControlFlags = e.ControlFlags;
            Checksum = Known.Int(e.Checksum);
            ChecksumOk = Known.Tri(e.ChecksumOk);
            IsPush = Known.Bool(e.IsPush);
            Incremental = Known.Bool(e.Incremental);
        }

        internal NativePositionEntry(in IadPositionEntry e)
        {
            PositionMm = e.PositionMm;
            BandMm = e.BandMm;
            SpeedMmS = e.SpeedMmS;
            ZonePlusMm = e.ZonePlusMm;
            ZoneMinusMm = e.ZoneMinusMm;
            AccelG = e.AccelG;
            DecelG = e.DecelG;
            PushPower = e.PushPower;
            LoadThreshold = e.LoadThreshold;
            ControlFlags = e.ControlFlags;
            Checksum = Known.Int(e.Checksum);
            ChecksumOk = Known.Tri(e.ChecksumOk);
            IsPush = Known.Bool(e.IsPush);
            Incremental = Known.Bool(e.Incremental);
        }

        /// <summary>PCMD.</summary>
        public double PositionMm { get; set; }

        /// <summary>INP (push rows: the push band).</summary>
        public double BandMm { get; set; } = 0.10;

        /// <summary>VCMD.</summary>
        public double SpeedMmS { get; set; } = 50.0;

        /// <summary>ZNMP.</summary>
        public double ZonePlusMm { get; set; }

        /// <summary>ZNLP.</summary>
        public double ZoneMinusMm { get; set; }

        /// <summary>ACMD.</summary>
        public double AccelG { get; set; } = 0.30;

        /// <summary>DCMD.</summary>
        public double DecelG { get; set; } = 0.30;

        /// <summary>PPOW counts (255 = 100 %).</summary>
        public int PushPower { get; set; }

        /// <summary>LPOW.</summary>
        public int LoadThreshold { get; set; }

        /// <summary>CTLF (push / direction / role bits).</summary>
        public int ControlFlags { get; set; }

        /// <summary>+0xF as read; null for an entry built in code (ignored in inputs).</summary>
        public int? Checksum { get; private set; }

        /// <summary>The stored checksum matches the data (output).</summary>
        public bool? ChecksumOk { get; private set; }

        /// <summary>Output.</summary>
        public bool IsPush { get; private set; }

        /// <summary>Output.</summary>
        public bool Incremental { get; private set; }

        internal IadPositionEntry ToNative() => new IadPositionEntry
        {
            StructSize = SizedStruct.Size<IadPositionEntry>(),
            PositionMm = PositionMm,
            BandMm = BandMm,
            SpeedMmS = SpeedMmS,
            ZonePlusMm = ZonePlusMm,
            ZoneMinusMm = ZoneMinusMm,
            AccelG = AccelG,
            DecelG = DecelG,
            PushPower = PushPower,
            LoadThreshold = LoadThreshold,
            ControlFlags = ControlFlags,
            Checksum = int.MinValue,
            ChecksumOk = NativeMethods.TriUnknown,
        };

        /// <inheritdoc />
        public override string ToString() => string.Format(
            CultureInfo.InvariantCulture,
            "pos={0:F2} speed={1:F2} band={2:F2} accel={3:F2} decel={4:F2}",
            PositionMm,
            SpeedMmS,
            BandMm,
            AccelG,
            DecelG);
    }

    /// <summary>Outcome of a position-table write or a teach (iad_position_write_result; C++ PositionWriteResult).</summary>
    internal sealed class NativePositionWriteResult
    {
        internal NativePositionWriteResult(in IadPositionWriteResult r, string json)
        {
            Entry = new NativePositionEntry(r);
            Row = r.Row;
            Skipped = Known.Bool(r.Skipped);
            Forced = Known.Bool(r.Forced);
            WarningCount = r.WarningCount;
            Json = json;
            var details = MiniJson.TryParse(json) as JsonObject;
            Warnings = details?.Strings("warnings") ?? Array.Empty<string>();
            Note = details?.String("note") ?? string.Empty;
        }

        /// <summary>Read back after the write; when skipped, what the unit holds.</summary>
        public NativePositionEntry Entry { get; }

        public int Row { get; }

        /// <summary>Non-volatile guard: the row already held these 16 words, nothing written.</summary>
        public bool Skipped { get; }

        /// <summary>Unchanged but written anyway (force).</summary>
        public bool Forced { get; }

        public int WarningCount { get; }

        /// <summary>The warnings (from <see cref="Json"/>).</summary>
        public IReadOnlyList<string> Warnings { get; }

        /// <summary>The note (from <see cref="Json"/>).</summary>
        public string Note { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the write; "" when not available).</summary>
        public string Json { get; }
    }

    // ------------------------------------------------------------------------------------------ parameters

    /// <summary>
    /// One parameter, decoded (C++ ParameterInfo). From <see cref="NativeAxis.ReadParameter(string)"/> (iad_parameter:
    /// text fields cut at their sizes, no choices) or <see cref="NativeAxis.Parameters"/> (the JSON record: full text and
    /// the choices).
    /// </summary>
    internal sealed class NativeParameter
    {
        internal unsafe NativeParameter(IadParameter p)
        {
            Number = p.Number != 0 ? p.Number : (int?)null;
            Value = Known.Double(p.Value);
            Min = Known.Double(p.Min);
            Max = Known.Double(p.Max);
            Writable = Known.Bool(p.Writable);
            Dangerous = Known.Bool(p.Dangerous);
            Name = Utf8.FromFixed(p.Name, 64);
            Symbol = Utf8.FromFixed(p.Symbol, 16);
            Item = Utf8.FromFixed(p.Item, 16);
            Address = Utf8.FromFixed(p.Address, 16);
            Text = Utf8.FromFixed(p.Text, 64);
            Unit = Utf8.FromFixed(p.Unit, 16);
            Raw = Utf8.FromFixed(p.Raw, 16);
            Family = Utf8.FromFixed(p.Family, 8);
            Choices = new Dictionary<string, string>();
            Note = string.Empty;
        }

        internal NativeParameter(JsonObject p)
        {
            Number = p.Int("number");
            Value = p.Number("value");
            Min = p.Number("min");
            Max = p.Number("max");
            Writable = p.Bool("writable") ?? false;
            Dangerous = p.Bool("dangerous") ?? false;
            Name = p.String("name") ?? string.Empty;
            Symbol = p.String("symbol") ?? string.Empty;
            Item = p.String("item") ?? string.Empty;
            Address = p.String("address") ?? string.Empty;
            Text = p.String("text") ?? (p.Get("value") as string) ?? string.Empty;
            Unit = p.String("unit") ?? string.Empty;
            Raw = p.String("raw") ?? string.Empty;
            Family = p.String("family") ?? string.Empty;
            Note = p.String("note") ?? string.Empty;
            var choices = new Dictionary<string, string>(StringComparer.Ordinal);
            if (p.Object("choices") is { } c)
            {
                foreach (var key in c.Keys)
                {
                    choices[key] = c.String(key) ?? string.Empty;
                }
            }

            Choices = choices;
        }

        /// <summary>P-number / No.; null for a raw item.</summary>
        public int? Number { get; }

        /// <summary>The number; null when the value is only a label.</summary>
        public double? Value { get; }

        public double? Min { get; }

        public double? Max { get; }

        public bool Writable { get; }

        public bool Dangerous { get; }

        public string Name { get; }

        public string Symbol { get; }

        /// <summary>"class.item", e.g. "8.15".</summary>
        public string Item { get; }

        /// <summary>"0x0810".</summary>
        public string Address { get; }

        /// <summary>As the PC tool shows it: "1:Forward", "disable", "12.50".</summary>
        public string Text { get; }

        public string Unit { get; }

        /// <summary>Hex.</summary>
        public string Raw { get; }

        /// <summary>"EC" or "CON".</summary>
        public string Family { get; }

        /// <summary>Value labels (from the JSON record only).</summary>
        public IReadOnlyDictionary<string, string> Choices { get; }

        /// <summary>From the JSON record only.</summary>
        public string Note { get; }

        /// <summary>The key to read or write this parameter by: its number, else its item.</summary>
        public string Key => Number?.ToString(CultureInfo.InvariantCulture) ?? Item;

        /// <inheritdoc />
        public override string ToString() => $"{Key} {Name} = {Text} {Unit}".TrimEnd();
    }

    /// <summary>Outcome of a parameter write (iad_param_write_result; C++ ParameterWriteResult).</summary>
    internal sealed class NativeParameterWriteResult
    {
        internal unsafe NativeParameterWriteResult(IadParamWriteResult r, string json)
        {
            Number = r.Number != 0 ? r.Number : (int?)null;
            Changed = Known.Bool(r.Changed);
            Verified = Known.Bool(r.Verified);
            Committed = Known.Bool(r.Committed);
            CommitConfirmed = Known.Bool(r.CommitConfirmed);
            Saved = Known.Bool(r.Saved);
            Skipped = Known.Bool(r.Skipped);
            Forced = Known.Bool(r.Forced);
            ListedForModel = Known.Bool(r.ListedForModel);
            VerifiedForModel = Known.Bool(r.VerifiedForModel);
            HomeParameter = Known.Bool(r.HomeParameter);
            WarningCount = r.WarningCount;
            Key = Utf8.FromFixed(r.Key, 16);
            Address = Utf8.FromFixed(r.Address, 16);
            Name = Utf8.FromFixed(r.Name, 64);
            Before = Utf8.FromFixed(r.Before, 64);
            After = Utf8.FromFixed(r.After, 64);
            Unit = Utf8.FromFixed(r.Unit, 16);
            Json = json;
            var details = MiniJson.TryParse(json) as JsonObject;
            Warnings = details?.Strings("warnings") ?? Array.Empty<string>();
            Note = details?.String("note") ?? string.Empty;
        }

        /// <summary>P-number / No.; null for a raw item.</summary>
        public int? Number { get; }

        /// <summary>The item's raw value changed.</summary>
        public bool Changed { get; }

        /// <summary>Read back equal to what was written (true when skipped: the unit holds it).</summary>
        public bool Verified { get; }

        /// <summary>PARW sent and the handshake ended idle.</summary>
        public bool Committed { get; }

        /// <summary>The unit was seen acting on PARW.</summary>
        public bool CommitConfirmed { get; }

        /// <summary>Committed and confirmed: in non-volatile memory as far as the unit showed.</summary>
        public bool Saved { get; }

        /// <summary>Non-volatile guard: unchanged, nothing written or committed.</summary>
        public bool Skipped { get; }

        /// <summary>Unchanged but written anyway (force).</summary>
        public bool Forced { get; }

        public bool ListedForModel { get; }

        /// <summary>Kept for compatibility: an IA-OS parameter list is recorded for the unit code.</summary>
        public bool VerifiedForModel { get; }

        public bool HomeParameter { get; }

        public int WarningCount { get; }

        /// <summary>"8.9".</summary>
        public string Key { get; }

        /// <summary>"0x0810".</summary>
        public string Address { get; }

        public string Name { get; }

        /// <summary>As the PC tool shows it.</summary>
        public string Before { get; }

        public string After { get; }

        public string Unit { get; }

        /// <summary>The warnings (from <see cref="Json"/>).</summary>
        public IReadOnlyList<string> Warnings { get; }

        /// <summary>The note (from <see cref="Json"/>).</summary>
        public string Note { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the write; "" when not available).</summary>
        public string Json { get; }
    }

    // ------------------------------------------------------------------------------------------ unit

    /// <summary>Outcome of a software reset (iad_reset_result; C++ SoftwareResetResult).</summary>
    internal sealed class NativeResetResult
    {
        internal unsafe NativeResetResult(IadResetResult r, string json)
        {
            Status = new NativeStatus(r.Status);
            UptimeBeforeS = Known.Double(r.UptimeBeforeS);
            UptimeAfterS = Known.Double(r.UptimeAfterS);
            ElapsedS = Known.Double(r.ElapsedS);
            Accepted = Known.Bool(r.Accepted);
            ReplyLost = Known.Bool(r.ReplyLost);
            Confirmed = Known.Bool(r.Confirmed);
            BaudRate = r.BaudRate > 0 ? r.BaudRate : (int?)null;
            BaudChanged = Known.Bool(r.BaudChanged);
            NeedsHome = Known.Bool(r.NeedsHome);
            ConfirmedBy = Utf8.FromFixed(r.ConfirmedBy, 64);
            Json = json;
            Notes = (MiniJson.TryParse(json) as JsonObject)?.Strings("notes") ?? Array.Empty<string>();
        }

        /// <summary>The first status after the reset.</summary>
        public NativeStatus Status { get; }

        public double? UptimeBeforeS { get; }

        public double? UptimeAfterS { get; }

        public double? ElapsedS { get; }

        /// <summary>Answered the reset with rejection 0.</summary>
        public bool Accepted { get; }

        /// <summary>No (complete) answer; the reset was confirmed from the status.</summary>
        public bool ReplyLost { get; }

        /// <summary>The restart was seen.</summary>
        public bool Confirmed { get; }

        /// <summary>The rate the unit answered at afterwards; null on a transport without one.</summary>
        public int? BaudRate { get; }

        public bool BaudChanged { get; }

        /// <summary>HEND is 0 after the reset: home (after an alarm reset if an alarm is active).</summary>
        public bool NeedsHome { get; }

        /// <summary>How the restart was confirmed, in words.</summary>
        public string ConfirmedBy { get; }

        /// <summary>The notes (from <see cref="Json"/>).</summary>
        public IReadOnlyList<string> Notes { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the reset; "" when not available).</summary>
        public string Json { get; }
    }

    /// <summary>Outcome of a baud change (iad_baud_result; C++ BaudChangeResult). Volatile: until the unit is powered off.</summary>
    internal sealed class NativeBaudResult
    {
        internal unsafe NativeBaudResult(IadBaudResult r, string json)
        {
            Previous = r.Previous;
            BaudRate = r.BaudRate;
            Method = r.Method == 1 ? BaudChangeMethod.Break : BaudChangeMethod.Direct;
            Code = r.Code;
            Verified = Known.Bool(r.Verified);
            Probed = Known.Bool(r.Probed);
            CodeVerified = Known.Bool(r.CodeVerified);
            Frame = Utf8.FromFixed(r.Frame, 64);
            Json = json;
            Note = (MiniJson.TryParse(json) as JsonObject)?.String("note") ?? string.Empty;
        }

        /// <summary>The rate before.</summary>
        public int Previous { get; }

        /// <summary>The rate the port is at now and the unit answered at.</summary>
        public int BaudRate { get; }

        public BaudChangeMethod Method { get; }

        /// <summary>IAI baud code sent.</summary>
        public int Code { get; }

        /// <summary>The first read at the new rate was answered.</summary>
        public bool Verified { get; }

        /// <summary>That read failed and the rate was found by probing instead.</summary>
        public bool Probed { get; }

        /// <summary>The rate's code is verified on hardware on at least some units.</summary>
        public bool CodeVerified { get; }

        /// <summary>The broadcast frame sent, hex.</summary>
        public string Frame { get; }

        /// <summary>The note (from <see cref="Json"/>).</summary>
        public string Note { get; }

        /// <summary>The full record (iad_axis_last_result_json right after the change; "" when not available).</summary>
        public string Json { get; }
    }

    // ------------------------------------------------------------------------------------------ alarms

    /// <summary>One alarm history slot (C++ AlarmRecord, from iad_axis_read_alarm_history_json).</summary>
    internal sealed class NativeAlarmRecord
    {
        internal NativeAlarmRecord(JsonObject r)
        {
            Index = r.Int("index") ?? 0;
            Code = r.Int("code") ?? 0;
            Name = r.String("name") ?? string.Empty;
            Level = Known.Level(r.String("level"));
            Detail = r.Int("detail") ?? 0;
            Address = r.Int("address") ?? 0;
            TimeMs = r.Long("time_ms");
            PowerUp = r.Bool("power_up") ?? false;
            TimeAfterPowerOn = r.String("time_after_power_on");
        }

        /// <summary>Slot index (0 = newest).</summary>
        public int Index { get; }

        public int Code { get; }

        public string Name { get; }

        public AlarmLevel Level { get; }

        /// <summary>0000 = none.</summary>
        public int Detail { get; }

        /// <summary>FFFF = none.</summary>
        public int Address { get; }

        /// <summary>ms after power-on; null when not stamped.</summary>
        public long? TimeMs { get; }

        /// <summary>The 0x0FFF "PowerUP No Error" marker the unit writes at every power-up.</summary>
        public bool PowerUp { get; }

        /// <summary>"[d d ]hh:mm:ss.mmm" after power-on; null without a time stamp.</summary>
        public string? TimeAfterPowerOn { get; }

        /// <inheritdoc />
        public override string ToString() => string.Format(CultureInfo.InvariantCulture, "#{0} 0x{1:x3} {2}", Index, Code, Name);
    }
}
