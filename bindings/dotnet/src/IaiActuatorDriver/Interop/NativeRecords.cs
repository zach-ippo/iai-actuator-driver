using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The core's outputs as the public records of this library (stage 1 of the switch: the records stay the API, the
    /// core fills them). Pure: no library call. Where the C ABI gives less than the managed decoder read, the record says
    /// so: a <see cref="Status"/> from the core carries the raw words the core reports (ALMC, DIPM, DOPM, DSS1, DSS2,
    /// DSSE and PNOW from the position); STAT, VNOW, CNOW, DEVI, STIM, SIPM and ZONS (the position markers) need the raw
    /// status words of the ABI (not in 0.3), so <see cref="Status.AtBackwardEnd"/> and its siblings are <c>null</c>.
    /// </summary>
    internal static class NativeRecords
    {
        /// <summary>A core status as the public <see cref="Status"/>.</summary>
        internal static Status ToRecord(this NativeStatus s)
        {
            if (s is null)
            {
                throw new ArgumentNullException(nameof(s));
            }

            var raw = new Dictionary<string, uint>
            {
                ["PNOW"] = unchecked((uint)(int)Math.Round(s.PositionMm / Registers.POSITION_SCALE)),
                ["ALMC"] = (uint)s.AlarmCode,
                ["DIPM"] = s.RawDipm,
                ["DOPM"] = s.RawDopm,
                ["DSS1"] = s.RawDss1,
                ["DSS2"] = s.RawDss2,
                ["DSSE"] = s.RawDsse,
            };
            return new Status
            {
                PositionMm = s.PositionMm,
                AlarmCode = s.AlarmCode,
                ServoOn = s.ServoOn,
                Homed = s.Homed,
                InPosition = s.InPosition,
                Moving = s.Moving,
                Alarm = s.Alarm,
                ModbusEnabled = s.ModbusEnabled,
                EmergencyStop = s.EmergencyStop,
                VelocityMmS = s.VelocityMmS,
                CurrentMa = s.CurrentMa.HasValue ? (int)Math.Round(s.CurrentMa.Value) : (int?)null,
                Raw = new ReadOnlyDictionary<string, uint>(raw),
            };
        }

        /// <summary>One core range as the public <see cref="LimitRange"/> (<paramref name="source"/>: from iad_axis_limits_json).</summary>
        internal static LimitRange ToRecord(this NativeRange r, string source = "") => new LimitRange
        {
            Min = r.Min,
            Max = r.Max,
            Unit = r.Unit,
            Source = source,
            UnitMax = r.UnitMax,
            Configured = r.Configured,
            Available = r.Available,
            RecommendedMin = r.RecommendedMin,
            Count = r.Count,
        };

        /// <summary>
        /// The core's limits as the public <see cref="AxisRanges"/>; <paramref name="limitsJson"/> (iad_axis_limits_json,
        /// optional) gives each range's source in words.
        /// </summary>
        internal static AxisRanges ToRecord(this NativeLimits l, string? limitsJson = null)
        {
            var json = MiniJson.TryParse(limitsJson) as JsonObject;
            string Source(string key) => json?.Object(key)?.String("source") ?? string.Empty;
            return new AxisRanges
            {
                Units = l.Units.Length > 0 ? l.Units : "mm",
                Kind = l.Kind.Length > 0 ? l.Kind : "unknown",
                Family = l.Family.Length > 0 ? l.Family : Registers.GENERIC_FAMILY,
                Position = l.Position.ToRecord(Source("position")),
                Speed = l.Speed.ToRecord(Source("speed")),
                Accel = l.Accel.ToRecord(Source("accel")),
                Decel = l.Decel.ToRecord(Source("decel")),
                AccelDecel = l.AccelDecel.ToRecord(Source("accel_decel")),
                Push = l.Push.ToRecord(Source("push")),
                Band = l.Band.ToRecord(Source("band")),
                PushBand = l.PushBand.ToRecord(Source("push_band")),
                StoredRows = l.StoredRows.ToRecord(Source("stored_rows")),
                JogStep = l.JogStep.ToRecord(Source("jog_step")),
                SpeedOverride = l.SpeedOverride.ToRecord(Source("speed_override")),
            };
        }

        /// <summary>The core's identity as the public <see cref="DeviceInfo"/> (no FC11 bytes, directory or categories: those are the managed readers').</summary>
        internal static DeviceInfo ToRecord(this NativeIdentity i) => new DeviceInfo
        {
            Slave = i.Slave,
            SystemCode = i.SystemCode,
            SystemName = Text(i.SystemName),
            RunStatus = i.RunStatus,
            UnitCode = i.UnitCode,
            UnitName = Text(i.UnitName),
            Firmware = Text(i.Firmware),
            SerialNumber = Text(i.SerialNumber),
            UserName = Text(i.UserName),
            ModelName = Text(i.ModelName),
            SoftLimitsMm = i.SoftMinMm.HasValue && i.SoftMaxMm.HasValue ? (i.SoftMinMm.Value, i.SoftMaxMm.Value) : ((double, double)?)null,
            MaxSpeedMmS = i.MaxSpeedMmS,
            IdentitySource = Text(i.IdentitySource),
            UnitKnown = i.UnitKnown,
            ActuatorKind = i.ActuatorKind,
            AxisNumber = i.AxisNumber,
            ActuatorSerial = Text(i.ActuatorSerial),
            MaxAccelG = i.MaxAccelG,
            MaxDecelG = i.MaxDecelG,
            ProductFamily = Text(i.ProductFamily),
            ControllerName = Text(i.ControllerName),
            Units = i.Units.Length > 0 ? i.Units : "mm",
            MaxPositions = i.MaxPositions,
            RotaryAtyp = i.ActuatorKind.HasValue ? (bool?)null : i.Rotary,
        };

        /// <summary>A stored row from the core as the public <see cref="PositionEntry"/>.</summary>
        internal static PositionEntry ToRecord(this NativePositionEntry e) => new PositionEntry
        {
            PositionMm = e.PositionMm,
            BandMm = e.BandMm,
            SpeedMmS = e.SpeedMmS,
            ZonePlusMm = e.ZonePlusMm,
            ZoneMinusMm = e.ZoneMinusMm,
            AccelG = e.AccelG,
            DecelG = e.DecelG,
            PushPower = e.PushPower,
            LoadThreshold = e.LoadThreshold,
            ControlFlags = e.ControlFlags,
            Checksum = e.Checksum,
        };

        /// <summary>A public <see cref="PositionEntry"/> as the core's input row (the checksum is the core's to compute).</summary>
        internal static NativePositionEntry ToNative(this PositionEntry e) => new NativePositionEntry
        {
            PositionMm = e.PositionMm,
            BandMm = e.BandMm,
            SpeedMmS = e.SpeedMmS,
            ZonePlusMm = e.ZonePlusMm,
            ZoneMinusMm = e.ZoneMinusMm,
            AccelG = e.AccelG,
            DecelG = e.DecelG,
            PushPower = e.PushPower,
            LoadThreshold = e.LoadThreshold,
            ControlFlags = e.ControlFlags,
        };

        /// <summary>A decoded parameter as the public <see cref="ParameterInfo"/>.</summary>
        internal static ParameterInfo ToRecord(this NativeParameter p) => new ParameterInfo
        {
            Number = p.Number ?? 0,
            Name = p.Name,
            Symbol = Text(p.Symbol),
            Item = p.Item,
            Address = Text(p.Address),
            Value = p.Value.HasValue ? (object)p.Value.Value : Text(p.Text),
            Text = Text(p.Text),
            Unit = p.Unit,
            Min = p.Min,
            Max = p.Max,
            Choices = p.Choices,
            Writable = p.Writable,
            Dangerous = p.Dangerous,
            Raw = Text(p.Raw),
            Family = p.Family.Length > 0 ? p.Family : "EC",
            Note = p.Note,
        };

        /// <summary>The outcome of a parameter write as the public <see cref="ParameterWriteResult"/>.</summary>
        internal static ParameterWriteResult ToRecord(this NativeParameterWriteResult w) => new ParameterWriteResult
        {
            Number = w.Number ?? 0,
            Name = w.Name,
            Key = w.Key,
            Address = w.Address,
            Unit = w.Unit,
            Before = w.Before,
            After = w.After,
            Changed = w.Changed,
            Verified = w.Verified,
            Committed = w.Committed,
            CommitConfirmed = w.CommitConfirmed,
            ListedForModel = w.ListedForModel,
            VerifiedForModel = w.VerifiedForModel,
            HomeParameter = w.HomeParameter,
            Warnings = w.Warnings,
            Note = w.Note,
            Skipped = w.Skipped,
            Forced = w.Forced,
        };

        /// <summary>The outcome of a software reset as the public <see cref="SoftwareResetResult"/>.</summary>
        internal static SoftwareResetResult ToRecord(this NativeResetResult r) => new SoftwareResetResult
        {
            Accepted = r.Accepted,
            ReplyLost = r.ReplyLost,
            BaudRate = r.BaudRate,
            BaudChanged = r.BaudChanged,
            Status = r.Status.ToRecord(),
            UptimeBeforeS = r.UptimeBeforeS,
            UptimeAfterS = r.UptimeAfterS,
            ElapsedS = r.ElapsedS ?? 0.0,
            Notes = r.Notes,
            Confirmed = r.Confirmed,
            ConfirmedBy = r.ConfirmedBy,
        };

        /// <summary>The outcome of a baud change as the public <see cref="BaudChangeResult"/>.</summary>
        internal static BaudChangeResult ToRecord(this NativeBaudResult b) => new BaudChangeResult
        {
            Previous = b.Previous,
            BaudRate = b.BaudRate,
            Method = b.Method,
            Code = b.Code,
            Frame = b.Frame,
            Verified = b.Verified,
            Probed = b.Probed,
            Note = b.Note.Length > 0 ? b.Note : BaudChangeResult.VolatileNote,
            CodeVerified = b.CodeVerified,
        };

        /// <summary>One alarm-history row as the public <see cref="AlarmRecord"/> (name and level come from the managed tables).</summary>
        internal static AlarmRecord ToRecord(this NativeAlarmRecord a) => new AlarmRecord(a.Index, a.Code, a.Detail, a.Address, a.TimeMs);

        /// <summary>Several rows.</summary>
        internal static IReadOnlyList<AlarmRecord> ToRecords(this IEnumerable<NativeAlarmRecord> rows) =>
            rows.Select(ToRecord).ToList().AsReadOnly();

        private static string? Text(string? value) => string.IsNullOrEmpty(value) ? null : value;
    }
}
