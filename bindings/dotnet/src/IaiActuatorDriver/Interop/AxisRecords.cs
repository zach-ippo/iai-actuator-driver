using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver.Interop
{
    /// <summary>
    /// The core axis's JSON outputs as the public records (switch stage S3): iad_axis_identity_json -&gt;
    /// <see cref="DeviceInfo"/>, the axis limits -&gt; <see cref="AxisLimits"/>, iad_axis_limits_json -&gt;
    /// <see cref="AxisRanges"/>, the parameter, monitor, alarm-history and write-result JSON. The core writes the field
    /// names of the Python edition; pure: no library call.
    /// </summary>
    internal static class AxisRecords
    {
        /// <summary>A JSON number as a double (null stays null).</summary>
        internal static double? Number(this JsonObject o, string key, int digits) =>
            o.Number(key) is double d ? Math.Round(d, digits) : (double?)null;

        /// <summary>[min, max] as a pair, or null.</summary>
        internal static (double Min, double Max)? Pair(this JsonObject o, string key) =>
            o.Array(key) is { Count: 2 } pair && pair[0] is double lo && pair[1] is double hi ? (lo, hi) : ((double, double)?)null;

        /// <summary>The unit's own limits (the read_axis_limits fields), or null.</summary>
        internal static AxisLimits? ToAxisLimits(JsonObject? o)
        {
            if (o is null)
            {
                return null;
            }

            return new AxisLimits
            {
                MaxSpeedMmS = o.Number("max_speed_mm_s"),
                MaxAccelG = o.Number("max_accel_g"),
                MaxDecelG = o.Number("max_decel_g"),
                MaxPushSpeedMmS = o.Number("max_push_speed_mm_s"),
                MaxPushPct = o.Number("max_push_pct"),
                MinPushPct = o.Number("min_push_pct") ?? Registers.MNPP_DEFAULT_PCT,
                MinPushFromUnit = o.Bool("min_push_from_ec") ?? false,
                PushForceFactor = o.Int("push_force_factor"),
                PushForceOffset = o.Int("push_force_offset"),
                ActuatorKind = o.Int("actuator_kind"),
                RotaryAtyp = o.Bool("rotary_atyp"),
                DefaultSpeedMmS = o.Number("default_speed_mm_s"),
                SupportsPush = o.Bool("supports_push"),
                SupportsRelative = o.Bool("supports_relative"),
                Source = o.String("source") ?? string.Empty,
                MaxPositions = o.Int("max_positions"),
                MinBand = o.Number("min_band"),
            };
        }

        /// <summary>The JSON iad_axis_set_axis_limits_json takes for <paramref name="limits"/>.</summary>
        internal static string AxisLimitsJson(AxisLimits limits)
        {
            var fields = new List<string>();
            void Add(string key, string value) => fields.Add("\"" + key + "\": " + value);
            string Num(double? v) => v.HasValue ? v.Value.ToString("R", CultureInfo.InvariantCulture) : "null";
            string Int(int? v) => v.HasValue ? v.Value.ToString(CultureInfo.InvariantCulture) : "null";
            string Flag(bool? v) => v.HasValue ? (v.Value ? "true" : "false") : "null";
            Add("max_speed_mm_s", Num(limits.MaxSpeedMmS));
            Add("max_accel_g", Num(limits.MaxAccelG));
            Add("max_decel_g", Num(limits.MaxDecelG));
            Add("max_push_speed_mm_s", Num(limits.MaxPushSpeedMmS));
            Add("max_push_pct", Num(limits.MaxPushPct));
            Add("min_push_pct", Num(limits.MinPushPct));
            Add("min_push_from_ec", Flag(limits.MinPushFromUnit));
            Add("push_force_factor", Int(limits.PushForceFactor));
            Add("push_force_offset", Int(limits.PushForceOffset));
            Add("actuator_kind", Int(limits.ActuatorKind));
            Add("rotary_atyp", Flag(limits.RotaryAtyp));
            Add("default_speed_mm_s", Num(limits.DefaultSpeedMmS));
            Add("supports_push", Flag(limits.SupportsPush));
            Add("supports_relative", Flag(limits.SupportsRelative));
            Add("source", Quote(limits.Source));
            Add("max_positions", Int(limits.MaxPositions));
            Add("min_band", Num(limits.MinBand));
            return "{" + string.Join(", ", fields) + "}";
        }

        /// <summary>The directory records (iad_axis_categories_json, or "categories" of a JSON object).</summary>
        internal static IReadOnlyList<IndexCategory> ToCategories(List<object?>? items) =>
            (items ?? new List<object?>()).OfType<JsonObject>().Select(c => new IndexCategory(
                c.Int("category_id") ?? 0,
                c.Int("model_code") ?? 0,
                c.Int("member_count") ?? 0,
                c.Int("list_count") ?? 0,
                c.Int("data_address") ?? 0,
                c.Int("id_list_address") ?? 0,
                c.Int("coil_base") ?? 0,
                c.Int("member_size") ?? 0)).ToList().AsReadOnly();

        /// <summary>
        /// The directory's records as the core's directory readers take and give them (iad_axis_info_call_json
        /// "records"): one list [CI, MD, DQ, LQ, DA, IA, BA, MS] per record.
        /// </summary>
        internal static string RecordsJson(IEnumerable<IndexCategory> records) =>
            "[" + string.Join(", ", records.Select(r => "[" + string.Join(", ", new[]
            {
                r.CategoryId, r.ModelCode, r.MemberCount, r.ListCount, r.DataAddress, r.IdListAddress, r.CoilBase, r.MemberSize,
            }.Select(v => v.ToString(CultureInfo.InvariantCulture))) + "]")) + "]";

        /// <summary>The records of "read_directory" ([[8 numbers], ...]) as <see cref="IndexCategory"/>.</summary>
        internal static IReadOnlyList<IndexCategory> ToRecords(List<object?>? items) =>
            (items ?? new List<object?>()).OfType<List<object?>>().Where(r => r.Count == 8).Select(r =>
            {
                var w = r.Select(v => v is double d ? (int)d : 0).ToArray();
                return new IndexCategory(w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
            }).ToList().AsReadOnly();

        /// <summary>A class block of the core's "read_class" (info.ClassBlock / ClassItem fields).</summary>
        internal static ClassBlock ToClassBlock(JsonObject o) => new ClassBlock(
            o.Int("class_id") ?? 0,
            o.Int("address") ?? 0,
            o.Int("words") ?? 0,
            o.Int("member_size") ?? 0,
            o.Int("entries") ?? 0,
            o.Int("id_map_address"),
            (o.Array("items") ?? new List<object?>()).OfType<JsonObject>().Select(i => new ClassItem(
                i.Int("class_id") ?? 0,
                i.Int("item_id"),
                i.Int("entry") ?? 0,
                i.Int("index") ?? 0,
                i.Int("address") ?? 0,
                i.Int("size") ?? 0,
                (uint)(i.Long("raw") ?? 0))).ToList().AsReadOnly());

        /// <summary>The identity record of the last connect / identify (iad_axis_identity_json).</summary>
        internal static DeviceInfo ToDeviceInfo(JsonObject o)
        {
            var header = o.Object("index_header");
            return new DeviceInfo
            {
                Slave = o.Int("slave") ?? 0,
                SystemCode = o.Int("system_code"),
                SystemName = o.String("system_name"),
                RunStatus = o.Int("run_status"),
                UnitCode = o.Int("unit_code"),
                UnitName = o.String("unit_name"),
                Firmware = o.String("firmware"),
                SlaveIdRaw = o.String("slave_id_raw") is string raw ? Hex(raw) : null,
                IndexHeader = header is null
                    ? null
                    : new IndexHeader(header.Int("model_code") ?? 0, header.Int("words_per_record") ?? 0, header.Int("record_count") ?? 0, header.Int("rx_buffer") ?? 0, header.Int("tx_buffer") ?? 0),
                Categories = ToCategories(o.Array("categories")),
                SerialNumber = o.String("serial_number"),
                UserName = o.String("user_name"),
                ModelName = o.String("model_name"),
                SoftLimitsMm = o.Pair("soft_limits_mm"),
                MaxSpeedMmS = o.Number("max_speed_mm_s"),
                Errors = o.Strings("errors"),
                IdentitySource = o.String("identity_source"),
                UnitKnown = o.Bool("unit_known"),
                ActuatorKind = o.Int("actuator_kind"),
                ActuatorKindName = o.String("actuator_kind_name"),
                AxisNumber = o.Int("axis_number"),
                ActuatorSerial = o.String("actuator_serial"),
                Accl = o.Long("accl") is long accl ? (uint)accl : (uint?)null,
                MaxAccelG = o.Number("max_accel_g"),
                MaxDecelG = o.Number("max_decel_g"),
                SpeedLimitSource = o.String("speed_limit_source"),
                ProductFamily = o.String("product_family"),
                ControllerName = o.String("controller_name"),
                Units = o.String("units") ?? "mm",
                MaxPositions = o.Int("max_positions"),
                RotaryAtyp = o.Bool("rotary_atyp"),
            };
        }

        /// <summary>Every range the driver enforces (iad_axis_limits_json).</summary>
        internal static AxisRanges ToRanges(JsonObject o)
        {
            LimitRange Range(string key)
            {
                var r = o.Object(key) ?? new JsonObject();
                return new LimitRange
                {
                    Min = r.Number("min"),
                    Max = r.Number("max"),
                    Unit = r.String("unit") ?? string.Empty,
                    Source = r.String("source") ?? string.Empty,
                    UnitMax = r.Number("unit_max"),
                    Configured = r.Number("configured"),
                    Available = r.Bool("available"),
                    RecommendedMin = r.Number("recommended_min"),
                    Count = r.Int("count"),
                };
            }

            return new AxisRanges
            {
                Units = o.String("units") ?? "mm",
                Kind = o.String("kind") ?? "unknown",
                Family = o.String("family") ?? Registers.GENERIC_FAMILY,
                Position = Range("position"),
                Speed = Range("speed"),
                Accel = Range("accel"),
                Decel = Range("decel"),
                AccelDecel = Range("accel_decel"),
                Push = Range("push"),
                Band = Range("band"),
                PushBand = Range("push_band"),
                StoredRows = Range("stored_rows"),
                JogStep = Range("jog_step"),
                SpeedOverride = Range("speed_override"),
            };
        }

        /// <summary>One decoded parameter (iad_axis_parameters_json / iad_axis_read_parameter_json).</summary>
        internal static ParameterInfo ToParameter(JsonObject p)
        {
            var family = p.String("family") ?? "EC";
            var unit = p.String("unit") ?? string.Empty;
            object? value = p.Get("value") switch
            {
                double d when family == "raw" => (long)d,
                double d when unit is "mm" or "deg" or "mm/s" or "deg/s" or "G" || d != Math.Floor(d) => d,
                double d => (int)d,
                string text => text,
                _ => null,
            };
            return new ParameterInfo
            {
                Number = p.Int("number") ?? 0,
                Name = p.String("name") ?? string.Empty,
                Symbol = p.String("symbol"),
                Item = p.String("item") ?? string.Empty,
                Address = p.String("address"),
                Value = value,
                Text = p.String("text"),
                Unit = p.String("unit") ?? string.Empty,
                Min = p.Number("min"),
                Max = p.Number("max"),
                Choices = Choices(p.Object("choices")),
                Writable = p.Bool("writable") ?? false,
                Dangerous = p.Bool("dangerous") ?? false,
                Raw = p.String("raw"),
                Family = family,
                Unsigned = p.Long("unsigned") is long u ? (uint)u : (uint?)null,
                Signed = p.Long("signed") is long sv ? unchecked((int)sv) : (int?)null,
                Words = p.Int("words"),
                Named = p.Int("named"),
                Note = p.String("note") ?? string.Empty,
            };
        }

        /// <summary>One EC user parameter of iad_axis_user_parameters_json.</summary>
        internal static DecodedParameter ToDecodedParameter(JsonObject p)
        {
            object? value = p.Get("value") switch
            {
                double d when (p.String("unit") ?? string.Empty) == "mm" => d,
                double d => (int)d,
                string s => s,
                _ => null,
            };
            return new DecodedParameter
            {
                No = p.Int("no") ?? 0,
                Name = p.String("name") ?? string.Empty,
                Key = p.String("key") ?? string.Empty,
                Unit = p.String("unit") ?? string.Empty,
                Encoding = p.String("encoding") ?? string.Empty,
                Evidence = p.String("evidence") ?? string.Empty,
                ReadOnly = p.Bool("read_only") ?? false,
                Note = p.String("note") ?? string.Empty,
                Found = p.Bool("found") ?? false,
                Address = p.String("address"),
                RawHex = p.String("raw_hex"),
                Value = value,
                Text = p.String("text") ?? string.Empty,
                Pattern = p.Long("pattern") is long pattern ? (uint)pattern : (uint?)null,
                Listed = p.Bool("listed") ?? false,
                VerifiedForModel = p.Bool("verified_for_model") ?? false,
                HomeParameter = p.Bool("home_parameter") ?? false,
                Writable = p.Bool("writable") ?? false,
                Choices = Choices(p.Object("choices")),
                ShownByIaos = p.Bool("shown_by_iaos"),
                HiddenReason = p.String("hidden_reason") ?? string.Empty,
                ModelNote = p.String("model_note") ?? string.Empty,
            };
        }

        /// <summary>This unit's parameter list (iad_axis_parameter_model_json).</summary>
        internal static ParameterModel ToParameterModel(JsonObject m) => new ParameterModel
        {
            UnitCode = m.Int("unit_code"),
            SystemCode = m.Int("system_code"),
            ModelName = m.String("model_name"),
            Family = m.String("family"),
            Known = m.Bool("known") ?? false,
            Candidates = Ints(m.Array("candidates")),
            Listed = Ints(m.Array("listed")),
            Hidden = (m.Array("hidden") ?? new List<object?>()).OfType<JsonObject>().Select(h => (h.Int("no") ?? 0, h.String("reason") ?? string.Empty)).ToList().AsReadOnly(),
            Verified = m.Bool("verified") ?? false,
            SeenOn = m.Strings("seen_on"),
            Note = m.String("note") ?? string.Empty,
        };

        /// <summary>The outcome of a parameter write (iad_axis_last_result_json after a write).</summary>
        internal static ParameterWriteResult ToWriteResult(JsonObject w)
        {
            object? value = w.Get("value") switch
            {
                double d when (w.String("unit") ?? string.Empty) is "mm" or "deg" || d != Math.Floor(d) => d,
                double d => (long)d is long l && l >= int.MinValue && l <= int.MaxValue ? (int)l : (object)l,
                string s => s,
                _ => null,
            };
            return new ParameterWriteResult
            {
                Number = w.Int("no") ?? 0,
                Name = w.String("name") ?? string.Empty,
                Key = w.String("key") ?? string.Empty,
                Address = w.String("address") ?? string.Empty,
                Unit = w.String("unit") ?? string.Empty,
                Before = w.String("before") ?? string.Empty,
                After = w.String("after") ?? string.Empty,
                Value = value,
                RawBefore = w.String("raw_before") ?? string.Empty,
                RawAfter = w.String("raw_after") ?? string.Empty,
                Changed = w.Bool("changed") ?? false,
                Verified = w.Bool("verified") ?? false,
                Committed = w.Bool("committed") ?? false,
                ListedForModel = w.Bool("listed_for_model") ?? false,
                VerifiedForModel = w.Bool("verified_for_model") ?? false,
                HomeParameter = w.Bool("home_parameter") ?? false,
                Warnings = w.Strings("warnings"),
                Note = w.String("note") ?? string.Empty,
                CommitConfirmed = w.Bool("commit_confirmed") ?? false,
                Skipped = w.Bool("skipped") ?? false,
            };
        }

        /// <summary>One read of the monitor block (iad_axis_read_monitor_json).</summary>
        internal static MonitorReading ToMonitor(JsonObject m) => new MonitorReading
        {
            Units = m.String("units") ?? "mm",
            Position = m.Number("position"),
            Velocity = m.Number("velocity"),
            Current = m.Int("current"),
            CurrentPct = m.Number("current_pct"),
            RatedCurrent = m.Long("rated_current") is long rated ? (uint)rated : (uint?)null,
            Deviation = m.Int("deviation"),
            Alarm = m.Int("alarm") ?? 0,
            AlarmName = m.String("alarm_name") ?? string.Empty,
            AlarmLevel = m.String("alarm_level") ?? string.Empty,
            UptimeS = m.Number("uptime_s"),
            Raw = (m.Object("raw") ?? new JsonObject()).Keys.ToDictionary(k => k, k => m.Object("raw")!.String(k) ?? string.Empty),
            Words = (m.Array("words") ?? new List<object?>()).Select(w => w is double d ? (ushort)d : (ushort)0).ToList().AsReadOnly(),
        };

        /// <summary>The alarm history (iad_axis_read_alarm_history_json), newest first.</summary>
        internal static IReadOnlyList<AlarmRecord> ToAlarmRecords(List<object?>? rows) =>
            (rows ?? new List<object?>()).OfType<JsonObject>().Select(r => new AlarmRecord(
                r.Int("index") ?? 0,
                r.Int("code") ?? 0,
                r.Int("detail") ?? 0,
                r.Int("address") ?? 0,
                r.Long("time_ms"))).ToList().AsReadOnly();

        /// <summary>A JSON string literal.</summary>
        internal static string Quote(string text)
        {
            var b = new System.Text.StringBuilder("\"");
            foreach (var c in text)
            {
                switch (c)
                {
                    case '"':
                        b.Append("\\\"");
                        break;
                    case '\\':
                        b.Append("\\\\");
                        break;
                    default:
                        if (c < 0x20)
                        {
                            b.Append("\\u").Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                        }
                        else
                        {
                            b.Append(c);
                        }

                        break;
                }
            }

            return b.Append('"').ToString();
        }

        private static IReadOnlyDictionary<string, string> Choices(JsonObject? choices) =>
            choices is null ? new Dictionary<string, string>() : choices.Keys.ToDictionary(k => k, k => choices.String(k) ?? string.Empty);

        private static IReadOnlyList<int> Ints(List<object?>? items) =>
            (items ?? new List<object?>()).OfType<double>().Select(d => (int)d).ToList().AsReadOnly();

        /// <summary>The outcome of a baud change as the public record (the core's BaudChangeResult JSON, as a port's change gives it).</summary>
        internal static BaudChangeResult ToBaudChange(JsonObject o)
        {
            var note = o.String("note");
            return new BaudChangeResult
            {
                Previous = o.Int("previous") ?? 0,
                BaudRate = o.Int("baudrate") ?? 0,
                Method = o.String("method") == "break" ? BaudChangeMethod.Break : BaudChangeMethod.Direct,
                Code = o.Int("code") ?? 0,
                Frame = o.String("frame") ?? string.Empty,
                Verified = o.Bool("verified") ?? false,
                Probed = o.Bool("probed") ?? false,
                Note = string.IsNullOrEmpty(note) ? BaudChangeResult.VolatileNote : note!,
                CodeVerified = o.Bool("code_verified") ?? true,
            };
        }

        /// <summary>Bytes of a hex text "ec ff 00 00 02".</summary>
        internal static IReadOnlyList<byte> Hex(string text) =>
            new ReadOnlyCollection<byte>(text.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries)
                .Select(h => byte.Parse(h, NumberStyles.HexNumber, CultureInfo.InvariantCulture)).ToList());
    }
}
