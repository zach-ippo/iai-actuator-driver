using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    public sealed partial class Axis
    {
        /// <summary>Classes a raw "class.item" write may touch: 7, 8, 9, 0x0C.</summary>
        public static readonly IReadOnlyCollection<int> WritableParameterClasses = new[] { 0x0007, 0x0008, 0x0009, 0x000C };

        // ------------------------------------------------------------------ non-volatile write guard

        /// <summary>
        /// Non-volatile writes this process has made on this axis (parameter commits, stored-row writes), by every axis object on
        /// the same unit (a serial port is known by its name: a reconnect or a new object per job keeps counting); read-only. The
        /// units' non-volatile memory lasts about 100,000 write cycles (IAI); skipped writes are not counted, writes by another
        /// process are not seen.
        /// </summary>
        public int NonVolatileWrites => (int)NvCounts().Total;

        /// <summary>The same, counted over the last hour (the window of the rate WARNING); read-only.</summary>
        public int NonVolatileWritesLastHour => NvCounts().LastHour;

        /// <summary>The core's counts of this unit (iad_axis_set_nv_unit): every write and those of the last hour.</summary>
        private (long Total, int LastHour) NvCounts()
        {
            long total = 0;
            var lastHour = 0;
            Run(() => NativeMethods.iad_axis_nonvolatile_writes(H, out total, out lastHour, out _));
            return (total, lastHour);
        }

        /// <summary>
        /// The unit's writes in the hour before <paramref name="now"/> (<see cref="Timing.Now"/> seconds): a test seam of the
        /// rate warning (the core's iad_testing_axis_nv_last_hour). Asking a later time drops the older writes from the window.
        /// </summary>
        internal int NonVolatileWritesLastHourAt(double now)
        {
            var count = 0;
            Run(() => NativeTestingMethods.iad_testing_axis_nv_last_hour(H, CoreTime(now), out count));
            return count;
        }

        /// <summary>
        /// Count one non-volatile write at <paramref name="now"/> (<see cref="Timing.Now"/> seconds) that was not made
        /// (<paramref name="what"/>: for the log): a test seam of the rate warning (the core's iad_testing_axis_nv_record). The
        /// unit's bookkeeping counts it as a real write: while more than <see cref="NvWarnPerHour"/> fall in the last hour,
        /// one WARNING per hour (over every axis object of this process on the unit). Never refuses: the driver warns, the
        /// integrator decides.
        /// </summary>
        internal void NoteNonVolatileWrite(string what, double now) =>
            Run(() => NativeTestingMethods.iad_testing_axis_nv_record(H, CoreTime(now), Utf8.Encode(what), out _));

        /// <summary>A <see cref="Timing.Now"/> time on the core's clock (iad_clock_now).</summary>
        private static double CoreTime(double now)
        {
            NativeErrors.Check(NativeTestingMethods.iad_clock_now(out var core));
            return now - Timing.Now + core;
        }

        // ------------------------------------------------------------------ position table

        /// <summary>
        /// Write row <paramref name="n"/> with FC10 and verify it by reading it back. Back up the unit with IA-OS first. Guards
        /// (all raise before anything is written): rows 256-267 (an EC's internal simple-data store) and a row the
        /// table cannot address (<see cref="NonVolatileWriteException"/>); values finite and inside the stroke / caps;
        /// push power 1 .. the push cap (or the unit's own unchanged value); the axis idle. Written with a warning in
        /// the log: a row past <see cref="MaxPositions"/> - 1, a row the EC's position mode does not run, an UNVERIFIED
        /// row layout, a push on the middle row of a 3-position EC, unknown control-flag bits or a change of the role
        /// bits 0x0030. <paramref name="allowNonvolatile"/> and <paramref name="allowUnsafe"/> are accepted and ignored
        /// (no longer needed). All 16 words are written, the +0xF checksum included (<paramref name="writeChecksum"/>
        /// false writes the 15 data words only). VERIFIED: the FC10 write reaches non-volatile
        /// memory at once and the unit uses the row at once (no reset needed), unlike parameters.
        /// <para>
        /// <b>Unchanged rows are not written</b> (the memory lasts about 100,000 write cycles, IAI): after the guards, when the words
        /// the write would produce equal the stored ones (checksum included, or with <paramref name="writeChecksum"/> false the 15
        /// data words and a valid stored checksum) nothing is written and the stored row comes back with
        /// <see cref="PositionEntry.Skipped"/> true (<see cref="NvSkipNote"/>). <paramref name="force"/> writes anyway. A write
        /// counts in <see cref="NonVolatileWrites"/>.
        /// </para>
        /// </summary>
        public PositionEntry WritePositionTable(int n, PositionEntry entry, bool allowNonvolatile = false, bool writeChecksum = true, bool allowUnsafe = false, bool force = false)
        {
            if (entry is null)
            {
                throw new ArgumentNullException(nameof(entry));
            }

            foreach (var (name, value) in new[]
            {
                ("PositionMm", entry.PositionMm), ("BandMm", entry.BandMm), ("SpeedMmS", entry.SpeedMmS), ("ZonePlusMm", entry.ZonePlusMm),
                ("ZoneMinusMm", entry.ZoneMinusMm), ("AccelG", entry.AccelG), ("DecelG", entry.DecelG),
            })
            {
                if (!IsFinite(value))
                {
                    throw new SoftLimitException($"{name} {value} is not a finite number");
                }
            }

            var e = entry.ToNative().ToNative();
            var o = NvOptions(allowNonvolatile, allowUnsafe, force);
            IadPositionWriteResult r = default;
            Run(() => NativeMethods.iad_axis_write_position_row(H, n, ref e, ref o, Known.Flag(writeChecksum), out r));
            var row = new NativePositionEntry(r).ToRecord();
            return Known.Bool(r.Skipped) ? row with { Skipped = true } : row;
        }

        private static IadNvWriteOptions NvOptions(bool allowNonvolatile, bool allowUnsafe, bool force)
        {
            var o = SizedStruct.Fresh<IadNvWriteOptions>();
            NativeErrors.Check(NativeMethods.iad_nv_write_options_init(ref o));
            o.AllowNonvolatile = Known.Flag(allowNonvolatile);
            o.AllowUnsafe = Known.Flag(allowUnsafe);
            o.Force = Known.Flag(force);
            return o;
        }

        // ------------------------------------------------------------------ parameters

        /// <summary>
        /// The unit's named parameters, decoded. READ-ONLY (FC03 / FC11). On an EC: the EC user parameters (IA-OS's P-numbers,
        /// family "EC"); on a CON-family controller (and any other IAI unit): the numbered CON parameters whose item is in the
        /// unit's ID map (family "CON"). Ranges: P1 1.00 .. the largest P1 read, P2 0.01-1.00, P4 0.00-5.00, choices; the CON
        /// table's own ranges.
        /// </summary>
        public IReadOnlyList<ParameterInfo> Parameters() =>
            (Json((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_parameters_json(H, b, s, out r)) as List<object?> ?? new List<object?>())
                .OfType<JsonObject>().Select(AxisRecords.ToParameter).ToList().AsReadOnly();

        /// <summary>
        /// The EC user parameters for this unit (IAI's numbering), decoded with the per-model flags (listed, writable,
        /// home parameter). READ-ONLY. A unit code without an IA-OS list lists every mapped parameter found on the
        /// unit.
        /// </summary>
        public IReadOnlyList<DecodedParameter> UserParameterList() =>
            (Json((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_user_parameters_json(H, b, s, out r)) as List<object?> ?? new List<object?>())
                .OfType<JsonObject>().Select(AxisRecords.ToDecodedParameter).ToList().AsReadOnly();

        /// <summary>One named parameter (an EC P-number or a CON No.), decoded. READ-ONLY.</summary>
        public ParameterInfo ReadParameter(int number) => ReadParameterKey(number.ToString(Inv));

        /// <summary>
        /// One parameter, decoded (READ-ONLY): <paramref name="key"/> is a named parameter number ("5", "P5", "No.35") or any
        /// item "class.item" / "class.item[entry]" the unit exposes (e.g. "8.15", "C.1[1]"; class in hex above 9). A raw item
        /// gives raw / unsigned / signed plus the CON parameter that lives there, if any.
        /// </summary>
        public ParameterInfo ReadParameter(string key)
        {
            if (ParameterNumber(key) is null)
            {
                ParseItemKey(key);  // the key's form, checked as before anything is read
            }

            return ReadParameterKey(key.Trim());
        }

        private ParameterInfo ReadParameterKey(string key)
        {
            var k = Utf8.EncodeRequired(key, nameof(key));
            return AxisRecords.ToParameter(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_read_parameter_json(H, k, b, s, out r)));
        }

        /// <summary>
        /// Write one named parameter (verified by read-back; <paramref name="commit"/> saves it with PARW like IA-OS; it
        /// applies after <see cref="SoftwareReset"/> or a power cycle - VERIFIED on the EC and on the PCON). The value as the PC
        /// tool shows it (number, label such as "1:Forward"; range-checked). On an EC this is
        /// <see cref="WriteUserParameter"/>. Dangerous CON writes (home, motor / encoder, lead / axis type, gains,
        /// PIO / fieldbus / baud configuration, widening a soft limit) are written with a warning
        /// (<see cref="ParameterWriteResult.Warnings"/>). <paramref name="allowDangerous"/>,
        /// <paramref name="allowUnlisted"/> and <paramref name="allowHomeParams"/> are accepted and ignored (no longer
        /// needed). There is no per-model opt-in. The axis must be at rest. Raises
        /// <see cref="UnsafeCommandException"/> (read-only or not offered) / <see cref="SoftLimitException"/> (nothing
        /// written), <see cref="VerificationException"/>.
        /// <para>
        /// An unchanged value writes and commits nothing (<see cref="ParameterWriteResult.Skipped"/>, <see cref="NvSkipNote"/>; the
        /// non-volatile memory lasts about 100,000 write cycles, IAI) unless a parameter this process wrote to the unit's RAM is
        /// still uncommitted (the commit then saves it) or <paramref name="force"/> is true: see
        /// <see cref="WriteUserParameter"/>. A commit counts in <see cref="NonVolatileWrites"/>.
        /// </para>
        /// </summary>
        public ParameterWriteResult WriteParameter(int number, object value, bool commit = true, bool allowDangerous = false, bool allowUnlisted = false, bool allowHomeParams = false, bool force = false) =>
            WriteParameterKey(number.ToString(Inv), value, commit, allowDangerous, allowUnlisted, allowHomeParams, force);

        /// <summary>
        /// Write one parameter by key: a named parameter number ("P2", "No.35", "2"; see
        /// <see cref="WriteParameter(int, object, bool, bool, bool, bool, bool)"/>) or a raw item "class.item" of class 7, 8, 9 or
        /// 0x0C, which takes the raw integer for the whole item; an item of unknown meaning or a dangerous one is
        /// written with a warning (<paramref name="allowDangerous"/> is accepted and ignored).
        /// </summary>
        public ParameterWriteResult WriteParameter(string key, object value, bool commit = true, bool allowDangerous = false, bool allowUnlisted = false, bool allowHomeParams = false, bool force = false)
        {
            if (ParameterNumber(key) is null)
            {
                var (classId, _, _) = ParseItemKey(key);
                if (!WritableParameterClasses.Contains(classId))
                {
                    throw new UnsafeCommandException($"class 0x{classId:X4} is not a parameter class (writable: 7, 8, 9, 0x0C); nothing written");
                }

                if (!(value is int || value is long || value is uint))
                {
                    throw new SoftLimitException($"a raw item takes the raw integer, got {UserParameters.Show(value)}; nothing written");
                }
            }

            return WriteParameterKey(key.Trim(), value, commit, allowDangerous, allowUnlisted, allowHomeParams, force);
        }

        /// <summary>
        /// Write EC user parameter P<paramref name="no"/> = <paramref name="value"/>, verify it by read-back and (with
        /// <paramref name="commit"/>) save it. Only the parameters confirmed on hardware are written (P1 7.1, P2 8.9, P3 = the
        /// parity of the ORG home pattern 9.16, P4 9.21, P5 / P6 8.14 bits 6 / 10, P7 8.50 bit 23, P9 8.2, P10 8.50 bit 18, P12
        /// 8.1 bit 16); P8, P11, P13-P18 are read-only here. The item is found by class + item ID; shared items are
        /// read-modify-written. Driver sanity ranges: P1 1.00 mm .. the largest P1 read (it can only narrow the stroke) and not
        /// below the current position, P2 0.01-1.00 mm (and &gt;= MINI 3.67 when CINP / RINP are set), P4 0.00-5.00 mm, P9 only
        /// from the valid set and 4 / 5 only on a unit already in that mode, P12 != 0 not while P9 is 3 / 4 (UNVERIFIED
        /// rules). P3 / P4 (home) are written with the home warning (ABS error 0x0EE after a reset on an absolute unit,
        /// VERIFIED) and a parameter not listed for this unit with a warning; <paramref name="allowHomeParams"/> and
        /// <paramref name="allowUnlisted"/> are accepted and ignored (no longer needed). Commit: coil 0x0428 (PARW;
        /// ON-WIRE, persistence VERIFIED on EC #3) with the DRGE b7 / DSSE b7 handshake before and after. The unit applies the
        /// value only after a software reset or a power cycle.
        /// <para>
        /// <b>An unchanged value is not written</b> (the memory lasts about 100,000 write cycles, IAI): when the unit already holds the
        /// raw value, nothing is written and nothing is committed - <see cref="ParameterWriteResult.Skipped"/> true,
        /// <see cref="ParameterWriteResult.Changed"/> and <see cref="ParameterWriteResult.Committed"/> false, note
        /// <see cref="NvSkipNote"/> - unless a parameter this process wrote to the unit's RAM is still uncommitted (an earlier
        /// <c>commit: false</c> write, or a commit that failed or was not confirmed: the commit then runs and saves it) or
        /// <paramref name="force"/> is true. A RAM-only write made by another program or process (an earlier CLI run with
        /// <c>--no-commit</c>) is not seen: commit it with <c>force: true</c>. A commit counts in <see cref="NonVolatileWrites"/>.
        /// </para>
        /// </summary>
        public ParameterWriteResult WriteUserParameter(int no, object value, bool commit = true, bool allowHomeParams = false, bool allowUnlisted = false, bool force = false) =>
            WriteParameterKey("P" + no.ToString(Inv), value, commit, allowDangerous: false, allowUnlisted, allowHomeParams, force);

        /// <summary>
        /// This unit's EC parameter list (<see cref="UserParameters.ModelParameterList"/>) from its unit code (FC11) and model
        /// string (0x0600). READ-ONLY.
        /// </summary>
        public ParameterModel ReadParameterModel() => ParameterModelRead(visibility: false);

        private ParameterModel ParameterModelRead(bool visibility) =>
            AxisRecords.ToParameterModel(JsonObj((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_parameter_model_json(H, Known.Flag(visibility), b, s, out r)));

        /// <summary>One parameter write on the core (a number, or the text a label or value is shown as) and its outcome.</summary>
        private ParameterWriteResult WriteParameterKey(string key, object value, bool commit, bool allowDangerous, bool allowUnlisted, bool allowHomeParams, bool force)
        {
            if (value is null)
            {
                throw new ArgumentNullException(nameof(value));
            }

            var k = Utf8.EncodeRequired(key, nameof(key));
            var o = SizedStruct.Fresh<IadParamWriteOptions>();
            NativeErrors.Check(NativeMethods.iad_param_write_options_init(ref o));
            o.Commit = Known.Flag(commit);
            o.AllowDangerous = Known.Flag(allowDangerous);
            o.AllowUnlisted = Known.Flag(allowUnlisted);
            o.AllowHomeParams = Known.Flag(allowHomeParams);
            o.Force = Known.Flag(force);
            IadParamWriteResult r = default;
            switch (value)
            {
                case string text:
                    var v = Utf8.EncodeRequired(text, nameof(value));
                    Run(() => NativeMethods.iad_axis_write_parameter(H, k, v, ref o, out r));
                    break;
                case IConvertible number when value is double || value is float || value is decimal || value is int || value is long
                                               || value is uint || value is short || value is ushort || value is byte || value is sbyte || value is ulong:
                    var d = number.ToDouble(CultureInfo.InvariantCulture);
                    Run(() => NativeMethods.iad_axis_write_parameter_number(H, k, d, ref o, out r));
                    break;
                default:
                    var shown = Utf8.EncodeRequired(Convert.ToString(value, CultureInfo.InvariantCulture) ?? string.Empty, nameof(value));
                    Run(() => NativeMethods.iad_axis_write_parameter(H, k, shown, ref o, out r));
                    break;
            }

            return AxisRecords.ToWriteResult(LastResult());
        }

        private static int? ParameterNumber(string key)
        {
            if (key is null)
            {
                throw new ArgumentNullException(nameof(key));
            }

            var text = key.Trim();
            foreach (var prefix in new[] { "No.", "no.", "P", "p" })
            {
                if (text.StartsWith(prefix, StringComparison.Ordinal))
                {
                    var rest = text.Substring(prefix.Length).Trim();
                    if (rest.Length > 0 && rest.All(char.IsDigit))
                    {
                        return int.Parse(rest, CultureInfo.InvariantCulture);
                    }
                }
            }

            return text.Length > 0 && text.All(char.IsDigit) ? int.Parse(text, CultureInfo.InvariantCulture) : (int?)null;
        }

        /// <summary>"8.15" / "C.1[1]" / "0x0C.1" -&gt; (class, item, entry); the class is hexadecimal, as in <see cref="ClassItem.Key"/>.</summary>
        internal static (int ClassId, int ItemId, int Entry) ParseItemKey(string key)
        {
            var text = key.Trim();
            var entry = 0;
            if (text.EndsWith("]", StringComparison.Ordinal) && text.Contains("["))
            {
                var open = text.IndexOf('[');
                if (!int.TryParse(text.Substring(open + 1, text.Length - open - 2), NumberStyles.None, CultureInfo.InvariantCulture, out entry))
                {
                    throw new UnsafeCommandException($"parameter key '{key}': bad entry");
                }

                text = text.Substring(0, open);
            }

            var dot = text.IndexOf('.');
            var cls = dot < 0 ? text : text.Substring(0, dot).Trim();
            var item = dot < 0 ? string.Empty : text.Substring(dot + 1).Trim();
            if (dot < 0 || item.Length == 0 || !item.All(char.IsDigit))
            {
                throw new UnsafeCommandException($"parameter key '{key}': use a parameter number or 'class.item' such as '8.15'");
            }

            if (cls.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            {
                cls = cls.Substring(2);
            }

            if (!int.TryParse(cls, NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out var classId))
            {
                throw new UnsafeCommandException($"parameter key '{key}': class '{cls}' is not a number");
            }

            return (classId, int.Parse(item, CultureInfo.InvariantCulture), entry);
        }

        // ------------------------------------------------------------------ software reset

        /// <summary>
        /// Restart the unit's firmware with the frames IA-OS sends (ON-WIRE; VERIFIED on EC #3 and on the PCON): servo OFF with
        /// 0x45 cmd 9 op1 0 (DSS1 must show it), then 0x45 cmd 5 op1 1, sent once (a lost answer is tolerated and the reset
        /// confirmed from the status). The SFRS coil 0x0420 is never used (alarm 0x090 with the servo on, VERIFIED). Then the
        /// link is re-synced in place (the port is never closed): after <paramref name="resetWaitS"/> the unit is probed at its
        /// power-up rate (BRSL), the current rate and the usual rates until it answers or <paramref name="timeoutS"/> passed
        /// (<see cref="ModbusTimeoutException"/>). Requires the axis at rest and no parameter write in progress (else
        /// <see cref="EcNotReadyException"/>, nothing sent). The restart is confirmed by PMSS dropping to 0, a reply gap
        /// right after the command (some units keep PMSS 1 and their uptime; the baud rate is kept across a software reset,
        /// VERIFIED on the EC-GRBP8M gripper and the PCON-CB), the uptime restarting or HEND dropping; with none of them
        /// the result says <c>Confirmed = false</c> (never an exception). An incremental unit loses its home at every
        /// software reset: run <see cref="Home"/> before the next move.
        /// </summary>
        public SoftwareResetResult SoftwareReset(double timeoutS = ResetTimeoutS, double resetWaitS = ResetWaitS)
        {
            var o = SizedStruct.Fresh<IadResetOptions>();
            NativeErrors.Check(NativeMethods.iad_reset_options_init(ref o));
            o.TimeoutS = timeoutS;
            o.ResetWaitS = resetWaitS;
            IadResetResult r = default;
            Run(() => NativeMethods.iad_axis_software_reset(H, ref o, out r));
            return new NativeResetResult(r, LastResultText()).ToRecord();
        }

        // ------------------------------------------------------------------ baud rate

        /// <summary>
        /// Switch the unit (and the open port) to <paramref name="rate"/> baud with the broadcast baud change IA-OS sends
        /// (<c>00 45 00 04 00 &lt;code&gt; 00 05</c>, ON-WIRE; no answer). 115200 and 230400 are verified on every unit tested;
        /// 38400 (0x06) is unit-dependent (<see cref="ModbusRtu.UnitDependentBaudRates"/>: the EC-GRBP8M gripper and the
        /// PCON-CB take it, VERIFIED, the EC-R6S rod ignores it) and is sent, the check after the switch deciding.
        /// Another rate with an IAI code is sent with a warning in the log (<see cref="BaudChangeResult.CodeVerified"/>
        /// false); a rate with no code is refused. <paramref name="allowUnverifiedCode"/> is accepted and ignored (no
        /// longer needed). Direct: broadcast at the current rate, wait <paramref name="settleS"/>, switch the port in
        /// place, verify with a read. Break
        /// (the sequence IA-OS uses, ON-WIRE; UNVERIFIED on an EC): port to 9600, line BREAK, broadcast at 9600, switch, verify.
        /// If the verifying read fails the unit is probed at the new, the previous and every other IAI rate: another rate
        /// raises <see cref="VerificationException"/> (the port is left at the rate that answered), none raises
        /// <see cref="ModbusTimeoutException"/>. Volatile (the unit is back at its power-up rate after a power cycle); a
        /// broadcast reaches every unit on the line. Requires the axis at rest and a transport with a baud rate.
        /// </summary>
        public BaudChangeResult SetBaudRate(int rate, BaudChangeMethod method = BaudChangeMethod.Direct, double settleS = BaudSettleS, bool allowUnverifiedCode = false)
        {
            var o = SizedStruct.Fresh<IadBaudOptions>();
            NativeErrors.Check(NativeMethods.iad_baud_options_init(ref o));
            o.Method = method == BaudChangeMethod.Break ? 1 : 0;
            o.SettleS = settleS;
            o.AllowUnverifiedCode = Known.Flag(allowUnverifiedCode);
            IadBaudResult r = default;
            Run(() => NativeMethods.iad_axis_set_baud_rate(H, rate, ref o, out r));
            BaudDetected = false;
            return new NativeBaudResult(r, LastResultText()).ToRecord();
        }

        /// <summary>The last configuration call's details as JSON right after it succeeded, "" when they cannot be read.</summary>
        private string LastResultText() => _core.LastResultJsonOrEmpty();
    }
}
