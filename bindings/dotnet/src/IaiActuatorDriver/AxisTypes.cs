using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver
{
    /// <summary>How servo / home / alarm-reset commands are sent (moves always use 0x9900, stops always the STOP coil).</summary>
    public enum CommandMethod
    {
        /// <summary>FC05 coils (SON, HOME, ALRS; VERIFIED). The default.</summary>
        Coil,

        /// <summary>IAI function 0x45 (servo cmd 9 and home cmd 0x0A VERIFIED on an EC; alarm reset cmd 5 op1 0 frame accepted, clearing a real alarm UNVERIFIED).</summary>
        Iai,

        /// <summary>EXPERIMENTAL: IAI function 0x48 (UNVERIFIED on every unit tested; they all answer 0x45).</summary>
        Iai2,
    }

    /// <summary>How <see cref="Axis.SetBaudRate"/> switches the rate.</summary>
    public enum BaudChangeMethod
    {
        /// <summary>Broadcast at the current rate, then switch the port in place (VERIFIED for 115200 / 230400).</summary>
        Direct,

        /// <summary>Port to 9600, line BREAK, broadcast at 9600 (the sequence IA-OS uses, ON-WIRE; UNVERIFIED on an EC).</summary>
        Break,
    }

    /// <summary>Outcome of a push.</summary>
    public enum PushOutcome
    {
        /// <summary>The push met the work (PEND and DSSE bit 0, PSFL 0); the motor keeps pushing until the next move.</summary>
        Contact,

        /// <summary>The push went over its whole band without contact (DSS1 PSFL).</summary>
        Missed,
    }

    /// <summary>
    /// Settings of an <see cref="Axis"/>. Every cap is optional: <c>null</c> = the unit's own maximum, read from the unit.
    /// A configured cap can only narrow the unit's own range, never widen it.
    /// </summary>
    public sealed record AxisOptions
    {
        /// <summary>Soft limits (min, max) for this application, or <c>null</c> for the unit's own soft limits (minus its 0.30 margin).</summary>
        public (double Min, double Max)? Stroke { get; init; }

        /// <summary>Optional cap for any commanded speed (units/s); <c>null</c> = MAXV 7.4.</summary>
        public double? MaxSpeedMmS { get; init; }

        /// <summary>Optional cap for any commanded acceleration / deceleration (G); <c>null</c> = MAXA / MAXD 7.5 / 7.6.</summary>
        public double? MaxAccelG { get; init; }

        /// <summary>Optional cap for the push force (1..100 %); <c>null</c> = MXPP 7.9.</summary>
        public double? MaxPushPct { get; init; }

        /// <summary>How servo / home / alarm reset are sent (default <see cref="CommandMethod.Coil"/>).</summary>
        public CommandMethod Strategy { get; init; } = CommandMethod.Coil;

        /// <summary>Status polling period while waiting (s).</summary>
        public double PollIntervalS { get; init; } = 0.02;

        /// <summary>Take Modbus control (PMSL) automatically at rest before a command (default true).</summary>
        public bool AutoModbusControl { get; init; } = true;

        /// <summary>A move, home or STOP counts as done only once PNOW stayed unchanged this long (PEND can rise early; VERIFIED).</summary>
        public double SettleS { get; init; } = Axis.SettleSDefault;

        /// <summary>A push outcome must hold this long, position unchanged, before it is reported (hardware test G5).</summary>
        public double PushSettleS { get; init; } = Axis.PushSettleSDefault;

        /// <summary>When a move counts as done by default (<see cref="CompletionRule.Safe"/>; <see cref="Axis.EnableFast"/> sets Fast).</summary>
        public CompletionRule Completion { get; init; } = CompletionRule.Safe;
    }

    /// <summary>
    /// The unit's own limits, read from class 7 by item ID (no per-model table in the driver). MAXV / MAXA VERIFIED on EC #3
    /// (800.00 mm/s, 1.00 G); the meaning of the push items is UNVERIFIED. Positions / speeds in the axis's units (mm, or
    /// deg on a rotary axis).
    /// </summary>
    public sealed record AxisLimits
    {
        /// <summary>MAXV 7.4 (units/s).</summary>
        public double? MaxSpeedMmS { get; init; }

        /// <summary>MAXA 7.5 (G).</summary>
        public double? MaxAccelG { get; init; }

        /// <summary>MAXD 7.6 (G).</summary>
        public double? MaxDecelG { get; init; }

        /// <summary>MXPH 7.8 (units/s).</summary>
        public double? MaxPushSpeedMmS { get; init; }

        /// <summary>MXPP 7.9: the driver's hard push cap (%; 0 = no push).</summary>
        public double? MaxPushPct { get; init; }

        /// <summary>MNPP 7.58, or 20 % when absent: only a recommendation (below it the driver warns).</summary>
        public double MinPushPct { get; init; } = Registers.MNPP_DEFAULT_PCT;

        /// <summary>MNPP was read from the unit (else the 20 % default).</summary>
        public bool MinPushFromUnit { get; init; }

        /// <summary>TCPC 7.34 (0 = no push).</summary>
        public int? PushForceFactor { get; init; }

        /// <summary>TCOA 7.35.</summary>
        public int? PushForceOffset { get; init; }

        /// <summary>AKID 7.33 bits 0-7.</summary>
        public int? ActuatorKind { get; init; }

        /// <summary>ATYP 7.7 bit 8: rotary on a unit without AKID (CON controllers; UNVERIFIED).</summary>
        public bool? RotaryAtyp { get; init; }

        /// <summary>The default speed 0x0812 (item 8.10 VCMD), read only as the speed fallback.</summary>
        public double? DefaultSpeedMmS { get; init; }

        /// <summary>SUPT b11 (class 1; <c>null</c> if not read).</summary>
        public bool? SupportsPush { get; init; }

        /// <summary>SUPT b8 (class 1; <c>null</c> if not read).</summary>
        public bool? SupportsRelative { get; init; }

        /// <summary>Where the values came from, or what could not be read.</summary>
        public string Source { get; init; } = string.Empty;

        /// <summary>Stored rows the unit uses (MAXP 3.2, else the directory's row count), once read.</summary>
        public int? MaxPositions { get; init; }

        /// <summary>MINI 3.67 when the unit enforces it (SUP2 CINP and ATYP RINP set), else <c>null</c>.</summary>
        public double? MinBand { get; init; }

        /// <summary>"deg" on a rotary axis (AKID 1; without AKID, ATYP 7.7 bit 8), else "mm".</summary>
        public string Units => Registers.AxisUnits(ActuatorKind, RotaryAtyp);

        /// <summary>The actuator kind in words.</summary>
        public string Kind => Registers.AxisKindName(ActuatorKind, RotaryAtyp);

        /// <summary>Rotary axis (AKID 1, or ATYP bit 8 without AKID).</summary>
        public bool? Rotary => ActuatorKind.HasValue ? ActuatorKind.Value == Registers.AKID_ROTARY : RotaryAtyp;

        /// <summary>Push is possible: SUPT PUSH not 0, MXPP &gt; 0 and TCPC &gt; 0 (UNVERIFIED rule). <c>null</c> if none was read.</summary>
        public bool? PushAvailable
        {
            get
            {
                if (MaxPushPct is null && PushForceFactor is null && SupportsPush is null)
                {
                    return null;
                }

                return !(SupportsPush == false || MaxPushPct == 0 || PushForceFactor == 0);
            }
        }

        /// <summary>Push force estimate TCPC x % / MXPP + TCOA (unit probably N; UNVERIFIED).</summary>
        public double? PushForceEstimate(double pct)
        {
            if (!MaxPushPct.HasValue || MaxPushPct.Value == 0 || !PushForceFactor.HasValue)
            {
                return null;
            }

            return Math.Round((PushForceFactor.Value * pct / MaxPushPct.Value) + (PushForceOffset ?? 0), 1);
        }

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["max_speed_mm_s"] = MaxSpeedMmS,
            ["max_accel_g"] = MaxAccelG,
            ["max_decel_g"] = MaxDecelG,
            ["max_push_speed_mm_s"] = MaxPushSpeedMmS,
            ["max_push_pct"] = MaxPushPct,
            ["min_push_pct"] = MinPushPct,
            ["min_push_from_unit"] = MinPushFromUnit,
            ["push_force_factor"] = PushForceFactor,
            ["push_force_offset"] = PushForceOffset,
            ["actuator_kind"] = ActuatorKind,
            ["rotary_atyp"] = RotaryAtyp,
            ["default_speed_mm_s"] = DefaultSpeedMmS,
            ["supports_push"] = SupportsPush,
            ["supports_relative"] = SupportsRelative,
            ["source"] = Source,
            ["max_positions"] = MaxPositions,
            ["min_band"] = MinBand,
            ["push_available"] = PushAvailable,
            ["rotary"] = Rotary,
            ["units"] = Units,
            ["kind"] = Kind,
        };
    }

    /// <summary>One range the driver enforces (see <see cref="Axis.Limits"/>).</summary>
    public sealed record LimitRange
    {
        /// <summary>Smallest accepted value (<c>null</c>: not known).</summary>
        public double? Min { get; init; }

        /// <summary>Largest accepted value (<c>null</c>: not known).</summary>
        public double? Max { get; init; }

        /// <summary>"mm" / "deg" / "mm/s" / "G" / "%" / "row".</summary>
        public string Unit { get; init; } = string.Empty;

        /// <summary>Where the range comes from.</summary>
        public string Source { get; init; } = string.Empty;

        /// <summary>The unit's own maximum (<c>null</c>: not known / not applicable).</summary>
        public double? UnitMax { get; init; }

        /// <summary>The configured cap (<c>null</c>: none).</summary>
        public double? Configured { get; init; }

        /// <summary>Push only: the unit can push (<c>null</c>: not known).</summary>
        public bool? Available { get; init; }

        /// <summary>Push only: MNPP 7.58 or 20 % (a recommendation, not a limit).</summary>
        public double? RecommendedMin { get; init; }

        /// <summary>Stored rows only: the number of rows.</summary>
        public int? Count { get; init; }

        /// <summary>JSON-friendly dictionary (keys that do not apply are left out).</summary>
        public IDictionary<string, object?> ToDictionary()
        {
            var d = new Dictionary<string, object?> { ["min"] = Min, ["max"] = Max, ["unit"] = Unit, ["source"] = Source };
            if (UnitMax.HasValue || Configured.HasValue)
            {
                d["unit_max"] = UnitMax;
                d["configured"] = Configured;
            }

            if (Available.HasValue || RecommendedMin.HasValue)
            {
                d["available"] = Available;
                d["recommended_min"] = RecommendedMin;
            }

            if (Count.HasValue)
            {
                d["count"] = Count;
            }

            return d;
        }
    }

    /// <summary>
    /// Every range the driver enforces on one axis, read from the unit itself (<see cref="Axis.Limits"/>). Positions in
    /// <see cref="Units"/> (mm, or deg on a rotary axis), speeds in units/s, accelerations in G.
    /// </summary>
    public sealed record AxisRanges
    {
        public string Units { get; init; } = "mm";

        public string Kind { get; init; } = "unknown";

        public string Family { get; init; } = Registers.GENERIC_FAMILY;

        /// <summary>The effective stroke: soft limits 7.1 / 7.2 minus the 0.30 margin, intersected with a configured stroke.</summary>
        public LimitRange Position { get; init; } = new LimitRange();

        /// <summary>0.01 .. min(MAXV 7.4, configured cap).</summary>
        public LimitRange Speed { get; init; } = new LimitRange();

        /// <summary>0.01 .. min(MAXA 7.5, configured cap) (a move with its own deceleration).</summary>
        public LimitRange Accel { get; init; } = new LimitRange();

        /// <summary>0.01 .. min(MAXD 7.6, configured cap) (a move with its own deceleration).</summary>
        public LimitRange Decel { get; init; } = new LimitRange();

        /// <summary>0.01 .. min(MAXA, MAXD, configured cap): a 0x9900 move uses one value for both.</summary>
        public LimitRange AccelDecel { get; init; } = new LimitRange();

        /// <summary>Push force 1 .. min(MXPP 7.9, configured cap) %, with the MNPP recommendation.</summary>
        public LimitRange Push { get; init; } = new LimitRange();

        /// <summary>Positioning band: MINI 3.67 when enforced (else 0.01) .. 10.</summary>
        public LimitRange Band { get; init; } = new LimitRange();

        /// <summary>Push band: 0.01 .. the stroke.</summary>
        public LimitRange PushBand { get; init; } = new LimitRange();

        /// <summary>Stored rows 0 .. <see cref="Axis.MaxPositions"/> - 1.</summary>
        public LimitRange StoredRows { get; init; } = new LimitRange();

        /// <summary>Jog step: 0.01 .. the stroke.</summary>
        public LimitRange JogStep { get; init; } = new LimitRange();

        /// <summary>Speed override of <see cref="Axis.MoveToPosition"/>: 1 .. 100 %.</summary>
        public LimitRange SpeedOverride { get; init; } = new LimitRange();

        /// <summary>JSON-friendly dictionary (the keys of the Python edition's <c>limits()</c>).</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["units"] = Units,
            ["kind"] = Kind,
            ["family"] = Family,
            ["position"] = Position.ToDictionary(),
            ["speed"] = Speed.ToDictionary(),
            ["accel"] = Accel.ToDictionary(),
            ["decel"] = Decel.ToDictionary(),
            ["accel_decel"] = AccelDecel.ToDictionary(),
            ["push"] = Push.ToDictionary(),
            ["band"] = Band.ToDictionary(),
            ["push_band"] = PushBand.ToDictionary(),
            ["stored_rows"] = StoredRows.ToDictionary(),
            ["jog_step"] = JogStep.ToDictionary(),
            ["speed_override"] = SpeedOverride.ToDictionary(),
        };
    }

    /// <summary>
    /// One position-table row (16 words at the table base + 16 x n; 0x1000 on every unit tested). Units: the axis's units
    /// (mm, or deg on a rotary axis), units/s, G. To change a row, start from what the unit holds so its control flags
    /// (and the unknown 0x0010 / 0x0020 role bits) are kept: <c>axis.ReadPositionTable(2) with { PositionMm = 60.0 }</c>.
    /// </summary>
    public sealed record PositionEntry
    {
        public double PositionMm { get; init; }

        public double BandMm { get; init; } = 0.10;

        public double SpeedMmS { get; init; } = 50.0;

        public double ZonePlusMm { get; init; }

        public double ZoneMinusMm { get; init; }

        public double AccelG { get; init; } = 0.30;

        public double DecelG { get; init; } = 0.30;

        /// <summary>PPOW: push current limit, 255 = 100 %.</summary>
        public int PushPower { get; init; }

        /// <summary>LPOW: load output threshold.</summary>
        public int LoadThreshold { get; init; }

        /// <summary>CTLF: bit 1 push, bit 2 push away from home, bit 3 incremental (+ role bits 0x0030).</summary>
        public int ControlFlags { get; init; }

        /// <summary>Word +0xF as read (byte-sum checksum of the row), <c>null</c> when not read.</summary>
        public int? Checksum { get; init; }

        /// <summary>
        /// <see cref="Axis.WritePositionTable"/>: the row already held this data, nothing was written (<see cref="Axis.NvSkipNote"/>).
        /// Not part of the equality: a skipped row equals the stored row it stands for.
        /// </summary>
        public bool Skipped { get; init; }

        /// <summary>Equal when every data field is equal (<see cref="Skipped"/> does not take part).</summary>
        public bool Equals(PositionEntry? other) =>
            other is not null
            && PositionMm.Equals(other.PositionMm) && BandMm.Equals(other.BandMm) && SpeedMmS.Equals(other.SpeedMmS)
            && ZonePlusMm.Equals(other.ZonePlusMm) && ZoneMinusMm.Equals(other.ZoneMinusMm)
            && AccelG.Equals(other.AccelG) && DecelG.Equals(other.DecelG)
            && PushPower == other.PushPower && LoadThreshold == other.LoadThreshold
            && ControlFlags == other.ControlFlags && Checksum == other.Checksum;

        /// <inheritdoc />
        public override int GetHashCode()
        {
            unchecked
            {
                var hash = 17;
                foreach (var part in new object?[] { PositionMm, BandMm, SpeedMmS, ZonePlusMm, ZoneMinusMm, AccelG, DecelG, PushPower, LoadThreshold, ControlFlags, Checksum })
                {
                    hash = (hash * 31) + (part?.GetHashCode() ?? 0);
                }

                return hash;
            }
        }

        /// <summary>A push row (CTLF bit 1).</summary>
        public bool IsPush => (ControlFlags & Registers.CTLF_PUSH) != 0;

        /// <summary>An incremental row (CTLF bit 3): the driver never runs it.</summary>
        public bool Incremental => (ControlFlags & Registers.CTLF_INC) != 0;

        /// <summary>Whether the read checksum matches the row's data (<c>null</c> if no checksum was read).</summary>
        public bool? ChecksumOk => Checksum.HasValue ? Checksum.Value == Registers.PositionChecksum(ToRegisters()) : (bool?)null;

        /// <summary>Decode 15 or 16 words.</summary>
        public static PositionEntry FromRegisters(IReadOnlyList<ushort> words)
        {
            if (words is null || words.Count < Registers.POSITION_WORDS_WRITABLE)
            {
                throw new ArgumentException($"need at least {Registers.POSITION_WORDS_WRITABLE} words", nameof(words));
            }

            return new PositionEntry
            {
                PositionMm = Math.Round(Registers.FromI32Words(words[0], words[1]) * Registers.POSITION_SCALE, 2),
                BandMm = Math.Round(Registers.FromU32Words(words[2], words[3]) * Registers.POSITION_SCALE, 2),
                SpeedMmS = Math.Round(Registers.FromU32Words(words[4], words[5]) * Registers.VELOCITY_SCALE, 2),
                ZonePlusMm = Math.Round(Registers.FromI32Words(words[6], words[7]) * Registers.POSITION_SCALE, 2),
                ZoneMinusMm = Math.Round(Registers.FromI32Words(words[8], words[9]) * Registers.POSITION_SCALE, 2),
                AccelG = Math.Round(words[10] * Registers.ACCEL_SCALE, 2),
                DecelG = Math.Round(words[11] * Registers.ACCEL_SCALE, 2),
                PushPower = words[12],
                LoadThreshold = words[13],
                ControlFlags = words[14],
                Checksum = words.Count > 15 ? words[15] : (int?)null,
            };
        }

        /// <summary>Encode the 15 writable words (+0x0..+0xE). Throws <see cref="ArgumentException"/> for unencodable values.</summary>
        public ushort[] ToRegisters()
        {
            foreach (var (name, value) in new[] { ("PushPower", PushPower), ("LoadThreshold", LoadThreshold), ("ControlFlags", ControlFlags) })
            {
                if (value < 0 || value > 0xFFFF)
                {
                    throw new ArgumentException($"{name} must be in 0..65535");
                }
            }

            var accel = Registers.ToCounts(AccelG, Registers.ACCEL_SCALE);
            var decel = Registers.ToCounts(DecelG, Registers.ACCEL_SCALE);
            if (accel < 0 || accel > 0xFFFF || decel < 0 || decel > 0xFFFF)
            {
                throw new ArgumentException("accel/decel out of range");
            }

            var words = new List<ushort>(Registers.POSITION_WORDS_WRITABLE);
            void Add((ushort High, ushort Low) w)
            {
                words.Add(w.High);
                words.Add(w.Low);
            }

            Add(Registers.ToI32Words(Registers.ToCounts(PositionMm)));
            Add(Registers.ToU32Words(Registers.ToCounts(BandMm)));
            Add(Registers.ToU32Words(Registers.ToCounts(SpeedMmS, Registers.VELOCITY_SCALE)));
            Add(Registers.ToI32Words(Registers.ToCounts(ZonePlusMm)));
            Add(Registers.ToI32Words(Registers.ToCounts(ZoneMinusMm)));
            words.Add((ushort)accel);
            words.Add((ushort)decel);
            words.Add((ushort)PushPower);
            words.Add((ushort)LoadThreshold);
            words.Add((ushort)ControlFlags);
            return words.ToArray();
        }

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["position_mm"] = PositionMm,
            ["band_mm"] = BandMm,
            ["speed_mm_s"] = SpeedMmS,
            ["zone_plus_mm"] = ZonePlusMm,
            ["zone_minus_mm"] = ZoneMinusMm,
            ["accel_g"] = AccelG,
            ["decel_g"] = DecelG,
            ["push_power"] = PushPower,
            ["load_threshold"] = LoadThreshold,
            ["control_flags"] = ControlFlags,
            ["checksum"] = Checksum,
            ["checksum_ok"] = ChecksumOk,
            ["skipped"] = Skipped,
        };
    }

    /// <summary>Outcome of <see cref="Axis.Push"/>.</summary>
    public sealed record PushResult(PushOutcome Outcome, double PositionMm, Status Status)
    {
        /// <summary>E.g. a force below the unit's recommended minimum (MNPP, or 20 %), which is allowed.</summary>
        public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

        /// <summary>The push met the work; the motor keeps pushing until the next move command.</summary>
        public bool Contact => Outcome == PushOutcome.Contact;

        /// <summary>The push went over the whole band without contact.</summary>
        public bool Missed => Outcome == PushOutcome.Missed;

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["outcome"] = Outcome == PushOutcome.Contact ? "contact" : "missed",
            ["position_mm"] = PositionMm,
            ["status"] = AxisJson.Status(Status),
            ["warnings"] = Warnings.ToList(),
        };
    }

    /// <summary>Outcome of <see cref="Axis.WriteParameter(int, object, bool, bool, bool, bool, bool)"/> (values as the PC tool shows them).</summary>
    public sealed record ParameterWriteResult
    {
        public int Number { get; init; }

        public string Name { get; init; } = string.Empty;

        /// <summary>class.item, e.g. "8.9".</summary>
        public string Key { get; init; } = string.Empty;

        /// <summary>Where the item was found on this unit (by item ID), e.g. "0x0810".</summary>
        public string Address { get; init; } = string.Empty;

        public string Unit { get; init; } = string.Empty;

        public string Before { get; init; } = string.Empty;

        public string After { get; init; } = string.Empty;

        public object? Value { get; init; }

        public string RawBefore { get; init; } = string.Empty;

        public string RawAfter { get; init; } = string.Empty;

        /// <summary>The item's raw value changed (an unchanged one is committed only to save earlier RAM writes, or forced).</summary>
        public bool Changed { get; init; }

        /// <summary>Read back equal to what was written.</summary>
        public bool Verified { get; init; }

        /// <summary>The PARW coil 0x0428 was sent and the handshake ended idle (DRGE b7 = 0, DSSE b7 = 1).</summary>
        public bool Committed { get; init; }

        public bool ListedForModel { get; init; }

        /// <summary>Kept for compatibility: an IA-OS parameter list is recorded for the unit code.</summary>
        public bool VerifiedForModel { get; init; }

        /// <summary>A home parameter (EC P3 / P4, CON No.5 / No.22).</summary>
        public bool HomeParameter { get; init; }

        public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

        public string Note { get; init; } = string.Empty;

        /// <summary>The unit was seen acting on the commit coil (DRGE b7 read 1 or DSSE b7 read 0 after it).</summary>
        public bool CommitConfirmed { get; init; }

        /// <summary>
        /// The unit already held the value and no parameter this process wrote to the unit's RAM waits for a commit: nothing
        /// written, nothing committed (<see cref="Axis.NvSkipNote"/>; <c>force: true</c> writes and commits anyway).
        /// </summary>
        public bool Skipped { get; init; }

        /// <summary>
        /// The value is in non-volatile memory as far as the unit showed: committed and the commit was seen, or <see cref="Skipped"/>
        /// - the unit already held it and nothing this process wrote is uncommitted, ASSUMING no RAM-only write by another program
        /// or process (an earlier CLI run with <c>--no-commit</c>: write again with <c>force: true</c>).
        /// </summary>
        public bool Saved => Skipped || (Committed && CommitConfirmed);

        /// <summary>Always true: the unit applies a parameter only after a software reset or a power cycle (VERIFIED).</summary>
        public bool NeedsReset => true;

        /// <summary>When the new value applies, in words.</summary>
        public string TakesEffect
        {
            get
            {
                if (Skipped)
                {
                    return "nothing new: the unit already holds this value (a value committed earlier applies after a software reset or a power cycle; "
                        + "one left in RAM only by another program or process is NOT saved: write it with force: true)";
                }

                if (Committed && !CommitConfirmed)
                {
                    return "after a software reset or a power cycle IF the commit took - it was NOT confirmed (the unit never showed PARW busy): check the value after a power cycle";
                }

                return Committed
                    ? "after a software reset (SoftwareReset()) or a power cycle"
                    : "RAM only: lost at the next power-up (probably at a software reset too); untested before then";
            }
        }

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["no"] = Number,
            ["name"] = Name,
            ["key"] = Key,
            ["address"] = Address,
            ["unit"] = Unit,
            ["before"] = Before,
            ["after"] = After,
            ["value"] = Value,
            ["raw_before"] = RawBefore,
            ["raw_after"] = RawAfter,
            ["changed"] = Changed,
            ["verified"] = Verified,
            ["committed"] = Committed,
            ["listed_for_model"] = ListedForModel,
            ["verified_for_model"] = VerifiedForModel,
            ["home_parameter"] = HomeParameter,
            ["warnings"] = Warnings.ToList(),
            ["note"] = Note,
            ["commit_confirmed"] = CommitConfirmed,
            ["skipped"] = Skipped,
            ["saved"] = Saved,
            ["needs_reset"] = NeedsReset,
            ["takes_effect"] = TakesEffect,
        };
    }

    /// <summary>Outcome of <see cref="Axis.SoftwareReset"/>: the status once the unit answered again.</summary>
    public sealed record SoftwareResetResult
    {
        /// <summary>The unit answered the reset command with rejection 0.</summary>
        public bool Accepted { get; init; }

        /// <summary>No (complete) answer to the reset command; the reset was confirmed from the status.</summary>
        public bool ReplyLost { get; init; }

        /// <summary>The rate the unit answered at after the reset (<c>null</c>: transport without one).</summary>
        public int? BaudRate { get; init; }

        public bool BaudChanged { get; init; }

        /// <summary>First status after the reset.</summary>
        public Status Status { get; init; } = new Status();

        public double? UptimeBeforeS { get; init; }

        public double? UptimeAfterS { get; init; }

        public double ElapsedS { get; init; }

        public IReadOnlyList<string> Notes { get; init; } = Array.Empty<string>();

        /// <summary>The restart was seen (PMSS 1 -&gt; 0, a reply gap, the uptime restarting or HEND lost).</summary>
        public bool Confirmed { get; init; } = true;

        /// <summary>How, in words.</summary>
        public string ConfirmedBy { get; init; } = string.Empty;

        /// <summary>PMSS after the reset (0 expected on an EC: like a power-up, PIO has control again).</summary>
        public bool ModbusEnabled => Status.ModbusEnabled;

        public bool SafetySpeed => Status.SafetySpeed;

        public bool ServoOn => Status.ServoOn;

        public bool Homed => Status.Homed;

        /// <summary>HEND is 0 after the reset: run <see cref="Axis.Home"/> (after <see cref="Axis.ResetAlarm"/> if an alarm is active).</summary>
        public bool NeedsHome => !Status.Homed;

        public int AlarmCode => Status.AlarmCode;

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["accepted"] = Accepted,
            ["reply_lost"] = ReplyLost,
            ["baudrate"] = BaudRate,
            ["baud_changed"] = BaudChanged,
            ["modbus_enabled"] = ModbusEnabled,
            ["safety_speed"] = SafetySpeed,
            ["servo_on"] = ServoOn,
            ["homed"] = Homed,
            ["needs_home"] = NeedsHome,
            ["alarm_code"] = AlarmCode,
            ["uptime_before_s"] = UptimeBeforeS,
            ["uptime_after_s"] = UptimeAfterS,
            ["elapsed_s"] = ElapsedS,
            ["notes"] = Notes.ToList(),
            ["confirmed"] = Confirmed,
            ["confirmed_by"] = ConfirmedBy,
            ["status"] = AxisJson.Status(Status),
        };
    }

    /// <summary>Outcome of <see cref="Axis.SetBaudRate"/>.</summary>
    public sealed record BaudChangeResult
    {
        /// <summary>Volatile: the unit answers at its power-up rate again after every power-up (VERIFIED on the EC).</summary>
        public const string VolatileNote =
            "volatile: the EC answers at 38400 baud again after every power-up (hardware test 17); a broadcast changes the rate of every unit on the RS-485 line";

        public int Previous { get; init; }

        /// <summary>The rate the port is at now and the unit answered at.</summary>
        public int BaudRate { get; init; }

        public BaudChangeMethod Method { get; init; }

        /// <summary>op1 of the broadcast (IAI baud code).</summary>
        public int Code { get; init; }

        /// <summary>The broadcast frame sent, hex.</summary>
        public string Frame { get; init; } = string.Empty;

        /// <summary>The first read at the new rate was answered.</summary>
        public bool Verified { get; init; }

        /// <summary>That read failed and the rate was found by probing instead.</summary>
        public bool Probed { get; init; }

        public string Note { get; init; } = VolatileNote;

        /// <summary>The rate's code is verified on hardware (<see cref="ModbusRtu.VerifiedBaudRates"/> and <see cref="ModbusRtu.UnitDependentBaudRates"/>).</summary>
        public bool CodeVerified { get; init; } = true;

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["previous"] = Previous,
            ["baudrate"] = BaudRate,
            ["method"] = Method == BaudChangeMethod.Break ? "break" : "direct",
            ["code"] = Code,
            ["frame"] = Frame,
            ["verified"] = Verified,
            ["probed"] = Probed,
            ["note"] = Note,
            ["code_verified"] = CodeVerified,
        };
    }

    /// <summary>One read of the monitor block (0x9000), decoded (<see cref="Axis.ReadMonitor"/>).</summary>
    public sealed record MonitorReading
    {
        public string Units { get; init; } = "mm";

        /// <summary>PNOW (units).</summary>
        public double? Position { get; init; }

        /// <summary>VNOW (units/s; the unit's own measurement, coarsely quantised on an EC).</summary>
        public double? Velocity { get; init; }

        /// <summary>CNOW raw.</summary>
        public int? Current { get; init; }

        /// <summary>CNOW / MCUR 4.5 x 100 when the rated current is readable.</summary>
        public double? CurrentPct { get; init; }

        /// <summary>MCUR 4.5.</summary>
        public uint? RatedCurrent { get; init; }

        /// <summary>DEVI (encoder counts).</summary>
        public int? Deviation { get; init; }

        /// <summary>ALMC.</summary>
        public int Alarm { get; init; }

        public string AlarmName { get; init; } = string.Empty;

        public string AlarmLevel { get; init; } = string.Empty;

        /// <summary>STIM in s.</summary>
        public double? UptimeS { get; init; }

        /// <summary>Every monitor word by name, hex.</summary>
        public IReadOnlyDictionary<string, string> Raw { get; init; } = new Dictionary<string, string>();

        public IReadOnlyList<ushort> Words { get; init; } = Array.Empty<ushort>();

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["units"] = Units,
            ["position"] = Position,
            ["velocity"] = Velocity,
            ["current"] = Current,
            ["current_pct"] = CurrentPct,
            ["rated_current"] = RatedCurrent,
            ["deviation"] = Deviation,
            ["alarm"] = Alarm,
            ["alarm_name"] = AlarmName,
            ["alarm_level"] = AlarmLevel,
            ["uptime_s"] = UptimeS,
            ["raw"] = Raw.ToDictionary(kv => kv.Key, kv => (object?)kv.Value),
            ["words"] = Words.Select(w => (int)w).ToList(),
        };
    }

    /// <summary>
    /// One parameter as <see cref="Axis.Parameters"/> / <see cref="Axis.ReadParameter(string)"/> report it: a named EC user
    /// parameter (<see cref="Family"/> "EC"), a numbered CON parameter ("CON") or a raw class item ("raw").
    /// </summary>
    public sealed record ParameterInfo
    {
        /// <summary>Parameter number (EC P-number or CON No.); 0 for a raw item without a named parameter.</summary>
        public int Number { get; init; }

        public string Name { get; init; } = string.Empty;

        /// <summary>CON symbol (e.g. "PSHV"); <c>null</c> for EC / raw.</summary>
        public string? Symbol { get; init; }

        /// <summary>class.item, e.g. "8.15" (class in hex above 9).</summary>
        public string Item { get; init; } = string.Empty;

        public string? Address { get; init; }

        /// <summary>The value as the PC tool shows it (double for mm values, int for numbers / choices, string for an address).</summary>
        public object? Value { get; init; }

        public string? Text { get; init; }

        public string Unit { get; init; } = string.Empty;

        public double? Min { get; init; }

        public double? Max { get; init; }

        /// <summary>Number -&gt; label.</summary>
        public IReadOnlyDictionary<string, string> Choices { get; init; } = new Dictionary<string, string>();

        /// <summary>A write is accepted (EC: listed, located, not read-only; CON: writable).</summary>
        public bool Writable { get; init; }

        /// <summary>A dangerous parameter (home direction / offset and the like): written with a warning.</summary>
        public bool Dangerous { get; init; }

        /// <summary>The item's raw value, hex.</summary>
        public string? Raw { get; init; }

        /// <summary>"EC", "CON" or "raw".</summary>
        public string Family { get; init; } = "EC";

        /// <summary>Raw item: the unsigned value.</summary>
        public uint? Unsigned { get; init; }

        /// <summary>Raw item: the signed value.</summary>
        public int? Signed { get; init; }

        /// <summary>Raw item: its size in registers.</summary>
        public int? Words { get; init; }

        /// <summary>Raw item: the CON parameter number that lives there, if any.</summary>
        public int? Named { get; init; }

        /// <summary>What the driver knows about the item, or a model note.</summary>
        public string Note { get; init; } = string.Empty;

        /// <summary>JSON-friendly dictionary.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["number"] = Number,
            ["name"] = Name,
            ["symbol"] = Symbol,
            ["item"] = Item,
            ["address"] = Address,
            ["value"] = Value,
            ["text"] = Text,
            ["unit"] = Unit,
            ["min"] = Min,
            ["max"] = Max,
            ["choices"] = Choices.ToDictionary(kv => kv.Key, kv => (object?)kv.Value),
            ["writable"] = Writable,
            ["dangerous"] = Dangerous,
            ["raw"] = Raw,
            ["family"] = Family,
            ["unsigned"] = Unsigned,
            ["signed"] = Signed,
            ["words"] = Words,
            ["named"] = Named,
            ["note"] = Note,
        };
    }

    /// <summary>JSON-friendly views of core records (no I/O).</summary>
    public static class AxisJson
    {
        /// <summary>A <see cref="IaiActuatorDriver.Status"/> with its derived flags.</summary>
        public static IDictionary<string, object?> Status(Status st) => new Dictionary<string, object?>
        {
            ["position_mm"] = st.PositionMm,
            ["alarm_code"] = st.AlarmCode,
            ["servo_on"] = st.ServoOn,
            ["homed"] = st.Homed,
            ["in_position"] = st.InPosition,
            ["moving"] = st.Moving,
            ["alarm"] = st.Alarm,
            ["modbus_enabled"] = st.ModbusEnabled,
            ["emergency_stop"] = st.EmergencyStop,
            ["velocity_mm_s"] = st.VelocityMmS,
            ["current_ma"] = st.CurrentMa,
            ["raw"] = st.Raw.ToDictionary(kv => kv.Key, kv => (object?)(long)kv.Value),
            ["warning"] = st.Warning,
            ["alarm_level"] = st.AlarmLevel.HasValue ? Alarms.LevelKey(st.AlarmLevel.Value) : null,
            ["move_bit"] = st.MoveBit,
            ["homing"] = st.Homing,
            ["paused"] = st.Paused,
            ["target_reached"] = st.TargetReached,
            ["push_missed"] = st.PushMissed,
            ["pushing"] = st.Pushing,
            ["push_contact"] = st.PushContact,
            ["safety_speed"] = st.SafetySpeed,
            ["at_backward_end"] = st.AtBackwardEnd,
            ["at_forward_end"] = st.AtForwardEnd,
            ["at_middle"] = st.AtMiddle,
            ["vnow_mm_s"] = st.VnowMmS,
            ["idle"] = st.Idle,
            ["ready"] = st.Ready,
        };

        /// <summary>A <see cref="IaiActuatorDriver.DeviceInfo"/> with its derived fields (directory records left out).</summary>
        public static IDictionary<string, object?> DeviceInfo(DeviceInfo info) => new Dictionary<string, object?>
        {
            ["slave"] = info.Slave,
            ["system_code"] = info.SystemCode,
            ["system_name"] = info.SystemName,
            ["run_status"] = info.RunStatus,
            ["unit_code"] = info.UnitCode,
            ["unit_name"] = info.UnitName,
            ["firmware"] = info.Firmware,
            ["slave_id_raw"] = info.SlaveIdRaw is null ? null : ModbusRtu.HexText(info.SlaveIdRaw),
            ["serial_number"] = info.SerialNumber,
            ["user_name"] = info.UserName,
            ["model_name"] = info.ModelName,
            ["soft_limits_mm"] = info.SoftLimitsMm is null ? null : new List<object?> { info.SoftLimitsMm.Value.Min, info.SoftLimitsMm.Value.Max },
            ["max_speed_mm_s"] = info.MaxSpeedMmS,
            ["errors"] = info.Errors.ToList(),
            ["identity_source"] = info.IdentitySource,
            ["unit_known"] = info.UnitKnown,
            ["actuator_kind"] = info.ActuatorKind,
            ["actuator_kind_name"] = info.ActuatorKindName,
            ["rotary_atyp"] = info.RotaryAtyp,
            ["axis_number"] = info.AxisNumber,
            ["actuator_serial"] = info.ActuatorSerial,
            ["accl"] = info.Accl,
            ["max_accel_g"] = info.MaxAccelG,
            ["max_decel_g"] = info.MaxDecelG,
            ["speed_limit_source"] = info.SpeedLimitSource,
            ["product_family"] = info.ProductFamily,
            ["controller_name"] = info.ControllerName,
            ["units"] = info.Units,
            ["max_positions"] = info.MaxPositions,
            ["model"] = info.Model,
            ["controller"] = info.Controller,
            ["family"] = info.Family,
            ["actuator"] = info.Actuator,
            ["extensibility_level"] = info.ExtensibilityLevel,
            ["rotary"] = info.Rotary,
            ["kind"] = info.Kind,
        };

        /// <summary>An <see cref="IaiActuatorDriver.AlarmRecord"/> with its derived fields.</summary>
        public static IDictionary<string, object?> AlarmRecord(AlarmRecord r) => new Dictionary<string, object?>
        {
            ["index"] = r.Index,
            ["code"] = r.Code,
            ["hex"] = "0x" + r.Code.ToString("X3", CultureInfo.InvariantCulture),
            ["name"] = r.Name,
            ["detail"] = r.Detail,
            ["address"] = r.Address,
            ["time_ms"] = r.TimeMs,
            ["time_after_power_on"] = r.TimeAfterPowerOn,
            ["power_up"] = r.PowerUp,
            ["level"] = Alarms.LevelKey(r.Level),
        };
    }
}
