using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace IaiActuatorDriver
{
    /// <summary>
    /// Decoded monitor block (0x9000...). <see cref="Raw"/> holds the register words by name (32-bit values combined, high
    /// word first).
    /// </summary>
    /// <remarks>
    /// <see cref="Moving"/> is "the axis is in motion": DSSE MOVE, a home return in progress, or servo on without PEND /
    /// PSFL (and not paused). The raw MOVE bit (<see cref="MoveBit"/>) alone is not reliable on the EC: it clears while
    /// still decelerating and during the homing creep (VERIFIED). <see cref="VelocityMmS"/> is 0 when not moving; while
    /// moving it is VNOW (0.01 mm/s, but quantized in about 12.5 mm/s steps on the EC; <c>null</c> if only the short block
    /// was read) - the axis class replaces it with a speed derived from successive PNOW samples. <see cref="CurrentMa"/> is
    /// CNOW as documented for CON controllers (DOC).
    /// </remarks>
    public sealed record Status
    {
        private static readonly IReadOnlyDictionary<string, uint> EmptyRaw = new ReadOnlyDictionary<string, uint>(new Dictionary<string, uint>());

        /// <summary>PNOW in mm (0.01 mm counts, signed).</summary>
        public double PositionMm { get; init; }

        /// <summary>ALMC: the present alarm code, 0 = none.</summary>
        public int AlarmCode { get; init; }

        /// <summary>DSS1 SV.</summary>
        public bool ServoOn { get; init; }

        /// <summary>DSS1 HEND: home return complete.</summary>
        public bool Homed { get; init; }

        /// <summary>DSS1 PEND: inside the positioning band.</summary>
        public bool InPosition { get; init; }

        /// <summary>The axis is in motion (see the remarks).</summary>
        public bool Moving { get; init; }

        /// <summary>A fault that blocks motion: ALMH, or a code of level operation cancelled / cold start (or unknown without ALML).</summary>
        public bool Alarm { get; init; }

        /// <summary>DSSE PMSS: PIO disabled, Modbus has control.</summary>
        public bool ModbusEnabled { get; init; }

        /// <summary>DSS1 EMGS: emergency stop active.</summary>
        public bool EmergencyStop { get; init; }

        /// <summary>0 when not moving; VNOW (coarse) while moving; <c>null</c> when only the short block was read.</summary>
        public double? VelocityMmS { get; init; }

        /// <summary>CNOW (signed), <c>null</c> when only the short block was read.</summary>
        public int? CurrentMa { get; init; }

        /// <summary>The words read, by name: PNOW, ALMC, DIPM, DOPM, DSS1, DSS2, DSSE (+ STAT, VNOW, CNOW with 14 words; + DEVI, STIM, SIPM, ZONS with 20).</summary>
        public IReadOnlyDictionary<string, uint> Raw { get; init; } = EmptyRaw;

        /// <summary>
        /// A message-level alarm is active (a warning: the axis keeps running), not a fault: <see cref="Alarm"/> is false and
        /// either the code is of the message level or DSS1 b9 ALML is set.
        /// </summary>
        public bool Warning =>
            !Alarm && ((RawWord("DSS1") & Registers.DSS1_ALML) != 0 || (AlarmCode != 0 && Alarms.AlarmLevelOf(AlarmCode) == IaiActuatorDriver.AlarmLevel.Message));

        /// <summary>The level of the present alarm code, <c>null</c> when no code is set.</summary>
        public AlarmLevel? AlarmLevel => AlarmCode != 0 ? Alarms.AlarmLevelOf(AlarmCode) : (AlarmLevel?)null;

        /// <summary>Same as <see cref="Alarm"/>.</summary>
        public bool Fault => Alarm;

        /// <summary>Raw DSSE MOVE bit (clears early during deceleration; prefer <see cref="Moving"/>).</summary>
        public bool MoveBit => (RawWord("DSSE") & Registers.DSSE_MOVE) != 0;

        /// <summary>Home return in progress (DSSE GHMS, still set during the final creep).</summary>
        public bool Homing => (RawWord("DSSE") & Registers.DSSE_GHMS) != 0;

        /// <summary>Pause (STP) active (DSS1 bit 5).</summary>
        public bool Paused => (RawWord("DSS1") & Registers.DSS1_STP) != 0;

        /// <summary>DSS1 bit 0: the last commanded move ended at its target (0 after a STOP).</summary>
        public bool TargetReached => (RawWord("DSS1") & Registers.DSS1_TARGET_REACHED) != 0;

        /// <summary>DSS1 bit 11 (PSFL): the last push went over its whole band without contact.</summary>
        public bool PushMissed => (RawWord("DSS1") & Registers.DSS1_PSFL) != 0;

        /// <summary>DSSE bit 10: push in progress, or holding after contact.</summary>
        public bool Pushing => (RawWord("DSSE") & Registers.DSSE_PUSH) != 0;

        /// <summary>
        /// The contact fingerprint (PEND and DSSE bit 0) in this one sample. One sample is not proof of contact: a push that
        /// then missed showed it for a moment while still moving (VERIFIED, hardware test G5).
        /// </summary>
        public bool PushContact => InPosition && (RawWord("DSSE") & Registers.DSSE_PUSH_CONTACT) != 0;

        /// <summary>DSS1 bit 14 (SFTY): safety speed enabled on the unit.</summary>
        public bool SafetySpeed => (RawWord("DSS1") & Registers.DSS1_SFTY) != 0;

        /// <summary>
        /// ZONS bit 12 (LS0): within the LS range of position row 0, the backward end; <c>null</c> if ZONS was not read.
        /// VERIFIED on the 3-position slider and the 2-position gripper. The LS bits follow the position, also while moving;
        /// for a push row the marker is at its push end. DOPM is not used: its bit layout differs between models.
        /// </summary>
        public bool? AtBackwardEnd => Zons(Registers.ZONS_LS0);

        /// <summary>ZONS bit 13 (LS1): within the LS range of row 1 (a push row: its push end); <c>null</c> if not read.</summary>
        public bool? AtForwardEnd => Zons(Registers.ZONS_LS1);

        /// <summary>ZONS bit 14 (LS2): within the LS range of row 2 (also on the 2-position gripper); <c>null</c> if not read.</summary>
        public bool? AtMiddle => Zons(Registers.ZONS_LS2);

        /// <summary>VNOW in mm/s (coarse), if the long status block was read.</summary>
        public double? VnowMmS => Raw.TryGetValue("VNOW", out var vnow)
            ? Math.Round(unchecked((int)vnow) * Registers.VELOCITY_SCALE, 2)
            : (double?)null;

        /// <summary>Not moving, not paused, not homing: safe for relative decisions and non-volatile writes.</summary>
        public bool Idle => !Moving && !Paused && !Homing;

        /// <summary>True when a move would be accepted (as far as the status bits tell).</summary>
        public bool Ready => ServoOn && Homed && ModbusEnabled && !Alarm && !EmergencyStop && !Paused && !Homing;

        /// <summary>
        /// Decode words read from 0x9000: 8 (PNOW..DSSE), 14 (..CNOW) or 20 (..ZONS, the default read). The raw DOPM and
        /// ZONS words are kept in <see cref="Raw"/>; the position markers come from ZONS only and are <c>null</c> when
        /// fewer than 20 words were read.
        /// </summary>
        public static Status FromRegisters(IReadOnlyList<ushort> words)
        {
            if (words is null)
            {
                throw new ArgumentNullException(nameof(words));
            }

            if (words.Count < Registers.STATUS_BLOCK_MIN_WORDS)
            {
                throw new ArgumentException($"need at least {Registers.STATUS_BLOCK_MIN_WORDS} words", nameof(words));
            }

            int alarmCode = words[2];
            int dss1 = words[5];
            int dss2 = words[6];
            int dsse = words[7];
            var raw = new Dictionary<string, uint>
            {
                ["PNOW"] = Registers.FromU32Words(words[0], words[1]),
                ["ALMC"] = (uint)alarmCode,
                ["DIPM"] = words[3],
                ["DOPM"] = words[4],
                ["DSS1"] = (uint)dss1,
                ["DSS2"] = (uint)dss2,
                ["DSSE"] = (uint)dsse,
            };
            int? current = null;
            double? vnow = null;
            if (words.Count >= 14)
            {
                raw["STAT"] = Registers.FromU32Words(words[8], words[9]);
                raw["VNOW"] = Registers.FromU32Words(words[10], words[11]);
                raw["CNOW"] = Registers.FromU32Words(words[12], words[13]);
                vnow = Math.Round(Registers.FromI32Words(words[10], words[11]) * Registers.VELOCITY_SCALE, 2);
                current = Registers.FromI32Words(words[12], words[13]);
            }

            if (words.Count >= Registers.STATUS_BLOCK_WORDS)
            {
                raw["DEVI"] = Registers.FromU32Words(words[14], words[15]);
                raw["STIM"] = Registers.FromU32Words(words[16], words[17]);
                raw["SIPM"] = words[18];
                raw["ZONS"] = words[19];
            }

            // Fault rule (Alarms.AlarmIsFault): ALMH, or a code of level operation cancelled / cold start. A message-level
            // code is a warning - the axis keeps running - and never blocks motion. An unknown-level code without the
            // warning bit ALML is a fault (fail safe).
            var alarm = Alarms.AlarmIsFault(alarmCode, (dss1 & Registers.DSS1_ALMH) != 0, (dss1 & Registers.DSS1_ALML) != 0);
            var servoOn = (dss1 & Registers.DSS1_SV) != 0;

            // "Settled" states in which PEND legitimately stays 0 fall back to the raw bits. A warning does not settle the
            // axis: it keeps running.
            var settled = !servoOn
                || (dss1 & (Registers.DSS1_PEND | Registers.DSS1_PSFL | Registers.DSS1_STP)) != 0
                || alarm
                || (dss1 & Registers.DSS1_HEND) == 0;
            var moving = (dsse & (Registers.DSSE_MOVE | Registers.DSSE_GHMS)) != 0 || !settled;
            if (!moving)
            {
                vnow = 0.0;
            }

            return new Status
            {
                PositionMm = Math.Round(Registers.FromI32Words(words[0], words[1]) * Registers.POSITION_SCALE, 2),
                AlarmCode = alarmCode,
                ServoOn = servoOn,
                Homed = (dss1 & Registers.DSS1_HEND) != 0,
                InPosition = (dss1 & Registers.DSS1_PEND) != 0,
                Moving = moving,
                Alarm = alarm,
                ModbusEnabled = (dsse & Registers.DSSE_PMSS) != 0,
                EmergencyStop = (dss1 & Registers.DSS1_EMGS) != 0,
                VelocityMmS = vnow,
                CurrentMa = current,
                Raw = new ReadOnlyDictionary<string, uint>(raw),
            };
        }

        private uint RawWord(string name) => Raw.TryGetValue(name, out var value) ? value : 0u;

        private bool? Zons(int bit) => Raw.TryGetValue("ZONS", out var zons) ? (zons & bit) != 0 : (bool?)null;
    }
}
