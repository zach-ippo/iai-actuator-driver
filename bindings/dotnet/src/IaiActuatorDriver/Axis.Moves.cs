using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    // Moves that do not block (move handles), changes of a move in progress, sequences, teach and the completion rule.
    public sealed partial class Axis
    {
        // ------------------------------------------------------------------ move handles

        /// <summary>
        /// Start an absolute move without waiting and return its handle: every check of <see cref="MoveTo"/> has passed and the
        /// command has been sent when this returns. Wait for it, poll it or cancel it through the <see cref="MoveHandle"/>;
        /// dispose the handle when done (the move itself is not affected). <paramref name="timeout"/> (s) <c>null</c> = twice
        /// the move's own estimate plus 2 s. <paramref name="decelG"/> (a deceleration of its own), <paramref name="accelPattern"/>
        /// and <paramref name="completion"/> are as for <see cref="MoveTo"/>.
        /// </summary>
        public MoveHandle StartMoveTo(
            double position,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            double? decelG = null,
            AccelPattern accelPattern = AccelPattern.Default,
            CompletionRule? completion = null)
        {
            CheckFinite(position, speed, accelG, band, decelG);
            var o = NativeAxis.MoveOptions(speed, accelG, band, false, timeout, jog: false, Decel(accelG, decelG), accelPattern.ToNative(), completion.ToNative());
            return Started(position, ref o, (double target, ref IadMoveOptions options, out Interop.MoveHandle handle) =>
                NativeMethods.iad_axis_start_move_to(H, target, ref options, out handle));
        }

        /// <summary>Start a move relative to the measured position without waiting (see <see cref="StartMoveTo"/>).</summary>
        public MoveHandle StartMoveBy(
            double delta,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            AccelPattern accelPattern = AccelPattern.Default,
            CompletionRule? completion = null)
        {
            if (!IsFinite(delta))
            {
                throw new SoftLimitException($"invalid distance {delta}");
            }

            CheckFinite(null, speed, accelG, band, null);
            var o = NativeAxis.MoveOptions(speed, accelG, band, false, timeout, jog: false, null, accelPattern.ToNative(), completion.ToNative());
            return Started(delta, ref o, (double step, ref IadMoveOptions options, out Interop.MoveHandle handle) =>
                NativeMethods.iad_axis_start_move_by(H, step, ref options, out handle));
        }

        /// <summary>A small relative step without waiting (see <see cref="Jog"/> and <see cref="StartMoveTo"/>).</summary>
        public MoveHandle StartJog(
            double step,
            double speed = 10.0,
            double accelG = 0.1,
            double band = 0.1,
            double? timeout = null,
            CompletionRule? completion = null)
        {
            if (!IsFinite(step))
            {
                throw new SoftLimitException($"invalid distance {step}");
            }

            CheckFinite(null, speed, accelG, band, null);
            var o = NativeAxis.MoveOptions(speed, accelG, band, false, timeout, jog: true, null, NativeAccelPattern.Default, completion.ToNative());
            return Started(step, ref o, (double delta, ref IadMoveOptions options, out Interop.MoveHandle handle) =>
                NativeMethods.iad_axis_start_jog(H, delta, ref options, out handle));
        }

        private delegate int StartCall(double value, ref IadMoveOptions options, out Interop.MoveHandle handle);

        private MoveHandle Started(double value, ref IadMoveOptions options, StartCall start)
        {
            Interop.MoveHandle? native = null;
            TransportBridge.ClearError();
            var result = start(value, ref options, out native);
            if (result != 0)
            {
                native?.Dispose();
                throw TransportBridge.Raise(result);
            }

            return new MoveHandle(this, NativeMoveHandle.Adopt(_core, 0, native!));
        }

        // ------------------------------------------------------------------ changing a move in progress

        /// <summary>
        /// Change the speed of the 0x9900 move that is running now (the whole block is sent again with the new speed). Refused,
        /// nothing sent, without a move of this axis running (<see cref="EcNotReadyException"/>) and above the speed cap
        /// (<see cref="SoftLimitException"/>). With <paramref name="wait"/> the call returns the settled status once the move
        /// is done (<paramref name="timeout"/> s, <c>null</c> = the move's own estimate); without it <c>null</c> at once.
        /// VERIFIED on the EC-GRBP8M gripper and the PCON-CB.
        /// </summary>
        public Status? ChangeSpeed(double speed, bool wait = false, double? timeout = null, CancellationToken cancellationToken = default)
        {
            if (!IsFinite(speed))
            {
                throw new SoftLimitException($"invalid speed {speed}");
            }

            IadStatus st = default;
            var o = IadChangeOptions.Of(wait, Known.OrNaN(timeout));
            Waiting(cancellationToken, () => NativeMethods.iad_axis_change_speed(H, speed, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Change the acceleration (equal to the deceleration) of the 0x9900 move that is running now. As
        /// <see cref="ChangeSpeed"/>: refused, nothing sent, without a move of this axis running.
        /// </summary>
        public Status? ChangeAccel(double accelG, bool wait = false, double? timeout = null, CancellationToken cancellationToken = default)
        {
            if (!IsFinite(accelG))
            {
                throw new SoftLimitException($"invalid acceleration {accelG}");
            }

            IadStatus st = default;
            var o = IadChangeOptions.Of(wait, Known.OrNaN(timeout));
            Waiting(cancellationToken, () => NativeMethods.iad_axis_change_accel(H, accelG, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Drop the remaining travel of a paused move (<see cref="Pause"/>) and release the pause; returns the status after the
        /// release. Refused, nothing sent, without an active pause (<see cref="EcNotReadyException"/>) or with a fault alarm
        /// (<see cref="EcAlarmException"/>). <paramref name="timeout"/> (s) is how long the stop may take.
        /// </summary>
        public Status CancelPausedMove(double timeout = 5.0)
        {
            IadStatus st = default;
            var o = IadCancelPausedMoveOptions.Of(timeout);
            Run(() => NativeMethods.iad_axis_cancel_paused_move(H, ref o, out st));
            return Required(st);
        }

        /// <summary>
        /// End every blocking call of this axis that runs now on another thread (a move, a jog, a home return, a push, a stored
        /// row, <see cref="WaitIdle"/>, a move handle's wait, the servo wait): each throws <see cref="OperationCanceledException"/>
        /// soon, a motion wait after sending a decel stop. Calls started afterwards are not affected. Sends nothing itself; safe
        /// from any thread.
        /// </summary>
        public void CancelWaits() => Run(() => NativeMethods.iad_axis_cancel(H));

        // ------------------------------------------------------------------ sequences, teach

        /// <summary>
        /// Absolute moves through <paramref name="points"/> in order; one settled status per point. Every point is checked before
        /// the first frame (<see cref="SoftLimitException"/> names the point, nothing sent). Without
        /// <paramref name="blendBand"/> each point completes before the next is sent; with 0.01 .. 10 units the next target is
        /// sent once the axis is within that distance of the current one (UNVERIFIED). <paramref name="timeout"/> (s) is per
        /// point (<c>null</c> = twice the move's estimate plus 2 s).
        /// </summary>
        public IReadOnlyList<Status> MoveSequence(
            IEnumerable<SequencePoint> points,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? blendBand = null,
            CompletionRule? completion = null,
            double? timeout = null,
            CancellationToken cancellationToken = default)
        {
            if (points is null)
            {
                throw new ArgumentNullException(nameof(points));
            }

            var native = points.Select(p => (p ?? throw new ArgumentException("a sequence point is null", nameof(points)))).ToList();
            if (native.Count == 0)
            {
                throw new ArgumentException("no point to move to", nameof(points));
            }

            for (var i = 0; i < native.Count; i++)
            {
                try
                {
                    CheckFinite(native[i].Position, native[i].Speed ?? speed, native[i].AccelG ?? accelG, native[i].Band ?? band, null);
                }
                catch (SoftLimitException exc)
                {
                    throw new SoftLimitException($"point {i}: {exc.Message}");
                }
            }

            var wire = native.Select(p => new NativeSequencePoint(p.Position, p.Speed, p.AccelG, p.Band).ToNative()).ToArray();
            var o = SizedStruct.Fresh<IadSequenceOptions>();
            NativeErrors.Check(NativeMethods.iad_sequence_options_init(ref o));
            o.Speed = speed;
            o.AccelG = accelG;
            o.Band = band;
            o.BlendBand = Known.OrNaN(blendBand);
            o.TimeoutS = Known.OrNaN(timeout);
            o.Completion = (int)completion.ToNative();
            var statuses = new IadStatus[wire.Length];
            var count = UIntPtr.Zero;
            Waiting(cancellationToken, () =>
                NativeMethods.iad_axis_move_sequence(H, wire, NativeText.Size(wire.Length), ref o, statuses, NativeText.Size(statuses.Length), out count));
            var done = Math.Min(NativeText.ToInt(count), statuses.Length);
            var result = new List<Status>(done);
            for (var i = 0; i < done; i++)
            {
                result.Add(new NativeStatus(statuses[i]).ToRecord());
            }

            return result;
        }

        /// <summary>
        /// <see cref="MoveSequence(IEnumerable{SequencePoint}, double, double, double, double?, CompletionRule?, double?, CancellationToken)"/>
        /// through plain positions, each with the sequence's speed, acceleration and band.
        /// </summary>
        public IReadOnlyList<Status> MoveSequence(
            IEnumerable<double> positions,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            double? blendBand = null,
            CompletionRule? completion = null,
            double? timeout = null,
            CancellationToken cancellationToken = default) =>
            MoveSequence(
                (positions ?? throw new ArgumentNullException(nameof(positions))).Select(p => new SequencePoint(p)),
                speed,
                accelG,
                band,
                blendBand,
                completion,
                timeout,
                cancellationToken);

        /// <summary>
        /// Store the current position as stored row <paramref name="n"/> (the row is read, its position replaced and written back
        /// with its checksum). Non-volatile; <paramref name="allowNonvolatile"/> and <paramref name="allowUnsafe"/> are
        /// accepted and ignored (no longer needed). A row the EC's position mode does not run is taught with a warning;
        /// a row that already holds the position is not written
        /// (<see cref="PositionEntry.Skipped"/>) unless <paramref name="force"/>. <paramref name="viaCoils"/> teaches with the
        /// MOD / POSR / TEAC coil sequence instead (EXPERIMENTAL; needs coils). Returns the row read back.
        /// </summary>
        public PositionEntry TeachPosition(int n, bool allowNonvolatile = false, bool viaCoils = false, bool allowUnsafe = false, bool force = false)
        {
            var o = NvOptions(allowNonvolatile, allowUnsafe, force);
            IadPositionWriteResult r = default;
            Run(() => NativeMethods.iad_axis_teach_position(H, n, viaCoils ? (int)NativeTeachMethod.Coil : (int)NativeTeachMethod.Table, ref o, out r));
            var row = new NativePositionEntry(r).ToRecord();
            return Known.Bool(r.Skipped) ? row with { Skipped = true } : row;
        }

        // ------------------------------------------------------------------ lean status, line, caps

        /// <summary>
        /// The lean status: one read of 0x9000-0x9007 (PNOW, ALMC, DIPM, DOPM, DSS1, DSS2, DSSE), the 8 words the status monitor and the
        /// fast completion use (no current or uptime; no velocity while moving). READ-ONLY. <paramref name="retries"/> <c>null</c> =
        /// the port's.
        /// </summary>
        public Status ReadLeanStatus(int? retries = null)
        {
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_read_lean_status(H, retries ?? int.MinValue, out st));
            return new NativeStatus(st).ToRecord();
        }

        /// <summary>The line the unit sits on, by name ("rcon", "robonet"...); <c>null</c> on a direct connection.</summary>
        public string? LineName
        {
            get
            {
                var name = Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_line_name(H, b, s, out r));
                return name.Length == 0 ? null : name;
            }
        }

        /// <summary>How <see cref="Stop"/> stops this axis: "coil" (the STOP coil sequence) or the register path behind an RCON gateway.</summary>
        public string StopPath => Text((byte[]? b, UIntPtr s, out UIntPtr r) => NativeMethods.iad_axis_stop_path(H, b, s, out r));

        /// <summary>
        /// The documented minimum speed (units/s) and where it comes from, or why it is unknown (a speed below it is allowed with a warning,
        /// never refused). May read MAXV / MAXA / MAXD / MXPP once. READ-ONLY.
        /// </summary>
        public (double? Speed, string Source) RecommendedMinSpeed()
        {
            IadMotionCaps caps = default;
            Run(() => NativeMethods.iad_axis_motion_caps(H, out caps));
            var native = new NativeMotionCaps(caps);
            return (native.MinSpeed, native.MinSpeedSource);
        }

        // ------------------------------------------------------------------ completion rule

        /// <summary>
        /// When this axis's moves count as done by default: <see cref="CompletionRule.Safe"/> (PEND, the position within the
        /// band and PNOW unchanged over <see cref="SettleS"/>) or <see cref="CompletionRule.Fast"/> (one sample: PEND, within
        /// the band, DSSE MOVE clear; what <see cref="EnableFast"/> sets). A move can pick its own rule.
        /// </summary>
        public CompletionRule Completion
        {
            get
            {
                var o = SizedStruct.Fresh<IadAxisOptions>();
                NativeErrors.Check(NativeMethods.iad_axis_get_options(H, ref o));
                return o.Completion == (int)NativeCompletion.Fast ? CompletionRule.Fast : CompletionRule.Safe;
            }

            set
            {
                lock (_sync)
                {
                    _completion = value;
                }

                try
                {
                    ApplyOptions();
                }
                finally
                {
                    lock (_sync)
                    {
                        _completion = null;
                    }
                }
            }
        }
    }
}
