using System;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    public sealed partial class Axis
    {
        // ------------------------------------------------------------------ mode / servo / alarm

        /// <summary>
        /// Give Modbus command authority (PMSL coil 0x0427 ON; PIO inputs are then ignored). VERIFIED: DSSE PMSS follows; PMSS
        /// is 0 after every power-up. Nothing is written when PMSS is already 1. Refused (<see cref="EcNotReadyException"/>,
        /// nothing written) while the axis moves, homes or pushes: switching PMSL on mid-move stops the move (VERIFIED). A pause
        /// (DSS1 STP) at a standstill with the servo off is no refusal; a pause with the servo on is (a PLC holding a stop input
        /// on a move). A PCON in PIO mode with nothing on its PIO connector pauses and turns the servo off (VERIFIED on the
        /// PCON-CB, 2026-11-02); 1.0.1 taking control there: VERIFIED on the core from Python (PCON-CB, 2026-11-02);
        /// through this binding UNVERIFIED (tested on the simulator).
        /// </summary>
        public Status? EnableModbusControl(bool verify = true)
        {
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_enable_modbus_control(H, Known.Flag(verify), out st));
            return ToStatus(st);  // none when PMSL was written without verify
        }

        /// <summary>Hand command authority back to PIO (PMSL coil OFF). Refused while the axis moves.</summary>
        public void DisableModbusControl() => Run(() => NativeMethods.iad_axis_modbus_control(H, 0));

        /// <summary>
        /// Turn the servo on / off. Turning off while moving first does a decel stop. Takes Modbus control first if needed (at
        /// rest only). With <paramref name="wait"/> the call returns once DSS1 SV follows (<see cref="EcNotReadyException"/>
        /// after <paramref name="timeout"/> s, <see cref="EcAlarmException"/> on an alarm).
        /// </summary>
        public Status? Servo(bool on, bool wait = true, double timeout = 3.0)
        {
            IadStatus st = default;
            var o = IadServoOptions.Of(wait, timeout);
            Run(() => NativeMethods.iad_axis_servo(H, Known.Flag(on), ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Reset the current alarm (not a software reset) and return the new status. <paramref name="method"/>:
        /// <see cref="CommandMethod.Coil"/> (ALRS coil pulse, VERIFIED: it cleared ABS error 0x0EE on the gripper),
        /// <see cref="CommandMethod.Iai"/> (0x45 cmd 5 op1 0 - frame accepted with no alarm active; clearing a real alarm
        /// UNVERIFIED) or <c>null</c> (the configured strategy's). Takes Modbus control first if needed and the axis is at
        /// rest. A cold-start alarm needs a software reset or a power cycle instead (UNVERIFIED).
        /// </summary>
        public Status ResetAlarm(CommandMethod? method = null)
        {
            var code = MethodFor(method);
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_reset_alarm(H, code, out st));
            return Required(st);
        }

        /// <summary>
        /// Set the action mode. <paramref name="pc"/> true = PC / Modbus control (PMSS = 1), false = PIO (PMSS = 0).
        /// <paramref name="safety"/> is tri-state: <c>null</c> (default) keeps the unit's present safety speed (DSS1 SFTY is
        /// read first), so taking PC mode never switches safety speed off by itself; true / false switch it explicitly. PIO
        /// is 0x45 cmd 0x0C op1 = 0 (VERIFIED on EC #3: (0,0); (0,1) is UNVERIFIED). PC is the PMSL coil (only when PMSS is
        /// 0) and the SFTY coil when the safety speed changes: 0x45 op1 = 1 left a PCON-CB refusing servo ON (VERIFIED,
        /// 2026-11-02); a unit without coils still gets op1 = 1. Refused while the axis moves, pushes, pauses or homes; taking PC control
        /// from PIO allows a pause at a standstill with the servo off, as <see cref="EnableModbusControl"/> does. With
        /// <paramref name="verify"/> the result is read back (<see cref="VerificationException"/> if it did not follow).
        /// </summary>
        public Status SetActionMode(bool pc = true, bool? safety = null, bool verify = true)
        {
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_set_action_mode(H, Known.Flag(pc), Known.Tri(safety), Known.Flag(verify), out st));
            return Required(st);
        }

        /// <summary>
        /// Explicitly switch the safety-speed mode (SFTY coil 0x0401). VERIFIED: ON caps motion at about 100 mm/s; volatile
        /// (off after power-up). The driver never changes this by itself. Takes Modbus control first if needed (at rest).
        /// Raises <see cref="VerificationException"/> if DSS1 bit 14 does not follow.
        /// </summary>
        public Status SetSafetySpeed(bool on, bool verify = true)
        {
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_set_safety_speed(H, Known.Flag(on), Known.Flag(verify), out st));
            return Required(st);
        }

        /// <summary>The home pattern ORG (item 9.16 by item ID; READ-ONLY): 7 on all three units tested; <c>null</c> if the item is missing.</summary>
        public uint? ReadHomePattern()
        {
            var known = 0;
            uint pattern = 0;
            Run(() => NativeMethods.iad_axis_read_home_pattern(H, out known, out pattern));
            return Known.Bool(known) ? pattern : (uint?)null;
        }

        /// <summary>
        /// The power-up baud rate from BRSL (item 8.3 by item ID; READ-ONLY; code 6 = 38400 on all three units tested,
        /// VERIFIED), also kept in <see cref="BootBaudRate"/>. <c>null</c> if missing, not readable or an unknown code.
        /// </summary>
        public int? ReadBootBaudRate()
        {
            var rate = 0;
            Run(() => NativeMethods.iad_axis_read_boot_baud_rate(H, out rate));
            return Known.Int(rate) is int r && r > 0 ? r : (int?)null;
        }

        // ------------------------------------------------------------------ motion

        /// <summary>
        /// Home return, then wait for HEND, PEND and GHMS clear (and the position settled). <paramref name="method"/>:
        /// <see cref="CommandMethod.Coil"/> (HOME coil OFF -&gt; ON edge, released after HEND; VERIFIED),
        /// <see cref="CommandMethod.Iai"/> (0x45 cmd 0x0A with op1 = the home pattern ORG 9.16 &amp; 0xFFFF; VERIFIED on EC #3;
        /// ORG is read first and an invalid one is refused before anything is sent) or <c>null</c> (the configured
        /// strategy's). With <paramref name="wait"/> false the HOME coil stays ON: call <see cref="FinishHome"/> once homed.
        /// An answered refusal means the frame did not run (raised, nothing stopped); a lost answer means it may have run:
        /// the status is watched for 2 s and a home return seen running is waited for, otherwise a decel stop is sent and the
        /// error raised. Rejection 0x24 (a home already running that this call did not start) stops it and raises. On timeout
        /// a decel stop is sent and <see cref="MotionTimeoutException"/> raised.
        /// </summary>
        public Status? Home(bool wait = true, double? timeout = null, CommandMethod? method = null, CancellationToken cancellationToken = default)
        {
            var code = MethodFor(method);
            IadStatus st = default;
            var o = IadHomeOptions.Of(wait, Known.OrNaN(timeout), code);
            Waiting(cancellationToken, () => NativeMethods.iad_axis_home(H, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>Release the HOME coil (coil strategy); needed after <c>Home(wait: false)</c>.</summary>
        public void FinishHome() => Run(() => NativeMethods.iad_axis_finish_home(H));

        /// <summary>
        /// Every check <see cref="MoveTo"/> runs before it sends anything - target, speed, accel / decel and band against
        /// <see cref="Limits"/>, the command-profile block when <paramref name="decelG"/> differs, and the axis ready to move -
        /// with nothing sent. Returns the status read; raises what <see cref="MoveTo"/> would. READ-ONLY.
        /// </summary>
        public Status CheckMove(double position, double speed = 50.0, double accelG = 0.1, double band = 0.1, double? decelG = null, AccelPattern accelPattern = AccelPattern.Default)
        {
            CheckFinite(position, speed, accelG, band, decelG);
            var o = NativeAxis.MoveOptions(speed, accelG, band, true, null, jog: false, Decel(accelG, decelG), accelPattern.ToNative());
            IadStatus st = default;
            Run(() => NativeMethods.iad_axis_check_move(H, position, ref o, out st));
            return Required(st);
        }

        /// <summary>
        /// Absolute move to <paramref name="position"/> (mm, or deg on a rotary axis) at <paramref name="speed"/> (units/s).
        /// Without <paramref name="decelG"/> (or equal to <paramref name="accelG"/>) the direct numeric command is used (FC10,
        /// 9 words at 0x9900, CTLF = 0; one value for acceleration and deceleration). With a different deceleration the
        /// command-profile path: FC17 writes the 20-word profile at 0x0F00 (SPOW / DPOW kept from the unit) and reads 0x9000
        /// x34, then the PCRF coil 0x042E starts it - VERIFIED on the PCON, UNVERIFIED on an EC. Every value is checked
        /// against <see cref="Limits"/> first (<see cref="SoftLimitException"/>, nothing written). A move in progress is
        /// re-targeted. With <paramref name="wait"/> the settled status is returned; on timeout, alarm or cancellation a decel
        /// stop is sent and the error raised. <paramref name="accelPattern"/> picks the acceleration pattern (CTLF b6-b7; UNVERIFIED
        /// on hardware); <paramref name="completion"/> the completion rule of this move (<c>null</c> = the axis's <see cref="Completion"/>).
        /// To start a move and keep its handle instead of waiting see <see cref="StartMoveTo"/>.
        /// </summary>
        public Status? MoveTo(
            double position,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            double? decelG = null,
            CancellationToken cancellationToken = default,
            AccelPattern accelPattern = AccelPattern.Default,
            CompletionRule? completion = null)
        {
            CheckFinite(position, speed, accelG, band, decelG);
            var o = NativeAxis.MoveOptions(speed, accelG, band, wait, timeout, jog: false, Decel(accelG, decelG), accelPattern.ToNative(), completion.ToNative());
            IadStatus st = default;
            Waiting(cancellationToken, () => NativeMethods.iad_axis_move_to(H, position, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Move relative to the measured current position. Sent as an absolute target (CTLF = 0), never as an incremental
        /// move: the unit's base for an incremental move is not verified, and only an absolute target can be checked against
        /// the soft limits exactly. <paramref name="accelPattern"/> and <paramref name="completion"/> as for <see cref="MoveTo"/>.
        /// </summary>
        public Status? MoveBy(
            double delta,
            double speed = 50.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            CancellationToken cancellationToken = default,
            AccelPattern accelPattern = AccelPattern.Default,
            CompletionRule? completion = null)
        {
            if (!IsFinite(delta))
            {
                throw new SoftLimitException($"invalid distance {delta}");
            }

            CheckFinite(null, speed, accelG, band, null);
            var o = NativeAxis.MoveOptions(speed, accelG, band, wait, timeout, jog: false, null, accelPattern.ToNative(), completion.ToNative());
            IadStatus st = default;
            Waiting(cancellationToken, () => NativeMethods.iad_axis_move_by(H, delta, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Jog step: a relative move (<see cref="MoveBy"/>) at a low default speed. The unit's JOG coils (0x0416 / 0x0417) are
        /// never used: they run at about 100 mm/s from a unit parameter, not at the requested speed (VERIFIED).
        /// </summary>
        public Status? Jog(
            double step,
            double speed = 10.0,
            double accelG = 0.1,
            double band = 0.1,
            bool wait = true,
            double? timeout = null,
            CancellationToken cancellationToken = default,
            CompletionRule? completion = null) =>
            MoveBy(step, speed, accelG, band, wait, timeout, cancellationToken, AccelPattern.Default, completion);

        /// <summary>
        /// Push move (VERIFIED): approach <paramref name="target"/> at <paramref name="speed"/>, then push over
        /// <paramref name="band"/>. <paramref name="direction"/> +1 pushes away from home, -1 toward home. The push runs at
        /// <paramref name="forcePct"/> current limit (PPOW = round(pct x 255 / 100)) up to <see cref="PushCapPct"/> (MXPP 7.9,
        /// the hard cap); below the recommended minimum MNPP (7.58, or 20 %) it is allowed with a warning. Refused when the
        /// unit cannot push (SUPT PUSH = 0, MXPP = 0 or TCPC = 0; UNVERIFIED rule), when the axis is not stopped, or when it
        /// is already inside or past the push zone (the unit would skip the approach). The band end must lie inside the
        /// effective stroke. Outcome "contact" (the motor keeps pushing until the next move) or "missed", reported only after
        /// it held for <see cref="PushSettleS"/> with the position unchanged (hardware test G5).
        /// </summary>
        public PushResult? Push(
            double target,
            double band,
            double forcePct,
            int direction = 1,
            double speed = 10.0,
            double accelG = 0.1,
            bool wait = true,
            double? timeout = null,
            CancellationToken cancellationToken = default)
        {
            if (direction != 1 && direction != -1)
            {
                throw new SoftLimitException($"direction must be +1 (away from home) or -1 (toward home), got {direction}");
            }

            if (!IsFinite(forcePct))
            {
                throw new SoftLimitException($"invalid push force {forcePct}");
            }

            CheckFinite(target, speed, accelG, band, null);

            var o = SizedStruct.Fresh<IadPushOptions>();
            NativeErrors.Check(NativeMethods.iad_push_options_init(ref o));
            o.Direction = direction;
            o.Speed = speed;
            o.AccelG = accelG;
            o.Wait = Known.Flag(wait);
            o.TimeoutS = Known.OrNaN(timeout);
            IadPushResult r = default;
            Waiting(cancellationToken, () => NativeMethods.iad_axis_push(H, target, band, forcePct, ref o, out r));
            if (!wait)
            {
                return null;
            }

            var warnings = LastResult().Strings("warnings");
            return new PushResult(r.Outcome == 2 ? PushOutcome.Missed : PushOutcome.Contact, r.PositionMm, Required(r.Status)) { Warnings = warnings };
        }

        /// <summary>
        /// Run stored position <paramref name="n"/> (0 .. <see cref="MaxPositions"/> - 1; on an EC 0 = backward end, 1 =
        /// forward end, 2 = middle) with FC06 0x9800 = n, or - with <paramref name="speedPct"/> (1..100 % of the row's stored
        /// speed) - with 0x45 cmd 6 (op1 = n, op2 = max(1, round(255 x pct / 100)); VERIFIED on the PCON, UNVERIFIED on an EC).
        /// The row is read and checked first (position / push end inside the effective stroke, not incremental, checksum OK;
        /// unless <paramref name="allowAboveCaps"/> its stored speed / accel / push force within the configured caps, else only
        /// the unit's own MAXV / MAXA / MAXD / MXPP). The unit silently ignores 0x9800 while PMSS = 0 (VERIFIED), so Modbus
        /// control is taken first at rest and a move that does not visibly start within 0.5 s is stopped and reported. A push
        /// row is refused while the axis is inside or past its push zone. With <paramref name="wait"/> the settle rules of
        /// <see cref="MoveTo"/> (or of <see cref="Push"/> for a push row) apply.
        /// </summary>
        public Status? MoveToPosition(
            int n,
            double? speedPct = null,
            bool wait = true,
            double? timeout = null,
            bool allowAboveCaps = false,
            CancellationToken cancellationToken = default)
        {
            var o = SizedStruct.Fresh<IadPositionMoveOptions>();
            NativeErrors.Check(NativeMethods.iad_position_move_options_init(ref o));
            o.SpeedPct = Known.OrNaN(speedPct);
            o.Wait = Known.Flag(wait);
            o.TimeoutS = timeout ?? o.TimeoutS;
            o.AllowAboveCaps = Known.Flag(allowAboveCaps);
            IadStatus st = default;
            Waiting(cancellationToken, () => NativeMethods.iad_axis_move_to_position(H, n, ref o, out st));
            return wait ? Required(st) : null;
        }

        /// <summary>
        /// Decel stop with the VERIFIED STOP coil sequence (works with every strategy and with PMSS = 0 on a Modbus-commanded
        /// move): STOP 0x042C ON, hold until the axis has stopped (PNOW unchanged over <see cref="SettleS"/>; at most
        /// <paramref name="timeout"/> s), then OFF again. PMSL is not touched. A Modbus STOP on a move a PLC commanded over PIO
        /// is untested: stop those from the PLC or with the hardware stop. Returns the last status of the hold. If the axis
        /// did not stop in time, <paramref name="wait"/> raises <see cref="MotionTimeoutException"/>; otherwise a warning is
        /// logged. Safe to call from another thread while a move waits.
        /// </summary>
        public Status Stop(bool wait = false, double timeout = 5.0)
        {
            IadStatus st = default;
            var o = IadStopOptions.Of(wait, timeout);
            Run(() => NativeMethods.iad_axis_stop(H, ref o, out st));
            return Required(st);
        }

        /// <summary>
        /// First half of a group stop (<see cref="Cell.StopAll"/>): the STOP coil 0x042C ON, nothing else, recorded so a later
        /// <see cref="Stop"/> (or <see cref="Close"/>) releases it.
        /// </summary>
        internal void SendStopCoil() => Run(() => NativeMethods.iad_axis_begin_stop(H));

        /// <summary>
        /// Decelerate and hold, keeping the remaining travel (STP coil 0x040A ON; VERIFIED). STP is a level input: while it is
        /// ON no move runs and the driver refuses new motion commands.
        /// </summary>
        public void Pause() => Run(() => NativeMethods.iad_axis_pause(H));

        /// <summary>Release the pause; a paused move continues by itself (STP coil OFF; VERIFIED).</summary>
        public void Resume() => Run(() => NativeMethods.iad_axis_resume(H));

        /// <summary>
        /// Block until PEND (or PSFL) is set, |position - target| &lt;= band (if a target is given) and PNOW stayed unchanged
        /// for <see cref="SettleS"/>. Returns the settled status. If the axis settles in position away from
        /// <paramref name="target"/> (stopped by someone else, command ignored, the unit's own clamp),
        /// <see cref="MotionIncompleteException"/> is raised after 0.5 s. On timeout, alarm or cancellation a decel stop is
        /// sent and <see cref="MotionTimeoutException"/> / <see cref="EcAlarmException"/> / <see cref="OperationCanceledException"/> raised.
        /// <paramref name="completion"/> picks the completion rule for this wait (<c>null</c> = the axis's <see cref="Completion"/>).
        /// </summary>
        public Status WaitIdle(double? target = null, double? band = null, double timeout = 10.0, CancellationToken cancellationToken = default, CompletionRule? completion = null)
        {
            IadStatus st = default;
            var o = IadWaitIdleOptions.Of(Known.OrNaN(target), Known.OrNaN(band), timeout, completion is { } rule ? (int)rule : 0);  // 0: the axis's rule
            Waiting(cancellationToken, () => NativeMethods.iad_axis_wait_idle(H, ref o, out st));
            return Required(st);
        }

        // ------------------------------------------------------------------ internals: motion

        /// <summary>The core's code of a per-call command method (<c>null</c>: the strategy's); Iai2 is a strategy only.</summary>
        internal static int MethodFor(CommandMethod? method)
        {
            if (method == CommandMethod.Iai2)
            {
                throw new ArgumentException("method must be Coil or Iai (or null for the strategy's)", nameof(method));
            }

            return NativeAxisOptions.MethodCode(method);
        }

        /// <summary>The .NET check of the motion inputs, before the core sees them: a value that is not a number is refused as before.</summary>
        internal static void CheckFinite(double? target, double speed, double accelG, double band, double? decelG)
        {
            foreach (var (name, value) in new[] { ("target", target), ("speed", speed), ("accel", accelG), ("band", band), ("deceleration", decelG) })
            {
                if (value.HasValue && !IsFinite(value.Value))
                {
                    throw new SoftLimitException($"invalid {name} {value.Value}");
                }
            }
        }

        /// <summary>A deceleration of its own (the command-profile path), or none when it equals the acceleration.</summary>
        internal static double? Decel(double accelG, double? decelG) => decelG.HasValue && decelG.Value != accelG ? decelG : null;
    }
}
