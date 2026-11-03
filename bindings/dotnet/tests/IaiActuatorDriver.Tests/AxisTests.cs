using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Frame helpers for the axis tests.</summary>
    internal static class Frames
    {
        public static List<byte[]> Of(FakeTransport t, byte function) => t.Writes.Where(f => f.Length > 1 && f[1] == function).ToList();

        public static List<(int Address, bool On)> Coils(FakeTransport t) =>
            Of(t, 0x05).Select(f => (((f[2] << 8) | f[3]), f[4] == 0xFF)).ToList();

        public static List<(int Address, ushort[] Words)> Writes(FakeTransport t) =>
            Of(t, 0x10).Select(f => (((f[2] << 8) | f[3]), Enumerable.Range(0, (f[4] << 8) | f[5]).Select(i => (ushort)((f[7 + (2 * i)] << 8) | f[8 + (2 * i)])).ToArray())).ToList();

        public static List<byte[]> Iai(FakeTransport t) => Of(t, 0x45);

        /// <summary>The PDU of a frame (without slave and CRC), hex.</summary>
        public static string Pdu(byte[] frame) => ModbusRtu.HexText(frame.Skip(1).Take(frame.Length - 3)).ToUpperInvariant();
    }

    public class AxisIdentityAndLimitsTests
    {
        [Fact]
        public void Identify_reads_identity_and_the_units_own_ranges()
        {
            var sim = new SimUnit();
            var axis = sim.Axis();
            var info = axis.Identify();
            Assert.Equal("EC-R6S-150-1-PN", info.Model);
            Assert.Equal((-0.30, 150.30), info.SoftLimitsMm!.Value);
            Assert.Equal(800.0, info.MaxSpeedMmS);
            Assert.Equal(1.0, info.MaxAccelG);
            Assert.Equal(1.0, info.MaxDecelG);
            Assert.Equal(3, info.ActuatorKind);
            Assert.Equal("rod", info.Kind);
            Assert.Equal("mm", info.Units);
            Assert.Equal(3, info.MaxPositions);
            Assert.Equal("EC", info.ProductFamily);
            Assert.StartsWith("MAXV item 7.4", info.SpeedLimitSource);
        }

        [Fact]
        public void Limits_come_from_the_unit()
        {
            var axis = new SimUnit().Axis();
            var limits = axis.Limits();
            Assert.Equal("mm", limits.Units);
            Assert.Equal("rod", limits.Kind);
            Assert.Equal(0.0, limits.Position.Min);
            Assert.Equal(150.0, limits.Position.Max);
            Assert.Equal(800.0, limits.Speed.Max);
            Assert.Equal(800.0, limits.Speed.UnitMax);
            Assert.Null(limits.Speed.Configured);
            Assert.Equal(1.0, limits.AccelDecel.Max);
            Assert.True(limits.Push.Available);
            Assert.Equal(70.0, limits.Push.Max);
            Assert.Equal(20.0, limits.Push.RecommendedMin);
            Assert.Equal(3, limits.StoredRows.Count);
            Assert.Equal(150.0, limits.JogStep.Max);
            var json = JsonSerializer.Serialize(limits.ToDictionary());
            Assert.Contains("\"speed_override\"", json);
        }

        [Fact]
        public void A_configured_cap_only_narrows()
        {
            var axis = new SimUnit().Axis(SimUnit.Fast with { MaxSpeedMmS = 100, MaxAccelG = 0.3, MaxPushPct = 50, Stroke = (10.0, 500.0) });
            var limits = axis.Limits();
            Assert.Equal(100.0, limits.Speed.Max);
            Assert.Equal(0.3, limits.AccelDecel.Max);
            Assert.Equal(50.0, limits.Push.Max);
            Assert.Equal(10.0, limits.Position.Min);
            Assert.Equal(150.0, limits.Position.Max);
        }

        [Fact]
        public void Options_are_validated()
        {
            var (bus, _) = new SimUnit().Bus();
            Assert.Throws<ArgumentOutOfRangeException>(() => new Axis(bus, 1, new AxisOptions { MaxSpeedMmS = 0 }));
            Assert.Throws<ArgumentOutOfRangeException>(() => new Axis(bus, 1, new AxisOptions { MaxAccelG = 3.5 }));
            Assert.Throws<ArgumentOutOfRangeException>(() => new Axis(bus, 1, new AxisOptions { MaxPushPct = 101 }));
            Assert.Throws<ArgumentException>(() => new Axis(bus, 1, new AxisOptions { Stroke = (5.0, 5.0) }));
        }

        [Fact]
        public void A_rotary_unit_works_in_degrees()
        {
            var sim = new SimUnit();
            sim.PutU32(0x0700 + (2 * 23), 1); // AKID 7.33 = 1 (rotary)
            var axis = sim.Axis();
            Assert.Equal("deg", axis.Units);
            Assert.Equal("rotary", axis.Kind);
            Assert.Equal("deg/s", axis.Limits().Speed.Unit);
            Assert.Contains(axis.Identify().Errors, e => e.StartsWith("AKID = 1 (rotary)", StringComparison.Ordinal));
        }

        [Fact]
        public void Status_derives_the_velocity_and_monitor_reads_the_whole_block()
        {
            var sim = new SimUnit().Ready(10.0);
            var axis = sim.Axis();
            var st = axis.Status();
            Assert.Equal(10.0, st.PositionMm);
            Assert.True(st.Ready);
            var m = axis.ReadMonitor();
            Assert.Equal(10.0, m.Position);
            Assert.Equal(0, m.Alarm);
            Assert.NotNull(m.UptimeS);
            Assert.True(m.Words.Count >= 20);
        }

        [Fact]
        public void Alarm_history_skips_empty_slots_newest_first()
        {
            var history = new SimUnit().Axis().ReadAlarmHistory();
            Assert.Equal(4, history.Count);
            Assert.True(history[0].PowerUp);
            Assert.Equal(0x0C1, history[1].Code);
            Assert.Null(history[0].TimeMs);
            Assert.Equal(32, new SimUnit().Axis().ReadAlarmHistory(includeEmpty: true).Count);
        }

        [Fact]
        public void Unit_info_is_json_serialisable_and_read_only()
        {
            var sim = new SimUnit().Ready(5.0);
            var (bus, transport) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var info = axis.UnitInfo();
            Assert.Empty((List<object?>)info["errors"]!);
            var json = JsonSerializer.Serialize(info);
            Assert.Contains("EC-R6S-150-1-PN", json);
            Assert.Contains("\"stored_rows\":3", json);
            Assert.All(transport.Writes, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));
            Assert.Throws<ArgumentException>(() => axis.UnitInfo(new[] { "nope" }));
        }
    }

    public class AxisMotionTests
    {
        private static (SimUnit Sim, Axis Axis, FakeTransport T) Ready(double position = 0.0, AxisOptions? options = null)
        {
            var sim = new SimUnit().Ready(position);
            var (bus, transport) = sim.Bus();
            return (sim, new Axis(bus, 1, options ?? SimUnit.Fast), transport);
        }

        [Fact]
        public void MoveTo_writes_the_full_9900_block_and_returns_the_settled_status()
        {
            var (sim, axis, t) = Ready();
            var st = axis.MoveTo(50.0, speed: 20, accelG: 0.2, band: 0.1);
            Assert.NotNull(st);
            Assert.Equal(50.0, st!.PositionMm);
            Assert.True(st.InPosition);
            var write = Frames.Writes(t).Single(w => w.Address == Registers.DIRECT_MOVE_BASE);
            Assert.Equal(new ushort[] { 0, 5000, 0, 10, 0, 2000, 20, 0, 0 }, write.Words); // CTLF always stated (0)
            Assert.Equal(50.0, sim.Position);
        }

        [Fact]
        public void Out_of_range_inputs_send_nothing()
        {
            var (_, axis, t) = Ready();
            axis.Limits();
            var before = t.Writes.Count;
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(150.01));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(-0.01));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(50, speed: 800.5));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(50, accelG: 1.01));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(50, band: 10.5));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(double.NaN));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(50, speed: 0));
            Assert.DoesNotContain(t.Writes.Skip(before), f => f[1] != 0x03);
            Assert.Equal(150.0, axis.MoveTo(150.0)!.PositionMm); // limits are inclusive
        }

        [Fact]
        public void Preconditions_are_checked_before_moving()
        {
            var (sim, axis, _) = Ready();
            sim.Homed = false;
            Assert.Throws<EcNotReadyException>(() => axis.MoveTo(10));
            sim.Homed = true;
            sim.Servo = false;
            Assert.Throws<EcNotReadyException>(() => axis.MoveTo(10));
            sim.Servo = true;
            sim.AlarmCode = 0x0C1;
            sim.AlarmBit = true;
            Assert.Throws<EcAlarmException>(() => axis.MoveTo(10));
            sim.AlarmCode = 0;
            sim.AlarmBit = false;
            sim.Stp = true;
            Assert.Throws<EcNotReadyException>(() => axis.MoveTo(10));
        }

        [Fact]
        public void Modbus_control_is_taken_at_rest_before_the_first_move()
        {
            var (sim, axis, t) = Ready();
            sim.Pmss = false;
            axis.MoveTo(20.0);
            var coils = Frames.Coils(t);
            Assert.Contains((Registers.COIL_PMSL, true), coils);
            Assert.True(sim.Pmss);

            sim.Pmss = false;
            axis.AutoModbusControl = false;
            Assert.Throws<EcNotReadyException>(() => axis.MoveTo(30.0));
        }

        [Fact]
        public void MoveBy_and_Jog_are_absolute_targets_never_jog_coils()
        {
            var (sim, axis, t) = Ready(40.0);
            axis.MoveBy(-10.0);
            Assert.Equal(30.0, sim.Position);
            axis.Jog(2.5);
            Assert.Equal(32.5, sim.Position);
            Assert.DoesNotContain(Frames.Coils(t), c => c.Address == Registers.COIL_JOG_PLUS || c.Address == Registers.COIL_JOG_MINUS);
            Assert.All(Frames.Writes(t).Where(w => w.Address == Registers.DIRECT_MOVE_BASE), w => Assert.Equal(0, w.Words[8]));
        }

        [Fact]
        public void A_profile_move_uses_FC17_and_the_PCRF_coil()
        {
            var (sim, axis, t) = Ready();
            var st = axis.MoveTo(60.0, speed: 30, accelG: 0.3, decelG: 0.2);
            Assert.Equal(60.0, st!.PositionMm);
            Assert.Single(Frames.Of(t, 0x17));
            Assert.Contains((Registers.COIL_PCRF, true), Frames.Coils(t));
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(10.0, accelG: 0.3, decelG: 1.5)); // MAXD 1.00
        }

        [Fact]
        public void Stop_holds_the_STOP_coil_until_standstill_then_releases_it()
        {
            var (_, axis, t) = Ready();
            axis.Stop();
            var stops = Frames.Coils(t).Where(c => c.Address == Registers.COIL_STOP).ToList();
            Assert.Equal(new[] { (Registers.COIL_STOP, true), (Registers.COIL_STOP, false) }, stops);
        }

        [Fact]
        public void A_timeout_stops_the_axis_and_raises()
        {
            var (sim, axis, t) = Ready();
            sim.StepMm = 0.01;
            var exc = Assert.Throws<MotionTimeoutException>(() => axis.MoveTo(100.0, timeout: 0.1));
            Assert.True(exc.StopAttempted);
            Assert.Contains(exc.Notes, n => n.StartsWith("decel stop sent", StringComparison.Ordinal));
            var stops = Frames.Coils(t).Where(c => c.Address == Registers.COIL_STOP).ToList();
            Assert.Equal(new[] { (Registers.COIL_STOP, true), (Registers.COIL_STOP, false) }, stops);
        }

        [Fact]
        public void An_alarm_during_motion_stops_and_raises()
        {
            var (sim, axis, _) = Ready();
            sim.StepMm = 5;
            sim.AlarmDuringMove = 0x0C1;
            sim.AlarmAtMm = 20;
            var exc = Assert.Throws<EcAlarmException>(() => axis.MoveTo(100.0));
            Assert.Equal(0x0C1, exc.AlarmCode);
            Assert.True(exc.StopAttempted);
        }

        [Fact]
        public void A_stop_from_another_thread_ends_the_wait_as_incomplete()
        {
            var (sim, axis, _) = Ready();
            sim.StepMm = 0.5;
            var move = Task.Run(() => axis.MoveTo(140.0, timeout: 20));
            Assert.True(SpinWait.SpinUntil(() => sim.IsMoving, 5000));
            axis.Stop();
            var exc = Assert.Throws<AggregateException>(() => move.Wait(TimeSpan.FromSeconds(10)));
            Assert.IsType<MotionIncompleteException>(exc.InnerException);
            Assert.True(sim.Position < 140.0);
        }

        [Fact]
        public void Cancellation_stops_the_axis()
        {
            var (sim, axis, t) = Ready();
            sim.StepMm = 0.05;
            using var cts = new CancellationTokenSource(TimeSpan.FromMilliseconds(80));
            Assert.ThrowsAny<OperationCanceledException>(() => axis.MoveTo(140.0, cancellationToken: cts.Token));
            Assert.Contains((Registers.COIL_STOP, true), Frames.Coils(t));
            Assert.False(sim.IsMoving);
        }

        [Fact]
        public void Pause_and_resume_use_the_STP_coil()
        {
            var (sim, axis, t) = Ready();
            axis.Pause();
            Assert.True(sim.Stp);
            Assert.Throws<EcNotReadyException>(() => axis.MoveTo(10.0));
            axis.Resume();
            Assert.False(sim.Stp);
            Assert.Equal(new[] { (Registers.COIL_STP, true), (Registers.COIL_STP, false) }, Frames.Coils(t));
        }

        [Fact]
        public void Push_contact_writes_CTLF_and_PPOW()
        {
            var (sim, axis, t) = Ready();
            sim.ContactAt = 42.0;
            var result = axis.Push(30.0, band: 20.0, forcePct: 30);
            Assert.NotNull(result);
            Assert.True(result!.Contact);
            Assert.Equal(42.0, result.PositionMm);
            Assert.Empty(result.Warnings);
            var w = Frames.Writes(t).Single(x => x.Address == Registers.DIRECT_MOVE_BASE).Words;
            Assert.Equal(2000, w[3]); // INP = the push band
            Assert.Equal(76, w[7]); // 30 % -> 76.5 -> 76 (half to even)
            Assert.Equal(Registers.CTLF_PUSH | Registers.CTLF_PUSH_AWAY, w[8]);
        }

        [Fact]
        public void Push_missed_toward_home_and_low_force_warns()
        {
            var (_, axis, t) = Ready(100.0);
            var result = axis.Push(60.0, band: 10.0, forcePct: 10, direction: -1);
            Assert.True(result!.Missed);
            Assert.Equal(50.0, result.PositionMm);
            Assert.Contains(result.Warnings, x => x.Contains("below this unit's recommended minimum 20 %"));
            Assert.Equal(Registers.CTLF_PUSH, Frames.Writes(t).Single(x => x.Address == Registers.DIRECT_MOVE_BASE).Words[8]);
        }

        [Fact]
        public void Push_guards()
        {
            var (_, axis, _) = Ready(35.0);
            Assert.Throws<SoftLimitException>(() => axis.Push(30.0, 10.0, 71)); // MXPP 70
            Assert.Throws<SoftLimitException>(() => axis.Push(30.0, 10.0, 30, direction: 2));
            Assert.Throws<SoftLimitException>(() => axis.Push(145.0, 10.0, 30)); // band end 155 outside the stroke
            Assert.Throws<EcNotReadyException>(() => axis.Push(30.0, 10.0, 30)); // inside the zone
        }

        [Fact]
        public void Servo_and_alarm_reset_coils()
        {
            var sim = new SimUnit();
            sim.Pmss = true;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.True(axis.Servo(true)!.ServoOn);
            sim.AlarmCode = 0x0C1;
            sim.AlarmBit = true;
            Assert.Equal(0, axis.ResetAlarm().AlarmCode);
            Assert.False(axis.Servo(false)!.ServoOn);
            Assert.Equal(
                new[] { (Registers.COIL_SON, true), (Registers.COIL_ALRS, true), (Registers.COIL_ALRS, false), (Registers.COIL_SON, false) },
                Frames.Coils(t));
            Assert.Throws<ArgumentException>(() => axis.ResetAlarm(CommandMethod.Iai2));
        }

        [Fact]
        public void Home_with_the_coil_edge_releases_the_coil()
        {
            var sim = new SimUnit();
            sim.Pmss = true;
            sim.Servo = true;
            sim.Position = 37.0;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var st = axis.Home();
            Assert.True(st!.Homed);
            Assert.Equal(0.0, st.PositionMm);
            Assert.Equal(
                new[] { (Registers.COIL_HOME, false), (Registers.COIL_HOME, true), (Registers.COIL_HOME, false) },
                Frames.Coils(t));
        }

        [Fact]
        public void Home_with_0x45_sends_the_ORG_pattern_once()
        {
            var sim = new SimUnit().Ready(20.0);
            sim.Homed = false;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.Equal(7u, axis.ReadHomePattern());
            axis.Home(method: CommandMethod.Iai);
            Assert.Equal("45 00 0A 00 07 00 00", Frames.Pdu(Frames.Iai(t).Single()));
        }

        [Fact]
        public void Home_with_0x45_refuses_an_invalid_ORG_before_sending()
        {
            var sim = new SimUnit().Ready();
            sim.PutU32(0x0900 + (2 * 15), 0); // ORG 9.16 = 0
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.Throws<EcNotReadyException>(() => axis.Home(method: CommandMethod.Iai));
            Assert.Empty(Frames.Iai(t));
        }

        [Fact]
        public void A_lost_home_reply_with_the_home_running_is_waited_for()
        {
            var sim = new SimUnit().Ready(20.0);
            sim.DropHomeReply = true;
            sim.StepMm = 5;
            var axis = sim.Axis();
            Assert.True(axis.Home(method: CommandMethod.Iai)!.Homed);
        }

        [Fact]
        public void A_home_already_running_is_stopped_and_an_answered_refusal_is_not()
        {
            var sim = new SimUnit().Ready(20.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            sim.RejectHome = 0x24;
            var running = Assert.Throws<IaiCommandRejectedException>(() => axis.Home(method: CommandMethod.Iai));
            Assert.True(running.StopAttempted);
            Assert.Contains((Registers.COIL_STOP, true), Frames.Coils(t));

            var stopsBefore = Frames.Coils(t).Count(c => c.Address == Registers.COIL_STOP);
            sim.RejectHome = 0x11;
            var refused = Assert.Throws<IaiCommandRejectedException>(() => axis.Home(method: CommandMethod.Iai));
            Assert.False(refused.StopAttempted);
            Assert.Equal(stopsBefore, Frames.Coils(t).Count(c => c.Address == Registers.COIL_STOP));
        }

        [Fact]
        public void Action_mode_keeps_the_present_safety_speed_unless_told()
        {
            var sim = new SimUnit().Ready();
            sim.Pmss = false;
            sim.Sfty = true;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var st = axis.SetActionMode(pc: true);
            Assert.True(st.ModbusEnabled);
            Assert.True(st.SafetySpeed);
            Assert.Empty(Frames.Iai(t)); // PC: the PMSL coil, never 0x45 op1 = 1
            Assert.Equal((Registers.COIL_PMSL, true), Frames.Coils(t).Last());
            axis.SetActionMode(pc: true, safety: false);
            Assert.Equal((Registers.COIL_SFTY, false), Frames.Coils(t).Last());
            Assert.Empty(Frames.Iai(t));
            Assert.False(axis.SetSafetySpeed(false).SafetySpeed);
            Assert.True(axis.SetSafetySpeed(true).SafetySpeed);
        }

        [Fact]
        public void Mode_changes_are_refused_while_moving()
        {
            var (sim, axis, _) = Ready();
            sim.StepMm = 0.01;
            axis.MoveTo(100.0, wait: false);
            Assert.Throws<EcNotReadyException>(() => axis.SetActionMode(true));
            sim.Pmss = false;
            Assert.Throws<EcNotReadyException>(() => axis.EnableModbusControl());
            Assert.Throws<EcNotReadyException>(() => axis.WritePositionTable(0, new PositionEntry(), allowNonvolatile: true));
            axis.Stop();
        }
    }

    public class AxisStoredPositionTests
    {
        [Fact]
        public void Rows_decode_with_their_checksum()
        {
            var axis = new SimUnit().Axis();
            var row1 = axis.ReadPositionTable(1);
            Assert.Equal(30.0, row1.PositionMm);
            Assert.Equal(120.0, row1.BandMm);
            Assert.Equal(16.0, row1.SpeedMmS);
            Assert.Equal(26, row1.PushPower);
            Assert.True(row1.IsPush);
            Assert.True(row1.ChecksumOk);
            Assert.Equal(row1.ToRegisters(), UnitFixtures.RodPositions[1].Take(15).ToArray());
        }

        [Fact]
        public void MoveToPosition_writes_9800_and_waits()
        {
            var sim = new SimUnit().Ready(50.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var st = axis.MoveToPosition(0);
            Assert.Equal(0.0, st!.PositionMm);
            var write = Frames.Of(t, 0x06).Single();
            Assert.Equal("06 98 00 00 00", Frames.Pdu(write));
        }

        [Fact]
        public void A_speed_override_uses_0x45_cmd_6()
        {
            var sim = new SimUnit().Ready(50.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            axis.MoveToPosition(0, speedPct: 25);
            Assert.Equal("45 00 06 00 00 00 3F", Frames.Pdu(Frames.Iai(t).Single())); // round(256 x 0.25) - 1 = 63: runs 64 / 256
            Assert.Throws<SoftLimitException>(() => axis.MoveToPosition(0, speedPct: 0));
            Assert.Throws<SoftLimitException>(() => axis.MoveToPosition(0, speedPct: 101));
        }

        [Fact]
        public void A_push_row_runs_its_push()
        {
            var sim = new SimUnit().Ready(0.0);
            sim.ContactAt = 80.0;
            var axis = sim.Axis();
            var st = axis.MoveToPosition(1);
            Assert.True(st!.PushContact);
            Assert.Equal(80.0, st.PositionMm);
        }

        [Fact]
        public void Stored_row_guards_send_nothing()
        {
            var sim = new SimUnit().Ready(0.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.Throws<SoftLimitException>(() => axis.MoveToPosition(3)); // the EC uses rows 0-2
            Assert.Throws<SoftLimitException>(() => axis.MoveToPosition(2)); // stored speed 0
            sim.Position = 60.0;
            Assert.Throws<EcNotReadyException>(() => axis.MoveToPosition(1)); // inside the push zone
            var capped = new Axis(bus, 1, SimUnit.Fast with { MaxSpeedMmS = 100 });
            Assert.Throws<SoftLimitException>(() => capped.MoveToPosition(0)); // stored 240 mm/s above the cap
            Assert.Empty(Frames.Of(t, 0x06));
            Assert.Equal(0.0, capped.MoveToPosition(0, allowAboveCaps: true)!.PositionMm);
        }

        [Fact]
        public void An_ignored_stored_move_is_stopped_and_reported()
        {
            var sim = new SimUnit().Ready(50.0);
            sim.IgnoreStoredMoves = true;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var exc = Assert.Throws<MotionIncompleteException>(() => axis.MoveToPosition(0));
            Assert.True(exc.StopAttempted);
            Assert.Contains((Registers.COIL_STOP, false), Frames.Coils(t));
        }

        [Fact]
        public void Position_table_writes_need_no_opt_in()
        {
            var sim = new SimUnit().Ready(0.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var entry = axis.ReadPositionTable(0) with { PositionMm = 60.0 };
            // simple-data rows: never
            Assert.Throws<NonVolatileWriteException>(() => axis.WritePositionTable(260, entry));
            Assert.Throws<SoftLimitException>(() => axis.WritePositionTable(0, entry with { PositionMm = 151 }));
            var push = entry with { ControlFlags = Registers.CTLF_PUSH, PushPower = 200 };
            // above MXPP: a range check
            Assert.Throws<SoftLimitException>(() => axis.WritePositionTable(0, push, allowUnsafe: true));
            Assert.Empty(Frames.Writes(t));

            var written = axis.WritePositionTable(0, entry); // no allowNonvolatile
            Assert.Equal(60.0, written.PositionMm);
            Assert.True(written.ChecksumOk);
            var frame = Frames.Writes(t).Single();
            Assert.Equal(Registers.POSITION_TABLE_BASE, frame.Address);
            Assert.Equal(16, frame.Words.Length);

            var row1 = axis.ReadPositionTable(1) with { SpeedMmS = 20.0 }; // keeps its own low push power (10 %)
            Assert.Equal(20.0, axis.WritePositionTable(1, row1, allowNonvolatile: true).SpeedMmS); // still accepted

            using (var capture = new LogCapture())
            {
                Assert.Equal(60.0, axis.WritePositionTable(3, entry).PositionMm); // past the unit's rows
                Assert.Contains(capture.Messages, m => m.Contains("row 3 is past them: written anyway"));
            }

            using (var capture = new LogCapture())
            {
                axis.WritePositionTable(0, entry with { PositionMm = 61.0, ControlFlags = 0x0010 }); // a role bit
                Assert.Contains(
                    capture.Messages,
                    m => m.Contains("change the unknown role bits 0x0030") && m.Contains("written anyway"));
            }
        }
    }

    public class AxisParameterTests
    {
        [Fact]
        public void EC_parameters_follow_the_units_list()
        {
            var axis = new SimUnit().Axis();
            var prms = axis.Parameters();
            Assert.Equal(new[] { 1, 2, 3, 4, 5, 6, 8, 9, 12 }, prms.Select(p => p.Number).ToArray()); // what IA-OS lists on the rod
            var p1 = prms.Single(p => p.Number == 1);
            Assert.Equal(150.0, p1.Value);
            Assert.Equal("150.00 mm", p1.Text);
            Assert.Equal(150.0, p1.Max);
            Assert.Equal("1:Forward", prms.Single(p => p.Number == 3).Text);
            Assert.True(prms.Single(p => p.Number == 3).Dangerous);
            Assert.False(prms.Single(p => p.Number == 8).Writable);
            Assert.Equal(0.1, axis.ReadParameter("P2").Value);
            Assert.True(axis.ReadParameterModel().Verified);
        }

        [Fact]
        public void Raw_items_read_by_class_and_item()
        {
            var axis = new SimUnit().Axis();
            var item = axis.ReadParameter("9.16");
            Assert.Equal("raw", item.Family);
            Assert.Equal(7u, item.Unsigned);
            Assert.Equal(5, item.Named); // CON No.5 home direction lives there
            Assert.True(item.Dangerous);
            Assert.Throws<UnsafeCommandException>(() => axis.ReadParameter("8.999"));
            Assert.Throws<UnsafeCommandException>(() => axis.ReadParameter("x"));
        }

        [Fact]
        public void A_parameter_write_is_verified_and_committed_with_PARW()
        {
            var sim = new SimUnit().Ready(0.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var result = axis.WriteParameter(2, 0.2);
            Assert.True(result.Verified);
            Assert.True(result.Changed);
            Assert.True(result.Committed);
            Assert.True(result.CommitConfirmed);
            Assert.True(result.Saved);
            Assert.Equal("0.10 mm", result.Before);
            Assert.Equal("0.20 mm", result.After);
            Assert.Equal("0x0810", result.Address);
            var write = Frames.Writes(t).Single();
            Assert.Equal(0x0810, write.Address);
            Assert.Equal(new ushort[] { 0, 20 }, write.Words);
            Assert.Contains((Registers.COIL_PARW, true), Frames.Coils(t));
            Assert.Equal(20, sim.Word(0x0811));
        }

        [Fact]
        public void A_choice_parameter_takes_its_label_and_keeps_the_other_bits()
        {
            var sim = new SimUnit().Ready(0.0);
            var axis = sim.Axis();
            var before = sim.Word(0x0800 + (2 * 13) + 1); // 8.14 low word
            var result = axis.WriteParameter("P5", "enable", commit: false);
            Assert.False(result.Committed);
            Assert.Equal("1:Enable", result.After);
            Assert.Equal(before | (1 << 6), sim.Word(0x0800 + (2 * 13) + 1));
        }

        [Fact]
        public void Parameter_guards_send_nothing()
        {
            var sim = new SimUnit().Ready(0.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.Throws<UnsafeCommandException>(() => axis.WriteParameter(8, 0)); // read-only here
            Assert.Throws<UnsafeCommandException>(() => axis.WriteParameter(42, 1));
            Assert.Throws<SoftLimitException>(() => axis.WriteParameter(2, 1.5)); // P2 0.01..1.00
            Assert.Throws<SoftLimitException>(() => axis.WriteParameter(1, 151.0)); // P1 can only narrow
            Assert.Throws<SoftLimitException>(() => axis.WriteParameter(5, true));
            Assert.Throws<SoftLimitException>(() => axis.WriteParameter(5, "maybe"));
            Assert.Throws<UnsafeCommandException>(() => axis.WriteParameter("3.2", 1)); // not a parameter class
            Assert.Empty(Frames.Writes(t));
            Assert.DoesNotContain(Frames.Coils(t), c => c.Address == Registers.COIL_PARW);
        }

        [Fact]
        public void Home_unlisted_and_dangerous_parameters_are_written_with_a_warning()
        {
            var sim = new SimUnit().Ready(0.0);
            var axis = sim.Axis();
            var home = axis.WriteParameter(3, 0, commit: false); // home: no allowHomeParams needed
            Assert.True(home.HomeParameter);
            Assert.Contains(home.Warnings, w => w.StartsWith("P3 / P4 are home parameters", StringComparison.Ordinal));
            var unlisted = axis.WriteParameter(7, 1, commit: false); // not listed on the rod
            Assert.Contains(
                unlisted.Warnings,
                w => w.Contains("is not in the parameter list") && w.Contains("written anyway"));
            var gain = axis.WriteParameter("9.7", 10, commit: false); // a servo gain item: dangerous
            Assert.Contains(gain.Warnings, w => w.Contains("raw write of '9.7'") && w.Contains("written anyway"));
            var wider = axis.WriteParameter("7.1", 20000, commit: false); // widens the + soft limit
            Assert.Contains(wider.Warnings, w => w.Contains("widens the + soft limit 7.1"));
        }

        [Fact]
        public void The_home_direction_flips_the_ORG_parity_with_the_home_warning()
        {
            var sim = new SimUnit().Ready(0.0);
            var axis = sim.Axis();
            var result = axis.WriteParameter(3, "0:Reverse", allowHomeParams: true); // the flag is accepted and ignored
            Assert.True(result.HomeParameter);
            Assert.Equal("0:Reverse", result.After);
            Assert.Equal(8, sim.Word(0x0900 + (2 * 15) + 1)); // ORG 7 -> 8
            Assert.Contains(result.Warnings, w => w.StartsWith("P3 / P4 are home parameters", StringComparison.Ordinal));
        }

        [Fact]
        public void P1_narrows_the_stroke_at_once()
        {
            var sim = new SimUnit().Ready(0.0);
            var axis = sim.Axis();
            axis.WriteParameter(1, 120.0);
            Assert.Equal(120.0, axis.EffectiveStroke!.Value.Max);
            Assert.Throws<SoftLimitException>(() => axis.MoveTo(121.0));
        }

        [Fact]
        public void CON_parameters_use_the_CON_numbers()
        {
            var sim = new SimUnit("pcon").Ready(60.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.False(axis.IsEc);
            Assert.Equal("PCON", axis.ProductFamily);
            var prms = axis.Parameters();
            Assert.All(prms, p => Assert.Equal("CON", p.Family));
            var pshv = prms.Single(p => p.Number == 34);
            Assert.Equal("PSHV", pshv.Symbol);
            Assert.Equal("mm/s", pshv.Unit);

            var result = axis.WriteParameter("No.34", 15.0); // at most the actuator's MXPH 7.8, 20 mm/s (ME0342-4H p.6-2)
            Assert.Equal("15.00 mm/s", result.After);
            Assert.True(result.Saved);
            Assert.Throws<SoftLimitException>(() => axis.WriteParameter(35, 300)); // safety velocity 1..250
            var home = axis.WriteParameter(5, "Forward"); // ORG 8 -> 7; no allowHomeParams needed
            Assert.True(home.HomeParameter);
            Assert.Contains(home.Warnings, w => w.Contains("is a dangerous parameter"));
            var gain = axis.WriteParameter(7, 3); // a gain: no allowDangerous needed
            Assert.Contains(gain.Warnings, w => w.Contains("is a dangerous parameter"));
            Assert.Equal(3, Frames.Writes(t).Count);
        }

        [Fact]
        public void The_PCON_reports_64_stored_rows_and_no_EC_floor()
        {
            var axis = new SimUnit("pcon").Axis();
            Assert.Equal(64, axis.MaxPositions());
            Assert.Equal(87.90, axis.ReadPositionTable(1).PositionMm, 2);
        }
    }

    public class AxisUnitTests
    {
        [Fact]
        public void Software_reset_on_an_EC_is_confirmed_by_PMSS()
        {
            var sim = new SimUnit().Ready(10.0);
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var result = axis.SoftwareReset(timeoutS: 2.0, resetWaitS: 0.02);
            Assert.True(result.Accepted);
            Assert.True(result.Confirmed);
            Assert.Contains("PMSS 1 -> 0", result.ConfirmedBy);
            Assert.True(result.NeedsHome);
            Assert.False(result.ModbusEnabled);
            var iai = Frames.Iai(t).Select(Frames.Pdu).ToList();
            Assert.Equal(new[] { "45 00 09 00 00 00 00", "45 00 05 00 01 00 00" }, iai);
            Assert.DoesNotContain(Frames.Coils(t), c => c.Address == Registers.COIL_SFRS);
            Assert.NotNull(JsonSerializer.Serialize(result.ToDictionary()));
        }

        [Fact]
        public void Software_reset_on_a_CON_controller_is_confirmed_by_the_reply_gap()
        {
            var sim = new SimUnit("pcon").Ready(60.0);
            var axis = sim.Axis();
            var result = axis.SoftwareReset(timeoutS: 2.0, resetWaitS: 0.05);
            Assert.True(result.Confirmed);
            Assert.StartsWith("the unit was silent after the command", result.ConfirmedBy); // then "the servo came back on by itself"
            Assert.True(result.ModbusEnabled);
            Assert.True(result.Homed);
        }

        [Fact]
        public void Software_reset_is_refused_while_moving()
        {
            var sim = new SimUnit().Ready();
            sim.StepMm = 0.01;
            var (bus, t) = sim.Bus();
            var axis = new Axis(bus, 1, SimUnit.Fast);
            axis.MoveTo(100, wait: false);
            Assert.Throws<EcNotReadyException>(() => axis.SoftwareReset());
            Assert.Empty(Frames.Iai(t));
            axis.Stop();
        }

        [Fact]
        public void Baud_change_broadcasts_and_switches_the_port_in_place()
        {
            var sim = new SimUnit().Ready();
            var (bus, t) = sim.Bus(baudRate: 38400);
            var axis = new Axis(bus, 1, SimUnit.Fast);
            Assert.Throws<SoftLimitException>(() => axis.SetBaudRate(12345)); // no IAI code
            Assert.Empty(t.Writes); // the refusal sent nothing
            var result = axis.SetBaudRate(115200, settleS: 0.01);
            Assert.True(result.Verified);
            Assert.Equal(38400, result.Previous);
            Assert.Equal(115200, axis.BaudRate);
            Assert.Equal("00 45 00 04 00 14 00 05", result.Frame.Substring(0, 23));
            Assert.Equal(new[] { 115200 }, t.BaudRates);
        }

        [Fact]
        public void The_38400_code_is_sent_without_the_opt_in_and_taken_by_a_unit_that_takes_it()
        {
            // The EC-GRBP8M gripper and the PCON-CB take code 0x06 (VERIFIED): no opt-in, and the check after the switch confirms it.
            var sim = new SimUnit().Ready();
            var (bus, t) = sim.Bus(baudRate: 230400);
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var result = axis.SetBaudRate(38400, settleS: 0.01);
            Assert.True(result.Verified);
            Assert.True(result.CodeVerified); // no "not verified" warning for a unit-dependent rate
            Assert.Equal(230400, result.Previous);
            Assert.Equal("00 45 00 04 00 06 00 05", result.Frame.Substring(0, 23));
            Assert.Equal(38400, axis.BaudRate);
            Assert.Equal(new[] { 38400 }, t.BaudRates);
        }

        [Fact]
        public void A_baud_change_the_unit_ignores_is_reported()
        {
            var sim = new SimUnit().Ready();
            var (bus, t) = sim.Bus(baudRate: 115200);
            sim.Transport = null; // the unit ignores the broadcast
            var axis = new Axis(bus, 1, SimUnit.Fast);
            var exc = Assert.Throws<VerificationException>(() => axis.SetBaudRate(230400, settleS: 0.01));
            Assert.Contains("still answers at 115200", exc.Message);
            Assert.DoesNotContain("power cycle", exc.Message);
            Assert.Equal(115200, axis.BaudRate);
            // The 38400 code is sent and ignored (as by the EC-R6S rod): the unit keeps answering at its rate.
            var ignored = Assert.Throws<VerificationException>(() => axis.SetBaudRate(38400, settleS: 0.01));
            Assert.Contains("still answers at 115200", ignored.Message);
            Assert.EndsWith("this unit ignores the 38400 code (the EC-R6S rod does too): a power cycle brings it back to 38400", ignored.Message);
            Assert.Equal(115200, axis.BaudRate);
        }

        [Fact]
        public void Open_on_a_transport_and_connect_take_control_at_rest()
        {
            var sim = new SimUnit();
            sim.Servo = true;
            sim.Homed = true;
            var transport = new FakeTransport(sim.Respond, 38400) { AnswerAtBaudRate = 38400 };
            using var axis = Axis.Open(transport, options: SimUnit.Fast);
            Assert.True(axis.BaudDetected);
            Assert.Equal(38400, axis.BaudRate);
            var info = axis.Connect();
            Assert.Equal("EC-R6S-150-1-PN", info.Model);
            Assert.True(sim.Pmss);
            axis.Close();
            Assert.True(transport.Closed);
        }

        [Fact]
        public void An_axis_on_a_shared_port()
        {
            var sim = new SimUnit(address: 2).Ready(5.0);
            using var port = new Port(new FakeTransport(sim.Respond));
            var axis = port.Axis(2, SimUnit.Fast);
            Assert.Equal(2, axis.Slave);
            Assert.Equal(5.0, axis.Status().PositionMm);
        }
    }

    public class AxisSettleAndCommitTests
    {
        [Fact]
        public void A_commit_the_unit_never_shows_busy_is_reported_unconfirmed()
        {
            var sim = new SimUnit().Ready(0.0);
            sim.ParwBusyReads = 0;
            var axis = sim.Axis();
            var result = axis.WriteParameter(2, 0.3);
            Assert.True(result.Committed);
            Assert.False(result.CommitConfirmed);
            Assert.False(result.Saved);
            Assert.Contains(result.Warnings, w => w.StartsWith("commit NOT confirmed", StringComparison.Ordinal));
            Assert.Contains("NOT confirmed", result.TakesEffect);
        }

        [Fact]
        public void A_CON_unit_writes_rows_beyond_the_EC_three()
        {
            var sim = new SimUnit("pcon").Ready(60.0);
            var axis = sim.Axis();
            var entry = axis.ReadPositionTable(10) with { PositionMm = 25.0, SpeedMmS = 100.0, AccelG = 0.3, DecelG = 0.3, BandMm = 0.1 };
            var written = axis.WritePositionTable(10, entry, allowNonvolatile: true);
            Assert.Equal(25.0, written.PositionMm);
            Assert.Equal(25.0, axis.MoveToPosition(10)!.PositionMm);
        }
    }

    public class ParameterCodecTests
    {
        [Fact]
        public void User_parameter_parse_and_encode()
        {
            var p3 = UserParameters.BY_NO[3];
            Assert.Equal(1, UserParameters.Parse(p3, "forward"));
            Assert.Equal(0, UserParameters.Parse(p3, "0:Reverse"));
            Assert.Equal(8u, UserParameters.Encode(p3, 7u, 0)); // odd 7 = Forward -> 8 = Reverse
            Assert.Equal(7u, UserParameters.Encode(p3, 7u, 1));
            Assert.Throws<ArgumentException>(() => UserParameters.Encode(p3, 0xFFFFFFFFu, 1));
            var p12 = UserParameters.BY_NO[12];
            Assert.Equal(1, UserParameters.Parse(p12, "instant"));
            var p1 = UserParameters.BY_NO[1];
            Assert.Equal(15030u, UserParameters.Encode(p1, 0, 150.0));
            Assert.Throws<ArgumentException>(() => UserParameters.Parse(p1, "abc"));
            Assert.Throws<ArgumentException>(() => UserParameters.Encode(UserParameters.BY_NO[8], 0, 1));
        }

        [Fact]
        public void The_parameter_list_follows_the_unit_code_whatever_the_model_string()
        {
            // another model type with unit 0x00 gets the same IA-OS list, the same note and no model label
            var rod = UserParameters.ModelParameterList(0x00, "EC-R6S-150-1-PN");
            var other = UserParameters.ModelParameterList(0x00, "EC-RR7-200-1-PN");
            Assert.True(other.Known && other.Verified);
            Assert.Equal(rod.Listed, other.Listed);
            Assert.Equal("the parameters IA-OS lists for unit 0x00 (EC standard)", other.Note);
            Assert.Equal(rod.Note, other.Note);
            Assert.Empty(other.SeenOn);
            Assert.True(UserParameters.ModelParameterList(0x0A, null).Verified); // no model string read: same list

            var unknown = UserParameters.ModelParameterList(0x7E, "");
            Assert.False(unknown.Known || unknown.Verified);
            Assert.Equal(UserParameters.BY_NO.Keys.OrderBy(n => n), unknown.Listed); // every mapped parameter
            Assert.Equal("unit code 0x7E: every mapped item found on the unit is shown", unknown.Note);
            Assert.Equal("unit code not readable (FC11 / CTCL): every mapped item found on the unit is shown",
                UserParameters.ModelParameterList(null, null).Note);

            var p2 = new DecodedParameter { No = 2, Found = true };
            Assert.Equal(string.Empty, UserParameters.Annotate(new[] { p2 }, 0x7E, null).Single().ModelNote);
            var listed = UserParameters.Annotate(new[] { p2 }, 0x00, "EC-RR7-200-1-PN").Single();
            Assert.Equal((string.Empty, true, true), (listed.ModelNote, listed.VerifiedForModel, listed.Writable));
        }

        [Fact]
        public void CON_parameter_codecs()
        {
            var baud = ConParameters.BY_NO[16];
            Assert.Equal(38400L, ConParameters.Decode(baud, 6));
            Assert.Equal(20u, ConParameters.Encode(baud, 6, ConParameters.Parse(baud, 115200)));
            Assert.Throws<ArgumentException>(() => ConParameters.Parse(baud, 12345));
            var pair = ConParameters.BY_NO[18];
            Assert.Equal(2L, ConParameters.Decode(pair, 3));
            Assert.Equal(3u, ConParameters.Encode(pair, 0, ConParameters.Parse(pair, "Normally closed")));
            var ip = ConParameters.BY_NO[140];
            Assert.Equal("192.168.0.1", ConParameters.Decode(ip, 0xC0A80001));
            Assert.Equal(0xC0A80001u, ConParameters.Encode(ip, 0, ConParameters.Parse(ip, "192.168.0.1")));
            var pct = ConParameters.BY_NO[12];
            Assert.Equal("70 %", ConParameters.Text(pct, ConParameters.Decode(pct, 0xB2)));
            Assert.Equal("1:Forward", ConParameters.Text(ConParameters.BY_NO[5], ConParameters.Decode(ConParameters.BY_NO[5], 7)));
            Assert.Equal((12, 1, 1), Axis.ParseItemKey("C.1[1]"));
            Assert.Equal((12, 1, 0), Axis.ParseItemKey("0x0C.1"));
            Assert.Equal((8, 15, 0), Axis.ParseItemKey("8.15"));
        }
    }
}
