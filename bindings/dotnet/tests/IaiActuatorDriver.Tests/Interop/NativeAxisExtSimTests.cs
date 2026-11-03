using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 0.2 additions of <see cref="NativeAxis"/> on the native core's simulated units: push, command-profile
    /// moves, patterns, sequences, move handles, the cancel entry, the status monitor as C# events, fast mode, recovery,
    /// backups, unit info, the self-test, the line, the strategy and the modes. The scenarios follow
    /// core/tests/test_c_api_ext.cpp. No hardware, no port opened. Timing: time scale 20 or more where a wait is measured,
    /// real time only where a call must land while a move runs, generous deadlines (the PC may be busy).
    /// </summary>
    public class NativeAxisExtSimTests
    {
        private static NativeSimOptions Fast => new NativeSimOptions { TimeScale = 20 };

        private static NativeSimOptions Faster => new NativeSimOptions { TimeScale = 50 };

        private static NativeSimOptions RealTime => new NativeSimOptions { TimeScale = 1 };

        private static NativeRecoveryPolicy QuickPolicy => new NativeRecoveryPolicy { PollS = 0.02 };

        /// <summary>Wait until <paramref name="condition"/> holds or 20 s passed; whether it holds.</summary>
        private static bool Eventually(Func<bool> condition)
        {
            var clock = Stopwatch.StartNew();
            while (clock.Elapsed < TimeSpan.FromSeconds(20))
            {
                if (condition())
                {
                    return true;
                }

                Thread.Sleep(20);
            }

            return condition();
        }

        [NativeFact]
        public void Push_patterns_and_sequences()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Connect();
            var push = axis.Push(80.0, 10.0, 30.0)!;
            Assert.Equal(PushOutcome.Missed, push.Outcome);
            Assert.Equal("missed", push.OutcomeName);
            Assert.InRange(push.PositionMm, 89.9, 90.1);
            Assert.True(push.Status.PushMissed);
            Assert.Contains("\"outcome\":\"missed\"", push.Json);
            Assert.Throws<SoftLimitException>(() => axis.Push(80.0, 10.0, 75.0));  // above the force cap
            Assert.Throws<ArgumentException>(() => axis.Push(80.0, 10.0, 30.0, direction: 2));  // (0.8) 0 is +1, the default

            Assert.InRange(axis.MoveTo(100.0, accelPattern: NativeAccelPattern.SMotion)!.PositionMm, 99.89, 100.11);
            Assert.Throws<ArgumentException>(() => axis.MoveTo(90.0, accelPattern: (NativeAccelPattern)9));
            Assert.Throws<EcNotReadyException>(() => axis.ChangeAccel(0.2, wait: false));  // no move of this axis

            var statuses = axis.MoveSequence(
                new[] { new NativeSequencePoint(20.0), new NativeSequencePoint(40.0, speed: 80.0), new NativeSequencePoint(60.0) },
                speed: 100.0);
            Assert.Equal(3, statuses.Count);
            Assert.InRange(statuses[2].PositionMm, 59.89, 60.11);
            Assert.InRange(axis.MoveSequence(new[] { 50.0, 30.0 }, speed: 100.0).Last().PositionMm, 29.89, 30.11);
            Assert.Throws<SoftLimitException>(() => axis.MoveSequence(new[] { 20.0, 1e6 }));  // checked before the first frame
            Assert.InRange(axis.Status().PositionMm, 29.89, 30.11);
            Assert.Throws<ArgumentException>(() => axis.MoveSequence(Array.Empty<double>()));
        }

        [NativeFact]
        public void A_deceleration_of_its_own_takes_the_command_profile_path()
        {
            using var pcon = NativeAxis.OpenSim("pcon", Fast);
            pcon.Connect();
            var profileFrames = 0;
            pcon.SetFrameHandler((direction, frame) =>
            {
                if (direction == FrameDirection.Tx && frame.Length > 2 && frame[1] == 0x17)
                {
                    Interlocked.Increment(ref profileFrames);
                }
            });
            Assert.InRange(pcon.MoveTo(120.0, speed: 100.0, accelG: 0.3, decelG: 0.1)!.PositionMm, 119.89, 120.11);
            Assert.True(Volatile.Read(ref profileFrames) > 0);
            var e = Assert.Throws<SoftLimitException>(() => pcon.MoveTo(100.0, speed: 100.0, accelG: 0.3, decelG: 1.5));  // above MAXD
            Assert.Contains("MAXD", e.Message);
            Assert.InRange(pcon.CheckMove(100.0, speed: 100.0, accelG: 0.3, decelG: 0.1).PositionMm, 119.89, 120.11);
        }

        [NativeFact]
        public void Change_speed_and_acceleration_of_a_running_move()
        {
            using var axis = NativeAxis.OpenSim("slider", RealTime);
            axis.MoveTo(130.0, speed: 20.0, wait: false);  // from 73.95 mm: almost 3 s
            Assert.Null(axis.ChangeAccel(0.2, wait: false));
            Assert.InRange(axis.ChangeSpeed(100.0, timeout: 30.0)!.PositionMm, 129.89, 130.11);
        }

        [NativeFact]
        public void Move_handles_poll_wait_info_and_json()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Connect();
            using (var move = axis.StartMoveTo(100.0))
            {
                var info = move.Info;
                Assert.Equal(100.0, info.Target, 6);
                Assert.True(info.DeadlineS > info.SentAtS);
                Assert.Equal(NativeCompletion.Safe, info.Completion);
                NativeStatus? done = null;
                for (var i = 0; i < 400 && done is null; i++)
                {
                    done = move.Poll(0.05);  // short slices: the move runs on between them
                }

                Assert.NotNull(done);
                Assert.InRange(done!.PositionMm, 99.89, 100.11);
                Assert.True(move.IsDone);
                Assert.InRange(move.Wait().PositionMm, 99.89, 100.11);  // a late wait returns the result
                info = move.Info;
                Assert.NotNull(info.Result);
                Assert.False(info.Cancelled);
                Assert.Contains("\"done\":true", move.Json());
                Assert.Same(axis, move.Axis);
            }

            using (var by = axis.StartMoveBy(-50.0))
            {
                Assert.InRange(by.Wait(30.0).PositionMm, 49.89, 50.11);
            }

            using (var jog = axis.StartJog(1.0))
            {
                Assert.InRange(jog.WaitAsync(30.0).Result.PositionMm, 50.89, 51.11);
            }

            Assert.Throws<SoftLimitException>(() => axis.StartMoveTo(1e6));
            var closed = axis.StartMoveTo(60.0, speed: 100.0);
            closed.Wait(30.0);
            closed.Dispose();
            closed.Dispose();
            Assert.True(closed.IsClosed);
            Assert.Throws<ObjectDisposedException>(() => closed.Info);
        }

        [NativeFact]
        public void Cancelling_a_move_handle_stops_the_axis()
        {
            using var axis = NativeAxis.OpenSim("slider", RealTime);
            using var move = axis.StartMoveTo(140.0, speed: 20.0);  // over 3 s
            Assert.True(Eventually(() => axis.Status().Moving));
            var stopped = move.Cancel(wait: true);
            Assert.False(stopped.Moving);
            Assert.True(stopped.PositionMm < 139.0);
            Assert.True(move.Info.Cancelled);
        }

        [NativeFact]
        public void A_cancellation_token_ends_the_wait_with_a_decel_stop()
        {
            using var axis = NativeAxis.OpenSim("slider", RealTime);
            using var move = axis.StartMoveTo(140.0, speed: 20.0);
            using var cts = new CancellationTokenSource(TimeSpan.FromMilliseconds(300));
            var clock = Stopwatch.StartNew();
            var e = Assert.Throws<OperationCanceledException>(() => move.Wait(cancellationToken: cts.Token));
            Assert.Equal(cts.Token, e.CancellationToken);
            Assert.True(clock.Elapsed < TimeSpan.FromSeconds(20));
            Assert.True(Eventually(() => !axis.Status().Moving));
            Assert.True(axis.Status().PositionMm < 139.0);
        }

        [NativeFact]
        public void Cancel_from_another_thread_ends_a_blocking_move()
        {
            using var axis = NativeAxis.OpenSim("slider", RealTime);
            Exception? error = null;
            var mover = new Thread(() =>
            {
                try
                {
                    axis.MoveTo(140.0, speed: 20.0);
                }
                catch (Exception e)
                {
                    error = e;
                }
            });
            mover.Start();
            Assert.True(Eventually(() => axis.Status().Moving));
            axis.Cancel();
            Assert.True(mover.Join(TimeSpan.FromSeconds(30)));
            Assert.IsType<OperationCanceledException>(error);
            Assert.Equal(NativeResult.Cancelled, NativeErrors.ResultOf(error!));
            Assert.Contains("WaitCancelledError", NativeErrors.ErrorJsonOf(error!));
            Assert.True(axis.Status().PositionMm < 139.0);
            axis.Cancel();  // nothing waits: nothing happens, later calls are not affected
            Assert.InRange(axis.MoveTo(100.0, speed: 100.0)!.PositionMm, 99.89, 100.11);
        }

        [NativeFact]
        public void Monitor_events_are_raised_on_the_binding_thread()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Connect();
            Assert.Null(axis.NextEvent());
            Assert.Throws<EcNotReadyException>(() => axis.MonitorLatest());
            Assert.Throws<ArgumentException>(() => axis.StartMonitor(new NativeMonitorOptions { QueueCapacity = 0 }));
            Assert.False(axis.EventPumpRunning);

            var statuses = new ConcurrentQueue<NativeEvent>();
            var all = 0;
            var done = new ManualResetEventSlim();
            var handlerThread = 0;
            axis.EventReceived += (s, e) => Interlocked.Increment(ref all);
            axis.StatusChanged += (s, e) => statuses.Enqueue(e);
            axis.MotionDone += (s, e) =>
            {
                Volatile.Write(ref handlerThread, Environment.CurrentManagedThreadId);
                done.Set();
            };
            axis.StatusChanged += (s, e) => throw new InvalidOperationException("a handler that throws is ignored");
            axis.StartMonitor();
            Assert.True(axis.EventPumpRunning);

            var sample = axis.MonitorWaitUpdate(0, 5.0);
            Assert.NotNull(sample);
            Assert.True(sample!.Seq > 0);
            Assert.True(sample.Running);
            Assert.NotNull(axis.MonitorWaitUpdate(sample.Seq, 5.0));

            axis.MoveTo(60.0, speed: 100.0);
            Assert.True(done.Wait(TimeSpan.FromSeconds(20)));
            Assert.NotEqual(0, Volatile.Read(ref handlerThread));
            Assert.NotEqual(Environment.CurrentManagedThreadId, Volatile.Read(ref handlerThread));
            Assert.NotEmpty(statuses);
            Assert.All(statuses, e => Assert.Equal("SIM:1", e.Axis));
            Assert.All(statuses, e => Assert.NotNull(e.Status));
            Assert.True(Volatile.Read(ref all) >= statuses.Count);

            Assert.NotNull(axis.MonitorWaitDone(5.0));
            Assert.True(axis.MonitorLatest().Polls > 0);
            Assert.Contains("\"running\":true", axis.MonitorInfoJson());
            axis.StopMonitor();
            Assert.False(axis.EventPumpRunning);
            axis.StopMonitor();  // no monitor: no-op
        }

        [NativeFact]
        public void Closing_the_axis_ends_its_event_thread()
        {
            var axis = NativeAxis.OpenSim("slider", Fast);
            axis.StartMonitor();
            Assert.True(axis.EventPumpRunning);
            axis.Close();
            Assert.False(axis.EventPumpRunning);
            Assert.Throws<ObjectDisposedException>(() => axis.NextEvent());
        }

        [NativeFact]
        public void Fast_mode_keeps_the_rate_on_a_line_without_one()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            axis.Connect();
            Assert.Throws<ArgumentException>(() => axis.EnableFast(-5));
            var on = axis.EnableFast();
            Assert.Equal(NativeCompletion.Fast, on.Completion);
            Assert.True(on.Monitor);
            Assert.Null(on.BaudRate);  // the simulated unit has no baud rate: kept ...
            Assert.NotEmpty(on.Notes);  // ... and the reason noted
            Assert.True(axis.FastMode);
            Assert.Contains("\"completion\":\"fast\"", on.Json);
            Assert.InRange(axis.MoveTo(50.0, speed: 100.0)!.PositionMm, 49.89, 50.11);
            var off = axis.DisableFast();
            Assert.Equal(NativeCompletion.Safe, off.Completion);
            Assert.False(off.Monitor);
            Assert.False(axis.FastMode);

            using var atOpen = NativeAxis.OpenSim("slider", Fast, fast: true);
            Assert.True(atOpen.FastMode);
        }

        [NativeFact]
        public void Frame_and_log_handlers_see_the_traffic()
        {
            using var axis = NativeAxis.OpenSim("slider", Fast);
            var frames = 0;
            var tx = 0;
            axis.SetFrameHandler((direction, frame) =>
            {
                if (frame.Length > 0)
                {
                    Interlocked.Increment(ref frames);
                }

                if (direction == FrameDirection.Tx)
                {
                    Interlocked.Increment(ref tx);
                }
            });
            axis.Status();
            Assert.True(Volatile.Read(ref frames) >= 2);  // the request and the answer
            Assert.True(Volatile.Read(ref tx) >= 1);
            axis.SetFrameHandler(null);
            var before = Volatile.Read(ref frames);
            axis.Status();
            Assert.Equal(before, Volatile.Read(ref frames));

            var lines = new ConcurrentQueue<string>();
            NativeDriver.SetLogHandler((level, source, message) => lines.Enqueue(source + " " + message));
            try
            {
                axis.Connect();
                axis.EnableFast();  // a simulated unit keeps its rate: the note is logged
                Assert.Contains(lines, l => l.Contains("fast mode"));
                Assert.All(lines, l => Assert.StartsWith("iai_actuator_driver", l));
            }
            finally
            {
                NativeDriver.SetLogHandler(null);
            }

            var count = lines.Count;
            axis.DisableFast();
            axis.EnableFast();
            Assert.Equal(count, lines.Count);  // removed: nobody listens
        }

        [NativeFact]
        public void Track_check_link_and_recover()
        {
            using var axis = NativeAxis.OpenSim("gripper", Faster);
            axis.Connect();
            Assert.Equal("ok", axis.LinkState);
            Assert.Throws<EcNotReadyException>(() => axis.SessionJson());
            Assert.Throws<ArgumentException>(() => axis.Track("G", rows: "x"));  // a bad row list
            axis.Track("G");
            Assert.Contains("\"name\":\"G\"", axis.SessionJson());

            var check = axis.CheckLink();
            Assert.True(check.Ok);
            Assert.Equal("ok", check.State);
            Assert.NotNull(check.Status);
            Assert.Empty(check.Reasons);
            Assert.StartsWith("{", check.Json);

            var report = axis.Recover(10.0, QuickPolicy);  // never moves anything
            Assert.True(report.SameUnit);
            Assert.NotEqual(string.Empty, report.Summary);
            Assert.StartsWith("{", report.Json);
            Assert.Equal("ok", axis.LinkState);

            axis.NoteTarget(12.5);
            axis.NoteTarget(null);
            Assert.Throws<ArgumentException>(() => axis.NoteTarget(double.PositiveInfinity));
            axis.NoteControl(modbusControl: true);
            Assert.Throws<ArgumentException>(() => axis.NoteControl(baudRate: -9600));
            axis.Forget();
            Assert.Throws<EcNotReadyException>(() => axis.SessionJson());

            // No session and no identity read: the recovery cannot confirm the unit.
            using var fresh = NativeAxis.OpenSim("gripper", Faster);
            var e = Assert.Throws<RecoveryException>(() => fresh.Recover(1.0, QuickPolicy));
            Assert.Equal(NativeResult.Recovery, NativeErrors.ResultOf(e));
            Assert.Contains("\"kind\":\"RecoveryError\"", NativeErrors.ErrorJsonOf(e));
        }

        [NativeFact]
        public void Backups_and_a_session_from_another_units_backup()
        {
            var dir = Path.Combine(Path.GetTempPath(), "iad_cs_backup_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                using var gripper = NativeAxis.OpenSim("gripper", Faster);
                gripper.Connect();
                var file = Path.Combine(dir, "gripper.json");
                var backup = gripper.BackupJson("used", file);
                Assert.Contains("ec_driver.backup/1", backup);
                Assert.True(File.Exists(file));
                Assert.Equal(backup, gripper.LastResultJson());
                Assert.Throws<ArgumentException>(() => gripper.BackupJson("bogus rows"));

                using var slider = NativeAxis.OpenSim("slider", Faster);
                slider.Connect();
                var many = Path.Combine(dir, "many");
                var listed = NativeAxis.BackupAxesJson(new[] { slider, gripper }, many, new[] { "slider", "grip:1" }, "0,1");
                Assert.Contains("\"slider\":", listed);
                Assert.Contains("\"grip:1\":", listed);
                Assert.DoesNotContain("error:", listed);
                Assert.True(File.Exists(Path.Combine(many, "slider.json")));
                Assert.True(File.Exists(Path.Combine(many, "grip_1.json")));
                Assert.Throws<ArgumentException>(() => NativeAxis.BackupAxesJson(new[] { slider, gripper }, many, new[] { "one" }));

                // The gripper's backup on the slider: the recovery refuses the other unit, nothing written.
                Assert.Throws<ArgumentException>(() => slider.SessionFromBackup("not json"));
                slider.SessionFromBackup(backup, "S");
                var e = Assert.Throws<UnitReplacedException>(() => slider.Recover(2.0, QuickPolicy));
                Assert.Contains("\"expected\":{", NativeErrors.ErrorJsonOf(e));
                Assert.Contains("\"found\":{", NativeErrors.ErrorJsonOf(e));
                slider.SessionFromBackupFile(file, "S");
                Assert.Throws<IOException>(() => slider.SessionFromBackupFile(Path.Combine(dir, "missing.json")));
            }
            finally
            {
                try
                {
                    Directory.Delete(dir, recursive: true);
                }
                catch (IOException)
                {
                }
            }
        }

        [NativeFact]
        public void Unit_info_self_test_line_strategy_and_modes()
        {
            using var axis = NativeAxis.OpenSim("slider", Faster);
            axis.Connect();
            var info = axis.ReadUnitInfoJson(new[] { "identity" });
            Assert.Contains("ec_driver.unit_info/1", info);
            Assert.Contains("\"identity\":{", info);
            Assert.Throws<ArgumentException>(() => axis.ReadUnitInfoJson(new[] { "bogus" }));
            Assert.Contains("\"position\":", axis.ReadMonitorJson());

            var lines = new ConcurrentQueue<NativeEvent>();
            axis.CheckItem += (s, e) => lines.Enqueue(e);
            axis.StartEventPump();
            var check = axis.SelfTest(portLabel: "SIM");
            Assert.True(check.Passed);
            Assert.Equal(0, check.Failed);
            Assert.True(check.Items > 5);
            Assert.Contains("\"port\":\"SIM\"", check.Json);
            Assert.True(Eventually(() => lines.Count >= check.Items));
            Assert.Equal(check.Items, lines.Count);
            Assert.True(lines.TryPeek(out var first));
            Assert.Equal("link", first!.Name);
            Assert.Equal("PASS", first.Cause);
            axis.StopEventPump();
            Assert.Throws<ArgumentException>(() => axis.SelfTest(stepMm: -1.0));

            Assert.Contains("\"kind\":\"direct\"", axis.LineJson());
            Assert.Equal("null", axis.GatewayStatusJson());
            Assert.Equal(string.Empty, axis.LineName);
            axis.SetPositionUnit(0.01);
            Assert.Throws<ArgumentException>(() => axis.SetPositionUnit(0.5));
            Assert.Throws<EcNotReadyException>(() => axis.GatewayMonitorJson());

            Assert.Equal(NativeCommandStrategy.Coil, axis.CommandStrategy);
            Assert.Equal("coil", axis.StopPath);
            axis.CommandStrategy = NativeCommandStrategy.Iai;
            Assert.Equal(NativeCommandStrategy.Iai, axis.CommandStrategy);
            Assert.Throws<ArgumentException>(() => axis.CommandStrategy = (NativeCommandStrategy)9);
            axis.CommandStrategy = NativeCommandStrategy.Coil;

            Assert.True(axis.SetSafetySpeed(true).SafetySpeed);
            Assert.False(axis.SetSafetySpeed(false).SafetySpeed);
            axis.DisableModbusControl();  // PIO control (PMSL off), then PC control again
            Assert.False(axis.Status().ModbusEnabled);
            axis.EnableModbusControl();
            Assert.True(axis.Status().ModbusEnabled);
            Assert.False(axis.SetActionMode(pc: false, safety: false).ModbusEnabled);  // the 0x45 cmd 0x0C toggle
            var mode = axis.SetActionMode(pc: true);
            Assert.True(mode.ModbusEnabled);
            Assert.False(mode.SafetySpeed);  // kept

            Assert.Throws<ArgumentException>(() => axis.TeachPosition(0, (NativeTeachMethod)5));
            Assert.Equal(0, axis.TeachPosition(0, NativeTeachMethod.Table).Row); // no opt-in needed
            // still accepted
            Assert.Equal(0, axis.TeachPosition(0, NativeTeachMethod.Table, allowNonvolatile: true).Row);
            var pushSpeed = axis.Limits().PushSpeed;
            Assert.True(pushSpeed is null || pushSpeed > 0.0);
        }

        [NativeFact]
        public void Lean_status_caps_home_pattern_and_completion_waits()
        {
            using var axis = NativeAxis.OpenSim("gripper", Fast);
            axis.Connect();
            var full = axis.Status();
            var lean = axis.ReadLeanStatus();
            Assert.Equal(8, lean.RawWords);
            Assert.Null(lean.CurrentMa);
            Assert.InRange(lean.PositionMm, full.PositionMm - 0.011, full.PositionMm + 0.011);
            Assert.Equal(full.ServoOn, lean.ServoOn);
            Assert.Equal(8, axis.ReadLeanStatus(2).RawWords);
            Assert.Throws<ArgumentException>(() => axis.ReadLeanStatus(-1));
            Assert.Equal(38400, axis.ReadBootBaudRate());

            axis.MoveTo(6.0, speed: 10.0, wait: false);
            var st = axis.WaitIdle(6.0, null, 10.0, NativeCompletion.Fast);
            Assert.InRange(st.PositionMm, 5.89, 6.11);
            Assert.True(st.InPosition);
            axis.MoveTo(4.0, speed: 10.0, wait: false);
            Assert.InRange(axis.WaitIdle(4.0, 0.1, 10.0, NativeCompletion.Safe).PositionMm, 3.89, 4.11);
            Assert.Throws<ArgumentException>(() => axis.WaitIdle(timeout: -1.0, completion: NativeCompletion.Safe));

            var caps = axis.MotionCaps();
            Assert.True(caps.SpeedCap > 0.0);
            Assert.True(caps.AccelCap > 0.0);
            Assert.True(caps.PushCap > 0.0);
            Assert.NotNull(caps.MinSpeedSource);
            _ = axis.ReadHomePattern();  // known or not, by the unit's directory

            axis.Home(wait: false);
            Thread.Sleep(50);
            Assert.True(Eventually(() =>
            {
                var s = axis.Status();
                return s.Homed && s.Idle;
            }));
            axis.FinishHome();
            Assert.True(axis.Status().Homed);
        }
    }
}
