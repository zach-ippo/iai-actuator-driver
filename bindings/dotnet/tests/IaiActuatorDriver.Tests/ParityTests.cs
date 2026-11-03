using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Simulation;
using IaiActuatorDriver.Tests.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// The public <see cref="Axis"/> members that mirror the Python edition: fast mode, the status monitor and its events, a change of
    /// speed or acceleration during a move, the acceleration pattern and a deceleration of its own, move handles, the backup and the
    /// unit information, the link (track, check, recover) and the self-test. Each runs on the simulator; no port is opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class ParityTests
    {
        private static (Axis Axis, SimulatedUnit Sim, FrameRecorder Rec) Open(SimProfile profile = SimProfile.Slider, double timeScale = 50.0, AxisOptions? options = null)
        {
            var sim = new SimulatedUnit(SimKit.Sim(profile, timeScale));
            var (axis, rec) = SimKit.Open(sim, options);
            return (axis, sim, rec);
        }

        private static bool Eventually(Func<bool> condition, double seconds = 20.0)
        {
            var end = DateTime.UtcNow.AddSeconds(seconds);
            while (DateTime.UtcNow < end)
            {
                if (condition())
                {
                    return true;
                }

                Thread.Sleep(20);
            }

            return condition();
        }

        // ------------------------------------------------------------------ fast mode

        [NativeFact]
        public void Fast_mode_sets_the_completion_rule_and_starts_the_monitor_and_other_settings_keep_the_rule()
        {
            var (axis, _, _) = Open();
            using (axis)
            {
                axis.Connect();
                Assert.False(axis.FastMode);
                Assert.Equal(CompletionRule.Safe, axis.Completion);
                Assert.Null(axis.Monitor);

                var on = axis.EnableFast();
                Assert.Equal(CompletionRule.Fast, on.Completion);
                Assert.True(on.Monitor);
                Assert.True(axis.FastMode);
                Assert.Equal(CompletionRule.Fast, axis.Completion);
                Assert.True(axis.Monitor!.Running);
                Assert.Equal(on.Notes, axis.FastNotes);
                Assert.Equal("fast", on.ToDictionary()["completion"]);
                Assert.InRange(axis.MoveTo(50.0, speed: 100.0)!.PositionMm, 49.89, 50.11);

                axis.SettleS = 0.2;  // another setting is pushed to the core: the rule it holds stays
                Assert.Equal(CompletionRule.Fast, axis.Completion);

                var off = axis.DisableFast();
                Assert.Equal(CompletionRule.Safe, off.Completion);
                Assert.False(off.Monitor);
                Assert.False(axis.FastMode);
                Assert.Equal(CompletionRule.Safe, axis.Completion);
                Assert.False(axis.Monitor?.Running ?? false);

                axis.Completion = CompletionRule.Fast;
                Assert.Equal(CompletionRule.Fast, axis.Completion);
                axis.Completion = CompletionRule.Safe;
                Assert.Equal(CompletionRule.Safe, axis.Completion);
            }
        }

        [NativeFact]
        public void Connect_can_enable_fast_mode_and_an_option_picks_the_completion_rule()
        {
            var (axis, _, _) = Open();
            using (axis)
            {
                axis.Connect(takeControl: true, fast: true);
                Assert.True(axis.FastMode);
                Assert.True(axis.Monitor!.Running);
            }

            var (pinned, _, _) = Open(options: new AxisOptions { Completion = CompletionRule.Fast, PollIntervalS = 0.005 });
            using (pinned)
            {
                Assert.Equal(CompletionRule.Fast, pinned.Completion);
                pinned.Connect();
                Assert.InRange(pinned.MoveTo(40.0, speed: 100.0)!.PositionMm, 39.89, 40.11);
            }
        }

        // ------------------------------------------------------------------ status monitor and events

        [NativeFact]
        public void The_monitor_samples_the_axis_and_raises_the_events_on_its_own_thread()
        {
            var (axis, _, _) = Open();
            using (axis)
            {
                axis.Connect();
                Assert.Null(axis.Monitor);
                var changes = new ConcurrentQueue<StatusChangedEventArgs>();
                var done = new ManualResetEventSlim();
                var handlerThread = 0;
                axis.StatusChanged += (_, _) => throw new InvalidOperationException("a handler that throws is ignored, and the ones after it still run");
                axis.StatusChanged += (_, e) => changes.Enqueue(e);
                axis.MotionDone += (_, e) =>
                {
                    Volatile.Write(ref handlerThread, Environment.CurrentManagedThreadId);
                    done.Set();
                };

                var monitor = axis.StartMonitor(new MonitorOptions { GapS = 0.002, LinkCheckS = null });
                Assert.Same(monitor, axis.Monitor);
                Assert.Same(monitor, axis.StartMonitor());  // one that runs is kept
                Assert.True(monitor.Running);
                var sample = monitor.WaitUpdate(timeout: 5.0);
                Assert.NotNull(sample);
                Assert.True(sample!.Seq > 0);
                Assert.NotNull(sample.Status);
                Assert.NotNull(monitor.WaitUpdate(sample.Seq, 5.0));

                axis.MoveTo(60.0, speed: 100.0);
                Assert.True(done.Wait(TimeSpan.FromSeconds(20)));
                Assert.NotEqual(Environment.CurrentManagedThreadId, Volatile.Read(ref handlerThread));
                Assert.True(Eventually(() => !changes.IsEmpty));
                Assert.All(changes, e => Assert.NotNull(e.Status));
                Assert.Contains(changes, e => e.Previous != null);

                Assert.NotNull(monitor.WaitDone(5.0));
                var latest = monitor.Latest!;
                Assert.True(latest.Polls > 0);
                Assert.True(monitor.Polls >= latest.Polls);
                Assert.True(monitor.IsDone(latest.Status!));
                Assert.False(MonitorFastDone(latest.Status!, target: 0.0));
                Assert.Equal(latest.Status!.PositionMm, monitor.Status!.PositionMm, 1);
                Assert.NotNull(monitor.WaitFor(s => s.InPosition, 5.0));
                Assert.Null(monitor.WaitFor(s => s.AlarmCode == 0x1FF, 0.3));  // never true: ends at the limit
                Assert.Equal("ok", monitor.LinkState);
                Assert.Null(monitor.Error);

                using (monitor.Suspend())
                {
                    Assert.True(monitor.Suspended);
                }

                Assert.False(monitor.Suspended);
                Assert.Throws<OperationCanceledException>(() => monitor.WaitFor(_ => false, 5.0, new CancellationToken(true)));

                axis.StopMonitor();
                Assert.False(axis.Monitor?.Running ?? false);
                axis.StopMonitor();  // none running: nothing happens
            }
        }

        private static bool MonitorFastDone(Status status, double target) => AxisMonitor.FastDone(status, target, 0.1);

        [NativeFact]
        public void Closing_the_axis_stops_its_monitor()
        {
            var (axis, _, _) = Open();
            axis.Connect();
            var monitor = axis.StartMonitor();
            Assert.True(monitor.Running);
            axis.Close();
            Assert.True(Eventually(() => !monitor.Running, 5.0));
        }

        // ------------------------------------------------------------------ changing a move, patterns, deceleration

        [NativeFact]
        public void A_move_in_progress_takes_a_new_speed_and_acceleration()
        {
            var (axis, _, _) = Open(timeScale: 1.0);
            using (axis)
            {
                axis.Connect();
                Assert.Throws<EcNotReadyException>(() => axis.ChangeSpeed(50.0));  // no move of this axis is running
                axis.MoveTo(130.0, speed: 20.0, wait: false);  // from 73.95 mm: almost 3 s
                Assert.Null(axis.ChangeAccel(0.2));
                Assert.Throws<SoftLimitException>(() => axis.ChangeSpeed(double.NaN));
                Assert.Throws<SoftLimitException>(() => axis.ChangeSpeed(5000.0));  // above the unit's own maximum
                var st = axis.ChangeSpeed(100.0, wait: true, timeout: 30.0)!;
                Assert.InRange(st.PositionMm, 129.89, 130.11);
                Assert.Throws<EcNotReadyException>(() => axis.ChangeAccel(0.2));
            }
        }

        [NativeFact]
        public void Accel_patterns_and_a_deceleration_of_its_own_reach_the_unit()
        {
            var (axis, _, _) = Open();
            using (axis)
            {
                axis.Connect();
                Assert.InRange(axis.MoveTo(100.0, accelPattern: AccelPattern.SMotion)!.PositionMm, 99.89, 100.11);
                Assert.InRange(axis.MoveBy(-10.0, accelPattern: AccelPattern.Filter, completion: CompletionRule.Fast)!.PositionMm, 89.89, 90.11);
                Assert.Throws<ArgumentException>(() => axis.MoveTo(90.0, accelPattern: (AccelPattern)9));
                Assert.InRange(axis.CheckMove(80.0, accelPattern: AccelPattern.Trapezoid).PositionMm, 89.89, 90.11);  // nothing moved
            }

            var (pcon, _, rec) = Open(SimProfile.Pcon);
            using (pcon)
            {
                pcon.Connect();
                Assert.InRange(pcon.MoveTo(120.0, speed: 100.0, accelG: 0.3, decelG: 0.1)!.PositionMm, 119.89, 120.11);
                Assert.NotEmpty(SimKit.Sent(rec, 0x17));  // the command-profile block
                var e = Assert.Throws<SoftLimitException>(() => pcon.MoveTo(100.0, speed: 100.0, accelG: 0.3, decelG: 1.5));
                Assert.Contains("MAXD", e.Message);
            }
        }

        [NativeFact]
        public void Move_sequences_and_teach_run_on_the_axis()
        {
            var (axis, sim, _) = Open();
            using (axis)
            {
                axis.Connect();
                var statuses = axis.MoveSequence(new[] { new SequencePoint(20.0), new SequencePoint(40.0) { Speed = 80.0 }, 60.0 }, speed: 100.0);
                Assert.Equal(3, statuses.Count);
                Assert.InRange(statuses[2].PositionMm, 59.89, 60.11);
                Assert.InRange(axis.MoveSequence(new[] { 50.0, 30.0 }, speed: 100.0).Last().PositionMm, 29.89, 30.11);
                var e = Assert.Throws<SoftLimitException>(() => axis.MoveSequence(new[] { 20.0, 1e6 }));  // checked before the first frame
                Assert.Contains("point 1", e.Message);
                Assert.Throws<SoftLimitException>(() => axis.MoveSequence(new[] { 20.0, double.NaN }));
                Assert.InRange(axis.Status().PositionMm, 29.89, 30.11);
                Assert.Throws<ArgumentException>(() => axis.MoveSequence(Array.Empty<double>()));

                var taught = axis.TeachPosition(0); // no opt-in needed
                Assert.InRange(taught.PositionMm, 29.89, 30.11);
                Assert.True(axis.TeachPosition(0, allowNonvolatile: true).Skipped);  // already holds the position: not written again
                Assert.Equal(1, sim.NonvolatileWrites);
            }
        }

        [NativeFact]
        public void The_lean_status_the_line_the_stop_path_the_minimum_speed_and_a_waits_own_completion_rule()
        {
            var (axis, _, _) = Open(SimProfile.Gripper);
            using (axis)
            {
                axis.Connect();
                var full = axis.Status();
                var lean = axis.ReadLeanStatus();
                Assert.Null(lean.CurrentMa);  // the 8 words carry no current
                Assert.InRange(lean.PositionMm, full.PositionMm - 0.011, full.PositionMm + 0.011);
                Assert.Equal(full.ServoOn, lean.ServoOn);
                Assert.Throws<ArgumentException>(() => axis.ReadLeanStatus(-1));
                Assert.Null(axis.LineName);  // a direct connection
                Assert.Equal("coil", axis.StopPath);
                Assert.NotEmpty(axis.RecommendedMinSpeed().Source);

                axis.MoveTo(6.0, speed: 10.0, wait: false);
                var st = axis.WaitIdle(6.0, null, 10.0, completion: CompletionRule.Fast);
                Assert.InRange(st.PositionMm, 5.89, 6.11);
                Assert.True(st.InPosition);
                axis.MoveTo(4.0, speed: 10.0, wait: false);
                Assert.InRange(axis.WaitIdle(4.0, 0.1, 10.0, completion: CompletionRule.Safe).PositionMm, 3.89, 4.11);
            }
        }

        // ------------------------------------------------------------------ move handles

        [NativeFact]
        public async Task A_started_move_is_polled_waited_for_and_cancelled_through_its_handle()
        {
            var (axis, _, _) = Open();
            using (axis)
            {
                axis.Connect();
                using (var move = axis.StartMoveTo(100.0, speed: 100.0))
                {
                    Assert.Equal(100.0, move.Target, 6);
                    Assert.Equal(CompletionRule.Safe, move.Completion);
                    Assert.True(move.TimeoutS > 0.0);
                    Assert.False(move.Cancelled);
                    Assert.Null(move.Result);
                    Status? done = null;
                    for (var i = 0; i < 400 && done is null; i++)
                    {
                        done = move.Poll(0.05);  // short slices: the move runs on between them
                    }

                    Assert.NotNull(done);
                    Assert.InRange(done!.PositionMm, 99.89, 100.11);
                    Assert.True(move.IsDone());
                    Assert.Equal(done, move.Result);
                    Assert.Equal(done, move.Wait());
                    Assert.Equal(true, move.Info.ToDictionary()["done"]);
                    Assert.Contains("MoveHandle(target=100", move.ToString());
                }

                using (var by = axis.StartMoveBy(-10.0, speed: 100.0, completion: CompletionRule.Fast))
                {
                    Assert.Equal(CompletionRule.Fast, by.Completion);
                    var st = await by.WaitAsync(30.0);
                    Assert.InRange(st.PositionMm, 89.89, 90.11);
                }

                using (var jog = axis.StartJog(2.0, speed: 20.0))
                {
                    Assert.InRange(jog.Wait(30.0).PositionMm, 91.89, 92.11);
                }

                Assert.Throws<SoftLimitException>(() => axis.StartMoveTo(1e6));
                Assert.Throws<SoftLimitException>(() => axis.StartMoveBy(double.NaN));
                Assert.Throws<SoftLimitException>(() => axis.StartJog(double.PositiveInfinity));
            }
        }

        [NativeFact]
        public void A_handle_cancels_its_move_and_a_cancelled_token_ends_a_wait()
        {
            var (axis, _, _) = Open(timeScale: 1.0);
            using (axis)
            {
                axis.Connect();
                var move = axis.StartMoveTo(130.0, speed: 20.0);  // from 73.95 mm: almost 3 s
                Thread.Sleep(300);
                var stopped = move.Cancel();
                Assert.True(move.Cancelled);
                Assert.Equal(stopped, move.Result);
                Assert.InRange(stopped.PositionMm, 74.0, 129.0);  // stopped on the way
                move.Dispose();
                Assert.True(move.IsClosed);

                using var waiting = axis.StartMoveTo(40.0, speed: 20.0);  // back toward 40 mm: about 2 s
                using var cts = new CancellationTokenSource(TimeSpan.FromMilliseconds(300));
                Assert.Throws<OperationCanceledException>(() => waiting.Wait(30.0, cts.Token));
                Assert.True(axis.Status().PositionMm > 41.0);  // the cancel sent a decel stop
            }
        }

        /// <summary>A transport over a simulated unit that throws what it is told to on the next write.</summary>
        private sealed class ThrowingTransport : ITransport
        {
            private readonly SimulatedUnit _unit;

            public ThrowingTransport(SimulatedUnit unit) => _unit = unit;

            public Exception? Failure { get; set; }

            public int BaudRate
            {
                get => _unit.BaudRate;
                set => _unit.BaudRate = value;
            }

            public TimeSpan ReadTimeout { get; set; } = TimeSpan.FromSeconds(0.2);

            public void Write(byte[] buffer, int offset, int count)
            {
                if (Failure is { } failure)
                {
                    throw failure;
                }

                _unit.Write(buffer, offset, count);
            }

            public int Read(byte[] buffer, int offset, int count) => _unit.Read(buffer, offset, count);

            public void ResetInputBuffer() => _unit.ResetInputBuffer();

            public void SendBreak(TimeSpan duration) => _unit.SendBreak(duration);

            public void Close() => _unit.Close();

            public void Dispose() => _unit.Dispose();
        }

        [NativeFact]
        public void A_managed_transports_own_exception_comes_through_the_new_calls()
        {
            var sim = new SimulatedUnit(SimKit.Sim(SimProfile.Slider, 50.0));
            var transport = new ThrowingTransport(sim);
            using var axis = new Axis(new ModbusRtu(transport, minGap: TimeSpan.Zero), 1, new AxisOptions { PollIntervalS = 0.005 }, ownsBus: true);
            axis.Connect();
            using var move = axis.StartMoveTo(100.0, speed: 100.0);

            var io = new IOException("cable pulled");
            transport.Failure = io;
            Assert.Same(io, Assert.ThrowsAny<ModbusException>(() => move.Poll(0.2)).InnerException);  // a handle's wait
            Assert.Same(io, Assert.ThrowsAny<ModbusException>(() => axis.StartMoveTo(50.0)).InnerException);
            Assert.Same(io, Assert.ThrowsAny<ModbusException>(() => axis.ChangeSpeed(30.0)).InnerException);
            Assert.Same(io, Assert.ThrowsAny<ModbusException>(() => axis.Backup()).InnerException);
            Assert.Same(io, Assert.ThrowsAny<ModbusException>(() => axis.UnitInfo()).InnerException);

            var odd = new NotImplementedException("no frames today");  // any other exception is the transport's own
            transport.Failure = odd;
            Assert.Same(odd, Assert.Throws<NotImplementedException>(() => axis.ReadLeanStatus()));
            transport.Failure = null;
        }

        // ------------------------------------------------------------------ backup and unit information

        [NativeFact]
        public void A_backup_is_read_only_and_can_be_written_to_a_file()
        {
            var (axis, sim, _) = Open(SimProfile.Gripper);
            var path = Path.Combine(Path.GetTempPath(), "iad_cs_parity_" + Guid.NewGuid().ToString("N") + ".json");
            try
            {
                using (axis)
                {
                    axis.Connect();
                    sim.ClearRequests();
                    var data = axis.Backup(path);
                    Assert.Equal("ec_driver.backup/1", ((IDictionary<string, object?>)data["meta"]!)["schema"]);
                    Assert.All(new[] { "identity", "directory", "parameters", "positions", "alarms", "maintenance", "errors" }, k => Assert.True(data.ContainsKey(k), k));
                    Assert.True(File.Exists(path));
                    Assert.Equal(path, ((IDictionary<string, object?>)data["meta"]!)["path"]);
                    Assert.Contains("ec_driver.backup/1", File.ReadAllText(path));
                    Assert.All(sim.Requests, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));  // FC03 / FC11 only

                    var rows = (IDictionary<string, object?>)data["positions"]!;
                    Assert.NotEmpty((IList<object?>)rows["rows"]!);
                    var two = axis.Backup(new[] { 0, 1 });
                    Assert.Equal(2, ((IList<object?>)((IDictionary<string, object?>)two["positions"]!)["rows"]!).Count);
                    Assert.Throws<ArgumentException>(() => axis.Backup(rows: "bogus rows"));
                    Assert.Contains("\"identity\"", axis.BackupJson());
                    Assert.StartsWith("{", JsonSerializer.Serialize(data));
                }
            }
            finally
            {
                File.Delete(path);
            }
        }

        [NativeFact]
        public void Backups_of_several_axes_go_to_one_folder()
        {
            var dir = Path.Combine(Path.GetTempPath(), "iad_cs_parity_" + Guid.NewGuid().ToString("N"));
            try
            {
                var (a, _, _) = Open(SimProfile.Gripper);
                var (b, _, _) = Open(SimProfile.Slider);
                using (a)
                using (b)
                {
                    a.Connect();
                    b.Connect();
                    var files = Axis.BackupAll(new[] { a, b }, dir, new[] { "grip", "slide:1" }, rows: "0,1");
                    Assert.Equal(new[] { "grip", "slide:1" }, files.Keys.ToArray());
                    Assert.True(File.Exists(Path.Combine(dir, "grip.json")));
                    Assert.True(File.Exists(Path.Combine(dir, "slide_1.json")));
                    Assert.Throws<ArgumentException>(() => Axis.BackupAll(new[] { a, b }, dir, new[] { "one" }));
                }
            }
            finally
            {
                if (Directory.Exists(dir))
                {
                    Directory.Delete(dir, recursive: true);
                }
            }
        }

        [NativeFact]
        public void Unit_info_has_the_python_sections_and_the_limits_as_an_extra()
        {
            var (axis, sim, _) = Open(SimProfile.Gripper);
            using (axis)
            {
                axis.Connect();
                sim.ClearRequests();
                var info = axis.UnitInfo();
                Assert.Equal(
                    new[] { "meta", "identity", "parameters", "monitor", "status_words", "io", "control_words", "alarms", "maintenance", "positions", "errors", "limits" }.OrderBy(k => k),
                    info.Keys.OrderBy(k => k));
                Assert.All(new[] { "identity", "parameters", "monitor", "status_words", "io", "control_words", "alarms", "maintenance", "positions", "limits" }, k => Assert.NotNull(info[k]));
                Assert.Equal(Axis.UnitInfoSchema, ((IDictionary<string, object?>)info["meta"]!)["schema"]);
                Assert.Equal("ec_driver.unit_info/1", Axis.UnitInfoSchema);
                Assert.Contains("DSS1", (IDictionary<string, object?>)info["status_words"]!);
                Assert.Contains("position", (IDictionary<string, object?>)info["limits"]!);
                Assert.Contains("stored_rows", (IDictionary<string, object?>)info["positions"]!);
                Assert.Empty((IList<object?>)info["errors"]!);
                Assert.All(sim.Requests, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));  // read-only
                Assert.Equal(Axis.UnitInfoSections.Count + 1, ((IList<object?>)((IDictionary<string, object?>)info["meta"]!)["sections"]!).Count);

                var some = axis.UnitInfo(new[] { "identity", "maintenance" });
                Assert.NotNull(some["identity"]);
                Assert.NotNull(some["maintenance"]);
                Assert.Null(some["alarms"]);
                Assert.Null(some["limits"]);

                var only = axis.UnitInfo(new[] { "limits" });  // the extra alone asks the unit for nothing else
                Assert.NotNull(only["limits"]);
                Assert.Null(only["identity"]);
                Assert.Throws<ArgumentException>(() => axis.UnitInfo(new[] { "nope" }));
                Assert.StartsWith("{", JsonSerializer.Serialize(info));
            }
        }

        // ------------------------------------------------------------------ link: track, check, recover

        [NativeFact]
        public void A_tracked_unit_is_checked_and_recovered_after_a_power_cycle()
        {
            var (axis, sim, _) = Open(SimProfile.Gripper);
            using (axis)
            {
                axis.Connect();
                Assert.Equal("ok", axis.LinkState);
                Assert.Null(axis.Session);  // none until Track: the identity connect read is only the baseline of a recovery

                var session = axis.Track("G");
                Assert.Equal("G", session.Name);
                Assert.NotNull(session.SerialNumber);
                Assert.Equal(session.SerialNumber, axis.Session!.SerialNumber);
                Assert.True(session.AlarmHistoryCount > 0);
                var check = axis.CheckLink();
                Assert.True(check.Ok);
                Assert.Equal("ok", check.State);
                Assert.NotNull(check.Status);
                Assert.Empty(check.Reasons);
                Assert.Equal("ok", check.ToDictionary()["state"]);

                var events = new ConcurrentQueue<RecoveryEvent>();
                axis.LinkChanged += (_, e) => events.Enqueue(e.Event);
                axis.Servo(true);
                sim.PowerCycle();  // the unit restarts: uptime drops, PC control is lost
                var restarted = axis.CheckLink();
                Assert.Equal("unit_restarted", restarted.State);
                Assert.NotEmpty(restarted.Reasons);
                Assert.Equal("unit_restarted", axis.LinkState);
                Assert.True(Eventually(() => events.Any(e => e.Kind == "unit_restarted")));

                var report = axis.Recover(10.0, new RecoveryPolicy { PollS = 0.02 });  // never moves anything
                Assert.True(report.SameUnit);
                Assert.True(report.Restarted);
                Assert.True(report.ControlRestored || report.ModbusControl == true);
                Assert.Equal("unit_restarted", report.Cause);
                Assert.NotEmpty(report.Actions);
                Assert.Contains("recovered", report.Summary);
                Assert.Equal("ok", axis.LinkState);
                Assert.True(Eventually(() => events.Any(e => e.Kind == "recovered")));
                Assert.Equal(report.Summary, report.ToDictionary()["summary"]);
                Assert.StartsWith("{", report.ToJson());
                Assert.Equal(sim.PositionMm, axis.Status().PositionMm, 2);

                axis.NoteTarget(5.0);
                axis.NoteControl(modbusControl: true);
                axis.Forget();
                Assert.Null(axis.Session);
            }
        }

        [NativeFact]
        public void A_recovery_without_a_session_or_for_another_unit_is_refused()
        {
            var (fresh, _, _) = Open(SimProfile.Gripper);
            using (fresh)
            {
                Assert.Throws<RecoveryException>(() => fresh.Recover(1.0, new RecoveryPolicy { PollS = 0.02, RequireSession = true }));
            }

            var dir = Path.Combine(Path.GetTempPath(), "iad_cs_parity_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                var (gripper, _, _) = Open(SimProfile.Gripper);
                var (slider, _, _) = Open(SimProfile.Slider);
                using (gripper)
                using (slider)
                {
                    gripper.Connect();
                    slider.Connect();
                    var file = Path.Combine(dir, "gripper.json");
                    var backup = gripper.BackupJson(file);
                    Assert.Throws<ArgumentException>(() => slider.TrackBackup("not json"));
                    slider.TrackBackup(backup, "S");
                    Assert.Equal("S", slider.Session!.Name);
                    var replaced = Assert.Throws<UnitReplacedException>(() => slider.Recover(2.0, new RecoveryPolicy { PollS = 0.02 }));  // the gripper's backup on the slider
                    Assert.NotNull(replaced.Report);
                    Assert.False(replaced.Report!.SameUnit);
                    slider.TrackBackupFile(file, "S2");
                    Assert.Equal("S2", slider.Session!.Name);
                    Assert.Throws<IOException>(() => slider.TrackBackupFile(Path.Combine(dir, "missing.json")));
                    slider.Forget();
                    Assert.Equal("T", slider.TrackIdentity("T").Name);
                }
            }
            finally
            {
                Directory.Delete(dir, recursive: true);
            }
        }

        // ------------------------------------------------------------------ the self-test

        [NativeFact]
        public void The_self_test_reports_each_line_to_the_caller_and_returns_the_report()
        {
            var (axis, sim, _) = Open(SimProfile.Gripper);
            using (axis)
            {
                axis.Connect();
                sim.ClearRequests();
                var lines = new List<CheckItem>();
                var threads = new HashSet<int>();
                var report = axis.RunCheck(new CheckOptions { PortLabel = "SIM" }, item =>
                {
                    lines.Add(item);
                    threads.Add(Environment.CurrentManagedThreadId);
                });
                Assert.True(report.Passed);
                Assert.Equal("SIM", report.Port);
                Assert.Equal(1, report.Slave);
                Assert.True(report.Items.Count > 5);
                Assert.Equal(report.Items.Select(i => i.Name), lines.Select(i => i.Name));  // every line, once, in order
                Assert.Equal(new[] { Environment.CurrentManagedThreadId }, threads);  // on the calling thread
                Assert.Equal("link", report.Items[0].Name);
                Assert.Equal(CheckResult.Pass, report.Items[0].Result);
                Assert.Contains("PASS", report.Items[0].ToLine());
                Assert.All(report.Items, i => Assert.NotEqual(CheckResult.Fail, i.Result));
                Assert.NotNull(report.Model);
                Assert.NotNull(report.UnitInfo);
                Assert.NotNull(report.Stroke);
                Assert.All(sim.Requests, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));  // read-only without motion

                var path = Path.Combine(Path.GetTempPath(), "iad_cs_parity_" + Guid.NewGuid().ToString("N") + ".json");
                try
                {
                    Assert.Equal(path, report.WriteJson(path));
                    using var doc = JsonDocument.Parse(File.ReadAllText(path));
                    Assert.Equal("PASS", doc.RootElement.GetProperty("result").GetString());
                    Assert.Equal(report.Items.Count, doc.RootElement.GetProperty("items").GetArrayLength());
                }
                finally
                {
                    File.Delete(path);
                }

                Assert.Throws<ArgumentException>(() => axis.RunCheck(new CheckOptions { StepMm = -1.0 }));
                var broken = Assert.Throws<InvalidOperationException>(() => axis.RunCheck(null, _ => throw new InvalidOperationException("stop here")));
                Assert.Equal("stop here", broken.Message);
                Assert.True(axis.Status().Idle);  // the axis is usable again
                Assert.Throws<OperationCanceledException>(() => axis.RunCheck(null, null, new CancellationToken(true)));
            }
        }

        [NativeFact]
        public void The_motion_check_moves_out_and_back_and_restores_the_state()
        {
            var (axis, sim, _) = Open(SimProfile.Slider);
            using (axis)
            {
                axis.Connect();
                var before = sim.PositionMm;
                var report = axis.RunCheck(new CheckOptions { Motion = true, StepMm = 5.0, SpeedMmS = 50.0 });
                Assert.True(report.Passed);
                Assert.True(report.MotionRequested);
                Assert.Contains(report.Items, i => i.Name.Contains("motion") && i.Result == CheckResult.Pass);
                Assert.Equal(before, sim.PositionMm, 1);
            }
        }
    }
}
