using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
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
    /// The ABI 0.2 additions of <see cref="NativeCell"/> on simulated lines: per-axis moves, pause / resume / cancel,
    /// monitors as C# events, recovery, lines, backups and coordinated plans. The scenarios follow
    /// core/tests/test_c_api_ext.cpp. No hardware.
    /// </summary>
    public class NativeCellExtSimTests
    {
        private static NativeSimOptions Fast => new NativeSimOptions { TimeScale = 20 };

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
        public void Per_axis_moves_pause_resume_and_cancel()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider", "pcon" }, sim: Fast);
            var moved = cell.MoveAll(new[]
            {
                new NativeAxisMove("SIM1:1", 30.0) { Speed = 50.0 },
                new NativeAxisMove("SIM2:1", 60.0) { Speed = 50.0, AccelG = 0.3, DecelG = 0.1 },
            })!;
            Assert.Equal(new[] { "SIM1:1", "SIM2:1" }, moved.Names);
            Assert.InRange(moved["SIM1:1"].PositionMm, 29.89, 30.11);
            Assert.InRange(moved["SIM2:1"].PositionMm, 59.89, 60.11);

            var refused = Assert.Throws<CellException>(() => cell.MoveAll(new[] { new NativeAxisMove("SIM1:1", 30.0), new NativeAxisMove("SIM2:1", 1e6) }));
            Assert.IsType<SoftLimitException>(refused.Errors["SIM2:1"]);
            Assert.Throws<ArgumentOutOfRangeException>(() => cell.MoveAll(new[] { new NativeAxisMove("SIM9:1", 1.0) }));
            Assert.Throws<ArgumentException>(() => cell.MoveAll(Array.Empty<NativeAxisMove>()));

            // Per-axis values: the pcon decelerates at a value of its own, so its move takes the command-profile path
            // (FC 0x17), the slider's the 0x9900 block.
            var profileFrames = 0;
            var pcon = cell["SIM2:1"];
            pcon.SetFrameHandler((direction, frame) =>
            {
                if (direction == FrameDirection.Tx && frame.Length > 2 && frame[1] == 0x17)
                {
                    Interlocked.Increment(ref profileFrames);
                }
            });
            var values = cell.MoveAllValues(
                new Dictionary<string, double> { ["SIM1:1"] = 20.0, ["SIM2:1"] = 50.0 },
                speeds: new Dictionary<string, double> { ["SIM1:1"] = 40.0 },
                accelsG: new Dictionary<string, double> { ["SIM2:1"] = 0.3 },
                decelsG: new Dictionary<string, double> { ["SIM2:1"] = 0.1 })!;
            Assert.InRange(values["SIM1:1"].PositionMm, 19.89, 20.11);
            Assert.InRange(values["SIM2:1"].PositionMm, 49.89, 50.11);
            Assert.True(Volatile.Read(ref profileFrames) > 0);
            pcon.SetFrameHandler(null);
            var far = Assert.Throws<CellException>(() => cell.MoveAllValues(new Dictionary<string, double> { ["SIM1:1"] = 20.0, ["SIM2:1"] = 1e6 }));
            Assert.IsType<SoftLimitException>(far.Errors["SIM2:1"]);
            Assert.InRange(cell.MoveAll(new Dictionary<string, double> { ["SIM2:1"] = 40.0 }, speed: 50.0, accelG: 0.3, decelG: 0.1)!["SIM2:1"].PositionMm, 39.89, 40.11);

            Assert.Equal(2, cell.PauseAll().Count);
            Assert.Equal(2, cell.ResumeAll().Count);
            cell.Cancel();  // nothing waits: nothing happens
            var unknown = Assert.Throws<CellException>(() => cell.Cancel(new[] { "SIM9:1" }));  // an error of its own
            Assert.IsType<ArgumentOutOfRangeException>(unknown.Errors["SIM9:1"]);
            Assert.Equal(2, cell.StatusAll().Count);
        }

        [NativeFact]
        public void Cancel_calls_reach_every_listed_axis_then_report_the_unknown_names()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider", "pcon" }, sim: Fast);
            // a remembered list naming axes that dropped out of a reopened cell: the axes the cell has are latched all the same
            var latch = Assert.Throws<CellException>(() => cell.CancelLatch(new[] { "SIM9:1", "SIM1:1", "SIM8:1", "SIM9:1" }));
            Assert.Equal(new[] { "SIM8:1", "SIM9:1" }, latch.Errors.Keys.OrderBy(k => k, StringComparer.Ordinal).ToArray());
            Assert.All(latch.Errors.Values, e => Assert.IsType<ArgumentOutOfRangeException>(e));
            Assert.True(cell["SIM1:1"].CancelLatched);
            Assert.False(cell["SIM2:1"].CancelLatched);  // not listed
            cell.CancelLatch(new[] { "SIM2:1" });
            var reset = Assert.Throws<CellException>(() => cell.CancelReset(new[] { "SIM1:1", "SIM9:1" }));
            Assert.Equal(new[] { "SIM9:1" }, reset.Errors.Keys.ToArray());
            Assert.False(cell["SIM1:1"].CancelLatched);
            Assert.True(cell["SIM2:1"].CancelLatched);
            Assert.Throws<CellException>(() => cell.CancelReset(new[] { "SIM9:1" }));  // only unknown names: nothing touched
            Assert.True(cell["SIM2:1"].CancelLatched);
            cell.CancelReset();
            Assert.False(cell["SIM2:1"].CancelLatched);
        }

        [NativeFact]
        public void Cancel_ends_a_waiting_axis_though_the_list_names_an_unknown_one()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider" }, sim: new NativeSimOptions { TimeScale = 1, PositionMm = 20.0 });
            var axis = cell["SIM1:1"];
            Exception? ended = null;
            var mover = new Thread(() =>
            {
                try
                {
                    axis.MoveTo(140.0, speed: 20.0);
                }
                catch (Exception exc)
                {
                    ended = exc;
                }
            });
            mover.Start();
            try
            {
                Thread.Sleep(500);
                var failure = Assert.Throws<CellException>(() => cell.Cancel(new[] { "SIM9:1", "SIM1:1" }));
                Assert.Equal(new[] { "SIM9:1" }, failure.Errors.Keys.ToArray());
                Assert.True(mover.Join(TimeSpan.FromSeconds(3)), "the cancel did not reach the waiting axis");
            }
            finally
            {
                mover.Join(TimeSpan.FromSeconds(30));
            }

            Assert.IsType<OperationCanceledException>(ended);
            Assert.True(axis.Status().PositionMm < 139.0);
        }

        [NativeFact]
        public void Monitors_raise_every_axis_events_on_the_cell()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider", "pcon" }, sim: Fast);
            var done = new ConcurrentDictionary<string, bool>();
            var statuses = 0;
            cell.StatusChanged += (s, e) => Interlocked.Increment(ref statuses);
            cell.MotionDone += (s, e) => done[e.Axis] = true;
            cell.StartMonitors();
            Assert.True(cell.EventPumpRunning);
            cell.MoveAll(new Dictionary<string, double> { ["SIM1:1"] = 50.0, ["SIM2:1"] = 40.0 });
            Assert.True(Eventually(() => done.ContainsKey("SIM1:1") && done.ContainsKey("SIM2:1")));
            Assert.True(Volatile.Read(ref statuses) > 0);
            cell.StartMonitors(new[] { "SIM1:1" });  // already fed: not twice
            cell.StopMonitors();
            cell.StopEventPump();
            Assert.False(cell.EventPumpRunning);

            Assert.Contains("\"SIM1\":{", cell.LinesJson());
            Assert.Equal("{}", cell.GatewayStatusJson());
            cell.SetPositionUnit("SIM1", 0.01);
            cell.SetPositionUnit("SIM1", 0.01, new[] { 1 });
            Assert.Throws<ArgumentOutOfRangeException>(() => cell.SetPositionUnit("NOPE", 0.01));
            Assert.Throws<EcNotReadyException>(() => cell.GatewayMonitorJson("SIM1"));  // no RCON gateway on a direct line
            Assert.Contains("\"readings\":{\"SIM1:1\"", cell.MonitorAllJson());
        }

        [NativeFact]
        public void Track_check_links_recover_and_back_up()
        {
            var dir = Path.Combine(Path.GetTempPath(), "iad_cs_cell_" + Guid.NewGuid().ToString("N"));
            try
            {
                using var cell = NativeCell.OpenSim(new[] { "slider", "pcon" }, sim: Fast);
                Assert.Throws<EcNotReadyException>(() => cell.LastResultJson());
                Assert.Equal(2, cell.TrackAll().Count);
                Assert.Contains("\"SIM1:1\":{", cell.LastResultJson());

                var checks = cell.CheckLinks();
                Assert.Equal(new[] { "SIM1:1", "SIM2:1" }, checks.Select(c => c.Key));
                Assert.All(checks, c => Assert.True(c.Value.Ok));
                Assert.Equal("SIM2:1", Assert.Single(cell.CheckLinks(new[] { "SIM2:1" })).Key);

                var recovered = cell.RecoverAll(new[] { "SIM1:1" }, 10.0, new NativeRecoveryPolicy { PollS = 0.02 });
                Assert.Equal(new[] { "SIM1:1" }, recovered.Names);
                Assert.Contains("\"reports\":{\"SIM1:1\"", cell.LastResultJson());

                cell.MonitorLinks(0.05);
                Assert.True(cell.EventPumpRunning);
                Assert.Throws<ArgumentException>(() => cell.MonitorLinks(-1.0));
                Thread.Sleep(100);
                cell.StopLinkMonitor();
                cell.StopLinkMonitor();  // no-op

                var saved = cell.BackupJson(dir, new[] { "SIM2:1" }, "used");
                Assert.Contains("\"SIM2:1\":", saved);
                Assert.DoesNotContain("error:", saved);
                Assert.Equal(saved, cell.LastResultJson());
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
        public void Coordinated_plan_and_refusals()
        {
            using var cell = NativeCell.OpenSim(new[] { "slider", "slider" }, sim: new NativeSimOptions { TimeScale = 20, PositionMm = 50.0 });
            // min_step_s pinned (about the model's shortest step for one axis per port): without it the plan measures
            // the line now, and a busy PC stretches that measurement until the circle's 98 ms steps are refused
            var plan = cell.CoordPlanJson(
                "{\"axes\": {\"x\": \"SIM1:1\", \"y\": \"SIM2:1\"}, "
                + "\"path\": {\"shape\": \"circle\", \"center\": [50, 40], \"radius\": 10}, "
                + "\"speed\": 20, \"accel_g\": 0.3, \"min_step_s\": 0.045}");
            Assert.Contains("\"mode\":\"stream\"", plan);
            Assert.Contains("\"summary\":", plan);
            Assert.InRange(cell.StatusAll()["SIM1:1"].PositionMm, 49.99, 50.01);  // a dry run: nothing moves

            const string tooFast = "{\"axes\": {\"x\": \"SIM1:1\", \"y\": \"SIM2:1\"}, "
                + "\"path\": {\"segments\": [{\"line\": {\"x\": 10, \"y\": 20}}]}, \"speed\": 100000}";
            var e = Assert.Throws<SoftLimitException>(() => cell.CoordPlanJson(tooFast));
            Assert.Contains("\"kind\":\"InfeasiblePathError\"", NativeErrors.ErrorJsonOf(e));
            Assert.Throws<SoftLimitException>(() => cell.CoordRunJson(tooFast));  // refused before anything is sent
            Assert.InRange(cell.StatusAll()["SIM1:1"].PositionMm, 49.99, 50.01);
            // a timeout that is not above 0 ("wait forever" in VB and LabVIEW) is refused before anything is sent
            const string head = "{\"axes\": {\"x\": \"SIM1:1\", \"y\": \"SIM2:1\"}, "
                + "\"path\": {\"segments\": [{\"line\": {\"x\": 70, \"y\": 60}}]}, \"speed\": 40, \"accel_g\": 0.3, \"timeout_s\": ";
            foreach (var bad in new[] { "0", "-1", "-0.5" })
            {
                var refused = Assert.Throws<ArgumentException>(() => cell.CoordRunJson(head + bad + "}"));
                Assert.Contains("nothing sent", refused.Message);
                Assert.Throws<ArgumentException>(() => cell.CoordPlanJson(head + bad + "}"));
            }

            Assert.InRange(cell.StatusAll()["SIM1:1"].PositionMm, 49.99, 50.01);
            Assert.Contains("\"completed\":true", cell.CoordRunJson(head + "30}"));
            Assert.InRange(cell.StatusAll()["SIM1:1"].PositionMm, 69.89, 70.11);
            Assert.Throws<ArgumentException>(() => cell.CoordPlanJson("not json"));
            Assert.Throws<ArgumentException>(() => cell.CoordPlanJson(
                "{\"path\": {\"segments\": [{\"line\": {\"SIM1:1\": 10}}]}, \"speed\": 10, \"sped\": 1}"));
        }
    }
}
