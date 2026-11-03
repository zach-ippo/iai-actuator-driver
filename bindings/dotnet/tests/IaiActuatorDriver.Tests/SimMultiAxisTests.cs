using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// Several axes against the simulator's <see cref="SimLine"/>: one line with several units, one unit per port, a shared
    /// line driven from two threads, the STOP-first rule of <see cref="Cell.StopAll"/>, the broadcast baud change and cells
    /// of 16 axes and more. Real time (the units' clock is sped up), so the tests run one at a time.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class SimMultiAxisTests
    {
        /// <summary>Quick scans (the simulator answers at once), no per-axis identification, stroke 0..150, 5 ms polls.</summary>
        private static readonly CellOptions Quick = new CellOptions
        {
            ScanTimeout = TimeSpan.FromMilliseconds(10),
            AxisOptions = SimKit.MakeOptions(),
            Identify = false,
        };

        private static SimLine Line(string name, IEnumerable<int> slaves, double timeScale = 20.0, SimProfile profile = SimProfile.Slider) =>
            SimLine.WithSlaves(slaves, SimKit.Sim(profile, timeScale), name);

        /// <summary>
        /// A line of sliders on stepped clocks (<see cref="SteppedClockUnit"/>): each advances <paramref name="stepS"/> at
        /// every frame on the line (every unit sees every frame), so a move lasts a number of requests, not a time that a
        /// busy PC can overrun before the test looks.
        /// </summary>
        private static SimLine SteppedLine(string name, IEnumerable<int> slaves, double stepS) =>
            new SimLine(slaves.Select(s => (SimulatedUnit)new SteppedClockUnit(stepS, SimKit.Sim(timeScale: 1.0) with { Slave = s })).ToList(), name);

        private static Dictionary<string, SimLine> Lines(params SimLine[] lines) => lines.ToDictionary(l => l.Name);

        private static int Switches(IReadOnlyList<int> sequence) => sequence.Zip(sequence.Skip(1), (a, b) => a != b).Count(x => x);

        private static void Near(double expected, double actual, double delta = 0.1) =>
            Assert.True(Math.Abs(expected - actual) <= delta, $"expected {expected} +/- {delta}, got {actual}");

        // ------------------------------------------------------------------ one line, several axes

        [Fact]
        public async Task Two_axes_on_one_line_from_two_threads()
        {
            // stepped clocks: the moves (about 55 mm each way from 73.95) last some 110 requests, however slowly a busy PC
            // runs the threads; both threads start their moves together (a barrier)
            var line = SteppedLine("L", new[] { 1, 2 }, 0.005);
            using var bus = new ModbusRtu(line, minGap: TimeSpan.Zero);
            var axes = new[] { 1, 2 }.ToDictionary(s => s, s => new Axis(bus, s, SimKit.MakeOptions()));
            var targets = new Dictionary<int, double> { [1] = 20.0, [2] = 130.0 };
            using var together = new Barrier(2);
            var start = line.Requests.Count;
            var tasks = axes.ToDictionary(
                kv => kv.Key,
                kv => Task.Factory.StartNew(
                    () =>
                    {
                        together.SignalAndWait(TimeSpan.FromSeconds(10));
                        return kv.Value.MoveTo(targets[kv.Key], 100.0, 0.3);
                    },
                    CancellationToken.None,
                    TaskCreationOptions.LongRunning,
                    TaskScheduler.Default));
            await Task.WhenAll(tasks.Values.ToArray<Task>()).WaitAsync(TimeSpan.FromSeconds(30));
            foreach (var s in targets.Keys)
            {
                Near(targets[s], (await tasks[s])!.PositionMm);
                Near(targets[s], line[s].PositionMm);
            }

            var sequence = line.RequestSlaves().Skip(start).Where(a => a == 1 || a == 2).ToList();
            Assert.True(Switches(sequence) > 10, "frames of the two axes did not interleave");
        }

        [Fact]
        public void A_cell_on_one_line_moves_its_axes_together()
        {
            // stepped clocks (as the Python test): the moves still run when the test looks, however long a busy PC took
            var line = SteppedLine("L", new[] { 1, 2, 3 }, 0.05);
            using var cell = Cell.Open(Lines(line), options: Quick);
            cell.MoveAll(new Dictionary<string, double> { ["L:1"] = 40.0, ["L:2"] = 140.0 }, speed: 100.0, accelG: 0.3, wait: false);
            Assert.True(line[1].Moving && line[2].Moving);
            Assert.False(line[3].Moving);
            var done = cell.WaitAll(new[] { "L:1", "L:2" });
            Near(40.0, done["L:1"].PositionMm);
            Near(140.0, done["L:2"].PositionMm);
            Assert.Equal(new[] { "L:1", "L:2", "L:3" }, cell.StatusAll().Keys);
        }

        // ------------------------------------------------------------------ one unit per port

        [Fact]
        public void Parallel_moves_on_four_ports()
        {
            // a stepped clock per unit (50 ms per request it gets, as the Python test): no axis can arrive before the
            // check that all four move at once, however long a busy PC takes to make it
            var ports = Enumerable.Range(1, 4).Select(i => SteppedLine($"SIM{i}", new[] { 1 }, 0.05)).ToDictionary(l => l.Name);
            using var cell = Cell.Open(ports, options: Quick);
            var targets = cell.Names.Select((n, i) => (n, target: 10.0 + (30 * i))).ToDictionary(x => x.n, x => x.target);
            cell.MoveAll(targets, speed: 100.0, accelG: 0.3, wait: false);
            Assert.All(ports.Values, line => Assert.True(line[1].Moving, "not all axes moving at once"));
            var done = cell.WaitAll();
            foreach (var kv in targets)
            {
                Near(kv.Value, done[kv.Key].PositionMm);
            }
        }

        [Fact]
        public void MoveAll_with_per_axis_speeds_and_each_units_own_limits()
        {
            var ports = SimLine.SimPorts(2, options: SimKit.Sim(timeScale: 20.0));
            using var cell = Cell.Open(ports, options: Quick with { Identify = true });
            var done = cell.MoveAll(new[] { new AxisMove("SIM1:1", 30.0) { Speed = 50.0, AccelG = 0.3 }, new AxisMove("SIM2:1", 60.0) { Speed = 80.0, AccelG = 0.3 } });
            Near(30.0, done!["SIM1:1"].PositionMm);
            Near(60.0, done["SIM2:1"].PositionMm);

            var exc = Assert.Throws<CellException>(() => cell.MoveAll(new Dictionary<string, double> { ["SIM1:1"] = 500.0, ["SIM2:1"] = 20.0 }));
            Assert.IsType<SoftLimitException>(exc.Errors["SIM1:1"]);
            Assert.DoesNotContain("SIM2:1", exc.Errors.Keys);
            Assert.False(ports["SIM2"][1].Moving); // nothing started: every move is checked first
            Near(60.0, ports["SIM2"][1].PositionMm);
        }

        [Fact]
        public void An_alarm_on_one_axis_stops_the_others()
        {
            var ports = SimLine.SimPorts(3, options: SimKit.Sim(timeScale: 2.0));
            using var cell = Cell.Open(ports, options: Quick);
            using var timer = new Timer(_ => ports["SIM2"][1].TriggerAlarm(0x0C0), null, 150, Timeout.Infinite);
            var exc = Assert.Throws<CellException>(() => cell.MoveAll(cell.Names.ToDictionary(n => n, n => 140.0), speed: 60.0, accelG: 0.3));
            Assert.IsType<EcAlarmException>(exc.Errors["SIM2:1"]);
            foreach (var name in new[] { "SIM1", "SIM3" })
            {
                Assert.False(ports[name][1].Moving);
                Assert.True(ports[name][1].PositionMm < 139.0);
            }
        }

        [Fact]
        public void ServoAll_and_HomeAll_on_three_ports()
        {
            var ports = SimLine.SimPorts(3, options: SimKit.Sim(timeScale: 50.0));
            using var cell = Cell.Open(ports, options: Quick);
            cell.ServoAll(false);
            Assert.All(ports.Values, line => Assert.False(line[1].ServoOn));
            cell.ServoAll(true);
            Assert.All(ports.Values, line => Assert.True(line[1].ServoOn));
            var homed = cell.HomeAll();
            foreach (var kv in homed)
            {
                Assert.True(kv.Value!.Homed, kv.Key);
                Near(0.0, kv.Value.PositionMm, 0.05);
            }
        }

        // ------------------------------------------------------------------ STOP first

        [Fact]
        public void StopAll_on_six_axes_sends_every_STOP_first()
        {
            var ports = SimLine.SimPorts(2, new[] { 1, 2, 3 }, SimKit.Sim(timeScale: 1.0));
            using var cell = Cell.Open(ports, options: Quick);
            Assert.Equal(6, cell.Count);
            cell.MoveAll(cell.Names.ToDictionary(n => n, n => 140.0), speed: 50.0, wait: false);
            Thread.Sleep(50);
            var marks = ports.ToDictionary(kv => kv.Key, kv => kv.Value.Requests.Count);
            var started = DateTime.UtcNow;
            var result = cell.StopAll();
            Assert.True(DateTime.UtcNow - started < TimeSpan.FromSeconds(3));
            foreach (var kv in ports)
            {
                var after = kv.Value.Requests.Skip(marks[kv.Key]).Select(r => r.Frame).ToList();
                var first = after.Take(3).ToList();
                Assert.Equal(new[] { 1, 2, 3 }, first.Select(f => (int)f[0]).OrderBy(s => s));
                Assert.All(first, f => Assert.Equal("05 04 2C FF 00", Frames.Pdu(f))); // STOP ON to every axis before anything else
                foreach (var unit in kv.Value)
                {
                    Assert.False(unit.Moving);
                    Assert.False(unit.StopHeld); // released again after the stop
                    Assert.True(unit.PositionMm < 139.0);
                }
            }

            Assert.Equal(6, result.Count);
        }

        [Fact]
        public void StopAll_attempts_every_axis_and_reports_the_failures()
        {
            var ports = SimLine.SimPorts(2, new[] { 1, 2 }, SimKit.Sim(timeScale: 20.0));
            foreach (var line in ports.Values)
            {
                line.ReadTimeout = TimeSpan.FromMilliseconds(30);
            }

            using var cell = Cell.Open(ports, options: Quick with { Retries = 0 });
            ports["SIM1"].Remove(2); // cable pulled: this axis no longer answers
            var results = cell.StopAll(returnExceptions: true);
            Assert.True(results.Errors.ContainsKey("SIM1:2"));
            Assert.Equal(new[] { "SIM1:1", "SIM2:1", "SIM2:2" }, results.Keys);
            var exc = Assert.Throws<CellException>(() => cell.StopAll());
            Assert.Equal(new[] { "SIM1:2" }, exc.Errors.Keys);
        }

        // ------------------------------------------------------------------ broadcast baud change

        private static SimLine BaudLine(double timeScale = 50.0) =>
            SimLine.WithSlaves(new[] { 1, 2, 3 }, SimKit.Sim(timeScale: timeScale) with { BaudRate = 115200 }, "L");

        [Fact]
        public void A_baud_change_on_a_shared_line_switches_every_unit()
        {
            var line = BaudLine();
            using var cell = Cell.Open(Lines(line), options: Quick);
            var port = cell.Ports["L"];
            var result = port.SetBaudRate(230400);
            Assert.Equal(new[] { 1, 2, 3 }, result.Answered.Keys);
            Assert.True(result.AllAnswered);
            Assert.Equal(230400, port.BaudRate);
            Assert.All(line, unit => Assert.Equal(230400, unit.EcBaudRate));
            Assert.Equal(1, line.RequestSlaves().Count(a => a == 0)); // one broadcast frame
            Assert.Equal(3, cell.StatusAll().Count);
        }

        [Fact]
        public void A_rate_without_a_code_is_refused_and_an_unverified_one_sent_with_a_warning()
        {
            var line = BaudLine();
            using var port = new Port(line, "L", minGap: TimeSpan.Zero);
            port.ScanUnits(identify: false, timeout: TimeSpan.FromMilliseconds(10));
            foreach (var s in new[] { 1, 2, 3 })
            {
                port.Axis(s, SimKit.MakeOptions());
            }

            Assert.Throws<SoftLimitException>(() => port.SetBaudRate(12345)); // no IAI code
            Assert.DoesNotContain(0, line.RequestSlaves());
            Assert.All(line, unit => Assert.Equal(115200, unit.EcBaudRate));

            using var capture = new LogCapture();
            port.SetBaudRate(57600); // an UNVERIFIED code: no allowUnverifiedCode needed
            Assert.Equal(1, line.RequestSlaves().Count(a => a == 0)); // one broadcast frame
            Assert.Contains(
                capture.Messages,
                m => m.Contains("baud rate 57600: its code is UNVERIFIED") && m.Contains("Sent anyway"));
        }

        [Fact]
        public void A_baud_change_is_refused_while_an_axis_moves()
        {
            var line = BaudLine(timeScale: 1.0);
            using var cell = Cell.Open(Lines(line), options: Quick);
            cell.MoveAll(new Dictionary<string, double> { ["L:2"] = 140.0 }, speed: 20.0, wait: false);
            Assert.Throws<EcNotReadyException>(() => cell.Ports["L"].SetBaudRate(230400));
            Assert.DoesNotContain(0, line.RequestSlaves());
            cell.StopAll();
        }

        // ------------------------------------------------------------------ 16 axes

        [Fact]
        public void Sixteen_ports_one_axis_each()
        {
            var ports = SimLine.SimPorts(16, options: SimKit.Sim(timeScale: 10.0));
            using var cell = Cell.Open(ports, options: Quick);
            Assert.Equal(16, cell.Count);
            Assert.Equal("SIM1:1", cell.Names[0]);
            Assert.Equal("SIM16:1", cell.Names[15]);
            var targets = cell.Names.Select((n, i) => (n, target: 5.0 + (8 * i))).ToDictionary(x => x.n, x => x.target);
            var done = cell.MoveAll(targets, speed: 100.0, accelG: 0.3);
            foreach (var kv in targets)
            {
                Near(kv.Value, done![kv.Key].PositionMm);
            }

            Assert.Equal(16, cell.StopAll().Count);
        }

        [Fact]
        public void Sixteen_axes_on_one_line()
        {
            var line = Line("RCON", Enumerable.Range(1, 16), timeScale: 10.0);
            using var cell = Cell.Open(Lines(line), options: Quick);
            Assert.Equal(Enumerable.Range(1, 16).Select(s => "RCON:" + s), cell.Names);
            var targets = Enumerable.Range(0, 16).ToDictionary(i => cell.Names[i], i => 10.0 + (7 * i));
            var done = cell.MoveAll(targets, speed: 100.0, accelG: 0.3);
            for (var i = 0; i < 16; i++)
            {
                Near(targets[cell.Names[i]], done![cell.Names[i]].PositionMm);
                Near(targets[cell.Names[i]], line[i + 1].PositionMm);
            }
        }

        [Fact]
        public void A_cell_takes_every_axis_unless_MaxAxes_caps_it()
        {
            using (var all = Cell.Open(Lines(Line("A", Enumerable.Range(1, 9)), Line("B", Enumerable.Range(1, 9))), Enumerable.Range(1, 9), Quick))
            {
                Assert.Equal(18, all.Count);  // no limit by default
            }

            var capped = Quick with { MaxAxes = 16 };
            var listed = Lines(Line("A", Enumerable.Range(1, 9)), Line("B", Enumerable.Range(1, 9)));
            Assert.Throws<ArgumentException>(() => Cell.Open(listed, Enumerable.Range(1, 9), capped));
            Assert.All(listed.Values, line => Assert.True(line.Closed));

            var scanned = Lines(Line("A", Enumerable.Range(1, 8)), Line("B", Enumerable.Range(1, 8)), Line("C", new[] { 1 }));
            using var cell = Cell.Open(scanned, options: capped);
            Assert.Equal(16, cell.Count);
            Assert.Equal(16, cell.StatusAll().Count);
            Assert.DoesNotContain("C:1", cell.Names);
            Assert.True(scanned["C"].Closed); // the port that lost all its axes is closed
        }
    }
}
