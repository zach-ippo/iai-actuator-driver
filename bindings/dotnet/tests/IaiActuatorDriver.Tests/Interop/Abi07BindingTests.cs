using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The round-5 fixes of the C# binding: a cell over ports that have axes already runs on those axes, a cell's group
    /// command keeps what a managed transport threw, a probe's wait caps a transport timeout raised after the open, an
    /// axis that cannot be built releases the cell's core axis, and the line option. No port opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class Abi07BindingTests
    {
        [NativeFact]
        public void A_cell_over_a_port_with_an_axis_runs_its_group_commands_on_that_axis()
        {
            var sim = new SimulatedUnit(SimKit.Sim()); // a slider at 73.95 mm
            var port = new Port(sim, "P", minGap: TimeSpan.Zero);
            var axis = port.Axis(1, new AxisOptions { Stroke = (0.0, 50.0), MaxSpeedMmS = 20.0 });
            using var cell = Cell.Open(new[] { port }, new[] { 1 }, new CellOptions { Identify = false });
            Assert.Same(axis, cell["P:1"]);
            var sent = sim.Requests.Count;

            // the axis's own stroke and speed cap: refused before anything is sent, as axis.MoveTo refuses them
            var outside = Assert.Throws<CellException>(() => cell.MoveAll(new Dictionary<string, double> { ["P:1"] = 80.0 }, speed: 10.0));
            Assert.IsType<SoftLimitException>(outside.Errors["P:1"]);
            var fast = Assert.Throws<CellException>(() => cell.MoveAll(new Dictionary<string, double> { ["P:1"] = 30.0 }, speed: 100.0));
            Assert.IsType<SoftLimitException>(fast.Errors["P:1"]);
            Assert.DoesNotContain(sim.Requests.Skip(sent), f => f.Length > 3 && f[1] == 0x10 && f[2] == 0x99); // no move went out
            Assert.Equal(73.95, sim.PositionMm, 2);

            Assert.InRange(cell.MoveAll(new Dictionary<string, double> { ["P:1"] = 45.0 }, speed: 20.0)!["P:1"].PositionMm, 44.9, 45.1);
            Assert.InRange(axis.Status().PositionMm, 44.9, 45.1);
        }

        [NativeFact]
        public void A_cell_group_command_keeps_what_a_managed_transport_threw()
        {
            var transport = new FailingTransport(new SimulatedUnit(SimKit.Sim()));
            using var cell = Cell.Open(new Dictionary<string, ITransport> { ["P"] = transport }, new[] { 1 }, new CellOptions { Identify = false });
            Assert.True(cell.StatusAll().Succeeded);

            var io = new IOException("cable pulled");
            transport.Failure = io;
            var status = Assert.IsType<ModbusException>(cell.StatusAll(returnExceptions: true).Errors["P:1"]);
            Assert.Same(io, status.InnerException); // as axis.Status() raises it
            Assert.StartsWith("transport error: cable pulled", status.Message);
            var limits = Assert.IsType<ModbusException>(cell.LimitsAll(returnExceptions: true).Errors["P:1"]); // the core's group read
            Assert.Same(io, limits.InnerException);

            var odd = new NotImplementedException("no frames today");
            transport.Failure = odd;
            Assert.Same(odd, cell.StatusAll(returnExceptions: true).Errors["P:1"]); // anything else: the transport's own exception
            Assert.Same(odd, Assert.Throws<CellException>(() => cell.StatusAll()).Errors["P:1"]);

            transport.Failure = null;
            Assert.True(cell.StatusAll().Succeeded);
        }

        [NativeFact]
        public void A_probe_caps_a_transport_timeout_raised_after_the_open()
        {
            var transport = new FailingTransport(new SimulatedUnit(SimKit.Sim())) { ReadTimeout = TimeSpan.FromMilliseconds(50), BlockWhenSilent = true };
            using var bus = new ModbusRtu(transport, minGap: TimeSpan.Zero);
            transport.ReadTimeout = TimeSpan.FromSeconds(1.0); // raised after the open: the core held 50 ms
            var clock = Stopwatch.StartNew();
            Assert.False(bus.Probe(5, TimeSpan.FromMilliseconds(200))); // nobody at 5
            Assert.True(clock.Elapsed < TimeSpan.FromSeconds(0.7), $"the probe waited {clock.Elapsed.TotalSeconds:0.00} s");
            Assert.Equal(TimeSpan.FromSeconds(1.0), transport.ReadTimeout); // put back after the probe
            Assert.True(bus.Probe(1, TimeSpan.FromMilliseconds(200)));
        }

        [NativeFact]
        public void An_axis_that_cannot_be_built_releases_the_cells_core_axis()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            using var port = new Port(sim, "P", minGap: TimeSpan.Zero);
            var core = NativeAxis.OpenOnBus(port.Bus.Native, 1);
            Assert.Throws<ArgumentOutOfRangeException>(() => port.AxisOf(1, new AxisOptions { MaxSpeedMmS = -1.0 }, core));
            Assert.True(core.IsClosed);
            Assert.Empty(port.Axes);
        }

        [NativeFact]
        public void A_port_finds_its_line_once_or_takes_the_one_given()
        {
            var auto = new SimulatedUnit(SimKit.Sim());
            using (var port = new Port(auto, "A", minGap: TimeSpan.Zero))
            {
                Assert.Equal(LineKind.Auto, port.Line);
                Assert.Equal(new[] { 1 }, port.ScanUnits(identify: false).Select(h => h.Slave));
                Assert.Contains(auto.Requests, f => f[0] == 0xD0 || f[0] == 0x3F); // the gateway probes, read-only
                Assert.DoesNotContain(auto.Requests, f => (f[0] == 0xD0 || f[0] == 0x3F) && f[1] != 0x03);
            }

            var direct = new SimulatedUnit(SimKit.Sim());
            using (var port = new Port(direct, "D", minGap: TimeSpan.Zero, line: LineKind.Direct))
            {
                Assert.Equal(new[] { 1 }, port.ScanUnits(identify: false).Select(h => h.Slave));
                Assert.DoesNotContain(direct.Requests, f => f[0] == 0xD0 || f[0] == 0x3F);
            }

            Assert.Throws<ArgumentOutOfRangeException>(() => new Port(new SimulatedUnit(SimKit.Sim()), "X", line: (LineKind)42));
            using var cell = Cell.Open(SimLine.SimPorts(1), options: new CellOptions { Identify = false, Line = LineKind.Direct });
            Assert.Equal(LineKind.Direct, cell.Ports["SIM1"].Line);
        }

        /// <summary>
        /// A managed transport over a simulated unit that throws <see cref="Failure"/> on a write and, with
        /// <see cref="BlockWhenSilent"/>, waits its whole read timeout when no answer is there (as a serial port does).
        /// </summary>
        private sealed class FailingTransport : ITransport
        {
            private readonly SimulatedUnit _unit;

            public FailingTransport(SimulatedUnit unit) => _unit = unit;

            public Exception? Failure { get; set; }

            public bool BlockWhenSilent { get; set; }

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

            public int Read(byte[] buffer, int offset, int count)
            {
                var n = _unit.Read(buffer, offset, count);
                if (n == 0 && BlockWhenSilent)
                {
                    Thread.Sleep(ReadTimeout);
                }

                return n;
            }

            public void ResetInputBuffer() => _unit.ResetInputBuffer();

            public void SendBreak(TimeSpan duration) => _unit.SendBreak(duration);

            public void Close() => _unit.Close();

            public void Dispose() => Close();
        }
    }
}
