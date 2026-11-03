using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary><see cref="SimLine"/>: several simulated units on one line, and the port layer (scan, baud detection, cell plumbing) on top of it.</summary>
    [Collection(SimCollection.Name)]
    public class SimLineTests
    {
        private static readonly CellOptions Quick = new CellOptions
        {
            ScanTimeout = TimeSpan.FromMilliseconds(10),
            AxisOptions = SimKit.MakeOptions(),
            Identify = false,
        };

        private static SimLine Line(string name, params int[] slaves) => SimLine.WithSlaves(slaves, SimKit.Sim(), name);

        private static Dictionary<string, SimLine> Lines(params SimLine[] lines) => lines.ToDictionary(l => l.Name);

        private static int Asked(SimLine line, int slave) => line.RequestSlaves().Count(a => a == slave);

        // ------------------------------------------------------------------ the line itself

        [Fact]
        public void Only_the_addressed_unit_answers_and_an_unknown_address_is_silent()
        {
            var line = new SimLine(new[]
            {
                new SimulatedUnit(SimKit.Sim() with { Slave = 1 }),
                new SimulatedUnit(SimKit.Sim() with { Slave = 3, PositionMm = 20.0 }),
            });
            using var bus = new ModbusRtu(line, minGap: TimeSpan.Zero);
            var one = bus.ReadRegisters(1, Registers.MONITOR_BASE, 2);
            var three = bus.ReadRegisters(3, Registers.MONITOR_BASE, 2);
            Assert.Equal(73.95, SimKit.Pnow(one));
            Assert.NotEqual(SimKit.Pnow(one), SimKit.Pnow(three)); // each unit answers for itself
            Assert.False(bus.Probe(2, TimeSpan.FromMilliseconds(5)));
            Assert.True(bus.Probe(3, TimeSpan.FromMilliseconds(5)));
            Assert.Equal(new[] { 1, 3 }, line.Slaves);
            Assert.Equal(2, line.Count);
        }

        [Fact]
        public void A_duplicate_or_out_of_range_address_is_refused()
        {
            var line = Line("L", 1);
            Assert.Throws<ArgumentException>(() => line.Add(new SimulatedUnit(new SimulatedUnitOptions { Slave = 1 })));
            Assert.ThrowsAny<ArgumentException>(() => line.Add(new SimulatedUnit(new SimulatedUnitOptions { Slave = 248 })));
            Assert.Throws<ArgumentException>(() => SimLine.WithSlaves(new[] { 2, 2 }));
        }

        [Fact]
        public void A_broadcast_reaches_every_unit_and_nobody_answers()
        {
            var line = SimLine.WithSlaves(new[] { 1, 2, 3 }, SimKit.Sim());
            using var bus = new ModbusRtu(line, minGap: TimeSpan.Zero);
            bus.BroadcastIaiCommand(0, 0x09, 0, settle: TimeSpan.Zero); // servo off to all (a safe-list command)
            Assert.Equal(new[] { false, false, false }, line.Select(u => u.ServoOn));
            Assert.Equal(0, line.Read(new byte[10], 0, 10));
            Assert.Equal(new[] { 0 }, line.RequestSlaves());
        }

        [Fact]
        public void A_unit_taken_off_the_line_stops_answering()
        {
            var line = Line("L", 1, 2);
            using var bus = new ModbusRtu(line, minGap: TimeSpan.Zero);
            Assert.True(bus.Probe(2, TimeSpan.FromMilliseconds(5)));
            var unit = line.Remove(2);
            Assert.Equal(2, unit.Slave);
            Assert.False(bus.Probe(2, TimeSpan.FromMilliseconds(5)));
            Assert.Throws<KeyNotFoundException>(() => line.Remove(2));
        }

        [Fact]
        public void Ports_are_named_and_hold_their_slaves()
        {
            var ports = SimLine.SimPorts(3, new[] { 1, 2 });
            Assert.Equal(new[] { "SIM1", "SIM2", "SIM3" }, ports.Keys);
            Assert.Equal(new[] { 1, 2 }, ports["SIM2"].Slaves);
            Assert.Equal("SIM2", ports["SIM2"].Name);
            Assert.Equal(new[] { "A1", "A2" }, SimLine.SimPorts(2, prefix: "A").Keys);
            Assert.Throws<ArgumentOutOfRangeException>(() => SimLine.SimPorts(0));
            Assert.Contains("SIM1", ports["SIM1"].ToString());
        }

        [Fact]
        public void Close_closes_every_unit_and_refuses_new_frames()
        {
            var line = Line("L", 1, 2);
            line.Close();
            Assert.True(line.Closed);
            Assert.All(line, u => Assert.True(u.Closed));
            Assert.Throws<IOException>(() => line.Write(Crc16.BuildFrame(1, new byte[] { 0x03, 0x90, 0x00, 0x00, 0x01 }), 0, 8));
        }

        [Fact]
        public void The_line_has_a_baud_rate_only_when_its_units_model_one()
        {
            var plain = Line("L", 1);
            Assert.Equal(0, plain.BaudRate);
            Assert.Throws<NotSupportedException>(() => plain.BaudRate = 38400);
            var modelled = SimLine.WithSlaves(new[] { 1, 2 }, SimKit.Sim() with { BaudRate = 115200 });
            Assert.Equal(115200, modelled.BaudRate);
            modelled.BaudRate = 38400;
            Assert.All(modelled, u => Assert.Equal(38400, u.BaudRate));
        }

        [Fact]
        public void A_break_reaches_every_unit_on_the_wire()
        {
            var line = SimLine.WithSlaves(new[] { 1, 2 }, SimKit.Sim() with { BaudRate = 115200 });
            line.SendBreak(TimeSpan.FromMilliseconds(200));
            Assert.All(line, u => Assert.Equal(9600, u.EcBaudRate)); // a long BREAK drops a unit to 9600 baud (DOC)
        }

        // ------------------------------------------------------------------ scan

        [Fact]
        public void A_scan_of_16_addresses_asks_each_missing_address_once()
        {
            var line = SimLine.WithSlaves(new[] { 1, 5, 12 }, SimKit.Sim(timeScale: 20.0), "L");
            using var port = new Port(line, "L");
            var started = DateTime.UtcNow;
            var hits = port.ScanUnits();
            var elapsed = DateTime.UtcNow - started;
            Assert.Equal(new[] { 1, 5, 12 }, hits.Select(h => h.Slave));
            Assert.True(elapsed < TimeSpan.FromSeconds(2.5), $"scan took {elapsed.TotalSeconds:F2} s");
            Assert.True(elapsed > TimeSpan.FromSeconds(13 * ModbusRtu.ScanTimeout.TotalSeconds * 0.8), "each missing address should wait for its probe timeout");
            foreach (var s in Enumerable.Range(1, 16).Except(new[] { 1, 5, 12 }))
            {
                Assert.Equal(1, Asked(line, s)); // no retry for a missing address
            }

            Assert.Equal(new[] { "L:1", "L:5", "L:12" }, hits.Select(h => h.Name));
            Assert.Equal(4, hits[1].AxisNumber);
            Assert.Equal(0xEC, hits[0].SystemCode);
            Assert.Equal("EC dust/drip-proof", hits[0].Family);
            Assert.Null(hits[0].Error);
        }

        [Fact]
        public void A_scan_is_read_only_and_closes_the_transport_it_was_given()
        {
            var line = Line("L", 1, 2);
            var hits = PortManager.Scan(line, "L", new[] { 1, 2, 3, 4 }, TimeSpan.FromMilliseconds(10));
            Assert.Equal(new[] { 1, 2 }, hits.Select(h => h.Slave));
            Assert.All(line.Requests, r => Assert.Contains(r.Frame[1], new byte[] { 0x03, 0x11 }));
            Assert.True(line.Closed);
        }

        [Fact]
        public void A_scan_identifies_a_late_FC11_answer()
        {
            var line = SimLine.WithSlaves(new[] { 3 }, SimKit.Sim(SimProfile.Rod), "L"); // the rod answers FC11 after 0.3 s
            using var port = new Port(line, "L", minGap: TimeSpan.Zero);
            var hits = port.ScanUnits(new[] { 1, 2, 3 }, TimeSpan.FromMilliseconds(10));
            Assert.Equal(new[] { 3 }, hits.Select(h => h.Slave));
            Assert.Equal("0002", hits[0].Firmware);
            Assert.Equal("ec ff 00 00 02", hits[0].ToDictionary()["slave_id_raw"]);
        }

        [Fact]
        public void A_scan_detects_the_baud_rate_once_per_port()
        {
            var options = SimKit.Sim() with { BaudRate = 115200, EcBaudRate = 38400 }; // the units are at their power-up rate
            var line = SimLine.WithSlaves(new[] { 2, 3 }, options, "L");
            using var port = new Port(line, "L", minGap: TimeSpan.Zero);
            var hits = port.ScanUnits(new[] { 1, 2, 3, 4 }, TimeSpan.FromMilliseconds(10), identify: false);
            Assert.Equal(new[] { 2, 3 }, hits.Select(h => h.Slave));
            Assert.True(port.BaudDetected);
            Assert.Equal(38400, port.BaudRate);
            Assert.Equal(38400, line.BaudRate);
            var sent = line.Requests.Count;
            port.EnsureBaudRate();
            Assert.Equal(sent, line.Requests.Count);
        }

        [Fact]
        public void Detecting_the_rate_asks_several_slaves_and_fails_when_nothing_answers()
        {
            var line = SimLine.WithSlaves(new[] { 4 }, SimKit.Sim() with { BaudRate = 115200, EcBaudRate = 38400 });
            using var bus = new ModbusRtu(line, minGap: TimeSpan.Zero);
            Assert.Equal(38400, bus.DetectBaudRate(new[] { 1, 2, 3, 4 }, probeTimeout: TimeSpan.FromMilliseconds(10)));
            Assert.Equal(38400, line.BaudRate);

            var silent = SimLine.WithSlaves(new[] { 9 }, SimKit.Sim() with { BaudRate = 115200 });
            using var other = new ModbusRtu(silent, minGap: TimeSpan.Zero);
            var exc = Assert.Throws<ModbusTimeoutException>(() => other.DetectBaudRate(new[] { 1, 2 }, new[] { 115200, 38400 }, probeTimeout: TimeSpan.FromMilliseconds(10)));
            Assert.Contains("none of slaves 1, 2", exc.Message);
            Assert.Equal(115200, silent.BaudRate);
        }

        [Fact]
        public void An_empty_line_gives_an_empty_scan_and_a_bad_address_is_refused()
        {
            var empty = new SimLine(name: "E");
            Assert.Empty(PortManager.Scan(empty, "E", new[] { 1, 2, 3 }, TimeSpan.FromMilliseconds(5)));
            Assert.Throws<ArgumentOutOfRangeException>(() => PortManager.Scan(Line("L", 1), "L", new[] { 0, 1 }));
        }

        // ------------------------------------------------------------------ cell plumbing

        [Fact]
        public void Names_lookup_and_one_bus_per_port()
        {
            var ports = SimLine.SimPorts(2, new[] { 1, 3 });
            using (var cell = Cell.Open(ports, options: Quick))
            {
                Assert.Equal(new[] { "SIM1:1", "SIM1:3", "SIM2:1", "SIM2:3" }, cell.Names);
                Assert.Same(cell["SIM2:3"], cell[3]);
                Assert.Equal("SIM1:3", cell.NameOf(cell[1]));
                Assert.True(cell.Contains("SIM1:1"));
                Assert.False(cell.Contains("SIM9:1"));
                Assert.Equal("SIM2", cell.PortOf("SIM2:1").Name);
                Assert.Same(cell[0].Bus, cell[1].Bus); // one bus per port, shared by its axes
                Assert.NotSame(cell[0].Bus, cell[2].Bus);
                Assert.Throws<KeyNotFoundException>(() => cell["COM3:1"]);
            }

            Assert.All(ports.Values, line => Assert.True(line.Closed));
        }

        [Fact]
        public void Open_reads_each_units_own_ranges()
        {
            var line = new SimLine(
                new[]
                {
                    new SimulatedUnit(SimKit.Sim(SimProfile.Slider) with { Slave = 1 }),
                    new SimulatedUnit(SimKit.Sim(SimProfile.Gripper) with { Slave = 2 }),
                },
                "L");
            using var cell = Cell.Open(Lines(line), options: Quick with { Identify = true, AxisOptions = null });
            Assert.Equal((-0.3, 150.3), cell.Info["L:1"].SoftLimitsMm!.Value);
            Assert.Equal((-0.3, 9.8), cell.Info["L:2"].SoftLimitsMm!.Value);
            Assert.Equal((-0.3, 9.8), cell["L:2"].SoftLimitsMm!.Value);
        }

        [Fact]
        public void Listed_slaves_must_answer_and_skip_the_scan()
        {
            var line = Line("L", 1);
            var exc = Assert.Throws<ModbusTimeoutException>(() => Cell.Open(Lines(line), new[] { 1, 2 }, Quick));
            Assert.Contains("slave(s) 2", exc.Message);
            Assert.True(line.Closed);

            var three = Line("M", 1, 2, 3);
            using var cell = Cell.Open(Lines(three), new[] { 2 }, Quick);
            Assert.Equal(new[] { "M:2" }, cell.Names);
            Assert.Equal(0, Asked(three, 9));
            Assert.Equal(0, Asked(three, 1));
        }

        [Fact]
        public void A_scan_skips_dead_ports_and_nothing_found_raises()
        {
            var a = Line("A", 1);
            var b = new SimLine(name: "B");
            using (var cell = Cell.Open(Lines(a, b), options: Quick with { ScanSlaves = new[] { 1, 2, 3 } }))
            {
                Assert.Equal(new[] { "A:1" }, cell.Names);
                Assert.Contains("B", cell.PortErrors.Keys);
            }

            Assert.True(b.Closed);
            var empty = new SimLine(name: "E");
            Assert.Throws<EcNotReadyException>(() => Cell.Open(Lines(empty), options: Quick with { ScanSlaves = new[] { 1, 2 } }));
            Assert.True(empty.Closed);
        }

        [Fact]
        public void MaxAxes_refuses_listed_slaves_and_trims_a_scan()
        {
            Assert.Throws<ArgumentException>(() => Cell.Open(Lines(Line("L", 1, 2, 3)), new[] { 1, 2, 3 }, Quick with { MaxAxes = 2 }));
            using var cell = Cell.Open(Lines(Line("M", 1, 2, 3)), options: Quick with { MaxAxes = 2 });
            Assert.Equal(new[] { "M:1", "M:2" }, cell.Names);
        }
    }
}
