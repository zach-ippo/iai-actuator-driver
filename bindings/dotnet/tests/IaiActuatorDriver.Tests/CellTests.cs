using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// Several test units (<see cref="SimUnit"/>) on one <see cref="FakeTransport"/>, the way an RCON gateway or an SIO daisy
    /// chain puts several axes on one serial line: every frame reaches every unit, only the addressed one answers, a
    /// broadcast reaches all and none answers.
    /// </summary>
    /// <remarks>
    /// The fast, deterministic variant (no real-time simulation, exact frame checks). The same behaviour against the simulator's
    /// <see cref="IaiActuatorDriver.Simulation.SimLine"/> is tested in <c>SimLineTests</c> and <c>SimMultiAxisTests</c>.
    /// </remarks>
    internal sealed class UnitLine
    {
        public UnitLine(string name, IEnumerable<SimUnit> units, int baudRate = 0)
        {
            Name = name;
            Units = units.ToList();
            Transport = new FakeTransport(Respond, baudRate);
            if (baudRate > 0)
            {
                Transport.AnswerAtBaudRate = baudRate;
            }

            foreach (var unit in Units)
            {
                unit.Transport = Transport;
            }
        }

        public string Name { get; }

        public List<SimUnit> Units { get; }

        public FakeTransport Transport { get; }

        public SimUnit this[int slave] => Units.Single(u => u.Address == slave);

        /// <summary>A line of rods at <paramref name="slaves"/>, each ready at 0 mm.</summary>
        public static UnitLine Rods(string name, params int[] slaves) => new UnitLine(name, slaves.Select(s => new SimUnit("rod", s).Ready(0.0)));

        public int Requests(int slave, byte function) => Transport.Writes.Count(f => f.Length > 1 && f[0] == slave && f[1] == function);

        public List<(int Slave, int Address, bool On)> Coils() =>
            Transport.Writes.Where(f => f.Length > 5 && f[1] == 0x05).Select(f => ((int)f[0], (f[2] << 8) | f[3], f[4] == 0xFF)).ToList();

        private byte[]? Respond(byte[] request)
        {
            byte[]? answer = null;
            foreach (var unit in Units)
            {
                var reply = unit.Respond(request);
                answer ??= reply;
            }

            return answer;
        }
    }

    public class PortManagerTests
    {
        private static readonly SerialPortInfo[] FakePorts =
        {
            new SerialPortInfo("COM1", "Communications Port (COM1)"),
            new SerialPortInfo("COM10", "IAI USB to UART Bridge Controller (COM10)", 0x10C4, 0x81D7, "IAI Corporation"),
            new SerialPortInfo("COM4", "USB Serial Port (COM4)", 0x0403, 0x6001, "FTDI"),
            new SerialPortInfo("COM3", "IAI USB to UART Bridge Controller (COM3)", 0x10C4, 0x81D7, "IAI Corporation"),
            new SerialPortInfo("COM7", "Some adapter", 0x1234, 0x5678, "IAI Corporation"),
        };

        [Fact]
        public void IAI_adapters_come_first_in_natural_order()
        {
            var entries = PortManager.Sort(FakePorts);
            Assert.Equal(new[] { "COM3", "COM7", "COM10", "COM1", "COM4" }, entries.Select(e => e.Port));
            Assert.Equal(new[] { true, true, true, false, false }, entries.Select(e => e.IsIai));
            var com3 = entries[0].ToDictionary();
            Assert.Equal(new[] { "port", "description", "vid", "pid", "is_iai" }, com3.Keys);
            Assert.Equal(0x10C4, com3["vid"]);
            Assert.Equal(0x81D7, com3["pid"]);
        }

        [Fact]
        public void A_port_without_usb_ids_is_still_listed_and_can_be_IAI_by_its_name()
        {
            Assert.False(new SerialPortInfo("COM5").IsIai);
            Assert.True(new SerialPortInfo("COM5", "IAI USB to UART Bridge Controller (COM5)").IsIai);
            Assert.False(new SerialPortInfo("COM5", "IAIX thing").IsIai);
        }

        [Fact]
        public void ListPorts_opens_nothing_and_keeps_the_IAI_first_order()
        {
            var entries = PortManager.ListPorts(); // READ-ONLY: port names and the device registry only
            var flags = entries.Select(e => e.IsIai).ToList();
            Assert.Equal(flags.OrderByDescending(f => f), flags);
            Assert.Equal(entries.Where(e => e.IsIai).Select(e => e.Port), PortManager.IaiPortNames());
        }

        [Fact]
        public void Helpers()
        {
            Assert.Equal("COM3:1", PortManager.AxisName("COM3", 1));
            Assert.Equal("1-3,5,7-8", PortManager.Ranges(new[] { 1, 2, 3, 5, 7, 8 }));
            Assert.Equal("-", PortManager.Ranges(Array.Empty<int>()));
            Assert.Equal(Enumerable.Range(1, 16), PortManager.DefaultSlaves);
        }

        [Fact]
        public void ScanUnits_finds_the_answering_addresses_and_reads_FC11()
        {
            var line = UnitLine.Rods("L", 1, 5, 12);
            using var port = new Port(line.Transport, "L", minGap: TimeSpan.Zero);
            var hits = port.ScanUnits(timeout: TimeSpan.FromMilliseconds(5));
            Assert.Equal(new[] { 1, 5, 12 }, hits.Select(h => h.Slave));
            Assert.Equal(new[] { "L:1", "L:5", "L:12" }, hits.Select(h => h.Name));
            Assert.Equal(4, hits[1].AxisNumber);
            Assert.Equal(0xEC, hits[0].SystemCode);
            Assert.Equal(0x00, hits[0].UnitCode);
            Assert.Equal("0002", hits[0].Firmware);
            Assert.Equal("EC", hits[0].ProductFamily);
            Assert.Null(hits[0].Error);
            Assert.Equal("ec ff 00 00 02", hits[0].ToDictionary()["slave_id_raw"]);
            foreach (var s in Enumerable.Range(1, 16).Except(new[] { 1, 5, 12 }))
            {
                Assert.Equal(1, line.Requests(s, 0x03)); // each missing address asked once, no retry
            }
        }

        [Fact]
        public void A_scan_is_read_only_and_closes_a_transport_it_was_given()
        {
            var line = UnitLine.Rods("L", 1, 2);
            var hits = PortManager.Scan(line.Transport, "L", new[] { 1, 2, 3, 4 }, TimeSpan.FromMilliseconds(5));
            Assert.Equal(new[] { 1, 2 }, hits.Select(h => h.Slave));
            Assert.All(line.Transport.Writes, f => Assert.Contains(f[1], new byte[] { 0x03, 0x11 }));
            Assert.True(line.Transport.Closed);
        }

        [Fact]
        public void The_baud_rate_is_detected_once_per_port()
        {
            var line = new UnitLine("L", new[] { new SimUnit("rod", 2).Ready(), new SimUnit("rod", 3).Ready() }, baudRate: 115200);
            line.Transport.AnswerAtBaudRate = 38400; // the units are at their power-up rate
            using var port = new Port(line.Transport, "L", minGap: TimeSpan.Zero);
            var hits = port.ScanUnits(new[] { 1, 2, 3, 4 }, TimeSpan.FromMilliseconds(5), identify: false);
            Assert.Equal(new[] { 2, 3 }, hits.Select(h => h.Slave));
            Assert.True(port.BaudDetected);
            Assert.Equal(38400, port.BaudRate);
            var sent = line.Transport.Writes.Count;
            port.EnsureBaudRate();
            Assert.Equal(sent, line.Transport.Writes.Count);
        }

        [Fact]
        public void A_fixed_rate_is_never_probed_and_nothing_answering_gives_an_empty_scan()
        {
            var transport = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 1 };
            using var port = new Port(transport, "F", minGap: TimeSpan.Zero, autoBaud: false);
            Assert.Empty(port.ScanUnits(new[] { 1, 2 }, TimeSpan.FromMilliseconds(5)));
            Assert.Empty(transport.BaudRates);
            Assert.Throws<ArgumentOutOfRangeException>(() => port.ScanUnits(new[] { 0, 1 }));
        }

        [Fact]
        public void Port_Axis_is_created_once_per_address()
        {
            var line = UnitLine.Rods("L", 1, 2);
            using var port = new Port(line.Transport, "L", minGap: TimeSpan.Zero);
            var a = port.Axis(2, SimUnit.Fast);
            Assert.Same(a, port.Axis(2));
            Assert.NotSame(a, port.Axis(1));
            Assert.Equal(new[] { 1, 2 }, port.Axes.Keys);
            Assert.Throws<ArgumentOutOfRangeException>(() => port.Axis(0));
        }

        [Fact]
        public void A_port_baud_change_is_one_broadcast_for_every_unit_on_the_line()
        {
            var line = new UnitLine("L", new[] { new SimUnit("rod", 1).Ready(), new SimUnit("rod", 2).Ready() }, baudRate: 38400);
            using var port = new Port(line.Transport, "L", minGap: TimeSpan.Zero, autoBaud: false);
            port.Axis(1, SimUnit.Fast);
            port.Axis(2, SimUnit.Fast);
            line[2].Stp = true; // paused: not at rest
            Assert.Throws<EcNotReadyException>(() => port.SetBaudRate(115200, settleS: 0.01));
            Assert.DoesNotContain(line.Transport.Writes, f => f[1] == 0x45);
            line[2].Stp = false;
            var result = port.SetBaudRate(115200, settleS: 0.01);
            Assert.True(result.AllAnswered);
            Assert.Equal(new[] { 1, 2 }, result.Answered.Keys);
            Assert.Equal(115200, port.BaudRate);
            Assert.Single(line.Transport.Writes, f => f[0] == 0 && f[1] == 0x45);
            Assert.Contains("\"answered\"", System.Text.Json.JsonSerializer.Serialize(result.ToDictionary()));
        }
    }

    public class CellTests
    {
        private static readonly CellOptions Quick = new CellOptions
        {
            ScanTimeout = TimeSpan.FromMilliseconds(5),
            AxisOptions = SimUnit.Fast,
            Identify = false,
        };

        private static Dictionary<string, ITransport> Lines(params UnitLine[] lines) => lines.ToDictionary(l => l.Name, l => (ITransport)l.Transport);

        [Fact]
        public void Names_lookup_and_one_bus_per_port()
        {
            var a = UnitLine.Rods("SIM1", 1, 3);
            var b = UnitLine.Rods("SIM2", 1, 3);
            using (var cell = Cell.Open(Lines(a, b), options: Quick))
            {
                Assert.Equal(new[] { "SIM1:1", "SIM1:3", "SIM2:1", "SIM2:3" }, cell.Names);
                Assert.Same(cell["SIM2:3"], cell[3]);
                Assert.Equal("SIM1:3", cell.NameOf(cell[1]));
                Assert.True(cell.Contains("SIM1:1"));
                Assert.False(cell.Contains("SIM9:1"));
                Assert.Equal("SIM2", cell.PortOf("SIM2:1").Name);
                Assert.Same(cell[0].Bus, cell[1].Bus);
                Assert.NotSame(cell[0].Bus, cell[2].Bus);
                Assert.Equal("mm", cell.UnitsOf("SIM1:1"));
                Assert.Throws<KeyNotFoundException>(() => cell["COM3:1"]);
                Assert.Equal(4, cell.Count);
                Assert.Equal(cell.Axes, cell.ToList());
            }

            Assert.True(a.Transport.Closed);
            Assert.True(b.Transport.Closed);
        }

        [Fact]
        public void Open_reads_each_units_own_identity()
        {
            var line = new UnitLine("L", new[] { new SimUnit("rod", 1).Ready(), new SimUnit("pcon", 2).Ready(60.0) });
            using var cell = Cell.Open(Lines(line), options: Quick with { Identify = true });
            Assert.Equal("EC-R6S-150-1-PN", cell.Info["L:1"].Model);
            Assert.Equal("EC", cell.Info["L:1"].ProductFamily);
            Assert.Equal("PCON", cell.Info["L:2"].ProductFamily);
            Assert.Equal((-0.30, 150.30), cell["L:1"].SoftLimitsMm!.Value);
            var limits = cell.LimitsAll();
            Assert.Equal(new[] { "L:1", "L:2" }, limits.Keys);
            Assert.Equal(800.0, limits["L:1"].Speed.UnitMax);
        }

        [Fact]
        public void Default_ports_are_the_IAI_adapters_and_no_other_port_is_opened()
        {
            var com3 = UnitLine.Rods("COM3", 1);
            var com10 = UnitLine.Rods("COM10", 1);
            var opened = new List<string>();
            var options = Quick with
            {
                PortLister = () => PortManager.Sort(new[]
                {
                    new SerialPortInfo("COM10", "IAI USB to UART Bridge Controller (COM10)", 0x10C4, 0x81D7),
                    new SerialPortInfo("COM1", "Communications Port (COM1)"),
                    new SerialPortInfo("COM3", "IAI USB to UART Bridge Controller (COM3)", 0x10C4, 0x81D7),
                }),
                SerialOpener = (name, baud, timeout) =>
                {
                    opened.Add(name);
                    return name == "COM3" ? com3.Transport : name == "COM10" ? com10.Transport : throw new IOException("not an IAI port");
                },
            };
            using var cell = Cell.Open(options);
            Assert.Equal(new[] { "COM3:1", "COM10:1" }, cell.Names);
            Assert.DoesNotContain("COM1", opened);
        }

        [Fact]
        public void Without_any_IAI_adapter_nothing_is_opened()
        {
            var opened = 0;
            var options = Quick with
            {
                PortLister = () => new[] { new SerialPortInfo("COM1", "Communications Port (COM1)") },
                SerialOpener = (name, baud, timeout) =>
                {
                    opened++;
                    throw new IOException("must not open");
                },
            };
            var exc = Assert.Throws<EcNotReadyException>(() => Cell.Open(options));
            Assert.Contains("no IAI USB adapter", exc.Message);
            Assert.Equal(0, opened);
        }

        [Fact]
        public void Listed_slaves_must_answer_and_every_port_is_closed_again()
        {
            var line = UnitLine.Rods("L", 1);
            var other = UnitLine.Rods("M", 1, 2);
            var exc = Assert.Throws<ModbusTimeoutException>(() => Cell.Open(Lines(line, other), new[] { 1, 2 }, Quick));
            Assert.Contains("slave(s) 2", exc.Message);
            Assert.True(line.Transport.Closed);
            Assert.True(other.Transport.Closed);
        }

        [Fact]
        public void Listed_slaves_skip_the_scan()
        {
            var line = UnitLine.Rods("L", 1, 2, 3);
            using var cell = Cell.Open(Lines(line), new[] { 2 }, Quick);
            Assert.Equal(new[] { "L:2" }, cell.Names);
            Assert.Equal(0, line.Requests(1, 0x03));
            Assert.Equal(0, line.Requests(3, 0x03));
        }

        [Fact]
        public void Ports_named_with_their_slaves()
        {
            var a = UnitLine.Rods("COM5", 1);
            var b = UnitLine.Rods("COM6", 2, 3);
            var lines = new Dictionary<string, UnitLine> { ["COM5"] = a, ["COM6"] = b };
            var options = Quick with { SerialOpener = (name, baud, timeout) => lines[name].Transport };
            using var cell = Cell.Open(new Dictionary<string, int[]> { ["COM5"] = new[] { 1 }, ["COM6"] = new[] { 3 } }, options);
            Assert.Equal(new[] { "COM5:1", "COM6:3" }, cell.Names);
        }

        [Fact]
        public void MaxAxes_refuses_listed_slaves_and_trims_a_scan()
        {
            var listed = UnitLine.Rods("L", 1, 2, 3);
            Assert.Throws<ArgumentException>(() => Cell.Open(Lines(listed), new[] { 1, 2, 3 }, Quick with { MaxAxes = 2 }));
            Assert.True(listed.Transport.Closed);
            var scanned = UnitLine.Rods("M", 1, 2, 3);
            using var cell = Cell.Open(Lines(scanned), options: Quick with { MaxAxes = 2 });
            Assert.Equal(new[] { "M:1", "M:2" }, cell.Names);
        }

        [Fact]
        public void A_scan_skips_dead_ports()
        {
            var a = UnitLine.Rods("A", 1);
            var b = new UnitLine("B", Array.Empty<SimUnit>());
            using (var cell = Cell.Open(Lines(a, b), options: Quick with { ScanSlaves = new[] { 1, 2, 3 } }))
            {
                Assert.Equal(new[] { "A:1" }, cell.Names);
                Assert.Contains("B", cell.PortErrors.Keys);
                Assert.Contains("no axis answered at addresses 1-3", cell.PortErrors["B"].Message);
            }

            Assert.True(b.Transport.Closed);
        }

        [Fact]
        public void Nothing_found_raises_and_closes()
        {
            var empty = new UnitLine("E", Array.Empty<SimUnit>());
            Assert.Throws<EcNotReadyException>(() => Cell.Open(Lines(empty), options: Quick with { ScanSlaves = new[] { 1, 2 } }));
            Assert.True(empty.Transport.Closed);
        }

        [Fact]
        public void A_cell_from_open_ports_and_their_axes()
        {
            var line = UnitLine.Rods("L", 1, 2);
            var port = new Port(line.Transport, "L", minGap: TimeSpan.Zero);
            port.Axis(2, SimUnit.Fast);
            port.Axis(1, SimUnit.Fast);
            using var cell = new Cell(new[] { port });
            Assert.Equal(new[] { "L:1", "L:2" }, cell.Names);
            Assert.Equal(2, cell.StatusAll().Count);
        }

        [Fact]
        public void StatusAll_collects_per_axis_results_and_errors()
        {
            var a = UnitLine.Rods("A", 1, 2);
            var b = UnitLine.Rods("B", 1);
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            a[2].Position = 12.5;
            var all = cell.StatusAll();
            Assert.True(all.Succeeded);
            Assert.Equal(12.5, all["A:2"].PositionMm);
            Assert.Equal(new[] { "A:1", "A:2", "B:1" }, all.Names);

            a.Units.Remove(a[2]); // cable pulled
            var exc = Assert.Throws<CellException>(() => cell.StatusAll());
            Assert.Equal("StatusAll", exc.Operation);
            Assert.IsType<ModbusTimeoutException>(exc.Errors["A:2"]);
            Assert.Equal(new[] { "A:1", "B:1" }, exc.Results.Keys);

            var partial = cell.StatusAll(returnExceptions: true);
            Assert.False(partial.Succeeded);
            Assert.Equal(new[] { "A:1", "B:1" }, partial.Keys);
            var missing = Assert.Throws<KeyNotFoundException>(() => partial["A:2"]);
            Assert.IsType<ModbusTimeoutException>(missing.InnerException);
            Assert.Single(cell.StatusAll(new[] { "B:1" }));
        }

        [Fact]
        public void A_port_name_longer_than_the_results_field_keeps_its_results_and_errors()
        {
            // (0.8) the core's per-axis results carry the axis name cut at 127 bytes; the index (the position in the
            // request) is the key, so neither an axis's status nor its failure is lost
            var longName = new string('p', 150);
            var first = longName + ":1";
            var second = longName + ":2";
            var a = UnitLine.Rods(longName, 1, 2);
            var b = UnitLine.Rods("B", 1);
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            Assert.Equal(new[] { first, second, "B:1" }, cell.Names);
            a[2].Position = 12.5;
            var all = cell.StatusAll();
            Assert.True(all.Succeeded);
            Assert.Equal(3, all.Count);
            Assert.Equal(12.5, all[second].PositionMm);

            a.Units.Remove(a[2]); // cable pulled
            var exc = Assert.Throws<CellException>(() => cell.StatusAll());
            Assert.IsType<ModbusTimeoutException>(exc.Errors[second]);
            Assert.Equal(new[] { first, "B:1" }, exc.Results.Keys);
            var partial = cell.StatusAll(returnExceptions: true);
            Assert.Equal(new[] { first, "B:1" }, partial.Keys);
            Assert.IsType<ModbusTimeoutException>(partial.Errors[second]);

            // the group call under the managed layer, a list in another order
            var core = cell.Core!.StatusAll(new[] { "B:1", second, first }, returnExceptions: true);
            Assert.Equal(new[] { "B:1", first }, core.Keys.OrderBy(k => k).ToArray());
            Assert.IsType<ModbusTimeoutException>(core.Errors[second]);
            Assert.Single(cell.StatusAll(new[] { second }, returnExceptions: true).Errors);
        }

        [Fact]
        public void MoveAll_checks_every_axis_before_anything_moves()
        {
            var a = UnitLine.Rods("A", 1);
            var b = UnitLine.Rods("B", 1);
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            var exc = Assert.Throws<CellException>(() => cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 50.0, ["B:1"] = 500.0 }));
            Assert.StartsWith("MoveAll (check; nothing started)", exc.Operation);
            Assert.IsType<SoftLimitException>(exc.Errors["B:1"]);
            Assert.DoesNotContain(a.Transport.Writes, f => f[1] == 0x10);
            Assert.DoesNotContain(b.Transport.Writes, f => f[1] == 0x10);
            Assert.Throws<KeyNotFoundException>(() => cell.MoveAll(new Dictionary<string, double> { ["C:1"] = 1.0 }));
        }

        [Fact]
        public void MoveAll_moves_every_axis_and_waits_for_all()
        {
            var a = UnitLine.Rods("A", 1, 2); // two axes sharing one line
            var b = UnitLine.Rods("B", 1);
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            var done = cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 50.0, ["A:2"] = 75.0, ["B:1"] = 30.0 }, speed: 40);
            Assert.Equal(50.0, done!["A:1"].PositionMm);
            Assert.Equal(75.0, done["A:2"].PositionMm);
            Assert.Equal(30.0, done["B:1"].PositionMm);
            var moves = a.Transport.Writes.Where(f => f[1] == 0x10 && f[2] == 0x99 && f[3] == 0x00).Select(f => (int)f[0]).ToList();
            Assert.Equal(new[] { 1, 2 }, moves);
        }

        [Fact]
        public void MoveAll_with_per_axis_values_then_WaitAll()
        {
            var a = UnitLine.Rods("A", 1);
            var b = UnitLine.Rods("B", 1);
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            var started = cell.MoveAll(new[] { new AxisMove("A:1", 20.0) { Speed = 10 }, new AxisMove("B:1", 40.0) { Band = 0.5 } }, wait: false);
            Assert.Null(started);
            var all = cell.WaitAll();
            Assert.Equal(20.0, all["A:1"].PositionMm);
            Assert.Equal(40.0, all["B:1"].PositionMm);
            Assert.Throws<ArgumentException>(() => cell.MoveAll(new[] { new AxisMove("A:1", 1.0), new AxisMove("A:1", 2.0) }));
        }

        [Fact]
        public void A_failure_on_one_axis_stops_the_others()
        {
            var a = UnitLine.Rods("A", 1);
            var b = UnitLine.Rods("B", 1);
            a[1].StepMm = 5;
            a[1].AlarmDuringMove = 0x0C1;
            a[1].AlarmAtMm = 20;
            b[1].StepMm = 0.5;
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            var exc = Assert.Throws<CellException>(() => cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 100.0, ["B:1"] = 140.0 }, timeout: 20));
            Assert.Equal("WaitAll", exc.Operation);
            Assert.IsType<EcAlarmException>(exc.Errors["A:1"]);
            Assert.Contains("B:1", exc.Errors.Keys); // stopped short of its target
            Assert.Contains((1, Registers.COIL_STOP, true), b.Coils());
            Assert.Contains((1, Registers.COIL_STOP, false), b.Coils());
            Assert.True(b[1].Position < 140.0);
        }

        [Fact]
        public void StopAll_sends_STOP_to_every_axis_first_then_holds_and_releases_each()
        {
            var a = UnitLine.Rods("A", 1, 2);
            foreach (var unit in a.Units)
            {
                unit.StepMm = 0.5;
            }

            using var cell = Cell.Open(Lines(a), options: Quick);
            cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 100.0, ["A:2"] = 100.0 }, wait: false);
            var mark = a.Transport.Writes.Count;
            var stopped = cell.StopAll();
            var after = a.Transport.Writes.Skip(mark).ToList();
            Assert.Equal("05 04 2C FF 00", Frames.Pdu(after[0]));
            Assert.Equal("05 04 2C FF 00", Frames.Pdu(after[1]));
            Assert.Equal(new[] { 1, 2 }, new[] { (int)after[0][0], (int)after[1][0] });
            Assert.Contains((1, Registers.COIL_STOP, false), a.Coils());
            Assert.Contains((2, Registers.COIL_STOP, false), a.Coils());
            Assert.False(a[1].StopCoil);
            Assert.False(a[2].StopCoil);
            Assert.Equal(2, stopped.Count);
            Assert.True(a[1].Position < 100.0);
        }

        [Fact]
        public void StopAll_with_an_unknown_name_still_stops_the_axes_it_has()
        {
            // a remembered list naming an axis that dropped out of a reopened cell: the STOP still goes out
            var a = UnitLine.Rods("A", 1, 2);
            foreach (var unit in a.Units)
            {
                unit.StepMm = 0.5;
            }

            using var cell = Cell.Open(Lines(a), options: Quick);
            cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 100.0, ["A:2"] = 100.0 }, wait: false);
            var mark = a.Transport.Writes.Count;
            var error = Assert.Throws<CellException>(() => cell.StopAll(new[] { "A:1", "NOSUCH:9" }));
            var after = a.Transport.Writes.Skip(mark).ToList();
            Assert.Equal("05 04 2C FF 00", Frames.Pdu(after[0]));
            Assert.Equal(1, (int)after[0][0]);
            Assert.Contains((1, Registers.COIL_STOP, false), a.Coils());
            Assert.Equal(new[] { "NOSUCH:9" }, error.Errors.Keys.ToArray());
            Assert.IsType<KeyNotFoundException>(error.Errors["NOSUCH:9"]);
            Assert.True(error.Results.ContainsKey("A:1"));
            var kept = cell.StopAll(new[] { "NOSUCH:9", "A:2" }, returnExceptions: true);
            Assert.Equal(new[] { "A:2", "NOSUCH:9" }, kept.Names.ToArray());
            Assert.True(kept.ContainsKey("A:2"));
            Assert.Contains((2, Registers.COIL_STOP, false), a.Coils());
            Assert.True(a[2].Position < 100.0);
        }

        [Fact]
        public void ServoAll_and_HomeAll()
        {
            var a = UnitLine.Rods("A", 1, 2);
            foreach (var unit in a.Units)
            {
                unit.Servo = false;
                unit.Homed = false;
                unit.Position = 40.0;
            }

            using var cell = Cell.Open(Lines(a), options: Quick);
            var on = cell.ServoAll(true);
            Assert.True(on.Succeeded);
            Assert.All(a.Units, u => Assert.True(u.Servo));
            var homed = cell.HomeAll();
            Assert.All(homed.Values, st => Assert.True(st!.Homed));
            Assert.All(a.Units, u => Assert.Equal(0.0, u.Position));
            cell.ServoAll(false);
            Assert.All(a.Units, u => Assert.False(u.Servo));
        }

        [Fact]
        public void Cancelling_a_wait_stops_every_waiting_axis()
        {
            var a = UnitLine.Rods("A", 1);
            var b = UnitLine.Rods("B", 1);
            a[1].StepMm = 0.1;
            b[1].StepMm = 0.1;
            using var cell = Cell.Open(Lines(a, b), options: Quick);
            cell.MoveAll(new Dictionary<string, double> { ["A:1"] = 100.0, ["B:1"] = 100.0 }, wait: false);
            using var cts = new CancellationTokenSource(TimeSpan.FromMilliseconds(100));
            Assert.ThrowsAny<OperationCanceledException>(() => cell.WaitAll(timeout: 20, cancellationToken: cts.Token));
            Assert.Contains((1, Registers.COIL_STOP, true), a.Coils());
            Assert.Contains((1, Registers.COIL_STOP, true), b.Coils());
            Assert.False(a[1].StopCoil);
            Assert.False(b[1].StopCoil);
        }

        [Fact]
        public void A_closed_cell_refuses_commands_and_Close_is_safe_twice()
        {
            var a = UnitLine.Rods("A", 1);
            var cell = Cell.Open(Lines(a), options: Quick);
            cell.Close();
            cell.Dispose();
            Assert.True(cell.IsClosed);
            Assert.True(a.Transport.Closed);
            Assert.Throws<EcNotReadyException>(() => cell.StatusAll());
            Assert.Throws<EcNotReadyException>(() => cell.StopAll());
        }

        [Fact]
        public void Duplicate_ports_and_bad_options_are_refused()
        {
            Assert.Throws<ArgumentException>(() => Cell.Open(new[] { "COM3", "COM3" }, options: Quick with { SerialOpener = (n, b, t) => throw new IOException("never") }));
            Assert.Throws<ArgumentOutOfRangeException>(() => Cell.Open(new[] { "COM3" }, options: Quick with { MaxAxes = -1 }));
            Assert.Throws<ArgumentOutOfRangeException>(() => Cell.Open(new[] { "COM3" }, new[] { 0 }, Quick));
        }

        [Fact]
        public void No_axis_limit_by_default_and_MaxAxes_still_caps()
        {
            Assert.Equal(0, new CellOptions().MaxAxes);  // no limit
            var lines = Enumerable.Range(1, 17).ToDictionary(i => "COM" + i, i => UnitLine.Rods("COM" + i, 1));
            var options = Quick with { SerialOpener = (name, baud, timeout) => lines[name].Transport };
            using (var cell = Cell.Open(lines.Keys, new[] { 1 }, options))  // 17 ports, one axis each
            {
                Assert.Equal(17, cell.Names.Count);
            }

            var nine = Enumerable.Range(1, 9).ToArray();
            using (var cell = Cell.Open(Lines(UnitLine.Rods("A", nine), UnitLine.Rods("B", nine)), options: Quick with { ScanSlaves = nine }))
            {
                Assert.Equal(18, cell.Names.Count);  // 2 lines x 9
            }

            using (var cell = Cell.Open(Lines(UnitLine.Rods("C", nine), UnitLine.Rods("D", nine)), options: Quick with { ScanSlaves = nine, MaxAxes = 17 }))
            {
                Assert.Equal(17, cell.Names.Count);
            }
        }

        [Fact]
        public void MoveTimeout_follows_the_trapezoid()
        {
            // 100 mm at 50 mm/s and 0.1 G: 2 x (2 s + 0.051 s) + 2 s
            Assert.Equal(6.1, Cell.MoveTimeout(100.0, 50.0, 0.1), 2);
            Assert.True(Cell.MoveTimeout(0.0, 50.0, 0.1) >= 2.0);
        }
    }
}
