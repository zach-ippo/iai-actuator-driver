using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>The directory (index table) reader on the rod's real index table.</summary>
    public class DirectoryTests
    {
        private static (ModbusRtu Bus, FakeUnit Unit, IRegisterReader Reader) Rod()
        {
            var unit = RodFixture.Unit();
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            return (bus, unit, bus.ForSlave(1));
        }

        [Fact]
        public void ParsesTheRodIndexTable()
        {
            var dir = UnitDirectory.FromWords(T.Words(RodFixture.IndexTable));
            Assert.Equal(new IndexHeader(1, 8, 22, 0x100, 0x100), dir.Header);
            Assert.Equal(22, dir.Records.Count);
            var monitor = dir.Get(Registers.CATEGORY_MONITOR);
            Assert.Equal(0x9000, monitor.DataAddress);
            Assert.Equal(new BlockExtent(34, true, 34), monitor.Extent);
            Assert.Equal("monitor", monitor.Name);
            var control = dir.Get(Registers.CATEGORY_IO_CONTROL);
            Assert.Equal((0x0D00, 0x0D80, 0x0400, 1), (control.DataAddress, control.IdListAddress, control.CoilBase, control.MemberSize));
            Assert.Equal(new BlockExtent(12288, true, 12288), dir.Get(Registers.CATEGORY_POSITION_TABLE).Extent);
            Assert.Equal(new BlockExtent(7, false, 14), dir.Get(0x0012).Extent); // 0x7F00: mixed sizes, unknown
            Assert.Null(dir.Find(0x0114));
            Assert.Throws<KeyNotFoundException>(() => dir.Get(0x0114));
            foreach (var pair in Registers.EXPECTED_CATEGORY_BASES)
            {
                Assert.Equal(pair.Value, dir.Get(pair.Key).DataAddress);
            }
        }

        [Fact]
        public void ReadOverModbus()
        {
            var (_, unit, reader) = Rod();
            var dir = UnitDirectory.Read(reader);
            Assert.Equal(22, dir.Records.Count);
            Assert.Equal(new[] { (0x0000, 8), (0x0008, 60), (0x0044, 60), (0x0080, 56) }, unit.Reads);
        }

        [Fact]
        public void UnexpectedLayoutIsRejected()
        {
            var words = T.Words(RodFixture.IndexTable);
            words[1] = 7;
            Assert.Throws<InvalidDataException>(() => UnitDirectory.FromWords(words));
            var unit = new FakeUnit();
            unit.Put(0, words);
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            Assert.Throws<InvalidDataException>(() => UnitDirectory.Read(bus.ForSlave(1)));
            Assert.Single(unit.Reads); // only the header was read
        }

        [Fact]
        public void ReadableWordsNeverReachTheNextBlock()
        {
            var a = new IndexCategory(0x0777, 0, 10, 1, 0xA000, 0, 0, 0); // unknown mixed block: 10 certain, 20 at most
            var b = new IndexCategory(0x0778, 0, 4, 1, 0xA00C, 0, 0, 1); // the next block starts 12 words later
            Assert.Equal(20, UnitDirectory.ReadableWords(a, null));
            Assert.Equal(12, UnitDirectory.ReadableWords(a, new[] { a, b }));
            Assert.Equal(4, UnitDirectory.ReadableWords(b, new[] { a, b }));
        }

        [Fact]
        public void ReadBlockStaysInsideTheBlock()
        {
            var (_, unit, reader) = Rod();
            var dir = UnitDirectory.FromWords(T.Words(RodFixture.IndexTable));
            Assert.Equal(new ushort[] { 0x0000, 0x3AB6, 0xFFFF, 0xFFE2 }, dir.ReadBlock(reader, 7, 0, 4));
            Assert.Equal(52, dir.ReadBlock(reader, 7).Length); // default: the certain extent
            Assert.Equal(14, dir.ReadBlock(reader, 0x0012, 0, 14).Length); // uncertain tail up to words max
            var before = unit.Reads.Count;
            foreach (var (classId, offset, count) in new[] { (7, 51, 2), (7, -1, 1), (7, 0, 0), (0x0103, 9, 2), (0x0012, 13, 2) })
            {
                Assert.Throws<ArgumentOutOfRangeException>(() => dir.ReadBlock(reader, classId, offset, count));
            }

            Assert.Equal(before, unit.Reads.Count); // refused before anything was sent
        }

        [Fact]
        public void ReadClassLabelsItemsById()
        {
            var (_, unit, reader) = Rod();
            var dir = UnitDirectory.FromWords(T.Words(RodFixture.IndexTable));
            var c7 = dir.ReadClass(reader, Registers.CATEGORY_LIMIT_PARAMS);
            Assert.Equal((0x0700, 52, 2, 0x5100), (c7.Address, c7.Words, c7.MemberSize, c7.IdMapAddress!.Value));
            Assert.Equal(26, c7.Items.Count);
            Assert.Equal(15030u, c7.Item(Registers.AXIS_ITEM_LIMM)!.Raw); // +150.30 mm
            Assert.Equal(-30, c7.Item(Registers.AXIS_ITEM_LIML)!.Signed); // -0.30 mm
            Assert.Equal(-0.3, c7.Item(Registers.AXIS_ITEM_LIML)!.Scaled);
            Assert.Equal(800.0, c7.Item(Registers.AXIS_ITEM_MAXV)!.Scaled); // MAXV 800.00 mm/s (VERIFIED on EC #3)
            Assert.Equal(3u, c7.Item(Registers.AXIS_ITEM_AKID)!.Raw); // rod
            Assert.Equal("7.33", c7.Item(Registers.AXIS_ITEM_AKID)!.Key);
            Assert.Equal("FFFFFFE2", c7.Item(Registers.AXIS_ITEM_LIML)!.Hex);
            Assert.Null(c7.Item(58)); // no MNPP on the rod
            Assert.Contains((0x5100, 52), unit.Reads);

            var monitor = dir.ReadClass(reader, Registers.CATEGORY_MONITOR); // mixed sizes, no ID map: word by word
            Assert.Equal(34, monitor.Items.Count);
            Assert.All(monitor.Items, i => Assert.Null(i.ItemId));

            unit.Put(0x5600, "0001000000020000"); // zone class ID map: items 1 and 2
            var zones = dir.ReadClass(reader, Registers.CATEGORY_ZONES); // 2 items x 2 entries
            Assert.Equal(new[] { 0, 0, 1, 1 }, zones.Items.Select(i => i.Entry));
            Assert.Equal(new int?[] { 1, 2, 1, 2 }, zones.Items.Select(i => i.ItemId));
            Assert.Equal("C.2[1]", zones.Items[3].Key);
            Assert.Throws<ArgumentOutOfRangeException>(() => dir.ReadClass(reader, Registers.CATEGORY_POSITION_TABLE));
            Assert.Throws<KeyNotFoundException>(() => dir.ReadClass(reader, 0x0114));
        }

        [Fact]
        public void SupportWordsAndAccl()
        {
            var (_, unit, reader) = Rod();
            unit.Put(0x0100, "00002BD7 0001 0002 0003 0004 4384009A"); // SUPT DIAG MVPT SPTS CTYP SUP2 (6 members)
            var dir = UnitDirectory.FromWords(T.Words(RodFixture.IndexTable));
            var support = dir.ReadSupportWords(reader)!;
            Assert.Equal(new[] { "SUPT", "DIAG", "MVPT", "SPTS", "CTYP", "SUP2" }, support.Keys);
            Assert.Equal(0x00002BD7u, support["SUPT"]);
            Assert.Equal(0x4384009Au, support["SUP2"]);
            Assert.True(Alarms.AbsoluteEncoderFromSupport(support));
            Assert.Equal(0u, dir.ReadAccl(reader)); // the rod's ACCL is 0 (extensibility level 0)
        }
    }

    /// <summary>Identity parsing and the identity reader.</summary>
    public class IdentityTests
    {
        [Fact]
        public void SlaveIdAndCtclParsing()
        {
            var sid = SlaveIdentity.FromSlaveId(T.Hex("EC FF 02 00 13"))!;
            Assert.Equal((0xEC, (int?)0xFF, 0x02, "0013", "FC11"), (sid.SystemCode, sid.RunStatus, sid.UnitCode, sid.Firmware, sid.Source));
            Assert.Null(SlaveIdentity.FromSlaveId(T.Hex("EC FF 02")));
            var ctcl = SlaveIdentity.FromCtcl(0xEC0A0002)!;
            Assert.Equal((0xEC, 0x0A, "0002", "CTCL 0x0300"), (ctcl.SystemCode, ctcl.UnitCode, ctcl.Firmware, ctcl.Source));
            Assert.Null(SlaveIdentity.FromCtcl(0x00000002));
        }

        [Fact]
        public void IdentificationBlockIsPrintableAscii()
        {
            var block = IdentificationBlock.FromWords(T.Words(RodFixture.Identification));
            Assert.Equal("EC-R6S-150-1-PN", block.ModelName);
            Assert.Equal("SIM-0000003", block.SerialNumber);
            Assert.Null(block.UserName); // blank on EC #3
            Assert.Equal(0u, block.Accl);
            Assert.Equal(0, block.ExtensibilityLevel);

            var binary = T.Words(RodFixture.Identification);
            binary[0x20] = 0x00B8; // a CON controller keeps binary data where an EC keeps its model string
            Assert.Null(IdentificationBlock.FromWords(binary).ModelName);
            Assert.Null(IdentificationBlock.FromWords(binary.Take(8).ToArray()).ModelName); // short block
        }

        [Fact]
        public void ReadsTheRod()
        {
            var unit = RodFixture.Unit();
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            var info = DeviceIdentity.Read(bus, 1);
            Assert.Equal((0xEC, 0x00, "0002", "FC11"), (info.SystemCode!.Value, info.UnitCode!.Value, info.Firmware, info.IdentitySource));
            Assert.Equal("EC", info.ProductFamily);
            Assert.Equal("EC 24 V", info.SystemName);
            Assert.Equal("EC standard", info.UnitName);
            Assert.True(info.UnitKnown);
            Assert.Equal("EC-R6S-150-1-PN", info.Model);
            Assert.Equal("SIM-0000003", info.SerialNumber);
            Assert.Equal(0, info.AxisNumber);
            Assert.Equal(22, info.Categories.Count);
            Assert.Equal(0x9000, info.Category(Registers.CATEGORY_MONITOR)!.DataAddress);
            Assert.Equal("EC 24 V, unit 0x00 (EC standard)", info.Controller);
            Assert.Empty(info.Errors);
            Assert.Null(info.ControllerName);
            Assert.Null(info.SoftLimitsMm); // axis class
        }

        [Fact]
        public void FallsBackToCtclWhenFc11IsSilent()
        {
            var unit = RodFixture.Unit();
            unit.SlaveId = null;
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            var info = DeviceIdentity.Read(bus, 1);
            Assert.Equal("CTCL 0x0300", info.IdentitySource);
            Assert.Equal((0xEC, 0x00, "0002"), (info.SystemCode!.Value, info.UnitCode!.Value, info.Firmware));
            Assert.Contains(info.Errors, e => e.Contains("slave id"));
            Assert.Contains(info.Errors, e => e.Contains("CTCL"));
            Assert.Equal("EC-R6S-150-1-PN", info.Model);
        }

        [Fact]
        public void ReportsWhatItCannotReadAndNeverThrowsOnIt()
        {
            var silent = new ModbusRtu(new FakeTransport(), retries: 0, minGap: TimeSpan.Zero);
            var info = DeviceIdentity.Read(silent, 1);
            Assert.Null(info.SystemCode);
            Assert.Equal("unknown", info.Controller);
            Assert.Equal("IAI controller", info.ProductFamily);
            Assert.Contains(info.Errors, e => e.StartsWith("report slave id: not answered"));
            Assert.Contains(info.Errors, e => e.StartsWith("index table:"));
            Assert.All(info.Errors, e => Assert.True(e.All(c => c >= 0x20 && c <= 0x7E)));
        }

        [Fact]
        public void UnknownUnitAndAxisNumberMismatchAreReported()
        {
            var unit = RodFixture.Unit(2);
            unit.SlaveId = T.Hex("AE FF 45 00 00"); // a PCON controller, a unit code without a family name
            unit.Put(0x8004, new ushort[] { 0, 0 }); // AXNO 0, but slave 2 means axis 1
            unit.Put(0x8010, T.Words("50434F4E20202020 43422020 28464229")); // "PCON    " "CB  " "(FB)"
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            var info = DeviceIdentity.Read(bus, 2);
            Assert.Equal("PCON", info.ProductFamily);
            Assert.False(info.UnitKnown); // no family name: display only, never a warning
            Assert.Equal((0xAE, 0x45), (info.SystemCode!.Value, info.UnitCode!.Value)); // reported as the unit says
            Assert.Equal("PCON-CB (FB)", info.ControllerName);
            Assert.DoesNotContain(info.Errors,
                e => e.Contains("UNVERIFIED") || e.Contains("tried on hardware") || e.Contains("PCON unit 0x45"));
            Assert.Contains(info.Errors, e => e.Contains("AXNO (0x8004) = 0, but slave address 2 means axis 1"));
        }
    }

    /// <summary>The port: scan, identify, open checks.</summary>
    public class PortTests
    {
        [Fact]
        public void ScanFindsAnsweringSlavesOnly()
        {
            var transport = new FakeTransport(FakeUnit.Line(RodFixture.Unit(1), RodFixture.Unit(3), new FakeUnit(16)));
            using var port = new Port(transport, "fake", minGap: TimeSpan.Zero);
            var started = DateTime.UtcNow;
            var found = port.Scan(timeout: TimeSpan.FromMilliseconds(10));
            Assert.Equal(new[] { 1, 3, 16 }, found);
            Assert.Equal(16, transport.Writes.Count); // one request per address, no retries
            Assert.True(DateTime.UtcNow - started < TimeSpan.FromSeconds(5));
            Assert.Equal(Enumerable.Range(1, 16).Select(a => (byte)a), transport.Writes.Select(w => w[0]));
        }

        [Fact]
        public void ScanCountsAnExceptionReplyAsAnAnswer()
        {
            var transport = new FakeTransport(request => request[0] == 5 ? T.Response(5, "83 02") : null);
            using var port = new Port(transport, minGap: TimeSpan.Zero);
            Assert.Equal(new[] { 5 }, port.Scan(new[] { 4, 5, 6 }, TimeSpan.FromMilliseconds(5)));
        }

        [Fact]
        public void ScanIgnoresAForeignOrCorruptReply()
        {
            var transport = new FakeTransport(request =>
            {
                if (request[0] == 2)
                {
                    return T.ReadResponse(7, 0); // another address answers
                }

                if (request[0] == 3)
                {
                    var bad = T.ReadResponse(3, 0);
                    bad[bad.Length - 1] ^= 0xFF;
                    return bad;
                }

                return null;
            });
            using var port = new Port(transport, minGap: TimeSpan.Zero);
            Assert.Empty(port.Scan(new[] { 2, 3 }, TimeSpan.FromMilliseconds(5)));
        }

        [Fact]
        public void IdentifiesEveryAxisOnOneLine()
        {
            var transport = new FakeTransport(FakeUnit.Line(RodFixture.Unit(1), RodFixture.Unit(2)));
            using var port = new Port(transport, "line", minGap: TimeSpan.Zero);
            var infos = port.Scan(new[] { 1, 2, 3 }, TimeSpan.FromMilliseconds(5)).Select(port.Identify).ToList();
            Assert.Equal(new[] { 0, 1 }, infos.Select(i => i.AxisNumber!.Value));
            Assert.All(infos, i => Assert.Empty(i.Errors));
            Assert.Equal(22, port.ReadDirectory(2).Records.Count);
            Assert.Equal("Port(line)", port.ToString());
        }

        [Fact]
        public void DetectBaudRateAddsTheChecklist()
        {
            using var port = new Port(new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 1 }, minGap: TimeSpan.Zero);
            var exc = Assert.Throws<ModbusTimeoutException>(() => port.DetectBaudRate(new[] { 1 }, new[] { 38400 }));
            Assert.Contains("24 V supply", exc.Message);
            Assert.False(port.BaudDetected);
        }

        [Fact]
        public void DisposeClosesTheTransport()
        {
            var transport = new FakeTransport();
            var port = new Port(transport);
            port.Dispose();
            Assert.True(transport.Closed);
            Assert.True(port.Bus.IsClosed);
        }
    }
}
