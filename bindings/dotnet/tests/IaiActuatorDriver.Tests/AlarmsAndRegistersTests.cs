using System;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    public class AlarmTests
    {
        [Fact]
        public void Names()
        {
            Assert.Equal("ABS error (2)", Alarms.AlarmName(0x0EE)); // curated
            Assert.Equal("Real Time Clock oscillation stop detection", Alarms.AlarmName(0x069)); // IAI's list
            Assert.Equal("PowerUP (no error)", Alarms.AlarmName(0xFFF));
            Assert.Equal("alarm 0x123", Alarms.AlarmName(0x123));
            Assert.Equal(269, Alarms.CONTROLLER_ALARM_NAMES.Count);
        }

        [Fact]
        public void MessageCodesAreTheNamedCodesOfTheMessageRange()
        {
            var named = Alarms.CONTROLLER_ALARM_NAMES.Keys.Where(c => c >= 0x011 && c <= 0x07F).OrderBy(c => c);
            Assert.Equal(named, Alarms.MESSAGE_CODES.OrderBy(c => c));
        }

        [Theory]
        [InlineData(0x000, AlarmLevel.None)]
        [InlineData(0xFFF, AlarmLevel.None)]
        [InlineData(0x005, AlarmLevel.ColdStart)]
        [InlineData(0x04E, AlarmLevel.Message)]
        [InlineData(0x069, AlarmLevel.Message)]
        [InlineData(0x030, AlarmLevel.Unknown)]
        [InlineData(0x090, AlarmLevel.OperationCancelled)]
        [InlineData(0x0C1, AlarmLevel.OperationCancelled)]
        [InlineData(0x0D9, AlarmLevel.OperationCancelled)]
        [InlineData(0x0E0, AlarmLevel.ColdStart)]
        [InlineData(0x0CE, AlarmLevel.ColdStart)]
        [InlineData(0x0D2, AlarmLevel.ColdStart)]
        [InlineData(0x842, AlarmLevel.Unknown)]
        public void Levels(int code, AlarmLevel level) => Assert.Equal(level, Alarms.AlarmLevelOf(code));

        [Fact]
        public void Level0EEDependsOnTheEncoder()
        {
            Assert.Equal(AlarmLevel.OperationCancelled, Alarms.AlarmLevelOf(0x0EE));
            Assert.Equal(AlarmLevel.OperationCancelled, Alarms.AlarmLevelOf(0x0EE, absoluteEncoder: true));
            Assert.Equal(AlarmLevel.ColdStart, Alarms.AlarmLevelOf(0x0EE, absoluteEncoder: false));
            Assert.Equal("operation_cancelled", Alarms.LevelKey(AlarmLevel.OperationCancelled));
        }

        [Fact]
        public void AbsoluteEncoderFromSupportWords()
        {
            Assert.Null(Alarms.AbsoluteEncoderFromSupport(null));
            Assert.True(Alarms.AbsoluteEncoderFromSupport(new Dictionary<string, uint> { ["SUPT"] = 0x60002BFF, ["SUP2"] = 0x838C009A }));
            Assert.True(Alarms.AbsoluteEncoderFromSupport(new Dictionary<string, uint> { ["SUP2"] = 1u << 18 }));
            Assert.False(Alarms.AbsoluteEncoderFromSupport(new Dictionary<string, uint> { ["SUPT"] = 0x00000003 }));
            Assert.Null(Alarms.AbsoluteEncoderFromSupport(new Dictionary<string, uint> { ["SUP2"] = 0 }));
        }

        [Fact]
        public void FaultRule()
        {
            Assert.False(Alarms.AlarmIsFault(0));
            Assert.True(Alarms.AlarmIsFault(0, alarmBit: true));
            Assert.False(Alarms.AlarmIsFault(0x04E));
            Assert.True(Alarms.AlarmIsFault(0x0E0));
            Assert.True(Alarms.AlarmIsFault(0x030));
            Assert.False(Alarms.AlarmIsFault(0x030, warningBit: true));
            Assert.True(Alarms.AlarmIsFault(0x0C1, warningBit: true));
        }

        [Fact]
        public void Groups()
        {
            Assert.Equal("D", Alarms.AlarmGroup(0x0EE));
            Assert.Equal("Warning", Alarms.AlarmGroup(0x04E));
            Assert.Equal("Other", Alarms.AlarmGroup(0x090));
            Assert.Equal("controller-encoder link error", Alarms.GROUP_NAMES["D"]);
        }

        [Fact]
        public void HistorySlotsFromTheRod()
        {
            var words = T.Words(RodFixture.AlarmHistory);
            var slots = Alarms.AlarmHistorySlots(words);
            Assert.Equal(4, slots.Count);
            Assert.True(slots[0].PowerUp);
            Assert.Null(slots[0].TimeMs);
            Assert.Equal(0x0C1, slots[1].Code);
            Assert.Equal("Servo error", slots[1].Name);
            Assert.Equal(0x0003FBC3, slots[1].TimeMs);
            Assert.Equal("00:04:21.059", slots[1].TimeAfterPowerOn);
            Assert.Equal(0xFFFF, slots[1].Address);
            Assert.Equal(AlarmLevel.OperationCancelled, slots[1].Level);

            var padded = words.Concat(new ushort[6]).Concat(T.Words("0000FFFF000000C100000001")).ToArray();
            Assert.Equal(4, Alarms.AlarmHistorySlots(padded).Count); // ends at the first empty slot
            Assert.Equal(6, Alarms.AlarmHistorySlots(padded, includeEmpty: true).Count);
            Assert.Equal("1 d 00:00:00.000", new AlarmRecord(0, 0x0C1, 0, 0xFFFF, 86400000).TimeAfterPowerOn);
        }

        [Fact]
        public void DetailDecoding()
        {
            Assert.Equal("row 3, field positioning band", Alarms.DescribeAlarmDetail(0x0A2, 3, 0x1032));
            Assert.Equal("direct-move field speed (0x0F02)", Alarms.DescribeAlarmDetail(0x0A3, 0, 0x0F02));
            Assert.Equal("parameter at 0x0812", Alarms.DescribeAlarmDetail(0x0A1, 0, 0x0812));
            Assert.Equal(string.Empty, Alarms.DescribeAlarmDetail(0x0A1, 0, 0xFFFF));
            Assert.Equal(string.Empty, Alarms.DescribeAlarmDetail(0x0C1, 0, 0x1032));
        }

        [Fact]
        public void InfoMatchesThePythonEdition()
        {
            var abs = Alarms.AlarmInfoOf(0x0EE);
            Assert.Equal("0x0EE", abs.Hex);
            Assert.Equal(AlarmLevel.OperationCancelled, abs.Level);
            Assert.Equal("operation cancelled on absolute units / cold start on incremental units", abs.LevelText);
            Assert.Equal("D", abs.Group);
            Assert.True(abs.Fault);
            Assert.Equal("the EC stopped and switched the servo off: alarm reset; if it stays, software reset or power cycle. Power-cycle; if it recurs, the controller or motor may need service", abs.Remedy);
            Assert.Equal("home lost: a home return after the reset (absolute units: alarm reset; incremental units: software reset or power cycle)", abs.HomeNeeded);

            var reset = Alarms.AlarmInfoOf(0x090);
            Assert.Equal("the EC stopped and switched the servo off: remove the cause, alarm reset, servo on. A software reset was sent with the servo on: alarm reset; switch the servo off before a software reset", reset.Remedy);
            Assert.Equal("alarm reset, then servo on", reset.Reset);

            var warning = Alarms.AlarmInfoOf(0x04E);
            Assert.True(warning.Warning);
            Assert.False(warning.Fault);

            var cold = Alarms.AlarmInfoOf(0x0E0);
            Assert.Equal("incremental units: home return after the reset", cold.HomeNeeded);

            Assert.Equal("row 1, field speed", Alarms.AlarmInfoOf(0x0A2, detail: 1, address: 0x1014).DetailText);
            Assert.Equal(string.Empty, Alarms.AlarmInfoOf(0).Remedy);
        }
    }

    public class RegistersTests
    {
        [Fact]
        public void WordHelpers()
        {
            Assert.Equal(((ushort)0xFFFF, (ushort)0xFFF5), Registers.ToI32Words(-11));
            Assert.Equal(-11, Registers.FromI32Words(0xFFFF, 0xFFF5));
            Assert.Equal(((ushort)0x0001, (ushort)0x3880), Registers.ToU32Words(80000));
            Assert.Equal(80000u, Registers.FromU32Words(0x0001, 0x3880));
            Assert.Throws<ArgumentOutOfRangeException>(() => Registers.ToI32Words(1L << 31));
            Assert.Throws<ArgumentOutOfRangeException>(() => Registers.ToU32Words(-1));
            Assert.Equal(7395, Registers.ToCounts(73.95));
            Assert.Equal(-30, Registers.ToCounts(-0.30));
            Assert.Throws<ArgumentOutOfRangeException>(() => Registers.ToCounts(double.NaN));
            Assert.Equal(76, Registers.PushPower(30)); // 76.5, half to even (as the Python edition)
            Assert.Equal(51, Registers.PushPower(20));
            Assert.Equal(178, Registers.PushPower(70));
            Assert.Equal(255, Registers.PushPower(100));
        }

        [Fact]
        public void TextHelpersReturnPrintableAscii()
        {
            Assert.Equal("EC-R", Registers.WordsToAscii(new ushort[] { 0x4543, 0x2D52, 0x0000, 0x2020 }));
            Assert.Equal("AB", Registers.WordsToAscii(new ushort[] { 0x41B8, 0x0742 })); // non-printable bytes dropped
            Assert.Null(Registers.WordsToText(new ushort[] { 0x00B8, 0x0000 })); // binary: not a text field
            Assert.Null(Registers.WordsToText(new ushort[] { 0x2020 })); // blank
            Assert.Equal("EC-R", Registers.WordsToText(new ushort[] { 0x4543, 0x2D52 }));
            Assert.Equal("abc", Registers.Printable("aéb\nc"));
            Assert.Equal(string.Empty, Registers.Printable(null));
        }

        [Fact]
        public void PositionTable()
        {
            Assert.Equal(0x1000, Registers.PositionAddress(0));
            Assert.Equal(0x1010, Registers.PositionAddress(1));
            Assert.Throws<ArgumentOutOfRangeException>(() => Registers.PositionAddress(0x300));
            // rod row 0 as read live: checksum word 0x00A0
            var row0 = T.Words("0000 0000 0000 000A 0000 5DC0 0000 0000 0000 0000 000A 002F 0000 0000 0000");
            Assert.Equal(0xA0, Registers.PositionChecksum(row0));
        }

        [Fact]
        public void CategoryExtents()
        {
            Assert.Equal(new BlockExtent(58, true, 58), Registers.CategoryExtent(0x0007, 29, 1, 2));
            Assert.Equal(new BlockExtent(192, true, 192), Registers.CategoryExtent(0x0005, 3, 32, 2));
            Assert.Equal(new BlockExtent(34, true, 34), Registers.CategoryExtent(0x0101, 22, 1, 0)); // monitor 0x9000-0x9021
            Assert.Equal(new BlockExtent(10, true, 10), Registers.CategoryExtent(0x0103, 7, 1, 0)); // 0x9900-0x9909
            Assert.Equal(new BlockExtent(40, true, 40), Registers.CategoryExtent(0x0114, 20, 1, 0));
            Assert.Equal(new BlockExtent(18, false, 36), Registers.CategoryExtent(0x0112, 18, 1, 0)); // unknown sizes
            Assert.Equal(new BlockExtent(20, false, 40), Registers.CategoryExtent(0x0101, 20, 1, 0)); // another DQ
            Assert.Equal(new BlockExtent(18, true, 18), Registers.CategoryExtent(0x0001, 11, 1, 0)); // support words
            Assert.Equal(new BlockExtent(8, true, 8), Registers.CategoryExtent(0x0001, 6, 1, 0));
        }

        [Fact]
        public void Families()
        {
            Assert.Equal("EC", Registers.ProductFamily(0xEC));
            Assert.Equal("PCON", Registers.ProductFamily(0xAE));
            Assert.Equal("IAI controller", Registers.ProductFamily(0x12));
            Assert.Equal("IAI controller", Registers.ProductFamily(null));
            Assert.True(Registers.IsEcFamily(0xED));
            Assert.False(Registers.IsEcFamily(0xAE));
            Assert.Equal("EC(MS) \"2-size EC\"", Registers.EcFamily(0xEC, 0x0A));
            Assert.Null(Registers.EcFamily(0xEC, 0x05));
            Assert.Equal("rod", Registers.AxisKindName(3));
            Assert.Equal("kind 42", Registers.AxisKindName(42));
            Assert.Equal("deg", Registers.AxisUnits(Registers.AKID_ROTARY));
            Assert.Equal("mm", Registers.AxisUnits(null));
        }

        [Fact]
        public void NeverSendTables()
        {
            Assert.Equal("clear position data", Registers.IAI_NEVER_SEND[0xF3]);
            Assert.Equal("factory inspection command", Registers.IAI_NEVER_SEND[0xDD]);
            Assert.Equal("write maintenance information", Registers.IAI_NEVER_SEND[0x3C]);
            Assert.False(Registers.IAI_NEVER_SEND.ContainsKey(0x3D));
            foreach (var command in ModbusRtu.IaiSafeCommands.Keys)
            {
                Assert.DoesNotContain(command, Registers.IAI_NEVER_SEND.Keys);
            }

            foreach (var command in ModbusRtu.IaiSafeCommands2.Keys)
            {
                Assert.DoesNotContain(command, Registers.IAI2_NEVER_SEND.Keys);
            }
        }
    }
}
