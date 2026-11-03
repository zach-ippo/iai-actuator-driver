using System;
using System.Linq;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Status decoding from real monitor words (0x9000...).</summary>
    public class StatusTests
    {
        // Live 0x9000 x20 reads (VERIFIED): the 3-position slider at 74.99 mm (stored middle 75.00) and the 2-position
        // gripper at its push end 9.50 mm.
        private static readonly ushort[] SliderLive = T.Words("0000 1D4B 0000 0000 000C 3099 8000 33C2 0008 801F 0000 0000 0000 0283 FFFF FFFF 0022 20C4 0300 4003");
        private static readonly ushort[] GripperLive = T.Words("0000 03B6 0000 0000 0006 3098 8002 32C2 0008 801F 0000 0000 0000 0000 0000 0000 0000 AEE9 0300 2003");

        // PCON-CB + RCP6 at power-up, PNOW..DSSE: -2.76 mm, ALMC 0x069 (real-time clock stop, message level), servo off.
        private static readonly ushort[] PconLive = T.Words("FFFF FEEC 0069 0000 6600 2210 8000 33C2");

        private static readonly ushort[] Idle = { 0, 7395, 0, 0, 0, 0x3098, 0x8000, 0x33C2 };

        [Fact]
        public void SliderLiveWords()
        {
            var st = Status.FromRegisters(SliderLive);
            Assert.Equal(74.99, st.PositionMm);
            Assert.Equal(((bool?)false, (bool?)true, (bool?)false), (st.AtBackwardEnd, st.AtMiddle, st.AtForwardEnd));
            Assert.Equal(0x000Cu, st.Raw["DOPM"]);
            Assert.Equal(0x4003u, st.Raw["ZONS"]);
            Assert.True(st.ServoOn);
            Assert.True(st.Homed);
            Assert.True(st.InPosition);
            Assert.True(st.TargetReached);
            Assert.True(st.ModbusEnabled);
            Assert.False(st.Moving);
            Assert.False(st.Alarm);
            Assert.Equal(0.0, st.VelocityMmS);
            Assert.Equal(0x0283, st.CurrentMa);
            Assert.Equal(0xFFFFFFFFu, st.Raw["DEVI"]);
            Assert.Equal(0x002220C4u, st.Raw["STIM"]);
            Assert.True(st.Ready);
            Assert.True(st.Idle);
        }

        [Fact]
        public void GripperLiveWordsDopmBit2IsNotMiddle()
        {
            var st = Status.FromRegisters(GripperLive);
            Assert.Equal(9.5, st.PositionMm);
            Assert.True((st.Raw["DOPM"] & 0x0004) != 0, "DOPM b2 (*ALM on the gripper) is set in the live word");
            Assert.False(st.AtMiddle);
            Assert.True(st.AtForwardEnd); // ZONS b13
            Assert.False(st.AtBackwardEnd);
            Assert.False(st.Alarm);
            Assert.False(st.ModbusEnabled); // DSSE 0x32C2: PMSS clear
            Assert.False(st.Ready);
        }

        [Fact]
        public void PconLiveWords()
        {
            var st = Status.FromRegisters(PconLive);
            Assert.Equal(-2.76, st.PositionMm);
            Assert.Equal(0x069, st.AlarmCode);
            Assert.Equal(AlarmLevel.Message, st.AlarmLevel);
            Assert.False(st.Alarm); // a message-level code is a warning, not a fault
            Assert.True(st.Warning);
            Assert.False(st.ServoOn);
            Assert.True(st.Homed);
            Assert.False(st.Moving);
            Assert.True(st.ModbusEnabled);
            Assert.Null(st.CurrentMa);
            Assert.Equal(0.0, st.VelocityMmS);
            Assert.Null(st.AtMiddle);
            Assert.Null(st.VnowMmS);
            Assert.Equal(0xFFFFFEECu, st.Raw["PNOW"]);
        }

        [Theory]
        [InlineData(8)]
        [InlineData(14)]
        public void NoMarkerWithoutZons(int n)
        {
            var st = Status.FromRegisters(GripperLive.Take(n).ToArray());
            Assert.Equal(((bool?)null, (bool?)null, (bool?)null), (st.AtBackwardEnd, st.AtMiddle, st.AtForwardEnd));
            Assert.False(st.Raw.ContainsKey("ZONS"));
        }

        [Fact]
        public void AlarmLogic()
        {
            var warn = (ushort[])Idle.Clone();
            warn[2] = 0x04E;
            warn[5] = (ushort)(0x3098 | Registers.DSS1_ALML);
            var st = Status.FromRegisters(warn);
            Assert.False(st.Alarm);
            Assert.True(st.Warning);

            var bare = (ushort[])Idle.Clone();
            bare[2] = 0x0E0; // code without any alarm bit: treated as an alarm (fail safe)
            Assert.True(Status.FromRegisters(bare).Alarm);

            var major = (ushort[])Idle.Clone();
            major[5] |= (ushort)Registers.DSS1_ALMH;
            Assert.True(Status.FromRegisters(major).Alarm);
            Assert.False(Status.FromRegisters(major).Warning);

            var unknownWithAlml = (ushort[])Idle.Clone();
            unknownWithAlml[2] = 0x030; // unknown level, warning bit set: not a fault
            unknownWithAlml[5] = (ushort)(0x3098 | Registers.DSS1_ALML);
            Assert.False(Status.FromRegisters(unknownWithAlml).Alarm);

            var negative = (ushort[])Idle.Clone();
            negative[0] = 0xFFFF;
            negative[1] = 0xFFF5;
            Assert.Equal(-0.11, Status.FromRegisters(negative).PositionMm);

            Assert.Throws<ArgumentException>(() => Status.FromRegisters(Idle.Take(5).ToArray()));
        }

        [Fact]
        public void MovingRules()
        {
            // servo on, homed, no PEND / PSFL / STP: still moving although the MOVE bit is clear (deceleration tail)
            var tail = (ushort[])Idle.Clone();
            tail[5] = 0x3090;
            var st = Status.FromRegisters(tail);
            Assert.True(st.Moving);
            Assert.False(st.MoveBit);
            Assert.False(st.Idle);

            var homing = (ushort[])Idle.Clone();
            homing[5] = 0x3088;
            homing[7] = 0x3BF2; // GHMS + MOVE
            Assert.True(Status.FromRegisters(homing).Homing);
            Assert.True(Status.FromRegisters(homing).Moving);

            var paused = (ushort[])Idle.Clone();
            paused[5] = 0x30B0; // STP
            Assert.True(Status.FromRegisters(paused).Paused);
            Assert.False(Status.FromRegisters(paused).Moving);

            var contact = (ushort[])Idle.Clone();
            contact[7] = 0x37C3; // push holding after contact
            Assert.True(Status.FromRegisters(contact).PushContact);
            Assert.True(Status.FromRegisters(contact).Pushing);
        }

        [Fact]
        public void VnowWhileMoving()
        {
            var words = SliderLive.Take(14).ToArray();
            words[5] = 0x3090; // moving
            words[7] = 0x33F2;
            words[10] = 0xFFFF; // VNOW = -1248 -> -12.48 mm/s
            words[11] = 0xFB20;
            var st = Status.FromRegisters(words);
            Assert.True(st.Moving);
            Assert.Equal(-12.48, st.VelocityMmS);
            Assert.Equal(-12.48, st.VnowMmS);
        }
    }
}
