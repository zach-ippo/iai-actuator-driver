using System.Text;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>CRC-16/MODBUS and frame building against frames from IAI manual ME0162 and frames exchanged with units.</summary>
    public class CrcTests
    {
        // Complete frames from ME0162 (CRCs independently re-checked); the IAI ones at the end were sent to an EC on
        // hardware and answered (VERIFIED), the index-header and 0x48 frames are reference encodings.
        public static readonly TheoryData<string> KnownFrames = new TheoryData<string>
        {
            "01 03 90 00 00 0A E8 CD", // read 9000h x10
            "01 03 90 00 00 02 E9 0B", // read PNOW
            "01 03 04 00 00 0B FE 7C 83", // PNOW response = 30.70 mm
            "01 03 02 30 98 AD EE", // DSS1 response 3098h
            "01 03 14 00 00 00 00 00 00 00 00 6E 00 60 18 80 00 23 C7 00 00 00 19 18 A6", // burst read response
            "01 05 04 03 FF 00 7D 0A", // SON on
            "01 05 04 03 00 00 3C FA", // SON off
            "01 05 04 07 FF 00 3C CB", // ALRS on
            "01 05 04 07 00 00 7D 3B", // ALRS off
            "01 05 04 0A FF 00 AD 08", // STP on
            "01 05 04 0B 00 00 BD 38", // HOME off
            "01 05 04 0B FF 00 FC C8", // HOME on
            "01 05 04 27 FF 00 3D 01", // PMSL on
            "01 05 04 2C FF 00 4C C3", // STOP
            "01 06 0D 00 10 00 86 A6", // DRG1 = SON
            "01 06 98 00 00 01 67 6A", // POSR immediate move to 1
            "01 10 99 00 00 02 04 00 00 13 88 38 AF", // move to 50.00 mm
            "01 10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 50 CF", // 50 mm, 0.1, 100 mm/s, 0.3 G
            "01 10 99 00 00 07 AF 57", // its response
            "01 10 10 C0 00 0F 84 F1", // position-table write response
            "01 11 C0 2C", // report slave id (VERIFIED)
            "01 03 00 00 00 08 44 0C", // index header read (VERIFIED)
            "01 03 0F 00 00 14 46 D1", // active profile read (class 0x0F, 20 words)
            "01 45 00 09 00 01 00 00 18 C2", // 0x45 servo on (VERIFIED)
            "01 45 00 0A 00 00 00 00 0D 02", // 0x45 home, op1 0 (VERIFIED: rejected 0x11; op1 must be ORG)
            "00 45 00 04 00 14 00 05 25 08", // broadcast baud change to 115200 (ON-WIRE)
            "01 48 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D5 98", // 0x48: group 0, cmd 0x0040, 4 x u32 0
        };

        [Fact]
        public void KnownVector()
        {
            var crc = Crc16.Compute(T.Hex("01 03 90 00 00 0A"));
            Assert.Equal(0xCDE8, crc); // E8 CD on the wire, low byte first
        }

        [Fact]
        public void StandardCheckValue() => Assert.Equal(0x4B37, Crc16.Compute(Encoding.ASCII.GetBytes("123456789")));

        [Fact]
        public void EmptyInput() => Assert.Equal(0xFFFF, Crc16.Compute(new byte[0]));

        [Theory]
        [MemberData(nameof(KnownFrames))]
        public void FramesCheckAndRebuild(string text)
        {
            var frame = T.Hex(text);
            Assert.True(Crc16.Check(frame));
            var pdu = new byte[frame.Length - 3];
            System.Array.Copy(frame, 1, pdu, 0, pdu.Length);
            Assert.Equal(frame, Crc16.BuildFrame(frame[0], pdu));
        }

        [Fact]
        public void CorruptionDetected()
        {
            var frame = T.Hex("01 03 90 00 00 0A E8 CD");
            frame[3] ^= 0x01;
            Assert.False(Crc16.Check(frame));
            Assert.False(Crc16.Check(T.Hex("01 03")));
        }
    }
}
