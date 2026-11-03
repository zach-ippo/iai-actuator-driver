using System;
using System.Collections.Generic;

namespace IaiActuatorDriver
{
    /// <summary>
    /// CRC-16/MODBUS (polynomial 0xA001 reflected, init 0xFFFF) and RTU frame helpers. On the wire the CRC is sent low
    /// byte first (IAI manual ME0162; DOC, and VERIFIED on every frame exchanged with the units tested).
    /// </summary>
    public static class Crc16
    {
        /// <summary>CRC-16/MODBUS of <paramref name="data"/> (the catalogue check value of "123456789" is 0x4B37).</summary>
        public static ushort Compute(IReadOnlyList<byte> data) => Compute(data, 0, data.Count);

        /// <summary>CRC-16/MODBUS of <paramref name="count"/> bytes of <paramref name="data"/> from <paramref name="offset"/>.</summary>
        public static ushort Compute(IReadOnlyList<byte> data, int offset, int count)
        {
            if (data is null)
            {
                throw new ArgumentNullException(nameof(data));
            }

            if (offset < 0 || count < 0 || offset + count > data.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(count));
            }

            int crc = 0xFFFF;
            for (var i = offset; i < offset + count; i++)
            {
                crc ^= data[i];
                for (var bit = 0; bit < 8; bit++)
                {
                    crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xA001 : crc >> 1;
                }
            }

            return (ushort)crc;
        }

        /// <summary>True if the last two bytes of <paramref name="frame"/> (low byte first) are the CRC of the rest.</summary>
        public static bool Check(IReadOnlyList<byte> frame)
        {
            if (frame is null || frame.Count < 4)
            {
                return false;
            }

            var crc = Compute(frame, 0, frame.Count - 2);
            return frame[frame.Count - 2] == (crc & 0xFF) && frame[frame.Count - 1] == crc >> 8;
        }

        /// <summary>The RTU frame <c>[slave][pdu...][crc lo][crc hi]</c>. No address check: slave 0 is the broadcast.</summary>
        public static byte[] BuildFrame(int slave, IReadOnlyList<byte> pdu)
        {
            if (slave < 0 || slave > 0xFF)
            {
                throw new ArgumentOutOfRangeException(nameof(slave), slave, "slave address must be 0..255");
            }

            var frame = new byte[pdu.Count + 3];
            frame[0] = (byte)slave;
            for (var i = 0; i < pdu.Count; i++)
            {
                frame[i + 1] = pdu[i];
            }

            var crc = Compute(frame, 0, frame.Length - 2);
            frame[frame.Length - 2] = (byte)(crc & 0xFF);
            frame[frame.Length - 1] = (byte)(crc >> 8);
            return frame;
        }
    }
}
