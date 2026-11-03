using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Hex / frame helpers shared by the test classes.</summary>
    internal static class T
    {
        /// <summary>"01 03 90 00" (spaces optional) -> bytes.</summary>
        public static byte[] Hex(string text)
        {
            var clean = text.Replace(" ", string.Empty);
            return Enumerable.Range(0, clean.Length / 2)
                .Select(i => byte.Parse(clean.Substring(2 * i, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture))
                .ToArray();
        }

        /// <summary>"0000 1D4B ..." or a continuous hex string -> 16-bit words.</summary>
        public static ushort[] Words(string text)
        {
            var b = Hex(text);
            return Enumerable.Range(0, b.Length / 2).Select(i => (ushort)((b[2 * i] << 8) | b[(2 * i) + 1])).ToArray();
        }

        /// <summary>A complete frame for <paramref name="slave"/> with the PDU given in hex.</summary>
        public static byte[] Response(int slave, string pdu) => Crc16.BuildFrame(slave, Hex(pdu));

        /// <summary>An FC03 response carrying <paramref name="values"/>.</summary>
        public static byte[] ReadResponse(int slave, params ushort[] values)
        {
            var pdu = new List<byte> { 0x03, (byte)(2 * values.Length) };
            foreach (var v in values)
            {
                pdu.Add((byte)(v >> 8));
                pdu.Add((byte)v);
            }

            return Crc16.BuildFrame(slave, pdu);
        }

        /// <summary>A master on a fresh fake with the given script, no inter-frame gap.</summary>
        public static (ModbusRtu Bus, FakeTransport Transport) Scripted(params byte[]?[] responses) => ScriptedWith(2, responses);

        /// <summary>As <see cref="Scripted"/>, with <paramref name="retries"/>.</summary>
        public static (ModbusRtu Bus, FakeTransport Transport) ScriptedWith(int retries, params byte[]?[] responses)
        {
            var transport = new FakeTransport();
            transport.Enqueue(responses);
            return (new ModbusRtu(transport, retries, TimeSpan.Zero), transport);
        }
    }

    /// <summary>
    /// A fake unit behind a <see cref="FakeTransport"/> responder: FC03 from a register map (unmapped registers read 0, like
    /// a real unit), FC11 from <see cref="SlaveId"/> (null = silent), anything else an Illegal Function exception.
    /// </summary>
    internal sealed class FakeUnit
    {
        public FakeUnit(int address = 1) => Address = address;

        public int Address { get; }

        public Dictionary<int, ushort> Memory { get; } = new Dictionary<int, ushort>();

        public byte[]? SlaveId { get; set; }

        public List<(int Address, int Count)> Reads { get; } = new List<(int, int)>();

        public void Put(int address, IReadOnlyList<ushort> words)
        {
            for (var i = 0; i < words.Count; i++)
            {
                Memory[address + i] = words[i];
            }
        }

        public void Put(int address, string hex) => Put(address, T.Words(hex));

        public byte[]? Respond(byte[] request)
        {
            if (request[0] != Address || !Crc16.Check(request))
            {
                return null;
            }

            switch (request[1])
            {
                case 0x03:
                    var address = (request[2] << 8) | request[3];
                    var count = (request[4] << 8) | request[5];
                    Reads.Add((address, count));
                    var values = Enumerable.Range(address, count).Select(a => Memory.TryGetValue(a, out var v) ? v : (ushort)0).ToArray();
                    return T.ReadResponse(Address, values);
                case 0x11:
                    if (SlaveId is null)
                    {
                        return null;
                    }

                    var pdu = new List<byte> { 0x11, (byte)SlaveId.Length };
                    pdu.AddRange(SlaveId);
                    return Crc16.BuildFrame(Address, pdu);
                default:
                    return Crc16.BuildFrame(Address, new byte[] { (byte)(request[1] | 0x80), 0x01 });
            }
        }

        /// <summary>A responder serving several units on one line (an RCON / daisy-chain layout).</summary>
        public static Func<byte[], byte[]?> Line(params FakeUnit[] units) =>
            request => units.Select(u => u.Respond(request)).FirstOrDefault(r => r != null);
    }

    /// <summary>Data read from the EC-R6S-150-1-PN rod (EC #3, 2026-10-28; serial numbers replaced by placeholders).</summary>
    internal static class RodFixture
    {
        public const string IndexTable =
            "000100080016010001000000000000000001010000060001010000000000000000020002000A00010200028001000001" +
            "00030004003A000103000380000000020004000200150001040004800000000200050002000300200500580000000002" +
            "0006000100240001060000000000000000070102001A0001070051000000000200080002002400010800520000000002" +
            "00090002003400010900530000000002000C0000000200020C00560000000002000D0002000A00010D000D8004000001" +
            "000F0100000D00010F000000000000000010010100080300100050000000000200120002000700017F00000000000000" +
            "010000020010000180000000000000000013000000100001810000000000000000140000001000018200000000000000" +
            "02000000001A000184008480000000020101000300160001900000000000000001020001000300019800000000000000" +
            "0103000100070001990000000000000001040000000300019A00000000000000";

        public const string Identification =
            "53494D2D30303030303033202020202053494D2D303030303030332020202020303030300000000020170406B2002380" +
            "0000000000000000000000000000000045432D5236532D3135302D312D504E2020202020202020202020202020202020" +
            "202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020";

        /// <summary>Class 7 data (0x0700, 26 x 2 words).</summary>
        public const string Class7 =
            "00003AB6FFFFFFE2000007D000013880000000640000006405350000000007D00000004600000000014A007400000096" +
            "0000177000001770FFFFFFFFFFFFFFFF000001F40000003200000064000000640000001E000001F4000003E800000003" +
            "0000003800000000";

        /// <summary>Class 7 ID map (0x5100, 26 x (id, 0)).</summary>
        public const string Class7Ids =
            "000100000002000000030000000400000005000000060000000700000008000000090000000A0000000F000000100000" +
            "001100000012000000130000001400000015000000160000001700000018000000190000001A0000001D000000210000" +
            "0022000000230000";

        public const string AlarmHistory =
            "0000FFFF00000FFFFFFFFFFF0000FFFF000000C10003FBC30000FFFF000000C1000311080000FFFF00000FFFFFFFFFFF";

        /// <summary>A fake rod at <paramref name="address"/>: directory, identification, class 7, CTCL, FC11 EC FF 00 00 02.</summary>
        public static FakeUnit Unit(int address = 1)
        {
            var unit = new FakeUnit(address) { SlaveId = T.Hex("EC FF 00 00 02") };
            unit.Put(0x0000, IndexTable);
            unit.Put(0x0600, Identification);
            unit.Put(0x0700, Class7);
            unit.Put(0x5100, Class7Ids);
            unit.Put(0x0300, "EC000002"); // CTCL, class 3 item 1
            unit.Put(0x0380, "00010000"); // class 3 ID map, first entry
            unit.Put(0x8004, new ushort[] { 0, (ushort)(address - 1) }); // AXNO
            return unit;
        }
    }
}
