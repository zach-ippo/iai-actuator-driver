using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Frame building and parsing for every function code.</summary>
    public class FramingTests
    {
        [Fact]
        public void ReadRegistersRequestAndDecode()
        {
            var (bus, t) = T.Scripted(T.Hex("01 03 04 00 00 0B FE 7C 83"));
            Assert.Equal(new ushort[] { 0x0000, 0x0BFE }, bus.ReadRegisters(1, 0x9000, 2));
            Assert.Equal(T.Hex("01 03 90 00 00 02 E9 0B"), Assert.Single(t.Writes));
        }

        [Fact]
        public void WriteCoilFrames()
        {
            var (bus, t) = T.Scripted(T.Hex("01 05 04 03 FF 00 7D 0A"), T.Hex("01 05 04 03 00 00 3C FA"));
            bus.WriteCoil(1, 0x0403, true);
            bus.WriteCoil(1, 0x0403, false);
            Assert.Equal(new[] { T.Hex("01 05 04 03 FF 00 7D 0A"), T.Hex("01 05 04 03 00 00 3C FA") }, t.Writes);
        }

        [Fact]
        public void WriteRegisterFrame()
        {
            var (bus, t) = T.Scripted(T.Hex("01 06 0D 00 10 00 86 A6"));
            bus.WriteRegister(1, 0x0D00, 0x1000);
            Assert.Equal(T.Hex("01 06 0D 00 10 00 86 A6"), Assert.Single(t.Writes));
        }

        [Fact]
        public void WriteRegistersFrame()
        {
            var (bus, t) = T.Scripted(T.Hex("01 10 99 00 00 07 AF 57"));
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 5000, 0, 10, 0, 10000, 30 });
            Assert.Equal(T.Hex("01 10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 50 CF"), Assert.Single(t.Writes));
        }

        [Fact]
        public void ReadWriteRegistersFrame()
        {
            var (bus, t) = T.Scripted(T.Response(1, "17 04 00 01 00 02"));
            Assert.Equal(new ushort[] { 1, 2 }, bus.ReadWriteRegisters(1, 0x9000, 2, 0x9900, new ushort[] { 0x0000, 0x1388 }));
            Assert.Equal(T.Response(1, "17 90 00 00 02 99 00 00 02 04 00 00 13 88"), Assert.Single(t.Writes));
        }

        [Fact]
        public void ReportSlaveIdVariableLength()
        {
            var (bus, t) = T.Scripted(T.Response(1, "11 05 EC FF 00 00 20"));
            Assert.Equal(T.Hex("EC FF 00 00 20"), bus.ReportSlaveId(1));
            Assert.Equal(T.Hex("01 11 C0 2C"), Assert.Single(t.Writes));
        }

        [Fact]
        public void IaiCommandFrame()
        {
            var (bus, t) = T.Scripted(T.Response(1, "45 00 00"));
            var result = bus.IaiCommand(1, 0, 0x09, 1);
            Assert.Equal(new IaiResponse(0, 0), result);
            Assert.Equal(T.Hex("01 45 00 09 00 01 00 00 18 C2"), Assert.Single(t.Writes));
        }

        [Fact]
        public void IaiCommandRejectedDecoded()
        {
            var (bus, _) = T.Scripted(T.Response(1, "45 21 07"));
            var exc = Assert.Throws<IaiCommandRejectedException>(() => bus.IaiCommand(1, 0, 0x0A));
            Assert.Equal(0x21, exc.Rejection);
            Assert.Equal(0x07, exc.StatusByte);
            Assert.Equal("command code error", exc.Reason);
            var (bus2, _) = T.Scripted(T.Response(1, "45 21 07"));
            Assert.Equal(0x21, bus2.IaiCommand(1, 0, 0x0A, check: false).Rejection);
        }

        [Fact]
        public void IaiCommand2Frame()
        {
            var (bus, t) = T.Scripted(T.Response(1, "48 00 00" + " 00 00 00 01" + " 00 00 00 00 00 00 00 00 00 00 00 00"));
            var result = bus.IaiCommand2(1, 0x30, new uint[] { 1 });
            Assert.Equal(0, result.Rejection);
            Assert.Equal(new uint[] { 1, 0, 0, 0 }, result.Status);
            var request = Assert.Single(t.Writes);
            Assert.Equal(2 + 19 + 2, request.Length);
            Assert.Equal(T.Hex("01 48 00 00 30 00 00 00 01"), request.Take(9).ToArray()); // addr, 0x48, group 0, cmd u16, op1 u32
            Assert.All(request.Skip(9).Take(12), b => Assert.Equal(0, b));
        }

        [Fact]
        public void IaiCommand2ReferenceFrame()
        {
            // 0x0040 is "clear the alarm list" (never-send): built here only to check the frame layout against a fake.
            var (bus, t) = T.Scripted(T.Response(1, "48 00 00" + string.Concat(Enumerable.Repeat(" 00", 16))));
            bus.IaiCommand2(1, 0x0040, allowUnsafe: true, allowNeverSend: true);
            Assert.Equal(T.Hex("01 48 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D5 98"), Assert.Single(t.Writes));
        }

        [Fact]
        public void BroadcastBaudFrame()
        {
            var (bus, t) = T.Scripted();
            t.Inject(0x55, 0xAA); // line noise before
            var frame = bus.BroadcastIaiCommand(0, Registers.IAI_CMD_BAUD, 0x14, Registers.IAI_BAUD_OP2, allowUnsafe: true, allowNeverSend: true, settle: TimeSpan.Zero);
            Assert.Equal(T.Hex("00 45 00 04 00 14 00 05 25 08"), frame);
            Assert.Equal(frame, Assert.Single(t.Writes));
        }

        [Fact]
        public void EchoMismatchIsFrameErrorAfterRetries()
        {
            var wrong = T.Hex("01 05 04 03 00 00 3C FA"); // answers OFF to an ON request
            var (bus, t) = T.Scripted(wrong, wrong, wrong);
            Assert.Throws<ModbusFrameException>(() => bus.WriteCoil(1, 0x0403, true));
            Assert.Equal(3, t.Writes.Count);
        }

        [Fact]
        public void EchoMismatchIsRetried()
        {
            var (bus, t) = T.Scripted(T.Hex("01 05 04 03 00 00 3C FA"), T.Hex("01 05 04 03 FF 00 7D 0A")); // a stale OFF echo, then the ON echo
            bus.WriteCoil(1, 0x0403, true);
            Assert.Equal(2, t.Writes.Count);
        }

        [Fact]
        public void ArgumentValidationSendsNothing()
        {
            var (bus, t) = T.Scripted();
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadRegisters(0, 0x9000, 1)); // broadcast not supported
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadRegisters(248, 0x9000, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadRegisters(1, 0x9000, 0));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadRegisters(1, 0xFFFF, 2));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.WriteRegister(1, 0x10000, 1));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.WriteRegisters(1, 0x9900, new ushort[0]));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.WriteRegisters(1, 0x9900, new ushort[124]));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadWriteRegisters(1, 0x9000, 61, 0x9900, new ushort[1]));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.IaiCommand(1, 0, 0x09, 0x10000));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.IaiCommand2(1, 0x10030));
            Assert.Throws<ArgumentException>(() => bus.IaiCommand2(1, 0x30, new uint[5]));
            Assert.Empty(t.Writes);
        }
    }

    /// <summary>Exception responses, retries, timeouts and partial frames.</summary>
    public class RetryTests
    {
        [Fact]
        public void ExceptionResponseNotRetried()
        {
            var (bus, t) = T.Scripted(T.Response(1, "83 02"));
            var exc = Assert.Throws<ModbusSlaveException>(() => bus.ReadRegisters(1, 0x7000, 1));
            Assert.Equal(2, exc.Code);
            Assert.Equal(3, exc.Function);
            Assert.Contains("Illegal Data Address", exc.Message);
            Assert.Single(t.Writes);
        }

        [Fact]
        public void TimeoutThenSuccess()
        {
            var (bus, t) = T.Scripted(null, T.ReadResponse(1, 7));
            Assert.Equal(new ushort[] { 7 }, bus.ReadRegisters(1, 0x9002, 1));
            Assert.Equal(2, t.Writes.Count);
        }

        [Fact]
        public void TimeoutAfterRetries()
        {
            var (bus, t) = T.Scripted(null, null, null, null);
            var exc = Assert.Throws<ModbusTimeoutException>(() => bus.ReadRegisters(1, 0x9002, 1));
            Assert.Equal(3, t.Writes.Count); // 1 + 2 retries
            Assert.Equal(3, exc.Attempts);
            Assert.Contains("gave up after 3 attempts", exc.Message);
        }

        [Fact]
        public void DefaultRetriesIsThree()
        {
            // IAI recommends 3 retries (the scripted helper above pins 2 so its scripts stay short).
            Assert.Equal(3, ModbusRtu.DefaultRetries);
            Assert.Equal(3, new ModbusRtu(new FakeTransport()).Retries);
        }

        [Fact]
        public void RetriesZero()
        {
            var (bus, t) = T.ScriptedWith(0, null, T.ReadResponse(1, 7));
            var exc = Assert.Throws<ModbusTimeoutException>(() => bus.ReadRegisters(1, 0x9002, 1));
            Assert.Single(t.Writes);
            Assert.DoesNotContain("gave up", exc.Message);
        }

        [Fact]
        public void RetriesPerCallOverrideOnlyThatCall()
        {
            var (bus, t) = T.Scripted(null, T.ReadResponse(1, 7), null, T.Response(1, "11 05 EC FF 02 00 13"));
            Assert.Throws<ModbusTimeoutException>(() => bus.ReadRegisters(1, 0x9002, 1, retries: 0));
            Assert.Single(t.Writes);
            Assert.Equal(new ushort[] { 7 }, bus.ReadRegisters(1, 0x9002, 1)); // the default (2 retries) is untouched
            Assert.Equal(2, bus.Retries);
            Assert.Throws<ModbusTimeoutException>(() => bus.ReportSlaveId(1, retries: 0, timeout: TimeSpan.FromMilliseconds(1)));
            Assert.Equal(T.Hex("EC FF 02 00 13"), bus.ReportSlaveId(1));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.ReadRegisters(1, 0x9002, 1, retries: -1));
        }

        [Fact]
        public void CrcErrorThenSuccess()
        {
            var bad = T.ReadResponse(1, 7);
            bad[bad.Length - 1] ^= 0xFF;
            var (bus, t) = T.Scripted(bad, T.ReadResponse(1, 7));
            Assert.Equal(new ushort[] { 7 }, bus.ReadRegisters(1, 0x9002, 1));
            Assert.Equal(2, t.Writes.Count);
        }

        [Fact]
        public void CrcErrorAfterRetries()
        {
            var bad = T.ReadResponse(1, 7);
            bad[bad.Length - 2] ^= 0x01;
            var (bus, _) = T.Scripted(bad, bad, bad);
            Assert.Throws<ModbusCrcException>(() => bus.ReadRegisters(1, 0x9002, 1));
        }

        [Fact]
        public void PartialResponseIsTimeoutAndRetried()
        {
            var full = T.ReadResponse(1, 1, 2);
            var log = new List<(FrameDirection, byte[])>();
            var (bus, t) = T.Scripted(full.Take(4).ToArray(), full);
            bus.FrameLogger = (d, f) => log.Add((d, f));
            Assert.Equal(new ushort[] { 1, 2 }, bus.ReadRegisters(1, 0x9000, 2));
            Assert.Equal(2, t.Writes.Count);
            Assert.Contains(log, e => e.Item1 == FrameDirection.RxPartial && e.Item2.Length == 4);
        }

        [Fact]
        public void WrongSlaveAndWrongFunctionRetried()
        {
            var (bus, t) = T.Scripted(T.ReadResponse(2, 1), T.Response(1, "06 90 02 00 01"), T.ReadResponse(1, 9));
            Assert.Equal(new ushort[] { 9 }, bus.ReadRegisters(1, 0x9002, 1));
            Assert.Equal(3, t.Writes.Count);
            var (bus2, _) = T.Scripted(T.ReadResponse(2, 1), T.ReadResponse(2, 1), T.ReadResponse(2, 1));
            Assert.Throws<ModbusFrameException>(() => bus2.ReadRegisters(1, 0x9002, 1));
        }

        [Fact]
        public void BadCrcOnExceptionFrameIsRetried()
        {
            var bad = T.Response(1, "83 02");
            bad[bad.Length - 1] ^= 0xFF;
            var (bus, t) = T.Scripted(bad, T.Response(1, "83 02"));
            Assert.Throws<ModbusSlaveException>(() => bus.ReadRegisters(1, 0x7000, 1));
            Assert.Equal(2, t.Writes.Count);
        }

        [Fact]
        public void WaitsForALateAnswerWithinTheTimeout()
        {
            // FC11 waits up to SlaveIdTimeout: an answer arriving after the transport's own (instant) read still counts.
            var transport = new FakeTransport();
            var bus = new ModbusRtu(transport, 0, TimeSpan.Zero);
            transport.Responder = request =>
            {
                Task.Delay(30).ContinueWith(_ => transport.Inject(T.Response(1, "11 05 EC FF 00 00 02")));
                return null;
            };
            Assert.Equal(T.Hex("EC FF 00 00 02"), bus.ReportSlaveId(1, timeout: TimeSpan.FromSeconds(2)));
        }
    }

    /// <summary>Splitting, the inter-frame gap, the frame logger, closing and threads.</summary>
    public class BusBehaviourTests
    {
        [Fact]
        public void LargeReadsAreSplit()
        {
            var unit = new FakeUnit();
            unit.Put(0x1000, Enumerable.Range(0, 130).Select(i => (ushort)i).ToArray());
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            var values = bus.ReadRegisters(1, 0x1000, 130);
            Assert.Equal(Enumerable.Range(0, 130).Select(i => (ushort)i), values);
            Assert.Equal(new[] { (0x1000, 60), (0x103C, 60), (0x1078, 10) }, unit.Reads);
        }

        [Fact]
        public void MinimumGapBetweenRequests()
        {
            var transport = new FakeTransport();
            transport.Enqueue(Enumerable.Repeat(T.ReadResponse(1, 0), 5).ToArray());
            var bus = new ModbusRtu(transport); // default 5 ms gap
            for (var i = 0; i < 5; i++)
            {
                bus.ReadRegisters(1, 0x9002, 1);
            }

            var times = transport.WriteTimes;
            for (var i = 1; i < times.Count; i++)
            {
                Assert.True(times[i] - times[i - 1] >= 0.0049, $"gap {times[i] - times[i - 1]:F4} s");
            }
        }

        [Fact]
        public void FrameLoggerSeesTxAndRx()
        {
            var frames = new List<(FrameDirection, byte[])>();
            var transport = new FakeTransport();
            transport.Enqueue(T.Hex("01 03 04 00 00 0B FE 7C 83"));
            var bus = new ModbusRtu(transport, frameLogger: (d, f) => frames.Add((d, f)));
            bus.ReadRegisters(1, 0x9000, 2);
            Assert.Equal(2, frames.Count);
            Assert.Equal(FrameDirection.Tx, frames[0].Item1);
            Assert.Equal(T.Hex("01 03 90 00 00 02 E9 0B"), frames[0].Item2);
            Assert.Equal(FrameDirection.Rx, frames[1].Item1);
            Assert.Equal(T.Hex("01 03 04 00 00 0B FE 7C 83"), frames[1].Item2);
        }

        [Fact]
        public void BrokenLoggerDoesNotBreakIo()
        {
            var logged = new List<string>();
            void Sink(DriverLogLevel level, string source, string message, Exception? exc) => logged.Add(message);
            DriverLog.Message += Sink;
            try
            {
                var (bus, _) = T.Scripted(T.ReadResponse(1, 0));
                bus.FrameLogger = (d, f) => throw new InvalidOperationException("logger bug");
                Assert.Equal(new ushort[] { 0 }, bus.ReadRegisters(1, 0x9002, 1));
                Assert.Contains("frame logger raised", logged);
            }
            finally
            {
                DriverLog.Message -= Sink;
            }
        }

        [Fact]
        public void ClosedBus()
        {
            var (bus, t) = T.Scripted();
            bus.Close();
            bus.Close();
            Assert.True(t.Closed);
            Assert.True(bus.IsClosed);
            Assert.Throws<ModbusException>(() => bus.ReadRegisters(1, 0x9000, 2));
        }

        [Fact]
        public async Task ThreadsShareTheBus()
        {
            var unit = new FakeUnit();
            unit.Put(0x9005, new ushort[] { 0x3098 });
            var bus = new ModbusRtu(new FakeTransport(unit.Respond), minGap: TimeSpan.Zero);
            var tasks = Enumerable.Range(0, 4).Select(_ => Task.Run(() =>
            {
                for (var i = 0; i < 40; i++)
                {
                    Assert.Equal(new ushort[] { 0x3098 }, bus.ReadRegisters(1, 0x9005, 1));
                }
            })).ToArray();
            await Task.WhenAll(tasks);
            Assert.Equal(160, unit.Reads.Count);
        }

        [Fact]
        public void TransportErrorIsModbusExceptionAndNotRetried()
        {
            var (bus, t) = T.Scripted();
            t.Close(); // the fake throws an IOException on write once closed
            var exc = Assert.Throws<ModbusException>(() => bus.ReadRegisters(1, 0x9000, 1));
            Assert.Contains("transport error", exc.Message);
            Assert.IsType<System.IO.IOException>(exc.InnerException);
        }

        [Fact]
        public void ProbeShortensAndRestoresTheReadTimeout()
        {
            var seen = new List<TimeSpan>();
            var transport = new FakeTransport { ReadTimeout = TimeSpan.FromSeconds(0.2) };
            transport.Responder = request =>
            {
                seen.Add(transport.ReadTimeout);
                return null;
            };
            var bus = new ModbusRtu(transport, minGap: TimeSpan.Zero);
            Assert.False(bus.Probe(1, TimeSpan.FromMilliseconds(20)));
            Assert.Equal(TimeSpan.FromMilliseconds(20), Assert.Single(seen));
            Assert.Equal(TimeSpan.FromSeconds(0.2), transport.ReadTimeout);
        }

        [Fact]
        public void SendBreakUsesTheTransport()
        {
            var (bus, t) = T.Scripted();
            bus.SendBreak(TimeSpan.FromMilliseconds(200));
            Assert.Equal(TimeSpan.FromMilliseconds(200), Assert.Single(t.Breaks));
            Assert.Throws<ArgumentOutOfRangeException>(() => bus.SendBreak(TimeSpan.FromSeconds(3)));
        }
    }

    /// <summary>The safe list, the never-send lists and rejection decoding.</summary>
    public class SafetyTests
    {
        [Fact]
        public void UnsafeIaiCommandsRefused()
        {
            var (bus, t) = T.Scripted();
            foreach (var command in new[] { 0x01, 0x02, 0x04, 0x0F, 0x10, 0x1D, 0x73, 0xF3, 0xFF, 0x15 })
            {
                Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand(1, 0, command));
            }

            Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand(1, 0, 0x05, 1)); // software reset
            foreach (var command in new[] { 0x10, 0x20, 0x24, 0x40, 0x41, 0x80 })
            {
                Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand2(1, command));
            }

            Assert.Empty(t.Writes);
        }

        [Fact]
        public void GroupBypassClosed()
        {
            var (bus, t) = T.Scripted();
            Assert.Contains("group", Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand(1, 0x05, 0x09, 1)).Message);
            foreach (var command in new[] { 0x1030, 0x1000, 0xFFFD, 0x0130 })
            {
                Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand2(1, command));
            }

            Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand2(1, 0x30, new uint[] { 1 }, group: 0x10));
            Assert.Empty(t.Writes);
            var (bus2, t2) = T.Scripted(T.Response(1, "45 00 00"));
            bus2.IaiCommand(1, 0x05, 0x09, 1, allowUnsafe: true);
            Assert.Equal(T.Hex("01 45 05 09"), t2.Writes[0].Take(4).ToArray());
        }

        [Fact]
        public void UnsafeAllowedExplicitly()
        {
            var (bus, t) = T.Scripted(T.Response(1, "45 00 00"));
            bus.IaiCommand(1, 0, 0x15, 0, 2, allowUnsafe: true); // not on the safe list, not on the never-send list
            Assert.Single(t.Writes);
        }

        [Theory]
        [InlineData(0x01)]
        [InlineData(0x02)]
        [InlineData(0x03)]
        [InlineData(0x04)]
        [InlineData(0x0F)]
        [InlineData(0x10)]
        [InlineData(0x11)]
        [InlineData(0x1D)]
        [InlineData(0x20)]
        [InlineData(0x3C)]
        [InlineData(0x4F)]
        [InlineData(0x73)]
        [InlineData(0x90)]
        [InlineData(0x91)]
        [InlineData(0xDD)]
        [InlineData(0xE8)]
        [InlineData(0xF3)]
        [InlineData(0xFF)]
        public void NeverSendListNeedsItsOwnFlag(int command)
        {
            var (bus, t) = T.Scripted(T.Response(1, "45 00 00"));
            Assert.Contains("never-send", Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand(1, 0, command, allowUnsafe: true)).Message);
            Assert.Throws<UnsafeCommandException>(() => bus.BroadcastIaiCommand(0, command, allowUnsafe: true, settle: TimeSpan.Zero));
            Assert.Empty(t.Writes);
        }

        [Theory]
        [InlineData(0x0010)]
        [InlineData(0x0012)]
        [InlineData(0x0020)]
        [InlineData(0x0040)]
        [InlineData(0x0041)]
        [InlineData(0x0080)]
        [InlineData(0x1000)]
        [InlineData(0xFFFD)]
        public void NeverSendList2NeedsItsOwnFlag(int command)
        {
            var (bus, t) = T.Scripted(T.Response(1, "48 00 00" + string.Concat(Enumerable.Repeat(" 00", 16))));
            Assert.Contains("never-send", Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand2(1, command, allowUnsafe: true)).Message);
            Assert.Empty(t.Writes);
        }

        [Fact]
        public void NeverSendOnlyOnExplicitRequest()
        {
            var (bus, t) = T.Scripted(T.Response(1, "45 00 00"));
            bus.IaiCommand(1, 0, 0x0F, allowUnsafe: true, allowNeverSend: true);
            Assert.Single(t.Writes);
        }

        [Theory]
        [InlineData(0x11, "1st operand error")]
        [InlineData(0x23, "servo is OFF")]
        [InlineData(0x24, "during homing")]
        [InlineData(0x25, "software reset while the servo is ON")]
        [InlineData(0x26, "TEACH mode")]
        [InlineData(0x27, "invalid HOME")]
        [InlineData(0x3A, "monitor (PIO) mode")]
        public void RejectionNamesTheReasonAndIsNotRetried(int code, string reason)
        {
            var (bus, t) = T.Scripted(T.Response(1, $"45 {code:X2} 00"), T.Response(1, "45 00 00"));
            var exc = Assert.Throws<IaiCommandRejectedException>(() => bus.IaiCommand(1, 0, 0x0A, 7));
            Assert.Equal(code, exc.Rejection);
            Assert.Contains(reason, exc.Reason);
            Assert.Contains(reason, exc.Message);
            Assert.Single(t.Writes); // a rejection is an answer: never retried
        }

        [Fact]
        public void HomeOperandHintIsCommandSpecific()
        {
            var (bus, _) = T.Scripted(T.Response(1, "45 11 00"));
            var exc = Assert.Throws<IaiCommandRejectedException>(() => bus.IaiCommand(1, 0, Registers.IAI_CMD_HOME, 0));
            Assert.Contains("ORG (item 9.16)", exc.Hint);
            Assert.Equal(string.Empty, Registers.IaiRejectionHint(0x45, 0x11, Registers.IAI_CMD_SERVO));
            Assert.Equal("turn the servo on first", Registers.IaiRejectionHint(0x45, 0x23));
        }

        [Fact]
        public void Rejection2Decoded()
        {
            var (bus, t) = T.Scripted(T.Response(1, "48 10 07" + string.Concat(Enumerable.Repeat(" 00", 16))));
            var exc = Assert.Throws<IaiCommandRejectedException>(() => bus.IaiCommand2(1, 0x0030, new uint[] { 1 }));
            Assert.Equal(0x1007, exc.Rejection);
            Assert.Single(t.Writes);
            Assert.Contains("software reset while the servo is ON", exc.Message);
            Assert.Contains("0x1007", exc.Message);
            Assert.Equal(4, exc.StatusWords!.Count);
        }
    }

    /// <summary>Auto-baud detection in place.</summary>
    public class BaudDetectTests
    {
        [Fact]
        public void FindsRateInPlaceWithoutRetries()
        {
            var t = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 38400 };
            t.Enqueue(T.ReadResponse(1, 0x1CE3));
            var bus = new ModbusRtu(t, 2, TimeSpan.Zero);
            Assert.Equal(38400, bus.DetectBaudRate(1));
            Assert.Equal(38400, bus.BaudRate);
            Assert.Equal(new[] { 38400 }, t.BaudRates); // 115200 tried first (no answer, no retry), then 38400
            Assert.Equal(2, t.Writes.Count);
            Assert.Equal(2, bus.Retries);
            Assert.False(t.Closed, "the port must never be closed while detecting");
        }

        [Fact]
        public void ExceptionReplyCountsAsAnswer()
        {
            var t = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 115200 };
            t.Enqueue(T.Response(1, "83 02"));
            Assert.Equal(115200, new ModbusRtu(t, minGap: TimeSpan.Zero).DetectBaudRate(1));
        }

        [Fact]
        public void NoAnswerRestoresStartRate()
        {
            var t = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 1 };
            var bus = new ModbusRtu(t, minGap: TimeSpan.Zero);
            Assert.Throws<ModbusTimeoutException>(() => bus.DetectBaudRate(1, new[] { 38400, 9600 }));
            Assert.Equal(115200, t.BaudRate);
            Assert.Equal(3, t.Writes.Count);
        }

        [Fact]
        public void DetectOrderMatchesThePythonEdition()
        {
            var t = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 1 };
            var bus = new ModbusRtu(t, minGap: TimeSpan.Zero);
            Assert.Throws<ModbusTimeoutException>(() => bus.DetectBaudRate(1));
            Assert.Equal(new[] { 38400, 9600, 230400, 57600, 19200, 115200 }, t.BaudRates);
            Assert.Equal(new[] { 115200, 38400, 9600, 230400, 57600, 19200, 14400, 28800, 76800 }, ModbusRtu.BaudProbeAll);
            Assert.Equal(38400, ModbusRtu.BaudByCode[6]);
        }

        [Fact]
        public void SeveralSlavesFirstAnswerSettlesTheRate()
        {
            var rod = RodFixture.Unit(3);
            var t = new FakeTransport(FakeUnit.Line(rod), baudRate: 115200) { AnswerAtBaudRate = 230400 };
            var bus = new ModbusRtu(t, minGap: TimeSpan.Zero);
            Assert.Equal(230400, bus.DetectBaudRate(new[] { 1, 2, 3 }, probeTimeout: TimeSpan.FromMilliseconds(5)));
        }

        [Fact]
        public void TransportWithoutBaudRate()
        {
            var bus = new ModbusRtu(new FakeTransport());
            Assert.Null(bus.BaudRate);
            Assert.False(bus.CanChangeBaudRate);
            Assert.Throws<NotSupportedException>(() => bus.DetectBaudRate(1));
            Assert.Throws<NotSupportedException>(() => bus.SetBaudRate(38400));
        }
    }
}
