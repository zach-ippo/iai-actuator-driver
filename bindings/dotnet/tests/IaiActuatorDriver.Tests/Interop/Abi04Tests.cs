using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 0.4 bus additions through the interop layer: the bus lock, the IAI opt-ins as flags, 0x48, broadcasts, the
    /// line of a bus and the response timing. On the core's simulated unit and on a managed transport; no port opened.
    /// </summary>
    public class Abi04Tests
    {
        [NativeFact]
        public void The_bus_lock_is_held_by_the_calling_thread_and_given_back()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\"}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            Assert.True(bus.Lock());
            Assert.True(bus.Lock(0.0));  // recursive
            bus.ReadRegisters(1, 0x9000, 1);  // this thread's own requests go through
            var otherGotIt = true;
            var other = new System.Threading.Thread(() =>
            {
                otherGotIt = bus.Lock(0.0);
                if (otherGotIt)
                {
                    bus.Unlock();
                }
            });
            other.Start();
            other.Join();
            Assert.False(otherGotIt);  // another thread does not get it
            bus.Unlock();
            bus.Unlock();
            Assert.Throws<ArgumentException>(() => bus.Unlock());  // none held
        }

        [NativeFact]
        public void Iai_commands_take_their_opt_ins_as_flags()
        {
            var fake = new FakeTransport();
            fake.Enqueue(T.Response(1, "45 00 00"), T.Response(1, "45 21 07"), T.Response(1, "48 00 00" + " 00 00 00 01" + string.Concat(Enumerable.Repeat(" 00", 12))));
            using var transport = TransportBridge.Open(fake, null, forwardClose: false);
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);

            Assert.Throws<UnsafeCommandException>(() => bus.IaiCommand(1, 0, 0x0F, 0, 0, NativeMethods.IaiAllowUnsafe, null));  // never-send
            Assert.Empty(fake.Writes);
            Assert.Equal((0, 0), bus.IaiCommand(1, 0, 0x0F, 0, 0, NativeMethods.IaiAllowUnsafe | NativeMethods.IaiAllowNeverSend, null));
            Assert.Equal((0x21, 0x07), bus.IaiCommand(1, 0, 0x0A, 0, 0, NativeMethods.IaiNoCheck, null));

            var (rejection, status) = bus.IaiCommand2(1, 0, 0x30, new uint[] { 1 }, 0, null);
            Assert.Equal(0, rejection);
            Assert.Equal(new uint[] { 1, 0, 0, 0 }, status);
            Assert.Equal(T.Hex("01 48 00 00 30 00 00 00 01"), fake.Writes[2].Take(9).ToArray());
        }

        [NativeFact]
        public void Broadcasts_return_the_frame_sent()
        {
            var fake = new FakeTransport();
            using var transport = TransportBridge.Open(fake, null, forwardClose: false);
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            var frame = bus.BroadcastIaiCommand(0, Registers.IAI_CMD_BAUD, 0x14, Registers.IAI_BAUD_OP2, NativeMethods.IaiAllowUnsafe | NativeMethods.IaiAllowNeverSend, 0.0);
            Assert.Equal(T.Hex("00 45 00 04 00 14 00 05 25 08"), frame);
            Assert.Throws<UnsafeCommandException>(() => bus.BroadcastIaiCommand(0, Registers.IAI_CMD_BAUD, 0x14, Registers.IAI_BAUD_OP2, NativeMethods.IaiAllowUnsafe, 0.0));
            Assert.Equal(T.Hex("00 05 04 2C FF 00 4D 12"), bus.Broadcast(T.Hex("05 04 2C FF 00"), 0.0));
            Assert.Throws<ArgumentException>(() => bus.Broadcast(T.Hex("03 90 00 00 01"), 0.0));  // never a read
            Assert.Equal(2, fake.Writes.Count);
        }

        [NativeFact]
        public void The_line_and_the_response_timing_of_a_bus()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\"}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            Assert.Equal(NativeLineKind.Direct, bus.DetectLine(NativeLineKind.Direct));
            Assert.StartsWith("{", bus.LineJson());
            bus.SetResponseDelay(1, 20.0);
            Assert.Equal(20.0, bus.ResponseDelay(1));
            Assert.True(bus.ResponseTimeout(1, T.Hex("03 90 00 00 14"), 45) >= 0.2);
            Assert.Equal(0, bus.SuspendMonitors());
            bus.ResumeMonitors();
        }

        [NativeFact]
        public void Reads_take_their_per_call_options()
        {
            var fake = new FakeTransport();
            fake.Enqueue(T.ReadResponse(1, 1, 2), T.ReadResponse(1, 3, 4), T.Response(1, "11 05 EC FF 01 02 03"));
            using var transport = TransportBridge.Open(fake, null, forwardClose: false);
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);

            Assert.Equal(new ushort[] { 1, 2, 3, 4 }, bus.ReadRegisters(1, 0x9000, 4, null, 0.5, 2));  // two frames of two
            Assert.Equal(T.Hex("01 03 90 00 00 02"), fake.Writes[0].Take(6).ToArray());
            Assert.Equal(T.Hex("01 03 90 02 00 02"), fake.Writes[1].Take(6).ToArray());
            Assert.Equal(T.Hex("EC FF 01 02 03"), bus.ReportSlaveId(1, 0, 0.1));
            Assert.ThrowsAny<ArgumentException>(() => bus.ReadRegisters(1, 0x9000, 4, null, null, 126));  // 1..125 per frame
        }

        [NativeFact]
        public void Baud_detection_takes_targets_a_probe_address_and_the_order()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\", \"baud_rate\": 115200, \"unit_baud_rate\": 38400}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            Assert.Equal(38400, bus.DetectBaudRate(new[] { 2, 1 }, new[] { 9600, 38400 }, 0x0300, false, 0.05));
            var probes = (MiniJson.Parse(sim.RequestsJson()) as List<object?>)!.OfType<JsonObject>().Select(r => r.String("frame")!).ToList();
            Assert.Equal(new[] { "02", "01", "02", "01" }, probes.Select(f => f.Substring(0, 2)));  // 9600 first, then 38400
            Assert.All(probes, f => Assert.Equal("03 03 00", f.Substring(3, 8), ignoreCase: true));
            Assert.Equal(38400, (MiniJson.Parse(sim.StateJson()) as JsonObject)!.Int("baud_rate"));
        }

        [NativeFact]
        public void The_stateless_functions_answer_by_name()
        {
            Assert.True(NativeCalls.Has("alarms.alarm_name"));
            Assert.False(NativeCalls.Has("no.such_function"));
            Assert.Equal("\"" + Alarms.AlarmName(0x0E8) + "\"", NativeCalls.Call("alarms.alarm_name", "{\"code\": 232}"));
            var levels = (List<object?>)NativeCalls.CallParsed("alarms.alarm_level", "[{\"code\": 78}, {\"code\": 128}]")!;
            Assert.Equal(new[] { Alarms.LevelKey(Alarms.AlarmLevelOf(0x04E)), Alarms.LevelKey(Alarms.AlarmLevelOf(0x080)) }, levels.Cast<string>());
            Assert.Throws<NotSupportedException>(() => NativeCalls.Call("no.such_function"));
            Assert.Throws<ArgumentException>(() => NativeCalls.Call("alarms.alarm_name", "{}"));
        }

        [NativeFact]
        public void The_alarm_tables_match_the_core()
        {
            var codes = Enumerable.Range(0, 0x1000).ToList();
            var args = "[" + string.Join(", ", codes.Select(c => "{\"code\": " + c.ToString(CultureInfo.InvariantCulture) + "}")) + "]";
            var names = ((List<object?>)NativeCalls.CallParsed("alarms.alarm_name", args)!).Cast<string>().ToList();
            var levels = ((List<object?>)NativeCalls.CallParsed("alarms.alarm_level", args)!).Cast<string>().ToList();
            Assert.Equal(codes.Select(Alarms.AlarmName), names);
            Assert.Equal(codes.Select(c => Alarms.LevelKey(Alarms.AlarmLevelOf(c))), levels);
        }
    }
}
