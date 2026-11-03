using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 0.3 additions through the interop layer: the binding's own transport through callbacks, buses, the frame
    /// sink, simulated units and lines with their hooks, axes and cells opened on transports and buses, the cancel latch.
    /// No port opened.
    /// </summary>
    public class Abi03Tests
    {
        [Fact]
        public void The_03_structs_have_the_sizes_the_core_freezes()
        {
            Assert.Equal(56, Marshal.SizeOf<IadBusOptions>());
            Assert.Equal(24, (int)Marshal.OffsetOf<IadTransportCallbacks>(nameof(IadTransportCallbacks.Write)));
            Assert.Equal(24 + (9 * IntPtr.Size), Marshal.SizeOf<IadTransportCallbacks>());
        }

        [NativeFact]
        public void A_managed_transport_runs_the_core_bus_through_callbacks()
        {
            var fake = new FakeTransport();
            fake.Enqueue(T.Hex("01 03 04 00 00 0B FE 7C 83"));
            var frames = new System.Collections.Generic.List<(FrameDirection, byte[])>();
            using (var transport = TransportBridge.Open(fake, "FAKE", forwardClose: true))
            using (var bus = NativeBus.Open(transport, retries: 0, minGapS: 0))
            {
                bus.SetFrameSink((d, f) => frames.Add((d, f)));
                Assert.Equal(new ushort[] { 0x0000, 0x0BFE }, bus.ReadRegisters(1, 0x9000, 2));
                Assert.Contains("\"name\":\"FAKE\"", bus.InfoJson());
            }

            Assert.Equal(T.Hex("01 03 90 00 00 02 E9 0B"), Assert.Single(fake.Writes));
            Assert.Equal(new[] { FrameDirection.Tx, FrameDirection.Rx }, frames.Select(f => f.Item1));
            Assert.True(fake.Closed, "closing the bus closes a bridged transport that forwards its close");
        }

        [NativeFact]
        public void A_bridge_that_does_not_forward_close_leaves_the_transport_open()
        {
            var fake = new FakeTransport();
            using (var transport = TransportBridge.Open(fake, null, forwardClose: false))
            using (NativeBus.Open(transport))
            {
            }

            Assert.False(fake.Closed);
        }

        [NativeFact]
        public void What_the_transport_throws_comes_back_as_the_managed_edition_raised_it()
        {
            var fake = new FakeTransport();
            fake.Close();  // the fake throws IOException on write once closed
            using var transport = TransportBridge.Open(fake, null, forwardClose: false);
            using var bus = NativeBus.Open(transport, retries: 2);
            var e = Assert.Throws<ModbusException>(() => bus.ReadRegisters(1, 0x9000, 1));
            Assert.Contains("transport error", e.Message);
            Assert.IsType<IOException>(e.InnerException);
            Assert.Equal(NativeResult.Transport, NativeErrors.ResultOf(e));
            Assert.Empty(fake.Writes);  // a transport error is never retried

            var odd = new FakeTransport(_ => throw new FormatException("responder bug"));
            using var t2 = TransportBridge.Open(odd, null, forwardClose: false);
            using var b2 = NativeBus.Open(t2);
            Assert.Equal("responder bug", Assert.Throws<FormatException>(() => b2.ReadRegisters(1, 0x9000, 1)).Message);
        }

        [NativeFact]
        public void A_simulated_unit_with_its_hooks()
        {
            using var sim = NativeSim.Open("{\"profile\": \"gripper\", \"slave\": 2}");
            Assert.Contains("\"gripper\"", sim.StateJson());
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            Assert.True(bus.Probe(2, 0.05, 0x9000));
            Assert.False(bus.Probe(3, 0.05, 0x9000));
            Assert.Contains("\"frame\"", sim.RequestsJson());

            sim.SetWordOverride(0x9000, 0x1234);
            Assert.Equal(0x1234, sim.RegisterValue(0x9000));
            Assert.Equal(new ushort[] { 0x1234 }, bus.ReadRegisters(2, 0x9000, 1));
            sim.SetWordOverride(0x9000, null);

            sim.SetUnreadable(0x9010, 4);
            Assert.Equal(2, Assert.Throws<ModbusSlaveException>(() => bus.ReadRegisters(2, 0x9010, 1)).Code);
            Assert.Throws<ArgumentOutOfRangeException>(() => sim.RegisterValue(0x9011));  // kind std::out_of_range
            sim.SetUnreadable(0x9010, 4, unreadable: false);
            bus.ReadRegisters(2, 0x9010, 1);

            Assert.Equal("{}", sim.Control("{\"command\": \"clear_requests\"}").Trim());
            Assert.StartsWith("[", sim.EventsJson());
            Assert.Throws<ArgumentException>(() => sim.Control("{\"command\": \"no-such-command\"}"));
        }

        [NativeFact]
        public void A_request_hook_drops_replaces_or_refuses_frames()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\"}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);

            sim.SetRequestHook(_ => NativeSimAnswer.Drop);
            Assert.False(bus.Probe(1, 0.05, 0x9000));

            sim.SetRequestHook(_ => new NativeSimAnswer(NativeSimAction.Exception, exceptionCode: 4));
            Assert.Equal(4, Assert.Throws<ModbusSlaveException>(() => bus.ReadRegisters(1, 0x9000, 1)).Code);

            sim.SetRequestHook(request => new NativeSimAnswer(NativeSimAction.Reply, Crc16.BuildFrame(1, T.Hex("03 02 AB CD"))));
            Assert.Equal(new ushort[] { 0xABCD }, bus.ReadRegisters(1, 0x9000, 1));

            sim.SetRequestHook(request => new NativeSimAnswer(NativeSimAction.Reply, sim.HandleFrame(request)));  // the unit's own answer
            bus.ReadRegisters(1, 0x9000, 1);

            sim.SetRequestHook(_ => throw new InvalidOperationException("hook bug"));  // counts as pass
            bus.ReadRegisters(1, 0x9000, 1);

            sim.SetRequestHook(null);
            Assert.True(bus.Probe(1, 0.05, 0x9000));
        }

        [NativeFact]
        public void A_simulated_line_carries_several_units()
        {
            using var one = NativeSim.Open("{\"profile\": \"slider\", \"slave\": 1}");
            using var two = NativeSim.Open("{\"profile\": \"rod\", \"slave\": 2}");
            using var line = NativeSimLine.Open("LINE", new[] { one });
            line.Add(two);
            Assert.Throws<ArgumentException>(() => line.Add(two));  // the address is taken
            using var transport = line.Transport();
            using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            Assert.True(bus.Probe(1, 0.05, 0x9000));
            Assert.True(bus.Probe(2, 0.05, 0x9000));
            Assert.Contains("\"slave\"", line.RequestsJson());
            Assert.Contains("\"LINE\"", line.Control("{\"command\": \"state\"}"));

            line.Remove(2);
            Assert.False(bus.Probe(2, 0.05, 0x9000));
            Assert.Throws<ArgumentOutOfRangeException>(() => line.Remove(2));
        }

        [NativeFact]
        public void Axes_on_a_bus_and_on_a_transport()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\", \"time_scale\": 20}");
            using var transport = sim.Transport();
            using var bus = NativeBus.Open(transport);
            using (var axis = NativeAxis.OpenOnBus(bus, 1))
            {
                Assert.Equal(1, axis.Connect().Slave);
                Assert.True(axis.Status().ServoOn);
                using var shared = axis.Bus();
                Assert.Contains("\"closed\":false", shared.InfoJson().Replace(" ", string.Empty));
            }

            Assert.True(bus.Probe(1, 0.05, 0x9000));  // closing the axis leaves the bus open

            using var other = NativeSim.Open("{\"profile\": \"gripper\"}");
            using var t2 = other.Transport();
            using var owner = NativeAxis.OpenOnTransport(t2);
            Assert.Equal(1, owner.Connect().Slave);
        }

        [NativeFact]
        public void A_cell_on_transports_and_the_bus_of_a_port()
        {
            using var one = NativeSim.Open("{\"profile\": \"slider\"}");
            using var two = NativeSim.Open("{\"profile\": \"gripper\"}");
            using var t1 = one.Transport();
            using var t2 = two.Transport();
            using var cell = NativeCell.OpenOnTransports(new[] { t1, t2 }, new[] { "P1", "P2" }, new[] { 1 });
            Assert.Equal(new[] { "P1:1", "P2:1" }, cell.Names.OrderBy(n => n, StringComparer.Ordinal));
            using var bus = cell.Bus("P2");
            Assert.True(bus.Probe(1, 0.05, 0x9000));
            Assert.Throws<ArgumentOutOfRangeException>(() => cell.Bus("P9"));

            cell.CancelLatch(new[] { "P1:1" });
            Assert.True(cell["P1:1"].CancelLatched);
            Assert.False(cell["P2:1"].CancelLatched);
            cell.CancelReset();
            Assert.False(cell["P1:1"].CancelLatched);
        }

        [NativeFact]
        public void The_cancel_latch_refuses_calls_until_it_is_reset()
        {
            using var axis = NativeAxis.OpenSim("slider", new NativeSimOptions { TimeScale = 20 });
            axis.Connect();
            axis.Home();
            Assert.False(axis.CancelLatched);
            axis.CancelLatch();
            Assert.True(axis.CancelLatched);
            Assert.Throws<OperationCanceledException>(() => axis.MoveTo(20.0, speed: 100.0));
            Assert.True(axis.Status().ServoOn);  // reads are not refused
            axis.CancelReset();
            Assert.InRange(axis.MoveTo(20.0, speed: 100.0)!.PositionMm, 19.9, 20.1);
        }

        [Fact]
        public void An_out_of_range_kind_is_an_ArgumentOutOfRangeException()
        {
            var json = "{\"code\": -10, \"kind\": \"std::out_of_range\", \"message\": \"no axis 'X' in this cell\"}";
            var e = NativeErrors.Create((int)NativeResult.InvalidArgument, "no axis 'X' in this cell", json);
            Assert.IsType<ArgumentOutOfRangeException>(e);
            Assert.Contains("no axis 'X' in this cell", e.Message);
            Assert.IsType<ArgumentException>(NativeErrors.Create((int)NativeResult.InvalidArgument, "bad", "{\"kind\": \"std::invalid_argument\"}"));
        }
    }

    /// <summary>The clock seam is process-wide: these run alone, after the parallel tests.</summary>
    [Collection(SimCollection.Name)]
    public class Abi03ClockTests
    {
        [NativeFact]
        public void A_virtual_clock_moves_only_when_advanced_or_slept_on()
        {
            try
            {
                NativeClock.UseVirtual(1000.0);
                Assert.Equal(1000.0, NativeClock.Now());
                NativeClock.Advance(2.5);
                Assert.Equal(1002.5, NativeClock.Now());

                var now = 50.0;
                NativeClock.UseCallbacks(() => now, s => now += s);
                Assert.Equal(50.0, NativeClock.Now());
                using var sim = NativeSim.Open("{\"profile\": \"slider\"}");
                using var transport = sim.Transport();
                using var bus = NativeBus.Open(transport, retries: 0, minGapS: 0.5);
                bus.ReadRegisters(1, 0x9000, 1);
                bus.ReadRegisters(1, 0x9000, 1);  // the 0.5 s gap is slept on the callback clock
                Assert.True(now >= 50.5, $"the clock callback was not slept on ({now})");
            }
            finally
            {
                NativeClock.UseReal();
            }

            Assert.Throws<EcNotReadyException>(() => NativeClock.Advance(1.0));
        }
    }
}
