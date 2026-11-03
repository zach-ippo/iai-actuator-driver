using System;
using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The rest of the ABI 0.5 bindings (core-bus, py-monitor, py-gateway, py-cell) and the 0.6 csharp block through the
    /// interop layer, and what the transport bridge and the bus facade do at their edges: the read timeout a managed
    /// transport has when the core reads, an empty candidate list, a closed bus. No port opened.
    /// </summary>
    public class Abi05BindingTests
    {
        [Fact]
        public void The_05_callbacks_struct_appends_set_read_timeout()
        {
            Assert.Equal(24 + (10 * IntPtr.Size), System.Runtime.InteropServices.Marshal.SizeOf<IadTransportCallbacks05>());
        }

        [NativeFact]
        public void A_read_waits_the_transports_own_timeout_as_it_is_now()
        {
            var transport = new TimeoutRecorder { ReadTimeout = TimeSpan.FromSeconds(0.2) };
            using var bus = new ModbusRtu(transport, retries: 0, minGap: TimeSpan.Zero);
            transport.ReadTimeout = TimeSpan.FromSeconds(1.0);  // raised after the bus was made: a slow unit
            Assert.Throws<ModbusTimeoutException>(() => bus.ReadRegisters(1, 0x9000, 1));
            Assert.All(transport.SeenAtRead, t => Assert.Equal(TimeSpan.FromSeconds(1.0), t));
            Assert.NotEmpty(transport.SeenAtRead);

            transport.SeenAtRead.Clear();
            Assert.False(bus.Probe(1, TimeSpan.FromMilliseconds(20)));  // a probe shortens it for its own request only
            Assert.All(transport.SeenAtRead, t => Assert.Equal(TimeSpan.FromMilliseconds(20), t));
            Assert.Equal(TimeSpan.FromSeconds(1.0), transport.ReadTimeout);
        }

        [NativeFact]
        public void An_empty_candidate_list_asks_the_present_rate_only()
        {
            var transport = new FakeTransport(baudRate: 115200) { AnswerAtBaudRate = 38400 };
            using var bus = new ModbusRtu(transport, minGap: TimeSpan.Zero);
            Assert.Throws<ModbusTimeoutException>(() => bus.DetectBaudRate(1, Array.Empty<int>()));
            Assert.DoesNotContain(38400, transport.BaudRates);
            Assert.Equal(115200, bus.BaudRate);
        }

        [NativeFact]
        public void A_baud_change_on_a_closed_bus_says_the_bus_is_closed()
        {
            var bus = new ModbusRtu(new FakeTransport(baudRate: 115200), minGap: TimeSpan.Zero);
            bus.Close();
            var exc = Assert.Throws<ModbusException>(() => bus.SetBaudRate(38400));
            Assert.Contains("bus is closed", exc.Message);
        }

        [NativeFact]
        public void A_port_is_the_cores_with_its_line_stated_direct()
        {
            var transport = new FakeTransport(FakeUnit.Line(RodFixture.Unit(1)));
            using var port = new Port(transport, "P", minGap: TimeSpan.Zero);
            var info = port.Bus.Native.PortInfo();
            Assert.Equal("P", info.String("name"));
            Assert.Equal("direct", info.String("line"));
            Assert.Equal(new[] { 1 }, NativeBusLines.Scan(port.Bus.Native, new[] { 1, 2 }, 0.01));
            Assert.Contains("\"system_code\":236", NativeBusLines.IdentifyJson(port.Bus.Native, 1));
            Assert.Equal(NativeLineKind.Direct, NativeBusLines.DetectPortLine(port.Bus.Native));
            Assert.Equal(string.Empty, NativeBusLines.RobonetHint(port.Bus.Native, 1, false));
        }

        [NativeFact]
        public void Lines_and_gateways_through_their_handles()
        {
            Assert.Contains("rcon", NativeLine.RulesJson());
            var transport = new FakeTransport();
            using var bus = new ModbusRtu(transport, retries: 0, minGap: TimeSpan.Zero);
            using (var direct = NativeLine.Of(bus.Native))
            {
                Assert.Contains("\"kind\":\"direct\"", direct.Json());
                Assert.True(direct.Allows("broadcast"));
                Assert.Null(direct.Gateway());
            }

            using var gateway = NativeGateway.Open(bus.Native, NativeLineKind.Rcon);
            Assert.Contains("\"address\"", gateway.InfoJson());
            gateway.SetCacheS(0.5);
            using var rcon = NativeLine.Open(NativeLineKind.Rcon, gateway);
            rcon.SetPositionUnit(0.01);
            Assert.Equal(0.01, rcon.PositionUnit()!.Value.UnitMm, 6);
            Assert.False(rcon.Allows("broadcast"));
            Assert.Throws<UnsafeCommandException>(() => rcon.Require("broadcast"));
            rcon.NoteUnreachable(new[] { 5 });
            using (var gw = rcon.Gateway())
            {
                Assert.NotNull(gw);
            }

            rcon.AttachTo(bus.Native);
            using var attached = NativeLine.Of(bus.Native);
            Assert.Contains("\"kind\":\"rcon\"", attached.Json());
            Assert.Null(NativeBusLines.ProbeGateway(bus.Native, NativeLineKind.Rcon, timeoutS: 0.01));  // nothing answers
        }

        [NativeFact]
        public void The_monitor_target_and_the_port_owner()
        {
            var unit = new SimUnit("rod", 1).Ready(0.0);
            var transport = new FakeTransport(unit.Respond);
            unit.Transport = transport;
            using var axis = Axis.Open(transport, 1, SimUnit.Fast, autoBaud: false);
            Assert.Null(axis.Core.MonitorTarget());
            axis.Core.SetPortOwner(true);
            Assert.Throws<EcNotReadyException>(() => axis.Core.SuspendMonitor(0.1));  // no monitor to hold
        }

        /// <summary>A transport that answers nothing and notes its read timeout at every read.</summary>
        private sealed class TimeoutRecorder : ITransport
        {
            public List<TimeSpan> SeenAtRead { get; } = new List<TimeSpan>();

            public int BaudRate { get; set; }

            public TimeSpan ReadTimeout { get; set; }

            public void Write(byte[] buffer, int offset, int count)
            {
            }

            public int Read(byte[] buffer, int offset, int count)
            {
                SeenAtRead.Add(ReadTimeout);
                return 0;
            }

            public void ResetInputBuffer()
            {
            }

            public void SendBreak(TimeSpan duration) => throw new NotSupportedException();

            public void Close()
            {
            }

            public void Dispose()
            {
            }
        }
    }
}
