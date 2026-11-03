using System;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// The ABI 0.6 bindings of the round-4 areas through the interop layer: the event source and the recovery fields
    /// carved from the 0.2 structs, the recovery session (py-recovery), the directory readers and the self-test lines
    /// (py-info-cli), the bus moved onto another transport (fixes), the simulator's ports and gateway lines
    /// (core-sim-gw), and the public classes' use of them (the cell's core, the class reader of the unit info, a
    /// simulator override that throws, the push blip read from the core). No port opened.
    /// </summary>
    public class Abi06BindingTests
    {
        private static NativeSimOptions Faster => new NativeSimOptions { TimeScale = 50 };

        private static (NativeSim Sim, TransportHandle Transport, NativeBus Bus, NativeAxis Axis) OnBus(string profile)
        {
            var sim = NativeSim.Open("{\"profile\": \"" + profile + "\", \"time_scale\": 50}");
            var transport = sim.Transport();
            var bus = NativeBus.Open(transport, retries: 0, minGapS: 0);
            var axis = NativeAxis.OpenOnBus(bus, 1);
            axis.Connect();
            return (sim, transport, bus, axis);
        }

        [Fact]
        public void The_06_fields_take_reserved_space_and_keep_the_sizes()
        {
            Assert.Equal(736, Marshal.SizeOf<IadEvent>());  // 0.8: the layout reset
            Assert.Equal((int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Message)) + 256, (int)Marshal.OffsetOf<IadEvent>(nameof(IadEvent.Source)));
            Assert.Equal(80, Marshal.SizeOf<IadRecoveryPolicy>());
            Assert.Equal((int)Marshal.OffsetOf<IadRecoveryPolicy>(nameof(IadRecoveryPolicy.CheckParameters)) + 4, (int)Marshal.OffsetOf<IadRecoveryPolicy>(nameof(IadRecoveryPolicy.RequireSession)));
            Assert.Equal(768, Marshal.SizeOf<IadRecoverySummary>());
            Assert.Equal((int)Marshal.OffsetOf<IadRecoverySummary>(nameof(IadRecoverySummary.NoteCount)) + 4, (int)Marshal.OffsetOf<IadRecoverySummary>(nameof(IadRecoverySummary.ControlKept)));
        }

        [NativeFact]
        public void A_session_from_the_identity_its_fields_and_the_link_state()
        {
            var (sim, transport, bus, axis) = OnBus("gripper");
            using (sim)
            using (transport)
            using (bus)
            using (axis)
            {
                Assert.Throws<EcNotReadyException>(() => axis.SessionUpdate("{\"recovering\": true}"));  // no session yet
                axis.SessionFromInfo(null, "G");  // the identity Connect read
                Assert.Contains("\"name\":\"G\"", axis.SessionJson());
                axis.SessionUpdate("{\"recovering\": true}");
                Assert.Equal("recovering", axis.CheckLink().State);
                axis.SessionUpdate("{\"recovering\": false}");
                Assert.Throws<ArgumentException>(() => axis.SessionUpdate("{\"colour\": 1}"));
                Assert.Equal("ok", axis.CheckLink().State);

                while (axis.NextEvent() != null)
                {
                    // drop what the checks queued
                }

                Assert.True(axis.SetLinkState("unit_silent", "no answer", "G"));
                Assert.False(axis.SetLinkState("unit_silent"));  // unchanged
                var lost = axis.NextEvent(1.0);
                Assert.NotNull(lost);
                Assert.Equal(NativeEventKind.Link, lost!.Kind);
                Assert.Equal("link_lost", lost.Name);
                Assert.Equal(NativeEventSource.SetLinkState, lost.Source);
                Assert.True(axis.SetLinkState("ok"));
                Assert.Equal("recovered", axis.NextEvent(1.0)?.Name);
                Assert.Throws<ArgumentException>(() => axis.SetLinkState("sleeping"));

                var words = bus.ReadRegisters(1, 0x9000, Registers.STATUS_BLOCK_WORDS);
                Assert.Empty(axis.RestartSigns(words));
                Assert.Contains("new power-up record (0x0FFF) in the alarm history", axis.RestartSigns(words, "{\"new_records\": [[4095, 0, 0, null]]}"));

                Assert.True(axis.PortPresent());
                sim.Unplug();
                Assert.False(axis.PortPresent());
                sim.Replug();

                axis.Forget();
                Assert.Throws<EcNotReadyException>(() => axis.RestartSigns(words));
                var e = Assert.Throws<RecoveryException>(() => axis.Recover(1.0, new NativeRecoveryPolicy { PollS = 0.02, RequireSession = true }));
                Assert.Contains("\"kind\":\"RecoveryError\"", NativeErrors.ErrorJsonOf(e));
            }
        }

        [NativeFact]
        public void A_recovery_reports_the_control_it_kept()
        {
            using var axis = NativeAxis.OpenSim("gripper", Faster);
            axis.Connect();
            axis.Track("G");
            var report = axis.Recover(10.0, new NativeRecoveryPolicy { PollS = 0.02 });
            Assert.True(report.SameUnit);
            Assert.Equal(report.Json.Contains("\"control_kept\":true"), report.ControlKept);
        }

        [NativeFact]
        public void The_directory_readers_by_name_and_the_self_test_lines()
        {
            using var axis = NativeAxis.OpenSim("slider", Faster);
            axis.Connect();
            var directory = (JsonObject)MiniJson.Parse(axis.InfoCallJson("read_directory"))!;
            var records = AxisRecords.ToRecords(directory.Array("records"));
            Assert.Contains(records, r => r.CategoryId == 0x0007);

            var block = AxisRecords.ToClassBlock((JsonObject)MiniJson.Parse(axis.InfoCallJson("read_class", "{\"class_id\": 7}"))!);
            Assert.Equal(7, block.ClassId);
            Assert.NotEmpty(block.Items);
            var given = AxisRecords.ToClassBlock((JsonObject)MiniJson.Parse(axis.InfoCallJson("read_class", "{\"class_id\": 7, \"records\": " + AxisRecords.RecordsJson(records) + "}"))!);
            Assert.Equal(block.Items.Count, given.Items.Count);

            Assert.ThrowsAny<ArgumentException>(() => axis.InfoCallJson("read_class", "{\"class_id\": 48879}"));  // not in the directory: nothing sent
            Assert.Throws<NotSupportedException>(() => axis.InfoCallJson("read_everything"));
            Assert.Equal("[]", axis.SelfTestItemsJson());  // no self-test runs
        }

        [NativeFact]
        public void A_bus_moves_onto_another_transport_in_place()
        {
            using var at10 = NativeSim.Open("{\"profile\": \"slider\", \"position_mm\": 10}");
            using var at20 = NativeSim.Open("{\"profile\": \"slider\", \"position_mm\": 20}");
            using var first = at10.Transport();
            using var second = at20.Transport();
            using var bus = NativeBus.Open(first, retries: 0, minGapS: 0);
            Assert.Equal(1000, bus.ReadRegisters(1, 0x9000, 2)[1]);
            bus.SetTransport(second);
            Assert.Equal(2000, bus.ReadRegisters(1, 0x9000, 2)[1]);
            bus.SetTransport(second);  // the same one again: nothing changes
            Assert.Equal(2000, bus.ReadRegisters(1, 0x9000, 2)[1]);
        }

        [NativeFact]
        public void A_gateway_line_answers_through_its_port_and_takes_its_controls()
        {
            using var one = NativeSim.Open("{\"profile\": \"pcon\", \"slave\": 1}");
            Assert.Throws<ArgumentException>(() => NativeSimLine.OpenGateway("canbus", null, new[] { one }));
            using var line = NativeSimLine.OpenGateway(null, "{\"name\": \"R\"}", new[] { one });
            using var port = line.Port();
            using var bus = NativeBus.Open(port, retries: 0, minGapS: 0);
            Assert.Equal(0x8000, bus.ReadRegisters(0xD0, 0x3700, 11)[0]);  // GWSR: MANU
            Assert.Equal(2, bus.ReadRegisters(1, 0x9000, 2).Length);

            var gateway = (JsonObject)MiniJson.Parse(line.GatewayJson())!;
            Assert.Equal("rcon", gateway.String("kind"));
            Assert.Equal(true, gateway.Object("gateway")?.Bool("manu"));
            line.SetGateway("{\"manu\": false, \"alarm_code\": 2234}");
            Assert.Equal(0, bus.ReadRegisters(0xD0, 0x3700, 1)[0]);
            Assert.Throws<ArgumentException>(() => line.SetGateway("{\"running\": true}"));  // an RCON gateway has no RUN flag

            var reply = line.GatewayHandle(Crc16.BuildFrame(0xD0, new byte[] { 0x03, 0x37, 0x00, 0x00, 0x01 }));
            Assert.NotNull(reply);
            Assert.Equal(0xD0, reply![0]);

            using var plain = NativeSimLine.Open("P");
            Assert.Throws<ArgumentException>(() => plain.GatewayJson());
        }

        [NativeFact]
        public void A_unit_on_its_port_holds_the_bus_as_its_transport_does()
        {
            using var sim = NativeSim.Open("{\"profile\": \"slider\", \"position_mm\": 10}");
            using var port = sim.Port();
            using var bus = NativeBus.Open(port, retries: 0, minGapS: 0);
            Assert.Equal(1000, bus.ReadRegisters(1, 0x9000, 2)[1]);
        }

        [NativeFact]
        public void A_cell_owns_its_core_through_the_interop_cell()
        {
            using var cell = Cell.Open(SimLine.SimPorts(2, options: new SimulatedUnitOptions { TimeScale = 50.0 }));
            Assert.NotNull(cell.Core);
            Assert.Equal(cell.Names, cell.Core!.Names);
            Assert.Equal("null", cell.Core.LinkMonitorJson());
            cell.Core.MonitorLinks(0.05, raiseEvents: false);
            var clock = Stopwatch.StartNew();
            while (!cell.Core.LinkMonitorJson().Contains("\"running\":true") && clock.Elapsed < TimeSpan.FromSeconds(20))
            {
                Thread.Sleep(20);
            }

            Assert.Contains("\"running\":true", cell.Core.LinkMonitorJson());
            cell.Core.StopLinkMonitor();
            Assert.Equal("null", cell.Core.LinkMonitorJson());
            cell.Close();
            Assert.True(cell.Core.IsClosed);
        }

        [NativeFact]
        public void An_override_that_throws_fails_the_write_with_its_exception()
        {
            var sim = new ThrowingUnit();
            using var axis = Axis.Open(sim, options: new AxisOptions { PollIntervalS = 0.005 });
            sim.Throw = true;
            Assert.Throws<NotImplementedException>(() => axis.Status());  // the override's own exception
            sim.Throw = false;
            axis.Status();
        }

        [NativeFact]
        public void The_push_blip_is_the_cores()
        {
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Profile = SimProfile.Gripper, PushBlipMm = 6.76 });
            Assert.Equal(6.76, sim.PushBlipMm);
            Assert.Equal(6.76, ((JsonObject)MiniJson.Parse(sim.CoreSim.StateJson())!).Number("push_blip_mm"));
            sim.PushBlipMm = null;
            Assert.Null(sim.PushBlipMm);
            Assert.Null(((JsonObject)MiniJson.Parse(sim.CoreSim.StateJson())!).Number("push_blip_mm"));
        }

        /// <summary>A unit whose frame handling throws on request.</summary>
        private sealed class ThrowingUnit : SimulatedUnit
        {
            public ThrowingUnit()
                : base(new SimulatedUnitOptions { TimeScale = 50.0 })
            {
            }

            public volatile bool Throw;

            protected override void WriteFrame(byte[] data)
            {
                if (Throw)
                {
                    throw new NotImplementedException("no frames today");
                }

                base.WriteFrame(data);
            }
        }
    }
}
