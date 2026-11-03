using System;
using System.IO;
using System.Linq;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests.Interop
{
    /// <summary>
    /// <see cref="SimulatedUnit"/> on the native core's simulator: the state and the test hooks through the core, the
    /// overrides of the protected members through the core's request hook, a clock of the unit's own, slave 0. No port
    /// opened.
    /// </summary>
    [Collection(SimCollection.Name)]
    public sealed class SimFacadeTests
    {
        /// <summary>The home frame is answered with rejection 0x11 and nothing runs (Fc45 through the hook).</summary>
        private sealed class HomeRefused : SimulatedUnit
        {
            public HomeRefused(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override byte[] Fc45(byte[] body) =>
                body.Length > 1 && body[1] == Registers.IAI_CMD_HOME ? new byte[] { 0x45, 0x11, 0 } : base.Fc45(body);
        }

        /// <summary>AKID = 1 (a rotary actuator) and FC11 refused: ReadWord and Fc11 through the hook.</summary>
        private sealed class RotaryNoId : SimulatedUnit
        {
            protected override int ReadWord(int address) => address == 0x072F ? 1 : base.ReadWord(address);

            protected override byte[] Fc11(byte[] body) => throw new SimException(0x01);
        }

        /// <summary>The answer to every second coil write is lost on the line; the coil still acts.</summary>
        private sealed class LossyCoils : SimulatedUnit
        {
            private int _coils;

            protected override void WriteFrame(byte[] data)
            {
                if (data.Length > 1 && data[1] == 0x05 && _coils++ % 2 == 1)
                {
                    InjectFault(SimFault.Timeout);
                }

                base.WriteFrame(data);
            }
        }

        [NativeFact]
        public void A_unit_runs_on_the_core_and_reads_its_state_there()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            Assert.NotNull(new SimulatedUnit(new SimulatedUnitOptions { Clock = () => 1000.0 }).CoreSim);  // a clock of its own: the core unit's clock
            var (axis, _, rec) = SimKit.MakeAxis(unit: sim);
            using (axis)
            {
                Assert.Equal(73.95, sim.PositionMm);
                axis.MoveTo(80.0, speed: 50.0);
                Assert.InRange(sim.PositionMm, 79.9, 80.1);
                Assert.False(sim.Moving);
                Assert.True(sim.ServoOn && sim.Homed && sim.Pmsl);
                Assert.Equal(0, sim.AlarmCode);
                Assert.Equal(rec.Tx.Count, sim.Requests.Count);
                Assert.Equal(rec.Tx.Last(), sim.Requests.Last());
                sim.ClearRequests();
                Assert.Empty(sim.Requests);
            }

            sim.TriggerAlarm(0x0D9);
            Assert.Equal(0x0D9, sim.AlarmCode);
            Assert.False(sim.ServoOn);
            sim.SetCoilState(Registers.COIL_STOP, true);
            Assert.True(sim.CoilState(Registers.COIL_STOP) && sim.StopHeld);
            Assert.Equal(16, sim.PositionRow(1).Length);
            sim.ObstacleMm = 85.0;
            Assert.Equal(85.0, sim.ObstacleMm);
            sim.PowerCycle();
            Assert.Equal(0, sim.AlarmCode);
            Assert.False(sim.Pmsl);
            Assert.Contains("power cycle", string.Join("; ", sim.Events));
        }

        [NativeFact]
        public void The_port_closes_and_the_line_is_modelled_in_the_core()
        {
            var sim = new SimulatedUnit(SimKit.Sim() with { BaudRate = 115200, EcBaudRate = 38400 });
            Assert.Equal(115200, sim.BaudRate);
            Assert.Equal(38400, sim.EcBaudRate);
            Assert.Empty(SimKit.Exchange(sim, "03 90 00 00 01"));  // the unit listens at another rate
            sim.BaudRate = 38400;
            Assert.Equal(7, SimKit.Exchange(sim, "03 90 00 00 01").Length);
            sim.SendBreak(TimeSpan.FromMilliseconds(200));
            Assert.Equal(9600, sim.EcBaudRate);

            var plain = new SimulatedUnit(SimKit.Sim());
            Assert.Throws<NotSupportedException>(() => plain.BaudRate = 9600);
            plain.Close();
            Assert.True(plain.Closed);
            Assert.Throws<IOException>(() => plain.Write(SimKit.Frame(1, "03 90 00 00 01")));
        }

        [NativeFact]
        public void Overrides_of_the_protected_members_run_in_the_core_hook()
        {
            var (axis, rec) = SimKit.Open(new HomeRefused(SimKit.Sim(SimProfile.Rod)));
            using (axis)
            {
                Assert.Equal(0x11, Assert.Throws<IaiCommandRejectedException>(() => axis.Home(method: CommandMethod.Iai)).Rejection);
                Assert.Single(rec.Tx, f => f.Length > 3 && f[1] == 0x45 && f[3] == Registers.IAI_CMD_HOME);
            }

            var rotary = new RotaryNoId();
            Assert.Equal(T.Hex("01 91 01"), SimKit.Exchange(rotary, "11").Take(3).ToArray());
            var akid = SimKit.Exchange(rotary, "03 07 2E 00 02");
            Assert.Equal(T.Hex("01 03 04 00 00 00 01"), akid.Take(7).ToArray());
            Assert.True(Crc16.Check(akid));
            Assert.Equal(2, rotary.Requests.Count);  // the core records every frame once
        }

        [NativeFact]
        public void A_fault_injected_inside_the_hook_loses_that_answer_only()
        {
            var sim = new LossyCoils();
            var on = SimKit.Exchange(sim, "05 04 01 FF 00");  // SFTY on: answered
            Assert.Equal(T.Hex("01 05 04 01 FF 00"), on.Take(6).ToArray());
            Assert.True(sim.SafetySpeed);
            Assert.Empty(SimKit.Exchange(sim, "05 04 01 00 00"));  // SFTY off: acted on, answer lost
            Assert.False(sim.SafetySpeed);
            Assert.Equal(7, SimKit.Exchange(sim, "03 90 00 00 01").Length);
        }

        [NativeFact]
        public void A_unit_with_a_clock_of_its_own_runs_on_the_core_on_that_clock()
        {
            var clock = new FakeClock();
            var sim = new SimulatedUnit(new SimulatedUnitOptions { PositionMm = 10.0, Clock = clock.Read });
            var bus = SimKit.Bus(sim);
            bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(60.0, 0.10, 100.0, 0.30));
            Assert.Equal(10.0, SimKit.Pnow(bus.ReadRegisters(1, 0x9000, 8)), 2); // no time passed: still at the start
            clock.Advance(0.2);
            var moved = SimKit.Pnow(bus.ReadRegisters(1, 0x9000, 8));
            Assert.InRange(moved, 10.5, 59.0); // on its way, on the fake clock's time
            clock.Advance(5.0);
            Assert.Equal(60.0, SimKit.Pnow(bus.ReadRegisters(1, 0x9000, 8)), 2);
            Assert.False(sim.Moving);
        }

        [NativeFact]
        public void The_state_hooks_are_the_cores_controls()
        {
            var sim = new SimulatedUnit(SimKit.Sim());
            sim.Homed = false;
            Assert.False(sim.Homed);
            Assert.Contains("\"homed\":false", sim.CoreSim.StateJson());
        }

        [NativeFact]
        public void Slave_0_acts_on_broadcasts_and_never_answers()
        {
            // as the Python edition's simulator: a unit with no address of its own (the core's, since ABI 0.7)
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Slave = 0 });
            Assert.Equal(0, sim.Slave);
            Assert.Contains("\"slave\":0", sim.CoreSim.StateJson());
            Assert.Empty(SimKit.Exchange(sim, "03 90 00 00 02", slave: 1)); // no unit at 1
            Assert.True(sim.ServoOn);
            Assert.Empty(SimKit.Exchange(sim, "05 04 03 00 00", slave: 0)); // a broadcast: no answer...
            Assert.False(sim.ServoOn); // ...but executed
            Assert.Throws<ArgumentOutOfRangeException>(() => new SimulatedUnit(new SimulatedUnitOptions { Slave = 248 }));
        }
    }
}
