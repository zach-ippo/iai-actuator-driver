using System.Collections.Generic;
using System.Linq;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>
    /// Guards of the axis class: the 0x45 home after a lost or refused reply, the tri-state safety speed of the action mode,
    /// mm parameters on a rotary unit (in deg) and position-table writes on any unit code. Simulator only.
    /// </summary>
    [Collection(SimCollection.Name)]
    public class SimEcGuardTests
    {
        private static readonly byte[] HomeFrame = T.Hex("01 45 00 0A 00 07 00 00 BC C3"); // 0x45 home, op1 = ORG 7 (VERIFIED on EC #3)

        private static bool IsHomeFrame(byte[] data) => data.Length > 3 && data[1] == 0x45 && data[3] == Registers.IAI_CMD_HOME;

        private static List<byte[]> HomeFrames(FrameRecorder rec) => SimKit.Sent(rec, 0x45).Where(f => f[3] == Registers.IAI_CMD_HOME).ToList();

        private static List<bool> StopCoil(FrameRecorder rec) => SimKit.CoilWrites(rec.Tx, Registers.COIL_STOP);

        private static SimulatedUnitOptions Rod(double timeScale) => new SimulatedUnitOptions { Profile = SimProfile.Rod, TimeScale = timeScale };

        /// <summary>The unit runs the home frame, but its answer is lost on the line.</summary>
        private sealed class HomeLostReplySim : SimulatedUnit
        {
            public HomeLostReplySim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override void WriteFrame(byte[] data)
            {
                if (IsHomeFrame(data))
                {
                    InjectFault(SimFault.Timeout);
                }

                base.WriteFrame(data);
            }
        }

        /// <summary>The home frame never reaches the unit (nothing runs, nothing is answered).</summary>
        private sealed class HomeFrameLostSim : SimulatedUnit
        {
            public HomeFrameLostSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override void WriteFrame(byte[] data)
            {
                if (IsHomeFrame(data))
                {
                    RecordRequest(data);
                    return;
                }

                base.WriteFrame(data);
            }
        }

        /// <summary>The home frame starts a home return, but the answer is rejection 0x24 (as a repeated frame would get).</summary>
        private sealed class HomeAlreadyRunningSim : SimulatedUnit
        {
            public HomeAlreadyRunningSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override byte[] Fc45(byte[] body)
            {
                if (body.Length > 1 && body[1] == Registers.IAI_CMD_HOME)
                {
                    base.Fc45(body);
                    return new byte[] { 0x45, 0x24, 0 };
                }

                return base.Fc45(body);
            }
        }

        /// <summary>The home frame is answered with rejection 0x11 and nothing runs.</summary>
        private sealed class HomeRefusedSim : SimulatedUnit
        {
            public HomeRefusedSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override byte[] Fc45(byte[] body) =>
                body.Length > 1 && body[1] == Registers.IAI_CMD_HOME ? new byte[] { 0x45, 0x11, 0 } : base.Fc45(body);
        }

        /// <summary>AKID (item 7.33, 0x072E-0x072F on the slider) = 1: a rotary actuator.</summary>
        private sealed class RotarySim : SimulatedUnit
        {
            public RotarySim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override int ReadWord(int address) => address == 0x072F ? 1 : base.ReadWord(address);
        }

        /// <summary>FC11 too slow and CTCL (class 3 item 1, 0x0300) blank: the unit code cannot be read.</summary>
        private sealed class NoIdentitySim : SimulatedUnit
        {
            public NoIdentitySim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            protected override int ReadWord(int address) => address == 0x0300 || address == 0x0301 ? 0 : base.ReadWord(address);
        }

        // ------------------------------------------------------------------ 0x45 home: lost / refused replies

        [Fact]
        public void Lost_reply_while_the_home_runs_is_waited_for()
        {
            var (axis, rec) = SimKit.Open(new HomeLostReplySim(Rod(50.0)));
            Status st;
            List<string> logs;
            using (axis)
            {
                axis.MoveTo(30.0, speed: 50.0);
                rec.Clear();
                using (var capture = new LogCapture())
                {
                    st = axis.Home(method: CommandMethod.Iai)!;
                    logs = capture.Messages.ToList();
                }
            }

            Assert.Contains(logs, line => line.Contains("seen running"));
            Assert.True(st.Homed && st.PositionMm == 0.0);
            Assert.Equal(new[] { HomeFrame }, HomeFrames(rec)); // sent once: no retry that would be rejected 0x24
            Assert.Empty(StopCoil(rec)); // the home was seen running: it is this call's, nothing stopped
        }

        [Fact]
        public void Lost_reply_without_a_home_stops_and_raises()
        {
            var (axis, rec) = SimKit.Open(new HomeFrameLostSim(Rod(50.0)));
            using (axis)
            {
                axis.MoveTo(30.0, speed: 50.0);
                rec.Clear();
                ModbusTimeoutException caught;
                using (new LogCapture())
                {
                    caught = Assert.Throws<ModbusTimeoutException>(() => axis.Home(method: CommandMethod.Iai));
                }

                Assert.Single(HomeFrames(rec));
                Assert.Equal(new[] { true, false }, StopCoil(rec)); // on doubt: decel stop (the frame may have run)
                var notes = string.Join(" ", caught.Notes);
                Assert.Contains("no home return was seen", notes);
                Assert.Contains("decel stop sent", notes);
                Assert.Equal(30.0, axis.Status().PositionMm);
            }
        }

        [Fact]
        public void Rejection_0x24_stops_the_running_home()
        {
            var sim = new HomeAlreadyRunningSim(Rod(2.0));
            var (axis, rec) = SimKit.Open(sim);
            using (axis)
            {
                axis.MoveTo(30.0, speed: 50.0);
                rec.Clear();
                sim.ClearEvents();
                var caught = Assert.Throws<IaiCommandRejectedException>(() => axis.Home(method: CommandMethod.Iai));
                Assert.Equal(0x24, caught.Rejection);
                Assert.Contains(caught.Notes, n => n.Contains("home return already in progress"));
                Assert.Equal(new[] { true, false }, StopCoil(rec)); // never "nothing moves" while it homes
                var events = sim.Events.ToList();
                Assert.True(events.IndexOf("home") < events.IndexOf("stop"));
                var st = axis.Status();
                Assert.False(st.Homing || st.Moving);
                Assert.False(st.Homed); // the home was cut short
            }
        }

        [Fact]
        public void Answered_refusal_sends_no_stop()
        {
            var (axis, rec) = SimKit.Open(new HomeRefusedSim(Rod(50.0)));
            using (axis)
            {
                axis.MoveTo(30.0, speed: 50.0);
                rec.Clear();
                Assert.Equal(0x11, Assert.Throws<IaiCommandRejectedException>(() => axis.Home(method: CommandMethod.Iai)).Rejection);
                Assert.Single(HomeFrames(rec));
                Assert.Empty(StopCoil(rec)); // this frame did not run: nothing to stop
            }
        }

        // ------------------------------------------------------------------ action mode: tri-state safety speed

        [Fact]
        public void Pc_mode_keeps_the_present_safety_speed()
        {
            var sim = new SimulatedUnit(new SimulatedUnitOptions { TimeScale = 50.0, Pmsl = false, SafetySpeed = true }); // e.g. left on by the PC tool's teach mode
            var (axis, rec) = SimKit.Open(sim);
            using (axis)
            {
                // PC: the PMSL coil (PMSS 0 only) and the SFTY coil (only when it changes), never 0x45 op1 = 1 (on the
                // PCON-CB that left the servo refusing to come on; VERIFIED 2026-11-02); PIO stays 0x45 cmd 0x0C op1 = 0
                var st = axis.SetActionMode(true); // safety null: keep SFTY as it is
                Assert.Empty(SimKit.Sent(rec, 0x45));
                Assert.Equal(new[] { T.Hex("01 05 04 27 FF 00 3D 01") }, SimKit.Sent(rec, 0x05));
                Assert.True(st.ModbusEnabled && st.SafetySpeed && sim.SafetySpeed);
                rec.Clear();
                st = axis.SetActionMode(true, false); // explicit off
                Assert.Empty(SimKit.Sent(rec, 0x45));
                Assert.Equal(new[] { T.Hex("01 05 04 01 00 00 9D 3A") }, SimKit.Sent(rec, 0x05));
                Assert.False(st.SafetySpeed);
                rec.Clear();
                st = axis.SetActionMode(true); // off stays off: nothing written
                Assert.Empty(SimKit.Sent(rec, 0x45));
                Assert.Empty(SimKit.Sent(rec, 0x05));
                Assert.False(st.SafetySpeed);
                rec.Clear();
                st = axis.SetActionMode(true, true); // explicit on
                Assert.Empty(SimKit.Sent(rec, 0x45));
                Assert.Equal(new[] { T.Hex("01 05 04 01 FF 00 DC CA") }, SimKit.Sent(rec, 0x05));
                Assert.True(st.SafetySpeed);
                rec.Clear();
                st = axis.SetActionMode(false); // PIO: the safety speed is kept too (op2 = 1)
                Assert.Equal(new byte[] { 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01 }, SimKit.Sent(rec, 0x45)[0].Skip(2).Take(6));
                Assert.False(st.ModbusEnabled);
                Assert.True(sim.SafetySpeed);
            }
        }

        [Fact]
        public void Pcon_servo_comes_on_after_taking_pc_control_with_the_action_mode()
        {
            // On the PCON-CB every 0x45 cmd 0x0C with op1 = 1 cleared DSSE b12 and servo ON was then ignored (no alarm)
            // until a PMSL coil OFF (VERIFIED, 2026-11-02; the simulator's PCON models it).
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Profile = SimProfile.Pcon, TimeScale = 20.0 });
            var (axis, rec) = SimKit.Open(sim);
            using (axis)
            {
                axis.SetActionMode(false); // PIO with nothing on the PIO connector: paused, servo off
                Assert.False(axis.Status().ServoOn);
                rec.Clear();
                Assert.True(axis.SetActionMode(true).ModbusEnabled);
                Assert.True(axis.Servo(true)!.ServoOn); // before the fix: "servo did not turn on within 3 s"
                Assert.DoesNotContain(SimKit.Sent(rec, 0x45), f => f[3] == 0x0C && f[5] == 1);
                axis.Servo(false);
                Assert.True(axis.SetActionMode(true, true).SafetySpeed); // already in PC: the safety speed by its coil
                Assert.False(axis.SetActionMode(true, false).SafetySpeed);
                Assert.True(axis.Servo(true)!.ServoOn);
            }
        }

        /// <summary>Every FC05 write answered with exception 01 (no coils); while <see cref="HoldPio"/>, DSSE reads PMSS 0.</summary>
        private sealed class NoCoilsSim : SimulatedUnit
        {
            public NoCoilsSim(SimulatedUnitOptions o)
                : base(o)
            {
            }

            public bool HoldPio { get; set; }

            protected override void WriteCoil(int address, bool on) => throw new SimException(0x01);

            protected override int ReadWord(int address) =>
                HoldPio && address == Registers.DSSE ? base.ReadWord(address) & ~Registers.DSSE_PMSS : base.ReadWord(address);
        }

        [Fact]
        public void Pc_mode_without_coils_falls_back_to_the_command()
        {
            // PC by 0x45 cmd 0x0C op1 = 1, the one way left, and PMSS still read back; once the bus knows FC05 is refused,
            // no coil is tried at all
            var sim = new NoCoilsSim(new SimulatedUnitOptions { TimeScale = 50.0, Pmsl = false });
            var (axis, rec) = SimKit.Open(sim);
            List<byte[]> ActionOps() =>
                SimKit.Sent(rec, 0x45).Where(f => f[3] == Registers.IAI_CMD_ACTION_MODE).Select(f => f.Skip(2).Take(6).ToArray()).ToList();
            using (axis)
            {
                Assert.False(axis.Status().ModbusEnabled);
                rec.Clear();
                var st = axis.SetActionMode(true, false);
                Assert.Equal(new[] { T.Hex("01 05 04 27 FF 00 3D 01") }, SimKit.Sent(rec, 0x05)); // tried once: exception 01
                Assert.Equal(new[] { new byte[] { 0x00, 0x0C, 0x00, 0x01, 0x00, 0x00 } }, ActionOps());
                Assert.True(st.ModbusEnabled && sim.Pmsl);
                axis.SetActionMode(false, false);
                rec.Clear();
                st = axis.SetActionMode(true, true); // FC05 known refused: straight to the command, op2 = the safety speed
                Assert.Empty(SimKit.Sent(rec, 0x05));
                Assert.Equal(new[] { new byte[] { 0x00, 0x0C, 0x00, 0x01, 0x00, 0x01 } }, ActionOps());
                Assert.True(st.ModbusEnabled && st.SafetySpeed);
                axis.SetActionMode(false, false);
                sim.HoldPio = true; // op1 = 1 accepted, PMSS stays 0
                rec.Clear();
                Assert.Contains("PMSS reads 0", Assert.Throws<VerificationException>(() => axis.SetActionMode(true, false)).Message);
                Assert.Equal(new[] { new byte[] { 0x00, 0x0C, 0x00, 0x01, 0x00, 0x00 } }, ActionOps());
                sim.HoldPio = false;
            }
        }

        // ------------------------------------------------------------------ rotary unit: mm parameters

        [Theory]
        [InlineData(2, 0.2, false)]
        [InlineData(4, 1.0, true)]
        public void Mm_parameters_are_written_in_degrees_on_a_rotary_unit(int no, double value, bool home)
        {
            var (axis, _) = SimKit.Open(new RotarySim(new SimulatedUnitOptions { TimeScale = 50.0 }));
            using (axis)
            {
                var result = axis.WriteUserParameter(no, value, commit: false, allowHomeParams: home);
                Assert.Equal("deg", result.Unit);
                Assert.Contains(result.Warnings, w => w.Contains("deg on this rotary axis"));
                Assert.Equal("1:Enable", axis.WriteUserParameter(5, 1, commit: false).After); // a bit parameter: no unit at all
            }
        }

        // ------------------------------------------------------------------ position table: any unit code

        private static PositionEntry RowWithinCaps(Axis axis) => axis.ReadPositionTable(0) with { SpeedMmS = 50.0, AccelG = 0.2, DecelG = 0.2 };

        [Fact]
        public void Unknown_unit_code_writes()
        {
            var (axis, rec) = SimKit.Open(new SimulatedUnit(new SimulatedUnitOptions { TimeScale = 50.0, UnitCode = 0x0C }));
            using (axis)
            {
                var entry = RowWithinCaps(axis);
                rec.Clear();
                Assert.Equal(entry.PositionMm, axis.WritePositionTable(0, entry, allowNonvolatile: true).PositionMm); // no per-model opt-in
                Assert.Single(SimKit.Sent(rec, 0x10));
            }
        }

        [Fact]
        public void Unreadable_unit_code_writes()
        {
            var (axis, rec) = SimKit.Open(new NoIdentitySim(new SimulatedUnitOptions { TimeScale = 50.0, SlaveIdDelayS = 2.0 }), retries: 0);
            using (axis)
            {
                var entry = RowWithinCaps(axis);
                rec.Clear();
                axis.WritePositionTable(0, entry, allowNonvolatile: true);
                Assert.Single(SimKit.Sent(rec, 0x10));
            }
        }

        [Fact]
        public void Known_unit_code_writes_without_an_opt_in()
        {
            var (axis, rec) = SimKit.Open(new SimulatedUnit(Rod(50.0))); // unit 0x00 (EC-R6S, also the SP unit)
            using (axis)
            {
                var entry = RowWithinCaps(axis);
                rec.Clear();
                axis.WritePositionTable(0, entry, allowNonvolatile: true);
                Assert.Single(SimKit.Sent(rec, 0x10));
            }
        }
    }
}
