using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>The axis class against <see cref="SimulatedUnit"/>: identify, status, commands, soft limits, motion, safety guards.</summary>
    [Collection(SimCollection.Name)]
    public class SimEcTests
    {
        private static void Near(double expected, double actual, double delta) =>
            Assert.True(Math.Abs(expected - actual) <= delta, $"expected {expected} +/- {delta}, got {actual}");

        /// <summary>No monitor words past DSSE (an older unit): the status falls back to the 8-word block.</summary>
        private sealed class NarrowSim : SimulatedUnit
        {
            protected override int ReadWord(int address) =>
                address >= 0x9008 && address <= 0x9021 ? throw new SimException(0x02) : base.ReadWord(address);
        }

        /// <summary>FC11 refused.</summary>
        private class NoIdSim : SimulatedUnit
        {
            protected override byte[] Fc11(byte[] body) => throw new SimException(0x01);
        }

        /// <summary>FC11 refused and CTCL (class 3 item 1) blank.</summary>
        private sealed class NoIdentity : NoIdSim
        {
            protected override int ReadWord(int address) => address == 0x0300 || address == 0x0301 ? 0 : base.ReadWord(address);
        }

        // ------------------------------------------------------------------ read-only

        [Fact]
        public void Status_decodes_live_values()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                var st = axis.Status();
                Assert.Equal(73.95, st.PositionMm);
                Assert.Equal(0, st.AlarmCode);
                Assert.True(st.ServoOn && st.Homed && st.InPosition && st.ModbusEnabled);
                Assert.False(st.Moving || st.Alarm || st.EmergencyStop || st.Warning);
                Assert.Equal(0x3098u, st.Raw["DSS1"]);
                Assert.Equal(0x33C2u, st.Raw["DSSE"]);
                Assert.Equal(0x0003u, st.Raw["ZONS"]); // both zones, no position marker at 73.95 mm
                Assert.Equal((uint)Registers.DOPM_3POS_NO_ALARM, st.Raw["DOPM"]);
                Assert.False(st.AtMiddle);
                Assert.Equal(0.0, st.VelocityMmS);
                Assert.Equal(120, st.CurrentMa);
                Assert.True(st.Ready);
            }
        }

        [Fact]
        public void Status_falls_back_to_the_short_block()
        {
            var axis = new Axis(SimKit.Bus(new NarrowSim()));
            var st = axis.Status();
            Assert.Equal(73.95, st.PositionMm);
            Assert.Equal(0.0, st.VelocityMmS); // MOVE clear
            Assert.Null(st.CurrentMa);
            Assert.False(st.Raw.ContainsKey("CNOW"));
            Assert.False(st.Raw.ContainsKey("ZONS"));
            Assert.Null(st.AtBackwardEnd); // never reported without ZONS (DOPM is not used for markers)
            Assert.Null(st.AtForwardEnd);
            Assert.Null(st.AtMiddle);
            Assert.Equal(73.95, axis.Status().PositionMm); // stays on the short block
        }

        [Fact]
        public void Identify()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                var info = axis.Identify();
                Assert.Equal(0xEC, info.SystemCode);
                Assert.Equal("MWB-EC-DSRR4-WA-MF", info.Model); // the 0x0600 model string is the display name
                Assert.Equal("EC 24 V, unit 0x02 (EC dust/drip-proof)", info.Controller); // the unit code's family name
                Assert.Equal((9, "slider (small) (AKID 9)"), (info.ActuatorKind!.Value, info.Actuator)); // kind from AKID
                Assert.Equal(("FC11", true, 0), (info.IdentitySource, info.UnitKnown!.Value, info.AxisNumber!.Value));
                Assert.Equal("MWB-EC-DSRR4", info.UserName); // 0x061A: the user-editable unit name
                Assert.Equal((1, false), (info.ExtensibilityLevel!.Value, info.Rotary!.Value)); // ACCL 0x00010110: bits 8-11 = 1
                Assert.Equal(700.0, info.MaxSpeedMmS);
                Assert.Equal((1.0, 1.0), (info.MaxAccelG!.Value, info.MaxDecelG!.Value));
                Assert.Contains("MAXV item 7.4", info.SpeedLimitSource);
                Assert.Equal("0013", info.Firmware);
                Assert.Equal(27, info.IndexHeader!.RecordCount);
                Assert.Equal(27, info.Categories.Count);
                Assert.Equal(0x9900, info.Category(Registers.CATEGORY_COMMAND2)!.DataAddress);
                Assert.Equal(0x0400, info.Category(Registers.CATEGORY_IO_CONTROL)!.CoilBase);
                Assert.Empty(info.Errors);
                Assert.Equal("ec ff 02 00 13", AxisJson.DeviceInfo(info)["slave_id_raw"]);
            }
        }

        [Fact]
        public void Identify_reports_unsupported_parts()
        {
            // FC11 refused: the same identity comes from CTCL (class 3 item 1, 0x0300), the fallback source (UNVERIFIED: FC11 answered on every unit tested)
            var info = new Axis(SimKit.Bus(new NoIdSim())).Identify();
            Assert.Equal((0xEC, 0x02, "0013"), (info.SystemCode!.Value, info.UnitCode!.Value, info.Firmware));
            Assert.Equal("CTCL 0x0300", info.IdentitySource);
            Assert.Null(info.SlaveIdRaw);
            Assert.Equal("MWB-EC-DSRR4-WA-MF", info.Model); // still named from 0x0600
            Assert.Equal(27, info.Categories.Count);
            Assert.Contains(info.Errors, e => e.Contains("slave id", StringComparison.OrdinalIgnoreCase));
            Assert.Contains(info.Errors, e => e.Contains("CTCL"));

            info = new Axis(SimKit.Bus(new NoIdentity())).Identify();
            Assert.Null(info.SystemCode);
            Assert.Equal("unknown", info.Controller);
            Assert.Null(info.IdentitySource);
        }

        [Fact]
        public void Read_position_table()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                var entries = Enumerable.Range(0, 3).Select(axis.ReadPositionTable).ToList();
                Assert.Equal(new[] { 0.0, 150.0, 75.0 }, entries.Select(e => e.PositionMm));
                Assert.Equal(new[] { 108.5, 248.5, 248.5 }, entries.Select(e => e.SpeedMmS));
                Assert.Equal(0.3, entries[1].AccelG);
                Assert.Equal(0.3, entries[1].DecelG);
                Assert.Equal(0.1, entries[1].BandMm);
                Assert.Equal(0x0010, entries[1].ControlFlags);
                Assert.Equal(0x0065, entries[1].Checksum);
                Assert.All(entries, e => Assert.True(e.ChecksumOk));
                Assert.ThrowsAny<ArgumentException>(() => axis.ReadPositionTable(0x300));
            }
        }

        [Fact]
        public void Status_alarm_logic()
        {
            var baseWords = new ushort[] { 0, 7395, 0, 0, 0, 0x3098, 0x8000, 0x33C2 };
            var warn = (ushort[])baseWords.Clone();
            warn[2] = 0x04E;
            warn[5] = (ushort)(0x3098 | Registers.DSS1_ALML);
            var st = Status.FromRegisters(warn);
            Assert.False(st.Alarm);
            Assert.True(st.Warning);
            var bare = (ushort[])baseWords.Clone();
            bare[2] = 0x0E0; // a code without any alarm bit: treated as an alarm (fail safe)
            Assert.True(Status.FromRegisters(bare).Alarm);
            var major = (ushort[])baseWords.Clone();
            major[5] |= Registers.DSS1_ALMH;
            Assert.True(Status.FromRegisters(major).Alarm);
            var negative = (ushort[])baseWords.Clone();
            negative[0] = 0xFFFF;
            negative[1] = 0xFFF5;
            Assert.Equal(-0.11, Status.FromRegisters(negative).PositionMm);
            Assert.ThrowsAny<ArgumentException>(() => Status.FromRegisters(baseWords.Take(5).ToArray()));
        }

        [Fact]
        public void Options_are_validated()
        {
            var bus = SimKit.Bus(new SimulatedUnit());
            Assert.ThrowsAny<ArgumentException>(() => new Axis(bus, 1, new AxisOptions { Stroke = (10.0, 5.0) }));
            Assert.ThrowsAny<ArgumentException>(() => new Axis(bus, 1, new AxisOptions { MaxSpeedMmS = 0 }));
            Assert.ThrowsAny<ArgumentException>(() => new Axis(bus, 1, new AxisOptions { MaxAccelG = 5.0 }));
            Assert.ThrowsAny<ArgumentException>(() => new Axis(bus, 1, new AxisOptions { Strategy = (CommandMethod)42 }));
            Assert.IsType<IaiCommandStrategy>(new Axis(bus, 1, new AxisOptions { Strategy = CommandMethod.Iai }).Strategy);
        }

        // ------------------------------------------------------------------ moves

        [Fact]
        public void Move_to_and_wait()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var st = axis.MoveTo(80.0, speed: 50.0, accelG: 0.1)!;
                Near(80.0, st.PositionMm, 0.1);
                Assert.False(st.Moving);
                Assert.True(st.InPosition);
                Assert.Equal(80.0, sim.PositionMm, 6);
            }
        }

        [Fact]
        public void Move_frame_is_the_full_block_with_ctlf()
        {
            // DOC example (50 mm, 0.1 mm, 100 mm/s, 0.3 G) extended to 9 words: PPOW 0, CTLF 0
            var (axis, _, rec) = SimKit.MakeAxis();
            using (axis)
            {
                rec.Clear();
                axis.MoveTo(50.0, speed: 100.0, accelG: 0.3, band: 0.1, wait: false);
                var expected = SimKit.Frame(1, "10 99 00 00 09 12 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 00 00 00 00");
                Assert.Contains(rec.Tx, f => f.SequenceEqual(expected));
                axis.WaitIdle(50.0, 0.1, timeout: 5.0);
            }
        }

        public static IEnumerable<object[]> BadMoves() => new[]
        {
            new object[] { 150.01, 50.0, 0.1, 0.1 },
            new object[] { -0.01, 50.0, 0.1, 0.1 },
            new object[] { double.NaN, 50.0, 0.1, 0.1 },
            new object[] { 80.0, 0.0, 0.1, 0.1 },
            new object[] { 80.0, 100.01, 0.1, 0.1 },
            new object[] { 80.0, 50.0, 0.31, 0.1 },
            new object[] { 80.0, 50.0, 0.0, 0.1 },
            new object[] { 80.0, 50.0, 0.1, 0.0 },
            new object[] { 80.0, 50.0, 0.1, 20.0 },
        };

        [Theory]
        [MemberData(nameof(BadMoves))]
        public void Soft_limits_send_nothing(double mm, double speed, double accelG, double band)
        {
            var (axis, sim, rec) = SimKit.MakeAxis();
            using (axis)
            {
                Assert.Throws<SoftLimitException>(() => axis.MoveTo(mm, speed, accelG, band));
                Assert.Empty(rec.Tx);
                Assert.False(sim.Moving);
            }
        }

        [Fact]
        public void Limits_are_inclusive()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                axis.MoveTo(150.0, speed: 100.0, accelG: 0.3);
                Assert.Equal(150.0, axis.Status().PositionMm);
                axis.MoveTo(0.0, speed: 100.0, accelG: 0.3);
                Assert.Equal(0.0, axis.Status().PositionMm);
            }
        }

        [Fact]
        public void Move_by()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                Near(78.95, axis.MoveBy(5.0, speed: 20.0)!.PositionMm, 0.01);
                Near(68.95, axis.MoveBy(-10.0, speed: 20.0)!.PositionMm, 0.01);
                Assert.Throws<SoftLimitException>(() => axis.MoveBy(100.0));
                Assert.Throws<SoftLimitException>(() => axis.MoveBy(double.PositiveInfinity));
            }
        }

        [Fact]
        public void Target_reached_flag()
        {
            var (axis, _, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 2.0));
            using (axis)
            {
                Assert.False(axis.Status().TargetReached); // live start value 0x3098
                var st = axis.MoveTo(75.0, speed: 50.0)!;
                Assert.True(st.TargetReached);
                Assert.Equal(0x3099u, st.Raw["DSS1"]); // as seen live after a completed move (b14 aside)
                axis.MoveTo(120.0, speed: 20.0, wait: false);
                st = axis.Status();
                Assert.Equal(0x33F2u, st.Raw["DSSE"]); // live value while moving
                Assert.Equal(0u, st.Raw["DSS1"] & (uint)(Registers.DSS1_PEND | Registers.DSS1_BKRL)); // live: b3, b7 clear while moving
                axis.Stop(wait: true);
                Assert.False(axis.Status().TargetReached);
            }
        }

        [Fact]
        public void Velocity_from_position_samples()
        {
            var (axis, _, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 1.0));
            using (axis)
            {
                axis.MoveTo(120.0, speed: 20.0, accelG: 0.3, wait: false);
                Thread.Sleep(150);
                axis.Status();
                Thread.Sleep(100);
                var st = axis.Status();
                Assert.True(st.Moving);
                Near(20.0, st.VelocityMmS!.Value, 3.0);
                st = axis.Stop(wait: true);
                Assert.Equal(0.0, st.VelocityMmS);
            }
        }

        [Fact]
        public void Retarget_with_a_dropped_reply()
        {
            // hardware: the read right after a mid-move write got no reply; the retries must absorb it
            var (axis, _, rec) = SimKit.MakeAxis(SimKit.Sim(timeScale: 2.0) with { DropReplyAfterRetarget = true });
            using (axis)
            {
                axis.MoveTo(120.0, speed: 20.0, wait: false);
                Thread.Sleep(50);
                Near(90.0, axis.MoveTo(90.0, speed: 20.0)!.PositionMm, 0.1);
                var tx = rec.Tx;
                var lastMove = tx.Select((f, i) => (f, i)).Last(x => x.f[1] == 0x10).i;
                Assert.Equal(tx[lastMove + 1], tx[lastMove + 2]); // the unanswered read was repeated
                Assert.Equal(0x03, tx[lastMove + 1][1]);
            }
        }

        [Fact]
        public void Preconditions()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                sim.ServoOn = false;
                Assert.Contains("servo is off", Assert.Throws<EcNotReadyException>(() => axis.MoveTo(80.0)).Message);
                sim.ServoOn = true;
                sim.Homed = false;
                Assert.Contains("not homed", Assert.Throws<EcNotReadyException>(() => axis.MoveTo(80.0)).Message);
                sim.Homed = true;
                sim.Pmsl = false;
                axis.AutoModbusControl = false; // strict mode: no automatic PMSL write
                Assert.Contains("EnableModbusControl", Assert.Throws<EcNotReadyException>(() => axis.MoveTo(80.0)).Message);
                axis.AutoModbusControl = true;
                sim.Pmsl = true;
                sim.TriggerAlarm(0x0E0);
                Assert.Equal(0x0E0, Assert.Throws<EcAlarmException>(() => axis.MoveTo(80.0)).AlarmCode);
                Assert.False(sim.Moving);
            }
        }

        [Fact]
        public void Stop_mid_move()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 5.0));
            using (axis)
            {
                axis.MoveTo(140.0, speed: 50.0, accelG: 0.3, wait: false);
                Thread.Sleep(100);
                Assert.True(axis.Status().Moving);
                var st = axis.Stop(wait: true);
                Assert.False(st.Moving);
                Assert.InRange(st.PositionMm, 73.96, 138.99);
                Assert.Contains("stop", sim.Events);
            }
        }

        [Fact]
        public void Wait_timeout_stops_and_raises()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 1.0));
            using (axis)
            {
                var exc = Assert.Throws<MotionTimeoutException>(() => axis.MoveTo(140.0, speed: 10.0, accelG: 0.1, timeout: 0.2));
                Assert.True(exc.StopAttempted);
                Assert.Contains("decel stop sent after this error", exc.Notes);
                Assert.Single(sim.Events, e => e == "stop");
                Thread.Sleep(100);
                Assert.False(sim.Moving);
                Assert.True(sim.PositionMm < 80.0);
            }
        }

        [Fact]
        public void Alarm_during_motion()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 2.0));
            using (axis)
            {
                axis.MoveTo(140.0, speed: 20.0, wait: false);
                sim.TriggerAlarm(0x0E0);
                Assert.Equal(0x0E0, Assert.Throws<EcAlarmException>(() => axis.WaitIdle(140.0, 0.1, timeout: 5.0)).AlarmCode);
                Assert.Contains("stop", sim.Events);
            }
        }

        [Fact]
        public void Link_error_during_wait_attempts_a_stop()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 1.0));
            using (axis)
            {
                axis.MoveTo(140.0, speed: 20.0, wait: false);
                sim.InjectFault(SimFault.Timeout, 1 + ModbusRtu.DefaultRetries); // the next status read fails completely (every attempt)
                var exc = Assert.Throws<ModbusTimeoutException>(() => axis.WaitIdle(140.0, 0.1, timeout: 5.0));
                Assert.True(exc.StopAttempted);
                Assert.Contains("stop", sim.Events);
            }
        }

        [Fact]
        public void Cancelling_a_wait_stops()
        {
            // the C# edition's counterpart of Ctrl+C during a wait: every waiting call takes a CancellationToken
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 1.0));
            using (axis)
            using (var cts = new CancellationTokenSource(TimeSpan.FromMilliseconds(80)))
            {
                Assert.ThrowsAny<OperationCanceledException>(() => axis.MoveTo(140.0, speed: 20.0, cancellationToken: cts.Token));
                Assert.Contains("stop", sim.Events);
                Assert.False(sim.StopHeld);
                Assert.False(sim.Moving);
            }
        }

        [Fact]
        public void Pause_resume()
        {
            var (axis, _, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 5.0));
            using (axis)
            {
                axis.MoveTo(100.0, speed: 20.0, accelG: 0.3, wait: false);
                Thread.Sleep(50);
                axis.Pause();
                Thread.Sleep(50);
                var st = axis.Status();
                Assert.True(st.Paused);
                Assert.False(st.Moving);
                axis.Resume();
                Assert.Equal(100.0, axis.WaitIdle(100.0, 0.1, timeout: 10.0).PositionMm);
            }
        }

        [Fact]
        public void Dispose_closes_an_owned_port_and_never_stops()
        {
            // Dispose only releases a STOP coil left on and closes an owned port (it cannot see an exception leaving the
            // using block, so it never stops a move: cancel the wait instead)
            var sim = new SimulatedUnit(new SimulatedUnitOptions { TimeScale = 5.0 });
            Assert.Throws<InvalidOperationException>((Action)(() =>
            {
                using var axis = Axis.Open(sim, options: new AxisOptions { PollIntervalS = 0.005 });
                axis.MoveTo(140.0, speed: 20.0, wait: false);
                throw new InvalidOperationException("user code failed");
            }));
            Assert.True(sim.Closed);
            Assert.DoesNotContain("stop", sim.Events);

            var clean = new SimulatedUnit();
            using (var axis = Axis.Open(clean))
            {
                axis.Status();
            }

            Assert.True(clean.Closed);
            Assert.DoesNotContain("stop", clean.Events);
            var bus = SimKit.Bus(new SimulatedUnit());
            new Axis(bus).Dispose();
            bus.ReadRegisters(1, Registers.PNOW, 2); // a borrowed bus stays open
        }

        // ------------------------------------------------------------------ command strategies

        private static (SimulatedUnit Sim, IReadOnlyList<byte[]> Tx) RunCycle(CommandMethod strategy)
        {
            var (axis, sim, rec) = SimKit.MakeAxis(strategy: strategy);
            using (axis)
            {
                axis.Servo(false);
                Assert.False(sim.ServoOn);
                axis.Servo(true);
                Assert.True(sim.ServoOn);
                var st = axis.Home()!;
                Assert.True(st.Homed);
                Assert.Equal(0.0, st.PositionMm);
                axis.MoveTo(20.0, speed: 100.0, accelG: 0.3, wait: false);
                axis.Stop(wait: true);
                sim.TriggerAlarm(0x0E0);
                Assert.False(sim.ServoOn);
                st = axis.ResetAlarm();
                Assert.False(st.Alarm);
                Assert.Equal(0, st.AlarmCode);
                return (sim, rec.Tx);
            }
        }

        private static int IndexOf(IReadOnlyList<byte[]> frames, byte[] frame, int start = 0)
        {
            for (var i = start; i < frames.Count; i++)
            {
                if (frames[i].SequenceEqual(frame))
                {
                    return i;
                }
            }

            return -1;
        }

        [Fact]
        public void Coil_strategy()
        {
            var (_, tx) = RunCycle(CommandMethod.Coil);
            foreach (var frame in new[]
            {
                "01 05 04 03 00 00 3C FA", // servo off
                "01 05 04 03 FF 00 7D 0A", // servo on
                "01 05 04 0B 00 00 BD 38", // home: 0 ...
                "01 05 04 0B FF 00 FC C8", // ... then the rising edge
                "01 05 04 2C FF 00 4C C3", // stop
                "01 05 04 07 FF 00 3C CB", // alarm reset pulse
                "01 05 04 07 00 00 7D 3B",
            })
            {
                Assert.True(IndexOf(tx, T.Hex(frame)) >= 0, frame);
            }

            var homeOn = IndexOf(tx, T.Hex("01 05 04 0B FF 00 FC C8"));
            Assert.True(IndexOf(tx, T.Hex("01 05 04 0B 00 00 BD 38"), homeOn + 1) > homeOn); // released after homing
        }

        [Fact]
        public void Iai_strategy()
        {
            var (_, tx) = RunCycle(CommandMethod.Iai);
            Assert.True(IndexOf(tx, T.Hex("01 45 00 09 00 01 00 00 18 C2")) >= 0); // 0x45 servo-on frame (VERIFIED, test I2)
            Assert.True(IndexOf(tx, T.Hex("01 45 00 0A 00 07 00 00 BC C3")) >= 0); // 0x45 home frame: op1 = ORG (9.16) = 7 (VERIFIED)
            Assert.True(IndexOf(tx, T.Hex("01 45 00 0A 00 00 00 00 0D 02")) < 0); // op1 = 0 is rejected 0x11 (test I3)
            Assert.True(IndexOf(tx, T.Hex("01 45 00 05 00 00 00 00 59 03")) >= 0); // 0x45 alarm reset, op1 0 (VERIFIED)
            Assert.DoesNotContain(tx, f => f[1] == 0x45 && f[3] == 0x0B); // 0x45 decel stop never used
            var coils = SimKit.Of(tx, 0x05).Select(f => (f[2] << 8) | f[3]).Distinct().ToList();
            Assert.Equal(new[] { Registers.COIL_STOP }, coils); // the only coil: the verified STOP
        }

        [Fact]
        public void Iai2_strategy()
        {
            var (sim, tx) = RunCycle(CommandMethod.Iai2);
            Assert.Contains(tx, f => f[1] == 0x48 && f[3] == 0x00 && f[4] == 0x31); // group, cmd u16 0x0031
            Assert.Contains("home complete", sim.Events);
        }

        [Fact]
        public void Servo_off_while_moving_stops_first()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim(timeScale: 2.0));
            using (axis)
            {
                axis.MoveTo(140.0, speed: 20.0, wait: false);
                axis.Servo(false);
                var events = sim.Events.ToList();
                Assert.True(events.IndexOf("stop") < events.IndexOf("servo off"));
            }
        }

        [Fact]
        public void Servo_on_refused_with_an_alarm()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                sim.TriggerAlarm(0x0E0);
                Assert.Throws<EcAlarmException>(() => axis.Servo(true));
            }
        }

        [Fact]
        public void Enable_modbus_control()
        {
            var (axis, sim, rec) = SimKit.MakeAxis();
            using (axis)
            {
                sim.Pmsl = false;
                Assert.False(axis.Status().ModbusEnabled);
                Assert.True(axis.EnableModbusControl()!.ModbusEnabled);
                Assert.True(IndexOf(rec.Tx, T.Hex("01 05 04 27 FF 00 3D 01")) >= 0);
                axis.DisableModbusControl();
                Assert.False(sim.Pmsl);
            }
        }

        // ------------------------------------------------------------------ position table writes

        [Fact]
        public void Table_write_needs_no_opt_in()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var entry = axis.ReadPositionTable(2) with { PositionMm = 60.0, SpeedMmS = 50.0 };
                Assert.Equal(60.0, axis.WritePositionTable(2, entry).PositionMm); // no allowNonvolatile
                Assert.Equal(1, sim.NonvolatileWrites);
            }
        }

        [Fact]
        public void Table_write_when_allowed()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var original = axis.ReadPositionTable(2);
                var entry = new PositionEntry { PositionMm = 60.0, BandMm = 0.2, SpeedMmS = 50.0, AccelG = 0.2, DecelG = 0.25, ControlFlags = original.ControlFlags };
                var back = axis.WritePositionTable(2, entry, allowNonvolatile: true);
                Assert.Equal(1, sim.NonvolatileWrites);
                Assert.Equal(60.0, back.PositionMm);
                Assert.Equal(0.25, back.DecelG);
                Assert.NotEqual(original.Checksum, back.Checksum); // updated (by the driver's own checksum word)
                Assert.True(back.ChecksumOk);
                Assert.Equal(0.2, axis.ReadPositionTable(2).BandMm);
            }
        }

        public static IEnumerable<object[]> BadEntries() => new[]
        {
            new object[] { new PositionEntry { PositionMm = 151.0 } },
            new object[] { new PositionEntry { PositionMm = 50.0, SpeedMmS = 248.5 } }, // above MaxSpeedMmS
            new object[] { new PositionEntry { PositionMm = 50.0, AccelG = 0.5 } },
            new object[] { new PositionEntry { PositionMm = 50.0, BandMm = 0.0 } },
        };

        [Theory]
        [MemberData(nameof(BadEntries))]
        public void Table_write_validation(PositionEntry entry)
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                Assert.Throws<SoftLimitException>(() => axis.WritePositionTable(0, entry, allowNonvolatile: true));
                Assert.Equal(0, sim.NonvolatileWrites);
            }
        }

        [Fact]
        public void Table_checksum_not_updated_is_detected()
        {
            var (axis, sim, _) = SimKit.MakeAxis(SimKit.Sim() with { RecomputePositionChecksum = false });
            using (axis)
            {
                var entry = axis.ReadPositionTable(2) with { PositionMm = 60.0, SpeedMmS = 50.0 };
                Assert.Contains("checksum", Assert.Throws<VerificationException>(() => axis.WritePositionTable(2, entry, allowNonvolatile: true, writeChecksum: false)).Message);
                Assert.True(axis.WritePositionTable(2, entry, allowNonvolatile: true).ChecksumOk); // default: the checksum is written
                Assert.Equal(Registers.PositionChecksum(entry.ToRegisters()), sim.PositionRow(2)[15]);
            }
        }

        [Fact]
        public void Table_rows_past_the_units_own_are_written_with_a_warning()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var entry = axis.ReadPositionTable(2) with { PositionMm = 60.0, SpeedMmS = 50.0 };
                foreach (var n in new[] { 256, 260, 267 })
                {
                    // the EC's internal simple-data store: never, even with allowUnsafe
                    Assert.Contains("simple-data", Assert.Throws<NonVolatileWriteException>(() => axis.WritePositionTable(n, entry, allowNonvolatile: true, allowUnsafe: true)).Message);
                }

                // a row the table cannot address: refused
                var unaddressable = Assert.Throws<NonVolatileWriteException>(() => axis.WritePositionTable(-1, entry));
                Assert.Contains("not written", unaddressable.Message);
                Assert.Equal(0, sim.NonvolatileWrites);

                foreach (var n in new[] { 3, 255, 0x2FF })
                {
                    using var capture = new LogCapture();
                    Assert.Equal(60.0, axis.WritePositionTable(n, entry).PositionMm);
                    Assert.Contains(capture.Messages, m => m.Contains("rows 0-2") && m.Contains("written anyway"));
                }

                Assert.Equal(3, sim.NonvolatileWrites);
                var blank = new PositionEntry { PositionMm = 61.0, SpeedMmS = 50.0 };
                // still accepted
                Assert.Equal(61.0, axis.WritePositionTable(3, blank, true, allowUnsafe: true).PositionMm);
            }
        }

        public static IEnumerable<object[]> BadValues() => new[]
        {
            new object[] { "PushPower 0xFFFF", (Func<PositionEntry, PositionEntry>)(e => e with { PushPower = 0xFFFF }) },
            new object[] { "PushPower 300", (Func<PositionEntry, PositionEntry>)(e => e with { PushPower = 300 }) },
            new object[] { "LoadThreshold 300", (Func<PositionEntry, PositionEntry>)(e => e with { LoadThreshold = 300 }) },
            new object[] { "flags 0xFFFF", (Func<PositionEntry, PositionEntry>)(e => e with { ControlFlags = 0xFFFF }) },
            new object[] { "above the slider's MXPP 60 % (153)", (Func<PositionEntry, PositionEntry>)(e => e with { ControlFlags = 0x0020 | Registers.CTLF_PUSH, PushPower = 154 }) },
            new object[] { "a push row without force", (Func<PositionEntry, PositionEntry>)(e => e with { ControlFlags = 0x0020 | Registers.CTLF_PUSH, PushPower = 0 }) },
            new object[] { "decel above MaxAccelG", (Func<PositionEntry, PositionEntry>)(e => e with { AccelG = 0.3, DecelG = 0.31 }) },
            new object[] { "zone +inf", (Func<PositionEntry, PositionEntry>)(e => e with { ZonePlusMm = double.PositiveInfinity }) },
            new object[] { "zone NaN", (Func<PositionEntry, PositionEntry>)(e => e with { ZoneMinusMm = double.NaN }) },
            new object[] { "position inf", (Func<PositionEntry, PositionEntry>)(e => e with { PositionMm = double.PositiveInfinity }) },
            new object[] { "does not fit int32 counts", (Func<PositionEntry, PositionEntry>)(e => e with { ZonePlusMm = 3e7 }) },
        };

        [Theory]
        [MemberData(nameof(BadValues))]
        public void Table_entry_values_validated(string what, Func<PositionEntry, PositionEntry> change)
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var baseEntry = axis.ReadPositionTable(2) with { SpeedMmS = 50.0 };
                // allowUnsafe lifts none of these (it is accepted and ignored)
                var bad = change(baseEntry);
                var refused = Assert.Throws<SoftLimitException>(
                    () => axis.WritePositionTable(2, bad, allowUnsafe: true));
                Assert.True(refused != null, what);
                Assert.Equal(0, sim.NonvolatileWrites);
            }
        }

        [Fact]
        public void Table_unknown_flag_bits_are_written_with_a_warning()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                // unknown bit 6
                var entry = axis.ReadPositionTable(2) with { SpeedMmS = 50.0, ControlFlags = 0x0020 | 0x0040 };
                using var capture = new LogCapture();
                Assert.Equal(0x0060, axis.WritePositionTable(2, entry).ControlFlags);
                Assert.Contains(capture.Messages, m => m.Contains("use unknown bits") && m.Contains("written anyway"));
                Assert.Equal(1, sim.NonvolatileWrites);
            }
        }

        [Fact]
        public void Table_push_rows_within_the_units_caps()
        {
            var (axis, _, _) = SimKit.MakeAxis();
            using (axis)
            {
                // push rows on row 1: row 2 is the middle point of this 3-position slider, no push there (DOC ME3837-1D p.2-5)
                var baseEntry = axis.ReadPositionTable(1) with { PositionMm = 140.0, SpeedMmS = 50.0 };
                Assert.Throws<SoftLimitException>(() => axis.WritePositionTable(1, baseEntry with { ControlFlags = 0x0010 | Registers.CTLF_PUSH, PushPower = 154 }, true));
                Assert.Equal(60.0, axis.PushCapPct); // min(the fallback 70, MXPP 7.9 = 60) on the slider
                var pushing = baseEntry with { ControlFlags = 0x0010 | Registers.CTLF_PUSH, PushPower = 153 };
                Assert.Equal(153, axis.WritePositionTable(1, pushing, true).PushPower);

                // below the recommended minimum (MNPP 20 %) is only a recommendation (EC #3 ships a 10 % row; VERIFIED to run)
                var weak = baseEntry with { ControlFlags = 0x0010 | Registers.CTLF_PUSH, PushPower = 10 };
                Assert.Equal(10, axis.WritePositionTable(1, weak, true).PushPower);
            }
        }

        [Fact]
        public void Table_role_bits_of_rows_1_and_2_are_kept_or_changed_with_a_warning()
        {
            var (axis, sim, _) = SimKit.MakeAxis();
            using (axis)
            {
                var kept = axis.ReadPositionTable(2) with { PositionMm = 60.0, SpeedMmS = 50.0 };
                using (var quiet = new LogCapture())
                {
                    Assert.Equal(0x0020, axis.WritePositionTable(2, kept, true).ControlFlags);
                    Assert.DoesNotContain(quiet.Messages, m => m.Contains("role bits"));
                }

                var fresh = new PositionEntry { PositionMm = 61.0, SpeedMmS = 50.0 }; // ControlFlags 0 clears 0x0020
                using (var capture = new LogCapture())
                {
                    Assert.Equal(0, axis.WritePositionTable(2, fresh).ControlFlags); // no allowUnsafe needed
                    Assert.Contains(capture.Messages, m => m.Contains("role bits") && m.Contains("written anyway"));
                }

                Assert.Equal(2, sim.NonvolatileWrites);
            }
        }

        [Fact]
        public void Table_full_entry_with_checksum_is_written_by_default()
        {
            var (axis, _, rec) = SimKit.MakeAxis();
            using (axis)
            {
                rec.Clear();
                var entry = axis.ReadPositionTable(0) with { PositionMm = 5.0, SpeedMmS = 50.0 };
                axis.WritePositionTable(0, entry, allowNonvolatile: true);
                var write = rec.Tx.First(f => f[1] == 0x10);
                Assert.Equal((0x1000, 16), ((write[2] << 8) | write[3], (write[4] << 8) | write[5]));
            }
        }

        [Fact]
        public void Table_checksum_helper_matches_live_rows()
        {
            var sim = new SimulatedUnit();
            for (var n = 0; n < 4; n++)
            {
                var row = sim.PositionRow(n);
                Assert.Equal(row[15], Registers.PositionChecksum(row));
                Assert.Equal(0, row.Sum(w => (w >> 8) + (w & 0xFF)) % 256);
            }
        }

        [Fact]
        public void Table_entry_round_trip()
        {
            var words = new SimulatedUnit().PositionRow(1);
            var entry = PositionEntry.FromRegisters(words);
            Assert.Equal(words.Take(15), entry.ToRegisters());
            Assert.Equal(Registers.POSITION_WORDS_WRITABLE, entry.ToRegisters().Length);
        }
    }
}
