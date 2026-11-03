using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using IaiActuatorDriver.Simulation;
using Xunit;

namespace IaiActuatorDriver.Tests
{
    /// <summary>SimulatedUnit: seeded live values, directory, byte-level protocol and the motion model (raw frames, no axis class).</summary>
    public class SimulatedUnitTests
    {
        private static void Near(double expected, double actual, double delta) =>
            Assert.True(Math.Abs(expected - actual) <= delta, $"expected {expected} +/- {delta}, got {actual}");

        private static (SimulatedUnit Sim, ModbusRtu Bus) Make(SimulatedUnitOptions? options = null)
        {
            var sim = new SimulatedUnit(options);
            return (sim, SimKit.Bus(sim));
        }

        private static (SimulatedUnit Sim, ModbusRtu Bus, FakeClock Clock) Clocked(SimulatedUnitOptions? options = null)
        {
            var clock = new FakeClock();
            var sim = new SimulatedUnit((options ?? new SimulatedUnitOptions()) with { Clock = clock.Read });
            return (sim, SimKit.Bus(sim), clock);
        }

        private static (int Dss1, int Dsse) Bits(ModbusRtu bus)
        {
            var words = bus.ReadRegisters(1, 0x9000, 8);
            return (words[5], words[7]);
        }

        // ------------------------------------------------------------------ seeded live values

        [Fact]
        public void Seed_live_monitor_values()
        {
            var (_, bus) = Make();
            var words = bus.ReadRegisters(1, 0x9000, 8);
            Assert.Equal(new ushort[] { 0x0000, 0x1CE3 }, words.Take(2)); // 73.95 mm
            Assert.Equal(0, words[2]); // no alarm
            Assert.Equal(0x3098, words[5]); // DSS1
            Assert.Equal(0x33C2, words[7]); // DSSE
        }

        [Fact]
        public void Seed_live_direct_move_block_and_drg1()
        {
            var (_, bus) = Make();
            Assert.Equal(new ushort[] { 0, 7395, 0, 10, 0x0001, 0x1170, 30 }, bus.ReadRegisters(1, 0x9900, 7));
            Assert.Equal(new ushort[] { 0x1000 }, bus.ReadRegisters(1, 0x0D00, 1));
        }

        [Fact]
        public void Seed_live_position_rows()
        {
            var (_, bus) = Make();
            ushort[] Row(int n) => bus.ReadRegisters(1, 0x1000 + (16 * n), 16);
            Assert.Equal(new ushort[] { 0, 0, 0, 10, 0, 0x2A62 }, Row(0).Take(6)); // 0.00 mm, 108.50 mm/s
            Assert.Equal(new ushort[] { 0, 0x3A98, 0, 10, 0, 0x6112 }, Row(1).Take(6)); // 150.00 mm, 248.50 mm/s
            Assert.Equal(new ushort[] { 0, 0x1D4C }, Row(2).Take(2)); // 75.00 mm
            Assert.Equal(new ushort[] { 0x1E, 0x1E }, Row(1).Skip(10).Take(2)); // 0.30 G
            Assert.Equal(0xB2, Row(3)[12]);
        }

        [Fact]
        public void Seed_directory()
        {
            var (_, bus) = Make();
            var header = bus.ReadRegisters(1, 0x0000, 8);
            Assert.Equal(new ushort[] { 0x0001, 0x0008, 0x001B, 0x0100, 0x0100 }, header.Take(5));
            var records = bus.ReadRegisters(1, 0x0008, 8 * header[2]);
            var byCi = Enumerable.Range(0, header[2]).ToDictionary(i => (int)records[8 * i], i => records.Skip(8 * i).Take(8).ToArray());
            foreach (var kv in Registers.EXPECTED_CATEGORY_BASES)
            {
                Assert.Equal(kv.Value, byCi[kv.Key][4]);
            }

            Assert.Equal(Registers.EXPECTED_COIL_BASE, byCi[Registers.CATEGORY_IO_CONTROL][6]);
        }

        [Fact]
        public void Seed_embedded_directory_and_backup_file()
        {
            var sim = new SimulatedUnit();
            Assert.Equal("embedded", sim.IndexSource);
            Assert.Equal(new ushort[] { 1, 8, 0x1B }, SimKit.Bus(sim).ReadRegisters(1, 0, 3));
            Assert.Equal("embedded:gripper", new SimulatedUnit(SimProfile.Gripper).IndexSource);
            Assert.Throws<FileNotFoundException>(() => new SimulatedUnit(new SimulatedUnitOptions { BkecPath = "does-not-exist.bkec" }));

            // a PC-tool backup (AAAA,VVVV lines): the directory is read from its first 0x100 lines
            var path = Path.Combine(Path.GetTempPath(), $"ec-sim-{Guid.NewGuid():N}.bkec");
            try
            {
                var bus = SimKit.Bus(sim);
                var words = bus.ReadRegisters(1, 0, 125).Concat(bus.ReadRegisters(1, 125, 99)).ToArray(); // the slider's index table, as the unit serves it (a read takes up to 125 words)
                File.WriteAllLines(path, new[] { "; backup" }.Concat(words.Select((w, i) => $"{i:X4},{w:X4}")).Concat(new[] { "0100,1234" }));
                var regs = SimulatedUnit.LoadBkecRegisters(path);
                Assert.Equal(225, regs.Count);
                Assert.Equal(0x001B, regs[0x0002]);
                Assert.Equal(0x9900, regs[0x00BC]); // CI 0x103 -> DA 0x9900
                var fromFile = new SimulatedUnit(new SimulatedUnitOptions { BkecPath = path });
                Assert.StartsWith("bkec:", fromFile.IndexSource);
                Assert.Equal(SimKit.Bus(sim).ReadRegisters(1, 0, 60), SimKit.Bus(fromFile).ReadRegisters(1, 0, 60));
            }
            finally
            {
                File.Delete(path);
            }
        }

        [Fact]
        public void Seed_report_slave_id()
        {
            var (sim, bus) = Make();
            Assert.Equal(T.Hex("EC FF 02 00 13"), bus.ReportSlaveId(1));
            Assert.Equal(T.Hex("01 11 05 EC FF 02 00 13 C1 5D"), SimKit.Exchange(sim, "11")); // byte for byte the test EC's answer
        }

        // ------------------------------------------------------------------ protocol

        [Fact]
        public void Protocol_exception_codes()
        {
            var sim = new SimulatedUnit();
            Assert.Equal(SimKit.Frame(1, "83 02"), SimKit.Exchange(sim, "03 70 00 00 01"));
            Assert.Equal(SimKit.Frame(1, "83 02"), SimKit.Exchange(sim, "03 90 20 00 04")); // spans the end of the block
            Assert.Equal(SimKit.Frame(1, "83 03"), SimKit.Exchange(sim, "03 90 00 00 00"));
            Assert.Equal(SimKit.Frame(1, "81 01"), SimKit.Exchange(sim, "01 00 00 00 01"));
            Assert.Equal(SimKit.Frame(1, "85 03"), SimKit.Exchange(sim, "05 04 03 12 34"));
            Assert.Equal(SimKit.Frame(1, "85 02"), SimKit.Exchange(sim, "05 04 99 FF 00"));
            Assert.Equal(SimKit.Frame(1, "86 02"), SimKit.Exchange(sim, "06 90 00 00 01")); // read-only
        }

        [Fact]
        public void Protocol_silent_cases()
        {
            var sim = new SimulatedUnit();
            var frame = SimKit.Frame(1, "03 90 00 00 02");
            frame[frame.Length - 1] ^= 0xFF;
            sim.Write(frame);
            Assert.Empty(sim.Read(100)); // bad CRC
            Assert.Empty(SimKit.Exchange(sim, "03 90 00 00 02", slave: 5)); // another slave
            Assert.Empty(SimKit.Exchange(sim, "05 04 03 00 00", slave: 0)); // broadcast: no answer...
            Assert.False(sim.ServoOn); // ...but executed
            Assert.Empty(SimKit.Exchange(sim, "03 90 00 00 02", slave: 0)); // broadcast read ignored
        }

        [Fact]
        public void Protocol_echoes()
        {
            var sim = new SimulatedUnit();
            Assert.Equal(T.Hex("01 05 04 27 FF 00 3D 01"), SimKit.Exchange(sim, "05 04 27 FF 00"));
            Assert.Equal(
                T.Hex("01 10 99 00 00 07 AF 57"),
                SimKit.Exchange(sim, "10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E"));
        }

        [Fact]
        public void Protocol_fault_kinds()
        {
            var sim = new SimulatedUnit();
            var good = SimKit.Exchange(sim, "03 90 05 00 01");
            sim.InjectFault(SimFault.Timeout);
            Assert.Empty(SimKit.Exchange(sim, "03 90 05 00 01"));
            sim.InjectFault(SimFault.Crc);
            Assert.NotEqual(good, SimKit.Exchange(sim, "03 90 05 00 01"));
            sim.InjectFault(SimFault.Partial);
            Assert.True(SimKit.Exchange(sim, "03 90 05 00 01").Length < good.Length);
            sim.InjectFault(SimFault.WrongSlave);
            var foreign = SimKit.Exchange(sim, "03 90 05 00 01");
            Assert.Equal(2, foreign[0]);
            Assert.True(Crc16.Check(foreign));
            Assert.Equal(good, SimKit.Exchange(sim, "03 90 05 00 01"));
            Assert.Throws<ArgumentException>(() => sim.InjectFault((SimFault)99));
        }

        [Fact]
        public void Protocol_closed_unit_is_a_transport_error()
        {
            var (sim, bus) = Make();
            sim.Close();
            Assert.Throws<IOException>(() => sim.Write(SimKit.Frame(1, "03 90 00 00 02")));
            var exc = Assert.Throws<ModbusException>(() => bus.ReadRegisters(1, 0x9000, 2)); // surfaces as a ModbusException
            Assert.IsType<IOException>(exc.InnerException);
        }

        [Fact]
        public void Protocol_pmsl_off()
        {
            var (sim, bus) = Make();
            bus.WriteCoil(1, Registers.COIL_PMSL, false);
            Assert.Equal(0, bus.ReadRegisters(1, Registers.DSSE, 1)[0] & Registers.DSSE_PMSS);
            bus.WriteCoil(1, Registers.COIL_SON, false);
            Assert.True(sim.ServoOn); // other coils ignored (DOC; UNVERIFIED on the EC)
            Assert.Equal(0x3A, bus.IaiCommand(1, 0, 0x09, 0, check: false).Rejection);
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 8000 });
            Assert.True(sim.Moving); // hardware test 18: 0x9900 moves run with PMSS = 0
            bus.WriteCoil(1, Registers.COIL_STOP, true); // hardware test 19: STOP stops a Modbus move with PMSS = 0
            Assert.Contains("stop", sim.Events);
            bus.WriteCoil(1, Registers.COIL_STOP, false);
            Assert.Contains("stop released", sim.Events);
        }

        [Fact]
        public void Protocol_position_table_write_counts_nonvolatile()
        {
            var (sim, bus) = Make();
            bus.WriteRegisters(1, 0x1030, new ushort[] { 0, 1234 });
            Assert.Equal(1, sim.NonvolatileWrites);
            Assert.Equal(new ushort[] { 0, 1234 }, sim.PositionRow(3).Take(2));
        }

        [Fact]
        public void Protocol_iai_rejections()
        {
            var (sim, bus) = Make();
            Assert.Equal(0x21, bus.IaiCommand(1, 0, 0x0D, check: false).Rejection); // not simulated
            Assert.Equal(0x21, bus.IaiCommand(1, 0, 0x05, 2, allowUnsafe: true, check: false).Rejection);
            Assert.Equal(0x21, bus.IaiCommand(1, 0, 0x04, 0x99, allowUnsafe: true, allowNeverSend: true, check: false).Rejection);
            Assert.Equal(0x21, bus.IaiCommand2(1, 0x35, check: false).Rejection);

            // op1 = 1 is the software reset (ON-WIRE): with the servo ON it raises alarm 0x090 (UNVERIFIED for 0x45; VERIFIED for the SFRS coil, test R1)
            Assert.Equal(0, bus.IaiCommand(1, 0, 0x05, 1, allowUnsafe: true, check: false).Rejection);
            Assert.Equal((0x090, false, 0), (sim.AlarmCode, sim.ServoOn, sim.SoftwareResets));
        }

        [Fact]
        public void Protocol_options_are_validated()
        {
            Assert.Throws<ArgumentException>(() => new SimulatedUnit(new SimulatedUnitOptions { Profile = (SimProfile)42 }));
            Assert.Throws<ArgumentException>(() => new SimulatedUnit(new SimulatedUnitOptions { Dopm = (DopmLayout)4 }));
            Assert.Throws<ArgumentOutOfRangeException>(() => new SimulatedUnit(new SimulatedUnitOptions { TimeScale = 0 }));
            Assert.Equal(6, Enum.GetValues(typeof(SimProfile)).Length);
        }

        // ------------------------------------------------------------------ motion model (settling tail off: the pure trapezoid)

        private static void Move(ModbusRtu bus, double mm, double speed, double accelG, double band = 0.1) =>
            bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(mm, band, speed, accelG).Take(7).ToArray());

        [Fact]
        public void Motion_trapezoid()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 83.95, 10.0, 0.1); // 10 mm at 10 mm/s, 0.1 G: about 1.01 s
            clock.Advance(0.5);
            Near(73.95 + 5.0 - 0.051, sim.PositionMm, 0.02);
            var (dss1, dsse) = Bits(bus);
            Assert.NotEqual(0, dsse & Registers.DSSE_MOVE);
            Assert.Equal(0, dss1 & Registers.DSS1_PEND);
            var vnow = bus.ReadRegisters(1, Registers.VNOW, 2);
            Assert.Equal(1248, Registers.FromI32Words(vnow[0], vnow[1])); // 10 mm/s cruise reads 12.48: quantized like the EC
            clock.Advance(0.49);
            Assert.True(sim.Moving);
            clock.Advance(0.03);
            Assert.False(sim.Moving);
            Assert.Equal(new ushort[] { 0, 8395 }, bus.ReadRegisters(1, 0x9000, 2));
            (dss1, dsse) = Bits(bus);
            Assert.NotEqual(0, dss1 & Registers.DSS1_PEND);
            Assert.Equal(0, dsse & Registers.DSSE_MOVE);
        }

        [Fact]
        public void Motion_triangular_and_backwards()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 73.45, 100.0, 0.1); // 0.5 mm: never reaches 100 mm/s
            clock.Advance((2 * Math.Sqrt(0.5 / 980)) + 0.01);
            Assert.False(sim.Moving);
            Assert.Equal(73.45, sim.PositionMm, 6);
        }

        [Fact]
        public void Motion_decel_stop()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 140.0, 50.0, 0.1);
            clock.Advance(0.5);
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            var atStop = sim.PositionMm;
            clock.Advance(1.0);
            Assert.False(sim.Moving);
            Near(atStop + (50.0 * 50.0 / (2 * 980.0)), sim.PositionMm, 0.05);
            Assert.True(sim.PositionMm < 140.0);
        }

        [Fact]
        public void Motion_pause_resume()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 100.0, 20.0, 0.3);
            clock.Advance(0.3);
            bus.WriteCoil(1, Registers.COIL_STP, true);
            clock.Advance(1.0);
            var pausedAt = sim.PositionMm;
            var (dss1, dsse) = Bits(bus);
            Assert.NotEqual(0, dss1 & Registers.DSS1_STP);
            Assert.Equal(0, dsse & Registers.DSSE_MOVE);
            clock.Advance(1.0);
            Assert.Equal(pausedAt, sim.PositionMm);
            bus.WriteCoil(1, Registers.COIL_STP, false);
            clock.Advance(5.0);
            Assert.Equal(100.0, sim.PositionMm);
        }

        [Fact]
        public void Motion_soft_limit_alarm()
        {
            var (sim, bus, _) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 150.5, 10.0, 0.1);
            var words = bus.ReadRegisters(1, 0x9000, 8);
            Assert.Equal(0x0D9, words[2]);
            Assert.NotEqual(0, words[5] & Registers.DSS1_ALMH);
            Assert.False(sim.ServoOn);
            Assert.False(sim.Moving);
            bus.WriteCoil(1, Registers.COIL_ALRS, true);
            bus.WriteCoil(1, Registers.COIL_ALRS, false);
            Assert.Equal(new ushort[] { 0 }, bus.ReadRegisters(1, Registers.ALMC, 1));
        }

        [Theory]
        [InlineData(800.0, 0.1, 0x0AB)]
        [InlineData(10.0, 2.0, 0x0A7)]
        public void Motion_parameter_alarms(double speed, double accelG, int code)
        {
            var (sim, bus, _) = Clocked();
            bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(80.0, 0.1, speed, accelG).Take(7).ToArray());
            Assert.Equal(code, sim.AlarmCode);
        }

        [Fact]
        public void Motion_operation_cancelled_alarm_when_not_homed()
        {
            // 0x083 is "operation cancelled" in the alarm levels (0x080-0x0FF; UNVERIFIED): ALMH, the servo switches off
            var (sim, bus, _) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            sim.Homed = false;
            Move(bus, 80.0, 10.0, 0.1);
            var dss1 = bus.ReadRegisters(1, Registers.DSS1, 1)[0];
            Assert.Equal(0x083, sim.AlarmCode);
            Assert.NotEqual(0, dss1 & Registers.DSS1_ALMH);
            Assert.False(sim.ServoOn);
        }

        [Fact]
        public void Motion_message_level_alarm_keeps_the_axis_running()
        {
            // 0x04E "total moving count exceeded" (a maintenance warning) is message level: ALML, the move continues
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 100.0, 20.0, 0.3);
            clock.Advance(0.5);
            sim.TriggerAlarm(0x04E);
            var dss1 = bus.ReadRegisters(1, Registers.DSS1, 1)[0];
            Assert.True((dss1 & Registers.DSS1_ALML) != 0 && (dss1 & Registers.DSS1_ALMH) == 0);
            Assert.True(sim.ServoOn && sim.Moving);
            clock.Advance(5.0);
            Assert.Equal(100.0, sim.PositionMm);
        }

        [Fact]
        public void Motion_home()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            bus.WriteCoil(1, Registers.COIL_HOME, false);
            bus.WriteCoil(1, Registers.COIL_HOME, true);
            var (dss1, dsse) = Bits(bus);
            Assert.Equal(0, dss1 & Registers.DSS1_HEND);
            Assert.NotEqual(0, dsse & Registers.DSSE_GHMS);
            clock.Advance((73.95 / 20) + 0.5);
            (dss1, dsse) = Bits(bus);
            Assert.NotEqual(0, dss1 & Registers.DSS1_HEND);
            Assert.Equal(0, dsse & Registers.DSSE_GHMS);
            Assert.Equal(0.0, sim.PositionMm);
        }

        [Fact]
        public void Motion_incremental_and_position_moves()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 500, 0, 10, 0, 5000, 30, 0, Registers.CTLF_INC });
            clock.Advance(2.0);
            Assert.Equal(78.95, sim.PositionMm, 6);
            Assert.Equal(new ushort[] { 0 }, bus.ReadRegisters(1, Registers.CTLF, 1)); // CTLF resets after the start
            bus.WriteRegister(1, 0x9800, 2); // immediate move to position 2 (75.00 mm)
            clock.Advance(2.0);
            Assert.Equal(75.0, sim.PositionMm, 6);
            bus.WriteRegister(1, Registers.POSR, 0);
            bus.WriteCoil(1, Registers.COIL_CSTR, true);
            clock.Advance(3.0);
            Assert.Equal(0.0, sim.PositionMm, 6);
        }

        [Fact]
        public void Motion_drg1_register_commands()
        {
            var (sim, bus, _) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            bus.WriteRegister(1, Registers.DRG1, 0x0000);
            Assert.False(sim.ServoOn);
            bus.WriteRegister(1, Registers.DRG1, Registers.DRG1_SON);
            Assert.True(sim.ServoOn);
            bus.WriteRegister(1, Registers.DRG1, Registers.DRG1_SON | Registers.DRG1_HOME);
            Assert.False(sim.Homed);
        }

        [Fact]
        public void Motion_read_write_registers_move()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            var words = bus.ReadWriteRegisters(1, 0x9000, 8, 0x9900, new ushort[] { 0, 7500, 0, 10, 0, 1000, 30 });
            Assert.Equal(8, words.Length);
            clock.Advance(1.0);
            Assert.Equal(75.0, sim.PositionMm, 6);
        }

        [Fact]
        public void Motion_servo_off_stops()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { SettleSpeedMmS = 0.0 });
            Move(bus, 140.0, 50.0, 0.1);
            clock.Advance(0.2);
            bus.IaiCommand(1, 0, 0x09, 0);
            Assert.False(sim.Moving);
            Assert.False(sim.ServoOn);
            bus.IaiCommand2(1, 0x30, new uint[] { 1 });
            Assert.True(sim.ServoOn);
        }

        // ------------------------------------------------------------------ what the hardware tests showed

        private static void Block(ModbusRtu bus, double mm, double band, double speed, double accelG, int ppow = 0, int ctlf = 0) =>
            bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(mm, band, speed, accelG, ppow, ctlf));

        [Fact]
        public void Hardware_stale_ctlf_hazard_exists()
        {
            // why the driver writes all 9 words: a pending CTLF changes the next 7-word move
            var (sim, bus, clock) = Clocked();
            bus.WriteRegister(1, Registers.CTLF, Registers.CTLF_INC);
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 3000, 0, 10, 0, 10000, 30 }); // "absolute 30 mm"
            clock.Advance(3.0);
            Assert.Equal(103.95, sim.PositionMm, 6);
        }

        [Fact]
        public void Hardware_move_bit_clears_during_deceleration()
        {
            var (sim, bus, clock) = Clocked();
            Block(bus, 93.95, 0.1, 100.0, 0.1); // about 0.30 s; decel from about 0.20 s
            clock.Advance(0.1);
            var (dss1, dsse) = Bits(bus);
            Assert.NotEqual(0, dsse & Registers.DSSE_MOVE);
            clock.Advance(0.15);
            (dss1, dsse) = Bits(bus);
            Assert.True(sim.Moving);
            Assert.Equal(0, dsse & Registers.DSSE_MOVE); // cleared while still decelerating...
            Assert.Equal(0, dss1 & Registers.DSS1_PEND); // ...but PEND is not set yet
            clock.Advance(0.1);
            (dss1, _) = Bits(bus);
            Assert.NotEqual(0, dss1 & Registers.DSS1_PEND);
            Assert.Equal(93.95, sim.PositionMm);

            var other = new SimulatedUnit(new SimulatedUnitOptions { Clock = clock.Read, MoveBitClearsInDecel = false });
            var otherBus = SimKit.Bus(other);
            otherBus.WriteRegisters(1, 0x9900, new ushort[] { 0, 9395, 0, 10, 0, 10000, 10, 0, 0 });
            clock.Advance(0.25);
            Assert.NotEqual(0, otherBus.ReadRegisters(1, Registers.DSSE, 1)[0] & Registers.DSSE_MOVE);
        }

        [Fact]
        public void Hardware_stop_latches_until_released()
        {
            var (sim, bus, clock) = Clocked();
            Block(bus, 140.0, 0.1, 50.0, 0.1);
            clock.Advance(0.5);
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            clock.Advance(1.0);
            var stoppedAt = sim.PositionMm;
            Assert.False(sim.Moving);
            Assert.True(sim.StopHeld);
            Block(bus, 80.0, 0.1, 50.0, 0.1); // ignored while STOP is held
            clock.Advance(1.0);
            Assert.Equal(stoppedAt, sim.PositionMm);
            Assert.Contains("move ignored (STOP held)", sim.Events);
            bus.WriteCoil(1, Registers.COIL_STOP, false);
            Block(bus, 80.0, 0.1, 50.0, 0.1);
            clock.Advance(3.0);
            Assert.Equal(80.0, sim.PositionMm);
        }

        [Fact]
        public void Hardware_stop_auto_reset_option()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { StopLatches = false });
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            Assert.False(sim.StopHeld);
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 8000, 0, 10, 0, 5000, 30, 0, 0 });
            clock.Advance(2.0);
            Assert.Equal(80.0, sim.PositionMm);
        }

        [Fact]
        public void Hardware_stp_is_a_level_input()
        {
            var (sim, bus, clock) = Clocked();
            bus.WriteCoil(1, Registers.COIL_STP, true); // paused while idle
            Assert.NotEqual(0, Bits(bus).Dss1 & Registers.DSS1_STP);
            Block(bus, 90.0, 0.1, 50.0, 0.1); // waits while STP is ON
            clock.Advance(2.0);
            Assert.Equal(73.95, sim.PositionMm);
            Assert.True(sim.Moving);
            bus.WriteCoil(1, Registers.COIL_STP, false); // resumes by itself on release (live)
            clock.Advance(2.0);
            Assert.Equal(90.0, sim.PositionMm);
        }

        [Fact]
        public void Hardware_home_phases()
        {
            var (sim, bus, clock) = Clocked();
            bus.WriteCoil(1, Registers.COIL_HOME, false);
            bus.WriteCoil(1, Registers.COIL_HOME, true);
            clock.Advance(1.0);
            var (dss1, dsse) = Bits(bus);
            Assert.Equal(0x3BF2, dsse); // live: homing
            Assert.Equal(0, dss1 & (Registers.DSS1_HEND | Registers.DSS1_PEND));
            clock.Advance(1.6); // the seek to the mechanical end takes about 2.54 s
            Assert.True(sim.PositionMm < -1.0);
            (dss1, dsse) = Bits(bus);
            Assert.Equal(0x39C2, dsse); // live: creep back, MOVE 0 but GHMS 1
            Assert.Equal(0, dss1 & Registers.DSS1_HEND);
            clock.Advance(1.0);
            (dss1, dsse) = Bits(bus);
            Assert.Equal(0x33C2, dsse);
            Assert.Equal(Registers.DSS1_HEND | Registers.DSS1_PEND, dss1 & (Registers.DSS1_HEND | Registers.DSS1_PEND));
            Assert.Equal(0.0, sim.PositionMm);
        }

        [Fact]
        public void Hardware_push_missed()
        {
            var (sim, bus, clock) = Clocked();
            Block(bus, 80.0, 10.0, 10.0, 0.1, ppow: 76, ctlf: Registers.CTLF_PUSH | Registers.CTLF_PUSH_AWAY);
            clock.Advance(0.3);
            Assert.Equal(0, Bits(bus).Dsse & Registers.DSSE_PUSH); // approach
            clock.Advance(0.8);
            Assert.NotEqual(0, Bits(bus).Dsse & Registers.DSSE_PUSH); // pushing beyond 80 mm
            Assert.True(sim.PositionMm > 80.0);
            clock.Advance(1.0);
            var (dss1, _) = Bits(bus);
            Assert.Equal(90.0, sim.PositionMm); // PCMD + band
            Assert.NotEqual(0, dss1 & Registers.DSS1_PSFL);
            Assert.Equal(0, dss1 & Registers.DSS1_PEND);
            Block(bus, 75.0, 0.1, 50.0, 0.1); // the next move clears PSFL
            Assert.Equal(0, Bits(bus).Dss1 & Registers.DSS1_PSFL);
        }

        [Fact]
        public void Hardware_push_contact()
        {
            var (sim, bus, clock) = Clocked();
            sim.ObstacleMm = 85.0;
            Block(bus, 80.0, 20.0, 10.0, 0.1, ppow: 76, ctlf: Registers.CTLF_PUSH | Registers.CTLF_PUSH_AWAY);
            clock.Advance(2.0);
            var (dss1, dsse) = Bits(bus);
            Assert.Equal(85.0, sim.PositionMm);
            Assert.Equal(0x3099, dss1); // live 0x7099 (b14 SFTY aside)
            Assert.Equal(0x37C3, dsse); // live after contact
        }

        [Fact]
        public void Hardware_push_toward_home_and_skipped_approach()
        {
            var (sim, bus, clock) = Clocked();
            Block(bus, 60.0, 10.0, 10.0, 0.1, ppow: 76, ctlf: Registers.CTLF_PUSH); // DIR 0 = toward home
            clock.Advance(4.0);
            Assert.Equal(50.0, sim.PositionMm);
            var past = new SimulatedUnit(new SimulatedUnitOptions { Clock = clock.Read }); // at 73.95, already past PCMD 70 for a + push
            SimKit.Bus(past).WriteRegisters(1, 0x9900, new ushort[] { 0, 7000, 0, 1000, 0, 1000, 10, 76, Registers.CTLF_PUSH | Registers.CTLF_PUSH_AWAY });
            clock.Advance(0.2);
            Assert.True(past.PositionMm > 73.95); // pushes from where it is, no approach back to 70
            clock.Advance(2.0);
            Assert.Equal(80.0, past.PositionMm);
        }

        [Fact]
        public void Hardware_soft_limit_block_and_identification()
        {
            var (sim, bus, clock) = Clocked();
            Assert.Equal(new ushort[] { 0x0000, 0x3AB6, 0xFFFF, 0xFFE2 }, bus.ReadRegisters(1, 0x0700, 4)); // live values
            Assert.Equal("MWB-EC-DSRR4-WA-MF", Registers.WordsToAscii(bus.ReadRegisters(1, 0x0620, 0x28)));
            var shorter = new SimulatedUnit(new SimulatedUnitOptions { StrokeMm = 50.0 });
            Assert.Equal(new ushort[] { 0, 5030 }, SimKit.Bus(shorter).ReadRegisters(1, 0x0700, 2));
            var hidden = new SimulatedUnit(new SimulatedUnitOptions { SoftLimits = false });
            Assert.Equal(SimKit.Frame(1, "83 02"), SimKit.Exchange(hidden, "03 07 00 00 04"));
            Block(bus, 150.2, 0.1, 50.0, 0.1); // inside the unit's own +150.30 limit
            clock.Advance(3.0);
            Assert.Equal(0, sim.AlarmCode);
        }

        [Fact]
        public void Hardware_alarm_history()
        {
            var (sim, bus, _) = Clocked();
            Assert.Equal(
                new ushort[] { 0, 0xFFFF, 0, 0x0FFF, 0xFFFF, 0xFFFF, 3, 0xFFFF, 0, 0x00EE, 0x000E, 0x84DB },
                bus.ReadRegisters(1, 0x0500, 12));
            sim.TriggerAlarm(0x0E0);
            Assert.Equal(0x0E0, bus.ReadRegisters(1, 0x0500, 6)[3]);
            Assert.Equal(0x0FFF, bus.ReadRegisters(1, 0x0506, 6)[3]); // shifted down
            Assert.Equal(new ushort[] { 0, 0xFFFF, 0, 0, 0xFFFF, 0xFFFF }, bus.ReadRegisters(1, 0x05BA, 6));
        }

        [Fact]
        public void Hardware_safety_speed_coil()
        {
            var (_, bus, _) = Clocked();
            bus.WriteCoil(1, Registers.COIL_SFTY, true);
            Assert.NotEqual(0, Bits(bus).Dss1 & Registers.DSS1_SFTY);
            Assert.NotEqual(0, bus.ReadRegisters(1, Registers.DRG1, 1)[0] & Registers.DRG1_SFTY);
        }

        [Fact]
        public void Hardware_fc48_with_16_bit_command()
        {
            var sim = new SimulatedUnit();
            var reply = SimKit.Exchange(sim, "48 00 00 34" + string.Concat(Enumerable.Repeat(" 00", 16))); // ID group 0, decel stop cmd u16 0x0034
            Assert.Equal(T.Hex("48 00 00"), reply.Skip(1).Take(3));
            Assert.Contains("stop", sim.Events);
            var unknown = SimKit.Exchange(sim, "48 00 10 30" + string.Concat(Enumerable.Repeat(" 00", 16))); // 0x1030 is not servo
            Assert.Equal(T.Hex("48 00 21"), unknown.Skip(1).Take(3));
        }

        // ------------------------------------------------------------------ position markers (EC parameter 2: +/-0.10 mm around each row)

        [Fact]
        public void Markers_follow_stored_positions()
        {
            var (sim, bus, clock) = Clocked();
            int Dopm() => bus.ReadRegisters(1, Registers.DOPM, 1)[0];
            int Zons() => bus.ReadRegisters(1, Registers.ZONS, 1)[0];
            Assert.Equal(Registers.DOPM_3POS_NO_ALARM, Dopm()); // 73.95 mm: no marker
            Assert.Equal(0x0003, Zons());

            // hardware test 20 (slider, 3-position mode): DOPM 0x000C / 0x0009 / 0x000A, ZONS 0x4003 / 0x1003 / 0x2003
            foreach (var (target, dopmLive, zonsLive) in new[] { (75.0, 0x000C, 0x4003), (0.0, 0x0009, 0x1003), (150.0, 0x000A, 0x2003) })
            {
                bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(target, 0.10, 100.0, 0.30));
                clock.Advance(0.3);
                if (Math.Abs(sim.PositionMm - target) > 1.0)
                {
                    Assert.Equal(0, Dopm() & 0x7); // mid-stroke: no row nearby, all markers dark
                    Assert.Equal(0, Zons() & 0x7000);
                }

                clock.Advance(30);
                Assert.Equal(dopmLive, Dopm());
                Assert.Equal(zonsLive, Zons());
            }
        }

        [Theory]
        [InlineData(150.10, 0x2003)]
        [InlineData(149.89, 0x0003)]
        [InlineData(-0.10, 0x1003)]
        [InlineData(0.11, 0x0003)]
        [InlineData(74.90, 0x4003)]
        [InlineData(75.11, 0x0003)]
        public void Markers_ls_range_edges(double position, int expected)
        {
            // slider: ZONS 0x2003 at 150.10 and 0x1003 at -0.10 (motion log), dark 0.11 away
            var sim = new SimulatedUnit(new SimulatedUnitOptions { PositionMm = position });
            Assert.Equal(expected, SimKit.Bus(sim).ReadRegisters(1, Registers.ZONS, 1)[0]);
        }

        [Fact]
        public void Markers_lit_while_moving_through_a_row()
        {
            // slider 73.95 -> 76.00 at 10 mm/s: LS2 is lit while passing 74.90..75.10 mm, then dark again
            var (sim, bus, clock) = Clocked();
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 7600, 0, 10, 0, 1000, 10, 0, 0 });
            var litWhileMoving = new List<double>();
            for (var i = 0; i < 400; i++)
            {
                clock.Advance(0.005);
                var pos = sim.PositionMm;
                var zons = bus.ReadRegisters(1, Registers.ZONS, 1)[0];
                Assert.Equal(Math.Abs(Math.Round(pos * 100) - 7500) <= 10, (zons & Registers.ZONS_LS2) != 0);
                if ((zons & Registers.ZONS_LS2) != 0 && sim.Moving)
                {
                    litWhileMoving.Add(pos);
                }
            }

            Assert.NotEmpty(litWhileMoving);
            Assert.False(sim.Moving);
            Assert.Equal(0x0003, bus.ReadRegisters(1, Registers.ZONS, 1)[0]); // 76.00: dark
        }

        // ------------------------------------------------------------------ settling tail: PEND / MEND before standstill

        private static List<(double Pos, int Dss1, int Dsse, bool Moving)> RunMove(SimulatedUnit sim, FakeClock clock, double target)
        {
            var bus = SimKit.Bus(sim);
            bus.WriteRegisters(1, 0x9900, SimKit.MoveBlock(target, 0.10, 10.0, 0.10));
            var samples = new List<(double, int, int, bool)>();
            for (var i = 0; i < 1000; i++)
            {
                clock.Advance(0.002);
                var words = bus.ReadRegisters(1, 0x9000, 8);
                samples.Add((SimKit.Pnow(words), words[5], words[7], sim.Moving));
                if (!sim.Moving)
                {
                    break;
                }
            }

            return samples;
        }

        [Fact]
        public void Settling_pend_and_mend_before_standstill()
        {
            var clock = new FakeClock();
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Profile = SimProfile.Gripper, PositionMm = 3.0, Clock = clock.Read });
            var samples = RunMove(sim, clock, 8.0); // the self-check move: 3.00 -> 8.00 at 10 mm/s
            var early = samples.Where(s => s.Moving && (s.Dss1 & 0x0809) == 0x0009).ToList(); // MEND + PEND, PSFL 0, while still moving
            Assert.NotEmpty(early);
            foreach (var s in early)
            {
                Assert.True(Math.Abs(s.Pos - 8.0) <= 0.10 + 1e-9);
                Assert.Equal(0, s.Dsse & Registers.DSSE_MOVE); // live 0x33C2 in the settling tail
            }

            Assert.Contains(early, s => s.Pos < 7.99);
            var tail = samples.Where(s => s.Moving && s.Pos >= 7.95 && s.Pos < 8.0).ToList();
            Assert.True(tail.Count * 0.002 >= 0.05, "settling tail too short to be seen by a poller");
            Assert.Equal(8.0, samples[samples.Count - 1].Pos);
            Assert.Equal(0x0009, samples[samples.Count - 1].Dss1 & 0x0009);
        }

        [Fact]
        public void Settling_options_off_give_the_pure_trapezoid()
        {
            var clock = new FakeClock();
            var sim = new SimulatedUnit(new SimulatedUnitOptions
            {
                Profile = SimProfile.Gripper, PositionMm = 3.0, Clock = clock.Read, EarlyInPosition = false, SettleSpeedMmS = 0.0,
            });
            var samples = RunMove(sim, clock, 8.0);
            Assert.DoesNotContain(samples, s => s.Moving && (s.Dss1 & Registers.DSS1_PEND) != 0);
            Assert.Equal(8.0, samples[samples.Count - 1].Pos);
        }

        // ------------------------------------------------------------------ hardware test G5: a contact blip, then a miss

        [Fact]
        public void Push_blip_then_miss()
        {
            var clock = new FakeClock();
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Profile = SimProfile.Gripper, PositionMm = 1.0, Clock = clock.Read, PushBlipMm = 6.76 });
            var bus = SimKit.Bus(sim);
            // push + (closing) 30 %, PCMD 2.00, band 5.00, 10 mm/s
            bus.WriteRegisters(1, 0x9900, new ushort[] { 0, 200, 0, 500, 0, 1000, 10, 76, Registers.CTLF_PUSH | Registers.CTLF_PUSH_AWAY });
            var blips = new List<double>();
            for (var i = 0; i < 2000; i++)
            {
                clock.Advance(0.001);
                var words = bus.ReadRegisters(1, 0x9000, 8);
                if ((words[7] & Registers.DSSE_PUSH_CONTACT) != 0)
                {
                    blips.Add(SimKit.Pnow(words));
                    Assert.True(sim.Moving);
                    Assert.Equal(0x0009, words[5] & 0x0809); // PEND + MEND, PSFL 0 (live 0x3099)
                    Assert.Equal(0x37C3, words[7]); // live: PUSH + b0, MOVE 0
                }

                if (!sim.Moving)
                {
                    break;
                }
            }

            Assert.NotEmpty(blips);
            Assert.All(blips, p => Assert.InRange(p, 6.76, 6.86));
            var dss1 = bus.ReadRegisters(1, Registers.DSS1, 1)[0];
            Assert.Equal(7.0, sim.PositionMm);
            Assert.NotEqual(0, dss1 & Registers.DSS1_PSFL);
            Assert.Equal(0, bus.ReadRegisters(1, Registers.DSSE, 1)[0] & Registers.DSSE_PUSH_CONTACT);
        }

        // ------------------------------------------------------------------ PIO test hook (worst case: a held start input restarts after STOP)

        [Fact]
        public void Pio_held_start_input_restarts_after_stop_release()
        {
            var (sim, bus, clock) = Clocked(new SimulatedUnitOptions { Pmsl = false });
            Assert.True(sim.PioMove(120.0, 20.0, holdStart: true));
            clock.Advance(0.5);
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            clock.Advance(1.0);
            Assert.False(sim.Moving);
            var stopped = sim.PositionMm;
            bus.WriteCoil(1, Registers.COIL_STOP, false);
            Assert.True(sim.Moving, "held start input should restart the move");
            Assert.Contains("pio start input still on: move restarted", sim.Events);
            clock.Advance(0.5);
            Assert.True(sim.PositionMm > stopped + 1.0);
            sim.PioRelease();
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            clock.Advance(2.0);
            bus.WriteCoil(1, Registers.COIL_STOP, false);
            Assert.False(sim.Moving);
        }

        [Fact]
        public void Pio_ignored_with_modbus_control()
        {
            var sim = new SimulatedUnit(new SimulatedUnitOptions { Pmsl = true });
            Assert.False(sim.PioMove(120.0));
            Assert.False(sim.Moving);
            sim = new SimulatedUnit(new SimulatedUnitOptions { Pmsl = false });
            sim.PioMove(120.0, holdStart: true);
            var bus = SimKit.Bus(sim);
            bus.WriteCoil(1, Registers.COIL_PMSL, true); // PMSL on: stops the move, PIO inputs ignored
            bus.WriteCoil(1, Registers.COIL_STOP, true);
            bus.WriteCoil(1, Registers.COIL_STOP, false);
            Assert.DoesNotContain("pio start input still on: move restarted", sim.Events);
        }
    }
}
