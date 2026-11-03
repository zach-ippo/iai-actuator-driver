"""SimulatedEC: seeded live values, index table, byte-level protocol and the motion model."""

from __future__ import annotations

import struct
import unittest
from pathlib import Path
from unittest import mock

from ec_testutil import FakeClock, hexbytes

from iai_actuator_driver import registers as R
from iai_actuator_driver import sim as sim_module
from iai_actuator_driver.errors import ModbusError, TransportError
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import DEFAULT_BKEC_PATH, SimulatedEC, load_bkec_registers


def exchange(sim: SimulatedEC, pdu_hex: str, slave: int = 1) -> bytes:
    """Send one raw request and return whatever the simulator answers."""
    sim.reset_input_buffer()
    sim.write(build_frame(slave, hexbytes(pdu_hex)))
    return sim.read(512)


class SeedTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sim = SimulatedEC()
        self.bus = ModbusRTU(self.sim, min_gap_s=0.0)

    def test_live_monitor_values(self) -> None:
        words = self.bus.read_registers(1, 0x9000, 8)
        self.assertEqual(words[0:2], [0x0000, 0x1CE3])  # 73.95 mm
        self.assertEqual(words[2], 0)  # no alarm
        self.assertEqual(words[5], 0x3098)  # DSS1
        self.assertEqual(words[7], 0x33C2)  # DSSE

    def test_live_direct_move_block(self) -> None:
        self.assertEqual(self.bus.read_registers(1, 0x9900, 7), [0, 7395, 0, 10, 0x0001, 0x1170, 30])

    def test_live_drg1_readback(self) -> None:
        self.assertEqual(self.bus.read_registers(1, 0x0D00, 1), [0x1000])

    def test_live_position_rows(self) -> None:
        row = lambda n: self.bus.read_registers(1, 0x1000 + 16 * n, 16)  # noqa: E731
        self.assertEqual(row(0)[:6], [0, 0, 0, 10, 0, 0x2A62])  # 0.00 mm, 108.50 mm/s
        self.assertEqual(row(1)[:6], [0, 0x3A98, 0, 10, 0, 0x6112])  # 150.00 mm, 248.50 mm/s
        self.assertEqual(row(2)[:2], [0, 0x1D4C])  # 75.00 mm
        self.assertEqual(row(1)[10:12], [0x1E, 0x1E])  # 0.30 G
        self.assertEqual(row(3)[12], 0xB2)

    def test_index_table(self) -> None:
        header = self.bus.read_registers(1, 0x0000, 8)
        self.assertEqual(header[:5], [0x0001, 0x0008, 0x001B, 0x0100, 0x0100])
        records = self.bus.read_registers(1, 0x0008, 8 * header[2])
        by_ci = {records[i]: records[i : i + 8] for i in range(0, len(records), 8)}
        for ci, base in R.EXPECTED_CATEGORY_BASES.items():
            self.assertEqual(by_ci[ci][4], base)
        self.assertEqual(by_ci[R.CATEGORY_IO_CONTROL][6], R.EXPECTED_COIL_BASE)

    def test_embedded_fallback(self) -> None:
        with mock.patch.object(sim_module, "DEFAULT_BKEC_PATH", Path("no-such-dir") / "missing.bkec"):
            sim = SimulatedEC(bkec_path=None)
        self.assertEqual(sim.index_source, "embedded")
        self.assertEqual(ModbusRTU(sim).read_registers(1, 0, 3), [1, 8, 0x1B])
        with self.assertRaises(FileNotFoundError):
            SimulatedEC(bkec_path="does-not-exist.bkec")

    def test_report_slave_id(self) -> None:
        self.assertEqual(self.bus.report_slave_id(1), bytes([0xEC, 0xFF, 0x02, 0x00, 0x13]))
        # byte-for-byte the answer of the EC-DS4H slider (VERIFIED)
        self.assertEqual(exchange(self.sim, "11"), hexbytes("01 11 05 EC FF 02 00 13 C1 5D"))


class ProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sim = SimulatedEC()

    def test_exception_codes(self) -> None:
        self.assertEqual(exchange(self.sim, "03 70 00 00 01"), build_frame(1, hexbytes("83 02")))
        self.assertEqual(exchange(self.sim, "03 90 20 00 04"), build_frame(1, hexbytes("83 02")))  # spans end
        self.assertEqual(exchange(self.sim, "03 90 00 00 00"), build_frame(1, hexbytes("83 03")))
        self.assertEqual(exchange(self.sim, "01 00 00 00 01"), build_frame(1, hexbytes("81 01")))
        self.assertEqual(exchange(self.sim, "05 04 03 12 34"), build_frame(1, hexbytes("85 03")))
        self.assertEqual(exchange(self.sim, "05 04 99 FF 00"), build_frame(1, hexbytes("85 02")))
        self.assertEqual(exchange(self.sim, "06 90 00 00 01"), build_frame(1, hexbytes("86 02")))  # read-only

    def test_silent_cases(self) -> None:
        frame = bytearray(build_frame(1, hexbytes("03 90 00 00 02")))
        frame[-1] ^= 0xFF
        self.sim.write(bytes(frame))
        self.assertEqual(self.sim.read(100), b"")  # bad CRC
        self.assertEqual(exchange(self.sim, "03 90 00 00 02", slave=5), b"")  # other slave
        self.assertEqual(exchange(self.sim, "05 04 03 00 00", slave=0), b"")  # broadcast: no answer...
        self.assertFalse(self.sim.servo_on)  # ...but executed
        self.assertEqual(exchange(self.sim, "03 90 00 00 02", slave=0), b"")  # broadcast read ignored

    def test_echoes(self) -> None:
        self.assertEqual(exchange(self.sim, "05 04 27 FF 00"), hexbytes("01 05 04 27 FF 00 3D 01"))
        self.assertEqual(
            exchange(self.sim, "10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E"),
            hexbytes("01 10 99 00 00 07 AF 57"),
        )

    def test_fault_kinds(self) -> None:
        good = exchange(self.sim, "03 90 05 00 01")
        self.sim.inject_fault("timeout")
        self.assertEqual(exchange(self.sim, "03 90 05 00 01"), b"")
        self.sim.inject_fault("crc")
        self.assertNotEqual(exchange(self.sim, "03 90 05 00 01"), good)
        self.sim.inject_fault("partial")
        self.assertLess(len(exchange(self.sim, "03 90 05 00 01")), len(good))
        self.assertEqual(exchange(self.sim, "03 90 05 00 01"), good)
        with self.assertRaises(ValueError):
            self.sim.inject_fault("smoke")

    def test_closed(self) -> None:
        bus = ModbusRTU(self.sim, min_gap_s=0.0)
        self.sim.close()
        with self.assertRaises(OSError):
            self.sim.write(build_frame(1, hexbytes("03 90 00 00 02")))
        with self.assertRaises(ModbusError) as ctx:  # transport errors surface as ModbusError
            bus.read_registers(1, 0x9000, 2)
        self.assertIsInstance(ctx.exception.__cause__, OSError)

    def test_pmsl_off(self) -> None:
        bus = ModbusRTU(self.sim, min_gap_s=0.0)
        bus.write_coil(1, R.COIL_PMSL, False)
        self.assertEqual(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PMSS, 0)
        bus.write_coil(1, R.COIL_SON, False)
        self.assertTrue(self.sim.servo_on)  # other coils ignored (ME0162; unverified on the EC)
        self.assertEqual(bus.iai_command(1, 0, 0x09, 0, check=False).rejection, 0x3A)
        bus.write_registers(1, 0x9900, [0, 8000])
        self.assertTrue(self.sim.moving)  # as on the EC-DS4H slider: 0x9900 moves run with PMSS = 0
        bus.write_coil(1, R.COIL_STOP, True)  # as on the slider: STOP stops a Modbus move with PMSS = 0
        self.assertIn("stop", self.sim.events)
        bus.write_coil(1, R.COIL_STOP, False)
        self.assertIn("stop released", self.sim.events)

    def test_position_table_write_counts_nonvolatile(self) -> None:
        bus = ModbusRTU(self.sim, min_gap_s=0.0)
        bus.write_registers(1, 0x1030, [0, 1234])
        self.assertEqual(self.sim.nonvolatile_writes, 1)
        self.assertEqual(self.sim.position_row(3)[:2], [0, 1234])

    def test_iai_rejections(self) -> None:
        bus = ModbusRTU(self.sim, min_gap_s=0.0)
        self.assertEqual(bus.iai_command(1, 0, 0x0D, check=False).rejection, 0x21)  # not simulated
        self.assertEqual(bus.iai_command(1, 0, 0x05, 2, allow_unsafe=True, check=False).rejection, 0x21)
        self.assertEqual(
            bus.iai_command(1, 0, 0x04, 0x99, allow_unsafe=True, allow_never_send=True, check=False).rejection, 0x21
        )
        self.assertEqual(bus.iai_command2(1, 0x35, check=False).rejection, 0x21)
        # op1 = 1 is the software reset (ON-WIRE): with the servo ON it raises alarm 0x090 (UNVERIFIED for 0x45;
        # VERIFIED for the SFRS coil on the EC-GRBP8M gripper).
        self.assertEqual(bus.iai_command(1, 0, 0x05, 1, allow_unsafe=True, check=False).rejection, 0)
        self.assertEqual((self.sim.alarm_code, self.sim.servo_on, self.sim.software_resets), (0x090, False, 0))


class MotionModelTest(unittest.TestCase):
    """The pure trapezoid (settling tail switched off; see SettlingTailTest for the default)."""

    def setUp(self) -> None:
        self.clock = FakeClock()
        self.sim = SimulatedEC(clock=self.clock, settle_speed_mm_s=0.0)
        self.bus = ModbusRTU(self.sim, min_gap_s=0.0)

    def move(self, mm: float, speed: float, accel_g: float, band: float = 0.1) -> None:
        words = [*R.to_i32_words(round(mm * 100)), *R.to_u32_words(round(band * 100))]
        words += [*R.to_u32_words(round(speed * 100)), round(accel_g * 100)]
        self.bus.write_registers(1, 0x9900, words)

    def bits(self) -> tuple[int, int]:
        words = self.bus.read_registers(1, 0x9000, 8)
        return words[5], words[7]

    def test_trapezoid(self) -> None:
        self.move(83.95, 10.0, 0.1)  # 10 mm at 10 mm/s, 0.1 G: ~1.01 s
        self.clock.advance(0.5)
        pos = self.sim.position_mm
        self.assertAlmostEqual(pos, 73.95 + 5.0 - 0.051, delta=0.02)
        dss1, dsse = self.bits()
        self.assertTrue(dsse & R.DSSE_MOVE)
        self.assertFalse(dss1 & R.DSS1_PEND)
        vnow = self.bus.read_registers(1, R.VNOW, 2)
        self.assertEqual(R.from_i32_words(*vnow), 1248)  # 10 mm/s cruise reads 12.48: quantized like the EC
        self.clock.advance(0.49)
        self.assertTrue(self.sim.moving)
        self.clock.advance(0.03)
        self.assertFalse(self.sim.moving)
        self.assertEqual(self.bus.read_registers(1, 0x9000, 2), [0, 8395])
        dss1, dsse = self.bits()
        self.assertTrue(dss1 & R.DSS1_PEND)
        self.assertFalse(dsse & R.DSSE_MOVE)

    def test_triangular_and_backwards(self) -> None:
        self.move(73.45, 100.0, 0.1)  # 0.5 mm: never reaches 100 mm/s
        self.clock.advance(2 * (0.5 / 980) ** 0.5 + 0.01)
        self.assertFalse(self.sim.moving)
        self.assertAlmostEqual(self.sim.position_mm, 73.45, places=6)

    def test_decel_stop(self) -> None:
        self.move(140.0, 50.0, 0.1)
        self.clock.advance(0.5)
        self.bus.write_coil(1, R.COIL_STOP, True)
        at_stop = self.sim.position_mm
        self.clock.advance(1.0)
        self.assertFalse(self.sim.moving)
        stop_distance = 50.0**2 / (2 * 980.0)
        self.assertAlmostEqual(self.sim.position_mm, at_stop + stop_distance, delta=0.05)
        self.assertLess(self.sim.position_mm, 140.0)

    def test_pause_resume(self) -> None:
        self.move(100.0, 20.0, 0.3)
        self.clock.advance(0.3)
        self.bus.write_coil(1, R.COIL_STP, True)
        self.clock.advance(1.0)
        paused_at = self.sim.position_mm
        dss1, dsse = self.bits()
        self.assertTrue(dss1 & R.DSS1_STP)
        self.assertFalse(dsse & R.DSSE_MOVE)
        self.clock.advance(1.0)
        self.assertEqual(self.sim.position_mm, paused_at)
        self.bus.write_coil(1, R.COIL_STP, False)
        self.clock.advance(5.0)
        self.assertEqual(self.sim.position_mm, 100.0)

    def test_soft_limit_alarm(self) -> None:
        self.move(150.5, 10.0, 0.1)
        words = self.bus.read_registers(1, 0x9000, 8)
        self.assertEqual(words[2], 0x0D9)
        self.assertTrue(words[5] & R.DSS1_ALMH)
        self.assertFalse(self.sim.servo_on)
        self.assertFalse(self.sim.moving)
        self.bus.write_coil(1, R.COIL_ALRS, True)
        self.bus.write_coil(1, R.COIL_ALRS, False)
        self.assertEqual(self.bus.read_registers(1, R.ALMC, 1), [0])

    def test_parameter_alarms(self) -> None:
        for kwargs, code in (({"speed": 800.0, "accel_g": 0.1}, 0x0AB), ({"speed": 10.0, "accel_g": 2.0}, 0x0A7)):
            with self.subTest(code=code):
                sim = SimulatedEC(clock=self.clock)
                bus = ModbusRTU(sim, min_gap_s=0.0)
                words = [0, 8000, 0, 10, *R.to_u32_words(round(kwargs["speed"] * 100)), round(kwargs["accel_g"] * 100)]
                bus.write_registers(1, 0x9900, words)
                self.assertEqual(sim.alarm_code, code)

    def test_operation_cancelled_alarm_when_not_homed(self) -> None:
        # 0x083 is "operation cancelled" in the alarm levels (0x080-0x0FF; UNVERIFIED): ALMH, the servo switches off
        self.sim.homed = False
        self.move(80.0, 10.0, 0.1)
        dss1 = self.bus.read_registers(1, R.DSS1, 1)[0]
        self.assertEqual(self.sim.alarm_code, 0x083)
        self.assertTrue(dss1 & R.DSS1_ALMH)
        self.assertFalse(self.sim.servo_on)

    def test_message_level_alarm_keeps_the_axis_running(self) -> None:
        # 0x04E "total moving count exceeded" (a maintenance warning) is message level: ALML, the move continues
        self.move(100.0, 20.0, 0.3)
        self.clock.advance(0.5)
        self.sim.trigger_alarm(0x04E)
        dss1 = self.bus.read_registers(1, R.DSS1, 1)[0]
        self.assertTrue(dss1 & R.DSS1_ALML and not dss1 & R.DSS1_ALMH)
        self.assertTrue(self.sim.servo_on and self.sim.moving)
        self.clock.advance(5.0)
        self.assertEqual(self.sim.position_mm, 100.0)

    def test_home(self) -> None:
        self.bus.write_coil(1, R.COIL_HOME, False)
        self.bus.write_coil(1, R.COIL_HOME, True)
        dss1, dsse = self.bits()
        self.assertFalse(dss1 & R.DSS1_HEND)
        self.assertTrue(dsse & R.DSSE_GHMS)
        self.clock.advance(73.95 / 20 + 0.5)
        dss1, dsse = self.bits()
        self.assertTrue(dss1 & R.DSS1_HEND)
        self.assertFalse(dsse & R.DSSE_GHMS)
        self.assertEqual(self.sim.position_mm, 0.0)

    def test_incremental_and_position_moves(self) -> None:
        self.bus.write_registers(1, 0x9900, [0, 500, 0, 10, 0, 5000, 30, 0, R.CTLF_INC])
        self.clock.advance(2.0)
        self.assertAlmostEqual(self.sim.position_mm, 78.95, places=6)
        self.assertEqual(self.bus.read_registers(1, R.CTLF, 1), [0])  # CTLF resets after start
        self.bus.write_register(1, 0x9800, 2)  # immediate move to position 2 (75.00 mm)
        self.clock.advance(2.0)
        self.assertAlmostEqual(self.sim.position_mm, 75.0, places=6)
        self.bus.write_register(1, R.POSR, 0)
        self.bus.write_coil(1, R.COIL_CSTR, True)
        self.clock.advance(3.0)
        self.assertAlmostEqual(self.sim.position_mm, 0.0, places=6)

    def test_drg1_register_commands(self) -> None:
        self.bus.write_register(1, R.DRG1, 0x0000)
        self.assertFalse(self.sim.servo_on)
        self.bus.write_register(1, R.DRG1, R.DRG1_SON)
        self.assertTrue(self.sim.servo_on)
        self.bus.write_register(1, R.DRG1, R.DRG1_SON | R.DRG1_HOME)
        self.assertFalse(self.sim.homed)

    def test_read_write_registers_move(self) -> None:
        words = self.bus.read_write_registers(1, 0x9000, 8, 0x9900, [0, 7500, 0, 10, 0, 1000, 30])
        self.assertEqual(len(words), 8)
        self.clock.advance(1.0)
        self.assertAlmostEqual(self.sim.position_mm, 75.0, places=6)

    def test_servo_off_stops(self) -> None:
        self.move(140.0, 50.0, 0.1)
        self.clock.advance(0.2)
        self.bus.iai_command(1, 0, 0x09, 0)
        self.assertFalse(self.sim.moving)
        self.assertFalse(self.sim.servo_on)
        self.bus.iai_command2(1, 0x30, (1,))
        self.assertTrue(self.sim.servo_on)


class HardwareFindingsModelTest(unittest.TestCase):
    """The simulator mimics the behaviour VERIFIED on hardware (FakeClock, raw frames)."""

    def setUp(self) -> None:
        self.clock = FakeClock()
        self.sim = SimulatedEC(clock=self.clock)
        self.bus = ModbusRTU(self.sim, min_gap_s=0.0)

    def block(self, mm: float, band: float, speed: float, accel_g: float, ppow: int = 0, ctlf: int = 0) -> None:
        words = [*R.to_i32_words(round(mm * 100)), *R.to_u32_words(round(band * 100))]
        words += [*R.to_u32_words(round(speed * 100)), round(accel_g * 100), ppow, ctlf]
        self.bus.write_registers(1, 0x9900, words)

    def bits(self) -> tuple[int, int]:
        words = self.bus.read_registers(1, 0x9000, 8)
        return words[5], words[7]

    def test_stale_ctlf_hazard_exists(self) -> None:
        # Why the driver writes all 9 words: a pending CTLF changes the next 7-word move.
        self.bus.write_register(1, R.CTLF, R.CTLF_INC)
        self.bus.write_registers(1, 0x9900, [0, 3000, 0, 10, 0, 10000, 30])  # "absolute 30 mm"
        self.clock.advance(3.0)
        self.assertAlmostEqual(self.sim.position_mm, 103.95, places=6)

    def test_move_bit_clears_during_deceleration(self) -> None:
        self.block(93.95, 0.1, 100.0, 0.1)  # ~0.30 s; decel from ~0.20 s
        self.clock.advance(0.1)
        dss1, dsse = self.bits()
        self.assertTrue(dsse & R.DSSE_MOVE)
        self.clock.advance(0.15)
        dss1, dsse = self.bits()
        self.assertTrue(self.sim.moving)
        self.assertFalse(dsse & R.DSSE_MOVE)  # cleared while still decelerating...
        self.assertFalse(dss1 & R.DSS1_PEND)  # ...but PEND is not set yet
        self.clock.advance(0.1)
        dss1, dsse = self.bits()
        self.assertTrue(dss1 & R.DSS1_PEND)
        self.assertEqual(self.sim.position_mm, 93.95)
        sim = SimulatedEC(clock=self.clock, move_bit_clears_in_decel=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x9900, [0, 9395, 0, 10, 0, 10000, 10, 0, 0])
        self.clock.advance(0.25)
        self.assertTrue(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_MOVE)

    def test_stop_latches_until_released(self) -> None:
        self.block(140.0, 0.1, 50.0, 0.1)
        self.clock.advance(0.5)
        self.bus.write_coil(1, R.COIL_STOP, True)
        self.clock.advance(1.0)
        stopped_at = self.sim.position_mm
        self.assertFalse(self.sim.moving)
        self.assertTrue(self.sim.stop_held)
        self.block(80.0, 0.1, 50.0, 0.1)  # ignored while STOP is held
        self.clock.advance(1.0)
        self.assertEqual(self.sim.position_mm, stopped_at)
        self.assertIn("move ignored (STOP held)", self.sim.events)
        self.bus.write_coil(1, R.COIL_STOP, False)
        self.block(80.0, 0.1, 50.0, 0.1)
        self.clock.advance(3.0)
        self.assertEqual(self.sim.position_mm, 80.0)

    def test_stop_auto_reset_option(self) -> None:
        sim = SimulatedEC(clock=self.clock, stop_latches=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_coil(1, R.COIL_STOP, True)
        self.assertFalse(sim.stop_held)
        bus.write_registers(1, 0x9900, [0, 8000, 0, 10, 0, 5000, 30, 0, 0])
        self.clock.advance(2.0)
        self.assertEqual(sim.position_mm, 80.0)

    def test_stp_is_a_level_input(self) -> None:
        self.bus.write_coil(1, R.COIL_STP, True)  # paused while idle
        dss1, _ = self.bits()
        self.assertTrue(dss1 & R.DSS1_STP)
        self.block(90.0, 0.1, 50.0, 0.1)  # waits while STP is ON
        self.clock.advance(2.0)
        self.assertEqual(self.sim.position_mm, 73.95)
        self.assertTrue(self.sim.moving)
        self.bus.write_coil(1, R.COIL_STP, False)  # auto-resumes on release (live)
        self.clock.advance(2.0)
        self.assertEqual(self.sim.position_mm, 90.0)

    def test_home_phases(self) -> None:
        self.bus.write_coil(1, R.COIL_HOME, False)
        self.bus.write_coil(1, R.COIL_HOME, True)
        self.clock.advance(1.0)
        dss1, dsse = self.bits()
        self.assertEqual(dsse, 0x3BF2)  # live: homing
        self.assertFalse(dss1 & (R.DSS1_HEND | R.DSS1_PEND))
        self.clock.advance(1.6)  # seek to the mechanical end takes ~2.54 s
        self.assertLess(self.sim.position_mm, -1.0)
        dss1, dsse = self.bits()
        self.assertEqual(dsse, 0x39C2)  # live: creep back, MOVE 0 but GHMS 1
        self.assertFalse(dss1 & R.DSS1_HEND)
        self.clock.advance(1.0)
        dss1, dsse = self.bits()
        self.assertEqual(dsse, 0x33C2)
        self.assertEqual(dss1 & (R.DSS1_HEND | R.DSS1_PEND), R.DSS1_HEND | R.DSS1_PEND)
        self.assertEqual(self.sim.position_mm, 0.0)

    def test_push_missed(self) -> None:
        self.block(80.0, 10.0, 10.0, 0.1, ppow=76, ctlf=R.CTLF_PUSH | R.CTLF_PUSH_AWAY)
        self.clock.advance(0.3)
        _, dsse = self.bits()
        self.assertFalse(dsse & R.DSSE_PUSH)  # approach
        self.clock.advance(0.8)
        _, dsse = self.bits()
        self.assertTrue(dsse & R.DSSE_PUSH)  # pushing beyond 80 mm
        self.assertGreater(self.sim.position_mm, 80.0)
        self.clock.advance(1.0)
        dss1, dsse = self.bits()
        self.assertEqual(self.sim.position_mm, 90.0)  # PCMD + band
        self.assertTrue(dss1 & R.DSS1_PSFL)
        self.assertFalse(dss1 & R.DSS1_PEND)
        self.block(75.0, 0.1, 50.0, 0.1)  # the next move clears PSFL
        self.assertFalse(self.bits()[0] & R.DSS1_PSFL)

    def test_push_contact(self) -> None:
        self.sim.obstacle_mm = 85.0
        self.block(80.0, 20.0, 10.0, 0.1, ppow=76, ctlf=R.CTLF_PUSH | R.CTLF_PUSH_AWAY)
        self.clock.advance(2.0)
        dss1, dsse = self.bits()
        self.assertEqual(self.sim.position_mm, 85.0)
        self.assertEqual(dss1, 0x3099)  # live 0x7099 (b14 SFTY aside)
        self.assertEqual(dsse, 0x37C3)  # live after contact

    def test_push_toward_home_and_skipped_approach(self) -> None:
        self.block(60.0, 10.0, 10.0, 0.1, ppow=76, ctlf=R.CTLF_PUSH)  # DIR 0 = toward home
        self.clock.advance(4.0)
        self.assertEqual(self.sim.position_mm, 50.0)
        sim = SimulatedEC(clock=self.clock)  # at 73.95, already past PCMD 70 for a + push
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x9900, [0, 7000, 0, 1000, 0, 1000, 10, 76, R.CTLF_PUSH | R.CTLF_PUSH_AWAY])
        self.clock.advance(0.2)
        self.assertGreater(sim.position_mm, 73.95)  # pushes from where it is, no approach back to 70
        self.clock.advance(2.0)
        self.assertEqual(sim.position_mm, 80.0)

    def test_soft_limit_block_and_identification(self) -> None:
        self.assertEqual(self.bus.read_registers(1, 0x0700, 4), [0x0000, 0x3AB6, 0xFFFF, 0xFFE2])  # live values
        self.assertEqual(R.words_to_ascii(self.bus.read_registers(1, 0x0620, 0x28)), "MWB-EC-DSRR4-WA-MF")
        short = SimulatedEC(stroke_mm=50.0)
        self.assertEqual(ModbusRTU(short).read_registers(1, 0x0700, 2), [0, 5030])
        hidden = SimulatedEC(soft_limits=False)
        self.assertEqual(exchange(hidden, "03 07 00 00 04"), build_frame(1, hexbytes("83 02")))
        self.block(150.2, 0.1, 50.0, 0.1)  # inside the EC's own +150.30 limit
        self.clock.advance(3.0)
        self.assertEqual(self.sim.alarm_code, 0)

    def test_alarm_history(self) -> None:
        first = self.bus.read_registers(1, 0x0500, 12)
        self.assertEqual(first, [0, 0xFFFF, 0, 0x0FFF, 0xFFFF, 0xFFFF, 3, 0xFFFF, 0, 0x00EE, 0x000E, 0x84DB])
        self.sim.trigger_alarm(0x0E0)
        newest = self.bus.read_registers(1, 0x0500, 6)
        self.assertEqual(newest[3], 0x0E0)
        self.assertEqual(self.bus.read_registers(1, 0x0506, 6)[3], 0x0FFF)  # shifted down
        self.assertEqual(self.bus.read_registers(1, 0x05BA, 6), [0, 0xFFFF, 0, 0, 0xFFFF, 0xFFFF])

    def test_safety_speed_coil(self) -> None:
        self.bus.write_coil(1, R.COIL_SFTY, True)
        self.assertTrue(self.bits()[0] & R.DSS1_SFTY)
        self.assertTrue(self.bus.read_registers(1, R.DRG1, 1)[0] & R.DRG1_SFTY)

    def test_fc48_with_16_bit_command(self) -> None:
        reply = exchange(self.sim, "48 00 00 34" + " 00" * 16)  # ID group 0, decel stop cmd u16 0x0034
        self.assertEqual(reply[1:4], hexbytes("48 00 00"))
        self.assertIn("stop", self.sim.events)
        unknown = exchange(self.sim, "48 00 10 30" + " 00" * 16)  # 0x1030 is not servo
        self.assertEqual(unknown[1:4], hexbytes("48 00 21"))


class LoadBkecTest(unittest.TestCase):
    @unittest.skipUnless(DEFAULT_BKEC_PATH.is_file(), "baseline backup not present")
    def test_first_lines(self) -> None:
        regs = load_bkec_registers(DEFAULT_BKEC_PATH)
        self.assertEqual(len(regs), 0x100)
        self.assertEqual(regs[0x0002], 0x001B)
        self.assertEqual(struct.pack(">H", regs[0x00BC]), b"\x99\x00")  # CI 0x103 -> DA 0x9900


class PositionMarkerTest(unittest.TestCase):
    """Markers follow the position alone (EC parameter 2: +/-0.10 mm around each row), moving or not."""

    def test_markers_follow_stored_positions(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(clock=clock)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        dopm = lambda: bus.read_registers(1, R.DOPM, 1)[0]  # noqa: E731
        zons = lambda: bus.read_registers(1, R.ZONS, 1)[0]  # noqa: E731
        self.assertEqual(dopm(), R.DOPM_3POS_NO_ALARM)  # 73.95 mm: no marker
        self.assertEqual(zons(), 0x0003)
        # EC-DS4H slider (3-position mode, VERIFIED): DOPM 0x000C / 0x0009 / 0x000A, ZONS 0x4003 / 0x1003 / 0x2003.
        for target, dopm_live, zons_live in ((75.0, 0x000C, 0x4003), (0.0, 0x0009, 0x1003), (150.0, 0x000A, 0x2003)):
            bus.write_registers(1, 0x9900, [*R.to_i32_words(round(target * 100)), 0, 10, 0, 10000, 30, 0, 0])
            clock.advance(0.3)
            if abs(sim.position_mm - target) > 1.0:  # mid-stroke: no row nearby, all markers dark
                self.assertEqual(dopm() & 0x7, 0, sim.position_mm)
                self.assertEqual(zons() & 0x7000, 0, sim.position_mm)
            clock.advance(30)
            self.assertEqual(dopm(), dopm_live, target)
            self.assertEqual(zons(), zons_live, target)

    def test_ls_range_edges(self) -> None:
        """Slider: ZONS 0x2003 at 150.10 and 0x1003 at -0.10 (the soft-limit clamp, VERIFIED), dark at 0.11 away."""
        for pos, expected in ((150.10, 0x2003), (149.89, 0x0003), (-0.10, 0x1003), (0.11, 0x0003),
                              (74.90, 0x4003), (75.11, 0x0003)):
            sim = SimulatedEC(position_mm=pos)
            self.assertEqual(ModbusRTU(sim).read_registers(1, R.ZONS, 1)[0], expected, pos)

    def test_markers_lit_while_moving_through_a_row(self) -> None:
        """Slider 73.95 -> 76.00 at 10 mm/s: LS2 is lit while passing 74.90..75.10 mm, then dark again."""
        clock = FakeClock()
        sim = SimulatedEC(clock=clock)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x9900, [0, 7600, 0, 10, 0, 1000, 10, 0, 0])
        lit_while_moving = []
        for _ in range(400):
            clock.advance(0.005)
            pos, zons = sim.position_mm, bus.read_registers(1, R.ZONS, 1)[0]
            self.assertEqual(bool(zons & R.ZONS_LS2), abs(round(pos * 100) - 7500) <= 10, pos)
            if zons & R.ZONS_LS2 and sim.moving:
                lit_while_moving.append(pos)
        self.assertTrue(lit_while_moving)
        self.assertFalse(sim.moving)
        self.assertEqual(bus.read_registers(1, R.ZONS, 1)[0], 0x0003)  # 76.00: dark


class SettlingTailTest(unittest.TestCase):
    """PEND / MEND rise inside the band before standstill (gripper 3.04 -> 3.00 mm with DSS1 0x3099)."""

    def run_move(self, sim: SimulatedEC, clock: FakeClock, target: float) -> list[tuple[float, int, int, bool]]:
        bus = ModbusRTU(sim, min_gap_s=0.0)
        bus.write_registers(1, 0x9900, [*R.to_i32_words(round(target * 100)), 0, 10, 0, 1000, 10, 0, 0])
        samples = []
        for _ in range(1000):
            clock.advance(0.002)
            words = bus.read_registers(1, 0x9000, 8)
            samples.append((R.from_i32_words(words[0], words[1]) / 100, words[5], words[7], sim.moving))
            if not sim.moving:
                break
        return samples

    def test_pend_and_mend_before_standstill(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(profile="gripper", position_mm=3.0, clock=clock)
        samples = self.run_move(sim, clock, 8.0)  # the iai-check move: 3.00 -> 8.00 at 10 mm/s
        early = [(pos, dss1, dsse) for pos, dss1, dsse, moving in samples
                 if moving and dss1 & 0x0809 == 0x0009]  # MEND + PEND, PSFL 0, while still moving
        self.assertTrue(early, "no in-band PEND / MEND sample while moving")
        for pos, _dss1, dsse in early:
            self.assertLessEqual(abs(pos - 8.0), 0.10 + 1e-9)
            self.assertFalse(dsse & R.DSSE_MOVE)  # live 0x33C2 in the settling tail
        self.assertTrue(any(pos < 7.99 for pos, _, _ in early), early)
        tail = [s for s in samples if s[3] and 7.95 <= s[0] < 8.0]
        self.assertGreaterEqual(len(tail) * 0.002, 0.05, "settling tail too short to be seen by a poller")
        self.assertEqual(samples[-1][0], 8.0)
        self.assertEqual(samples[-1][1] & 0x0009, 0x0009)

    def test_options_off_give_the_pure_trapezoid(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(profile="gripper", position_mm=3.0, clock=clock, early_in_position=False,
                          settle_speed_mm_s=0.0)
        samples = self.run_move(sim, clock, 8.0)
        self.assertFalse([s for s in samples if s[3] and s[1] & R.DSS1_PEND])
        self.assertEqual(samples[-1][0], 8.0)


class PushBlipTest(unittest.TestCase):
    """As on the EC-GRBP8M gripper: the contact fingerprint for a moment, then a miss at the band end."""

    def test_blip_then_miss(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(profile="gripper", position_mm=1.0, clock=clock, push_blip_mm=6.76)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        # push + (closing) 30 %, PCMD 2.00, band 5.00, 10 mm/s
        bus.write_registers(1, 0x9900, [0, 200, 0, 500, 0, 1000, 10, 76, R.CTLF_PUSH | R.CTLF_PUSH_AWAY])
        blips = []
        for _ in range(2000):
            clock.advance(0.001)
            words = bus.read_registers(1, 0x9000, 8)
            pos, dss1, dsse = R.from_i32_words(words[0], words[1]) / 100, words[5], words[7]
            if dsse & R.DSSE_PUSH_CONTACT:
                blips.append(pos)
                self.assertTrue(sim.moving)
                self.assertEqual(dss1 & 0x0809, 0x0009)  # PEND + MEND, PSFL 0 (live 0x3099)
                self.assertEqual(dsse, 0x37C3)  # live: PUSH + b0, MOVE 0
            if not sim.moving:
                break
        self.assertTrue(blips)
        self.assertTrue(all(6.76 <= p <= 6.86 for p in blips), blips)
        dss1 = bus.read_registers(1, R.DSS1, 1)[0]
        self.assertEqual(sim.position_mm, 7.0)
        self.assertTrue(dss1 & R.DSS1_PSFL)
        self.assertFalse(bus.read_registers(1, R.DSSE, 1)[0] & R.DSSE_PUSH_CONTACT)


class PioMoveTest(unittest.TestCase):
    """Test hook for a PLC-commanded move (worst case: a held start input restarts it after STOP)."""

    def test_held_start_input_restarts_after_stop_release(self) -> None:
        clock = FakeClock()
        sim = SimulatedEC(clock=clock, pmsl=False)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        self.assertTrue(sim.pio_move(120.0, 20.0, hold_start=True))
        clock.advance(0.5)
        bus.write_coil(1, R.COIL_STOP, True)
        clock.advance(1.0)
        self.assertFalse(sim.moving)
        stopped = sim.position_mm
        bus.write_coil(1, R.COIL_STOP, False)
        self.assertTrue(sim.moving, "held start input should restart the move")
        self.assertIn("pio start input still on: move restarted", sim.events)
        clock.advance(0.5)
        self.assertGreater(sim.position_mm, stopped + 1.0)
        sim.pio_release()
        bus.write_coil(1, R.COIL_STOP, True)
        clock.advance(2.0)
        bus.write_coil(1, R.COIL_STOP, False)
        self.assertFalse(sim.moving)

    def test_ignored_with_modbus_control(self) -> None:
        sim = SimulatedEC(pmsl=True)
        self.assertFalse(sim.pio_move(120.0))
        self.assertFalse(sim.moving)
        sim = SimulatedEC(pmsl=False)
        sim.pio_move(120.0, hold_start=True)
        ModbusRTU(sim).write_coil(1, R.COIL_PMSL, True)  # PMSL on: stops the move, PIO inputs ignored
        bus = ModbusRTU(sim)
        bus.write_coil(1, R.COIL_STOP, True)
        bus.write_coil(1, R.COIL_STOP, False)
        self.assertNotIn("pio start input still on: move restarted", sim.events)


class OnRequestRaisesTest(unittest.TestCase):
    """An exception raised in ``on_request`` fails the request, as on a port that fails: an OSError becomes the cause
    of the bus's TransportError, anything else (a test's assert) reaches the caller as it is."""

    class Failing(SimulatedEC):
        error: BaseException | None = None

        def on_request(self, frame: bytes) -> bytes | None:
            if self.error is not None and frame[1] == 0x03:
                raise self.error
            return NotImplemented  # type: ignore[return-value]

    def test_an_assertion_reaches_the_caller(self) -> None:
        sim = self.Failing()
        bus = ModbusRTU(sim, min_gap_s=0.0)
        sim.error = AssertionError("this frame must never be sent")
        with self.assertRaises(AssertionError) as caught:
            bus.read_registers(1, R.PNOW, 2)
        self.assertIn("must never be sent", str(caught.exception))
        sim.error = None
        self.assertEqual(len(bus.read_registers(1, R.PNOW, 2)), 2)  # the bus goes on

    def test_an_os_error_is_the_cause_of_a_transport_error(self) -> None:
        sim = self.Failing()
        bus = ModbusRTU(sim, min_gap_s=0.0)
        sim.error = OSError("the port died mid-request")
        with self.assertRaises(TransportError) as caught:
            bus.read_registers(1, R.PNOW, 2)
        self.assertIsInstance(caught.exception.__cause__, OSError)
        self.assertIn("the port died mid-request", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
