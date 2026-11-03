"""Elecylinder against SimulatedEC: identify, status, commands, soft limits, motion, safety guards."""

from __future__ import annotations

import _thread
import struct
import threading
import time
import unittest
from dataclasses import replace

from ec_testutil import hexbytes, make_ec

from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder, IaiCommandStrategy, PositionEntry, Status
from iai_actuator_driver.errors import (
    EcAlarmError,
    EcNotReadyError,
    ModbusTimeout,
    MotionTimeoutError,
    NonVolatileWriteError,
    SoftLimitError,
    VerificationError,
)
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimException, SimulatedEC
from iai_actuator_driver.testing import virtual_clock


class ReadOnlyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec()

    def tearDown(self) -> None:
        self.ec.close()

    def test_status_decodes_live_values(self) -> None:
        st = self.ec.status()
        self.assertEqual(st.position_mm, 73.95)
        self.assertEqual(st.alarm_code, 0)
        self.assertTrue(st.servo_on and st.homed and st.in_position and st.modbus_enabled)
        self.assertFalse(st.moving or st.alarm or st.emergency_stop or st.warning)
        self.assertEqual(st.raw["DSS1"], 0x3098)
        self.assertEqual(st.raw["DSSE"], 0x33C2)
        self.assertEqual(st.raw["ZONS"], 0x0003)  # both zones, no position marker at 73.95 mm
        self.assertEqual(st.raw["DOPM"], R.DOPM_3POS_NO_ALARM)
        self.assertIs(st.at_middle, False)
        self.assertEqual(st.velocity_mm_s, 0.0)
        self.assertEqual(st.current_ma, 120)
        self.assertTrue(st.ready)

    def test_status_falls_back_to_short_block(self) -> None:
        ec = Elecylinder(ModbusRTU(SimulatedEC(unreadable=[(0x9008, 0x1A)])))  # 0x9008..0x9021 refused (exception 02)
        st = ec.status()
        self.assertEqual(st.position_mm, 73.95)
        self.assertEqual(st.velocity_mm_s, 0.0)  # MOVE clear
        self.assertIsNone(st.current_ma)
        self.assertNotIn("CNOW", st.raw)
        self.assertNotIn("ZONS", st.raw)
        for marker in (st.at_backward_end, st.at_forward_end, st.at_middle):
            self.assertIsNone(marker)  # never reported without ZONS (DOPM is not used for markers)
        self.assertEqual(ec.status().position_mm, 73.95)  # stays on the short block

    def test_identify(self) -> None:
        info = self.ec.identify()
        self.assertEqual(info.system_code, 0xEC)
        self.assertEqual(info.model, "MWB-EC-DSRR4-WA-MF")  # the 0x0600 model string is the display name
        self.assertEqual(info.controller, "EC 24 V, unit 0x02 (EC dust/drip-proof)")  # the unit code's family name
        self.assertEqual((info.actuator_kind, info.actuator), (9, "slider (small) (AKID 9)"))  # kind from AKID
        self.assertEqual((info.identity_source, info.unit_known, info.axis_number), ("FC11", True, 0))
        self.assertEqual(info.user_name, "MWB-EC-DSRR4")  # 0x061A ANA1-3: the user-editable unit name
        self.assertEqual(info.short_model, "MWB-EC-DSRR4")  # the compatibility alias of user_name
        self.assertEqual((info.extensibility_level, info.rotary), (1, False))  # ACCL 0x00010110: bits 8-11 = 1
        self.assertEqual(info.max_speed_mm_s, 700.0)
        self.assertEqual((info.max_accel_g, info.max_decel_g), (1.0, 1.0))
        self.assertIn("MAXV item 7.4", info.speed_limit_source)
        self.assertEqual(info.firmware, "0013")
        self.assertEqual(info.index_header.record_count, 27)
        self.assertEqual(len(info.categories), 27)
        self.assertEqual(info.category(R.CATEGORY_COMMAND2).data_address, 0x9900)
        self.assertEqual(info.category(R.CATEGORY_IO_CONTROL).coil_base, 0x0400)
        self.assertEqual(info.errors, ())
        self.assertEqual(info.to_dict()["slave_id_raw"], "ec ff 02 00 13")

    def test_identify_reports_unsupported_parts(self) -> None:
        class NoIdSim(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1] == 0x11:
                    raise SimException(0x01)
                return NotImplemented

        info = Elecylinder(ModbusRTU(NoIdSim())).identify()
        # FC11 refused: the same identity comes from CTCL (class 3 item 1, 0x0300), the fallback source (UNVERIFIED:
        # FC11 answered on every unit tested)
        self.assertEqual((info.system_code, info.unit_code, info.firmware), (0xEC, 0x02, "0013"))
        self.assertEqual(info.identity_source, "CTCL 0x0300")
        self.assertIsNone(info.slave_id_raw)
        self.assertEqual(info.model, "MWB-EC-DSRR4-WA-MF")  # still named from 0x0600
        self.assertEqual(len(info.categories), 27)
        self.assertTrue(any("slave id" in e for e in info.errors))
        self.assertTrue(any("CTCL" in e for e in info.errors))

        info = Elecylinder(ModbusRTU(NoIdSim(word_overrides={0x0300: 0, 0x0301: 0}))).identify()  # no CTCL either
        self.assertIsNone(info.system_code)
        self.assertEqual(info.controller, "unknown")
        self.assertIsNone(info.identity_source)

    def test_read_position_table(self) -> None:
        entries = [self.ec.read_position_table(n) for n in range(3)]
        self.assertEqual([e.position_mm for e in entries], [0.0, 150.0, 75.0])
        self.assertEqual([e.speed_mm_s for e in entries], [108.5, 248.5, 248.5])
        self.assertEqual(entries[1].accel_g, 0.3)
        self.assertEqual(entries[1].decel_g, 0.3)
        self.assertEqual(entries[1].band_mm, 0.1)
        self.assertEqual(entries[1].control_flags, 0x0010)
        self.assertEqual(entries[1].checksum, 0x0065)
        self.assertTrue(all(e.checksum_ok for e in entries))
        with self.assertRaises(ValueError):
            self.ec.read_position_table(0x300)

    def test_status_alarm_logic(self) -> None:
        base = [0, 7395, 0, 0, 0, 0x3098, 0x8000, 0x33C2]
        warn = list(base)
        warn[2], warn[5] = 0x04E, 0x3098 | R.DSS1_ALML
        st = Status.from_registers(warn)
        self.assertFalse(st.alarm)
        self.assertTrue(st.warning)
        bare = list(base)
        bare[2] = 0x0E0  # code without any alarm bit: treated as an alarm (fail safe)
        self.assertTrue(Status.from_registers(bare).alarm)
        major = list(base)
        major[5] |= R.DSS1_ALMH
        self.assertTrue(Status.from_registers(major).alarm)
        negative = list(base)
        negative[0:2] = [0xFFFF, 0xFFF5]
        self.assertEqual(Status.from_registers(negative).position_mm, -0.11)
        with self.assertRaises(ValueError):
            Status.from_registers(base[:5])

    def test_constructor_validation(self) -> None:
        bus = ModbusRTU(SimulatedEC())
        for kwargs in ({"stroke": (10.0, 5.0)}, {"max_speed_mm_s": 0}, {"max_accel_g": 5.0}, {"strategy": "x"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Elecylinder(bus, **kwargs)  # type: ignore[arg-type]
        self.assertIsInstance(Elecylinder(bus, strategy=IaiCommandStrategy()).strategy, IaiCommandStrategy)


class MoveTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec()

    def tearDown(self) -> None:
        self.ec.close()

    def test_move_to_and_wait(self) -> None:
        st = self.ec.move_to(80.0, speed_mm_s=50.0, accel_g=0.1)
        self.assertIsNotNone(st)
        self.assertAlmostEqual(st.position_mm, 80.0, delta=0.1)
        self.assertFalse(st.moving)
        self.assertTrue(st.in_position)
        self.assertAlmostEqual(self.sim.position_mm, 80.0, places=6)

    def test_move_frame_is_full_block_with_ctlf(self) -> None:
        # ME0162 p.5-136 example (50 mm, 0.1 mm, 100 mm/s, 0.3 G) extended to 9 words: PPOW 0, CTLF 0.
        self.rec.clear()
        self.ec.move_to(50.0, speed_mm_s=100.0, accel_g=0.3, band_mm=0.1, wait=False)
        expected = build_frame(1, hexbytes("10 99 00 00 09 12 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 00 00 00 00"))
        self.assertIn(expected, self.rec.tx)
        self.ec.wait_idle(50.0, 0.1, timeout=5.0)

    def test_soft_limits_send_nothing(self) -> None:
        self.rec.clear()
        bad_calls = [
            {"mm": 150.01},
            {"mm": -0.01},
            {"mm": float("nan")},
            {"mm": 80.0, "speed_mm_s": 0.0},
            {"mm": 80.0, "speed_mm_s": 100.01},
            {"mm": 80.0, "accel_g": 0.31},
            {"mm": 80.0, "accel_g": 0.0},
            {"mm": 80.0, "band_mm": 0.0},
            {"mm": 80.0, "band_mm": 20.0},
        ]
        for kwargs in bad_calls:
            with self.subTest(kwargs=kwargs), self.assertRaises(SoftLimitError):
                self.ec.move_to(**kwargs)  # type: ignore[arg-type]
        self.assertEqual(self.rec.tx, [])
        self.assertFalse(self.sim.moving)

    def test_limits_are_inclusive(self) -> None:
        self.ec.move_to(150.0, speed_mm_s=100.0, accel_g=0.3)
        self.assertEqual(self.ec.status().position_mm, 150.0)
        self.ec.move_to(0.0, speed_mm_s=100.0, accel_g=0.3)
        self.assertEqual(self.ec.status().position_mm, 0.0)

    def test_move_by(self) -> None:
        st = self.ec.move_by(5.0, speed_mm_s=20.0)
        self.assertAlmostEqual(st.position_mm, 78.95, delta=0.01)
        st = self.ec.move_by(-10.0, speed_mm_s=20.0)
        self.assertAlmostEqual(st.position_mm, 68.95, delta=0.01)
        with self.assertRaises(SoftLimitError):
            self.ec.move_by(100.0)
        with self.assertRaises(SoftLimitError):
            self.ec.move_by(float("inf"))

    def test_target_reached_flag(self) -> None:
        ec, _, _ = make_ec(time_scale=2.0)
        try:
            self.assertFalse(ec.status().target_reached)  # live start value 0x3098
            st = ec.move_to(75.0, speed_mm_s=50.0)
            self.assertTrue(st.target_reached)
            self.assertEqual(st.raw["DSS1"], 0x3099)  # as seen live after a completed move (b14 aside)
            ec.move_to(120.0, speed_mm_s=20.0, wait=False)
            st = ec.status()
            self.assertEqual(st.raw["DSSE"], 0x33F2)  # live value while moving
            self.assertEqual(st.raw["DSS1"] & (R.DSS1_PEND | R.DSS1_BKRL), 0)  # live: b3, b7 clear while moving
            ec.stop(wait=True)
            self.assertFalse(ec.status().target_reached)
        finally:
            ec.close()

    def test_velocity_from_position_samples(self) -> None:
        # the driver and the unit on one virtual clock: the two reads are 0.1 s apart for both (on the wall clock a
        # busy PC could stretch the time between a read and its time stamp)
        with virtual_clock() as clock:
            ec, sim, _ = make_ec(time_scale=1.0, clock=clock.perf_counter)
            try:
                ec.move_to(120.0, speed_mm_s=20.0, accel_g=0.3, wait=False)
                clock.advance(0.15)
                ec.status()
                clock.advance(0.1)
                st = ec.status()
                self.assertTrue(st.moving)
                self.assertAlmostEqual(st.velocity_mm_s, 20.0, delta=3.0)
                st = ec.stop(wait=True)
                self.assertEqual(st.velocity_mm_s, 0.0)
            finally:
                ec.close()

    def test_retarget_with_dropped_reply(self) -> None:
        # On the EC-DS4H slider the read right after a mid-move write got no reply (VERIFIED); retries must absorb it.
        ec, sim, rec = make_ec(time_scale=2.0, drop_reply_after_retarget=True)
        try:
            ec.move_to(120.0, speed_mm_s=20.0, wait=False)
            time.sleep(0.05)
            st = ec.move_to(90.0, speed_mm_s=20.0)
            self.assertAlmostEqual(st.position_mm, 90.0, delta=0.1)
            moves = [i for i, f in enumerate(rec.tx) if f[1] == 0x10]
            after = rec.tx[moves[-1] + 1 : moves[-1] + 3]
            self.assertEqual(after[0], after[1])  # the unanswered read was repeated
            self.assertEqual(after[0][1], 0x03)
        finally:
            ec.close()

    def test_preconditions(self) -> None:
        self.sim.servo_on = False
        with self.assertRaisesRegex(EcNotReadyError, "servo is off"):
            self.ec.move_to(80.0)
        self.sim.servo_on = True
        self.sim.homed = False
        with self.assertRaisesRegex(EcNotReadyError, "not homed"):
            self.ec.move_to(80.0)
        self.sim.homed = True
        self.sim.pmsl = False
        self.ec.auto_modbus_control = False  # strict mode: no automatic PMSL write
        with self.assertRaisesRegex(EcNotReadyError, "enable_modbus_control"):
            self.ec.move_to(80.0)
        self.ec.auto_modbus_control = True
        self.sim.pmsl = True
        self.sim.trigger_alarm(0x0E0)
        with self.assertRaises(EcAlarmError) as ctx:
            self.ec.move_to(80.0)
        self.assertEqual(ctx.exception.alarm_code, 0x0E0)
        self.assertFalse(self.sim.moving)

    def test_stop_mid_move(self) -> None:
        ec, sim, _ = make_ec(time_scale=5.0)
        try:
            ec.move_to(140.0, speed_mm_s=50.0, accel_g=0.3, wait=False)
            time.sleep(0.1)
            self.assertTrue(ec.status().moving)
            st = ec.stop(wait=True)
            self.assertFalse(st.moving)
            self.assertGreater(st.position_mm, 73.95)
            self.assertLess(st.position_mm, 139.0)
            self.assertIn("stop", sim.events)
        finally:
            ec.close()

    def test_wait_timeout_stops_and_raises(self) -> None:
        ec, sim, _ = make_ec(time_scale=1.0)
        try:
            with self.assertRaises(MotionTimeoutError) as ctx:
                ec.move_to(140.0, speed_mm_s=10.0, accel_g=0.1, timeout=0.2)
            self.assertTrue(getattr(ctx.exception, "stop_attempted", False))
            self.assertIn("iai_actuator_driver: decel stop sent after this error", ctx.exception.__notes__)
            self.assertEqual(list(sim.events).count("stop"), 1)
            time.sleep(0.1)
            self.assertFalse(sim.moving)
            self.assertLess(sim.position_mm, 80.0)
        finally:
            ec.close()

    def test_alarm_during_motion(self) -> None:
        ec, sim, _ = make_ec(time_scale=2.0)
        try:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            sim.trigger_alarm(0x0E0)
            with self.assertRaises(EcAlarmError) as ctx:
                ec.wait_idle(140.0, 0.1, timeout=5.0)
            self.assertEqual(ctx.exception.alarm_code, 0x0E0)
            self.assertIn("stop", sim.events)
        finally:
            ec.close()

    def test_link_error_during_wait_attempts_stop(self) -> None:
        ec, sim, _ = make_ec(time_scale=1.0)
        try:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            sim.inject_fault("timeout", 3)  # the next status read fails completely
            with self.assertRaises(ModbusTimeout) as ctx:
                ec.wait_idle(140.0, 0.1, timeout=5.0)
            self.assertTrue(getattr(ctx.exception, "stop_attempted", False))
            self.assertIn("stop", sim.events)
        finally:
            ec.close()

    def test_keyboard_interrupt_during_wait_stops(self) -> None:
        ec, sim, _ = make_ec(time_scale=1.0)
        timer = threading.Timer(0.3, _thread.interrupt_main)  # a real Ctrl+C while the 7 s move is waited for
        try:
            timer.start()
            with self.assertRaises(KeyboardInterrupt):
                ec.move_to(140.0, speed_mm_s=20.0)
            self.assertIn("stop", sim.events)
        finally:
            timer.cancel()
            ec.close()

    def test_pause_resume(self) -> None:
        ec, sim, _ = make_ec(time_scale=5.0)
        try:
            ec.move_to(100.0, speed_mm_s=20.0, accel_g=0.3, wait=False)
            time.sleep(0.05)
            ec.pause()
            time.sleep(0.05)
            st = ec.status()
            self.assertTrue(st.paused)
            self.assertFalse(st.moving)
            ec.resume()
            st = ec.wait_idle(100.0, 0.1, timeout=10.0)
            self.assertEqual(st.position_mm, 100.0)
        finally:
            ec.close()

    def test_context_manager_stops_on_error_and_closes(self) -> None:
        sim = SimulatedEC(time_scale=5.0)
        with self.assertRaises(RuntimeError) as ctx:
            with Elecylinder.open(sim, poll_interval_s=0.005) as ec:
                ec.move_to(140.0, speed_mm_s=20.0, wait=False)
                raise RuntimeError("user code failed")
        self.assertIn("stop", sim.events)
        self.assertTrue(sim.closed)
        self.assertIn("iai_actuator_driver: decel stop sent on leaving the Elecylinder context", ctx.exception.__notes__)

    def test_context_manager_clean_exit(self) -> None:
        sim = SimulatedEC()
        with Elecylinder.open(sim) as ec:
            ec.status()
        self.assertTrue(sim.closed)
        self.assertNotIn("stop", sim.events)
        bus = ModbusRTU(SimulatedEC())
        with Elecylinder(bus):
            pass
        bus.read_registers(1, R.PNOW, 2)  # a borrowed bus stays open


class CommandStrategyTest(unittest.TestCase):
    def run_cycle(self, strategy: str) -> tuple[SimulatedEC, list[bytes]]:
        ec, sim, rec = make_ec(strategy=strategy)
        try:
            ec.servo(False)
            self.assertFalse(sim.servo_on)
            ec.servo(True)
            self.assertTrue(sim.servo_on)
            st = ec.home()
            self.assertTrue(st.homed)
            self.assertEqual(st.position_mm, 0.0)
            ec.move_to(20.0, speed_mm_s=100.0, accel_g=0.3, wait=False)
            ec.stop(wait=True)
            sim.trigger_alarm(0x0E0)
            self.assertFalse(sim.servo_on)
            st = ec.reset_alarm()
            self.assertFalse(st.alarm)
            self.assertEqual(st.alarm_code, 0)
            return sim, rec.tx
        finally:
            ec.close()

    def test_coil_strategy(self) -> None:
        _, tx = self.run_cycle("coil")
        for frame in (
            "01 05 04 03 00 00 3C FA",  # servo off
            "01 05 04 03 FF 00 7D 0A",  # servo on
            "01 05 04 0B 00 00 BD 38",  # home: 0 ...
            "01 05 04 0B FF 00 FC C8",  # ... then rising edge
            "01 05 04 2C FF 00 4C C3",  # stop
            "01 05 04 07 FF 00 3C CB",  # alarm reset pulse
            "01 05 04 07 00 00 7D 3B",
        ):
            self.assertIn(hexbytes(frame), tx)
        home_on = tx.index(hexbytes("01 05 04 0B FF 00 FC C8"))
        self.assertIn(hexbytes("01 05 04 0B 00 00 BD 38"), tx[home_on + 1 :])  # released after homing

    def test_iai_strategy(self) -> None:
        _, tx = self.run_cycle("iai")
        self.assertIn(hexbytes("01 45 00 09 00 01 00 00 18 C2"), tx)  # 0x45 servo-on frame (VERIFIED on the gripper)
        self.assertIn(hexbytes("01 45 00 0A 00 07 00 00 BC C3"), tx)  # 0x45 home frame: op1 = ORG (9.16) = 7 (VERIFIED)
        self.assertNotIn(hexbytes("01 45 00 0A 00 00 00 00 0D 02"), tx)  # op1 = 0: rejected 0x11 by the gripper
        self.assertIn(hexbytes("01 45 00 05 00 00 00 00 59 03"), tx)  # 0x45 alarm reset, op1 0 (VERIFIED)
        self.assertFalse(any(f[1] == 0x45 and f[3] == 0x0B for f in tx))  # 0x45 decel stop never used
        coils = {struct.unpack(">H", f[2:4])[0] for f in tx if f[1] == 0x05}
        self.assertEqual(coils, {R.COIL_STOP})  # the only coil: the verified STOP

    def test_iai2_strategy(self) -> None:
        sim, tx = self.run_cycle("iai2")
        self.assertTrue(any(f[1] == 0x48 and f[3:5] == bytes([0x00, 0x31]) for f in tx))  # group, cmd u16 0x0031
        self.assertIn("home complete", sim.events)

    def test_servo_off_while_moving_stops_first(self) -> None:
        ec, sim, _ = make_ec(time_scale=2.0)
        try:
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            ec.servo(False)
            events = list(sim.events)
            self.assertLess(events.index("stop"), events.index("servo off"))
        finally:
            ec.close()

    def test_servo_on_refused_with_alarm(self) -> None:
        ec, sim, _ = make_ec()
        try:
            sim.trigger_alarm(0x0E0)
            with self.assertRaises(EcAlarmError):
                ec.servo(True)
        finally:
            ec.close()

    def test_enable_modbus_control(self) -> None:
        ec, sim, rec = make_ec()
        try:
            sim.pmsl = False
            self.assertFalse(ec.status().modbus_enabled)
            st = ec.enable_modbus_control()
            self.assertTrue(st.modbus_enabled)
            self.assertIn(hexbytes("01 05 04 27 FF 00 3D 01"), rec.tx)
            ec.disable_modbus_control()
            self.assertFalse(sim.pmsl)
        finally:
            ec.close()


class PositionTableWriteTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec()

    def tearDown(self) -> None:
        self.ec.close()

    def test_written_without_an_opt_in(self) -> None:
        entry = replace(self.ec.read_position_table(2), position_mm=60.0, speed_mm_s=50.0)
        self.assertEqual(self.ec.write_position_table(2, entry).position_mm, 60.0)  # no allow_nonvolatile needed
        self.assertIsInstance(NonVolatileWriteError("x"), PermissionError)
        self.assertEqual(self.sim.nonvolatile_writes, 1)

    def test_write_when_allowed(self) -> None:
        original = self.ec.read_position_table(2)
        entry = PositionEntry(position_mm=60.0, band_mm=0.2, speed_mm_s=50.0, accel_g=0.2, decel_g=0.25,
                              control_flags=original.control_flags)  # fmt: skip
        back = self.ec.write_position_table(2, entry, allow_nonvolatile=True)
        self.assertEqual(self.sim.nonvolatile_writes, 1)
        self.assertEqual(back.position_mm, 60.0)
        self.assertEqual(back.decel_g, 0.25)
        self.assertNotEqual(back.checksum, original.checksum)  # updated by the (simulated) EC
        self.assertTrue(back.checksum_ok)
        self.assertEqual(self.ec.read_position_table(2).band_mm, 0.2)

    def test_write_validation(self) -> None:
        for entry in (
            PositionEntry(position_mm=151.0),
            PositionEntry(position_mm=50.0, speed_mm_s=248.5),  # above max_speed_mm_s
            PositionEntry(position_mm=50.0, accel_g=0.5),
            PositionEntry(position_mm=50.0, band_mm=0.0),
        ):
            with self.subTest(entry=entry), self.assertRaises(SoftLimitError):
                self.ec.write_position_table(0, entry, allow_nonvolatile=True)
        self.assertEqual(self.sim.nonvolatile_writes, 0)

    def test_checksum_not_updated_is_detected(self) -> None:
        ec, sim, _ = make_ec(recompute_position_checksum=False)
        try:
            entry = replace(ec.read_position_table(2), position_mm=60.0, speed_mm_s=50.0)
            with self.assertRaisesRegex(VerificationError, "checksum"):
                ec.write_position_table(2, entry, allow_nonvolatile=True, write_checksum=False)
            back = ec.write_position_table(2, entry, allow_nonvolatile=True)  # default: checksum written
            self.assertTrue(back.checksum_ok)
            self.assertEqual(sim.position_row(2)[15], R.position_checksum(entry.to_registers()))
        finally:
            ec.close()

    def test_rows_past_the_ec_table_are_written_with_a_warning(self) -> None:
        entry = replace(self.ec.read_position_table(2), position_mm=60.0, speed_mm_s=50.0)
        for n in (256, 260, 267):  # the EC's internal simple-data store: never, whatever the options
            with self.subTest(n=n), self.assertRaisesRegex(NonVolatileWriteError, "simple-data"):
                self.ec.write_position_table(n, entry, allow_nonvolatile=True, allow_unsafe=True)
        self.assertEqual(self.sim.nonvolatile_writes, 0)
        for n in (3, 255, 0x2FF):  # past the rows the EC runs: written, with a warning
            with self.subTest(n=n), self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                self.assertEqual(self.ec.write_position_table(n, entry).position_mm, 60.0)
            self.assertIn("rows 0-2", "\n".join(logs.output))
        self.assertEqual(self.sim.nonvolatile_writes, 3)

    def test_entry_values_validated(self) -> None:
        base = replace(self.ec.read_position_table(2), speed_mm_s=50.0)
        for changes in (
            {"push_power": 0xFFFF},
            {"push_power": 300},
            {"load_threshold": 300},
            {"control_flags": 0xFFFF},
            {"accel_g": 0.3, "decel_g": 0.31},  # above max_accel_g
            {"zone_plus_mm": float("inf")},
            {"zone_minus_mm": float("nan")},
            {"position_mm": float("inf")},
            {"zone_plus_mm": 3e7},  # does not fit int32 counts
        ):
            with self.subTest(changes=changes), self.assertRaises(SoftLimitError):
                self.ec.write_position_table(2, replace(base, **changes), allow_nonvolatile=True)
        # push rows on row 1 (row 2 is the middle point of this 3-position slider: no push there, DOC ME3837-1D p.2-5)
        forward = replace(self.ec.read_position_table(1), position_mm=140.0, speed_mm_s=50.0)
        for changes in (
            {"control_flags": 0x0010 | R.CTLF_PUSH, "push_power": 154},  # above floor(MXPP 60 % x 255 / 100) = 153
            {"control_flags": 0x0010 | R.CTLF_PUSH, "push_power": 0},  # a push row without force
        ):
            with self.subTest(changes=changes), self.assertRaises(SoftLimitError):
                self.ec.write_position_table(1, replace(forward, **changes), allow_nonvolatile=True)
        # allow_unsafe (accepted and ignored) lifts no range check: push power and load threshold stay refused
        for changes in ({"control_flags": 0x0010 | R.CTLF_PUSH, "push_power": 154}, {"load_threshold": 300}):
            with self.subTest(changes=changes, allow_unsafe=True), self.assertRaises(SoftLimitError):
                self.ec.write_position_table(1, replace(forward, **changes), allow_unsafe=True)
        self.assertEqual(self.sim.nonvolatile_writes, 0)
        self.assertEqual(self.ec.push_cap_pct, 60.0)  # min(max_push_pct 70, MXPP 7.9 = 60) on the slider
        pushing = replace(forward, control_flags=0x0010 | R.CTLF_PUSH, push_power=153)
        self.assertEqual(self.ec.write_position_table(1, pushing, True).push_power, 153)
        # below the recommended minimum (MNPP 20 %) is only a recommendation (the EC-R6S rod ships a 10 % row that
        # runs, VERIFIED)
        weak = replace(forward, control_flags=0x0010 | R.CTLF_PUSH, push_power=10)
        self.assertEqual(self.ec.write_position_table(1, weak, True).push_power, 10)
        # a push on the 3-position middle row and an unknown flag bit are written, each with a warning
        for changes, warning in (
            ({"control_flags": 0x0020 | R.CTLF_PUSH, "push_power": 100}, "middle point"),
            ({"control_flags": 0x0020 | 0x0040}, "unknown bits"),  # unknown bit 6
        ):
            with self.subTest(changes=changes), self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                written = self.ec.write_position_table(2, replace(base, **changes))
            self.assertEqual(written.control_flags, changes["control_flags"])
            self.assertIn(warning, "\n".join(logs.output))

    def test_role_bits_of_rows_1_and_2_are_kept(self) -> None:
        fresh = PositionEntry(position_mm=60.0, speed_mm_s=50.0)  # control_flags 0 clears 0x0020
        kept = replace(self.ec.read_position_table(2), position_mm=60.0, speed_mm_s=50.0)
        with self.assertNoLogs("iai_actuator_driver", "WARNING"):
            self.assertEqual(self.ec.write_position_table(2, kept).control_flags, 0x0020)
        with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
            back = self.ec.write_position_table(2, fresh)
        self.assertIn("role bits", "\n".join(logs.output))
        self.assertEqual(back.control_flags, 0)

    def test_full_entry_with_checksum_is_written_by_default(self) -> None:
        self.rec.clear()
        entry = replace(self.ec.read_position_table(0), position_mm=5.0, speed_mm_s=50.0)
        self.ec.write_position_table(0, entry, allow_nonvolatile=True)
        write = next(f for f in self.rec.tx if f[1] == 0x10)
        self.assertEqual(struct.unpack(">HH", write[2:6]), (0x1000, 16))

    def test_checksum_helper_matches_live_rows(self) -> None:
        for n in range(4):
            row = self.sim.position_row(n)
            self.assertEqual(R.position_checksum(row), row[15])
            self.assertEqual(sum(b for w in row for b in (w >> 8, w & 0xFF)) % 256, 0)

    def test_entry_round_trip(self) -> None:
        words = self.sim.position_row(1)
        entry = PositionEntry.from_registers(words)
        self.assertEqual(entry.to_registers(), words[:15])
        self.assertEqual(len(entry.to_registers()), R.POSITION_WORDS_WRITABLE)


if __name__ == "__main__":
    unittest.main()
