"""Driver behaviour based on hardware observations: completion, home, jog, push, soft limits, identity, alarm
history."""

from __future__ import annotations

import math
import struct
import time
import unittest

from ec_testutil import FrameRecorder, SteppedClockSim, hexbytes, make_ec

from iai_actuator_driver import registers as R
from iai_actuator_driver import testing
from iai_actuator_driver.ec import Elecylinder, Status
from iai_actuator_driver.errors import EcNotReadyError, MotionTimeoutError, SoftLimitError, VerificationError
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimulatedEC


def moves(frames: list[bytes]) -> list[tuple[int, ...]]:
    """Decode the 9-word 0x9900 writes as (pcmd, inp, vcmd, acmd, ppow, ctlf)."""
    out = []
    for f in frames:
        if f[1] == 0x10 and struct.unpack(">HH", f[2:6]) == (R.DIRECT_MOVE_BASE, 9):
            w = struct.unpack(">9H", f[7:25])
            out.append((R.from_i32_words(w[0], w[1]), R.from_u32_words(w[2], w[3]),
                        R.from_u32_words(w[4], w[5]), w[6], w[7], w[8]))  # fmt: skip
    return out


def status_samples(rec: FrameRecorder) -> list[Status]:
    """The status blocks the driver read (FC03 at 0x9000), decoded from the recorded replies."""
    samples: list[Status] = []
    words = 0  # words of the 0x9000 block the last request read (0: another request)
    for direction, frame in rec.frames:
        if direction == "tx":
            address, count = struct.unpack(">HH", frame[2:6]) if frame[1] == 0x03 else (0, 0)
            words = count if address == R.MONITOR_BASE else 0
        elif words >= R.STATUS_BLOCK_MIN_WORDS and frame[1] == 0x03:
            samples.append(Status.from_registers(struct.unpack(f">{words}H", frame[3:-2])))
    return samples


class CompletionTest(unittest.TestCase):
    """MOVE clears during deceleration: completion is PEND + position, never MOVE alone."""

    def test_fast_move_completes_at_target(self) -> None:
        # a stepped clock: the unit's time advances 20 ms with every request, so the ~0.1 s deceleration (0.1 G from
        # 100 mm/s) holds four polls with MOVE clear and the axis outside the band (three are required), whatever
        # the PC's load (on the wall clock the polls had to land inside it by luck)
        ec, sim, rec = make_ec(sim=SteppedClockSim(step_s=0.02))
        try:
            st = ec.move_to(130.0, speed_mm_s=100.0, accel_g=0.1)
            samples = status_samples(rec)
            seen_early_clear = [s.position_mm for s in samples if s.moving and not s.move_bit and not s.in_position]
            self.assertGreaterEqual(len(seen_early_clear), 3, "the simulator should clear MOVE while decelerating")
            self.assertTrue(all(p < 130.0 for p in seen_early_clear))
            self.assertEqual(st.position_mm, 130.0)
            self.assertTrue(st.in_position and st.target_reached)
            self.assertFalse(sim.moving)
        finally:
            ec.close()


class HomeTest(unittest.TestCase):
    def test_home_sequence(self) -> None:
        ec, sim, rec = make_ec(time_scale=5.0)
        try:
            ec.home(wait=False)
            dsse_seen, lowest = set(), 100.0
            deadline = time.perf_counter() + 5.0
            while time.perf_counter() < deadline:
                st = ec.status()
                dsse_seen.add(st.raw["DSSE"])
                lowest = min(lowest, st.position_mm)
                if st.homed and st.in_position and not st.homing:
                    break
                time.sleep(0.002)
            ec.finish_home()
            self.assertIn(0x3BF2, dsse_seen)  # live: homing
            self.assertIn(0x39C2, dsse_seen)  # live: final creep (MOVE 0, GHMS 1)
            self.assertLess(lowest, -1.0)  # went to the mechanical end (-2.00)
            self.assertEqual(st.position_mm, 0.0)
            coils = [f for f in rec.tx if f[1] == 0x05]
            self.assertEqual(coils, [build_frame(1, hexbytes(h)) for h in ("05 04 0B 00 00", "05 04 0B FF 00", "05 04 0B 00 00")])
        finally:
            ec.close()

    def test_home_blocking(self) -> None:
        ec, sim, _ = make_ec(time_scale=20.0)
        try:
            st = ec.home()
            self.assertTrue(st.homed and st.in_position and not st.homing)
            self.assertEqual(st.position_mm, 0.0)
            self.assertFalse(sim.coil_state(R.COIL_HOME))
        finally:
            ec.close()

    def test_home_timeout_stops_and_releases(self) -> None:
        ec, sim, rec = make_ec(time_scale=1.0)
        try:
            with self.assertRaises(MotionTimeoutError):
                ec.home(timeout=0.2)
            self.assertFalse(sim.coil_state(R.COIL_HOME))
            self.assertFalse(sim.stop_held)
            self.assertIn("stop", sim.events)
            self.assertFalse(sim.moving)
            self.assertFalse(sim.homed)
        finally:
            ec.close()

    def test_home_not_started(self) -> None:
        ec, sim, _ = make_ec()
        try:
            sim.set_coil_state(R.COIL_STOP, True)  # a latched STOP makes the EC ignore HOME
            with testing.timings(home_start_s=0.2), self.assertRaisesRegex(EcNotReadyError, "did not start"):
                ec.home()
            self.assertFalse(sim.coil_state(R.COIL_HOME))
        finally:
            ec.close()


class TimeoutArgumentTest(unittest.TestCase):
    """A timeout of 0, a negative one (-1 is "wait forever" in VB and LabVIEW) or an infinity is refused before
    anything is sent (it used to send the command and cut it short); None stays the default."""

    def test_bad_timeouts_send_nothing(self) -> None:
        ec, sim, rec = make_ec(time_scale=20.0)
        try:
            ec.move_to(60.0)
            mark = len(rec.tx)
            for bad in (-1.0, 0.0, math.inf):
                with self.assertRaisesRegex(ValueError, "nothing sent"):
                    ec.home(timeout=bad)
                with self.assertRaises(ValueError):
                    ec.move_to(30.0, timeout=bad)
                with self.assertRaises(ValueError):
                    ec.move_to(30.0, wait=False, timeout=bad)
                with self.assertRaises(ValueError):
                    ec.servo(False, timeout=bad)
            commands = [f for f in rec.tx[mark:] if f[1] in (0x05, 0x06, 0x0F, 0x10, 0x45)]
            self.assertEqual(commands, [])
            st = ec.status()
            self.assertTrue(st.homed and st.servo_on)
            self.assertAlmostEqual(st.position_mm, 60.0, delta=0.11)
            self.assertAlmostEqual(ec.move_to(30.0, timeout=20.0).position_mm, 30.0, delta=0.11)
        finally:
            ec.close()


class JogTest(unittest.TestCase):
    def test_jog_is_a_relative_move_without_jog_coils(self) -> None:
        ec, sim, rec = make_ec()
        try:
            st = ec.jog(5.0)
            self.assertEqual(st.position_mm, 78.95)
            st = ec.jog(-10.0, speed_mm_s=20.0)
            self.assertEqual(st.position_mm, 68.95)
            coils = {struct.unpack(">H", f[2:4])[0] for f in rec.tx if f[1] == 0x05}
            self.assertFalse(coils & {R.COIL_JOG_PLUS, R.COIL_JOG_MINUS})
            self.assertEqual([m[0] for m in moves(rec.tx)], [7895, 6895])  # absolute targets
            self.assertTrue(all(m[5] == R.CTLF_ABSOLUTE for m in moves(rec.tx)))
            with self.assertRaises(SoftLimitError):
                ec.jog(100.0)
        finally:
            ec.close()


class PushTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ec, self.sim, self.rec = make_ec(time_scale=20.0)

    def tearDown(self) -> None:
        self.ec.close()

    def test_push_missed_away_from_home(self) -> None:
        result = self.ec.push(80.0, 10.0, 30.0)
        self.assertTrue(result.missed)
        self.assertEqual(result.position_mm, 90.0)  # PCMD + band
        self.assertTrue(result.status.push_missed)
        self.assertFalse(result.status.in_position)
        frame = moves(self.rec.tx)[-1]
        # PPOW 30 % = 76 (as sent on hardware, VERIFIED), CTLF push + away from home
        self.assertEqual(frame, (8000, 1000, 1000, 10, 76, 0x0006))
        self.assertEqual((R.push_power(20), R.push_power(70)), (R.PPOW_MIN, R.PPOW_MAX))  # 0x33 / 0xB2 (ME0162)

    def test_push_toward_home(self) -> None:
        result = self.ec.push(60.0, 10.0, 25.0, direction=-1)
        self.assertTrue(result.missed)
        self.assertEqual(result.position_mm, 50.0)
        self.assertEqual(moves(self.rec.tx)[-1][5], R.CTLF_PUSH)

    def test_push_contact_then_retract(self) -> None:
        self.sim.obstacle_mm = 85.0
        result = self.ec.push(80.0, 20.0, 30.0)
        self.assertTrue(result.contact)
        self.assertEqual(result.position_mm, 85.0)
        self.assertEqual(result.status.raw["DSSE"], 0x37C3)  # live value after contact
        self.assertEqual(result.status.raw["DSS1"], 0x3099)  # live 0x7099 minus SFTY
        self.assertEqual(self.ec.move_to(75.0, speed_mm_s=20.0).position_mm, 75.0)

    def test_start_inside_or_past_the_zone_is_refused(self) -> None:
        self.rec.clear()
        for target, direction in ((70.0, 1), (65.0, 1), (78.0, -1), (90.0, -1)):
            with self.subTest(target=target, direction=direction), self.assertRaisesRegex(EcNotReadyError, "push zone"):
                self.ec.push(target, 10.0, 30.0, direction=direction)
        self.assertEqual(moves(self.rec.tx), [])

    def test_validation(self) -> None:
        self.rec.clear()
        bad = [
            {"target_mm": 145.0, "band_mm": 10.0, "force_pct": 30.0},  # band end 155 outside
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": 75.0},  # above the 70 % cap
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": 61.0},  # above the slider's MXPP (item 7.9) 60 %
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": 0.1},  # PPOW 0: no force at all
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": float("nan")},
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": 30.0, "direction": 0},
            {"target_mm": 80.0, "band_mm": 0.0, "force_pct": 30.0},
            {"target_mm": 80.0, "band_mm": 10.0, "force_pct": 30.0, "speed_mm_s": 500.0},
        ]
        for kwargs in bad:
            with self.subTest(kwargs=kwargs), self.assertRaises(SoftLimitError):
                self.ec.push(**kwargs)  # type: ignore[arg-type]
        self.assertEqual(moves(self.rec.tx), [])
        capped = Elecylinder(self.ec.bus, stroke=(0.0, 150.0), max_push_pct=40.0)
        with self.assertRaises(SoftLimitError):
            capped.push(80.0, 10.0, 50.0)

    def test_force_below_the_recommendation_is_allowed_with_a_warning(self) -> None:
        # MNPP (item 7.58; 20 % on the slider) is only a recommendation; the EC-R6S rod ships a 10 % push row (VERIFIED)
        self.rec.clear()
        with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
            result = self.ec.push(80.0, 10.0, 10.0)
        self.assertTrue(result.missed)
        self.assertEqual(moves(self.rec.tx)[-1][4], 25)  # PPOW = floor(10 x 255 / 100) (DOC ME0162-12A p.5-132)
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("recommended minimum 20 % (MNPP, item 7.58)", result.warnings[0])
        self.assertTrue(any("recommended minimum" in line for line in logs.output), logs.output)
        self.assertEqual(self.ec.push_cap_pct, 60.0)  # min(max_push_pct 70, MXPP 60)
        self.assertEqual(self.ec.push(100.0, 10.0, 60.0).warnings, ())  # from 90.00: approach side of 100..110

    def test_push_refused_when_the_ec_cannot_push(self) -> None:
        # MXPP (item 7.9, 0x0710 on the slider) = 0: no push (UNVERIFIED)
        ec, _, rec = make_ec(sim=SimulatedEC(time_scale=20.0, word_overrides={0x0710: 0, 0x0711: 0}))
        try:
            with self.assertRaisesRegex(SoftLimitError, "cannot push"):
                ec.push(80.0, 10.0, 30.0)
            self.assertEqual(moves(rec.tx), [])
            self.assertFalse(ec.axis_limits.push_available)
        finally:
            ec.close()

    def test_push_needs_a_stopped_axis(self) -> None:
        self.ec.move_to(140.0, speed_mm_s=20.0, wait=False)
        with self.assertRaisesRegex(EcNotReadyError, "stopped axis"):
            self.ec.push(145.0, 3.0, 30.0)


class StrokeTest(unittest.TestCase):
    """The EC's own soft limits bound every target (the EC-DS4H slider clamps a target past them silently)."""

    def test_default_stroke_comes_from_the_ec(self) -> None:
        ec = Elecylinder(ModbusRTU(SimulatedEC(time_scale=50.0)), poll_interval_s=0.005)
        self.assertIsNone(ec.stroke)
        self.assertIsNone(ec.effective_stroke)
        self.assertEqual(ec.read_soft_limits(), (-0.3, 150.3))
        self.assertEqual(ec.ec_stroke, (0.0, 150.0))
        self.assertEqual(ec.effective_stroke, (0.0, 150.0))
        self.assertEqual(ec.move_to(150.0, speed_mm_s=100.0, accel_g=0.3).position_mm, 150.0)
        with self.assertRaises(SoftLimitError):
            ec.move_to(150.2)

    def test_configured_stroke_narrowed_to_a_shorter_ec(self) -> None:
        sim = SimulatedEC(stroke_mm=50.0, position_mm=25.0, time_scale=50.0)
        ec = Elecylinder(ModbusRTU(sim), stroke=(0.0, 150.0), poll_interval_s=0.005)
        with self.assertRaisesRegex(SoftLimitError, r"\(soft limits -0.30..50.30 mm\)"):
            ec.move_to(100.0)
        self.assertFalse(any(f[1] == 0x10 for f in sim.requests))
        self.assertEqual(ec.stroke, (0.0, 150.0))  # configured value kept (GUI reads it)
        self.assertEqual(ec.effective_stroke, (0.0, 50.0))
        self.assertEqual(ec.move_to(45.0).position_mm, 45.0)
        self.assertEqual(sim.alarm_code, 0)

    def test_unknown_limits_need_a_configured_stroke(self) -> None:
        sim = SimulatedEC(soft_limits=False, time_scale=50.0)
        ec = Elecylinder(ModbusRTU(sim), poll_interval_s=0.005)
        with self.assertRaisesRegex(EcNotReadyError, "stroke unknown"):
            ec.move_to(80.0)
        self.assertIn("not readable", ec.soft_limits_note)
        ec.stroke = (0.0, 150.0)
        self.assertEqual(ec.move_to(80.0).position_mm, 80.0)

    def test_no_overlap(self) -> None:
        ec, _, _ = make_ec()
        try:
            ec.stroke = (200.0, 300.0)
            with self.assertRaisesRegex(SoftLimitError, "does not overlap"):
                ec.move_to(250.0)
            with self.assertRaises(ValueError):
                ec.stroke = (5.0, 1.0)
        finally:
            ec.close()

    def test_limits_follow_the_index_table(self) -> None:
        ec, _, rec = make_ec()
        try:
            ec.identify()
            rec.clear()
            ec.read_soft_limits()
            # items 7.1 / 7.2 by ID: class 7 (DA 0x0700) and its ID map, as every other class-7 value
            self.assertEqual(rec.tx[0][:4], build_frame(1, hexbytes("03 07 00 00 04"))[:4])
            self.assertIn("by ID", ec.soft_limits_note)
            self.assertEqual(ec.soft_limits_mm, (-0.3, 150.3))
        finally:
            ec.close()


class IdentityTest(unittest.TestCase):
    def test_identify_reads_identification_and_limits(self) -> None:
        ec, _, _ = make_ec()
        try:
            info = ec.identify()
            self.assertEqual(info.serial_number, "SIM-0000001")
            self.assertEqual(info.short_model, "MWB-EC-DSRR4")
            self.assertEqual(info.model_name, "MWB-EC-DSRR4-WA-MF")
            self.assertEqual(info.soft_limits_mm, (-0.3, 150.3))
            self.assertEqual(info.to_dict()["soft_limits_mm"], [-0.3, 150.3])
            self.assertEqual(info.errors, ())
        finally:
            ec.close()

    def test_fc11_silent_is_not_fatal(self) -> None:
        class SilentFc11(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                return None if frame[1] == 0x11 else NotImplemented

        ec = Elecylinder(ModbusRTU(SilentFc11()))
        info = ec.identify()
        self.assertEqual(len(info.categories), 27)  # the index table was still read
        self.assertTrue(any("not answered" in e for e in info.errors))
        self.assertEqual((info.system_code, info.unit_code, info.identity_source), (0xEC, 0x02, "CTCL 0x0300"))
        fc11 = [f for f in ec.bus.transport.requests if f[1] == 0x11]
        self.assertEqual(len(fc11), 1)  # sent once (no retry; CTCL is the fallback)


class AlarmHistoryTest(unittest.TestCase):
    def test_read_and_new_alarm(self) -> None:
        ec, sim, _ = make_ec()
        try:
            records = ec.read_alarm_history()
            self.assertEqual(len(records), 9)
            self.assertTrue(records[0].power_up)
            self.assertEqual((records[1].code, records[1].detail, records[1].time_raw), (0x0EE, 3, 951515))
            self.assertEqual(records[1].name, "ABS error (2)")
            self.assertIsNone(records[0].time_raw)
            self.assertEqual(len(ec.read_alarm_history(include_empty=True)), 32)
            sim.trigger_alarm(0x0E0)
            newest = ec.read_alarm_history()[0]
            self.assertEqual((newest.index, newest.code, newest.name), (0, 0x0E0, "Over load"))
            self.assertIn("power_up", newest.to_dict())
        finally:
            ec.close()


class SafetySpeedTest(unittest.TestCase):
    def test_reported_and_explicit_switch(self) -> None:
        ec, sim, _ = make_ec(safety_speed=True)
        try:
            self.assertTrue(ec.status().safety_speed)
            self.assertEqual(ec.status().raw["DSS1"], 0x7098)  # live value with SFTY
            st = ec.set_safety_speed(False)
            self.assertFalse(st.safety_speed)
            self.assertFalse(sim.safety_speed)
            self.assertTrue(ec.set_safety_speed(True).safety_speed)
        finally:
            ec.close()

    def test_verification(self) -> None:
        class IgnoresSfty(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1] == 0x05 and struct.unpack(">H", frame[2:4])[0] == R.COIL_SFTY:
                    return build_frame(self.slave, frame[1:6])  # the normal echo, the coil not acted on
                return NotImplemented

        ec = Elecylinder(ModbusRTU(IgnoresSfty()))
        with self.assertRaises(VerificationError):
            ec.set_safety_speed(True)


if __name__ == "__main__":
    unittest.main()
