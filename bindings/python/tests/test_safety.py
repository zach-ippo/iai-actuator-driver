"""Safety: stale CTLF, STOP release, stop path per strategy, context exit, pause and status decoding."""

from __future__ import annotations

import struct
import time
import unittest
from dataclasses import replace

from ec_testutil import hexbytes, make_ec

from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder, Status
from iai_actuator_driver.errors import (
    EcNotReadyError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    SoftLimitError,
)
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimException, SimulatedEC

STOP_ON = build_frame(1, hexbytes("05 04 2C FF 00"))
STOP_OFF = build_frame(1, hexbytes("05 04 2C 00 00"))


def coil_frames(frames: list[bytes]) -> list[bytes]:
    return [f for f in frames if f[1] == 0x05]


class StaleCtlfTest(unittest.TestCase):
    """A CTLF bit left by another tool must not change an absolute move."""

    def test_incremental_bit_left_by_another_tool(self) -> None:
        ec, sim, rec = make_ec()
        try:
            ec.bus.write_register(1, R.CTLF, R.CTLF_INC)  # another tool; its move never started
            self.assertEqual(ec.read_registers(R.CTLF, 1), [R.CTLF_INC])
            st = ec.move_to(30.0, speed_mm_s=100.0, accel_g=0.3)
            self.assertEqual(st.position_mm, 30.0)  # not 73.95 + 30
            self.assertAlmostEqual(sim.position_mm, 30.0, places=6)
            move = [f for f in rec.tx if f[1] == 0x10][-1]
            self.assertEqual(struct.unpack(">HH", move[2:6]), (R.DIRECT_MOVE_BASE, R.DIRECT_MOVE_WORDS))
            self.assertEqual(struct.unpack(">HH", move[-6:-2]), (0, R.CTLF_ABSOLUTE))  # PPOW, CTLF
        finally:
            ec.close()

    def test_push_bit_left_by_another_tool(self) -> None:
        ec, sim, _ = make_ec()
        try:
            ec.bus.write_register(1, R.CTLF, R.CTLF_PUSH | R.CTLF_PUSH_AWAY)
            st = ec.move_to(90.0, speed_mm_s=100.0, accel_g=0.3)
            self.assertEqual(st.position_mm, 90.0)
            self.assertFalse(st.push_missed or st.pushing)
            self.assertNotIn("push", " ".join(sim.events))
        finally:
            ec.close()


class StopReleaseTest(unittest.TestCase):
    """STOP is written ON, held until the axis stopped, then OFF (VERIFIED on the EC-DS4H slider)."""

    def test_stop_on_then_off_after_the_axis_stopped(self) -> None:
        ec, sim, rec = make_ec(time_scale=5.0)
        try:
            ec.move_to(140.0, speed_mm_s=50.0, accel_g=0.3, wait=False)
            time.sleep(0.1)
            rec.clear()
            st = ec.stop(wait=True)
            self.assertEqual(coil_frames(rec.tx), [STOP_ON, STOP_OFF])
            on, off = rec.tx.index(STOP_ON), rec.tx.index(STOP_OFF)
            self.assertTrue(any(f[1] == 0x03 for f in rec.tx[on:off]))  # held while polling
            self.assertFalse(st.moving)
            self.assertTrue(st.in_position)
            self.assertFalse(st.target_reached)  # DSS1 b0 = 0 after a STOP (live)
            self.assertFalse(sim.stop_held)
            self.assertIn("stop released", sim.events)
            self.assertEqual(ec.move_to(80.0, speed_mm_s=50.0).position_mm, 80.0)  # not blocked
        finally:
            ec.close()

    def test_stop_while_idle_and_with_auto_reset_ec(self) -> None:
        for latches in (True, False):
            with self.subTest(stop_latches=latches):
                ec, sim, rec = make_ec(stop_latches=latches)
                try:
                    st = ec.stop(wait=True)
                    self.assertTrue(st.in_position)
                    self.assertEqual(coil_frames(rec.tx), [STOP_ON, STOP_OFF])
                    self.assertEqual(ec.move_to(80.0).position_mm, 80.0)
                finally:
                    ec.close()

    def test_error_path_releases_stop(self) -> None:
        ec, sim, rec = make_ec(time_scale=1.0)
        try:
            with self.assertRaises(MotionTimeoutError):
                ec.move_to(140.0, speed_mm_s=10.0, accel_g=0.1, timeout=0.2)
            self.assertEqual(coil_frames(rec.tx)[-2:], [STOP_ON, STOP_OFF])
            self.assertFalse(sim.stop_held)
        finally:
            ec.close()

    def test_failed_release_is_retried_before_the_next_command(self) -> None:
        drops = 3

        class DropStopOff(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                nonlocal drops
                if frame[1:6] == hexbytes("05 04 2C 00 00") and drops:
                    drops -= 1
                    self.handle(frame)
                    return None  # the release is executed but never answered
                return NotImplemented  # type: ignore[return-value]

        sim = DropStopOff(time_scale=50.0)
        rec_bus = ModbusRTU(sim)
        ec = Elecylinder(rec_bus, stroke=(0.0, 150.0), poll_interval_s=0.005)
        with self.assertRaises(ModbusTimeout):
            ec.stop()
        self.assertTrue(ec.stop_held)
        before = len(sim.requests)
        ec.move_to(80.0)
        sent = list(sim.requests)[before:]
        first_write = next(f for f in sent if f[1] in (0x05, 0x10))
        self.assertEqual(first_write, STOP_OFF)  # release first, then the move
        self.assertFalse(ec.stop_held)

    def test_stop_held_by_someone_else_is_reported_quickly(self) -> None:
        ec, sim, _ = make_ec()
        try:
            ec.bus.write_coil(1, R.COIL_STOP, True)  # another master left STOP latched
            start = time.perf_counter()
            with self.assertRaisesRegex(MotionIncompleteError, "ignored or cut short"):
                ec.move_to(90.0, speed_mm_s=50.0, timeout=30.0)
            self.assertLess(time.perf_counter() - start, 5.0)  # not the 30 s timeout
            self.assertIn("move ignored (STOP held)", sim.events)
            self.assertFalse(sim.stop_held)  # the driver's own STOP sequence released it
        finally:
            ec.close()

    def test_close_releases_a_stop_left_on(self) -> None:
        ec, sim, _ = make_ec()
        ec.begin_stop()  # STOP ON, held until a stop() or close() releases it
        self.assertTrue(ec.stop_held)
        self.assertTrue(sim.coil_state(R.COIL_STOP))
        ec.close()
        self.assertFalse(ec.stop_held)
        self.assertFalse(sim.coil_state(R.COIL_STOP))
        self.assertFalse(sim.stop_held)


class StopPathStrategyTest(unittest.TestCase):
    """The safety stop always uses the STOP coil (VERIFIED on hardware), whatever the command strategy."""

    def test_timeout_stop_uses_coil_with_iai_strategies(self) -> None:
        for strategy in ("iai", "iai2"):
            with self.subTest(strategy=strategy):
                ec, sim, rec = make_ec(strategy=strategy, time_scale=1.0)
                try:
                    with self.assertRaises(MotionTimeoutError) as ctx:
                        ec.move_to(140.0, speed_mm_s=10.0, accel_g=0.1, timeout=0.2)
                    self.assertIn("iai_actuator_driver: decel stop sent after this error", ctx.exception.__notes__)
                    self.assertEqual(coil_frames(rec.tx), [STOP_ON, STOP_OFF])
                    self.assertFalse(any(f[1] in (0x45, 0x48) for f in rec.tx))
                finally:
                    ec.close()

    def test_stop_works_when_the_ec_rejects_iai_commands(self) -> None:
        class NoIai(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                if frame[1] == 0x48:
                    raise SimException(0x01)  # the EC rejects the IAI command
                return NotImplemented  # type: ignore[return-value]

        sim = NoIai(time_scale=5.0)
        ec = Elecylinder(ModbusRTU(sim), stroke=(0.0, 150.0), strategy="iai2", poll_interval_s=0.005)
        ec.move_to(140.0, speed_mm_s=50.0, wait=False)
        time.sleep(0.05)
        st = ec.stop(wait=True)
        self.assertFalse(st.moving)
        self.assertFalse(sim.moving)


class ContextExitTest(unittest.TestCase):
    """Leaving the context sends STOP only if this object commanded motion."""

    def test_read_only_use_sends_no_stop(self) -> None:
        sim = SimulatedEC()
        with self.assertRaises(RuntimeError):
            with Elecylinder.open(sim) as ec:
                ec.status()
                ec.identify()
                raise RuntimeError("user code failed")
        self.assertEqual([f for f in sim.requests if f[1] != 0x03 and f[1] != 0x11], [])
        self.assertTrue(sim.closed)

    def test_soft_limit_error_sends_nothing(self) -> None:
        sim = SimulatedEC()
        with self.assertRaises(SoftLimitError):
            with Elecylinder.open(sim, stroke=(0.0, 150.0)) as ec:
                ec.move_to(500.0)
        self.assertFalse(any(f[1] in (0x05, 0x06, 0x10) for f in sim.requests))

    def test_finished_move_then_error_sends_no_stop(self) -> None:
        sim = SimulatedEC(time_scale=50.0)
        with self.assertRaises(RuntimeError):
            with Elecylinder.open(sim, poll_interval_s=0.005) as ec:
                ec.move_to(80.0)
                raise RuntimeError("later failure")
        self.assertNotIn("stop", sim.events)

    def test_pending_motion_is_stopped(self) -> None:
        sim = SimulatedEC(time_scale=5.0)
        with self.assertRaises(RuntimeError):
            with Elecylinder.open(sim, poll_interval_s=0.005) as ec:
                ec.move_to(140.0, speed_mm_s=20.0, wait=False)
                raise RuntimeError("user code failed")
        self.assertIn("stop", sim.events)
        self.assertFalse(sim.stop_held)


class PauseTest(unittest.TestCase):
    """STP is a level input; the driver refuses motion while paused."""

    def test_paused_axis_refuses_moves(self) -> None:
        ec, sim, rec = make_ec()
        try:
            ec.pause()
            st = ec.status()
            self.assertTrue(st.paused)
            self.assertFalse(st.ready)
            self.assertFalse(st.idle)
            rec.clear()
            with self.assertRaisesRegex(EcNotReadyError, "paused"):
                ec.move_to(90.0)
            with self.assertRaisesRegex(EcNotReadyError, "paused"):
                ec.home()
            self.assertFalse(any(f[1] in (0x05, 0x10) for f in rec.tx))
            ec.resume()
            self.assertEqual(ec.move_to(90.0, speed_mm_s=50.0).position_mm, 90.0)
        finally:
            ec.close()

    def test_no_position_table_write_while_paused_mid_move(self) -> None:
        ec, sim, _ = make_ec(time_scale=5.0)
        try:
            entry = replace(ec.read_position_table(2), position_mm=60.0, speed_mm_s=50.0)
            ec.move_to(140.0, speed_mm_s=20.0, wait=False)
            time.sleep(0.05)
            ec.pause()
            time.sleep(0.05)
            st = ec.status()
            self.assertTrue(st.paused)
            self.assertFalse(st.move_bit)  # MOVE is 0 while paused...
            with self.assertRaisesRegex(EcNotReadyError, "paused|stop the axis"):
                ec.write_position_table(2, entry, allow_nonvolatile=True)  # ...but that is not idle
            self.assertEqual(sim.nonvolatile_writes, 0)
            ec.resume()
            self.assertEqual(ec.wait_idle(140.0, 0.1, timeout=20.0).position_mm, 140.0)
        finally:
            ec.close()


class StatusDecodeTest(unittest.TestCase):
    """``moving`` stays true while MOVE is already clear in the deceleration (VERIFIED), and the status flags."""

    @staticmethod
    def status(dss1: int, dsse: int) -> Status:
        return Status.from_registers([0, 7395, 0, 0, 0, dss1, 0x8000, dsse])

    def test_move_bit_clear_during_decel_still_moving(self) -> None:
        st = self.status(0x3090, 0x33C2)  # servo on, homed, PEND 0, MOVE 0: decelerating
        self.assertTrue(st.moving)
        self.assertFalse(st.move_bit)
        self.assertFalse(st.idle)

    def test_flags(self) -> None:
        paused = self.status(0x30B0, 0x33C2)
        self.assertTrue(paused.paused and not paused.moving and not paused.idle)
        creep = self.status(0x3080, 0x39C2)  # homing creep: MOVE 0, GHMS 1
        self.assertTrue(creep.moving and creep.homing)
        contact = self.status(0x7099, 0x37C3)  # live after push contact
        self.assertTrue(contact.push_contact and contact.pushing and contact.target_reached and contact.safety_speed)
        self.assertFalse(contact.moving)
        missed = self.status(0x3890, 0x33C2)  # PSFL, PEND 0
        self.assertTrue(missed.push_missed)
        self.assertFalse(missed.moving)
        self.assertTrue(missed.idle)
        data = contact.to_dict()
        for key in ("push_contact", "pushing", "push_missed", "homing", "paused", "safety_speed", "target_reached",
                    "move_bit", "idle", "vnow_mm_s"):  # fmt: skip
            self.assertIn(key, data)

    def test_vnow_scaling(self) -> None:
        words = [0, 7395, 0, 0, 0, 0x3090, 0x8000, 0x33F2, 0, 0, 0, 1248, 0, 250]
        st = Status.from_registers(words)
        self.assertEqual(st.vnow_mm_s, 12.48)
        self.assertEqual(st.velocity_mm_s, 12.48)


if __name__ == "__main__":
    unittest.main()
