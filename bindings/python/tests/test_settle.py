"""Settle rules, from motion recorded on the EC-GRBP8M gripper (VERIFIED).

1. A move, the STOP hold and iai-check count as done only once PNOW has stopped changing: on the
   gripper PEND + MEND (DSS1 0x3099) were set at 3.04 mm with the next sample at 3.00 mm, and an
   iai-check move "reached" 7.91 mm for a target of 8.00 and read 7.97 mm next.
2. A push outcome (contact or miss) must hold, position unchanged, for ``push_settle_s``: in a 30 %
   push with a 5 mm band the full contact fingerprint (PEND + MEND, DSSE 0x37C3) appeared at 6.76 mm
   and the push then missed at 7.00 mm.

The simulator reproduces both (``early_in_position`` with its settling tail, ``push_blip_mm``); a
stepped clock makes every poll see the axis a fixed time later, so these tests do not depend on the
host's speed.
"""

from __future__ import annotations

import unittest

from ec_testutil import FrameRecorder, SteppedClockSim

from iai_actuator_driver import registers as R
from iai_actuator_driver.check import PASS, run_check
from iai_actuator_driver.ec import Elecylinder, Status
from iai_actuator_driver.modbus import ModbusRTU


class StatusRecorder(FrameRecorder):
    """Frame logger that also decodes every status read on the bus (FC03 at 0x9000, 8 to 22 words) into
    ``statuses``: the samples a wait saw, also one the driver runs inside a single call."""

    def __init__(self) -> None:
        super().__init__()
        self.statuses: list[Status] = []
        self._status_read = False

    def __call__(self, direction: str, frame: bytes) -> None:
        super().__call__(direction, frame)
        if direction == "tx":
            self._status_read = frame[1] == 0x03 and int.from_bytes(frame[2:4], "big") == R.MONITOR_BASE
        elif direction == "rx" and self._status_read:
            self._status_read = False
            if frame[1] == 0x03:  # not an exception reply
                data = frame[3 : 3 + frame[2]]
                words = [int.from_bytes(data[i : i + 2], "big") for i in range(0, len(data), 2)]
                if len(words) >= R.STATUS_BLOCK_MIN_WORDS:
                    self.statuses.append(Status.from_registers(words))

    def clear(self) -> None:
        super().clear()
        self.statuses.clear()


def open_recording(sim: SteppedClockSim, **kwargs: object) -> tuple[Elecylinder, list[Status], StatusRecorder]:
    """Elecylinder on ``sim`` whose status samples from here on are recorded (decoded from the bus's frames)."""
    rec = StatusRecorder()
    kwargs.setdefault("poll_interval_s", 0.002)
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), owns_bus=True, **kwargs)  # type: ignore[arg-type]
    rec.clear()
    return ec, rec.statuses, rec


class MoveSettleTest(unittest.TestCase):
    def test_move_returns_the_settled_position(self) -> None:
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
        ec, seen, _ = open_recording(sim)
        with ec:
            st = ec.move_to(8.0, speed_mm_s=10.0)  # the iai-check move of the gripper report
        early = [s for s in seen if s.in_position and s.target_reached and 7.9 <= s.position_mm < 8.0]
        self.assertTrue(early, "the poller never saw PEND + MEND before standstill")
        self.assertEqual(st.position_mm, 8.0)
        self.assertFalse(sim.moving)
        self.assertTrue(st.in_position and st.target_reached)

    def test_without_settling_the_early_sample_is_taken(self) -> None:
        """Without the settle check (settle_s = 0) the result is a position the axis is still leaving."""
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
        ec, _, _ = open_recording(sim, settle_s=0.0)
        with ec:
            st = ec.move_to(8.0, speed_mm_s=10.0)
            self.assertLess(st.position_mm, 8.0)
            self.assertTrue(sim.moving, "the axis was still moving when the move was reported done")

    def test_home_waits_for_standstill(self) -> None:
        sim = SteppedClockSim(step_s=0.01, position_mm=5.0)
        ec, seen, _ = open_recording(sim)
        with ec:
            st = ec.home()
        self.assertEqual(st.position_mm, 0.0)
        self.assertFalse(sim.moving)
        final = [s for s in seen if s.homed and s.in_position and not s.homing]
        self.assertGreaterEqual(len(final), 2, "home completion accepted on a single poll")

    def test_stop_hold_waits_for_standstill(self) -> None:
        sim = SteppedClockSim(step_s=0.01, position_mm=20.0)
        ec, seen, rec = open_recording(sim)
        with ec:
            ec.move_to(140.0, speed_mm_s=50.0, wait=False)
            for _ in range(20):
                ec.status()  # under way
            st = ec.stop(wait=True)
            self.assertFalse(sim.moving, "STOP released while the axis was still moving")
            self.assertEqual(st.position_mm, round(sim.position_mm, 2))
        stop_writes = [f for f in rec.tx if f[1] == 0x05 and int.from_bytes(f[2:4], "big") == R.COIL_STOP]
        self.assertEqual([f[4] == 0xFF for f in stop_writes], [True, False])

    def test_ec_check_reports_the_settled_position(self) -> None:
        sim = SteppedClockSim(step_s=0.01, profile="gripper", position_mm=3.0)
        ec, _, _ = open_recording(sim)
        with ec:
            report = run_check(ec, motion=True, port_label="sim")
        items = {item.name: item for item in report.items}
        self.assertTrue(report.passed, [(i.name, i.result, i.detail) for i in report.items])
        # the step is at most a quarter of the 9.50 mm stroke: 3.00 + 2.38, toward the side with more room
        for name, goal in (("motion_out", 5.38), ("motion_back", 3.0)):
            item = items[name]
            self.assertEqual(item.result, PASS)
            self.assertEqual(item.data["target_mm"], goal)
            self.assertEqual(item.data["reached_mm"], goal)  # an unsettled sample read 7.91 for 8.00 on the gripper
            self.assertEqual(item.data["error_mm"], 0.0)
            self.assertIs(item.data["settled"], True)
            self.assertIn("settled", item.detail)


class PushSettleTest(unittest.TestCase):
    def make(self, **kwargs: object) -> tuple[SteppedClockSim, Elecylinder, list[Status]]:
        sim = SteppedClockSim(step_s=0.005, profile="gripper", position_mm=1.0, push_blip_mm=6.76)
        ec, seen, _ = open_recording(sim, **kwargs)
        return sim, ec, seen

    def test_transient_contact_blip_is_a_miss(self) -> None:
        sim, ec, seen = self.make()
        with ec:
            result = ec.push(2.0, 5.0, 30.0, +1, speed_mm_s=10.0)  # as on the gripper: push + 30 %, band 5 to 7.00 mm
        blips = [s for s in seen if s.push_contact]
        self.assertTrue(blips, "the poller never saw the transient contact fingerprint")
        for s in blips:
            self.assertEqual(s.raw["DSSE"], 0x37C3)
            self.assertEqual(s.raw["DSS1"] & 0x0809, 0x0009)
        self.assertEqual(result.outcome, "missed")
        self.assertEqual(result.position_mm, 7.0)
        self.assertTrue(result.status.push_missed)
        self.assertFalse(sim.moving)

    def test_without_settling_the_blip_reads_as_contact(self) -> None:
        """Without the settle rule (push_settle_s = 0) an empty grip reads as "contact"."""
        sim, ec, _ = self.make(push_settle_s=0.0)
        with ec:
            result = ec.push(2.0, 5.0, 30.0, +1, speed_mm_s=10.0)
            self.assertEqual(result.outcome, "contact")
            self.assertTrue(6.76 <= result.position_mm <= 6.86, result.position_mm)
            self.assertTrue(sim.moving)
            ec.stop()

    def test_real_contact_still_reported(self) -> None:
        sim, ec, _ = self.make()
        sim.push_blip_mm = None
        sim.obstacle_mm = 5.0
        with ec:
            result = ec.push(2.0, 5.0, 30.0, +1, speed_mm_s=10.0)
            self.assertEqual(result.outcome, "contact")
            self.assertEqual(result.position_mm, 5.0)
            self.assertTrue(result.status.push_contact)
            ec.move_to(1.0, speed_mm_s=10.0)  # retract ends the hold


if __name__ == "__main__":
    unittest.main()
