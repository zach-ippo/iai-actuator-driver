"""Findings of the full feature test on the EC-GRBP8M gripper (2-position mode, P9 = 0), simulated.

1. Stored rows follow the position mode: row 2 is not run in 2-position mode (the gripper rejected 0x45 cmd 6 row 2
   with 0x11 "1st operand error"); the driver refuses a move to it before anything is sent and translates the
   rejection; a write or teach of it goes through with a warning.
2. A same-value teach / row write is skipped before the speed cap (a row stored at 53 mm/s on a 10 mm/s axis); a
   refused value names the row, the field, its unit and whose cap it is.
3. A software reset keeps PMSS 1 on the gripper; the restart is confirmed by the servo coming back on by itself.
"""

from __future__ import annotations

import contextlib
import dataclasses
import io
import unittest

from ec_testutil import FrameRecorder

from iai_actuator_driver import cli
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import IaiCommandRejected, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC

INACTIVE = "row 2 (middle) is not used in 2-position mode (P9 = 0"


def open_axis(
    profile: str = "gripper", *, sim: SimulatedEC | None = None, **kwargs: object
) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = sim or SimulatedEC(profile=profile, time_scale=50.0)
    rec = FrameRecorder()
    kwargs.setdefault("poll_interval_s", 0.005)
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), owns_bus=True, **kwargs)  # type: ignore[arg-type]
    return ec, sim, rec


def commands(rec: FrameRecorder) -> list[bytes]:
    """Frames that command the unit: FC05 / FC06 / FC10 writes and 0x45 / 0x48 commands."""
    return [f for f in rec.tx if f[1] in (0x05, 0x06, 0x10, 0x45, 0x48)]


def run_cli(*argv: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stderr(err):
        code = cli.main(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class PositionModeRowsTest(unittest.TestCase):
    def test_simulator_rejects_row_2_in_2_position_mode(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50.0)
        bus = ModbusRTU(sim, min_gap_s=0.0)
        with self.assertRaises(IaiCommandRejected) as caught:
            bus.iai_command(1, 0, R.IAI_CMD_MOVE_POSITION, 2, 46, retries=0)
        self.assertEqual(caught.exception.rejection, 0x11)
        self.assertIn("2-position mode", caught.exception.hint)
        self.assertEqual(sim.override_moves, 0)
        bus.iai_command(1, 0, R.IAI_CMD_MOVE_POSITION, 0, 46, retries=0)  # rows 0 and 1 still run
        self.assertEqual(sim.override_moves, 1)

    def test_row_2_refused_before_anything_is_sent(self) -> None:
        ec, sim, rec = open_axis()
        with ec:
            ec.connect(take_control=True)
            for kwargs in ({}, {"speed_pct": 18}):
                rec.clear()
                with self.assertRaises(SoftLimitError) as caught:
                    ec.move_to_position(2, **kwargs)
                self.assertIn(INACTIVE, str(caught.exception))
                self.assertIn("runs row 0 (backward end) and row 1 (forward end) only", str(caught.exception))
                self.assertIn("nothing sent", str(caught.exception))
                self.assertEqual(commands(rec), [])
            st = ec.move_to_position(0, speed_pct=18)  # an active row still runs
            self.assertAlmostEqual(st.position_mm, 3.0, delta=0.01)
        self.assertEqual(sim.override_moves, 1)

    def test_rejection_translated_when_the_mode_is_unknown(self) -> None:
        # P9 not readable (the unit's directory lists no item 8.2): the row is sent
        ec, sim, _ = open_axis(sim=SimulatedEC(profile="gripper", time_scale=50.0, missing_items=[R.POSITION_MODE_ITEM]))
        with ec:
            ec.connect(take_control=True)
            with self.assertRaises(SoftLimitError) as caught:
                ec.move_to_position(2, speed_pct=18, allow_above_caps=True)
            text = str(caught.exception)
            self.assertIn("row 2 (middle) is not used in the unit's position mode", text)
            self.assertIn("0x11", text)
            self.assertIsInstance(caught.exception.__cause__, IaiCommandRejected)
            self.assertAlmostEqual(ec.status().position_mm, 9.5, delta=0.01)  # nothing moved
        self.assertEqual(sim.override_moves, 0)

    def test_three_position_slider_runs_row_2(self) -> None:
        ec, sim, _ = open_axis("slider")
        with ec:
            st = ec.move_to_position(2, speed_pct=50, allow_above_caps=True)
        self.assertEqual(st.position_mm, 75.0)

    def test_con_unit_rows_are_not_limited_by_p9(self) -> None:
        ec, _, rec = open_axis("pcon")
        with ec:
            ec.identify()
            # no position mode applies (class 8 item 2 is another parameter on a CON unit): row 2 is sent, not refused
            rec.clear()
            ec.move_to_position(2)
            self.assertTrue(commands(rec))
            self.assertEqual(ec.max_positions(), 64)

    def test_teach_and_write_row_2_with_a_warning(self) -> None:
        ec, _, _ = open_axis()
        with ec:
            ec.connect(take_control=True)
            row2 = ec.read_position_table(2)
            with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                back = ec.write_position_table(2, dataclasses.replace(row2, position_mm=5.0))
            self.assertEqual(back.position_mm, 5.0)
            self.assertIn(INACTIVE, logs.output[0])
            self.assertIn("written anyway", logs.output[0])
            with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                taught = ec.teach_position(2)
            self.assertIn(INACTIVE, "\n".join(logs.output))
            self.assertAlmostEqual(taught.position_mm, ec.status().position_mm, delta=0.05)
            self.assertEqual(ec.nonvolatile_writes, 2)

    def test_cli_goto_2_is_refused(self) -> None:
        code, _, err = run_cli("--sim", "--sim-profile", "gripper", "--sim-time-scale", "50", "goto", "2",
                               "--speed-pct", "18")  # fmt: skip
        self.assertEqual(code, 2)
        self.assertIn(INACTIVE, err)


class UnchangedRowSkipTest(unittest.TestCase):
    """CHECK-1: the gripper's row 0 is stored at 53 mm/s; the axis is opened with a 10 mm/s cap."""

    def test_same_row_write_is_skipped_before_the_cap(self) -> None:
        ec, _, rec = open_axis(max_speed_mm_s=10.0)
        with ec:
            ec.connect(take_control=True)
            row0 = ec.read_position_table(0)
            self.assertEqual(row0.speed_mm_s, 53.0)
            rec.clear()
            self.assertTrue(ec.write_position_table(0, row0, allow_nonvolatile=True).skipped)
            self.assertEqual(commands(rec), [])
            with self.assertRaises(SoftLimitError) as caught:  # a real change: the cap applies, with context
                ec.write_position_table(0, dataclasses.replace(row0, position_mm=3.5), allow_nonvolatile=True)
            self.assertEqual(
                str(caught.exception),
                "row 0 stored speed 53 mm/s is above this axis's max_speed_mm_s 10 mm/s; open the axis with a higher "
                "max_speed_mm_s (command line: --max-speed or --unit-max) to write it",
            )
            with self.assertRaisesRegex(SoftLimitError, r"^row 0 speed 20 mm/s is above this axis's max_speed_mm_s"):
                ec.write_position_table(0, dataclasses.replace(row0, speed_mm_s=20.0), allow_nonvolatile=True)
            self.assertEqual(ec.nonvolatile_writes, 0)

    def test_same_value_teach_is_skipped_before_the_cap(self) -> None:
        ec, _, rec = open_axis(max_speed_mm_s=10.0)
        with ec:
            ec.connect(take_control=True)
            ec.move_to(3.0, speed_mm_s=10.0)  # row 0 holds 3.00 mm
            rec.clear()
            entry = ec.teach_position(0, allow_nonvolatile=True)
            self.assertTrue(entry.skipped)
            self.assertEqual(commands(rec), [])
            ec.move_to(3.5, speed_mm_s=10.0)
            with self.assertRaisesRegex(SoftLimitError, "row 0 stored speed 53 mm/s .*--unit-max"):
                ec.teach_position(0, allow_nonvolatile=True)
            self.assertEqual(ec.nonvolatile_writes, 0)

    def test_force_still_checks_every_value(self) -> None:
        ec, _, _ = open_axis(max_speed_mm_s=10.0)
        with ec:
            ec.connect(take_control=True)
            row0 = ec.read_position_table(0)
            with self.assertRaisesRegex(SoftLimitError, "max_speed_mm_s 10"):
                ec.write_position_table(0, row0, allow_nonvolatile=True, force=True)


class SoftwareResetPmssTest(unittest.TestCase):
    def test_gripper_keeps_pmss_and_is_confirmed_by_the_servo(self) -> None:
        ec, sim, _ = open_axis()
        with ec:
            ec.connect(take_control=True)
            result = ec.software_reset(reset_wait_s=0.0)
        self.assertTrue(result.confirmed)
        self.assertTrue(result.modbus_enabled)
        self.assertNotIn("PMSS 1 -> 0", result.confirmed_by)
        self.assertIn("the servo came back on by itself", result.confirmed_by)
        self.assertTrue(any("EC-GRBP8M" in note for note in result.notes))


if __name__ == "__main__":
    unittest.main()
