"""The non-volatile write guard (simulator only).

The units' non-volatile memory lasts about 100,000 write cycles (IAI). Parameter commits, stored-row writes and
teaching skip a write that would change nothing (``force=True`` writes anyway), every axis counts its non-volatile
writes, and more than ``NV_WARN_PER_HOUR`` in the last hour logs one WARNING per hour (never a refusal).
"""

from __future__ import annotations

import json
import unittest
from dataclasses import replace

from ec_testutil import FrameRecorder, make_ec
from test_cli import SimFactory, run

from iai_actuator_driver import registers as R
from iai_actuator_driver import testing
from iai_actuator_driver.ec import NV_SKIP_NOTE, NV_WARN_PER_HOUR, NV_WINDOW_S, Elecylinder
from iai_actuator_driver.errors import ModbusTimeout, NonVolatileWriteError, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC


def open_sim(profile: str, sim: SimulatedEC | None = None) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = SimulatedEC(profile=profile, time_scale=50.0) if sim is None else sim
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), max_speed_mm_s=100.0, max_accel_g=0.3,
                     poll_interval_s=0.005, owns_bus=True)  # fmt: skip
    return ec, sim, rec


def fc10(rec: FrameRecorder) -> list[bytes]:
    return [f for f in rec.tx if f[1] == 0x10]


def reopen_axis(sim: SimulatedEC) -> Elecylinder:
    """A new axis object on the same simulated unit (a reconnect, or a new object per job, in this process)."""
    sim.reopen()
    return Elecylinder(ModbusRTU(sim, min_gap_s=0.0), max_speed_mm_s=700.0, max_accel_g=0.3, poll_interval_s=0.005,
                       owns_bus=True)  # fmt: skip


class LosesParw(SimulatedEC):
    """While ``lose_parw``, the PARW coil frame is lost on the line: nothing commits and nothing answers."""

    lose_parw = True

    def on_request(self, frame: bytes) -> bytes | None:
        if self.lose_parw and frame[1] == 0x05 and int.from_bytes(frame[2:4], "big") == R.COIL_PARW:
            return None
        return NotImplemented  # type: ignore[return-value]


class SameUnit:
    """Stands in for ``cli.SimulatedEC``: every CLI run reaches the same simulated unit (reopened)."""

    def __init__(self) -> None:
        self.sim: SimulatedEC | None = None

    def __call__(self, **kwargs: object) -> SimulatedEC:
        if self.sim is None:
            self.sim = SimulatedEC(**kwargs)  # type: ignore[arg-type]
        else:
            self.sim.reopen()
        return self.sim


class ParameterGuardTest(unittest.TestCase):
    def test_unchanged_value_writes_and_commits_nothing(self) -> None:
        ec, sim, rec = open_sim("gripper")
        with ec:
            rec.clear()
            result = ec.write_parameter(2, 0.10)  # P2 is 0.10 on the simulated gripper
            self.assertEqual((result.skipped, result.changed, result.committed), (True, False, False))
            self.assertEqual(result.note, NV_SKIP_NOTE)
            self.assertTrue(result.saved)  # the unit already holds it
            self.assertIn("already holds", result.takes_effect)
            self.assertEqual(fc10(rec), [])
            self.assertEqual((sim.parameter_commits, ec.nonvolatile_writes), (0, 0))
            self.assertTrue(result.to_dict()["skipped"])
            self.assertFalse(result.needs_reset)  # nothing written, nothing new to apply
            self.assertFalse(result.to_dict()["needs_reset"])

    def test_force_commits_an_unchanged_value(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            result = ec.write_parameter(2, 0.10, force=True)
        self.assertEqual((result.skipped, result.committed, result.commit_confirmed), (False, True, True))
        self.assertTrue(result.forced)  # the public result says it was forced (the field is last: positional use holds)
        self.assertTrue(result.to_dict()["forced"])
        self.assertTrue(result.needs_reset)
        self.assertEqual((sim.parameter_commits, ec.nonvolatile_writes), (1, 1))

    def test_a_changed_value_is_committed_and_counted(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            result = ec.write_parameter(2, 0.20)
        self.assertEqual((result.skipped, result.changed, result.committed), (False, True, True))
        self.assertEqual((sim.parameter_commits, ec.nonvolatile_writes, ec.nonvolatile_writes_last_hour), (1, 1, 1))

    def test_earlier_ram_only_writes_are_still_committed(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            ram = ec.write_parameter(5, "1:Enable", commit=False)  # RAM only: waits for a commit
            self.assertEqual((ram.changed, ram.committed, ram.skipped), (True, False, False))
            result = ec.write_parameter(2, 0.10)  # unchanged, but the commit saves P5
            self.assertEqual((result.skipped, result.changed, result.committed), (False, False, True))
            self.assertTrue(result.needs_reset)  # the commit saved P5, which still waits for its reset
            self.assertEqual(sim.parameter_commits, 1)
            self.assertTrue(ec.write_parameter(2, 0.10).skipped)  # nothing left uncommitted now
            self.assertEqual((sim.parameter_commits, ec.nonvolatile_writes), (1, 1))

    def test_unchanged_ram_only_write_is_not_pending(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            self.assertFalse(ec.write_parameter(2, 0.10, commit=False).skipped)  # nothing asked of the memory
            self.assertTrue(ec.write_parameter(2, 0.10).skipped)  # nothing was written to RAM either
        self.assertEqual(sim.parameter_commits, 0)

    def test_raw_item_and_con_parameter(self) -> None:
        ec, sim, rec = open_sim("gripper")
        with ec:
            raw = ec.read_parameter("8.9")["unsigned"]
            rec.clear()
            result = ec.write_parameter("8.9", raw, allow_dangerous=True)
            self.assertTrue(result.skipped)
            self.assertEqual((fc10(rec), sim.parameter_commits), ([], 0))
        ec, sim, _ = make_ec(profile="pcon")
        with ec:
            prm = next(p for p in ec.parameters() if p["writable"] and not p["dangerous"])
            result = ec.write_parameter(prm["number"], prm["value"])
            self.assertEqual((result.skipped, result.committed, sim.parameter_commits), (True, False, 0))
            self.assertFalse(any("software reset" in w for w in result.warnings))
            forced = ec.write_parameter(prm["number"], prm["value"], force=True)
            self.assertEqual((forced.committed, sim.parameter_commits, ec.nonvolatile_writes), (True, 1, 1))


class PendingCommitTest(unittest.TestCase):
    """A parameter written to RAM stays pending until a commit is confirmed, across axis objects of the process."""

    def test_an_unconfirmed_commit_is_committed_again(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50.0, parw_latches=False)  # PARW answered, never latched
        ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0), max_speed_mm_s=100.0, max_accel_g=0.3, poll_interval_s=0.005,
                         owns_bus=True)  # fmt: skip
        with ec:
            first = ec.write_parameter(2, 0.20)
            self.assertEqual((first.committed, first.commit_confirmed, first.saved), (True, False, False))
            sim.parw_latches = True
            retry = ec.write_parameter(2, 0.20)  # the same value: the commit runs, it is not skipped
            self.assertEqual((retry.skipped, retry.changed, retry.committed, retry.saved), (False, False, True, True))
            self.assertEqual(sim.parameter_commits, 1)
            self.assertTrue(ec.write_parameter(2, 0.20).skipped)  # confirmed now: nothing pending
        sim.power_cycle()
        with reopen_axis(sim) as again:
            self.assertEqual(again.read_parameter(2)["text"], "0.20 mm")

    def test_a_lost_parw_coil_is_committed_again(self) -> None:
        sim = LosesParw(profile="gripper", time_scale=50.0)
        ec, _, _ = open_sim("gripper", sim)
        with ec:
            with self.assertRaises(ModbusTimeout):
                ec.write_parameter(2, 0.20)
            self.assertEqual(sim.parameter_commits, 0)
            sim.lose_parw = False
            retry = ec.write_parameter(2, 0.20)
            self.assertEqual((retry.skipped, retry.committed, retry.saved), (False, True, True))
            self.assertEqual(sim.parameter_commits, 1)

    def test_a_ram_only_write_is_seen_by_a_new_axis_object(self) -> None:
        ec, sim, _ = open_sim("gripper")
        with ec:
            self.assertFalse(ec.write_parameter(2, 0.20, commit=False).committed)
        with reopen_axis(sim) as again:  # a reconnect in the same process
            result = again.write_parameter(2, 0.20)
            self.assertEqual((result.skipped, result.committed, result.saved), (False, True, True))
        self.assertEqual(sim.parameter_commits, 1)


class PositionGuardTest(unittest.TestCase):
    def test_unchanged_row_is_not_written(self) -> None:
        ec, sim, rec = make_ec(max_speed_mm_s=700.0)  # row 2 runs at 248.50 mm/s
        with ec:
            stored = ec.read_position_table(2)
            rec.clear()
            back = ec.write_position_table(2, stored, allow_nonvolatile=True)
            self.assertTrue(back.skipped)
            self.assertEqual(back, stored)  # the stored row (skipped does not take part in equality)
            self.assertEqual(fc10(rec), [])
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (0, 0))
            self.assertFalse(ec.read_position_table(2).skipped)

    def test_changed_or_forced_row_is_written_and_counted(self) -> None:
        ec, sim, _ = make_ec(max_speed_mm_s=700.0)  # row 2 runs at 248.50 mm/s
        with ec:
            stored = ec.read_position_table(2)
            back = ec.write_position_table(2, replace(stored, position_mm=60.0), allow_nonvolatile=True)
            self.assertFalse(back.skipped)
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (1, 1))
            again = ec.write_position_table(2, back, allow_nonvolatile=True, force=True)
            self.assertFalse(again.skipped)
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (2, 2))

    def test_guards_still_come_first(self) -> None:
        ec, sim, _ = make_ec()
        with ec:
            with self.assertRaisesRegex(NonVolatileWriteError, "simple-data"):  # refused, not "skipped"
                ec.write_position_table(256, ec.read_position_table(0))
            with self.assertRaises(SoftLimitError):
                ec.write_position_table(0, replace(ec.read_position_table(0), position_mm=151.0))
            self.assertTrue(ec.write_position_table(0, ec.read_position_table(0)).skipped)  # no opt-in needed
        self.assertEqual(sim.nonvolatile_writes, 0)

    def test_teach_table_skips_a_row_that_holds_the_position(self) -> None:
        ec, sim, _ = make_ec(max_speed_mm_s=700.0)
        with ec:
            ec.move_to(60.0, speed_mm_s=50.0)
            self.assertFalse(ec.teach_position(0, allow_nonvolatile=True).skipped)
            again = ec.teach_position(0, allow_nonvolatile=True)
            self.assertTrue(again.skipped)
            self.assertEqual(again.position_mm, 60.0)
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (1, 1))
            self.assertFalse(ec.teach_position(0, allow_nonvolatile=True, force=True).skipped)
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (2, 2))

    def test_teach_coil_skips_a_complete_row_that_holds_the_position(self) -> None:
        ec, sim, rec = make_ec(profile="pcon")
        with ec:
            position = ec.status().position_mm
            blank = ec.teach_position(5, allow_nonvolatile=True, method="coil")  # an empty row: taught
            self.assertFalse(blank.skipped)
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (1, 1))
            row = replace(ec.read_position_table(5), position_mm=position, speed_mm_s=50.0, band_mm=0.1,
                          accel_g=0.3, decel_g=0.3)  # fmt: skip
            ec.write_position_table(5, row, allow_nonvolatile=True)
            self.assertEqual(ec.nonvolatile_writes, 2)
            rec.clear()
            entry = ec.teach_position(5, allow_nonvolatile=True, method="coil")
            self.assertTrue(entry.skipped)
            self.assertEqual([f for f in rec.tx if f[1] in (0x05, 0x06)], [])  # no MOD / POSR / TEAC
            self.assertEqual((sim.nonvolatile_writes, ec.nonvolatile_writes), (2, 2))
            self.assertFalse(ec.teach_position(5, allow_nonvolatile=True, method="coil", force=True).skipped)
            self.assertEqual(ec.nonvolatile_writes, 3)

    def test_teach_coil_teaches_a_row_with_a_bad_checksum(self) -> None:
        for method in ("table", "coil"):  # the same rule for both methods
            ec, sim, _ = make_ec(profile="pcon")
            with ec:
                row = replace(ec.read_position_table(5), position_mm=ec.status().position_mm, speed_mm_s=50.0,
                              band_mm=0.1, accel_g=0.3, decel_g=0.3)  # fmt: skip
                ec.write_position_table(5, row, allow_nonvolatile=True)
                damaged = sim.position_row(5)
                damaged[R.POS_CHECKSUM] ^= 0x0001  # the checksum word only
                sim.set_position_row(5, damaged)
                self.assertIs(ec.read_position_table(5).checksum_ok, False)
                entry = ec.teach_position(5, allow_nonvolatile=True, method=method)
                self.assertFalse(entry.skipped, method)
                self.assertIs(ec.read_position_table(5).checksum_ok, True, method)
                self.assertEqual(sim.nonvolatile_writes, 2, method)


class RateWarningTest(unittest.TestCase):
    def test_one_warning_per_hour_and_never_a_refusal(self) -> None:
        with testing.virtual_clock() as clock:  # the axis, the bus and the unit on one clock: an hour passes at once
            ec, sim, _ = make_ec(max_speed_mm_s=700.0, clock=clock.perf_counter)  # row 2 runs at 248.50 mm/s
            with ec:
                stored = ec.read_position_table(2)
                with self.assertNoLogs("iai_actuator_driver.ec", "WARNING"):
                    for _ in range(NV_WARN_PER_HOUR):
                        ec.write_position_table(2, stored, allow_nonvolatile=True, force=True)
                with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                    ec.write_position_table(2, stored, allow_nonvolatile=True, force=True)
                [message] = logs.output
                self.assertIn(f"{NV_WARN_PER_HOUR + 1} non-volatile writes in the last hour on slave 1", message)
                self.assertIn("about 100,000 write cycles (about 68 days at this rate)", message)
                self.assertIn("use move_to (direct moves are RAM only)", message)
                with self.assertNoLogs("iai_actuator_driver.ec", "WARNING"):  # one per hour, the writes still go through
                    written = ec.write_position_table(2, replace(stored, position_mm=61.0), allow_nonvolatile=True)
                self.assertFalse(written.skipped)
                self.assertEqual(sim.nonvolatile_writes, NV_WARN_PER_HOUR + 2)
                self.assertEqual(ec.nonvolatile_writes, NV_WARN_PER_HOUR + 2)
                # An hour later the old writes have left the window and the warning may come again.
                clock.advance(NV_WINDOW_S + 1)
                self.assertEqual(ec.nonvolatile_writes_last_hour, 0)
                with self.assertNoLogs("iai_actuator_driver.ec", "WARNING"):
                    ec.write_position_table(2, stored, allow_nonvolatile=True, force=True)
                self.assertEqual(ec.nonvolatile_writes_last_hour, 1)

    def test_counted_per_unit_across_axis_objects(self) -> None:
        ec, sim, _ = make_ec(max_speed_mm_s=700.0)  # row 2 runs at 248.50 mm/s
        with ec, self.assertNoLogs("iai_actuator_driver.ec", "WARNING"):
            stored = ec.read_position_table(2)
            for _ in range(NV_WARN_PER_HOUR - 1):
                ec.write_position_table(2, stored, allow_nonvolatile=True, force=True)
        for i in range(2):  # open, write a row, close: one object per job
            with reopen_axis(sim) as job:
                stored = job.read_position_table(2)
                if i == 0:
                    with self.assertNoLogs("iai_actuator_driver.ec", "WARNING"):
                        job.write_position_table(2, replace(stored, position_mm=60.0), allow_nonvolatile=True)
                else:
                    with self.assertLogs("iai_actuator_driver.ec", "WARNING") as logs:
                        job.write_position_table(2, replace(stored, position_mm=61.0), allow_nonvolatile=True)
                    self.assertIn(f"{NV_WARN_PER_HOUR + 1} non-volatile writes in the last hour", logs.output[0])
                self.assertEqual(job.nonvolatile_writes, NV_WARN_PER_HOUR + i)
                self.assertEqual(job.nonvolatile_writes_last_hour, NV_WARN_PER_HOUR + i)
        self.assertEqual(sim.nonvolatile_writes, NV_WARN_PER_HOUR + 1)
        other, _, _ = make_ec()  # another unit: its own count
        with other:
            self.assertEqual(other.nonvolatile_writes, 0)

    def test_counters_are_read_only(self) -> None:
        ec, _, _ = make_ec()
        with ec:
            with self.assertRaises(AttributeError):
                ec.nonvolatile_writes = 5  # type: ignore[misc]


def row2_at_the_current_position(sim: SimulatedEC) -> None:
    """Row 2 of the simulated slider holds PNOW (73.95 mm), its other fields as they are."""
    row = sim.position_row(2)
    row[0:2] = R.to_i32_words(7395)
    row[R.POS_CHECKSUM] = R.position_checksum(row[:15])
    sim.set_position_row(2, row)


class CliTest(unittest.TestCase):
    def test_no_commit_then_param_set_in_a_new_process(self) -> None:
        unit = SameUnit()
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.20", "--no-commit",
                             factory=unit)  # fmt: skip
        self.assertEqual(code, 0, err)
        self.assertIn("'param set 2 0.20 --force'", out)
        assert unit.sim is not None
        testing.forget_nonvolatile_state(unit.sim)  # each CLI run is a new process: it cannot see the RAM-only write
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.20", factory=unit)
        self.assertEqual(code, 0, err)
        self.assertIn(NV_SKIP_NOTE, out)
        self.assertIn("earlier --no-commit run left this value in RAM only", out)
        self.assertEqual(unit.sim.parameter_commits, 0)
        testing.forget_nonvolatile_state(unit.sim)
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.20", "--force", factory=unit)
        self.assertEqual(code, 0, err)
        self.assertIn("committed to non-volatile memory", out)
        unit.sim.power_cycle()
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "get", "2", factory=unit)
        self.assertEqual(json.loads(out)["text"], "0.20 mm")

    def test_param_set_prints_the_note_and_takes_force(self) -> None:
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.1")
        self.assertEqual(code, 0, err)
        self.assertIn(NV_SKIP_NOTE, out)
        self.assertIn("--force", out)
        self.assertNotIn("committed to non-volatile memory", out)
        code, out, err = run("--sim", "--sim-profile", "gripper", "param", "set", "2", "0.1", "--force")
        self.assertEqual(code, 0, err)
        self.assertIn("committed to non-volatile memory", out)

    def test_teach_prints_the_note_and_takes_force(self) -> None:
        factory = SimFactory(setup=row2_at_the_current_position)
        code, out, err = run("--sim", "--unit-max", "teach", "2", "--allow-nonvolatile", factory=factory)
        self.assertEqual(code, 0, err)
        self.assertIn(f"row 2 already holds the current position 73.95 mm: {NV_SKIP_NOTE}", out)
        self.assertEqual(factory.instances[-1].nonvolatile_writes, 0)
        code, out, err = run("--sim", "--unit-max", "teach", "2", "--allow-nonvolatile", "--force", factory=factory)
        self.assertEqual(code, 0, err)
        self.assertIn("row 2 taught from the current position: 73.95 mm", out)
        self.assertEqual(factory.instances[-1].nonvolatile_writes, 1)


if __name__ == "__main__":
    unittest.main()
