"""Adaptive behaviour and multi-axis safety (simulator only).

Ctrl+C during a cell move stops every axis at once; a cell move checks every axis before any starts; a CON rotary
axis (ATYP 7.7 bit 8, no AKID) is labelled in deg; a damaged FC11 answer never fails identify(); the position table
is addressed from the unit's own directory; the row count, the self-test step and the messages adapt to the unit.
"""

from __future__ import annotations

import _thread
import struct
import threading
import time
import unittest

from ec_testutil import FrameRecorder
from test_cli import run

from iai_actuator_driver import registers as R
from iai_actuator_driver.cell import Cell, CellError
from iai_actuator_driver.check import PASS, run_check
from iai_actuator_driver.ec import Elecylinder, PositionEntry
from iai_actuator_driver.errors import SoftLimitError, VerificationError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine

OPEN = {"poll_interval_s": 0.005, "scan_timeout": 0.01}


def open_axis(profile: str = "pcon", **kwargs: object) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    sim = SimulatedEC(profile=profile, time_scale=50.0, **kwargs)  # type: ignore[arg-type]
    rec = FrameRecorder()
    ec = Elecylinder(ModbusRTU(sim, frame_logger=rec, min_gap_s=0.0), poll_interval_s=0.005, owns_bus=True)
    return ec, sim, rec


class CellInterruptTest(unittest.TestCase):
    def test_ctrl_c_during_move_all_stops_every_axis_at_once(self) -> None:
        # a mixed line (slider EC + PCON), a 7 s move: Ctrl+C after 0.5 s must STOP both axes right away
        line = SimLine([SimulatedEC(slave=1, profile="slider", position_mm=0.0),
                        SimulatedEC(slave=2, profile="pcon", position_mm=0.0)], name="L")  # fmt: skip
        with Cell.open([line], **OPEN) as cell:
            timer = threading.Timer(0.5, _thread.interrupt_main)
            start = time.perf_counter()
            timer.start()
            try:
                with self.assertRaises(KeyboardInterrupt):
                    cell.move_all({"L:1": 140.0, "L:2": 140.0}, speed_mm_s=20, accel_g=0.3)
            finally:
                timer.cancel()
            elapsed = time.perf_counter() - start
            self.assertLess(elapsed, 2.5, "KeyboardInterrupt arrived only after the moves")
            for unit in line:
                self.assertFalse(unit.moving)
                self.assertLess(unit.position_mm, 60.0)
            stops = [f for _, f in line.requests if f[1] == 0x05 and struct.unpack(">H", f[2:4])[0] == R.COIL_STOP]
            self.assertTrue(stops, "no STOP was sent")


class CellPrecheckTest(unittest.TestCase):
    def test_one_bad_target_starts_no_axis(self) -> None:
        line = SimLine([SimulatedEC(slave=1, time_scale=5), SimulatedEC(slave=2, time_scale=5),
                        SimulatedEC(slave=3, profile="rotary", time_scale=5)], name="L")  # fmt: skip
        with Cell.open([line], **OPEN) as cell:
            before = {s: line[s].position_mm for s in (1, 2, 3)}
            sent_before = len(line.requests)
            with self.assertRaises(CellError) as cm:
                cell.move_all({"L:1": 100.0, "L:2": 100.0, "L:3": 9999.0})
            self.assertIsInstance(cm.exception.errors["L:3"], SoftLimitError)
            self.assertIn("nothing started", str(cm.exception))
            time.sleep(0.1)
            for s in (1, 2, 3):
                self.assertFalse(line[s].moving)
                self.assertEqual(line[s].position_mm, before[s])
            writes = [f for _, f in line.requests[sent_before:] if f[1] in (0x10, 0x17)]
            self.assertEqual(writes, [], "a move was sent before every axis was checked")

    def test_profile_move_is_checked_first_too(self) -> None:
        line = SimLine([SimulatedEC(slave=1, time_scale=5), SimulatedEC(slave=2, profile="pcon", time_scale=5)],
                       name="L")  # fmt: skip
        with Cell.open([line], **OPEN) as cell:
            with self.assertRaisesRegex(CellError, "deceleration"):
                cell.move_all({"L:1": 100.0, "L:2": 100.0}, decel_g={"L:1": 0.1, "L:2": 50.0})
            self.assertFalse(line[1].moving or line[2].moving)


class ConRotaryTest(unittest.TestCase):
    def test_atyp_rotary_on_a_con_controller_is_in_degrees(self) -> None:
        sim = SimulatedEC(profile="pcon", time_scale=50.0)
        ec = Elecylinder(ModbusRTU(sim, min_gap_s=0.0))
        self.assertEqual(ec.units, "mm")
        ec.write_parameter(78, 1, commit=False, allow_dangerous=True)  # No.78 Axis action type: 1 Rotary axis
        with Elecylinder(ModbusRTU(sim, min_gap_s=0.0), owns_bus=True) as ec:
            self.assertIsNone(ec.read_axis_limits().actuator_kind)  # a CON controller has no AKID
            self.assertEqual((ec.units, ec.kind, ec.limits()["units"]), ("deg", "rotary", "deg"))
            info = ec.identify()
            self.assertEqual((info.units, info.kind, info.rotary), ("deg", "rotary", True))
            self.assertTrue(any("ATYP 7.7 bit 8" in e for e in info.errors))
            offset = next(p for p in ec.parameters() if p["number"] == 22)  # Home offset
            self.assertEqual(offset["unit"], "deg")
            self.assertTrue(offset["text"].endswith(" deg"), offset["text"])


class IdentifyFaultTest(unittest.TestCase):
    def test_a_damaged_fc11_answer_is_retried_once(self) -> None:
        ec, sim, _ = open_axis("pcon")
        with ec:
            sim.inject_fault("crc")
            info = ec.identify()
        self.assertEqual((info.identity_source, info.product_family), ("FC11", "PCON"))

    def test_two_damaged_fc11_answers_fall_back_to_ctcl(self) -> None:
        ec, sim, _ = open_axis("pcon")
        with ec:
            sim.inject_fault("crc", 2)
            info = ec.identify()
        self.assertEqual((info.identity_source, info.product_family), ("CTCL 0x0300", "PCON"))
        self.assertTrue(any("report slave id" in e for e in info.errors))

    def test_cell_identify_survives_a_crc_error(self) -> None:
        line = SimLine([SimulatedEC(slave=1, profile="unknown"), SimulatedEC(slave=2, profile="rotary"),
                        SimulatedEC(slave=3, profile="pcon")], name="L1")  # fmt: skip
        with Cell.open([line], identify=False, **OPEN) as cell:
            line[3].inject_fault("crc", 2)
            infos = cell.identify_all()
            self.assertEqual(infos["L1:3"].product_family, "PCON")

    def test_every_unit_is_identified_without_a_model_label(self) -> None:
        # unit_known = the driver has an EC family name for the code (display only): False for the PCON-CB and for an
        # unnamed unit, and neither gets a note about its model
        for profile, family in (("pcon", "PCON"), ("unknown", "IAI controller")):
            with self.subTest(profile=profile):
                ec, _, _ = open_axis(profile)
                with ec:
                    info = ec.identify()
                self.assertFalse(info.unit_known)
                self.assertEqual(info.product_family, family)
                self.assertFalse(any("UNVERIFIED" in e or "tried on hardware" in e or "family table" in e
                                     for e in info.errors), info.errors)


class PositionTableLayoutTest(unittest.TestCase):
    def test_rows_are_addressed_from_the_directory(self) -> None:
        ec, _, rec = open_axis("pcon")
        with ec:
            ec.read_position_table(3)
            self.assertEqual(struct.unpack(">H", rec.tx[-1][2:4])[0], 0x1000 + 16 * 3)  # the directory's 16-word rows
        ec, _, rec = open_axis("pcon", row_words=20)  # 20-word rows: a layout the driver does not know
        with ec:
            ec.identify()  # reads the directory
            table = next(c for c in ec.categories if c.category_id == R.CATEGORY_POSITION_TABLE)
            self.assertEqual(table.member_count * table.member_size, 20)
            rec.clear()
            ec.read_position_table(3)  # reads still work, at the directory's stride
            self.assertEqual(struct.unpack(">H", rec.tx[0][2:4])[0], 0x1000 + 20 * 3)
            entry = PositionEntry(position_mm=10.0, speed_mm_s=50.0, accel_g=0.3, decel_g=0.3)
            rec.clear()
            with (self.assertLogs("iai_actuator_driver", "WARNING") as logs,
                  self.assertRaisesRegex(VerificationError, "read-back")):
                ec.write_position_table(3, entry)  # written anyway; the simulator keeps 16-word rows
            self.assertIn("field order is UNVERIFIED", "\n".join(logs.output))
            write = next(f for f in rec.tx if f[1] == 0x10)
            self.assertEqual(struct.unpack(">H", write[2:4])[0], 0x1000 + 20 * 3)
            with self.assertRaises(ValueError):
                ec.read_position_table(table.list_count)  # beyond LQ


class RowCountTest(unittest.TestCase):
    def test_a_non_ec_unit_does_not_get_the_ec_floor(self) -> None:
        ec, _, _ = open_axis("unknown")
        with ec:
            rows = ec.limits()["stored_rows"]["count"]  # before identify()
            self.assertEqual(rows, ec.identify().max_positions)
            self.assertEqual(rows, 2)


class CheckStepTest(unittest.TestCase):
    def test_motion_step_adapts_to_a_short_stroke(self) -> None:
        ec, _, _ = open_axis("unknown", stroke_mm=4, position_mm=2)
        with ec:
            report = run_check(ec, motion=True, port_label="sim")
        items = {i.name: i for i in report.items}
        self.assertEqual(items["motion_out"].result, PASS, items.get("motion"))
        self.assertEqual(items["motion_out"].data["target_mm"], 3.0)  # a quarter of the 4 mm stroke

    def test_messages_use_deg_on_a_rotary_axis(self) -> None:
        ec, _, _ = open_axis("rotary")
        with ec:
            report = run_check(ec, motion=True, port_label="sim")
        items = {i.name: i for i in report.items}
        self.assertIn(" deg", items["soft_limits"].detail)
        self.assertNotIn(" mm", items["position_table"].detail)
        self.assertIn("deg/s", items["motion_out"].detail)


class RotaryTextTest(unittest.TestCase):
    def test_reports_and_errors_say_deg(self) -> None:
        ec, _, _ = open_axis("rotary")
        with ec:
            ec.identify()
            with self.assertRaisesRegex(SoftLimitError, r"deg outside the effective stroke 0\.00\.\.360\.00 deg"):
                ec.jog(-5.0)
        code, out, _ = run("--sim", "--sim-profile", "rotary", "info")
        self.assertEqual(code, 0)
        soft = next(line for line in out.splitlines() if "soft limits" in line)
        self.assertIn("deg", soft)
        self.assertNotIn(" mm", soft)
        code, out, _ = run("--sim", "--sim-profile", "rotary", "param", "list")
        self.assertEqual(code, 0)
        p1 = next(line for line in out.splitlines() if line.strip().startswith("P1 "))
        self.assertIn(" deg", p1)
        self.assertNotIn(" mm", p1)


if __name__ == "__main__":
    unittest.main()
