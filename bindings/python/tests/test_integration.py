"""Multi-axis cell + universal axis model together, and their CLI commands (simulator only).

A mixed line (EC slider, PCON + RCP6, rotary EC) and mixed ports must work with the same commands: each axis
reports its own family, units and ranges; moves with their own deceleration run on a shared line; the CLI
offers limits / monitor / move --decel / goto --speed-pct / CON parameter numbers.
"""

from __future__ import annotations

import contextlib
import io
import json
import unittest
from unittest import mock

from iai_actuator_driver import cli
from iai_actuator_driver.cell import Cell
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine

OPEN = {"poll_interval_s": 0.005, "scan_timeout": 0.01}


def mixed_line(time_scale: float = 20.0) -> SimLine:
    return SimLine(
        [
            SimulatedEC(slave=1, profile="slider", time_scale=time_scale),
            SimulatedEC(slave=2, profile="pcon", time_scale=time_scale),
            SimulatedEC(slave=3, profile="rotary", time_scale=time_scale),
        ],
        name="MIX",
    )


class MixedCellTest(unittest.TestCase):
    def test_scan_names_every_family(self) -> None:
        with Cell.open([mixed_line()], **OPEN) as cell:
            self.assertEqual(cell.names, ("MIX:1", "MIX:2", "MIX:3"))
            fams = {n: info.product_family for n, info in cell.info.items()}
            self.assertEqual(fams, {"MIX:1": "EC", "MIX:2": "PCON", "MIX:3": "EC"})
            self.assertEqual([cell.units_of(n) for n in cell.names], ["mm", "mm", "deg"])
            limits = cell.limits_all()
            self.assertEqual(limits["MIX:3"]["units"], "deg")
            self.assertEqual(limits["MIX:2"]["family"], "PCON")
            self.assertGreater(limits["MIX:2"]["stored_rows"]["max"], 2)  # the PCON uses more than 3 rows
            mon = cell.monitor_all()
            self.assertEqual(set(mon), set(cell.names))
            self.assertEqual(mon["MIX:3"]["units"], "deg")

    def test_mixed_moves_with_their_own_deceleration(self) -> None:
        with Cell.open([mixed_line()], **OPEN) as cell:
            done = cell.move_all(
                {"MIX:1": 60.0, "MIX:2": 100.0, "MIX:3": 45.0}, speed_mm_s=100, accel_g=0.3,
                decel_g={"MIX:1": 0.3, "MIX:2": 0.1, "MIX:3": 0.2},
            )  # fmt: skip
            assert done is not None
            self.assertAlmostEqual(done["MIX:1"].position_mm, 60.0, delta=0.1)
            self.assertAlmostEqual(done["MIX:2"].position_mm, 100.0, delta=0.1)
            self.assertAlmostEqual(done["MIX:3"].position_mm, 45.0, delta=0.1)
            line = cell.port_of("MIX:2").bus.transport
            self.assertGreaterEqual(line[2].profile_moves, 1)  # the PCON took the command-profile path
            self.assertGreaterEqual(line[3].profile_moves, 1)
            self.assertEqual(line[1].profile_moves, 0)  # decel == accel: the direct numeric move


def run_cli(*argv: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stderr(err):
        code = cli.main(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class CliNewCommandsTest(unittest.TestCase):
    def test_limits(self) -> None:
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "limits")
        self.assertEqual(code, 0, out)
        self.assertIn("family PCON", out)
        self.assertIn("stored_rows", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "rotary", "--json", "limits")
        data = json.loads(out)
        self.assertEqual((data["units"], data["position"]["unit"]), ("deg", "deg"))

    def test_monitor(self) -> None:
        code, out, _ = run_cli("--sim", "monitor", "--count", "2", "--interval", "0")
        self.assertEqual(code, 0, out)
        self.assertEqual(out.count("position 73.95 mm"), 2)
        code, out, _ = run_cli("--sim", "--json", "monitor")
        self.assertEqual(json.loads(out)["position"], 73.95)

    def test_identify_and_status_show_family_and_units(self) -> None:
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "identify")
        self.assertEqual(code, 0, out)
        self.assertIn("family     PCON", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "rotary", "identify")
        self.assertIn("units      deg", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "rotary", "status")
        self.assertRegex(out, r"position\s+[-\d.]+ deg")

    def test_move_with_decel_and_goto_with_speed_override(self) -> None:
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--sim-time-scale", "50", "move", "100",
                                 "--speed", "100", "--accel", "0.3", "--decel", "0.1")  # fmt: skip
        self.assertEqual(code, 0, err)
        self.assertIn("100.00 mm", out)
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--sim-time-scale", "50", "goto", "6", "--allow-fast")
        self.assertEqual(code, 0, err)  # a row beyond the EC's three
        self.assertIn("48.21 mm", out)
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--unit-max", "--sim-time-scale", "50", "goto", "1",
                                 "--speed-pct", "25")  # fmt: skip
        self.assertEqual(code, 0, err)
        code, _, err = run_cli("--sim", "--sim-profile", "pcon", "goto", "64")
        self.assertEqual(code, 2)

    def test_position_unit_0_001_is_refused_on_a_direct_line(self) -> None:
        # ON-WIRE on a PCON-CB: a stated 0.001 on a direct line read 94.99 as 9.499 (and targets would go x 10)
        for line in ("auto", "direct"):
            code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--line", line, "--position-unit", "0.001",
                                     "status")  # fmt: skip
            self.assertEqual(code, 2, out)
            self.assertIn("position unit 0.001 mm refused on a direct line", err)
        code, _, err = run_cli("--sim", "--sim-profile", "pcon", "--position-unit", "0.01", "status")
        self.assertEqual(code, 0, err)

    def test_goto_notes_a_rounded_speed_override(self) -> None:
        # the unit runs (code + 1) / 256 of the row's speed (ON-WIRE on the PCON-CB): 0.1 % runs as code 1 = 0.78 %
        code, _, err = run_cli("--sim", "--sim-profile", "pcon", "--unit-max", "--sim-time-scale", "50", "goto", "1",
                               "--speed-pct", "0.1", "--no-wait")  # fmt: skip
        self.assertEqual(code, 0, err)
        self.assertIn("note: --speed-pct 0.1: the row runs at 0.78 % of its stored speed (override code 1;", err)
        code, _, err = run_cli("--sim", "--sim-profile", "pcon", "--unit-max", "--sim-time-scale", "50", "goto", "1",
                               "--speed-pct", "25")  # fmt: skip
        self.assertEqual(code, 0, err)
        self.assertNotIn("note: --speed-pct", err)  # 25 % runs as code 63 = 25.00 %

    def test_con_parameters(self) -> None:
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "param", "list")
        self.assertEqual(code, 0, out)
        self.assertIn("PCON parameters", out)
        self.assertIn("No.", out)
        self.assertIn("DANGEROUS", out)
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "param", "get", "8.15")
        self.assertEqual(code, 0, out)
        self.assertIn('"item"', out)
        code, out, _ = run_cli("--sim", "--sim-profile", "pcon", "--json", "param", "list")
        params = json.loads(out)
        safe = next(p for p in params if p["writable"] and not p["dangerous"] and p["min"] is not None
                    and p["max"] is not None and p["max"] > p["min"])  # fmt: skip
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "param", "set", f"No.{safe['number']}",
                                 str(safe["max"]))  # fmt: skip
        self.assertEqual(code, 0, err)
        self.assertIn(f"No.{safe['number']}", out)
        self.assertIn("committed", out)
        dangerous = next(p for p in params if p["writable"] and p["dangerous"])
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "param", "set", str(dangerous["number"]),
                                 str(dangerous["value"]), "--force")  # fmt: skip
        self.assertEqual(code, 0, err)  # no option needed: written with a warning
        self.assertIn("is a dangerous parameter", out)

    def test_swreset_reports_confirmation(self) -> None:
        code, out, err = run_cli("--sim", "--sim-profile", "pcon", "--sim-time-scale", "50", "swreset")
        self.assertEqual(code, 0, err)
        self.assertIn("restart confirmed by", out)

    def test_scan_names_the_pcon(self) -> None:
        code, out, _ = run_cli("scan", "--sim", "--sim-profile", "pcon", "--sim-slaves", "1-2")
        self.assertEqual(code, 0, out)
        self.assertIn("PCON", out)
        self.assertNotIn("not named by the driver", out)

    def test_old_model_opt_in_is_a_no_op(self) -> None:
        code, out, err = run_cli("--sim", "param", "set", "5", "1", "--allow-unverified-model")
        self.assertEqual(code, 0, err)
        self.assertIn("committed to non-volatile memory", out)
        self.assertNotIn("UNVERIFIED", out)

    def test_a_unit_without_a_family_name_is_reported_without_a_label(self) -> None:
        # system 0x99 / unit 0x7E: shown as the unit reports itself, with no note about its model, and driven the same
        # way (the move works)
        for args in (("--sim", "--sim-profile", "unknown", "identify"),
                     ("--sim", "--sim-profile", "unknown", "info", "--section", "identity"),
                     ("scan", "--sim", "--sim-profile", "unknown", "--sim-slaves", "1"),
                     ("--sim", "--sim-profile", "unknown", "--sim-time-scale", "50", "move", "5", "--speed", "30")):
            with self.subTest(args=args):
                code, out, err = run_cli(*args)
                self.assertEqual(code, 0, err)
                if "move" not in args:
                    self.assertIn("unit 0x7E", out)
                for text in ("UNVERIFIED", "not in the driver's EC table", "not named by the driver", "untested"):
                    self.assertNotIn(text, out)


if __name__ == "__main__":
    unittest.main()
