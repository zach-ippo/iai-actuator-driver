"""Runs the hardware acceptance harness on the driver's simulator: no serial port is opened.

    python -m unittest discover -s tools/hw/tests -t .        (from the repository root)

The driver is taken from bindings/python of this checkout when it is there, else the installed package is used. The native core
is found the way the package finds it (IAI_ACTUATOR_DRIVER_NATIVE, or next to the package); the package needs it.
"""

from __future__ import annotations

import contextlib
import importlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.hw import cli_edition, common, main as harness, pio_watch, python_edition, session  # noqa: E402
from tools.hw import power_cycle, report  # noqa: E402

SRC = ROOT / "bindings" / "python"
SRC_ARGS = ["--src", str(SRC)] if (SRC / "iai_actuator_driver").is_dir() else []
SAFE_STEPS = "status,limits,tables,move_to,move_by,jog,refusals,nv_guard,pc_pio,check"


def run(*argv: str, results: str) -> tuple[int, str, dict]:
    """The harness in this process; returns (exit code, console text, summary.json of the run or {})."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        code = harness.main([*argv, *SRC_ARGS, "--results", results])
    folders = sorted(Path(results).glob("*"))
    summary = {}
    if folders and (folders[-1] / "summary.json").is_file():
        summary = json.loads((folders[-1] / "summary.json").read_text(encoding="utf-8"))
    return code, out.getvalue(), summary


def verdicts(summary: dict) -> dict[str, str]:
    return {row["key"]: row["verdict"] for row in summary.get("steps", [])}


class ArgumentsTest(unittest.TestCase):
    def test_stroke_band(self) -> None:
        self.assertEqual(session.stroke_band("60"), ("pct", 60.0, 60.0))
        self.assertEqual(session.stroke_band("2.0:7.5"), ("mm", 2.0, 7.5))
        for bad in ("5", "95", "7:2", "abc", "nan:3"):
            with self.assertRaises(Exception, msg=bad):
                session.stroke_band(bad)

    def test_band_and_geometry(self) -> None:
        self.assertEqual(common.band_from_stroke(("pct", 60.0, 60.0), (0.0, 10.0)), (2.0, 8.0))
        self.assertEqual(common.band_from_stroke(("mm", 2.0, 7.5), (0.0, 5.0)), (2.0, 5.0))
        self.assertIsNone(common.band_from_stroke(("pct", 60.0, 60.0), None))
        g = common.Geometry(2.0, 8.0, 20.0)
        self.assertTrue(g.slow_usable)
        self.assertTrue(g.lo < g.a < g.c < g.b < g.hi)
        self.assertEqual((g.v, g.acc), (10.0, 0.1))
        self.assertFalse(common.Geometry(2.0, 3.0, 20.0).usable)
        self.assertEqual(common.Geometry(2.0, 8.0, 4.0).v, 4.0)  # never above the speed cap

    def test_steps(self) -> None:
        keys, default = ["a", "b", "c"], ["a", "b"]
        self.assertEqual(harness.resolve_steps("default", "", keys, default), ["a", "b"])
        self.assertEqual(harness.resolve_steps("default,c", "a", keys, default), ["b", "c"])
        self.assertEqual(harness.resolve_steps("all", "", keys, default), keys)
        with self.assertRaises(session.SetupError):
            harness.resolve_steps("x", "", keys, default)

    def test_backup_comparison(self) -> None:
        bk = {"parameters": {"classes": [{"class_id": 3, "items": [{"id": 1, "entry": 0, "hex": "00"}]},
                                         {"class_id": 8, "items": [{"id": 1, "entry": 0, "hex": "0A"}]}]},
              "positions": {"rows": [{"n": 0, "words": ["0001"]}]}, "alarms": {"words": []}}
        other = json.loads(json.dumps(bk))
        other["parameters"]["classes"][0]["items"][0]["hex"] = "FF"  # the live class is left out
        self.assertEqual(common.compare_backups(bk, other)["parameters_changed"], {})
        self.assertEqual(common.compare_backups(bk, other)["parameter_items"], 1)  # the live item is not counted as compared
        self.assertEqual(common.compare_backups(bk, other)["live_items_left_out"], 1)
        other["parameters"]["classes"][1]["items"][0]["hex"] = "0B"
        other["positions"]["rows"][0]["words"] = ["0002"]
        result = common.compare_backups(bk, other)
        self.assertEqual(len(result["parameters_changed"]), 1)
        self.assertEqual(result["rows_changed"], [0])

    def test_backup_comparison_leaves_out_both_live_classes(self) -> None:
        # class 4 holds live values too (a real unit changed six of its items between two backups with no write): only a
        # change in a parameter class (7, 8, 9, 0x0C) counts
        bk = {"parameters": {"classes": [{"class_id": 3, "items": [{"id": 1, "entry": 0, "hex": "00"}]},
                                         {"class_id": 4, "items": [{"id": 9, "entry": 0, "hex": "FFFF4925"}, {"id": 2, "entry": 0, "hex": "0"}]},
                                         {"class": "0x0004", "items": [{"id": 5, "entry": 0, "hex": "1"}]},
                                         {"class_id": 9, "items": [{"id": 16, "entry": 0, "hex": "07"}]}]},
              "positions": {"rows": []}, "alarms": {"words": []}}
        other = json.loads(json.dumps(bk))
        other["parameters"]["classes"][1]["items"][0]["hex"] = "FFFF4924"
        other["parameters"]["classes"][1]["items"][1]["hex"] = "1"
        other["parameters"]["classes"][2]["items"][0]["hex"] = "2"
        result = common.compare_backups(bk, other)
        self.assertEqual(result["parameters_changed"], {})
        self.assertEqual(result["live_items_changed"], 3)
        self.assertEqual((result["parameter_items"], result["live_items_left_out"]), (1, 4))
        other["parameters"]["classes"][3]["items"][0]["hex"] = "08"
        self.assertEqual(len(common.compare_backups(bk, other)["parameters_changed"]), 1)


class AlarmAtEndTest(unittest.TestCase):
    def test_a_warning_present_at_the_start_that_recover_reset_is_a_check_not_a_failure(self) -> None:
        ok, note = common.alarm_at_end(0x069, 0, {0x069}, True, "Real Time Clock oscillation stop detection")
        self.assertTrue(ok)
        self.assertIn("alarm 0x069 (Real Time Clock oscillation stop detection)", note)
        self.assertIn("reset by recover()", note)

    def test_anything_else_is_not_as_found(self) -> None:
        self.assertEqual(common.alarm_at_end(0x069, 0x069, set(), True), (True, None))  # as found
        self.assertEqual(common.alarm_at_end(0, 0, set(), False), (True, None))
        self.assertEqual(common.alarm_at_end(0, 0x069, set(), True), (False, None))  # a new alarm
        self.assertEqual(common.alarm_at_end(0x069, 0x06A, {0x069}, True), (False, None))  # another code at the end
        self.assertEqual(common.alarm_at_end(0x0A1, 0, {0x0A1}, False), (False, None))  # a fault-level alarm
        self.assertEqual(common.alarm_at_end(0x04E, 0, {0x04E}, True), (False, None))  # only 0x069 may be gone
        self.assertEqual(common.alarm_at_end(0x069, 0, set(), False), (False, None))  # 0x069 not at message level

    def test_the_power_up_alarm_cleared_another_way_is_as_found(self) -> None:
        ok, note = common.alarm_at_end(0x069, 0, set(), True)  # e.g. an alarm reset edge of the driver during the run
        self.assertTrue(ok)
        self.assertIn("alarm 0x069, the unit's normal message-level state at the start, was cleared during the run",
                      note)

    def test_only_0x069_is_a_normal_start_alarm(self) -> None:
        self.assertIsNone(common.start_alarm_problem(False, 0))
        self.assertIsNone(common.start_alarm_problem(False, 0x069))
        self.assertIn("0x04E, a message-level alarm other than 0x069", common.start_alarm_problem(False, 0x04E))
        self.assertEqual(common.start_alarm_problem(True, 0x069), "alarm 0x069")  # a fault: refused whatever the code
        self.assertEqual(common.start_alarm_problem(True, 0x0A1), "alarm 0x0A1")


class StartAlarmTest(unittest.TestCase):
    """pio-watch, power-cycle and the command-line edition apply the same start rule: 0x069 at message level goes on,
    another message-level alarm (0x04E) is refused before anything is sent."""

    @staticmethod
    def status(code: int) -> SimpleNamespace:
        return SimpleNamespace(position_mm=5.0, servo_on=True, homed=True, in_position=True, moving=False, alarm=False,
                               alarm_code=code, modbus_enabled=True, safety_speed=False)

    def check(self, start: Any, refused_text: str) -> None:
        self.assertEqual(start(0x069)[0], report.PASS)
        with self.assertRaises(report.Abort) as refused:
            start(0x04E)
        self.assertIn("alarm 0x04E, a message-level alarm other than 0x069", str(refused.exception))
        self.assertIn(refused_text, str(refused.exception))

    def test_pio_watch(self) -> None:
        def start(code: int) -> Any:
            watch = pio_watch.PioWatch.__new__(pio_watch.PioWatch)
            watch.raw = mock.Mock(baudrate=38400, status=lambda: self.status(code))
            watch.remember = lambda where, st: None  # type: ignore[method-assign]
            return watch._state()

        self.check(start, "the harness never clears an alarm")

    def test_power_cycle(self) -> None:
        def start(code: int) -> Any:
            cycle = power_cycle.PowerCycle.__new__(power_cycle.PowerCycle)
            cycle.args = SimpleNamespace(stroke_band=("pct", 60, 60), speed_cap=10.0)
            cycle.raw = mock.Mock(baudrate=session.POWER_UP_BAUD, status=lambda: self.status(code),
                                  ec_stroke=(0.0, 10.0), effective_stroke=(2.0, 8.0), speed_cap_mm_s=10.0,
                                  accel_cap_g=0.1)
            cycle.remember = lambda where, st: None  # type: ignore[method-assign]
            return cycle._preflight()

        self.check(start, "the start state is not what this test needs")

    def test_the_command_line_edition(self) -> None:
        limits = json.dumps({"position": {"min": 0.0, "max": 10.0}, "speed": {"max": 10.0}, "accel": {"max": 0.1}})

        def start(code: int) -> Any:
            edition = cli_edition.CliEdition.__new__(cli_edition.CliEdition)
            edition.args = SimpleNamespace(stroke_band=("pct", 60, 60), speed_cap=10.0)
            edition.selected, edition.caps, edition.geo, edition.motion_ok = ["status"], [], None, False
            edition.status = lambda: {**vars(self.status(code)), "raw": {"DIPM": 0}}  # type: ignore[method-assign]
            edition.cli = lambda *words, **kw: cli_edition.Result(  # type: ignore[method-assign]
                0, limits, "", 0.1, list(words))
            return edition._preflight()

        self.check(start, "not touching the unit")


class SafetyRulesTest(unittest.TestCase):
    def test_a_real_port_without_a_terminal_or_yes_is_refused_before_it_is_opened(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(session, "has_terminal", return_value=False), \
                mock.patch.object(python_edition, "open_axis", side_effect=AssertionError("the port must not be opened")):
            code, _, _ = run("--port", "COM_NOT_THERE", results=tmp)
            self.assertEqual(code, 2)
            self.assertEqual(list(Path(tmp).glob("*")), [])  # not even a results folder

    def test_bad_arguments_exit_2(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            for argv in (["--sim", "--steps", "nonsense"], ["--sim", "--port", "COM3"], ["--sim", "--slave", "0"]):
                self.assertEqual(run(*argv, results=tmp)[0], 2, argv)
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):  # argparse: a speed cap above the limit
                run("--sim", "--speed-cap", "500", results=tmp)

    def test_the_guard_refuses_writes_and_the_same_value_write_is_skipped(self) -> None:
        iad = session.load_driver(str(SRC) if SRC_ARGS else None, None, "auto")
        ctx = mock.Mock(iad=iad, args=mock.Mock(sim=True, sim_profile="gripper", sim_baud=38400, sim_time_scale=1.0, slave=1))
        ctx.sim_options = {}
        raw, _ = session.open_axis(ctx, {"max_speed_mm_s": 20.0, "max_accel_g": 0.1})
        try:
            ax = session.GuardedAxis(raw)
            raw.connect(take_control=False)
            for name in sorted(session.FORBIDDEN_CALLS):
                with self.assertRaises(session.SafetyError, msg=name):
                    getattr(ax, name)
            number = next(p["number"] for p in ax.parameters() if p.get("writable") and not p.get("dangerous"))
            _, result, used = ax.same_value_write(number)
            self.assertTrue(result.skipped)
            self.assertEqual((used, ax.nonvolatile_writes), (0, 0))
        finally:
            raw.close()


class LoadDriverTest(unittest.TestCase):
    def test_a_stray_folder_that_holds_only_pycache_is_not_taken_for_the_driver(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "iai_actuator_driver" / "__pycache__").mkdir(parents=True)  # imports as a namespace package
            others = {k: v for k, v in sys.modules.items() if k.split(".")[0] != "iai_actuator_driver"}  # not imported yet
            elsewhere = [p for p in sys.path if not (Path(p or ".") / "iai_actuator_driver" / "__init__.py").is_file()]  # no driver
            with mock.patch.dict(sys.modules, others, clear=True), mock.patch.object(sys, "path", [tmp, *elsewhere]):
                importlib.invalidate_caches()
                with self.assertRaises(session.SetupError) as raised:
                    session.load_driver(None, None, "auto")
        text = str(raised.exception)
        self.assertTrue(text.startswith("cannot import iai_actuator_driver: "), text)
        self.assertIn("no __init__.py", text)
        self.assertIn("Install it", text)


class DocumentationTest(unittest.TestCase):
    def test_the_readme_names_every_step(self) -> None:
        text = (ROOT / "tools" / "hw" / "README.md").read_text(encoding="utf-8")
        for key in python_edition.STEP_KEYS:
            self.assertIn(f"`{key}`", text, key)
        for key in ("status", "limits", "info", "home", "move", "jog", "push", "stop", "monitor", "check", "recover"):
            self.assertIn(f"`{key}", text, key)
        self.assertTrue(set(cli_edition.STEP_KEYS) & set(python_edition.STEP_KEYS))


class PythonEditionTest(unittest.TestCase):
    def test_the_safe_set_on_the_simulator_passes_and_leaves_the_unit_as_found(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--steps", SAFE_STEPS, results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            self.assertEqual(summary["verdict"], "PASS", text[-3000:])
            v = verdicts(summary)
            self.assertEqual(v["backup"], "PASS")  # the read-only backup comes first
            self.assertEqual(v["end"], "PASS")
            self.assertEqual(v["nv_guard"], "PASS")
            self.assertEqual([k for k, x in v.items() if x == "FAIL"], [])
            keys = [r["key"] for r in summary["steps"]]
            self.assertLess(keys.index("backup"), keys.index("move_to"))
            self.assertEqual(summary["end_state"]["nonvolatile_writes"], 0)
            self.assertEqual(summary["backup_compare"]["parameters_changed"], {})
            self.assertEqual(summary["backup_compare"]["rows_changed"], [])
            folder = next(Path(tmp).glob("*"))
            for name in ("summary.md", "summary.json", "run.log", "backup_start.json", "backup_end.json", "unit_info_start.json", "start_state.json"):
                self.assertTrue((folder / name).is_file(), name)
            self.assertIn("PASS", (folder / "summary.md").read_text(encoding="utf-8"))

    def test_a_unit_with_an_alarm_is_not_touched(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "alarm", "--steps", "status,move_to", results=tmp)
            self.assertEqual(code, 2, text[-2000:])
            v = verdicts(summary)
            self.assertEqual(v["backup"], "PASS")  # the backup is made, nothing else is
            self.assertNotIn("move_to", v)
            self.assertNotIn("prepare", v)

    def test_a_failed_step_stops_the_rest_and_the_unit_is_put_back(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "fault", "--steps", "move_to,move_by,jog", results=tmp)
            self.assertEqual(code, 1, text[-2000:])
            v = verdicts(summary)
            self.assertEqual((v["move_by"], v["jog"], v["end"]), ("FAIL", "SKIP", "PASS"))
            self.assertAlmostEqual(summary["end_state"]["status"]["position_mm"], summary["start_state"]["position_mm"], delta=0.1)

    def test_ctrl_c_in_the_middle_puts_the_unit_back(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "interrupt", "--steps", "move_to,pause,stop,speed", results=tmp)
            self.assertEqual(code, 130, text[-2000:])
            v = verdicts(summary)
            self.assertIn("INTR", v.values())
            self.assertEqual(v["end"], "PASS")

    def test_start_outside_the_band_skips_the_moves_and_leaves_the_unit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "outside", "--steps", "move_to,status", results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["move_to"], v["preflight"], v["end"]), (0, "SKIP", "CHECK", "PASS"), text[-2000:])
            self.assertAlmostEqual(summary["end_state"]["status"]["position_mm"], 9.0, delta=0.05)

    def test_the_unit_is_not_handed_to_pio_while_an_input_is_on(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "pio-input", "--steps", "pc_pio", results=tmp)
            self.assertEqual(verdicts(summary)["pc_pio"], "CHECK", text[-2000:])
            self.assertEqual(verdicts(summary)["end"], "PASS")

    def test_a_warning_that_recover_resets_makes_the_end_a_check_naming_it(self) -> None:
        # the unit starts with the message-level alarm 0x069 (as a PCON whose clock is not set); recover() resets it, so the
        # end differs from the start in that one point: CHECK with the alarm named, not FAIL
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "warning", "--sim-baud", "115200", "--steps", "link,pc_pio,recover",
                                      results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["recover"], v["end"], summary["verdict"]), (0, "PASS", "CHECK", "CHECK"), text[-3000:])
            end = next(r for r in summary["steps"] if r["key"] == "end")
            self.assertIn("alarm 0x069 (Real Time Clock oscillation stop detection)", end["detail"])
            self.assertIn("reset by recover()", end["detail"])
            self.assertEqual(summary["start_state"]["alarm_code"], 0x069)
            self.assertEqual(summary["end_state"]["alarm_reset_by_recover"], 0x069)

    def test_a_start_alarm_0x069_cleared_during_the_run_is_the_units_normal_state(self) -> None:
        # after a power-up the unit shows 0x069; an alarm reset in the run (not recover()) clears it: as found, CHECK
        status = python_edition.PythonEdition.s_status

        def status_then_cleared(edition: Any) -> Any:
            result = status(edition)
            edition.raw.reset_alarm()  # the unit's alarm is cleared in the middle of the run (test only)
            return result

        patch = mock.patch.object(python_edition.PythonEdition, "s_status", status_then_cleared)
        with tempfile.TemporaryDirectory() as tmp, patch:
            code, text, summary = run("--sim", "--sim-scenario", "warning", "--steps", "status,move_to", results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["preflight"], v["status"], v["move_to"], v["end"]),
                             (0, "PASS", "PASS", "PASS", "CHECK"), text[-3000:])
            rows = {r["key"]: r["detail"] for r in summary["steps"]}
            self.assertIn("alarm 0x069 at message level: the unit's normal state after a power-up", rows["preflight"])
            self.assertIn("was cleared during the run", rows["end"])
            self.assertNotIn("NOT AS FOUND", rows["end"])
            self.assertEqual(summary["start_state"]["alarm_code"], 0x069)
            self.assertEqual(summary["end_state"]["start_alarm_cleared"], 0x069)
            self.assertNotIn("alarm_reset_by_recover", summary["end_state"])
            self.assertEqual(summary["end_state"]["nonvolatile_writes"], 0)

    def test_another_message_level_start_alarm_is_refused(self) -> None:
        # strict: only 0x069 is a normal start state; the maintenance warning 0x04E keeps the unit untouched
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "other-warning", "--steps", "status,move_to",
                                      results=tmp)
            self.assertEqual(code, 2, text[-2000:])
            v = verdicts(summary)
            self.assertEqual(v["backup"], "PASS")
            self.assertNotIn("move_to", v)
            self.assertNotIn("prepare", v)
            self.assertIn("alarm 0x04E, a message-level alarm other than 0x069", text)

    def test_a_warning_left_alone_is_as_found(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-scenario", "warning", "--steps", "status,move_to", results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["end"]), (0, "PASS"), text[-3000:])

    def test_pc_pio_on_a_pcon_that_pauses_and_switches_its_servo_off_in_pio_mode(self) -> None:
        # nothing on the PCON's PIO connector: PIO mode shows a pause (STP) at rest with the servo off; PC control is taken
        # back all the same and the servo comes on again before the PC-control moves
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-profile", "pcon", "--steps", "pc_pio", results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["pc_pio"], v["end"]), (0, "PASS", "PASS"), text[-3000:])
            self.assertIn("the servo went off in PIO mode and is on again", text)
            self.assertNotIn("still moving", text)

    def test_servo_across_the_toggle(self) -> None:
        # (call, servo on, paused) after each call, the switch to PIO mode first
        pcon = [("pio", False, True), ("pc", False, False), ("off", False, True), ("on", False, False)]
        self.assertEqual(common.servo_across_toggle(True, pcon), ("pio", True))  # the unwired *STP pattern: off with a pause
        self.assertEqual(common.servo_across_toggle(False, pcon), (None, False))  # it was off at the start: nothing lost
        self.assertEqual(common.servo_across_toggle(True, [("pio", True, False), ("pc", True, False)]), (None, False))
        self.assertEqual(common.servo_across_toggle(True, [("pio", True, False), ("pc", False, False)]), ("pc", False))
        self.assertEqual(common.servo_across_toggle(True, [("pio", False, False), ("pc", False, False)]), ("pio", False))  # off, no pause
        # the pause in PIO mode, the servo off only by the next status (a command line's reply on the PCON-CB, 2026-11-02)
        self.assertEqual(common.servo_across_toggle(True, [("pio", True, True), ("pc", False, False)]), ("pc", True))
        self.assertEqual(common.servo_across_toggle(True, [("pio", True, False), ("pc", False, True)]), ("pc", False))  # no pause in PIO
        self.assertEqual(common.servo_across_toggle(True, [("pio", None, None), ("pc", True, False)]), (None, False))  # unknown
        # the pause in PIO mode, the servo still on in the first PC-mode status: a later loss is not that pattern
        later = [("pio", True, True), ("pc", True, False), ("off", False, True), ("on", False, False)]
        self.assertEqual(common.servo_across_toggle(True, later), ("off", False))
        later[2] = ("off", True, True)
        self.assertEqual(common.servo_across_toggle(True, later), ("on", False))

    def _python_pc_pio(self, script: dict[str, tuple[bool, bool]], *, servo_on: bool = True,
                       locks_servo: bool = False) -> tuple[str, str, list[str]]:
        """The pc_pio step on a scripted axis: ``script`` gives (servo on, paused) after each call; PMSS follows the calls.
        ``locks_servo``: set_action_mode(PC) leaves servo ON refused until disable_modbus_control (1.0.1 before its fix on
        the PCON-CB)."""
        state = SimpleNamespace(raw={"DIPM": 0}, servo_on=servo_on, paused=False, modbus_enabled=True, safety_speed=False,
                                position_mm=5.0)
        calls: list[str] = []
        locked = [False]

        class Axis:
            def status(self) -> Any:
                return SimpleNamespace(**vars(state))

            def _do(self, call: str, pmss: bool) -> None:
                calls.append(call)
                state.modbus_enabled = pmss
                state.servo_on, state.paused = script[call]
                if call == "set_action_mode(PC)":
                    locked[0] = locks_servo
                elif call == "disable_modbus_control":
                    locked[0] = False

            def set_action_mode(self, pc: bool) -> None:
                self._do("set_action_mode(PC)" if pc else "set_action_mode(PIO)", pc)

            def disable_modbus_control(self) -> None:
                self._do("disable_modbus_control", False)

            def enable_modbus_control(self) -> None:
                self._do("enable_modbus_control", True)

            def servo(self, on: bool) -> None:
                calls.append(f"servo({on})")
                if on and locked[0]:
                    raise RuntimeError("servo did not turn on within 3 s (the status shows no known cause)")
                state.servo_on = on

        edition = python_edition.PythonEdition.__new__(python_edition.PythonEdition)
        edition.ax = edition.raw = Axis()
        edition.geo, edition.start, edition.motion_ok, edition.pio_handed = None, {"safety_speed": False}, False, False
        edition.say = lambda *_: None
        edition._still = lambda seconds: 0.0
        verdict, text, _rec = edition.s_pc_pio()
        return verdict, text, calls

    def test_pc_pio_servo_off_with_a_pause_in_pio_mode_is_switched_on_again(self) -> None:
        script = {"set_action_mode(PIO)": (False, True), "set_action_mode(PC)": (False, False),
                  "disable_modbus_control": (False, True), "enable_modbus_control": (False, False)}
        verdict, text, calls = self._python_pc_pio(script)
        self.assertEqual(verdict, "PASS", text)
        self.assertIn("the servo went off in PIO mode and is on again", text)
        self.assertEqual(calls[-1], "servo(True)")

    def test_pc_pio_tries_servo_on_straight_after_set_action_mode_pc(self) -> None:
        # before any PMSL toggle: the toggle of the step put right what 1.0.1's action-mode PC broke on the PCON-CB
        script = {"set_action_mode(PIO)": (False, True), "set_action_mode(PC)": (False, False),
                  "disable_modbus_control": (False, True), "enable_modbus_control": (False, False)}
        for servo_on in (True, False):  # also for a unit found with the servo off
            with self.subTest(servo_on=servo_on):
                verdict, text, calls = self._python_pc_pio(script, servo_on=servo_on)
                self.assertEqual(verdict, "PASS", text)
                self.assertEqual(calls[calls.index("set_action_mode(PC)") + 1], "servo(True)")
                self.assertIn("servo ON right after it: taken", text)
                verdict, text, calls = self._python_pc_pio(script, servo_on=servo_on, locks_servo=True)
                self.assertEqual(verdict, "FAIL", text)
                self.assertIn("servo ON right after it NOT taken: RuntimeError: servo did not turn on", text)

    def test_pc_pio_a_servo_lost_in_another_way_fails_the_step(self) -> None:
        for lost_at in ("set_action_mode(PC)", "disable_modbus_control", "enable_modbus_control"):
            with self.subTest(lost_at):
                script = {"set_action_mode(PIO)": (True, False), "set_action_mode(PC)": (True, False),
                          "disable_modbus_control": (True, False), "enable_modbus_control": (True, False)}
                script[lost_at] = (False, False)
                verdict, text, calls = self._python_pc_pio(script)
                self.assertEqual(verdict, "FAIL", text)
                self.assertIn(f"the servo was off after {lost_at}", text)
                self.assertNotIn("servo(True)", calls)  # not repaired behind the verdict
        # a pause in PIO mode, but the servo still on in the first PC-mode status: a later loss fails the step too
        for lost_at in ("disable_modbus_control", "enable_modbus_control"):
            with self.subTest(lost_at, paused_in_pio=True):
                script = {"set_action_mode(PIO)": (True, True), "set_action_mode(PC)": (True, False),
                          "disable_modbus_control": (True, True), "enable_modbus_control": (True, False)}
                script[lost_at] = (False, script[lost_at][1])
                verdict, text, calls = self._python_pc_pio(script)
                self.assertEqual(verdict, "FAIL", text)
                self.assertIn(f"the servo was off after {lost_at}", text)
                self.assertNotIn("servo(True)", calls)
        # off in PIO mode but no pause shown: not the pattern of an unwired stop input
        script = {"set_action_mode(PIO)": (False, False), "set_action_mode(PC)": (False, False),
                  "disable_modbus_control": (False, False), "enable_modbus_control": (False, False)}
        verdict, text, calls = self._python_pc_pio(script)
        self.assertEqual(verdict, "FAIL", text)
        self.assertNotIn("servo(True)", calls)

    def test_list_steps(self) -> None:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(harness.main(["--list-steps"]), 0)
        self.assertIn("recover", out.getvalue())


class CommandLineEditionTest(unittest.TestCase):
    def test_only_commands_that_read_move_or_switch_are_run(self) -> None:
        args = mock.Mock(sim=True, sim_profile="gripper", sim_baud=38400, sim_time_scale=1.0, slave=1, native=None, port=None)
        edition = cli_edition.CliEdition(mock.Mock(args=args), [], "cli")
        edition.program = ["no-such-program"]
        for words in (("param", "set", "P2", "1"), ("teach", "1"), ("swreset",), ("baud", "115200"), ("alarm-reset",), ("param",)):
            with self.assertRaises(session.SetupError, msg=words):
                edition.cli(*words)
        self.assertEqual(edition.log, [])  # nothing was run

    def test_json_must_be_the_whole_of_stdout(self) -> None:
        NL = chr(10)

        def result(text: str) -> cli_edition.Result:
            return cli_edition.Result(0, text, "", 0.1, ["mode", "pc"])

        self.assertEqual(result('{"a": 1}' + NL).json(), {"a": 1})
        self.assertEqual(result('{' + NL + '  "modbus_enabled": true' + NL + '}' + NL).json(), {"modbus_enabled": True})
        # a note line on stdout before the JSON of `mode pc` (the C# command line printed one on a real unit) is a defect of the
        # command line: the harness does not skip it, so the step that needs the JSON fails and the defect shows
        note = "enabling Modbus control (PMSL) while the axis is at rest"
        polluted = note + NL + '{' + NL + '  "modbus_enabled": true' + NL + '}' + NL
        self.assertIsNone(result(polluted).json())
        edition = cli_edition.CliEdition.__new__(cli_edition.CliEdition)
        with self.assertRaises(cli_edition.CommandFailed) as caught:
            edition.ok(result(polluted))
        self.assertIn("not one JSON document", str(caught.exception))
        self.assertIsNone(result("no json here" + NL).json())
        self.assertIsNone(result('{"t": 1}' + NL + '{"t": 2}' + NL).json())  # one JSON document per line is not one document

    def _cli_pc_pio(self, pio: dict[str, Any], pc: dict[str, Any],
                    after: dict[str, Any] | None = None) -> tuple[str, str, list[tuple[str, ...]]]:
        """The pc_pio step with scripted answers of `mode pio` and `mode pc`; the status read after them is what `mode pc` showed,
        changed by ``after``."""
        state: dict[str, Any] = {"raw": {"DIPM": 0}, "servo_on": True, "paused": False, "modbus_enabled": True}
        commands: list[tuple[str, ...]] = []
        replies = {("mode", "pio"): pio, ("mode", "pc"): pc, ("servo", "on"): {"servo_on": True}}

        def cli(*words: str, **_kw: Any) -> tuple[str, ...]:
            commands.append(words)
            if words[0] != "status":
                state.update(replies[words])
            return words

        edition = cli_edition.CliEdition.__new__(cli_edition.CliEdition)
        edition.start, edition.motion_ok, edition.persist, edition.pio_handed = {"dipm": 0}, False, False, False
        edition.cli = cli  # type: ignore[method-assign]
        edition.ok = lambda words: dict(replies[words])  # type: ignore[method-assign]
        edition.status = lambda: dict(state, **(after if ("mode", "pc") in commands and after else {}))  # type: ignore[method-assign]
        verdict, text = edition.s_pc_pio()
        return verdict, text, commands

    def test_pc_pio_servo_off_with_a_pause_in_pio_mode_is_switched_on_again(self) -> None:
        verdict, text, commands = self._cli_pc_pio({"modbus_enabled": False, "servo_on": False, "paused": True},
                                                   {"modbus_enabled": True, "servo_on": False, "paused": False})
        self.assertEqual(verdict, "PASS", text)
        self.assertIn("the servo went off in PIO mode (with a pause) and is on again", text)
        self.assertEqual(commands[-1], ("servo", "on"))

    def test_pc_pio_servo_still_on_in_the_pio_reply_with_the_pause_is_switched_on_again(self) -> None:
        # the PCON-CB through a command line, 2026-11-02: the `mode pio` reply had the pause with the servo still on
        # (DSS1 0x3239), and the servo was off by the `mode pc` reply
        verdict, text, commands = self._cli_pc_pio({"modbus_enabled": False, "servo_on": True, "paused": True},
                                                   {"modbus_enabled": True, "servo_on": False, "paused": False})
        self.assertEqual(verdict, "PASS", text)
        self.assertIn("is on again", text)
        self.assertEqual(commands[-1], ("servo", "on"))

    def _cli_restore(self, start_servo: bool, position: float) -> tuple[list[tuple[str, ...]], dict[str, Any]]:
        """CliEdition._restore on a scripted unit at rest in PC mode, servo off, at ``position`` (start 10.00 mm)."""
        state: dict[str, Any] = {"moving": False, "modbus_enabled": True, "alarm": False, "alarm_code": 0, "servo_on": False,
                                 "position_mm": position}
        commands: list[tuple[str, ...]] = []

        def cli(*words: str, **_kw: Any) -> tuple[str, ...]:
            commands.append(words)
            if words[0] == "servo":
                state["servo_on"] = words[1] == "on"
            return words

        def go(mm: float) -> dict[str, Any]:
            commands.append(("move", f"{mm:.2f}"))
            if not state["servo_on"]:
                raise RuntimeError("servo is off")
            state["position_mm"] = mm
            return dict(state)

        edition = cli_edition.CliEdition.__new__(cli_edition.CliEdition)
        edition.start = {"servo_on": start_servo, "homed": True, "position_mm": 10.0}
        edition.notes, edition.persist, edition.changed, edition.pio_handed = [], True, True, False
        edition.motion_ok, edition.geo = True, object()
        edition.say = lambda *_: None  # type: ignore[method-assign]
        edition.cli = cli  # type: ignore[method-assign]
        edition.ok = lambda words: {}  # type: ignore[method-assign]
        edition.status = lambda: dict(state)  # type: ignore[method-assign]
        edition.go = go  # type: ignore[method-assign]
        edition._restore()
        return commands, state

    def test_restore_switches_the_servo_on_for_the_return_move_of_a_unit_found_with_it_off(self) -> None:
        commands, state = self._cli_restore(False, 75.0)
        self.assertEqual(commands, [("servo", "on"), ("move", "10.00"), ("servo", "off")])
        self.assertEqual((state["position_mm"], state["servo_on"]), (10.0, False))
        commands, state = self._cli_restore(False, 10.0)  # at the start position: the servo is left off, nothing sent
        self.assertEqual(commands, [])
        commands, state = self._cli_restore(True, 75.0)  # found on: on, back, and left on
        self.assertEqual(commands, [("servo", "on"), ("move", "10.00")])

    def test_pc_pio_a_servo_lost_in_another_way_fails_the_step(self) -> None:
        verdict, text, commands = self._cli_pc_pio({"modbus_enabled": False, "servo_on": True, "paused": False},
                                                   {"modbus_enabled": True, "servo_on": False, "paused": False})
        self.assertEqual(verdict, "FAIL", text)
        self.assertIn("the servo was off after mode pc", text)
        self.assertNotIn(("servo", "on"), commands)
        # the pause in PIO mode, the servo on in both replies and off only in the status read after them: not that pattern
        verdict, text, commands = self._cli_pc_pio({"modbus_enabled": False, "servo_on": True, "paused": True},
                                                   {"modbus_enabled": True, "servo_on": True, "paused": False}, {"servo_on": False})
        self.assertEqual(verdict, "FAIL", text)
        self.assertIn("the servo was off after the status after mode pc", text)
        self.assertNotIn(("servo", "on"), commands)

    def test_a_missing_program_is_one_skip_not_a_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            for edition in ("cli", "csharp"):
                code, text, summary = run("--sim", "--edition", edition, "--cli-exe", str(Path(tmp) / "no-such-program"), results=tmp)
                if code == 0 and verdicts(summary).get("open") != "SKIP":
                    self.skipTest("a command line was found next to the checkout")  # then the real thing was run, not this case
                self.assertEqual((code, verdicts(summary).get("open")), (0, "SKIP"), text[-1500:])


class InteractiveTest(unittest.TestCase):
    def test_power_cycle_on_the_simulator(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("power-cycle", "--sim", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            v = verdicts(summary)
            self.assertEqual((v["wait"], v["recover"], v["end"]), ("PASS", "PASS", "PASS"))

    def test_recover_step_after_the_pc_pio_toggle_must_keep_or_restore_pc_control(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("--sim", "--sim-baud", "115200", "--steps", "link,pc_pio,recover", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            v = verdicts(summary)
            self.assertEqual((v["pc_pio"], v["recover"], v["end"]), ("PASS", "PASS", "PASS"), text[-3000:])
            self.assertIn("PC control kept True", text)  # the toggle's enable_modbus_control told the session that PC control is wanted

    def test_power_cycle_that_never_loses_power_gives_up_and_says_where_the_unit_is(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, _ = run("power-cycle", "--sim", "--sim-scenario", "never", "--loss-timeout", "2", results=tmp)
            self.assertEqual(code, 3, text[-2000:])
            self.assertIn("WHERE THE UNIT IS LEFT", text)

    def test_power_cycle_with_a_fault_alarm_after_the_cycle_stops_before_any_move(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("power-cycle", "--sim", "--sim-scenario", "alarm", results=tmp)
            v = verdicts(summary)
            self.assertEqual((code, v["recover"]), (5, "FAIL"), text[-2000:])
            self.assertFalse({"servo", "home", "return"} & set(v), "no servo, home or return move after a bad recovery")

    def test_pio_watch_on_the_simulator_restores_pc_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--seconds", "4", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            self.assertEqual(verdicts(summary)["restore"], "PASS")
            self.assertTrue(summary["end_state"]["pc_mode_restored"])
            self.assertGreaterEqual(len(summary["watch"]["moves"]), 1)

    def test_pio_watch_stop_verdict_does_not_claim_a_cause_for_a_move_after_the_release(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--stop-on-move", "--seconds", "6", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            rec = summary["stop_on_move"]
            self.assertTrue(rec["accepted"], rec)
            self.assertTrue(rec["restarted_after_stop"], rec)  # the simulated PLC holds its start input through the STOP
            self.assertEqual(rec["dipm_at_move_start"], rec["dipm_after_release"])
            self.assertIn("not established", rec["verdict"])
            self.assertNotIn("still held", rec["verdict"])
            self.assertNotIn("restarts it", rec["verdict"])

    def test_pio_watch_stop_on_a_push_row_that_rests_at_its_approach_is_effective(self) -> None:
        # as on the gripper: the stopped ST1 move rested at the push row's approach (4.11 mm), MEND 0, ST1 still held
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--stop-on-move", "--sim-scenario", "push", "--seconds", "6", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            rec = summary["stop_on_move"]
            self.assertEqual((rec["push_row_cut"], rec["mend_after"], rec["dipm_after_release"]), (1, False, 0x0002), rec)
            self.assertTrue(rec["verdict"].startswith("STOP ACCEPTED in PIO mode: the move of push row 1"), rec["verdict"])
            self.assertIn("short of its push end 9.50 mm", rec["verdict"])
            self.assertNotIn("INCONCLUSIVE", rec["verdict"])
            self.assertEqual(verdicts(summary)["restore"], "PASS")

    def test_pio_watch_stop_after_the_push_ran_out_stays_inconclusive(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--stop-on-move", "--sim-scenario", "push-done", "--seconds", "6", results=tmp)
            self.assertEqual(code, 0, text[-3000:])
            rec = summary["stop_on_move"]
            self.assertIsNone(rec["push_row_cut"], rec)
            self.assertIn("INCONCLUSIVE", rec["verdict"])
            self.assertEqual(verdicts(summary)["restore"], "PASS")

    def test_push_row_cut_needs_mend_0_the_held_input_and_a_rest_short_of_the_push_end(self) -> None:
        rows = [{"n": 1, "label": "row 1 4.1 mm", "approach_mm": 4.1, "end_mm": 9.5}]
        rest = {"moving": False, "target_reached": False, "pushing": False, "push_missed": False, "DIPM": 2, "pos_mm": 4.11}
        self.assertEqual(pio_watch.push_row_cut(rows, ["row 1 4.1 mm"], rest, 2, 2)["n"], 1)
        for change, dipm_after in (({"target_reached": True}, 2),  # MEND 1: the move finished
                                   ({"moving": True}, 2), ({"pushing": True}, 2), ({"push_missed": True}, 2),
                                   ({"DIPM": 0}, 2), ({}, 0), ({}, None)):  # the start input released
            self.assertIsNone(pio_watch.push_row_cut(rows, ["row 1 4.1 mm"], {**rest, **change}, 2, dipm_after), (change, dipm_after))
        self.assertIsNone(pio_watch.push_row_cut(rows, ["row 1 4.1 mm"], rest, 0, 0))  # no start input seen
        self.assertIsNone(pio_watch.push_row_cut(rows, ["row 0 3.0 mm"], {**rest, "pos_mm": 3.0}, 2, 2))  # a plain row's target
        self.assertIsNone(pio_watch.push_row_cut(rows, ["row 1 4.1 mm", "row 0 4.1 mm"], rest, 2, 2))  # also a plain row's target
        self.assertIsNone(pio_watch.push_row_cut([{**rows[0], "end_mm": 4.2}], ["row 1 4.1 mm"], rest, 2, 2))  # at the push end

    def test_pio_watch_refuses_the_hand_over_while_an_input_is_on(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--sim-scenario", "dipm-on", "--seconds", "3", results=tmp)
            self.assertEqual(code, 2, text[-2000:])
            self.assertNotIn("watch", verdicts(summary))

    def test_pio_watch_that_cannot_confirm_pc_mode_says_so(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--sim-scenario", "no-restore", "--seconds", "2", "--settle-timeout", "2", results=tmp)
            self.assertEqual(code, 6, text[-2000:])
            self.assertEqual(verdicts(summary)["restore"], "FAIL")
            self.assertIn("mode pc", text)

    def test_ctrl_c_in_pio_mode_restores_pc_mode_first(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            code, text, summary = run("pio-watch", "--sim", "--sim-scenario", "interrupt", "--seconds", "10", results=tmp)
            self.assertEqual(code, 130, text[-2000:])
            self.assertEqual(verdicts(summary)["restore"], "PASS")


if __name__ == "__main__":
    unittest.main()
