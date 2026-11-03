"""CLI (python -m iai_actuator_driver / iai-check) against the simulator, including the compatibility check."""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from iai_actuator_driver import cli
from iai_actuator_driver.sim import SimulatedEC


class SimFactory:
    """Stands in for ``cli.SimulatedEC`` so tests can inspect (and pre-configure) the simulator."""

    def __init__(self, setup=None, **sim_kwargs: object) -> None:  # type: ignore[no-untyped-def]
        self.instances: list[SimulatedEC] = []
        self.setup = setup
        self.sim_kwargs = sim_kwargs

    def __call__(self, **kwargs: object) -> SimulatedEC:
        sim = SimulatedEC(**{**kwargs, **self.sim_kwargs})  # type: ignore[arg-type]
        if self.setup is not None:
            self.setup(sim)
        self.instances.append(sim)
        return sim


def run(*argv: str, factory: SimFactory | None = None, entry=cli.main) -> tuple[int, str, str]:  # type: ignore[no-untyped-def]
    out, err = io.StringIO(), io.StringIO()
    with mock.patch.object(cli, "SimulatedEC", factory or SimFactory()), contextlib.redirect_stderr(err):
        code = entry(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class CommandsTest(unittest.TestCase):
    def test_status(self) -> None:
        code, out, _ = run("--sim", "status")
        self.assertEqual(code, 0)
        self.assertIn("73.95 mm", out)
        self.assertIn("SERVO HOMED IN-POS MODBUS", out)
        self.assertIn("DSS1=0x3098", out)

    def test_status_json(self) -> None:
        code, out, _ = run("--sim", "--json", "status")
        self.assertEqual(code, 0)
        data = json.loads(out)
        self.assertEqual(data["position_mm"], 73.95)
        self.assertTrue(data["ready"])

    def test_identify(self) -> None:
        code, out, _ = run("--sim", "identify")
        self.assertEqual(code, 0)
        self.assertIn("model      MWB-EC-DSRR4-WA-MF", out)
        self.assertIn("controller EC 24 V, unit 0x02 (EC dust/drip-proof)", out)
        self.assertIn("actuator   slider (small) (AKID 9)", out)
        self.assertIn("axis no.   0 (AXNO 0x8004; slave 1)", out)
        # conservative default cap (100 mm/s), lowered to the unit's own maximum; --unit-max lifts it
        self.assertIn("max speed  unit max 700.00 mm/s (MAXV 7.4); driver cap 100.00 mm/s", out)
        self.assertIn("units      mm (kind slider (small))", out)
        self.assertIn("push       20 % (recommended) .. 60 % (MXPP); driver cap 60 %", out)
        self.assertIn("CI=0x0103 DA=0x9900", out)
        code, out, _ = run("--sim", "--json", "identify")
        self.assertEqual(json.loads(out)["firmware"], "0013")
        code, out, _ = run("--sim", "--unit-max", "identify")
        self.assertIn("max speed  unit max 700.00 mm/s (MAXV 7.4); driver cap 700.00 mm/s", out)
        self.assertIn("max accel  unit max 1.00 / 1.00 G (MAXA / MAXD); driver cap 1.00 G", out)
        code, out, _ = run("--sim", "--max-speed", "500", "identify")
        self.assertIn("driver cap 500.00 mm/s", out)

    def test_default_caps_are_conservative(self) -> None:
        # a typo such as --speed 500 is refused by the default 100 mm/s cap; --unit-max allows the unit's own maximum
        code, _, err = run("--sim", "--sim-time-scale", "50", "move", "80", "--speed", "500")
        self.assertEqual(code, 2)
        self.assertIn("max_speed_mm_s", err)
        code, _, err = run("--sim", "--sim-time-scale", "50", "--unit-max", "move", "80", "--speed", "500")
        self.assertEqual(code, 0, err)
        # goto: a stored row faster than the default cap needs --allow-fast (or --max-speed / --unit-max)
        code, _, err = run("--sim", "--sim-profile", "pcon", "--sim-time-scale", "50", "goto", "0")
        self.assertEqual(code, 2)
        self.assertIn("allow_above_caps", err)
        code, _, err = run("--sim", "--sim-profile", "pcon", "--sim-time-scale", "50", "goto", "0", "--allow-fast")
        self.assertEqual(code, 0, err)

    def test_move_and_limits(self) -> None:
        factory = SimFactory()
        code, out, _ = run("--sim", "--sim-time-scale", "20", "move", "80", "--speed", "50", factory=factory)
        self.assertEqual(code, 0)
        self.assertIn("80.00 mm", out)
        self.assertAlmostEqual(factory.instances[0].position_mm, 80.0, places=6)
        code, _, err = run("--sim", "move", "200")
        self.assertEqual(code, 2)
        self.assertIn("SoftLimitError", err)
        code, _, err = run("--sim", "move", "80", "--speed", "900")  # above MAXV 700
        self.assertEqual(code, 2)
        self.assertIn("speed", err)

    def test_move_no_wait(self) -> None:
        code, out, _ = run("--sim", "move", "75", "--no-wait")
        self.assertEqual(code, 0)
        self.assertIn("sent", out)

    def test_servo_home_stop_reset(self) -> None:
        def no_pmsl(sim: SimulatedEC) -> None:
            sim.pmsl = False

        factory = SimFactory(no_pmsl)
        code, out, _ = run("--sim", "servo", "on", factory=factory)
        self.assertEqual(code, 0)
        self.assertIn("enabling Modbus control (PMSL) while the axis is at rest", out)
        self.assertTrue(factory.instances[0].pmsl)
        code, out, _ = run("--sim", "servo", "off")
        self.assertEqual(code, 0)
        self.assertNotIn("SERVO", out.split("flags")[1].splitlines()[0])
        code, out, _ = run("--sim", "--sim-time-scale", "50", "--strategy", "iai", "home")
        self.assertEqual(code, 0)
        self.assertIn("0.00 mm", out)
        self.assertEqual(run("--sim", "stop")[0], 0)
        self.assertEqual(run("--sim", "reset")[0], 0)

    def test_json_output_stays_one_document_when_modbus_control_is_enabled(self) -> None:
        def no_pmsl(sim: SimulatedEC) -> None:
            sim.pmsl = False

        note = "enabling Modbus control (PMSL) while the axis is at rest"
        code, out, err = run("--sim", "--json", "mode", "pc", factory=SimFactory(no_pmsl))
        self.assertEqual(code, 0, err)
        self.assertTrue(json.loads(out)["modbus_enabled"])  # stdout is the JSON document and nothing else
        self.assertNotIn(note, out)
        self.assertIn(note, err)  # the note is kept, on stderr (the C++ command line does the same)

    def test_frame_logging(self) -> None:
        code, _, err = run("--sim", "--log-frames", "status")
        self.assertEqual(code, 0)
        self.assertIn("tx", err)
        self.assertIn("01 03 90 00 00 16", err)  # 22 words: PNOW..SSSE

    def test_requires_port_or_sim(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            cli.main(["status"])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            cli.main(["--sim", "--port", "COM99", "status"])

    def test_link_error_exit_code(self) -> None:
        def dead(sim: SimulatedEC) -> None:
            sim.inject_fault("timeout", 100)

        code, _, err = run("--sim", "status", factory=SimFactory(dead))
        self.assertEqual(code, 2)
        self.assertIn("ModbusTimeout", err)


class CheckTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.report = Path(self.tmp.name) / "report.json"

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def load(self) -> dict:  # type: ignore[type-arg]
        return json.loads(self.report.read_text(encoding="utf-8"))

    def test_read_only_check(self) -> None:
        factory = SimFactory()
        code, out, _ = run("--sim", "check", "--report", str(self.report), factory=factory)
        self.assertEqual(code, 0, out)
        for name in ("link", "slave_id", "index_table", "status", "alarm", "position_table", "direct_move_block"):
            self.assertRegex(out, rf"PASS  {name}")
        self.assertIn("SKIP  motion", out)
        self.assertIn("RESULT PASS", out)
        report = self.load()
        self.assertEqual(report["result"], "PASS")
        self.assertEqual(report["model"], "MWB-EC-DSRR4-WA-MF")
        self.assertEqual(report["controller"], "EC 24 V, unit 0x02 (EC dust/drip-proof)")
        self.assertEqual(report["ec_max_speed_mm_s"], 700.0)
        self.assertEqual(report["speed_cap_mm_s"], 100.0)  # no --max-speed: the conservative default cap
        self.assertRegex(out, r"PASS  speed_limit")
        self.assertEqual(report["firmware"], "0013")
        self.assertIn("timestamp", report)
        self.assertFalse(report["motion_requested"])
        sent_functions = {frame[1] for frame in factory.instances[0].requests}
        self.assertLessEqual(sent_functions, {0x03, 0x11})  # read-only: nothing but reads and slave id

    def test_motion_check_returns_to_start(self) -> None:
        factory = SimFactory()
        code, out, _ = run(
            "--sim", "--sim-time-scale", "10", "check", "--motion", "--report", str(self.report), factory=factory
        )
        self.assertEqual(code, 0, out)
        self.assertIn("PASS  motion_out", out)
        self.assertIn("PASS  motion_back", out)
        report = self.load()
        items = {item["name"]: item for item in report["items"]}
        self.assertAlmostEqual(items["motion_out"]["data"]["reached_mm"], 78.95, delta=0.1)
        self.assertAlmostEqual(factory.instances[0].position_mm, 73.95, places=6)
        self.assertTrue(report["motion_requested"])

    def test_motion_check_near_end_goes_backwards(self) -> None:
        factory = SimFactory(position_mm=148.0)
        code, out, _ = run(
            "--sim", "--sim-time-scale", "10", "check", "--motion", "--report", str(self.report), factory=factory
        )
        self.assertEqual(code, 0, out)
        items = {item["name"]: item for item in self.load()["items"]}
        self.assertAlmostEqual(items["motion_out"]["data"]["target_mm"], 143.0, delta=0.001)
        self.assertAlmostEqual(factory.instances[0].position_mm, 148.0, places=6)

    def test_check_fails_on_alarm_and_skips_motion(self) -> None:
        def alarm(sim: SimulatedEC) -> None:
            sim.trigger_alarm(0x0E0)

        code, out, _ = run("--sim", "check", "--motion", "--report", str(self.report), factory=SimFactory(alarm))
        self.assertEqual(code, 1)
        self.assertIn("FAIL  alarm", out)
        self.assertIn("SKIP  motion", out)
        self.assertEqual(self.load()["result"], "FAIL")

    def test_check_no_link(self) -> None:
        def dead(sim: SimulatedEC) -> None:
            sim.inject_fault("timeout", 1000)

        code, out, _ = run("--sim", "check", "--report", str(self.report), factory=SimFactory(dead))
        self.assertEqual(code, 1)
        self.assertIn("FAIL  link", out)
        self.assertIn("SKIP  slave_id", out)

    def test_ec_check_entry_point(self) -> None:
        code, out, _ = run("--sim", "--report", str(self.report), entry=cli.check_main)
        self.assertEqual(code, 0, out)
        self.assertEqual(self.load()["tool"], "iai-check (unofficial)")

    def test_report_records_identity_and_limits(self) -> None:
        code, out, _ = run("--sim", "check", "--report", str(self.report))
        self.assertEqual(code, 0, out)
        report = self.load()
        self.assertEqual(report["serial_number"], "SIM-0000001")
        self.assertEqual(report["model_name"], "MWB-EC-DSRR4-WA-MF")
        self.assertEqual(report["ec_soft_limits_mm"], [-0.3, 150.3])
        self.assertEqual(report["stroke"], [0.0, 150.0])
        self.assertIsNone(report["configured_stroke"])
        for name in ("identification", "soft_limits", "alarm_history", "safety_speed"):
            self.assertRegex(out, rf"PASS  {name}")

    def test_unwritable_report_is_not_a_device_write(self) -> None:
        factory = SimFactory()
        missing = Path(self.tmp.name) / "no" / "such" / "dir" / "r.json"
        with mock.patch("tempfile.gettempdir", return_value=self.tmp.name):
            code, out, err = run("--sim", "--log-frames", "check", "--report", str(missing), factory=factory)
        self.assertEqual(code, 0, out)
        self.assertIn("RESULT PASS", out)
        self.assertIn("fallback", out)
        self.assertTrue((Path(self.tmp.name) / "r.json").is_file())
        self.assertIn("could not write", err)
        self.assertNotIn("01 05 04 2c", err)  # no STOP (or any coil) written by the read-only check
        self.assertLessEqual({f[1] for f in factory.instances[0].requests}, {0x03, 0x11})
        with mock.patch("tempfile.gettempdir", return_value=str(Path(self.tmp.name) / "also-missing")):
            code, out, _ = run("--sim", "check", "--report", str(missing))
        self.assertEqual(code, 0)
        self.assertIn("RESULT PASS - report printed above", out)
        self.assertIn('"tool": "iai-check (unofficial)"', out)

    def test_fc11_silent_is_a_warning(self) -> None:
        class SilentFc11(SimulatedEC):
            def on_request(self, frame: bytes) -> bytes | None:
                return None if frame[1] == 0x11 else NotImplemented

        out_buf = io.StringIO()
        with mock.patch.object(cli, "SimulatedEC", SilentFc11), contextlib.redirect_stderr(io.StringIO()):
            code = cli.main(["--sim", "check", "--report", str(self.report)], out=out_buf)
        out = out_buf.getvalue()
        self.assertEqual(code, 0, out)
        self.assertIn("WARN  slave_id", out)
        self.assertIn("identity read from CTCL 0x0300", out)  # the same identity, from class 3 item 1
        self.assertIn("PASS  index_table", out)
        self.assertIn("RESULT PASS", out)

    def test_configured_stroke_checked_against_the_ec(self) -> None:
        code, out, _ = run("--sim", "--stroke", "0", "300", "check", "--report", str(self.report))
        self.assertEqual(code, 0, out)
        self.assertIn("WARN  soft_limits", out)
        self.assertIn("narrows it to 0.00..150.00", out)
        code, out, _ = run("--sim", "--stroke", "200", "300", "check", "--motion", "--report", str(self.report))
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL  soft_limits", out)
        self.assertIn("SKIP  motion", out)

    def test_short_ec_with_default_150_stroke(self) -> None:
        factory = SimFactory(stroke_mm=50.0, position_mm=25.0)
        code, out, _ = run("--sim", "--stroke", "0", "150", "check", "--report", str(self.report), factory=factory)
        self.assertEqual(code, 0, out)
        self.assertIn("WARN  soft_limits", out)
        self.assertIn("WARN  position_table", out)  # rows at 150 and 75 mm lie outside 0..50
        self.assertEqual(self.load()["stroke"], [0.0, 50.0])

    def test_safety_speed_reported(self) -> None:
        def sfty(sim: SimulatedEC) -> None:
            sim.safety_speed = True

        code, out, _ = run("--sim", "check", "--report", str(self.report), factory=SimFactory(sfty))
        self.assertEqual(code, 0, out)
        self.assertIn("WARN  safety_speed", out)
        items = {item["name"]: item for item in self.load()["items"]}
        self.assertTrue(items["safety_speed"]["data"]["safety_speed"])


class NewCommandsTest(unittest.TestCase):
    def test_alarms(self) -> None:
        code, out, _ = run("--sim", "alarms")
        self.assertEqual(code, 0)
        self.assertIn("ABS error (2)", out)
        self.assertIn("PowerUP", out)
        code, out, _ = run("--sim", "--json", "alarms")
        self.assertEqual(json.loads(out)[1]["code"], 0x0EE)

    def test_identify_shows_limits_and_serial(self) -> None:
        code, out, _ = run("--sim", "identify")
        self.assertEqual(code, 0)
        self.assertIn("serial     SIM-0000001", out)
        self.assertIn("soft limit -0.30..150.30 mm", out)
        self.assertIn("stroke     0.00..150.00 mm", out)

    def test_jog_and_push(self) -> None:
        factory = SimFactory()
        code, out, _ = run("--sim", "--sim-time-scale", "20", "jog", "5", factory=factory)
        self.assertEqual(code, 0, out)
        self.assertIn("78.95 mm", out)
        code, out, _ = run("--sim", "--sim-time-scale", "20", "push", "80", "10", "30")
        self.assertEqual(code, 0, out)
        self.assertIn("push missed at 90.00 mm", out)
        code, _, err = run("--sim", "push", "70", "10", "30")
        self.assertEqual(code, 2)
        self.assertIn("push zone", err)


if __name__ == "__main__":
    unittest.main()
