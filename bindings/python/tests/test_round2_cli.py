"""CLI commands against the simulator: change speed, pause / cancel-pause, teach, backup, line, status flags,
servo reasons, stop-all (with the broadcast option), the line options and the no-answer hint.

Simulator only: no serial port is opened.
"""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from iai_actuator_driver import cli
from iai_actuator_driver.errors import NO_ANSWER_HINT
from iai_actuator_driver.sim import SimulatedEC

from test_cli import SimFactory, run  # the shared helpers of the CLI tests


def _frames(sim: SimulatedEC) -> list[bytes]:
    """The requests a simulated unit has received (its own log)."""
    return [bytes(f) for f in sim.requests]


class ParserTest(unittest.TestCase):
    def test_new_commands_and_options_are_in_the_help(self) -> None:
        text = cli.build_parser().format_help()
        for word in ("pause", "resume", "cancel-pause", "teach", "backup", "line", "--line", "--position-unit",
                     "register"):
            self.assertIn(word, text)
        self.assertIn("More commands, one line each", cli.__doc__)
        self.assertEqual(cli.build_parser().parse_args(["--sim", "status"]).retries, 3)
        self.assertIsNone(cli.build_parser().parse_args(["--sim", "status"]).strategy)
        self.assertEqual(cli.build_parser().parse_args(["--sim", "status"]).line, "auto")

    def test_stop_all_is_a_port_level_command(self) -> None:
        self.assertEqual(cli._port_command(["--json", "stop-all", "--ports", "COM3"]), "stop-all")
        args = cli.build_port_parser().parse_args(["stop-all", "--sim", "--broadcast"])
        self.assertTrue(args.broadcast)


class StatusFlagsTest(unittest.TestCase):
    def test_status_prints_the_condition_flags_that_are_set(self) -> None:
        code, out, _ = run("--sim", "status")
        self.assertEqual(code, 0)
        flags = next(line for line in out.splitlines() if line.startswith("flags"))
        self.assertNotIn("EMG-INPUT", flags)
        self.assertIn("row done", out)

        def drive_cut(sim: SimulatedEC) -> None:
            sim.set_inputs(drive_cut=True)

        code, out, _ = run("--sim", "status", factory=SimFactory(setup=drive_cut))
        self.assertEqual(code, 0)
        self.assertIn("DRIVE-POWER-CUT", next(line for line in out.splitlines() if line.startswith("flags")))

    def test_status_json_has_the_flags(self) -> None:
        code, out, _ = run("--sim", "--json", "status")
        data = json.loads(out)
        for key in ("emg_input", "drive_power_cut", "enable_off", "cold_start_alarm", "position_complete_no"):
            self.assertIn(key, data)

    def test_servo_on_is_refused_with_its_reasons(self) -> None:
        def enable_released(sim: SimulatedEC) -> None:
            sim.set_inputs(enable=False)

        code, _, err = run("--sim", "servo", "on", factory=SimFactory(setup=enable_released))
        self.assertEqual(code, 2)
        self.assertIn("ServoBlockedError", err)
        self.assertIn("enable switch", err)


class ChangeSpeedTest(unittest.TestCase):
    def test_change_speed_of_the_running_move(self) -> None:
        code, out, err = run("--sim", "--sim-time-scale", "2", "--max-speed", "200", "move", "60", "--speed", "10",
                             "--accel", "0.3", "--change-speed", "80", "--change-after", "0.2")
        self.assertEqual(code, 0, err)
        self.assertIn("changing the speed of the running move to 80", err)
        self.assertIn("60.00 mm", out)

    def test_change_options_need_a_plain_waiting_move(self) -> None:
        for extra in (["--no-wait"], ["--decel", "0.1"]):
            code, _, err = run("--sim", "move", "60", "--change-speed", "50", *extra)
            self.assertEqual(code, 2)
            self.assertIn("plain move that waits", err)

    def test_change_speed_outside_the_range_is_refused(self) -> None:
        code, _, err = run("--sim", "--sim-time-scale", "2", "move", "60", "--speed", "10", "--accel", "0.3",
                           "--change-speed", "5000", "--change-after", "0.2")
        self.assertEqual(code, 2)
        self.assertIn("SoftLimitError", err)


class PauseTest(unittest.TestCase):
    def test_cancel_pause_without_a_pause_is_refused_and_sends_nothing(self) -> None:
        code, _, err = run("--sim", "cancel-pause")
        self.assertEqual(code, 2)
        self.assertIn("no pause is active", err)

    def test_pause_then_cancel_pause_on_the_same_unit(self) -> None:
        factory = SimFactory(time_scale=2.0)
        code, out, _ = run("--sim", "--sim-time-scale", "2", "pause", factory=factory)
        self.assertEqual(code, 0)
        self.assertIn("paused", out)
        self.assertIn("resume", out)
        pause_on = bytes.fromhex("01 05 04 0A FF 00")
        self.assertTrue(any(bytes(frame).startswith(pause_on) for frame in _frames(factory.instances[0])),
                        "the STP coil was written ON")

    def test_resume(self) -> None:
        code, out, _ = run("--sim", "resume")
        self.assertEqual(code, 0)
        self.assertIn("pause released", out)


class TeachTest(unittest.TestCase):
    def test_teach_needs_no_option(self) -> None:
        code, out, err = run("--sim", "--unit-max", "teach", "2")
        self.assertEqual(code, 0, err)
        self.assertIn("row 2 taught from the current position: 73.95 mm", out)
        commands = cli.build_parser()._subparsers._group_actions[0].choices  # noqa: SLF001
        for name in ("teach", "baud"):  # the old options are hidden from the help
            self.assertNotIn("--allow-", commands[name].format_help())
        pset = commands["param"]._subparsers._group_actions[0].choices["set"]  # noqa: SLF001
        self.assertNotIn("--allow-", pset.format_help())

    def test_teach_writes_the_current_position(self) -> None:
        code, out, err = run("--sim", "--unit-max", "teach", "2", "--allow-nonvolatile")  # still accepted (ignored)
        self.assertEqual(code, 0, err)
        self.assertIn("row 2 taught from the current position: 73.95 mm", out)

    def test_teach_json(self) -> None:
        code, out, err = run("--sim", "--unit-max", "--json", "teach", "2", "--allow-nonvolatile")
        self.assertEqual(code, 0, err)
        self.assertEqual(json.loads(out)["position_mm"], 73.95)


class BackupTest(unittest.TestCase):
    def test_backup_writes_a_read_only_json_file(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "unit.json"
            factory = SimFactory()
            code, out, err = run("--sim", "backup", str(target), "--rows", "0,1", factory=factory)
            self.assertEqual(code, 0, err)
            self.assertIn("backup written to", out)
            data = json.loads(target.read_text(encoding="utf-8"))
        self.assertIs(data["meta"]["read_only"], True)
        self.assertEqual(len(data["positions"]["rows"]), 2)
        self.assertIn("2 position rows", out)

    def test_rows_option_is_checked(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                cli._rows_option("x")
            self.assertEqual(cli._rows_option("0, 2"), [0, 2])
            self.assertEqual(cli._rows_option("all"), "all")
            code, _, err = run("--sim", "backup", str(Path(folder) / "x.json"), "--rows", "abc")
        self.assertEqual(code, 2)
        self.assertIn("--rows", err)


class LineTest(unittest.TestCase):
    def test_line_on_a_direct_unit(self) -> None:
        code, out, _ = run("--sim", "line")
        self.assertEqual(code, 0)
        self.assertIn("line       direct", out)
        self.assertIn("coils yes", out)
        self.assertIn("stop path coil", out)

    def test_line_on_an_rcon_gateway(self) -> None:
        code, out, err = run("--sim", "--sim-line", "rcon", "--position-unit", "0.01", "line", "--monitor")
        self.assertEqual(code, 0, err)
        self.assertIn("line       rcon", out)
        self.assertIn("coils NO", out)
        self.assertIn("strategy   register; stop path register", out)
        self.assertIn("pos. unit  0.01 mm (stated)", out)
        self.assertIn("MANU", out)
        self.assertIn("monitor    axis 0", out)
        self.assertIn("line rcon", err)  # noted on stderr when the port opens

    def test_line_json(self) -> None:
        code, out, err = run("--sim", "--sim-line", "rcon", "--position-unit", "0.001", "--json", "line", "--monitor")
        self.assertEqual(code, 0, err)
        data = json.loads(out)
        self.assertEqual(data["line"]["kind"], "rcon")
        self.assertTrue(data["gateway_status"]["manu"])
        self.assertEqual(data["monitor"]["slave"], 1)
        self.assertEqual(data["errors"], [])

    def test_monitor_flag_on_a_direct_line_is_explained(self) -> None:
        code, out, _ = run("--sim", "line", "--monitor")
        self.assertEqual(code, 1)
        self.assertIn("RCON gateway only", out)

    def test_motion_on_rcon_needs_the_position_unit(self) -> None:
        code, _, err = run("--sim", "--sim-line", "rcon", "--stroke", "0", "100", "move", "10")
        self.assertEqual(code, 2)
        self.assertIn("PositionUnitError", err)

    def test_status_on_rcon_names_the_line(self) -> None:
        code, out, err = run("--sim", "--sim-line", "rcon", "status")
        self.assertEqual(code, 0, err)
        self.assertIn("line       rcon", out)


class StopAllTest(unittest.TestCase):
    def test_stop_all_on_a_simulated_line(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stderr(err):
            code = cli.main(["stop-all", "--sim", "--sim-slaves", "1-3"], out=out)
        self.assertEqual(code, 0, err.getvalue())
        text = out.getvalue()
        for name in ("sim:1", "sim:2", "sim:3"):
            self.assertIn(f"{name}: stopped", text)
        self.assertIn("not a safety function", text)

    def test_stop_all_with_the_broadcast_option(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        created: list = []
        real = cli.SimLine.with_slaves

        def make(*args, **kwargs):  # type: ignore[no-untyped-def]
            line = real(*args, **kwargs)
            created.append(line)
            return line

        with contextlib.redirect_stderr(err), mock.patch.object(cli.SimLine, "with_slaves", side_effect=make):
            code = cli.main(["--json", "stop-all", "--sim", "--sim-slaves", "1-2", "--broadcast"], out=out)
        self.assertEqual(code, 0, err.getvalue())
        data = json.loads(out.getvalue())
        self.assertEqual(sorted(data), ["sim:1", "sim:2"])
        frames = [f for _, f in created[0].requests]
        self.assertIn(bytes.fromhex("00 05 04 2C FF 00 4D 12"), frames)
        self.assertIn(bytes.fromhex("00 05 04 2C 00 00 0C E2"), frames)

    def test_stop_all_without_broadcast_sends_no_broadcast(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        created: list = []
        real = cli.SimLine.with_slaves

        def make(*args, **kwargs):  # type: ignore[no-untyped-def]
            line = real(*args, **kwargs)
            created.append(line)
            return line

        with contextlib.redirect_stderr(err), mock.patch.object(cli.SimLine, "with_slaves", side_effect=make):
            self.assertEqual(cli.main(["stop-all", "--sim", "--sim-slaves", "1-2"], out=out), 0, err.getvalue())
        self.assertNotIn(0, [f[0] for _, f in created[0].requests], "address 0 was never used")


class NoAnswerHintTest(unittest.TestCase):
    def test_the_hint_names_the_unsupported_controllers(self) -> None:
        for word in ("SEL protocol", "XSEL", "SSEL", "ASEL", "PSEL", "MSEL", "TTA", "do not speak Modbus RTU",
                     "RCON-EC / REC", "serial port disabled", "24 V", "IA-OS"):
            self.assertIn(word, NO_ANSWER_HINT)

    def test_open_adds_the_hint_when_nothing_answers(self) -> None:
        from iai_actuator_driver.ec import Elecylinder
        from iai_actuator_driver.errors import ModbusTimeout

        port = SimulatedEC(baudrate=115200, ec_baudrate=1)  # answers at no rate the auto-detection tries
        with self.assertRaises(ModbusTimeout) as caught:
            Elecylinder.open(port)
        self.assertTrue(port.closed)
        notes = " ".join(getattr(caught.exception, "__notes__", []))
        self.assertIn("XSEL", notes)

    def test_scan_prints_the_hint_when_no_axis_answers(self) -> None:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stderr(err):
            code = cli.main(["scan", "--sim", "--sim-slaves", "5", "--slaves", "1-2"], out=out)
        self.assertEqual(code, 1)
        self.assertIn("no axis answered", out.getvalue())
        self.assertIn("SEL protocol", out.getvalue())

    def test_gui_messages_carry_the_hint(self) -> None:
        gui = Path(__file__).resolve().parents[3] / "apps" / "gui"  # the GUI of the source checkout
        for name in ("backend.py", "workbench.py"):
            self.assertIn("NO_ANSWER_HINT", (gui / name).read_text(encoding="utf-8"), name)


if __name__ == "__main__":
    unittest.main()
