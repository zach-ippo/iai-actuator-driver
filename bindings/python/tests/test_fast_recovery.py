"""The fast path wired to power / link-loss recovery: ``Axis.recover`` / ``track`` / ``check_link``, the status
monitor's link watch, ``disable_fast``, the cell's monitors, ``recovery.session_from_backup`` and the command line
(``--fast``, ``recover``, ``monitor --watch``).

Simulator only: no serial port is opened.
"""

from __future__ import annotations

import contextlib
import io
import json
import logging
import re
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from iai_actuator_driver import cli
from iai_actuator_driver import recovery as rc
from iai_actuator_driver.cell import Cell
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.info import backup
from iai_actuator_driver.modbus import TransportError
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_ports

from ec_testutil import FrameRecorder, make_ec

QUICK = rc.RecoveryPolicy(poll_s=0.02)
OPEN = {"poll_interval_s": 0.005, "scan_timeout": 0.01}
for _name in ("iai_actuator_driver.recovery", "iai_actuator_driver.monitor", "iai_actuator_driver.cell"):
    logging.getLogger(_name).setLevel(logging.CRITICAL)  # every event / stop is also logged as a warning

LEAN_KEYS = {"PNOW", "ALMC", "DIPM", "DOPM", "DSS1", "DSS2", "DSSE"}


def wait_until(predicate, timeout: float = 5.0) -> bool:  # type: ignore[no-untyped-def]
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def open_axis(sim: SimulatedEC | SimLine) -> tuple[Elecylinder, FrameRecorder]:
    rec = FrameRecorder()
    ax = Elecylinder.open(sim, frame_logger=rec, poll_interval_s=0.005)  # type: ignore[arg-type]
    ax.connect()
    return ax, rec


class Events:
    def __init__(self) -> None:
        self.items: list[rc.RecoveryEvent] = []

    def __call__(self, event: rc.RecoveryEvent) -> None:
        self.items.append(event)

    @property
    def kinds(self) -> list[str]:
        return [e.kind for e in self.items]

    @property
    def pairs(self) -> list[tuple[str, str | None]]:
        return [(e.kind, e.cause) for e in self.items]


# ====================================================================== Axis.recover / track / check_link


class AxisRecoverTest(unittest.TestCase):
    def test_policy_may_be_a_mapping_of_its_fields_and_anything_else_is_a_type_error(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        sim.power_cycle()
        report = ax.recover(timeout=5, policy={"poll_s": 0.02, "restore_control": False})
        self.assertTrue(report.same_unit)
        with self.assertRaisesRegex(TypeError, "policy"):
            ax.recover(timeout=1, policy={"no_such_field": 1})
        with self.assertRaisesRegex(TypeError, "policy"):
            ax.recover(timeout=1, policy=3)

    def test_without_track_the_identity_read_at_connect_is_the_baseline(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        self.assertIsNone(rc.session_of(ax))
        self.assertIsNotNone(ax.device_info)
        sim.power_cycle()
        report = ax.recover(timeout=5, policy=QUICK)
        self.assertTrue(report.same_unit)
        self.assertTrue(report.needs_home)  # an incremental rod lost HEND
        sess = rc.session_of(ax)
        assert sess is not None and ax.device_info is not None
        self.assertEqual(sess.serial_number, ax.device_info.serial_number)
        self.assertEqual(ax.link_state, rc.LINK_OK)

    def test_track_and_check_link_wrappers(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, rec = open_axis(sim)
        self.addCleanup(ax.close)
        sess = ax.track(name="G")
        self.assertIs(rc.session_of(ax), sess)
        rec.clear()
        chk = ax.check_link()
        self.assertTrue(chk.ok)
        self.assertEqual([f[1] for f in rec.tx], [0x03])  # one status read, nothing written
        sim.power_cycle()
        self.assertEqual(ax.check_link().state, rc.UNIT_RESTARTED)
        self.assertEqual(ax.link_state, rc.UNIT_RESTARTED)

    def test_a_replaced_unit_is_refused(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        other = SimulatedEC(profile="gripper", time_scale=50, serial_number="OTHER-UNIT-0001")
        ax.bus.transport = other  # the cable now leads to another unit
        with self.assertRaises(rc.UnitReplacedError):
            ax.recover(timeout=1, policy=QUICK)
        self.assertEqual(ax.link_state, rc.UNIT_REPLACED)

    def test_a_monitor_that_ended_on_the_vanished_port_is_restarted(self) -> None:
        line = SimLine([SimulatedEC(profile="gripper", time_scale=50)], name="L1")
        ax, _ = open_axis(line)
        self.addCleanup(ax.close)
        events = Events()
        mon = ax.start_monitor(on_event=events)
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        line.unplug()
        self.assertTrue(wait_until(lambda: not mon.running))
        self.assertIsInstance(mon.error, TransportError)
        self.assertEqual(events.pairs, [("link_lost", rc.PORT_VANISHED)])
        line.replug()
        report = ax.recover(timeout=5, policy=QUICK, on_event=events)
        self.assertTrue(report.port_reopened)
        self.assertIs(ax.monitor, mon)
        self.assertTrue(mon.running)
        polls = mon.polls
        self.assertTrue(wait_until(lambda: mon.polls > polls + 3), "the monitor polls again")
        self.assertEqual(events.kinds, ["link_lost", "recovered"])

    def test_a_running_monitor_is_held_during_the_recovery(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        ax.track()
        mon = ax.start_monitor()
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        sim.power_off()
        threading.Timer(0.2, sim.power_on).start()
        held: list[bool] = []
        report = ax.recover(timeout=5, policy=QUICK, on_event=lambda e: held.append(mon.suspended))
        self.assertTrue(report.restarted)
        self.assertTrue(held and all(held), "the monitor was suspended while recover() ran")
        self.assertTrue(mon.running)
        polls = mon.polls
        self.assertTrue(wait_until(lambda: mon.polls > polls + 3))

    def test_a_failed_recovery_leaves_the_monitor_as_it_was(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        ax.track()
        mon = ax.start_monitor()
        sim.power_off()
        with self.assertRaises(rc.RecoveryError):
            ax.recover(timeout=0.2, policy=QUICK)
        self.assertTrue(mon.running, "still running (polls fail until the power is back)")
        sim.power_on()


# ====================================================================== the monitor's link watch


class MonitorLinkWatchTest(unittest.TestCase):
    def test_power_off_is_link_lost_and_power_on_a_restart(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50)
        ax, rec = open_axis(sim)
        self.addCleanup(ax.close)
        events = Events()
        rec.clear()
        mon = ax.start_monitor(on_event=events, link_check_s=0.05)
        self.assertTrue(wait_until(lambda: mon.link_checks >= 2), "link checks run once the axis is identified")
        self.assertIsNotNone(rc.session_of(ax), "a session made from the identity connect() read")
        sim.power_off()
        self.assertTrue(wait_until(lambda: events.kinds == ["link_lost"]))
        self.assertEqual(events.items[0].cause, rc.UNIT_SILENT)
        self.assertTrue(mon.running, "a silent unit does not end the monitor")
        self.assertEqual(mon.link_state, rc.UNIT_SILENT)
        sim.power_on()
        self.assertTrue(wait_until(lambda: "unit_restarted" in events.kinds))
        self.assertEqual(events.kinds, ["link_lost", "unit_restarted"])
        self.assertTrue(any("uptime counter dropped" in e.message for e in events.items))
        self.assertEqual(ax.link_state, rc.UNIT_RESTARTED)
        self.assertEqual({f[1] for f in rec.tx}, {0x03}, "the link watch only reads")

    def test_no_link_check_without_a_session_but_a_lost_link_is_still_seen(self) -> None:
        ec, sim, rec = make_ec()
        self.addCleanup(ec.close)
        events = Events()
        mon = ec.start_monitor(on_event=events, link_check_s=0.01)
        self.assertTrue(wait_until(lambda: mon.polls >= 20))
        self.assertEqual(mon.link_checks, 0)
        self.assertIsNone(rc.session_of(ec))
        sim.power_off()
        self.assertTrue(wait_until(lambda: events.kinds == ["link_lost"]))
        self.assertGreaterEqual(mon.error_count, 3)
        sim.power_on()

    def test_unplug_is_port_vanished_and_ends_the_monitor(self) -> None:
        line = SimLine([SimulatedEC(profile="slider", time_scale=50)], name="L2")
        ax, _ = open_axis(line)
        self.addCleanup(ax.close)
        events = Events()
        mon = ax.start_monitor(on_event=events)
        self.assertTrue(wait_until(lambda: mon.polls >= 3))
        line.unplug()
        self.assertTrue(wait_until(lambda: not mon.running))
        self.assertEqual(events.pairs, [("link_lost", rc.PORT_VANISHED)])
        self.assertEqual(mon.link_state, rc.PORT_VANISHED)

    def test_deliberate_changes_by_the_axis_are_not_restarts(self) -> None:
        sim = SimulatedEC(profile="rod", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        ax.track()
        events = Events()
        mon = ax.start_monitor(on_event=events, link_check_s=0.02)
        self.assertTrue(wait_until(lambda: mon.link_checks >= 2))
        ax.disable_modbus_control()
        checks = mon.link_checks
        self.assertTrue(wait_until(lambda: mon.link_checks >= checks + 3))
        ax.enable_modbus_control()
        result = ax.software_reset()
        self.assertFalse(result.homed)  # the rod lost HEND at the reset: noted, not a restart
        checks = mon.link_checks
        self.assertTrue(wait_until(lambda: mon.link_checks >= checks + 3))
        self.assertEqual(events.items, [])
        self.assertEqual(ax.link_state, rc.LINK_OK)

    def test_link_check_interval_is_validated(self) -> None:
        ec, _, _ = make_ec()
        self.addCleanup(ec.close)
        for bad in (-1, True, "1", 4000):
            with self.assertRaises(ValueError):
                ec.start_monitor(link_check_s=bad)
        mon = ec.start_monitor(link_check_s=None)
        self.assertIsNone(mon.link_check_s)

    def test_disable_fast_undoes_enable_fast(self) -> None:
        ec, _, _ = make_ec()
        self.addCleanup(ec.close)
        result = ec.enable_fast(baudrate=None)
        self.assertTrue(result["monitor"] and ec.fast_mode)
        self.assertEqual(ec.completion, "fast")
        mon = ec.monitor
        self.assertEqual(ec.disable_fast(), {"baudrate": ec.baudrate, "completion": "safe", "monitor": False,
                                             "notes": []})  # fmt: skip
        self.assertFalse(ec.fast_mode)
        self.assertIsNone(ec.monitor)
        assert mon is not None
        self.assertFalse(mon.running)


# ====================================================================== the cell


class CellMonitorTest(unittest.TestCase):
    def test_status_all_takes_a_running_monitors_sample(self) -> None:
        ports = sim_ports(2, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            mons = cell.start_monitors(["SIM1:1"])
            self.assertTrue(wait_until(lambda: mons["SIM1:1"].polls >= 3))
            mark = len(ports["SIM1"].requests)
            res = cell.status_all()
            self.assertEqual(set(res["SIM1:1"].raw), LEAN_KEYS)  # the monitor's lean sample
            self.assertIn("STIM", res["SIM2:1"].raw)  # read
            fresh = [f for _, f in ports["SIM1"].requests[mark:] if f[:2] == b"\x01\x03" and f[5] == 22]
            self.assertEqual(fresh, [], "no full status read for the monitored axis")
            self.assertIn("STIM", cell.status_all(monitor_max_age_s=None)["SIM1:1"].raw)
            cell.stop_monitors()
            self.assertIsNone(cell["SIM1:1"].monitor)
            self.assertIn("STIM", cell.status_all()["SIM1:1"].raw)

    def test_monitor_link_events_go_to_the_fault_handling_and_recover_all_restarts_it(self) -> None:
        ports = sim_ports(2, time_scale=50)
        faults: list[tuple[str, str]] = []
        with Cell.open(ports, on_fault=lambda n, k, e: faults.append((n, k)), **OPEN) as cell:
            cell.track_all()
            mons = cell.start_monitors()
            self.assertTrue(wait_until(lambda: all(m.polls >= 3 for m in mons.values())))
            ports["SIM1"].unplug()
            self.assertTrue(wait_until(lambda: cell.faults == {"SIM1:1": rc.PORT_VANISHED}))
            self.assertTrue(wait_until(lambda: faults == [("SIM1:1", rc.PORT_VANISHED)]))
            self.assertEqual([(e.kind, e.axis) for e in cell.events], [("link_lost", "SIM1:1")])
            self.assertTrue(wait_until(lambda: not mons["SIM1:1"].running))  # the monitor thread ends just after the fault
            ports["SIM1"].replug()
            reports = cell.recover_all(timeout=3, policy=QUICK)
            self.assertTrue(reports["SIM1:1"].port_reopened)
            self.assertEqual(cell.faults, {})
            self.assertTrue(mons["SIM1:1"].running, "recover_all restarted the monitor")
            self.assertIs(cell["SIM1:1"].monitor, mons["SIM1:1"])
            self.assertTrue(mons["SIM2:1"].running)


# ====================================================================== edge cases


def far_target(ax: Elecylinder) -> float:
    """A target near the end of the stroke away from the axis."""
    lo, hi = ax.soft_limits_mm
    return hi - 0.5 if ax.status().position_mm < (lo + hi) / 2 else lo + 0.5


class RecoveryEdgeCaseTest(unittest.TestCase):
    def test_a_stopped_home_return_is_not_a_restart(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=1)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        ax.track()
        events = Events()
        mon = ax.start_monitor(on_event=events, link_check_s=0.02)
        ax.home(wait=False)
        self.assertTrue(wait_until(lambda: ax.status().homing))
        ax.stop()
        ax.finish_home()
        self.assertFalse(ax.status().homed)  # HEND 0, not homing: the home return was stopped
        checks = mon.link_checks
        self.assertTrue(wait_until(lambda: mon.link_checks >= checks + 3))
        self.assertEqual(events.items, [])
        self.assertEqual(ax.link_state, rc.LINK_OK)
        sess = rc.session_of(ax)
        assert sess is not None
        self.assertFalse(sess.homed, "the home return's loss of HEND is recorded as deliberate")

    def test_a_late_wait_on_a_finished_move_returns_its_status(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=10)
        ax, rec = open_axis(sim)
        self.addCleanup(ax.close)
        target = far_target(ax)
        handle = ax.move_to(target, speed_mm_s=50, wait=False, completion="safe")
        time.sleep(handle.timeout + 0.1)  # the application looked away past the move's estimate
        rec.clear()
        st = handle.wait()
        self.assertAlmostEqual(st.position_mm, target, delta=0.1)
        self.assertEqual([f for f in rec.tx if f[1] == 0x05], [], "no STOP sent")

    def test_a_move_still_running_after_a_vanished_port_is_reported_as_running(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=1)
        line = SimLine([sim], name="L5")
        ax, _ = open_axis(line)
        self.addCleanup(ax.close)
        ax.track()
        ax.move_to(far_target(ax), speed_mm_s=2, wait=False)
        line.unplug()  # the unit keeps power: the move runs on (no communication watchdog)
        line.replug()
        report = ax.recover(timeout=3, policy=QUICK)
        self.addCleanup(ax.stop)
        assert report.status is not None
        self.assertTrue(report.status.moving)
        self.assertTrue(report.needs_attention)
        self.assertIn("still moving", report.summary())
        self.assertTrue(any("still moving" in n for n in report.notes))
        self.assertFalse(any("cut off" in n for n in report.notes))

    def test_recovering_one_axis_of_a_shared_line_holds_the_other_monitors(self) -> None:
        units = [SimulatedEC(profile="gripper", time_scale=50, slave=s, baudrate=115200,
                             serial_number=f"SN-{s:04d}") for s in (1, 2)]  # fmt: skip
        line = SimLine(units, name="LS")
        faults: list[tuple[str, str]] = []
        with Cell.open([line], slaves=[1, 2], on_fault=lambda n, k, e: faults.append((n, k)), **OPEN) as cell:
            cell.track_all()
            mons = cell.start_monitors()
            self.assertTrue(wait_until(lambda: all(m.polls >= 3 for m in mons.values())))
            units[0].power_cycle()  # unit 1 only: back at 38400, unit 2 still at 115200
            self.assertTrue(wait_until(lambda: "LS:1" in cell.faults))
            reports = cell.recover_all(["LS:1"], timeout=5, policy=QUICK)
            self.assertTrue(reports["LS:1"].baud_restored)
            polls = mons["LS:2"].polls
            self.assertTrue(wait_until(lambda: mons["LS:2"].polls > polls + 5))
            time.sleep(0.2)  # a fault thread started by a failed sibling poll would have run by now
            self.assertEqual(faults, [("LS:1", rc.UNIT_SILENT)])
            self.assertEqual(cell.faults, {})
            self.assertEqual(cell["LS:2"].link_state, rc.LINK_OK)


# ====================================================================== session_from_backup


class SessionFromBackupTest(unittest.TestCase):
    def test_the_unit_must_be_the_one_backed_up(self) -> None:
        sim = SimulatedEC(profile="gripper", time_scale=50)
        ax, _ = open_axis(sim)
        self.addCleanup(ax.close)
        data = backup(ax)
        sess = rc.session_from_backup(ax, data, name="G")
        self.assertEqual(sess.serial_number, data["identity"]["serial_number"])
        self.assertIs(rc.session_of(ax), sess)
        report = ax.recover(timeout=2, policy=rc.RecoveryPolicy(poll_s=0.02, check_rows=True, check_parameters=True))
        self.assertTrue(report.same_unit)
        self.assertTrue(report.rows_checked)
        self.assertGreater(report.parameters_checked, 0)
        self.assertEqual((report.row_problems, report.parameter_differences), ([], []))
        other = json.loads(json.dumps(data))
        other["identity"]["serial_number"] = "SOMEONE-ELSE"
        rc.session_from_backup(ax, other)
        with self.assertRaises(rc.UnitReplacedError):
            ax.recover(timeout=1, policy=QUICK)
        with self.assertRaises(ValueError):
            rc.session_from_backup(ax, {"identity": {}})
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(rc.session_from_backup(ax, path).model_name, data["identity"]["model_name"])


# ====================================================================== command line


class SimFactory:
    def __init__(self, setup=None) -> None:  # type: ignore[no-untyped-def]
        self.instances: list[SimulatedEC] = []
        self.setup = setup

    def __call__(self, **kwargs: object) -> SimulatedEC:
        sim = SimulatedEC(**kwargs)  # type: ignore[arg-type]
        if self.setup is not None:
            self.setup(sim)
        self.instances.append(sim)
        return sim


def run(*argv: str, factory: SimFactory | None = None) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with mock.patch.object(cli, "SimulatedEC", factory or SimFactory()), contextlib.redirect_stderr(err):
        code = cli.main(list(argv), out=out)
    return code, out.getvalue(), err.getvalue()


class CliTest(unittest.TestCase):
    def test_move_and_jog_with_fast(self) -> None:
        code, out, err = run("--sim", "move", "30", "--fast")
        self.assertEqual(code, 0, err)
        position = re.search(r"position\s+(-?[\d.]+) mm", out)
        self.assertIsNotNone(position, out)
        self.assertAlmostEqual(float(position.group(1)), 30.0, delta=0.15)  # fast mode ends inside the in-position band
        self.assertIn("markers    not read (ZONS)", out)  # the final status is the monitor's lean sample
        code, out, err = run("--sim", "--json", "jog", "2", "--fast")
        self.assertEqual(code, 0, err)
        self.assertAlmostEqual(json.loads(out)["position_mm"], 75.95, delta=0.15)
        self.assertNotIn("--fast", build_help("home"))

    def test_recover_reports_and_never_moves(self) -> None:
        factory = SimFactory()
        code, out, err = run("--sim", "recover", "--target", "10", factory=factory)
        self.assertEqual(code, 0, err)
        self.assertIn("+63.95 from the last target", out)
        self.assertIn("needs attention: YES (nothing was moved)", out)
        self.assertIn("uptime", out)
        self.assertIn("PC control yes (kept)", out)
        self.assertAlmostEqual(factory.instances[0].position_mm, 73.95, delta=0.01)
        code, out, err = run("--sim", "--json", "recover")
        self.assertEqual(code, 0, err)
        data = json.loads(out)
        self.assertTrue(data["same_unit"])
        self.assertEqual((data["control_kept"], data["control_restored"]), (True, False))
        self.assertFalse(data["needs_attention"])
        self.assertIn("summary", data)

    def test_recover_against_a_backup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit.json"
            code, _, err = run("--sim", "backup", str(path))
            self.assertEqual(code, 0, err)
            code, out, err = run("--sim", "recover", "--backup", str(path))
            self.assertEqual(code, 0, err)
            self.assertIn("ready to move: yes", out)
            data = json.loads(path.read_text(encoding="utf-8"))
            data["identity"]["serial_number"] = "SOMEONE-ELSE"
            path.write_text(json.dumps(data), encoding="utf-8")
            code, _, err = run("--sim", "recover", "--backup", str(path))
            self.assertEqual(code, 2)
            self.assertIn("UnitReplacedError", err)
            code, _, err = run("--sim", "recover", "--backup", str(Path(tmp) / "missing.json"))
            self.assertEqual(code, 2)
            self.assertIn("missing.json", err)

    def test_recover_rows_used_compares_only_the_rows_the_unit_uses(self) -> None:
        def recover(path: Path, *extra: str) -> tuple[int, dict, str]:
            code, out, err = run("--sim", "--json", "recover", "--backup", str(path), *extra)
            return code, (json.loads(out) if code == 0 else {}), err

        def failing(report: dict) -> list[str]:
            return [problem.split(":")[0] for problem in report["row_problems"]]

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit.json"
            # rows 0-2 and 256-257 are the unit's own (a 3-position EC); 100 and 300 are not
            self.assertEqual(run("--sim", "backup", str(path), "--rows", "0,1,2,100,256,257,300")[0], 0)
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["identity"]["max_positions"], 3)
            for n in (1, 100):  # one row the unit uses and one it does not: both differ from the unit now
                next(r for r in data["positions"]["rows"] if r["n"] == n)["words"][1] = "0123"
            path.write_text(json.dumps(data), encoding="utf-8")
            for extra in ((), ("--rows", "all")):  # the default: every row the file holds
                code, report, err = recover(path, *extra)
                self.assertEqual(code, 0, err)
                self.assertEqual(sorted(report["rows_checked"]), [0, 1, 2, 100, 256, 257, 300])
                self.assertEqual(failing(report), ["row 1", "row 100"])
            code, report, err = recover(path, "--rows", "used")
            self.assertEqual(code, 0, err)
            self.assertEqual(sorted(report["rows_checked"]), [0, 1, 2, 256, 257])
            self.assertEqual(failing(report), ["row 1"])
            self.assertTrue(report["needs_attention"])
            data["identity"].pop("max_positions")  # a file that does not say which rows the unit uses: every row, with a note
            path.write_text(json.dumps(data), encoding="utf-8")
            code, report, err = recover(path, "--rows", "used")
            self.assertEqual(code, 0, err)
            self.assertIn("does not say how many rows the unit uses", err)
            self.assertEqual(len(report["rows_checked"]), 7)
        code, _, err = run("--sim", "recover", "--rows", "used")
        self.assertEqual(code, 2)
        self.assertIn("--rows needs --backup", err)
        self.assertIn("--rows", build_help("recover"))

    def test_monitor_watch_prints_changes_and_a_summary(self) -> None:
        code, out, err = run("--sim", "monitor", "--watch", "--duration", "0.3", "--interval", "0.05")
        self.assertEqual(code, 0, err)
        self.assertIn("SERVO HOMED IN-POS MODBUS", out)
        self.assertRegex(out, r"\d+ polls \(")
        code, out, err = run("--sim", "--json", "monitor", "--watch", "--duration", "0.3")
        self.assertEqual(code, 0, err)
        lines = [json.loads(line) for line in out.splitlines()]
        self.assertEqual(lines[0]["type"], "status")
        self.assertEqual(lines[-1]["type"], "summary")
        self.assertEqual(lines[-1]["link_state"], "ok")

    def test_monitor_watch_reports_a_lost_link_and_a_vanished_port(self) -> None:
        def later(sim: SimulatedEC) -> None:
            threading.Timer(0.6, sim.power_off).start()
            threading.Timer(1.2, sim.unplug).start()

        code, out, err = run("--sim", "monitor", "--watch", "--duration", "6", factory=SimFactory(later))
        self.assertEqual(code, 2, out + err)  # the port vanished: the watch ends early
        self.assertIn("LINK_LOST", out)
        self.assertIn("status monitor stopped", out)
        self.assertIn("link port_vanished", out)


def build_help(command: str) -> str:
    parser = cli.build_parser()
    sub = parser._subparsers._group_actions[0].choices[command]  # noqa: SLF001
    return sub.format_help()


if __name__ == "__main__":
    unittest.main()
