"""Several axes driven together: shared lines, parallel ports, stop_all, broadcast baud, 16-axis cells (simulator)."""

from __future__ import annotations

import threading
import time
import unittest


from iai_actuator_driver.cell import Cell, CellError, Port
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import EcAlarmError, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_line, sim_ports

FAST = {"poll_interval_s": 0.005}
OPEN = {**FAST, "scan_timeout": 0.01}  # quick scans: the simulator answers at once


def _switches(seq: list[int]) -> int:
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


class SharedLineTest(unittest.TestCase):
    def test_two_axes_on_one_line_from_two_threads(self) -> None:
        # a stepped clock: the units move only with the line's requests (each unit sees every frame on its line), so
        # the moves (about 55 mm each way from 73.95) last some 110 requests, not a time that a busy PC can overrun
        # before the second thread polls; both threads start their moves together (a barrier)
        line = sim_line([1, 2], name="L", step_s=0.005)
        bus = ModbusRTU(line)
        axes = [Elecylinder(bus, slave=s, **FAST) for s in (1, 2)]
        targets = {1: 20.0, 2: 130.0}
        results: dict[int, object] = {}
        together = threading.Barrier(2)

        def drive(ec: Elecylinder) -> None:
            try:
                together.wait(10)
                results[ec.slave] = ec.move_to(targets[ec.slave], speed_mm_s=100, accel_g=0.3)
            except Exception as exc:  # reported below
                results[ec.slave] = exc

        start = len(line.requests)
        threads = [threading.Thread(target=drive, args=(ec,)) for ec in axes]
        for t in threads:
            t.start()
        for t in threads:
            t.join(15)
        for s, target in targets.items():
            st = results[s]
            self.assertNotIsInstance(st, Exception, st)
            self.assertAlmostEqual(st.position_mm, target, delta=0.1)  # type: ignore[union-attr]
            self.assertAlmostEqual(line[s].position_mm, target, delta=0.1)
        seq = [a for a in line.request_slaves()[start:] if a in (1, 2)]
        self.assertGreater(_switches(seq), 10, "frames of the two axes did not interleave")

    def test_cell_on_one_line_moves_together(self) -> None:
        # a stepped clock: each unit's time advances by its own requests only, so "moving right after the start" holds
        # whatever the PC's load is (a unit on real time at time_scale 5 can finish within one scheduling delay); the
        # step is coarse, so a loaded PC needs few polls to see the arrival (the wait's timeout is real time)
        line = sim_line([1, 2, 3], name="L", step_s=0.05)
        with Cell.open([line], **OPEN) as cell:
            cell.move_all({0: 40.0, 1: 140.0}, speed_mm_s=100, accel_g=0.3, wait=False)
            self.assertTrue(line[1].moving and line[2].moving)
            self.assertFalse(line[3].moving)
            done = cell.wait_all(["L:1", "L:2"])
            self.assertAlmostEqual(done["L:1"].position_mm, 40.0, delta=0.1)
            self.assertAlmostEqual(done["L:2"].position_mm, 140.0, delta=0.1)
            st = cell.status_all()
            self.assertEqual(list(st), ["L:1", "L:2", "L:3"])


class MultiPortTest(unittest.TestCase):
    def test_parallel_moves_on_four_ports(self) -> None:
        # a stepped clock (see test_cell_on_one_line_moves_together): no axis can arrive before the check
        ports = sim_ports(4, step_s=0.05)
        with Cell.open(ports, **OPEN) as cell:
            targets = {name: 10.0 + 30 * i for i, name in enumerate(cell.names)}
            cell.move_all(targets, speed_mm_s=100, accel_g=0.3, wait=False)
            self.assertTrue(all(line[1].moving for line in ports.values()), "not all axes moving at once")
            done = cell.wait_all()
            for name, target in targets.items():
                self.assertAlmostEqual(done[name].position_mm, target, delta=0.1)

    def test_move_all_per_axis_speed_and_limits(self) -> None:
        ports = sim_ports(2, time_scale=20)
        with Cell.open(ports, **OPEN) as cell:
            res = cell.move_all({"SIM1:1": 30.0, "SIM2:1": 60.0}, speed_mm_s={"SIM1:1": 50.0, 1: 80.0})
            self.assertAlmostEqual(res["SIM2:1"].position_mm, 60.0, delta=0.1)
            with self.assertRaises(CellError) as cm:
                cell.move_all({0: 500.0, 1: 20.0})  # 500 mm is outside the slider's own soft limits
            self.assertIsInstance(cm.exception.errors["SIM1:1"], SoftLimitError)
            self.assertNotIn("SIM2:1", cm.exception.errors)
            self.assertFalse(ports["SIM2"][1].moving)  # the started axis was stopped (stop_others_on_error)

    def test_alarm_on_one_axis_stops_the_others(self) -> None:
        ports = sim_ports(3, time_scale=2)
        with Cell.open(ports, **OPEN) as cell:
            threading.Timer(0.15, lambda: ports["SIM2"][1].trigger_alarm(0x0C0)).start()
            with self.assertRaises(CellError) as cm:
                cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=60, accel_g=0.3)
            self.assertIsInstance(cm.exception.errors["SIM2:1"], EcAlarmError)
            for name in ("SIM1", "SIM3"):
                self.assertFalse(ports[name][1].moving)
                self.assertLess(ports[name][1].position_mm, 139.0)

    def test_home_and_servo_all(self) -> None:
        ports = sim_ports(3, time_scale=50)
        with Cell.open(ports, **OPEN) as cell:
            cell.servo_all(False)
            self.assertFalse(any(line[1].servo_on for line in ports.values()))
            cell.servo_all(True)
            homed = cell.home_all()
            for name, st in homed.items():
                self.assertTrue(st.homed, name)
                self.assertAlmostEqual(st.position_mm, 0.0, delta=0.05)

    def test_leaving_the_cell_on_an_exception_stops_every_axis(self) -> None:
        ports = sim_ports(2, time_scale=1)
        with self.assertRaises(RuntimeError) as cm:
            with Cell.open(ports, **OPEN) as cell:
                cell.move_all({0: 140.0, 1: 140.0}, speed_mm_s=50, wait=False)
                raise RuntimeError("application error")
        self.assertTrue(any("stop_all" in n for n in getattr(cm.exception, "__notes__", [])))
        for line in ports.values():
            self.assertFalse(line[1].moving)
            self.assertFalse(line[1].stop_held)


class StopAllTest(unittest.TestCase):
    def test_stop_all_on_six_axes_sends_every_stop_first(self) -> None:
        ports = sim_ports(2, slaves=[1, 2, 3], time_scale=1)
        with Cell.open(ports, **OPEN) as cell:
            self.assertEqual(len(cell), 6)
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=50, wait=False)
            time.sleep(0.05)
            marks = {name: len(line.requests) for name, line in ports.items()}
            t0 = time.perf_counter()
            result = cell.stop_all()
            self.assertLess(time.perf_counter() - t0, 3.0)
            for name, line in ports.items():
                after = [f for _, f in line.requests[marks[name]:]]
                stop_on = [f[0] for f in after[:3]]
                self.assertEqual(sorted(stop_on), [1, 2, 3], f"{name}: STOP not first to every axis")
                for f in after[:3]:
                    self.assertEqual(f[1:6], bytes([0x05, 0x04, 0x2C, 0xFF, 0x00]))
                for unit in line:
                    self.assertFalse(unit.moving)
                    self.assertFalse(unit.stop_held)  # released after the stop
                    self.assertLess(unit.position_mm, 139.0)
            self.assertEqual(len(result), 6)

    def test_stop_all_attempts_every_axis_and_reports_failures(self) -> None:
        ports = sim_ports(2, slaves=[1, 2], time_scale=20)
        with Cell.open(ports, **OPEN) as cell:
            ports["SIM1"].remove(2)  # cable pulled: this axis no longer answers
            res = cell.stop_all(return_exceptions=True)
            self.assertIsInstance(res["SIM1:2"], Exception)
            for name in ("SIM1:1", "SIM2:1", "SIM2:2"):
                self.assertNotIsInstance(res[name], Exception)
            with self.assertRaises(CellError) as cm:
                cell.stop_all()
            self.assertEqual(list(cm.exception.errors), ["SIM1:2"])

    def test_stop_all_and_pause_all_with_an_unknown_name_act_on_the_axes_they_have(self) -> None:
        # a remembered list naming an axis that dropped out of a reopened cell: the STOP still goes out to the others
        ports = sim_ports(2, slaves=[1], time_scale=20)
        with Cell.open(ports, **OPEN) as cell:
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=2, wait=False)
            with self.assertRaises(CellError) as cm:
                cell.stop_all(["SIM1:1", "NOSUCH:9"])
            self.assertEqual(list(cm.exception.errors), ["NOSUCH:9"])
            self.assertIsInstance(cm.exception.errors["NOSUCH:9"], KeyError)
            self.assertFalse(ports["SIM1"][1].moving)  # stopped although the list named an unknown axis
            self.assertTrue(ports["SIM2"][1].moving)  # not listed
            out = cell.pause_all(["NOSUCH:9", "SIM2:1"], return_exceptions=True)
            self.assertEqual(list(out), ["SIM2:1", "NOSUCH:9"])
            self.assertIsNone(out["SIM2:1"])
            self.assertTrue(ports["SIM2"][1].paused)
            cell.resume_all(["SIM2:1"])
            self.assertEqual(list(cell.stop_all(["NOSUCH:9"], return_exceptions=True)), ["NOSUCH:9"])
            self.assertTrue(ports["SIM2"][1].moving)  # only unknown names: nothing stopped, never "every axis"
            self.assertEqual(len(cell.stop_all()), 2)


class BroadcastBaudTest(unittest.TestCase):
    def make_line(self) -> SimLine:
        return SimLine([SimulatedEC(slave=s, baudrate=115200) for s in (1, 2, 3)], name="L")

    def test_baud_change_on_a_shared_line_switches_every_unit(self) -> None:
        line = self.make_line()
        with Cell.open([line], **OPEN) as cell:
            port = cell.ports["L"]
            result = port.set_baudrate(230400)
            self.assertEqual(result.answered, {1: True, 2: True, 3: True})
            self.assertTrue(result.all_answered)
            self.assertEqual(port.baudrate, 230400)
            self.assertEqual([u.ec_baudrate for u in line], [230400, 230400, 230400])
            self.assertEqual(line.request_slaves().count(0), 1)  # one broadcast frame
            st = cell.status_all()
            self.assertEqual(len(st), 3)

    def test_unverified_rate_is_sent_with_a_warning(self) -> None:
        line = self.make_line()
        with Port(line, name="L") as port:
            port.scan(identify=False)
            for s in (1, 2, 3):
                port.axis(s)
            with self.assertRaisesRegex(SoftLimitError, "no IAI code"):
                port.set_baudrate(12345, allow_unverified_code=True)
            self.assertNotIn(0, line.request_slaves())
            self.assertEqual([u.ec_baudrate for u in line], [115200] * 3)
            with self.assertLogs("iai_actuator_driver", "WARNING") as logs:
                result = port.set_baudrate(57600)  # allow_unverified_code: no longer needed
            self.assertIn("its code is UNVERIFIED", "\n".join(logs.output))
            self.assertTrue(result.all_answered)
            self.assertEqual([u.ec_baudrate for u in line], [57600] * 3)

    def test_refused_while_an_axis_moves(self) -> None:
        line = self.make_line()
        for unit in line:
            unit.time_scale = 1.0
        with Cell.open([line], **OPEN) as cell:
            cell.move_all({"L:2": 140.0}, speed_mm_s=20, wait=False)
            with self.assertRaises(Exception):
                cell.ports["L"].set_baudrate(230400)
            self.assertNotIn(0, line.request_slaves())
            cell.stop_all()


class SixteenAxisTest(unittest.TestCase):
    def test_sixteen_ports_one_axis_each(self) -> None:
        ports = sim_ports(16, time_scale=10)
        with Cell.open(ports, **OPEN) as cell:
            self.assertEqual(len(cell), 16)
            self.assertEqual(cell.names[0], "SIM1:1")
            self.assertEqual(cell.names[15], "SIM16:1")
            targets = {name: 5.0 + 8 * i for i, name in enumerate(cell.names)}
            done = cell.move_all(targets, speed_mm_s=100, accel_g=0.3)
            for name, target in targets.items():
                self.assertAlmostEqual(done[name].position_mm, target, delta=0.1)
            self.assertEqual(len(cell.stop_all()), 16)

    def test_sixteen_axes_on_one_line(self) -> None:
        line = sim_line(range(1, 17), name="RCON", time_scale=10)
        with Cell.open([line], **OPEN) as cell:
            self.assertEqual(cell.names, tuple(f"RCON:{s}" for s in range(1, 17)))
            targets = {i: 10.0 + 7 * i for i in range(16)}
            done = cell.move_all(targets, speed_mm_s=100, accel_g=0.3)
            for i, target in targets.items():
                self.assertAlmostEqual(done[cell.names[i]].position_mm, target, delta=0.1)
                self.assertAlmostEqual(line[i + 1].position_mm, target, delta=0.1)

    def test_mixed_ports_take_every_axis_unless_capped(self) -> None:
        listed = {"A": range(1, 9), "B": range(1, 9), "C": [1]}
        ports = {"A": sim_line(range(1, 9), name="A"), "B": sim_line(range(1, 9), name="B"),
                 "C": sim_line([1], name="C")}  # fmt: skip
        with Cell.open(ports, slaves=listed, identify=False) as cell:  # no limit by default: 17 axes
            self.assertEqual(len(cell), 17)
        ports = {"A": sim_line(range(1, 9), name="A"), "B": sim_line(range(1, 9), name="B"),
                 "C": sim_line([1], name="C")}  # fmt: skip
        with self.assertRaises(ValueError):
            Cell.open(ports, slaves=listed, max_axes=16, identify=False)
        ports = {"A": sim_line(range(1, 9), name="A"), "B": sim_line(range(1, 9), name="B")}
        with Cell.open(ports, identify=False, scan_timeout=0.01) as cell:
            self.assertEqual(len(cell), 16)
            self.assertEqual(len(cell.status_all()), 16)


if __name__ == "__main__":
    unittest.main()
