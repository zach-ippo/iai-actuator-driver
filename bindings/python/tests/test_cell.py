"""Port discovery, address scan, the Port / Cell plumbing and the port-level CLI commands (simulator only)."""

from __future__ import annotations

import io
import logging
import time
import unittest
from contextlib import redirect_stderr
from typing import Any
from unittest import mock

from iai_actuator_driver import cli, testing
from iai_actuator_driver.cell import (
    IAI_USB_PID,
    IAI_USB_VID,
    Cell,
    Port,
    _ranges,
    axis_name,
    list_ports,
    scan,
)
from iai_actuator_driver.errors import EcNotReadyError, ModbusTimeout
from iai_actuator_driver.modbus import SCAN_TIMEOUT_S, ModbusRTU
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_line, sim_ports


def _comport(device: str, vid: int | None, pid: int | None, description: str = "",
             manufacturer: str = "") -> dict[str, Any]:
    return {"port": device, "vid": vid, "pid": pid, "description": description, "manufacturer": manufacturer}


FAKE_PORTS = [
    _comport("COM1", None, None, "Communications Port (COM1)"),
    _comport("COM10", IAI_USB_VID, IAI_USB_PID, "IAI USB to UART Bridge Controller (COM10)", "IAI Corporation"),
    _comport("COM4", 0x0403, 0x6001, "USB Serial Port (COM4)", "FTDI"),
    _comport("COM3", IAI_USB_VID, IAI_USB_PID, "IAI USB to UART Bridge Controller (COM3)", "IAI Corporation"),
    _comport("COM7", 0x1234, 0x5678, "Some IAI adapter", "IAI Corporation"),
]


def _fc03_count(line: SimLine, slave: int) -> int:
    return sum(1 for _, f in line.requests if f[0] == slave and f[1] == 0x03)


class ListPortsTest(unittest.TestCase):
    def test_iai_adapters_first_in_natural_order(self) -> None:
        with testing.fake_ports(FAKE_PORTS):
            entries = list_ports()
        self.assertEqual([e["port"] for e in entries], ["COM3", "COM7", "COM10", "COM1", "COM4"])
        self.assertEqual([e["is_iai"] for e in entries], [True, True, True, False, False])
        com3 = entries[0]
        self.assertEqual((com3["vid"], com3["pid"]), (0x10C4, 0x81D7))
        self.assertIn("IAI USB to UART", com3["description"])
        self.assertEqual(set(com3), {"port", "description", "vid", "pid", "is_iai"})

    def test_no_ports(self) -> None:
        with testing.fake_ports([]):
            self.assertEqual(list_ports(), [])

    def test_the_core_lists_the_fake_ports_too(self) -> None:
        def core() -> list[str]:  # the core's enumeration: a recovery's port check, the cell's port discovery
            from iai_actuator_driver import native

            return [p["port"] for p in native._read_json(native.load(), "iad_list_ports_json")]

        before = core()
        with testing.fake_ports(FAKE_PORTS):
            names = [e["port"] for e in list_ports()]
            self.assertEqual(names, ["COM3", "COM7", "COM10", "COM1", "COM4"])
            self.assertEqual(core(), names)
        self.assertEqual(core(), before)  # the PC's own again

    def test_cell_default_ports_are_the_iai_adapters(self) -> None:
        with testing.fake_ports(FAKE_PORTS):
            specs = Cell._port_specs(None, None)
        self.assertEqual(list(specs), ["COM3", "COM7", "COM10"])

    def test_cell_without_any_adapter_raises_before_opening_anything(self) -> None:
        with testing.fake_ports(FAKE_PORTS[:1]), mock.patch("iai_actuator_driver.cell.Port") as port_cls:
            with self.assertRaises(EcNotReadyError):
                Cell.open()
        port_cls.assert_not_called()

    def test_slave_mapping_names_the_ports(self) -> None:
        specs = Cell._port_specs(None, {"COM5": [1], "COM6": [2, 3]})
        self.assertEqual(specs, {"COM5": "COM5", "COM6": "COM6"})


class ProbeTest(unittest.TestCase):
    def test_probe_answers_and_silence(self) -> None:
        line = sim_line([2])
        bus = ModbusRTU(line)
        self.assertTrue(bus.probe(2))
        t0 = time.perf_counter()
        self.assertFalse(bus.probe(3, timeout=0.05))
        self.assertGreaterEqual(time.perf_counter() - t0, 0.04)
        self.assertEqual(_fc03_count(line, 3), 1)  # no retry

    def test_probe_shortens_and_restores_a_settable_timeout(self) -> None:
        seen: list[float] = []

        class SlowPort:
            timeout = 0.2

            def write(self, data: bytes) -> None:
                seen.append(self.timeout)

            def read(self, n: int) -> bytes:
                return b""

            def reset_input_buffer(self) -> None:
                pass

            def close(self) -> None:
                pass

        port = SlowPort()
        bus = ModbusRTU(port)  # type: ignore[arg-type]
        self.assertFalse(bus.probe(1, timeout=0.03))
        self.assertEqual(seen, [0.03])
        self.assertEqual(port.timeout, 0.2)

    def test_detect_baudrate_over_several_slaves(self) -> None:
        line = SimLine([SimulatedEC(slave=4, baudrate=115200, ec_baudrate=38400)])
        bus = ModbusRTU(line)
        self.assertEqual(bus.detect_baudrate([1, 2, 3, 4]), 38400)
        self.assertEqual(line.baudrate, 38400)

    def test_detect_baudrate_nothing_answers(self) -> None:
        line = SimLine([SimulatedEC(slave=9, baudrate=115200)])
        bus = ModbusRTU(line)
        with self.assertRaises(ModbusTimeout) as cm:
            bus.detect_baudrate([1, 2], [115200, 38400], probe_timeout=0.01)
        self.assertIn("none of slaves 1, 2", str(cm.exception))
        self.assertEqual(line.baudrate, 115200)


class ScanTest(unittest.TestCase):
    def test_scan_16_addresses_with_missing_slaves_takes_about_a_second(self) -> None:
        # the bus, the axis and the units on one virtual clock: the time is what the scan waited, whatever the PC's load
        with testing.virtual_clock() as clock:
            line = sim_line([1, 5, 12], clock=clock.perf_counter)
            with Port(line, name="L") as port:
                t0 = clock.perf_counter()
                hits = port.scan()
                elapsed = clock.perf_counter() - t0
        self.assertEqual([h.slave for h in hits], [1, 5, 12])
        self.assertLess(elapsed, 1.5)
        self.assertGreaterEqual(elapsed, 13 * SCAN_TIMEOUT_S)  # each missing address waited SCAN_TIMEOUT_S
        for s in set(range(1, 17)) - {1, 5, 12}:
            self.assertEqual(_fc03_count(line, s), 1, f"slave {s} asked more than once")
        self.assertEqual([h.name for h in hits], ["L:1", "L:5", "L:12"])
        self.assertEqual(hits[1].axis_number, 4)
        self.assertEqual(hits[0].system_code, 0xEC)
        self.assertEqual(hits[0].family, "EC dust/drip-proof")
        self.assertIsNone(hits[0].error)

    def test_scan_is_read_only(self) -> None:
        line = sim_line([1, 2])
        scan(line, range(1, 5))
        self.assertLessEqual({f[1] for _, f in line.requests}, {0x03, 0x11})
        self.assertTrue(line.closed)  # a transport given to scan() is closed again

    def test_scan_identifies_a_late_fc11(self) -> None:
        line = SimLine([SimulatedEC(slave=3, profile="rod")])  # the rod answers FC11 after 0.3 s
        hits = scan(line, [1, 2, 3])
        self.assertEqual([h.slave for h in hits], [3])
        self.assertEqual(hits[0].firmware, "0002")
        self.assertEqual(hits[0].to_dict()["slave_id_raw"], "ec ff 00 00 02")

    def test_scan_detects_the_baud_rate_once(self) -> None:
        line = SimLine([SimulatedEC(slave=2, baudrate=115200, ec_baudrate=38400),
                        SimulatedEC(slave=3, baudrate=115200, ec_baudrate=38400)])  # fmt: skip
        with Port(line, name="L") as port:
            hits = port.scan(range(1, 5), identify=False, timeout=0.01)
            self.assertEqual([h.slave for h in hits], [2, 3])
            self.assertTrue(port.baud_detected)
            self.assertEqual(port.baudrate, 38400)
            sent = len(line.requests)
            port.ensure_baudrate()
            self.assertEqual(len(line.requests), sent)  # once per port

    def test_scan_empty_line(self) -> None:
        line = SimLine([], name="E")
        self.assertEqual(scan(line, range(1, 4)), [])

    def test_scan_rejects_bad_address(self) -> None:
        with self.assertRaises(ValueError):
            scan(sim_line([1]), [0, 1])


class SimLineTest(unittest.TestCase):
    def test_duplicate_address_refused(self) -> None:
        line = sim_line([1])
        with self.assertRaises(ValueError):
            line.add(SimulatedEC(slave=1))

    def test_broadcast_reaches_every_unit_and_nobody_answers(self) -> None:
        line = SimLine.with_slaves([1, 2, 3])
        bus = ModbusRTU(line)
        bus.broadcast_iai_command(0, 0x09, 0, settle_s=0)  # servo off to all (safe-list command)
        self.assertEqual([u.servo_on for u in line], [False, False, False])
        self.assertEqual(line.read(10), b"")

    def test_sim_ports_names(self) -> None:
        ports = sim_ports(3, slaves=[1, 2])
        self.assertEqual(list(ports), ["SIM1", "SIM2", "SIM3"])
        self.assertEqual(ports["SIM2"].slaves, [1, 2])
        self.assertEqual(ports["SIM2"].name, "SIM2")


class CellPlumbingTest(unittest.TestCase):
    def test_names_and_lookup(self) -> None:
        ports = sim_ports(2, slaves=[1, 3])
        with Cell.open(ports, identify=False, scan_timeout=0.01) as cell:
            self.assertEqual(cell.names, ("SIM1:1", "SIM1:3", "SIM2:1", "SIM2:3"))
            self.assertIs(cell["SIM2:3"], cell[3])
            self.assertIs(cell[-1], cell.axes[3])
            self.assertEqual(cell.name_of(cell[1]), "SIM1:3")
            self.assertIn("SIM1:1", cell)
            self.assertNotIn("SIM9:1", cell)
            self.assertEqual(cell.port_of("SIM2:1").name, "SIM2")
            self.assertIs(cell[0].bus, cell[1].bus)  # one bus per port, shared by its axes
            self.assertIsNot(cell[0].bus, cell[2].bus)
            with self.assertRaises(KeyError):
                cell["COM3:1"]
        self.assertTrue(all(line.closed for line in ports.values()))

    def test_identify_reads_each_units_own_ranges(self) -> None:
        line = SimLine([SimulatedEC(slave=1, profile="slider"), SimulatedEC(slave=2, profile="gripper")], name="L")
        with Cell.open([line], scan_timeout=0.01) as cell:
            self.assertEqual(cell.info["L:1"].soft_limits_mm, (-0.3, 150.3))
            self.assertEqual(cell.info["L:2"].soft_limits_mm, (-0.3, 9.8))
            self.assertEqual(cell["L:2"].soft_limits_mm, (-0.3, 9.8))

    def test_listed_slaves_must_answer(self) -> None:
        line = sim_line([1], name="L")
        with self.assertRaises(ModbusTimeout) as cm:
            Cell.open({"L": line}, slaves={"L": [1, 2]}, identify=False, scan_timeout=0.01)
        self.assertIn("slave(s) 2", str(cm.exception))
        self.assertTrue(line.closed)

    def test_listed_slaves_skip_the_scan(self) -> None:
        line = sim_line([1, 2, 3], name="L")
        with Cell.open([line], slaves=[2], identify=False, scan_timeout=0.01) as cell:
            self.assertEqual(cell.names, ("L:2",))
        self.assertEqual(_fc03_count(line, 9), 0)

    def test_max_axes(self) -> None:
        with self.assertRaises(ValueError):
            Cell.open([sim_line([1, 2, 3], name="L")], slaves=[1, 2, 3], max_axes=2, identify=False, scan_timeout=0.01)
        line = sim_line([1, 2, 3], name="M")
        with self.assertLogs("iai_actuator_driver.cell", logging.WARNING):
            cell = Cell.open([line], max_axes=2, identify=False, scan_timeout=0.01)
        with cell:
            self.assertEqual(cell.names, ("M:1", "M:2"))
        with self.assertRaises(ValueError):
            Cell.open([sim_line([1], name="Z")], max_axes=0)

    def test_no_axis_limit_by_default(self) -> None:
        nine = range(1, 10)
        lines = [sim_line(nine, name="A"), sim_line(nine, name="B")]  # 2 lines x 9 = 18 axes
        with Cell.open(lines, identify=False, scan_slaves=nine, scan_timeout=0.01) as cell:
            self.assertEqual(len(cell.names), 18)
        with Cell.open(sim_ports(17), slaves=[1], identify=False) as cell:  # 17 ports, one axis each
            self.assertEqual(len(cell.names), 17)
        lines = [sim_line(nine, name="C"), sim_line(nine, name="D")]
        with self.assertLogs("iai_actuator_driver.cell", logging.WARNING):
            cell = Cell.open(lines, max_axes=17, identify=False, scan_slaves=nine, scan_timeout=0.01)
        with cell:
            self.assertEqual(len(cell.names), 17)  # max_axes still caps

    def test_scan_skips_dead_ports(self) -> None:
        ports = {"A": sim_line([1], name="A"), "B": SimLine([], name="B")}
        with self.assertLogs("iai_actuator_driver.cell", logging.WARNING):
            cell = Cell.open(ports, identify=False, scan_slaves=range(1, 4))
        with cell:
            self.assertEqual(cell.names, ("A:1",))
            self.assertIn("B", cell.port_errors)
        self.assertTrue(ports["B"].closed)

    def test_nothing_found(self) -> None:
        with self.assertRaises(EcNotReadyError), self.assertLogs("iai_actuator_driver.cell", logging.WARNING):
            Cell.open([SimLine([], name="E")], scan_slaves=[1, 2])

    def test_helpers(self) -> None:
        self.assertEqual(axis_name("COM3", 1), "COM3:1")
        self.assertEqual(_ranges([1, 2, 3, 5, 7, 8]), "1-3,5,7-8")


class PortCliTest(unittest.TestCase):
    def run_cli(self, argv: list[str]) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with redirect_stderr(err):
            code = cli.main(argv, out=out)
        return code, out.getvalue(), err.getvalue()

    def test_ports(self) -> None:
        with testing.fake_ports(FAKE_PORTS):
            code, out, _ = self.run_cli(["ports"])
        self.assertEqual(code, 0)
        lines = out.splitlines()
        self.assertTrue(lines[1].startswith("COM3"))
        self.assertIn("10C4:81D7", lines[1])
        with testing.fake_ports(FAKE_PORTS):
            code, out, _ = self.run_cli(["--json", "ports"])
        self.assertEqual(code, 0)
        self.assertIn('"is_iai": true', out)

    def test_scan_sim(self) -> None:
        code, out, _ = self.run_cli(["scan", "--sim", "--sim-slaves", "1,3", "--slaves", "1-4"])
        self.assertEqual(code, 0)
        self.assertIn("sim: 2 axis/axes", out)
        self.assertIn("sim:3", out)
        self.assertIn("axis 2", out)
        code, out, _ = self.run_cli(["scan", "--sim", "--slaves", "2-3"])
        self.assertEqual(code, 1)
        self.assertIn("no axis answered", out)

    def test_scan_without_adapters(self) -> None:
        with testing.fake_ports(FAKE_PORTS[:1]):
            code, _, err = self.run_cli(["scan"])
        self.assertEqual(code, 2)
        self.assertIn("no IAI USB adapter", err)

    def test_axis_option_selects_the_slave(self) -> None:
        code, out, _ = self.run_cli(["--sim", "--axis", "2", "identify"])
        self.assertEqual(code, 0)
        self.assertIn("slave 3", out)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            cli.main(["--sim", "--axis", "1", "--slave", "2", "status"], out=io.StringIO())

    def test_port_command_detection(self) -> None:
        self.assertEqual(cli._port_command(["scan", "--ports", "COM3,COM4"]), "scan")
        self.assertEqual(cli._port_command(["--json", "ports"]), "ports")
        self.assertIsNone(cli._port_command(["--port", "COM3", "status"]))
        self.assertIsNone(cli._port_command(["--port", "scan", "status"]))  # "scan" is the --port value here
        self.assertEqual(cli._slave_list("1-3,7"), [1, 2, 3, 7])


if __name__ == "__main__":
    unittest.main()
