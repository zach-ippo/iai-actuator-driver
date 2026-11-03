"""Gateway lines (RCON, ROBONET): detection, line rules and bus guards, MANU, position unit, gateway monitor, identity.

Simulator only (``sim_line.SimGatewayLine``): every gateway rule is DOC (ME0413, ME0208), UNVERIFIED on hardware.
"""

from __future__ import annotations

import json
import time
import unittest

from iai_actuator_driver import registers as R
from iai_actuator_driver.cell import Cell, Port
from iai_actuator_driver.errors import EcNotReadyError, ModbusException, ModbusTimeout
from iai_actuator_driver.gateway import (
    LINE_DIRECT,
    LINE_KINDS,
    LINE_RCON,
    LINE_ROBONET,
    LINE_ROBONET_GATEWAY,
    OP_AXIS_CONTROL,
    OP_BAUD_CHANGE,
    OP_BROADCAST,
    OP_COIL,
    OP_IAI_COMMAND,
    OP_REPORT_SLAVE_ID,
    OP_SOFTWARE_RESET,
    OPERATIONS,
    GatewayModeError,
    Line,
    LineRuleError,
    PositionUnitError,
    RconAxisMonitor,
    RconGateway,
    RoboNetGatewayStatus,
    UnsupportedLineError,
    detect_line,
    read_serial_code,
    robonet_hint,
)
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimGatewayLine, SimLine, sim_line, sim_rcon_line, sim_robonet_line

from ec_testutil import hexbytes

FAST = {"poll_interval_s": 0.005}
OPEN = {**FAST, "scan_timeout": 0.01, "identify": False}


def frames(line: SimLine) -> list[bytes]:
    return [f for _, f in line.requests]


def rcon_bus(line: SimGatewayLine) -> ModbusRTU:
    bus = ModbusRTU(line, min_gap_s=0.0)
    detect_line(bus)
    return bus


class FrameTest(unittest.TestCase):
    """The read-only gateway frames, CRC included."""

    def test_probe_and_monitor_frames(self) -> None:
        cases = {
            (0xD0, "03 37 00 00 0B"): "D0 03 37 00 00 0B 18 38",
            (0x3F, "03 F7 00 00 02"): "3F 03 F7 00 00 02 F2 A1",
            (0xD0, "03 37 08 00 02"): "D0 03 37 08 00 02 59 FC",
            (0xD0, "03 29 00 00 78"): "D0 03 29 00 00 78 5F F5",
            (0xD0, "03 29 78 00 78"): "D0 03 29 78 00 78 DF EC",
            (0x01, "03 94 00 00 08"): "01 03 94 00 00 08 68 3C",
        }
        for (slave, pdu), expected in cases.items():
            with self.subTest(expected=expected):
                self.assertEqual(build_frame(slave, hexbytes(pdu)), hexbytes(expected))

    def test_16_axes_monitor_is_two_frames(self) -> None:
        line = sim_rcon_line(range(1, 17), time_scale=20)
        bus = rcon_bus(line)
        mark = len(line.requests)
        mons = bus.line.gateway.read_monitor()
        sent = frames(line)[mark:]
        self.assertEqual(sent, [hexbytes("D0 03 29 00 00 78 5F F5"), hexbytes("D0 03 29 78 00 78 DF EC")])
        self.assertEqual(sorted(mons), list(range(16)))
        self.assertTrue(all(isinstance(m, RconAxisMonitor) for m in mons.values()))


class DetectionTest(unittest.TestCase):
    def test_direct_line_costs_two_short_read_only_probes(self) -> None:
        line = sim_line([1], name="D")
        with Port(line, name="D") as port:
            started = time.perf_counter()
            found = port.detect_line()
            elapsed = time.perf_counter() - started
            self.assertEqual(found.kind, LINE_DIRECT)
            self.assertFalse(found.detected)
            self.assertLess(elapsed, 0.3)
            sent = frames(line)
            self.assertEqual(sent, [hexbytes("D0 03 37 00 00 0B 18 38"), hexbytes("3F 03 F7 00 00 02 F2 A1")])
            port.scan([1], identify=False)
            self.assertEqual(len([f for f in frames(line) if f[0] in (0xD0, 0x3F)]), 2, "probed twice")

    def test_stated_direct_line_sends_no_probe(self) -> None:
        line = sim_line([1], name="D")
        with Port(line, name="D", line="direct") as port:
            port.scan([1], identify=False)
            self.assertEqual(port.line.kind, LINE_DIRECT)
        self.assertFalse([f for f in frames(line) if f[0] in (0xD0, 0x3F)])

    def test_unit_at_0x3f_answering_an_exception_is_not_a_gateway(self) -> None:
        line = SimLine([SimulatedEC(slave=1), SimulatedEC(slave=0x3F)], name="D")  # 0xF700 is no EC register
        with Port(line, name="D") as port:
            self.assertEqual(port.detect_line().kind, LINE_DIRECT)

    def test_rcon_detected(self) -> None:
        line = sim_rcon_line([1, 2, 3], time_scale=20)
        with Port(line, name="R") as port:
            hits = port.scan(identify=False)
            self.assertEqual([h.slave for h in hits], [1, 2, 3])
            self.assertEqual(port.line.kind, LINE_RCON)
            self.assertTrue(port.line.detected)
            self.assertIsInstance(port.gateway, RconGateway)
            st = port.gateway.last_status
            self.assertTrue(st.manu)
            self.assertEqual(st.linked_slaves, (1, 2, 3))
            self.assertEqual(port.line.command_strategy, "register")
            self.assertIs(port.bus.line, port.line)

    def test_robonet_gateway_mode_gives_no_axis(self) -> None:
        line = sim_robonet_line([1, 2], gateway_mode=True, time_scale=20)
        with Port(line, name="G") as port:
            self.assertEqual(port.scan(), [])
            self.assertEqual(port.line.kind, LINE_ROBONET_GATEWAY)
            st = port.gateway_status()
            self.assertIsInstance(st, RoboNetGatewayStatus)
            self.assertTrue(st.running)
            self.assertEqual(st.linked_axes, (0, 1))
            with self.assertRaises(UnsupportedLineError) as cm:
                port.axis(1)
            self.assertIn("SW2 ON", str(cm.exception))
            with self.assertRaises(UnsupportedLineError):
                port.line.require_control(1)
        self.assertFalse([f for f in frames(line) if f[1] != 0x03], "a write reached a gateway-mode line")
        with self.assertRaises(EcNotReadyError) as cm2:
            Cell.open([sim_robonet_line([1], gateway_mode=True, name="G2")], **OPEN)
        self.assertIn("SW2", str(cm2.exception))

    def test_robonet_gateway_status_decoding(self) -> None:
        st = RoboNetGatewayStatus((0x8021, 0x0003))  # the ME0208 p.128-129 example reply
        self.assertTrue(st.running)
        self.assertEqual((st.positioner_axes, st.direct_axes, st.linked_axes), (1, 1, (0, 1)))

    def test_stated_rcon_without_gateway_refuses_control(self) -> None:
        line = sim_line([1], name="X")
        with Port(line, name="X", line="rcon", position_unit_mm=0.01) as port:
            self.assertEqual(port.detect_line().kind, LINE_RCON)
            self.assertIsNone(port.gateway)
            with self.assertRaises(GatewayModeError):
                port.line.require_motion(1)


class LineRulesTest(unittest.TestCase):
    def test_operations_per_kind(self) -> None:
        expect = {
            LINE_DIRECT: {op: True for op in OPERATIONS},
            LINE_RCON: {OP_COIL: False, OP_IAI_COMMAND: False, OP_BAUD_CHANGE: False, OP_REPORT_SLAVE_ID: False,
                        OP_AXIS_CONTROL: True},
            LINE_ROBONET: {OP_COIL: True, OP_IAI_COMMAND: False, OP_BAUD_CHANGE: False, OP_REPORT_SLAVE_ID: True,
                           OP_SOFTWARE_RESET: False},
            LINE_ROBONET_GATEWAY: {OP_AXIS_CONTROL: False, OP_COIL: False},
        }  # fmt: skip
        for kind, ops in expect.items():
            line = Line(kind)
            for op, allowed in ops.items():
                with self.subTest(kind=kind, op=op):
                    self.assertEqual(line.allows(op), allowed)
                    if not allowed:
                        with self.assertRaises(LineRuleError):
                            line.require(op)
        with self.assertRaises(ValueError):
            Line(LINE_DIRECT).allows("teleport")
        with self.assertRaises(ValueError):
            Line("canbus")

    def test_line_specific_messages(self) -> None:
        cases = {
            (LINE_RCON, OP_SOFTWARE_RESET): "ME0413 p.3-1",
            (LINE_RCON, OP_BAUD_CHANGE): "ME0413 p.6-1 #2",
            (LINE_ROBONET, OP_BAUD_CHANGE): "230.4 kbps",
            (LINE_ROBONET, OP_IAI_COMMAND): "ME0208 p.26",
            (LINE_ROBONET_GATEWAY, OP_COIL): "SW2 ON",
        }
        for (kind, op), text in cases.items():
            with self.subTest(kind=kind, op=op):
                with self.assertRaises(LineRuleError) as cm:
                    Line(kind).require(op)
                self.assertIn(text, str(cm.exception))
                self.assertIn("nothing sent", str(cm.exception))

    def test_to_dict_is_json(self) -> None:
        for kind in LINE_KINDS:
            json.dumps(Line(kind).to_dict())

    def test_one_line_rule_error_class(self) -> None:
        from iai_actuator_driver import errors

        self.assertIs(LineRuleError, errors.LineRuleError)
        self.assertTrue(issubclass(LineRuleError, errors.UnsafeCommandError))

    def test_axis_class_reads_the_line(self) -> None:
        """The axis class duck-types ``bus.line``: on RCON it sees no coils / IAI commands and the 'register' path."""
        line = sim_rcon_line([1], time_scale=20)
        with Port(line, name="R", position_unit_mm=0.01) as port:
            ax = port.axis(1)
            self.assertFalse(ax.bus.line.allows(OP_COIL))
            self.assertFalse(ax.bus.line.allows(OP_IAI_COMMAND))
            self.assertFalse(ax.bus.line.allows(OP_BROADCAST))
            self.assertEqual(ax.line_name, LINE_RCON)
            self.assertEqual(ax.stop_path, "register")
        with Port(sim_line([1], name="D"), name="D") as port:
            ax = port.axis(1)
            line = ax.bus.line
            self.assertTrue(line.allows(OP_COIL) and line.allows(OP_IAI_COMMAND) and line.allows(OP_BROADCAST))
            self.assertIsNone(ax.line_name)


class RconGuardTest(unittest.TestCase):
    """On an RCON line the bus refuses FC05, 0x45, 0x48, broadcasts and writes to the gateway: nothing is sent."""

    def setUp(self) -> None:
        self.line = sim_rcon_line([1, 2], time_scale=20)
        self.bus = rcon_bus(self.line)
        self.mark = len(self.line.requests)

    def sent(self) -> list[bytes]:
        return frames(self.line)[self.mark:]

    def test_refused_requests_send_nothing(self) -> None:
        calls = {
            "FC05 coil": lambda: self.bus.write_coil(1, R.COIL_STOP, True),
            "0x45 servo": lambda: self.bus.iai_command(1, 0, 0x09, 1),
            "0x45 reset": lambda: self.bus.iai_command(1, 0, 0x05, 1, allow_unsafe=True),
            "0x48": lambda: self.bus.iai_command2(1, 0x30, [1]),
            "write to the gateway": lambda: self.bus.write_register(0xD0, 0x3700, 0),
            "broadcast": lambda: self.bus.broadcast_coil(R.COIL_STOP, True),
        }
        for name, call in calls.items():
            with self.subTest(name), self.assertRaises(LineRuleError):
                call()
        self.assertEqual(self.sent(), [])

    def test_documented_writes_pass(self) -> None:
        self.bus.write_register(1, R.DRG1, 0x1000)  # FC06 DRG1 servo on (ME0413 p.5-11)
        self.assertEqual(self.sent()[-1], hexbytes("01 06 0D 00 10 00 86 A6"))
        self.assertEqual(len(self.bus.read_registers(1, 0x9000, 8)), 8)

    def test_fc11_read_passes_the_guard_and_the_sim_rejects_it(self) -> None:
        with self.assertRaises(ModbusException) as cm:
            self.bus.report_slave_id(1, retries=0, timeout=None)
        self.assertEqual(cm.exception.code, 0x01)
        self.assertIn(0x11, self.bus.unsupported_functions(1))


class ManuTest(unittest.TestCase):
    def test_auto_refuses_then_manu_allows(self) -> None:
        line = sim_rcon_line([1], manu=False, time_scale=20)
        with Port(line, name="R", position_unit_mm=0.01) as port:
            port.scan(identify=False)
            with self.assertRaises(GatewayModeError) as cm:
                port.line.require_control(1)
            self.assertIn("MANU", str(cm.exception))
            self.assertIn("ME0413 p.6-1 #5", str(cm.exception))
            line.gateway.manu = True  # the switch is turned: a refusal re-reads, so the next call passes
            self.assertEqual(port.line.require_motion(1), 0.01)

    def test_manu_read_is_cached(self) -> None:
        line = sim_rcon_line([1], time_scale=20)
        with Port(line, name="R", position_unit_mm=0.01) as port:
            port.scan(identify=False)
            port.line.require_control(1)
            n = len(line.gateway.requests)
            for _ in range(5):
                port.line.require_control(1)
            self.assertEqual(len(line.gateway.requests), n, "GWSR read again inside the cache window")
            port.gateway.cache_s = 0.0
            port.line.require_control(1)
            self.assertEqual(len(line.gateway.requests), n + 1)

    def test_gateway_alarm_and_emergency_stop(self) -> None:
        line = sim_rcon_line([1], gateway_alarm=0x8BA, emergency_stop=True)
        bus = rcon_bus(line)
        st = bus.line.gateway.read_status()
        self.assertEqual(st.alarm_code, 0x8BA)
        self.assertIn("Operation mode", st.alarm_name or "")
        self.assertTrue(st.emergency_stop)
        self.assertEqual(bus.line.gateway.read_alarm(), 0x8BA)


class PositionUnitTest(unittest.TestCase):
    def test_refused_until_stated(self) -> None:
        line = sim_rcon_line([1, 2], time_scale=20)
        with Port(line, name="R") as port:
            port.scan(identify=False)
            self.assertIsNone(port.line.position_unit_mm(1))
            with self.assertRaises(PositionUnitError) as cm:
                port.line.require_motion(1)
            self.assertIn("No.207", str(cm.exception))
            port.set_position_unit(0.001, slaves=[1])
            self.assertEqual(port.line.require_motion(1), 0.001)
            self.assertEqual(port.line.position_unit_source(1), "stated")
            with self.assertRaises(PositionUnitError):
                port.line.require_position_unit(2)
            port.set_position_unit(0.01)
            self.assertEqual(port.line.require_position_unit(2), 0.01)
            self.assertEqual(port.line.require_position_unit(1), 0.001)  # the per-axis statement wins
            with self.assertRaises(ValueError):
                port.set_position_unit(0.1)

    def test_direct_line_default_unit(self) -> None:
        line = Line(LINE_DIRECT)
        self.assertEqual(line.require_motion(1), R.POSITION_SCALE)
        self.assertEqual(line.position_unit_source(1), "default")

    def test_sim_0_001_unit_on_the_wire(self) -> None:
        line = sim_rcon_line([1], position_unit_mm=0.001, time_scale=50)
        bus = rcon_bus(line)
        unit = line[1]
        pnow = R.from_i32_words(*bus.read_registers(1, R.PNOW, 2))
        self.assertEqual(pnow, round(unit.position_mm * 1000))
        target = 45.678
        words = [*R.to_i32_words(round(target * 1000)), *R.to_u32_words(100), *R.to_u32_words(2000), 10, 0, 0]
        bus.write_registers(1, R.DIRECT_MOVE_BASE, words)
        deadline = time.perf_counter() + 5.0
        while unit.moving and time.perf_counter() < deadline:
            time.sleep(0.01)
        self.assertAlmostEqual(unit.position_mm, 45.68, places=2)  # the sim counts 0.01 mm
        mon = bus.line.gateway.read_monitor([0])[0]
        self.assertEqual(mon.position_counts, round(unit.position_mm * 1000))


class StatusViaGatewayTest(unittest.TestCase):
    def test_status_all_reads_the_monitor_in_one_frame(self) -> None:
        line = sim_rcon_line([1, 2, 3], position_unit_mm=0.001, time_scale=20)
        with Cell.open([line], position_unit_mm=0.001, **OPEN) as cell:
            mark = len(line.requests)
            st = cell.status_all()
            sent = frames(line)[mark:]
            self.assertEqual(len(sent), 1, [f.hex(" ") for f in sent])
            self.assertEqual(sent[0][:6], hexbytes("D0 03 29 00 00 2D"))  # axes 0-2: 45 words
            for s in (1, 2, 3):
                status = st[f"RCON:{s}"]
                self.assertEqual(status.raw["GATEWAY_MONITOR"], 1)
                self.assertAlmostEqual(status.position_mm, line[s].position_mm, places=2)
                self.assertTrue(status.servo_on)
            mark = len(line.requests)
            cell.status_all(via_gateway=False)
            self.assertFalse([f for f in frames(line)[mark:] if f[0] == 0xD0])

    def test_falls_back_to_per_axis_reads(self) -> None:
        line = sim_rcon_line([1, 2], time_scale=20)
        with Cell.open([line], position_unit_mm=0.01, **OPEN) as cell:
            line.gateway.slave = 0xD7  # the gateway stops answering at D0h
            st = cell.status_all()
            self.assertNotIn("GATEWAY_MONITOR", st["RCON:1"].raw)

    def test_gateway_objects_on_the_cell(self) -> None:
        with Cell.open([sim_rcon_line([1], time_scale=20), sim_line([1], name="D")], **OPEN) as cell:
            self.assertEqual(cell.lines, {"RCON": LINE_RCON, "D": LINE_DIRECT})
            self.assertEqual(list(cell.gateways), ["RCON"])
            self.assertTrue(cell.gateway_status_all()["RCON"].manu)


class IdentityTest(unittest.TestCase):
    def test_scan_uses_the_serial_code_not_fc11(self) -> None:
        line = sim_rcon_line([1, 2], serials={2: "RCON-PC-0042"}, time_scale=20)
        with Port(line, name="R") as port:
            hits = port.scan()
        self.assertEqual([h.serial_code for h in hits], ["SIMRCON-00", "RCON-PC-0042"])
        self.assertEqual([h.product_family for h in hits], ["RCON", "RCON"])
        self.assertEqual(hits[0].to_dict()["line"], LINE_RCON)
        self.assertNotIn(0x11, [f[1] for f in frames(line)])

    def test_read_serial_code(self) -> None:
        bus = rcon_bus(sim_rcon_line([3], serials={3: "AB"}))
        self.assertEqual(read_serial_code(bus, 3), "AB")
        self.assertIsNone(read_serial_code(ModbusRTU(SimulatedEC(), min_gap_s=0.0), 1))


class RconEcTest(unittest.TestCase):
    """ECs behind an EC connection unit: linked at the gateway, silent on Modbus (ME0413 p.4-1): explained."""

    def test_linked_silent_axes_are_explained(self) -> None:
        line = sim_rcon_line([1], linked_only=[5], time_scale=20)
        with Port(line, name="R") as port:
            hits = port.scan(identify=False)
            self.assertEqual([h.slave for h in hits], [1])
            self.assertEqual(port.line.unreachable_slaves, (5,))
            why = port.line.explain()
            self.assertIn("ME0413 p.4-1", why)
            self.assertIn("ME0394 p.4-56", why)
        probed = {f[0] for f in frames(line) if f[1] == 0x03 and f[2:4] == b"\x90\x00"}
        self.assertEqual(probed, {1, 5}, "the scan probed unlinked addresses")

    def test_cell_open_with_listed_ec_axis(self) -> None:
        line = sim_rcon_line([1], linked_only=[2], time_scale=20)
        with self.assertRaises(ModbusTimeout) as cm:
            Cell.open([line], slaves=[1, 2], **OPEN)
        notes = " ".join(getattr(cm.exception, "__notes__", []))
        self.assertIn("RCON-EC", notes)


class SimGatewayLineTest(unittest.TestCase):
    """What the gateway simulators answer when the bus does not guard (a 'direct' line bus)."""

    def test_rcon_units_reject_undocumented_functions(self) -> None:
        line = sim_rcon_line([1], time_scale=20)
        bus = ModbusRTU(line, min_gap_s=0.0)  # no detection: the bus guard does not know the gateway
        with self.assertRaises(ModbusException) as cm:
            bus.write_coil(1, R.COIL_STOP, True)
        self.assertEqual(cm.exception.code, 0x01)
        self.assertEqual(len(line.rejected), 1)
        self.assertFalse(line[1].stop_held)
        bus.broadcast_coil(R.COIL_STOP, True, settle_s=0.0)
        self.assertEqual(len(line.dropped_broadcasts), 1)
        self.assertFalse(line[1].stop_held)

    def test_robonet_through_mode_model(self) -> None:
        line = sim_robonet_line([1], time_scale=20)
        bus = ModbusRTU(line, min_gap_s=0.0, retries=0)
        with self.assertRaises(ModbusException):
            bus.iai_command(1, 0, 0x09, 1)
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, R.INDEX_HEADER, R.INDEX_HEADER_WORDS)  # no directory
        self.assertEqual(len(bus.read_registers(1, 0x9000, 2)), 2)
        bus.write_coil(1, R.COIL_SON, True)  # coils are carried
        hint = robonet_hint(bus, 1, directory_timed_out=True)
        self.assertIn("line='robonet'", hint or "")
        self.assertIsNone(robonet_hint(bus, 1, directory_timed_out=False))

    def test_robonet_stated_line_rules(self) -> None:
        line = sim_robonet_line([1, 2], time_scale=20)
        with Port(line, name="N", line="robonet") as port:
            hits = port.scan(identify=False)
            self.assertEqual([h.slave for h in hits], [1, 2])
            self.assertFalse([f for f in frames(line) if f[0] in (0xD0, 0x3F)], "through mode was probed")
            bus = port.bus
            with self.assertRaises(LineRuleError):
                bus.iai_command(1, 0, 0x09, 1)
            with self.assertRaises(LineRuleError) as cm:
                bus.write_registers(1, 0x1000, [0] * 76)
            self.assertIn("160-byte", str(cm.exception))
            with self.assertRaises(LineRuleError):
                port.set_baudrate(115200)
            bus.write_coil(1, R.COIL_SON, True)
            self.assertTrue(port.line.rules.directory_timeout_means_absent)
            self.assertEqual(port.line.command_strategy, "coil")

    def test_robonet_baud_detection_skips_slow_rates(self) -> None:
        unit = SimulatedEC(slave=1, baudrate=115200, ec_baudrate=9600)
        line = SimLine([unit], name="N")
        with Port(line, name="N", line="robonet") as port:
            self.assertEqual(port.scan([1], identify=False), [])
        rates = {int(e.split()[2]) for e in unit.events if e.startswith("frame at ")}
        self.assertTrue(rates)
        self.assertTrue(all(38400 <= r <= 230400 for r in rates), rates)  # ME0208 p.26: 38.4-230.4 kbps only
        self.assertEqual(line.baudrate, 115200)  # the starting rate is restored

    def test_port_axis_picks_the_line_strategy(self) -> None:
        from iai_actuator_driver.ec import STRATEGIES

        if "register" not in STRATEGIES:
            self.skipTest("the axis class has no 'register' strategy yet")
        line = sim_rcon_line([1], time_scale=20)
        with Port(line, name="R") as port:
            self.assertEqual(port.axis(1).strategy.name, "register")


if __name__ == "__main__":
    unittest.main()
