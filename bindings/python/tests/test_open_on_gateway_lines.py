"""Elecylinder.open on gateway lines: line detection, retries, strategy, identity without FC11, absent directory.

Simulator only. Every gateway rule is DOC (ME0413, ME0208) and UNVERIFIED on hardware.
"""

from __future__ import annotations

import unittest

from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import EcNotReadyError, LineRuleError, ModbusTimeout
from iai_actuator_driver.gateway import LINE_RCON, LINE_ROBONET, PositionUnitError, UnsupportedLineError
from iai_actuator_driver.sim_line import SimLine, sim_line, sim_rcon_line, sim_robonet_line

FAST = {"poll_interval_s": 0.005}
GATEWAY_FRAMES = (b"\xd0\x03\x37\x00", b"\x3f\x03\xf7\x00")  # the two read-only line probes


def sent(line: SimLine) -> list[bytes]:
    return [f for _, f in line.requests]


def function_codes(line: SimLine) -> set[int]:
    return {f[1] for f in sent(line) if len(f) > 1}


def _no_access(address: int) -> bool:
    """The directory, identification, limit, parameter and table areas (below the DRG1 registers at 0x0D00) and the
    0x8000 system area: 'system, no access' on an RCON axis (DOC ME0413 p.5-10)."""
    return address < R.DRG1 or 0x8000 <= address < 0x9000


def no_access_reads(line: SimLine) -> list[bytes]:
    return [f for f in sent(line) if f[1] == 0x03 and 1 <= f[0] <= 16 and _no_access(int.from_bytes(f[2:4], "big"))]


class OpenDefaultsTest(unittest.TestCase):
    def test_three_retries_by_default(self) -> None:
        with Elecylinder.open(sim_line([1]), **FAST) as ec:
            self.assertEqual(ec.bus.retries, 3)

    def test_retries_can_still_be_chosen(self) -> None:
        with Elecylinder.open(sim_line([1]), retries=1, **FAST) as ec:
            self.assertEqual(ec.bus.retries, 1)

    def test_plain_line_is_probed_once_and_stays_direct(self) -> None:
        line = sim_line([1])
        with Elecylinder.open(line, **FAST) as ec:
            self.assertEqual(ec.bus.line.kind, "direct")
            self.assertIsNone(ec.line_name)
            self.assertEqual(ec.stop_path, "coil")
            self.assertEqual(ec.strategy.name, "coil")
        probes = [f for f in sent(line) if f[:4] in GATEWAY_FRAMES]
        self.assertEqual(len(probes), 2)
        self.assertEqual(function_codes(line) - {0x03}, set(), "line detection sent something that is not a read")

    def test_stated_direct_line_sends_no_probe(self) -> None:
        line = sim_line([1])
        with Elecylinder.open(line, line="direct", **FAST):
            pass
        self.assertFalse([f for f in sent(line) if f[:4] in GATEWAY_FRAMES])

    def test_unknown_line_kind_is_refused_and_the_port_closed(self) -> None:
        line = sim_line([1])
        with self.assertRaises(ValueError):
            Elecylinder.open(line, line="canbus", **FAST)
        self.assertTrue(line.closed)


class OpenRconTest(unittest.TestCase):
    def test_rcon_axis_takes_the_register_strategy(self) -> None:
        line = sim_rcon_line([1], serials={1: "RC-TEST-01"})
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            self.assertEqual(ec.bus.line.kind, LINE_RCON)
            self.assertEqual(ec.line_name, LINE_RCON)
            self.assertEqual(ec.strategy.name, "register")
            self.assertEqual(ec.stop_path, "register")
            self.assertEqual(ec.bus.line.position_unit_mm(1), 0.01)

    def test_an_explicit_strategy_is_kept(self) -> None:
        with Elecylinder.open(sim_rcon_line([1]), strategy="coil", position_unit_mm=0.01, **FAST) as ec:
            self.assertEqual(ec.strategy.name, "coil")
            self.assertEqual(ec.stop_path, "register", "the stop path follows the line even when the strategy is coil")

    def test_identity_is_the_serial_code_and_fc11_is_not_sent(self) -> None:
        line = sim_rcon_line([1], serials={1: "RC-TEST-01"})
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            info = ec.identify()
        # a directory that does answer gives the system / unit code (CTCL) after the serial code
        self.assertEqual(info.identity_source, "CTCL 0x0300")
        self.assertIsNotNone(info.system_code)
        self.assertTrue(any("FC11" in e for e in info.errors))
        self.assertNotIn(0x11, function_codes(line))
        self.assertTrue([f for f in sent(line) if f[1:4] == b"\x03\x94\x00"], "the serial code was not read")

    def test_unit_code_read_does_not_send_fc11(self) -> None:
        line = sim_rcon_line([1])
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            ec.identify()
        self.assertNotIn(0x11, function_codes(line))

    def test_gateway_must_be_in_manu_before_control(self) -> None:
        line = sim_rcon_line([1], manu=False)
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            with self.assertRaises(EcNotReadyError):
                ec.enable_modbus_control()

    def test_no_coil_is_written_when_opening_and_stopping(self) -> None:
        line = sim_rcon_line([1])
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            ec.enable_modbus_control()
            ec.stop()
        self.assertNotIn(0x05, function_codes(line))
        self.assertNotIn(0x45, function_codes(line))


class RconPositionUnitTest(unittest.TestCase):
    """mm commands behind an RCON gateway need the unit's position unit (parameter No.207, not readable over Modbus)."""

    def test_motion_is_refused_until_the_unit_is_stated(self) -> None:
        line = sim_rcon_line([1])
        with Elecylinder.open(line, stroke=(0.0, 100.0), **FAST) as ec:
            ec.enable_modbus_control()
            ec.servo(True, wait=True)
            before = len(sent(line))
            with self.assertRaises(PositionUnitError) as cm:
                ec.move_to(10.0, 50.0, wait=False)
            self.assertIn("No.207", str(cm.exception))
            with self.assertRaises(PositionUnitError):
                ec.check_move(10.0, 50.0)
            ec.bus.line.set_position_unit(0.01)
            ec.move_to(10.0, 50.0, wait=True, timeout=10.0)
        written = {f[1] for f in sent(line)[before:]} - {0x03}
        self.assertEqual(written & {0x10}, {0x10}, "the move went out only after the unit was stated")

    def test_a_0_001_unit_scales_target_and_band_on_the_wire(self) -> None:
        line = sim_rcon_line([1], position_unit_mm=0.001)
        with Elecylinder.open(line, position_unit_mm=0.001, stroke=(0.0, 100.0), **FAST) as ec:
            ec.enable_modbus_control()
            ec.servo(True, wait=True)
            status = ec.move_to(45.67, 50.0, band_mm=0.2, wait=True, timeout=10.0)
            self.assertAlmostEqual(status.position_mm, 45.67, places=2)
            self.assertAlmostEqual(ec.read_monitor()["position"], 45.67, places=2)
        blocks = [f for f in sent(line) if f[1] == 0x10 and f[2:4] == b"\x99\x00"]
        self.assertTrue(blocks)
        self.assertEqual(int.from_bytes(blocks[-1][7:11], "big", signed=True), 45670, "target in 0.001 mm counts")
        self.assertEqual(int.from_bytes(blocks[-1][11:15], "big"), 200, "band in 0.001 mm counts")
        self.assertEqual(int.from_bytes(blocks[-1][15:19], "big"), 5000, "the speed is not scaled")

    def test_a_0_01_unit_changes_nothing(self) -> None:
        line = sim_rcon_line([1])
        with Elecylinder.open(line, position_unit_mm=0.01, stroke=(0.0, 100.0), **FAST) as ec:
            ec.enable_modbus_control()
            ec.servo(True, wait=True)
            ec.move_to(45.67, 50.0, wait=True, timeout=10.0)
        blocks = [f for f in sent(line) if f[1] == 0x10 and f[2:4] == b"\x99\x00"]
        self.assertEqual(int.from_bytes(blocks[-1][7:11], "big", signed=True), 4567)


class OpenWithoutDirectoryTest(unittest.TestCase):
    """RCON axes: a timed-out 0x0000 read means 'no directory', not a failure (DOC ME0413 p.5-10)."""

    def test_identify_treats_the_silent_directory_as_absent(self) -> None:
        line = sim_rcon_line([1], silent_no_access=True, serials={1: "RC-TEST-02"})
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            info = ec.identify()
            self.assertTrue(ec.directory_absent)
        self.assertTrue(any("treated as absent" in e for e in info.errors), info.errors)
        self.assertIsNone(info.soft_limits_mm)
        self.assertIsNone(info.max_speed_mm_s)
        self.assertEqual(info.serial_number, "RC-TEST-02", "the serial code is the identity")
        self.assertEqual(info.identity_source, "serial code 0x9400")
        below = no_access_reads(line)
        self.assertEqual([f[2:4].hex() for f in below].count("0000"), 4, "one read, 3 retries, then nothing more")
        self.assertEqual({f[2:4].hex() for f in below}, {"0000"}, "reads of the no-access areas beyond the directory")

    def test_limits_are_not_read_without_a_directory(self) -> None:
        line = sim_rcon_line([1], silent_no_access=True)
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            self.assertIsNone(ec.read_soft_limits())
            self.assertIn("pass stroke", ec.soft_limits_note)
            ec.read_axis_limits()
            self.assertEqual(ec.max_positions(), R.POSITION_ACTIVE_ROWS)
        self.assertEqual({f[2:4].hex() for f in no_access_reads(line)}, {"0000"})

    def test_motion_needs_an_explicit_stroke(self) -> None:
        line = sim_rcon_line([1], silent_no_access=True)
        with Elecylinder.open(line, position_unit_mm=0.01, **FAST) as ec:
            ec.enable_modbus_control()
            with self.assertRaises(EcNotReadyError) as cm:
                ec.move_to(10.0, 50.0, wait=False)
            self.assertIn("stroke", str(cm.exception))

    def test_a_move_with_an_explicit_stroke_uses_only_documented_rcon_functions(self) -> None:
        line = sim_rcon_line([1], silent_no_access=True)
        with Elecylinder.open(line, position_unit_mm=0.01, stroke=(0.0, 100.0), **FAST) as ec:
            ec.identify()
            ec.enable_modbus_control()
            ec.servo(True, wait=True)
            status = ec.move_to(10.0, 50.0, wait=True, timeout=10.0)
            self.assertEqual(status.position_mm, 10.0)
            self.assertEqual(status.line, LINE_RCON)
            ec.stop()
        self.assertEqual({f[2:4].hex() for f in no_access_reads(line)}, {"0000"})
        self.assertLessEqual(function_codes(line), {0x03, 0x06, 0x10, 0x17}, "FC01/03/04/06/10/17 only on RCON")

    def test_a_directory_timeout_still_raises_on_a_direct_line(self) -> None:
        line = sim_line([1])
        with Elecylinder.open(line, **FAST) as ec:
            line.remove(1)
            before = len(sent(line))
            with self.assertRaises(ModbusTimeout):
                ec.read_position_table(0)  # reads the directory first
            reads = [f for f in sent(line)[before:] if f[1:4] == b"\x03\x00\x00"]
            self.assertTrue(reads, "the directory was not read")
            self.assertFalse(ec.directory_absent)


class OpenRobonetTest(unittest.TestCase):
    def test_gateway_mode_gives_no_axis_and_closes_the_port(self) -> None:
        line = sim_robonet_line([1], gateway_mode=True)
        with self.assertRaises(UnsupportedLineError):
            Elecylinder.open(line, **FAST)
        self.assertTrue(line.closed)

    def test_through_mode_takes_the_coil_strategy_and_refuses_iai_commands(self) -> None:
        line = sim_robonet_line([1])
        with Elecylinder.open(line, line="robonet", **FAST) as ec:
            self.assertEqual(ec.bus.line.kind, LINE_ROBONET)
            self.assertEqual(ec.strategy.name, "coil")
            self.assertEqual(ec.stop_path, "coil")
            with self.assertRaises(LineRuleError):
                ec.software_reset()
        self.assertFalse([f for f in sent(line) if f[:4] in GATEWAY_FRAMES], "a stated line is not probed")


if __name__ == "__main__":
    unittest.main()
