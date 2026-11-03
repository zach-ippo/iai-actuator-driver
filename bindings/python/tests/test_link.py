"""Link layer: broadcast frames and their guard, the response-timeout formula, retries, Cell broadcast STOP / pause.

Simulator only: no serial port is opened.
"""

from __future__ import annotations

import struct
import time
import unittest

from iai_actuator_driver import registers as R
from iai_actuator_driver.cell import Cell, Port
from iai_actuator_driver.errors import ModbusTimeout
from iai_actuator_driver.gateway import LINE_RCON, Line, LineRuleError
from iai_actuator_driver.modbus import (
    MIN_FRAME_TIMEOUT_S,
    RECOMMENDED_RETRIES,
    USB_MARGIN_S,
    ModbusRTU,
    build_frame,
)
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_line, sim_ports, sim_rcon_line

from ec_testutil import ScriptedTransport, hexbytes, read_response

FAST = {"poll_interval_s": 0.005}
OPEN = {**FAST, "scan_timeout": 0.01, "identify": False}


def fc03(address: int, count: int) -> bytes:
    return struct.pack(">BHH", 0x03, address, count)


class BroadcastFrameTest(unittest.TestCase):
    """The broadcast frames, CRC-checked, and what may be broadcast."""

    FRAMES = {
        (R.COIL_STOP, True): "00 05 04 2C FF 00 4D 12",
        (R.COIL_STOP, False): "00 05 04 2C 00 00 0C E2",
        (R.COIL_STP, True): "00 05 04 0A FF 00 AC D9",
        (R.COIL_STP, False): "00 05 04 0A 00 00 ED 29",
        (R.COIL_SON, False): "00 05 04 03 00 00 3D 2B",
    }

    def test_coil_broadcast_frames(self) -> None:
        t = ScriptedTransport()
        bus = ModbusRTU(t, min_gap_s=0.0)
        for (coil, on), expected in self.FRAMES.items():
            with self.subTest(coil=hex(coil), on=on):
                frame = bus.broadcast_coil(coil, on, settle_s=0.0)
                self.assertEqual(frame, hexbytes(expected))
        self.assertEqual(t.writes, [hexbytes(f) for f in self.FRAMES.values()])

    def test_broadcast_is_never_answered_or_read(self) -> None:
        t = ScriptedTransport([read_response(0, [1])])  # line noise after the broadcast is dropped
        bus = ModbusRTU(t, min_gap_s=0.0)
        bus.broadcast_coil(R.COIL_STOP, True, settle_s=0.0)
        self.assertEqual(t.read(100), b"")

    def test_reads_cannot_be_broadcast(self) -> None:
        t = ScriptedTransport()
        bus = ModbusRTU(t, min_gap_s=0.0)
        for pdu in (fc03(0x9000, 1), bytes([0x11]), bytes([0x45, 0, 9, 0, 0, 0, 0]), b""):
            with self.subTest(pdu=pdu.hex()):
                with self.assertRaises(ValueError):
                    bus.broadcast(pdu)
        self.assertEqual(t.writes, [])
        bus.broadcast(struct.pack(">BHH", 0x06, 0x0D00, 0x1020), settle_s=0.0)  # FC06 / FC10 are allowed
        self.assertEqual(t.writes[0][0], 0)

    def test_unicast_paths_still_refuse_address_0(self) -> None:
        bus = ModbusRTU(ScriptedTransport(), min_gap_s=0.0)
        with self.assertRaises(ValueError):
            bus.write_coil(0, R.COIL_STOP, True)

    def test_gateway_line_refuses_every_broadcast(self) -> None:
        for kind in ("rcon", "robonet", "robonet-gateway"):
            with self.subTest(kind=kind):
                t = ScriptedTransport()
                bus = ModbusRTU(t, min_gap_s=0.0)
                bus.line = Line(kind)
                with self.assertRaises(LineRuleError):
                    bus.broadcast_coil(R.COIL_STOP, True)
                with self.assertRaises(LineRuleError) as cm:
                    bus.broadcast_iai_command(0, 0x04, 0x14, 0x05, allow_unsafe=True, allow_never_send=True)
                self.assertIn("nothing sent", str(cm.exception))
                with self.assertRaises(LineRuleError):
                    bus.broadcast_iai_command(0, 0x09, 0)  # even a safe-list command: no broadcast here
                self.assertEqual(t.writes, [])

    def test_baud_broadcast_unchanged_on_a_direct_line(self) -> None:
        t = ScriptedTransport()
        bus = ModbusRTU(t, min_gap_s=0.0)
        frame = bus.broadcast_iai_command(0, 0x04, 0x14, 0x05, allow_unsafe=True, allow_never_send=True, settle_s=0.0)
        self.assertEqual(frame, hexbytes("00 45 00 04 00 14 00 05 25 08"))


class TimeoutFormulaTest(unittest.TestCase):
    """Tout = 3 x Tint + RTIM + 10 x (response bytes + 8) / kbps + margin, floor 0.2 s (ME0162-12A p.4-1, 4-2)."""

    def bus(self, baud: int | None) -> ModbusRTU:
        return ModbusRTU(SimulatedEC(baudrate=baud), min_gap_s=0.0)

    def test_60_register_read_at_9600_waits_longer_than_0_2_s(self) -> None:
        bus = self.bus(9600)
        tout = bus.response_timeout(1, fc03(0x9000, 60), 5 + 120)
        expected = max(0.5, (3 * 60 + 5 + 10 * (125 + 8) / 9.6) / 1000 + USB_MARGIN_S)  # 0.5 s floor at 9600 (VERIFIED)
        self.assertAlmostEqual(tout, expected, places=6)
        self.assertGreater(tout, 0.2)

    def test_small_read_keeps_the_verified_0_2_s_floor(self) -> None:
        for baud in (38400, 115200, 230400):
            with self.subTest(baud=baud):
                self.assertEqual(self.bus(baud).response_timeout(1, fc03(0x9000, 1), 7), MIN_FRAME_TIMEOUT_S)

    def test_response_delay_per_slave(self) -> None:
        bus = self.bus(9600)
        base = bus.response_timeout(1, fc03(0x9000, 60), 125)
        bus.set_response_delay(1, 405.0)  # far enough above the 0.5 s slow-rate floor to show the full 0.4 s
        self.assertAlmostEqual(bus.response_timeout(1, fc03(0x9000, 60), 125) - base, 0.4 - (0.5 - (3 * 60 + 5 + 10 * 133 / 9.6) / 1000 - USB_MARGIN_S), places=6)
        self.assertEqual(bus.response_delay_ms(2), 5.0)
        with self.assertRaises(ValueError):
            bus.set_response_delay(1, -1)

    def test_fc10_and_fc17_count_their_registers(self) -> None:
        bus = self.bus(9600)
        fc10 = struct.pack(">BHHB", 0x10, 0x1000, 120, 240) + bytes(240)
        fc17 = struct.pack(">BHHHHB", 0x17, 0x9000, 60, 0x0F00, 60, 120) + bytes(120)
        self.assertAlmostEqual(bus.response_timeout(1, fc10, 8), max(0.5, (360 + 5 + 10 * 16 / 9.6) / 1000 + USB_MARGIN_S))
        self.assertAlmostEqual(bus.response_timeout(1, fc17, 125), max(0.5, (360 + 5 + 10 * 133 / 9.6) / 1000 + USB_MARGIN_S))

    def test_gateway_line_minimum(self) -> None:
        bus = self.bus(115200)
        bus.line = Line(LINE_RCON)
        self.assertEqual(bus.response_timeout(1, fc03(0x9000, 1), 7), 0.3)

    def test_latency_line_gets_the_formula(self) -> None:
        """A 60-register read at 9600 baud answered after 0.25 s arrives (a fixed 0.2 s timeout would have expired)."""
        line = SimLine([SimulatedEC(slave=1, profile="pcon", baudrate=9600)], latency_s=0.25)
        bus = ModbusRTU(line, min_gap_s=0.0, retries=0)
        started = time.perf_counter()
        self.assertEqual(len(bus.read_registers(1, R.POSITION_TABLE_BASE, 60)), 60)  # 60 words of the 64-row table
        self.assertGreaterEqual(time.perf_counter() - started, 0.25)

    def test_latency_line_small_read_times_out_at_the_floor(self) -> None:
        line = SimLine([SimulatedEC(slave=1, baudrate=115200)], latency_s=0.3)
        bus = ModbusRTU(line, min_gap_s=0.0, retries=0)
        started = time.perf_counter()
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, 0x9000, 1)
        self.assertLess(time.perf_counter() - started, 0.3)

    def test_plain_simulator_keeps_its_immediate_timeouts(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim, min_gap_s=0.0, retries=0)
        sim.inject_fault("timeout")
        started = time.perf_counter()
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, 0x9000, 2)
        self.assertLess(time.perf_counter() - started, 0.1)


class RetriesTest(unittest.TestCase):
    def test_manual_retries_are_the_port_and_cell_default(self) -> None:
        self.assertEqual(RECOMMENDED_RETRIES, 3)
        with Port(sim_line([1]), name="L", line="direct") as port:
            self.assertEqual(port.bus.retries, 3)
        with Cell.open([sim_line([1], name="C")], **OPEN) as cell:
            self.assertEqual(cell.ports["C"].bus.retries, 3)

    def test_three_lost_replies_are_absorbed(self) -> None:
        t = ScriptedTransport([None, None, None, read_response(1, [7])])
        bus = ModbusRTU(t, min_gap_s=0.0, retries=RECOMMENDED_RETRIES)
        self.assertEqual(bus.read_registers(1, 0x9002, 1), [7])
        self.assertEqual(len(t.writes), 4)

    def test_bare_bus_default_unchanged(self) -> None:
        self.assertEqual(ModbusRTU(ScriptedTransport()).retries, 2)

    def test_exception_01_is_remembered(self) -> None:
        t = ScriptedTransport([build_frame(1, bytes([0xC5, 0x01]))])
        bus = ModbusRTU(t, min_gap_s=0.0)
        with self.assertRaises(Exception):
            bus.iai_command(1, 0, 0x09, 1)
        self.assertEqual(bus.unsupported_functions(1), frozenset({0x45}))
        self.assertEqual(bus.unsupported_functions(2), frozenset())


class BaudDetectGatewayProbeTest(unittest.TestCase):
    """detect_baudrate takes (slave, register) probes: a line where only a gateway answers still settles the rate."""

    class GatewayAt(ScriptedTransport):
        def __init__(self, rate: int, start: int) -> None:
            super().__init__()
            self.rate = rate
            self.baudrate = start

        def set_baudrate(self, rate: int) -> None:
            self.baudrate = rate

        def write(self, data: bytes) -> None:
            self.writes.append(bytes(data))
            if self.baudrate == self.rate and data[0] == 0x3F and data[1] == 0x03:
                self._buffer += build_frame(0x3F, bytes.fromhex("03 02 80 21"))

    def test_gateway_probe_finds_the_rate(self) -> None:
        t = self.GatewayAt(38400, 115200)
        bus = ModbusRTU(t, min_gap_s=0.0)
        rate = bus.detect_baudrate([1, (0x3F, 0xF700)], [115200, 38400])
        self.assertEqual(rate, 38400)
        self.assertIn(hexbytes("3F 03 F7 00 00 01") + build_frame(0x3F, fc03(0xF700, 1))[-2:], t.writes)

    def test_message_names_the_gateway_probe(self) -> None:
        t = self.GatewayAt(9600, 115200)
        bus = ModbusRTU(t, min_gap_s=0.0)
        with self.assertRaises(ModbusTimeout) as cm:
            bus.detect_baudrate([1, (0x3F, 0xF700)], [115200], probe_timeout=0.01)
        self.assertIn("0x3F", str(cm.exception))


class CellBroadcastTest(unittest.TestCase):
    """Broadcast STOP / pause on daisy chains only (opt-in), never on a gateway line."""

    def test_stop_all_broadcast_on_a_daisy_chain(self) -> None:
        line = SimLine([SimulatedEC(slave=s, time_scale=1) for s in (1, 2, 3)], name="L")
        with Cell.open([line], **OPEN) as cell:
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=50, wait=False)
            time.sleep(0.05)
            mark = len(line.requests)
            cell.stop_all(broadcast=True)
            after = [f for _, f in line.requests[mark:]]
            self.assertEqual(after[0], hexbytes("00 05 04 2C FF 00 4D 12"), "the broadcast STOP is not first")
            self.assertEqual(after[-1], hexbytes("00 05 04 2C 00 00 0C E2"), "no broadcast release at the end")
            self.assertEqual(sum(1 for f in after if f[0] == 0), 2)
            for unit in line:
                self.assertFalse(unit.moving)
                self.assertFalse(unit.stop_held)
                self.assertLess(unit.position_mm, 139.0)

    def test_stop_all_default_sends_no_broadcast(self) -> None:
        line = sim_line([1, 2], name="L", time_scale=20)
        with Cell.open([line], **OPEN) as cell:
            mark = len(line.requests)
            cell.stop_all()
            self.assertNotIn(0, [f[0] for _, f in line.requests[mark:]])

    def test_no_broadcast_for_a_subset_or_a_single_axis_port(self) -> None:
        ports = sim_ports(2, slaves=[1, 2], time_scale=20)
        ports["SIM2"].remove(2)
        with Cell.open(ports, **OPEN) as cell:
            marks = {n: len(line.requests) for n, line in ports.items()}
            cell.stop_all(["SIM1:1", "SIM2:1"], broadcast=True)  # SIM1:2 not selected; SIM2 has one axis
            for name, line in ports.items():
                self.assertNotIn(0, [f[0] for _, f in line.requests[marks[name]:]], name)

    def test_pause_and_resume_all_broadcast(self) -> None:
        line = SimLine([SimulatedEC(slave=s, time_scale=1) for s in (1, 2)], name="L")
        with Cell.open([line], **OPEN) as cell:
            cell.move_all({n: 140.0 for n in cell.names}, speed_mm_s=20, wait=False)
            mark = len(line.requests)
            cell.pause_all(broadcast=True)
            self.assertEqual(line.requests[mark][1], hexbytes("00 05 04 0A FF 00 AC D9"))
            time.sleep(0.3)
            held = [u.position_mm for u in line]
            time.sleep(0.2)
            self.assertEqual(held, [u.position_mm for u in line], "an axis kept moving while paused")
            mark = len(line.requests)
            cell.resume_all(broadcast=True)
            self.assertEqual(line.requests[-1][1], hexbytes("00 05 04 0A 00 00 ED 29"))
            time.sleep(0.2)
            self.assertTrue(all(u.moving for u in line))
            cell.stop_all()

    def test_port_broadcasts_refused_on_rcon(self) -> None:
        line = sim_rcon_line([1, 2], time_scale=20)
        with Port(line, name="R") as port:
            port.scan(identify=False)
            for call in (port.broadcast_stop, port.broadcast_pause, port.broadcast_servo_off):
                with self.subTest(call=call.__name__), self.assertRaises(LineRuleError):
                    call()
            with self.assertRaises(LineRuleError) as cm:
                port.set_baudrate(230400)
            self.assertIn("ME0413 p.6-1 #2", str(cm.exception))
        self.assertNotIn(0, [f[0] for _, f in line.requests])
        self.assertEqual(line.dropped_broadcasts, [])

    def test_stop_all_on_rcon_sends_no_broadcast_and_no_coil(self) -> None:
        line = sim_rcon_line([1, 2], time_scale=20)
        with Cell.open([line], position_unit_mm=0.01, **OPEN) as cell:
            mark = len(line.requests)
            cell.stop_all(broadcast=True, return_exceptions=True)
            sent = [f for _, f in line.requests[mark:]]
            self.assertNotIn(0, [f[0] for f in sent])
            self.assertNotIn(0x05, [f[1] for f in sent], "an FC05 coil reached an RCON line")


if __name__ == "__main__":
    unittest.main()
