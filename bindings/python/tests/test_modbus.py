"""ModbusRTU: framing, every function code, exceptions, retries/timeouts, gap, logging, safety."""

from __future__ import annotations

import struct
import threading
import time
import unittest

from ec_testutil import FrameRecorder, ScriptedTransport, hexbytes, read_response, response

from iai_actuator_driver.errors import (
    IaiCommandRejected,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    UnsafeCommandError,
)
from iai_actuator_driver.modbus import MAX_READ_REGISTERS, ModbusRTU, Transport, build_frame
from iai_actuator_driver.sim import SimulatedEC


def scripted(*responses: bytes | None, retries: int = 2) -> tuple[ModbusRTU, ScriptedTransport]:
    transport = ScriptedTransport(responses)
    return ModbusRTU(transport, retries=retries, min_gap_s=0.0), transport


class FramingTest(unittest.TestCase):
    def test_read_registers_request_and_decode(self) -> None:
        bus, t = scripted(hexbytes("01 03 04 00 00 0B FE 7C 83"))
        self.assertEqual(bus.read_registers(1, 0x9000, 2), [0x0000, 0x0BFE])
        self.assertEqual(t.writes, [hexbytes("01 03 90 00 00 02 E9 0B")])

    def test_write_coil_frames(self) -> None:
        bus, t = scripted(hexbytes("01 05 04 03 FF 00 7D 0A"), hexbytes("01 05 04 03 00 00 3C FA"))
        bus.write_coil(1, 0x0403, True)
        bus.write_coil(1, 0x0403, False)
        self.assertEqual(t.writes, [hexbytes("01 05 04 03 FF 00 7D 0A"), hexbytes("01 05 04 03 00 00 3C FA")])

    def test_write_register_frame(self) -> None:
        bus, t = scripted(hexbytes("01 06 0D 00 10 00 86 A6"))
        bus.write_register(1, 0x0D00, 0x1000)
        self.assertEqual(t.writes, [hexbytes("01 06 0D 00 10 00 86 A6")])

    def test_write_registers_frame(self) -> None:
        bus, t = scripted(hexbytes("01 10 99 00 00 07 AF 57"))
        bus.write_registers(1, 0x9900, [0, 5000, 0, 10, 0, 10000, 30])
        self.assertEqual(
            t.writes, [hexbytes("01 10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 50 CF")]
        )

    def test_read_write_registers(self) -> None:
        bus, t = scripted(response(1, "17 04 00 01 00 02"))
        self.assertEqual(bus.read_write_registers(1, 0x9000, 2, 0x9900, [0x0000, 0x1388]), [1, 2])
        expected = build_frame(1, hexbytes("17 90 00 00 02 99 00 00 02 04 00 00 13 88"))
        self.assertEqual(t.writes, [expected])

    def test_report_slave_id_variable_length(self) -> None:
        bus, t = scripted(response(1, "11 05 EC FF 00 00 20"))
        self.assertEqual(bus.report_slave_id(1), hexbytes("EC FF 00 00 20"))
        self.assertEqual(t.writes, [hexbytes("01 11 C0 2C")])

    def test_iai_command_frame(self) -> None:
        bus, t = scripted(response(1, "45 00 00"))
        result = bus.iai_command(1, 0, 0x09, 1)
        self.assertEqual((result.rejection, result.status), (0, 0))
        self.assertEqual(t.writes, [hexbytes("01 45 00 09 00 01 00 00 18 C2")])

    def test_iai_command_rejected(self) -> None:
        bus, _ = scripted(response(1, "45 21 07"))
        with self.assertRaises(IaiCommandRejected) as ctx:
            bus.iai_command(1, 0, 0x0A)
        self.assertEqual((ctx.exception.rejection, ctx.exception.status), (0x21, 0x07))
        bus2, _ = scripted(response(1, "45 21 07"))
        self.assertEqual(bus2.iai_command(1, 0, 0x0A, check=False).rejection, 0x21)

    def test_iai_command2_frame(self) -> None:
        reply = response(1, "48 00 00" + " 00 00 00 01" + " 00 00 00 00" * 3)
        bus, t = scripted(reply)
        result = bus.iai_command2(1, 0x30, (1,))
        self.assertEqual(result.rejection, 0)
        self.assertEqual(result.status, (1, 0, 0, 0))
        request = t.writes[0]
        self.assertEqual(len(request), 2 + 19 + 2)
        self.assertEqual(request[:5], hexbytes("01 48 00 00 30"))  # addr, 0x48, ID group 0, cmd u16
        self.assertEqual(struct.unpack(">4I", request[5:21]), (1, 0, 0, 0))

    def test_echo_mismatch_is_retried_then_a_frame_error(self) -> None:
        wrong = hexbytes("01 05 04 03 00 00 3C FA")  # answers OFF to an ON request
        bus, t = scripted(wrong, wrong, wrong)
        with self.assertRaises(ModbusFrameError):
            bus.write_coil(1, 0x0403, True)
        self.assertEqual(len(t.writes), 3)  # the request and its 2 retries

    def test_echo_mismatch_then_the_right_echo_succeeds(self) -> None:
        right = build_frame(1, hexbytes("05 04 03 FF 00"))
        bus, t = scripted(hexbytes("01 05 04 03 00 00 3C FA"), right)
        bus.write_coil(1, 0x0403, True)
        self.assertEqual(len(t.writes), 2)

    def test_argument_validation(self) -> None:
        bus, t = scripted()
        with self.assertRaises(ValueError):
            bus.read_registers(0, 0x9000, 1)  # broadcast not supported
        with self.assertRaises(ValueError):
            bus.read_registers(1, 0x9000, 0)
        with self.assertRaises(ValueError):
            bus.write_register(1, 0x0D00, 0x10000)
        with self.assertRaises(ValueError):
            bus.write_registers(1, 0x9900, [])
        with self.assertRaises(ValueError):
            bus.write_registers(1, 0x9900, [0] * 124)
        self.assertEqual(t.writes, [])

    def test_transport_protocol(self) -> None:
        self.assertIsInstance(SimulatedEC(), Transport)
        self.assertIsInstance(ScriptedTransport(), Transport)


class ExceptionAndRetryTest(unittest.TestCase):
    def test_exception_response_not_retried(self) -> None:
        bus, t = scripted(response(1, "83 02"))
        with self.assertRaises(ModbusException) as ctx:
            bus.read_registers(1, 0x7000, 1)
        self.assertEqual((ctx.exception.code, ctx.exception.function), (2, 3))
        self.assertIn("Illegal Data Address", str(ctx.exception))
        self.assertEqual(len(t.writes), 1)

    def test_timeout_then_success(self) -> None:
        bus, t = scripted(None, read_response(1, [7]))
        self.assertEqual(bus.read_registers(1, 0x9002, 1), [7])
        self.assertEqual(len(t.writes), 2)

    def test_timeout_after_retries(self) -> None:
        bus, t = scripted(None, None, None, None)
        with self.assertRaises(ModbusTimeout) as ctx:
            bus.read_registers(1, 0x9002, 1)
        self.assertEqual(len(t.writes), 3)  # 1 + 2 retries
        self.assertIsInstance(ctx.exception, TimeoutError)

    def test_retries_zero(self) -> None:
        bus, t = scripted(None, read_response(1, [7]), retries=0)
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, 0x9002, 1)
        self.assertEqual(len(t.writes), 1)

    def test_retries_per_call(self) -> None:
        """A per-call ``retries`` (the GUI's preemptible reads use 0) overrides the bus default for that call only."""
        bus, t = scripted(None, read_response(1, [7]), None, response(1, "11 05 EC FF 02 00 13"), retries=2)
        with self.assertRaises(ModbusTimeout) as ctx:
            bus.read_registers(1, 0x9002, 1, retries=0)
        self.assertEqual(len(t.writes), 1)
        self.assertNotIn("gave up", " ".join(getattr(ctx.exception, "__notes__", [])))
        self.assertEqual(bus.read_registers(1, 0x9002, 1), [7])  # the default (2 retries) is untouched
        self.assertEqual(bus.retries, 2)
        with self.assertRaises(ModbusTimeout):
            bus.report_slave_id(1, retries=0)
        self.assertEqual(bus.report_slave_id(1), hexbytes("EC FF 02 00 13"))
        with self.assertRaises(ValueError):
            bus.read_registers(1, 0x9002, 1, retries=-1)

    def test_crc_error_then_success(self) -> None:
        bad = bytearray(read_response(1, [7]))
        bad[-1] ^= 0xFF
        bus, t = scripted(bytes(bad), read_response(1, [7]))
        self.assertEqual(bus.read_registers(1, 0x9002, 1), [7])
        self.assertEqual(len(t.writes), 2)

    def test_crc_error_after_retries(self) -> None:
        bad = bytearray(read_response(1, [7]))
        bad[-2] ^= 0x01
        bus, _ = scripted(bytes(bad), bytes(bad), bytes(bad))
        with self.assertRaises(ModbusCRCError):
            bus.read_registers(1, 0x9002, 1)

    def test_partial_response_is_timeout(self) -> None:
        full = read_response(1, [1, 2])
        bus, t = scripted(full[:4], full)
        self.assertEqual(bus.read_registers(1, 0x9000, 2), [1, 2])
        self.assertEqual(len(t.writes), 2)

    def test_wrong_slave_and_wrong_function_retried(self) -> None:
        bus, t = scripted(read_response(2, [1]), response(1, "06 90 02 00 01"), read_response(1, [9]))
        self.assertEqual(bus.read_registers(1, 0x9002, 1), [9])
        self.assertEqual(len(t.writes), 3)
        bus2, _ = scripted(read_response(2, [1]), read_response(2, [1]), read_response(2, [1]))
        with self.assertRaises(ModbusFrameError):
            bus2.read_registers(1, 0x9002, 1)

    def test_bad_crc_on_exception_frame(self) -> None:
        bad = bytearray(response(1, "83 02"))
        bad[-1] ^= 0xFF
        bus, _ = scripted(bytes(bad), response(1, "83 02"))
        with self.assertRaises(ModbusException):
            bus.read_registers(1, 0x7000, 1)

    def test_sim_fault_injection_through_bus(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim, min_gap_s=0.0)
        for kind in ("timeout", "crc", "partial", "wrong_slave"):
            with self.subTest(kind=kind):
                sim.inject_fault(kind)
                self.assertEqual(bus.read_registers(1, 0x9000, 2), [0, 7395])
        sim.inject_fault("timeout", 3)
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, 0x9000, 2)


class BusBehaviourTest(unittest.TestCase):
    def test_large_reads_are_split(self) -> None:
        recorder = FrameRecorder()
        bus = ModbusRTU(SimulatedEC(), frame_logger=recorder, min_gap_s=0.0)
        values = bus.read_registers(1, 0x1000, 130)
        self.assertEqual(len(values), 130)
        counts = [struct.unpack(">H", f[4:6])[0] for f in recorder.tx]
        self.assertEqual(counts, [60, 60, 10])
        self.assertTrue(all(c <= MAX_READ_REGISTERS for c in counts))
        self.assertEqual(values[0x11], 0x3A98)  # position 1 = 150.00 mm

    def test_minimum_gap_between_requests(self) -> None:
        transport = ScriptedTransport([read_response(1, [0])] * 5)
        bus = ModbusRTU(transport)  # default 5 ms gap
        for _ in range(5):
            bus.read_registers(1, 0x9002, 1)
        gaps = [b - a for a, b in zip(transport.write_times, transport.write_times[1:])]
        self.assertTrue(all(g >= 0.0049 for g in gaps), gaps)

    def test_frame_logger_sees_tx_and_rx(self) -> None:
        recorder = FrameRecorder()
        transport = ScriptedTransport([hexbytes("01 03 04 00 00 0B FE 7C 83")])
        bus = ModbusRTU(transport, frame_logger=recorder)
        bus.read_registers(1, 0x9000, 2)
        self.assertEqual(
            recorder.frames,
            [("tx", hexbytes("01 03 90 00 00 02 E9 0B")), ("rx", hexbytes("01 03 04 00 00 0B FE 7C 83"))],
        )

    def test_broken_logger_does_not_break_io(self) -> None:
        def broken(direction: str, frame: bytes) -> None:
            raise RuntimeError("logger bug")

        bus = ModbusRTU(SimulatedEC(), frame_logger=broken, min_gap_s=0.0)
        with self.assertLogs("iai_actuator_driver.modbus", level="ERROR"):
            self.assertEqual(bus.read_registers(1, 0x9002, 1), [0])

    def test_closed_bus(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim)
        bus.close()
        bus.close()
        self.assertTrue(sim.closed)
        with self.assertRaises(ModbusError):
            bus.read_registers(1, 0x9000, 2)

    def test_threads_share_bus(self) -> None:
        bus = ModbusRTU(SimulatedEC(), min_gap_s=0.0)
        errors: list[BaseException] = []

        def worker() -> None:
            try:
                for _ in range(40):
                    self.assertEqual(bus.read_registers(1, 0x9005, 1), [0x3098])
            except BaseException as exc:  # pragma: no cover - reported below
                errors.append(exc)

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        self.assertEqual(errors, [])

    def test_every_fc_against_sim(self) -> None:
        sim = SimulatedEC()
        bus = ModbusRTU(sim, min_gap_s=0.0)
        self.assertEqual(bus.read_registers(1, 0x9005, 1), [0x3098])  # FC03
        bus.write_coil(1, 0x0427, True)  # FC05
        bus.write_register(1, 0x0D03, 2)  # FC06
        self.assertEqual(bus.read_registers(1, 0x0D03, 1), [2])
        bus.write_registers(1, 0x9902, [0, 20])  # FC10 (band only: does not start a move)
        self.assertEqual(bus.read_write_registers(1, 0x9902, 2, 0x9902, [0, 10]), [0, 10])  # FC17
        self.assertEqual(bus.report_slave_id(1)[0], 0xEC)  # FC11
        self.assertEqual(bus.iai_command(1, 0, 0x0B).rejection, 0)  # FC45 decel stop
        self.assertEqual(bus.iai_command2(1, 0x34).rejection, 0)  # FC48 decel stop


class SafetyTest(unittest.TestCase):
    def test_unsafe_iai_commands_refused(self) -> None:
        bus, t = scripted()
        for command in (0x01, 0x02, 0x04, 0x0F, 0x10, 0x1D, 0x73, 0xF3, 0xFF):
            with self.subTest(command=command), self.assertRaises(UnsafeCommandError):
                bus.iai_command(1, 0, command)
        with self.assertRaises(UnsafeCommandError):
            bus.iai_command(1, 0, 0x05, 1)  # software reset
        for command in (0x10, 0x20, 0x24, 0x40, 0x41, 0x80):
            with self.subTest(command2=command), self.assertRaises(UnsafeCommandError):
                bus.iai_command2(1, command)
        self.assertEqual(t.writes, [])

    def test_iai_group_bypass_closed(self) -> None:
        bus, t = scripted()
        with self.assertRaisesRegex(UnsafeCommandError, "group"):
            bus.iai_command(1, 0x05, 0x09, 1)  # safe command, but to another ID group
        for command in (0x1030, 0x1000, 0xFFFD, 0x0130):  # high byte set: not servo / not on the list
            with self.subTest(command=command), self.assertRaises(UnsafeCommandError):
                bus.iai_command2(1, command)
        with self.assertRaises(ValueError):
            bus.iai_command2(1, 0x10030)
        with self.assertRaises(TypeError):
            bus.iai_command2(1, 0x30, (1,), 0x10)  # the ID group is keyword-only (group=...)
        self.assertEqual(t.writes, [])
        bus2, t2 = scripted(response(1, "45 00 00"))
        bus2.iai_command(1, 0x05, 0x09, 1, allow_unsafe=True)
        self.assertEqual(t2.writes[0][:4], hexbytes("01 45 05 09"))

    def test_iai_command2_sends_16_bit_command(self) -> None:
        reply = response(1, "48 00 00" + " 00 00 00 00" * 4)
        bus, t = scripted(reply)
        bus.iai_command2(1, 0x0031)
        self.assertEqual(t.writes[0][:5], hexbytes("01 48 00 00 31"))  # ID group 0, then the u16 command
        self.assertEqual(len(t.writes[0]), 2 + 19 + 2)

    def test_unsafe_allowed_explicitly(self) -> None:
        bus, t = scripted(response(1, "45 00 00"))
        bus.iai_command(1, 0, 0x15, 0, 2, allow_unsafe=True)  # monitor channel select: not on the safe list
        self.assertEqual(len(t.writes), 1)

    def test_never_send_list_needs_its_own_flag(self) -> None:
        # 0x0F clear alarm list wipes the NV history (UNVERIFIED, never sent): allow_unsafe alone is not enough
        for command in (0x01, 0x02, 0x03, 0x04, 0x0F, 0x10, 0x11, 0x1D, 0x20, 0x3C, 0x4F, 0x73, 0x90, 0x91, 0xDD,
                        0xE8, 0xF3, 0xFF):
            with self.subTest(command=hex(command)):
                bus, t = scripted(response(1, "45 00 00"))
                with self.assertRaisesRegex(UnsafeCommandError, "never-send"):
                    bus.iai_command(1, 0, command, allow_unsafe=True)
                with self.assertRaises(UnsafeCommandError):
                    bus.broadcast_iai_command(0, command, allow_unsafe=True, settle_s=0)
                self.assertEqual(t.writes, [])
        for command in (0x0010, 0x0012, 0x0020, 0x0040, 0x0041, 0x0080, 0x1000, 0xFFFD):
            with self.subTest(command2=hex(command)):
                bus, t = scripted(response(1, "48 00 00" + " 00" * 16))
                with self.assertRaisesRegex(UnsafeCommandError, "never-send"):
                    bus.iai_command2(1, command, allow_unsafe=True)
                self.assertEqual(t.writes, [])
        bus, t = scripted(response(1, "45 00 00"))
        bus.iai_command(1, 0, 0x0F, allow_unsafe=True, allow_never_send=True)  # only on an explicit request
        self.assertEqual(len(t.writes), 1)

    def test_rejection_names_the_reason_and_is_not_retried(self) -> None:
        for code, reason in ((0x11, "1st operand error"), (0x23, "servo is OFF"), (0x24, "during homing"),
                             (0x25, "software reset while the servo is ON"), (0x26, "TEACH mode"),
                             (0x27, "invalid HOME"), (0x3A, "monitor (PIO) mode")):
            with self.subTest(code=hex(code)):
                bus, t = scripted(response(1, f"45 {code:02X} 00"), response(1, "45 00 00"))
                with self.assertRaises(IaiCommandRejected) as caught:
                    bus.iai_command(1, 0, 0x0A, 7)
                self.assertEqual(caught.exception.rejection, code)
                self.assertIn(reason, caught.exception.reason)
                self.assertIn(reason, str(caught.exception))
                self.assertEqual(len(t.writes), 1)  # a rejection is an answer: never retried
        bus, t = scripted(response(1, "48 10 07" + " 00" * 16))
        with self.assertRaises(IaiCommandRejected) as caught:
            bus.iai_command2(1, 0x0030, (1,))
        self.assertEqual((caught.exception.rejection, len(t.writes)), (0x1007, 1))
        self.assertIn("software reset while the servo is ON", str(caught.exception))
        self.assertIn("0x1007", str(caught.exception))


class BaudTransport(ScriptedTransport):
    """Scripted transport with a baud rate: only answers (with the script) at ``answer_at``."""

    def __init__(self, answer_at: int, start: int = 115200, responses: list[bytes | None] | None = None) -> None:
        super().__init__(responses or [])
        self.answer_at = answer_at
        self.baudrate = start
        self.rates: list[int] = []

    def set_baudrate(self, baudrate: int) -> None:
        self.baudrate = baudrate
        self.rates.append(baudrate)

    def write(self, data: bytes) -> None:
        if self.baudrate == self.answer_at:
            super().write(data)
        else:
            self.writes.append(bytes(data))  # garbled at the wrong rate: no answer


class BaudDetectTest(unittest.TestCase):
    def test_finds_rate_in_place_without_retries(self) -> None:
        t = BaudTransport(answer_at=38400, responses=[read_response(1, [0x1CE3])])
        bus = ModbusRTU(t, retries=2, min_gap_s=0.0)
        self.assertEqual(bus.detect_baudrate(1), 38400)
        self.assertEqual(bus.baudrate, 38400)
        self.assertEqual(t.rates, [38400])  # 115200 tried first (no answer, no retry), then 38400
        self.assertEqual(len(t.writes), 2)
        self.assertEqual(bus.retries, 2, "retries not restored")
        self.assertFalse(t.closed, "the port must never be closed while detecting")

    def test_exception_reply_counts_as_answer(self) -> None:
        t = BaudTransport(answer_at=115200, responses=[response(1, "83 02")])
        self.assertEqual(ModbusRTU(t, min_gap_s=0.0).detect_baudrate(1), 115200)

    def test_no_answer_restores_start_rate(self) -> None:
        t = BaudTransport(answer_at=1)
        bus = ModbusRTU(t, min_gap_s=0.0)
        with self.assertRaises(ModbusTimeout):
            bus.detect_baudrate(1, candidates=(38400, 9600))
        self.assertEqual(t.baudrate, 115200)
        self.assertEqual(len(t.writes), 3)

    def test_simulator_has_no_baud_rate(self) -> None:
        bus = ModbusRTU(SimulatedEC())
        self.assertIsNone(bus.baudrate)
        with self.assertRaises(TypeError):
            bus.detect_baudrate(1)


class _SilentLine(ScriptedTransport):
    """A line where nothing answers: each read blocks for the transport's timeout, as a serial port's does."""

    def __init__(self, timeout: float) -> None:
        super().__init__()
        self.timeout = timeout

    def read(self, n: int) -> bytes:
        time.sleep(self.timeout)
        return b""


class _WriteFails(ScriptedTransport):
    """A test transport whose write check fails (an error of its own, not the port's)."""

    def write(self, data: bytes) -> None:
        raise ValueError("unexpected request")


class _Failing(ScriptedTransport):
    """A test transport whose ``method`` raises ``error``."""

    def __init__(self, method: str, error: BaseException) -> None:
        super().__init__()
        self._method, self._error = method, error

    def _maybe(self, method: str) -> None:
        if method == self._method:
            raise self._error

    def write(self, data: bytes) -> None:
        self._maybe("write")
        super().write(data)

    def read(self, n: int) -> bytes:
        self._maybe("read")
        return super().read(n)

    def reset_input_buffer(self) -> None:
        self._maybe("reset_input_buffer")
        super().reset_input_buffer()


class PythonTransportTest(unittest.TestCase):
    def test_an_error_that_is_not_an_oserror_reaches_the_caller_from_every_transport_call(self) -> None:
        for method in ("write", "read", "reset_input_buffer"):
            with self.subTest(method=method):
                bus = ModbusRTU(_Failing(method, ValueError(f"{method} refused")), retries=2, min_gap_s=0.0)
                with self.assertRaisesRegex(ValueError, f"{method} refused"):
                    bus.read_registers(1, 0x9000, 1)

    def test_an_oserror_of_the_transport_is_the_cause_of_a_transport_error(self) -> None:
        for method in ("write", "read", "reset_input_buffer"):
            with self.subTest(method=method):
                cause = OSError(f"{method} gone")
                bus = ModbusRTU(_Failing(method, cause), retries=2, min_gap_s=0.0)
                with self.assertRaises(ModbusError) as caught:
                    bus.read_registers(1, 0x9000, 1)
                self.assertIn(f"{method} gone", str(caught.exception))
                self.assertIs(caught.exception.__cause__, cause)

    def test_a_timeout_raised_after_the_bus_was_made_still_bounds_a_probe(self) -> None:
        transport = _SilentLine(0.05)
        bus = ModbusRTU(transport, retries=0, min_gap_s=0.0)
        transport.timeout = 1.0  # the application's choice, after the bus was made
        start = time.perf_counter()
        self.assertFalse(bus.probe(5, timeout=0.2))
        self.assertLess(time.perf_counter() - start, 0.6)
        self.assertEqual(transport.timeout, 1.0)  # put back after the probe

    def test_an_error_of_a_transports_write_that_is_not_an_oserror_reaches_the_caller(self) -> None:
        bus = ModbusRTU(_WriteFails(), retries=2, min_gap_s=0.0)
        with self.assertRaisesRegex(ValueError, "unexpected request"):
            bus.read_registers(1, 0x9000, 1)


if __name__ == "__main__":
    unittest.main()
