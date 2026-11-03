"""Seeded random tests: malformed Modbus replies, random status words and random command sequences.

No hardware and no serial port. Every case comes from ``random.Random`` with a fixed seed, and the bus, the driver and
the simulator run on one virtual clock (``iai_actuator_driver.testing.virtual_clock``): a wait advances the clock at
once, so a timeout costs no real time and every run sends the same frames and gets the same answers, on any PC.

* ``ModbusRTU`` against random garbage, truncated frames, a wrong CRC, slave or function, an exception response for
  every function code, extra trailing bytes and the reply to the previous request, through the real class and fake
  transports: only the exceptions of ``iai_actuator_driver.errors`` come out, the number of attempts follows ``retries``, no
  request waits longer than its timeouts allow, and no damaged or foreign reply is taken as the answer.
* ``Status.from_registers`` over random register words: no crash, and every flag agrees with its bit.
* ``Elecylinder`` on the simulated EC-DS4H slider under random command sequences (servo, home, moves inside and outside
  the stroke, stop, pause, resume, alarm reset): only documented exceptions, the axis stands still after ``stop()``,
  and its position stays inside the soft limits.
"""

from __future__ import annotations

import contextlib
import json
import logging
import math
import random
import struct
import unittest
from collections import Counter
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, replace

from ec_testutil import ScriptedTransport

from iai_actuator_driver import alarms, info
from iai_actuator_driver import registers as R
from iai_actuator_driver.ec import Elecylinder, MoveHandle, Status
from iai_actuator_driver.errors import (
    EXCEPTION_NAMES,
    EcAlarmError,
    EcDriverError,
    EcNotReadyError,
    IaiCommandRejected,
    ModbusCRCError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    SoftLimitError,
)
from iai_actuator_driver.modbus import (
    IAI_SAFE_COMMANDS,
    IAI_SAFE_COMMANDS2,
    MAX_READ_REGISTERS,
    MAX_WRITE_REGISTERS,
    SLAVE_ID_TIMEOUT_S,
    IaiResponse,
    IaiResponse2,
    ModbusRTU,
    build_frame,
    check_crc,
)
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.testing import VirtualClock, virtual_clock

FRAME_SEED = 1029
STATUS_SEED = 2026
SEQUENCES = ((11, "coil"), (23, "register"), (37, "iai"))  # (seed, command strategy) of each random sequence

REQUEST_KINDS = ("fc03", "fc05", "fc06", "fc10", "fc11", "fc17", "iai", "iai2", "probe")
EXCEPTION_CODES = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0A, 0x0B, 0x7F, 0xFF)
"""Exception codes the fake slaves answer with: the four of ``errors.EXCEPTION_NAMES`` and codes without a name."""

READ_LIMIT = 5000
"""Reads one request may make. The longest wait here (FC11: 0.65 s in 2 ms steps) needs about 330: more means a read
loop that no deadline ends."""


# =========================================================================== requests, replies and what must happen


@dataclass
class Request:
    """One request of the fuzz: how to send it, its correct reply and what the call returns for that reply."""

    name: str
    slave: int
    function: int
    reply_len: int | None  # length of a normal reply; None: a byte count follows (FC11)
    reply: bytes
    send: Callable[[ModbusRTU], object]
    result: object = None  # what ``send`` returns for ``reply``
    rejected: bool = False  # ``reply`` carries an IAI rejection: ``send`` raises IaiCommandRejected
    retried: bool = True  # False for probe(): one attempt, and True / False instead of an exception
    wait_s: float = 0.0  # longest wait of one attempt on a transport without a read timeout


def random_request(
    rng: random.Random, kind: str | None = None, *, slave: int | None = None, rejection: bool = True
) -> Request:
    """A random request of ``kind`` (one of :data:`REQUEST_KINDS`; random when None) with a random correct reply.
    ``rejection``: an IAI reply may carry a rejection code."""
    kind = rng.choice(REQUEST_KINDS) if kind is None else kind
    s = rng.randint(1, 247) if slave is None else slave
    if kind == "fc03":
        n = rng.randint(1, MAX_READ_REGISTERS)
        address = rng.randint(0, 0x10000 - n)
        words = [rng.getrandbits(16) for _ in range(n)]
        reply = build_frame(s, struct.pack(f">BB{n}H", 0x03, 2 * n, *words))
        return Request(f"FC03 {address:#06x} x{n}", s, 0x03, 5 + 2 * n, reply,
                       lambda bus: bus.read_registers(s, address, n), words)  # fmt: skip
    if kind == "fc05":
        address, on = rng.getrandbits(16), rng.random() < 0.5
        reply = build_frame(s, struct.pack(">BHH", 0x05, address, 0xFF00 if on else 0x0000))
        return Request(f"FC05 {address:#06x} {on}", s, 0x05, 8, reply, lambda bus: bus.write_coil(s, address, on))
    if kind == "fc06":
        address, value = rng.getrandbits(16), rng.getrandbits(16)
        reply = build_frame(s, struct.pack(">BHH", 0x06, address, value))
        return Request(f"FC06 {address:#06x}", s, 0x06, 8, reply, lambda bus: bus.write_register(s, address, value))
    if kind == "fc10":
        n, address = rng.randint(1, MAX_WRITE_REGISTERS), rng.getrandbits(16)
        values = [rng.getrandbits(16) for _ in range(n)]
        reply = build_frame(s, struct.pack(">BHH", 0x10, address, n))
        return Request(f"FC10 {address:#06x} x{n}", s, 0x10, 8, reply,
                       lambda bus: bus.write_registers(s, address, values))  # fmt: skip
    if kind == "fc11":
        data = bytes(rng.getrandbits(8) for _ in range(rng.randint(1, 30)))
        reply = build_frame(s, bytes([0x11, len(data)]) + data)
        return Request("FC11", s, 0x11, None, reply, lambda bus: bus.report_slave_id(s), data,
                       wait_s=SLAVE_ID_TIMEOUT_S)  # fmt: skip
    if kind == "fc17":
        rn, wn = rng.randint(1, MAX_READ_REGISTERS), rng.randint(1, 60)
        ra, wa = rng.getrandbits(16), rng.getrandbits(16)
        values = [rng.getrandbits(16) for _ in range(wn)]
        words = [rng.getrandbits(16) for _ in range(rn)]
        reply = build_frame(s, struct.pack(f">BB{rn}H", 0x17, 2 * rn, *words))
        return Request(f"FC17 {ra:#06x} x{rn}", s, 0x17, 5 + 2 * rn, reply,
                       lambda bus: bus.read_write_registers(s, ra, rn, wa, values), words)  # fmt: skip
    if kind == "iai":
        command = rng.choice(sorted(IAI_SAFE_COMMANDS))
        op1 = 0 if command == 0x05 else rng.getrandbits(16)  # 0x45/0x05 with op1 != 0 is the software reset
        op2, status = rng.getrandbits(16), rng.getrandbits(8)
        code = rng.randint(0x01, 0x3F) if rejection and rng.random() < 0.2 else 0
        reply = build_frame(s, bytes([0x45, code, status]))
        return Request(f"0x45 cmd {command:#04x}", s, 0x45, 6, reply,
                       lambda bus: bus.iai_command(s, 0, command, op1, op2), IaiResponse(0, status),
                       rejected=code != 0)  # fmt: skip
    if kind == "iai2":
        command = rng.choice(sorted(IAI_SAFE_COMMANDS2))
        operands = tuple(rng.getrandbits(32) for _ in range(rng.randint(0, 4)))
        status = (rng.getrandbits(32), rng.getrandbits(32), rng.getrandbits(32), rng.getrandbits(32))
        code = rng.randint(0x1000, 0x13FF) if rejection and rng.random() < 0.2 else 0
        reply = build_frame(s, struct.pack(">BH4I", 0x48, code, *status))
        return Request(f"0x48 cmd {command:#06x}", s, 0x48, 22, reply,
                       lambda bus: bus.iai_command2(s, command, operands), IaiResponse2(0, status),
                       rejected=code != 0)  # fmt: skip
    if kind == "probe":
        address, wait = rng.getrandbits(16), rng.choice((0.02, 0.05, 0.2))
        reply = build_frame(s, struct.pack(">BBH", 0x03, 2, rng.getrandbits(16)))
        return Request(f"probe {address:#06x}", s, 0x03, 7, reply,
                       lambda bus: bus.probe(s, timeout=wait, address=address), True, retried=False,
                       wait_s=wait)  # fmt: skip
    raise ValueError(f"unknown request kind {kind!r}")


def classify(data: bytes, request: Request) -> tuple[str, bytes]:
    """What a Modbus RTU master must make of ``data``, the bytes that answer one attempt of ``request``.

    ``("ok", frame)`` and ``("exception", frame)`` end the request; ``"timeout"`` (too few bytes), ``"crc"`` and
    ``"frame"`` (another slave or function) are retried. The frame's length follows from its head: 5 bytes for an
    exception response, else the request's reply length, or 5 + the byte count for FC11."""
    if len(data) < 2:
        return "timeout", b""
    if data[0] != request.slave:
        return "frame", b""
    if data[1] == request.function | 0x80:
        verdict, length = "exception", 5
    elif data[1] != request.function:
        return "frame", b""
    elif request.reply_len is None:
        if len(data) < 3:
            return "timeout", b""
        verdict, length = "ok", 5 + data[2]
    else:
        verdict, length = "ok", request.reply_len
    frame = data[:length]
    if len(frame) < length:
        return "timeout", b""
    return (verdict, frame) if check_crc(frame) else ("crc", b"")


RETRIED = {"timeout": ModbusTimeout, "crc": ModbusCRCError, "frame": ModbusFrameError}


@dataclass
class Expected:
    """The outcome a request must have: its result or its exception, and the number of frames sent."""

    attempts: int
    result: object = None
    error: type[Exception] | None = None
    frame: bytes = b""  # the exception response or the rejecting reply that ended the request


def expected(request: Request, script: list[bytes | None], retries: int) -> Expected:
    """What ``request`` must do when attempt k is answered with ``script[k]`` (None: silence)."""
    tries = retries + 1 if request.retried else 1
    verdict = "timeout"
    for attempt in range(tries):
        data = script[attempt] if attempt < len(script) else None
        verdict, frame = classify(data or b"", request)
        if verdict == "exception":
            if not request.retried:
                return Expected(attempt + 1, True, frame=frame)  # probe(): an exception response is an answer
            return Expected(attempt + 1, error=ModbusException, frame=frame)
        if verdict == "ok":
            if request.rejected:
                return Expected(attempt + 1, error=IaiCommandRejected, frame=frame)
            return Expected(attempt + 1, request.result)
    if not request.retried:
        return Expected(tries, False)
    return Expected(tries, error=RETRIED[verdict])


CORRUPTIONS = ("silence", "garbage", "truncated", "bit_flip", "wrong_slave", "wrong_function", "exception",
               "foreign_exception", "trailing", "noise_first", "previous")  # fmt: skip


def damaged_reply(rng: random.Random, request: Request, previous: Request) -> bytes | None:
    """A random damaged, foreign or missing answer to ``request``. It never reads as a correct reply carrying other
    data than ``request.reply``: no Modbus master could tell such a frame from the real answer."""
    for _ in range(100):
        data = _damage(rng, request, previous, rng.choice(CORRUPTIONS))
        if data is None:
            return None
        verdict, frame = classify(data, request)
        if verdict != "ok" or frame == request.reply:
            return data
    return None


def _damage(rng: random.Random, request: Request, previous: Request, kind: str) -> bytes | None:
    good = request.reply
    noise = lambda n: bytes(rng.getrandbits(8) for _ in range(n))  # noqa: E731
    if kind == "silence":
        return None
    if kind == "garbage":
        data = bytearray(noise(rng.randint(1, 40)))
        if rng.random() < 0.5:  # this slave, often this function: reaches the length and CRC checks too
            data[0] = request.slave
            if len(data) > 1 and rng.random() < 0.7:
                data[1] = rng.choice((request.function, request.function | 0x80))
        return bytes(data)
    if kind == "truncated":
        return good[: rng.randrange(1, len(good))]
    if kind == "bit_flip":
        data = bytearray(good)
        bit = rng.randrange(8 * len(data))
        data[bit // 8] ^= 1 << (bit % 8)
        return bytes(data)
    if kind == "wrong_slave":
        return build_frame((request.slave + rng.randint(1, 255)) % 256, good[1:-2])
    if kind == "wrong_function":
        other = rng.choice([f for f in range(256) if f not in (request.function, request.function | 0x80)])
        return build_frame(request.slave, bytes([other]) + good[2:-2])
    if kind == "exception":
        return build_frame(request.slave, bytes([request.function | 0x80, rng.choice(EXCEPTION_CODES)]))
    if kind == "foreign_exception":  # an exception response to another function
        other = rng.choice([f for f in range(0x80) if f != request.function])
        return build_frame(request.slave, bytes([other | 0x80, rng.choice(EXCEPTION_CODES)]))
    if kind == "trailing":
        return good + noise(rng.randint(1, 12))
    if kind == "noise_first":
        return noise(rng.randint(1, 6)) + good
    return previous.reply + good  # "previous": the late reply to the request before, then this one


class GuardedTransport(ScriptedTransport):
    """:class:`ScriptedTransport` that also takes bytes arriving between requests (:meth:`arrive`) and fails the test
    when one request keeps reading past :data:`READ_LIMIT`."""

    def __init__(self, responses: Iterable[bytes | None] = ()) -> None:
        super().__init__(responses)
        self.reads = 0

    def write(self, data: bytes) -> None:
        self.reads = 0
        super().write(data)

    def read(self, n: int) -> bytes:
        self.reads += 1
        if self.reads > READ_LIMIT:
            raise AssertionError(f"one request read {self.reads} times: its wait never ends")
        return super().read(n)

    def arrive(self, data: bytes) -> None:
        """Bytes reach the PC before the next request (line noise, a reply that came late)."""
        self._buffer += data


def hexdump(script: Iterable[bytes | None]) -> list[str | None]:
    return [None if data is None else data.hex(" ") for data in script]


# =========================================================================== (a) malformed replies


class MalformedReplyTest(unittest.TestCase):
    """ModbusRTU against damaged, foreign and missing replies, through a fake transport."""

    CASES = 2500

    def check(self, bus: ModbusRTU, transport: GuardedTransport, clock: VirtualClock, request: Request,
              script: list[bytes | None], retries: int) -> Expected:  # fmt: skip
        """Send ``request`` with ``script`` as the answers and check the outcome against :func:`expected`."""
        want = expected(request, script, retries)
        bus.retries = retries
        transport.responses = list(script)
        sent, start = len(transport.writes), clock.now
        result: object = None
        error: EcDriverError | None = None
        try:
            result = request.send(bus)
        except EcDriverError as exc:  # anything else is not a documented exception: the test errors
            error = exc
        frames = transport.writes[sent:]
        self.assertEqual(len(frames), want.attempts, "frames sent")
        self.assertEqual(len(set(frames)), 1, "a retry sends the same request again")
        if want.error is None:
            self.assertIsNone(error)
            self.assertEqual(result, want.result)
        else:
            self.assertIs(type(error), want.error, f"{error!r}")
            if isinstance(error, ModbusException):
                self.assertEqual((error.code, error.function), (want.frame[2], request.function))
            elif isinstance(error, IaiCommandRejected):
                code = want.frame[2] if request.function == 0x45 else struct.unpack(">H", want.frame[2:4])[0]
                self.assertEqual(error.rejection, code)
            elif want.attempts > 1:
                self.assertIn(f"gave up after {want.attempts} attempts", getattr(error, "__notes__", []))
        # each attempt waits at most its deadline, the 5 ms gap and one 2 ms read step
        self.assertLessEqual(clock.now - start, want.attempts * (request.wait_s + 0.01), "time spent")
        return want

    def test_random_replies(self) -> None:
        """Random requests of every kind against random answers per attempt (the correct reply about one time in four):
        the outcome, the frames sent and the time spent are exactly what Modbus RTU framing calls for."""
        rng = random.Random(FRAME_SEED)
        transport = GuardedTransport()
        unsupported: dict[int, set[int]] = {}
        outcomes: Counter[str] = Counter()
        with virtual_clock() as clock:
            bus = ModbusRTU(transport)
            previous = random_request(rng)
            for case in range(self.CASES):
                request = random_request(rng)
                retries = rng.randint(0, 3)
                script = [request.reply if rng.random() < 0.25 else damaged_reply(rng, request, previous)
                          for _ in range(retries + 1)]  # fmt: skip
                if rng.random() < 0.2:
                    transport.arrive(previous.reply)  # a late reply already waiting: flushed before the request
                with self.subTest(case=case, request=request.name, retries=retries, script=hexdump(script)):
                    want = self.check(bus, transport, clock, request, script, retries)
                    if want.frame and want.error is not IaiCommandRejected and want.frame[2] == 0x01:
                        unsupported.setdefault(request.slave, set()).add(request.function)
                    self.assertEqual(bus.unsupported_functions(request.slave),
                                     frozenset(unsupported.get(request.slave, ())))  # fmt: skip
                    outcomes[want.error.__name__ if want.error else "result"] += 1
                previous = request
        for name in ("result", "ModbusTimeout", "ModbusCRCError", "ModbusFrameError", "ModbusException",
                     "IaiCommandRejected"):  # fmt: skip
            self.assertGreater(outcomes[name], 20, f"too few cases ended with {name}: {dict(outcomes)}")

    def test_exception_response_for_every_function(self) -> None:
        """An exception response to any function is raised at once (one frame sent, even with retries left), with its
        code and the request's function; code 01 marks the function as unsupported by that slave. probe() counts it
        as an answer."""
        rng = random.Random(FRAME_SEED + 1)
        with virtual_clock():
            for kind in REQUEST_KINDS:
                for code in EXCEPTION_CODES:
                    request = random_request(rng, kind, rejection=False)
                    answer = build_frame(request.slave, bytes([request.function | 0x80, code]))
                    transport = GuardedTransport([answer, request.reply])
                    bus = ModbusRTU(transport, retries=3)
                    with self.subTest(kind=kind, code=code):
                        if not request.retried:
                            self.assertIs(request.send(bus), True)
                        else:
                            with self.assertRaises(ModbusException) as caught:
                                request.send(bus)
                            self.assertEqual((caught.exception.code, caught.exception.function),
                                             (code, request.function))  # fmt: skip
                            self.assertIn(EXCEPTION_NAMES.get(code, "Unknown exception"), str(caught.exception))
                        self.assertEqual(len(transport.writes), 1)
                        self.assertEqual(request.function in bus.unsupported_functions(request.slave), code == 0x01)

    def test_trailing_bytes_never_reach_the_next_request(self) -> None:
        """Bytes after a complete reply (noise, or a copy of another frame) are left behind and flushed before the
        next request: every request of a sequence still gets its own answer, in one attempt."""
        rng = random.Random(FRAME_SEED + 2)
        with virtual_clock():
            transport = GuardedTransport()
            bus = ModbusRTU(transport, retries=2)
            previous = random_request(rng, rejection=False)
            for case in range(200):
                request = random_request(rng, rejection=False)
                tail = previous.reply if case % 2 else bytes(rng.getrandbits(8) for _ in range(rng.randint(1, 20)))
                transport.responses = [request.reply + tail]
                sent = len(transport.writes)
                with self.subTest(case=case, request=request.name, tail=tail.hex(" ")):
                    self.assertEqual(request.send(bus), request.result)
                    self.assertEqual(len(transport.writes) - sent, 1)
                previous = request

    def test_late_reply_in_the_buffer_is_discarded(self) -> None:
        """The reply to a request that already gave up arrives before the next request goes out, with the same shape
        as the next reply (Modbus RTU has no transaction number to tell them apart): the bus flushes its input before
        every request, so the next request returns its own data."""
        rng = random.Random(FRAME_SEED + 3)
        with virtual_clock():
            transport = GuardedTransport()
            bus = ModbusRTU(transport, retries=1)
            for kind in ("fc03", "fc17", "fc11", "iai", "iai2", "probe"):
                with self.subTest(kind=kind):
                    first = random_request(rng, kind, slave=5, rejection=False)
                    second = random_request(rng, kind, slave=5, rejection=False)
                    transport.responses = [None, None]
                    if first.retried:
                        with self.assertRaises(ModbusTimeout):
                            first.send(bus)
                    else:
                        self.assertIs(first.send(bus), False)
                    transport.arrive(first.reply)  # too late for the first request
                    transport.responses = [second.reply]
                    self.assertEqual(second.send(bus), second.result)

    def test_reply_to_the_previous_write_is_never_taken_as_the_answer(self) -> None:
        """A late echo of another write (another coil, register or block), or an FC03 / FC17 reply of the right length
        and a valid CRC whose byte count is wrong, comes in instead of the answer: the call raises ModbusFrameError
        and returns nothing."""
        rng = random.Random(FRAME_SEED + 4)
        with virtual_clock():
            for kind in ("fc05", "fc06", "fc10"):
                request = random_request(rng, kind, slave=7)
                other = random_request(rng, kind, slave=7)
                with self.subTest(kind=kind):
                    self.assertNotEqual(other.reply, request.reply)
                    bus = ModbusRTU(GuardedTransport([other.reply]), retries=0)
                    with self.assertRaises(ModbusFrameError):
                        request.send(bus)
            for kind in ("fc03", "fc17"):
                request = random_request(rng, kind, slave=7)
                body = request.reply[1:-2]
                wrong = build_frame(7, bytes([body[0], (body[1] + 2) % 256]) + body[2:])  # right length, wrong count
                with self.subTest(kind=kind):
                    bus = ModbusRTU(GuardedTransport([wrong]), retries=0)
                    with self.assertRaises(ModbusFrameError):
                        request.send(bus)

    def test_mismatched_echo_or_byte_count_is_retried(self) -> None:
        """ModbusRTU retries "after a timeout / CRC error / mismatched frame" (its docstring), and a wrong echo is a
        mismatched frame (errors.ModbusFrameError). A late echo of another write, then the right echo on the retry,
        should end in success after two frames, as a wrong slave or function does."""
        rng = random.Random(FRAME_SEED + 5)
        with virtual_clock():
            for kind in ("fc05", "fc06", "fc10", "fc03", "fc17"):
                request = random_request(rng, kind, slave=9)
                if kind in ("fc03", "fc17"):
                    body = request.reply[1:-2]
                    wrong = build_frame(9, bytes([body[0], (body[1] + 2) % 256]) + body[2:])
                else:
                    wrong = random_request(rng, kind, slave=9).reply
                transport = GuardedTransport([wrong + request.reply, request.reply])
                bus = ModbusRTU(transport, retries=2)
                with self.subTest(kind=kind):
                    try:
                        result = request.send(bus)
                    except ModbusFrameError as exc:
                        self.fail(f"raised after {len(transport.writes)} frame(s), retries left: {exc}")
                    self.assertEqual(result, request.result)
                    self.assertEqual(len(transport.writes), 2)


# =========================================================================== (a) the same on a line with timing


@dataclass
class Chunk:
    """Bytes that reach the PC ``delay_s`` after the request they answer, ``byte_s`` apart."""

    delay_s: float
    data: bytes
    byte_s: float = 0.0001  # about 115200 baud


class LineTransport:
    """A serial line on the virtual clock, with a numeric read timeout as a real port has (the bus then applies its
    response timeout). Each request is answered with its scripted chunks; ``reset_input_buffer()`` drops only the bytes
    that have arrived, the rest is still on its way."""

    def __init__(self, clock: VirtualClock, script: Iterable[list[Chunk]] = ()) -> None:
        self.clock = clock
        self.script = list(script)
        self.timeout = 0.2
        self.writes: list[bytes] = []
        self.reads = 0
        self._line: list[tuple[float, int]] = []  # (arrival time, byte), in arrival order

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))
        self.reads = 0
        for chunk in self.script.pop(0) if self.script else []:
            start = self.clock.now + chunk.delay_s
            self._line += [(start + i * chunk.byte_s, byte) for i, byte in enumerate(chunk.data)]
        self._line.sort(key=lambda item: item[0])

    def _arrived(self) -> int:
        return next((i for i, (t, _) in enumerate(self._line) if t > self.clock.now), len(self._line))

    def read(self, n: int) -> bytes:
        self.reads += 1
        if self.reads > READ_LIMIT:
            raise AssertionError(f"one request read {self.reads} times: its wait never ends")
        count = min(n, self._arrived())
        data = bytes(byte for _, byte in self._line[:count])
        del self._line[:count]
        return data

    def reset_input_buffer(self) -> None:
        del self._line[: self._arrived()]

    def close(self) -> None:
        pass


class SlowLineTest(unittest.TestCase):
    """ModbusRTU on a transport with a read timeout (the bus applies its response timeout), on the virtual clock."""

    @staticmethod
    def attempt_wait(bus: ModbusRTU, request: Request, frame: bytes) -> float:
        """How long one attempt of ``request`` (sent as ``frame``) waits for an answer that does not come."""
        if not request.retried:
            return request.wait_s  # probe(): its own timeout
        tout = bus.response_timeout(request.slave, frame[1:-2], request.reply_len)
        return max(tout, SLAVE_ID_TIMEOUT_S) if request.function == 0x11 else tout

    @staticmethod
    def request_frame(clock: VirtualClock, request: Request) -> bytes:
        """The frame ``request`` sends (sent once into silence)."""
        transport = LineTransport(clock)
        with contextlib.suppress(ModbusTimeout):
            request.send(ModbusRTU(transport, retries=0))
        return transport.writes[0]

    def test_reply_trickling_in_is_assembled(self) -> None:
        """A reply arriving one byte per 1.04 ms (9600 baud) is read whole: one frame sent, the right result."""
        rng = random.Random(FRAME_SEED + 10)
        with virtual_clock() as clock:
            for kind in REQUEST_KINDS:
                request = random_request(rng, kind, rejection=False)
                transport = LineTransport(clock, [[Chunk(0.003, request.reply, byte_s=0.00104)]])
                bus = ModbusRTU(transport, retries=2)
                with self.subTest(kind=kind):
                    self.assertEqual(request.send(bus), request.result)
                    self.assertEqual(len(transport.writes), 1)

    def test_silence_costs_the_response_timeout_per_attempt(self) -> None:
        """No answer: each attempt waits the response timeout (FC11 at least 0.65 s; probe() its own timeout, once),
        then ModbusTimeout. The request never waits longer than attempts x (timeout + gap + one read step)."""
        rng = random.Random(FRAME_SEED + 11)
        with virtual_clock() as clock:
            for kind in REQUEST_KINDS:
                request = random_request(rng, kind)
                transport = LineTransport(clock)
                bus = ModbusRTU(transport, retries=2)
                with self.subTest(kind=kind):
                    start = clock.now
                    if request.retried:
                        with self.assertRaises(ModbusTimeout) as caught:
                            request.send(bus)
                        self.assertIn("gave up after 3 attempts", caught.exception.__notes__)
                    else:
                        self.assertIs(request.send(bus), False)
                    attempts = len(transport.writes)
                    self.assertEqual(attempts, 3 if request.retried else 1)
                    wait = self.attempt_wait(bus, request, transport.writes[0])
                    self.assertGreaterEqual(clock.now - start, attempts * wait)
                    self.assertLessEqual(clock.now - start, attempts * (wait + 0.008))

    def test_reply_that_stalls_mid_frame(self) -> None:
        """Half a reply in time and the rest after the deadline: that attempt times out (the partial frame is logged as
        rx-partial), the late rest is flushed before the retry, and the retry's answer is read correctly."""
        rng = random.Random(FRAME_SEED + 12)
        with virtual_clock() as clock:
            for kind in ("fc03", "fc11", "fc17", "iai2"):
                request = random_request(rng, kind, rejection=False)
                frames: list[tuple[str, bytes]] = []
                half = len(request.reply) // 2
                transport = LineTransport(clock)
                bus = ModbusRTU(transport, retries=2, frame_logger=lambda d, f: frames.append((d, f)))
                deadline = self.attempt_wait(bus, request, self.request_frame(clock, request))
                transport.script = [
                    # the rest comes after the deadline, all at once, before the retry's flush (5 ms gap)
                    [Chunk(0.003, request.reply[:half]), Chunk(deadline + 0.0035, request.reply[half:], 0.0)],
                    [Chunk(0.003, request.reply)],
                ]
                with self.subTest(kind=kind):
                    self.assertEqual(request.send(bus), request.result)
                    self.assertEqual(len(transport.writes), 2)
                    self.assertIn(("rx-partial", request.reply[:half]), frames)

    def test_late_reply_to_the_previous_request_is_not_the_answer(self) -> None:
        """Request A gives up; its reply comes while request B (a read of another length) waits for its own. B rejects
        those bytes, retries, and returns its own data: A's data never comes out of B."""
        rng = random.Random(FRAME_SEED + 13)
        with virtual_clock() as clock:
            for case in range(20):
                a = random_request(rng, "fc03", slave=3)
                b = random_request(rng, "fc03", slave=3)
                if a.reply_len == b.reply_len:
                    continue  # same shape: no Modbus RTU master can tell them apart once B is sent
                transport = LineTransport(clock)
                bus = ModbusRTU(transport, retries=0)
                tout = self.attempt_wait(bus, a, self.request_frame(clock, a))
                transport.script = [  # each chunk arrives at once
                    [Chunk(tout + 0.020, a.reply, 0.0)],  # A's reply: about 14 ms into B's first attempt
                    [Chunk(0.045, b.reply, 0.0)],
                    [Chunk(0.003, b.reply, 0.0)],
                    [Chunk(0.003, b.reply, 0.0)],
                ]
                with self.subTest(case=case, a=a.name, b=b.name):
                    with self.assertRaises(ModbusTimeout):
                        a.send(bus)
                    bus.retries = 2
                    self.assertEqual(b.send(bus), b.result)
                    self.assertEqual(len(transport.writes), 3)  # A once; B twice: its first answer began with A's


# =========================================================================== (b) status decoding


def signed32(high: int, low: int) -> int:
    value = (high << 16) | low
    return value - (1 << 32) if value & 0x80000000 else value


def random_alarm_code(rng: random.Random) -> int:
    """An alarm code from every level range: none, cold start, message, unknown, operation cancelled, any word."""
    return rng.choice((
        0, R.ALARM_POWER_UP, rng.randint(0x001, 0x010), rng.randint(0x011, 0x07F),
        rng.choice(sorted(alarms.MESSAGE_CODES)), rng.randint(0x080, 0x0FF), 0x0EE, rng.getrandbits(16),
    ))  # fmt: skip


class StatusDecodeTest(unittest.TestCase):
    """Status.from_registers over random register words: no crash, and every flag agrees with its bit."""

    CASES = 3000

    def test_random_words(self) -> None:
        rng = random.Random(STATUS_SEED)
        for case in range(self.CASES):
            length = rng.choice((8, 14, 20, 22, rng.randint(8, 34)))
            words = [rng.getrandbits(16) for _ in range(length)]
            if rng.random() < 0.6:
                words[2] = random_alarm_code(rng)
            if rng.random() < 0.5:
                words[7] &= ~(R.DSSE_MOVE | R.DSSE_GHMS)  # at rest: the "moving" rule then rests on DSS1
            with self.subTest(case=case, words=[f"{w:04X}" for w in words]):
                st = Status.from_registers(words)
                self.check_flags(st, words)
                self.check_rcon(replace(st, line="rcon"), words)
                self.check_views(st, words)

    def test_short_block_is_refused(self) -> None:
        for length in range(R.STATUS_BLOCK_MIN_WORDS):
            with self.subTest(length=length), self.assertRaises(ValueError):
                Status.from_registers([0x3098] * length)

    def check_flags(self, st: Status, words: list[int]) -> None:
        n = len(words)
        code, dss1, dss2, dsse = words[2], words[5], words[6], words[7]
        stat = (words[8] << 16) | words[9] if n >= 14 else None
        self.assertEqual(st.position_mm, round(signed32(words[0], words[1]) * R.POSITION_SCALE, 2))
        self.assertEqual(st.raw["PNOW"], (words[0] << 16) | words[1])
        self.assertEqual(st.alarm_code, code)
        for flag, value in (
            (st.servo_on, dss1 & R.DSS1_SV), (st.homed, dss1 & R.DSS1_HEND), (st.in_position, dss1 & R.DSS1_PEND),
            (st.emergency_stop, dss1 & R.DSS1_EMGS), (st.paused, dss1 & R.DSS1_STP),
            (st.target_reached, dss1 & R.DSS1_TARGET_REACHED), (st.push_missed, dss1 & R.DSS1_PSFL),
            (st.safety_speed, dss1 & R.DSS1_SFTY), (st.modbus_enabled, dsse & R.DSSE_PMSS),
            (st.homing, dsse & R.DSSE_GHMS), (st.move_bit, dsse & R.DSSE_MOVE), (st.pushing, dsse & R.DSSE_PUSH),
            (st.excitation_done, dsse & R.DSSE_PSNS), (st.emg_input, dsse & R.DSSE_EMGP),
            (st.manual_mode, dsse & R.DSSE_RMDS), (st.enable_off, not dss2 & R.DSS2_ENBS),
            (st.push_contact, dss1 & R.DSS1_PEND and dsse & R.DSSE_PUSH_CONTACT),
            (st.drive_power_cut, dsse & R.DSSE_MPUV or (stat is not None and not stat & R.STAT_MPOW)),
        ):  # fmt: skip
            self.assertIs(flag, bool(value))
        # alarm levels (alarms.alarm_level): ALMH or a fault level is a fault, a message-level code never is, an
        # unknown level is one unless ALML is set; a warning is never a fault
        level = alarms.alarm_level(code)
        if dss1 & R.DSS1_ALMH or level in alarms.FAULT_LEVELS:
            self.assertTrue(st.alarm)
        elif code == 0 or level in (alarms.LEVEL_NONE, alarms.LEVEL_MESSAGE):
            self.assertFalse(st.alarm)
        else:
            self.assertIs(st.alarm, not dss1 & R.DSS1_ALML)
        self.assertIs(st.fault, st.alarm)
        self.assertIs(st.warning, not st.alarm and bool(dss1 & R.DSS1_ALML or (code and level == alarms.LEVEL_MESSAGE)))
        self.assertEqual(st.alarm_level, level if code else None)
        # motion: MOVE or GHMS is motion; otherwise "moving" needs the servo on without PEND / PSFL and no pause
        if dsse & (R.DSSE_MOVE | R.DSSE_GHMS):
            self.assertTrue(st.moving)
        elif st.moving:
            self.assertTrue(st.servo_on and not st.in_position and not st.push_missed and not st.paused)
        self.assertIs(st.idle, not st.moving and not st.paused and not st.homing)
        self.assertIs(st.ready, st.servo_on and st.homed and st.modbus_enabled and not st.alarm
                      and not st.emergency_stop and not st.paused and not st.homing)  # fmt: skip
        vnow = round(signed32(words[10], words[11]) * R.VELOCITY_SCALE, 2) if n >= 14 else None
        self.assertEqual(st.vnow_mm_s, vnow)
        self.assertEqual(st.velocity_mm_s, vnow if st.moving else 0.0)
        self.assertEqual(st.current_ma, signed32(words[12], words[13]) if n >= 14 else None)
        # flags of the longer blocks: None when their word was not read, never a guessed False
        for flag, value in ((st.auto_servo_off, R.STAT_ASOF), (st.nv_busy, R.STAT_AEEP), (st.battery_low, R.STAT_BATL)):
            self.assertEqual(flag, None if stat is None else bool(stat & value))
        zons = words[19] if n >= 20 else None
        for flag, value in ((st.at_backward_end, R.ZONS_LS0), (st.at_forward_end, R.ZONS_LS1),
                            (st.at_middle, R.ZONS_LS2)):  # fmt: skip
            self.assertEqual(flag, None if zons is None else bool(zons & value))
        poss, ssse = (words[20], words[21]) if n >= 22 else (None, None)
        self.assertEqual(st.position_complete_no, None if poss is None else poss & R.POSS_MASK)
        self.assertEqual(st.cold_start_alarm, None if ssse is None else bool(ssse & R.SSSE_ALMC))
        self.assertEqual(st.rtc_in_use, None if ssse is None else bool(ssse & R.SSSE_RTC))

    def check_rcon(self, st: Status, words: list[int]) -> None:
        """Behind an RCON gateway the bits its units leave undefined read None (DOC ME0413 p.5-15 to 5-19)."""
        n = len(words)
        self.assertIsNone(st.at_backward_end)
        self.assertIsNone(st.at_forward_end)
        self.assertIsNone(st.at_middle)
        self.assertIsNone(st.nv_busy)
        self.assertEqual(st.position_complete_no, words[20] & R.POSS_MASK_RCON if n >= 22 else None)
        self.assertIs(st.push_contact, st.in_position and bool(words[7] & R.DSSE_PUSH) and not st.push_missed)

    def check_views(self, st: Status, words: list[int]) -> None:
        """The JSON dict (CLI / GUI) and the unit-info view of the same words agree with the status."""
        data = json.loads(json.dumps(st.to_dict()))
        for key in ("moving", "alarm", "warning", "idle", "ready", "paused", "homing", "push_contact"):
            self.assertEqual(data[key], getattr(st, key), key)
        view = info.decode_monitor(st.raw)
        self.assertEqual(view["monitor"]["position_mm"], st.position_mm)
        self.assertEqual(view["io"]["lamps"]["alarm"], st.alarm)
        self.assertEqual(view["io"]["lamps"]["warning"], st.warning)
        for name, decoded in view["status_words"].items():
            if "bits" in decoded:  # every set bit shows once, named or as "b<n>"
                self.assertEqual(sum(1 << b["bit"] for b in decoded["bits"] if b["on"]), st.raw[name], name)
        if len(words) >= 20:
            lamps = view["io"]["lamps"]
            self.assertEqual((lamps["backward_end"], lamps["forward_end"], lamps["middle"]),
                             (st.at_backward_end, st.at_forward_end, st.at_middle))  # fmt: skip


# =========================================================================== (c) random command sequences


STROKE = (0.0, 150.0)  # the EC-DS4H slider's stroke; its soft limits are 0.30 mm wider on each side
MAX_SPEED_MM_S = 200.0
MAX_ACCEL_G = 0.5
MECHANICAL_END_MM = -2.0  # where the simulator's home return turns back (iai_actuator_driver.sim notes): past the soft limit
HOME_OVERSHOOT_MM = 0.05  # the same overshoot at the 30 mm/s of the home return, when it is paused or stopped
FAULT_CODES = (0x0A3, 0x0C1)  # operation-cancelled level: the unit stops and switches the servo off
WARNING_CODES = (0x04E,)  # message level (maintenance warning): the axis keeps running
WRITE_FUNCTIONS = frozenset({0x05, 0x06, 0x0F, 0x10, 0x17, 0x45, 0x48})
REFUSALS = (EcNotReadyError, EcAlarmError)
WAIT_ERRORS = (MotionTimeoutError, MotionIncompleteError, EcAlarmError, EcNotReadyError)


class BusWatch:
    """Frame logger that checks every status reply and every move command on the bus as it passes.

    Each read of the 0x9000 block (FC03, or the FC17 of a move with its own deceleration) is decoded: with HEND set the
    position must lie inside the soft limits; without it (homing, or a home return cut short) between the mechanical end
    and the upper soft limit. The target of each 0x9900 move block and 0x0F00 command profile must lie in the stroke.
    The bus swallows what a frame logger raises, so violations are collected for the test to check."""

    def __init__(self, soft_limits: tuple[float, float]) -> None:
        self.soft_limits = soft_limits
        self.violations: list[str] = []
        self.writes = 0  # write requests sent (FC05 / 06 / 0F / 10 / 17, IAI commands)
        self.samples = 0
        self._words = 0  # words of the 0x9000 block that the last request sent reads (0: none)

    def __call__(self, direction: str, frame: bytes) -> None:
        if direction == "tx":
            self._words, function = 0, frame[1]
            if function in WRITE_FUNCTIONS:
                self.writes += 1
            if function in (0x03, 0x10, 0x17) and len(frame) >= 8:
                address, count = struct.unpack(">HH", frame[2:6])
                if function == 0x03 and address == R.MONITOR_BASE:
                    self._words = count
                elif function == 0x10 and address == R.DIRECT_MOVE_BASE:
                    self.check_target(frame[7:11])
                elif function == 0x17:
                    self._words = count if address == R.MONITOR_BASE else 0
                    if struct.unpack(">H", frame[6:8])[0] == R.PROFILE_BASE:
                        self.check_target(frame[11:15])
        elif direction == "rx" and self._words >= R.STATUS_BLOCK_MIN_WORDS and frame[1] in (0x03, 0x17):
            st = Status.from_registers(struct.unpack(f">{self._words}H", frame[3:-2]))
            self.samples += 1
            lo, hi = self.soft_limits
            low = lo if st.homed else MECHANICAL_END_MM - HOME_OVERSHOOT_MM
            if not low - 1e-9 <= st.position_mm <= hi + 1e-9:
                self.violations.append(f"position {st.position_mm:.2f} mm (HEND {int(st.homed)}) outside "
                                       f"{low:.2f}..{hi:.2f} mm")  # fmt: skip

    def check_target(self, pcmd: bytes) -> None:
        target = R.from_i32_words(*struct.unpack(">HH", pcmd)) * R.POSITION_SCALE
        if not STROKE[0] - 1e-9 <= target <= STROKE[1] + 1e-9:
            self.violations.append(f"move sent with the target {target:.2f} mm, outside the stroke {STROKE}")


class _Records(logging.Handler):
    def __init__(self) -> None:
        super().__init__(logging.INFO)
        self.records: list[logging.LogRecord] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(record)


@contextlib.contextmanager
def captured_log(name: str = "iai_actuator_driver") -> Iterator[list[logging.LogRecord]]:
    """Collect the package's log records instead of printing them."""
    logger = logging.getLogger(name)
    handler = _Records()
    saved = logger.level, logger.propagate
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    try:
        yield handler.records
    finally:
        logger.removeHandler(handler)
        logger.level, logger.propagate = saved


class SequenceRun:
    """One seeded random command sequence on the simulated EC-DS4H slider, checked after every command.

    Before each command the status is read (``pre``). Expected outcomes follow the documented preconditions: a move
    needs the axis ready (servo on, homed, Modbus control, no alarm, not paused, not homing), a home return the same
    without "homed", a servo ON no alarm; a refusal is EcAlarmError while an alarm is active, else EcNotReadyError, and
    sends nothing. A target outside the stroke is SoftLimitError whatever the state, and sends nothing."""

    COMMANDS = {
        "move": 30, "move_outside": 8, "home": 6, "stop": 12, "pause": 6, "resume": 10, "reset_alarm": 6,
        "servo_on": 7, "servo_off": 3, "alarm": 3, "wait_move": 8, "idle": 4,
    }  # fmt: skip

    def __init__(self, test: unittest.TestCase, seed: int, strategy: str, clock: VirtualClock) -> None:
        self.test = test
        self.seed = seed
        self.rng = random.Random(seed)
        self.clock = clock
        self.sim = SimulatedEC(clock=clock.perf_counter)
        self.watch = BusWatch(self.sim.soft_limits_mm)
        self.ax = Elecylinder(ModbusRTU(self.sim, frame_logger=self.watch), stroke=STROKE,
                              max_speed_mm_s=MAX_SPEED_MM_S, max_accel_g=MAX_ACCEL_G, strategy=strategy,
                              poll_interval_s=0.05, owns_bus=True)  # fmt: skip
        self.handle: MoveHandle | None = None
        self.still_at: float | None = None  # where stop() (or an error's stop) left the axis: it stays there
        self.home_coil_on = False  # home(wait=False) leaves the HOME coil on until finish_home()
        self.history: list[str] = []
        self.counts: Counter[str] = Counter()

    def fail(self, text: str) -> None:
        recent = "\n  ".join(self.history[-8:])
        self.test.fail(f"seed {self.seed}: {text}\nlast commands:\n  {recent}")

    def expect(self, condition: object, text: str) -> None:
        if not condition:
            self.fail(text)

    def choose(self, pre: Status) -> str:
        """A random command, more often the one that makes the axis ready again (alarm reset, servo on, home, resume)
        and stop / pause while it moves; the refused commands still come up often."""
        weights = dict(self.COMMANDS)
        if pre.alarm:
            weights["reset_alarm"] *= 6
        elif not pre.servo_on:
            weights["servo_on"] *= 6
        elif not pre.homed and not pre.homing:
            weights["home"] *= 6
        if pre.paused:
            weights["resume"] *= 3
        if pre.moving:
            weights["stop"] *= 2
            weights["pause"] *= 2
            if self.handle is not None and not pre.paused:
                weights["wait_move"] *= 3
        return self.rng.choices(list(weights), list(weights.values()))[0]

    def run(self, steps: int) -> Counter[str]:
        for step in range(steps):
            self.check_still()
            pre = self.ax.status()
            if self.home_coil_on and not pre.homing:
                self.ax.finish_home()  # the home return has ended: release the HOME coil (see home())
                self.home_coil_on = False
            name = self.choose(pre)
            self.history.append(f"{step}: {name} (pos {pre.position_mm:.2f}, servo {int(pre.servo_on)}, homed "
                                f"{int(pre.homed)}, homing {int(pre.homing)}, paused {int(pre.paused)}, alarm "
                                f"{pre.alarm_code:#05x}, moving {int(pre.moving)})")  # fmt: skip
            try:
                getattr(self, f"do_{name}")(pre)
            except AssertionError:
                raise
            except Exception as exc:  # each command catches the outcomes its preconditions allow
                self.fail(f"{name} raised {exc!r}")
            self.check_position()
            self.expect(not self.watch.violations, "; ".join(self.watch.violations))
        self.ax.stop()
        self.ax.close()
        return self.counts

    # -- invariants

    def check_still(self) -> None:
        """After stop() the axis stands still until the next accepted motion command."""
        if self.still_at is not None:
            self.expect(not self.sim.moving, "the axis moves again after a stop, without a new motion command")
            self.expect(abs(self.sim.position_mm - self.still_at) < 1e-6,
                        f"the axis left its stop position {self.still_at:.3f} mm: now {self.sim.position_mm:.3f} mm")

    def check_position(self) -> None:
        lo, hi = self.sim.soft_limits_mm
        low = lo if self.sim.homed else MECHANICAL_END_MM - HOME_OVERSHOOT_MM
        pos = self.sim.position_mm
        self.expect(low - 1e-9 <= pos <= hi + 1e-9, f"position {pos:.3f} mm outside {low:.2f}..{hi:.2f} mm")

    def stopped(self, what: str) -> None:
        """The axis must stand still now and stay where it is."""
        self.expect(not self.sim.moving, f"the axis still moves after {what}")
        self.still_at = self.sim.position_mm

    def motion_started(self) -> None:
        self.still_at = None

    def refused(self, exc: EcDriverError, pre: Status, writes: int, what: str) -> None:
        """``exc`` must be the documented refusal for ``pre``, and nothing may have been written."""
        want = EcAlarmError if pre.alarm else EcNotReadyError
        self.expect(isinstance(exc, want), f"{what} refused with {exc!r}, expected {want.__name__}")
        self.expect(self.watch.writes == writes, f"{what} refused, but {self.watch.writes - writes} frame(s) written")
        self.counts["refused"] += 1

    # -- commands

    def do_move(self, pre: Status) -> None:
        rng = self.rng
        target = round(rng.uniform(*STROKE), 2)
        speed, accel = round(rng.uniform(15.0, MAX_SPEED_MM_S), 1), round(rng.uniform(0.1, MAX_ACCEL_G), 2)
        band, wait = round(rng.uniform(0.05, 1.0), 2), rng.random() < 0.5
        decel = round(rng.uniform(0.1, MAX_ACCEL_G), 2) if rng.random() < 0.25 else None  # the command-profile path
        self.history[-1] += f" -> move_to({target}, {speed}, {accel}, {band}, wait={wait}, decel_g={decel})"
        writes = self.watch.writes
        try:
            result = self.ax.move_to(target, speed, accel, band, wait=wait, decel_g=decel)
        except REFUSALS as exc:
            if pre.ready:
                self.fail(f"move_to refused on a ready axis: {exc!r}")
            self.refused(exc, pre, writes, "move_to")
            return
        self.expect(pre.ready or pre.homing, "move_to accepted on an axis that was not ready")
        self.motion_started()
        if not wait:
            self.expect(isinstance(result, MoveHandle), f"move_to(wait=False) returned {result!r}")
            self.handle = result  # type: ignore[assignment]
            self.counts["move started"] += 1
            return
        assert isinstance(result, Status)
        self.expect(abs(result.position_mm - target) <= band + 1e-6, f"move_to ended at {result.position_mm:.2f} mm")
        self.expect(abs(self.sim.position_mm - target) <= band + 1e-6, "the axis is not at the target")
        self.expect(not self.sim.moving, "move_to returned before the axis stood still")
        self.counts["move done"] += 1

    def do_move_outside(self, pre: Status) -> None:
        rng = self.rng
        target, speed, accel, band, decel = round(rng.uniform(*STROKE), 2), 50.0, 0.3, 0.1, None
        which = rng.choice(("below", "above", "nan", "inf", "speed", "accel", "decel", "band"))
        if which == "below":
            target = round(rng.uniform(-50.0, STROKE[0] - 0.01), 2)
        elif which == "above":
            target = round(rng.uniform(STROKE[1] + 0.01, 200.0), 2)
        elif which == "nan":
            target = math.nan
        elif which == "inf":
            target = rng.choice((math.inf, -math.inf))
        elif which == "speed":
            speed = round(rng.uniform(MAX_SPEED_MM_S + 0.1, 1000.0), 1)
        elif which == "accel":
            accel = round(rng.uniform(MAX_ACCEL_G + 0.01, 3.0), 2)
        elif which == "decel":
            decel = round(rng.uniform(MAX_ACCEL_G + 0.01, 3.0), 2)
        else:
            band = rng.choice((0.0, -1.0, 20.0))
        if decel is None and rng.random() < 0.25:
            decel = 0.2  # the command-profile path checks the same limits
        self.history[-1] += f" -> move_to({target}, {speed}, {accel}, {band}, decel_g={decel})"
        writes = self.watch.writes
        try:
            self.ax.move_to(target, speed, accel, band, wait=rng.random() < 0.5, decel_g=decel)
        except SoftLimitError:
            self.expect(self.watch.writes == writes, "SoftLimitError, but a frame was written")
            self.counts["soft limit"] += 1
            return
        self.fail(f"move_to outside the limits ({which}) was accepted")

    def do_home(self, pre: Status) -> None:
        wait = self.rng.random() < 0.5
        self.history[-1] += f" -> home(wait={wait})"
        able = pre.servo_on and pre.modbus_enabled and not pre.alarm and not pre.paused and not pre.homing
        writes = self.watch.writes
        try:
            result = self.ax.home(wait=wait)
        except REFUSALS as exc:
            if able:
                self.fail(f"home refused although the servo is on and nothing blocks it: {exc!r}")
            self.refused(exc, pre, writes, "home")
            return
        self.expect(able or pre.homing, "home accepted although the axis could not home")
        self.motion_started()
        if wait:
            assert result is not None
            self.expect(result.homed and not result.homing and abs(result.position_mm) <= 0.01,
                        f"home returned {result.position_mm:.2f} mm, HEND {int(result.homed)}")  # fmt: skip
            self.counts["home done"] += 1
        else:
            self.home_coil_on = True
            self.counts["home started"] += 1

    def do_stop(self, pre: Status) -> None:
        st = self.ax.stop()
        assert st is not None
        self.stopped("stop()")
        self.expect(abs(st.position_mm - self.sim.position_mm) < 0.01, "stop() returned another position")
        self.clock.sleep(self.rng.uniform(0.1, 1.0))
        self.check_still()
        self.counts["stop while moving" if pre.moving else "stop at rest"] += 1

    def do_pause(self, pre: Status) -> None:
        self.ax.pause()
        self.counts["pause"] += 1

    def do_resume(self, pre: Status) -> None:
        self.ax.resume()
        self.counts["resume"] += 1

    def do_reset_alarm(self, pre: Status) -> None:
        st = self.ax.reset_alarm()
        self.expect(st.alarm_code == 0 and not st.alarm, f"alarm {st.alarm_code:#05x} left after reset_alarm()")
        self.counts["alarm reset" if pre.alarm_code else "reset without alarm"] += 1

    def do_servo_on(self, pre: Status) -> None:
        writes = self.watch.writes
        try:
            st = self.ax.servo(True)
        except REFUSALS as exc:
            if not pre.alarm and not pre.homing:
                self.fail(f"servo(True) refused without an alarm: {exc!r}")
            self.refused(exc, pre, writes, "servo(True)")
            return
        self.expect(not pre.alarm, "servo(True) accepted while an alarm was active")
        assert st is not None
        self.expect(st.servo_on and self.sim.servo_on, "servo(True) returned, but the servo is off")
        self.counts["servo on"] += 1

    def do_servo_off(self, pre: Status) -> None:
        st = self.ax.servo(False)
        assert st is not None
        self.expect(not st.servo_on and not self.sim.servo_on, "servo(False) returned, but the servo is on")
        self.stopped("servo(False)")
        self.counts["servo off"] += 1

    def do_alarm(self, pre: Status) -> None:
        code = self.rng.choice(FAULT_CODES + WARNING_CODES)
        self.history[-1] += f" -> the unit raises {code:#05x}"
        self.sim.trigger_alarm(code)
        if code in FAULT_CODES:
            self.expect(not self.sim.servo_on, "a fault alarm left the servo on")
            self.stopped(f"alarm {code:#05x}")
        self.counts["alarm raised"] += 1

    def do_wait_move(self, pre: Status) -> None:
        handle, self.handle = self.handle, None
        if handle is None:
            self.do_idle(pre)
            return
        self.history[-1] += f" -> {handle!r}.wait()"
        try:
            st = handle.wait()
        except WAIT_ERRORS as exc:  # the wait stops the axis before it raises
            self.expect(getattr(exc, "stop_attempted", False), f"{exc!r} raised without a stop")
            self.stopped(f"{type(exc).__name__} from MoveHandle.wait()")
            self.counts[f"wait: {type(exc).__name__}"] += 1
            return
        self.expect(abs(st.position_mm - handle.target_mm) <= handle.band_mm + 1e-6,
                    f"MoveHandle.wait() returned at {st.position_mm:.2f} mm")  # fmt: skip
        self.expect(not self.sim.moving, "MoveHandle.wait() returned before the axis stood still")
        self.counts["wait done"] += 1

    def do_idle(self, pre: Status) -> None:
        self.clock.sleep(self.rng.uniform(0.0, 2.0))
        self.ax.status()
        self.counts["idle"] += 1


class CommandSequenceTest(unittest.TestCase):
    """Elecylinder on the simulator under seeded random command sequences."""

    STEPS = 300

    def test_random_sequences(self) -> None:
        """One sequence per command strategy: coils (the default), DRG1 register writes (whose stop is the coil-free
        substitute: pause, cancel, release) and IAI 0x45 commands."""
        for seed, strategy in SEQUENCES:
            with self.subTest(seed=seed, strategy=strategy), captured_log() as records:
                with virtual_clock() as clock:
                    run = SequenceRun(self, seed, strategy, clock)
                    counts = run.run(self.STEPS)
                self.assertGreater(run.watch.samples, 2 * self.STEPS)
                # the sequence did what it is for (the counts are the same on every run; these leave a margin)
                for name, least in (("move done", 10), ("move started", 10), ("soft limit", 10), ("refused", 20),
                                    ("stop while moving", 5), ("home done", 3), ("alarm reset", 3),
                                    ("wait done", 1)):  # fmt: skip
                    self.assertGreaterEqual(counts[name], least, f"{name}: {dict(counts)}")
                stuck = [r.getMessage() for r in records if "still moving" in r.getMessage()]
                self.assertEqual(stuck, [], "a stop did not bring the axis to rest")


class LateStopTest(unittest.TestCase):
    """stop() and pause() while a fast move brakes, on the simulated EC-DS4H slider (MAXV 700 mm/s)."""

    def test_stop_while_braking_stays_inside_the_soft_limits(self) -> None:
        """A move between the ends of the stroke, 0.00 and 150.00 mm, at 700 mm/s and 0.30 G, stopped or paused while it
        brakes: the axis must come to rest at or before the target, inside the soft limits (0.30 mm wider)."""
        for start, target in ((0.0, 150.0), (150.0, 0.0)):
            for command in ("stop", "pause"):
                for delay in (0.25, 0.30, 0.35, 0.40):  # the move brakes from about 0.23 s to 0.45 s after its start
                    with self.subTest(start=start, target=target, command=command, delay=delay), captured_log():
                        with virtual_clock() as clock:
                            sim = SimulatedEC(clock=clock.perf_counter, position_mm=start)
                            ax = Elecylinder(ModbusRTU(sim), stroke=STROKE, poll_interval_s=0.05)
                            handle = ax.move_to(target, 700.0, 0.3, wait=False)
                            assert isinstance(handle, MoveHandle)
                            clock.sleep(handle.sent_at + delay - clock.now)
                            if command == "stop":
                                ax.stop()
                            else:
                                ax.pause()
                            clock.sleep(2.0)
                            low, high = sorted((start, target))
                            self.assertGreaterEqual(sim.position_mm, low - 1e-6)
                            self.assertLessEqual(sim.position_mm, high + 1e-6)
                            if command == "pause":
                                ax.resume()
                                clock.sleep(2.0)
                                self.assertAlmostEqual(sim.position_mm, target, places=6)


if __name__ == "__main__":
    unittest.main()
