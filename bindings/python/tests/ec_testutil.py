"""Shared helpers for the iai_actuator_driver test suites (stdlib unittest only, no hardware)."""

from __future__ import annotations

import struct
import time
from collections.abc import Iterable

from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.modbus import ModbusRTU, build_frame
from iai_actuator_driver.sim import SimulatedEC


def hexbytes(text: str) -> bytes:
    """``"01 03 90 00"`` -> bytes."""
    return bytes.fromhex(text.replace(" ", ""))


class ScriptedTransport:
    """Transport that records requests and answers each with the next scripted response.

    A ``None`` entry means "no answer" (timeout). When the script runs out, nothing is answered.
    """

    def __init__(self, responses: Iterable[bytes | None] = ()) -> None:
        self.responses = list(responses)
        self.writes: list[bytes] = []
        self.write_times: list[float] = []
        self._buffer = bytearray()
        self.closed = False

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))
        self.write_times.append(time.perf_counter())
        if self.responses:
            response = self.responses.pop(0)
            if response is not None:
                self._buffer += response

    def read(self, n: int) -> bytes:
        chunk = bytes(self._buffer[:n])
        del self._buffer[:n]
        return chunk

    def reset_input_buffer(self) -> None:
        self._buffer.clear()

    def close(self) -> None:
        self.closed = True


class FakeClock:
    """Manually advanced clock for deterministic simulator tests."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class SteppedClockSim(SimulatedEC):
    """SimulatedEC on its own clock that advances ``step_s`` on every request (``SimulatedEC(step_s=...)``).

    Each driver poll then sees the axis exactly ``step_s`` later, whatever the host's speed, so
    tests of short-lived states (the in-band settling tail, a momentary contact during a push)
    are deterministic.
    """

    def __init__(self, step_s: float = 0.01, **kwargs: object) -> None:
        super().__init__(step_s=step_s, **kwargs)  # type: ignore[arg-type]


class FrameRecorder:
    """Frame logger collecting ``(direction, frame)`` pairs."""

    def __init__(self) -> None:
        self.frames: list[tuple[str, bytes]] = []

    def __call__(self, direction: str, frame: bytes) -> None:
        self.frames.append((direction, frame))

    @property
    def tx(self) -> list[bytes]:
        return [f for d, f in self.frames if d == "tx"]

    def clear(self) -> None:
        self.frames.clear()


def make_ec(
    *,
    time_scale: float = 50.0,
    strategy: str = "coil",
    max_speed_mm_s: float = 100.0,
    max_accel_g: float = 0.3,
    sim: SimulatedEC | None = None,
    **sim_kwargs: object,
) -> tuple[Elecylinder, SimulatedEC, FrameRecorder]:
    """Elecylinder on a fresh simulator (or ``sim``), with a frame recorder attached to the bus."""
    if sim is None:
        sim = SimulatedEC(time_scale=time_scale, **sim_kwargs)  # type: ignore[arg-type]
    recorder = FrameRecorder()
    bus = ModbusRTU(sim, frame_logger=recorder)
    ec = Elecylinder(
        bus,
        stroke=(0.0, 150.0),
        max_speed_mm_s=max_speed_mm_s,
        max_accel_g=max_accel_g,
        strategy=strategy,
        poll_interval_s=0.005,
        owns_bus=True,
    )
    return ec, sim, recorder


def response(slave: int, pdu_hex: str) -> bytes:
    """Build a response frame with a valid CRC from a PDU given in hex."""
    return build_frame(slave, hexbytes(pdu_hex))


def read_response(slave: int, words: list[int]) -> bytes:
    """FC03 response frame for ``words``."""
    return build_frame(slave, struct.pack(f">BB{len(words)}H", 3, 2 * len(words), *words))
