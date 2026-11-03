"""A small machine sequence, written the way a PLC program runs one.

The program is a step sequence (a state machine) driven by a fixed scan. Every scan (``--scan``, default 20 ms) it
reads the input image (the status of every axis, kept fresh in the background by the axes' status monitors), checks
the alarm conditions, runs the active step and gives the next commands. Moves start with ``wait=False`` and never block
a scan; every step has a step timer, and a counter counts the finished cycles.

    START     servo ON; home the axes that report no home (HEND 0)    once, before the first scan
    TO_A      every axis toward its position A                        -> WORK_A when all are in position
    WORK_A    dwell for --dwell s (the process at A)                  -> TO_B
    TO_B      every axis toward its position B                        -> WORK_B when all are in position
    WORK_B    dwell; count the cycle                                  -> TO_A, or TO_REST after --cycles
    TO_REST   every axis back to the middle of its stroke             -> END: servo OFF

Alarm: an axis alarm, an emergency stop, a lost link or a step timer that runs out stops every axis
(``cell.stop_all``), reports where each axis is and why, and ends the program with exit code 1. Ctrl+C is the
operator's STOP: every axis gets a decel STOP and the program ends with exit code 130. In both cases the servos are
left as they are (a vertical axis without a brake can drop when its servo goes off).

Run it from the project folder::

    python examples/python/machine_sequence.py --sim                     # three simulated axes, three cycles
    python examples/python/machine_sequence.py --sim --sim-alarm 3       # a simulated axis alarm 3 s into the sequence
    python examples/python/machine_sequence.py --port COM3 --port COM4 --cycles 10

Positions A and B lie around the middle of each unit's own stroke, ``--step`` apart (default 10 mm, at most a quarter
of the stroke). The speed is 20 mm/s, raised to a unit's documented minimum speed where that is higher (``--speed``
sets it): values for a first run on unloaded units. Homing runs an axis to its mechanical end: keep the travel clear.
The driver is not a safety function: keep the hardware emergency stop within reach.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, NamedTuple

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Cell, EcDriverError, EcNotReadyError, MoveHandle, SimLine, Status, sim_ports  # noqa: E402
from iai_actuator_driver.alarms import alarm_info, alarm_name  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

FIRST_RUN_SPEED = 20.0  # units/s (mm/s, deg/s on a rotary axis) when --speed is not given
SIM_ALARM_CODE = 0x0D8  # simulator: "Deviation error", the alarm of a blocked axis


class Station(NamedTuple):
    """The positions of one axis in the sequence, all inside its own range."""

    a: float
    b: float
    rest: float


class SequenceAlarm(Exception):
    """An alarm condition the sequence detects itself: an axis alarm or emergency stop, a lost link, a step timer."""


class LinkFaults:
    """``Cell.open(on_fault=...)``: records the axes whose link failed. It may be called from a driver thread, so it
    only records; the next scan raises the alarm."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._faults: list[str] = []

    def __call__(self, name: str, kind: str, error: BaseException | None) -> None:
        with self._lock:
            self._faults.append(f"{name} {kind}")

    def first(self) -> str | None:
        with self._lock:
            return self._faults[0] if self._faults else None


class MachineSequence:
    """The step sequence on a cell. :meth:`run` does the start-up, then one :meth:`scan` every ``scan_s`` until END."""

    def __init__(
        self,
        cell: Cell,
        stations: Mapping[str, Station],
        speeds: Mapping[str, float],
        faults: LinkFaults,
        *,
        cycles: int,
        accel: float,
        dwell_s: float,
        scan_s: float,
    ) -> None:
        self.cell = cell
        self.stations = dict(stations)
        self.speeds = dict(speeds)
        self.faults = faults
        self.cycles = cycles
        self.accel = accel
        self.dwell_s = dwell_s
        self.scan_s = scan_s
        self.step = "START"
        self.cycle = 0  # the cycle counter: finished cycles
        self.moves: dict[str, MoveHandle] = {}
        self.started = time.perf_counter()
        self.step_started = self.started
        self.step_limit_s: float | None = None  # the step timer; None: no limit
        self.cycle_started = self.started
        self.scans = 0
        self.longest_scan_s = 0.0

    def run(self) -> None:
        """START, then the scans at a fixed period until END. Raises :class:`SequenceAlarm` or the driver's error."""
        self.start_up()
        next_scan = time.perf_counter()
        while self.step != "END":
            began = time.perf_counter()
            self.scan()
            self.scans += 1
            self.longest_scan_s = max(self.longest_scan_s, time.perf_counter() - began)
            next_scan += self.scan_s
            delay = next_scan - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_scan = time.perf_counter()  # the scan took longer than its period: the next one starts at once

    def start_up(self) -> None:
        """START: servo ON, then home the axes that report no home. Blocking group commands, before the first scan."""
        print("START: servo ON")
        self.cell.servo_all(True)
        homes = [name for name, st in self.cell.status_all().items() if not st.homed]
        if homes:
            print(f"START: home {', '.join(homes)} (each runs to its mechanical end)")
            self.cell.home_all(homes)
        self.cycle_started = time.perf_counter()
        self.start_moves("TO_A", "a")

    def scan(self) -> None:
        """One scan: read the input image, check the alarm conditions, run the active step."""
        image = self.cell.status_all()  # one status per axis: the monitors' latest samples, no wait on the bus
        self.check(image)
        if self.step in ("TO_A", "TO_B", "TO_REST"):
            done = [self.move_done(name, move) for name, move in self.moves.items()]  # every axis, every scan
            if all(done):
                self.arrived()
        elif time.perf_counter() - self.step_started >= self.dwell_s:  # WORK_A or WORK_B: the dwell is over
            if self.step == "WORK_A":
                self.start_moves("TO_B", "b")
            else:
                self.count_cycle()

    @staticmethod
    def move_done(name: str, move: MoveHandle) -> bool:
        """One non-blocking check of one move by the driver's completion rule. ``done()`` raises on an alarm, an
        emergency stop or a servo that went off (that axis is stopped); the alarm names the axis."""
        try:
            return move.done()
        except EcDriverError as exc:
            raise SequenceAlarm(f"{name}: {type(exc).__name__}: {exc}") from exc

    def check(self, image: Mapping[str, Status]) -> None:
        """The alarm conditions, every scan: a lost link, an axis alarm or emergency stop, the step timer."""
        fault = self.faults.first()
        if fault is not None:
            raise SequenceAlarm(f"link lost: {fault}")
        for name, st in image.items():
            if st.alarm:
                code = f"0x{st.alarm_code:03X} {alarm_name(st.alarm_code)}" if st.alarm_code else "(no code)"
                raise SequenceAlarm(f"{name}: alarm {code}")
            if st.emergency_stop:
                raise SequenceAlarm(f"{name}: emergency stop")
        if self.step_limit_s is not None and time.perf_counter() - self.step_started > self.step_limit_s:
            raise SequenceAlarm(f"step {self.step} not done within its step timer ({self.step_limit_s:.1f} s)")

    def enter(self, step: str, limit_s: float | None = None) -> None:
        """Make ``step`` the active step and restart the step timer."""
        print(f"[{time.perf_counter() - self.started:6.2f} s] {self.step} -> {step} (cycles done: {self.cycle})")
        self.step = step
        self.step_started = time.perf_counter()
        self.step_limit_s = limit_s

    def start_moves(self, step: str, position: str) -> None:
        """Start every axis toward its ``position`` (``"a"``, ``"b"`` or ``"rest"``) without waiting, then enter
        ``step``. Every target is checked first, so nothing moves when one is refused. The step timer is the slowest
        move's own time limit (twice its computed time, plus 2 s)."""
        targets = {name: getattr(station, position) for name, station in self.stations.items()}
        for name, target in targets.items():
            self.cell[name].check_move(target, speed_mm_s=self.speeds[name], accel_g=self.accel)
        self.moves = {
            name: self.cell[name].move_to(target, speed_mm_s=self.speeds[name], accel_g=self.accel, wait=False)
            for name, target in targets.items()
        }
        self.enter(step, limit_s=max(move.timeout for move in self.moves.values()))

    def arrived(self) -> None:
        """Every axis is in position: the next step."""
        if self.step == "TO_A":
            self.enter("WORK_A")
        elif self.step == "TO_B":
            self.enter("WORK_B")
        else:
            print("TO_REST done: servo OFF")
            self.cell.servo_all(False)
            self.enter("END")

    def count_cycle(self) -> None:
        """End of WORK_B: count the cycle, then the next one or the way to rest."""
        self.cycle += 1
        now = time.perf_counter()
        print(f"cycle {self.cycle}/{self.cycles} done in {now - self.cycle_started:.2f} s")
        self.cycle_started = now
        if self.cycle >= self.cycles:
            self.start_moves("TO_REST", "rest")
        else:
            self.start_moves("TO_A", "a")


def plan_station(limits: Mapping[str, Any], step: float) -> Station:
    """A and B ``step`` apart (at most a quarter of the range) around the middle of the unit's own range."""
    lo, hi = limits["position"]["min"], limits["position"]["max"]
    if lo is None or hi is None:
        raise EcNotReadyError("the unit's position range is not known (no soft limits read): no positions to plan")
    middle = round((lo + hi) / 2, 2)
    half = round(min(step, (hi - lo) / 4) / 2, 2)
    return Station(a=round(middle - half, 2), b=round(middle + half, 2), rest=middle)


def first_run_speed(limits: Mapping[str, Any], requested: float | None) -> float:
    """``requested``, else 20 units/s raised to the unit's documented minimum speed (below it the unit may run
    unsteadily; the driver warns, never refuses) and kept under the unit's maximum."""
    if requested is not None:
        return requested
    speed = limits["speed"]
    return min(max(FIRST_RUN_SPEED, speed["recommended_min"] or 0.0), speed["max"])


def describe(cell: Cell, stations: Mapping[str, Station], speeds: Mapping[str, float]) -> None:
    print(f"cell: {len(cell)} {'axis' if len(cell) == 1 else 'axes'}")
    for name in cell.names:
        units, st = cell.units_of(name), stations[name]
        print(f"  {name:<9} {cell.info[name].model} ({cell[name].kind}): A {st.a:.2f}, B {st.b:.2f}, rest "
              f"{st.rest:.2f} {units} at {speeds[name]:g} {units}/s")


def report_axes(cell: Cell) -> None:
    """A fresh full status read of every axis: position, servo and alarm, with what the driver knows of the alarm."""
    for name, st in cell.status_all(return_exceptions=True, monitor_max_age_s=None).items():
        if isinstance(st, BaseException):
            print(f"  {name}: no status ({type(st).__name__}: {st})")
            continue
        text = f"  {name}: at {st.position_mm:.2f} {cell.units_of(name)}, servo {'ON' if st.servo_on else 'OFF'}"
        if st.alarm_code:
            about = alarm_info(st.alarm_code, family=cell[name].product_family)
            text += f", alarm 0x{st.alarm_code:03X} {about['name']}"
            if about["fault"]:
                text += f", {about['level_text']}: {about['remedy']}"
            else:
                text += " (a warning: the axis runs on)"
        elif st.alarm or st.warning:  # the bit can be set with no code (code 0)
            text += f", {'alarm' if st.alarm else 'warning'} (no code)"
        print(text)


def stop_and_report(cell: Cell, sequence: MachineSequence, reason: str) -> None:
    """The alarm handler, also the operator's STOP: a decel STOP to every axis first, then the report."""
    print(f"\n{reason}\n  step {sequence.step}, cycles done: {sequence.cycle}; STOP to every axis")
    for name, result in cell.stop_all(return_exceptions=True).items():
        if isinstance(result, BaseException):
            print(f"  {name}: STOP not confirmed ({type(result).__name__}: {result}); the unit has no communication "
                  "watchdog: a move it is running runs to its end")
    report_axes(cell)
    if cell.faults:
        print(f"  link faults: {', '.join(f'{name} {kind}' for name, kind in cell.faults.items())}")


def raise_sim_alarm_later(lines: Mapping[str, SimLine], after_s: float) -> threading.Timer:
    """Simulator only: alarm 0x0D8 on the first unit of the last simulated port, ``after_s`` from now."""
    line = list(lines.values())[-1]
    unit = line[line.slaves[0]]

    def fire() -> None:
        print(f"  [simulator] alarm 0x{SIM_ALARM_CODE:03X} on {line.name}:{unit.slave}")
        unit.trigger_alarm(SIM_ALARM_CODE)

    timer = threading.Timer(after_s, fire)
    timer.daemon = True
    timer.start()
    return timer


def port_names(values: Sequence[str]) -> list[str]:
    """``--port COM3 --port COM4`` or ``--port COM3,COM4`` -> ``["COM3", "COM4"]``."""
    return [name.strip() for value in values for name in value.split(",") if name.strip()]


def report_error(exc: BaseException) -> None:
    sys.stdout.flush()  # keep the order when both streams go to one file
    print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)


def positive_int(text: str) -> int:
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError(f"must be 1 or more, got {value}")
    return value


def positive_float(text: str) -> float:
    value = float(text)
    if not value > 0:
        raise argparse.ArgumentTypeError(f"must be more than 0, got {text}")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="iai-actuator-driver example: a machine sequence run like a PLC program (unofficial; not an IAI product)"
    )
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument(
        "--port", action="append", metavar="COMx",
        help="a serial port with one or more axes; repeat it (--port COM3 --port COM4) or list them (COM3,COM4). "
        "Close IA-OS first",
    )  # fmt: skip
    where.add_argument("--sim", action="store_true", help="simulated ports instead of hardware")
    parser.add_argument("--cycles", type=positive_int, default=3, metavar="N", help="cycles to run (default 3)")
    parser.add_argument(
        "--speed", type=float, default=None,
        help="mm/s (deg/s on a rotary axis); default 20, raised to a unit's documented minimum where that is higher",
    )  # fmt: skip
    parser.add_argument("--accel", type=float, default=0.1, help="acceleration in G (default 0.1)")
    parser.add_argument(
        "--step", type=float, default=10.0,
        help="distance between A and B in mm or deg (default 10; at most a quarter of a stroke)",
    )  # fmt: skip
    parser.add_argument("--dwell", type=float, default=0.3, metavar="S", help="dwell at A and at B in s (default 0.3)")
    parser.add_argument(
        "--scan", type=positive_float, default=0.02, metavar="S", help="scan period in s (default 0.02)"
    )
    parser.add_argument(
        "--sim-axes", type=int, default=3, choices=range(1, 17), metavar="N",
        help="with --sim: simulated ports, one axis each (1..16, default 3)",
    )  # fmt: skip
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="slider",
        help="with --sim: the simulated unit (default slider)",
    )  # fmt: skip
    parser.add_argument(
        "--sim-alarm", type=float, default=None, metavar="S",
        help="with --sim: alarm 0x0D8 (deviation error, as when an axis is blocked) on the last simulated axis S "
        "seconds after the sequence starts",
    )  # fmt: skip
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="  [driver %(levelname)s] %(message)s", stream=sys.stdout)
    if sys.platform == "win32":  # Ctrl+Break ends the program like Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    lines = sim_ports(args.sim_axes, profile=args.sim_profile) if args.sim else None
    ports: Any = lines if lines is not None else port_names(args.port)
    faults = LinkFaults()
    try:
        with Cell.open(ports, on_fault=faults) as cell:  # stop_on_fault: a lost link stops every other axis at once
            limits = cell.limits_all()
            stations = {name: plan_station(limits[name], args.step) for name in cell.names}
            speeds = {name: first_run_speed(limits[name], args.speed) for name in cell.names}
            describe(cell, stations, speeds)
            cell.start_monitors()  # the input image: each axis's status monitor reads its status in the background
            sequence = MachineSequence(cell, stations, speeds, faults, cycles=args.cycles, accel=args.accel,
                                       dwell_s=args.dwell, scan_s=args.scan)  # fmt: skip
            timer = None
            if lines is not None and args.sim_alarm is not None:
                timer = raise_sim_alarm_later(lines, args.sim_alarm)
            try:
                sequence.run()
            except KeyboardInterrupt:
                stop_and_report(cell, sequence, "operator STOP (Ctrl+C)")
                return 130
            except (SequenceAlarm, EcDriverError, OSError) as exc:
                reason = str(exc) if isinstance(exc, SequenceAlarm) else f"{type(exc).__name__}: {exc}"
                stop_and_report(cell, sequence, f"ALARM: {reason}")
                return 1
            except Exception:
                stop_and_report(cell, sequence, "ALARM: an error in the program itself (traceback below)")
                raise
            finally:
                if timer is not None:
                    timer.cancel()
            print(f"\n{sequence.cycle} cycles in {time.perf_counter() - sequence.started:.1f} s; {sequence.scans} scans "
                  f"every {args.scan * 1000:g} ms, the longest took {sequence.longest_scan_s * 1000:.1f} ms")
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("interrupted", file=sys.stderr)
        return 130
    except (EcDriverError, OSError, ValueError) as exc:
        report_error(exc)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
