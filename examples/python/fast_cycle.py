"""A short repeated cycle with fast completion: the time around each move, and how to cut it.

The cycle is two moves near the middle of the stroke, A -> B -> A. It runs ``--cycles`` times with the default
completion rule, ``"safe"``: a move is done when the unit reports in position, the position is inside the band and it
has not changed over the settle time (0.1 s). Then ``ax.enable_fast()`` sets ``completion="fast"`` (done on the first
sample that is in position, inside the band and not moving) and starts the status monitor, a background thread whose
samples replace the status reads before and after each move. The same cycle runs again. Every cycle time is printed,
then both passes side by side.

Run it from the project folder::

    python examples/python/fast_cycle.py --sim                          # the simulator, no hardware
    python examples/python/fast_cycle.py --port COM3                    # one unit on COM3 (close IA-OS first)
    python examples/python/fast_cycle.py --port COM3 --cycles 20 --baud 230400

``--baud 115200`` or ``--baud 230400`` also raises the link rate for the fast pass, with the broadcast baud change
(both rates VERIFIED on the EC-R6S rod, 230400 also on the EC-GRBP8M gripper and the PCON-CB). The change is volatile: the unit is back at 38400 after a power cycle, and the next
``Axis.open`` finds the rate by itself.

Measured on the EC-GRBP8M gripper at 38400 baud, the driver spent about 0.38 s around each move with the defaults
(VERIFIED); fast completion, the monitor and a faster link cut that time (on the PCON-CB the median move went from
502 ms to 287 ms at 38400 baud and 254 ms at 230400, VERIFIED). The simulator answers at once, so on
``--sim`` the times show the completion rules and the simulated settling, not the time a frame takes on a serial
line.

Motion: ``--step`` apart (default 10 mm, at most a quarter of the stroke) at 20 mm/s (raised to the unit's documented
minimum speed where that is higher; ``--speed`` sets it): values for a first run on an unloaded unit. Raise them once
the setup is proven; the driver refuses anything outside the unit's own ranges before it is sent. The axis is homed
only if it reports no home; it then runs to its mechanical end, so keep the travel clear. At the end the servo is
switched off. On Ctrl+C or an error the axis gets a decel STOP and the servo is left as it is. The driver is not a
safety function: keep the hardware emergency stop within reach.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import statistics
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Axis, EcDriverError, EcNotReadyError, SimulatedEC  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

FIRST_RUN_SPEED = 20.0  # units/s (mm/s, deg/s on a rotary axis) when --speed is not given
G_MM_S2 = 9806.65  # 1 G in mm/s2
SIM_BAUDRATE = 38400  # the simulated line runs at the rate the units power up at


def cycle_points(ax: Axis, step: float) -> tuple[float, float]:
    """A and B: ``step`` apart (at most a quarter of the unit's range) around the middle of its own range."""
    position = ax.limits()["position"]
    lo, hi = position["min"], position["max"]
    if lo is None or hi is None:
        raise EcNotReadyError("the unit's position range is not known (no soft limits read): no positions to plan")
    middle = (lo + hi) / 2
    half = min(step, (hi - lo) / 4) / 2
    return round(middle - half, 2), round(middle + half, 2)


def first_run_speed(ax: Axis, requested: float | None) -> float:
    """``requested``, else 20 units/s raised to the unit's documented minimum speed (below it the unit may run
    unsteadily; the driver warns, never refuses) and kept under the unit's maximum."""
    if requested is not None:
        return requested
    speed = ax.limits()["speed"]
    return min(max(FIRST_RUN_SPEED, speed["recommended_min"] or 0.0), speed["max"])


def motion_time(distance: float, speed: float, accel_g: float) -> float:
    """Time of one trapezoidal move (triangular when it is too short to reach ``speed``): the motion alone."""
    accel = accel_g * G_MM_S2
    if distance >= speed * speed / accel:
        return distance / speed + speed / accel
    return 2 * math.sqrt(distance / accel)


def get_ready(ax: Axis, start: float, speed: float, args: argparse.Namespace) -> None:
    """Servo ON and home only when needed, then move to the start of the cycle."""
    st = ax.status()
    if not st.servo_on:
        print("servo ON")
        ax.servo(True)
    if not st.homed:
        print("home: the unit reports no home (HEND); it runs to its mechanical end")
        ax.home()
    ax.move_to(start, speed_mm_s=speed, accel_g=args.accel, band_mm=args.band)


def run_pass(ax: Axis, label: str, a: float, b: float, speed: float, args: argparse.Namespace) -> list[float]:
    """``--cycles`` cycles A -> B -> A, each move done by the axis's completion rule. Returns the cycle times in s."""
    monitor = "on" if ax.monitor is not None and ax.monitor.running else "off"
    print(f"\n{label} pass: completion {ax.completion!r}, status monitor {monitor}, {ax.baudrate or 'no'} baud")
    times: list[float] = []
    for n in range(1, args.cycles + 1):
        started = time.perf_counter()
        ax.move_to(b, speed_mm_s=speed, accel_g=args.accel, band_mm=args.band)
        st = ax.move_to(a, speed_mm_s=speed, accel_g=args.accel, band_mm=args.band)
        times.append(time.perf_counter() - started)
        print(f"  cycle {n:3d}: {times[-1]:.3f} s (back at {st.position_mm:.2f} {ax.units})")
    return times


def print_summary(passes: dict[str, list[float]], motion: float, band: float, units: str) -> None:
    """Both passes side by side, against the time of the motion alone."""
    print(f"\ncycle times in s; the motion alone takes about {motion:.3f} s per cycle (trapezoid from distance, "
          "speed and acceleration)")
    print("  pass    mean     min     max   above the motion")
    for label, times in passes.items():
        mean = statistics.fmean(times)
        above = round(mean - motion, 3) + 0.0  # + 0.0: no "-0.000"
        print(f"  {label:<5} {mean:7.3f} {min(times):7.3f} {max(times):7.3f}   {above:7.3f}")
    saved = statistics.fmean(passes["safe"]) - statistics.fmean(passes["fast"])
    print(f"fast completion: {saved:.3f} s less per cycle ({saved / 2:.3f} s per move). It returns on the first sample "
          f"in position inside the band, so the axis can still settle by up to the band ({band:g} {units}) afterwards")


def run(ax: Axis, args: argparse.Namespace) -> None:
    """The whole example on an open axis."""
    info = ax.connect()  # identity and the unit's own ranges; PC control is taken while the axis is at rest
    print(f"{info.model} ({info.product_family}, {ax.kind}), serial {info.serial_number}, {ax.baudrate or 'no'} baud")
    a, b = cycle_points(ax, args.step)
    speed = first_run_speed(ax, args.speed)
    units = ax.units
    get_ready(ax, a, speed, args)
    motion = 2 * motion_time(abs(b - a), speed, args.accel)
    print(f"cycle: {a:.2f} -> {b:.2f} -> {a:.2f} {units} at {speed:g} {units}/s, {args.accel:g} G, "
          f"band {args.band:g} {units}")

    passes = {"safe": run_pass(ax, "safe", a, b, speed, args)}
    setup: dict[str, Any] = ax.enable_fast(baudrate=args.baud)  # baudrate None keeps the rate
    print(f"\nenable_fast(): completion {setup['completion']!r}, status monitor {'on' if setup['monitor'] else 'off'}, "
          f"{setup['baudrate'] or 'no'} baud")
    for note in setup["notes"]:
        print(f"  note: {note}")
    passes["fast"] = run_pass(ax, "fast", a, b, speed, args)
    ax.disable_fast()  # completion "safe" again and the monitor stopped; the baud rate stays as it is
    print_summary(passes, motion, args.band, units)

    if args.baud is not None and ax.baudrate == args.baud:
        print(f"the unit stays at {args.baud} baud until its next power cycle (then 38400); Axis.open finds the rate "
              "by itself")
    print("servo OFF")
    ax.servo(False)


def stop_axis(ax: Axis) -> None:
    """Decel STOP (Ctrl+C or an error). A STOP that cannot be sent is reported: the unit has no communication
    watchdog, so a move it is running then runs to its end."""
    try:
        ax.stop()
    except (EcDriverError, OSError) as exc:
        print(f"decel STOP not sent: {type(exc).__name__}: {exc}")
    else:
        print("decel STOP sent; the servo is left as it is")


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="iai-actuator-driver example: a short repeated cycle with fast completion (unofficial; not an IAI product)"
    )
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", metavar="COMx", help="serial port of the unit, e.g. COM3 (close IA-OS first)")
    where.add_argument("--sim", action="store_true", help="use the simulator instead of hardware")
    parser.add_argument("--slave", type=int, default=1, help="Modbus address = axis number + 1 (default 1)")
    parser.add_argument("--cycles", type=positive_int, default=5, metavar="N", help="cycles per pass (default 5)")
    parser.add_argument(
        "--speed", type=float, default=None,
        help="mm/s (deg/s on a rotary axis); default 20, raised to the unit's documented minimum where that is higher",
    )  # fmt: skip
    parser.add_argument("--accel", type=float, default=0.1, help="acceleration in G (default 0.1)")
    parser.add_argument("--band", type=float, default=0.1, help="positioning band in mm or deg (default 0.1)")
    parser.add_argument(
        "--step", type=float, default=10.0,
        help="distance between A and B in mm or deg (default 10; at most a quarter of the stroke)",
    )  # fmt: skip
    parser.add_argument(
        "--baud", type=int, choices=(115200, 230400), default=None,
        help="also raise the link to this rate for the fast pass (volatile: 38400 again after a power cycle)",
    )  # fmt: skip
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="slider",
        help="with --sim: the simulated unit (default slider)",
    )  # fmt: skip
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="  [driver %(levelname)s] %(message)s", stream=sys.stdout)
    if sys.platform == "win32":  # Ctrl+Break ends the program like Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    port: Any = args.port
    if args.sim:
        port = SimulatedEC(slave=args.slave, profile=args.sim_profile, baudrate=SIM_BAUDRATE)
    try:
        with Axis.open(port, slave=args.slave) as ax:  # the baud rate is found by itself
            try:
                run(ax, args)
            except BaseException:
                stop_axis(ax)  # STOP first; the axis does it again when the with block ends on the error
                raise
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("interrupted", file=sys.stderr)
        return 130
    except (EcDriverError, OSError, ValueError) as exc:
        report_error(exc)
        return 2
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
