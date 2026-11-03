"""First move with one axis: open the port, connect, servo ON, home, move, read the status, servo OFF.

The order a program follows with one unit (any ELECYLINDER or CON-family controller): open the port (the baud rate is
found automatically), connect (identity and the unit's own ranges; Modbus control is taken while the axis is at rest),
servo ON, a home return when the axis is not homed, an absolute move and two relative moves that each wait until the
axis is in position, a status read, servo OFF. Every target and speed is checked against the unit's own ranges
(``ax.limits()``) before anything is sent; a value outside them raises ``SoftLimitError`` and nothing moves.

Run it from the project folder (close IA-OS first: only one program can use the serial port)::

    python examples/python/first_move.py --sim                      # the simulator (an ELECYLINDER slider), no hardware
    python examples/python/first_move.py --sim --sim-profile rod    # another simulated unit
    python examples/python/first_move.py --port COM3                # the unit on COM3, address 1
    python examples/python/first_move.py --port COM3 --home         # a home return even when the status says homed

Motion: to the middle of the stroke, then 10 mm further (at most a quarter of the stroke) and back, at 20 mm/s (deg/s
on a rotary axis) or at the unit's documented minimum speed when that is higher. The home return runs only when the
status says the axis is not homed, or with ``--home``: the axis runs to its mechanical end, so keep the travel clear.
Test a new setup unloaded, and keep a hand on the emergency stop.

End: the servo is switched OFF (a vertical axis without a brake can fall when the servo goes off: support it first),
and Modbus control is handed back to PIO when PIO had it at the start. On Ctrl+C or an error after a motion command a
decel STOP is sent before the port is closed (by the driver inside a move, or when the ``with`` block is left); the
servo is then left as it is.

Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Axis, EcAlarmError, EcDriverError, SimulatedEC, Status  # noqa: E402
from iai_actuator_driver.alarms import alarm_info  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

BANNER = "iai-actuator-driver example: first move (UNOFFICIAL - not an IAI product)"
SPEED = 20.0  # units/s: the default speed for a first test (GETTING_STARTED.md: unloaded, mid-stroke, 20 mm/s or less)


def positive(text: str) -> float:
    """A finite number above 0 (argparse type)."""
    try:
        value = float(text)
    except ValueError:
        value = math.nan
    if not (math.isfinite(value) and value > 0):
        raise argparse.ArgumentTypeError(f"a number above 0 is needed, got {text!r}")
    return value


def build_parser() -> argparse.ArgumentParser:
    """Command-line options: ``--port COM3`` or ``--sim`` (one of them is required)."""
    parser = argparse.ArgumentParser(description=BANNER)
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial port, e.g. COM3 (close IA-OS first)")
    where.add_argument("--sim", action="store_true", help="use the built-in simulator instead of hardware")
    parser.add_argument("--slave", type=int, default=1, help="Modbus address = axis number + 1 (default 1)")
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="slider",
        help="simulated unit with --sim (default slider)",
    )  # fmt: skip
    parser.add_argument(
        "--speed", type=positive, default=None,
        help=f"speed in mm/s, deg/s on a rotary axis (default {SPEED:g}, or the unit's documented minimum if higher)",
    )  # fmt: skip
    parser.add_argument(
        "--distance", type=positive, default=10.0,
        help="the step out and back in mm, deg on a rotary axis (default 10; at most a quarter of the stroke)",
    )  # fmt: skip
    parser.add_argument("--home", action="store_true", help="run a home return even when the status says homed")
    return parser


def open_axis(args: argparse.Namespace) -> Axis:
    """The axis on ``--port`` (baud rate found automatically), or a simulated unit with ``--sim``."""
    if args.sim:
        # pmsl=False: PIO in control, as after a power-up, so connect() takes Modbus control as it does on hardware.
        return Axis.open(SimulatedEC(slave=args.slave, profile=args.sim_profile, pmsl=False), slave=args.slave)
    return Axis.open(args.port, slave=args.slave)


def show(label: str, text: str) -> None:
    """One line of output: a short label, then the text."""
    print(f"{label:<10} {text}")


def describe(st: Status, units: str) -> str:
    """The status in one line: position and the flags that matter around a move."""
    flags = [
        name
        for name, on in (
            ("servo ON", st.servo_on), ("homed", st.homed), ("in position", st.in_position), ("moving", st.moving),
            ("PC control", st.modbus_enabled), ("emergency stop", st.emergency_stop),
        )
        if on
    ]  # fmt: skip
    text = f"position {st.position_mm:.2f} {units}, " + (", ".join(flags) or "no flags set")
    if st.alarm or st.warning:  # the bit can be set with no code (code 0)
        text += f"; {'alarm' if st.alarm else 'warning'}"
        if st.alarm_code:
            alarm = alarm_info(st.alarm_code)
            text += f" {alarm['hex']} {alarm['name']}"
        else:
            text += " (no code)"
    return text


def pick_speed(requested: float | None, limits: dict[str, Any]) -> float:
    """``requested``, else the default speed or the unit's documented minimum speed when that is higher (below it a
    unit may run noisily or unsteadily; the driver only warns)."""
    if requested is not None:
        return requested
    minimum = limits["speed"]["recommended_min"]
    return max(SPEED, minimum) if minimum else SPEED


def first_move(ax: Axis, args: argparse.Namespace) -> None:
    """Connect, servo ON, home if needed, move to mid-stroke, step out and back, read the status, servo OFF."""
    found = ax.status()  # read-only: the state the unit was in before this program sent anything
    info = ax.connect()  # identity and ranges; Modbus control is taken here, at rest only
    units = ax.units  # "mm", or "deg" on a rotary axis
    limits = ax.limits()  # every range the driver enforces, read from the unit itself
    low, high = limits["position"]["min"], limits["position"]["max"]
    show("unit", f"{info.model} ({ax.product_family}; actuator: {ax.kind}), firmware {info.firmware}, "
                 f"address {ax.slave}" + (f", {ax.baudrate} baud" if ax.baudrate else ""))  # fmt: skip
    if low is None or high is None:
        raise SystemExit("the unit's soft limits are not readable here: open it with "
                         "Axis.open(port, stroke=(min, max))")
    minimum = limits["speed"]["recommended_min"]
    show("range", f"{low:.2f} .. {high:.2f} {units}; speed up to {limits['speed']['max']:g} {units}/s"
                  + (f", documented minimum {minimum:g} {units}/s" if minimum else ""))  # fmt: skip
    speed = pick_speed(args.speed, limits)
    show("speed", f"{speed:g} {units}/s for every move here")
    show("status", describe(found, units))

    st = ax.servo(True)  # refused with the reasons (ServoBlockedError) when the status shows it cannot come on
    show("servo", "ON")
    if args.home or not st.homed:
        show("home", "home return: the axis runs to its mechanical end, keep the travel clear")
        started = time.perf_counter()
        st = ax.home()
        show("", f"homed at {st.position_mm:.2f} {units} ({time.perf_counter() - started:.1f} s)")
    else:
        show("home", "not needed: the status says homed (--home runs a home return anyway)")

    middle = round((low + high) / 2, 2)
    step = round(min(args.distance, (high - low) / 4), 2)
    started = time.perf_counter()
    st = ax.move_to(middle, speed_mm_s=speed)  # absolute; returns once in position and settled
    show("move", f"to the middle of the stroke: at {st.position_mm:.2f} {units} ({time.perf_counter() - started:.1f} s)")
    for delta in (step, -step):
        started = time.perf_counter()
        st = ax.move_by(delta, speed_mm_s=speed)  # relative to the measured position
        show("move", f"by {delta:+.2f} {units}: at {st.position_mm:.2f} {units} ({time.perf_counter() - started:.1f} s)")

    show("status", describe(ax.status(), units))
    ax.servo(False)
    show("servo", "OFF")
    if not found.modbus_enabled:
        ax.disable_modbus_control()  # PIO had control when this program started: hand it back
        show("control", "handed back to PIO")


def report(exc: BaseException) -> None:
    """Print the error or the interruption with the driver's notes (they say whether a decel STOP was sent)."""
    sys.stdout.flush()  # keep the order when both streams go to one file
    if isinstance(exc, KeyboardInterrupt):
        print("interrupted (Ctrl+C)", file=sys.stderr)
    else:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)
    if isinstance(exc, EcAlarmError):
        alarm = alarm_info(exc.alarm_code)
        print(f"       alarm {alarm['hex']} {alarm['name']}: {alarm['reset']}", file=sys.stderr)


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point. Exit codes: 0 done, 2 error, 130 interrupted (Ctrl+C)."""
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="  [driver %(levelname)s] %(message)s", stream=sys.stdout)
    if sys.platform == "win32":  # Ctrl+Break ends the program like Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    print(BANNER)
    try:
        # Leaving this block on an error or Ctrl+C after a motion command sends a decel STOP; it always closes the port.
        with open_axis(args) as ax:
            first_move(ax, args)
    except KeyboardInterrupt as exc:
        report(exc)
        return 130
    except (EcDriverError, OSError) as exc:
        report(exc)
        return 2
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
