"""Push / press with one axis: approach, push over a band at a set force, read the result (contact or missed), retract.

``ax.push(target_mm, band_mm, force_pct, direction)`` moves to ``target_mm`` at the given speed, then pushes up to
``band_mm`` further at the unit's push speed, with the motor current limited to ``force_pct`` % (push moves with and
without contact are VERIFIED on hardware). The result (``PushResult``) is ``"contact"``: the axis met
the work and keeps pushing until the next move command, or ``"missed"``: it went over the whole band and stopped at
its end. The driver reports either one only after it has held for 0.15 s with the position unchanged. The next move
(here the retract with ``move_to``) ends the push. ``ax.limits()["push"]`` says whether the unit can push, its force
range (1 % .. the unit's MXPP) and its recommended minimum (MNPP, or 20 %): a lower force is allowed, with a warning.

Run it from the project folder (close IA-OS first: only one program can use the serial port)::

    python examples/python/push_press.py --sim                        # a simulated workpiece inside the band: contact
    python examples/python/push_press.py --sim --sim-no-workpiece     # nothing in the band: missed
    python examples/python/push_press.py --sim --sim-profile gripper  # the gripper profile grips the workpiece
    python examples/python/push_press.py --port COM3                  # an unloaded unit: expect missed
    python examples/python/push_press.py --port COM3 --force-pct 30 --band 5 --toward-home

Motion (defaults): a move to the start position, one band's length before the middle of the stroke; the push itself
(the approach to the middle at 20 mm/s, deg/s on a rotary axis, or at the unit's documented minimum speed when that is
higher; then up to 10 mm further, at most a quarter of the stroke, away from home at the unit's push speed and at its
recommended minimum force); then the retract to the start position. If the axis is not homed, a home return runs
first (the axis runs to its mechanical end: keep the travel clear). There is no communication watchdog: if the PC
or the cable fails, a push keeps pushing. Design the machine so that this is safe.

End: the servo stays ON, unless it was OFF at the start (then it is switched OFF again; a vertical axis without a
brake can fall when the servo goes off: support it first). Modbus control is handed back to PIO when PIO had it at the
start. On Ctrl+C or an error after a motion command, also while a contact holds, a decel STOP is sent before the port
is closed (by the driver inside a move, or when the ``with`` block is left); the servo is then left as it is.

Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Axis, EcAlarmError, EcDriverError, SimulatedEC, Status  # noqa: E402
from iai_actuator_driver.alarms import alarm_info  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

BANNER = "iai-actuator-driver example: push / press (UNOFFICIAL - not an IAI product)"
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
        "--sim-no-workpiece", action="store_true",
        help="with --sim: no simulated workpiece in the band, so the push misses",
    )  # fmt: skip
    parser.add_argument(
        "--force-pct", type=positive, default=None,
        help="push force in %%, the motor current limit (default: the unit's recommended minimum, MNPP or 20)",
    )  # fmt: skip
    parser.add_argument(
        "--band", type=positive, default=10.0,
        help="push band in mm, deg on a rotary axis (default 10; at most a quarter of the stroke)",
    )  # fmt: skip
    parser.add_argument("--toward-home", action="store_true", help="push toward home (default: away from home)")
    parser.add_argument(
        "--speed", type=positive, default=None,
        help=f"move and approach speed in mm/s, deg/s on a rotary axis (default {SPEED:g}, or the unit's documented "
        "minimum if higher); the push itself runs at the unit's push speed",
    )  # fmt: skip
    return parser


def open_axis(args: argparse.Namespace) -> tuple[Axis, SimulatedEC | None]:
    """The axis on ``--port`` (baud rate found automatically), or a simulated unit with ``--sim`` (also returned, so
    the example can place a workpiece in it)."""
    if args.sim:
        # pmsl=False: PIO in control, as after a power-up, so connect() takes Modbus control as it does on hardware.
        sim = SimulatedEC(slave=args.slave, profile=args.sim_profile, pmsl=False)
        return Axis.open(sim, slave=args.slave), sim
    return Axis.open(args.port, slave=args.slave), None


def show(label: str, text: str) -> None:
    """One line of output: a short label, then the text."""
    print(f"{label:<10} {text}")


def describe(st: Status, units: str) -> str:
    """The status in one line: position and the flags that matter around a push."""
    flags = [
        name
        for name, on in (
            ("servo ON", st.servo_on), ("homed", st.homed), ("in position", st.in_position), ("moving", st.moving),
            ("pushing", st.pushing), ("push contact", st.push_contact), ("push missed", st.push_missed),
            ("PC control", st.modbus_enabled), ("emergency stop", st.emergency_stop),
        )
        if on
    ]  # fmt: skip
    text = f"position {st.position_mm:.2f} {units}, " + (", ".join(flags) or "no flags set")
    if (st.alarm or st.warning) and st.alarm_code:
        alarm = alarm_info(st.alarm_code)
        text += f"; {'alarm' if st.alarm else 'warning'} {alarm['hex']} {alarm['name']}"
    return text


def pick_speed(requested: float | None, limits: dict[str, Any]) -> float:
    """``requested``, else the default speed or the unit's documented minimum speed when that is higher (below it a
    unit may run noisily or unsteadily; the driver only warns)."""
    if requested is not None:
        return requested
    minimum = limits["speed"]["recommended_min"]
    return max(SPEED, minimum) if minimum else SPEED


def push_press(ax: Axis, args: argparse.Namespace, sim: SimulatedEC | None) -> None:
    """Move to the start, push, report contact or missed, retract to the start."""
    found = ax.status()  # read-only: the state the unit was in before this program sent anything
    info = ax.connect()  # identity and ranges; Modbus control is taken here, at rest only
    units = ax.units  # "mm", or "deg" on a rotary axis
    limits = ax.limits()
    low, high = limits["position"]["min"], limits["position"]["max"]
    push = limits["push"]  # available, force range and recommended minimum (%), push speed
    show("unit", f"{info.model} ({ax.product_family}; actuator: {ax.kind}), firmware {info.firmware}, "
                 f"address {ax.slave}" + (f", {ax.baudrate} baud" if ax.baudrate else ""))  # fmt: skip
    if low is None or high is None:
        raise SystemExit("the unit's soft limits are not readable here: open it with "
                         "Axis.open(port, stroke=(min, max))")
    show("status", describe(found, units))
    if push["available"] is False:
        show("push", "this unit reports no push function (SUPT PUSH, MXPP or TCPC is 0): nothing to do")
        return

    force = args.force_pct if args.force_pct is not None else min(push["recommended_min"], push["max"])
    direction = -1 if args.toward_home else 1  # +1 pushes away from home (increasing position)
    band = round(min(args.band, (high - low) / 4), 2)
    target = round((low + high) / 2, 2)  # where the approach ends and the push starts
    end = round(target + direction * band, 2)  # the push goes no further than this
    start = round(target - direction * band, 2)  # on the approach side: a push must not start inside its zone
    speed = pick_speed(args.speed, limits)
    push_speed, source = ax.push_speed_mm_s(speed)  # min(the approach speed, the unit's PSHV 8.15 / MXPH 7.8)
    show("force", f"{force:g} % (this unit: 1 .. {push['max']:g} %, recommended minimum {push['recommended_min']:g} %)")
    show("plan", f"start {start:.2f}, approach {target:.2f} at {speed:g} {units}/s, push to at most {end:.2f} "
                 f"{units} at {push_speed:g} {units}/s ({source}), retract to {start:.2f}")  # fmt: skip
    if sim is not None and not args.sim_no_workpiece:
        sim.obstacle_mm = round(target + direction * band / 2, 2)  # simulator only: something to push against
        show("sim", f"a simulated workpiece at {sim.obstacle_mm:.2f} {units}")

    st = ax.servo(True)  # refused with the reasons (ServoBlockedError) when the status shows it cannot come on
    if not st.homed:
        show("home", "home return: the axis runs to its mechanical end, keep the travel clear")
        ax.home()
    st = ax.move_to(start, speed_mm_s=speed)
    show("start", f"at {st.position_mm:.2f} {units}")

    # A force below the unit's recommended minimum is allowed: the driver logs a warning (also in result.warnings).
    result = ax.push(target, band_mm=band, force_pct=force, direction=direction, speed_mm_s=speed)
    if result.contact:
        show("result", f"contact at {result.position_mm:.2f} {units}: the axis keeps pushing until the next move")
    else:
        show("result", f"missed: no contact over the band, stopped at {result.position_mm:.2f} {units}")
    show("status", describe(result.status, units))

    st = ax.move_to(start, speed_mm_s=speed)  # the retract: a new move command ends the push
    show("retract", f"at {st.position_mm:.2f} {units}")
    if not found.servo_on:
        ax.servo(False)  # the servo was OFF when this program started
        show("servo", "OFF (as found)")
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
        ax, sim = open_axis(args)
        # Leaving this block on an error or Ctrl+C after a motion command sends a decel STOP; it always closes the port.
        with ax:
            push_press(ax, args, sim)
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
