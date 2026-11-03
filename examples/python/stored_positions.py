"""Stored positions: read the position table, then run stored rows slowly with move_to_position and the speed override.

A stored row (position, speed, acceleration / deceleration, band, an optional push) lives in the unit's non-volatile
memory; it is set up with IA-OS, ``ax.write_position_table()`` or ``ax.teach_position()``. This example reads the
table with ``ax.read_position_table(n)`` (rows 0 .. ``ax.max_positions()`` - 1), then runs rows with
``ax.move_to_position(n, speed_pct=...)``: the unit runs the row at that share of its stored speed (IAI command 0x45
cmd 6), and nothing is written to the unit (VERIFIED on the EC-GRBP8M gripper and the PCON-CB). The driver checks
each row before anything is sent: a position or push end outside the stroke, an incremental row, a bad checksum, or a
speed, acceleration or push force above the unit's maximum is refused with ``SoftLimitError``, and the example goes
on with the next row.

Run it from the project folder (close IA-OS first: only one program can use the serial port)::

    python examples/python/stored_positions.py --sim                          # simulated ELECYLINDER slider
    python examples/python/stored_positions.py --sim --sim-profile gripper    # a gripper whose row 1 is a push row
    python examples/python/stored_positions.py --port COM3
    python examples/python/stored_positions.py --port COM3 --rows 0,1 --speed 10

Motion: each row runs at the whole percentage of its stored speed (1..100 %) that brings it down to about 20 mm/s
(deg/s on a rotary axis), or to the unit's documented minimum speed when that is higher; a row stored faster than 100
times that speed is not run. Acceleration and deceleration are the row's own. By default the rows in use among 0, 1
and 2 run, in that order (on an ELECYLINDER: backward end, forward end, middle); a row with a stored speed of 0 is
empty. A push row runs its push: on an unloaded unit it ends "missed" at the end of its band. At the end the axis
moves back to where it was before the first row, which also ends a push that holds against the work. If the axis is
not homed, a home return runs first (the axis runs to its mechanical end: keep the travel clear).

End: the servo stays ON, unless it was OFF at the start (then it is switched OFF again; a vertical axis without a
brake can fall when the servo goes off: support it first). Modbus control is handed back to PIO when PIO had it at the
start. On Ctrl+C or an error after a motion command a decel STOP is sent before the port is closed (by the driver
inside a move, or when the ``with`` block is left); the servo is then left as it is.

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

from iai_actuator_driver import (  # noqa: E402
    Axis,
    EcAlarmError,
    EcDriverError,
    LineRuleError,
    PositionEntry,
    SimulatedEC,
    SoftLimitError,
    Status,
)
from iai_actuator_driver import registers as R  # noqa: E402
from iai_actuator_driver.alarms import alarm_info  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

BANNER = "iai-actuator-driver example: stored positions (UNOFFICIAL - not an IAI product)"
SPEED = 20.0  # units/s: the default speed for a first test (GETTING_STARTED.md: unloaded, mid-stroke, 20 mm/s or less)


def row_list(text: str) -> list[int]:
    """``"0,1,2"`` -> ``[0, 1, 2]`` (argparse type)."""
    try:
        rows = [int(part) for part in text.split(",") if part.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError(f"rows are numbers such as 0,1,2, got {text!r}") from None
    if not rows or min(rows) < 0:
        raise argparse.ArgumentTypeError(f"rows are numbers such as 0,1,2, got {text!r}")
    return rows


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
        "--rows", type=row_list, default=None,
        help="rows to run, in this order, e.g. 0,1,2 (default: the rows in use among 0, 1 and 2)",
    )  # fmt: skip
    parser.add_argument(
        "--speed", type=positive, default=None,
        help=f"about this speed for every row, mm/s or deg/s (default {SPEED:g}, or the unit's documented minimum "
        "if higher); the override is a whole percentage of the row's stored speed",
    )  # fmt: skip
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


def describe_row(entry: PositionEntry, units: str) -> str:
    """One stored row in one line, as the unit holds it."""
    text = (f"{entry.position_mm:8.2f} {units} at {entry.speed_mm_s:7.2f} {units}/s, accel / decel "
            f"{entry.accel_g:.2f} / {entry.decel_g:.2f} G, band {entry.band_mm:.2f} {units}")  # fmt: skip
    if entry.control_flags & R.CTLF_PUSH:
        away = entry.control_flags & R.CTLF_PUSH_AWAY
        end = entry.position_mm + (entry.band_mm if away else -entry.band_mm)
        text += f", push {R.push_pct(entry.push_power):.0f} % to {end:.2f} {units}"
    if entry.control_flags & R.CTLF_INC:
        text += ", incremental"
    if entry.checksum_ok is False:
        text += ", checksum BAD"
    return text


def pick_speed(requested: float | None, limits: dict[str, Any]) -> float:
    """``requested``, else the default speed or the unit's documented minimum speed when that is higher (below it a
    unit may run noisily or unsteadily; the driver only warns)."""
    if requested is not None:
        return requested
    minimum = limits["speed"]["recommended_min"]
    return max(SPEED, minimum) if minimum else SPEED


def stored_positions(ax: Axis, args: argparse.Namespace) -> None:
    """Read the table, run the chosen rows at about ``speed``, move back to the start position."""
    found = ax.status()  # read-only: the state the unit was in before this program sent anything
    info = ax.connect()  # identity and ranges; Modbus control is taken here, at rest only
    units = ax.units  # "mm", or "deg" on a rotary axis
    limits = ax.limits()
    low, high = limits["position"]["min"], limits["position"]["max"]
    show("unit", f"{info.model} ({ax.product_family}; actuator: {ax.kind}), firmware {info.firmware}, "
                 f"address {ax.slave}" + (f", {ax.baudrate} baud" if ax.baudrate else ""))  # fmt: skip
    if low is None or high is None:
        raise SystemExit("the unit's soft limits are not readable here: open it with "
                         "Axis.open(port, stroke=(min, max))")
    show("status", describe(found, units))

    count = ax.max_positions()  # the unit's own row count: 3 on an ELECYLINDER, 64 on the PCON-CB tested
    table = {n: ax.read_position_table(n) for n in range(count)}  # read-only
    used = [n for n, entry in table.items() if entry.speed_mm_s > 0]
    show("table", f"{count} stored rows (0 .. {count - 1}), {len(used)} in use")
    for n in used:
        show(f"row {n}", describe_row(table[n], units))
    if len(used) < count:
        empty = count - len(used)
        show("", f"{empty} empty row{'s' if empty > 1 else ''} (stored speed 0) not listed")

    speed = pick_speed(args.speed, limits)
    rows = args.rows if args.rows is not None else [n for n in used if n < 3]
    show("run", f"rows {', '.join(map(str, rows)) or 'none'} at about {speed:g} {units}/s")

    st = ax.servo(True)  # refused with the reasons (ServoBlockedError) when the status shows it cannot come on
    if not st.homed:
        show("home", "home return: the axis runs to its mechanical end, keep the travel clear")
        st = ax.home()
    start = round(min(max(st.position_mm, low), high), 2)  # where the axis goes back to at the end

    for n in rows:
        entry = table.get(n)
        if entry is None:
            show(f"row {n}", f"not run: this unit has rows 0 .. {count - 1}")
            continue
        if entry.speed_mm_s <= 0:
            show(f"row {n}", "not run: empty (stored speed 0)")
            continue
        pct = max(1, min(100, int(100 * speed / entry.speed_mm_s)))  # a whole percentage, rounded down
        if entry.speed_mm_s * pct / 100 > speed:  # even 1 % is faster than wanted
            show(f"row {n}", f"not run: 1 % of its stored {entry.speed_mm_s:g} {units}/s is above {speed:g} {units}/s")
            continue
        show(f"row {n}", f"to {entry.position_mm:.2f} {units} at {pct} % of {entry.speed_mm_s:g} {units}/s "
                         f"(about {entry.speed_mm_s * pct / 100:.1f} {units}/s)")  # fmt: skip
        started = time.perf_counter()
        try:
            st = ax.move_to_position(n, speed_pct=pct)  # waits until the row is done (a push row: contact or missed)
        except (SoftLimitError, LineRuleError) as exc:  # refused by the driver's checks: nothing was sent
            show("", f"not run: {exc}")
            continue
        result = f"at {st.position_mm:.2f} {units} ({time.perf_counter() - started:.1f} s)"
        if st.push_contact:
            result += "; push: contact, the axis keeps pushing until the next move command"
        elif st.push_missed:
            result += "; push: missed, no contact over the band"
        show("", result)

    st = ax.move_to(start, speed_mm_s=speed)  # a direct move (RAM only); also ends a push that holds
    show("back", f"to the start position: at {st.position_mm:.2f} {units}")
    show("status", describe(ax.status(), units))
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
        # Leaving this block on an error or Ctrl+C after a motion command sends a decel STOP; it always closes the port.
        with open_axis(args) as ax:
            stored_positions(ax, args)
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
