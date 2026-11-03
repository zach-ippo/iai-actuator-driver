"""Several axes as one cell: open, read, servo, home, move together, stop, and the fault policy.

Shows ``Cell.open`` over several ports (one axis per COM port is the normal setup, VERIFIED; several axes on one port,
by address, are UNVERIFIED on hardware), ``status_all``, ``servo_all``, ``home_all``, ``move_all``, ``stop_all`` during
a move, and the cell's fault policy: when the link of one axis fails, ``on_fault`` is called once for that axis and,
with ``stop_on_fault`` (the default), every other axis gets a decel STOP. Every position comes from the unit's own
range, so the same script runs on any mix of units (slider, rod, gripper, a CON controller ...).

Run it from the project folder::

    python examples/python/multi_axis_cell.py --sim                       # three simulated ports, no hardware
    python examples/python/multi_axis_cell.py --sim --sim-axes 5 --home   # five simulated ports; home every axis
    python examples/python/multi_axis_cell.py --port COM3 --port COM4     # two IAI USB adapters (close IA-OS first)

With ``--sim`` the script also pulls the simulated USB adapter of the last port during a move, to show the fault
policy, and brings that axis back with ``recover_all``. On hardware that part is left out; ``on_fault`` still reports
a real fault. ``--no-stop-on-fault`` leaves the other axes running when one fails.

Motion: around the middle of each unit's stroke, ``--step`` apart (default 10 mm, at most a quarter of the stroke), at
20 mm/s (raised to a unit's documented minimum speed where that is higher; ``--speed`` sets it): values for a first run
on an unloaded unit. Homing (the axes that report no home, or every axis with ``--home``) runs each axis to its
mechanical end: keep the travel clear. At the end every servo is switched off. On Ctrl+C or an error every axis gets a
decel STOP and the servos are left as they are (a vertical axis without a brake can drop when its servo goes off).
The driver is not a safety function: keep the hardware emergency stop within reach.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, NamedTuple

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Cell, EcDriverError, EcNotReadyError, SimLine, sim_ports  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

FIRST_RUN_SPEED = 20.0  # units/s (mm/s, deg/s on a rotary axis) when --speed is not given
FAULT_AFTER_S = 0.15  # simulator: the adapter is pulled this long after the move starts


class Positions(NamedTuple):
    """Three positions near the middle of one unit's own range."""

    low: float
    middle: float
    high: float


def work_positions(limits: Mapping[str, Any], step: float) -> Positions:
    """The middle of the unit's position range and ``step`` around it (at most a quarter of the range)."""
    lo, hi = limits["position"]["min"], limits["position"]["max"]
    if lo is None or hi is None:
        raise EcNotReadyError("the unit's position range is not known (no soft limits read): no positions to plan")
    middle = round((lo + hi) / 2, 2)
    half = round(min(step, (hi - lo) / 4) / 2, 2)
    return Positions(round(middle - half, 2), middle, round(middle + half, 2))


def first_run_speed(limits: Mapping[str, Any], requested: float | None) -> float:
    """``requested``, else 20 units/s raised to the unit's documented minimum speed (below it the unit may run
    unsteadily; the driver warns, never refuses) and kept under the unit's maximum."""
    if requested is not None:
        return requested
    speed = limits["speed"]
    return min(max(FIRST_RUN_SPEED, speed["recommended_min"] or 0.0), speed["max"])


def on_fault(name: str, kind: str, error: BaseException | None) -> None:
    """``Cell.open(on_fault=...)``: called once when the link of an axis fails. It may run in a driver thread, so it
    only reports. With ``stop_on_fault`` the cell then stops every other axis by itself."""
    cause = f"{type(error).__name__}: {error}" if error is not None else "seen by a link check"
    print(f"on_fault: {name}: {kind} ({cause})")


def describe(cell: Cell, plan: Mapping[str, Positions], speeds: Mapping[str, float]) -> None:
    """One line per axis: identity, kind, units and the planned positions."""
    kinds = ", ".join(f"{port} {kind}" for port, kind in cell.lines.items())
    print(f"cell: {len(cell)} {'axis' if len(cell) == 1 else 'axes'}; ports and lines: {kinds}")
    for name in cell.names:
        info, units, p = cell.info[name], cell.units_of(name), plan[name]
        print(f"  {name:<9} {info.model} ({info.product_family}, {cell[name].kind}), serial {info.serial_number}; "
              f"positions {p.low:.2f} / {p.middle:.2f} / {p.high:.2f} {units} at {speeds[name]:g} {units}/s")
    for port, exc in cell.port_errors.items():
        print(f"  skipped port {port}: {exc}")


def print_status(cell: Cell, title: str) -> None:
    """``status_all``: one status per axis (ports read in parallel)."""
    print(title)
    for name, st in cell.status_all().items():
        flags = {
            "servo ON": st.servo_on, "homed": st.homed, "in position": st.in_position, "moving": st.moving,
            "PC control": st.modbus_enabled, "ALARM": st.alarm, "warning": st.warning,
            "EMERGENCY STOP": st.emergency_stop,
        }  # fmt: skip
        shown = ", ".join(text for text, on in flags.items() if on) or "-"
        alarm = f", alarm code 0x{st.alarm_code:03X}" if st.alarm_code else ""
        print(f"  {name:<9} {st.position_mm:8.2f} {cell.units_of(name):<3} {shown}{alarm}")


def move(cell: Cell, label: str, targets: Mapping[str, float], speeds: Mapping[str, float], accel: float) -> None:
    """``move_all``: every target is checked against its own unit first (nothing starts if one is refused), then the
    axes start together and the call returns once every axis is in position."""
    started = time.perf_counter()
    done = cell.move_all(targets, speed_mm_s=speeds, accel_g=accel) or {}
    where = ", ".join(f"{name} {st.position_mm:.2f}" for name, st in done.items())
    print(f"move_all {label}: {where} ({time.perf_counter() - started:.2f} s)")


def stop_during_move(cell: Cell, plan: Mapping[str, Positions], speeds: Mapping[str, float], accel: float) -> None:
    """``stop_all`` while every axis moves: the STOP goes to every axis first, then each is held until it stands
    still. The axes are at their low positions when this starts."""
    cell.move_all({n: p.high for n, p in plan.items()}, speed_mm_s=speeds, accel_g=accel, wait=False)
    time.sleep(min(1.0, min((p.high - p.low) / speeds[n] for n, p in plan.items()) / 2))
    stopped = cell.stop_all()
    where = ", ".join(f"{name} {'?' if st is None else f'{st.position_mm:.2f}'}" for name, st in stopped.items())
    print(f"stop_all during a move to the high positions: stopped at {where}")


def show_fault_policy(
    cell: Cell, lines: Mapping[str, SimLine], plan: Mapping[str, Positions], speeds: Mapping[str, float], accel: float
) -> None:
    """Simulator only: pull the USB adapter of the last port while every axis moves to its low position, see what the
    fault policy does, then plug it back in and bring the axis back with ``recover_all`` (which never moves it)."""
    port = list(lines)[-1]
    others = [name for name in cell.names if cell.port_of(name).name != port]
    policy = "stop every other axis" if cell.stop_on_fault else "leave the other axes running"
    if not others:
        policy += " (this cell has no other axis)"
    print(f"\nfault policy (simulator): the USB adapter of {port} is pulled during a move; the cell is set to {policy}")
    cell.move_all({n: p.low for n, p in plan.items()}, speed_mm_s=speeds, accel_g=accel, wait=False)
    time.sleep(FAULT_AFTER_S)
    lines[port].unplug()
    readings = cell.status_all(return_exceptions=True)  # the read on the pulled port fails: on_fault, then the policy
    for name, result in readings.items():
        if isinstance(result, BaseException):
            print(f"  status_all: {name}: {type(result).__name__}: {result}")
    print(f"  cell.faults: {cell.faults}")
    if not cell.stop_on_fault:
        cell.wait_all(others)  # the other axes finish their moves
    for name, st in cell.status_all(others).items():
        print(f"  {name} at {st.position_mm:.2f} (target {plan[name].low:.2f})")
    lines[port].replug()
    reports = cell.recover_all(list(cell.faults), timeout=10)
    for name, report in reports.items():
        print(f"  recover_all: {report.summary()}; actions: {', '.join(report.actions) or 'none'}")
    cell.wait_all(list(reports))  # a move the unit was running when its adapter was pulled ends by itself
    for name, st in cell.status_all(list(reports)).items():
        print(f"  {name} at {st.position_mm:.2f} (target {plan[name].low:.2f}): the unit has no communication "
              "watchdog, so the move it was running when the link went ran to its end")
    print(f"  events: {', '.join(f'{e.kind} {e.axis}' for e in cell.events)}")


def run(cell: Cell, args: argparse.Namespace, lines: Mapping[str, SimLine] | None) -> None:
    """The whole example on an open cell."""
    limits = cell.limits_all()  # each unit's own ranges, units and kind
    plan = {name: work_positions(limits[name], args.step) for name in cell.names}
    speeds = {name: first_run_speed(limits[name], args.speed) for name in cell.names}
    describe(cell, plan, speeds)
    print_status(cell, "status_all")

    print("servo_all(True)")
    cell.servo_all(True)
    homes = [name for name, st in cell.status_all().items() if args.home or not st.homed]
    if homes:
        print(f"home_all: {', '.join(homes)} (each runs to its mechanical end)")
        cell.home_all(homes)
    else:
        print("home_all: not needed, every axis reports its home (HEND); --home homes them anyway")
    cell.track_all()  # identity, baud rate, PC control, HEND: the baseline recover_all() compares with

    move(cell, "to the middle", {n: p.middle for n, p in plan.items()}, speeds, args.accel)
    move(cell, "to the high positions", {n: p.high for n, p in plan.items()}, speeds, args.accel)
    move(cell, "to the low positions", {n: p.low for n, p in plan.items()}, speeds, args.accel)
    stop_during_move(cell, plan, speeds, args.accel)
    if lines is not None:
        show_fault_policy(cell, lines, plan, speeds, args.accel)
    move(cell, "back to the middle", {n: p.middle for n, p in plan.items()}, speeds, args.accel)
    print_status(cell, "status_all")
    print("servo_all(False)")
    cell.servo_all(False)


def stop_every_axis(cell: Cell) -> None:
    """Decel STOP to every axis (Ctrl+C or an error). A STOP that could not be confirmed is reported: the unit has no
    communication watchdog, so a move it is running then runs to its end."""
    results = cell.stop_all(return_exceptions=True)
    print("decel STOP sent to every axis; the servos are left as they are")
    for name, result in results.items():
        if isinstance(result, BaseException):
            print(f"  STOP not confirmed on {name}: {type(result).__name__}: {result}")


def port_names(values: Sequence[str]) -> list[str]:
    """``--port COM3 --port COM4`` or ``--port COM3,COM4`` -> ``["COM3", "COM4"]``."""
    return [name.strip() for value in values for name in value.split(",") if name.strip()]


def report_error(exc: BaseException) -> None:
    sys.stdout.flush()  # keep the order when both streams go to one file
    print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="iai-actuator-driver example: several axes as one cell (unofficial; not an IAI product)"
    )
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument(
        "--port", action="append", metavar="COMx",
        help="a serial port with one or more axes; repeat it (--port COM3 --port COM4) or list them (COM3,COM4). "
        "Close IA-OS first",
    )  # fmt: skip
    where.add_argument("--sim", action="store_true", help="simulated ports instead of hardware")
    parser.add_argument(
        "--speed", type=float, default=None,
        help="mm/s (deg/s on a rotary axis); default 20, raised to a unit's documented minimum where that is higher",
    )  # fmt: skip
    parser.add_argument("--accel", type=float, default=0.1, help="acceleration in G (default 0.1)")
    parser.add_argument(
        "--step", type=float, default=10.0,
        help="distance between the low and high positions in mm or deg (default 10; at most a quarter of a stroke)",
    )  # fmt: skip
    parser.add_argument("--home", action="store_true", help="home every axis, not only those that report no home")
    parser.add_argument(
        "--no-stop-on-fault", action="store_true", help="leave the other axes running when the link of one axis fails"
    )
    parser.add_argument(
        "--sim-axes", type=int, default=3, choices=range(1, 17), metavar="N",
        help="with --sim: simulated ports, one axis each (1..16, default 3)",
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
    lines = sim_ports(args.sim_axes, profile=args.sim_profile) if args.sim else None
    ports: Any = lines if lines is not None else port_names(args.port)
    try:
        with Cell.open(ports, on_fault=on_fault, stop_on_fault=not args.no_stop_on_fault) as cell:
            try:
                run(cell, args, lines)
            except BaseException:
                stop_every_axis(cell)  # STOP first; the cell does it again when the with block ends on the error
                raise
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("interrupted", file=sys.stderr)
        return 130
    except (EcDriverError, OSError, ValueError) as exc:  # CellError (one or more axes failed) is an EcDriverError
        report_error(exc)
        return 2
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
