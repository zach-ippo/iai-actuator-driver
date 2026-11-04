"""Draw a circle with two axes: a dry-run plan printed as numbers, then the run (coordinated motion, UNVERIFIED on
hardware; see docs/DRIVER.md, "Coordinated motion (alpha)").

    python examples/python/draw_circle.py --sim                              # two simulated axes, no serial port
    python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --dry-run    # plan only: reads positions and ranges
    python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --radius 10 --speed 10

Without ``--center`` the circle passes through the current position (it starts there, at 0 deg). Start small and slow
on hardware, unloaded and mid-stroke; Ctrl+C sends both axes a decel stop and prints the result so far.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path as _FilePath

ROOT = _FilePath(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import Cell, CoordinatedMotionError, EcNotReadyError, Path, SoftLimitError, sim_ports  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sim", action="store_true", help="two simulated axes (no serial port)")
    ap.add_argument("--x", default=None, help="cell name of the x axis, e.g. COM3:1")
    ap.add_argument("--y", default=None, help="cell name of the y axis, e.g. COM4:1")
    ap.add_argument("--radius", type=float, default=10.0, help="mm (default 10)")
    ap.add_argument("--speed", type=float, default=20.0, help="path speed, mm/s (default 20)")
    ap.add_argument("--accel", type=float, default=0.3, help="ramps, G (default 0.3)")
    ap.add_argument("--chord-error", type=float, default=0.05, help="mm (default 0.05)")
    ap.add_argument("--center", default=None, help="X,Y (default: the current position minus the radius on x)")
    ap.add_argument("--dry-run", action="store_true", help="plan and print only; nothing is sent")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")

    if args.sim:
        cell = Cell.open(sim_ports(2))
        x_name, y_name = "SIM1:1", "SIM2:1"
    else:
        if not args.x or not args.y:
            ap.error("give --x and --y (cell names such as COM3:1), or --sim")
        ports = sorted({args.x.split(":")[0], args.y.split(":")[0]})
        cell = Cell.open(ports)
        x_name, y_name = args.x, args.y
    with cell:
        xy = cell.coordinated(x=x_name, y=y_name)
        here = xy.positions()
        if args.center:
            cx, cy = (float(v) for v in args.center.split(","))
        else:
            cx, cy = here["x"] - args.radius, here["y"]
        path = Path.circle({"x": cx, "y": cy}, args.radius)
        try:
            plan = xy.plan(path, args.speed, args.accel, chord_error=args.chord_error)
        except SoftLimitError as exc:
            print(f"refused: {exc}")
            return 2
        print(plan.summary())
        print(f"{'k':>4} {'t s':>8} {'x':>9} {'y':>9} {'vx':>8} {'vy':>8}")
        for row in plan.table():
            print(f"{row['k']:>4} {row['t']:>8.3f} {row['x']:>9.3f} {row['y']:>9.3f} "
                  f"{row.get('v_x', 0):>8.2f} {row.get('v_y', 0):>8.2f}")  # fmt: skip
        if args.dry_run:
            return 0
        try:
            result = xy.run_plan(plan)
        except EcNotReadyError as exc:
            print(f"not ready (nothing sent): {exc} - servo on and home both axes first")
            return 2
        except CoordinatedMotionError as exc:
            print(f"stopped: {exc}")
            print(exc.result.summary())
            return 1
        except KeyboardInterrupt as exc:  # the run sent both axes a decel stop, then re-raised Ctrl+C
            print("interrupted: both axes were sent a decel stop")
            partial = getattr(exc, "coordinated_result", None)
            if partial is not None:
                print(partial.summary())
            return 130
        print(result.summary())
        for t, x, y in result.trace[:: max(1, len(result.trace) // 20)]:
            print(f"  t {t:6.3f}  x {x:8.3f}  y {y:8.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
