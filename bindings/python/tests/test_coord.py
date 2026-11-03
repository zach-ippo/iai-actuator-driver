"""Coordinated (interpolated) multi-axis motion: geometry, the planner's checks, sync and streamed runs (simulator)."""

from __future__ import annotations

import _thread
import json
import logging
import math
import struct
import threading
import time
import unittest
from collections.abc import Iterable
from typing import Any
from unittest import mock

import iai_actuator_driver
from iai_actuator_driver import registers as R
from iai_actuator_driver.cell import Cell
from iai_actuator_driver.coord import (
    Arc,
    AxisRange,
    Circle,
    CoordinatedGroup,
    CoordinatedMotionError,
    CoordinatedResult,
    Line,
    LineTiming,
    MotionPlan,
    Path,
    Polyline,
    path_deviation,
    plan,
)
from iai_actuator_driver.ec import Elecylinder
from iai_actuator_driver.errors import EcAlarmError, EcNotReadyError, SoftLimitError
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC
from iai_actuator_driver.sim_line import SimLine, sim_line, sim_ports

# The planning code (``plan``, the path classes: GeometryTest, PlannerTest) lives in ``_shared/coord.py``, in Python.
# The runs (CoordinatedRunTest, CellCoordinatedTest) are the core's ``CoordinatedGroup``; the run's own internals (the
# line timing, lost echoes, late frames, the monitors held during a run) are tested in core/tests/test_coord*.cpp.
# CoreGroupTest is what differs from the runner of the 1.0.x fallback.

OPEN = {"poll_interval_s": 0.005, "scan_timeout": 0.05}
# a line whose replies come late: the scan waits for them in real time, so a busy PC must not make it give up first
# (and only address 1 is scanned: the absent ones would each cost the whole wait)
LATE_OPEN = {**OPEN, "scan_timeout": 1.0, "scan_slaves": [1]}
RANGE = {"position": (0.0, 150.0), "speed": 700.0, "accel_g": 1.0}
LIMITS = {"x": RANGE, "y": RANGE}
CENTER = {"x": 75.0, "y": 75.0}
# the model's shortest step for one axis per port: tests whose subject is not the measured line pass it, so a plan is
# not refused because a busy PC stretched the measurement (min_step_s also skips run_plan's timing check)
STEP = {"min_step_s": LineTiming().min_step_s}


def _move_frames(line: SimLine, slave: int = 1, *, start: int = 0) -> list[tuple[float, bytes]]:
    """``(line time, frame)`` of every 0x9900 block written to ``slave`` on ``line`` from its ``start``-th request."""
    return [(t, frame) for t, frame in list(line.requests)[start:]
            if frame[0] == slave and frame[1] == 0x10 and struct.unpack(">H", frame[2:4])[0] == R.DIRECT_MOVE_BASE]


def _moves(line: SimLine, slave: int = 1, *, after: float | None = None, start: int = 0) -> list[dict[str, float]]:
    """Every 0x9900 block written to ``slave`` on ``line`` (from its ``start``-th request, after the line time
    ``after``): target, band, speed, accel."""
    out = []
    for t, frame in _move_frames(line, slave, start=start):
        if after is not None and t <= after:
            continue
        words = struct.unpack(f">{frame[6] // 2}H", frame[7:-2])
        out.append({"target": R.from_i32_words(words[0], words[1]) * 0.01,
                    "band": R.from_u32_words(words[2], words[3]) * 0.01,
                    "speed": R.from_u32_words(words[4], words[5]) * 0.01, "accel": words[6] * 0.01})  # fmt: skip
    return out


def _first_halt(line: SimLine, slave: int = 1, *, start: int = 0) -> float | None:
    """The line time of the first STOP or pause coil written ON to ``slave`` from the ``start``-th request (``None``:
    none)."""
    for t, frame in list(line.requests)[start:]:
        if frame[0] in (0, slave) and frame[1] == 0x05 and frame[4] == 0xFF:
            if struct.unpack(">H", frame[2:4])[0] in (R.COIL_STOP, R.COIL_STP):
                return t
    return None


def _latency_ports(n: int, latency_s: float, **sim_kwargs: Any) -> dict[str, SimLine]:
    return {f"SIM{i}": SimLine([SimulatedEC(slave=1, **sim_kwargs)], name=f"SIM{i}", latency_s=latency_s)
            for i in range(1, n + 1)}  # fmt: skip


def _when_moving(unit: SimulatedEC, action: Any, delay_s: float = 0.3) -> threading.Thread:
    """Run ``action`` in a thread ``delay_s`` after ``unit`` starts moving: well into the run, after its planning and
    checks (a busy PC stretches those, so a fixed delay from the call could land before the run). Join it."""

    def wait() -> None:
        give_up = time.monotonic() + 10.0
        while not unit.moving and time.monotonic() < give_up:
            time.sleep(0.01)
        time.sleep(delay_s)
        action()

    th = threading.Thread(target=wait, daemon=True)
    th.start()
    return th


class _Truth:
    """The simulated axes' true positions, read back to back in a background thread during a run: the path the axes
    really drew, independent of the driver's status samples and of PC load."""

    def __init__(self, units: Iterable[SimulatedEC]) -> None:
        self.units = list(units)
        self.points: list[tuple[float, ...]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def __enter__(self) -> _Truth:
        self._thread.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self._stop.set()
        self._thread.join(2.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.points.append(tuple(u.position_mm for u in self.units))
            self._stop.wait(0.003)

    def deviation(self, polyline: list[tuple[float, ...]]) -> float:
        return path_deviation(self.points, polyline)[0]


class GeometryTest(unittest.TestCase):
    def test_arc_and_circle_lengths(self) -> None:
        quarter = Path({"x": 10.0, "y": 0.0}, [Arc((0, 0), {"x": 0.0, "y": 10.0})])
        self.assertAlmostEqual(quarter.length(), 5 * math.pi, places=9)
        cw = Path({"x": 10.0, "y": 0.0}, [Arc((0, 0), {"x": 0.0, "y": 10.0}, ccw=False)])
        self.assertAlmostEqual(cw.length(), 15 * math.pi, places=9)
        self.assertAlmostEqual(Path.circle((0, 0), 10.0).length(), 20 * math.pi, places=9)
        self.assertAlmostEqual(Path.circle((0, 0), 10.0, turns=2.5).length(), 50 * math.pi, places=9)
        helix = Path({"x": 10.0, "y": 0.0, "z": 0.0}, [Arc((0, 0), {"x": 10.0, "y": 0.0, "z": 5.0})])
        self.assertAlmostEqual(helix.length(), math.hypot(20 * math.pi, 5.0), places=9)

    def test_rectangle_polygon_and_circle_points(self) -> None:
        rect = plan(Path.rectangle((10, 20), 30, 15), 20, 0.1, limits=LIMITS)
        self.assertEqual(rect.mode, "sync")
        self.assertEqual(rect.points, [(10, 20), (40, 20), (40, 35), (10, 35), (10, 20)])
        hexagon = Path.polygon({"x": 50, "y": 50, "z": 7.0}, 10.0, 6)
        self.assertEqual(hexagon.start, {"z": 7.0, "x": 60.0, "y": 50.0})
        pieces = hexagon.pieces()
        self.assertEqual(len(pieces), 6)
        for p in pieces:
            self.assertAlmostEqual(math.hypot(p.end["x"] - 50, p.end["y"] - 50), 10.0, places=9)
            self.assertEqual(p.end["z"], 7.0)
        circle = plan(Path.circle((75, 75), 10.0, start_deg=90), 20, 0.3, limits=LIMITS)
        self.assertEqual(circle.points[0], circle.points[-1])
        self.assertAlmostEqual(circle.points[0][0], 75.0, places=9)
        self.assertAlmostEqual(circle.points[0][1], 85.0, places=9)
        for x, y in circle.points:
            self.assertAlmostEqual(math.hypot(x - 75, y - 75), 10.0, places=9)
        # counter-clockwise from 90 deg: x decreases first
        self.assertLess(circle.points[1][0], 75.0)

    def test_chord_count_follows_the_chord_error(self) -> None:
        for r, e in ((10.0, 0.05), (10.0, 0.01), (50.0, 0.05), (2.0, 0.2)):
            p = plan(Path.circle((75, 75), r), 10, 0.3, chord_error=e, limits=LIMITS, min_step_s=0)
            step = 2 * math.acos(1 - e / r)
            self.assertEqual(p.steps, max(4, math.ceil(2 * math.pi / step - 1e-9)), (r, e))
            self.assertLessEqual(p.est_chord_error, e + 1e-12)
            mid = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(p.points, p.points[1:])]
            worst = max(r - math.hypot(x - 75, y - 75) for x, y in mid)
            self.assertAlmostEqual(worst, p.est_chord_error, places=9)

    def test_bad_geometry_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            Path({"x": 10.0, "y": 0.0}, [Arc((0, 0), {"x": 0.0, "y": 12.0})]).pieces()
        with self.assertRaises(ValueError):
            Path({"x": 0.0, "y": 0.0}, [Circle((50, 50), 5.0)]).pieces()
        with self.assertRaises(ValueError):
            Path(None, [Line({"x": 1.0})]).pieces()
        with self.assertRaises(ValueError):
            Path({"x": 0.0}, [Line({"y": 1.0})]).pieces()  # y has no start
        with self.assertRaises(ValueError):
            Circle((0, 0), -1.0)
        with self.assertRaises(ValueError):
            Path.rectangle((0, 0), 0, 5)
        with self.assertRaises(ValueError):
            plan(Path({"x": 1.0}, [Line({"x": 1.0})]), 10, 0.1)  # does not move
        with self.assertRaises(ValueError):
            plan(Path.circle((75, 75), 5.0), 10, 0.1, mode="sync")

    def test_degenerate_arcs_raise_value_error(self) -> None:
        # an arc from 5 microns off its center to the center passed the end tolerance and divided by zero
        with self.assertRaisesRegex(ValueError, "ends at its center"):
            plan(Path({"x": 75.005, "y": 75.0}, [Arc((75, 75), {"x": 75.0, "y": 75.0})]), 10, 0.3, min_step_s=0)
        with self.assertRaisesRegex(ValueError, "below one position count"):
            plan(Path({"x": 75.005, "y": 75.0}, [Arc((75, 75), {"x": 74.995, "y": 75.0})]), 10, 0.3, min_step_s=0)
        with self.assertRaisesRegex(ValueError, "below one position count"):
            plan(Path.circle((75, 75), 0.005), 10, 0.3, min_step_s=0)

    def test_axes_not_named_hold_and_polyline_equals_lines(self) -> None:
        path = Path({"x": 0.0, "y": 0.0, "z": 3.0}, [Polyline([{"x": 10.0}, {"y": 5.0}]), Line({"z": 4.0})])
        self.assertTrue(path.straight)
        p = plan(path, 10, 0.1)
        self.assertEqual(p.axes, ("x", "y", "z"))
        self.assertEqual(p.points, [(0, 0, 3), (10, 0, 3), (10, 5, 3), (10, 5, 4)])
        self.assertEqual(p.speeds[0], (10.0, 0.0, 0.0))  # only x moves in the first segment


class PlannerTest(unittest.TestCase):
    def test_sync_speed_and_accel_scale_with_each_axis_share(self) -> None:
        p = plan(Path({"x": 10.0, "y": 10.0}, [Line({"x": 40.0, "y": 20.0})]), 50, 0.3, limits=LIMITS)
        length = math.hypot(30, 10)
        self.assertEqual(p.speeds[0], (round(50 * 30 / length, 2), round(50 * 10 / length, 2)))
        self.assertEqual(p.accels[0], (round(0.3 * 30 / length, 2), round(0.3 * 10 / length, 2)))
        a = 0.3 * R.G_MM_S2
        self.assertAlmostEqual(p.duration, length / 50 + 50 / a, places=9)  # trapezoid: L / v + v / a

    def test_stream_profile_is_a_trapezoid_along_the_path(self) -> None:
        p = plan(Path({"x": 0.0, "y": 0.0}, [Line({"x": 100.0})]), 50, 0.01, mode="stream", limits=LIMITS)
        self.assertEqual(p.mode, "stream")
        a = 0.01 * R.G_MM_S2
        self.assertAlmostEqual(p.duration, 100 / 50 + 50 / a, places=6)
        path_speeds = [math.dist(u, v) / (t1 - t0) for u, v, t0, t1 in zip(p.points, p.points[1:], p.times, p.times[1:])]
        self.assertLess(path_speeds[0], 50)
        self.assertLess(path_speeds[-1], 50)
        self.assertAlmostEqual(max(path_speeds), 50, delta=0.5)
        self.assertGreaterEqual(p.shortest_step_s, p.min_step_s - 1e-9)  # the short ramp steps merged (collinear)

    def test_auto_mode(self) -> None:
        self.assertEqual(plan(Path.rectangle((10, 10), 5, 5), 10, 0.1).mode, "sync")
        self.assertEqual(plan(Path.circle((75, 75), 5.0), 10, 0.1, min_step_s=0).mode, "stream")

    def test_soft_limit_refusal(self) -> None:
        with self.assertRaises(SoftLimitError) as ctx:
            plan(Path.circle((145, 75), 10.0), 20, 0.3, limits=LIMITS)
        self.assertIn("soft limits", str(ctx.exception))
        self.assertIsNone(ctx.exception.max_speed)
        self.assertIn("no speed", str(ctx.exception))

    def test_the_start_point_itself_is_not_range_checked(self) -> None:
        # the axes already stand at points[0] (e.g. -0.01 inside the unit's soft limit, outside the effective stroke):
        # only the commanded waypoints count, as for a single-axis move
        p = plan(Path({"x": -0.01, "y": 5.0}, [Line({"x": 5.0, "y": 5.0})]), 10, 0.1, limits=LIMITS)
        self.assertEqual(p.points[-1], (5.0, 5.0))
        with self.assertRaises(SoftLimitError) as ctx:
            plan(Path({"x": 5.0, "y": 5.0}, [Line({"x": -0.02})]), 10, 0.1, limits=LIMITS)
        self.assertIn("waypoint 1", str(ctx.exception))

    def test_axis_speed_refusal_names_the_largest_feasible_speed(self) -> None:
        slow = {"x": RANGE, "y": {**RANGE, "speed": 10.0}}
        with self.assertRaises(SoftLimitError) as ctx:
            plan(Path({"x": 10.0, "y": 10.0}, [Line({"x": 40.0, "y": 40.0})]), 50, 0.3, limits=slow)
        self.assertIn("axis y", str(ctx.exception))
        self.assertAlmostEqual(ctx.exception.max_speed, 10 * math.sqrt(2), delta=0.02)
        ok = plan(Path({"x": 10.0, "y": 10.0}, [Line({"x": 40.0, "y": 40.0})]), ctx.exception.max_speed, 0.3,
                  limits=slow)  # fmt: skip
        self.assertLessEqual(ok.max_speed("y"), 10.0)

    def test_accel_refusals(self) -> None:
        gentle = {"x": {**RANGE, "accel_g": 0.2}, "y": RANGE}
        with self.assertRaises(SoftLimitError) as ctx:
            plan(Path({"x": 10.0, "y": 10.0}, [Line({"x": 40.0})]), 50, 0.3, limits=gentle)
        self.assertIn("acceleration", str(ctx.exception))
        with self.assertRaises(SoftLimitError) as ctx:  # centripetal: v^2 / r above the smallest axis accel
            plan(Path.circle((75, 75), 1.0), 100, 0.05, limits={a: {**RANGE, "accel_g": 0.1} for a in "xy"},
                 min_step_s=0, chord_error=0.2)  # fmt: skip
        self.assertIn("centripetal", str(ctx.exception))
        self.assertAlmostEqual(ctx.exception.max_speed, math.sqrt(0.1 * R.G_MM_S2 * 1.0), delta=0.6)

    def test_step_period_refusal(self) -> None:
        with self.assertRaises(SoftLimitError) as ctx:
            plan(Path.circle((75, 75), 10.0), 100, 0.3, limits=LIMITS, min_step_s=0.2)
        self.assertIn("the line can carry", str(ctx.exception))
        feasible = ctx.exception.max_speed
        self.assertGreater(feasible, 0)
        self.assertGreaterEqual(plan(Path.circle((75, 75), 10.0), feasible, 0.3, limits=LIMITS,
                                     min_step_s=0.2).shortest_step_s, 0.2 - 1e-9)  # fmt: skip
        # the default estimate: slower at 38400 than at 230400, more axes per port take longer
        self.assertGreater(LineTiming(38400).min_step_s, LineTiming(230400).min_step_s)
        self.assertAlmostEqual(LineTiming(axes_on_port=3).command_period_s, 3 * LineTiming().command_period_s)
        # below 38400 every reply is taken as slow as the worst one seen at 9600 (~285 ms), and a measurement counts
        self.assertGreater(LineTiming(9600).min_step_s, 0.8)
        self.assertAlmostEqual(LineTiming(measured_read_s=0.04, measured_write_s=0.05).command_period_s, 0.09)

    def test_max_speed_passes_when_the_profile_never_reaches_cruise(self) -> None:
        cases = [
            (Path.circle((75, 75), 10.0), 500, 0.01, {a: {"position": (0, 150)} for a in "xy"}, 0.2),
            (Path.circle((75, 75), 10.0), 500, 0.01, {"x": {"position": (0, 150), "speed": 5.0}, "y": RANGE}, 0),
            (Path.circle((75, 75), 10.0), 500, 0.1, LIMITS, None),
            (Path({"x": 85.15, "y": 75.0}, [Line({"x": 85.0}), Circle((75, 75), 10.0)]), 20, 0.3, LIMITS, None),
        ]
        for path, speed, accel, limits, min_step in cases:
            with self.assertRaises(SoftLimitError) as ctx:
                plan(path, speed, accel, limits=limits, min_step_s=min_step)
            best = ctx.exception.max_speed
            self.assertIsNotNone(best)
            plan(path, best, accel, limits=limits, min_step_s=min_step)  # passes at the advertised speed
            with self.assertRaises(SoftLimitError):
                plan(path, best + 0.05, accel, limits=limits, min_step_s=min_step)  # and it is about the largest

    def test_frameless_and_tiny_steps_merge(self) -> None:
        # a sub-count tail and a float arc end followed by its rounded value: no step without a frame, none too short
        end = {"x": 75 + 10 * math.cos(1.0), "y": 75 + 10 * math.sin(1.0)}
        paths = [
            Path.circle((75, 75), 10).then(Line({"x": 85.0004})),
            Path({"x": 85.0, "y": 75.0}, [Arc((75, 75), end), Line({k: round(v, 3) for k, v in end.items()}),
                                          Line({"x": 60.0, "y": 70.0})]),  # fmt: skip
        ]
        for path in paths:
            p = plan(path, 20, 0.3, limits=LIMITS)
            self.assertGreaterEqual(p.shortest_step_s, p.min_step_s - 1e-9)
            self.assertTrue(all(any(v > 0 for v in sp) for sp in p.speeds))
            self.assertLessEqual(p.est_chord_error, 0.05 + 1e-9)

    def test_min_speed_is_one_warning_per_axis(self) -> None:
        limits = {a: {**RANGE, "min_speed": 12.5} for a in "xy"}
        p = plan(Path.circle((75, 75), 10.0), 20, 0.3, limits=limits)
        self.assertEqual(len([w for w in p.warnings if "documented minimum" in w]), 2)

    def test_axis_range_from_limits_dict(self) -> None:
        rng = AxisRange.of({"units": "mm", "position": {"min": 0.0, "max": 150.0},
                            "speed": {"max": 700.0, "recommended_min": 12.5},
                            "accel_decel": {"max": 1.0}, "accel": {"max": 2.0}})  # fmt: skip
        self.assertEqual(rng, AxisRange((0.0, 150.0), 700.0, 1.0, 12.5, "mm"))

    def test_deviation_helper(self) -> None:
        line = [(0.0, 0.0), (10.0, 0.0)]
        self.assertEqual(path_deviation([(5.0, 1.0), (5.0, -3.0)], line), (3.0, 2.0))
        self.assertAlmostEqual(path_deviation([(12.0, 0.0)], line)[0], 2.0)


class CoordinatedRunTest(unittest.TestCase):
    def setUp(self) -> None:
        logging.disable(logging.WARNING)  # the documented-minimum warnings of the motion checks
        self.ports = sim_ports(2)
        self.cell = Cell.open(self.ports, **OPEN)
        self.xy = self.cell.coordinated(x="SIM1:1", y="SIM2:1")
        self.sims = {"x": self.ports["SIM1"][1], "y": self.ports["SIM2"][1]}
        self.marks = {name: 0 for name in self.ports}

    def tearDown(self) -> None:
        self.cell.close()
        logging.disable(logging.NOTSET)

    def frames(self) -> dict[str, list[dict[str, float]]]:
        """The 0x9900 blocks of each axis since :meth:`clear`."""
        return {"x": _moves(self.ports["SIM1"], start=self.marks["SIM1"]),
                "y": _moves(self.ports["SIM2"], start=self.marks["SIM2"])}  # fmt: skip

    def clear(self) -> None:
        """Count the requests from here on (``line.requests`` is a copy)."""
        self.marks = {name: len(line.requests) for name, line in self.ports.items()}

    def stood_still(self) -> None:
        time.sleep(0.05)
        self.assertFalse(any(s.moving for s in self.sims.values()), {a: s.position_mm for a, s in self.sims.items()})

    def test_sync_line_scales_speeds_and_lands_together(self) -> None:
        self.xy.line_to({"x": 20.0, "y": 30.0}, 60, 0.5)
        self.clear()
        with _Truth(self.sims.values()) as truth:
            result = self.xy.line_to({"x": 80.0, "y": 50.0}, 40, 0.3)
        self.assertTrue(result.completed)
        frames = self.frames()
        self.assertEqual(len(frames["x"]), 1)
        self.assertEqual(len(frames["y"]), 1)
        fx, fy = frames["x"][0], frames["y"][0]
        self.assertAlmostEqual(fx["speed"] / fy["speed"], 60 / 20, delta=0.01)
        self.assertAlmostEqual(math.hypot(fx["speed"], fy["speed"]), 40, delta=0.02)
        self.assertAlmostEqual(fx["accel"] / fy["accel"], 3, delta=0.35)  # 0.01 G steps
        self.assertAlmostEqual(self.sims["x"].position_mm, 80.0, delta=0.1)
        self.assertAlmostEqual(self.sims["y"].position_mm, 50.0, delta=0.1)
        # the point stayed on the straight line: the true path within 0.1 plus what the start skew explains (on the
        # wire: the core reports the skew of the frame calls, which a busy PC can leave short of it)
        ((tx, _),), ((ty, _),) = (_move_frames(self.ports[n], start=self.marks[n]) for n in ("SIM1", "SIM2"))
        self.assertLess(truth.deviation(result.plan.points), 0.1 + 40 * max(result.worst_skew_s, abs(tx - ty)))
        self.assertLess(result.max_deviation, 0.5)  # the estimate from the samples stays in the same range
        self.assertLess(result.worst_skew_s, 0.05)
        json.dumps(result.to_dict())

    def test_streamed_circle_on_two_sims(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        self.clear()
        plan_ = self.xy.plan(Path.circle(CENTER, 10.0), 30, 0.3, **STEP)
        self.assertEqual(plan_.mode, "stream")
        self.assertIsNone(plan_.lead_in)
        self.assertFalse(any("lead-in" in w for w in plan_.warnings))
        with _Truth(self.sims.values()) as truth:
            result = self.xy.run_plan(plan_)
        self.assertTrue(result.completed, result.error)
        # under PC load frames slip (counted, below the abort limit): the bound grows by what the lateness explains
        self.assertLess(truth.deviation(plan_.points), 0.3 + plan_.path_speed * result.worst_late_s)
        self.assertLess(result.max_deviation, 0.5 + plan_.path_speed * result.worst_late_s)
        self.assertGreater(len(result.trace), 20)
        self.assertGreater(result.sample_spacing_s, 0)
        self.assertAlmostEqual(self.sims["x"].position_mm, 85.0, delta=0.1)
        self.assertAlmostEqual(self.sims["y"].position_mm, 75.0, delta=0.1)
        for axis, frames in self.frames().items():
            self.assertGreater(len(frames), plan_.steps // 2, axis)
            for f in frames:
                self.assertTrue(0.0 <= f["target"] <= 150.0, f)
                self.assertTrue(0.01 <= f["speed"] <= 700.0, f)
                self.assertTrue(0.01 <= f["accel"] <= 1.0, f)
                self.assertLessEqual(abs(f["target"] - 75.0), 10.01, f)  # every target on the circle's span
        data = json.loads(json.dumps(result.to_dict()))
        self.assertEqual(data["axes"], ["x", "y"])
        self.assertEqual(data["plan"]["mode"], "stream")
        self.assertEqual(len(data["trace"][0]), 3)

    def test_sync_rectangle_across_ports_stays_on_path(self) -> None:
        # every segment moves one axis on its own port: only the barrier keeps the other port from running ahead
        self.xy.line_to({"x": 20.0, "y": 20.0}, 60, 0.5)
        path = Path.rectangle({"x": 20.0, "y": 20.0}, 20, 15)
        with _Truth(self.sims.values()) as truth:
            result = self.xy.run(path, 40, 0.3)
        self.assertTrue(result.completed)
        self.assertEqual(result.plan.mode, "sync")
        self.assertLess(truth.deviation(result.plan.points), 0.3)
        self.assertLess(result.max_deviation, 0.3)
        self.assertAlmostEqual(self.sims["x"].position_mm, 20.0, delta=0.1)
        self.assertAlmostEqual(self.sims["y"].position_mm, 20.0, delta=0.1)

    def test_fault_alarm_stops_every_axis_of_the_group(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        alarm = _when_moving(self.sims["x"], lambda: self.sims["y"].trigger_alarm(0x0C0), 0.6)  # a quarter turn on
        try:
            with self.assertRaises(CoordinatedMotionError) as ctx:
                self.xy.circle(CENTER, 10.0, 25, 0.3, **STEP)
        finally:
            alarm.join(15)
        err = ctx.exception
        self.assertEqual(err.axis, "y")
        self.assertFalse(err.result.completed)
        self.assertIn("alarm", err.result.error)
        self.assertGreater(len(err.result.samples["x"]), 2)
        self.stood_still()
        self.assertIn("stop", self.sims["x"].events)
        self.assertNotAlmostEqual(self.sims["x"].position_mm, 85.0, delta=0.5)  # stopped on the way, not at the end

    def test_outside_stop_ends_the_run(self) -> None:
        # a stop from another thread (the group's, the cell's, one axis's, a pause) must end the run: no frame after
        # the first STOP / pause frame, every axis stopped, CoordinatedMotionError - never completed=True
        stops = {
            "group stop": lambda: self.xy.stop(),
            "cell stop_all": lambda: self.cell.stop_all(),
            "axis stop": lambda: self.cell["SIM2:1"].stop(),
            "axis pause": lambda: self.cell["SIM1:1"].pause(),
        }
        for label, stop in stops.items():
            with self.subTest(label):
                self.cell.resume_all()
                self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
                self.clear()
                th = _when_moving(self.sims["x"], stop)
                try:
                    with self.assertRaises(CoordinatedMotionError) as ctx:
                        self.xy.circle(CENTER, 10.0, 15, 0.3, **STEP)
                finally:
                    th.join(15)
                self.assertFalse(ctx.exception.result.completed)
                self.assertIn("outside", str(ctx.exception))
                self.stood_still()
                halted = [_first_halt(line, start=self.marks[n]) for n, line in self.ports.items()]
                self.assertTrue(any(t is not None for t in halted), label)
                first = min(t for t in halted if t is not None)
                for name, line in self.ports.items():
                    self.assertEqual(_moves(line, after=first, start=self.marks[name]), [], label)

    def test_stop_from_the_main_thread_ends_a_run_in_a_worker_thread(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        box: dict[str, BaseException | Any] = {}

        def work() -> None:
            try:
                box["result"] = self.xy.circle(CENTER, 10.0, 15, 0.3, **STEP)
            except BaseException as exc:  # noqa: BLE001
                box["error"] = exc

        worker = threading.Thread(target=work)
        worker.start()
        _when_moving(self.sims["x"], lambda: None).join(15)  # well into the run
        self.xy.stop()
        worker.join(10)
        self.assertIsInstance(box.get("error"), CoordinatedMotionError, box)
        self.stood_still()
        positions = {a: s.position_mm for a, s in self.sims.items()}
        time.sleep(0.2)
        self.assertEqual(positions, {a: s.position_mm for a, s in self.sims.items()})  # nothing restarted them

    def test_refused_plan_sends_nothing(self) -> None:
        self.clear()
        with self.assertRaises(SoftLimitError):
            self.xy.run(Path.circle({"x": 145.0, "y": 75.0}, 10.0), 20, 0.3)
        with self.assertRaises(SoftLimitError):
            self.xy.run(Path.circle(CENTER, 10.0), 200, 0.3)  # steps too short for the line
        # a plan made elsewhere: its middle waypoint leaves the range (the core runs only a plan it made)
        with self.assertRaises(NotImplementedError):
            self.xy.run_plan(plan(Path({"x": 73.95, "y": 73.95}, [Polyline([{"x": 160.0}, {"x": 73.95}])]), 20, 0.3))
        self.assertEqual(self.frames(), {"x": [], "y": []})
        with self.assertRaises(ValueError):
            self.xy.run(Path({"z": 0.0}, [Line({"z": 5.0})]), 10, 0.1)  # not an axis of the group

    def test_a_stroke_narrowed_after_planning_is_refused(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        self.assertEqual(self.xy.ranges()["x"].position, (0.0, 150.0))
        plan_ = self.xy.plan(Path.circle(CENTER, 10.0), 20, 0.3, **STEP)
        self.cell["SIM1:1"].stroke = (70.0, 150.0)  # e.g. a fixture added: the circle reaches x = 65
        self.clear()
        with self.assertRaises(SoftLimitError) as ctx:
            self.xy.run_plan(plan_)
        self.assertIn("70..150", str(ctx.exception))
        self.assertEqual(self.frames(), {"x": [], "y": []})
        with self.assertRaisesRegex(SoftLimitError, "70..150"):
            self.xy.plan(Path.circle(CENTER, 10.0), 20, 0.3, **STEP)  # the dry run reads the ranges again too

    def test_lead_in_runs_first_as_a_synchronized_line(self) -> None:
        p = self.xy.plan(Path.circle(CENTER, 10.0, turns=0.5), 20, 0.3, **STEP)
        self.assertIn("lead-in", p.warnings[0])
        self.assertIsNotNone(p.lead_in)
        self.assertEqual(p.lead_in.mode, "sync")
        self.assertAlmostEqual(p.lead_in.points[0][0], self.sims["x"].position_mm, places=6)
        self.assertAlmostEqual(p.points[0][0], 85.0, places=6)
        with _Truth(self.sims.values()) as truth:
            result = self.xy.run_plan(p)
        self.assertTrue(result.completed, result.error)
        self.assertLess(truth.deviation(p.lead_in.points[:1] + p.points), 0.3 + 20 * result.worst_late_s)
        self.assertAlmostEqual(self.sims["x"].position_mm, 65.0, delta=0.1)  # half a turn from 0 deg

    def test_a_circle_a_little_off_radius_needs_no_lead_in(self) -> None:
        # 0.15 mm outside a 40 mm circle: within its 0.5 % tolerance, so it starts where the axes are
        self.xy.line_to({"x": 115.15, "y": 75.0}, 60, 0.5)
        result = self.xy.circle(CENTER, 40.0, 40, 0.3, turns=0.25, **STEP)
        self.assertTrue(result.completed)
        self.assertIsNone(result.plan.lead_in)
        self.assertFalse(any("lead-in" in w for w in result.warnings))

    def test_line_to_where_the_axes_are_returns_at_once(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        self.clear()
        result = self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        self.assertTrue(result.completed)
        self.assertEqual(result.frames, 0)
        self.assertEqual(self.frames(), {"x": [], "y": []})
        self.assertAlmostEqual(result.final["x"], 85.0, delta=0.01)

    def test_link_faults_stop_the_group(self) -> None:
        for kind in ("unplug", "power_off"):
            with self.subTest(kind):
                ports = _latency_ports(2, 0.01)
                with Cell.open(ports, **LATE_OPEN) as cell:
                    xy = cell.coordinated(x="SIM1:1", y="SIM2:1")
                    sx, sy = ports["SIM1"][1], ports["SIM2"][1]
                    xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
                    mark: dict[str, float] = {}

                    def fault() -> None:
                        # well into the circle (the checks before it read the slow line for a while: longer when busy)
                        give_up = time.monotonic() + 10.0
                        while not sx.moving and time.monotonic() < give_up:  # noqa: B023
                            time.sleep(0.002)
                        time.sleep(0.3)
                        mark["t"] = time.monotonic()
                        ports["SIM2"].unplug() if kind == "unplug" else sy.power_off()  # noqa: B023

                    th = threading.Thread(target=fault, daemon=True)
                    th.start()
                    try:
                        with self.assertRaises(CoordinatedMotionError) as ctx:
                            # a late frame and a slow arrival are not the subject: a busy PC must not end the run
                            xy.circle(CENTER, 10.0, 20, 0.3, late_abort_s=10.0, timeout=30.0, **STEP)
                    finally:
                        th.join(12.0)
                    self.assertEqual(ctx.exception.axis, "y")
                    time.sleep(0.05)
                    self.assertFalse(sx.moving)
                    self.assertIn("stop", sx.events)
                    halt = _first_halt(ports["SIM1"])
                    self.assertIsNotNone(halt)
                    # bounded waits: not the bus's retries x timeouts (0.4 s on an idle PC)
                    self.assertLess(halt - mark["t"], 1.0)

    @staticmethod
    def _lose_echo(ec: Elecylinder, unit: SimulatedEC, frame: int) -> Any:
        """A patch of ``Elecylinder._send_direct`` that loses the echo of ``ec``'s ``frame``-th 0x9900 frame."""
        real = Elecylinder._send_direct
        count = [0]

        def send(axis: Elecylinder, *args: Any, **kwargs: Any) -> None:
            if axis is ec:
                count[0] += 1
                if count[0] == frame:
                    unit.inject_fault("timeout")
            real(axis, *args, **kwargs)

        return mock.patch.object(Elecylinder, "_send_direct", send)

    def test_a_path_that_moves_only_through_its_lead_in_gets_there(self) -> None:
        # the path itself moves no axis by a count (a sub-count arc, a stream line to its own start): the lead-in
        # still takes the axes to it - completed=True never stands for a move that did not happen
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        cases = [(Path({"x": 60.0, "y": 60.0}, [Arc((50.0, 60.0), {"x": 60.0, "y": 60.004})]), "auto"),
                 (Path({"x": 40.0, "y": 50.0}, [Line({"x": 40.0, "y": 50.0})]), "stream")]  # fmt: skip
        for path, mode in cases:
            with self.subTest(mode):
                result = self.xy.run(path, 20, 0.3, mode=mode, **STEP)
                self.assertTrue(result.completed, result.error)
                self.assertGreater(result.frames, 0)
                self.assertIn("lead-in", result.warnings[0])
                for axis in "xy":
                    self.assertAlmostEqual(self.sims[axis].position_mm, path.start[axis], delta=0.1)

    def test_ctrl_c_stops_the_group_and_reraises(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        th = _when_moving(self.sims["x"], _thread.interrupt_main)
        try:
            with self.assertRaises(KeyboardInterrupt) as ctx:
                self.xy.circle(CENTER, 10.0, 10, 0.3)
        finally:
            th.join(15)
        result = ctx.exception.coordinated_result  # type: ignore[attr-defined]
        self.assertIsNotNone(result)
        self.assertFalse(result.completed)
        self.assertIn("interrupted", result.error)
        self.stood_still()
        self.assertIn("stop", self.sims["x"].events)
        self.assertIn("stop", self.sims["y"].events)

    def test_one_run_at_a_time(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        self.clear()
        box: dict[str, Any] = {}
        half = Path.circle(CENTER, 10.0, turns=0.5)
        worker = threading.Thread(target=lambda: box.update(result=self.xy.run(half, 15, 0.3, **STEP)))
        worker.start()
        _when_moving(self.sims["x"], lambda: None, 0.05).join(15)  # the first run is under way
        with self.assertRaises(EcNotReadyError):
            self.xy.circle(CENTER, 10.0, 15, 0.3, **STEP)
        worker.join(15)
        result = box["result"]
        self.assertTrue(result.completed)
        # only the first run's frames went out: one per step that moves x
        self.assertEqual(len(self.frames()["x"]), sum(1 for sp in result.plan.speeds if sp[0] > 0))

    def test_axes_held_by_the_path_must_be_ready_too(self) -> None:
        ports = sim_ports(3)
        with Cell.open(ports, **OPEN) as cell:
            xyz = cell.coordinated(x="SIM1:1", y="SIM2:1", z="SIM3:1")
            xyz.line_to({"x": 85.0, "y": 75.0, "z": 20.0}, 60, 0.5)
            for spoil, fix in ((lambda: cell["SIM3:1"].servo(False), lambda: cell["SIM3:1"].servo(True)),
                               (lambda: cell["SIM3:1"].pause(), lambda: cell["SIM3:1"].resume())):  # fmt: skip
                spoil()
                marks = {name: len(line.requests) for name, line in ports.items()}
                with self.assertRaises(EcNotReadyError):
                    xyz.circle({"x": 75.0, "y": 75.0, "z": 20.0}, 10.0, 20, 0.3, **STEP)  # z held at 20
                for name in ("SIM1", "SIM2"):
                    self.assertEqual(_moves(ports[name], start=marks[name]), [], name)
                fix()

    def test_automatic_servo_off_is_woken_the_documented_way(self) -> None:
        ports = sim_ports(2, auto_servo_off_s=0.2)
        with Cell.open(ports, **OPEN) as cell:
            xy = cell.coordinated(x="SIM1:1", y="SIM2:1")
            sx, sy = ports["SIM1"][1], ports["SIM2"][1]
            xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
            time.sleep(0.4)  # well past the delay (a unit switching off in the last ms before a frame is not modelled)
            self.assertFalse(cell["SIM1:1"].status().servo_on)
            self.assertTrue(xy.circle(CENTER, 10.0, 20, 0.3, turns=0.25, **STEP).completed)
            time.sleep(0.4)
            # a rectangle: y stands through the x segments (longer than the delay), x through the y ones
            self.assertTrue(xy.run(Path.rectangle({"x": 75.0, "y": 85.0}, 20, -15), 40, 0.3).completed)
            for unit in (sx, sy):
                self.assertFalse([e for e in unit.events if "0x9900" in e and "servo on" in e], list(unit.events)[-30:])


class CellCoordinatedTest(unittest.TestCase):
    def test_naming_forms(self) -> None:
        with Cell.open(sim_ports(3), **OPEN) as cell:
            self.assertEqual(cell.coordinated(x="SIM1:1", y="SIM3:1").names, ("x", "y"))
            self.assertEqual(cell.coordinated({"a": "SIM2:1"}).names, ("a",))
            self.assertEqual(cell.coordinated("SIM1:1", 1).names, ("SIM1:1", "SIM2:1"))
            self.assertEqual(cell.coordinated().names, cell.names)
            group = cell.coordinated(x=0, y=cell["SIM2:1"])
            self.assertIs(group.axes["y"], cell["SIM2:1"])
            with self.assertRaises(ValueError):
                cell.coordinated(x="SIM1:1", y="SIM1:1")
            with self.assertRaises(KeyError):
                cell.coordinated(x="SIM9:1")

    def test_a_link_fault_follows_the_cells_stop_on_fault(self) -> None:
        # z is outside the group and moving on its own: the cell's stop_on_fault stops it too
        logging.disable(logging.WARNING)
        try:
            ports = sim_ports(3)
            faults: list[tuple[str, str]] = []
            with Cell.open(ports, on_fault=lambda n, k, e: faults.append((n, k)), **OPEN) as cell:
                xy = cell.coordinated(x="SIM1:1", y="SIM2:1")
                xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
                z = ports["SIM3"][1]
                cell["SIM3:1"].move_to(140.0, 10, 0.3, wait=False)
                th = _when_moving(ports["SIM1"][1], ports["SIM2"].unplug)
                try:
                    with self.assertRaises(CoordinatedMotionError):
                        xy.circle(CENTER, 10.0, 15, 0.3, **STEP)
                finally:
                    th.join(15)
                time.sleep(0.05)
                self.assertEqual([n for n, _ in faults], ["SIM2:1"])
                self.assertFalse(z.moving)
                self.assertLess(z.position_mm, 139.0)
                self.assertIn("stop", z.events)
        finally:
            logging.disable(logging.NOTSET)

    def test_a_link_fault_in_a_sync_segment_stops_the_group_before_a_slow_on_fault(self) -> None:
        # y's unit is powered off in the middle of one long sync segment: only the status reads see it (three failed
        # in a row). x's STOP must go out at once - before the cell's fault handling, whose on_fault takes 0.5 s here
        logging.disable(logging.WARNING)
        try:
            ports = _latency_ports(2, 0.01)
            calls: list[tuple[float, str]] = []

            def on_fault(name: str, kind: str, error: BaseException | None) -> None:
                calls.append((time.monotonic(), name))
                time.sleep(0.5)  # a log, a dialog

            with Cell.open(ports, on_fault=on_fault, **LATE_OPEN) as cell:
                xy = cell.coordinated(x="SIM1:1", y="SIM2:1")
                sx, sy = ports["SIM1"][1], ports["SIM2"][1]
                xy.line_to({"x": 10.0, "y": 10.0}, 100, 0.5)
                mark: dict[str, float] = {}

                def fault() -> None:
                    give_up = time.monotonic() + 5.0
                    while not sx.moving and time.monotonic() < give_up:
                        time.sleep(0.002)
                    time.sleep(0.3)
                    mark["t"] = time.monotonic()
                    sy.power_off()

                th = threading.Thread(target=fault, daemon=True)
                th.start()
                try:
                    with self.assertRaises(CoordinatedMotionError) as ctx:
                        xy.line_to({"x": 140.0, "y": 60.0}, 40, 0.3)  # about 3.6 s
                finally:
                    th.join(6.0)
                self.assertEqual(ctx.exception.axis, "y")
                halt = _first_halt(ports["SIM1"])
                self.assertIsNotNone(halt)
                self.assertLess(halt - mark["t"], 1.0)  # 0.4 s on an idle PC: the waits are bounded, not stretched
                self.assertEqual([n for _, n in calls], ["SIM2:1"])
                self.assertLess(halt, calls[0][0])  # the STOP was on the wire before on_fault ran
                time.sleep(0.05)
                self.assertFalse(sx.moving)
                self.assertLess(sx.position_mm, 100.0)
        finally:
            logging.disable(logging.NOTSET)


class CoreGroupTest(unittest.TestCase):
    """``CoordinatedGroup``: the core plans, runs and stops; the records are the shared ones."""

    def setUp(self) -> None:
        logging.disable(logging.WARNING)  # the documented-minimum warnings of the motion checks
        self.addCleanup(logging.disable, logging.NOTSET)
        self.ports = sim_ports(2)
        self.cell = self.enterContext(Cell.open(self.ports, **OPEN))
        self.xy = self.cell.coordinated(x="SIM1:1", y="SIM2:1")
        self.sims = {"x": self.ports["SIM1"][1], "y": self.ports["SIM2"][1]}

    def marks(self) -> dict[str, int]:
        return {name: len(line.requests) for name, line in self.ports.items()}

    def moves(self, marks: dict[str, int]) -> list[dict[str, float]]:
        """The 0x9900 blocks on both ports since ``marks``."""
        return [m for name, line in self.ports.items() for m in _moves(line, start=marks[name])]

    def test_the_public_names_are_the_cores_group(self) -> None:
        self.assertIs(iai_actuator_driver.CoordinatedGroup, CoordinatedGroup)
        self.assertIs(iai_actuator_driver.coord.CoordinatedGroup, CoordinatedGroup)
        self.assertIsInstance(self.xy, CoordinatedGroup)
        self.assertEqual(type(self.xy).__module__, "iai_actuator_driver.coord")
        self.assertIs(iai_actuator_driver.MotionPlan, MotionPlan)  # the records are the shared ones
        self.assertEqual(self.xy.names, ("x", "y"))
        self.assertEqual(repr(self.xy), "CoordinatedGroup({'x': 'SIM1:1', 'y': 'SIM2:1'})")

    def test_plan_and_run_plan_give_the_shared_records(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        plan_ = self.xy.plan(Path.circle(CENTER, 10.0, turns=0.5), 25, 0.3, **STEP)
        self.assertIsInstance(plan_, MotionPlan)
        self.assertEqual((plan_.mode, plan_.axes), ("stream", ("x", "y")))
        self.assertIsNotNone(plan_.timing.measured_write_s)  # the core measured the line for this plan
        self.assertEqual(set(plan_.ranges), {"x", "y"})
        self.assertIn("stream plan over x, y", plan_.summary())
        self.assertEqual(plan_.table()[0]["k"], 0)
        result = self.xy.run_plan(plan_)
        self.assertIsInstance(result, CoordinatedResult)
        self.assertTrue(result.completed, result.error)
        self.assertIs(self.xy.last_result, result)
        self.assertEqual(result.plan.mode, "stream")
        self.assertEqual(len(result.trace[0]), 3)
        self.assertAlmostEqual(self.sims["x"].position_mm, 65.0, delta=0.1)  # half a turn
        self.assertEqual(json.loads(json.dumps(result.to_dict()))["axes"], ["x", "y"])
        self.assertIn("completed", result.summary())

    def test_what_the_core_does_not_have_says_so(self) -> None:
        with self.assertRaisesRegex(NotImplementedError, "plan.timing"):
            self.xy.timing()
        marks = self.marks()
        made = plan(Path({"x": 73.95, "y": 73.95}, [Line({"x": 80.0})]), 20, 0.3)
        with self.assertRaisesRegex(NotImplementedError, "group.plan"):
            self.xy.run_plan(made)
        self.assertEqual(self.moves(marks), [])

    def test_run_plan_refuses_a_plan_whose_start_the_axes_left(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        plan_ = self.xy.plan(Path.circle(CENTER, 10.0, turns=0.25), 25, 0.3, **STEP)
        self.xy.line_to({"x": 80.0, "y": 75.0}, 60, 0.5)
        marks = self.marks()
        with self.assertRaisesRegex(EcNotReadyError, "plan again"):
            self.xy.run_plan(plan_)
        self.assertEqual(self.moves(marks), [])

    def test_a_refusal_names_the_reasons_and_the_largest_feasible_speed(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        marks = self.marks()
        with self.assertRaises(SoftLimitError) as ctx:
            self.xy.plan(Path.circle(CENTER, 10.0), 200, 0.3, **STEP)  # steps too short for the line
        reasons, best = ctx.exception.reasons, ctx.exception.max_speed  # type: ignore[attr-defined]
        self.assertTrue(reasons)
        self.assertIsInstance(best, float)
        self.assertLess(best, 200)
        self.assertEqual(self.xy.plan(Path.circle(CENTER, 10.0), best, 0.3, **STEP).mode, "stream")
        with self.assertRaisesRegex(ValueError, "finite"):
            self.xy.run(Path.circle(CENTER, 10.0), 20, 0.3, timeout=math.inf)
        with self.assertRaises(ValueError):  # the core refuses a timeout that is not above 0
            self.xy.run(Path.circle(CENTER, 10.0), 20, 0.3, timeout=0)
        self.assertEqual(self.moves(marks), [])

    def test_a_stopped_run_names_the_axis_the_stop_and_the_cause(self) -> None:
        self.xy.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        alarm = _when_moving(self.sims["x"], lambda: self.sims["y"].trigger_alarm(0x0C0))
        try:
            with self.assertRaises(CoordinatedMotionError) as ctx:
                self.xy.circle(CENTER, 10.0, 25, 0.3, **STEP)
        finally:
            alarm.join(15)
        err = ctx.exception
        self.assertEqual(err.axis, "y")
        self.assertIsInstance(err.result, CoordinatedResult)
        self.assertFalse(err.result.completed)
        self.assertIs(self.xy.last_result, err.result)
        self.assertIn("x", err.stop)
        self.assertLessEqual(set(err.stop), {"x", "y"})  # the path names, not the cell's
        self.assertIsInstance(err.__cause__, EcAlarmError)

    def test_a_group_built_directly_runs_on_a_core_cell_of_its_own(self) -> None:
        line = sim_line([1, 2], name="L")
        bus = ModbusRTU(line)
        self.addCleanup(bus.close)
        x, y = Elecylinder(bus, slave=1, poll_interval_s=0.005), Elecylinder(bus, slave=2, poll_interval_s=0.005)
        group = CoordinatedGroup({"x": x, "y": y})
        result = group.run(Path.rectangle({"x": 73.95, "y": 73.95}, 10, 5), 40, 0.3, blend=0.5)
        self.assertTrue(result.completed, result.error)
        self.assertEqual(result.plan.steps, 4)
        self.assertAlmostEqual(line[2].position_mm, 73.95, delta=0.1)
        group.line_to({"x": 85.0, "y": 75.0}, 60, 0.5)
        circle = group.circle(CENTER, 10.0, 15, 0.3, turns=0.25, min_step_s=LineTiming(axes_on_port=2).min_step_s)
        self.assertTrue(circle.completed, circle.error)
        self.assertEqual(circle.plan.timing.axes_on_port, 2)  # both axes on the one port
        stopped = group.stop()
        self.assertEqual(set(stopped), {"x", "y"})
        self.assertTrue(all(st.position_mm > 0 for st in stopped.values()), stopped)
        del group  # its own core cell is released; the axes stay open
        self.assertGreater(x.status().position_mm, 0)


if __name__ == "__main__":
    unittest.main()
