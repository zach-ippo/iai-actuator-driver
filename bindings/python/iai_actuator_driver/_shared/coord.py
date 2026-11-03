"""The Python part of ``iai_actuator_driver.coord``: the geometry (:class:`Path` and its segments), the
planner (:func:`plan`), :class:`AxisRange`, :class:`LineTiming`, the records (:class:`MotionPlan`,
:class:`CoordinatedResult`, :class:`CoordinatedMotionError`) and the timing constants. Pure Python, no I/O.

The planner runs every check before anything is sent: every commanded waypoint inside each axis's range, every per-axis
speed and acceleration within the axis's limits, the centripetal acceleration ``v^2 / r`` within the smallest axis
acceleration, and the step time not shorter than what the line can carry (:class:`LineTiming`; ``min_step_s``
overrides it). An infeasible path raises :class:`~iai_actuator_driver.errors.SoftLimitError` naming the reason and the
largest feasible speed (``.max_speed``); nothing is sent. ``mode="sync"`` runs each segment as one move per axis,
``mode="stream"`` cuts the path into chords with a time per waypoint, ``mode="auto"`` picks sync for a path made only
of lines.

``iai_actuator_driver.coord`` (the core plans and runs; the rules of a run and the evidence are in its documentation)
imports these names from here.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Union

from .. import registers as R
from ..errors import EcDriverError, SoftLimitError
from . import publish

G = R.G_MM_S2  # 1 G in units/s^2 (IAI: 9800 mm/s^2)
MODES = ("auto", "sync", "stream")
LINE_STEP_S = 0.1  # stream mode: a straight piece is cut into steps of at most this long at full speed
MAX_CHORD_ANGLE = math.pi / 2  # at most a quarter turn per chord, whatever the chord error allows
MIN_RADIUS = 0.01  # an arc or circle radius below one position count is refused
START_LEAD_S = 0.05  # the schedule starts this long after the port threads are launched
JITTER_S = 0.002  # PC timing jitter assumed by the estimates (1 ms timer)
LATE_FRACTION = 0.1  # a frame later than this share of its step (and 2 ms) counts as late
SILENT_READS = 3  # failed status reads in a row on one axis = a link fault
POLL_S = 0.005  # status poll gap while waiting for the last waypoint
SAMPLE_BOUND_S = 0.15  # stream: an axis unread this long is read even if the next frame goes late (twice: abort)
SAMPLE_GAP_S = 0.02  # stream: while the slack allows, each axis is read about this often
WAIT_FACTOR = 3.0  # during a run every transaction waits at most this many times its measured (or modelled) time ...
WAIT_MIN_S = 0.05  # ... and at least this long
WATCH_S = 0.01  # the main thread looks at the ports' errors this often
SLOW_BAUD = 38400  # below this rate every reply is assumed as slow as the worst one seen at 9600 ...
SLOW_REPLY_S = 0.285  # ... (VERIFIED on the EC gripper at 9600: replies alternate ~65 / ~285 ms)
WAKE_ROUNDS = 3  # automatic servo OFF checks before a segment / a stream, repeated while one had to wake an axis
MEASURE_TOLERANCE = 0.8  # run_plan re-measures the line: a plan is refused below this share of the new minimum step
MEASURE_SAMPLES = 3  # the line measurement times this many transactions of each kind per axis and keeps the median


# =========================================================================== geometry


Point = Mapping[str, float]


def _number(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number, got {value!r}")
    return float(value)


def _positive(value: Any, what: str) -> float:
    number = _number(value, what)
    if number <= 0:
        raise ValueError(f"{what} must be > 0, got {value!r}")
    return number


def _point(value: Any, what: str) -> dict[str, float]:
    if not isinstance(value, Mapping) or not value:
        raise ValueError(f"{what} must be a non-empty mapping {{axis name: position}}, got {value!r}")
    out: dict[str, float] = {}
    for key, pos in value.items():
        if not isinstance(key, str) or not key:
            raise ValueError(f"{what}: axis names must be strings, got {key!r}")
        out[key] = _number(pos, f"{what}[{key!r}]")
    return out


def _plane(plane: Sequence[str]) -> tuple[str, str]:
    try:
        a, b = plane
    except (TypeError, ValueError):
        raise ValueError(f"plane must be two axis names, got {plane!r}") from None
    if not isinstance(a, str) or not isinstance(b, str) or a == b:
        raise ValueError(f"plane must be two different axis names, got {plane!r}")
    return a, b


def _pair(value: Any, plane: tuple[str, str], what: str) -> tuple[float, float]:
    """A point in the plane: ``{plane[0]: .., plane[1]: ..}`` (other keys ignored) or a 2-sequence."""
    if isinstance(value, Mapping):
        missing = [a for a in plane if a not in value]
        if missing:
            raise ValueError(f"{what} must give both plane axes {plane}, missing {missing}")
        return _number(value[plane[0]], what), _number(value[plane[1]], what)
    try:
        a, b = value
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be (a, b) or {{axis: position}}, got {value!r}") from None
    return _number(a, what), _number(b, what)


def _others(value: Any, plane: tuple[str, str]) -> dict[str, float]:
    """The axes of a mapping outside the plane (held positions), ``{}`` for a sequence."""
    if not isinstance(value, Mapping):
        return {}
    return {k: v for k, v in _point(value, "point").items() if k not in plane}


@dataclass(frozen=True, init=False)
class Line:
    """A straight move to ``to`` (``{axis: position}``); axes not named hold their position."""

    to: dict[str, float]

    def __init__(self, to: Point) -> None:
        object.__setattr__(self, "to", _point(to, "Line(to)"))


@dataclass(frozen=True, init=False)
class Arc:
    """A circular arc in ``plane`` from the current point around ``center`` to ``end`` (``ccw``: counter-clockwise
    seen with ``plane[0]`` to the right and ``plane[1]`` up). ``end`` == the start is a full circle. Axes of ``end``
    outside the plane move linearly along the arc (a helix); the others hold. The end must lie on the circle through
    the start (0.5 % of the radius, at least 0.01)."""

    center: tuple[float, float]
    end: dict[str, float]
    ccw: bool
    plane: tuple[str, str]

    def __init__(self, center: Point | Sequence[float], end: Point, ccw: bool = True, *,
                 plane: Sequence[str] = ("x", "y")) -> None:  # fmt: skip
        p = _plane(plane)
        object.__setattr__(self, "plane", p)
        object.__setattr__(self, "center", _pair(center, p, "Arc center"))
        object.__setattr__(self, "end", _point(end, "Arc end"))
        object.__setattr__(self, "ccw", bool(ccw))


@dataclass(frozen=True, init=False)
class Circle:
    """``turns`` full turns (fractions allowed) in ``plane`` around ``center`` with ``radius``, starting and (for whole
    turns) ending at the angle ``start_deg``. The path must already be at that start point (:meth:`Path.circle` and
    :meth:`CoordinatedGroup.circle` put it there)."""

    center: tuple[float, float]
    radius: float
    start_deg: float
    turns: float
    ccw: bool
    plane: tuple[str, str]

    def __init__(self, center: Point | Sequence[float], radius: float, start_deg: float = 0.0, turns: float = 1.0,
                 ccw: bool = True, *, plane: Sequence[str] = ("x", "y")) -> None:  # fmt: skip
        p = _plane(plane)
        object.__setattr__(self, "plane", p)
        object.__setattr__(self, "center", _pair(center, p, "Circle center"))
        object.__setattr__(self, "radius", _positive(radius, "Circle radius"))
        object.__setattr__(self, "start_deg", _number(start_deg, "Circle start_deg"))
        object.__setattr__(self, "turns", _positive(turns, "Circle turns"))
        object.__setattr__(self, "ccw", bool(ccw))

    def point_at(self, deg: float) -> dict[str, float]:
        """The point of the circle at ``deg`` (plane axes only)."""
        rad = math.radians(deg)
        return {self.plane[0]: self.center[0] + self.radius * math.cos(rad),
                self.plane[1]: self.center[1] + self.radius * math.sin(rad)}  # fmt: skip

    @property
    def start(self) -> dict[str, float]:
        return self.point_at(self.start_deg)


@dataclass(frozen=True, init=False)
class Polyline:
    """Straight moves through ``points`` one after another (each a ``{axis: position}``)."""

    points: tuple[dict[str, float], ...]

    def __init__(self, points: Iterable[Point]) -> None:
        pts = tuple(_point(p, f"Polyline point {i}") for i, p in enumerate(points))
        if not pts:
            raise ValueError("Polyline needs at least one point")
        object.__setattr__(self, "points", pts)


Segment = Union[Line, Arc, Circle, Polyline]


@dataclass(frozen=True)
class _Piece:
    """One resolved piece of a path: straight (``plane`` None) or an arc; both carry every axis of the path."""

    start: dict[str, float]
    end: dict[str, float]
    plane: tuple[str, str] | None = None
    center: tuple[float, float] = (0.0, 0.0)
    r0: float = 0.0
    r1: float = 0.0
    a0: float = 0.0
    sweep: float = 0.0  # signed, radians

    @property
    def straight(self) -> bool:
        return self.plane is None

    def at(self, f: float) -> dict[str, float]:
        """The point at fraction ``f`` (0..1) of the piece."""
        if f >= 1.0:
            return dict(self.end)
        out = {k: s + (self.end[k] - s) * f for k, s in self.start.items()}
        if self.plane is not None:
            a, b = self.plane
            r = self.r0 + (self.r1 - self.r0) * f
            angle = self.a0 + self.sweep * f
            out[a] = self.center[0] + r * math.cos(angle)
            out[b] = self.center[1] + r * math.sin(angle)
        return out

    def length(self) -> float:
        if self.plane is None:
            return math.dist(tuple(self.start.values()), tuple(self.end[k] for k in self.start))
        rest = math.sqrt(sum((self.end[k] - s) ** 2 for k, s in self.start.items() if k not in self.plane))
        return math.hypot((self.r0 + self.r1) / 2 * abs(self.sweep), rest)


def _close(a: Mapping[str, float], b: Mapping[str, float], tol: float = 1e-9) -> bool:
    return all(abs(a[k] - b[k]) <= tol for k in a)


class Path:
    """A start point plus segments (:class:`Line`, :class:`Arc`, :class:`Circle`, :class:`Polyline`).

    ``start`` ``None``: the path starts wherever the axes are (``CoordinatedGroup.plan`` reads them). ``then()``
    appends segments and returns the path, so ``Path(start).then(Line(a), Arc(c, b))`` chains."""

    def __init__(self, start: Point | None = None, segments: Iterable[Segment] = ()) -> None:
        self.start: dict[str, float] | None = None if start is None else _point(start, "Path start")
        self.segments: list[Segment] = []
        self.then(*segments)

    def then(self, *segments: Segment) -> Path:
        for seg in segments:
            if not isinstance(seg, (Line, Arc, Circle, Polyline)):
                raise TypeError(f"a path segment must be Line, Arc, Circle or Polyline, got {seg!r}")
            self.segments.append(seg)
        return self

    def __repr__(self) -> str:
        return f"Path(start={self.start}, segments={self.segments})"

    @property
    def axes(self) -> tuple[str, ...]:
        """Every axis the path names, start first, in order of appearance."""
        names: dict[str, None] = dict.fromkeys(self.start or {})
        for seg in self.segments:
            if isinstance(seg, Line):
                names.update(dict.fromkeys(seg.to))
            elif isinstance(seg, Polyline):
                for p in seg.points:
                    names.update(dict.fromkeys(p))
            elif isinstance(seg, Arc):
                names.update(dict.fromkeys(seg.plane))
                names.update(dict.fromkeys(seg.end))
            else:
                names.update(dict.fromkeys(seg.plane))
        return tuple(names)

    @property
    def straight(self) -> bool:
        """True when the path is made only of lines (``mode="auto"`` runs it synchronized)."""
        return all(isinstance(seg, (Line, Polyline)) for seg in self.segments)

    def pieces(self, start: Point | None = None) -> list[_Piece]:
        """The resolved pieces from ``start`` (default: the path's own), every piece carrying every axis. Zero-length
        lines are dropped. Raises ``ValueError`` for an axis without a start position, an arc whose end is off its
        circle, or a :class:`Circle` the path is not at the start of."""
        origin = self.start if start is None else _point(start, "start")
        if origin is None:
            raise ValueError("the path has no start point: give Path(start=...) or plan it from the axes' positions")
        axes = dict.fromkeys(self.axes) | dict.fromkeys(origin)
        missing = [a for a in axes if a not in origin]
        if missing:
            raise ValueError(f"no start position for axis/axes {missing}")
        cur = {a: origin[a] for a in axes}
        out: list[_Piece] = []

        def line(to: Mapping[str, float]) -> None:
            nonlocal cur
            end = {**cur, **to}
            if not _close(cur, end):
                out.append(_Piece(dict(cur), end))
            cur = end

        for seg in self.segments:
            if isinstance(seg, Line):
                line(seg.to)
            elif isinstance(seg, Polyline):
                for p in seg.points:
                    line(p)
            elif isinstance(seg, Arc):
                piece = _arc_piece(cur, seg)
                out.append(piece)
                cur = dict(piece.end)
            else:
                piece = _circle_piece(cur, seg)
                out.append(piece)
                cur = dict(piece.end)
        return out

    def length(self, start: Point | None = None) -> float:
        """The exact path length (arcs as arcs), in the axes' units."""
        return sum(p.length() for p in self.pieces(start))

    # -- demo shapes -------------------------------------------------------------------------------

    @classmethod
    def circle(cls, center: Point | Sequence[float], radius: float, *, plane: Sequence[str] = ("x", "y"),
               start_deg: float = 0.0, turns: float = 1.0, ccw: bool = True) -> Path:  # fmt: skip
        """A path that starts on the circle at ``start_deg`` and runs ``turns`` around it. Axes of a ``center`` mapping
        outside the plane are held at those positions."""
        seg = Circle(center, radius, start_deg, turns, ccw, plane=plane)
        return cls({**_others(center, seg.plane), **seg.start}, [seg])

    @classmethod
    def rectangle(cls, corner: Point | Sequence[float], width: float, height: float, *,
                  plane: Sequence[str] = ("x", "y")) -> Path:  # fmt: skip
        """A closed rectangle from ``corner`` (``width`` along ``plane[0]``, ``height`` along ``plane[1]``; negative
        values go the other way), four lines back to the corner."""
        p = _plane(plane)
        x0, y0 = _pair(corner, p, "rectangle corner")
        w, h = _number(width, "width"), _number(height, "height")
        if w == 0 or h == 0:
            raise ValueError("rectangle width and height must not be 0")
        a, b = p
        corners = [(x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h), (x0, y0)]
        return cls({**_others(corner, p), a: x0, b: y0}, [Polyline([{a: x, b: y} for x, y in corners])])

    @classmethod
    def polygon(cls, center: Point | Sequence[float], radius: float, sides: int, *, plane: Sequence[str] = ("x", "y"),
                start_deg: float = 0.0) -> Path:  # fmt: skip
        """A closed regular polygon with ``sides`` corners on the circle (``center``, ``radius``), the first corner at
        ``start_deg``."""
        if isinstance(sides, bool) or not isinstance(sides, int) or sides < 3:
            raise ValueError(f"sides must be an integer >= 3, got {sides!r}")
        ring = Circle(center, radius, start_deg, plane=plane)
        corners = [ring.point_at(start_deg + 360.0 * k / sides) for k in range(1, sides + 1)]
        corners[-1] = ring.start
        return cls({**_others(center, ring.plane), **ring.start}, [Polyline(corners)])


def _arc_piece(cur: dict[str, float], seg: Arc) -> _Piece:
    a, b = seg.plane
    for axis in seg.plane:
        if axis not in cur:
            raise ValueError(f"no start position for axis {axis!r} of the arc's plane")
    cx, cy = seg.center
    end = {**cur, **seg.end}
    r0 = math.hypot(cur[a] - cx, cur[b] - cy)
    r1 = math.hypot(end[a] - cx, end[b] - cy)
    if r0 < 1e-6:
        raise ValueError(f"the arc starts at its center ({cx:g}, {cy:g})")
    if r1 < 1e-6:
        raise ValueError(f"the arc ends at its center ({cx:g}, {cy:g})")
    if min(r0, r1) < MIN_RADIUS:
        raise ValueError(f"the arc's radius ({min(r0, r1):g}) is below one position count ({MIN_RADIUS:g})")
    if abs(r1 - r0) > max(0.01, 0.005 * r0):
        raise ValueError(f"the arc's end is not on its circle: radius {r0:.3f} at the start, {r1:.3f} at the end")
    a0 = math.atan2(cur[b] - cy, cur[a] - cx)
    a1 = math.atan2(end[b] - cy, end[a] - cx)
    if seg.ccw:
        sweep = (a1 - a0) % (2 * math.pi)
        if sweep < 1e-9 or sweep > 2 * math.pi - 1e-9:
            sweep = 2 * math.pi
    else:
        sweep = -((a0 - a1) % (2 * math.pi))
        if sweep > -1e-9 or sweep < -2 * math.pi + 1e-9:
            sweep = -2 * math.pi
    return _Piece(dict(cur), end, seg.plane, seg.center, r0, r1, a0, sweep)


def _circle_piece(cur: dict[str, float], seg: Circle) -> _Piece:
    a, b = seg.plane
    for axis in seg.plane:
        if axis not in cur:
            raise ValueError(f"no start position for axis {axis!r} of the circle's plane")
    if seg.radius < MIN_RADIUS:
        raise ValueError(f"the circle's radius ({seg.radius:g}) is below one position count ({MIN_RADIUS:g})")
    start = seg.start
    tol = max(0.01, 0.005 * seg.radius)
    if math.hypot(cur[a] - start[a], cur[b] - start[b]) > tol:
        raise ValueError(
            f"the circle starts at ({start[a]:.3f}, {start[b]:.3f}) but the path is at ({cur[a]:.3f}, {cur[b]:.3f}): "
            "add a Line to the circle's start first (Path.circle and CoordinatedGroup.circle do)"
        )
    sweep = 2 * math.pi * seg.turns * (1 if seg.ccw else -1)
    whole = abs(seg.turns - round(seg.turns)) < 1e-9
    end_pt = start if whole else seg.point_at(seg.start_deg + math.degrees(sweep))
    end = {**cur, **end_pt} if not whole else dict(cur)
    a0 = math.radians(seg.start_deg)
    return _Piece(dict(cur), end, seg.plane, seg.center, seg.radius, seg.radius, a0, sweep)


# =========================================================================== limits and line timing


def _opt(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


@dataclass(frozen=True)
class AxisRange:
    """The ranges the planner checks one axis against: ``position`` (min, max), ``max_speed`` (units/s),
    ``max_accel_g`` (one value for accel and decel, as a 0x9900 move uses), ``min_speed`` (the documented minimum: a
    warning, never a refusal). ``None`` = not checked. :meth:`of` takes an ``Elecylinder.limits()`` dict too."""

    position: tuple[float, float] | None = None
    max_speed: float | None = None
    max_accel_g: float | None = None
    min_speed: float | None = None
    units: str = "mm"

    @classmethod
    def of(cls, value: AxisRange | Mapping[str, Any] | None) -> AxisRange:
        """From an ``AxisRange``, an ``ax.limits()`` dict, or ``{"position": (lo, hi), "speed": max, "accel_g": max,
        "min_speed": v}``."""
        if value is None:
            return cls()
        if isinstance(value, AxisRange):
            return value
        pos = value.get("position")
        if isinstance(pos, Mapping):  # the Elecylinder.limits() dict
            lo, hi = _opt(pos.get("min")), _opt(pos.get("max"))
            speed = value.get("speed") or {}
            accel = value.get("accel_decel") or value.get("accel") or {}
            return cls(
                position=None if lo is None or hi is None else (lo, hi), max_speed=_opt(speed.get("max")),
                max_accel_g=_opt(accel.get("max")), min_speed=_opt(speed.get("recommended_min")),
                units=str(value.get("units") or "mm"),
            )  # fmt: skip
        position = None if pos is None else (float(pos[0]), float(pos[1]))
        return cls(position=position, max_speed=_opt(value.get("speed", value.get("max_speed"))),
                   max_accel_g=_opt(value.get("accel_g", value.get("max_accel_g"))),
                   min_speed=_opt(value.get("min_speed")), units=str(value.get("units", "mm")))  # fmt: skip

    def to_dict(self) -> dict[str, Any]:
        return {"position": None if self.position is None else list(self.position), "max_speed": self.max_speed,
                "max_accel_g": self.max_accel_g, "min_speed": self.min_speed, "units": self.units}  # fmt: skip


WRITE_FRAME_BYTES = (27, 8)  # FC10, 9 words at 0x9900: request 27 bytes, echo 8
READ_FRAME_BYTES = (8, 21)  # FC03, 8 words at 0x9000 (the lean status): request 8 bytes, answer 21


@dataclass(frozen=True)
class LineTiming:
    """How often one port can re-target its axes: the estimate behind the planner's shortest step (the model is
    short of the EC gripper's measured 82 / 53 ms at 38400 / 230400 baud, VERIFIED: the measured times count). Per transaction the model takes the bytes on the wire (10 bits each), the unit's response delay RTIM
    (EC item 8.4 / CON No.17, 5 ms by default, DOC; below 38400 baud at least the ~285 ms reply VERIFIED at 9600), the
    bus's minimum gap between transactions and an allowance for the USB adapter and the PC (``turnaround_s``).
    ``measured_write_s`` / ``measured_read_s``: what one transaction took on this line just before (a full status read,
    longer on the wire than the 0x9900 write, and a lean status read; per axis the median of :data:`MEASURE_SAMPLES`,
    the slowest axis); the frame and read times are at least those.
    Per step and axis one 0x9900 write plus ``reads_per_step`` lean status reads (the samples); the axes of a port take
    turns; ``margin`` covers the rest."""

    baudrate: int = 115200
    axes_on_port: int = 1
    response_ms: float = 5.0
    gap_s: float = 0.005
    turnaround_s: float = 0.002
    reads_per_step: int = 1
    margin: float = 1.5
    measured_write_s: float | None = None
    measured_read_s: float | None = None

    def frame_s(self, request_bytes: int, response_bytes: int) -> float:
        """One transaction by the model (no measurement), seconds."""
        wire = 10.0 * (request_bytes + response_bytes) / max(1, self.baudrate)
        reply = self.response_ms / 1000.0
        if self.baudrate < SLOW_BAUD:
            reply = max(reply, SLOW_REPLY_S)
        return wire + reply + self.gap_s + self.turnaround_s

    @property
    def write_s(self) -> float:
        return max(self.frame_s(*WRITE_FRAME_BYTES), self.measured_write_s or 0.0)

    @property
    def read_s(self) -> float:
        return max(self.frame_s(*READ_FRAME_BYTES), self.measured_read_s or 0.0)

    @property
    def command_period_s(self) -> float:
        """One step of the port: every axis written once and read ``reads_per_step`` times."""
        return self.axes_on_port * (self.write_s + self.reads_per_step * self.read_s)

    @property
    def min_step_s(self) -> float:
        return self.command_period_s * self.margin

    @property
    def model(self) -> LineTiming:
        """The same line by the model alone (the measurements dropped)."""
        return replace(self, measured_write_s=None, measured_read_s=None)

    def to_dict(self) -> dict[str, Any]:
        def r5(v: float | None) -> float | None:
            return None if v is None else round(v, 5)

        return {"baudrate": self.baudrate, "axes_on_port": self.axes_on_port, "response_ms": self.response_ms,
                "gap_s": self.gap_s, "turnaround_s": self.turnaround_s, "reads_per_step": self.reads_per_step,
                "margin": self.margin, "measured_write_s": r5(self.measured_write_s),
                "measured_read_s": r5(self.measured_read_s), "write_s": r5(self.write_s), "read_s": r5(self.read_s),
                "command_period_s": r5(self.command_period_s), "min_step_s": r5(self.min_step_s)}  # fmt: skip


# =========================================================================== the planner


@dataclass
class MotionPlan:
    """A planned coordinated move (no I/O). ``points[k]`` are the waypoints (``points[0]`` the start) in ``axes``
    order, ``times[k]`` their planned times (s), ``speeds[k]`` / ``accels[k]`` what each axis is commanded for step
    ``k`` (``points[k]`` -> ``points[k + 1]``; units/s and G, 0 = no frame: the axis's target does not change).
    ``sync``: one step per segment; ``stream``: one step per chord. ``lead_in``: a synchronized line from where the
    axes were to ``points[0]``, run first. ``min_step_override``: ``min_step_s`` was given (the run then does not
    check the steps against the line again)."""

    mode: str
    axes: tuple[str, ...]
    points: list[tuple[float, ...]]
    times: list[float]
    speeds: list[tuple[float, ...]]
    accels: list[tuple[float, ...]]
    path_speed: float
    accel_g: float
    chord_error: float
    length: float
    band: float
    min_step_s: float
    est_chord_error: float
    est_timing_error_s: float
    est_deviation: float
    blend: float | None = None
    warnings: list[str] = field(default_factory=list)
    timing: LineTiming | None = None
    ranges: dict[str, AxisRange] = field(default_factory=dict)
    lead_in: MotionPlan | None = None
    min_step_override: bool = False

    @property
    def steps(self) -> int:
        return len(self.points) - 1

    @property
    def duration(self) -> float:
        return self.times[-1]

    @property
    def start(self) -> dict[str, float]:
        return self.waypoint(0)

    @property
    def end(self) -> dict[str, float]:
        return self.waypoint(-1)

    @property
    def shortest_step_s(self) -> float:
        return min((b - a for a, b in zip(self.times, self.times[1:])), default=0.0)

    def waypoint(self, k: int) -> dict[str, float]:
        return dict(zip(self.axes, self.points[k]))

    def max_speed(self, axis: str) -> float:
        i = self.axes.index(axis)
        return max((s[i] for s in self.speeds), default=0.0)

    def max_accel(self, axis: str) -> float:
        i = self.axes.index(axis)
        return max((a[i] for a in self.accels), default=0.0)

    def moves(self, axis: str) -> bool:
        """True when ``axis`` gets at least one frame."""
        return self.max_speed(axis) > 0

    def summary(self) -> str:
        units = next((r.units for r in self.ranges.values()), "mm")
        lines = [
            f"{self.mode} plan over {', '.join(self.axes)}: {self.steps} step(s), {self.length:.2f} {units} in "
            f"{self.duration:.2f} s at {self.path_speed:g} {units}/s ({self.accel_g:g} G ramps)",
            f"shortest step {self.shortest_step_s * 1000:.1f} ms (the line allows {self.min_step_s * 1000:.1f} ms); "
            f"chord error <= {self.est_chord_error:.3f}; timing error ~{self.est_timing_error_s * 1000:.1f} ms; "
            f"estimated deviation ~{self.est_deviation:.3f} {units}",
        ]
        if self.lead_in is not None:
            lines.append(f"lead-in first: a synchronized line from {self.lead_in.start} ({self.lead_in.length:.2f} "
                         f"{units}, {self.lead_in.duration:.2f} s)")  # fmt: skip
        lines += [f"warning: {w}" for w in self.warnings]
        return "\n".join(lines)

    def table(self) -> list[dict[str, Any]]:
        """One row per waypoint: ``k``, ``t``, the positions and (for the step that ends there) the axis speeds."""
        rows = []
        for k, (t, p) in enumerate(zip(self.times, self.points)):
            row: dict[str, Any] = {"k": k, "t": round(t, 4)}
            row.update({a: round(v, 3) for a, v in zip(self.axes, p)})
            if k > 0:
                row.update({f"v_{a}": s for a, s in zip(self.axes, self.speeds[k - 1])})
            rows.append(row)
        return rows

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode, "axes": list(self.axes),
            "points": [[round(v, 4) for v in p] for p in self.points], "times": [round(t, 5) for t in self.times],
            "speeds": [list(s) for s in self.speeds], "accels": [list(a) for a in self.accels],
            "path_speed": self.path_speed, "accel_g": self.accel_g, "chord_error": self.chord_error,
            "length": round(self.length, 4), "duration": round(self.duration, 4), "band": self.band, "blend": self.blend,
            "steps": self.steps, "min_step_s": round(self.min_step_s, 5),
            "shortest_step_s": round(self.shortest_step_s, 5), "est_chord_error": round(self.est_chord_error, 5),
            "est_timing_error_s": round(self.est_timing_error_s, 5), "est_deviation": round(self.est_deviation, 5),
            "warnings": list(self.warnings), "timing": None if self.timing is None else self.timing.to_dict(),
            "ranges": {a: r.to_dict() for a, r in self.ranges.items()},
            "lead_in": None if self.lead_in is None else self.lead_in.to_dict(),
            "min_step_override": self.min_step_override,
        }  # fmt: skip


def _profile(length: float, speed: float, accel: float) -> tuple[float, float, float, float]:
    """(peak speed, ramp time, ramp length, total time) of a trapezoid from rest to rest over ``length``."""
    if length <= 0:
        return 0.0, 0.0, 0.0, 0.0
    ramp = speed * speed / (2 * accel)
    if 2 * ramp >= length:
        peak = math.sqrt(accel * length)
        return peak, peak / accel, length / 2, 2 * peak / accel
    return speed, speed / accel, ramp, 2 * speed / accel + (length - 2 * ramp) / speed


def _time_at(s: float, length: float, speed: float, accel: float) -> float:
    """When the trapezoid over ``length`` passes ``s``."""
    peak, t_ramp, s_ramp, total = _profile(length, speed, accel)
    if s <= 0:
        return 0.0
    if s >= length:
        return total
    if s < s_ramp:
        return math.sqrt(2 * s / accel)
    if s <= length - s_ramp:
        return t_ramp + (s - s_ramp) / peak
    return total - math.sqrt(2 * (length - s) / accel)


def _counts(value: float) -> int:
    return R.to_counts(value)


def _floor2(value: float) -> float:
    return math.floor(value * 100 + 1e-9) / 100


def _ceil2(value: float) -> float:
    return math.ceil(value * 100 - 1e-9) / 100


class _NoMotion(ValueError):
    """The path moves no axis (by at least one position count)."""


class _Infeasible(Exception):
    """A plan's checks failed (internal: :func:`plan` turns it into a :class:`SoftLimitError`)."""

    def __init__(self, checks: _Checks) -> None:
        super().__init__("; ".join(checks.problems))
        self.checks = checks


class _Checks:
    """Collects the refusals of one plan and, per refusal, a first estimate of the largest speed that would pass."""

    def __init__(self, speed: float) -> None:
        self.speed = speed
        self.problems: list[str] = []
        self.feasible: list[float] = []
        self.no_speed_helps = False

    def add(self, text: str, feasible_speed: float | None = None) -> None:
        self.problems.append(text)
        if feasible_speed is None:
            self.no_speed_helps = True
        else:
            self.feasible.append(max(0.0, feasible_speed))

    @property
    def estimate(self) -> float | None:
        """The linear estimate of the largest feasible speed (exact for sync plans, a first guess for stream ones)."""
        return None if self.no_speed_helps or not self.feasible else _floor2(min(self.feasible))

    def raise_if_any(self) -> None:
        if self.problems:
            raise _Infeasible(self)

    def error(self, best: float | None) -> SoftLimitError:
        if best is None:
            tail = "no speed makes this path feasible"
        else:
            tail = f"the largest feasible speed is about {best:g} (asked {self.speed:g})"
        err = SoftLimitError(f"coordinated move refused, nothing sent: {'; '.join(self.problems)}; {tail}")
        err.reasons = list(self.problems)  # type: ignore[attr-defined]
        err.max_speed = best  # type: ignore[attr-defined]
        return err


def _check_positions(checks: _Checks, axes: tuple[str, ...], points: list[tuple[float, ...]],
                     ranges: Mapping[str, AxisRange]) -> None:  # fmt: skip
    """Every commanded waypoint (``points[1:]``; ``points[0]`` is where the axes already are, never sent) inside each
    axis's range."""
    for i, axis in enumerate(axes):
        rng = ranges[axis].position
        if rng is None or len(points) < 2:
            continue
        lo, hi = rng
        values = [p[i] for p in points[1:]]
        worst = min(values) if min(values) < lo - 1e-9 else max(values) if max(values) > hi + 1e-9 else None
        if worst is not None:
            k = values.index(worst) + 1
            checks.add(f"axis {axis}: waypoint {k} at {worst:.3f} is outside its range {lo:g}..{hi:g} (the soft limits "
                       "minus their margin, within a configured stroke)")  # fmt: skip


def _check_axis_rates(checks: _Checks, axes: tuple[str, ...], speeds: list[tuple[float, ...]],
                      accels: list[tuple[float, ...]], ranges: Mapping[str, AxisRange], accel_g: float,
                      *, frame_accel: bool) -> list[str]:  # fmt: skip
    """Per-axis speed (refusal), accel (refusal for sync; for stream the path accel is checked instead) and the
    documented minimum speed (one warning per axis)."""
    warnings = []
    for i, axis in enumerate(axes):
        rng = ranges[axis]
        moving = [s[i] for s in speeds if s[i] > 0]
        if not moving:
            continue
        top = max(moving)
        if rng.max_speed is not None and top > rng.max_speed + 1e-9:
            checks.add(f"axis {axis}: {top:g} {rng.units}/s above its maximum {rng.max_speed:g}",
                       checks.speed * rng.max_speed / top)  # fmt: skip
        cap = rng.max_accel_g
        if cap is not None:
            if frame_accel:
                if accel_g > cap + 1e-9:
                    checks.add(f"axis {axis}: path acceleration {accel_g:g} G above its maximum {cap:g} G (lower accel_g)")
            else:
                top_a = max(a[i] for a in accels)
                if top_a > cap + 1e-9:
                    checks.add(f"axis {axis}: acceleration {top_a:g} G above its maximum {cap:g} G (lower accel_g)")
        if rng.min_speed is not None and min(moving) < rng.min_speed - 1e-9:
            warnings.append(
                f"axis {axis}: speeds down to {min(moving):g} {rng.units}/s, below its documented minimum "
                f"{rng.min_speed:g} {rng.units}/s (allowed; it may run unsteadily there)"
            )
    return warnings


def _largest_speed(attempt: Callable[[float], MotionPlan], speed: float, estimate: float | None) -> float | None:
    """The largest speed (0.01 steps) at which ``attempt`` passes every check, by bisection over the planner itself:
    per-axis speeds and ``v^2 / r`` rise with the speed and step times fall, so feasibility is monotone. ``None`` when
    not even 0.01 passes."""

    def ok(v: float) -> bool:
        try:
            attempt(v)
        except _Infeasible:
            return False
        return True

    lo = None
    if estimate is not None and 0.01 <= estimate < speed and ok(estimate):
        lo = estimate
    elif ok(0.01):
        lo = 0.01
    if lo is None:
        return None
    hi = speed
    while hi - lo > 0.004:
        mid = (lo + hi) / 2
        if ok(mid):
            lo = mid
        else:
            hi = mid
    best = _floor2(lo)
    for _ in range(5):  # rounding: step down until it passes
        if best < 0.01:
            return None
        if ok(best):
            return best
        best = round(best - 0.01, 2)
    return None


def plan(
    path: Path,
    speed: float,
    accel_g: float = 0.1,
    *,
    chord_error: float = 0.05,
    mode: str = "auto",
    limits: Mapping[str, AxisRange | Mapping[str, Any]] | None = None,
    min_step_s: float | None = None,
    timing: LineTiming | None = None,
    band: float = 0.1,
    start: Point | None = None,
    blend: float | None = None,
    stream_accel_g: float | None = None,
) -> MotionPlan:
    """Plan ``path`` at path speed ``speed`` (units/s) with ramps of ``accel_g`` (G). Pure: no I/O.

    ``chord_error``: the largest distance between an arc and its chords (stream mode). ``mode``: ``"auto"``,
    ``"sync"`` (straight lines only) or ``"stream"``. ``limits``: ``{axis: AxisRange | ax.limits() dict}`` (axes
    missing there are not range-checked). ``min_step_s``: the shortest step the line can carry (default:
    ``timing.min_step_s``, :class:`LineTiming` defaults when no ``timing`` is given). ``band``: the positioning band of
    every frame. ``start``: overrides the path's start. ``blend`` (sync): start the next segment once every axis is
    within this distance of its target. ``stream_accel_g``: a fixed re-targeting acceleration instead of the automatic
    one. Raises :class:`~iai_actuator_driver.errors.SoftLimitError` (``.reasons``, ``.max_speed``: the largest speed that passes
    every check, found by re-running them) when the path is infeasible, ``ValueError`` for a path that moves no axis."""
    speed = _positive(speed, "speed")
    accel_g = _positive(accel_g, "accel_g")
    chord_error = _positive(chord_error, "chord_error")
    band = _positive(band, "band")
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
    if not isinstance(path, Path):
        raise TypeError(f"path must be a Path, got {path!r}")
    pieces = path.pieces(start)
    if not pieces:
        raise _NoMotion("the path does not move any axis")
    axes = tuple(pieces[0].start)
    straight = all(p.straight for p in pieces)
    if mode == "auto":
        mode = "sync" if straight else "stream"
    elif mode == "sync" and not straight:
        raise ValueError("mode='sync' runs straight lines only; use mode='stream' (or 'auto') for arcs and circles")
    if blend is not None:
        blend = _positive(blend, "blend")
        if mode != "sync":
            raise ValueError("blend applies to mode='sync' (a streamed path never stops between its steps)")
    if stream_accel_g is not None:
        stream_accel_g = _positive(stream_accel_g, "stream_accel_g")
    ranges = {a: AxisRange.of((limits or {}).get(a)) for a in axes}
    timing = timing or LineTiming()
    min_step = timing.min_step_s if min_step_s is None else max(0.0, _number(min_step_s, "min_step_s"))
    common = {"accel_g": accel_g, "chord_error": chord_error, "band": band, "ranges": ranges, "timing": timing,
              "min_step": min_step}  # fmt: skip

    def attempt(v: float) -> MotionPlan:
        if mode == "sync":
            return _plan_sync(pieces, axes, speed=v, blend=blend, **common)
        return _plan_stream(pieces, axes, speed=v, stream_accel_g=stream_accel_g, **common)

    try:
        result = attempt(speed)
    except _Infeasible as exc:
        checks = exc.checks
        best = None if checks.no_speed_helps else _largest_speed(attempt, speed, checks.estimate)
        raise checks.error(best) from None
    result.min_step_override = min_step_s is not None
    return result


def _plan_sync(pieces: list[_Piece], axes: tuple[str, ...], *, speed: float, accel_g: float, chord_error: float,
               band: float, ranges: dict[str, AxisRange], timing: LineTiming, min_step: float,
               blend: float | None) -> MotionPlan:  # fmt: skip
    accel = accel_g * G
    points = [tuple(pieces[0].start[a] for a in axes)] + [tuple(p.end[a] for a in axes) for p in pieces]
    times = [0.0]
    speeds: list[tuple[float, ...]] = []
    accels: list[tuple[float, ...]] = []
    warnings: list[str] = []
    rounded: set[str] = set()
    for p in pieces:
        d = [p.end[a] - p.start[a] for a in axes]
        length = math.hypot(*d)
        times.append(times[-1] + _profile(length, speed, accel)[3])
        sp, ac = [], []
        for i, axis in enumerate(axes):
            if _counts(p.end[axis]) == _counts(p.start[axis]):
                sp.append(0.0)
                ac.append(0.0)
                continue
            share = abs(d[i]) / length
            sp.append(max(0.01, round(speed * share, 2)))
            a_i = max(0.01, round(accel_g * share, 2))
            if abs(a_i - accel_g * share) > 0.1 * accel_g * share and axis not in rounded:
                rounded.add(axis)
                warnings.append(f"axis {axis}: acceleration {accel_g * share:.4f} G rounds to {a_i:g} G (0.01 G steps): "
                                "its ramps differ a little from the others, the path bends slightly there")  # fmt: skip
            ac.append(a_i)
        speeds.append(tuple(sp))
        accels.append(tuple(ac))
    checks = _Checks(speed)
    _check_positions(checks, axes, points, ranges)
    warnings += _check_axis_rates(checks, axes, speeds, accels, ranges, accel_g, frame_accel=False)
    checks.raise_if_any()
    skew = JITTER_S + max(0, timing.axes_on_port - 1) * timing.write_s
    return MotionPlan(
        "sync", axes, points, times, speeds, accels, speed, accel_g, chord_error, sum(p.length() for p in pieces), band,
        min_step, 0.0, skew, speed * skew, blend, warnings, timing, ranges,
    )  # fmt: skip


def _merge_steps(times: list[float], dist: list[float], counts: list[list[int]], inner: list[bool], min_step: float,
                 max_cut: float) -> tuple[list[int], float]:  # fmt: skip
    """The waypoints a streamed plan keeps (indices) and the largest corner cut. A waypoint whose counts equal the last
    kept one's on every axis sends no frame: it is dropped and its step merges into the next (the last waypoint too:
    the end then differs from the path's by less than one count). A step shorter than ``min_step`` merges with its
    neighbour across a waypoint inside a straight piece (``inner``: collinear, the path does not change; e.g. at the
    foot of a ramp) or, when the step is no longer than ``max_cut`` (the chord error), across any waypoint: the path
    then cuts that corner by less than the step's length."""
    inner = list(inner)
    keep = [0]
    for k in range(1, len(times)):
        if counts[k] != counts[keep[-1]]:
            keep.append(k)
    cut = 0.0
    if min_step > 0:
        i = 1
        while i < len(keep):
            a, b = keep[i - 1], keep[i]
            length = dist[b] - dist[a]
            if times[b] - times[a] < min_step - 1e-9:
                for j, drop in ((i, b), (i - 1, a)):
                    if not 0 < j < len(keep) - 1 or not (inner[drop] or length <= max_cut):
                        continue
                    if not inner[drop]:  # a corner cut: its neighbours are no longer collinear with theirs
                        cut = max(cut, length)
                        inner[keep[j - 1]] = inner[keep[j + 1]] = False
                    del keep[j]
                    i = max(1, j)
                    break
                else:
                    i += 1
                continue
            i += 1
    return keep, cut


def _plan_stream(pieces: list[_Piece], axes: tuple[str, ...], *, speed: float, accel_g: float, chord_error: float,
                 band: float, ranges: dict[str, AxisRange], timing: LineTiming, min_step: float,
                 stream_accel_g: float | None) -> MotionPlan:  # fmt: skip
    accel = accel_g * G
    max_piece = speed * max(LINE_STEP_S, 2 * min_step)
    points = [tuple(pieces[0].start[a] for a in axes)]
    radii: list[float] = []  # the radius each step runs on (inf: straight)
    inner = [False]  # per waypoint: inside a straight piece (collinear with both neighbours)
    est_chord = 0.0
    for p in pieces:
        if p.straight:
            n = max(1, math.ceil(p.length() / max_piece - 1e-9))
            radius = math.inf
        else:
            radius = min(p.r0, p.r1)
            if not radius >= MIN_RADIUS:  # Path.pieces refuses these already
                raise ValueError(f"an arc radius {radius:g} below one count ({MIN_RADIUS:g}) cannot be streamed")
            e = min(chord_error, radius)
            step = 2 * math.acos(1 - e / radius)
            n = max(1, math.ceil(abs(p.sweep) / step - 1e-9), math.ceil(abs(p.sweep) / MAX_CHORD_ANGLE - 1e-9))
            est_chord = max(est_chord, max(p.r0, p.r1) * (1 - math.cos(abs(p.sweep) / n / 2)))
        for j in range(1, n + 1):
            q = p.at(j / n)
            points.append(tuple(q[a] for a in axes))
            radii.append(radius)
            inner.append(p.straight and j < n)
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.dist(a, b))
    total = s[-1]
    times = [_time_at(x, total, speed, accel) for x in s]
    times[-1] = _profile(total, speed, accel)[3]
    counts = [[_counts(v) for v in p] for p in points]
    keep, cut = _merge_steps(times, s, counts, inner, min_step, chord_error)
    est_chord = max(est_chord, cut)
    if len(keep) < 2:
        raise _NoMotion("the path moves no axis by a whole position count (0.01)")
    radii = [min(radii[a:b]) for a, b in zip(keep, keep[1:])]
    points = [points[k] for k in keep]
    times = [times[k] for k in keep]
    counts = [counts[k] for k in keep]
    speeds: list[tuple[float, ...]] = []
    accels: list[tuple[float, ...]] = []
    capped: dict[str, float] = {}
    previous = [0.0] * len(axes)
    for k in range(len(points) - 1):
        dt = times[k + 1] - times[k]
        sp, ac = [], []
        for i, axis in enumerate(axes):
            delta = abs(counts[k + 1][i] - counts[k][i]) * R.POSITION_SCALE
            if delta == 0:
                sp.append(0.0)
                ac.append(0.0)
                previous[i] = 0.0
                continue
            v = max(0.01, round(delta / dt, 2))
            need = stream_accel_g if stream_accel_g is not None else max(4 * abs(v - previous[i]), 8 * v) / dt / G
            a_i = min(R.ACMD_MAX_RAW * R.ACCEL_SCALE, max(0.01, _ceil2(need)))  # ACMD tops out at 3.00 G
            cap = ranges[axis].max_accel_g
            if cap is not None and a_i > cap:
                capped[axis] = max(capped.get(axis, 0.0), need)
                a_i = max(0.01, _floor2(cap))
            sp.append(v)
            ac.append(a_i)
            previous[i] = v
        speeds.append(tuple(sp))
        accels.append(tuple(ac))
    checks = _Checks(speed)
    _check_positions(checks, axes, points, ranges)
    warnings = _check_axis_rates(checks, axes, speeds, accels, ranges, accel_g, frame_accel=True)
    caps = [ranges[a].max_accel_g for i, a in enumerate(axes) if any(sp[i] > 0 for sp in speeds)]
    caps = [c for c in caps if c is not None]
    if caps:
        cap_g = min(caps)
        worst = None
        for k, r in enumerate(radii):
            if math.isinf(r):
                continue
            v = math.dist(points[k], points[k + 1]) / (times[k + 1] - times[k])
            if v * v / r > cap_g * G + 1e-9 and (worst is None or r < worst[1]):
                worst = (v, r)
        if worst is not None:
            v, r = worst
            checks.add(f"centripetal acceleration {v * v / r / G:.3f} G on radius {r:g} at {v:.2f}/s is above the "
                       f"smallest axis acceleration {cap_g:g} G", math.sqrt(cap_g * G * r))  # fmt: skip
    shortest = min(b - a for a, b in zip(times, times[1:]))
    if min_step > 0 and shortest < min_step - 1e-9:
        checks.add(f"steps down to {shortest * 1000:.1f} ms, shorter than the {min_step * 1000:.1f} ms the line can "
                   "carry (lower the speed, raise chord_error, or a faster baud rate)",
                   speed * shortest / min_step)  # fmt: skip
    checks.raise_if_any()
    for axis, need in capped.items():
        warnings.append(f"axis {axis}: re-targeting wants up to {need:.2f} G, capped at its maximum "
                        f"{ranges[axis].max_accel_g:g} G: it lags behind the schedule there")  # fmt: skip
    skew = JITTER_S + max(0, timing.axes_on_port - 1) * timing.write_s
    return MotionPlan(
        "stream", axes, points, times, speeds, accels, speed, accel_g, chord_error, total, band, min_step, est_chord,
        skew, est_chord + speed * skew, None, warnings, timing, ranges,
    )  # fmt: skip


# =========================================================================== results


def _segment_distance(p: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
    ab = [y - x for x, y in zip(a, b)]
    l2 = sum(d * d for d in ab)
    if l2 == 0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, sum((pi - ai) * d for pi, ai, d in zip(p, a, ab)) / l2))
    return math.dist(p, [ai + t * d for ai, d in zip(a, ab)])


def path_deviation(points: Iterable[Sequence[float]], polyline: Sequence[Sequence[float]]) -> tuple[float, float]:
    """(max, mean) perpendicular distance of ``points`` from ``polyline``. (0, 0) for no points."""
    dists = []
    segments = list(zip(polyline, polyline[1:])) or [(polyline[0], polyline[0])]
    for p in points:
        dists.append(min(_segment_distance(p, a, b) for a, b in segments))
    if not dists:
        return 0.0, 0.0
    return max(dists), sum(dists) / len(dists)


@dataclass
class CoordinatedResult:
    """What a coordinated run did: the ``plan``, the status ``samples`` per axis (``(t, position)``, t from the
    planned start), the merged ``trace`` (``(t, p...)`` in plan axis order, only where every axis was read within one
    read time), the deviation of the trace from the planned polyline (an estimate from the status samples: blind
    between samples, coarser with fewer or slower reads; ``sample_spacing_s`` is the median time between two samples of
    one axis), the frames sent / skipped, late frames (stream: on the wire more than 10 % of their step after the
    planned time) and the worst lateness, the worst start skew between the axes of one step, the duration."""

    plan: MotionPlan
    samples: dict[str, list[tuple[float, float]]]
    trace: list[tuple[float, ...]]
    max_deviation: float
    mean_deviation: float
    frames: int
    skipped_frames: int
    late_frames: int
    worst_late_s: float
    worst_skew_s: float
    duration_s: float
    completed: bool
    error: str | None = None
    warnings: list[str] = field(default_factory=list)
    final: dict[str, float] = field(default_factory=dict)
    sample_spacing_s: float = 0.0

    def summary(self) -> str:
        state = "completed" if self.completed else f"STOPPED: {self.error}"
        return (f"{self.plan.mode} run {state} in {self.duration_s:.2f} s (planned {self.plan.duration:.2f} s): "
                f"deviation max {self.max_deviation:.3f} / mean {self.mean_deviation:.3f} (estimated from "
                f"{len(self.trace)} trace points); {self.frames} frames ({self.skipped_frames} skipped), "
                f"{self.late_frames} late (worst {self.worst_late_s * 1000:.1f} ms), worst skew "
                f"{self.worst_skew_s * 1000:.1f} ms; {sum(len(v) for v in self.samples.values())} samples, "
                f"~{self.sample_spacing_s * 1000:.0f} ms apart per axis")  # fmt: skip

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.plan.mode, "axes": list(self.plan.axes), "plan": self.plan.to_dict(),
            "samples": {a: [[round(t, 4), round(p, 3)] for t, p in v] for a, v in self.samples.items()},
            "trace": [[round(v, 4) for v in row] for row in self.trace],
            "max_deviation": round(self.max_deviation, 4), "mean_deviation": round(self.mean_deviation, 4),
            "frames": self.frames, "skipped_frames": self.skipped_frames, "late_frames": self.late_frames,
            "worst_late_s": round(self.worst_late_s, 5), "worst_skew_s": round(self.worst_skew_s, 5),
            "duration_s": round(self.duration_s, 4), "completed": self.completed, "error": self.error,
            "warnings": list(self.warnings), "final": {a: round(p, 3) for a, p in self.final.items()},
            "sample_spacing_s": round(self.sample_spacing_s, 5),
        }  # fmt: skip


class CoordinatedMotionError(EcDriverError):
    """A coordinated run failed or was stopped from outside. Every axis of the group was sent a decel stop first.
    ``result`` is the :class:`CoordinatedResult` so far, ``axis`` the axis that failed (``None``: not one axis),
    ``stop`` what the stop returned per axis. (Ctrl+C is not wrapped: ``KeyboardInterrupt`` is re-raised with the
    result in ``.coordinated_result``.)"""

    def __init__(self, message: str, result: CoordinatedResult, *, axis: str | None = None,
                 stop: Mapping[str, Any] | None = None) -> None:  # fmt: skip
        super().__init__(message)
        self.result = result
        self.axis = axis
        self.stop = dict(stop or {})


publish(globals(), "coord")
