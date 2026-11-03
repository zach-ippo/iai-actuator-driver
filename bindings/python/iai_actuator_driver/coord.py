"""Coordinated (interpolated) multi-axis motion, run by the PC: lines, arcs and circles over several axes.

IAI units have no interpolation of their own and no shared clock: every axis runs its own move. This module makes
several axes follow one path from the PC side::

    from iai_actuator_driver import Cell, Path, sim_ports

    with Cell.open(sim_ports(2)) as cell:
        xy = cell.coordinated(x="SIM1:1", y="SIM2:1")
        plan = xy.plan(Path.circle({"x": 75.0, "y": 75.0}, 10.0), speed=20, accel_g=0.3)   # dry run, nothing sent
        print(plan.summary())
        result = xy.run_plan(plan)                   # or xy.circle((75, 75), 10.0, speed=20)
        print(result.max_deviation)

Coordinates are ``{axis name: position}`` in each axis's own units (mm, or deg on a rotary axis); axes a segment does
not name hold their position. Two ways of running a path:

* ``mode="sync"`` (straight lines, polylines): each segment is one move per axis, started on every port at once (ports
  in parallel threads, the axes of a port back to back). Axis i gets speed ``v * |d_i| / L`` and acceleration =
  deceleration ``a * |d_i| / L``, so every axis runs the same trapezoid in time and the point stays on the straight
  line. The next segment starts when every axis is done (``blend``: when every axis is within that distance of its
  target).
* ``mode="stream"`` (arcs, circles, curves): the path is cut into chords - per arc the angle step is
  ``2 * acos(1 - e / r)`` for the chord error ``e`` - and every waypoint gets a time from a trapezoid speed profile along
  the path length. At the planned time ``t_k`` each axis is re-targeted to waypoint ``k + 1`` (one step ahead, so it
  never arrives and stops early) with the speed that gets it there on time and an acceleration high enough to change
  speed within a quarter of the step and to keep its braking distance within a sixteenth of it (capped by the axis).
  One thread of the core per port runs the schedule against one start time (the core's monotonic clock; Windows timer
  at 1 ms); a frame whose target does not change is skipped (a waypoint that changes no axis by a count is merged
  away); the last waypoint completes by the axis's normal completion rule. A frame's lateness is measured when its call
  begins, not when it reaches the wire (the core's bus stamps no per-request time); a frame later than one full step
  (``late_abort_s``) aborts. A lost echo is sent again at once (the frame is idempotent): it costs the echo wait (about
  three frame times, at least 50 ms), so with shorter steps the re-sent frame is late and the run aborts; both attempts
  lost are a link fault.
  Between frames each axis's status is read about every :data:`SAMPLE_GAP_S` while the slack allows, and at least
  every :data:`SAMPLE_BOUND_S` (longer on a slow line) even when that makes a frame late, so a fault on one axis stops
  the others within a few steps.
* ``mode="auto"``: sync for a path made only of lines, stream otherwise.

A path the axes are not at the start of gets a straight lead-in from where they are: one more segment of a sync path,
its own synchronized line before a streamed path (inside the stream schedule it would be one very short step).

The planner (:func:`plan`) is pure and runs every check before anything is sent: every commanded waypoint inside each
axis's range, every per-axis speed and acceleration within the axis's :meth:`~iai_actuator_driver.ec.Elecylinder.limits`, the
centripetal acceleration ``v^2 / r`` within the smallest axis acceleration, and the step time not shorter than what the
line can carry (:class:`LineTiming`: baud rate, frame lengths, the unit's response time, axes per port - at least what
was measured on the line; ``min_step_s`` overrides it). An infeasible path raises
:class:`~iai_actuator_driver.errors.SoftLimitError` naming the reason and the largest feasible speed (``.max_speed``, found by
re-running the planner's checks); nothing is sent. :meth:`CoordinatedGroup.run_plan` checks again against the axes as
they are now: the ranges read fresh, every moving axis's lowest and highest waypoint against its current fence as
:meth:`~iai_actuator_driver.ec.Elecylinder.move_to` checks a target, every axis of the plan ready (held ones too, none moving),
and a streamed plan's steps against the line timing the core measures inside that run.

During a run the status monitors of the run's axes are held (the run reads its own samples; a poll in flight would
delay a frame by a whole transaction); other axes' monitors on the same ports keep polling - their traffic is in the
measured line timing and in the frames' lateness. Every transaction waits at most about three times its measured time,
and the run never releases a STOP it did not send. Any error, a fault alarm (EMG, servo off, pause), a link fault, a
frame later than one step, an outside stop (``group.stop()``, ``cell.stop_all()``, the cell's stop on a link fault,
``ax.stop()`` / ``pause()`` from another thread, from the moment ``run_plan`` is called: during its checks, the lead-in
or between the lead-in and the path) or Ctrl+C stops **every axis of the group** (decel stop). Errors and outside stops
raise :class:`CoordinatedMotionError` (the trace so far in ``.result``); Ctrl+C re-raises ``KeyboardInterrupt`` with the
result in ``.coordinated_result``. Axes outside the group are not commanded, except that a link fault in a cell
(:data:`SILENT_READS` failed reads in a row, both attempts of a frame lost, the port failing) goes to the cell's fault
handling once the group's STOP frames are out, and follows its ``stop_on_fault`` policy (every other axis of the cell
is stopped too). One run at a time per axis. A speed below an axis's documented minimum is reported once per axis in
the plan's warnings, never logged per frame.

Evidence: a 0x9900 block re-sent mid-move re-targets the move and a new speed takes effect mid-move (VERIFIED on the EC
gripper, ``change_speed`` 2 -> 8 mm/s); unit replies take tens of ms (VERIFIED; up to ~285 ms at 9600 baud), hence the
measured line timing. With one real axis (the EC gripper) and a simulated second axis, a sync line, streamed circles
and the outside stops ran on hardware (VERIFIED: about 15 re-targets per second, no late frame; the measured shortest
step 82 ms at 38400 and 53 ms at 230400 baud, longer than the :class:`LineTiming` model). Two or more real axes
together - the start skew between real units, their combined path - are UNVERIFIED. The PC has no shared clock with the units: frame timing sets the smoothness.

The geometry (:class:`Path` and its segments), the planner (:func:`plan`), :class:`AxisRange`, :class:`LineTiming` and
the records (:class:`MotionPlan`, :class:`CoordinatedResult`, :class:`CoordinatedMotionError`) are the shared ones
(``_shared/coord.py``): pure Python, no I/O. The
:class:`CoordinatedGroup` is the core's: the core plans, checks, runs
and stops: :meth:`~CoordinatedGroup.plan` is ``iad_cell_coord_plan_json`` and a run is ``iad_cell_coord_run_json`` on
the group's core cell (the cell's own for :meth:`Cell.coordinated <iai_actuator_driver.cell.Cell.coordinated>`, one
made over the axes for a group built directly, ``iad_cell_open_axes``). This module converts the path to the core's
JSON request, the core's plan and result to the shared records, and its errors to the package's (a refusal's
``reasons`` and ``max_speed``; :class:`CoordinatedMotionError` with the result so far, the failing axis and the stop).

What differs from the runner of the 1.0.x fallback, from the shape of the C ABI:

* :meth:`~CoordinatedGroup.run_plan` runs a plan this module made: the core has no call that runs a plan object, so it
  plans the plan's request again from where the axes are and runs that (``result.plan`` is the plan that ran). Before
  that the axes must be at the plan's start (else refused, nothing sent). A plan made elsewhere (the module-level
  :func:`plan`) raises ``NotImplementedError``: plan the path with the group.
* :meth:`~CoordinatedGroup.timing` raises ``NotImplementedError``: the core measures the line itself for every plan and
  run; the timing a plan was made with is its ``timing``.
* The run reports when it ends: there are no live samples while it runs. An outside stop counts from the core's run
  call, its planning included (``run_plan`` reads the axes' positions for the start check just before).
"""

from __future__ import annotations

import ctypes
import dataclasses
import json
import logging
import math
import weakref
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from . import native as _n
from ._shared.coord import (  # noqa: F401 (the shared names: geometry, planner, records, the constants)
    G,
    JITTER_S,
    LATE_FRACTION,
    LINE_STEP_S,
    MAX_CHORD_ANGLE,
    MEASURE_SAMPLES,
    MEASURE_TOLERANCE,
    MIN_RADIUS,
    MODES,
    POLL_S,
    READ_FRAME_BYTES,
    SAMPLE_BOUND_S,
    SAMPLE_GAP_S,
    SILENT_READS,
    SLOW_BAUD,
    SLOW_REPLY_S,
    START_LEAD_S,
    WAIT_FACTOR,
    WAIT_MIN_S,
    WAKE_ROUNDS,
    WATCH_S,
    WRITE_FRAME_BYTES,
    Arc,
    AxisRange,
    Circle,
    CoordinatedMotionError,
    CoordinatedResult,
    Line,
    LineTiming,
    MotionPlan,
    Path,
    Point,
    Polyline,
    Segment,
    _opt,
    _others,
    _pair,
    _plane,
    _positive,
    path_deviation,
    plan,
)
from .errors import (  # noqa: F401 (re-exported)
    EcAlarmError,
    EcDriverError,
    EcNotReadyError,
    ModbusCRCError,
    ModbusFrameError,
    ModbusTimeout,
    MotionTimeoutError,
    SoftLimitError,
)
from .modbus import TransportError  # noqa: F401 (re-exported)
from .monitor import fast_done, suspend_bus, timer_resolution  # noqa: F401 (re-exported)
from .cell import _json_call
from .modbus import _check, _error

if TYPE_CHECKING:
    from .cell import Cell
    from .ec import Elecylinder, Status

__all__ = [
    "Line",
    "Arc",
    "Circle",
    "Polyline",
    "Path",
    "AxisRange",
    "LineTiming",
    "MotionPlan",
    "plan",
    "path_deviation",
    "CoordinatedGroup",
    "CoordinatedResult",
    "CoordinatedMotionError",
    "MODES",
]

log = logging.getLogger("iai_actuator_driver.coord")  # the public module name, on every backend

_RESULT_START = 256 * 1024  # first buffer of a run's result (samples and trace); a larger one is read from the cell
_INTERRUPTED = "iai_actuator_driver: every axis of the coordinated group was sent a decel stop"
_NO_TIMING = (
    "CoordinatedGroup.timing is not available on the core: the core measures the line itself for every plan and run. "
    "The timing a plan was made with is plan.timing (a LineTiming); LineTiming(baudrate, axes_on_port=...) gives the "
    "model's"
)
_FOREIGN_PLAN = (
    "on the core, run_plan runs a plan that CoordinatedGroup.plan made (the core plans its path again and runs it); "
    "this plan was made elsewhere (coord.plan): plan the path with group.plan(path, ...) and run that, or run it with "
    "group.run(path, ...); nothing sent"
)


# =========================================================================== the core's JSON


def _point_json(point: Mapping[str, float]) -> dict[str, float]:
    return {str(axis): float(value) for axis, value in point.items()}


def _segment_json(seg: Segment) -> dict[str, Any]:
    """One segment as the core's request names it (include/iai_actuator_driver.h, "cell: coordinated motion")."""
    if isinstance(seg, Line):
        return {"line": _point_json(seg.to)}
    if isinstance(seg, Polyline):
        return {"polyline": [_point_json(p) for p in seg.points]}
    if isinstance(seg, Arc):
        return {"arc": {"center": list(seg.center), "end": _point_json(seg.end), "ccw": seg.ccw,
                        "plane": list(seg.plane)}}  # fmt: skip
    return {"circle": {"center": list(seg.center), "radius": seg.radius, "start_deg": seg.start_deg,
                       "turns": seg.turns, "ccw": seg.ccw, "plane": list(seg.plane)}}  # fmt: skip


def _path_json(path: Path) -> dict[str, Any]:
    out: dict[str, Any] = {"segments": [_segment_json(seg) for seg in path.segments]}
    if path.start is not None:
        out["start"] = _point_json(path.start)
    return out


def _rows(values: Iterable[Sequence[Any]]) -> list[tuple[float, ...]]:
    return [tuple(float(v) for v in row) for row in values]


def _timing(data: Mapping[str, Any] | None) -> LineTiming | None:
    if not data:
        return None
    return LineTiming(baudrate=int(data["baudrate"]), axes_on_port=int(data["axes_on_port"]),
                      response_ms=float(data["response_ms"]), gap_s=float(data["gap_s"]),
                      turnaround_s=float(data["turnaround_s"]), reads_per_step=int(data["reads_per_step"]),
                      margin=float(data["margin"]), measured_write_s=_opt(data.get("measured_write_s")),
                      measured_read_s=_opt(data.get("measured_read_s")))  # fmt: skip


@dataclass
class _CorePlan(MotionPlan):
    """A :class:`MotionPlan` the core made, with the request it was made from (without the group's axes), which
    :meth:`CoordinatedGroup.run_plan` gives the core again."""

    request: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)


def _plan(data: Mapping[str, Any], request: Mapping[str, Any] | None = None) -> MotionPlan:
    """The shared record of the core's plan JSON (MotionPlan::to_json, the ``to_dict`` layout); with ``request``
    a plan :meth:`CoordinatedGroup.run_plan` can run (its lead-in is a plain record)."""
    lead = data.get("lead_in")
    fields = {
        "mode": str(data["mode"]), "axes": tuple(str(a) for a in data["axes"]), "points": _rows(data["points"]),
        "times": [float(t) for t in data["times"]], "speeds": _rows(data.get("speeds") or []),
        "accels": _rows(data.get("accels") or []), "path_speed": float(data["path_speed"]),
        "accel_g": float(data["accel_g"]), "chord_error": float(data.get("chord_error") or 0.0),
        "length": float(data.get("length") or 0.0), "band": float(data["band"]),
        "min_step_s": float(data.get("min_step_s") or 0.0),
        "est_chord_error": float(data.get("est_chord_error") or 0.0),
        "est_timing_error_s": float(data.get("est_timing_error_s") or 0.0),
        "est_deviation": float(data.get("est_deviation") or 0.0), "blend": _opt(data.get("blend")),
        "warnings": [str(w) for w in data.get("warnings") or []], "timing": _timing(data.get("timing")),
        "ranges": {str(a): AxisRange.of(r) for a, r in (data.get("ranges") or {}).items()},
        "lead_in": _plan(lead) if lead else None, "min_step_override": bool(data.get("min_step_override")),
    }  # fmt: skip
    return MotionPlan(**fields) if request is None else _CorePlan(**fields, request=dict(request))


def _result(data: Mapping[str, Any], request: Mapping[str, Any]) -> CoordinatedResult:
    """The shared record of the core's result JSON (CoordinatedResult::to_json, the ``to_dict`` layout)."""
    return CoordinatedResult(
        plan=_plan(data["plan"], request),
        samples={str(a): [(float(t), float(p)) for t, p in rows] for a, rows in (data.get("samples") or {}).items()},
        trace=_rows(data.get("trace") or []), max_deviation=float(data.get("max_deviation") or 0.0),
        mean_deviation=float(data.get("mean_deviation") or 0.0), frames=int(data.get("frames") or 0),
        skipped_frames=int(data.get("skipped_frames") or 0), late_frames=int(data.get("late_frames") or 0),
        worst_late_s=float(data.get("worst_late_s") or 0.0), worst_skew_s=float(data.get("worst_skew_s") or 0.0),
        duration_s=float(data.get("duration_s") or 0.0), completed=bool(data.get("completed")),
        error=None if data.get("error") is None else str(data["error"]),
        warnings=[str(w) for w in data.get("warnings") or []],
        final={str(a): float(p) for a, p in (data.get("final") or {}).items()},
        sample_spacing_s=float(data.get("sample_spacing_s") or 0.0),
    )  # fmt: skip


def _refusal(exc: BaseException) -> BaseException:
    """A refused path (``SoftLimitError``) with its ``reasons`` and ``max_speed`` (the largest feasible
    speed, ``None`` when no speed helps) from the core's error record."""
    if isinstance(exc, SoftLimitError):
        details = getattr(exc, "iad_details", None) or {}
        exc.reasons = [str(r) for r in details.get("reasons") or [str(exc)]]  # type: ignore[attr-defined]
        exc.max_speed = _opt(details.get("max_speed"))  # type: ignore[attr-defined]
    return exc


# =========================================================================== the group


class CoordinatedGroup:
    """Several axes (``{name: Elecylinder}``) moved along one path, planned, run and stopped by the core. Build it
    directly or with :meth:`~iai_actuator_driver.cell.Cell.coordinated`. Path coordinates use these names.

    The axes may sit on one port or several (ports run in parallel, the axes of a port take turns). :meth:`plan` is the
    dry run; :meth:`run` / :meth:`run_plan` move; :meth:`line_to` and :meth:`circle` are shortcuts from the current
    position; :meth:`stop` decel-stops every axis of the group (and ends a run in another thread). The rules of a
    run, and what differs from the runner of the 1.0.x fallback, are in this module's documentation."""

    def __init__(self, axes: Mapping[str, Elecylinder], *, cell: Cell | None = None,
                 cell_names: Mapping[str, str] | None = None, band_mm: float = 0.1) -> None:  # fmt: skip
        if not axes:
            raise ValueError("a coordinated group needs at least one axis")
        if len({id(ec) for ec in axes.values()}) != len(axes):
            raise ValueError("the same axis appears twice in the group")
        self.axes: dict[str, Elecylinder] = dict(axes)
        self.cell = cell
        self.cell_names: dict[str, str] = dict(cell_names or {})
        self.band = _positive(band_mm, "band_mm")
        self.last_result: CoordinatedResult | None = None
        self._lib = _n.load()
        self._own: int | None = None  # a core cell of the group's own (a group built directly)
        if cell is not None and all(n in self.cell_names for n in self.axes):
            self._core_names = {n: self.cell_names[n] for n in self.axes}
        else:  # the group's own core cell over its axes, named by the path names; released (never closed) with it
            if any("," in n for n in self.axes):
                raise ValueError(f"an axis name of a coordinated group has a comma: {list(self.axes)}")
            handles = (ctypes.c_void_p * len(self.axes))(*(ec._h(False) for ec in self.axes.values()))
            options = _n.CellOptions()
            _check(self._lib, "iad_cell_options_init", ctypes.byref(options))
            options.identify = 0  # max_axes stays 0: no limit
            out = ctypes.c_void_p()
            _check(self._lib, "iad_cell_open_axes", handles, len(self.axes), _n._utf8(",".join(self.axes)),
                   ctypes.byref(options), ctypes.byref(out))  # fmt: skip
            self._own = int(out.value or 0)
            weakref.finalize(self, _n._release, self._lib, "iad_cell_release", self._own)
            self._core_names = {n: n for n in self.axes}

    def __repr__(self) -> str:
        return f"CoordinatedGroup({ {n: self.cell_names.get(n, repr(ec)) for n, ec in self.axes.items()} })"

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self.axes)

    def _h(self) -> int:
        """The core cell the group runs on (the cell's, refused once it is closed, or the group's own)."""
        if self._own is not None:
            return self._own
        assert self.cell is not None
        return int(self.cell._h())

    def _check_names(self, names: Iterable[str]) -> None:
        unknown = [n for n in names if n not in self.axes]
        if unknown:
            raise ValueError(f"axis/axes {unknown} are not in this group (axes: {', '.join(self.axes)})")

    # -- reading -------------------------------------------------------------------------------------

    def ranges(self, *, refresh: bool = True) -> dict[str, AxisRange]:
        """Every axis's :class:`AxisRange` from ``ax.limits()``, read now (``refresh`` is kept for compatibility: the
        ranges are never cached). Raises when an axis's soft limits cannot be read and no stroke is configured (as a
        move would)."""
        out = {}
        for name in self.names:
            ec = self.axes[name]
            if ec.soft_limits_mm is None:
                ec.read_soft_limits()  # read again until a read succeeds, as the core's motion checks do
            if ec.effective_stroke is None:
                raise EcNotReadyError(f"axis {name}: stroke unknown: its soft limits are not available "
                                      f"({ec.soft_limits_note}); configure ax.stroke = (min, max)")  # fmt: skip
            out[name] = AxisRange.of(ec.limits())
        return out

    def timing(self, names: Iterable[str] | None = None, *, measure: bool = False) -> LineTiming:
        """Not on the core: ``NotImplementedError`` (the core measures the line for every plan and run itself; a plan's
        ``timing`` is the one it was made with)."""
        raise NotImplementedError(_NO_TIMING)

    def positions(self, names: Iterable[str] | None = None) -> dict[str, float]:
        """The current positions (one status read per axis)."""
        return {n: self.axes[n].status().position_mm for n in (self.names if names is None else names)}

    # -- planning ------------------------------------------------------------------------------------

    def _request(self, path: Path, speed: float, accel_g: float = 0.1, *, chord_error: float = 0.05,
                 mode: str = "auto", min_step_s: float | None = None, band: float | None = None,
                 blend: float | None = None, stream_accel_g: float | None = None) -> dict[str, Any]:  # fmt: skip
        """The core's request for ``path`` without the group's axes (the core checks every value)."""
        if not isinstance(path, Path):
            raise TypeError(f"path must be a Path, got {path!r}")
        self._check_names(path.axes)
        request: dict[str, Any] = {"path": _path_json(path), "speed": speed, "accel_g": accel_g,
                                   "chord_error": chord_error, "mode": mode,
                                   "band": self.band if band is None else band}  # fmt: skip
        optional = {"min_step_s": min_step_s, "blend": blend, "stream_accel_g": stream_accel_g}
        request.update({k: v for k, v in optional.items() if v is not None})
        return request

    def _text(self, request: Mapping[str, Any]) -> bytes:
        """The request with the group's axes, as the core reads it. A value that is no finite number is refused here
        (JSON has no NaN or infinity), as the core refuses it."""
        for key in ("speed", "accel_g", "chord_error", "band", "min_step_s", "blend", "stream_accel_g", "late_abort_s",
                    "timeout_s"):  # fmt: skip
            value = request.get(key)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{key} must be a finite number, got {value!r}; nothing sent")
        return _n._utf8(json.dumps({**request, "axes": self._core_names}))

    def plan(self, path: Path, speed: float, accel_g: float = 0.1, *, chord_error: float = 0.05, mode: str = "auto",
             min_step_s: float | None = None, band: float | None = None, blend: float | None = None,
             stream_accel_g: float | None = None) -> MotionPlan:  # fmt: skip
        """Dry run by the core (``iad_cell_coord_plan_json``): plan ``path`` from the axes' current positions against
        their own ranges and the line's timing, both read now. Nothing is sent (reads only). A path with a start the
        axes are not at gets a straight lead-in, with a warning. An infeasible path raises ``SoftLimitError``
        (``reasons``, ``max_speed``); one that moves no axis from where they are, ``ValueError``."""
        request = self._request(path, speed, accel_g, chord_error=chord_error, mode=mode, min_step_s=min_step_s,
                                band=band, blend=blend, stream_accel_g=stream_accel_g)  # fmt: skip
        try:
            data = _json_call(self._lib, "iad_cell_coord_plan_json", self._h(), self._text(request))
        except SoftLimitError as exc:
            raise _refusal(exc) from None
        return _plan(data, request)

    # -- running -------------------------------------------------------------------------------------

    def run(self, path: Path, speed: float, accel_g: float = 0.1, *, late_abort_s: float | None = None,
            timeout: float | None = None, **plan_kwargs: Any) -> CoordinatedResult:  # fmt: skip
        """Plan ``path`` and run it, in the core (``iad_cell_coord_run_json``). A path that moves no axis from where
        they are (e.g. ``line_to`` the current position) returns a completed result at once; nothing is sent."""
        return self._run(self._request(path, speed, accel_g, **plan_kwargs), late_abort_s, timeout)

    def run_plan(self, plan: MotionPlan, *, late_abort_s: float | None = None,
                 timeout: float | None = None) -> CoordinatedResult:  # fmt: skip
        """Run a plan this group (or another group on the core with these names) made. The core plans the same request
        again from where the axes are, checks that they are at the plan's start (its lead-in's, if any; else
        ``EcNotReadyError`` and nothing is sent), checks it against the axes as they are now and runs it (its lead-in
        first): every refusal, stop and error as :meth:`run`. A stop, pause or servo OFF sent to an axis of the plan
        after this call began - during the start check included - ends it. ``result.plan`` is the plan that ran.
        ``NotImplementedError`` for a plan made elsewhere (the module-level :func:`plan`)."""
        if not isinstance(plan, _CorePlan):
            raise NotImplementedError(_FOREIGN_PLAN)
        self._check_names(plan.axes)
        first = plan.lead_in if plan.lead_in is not None else plan
        # checked by the core, inside its run's stop window (a read here first would miss a stop sent meanwhile)
        start = {axis: first.points[0][i] for i, axis in enumerate(first.axes)}
        return self._run(plan.request, late_abort_s, timeout, expect_start=start)

    def _run(self, request: Mapping[str, Any], late_abort_s: float | None, timeout: float | None, *,
             expect_start: Mapping[str, float] | None = None) -> CoordinatedResult:  # fmt: skip
        """One core run of ``request``: blocks until it ends. On the main thread Ctrl+C sends every axis of the group a
        decel stop (the run notes it and ends) and re-raises ``KeyboardInterrupt`` with the result in
        ``.coordinated_result``."""
        full = dict(request)
        if late_abort_s is not None:
            full["late_abort_s"] = late_abort_s
        if timeout is not None:
            full["timeout_s"] = timeout
        if expect_start is not None:
            full["expect_start"] = dict(expect_start)
        text = self._text(full)
        handle = self._h()
        if self.cell is not None:
            self.cell._motion_commanded = True
        lib = self._lib
        buffer = ctypes.create_string_buffer(_RESULT_START)
        required = ctypes.c_size_t(0)
        outcome: list[tuple[int, dict[str, Any] | None]] = []
        names = list(self._core_names.values())

        def call(lib_: Any, name: str, *args: Any) -> tuple[int, dict[str, Any] | None]:
            outcome.append(_n._invoke(lib_, name, *args))
            return outcome[-1]

        def stop_group() -> Any:  # Ctrl+C: an outside stop of the group's axes ends the run (iad_cell_cancel does not)
            results, count = (_n.CellResult * len(names))(), ctypes.c_size_t(0)
            return _n._invoke(lib, "iad_cell_stop_all", handle, _n._utf8(",".join(names)), 5.0, 0, results,
                              len(names), ctypes.byref(count))  # fmt: skip

        try:
            code, details = _n._invoke_interruptible(
                lib, "iad_cell_coord_run_json", (handle, text, buffer, _RESULT_START, ctypes.byref(required)),
                stop_group, call=call, cancelled=_INTERRUPTED,
            )  # fmt: skip
        except KeyboardInterrupt as interrupt:
            interrupt.coordinated_result = self._interrupted(outcome, buffer, request)  # type: ignore[attr-defined]
            raise
        return self._outcome(code, details or {}, buffer, request)

    def _outcome(self, code: int, details: Mapping[str, Any], buffer: Any,
                 request: Mapping[str, Any]) -> CoordinatedResult:  # fmt: skip
        """The result of a run call, or its error: a stopped run as :class:`CoordinatedMotionError` (``result``,
        ``axis``, ``stop``, the cause behind it), a refusal as the package's error."""
        if code == _n.IAD_ERR_BUFFER_TOO_SMALL:  # the run is kept on the cell: read it without running again
            data = _json_call(self._lib, "iad_cell_last_result_json", self._h())
        elif code != _n.IAD_OK:
            if details.get("kind") == "CoordinatedMotionError":
                raise self._stopped(details, request)
            raise _refusal(_error(code, dict(details)))
        else:
            data = json.loads(buffer.value.decode("utf-8", errors="replace"))
        result = _result(data, request)
        self.last_result = result
        return result

    def _stopped(self, details: Mapping[str, Any], request: Mapping[str, Any]) -> CoordinatedMotionError:
        """The core's ``CoordinatedMotionError`` record as the shared exception. ``stop`` is ``{name: None}`` for each
        axis the group's stop reached and ``{name: error}`` for each it did not."""
        result = _result(details["result"], request)  # the core sends the result so far with every such error
        self.last_result = result
        path_name = {core: name for name, core in self._core_names.items()}
        stop = details.get("stop") or {}
        report: dict[str, Any] = {path_name.get(str(n), str(n)): None for n in stop.get("stopped") or []}
        for n, message in (stop.get("errors") or {}).items():
            report[path_name.get(str(n), str(n))] = EcDriverError(str(message))
        axis = details.get("axis")
        error = CoordinatedMotionError(str(details.get("message") or "coordinated move stopped"), result,
                                       axis=None if axis is None else str(axis), stop=report)  # fmt: skip
        for note in details.get("notes") or ():
            error.add_note(str(note))
        error.iad_code, error.iad_details = _n.IAD_ERR_DRIVER, dict(details)  # type: ignore[attr-defined]
        cause = details.get("cause")
        if isinstance(cause, Mapping):
            error.__cause__ = _error(int(cause.get("code") or _n.IAD_ERR_DRIVER), dict(cause))
        return error

    def _interrupted(self, outcome: list[tuple[int, dict[str, Any] | None]], buffer: Any,
                     request: Mapping[str, Any]) -> CoordinatedResult | None:  # fmt: skip
        """The result of a run ended by Ctrl+C (the call's own outcome; ``None`` when it had not ended)."""
        if not outcome:
            return None
        code, details = outcome[-1]
        try:
            result = self._outcome(code, details or {}, buffer, request)
        except CoordinatedMotionError as stopped:
            result = stopped.result
        except Exception:  # noqa: BLE001 - a refusal: nothing ran
            return None
        if not result.completed:
            result = dataclasses.replace(result, error="interrupted (KeyboardInterrupt)")
        self.last_result = result
        return result

    def line_to(self, point: Point, speed: float, accel_g: float = 0.1, **kwargs: Any) -> CoordinatedResult:
        """A synchronized straight move from the current position to ``point`` (axes not named hold). At ``point``
        already: a completed result, nothing sent."""
        return self.run(Path(None, [Line(point)]), speed, accel_g, **kwargs)

    def circle(self, center: Point | Sequence[float], radius: float, speed: float, accel_g: float = 0.1, *,
               plane: Sequence[str] | None = None, start_deg: float | None = None, turns: float = 1.0,
               ccw: bool = True, **kwargs: Any) -> CoordinatedResult:  # fmt: skip
        """A streamed circle in ``plane`` (default: the group's first two axes). ``start_deg`` ``None``: start where the
        axes are when they are on the circle (within 0.5 % of the radius, at least 0.01; the path then starts exactly
        there, no lead-in), else at 0 deg after a straight lead-in."""
        if plane is None:
            if len(self.axes) < 2:
                raise ValueError("a circle needs two axes")
            plane = self.names[:2]
        p = _plane(plane)
        self._check_names(p)
        if start_deg is not None:
            return self.run(Path.circle(center, radius, plane=p, start_deg=start_deg, turns=turns, ccw=ccw), speed,
                            accel_g, **kwargs)  # fmt: skip
        cx, cy = _pair(center, p, "circle center")
        held = _others(center, p)
        self._check_names(held)
        pos = self.positions([*p, *held])
        dx, dy = pos[p[0]] - cx, pos[p[1]] - cy
        on_circle = abs(math.hypot(dx, dy) - radius) <= max(0.01, 0.005 * radius)
        band = kwargs.get("band") or self.band
        if on_circle and all(abs(pos[a] - v) <= max(band, 0.01) for a, v in held.items()):
            seg = Circle(center, radius, math.degrees(math.atan2(dy, dx)), turns, ccw, plane=p)
            path = Path(pos, [seg])  # from where the axes are (held ones too): no lead-in for a radius a little off
        else:
            path = Path.circle(center, radius, plane=p, start_deg=0.0, turns=turns, ccw=ccw)
        return self.run(path, speed, accel_g, **kwargs)

    # -- stopping ------------------------------------------------------------------------------------

    def stop(self, *, timeout: float = 5.0) -> dict[str, Any]:
        """Decel stop on every axis of the group, as fast as possible: the core's ``stop_all`` on the group's axes (the
        STOP frame to every axis first, ports in parallel, then each axis holds it until it stands still). A run of
        this group in another thread sends nothing more and raises :class:`CoordinatedMotionError`. Returns
        ``{name: status or exception}``; never raises for one axis."""
        if self._own is None:
            assert self.cell is not None
            by_cell = self.cell.stop_all([self.cell_names[n] for n in self.axes], timeout=timeout,
                                         return_exceptions=True)  # fmt: skip
            return {n: by_cell.get(self.cell_names[n]) for n in self.axes}
        names = self.names
        results, count = (_n.CellResult * len(names))(), ctypes.c_size_t(0)
        code, details = _n._invoke(self._lib, "iad_cell_stop_all", self._own, _n._utf8(",".join(names)), float(timeout),
                                   0, results, len(names), ctypes.byref(count))  # fmt: skip
        if code not in (_n.IAD_OK, _n.IAD_ERR_CELL):
            error = _error(code, dict(details or {}))
            return {n: error for n in names}
        out: dict[str, Status | BaseException | None] = {}
        for r in results[: min(count.value, len(names))]:
            name = _n._text(r.axis)
            if name not in self.axes and 0 <= r.index < len(names):
                name = names[r.index]
            if r.code == _n.IAD_OK:
                out[name] = self.axes[name]._status(r.status)
            else:
                out[name] = _error(int(r.code), {"message": _n._text(r.message)})
        return {n: out.get(n) for n in names}
