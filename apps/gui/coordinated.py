"""The Coordinated (experimental) tab behind the web GUI: plan, run, stop and follow a two-axis path.

The motion is the driver's: the core's own coordinated motion on the workbench's
:class:`~iai_actuator_driver.cell.Cell`, through :class:`corecoord.CoreGroup` (``iad_cell_coord_plan_json`` /
``iad_cell_coord_run_json``: no live trace, the run plans again when it starts, as that module says). The GUI adds
no motion logic of its own.

* Plan (dry run): reads the positions, the limits and the line timing of both axes and plans the shape. A refused plan
  (``SoftLimitError`` with the reasons and the largest feasible speed, or a ``ValueError``) is an answer, not an error:
  the page shows the reasons and a preview of the asked shape. Planning holds both axes' locks.
* Run: only a stored plan that passed, by its id, once. A run thread holds both axes' locks for the whole run (the
  workbench poll then leaves the two axes alone) and calls ``group.run_plan``; the driver's own checks run again
  before anything is sent. STOP ALL, an axis STOP or pause, a fault or a late frame end the run through the driver.
* Status: the state machine (idle, planned, starting, running, completed, stopped, refused, failed), the live trace
  and, at the end, the result. A sampler thread reads ``group._active_run.samples`` every 50 ms (read-only: it sends
  nothing and changes nothing in the driver); after the run, ``result.trace`` stands in when no live sample was seen.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path as _FilePath
from typing import Any

ROOT = _FilePath(__file__).resolve().parents[2]  # the repository root of a source checkout
_DRIVER = ROOT / "bindings" / "python"  # its Python package, used before an installed one
if (_DRIVER / "iai_actuator_driver").is_dir() and str(_DRIVER) not in sys.path:
    sys.path.insert(0, str(_DRIVER))

from iai_actuator_driver import errors as E  # noqa: E402 (needs the sys.path entry above)
from iai_actuator_driver.coord import Circle, CoordinatedMotionError, Line, Path  # noqa: E402  (the path geometry)

try:
    from .backend import BackendError
    from .corecoord import CoreGroup
except ImportError:  # run as a script from apps/gui/
    from backend import BackendError  # type: ignore[no-redef]
    from corecoord import CoreGroup  # type: ignore[no-redef]

__all__ = ["CoordinatedRunner", "SHAPES"]

SHAPES = ("line", "circle", "rectangle", "polygon")
LOCK_WAIT_S = 2.0  # planning and the run wait at most this long for an axis lock
RUN_WAIT_S = 10.0  # the run request waits at most this long for the driver's checks
SAMPLE_S = 0.05  # the live trace is read this often
STOP_JOIN_S = 5.0  # a stop before a disconnect / close waits this long for the run to end
MAX_TRACE = 20_000  # trace points kept in full; beyond, every second one
MAX_PLAN_STEPS = 50_000  # most waypoints one planned arc path may need (the stream planner makes one per chord step)
RANGE_SLACK = 0.01  # a path this far outside an axis range is left to the driver's own check (one position count)
ARC_PREVIEW_POINTS = 48  # preview points per full turn of an arc or circle
BUSY_PLAN = "planning a coordinated move"
BUSY_RUN = "coordinated run"
RUN_ACTIVE = "A coordinated run is in progress: wait for it or press STOP ALL."
OUT_OF_DATE = "This plan is out of date: press Plan again."
STOPPED_BEFORE_START = "STOP was pressed before the run started: nothing was sent. Press Run again if you still want it."
RUN_STATES = ("starting", "running")
END_STATES = ("completed", "stopped", "refused", "failed")
ALIASES = ("x", "y")


def _describe(exc: BaseException) -> str:
    text = str(exc) or type(exc).__name__
    return f"{type(exc).__name__}: {text}"


def _bus_error(exc: BaseException) -> BackendError:
    if isinstance(exc, BackendError):
        return exc
    if isinstance(exc, (E.ModbusError, OSError)):
        return BackendError(_describe(exc), 502)
    return BackendError(_describe(exc), 409)


def _r(value: float | None, digits: int = 4) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(float(value), digits)


# ====================================================================== input checks (before any bus access)


def _number(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BackendError(f"{what} must be a finite number.", 400)
    return float(value)


def _pair(value: Any, what: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise BackendError(f"{what} must be [x, y].", 400)
    return _number(value[0], f"{what} x"), _number(value[1], f"{what} y")


def _positive(value: Any, what: str) -> float:
    number = _number(value, what)
    if number <= 0:
        raise BackendError(f"{what} must be above 0.", 400)
    return number


def _optional(shape: dict[str, Any], key: str, default: float | None) -> float | None:
    value = shape.get(key)
    return default if value is None else _number(value, key)


def parse_shape(shape: Any) -> dict[str, Any]:
    """The shape of a plan request, checked: numbers finite, sizes positive, counts in range."""
    if not isinstance(shape, dict):
        raise BackendError("shape must be an object with a kind (line, circle, rectangle or polygon).", 400)
    kind = shape.get("kind")
    if kind not in SHAPES:
        raise BackendError("shape kind must be line, circle, rectangle or polygon.", 400)
    if kind == "line":
        return {"kind": kind, "to": _pair(shape.get("to"), "to")}
    if kind == "circle":
        turns = _optional(shape, "turns", 1.0)
        assert turns is not None
        if not 0.05 <= turns <= 20:
            raise BackendError("turns must be 0.05..20.", 400)
        ccw = shape.get("ccw", True)
        if ccw is None:
            ccw = True
        if not isinstance(ccw, bool):
            raise BackendError("ccw must be true or false.", 400)
        return {"kind": kind, "center": _pair(shape.get("center"), "center"), "radius": _positive(shape.get("radius"),
                "radius"), "turns": turns, "ccw": ccw, "start_deg": _optional(shape, "start_deg", None)}  # fmt: skip
    if kind == "rectangle":
        width, height = _number(shape.get("width"), "width"), _number(shape.get("height"), "height")
        if width == 0 or height == 0:
            raise BackendError("width and height must not be 0.", 400)
        return {"kind": kind, "corner": _pair(shape.get("corner"), "corner"), "width": width, "height": height}
    sides = shape.get("sides")
    if isinstance(sides, bool) or not isinstance(sides, (int, float)) or sides != int(sides) or not 3 <= sides <= 64:
        raise BackendError("sides must be a whole number 3..64.", 400)
    return {"kind": kind, "center": _pair(shape.get("center"), "center"),
            "radius": _positive(shape.get("radius"), "radius"), "sides": int(sides),
            "start_deg": _optional(shape, "start_deg", 0.0)}  # fmt: skip


def parse_plan(body: dict[str, Any]) -> dict[str, Any]:
    axes = body.get("axes")
    if not isinstance(axes, dict) or not all(isinstance(axes.get(a), str) and axes.get(a) for a in ALIASES):
        raise BackendError('axes must be {"x": axis, "y": axis} (e.g. {"x": "COM3:1", "y": "COM4:1"}).', 400)
    if axes["x"] == axes["y"]:
        raise BackendError("X and Y must be two different axes.", 400)
    chord = _number(body.get("chord_error", 0.05), "chord_error")
    if not 0.001 <= chord <= 10:
        raise BackendError("chord_error must be 0.001..10.", 400)
    return {"axes": {"x": axes["x"], "y": axes["y"]}, "shape": parse_shape(body.get("shape")),
            "speed": _positive(body.get("speed"), "speed"), "accel_g": _positive(body.get("accel_g"), "accel_g"),
            "chord_error": chord}  # fmt: skip


def estimate_steps(shape: dict[str, Any], chord_error: float) -> int:
    """The waypoints the stream planner makes for a circle, in closed form (one per chord step of ``2 acos(1 - e / r)``
    radians; the driver's own count is never smaller). 0 for a shape that is planned in straight pieces."""
    if shape["kind"] != "circle":
        return 0
    radius = shape["radius"]
    step = 2 * math.acos(1 - min(chord_error, radius) / radius)
    return math.ceil(abs(shape["turns"]) * 2 * math.pi / step) if step > 0 else 0


def path_extent(path: Path, current: tuple[float, float]) -> tuple[list[float], list[float]]:
    """The box ``([x_min, x_max], [y_min, y_max])`` a path reaches, from the current position: the ends of every piece and,
    on an arc, its extreme points. Cheap (no planning), so it can refuse a path before the planner runs."""
    here = {"x": current[0], "y": current[1]}
    origin = path.start if path.start is not None else here
    xs, ys = [current[0], origin["x"]], [current[1], origin["y"]]
    try:
        pieces = path.pieces(origin)
    except (ValueError, TypeError):
        pieces = []
    quarter = math.pi / 2
    for piece in pieces:
        for end in (piece.start, piece.end):
            xs.append(end["x"])
            ys.append(end["y"])
        if piece.straight:
            continue
        radius = max(piece.r0, piece.r1)
        low, high = sorted((piece.a0, piece.a0 + piece.sweep))
        for k in range(math.ceil(low / quarter - 1e-9), math.floor(high / quarter + 1e-9) + 1):
            xs.append(piece.center[0] + radius * math.cos(k * quarter))
            ys.append(piece.center[1] + radius * math.sin(k * quarter))
    return [min(xs), max(xs)], [min(ys), max(ys)]


def pre_plan_reasons(shape: dict[str, Any], chord_error: float, path: Path, current: tuple[float, float],
                     slots: list[Any]) -> list[str]:
    """Why a path is refused before the planner runs: it leaves an axis's position range (the driver would refuse it
    after planning every waypoint), or it needs more than :data:`MAX_PLAN_STEPS` waypoints. Empty: plan it."""
    reasons: list[str] = []
    box = path_extent(path, current)
    for slot, (low, high) in zip(slots, box):
        bounds = _bounds(slot)
        if bounds is None:
            continue
        unit = (slot.limits or {}).get("units", "mm")
        if low < bounds[0] - RANGE_SLACK:
            reasons.append(f"{slot.name}: the path goes down to {low:.2f} {unit}, below the lowest position "
                           f"{bounds[0]:.2f} {unit} (outside its range {bounds[0]:g} to {bounds[1]:g} {unit})")
        if high > bounds[1] + RANGE_SLACK:
            reasons.append(f"{slot.name}: the path goes up to {high:.2f} {unit}, above the highest position "
                           f"{bounds[1]:.2f} {unit} (outside its range {bounds[0]:g} to {bounds[1]:g} {unit})")
    steps = estimate_steps(shape, chord_error)
    if steps > MAX_PLAN_STEPS:
        reasons.append(f"the circle needs about {steps:,} waypoints at chord error {chord_error:g} (at most "
                       f"{MAX_PLAN_STEPS:,}): use a larger chord error, fewer turns or a smaller radius")
    return reasons


def build_path(shape: dict[str, Any], current: tuple[float, float]) -> Path:
    """The iai_actuator_driver Path of a checked shape. A circle without ``start_deg`` starts where the axes are when they lie on
    it (within 0.5 % of the radius, at least 0.01), as ``CoordinatedGroup.circle`` does; else at 0 deg after the
    driver's lead-in."""
    kind = shape["kind"]
    if kind == "line":
        tx, ty = shape["to"]
        return Path(None, [Line({"x": tx, "y": ty})])
    if kind == "circle":
        (cx, cy), radius = shape["center"], shape["radius"]
        start = shape["start_deg"]
        if start is None:
            dx, dy = current[0] - cx, current[1] - cy
            if abs(math.hypot(dx, dy) - radius) <= max(0.01, 0.005 * radius):
                seg = Circle((cx, cy), radius, math.degrees(math.atan2(dy, dx)), shape["turns"], shape["ccw"])
                return Path({"x": current[0], "y": current[1]}, [seg])
            start = 0.0
        return Path.circle((cx, cy), radius, start_deg=start, turns=shape["turns"], ccw=shape["ccw"])
    if kind == "rectangle":
        return Path.rectangle(shape["corner"], shape["width"], shape["height"])
    return Path.polygon(shape["center"], shape["radius"], shape["sides"], start_deg=shape["start_deg"] or 0.0)


def core_path(shape: dict[str, Any], current: tuple[float, float]) -> dict[str, Any]:
    """The core's path JSON (``iad_cell_coord_plan_json``) of a checked shape: the same path as :func:`build_path`,
    a circle without ``start_deg`` starting where the axes are when they lie on it, else at 0 deg after the lead-in."""
    kind = shape["kind"]
    if kind == "line":
        tx, ty = shape["to"]
        return {"segments": [{"line": {"x": tx, "y": ty}}]}
    if kind == "circle":
        (cx, cy), radius = shape["center"], shape["radius"]
        circle = {"center": [cx, cy], "radius": radius, "turns": shape["turns"], "ccw": shape["ccw"]}
        start = shape["start_deg"]
        if start is None:
            dx, dy = current[0] - cx, current[1] - cy
            if abs(math.hypot(dx, dy) - radius) <= max(0.01, 0.005 * radius):
                return {"start": {"x": current[0], "y": current[1]},
                        "segments": [{"circle": {**circle, "start_deg": math.degrees(math.atan2(dy, dx))}}]}
            start = 0.0
        return {"shape": "circle", **circle, "start_deg": start}
    if kind == "rectangle":
        return {"shape": "rectangle", "corner": list(shape["corner"]), "width": shape["width"],
                "height": shape["height"]}
    return {"shape": "polygon", "center": list(shape["center"]), "radius": shape["radius"], "sides": shape["sides"],
            "start_deg": shape["start_deg"] or 0.0}


def make_group(cell: Any, x: str, y: str) -> Any:
    """The group of the two axes: on a cell of the core (it holds a core handle) :class:`CoreGroup`, whose run the
    sampler follows (the driver's own core group, ``cell.coordinated``, has no live view); else ``cell.coordinated`` (a test double)."""
    if getattr(cell, "_lib", None) is not None:
        return CoreGroup(cell, {"x": x, "y": y})
    return cell.coordinated(x=x, y=y)


def preview(path: Path, current: tuple[float, float]) -> tuple[list[list[float]], list[list[float]] | None]:
    """A polyline of the asked shape (a refused plan still draws): 2 points per straight piece, 48 per turn of an arc;
    and the straight lead-in from the current position when the path starts elsewhere."""
    here = {"x": current[0], "y": current[1]}
    origin = path.start if path.start is not None else here
    lead = None
    if path.start is not None and math.hypot(origin["x"] - here["x"], origin["y"] - here["y"]) > 0.01:
        lead = [[_r(here["x"]), _r(here["y"])], [_r(origin["x"]), _r(origin["y"])]]
    points: list[list[float]] = []
    try:
        pieces = path.pieces(origin)
    except (ValueError, TypeError):
        pieces = []
    for piece in pieces:
        count = 2 if piece.straight else max(ARC_PREVIEW_POINTS,
                                             math.ceil(ARC_PREVIEW_POINTS * abs(piece.sweep) / (2 * math.pi)))
        for i in range(count):
            p = piece.at(i / (count - 1))
            q = [_r(p["x"]), _r(p["y"])]
            if not points or points[-1] != q:
                points.append(q)  # type: ignore[arg-type]
    if not points:
        points = [[_r(origin["x"]), _r(origin["y"])]]  # type: ignore[list-item]
    return points, lead


def _points(plan: Any) -> list[list[float]]:
    ix, iy = plan.axes.index("x"), plan.axes.index("y")
    return [[_r(p[ix]), _r(p[iy])] for p in plan.points]  # type: ignore[misc]


def _readiness(slots: list[Any]) -> list[str]:
    """What the cached status of the two axes says against a run (the run's own checks decide)."""
    out = []
    for s in slots:
        st = s.status
        if not st:
            out.append(f"{s.name}: no status yet")
        else:
            if st.get("servo_on") is False:
                out.append(f"{s.name}: servo off")
            if st.get("homed") is False:
                out.append(f"{s.name}: not homed")
            if st.get("alarm"):
                code = st.get("alarm_code") or 0
                out.append(f"{s.name}: alarm 0x{code:03X} {st.get('alarm_name') or ''}".rstrip())
            if st.get("emergency_stop"):
                out.append(f"{s.name}: emergency stop")
            if st.get("moving"):
                out.append(f"{s.name}: moving")
        if s.fault:
            out.append(f"{s.name}: link fault")
    return out


def _bounds(slot: Any) -> list[float] | None:
    pos = (slot.limits or {}).get("position") or {}
    lo, hi = pos.get("min"), pos.get("max")
    return None if lo is None or hi is None else [lo, hi]


# ====================================================================== the runner


class _Phase:
    """The trace state of one driver run object (lead-in or path)."""

    def __init__(self, run: Any, index: int) -> None:
        self.run = run
        self.index = index  # 0 lead-in, 1 path
        self.counts = dict.fromkeys(ALIASES, 0)
        self.latest: dict[str, float | None] = dict.fromkeys(ALIASES)


class _RunCtx:
    """One run: its plan, group and slots, the threads and the outcome."""

    def __init__(self, run_id: str, plan_json: dict[str, Any], mp: Any, group: Any, slots: list[Any],
                 previous: _RunCtx | None) -> None:  # fmt: skip
        self.run_id = run_id
        self.plan_id: str = plan_json["plan_id"]
        self.plan_json = plan_json
        self.plan = mp
        self.group = group
        self.slots = slots
        self.names = [s.name for s in slots]
        self.axes = dict(plan_json["axes"])
        self.units = dict(plan_json["units"])
        self.planned_s: float = plan_json["planned_s"]
        self.previous = previous
        self.t_request = time.perf_counter()
        self.end_perf: float | None = None
        self.locked = threading.Event()
        self.done = threading.Event()  # the run thread ended
        self.finished = threading.Event()  # the end state is published (trace complete)
        self.lock_error: str | None = None
        self.stop_marks: dict[str, int] = {}  # the STOP counters of the two axes when the run was accepted
        self.outcome: str | None = None
        self.result: Any = None
        self.error: str | None = None
        self.axis: str | None = None
        self.seen_run = False
        self.live_path = False  # a live trace point of the path phase was published
        self.skip = 0
        self.thread: threading.Thread | None = None
        self.supervisor: threading.Thread | None = None

    def alive(self) -> bool:
        """From the run request until the end state is published (the trace complete)."""
        return not self.finished.is_set()


class CoordinatedRunner:
    """Plan / run / stop / status of the Coordinated tab for one :class:`~gui.workbench.Workbench`."""

    def __init__(self, wb: Any) -> None:
        self.wb = wb
        self._op = threading.Lock()  # one plan or run request at a time
        self._data = threading.Lock()  # the fields below
        self._plan_seq = 0
        self._run_seq = 0
        self._state = "idle"
        self._plan: dict[str, Any] | None = None  # the stored passed plan (its response)
        self._plan_obj: Any = None
        self._plan_group: Any = None
        self._plan_cell: Any = None
        self._plan_slots: list[Any] = []
        self._ctx: _RunCtx | None = None  # the current or last run
        self._trace: list[list[float]] = []
        self._phase: str | None = None
        self._result: dict[str, Any] | None = None
        self._error: str | None = None
        self.last_plan: dict[str, Any] | None = None  # for the test report (passed or refused)
        self.last_run: dict[str, Any] | None = None

    # ------------------------------------------------------------------ queries

    def active(self) -> bool:
        """True while a run (its thread or the trace's last flush) is going."""
        ctx = self._ctx
        return ctx is not None and ctx.alive()

    def axis_in_run(self, name: str) -> bool:
        ctx = self._ctx
        return ctx is not None and ctx.alive() and name in ctx.names

    def summary(self) -> dict[str, Any]:
        """The state's ``coord`` block."""
        with self._data:
            state, ctx = self._state, self._ctx
            if state == "planned" and self._plan is not None:
                return {"state": state, "plan_id": self._plan["plan_id"], "run_id": None,
                        "axes": [self._plan["axes"]["x"], self._plan["axes"]["y"]]}  # fmt: skip
            if ctx is not None and (state in RUN_STATES or state in END_STATES):
                return {"state": state, "plan_id": ctx.plan_id, "run_id": ctx.run_id, "axes": list(ctx.names)}
            return {"state": state, "plan_id": None, "run_id": None, "axes": []}

    def status(self, since: int = 0) -> dict[str, Any]:
        """GET /api/wb/coord/status: the state, the trace from index ``since`` and, at the end, the result."""
        with self._data:
            state, ctx = self._state, self._ctx
            group = ctx.group if ctx is not None else self._plan_group
            count = len(self._trace)
            since = 0 if since < 0 or since > count else since
            trace = [list(p) for p in self._trace[since:]]
            last = self._trace[-1] if self._trace else None
            plan = self._plan if state == "planned" else None
            result, error, phase = self._result, self._error, self._phase
        running = state in RUN_STATES
        out: dict[str, Any] = {"state": state, "plan_id": None, "run_id": None, "axes": {}, "units": {},
                               "phase": phase if running else None, "elapsed_s": None, "planned_s": None,
                               "trace": trace, "next": count, "position": None, "plan": plan,
                               "result": result if state in END_STATES else None, "error": error,
                               "live_trace": not isinstance(group, CoreGroup)}  # fmt: skip
        if plan is not None:
            out.update(plan_id=plan["plan_id"], axes=dict(plan["axes"]), units=dict(plan["units"]),
                       planned_s=plan["planned_s"])  # fmt: skip
            out["position"] = self._cached_position([self.wb.axes.get(plan["axes"][a]) for a in ALIASES])
        elif ctx is not None and (running or state in END_STATES):
            end = ctx.end_perf if not running and ctx.end_perf is not None else time.perf_counter()
            out.update(plan_id=ctx.plan_id, run_id=ctx.run_id, axes=dict(ctx.axes), units=dict(ctx.units),
                       planned_s=ctx.planned_s, elapsed_s=round(end - ctx.t_request, 3))  # fmt: skip
            if running:
                out["position"] = None if last is None else [last[1], last[2]]
            else:
                out["position"] = self._cached_position([self.wb.axes.get(n) for n in ctx.names])
        return out

    @staticmethod
    def _cached_position(slots: list[Any]) -> list[float] | None:
        values = []
        for s in slots:
            st = None if s is None else s.status
            if not st or st.get("position_mm") is None:
                return None
            values.append(st["position_mm"])
        return values

    # ------------------------------------------------------------------ plan

    def plan(self, body: dict[str, Any]) -> dict[str, Any]:
        """POST /api/wb/coord/plan: a dry run (reads only; nothing moves)."""
        req = parse_plan(body)
        if self.active():
            raise BackendError(RUN_ACTIVE, 409)
        slots = [self.wb._slot(req["axes"][a]) for a in ALIASES]
        with self._op:
            if self.active():
                raise BackendError(RUN_ACTIVE, 409)
            held = self._hold(slots)
            try:
                cell = self.wb.cell
                if cell is None:
                    raise BackendError("No axis connected.", 409)
                group = make_group(cell, slots[0].name, slots[1].name)
                try:
                    pos = group.positions()
                except Exception as exc:  # noqa: BLE001 - the driver's text
                    raise _bus_error(exc) from None
                current = (pos["x"], pos["y"])
                try:
                    path = build_path(req["shape"], current)
                except (ValueError, TypeError) as exc:
                    raise BackendError(f"Bad shape: {exc}", 400) from None
                refusal = None
                mp = None
                too_big = pre_plan_reasons(req["shape"], req["chord_error"], path, current, slots)
                if too_big:  # no waypoint is planned for a path that cannot pass: the work would grow with its size
                    refusal = (f"coordinated move refused, nothing planned or sent: {too_big[0]}", too_big, None, False)
                else:
                    try:
                        if isinstance(group, CoreGroup):
                            mp = group.plan(core_path(req["shape"], current), req["speed"], req["accel_g"],
                                            chord_error=req["chord_error"])
                        else:
                            mp = group.plan(path, req["speed"], req["accel_g"], chord_error=req["chord_error"])
                    except E.SoftLimitError as exc:
                        reasons = [str(r) for r in (getattr(exc, "reasons", None) or [str(exc)])]
                        refusal = (str(exc), reasons, getattr(exc, "max_speed", None), True)
                    except ValueError as exc:
                        refusal = (f"coordinated move refused, nothing sent: {exc}", [str(exc)], None, False)
                    except Exception as exc:  # noqa: BLE001 - a bus or driver error: no answer to the plan
                        self.wb.event("error", f"Coordinated plan failed: {_describe(exc)}")
                        raise _bus_error(exc) from None
            finally:
                self._release(held)
            common = {
                "axes": dict(req["axes"]),
                "units": {a: (s.limits or {}).get("units", "mm") for a, s in zip(ALIASES, slots)},
                "shape": req["shape"]["kind"], "current": [_r(current[0]), _r(current[1])],
                "bounds": {a: _bounds(s) for a, s in zip(ALIASES, slots)},
            }  # fmt: skip
            not_ready = _readiness(slots)
            common.update(ready=not not_ready, not_ready=not_ready, t=time.time())
            over = f"{req['shape']['kind']} over x={slots[0].name}, y={slots[1].name}"
            if refusal is not None:
                error, reasons, max_speed, by_speed = refusal
                path_pts, lead_pts = preview(path, current)
                out = {"passed": False, "plan_id": None, **common, "error": error, "reasons": reasons,
                       "max_speed": max_speed, "path": path_pts, "lead_in": lead_pts, "warnings": []}  # fmt: skip
                with self._data:
                    self._drop_plan()
                    self._state = "idle"
                    self._result = self._error = None
                    self._trace, self._phase = [], None
                    self.last_plan = {"plan_id": None, "text": over, "passed": False, "reasons": reasons,
                                      "max_speed": max_speed}  # fmt: skip
                tail = "" if not by_speed else ("; no speed makes it feasible" if max_speed is None
                                                else f"; largest feasible speed about {max_speed:g}")
                self.wb.event("warn", f"Coordinated plan refused: {'; '.join(reasons)}{tail}")
                return out
            assert mp is not None
            lead = mp.lead_in
            lead_s = lead.duration if lead is not None else 0.0
            warnings = list(mp.warnings) + ([f"lead-in: {w}" for w in lead.warnings] if lead is not None else [])
            with self._data:
                if self.wb.cell is not cell:
                    raise BackendError("The connected axes changed while planning: press Plan again.", 409)
                self._plan_seq += 1
                plan_id = f"p{self._plan_seq}"
                out = {
                    "passed": True, "plan_id": plan_id, **common, "mode": mp.mode, "summary": mp.summary(),
                    "steps": mp.steps, "length": _r(mp.length), "duration_s": _r(mp.duration), "lead_in_s": _r(lead_s),
                    "planned_s": _r(mp.duration + lead_s), "path_speed": mp.path_speed, "accel_g": mp.accel_g,
                    "chord_error": mp.chord_error, "shortest_step_s": _r(mp.shortest_step_s, 5),
                    "min_step_s": _r(mp.min_step_s, 5), "est_deviation": _r(mp.est_deviation, 5),
                    "est_chord_error": _r(mp.est_chord_error, 5), "warnings": warnings, "reasons": [],
                    "max_speed": None, "error": None, "path": _points(mp),
                    "lead_in": None if lead is None else _points(lead),
                }  # fmt: skip
                self._plan, self._plan_obj, self._plan_group, self._plan_cell = out, mp, group, cell
                self._plan_slots = slots
                self._state = "planned"
                self._result = self._error = None
                self._trace, self._phase = [], None
                self.last_plan = {"plan_id": plan_id, "text": over, "passed": True, "mode": mp.mode, "steps": mp.steps,
                                  "planned_s": out["planned_s"]}  # fmt: skip
            self.wb.event("info", f"Coordinated plan {plan_id} passed: {over} ({mp.mode}, {mp.steps} steps, "
                                  f"{out['planned_s']:.2f} s)")  # fmt: skip
            return out

    def _hold(self, slots: list[Any]) -> list[Any]:
        """Both axis locks for planning (at most LOCK_WAIT_S each), else 409."""
        held: list[Any] = []
        for s in sorted(slots, key=lambda s: s.name):
            if self.axis_in_run(s.name):
                self._release(held)
                raise BackendError(f"{s.name} is in a coordinated run: STOP ALL (Space / Esc) ends it.", 409)
            if not s.lock.acquire(timeout=LOCK_WAIT_S):
                self._release(held)
                raise BackendError(f"{s.name} is busy with {s.busy or 'another request'}; try again.", 409)
            held.append(s)
            s.busy = BUSY_PLAN
        return held

    @staticmethod
    def _release(held: list[Any]) -> None:
        for s in held:
            s.busy = None
            s.lock.release()

    def _drop_plan(self) -> None:
        self._plan = self._plan_obj = self._plan_group = self._plan_cell = None
        self._plan_slots = []

    def invalidate(self, reason: str) -> None:
        """Drop a stored plan (the connected axes changed). A run in progress is not touched."""
        with self._data:
            if self._plan is None or (self._ctx is not None and self._ctx.alive()):
                return
            plan_id = self._plan["plan_id"]
            self._drop_plan()
            if self._state == "planned":
                self._state = "idle"
        self.wb.event("info", f"Coordinated plan {plan_id} dropped: {reason}; press Plan again")

    # ------------------------------------------------------------------ run

    def run(self, body: dict[str, Any]) -> dict[str, Any]:
        """POST /api/wb/coord/run {plan_id}: run the stored passed plan once."""
        plan_id = body.get("plan_id")
        if not isinstance(plan_id, str) or not plan_id:
            raise BackendError("plan_id is missing: press Plan first.", 400)
        with self._op:
            with self._data:
                if self._ctx is not None and self._ctx.alive():
                    raise BackendError(RUN_ACTIVE, 409)
                plan = self._plan
                if plan is None or plan["plan_id"] != plan_id or self.wb.cell is not self._plan_cell:
                    raise BackendError(OUT_OF_DATE, 409)
                slots = list(self._plan_slots)
                if any(self.wb.axes.get(s.name) is not s for s in slots):
                    raise BackendError(OUT_OF_DATE, 409)
                busy = next((s for s in slots if s.busy), None)
                if busy is not None:
                    raise BackendError(f"{busy.name} is busy with {busy.busy}; try again.", 409)
                self.wb._refuse_pio_hold([s.name for s in slots], "the coordinated run")
                self._run_seq += 1
                ctx = _RunCtx(f"r{self._run_seq}", plan, self._plan_obj, self._plan_group, slots, self._ctx)
                ctx.stop_marks = self.wb.stop_epochs(ctx.names)
                self._ctx = ctx
                self._state = "starting"
                self._trace, self._phase, self._result, self._error = [], None, None, None
            ctx.thread = threading.Thread(target=self._run_thread, args=(ctx,), name=f"wb-coord-{ctx.run_id}",
                                          daemon=True)  # fmt: skip
            ctx.supervisor = threading.Thread(target=self._supervise, args=(ctx,), name=f"wb-coord-trace-{ctx.run_id}",
                                              daemon=True)  # fmt: skip
            try:
                ctx.thread.start()
                ctx.supervisor.start()
            except RuntimeError as exc:  # a thread could not be started
                if ctx.thread.is_alive():
                    ctx.group.stop()
                ctx.outcome, ctx.error = "failed", _describe(exc)
                ctx.done.set()
                self._publish(ctx)
                raise BackendError(f"The run could not start: {ctx.error}", 503) from None
            deadline = time.monotonic() + RUN_WAIT_S
            while time.monotonic() < deadline:
                if ctx.finished.wait(0.01):
                    break
                run = getattr(ctx.group, "_active_run", None)
                if run is not None:
                    self._mark_running(ctx, run)
                    break
        if ctx.finished.is_set():
            if ctx.lock_error is not None:
                raise BackendError(ctx.lock_error, 409)
            if ctx.outcome == "refused":
                raise BackendError(ctx.error or "refused", 409, kind="refused", sent=False)
        with self._data:
            state = self._state if self._ctx is ctx else "running"
        return {"state": state, "run_id": ctx.run_id, "plan_id": ctx.plan_id, "planned_s": ctx.planned_s}

    def _run_thread(self, ctx: _RunCtx) -> None:
        """Holds both axis locks (RLocks: taken and released here), consumes the plan and runs it."""
        held: list[Any] = []
        try:
            for s in sorted(ctx.slots, key=lambda s: s.name):
                if not s.lock.acquire(timeout=LOCK_WAIT_S):
                    ctx.lock_error = f"{s.name} is busy with {s.busy or 'another request'}; try again."
                    return
                held.append(s)
            for s in held:
                s.busy = BUSY_RUN
            if self.wb.stopped_since(ctx.stop_marks):  # a STOP came after Run was accepted: the group's own STOP
                ctx.lock_error = STOPPED_BEFORE_START  # (or the core's) only counts one that comes after the run began
                return
            with self._data:
                if self._plan is not None and self._plan["plan_id"] == ctx.plan_id:
                    self._drop_plan()  # one run per plan
            ctx.locked.set()
            try:
                ctx.result = ctx.group.run_plan(ctx.plan)
                ctx.outcome = "completed" if ctx.result.completed else "stopped"
            except CoordinatedMotionError as exc:
                ctx.outcome, ctx.result, ctx.error, ctx.axis = "stopped", exc.result, str(exc), exc.axis
            except Exception as exc:  # noqa: BLE001 - a refusal before sending, or a failure
                ctx.error = _describe(exc)
                ctx.result = getattr(ctx.group, "last_result", None)
                refused = isinstance(exc, (E.EcError, ValueError)) and ctx.result is None
                ctx.outcome = "refused" if refused else "failed"
        finally:
            ctx.end_perf = time.perf_counter()
            for s in held:
                s.busy = None
                s.lock.release()
            ctx.done.set()

    def _mark_running(self, ctx: _RunCtx, run: Any) -> None:
        phase = "lead_in" if ctx.plan.lead_in is not None and run.plan is ctx.plan.lead_in else "path"
        first = False
        with self._data:
            if self._ctx is ctx and self._state in RUN_STATES:
                self._state = "running"
                self._phase = phase
                first, ctx.seen_run = not ctx.seen_run, True
        if first:
            self.wb.event("info", f"Coordinated run {ctx.run_id} started (plan {ctx.plan_id})")

    def _supervise(self, ctx: _RunCtx) -> None:
        """The live trace (every SAMPLE_S while the run thread lives), then the end state."""
        current: _Phase | None = None
        assert ctx.thread is not None
        try:
            while True:
                alive = ctx.thread.is_alive()
                run = getattr(ctx.group, "_active_run", None)
                if run is not None and (current is None or run is not current.run):
                    if current is not None:
                        self._flush(ctx, current)  # the old run object keeps its samples
                    index = 0 if ctx.plan.lead_in is not None and run.plan is ctx.plan.lead_in else 1
                    current = _Phase(run, index)
                    self._mark_running(ctx, run)
                if current is not None:
                    self._flush(ctx, current)
                if not alive:
                    break
                ctx.done.wait(SAMPLE_S)
            ctx.thread.join()
            if current is not None:
                self._flush(ctx, current)
            if not ctx.live_path and ctx.result is not None and getattr(ctx.result, "plan", None) is ctx.plan:
                self._from_result(ctx)  # no live point of the path phase (the core reports its trace at the end)
        finally:
            self._publish(ctx)

    def _flush(self, ctx: _RunCtx, ph: _Phase) -> None:
        """New samples of one run object as trace points: each new sample holds the other axis's latest value."""
        new = []
        for alias in ALIASES:
            series = ph.run.samples.get(alias, [])
            end = len(series)
            new.extend((t, alias, p) for t, p in series[ph.counts[alias]:end])
            ph.counts[alias] = end
        if not new:
            return
        new.sort(key=lambda e: e[0])
        offset = ph.run.t0 - ctx.t_request
        points = []
        for t, alias, p in new:
            ph.latest[alias] = p
            x, y = ph.latest["x"], ph.latest["y"]
            if x is not None and y is not None:
                points.append([round(t + offset, 4), round(x, 4), round(y, 4), ph.index])
        if ph.index == 1 and points:
            ctx.live_path = True
        self._append(ctx, points)

    def _from_result(self, ctx: _RunCtx) -> None:
        """No live sample was seen: the result's merged trace for the path phase."""
        result = ctx.result
        try:
            ix, iy = result.plan.axes.index("x"), result.plan.axes.index("y")
        except (AttributeError, ValueError):
            return
        end = ctx.end_perf if ctx.end_perf is not None else time.perf_counter()
        offset = (end - ctx.t_request) - result.duration_s
        self._append(ctx, [[round(row[0] + offset, 4), round(row[1 + ix], 4), round(row[1 + iy], 4), 1]
                           for row in result.trace])  # fmt: skip

    def _append(self, ctx: _RunCtx, points: list[list[float]]) -> None:
        with self._data:
            if self._ctx is not ctx:
                return
            for p in points:
                if len(self._trace) >= MAX_TRACE:
                    ctx.skip += 1
                    if ctx.skip % 2:
                        continue
                self._trace.append(p)

    def _publish(self, ctx: _RunCtx) -> None:
        """The end state (after the trace's last flush), the result and the event."""
        level, message = "info", ""
        with self._data:
            if self._ctx is ctx:
                self._phase = None
                if ctx.lock_error is not None:  # nothing happened: back to the stored plan
                    self._ctx = ctx.previous
                    self._state = "planned" if self._plan is not None else "idle"
                else:
                    ctx.previous = None  # keep no chain of old runs
                    self._state = ctx.outcome or "failed"
                    self._error = ctx.error
                    self._result = None if ctx.result is None else self._result_json(ctx)
                    r = self._result
                    self.last_run = {"run_id": ctx.run_id, "plan_id": ctx.plan_id, "state": self._state,
                                     "error": ctx.error, "result": r, "units": ctx.units.get("x", "mm")}  # fmt: skip
                    units = ctx.units.get("x", "mm")
                    if self._state == "completed" and r is not None:
                        message = (f"Coordinated run {ctx.run_id} completed in {r['duration_s']:.2f} s: deviation max "
                                   f"{r['max_deviation']:.3f} / mean {r['mean_deviation']:.3f} {units}, "
                                   f"{r['late_frames']} late frames")  # fmt: skip
                    elif self._state == "stopped":
                        level, message = "warn", f"Coordinated run {ctx.run_id} stopped: {ctx.error}"
                    elif self._state == "refused":
                        level, message = "warn", f"Coordinated run {ctx.run_id} refused, nothing sent: {ctx.error}"
                    else:
                        level, message = "error", f"Coordinated run {ctx.run_id} failed: {ctx.error}"
        ctx.finished.set()
        if message:
            self.wb.event(level, message)

    @staticmethod
    def _result_json(ctx: _RunCtx) -> dict[str, Any]:
        r = ctx.result
        completed = bool(r.completed) and ctx.outcome == "completed"
        final = [r.final.get("x"), r.final.get("y")]
        return {
            "completed": completed, "error": None if completed else (ctx.error or r.error), "axis": ctx.axis,
            "max_deviation": _r(r.max_deviation), "mean_deviation": _r(r.mean_deviation), "frames": r.frames,
            "skipped_frames": r.skipped_frames, "late_frames": r.late_frames,
            "worst_late_ms": _r(r.worst_late_s * 1000, 1), "worst_skew_ms": _r(r.worst_skew_s * 1000, 1),
            "duration_s": _r(r.duration_s, 3), "planned_s": _r(r.plan.duration, 3),
            "samples": sum(len(v) for v in r.samples.values()), "sample_spacing_ms": round(r.sample_spacing_s * 1000),
            "final": None if None in final else [_r(final[0], 3), _r(final[1], 3)], "warnings": list(r.warnings),
            "summary": r.summary(),
        }  # fmt: skip

    # ------------------------------------------------------------------ stop

    def stop(self) -> dict[str, Any]:
        """POST /api/wb/coord/stop: decel stop on every axis of the current group (ends a run). Never waits for a lock."""
        with self._data:
            ctx = self._ctx
            if ctx is not None and ctx.alive():
                group = ctx.group
            elif self._plan_group is not None:
                group = self._plan_group
            elif ctx is not None and all(self.wb.axes.get(s.name) is s for s in ctx.slots):
                group = ctx.group
            else:
                group = None
        if group is None:
            return {"stopped": [], "errors": {}}
        self.wb.note_stop_axes(group.cell_names.values())  # a run accepted but not yet begun is cancelled
        results = group.stop()
        stopped, errors = [], {}
        for alias, value in results.items():
            name = group.cell_names.get(alias, alias)
            if isinstance(value, BaseException):
                errors[name] = _describe(value)
            else:
                stopped.append(name)
        self.wb.event("warn" if errors else "info", "Coordinated STOP: " + (
            "; ".join(f"{n}: {e}" for n, e in errors.items()) if errors else f"{', '.join(stopped)} stopped"))
        return {"stopped": stopped, "errors": errors}

    def note_stop(self) -> None:
        """A STOP reached the axes from outside (STOP ALL, an axis STOP): the core's group repeats it while its run call
        is still planning (:meth:`corecoord.CoreGroup.note_stop`). Nothing else changes; the run ends through the axes."""
        ctx = self._ctx
        if ctx is not None and ctx.alive():
            note = getattr(ctx.group, "note_stop", None)
            if note is not None:
                note()

    def stop_for(self, names: list[str], wait: float = STOP_JOIN_S) -> bool:
        """Stop a run that uses one of ``names`` (a disconnect) and wait at most ``wait`` for it to end."""
        ctx = self._ctx
        if ctx is None or not ctx.alive() or not set(ctx.names) & set(names):
            return False
        try:
            ctx.group.stop()
        except Exception as exc:  # noqa: BLE001 - the disconnect stops each axis again
            self.wb.event("error", f"Coordinated run {ctx.run_id}: stop failed: {_describe(exc)}")
        if ctx.supervisor is not None:
            ctx.supervisor.join(wait)
        return True

    def close(self) -> None:
        ctx = self._ctx
        if ctx is not None and ctx.alive():
            self.stop_for(ctx.names)
