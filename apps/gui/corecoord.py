"""The Coordinated tab's group on the C++ core.

The tab's runner (:mod:`coordinated`) works with a group object: ``positions()``, ``plan(...)``, ``run_plan(plan)``,
``stop()``. The core runs coordinated motion itself (``iad_cell_coord_plan_json`` / ``iad_cell_coord_run_json``,
``NativeCell.coordinated_plan`` / ``coordinated_run``), and :class:`CoreGroup` gives the runner the same few names over
those two calls.

What differs from the runner of the 1.0.x fallback, all of it from the shape of the core's interface:

* The core's run plans again from where the axes are when it starts, then runs; it has no "run this plan object" call.
  The plan shown on the page and the plan that runs come from the same request, and the run's own checks refuse a path
  that is no longer feasible before anything is sent.
* The core's run reports once, when it ends. There is no live trace; the trace of the run is returned with the result
  (also when the run was stopped). The lead-in of a plan that does not start where the axes are is not in that trace.
* The core notes an outside stop from the start of its run, not from the start of its planning. A STOP that arrives in
  between (the start check included) is repeated (:meth:`CoreGroup.note_stop`) until the run call returns, so it
  still ends the run; one noted before the core's run call is made refuses the run.

Reaching the core: the cell the workbench holds is the binding's ``Cell``; :func:`core_cell` takes a borrowed view of
its core handle as a ``NativeCell`` (the same step the driver's own modules take), which must never close it.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any

from iai_actuator_driver import errors as E
from iai_actuator_driver import native
from iai_actuator_driver.coord import CoordinatedMotionError

__all__ = ["CoreGroup", "CorePlan", "CoreResult", "core_cell"]

START_GRACE_S = 0.75  # the core's plan and its checks refuse within this; after it a run counts as started
STOP_REPEAT_S = 0.2  # a STOP that arrives while the core still plans is repeated this often ...
STOP_REPEAT_FOR_S = 3.0  # ... for at most this long, until the run call returns
STOPPED_BEFORE_START = "STOP was pressed before the run started: nothing was sent. Plan again if you still want it."


def core_cell(cell: Any) -> Any:
    """A borrowed ``NativeCell`` on the binding ``Cell``'s core handle: its handle stays the cell's own, so it is
    never closed or released here."""
    view = native.NativeCell(cell._lib, cell._h())
    view._finalizer.detach()
    return view


class CorePlan:
    """The core's plan (``MotionPlan`` as JSON) with the names the runner reads, and the request that made it."""

    def __init__(self, data: Mapping[str, Any], request: Mapping[str, Any]) -> None:
        self.data = dict(data)
        self.request = dict(request)
        self.mode: str = data["mode"]
        self.axes: list[str] = list(data["axes"])
        self.points: list[list[float]] = [list(p) for p in data["points"]]
        self.steps: int = int(data.get("steps") or 0)
        self.length: float = float(data.get("length") or 0.0)
        self.duration: float = float(data.get("duration") or 0.0)
        self.path_speed: float = float(data.get("path_speed") or 0.0)
        self.accel_g: float = float(data.get("accel_g") or 0.0)
        self.chord_error: float = float(data.get("chord_error") or 0.0)
        self.shortest_step_s: float = float(data.get("shortest_step_s") or 0.0)
        self.min_step_s: float = float(data.get("min_step_s") or 0.0)
        self.est_deviation: float = float(data.get("est_deviation") or 0.0)
        self.est_chord_error: float = float(data.get("est_chord_error") or 0.0)
        self.warnings: list[str] = [str(w) for w in data.get("warnings") or []]
        lead = data.get("lead_in")
        self.lead_in: CorePlan | None = None if not lead else CorePlan(lead, request)

    def summary(self) -> str:
        return str(self.data.get("summary") or f"{self.mode} plan over {', '.join(self.axes)}")


class CoreResult:
    """The core's run result as JSON with the names the runner reads (``CoordinatedResult``'s)."""

    def __init__(self, data: Mapping[str, Any], plan: CorePlan) -> None:
        self.data = dict(data)
        self.plan = plan
        self.completed = bool(data.get("completed"))
        self.error: str | None = data.get("error")
        self.samples: dict[str, list[tuple[float, float]]] = {
            str(a): [(float(t), float(p)) for t, p in rows] for a, rows in (data.get("samples") or {}).items()}
        self.trace: list[list[float]] = [list(row) for row in data.get("trace") or []]
        self.max_deviation = float(data.get("max_deviation") or 0.0)
        self.mean_deviation = float(data.get("mean_deviation") or 0.0)
        self.frames = int(data.get("frames") or 0)
        self.skipped_frames = int(data.get("skipped_frames") or 0)
        self.late_frames = int(data.get("late_frames") or 0)
        self.worst_late_s = float(data.get("worst_late_s") or 0.0)
        self.worst_skew_s = float(data.get("worst_skew_s") or 0.0)
        self.duration_s = float(data.get("duration_s") or 0.0)
        self.warnings: list[str] = [str(w) for w in data.get("warnings") or []]
        self.final: dict[str, float] = {str(a): float(p) for a, p in (data.get("final") or {}).items()}
        self.sample_spacing_s = float(data.get("sample_spacing_s") or 0.0)

    def summary(self) -> str:
        return str(self.data.get("summary") or ("completed" if self.completed else f"STOPPED: {self.error}"))


class _CoreRun:
    """What the runner's supervisor reads of a run in progress: its plan and start time. The core gives no live samples."""

    def __init__(self, plan: CorePlan, t0: float) -> None:
        self.plan = plan
        self.t0 = t0
        self.samples: dict[str, list[tuple[float, float]]] = {}


class CoreGroup:
    """Two (or more) cell axes under path names (``x``, ``y``), planned and run by the core."""

    def __init__(self, cell: Any, cell_names: Mapping[str, str]) -> None:
        self.cell = cell
        self.cell_names: dict[str, str] = dict(cell_names)
        self.names: list[str] = list(self.cell_names)
        self.last_result: CoreResult | None = None
        self._core = core_cell(cell)
        self._lock = threading.Lock()
        self._started: float | None = None  # perf_counter at the start of the run call, None when none is in flight
        self._runs: tuple[_CoreRun, _CoreRun | None] | None = None
        self._repeating = False
        self._stops = 0  # STOPs noted so far (note_stop): a run call that sees it change refuses to start

    # ------------------------------------------------------------------ reads

    def positions(self) -> dict[str, float]:
        """The position of every axis (one status read each)."""
        return {alias: float(self.cell[name].status().position_mm) for alias, name in self.cell_names.items()}

    # ------------------------------------------------------------------ plan

    def request(self, path: Mapping[str, Any], speed: float, accel_g: float, chord_error: float) -> dict[str, Any]:
        """The core's request for ``path`` (the JSON of ``iad_cell_coord_plan_json``)."""
        return {"axes": dict(self.cell_names), "path": dict(path), "speed": float(speed), "accel_g": float(accel_g),
                "chord_error": float(chord_error)}

    def plan(self, path: Mapping[str, Any], speed: float, accel_g: float, *, chord_error: float = 0.05) -> CorePlan:
        """Dry run: the plan of ``path`` (the core's path JSON) from where the axes are now. Reads only, nothing moves.
        An infeasible path raises ``SoftLimitError`` with ``reasons`` and ``max_speed``; one that moves no axis, a
        ``ValueError``."""
        request = self.request(path, speed, accel_g, chord_error)
        try:
            data = self._core.coordinated_plan(request)
        except E.SoftLimitError as exc:
            details = getattr(exc, "iad_details", None) or {}
            exc.reasons = [str(r) for r in details.get("reasons") or []]  # type: ignore[attr-defined]
            exc.max_speed = details.get("max_speed")  # type: ignore[attr-defined]
            raise
        return CorePlan(data, request)

    # ------------------------------------------------------------------ run

    @property
    def _active_run(self) -> _CoreRun | None:
        """The run the supervisor follows: ``None`` until the core's plan and checks have had :data:`START_GRACE_S` (a
        refusal comes back within it), then the lead-in (as long as its planned duration) and the path."""
        with self._lock:
            started, runs = self._started, self._runs
        if started is None or runs is None or time.perf_counter() - started < START_GRACE_S:
            return None
        path_run, lead_run = runs[0], runs[1]
        if lead_run is not None and time.perf_counter() - started < START_GRACE_S + lead_run.plan.duration:
            return lead_run
        return path_run

    def run_plan(self, plan: CorePlan) -> CoreResult:
        """Plan again from where the axes are and run (one blocking call); returns the result. A refusal raises before
        anything is sent. A run ended from outside (a STOP, a fault, a late frame) raises ``CoordinatedMotionError``
        with the result so far. A STOP that arrives while the start is still being checked is repeated like one that
        arrives while the core plans, and a run that has seen one before its first frame is refused."""
        with self._lock:
            self._started, self._runs = time.perf_counter(), None  # in flight: a STOP from here on is repeated
            stops = self._stops
        try:
            self._check_start(plan)
            t0 = time.perf_counter()
            path_run = _CoreRun(plan, t0)
            lead_run = _CoreRun(plan.lead_in, t0) if plan.lead_in is not None else None
            with self._lock:
                if self._stops != stops:  # a STOP came during the check: the axes have it, the run must not follow
                    raise E.EcNotReadyError(STOPPED_BEFORE_START)
                self.last_result = None
                self._started, self._runs = t0, (path_run, lead_run)
            try:
                data = self._core.coordinated_run(plan.request)
            except E.EcDriverError as exc:
                details = getattr(exc, "iad_details", None) or {}
                if details.get("kind") != "CoordinatedMotionError":
                    raise
                partial = CoreResult(details.get("result") or {"mode": plan.mode, "axes": plan.axes}, plan)
                raise CoordinatedMotionError(str(exc), partial, axis=details.get("axis"),  # type: ignore[arg-type]
                                             stop=details.get("stop")) from exc
        finally:
            with self._lock:
                self._started = self._runs = None
        result = CoreResult(data, plan)
        with self._lock:
            self.last_result = result
        return result

    def _check_start(self, plan: CorePlan) -> None:
        """Refuse a plan whose start is not where the axes are now (``plan again``). The core
        plans again from where the axes are, so without this check an axis moved after Plan would run a lead-in
        and a path the page never showed. Nothing is sent."""
        first = plan.lead_in if plan.lead_in is not None else plan
        tolerance = max(float(plan.data.get("band") or 0.0), 0.05)
        here = self.positions()
        for i, alias in enumerate(plan.axes):
            start = first.points[0][i]
            if abs(here[alias] - start) > tolerance:
                raise E.EcNotReadyError(f"axis {alias} is at {here[alias]:.3f}, the plan starts at {start:.3f}: plan "
                                        "again from the current position; nothing sent")

    # ------------------------------------------------------------------ stop

    def stop(self, *, timeout: float = 5.0) -> dict[str, Any]:
        """Decel stop on every axis of the group (``Cell.stop_all`` on those axes); ends a run of this group. Returns
        ``{path name: status or exception}``; never raises for one axis."""
        self.note_stop()
        by_cell = self.cell.stop_all([self.cell_names[n] for n in self.names], timeout=timeout, return_exceptions=True)
        return {n: by_cell.get(self.cell_names[n]) for n in self.names}

    def note_stop(self) -> None:
        """A STOP went to the axes from outside (this group's stop, STOP ALL, an axis STOP). While the run call is
        still going the STOP is repeated: the core notes an outside stop from the start of its run, so one that came
        during its planning would otherwise be missed and the run would start after it."""
        with self._lock:
            self._stops += 1
            if self._started is None or self._repeating:
                return
            self._repeating = True
        threading.Thread(target=self._repeat_stop, name="gui-coord-stop", daemon=True).start()

    def _repeat_stop(self) -> None:
        deadline = time.monotonic() + STOP_REPEAT_FOR_S
        try:
            while time.monotonic() < deadline:
                time.sleep(STOP_REPEAT_S)
                with self._lock:
                    if self._started is None:
                        return
                try:
                    self.cell.stop_all([self.cell_names[n] for n in self.names], timeout=2.0, return_exceptions=True)
                except Exception:  # noqa: BLE001 - the run's own stop path reports it
                    pass
        finally:
            with self._lock:
                self._repeating = False
