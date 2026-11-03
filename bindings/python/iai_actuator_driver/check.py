"""The customer compatibility self-test (iai-check).

Read-only by default (nothing is written to the EC): link, slave id, index table layout, identification, soft
limits vs the configured stroke, speed limit (MAXV item 7.4) vs the speed cap, status, alarm, alarm history,
position table, direct move block, Modbus control, safety speed, the full unit information
(:meth:`Elecylinder.unit_info`, stored in the report's ``unit_info``; it never fails the check) and the parameter
values against the documented ranges (``parameter_ranges``: WARN at most; on a CON unit a No.10 below the minimum
positioning band would raise cold-start alarm 0x0A1 at the next reset). With ``motion=True`` it also makes a short
move (default +/-5 mm at 10 mm/s) from the current position and back, inside the effective stroke, and then
restores the servo / Modbus-control state it found.

On the core: on a core axis :func:`run_check` runs ``iad_axis_self_test`` (``core/src/check.cpp``: read-only unless
``motion``) and builds the shared :class:`CheckReport` and :class:`CheckItem` (``_shared/check.py``) from the core's
JSON report (the test's own:
``iad_last_result_json``, read on the thread that ran it); ``backend`` is the backend in use, the report file is written
in Python (:meth:`CheckReport.write_json`). ``on_item`` sees each line as the core finishes it, on the caller's thread:
the test runs on a helper thread (``iad-call:<caller's name>``) while the caller reads the lines finished so far
(``iad_axis_self_test_items_json``) between waits of 50 ms, and the lines left are handed over from the report before
:func:`run_check` returns. Ctrl+C, or an exception raised by ``on_item``, cancels the waits of the axis (a decel stop
when the motion check moves) and is raised once the test has ended. An axis that is not a core axis (a test double),
or one whose bus was closed, runs the shared Python :func:`run_check` (on a closed bus its link line fails and the
rest is skipped, without I/O).

Every other name is the shared module's (``_shared/check.py``; its functions re-bound to this module, as in
:mod:`iai_actuator_driver.info`).
"""

from __future__ import annotations

import ctypes
import dataclasses
import json
import threading
import types
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

from . import native as _n
from . import _shared
from ._shared import check as _source
from .info import _core_axis

if TYPE_CHECKING:
    from .ec import Elecylinder

for _name, _value in vars(_source).items():
    if _name.startswith("__"):
        continue
    if isinstance(_value, types.FunctionType) and _value.__globals__ is vars(_source):
        _value = _shared.rebound(_value, globals())
    globals()[_name] = _value

PASS, FAIL, WARN, SKIP = _source.PASS, _source.FAIL, _source.WARN, _source.SKIP
CheckItem = _source.CheckItem
CheckReport = _source.CheckReport

_PY_RUN_CHECK = globals()["run_check"]  # the shared Python run_check: a test double's, a closed bus's
_SLICE_S = 0.05  # longest wait of the calling thread between two looks at the lines and at Ctrl+C
_SPANS = ("stroke", "configured_stroke", "ec_soft_limits_mm")  # (min, max) tuples in the report
_FLOATS = ("ec_max_speed_mm_s", "speed_cap_mm_s")


def _item(d: Mapping[str, Any]) -> CheckItem:
    data = d.get("data")
    return CheckItem(str(d["name"]), str(d["result"]), str(d["detail"]),
                     dict(data) if isinstance(data, Mapping) else {})  # fmt: skip


def _report(record: Mapping[str, Any]) -> CheckReport:
    """The shared report from the core's JSON report (the fields of :class:`CheckReport`; ``backend`` its default)."""
    values = {f.name: record.get(f.name) for f in dataclasses.fields(CheckReport) if f.name not in ("items", "backend")}
    for name in _SPANS:
        span = values[name]
        values[name] = None if span is None else (float(span[0]), float(span[1]))
    for name in _FLOATS:
        values[name] = None if values[name] is None else float(values[name])
    return CheckReport(**values, items=[_item(i) for i in record.get("items") or ()])


def _run(
    lib: Any, handle: int, args: tuple[Any, ...], poll: Callable[[], None]
) -> tuple[int, dict[str, Any] | None, str | None]:
    """iad_axis_self_test on a helper thread (its result read there: ``native._invoke_kept``); ``poll`` runs on the
    calling thread between waits. Ctrl+C or an exception of ``poll`` sets the axis's cancel latch (the core ends a
    motion wait with a decel stop), waits for the test to end (10 s at most) and is raised."""
    caller = threading.current_thread()
    box: list[Any] = []

    def run() -> None:  # holds ``args`` (and so the structs behind byref) until the call returns
        try:
            box.append(_n._invoke_kept(lib, "iad_axis_self_test", *args))
        except BaseException as exc:  # handed to the waiting thread
            box.append(exc)

    helper = _n._Helper(f"iad-call:{caller.name}", run)
    try:
        while not helper.ended:
            helper.wait(_SLICE_S)
            poll()
    except BaseException as stop:  # Ctrl+C, or what on_item raised
        try:
            _n._invoke(lib, "iad_axis_cancel_latch", handle)
        except Exception as exc:  # never hide the interrupt
            stop.add_note(f"iai_actuator_driver: cancelling the self-test failed: {exc}")
        else:
            if isinstance(stop, KeyboardInterrupt):
                stop.add_note("iai_actuator_driver: self-test cancelled on Ctrl+C (a decel stop was sent if it moved)")
        helper.wait(10.0)  # the core ends a wait right after the stop; a second Ctrl+C leaves at once
        if helper.ended:
            _n._invoke(lib, "iad_axis_cancel_reset", handle)
        raise
    outcome = box[0]
    if isinstance(outcome, BaseException):
        raise outcome
    return outcome  # type: ignore[no-any-return]


def run_check(
    ec: Elecylinder,
    *,
    motion: bool = False,
    port_label: str = "",
    step_mm: float = 5.0,
    speed_mm_s: float = 10.0,
    accel_g: float = 0.1,
    on_item: Callable[[CheckItem], None] | None = None,
) -> CheckReport:
    """Run the self-test against ``ec`` and return the report (``on_item`` sees each line as it completes)."""
    axis = _core_axis(ec)
    if axis is None:
        return _PY_RUN_CHECK(ec, motion=motion, port_label=port_label, step_mm=step_mm,  # type: ignore[no-any-return]
                               speed_mm_s=speed_mm_s, accel_g=accel_g, on_item=on_item)  # fmt: skip
    from .ec import _error, _sigint_is_python

    if getattr(axis.bus, "closed", False):  # the link line fails, the rest is skipped
        return _PY_RUN_CHECK(ec, motion=motion, port_label=port_label, step_mm=step_mm,  # type: ignore[no-any-return]
                               speed_mm_s=speed_mm_s, accel_g=accel_g, on_item=on_item)  # fmt: skip
    lib = axis._lib
    handle = axis._h()
    options = _n.CheckOptions()
    _n._check(lib, "iad_check_options_init", ctypes.byref(options))
    options.motion = int(bool(motion))
    options.step_mm, options.speed_mm_s, options.accel_g = float(step_mm), float(speed_mm_s), float(accel_g)
    out = _n.CheckSummary()
    args = (handle, ctypes.byref(options), _n._utf8(port_label or None), ctypes.byref(out))
    delivered = 0

    def deliver(items: list[Mapping[str, Any]]) -> None:
        nonlocal delivered
        for item in items:
            delivered += 1
            on_item(_item(item))  # type: ignore[misc]

    def poll() -> None:  # the lines finished since the last look
        if on_item is not None:
            deliver(_n._read_json(lib, "iad_axis_self_test_items_json", handle, delivered))

    caller = threading.current_thread()
    if on_item is None and (caller is not threading.main_thread() or not _sigint_is_python()):
        code, details, kept = _n._invoke_kept(lib, "iad_axis_self_test", *args)  # CC4 / CC7: no helper thread needed
    else:
        code, details, kept = _run(lib, handle, args, poll)
    if code != _n.IAD_OK:
        raise _error(code, details or {})
    # (0.7) the test's own report, read on the thread that ran it: a link check meanwhile replaces the handle's
    record = json.loads(kept) if kept is not None else _n._read_json(lib, "iad_axis_last_result_json", handle)
    if not isinstance(record, Mapping) or "items" not in record:
        raise RuntimeError("the self-test's report was replaced on the axis handle before it was read; run it again")
    if on_item is not None:
        deliver(list(record.get("items") or ())[delivered:])  # the lines finished after the last look
    return _report(record)
