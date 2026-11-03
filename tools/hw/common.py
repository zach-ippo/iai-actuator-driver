"""Shared by the editions: where the run may move (the geometry of the test positions), the step table and the comparison of
two read-only backups."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Sequence

from .session import ACCEL_G

TOL_MM = 0.1  # a move counts as arrived when the unit is within this of the target (the move's own in-position band)
# the one start alarm taken as the unit's normal state: the message-level alarm 0x069 (the calendar clock stopped) that a
# unit with its calendar (RTC) on raises at power-up (VERIFIED on the PCON-CB, 2026-11-03: one alarm reset clears it).
# Any other start alarm is refused (start_alarm_problem).
NORMAL_START_ALARM = 0x069
LIVE_CLASSES = (3, 4)  # the backup's live monitor classes (position, status words, counters): they change by themselves and are left out of the comparison


@dataclass(frozen=True)
class StepDef:
    """One selectable step. ``changes``: the step moves the axis or changes its servo / mode state (so the run asks for a
    confirmation and the unit is prepared first). ``needs``: ``"band"`` (a band of at least 2 mm), ``"slow"`` (a band of at
    least 4 mm, for the steps that stop or change a move half way)."""

    key: str
    title: str
    default: bool = True
    changes: bool = False
    needs: str = ""
    motion: bool = False
    critical: bool = False
    why: str = ""  # what it checks, one sentence (README and --list-steps)


class Geometry:
    """The test positions, all inside the band the run may move in (``lo`` .. ``hi`` mm).

    ``a`` and ``b`` are the two ends of the test travel around the middle ``c``; ``v`` is the test speed, ``v_slow`` the speed
    of the moves that are stopped half way, ``acc`` the acceleration. Everything is limited by the speed cap and by the
    acceleration cap, and the travel by the size of the band.
    """

    def __init__(self, lo: float, hi: float, speed_cap: float, accel_cap: float = ACCEL_G) -> None:
        self.lo, self.hi = float(lo), float(hi)
        span = self.hi - self.lo
        self.span = span
        self.c = (self.lo + self.hi) / 2.0
        self.d = min(0.5 * span, 20.0)  # test travel
        self.a = self.c - self.d / 2.0
        self.b = self.c + self.d / 2.0
        self.cap = float(speed_cap)
        self.v = min(10.0, self.cap)
        self.v_slow = min(3.0, self.cap / 2.0)
        self.acc = min(ACCEL_G, accel_cap)
        self.dec = max(self.acc / 2.0, 0.01)
        self.jog = min(0.5, self.d / 4.0)

    @property
    def usable(self) -> bool:
        return self.d >= 1.0

    @property
    def slow_usable(self) -> bool:
        return self.d >= 2.0

    def timeout(self, distance: float, speed: float) -> float:
        """Generous time for a move: twice the travel time plus 10 s."""
        return 2.0 * abs(distance) / max(speed, 0.1) + 10.0

    def inside(self, position: float) -> bool:
        return self.lo - 0.05 <= position <= self.hi + 0.05

    def describe(self) -> str:
        return (f"band {self.lo:.2f} .. {self.hi:.2f} mm, test travel {self.a:.2f} .. {self.b:.2f} mm, test speed {self.v:g} "
                f"mm/s (slow moves {self.v_slow:g}), acceleration {self.acc:g} G")


def band_from_stroke(spec: tuple[str, float, float], stroke: tuple[float, float] | None) -> tuple[float, float] | None:
    """The band for ``--stroke-band`` and the unit's stroke: ``("pct", 60, 60)`` is the middle 60 % of it, ``("mm", lo, hi)``
    that range (cut to the stroke when it is known). None when the stroke is not known and no range in mm was given."""
    kind, lo, hi = spec
    if kind == "mm":
        if stroke is not None:
            lo, hi = max(lo, stroke[0]), min(hi, stroke[1])
        return (lo, hi) if lo < hi else None
    if stroke is None:
        return None
    s_lo, s_hi = stroke
    margin = (s_hi - s_lo) * (1.0 - lo / 100.0) / 2.0
    return (round(s_lo + margin, 3), round(s_hi - margin, 3))


def near(a: float, b: float, tol: float = TOL_MM) -> bool:
    return abs(a - b) <= tol


def servo_across_toggle(started_on: bool, seen: Sequence[tuple[str, bool | None, bool | None]]) -> tuple[str | None, bool]:
    """Where the servo went off across the PC / PIO toggle. ``seen`` holds (the call, servo on, paused) read after each call,
    the switch to PIO mode first; an unknown servo state (None) is not a loss. Returns (the first call after which the servo
    was off, or None; whether that is what a PCON with nothing on its PIO connector does: a pause shows in PIO mode, so the
    first status has it, and its servo goes off, which the first status may not show yet: on the PCON-CB a command line's
    ``mode pio`` reply had the pause with the servo still on, and the servo was off by the ``mode pc`` reply, 2026-11-02).
    So that pattern needs the loss by the first PC-mode status (the second entry) at the latest. The servo is only switched
    on again for that pattern, and the step passes with a note; any other loss, a later one too (after the Modbus control
    off / on, say), is a failure of the step."""
    if not started_on:
        return None, False
    paused_in_pio = bool(seen and seen[0][2])
    for index, (call, servo_on, _paused) in enumerate(seen):
        if servo_on is False:
            return call, paused_in_pio and index <= 1
    return None, False


# ---------------------------------------------------------------------------------------------------- backup comparison
def _is_live(class_id: Any) -> bool:
    """A class id as an int (``3``) or as the backup's text (``"0x0003"``)."""
    if isinstance(class_id, str):
        try:
            class_id = int(class_id, 0)
        except ValueError:
            return False
    return class_id in LIVE_CLASSES


def _parameter_items(backup: dict[str, Any]) -> dict[tuple[Any, Any, Any], Any]:
    out: dict[tuple[Any, Any, Any], Any] = {}
    for cls in (backup.get("parameters") or {}).get("classes") or []:
        class_id = cls.get("class_id", cls.get("class"))
        for item in cls.get("items") or []:
            out[(class_id, item.get("id"), item.get("entry"))] = item.get("hex")
    return out


def _rows(backup: dict[str, Any]) -> dict[Any, Any]:
    return {r.get("n"): r.get("words") for r in (backup.get("positions") or {}).get("rows") or []}


def compare_backups(start: dict[str, Any], end: dict[str, Any]) -> dict[str, Any]:
    """What changed between two read-only backups of the same unit: parameter items outside the live monitor classes (3 and 4),
    position rows, alarm history. Empty lists and ``True`` flags mean nothing changed. ``parameter_items`` counts the items
    that were compared (those outside the live classes); ``live_items_left_out`` counts the live items that were not, and
    ``live_items_changed`` the live items that differ (information only: they change by themselves)."""
    a, b = _parameter_items(start), _parameter_items(end)
    differs = [k for k in sorted(set(a) | set(b), key=str) if a.get(k) != b.get(k)]
    changed = {str(k): (a.get(k), b.get(k)) for k in differs if not _is_live(k[0])}
    ra, rb = _rows(start), _rows(end)
    rows_changed = sorted(n for n in set(ra) | set(rb) if ra.get(n) != rb.get(n))
    alarms_same = json.dumps(start.get("alarms"), sort_keys=True, default=repr) == json.dumps(end.get("alarms"), sort_keys=True, default=repr)
    compared = sum(1 for k in b if not _is_live(k[0]))
    return {"parameter_items": compared, "live_items_left_out": len(b) - compared, "parameters_changed": changed,
            "live_items_changed": sum(1 for k in differs if _is_live(k[0])), "rows_checked": len(rb), "rows_changed": rows_changed, "alarm_history_same": alarms_same}


def start_alarm_problem(fault: bool, code: int) -> str | None:
    """The alarm rule of the start state: ``None`` when the run may go on, else why the unit is not touched. No alarm
    code, or the message-level alarm 0x069 alone (:data:`NORMAL_START_ALARM`, the unit's normal state after a power-up:
    it is recorded and the end is compared against it), is fine. A fault, or any other alarm code, also of the message
    level, is refused."""
    if fault:
        return f"alarm 0x{code:03X}"
    if code and code != NORMAL_START_ALARM:
        return (f"alarm 0x{code:03X}, a message-level alarm other than 0x{NORMAL_START_ALARM:03X} (the one start alarm "
                "taken as normal)")
    return None


def alarm_at_end(start_code: int, end_code: int, reset_by_recover: Any, message_level: bool,
                 name: str | None = None) -> tuple[bool, str | None]:
    """The alarm of the end check: ``(as found, note)``. The same code as at the start is as found. The message-level
    alarm 0x069 present at the start (the unit's normal state after a power-up) and gone at the end is as found too:
    one alarm reset clears it, and steps such as recover() may send one. The note names it (the step is CHECK).
    Anything else, a new alarm, another code or a fault-level one, is not as found (FAIL)."""
    if end_code == start_code:
        return True, None
    if start_code == NORMAL_START_ALARM and not end_code and message_level:
        what = f"alarm 0x{start_code:03X}" + (f" ({name})" if name else "")
        how = "was reset by recover()" if start_code in set(reset_by_recover or ()) else "was cleared during the run"
        return True, (f"{what}, the unit's normal message-level state at the start, {how}: the unit no longer "
                      "reports it")
    return False, None
