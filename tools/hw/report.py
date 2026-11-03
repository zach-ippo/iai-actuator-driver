"""Step results of the harness: one console line per step, a JSON record and a markdown summary."""

from __future__ import annotations

import dataclasses
import json
import platform
import time
import traceback
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

PASS, CHECK, FAIL, SKIP, INTR = "PASS", "CHECK", "FAIL", "SKIP", "INTR"


class Skip(Exception):
    """The step does not apply to this unit or set-up (the message says why)."""


class Check(Exception):
    """The step finished with a result a person should look at; it is not a failure."""


class Abort(Exception):
    """A failure after which no further step may run (the end sequence still does)."""


def jsonable(obj: Any, depth: int = 0) -> Any:
    """``obj`` as plain JSON types (driver result objects, dataclasses, bytes ...)."""
    if depth > 10:
        return repr(obj)
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, dict):
        return {str(k): jsonable(v, depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset)):
        return [jsonable(v, depth + 1) for v in obj]
    to_dict = getattr(obj, "to_dict", None)
    if callable(to_dict):
        try:
            return jsonable(to_dict(), depth + 1)
        except Exception:  # noqa: BLE001
            pass
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        try:
            return jsonable(dataclasses.asdict(obj), depth + 1)
        except Exception:  # noqa: BLE001
            pass
    if isinstance(obj, bytes):
        return obj.hex(" ")
    return repr(obj)


def pos_str(st: Any) -> str:
    """One line of a status: position, servo, home, in-position, moving, alarm, PC mode, safety speed."""
    return (f"pos {st.position_mm:.2f} servo {int(st.servo_on)} homed {int(st.homed)} inpos {int(st.in_position)} "
            f"moving {int(st.moving)} alarm 0x{st.alarm_code:03X} pc {int(st.modbus_enabled)} "
            f"sfty {int(bool(getattr(st, 'safety_speed', False)))}")


class Recorder:
    """Runs the steps, prints one ``PASS`` / ``CHECK`` / ``FAIL`` / ``SKIP`` line for each and keeps the rows.

    A step is a function returning ``(verdict, detail)`` or ``(verdict, detail, data)``. It may raise :class:`Skip`,
    :class:`Check` or :class:`Abort`; ``refusals`` (exception types the driver raises when it refuses a command the unit
    does not offer) count as CHECK; any other exception is a FAIL. After a failed ``motion`` step ``stop_motion`` is
    called. A failed ``critical`` step stops every later step except those run with ``always``.
    """

    def __init__(self, say: Callable[..., None], *, refusals: tuple[type[BaseException], ...] = (),
                 stop_motion: Callable[[], Any] | None = None) -> None:
        self.say = say
        self.refusals = refusals
        self.stop_motion = stop_motion
        self.rows: list[dict[str, Any]] = []
        self.aborted: str | None = None
        self.n = 0

    def add(self, key: str, title: str, verdict: str, detail: str, seconds: float = 0.0, data: Any = None) -> None:
        row: dict[str, Any] = {"key": key, "step": title, "verdict": verdict, "detail": detail, "s": round(seconds, 3)}
        if data is not None:
            row["data"] = jsonable(data)
        self.rows.append(row)
        self.say(f"{verdict:<5} [{self.n:02d}] {key:<14} {title}: {detail}" + (f" ({seconds:.2f} s)" if seconds >= 0.01 else ""))

    def step(self, key: str, title: str, fn: Callable[[], Any], *, motion: bool = False, critical: bool = False,
             always: bool = False) -> Any:
        self.n += 1
        if self.aborted and not always:
            self.add(key, title, SKIP, f"stopped after: {self.aborted}")
            return None
        t0 = time.perf_counter()
        data: Any = None
        try:
            out = fn()
            verdict, detail = out[0], out[1]
            data = out[2] if len(out) > 2 else None
        except Skip as exc:
            verdict, detail = SKIP, str(exc)
        except Check as exc:
            verdict, detail = CHECK, str(exc)
        except Abort as exc:
            verdict, detail = FAIL, str(exc)
            self.aborted = title
        except self.refusals as exc:  # an empty tuple catches nothing
            verdict, detail = CHECK, f"the driver refused: {type(exc).__name__}: {exc}"
        except KeyboardInterrupt:
            self.add(key, title, INTR, "Ctrl+C", time.perf_counter() - t0)
            self.aborted = title
            raise
        except Exception as exc:  # noqa: BLE001
            verdict, detail = FAIL, f"{type(exc).__name__}: {exc}"
            data = {"traceback": traceback.format_exc()}
            if critical:
                self.aborted = title
            if motion and self.stop_motion is not None:
                try:
                    data["after_stop"] = jsonable(self.stop_motion())
                except Exception as exc2:  # noqa: BLE001
                    data["after_stop"] = f"stop failed: {exc2!r}"
        self.add(key, title, verdict, detail, time.perf_counter() - t0, data)
        return data

    def counts(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for row in self.rows:
            out[row["verdict"]] = out.get(row["verdict"], 0) + 1
        return out

    def verdict(self) -> str:
        counts = self.counts()
        if counts.get(FAIL) or counts.get(INTR):
            return FAIL
        if counts and set(counts) == {SKIP}:
            return SKIP  # nothing ran
        return CHECK if counts.get(CHECK) else PASS

    def summary_line(self) -> str:
        counts = self.counts()
        return f"{self.verdict()}: " + ", ".join(f"{counts[v]} {v}" for v in (PASS, CHECK, FAIL, SKIP, INTR) if counts.get(v))


def escape(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", " ")


def write_markdown(path: Path, title: str, meta: dict[str, Any], rows: Iterable[dict[str, Any]], verdict: str,
                   counts: dict[str, int], notes: Iterable[str] = ()) -> None:
    """The run's summary for a person: result, set-up, one table of steps."""
    lines = [f"# {title}", "",
             f"Result: **{verdict}** (" + ", ".join(f"{n} {v}" for v, n in counts.items()) + ")", "", "| | |", "|---|---|"]
    lines += [f"| {escape(str(k))} | {escape(str(v))} |" for k, v in meta.items() if v not in (None, "")]
    lines += ["", "## Steps", "", "| # | Key | Step | Result | Detail | Seconds |", "|---|---|---|---|---|---|"]
    for i, row in enumerate(rows, 1):
        lines.append(f"| {i} | {row['key']} | {escape(row['step'])} | {row['verdict']} | {escape(row['detail'])} | {row.get('s', '')} |")
    notes = list(notes)
    if notes:
        lines += ["", "## Notes", ""] + [f"- {n}" for n in notes]
    lines += ["", "PASS: as expected. CHECK: not a failure, a person should look. FAIL: not as expected. SKIP: not applicable "
              "or not selected.", ""]
    path.write_text("\n".join(lines), encoding="utf-8", newline="\n")


def host_info() -> dict[str, str]:
    return {"python": platform.python_version(), "platform": platform.platform(), "machine": platform.node()}


def write_json(path: Path, doc: dict[str, Any]) -> None:
    path.write_text(json.dumps(jsonable(doc), indent=1), encoding="utf-8", newline="\n")
