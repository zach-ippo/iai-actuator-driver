"""The end of every run: summary.json and summary.md in the results folder, and the closing lines on the console."""

from __future__ import annotations

import datetime as dt
from types import SimpleNamespace
from typing import Any

from . import report


def finish(ctx: SimpleNamespace, ed: Any, code: int) -> None:
    """Write the run's record. ``ed`` is the edition (or subcommand) object: ``name``, ``rec`` (a Recorder), and optionally
    ``start``, ``band``, ``geo``, ``end_info``, ``compare``, ``notes`` and ``extra``."""
    rec = ed.rec
    verdict, counts = rec.verdict(), rec.counts()
    ended = dt.datetime.now()
    unit = getattr(ctx, "unit", None) or {}
    geo = getattr(ed, "geo", None)
    args = {k: v for k, v in vars(ctx.args).items()}
    meta = {
        "Run": f"{ctx.subcommand}, {ed.name} edition" + (" (simulator)" if ctx.args.sim else f" on {ctx.args.port}"),
        "Started": ctx.started.strftime("%Y-%m-%d %H:%M:%S"),
        "Ended": ended.strftime("%Y-%m-%d %H:%M:%S"),
        "Unit": " | ".join(str(unit[k]) for k in ("model", "serial_number", "firmware") if unit.get(k)),
        "Driver": " | ".join(f"{k} {v}" for k, v in (getattr(ctx, "driver", None) or {}).items() if v not in (None, "")),
        "Speed cap": f"{ctx.args.speed_cap:g} mm/s",
        "Test band": geo.describe() if geo else (f"{ed.band[0]:.2f} .. {ed.band[1]:.2f} mm" if getattr(ed, "band", None) else "none"),
        "Exit code": code,
        "Host": f"{report.host_info()['platform']} (Python {report.host_info()['python']})",
        "Results": str(ctx.out),
    }
    doc = {"harness": "tools/hw", "run": ctx.subcommand, "edition": ed.name, "command": ctx.command, "started": meta["Started"],
           "ended": meta["Ended"], "exit_code": code, "verdict": verdict, "counts": counts, "args": args, "driver": getattr(ctx, "driver", None),
           "unit": unit, "start_state": getattr(ed, "start", None), "band": getattr(ed, "band", None),
           "end_state": getattr(ed, "end_info", None), "backup_compare": getattr(ed, "compare", None),
           "notes": getattr(ed, "notes", []), "host": report.host_info(),
           "driver_messages": list(ctx.log_bridge.messages) if getattr(ctx, "log_bridge", None) else [], "steps": rec.rows, **(getattr(ed, "extra", None) or {})}
    report.write_json(ctx.out / "summary.json", doc)
    notes = list(getattr(ed, "notes", []))
    notes += [f"driver {m}" for m in (ctx.log_bridge.messages if getattr(ctx, "log_bridge", None) else [])]
    if code == 2:
        notes.append("the run was refused or could not start: see the lines above; the unit was not changed")
    report.write_markdown(ctx.out / "summary.md", f"Hardware acceptance: {ctx.subcommand}", meta, rec.rows, verdict, counts, notes)
    say = ctx.console.say
    say("=" * 72)
    say(f"{ctx.subcommand} ({ed.name}): {rec.summary_line()}")
    if unit.get("model"):
        say(f"unit    : {meta['Unit']}")
    say(f"results : {ctx.out}  (summary.md, summary.json, run.log" + (", backups" if getattr(ed, "backup_start", None) else "") + ")")
    say(f"EXIT {code}")
