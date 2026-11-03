"""The plain-text test report of the workbench (``POST /api/wb/report``).

Built from what the workbench already knows (the cached status, the event log, the last self-test, the last
coordinated plan and run): no bus traffic. Plain text with two-space indents, ready to paste into a GitHub
issue. The page adds what only the browser knows (user agent, screen, its own errors); those strings are cut to
300 characters, and at most 30 page errors are kept. Every field that comes from outside this module (the port
listing, the unit, the browser) goes through :func:`_clean`: a line break in a USB device's own description, for
example, must not start a forged line of the report.
"""

from __future__ import annotations

import datetime as _dt
import platform
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]  # the repository root of a source checkout
_DRIVER = ROOT / "bindings" / "python"  # its Python package, used before an installed one
if (_DRIVER / "iai_actuator_driver").is_dir() and str(_DRIVER) not in sys.path:
    sys.path.insert(0, str(_DRIVER))

from iai_actuator_driver import __version__  # noqa: E402 (needs the sys.path entry above)

try:
    from .resources import run_as
    from .workbench import SIM_SETUPS
except ImportError:  # run as a script from apps/gui/
    from resources import run_as  # type: ignore[no-redef]
    from workbench import SIM_SETUPS  # type: ignore[no-redef]

__all__ = ["build_report"]

PRODUCT = "IAI Actuator Driver"
CLIENT_TEXT_MAX = 300
CLIENT_ERRORS_MAX = 30
ERRORS_MAX = 30
EVENTS_MAX = 100
CHECK_ORDER = ("PASS", "WARN", "FAIL", "SKIP")


def _clean(value: Any, limit: int = CLIENT_TEXT_MAX) -> str:
    """One line of printable text, at most ``limit`` characters."""
    if value is None:
        return ""
    text = value if isinstance(value, str) else str(value)
    text = "".join(ch if ch.isprintable() else " " for ch in text)
    return " ".join(text.split())[:limit]


def _hms(t: Any) -> str:
    try:
        value = float(t)
    except (TypeError, ValueError):
        return "--:--:--"
    if value > 1e11:  # milliseconds (Date.now())
        value /= 1000.0
    try:
        return time.strftime("%H:%M:%S", time.localtime(value))
    except (OverflowError, OSError, ValueError):
        return "--:--:--"


def _os_text() -> str:
    system, machine = platform.system(), platform.machine()
    if system == "Windows":
        return f"Windows {platform.release()} ({platform.version()}, {machine})"
    if system == "Darwin":
        return f"macOS {platform.mac_ver()[0]} ({machine})"
    return f"{system} {platform.release()} ({machine})"


def _pyserial() -> str:
    try:
        import serial  # no port is opened: only the version is read
    except ImportError:
        return "pyserial not installed"
    version = getattr(serial, "__version__", None) or getattr(serial, "VERSION", None) or "unknown version"
    return f"pyserial {version}"


def _client_errors(raw: Any) -> list[tuple[Any, str]]:
    if not isinstance(raw, list):
        return []
    out = []
    for item in raw[:200]:
        if isinstance(item, dict):
            message = _clean(item.get("message"))
            t = item.get("t")
        else:
            message, t = _clean(item), None
        if message:
            out.append((t if isinstance(t, (int, float)) and not isinstance(t, bool) else None, message))
    out.sort(key=lambda e: -(e[0] or 0))
    return out[:CLIENT_ERRORS_MAX]


def _event_line(e: dict[str, Any]) -> str:
    return f"  {_hms(e.get('t'))} {_clean(e.get('level', '')):<6} {_clean(e.get('message'), 2000)}"


def _port_line(p: Any, axes: list[str]) -> str:
    baud = p.port.baudrate
    if p.simulated:
        label = _clean(SIM_SETUPS.get(p.setup, (p.setup or "simulator",))[0])
        what = f"simulator: {label}, {baud or 'no'} baud"
    else:
        kind = "IAI USB adapter" if p.is_iai else (_clean(p.description) or "serial port")
        line = _clean(getattr(getattr(p.port, "line", None), "kind", None)) or "unknown"
        what = f"{kind}, {line} line, {baud or 'unknown'} baud ({'detected' if p.baud_detected else 'set'})"
    return f"  {_clean(p.name)}  {what}, axes {', '.join(_clean(a) for a in axes) if axes else 'none'}"


def _axis_lines(a: Any) -> list[str]:
    info = a.info or {}
    head = f"  {_clean(a.name)}  "
    pad = " " * len(head)
    controller = (f"{_clean(info['system_name'])} / {_clean(info['unit_name'])}"
                  if info.get("system_name") and info.get("unit_name") else _clean(info.get("controller")))  # fmt: skip
    ident = [_clean(v) for v in (info.get("model"), info.get("kind"), controller) if v]
    if info.get("firmware"):
        ident.append(f"firmware {_clean(info['firmware'])}")
    if info.get("serial_number"):
        ident.append(f"serial {_clean(info['serial_number'])}")
    lines = [head + (", ".join(ident) if ident else "identity not read")]
    st = a.status
    units = (a.limits or {}).get("units", "mm")
    if st:
        now = ["servo on" if st.get("servo_on") else "servo off", "homed" if st.get("homed") else "not homed"]
        if st.get("position_mm") is not None:
            now.append(f"{st['position_mm']:.2f} {units}")
        if st.get("moving"):
            now.append("moving")
        if st.get("alarm") or st.get("alarm_code"):
            now.append(f"alarm 0x{st.get('alarm_code') or 0:03X} {_clean(st.get('alarm_name'))}".rstrip())
        else:
            now.append("no alarm")
    else:
        now = ["no status read"]
    now.append(f"fast mode {'on' if a.ec.fast_mode else 'off'}")
    now.append(f"link {_clean(a.ec.link_state) or 'unknown'}")
    if a.busy:
        now.append(f"busy: {_clean(a.busy)}")
    lines.append(pad + "now: " + ", ".join(now))
    if a.fault:
        lines.append(pad + f"fault: {_clean(a.fault.get('text'), 2000)} ({_clean(a.fault.get('message'), 2000)})")
    if a.error:
        lines.append(pad + f"last error: {_clean(a.error, 2000)}")
    nv, nv_hour = a.nv_counts()
    if nv is not None:
        lines.append(pad + f"non-volatile writes in this session: {nv} ({nv_hour} in the last hour)")
    check = a.check
    if check is None:
        lines.append(pad + "self-test: not run (Unit info & alarms > Self-test)")
    else:
        counts = check.get("counts") or {}
        tally = ", ".join(f"{counts.get(k, 0)} {k}" for k in CHECK_ORDER)
        lines.append(pad + f"self-test: {_clean(check.get('result'))} at {_hms(check.get('t'))} ({tally})")
        for item in check.get("items") or []:
            if item.get("result") in ("FAIL", "WARN"):
                lines.append(pad + f"  {item['result']} {_clean(item.get('name'))}: {_clean(item.get('detail'), 2000)}")
    return lines


def _coord_lines(coord: Any) -> list[str]:
    lines = []
    plan = coord.last_plan
    if plan:
        if plan.get("passed"):
            lines.append(f"  last plan {_clean(plan['plan_id'])}: {_clean(plan['text'], 2000)}, passed "
                         f"({_clean(plan.get('mode'))}, {plan.get('steps')} steps, "
                         f"{plan.get('planned_s') or 0:.2f} s)")  # fmt: skip
        else:
            tail = "" if plan.get("max_speed") is None else f"; largest feasible speed about {plan['max_speed']:g}"
            reasons = "; ".join(_clean(r, 2000) for r in plan.get("reasons") or [])
            lines.append(f"  last plan: {_clean(plan['text'], 2000)}, refused: {reasons}{tail}")
    run = coord.last_run
    if run:
        r = run.get("result") or {}
        state = _clean(run.get("state"))
        units = _clean(run.get("units")) or "mm"
        if state == "completed" and r:
            lines.append(f"  last run {_clean(run['run_id'])}: completed in {r.get('duration_s') or 0:.2f} s (planned "
                         f"{r.get('planned_s') or 0:.2f} s); deviation max {r.get('max_deviation') or 0:.3f} / mean "
                         f"{r.get('mean_deviation') or 0:.3f} {units}; {r.get('frames')} frames, "
                         f"{r.get('late_frames')} late")  # fmt: skip
        elif state == "stopped" and r:
            lines.append(f"  last run {_clean(run['run_id'])}: stopped after {r.get('duration_s') or 0:.2f} s: "
                         f"{_clean(run.get('error'), 2000)}")  # fmt: skip
        else:
            lines.append(f"  last run {_clean(run['run_id'])}: {state}: {_clean(run.get('error'), 2000)}")
    if coord.active():
        lines.append(f"  now: {_clean(coord.summary()['state'])}")
    return lines or ["  none"]


def build_report(wb: Any, client: dict[str, Any] | None, *, sim_only: bool) -> tuple[str, str]:
    """The test report as plain text and its file name (``iai-gui-report-YYYYMMDD-HHMMSS.txt``). Read-only."""
    client = client if isinstance(client, dict) else {}
    now = _dt.datetime.now().astimezone()
    with wb._lock:
        ports = list(wb.ports.values())
        axes = list(wb.axes.values())
        events = list(wb._events)
    lines = [
        f"{PRODUCT} test report",
        f"Created   {now:%Y-%m-%d %H:%M:%S %z}",
        f"Version   {PRODUCT} {__version__} ({run_as()})",
        "Mode      " + ("demo (--sim-only): simulators only" if sim_only else "normal: serial ports and simulators"),
        f"OS        {_os_text()}",
        f"Python    {platform.python_version()}, {_pyserial()}",
        f"Browser   {_clean(client.get('browser')) or 'not given'}",
        f"Screen    {_clean(client.get('screen')) or 'not given'}",
    ]
    page = _clean(client.get("page"))
    if page:
        lines.append(f"Page      {page}")
    lines += ["", f"Ports ({len(ports)})"]
    lines += [_port_line(p, [a.name for a in axes if a.port == p.name]) for p in ports] or ["  none"]
    lines += ["", f"Axes ({len(axes)})"]
    for a in axes:
        lines += _axis_lines(a)
    if not axes:
        lines.append("  none")
    lines += ["", "Coordinated (experimental)", *_coord_lines(wb.coord)]
    problems = [e for e in reversed(events) if e.get("level") in ("error", "warn")][:ERRORS_MAX]
    lines += ["", f"Errors and warnings (newest first, up to {ERRORS_MAX})"]
    lines += [_event_line(e) for e in problems] or ["  none"]
    page_errors = _client_errors(client.get("client_errors"))
    lines += ["", f"Page errors (newest first, up to {CLIENT_ERRORS_MAX})"]
    lines += [f"  {_hms(t) if t is not None else '--:--:--'} {m}" for t, m in page_errors] or ["  none"]
    lines += ["", f"Event log (oldest first, last {EVENTS_MAX})"]
    lines += [_event_line(e) for e in events[-EVENTS_MAX:]] or ["  none"]
    return "\n".join(lines) + "\n", f"iai-gui-report-{now:%Y%m%d-%H%M%S}.txt"
