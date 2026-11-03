"""Local web test GUI for the unofficial IAI ELECYLINDER (EC) PC driver.

After ``pip install .`` run ``iai-gui``; from a source checkout (the folder that holds ``gui`` and ``iai_actuator_driver``)
``python apps/gui/server.py`` does the same::

    iai-gui                                # http://127.0.0.1:8780 : the multi-axis workbench (/classic: one axis)
    iai-gui --sim-only                     # demo mode: simulators only, no serial port is listed or opened
    iai-gui --port 8790                    # another port
    iai-gui --stroke 0,50                  # optional default stroke override (mm); it can only narrow
    iai-gui --no-browser                   # do not open the browser at start

By default the stroke is each EC's own soft limits (0x0700 minus the 0.30 mm margin), read at
connect - e.g. 0..150 mm on the EC-DS4H-150 slider, 0..9.50 mm on the EC-GRBP8M-20 gripper. A stroke
given here or in the connect form is an override that can only narrow it.

The workbench (``/``, :mod:`gui.workbench`) connects one or many ports (one axis per port, or several axes
on one port by axis number), scans each for axes and offers every driver function with the unit's own
ranges. The classic single-axis page is at ``/classic``.

"Simulate" uses iai_actuator_driver's SimulatedEC and never opens a serial port. For the real EC, close /
disconnect IA-OS first (only one program can own the port); the baud rate is auto-detected.

Binds to 127.0.0.1 only. Every /api request must carry the per-run token and a loopback Host header, and a browser
request must come from this server's own page (Origin, Sec-Fetch-Site), so other web pages (or DNS rebinding) cannot
drive the actuator. The token is never in a served page: the launch address carries it after a ``#`` (a fragment is
not sent to the server), ``/token.js`` keeps it in the tab's sessionStorage, and another local account that fetches
``/`` learns nothing.

The pages, scripts and documents are found through :mod:`gui.resources` (a source checkout, an installed package or
the iai-gui.exe bundle); a file that is not there answers 404.
"""

from __future__ import annotations

import argparse
import html
import json
import os
import platform
import queue
import secrets
import shutil
import socket
import sys
import tempfile
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlsplit

try:
    from . import resources
    from .backend import DEFAULT_STROKE, BackendError
    from .controller import Controller, check_stroke, parse_stroke, require_number
    from .report import build_report
    from .workbench import Workbench, catalog
except ImportError:  # run as a script from apps/gui/
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import resources  # type: ignore[no-redef]
    from backend import DEFAULT_STROKE, BackendError  # type: ignore[no-redef]
    from controller import Controller, check_stroke, parse_stroke, require_number  # type: ignore[no-redef]
    from report import build_report  # type: ignore[no-redef]
    from workbench import Workbench, catalog  # type: ignore[no-redef]

from iai_actuator_driver import __version__  # noqa: E402 (the imports above put the driver of a source checkout on sys.path)

PRODUCT = "IAI Actuator Driver"
HOST = "127.0.0.1"
DEFAULT_PORT = 8780
MAX_BODY_BYTES = 16384
SOCKET_TIMEOUT_S = 15.0  # an idle connection frees its handler thread after this long (reads and writes, not processing)
TOKEN_HEADER = "X-EC-Token"
STATIC_FILES: dict[str, tuple[str, str]] = {
    "/": ("workbench.html", "text/html; charset=utf-8"),
    "/workbench.html": ("workbench.html", "text/html; charset=utf-8"),
    "/workbench.js": ("workbench.js", "text/javascript; charset=utf-8"),
    "/workbench.css": ("workbench.css", "text/css; charset=utf-8"),
    "/classic": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/unitinfo.js": ("unitinfo.js", "text/javascript; charset=utf-8"),
    "/params.js": ("params.js", "text/javascript; charset=utf-8"),
    "/about.js": ("about.js", "text/javascript; charset=utf-8"),
    "/coord.js": ("coord.js", "text/javascript; charset=utf-8"),
    "/coord.css": ("coord.css", "text/css; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
    "/token.js": ("token.js", "text/javascript; charset=utf-8"),
}
# Read-only documents linked from the About panel (plain text, from a source checkout; absent in a bare install).
ABOUT_DOCS: tuple[tuple[str, str, str], ...] = (
    ("Getting started", "/docs/GETTING_STARTED.md", "docs/GETTING_STARTED.md"),
    ("Coordinated motion", "/docs/COORDINATED.md", "docs/COORDINATED.md"),
    ("Compatibility", "/docs/COMPATIBILITY.md", "docs/COMPATIBILITY.md"),
    ("Function reference", "/docs/DRIVER.md", "docs/DRIVER.md"),
    ("Reference and evidence", "/docs/REFERENCE.md", "docs/REFERENCE.md"),
    ("README", "/README.md", "README.md"),
    ("Changelog", "/CHANGELOG.md", "CHANGELOG.md"),
)
DOC_FILES: dict[str, str] = {url: rel for _title, url, rel in ABOUT_DOCS}  # resolved by resources.doc_file
ABOUT_NOTICE = (
    "UNOFFICIAL. This is not an IAI product and is not supported, documented or endorsed by IAI. It was worked "
    "out from public IAI manuals, observed serial traffic and hardware tests. It drives every IAI unit that speaks "
    "Modbus RTU on its serial port the same way; the units it has been run on are listed under Compatibility.",
    "Not a safety function. STOP is a Modbus decelerate-stop; keep a hardware emergency stop and guarding. A unit has "
    "no communication watchdog: if the PC or the cable fails, a move in progress completes.",
)
ABOUT_EVIDENCE = (
    ("VERIFIED", "tried on real hardware with this driver"),
    ("ON-WIRE", "seen in observed serial traffic"),
    ("DOC", "written in a public IAI manual, not yet confirmed on the unit"),
    ("UNVERIFIED", "expected, not yet tried; describes a feature or path, never a unit: "
     "every unit is driven the same way"),
)


def about_info(sim_only: bool) -> dict[str, Any]:
    """What the About panel shows: product, version, the unofficial notice, the evidence words and the document links."""
    return {
        "product": PRODUCT, "version": __version__, "stage": "stable", "python": platform.python_version(),
        "sim_only": sim_only, "notice": list(ABOUT_NOTICE), "evidence": [list(e) for e in ABOUT_EVIDENCE],
        "docs": [{"title": title, "href": url, "available": resources.doc_file(rel).is_file()}
                 for title, url, rel in ABOUT_DOCS],
        "run_as": resources.run_as(),
    }
SECURITY_HEADERS = {
    "Cache-Control": "no-store",
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "Content-Security-Policy": ("default-src 'self'; base-uri 'none'; form-action 'none'; object-src 'none'; "
                                "frame-ancestors 'none'"),
    "Referrer-Policy": "no-referrer",
    "Cross-Origin-Resource-Policy": "same-origin",  # another site cannot load /workbench.js to find out that the GUI runs
    "Cross-Origin-Opener-Policy": "same-origin",  # a site that opened the GUI in a window loses its handle to it
}
SERVER_NAME = "iai-gui"  # the Server header: no version numbers
CLASSIC_NAME = "single-axis page"  # how a STOP result names the classic page's axis
CTRL_CLOSE_EVENT, CTRL_LOGOFF_EVENT, CTRL_SHUTDOWN_EVENT = 2, 5, 6  # Windows console control events
CONSOLE_LINE_MAX = 300
CONSOLE_QUEUE_MAX = 200
TOKEN_REFUSED = ("Missing or wrong access key. Open the GUI from the address printed in the console window "
                 "(it ends with #k=...), or start it again.")
ORIGIN_REFUSED = "The request does not come from this GUI's own page."
CLOSING_REFUSED = "The GUI is shutting down: only STOP is accepted now."
CLOSING_OK = frozenset({"/api/stop", "/api/wb/stop", "/api/wb/coord/stop", "/api/quit"})  # still answered while it ends

Body = dict[str, Any]
Query = dict[str, str]


def printable(text: str, limit: int = CONSOLE_LINE_MAX) -> str:
    """One line that is safe to write to a console: control characters (a request line can hold any byte) become
    backslash escapes such as \\x1b, and the line is cut to ``limit`` characters."""
    out = "".join(ch if ch.isprintable() else ("\\x%02x" % ord(ch) if ord(ch) < 256 else "\\u%04x" % ord(ch))
                  for ch in text)
    return out if len(out) <= limit else out[: limit - 3] + "..."


class ConsoleLog:
    """Console lines written by one thread. In the Windows console window a click selects text (QuickEdit) and every
    write blocks until the selection ends: a request thread that wrote its own error line would hold its connection
    open for that long (and with it the browser's six connections, STOP ALL among them). A request thread only puts
    the line on a bounded queue; a line that does not fit is dropped and counted. Only this writer thread can block."""

    def __init__(self, write: Callable[[str], None] | None = None, limit: int = CONSOLE_QUEUE_MAX) -> None:
        self._write = write if write is not None else self._to_stderr
        self._queue: queue.Queue[str] = queue.Queue(maxsize=limit)
        self._start = threading.Lock()
        self._thread: threading.Thread | None = None
        self.dropped = 0

    @staticmethod
    def _to_stderr(line: str) -> None:
        sys.stderr.write(line + "\n")
        sys.stderr.flush()

    def put(self, text: str) -> None:
        """Queue one line (never blocks); the writer thread starts with the first line."""
        if self._thread is None:
            with self._start:
                if self._thread is None:
                    self._thread = threading.Thread(target=self._run, name="gui-console", daemon=True)
                    self._thread.start()
        try:
            self._queue.put_nowait(printable(text))
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while True:
            line = self._queue.get()
            try:
                if self.dropped:
                    count, self.dropped = self.dropped, 0
                    self._write(f"[gui] {count} log line(s) dropped while the console was blocked")
                self._write(line)
            except Exception:  # noqa: BLE001 - a console that fails must not end the writer
                pass


def console_handler(shutdown: Callable[[], None]) -> Callable[[int], bool]:
    """The Windows console control handler: closing the console window (or logging off or shutting down) runs the
    safe shutdown first. Ctrl+C and Ctrl+Break are left to Python (False)."""

    def handle(event: int) -> bool:
        if event not in (CTRL_CLOSE_EVENT, CTRL_LOGOFF_EVENT, CTRL_SHUTDOWN_EVENT):
            return False
        try:
            shutdown()
        except Exception:  # noqa: BLE001 - the process ends right after this returns
            pass
        return True

    return handle


_console_callback: Any = None  # the registered callback must stay alive as long as the process


def install_console_handler(shutdown: Callable[[], None]) -> bool:
    """Windows only. Closing the console window ends the process without running ``finally`` blocks (Python maps only
    Ctrl+C and Ctrl+Break to signals), so a moving axis would run on and PIO would stay locked out. Registers
    :func:`console_handler`; Windows allows about 5 s after the close event, and the shutdown sends STOP to every axis
    first. Returns whether a handler is installed."""
    global _console_callback
    if os.name != "nt":
        return False
    try:
        import ctypes
        from ctypes import wintypes

        callback = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)(console_handler(shutdown))
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.SetConsoleCtrlHandler.argtypes = [ctypes.c_void_p, wintypes.BOOL]
        kernel32.SetConsoleCtrlHandler.restype = wintypes.BOOL
        if not kernel32.SetConsoleCtrlHandler(callback, True):
            return False
    except Exception:  # noqa: BLE001 - no console (pythonw, a service): Ctrl+C and Quit still work
        return False
    _console_callback = callback
    return True


def _flag(body: Body, key: str, default: bool | None = None) -> bool:
    value = body.get(key, default)
    if not isinstance(value, bool):
        raise BackendError(f"{key} must be true or false.", 400)
    return value


def _integer(body: Body, key: str, default: int) -> int:
    number = require_number(body.get(key, default), key)
    if number != int(number):
        raise BackendError(f"{key} must be a whole number.", 400)
    return int(number)


def _required(body: Body, key: str) -> float:
    if key not in body:
        raise BackendError(f"Missing field: {key}.", 400)
    return require_number(body[key], key)


def _direction(body: Body) -> int:
    """Push direction: +1 away from home, -1 toward home."""
    value = body.get("direction")
    if value not in (1, -1) or isinstance(value, bool):
        raise BackendError("direction must be 1 (away from home) or -1 (toward home).", 400)
    return int(value)


def _stroke(body: Body) -> tuple[float, float] | None:
    """Optional stroke override from the connect form; None (server default) when neither field is given."""
    if body.get("stroke_min_mm") is None and body.get("stroke_max_mm") is None:
        return None
    return check_stroke(_required(body, "stroke_min_mm"), _required(body, "stroke_max_mm"))


def _whole(body: Body, key: str) -> int:
    """A required whole number (a JSON integer, not a boolean)."""
    if key not in body:
        raise BackendError(f"Missing field: {key}.", 400)
    value = body[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise BackendError(f"{key} must be a whole number.", 400)
    return value


def _param_value(body: Body) -> Any:
    if "value" not in body:
        raise BackendError("Missing field: value.", 400)
    return body["value"]


def _method(body: Body) -> str:
    value = body.get("method", "direct")
    if not isinstance(value, str):
        raise BackendError("method must be 'direct' or 'break'.", 400)
    return value


def _control_mode(body: Body) -> bool:
    """``mode`` = "pc" (Modbus has control) or "pio"; returns True for PC."""
    value = body.get("mode")
    if value not in ("pc", "pio"):
        raise BackendError("mode must be 'pc' or 'pio'.", 400)
    return value == "pc"


def _sim_profile(body: Body) -> str:
    value = body.get("sim_profile", "slider")
    if not isinstance(value, str):
        raise BackendError("sim_profile must be a profile name.", 400)
    return value


def _query_int(query: Query, key: str, default: int | None, *, low: int = 0, high: int = 0xFFFF) -> int | None:
    """A whole number from the query string: decimal, or hex with a 0x prefix (e.g. class=0x0700)."""
    text = query.get(key, "").strip()
    if not text:
        return default
    try:
        value = int(text, 16) if text.lower().startswith("0x") else int(text, 10)
    except ValueError:
        raise BackendError(f"{key} must be a whole number (decimal, or hex with 0x).", 400) from None
    if not low <= value <= high:
        raise BackendError(f"{key} must be {low}-{high}.", 400)
    return value


def _read_block(ctl: Controller, query: Query) -> dict[str, Any]:
    """GET /api/read_block?class=0x0700[&offset=N][&count=N]: raw words of one directory block (read-only)."""
    if not query.get("class", "").strip():
        raise BackendError("Missing query parameter: class (the directory class ID, e.g. class=0x0700).", 400)
    class_id = _query_int(query, "class", None)
    assert class_id is not None
    return {"block": ctl.read_block(class_id, _query_int(query, "offset", 0) or 0, _query_int(query, "count", None))}


def _axis_query(query: Query) -> str:
    name = query.get("axis", "").strip()
    if not name:
        raise BackendError("Missing query parameter: axis (e.g. axis=COM3:1).", 400)
    return name


def _wb_post_routes(wb: Workbench) -> dict[str, Callable[[Body], Any]]:
    """The multi-axis workbench API (every axis named "<port>:<slave>", e.g. "COM3:1")."""
    return {
        "/api/wb/connect": lambda b: wb.connect(
            port=b.get("port"), simulate=_flag(b, "simulate", False), sim_setup=str(b.get("sim_setup", "slider")),
            slaves=b.get("slaves"), baud=None if b.get("baud") in (None, "", "auto") else _integer(b, "baud", 115200)),
        "/api/wb/disconnect": lambda b: wb.disconnect(b.get("port")),
        "/api/wb/call": lambda b: wb.call(b.get("axis"), b.get("fn"), b.get("args")),
        "/api/wb/stop": lambda b: wb.stop(b.get("axis"), bool(b.get("broadcast"))),
        "/api/wb/port": lambda b: wb.port_call(b.get("port"), b.get("fn"), b.get("args")),
        "/api/wb/group": lambda b: wb.group(b.get("op"), b),
        "/api/wb/coord/plan": lambda b: wb.coord.plan(b),
        "/api/wb/coord/run": lambda b: wb.coord.run(b),
        "/api/wb/coord/stop": lambda b: wb.coord.stop(),
        "/api/wb/report": lambda b: _report(wb, b),
    }


def _report(wb: Workbench, body: Body) -> dict[str, Any]:
    """POST /api/wb/report: the plain-text test report (cached state only: no bus traffic)."""
    text, filename = build_report(wb, body, sim_only=wb.sim_only)
    return {"text": text, "filename": filename}


def _wb_get_routes(wb: Workbench) -> dict[str, Callable[[Query], Any]]:
    return {
        "/api/wb/state": lambda q: wb.snapshot(_query_int(q, "since", 0, high=2**31) or 0),
        "/api/wb/ports": lambda q: wb.list_ports(),
        "/api/wb/functions": lambda q: catalog(),
        "/api/wb/positions": lambda q: wb.positions(_axis_query(q)),
        "/api/wb/parameters": lambda q: wb.parameters(_axis_query(q)),
        "/api/wb/overview": lambda q: wb.overview(_axis_query(q)),
        "/api/wb/coord/status": lambda q: wb.coord.status(_query_int(q, "since", 0, high=2**31) or 0),
    }


def _post_routes(ctl: Controller) -> dict[str, Callable[[Body], Any]]:
    return {
        "/api/connect": lambda b: ctl.connect(
            str(b.get("port", "COM3")), _integer(b, "baud", 115200), _integer(b, "slave", 1), _flag(b, "simulate", True),
            _stroke(b), _sim_profile(b)),
        "/api/disconnect": lambda b: ctl.disconnect(),
        "/api/servo": lambda b: ctl.servo(_flag(b, "on")),
        "/api/home": lambda b: ctl.home(),
        "/api/move": lambda b: ctl.move(
            _required(b, "position_mm"), _required(b, "speed_mm_s"), require_number(b.get("accel_g", 0.1), "accel_g")),
        "/api/jog": lambda b: ctl.jog(
            _required(b, "delta_mm"), _required(b, "speed_mm_s"), require_number(b.get("accel_g", 0.1), "accel_g")),
        "/api/push": lambda b: ctl.push(
            _required(b, "target_mm"), _required(b, "band_mm"), _required(b, "force_pct"), _direction(b),
            _required(b, "speed_mm_s"), require_number(b.get("accel_g", 0.1), "accel_g")),
        "/api/stop": lambda b: ctl.stop(take_control=_flag(b, "take_control", False)),
        "/api/control": lambda b: ctl.set_control(_control_mode(b)),
        "/api/reset_alarm": lambda b: ctl.reset_alarm(),
        # Stored-position move and configuration writes (each behind a confirm dialog in the browser).
        "/api/run_stored": lambda b: ctl.run_stored(_whole(b, "n"), _flag(b, "allow_above_caps", False)),
        "/api/param": lambda b: ctl.write_param(
            _whole(b, "no"), _param_value(b), _flag(b, "allow_home_params", False),
            _flag(b, "allow_unverified_model", False)),
        "/api/software_reset": lambda b: ctl.software_reset(),
        "/api/baud": lambda b: ctl.set_baud(_whole(b, "rate"), _method(b)),
    }


def _get_routes(ctl: Controller) -> dict[str, Callable[[Query], Any]]:
    # GET routes only read. The Unit info view uses only these (never a POST): unit_info / read_block
    # send FC03 + FC11 reads, unit_live decodes the status poll's words without any bus access.
    return {
        "/api/status": lambda q: ctl.snapshot(),
        "/api/positions": lambda q: {"rows": ctl.positions()},
        "/api/identify": lambda q: {"identity": ctl.identify()},
        "/api/unit_info": lambda q: {"info": ctl.unit_info()},
        "/api/unit_live": lambda q: ctl.unit_live(),
        "/api/read_block": lambda q: _read_block(ctl, q),
    }


class GuiServer(ThreadingHTTPServer):
    """ThreadingHTTPServer that carries the controller, the workbench and the per-run API token."""

    daemon_threads = True
    log_errors = True  # print failed requests (4xx/5xx) to the console
    # On Windows SO_REUSEADDR lets a second GUI silently share the port; bind exclusively instead.
    allow_reuse_address = os.name != "nt"

    def __init__(self, port: int, controller: Controller, token: str | None = None,
                 workbench: Workbench | None = None) -> None:
        super().__init__((HOST, port), Handler)
        self.controller = controller
        self.workbench = workbench if workbench is not None else Workbench()
        self.token = token or secrets.token_hex(16)
        self.port = self.server_address[1]
        self.console = ConsoleLog()
        self.quit_requested = threading.Event()
        self.closing = threading.Event()  # the safe shutdown has begun: no request may start or connect anything now
        self._closing = threading.Lock()  # held for the whole safe shutdown: a second caller waits for the first
        self._closed = False
        self.get_routes = {**_get_routes(controller), **_wb_get_routes(self.workbench),
                           "/api/about": lambda q: about_info(self.workbench.sim_only)}
        self.post_routes = {**_post_routes(controller), **_wb_post_routes(self.workbench)}
        # One STOP for both pages: the workbench's STOP ALL also stops the single-axis page's axis, and its STOP
        # also stops the workbench's axes.
        group = self.post_routes["/api/wb/group"]
        self.post_routes["/api/wb/stop"] = self._workbench_stop
        self.post_routes["/api/wb/group"] = lambda b: self._workbench_stop(b) if b.get("op") == "stop_all" else group(b)
        self.post_routes["/api/stop"] = self._classic_stop
        self.post_routes["/api/quit"] = lambda b: self.quit()

    def server_bind(self) -> None:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def launch_url(self) -> str:
        """The address that opens the GUI: the access key follows the ``#``, so it is never sent to a server, a log or a
        Referer (the page keeps it in its tab's sessionStorage and removes it from the address bar)."""
        return f"http://{HOST}:{self.port}/#k={self.token}"

    # ------------------------------------------------------------------ STOP and the end of the program

    @staticmethod
    def _in_parallel(first: Callable[[], Any], second: Callable[[], Any]) -> tuple[Any, BaseException | None]:
        """``first()`` in this thread and ``second()`` in another at the same time: each STOP waits for its own axis
        to stand still, so one after the other would delay the second. Returns first's result and second's exception
        (None when it passed); an exception of ``first`` is raised after ``second`` has finished."""
        box: dict[str, BaseException | None] = {"error": None}

        def run_second() -> None:
            try:
                second()
            except BaseException as exc:  # noqa: BLE001 - handed back to the caller
                box["error"] = exc

        worker = threading.Thread(target=run_second, name="gui-stop-second", daemon=True)
        worker.start()
        try:
            result = first()
        finally:
            worker.join()
        return result, box["error"]

    def stop_everything(self, *, broadcast: bool = False) -> dict[str, Any]:
        """STOP ALL for both pages: every axis of the workbench (ports in parallel) and the single-axis page's axis when
        it is connected. ``{"stopped": [...], "errors": {name: text}}``; the single-axis page not being connected is
        not an error."""
        if not self.controller.is_connected():
            out = self.workbench.stop(broadcast=broadcast)
            return {"stopped": list(out.get("stopped", [])), "errors": dict(out.get("errors", {}))}
        out, error = self._in_parallel(lambda: self.workbench.stop(broadcast=broadcast), self.controller.stop)
        stopped, errors = list(out.get("stopped", [])), dict(out.get("errors", {}))
        if error is None:
            stopped.append(CLASSIC_NAME)
        elif isinstance(error, BackendError) and error.http_status == 409 and not self.controller.is_connected():
            pass  # disconnected meanwhile: nothing to stop
        else:
            errors[CLASSIC_NAME] = str(error)
        return {"stopped": stopped, "errors": errors}

    def _workbench_stop(self, body: Body) -> dict[str, Any]:
        """POST /api/wb/stop and the group operation stop_all: one axis (``axis``), else everything."""
        if body.get("axis") is not None:
            return self.workbench.stop(body.get("axis"), bool(body.get("broadcast")))
        return self.stop_everything(broadcast=bool(body.get("broadcast")))

    def _classic_stop(self, body: Body) -> None:
        """POST /api/stop of the single-axis page: its own STOP (as before) and, at the same time, STOP ALL on the
        workbench's axes."""
        take_control = _flag(body, "take_control", False)
        if not self.workbench.axes:
            return self.controller.stop(take_control=take_control)
        if self.controller.is_connected():
            out, classic_error = self._in_parallel(self.workbench.stop,
                                                   lambda: self.controller.stop(take_control=take_control))
            if classic_error is not None:  # the single-axis page's own STOP failed: its message, as before
                raise classic_error
        else:
            out = self.workbench.stop()
        if out.get("errors"):
            raise BackendError("STOP failed on the workbench: " + "; ".join(f"{n}: {e}" for n, e in out["errors"].items()), 502)
        return None

    def shutdown_safely(self) -> None:
        """The end of the program, once: STOP to every axis of both pages first (fast: ports in parallel), then each unit
        left as found (a move stopped, the HOME coil released, control handed back to PIO if it had it). A second
        caller (Ctrl+C while the close handler runs) waits for the first."""
        self.closing.set()  # at once, also for a caller that waits below: the units are about to be left
        with self._closing:
            if self._closed:
                return
            self._closed = True
            try:
                self.stop_everything()
            except Exception:  # noqa: BLE001 - the disconnects below stop each axis again
                pass
            for close in (self.workbench.close, self.controller.close):
                try:
                    close()
                except Exception as exc:  # noqa: BLE001
                    print(f"Shutdown: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)

    def quit(self) -> dict[str, Any]:
        """POST /api/quit (the page's Quit button): the safe shutdown, then the server ends (after the reply is sent)."""
        self.shutdown_safely()
        self.quit_requested.set()
        return {"quit": True}


class Handler(BaseHTTPRequestHandler):
    """Static files plus the JSON API."""

    server: GuiServer
    server_version = SERVER_NAME
    sys_version = ""
    timeout = SOCKET_TIMEOUT_S  # an idle connection must not keep its thread for ever

    def version_string(self) -> str:
        return SERVER_NAME

    def do_GET(self) -> None:  # noqa: N802 (http.server naming)
        parts = urlsplit(self.path)
        path = parts.path
        if not self._trusted_host():
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "Untrusted Host header."})
        if path in STATIC_FILES:
            return self._send_static(*STATIC_FILES[path])
        if path in DOC_FILES:
            return self._send_doc(resources.doc_file(DOC_FILES[path]))
        if path == "/favicon.ico":
            return self._send(HTTPStatus.NO_CONTENT, b"", "image/x-icon")
        route = self.server.get_routes.get(path)
        if route is None:
            return self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"Not found: {path}"})
        if not self._trusted_origin():
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": ORIGIN_REFUSED})
        if not self._token_ok():
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": TOKEN_REFUSED})
        query = {key: values[-1] for key, values in parse_qs(parts.query).items()}
        self._run(lambda: route(query))

    def do_POST(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if not self._trusted_host():
            self._drain_body()
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "Untrusted Host header."})
        if not self._trusted_origin():
            self._drain_body()
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": ORIGIN_REFUSED})
        if not self._token_ok():
            self._drain_body()
            return self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": TOKEN_REFUSED})
        route = self.server.post_routes.get(path)
        if route is None:
            self._drain_body()
            return self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"Not found: {path}"})
        try:
            body = self._read_json()
        except BackendError as exc:
            return self._send_json(exc.http_status, {"ok": False, "error": str(exc)})
        if self.server.closing.is_set() and path not in CLOSING_OK and not (
                (path == "/api/wb/group" and body.get("op") == "stop_all")
                or (path == "/api/wb/call" and body.get("fn") == "stop")):  # STOP of one axis, from the Axis tab
            # The units were stopped and are being left as found: a command or a connect now would be left running by the
            # end of the program (a unit has no watchdog).
            return self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": CLOSING_REFUSED})
        self._run(lambda: route(body))
        if self.server.quit_requested.is_set():  # the reply is out: end serve_forever from another thread
            threading.Thread(target=self.server.shutdown, name="gui-quit", daemon=True).start()

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 (stdlib signature)
        # Status polling would flood the console; only report failed requests. The line goes to the console queue
        # (see ConsoleLog): no request thread ever waits for the console.
        if self.server.log_errors and len(args) >= 2 and str(args[1])[:1] in "45":
            self.server.console.put(f"[gui] {self.address_string()} {format % args}")

    def end_headers(self) -> None:
        for key, value in SECURITY_HEADERS.items():  # every answer, the 501 of an unsupported method included
            self.send_header(key, value)
        super().end_headers()

    # ------------------------------------------------------------------ helpers

    def _run(self, action: Callable[[], Any]) -> None:
        try:
            result = action()
        except BackendError as exc:
            return self._send_json(exc.http_status, {**exc.extra, "ok": False, "error": str(exc)})
        except Exception as exc:  # last resort: never leave the browser without an answer
            message = f"Internal error: {type(exc).__name__}: {exc}"
            self.server.controller.log("error", message)
            return self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": message})
        payload = {"ok": True}
        if isinstance(result, dict):
            payload.update(result)
        self._send_json(HTTPStatus.OK, payload)

    def _trusted_host(self) -> bool:
        port = self.server.port
        return self.headers.get("Host") in {f"127.0.0.1:{port}", f"localhost:{port}"}

    def _trusted_origin(self) -> bool:
        """A browser request to the API must come from this server's own page: ``Origin`` (sent on a POST and on a
        cross-origin request) must be this server's, and ``Sec-Fetch-Site`` must be same-origin (or none: an address
        typed in). A second layer behind the token and the Host check; a client without these headers (curl) passes."""
        port = self.server.port
        origin = self.headers.get("Origin")
        if origin is not None and origin not in {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}:
            return False
        site = self.headers.get("Sec-Fetch-Site")
        return site is None or site in ("same-origin", "none")

    def _token_ok(self) -> bool:
        supplied = self.headers.get(TOKEN_HEADER, "").encode("utf-8", "replace")
        return secrets.compare_digest(supplied, self.server.token.encode())

    def _drain_body(self) -> None:
        """Read (and drop) a refused request's body before answering: a socket closed with unread data is reset
        on Windows, and the client then sees 'connection aborted' instead of the 403 / 404."""
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return
        if 0 < length <= MAX_BODY_BYTES:
            self.rfile.read(length)

    def _read_json(self) -> Body:
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            raise BackendError("Bad Content-Length.", 400) from None
        if length < 0:  # rfile.read(-1) would read until the client closes the connection
            raise BackendError("Bad Content-Length.", 400)
        if length > MAX_BODY_BYTES:
            raise BackendError("Request body too large.", 413)
        if length == 0:
            return {}
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            raise BackendError("Content-Type must be application/json.", 415)
        try:
            body = json.loads(self.rfile.read(length))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise BackendError("Body is not valid JSON.", 400) from None
        if not isinstance(body, dict):
            raise BackendError("Body must be a JSON object.", 400)
        return body

    def _send_static(self, name: str, content_type: str) -> None:
        try:
            data = resources.gui_file(name).read_bytes()
        except OSError:
            return self._send_json(HTTPStatus.NOT_FOUND,
                                   {"ok": False, "error": f"Not found: {name} (not part of this install)."})
        if name.endswith(".html"):  # the access key is not in the page: it comes in the address (after the #)
            data = data.replace(b"__VERSION__", __version__.encode())
        self._send(HTTPStatus.OK, data, content_type)

    def _send_doc(self, path: Path) -> None:
        try:
            data = path.read_bytes()
        except OSError:
            return self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"Not found: {path.name} (not part of this install)."})
        self._send(HTTPStatus.OK, data, "text/plain; charset=utf-8")

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        self._send(status, json.dumps(payload).encode(), "application/json; charset=utf-8")

    def _send(self, status: int, data: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _stroke_arg(text: str) -> tuple[float, float]:
    try:
        return parse_stroke(text)
    except BackendError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from None


def _wait_if_frozen() -> None:
    """In the iai-gui.exe console window: keep a fatal start message readable until Enter."""
    if resources.frozen():
        try:
            input("Press Enter to close.")
        except EOFError:
            pass


def redirect_page(url: str) -> Path:
    """A page that only forwards to ``url``, in a new folder that only this user can read (mode 0700, the file 0600).
    On Linux and macOS the command line of a browser is visible to every user (/proc), so the address with the access
    key is not put on it: the browser is given this file instead. It is removed again after a minute."""
    folder = Path(tempfile.mkdtemp(prefix="iai-gui-"))
    page = folder / "open.html"
    page.write_text(
        '<!doctype html><meta charset="utf-8"><meta name="referrer" content="no-referrer"><title>iai-gui</title>'
        f'<meta http-equiv="refresh" content="0;url={html.escape(url)}">'
        f'<p><a href="{html.escape(url)}">Open the GUI</a></p>\n', encoding="utf-8")
    try:
        os.chmod(page, 0o600)
    except OSError:
        pass
    cleanup = threading.Timer(60.0, shutil.rmtree, args=(folder,), kwargs={"ignore_errors": True})
    cleanup.daemon = True
    cleanup.start()
    return page


def _open_browser(url: str, *, via_file: bool | None = None) -> None:
    if via_file is None:  # Windows: another account cannot read this one's command line. Elsewhere: a private file.
        via_file = os.name != "nt"
    try:
        webbrowser.open(redirect_page(url).as_uri() if via_file else url)
    except Exception as exc:  # noqa: BLE001 - the address is printed anyway
        print(f"Could not open the browser ({exc}): open the address above yourself.", file=sys.stderr, flush=True)


def main(argv: list[str] | None = None) -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        prog="iai-gui", description=f"{PRODUCT} workbench {__version__} - unofficial test GUI for IAI ELECYLINDER and "
        "CON-family units (loopback only; not an IAI product).")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port on 127.0.0.1 (default {DEFAULT_PORT})")
    parser.add_argument(
        "--stroke", type=_stroke_arg, default=DEFAULT_STROKE, metavar="MIN,MAX",
        help="optional default stroke override in mm (default: each EC's own soft limits minus their "
             "0.30 mm margin, read at connect); an override can only narrow the EC's limits")
    parser.add_argument("--sim-only", action="store_true",
                        help="demo mode: no serial port is listed or opened (workbench and single-axis page), simulators only")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser at start")
    args = parser.parse_args(argv)

    try:
        controller = Controller(stroke=args.stroke, sim_only=args.sim_only)
    except Exception as exc:  # noqa: BLE001 - say why, keep the console readable
        print(f"iai-gui could not start: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        _wait_if_frozen()
        return 1
    try:
        server = GuiServer(args.port, controller, workbench=Workbench(sim_only=args.sim_only))
    except OSError as exc:
        print(f"Cannot listen on {HOST}:{args.port}: {exc}", file=sys.stderr)
        print(f"Another iai-gui may be running: open http://{HOST}:{args.port}/ or start with --port {args.port + 1}.",
              file=sys.stderr, flush=True)
        controller.close()
        _wait_if_frozen()
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"iai-gui could not start: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        controller.close()
        _wait_if_frozen()
        return 1
    controller.start()
    url = server.launch_url()
    print(f"{PRODUCT} workbench {__version__} (unofficial, not an IAI product)", flush=True)
    print(f"Open {url}", flush=True)
    print("  The part after # is this run's access key: do not share the address. The single-axis page is at /classic.",
          flush=True)
    print("Quit with the Quit button in the page or Ctrl+C. Closing this window also stops the units first.", flush=True)
    install_console_handler(server.shutdown_safely)
    if args.sim_only:
        print("Demo mode (--sim-only): simulators only; no serial port is listed or opened.", flush=True)
    if args.stroke is None:
        print("Stroke: each EC's own soft limits, read at connect (override with --stroke or in the connect form)",
              flush=True)
    else:
        print(f"Default stroke override {args.stroke[0]:g}-{args.stroke[1]:g} mm (narrowed to each EC's own limits)",
              flush=True)
    if not args.no_browser:
        print("Opening the browser (start with --no-browser to skip)", flush=True)
        opener = threading.Timer(0.3, _open_browser, args=(url,))
        opener.daemon = True
        opener.start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.shutdown_safely()  # STOP everything, then leave each unit as found (once: Quit may have done it)
        print("GUI stopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
