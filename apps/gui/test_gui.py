"""Tests for the EC test GUI server (stdlib unittest; never opens a serial port).

Run from the repository root:  python -m unittest discover -s apps/gui -t apps -v
"""

from __future__ import annotations

import contextlib
import http.client
import io
import json
import math
import os
import re
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable, Iterator
from unittest import mock

from iai_actuator_driver import BACKEND, __version__
from iai_actuator_driver import registers as R
from iai_actuator_driver.errors import UnsafeCommandError
from iai_actuator_driver.coord import CoordinatedGroup
from iai_actuator_driver.modbus import ModbusRTU
from iai_actuator_driver.sim import SimulatedEC
from gui import backend
from gui.backend import DEFAULT_STROKE, BackendError, EcLimits, open_session
from gui import coordinated as coordinated_module
from gui import server as server_module
from gui.controller import UNIT_INFO_GROUPS, Controller, ReadInterrupted, parse_stroke
from gui.server import GuiServer, main
from gui import resources
from gui import workbench as workbench_module
from gui.report import build_report
from gui.workbench import FUNCTIONS, SIM_SETUPS, STATUS_KEYS, Workbench, classify_open_error, parse_slaves

TOKEN = "test-token"


class FakeAxis:
    """Deterministic test double for an axis session: constant-speed moves, no Modbus at all.

    Models the first test unit: 150 mm stroke, soft limits -0.30..150.30 mm, and PIO in control
    until the GUI enables Modbus control. STOP stops it either way (hardware test 19 showed this for a
    Modbus-commanded move with PMSS = 0; ``pio_move`` stands in for a PLC move, untested on hardware).
    ``pio_move(..., hold_start=True)`` models a PLC holding its start input: the move restarts after STOP.
    A push approaches its target, then pushes to the band end and misses, unless ``obstacle_mm``
    lies in the band (contact).
    """

    HOME_SPEED_MM_S = 50.0

    def __init__(self, slave: int = 1, stroke: tuple[float, float] | None = DEFAULT_STROKE) -> None:
        self.description = "test fake axis"
        self.baud: int | None = None
        self.line_baud: int | None = 115200  # the modelled line rate (a simulator session has no serial port)
        self.params: dict[int, Any] = {2: 0.1}
        self.param_opt_ins: list[bool] = []  # allow_unverified_model per write_param call
        self.max_speed_mm_s = 100.0
        self.stroke = (0.0, 150.0) if stroke is None else (float(stroke[0]), float(stroke[1]))  # None: the EC's own
        self.control_writes = 0
        self.obstacle_mm: float | None = None
        self._slave = slave
        self._pos = 73.95
        self._target = self._pos
        self._speed = 0.0
        self._t = time.perf_counter()
        self._servo = False
        self._homed = True  # WA battery-less absolute: homed after power-up
        self._homing = False
        self._phase: str | None = None  # None | "approach" | "push"
        self._push_end = 0.0
        self._push_speed = 0.0
        self._contact = False
        self._missed = False
        self._alarm = 0
        self._control = False
        self._pio_restart: tuple[float, float] | None = None
        self._table = [
            {"position_mm": 0.0, "speed_mm_s": 108.5, "accel_g": 0.3, "decel_g": 0.3, "band_mm": 0.1},
            {"position_mm": 150.0, "speed_mm_s": 248.5, "accel_g": 0.3, "decel_g": 0.3, "band_mm": 0.1},
            {"position_mm": 75.0, "speed_mm_s": 108.5, "accel_g": 0.3, "decel_g": 0.3, "band_mm": 0.1},
        ]

    @classmethod
    def factory(cls, port: str, baud: int, slave: int, simulate: bool, *,
                stroke: tuple[float, float] | None = DEFAULT_STROKE, sim_profile: str = "slider") -> "FakeAxis":
        if not simulate:
            raise BackendError("The test fake only simulates; tick 'Simulate'.", 400)
        return cls(slave, stroke)

    # Unit info: a minimal stand-in for Elecylinder.unit_info (one dict per group of sections).
    unit_info_fail_on: str | None = None  # a section whose group raises (a lost link)
    _gate: Any = None  # the controller's per-request gate inside preemptible()

    @contextlib.contextmanager
    def preemptible(self, gate: Any) -> Iterator[None]:
        self._gate = gate
        try:
            yield
        finally:
            self._gate = None

    def _request(self) -> Any:
        """One 'Modbus request' of a read-only read: inside the controller's gate when it runs preemptibly."""
        return self._gate() if self._gate is not None else contextlib.nullcontext()

    def unit_info(self, sections: Any) -> dict[str, Any]:
        sections = list(sections)
        self.unit_info_calls = getattr(self, "unit_info_calls", []) + [sections]
        with self._request():
            pass
        if self.unit_info_fail_on in sections:
            raise TimeoutError("no reply")
        data: dict[str, Any] = {
            "meta": {"schema": "ec_driver.unit_info/1", "read_only": True, "function_codes": ["03", "11"],
                     "sections": sections, "duration_s": 0.01},
            **{key: None for key in ("identity", "parameters", "monitor", "status_words", "io", "control_words",
                                     "alarms", "maintenance", "positions")},
            "errors": [],
        }
        if "identity" in sections:
            data["identity"] = {"model": "FAKE-EC", "position_mode": {
                "code": 4, "name": "3-position", "positions": 3, "layout": "3pos", "source": "fake",
                "confirmed": False, "note": ""}}
        if "maintenance" in sections:
            data["maintenance"] = {"total_moves": 7}
        if "positions" in sections:
            data["errors"].append("positions: the fake has no position data")
        return data

    def read_block(self, class_id: int, offset: int, count: int | None) -> dict[str, Any]:
        raise BackendError("The test fake has no register directory.", 404)

    def _advance(self) -> None:
        now = time.perf_counter()
        step = self._speed * (now - self._t)
        self._t = now
        if self._speed == 0:
            return
        gap = self._target - self._pos
        new = self._target if abs(gap) <= step else self._pos + math.copysign(step, gap)
        if self._phase == "push" and self.obstacle_mm is not None and \
                min(self._pos, new) <= self.obstacle_mm <= max(self._pos, new):
            self._pos, self._speed, self._phase, self._contact = self.obstacle_mm, 0.0, None, True
            return
        self._pos = new
        if new != self._target:
            return
        self._speed = 0.0
        if self._homing:
            self._homing, self._homed = False, True
        if self._phase == "approach":
            self._phase, self._target, self._speed = "push", self._push_end, self._push_speed
        elif self._phase == "push":
            self._phase, self._missed = None, True

    def _start(self, target: float, speed: float) -> None:
        self._advance()
        self._target, self._speed = target, speed
        self._phase, self._contact, self._missed = None, False, False

    def identify(self) -> dict[str, Any]:
        return {"model": "FAKE-EC (test double)", "slave": self._slave, "stroke_mm": list(self.stroke)}

    def status(self) -> dict[str, Any]:
        return self._status_now()

    def _status_now(self) -> dict[str, Any]:
        self._advance()
        moving = self._speed > 0
        dopm = {n: not moving and self._servo and abs(self._pos - row["position_mm"]) <= row["band_mm"]
                for n, row in enumerate(self._table)}
        return {
            "position_mm": round(self._pos, 2),
            "speed_mm_s": self._speed,
            "servo_on": self._servo,
            "homed": self._homed,
            "in_position": self._servo and not moving and not self._missed,
            "moving": moving,
            "alarm_code": self._alarm,
            "alarm": self._alarm != 0,
            "warning": False,
            "modbus_control": self._control,
            "emergency_stop": False,
            "safety_speed": False,
            "paused": False,
            "homing": self._homing,
            "pushing": self._phase == "push" or self._contact,
            "push_contact": self._contact,
            "push_missed": self._missed,
            "target_reached": False,
            "move_bit": moving,
            "at_backward_end": dopm[0],
            "at_forward_end": dopm[1],
            "at_middle": dopm[2],
            "raw": {},
        }

    def ec_limits(self) -> tuple[EcLimits | None, str]:
        return EcLimits((-0.30, 150.30), (0.0, 150.0)), "read from the fake"

    def set_stroke(self, stroke: tuple[float, float]) -> None:
        self.stroke = (float(stroke[0]), float(stroke[1]))

    def pio_move(self, position_mm: float, speed_mm_s: float, hold_start: bool = False) -> None:
        """Test hook: start a move as if a PLC had commanded it over PIO (``hold_start``: input stays on)."""
        self._pio_restart = (position_mm, speed_mm_s) if hold_start else None
        self._start(position_mm, speed_mm_s)

    def enable_control(self) -> None:
        self.control_writes += 1
        self._control = True

    def release_control(self) -> None:
        self._control = False

    def servo(self, on: bool) -> None:
        self._advance()
        if not on:
            self._target, self._speed, self._homing, self._phase = self._pos, 0.0, False, None
        self._servo = on

    def home(self) -> None:
        if not self._servo:
            raise BackendError("Fake axis: servo is OFF.")
        self._start(self.stroke[0], self.HOME_SPEED_MM_S)
        self._homed, self._homing = False, True

    def move_to(self, position_mm: float, speed_mm_s: float, accel_g: float) -> None:
        self._start(position_mm, speed_mm_s)

    def jog(self, delta_mm: float, speed_mm_s: float, accel_g: float) -> None:
        self._advance()
        self._start(round(self._pos + delta_mm, 2), speed_mm_s)

    def push(self, target_mm: float, band_mm: float, force_pct: float, direction: int,
             speed_mm_s: float, accel_g: float) -> None:
        self._start(target_mm, speed_mm_s)
        self._phase, self._push_end, self._push_speed = "approach", target_mm + direction * band_mm, min(speed_mm_s, 20)

    def stop(self) -> dict[str, Any]:
        """Stop at once; returns the standstill status (what the driver's STOP hold reads before releasing STOP)."""
        self._advance()  # STOP with or without Modbus control (test 19: a Modbus-commanded move, PMSS = 0)
        self._target, self._speed, self._homing, self._phase, self._contact = self._pos, 0.0, False, None, False
        final = self._status_now()  # (not self.status: tests patch that to model a failing read)
        if self._pio_restart is not None and not self._control:  # worst case: the PLC still holds its start input
            self._start(*self._pio_restart)
        return final

    def reset_alarm(self) -> None:
        self._alarm = 0

    def run_stored(self, n: int, allow_above_caps: bool) -> None:
        row = self._table[n]
        if row["speed_mm_s"] > self.max_speed_mm_s and not allow_above_caps:
            raise BackendError(f"Fake axis: row {n} runs at {row['speed_mm_s']} mm/s, above the cap.", 400)
        self._start(row["position_mm"], row["speed_mm_s"])

    def write_param(
        self, no: int, value: Any, allow_home_params: bool, allow_unverified_model: bool = False
    ) -> dict[str, Any]:
        self.param_opt_ins.append(allow_unverified_model)
        before, self.params[no] = self.params.get(no), value
        return {"no": no, "name": f"fake P{no}", "before": str(before), "after": str(value), "committed": True,
                "needs_reset": True, "warnings": ["fake warning"]}

    def software_reset(self) -> dict[str, Any]:
        self._advance()
        self._target, self._speed, self._phase, self._contact = self._pos, 0.0, None, False
        self._servo, self._control, self._homed = True, False, False  # like the EC-R6S rod: incremental, home lost
        return {"accepted": True, "reply_lost": False, "baudrate": self.line_baud, "modbus_enabled": False,
                "safety_speed": False, "servo_on": True, "homed": False, "needs_home": True, "alarm_code": 0,
                "elapsed_s": 0.0, "notes": ["fake reset"], "status": self._status_now()}

    def set_baudrate(self, rate: int, method: str) -> dict[str, Any]:
        previous, self.line_baud = self.line_baud, rate
        return {"previous": previous, "baudrate": rate, "method": method, "verified": True, "probed": False}

    def table_row(self, n: int) -> dict[str, Any]:
        return dict(self._table[n])

    def close(self) -> None:
        self.stop()


def http_call(base: str, path: str, data: Any = None, *, token: str = TOKEN, raw: bytes | None = None,
              host: str | None = None) -> tuple[int, dict[str, Any]]:
    body = raw if raw is not None else (None if data is None else json.dumps(data).encode())
    req = urllib.request.Request(base + path, data=body, method="GET" if body is None else "POST")
    req.add_header("X-EC-Token", token)
    req.add_header("Content-Type", "application/json")
    if host:
        req.add_header("Host", host)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        with e:
            return e.code, json.loads(e.read())


def wait_until(predicate: Callable[[], bool], timeout: float = 6.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def run_off_the_main_thread(function: Callable[[], Any]) -> Any:
    """Run ``function`` on a thread of its own and return its result (or raise its error).

    The GUI's driver calls run on request and poller threads, never on the main thread. A
    blocking call made on the main thread runs on a helper thread (so that Ctrl+C reaches Python), outside the
    caller's preemptible scope; a test of that scope therefore runs where the GUI does.
    """
    outcome: dict[str, Any] = {}

    def run() -> None:
        try:
            outcome["value"] = function()
        except BaseException as exc:  # handed to the test's thread
            outcome["error"] = exc

    worker = threading.Thread(target=run, name="test-off-main")
    worker.start()
    worker.join()
    if "error" in outcome:
        raise outcome["error"]
    return outcome.get("value")


class _ServerCase(unittest.TestCase):
    session_factory = staticmethod(FakeAxis.factory)

    @classmethod
    def setUpClass(cls) -> None:
        cls.ctl = Controller(session_factory=cls.session_factory, poll_interval_s=0.05)
        cls.server = GuiServer(0, cls.ctl, token=TOKEN)
        cls.server.log_errors = False
        cls.base = f"http://127.0.0.1:{cls.server.port}"
        cls.ctl.start()
        cls.thread = threading.Thread(target=cls.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.ctl.close()

    def tearDown(self) -> None:
        if self.ctl.snapshot()["connected"]:
            self.ctl.disconnect()

    def call(self, path: str, data: Any = None, *, token: str = TOKEN, raw: bytes | None = None,
             host: str | None = None) -> tuple[int, dict[str, Any]]:
        return http_call(self.base, path, data, token=token, raw=raw, host=host)

    def ok(self, path: str, data: Any = None) -> dict[str, Any]:
        code, body = self.call(path, data)
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])
        return body

    def status(self) -> dict[str, Any]:
        return self.ok("/api/status")

    def connect_sim(self, profile: str | None = None) -> dict[str, Any]:
        body = {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True}
        if profile is not None:
            body["sim_profile"] = profile
        return self.ok("/api/connect", body)

    def wait_idle(self) -> dict[str, Any]:
        self.assertTrue(wait_until(lambda: self.status()["status"]["moving"] is False), "axis never stopped moving")
        return self.status()["status"]

    def wait_at(self, position_mm: float, timeout: float = 10.0) -> None:
        """Wait until the axis has arrived at position_mm and stopped."""
        def arrived() -> bool:
            st = self.status()["status"]
            return st["moving"] is False and abs(st["position_mm"] - position_mm) < 0.15
        self.assertTrue(wait_until(arrived, timeout), f"never arrived at {position_mm} mm: {self.status()['status']}")


class FakeAxisApiTest(_ServerCase):
    """Full endpoint coverage against the test fake (deterministic)."""

    def test_static_files_and_headers(self) -> None:
        with urllib.request.urlopen(self.base + "/classic") as r:
            html = r.read().decode()
            self.assertNotIn(TOKEN, html)  # the access key comes in the launch address (#k=...), not in the page
            self.assertNotIn("__TOKEN__", html)
            self.assertIn('src="/token.js"', html)
            self.assertIn("not an IAI product", html)
            self.assertEqual(r.headers["X-Frame-Options"], "DENY")
        for path in ("/app.js", "/unitinfo.js", "/params.js", "/styles.css", "/token.js"):
            with urllib.request.urlopen(self.base + path) as r:
                self.assertEqual(r.status, 200)
        # A unit that cannot push (limits.max_push_pct 0): the push form is disabled and refuses a submit.
        self.assertIn('id="pushUnavailable"', html)
        with urllib.request.urlopen(self.base + "/app.js") as r:
            app = r.read().decode()
        self.assertIn('$("pushBtn").disabled = !c || noPush;', app)
        self.assertIn("if (cannotPush()) throw Error(CANNOT_PUSH_TEXT);", app)

    def test_token_and_host_required(self) -> None:
        self.assertEqual(self.call("/api/status", token="wrong")[0], 403)
        self.assertEqual(self.call("/api/stop", {}, token="")[0], 403)
        self.assertEqual(self.call("/api/status", host="evil.example:80")[0], 403)
        self.assertEqual(self.call("/api/nope")[0], 404)

    def test_disconnected_state(self) -> None:
        s = self.status()
        self.assertFalse(s["connected"])
        self.assertIsNone(s["status"])
        self.assertIsNone(s["limits"]["stroke_max_mm"])  # default: the EC's own soft limits, read at connect
        self.assertIsNone(s["default_stroke"])
        for path, data in (("/api/servo", {"on": True}), ("/api/home", {}), ("/api/stop", {}),
                           ("/api/move", {"position_mm": 10, "speed_mm_s": 20}), ("/api/reset_alarm", {}),
                           ("/api/disconnect", {})):
            self.assertEqual(self.call(path, data)[0], 409, path)
        self.assertEqual(self.call("/api/positions")[0], 409)
        self.assertEqual(self.call("/api/identify")[0], 409)

    def test_bad_connect_requests_refused(self) -> None:
        # The fake factory refuses a real port before any port is touched.
        code, body = self.call("/api/connect", {"port": "COM250", "baud": 115200, "slave": 1, "simulate": False})
        self.assertEqual(code, 400)
        self.assertIn("Simulate", body["error"])
        self.assertEqual(self.call("/api/connect", {"port": "not a port", "simulate": False})[0], 400)
        self.assertEqual(self.call("/api/connect", {"baud": 1234, "simulate": True})[0], 400)
        self.assertEqual(self.call("/api/connect", {"slave": 0, "simulate": True})[0], 400)
        self.assertFalse(self.status()["connected"])

    def test_bad_bodies(self) -> None:
        self.connect_sim()
        self.assertEqual(self.call("/api/move", raw=b"{not json")[0], 400)
        self.assertEqual(self.call("/api/move", raw=b"[1, 2]")[0], 400)
        self.assertEqual(self.call("/api/move", {"position_mm": "abc", "speed_mm_s": 20})[0], 400)
        self.assertEqual(self.call("/api/move", {"speed_mm_s": 20})[0], 400)
        self.assertEqual(self.call("/api/move", {"position_mm": float("nan"), "speed_mm_s": 20})[0], 400)
        self.assertEqual(self.call("/api/servo", {})[0], 400)
        self.assertEqual(self.call("/api/servo", {"on": "yes"})[0], 400)
        self.assertEqual(self.call("/api/connect", {"simulate": True})[0], 409)  # already connected

    def test_full_session(self) -> None:
        connected = self.connect_sim()
        self.assertTrue(connected["connection"]["simulate"])
        s = self.status()
        self.assertTrue(s["connected"] and s["comm_ok"])
        self.assertAlmostEqual(s["status"]["position_mm"], 73.95)

        self.assertIn("model", self.ok("/api/identify")["identity"])
        rows = self.ok("/api/positions")["rows"]
        self.assertEqual([(r["n"], r["label"], r["position_mm"]) for r in rows],
                         [(0, "Backward", 0.0), (1, "Forward", 150.0), (2, "Middle", 75.0)])

        code, body = self.call("/api/move", {"position_mm": 80, "speed_mm_s": 20, "accel_g": 0.1})
        self.assertEqual(code, 409)
        self.assertIn("Servo is OFF", body["error"])

        self.ok("/api/servo", {"on": True})
        self.assertTrue(wait_until(lambda: self.status()["status"]["servo_on"] is True))
        self.assertTrue(self.status()["status"]["modbus_control"])

        for bad in ({"position_mm": 150.5, "speed_mm_s": 20}, {"position_mm": -1, "speed_mm_s": 20},
                    {"position_mm": 80, "speed_mm_s": 101}, {"position_mm": 80, "speed_mm_s": 0},
                    {"position_mm": 80, "speed_mm_s": 20, "accel_g": 0.5}):
            self.assertEqual(self.call("/api/move", bad)[0], 400, bad)

        self.ok("/api/move", {"position_mm": 80, "speed_mm_s": 100, "accel_g": 0.1})
        self.assertTrue(self.status()["status"]["moving"], "status not refreshed right after the command")
        self.wait_at(80.0)
        # Logged once the position has settled (idle and unchanged for MOTION_SETTLE_S), with the settled value.
        self.assertTrue(wait_until(lambda: any("Motion finished at 80.00 mm" in e["msg"]
                                               for e in self.status()["events"]), 2))

        self.ok("/api/jog", {"delta_mm": 1, "speed_mm_s": 20})
        self.wait_at(81.0)
        self.ok("/api/jog", {"delta_mm": -10, "speed_mm_s": 100})
        self.wait_at(71.0)

        self.ok("/api/move", {"position_mm": 145, "speed_mm_s": 100})
        self.wait_at(145.0)
        code, body = self.call("/api/jog", {"delta_mm": 10, "speed_mm_s": 20})
        self.assertEqual(code, 400)
        self.assertIn("outside the stroke", body["error"])

        # STOP during a move: axis halts part-way.
        self.ok("/api/move", {"position_mm": 100, "speed_mm_s": 20})
        self.assertTrue(wait_until(lambda: self.status()["status"]["moving"] is True))
        code, body = self.call("/api/move", {"position_mm": 120, "speed_mm_s": 20})
        self.assertEqual(code, 409)
        self.assertIn("moving", body["error"])
        self.ok("/api/stop", {})
        stopped = self.wait_idle()["position_mm"]
        self.assertTrue(100 < stopped < 145, stopped)

        self.ok("/api/home", {})
        self.assertTrue(wait_until(lambda: self.status()["status"]["homed"] is True and
                                   self.status()["status"]["position_mm"] == 0.0))
        self.ok("/api/reset_alarm", {})

        self.ok("/api/move", {"position_mm": 75, "speed_mm_s": 100})
        self.wait_at(75.0)
        self.assertTrue(self.status()["status"]["at_middle"])
        self.ok("/api/push", {"target_mm": 80, "band_mm": 10, "force_pct": 30, "direction": 1, "speed_mm_s": 50})
        self.assertTrue(wait_until(lambda: (self.status()["push"] or {}).get("state") == "missed", 5),
                        self.status()["push"])
        self.assertAlmostEqual(self.status()["status"]["position_mm"], 90.0)
        for bad in ({"force_pct": 80}, {"force_pct": 10}, {"direction": 0}, {"direction": True}, {"band_mm": 0},
                    {"target_mm": 145}, {"speed_mm_s": 101}):
            body = {"target_mm": 80, "band_mm": 10, "force_pct": 30, "direction": 1, "speed_mm_s": 20, **bad}
            self.assertEqual(self.call("/api/push", body)[0], 400, bad)
        self.ok("/api/servo", {"on": False})

        messages = [e["msg"] for e in self.status()["events"]]
        for expected in ("Servo is ON", "Motion finished at 80.00 mm", "STOP sent", "Home complete", "Modbus control enabled",
                         "Push result: MISSED"):
            self.assertTrue(any(expected in m for m in messages), expected)

        self.ok("/api/disconnect", {})
        self.assertFalse(self.status()["connected"])


class StopBypassTest(unittest.TestCase):
    """STOP must not wait behind queued commands, and must cancel them."""

    def test_stop_cancels_queued_command(self) -> None:
        ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        outcome: dict[str, Any] = {}

        def queued_move() -> None:
            try:
                ctl.move(100, 20, 0.1)
                outcome["result"] = "moved"
            except BackendError as exc:
                outcome["result"] = str(exc)

        ctl._driver_lock.acquire()  # simulate a long command holding the bus
        try:
            worker = threading.Thread(target=queued_move)
            worker.start()
            time.sleep(0.1)  # the move is now waiting for the lock
            started = time.monotonic()
            ctl.stop()
            self.assertLess(time.monotonic() - started, 1.0, "STOP waited for the bus")
        finally:
            ctl._driver_lock.release()
        worker.join(5)
        self.assertIn("cancelled by STOP", outcome["result"])
        self.assertFalse(ctl.snapshot()["status"]["moving"])
        ctl.close()


def messages(ctl: Controller) -> list[str]:
    return [e["msg"] for e in ctl.snapshot(event_count=300)["events"]]


class _ControllerCase(unittest.TestCase):
    """Direct Controller tests (no HTTP)."""

    def make(self, **kwargs: Any) -> Controller:
        kwargs.setdefault("session_factory", FakeAxis.factory)
        kwargs.setdefault("poll_interval_s", 0.05)
        ctl = Controller(**kwargs)
        ctl.start()
        self.addCleanup(ctl.close)
        return ctl


class PioControlFakeTest(_ControllerCase):
    """Hardware tests 18 / 19 (STOP with PMSS = 0 on a Modbus-commanded move; PMSL only switched at rest).

    A Modbus STOP on a PLC-commanded move is untested on hardware: the GUI says so and, while PIO has
    control, watches for the axis moving again after STOP.
    """

    def test_stop_sent_while_pio_has_control(self) -> None:
        ctl = self.make(stop_confirm_s=0.4)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        self.assertFalse(ctl.snapshot()["status"]["modbus_control"])
        note = [m for m in messages(ctl) if m.startswith("PIO has control")]
        self.assertEqual(len(note), 1, messages(ctl))
        self.assertIn("untested on a move a PLC commanded over PIO", note[0])
        self.assertIn("PLC or with the hardware stop", note[0])
        self.assertNotIn("verified on hardware", note[0])
        fake.pio_move(140.0, 20.0)  # stands in for a move a PLC started over PIO
        ctl.stop()  # no error: STOP is sent with PMSS = 0
        self.assertTrue(wait_until(lambda: ctl.snapshot()["status"]["moving"] is False, 2))
        sent = [m for m in messages(ctl) if m.startswith("STOP sent")]
        self.assertEqual(len(sent), 1, messages(ctl))
        self.assertIn("verified only on Modbus-commanded moves", sent[0])
        self.assertIn("watching 5 s", sent[0])
        self.assertNotIn("works anyway", sent[0])
        time.sleep(0.6)
        self.assertIsNone(ctl.snapshot()["stop_alert"])  # stayed stopped: no restart alert
        self.assertFalse(fake._control, "STOP must not switch PMSL")
        self.assertEqual(fake.control_writes, 0)

    def test_restart_after_stop_is_alerted(self) -> None:
        """A PLC still holding its start input restarts the move once STOP is released (worst case)."""
        ctl = self.make(stop_confirm_s=0.4, pio_restart_watch_s=2.0)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.pio_move(140.0, 20.0, hold_start=True)
        ctl.stop()
        self.assertTrue(wait_until(lambda: ctl.snapshot()["stop_alert"] is not None, 2), messages(ctl))
        alert = ctl.snapshot()["stop_alert"]
        self.assertTrue(alert.startswith("AXIS MOVED AGAIN AFTER STOP"), alert)
        self.assertIn("PLC / hardware stop", alert)
        self.assertTrue(any(m.startswith("AXIS MOVED AGAIN AFTER STOP") for m in messages(ctl)), messages(ctl))
        time.sleep(0.6)  # the alert stays while the axis moves or stops again; it is not "STOP NOT CONFIRMED"
        self.assertTrue(ctl.snapshot()["stop_alert"].startswith("AXIS MOVED AGAIN"))
        fake.pio_move(140.0, 20.0)  # the PLC drops its start input
        ctl.stop()  # a new STOP clears the alert
        self.assertIsNone(ctl.snapshot()["stop_alert"])

    def test_restart_watch_ignores_own_commands_and_ends(self) -> None:
        ctl = self.make(pio_restart_watch_s=0.5)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.pio_move(140.0, 20.0)
        ctl.stop(take_control=True)  # Modbus takes control at rest
        ctl.servo(True)
        ctl.move(100, 50, 0.3)  # this GUI's own move inside the watch window is not a restart
        self.assertTrue(wait_until(lambda: ctl.snapshot()["status"]["moving"] is True, 2))
        time.sleep(0.3)
        self.assertIsNone(ctl.snapshot()["stop_alert"], messages(ctl))
        # The watch ends after pio_restart_watch_s: a later PLC move raises nothing.
        ctl2 = self.make(pio_restart_watch_s=0.3)
        ctl2.connect("SIMULATED", 115200, 1, True)
        fake2 = ctl2._session
        fake2.pio_move(140.0, 20.0)
        ctl2.stop()
        time.sleep(0.5)
        fake2.pio_move(140.0, 20.0)
        time.sleep(0.3)
        self.assertIsNone(ctl2.snapshot()["stop_alert"], messages(ctl2))

    def test_control_not_taken_while_pio_moves_the_axis(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake._servo = True
        fake.pio_move(140.0, 20.0)
        for attempt in (lambda: ctl.servo(True), lambda: ctl.move(100, 20, 0.1), lambda: ctl.reset_alarm()):
            with self.assertRaises(BackendError) as caught:
                attempt()
            self.assertEqual(caught.exception.http_status, 409)
            self.assertIn("at rest", str(caught.exception))
        self.assertEqual(fake.control_writes, 0, "PMSL switched during motion")
        self.assertTrue(fake.status()["moving"], "the PIO move was disturbed")
        ctl.stop()
        ctl.servo(True)  # at rest now: control is taken first
        self.assertEqual(fake.control_writes, 1)
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])

    def test_take_control_after_stop(self) -> None:
        ctl = self.make(stop_confirm_s=0.4)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.pio_move(140.0, 20.0)
        ctl.stop(take_control=True)
        self.assertTrue(wait_until(lambda: ctl.snapshot()["status"]["moving"] is False, 2))
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])
        # PIO had control when STOP was sent (control is only taken after it): the note says what is verified.
        self.assertTrue(any(m.startswith("STOP sent (PIO has control") for m in messages(ctl)), messages(ctl))
        self.assertTrue(any("after STOP, at rest" in m for m in messages(ctl)), messages(ctl))
        time.sleep(0.6)
        self.assertIsNone(ctl.snapshot()["stop_alert"])
        ctl.disconnect()
        self.assertFalse(fake._control, "control not handed back to PIO on disconnect")

    def test_api_stop_flag(self) -> None:
        ctl = self.make()
        server = GuiServer(0, ctl, token=TOKEN)
        server.log_errors = False
        threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.port}"
        connect = {"simulate": True, "stroke_min_mm": 0, "stroke_max_mm": 100}
        self.assertEqual(http_call(base, "/api/connect", {**connect, "stroke_max_mm": "x"})[0], 400)
        self.assertEqual(http_call(base, "/api/connect", {"simulate": True, "stroke_min_mm": 0})[0], 400)
        self.assertEqual(http_call(base, "/api/connect", {**connect, "stroke_max_mm": -1})[0], 400)
        code, body = http_call(base, "/api/connect", connect)
        self.assertEqual(code, 200, body)
        self.assertEqual(body["connection"]["stroke"]["in_use_mm"], [0, 100])
        code, body = http_call(base, "/api/stop", {})
        self.assertEqual(code, 200, body)  # STOP is sent while PIO has control
        self.assertEqual(http_call(base, "/api/stop", {"take_control": "yes"})[0], 400)
        self.assertEqual(http_call(base, "/api/stop", {"take_control": True})[0], 200)
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])


class StrokeFakeTest(_ControllerCase):
    """Finding: the stroke must be configurable per EC and cross-checked at connect."""

    def test_parse_stroke(self) -> None:
        self.assertEqual(parse_stroke("0,50"), (0.0, 50.0))
        self.assertEqual(parse_stroke(" 10.5 , 300 "), (10.5, 300.0))
        for bad in ("50", "50,0", "0,0", "-1,50", "a,b", "0,nan", "0,99999", "0,50,60"):
            with self.assertRaises(BackendError, msg=bad):
                parse_stroke(bad)
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            main(["--stroke", "50,0"])

    def test_server_stroke_default_is_the_ecs_own(self) -> None:
        for argv, expected in (([], None), (["--stroke", "0,50"], (0.0, 50.0))):
            with mock.patch.object(server_module, "Controller") as controller, \
                    mock.patch.object(server_module, "GuiServer", side_effect=OSError("port busy")), \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(argv), 1)
            controller.assert_called_once_with(stroke=expected, sim_only=False)
        self.assertIsNone(DEFAULT_STROKE)
        self.assertIsNone(server_module._stroke({"simulate": True}))
        self.assertIsNone(server_module._stroke({"stroke_min_mm": None, "stroke_max_mm": None}))

    def test_server_default_stroke(self) -> None:
        ctl = self.make(stroke=(0.0, 50.0))
        self.assertEqual(ctl.snapshot()["default_stroke"], [0.0, 50.0])
        self.assertEqual(ctl.snapshot()["limits"]["stroke_max_mm"], 50.0)
        ctl.connect("SIMULATED", 115200, 1, True)
        snap = ctl.snapshot()
        self.assertEqual(snap["limits"]["stroke_max_mm"], 50.0)
        stroke = snap["connection"]["stroke"]
        self.assertEqual(stroke["in_use_mm"], [0.0, 50.0])
        self.assertEqual(stroke["table_mm"], [0.0, 150.0])
        # The fake's table (150 mm EC) reaches beyond a 50 mm stroke: loud warnings for rows 1 and 2.
        self.assertEqual(len(stroke["warnings"]), 2, stroke["warnings"])
        self.assertIn("Forward (1) is 150.00 mm", stroke["warnings"][0])
        ctl.servo(True)
        with self.assertRaises(BackendError) as caught:
            ctl.move(120, 100, 0.3)
        self.assertEqual(caught.exception.http_status, 400)
        with self.assertRaises(BackendError):
            ctl.jog(-90, 20)  # 73.95 - 90 < 0
        ctl.move(40, 100, 0.3)

    def test_stroke_narrowed_to_ec_stroke(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True, stroke=(0.0, 300.0))
        snap = ctl.snapshot()
        self.assertAlmostEqual(snap["limits"]["stroke_max_mm"], 150.0)  # soft limit 150.30 minus the 0.30 margin
        self.assertAlmostEqual(snap["connection"]["stroke"]["ec_soft_limits_mm"][1], 150.3)
        self.assertEqual(snap["connection"]["stroke"]["ec_stroke_mm"], [0.0, 150.0])
        self.assertIn("narrowed", snap["connection"]["stroke"]["warnings"][0])
        ctl.servo(True)
        with self.assertRaises(BackendError):
            ctl.move(200, 50, 0.1)

    def test_stroke_outside_ec_soft_limits_blocks_motion(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True, stroke=(200.0, 300.0))
        self.assertIn("Motion disabled", ctl.snapshot()["motion_block"])
        for attempt in (lambda: ctl.move(250, 20, 0.1), lambda: ctl.jog(1, 20)):
            with self.assertRaises(BackendError) as caught:
                attempt()
            self.assertIn("Motion disabled", str(caught.exception))
        ctl.disconnect()
        self.assertIsNone(ctl.snapshot()["motion_block"])

    def test_bad_stroke_refused_before_connecting(self) -> None:
        ctl = self.make()
        for bad in ((50.0, 0.0), (0.0, float("inf")), (-5.0, 10.0)):
            with self.assertRaises(BackendError):
                ctl.connect("SIMULATED", 115200, 1, True, stroke=bad)
        self.assertFalse(ctl.snapshot()["connected"])


class PushCapFakeTest(_ControllerCase):
    """Controller.limits: a session cap of 0 is a value (this unit cannot push), only None means "not known"."""

    def test_zero_push_cap_disables_push(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        self.assertEqual(ctl.limits.max_push_pct, 70.0)  # the fake reports no cap: the GUI's own
        fake.max_push_pct = None  # type: ignore[union-attr]
        self.assertEqual((ctl.limits.max_push_pct, ctl.limits.min_push_pct), (70.0, 20.0))
        fake.max_push_pct = 50.0  # type: ignore[union-attr]
        self.assertEqual((ctl.limits.max_push_pct, ctl.limits.min_push_pct), (50.0, 20.0))
        fake.max_push_pct = 0.0  # type: ignore[union-attr]
        self.assertEqual((ctl.limits.max_push_pct, ctl.limits.min_push_pct), (0.0, 0.0))
        self.assertEqual(ctl.snapshot()["limits"]["max_push_pct"], 0.0)
        ctl.servo(True)
        for force in (0, 30):
            with self.assertRaisesRegex(BackendError, "this unit cannot push") as caught:
                ctl.push(80.0, 10.0, force, 1, 10.0)
            self.assertEqual(caught.exception.http_status, 400)
        self.assertIsNone(ctl.snapshot()["push"])

    def test_accel_cap_from_the_session_or_the_gui(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.accel_cap_g = 0.2  # type: ignore[union-attr]
        self.assertEqual(ctl.limits.max_accel_g, 0.2)
        fake.accel_cap_g = None  # type: ignore[union-attr]
        self.assertEqual(ctl.limits.max_accel_g, 0.3)


class DisconnectFakeTest(_ControllerCase):
    """Finding: disconnect must decide on STOP from a fresh read, not from the cached status."""

    def test_stop_sent_when_motion_was_commanded_even_if_cache_says_idle(self) -> None:
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        ctl._refresh_after_command = lambda session: None  # imitate a lost post-command read
        ctl.move(100, 20, 0.1)
        self.assertFalse(ctl.snapshot()["status"]["moving"], "cache should be stale for this test")
        ctl.disconnect()
        self.assertTrue(any(m.startswith("STOP sent before disconnect: axis is moving") for m in messages(ctl)),
                        messages(ctl))

    def test_stop_sent_when_status_read_fails(self) -> None:
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        fake = ctl._session
        with mock.patch.object(fake, "status", side_effect=TimeoutError("no reply")):
            ctl.disconnect()
        self.assertTrue(any("STOP sent before disconnect: status read failed" in m for m in messages(ctl)),
                        messages(ctl))
        self.assertTrue(any("STOP NOT CONFIRMED before disconnect: no status reply" in m for m in messages(ctl)),
                        messages(ctl))

    def test_unconfirmed_stop_before_disconnect(self) -> None:
        """The poller cannot confirm a disconnect STOP, so disconnect itself must check it."""
        ctl = self.make(poll_interval_s=60, stop_confirm_s=0.3)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        ctl.move(140, 20, 0.1)
        fake = ctl._session
        with mock.patch.object(fake, "stop"):  # the EC ignores the STOP
            ctl.disconnect()
        self.assertTrue(any(m.startswith("STOP NOT CONFIRMED before disconnect: the axis is still moving")
                            for m in messages(ctl)), messages(ctl))
        self.assertIn("STOP NOT CONFIRMED", ctl.snapshot()["last_error"])

    def test_confirmed_stop_before_disconnect(self) -> None:
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        ctl.move(140, 20, 0.1)
        ctl.disconnect()
        self.assertIn("Axis stopped before disconnect", messages(ctl))
        self.assertFalse(any("NOT CONFIRMED" in m for m in messages(ctl)), messages(ctl))

    def test_failed_take_control_still_restores_pmsl(self) -> None:
        """PMSL may latch even when the EC refuses it (Monitor mode); disconnect must write it back."""
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        with mock.patch.object(fake, "enable_control", side_effect=RuntimeError("PMSS still 0")):
            ctl.stop(take_control=True)  # the STOP itself worked: no error
        self.assertTrue(any("Taking Modbus control after STOP failed: RuntimeError: PMSS still 0" in m
                            for m in messages(ctl)), messages(ctl))
        with mock.patch.object(fake, "release_control") as release:
            ctl.disconnect()
        release.assert_called_once_with()

    def test_stop_before_disconnect_with_pio_in_control(self) -> None:
        """STOP is sent with PMSS = 0 too, so disconnect stops a moving axis even while PIO has control."""
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.pio_move(140.0, 20.0)
        ctl.disconnect()
        self.assertTrue(any(m.startswith("STOP sent before disconnect: axis is moving") for m in messages(ctl)),
                        messages(ctl))
        self.assertIn("Axis stopped before disconnect", messages(ctl))
        self.assertEqual(fake._speed, 0.0)
        self.assertEqual(fake.control_writes, 0)

    def test_idle_without_motion_sends_no_stop(self) -> None:
        ctl = self.make(poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl.servo(True)
        ctl.disconnect()
        self.assertFalse(any("STOP sent" in m for m in messages(ctl)), messages(ctl))


class PushSettleTest(unittest.TestCase):
    """Controller rule (hardware test G5): contact / missed must hold, position unchanged, before it is shown."""

    def make(self) -> Controller:
        ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=60)  # no poller: statuses fed by hand
        self.addCleanup(ctl.close)
        ctl._push = {"state": "running", "target_mm": 2.0, "band_mm": 5.0, "end_mm": 7.0, "force_pct": 30,
                     "direction": 1, "speed_mm_s": 10.0, "start_mm": 1.0, "position_mm": None,
                     "message": "Pushing...", "_started": time.monotonic(), "_seen_start": True,
                     "_idle_since": None, "_outcome": None}
        return ctl

    @staticmethod
    def status(pos: float, *, contact: bool = False, missed: bool = False, moving: bool = False) -> dict[str, Any]:
        return {"position_mm": pos, "moving": moving, "move_bit": moving, "in_position": contact,
                "push_contact": contact, "push_missed": missed, "alarm": False, "servo_on": True}

    def test_g5_blip_then_miss(self) -> None:
        ctl = self.make()
        ctl._check_push(self.status(6.76, contact=True))  # live 19:23:08.248: PEND + MEND, DSSE 0x37C3, MOVE 0
        self.assertEqual(ctl._push["state"], "running")
        time.sleep(0.2)
        ctl._check_push(self.status(6.95, moving=True))  # still pushing: the fingerprint is gone
        self.assertEqual(ctl._push["state"], "running")
        ctl._check_push(self.status(7.0, missed=True))
        self.assertEqual(ctl._push["state"], "running")  # a miss must hold too
        time.sleep(0.2)
        ctl._check_push(self.status(7.0, missed=True))
        self.assertEqual(ctl._push["state"], "missed")

    def test_contact_that_moves_is_not_contact(self) -> None:
        ctl = self.make()
        ctl._check_push(self.status(6.76, contact=True))
        time.sleep(0.2)
        ctl._check_push(self.status(6.90, contact=True))  # 0.14 mm further: still moving
        self.assertEqual(ctl._push["state"], "running")

    def test_held_contact_is_reported(self) -> None:
        ctl = self.make()
        ctl._check_push(self.status(5.00, contact=True))
        time.sleep(0.2)
        ctl._check_push(self.status(5.01, contact=True))  # a holding contact may creep a little (test 14)
        self.assertEqual(ctl._push["state"], "contact")
        self.assertIn("Contact at 5.01 mm", ctl._push["message"])

    def test_contact_with_psfl_is_not_contact(self) -> None:
        ctl = self.make()
        for _ in range(2):
            st = self.status(7.0, contact=True)
            st["push_missed"] = True
            ctl._check_push(st)
            time.sleep(0.2)
        self.assertNotEqual(ctl._push["state"], "contact")


@contextlib.contextmanager
def simulated_ec(**options: Any) -> Iterator[list[Any]]:
    """Make the GUI's simulate=True sessions use a SimulatedEC built with ``options``.

    ``soft_limits=False`` makes the soft-limit block 0x0700 unreadable (the default serves
    -0.30 / stroke + 0.30 mm, like the test EC). Yields the list of simulators created.
    """

    pmsl = options.pop("pmsl", True)
    created: list[Any] = []
    real_sim = options.pop("sim_class", SimulatedEC)

    def factory(slave: int = 1, profile: str = "slider", **session_options: Any) -> Any:
        # The session's own options (e.g. its modelled line baud rate) first; an explicit option wins.
        sim = real_sim(slave=slave, **{"profile": profile, **session_options, **options})
        sim.pmsl = pmsl
        created.append(sim)
        return sim

    with mock.patch.object(backend, "SimulatedEC", factory):
        yield created


class SerialTimingSim(SimulatedEC):
    """SimulatedEC that takes as long as an RS-485 line at ``baud`` (8N1: 10 bits per byte) to receive a
    request and send its answer, so bus-lock hold times are realistic (the plain simulator answers at once).

    ``frames`` records (time the request started on the line, request) for every request.
    """

    def __init__(self, *args: Any, baud: int = 38400, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.byte_s = 10 / baud
        self.frames: list[tuple[float, bytes]] = []

    def write(self, data: bytes) -> None:
        self.frames.append((time.monotonic(), bytes(data)))
        time.sleep(len(data) * self.byte_s)
        super().write(data)

    def read(self, n: int) -> bytes:
        chunk = super().read(n)
        time.sleep(len(chunk) * self.byte_s)
        return chunk

    def stop_coil_on_after(self, since: float) -> float | None:
        """When the first FC05 STOP 0x042C = FF00 after ``since`` started on the line (None: not sent)."""
        stop_on = struct.pack(">BHH", 0x05, R.COIL_STOP, 0xFF00)
        return next((t for t, frame in list(self.frames) if t >= since and frame[1:6] == stop_on), None)


class DriverFindingsTest(_ControllerCase):
    """The review findings, reproduced through the real iai_actuator_driver and SimulatedEC."""

    def make(self, **kwargs: Any) -> Controller:
        return super().make(session_factory=open_session, **kwargs)

    def test_stop_with_pio_in_control(self) -> None:
        """Through the real driver: STOP with PMSS = 0 stops the move and leaves PMSL alone.

        Hardware test 19 showed this for a Modbus-commanded move; here the simulator's ``pio_move``
        stands in for a PLC-commanded move, which is untested on hardware.
        """
        ctl = self.make(stop_confirm_s=0.5)
        with simulated_ec(pmsl=False) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        self.assertTrue(sim.pio_move(140.0, 20.0))  # stands in for a move a PLC started over PIO
        ctl.stop()
        self.assertFalse(sim.moving)
        self.assertLess(sim.position_mm, 100.0)
        self.assertFalse(sim.pmsl, "STOP must not switch PMSL")
        self.assertNotIn("pmsl on", sim.events)
        self.assertFalse(sim.coil_state(R.COIL_STOP), "STOP coil left ON")
        self.assertTrue(any(m.startswith("STOP sent") for m in messages(ctl)), messages(ctl))
        self.assertFalse(any("ignore" in m for m in messages(ctl)), messages(ctl))
        time.sleep(0.7)
        self.assertIsNone(ctl.snapshot()["stop_alert"])

        ctl.stop(take_control=True)  # at rest now: control is taken after the STOP
        self.assertTrue(sim.pmsl)
        ctl.disconnect()
        self.assertFalse(sim.pmsl, "Modbus control not handed back to PIO")

    def test_restart_after_stop_through_the_driver(self) -> None:
        """Worst case (untested on hardware): the PLC still holds its start input when STOP is released."""
        ctl = self.make(stop_confirm_s=0.5, pio_restart_watch_s=3.0)
        with simulated_ec(pmsl=False) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        self.assertTrue(sim.pio_move(140.0, 20.0, hold_start=True))
        ctl.stop()  # the driver holds STOP until standstill, then releases it: the sim restarts the move
        self.assertIn("pio start input still on: move restarted", sim.events)
        self.assertTrue(wait_until(lambda: (ctl.snapshot()["stop_alert"] or "").startswith("AXIS MOVED AGAIN"), 2),
                        messages(ctl))
        self.assertTrue(any(m.startswith("STOP sent") and "verified only on Modbus-commanded moves" in m
                            for m in messages(ctl)), messages(ctl))
        sim.pio_release()
        ctl.stop()
        self.assertFalse(sim.moving)
        self.assertIsNone(ctl.snapshot()["stop_alert"])

    def test_push_blip_is_not_reported_as_contact(self) -> None:
        """Hardware test G5 through the real driver: a transient contact fingerprint, then a miss."""
        # the unit runs on a stepped clock: its time advances 20 ms with every request, so the blip is seen by about
        # fifteen polls whatever the PC's load (on the wall clock the 50 ms poller had to land inside the ~0.15 s blip
        # by luck); the controller's own timers (the result settle) stay in real time, the run takes about a second
        ctl = self.make(poll_interval_s=0.01)
        with simulated_ec(profile="gripper", position_mm=1.0, push_blip_mm=5.0, push_blip_len_mm=1.5,
                          step_s=0.02) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        session = ctl._session
        seen: list[dict[str, Any]] = []
        real_status = session.status

        def recording_status() -> dict[str, Any]:
            st = real_status()
            seen.append(dict(st))
            return st

        session.status = recording_status  # type: ignore[method-assign]
        ctl.servo(True)
        ctl.push(2.0, 5.0, 30, 1, 10.0)  # G5-like: push + (closing) 30 %, band 5 to 7.00 mm
        self.assertTrue(wait_until(lambda: (ctl.snapshot()["push"] or {}).get("state") not in (None, "running"), 15),
                        ctl.snapshot()["push"])
        push = ctl.snapshot()["push"]
        self.assertGreaterEqual(sum(1 for st in seen if st["push_contact"]), 3, "the poller never saw the blip")
        self.assertEqual(push["state"], "missed", push)
        self.assertAlmostEqual(push["position_mm"], 7.0, delta=0.01)
        self.assertFalse(any("CONTACT" in m for m in messages(ctl)), messages(ctl))
        self.assertFalse(sim.moving)

    def test_push_refused_when_the_unit_cannot_push(self) -> None:
        """MXPP (item 7.9) = 0: the limits report max push 0 % (not the GUI cap) and a push is refused unsent."""

        ctl = self.make()
        with simulated_ec(word_overrides={0x0710: 0, 0x0711: 0}) as sims:  # the slider's MXPP reads 0
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        limits = ctl.snapshot()["limits"]
        self.assertEqual((limits["max_push_pct"], limits["min_push_pct"]), (0.0, 0.0))
        ctl.servo(True)
        before = len(sim.requests)
        with self.assertRaisesRegex(BackendError, "this unit cannot push") as caught:
            ctl.push(80.0, 10.0, 30, 1, 10.0)
        self.assertEqual(caught.exception.http_status, 400)
        writes = [f for f in list(sim.requests)[before:] if f[1] in (0x06, 0x10, 0x17)]
        self.assertEqual(writes, [])
        self.assertFalse(sim.moving)

    def test_slider_push_cap_is_its_mxpp(self) -> None:
        ctl = self.make()
        with simulated_ec():
            ctl.connect("SIMULATED", 115200, 1, True)
        limits = ctl.snapshot()["limits"]
        self.assertEqual((limits["max_push_pct"], limits["min_push_pct"]), (60.0, 20.0))  # MXPP 60 % on the slider

    def test_pmsl_never_written_during_a_pio_move(self) -> None:
        ctl = self.make()
        with simulated_ec(pmsl=False) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        self.assertTrue(sim.pio_move(140.0, 20.0))
        ctl._record_status(ctl._session, ctl._session.status())
        with self.assertRaises(BackendError) as caught:
            ctl.move(100, 20, 0.1)
        self.assertIn("at rest", str(caught.exception))
        self.assertNotIn("pmsl on", sim.events)
        self.assertTrue(sim.moving, "the PIO move was disturbed")
        ctl.stop()

    def test_gripper_profile(self) -> None:
        """2-position gripper: stroke 0-9.50 mm from its soft limits, 53 mm/s cap, ZONS markers."""
        ctl = self.make()
        with simulated_ec(profile="gripper", time_scale=10.0) as sims:
            connected = ctl.connect("SIMULATED", 115200, 1, True)  # no stroke given: the EC's own limits
        sim = sims[0]
        stroke = connected["connection"]["stroke"]
        self.assertIsNone(stroke["configured_mm"])
        self.assertEqual(stroke["ec_soft_limits_mm"], [-0.3, 9.8])
        self.assertEqual(stroke["in_use_mm"], [0.0, 9.5])
        self.assertEqual(stroke["ec_max_speed_mm_s"], 53.0)
        self.assertEqual(stroke["warnings"], [])
        identity = connected["identity"]
        self.assertEqual(identity["model"], "EC-GRBP8M-20-1-1-PN-WA-WL2")
        self.assertEqual(identity["controller"], 'EC 24 V, unit 0x0A (EC(MS) "2-size EC")')  # family name
        self.assertEqual(identity["actuator"], "gripper (AKID 4)")  # the kind comes from AKID (item 7.33)
        self.assertEqual(identity["speed_cap_mm_s"], 53.0)
        limits = ctl.snapshot()["limits"]
        self.assertEqual((limits["stroke_min_mm"], limits["stroke_max_mm"], limits["max_speed_mm_s"]), (0.0, 9.5, 53.0))
        st = ctl.snapshot()["status"]
        self.assertEqual(st["raw"]["DOPM"], 0x0006)  # b2 = *ALM on the gripper, not "Middle"
        self.assertEqual((st["at_backward_end"], st["at_middle"], st["at_forward_end"]), (False, False, True))
        ctl.servo(True)
        for bad in (lambda: ctl.move(5.0, 60.0, 0.1), lambda: ctl.move(9.6, 20.0, 0.1)):
            with self.assertRaises(BackendError) as caught:
                bad()
            self.assertEqual(caught.exception.http_status, 400)
        with self.assertRaises(BackendError) as caught:
            ctl.move(5.0, 60.0, 0.1)
        self.assertIn("53 mm/s (this EC's maximum speed (MAXV, item 7.4))", str(caught.exception))
        self.assertEqual(sim.alarm_code, 0)
        ctl.move(3.0, 50.0, 0.1)
        self.assertTrue(wait_until(lambda: not sim.moving, 5))
        st = ctl._session.status()
        self.assertEqual(st["raw"]["DOPM"], 0x0005)
        self.assertEqual((st["at_backward_end"], st["at_middle"], st["at_forward_end"]), (True, False, False))

    def test_stroke_override_only_narrows(self) -> None:
        ctl = self.make()
        with simulated_ec(profile="gripper"):
            ctl.connect("SIMULATED", 115200, 1, True, stroke=(0.0, 300.0))
        self.assertEqual(ctl.snapshot()["connection"]["stroke"]["in_use_mm"], [0.0, 9.5])
        self.assertEqual(ctl._session._ec.effective_stroke, (0.0, 9.5))
        ctl.disconnect()
        with simulated_ec(profile="gripper"):
            ctl.connect("SIMULATED", 115200, 1, True, stroke=(1.0, 5.0))
        self.assertEqual(ctl.snapshot()["connection"]["stroke"]["in_use_mm"], [1.0, 5.0])
        self.assertEqual(ctl._session._ec.effective_stroke, (1.0, 5.0))

    def test_unknown_soft_limits_without_override_block_motion(self) -> None:
        ctl = self.make()
        with simulated_ec(soft_limits=False):
            ctl.connect("SIMULATED", 115200, 1, True)
        snap = ctl.snapshot()
        self.assertIn("Motion disabled", snap["motion_block"])
        self.assertIsNone(snap["connection"]["stroke"]["in_use_mm"])
        self.assertIsNone(snap["limits"]["stroke_max_mm"])
        with self.assertRaises(BackendError):
            ctl.move(80, 20, 0.1)

    def test_stroke_from_ec_soft_limits(self) -> None:
        ctl = self.make()
        with simulated_ec(stroke_mm=50.0, soft_limits=True) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)  # server default 0-150
        sim = sims[0]
        snap = ctl.snapshot()
        self.assertEqual(snap["limits"]["stroke_max_mm"], 50.0)
        self.assertEqual(snap["connection"]["stroke"]["ec_soft_limits_mm"], [-0.3, 50.3])
        self.assertEqual(snap["connection"]["stroke"]["ec_stroke_mm"], [0.0, 50.0])
        self.assertEqual(ctl._session._ec.effective_stroke, (0.0, 50.0), "driver limits differ from the GUI's")
        with self.assertRaises(BackendError) as caught:
            ctl.move(50.2, 50, 0.3)  # inside the EC's soft limit, outside its stroke: the GUI and driver agree
        self.assertEqual(caught.exception.http_status, 400)
        with self.assertRaises(BackendError) as caught:
            ctl.move(120, 100, 0.3)
        self.assertEqual(caught.exception.http_status, 400)
        self.assertEqual(sim.alarm_code, 0)
        ctl.move(45, 50, 0.3)
        self.assertTrue(wait_until(lambda: not sim.moving and abs(sim.position_mm - 45) < 0.1, 5))

    def test_stroke_from_connect_form(self) -> None:
        ctl = self.make()
        with simulated_ec(stroke_mm=50.0, soft_limits=False) as sims:
            ctl.connect("SIMULATED", 115200, 1, True, stroke=(0.0, 50.0))
        stroke = ctl.snapshot()["connection"]["stroke"]
        self.assertIn("not readable", stroke["ec_soft_limits_note"])
        self.assertEqual(stroke["in_use_mm"], [0.0, 50.0])
        with self.assertRaises(BackendError):
            ctl.move(120, 100, 0.3)
        self.assertEqual(sims[0].alarm_code, 0)

    def test_disconnect_during_home_with_stale_cache(self) -> None:
        ctl = self.make(poll_interval_s=60)
        with simulated_ec() as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        sim = sims[0]
        ctl.servo(True)
        ctl.move(10, 100, 0.3)
        self.assertTrue(wait_until(lambda: not sim.moving, 5))
        ctl._record_status(ctl._session, ctl._session.status())  # cache: idle at 10 mm
        ctl._refresh_after_command = lambda session: None  # imitate a lost post-command read
        ctl.home()
        self.assertTrue(sim.coil_state(R.COIL_HOME))
        self.assertTrue(sim.moving)
        self.assertFalse(ctl.snapshot()["status"]["moving"], "cache should be stale for this test")
        ctl.disconnect()
        self.assertFalse(sim.coil_state(R.COIL_HOME), "HOME coil left raised after disconnect")
        self.assertFalse(sim.moving, "axis still homing after disconnect")
        self.assertTrue(any(m.startswith("STOP sent before disconnect") for m in messages(ctl)))
        self.assertIn("Axis stopped before disconnect", messages(ctl))

    def test_session_close_releases_home_coil(self) -> None:
        with simulated_ec() as sims:
            session = open_session("SIMULATED", 115200, 1, True)
        sim = sims[0]
        session.home()
        self.assertTrue(sim.coil_state(R.COIL_HOME))
        session.close()
        self.assertFalse(sim.coil_state(R.COIL_HOME))
        self.assertTrue(sim.closed)


class DriverSimulatorTest(_ServerCase):
    """Smoke test through the real iai_actuator_driver package and its SimulatedEC (simulate=True only)."""

    session_factory = staticmethod(open_session)

    def sim(self) -> SimulatedEC:
        return self.ctl._session._bus.transport  # white-box: the SimulatedEC behind ModbusRTU

    def test_simulator_session(self) -> None:
        connected = self.connect_sim()
        self.assertIn("iai_actuator_driver", connected["connection"]["driver"])
        s = self.status()
        self.assertTrue(s["comm_ok"], s)
        self.assertIsInstance(s["status"]["position_mm"], (int, float))
        self.ok("/api/identify")
        rows = self.ok("/api/positions")["rows"]
        self.assertEqual(len(rows), 3)
        self.ok("/api/servo", {"on": True})
        self.assertTrue(wait_until(lambda: self.status()["status"]["servo_on"] is True))
        if self.status()["status"].get("homed") is False:
            self.ok("/api/home", {})
            self.assertTrue(wait_until(lambda: self.status()["status"]["homed"] is True, 30))
        self.wait_idle()
        self.ok("/api/move", {"position_mm": 60, "speed_mm_s": 50, "accel_g": 0.1})
        self.wait_at(60.0, 15)
        self.ok("/api/jog", {"delta_mm": 1, "speed_mm_s": 20})
        self.wait_at(61.0, 10)
        self.ok("/api/stop", {})
        self.ok("/api/reset_alarm", {})
        self.ok("/api/servo", {"on": False})
        self.ok("/api/disconnect", {})

    def test_message_level_alarm_is_a_warning_not_a_fault(self) -> None:
        """Alarm levels: 0x011-0x07F are messages (the axis keeps running): warning lamp, motion still allowed."""
        self.connect_sim()
        sim = self.sim()
        sim.trigger_alarm(0x04E)  # "Total moving count is exceeded threshold." (maintenance warning)
        self.assertTrue(wait_until(lambda: self.status()["status"]["alarm_code"] == 0x04E))
        st = self.status()["status"]
        self.assertEqual((st["alarm"], st["warning"], st["alarm_level"]), (False, True, "message"))
        self.ok("/api/move", {"position_mm": 60, "speed_mm_s": 50, "accel_g": 0.1})  # not blocked by a warning
        self.wait_at(60.0, 15)
        live = self.ok("/api/unit_live")["live"]
        self.assertEqual((live["io"]["lamps"]["alarm"], live["io"]["lamps"]["warning"]), (False, True))
        self.assertEqual((live["alarm"]["level"], live["alarm"]["fault"]), ("message", False))
        sim.trigger_alarm(0x0C1)  # "Servo error": operation cancelled -> a fault (red lamp, motion refused)
        self.assertTrue(wait_until(lambda: self.status()["status"]["alarm"] is True))
        self.assertEqual(self.call("/api/move", {"position_mm": 50, "speed_mm_s": 50, "accel_g": 0.1})[0], 409)

    def test_identity_shows_family_and_akid(self) -> None:
        self.connect_sim("rod")
        identity = self.ok("/api/identify")["identity"]
        self.assertEqual(identity["controller"], "EC 24 V, unit 0x00 (EC standard)")
        self.assertEqual((identity["actuator"], identity["axis_number"], identity["unit_known"]), ("rod (AKID 3)", 0, True))
        self.assertEqual((identity["ec_max_speed_mm_s"], identity["ec_max_accel_g"]), (800.0, 1.0))
        self.assertEqual((identity["push_cap_pct"], identity["push_min_recommended_pct"]), (70.0, 20.0))
        limits = self.status()["limits"]
        self.assertEqual((limits["max_push_pct"], limits["max_accel_g"]), (70.0, 0.3))

    def test_home_releases_home_coil(self) -> None:
        """home() is sent without waiting, so the adapter must release the edge-triggered HOME coil."""
        self.connect_sim()
        sim = self.sim()
        self.ok("/api/servo", {"on": True})
        self.assertTrue(wait_until(lambda: self.status()["status"]["servo_on"] is True))
        self.ok("/api/move", {"position_mm": 10, "speed_mm_s": 100, "accel_g": 0.3})
        self.wait_at(10.0)
        self.ok("/api/home", {})
        self.assertTrue(sim.coil_state(R.COIL_HOME), "HOME coil not raised")
        self.wait_at(0.0, 15)
        self.assertTrue(self.status()["status"]["homed"])
        self.assertTrue(wait_until(lambda: sim.coil_state(R.COIL_HOME) is False, 3), "HOME coil left on after homing")

        # STOP during a home return releases it too.
        self.ok("/api/move", {"position_mm": 30, "speed_mm_s": 100, "accel_g": 0.3})
        self.wait_at(30.0)
        self.ok("/api/home", {})
        time.sleep(0.3)
        self.ok("/api/stop", {})
        self.assertFalse(sim.coil_state(R.COIL_HOME), "HOME coil left on after STOP during homing")


    def test_status_flags_and_position_markers(self) -> None:
        self.connect_sim()
        st = self.status()["status"]
        for key in ("safety_speed", "paused", "pushing", "homing", "target_reached", "push_contact", "push_missed",
                    "at_backward_end", "at_middle", "at_forward_end"):
            self.assertIs(st[key], False, key)
        self.assertIn("DOPM", st["raw"])
        self.assertEqual(st["raw"]["ZONS"], 0x0003)
        self.ok("/api/servo", {"on": True})
        # Lamps follow ZONS b12 / b13 / b14 (hardware test 20 words).
        for target, lamp, zons in ((75.0, "at_middle", 0x4003), (0.0, "at_backward_end", 0x1003),
                                   (150.0, "at_forward_end", 0x2003)):
            self.ok("/api/move", {"position_mm": target, "speed_mm_s": 100, "accel_g": 0.3})
            self.wait_at(target, 15)
            self.assertTrue(wait_until(lambda: self.status()["status"][lamp] is True, 2), lamp)
            st = self.status()["status"]
            self.assertEqual(st["raw"]["ZONS"], zons)
            self.assertEqual([k for k in ("at_backward_end", "at_middle", "at_forward_end") if st[k]], [lamp])
            self.assertTrue(st["target_reached"])
        self.sim().safety_speed = True
        self.assertTrue(wait_until(lambda: self.status()["status"]["safety_speed"] is True, 2))
        self.assertTrue(any("Safety speed ON" in e["msg"] for e in self.status()["events"]))

    def test_stop_mid_move_uses_stop_coil_and_jog_never_uses_jog_coils(self) -> None:
        self.connect_sim()
        sim = self.sim()
        self.ok("/api/servo", {"on": True})
        self.ok("/api/move", {"position_mm": 80, "speed_mm_s": 20, "accel_g": 0.1})
        self.wait_at(80.0, 10)
        self.ok("/api/jog", {"delta_mm": 5, "speed_mm_s": 20})
        self.wait_at(85.0, 10)
        self.ok("/api/move", {"position_mm": 140, "speed_mm_s": 20, "accel_g": 0.1})
        self.assertTrue(wait_until(lambda: self.status()["status"]["position_mm"] > 87, 5))
        self.ok("/api/stop", {})
        stopped = self.wait_idle()["position_mm"]
        self.assertTrue(87 < stopped < 100, stopped)
        self.assertFalse(sim.coil_state(R.COIL_STOP), "STOP coil left ON after the driver's stop sequence")
        self.assertIn("stop", sim.events)
        self.assertIn("stop released", sim.events)
        coils = {int.from_bytes(f[2:4], "big") for f in sim.requests if f[1] == 0x05}
        self.assertFalse(coils & {R.COIL_JOG_PLUS, R.COIL_JOG_MINUS}, "JOG coils used")
        self.ok("/api/move", {"position_mm": 90, "speed_mm_s": 20})  # STOP released: the next move runs
        self.wait_at(90.0, 10)

    def test_push_contact_and_missed(self) -> None:
        self.connect_sim()
        sim = self.sim()
        self.ok("/api/servo", {"on": True})
        self.ok("/api/move", {"position_mm": 60, "speed_mm_s": 100, "accel_g": 0.3})
        self.wait_at(60.0, 10)
        push = {"target_mm": 70, "band_mm": 10, "force_pct": 30, "direction": 1, "speed_mm_s": 50, "accel_g": 0.3}
        self.ok("/api/push", push)
        self.assertTrue(wait_until(lambda: (self.status()["push"] or {}).get("state") == "missed", 10),
                        self.status()["push"])
        self.assertTrue(self.status()["status"]["push_missed"])
        self.assertAlmostEqual(self.status()["push"]["position_mm"], 80.0, delta=0.15)
        # From inside the push zone the EC would skip the approach: the driver refuses (409).
        code, body = self.call("/api/push", push)
        self.assertEqual(code, 409, body)
        self.assertIn("push zone", body["error"])

        self.ok("/api/move", {"position_mm": 60, "speed_mm_s": 100, "accel_g": 0.3})
        self.wait_at(60.0, 10)
        sim.obstacle_mm = 74.0
        self.ok("/api/push", push)
        self.assertTrue(wait_until(lambda: (self.status()["push"] or {}).get("state") == "contact", 10),
                        self.status()["push"])
        st = self.status()["status"]
        self.assertTrue(st["push_contact"] and st["pushing"], st)
        self.assertAlmostEqual(st["position_mm"], 74.0, delta=0.15)
        self.ok("/api/move", {"position_mm": 60, "speed_mm_s": 50})  # retract releases the push
        self.wait_at(60.0, 10)
        self.assertFalse(self.status()["status"]["pushing"])

    def test_identity_and_soft_limits_shown(self) -> None:
        connected = self.connect_sim()
        stroke = connected["connection"]["stroke"]
        self.assertEqual(stroke["ec_soft_limits_mm"], [-0.3, 150.3])
        self.assertEqual(stroke["ec_stroke_mm"], [0.0, 150.0])
        self.assertIsNone(connected["connection"]["baud"])  # simulator: no serial port, no baud rate
        identity = self.ok("/api/identify")["identity"]
        self.assertEqual(identity["soft_limits_mm"], [-0.3, 150.3])
        self.assertEqual(identity["model"], "MWB-EC-DSRR4-WA-MF")
        self.assertEqual(identity["baud"], "simulated")


READ_ONLY_FUNCTIONS = {0x03, 0x11}
ALL_SECTIONS = ["identity", "parameters", "monitor", "alarms", "maintenance", "positions"]


def function_codes(sim: SimulatedEC) -> set[int]:
    """Modbus function codes the simulator has received (FC03 = read registers, FC11 = report slave ID)."""
    return {frame[1] for frame in list(sim.requests)}


class UnitInfoFakeTest(_ControllerCase):
    """Controller side of the Unit info view: grouped reads, merge, errors, live decode (test fake)."""

    def test_groups_are_read_and_merged(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        info = ctl.unit_info()
        self.assertEqual(fake.unit_info_calls, [list(group) for group in UNIT_INFO_GROUPS])
        self.assertEqual(info["meta"]["sections"], ALL_SECTIONS)
        self.assertEqual(info["meta"]["gui"]["groups_read"], len(UNIT_INFO_GROUPS))
        self.assertEqual(info["meta"]["gui"]["bus_lock"], "per request")
        self.assertTrue(info["meta"]["read_only"])
        self.assertEqual(info["identity"]["model"], "FAKE-EC")
        self.assertEqual(info["maintenance"], {"total_moves": 7})
        self.assertIsNone(info["parameters"])
        self.assertEqual(info["errors"], ["positions: the fake has no position data"])
        self.assertEqual(ctl._unit_mode["layout"], "3pos")  # the live view decodes DIPM / DOPM with it
        self.assertTrue(any(m.startswith("Unit info read (read-only, 3 of 3 groups, bus lock per request")
                            for m in messages(ctl)), messages(ctl))
        self.assertEqual(fake.control_writes, 0)
        ctl.disconnect()
        self.assertIsNone(ctl._unit_mode)

    def test_a_later_group_failing_is_listed(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl._session.unit_info_fail_on = "maintenance"
        info = ctl.unit_info()
        self.assertEqual(info["meta"]["sections"], ALL_SECTIONS[:4])
        self.assertEqual(info["meta"]["gui"]["groups_read"], 2)
        self.assertEqual(info["errors"], ["maintenance, positions: TimeoutError: no reply"])
        self.assertEqual(info["identity"]["model"], "FAKE-EC")

    def test_stop_ends_the_read_before_its_next_request(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        real_unit_info = fake.unit_info

        def unit_info(sections: Any) -> dict[str, Any]:
            result = real_unit_info(sections)
            if "monitor" in sections:
                ctl.stop()  # pressed during the second group (between two requests: the bus is free)
            return result

        fake.unit_info = unit_info
        info = ctl.unit_info()
        self.assertEqual(info["meta"]["gui"]["groups_read"], 2)
        self.assertEqual(info["errors"][-1],
                         "maintenance, positions: Unit info interrupted by STOP (nothing more was read). Try again.")
        self.assertTrue(any("interrupted by STOP after 2 of 3 groups" in m for m in messages(ctl)), messages(ctl))
        fake.unit_info = real_unit_info
        info = ctl.unit_info()  # a new Refresh after STOP reads everything again
        self.assertEqual(info["meta"]["gui"]["groups_read"], 3)
        fake.unit_info = lambda sections: (ctl.stop(), real_unit_info(sections))[1]  # STOP in the first group
        with self.assertRaises(ReadInterrupted) as caught:
            ctl.unit_info()
        self.assertEqual(caught.exception.http_status, 409)

    def test_first_group_failing_is_an_error(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        ctl._session.unit_info_fail_on = "identity"
        with self.assertRaises(BackendError) as caught:
            ctl.unit_info()
        self.assertEqual(caught.exception.http_status, 502)
        self.assertIn("Unit info failed: TimeoutError: no reply", str(caught.exception))

    def test_not_connected(self) -> None:
        ctl = self.make()
        for call in (ctl.unit_info, ctl.unit_live, lambda: ctl.read_block(7)):
            with self.assertRaises(BackendError) as caught:
                call()
            self.assertEqual(caught.exception.http_status, 409)

    def test_live_without_status_words(self) -> None:
        ctl = self.make()
        ctl.connect("SIMULATED", 115200, 1, True)
        live = ctl.unit_live()
        self.assertIsNone(live["live"])  # the fake's status has no raw words
        self.assertIsInstance(live["status_age_s"], float)
        with self.assertRaises(BackendError) as caught:
            ctl.read_block(7)
        self.assertEqual(caught.exception.http_status, 404)


class UnitInfoDriverTest(_ServerCase):
    """The Unit info view's endpoints through the real iai_actuator_driver and SimulatedEC, both profiles. Read-only."""

    session_factory = staticmethod(open_session)

    def sim(self) -> SimulatedEC:
        return self.ctl._session._bus.transport  # white-box: the SimulatedEC behind ModbusRTU

    def live(self) -> dict[str, Any]:
        return self.ok("/api/unit_live")["live"]

    def assert_read_only(self, sim: SimulatedEC) -> None:
        codes = function_codes(sim)
        self.assertIn(0x03, codes)
        self.assertLessEqual(codes, READ_ONLY_FUNCTIONS, "the Unit info view sent something other than FC03 / FC11")

    def test_slider(self) -> None:
        connected = self.connect_sim()
        self.assertEqual(connected["connection"]["sim_profile"], "slider")
        sim = self.sim()
        live = self.live()  # before any unit info read: the layout comes from the model string (option MF)
        self.assertEqual((live["io"]["layout"], live["position_mode"]["source"]), ("3pos", "model string option MF"))
        self.assertEqual(self.ok("/api/unit_live")["mode_source"], "model string / assumed (Refresh to read P9)")
        info = self.ok("/api/unit_info")["info"]
        self.assertEqual(info["errors"], [])
        meta = info["meta"]
        self.assertEqual((meta["schema"], meta["read_only"], meta["function_codes"]),
                         ("ec_driver.unit_info/1", True, ["03", "11"]))
        self.assertEqual(meta["sections"], ALL_SECTIONS)
        self.assertEqual(meta["gui"]["groups_read"], 3)
        ident = info["identity"]
        self.assertEqual((ident["model"], ident["serial_number"], ident["firmware"]),
                         ("MWB-EC-DSRR4-WA-MF", "SIM-0000001", "0013"))
        self.assertEqual(ident["position_mode"]["layout"], "3pos")
        self.assertEqual(ident["directory"]["record_count"], 27)
        named = {p["no"]: p["text"] for p in info["parameters"]["named"]}
        self.assertEqual((named[1], named[2], named[9]), ("150.00 mm", "0.10 mm", "4:3-position"))
        self.assertEqual([c["class"] for c in info["parameters"]["classes"]],
                         ["0x0003", "0x0004", "0x0007", "0x0008", "0x0009", "0x000C"])
        self.assertEqual([b["name"] for b in info["io"]["DOPM"]["bits"]], ["*ALM", "LS2", "LS1", "LS0"])
        self.assertLessEqual({"DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SSSE", "SIPM", "POSS"},
                             set(info["status_words"]))
        self.assertEqual(set(info["control_words"]) & {"DRG1", "DRG2", "DRGE"}, {"DRG1", "DRG2", "DRGE"})
        self.assertIn("uptime", info["monitor"])
        alarms = info["alarms"]
        self.assertEqual(alarms["slots"], 32)
        self.assertTrue(alarms["history"])
        self.assertTrue(all(h["name"] and not h["name"].startswith("alarm 0x") for h in alarms["history"]))
        self.assertEqual(alarms["current"]["code"], 0)
        self.assertIsInstance(info["maintenance"]["total_moves"], int)
        positions = info["positions"]
        self.assertEqual([r["position_mm"] for r in positions["rows"]], [0.0, 150.0, 75.0])
        self.assertEqual([r["n"] for r in positions["simple_data_rows"]], [260, 261, 262, 263, 266])  # 266 = Middle %
        self.assertEqual(positions["direct_move"]["address"], "0x9900")
        self.assertEqual(positions["active_profile"]["address"], "0x0F00")
        json.dumps(info)  # the Save JSON button relies on this

        # Live: the poller's status words decoded with the unit info's (3-position) layout.
        live = self.live()
        raw = self.status()["status"]["raw"]
        self.assertEqual(live["status_words"]["DSS1"]["hex"], f"0x{raw['DSS1']:04X}")
        self.assertEqual(live["io"]["layout"], "3pos")
        self.assertEqual(live["alarm"]["name"], "No alarm (in normal condition)")
        self.assertIn("SSSE", live["status_words"])  # in the 22-word status poll
        self.assert_read_only(sim)

    def test_gripper(self) -> None:
        connected = self.connect_sim("gripper")
        self.assertEqual(connected["connection"]["sim_profile"], "gripper")
        self.assertIn("gripper profile", connected["connection"]["driver"])
        sim = self.sim()
        live = self.live()  # before any unit info read: no MF option in the model string, 2 positions assumed
        self.assertEqual((live["io"]["layout"], live["position_mode"]["source"]), ("2pos", "assumed"))
        info = self.ok("/api/unit_info")["info"]
        self.assertEqual(info["errors"], [])
        ident = info["identity"]
        self.assertEqual(ident["model"], "EC-GRBP8M-20-1-1-PN-WA-WL2")
        self.assertEqual(ident["position_mode"]["layout"], "2pos")
        self.assertEqual(ident["directory"]["record_count"], 25)
        # The gripper's baseline (the values IA-OS showed for it; VERIFIED).
        listed = {p["no"]: p["text"] for p in info["parameters"]["named"] if p["listed"]}
        self.assertEqual(listed, {
            1: "9.50 mm", 2: "0.10 mm", 3: "1:Forward", 4: "0.50 mm", 5: "0:Disable", 6: "1:Enable", 7: "0:Enable",
            9: "0:Double", 10: "0:Disable", 12: "0:Continuous energization"})  # = the list IA-OS shows on the gripper
        self.assertEqual([b["name"] for b in info["io"]["DOPM"]["bits"]], ["*ALM", "LS1", "LS0"])
        self.assertEqual(info["positions"]["rows"][2]["role"], "unused in 2-position mode")
        live = self.live()
        self.assertEqual(live["io"]["layout"], "2pos")  # now from user parameter 9
        self.assertEqual(live["position_mode"]["source"], "user parameter 9 = 0")
        self.assertEqual(self.ok("/api/read_block?class=0x0007&count=2")["block"]["words"], [0x0000, 0x03D4])  # 9.80 mm
        self.assert_read_only(sim)

    def test_register_browser(self) -> None:
        self.connect_sim()
        sim = self.sim()
        block = self.ok("/api/read_block?class=0x0007")["block"]
        self.assertEqual((block["class"], block["address"], block["offset"]), ("0x0007", 0x0700, 0))
        self.assertEqual(block["count"], block["block_words"])
        self.assertEqual(block["words"][:4], [0x0000, 0x3AB6, 0xFFFF, 0xFFE2])  # +150.30 / -0.30 mm
        self.assertTrue(block["read_only"])
        row1 = self.ok("/api/read_block?class=16&offset=16&count=16")["block"]  # position table row 1, decimal class
        self.assertEqual((row1["address"], row1["words"][1]), (0x1010, 0x3A98))  # 150.00 mm
        self.assertEqual(self.ok("/api/read_block?class=0x0010")["block"]["count"], backend.BROWSE_DEFAULT_WORDS)
        # Compact monitor 0x9F00-0x9F27 (read live on the slider): 9F18 OLLV, 9F1A-1B TLMC, 9F1C-1D ODOM.
        compact = self.ok("/api/read_block?class=0x0114&offset=24&count=6")["block"]
        self.assertEqual((compact["address"], compact["words"]), (0x9F18, [0x0012, 0, 0, 0x0065, 0, 0x0002]))
        self.assertEqual((compact["exact"], compact["block_words"]), (True, 40))
        # A block of uncertain size is readable to its largest extent; the tail is flagged (exact False).
        tail = self.ok("/api/read_block?class=0x0109&offset=15&count=5")["block"]
        self.assertEqual((tail["address"], tail["words"]), (0x910F, [0x1A10, 0x292A, 0x2B10, 0x0000, 0x8008]))
        self.assertEqual((tail["exact"], tail["block_words"], tail["words_readable"]), (False, 15, 30))
        self.assertEqual(self.ok("/api/read_block?class=0x0112&offset=20&count=2")["block"]["words"], [0x1402, 0x0544])
        self.assertEqual(self.ok("/api/read_block?class=0x0109")["block"]["count"], 30)  # default: all readable
        for query, status, text in (
            ("class=0x7777", 404, "not in this EC's directory"),
            ("", 400, "Missing query parameter: class"),
            ("class=zz", 400, "whole number"),
            ("class=0x0007&count=0", 400, "outside it"),
            ("class=0x0010&count=513", 400, "at most 512"),
            ("class=0x0007&offset=60000", 400, "outside it"),
            ("class=0x0007&offset=-1", 400, "must be 0-65535"),
            ("class=0x0109&offset=29&count=2", 400, "30 at most (size uncertain)"),
            ("class=0x0114&offset=40&count=1", 400, "outside it"),
        ):
            code, body = self.call("/api/read_block?" + query)
            self.assertEqual(code, status, (query, body))
            self.assertIn(text, body["error"], query)
        self.assert_read_only(sim)

    def test_unit_info_endpoints_are_get_only_and_need_a_connection(self) -> None:
        for path in ("/api/unit_info", "/api/unit_live", "/api/read_block?class=0x0007"):
            self.assertEqual(self.call(path)[0], 409, path)
            self.assertEqual(self.call(path.split("?")[0], {})[0], 404, path)  # no POST route: never a command
            self.assertEqual(self.call(path, token="wrong")[0], 403, path)

    def test_live_sections_follow_the_poll(self) -> None:
        with simulated_ec(pmsl=False, time_scale=10.0) as sims:
            self.connect_sim()
        sim = sims[0]
        self.ok("/api/unit_info")
        self.assertTrue(sim.pio_move(75.0, 50.0))  # a PLC move: a simulator hook, nothing sent by the GUI

        def at_middle() -> bool:
            live = self.live()
            return bool(live and live["io"]["lamps"].get("middle") and not sim.moving)

        self.assertTrue(wait_until(at_middle, 5), self.live())
        live = self.live()
        ls2 = {b["name"]: b["on"] for b in live["status_words"]["ZONS"]["bits"]}
        self.assertTrue(ls2["LS2"] and not ls2["LS0"] and not ls2["LS1"], ls2)
        dopm = {b["name"]: b["on"] for b in live["io"]["DOPM"]["bits"]}
        self.assertTrue(dopm["LS2"], dopm)  # 3-position layout: b2 = LS2
        self.assertAlmostEqual(live["monitor"]["position_mm"], 75.0, delta=0.15)
        sim.safety_speed = True
        self.assertTrue(wait_until(lambda: "SFTY" in self.live()["status_words"]["DSS1"]["on"], 2))
        self.assert_read_only(sim)

    def test_bad_sim_profile(self) -> None:
        for bad in ("bogus", 5, None):
            code, body = self.call("/api/connect", {"simulate": True, "sim_profile": bad})
            self.assertEqual(code, 400, (bad, body))
        self.assertFalse(self.status()["connected"])

    def test_page_keeps_stop_visible_and_the_view_read_only(self) -> None:
        with urllib.request.urlopen(self.base + "/classic") as r:
            html = r.read().decode()
        for marker in ('id="tabInfo"', 'id="viewInfo"', 'id="infoRefresh"', 'id="browseForm"', "/unitinfo.js",
                       'id="simProfile"'):
            self.assertIn(marker, html)
        header = html[html.index('<header class="appbar">'):html.index("</header>")]
        self.assertIn('id="stopBtn"', header)  # STOP lives in the sticky header, outside both views
        self.assertLess(html.index('id="stopBtn"'), html.index('id="viewOperate"'))
        with urllib.request.urlopen(self.base + "/unitinfo.js") as r:
            js = r.read().decode()
        calls = re.findall(r"\bapi\(([^()]*)\)", js)
        self.assertGreaterEqual(len(calls), 3, calls)
        for call in calls:
            self.assertNotIn(",", call, f"unitinfo.js sends a request body (a command?): api({call})")
        self.assertEqual(sorted({re.search(r"/api/\w+", c).group(0) for c in calls}),
                         ["/api/read_block", "/api/unit_info", "/api/unit_live"])
        for forbidden in ("fetch(", "XMLHttpRequest", "POST", "sendBeacon", "/api/stop", "/api/move"):
            self.assertNotIn(forbidden, js)


class StopPreemptsReadsTest(_ControllerCase):
    """Review findings: STOP must preempt the read-only Unit info / register-browser reads within ~50 ms, and a
    STOP pressed during a Refresh must still catch a PLC restarting the axis.

    Real driver + SimulatedEC with serial byte timing at 38400 baud (the EC's power-up rate). With the bus lock
    held per group of sections, STOP's coil went out 250-400 ms after the button and the restart watch was
    defeated; now the lock is taken per (short) request and STOP ends the read.
    """

    BAUD = 38400
    # One in-flight 32-word request at 38400 baud is ~25 ms including the bus gap, so 50 ms locally. A shared CI
    # runner (environment variable CI set) can stall a thread for longer than that: 150 ms there, which still fails
    # the old behaviour (STOP went out 250-400 ms after the button).
    STOP_LATENCY_S = 0.15 if os.environ.get("CI", "").strip().lower() not in ("", "0", "false") else 0.05

    def make(self, **kwargs: Any) -> Controller:
        return super().make(session_factory=open_session, **kwargs)

    def connect_timed(self, ctl: Controller, **options: Any) -> SerialTimingSim:
        with simulated_ec(sim_class=SerialTimingSim, baud=self.BAUD, **options) as sims:
            ctl.connect("SIMULATED", 115200, 1, True)
        return sims[0]

    def stop_during(self, ctl: Controller, sim: SerialTimingSim, read: Callable[[], Any],
                    delay_s: float) -> tuple[float, dict[str, Any]]:
        """Run ``read`` in a thread, press STOP ``delay_s`` into it; returns (button -> STOP coil time, outcome)."""
        outcome: dict[str, Any] = {}

        def run() -> None:
            try:
                outcome["result"] = read()
            except BackendError as exc:
                outcome["error"] = exc

        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        time.sleep(delay_s)
        self.assertTrue(worker.is_alive(), "the read ended before STOP: nothing to preempt")
        pressed = time.monotonic()
        ctl.stop()
        coil = sim.stop_coil_on_after(pressed)
        worker.join(10)
        self.assertFalse(worker.is_alive(), "the read did not end")
        self.assertIsNotNone(coil, "the STOP coil was never written")
        assert coil is not None
        return coil - pressed, outcome

    def test_stop_coil_is_written_quickly_during_unit_info_and_register_reads(self) -> None:
        ctl = self.make(stop_confirm_s=1.0)
        sim = self.connect_timed(ctl)
        for label, read in (("Unit info", ctl.unit_info), ("Read registers", lambda: ctl.read_block(0x0010, 0, 512))):
            latency, outcome = self.stop_during(ctl, sim, read, 0.15)
            self.assertLess(latency, self.STOP_LATENCY_S, f"STOP during '{label}' waited {latency * 1000:.0f} ms")
            error = outcome.get("error")
            self.assertIsInstance(error, ReadInterrupted, outcome)
            self.assertIn(f"{label} interrupted by STOP", str(error))
        self.assertIsNone(ctl.snapshot()["stop_alert"], messages(ctl))
        self.assertEqual(ctl.read_block(0x0114, 24, 6)["words"], [0x0012, 0, 0, 0x0065, 0, 0x0002])  # reads work again

    def test_poll_and_commands_get_the_bus_during_a_refresh(self) -> None:
        ctl = self.make()
        self.connect_timed(ctl)
        outcome: dict[str, Any] = {}
        worker = threading.Thread(target=lambda: outcome.update(info=ctl.unit_info()), daemon=True)
        worker.start()
        time.sleep(0.3)
        started = time.monotonic()
        ctl.servo(True)  # a command clicked during a Refresh runs at once, not after the read
        self.assertLess(time.monotonic() - started, 0.3)
        ages = []
        while worker.is_alive():
            ages.append(ctl.snapshot()["status_age_s"])
            time.sleep(0.1)
        worker.join()
        self.assertEqual(outcome["info"]["errors"], [])
        self.assertGreater(len(ages), 5)
        self.assertLess(max(ages), 0.5, "the status poll stalled during the Refresh")

    def test_restart_after_stop_during_refresh_is_alerted(self) -> None:
        """Worst case (untested on hardware): a PLC holding its start input restarts the axis once STOP is released."""
        ctl = self.make(stop_confirm_s=1.0, pio_restart_watch_s=3.0)
        sim = self.connect_timed(ctl, pmsl=False)
        self.assertTrue(sim.pio_move(sim.position_mm + 20.0, 50.0, hold_start=True))
        latency, outcome = self.stop_during(ctl, sim, ctl.unit_info, 0.05)
        self.assertLess(latency, self.STOP_LATENCY_S)
        self.assertIsInstance(outcome.get("error"), ReadInterrupted, outcome)
        self.assertIn("pio start input still on: move restarted", sim.events)
        self.assertTrue(wait_until(lambda: (ctl.snapshot()["stop_alert"] or "").startswith("AXIS MOVED AGAIN"), 3),
                        messages(ctl))
        sim.pio_release()
        ctl.stop()
        self.assertFalse(sim.moving)


class StopWithoutBusLockTest(_ControllerCase):
    """STOP that cannot get the bus lock still arms the restart watch with the standstill its own hold read."""

    def test_restart_watch_armed_from_the_stop_hold(self) -> None:
        ctl = self.make(stop_confirm_s=2.0, pio_restart_watch_s=3.0)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        fake.pio_move(80.0, 15.0, hold_start=True)  # 73.95 -> 80 mm takes 0.4 s; STOP lands after 0.25 s
        ctl._driver_lock.acquire()  # another command holds the bus: STOP is sent without the lock
        try:
            ctl.stop()
            watch = ctl._restart_watch
            self.assertIsNotNone(watch)
            stopped_at = watch[3]
            self.assertIsNotNone(stopped_at, "restart watch armed without the standstill position")
            self.assertLess(stopped_at, 79.5)
            # The PLC restarts the axis; it is at rest again (80 mm) before the poller can read it.
            self.assertTrue(wait_until(lambda: fake.status()["moving"] is False, 2))
        finally:
            ctl._driver_lock.release()
        self.assertTrue(wait_until(lambda: (ctl.snapshot()["stop_alert"] or "").startswith("AXIS MOVED AGAIN"), 2),
                        messages(ctl))
        self.assertIn(f"(stopped at {stopped_at:.2f} mm)", ctl.snapshot()["stop_alert"])

    def test_status_read_before_the_watch_was_armed_is_ignored(self) -> None:
        ctl = self.make(pio_restart_watch_s=3.0, poll_interval_s=60)
        ctl.connect("SIMULATED", 115200, 1, True)
        fake = ctl._session
        ctl.stop()  # PIO in control: the restart watch is armed
        armed_at = ctl._restart_watch[1]
        moving = {**fake.status(), "moving": True}
        ctl._record_status(fake, dict(moving), armed_at - 0.01)  # began before arming: the motion STOP ended
        self.assertIsNone(ctl.snapshot()["stop_alert"])
        ctl._record_status(fake, dict(moving), armed_at + 0.01)
        self.assertTrue((ctl.snapshot()["stop_alert"] or "").startswith("AXIS MOVED AGAIN"))


class PreemptibleSessionTest(unittest.TestCase):
    """DriverSession.preemptible: short gated requests, retried outside the lock, read-only, this thread only."""

    def test_gated_reads(self) -> None:
        run_off_the_main_thread(self.check_gated_reads)

    def check_gated_reads(self) -> None:
        session = open_session("SIMULATED", 115200, 1, True)
        self.addCleanup(session.close)
        sim = session._bus.transport
        session.identify()  # caches the directory (ungated)
        gated: list[int] = []

        @contextlib.contextmanager
        def gate() -> Iterator[None]:
            gated.append(threading.get_ident())
            yield

        def fc03(since: int) -> list[tuple[int, int]]:
            return [struct.unpack(">HH", f[2:6]) for f in list(sim.requests)[since:] if f[1] == 0x03]

        before = len(sim.requests)
        with session.preemptible(gate):
            block = session.read_block(0x0010, 0, 100)
            other = threading.Thread(target=session.status)  # another thread (poll / STOP) is not gated
            other.start()
            other.join()
        self.assertEqual(block["count"], 100)
        table_reads = [(a, n) for a, n in fc03(before) if 0x1000 <= a < 0x4000]
        self.assertEqual(table_reads, [(0x1000, 32), (0x1020, 32), (0x1040, 32), (0x1060, 4)])
        self.assertEqual(len(gated), 4)  # one gate per request; the other thread's status read used none
        self.assertEqual(set(gated), {threading.get_ident()})
        self.assertIn(R.MONITOR_BASE, [a for a, _ in fc03(before)])

        # A lost reply is retried outside the bus lock: every attempt goes through the gate again.
        gated.clear()
        sim.inject_fault("timeout")
        with session.preemptible(gate):
            self.assertEqual(session.read_block(0x0007, 0, 2)["words"], [0x0000, 0x3AB6])
        self.assertEqual(len(gated), 2)

        # Read-only: a write from inside the context is refused before it is sent (its status read is fine).
        before = len(sim.requests)
        with session.preemptible(gate):
            with self.assertRaises((backend.ReadOnlyViolation, UnsafeCommandError)):  # the core's scope: UnsafeCommandError
                session.servo(True)
        self.assertEqual({f[1] for f in list(sim.requests)[before:]}, {0x03})
        session.servo(True)  # outside the context: a normal command again
        self.assertIn(0x05, {f[1] for f in list(sim.requests)[before:]})

    def test_stop_returns_the_standstill(self) -> None:
        session = open_session("SIMULATED", 115200, 1, True)
        self.addCleanup(session.close)
        final = session.stop()
        self.assertIsInstance(final, dict)
        self.assertIs(final["moving"], False)
        self.assertAlmostEqual(final["position_mm"], session._bus.transport.position_mm, places=2)


def BaudSim(answer_at: int, start: int) -> SimulatedEC:
    """A SimulatedEC on a modelled serial line: the PC side starts at ``start`` baud, the EC only answers at ``answer_at``."""
    return SimulatedEC(baudrate=start, ec_baudrate=answer_at)


class BaudDetectTest(unittest.TestCase):
    """A real-port session finds the EC's baud rate in place (no serial port is opened here)."""

    def open_with(self, transport: SimulatedEC, baud: int) -> Any:
        def fake_bus(port: str, baudrate: int = 115200, **_: Any) -> ModbusRTU:
            self.assertEqual(port, "COM99")
            return ModbusRTU(transport, timeout=0.05, min_gap_s=0.0)

        with mock.patch.object(backend, "ModbusRTU", fake_bus):
            return open_session("COM99", baud, 1, False)

    def test_detects_boot_rate(self) -> None:
        sim = BaudSim(answer_at=38400, start=115200)
        session = self.open_with(sim, 115200)
        self.addCleanup(session.close)
        self.assertEqual(session.baud, 38400)
        self.assertIn("38400 baud (auto-detected", session.description)
        self.assertIsInstance(session.status()["position_mm"], float)

    def test_no_answer_is_a_clear_error(self) -> None:
        sim = BaudSim(answer_at=4800, start=115200)
        with self.assertRaises(BackendError) as caught:
            self.open_with(sim, 115200)
        self.assertEqual(caught.exception.http_status, 502)
        self.assertIn("No answer from slave 1 on COM99", str(caught.exception))
        self.assertTrue(sim.closed, "port not closed after a failed connect")


# =========================================================================== configuration commands


CONFIG_ROUTES = (
    ("/api/run_stored", {"n": 0}), ("/api/param", {"no": 2, "value": 0.2}), ("/api/software_reset", {}),
    ("/api/baud", {"rate": 230400}),
)


class ConfigApiFakeTest(_ServerCase):
    """Run stored / parameter write / software reset / baud endpoints against the test fake."""

    def test_need_a_connection(self) -> None:
        for path, data in CONFIG_ROUTES:
            self.assertEqual(self.call(path, data)[0], 409, path)
            self.assertEqual(self.call(path)[0], 404, path)  # POST only: a GET is never a command

    def test_bad_requests(self) -> None:
        self.connect_sim()
        for path, data in (
            ("/api/run_stored", {}), ("/api/run_stored", {"n": 3}), ("/api/run_stored", {"n": True}),
            ("/api/run_stored", {"n": 1.5}), ("/api/run_stored", {"n": 0, "allow_above_caps": "yes"}),
            ("/api/param", {"value": 1}), ("/api/param", {"no": 2}), ("/api/param", {"no": 0, "value": 1}),
            ("/api/param", {"no": 13, "value": 1}), ("/api/param", {"no": 2, "value": True}),
            ("/api/param", {"no": 2, "value": [1]}), ("/api/param", {"no": 2, "value": "x" * 50}),
            ("/api/baud", {}), ("/api/baud", {"rate": 1234}), ("/api/baud", {"rate": 230400, "method": "bogus"}),
            ("/api/baud", {"rate": 230400, "method": 5}),
            # only 115200, 230400 and 38400 (checked after the switch: the EC-GRBP8M gripper and the PCON-CB take 0x06,
            # the EC-R6S rod ignored it); the other codes are UNVERIFIED
            ("/api/baud", {"rate": 9600, "method": "break"}),
            ("/api/baud", {"rate": 57600}), ("/api/baud", {"rate": 76800}),
            ("/api/param", {"no": 2, "value": 0.2, "allow_unverified_model": "yes"}),
        ):
            code, body = self.call(path, data)
            self.assertEqual(code, 400, (path, data, body))

    def test_flows(self) -> None:
        self.connect_sim()
        self.ok("/api/servo", {"on": True})
        result = self.ok("/api/param", {"no": 2, "value": 0.2})["result"]
        self.assertEqual((result["after"], result["committed"]), ("0.2", True))
        # A home parameter needs no opt-in; the old flag is still accepted (and ignored).
        self.assertEqual(self.ok("/api/param", {"no": 4, "value": 0.6})["result"]["after"], "0.6")
        self.assertEqual(self.ok("/api/param", {"no": 4, "value": 0.7, "allow_home_params": True})["result"]["after"],
                         "0.7")
        events = [e["msg"] for e in self.status()["events"]]
        self.assertTrue(any("takes effect after a software reset" in m for m in events))
        self.ok("/api/param", {"no": 5, "value": 1, "allow_unverified_model": True})
        self.assertEqual(self.ctl._session.param_opt_ins, [False, False, False, True])  # passed through when asked

        self.assertEqual(self.status()["connection"]["line_baud"], 115200)
        self.assertEqual(self.ok("/api/baud", {"rate": 230400, "method": "break"})["result"]["baudrate"], 230400)
        self.assertEqual(self.status()["connection"]["line_baud"], 230400)

        reset = self.ok("/api/software_reset", {})["result"]
        self.assertTrue(reset["needs_home"])
        snap = self.status()
        self.assertTrue(snap["swreset"]["needs_home"])
        self.assertFalse(snap["status"]["homed"])
        self.assertTrue(any("NEEDS HOME" in e["msg"] for e in snap["events"]))
        code, body = self.call("/api/run_stored", {"n": 2})
        self.assertEqual(code, 409)
        self.assertIn("not homed", body["error"])
        self.ok("/api/home", {})
        self.assertTrue(wait_until(lambda: self.status()["status"]["homed"] is True))

        code, body = self.call("/api/run_stored", {"n": 2})  # stored 108.5 mm/s > the fake's 100 mm/s cap
        self.assertEqual(code, 400)
        self.ok("/api/run_stored", {"n": 2, "allow_above_caps": True})
        self.wait_at(75.0)
        self.ok("/api/move", {"position_mm": 100, "speed_mm_s": 20})
        self.assertTrue(wait_until(lambda: self.status()["status"]["moving"] is True))
        for path, data in CONFIG_ROUTES[1:]:
            code, body = self.call(path, data)  # configuration commands only at rest
            self.assertEqual(code, 409, path)
            self.assertIn("only at rest", body["error"])
        self.ok("/api/stop", {})
        self.wait_idle()

    def test_motion_block_also_blocks_run_stored(self) -> None:
        with mock.patch.object(FakeAxis, "ec_limits", lambda self: (None, "not readable (test)")):
            self.connect_sim()
        code, body = self.call("/api/run_stored", {"n": 0})
        self.assertEqual(code, 409)
        self.assertIn("Motion disabled", body["error"])

    def test_page_has_the_controls_and_asks_first(self) -> None:
        with urllib.request.urlopen(self.base + "/classic") as r:
            html = r.read().decode()
        for marker in ('id="swResetBtn"', 'id="baudForm"', 'id="newBaud"', 'id="baudMethod"', 'id="storedMoves"',
                       'id="resetInfo"', "/params.js"):
            self.assertIn(marker, html)
        with urllib.request.urlopen(self.base + "/app.js") as r:
            app = r.read().decode()
        for path in ("/api/software_reset", "/api/baud", "/api/run_stored"):
            call = app.index(f'api("{path}"')
            self.assertGreater(call, app.rindex("confirm(", 0, call), f"{path} is not behind a confirm dialog")
        with urllib.request.urlopen(self.base + "/params.js") as r:
            params = r.read().decode()
        self.assertGreater(params.index('api("/api/param"'), params.index("confirm("))
        for text in ("PERMANENT", "SOFTWARE RESET", "home parameter"):
            self.assertIn(text, params)
        self.assertNotIn("/api/param", app)  # parameter writes come only from the Edit buttons (params.js)
        # No per-model opt-in or label: every unit's parameters are written the same way, with no model note.
        self.assertNotIn("needs_model_opt_in", params)
        self.assertNotIn("allow_unverified_model", params)
        self.assertNotIn("verified_for_model", params)
        self.assertNotIn("UNVERIFIED", params)
        # One plain confirm before a parameter write: no second "Really change" for a home parameter, no opt-in flag.
        self.assertEqual(params.count("confirm("), 1)
        self.assertNotIn("Really change", params)
        self.assertNotIn("allow_home_params", params)
        self.assertNotIn("verified_for_model", fetch(self.base, "/workbench.js")[2])  # no per-parameter tag
        self.assertNotIn("Known unit code", fetch(self.base, "/unitinfo.js")[2])  # no unit-code status row
        self.assertIn("NOT CONFIRMED", params)  # an unconfirmed commit is not reported as committed
        # Baud change: 115200 and 230400 (verified) and 38400 (checked after the switch; the EC-R6S rod ignored the
        # 38400 code) are offered.
        start = html.index('id="newBaud"')
        selector = html[start : html.index("</select>", start)]
        self.assertEqual(re.findall(r"<option[^>]*>(\d+)</option>", selector), ["115200", "230400", "38400"])
        self.assertIn("ignored by the EC-R6S rod", html)


class ConfigDriverTest(_ServerCase):
    """The same endpoints through the real iai_actuator_driver and SimulatedEC."""

    session_factory = staticmethod(open_session)

    def sim(self) -> SimulatedEC:
        return self.ctl._session._bus.transport  # white-box: the SimulatedEC behind ModbusRTU

    def test_parameter_write_is_committed(self) -> None:
        self.connect_sim("gripper")
        result = self.ok("/api/param", {"no": 2, "value": "0.20"})["result"]
        self.assertEqual((result["before"], result["after"], result["committed"]), ("0.10 mm", "0.20 mm", True))
        self.assertTrue(result["needs_reset"])
        self.assertEqual(self.sim().parameter_commits, 1)
        again = self.ok("/api/param", {"no": 2, "value": "0.20"})["result"]  # unchanged: nothing written
        self.assertEqual((again["skipped"], again["committed"]), (True, False))
        self.assertFalse(again["needs_reset"])
        self.assertEqual(self.sim().parameter_commits, 1)
        self.assertTrue(any("nothing written" in e["msg"] for e in self.ctl.snapshot(event_count=50)["events"]))
        named = {p["no"]: p for p in self.ok("/api/unit_info")["info"]["parameters"]["named"]}
        self.assertEqual(named[2]["text"], "0.20 mm")
        self.assertTrue(named[2]["writable"] and named[4]["home_parameter"])
        home = self.ok("/api/param", {"no": 4, "value": 0.6})["result"]  # a home parameter: written, with a warning
        self.assertTrue(home["committed"])
        self.assertTrue(any("home parameter" in w for w in home["warnings"]), home["warnings"])
        self.assertEqual(self.sim().parameter_commits, 2)
        code, body = self.call("/api/param", {"no": 2, "value": 5})
        self.assertEqual(code, 400)  # outside the driver's sanity range: nothing written
        self.assertEqual(self.sim().parameter_commits, 2)

    def test_every_model_is_written_the_same_way(self) -> None:
        self.connect_sim("slider")  # unit 0x02: IA-OS lists its parameters; no model is singled out
        named = {p["no"]: p for p in self.ok("/api/unit_info")["info"]["parameters"]["named"]}
        self.assertTrue(named[5]["writable"])
        self.assertFalse(named[5]["needs_model_opt_in"] or named[2]["needs_model_opt_in"])  # no per-model gate
        self.assertEqual(named[5]["model_note"], "")  # no model label before the write
        result = self.ok("/api/param", {"no": 5, "value": 1})["result"]
        self.assertEqual((result["after"], result["commit_confirmed"]), ("1:Enable", True))
        self.assertFalse(any("UNVERIFIED" in w for w in result["warnings"]), result["warnings"])
        self.assertEqual(self.sim().parameter_commits, 1)

    def test_a_unit_code_without_a_family_name_gets_no_label(self) -> None:
        self.connect_sim("unknown")  # a unit code the driver has no family name for: driven the same way
        identity = self.ok("/api/identify")["identity"]
        self.assertFalse(identity["unit_known"])  # display only
        params = self.ok("/api/unit_info")["info"]["parameters"]
        self.assertEqual(params["model"]["seen_on"], [])
        notes = [params["model"]["note"]] + [p["model_note"] for p in params["named"]]
        text = json.dumps(identity) + json.dumps(notes)  # the register-meaning evidence elsewhere stays as it is
        for word in ("UNVERIFIED", "untested", "not been tried", "family table"):
            self.assertNotIn(word, text)
        self.ok("/api/servo", {"on": True})
        self.assertTrue(wait_until(lambda: self.status()["status"]["servo_on"] is True))
        self.ok("/api/home", {})
        self.wait_idle()
        self.ok("/api/move", {"position_mm": 10, "speed_mm_s": 50, "accel_g": 0.1})  # not refused for its code
        self.wait_at(10)

    def test_software_reset_on_the_rod(self) -> None:
        self.connect_sim("rod")
        result = self.ok("/api/software_reset", {})["result"]
        self.assertTrue(result["accepted"] and result["needs_home"])
        self.assertFalse(result["status"]["homed"])
        snap = self.status()
        self.assertEqual((snap["swreset"]["needs_home"], snap["swreset"]["modbus_enabled"]), (True, False))
        self.assertTrue(wait_until(lambda: self.status()["status"]["homed"] is False))
        code, body = self.call("/api/run_stored", {"n": 0, "allow_above_caps": True})
        self.assertEqual(code, 409)
        self.assertIn("not homed", body["error"])
        self.assertEqual(self.sim().software_resets, 1)

    def test_baud_change_in_the_simulator_session(self) -> None:
        connected = self.connect_sim()
        self.assertEqual((connected["connection"]["baud"], connected["connection"]["line_baud"]), (None, 115200))
        result = self.ok("/api/baud", {"rate": 230400})["result"]
        self.assertTrue(result["verified"] and result["code_verified"])
        self.assertEqual(self.sim().ec_baudrate, 230400)
        snap = self.status()
        self.assertEqual((snap["connection"]["line_baud"], snap["connection"]["baud"]), (230400, None))
        self.assertTrue(wait_until(lambda: self.status()["comm_ok"] and self.status()["status_age_s"] < 0.5))
        self.ok("/api/baud", {"rate": 115200, "method": "break"})
        self.assertEqual(self.sim().ec_baudrate, 115200)

    def test_run_stored(self) -> None:
        self.connect_sim("gripper")
        self.ok("/api/run_stored", {"n": 0})  # 3.00 mm at its stored 53 mm/s (= the gripper's cap)
        self.wait_at(3.0)
        self.ok("/api/run_stored", {"n": 1})  # the grip push row: approach 4.10, push to 9.50, miss
        self.assertTrue(wait_until(lambda: self.status()["status"]["push_missed"] is True, 8), self.status()["status"])
        self.ok("/api/disconnect", {})
        self.connect_sim()
        code, body = self.call("/api/run_stored", {"n": 2})
        self.assertEqual(code, 400)
        self.assertIn("allow_above_caps", body["error"])
        self.ok("/api/run_stored", {"n": 2, "allow_above_caps": True})
        self.wait_at(75.0)


class StopDuringSoftwareResetTest(_ControllerCase):
    """STOP keeps pre-empting: during a software reset (which holds the driver lock for ~1 s) the STOP coil still
    goes out within STOP_LOCK_WAIT_S; the EC is restarting, so it may not answer, and the message says why."""

    def test_stop_goes_out_during_the_reset(self) -> None:
        ctl = self.make(session_factory=open_session)
        with simulated_ec(sim_class=SerialTimingSim, baud=115200, profile="rod") as sims:
            ctl.connect("SIMULATED", 115200, 1, True, sim_profile="rod")
        sim = sims[0]
        outcome: dict[str, Any] = {}

        def reset() -> None:
            try:
                outcome["result"] = ctl.software_reset()
            except BackendError as exc:
                outcome["error"] = exc

        worker = threading.Thread(target=reset, daemon=True)
        worker.start()
        time.sleep(0.3)
        self.assertTrue(worker.is_alive(), "the reset ended before STOP: nothing to preempt")
        pressed = time.monotonic()
        try:
            ctl.stop()
        except BackendError as exc:
            self.assertIn("software reset was in progress", str(exc))
        coil = sim.stop_coil_on_after(pressed)
        self.assertIsNotNone(coil, "STOP was never sent during the reset")
        assert coil is not None
        self.assertLess(coil - pressed, 0.6)
        worker.join(15)
        self.assertFalse(worker.is_alive())
        self.assertIn("result", outcome, outcome)
        self.assertTrue(outcome["result"]["result"]["needs_home"])


# ====================================================================== multi-axis workbench (/ and /api/wb/*)


FAKE_PORTS = [
    {"port": "COM3", "description": "IAI USB to UART Bridge Controller", "vid": 0x10C4, "pid": 0x81D7, "is_iai": True},
    {"port": "COM7", "description": "Some USB serial", "vid": 0x0403, "pid": 0x6001, "is_iai": False},
]


class _WorkbenchCase(unittest.TestCase):
    """The GUI server with a workbench on simulators only (the port lister is a fake: no port is touched)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls._backup_dir = tempfile.TemporaryDirectory()  # simulator backups must not land in the project's backups/
        cls.addClassCleanup(cls._backup_dir.cleanup)
        patcher = mock.patch.object(workbench_module, "BACKUP_DIR", Path(cls._backup_dir.name))
        patcher.start()
        cls.addClassCleanup(patcher.stop)
        cls.ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=0.05)
        cls.wb = Workbench(poll_interval_s=0.05, sim_kwargs={"time_scale": 20},
                           port_lister=lambda: [dict(p) for p in FAKE_PORTS])
        cls.server = GuiServer(0, cls.ctl, token=TOKEN, workbench=cls.wb)
        cls.server.log_errors = False
        cls.base = f"http://127.0.0.1:{cls.server.port}"
        cls.thread = threading.Thread(target=cls.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.wb.close()
        cls.ctl.close()

    def tearDown(self) -> None:
        if self.wb.ports:
            self.wb.disconnect()

    def call(self, path: str, data: Any = None, **kw: Any) -> tuple[int, dict[str, Any]]:
        return http_call(self.base, path, data, **kw)

    def ok(self, path: str, data: Any = None) -> dict[str, Any]:
        code, body = self.call(path, data)
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])
        return body

    def sim(self, setup: str = "slider") -> list[str]:
        body = self.ok("/api/wb/connect", {"simulate": True, "sim_setup": setup})
        return [a for port in body["connected"] for a in port["axes"]]

    def axis(self, name: str) -> dict[str, Any]:
        return next(a for a in self.ok("/api/wb/state")["axes"] if a["name"] == name)

    def run_fn(self, axis: str, fn: str, **args: Any) -> Any:
        return self.ok("/api/wb/call", {"axis": axis, "fn": fn, "args": args})["result"]

    def ready(self, *axes: str) -> None:
        for name in axes:
            self.run_fn(name, "servo", on=True)
            self.run_fn(name, "home")
        for name in axes:
            def homed(n: str = name) -> bool:
                ax = self.axis(n)
                st = ax["status"] or {}
                return st.get("homed") is True and not st.get("moving") and not ax["homing_watch"]
            self.assertTrue(wait_until(homed, 10), f"{name} never finished homing")

    def wait_still(self, name: str, timeout: float = 8.0) -> dict[str, Any]:
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["moving"] is False, timeout), f"{name} still moving")
        return self.axis(name)["status"]

    def wait_at(self, name: str, position: float, timeout: float = 8.0) -> None:
        def arrived() -> bool:
            st = self.axis(name)["status"]
            return st["moving"] is False and abs(st["position_mm"] - position) < 0.15
        self.assertTrue(wait_until(arrived, timeout), f"{name} never arrived at {position}: {self.axis(name)['status']}")


class WorkbenchPageTest(_WorkbenchCase):
    def test_root_is_the_workbench_with_stop_in_the_header(self) -> None:
        with urllib.request.urlopen(self.base + "/") as r:
            html = r.read().decode()
            self.assertEqual(r.headers["X-Frame-Options"], "DENY")
        self.assertNotIn(TOKEN, html)  # the access key is not in the page: it comes in the launch address
        self.assertNotIn("__TOKEN__", html)
        self.assertIn("not an IAI product", html)
        header = html[html.index('<header class="appbar">'):html.index("</header>")]
        self.assertIn('id="stopAll"', header)
        self.assertIn("SPACE / ESC", header)
        self.assertLess(html.index('id="stopAll"'), html.index('id="viewAxis"'))
        for marker in ('id="portList"', 'id="simSetup"', 'id="axisCards"', 'id="opForms"', 'id="posTable"',
                       'id="paramTable"', 'id="historyTable"', 'id="baudForm"', 'id="swReset"', 'id="groupTable"',
                       'id="grpMove"', 'id="fnList"', 'id="confirmDlg"', 'href="/classic"', "/workbench.js"):
            self.assertIn(marker, html)
        for path in ("/workbench.js", "/workbench.css", "/styles.css", "/classic"):
            with urllib.request.urlopen(self.base + path) as r:
                self.assertEqual(r.status, 200, path)

    def test_script_asks_before_every_write_and_stops_on_space_and_esc(self) -> None:
        with urllib.request.urlopen(self.base + "/workbench.js") as r:
            js = r.read().decode()
        self.assertIn('ev.key === "Escape"', js)
        self.assertIn('ev.key === " "', js)
        self.assertIn('rawApi("/api/wb/stop"', js)  # STOP goes round the request budget (stopRequest)
        # Non-volatile and port-wide writes are sent only after a confirm dialog (ask / confirmFor).
        for call in ('fn: "write_position_table"', 'fn: "write_parameter"', 'fn: "software_reset"', 'fn: "set_baudrate"'):
            at = js.index(call)
            before = js[:at]
            self.assertGreater(max(before.rfind("await ask("), before.rfind("await confirmFor(")),
                               max(before.rfind("\nasync function "), before.rfind("addEventListener(")), call)
        # One plain OK / Cancel: no tick box keeps OK disabled and no opt-in flag is sent.
        for gone in ("dlgAck", "ack:", "allow_dangerous", "allow_unverified_code", "allow_nonvolatile"):
            self.assertNotIn(gone, js)
        self.assertNotIn("dlgAck", fetch(self.base, "/workbench.html")[2])
        coord = fetch(self.base, "/coord.js")[2]
        self.assertNotIn("ack:", coord)  # ask() takes no tick box: the coordinated-run warning is in the text
        self.assertIn("Experimental: keep a hand on the emergency stop.", coord)
        self.assertIn("DANGEROUS parameter", js)  # the warning stays in the dialog text
        self.assertIn("NON-VOLATILE", js)
        self.assertIn("await confirmFor(spec, args, axis)", js)  # every catalogue Run (Functions view) asks too
        self.assertNotIn("innerHTML", js)  # unit data is always inserted as text

    def test_workbench_api_needs_the_token(self) -> None:
        for path in ("/api/wb/state", "/api/wb/ports", "/api/wb/functions"):
            self.assertEqual(self.call(path, token="wrong")[0], 403, path)
        self.assertEqual(self.call("/api/wb/stop", {}, token="")[0], 403)
        self.assertEqual(self.call("/api/wb/connect", {"simulate": True}, host="evil.example:80")[0], 403)
        self.assertFalse(self.wb.ports)


class WorkbenchConnectTest(_WorkbenchCase):
    def test_ports_listed_with_sim_setups(self) -> None:
        body = self.ok("/api/wb/ports")
        self.assertEqual([p["port"] for p in body["ports"]], ["COM3", "COM7"])
        self.assertTrue(body["ports"][0]["is_iai"])
        self.assertEqual({s["name"] for s in body["sim_setups"]}, set(SIM_SETUPS))
        self.assertFalse(any(p["connected"] for p in body["ports"]))

    def test_one_axis_per_port_and_several_axes_on_one_port(self) -> None:
        line = self.sim("line")
        self.assertEqual([n.split(":")[1] for n in line], ["1", "2", "3"])
        self.assertEqual(len({n.split(":")[0] for n in line}), 1)  # three axes on ONE port
        cell = self.sim("cell")
        self.assertEqual(len({n.split(":")[0] for n in cell}), 3)  # one axis per port
        state = self.ok("/api/wb/state")
        self.assertEqual(len(state["ports"]), 4)
        for ax in state["axes"]:
            self.assertEqual(ax["axis_number"], ax["slave"] - 1)
            self.assertIsNone(ax["error"], ax)
            self.assertIn("position", ax["limits"])
            self.assertIn(ax["limits"]["units"], ("mm", "deg"))
            self.assertIsNotNone(ax["status"])
        self.assertGreaterEqual(len({a["info"]["model"] for a in state["axes"]}), 3)
        port = line[0].split(":")[0]
        self.ok("/api/wb/disconnect", {"port": port})
        self.assertEqual(len(self.ok("/api/wb/state")["axes"]), 3)
        self.assertEqual(self.call("/api/wb/disconnect", {"port": port})[0], 404)

    def test_bad_requests(self) -> None:
        self.assertEqual(self.call("/api/wb/connect", {"simulate": True, "sim_setup": "bogus"})[0], 400)
        self.assertEqual(self.call("/api/wb/call", {"axis": "NOPE:1", "fn": "status"})[0], 404)
        [name] = self.sim("slider")
        self.assertEqual(self.call("/api/wb/call", {"axis": name, "fn": "rm_rf"})[0], 400)
        self.assertEqual(self.call("/api/wb/call", {"axis": name, "fn": "move_to", "args": {"mm": "x"}})[0], 400)
        self.assertEqual(self.call("/api/wb/call", {"axis": name, "fn": "move_to", "args": []})[0], 400)
        self.assertEqual(self.call("/api/wb/port", {"port": "SIMX", "fn": "scan"})[0], 404)
        self.assertEqual(self.call("/api/wb/group", {"op": "explode"})[0], 400)
        self.assertEqual(self.call("/api/wb/positions")[0], 400)

    def test_sim_only_mode_lists_and_opens_no_serial_port(self) -> None:
        wb = Workbench(sim_only=True, port_lister=lambda: self.fail("listed serial ports in sim-only mode"))
        try:
            self.assertEqual(wb.list_ports()["ports"], [])
            with self.assertRaises(BackendError):
                wb.connect(port="COM3")
        finally:
            wb.close()

    def test_parse_slaves(self) -> None:
        self.assertEqual(parse_slaves("1-3,5"), [1, 2, 3, 5])
        self.assertEqual(parse_slaves(""), list(range(1, 17)))
        for bad in ("0", "248", "a-b"):
            with self.assertRaises(BackendError):
                parse_slaves(bad)


class WorkbenchFunctionsTest(_WorkbenchCase):
    def test_catalogue_describes_every_input_with_a_range_from_the_unit(self) -> None:
        cat = self.ok("/api/wb/functions")
        names = [f["name"] for f in cat["axis"]]
        self.assertEqual(len(names), len(set(names)))
        for must in ("servo", "home", "move_to", "move_by", "jog", "push", "move_to_position", "stop", "pause",
                     "resume", "reset_alarm", "write_position_table", "write_parameter", "software_reset", "parameters",
                     "read_monitor", "read_alarm_history", "limits", "identify", "unit_info"):
            self.assertIn(must, names)
        self.assertEqual({f["name"] for f in cat["port"]}, {"scan", "set_baudrate"})
        for setup in ("slider", "gripper", "rotary", "pcon", "unknown"):
            [name] = self.sim(setup)
            limits = self.axis(name)["limits"]
            for f in cat["axis"]:
                self.assertTrue(f["desc"] and f["label"] and f["call"], f["name"])
                for inp in f["inputs"]:
                    if inp.get("range"):
                        self.assertIn(inp["range"], limits, (setup, f["name"], inp["name"]))
                        self.assertIn("max", limits[inp["range"]])
                    if inp["type"] in ("number", "int") and not inp.get("optional"):
                        self.assertIn("example", inp, (f["name"], inp["name"]))
            self.wb.disconnect()

    def test_every_read_function_runs_on_every_kind(self) -> None:
        reads = [f for f in FUNCTIONS if f["group"] == "Read"]
        examples = {"n": 0, "code": "0x0E0", "key": "8.15", "address": "0x9000", "count": 4}
        for setup in ("slider", "pcon", "rotary"):
            [name] = self.sim(setup)
            for f in reads:
                if f["name"] == "unit_info":
                    args = {"section": "identity"}
                else:
                    args = {i["name"]: examples[i["name"]] for i in f["inputs"] if i["name"] in examples}
                code, body = self.call("/api/wb/call", {"axis": name, "fn": f["name"], "args": args})
                self.assertEqual(code, 200, (setup, f["name"], body))
            self.wb.disconnect()

    def test_motion_ranges_and_the_drivers_error_text(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        limits = self.axis(name)["limits"]
        top = limits["position"]["max"]
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "move_to",
                                                "args": {"mm": top + 50, "speed_mm_s": 50, "accel_g": 0.3}})
        self.assertEqual(code, 400, body)
        self.assertIn("SoftLimitError", body["error"])  # the driver's own refusal, shown as it is
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "move_to",
                                                "args": {"mm": 10, "speed_mm_s": limits["speed"]["max"] * 2,
                                                         "accel_g": 0.3}})
        self.assertEqual(code, 400, body)
        self.run_fn(name, "check_move", mm=40, speed_mm_s=50, accel_g=0.3)
        self.run_fn(name, "move_to", mm=40, speed_mm_s=100, accel_g=0.3, decel_g=0.2)
        self.wait_at(name, 40.0)
        self.run_fn(name, "move_by", delta_mm=-10, speed_mm_s=100, accel_g=0.3)
        self.wait_at(name, 30.0)
        self.run_fn(name, "jog", delta_mm=1, speed_mm_s=10, accel_g=0.3)
        self.wait_at(name, 31.0)
        self.run_fn(name, "move_to_position", n=0, speed_pct=50)
        self.wait_at(name, 0.0)
        self.run_fn(name, "push", target_mm=20, band_mm=10, force_pct=30, direction=1, speed_mm_s=50, accel_g=0.3)

        def push_done() -> bool:
            st = self.axis(name)["status"] or {}
            return bool(st.get("push_missed") or st.get("push_contact"))
        self.assertTrue(wait_until(push_done, 10), self.axis(name))

    def test_stop_all_stops_every_axis_mid_move(self) -> None:
        # a stepped simulator clock: each unit's time advances with the requests it gets, so the three axes (2 mm/s
        # towards their far ends) are still mid-move when STOP arrives, however long a busy PC takes to send it (at
        # time_scale 20 the whole move was over within 0.25 s of real time)
        with mock.patch.object(self.wb, "sim_kwargs", {"step_s": 0.05}):
            names = self.sim("line")
        self.ready(*names)
        targets = {n: self.axis(n)["limits"]["position"]["max"] for n in names}
        self.ok("/api/wb/group", {"op": "move_all", "axes": names, "targets": targets, "speed": {n: 2 for n in names}})
        self.assertTrue(wait_until(lambda: all(self.axis(n)["status"]["moving"] for n in names), 3))
        body = self.ok("/api/wb/stop", {})
        self.assertEqual(sorted(body["stopped"]), sorted(names))
        for n in names:
            self.assertLess(self.wait_still(n)["position_mm"], targets[n] - 0.5, n)

    def test_group_move_refuses_all_when_one_target_is_out_of_range(self) -> None:
        names = self.sim("line")
        self.ready(*names)
        before = {n: self.axis(n)["status"]["position_mm"] for n in names}
        targets = {n: 3.0 for n in names}
        targets[names[2]] = 9999.0
        code, body = self.call("/api/wb/group", {"op": "move_all", "axes": names, "targets": targets,
                                                 "speed": {n: 20 for n in names}})
        self.assertIn(code, (400, 409), body)
        self.assertIn("SoftLimitError", body["error"])
        self.assertIn("nothing started", body["error"])
        time.sleep(0.3)
        for n in names:
            self.assertFalse(self.axis(n)["status"]["moving"], n)
            self.assertEqual(self.axis(n)["status"]["position_mm"], before[n], n)

    def test_stored_rows_parameters_and_overview(self) -> None:
        [name] = self.sim("pcon")
        limits = self.axis(name)["limits"]
        rows = self.ok(f"/api/wb/positions?axis={name}")["rows"]
        self.assertEqual(len(rows), limits["stored_rows"]["count"])
        self.assertGreater(len(rows), 3)  # every row the unit uses (the PCON has more than an EC)
        new = round(min(rows[5]["position_mm"] + 1.0, limits["position"]["max"]), 2)
        written = self.run_fn(name, "write_position_table", n=5, position_mm=new, speed_mm_s=100)
        self.assertEqual(written["position_mm"], new)
        again = self.run_fn(name, "write_position_table", n=5, position_mm=new, speed_mm_s=100)
        self.assertTrue(again["skipped"] and "nothing written" in again["note"])  # the same row: not written again
        self.assertEqual(self.ok(f"/api/wb/positions?axis={name}")["rows"][5]["position_mm"], new)
        params = self.ok(f"/api/wb/parameters?axis={name}")["parameters"]
        self.assertTrue(params)
        for key in ("number", "name", "value", "unit", "min", "max", "writable", "dangerous"):
            self.assertIn(key, params[0])
        danger = next((p for p in params if p["dangerous"] and p["writable"]), None)
        if danger is not None:
            code, body = self.call("/api/wb/call", {"axis": name, "fn": "write_parameter",
                                                    "args": {"key": str(danger["number"]), "value": danger["value"]}})
            self.assertEqual(code, 200, body)  # a dangerous parameter needs no opt-in
            self.assertTrue(body["result"]["skipped"], body)  # the value it holds: nothing written
        over = self.ok(f"/api/wb/overview?axis={name}")
        self.assertIn("monitor", over)
        for rec in over["history"] or []:
            self.assertIn("remedy", rec)

    def test_port_scan_and_broadcast_baud_change(self) -> None:
        names = self.sim("line")
        port = names[0].split(":")[0]
        body = self.ok("/api/wb/port", {"port": port, "fn": "scan", "args": {"slaves": "1-4"}})
        self.assertEqual(body["added"], [])
        self.assertEqual(len(body["hits"]), 3)
        result = self.ok("/api/wb/port", {"port": port, "fn": "set_baudrate", "args": {"rate": 230400}})["result"]
        self.assertTrue(all(result["answered"].values()), result)
        self.assertEqual(next(p for p in self.ok("/api/wb/state")["ports"] if p["name"] == port)["baud"], 230400)
        for n in names:
            self.run_fn(n, "status")
        # An UNVERIFIED rate code is sent without an opt-in (the core logs a warning instead of refusing it).
        result = self.ok("/api/wb/port", {"port": port, "fn": "set_baudrate", "args": {"rate": 57600}})["result"]
        self.assertTrue(all(result["answered"].values()), result)
        self.assertEqual(next(p for p in self.ok("/api/wb/state")["ports"] if p["name"] == port)["baud"], 57600)

    def test_events_are_logged(self) -> None:
        [name] = self.sim("gripper")
        self.run_fn(name, "servo", on=True)
        state = self.ok("/api/wb/state?since=0")
        self.assertTrue(any(name in e["message"] for e in state["events"]))
        later = self.ok(f"/api/wb/state?since={state['seq']}")
        self.assertEqual(later["events"], [])


class WorkbenchRound2Test(_WorkbenchCase):
    """The round-2 driver functions in the workbench: catalogue, change speed, cancel pause, teach, backup, line info,
    condition flags, servo reasons, broadcast STOP (simulator only)."""

    def line_of(self, name: str) -> Any:
        return self.wb.ports[name.split(":")[0]].transport

    def sim_of(self, name: str) -> SimulatedEC:
        return self.line_of(name)[int(name.split(":")[1])]

    def test_baud_choices_are_labelled_by_the_evidence_of_their_code(self) -> None:
        port_fns = {f["name"]: f for f in workbench_module.PORT_FUNCTIONS}
        labels = {rate: label for rate, label in port_fns["set_baudrate"]["inputs"][0]["choices"]}
        self.assertEqual((labels[115200], labels[230400]), ("115200", "230400"))
        self.assertIn("depends on the unit", labels[38400])  # the EC-GRBP8M gripper and the PCON-CB take it, the rod does not
        self.assertNotIn("UNVERIFIED", labels[38400])  # the core sends it without the opt-in, so the page asks for none
        self.assertIn("UNVERIFIED", labels[57600])

    def test_catalogue_has_the_new_functions_with_evidence_labels(self) -> None:
        by_name = {f["name"]: f for f in FUNCTIONS}
        for name, group in (("change_speed", "Motion"), ("change_accel", "Motion"), ("cancel_paused_move", "Motion"),
                            ("teach_position", "Configuration"), ("backup", "Read"), ("line_info", "Read")):
            self.assertEqual(by_name[name]["group"], group, name)
            self.assertTrue(by_name[name]["desc"] and by_name[name]["call"], name)
        self.assertEqual(by_name["teach_position"]["confirm"], "nonvolatile")  # a non-volatile write asks first
        self.assertIn("VERIFIED on the EC gripper and the PCON", by_name["move_to"]["desc"])
        self.assertIn("VERIFIED on the EC gripper and the PCON", by_name["move_to_position"]["desc"])
        self.assertEqual(len({f["name"] for f in FUNCTIONS}), len(FUNCTIONS))
        for name in ("change_speed", "change_accel", "cancel_paused_move", "recover"):  # verified on hardware now
            self.assertNotIn("UNVERIFIED", by_name[name]["desc"], name)
        for key in ("emg_input", "drive_power_cut", "enable_off", "cold_start_alarm", "auto_servo_off", "nv_busy",
                    "manual_mode", "battery_low", "rtc_in_use", "excitation_done", "position_complete_no"):
            self.assertIn(key, STATUS_KEYS)
        group = {f["name"]: f for f in self.ok("/api/wb/functions")["group"]}
        self.assertIn("broadcast", group["stop_all"]["desc"])

    def test_status_carries_the_condition_flags_and_the_port_its_line(self) -> None:
        [name] = self.sim("slider")
        status = self.axis(name)["status"]
        for key in ("emg_input", "drive_power_cut", "enable_off", "cold_start_alarm", "position_complete_no", "line"):
            self.assertIn(key, status)
        self.assertIs(status["emg_input"], False)
        self.assertEqual(self.ok("/api/wb/state")["ports"][0]["line"], "direct")

    def test_servo_on_is_refused_with_the_reasons_while_the_drive_power_is_cut(self) -> None:
        [name] = self.sim("slider")
        self.sim_of(name).set_inputs(drive_cut=True)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["drive_power_cut"] is True, 3))
        before = len(self.line_of(name).requests)
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "servo", "args": {"on": True}})
        self.assertEqual(code, 409, body)
        self.assertIn("ServoBlockedError", body["error"])
        self.assertIn("motor drive power is cut", body["error"])
        sent = {f[1] for _, f in self.line_of(name).requests[before:] if len(f) > 1}
        self.assertEqual(sent - {0x03}, set(), "nothing but reads went out before the refusal")

    def test_change_speed_re_sends_the_running_move_with_the_new_speed(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        top = self.axis(name)["limits"]["position"]["max"]
        self.run_fn(name, "move_to", mm=top, speed_mm_s=2, accel_g=0.3)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["moving"] is True, 3))
        self.run_fn(name, "change_speed", speed_mm_s=10)
        blocks = [f for _, f in self.line_of(name).requests if f[1] == 0x10 and f[2:4] == b"\x99\x00"]
        self.assertGreaterEqual(len(blocks), 2)
        self.assertEqual(blocks[-1][15:19], (10 * 100).to_bytes(4, "big"), "VCMD of the last 0x9900 block is 10 mm/s")
        self.assertEqual(blocks[-1][7:11], blocks[0][7:11], "same target")
        self.assertEqual(blocks[-1][11:15], blocks[0][11:15], "same band")
        self.run_fn(name, "change_accel", accel_g=0.2)
        self.ok("/api/wb/stop", {})
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "change_speed", "args": {"speed_mm_s": 10}})
        self.assertIn(code, (400, 409), body)  # nothing running any more: refused, nothing sent
        self.assertIn("nothing sent", body["error"])

    def test_cancel_paused_move_drops_the_remaining_travel(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        top = self.axis(name)["limits"]["position"]["max"]
        self.run_fn(name, "move_to", mm=top, speed_mm_s=2, accel_g=0.3)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["moving"] is True, 3))
        self.run_fn(name, "pause")
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["paused"] is True, 3))
        self.run_fn(name, "cancel_paused_move")
        status = self.wait_still(name)
        self.assertFalse(status["paused"])
        stopped_at = status["position_mm"]
        time.sleep(0.4)
        self.assertEqual(self.axis(name)["status"]["position_mm"], stopped_at, "the old target is not resumed")
        self.assertLess(stopped_at, top - 0.5)
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "cancel_paused_move", "args": {}})
        self.assertIn(code, (400, 409), body)  # nothing is paused now
        self.assertIn("no pause is active", body["error"])

    def test_teach_position_writes_the_current_position_into_the_row(self) -> None:
        [name] = self.sim("pcon")
        self.ready(name)
        self.run_fn(name, "move_to", mm=40, speed_mm_s=100, accel_g=0.3)
        self.wait_at(name, 40.0)
        row = self.run_fn(name, "teach_position", n=5)
        self.assertEqual(row["position_mm"], 40.0)
        self.assertEqual(self.run_fn(name, "read_position_table", n=5)["position_mm"], 40.0)

    def test_workbench_cases_do_not_write_backups_into_the_project(self) -> None:
        self.assertNotEqual(workbench_module.BACKUP_DIR, workbench_module.ROOT / "backups")

    def test_backup_is_read_only_and_saved_as_json(self) -> None:
        [name] = self.sim("gripper")
        before = len(self.line_of(name).requests)
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(workbench_module, "BACKUP_DIR", Path(folder)):
            result = self.run_fn(name, "backup", rows="0,1")
            saved = Path(result["saved_to"])
            self.assertEqual(saved.parent, Path(folder))
            data = json.loads(saved.read_text(encoding="utf-8"))
        self.assertIs(data["meta"]["read_only"], True)
        self.assertEqual(result["position_rows"], 2)
        self.assertGreater(result["parameter_items"], 0)
        sent = {f[1] for _, f in self.line_of(name).requests[before:]}
        self.assertLessEqual(sent, {0x03, 0x11}, "a backup sends FC03 / FC11 only")
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "backup", "args": {"rows": "abc"}})
        self.assertEqual(code, 400, body)

    def test_line_info_on_a_direct_line(self) -> None:
        [name] = self.sim("slider")
        info = self.run_fn(name, "line_info")
        self.assertEqual(info["line"]["kind"], "direct")
        self.assertIsNone(info["gateway_status"])
        self.assertEqual(info["stop_path"], "coil")
        self.assertEqual(info["errors"], [])
        info = self.run_fn(name, "line_info", monitor=True)
        self.assertTrue(any("RCON" in e for e in info["errors"]))

    def test_stop_all_can_broadcast_on_a_daisy_chain(self) -> None:
        # a stepped simulator clock (as test_stop_all_stops_every_axis_mid_move): the three axes are still mid-move when
        # the broadcast STOP arrives, however long a busy PC takes to send it
        with mock.patch.object(self.wb, "sim_kwargs", {"step_s": 0.05}):
            names = self.sim("line")
        self.ready(*names)
        top = {n: self.axis(n)["limits"]["position"]["max"] for n in names}
        self.ok("/api/wb/group", {"op": "move_all", "axes": names, "targets": top, "speed": {n: 2 for n in names}})
        self.assertTrue(wait_until(lambda: all(self.axis(n)["status"]["moving"] for n in names), 3))
        body = self.ok("/api/wb/stop", {"broadcast": True})
        self.assertEqual(sorted(body["stopped"]), sorted(names))
        frames = [f for _, f in self.line_of(names[0]).requests]
        self.assertIn(bytes.fromhex("00 05 04 2C FF 00 4D 12"), frames, "the broadcast STOP went out")
        self.assertIn(bytes.fromhex("00 05 04 2C 00 00 0C E2"), frames, "and was released")
        for n in names:
            self.assertLess(self.wait_still(n)["position_mm"], top[n] - 0.5, n)

    def test_pcon_push_power_uses_the_truncating_scale(self) -> None:
        [name] = self.sim("pcon")
        self.ready(name)
        row = self.run_fn(name, "write_position_table", n=6, position_mm=20, mode="push+", push_pct=70, band_mm=5)
        self.assertEqual(row["push_power"], 178)  # 70 % is 0xB2 (floor), never 0xB3
        self.assertEqual(row["push_pct"], 69.8)


def fetch(base: str, path: str, *, host: str | None = None) -> tuple[int, str, str]:
    """A plain GET without the API token (static pages and documents): (status, content type, text)."""
    req = urllib.request.Request(base + path)
    if host:
        req.add_header("Host", host)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.headers.get("Content-Type", ""), r.read().decode("utf-8")
    except urllib.error.HTTPError as e:
        with e:
            return e.code, e.headers.get("Content-Type", ""), e.read().decode("utf-8")


class ProductPolishTest(_WorkbenchCase):
    """Release polish: product title and version in both headers, the About panel, document links, consistent labels."""

    def test_both_pages_show_title_version_notice_about_and_keep_stop_in_the_header(self) -> None:
        for path, stop_id, title in (("/", "stopAll", "IAI Actuator Driver Workbench"), ("/classic", "stopBtn", "IAI Actuator Driver Single Axis")):
            status, ctype, html = fetch(self.base, path)
            self.assertEqual(status, 200, path)
            self.assertIn("text/html", ctype)
            self.assertNotIn("__VERSION__", html)
            self.assertNotIn("__TOKEN__", html)
            header = html[html.index('<header class="appbar">'):html.index("</header>")]
            self.assertIn(title, header, path)
            self.assertIn(f"v{__version__}", header, path)
            self.assertIn("Unofficial", header, path)
            self.assertIn("not an IAI product", header, path)
            self.assertIn('id="aboutBtn"', header, path)
            self.assertIn(f'id="{stop_id}"', header, path)  # STOP stays in the sticky header bar
            self.assertIn("SPACE / ESC", header, path)
            self.assertIn(f"<title>{title} {__version__}</title>", html, path)
            self.assertIn('src="/about.js"', html, path)
        self.assertIn('href="/classic"', fetch(self.base, "/")[2])
        self.assertIn('href="/"', fetch(self.base, "/classic")[2])

    def test_about_api_needs_the_token_and_describes_the_release(self) -> None:
        self.assertEqual(self.call("/api/about", token="wrong")[0], 403)
        about = self.ok("/api/about")
        self.assertEqual(about["version"], __version__)
        self.assertEqual(about["product"], "IAI Actuator Driver")
        self.assertEqual(about["stage"], "stable")
        self.assertFalse(about["sim_only"])
        self.assertTrue(any("not an IAI product" in t for t in about["notice"]), about["notice"])
        self.assertTrue(any("safety function" in t for t in about["notice"]), about["notice"])
        self.assertEqual([w for w, _ in about["evidence"]], ["VERIFIED", "ON-WIRE", "DOC", "UNVERIFIED"])
        docs = {d["title"]: d for d in about["docs"]}
        self.assertEqual(list(docs), ["Getting started", "Coordinated motion", "Compatibility", "Function reference",
                                      "Reference and evidence", "README", "Changelog"])
        for d in docs.values():
            self.assertTrue(d["available"], d)  # a source checkout has every document
            self.assertEqual(fetch(self.base, d["href"])[0], 200, d)

    def test_documents_are_plain_text_and_only_the_listed_ones_are_served(self) -> None:
        status, ctype, text = fetch(self.base, "/docs/COMPATIBILITY.md")
        self.assertEqual(status, 200)
        self.assertTrue(ctype.startswith("text/plain"), ctype)
        self.assertIn("Not supported", text)
        status, ctype, text = fetch(self.base, "/docs/COORDINATED.md")  # the doc the Coordinated tab belongs with
        self.assertEqual((status, ctype.startswith("text/plain")), (200, True))
        self.assertTrue(text.startswith("#"), text[:40])
        for path in ("/docs/NOPE.md", "/docs/../pyproject.toml", "/pyproject.toml", "/docs/", "/CLAUDE.md"):
            self.assertEqual(fetch(self.base, path)[0], 404, path)
        self.assertEqual(fetch(self.base, "/docs/DRIVER.md", host="evil.example:80")[0], 403)  # the Host check still applies

    def test_about_script_builds_text_only_and_the_demo_banner_exists(self) -> None:
        status, ctype, js = fetch(self.base, "/about.js")
        self.assertEqual(status, 200)
        self.assertIn("javascript", ctype)
        self.assertNotIn("innerHTML", js)
        self.assertIn('"/api/about"', js)
        self.assertIn("DEMO MODE", js)
        self.assertIn("showModal", js)

    def test_labels_are_the_same_on_both_pages_and_in_the_catalogue(self) -> None:
        _, _, classic = fetch(self.base, "/classic")
        _, _, workbench = fetch(self.base, "/")
        for html in (classic, workbench):
            self.assertIn("Alarm reset", html)
            self.assertNotIn("Reset alarm", html)
            self.assertIn("Close IA-OS", html)
            self.assertIn("Servo ON", html)
        self.assertNotIn("Reset alarm", fetch(self.base, "/app.js")[2])
        for name, (label, ports) in SIM_SETUPS.items():  # the single-axis page offers the same simulated units by name
            if len(ports) == 1 and len(ports[0]) == 1:
                self.assertIn(f'<option value="{name}"', classic)
                self.assertIn(f">{label}</option>", classic, name)
        labels = {f["name"]: f["label"] for f in FUNCTIONS}
        self.assertEqual(labels["reset_alarm"], "Alarm reset")
        self.assertEqual(labels["home"], "Home")
        self.assertEqual(labels["stop"].split()[0], "STOP")

    def test_workbench_demo_mode_hooks_and_default_baud(self) -> None:
        _, _, html = fetch(self.base, "/")
        for hook in ("serialHead", "portList", "serialFields", "connectPorts", "serialNote", "emptyText", "addSim"):
            self.assertIn(f'id="{hook}"', html)
        self.assertIn('<option value="auto" selected>auto-detect (recommended)</option>', html)
        js = fetch(self.base, "/workbench.js")[2]
        self.assertIn("applyDemoMode(!!v.sim_only)", js)


class WorkbenchFastRecoveryTest(_WorkbenchCase):
    """Fast mode per axis (live status from the monitor) and the power / link-loss banner with Recover."""

    def line_of(self, name: str) -> Any:
        return self.wb.ports[name.split(":")[0]].transport

    def test_page_has_the_fast_toggle_the_fault_banner_and_the_recover_button(self) -> None:
        _, _, html = fetch(self.base, "/")
        head = html[html.index('id="axisHead"'):html.index('id="axisTabs"')]
        for hook in ('id="fastToggle"', 'id="axFault"', 'id="axRecover"', 'id="axRecovery"', 'role="alert"'):
            self.assertIn(hook, head)
        js = fetch(self.base, "/workbench.js")[2]
        for needle in ('fn: "recover"', 'fn: "set_fast"', "renderRecovery(ax)", "nothing was moved", '"LIVE"', "(kept)"):
            self.assertIn(needle, js)
        self.assertNotIn("innerHTML", js)
        names = {f["name"]: f for f in self.ok("/api/wb/functions")["axis"]}
        self.assertEqual(names["recover"]["group"], "Link")
        self.assertIn("Never moves", names["recover"]["desc"])
        self.assertTrue(names["check_link"]["readonly"])

    def test_fast_mode_gives_live_status_from_the_monitor(self) -> None:
        [name] = self.sim("gripper")
        self.assertEqual(self.axis(name)["status"]["source"], "read")
        result = self.run_fn(name, "set_fast", on=True)
        self.assertEqual(result["completion"], "fast")
        ax = self.axis(name)
        self.assertTrue(ax["fast"])
        self.assertTrue(ax["monitor"]["running"])
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["source"] == "monitor"))
        self.assertTrue(wait_until(lambda: (self.axis(name)["monitor"]["polls"] or 0) > 10))
        self.ready(name)
        self.run_fn(name, "move_to", mm=5.0, speed_mm_s=20, accel_g=0.3)
        self.wait_at(name, 5.0)
        self.run_fn(name, "set_fast", on=False)
        ax = self.axis(name)
        self.assertFalse(ax["fast"])
        self.assertIsNone(ax["monitor"])
        self.assertEqual(ax["completion"], "safe")
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["source"] == "read"))

    def test_a_vanished_port_shows_a_fault_and_recover_clears_it(self) -> None:
        [name] = self.sim("gripper")
        line = self.line_of(name)
        line.unplug()
        self.assertTrue(wait_until(lambda: (self.axis(name)["fault"] or {}).get("kind") == "port_vanished"))
        fault = self.axis(name)["fault"]
        self.assertEqual(fault["event"], "link_lost")
        self.assertIn("vanished", fault["text"])
        self.assertEqual(self.axis(name)["link_state"], "port_vanished")
        line.replug()
        report = self.run_fn(name, "recover", timeout_s=5)
        self.assertTrue(report["port_reopened"])
        self.assertTrue(report["same_unit"])
        self.assertTrue(report["control_kept"])  # a vanished port does not drop PC control
        self.assertFalse(report["control_restored"])
        ax = self.axis(name)
        self.assertIsNone(ax["fault"])
        self.assertIsNone(ax["error"])
        self.assertTrue(ax["recovery"]["ok"])
        self.assertEqual(ax["recovery"]["report"]["summary"], report["summary"])
        self.assertEqual(ax["link_state"], "ok")
        messages = [e["message"] for e in self.ok("/api/wb/state")["events"]]
        self.assertTrue(any("link lost" in m for m in messages), messages)

    def test_a_power_cycle_is_a_restart_and_recover_restores_the_link(self) -> None:
        [name] = self.sim("rod")
        self.ready(name)
        self.run_fn(name, "set_fast", on=True)
        line = self.line_of(name)
        line.power_cycle()  # 115200 -> the unit is back at 38400: silent at the PC's rate
        self.assertTrue(wait_until(lambda: (self.axis(name)["fault"] or {}).get("kind") == "unit_silent"))
        report = self.run_fn(name, "recover", timeout_s=10)
        self.assertTrue(report["restarted"])
        self.assertTrue(report["baud_restored"])
        self.assertTrue(report["control_restored"])  # a power cycle drops PMSS: taken again
        self.assertFalse(report["control_kept"])
        self.assertTrue(report["needs_home"])  # an incremental rod: home before the next move
        self.assertTrue(report["needs_attention"])
        ax = self.axis(name)
        self.assertIsNone(ax["fault"])
        self.assertTrue(ax["monitor"]["running"], "the Fast-mode monitor is back after the recovery")
        self.assertEqual(self.wb.ports[name.split(":")[0]].port.baudrate, 115200)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["homed"] is False))

    def test_a_failed_recovery_is_reported(self) -> None:
        [name] = self.sim("gripper")
        line = self.line_of(name)
        line.unplug()
        self.assertTrue(wait_until(lambda: self.axis(name)["fault"] is not None))
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "recover", "args": {"timeout_s": 1}})
        self.assertNotEqual(code, 200)
        self.assertIn("RecoveryError", body["error"])
        ax = self.axis(name)
        self.assertFalse(ax["recovery"]["ok"])
        self.assertIsNotNone(ax["fault"])
        self.assertEqual(self.call("/api/wb/call", {"axis": name, "fn": "recover",
                                                    "args": {"timeout_s": 0}})[0], 400)  # fmt: skip
        line.replug()


class SimOnlyServerTest(unittest.TestCase):
    """--sim-only: the About panel says so, and neither page can open a serial port."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=0.05, sim_only=True)
        cls.wb = Workbench(poll_interval_s=0.05, sim_kwargs={"time_scale": 20}, sim_only=True,
                           port_lister=cls.fail_listing)
        cls.server = GuiServer(0, cls.ctl, token=TOKEN, workbench=cls.wb)
        cls.server.log_errors = False
        cls.base = f"http://127.0.0.1:{cls.server.port}"
        cls.thread = threading.Thread(target=cls.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        cls.thread.start()

    @staticmethod
    def fail_listing() -> list[dict[str, Any]]:
        raise AssertionError("listed serial ports in sim-only mode")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.wb.close()
        cls.ctl.close()

    def tearDown(self) -> None:
        if self.ctl.snapshot()["connected"]:
            self.ctl.disconnect()

    def test_about_reports_demo_mode(self) -> None:
        code, about = http_call(self.base, "/api/about")
        self.assertEqual(code, 200)
        self.assertTrue(about["sim_only"])
        code, ports = http_call(self.base, "/api/wb/ports")
        self.assertEqual((code, ports["ports"], ports["sim_only"]), (200, [], True))

    def test_single_axis_page_refuses_a_serial_port_but_accepts_the_simulator(self) -> None:
        code, body = http_call(self.base, "/api/connect", {"port": "COM3", "baud": 115200, "slave": 1, "simulate": False})
        self.assertEqual(code, 409, body)
        self.assertIn("--sim-only", body["error"])
        self.assertFalse(self.ctl.snapshot()["connected"])
        code, body = http_call(self.base, "/api/connect", {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True})
        self.assertEqual(code, 200, body)

    def test_workbench_refuses_a_serial_port_but_accepts_a_simulator(self) -> None:
        code, body = http_call(self.base, "/api/wb/connect", {"port": "COM3"})
        self.assertEqual(code, 409, body)
        code, body = http_call(self.base, "/api/wb/connect", {"simulate": True, "sim_setup": "slider"})
        self.assertEqual(code, 200, body)
        self.wb.disconnect()

    def test_main_passes_sim_only_to_both_controller_and_workbench(self) -> None:
        with mock.patch.object(server_module, "Controller") as controller, \
                mock.patch.object(server_module, "Workbench") as workbench, \
                mock.patch.object(server_module, "GuiServer", side_effect=OSError("port busy")), \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["--sim-only"]), 1)
        controller.assert_called_once_with(stroke=None, sim_only=True)
        workbench.assert_called_once_with(sim_only=True)

    def test_version_flag(self) -> None:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as cm:
            main(["--version"])
        self.assertEqual(cm.exception.code, 0)
        self.assertIn(__version__, out.getvalue())
        self.assertIn("iai-gui", out.getvalue())


# ====================================================================== GUI polish: coordinated tab, report, packaging


CORE_RUN = BACKEND == "native"  # the core's coordinated run has no live trace: its trace comes with the result

COORD_ENDPOINTS = (("/api/wb/coord/plan", {}), ("/api/wb/coord/run", {}), ("/api/wb/coord/stop", {}),
                   ("/api/wb/coord/status", None), ("/api/wb/report", {}))


def _frames_9900(line: Any, start: int) -> list[bytes]:
    """FC16 frames to 0x9900 (the direct-move block) on a simulated line since request index ``start``."""
    return [f for _t, f in line.requests[start:] if len(f) > 4 and f[1] == 0x10 and f[2:4] == b"\x99\x00"]


class CoordinatedApiTest(_WorkbenchCase):
    """The Coordinated (experimental) tab's API on two simulated sliders: plan, refuse, run, stop, status."""

    def pair(self) -> tuple[str, str]:
        x, y = self.sim("xy")
        self.assertNotEqual(x.split(":")[0], y.split(":")[0])  # one slider per port
        return x, y

    def line_of(self, name: str) -> Any:
        return self.wb.ports[name.split(":")[0]].transport

    def circle(self, x: str, y: str, radius: float = 5.0, speed: float = 15.0, **shape: Any) -> dict[str, Any]:
        px, py = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        return {"axes": {"x": x, "y": y}, "speed": speed, "accel_g": 0.3, "chord_error": 0.05,
                "shape": {"kind": "circle", "center": [px - radius, py], "radius": radius, **shape}}

    def status(self, since: int = 0) -> dict[str, Any]:
        return self.ok(f"/api/wb/coord/status?since={since}")

    def wait_running(self) -> dict[str, Any]:
        """Running with something to show: a live trace, or (the core reports its trace when the run
        ends) a run that has been going for a second."""

        def going() -> bool:
            st = self.status()
            return st["state"] == "running" and bool(st["trace"] or (CORE_RUN and (st["elapsed_s"] or 0) > 1.0))

        self.assertTrue(wait_until(going, 10), self.status())
        return self.status()

    def wait_end(self, timeout: float = 20.0) -> dict[str, Any]:
        self.assertTrue(wait_until(lambda: self.status()["state"] not in ("starting", "running"), timeout),
                        self.status()["state"])
        return self.status()

    def start(self, body: dict[str, Any]) -> dict[str, Any]:
        plan = self.ok("/api/wb/coord/plan", body)
        self.assertTrue(plan["passed"], plan)
        run = self.ok("/api/wb/coord/run", {"plan_id": plan["plan_id"]})
        self.assertIn(run["state"], ("running", "starting", "completed"), run)  # a short run can end within the request
        return plan

    def test_plan_passes_with_path_bounds_and_readiness_and_sends_nothing(self) -> None:
        x, y = self.pair()
        lines = [self.line_of(x), self.line_of(y)]
        marks = [len(line.requests) for line in lines]
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y))
        self.assertTrue(plan["passed"])
        self.assertRegex(plan["plan_id"], r"^p\d+$")
        self.assertEqual(plan["axes"], {"x": x, "y": y})
        self.assertEqual(plan["units"], {"x": "mm", "y": "mm"})
        self.assertEqual((plan["shape"], plan["mode"]), ("circle", "stream"))
        self.assertGreater(plan["steps"], 10)
        self.assertGreater(len(plan["path"]), 10)
        self.assertIsNone(plan["lead_in"])  # the circle passes through the current position
        self.assertEqual(plan["path"][0], plan["current"])
        self.assertAlmostEqual(plan["planned_s"], plan["duration_s"] + plan["lead_in_s"], places=3)
        self.assertEqual(plan["bounds"], {"x": [0.0, 150.0], "y": [0.0, 150.0]})
        self.assertEqual((plan["ready"], plan["not_ready"], plan["reasons"], plan["error"]), (True, [], [], None))
        self.assertIn("stream plan", plan["summary"])
        for line, mark in zip(lines, marks):
            self.assertEqual(_frames_9900(line, mark), [])  # a dry run: no move frame
        coord = self.ok("/api/wb/state")["coord"]
        self.assertEqual((coord["state"], coord["plan_id"], coord["axes"]), ("planned", plan["plan_id"], [x, y]))
        stored = self.status()
        self.assertEqual((stored["state"], stored["plan"]["plan_id"]), ("planned", plan["plan_id"]))
        events = [e["message"] for e in self.ok("/api/wb/state")["events"]]
        self.assertTrue(any(m.startswith(f"Coordinated plan {plan['plan_id']} passed: circle over x={x}") for m in events))

    def test_plan_refused_with_reasons_largest_speed_and_a_preview(self) -> None:
        x, y = self.pair()
        lines = [self.line_of(x), self.line_of(y)]
        marks = [len(line.requests) for line in lines]
        body = self.circle(x, y, speed=400.0)
        plan = self.ok("/api/wb/coord/plan", body)  # a refusal is an answer (HTTP 200), not an error
        self.assertFalse(plan["passed"])
        self.assertIsNone(plan["plan_id"])
        self.assertTrue(any("centripetal acceleration" in r for r in plan["reasons"]), plan["reasons"])
        self.assertGreater(plan["max_speed"], 1.0)
        self.assertLess(plan["max_speed"], 400.0)
        self.assertIn("nothing sent", plan["error"])
        self.assertGreaterEqual(len(plan["path"]), 40)  # the asked circle, sampled for the plot
        for line, mark in zip(lines, marks):
            self.assertEqual(_frames_9900(line, mark), [])
        self.assertEqual(self.status()["state"], "idle")
        here = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        nowhere = self.ok("/api/wb/coord/plan", {**body, "speed": 10.0, "shape": {"kind": "line", "to": list(here)}})
        self.assertFalse(nowhere["passed"])
        self.assertIsNone(nowhere["max_speed"])  # a ValueError: no speed helps
        self.assertTrue(nowhere["reasons"])

    def test_every_shape_plans(self) -> None:
        x, y = self.pair()
        px, py = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        base = {"axes": {"x": x, "y": y}, "speed": 10.0, "accel_g": 0.3, "chord_error": 0.05}
        for shape, mode in (({"kind": "line", "to": [px + 5, py + 5]}, "sync"),
                            ({"kind": "rectangle", "corner": [px, py], "width": 5, "height": -4}, "sync"),
                            ({"kind": "polygon", "center": [px - 5, py], "radius": 5, "sides": 6}, "sync"),
                            ({"kind": "circle", "center": [px + 20, py], "radius": 5, "turns": 0.5, "ccw": False},
                             "stream")):
            plan = self.ok("/api/wb/coord/plan", {**base, "shape": shape})
            self.assertTrue(plan["passed"], (shape, plan.get("reasons")))
            self.assertEqual(plan["mode"], mode, shape)
        self.assertIsNotNone(plan["lead_in"])  # the last circle starts away from the axes: a lead-in first
        self.assertGreater(plan["lead_in_s"], 0)

    def test_bad_input_unknown_axis_and_the_same_axis_twice(self) -> None:
        x, y = self.pair()
        good = self.circle(x, y)
        for change in ({"axes": None}, {"axes": {"x": x}}, {"axes": {"x": x, "y": x}}, {"shape": None},
                       {"shape": {"kind": "spiral"}}, {"shape": {**good["shape"], "radius": -1}},
                       {"shape": {**good["shape"], "center": [1, "a"]}}, {"shape": {**good["shape"], "turns": 50}},
                       {"shape": {"kind": "polygon", "center": [70, 70], "radius": 5, "sides": 2}},
                       {"shape": {"kind": "rectangle", "corner": [70, 70], "width": 0, "height": 5}},
                       {"speed": 0}, {"speed": "fast"}, {"accel_g": -1}, {"chord_error": 50}, {"speed": float("inf")}):
            code, body = self.call("/api/wb/coord/plan", {**good, **change})
            self.assertEqual(code, 400, (change, body))
        code, body = self.call("/api/wb/coord/plan", {**good, "axes": {"x": x, "y": "NOPE:1"}})
        self.assertEqual(code, 404, body)
        self.assertEqual(self.call("/api/wb/coord/run", {})[0], 400)

    def test_new_endpoints_need_the_token_and_a_loopback_host(self) -> None:
        for path, data in COORD_ENDPOINTS:
            self.assertEqual(self.call(path, data, token="wrong")[0], 403, path)
            self.assertEqual(self.call(path, data, token="")[0], 403, path)
            self.assertEqual(self.call(path, data, host="evil.example:80")[0], 403, path)
            self.assertEqual(self.call(path, data, host=f"192.168.1.5:{self.server.port}")[0], 403, path)
        self.assertEqual(self.call("/api/wb/coord/status")[0], 200)
        self.assertEqual(self.call("/api/wb/coord/status", host=f"localhost:{self.server.port}")[0], 200)

    def test_run_to_completion_with_a_trace_and_a_result_then_the_plan_is_spent(self) -> None:
        x, y = self.pair()
        self.assertEqual(self.call("/api/wb/coord/run", {"plan_id": "p999"})[0], 409)  # no such plan
        plan = self.start(self.circle(x, y))
        self.assertEqual(self.axis(x)["busy"], "coordinated run")
        st = self.wait_end()
        self.assertEqual(st["state"], "completed", st)
        self.assertEqual((st["plan_id"], st["axes"], st["units"]), (plan["plan_id"], {"x": x, "y": y},
                                                                   {"x": "mm", "y": "mm"}))  # fmt: skip
        trace = st["trace"]
        self.assertGreater(len(trace), 20)
        self.assertEqual(st["next"], len(trace))
        self.assertTrue(all(len(p) == 4 and p[3] in (0, 1) for p in trace))
        self.assertTrue(all(p[0] >= 0 for p in trace))
        self.assertGreater(trace[-1][0], trace[0][0])
        self.assertEqual(self.status(st["next"])["trace"], [])
        result = st["result"]
        self.assertTrue(result["completed"])
        self.assertIsNone(result["error"])
        for key in ("max_deviation", "mean_deviation", "frames", "skipped_frames", "late_frames", "worst_late_ms",
                    "worst_skew_ms", "duration_s", "planned_s", "samples", "sample_spacing_ms", "final", "warnings",
                    "summary"):
            self.assertIn(key, result)
        self.assertGreater(result["frames"], 10)
        self.assertLess(result["max_deviation"], 2.0)
        self.assertIsNone(self.axis(x)["busy"])
        code, body = self.call("/api/wb/coord/run", {"plan_id": plan["plan_id"]})  # one run per plan
        self.assertEqual(code, 409, body)
        self.assertIn("Plan again", body["error"])
        events = [e["message"] for e in self.ok("/api/wb/state")["events"]]
        self.assertTrue(any(m.startswith("Coordinated run") and "completed in" in m for m in events), events)

    def test_rectangle_runs_to_completion_and_ends_at_its_corner(self) -> None:
        x, y = self.pair()
        px, py = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        plan = self.start({"axes": {"x": x, "y": y}, "speed": 15.0, "accel_g": 0.3, "chord_error": 0.05,
                           "shape": {"kind": "rectangle", "corner": [px, py], "width": 6.0, "height": 4.0}})
        self.assertEqual((plan["shape"], plan["mode"]), ("rectangle", "sync"))
        self.assertEqual(plan["path"][0], plan["current"])  # starts where the axes are: no lead-in
        self.assertIsNone(plan["lead_in"])
        xs, ys = [p[0] for p in plan["path"]], [p[1] for p in plan["path"]]
        self.assertAlmostEqual(max(xs) - min(xs), 6.0, places=2)
        self.assertAlmostEqual(max(ys) - min(ys), 4.0, places=2)
        st = self.wait_end()
        self.assertEqual(st["state"], "completed", st)
        self.assertTrue(st["result"]["completed"])
        self.assertTrue(st["trace"])
        fx, fy = st["result"]["final"]
        self.assertAlmostEqual(fx, px, delta=0.15)
        self.assertAlmostEqual(fy, py, delta=0.15)
        self.assertEqual(self.ok("/api/wb/state")["coord"]["state"], "completed")
        report = self.ok("/api/wb/report", {})["text"]
        self.assertIn(f"last plan {plan['plan_id']}: rectangle over x={x}, y={y}, passed (sync", report)
        self.assertIn("completed in", report)

    def test_stop_all_mid_run_stops_it_and_both_axes_come_to_rest(self) -> None:
        x, y = self.pair()
        self.start(self.circle(x, y, radius=10.0, speed=10.0))
        self.wait_running()
        self.ok("/api/wb/stop", {})
        st = self.wait_end()
        self.assertEqual(st["state"], "stopped", st)
        self.assertFalse(st["result"]["completed"])
        self.assertTrue(st["error"])
        for name in (x, y):
            self.wait_still(name)
        self.assertEqual(self.ok("/api/wb/state")["coord"]["state"], "stopped")

    def test_coord_stop_endpoint_ends_a_run(self) -> None:
        self.assertEqual(self.ok("/api/wb/coord/stop", {})["stopped"], [])  # no group yet
        x, y = self.pair()
        self.start(self.circle(x, y, radius=10.0, speed=10.0))
        self.wait_running()
        body = self.ok("/api/wb/coord/stop", {})
        self.assertEqual(sorted(body["stopped"]), sorted([x, y]))
        self.assertEqual(self.wait_end()["state"], "stopped")

    def test_servo_off_then_run_is_refused_before_anything_is_sent(self) -> None:
        x, y = self.pair()
        self.run_fn(x, "servo", on=False)
        self.assertTrue(wait_until(lambda: self.axis(x)["status"]["servo_on"] is False, 3))
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y))
        self.assertTrue(plan["passed"])
        self.assertFalse(plan["ready"])
        self.assertIn(f"{x}: servo off", plan["not_ready"])
        mark = len(self.line_of(y).requests)
        code, body = self.call("/api/wb/coord/run", {"plan_id": plan["plan_id"]})
        self.assertEqual(code, 409, body)
        self.assertEqual((body["kind"], body["sent"]), ("refused", False))
        self.assertEqual(_frames_9900(self.line_of(y), mark), [])
        self.assertEqual(self.status()["state"], "refused")

    def test_an_axis_in_a_run_refuses_requests_at_once_and_the_poll_leaves_it_alone(self) -> None:
        x, y = self.pair()
        self.start(self.circle(x, y, radius=10.0, speed=10.0))
        self.wait_running()
        started = time.monotonic()
        code, body = self.call("/api/wb/call", {"axis": x, "fn": "move_to",
                                                "args": {"mm": 50, "speed_mm_s": 10, "accel_g": 0.1}})
        self.assertEqual(code, 409, body)
        self.assertIn("coordinated run", body["error"])
        self.assertLess(time.monotonic() - started, 2.0)  # at once, not after the 10 s lock wait
        for path, data in (("/api/wb/group", {"op": "home_all", "axes": [y]}),
                           ("/api/wb/coord/plan", self.circle(x, y)),
                           ("/api/wb/connect", {"simulate": True, "sim_setup": "slider"}),
                           ("/api/wb/port", {"port": y.split(":")[0], "fn": "scan", "args": {"slaves": "1"}})):
            code, body = self.call(path, data)
            self.assertEqual(code, 409, (path, body))
        self.assertEqual(self.call(f"/api/wb/positions?axis={x}")[0], 409)
        read_at = self.wb.axes[x].read_at
        time.sleep(0.5)
        self.assertEqual(self.wb.axes[x].read_at, read_at)  # the poll skips the axes of the run
        self.assertEqual(self.status()["state"], "running")
        self.ok("/api/wb/stop", {})
        self.wait_end()

    def test_disconnect_during_a_run_stops_it_first(self) -> None:
        x, y = self.pair()
        self.start(self.circle(x, y, radius=10.0, speed=10.0))
        self.wait_running()
        body = self.ok("/api/wb/disconnect", {"port": x.split(":")[0]})
        self.assertIn(x, body["axes"])
        self.assertNotEqual(body["axes"][x]["stop_confirmed"], False)
        st = self.wait_end()
        self.assertIn(st["state"], ("stopped", "failed"), st)
        self.wait_still(y)

    def test_a_run_that_was_accepted_but_not_begun_does_not_start_after_stop(self) -> None:
        x, y = self.pair()
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y, radius=10.0, speed=10.0))
        self.assertTrue(plan["passed"], plan)
        slot = self.wb.axes[x]
        held, release = threading.Event(), threading.Event()

        def hold() -> None:  # a back-up or a read holds x for a moment: the run thread waits for its lock
            with slot.lock:
                held.set()
                release.wait(15)

        holder = threading.Thread(target=hold, daemon=True)
        holder.start()
        self.assertTrue(held.wait(3))
        self.addCleanup(lambda: (release.set(), holder.join(3)))
        out: dict[str, Any] = {}
        runner = threading.Thread(target=lambda: out.update(r=self.call("/api/wb/coord/run", {"plan_id": plan["plan_id"]})),
                                  daemon=True)
        runner.start()

        def accepted() -> bool:
            ctx = self.wb.coord._ctx
            return ctx is not None and ctx.thread is not None and ctx.thread.is_alive()

        self.assertTrue(wait_until(accepted, 3))
        time.sleep(0.1)
        mark = len(self.line_of(y).requests)
        self.ok("/api/wb/stop", {})  # after Run was accepted, before the run began
        release.set()
        runner.join(8)
        code, body = out["r"]
        self.assertEqual(code, 409, body)
        self.assertIn("STOP was pressed before the run started", body["error"])
        self.assertEqual(_frames_9900(self.line_of(y), mark), [])
        self.assertEqual(self.status()["state"], "planned")  # nothing was sent; the plan can still be run
        for name in (x, y):
            self.assertFalse(self.axis(name)["status"]["moving"])

    @unittest.skipUnless(CORE_RUN, "the start check belongs to the core's group")
    def test_a_stop_while_the_start_is_checked_ends_the_run_before_it_begins(self) -> None:
        x, y = self.pair()
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y, radius=10.0, speed=10.0))
        self.assertTrue(plan["passed"], plan)
        group = self.wb.coord._plan_group
        read, release = threading.Event(), threading.Event()
        positions = group.positions

        def slow_positions() -> dict[str, float]:  # the start check is reading the axes: Run has passed every STOP test
            read.set()
            release.wait(15)
            return positions()

        self.addCleanup(release.set)
        mark = len(self.line_of(y).requests)
        out: dict[str, Any] = {}
        with mock.patch.object(group, "positions", slow_positions):
            runner = threading.Thread(target=lambda: out.update(r=self.call("/api/wb/coord/run",
                                                                            {"plan_id": plan["plan_id"]})), daemon=True)
            runner.start()
            self.assertTrue(read.wait(5))
            self.ok("/api/wb/stop", {})
            release.set()
            runner.join(10)
        code, body = out["r"]
        self.assertEqual(code, 409, body)
        self.assertEqual((body["kind"], body["sent"]), ("refused", False))
        self.assertIn("STOP was pressed before the run started", body["error"])
        self.assertEqual(self.status()["state"], "refused")
        self.assertEqual(_frames_9900(self.line_of(y), mark), [])
        for name in (x, y):
            self.assertFalse(self.axis(name)["status"]["moving"])

    def test_a_plan_is_not_run_from_a_place_other_than_where_it_was_made(self) -> None:
        x, y = self.pair()
        px = self.axis(x)["status"]["position_mm"]
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y, radius=10.0, speed=10.0))
        self.assertTrue(plan["passed"], plan)
        self.run_fn(x, "move_to", mm=px + 25, speed_mm_s=100, accel_g=0.3)  # an axis is moved after Plan
        self.wait_at(x, px + 25)
        mark = len(self.line_of(y).requests)
        code, body = self.call("/api/wb/coord/run", {"plan_id": plan["plan_id"]})
        self.assertEqual(code, 409, body)  # the core would plan a lead-in the page never showed: refused
        self.assertEqual((body["kind"], body["sent"]), ("refused", False))
        self.assertIn("plan again", body["error"])
        self.assertEqual(_frames_9900(self.line_of(y), mark), [])
        self.assertEqual(self.status()["state"], "refused")
        for name in (x, y):
            self.wait_still(name)
        self.assertAlmostEqual(self.axis(x)["status"]["position_mm"], px + 25, delta=0.2)

    def test_a_group_move_cannot_reach_an_axis_of_a_run_through_another_axis(self) -> None:
        x, y = self.pair()
        [z] = self.sim("slider")
        self.start(self.circle(x, y, radius=10.0, speed=10.0))
        self.wait_running()
        code, body = self.call("/api/wb/group", {"op": "move_all", "axes": [z], "targets": {x: 100.0}, "speed": {x: 20.0}})
        self.assertEqual(code, 400, body)  # the checks for a run were made for z, not for x
        code, body = self.call("/api/wb/group", {"op": "move_all", "targets": {x: 100.0}, "speed": {x: 20.0}})
        self.assertEqual(code, 409, body)
        self.assertIn("coordinated run", body["error"])
        self.assertEqual(self.status()["state"], "running")
        self.ok("/api/wb/stop", {})
        self.wait_end()


class WorkbenchPolishTest(_WorkbenchCase):
    """Self-test, non-volatile counts, catalogue text, connect error kinds, safe disconnect, the test report."""

    def test_catalogue_splits_evidence_from_the_plain_text_and_adds_the_new_parts(self) -> None:
        cat = self.ok("/api/wb/functions")
        by_name = {f["name"]: f for f in cat["axis"]}
        move = by_name["move_to"]
        self.assertEqual(move["plain"], "Absolute move. A deceleration different from the acceleration uses the "
                                        "command-profile path.")  # fmt: skip
        self.assertEqual(move["evidence"], ["VERIFIED on the EC gripper and the PCON"])
        self.assertIn("(VERIFIED on the EC gripper and the PCON)", move["desc"])  # desc unchanged
        speed = by_name["change_speed"]
        self.assertTrue(speed["plain"].endswith("or with no move running."), speed["plain"])
        self.assertIn("(same target)", speed["plain"])
        self.assertEqual(speed["evidence"], ["VERIFIED on the EC-GRBP8M gripper and the PCON-CB"])
        for spec in cat["axis"]:
            self.assertIn("plain", spec)
            self.assertIsInstance(spec["evidence"], list)
            self.assertNotRegex(spec["plain"], r"\b(VERIFIED|UNVERIFIED|ON-WIRE)\b", spec["name"])
        check = by_name["self_check"]
        self.assertEqual((check["group"], check["readonly"], check["inputs"]), ("Read", True, []))
        self.assertEqual(check["label"], "Self-test (iai-check, read-only)")
        self.assertEqual(cat["nv"], {"cycles": 100000, "warn_per_hour": 60,
                                     "skip_note": workbench_module.NV_SKIP_NOTE})  # fmt: skip
        setups = {s["name"]: s["label"] for s in cat["sim_setups"]}
        self.assertEqual(setups["xy"], "XY pair: two sliders, one per port")

    def test_self_test_result_and_state(self) -> None:
        [name] = self.sim("gripper")
        result = self.run_fn(name, "self_check")
        self.assertEqual((result["result"], result["passed"]), ("PASS", True))
        self.assertEqual(set(result["counts"]), {"PASS", "WARN", "FAIL", "SKIP"})
        self.assertEqual(sum(result["counts"].values()), len(result["items"]))
        self.assertEqual(set(result["items"][0]), {"name", "result", "detail"})
        self.assertIn("link", [i["name"] for i in result["items"]])
        check = self.axis(name)["check"]
        self.assertEqual(set(check), {"result", "t", "counts"})  # the state carries no items
        self.assertEqual(check["counts"], result["counts"])

    def test_nonvolatile_write_counts(self) -> None:
        [name] = self.sim("slider")
        before = self.axis(name)
        self.assertIsInstance(before["nv_writes"], int)
        row = self.ok(f"/api/wb/positions?axis={name}")["rows"][1]
        new = round(row["position_mm"] + (1.0 if row["position_mm"] < 75 else -1.0), 2)
        self.run_fn(name, "write_position_table", n=1, position_mm=new)
        after = self.axis(name)
        self.assertEqual(after["nv_writes"], before["nv_writes"] + 1)
        self.assertEqual(after["nv_writes_last_hour"], before["nv_writes_last_hour"] + 1)
        again = self.run_fn(name, "write_position_table", n=1, position_mm=new)
        self.assertTrue(again["skipped"])
        self.assertEqual(self.axis(name)["nv_writes"], after["nv_writes"])  # a skipped write is not counted

    def test_unchanged_parameter_write_is_skipped_and_not_counted(self) -> None:
        [name] = self.sim("slider")
        params = self.ok(f"/api/wb/parameters?axis={name}")["parameters"]
        param = next(p for p in params if p.get("writable") and not p.get("dangerous")
                     and isinstance(p.get("value"), (int, float)) and not isinstance(p.get("value"), bool))
        before = self.axis(name)["nv_writes"]
        result = self.run_fn(name, "write_parameter", key=str(param["number"]), value=param["value"])
        self.assertTrue(result["skipped"], result)
        self.assertFalse(result["changed"])
        self.assertEqual(result["before"], result["after"])
        self.assertEqual(self.axis(name)["nv_writes"], before)

    def test_ports_say_whether_the_baud_was_detected_and_whether_it_is_an_iai_adapter(self) -> None:
        [name] = self.sim("slider")
        port = next(p for p in self.ok("/api/wb/state")["ports"] if p["name"] == name.split(":")[0])
        self.assertEqual((port["baud_detected"], port["is_iai"]), (False, None))  # a simulator

    def test_connect_errors_carry_a_kind(self) -> None:
        cases = (
            (OSError("could not open port 'COM3': PermissionError(13, 'Access is denied.', None, 5)"), "in_use",
             "COM3 is in use by another program. Close IA-OS"),
            (OSError(16, "Device or resource busy"), "in_use", "in use by another program"),
            (OSError("could not open port 'COM3': FileNotFoundError(2, 'The system cannot find the file specified.', "
                     "None, 2)"), "not_found", "COM3 is not there (adapter unplugged?). Press Refresh."),
            (OSError("something odd"), "open_failed", "Cannot open COM3: OSError: something odd. Close IA-OS"),
        )
        for exc, kind, text in cases:
            with mock.patch.object(workbench_module, "Port", side_effect=exc):
                code, body = self.call("/api/wb/connect", {"port": "COM3"})
            self.assertEqual(code, 502, body)
            self.assertEqual((body["ok"], body["kind"], body["port"]), (False, kind, "COM3"), body)
            self.assertIn(text, body["error"])

        class Silent:
            baudrate = None

            def scan(self, slaves: Any) -> list[Any]:
                return []

            def close(self) -> None:
                pass

        with mock.patch.object(workbench_module, "Port", return_value=Silent()):
            code, body = self.call("/api/wb/connect", {"port": "COM3", "slaves": "1-2"})
        self.assertEqual(code, 409, body)
        self.assertEqual(body["kind"], "no_answer")
        self.assertIn("COM3: no axis answered at addresses 1..2 at any baud rate", body["error"])
        self.assertFalse(self.wb.ports)

    def test_classify_open_error_by_type_errno_and_text(self) -> None:
        self.assertEqual(classify_open_error(PermissionError(13, "Permission denied"), "/dev/ttyUSB0",
                                             windows=False)[0], "no_permission")  # fmt: skip
        kind, message = classify_open_error(OSError(13, "Permission denied"), "/dev/ttyUSB0", windows=False)
        self.assertEqual(kind, "no_permission")
        self.assertEqual(message, "No permission to open /dev/ttyUSB0. On Linux add your user to the dialout group "
                                  "and log in again.")  # fmt: skip
        self.assertEqual(classify_open_error(PermissionError(13, "x"), "COM3", windows=True)[0], "in_use")
        self.assertEqual(classify_open_error(FileNotFoundError(2, "No such file or directory"), "/dev/ttyUSB9",
                                             windows=False)[0], "not_found")  # fmt: skip
        wrapped = RuntimeError("open failed")
        wrapped.__cause__ = OSError(16, "Device or resource busy")
        self.assertEqual(classify_open_error(wrapped, "COM3")[0], "in_use")

    def test_disconnect_leaves_each_axis_safe_and_says_what_it_did(self) -> None:
        [moving] = self.sim("slider")
        [still] = self.sim("gripper")
        self.ok("/api/wb/call", {"axis": moving, "fn": "move_to", "args": {"mm": 140, "speed_mm_s": 2, "accel_g": 0.1}})
        self.assertTrue(wait_until(lambda: self.axis(moving)["status"]["moving"], 3))
        body = self.ok("/api/wb/disconnect", {"port": moving.split(":")[0]})
        result = body["axes"][moving]
        self.assertEqual((result["stopped"], result["stop_confirmed"], result["servo"]), (True, True, "on"))
        self.assertTrue(result["note"].startswith("stopped, servo left on"), result["note"])
        body = self.ok("/api/wb/disconnect", {"port": still.split(":")[0]})
        self.assertEqual(body["axes"][still], {"stopped": False, "stop_confirmed": None, "home_released": False,
                                               "pio_restored": False, "servo": "on",
                                               "note": "at rest, servo left on"})  # fmt: skip
        events = [e["message"] for e in self.ok("/api/wb/state")["events"]]
        self.assertIn(f"{still.split(':')[0]}: disconnected ({still} at rest, servo left on)", events)

    def test_disconnect_stops_an_axis_it_cannot_read_fresh(self) -> None:
        """The axis lock stays busy (a command holds it), so the disconnect gets no fresh status under it. Before, it
        decided from the cached status (at rest here, but the axis may have been moved since) and sent no STOP; now it
        STOPs whenever it could not read fresh."""
        [name] = self.sim("gripper")
        port = name.split(":")[0]
        slot = self.wb.axes[name]
        self.assertTrue(wait_until(lambda: (slot.status or {}).get("moving") is False, 3))
        frames: list[bytes] = []
        slot.ec.bus.frame_logger = lambda direction, frame: frames.append(bytes(frame)) if direction == "tx" else None
        held, done = threading.Event(), threading.Event()

        def hold() -> None:
            with slot.lock:
                held.set()
                done.wait(10)

        holder = threading.Thread(target=hold, daemon=True)
        holder.start()
        self.addCleanup(holder.join, 10)
        self.addCleanup(done.set)
        self.assertTrue(held.wait(3))
        with mock.patch.object(workbench_module, "DISCONNECT_LOCK_S", 0.05):
            body = self.ok("/api/wb/disconnect", {"port": port})
        done.set()
        result = body["axes"][name]
        self.assertEqual((result["stopped"], result["stop_confirmed"]), (True, True), result)
        self.assertTrue(result["note"].startswith("stopped"), result["note"])
        self.assertTrue(any(f[1:6] == b"\x05\x04\x2c\xff\x00" for f in frames), frames)  # STOP coil ON
        events = [e["message"] for e in self.ok("/api/wb/state")["events"]]
        self.assertIn(f"{name}: busy, no fresh status before disconnect: STOP anyway", events)

    def test_report_text_has_the_version_system_units_self_test_events_and_page_errors(self) -> None:
        [name] = self.sim("gripper")
        self.run_fn(name, "self_check")
        long_agent = "Mozilla/5.0 " + "x" * 400
        errors = [{"t": time.time() - i, "message": f"page error {i}\nsecond line"} for i in range(35)]
        body = self.ok("/api/wb/report", {"browser": long_agent, "screen": "1366x768, scale 1.25, window 1093x614",
                                          "page": "workbench", "client_errors": errors})  # fmt: skip
        self.assertRegex(body["filename"], r"^iai-gui-report-\d{8}-\d{6}\.txt$")
        text = body["text"]
        self.assertTrue(text.startswith("IAI Actuator Driver test report\n"), text[:80])
        for part in (f"Version   IAI Actuator Driver {__version__} (source checkout)", "Mode      normal: serial ports",
                     "OS        ", "Python    ", "pyserial", "Screen    1366x768, scale 1.25, window 1093x614",
                     "EC-GRBP8M-20", "firmware", "self-test: PASS at", "Coordinated (experimental)",
                     "Errors and warnings", "Page errors (newest first, up to 30)", "page error 0 second line",
                     "Event log (oldest first, last 100)", f"{name.split(':')[0]}: connected"):
            self.assertIn(part, text)
        browser = next(line for line in text.splitlines() if line.startswith("Browser   "))
        self.assertEqual(len(browser), len("Browser   ") + 300)  # client strings are cut to 300 characters
        page_errors = text.split("Page errors (newest first, up to 30)\n")[1].split("\n\n")[0].splitlines()
        self.assertEqual(len(page_errors), 30)
        self.assertNotIn("```", text)
        self.assertNotIn("\u2014", text)  # no em dash

    def test_report_without_client_data_says_none(self) -> None:
        text, filename = build_report(self.wb, {}, sim_only=True)
        self.assertIn("Mode      demo (--sim-only): simulators only", text)
        self.assertIn("Browser   not given", text)
        self.assertIn("Ports (0)\n  none", text)
        self.assertIn("Page errors (newest first, up to 30)\n  none", text)
        self.assertTrue(filename.endswith(".txt"))


class PioHandBackTest(unittest.TestCase):
    """A unit PIO had at connect gets control back at disconnect (simulators only)."""

    def test_disconnect_hands_control_back_to_pio(self) -> None:
        wb = Workbench(poll_interval_s=0.05, sim_kwargs={"time_scale": 20, "pmsl": False})
        try:
            [name] = [a for p in wb.connect(simulate=True, sim_setup="slider")["connected"] for a in p["axes"]]
            unit = wb.ports[name.split(":")[0]].transport[1]
            self.assertFalse(unit.pmsl)
            wb.call(name, "enable_modbus_control")
            self.assertTrue(unit.pmsl)
            result = wb.disconnect(name.split(":")[0])["axes"][name]
            self.assertTrue(result["pio_restored"])
            self.assertIn("control handed back to PIO", result["note"])
            self.assertFalse(unit.pmsl)
        finally:
            wb.close()


class ResourcesTest(unittest.TestCase):
    """Static files and documents through apps/gui/resources.py: source checkout and the iai-gui.exe bundle."""

    def test_source_checkout(self) -> None:
        root = Path(__file__).resolve().parents[2]
        self.assertFalse(resources.frozen())
        self.assertEqual(resources.bundle_root(), root)
        self.assertEqual(resources.gui_file("workbench.html"), root / "apps" / "gui" / "workbench.html")
        self.assertEqual(resources.doc_file("docs/DRIVER.md"), root / "docs" / "DRIVER.md")
        self.assertEqual(resources.user_dir(), root)
        self.assertEqual(resources.run_as(), "source checkout")
        self.assertEqual(workbench_module.BACKUP_DIR, root / "backups")

    def test_frozen_bundle(self) -> None:
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(sys, "frozen", True, create=True), \
                mock.patch.object(sys, "_MEIPASS", folder, create=True):
            self.assertTrue(resources.frozen())
            self.assertEqual(resources.bundle_root(), Path(folder))
            self.assertEqual(resources.gui_file("coord.js"), Path(folder) / "gui" / "coord.js")
            self.assertEqual(resources.doc_file("README.md"), Path(folder) / "README.md")
            with mock.patch.object(resources, "_writable", return_value=True):
                self.assertEqual(resources.user_dir(), Path(sys.executable).resolve().parent)
            self.assertEqual(resources.run_as(), "iai-gui.exe")
        with mock.patch.object(sys, "frozen", True, create=True):  # frozen by another tool: not the bundle
            self.assertFalse(resources.frozen())

    def test_the_users_own_data_folder(self) -> None:
        """Finding 13: an installed package writes to the user's own folder, never to the folder it was started in."""
        with mock.patch.object(sys, "platform", "win32"), mock.patch.dict(os.environ, {"LOCALAPPDATA": "/appdata/Local"}):
            self.assertEqual(resources.data_dir(), Path("/appdata/Local") / "IAI Actuator Driver")
        with mock.patch.object(sys, "platform", "darwin"), mock.patch.object(Path, "home", return_value=Path("/Users/x")):
            self.assertEqual(resources.data_dir(), Path("/Users/x/Library/Application Support/IAI Actuator Driver"))
        with mock.patch.object(sys, "platform", "linux"), mock.patch.object(Path, "home", return_value=Path("/home/x")), \
                mock.patch.dict(os.environ, {"XDG_DATA_HOME": "/xdg"}):
            self.assertEqual(resources.data_dir(), Path("/xdg/iai-actuator-driver"))
            del os.environ["XDG_DATA_HOME"]
            self.assertEqual(resources.data_dir(), Path("/home/x/.local/share/iai-actuator-driver"))

    def test_user_dir_is_the_exe_folder_only_when_it_can_be_written_to(self) -> None:
        with mock.patch.object(resources, "_checkout", return_value=False):  # an installed package
            self.assertEqual(resources.user_dir(), resources.data_dir())
            self.assertNotEqual(resources.user_dir(), Path.cwd())  # whatever folder a shortcut starts it in
            self.assertEqual(resources.run_as(), "installed package")
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(sys, "frozen", True, create=True), \
                mock.patch.object(sys, "_MEIPASS", folder, create=True):
            with mock.patch.object(resources, "_writable", return_value=True):
                self.assertEqual(resources.user_dir(), Path(sys.executable).resolve().parent)
            with mock.patch.object(resources, "_writable", return_value=False):  # an exe under Program Files
                self.assertEqual(resources.user_dir(), resources.data_dir())
        with tempfile.TemporaryDirectory() as folder:
            self.assertTrue(resources._writable(Path(folder)))
            self.assertFalse(resources._writable(Path(folder) / "missing"))

    def test_server_reads_static_files_from_the_bundle_and_answers_404_for_missing_ones(self) -> None:
        ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=0.05)
        wb = Workbench(poll_interval_s=0.05)
        srv = GuiServer(0, ctl, token=TOKEN, workbench=wb)
        srv.log_errors = False
        thread = threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{srv.port}"
        try:
            with tempfile.TemporaryDirectory() as folder, mock.patch.object(sys, "frozen", True, create=True), \
                    mock.patch.object(sys, "_MEIPASS", folder, create=True):
                (Path(folder) / "gui").mkdir()
                (Path(folder) / "gui" / "coord.js").write_bytes(b"// bundled\n")
                (Path(folder) / "README.md").write_bytes(b"bundled readme\n")
                self.assertEqual(fetch(base, "/coord.js")[::2], (200, "// bundled\n"))
                self.assertEqual(fetch(base, "/README.md")[::2], (200, "bundled readme\n"))
                self.assertEqual(fetch(base, "/")[0], 404)  # not in this bundle
                self.assertEqual(fetch(base, "/docs/DRIVER.md")[0], 404)
                about = http_call(base, "/api/about")[1]
                self.assertEqual(about["run_as"], "iai-gui.exe")
                self.assertFalse({d["title"]: d for d in about["docs"]}["Changelog"]["available"])
            self.assertEqual(http_call(base, "/api/about")[1]["run_as"], "source checkout")
        finally:
            srv.shutdown()
            srv.server_close()
            wb.close()
            ctl.close()


class ServerStartTest(unittest.TestCase):
    """main(): the browser opens after the bind unless --no-browser; a bind failure says what to do."""

    def run_main(self, argv: list[str], *, opened: threading.Event | None = None) -> tuple[int, str, Any]:
        fake = mock.MagicMock()
        fake.port = 8799
        fake.launch_url.return_value = "http://127.0.0.1:8799/#k=abcdef0123456789"

        def serve(**_kw: Any) -> None:
            if opened is not None:
                opened.wait(3)
            raise KeyboardInterrupt

        fake.serve_forever.side_effect = serve
        out = io.StringIO()
        with mock.patch.object(server_module, "Controller"), mock.patch.object(server_module, "Workbench"), \
                mock.patch.object(server_module, "GuiServer", return_value=fake), \
                mock.patch.object(server_module, "install_console_handler") as console_handler, \
                mock.patch.object(server_module.webbrowser, "open",
                                  side_effect=lambda url: opened.set() if opened else None) as browser, \
                contextlib.redirect_stdout(out):
            code = main(argv)
        console_handler.assert_called_once_with(fake.shutdown_safely)  # closing the console window runs the safe shutdown
        fake.shutdown_safely.assert_called_once_with()  # and so does the end of main() (Ctrl+C)
        return code, out.getvalue(), browser

    def test_the_browser_opens_by_default(self) -> None:
        opened = threading.Event()
        code, out, browser = self.run_main(["--port", "8799"], opened=opened)
        self.assertEqual(code, 0)
        url = "http://127.0.0.1:8799/#k=abcdef0123456789"  # the access key follows the #
        if os.name == "nt":
            browser.assert_called_once_with(url)
        else:  # elsewhere the browser gets a private file that forwards to the address, not the address itself
            [(args, _)] = browser.call_args_list  # one call: (positional arguments, keywords)
            target = args[0]
            self.assertTrue(target.startswith("file://"), target)
        self.assertIn("Opening the browser (start with --no-browser to skip)", out)
        self.assertIn(f"Open {url}", out)
        self.assertIn("Quit with the Quit button in the page or Ctrl+C", out)

    def test_no_browser(self) -> None:
        code, out, browser = self.run_main(["--port", "8799", "--no-browser"])
        self.assertEqual(code, 0)
        time.sleep(0.5)
        browser.assert_not_called()
        self.assertNotIn("Opening the browser", out)

    def test_bind_failure_names_the_way_out_and_waits_for_enter_when_frozen(self) -> None:
        err = io.StringIO()
        with mock.patch.object(server_module, "Controller"), mock.patch.object(server_module, "Workbench"), \
                mock.patch.object(server_module, "GuiServer", side_effect=OSError("address in use")), \
                mock.patch.object(server_module.resources, "frozen", return_value=True), \
                mock.patch("builtins.input", side_effect=EOFError) as enter, contextlib.redirect_stderr(err):
            self.assertEqual(main(["--port", "8780"]), 1)
        enter.assert_called_once_with("Press Enter to close.")
        self.assertIn("Another iai-gui may be running: open http://127.0.0.1:8780/ or start with --port 8781.",
                      err.getvalue())
        with mock.patch.object(server_module, "Controller"), mock.patch.object(server_module, "Workbench"), \
                mock.patch.object(server_module, "GuiServer", side_effect=OSError("address in use")), \
                mock.patch("builtins.input") as enter, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main([]), 1)
        enter.assert_not_called()  # a source checkout does not wait


class PageContractTest(_WorkbenchCase):
    """The page contract between the backend, workbench.html / workbench.js and coord.js / coord.css."""

    def test_workbench_page_has_the_new_ids_and_the_script_order(self) -> None:
        status, _ctype, html = fetch(self.base, "/")
        self.assertEqual(status, 200)
        for hook in ("tabCoord", "viewCoord", "startHere", "startTitle", "startDemo", "startHide", "startBtn",
                     "reportBtn", "reportDlg", "reportText", "reportCopy", "reportSave", "reportClose",
                     "logCopyReport", "apiToggle", "connMsg", "posNvNote", "paramNvNote", "selfCheckBtn",
                     "selfCheckCard", "selfCheckTable"):
            self.assertIn(f'id="{hook}"', html, hook)
        self.assertIn('data-view="viewCoord"', html)
        self.assertIn('href="/docs/GETTING_STARTED.md"', html)
        self.assertIn('href="/coord.css"', html)
        order = [html.index(f'src="{s}"') for s in ("/workbench.js", "/coord.js", "/about.js")]
        self.assertEqual(order, sorted(order))
        header = html[html.index('<header class="appbar">'):html.index("</header>")]
        self.assertIn('id="stopAll"', header)

    def test_every_file_the_pages_reference_is_served(self) -> None:
        for page in ("/", "/classic"):
            status, _ctype, html = fetch(self.base, page)
            self.assertEqual(status, 200, page)
            refs = re.findall(r'(?:src|href)="(/[^"#?]*)"', html)
            self.assertTrue(refs, page)
            for ref in refs:
                self.assertEqual(fetch(self.base, ref)[0], 200, f"{page} -> {ref}")

    def test_workbench_script_exposes_wb_and_sends_the_report(self) -> None:
        js = fetch(self.base, "/workbench.js")[2]
        self.assertIn("window.WB", js)
        self.assertIn("/api/wb/report", js)
        self.assertNotIn("innerHTML", js)

    def test_coordinated_script_and_style_are_served(self) -> None:
        status, ctype, js = fetch(self.base, "/coord.js")
        self.assertEqual(status, 200)
        self.assertIn("javascript", ctype)
        for part in ("/api/wb/coord/plan", "/api/wb/coord/run", "/api/wb/coord/status", "WB.stopAll"):
            self.assertIn(part, js)
        for hook in ("coordForm", "coordX", "coordY", "coordShape", "coordSpeed", "coordAccel", "coordChord",
                     "coordPlan", "coordRun", "coordStop", "coordPrepare", "coordReady", "coordPlot", "coordPlanOut",
                     "coordRunOut"):
            self.assertIn(hook, js)
        self.assertNotIn("innerHTML", js)
        self.assertNotIn("keydown", js)  # STOP ALL keys stay with workbench.js
        status, ctype, _css = fetch(self.base, "/coord.css")
        self.assertEqual(status, 200)
        self.assertIn("text/css", ctype)


# ====================================================================== security review (gui_security_review.md)
#
# Each class names the findings it covers. The review's numbers: 1 browser connection limit, 2 console close,
# 3 unbounded coordinated plan, 4 blocking console, 5 key in the page, 6 two STOP paths, 7 port names, 8 status age,
# 9 socket timeout and Content-Length, 10 raw console lines, 11 headers and Origin, 12 bundle, 13 where it writes,
# 14 test report, 15 STOP reply wait.


def raw_request(port: int, request: bytes, *, timeout: float = 3.0) -> bytes:
    """Send raw bytes and return what the server answers until it closes the connection (or ``timeout`` passes)."""
    chunks: list[bytes] = []
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        s.sendall(request)
        try:
            while True:
                data = s.recv(65536)
                if not data:
                    break
                chunks.append(data)
        except OSError:  # a timeout: the server kept the connection open
            pass
    return b"".join(chunks)


def status_of(raw: bytes) -> int:
    return int(raw.split(b" ", 2)[1])


def browser_call(port: int, method: str, path: str, data: Any = None, headers: dict[str, str] | None = None, *,
                 token: str = TOKEN) -> tuple[int, dict[str, str], bytes]:
    """One request with exactly the headers given (a browser's Origin and Sec-Fetch-Site included)."""
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        head = {"X-EC-Token": token, "Content-Type": "application/json", **(headers or {})}
        conn.request(method, path, None if data is None else json.dumps(data), head)
        r = conn.getresponse()
        return r.status, {k.lower(): v for k, v in r.getheaders()}, r.read()
    finally:
        conn.close()


class RequestHardeningTest(_WorkbenchCase):
    """Findings 4, 9, 10 and 11: Origin and Sec-Fetch-Site, headers, socket timeout, Content-Length, the console."""

    def test_a_page_from_another_origin_is_refused(self) -> None:
        port = self.server.port
        own = f"http://127.0.0.1:{port}"
        for headers in ({"Origin": "http://evil.example"}, {"Origin": "null"}, {"Origin": f"http://127.0.0.1:{port + 1}"},
                        {"Sec-Fetch-Site": "cross-site"}, {"Sec-Fetch-Site": "same-site"},
                        {"Origin": own, "Sec-Fetch-Site": "cross-site"}):
            code, _h, body = browser_call(port, "POST", "/api/wb/connect", {"simulate": True}, headers)
            self.assertEqual(code, 403, headers)
            self.assertIn("own page", json.loads(body)["error"])
            self.assertEqual(browser_call(port, "GET", "/api/wb/state", None, headers)[0], 403, headers)
        self.assertFalse(self.wb.ports)  # none of the refused requests ran
        for headers in ({}, {"Origin": own, "Sec-Fetch-Site": "same-origin"}, {"Sec-Fetch-Site": "none"},
                        {"Origin": f"http://localhost:{port}", "Host": f"localhost:{port}"}):
            self.assertEqual(browser_call(port, "GET", "/api/wb/state", None, headers)[0], 200, headers)
        code, _h, body = browser_call(port, "POST", "/api/wb/connect", {"simulate": True},
                                      {"Origin": own, "Sec-Fetch-Site": "same-origin"})
        self.assertEqual(code, 200, body)
        # A page load is no API request (it needs no key): a link on another site may open the GUI.
        self.assertEqual(browser_call(port, "GET", "/", None, {"Sec-Fetch-Site": "cross-site"})[0], 200)

    def test_every_answer_carries_the_security_headers_and_no_version_numbers(self) -> None:
        for method, path in (("GET", "/"), ("GET", "/workbench.js"), ("GET", "/token.js"), ("GET", "/docs/DRIVER.md"),
                             ("GET", "/api/wb/state"), ("GET", "/nope"), ("OPTIONS", "/api/wb/state"),
                             ("POST", "/api/wb/stop")):
            code, headers, _body = browser_call(self.server.port, method, path, {} if method == "POST" else None)
            self.assertIn(code, (200, 404, 501), (method, path))
            self.assertEqual(headers["cross-origin-resource-policy"], "same-origin", (method, path))
            self.assertEqual(headers["cross-origin-opener-policy"], "same-origin", (method, path))
            for part in ("default-src 'self'", "base-uri 'none'", "form-action 'none'", "object-src 'none'",
                         "frame-ancestors 'none'"):
                self.assertIn(part, headers["content-security-policy"], (method, path))
            self.assertEqual((headers["x-frame-options"], headers["referrer-policy"], headers["cache-control"]),
                             ("DENY", "no-referrer", "no-store"), (method, path))
            self.assertEqual(headers["server"], "iai-gui", (method, path))  # no product, Python or version numbers

    def test_a_negative_content_length_is_refused_at_once(self) -> None:
        port = self.server.port
        request = (f"POST /api/wb/connect HTTP/1.0\r\nHost: 127.0.0.1:{port}\r\nX-EC-Token: {TOKEN}\r\n"
                   "Content-Type: application/json\r\nContent-Length: -1\r\n\r\n").encode()
        started = time.monotonic()
        raw = raw_request(port, request, timeout=4)
        self.assertEqual(status_of(raw), 400, raw[:200])  # was: read(-1), no answer until the client closes
        self.assertLess(time.monotonic() - started, 3)
        self.assertFalse(self.wb.ports)

    def test_an_idle_connection_does_not_keep_its_thread(self) -> None:
        self.assertEqual(server_module.Handler.timeout, server_module.SOCKET_TIMEOUT_S)
        self.assertEqual(server_module.SOCKET_TIMEOUT_S, 15.0)
        with mock.patch.object(server_module.Handler, "timeout", 0.3):
            started = time.monotonic()
            with socket.create_connection(("127.0.0.1", self.server.port), timeout=5) as s:
                self.assertEqual(s.recv(100), b"")  # the server closed the idle connection
            self.assertLess(time.monotonic() - started, 3)

    def test_console_lines_are_escaped_and_cut(self) -> None:
        self.assertEqual(server_module.printable("GET /\x1b[2J\x00 end\r\n"), "GET /\\x1b[2J\\x00 end\\x0d\\x0a")
        self.assertEqual(server_module.printable("a" * 1000), "a" * 297 + "...")
        self.assertEqual(server_module.printable("é€ plain"), "é€ plain")

    def test_a_request_line_with_control_bytes_reaches_the_console_escaped(self) -> None:
        lines: list[str] = []
        log = server_module.ConsoleLog(write=lines.append)
        port = self.server.port
        with mock.patch.object(self.server, "console", log), mock.patch.object(self.server, "log_errors", True):
            raw = raw_request(port, b"GET /\x1b[31mred HTTP/1.0\r\nHost: 127.0.0.1:%d\r\n\r\n" % port)
            self.assertEqual(status_of(raw), 404)
            self.assertTrue(wait_until(lambda: any("red" in line for line in lines), 3), lines)
        line = next(line for line in lines if "red" in line)
        self.assertNotIn("\x1b", line)
        self.assertIn("\\x1b[31mred", line)

    def test_a_blocked_console_never_holds_a_request(self) -> None:
        gate = threading.Event()
        written: list[str] = []

        def write(line: str) -> None:  # a console window with a selection: every write waits
            gate.wait(10)
            written.append(line)

        log = server_module.ConsoleLog(write=write, limit=5)
        try:
            with mock.patch.object(self.server, "console", log), mock.patch.object(self.server, "log_errors", True):
                started = time.monotonic()
                for _ in range(30):  # refused requests are logged: each used to write to the console itself
                    self.assertEqual(self.call("/api/wb/state", token="wrong")[0], 403)
                self.assertLess(time.monotonic() - started, 5)
                self.assertGreater(log.dropped, 10)  # a full queue drops lines instead of waiting
        finally:
            gate.set()
        self.assertTrue(wait_until(lambda: any("dropped while the console was blocked" in line for line in written), 5),
                        written)


class OneStopForBothPagesTest(_WorkbenchCase):
    """Finding 6: STOP ALL of the workbench also stops the single-axis page's axis, and its STOP stops the workbench."""

    def setUp(self) -> None:
        [self.axis_name] = self.sim("slider")
        self.ready(self.axis_name)
        self.ok("/api/connect", {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True})
        self.addCleanup(lambda: self.ctl.snapshot()["connected"] and self.ctl.disconnect())
        self.ok("/api/servo", {"on": True})

    def start_both(self) -> None:
        self.ok("/api/wb/call", {"axis": self.axis_name, "fn": "move_to",
                                 "args": {"mm": 140, "speed_mm_s": 1, "accel_g": 0.1}})
        self.ok("/api/move", {"position_mm": 140, "speed_mm_s": 10, "accel_g": 0.1})
        self.assertTrue(wait_until(lambda: self.axis(self.axis_name)["status"]["moving"]
                                   and self.ok("/api/status")["status"]["moving"], 5), "both axes should be moving")

    def assert_both_stop(self) -> None:
        self.assertTrue(wait_until(lambda: not self.axis(self.axis_name)["status"]["moving"], 5), "workbench axis runs on")
        self.assertTrue(wait_until(lambda: not self.ok("/api/status")["status"]["moving"], 5), "single-axis page runs on")

    def test_the_workbench_stop_all_stops_the_single_axis_page_too(self) -> None:
        self.start_both()
        body = self.ok("/api/wb/stop", {})
        self.assertEqual(body["errors"], {})
        self.assertIn(self.axis_name, body["stopped"])
        self.assertIn("single-axis page", body["stopped"])
        self.assert_both_stop()

    def test_the_group_stop_all_does_the_same(self) -> None:
        self.start_both()
        self.assertIn("single-axis page", self.ok("/api/wb/group", {"op": "stop_all"})["stopped"])
        self.assert_both_stop()

    def test_the_single_axis_stop_stops_the_workbench_too(self) -> None:
        self.start_both()
        self.ok("/api/stop", {})
        self.assert_both_stop()

    def test_a_stop_of_one_workbench_axis_stays_that_axis_alone(self) -> None:
        self.start_both()
        self.ok("/api/wb/stop", {"axis": self.axis_name})
        self.assertTrue(wait_until(lambda: not self.axis(self.axis_name)["status"]["moving"], 5))
        self.assertTrue(self.ok("/api/status")["status"]["moving"])
        self.ok("/api/stop", {})

    def test_without_the_other_page_nothing_changes(self) -> None:
        self.ctl.disconnect()
        body = self.ok("/api/wb/stop", {})
        self.assertNotIn("single-axis page", body["stopped"])
        self.assertEqual(body["errors"], {})
        self.ok("/api/stop", {})  # the single-axis STOP with only workbench axes connected: they are stopped, no error
        self.wb.disconnect()
        self.assertEqual(self.call("/api/stop", {})[0], 409)  # nothing connected anywhere: "Not connected" as before
        self.assertEqual(self.ok("/api/wb/stop", {})["stopped"], [])


class StopReplyTest(_WorkbenchCase):
    """Finding 15: the reply to STOP, pause or resume does not wait for the axis lock."""

    def test_a_stop_reply_does_not_wait_for_a_held_axis_lock(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        slot = self.wb.axes[name]
        held, release = threading.Event(), threading.Event()

        def hold() -> None:  # a plan, a back-up or a run holds the axis for seconds
            with slot.lock:
                held.set()
                release.wait(10)

        holder = threading.Thread(target=hold, daemon=True)
        holder.start()
        self.assertTrue(held.wait(3))
        try:
            for call in (lambda: self.call("/api/wb/call", {"axis": name, "fn": "stop"}),
                         lambda: self.call("/api/wb/call", {"axis": name, "fn": "pause"}),
                         lambda: self.call("/api/wb/stop", {"axis": name})):
                started = time.monotonic()
                code, body = call()
                self.assertEqual(code, 200, body)
                self.assertLess(time.monotonic() - started, 0.8)  # the reply used to wait 1 s for the lock
        finally:
            release.set()
            holder.join(3)


class PortNameTest(_WorkbenchCase):
    """Finding 7: one port-name check for both pages, and only a port the PC lists is opened."""

    BAD = (r"\\attacker\share\x", r"\\.\COM3", "/dev/../etc/passwd", "/dev/null", "/etc/passwd", "COM0", "COM1000",
           "COM3:1", "/dev/serial/by-id/..", "http://evil.example/")

    def test_the_workbench_opens_only_a_listed_serial_port_name(self) -> None:
        with mock.patch.object(workbench_module, "Port") as port_class:
            for name in self.BAD:
                code, body = self.call("/api/wb/connect", {"port": name})
                self.assertEqual(code, 400, (name, body))
                self.assertIn("not a valid serial port name", body["error"])
            for name in ("COM9", "/dev/ttyUSB0"):  # a good name that the PC does not list
                code, body = self.call("/api/wb/connect", {"port": name})
                self.assertEqual((code, body.get("kind")), (404, "not_found"), (name, body))
                self.assertIn("not in the list of serial ports", body["error"])
            self.assertEqual(self.call("/api/wb/connect", {"port": 5})[0], 400)
            port_class.assert_not_called()  # nothing reached the serial driver

    def test_a_listed_name_is_opened_as_the_listing_spells_it(self) -> None:
        class Silent:
            baudrate = None

            def scan(self, slaves: Any) -> list[Any]:
                return []

            def close(self) -> None:
                pass

        with mock.patch.object(workbench_module, "Port", return_value=Silent()) as port_class:
            code, body = self.call("/api/wb/connect", {"port": " com3 "})
        self.assertEqual(code, 409, body)  # no axis answered: the port itself was opened
        self.assertEqual(port_class.call_args.args[0], "COM3")

    def test_an_unreadable_listing_does_not_block_a_valid_name(self) -> None:
        with mock.patch.object(self.wb, "port_lister", side_effect=OSError("enumeration failed")), \
                mock.patch.object(self.wb, "_listed", []), mock.patch.object(workbench_module, "Port") as port_class:
            port_class.side_effect = OSError("no such port")
            code, body = self.call("/api/wb/connect", {"port": "COM8"})
        self.assertEqual(code, 502, body)  # it got as far as opening the port
        port_class.assert_called_once()

    def test_the_single_axis_page_uses_the_same_check(self) -> None:
        for name in self.BAD:
            code, body = self.call("/api/connect", {"port": name, "simulate": False})
            self.assertEqual(code, 400, (name, body))
            self.assertIn("not a valid serial port name", body["error"])
        self.assertTrue(backend.check_port_name("COM12"))
        self.assertEqual(backend.check_port_name("/dev/serial/by-id/usb-FTDI_FT232R-if00-port0"),
                         "/dev/serial/by-id/usb-FTDI_FT232R-if00-port0")
        for name in ("/dev/cu.usbserial-1410", "/dev/cuaU0", "/dev/rfcomm0", "/dev/ttyUSB0", "/dev/ttyACM1"):
            self.assertEqual(backend.check_port_name(name), name)


class SafeShutdownTest(unittest.TestCase):
    """Finding 2: the safe shutdown runs from Ctrl+C, from the Quit button and from a closed console window."""

    def setUp(self) -> None:
        self.ctl = Controller(session_factory=FakeAxis.factory, poll_interval_s=0.05)
        self.wb = Workbench(poll_interval_s=0.05, sim_kwargs={"time_scale": 20}, port_lister=lambda: [])
        self.server = GuiServer(0, self.ctl, token=TOKEN, workbench=self.wb)
        self.server.log_errors = False
        self.base = f"http://127.0.0.1:{self.server.port}"
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        self.thread.start()
        self.addCleanup(self.cleanup)

    def cleanup(self) -> None:
        if self.thread.is_alive():
            self.server.shutdown()
        self.server.server_close()
        self.wb.close()
        self.ctl.close()

    def moving_everywhere(self) -> str:
        axis = http_call(self.base, "/api/wb/connect", {"simulate": True, "sim_setup": "slider"})[1]["connected"][0]["axes"][0]
        http_call(self.base, "/api/wb/call", {"axis": axis, "fn": "servo", "args": {"on": True}})
        http_call(self.base, "/api/wb/call", {"axis": axis, "fn": "home"})
        self.assertTrue(wait_until(lambda: (self.wb.axes[axis].status or {}).get("homed") is True
                                   and not self.wb.axes[axis].home_started, 10))
        self.assertEqual(http_call(self.base, "/api/wb/call", {"axis": axis, "fn": "move_to",
                                                               "args": {"mm": 140, "speed_mm_s": 1, "accel_g": 0.1}})[0], 200)
        http_call(self.base, "/api/connect", {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True})
        http_call(self.base, "/api/servo", {"on": True})
        http_call(self.base, "/api/move", {"position_mm": 140, "speed_mm_s": 10, "accel_g": 0.1})
        self.assertTrue(wait_until(lambda: (self.wb.axes[axis].status or {}).get("moving"), 5))
        return axis

    def test_quit_stops_everything_leaves_the_units_and_ends_the_server(self) -> None:
        self.assertEqual(http_call(self.base, "/api/quit", {}, token="wrong")[0], 403)  # the key is needed
        self.assertTrue(self.thread.is_alive())
        self.moving_everywhere()
        code, body = http_call(self.base, "/api/quit", {})
        self.assertEqual((code, body["quit"]), (200, True), body)
        self.thread.join(5)
        self.assertFalse(self.thread.is_alive(), "the server should have ended")
        self.assertFalse(self.wb.ports)  # every port closed ...
        self.assertFalse(self.ctl.snapshot()["connected"])  # ... and the single-axis page's axis too
        messages = [e["message"] for e in self.wb._events]
        stop = next(i for i, m in enumerate(messages) if m.startswith("STOP all:"))  # STOP ALL went out first ...
        gone = next(i for i, m in enumerate(messages) if "disconnected" in m)  # ... then the port was left
        self.assertLess(stop, gone, messages)
        self.server.shutdown_safely()  # a second call (Ctrl+C after Quit) is a no-op

    def test_shutdown_stops_before_it_disconnects_and_runs_once(self) -> None:
        order: list[str] = []
        with mock.patch.object(self.wb, "stop", side_effect=lambda *a, **k: order.append("stop") or {"stopped": []}), \
                mock.patch.object(self.wb, "close", side_effect=lambda: order.append("close wb")), \
                mock.patch.object(self.ctl, "close", side_effect=lambda: order.append("close classic")):
            self.server.shutdown_safely()
            self.server.shutdown_safely()
        self.assertEqual(order, ["stop", "close wb", "close classic"])

    def test_console_events_run_the_shutdown_and_ctrl_c_does_not(self) -> None:
        calls: list[int] = []
        handler = server_module.console_handler(lambda: calls.append(1))
        for event in (2, 5, 6):  # close, logoff, shutdown
            self.assertTrue(handler(event), event)
        self.assertEqual(len(calls), 3)
        for event in (0, 1):  # Ctrl+C and Ctrl+Break: left to Python (KeyboardInterrupt, the finally block of main())
            self.assertFalse(handler(event), event)
        self.assertEqual(len(calls), 3)

        def failing() -> None:
            raise RuntimeError("boom")

        self.assertTrue(server_module.console_handler(failing)(2))  # a failing shutdown must not stop the exit

    @unittest.skipUnless(os.name == "nt", "the console control handler is a Windows API")
    def test_the_handler_is_registered_with_windows(self) -> None:
        with mock.patch("ctypes.WinDLL") as dll:
            dll.return_value.SetConsoleCtrlHandler.return_value = 1
            self.assertTrue(server_module.install_console_handler(lambda: None))
            registered = dll.return_value.SetConsoleCtrlHandler.call_args.args
        self.assertEqual(registered[1], True)
        self.assertIsNotNone(server_module._console_callback)  # kept alive for the life of the process
        with mock.patch("ctypes.WinDLL") as dll:
            dll.return_value.SetConsoleCtrlHandler.return_value = 0
            self.assertFalse(server_module.install_console_handler(lambda: None))

    def test_the_pages_have_a_quit_button_and_the_documents_say_how_to_quit(self) -> None:
        for page in ("workbench.html", "index.html"):
            self.assertIn('id="quitBtn"', (Path(server_module.__file__).parent / page).read_text(encoding="utf-8"), page)
        about = (Path(server_module.__file__).parent / "about.js").read_text(encoding="utf-8")
        self.assertIn('"/api/quit"', about)
        self.assertIn("window.confirm(", about)
        readme = (Path(server_module.__file__).parent / "packaging" / "README.txt").read_text(encoding="utf-8")
        self.assertIn("Quit button", readme)
        self.assertNotIn("Close the console window to quit", readme)

    def test_nothing_starts_after_the_shutdown_has_begun(self) -> None:
        axis = self.moving_everywhere()
        gate = threading.Event()
        with mock.patch.object(self.wb, "close", side_effect=lambda: gate.wait(10)):  # a slow shutdown
            closer = threading.Thread(target=self.server.shutdown_safely, daemon=True)
            closer.start()
            try:
                self.assertTrue(wait_until(self.server.closing.is_set, 3))
                for path, data in (("/api/wb/connect", {"simulate": True, "sim_setup": "slider"}),
                                   ("/api/wb/call", {"axis": axis, "fn": "move_to",
                                                     "args": {"mm": 10, "speed_mm_s": 10, "accel_g": 0.1}}),
                                   ("/api/wb/group", {"op": "home_all", "axes": [axis]}),
                                   ("/api/wb/coord/run", {"plan_id": "p1"}),
                                   ("/api/connect", {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True}),
                                   ("/api/move", {"position_mm": 10, "speed_mm_s": 10, "accel_g": 0.1})):
                    code, body = http_call(self.base, path, data)
                    self.assertEqual(code, 503, (path, body))
                    self.assertIn("shutting down", body["error"])
                for path, data in (("/api/wb/stop", {}), ("/api/wb/group", {"op": "stop_all"}), ("/api/stop", {}),
                                   ("/api/wb/coord/stop", {}), ("/api/wb/call", {"axis": axis, "fn": "stop"})):
                    self.assertNotEqual(http_call(self.base, path, data)[0], 503, path)  # STOP is always answered
            finally:
                gate.set()
                closer.join(10)


class AccessKeyTest(_WorkbenchCase):
    """Finding 5: the access key is in the launch address after the #, never in a served page."""

    def test_no_served_file_holds_the_key(self) -> None:
        for path in list(server_module.STATIC_FILES) + list(server_module.DOC_FILES):
            status, _ctype, text = fetch(self.base, path)
            self.assertEqual(status, 200, path)
            self.assertNotIn(TOKEN, text, path)
            self.assertNotIn("__TOKEN__", text, path)
            self.assertNotIn("ec-token", text, path)
        self.assertEqual(fetch(self.base, "/api/about")[0], 403)  # and the API still needs it

    def test_launch_address_carries_the_key_after_a_hash(self) -> None:
        self.assertEqual(self.server.launch_url(), f"http://127.0.0.1:{self.server.port}/#k={TOKEN}")

    def test_both_pages_load_token_js_first_and_read_the_key_from_it(self) -> None:
        for page, script in (("/", "/workbench.js"), ("/classic", "/app.js")):
            html = fetch(self.base, page)[2]
            self.assertLess(html.index('src="/token.js"'), html.index(f'src="{script}"'), page)
        token_js = fetch(self.base, "/token.js")[2]
        for part in ("window.location.hash", "sessionStorage", "history.replaceState", "window.ECToken", "k=("):
            self.assertIn(part, token_js)
        self.assertNotIn("localStorage", token_js)  # the key is not kept beyond the tab
        for script in ("/workbench.js", "/app.js", "/about.js"):
            js = fetch(self.base, script)[2]
            self.assertIn("window.ECToken", js, script)
            self.assertNotIn("ec-token", js, script)

    def test_the_missing_key_message_says_what_to_do(self) -> None:
        self.assertIn("Open the address printed in", fetch(self.base, "/token.js")[2])
        self.assertIn("ends with #k=", fetch(self.base, "/token.js")[2])
        code, body = self.call("/api/wb/state", token="")
        self.assertEqual(code, 403)
        self.assertIn("#k=", body["error"])

    def test_without_windows_the_browser_gets_a_private_file_not_the_address(self) -> None:
        url = f"http://127.0.0.1:{self.server.port}/#k=abc123abc123abc1"
        with mock.patch.object(server_module.threading, "Timer") as timer:  # no cleanup thread in the test
            page = server_module.redirect_page(url)
        try:
            text = page.read_text(encoding="utf-8")
            self.assertIn(url, text)
            self.assertIn('name="referrer" content="no-referrer"', text)
            if os.name != "nt":
                self.assertEqual(page.stat().st_mode & 0o777, 0o600)
                self.assertEqual(page.parent.stat().st_mode & 0o777, 0o700)
            self.assertEqual(timer.call_args.args[:2], (60.0, shutil.rmtree))  # removed again after a minute
        finally:
            shutil.rmtree(page.parent, ignore_errors=True)
        with mock.patch.object(server_module.webbrowser, "open") as browser, \
                mock.patch.object(server_module, "redirect_page", return_value=Path(tempfile.gettempdir()) / "x" / "open.html"):
            server_module._open_browser(url, via_file=True)
            self.assertTrue(browser.call_args.args[0].startswith("file:///"), browser.call_args)
            self.assertNotIn("#k=", browser.call_args.args[0])  # the key is not on the browser's command line
            browser.reset_mock()
            server_module._open_browser(url, via_file=False)  # Windows: another account cannot read this command line
            browser.assert_called_once_with(url)


class CoordinatedBoundsTest(_WorkbenchCase):
    """Finding 3: a coordinated path is checked against the ranges, and its size, before any waypoint is planned."""

    def pair(self) -> tuple[str, str]:
        return tuple(self.sim("xy"))  # type: ignore[return-value]

    def circle(self, x: str, y: str, radius: float, **shape: Any) -> dict[str, Any]:
        px, py = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        return {"axes": {"x": x, "y": y}, "speed": 15.0, "accel_g": 0.3, "chord_error": 0.05,
                "shape": {"kind": "circle", "center": [px - radius, py], "radius": radius, **shape}}

    def test_extent_of_arcs_is_exact(self) -> None:
        center, radius = (10.0, 20.0), 5.0

        def box(turns: float, ccw: bool = True, start: float = 0.0) -> tuple[list[float], list[float]]:
            shape = {"kind": "circle", "center": center, "radius": radius, "turns": turns, "ccw": ccw, "start_deg": start}
            at = (center[0] + radius * math.cos(math.radians(start)), center[1] + radius * math.sin(math.radians(start)))
            return coordinated_module.path_extent(coordinated_module.build_path(shape, at), at)

        full = box(1.0)
        self.assertEqual([[round(v, 6) for v in axis] for axis in full], [[5.0, 15.0], [15.0, 25.0]])
        half = box(0.5)  # 0 to 180 degrees counter-clockwise: the upper half
        self.assertEqual([[round(v, 6) for v in axis] for axis in half], [[5.0, 15.0], [20.0, 25.0]])
        quarter = box(0.25)  # 0 to 90 degrees
        self.assertEqual([[round(v, 6) for v in axis] for axis in quarter], [[10.0, 15.0], [20.0, 25.0]])
        clockwise = box(0.25, ccw=False)  # 0 to -90 degrees
        self.assertEqual([[round(v, 6) for v in axis] for axis in clockwise], [[10.0, 15.0], [15.0, 20.0]])

    def test_the_step_count_is_closed_form_and_close_to_the_planners(self) -> None:
        shape = {"kind": "circle", "center": (0.0, 0.0), "radius": 20.0, "turns": 2.0, "ccw": True, "start_deg": None}
        estimate = coordinated_module.estimate_steps(shape, 0.05)
        self.assertEqual(coordinated_module.estimate_steps({**shape, "kind": "line"}, 0.05), 0)
        x, y = self.pair()
        plan = self.ok("/api/wb/coord/plan", self.circle(x, y, 20.0, turns=2.0))
        self.assertTrue(plan["passed"], plan.get("reasons"))
        self.assertLessEqual(abs(estimate - plan["steps"]), 0.5 * estimate, (estimate, plan["steps"]))

    def test_a_path_outside_the_ranges_is_refused_before_it_is_planned(self) -> None:
        x, y = self.pair()
        body = self.circle(x, y, 1e6, turns=20)  # the planner needed 35 s and 100s of MB for this one
        body["chord_error"] = 0.001
        with mock.patch.object(CoordinatedGroup, "plan", side_effect=AssertionError("the planner ran")):
            started = time.monotonic()
            plan = self.ok("/api/wb/coord/plan", body)
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertFalse(plan["passed"])
        self.assertTrue(any("outside its range" in r for r in plan["reasons"]), plan["reasons"])
        self.assertTrue(any(x in r for r in plan["reasons"]), plan["reasons"])
        self.assertIn("nothing planned or sent", plan["error"])
        self.assertGreaterEqual(len(plan["path"]), 40)  # the asked circle is still drawn
        self.assertEqual(self.ok("/api/wb/coord/status")["state"], "idle")
        # A smaller circle that still leaves the stroke is refused the same way (150 mm slider, centre 80 mm away).
        near = self.circle(x, y, 80.0)
        with mock.patch.object(CoordinatedGroup, "plan", side_effect=AssertionError("the planner ran")):
            self.assertFalse(self.ok("/api/wb/coord/plan", near)["passed"])

    def test_a_circle_with_too_many_waypoints_is_refused_before_it_is_planned(self) -> None:
        x, y = self.pair()
        body = self.circle(x, y, 20.0, turns=20)
        body["chord_error"] = 0.001  # about 6 300 waypoints
        with mock.patch.object(coordinated_module, "MAX_PLAN_STEPS", 500), \
                mock.patch.object(CoordinatedGroup, "plan", side_effect=AssertionError("the planner ran")):
            plan = self.ok("/api/wb/coord/plan", body)
        self.assertFalse(plan["passed"])
        self.assertTrue(any("waypoints" in r and "at most 500" in r for r in plan["reasons"]), plan["reasons"])
        self.assertTrue(self.ok("/api/wb/coord/plan", self.circle(x, y, 5.0))["passed"])  # a normal circle still plans

    def test_the_radius_field_is_limited_to_the_larger_stroke(self) -> None:
        js = fetch(self.base, "/coord.js")[2]
        self.assertIn("the larger stroke", js)
        self.assertIn("max: reach", js)


class StatusAgeAndStopPageTest(_WorkbenchCase):
    """Findings 1, 8 and 14 on the page side: request budget, STOP timeout and banner, status age, report dialog."""

    def js(self) -> str:
        return fetch(self.base, "/workbench.js")[2]

    def test_the_page_has_a_request_budget_that_stop_and_the_polls_bypass(self) -> None:
        js = self.js()
        self.assertIn("const BUDGET = 2;", js)
        bypass = re.search(r"const BYPASS = \[(.*?)\];", js).group(1)  # type: ignore[union-attr]
        for path in ("/api/wb/stop", "/api/wb/coord/stop", "/api/wb/state", "/api/wb/coord/status"):
            self.assertIn(f'"{path}"', bypass)
        for path in ("/api/wb/connect", "/api/wb/call", "/api/wb/group", "/api/wb/report", "/api/wb/coord/plan"):
            self.assertNotIn(f'"{path}"', bypass)  # everything else waits for one of the two places
        self.assertIn("if (inFlight >= BUDGET) await new Promise", js)
        self.assertIn("window.WB = {\n  api,", js.replace("\r\n", "\n"))  # coord.js calls the same queued api()

    def test_stop_is_cut_off_after_2_seconds_retried_once_and_leaves_a_banner(self) -> None:
        js = self.js()
        self.assertIn("const STOP_TIMEOUT_MS = 2000;", js)
        self.assertIn("new AbortController()", js)
        self.assertIn("attempt < 2", js)
        self.assertIn("STOP NOT CONFIRMED", js)
        self.assertIn("await stopRequest(", js)
        self.assertNotIn('await api("/api/wb/stop"', js)  # STOP never goes through the queue
        html = fetch(self.base, "/")[2]
        header = html[html.index('<header class="appbar">'):html.index("</header>")]
        self.assertIn('id="stopBanner"', header)  # in the sticky header: it cannot scroll away

    def test_an_old_status_is_marked_and_greyed(self) -> None:
        js = self.js()
        self.assertIn("const STALE_S = 1.0;", js)
        self.assertIn("ax.age_s > STALE_S", js)
        for part in ('classList.toggle("stale"', '$("axStale")', "status ${"):
            self.assertIn(part, js)
        self.assertIn('id="axStale"', fetch(self.base, "/")[2])
        self.assertIn(".axis-card.stale .ac-pos", fetch(self.base, "/workbench.css")[2])

    def test_the_event_log_button_opens_the_report_dialog_and_the_dialog_says_what_the_report_holds(self) -> None:
        self.assertIn('$("logCopyReport").addEventListener("click", openReport);', self.js())
        html = fetch(self.base, "/")[2]
        privacy = html[html.index('id="reportPrivacy"'):]
        privacy = privacy[:privacy.index("</p>")]
        for part in ("serial number", "operating system", "never holds the access key", "Read it before you send it"):
            self.assertIn(part, " ".join(privacy.split()))
        self.assertIn("BroadcastChannel", self.js())  # a second tab is announced

    def test_status_age_is_in_every_axis_summary(self) -> None:
        [name] = self.sim("slider")
        self.assertTrue(wait_until(lambda: self.axis(name)["age_s"] is not None, 3))
        self.assertLess(self.axis(name)["age_s"], 1.0)  # polled: fresh
        slot = self.wb.axes[name]
        with slot.lock:  # an operation holds the axis: the poll skips it and the age grows
            time.sleep(1.3)
            self.assertGreater(self.axis(name)["age_s"], 1.0)


class ReportCleaningTest(_WorkbenchCase):
    """Finding 14, server side: fields that come from outside cannot forge lines of the test report."""

    def test_fields_from_outside_cannot_forge_report_lines(self) -> None:
        [name] = self.sim("slider")
        slot = self.wb.axes[name]
        pslot = self.wb.ports[name.split(":")[0]]
        slot.info = {**slot.info, "model": "EC-S\nForged   line one", "serial_number": "12\r\n  Created   2099",
                     "firmware": "1.0\nVersion   forged"}
        slot.error = "bad\nPython    forged"
        slot.busy = "backup\nBrowser   forged"
        was = (pslot.simulated, pslot.is_iai, pslot.description)
        pslot.simulated, pslot.is_iai, pslot.description = False, False, "USB bridge\nCreated   forged"
        try:
            text = self.ok("/api/wb/report", {"browser": "UA\nOS        forged", "page": "p\nPorts (9)"})["text"]
        finally:
            pslot.simulated, pslot.is_iai, pslot.description = was
        for forged in ("\nForged", "\n  Created   2099", "\nVersion   forged", "\nPython    forged", "\nBrowser   forged",
                       "\nCreated   forged", "\nOS        forged", "\nPorts (9)"):
            self.assertNotIn(forged, text)
        for header in ("\nCreated   ", "\nVersion   ", "\nPython    ", "\nBrowser   ", "\nOS        "):
            self.assertEqual(text.count(header), 1, header)  # the real header lines, once each
        self.assertIn("EC-S Forged line one", text)  # the text is kept, on one line
        self.assertIn("USB bridge Created forged", " ".join(text.split()))


class BundleContentsTest(unittest.TestCase):
    """Finding 12: the exe bundles only the documents the About panel links, and no OpenSSL."""

    def test_the_spec_lists_the_documents_one_by_one_and_leaves_ssl_out(self) -> None:
        spec = (Path(server_module.__file__).parent / "iai-gui.spec").read_text(encoding="utf-8")
        listed_block = re.search(r"^DOCS = \((.*?)\)$", spec, re.M | re.S)
        self.assertIsNotNone(listed_block)
        listed = set(re.findall(r'"([A-Z_]+\.md)"', listed_block.group(1)))  # type: ignore[union-attr]
        linked = {rel.split("/", 1)[1] for _title, _url, rel in server_module.ABOUT_DOCS if rel.startswith("docs/")}
        self.assertEqual(listed, linked)
        self.assertNotIn('"*.md"', spec)  # no whole-folder pattern: a new document does not ship by itself
        excludes = re.search(r"excludes=\[(.*?)\]", spec).group(1)  # type: ignore[union-attr]
        for module in ("ssl", "_ssl"):
            self.assertIn(f'"{module}"', excludes)
        for name in linked:
            self.assertTrue((Path(server_module.__file__).resolve().parents[2] / "docs" / name).is_file(), name)



class MotionInputTest(_WorkbenchCase):
    """Input validation before any motion command: what is not a finite number never reaches a unit, on both pages."""

    BAD = (float("nan"), float("inf"), -float("inf"), True, "NaN", "1e999", "x", [], {})

    def test_a_bad_number_is_refused_and_no_move_frame_is_sent(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        line = self.wb.ports[name.split(":")[0]].transport
        mark = len(line.requests)
        good = {"mm": 40, "speed_mm_s": 20, "accel_g": 0.1}
        cases = {"move_to": good, "check_move": good, "move_by": {"delta_mm": 5, "speed_mm_s": 20, "accel_g": 0.1},
                 "jog": {"delta_mm": 1, "speed_mm_s": 10, "accel_g": 0.1},
                 "push": {"target_mm": 20, "band_mm": 5, "force_pct": 30, "direction": 1, "speed_mm_s": 20,
                          "accel_g": 0.1}}  # fmt: skip
        for fn, args in cases.items():
            for key in args:
                if key == "direction":
                    continue
                for bad in self.BAD:
                    code, body = self.call("/api/wb/call", {"axis": name, "fn": fn, "args": {**args, key: bad}})
                    self.assertEqual(code, 400, (fn, key, bad, body))
        for bad in self.BAD:  # a group move: one bad target refuses the whole group
            code, body = self.call("/api/wb/group", {"op": "move_all", "axes": [name], "targets": {name: bad},
                                                     "speed": {name: 10}})
            self.assertEqual(code, 400, (bad, body))
        self.assertEqual(_frames_9900(line, mark), [])  # nothing reached the unit
        self.assertFalse(self.axis(name)["status"]["moving"])

    def test_the_single_axis_page_refuses_a_bad_number_too(self) -> None:
        self.ok("/api/connect", {"port": "SIMULATED", "baud": 115200, "slave": 1, "simulate": True})
        self.addCleanup(lambda: self.ctl.snapshot()["connected"] and self.ctl.disconnect())
        self.ok("/api/servo", {"on": True})
        before = self.ok("/api/status")["status"]["position_mm"]
        for path, good in (("/api/move", {"position_mm": 100, "speed_mm_s": 20, "accel_g": 0.1}),
                           ("/api/jog", {"delta_mm": 1, "speed_mm_s": 10, "accel_g": 0.1})):
            for key in good:
                for bad in self.BAD:
                    code, body = self.call(path, {**good, key: bad})
                    self.assertEqual(code, 400, (path, key, bad, body))
        self.assertEqual(self.ok("/api/status")["status"]["position_mm"], before)
        self.assertFalse(self.ok("/api/status")["status"]["moving"])


# =========================================================================== PC / PIO control mode


class ControlModeFakeTest(_ControllerCase):
    """The control switch on the classic page: PMSS shown, at rest only, PIO stays chosen until PC is chosen."""

    def connect(self, **kwargs: Any) -> tuple[Controller, FakeAxis]:
        ctl = self.make(**kwargs)
        ctl.connect("SIMULATED", 115200, 1, True)
        return ctl, ctl._session

    def test_pc_then_pio_then_pc(self) -> None:
        ctl, fake = self.connect()
        self.assertFalse(ctl.snapshot()["status"]["modbus_control"])  # PIO after power-up
        self.assertFalse(ctl.snapshot()["pio_hold"])
        out = ctl.set_control(True)
        self.assertEqual((out["modbus_control"], out["changed"], out["pio_hold"]), (True, True, False))
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])
        self.assertEqual(fake.control_writes, 1)
        self.assertFalse(ctl.set_control(True)["changed"])  # already PC: nothing written
        self.assertEqual(fake.control_writes, 1)
        out = ctl.set_control(False)
        self.assertEqual((out["modbus_control"], out["changed"], out["pio_hold"]), (False, True, True))
        snap = ctl.snapshot()
        self.assertFalse(snap["status"]["modbus_control"])
        self.assertTrue(snap["pio_hold"])
        self.assertFalse(ctl.set_control(False)["changed"])
        self.assertTrue(ctl.set_control(True)["changed"])
        self.assertFalse(ctl.snapshot()["pio_hold"])
        self.assertTrue(any(m.startswith("Control mode: PIO") for m in messages(ctl)), messages(ctl))

    def test_commands_are_refused_while_pio_is_chosen_and_work_again_after_pc(self) -> None:
        ctl, fake = self.connect()
        ctl.set_control(True)
        ctl.set_control(False)
        writes = fake.control_writes
        for attempt in (lambda: ctl.servo(True), lambda: ctl.move(100, 20, 0.1), lambda: ctl.home(),
                        lambda: ctl.reset_alarm()):
            with self.assertRaises(BackendError) as caught:
                attempt()
            self.assertEqual(caught.exception.http_status, 409)
            self.assertIn("PIO control was chosen", str(caught.exception))
            self.assertIn("Switch to PC first", str(caught.exception))
        self.assertEqual(fake.control_writes, writes, "a command took PC control while PIO was chosen")
        self.assertFalse(fake.status()["modbus_control"])
        ctl.set_control(True)
        ctl.servo(True)
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])

    def test_without_the_switch_the_first_command_still_takes_control(self) -> None:
        ctl, fake = self.connect()
        ctl.servo(True)  # PIO found at connect, not chosen: the existing behaviour
        self.assertEqual(fake.control_writes, 1)
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])

    def test_refused_while_the_axis_moves(self) -> None:
        ctl, fake = self.connect()
        ctl.servo(True)  # PC now
        fake._homed = True
        ctl.move(140, 20, 0.1)
        self.assertTrue(wait_until(lambda: ctl.snapshot()["status"]["moving"] is True, 2))
        with mock.patch.object(fake, "release_control") as release:
            with self.assertRaises(BackendError) as caught:
                ctl.set_control(False)
        self.assertEqual(caught.exception.http_status, 409)
        self.assertIn("moving", str(caught.exception))
        self.assertIn("only switched at rest", str(caught.exception))
        release.assert_not_called()
        self.assertTrue(fake.status()["modbus_control"])
        self.assertFalse(ctl.snapshot()["pio_hold"])
        ctl.stop()
        self.assertTrue(wait_until(lambda: ctl.snapshot()["status"]["moving"] is False, 2))
        self.assertFalse(ctl.set_control(False)["modbus_control"])  # at rest: switched

    def test_refused_while_a_plc_moves_it_and_pc_is_not_taken(self) -> None:
        ctl, fake = self.connect()
        fake.pio_move(140.0, 20.0)
        with self.assertRaises(BackendError) as caught:
            ctl.set_control(True)
        self.assertEqual(caught.exception.http_status, 409)
        self.assertEqual(fake.control_writes, 0, "PMSL switched during motion")
        self.assertTrue(fake.status()["moving"], "the PIO move was disturbed")

    def test_refused_when_the_motion_state_cannot_be_read(self) -> None:
        ctl, fake = self.connect()
        real = fake.status
        with mock.patch.object(fake, "status", side_effect=lambda: {**real(), "moving": None}):
            with self.assertRaises(BackendError) as caught:
                ctl.set_control(True)
        self.assertIn("unknown motion state", str(caught.exception))
        self.assertEqual(fake.control_writes, 0)

    def test_a_unit_that_does_not_follow_is_reported(self) -> None:
        ctl, fake = self.connect()
        ctl.set_control(True)
        with mock.patch.object(fake, "release_control"):  # PMSL write has no effect: PMSS stays 1
            with self.assertRaises(BackendError) as caught:
                ctl.set_control(False)
        self.assertEqual(caught.exception.http_status, 502)
        self.assertIn("still reports PC control", str(caught.exception))

    def test_disconnect_leaves_it_as_chosen_and_reconnect_starts_clean(self) -> None:
        ctl, fake = self.connect()
        ctl.set_control(True)
        ctl.set_control(False)
        with mock.patch.object(fake, "release_control", wraps=fake.release_control) as release:
            ctl.disconnect()
        release.assert_not_called()  # PIO was chosen: nothing is left to hand back
        self.assertFalse(fake._control)
        ctl.connect("SIMULATED", 115200, 1, True)
        self.assertFalse(ctl.snapshot()["pio_hold"])
        self.assertFalse(ctl.snapshot()["status"]["modbus_control"])  # a new session: PIO as after power-up

    def test_pc_chosen_here_is_still_handed_back_to_pio_at_disconnect(self) -> None:
        ctl, fake = self.connect()
        ctl.set_control(True)
        self.assertTrue(fake._control)
        ctl.disconnect()
        self.assertFalse(fake._control)

    def test_stop_and_take_control_ends_the_pio_choice(self) -> None:
        ctl, fake = self.connect(stop_confirm_s=0.4)
        ctl.set_control(True)
        ctl.set_control(False)
        ctl.stop(take_control=True)
        self.assertTrue(ctl.snapshot()["status"]["modbus_control"])
        self.assertFalse(ctl.snapshot()["pio_hold"])


class ControlModeApiTest(_ServerCase):
    """POST /api/control (the test fake): validation and the answer."""

    def test_needs_a_connection_and_a_valid_mode(self) -> None:
        self.assertEqual(self.call("/api/control", {"mode": "pc"})[0], 409)  # not connected
        self.assertEqual(self.call("/api/control")[0], 404)  # GET is never a command
        self.connect_sim()
        for bad in (None, "", "PC", "plc", "on", True, 1, ["pc"]):
            code, body = self.call("/api/control", {} if bad is None else {"mode": bad})
            self.assertEqual(code, 400, (bad, body))
            self.assertIn("'pc' or 'pio'", body["error"])
        self.assertFalse(self.status()["status"]["modbus_control"])

    def test_switch_and_read_back(self) -> None:
        self.connect_sim()
        body = self.ok("/api/control", {"mode": "pc"})
        self.assertEqual((body["modbus_control"], body["changed"], body["pio_hold"]), (True, True, False))
        self.assertTrue(self.status()["status"]["modbus_control"])
        body = self.ok("/api/control", {"mode": "pio"})
        self.assertEqual((body["modbus_control"], body["pio_hold"]), (False, True))
        snap = self.status()
        self.assertFalse(snap["status"]["modbus_control"])
        self.assertTrue(snap["pio_hold"])
        code, body = self.call("/api/servo", {"on": True})
        self.assertEqual(code, 409)
        self.assertIn("PIO control was chosen", body["error"])


class ControlModeDriverTest(_ServerCase):
    """The same switch through the real driver and its simulator."""

    session_factory = staticmethod(open_session)

    def sim(self) -> SimulatedEC:
        return self.ctl._session._bus.transport  # white-box: the SimulatedEC behind ModbusRTU

    def test_pcon_paused_at_rest_in_pio_mode_takes_pc_control_back(self) -> None:
        # a PCON with nothing on its PIO connector pauses at rest with the servo off in PIO mode (VERIFIED on the
        # PCON-CB, 2026-11-02): the switch back to PC is not refused for that pause
        self.connect_sim("pcon")
        self.ok("/api/control", {"mode": "pc"})
        self.ok("/api/control", {"mode": "pio"})
        self.assertFalse(self.sim().pmsl)
        self.assertTrue(wait_until(lambda: self.status()["status"]["paused"] is True, 3))
        self.ok("/api/control", {"mode": "pc"})
        self.assertTrue(self.sim().pmsl)
        self.assertTrue(self.status()["status"]["modbus_control"])

    def test_a_pause_with_the_servo_on_still_blocks_pc_control(self) -> None:
        # a PLC holding *STP: paused at a standstill with the servo ON is a hold, not an unwired *STP input
        self.connect_sim("pcon")
        self.ok("/api/control", {"mode": "pc"})
        self.ok("/api/control", {"mode": "pio"})
        self.assertTrue(wait_until(lambda: self.status()["status"]["paused"] is True, 3))
        self.sim().set_word_override(R.DSS1, R.DSS1_PWR | R.DSS1_SV | R.DSS1_HEND | R.DSS1_STP)
        self.assertTrue(wait_until(lambda: self.status()["status"]["servo_on"] is True, 3))
        code, body = self.call("/api/control", {"mode": "pc"})
        self.assertEqual(code, 409, body)
        self.assertIn("paused", body["error"])
        self.assertFalse(self.sim().pmsl, "PMSL was written for a held pause")

    def test_pc_pio_pc_on_the_simulator(self) -> None:
        self.connect_sim("slider")
        self.ok("/api/control", {"mode": "pc"})
        self.assertTrue(self.sim().pmsl)
        self.assertTrue(self.status()["status"]["modbus_control"])
        self.ok("/api/servo", {"on": True})
        self.ok("/api/control", {"mode": "pio"})
        self.assertFalse(self.sim().pmsl)
        snap = self.status()
        self.assertFalse(snap["status"]["modbus_control"])
        self.assertTrue(snap["pio_hold"])
        code, body = self.call("/api/move", {"position_mm": 30, "speed_mm_s": 20, "accel_g": 0.1})
        self.assertEqual(code, 409)
        self.assertIn("PIO control was chosen", body["error"])
        self.assertFalse(self.sim().moving)
        self.assertFalse(self.sim().pmsl, "the refused move took PC control")
        self.ok("/api/control", {"mode": "pc"})
        self.assertTrue(self.sim().pmsl)
        self.ok("/api/move", {"position_mm": 30, "speed_mm_s": 20, "accel_g": 0.1})

    def test_refused_while_moving_and_works_after_stop(self) -> None:
        self.connect_sim("slider")
        self.ok("/api/servo", {"on": True})
        self.ok("/api/home", {})
        self.assertTrue(wait_until(lambda: self.status()["status"]["homed"] is True, 30))
        self.wait_idle()
        self.ok("/api/move", {"position_mm": 120, "speed_mm_s": 20, "accel_g": 0.1})
        self.assertTrue(wait_until(lambda: self.status()["status"]["moving"] is True, 3))
        for mode in ("pio", "pc"):
            code, body = self.call("/api/control", {"mode": mode})
            self.assertEqual(code, 409, body)
            self.assertIn("only switched at rest", body["error"])
        self.assertTrue(self.sim().pmsl, "PMSL switched during the move")
        self.assertTrue(self.status()["status"]["moving"])
        self.ok("/api/stop", {})
        self.wait_idle()
        self.ok("/api/control", {"mode": "pio"})
        self.assertFalse(self.sim().pmsl)


class ControlModeWorkbenchTest(_WorkbenchCase):
    """The control switch on the workbench axis: simulators only, through the same /api/wb/call as the buttons."""

    def unit(self, name: str) -> Any:
        port, slave = name.split(":")
        return self.wb.ports[port].transport[int(slave)]  # the simulated unit behind the axis

    def test_pc_pio_pc_and_the_state_the_page_shows(self) -> None:
        [name] = self.sim("slider")[:1]
        self.assertFalse(self.axis(name)["pio_hold"])
        out = self.run_fn(name, "enable_modbus_control")
        self.assertEqual(out["modbus_control"], True)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["modbus_enabled"] is True, 3))
        out = self.run_fn(name, "disable_modbus_control")
        self.assertEqual(out["modbus_control"], False)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["modbus_enabled"] is False, 3))
        self.assertTrue(self.axis(name)["pio_hold"])
        self.assertFalse(self.wb.axes[name].ec.auto_modbus_control)
        self.assertFalse(self.run_fn(name, "disable_modbus_control")["changed"])  # already PIO
        out = self.run_fn(name, "enable_modbus_control")
        self.assertTrue(out["changed"])
        self.assertFalse(self.axis(name)["pio_hold"])
        self.assertTrue(self.wb.axes[name].ec.auto_modbus_control)

    def test_commands_group_and_run_are_refused_while_pio_is_chosen(self) -> None:
        names = self.sim("xy")
        self.assertGreaterEqual(len(names), 2)
        a, b = names[:2]
        self.ready(a, b)
        self.run_fn(a, "enable_modbus_control")
        self.run_fn(a, "disable_modbus_control")
        for fn, args in (("servo", {"on": False}), ("home", {}), ("move_to", {"mm": 20, "speed_mm_s": 10, "accel_g": 0.1}),
                         ("move_by", {"delta_mm": 1, "speed_mm_s": 10, "accel_g": 0.1}), ("reset_alarm", {}),
                         ("set_safety_speed", {"on": True})):
            code, body = self.call("/api/wb/call", {"axis": a, "fn": fn, "args": args})
            self.assertEqual(code, 409, (fn, body))
            self.assertIn("PIO mode", body["error"])
            self.assertIn("Switch it to PC first", body["error"])
        self.assertFalse(self.unit(a).pmsl, "a refused command took PC control")
        code, body = self.call("/api/wb/group", {"op": "move_all", "axes": [a, b], "targets": {a: 10, b: 10},
                                                 "speed": {a: 10, b: 10}})
        self.assertEqual(code, 409, body)
        self.assertIn("PIO mode", body["error"])
        self.assertEqual(self.call("/api/wb/group", {"op": "status_all"})[0], 200)  # reads are not refused
        self.assertEqual(self.call("/api/wb/call", {"axis": a, "fn": "status", "args": {}})[0], 200)
        self.assertEqual(self.call("/api/wb/call", {"axis": a, "fn": "stop", "args": {}})[0], 200)  # STOP: never
        self.run_fn(a, "enable_modbus_control")
        self.run_fn(a, "move_to", mm=20, speed_mm_s=10, accel_g=0.1)
        self.wait_still(a)

    def test_a_coordinated_run_is_refused_while_an_axis_is_in_pio_mode(self) -> None:
        x, y = self.sim("xy")
        self.ready(x, y)
        for name in (x, y):  # away from the end stops: a circle needs room on every side
            self.run_fn(name, "move_to", mm=60, speed_mm_s=100, accel_g=0.3)
        self.wait_at(x, 60.0)
        self.wait_at(y, 60.0)
        px, py = self.axis(x)["status"]["position_mm"], self.axis(y)["status"]["position_mm"]
        plan = self.ok("/api/wb/coord/plan", {"axes": {"x": x, "y": y}, "speed": 15.0, "accel_g": 0.3,
                                              "chord_error": 0.05,
                                              "shape": {"kind": "circle", "center": [px - 5, py], "radius": 5.0}})
        self.assertTrue(plan["passed"], plan)
        self.run_fn(x, "disable_modbus_control")
        code, body = self.call("/api/wb/coord/run", {"plan_id": plan["plan_id"]})
        self.assertEqual(code, 409, body)
        self.assertIn("PIO mode", body["error"])
        self.assertIn("the coordinated run", body["error"])
        self.assertFalse(self.unit(x).pmsl)
        self.assertEqual(self.ok("/api/wb/coord/status")["state"], "planned")  # nothing was started

    def test_refused_while_the_axis_moves(self) -> None:
        # a stepped simulator clock: the move (140 mm at 10 mm/s) still runs when the switch is asked for, however long
        # a busy PC takes to ask (at time_scale 20 it was over within 0.7 s of real time)
        with mock.patch.object(self.wb, "sim_kwargs", {"step_s": 0.05}):
            [name] = self.sim("slider")[:1]
        self.ready(name)
        self.run_fn(name, "move_to", mm=140, speed_mm_s=10, accel_g=0.1)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["moving"] is True, 3))
        for fn in ("disable_modbus_control", "enable_modbus_control"):
            code, body = self.call("/api/wb/call", {"axis": name, "fn": fn, "args": {}})
            self.assertEqual(code, 409, (fn, body))
            self.assertIn("only switched at rest", body["error"])
            self.assertIn("moving", body["error"])
        self.assertTrue(self.unit(name).pmsl, "PMSL switched during the move")
        self.assertFalse(self.axis(name)["pio_hold"])
        self.call("/api/wb/stop", {"axis": name})
        self.wait_still(name)
        self.run_fn(name, "disable_modbus_control")
        self.assertTrue(self.axis(name)["pio_hold"])

    def test_pcon_paused_at_rest_in_pio_mode_takes_pc_control_back(self) -> None:
        # a PCON with nothing on its PIO connector pauses at rest with the servo off in PIO mode (VERIFIED on the
        # PCON-CB, 2026-11-02): the switch back to PC is not refused for that pause
        [name] = self.sim("pcon")[:1]
        self.run_fn(name, "enable_modbus_control")
        self.run_fn(name, "disable_modbus_control")
        self.assertFalse(self.unit(name).pmsl)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["paused"] is True, 3))
        out = self.run_fn(name, "enable_modbus_control")
        self.assertEqual(out["modbus_control"], True)
        self.assertTrue(self.unit(name).pmsl)
        self.assertFalse(self.axis(name)["pio_hold"])

    def test_a_pause_with_the_servo_on_still_blocks_pc_control(self) -> None:
        # a PLC holding *STP: paused at a standstill with the servo ON is a hold, not an unwired *STP input
        [name] = self.sim("pcon")[:1]
        self.run_fn(name, "enable_modbus_control")
        self.run_fn(name, "disable_modbus_control")
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["paused"] is True, 3))
        self.unit(name).set_word_override(R.DSS1, R.DSS1_PWR | R.DSS1_SV | R.DSS1_HEND | R.DSS1_STP)
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["servo_on"] is True, 3))
        code, body = self.call("/api/wb/call", {"axis": name, "fn": "enable_modbus_control", "args": {}})
        self.assertEqual(code, 409, body)
        self.assertIn("paused", body["error"])
        self.assertFalse(self.unit(name).pmsl, "PMSL was written for a held pause")

    def test_the_pages_have_the_switch_and_ask_before_it_switches(self) -> None:
        html = fetch(self.base, "/")[2]
        for hook in ("modeToggle", "modePc", "modePio", "axControl"):
            self.assertIn(f'id="{hook}"', html, hook)
        js = fetch(self.base, "/workbench.js")[2]
        self.assertIn('fn: mode === "pc" ? "enable_modbus_control" : "disable_modbus_control"', js)
        self.assertIn("own stored", js)  # the warning: stored rows run at their own speeds
        start = js.index("async function chooseMode")
        self.assertLess(js.index("await ask({", start), js.index("/api/wb/call", start))
        classic = fetch(self.base, "/classic")[2]
        for hook in ("modePcBtn", "modePioBtn", "controlInfo"):
            self.assertIn(f'id="{hook}"', classic, hook)
        app = fetch(self.base, "/app.js")[2]
        self.assertIn("their own stored speeds", app)
        start = app.index("async function chooseControl")
        self.assertLess(app.index("confirm(MODE_CONFIRM[mode])", start), app.index('api("/api/control"', start))


class ReviewFixesTest(_WorkbenchCase):
    """Review of the merged GUI: a STOP cancels what was not sent yet and reaches every axis, a group move names the axes
    it moves, and a scan range is checked before it is built."""

    def hold_lock(self, name: str) -> threading.Event:
        """Hold one axis's lock from another thread, as a long read or a back-up does; the returned event releases it."""
        slot = self.wb.axes[name]
        held, release = threading.Event(), threading.Event()

        def hold() -> None:
            with slot.lock:
                held.set()
                release.wait(15)

        holder = threading.Thread(target=hold, daemon=True)
        holder.start()
        self.assertTrue(held.wait(3))
        self.addCleanup(lambda: (release.set(), holder.join(3)))
        return release

    def post_when_waiting(self, path: str, data: dict[str, Any], hook: str = "_refuse_pio_hold") -> tuple[threading.Thread, dict[str, Any]]:
        """POST in a thread and return once the request has passed ``hook`` (so its STOP counter is read and it is
        about to wait for the axis lock)."""
        reached, out = threading.Event(), {}
        real = getattr(self.wb, hook)

        def signal(*args: Any, **kwargs: Any) -> Any:
            reached.set()
            return real(*args, **kwargs)

        def go() -> None:
            out["r"] = self.call(path, data)

        with mock.patch.object(self.wb, hook, side_effect=signal):
            thread = threading.Thread(target=go, daemon=True)
            thread.start()
            self.assertTrue(reached.wait(3))
        time.sleep(0.1)  # on to the lock wait
        return thread, out

    def at_rest_near(self, name: str, position: float, delta: float = 1.0) -> None:
        self.assertTrue(wait_until(lambda: self.axis(name)["status"]["moving"] is False, 6))
        self.assertAlmostEqual(self.axis(name)["status"]["position_mm"], position, delta=delta)

    MOVE = {"mm": 100, "speed_mm_s": 100, "accel_g": 0.3}

    def test_a_move_waiting_for_the_axis_lock_does_not_run_after_stop_all(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        release = self.hold_lock(name)
        thread, out = self.post_when_waiting("/api/wb/call", {"axis": name, "fn": "move_to", "args": self.MOVE})
        self.ok("/api/wb/stop", {})
        release.set()
        thread.join(5)
        code, body = out["r"]
        self.assertEqual(code, 409, body)
        self.assertIn("cancelled", body["error"])
        self.assertIn("STOP", body["error"])
        time.sleep(0.5)
        self.at_rest_near(name, 0.0)  # it was not sent
        self.run_fn(name, "move_to", **{**self.MOVE, "mm": 20})  # the next command after the STOP is not affected
        self.wait_at(name, 20.0)

    def test_a_stop_of_one_axis_cancels_only_that_axes_waiting_commands(self) -> None:
        x, y = self.sim("xy")
        self.ready(x, y)
        releases = [self.hold_lock(x), self.hold_lock(y)]
        threads = [self.post_when_waiting("/api/wb/call", {"axis": n, "fn": "move_to", "args": {**self.MOVE, "mm": 40}})
                   for n in (x, y)]
        self.ok("/api/wb/stop", {"axis": x})
        for release in releases:
            release.set()
        for thread, _out in threads:
            thread.join(5)
        self.assertEqual(threads[0][1]["r"][0], 409, threads[0][1]["r"])  # the stopped axis: not run
        self.assertEqual(threads[1][1]["r"][0], 200, threads[1][1]["r"])  # the other axis: run as asked
        self.wait_at(y, 40.0)
        self.at_rest_near(x, 0.0)

    def test_a_group_move_waiting_for_the_locks_does_not_run_after_stop_all(self) -> None:
        x, y = self.sim("xy")
        self.ready(x, y)
        release = self.hold_lock(y)
        thread, out = self.post_when_waiting(
            "/api/wb/group", {"op": "move_all", "axes": [x, y], "targets": {x: 40, y: 40}, "speed": {x: 100, y: 100}})
        self.ok("/api/wb/group", {"op": "stop_all"})
        release.set()
        thread.join(5)
        code, body = out["r"]
        self.assertEqual(code, 409, body)
        self.assertIn("cancelled", body["error"])
        time.sleep(0.5)
        self.at_rest_near(x, 0.0)
        self.at_rest_near(y, 0.0)

    def test_a_stop_that_comes_while_a_command_is_sent_is_sent_again(self) -> None:
        [name] = self.sim("slider")
        self.ready(name)
        real = self.wb._fn_move_to

        def stop_first(slot: Any, args: dict[str, Any]) -> Any:
            self.wb.stop()  # the STOP reaches the unit before the move does
            return real(slot, args)

        with mock.patch.object(self.wb, "_fn_move_to", side_effect=stop_first):
            self.run_fn(name, "move_to", mm=100, speed_mm_s=20, accel_g=0.3)
        self.at_rest_near(name, 0.0, delta=50.0)
        self.assertLess(self.axis(name)["status"]["position_mm"], 80.0)  # it did not run to its target
        self.assertTrue(any("STOP re-sent" in e["message"] for e in self.ok("/api/wb/state")["events"]))

    def test_stop_all_reaches_an_axis_that_is_connected_before_its_port_is_done(self) -> None:
        self.wb.start()
        real, calls = self.wb._add_axis, []
        second, gate = threading.Event(), threading.Event()

        def slow(port_slot: Any, slave: int) -> str:
            calls.append(slave)
            if len(calls) == 2:  # the first axis is on the page, the rest of the port is still being identified
                second.set()
                gate.wait(15)
            return real(port_slot, slave)

        with mock.patch.object(self.wb, "_add_axis", side_effect=slow):
            connecting = threading.Thread(target=lambda: self.wb.connect(simulate=True, sim_setup="line"), daemon=True)
            connecting.start()
            try:
                self.assertTrue(second.wait(10))
                [name] = list(self.wb.axes)
                self.assertTrue(self.wb.cell is None or name not in self.wb.cell.names)  # not in the group yet
                self.ready(name)
                self.run_fn(name, "move_to", mm=140, speed_mm_s=3, accel_g=0.1)
                self.assertTrue(wait_until(lambda: self.wb.axes[name].ec.status().moving is True, 3))
                out = self.ok("/api/wb/stop", {})
                self.assertIn(name, out["stopped"], out)
                self.assertEqual(out["errors"], {})
                self.assertTrue(wait_until(lambda: self.wb.axes[name].ec.status().moving is False, 3), "still moving")
                self.assertLess(self.wb.axes[name].ec.status().position_mm, 100.0)
            finally:
                gate.set()
                connecting.join(15)

    def test_a_group_move_must_name_the_axes_it_moves(self) -> None:
        x, y = self.sim("xy")
        self.ready(x, y)
        for data in ({"axes": [x], "targets": {y: 40}, "speed": {y: 100}},  # y is not in axes: no check was made for it
                     {"axes": [x, y], "targets": {y: 40}, "speed": {y: 100}},
                     {"axes": [x], "targets": {x: 40, y: 40}, "speed": {x: 100, y: 100}}):
            code, body = self.call("/api/wb/group", {"op": "move_all", **data})
            self.assertEqual(code, 400, (data, body))
            self.assertIn("same axes", body["error"])
        time.sleep(0.3)
        self.at_rest_near(y, 0.0)
        out = self.ok("/api/wb/group", {"op": "move_all", "targets": {y: 40}, "speed": {y: 100}})  # axes left out: the targets
        self.assertEqual(out["started"], [y])
        self.wait_at(y, 40.0)
        self.at_rest_near(x, 0.0)
        out = self.ok("/api/wb/group", {"op": "move_all", "axes": [x, y], "targets": {x: 30, y: 30},
                                        "speed": {x: 100, y: 100}})  # what the page sends
        self.assertEqual(sorted(out["started"]), sorted([x, y]))
        self.wait_at(x, 30.0)

    def test_a_scan_range_is_checked_before_it_is_built(self) -> None:
        import tracemalloc

        tracemalloc.start()
        try:
            for bad in ("1-3000000", "5-1", "0-4", "240-248", "1-3,1-3000000"):
                with self.assertRaises(BackendError, msg=bad):
                    parse_slaves(bad)
            peak = tracemalloc.get_traced_memory()[1]
        finally:
            tracemalloc.stop()
        self.assertLess(peak, 500_000, "a huge range was built before it was refused")
        code, body = self.call("/api/wb/connect", {"port": "COM3", "slaves": "1-3000000"})
        self.assertEqual(code, 400, body)
        self.assertEqual(parse_slaves("1-3,5, 7"), [1, 2, 3, 5, 7])
        self.assertEqual(parse_slaves("247"), [247])


if __name__ == "__main__":
    unittest.main()
