"""Multi-axis workbench behind the web GUI: ports, axes, live status and every driver function.

The normal setup is one axis per COM port (any number of ports, all used in parallel); up to 16 axes can also share
one port by axis number (RCON gateway / SIO daisy chain, slave = axis number + 1, UNVERIFIED on hardware), and
the two can be mixed. Every port is an :class:`iai_actuator_driver.cell.Port` (opened once, baud rate detected in place),
every axis an ordinary :class:`iai_actuator_driver.ec.Elecylinder` on it, and all of them together a
:class:`iai_actuator_driver.cell.Cell` for the group commands.

Every unit is driven the same way: the ranges shown and enforced come from the unit itself
(``Elecylinder.limits()``). The GUI adds no checks of its own
beyond input types; every refusal is the driver's (its error text is shown as it is).

Concurrency: one poll thread reads every axis's status. Commands run in the HTTP request thread under the
axis's lock and never wait for motion (``wait=False``), so the poll shows progress and the lock is short.
STOP / pause / resume never take the axis lock (they only wait for the bus, one frame at a time).

Fast mode (per axis, ``set_fast``): ``completion="fast"`` and the axis's status monitor
(:class:`iai_actuator_driver.monitor.AxisMonitor`); while it runs the live status comes from the monitor's samples (the lean
status: no ZONS / POSS / SSSE flags) and the poll sends nothing for that axis. The baud rate is not changed.

Power / link loss (:mod:`iai_actuator_driver.recovery`): every axis is tracked when it is added (read-only); each status read
is a link check (:func:`~iai_actuator_driver.recovery.check_link`), and the monitor checks the link itself. A lost link or a
restarted unit sets the axis's ``fault`` (the page shows a banner); ``recover`` runs :meth:`Elecylinder.recover`
(never moves the axis) and keeps its report for the page.

Simulator setups (:data:`SIM_SETUPS`) use :class:`iai_actuator_driver.sim.SimulatedEC` units on a
:class:`iai_actuator_driver.sim_line.SimLine`: no serial port is opened.

Coordinated motion (experimental, :mod:`gui.coordinated`): a run holds both axes' locks; meanwhile every request that
takes one of their locks, and every group operation on them, is refused at once (STOP, pause and resume still go
through and end the run). Connecting a port or scanning one is refused during a run; a disconnect stops the run first.

Disconnect leaves each unit safe and as found: a fresh status read, STOP (waiting for the standstill) when the axis
moves, homes or cannot be read, the HOME coil released, control handed back to PIO when PIO had it at connect, and the
servo left as it is.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import logging
import math
import os
import re
import sys
import threading
import time
from collections import Counter, deque
from collections.abc import Callable, Iterable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]  # the repository root of a source checkout
_DRIVER = ROOT / "bindings" / "python"  # its Python package, used before an installed one
if (_DRIVER / "iai_actuator_driver").is_dir() and str(_DRIVER) not in sys.path:
    sys.path.insert(0, str(_DRIVER))

from iai_actuator_driver import errors as E  # noqa: E402 (needs the sys.path entry above)
from iai_actuator_driver import recovery as RC  # noqa: E402
from iai_actuator_driver import registers as R  # noqa: E402
from iai_actuator_driver.alarms import alarm_info, alarm_name  # noqa: E402
from iai_actuator_driver.cell import DEFAULT_SLAVES, Cell, Port, axis_name, list_ports  # noqa: E402
from iai_actuator_driver.check import run_check  # noqa: E402
from iai_actuator_driver.ec import NV_SKIP_NOTE, NV_WARN_PER_HOUR, NV_WRITE_CYCLES, Elecylinder  # noqa: E402
from iai_actuator_driver.gateway import RconGateway  # noqa: E402
from iai_actuator_driver.info import SECTIONS, backup  # noqa: E402
from iai_actuator_driver.modbus import IAI_BAUD_CODES, UNIT_DEPENDENT_BAUD_RATES, VERIFIED_BAUD_RATES  # noqa: E402
from iai_actuator_driver.sim import SimulatedEC  # noqa: E402
from iai_actuator_driver.sim_line import SimLine  # noqa: E402

try:
    from .backend import BackendError, check_port_name
    from .coordinated import CoordinatedRunner
    from .resources import user_dir
except ImportError:  # run as a script from apps/gui/
    from backend import BackendError, check_port_name  # type: ignore[no-redef]
    from coordinated import CoordinatedRunner  # type: ignore[no-redef]
    from resources import user_dir  # type: ignore[no-redef]

__all__ = ["Workbench", "SIM_SETUPS", "FUNCTIONS", "catalog", "classify_open_error", "split_evidence"]

log = logging.getLogger(__name__)

HOME_START_S = 2.0  # a home return not seen to start within this releases the HOME coil
# The "Back up this unit" function writes here: the project's backups/ folder in a source checkout (git-ignored),
# next to iai-gui.exe in the bundle, else backups/ in the folder the GUI was started from (an installed package must
# not write into site-packages).
BACKUP_DIR = user_dir() / "backups"
SIM_BAUD = 115200
MAX_EVENTS = 200
MONITOR_AGE_S = 0.5  # a Fast-mode axis's live status: the monitor's sample when it is younger than this
RECOVER_TIMEOUT_S = 30.0
DISCONNECT_LOCK_S = 1.0  # a disconnect waits this long for an axis lock (else no fresh status: it sends STOP)
DISCONNECT_STOP_S = 3.0  # STOP before a disconnect waits this long for the standstill
RUN_REFUSED = "A coordinated run is in progress: wait for it or press STOP ALL."
STOP_CANCELLED = "{what} was cancelled: STOP was pressed while it waited, so nothing was sent."
# Functions that are Modbus commands: with PIO chosen on the axis they are refused (the unit would ignore them).
NEEDS_PC_CONTROL = frozenset({"servo", "home", "move_to", "move_by", "jog", "push", "move_to_position", "change_speed",
                              "change_accel", "reset_alarm", "set_safety_speed"})
_FAULT_KINDS = (RC.PORT_VANISHED, RC.UNIT_SILENT, RC.UNIT_RESTARTED, RC.UNIT_REPLACED)
_FAULT_TEXT = {
    RC.PORT_VANISHED: "the port vanished (USB adapter unplugged?)",
    RC.UNIT_SILENT: "the unit does not answer (24 V off, cable, or another baud rate after a power-up)",
    RC.UNIT_RESTARTED: "the unit restarted (power loss): PC control, home and the move in progress may be lost",
    RC.UNIT_REPLACED: "another unit answers at this address: nothing is written to it",
}

# name -> (label, [(port suffix, [(slave, profile), ...]), ...]); one entry per port.
SIM_SETUPS: dict[str, tuple[str, list[list[tuple[int, str]]]]] = {
    "slider": ("ELECYLINDER slider (EC-S)", [[(1, "slider")]]),
    "rod": ("ELECYLINDER rod (EC-R)", [[(1, "rod")]]),
    "gripper": ("ELECYLINDER gripper (EC-GR)", [[(1, "gripper")]]),
    "rotary": ("Rotary axis (deg)", [[(1, "rotary")]]),
    "pcon": ("PCON + RCP6 RoboCylinder", [[(1, "pcon")]]),
    "unknown": ("Generic IAI unit (a unit code without a family name)", [[(1, "unknown")]]),
    "line": ("Multi-axis line: 3 axes on ONE port (slider, rod, gripper)", [[(1, "slider"), (2, "rod"), (3, "gripper")]]),
    "cell": ("Cell: 3 ports, one axis each (slider, PCON, gripper)", [[(1, "slider")], [(1, "pcon")], [(1, "gripper")]]),
    "xy": ("XY pair: two sliders, one per port", [[(1, "slider")], [(1, "slider")]]),
}

STATUS_KEYS = (
    "position_mm", "velocity_mm_s", "servo_on", "homed", "in_position", "moving", "alarm", "alarm_code", "warning",
    "alarm_level", "emergency_stop", "modbus_enabled", "pushing", "push_contact", "push_missed", "paused", "homing",
    "at_backward_end", "at_forward_end", "at_middle", "safety_speed", "idle", "ready", "current_ma",
    # condition flags (None: not read / not defined on this line), DOC ME0162-12A p.4-18 to 4-25
    "emg_input", "drive_power_cut", "enable_off", "cold_start_alarm", "auto_servo_off", "nv_busy", "manual_mode",
    "battery_low", "rtc_in_use", "excitation_done", "position_complete_no", "line",
)


# ====================================================================== JSON helpers


def jsonable(value: Any) -> Any:
    """Driver results as plain JSON: ``to_dict()`` objects, dataclasses, tuples, bytes, non-finite floats."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if hasattr(value, "to_dict") and callable(value.to_dict):
        return jsonable(value.to_dict())
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return jsonable(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set, frozenset, deque)):
        return [jsonable(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return value.hex(" ")
    return str(value)


def describe(exc: BaseException) -> str:
    """The driver's own error text (with its notes), prefixed by the error type."""
    text = str(exc) or type(exc).__name__
    notes = getattr(exc, "__notes__", None) or []
    if notes:
        text += " (" + "; ".join(str(n) for n in notes) + ")"
    return f"{type(exc).__name__}: {text}"


def to_backend_error(exc: BaseException) -> BackendError:
    if isinstance(exc, BackendError):
        return exc
    if isinstance(exc, (ValueError, TypeError, E.SoftLimitError, E.UnsafeCommandError, E.NonVolatileWriteError)):
        return BackendError(describe(exc), 400)
    if isinstance(exc, (E.ModbusError, OSError)):
        return BackendError(describe(exc), 502)
    return BackendError(describe(exc), 409)


# ====================================================================== argument parsing


def _num(args: dict[str, Any], key: str, default: Any = ..., *, optional: bool = False) -> float | None:
    value = args.get(key)
    if value is None or value == "":
        if default is not ...:
            return default
        if optional:
            return None
        raise BackendError(f"Missing input: {key}.", 400)
    if isinstance(value, bool):
        raise BackendError(f"{key} must be a number.", 400)
    if isinstance(value, str):
        try:
            value = float(value)
        except ValueError:
            raise BackendError(f"{key} must be a number, got {value!r}.", 400) from None
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BackendError(f"{key} must be a finite number.", 400)
    return float(value)


def _int(args: dict[str, Any], key: str, default: Any = ..., *, optional: bool = False) -> int | None:
    value = args.get(key)
    if isinstance(value, str) and value.strip().lower().startswith("0x"):
        try:
            return int(value.strip(), 16)
        except ValueError:
            raise BackendError(f"{key} must be a whole number (decimal or 0x hex).", 400) from None
    number = _num(args, key, default, optional=optional)
    if number is None:
        return None
    if number != int(number):
        raise BackendError(f"{key} must be a whole number.", 400)
    return int(number)


def _bool(args: dict[str, Any], key: str, default: bool = False) -> bool:
    value = args.get(key, default)
    if value is None or value == "":
        return default
    if isinstance(value, str) and value.lower() in ("true", "false", "on", "off", "1", "0"):
        return value.lower() in ("true", "on", "1")
    if not isinstance(value, bool):
        raise BackendError(f"{key} must be true or false.", 400)
    return value


def _text(args: dict[str, Any], key: str, default: str | None = None) -> str | None:
    value = args.get(key, default)
    if value is None or value == "":
        return default
    if not isinstance(value, (str, int, float)) or isinstance(value, bool):
        raise BackendError(f"{key} must be text.", 400)
    return str(value).strip()


# ====================================================================== function catalogue


def _inp(name: str, label: str, kind: str = "number", **extra: Any) -> dict[str, Any]:
    return {"name": name, "label": label, "type": kind, **extra}


POS = _inp("mm", "Target position", range="position", example="frac:0.5")
SPEED = _inp("speed_mm_s", "Speed", range="speed", example="cap:50")
ACCEL = _inp("accel_g", "Acceleration", range="accel_decel", example="cap:0.3")
DECEL = _inp("decel_g", "Deceleration (optional)", range="decel", example="cap:0.3", optional=True,
             help="a value different from the acceleration uses the command-profile move")
BAND = _inp("band_mm", "Positioning band", range="band", example="cap:0.1", optional=True)
REL = _inp("delta_mm", "Distance (signed)", range="relative", example="cap:10")


def _section_choices() -> list[list[str]]:
    return [["", "all sections"]] + [[s, s] for s in SECTIONS]


FUNCTIONS: list[dict[str, Any]] = [
    # ---------------------------------------------------------------- motion
    {"name": "servo", "group": "Motion", "label": "Servo ON / OFF", "call": "ec.servo(on, wait=True)",
     "desc": "Switch the motor servo on or off and wait until the unit confirms. A servo ON that cannot work is "
             "refused with the reasons (EMG input, drive power cut, enable switch released).",
     "inputs": [_inp("on", "Servo on", "bool", example=True)]},
    {"name": "home", "group": "Motion", "label": "Home", "call": "ec.home(wait=False, method=...)",
     "desc": "Home return (~30 units/s to the mechanical end, then back to 0). The HOME coil is released when done.",
     "inputs": [_inp("method", "Method", "choice", optional=True,
                     choices=[["", "default (configured strategy)"], ["coil", "coil (HOME coil edge)"],
                              ["iai", "iai (0x45 cmd 0x0A with ORG)"]])]},
    {"name": "move_to", "group": "Motion", "label": "Move absolute", "call": "ec.move_to(mm, speed, accel, band, decel_g=...)",
     "desc": "Absolute move. A deceleration different from the acceleration uses the command-profile path "
             "(VERIFIED on the EC gripper and the PCON).",
     "inputs": [POS, SPEED, ACCEL, DECEL, BAND]},
    {"name": "move_by", "group": "Motion", "label": "Move relative", "call": "ec.move_by(delta, speed, accel, band)",
     "desc": "Move by a signed distance from the measured position (sent as an absolute, limit-checked target).",
     "inputs": [REL, SPEED, ACCEL, BAND]},
    {"name": "jog", "group": "Motion", "label": "Jog step", "call": "ec.jog(delta, speed, accel)",
     "desc": "One jog step: a short relative move at a low speed (the unit's JOG inputs are not used).",
     "inputs": [_inp("delta_mm", "Step (signed)", range="relative", example="cap:1"),
                _inp("speed_mm_s", "Speed", range="speed", example="cap:10"), ACCEL]},
    {"name": "push", "group": "Motion", "label": "Push", "call": "ec.push(target, band, force, direction, speed, accel)",
     "desc": "Approach the target, then push over the band at the given current limit; reports contact or miss.",
     "inputs": [_inp("target_mm", "Approach position", range="position", example="frac:0.4"),
                _inp("band_mm", "Push band", range="push_band", example="cap:5"),
                _inp("force_pct", "Push force", range="push", example="cap:30"),
                _inp("direction", "Direction", "choice", example=1,
                     choices=[[1, "+1 away from home"], [-1, "-1 toward home"]]),
                _inp("speed_mm_s", "Approach speed", range="speed", example="cap:20"), ACCEL]},
    {"name": "move_to_position", "group": "Motion", "label": "Run stored position",
     "call": "ec.move_to_position(n, speed_pct=..., allow_above_caps=...)",
     "desc": "Run a stored row at its own speed / accel / decel, optionally at a % of its speed (0x45 cmd 6: "
             "VERIFIED on the EC gripper and the PCON).",
     "inputs": [_inp("n", "Row", "int", range="stored_rows", example=1),
                _inp("speed_pct", "Speed override", range="speed_override", example=50, optional=True),
                _inp("allow_above_caps", "Allow a row above the driver caps (unit limits still apply)", "bool",
                     example=False, optional=True)]},
    {"name": "stop", "group": "Motion", "label": "STOP (decelerate)", "call": "ec.stop()", "nolock": True,
     "desc": "Decel stop with the STOP coil sequence (behind an RCON gateway: pause, wait, release); holds STOP "
             "until the axis stands still. Not a safety function.",
     "inputs": []},
    {"name": "pause", "group": "Motion", "label": "Pause", "call": "ec.pause()", "nolock": True,
     "desc": "Decelerate and hold, keeping the remaining travel (STP input).", "inputs": []},
    {"name": "resume", "group": "Motion", "label": "Resume", "call": "ec.resume()", "nolock": True,
     "desc": "Release the pause: a paused move continues by itself.", "inputs": []},
    {"name": "change_speed", "group": "Motion", "label": "Change speed of the running move",
     "call": "ec.change_speed(speed_mm_s)",
     "desc": "Re-sends the move in progress with a new speed (same target). A plain move only; refused for a push, "
             "stored-row or profile move, while paused, or with no move running. VERIFIED on the EC-GRBP8M gripper and the "
             "PCON-CB.",
     "inputs": [SPEED]},
    {"name": "change_accel", "group": "Motion", "label": "Change acceleration of the running move",
     "call": "ec.change_accel(accel_g)",
     "desc": "As above for the acceleration (= deceleration). VERIFIED on the EC-GRBP8M gripper.", "inputs": [ACCEL]},
    {"name": "cancel_paused_move", "group": "Motion", "label": "Cancel a paused move", "call": "ec.cancel_paused_move()",
     "desc": "Drop the remaining travel of a paused move and release the pause: the axis stays where it stopped. "
             "Refused when nothing is paused or a fault alarm is active. VERIFIED on the EC-GRBP8M gripper.", "inputs": []},
    {"name": "reset_alarm", "group": "Motion", "label": "Alarm reset", "call": "ec.reset_alarm(method=...)",
     "desc": "Clear a resettable alarm.",
     "inputs": [_inp("method", "Method", "choice", optional=True,
                     choices=[["", "default"], ["coil", "coil"], ["iai", "iai (0x45 cmd 5)"]])]},
    {"name": "check_move", "group": "Motion", "label": "Check a move (dry run)", "call": "ec.check_move(...)",
     "desc": "Every check move_to runs (ranges, readiness) with nothing sent.",
     "inputs": [POS, SPEED, ACCEL, DECEL, BAND]},
    {"name": "set_safety_speed", "group": "Motion", "label": "Safety speed", "call": "ec.set_safety_speed(on)",
     "desc": "Switch the unit's safety-speed mode on or off.",
     "inputs": [_inp("on", "Safety speed on", "bool", example=False)]},
    {"name": "enable_modbus_control", "group": "Motion", "label": "Take Modbus control", "call": "ec.enable_modbus_control()",
     "desc": "Take control from PIO (PMSL), at rest only; refused while the axis moves. Commands also take it "
             "automatically at rest, and this switches that back on after Give control back to PIO.", "inputs": []},
    {"name": "disable_modbus_control", "group": "Motion", "label": "Give control back to PIO",
     "call": "ec.disable_modbus_control()",
     "desc": "Release Modbus control (PMSL off), at rest only; refused while the axis moves. The unit then follows its "
             "PIO inputs and runs its stored rows at their own speeds. Motion commands of this page are refused (the "
             "automatic take-over is off) until Take Modbus control.", "inputs": []},
    {"name": "set_fast", "group": "Motion", "label": "Fast mode", "call": "ec.enable_fast(baudrate=None) / ec.disable_fast()",
     "desc": "Fast completion (a move is done on the first in-position sample, no settle window) and the axis's "
             "status monitor (live status, link watch). The baud rate is not changed (Port: Change baud rate).",
     "inputs": [_inp("on", "Fast mode on", "bool", example=True)]},
    # ---------------------------------------------------------------- link
    {"name": "recover", "group": "Link", "label": "Recover after a power / link loss", "call": "ec.recover(timeout=...)",
     "desc": "Waits for the port / unit, reopens the port, finds the baud rate, requires the same unit (serial number, "
             "model), restores baud rate, PC control and safety speed, resets a message-level alarm, and reports home, "
             "alarm and position. Never moves the axis (no servo ON, no home, no move). VERIFIED on the EC-GRBP8M gripper "
             "after a power cycle and a line break, and on the PCON-CB after a line break.",
     "inputs": [_inp("timeout_s", "Wait for the unit (s)", min=1, max=600, example=30, optional=True)]},
    {"name": "check_link", "group": "Link", "label": "Check the link", "call": "ec.check_link()",
     "desc": "One status read compared with the session: ok, port vanished, unit silent or unit restarted. Read-only.",
     "readonly": True, "inputs": []},
    # ---------------------------------------------------------------- read
    {"name": "status", "group": "Read", "label": "Status", "call": "ec.status()",
     "desc": "One status read: position, servo, homed, in position, moving, alarm and every flag.", "inputs": []},
    {"name": "identify", "group": "Read", "label": "Identify", "call": "ec.identify()",
     "desc": "Model, controller, actuator, firmware, serials, directory, soft limits, speed limit.", "inputs": []},
    {"name": "limits", "group": "Read", "label": "Limits (all ranges)", "call": "ec.limits()",
     "desc": "Every range the driver enforces on this axis, read from the unit itself.", "inputs": []},
    {"name": "read_monitor", "group": "Read", "label": "Monitor values", "call": "ec.read_monitor()",
     "desc": "Position, velocity, current, deviation, alarm, uptime (one block read).", "inputs": []},
    {"name": "read_position_table", "group": "Read", "label": "Read stored row", "call": "ec.read_position_table(n)",
     "desc": "One stored-position row: position, band, speed, zones, accel, decel, push power, flags.",
     "inputs": [_inp("n", "Row", "int", range="stored_rows", example=0)]},
    {"name": "read_alarm_history", "group": "Read", "label": "Alarm history", "call": "ec.read_alarm_history()",
     "desc": "The unit's alarm history (newest first) with names, levels and remedies.",
     "inputs": [_inp("include_empty", "Include empty slots", "bool", example=False, optional=True)]},
    {"name": "alarm_info", "group": "Read", "label": "Explain an alarm code", "call": "alarm_info(code)",
     "desc": "Name, level, how to reset and the remedy for any alarm code (no I/O).",
     "inputs": [_inp("code", "Alarm code (hex)", "int", min=0, max=0xFFFF, example="0x0E0")]},
    {"name": "parameters", "group": "Read", "label": "Parameters", "call": "ec.parameters()",
     "desc": "The unit's named parameters, decoded, with units, ranges and the dangerous flag.", "inputs": []},
    {"name": "read_parameter", "group": "Read", "label": "Read one parameter", "call": "ec.read_parameter(key)",
     "desc": "A named parameter (number, \"P5\", \"No.35\") or any raw item \"class.item\" (e.g. 8.15).",
     "inputs": [_inp("key", "Parameter or item", "text", example="8.15")]},
    {"name": "parameter_model", "group": "Read", "label": "Parameter list for this model", "call": "ec.parameter_model()",
     "desc": "Which parameter list applies to this unit.", "inputs": []},
    {"name": "read_soft_limits", "group": "Read", "label": "Soft limits", "call": "ec.read_soft_limits()",
     "desc": "The unit's soft limits (items 7.1 / 7.2) minus the 0.30 margin.", "inputs": []},
    {"name": "read_axis_limits", "group": "Read", "label": "Axis limits (class 7)", "call": "ec.read_axis_limits()",
     "desc": "MAXV, MAXA, MAXD, MXPP, MNPP, TCPC, AKID kind and the other per-model limits.", "inputs": []},
    {"name": "read_support_words", "group": "Read", "label": "Support flags", "call": "ec.read_support_words()",
     "desc": "The unit's support words (class 1): what it says it can do.", "inputs": []},
    {"name": "read_home_pattern", "group": "Read", "label": "Home pattern (ORG)", "call": "ec.read_home_pattern()",
     "desc": "The home pattern item 9.16 (used by the iai home method).", "inputs": []},
    {"name": "read_boot_baudrate", "group": "Read", "label": "Power-up baud rate", "call": "ec.read_boot_baudrate()",
     "desc": "The baud rate the unit starts at after a power cycle (item 8.3).", "inputs": []},
    {"name": "max_positions", "group": "Read", "label": "Stored row count", "call": "ec.max_positions()",
     "desc": "How many stored rows the unit uses (MAXP 3.2 / the directory).", "inputs": []},
    {"name": "line_info", "group": "Read", "label": "Line and gateway", "call": "ec.bus.line / gateway status",
     "desc": "The line this axis sits on (direct / rcon / robonet), what it carries, the position unit, the gateway "
             "status and, behind an RCON gateway, this axis's gateway monitor record. Read-only.",
     "inputs": [_inp("monitor", "Include the RCON gateway monitor record", "bool", example=False, optional=True)]},
    {"name": "backup", "group": "Read", "label": "Back up this unit (read-only)", "call": "info.backup(ec, path, rows=...)",
     "desc": "Save identity, directory, every parameter, the stored rows, the alarm history and the counters as JSON "
             "in the backups folder. Sends FC03 / FC11 only; nothing is written to the unit.",
     "inputs": [_inp("rows", "Rows: used, all (an EC has 768: slow) or numbers such as 0,1,2", "text", example="used",
                     optional=True)]},
    {"name": "read_registers", "group": "Read", "label": "Read raw registers", "call": "ec.read_registers(address, count)",
     "desc": "FC03 read of raw holding registers (read-only).",
     "inputs": [_inp("address", "Address (hex)", "int", min=0, max=0xFFFF, example="0x9000"),
                _inp("count", "Words", "int", min=1, max=64, example=8)]},
    {"name": "unit_info", "group": "Read", "label": "Unit info (everything)", "call": "ec.unit_info(sections=...)",
     "desc": "Everything the unit tells about itself, decoded (FC03 + FC11 only). Takes a few seconds.",
     "inputs": [_inp("section", "Section", "choice", optional=True, choices=_section_choices())]},
    {"name": "self_check", "group": "Read", "label": "Self-test (iai-check, read-only)",
     "call": "check.run_check(ec, motion=False)", "readonly": True,
     "desc": "The iai-check self-test: link, identity, limits, status, alarms, stored rows, parameters against their "
             "ranges. Read-only; takes a few seconds.",
     "inputs": []},
    # ---------------------------------------------------------------- non-volatile / configuration
    {"name": "write_position_table", "group": "Configuration", "label": "Write stored row (non-volatile)",
     "call": "ec.write_position_table(n, entry)", "confirm": "nonvolatile",
     "desc": "Change one stored row (starting from what the unit holds) and verify by read-back. A row that would "
             "not change is not written (one non-volatile write cycle saved).",
     "inputs": [_inp("n", "Row", "int", range="stored_rows", example=2),
                _inp("position_mm", "Position", range="position", example="frac:0.5"),
                _inp("speed_mm_s", "Speed", range="speed", example="cap:50", optional=True),
                _inp("accel_g", "Acceleration", range="accel", example="cap:0.3", optional=True),
                _inp("decel_g", "Deceleration", range="decel", example="cap:0.3", optional=True),
                _inp("band_mm", "Band (push band on a push row)", range="push_band", example="cap:0.1", optional=True),
                _inp("push_pct", "Push force (0 = normal move)", range="push0", example=0, optional=True),
                _inp("mode", "Mode", "choice", optional=True,
                     choices=[["", "keep"], ["normal", "normal move"], ["push+", "push away from home"],
                              ["push-", "push toward home"]]),
                _inp("zone_plus_mm", "Zone +", range="position", example="", optional=True),
                _inp("zone_minus_mm", "Zone -", range="position", example="", optional=True)]},
    {"name": "teach_position", "group": "Configuration", "label": "Teach current position (non-volatile)",
     "call": "ec.teach_position(n, method=...)", "confirm": "nonvolatile",
     "desc": "Store the present position as stored row n (the row's other fields are kept). Needs a homed axis at "
             "rest. Back the unit up first. Nothing is written when the row already holds this position.",
     "inputs": [_inp("n", "Row", "int", range="stored_rows", example=2),
                _inp("method", "Method", "choice", optional=True,
                     choices=[["table", "table (read the position, write the row)"],
                              ["coil", "coil (MOD / POSR / TEAC; EXPERIMENTAL)"]])]},
    {"name": "write_parameter", "group": "Configuration", "label": "Write parameter (non-volatile)",
     "call": "ec.write_parameter(key, value)", "confirm": "nonvolatile",
     "desc": "Write one parameter, verify by read-back and save it; applies after a software reset. A value the "
             "unit already holds is neither written nor committed. A dangerous one (home, encoder, gains, axis no., "
             "baud...) is written too, with a warning in the result.",
     "inputs": [_inp("key", "Parameter", "text", example="P2"), _inp("value", "Value", "text", example="5")]},
    {"name": "software_reset", "group": "Configuration", "label": "Software reset", "call": "ec.software_reset()",
     "confirm": "reset", "desc": "Restart the unit's firmware (servo off first); the link is re-synced in place.",
     "inputs": []},
]


def _baud_code_note(rate: int) -> str:
    """Suffix of a baud rate's label: nothing for a verified code, a note for a code some units ignore (38400: the
    EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod ignores it), UNVERIFIED for the rest."""
    if rate in VERIFIED_BAUD_RATES:
        return ""
    return " (depends on the unit)" if rate in UNIT_DEPENDENT_BAUD_RATES else " (UNVERIFIED code)"


PORT_FUNCTIONS: list[dict[str, Any]] = [
    {"name": "scan", "group": "Port", "label": "Scan addresses", "call": "Port.scan(slaves)",
     "desc": "Find the axes answering on this port (FC03 probe + FC11 identity) and add the new ones.",
     "inputs": [_inp("slaves", "Addresses (e.g. 1-16 or 1,2,5)", "text", example="1-16")]},
    {"name": "set_baudrate", "group": "Port", "label": "Change baud rate", "call": "Port.set_baudrate(rate, method=...)",
     "confirm": "danger", "desc": "Broadcast baud change: EVERY unit on this port switches (volatile).",
     "inputs": [_inp("rate", "Baud rate", "choice", example=115200,
                     choices=[[r, f"{r}" + _baud_code_note(r)] for r in sorted(IAI_BAUD_CODES)]),
                _inp("method", "Method", "choice", optional=True,
                     choices=[["direct", "direct"], ["break", "break (line BREAK, then 9600)"]])]},
]

GROUP_FUNCTIONS: list[dict[str, Any]] = [
    {"name": "move_all", "label": "Move selected together", "call": "Cell.move_all(targets, wait=False)",
     "desc": "Every move is checked against its own unit's ranges before any axis starts."},
    {"name": "home_all", "label": "Home selected", "call": "ec.home(wait=False) on each", "desc": "Home return on each."},
    {"name": "servo_all", "label": "Servo ON / OFF selected", "call": "Cell.servo_all(on)", "desc": "Servo on / off on each."},
    {"name": "stop_all", "label": "STOP ALL", "call": "Cell.stop_all(broadcast=False)",
     "desc": "STOP to every axis first, then wait. With broadcast: one broadcast STOP on every daisy chain first "
             "(never behind a gateway; UNVERIFIED on hardware)."},
    {"name": "status_all", "label": "Status of all", "call": "Cell.status_all()", "desc": "One status read per axis."},
]

_EVIDENCE = re.compile(r"\b(?:VERIFIED|UNVERIFIED|DOC|ON-WIRE)\b|\b0x[0-9A-Fa-f]+\b")
_EVIDENCE_START = re.compile(r"^(?:VERIFIED|UNVERIFIED|DOC|ON-WIRE)\b")
_PAREN = re.compile(r"\s*\(([^()]*)\)")


def split_evidence(desc: str) -> tuple[str, list[str]]:
    """A description without its evidence notes, and the notes: a parenthesis that holds an evidence word (VERIFIED,
    UNVERIFIED, DOC, ON-WIRE) or a 0x number, and trailing sentences that start with an evidence word."""
    evidence: list[str] = []

    def cut(match: re.Match[str]) -> str:
        if _EVIDENCE.search(match.group(1)):
            evidence.append(match.group(1).strip())
            return ""
        return match.group(0)

    text = _PAREN.sub(cut, desc)
    sentences = re.split(r"(?<=[.!?])\s+", text.strip())
    tail: list[str] = []
    while len(sentences) > 1 and _EVIDENCE_START.match(sentences[-1]):
        tail.insert(0, sentences.pop().rstrip(".").strip())
    evidence.extend(tail)
    plain = " ".join(sentences).strip()
    plain = re.sub(r"\s+([.,;:])", r"\1", plain)
    if plain and plain[-1] not in ".!?":
        plain += "."
    return plain, evidence


for _spec in (*FUNCTIONS, *PORT_FUNCTIONS, *GROUP_FUNCTIONS):
    _spec["plain"], _spec["evidence"] = split_evidence(_spec["desc"])

_BY_NAME = {f["name"]: f for f in FUNCTIONS}


def catalog() -> dict[str, Any]:
    """The function catalogue for the browser (descriptions, inputs, range keys, examples), the non-volatile memory
    note and the simulator setups."""
    return {"axis": FUNCTIONS, "port": PORT_FUNCTIONS, "group": GROUP_FUNCTIONS,
            "sim_setups": [{"name": k, "label": v[0]} for k, v in SIM_SETUPS.items()],
            "nv": {"cycles": NV_WRITE_CYCLES, "warn_per_hour": NV_WARN_PER_HOUR, "skip_note": NV_SKIP_NOTE}}


# ====================================================================== port open errors


_IN_USE_TEXT = ("Access is denied", "PermissionError(13", "busy")
_NOT_FOUND_TEXT = ("FileNotFoundError(2", "cannot find the file", "No such file")


def _chain(exc: BaseException) -> list[BaseException]:
    out: list[BaseException] = []
    e: BaseException | None = exc
    while e is not None and all(e is not seen for seen in out):
        out.append(e)
        e = e.__cause__ or e.__context__
    return out


def classify_open_error(exc: BaseException, port: str, *, windows: bool | None = None) -> tuple[str, str]:
    """Why a port did not open, as ``(kind, message)``: ``in_use``, ``no_permission``, ``not_found`` or
    ``open_failed``. By type, errno and text: pyserial on Windows raises ``SerialException`` with errno None and
    text such as "could not open port 'COM3': PermissionError(13, 'Access is denied.', None, 5)"."""
    windows = os.name == "nt" if windows is None else windows
    chain = _chain(exc)
    errnos = {getattr(e, "errno", None) for e in chain}
    text = " ".join(str(e) for e in chain)
    permission = any(isinstance(e, PermissionError) for e in chain)
    if (windows and permission) or 16 in errnos or any(t in text for t in _IN_USE_TEXT):
        return "in_use", (f"{port} is in use by another program. Close IA-OS (or disconnect it there) and any other "
                          "program using the port, then press Connect again.")
    if not windows and (permission or 13 in errnos or "Permission denied" in text):
        return "no_permission", (f"No permission to open {port}. On Linux add your user to the dialout group and log "
                                 "in again.")
    if any(isinstance(e, FileNotFoundError) for e in chain) or 2 in errnos or any(t in text for t in _NOT_FOUND_TEXT):
        return "not_found", f"{port} is not there (adapter unplugged?). Press Refresh."
    return "open_failed", f"Cannot open {port}: {describe(exc)}. Close IA-OS (or any program using it) first."


def parse_slaves(text: str | None) -> list[int]:
    """``"1-16"`` / ``"1,2,5"`` / ``"3"`` -> addresses (1..247)."""
    if text is None or not str(text).strip():
        return list(DEFAULT_SLAVES)
    out: list[int] = []
    for part in str(text).replace(" ", "").split(","):
        if not part:
            continue
        try:
            if "-" in part:
                a, b = (int(x) for x in part.split("-", 1))
            else:
                a = b = int(part)
        except ValueError:
            raise BackendError(f"Addresses must look like 1-16 or 1,2,5 (got {text!r}).", 400) from None
        if not 1 <= a <= b <= 247:  # before the range is built: "1-999999999" must not allocate a billion numbers
            raise BackendError("Addresses must be 1..247.", 400)
        out.extend(range(a, b + 1))
    if not out or len(out) > 247:
        raise BackendError("Addresses must be 1..247.", 400)
    return list(dict.fromkeys(out))


# ====================================================================== state


class AxisSlot:
    """One connected axis and what the GUI knows about it."""

    def __init__(self, name: str, port: str, slave: int, ec: Elecylinder) -> None:
        self.name = name
        self.port = port
        self.slave = slave
        self.ec = ec
        self.lock = threading.RLock()
        self.info: dict[str, Any] = {}
        self.limits: dict[str, Any] = {}
        self.status: dict[str, Any] | None = None
        self.error: str | None = None
        self.busy: str | None = None
        self.last: dict[str, Any] | None = None
        self.home_started: float | None = None
        self.home_seen = False
        self.read_at = 0.0
        self.fault: dict[str, Any] | None = None  # a lost link / restarted unit, until recover
        self.recovery: dict[str, Any] | None = None  # the last recover() report (or its failure)
        self.recovering = False  # recover() runs (its own result is logged, not the "recovered" event)
        self.check: dict[str, Any] | None = None  # the last self-test (with its items, for the test report)
        self.pio_at_connect = False  # PIO had control when the axis was added: a disconnect hands it back
        self.pio_hold = False  # the operator chose PIO control: no command takes PC control back by itself
        self.stop_epoch = 0  # counts the STOPs sent to this axis: a command that waited for the lock sees it change

    def monitor_summary(self) -> dict[str, Any] | None:
        mon = self.ec.monitor
        if mon is None:
            return None
        rate, age = mon.poll_rate_hz, mon.age_s
        return {"running": mon.running, "polls": mon.polls, "errors": mon.error_count, "link_checks": mon.link_checks,
                "poll_rate_hz": None if rate is None else round(rate, 1),
                "age_ms": None if age is None else round(age * 1000),
                "error": None if mon.error is None else describe(mon.error)}

    def nv_counts(self) -> tuple[int | None, int | None]:
        """Non-volatile writes this process made on the unit (all, in the last hour); no bus access."""
        try:
            return int(self.ec.nonvolatile_writes), int(self.ec.nonvolatile_writes_last_hour)
        except Exception:  # noqa: BLE001 - an axis object without the counter
            return None, None

    def summary(self) -> dict[str, Any]:
        nv, nv_hour = self.nv_counts()
        check = None if self.check is None else {k: self.check[k] for k in ("result", "t", "counts")}
        return {"name": self.name, "port": self.port, "slave": self.slave, "axis_number": self.slave - 1,
                "info": self.info, "limits": self.limits, "status": self.status, "error": self.error,
                "busy": self.busy, "last": self.last, "homing_watch": self.home_started is not None,
                "age_s": None if not self.read_at else round(time.monotonic() - self.read_at, 2),
                "fast": bool(self.ec.fast_mode), "completion": self.ec.completion, "monitor": self.monitor_summary(),
                "link_state": self.ec.link_state, "fault": self.fault, "recovery": self.recovery,
                "nv_writes": nv, "nv_writes_last_hour": nv_hour, "check": check, "pio_hold": self.pio_hold}


class PortSlot:
    def __init__(self, name: str, port: Port, transport: Any, simulated: bool, setup: str | None, *,
                 baud_detected: bool = False, is_iai: bool | None = None, description: str = "") -> None:
        self.name = name
        self.port = port
        self.transport = transport
        self.simulated = simulated
        self.setup = setup
        self.baud_detected = baud_detected  # the connect asked for auto-detect
        self.is_iai = is_iai  # None: a simulator
        self.description = description

    def summary(self, axes: Iterable[str]) -> dict[str, Any]:
        return {"name": self.name, "simulated": self.simulated, "setup": self.setup, "baud": self.port.baudrate,
                "line": self.port.line.kind, "axes": list(axes), "baud_detected": self.baud_detected,
                "is_iai": self.is_iai}


class _LogHandler(logging.Handler):
    def __init__(self, wb: Workbench) -> None:
        super().__init__(logging.WARNING)
        self.wb = wb

    def emit(self, record: logging.LogRecord) -> None:
        if record.name == RC.log.name and record.levelno < logging.ERROR:
            return  # link events: the workbench reports them itself (_link_event), with the axis's fault banner
        try:
            self.wb.event("warn" if record.levelno < logging.ERROR else "error", record.getMessage(), "driver")
        except Exception:  # noqa: BLE001 - logging must never raise
            pass


# ====================================================================== workbench


class Workbench:
    """Ports, axes, a poll thread and the dispatch of every driver function."""

    def __init__(self, *, poll_interval_s: float = 0.15, sim_kwargs: dict[str, Any] | None = None,
                 port_lister: Callable[[], list[dict[str, Any]]] = list_ports, sim_only: bool = False) -> None:
        self.poll_interval_s = poll_interval_s
        self.sim_only = sim_only  # demo mode: no serial port is listed or opened
        self.sim_kwargs = dict(sim_kwargs or {})
        self.port_lister = port_lister
        self.ports: dict[str, PortSlot] = {}
        self.axes: dict[str, AxisSlot] = {}
        self.cell: Cell | None = None
        self._lock = threading.RLock()  # ports / axes dicts
        self._events: deque[dict[str, Any]] = deque(maxlen=MAX_EVENTS)
        self._seq = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._handler = _LogHandler(self)
        self._sim_count = 0
        self._listed: list[dict[str, Any]] = []  # the last port listing (is_iai, description)
        self.coord = CoordinatedRunner(self)

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            logging.getLogger("iai_actuator_driver").addHandler(self._handler)
            self._thread = threading.Thread(target=self._poll_loop, name="workbench-poll", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self.coord.close()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        logging.getLogger("iai_actuator_driver").removeHandler(self._handler)
        for name in list(self.ports):
            try:
                self.disconnect(name)
            except BackendError:
                pass

    def event(self, level: str, message: str, source: str = "") -> None:
        with self._lock:
            self._seq += 1
            self._events.append({"seq": self._seq, "t": time.time(), "level": level, "source": source,
                                 "message": message})

    # ------------------------------------------------------------------ discovery / connection

    def list_ports(self) -> dict[str, Any]:
        """Serial ports (IAI adapters first; READ-ONLY, nothing is opened) and the simulator setups."""
        try:
            ports = [] if self.sim_only else self.port_lister()
        except Exception as exc:  # noqa: BLE001
            ports = []
            self.event("warn", f"listing serial ports failed: {describe(exc)}")
        self._listed = [dict(p) for p in ports]
        connected = set(self.ports)
        for p in ports:
            p["connected"] = p["port"] in connected
        return {"ports": ports, "sim_setups": catalog()["sim_setups"], "sim_only": self.sim_only}

    def connect(self, *, port: str | None = None, simulate: bool = False, sim_setup: str = "slider",
                slaves: str | None = None, baud: int | None = None) -> dict[str, Any]:
        """Open a port (or a simulator setup), scan it for axes and identify each one (READ-ONLY)."""
        if self.coord.active():
            raise BackendError(RUN_REFUSED, 409)
        if simulate:
            if sim_setup not in SIM_SETUPS:
                raise BackendError(f"Unknown simulator setup {sim_setup!r}; choose from {sorted(SIM_SETUPS)}.", 400)
            added = []
            for units in SIM_SETUPS[sim_setup][1]:
                self._sim_count += 1
                name = f"SIM{self._sim_count}"
                kwargs = {"baudrate": SIM_BAUD, **self.sim_kwargs}
                line = SimLine([SimulatedEC(slave=s, profile=p, **kwargs) for s, p in units], name=name)
                added.append(self._open_port(name, line, True, sim_setup, [s for s, _ in units], None))
            return {"connected": added}
        if self.sim_only:
            raise BackendError("Simulator-only mode (--sim-only): serial ports are not opened.", 409)
        if not port or not isinstance(port, str):
            raise BackendError("Choose a port (e.g. COM3).", 400)
        port = self._listed_name(check_port_name(port))  # a serial port name, and one the PC lists
        return {"connected": [self._open_port(port, port, False, None, parse_slaves(slaves), baud)]}

    def _listed_name(self, name: str) -> str:
        """``name`` as the PC's port listing spells it. A name the listing does not hold is refused (404, kind
        ``not_found``): only a port the page offered is opened. The listing of the last Refresh is used first, then one
        fresh listing; a listing that cannot be read does not block (the name was checked already)."""

        def key(text: str) -> str:
            return text.upper() if text[:3].upper() == "COM" else text  # COM names ignore case, device paths do not

        for fresh in (False, True):
            if fresh:
                try:
                    self._listed = [dict(p) for p in self.port_lister()]
                except Exception as exc:  # noqa: BLE001
                    self.event("warn", f"listing serial ports failed: {describe(exc)}; {name} is opened unchecked")
                    return name
            hit = next((str(p.get("port")) for p in self._listed if key(str(p.get("port"))) == key(name)), None)
            if hit is not None:
                return hit
        raise BackendError(f"{name} is not in the list of serial ports (adapter unplugged?). Press Refresh.", 404,
                           kind="not_found", port=name)

    def _port_listing(self, name: str) -> dict[str, Any] | None:
        """What the port listing says about a serial port (no port is opened)."""
        entry = next((p for p in self._listed if p.get("port") == name), None)
        if entry is None:
            try:
                self._listed = [dict(p) for p in self.port_lister()]
            except Exception:  # noqa: BLE001 - only the IAI flag and the description are missing then
                return None
            entry = next((p for p in self._listed if p.get("port") == name), None)
        return entry

    def _open_port(self, name: str, transport: Any, simulated: bool, setup: str | None, slaves: list[int],
                   baud: int | None) -> dict[str, Any]:
        with self._lock:
            if name in self.ports:
                raise BackendError(f"{name} is already connected.", 409)
        try:
            port = Port(transport, name=name, baudrate=baud)
        except Exception as exc:  # noqa: BLE001 - the port could not be opened: say why
            kind, message = classify_open_error(exc, name)
            raise BackendError(message, 502, kind=kind, port=name) from None
        listing = None if simulated else self._port_listing(name)
        slot = PortSlot(name, port, transport, simulated, setup, baud_detected=not simulated and baud is None,
                        is_iai=None if simulated else bool(listing and listing.get("is_iai")),
                        description=str((listing or {}).get("description") or ""))  # fmt: skip
        try:
            hits = port.scan(slaves)
        except Exception as exc:  # noqa: BLE001
            port.close()
            raise to_backend_error(exc) from None
        if not hits:
            port.close()
            raise BackendError(f"{name}: no axis answered at addresses {slaves[0]}..{slaves[-1]} at any baud rate; "
                               f"{E.NO_ANSWER_HINT}.", 409, kind="no_answer", port=name)
        with self._lock:
            self.ports[name] = slot
        found = []
        for hit in hits:
            found.append(self._add_axis(slot, hit.slave))
        self._rebuild_cell()
        self.event("info", f"{name}: connected at {port.baudrate or 'simulated'} baud, axes {', '.join(found)}"
                   + (f" ({SIM_SETUPS[setup][0]})" if setup else ""))
        self.start()
        return slot.summary(found)

    def _add_axis(self, pslot: PortSlot, slave: int) -> str:
        name = axis_name(pslot.name, slave)
        with self._lock:
            if name in self.axes:
                return name
        ec = pslot.port.axis(slave)
        aslot = AxisSlot(name, pslot.name, slave, ec)
        with aslot.lock:
            try:
                info = ec.identify()
                aslot.info = _info_summary(info)
            except Exception as exc:  # noqa: BLE001 - keep the axis; show why identify failed
                aslot.error = f"identify: {describe(exc)}"
            else:
                try:  # the baseline the link checks compare with (read-only)
                    RC.track(ec, name=name, info=info)
                except Exception as exc:  # noqa: BLE001 - a lost link is still seen; a restart is not
                    self.event("warn", f"{name}: not tracked for power-loss detection: {describe(exc)}")
            self._refresh_limits(aslot)
            self._read_status(aslot)
        aslot.pio_at_connect = (aslot.status or {}).get("modbus_enabled") is False
        with self._lock:
            self.axes[name] = aslot
        return name

    def disconnect(self, port: str | None = None) -> dict[str, Any]:
        """Close a port (all when ``port`` is None) and leave each unit safe and as found: a coordinated run on one of
        its axes is stopped first; per axis a fresh status read, STOP (waiting for the standstill) when it moves, homes
        or cannot be read fresh (also when its lock stays busy), the HOME coil released, control handed back to PIO
        when PIO had it at connect. The servo is never switched. Never refused during a run."""
        with self._lock:
            names = list(self.ports) if port is None else [port]
            for n in names:
                if n not in self.ports:
                    raise BackendError(f"{n} is not connected.", 404)
        report: dict[str, dict[str, Any]] = {}
        for n in names:
            with self._lock:
                on_port = [a.name for a in self.axes.values() if a.port == n]
            self.coord.stop_for(on_port)
            with self._lock:
                pslot = self.ports.pop(n)
                slots = [a for a in self.axes.values() if a.port == n]
                for a in slots:
                    self.axes.pop(a.name, None)
            notes = []
            for a in slots:
                result = self._leave_safe(a)
                report[a.name] = result
                notes.append(f"{a.name} {result['note']}")
                try:
                    a.ec.close()
                except Exception:  # noqa: BLE001
                    pass
            pslot.port.close()
            self.event("info", f"{n}: disconnected" + (f" ({'; '.join(notes)})" if notes else ""))
        self._rebuild_cell()
        return {"disconnected": names, "axes": report}

    def _leave_safe(self, a: AxisSlot) -> dict[str, Any]:
        """One axis before its port closes (see :meth:`disconnect`); returns what was done."""
        out: dict[str, Any] = {"stopped": False, "stop_confirmed": None, "home_released": False,
                               "pio_restored": False, "servo": None, "note": ""}  # fmt: skip
        locked = a.lock.acquire(timeout=DISCONNECT_LOCK_S)
        try:
            st: Any = None
            if locked:
                try:
                    st = a.ec.status()
                except Exception as exc:  # noqa: BLE001 - STOP anyway
                    self.event("warn", f"{a.name}: status read before disconnect failed: {describe(exc)}")
            else:
                self.event("warn", f"{a.name}: busy, no fresh status before disconnect: STOP anyway")
            cached = a.status or {}
            moving = bool(st.moving) if st is not None else bool(cached.get("moving"))
            homing = bool(st.homing) if st is not None else bool(cached.get("homing"))
            # STOP unless a fresh read under the axis lock shows it at rest (a cached status can be old: the axis may
            # have been moved since, by a command that still holds the lock)
            if moving or homing or st is None:
                out["stopped"] = True
                try:
                    st = a.ec.stop(wait=True, timeout=DISCONNECT_STOP_S) or st
                    out["stop_confirmed"] = True
                except Exception as exc:  # noqa: BLE001 - said loudly: the axis may still move
                    out["stop_confirmed"] = False
                    self.event("error", f"{a.name}: STOP NOT CONFIRMED before disconnect: {describe(exc)}")
            if a.home_started is not None or homing:
                try:
                    a.ec.finish_home()
                    a.home_started = None
                    out["home_released"] = True
                except Exception as exc:  # noqa: BLE001
                    self.event("warn", f"{a.name}: releasing the HOME coil failed: {describe(exc)}")
            servo = st.servo_on if st is not None else cached.get("servo_on")
            out["servo"] = None if servo is None else ("on" if servo else "off")
            pio_note = ""
            if a.pio_at_connect:
                modbus = st.modbus_enabled if st is not None else cached.get("modbus_enabled")
                at_rest = out["stop_confirmed"] is not False and (st is None or (not st.moving and not st.homing))
                if modbus is False:
                    out["pio_restored"], pio_note = True, "control left with PIO"
                elif at_rest and (locked or out["stopped"]):
                    try:
                        a.ec.disable_modbus_control()
                        out["pio_restored"], pio_note = True, "control handed back to PIO"
                    except Exception as exc:  # noqa: BLE001
                        pio_note = "control NOT handed back to PIO"
                        self.event("warn", f"{a.name}: handing control back to PIO failed: {describe(exc)}")
                else:
                    pio_note = "control NOT handed back to PIO"
        finally:
            if locked:
                a.lock.release()
        parts = ["STOP NOT CONFIRMED" if out["stop_confirmed"] is False else "stopped" if out["stopped"] else "at rest"]
        if out["servo"] is not None:
            parts.append(f"servo left {out['servo']}")
        if out["home_released"]:
            parts.append("HOME coil released")
        if pio_note:
            parts.append(pio_note)
        out["note"] = ", ".join(parts)
        return out

    def _rebuild_cell(self) -> None:
        with self._lock:
            axes = sorted(self.axes.values(), key=lambda a: (list(self.ports).index(a.port), a.slave))
            self.cell = Cell({n: p.port for n, p in self.ports.items()}, [(a.name, a.ec) for a in axes]) if axes else None
        self.coord.invalidate("the connected axes changed")

    # ------------------------------------------------------------------ snapshot

    def snapshot(self, since: int = 0) -> dict[str, Any]:
        with self._lock:
            axes = [a.summary() for a in self.axes.values()]
            ports = [p.summary(a.name for a in self.axes.values() if a.port == p.name) for p in self.ports.values()]
            events = [e for e in self._events if e["seq"] > since]
            seq = self._seq
        return {"ports": ports, "axes": axes, "events": events, "seq": seq, "polling": self._thread is not None,
                "coord": self.coord.summary()}

    def _refuse_in_run(self, names: Iterable[str]) -> None:
        """An axis in a coordinated run refuses lock-taking requests and group operations at once."""
        for name in names:
            if self.coord.axis_in_run(name):
                raise BackendError(f"{name} is in a coordinated run: STOP ALL (Space / Esc) ends it.", 409)

    def _refuse_pio_hold(self, names: Iterable[str], what: str) -> None:
        """An axis the operator put in PIO mode follows its PIO inputs: a command that needs PC control is refused with
        the reason (its automatic take-over is off), not sent to a unit that would ignore it."""
        for name in names:
            slot = self.axes.get(name)
            if slot is not None and slot.pio_hold:
                raise BackendError(f"{name} is in PIO mode (PC / PIO switch): the unit follows its PIO inputs, so {what} "
                                   "was not sent. Switch it to PC first.", 409)

    @staticmethod
    def _note_stop(slots: Iterable[AxisSlot]) -> None:
        """A STOP is about to be sent to these axes: a motion command that has not been sent yet (it waits for the axis
        lock, or has just taken it) sees the change of the axis's ``stop_epoch`` and is cancelled."""
        for slot in slots:
            slot.stop_epoch += 1

    def note_stop_axes(self, names: Iterable[str]) -> None:
        """:meth:`_note_stop` for axes by name (a STOP of the Coordinated tab's two axes)."""
        with self._lock:
            slots = [self.axes[n] for n in names if n in self.axes]
        self._note_stop(slots)

    def stop_epochs(self, names: Iterable[str]) -> dict[str, int]:
        """The STOP counters of these axes now, to compare with :meth:`stopped_since` later."""
        with self._lock:
            return {n: self.axes[n].stop_epoch for n in names if n in self.axes}

    def stopped_since(self, marks: dict[str, int]) -> bool:
        """Whether a STOP reached one of the axes of ``marks`` (:meth:`stop_epochs`) after it was taken."""
        with self._lock:
            return any(n in self.axes and self.axes[n].stop_epoch != e for n, e in marks.items())

    def _stop_again(self, slots: list[AxisSlot], what: str) -> None:
        """A STOP was pressed while ``what`` was being sent, so it may have reached the unit before the command did:
        STOP goes out again to these axes (as the single-axis page does)."""
        if not slots:
            return
        names = ", ".join(s.name for s in slots)
        errors = []
        for slot in slots:
            try:
                self._fn_stop(slot, {})
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{slot.name}: {describe(exc)}")
        if errors:
            raise BackendError(f"STOP after {what} FAILED: {'; '.join(errors)}. Use the hardware stop.", 502)
        self.event("warn", f"STOP re-sent on {names}: it was pressed while {what} was being sent")

    def _slot(self, name: Any) -> AxisSlot:
        with self._lock:
            slot = self.axes.get(str(name))
        if slot is None:
            raise BackendError(f"No axis {name!r} (connect a port first).", 404)
        return slot

    # ------------------------------------------------------------------ poll

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            with self._lock:
                slots = list(self.axes.values())
                lines = [p.transport for p in self.ports.values() if p.simulated]
            for slot in slots:
                if self._stop.is_set():
                    break
                if slot.lock.acquire(blocking=False):
                    try:
                        self._read_status(slot)
                    finally:
                        slot.lock.release()
            for line in lines:  # the simulated line keeps every request: keep it short
                if isinstance(line, SimLine) and len(line.requests) > 2000:
                    del line.requests[:-200]
            self._stop.wait(max(0.02, self.poll_interval_s - (time.monotonic() - started)))

    def _read_status(self, slot: AxisSlot) -> None:
        mon = slot.ec.monitor
        st = mon.fresh(MONITOR_AGE_S) if mon is not None else None
        source = "monitor"
        if st is None:
            source = "read"
            try:
                chk = RC.check_link(slot.ec, name=slot.name, on_event=lambda e, s=slot: self._link_event(s, e))
                if chk.state == RC.RECOVERING:
                    return
                if chk.status is None:
                    raise chk.error if chk.error is not None else E.ModbusError(chk.state)
                st = chk.status
            except Exception as exc:  # noqa: BLE001
                msg = describe(exc)
                if slot.error != msg:
                    self.event("error", f"{slot.name}: status read failed: {msg}")
                slot.error = msg
                return
        prev = slot.status or {}
        data = st.to_dict()
        status = {k: jsonable(data.get(k)) for k in STATUS_KEYS}
        status["alarm_name"] = alarm_name(st.alarm_code) if st.alarm_code else None
        status["source"] = source
        slot.status = status
        slot.read_at = time.monotonic()
        if slot.error and slot.error.startswith(("ModbusTimeout", "ModbusCRC", "ModbusFrame", "OSError")):
            slot.error = None
        if st.alarm_code and st.alarm_code != prev.get("alarm_code"):
            self.event("error" if st.alarm else "warn", f"{slot.name}: alarm {st.alarm_code:#05x} {status['alarm_name']}")
        if prev.get("moving") and not st.moving:
            self.event("info", f"{slot.name}: stopped at {st.position_mm:.2f} {slot.limits.get('units', 'mm')}")
        if slot.home_started is not None:
            self._follow_home(slot, st)

    def _follow_home(self, slot: AxisSlot, st: Any) -> None:
        assert slot.home_started is not None
        slot.home_seen = slot.home_seen or st.homing or not st.homed or st.move_bit
        done = slot.home_seen and st.homed and st.in_position and not st.homing
        ended = st.alarm or not st.servo_on
        late = not slot.home_seen and time.monotonic() - slot.home_started >= HOME_START_S
        if done or ended or late:
            try:
                slot.ec.finish_home()
            except Exception as exc:  # noqa: BLE001 - retried at the next poll
                self.event("warn", f"{slot.name}: releasing the HOME coil failed: {describe(exc)}")
                return
            slot.home_started = None
            self.event("info" if done else "warn", f"{slot.name}: " + (
                "home return complete" if done else
                "home return ended by an alarm / servo off" if ended else
                f"home return did not start within {HOME_START_S:g} s; HOME coil released"))

    def _refresh_limits(self, slot: AxisSlot) -> None:
        try:
            limits = jsonable(slot.ec.limits())
        except Exception as exc:  # noqa: BLE001
            self.event("warn", f"{slot.name}: limits not readable: {describe(exc)}")
            return
        span = limits.get("jog_step", {}).get("max")
        units = limits.get("units", "mm")
        limits["relative"] = {"min": None if span is None else -span, "max": span, "unit": units,
                              "source": "+/- the stroke"}
        push = dict(limits.get("push", {}))
        push["min"] = 0.0
        limits["push0"] = push
        slot.limits = limits

    def _link_event(self, slot: AxisSlot, event: Any) -> None:
        """A link event of one axis (from the poll's link check or the axis's monitor thread): the fault banner."""
        if event.kind == RC.EVENT_RECOVERED:
            slot.fault = None
            if not slot.recovering:
                self.event("info", f"{slot.name}: link back ({event.message})")
            return
        kind = event.cause if event.cause in _FAULT_KINDS else None
        if kind is None:
            return
        slot.fault = {"kind": kind, "event": event.kind, "text": _FAULT_TEXT[kind], "message": event.message,
                      "t": event.time}  # fmt: skip
        self.event("error", f"{slot.name}: {event.kind.replace('_', ' ')}: {_FAULT_TEXT[kind]} ({event.message})")

    # ------------------------------------------------------------------ calls

    def call(self, axis: Any, fn: Any, args: Any = None) -> dict[str, Any]:
        """Run one catalogue function on one axis; the result is JSON (the driver's error text on failure)."""
        slot = self._slot(axis)
        mark = slot.stop_epoch  # a STOP from here on cancels a motion command that is still waiting for the lock
        spec = _BY_NAME.get(str(fn))
        if spec is None:
            raise BackendError(f"Unknown function {fn!r}.", 400)
        if args is None:
            args = {}
        if not isinstance(args, dict):
            raise BackendError("args must be an object.", 400)
        runner = getattr(self, f"_fn_{spec['name']}", None)
        motion = spec["name"] in NEEDS_PC_CONTROL
        started = time.monotonic()
        if spec["name"] == "stop":
            self._note_stop([slot])
        try:
            if spec.get("nolock"):
                result = runner(slot, args) if runner else getattr(slot.ec, spec["name"])()
            else:
                self._refuse_in_run([slot.name])
                if motion:
                    self._refuse_pio_hold([slot.name], spec["label"])
                if not slot.lock.acquire(timeout=10.0):
                    raise BackendError(f"{slot.name} is busy with {slot.busy or 'another request'}; try again.", 409)
                slot.busy = spec["label"]
                try:
                    if motion and slot.stop_epoch != mark:  # STOP came while this waited: it must not run after it
                        raise BackendError(STOP_CANCELLED.format(what=spec["label"]), 409)
                    result = runner(slot, args) if runner else getattr(slot.ec, spec["name"])()
                    if motion and slot.stop_epoch != mark:  # STOP came while it was sent: it may have landed before
                        self._stop_again([slot], spec["label"])
                finally:
                    slot.busy = None
                    slot.lock.release()
        except Exception as exc:  # noqa: BLE001
            err = to_backend_error(exc)
            slot.last = {"fn": spec["name"], "ok": False, "error": str(err), "t": time.time()}
            self.event("error", f"{slot.name}: {spec['label']} refused: {err}")
            raise err from None
        out = jsonable(result)
        writes = spec["group"] != "Read" and not spec.get("readonly")
        if writes:  # show the new state at once, when the axis lock is free
            # STOP, pause and resume took no axis lock: their reply must not wait for it either (a plan, a backup or a
            # run can hold it for long); the poll shows the new state a moment later.
            got = slot.lock.acquire(blocking=False) if spec.get("nolock") else slot.lock.acquire(timeout=1.0)
            if got:
                try:
                    self._read_status(slot)
                finally:
                    slot.lock.release()
        slot.last = {"fn": spec["name"], "ok": True, "t": time.time(), "ms": round((time.monotonic() - started) * 1000)}
        if writes:
            self.event("info", f"{slot.name}: {spec['label']} ok")
        return {"axis": slot.name, "fn": spec["name"], "result": out}

    # Each _fn_ runs under the axis lock (except the nolock ones) and returns a JSON-able result.

    def _fn_set_fast(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        ec = slot.ec
        if not _bool(a, "on", True):
            return ec.disable_fast()
        ec.enable_fast(baudrate=None, monitor=False)
        if ec.monitor is None or not ec.monitor.running:  # a running one already reports to this slot
            ec.start_monitor(on_event=lambda e, s=slot: self._link_event(s, e), name=f"wb-monitor-{slot.name}")
        return {"baudrate": ec.baudrate, "completion": ec.completion, "monitor": slot.monitor_summary(),
                "notes": list(ec.fast_notes)}  # fmt: skip

    def _fn_self_check(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        report = run_check(slot.ec, motion=False, port_label=slot.port)
        tally = Counter(item.result for item in report.items)
        result = {"result": "PASS" if report.passed else "FAIL", "passed": report.passed,
                  "counts": {k: tally.get(k, 0) for k in ("PASS", "WARN", "FAIL", "SKIP")},
                  "items": [{"name": i.name, "result": i.result, "detail": i.detail} for i in report.items],
                  "t": time.time()}  # fmt: skip
        slot.check = result
        counts = result["counts"]
        self.event("info" if report.passed else "warn", f"{slot.name}: self-test {result['result']} ("
                   + ", ".join(f"{counts[k]} {k}" for k in ("PASS", "WARN", "FAIL", "SKIP")) + ")")
        return result

    def _fn_check_link(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.check_link(name=slot.name, on_event=lambda e, s=slot: self._link_event(s, e))

    def _fn_recover(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        timeout = _num(a, "timeout_s", RECOVER_TIMEOUT_S)
        if not 0 < timeout <= 600:
            raise BackendError("timeout_s must be 1..600 s.", 400)
        self.event("info", f"{slot.name}: recovering (waits up to {timeout:g} s; nothing is moved)")
        slot.recovering = True
        try:
            report = slot.ec.recover(timeout=timeout, name=slot.name, on_event=lambda e, s=slot: self._link_event(s, e))
        except RC.RecoveryError as exc:
            slot.recovery = {"ok": False, "error": describe(exc), "t": time.time(),
                             "report": None if exc.report is None else jsonable(exc.report)}  # fmt: skip
            if isinstance(exc, RC.UnitReplacedError):
                slot.fault = {"kind": RC.UNIT_REPLACED, "event": RC.EVENT_UNIT_REPLACED, "message": str(exc),
                              "text": _FAULT_TEXT[RC.UNIT_REPLACED], "t": time.time()}  # fmt: skip
            raise
        finally:
            slot.recovering = False
        slot.recovery = {"ok": True, "t": time.time(), "report": jsonable(report)}
        slot.fault = None
        slot.error = None
        cell = self.cell
        if cell is not None:
            cell.faults.pop(slot.name, None)
        self._refresh_limits(slot)  # a committed parameter may have changed a range at the restart
        self.event("warn" if report.needs_attention else "info", report.summary())
        return report

    def _rest_status(self, slot: AxisSlot, what: str, pause_ok: bool = False) -> Any:
        """A fresh status for a control-mode switch; refused unless the axis is at rest (PMSL ON mid-move stops the
        move; an unknown motion flag is not taken for rest). ``pause_ok`` (taking PC control): a pause at a standstill
        does not count while the unit is in PIO mode with the servo off (a PCON with nothing on its PIO connector
        pauses at rest there); the driver refuses it while anything moves."""
        st = slot.ec.status()
        busy = None
        paused = st.paused and not (pause_ok and st.modbus_enabled is False and not st.servo_on)
        if st.moving is not False:
            busy = "moving" if st.moving else "in an unknown motion state (moving cannot be read)"
        else:
            busy = next((text for text, on in (("homing", st.homing), ("pushing", st.pushing), ("paused", paused))
                         if on), None)
        if busy:
            raise BackendError(f"{what} refused: {slot.name} is {busy}. The control mode is only switched at rest "
                               "(switching it during motion stops or changes the move): press STOP, or wait.", 409)
        return st

    def _fn_enable_modbus_control(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        """PC mode: PMSL on (PMSS follows), the automatic take-over back on; at rest only."""
        st = self._rest_status(slot, "Switching to PC control", pause_ok=True)
        if st.modbus_enabled is not True:
            slot.ec.enable_modbus_control()
        slot.ec.auto_modbus_control = True
        slot.pio_hold = False
        return {"modbus_control": True, "changed": st.modbus_enabled is not True}

    def _fn_disable_modbus_control(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        """PIO mode: PMSL off (PMSS follows) and the automatic take-over off, so that no command of this page takes PC
        control back; at rest only. ``enable_modbus_control`` switches back."""
        st = self._rest_status(slot, "Switching to PIO control")
        if st.modbus_enabled is not False:
            slot.ec.disable_modbus_control()
            after = slot.ec.status()
            if after.modbus_enabled is True:
                raise BackendError(f"{slot.name}: the unit still reports PC control (PMSS = 1) after the switch to PIO.", 502)
        slot.ec.auto_modbus_control = False
        slot.pio_hold = True
        return {"modbus_control": False, "changed": st.modbus_enabled is not False}

    def _fn_servo(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.servo(_bool(a, "on", True), wait=True)

    def _fn_home(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        method = _text(a, "method")
        slot.home_started, slot.home_seen = time.monotonic(), False
        try:
            return slot.ec.home(wait=False, method=method)
        except BaseException:
            try:
                if not slot.ec.status().homing:  # nothing runs: release the HOME coil now
                    slot.home_started = None
                    slot.ec.finish_home()
            except Exception:  # noqa: BLE001 - the poll keeps watching (home_started stays set)
                pass
            raise

    def _motion(self, a: dict[str, Any]) -> tuple[float, float, float | None, float | None]:
        return (_num(a, "speed_mm_s"), _num(a, "accel_g"), _num(a, "decel_g", optional=True),
                _num(a, "band_mm", optional=True))

    def _fn_move_to(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        speed, accel, decel, band = self._motion(a)
        return slot.ec.move_to(_num(a, "mm"), speed, accel, 0.1 if band is None else band, wait=False, decel_g=decel)

    def _fn_check_move(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        speed, accel, decel, band = self._motion(a)
        st = slot.ec.check_move(_num(a, "mm"), speed, accel, 0.1 if band is None else band, decel_g=decel)
        return {"ok": True, "message": "every check passed; nothing was sent", "status": st}

    def _fn_move_by(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        speed, accel, _decel, band = self._motion(a)
        return slot.ec.move_by(_num(a, "delta_mm"), speed, accel, 0.1 if band is None else band, wait=False)

    def _fn_jog(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.jog(_num(a, "delta_mm"), _num(a, "speed_mm_s", 10.0), _num(a, "accel_g", 0.1), wait=False)

    def _fn_push(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        direction = _int(a, "direction", 1)
        if direction not in (1, -1):
            raise BackendError("direction must be 1 (away from home) or -1 (toward home).", 400)
        return slot.ec.push(_num(a, "target_mm"), _num(a, "band_mm"), _num(a, "force_pct"), direction,
                            _num(a, "speed_mm_s", 10.0), _num(a, "accel_g", 0.1), wait=False)

    def _fn_move_to_position(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.move_to_position(_int(a, "n"), wait=False, allow_above_caps=_bool(a, "allow_above_caps"),
                                        speed_pct=_num(a, "speed_pct", optional=True))

    def _fn_change_speed(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.change_speed(_num(a, "speed_mm_s"), wait=False)

    def _fn_change_accel(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.change_accel(_num(a, "accel_g"), wait=False)

    def _fn_stop(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        st = slot.ec.stop(wait=False)
        if slot.home_started is not None:
            slot.ec.finish_home()
            slot.home_started = None
        return st

    def _fn_reset_alarm(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.reset_alarm(method=_text(a, "method"))

    def _fn_set_safety_speed(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return slot.ec.set_safety_speed(_bool(a, "on"))

    def _fn_limits(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        self._refresh_limits(slot)
        return slot.limits

    def _fn_identify(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        info = slot.ec.identify()
        slot.info = _info_summary(info)
        return info

    def _fn_read_position_table(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        entry = slot.ec.read_position_table(_int(a, "n"))
        return _row_dict(_int(a, "n"), entry)

    def _fn_read_alarm_history(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        rows = slot.ec.read_alarm_history(include_empty=_bool(a, "include_empty"))
        return [{**r.to_dict(), "remedy": alarm_info(r.code)["remedy"]} for r in rows]

    def _fn_line_info(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        line = slot.ec.bus.line
        out: dict[str, Any] = {"line": line.to_dict(), "stop_path": slot.ec.stop_path, "gateway_status": None,
                               "monitor": None, "errors": []}
        gateway = line.gateway
        if gateway is not None:
            try:
                out["gateway_status"] = gateway.read_status().to_dict()
            except E.ModbusError as exc:
                out["errors"].append(f"gateway status: {describe(exc)}")
        if _bool(a, "monitor"):
            if not isinstance(gateway, RconGateway):
                out["errors"].append("the all-axes monitor exists on an RCON gateway only")
            else:
                try:
                    record = gateway.read_monitor([slot.slave - 1]).get(slot.slave - 1)
                    out["monitor"] = None if record is None else record.to_dict()
                    unit = line.position_unit_mm(slot.slave)
                    if record is not None and unit:
                        out["monitor"]["position_mm"] = round(record.position_counts * unit, 3)
                except (E.ModbusError, ValueError) as exc:
                    out["errors"].append(f"gateway monitor: {describe(exc)}")
        return out

    def _fn_backup(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        text = (_text(a, "rows", "used") or "used").replace(" ", "")
        if text in ("used", "all"):
            rows: Any = text
        else:
            try:
                rows = [int(p) for p in text.split(",") if p]
            except ValueError:
                raise BackendError("rows must be used, all or row numbers such as 0,1,2.", 400) from None
            if not rows or any(r < 0 for r in rows):
                raise BackendError("rows must be used, all or row numbers such as 0,1,2.", 400)
        safe = "".join(c if c.isalnum() else "-" for c in slot.name)
        path = BACKUP_DIR / f"{safe}-{_dt.datetime.now():%Y%m%d-%H%M%S}.json"
        BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        data = backup(slot.ec, path, rows=rows)
        parameters = data.get("parameters") or {}
        return {"saved_to": str(path), "read_only": True, "seconds": data["meta"].get("duration_s"),
                "parameter_items": sum(len(b.get("items") or []) for b in parameters.get("classes") or []),
                "named_parameters": len(parameters.get("named") or []),
                "position_rows": len((data.get("positions") or {}).get("rows") or []),
                "alarm_entries": len((data.get("alarms") or {}).get("history") or []),
                "errors": data.get("errors") or []}

    def _fn_teach_position(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        n = _int(a, "n")
        entry = slot.ec.teach_position(n, method=_text(a, "method", "table"))
        return _row_dict(n, entry)

    def _fn_alarm_info(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        return alarm_info(_int(a, "code"))

    def _fn_read_parameter(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        key = _text(a, "key")
        if not key:
            raise BackendError("Missing input: key.", 400)
        return slot.ec.read_parameter(int(key) if key.isdigit() else key)

    def _fn_read_registers(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        address, count = _int(a, "address"), _int(a, "count", 8)
        if not 0 <= address <= 0xFFFF or not 1 <= count <= 64:
            raise BackendError("address 0..0xFFFF, count 1..64.", 400)
        words = slot.ec.read_registers(address, count)
        return {"address": f"0x{address:04X}", "words": [f"0x{w:04X}" for w in words], "decimal": list(words)}

    def _fn_unit_info(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        section = _text(a, "section")
        return slot.ec.unit_info(sections=[section] if section else None)

    def _fn_write_position_table(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        n = _int(a, "n")
        entry = slot.ec.read_position_table(n)
        changes: dict[str, Any] = {}
        for key in ("position_mm", "speed_mm_s", "accel_g", "decel_g", "band_mm", "zone_plus_mm", "zone_minus_mm"):
            value = _num(a, key, optional=True)
            if value is not None:
                changes[key] = round(value, 2)
        flags = entry.control_flags
        mode = _text(a, "mode")
        if mode == "normal":
            flags &= ~(R.CTLF_PUSH | R.CTLF_PUSH_AWAY)
        elif mode in ("push+", "push-"):
            flags = (flags | R.CTLF_PUSH) & ~R.CTLF_PUSH_AWAY | (R.CTLF_PUSH_AWAY if mode == "push+" else 0)
        elif mode:
            raise BackendError("mode must be normal, push+ or push-.", 400)
        changes["control_flags"] = flags
        pct = _num(a, "push_pct", optional=True)
        if pct is not None:
            if not 0 <= pct <= 100:
                raise BackendError("push force must be 0..100 %.", 400)
            changes["push_power"] = R.push_power(pct)  # IAI's truncating scale: 70 % is 0xB2, never 0xB3
        new = dataclasses.replace(entry, **changes)
        written = slot.ec.write_position_table(n, new)
        return _row_dict(n, written)

    def _fn_write_parameter(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        key = _text(a, "key")
        if not key:
            raise BackendError("Missing input: key.", 400)
        if "value" not in a or a["value"] in (None, ""):
            raise BackendError("Missing input: value.", 400)
        value = a["value"]
        if isinstance(value, str):
            text = value.strip()
            try:
                value = int(text, 16) if text.lower().startswith("0x") else (int(text) if text.lstrip("-").isdigit()
                                                                             else float(text))
            except ValueError:
                value = text  # a label such as "1:Forward"
        k: int | str = int(key) if key.isdigit() else key
        result = slot.ec.write_parameter(k, value)
        self._refresh_limits(slot)
        return result

    def _fn_software_reset(self, slot: AxisSlot, a: dict[str, Any]) -> Any:
        result = slot.ec.software_reset()
        self._refresh_limits(slot)
        return result

    # ------------------------------------------------------------------ tables

    def positions(self, axis: Any) -> dict[str, Any]:
        """Every stored row the unit uses (rows 0 .. max_positions - 1)."""
        slot = self._slot(axis)
        with self._locked(slot, "reading the position table"):
            try:
                count = slot.ec.max_positions()
                rows = []
                for n in range(count):
                    try:
                        rows.append(_row_dict(n, slot.ec.read_position_table(n)))
                    except (E.ModbusException, ValueError) as exc:
                        rows.append({"n": n, "error": describe(exc)})
            except Exception as exc:  # noqa: BLE001
                raise to_backend_error(exc) from None
        return {"axis": slot.name, "rows": rows, "limits": slot.limits}

    def parameters(self, axis: Any) -> dict[str, Any]:
        slot = self._slot(axis)
        with self._locked(slot, "reading parameters"):
            try:
                params = slot.ec.parameters()
            except Exception as exc:  # noqa: BLE001
                raise to_backend_error(exc) from None
        return {"axis": slot.name, "parameters": jsonable(params)}

    def overview(self, axis: Any) -> dict[str, Any]:
        """Unit info panel: identity, monitor, current alarm and history with remedies (READ-ONLY)."""
        slot = self._slot(axis)
        out: dict[str, Any] = {"axis": slot.name, "info": slot.info, "limits": slot.limits}
        with self._locked(slot, "reading unit info"):
            for key, fn in (("monitor", slot.ec.read_monitor),
                            ("history", lambda: slot.ec.read_alarm_history())):
                try:
                    out[key] = jsonable(fn())
                except Exception as exc:  # noqa: BLE001
                    out[key] = None
                    out[f"{key}_error"] = describe(exc)
        for row in out.get("history") or []:
            row["remedy"] = alarm_info(row["code"])["remedy"]
        code = (slot.status or {}).get("alarm_code") or 0
        out["alarm"] = alarm_info(code) if code else None
        with self._lock:
            pslot = self.ports.get(slot.port)
        out["port"] = None if pslot is None else pslot.summary([a for a in self.axes if a.startswith(slot.port + ":")])
        return out

    class _Held:
        def __init__(self, slot: AxisSlot, what: str, wb: Workbench) -> None:
            self.slot, self.what, self.wb = slot, what, wb

        def __enter__(self) -> None:
            self.wb._refuse_in_run([self.slot.name])
            if not self.slot.lock.acquire(timeout=10.0):
                raise BackendError(f"{self.slot.name} is busy with {self.slot.busy or 'another request'}.", 409)
            self.slot.busy = self.what

        def __exit__(self, *exc: object) -> None:
            self.slot.busy = None
            self.slot.lock.release()

    def _locked(self, slot: AxisSlot, what: str) -> Workbench._Held:
        return Workbench._Held(slot, what, self)

    # ------------------------------------------------------------------ port functions

    def port_call(self, port: Any, fn: Any, args: Any = None) -> dict[str, Any]:
        with self._lock:
            pslot = self.ports.get(str(port))
        if pslot is None:
            raise BackendError(f"{port} is not connected.", 404)
        args = args or {}
        if not isinstance(args, dict):
            raise BackendError("args must be an object.", 400)
        slots = [a for a in list(self.axes.values()) if a.port == pslot.name]
        if fn == "scan":
            if self.coord.active():
                raise BackendError(RUN_REFUSED, 409)
            try:
                hits = pslot.port.scan(parse_slaves(_text(args, "slaves")))
            except Exception as exc:  # noqa: BLE001
                raise to_backend_error(exc) from None
            new = [self._add_axis(pslot, h.slave) for h in hits if axis_name(pslot.name, h.slave) not in self.axes]
            self._rebuild_cell()
            if new:
                self.event("info", f"{pslot.name}: added {', '.join(new)}")
            return {"port": pslot.name, "hits": jsonable(hits), "added": new}
        if fn == "set_baudrate":
            rate = _int(args, "rate")
            self._refuse_in_run(s.name for s in slots)
            held = [s for s in slots if s.lock.acquire(timeout=10.0)]
            try:
                if len(held) != len(slots):
                    raise BackendError("an axis on this port is busy; try again.", 409)
                result = pslot.port.set_baudrate(rate, method=_text(args, "method", "direct"))
            except Exception as exc:  # noqa: BLE001
                self.event("error", f"{pslot.name}: baud change refused: {describe(exc)}")
                raise to_backend_error(exc) from None
            finally:
                for s in held:
                    s.lock.release()
            self.event("info", f"{pslot.name}: baud rate now {pslot.port.baudrate}")
            return {"port": pslot.name, "result": jsonable(result)}
        raise BackendError(f"Unknown port function {fn!r}.", 400)

    # ------------------------------------------------------------------ group

    def stop(self, axis: Any = None, broadcast: bool = False) -> dict[str, Any]:
        """STOP one axis, or every axis (STOP frames to all first, ports in parallel). Never waits for a lock.
        ``broadcast`` also sends one broadcast STOP on every daisy chain (never behind a gateway; UNVERIFIED)."""
        self.coord.note_stop()  # a coordinated run of the core that is still planning repeats this STOP
        if axis is not None:
            return self.call(axis, "stop")  # which notes it for this axis
        with self._lock:
            cell = self.cell
            slots = list(self.axes.values())
        self._note_stop(slots)  # before the first frame: a command still waiting for its axis's lock is cancelled
        # An axis is on the page (and can be commanded) as soon as it is added, but the cell is rebuilt only when the
        # whole connect or scan is done: those axes are stopped one by one, next to the cell's STOP.
        known = set(cell.names) if cell is not None else set()
        extra = [s for s in slots if s.name not in known]
        if cell is None and not extra:
            return {"stopped": []}
        results: dict[str, Any] = {}

        def stop_one(slot: AxisSlot) -> None:
            try:
                results[slot.name] = slot.ec.stop(wait=False, timeout=5.0)
            except BaseException as exc:  # noqa: BLE001 - reported with the others
                results[slot.name] = exc

        workers = [threading.Thread(target=stop_one, args=(s,), name="wb-stop-extra", daemon=True) for s in extra]
        for worker in workers:
            worker.start()
        try:
            if cell is not None:
                results.update(cell.stop_all(return_exceptions=True, broadcast=bool(broadcast)))
        finally:
            for worker in workers:
                worker.join(10.0)
        errors = {n: describe(r) for n, r in results.items() if isinstance(r, BaseException)}
        for slot in slots:
            if slot.home_started is not None:
                try:
                    slot.ec.finish_home()
                    slot.home_started = None
                except Exception:  # noqa: BLE001 - retried by the poll
                    pass
        self.event("warn" if errors else "info", "STOP all: " + ("; ".join(f"{n}: {e}" for n, e in errors.items())
                                                              if errors else f"{len(results)} axes stopped"))
        return {"stopped": [n for n in results if n not in errors], "errors": errors}

    def group(self, op: Any, body: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            cell = self.cell
        if op == "stop_all":
            return self.stop(broadcast=bool(body.get("broadcast")))
        if cell is None:
            raise BackendError("No axis connected.", 409)
        names = body.get("axes") or list(cell.names)
        if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
            raise BackendError("axes must be a list of axis names.", 400)
        if op == "move_all":
            # The checks below (a coordinated run, PIO mode, the axis locks) are made for ``names``: the targets must
            # name exactly those axes, or an axis could be moved without them.
            targets = body.get("targets")
            if not isinstance(targets, dict) or not targets:
                raise BackendError("targets must be {axis: position}.", 400)
            if body.get("axes") and set(names) != {str(k) for k in targets}:
                raise BackendError("axes and targets must name the same axes.", 400)
            names = [str(k) for k in targets]
        slots = [self._slot(n) for n in names]
        marks = {s.name: s.stop_epoch for s in slots}  # a STOP from here on cancels this while it waits for the locks
        self._refuse_in_run(names)
        if op == "status_all":
            return {"result": jsonable(cell.status_all(names, return_exceptions=True))}
        self._refuse_pio_hold(names, f"the group operation {op}")
        held: list[AxisSlot] = []
        try:
            for s in sorted(slots, key=lambda s: s.name):
                if not s.lock.acquire(timeout=10.0):
                    raise BackendError(f"{s.name} is busy; try again.", 409)
                held.append(s)
                s.busy = op
            if any(s.stop_epoch != marks[s.name] for s in slots):  # STOP came while this waited for the locks
                raise BackendError(STOP_CANCELLED.format(what=f"The group operation {op}"), 409)
            if op == "move_all":
                targets = body.get("targets")
                if not isinstance(targets, dict) or not targets:
                    raise BackendError("targets must be {axis: position}.", 400)
                pos = {str(k): _num(targets, k) for k in targets}
                for k in pos:
                    self._slot(k)
                speed = {str(k): _num(body.get("speed") or {}, k) for k in pos}
                accel = {str(k): _num(body.get("accel") or {}, k, 0.3) for k in pos}
                cell.move_all(pos, speed_mm_s=speed, accel_g=accel, wait=False)
                self._stop_again([s for s in slots if s.stop_epoch != marks[s.name]], "the group move")
                self.event("info", "Group move started: " + ", ".join(f"{k} -> {v:g}" for k, v in pos.items()))
                return {"started": list(pos)}
            if op == "servo_all":
                on = _bool(body, "on", True)
                results = cell.servo_all(on, names, return_exceptions=True)
            elif op == "home_all":
                def home(slot: AxisSlot) -> Any:
                    slot.home_started, slot.home_seen = time.monotonic(), False
                    try:
                        return slot.ec.home(wait=False)
                    except BaseException:
                        slot.home_started = None
                        slot.ec.finish_home()
                        raise
                with ThreadPoolExecutor(max_workers=max(1, len(slots))) as pool:
                    futures = {s.name: pool.submit(home, s) for s in slots}
                results = {}
                for n, f in futures.items():
                    exc = f.exception()
                    results[n] = exc if exc is not None else f.result()
                self._stop_again([s for s in slots if s.stop_epoch != marks[s.name]], "the group home")
            else:
                raise BackendError(f"Unknown group operation {op!r}.", 400)
        except Exception as exc:  # noqa: BLE001
            err = to_backend_error(exc)
            self.event("error", f"Group {op} refused: {err}")
            raise err from None
        finally:
            for s in held:
                s.busy = None
                s.lock.release()
        errors = {n: describe(r) for n, r in results.items() if isinstance(r, BaseException)}
        self.event("warn" if errors else "info", f"Group {op}: " + (
            "; ".join(f"{n}: {e}" for n, e in errors.items()) if errors else f"{len(results)} axes ok"))
        return {"ok_axes": [n for n in results if n not in errors], "errors": errors}


# ====================================================================== helpers


def _info_summary(info: Any) -> dict[str, Any]:
    keys = ("model", "controller", "actuator", "kind", "units", "product_family", "family", "firmware",
            "serial_number", "system_name", "unit_name", "model_name", "actuator_kind_name", "max_positions",
            "identity_source", "unit_known", "soft_limits_mm", "max_speed_mm_s", "max_accel_g", "max_decel_g",
            "controller_name", "actuator_serial", "user_name")
    data = info.to_dict() if hasattr(info, "to_dict") else {}
    return {k: jsonable(data.get(k)) for k in keys}


def _row_dict(n: int, entry: Any) -> dict[str, Any]:
    flags = entry.control_flags
    push = bool(flags & R.CTLF_PUSH)
    row = {
        "n": n, "position_mm": entry.position_mm, "speed_mm_s": entry.speed_mm_s, "accel_g": entry.accel_g,
        "decel_g": entry.decel_g, "band_mm": entry.band_mm, "zone_plus_mm": entry.zone_plus_mm,
        "zone_minus_mm": entry.zone_minus_mm, "push_power": entry.push_power,
        "push_pct": round(R.push_pct(entry.push_power), 1), "control_flags": f"0x{flags:04X}",
        "mode": ("push+" if flags & R.CTLF_PUSH_AWAY else "push-") if push else "normal",
        "incremental": bool(flags & R.CTLF_INC), "checksum_ok": entry.checksum_ok,
    }
    if getattr(entry, "skipped", False):  # a write / teach that would not change the row wrote nothing
        row.update(skipped=True, note=NV_SKIP_NOTE)
    return row
