"""Command line: ``python -m iai_actuator_driver [--port COM3 | --sim] [--slave N | --axis N] <command>`` and ``iai-check``.

Commands: status, identify, info, limits, monitor [--watch], alarms, servo on|off, home, move POS [--decel G] [--fast],
jog DELTA [--fast], push POS BAND PCT, goto N [--speed-pct P], stop, pause, resume, cancel-pause, teach N,
backup [FILE], line [--monitor], recover, alarm-reset, swreset, baud RATE, param list, param get KEY,
param set KEY VALUE, check [--motion].
``--slave N`` (default 1) or ``--axis N`` (= slave N + 1) picks one axis on a port that carries several
(RCON gateway / SIO daisy chain; UNVERIFIED on hardware).

More commands, one line each (ranges are the unit's own: ``limits`` prints them):

* ``move 100 --speed 20 --change-speed 60 --change-after 1.5``: change the speed of the running move after 1.5 s
  (0.01 .. the unit's MAXV; ``--change-accel G`` likewise, 0.01 .. MAXA; plain 0x9900 moves only; VERIFIED on the
  EC-GRBP8M gripper, the speed change also on the PCON-CB)
* ``pause`` / ``resume``: hold a move and keep its travel / continue it (the pause stays set after this command ends)
* ``cancel-pause``: drop the remaining travel of a paused move and release the pause (VERIFIED on the EC-GRBP8M gripper)
* ``teach 2``: store the current position as stored row 2 (0 .. rows - 1; needs a homed axis at rest;
  non-volatile; ``--method coil`` is EXPERIMENTAL; a row that would not change is not written, ``--force``
  writes it anyway)
* ``backup [FILE] [--rows used|all|0,1,2]``: read-only JSON backup of one axis (FC03 / FC11 only)
* ``line [--monitor]``: the line (direct / rcon / robonet), its rules, the gateway status and, behind an RCON
  gateway, this axis's gateway monitor record (read-only)
* ``status`` also prints the condition flags (EMG input, drive power cut, enable switch released, cold-start alarm,
  automatic servo OFF, ...); ``servo on`` is refused with those reasons when the status shows why it cannot come on
* ``stop-all [--ports COM3,COM4] [--broadcast]``: STOP every axis on the ports; ``--broadcast`` also sends one
  broadcast STOP on a daisy chain (never behind a gateway; UNVERIFIED on hardware)
* ``--line auto|direct|rcon|robonet`` states the line (default: probed once, read-only); ``--position-unit 0.01|0.001``
  is an RCON unit's parameter No.207, needed before any mm-based motion there; the default ``--retries`` is 3

Fast path and power / link loss, one line each:
* ``move 50 --fast`` / ``jog 5 --fast``: the move is done on the first in-position sample (``completion="fast"``) and
  the status monitor feeds the wait; the baud rate is kept (``baud 230400`` raises it)
* ``monitor --watch [--duration S]``: run the status monitor (read-only): one line per status change (at most one per
  ``--interval``) and every link event at once (link lost, unit restarted); ends at Ctrl+C or after ``S`` seconds
* ``recover [--backup FILE [--rows used]] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]``: bring the
  axis back after a power or link loss: the same unit (the backup's identity), PC control, baud rate, message-level
  alarms, and a report of HEND, alarm and position. Never moves the axis (VERIFIED on the EC-GRBP8M gripper after a
  power cycle and a line break, and on the PCON-CB after a line break). With ``--backup`` every row the file holds is
  compared; ``--rows used`` compares only the rows ``backup --rows used`` holds (this command line only)

Any IAI unit is driven the same way (ELECYLINDER or a CON-family controller such as a PCON): positions are
in mm, or in deg on a rotary axis (AKID 7.33 = 1), and every range (stroke, speed, acceleration, push,
stored rows) is read from the unit itself; ``limits`` prints them. ``--max-speed`` / ``--max-accel`` /
``--max-push`` only narrow them further.

Port-level commands (no ``--port`` needed, READ-ONLY): ``iai-actuator-driver ports`` lists the serial ports (IAI
USB adapters first); ``iai-actuator-driver scan [--ports COM3,COM4] [--slaves 1-16]`` asks every address of every
port (default: every IAI adapter) and lists the axes that answer.
Exit codes: 0 ok, 1 check failed / nothing found by scan, 2 error, 130 interrupted.

``alarm-reset`` is the alarm reset (``reset`` is kept as a deprecated alias of it, with a warning, so
that existing scripts never trigger a software reset by surprise); ``swreset`` is the software reset
(servo OFF, then 0x45 cmd 5 op1 1, the frame IA-OS sends: an incremental unit loses its home). ``param set``
commits the value to non-volatile memory (PARW coil, as IA-OS's Transfer does) unless ``--no-commit``; it
applies after ``swreset`` or a power cycle. A value the unit already holds is neither written nor committed (the
non-volatile memory lasts about 100,000 write cycles, IAI); ``--force`` commits it anyway. Each run is a new
process: a value an earlier ``--no-commit`` run left in RAM looks unchanged to the next ``param set``, so commit it
with ``param set KEY VALUE --force``. ``KEY`` is an EC P-number (``5`` / ``P5``), a CON parameter number (``35`` /
``No.35``) or a raw item ``class.item``; every unit's parameters are written the same way. A dangerous write (home
direction / offset, motor / encoder, lead / axis type, gains, PIO / baud configuration, widening a soft limit, raw
items of unknown meaning) or a parameter the unit's list does not show is written with a warning. ``baud`` is
volatile: the EC is back at 38400 after a power-up; 115200 and 230400 have codes verified on hardware, 38400 is sent
and checked (the EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod ignores it), another code is sent with a
warning. ``--allow-nonvolatile``, ``--allow-dangerous``, ``--allow-home-params``, ``--allow-unlisted`` and
``--allow-unverified-code`` are still accepted and ignored (no longer needed).

Without ``--baud`` the baud rate is auto-detected on the open port, in the order 115200, 38400 (the EC's
power-up rate), 9600, 230400, 57600, 19200; the port is never closed and reopened in between.
Commands that change the EC's state (servo, home, move, jog, push, goto, alarm-reset, swreset, baud,
param set) first read identity, soft limits and speed limit and take Modbus control while the axis is
at rest; read-only commands (status, identify, info, alarms, param list, check without --motion)
write nothing. STOP is sent with or without Modbus control: with PMSS = 0 it stopped a Modbus-commanded
move on hardware, but it is untested on a move a PLC commanded over PIO (stop those from the PLC or with
the hardware stop).
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import datetime as _dt
import json
import math
import queue
import signal
import sys
import tempfile
import threading
import time
from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any, TextIO

from . import __version__
from ._backend import backend_info
from .cell import DEFAULT_SLAVES, Cell, Port, ScanHit, list_ports
from .check import CheckItem, CheckReport, run_check
from .ec import (
    STRATEGIES,
    AlarmRecord,
    BaudChangeResult,
    DeviceInfo,
    Elecylinder,
    ParameterWriteResult,
    PushResult,
    SoftwareResetResult,
    Status,
)
from .ec import FALLBACK_ACCEL_CAP_G, FALLBACK_SPEED_CAP, NV_SKIP_NOTE
from .errors import NO_ANSWER_HINT, EcDriverError, ModbusError
from .gateway import RconGateway
from .info import SECTIONS, backup
from .modbus import BAUD_DETECT_ORDER, IAI_BAUD_CODES, RECOMMENDED_RETRIES, SCAN_TIMEOUT_S, VERIFIED_BAUD_RATES
from .registers import POSITION_SIMPLE_DATA_ROWS, is_ec_family
from .sim import PROFILES, SimulatedEC
from .sim_line import SimLine, sim_rcon_line, sim_robonet_line
from .values import _at_rest_for_control

BANNER = "IAI Actuator Driver (UNOFFICIAL, not an IAI product)"
DEFAULT_MAX_PUSH_PCT = 70.0  # the CLI's conservative push cap without --max-push / --unit-max (the GUI uses the same)
STATE_CHANGING = ("servo", "home", "move", "jog", "push", "goto", "reset", "alarm-reset", "swreset", "baud", "mode",
                  "pause", "resume", "cancel-pause", "teach")
DEPRECATED_RESET_NOTE = (
    "note: 'reset' is deprecated and means ALARM reset; use 'alarm-reset'. "
    "The software reset is 'swreset'."
)
_LOG_FRAMES_HELP = (
    "print every Modbus frame to stderr, with the time it was printed: on the native core a request and its reply "
    "are printed together once the request has ended, so the time between them is not the reply time"
)


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial port, e.g. COM3 (close IA-OS first)")
    where.add_argument("--sim", action="store_true", help="use the built-in simulator instead of hardware")
    address = parser.add_mutually_exclusive_group()
    address.add_argument("--slave", type=int, default=1, help="Modbus address = axis number + 1 (default 1)")
    address.add_argument(
        "--axis", type=int, default=None,
        help="axis number on a shared port (RCON / daisy chain): same as --slave AXIS+1",
    )  # fmt: skip
    parser.add_argument(
        "--baud", type=int, default=None,
        help="fixed baud rate (default: auto-detect, trying " + ", ".join(map(str, BAUD_DETECT_ORDER)) + ")",
    )  # fmt: skip
    parser.add_argument("--timeout", type=float, default=0.2, help="response timeout in s (default 0.2)")
    parser.add_argument(
        "--retries", type=int, default=RECOMMENDED_RETRIES,
        help=f"retries on timeout/CRC error (default {RECOMMENDED_RETRIES}, the manufacturer's recommendation)",
    )  # fmt: skip
    parser.add_argument(
        "--strategy", choices=sorted(STRATEGIES), default=None,
        help="servo/home/reset command path (default: coil, or register behind an RCON gateway; register = FC06 DRG1 "
        "without coils; iai and iai2 are EXPERIMENTAL)",
    )  # fmt: skip
    parser.add_argument(
        "--line", choices=("auto", "direct", "rcon", "robonet"), default="auto",
        help="the line the unit sits on (default auto: probed once, read-only; robonet = SIO through mode, which "
        "cannot be probed). Behind a gateway the line's rules apply (no coils on RCON, no broadcast ...)",
    )  # fmt: skip
    parser.add_argument(
        "--position-unit", type=float, choices=(0.01, 0.001), default=None, metavar="MM",
        help="RCON only: the unit's position unit, parameter No.207 (0.01 or 0.001 mm), which the line cannot read; "
        "needed before any mm-based motion command there (a direct line uses the unit's own 0.01: 0.001 is refused)",
    )  # fmt: skip
    parser.add_argument(
        "--stroke", type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
        help="soft limits in mm (default: the EC's own soft limits; always narrowed to them)",
    )  # fmt: skip
    parser.add_argument(
        "--max-speed", type=float, default=None,
        help=f"speed cap in mm/s (deg/s on a rotary axis); default {FALLBACK_SPEED_CAP:g}, always lowered to the "
        "unit's own maximum MAXV, item 7.4 (--unit-max: the unit's maximum)",
    )  # fmt: skip
    parser.add_argument(
        "--max-accel", type=float, default=None,
        help=f"acceleration cap in G; default {FALLBACK_ACCEL_CAP_G:g}, always lowered to the unit's own MAXA / MAXD, "
        "items 7.5 / 7.6 (--unit-max: the unit's maximum)",
    )  # fmt: skip
    parser.add_argument(
        "--unit-max", action="store_true",
        help="lift the conservative default caps: speed, acceleration and push force up to the unit's own maxima "
        "(MAXV 7.4, MAXA / MAXD 7.5 / 7.6, MXPP 7.9); an explicit --max-speed / --max-accel / --max-push still applies",
    )  # fmt: skip
    parser.add_argument("--log-frames", action="store_true", help=_LOG_FRAMES_HELP)
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="slider",
        help="simulated unit with --sim: " + ", ".join(sorted(PROFILES)) + " (default slider)",
    )  # fmt: skip
    parser.add_argument("--sim-time-scale", type=float, default=1.0, help=argparse.SUPPRESS)
    parser.add_argument("--sim-baud", type=int, default=None, help=argparse.SUPPRESS)  # model the line's baud rate
    parser.add_argument("--sim-unit", type=lambda v: int(v, 0), default=None, help=argparse.SUPPRESS)  # FC11 unit code
    parser.add_argument("--sim-line", choices=("rcon", "robonet"), default=None, help=argparse.SUPPRESS)  # gateway sim
    parser.add_argument(
        "--max-push", type=float, default=None,
        help=f"push force cap in %%; default {DEFAULT_MAX_PUSH_PCT:g}, always lowered to the unit's own MXPP, item 7.9 "
        "(--unit-max: the unit's maximum)",
    )  # fmt: skip


def _add_check_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--motion", action="store_true", help="also do a short move (up to 5 mm / deg, at most a quarter of the stroke) at 10 units/s and back")
    parser.add_argument("--report", help="JSON report path (default iai-check-<timestamp>.json)")


def version_text() -> str:
    """``--version``: the driver version and the backend it runs on (always ``native``), the core
    library's version and its C ABI version."""
    info = backend_info()
    text = f"{__version__} (backend {info['backend']}"
    if info["backend"] == "native":
        abi = info.get("abi")
        if info.get("core_version"):
            text += f", core {info['core_version']}"
        if abi:
            text += f", ABI {'.'.join(map(str, abi)) if isinstance(abi, (tuple, list)) else abi}"
    return "%(prog)s " + text.replace("%", "%%") + ")"  # argparse formats it: only %(prog)s is a field


def build_parser() -> argparse.ArgumentParser:
    """Parser for ``python -m iai_actuator_driver``."""
    parser = argparse.ArgumentParser(prog="iai-actuator-driver", description=BANNER)
    parser.add_argument("--version", action="version", version=version_text())
    _add_connection_args(parser)
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="read position and status bits")
    sub.add_parser("identify", help="read slave id, index table, model / serial and soft limits")
    info = sub.add_parser("info", help="everything the EC tells about itself (read-only: FC03 + FC11)")
    info.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="machine-readable output")
    info.add_argument(
        "--section", action="append", choices=SECTIONS, default=None, help="only this part (repeatable; default: all)"
    )
    sub.add_parser("limits", help="every range the driver enforces, read from the unit itself (read-only)")
    monitor = sub.add_parser("monitor", help="position, velocity, current, deviation, alarm, uptime (read-only)")
    monitor.add_argument("--count", type=int, default=1, help="number of reads (default 1)")
    monitor.add_argument("--interval", type=float, default=0.5, help="seconds between reads (default 0.5)")
    monitor.add_argument(
        "--watch", action="store_true",
        help="run the status monitor: a line per status change (at most one per --interval) and every link event "
        "(link lost, unit restarted) until Ctrl+C or --duration; read-only",
    )  # fmt: skip
    monitor.add_argument("--duration", type=float, default=None, metavar="S",
                         help="--watch: stop after S seconds (default: until Ctrl+C)")  # fmt: skip
    sub.add_parser("alarms", help="read the alarm history")
    servo = sub.add_parser("servo", help="servo on/off (on also enables Modbus control if needed)")
    servo.add_argument("state", choices=("on", "off"))
    home = sub.add_parser("home", help="home return and wait")
    home.add_argument(
        "--method", choices=("coil", "iai"), default=None,
        help="coil: HOME coil (default strategy); iai: 0x45 cmd 0x0A with op1 = the EC's home pattern ORG",
    )  # fmt: skip
    move = sub.add_parser("move", help="absolute move to POS (mm, or deg on a rotary axis) and wait")
    move.add_argument("mm", type=float)
    move.add_argument("--speed", type=float, default=20.0, help="mm/s or deg/s (default 20)")
    move.add_argument("--accel", type=float, default=0.1, help="G (default 0.1)")
    move.add_argument(
        "--decel", type=float, default=None,
        help="deceleration in G when it differs from --accel (command-profile move, VERIFIED on the EC gripper and "
        "the PCON-CB; default: same as --accel)",
    )  # fmt: skip
    move.add_argument("--band", type=float, default=0.1, help="positioning band in mm / deg (default 0.1)")
    move.add_argument("--no-wait", action="store_true", help="return right after sending the command")
    move.add_argument(
        "--fast", action="store_true",
        help="fast completion: done on the first in-position sample, with the status monitor feeding the wait; the "
        "baud rate is kept (raise it with 'baud 230400')",
    )  # fmt: skip
    move.add_argument(
        "--change-speed", type=float, default=None, metavar="SPEED",
        help="after --change-after seconds change the running move's speed to SPEED (0.01 .. the unit's maximum; "
        "plain moves only; VERIFIED on the EC-GRBP8M gripper and the PCON-CB)",
    )  # fmt: skip
    move.add_argument(
        "--change-accel", type=float, default=None, metavar="G",
        help="after --change-after seconds change the running move's acceleration to G (0.01 .. MAXA; VERIFIED on the "
        "EC-GRBP8M gripper)",
    )  # fmt: skip
    move.add_argument("--change-after", type=float, default=1.0, metavar="S",
                      help="seconds after the start at which --change-speed / --change-accel apply (default 1)")  # fmt: skip
    jog = sub.add_parser("jog", help="relative move by DELTA mm (never uses the EC's JOG coils)")
    jog.add_argument("delta", type=float)
    jog.add_argument("--speed", type=float, default=10.0, help="mm/s (default 10)")
    jog.add_argument(
        "--fast", action="store_true",
        help="fast completion: done on the first in-position sample, with the status monitor feeding the wait; the "
        "baud rate is kept (raise it with 'baud 230400')",
    )  # fmt: skip
    push = sub.add_parser("push", help="approach MM, then push over BAND mm at PCT %% force")
    push.add_argument("mm", type=float)
    push.add_argument("band", type=float)
    push.add_argument(
        "pct", type=float,
        help="push force %% (capped by --max-push and the EC's MXPP, item 7.9; below MNPP / 20 %% only warned)",
    )  # fmt: skip
    push.add_argument("--toward-home", action="store_true", help="push toward home (default: away from home)")
    push.add_argument("--speed", type=float, default=10.0, help="approach speed mm/s (default 10)")
    goto = sub.add_parser("goto", help="run stored position N at its stored speed (or --speed-pct of it), and wait")
    goto.add_argument(
        "n", type=int,
        help="stored row 0 .. the unit's row count - 1 (see 'limits'); on an EC 0 backward end, 1 forward end, 2 middle",
    )  # fmt: skip
    goto.add_argument(
        "--speed-pct", type=float, default=None,
        help="run the row at 1-100 %% of its stored speed (0x45 cmd 6, VERIFIED on the EC gripper and the PCON-CB; "
        "in steps of 1/256, at least 0.78 %%; nothing written to the unit)",
    )  # fmt: skip
    goto.add_argument(
        "--allow-fast", action="store_true",
        help="run the row at its own stored speed / accel even above --max-speed / --max-accel (unit limit still applies)",
    )  # fmt: skip
    goto.add_argument("--no-wait", action="store_true", help="return once the motion has started")
    sub.add_parser("stop", help="decel stop (STOP coil on, wait, off; behind RCON the register path)")
    sub.add_parser("pause", help="hold the move and keep its remaining travel (stays set after this command ends)")
    sub.add_parser("resume", help="release the pause: the paused move continues")
    sub.add_parser(
        "cancel-pause",
        help="drop the remaining travel of a paused move and release the pause (VERIFIED on the EC-GRBP8M gripper)",
    )
    teach = sub.add_parser(
        "teach",
        help="store the current position as stored row N (non-volatile; homed axis at rest; the row's own stored "
        "speed / accel must be within the caps: --unit-max lifts the conservative defaults)",
    )
    teach.add_argument("n", type=int, help="stored row 0 .. the unit's row count - 1 (see 'limits')")
    teach.add_argument("--allow-nonvolatile", action="store_true", help=argparse.SUPPRESS)  # no-op (no longer needed)
    teach.add_argument("--force", action="store_true",
                       help="write the row even when it would not change (by default nothing is written then)")
    teach.add_argument(
        "--method", choices=("table", "coil"), default="table",
        help="table: read the position, write the row (default); coil: MOD / POSR / TEAC coil sequence (EXPERIMENTAL)",
    )  # fmt: skip
    bk = sub.add_parser("backup", help="read-only JSON backup of this unit: identity, directory, parameters, rows, alarms")
    bk.add_argument("file", nargs="?", default=None, help="output file (default ec-backup-<port>-s<slave>-<time>.json)")
    bk.add_argument(
        "--rows", default="used",
        help="stored rows to back up: used (default), all (an EC has 768: slow) or numbers such as 0,1,2",
    )  # fmt: skip
    rec = sub.add_parser(
        "recover",
        help="bring the axis back after a power or link loss: same unit, PC control, baud rate, message alarms, and a "
        "report of home, alarm and position (never moves; VERIFIED on the EC-GRBP8M gripper after a power cycle and a "
        "line break, and on the PCON-CB after a line break)",
    )  # fmt: skip
    rec.add_argument(
        "--backup", default=None, metavar="FILE",
        help="a 'backup' JSON of this unit: the unit must be that one (serial number, model); its stored rows and "
        "parameters are compared with the file",
    )  # fmt: skip
    rec.add_argument(
        "--rows", choices=("all", "used"), default=None,
        help="with --backup, the stored rows to compare: all (default) every row the file holds (768 on an EC for a "
        "'backup --rows all' file: about 25 s at 38400 baud, VERIFIED on the EC-GRBP8M gripper), or used only the rows "
        "'backup --rows used' holds (the first max_positions rows and an EC's rows 256-267): a change in any other row "
        "is then not noticed",
    )  # fmt: skip
    rec.add_argument("--target", type=float, default=None, metavar="POS",
                     help="the last commanded position: the report says how far the axis is from it")  # fmt: skip
    rec.add_argument(
        "--restore-baud", type=int, default=None, choices=sorted(VERIFIED_BAUD_RATES),
        help="switch the unit back to this rate (volatile; a broadcast: every unit of a shared line follows)",
    )  # fmt: skip
    rec.add_argument("--no-control", action="store_true", help="do not take PC control (PMSL) again")
    rec.add_argument("--wait", type=float, default=30.0, metavar="S",
                     help="seconds to wait for the unit to answer again (default 30)")  # fmt: skip
    line = sub.add_parser("line", help="the line (direct / rcon / robonet), its rules and the gateway status (read-only)")
    line.add_argument("--monitor", action="store_true", help="RCON: also this axis's gateway monitor record")
    areset = sub.add_parser("alarm-reset", help="alarm reset (ALRS pulse, or --method iai); not a software reset")
    areset.add_argument(
        "--method", choices=("coil", "iai"), default=None,
        help="coil: ALRS coil pulse (default strategy); iai: 0x45 cmd 5 op1 0",
    )  # fmt: skip
    mode = sub.add_parser("mode", help="action mode: pc (Modbus control, the PMSL coil) or pio (0x45 cmd 0x0C)")
    mode.add_argument("which", choices=("pc", "pio"), help="pc = Modbus control (PMSS 1), pio = PIO control (PMSS 0)")
    mode.add_argument("--safety", action="store_true", help="safety speed on (~100 mm/s cap)")
    dep = sub.add_parser("reset", help="DEPRECATED alias of alarm-reset (use alarm-reset, or swreset for a software reset)")
    dep.add_argument("--method", choices=("coil", "iai"), default=None, help=argparse.SUPPRESS)
    sub.add_parser(
        "swreset",
        help="software reset (servo OFF, then 0x45 cmd 5): applies committed parameters; incremental units lose home",
    )
    baud = sub.add_parser("baud", help="change the baud rate (broadcast 0x45 cmd 4); volatile: 38400 after power-up")
    baud.add_argument(
        "rate", type=int, choices=sorted(IAI_BAUD_CODES),
        help="new rate; verified codes: " + ", ".join(map(str, VERIFIED_BAUD_RATES))
        + "; 38400 is sent and checked (some units ignore its code); another code is sent with a warning "
        "(14400 / 28800 / 76800 are never auto-detected: reconnect with --baud RATE or power-cycle back to 38400)",
    )  # fmt: skip
    baud.add_argument("--allow-unverified-code", action="store_true", help=argparse.SUPPRESS)  # no-op (not needed)
    baud.add_argument(
        "--method", choices=("direct", "break"), default="direct",
        help="direct: broadcast at the current rate (default); break: 200 ms line break, 9600, broadcast",
    )  # fmt: skip
    param = sub.add_parser("param", help="parameters: list / get (read-only) or set (commits to non-volatile memory)")
    psub = param.add_subparsers(dest="param_cmd", required=True)
    psub.add_parser("list", help="the parameters of this unit, decoded (EC: P-numbers; CON: No.-numbers; read-only)")
    pget = psub.add_parser("get", help="read one parameter or raw item (read-only)")
    pget.add_argument("key", help="5 / P5 (EC), 35 / No.35 (CON) or a raw item class.item[entry], e.g. 8.15")
    pset = psub.add_parser(
        "set", help="write KEY = VALUE, verify, commit (PARW); applies after swreset / power cycle; a home, dangerous "
        "or unlisted parameter is written with a warning",
    )  # fmt: skip
    pset.add_argument("key", help="5 / P5 (EC), 35 / No.35 (CON) or a raw item class.item[entry]")
    pset.add_argument("value", help="the value as the PC tool shows it (mm, number or label such as 1:Forward); "
                      "a raw item takes the raw integer")  # fmt: skip
    pset.add_argument("--no-commit", action="store_true",
                      help="RAM only: lost at the next power-up (to save it later: param set KEY VALUE --force)")
    pset.add_argument("--force", action="store_true",
                      help="write and commit even when the unit already holds the value (by default nothing is written)")
    for name in ("--allow-dangerous", "--allow-home-params", "--allow-unlisted"):  # no-ops (no longer needed)
        pset.add_argument(name, action="store_true", help=argparse.SUPPRESS)
    pset.add_argument("--allow-unverified-model", action="store_true", help=argparse.SUPPRESS)  # no-op (compatibility)
    check = sub.add_parser("check", help="compatibility self-test (read-only unless --motion)")
    _add_check_args(check)
    return parser


class _FrameLines:
    """--log-frames: one line per frame on stderr, through a queue and one writer thread. The frame sink runs on the
    driver's threads (a status monitor, a STOP); a console write can block (a selection in a Windows console holds
    every write until it ends), so the sink only queues the line; a line that does not fit is dropped and counted.
    The time is when the sink was called: the core passes a request and its reply on together once the bus is let go
    (never with it held), so it is not the time the frame was on the wire."""

    LIMIT = 10000

    def __init__(self) -> None:
        self._queue: queue.Queue[str | None] = queue.Queue(maxsize=self.LIMIT)
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, name="cli-log-frames", daemon=True)
        self._thread.start()

    def __call__(self, direction: str, frame: bytes) -> None:
        try:
            self._queue.put_nowait(f"{time.perf_counter():.4f} {direction:<10} {frame.hex(' ')}")
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while (line := self._queue.get()) is not None:
            try:
                if self.dropped:
                    count, self.dropped = self.dropped, 0
                    print(f"{count} frame line(s) dropped while the console was blocked", file=sys.stderr)
                print(line, file=sys.stderr)
            except Exception:  # noqa: BLE001 - a console that fails must not end the writer
                pass

    def close(self, timeout: float = 5.0) -> None:
        """Write what is queued (waiting at most ``timeout`` s for the console), then end the writer."""
        try:
            self._queue.put(None, timeout=timeout)
        except queue.Full:
            return
        self._thread.join(timeout)


_frame_lines: _FrameLines | None = None


def _frame_sink() -> _FrameLines:
    """The --log-frames sink of this run (written out by :func:`_end_frame_lines` before the command returns)."""
    global _frame_lines
    if _frame_lines is None:
        _frame_lines = _FrameLines()
    return _frame_lines


def _end_frame_lines() -> None:
    global _frame_lines
    lines, _frame_lines = _frame_lines, None
    if lines is not None:
        lines.close()


def _open(args: argparse.Namespace) -> tuple[Elecylinder, str]:
    log_frame = _frame_sink() if args.log_frames else None
    port: Any
    gateway_sim = getattr(args, "sim_line", None)
    if args.sim and gateway_sim:
        make = sim_rcon_line if gateway_sim == "rcon" else sim_robonet_line
        port = make([args.slave], profile=args.sim_profile, time_scale=args.sim_time_scale)
    elif args.sim:
        port = SimulatedEC(
            slave=args.slave, time_scale=args.sim_time_scale, profile=args.sim_profile, baudrate=args.sim_baud,
            unit_code=getattr(args, "sim_unit", None),
        )
    else:
        port = args.port
    options: dict[str, Any] = {} if args.strategy is None else {"strategy": args.strategy}  # None: the line's own
    ec = Elecylinder.open(
        port,
        baudrate=args.baud,  # None: auto-detect in place on the open port
        timeout=args.timeout,
        retries=args.retries,
        frame_logger=log_frame,
        line=getattr(args, "line", "auto"),
        position_unit_mm=getattr(args, "position_unit", None),
        slave=args.slave,
        stroke=tuple(args.stroke) if args.stroke is not None else None,
        **_caps(args),
        **options,
    )
    line = ec.bus.line
    if line.is_gateway:
        print(f"note: line {line.kind} ({line.rules.title}); {line.rules.evidence}", file=sys.stderr)
    if args.sim:
        return ec, "sim"
    how = "auto-detected" if ec.baud_detected else "fixed by --baud"
    print(f"note: {args.port} at {ec.baudrate} baud ({how})", file=sys.stderr)
    return ec, args.port


def _caps(args: argparse.Namespace) -> dict[str, float | None]:
    """The configured caps: the explicit options; else the conservative defaults (100 units/s, 0.3 G, 70 %) unless
    --unit-max. The driver always lowers each to the unit's own maximum (MAXV / MAXA / MAXD / MXPP)."""
    unit_max = getattr(args, "unit_max", False)
    speed, accel, push = args.max_speed, args.max_accel, getattr(args, "max_push", None)
    return {
        "max_speed_mm_s": speed if speed is not None or unit_max else FALLBACK_SPEED_CAP,
        "max_accel_g": accel if accel is not None or unit_max else FALLBACK_ACCEL_CAP_G,
        "max_push_pct": push if push is not None or unit_max else DEFAULT_MAX_PUSH_PCT,
    }


def _print_status(st: Status, out: TextIO, units: str = "mm") -> None:
    flags = [
        name
        for name, on in (
            ("SERVO", st.servo_on), ("HOMED", st.homed), ("IN-POS", st.in_position), ("MOVING", st.moving),
            ("PAUSED", st.paused), ("HOMING", st.homing), ("PUSHING", st.pushing), ("PUSH-MISSED", st.push_missed),
            ("MODBUS", st.modbus_enabled), ("SAFETY-SPEED", st.safety_speed), ("ALARM", st.alarm),
            ("WARNING", st.warning), ("EMG", st.emergency_stop),
            # condition flags (None = not read / not defined on this line; DOC ME0162-12A p.4-18 to 4-25)
            ("EMG-INPUT", st.emg_input), ("DRIVE-POWER-CUT", st.drive_power_cut), ("ENABLE-SWITCH-OFF", st.enable_off),
            ("COLD-START-ALARM", st.cold_start_alarm), ("AUTO-SERVO-OFF", st.auto_servo_off),
            ("NV-WRITE-BUSY", st.nv_busy), ("AUTO-MODE", st.manual_mode is False), ("BATTERY-LOW", st.battery_low),
            ("RTC-IN-USE", st.rtc_in_use), ("EXCITATION-RUNNING", st.servo_on and not st.excitation_done),
        )
        if on
    ]  # fmt: skip
    markers = [
        name
        for name, on in (
            ("BACKWARD-END", st.at_backward_end), ("MIDDLE", st.at_middle), ("FORWARD-END", st.at_forward_end),
        )
        if on
    ]  # fmt: skip
    print(f"position   {st.position_mm:9.2f} {units}", file=out)
    if st.velocity_mm_s is not None:
        print(f"velocity   {st.velocity_mm_s:9.2f} {units}/s", file=out)
    if st.current_ma is not None:
        print(f"current    {st.current_ma:9d}", file=out)
    print(f"alarm code {st.alarm_code:#9x}", file=out)
    print(f"flags      {' '.join(flags) or '-'}", file=out)
    if st.position_complete_no is not None:
        print(f"row done   {st.position_complete_no} (POSS: the stored row last completed; 0 after a numeric move)", file=out)
    if st.line:
        print(f"line       {st.line}", file=out)
    marker_text = "not read (ZONS)" if st.at_backward_end is None else (" ".join(markers) or "-")
    print(f"markers    {marker_text}", file=out)
    print("raw        " + " ".join(f"{k}={v:#06x}" for k, v in st.raw.items()), file=out)


def _print_info(info: DeviceInfo, ec: Elecylinder, out: TextIO) -> None:
    print(f"model      {info.model}", file=out)
    print(f"controller {info.controller}", file=out)
    print(f"family     {info.product_family or '-'}"
          + (f" ({info.controller_name})" if info.controller_name else ""), file=out)  # fmt: skip
    if info.identity_source and info.identity_source != "FC11":
        print(f"identity   from {info.identity_source} (FC11 not answered)", file=out)
    print(f"actuator   {info.actuator or '-'}", file=out)
    kind = info.kind if info.kind.startswith("kind ") else f"kind {info.kind}"
    print(f"units      {info.units or 'mm'} ({kind})", file=out)
    if info.max_positions:
        print(f"rows       {info.max_positions} stored positions (0..{info.max_positions - 1})", file=out)
    print(f"model name {info.model_name or '-'}", file=out)
    print(f"unit name  {info.user_name or '-'} (0x061A, user-defined)", file=out)
    print(f"serial     {info.serial_number or '-'}", file=out)
    print(f"firmware   {info.firmware or '-'}", file=out)
    if info.axis_number is not None:
        print(f"axis no.   {info.axis_number} (AXNO 0x8004; slave {ec.slave})", file=out)
    if info.slave_id_raw is not None:
        print(f"slave id   {info.slave_id_raw.hex(' ')}", file=out)
    if ec.baudrate is not None:
        print(f"baud       {ec.baudrate}{' (auto-detected)' if ec.baud_detected else ''}", file=out)
    u = info.units or "mm"
    limits = info.soft_limits_mm
    print(f"soft limit {f'{limits[0]:.2f}..{limits[1]:.2f} {u}' if limits else '-'}", file=out)
    eff = ec.effective_stroke
    print(f"stroke     {f'{eff[0]:.2f}..{eff[1]:.2f} {u}' if eff else 'unknown'} (used for soft-limit checks)", file=out)
    source = "MAXV 7.4" if (info.speed_limit_source or "").startswith("MAXV") else "0x0812 fallback"
    ec_speed = f"{info.max_speed_mm_s:.2f} {u}/s ({source})" if info.max_speed_mm_s is not None else "not readable"
    print(f"max speed  unit max {ec_speed}; driver cap {ec.speed_cap_mm_s:.2f} {u}/s", file=out)
    if info.max_accel_g is not None or info.max_decel_g is not None:
        max_a = "n/a" if info.max_accel_g is None else f"{info.max_accel_g:.2f}"
        max_d = "n/a" if info.max_decel_g is None else f"{info.max_decel_g:.2f}"
        print(f"max accel  unit max {max_a} / {max_d} G (MAXA / MAXD); driver cap {ec.accel_cap_g:.2f} G", file=out)
    limits = ec.axis_limits
    if limits is not None and limits.max_push_pct is not None:
        push = "not available" if limits.push_available is False else (
            f"{limits.min_push_pct:g} % (recommended) .. {limits.max_push_pct:g} % (MXPP); driver cap {ec.push_cap_pct:g} %")
        print(f"push       {push}", file=out)
    if info.index_header is not None:
        h = info.index_header
        print(f"index      MODL={h.model_code:#06x} records={h.record_count} rx/tx buffer={h.rx_buffer}/{h.tx_buffer}", file=out)
    for cat in info.categories:
        print(
            f"  CI={cat.category_id:#06x} DA={cat.data_address:#06x} DQ={cat.member_count:<4} LQ={cat.list_count:<4} "
            f"BA={cat.coil_base:#06x}",
            file=out,
        )
    for err in info.errors:
        print(f"note       {err}", file=out)


def _run_line(ec: Elecylinder, args: argparse.Namespace, out: TextIO) -> int:
    """``line``: the line kind and its rules, the gateway status and, with ``--monitor``, this axis's RCON monitor
    record. READ-ONLY (FC03 only)."""
    line = ec.bus.line
    data: dict[str, Any] = {"line": line.to_dict(), "gateway_status": None, "monitor": None, "errors": []}
    gateway = line.gateway
    if gateway is not None:
        try:
            data["gateway_status"] = gateway.read_status().to_dict()
        except ModbusError as exc:
            data["errors"].append(f"gateway status: {type(exc).__name__}: {exc}")
    if args.monitor:
        if not isinstance(gateway, RconGateway):
            data["errors"].append("--monitor: the all-axes monitor exists on an RCON gateway only")
        else:
            try:
                record = gateway.read_monitor([ec.slave - 1]).get(ec.slave - 1)
                data["monitor"] = None if record is None else record.to_dict()
                unit = line.position_unit_mm(ec.slave)
                if record is not None and unit:
                    data["monitor"]["position_mm"] = round(record.position_counts * unit, 3)
            except (ModbusError, ValueError) as exc:
                data["errors"].append(f"gateway monitor: {type(exc).__name__}: {exc}")
    if args.json:
        print(json.dumps(data, indent=2), file=out)
        return 0
    rules = line.rules
    print(f"line       {line.kind}: {rules.title}" + (" (detected)" if line.detected else ""), file=out)
    print(f"evidence   {rules.evidence}", file=out)
    codes = "any" if rules.function_codes is None else " ".join(f"{c:02X}" for c in sorted(rules.function_codes))
    print(f"carries    function codes {codes}; coils {'yes' if rules.coils else 'NO'}; IAI 0x45 / 0x48 "
          f"{'yes' if rules.iai_commands else 'NO'}; broadcast {'yes' if rules.broadcast else 'NO'}; Report Slave ID "
          f"{'yes' if rules.report_slave_id else 'NO'}", file=out)  # fmt: skip
    print(f"strategy   {rules.command_strategy or 'the axis chooses (coil by default)'}; stop path {ec.stop_path}",
          file=out)  # fmt: skip
    unit = line.position_unit_mm(ec.slave)
    source = line.position_unit_source(ec.slave)
    print(f"pos. unit  {'NOT STATED (pass --position-unit 0.01|0.001)' if unit is None else f'{unit} mm ({source})'}",
          file=out)  # fmt: skip
    gw = data["gateway_status"]
    if gw is not None:
        manu = gw.get("manu")
        print(f"gateway    {gateway!r}: " + ("MANU" if manu else "AUTO (Modbus control is refused)")
              + f", linked axes {gw.get('linked_axes')}" + (f", alarm {gw['alarm_code']:#x} {gw['alarm_name']}"
                                                              if gw.get("alarm_code") else ""), file=out)  # fmt: skip
    mon = data["monitor"]
    if mon is not None:
        where = f"{mon['position_mm']:.3f} mm" if "position_mm" in mon else f"{mon['position_counts']} counts"
        print(f"monitor    axis {mon['axis']}: position {where}, current {mon['current_ma']} mA, alarm "
              f"{mon['alarm_code']:#x}, DSS1 {mon['dss1']:#06x}, DSSE {mon['dsse']:#06x}", file=out)  # fmt: skip
    if line.explain():
        print(f"note       {line.explain()}", file=out)
    for err in data["errors"]:
        print(f"error      {err}", file=out)
    return 0 if not data["errors"] else 1


def _rows_option(text: str) -> str | list[int]:
    if text in ("used", "all"):
        return text
    try:
        rows = [int(p) for p in text.replace(" ", "").split(",") if p]
    except ValueError:
        raise ValueError(f"--rows must be used, all or row numbers such as 0,1,2 (got {text!r})") from None
    if not rows or any(r < 0 for r in rows):
        raise ValueError(f"--rows must be used, all or row numbers such as 0,1,2 (got {text!r})")
    return rows


def _run_backup(ec: Elecylinder, label: str, args: argparse.Namespace, out: TextIO) -> int:
    """``backup``: :func:`iai_actuator_driver.info.backup` of this axis (FC03 / FC11 only), written as JSON."""
    rows = _rows_option(args.rows)
    safe = "".join(c if c.isalnum() else "-" for c in label)
    path = args.file or f"ec-backup-{safe}-s{ec.slave}-{_dt.datetime.now():%Y%m%d-%H%M%S}.json"
    data = backup(ec, path, rows=rows)
    if args.json:
        print(json.dumps(data, indent=2), file=out)
        return 0
    parameters = data.get("parameters") or {}
    items = sum(len(block.get("items") or []) for block in parameters.get("classes") or [])
    positions = (data.get("positions") or {}).get("rows") or []
    history = (data.get("alarms") or {}).get("history") or []
    print(f"backup written to {data['meta'].get('path', path)} (read-only: FC03 / FC11 only; {data['meta']['duration_s']} s)",
          file=out)  # fmt: skip
    print(f"contents   identity, directory, {items} parameter items ({len(parameters.get('named') or [])} named), "
          f"{len(positions)} position rows, {len(history)} alarm entries, maintenance counters", file=out)  # fmt: skip
    for err in data.get("errors") or []:
        print(f"not read   {err}", file=out)
    return 0


def _status_line(st: Status, units: str) -> str:
    """One compact line of a (lean) status for ``monitor --watch``."""
    flags = [
        name
        for name, on in (
            ("SERVO", st.servo_on), ("HOMED", st.homed), ("IN-POS", st.in_position), ("MOVING", st.moving),
            ("PAUSED", st.paused), ("HOMING", st.homing), ("PUSHING", st.pushing), ("MODBUS", st.modbus_enabled),
            ("ALARM", st.alarm), ("WARNING", st.warning), ("EMG", st.emergency_stop),
        )
        if on
    ]  # fmt: skip
    velocity = "" if st.velocity_mm_s is None else f"  velocity {st.velocity_mm_s:8.2f} {units}/s"
    alarm = f"  alarm 0x{st.alarm_code:03X}" if st.alarm_code else ""
    return f"position {st.position_mm:9.2f} {units}{velocity}  {' '.join(flags) or '-'}{alarm}"


def _run_watch(ec: Elecylinder, label: str, args: argparse.Namespace, out: TextIO) -> int:
    """``monitor --watch``: the status monitor with the link watch, printed until Ctrl+C or ``--duration``."""
    import queue

    try:
        ec.track(name=label)  # read-only: the baseline restart detection compares with
    except EcDriverError as exc:
        print(f"note: not tracked ({type(exc).__name__}: {exc}): a restart is not detected, a lost link still is",
              file=sys.stderr)  # fmt: skip
    items: queue.Queue[tuple[str, Any]] = queue.Queue()
    mon = ec.start_monitor(on_change=lambda st, _prev: items.put(("status", st)),
                           on_event=lambda event: items.put(("event", event)))  # fmt: skip
    started = time.perf_counter()
    deadline = None if args.duration is None else started + max(0.0, args.duration)
    interval = max(0.0, args.interval)
    pending: Status | None = None
    last_print = -math.inf
    code = 0

    def emit(kind: str, payload: dict[str, Any], text: str) -> None:
        t = round(time.perf_counter() - started, 3)
        if args.json:
            print(json.dumps({"t": t, "type": kind, **payload}), file=out, flush=True)
        else:
            print(f"{t:8.3f} s  {text}", file=out, flush=True)

    def show(st: Status) -> None:
        emit("status", {"status": st.to_dict()}, _status_line(st, ec.units))

    try:
        while deadline is None or time.perf_counter() < deadline:
            try:
                kind, item = items.get(timeout=0.05)
            except queue.Empty:
                kind, item = "", None
            if kind == "event":
                emit("event", item.to_dict(), f"{item.kind.upper()}: {item.message}")
            elif kind == "status":
                pending = item
            if pending is not None and time.perf_counter() - last_print >= interval:
                show(pending)
                pending, last_print = None, time.perf_counter()
            if not mon.running and items.empty():
                if mon.error is not None:
                    emit("stopped", {"error": f"{type(mon.error).__name__}: {mon.error}"},
                         f"status monitor stopped: {type(mon.error).__name__}: {mon.error} (run 'recover')")  # fmt: skip
                    code = 2
                break
    except KeyboardInterrupt:
        pass
    finally:
        ec.stop_monitor()
    if pending is not None:
        show(pending)
    rate = mon.poll_rate_hz
    summary = {"polls": mon.polls, "poll_rate_hz": None if rate is None else round(rate, 1),
               "errors": mon.error_count, "link_checks": mon.link_checks, "link_state": ec.link_state}  # fmt: skip
    emit("summary", summary, f"{mon.polls} polls ({summary['poll_rate_hz'] or 0:g} /s), {mon.error_count} errors, "
         f"{mon.link_checks} link checks, link {summary['link_state']}")  # fmt: skip
    return code


def _print_recovery(report: Any, ec: Elecylinder, out: TextIO) -> None:
    u = ec.units
    print(report.summary(), file=out)
    if report.restart_signs:
        print("restart signs: " + "; ".join(report.restart_signs), file=out)
    st = report.status
    if st is not None and st.raw.get("STIM") is not None:
        print(f"uptime     {st.raw['STIM'] / 1000:9.1f} s (the unit's power-on time; a power-up restarts it)", file=out)
    print(f"baud rate  {report.baudrate or 'n/a'}" + (" (restored)" if report.baud_restored else ""), file=out)
    control = " (taken again)" if report.control_restored else " (kept)" if report.control_kept else ""
    print(f"PC control {'yes' if report.modbus_control else 'no'}{control}", file=out)
    print(f"servo      {'ON' if report.servo_on else 'OFF'}", file=out)
    print(f"homed      {'yes' if report.homed else 'NO: home() before the next move'}", file=out)
    if report.position_mm is not None:
        where = f"position   {report.position_mm:9.2f} {u}"
        if report.last_target_mm is not None and report.position_error_mm is not None:
            where += f" (target {report.last_target_mm:.2f}, {report.position_error_mm:+.2f})"
        print(where, file=out)
    if report.alarm_code:
        print(f"alarm      0x{report.alarm_code:03X} {report.alarm_name or ''} ({report.alarm_level})", file=out)
    for action in report.actions:
        print(f"done:      {action}", file=out)
    for problem in (*report.limits_changed, *report.row_problems, *report.parameter_differences):
        print(f"differs:   {problem}", file=out)
    for note in report.notes:
        print(f"note:      {note}", file=out)
    print(f"ready to move: {'yes' if report.ready_to_move else 'no'}; needs attention: "
          f"{'YES' if report.needs_attention else 'no'} (nothing was moved)", file=out)  # fmt: skip


def _used_rows(data: dict[str, Any]) -> list[int] | None:
    """The rows of a backup that ``backup --rows used`` holds (the first ``max_positions`` and, on an EC, the simple-data
    rows 256-267), from the backup alone: no unit is read. ``None`` when it does not say how many rows the unit uses."""
    identity = data.get("identity") or {}
    limit = identity.get("max_positions")
    if not isinstance(limit, int) or isinstance(limit, bool):
        return None
    system = identity.get("system_code")
    simple = POSITION_SIMPLE_DATA_ROWS if system is None or is_ec_family(system) else ()
    rows = (data.get("positions") or {}).get("rows") or ()
    held = {r["n"] for r in rows if isinstance(r, dict) and isinstance(r.get("n"), int)}
    return sorted(n for n in held if n < limit or n in simple)


def _run_recover(ec: Elecylinder, label: str, args: argparse.Namespace, out: TextIO) -> int:
    """``recover``: re-sync an axis after a power or link loss (never moves it)."""
    from . import recovery

    if args.backup:
        try:
            data = json.loads(Path(args.backup).read_text(encoding="utf-8"))
            recovery.session_from_backup(ec, data, name=label)
        except (OSError, ValueError) as exc:
            print(f"error: backup {args.backup}: {exc}", file=sys.stderr)
            return 2
    else:
        data = None
        recovery.forget(ec)
        recovery.session_from_info(ec, ec.identify(), name=label)
    recovery.note_control(ec, modbus_control=not args.no_control, baudrate=args.restore_baud)
    if args.target is not None:
        recovery.note_target(ec, args.target)
    rows: bool | list[int] = data is not None
    if data is not None and args.rows == "used":
        rows = _used_rows(data)
        if rows is None:
            print("note: the backup does not say how many rows the unit uses (identity.max_positions): "
                  "comparing every row it holds", file=sys.stderr)  # fmt: skip
            rows = True
        elif not rows:
            print("note: the backup holds none of the rows the unit uses: no row is compared", file=sys.stderr)
    policy = recovery.RecoveryPolicy(check_rows=rows, check_parameters=data is not None, backup=data)
    report = ec.recover(timeout=max(0.0, args.wait), policy=policy, name=label)
    if args.json:
        print(json.dumps(report.to_dict(), indent=2, default=str), file=out)
    else:
        _print_recovery(report, ec, out)
    return 0


def _override_note(pct: float) -> None:
    """The note when the speed override is rounded: the unit runs (code + 1) / 256 of the row's stored speed (ON-WIRE
    on the PCON-CB), code = round(256 x pct / 100) - 1 within 1..255 (the driver's own rule)."""
    code = min(255, max(1, round(256 * pct / 100) - 1))
    runs = (code + 1) * 100 / 256
    if abs(runs - pct) > 0.05 * pct:
        print(f"note: --speed-pct {pct:g}: the row runs at {runs:.2f} % of its stored speed (override code {code}; "
              "the unit runs (code + 1) / 256)", file=sys.stderr)  # fmt: skip


def _move_and_change(ec: Elecylinder, args: argparse.Namespace) -> Status | None:
    """``move ... --change-speed / --change-accel``: start the move, wait ``--change-after`` s, change the running move
    and wait for it to finish (:meth:`Elecylinder.change_speed` / ``change_accel``; VERIFIED on the EC-GRBP8M gripper,
    the speed change also on the PCON-CB)."""
    ec.move_to(args.mm, speed_mm_s=args.speed, accel_g=args.accel, band_mm=args.band, wait=False)
    time.sleep(max(0.0, args.change_after))
    if args.change_speed is not None:
        print(f"changing the speed of the running move to {args.change_speed:g} {ec.units}/s", file=sys.stderr)
        result = ec.change_speed(args.change_speed, wait=args.change_accel is None)
    if args.change_accel is not None:
        print(f"changing the acceleration of the running move to {args.change_accel:g} G", file=sys.stderr)
        result = ec.change_accel(args.change_accel, wait=True)
    return result


def _print_alarms(records: list[AlarmRecord], out: TextIO) -> None:
    if not records:
        print("alarm history empty", file=out)
    for r in records:
        stamp = r.time_after_power_on or "-"
        print(f"#{r.index:<2} code {r.code:#06x} {r.level:<19} detail {r.detail:<5} {stamp:>17} after power-on  "
              f"{r.name}", file=out)


class _UnitsOut:
    """A text stream that shows mm as ``units`` (deg on a rotary axis): only the unit label differs per axis."""

    def __init__(self, out: TextIO, units: str) -> None:
        self._out, self._units = out, units

    def write(self, text: str) -> int:
        if self._units != "mm":
            text = text.replace(" mm/s", f" {self._units}/s").replace(" mm", f" {self._units}")
        return self._out.write(text)

    def flush(self) -> None:
        self._out.flush()


def _in_units(out: TextIO, units: str | None) -> TextIO:
    return out if not units or units == "mm" else _UnitsOut(out, units)  # type: ignore[return-value]


def _mm_range(value: Any) -> str:
    return "-" if not value else f"{value[0]:.2f} .. {value[1]:.2f} mm"


def _bits_line(name: str, word: dict[str, Any] | None) -> str:
    if word is None:
        return f"  {name:<5} -"
    return f"  {name:<5} {word['hex']:<10}  {' '.join(word['on']) or '-'}"


def _print_unit_info(data: dict[str, Any], out: TextIO) -> None:
    """Pretty text for ``info`` (``--json`` carries every field, including each bit's meaning and evidence).
    Positions and speeds are shown in the axis's units (deg / deg/s on a rotary axis)."""
    out = _in_units(out, (data.get("identity") or {}).get("units"))

    def p(text: str = "") -> None:
        print(text.rstrip(), file=out)

    meta = data["meta"]
    p(f"{BANNER} - unit info, read-only (FC03 + FC11), {meta['taken_at']}, {meta.get('duration_s', 0):.2f} s")
    ident = data.get("identity")
    if ident:
        p("\n[identity]")
        p(f"  model         {ident.get('model') or '-'}")
        p(f"  unit name     {ident.get('user_name') or '-'} (0x061A, user-defined; not a model field)")
        p(f"  serial        {ident.get('serial_number') or '-'}")
        p(f"  controller    {ident.get('controller')}, firmware {ident.get('firmware') or '-'}"
          + (f" (identity from {ident['identity_source']})" if ident.get("identity_source") not in (None, "FC11") else ""))
        p(f"  actuator      {ident.get('actuator') or '-'}")
        if ident.get("axis_number") is not None:
            p(f"  axis number   {ident['axis_number']} (AXNO 0x8004)")
        if ident.get("slave_id_raw"):
            p(f"  slave id      {ident['slave_id_raw']} (FC11)")
        baud = ident.get("baudrate")
        detected = " (auto-detected)" if ident.get("baud_detected") else ""
        link = f"baud {baud}{detected}" if baud else "no baud rate (simulator)"
        p(f"  link          slave {ident.get('slave')}, {link}")
        p(f"  soft limits   {_mm_range(ident.get('soft_limits_mm'))} (stroke {_mm_range(ident.get('stroke_mm'))}, "
          f"used {_mm_range(ident.get('effective_stroke_mm'))})")  # fmt: skip
        ec_speed = ident.get("ec_max_speed_mm_s")
        p(f"  speed         unit max {'-' if ec_speed is None else f'{ec_speed:.2f}'} mm/s (MAXV 7.4), driver cap "
          f"{ident.get('speed_cap_mm_s', 0):.2f} mm/s, accel cap {ident.get('accel_cap_g', ident.get('max_accel_g', 0)):.2f} G, "
          f"push cap {ident.get('push_cap_pct') or '-'} %")  # fmt: skip
        caps = ident.get("capabilities") or {}
        flags = ((caps.get("support") or {}).get("flags")) or {}
        if flags:
            on = [k.replace("_", " ") for k, v in flags.items() if v is True]
            p(f"  features      {', '.join(on) or '-'} (class 1 support bits)")
        coll = caps.get("collision") or {}
        if coll:
            p(f"  collision     {coll.get('reported')}; parameter items {', '.join(coll.get('parameter_items') or []) or 'none'}")
        link = caps.get("link") or {}
        if link.get("boot_baudrate"):
            p(f"  boot baud     {link['boot_baudrate']} (BRSL item 8.3), transmit delay {link.get('transmit_delay_ms')} ms")
        sm = caps.get("servo_monitor") or {}
        if sm:
            p(f"  servo monitor {sm.get('mode_name')} (FMNT 8.54 = {sm.get('fmnt_hex')})")
        mode = ident.get("position_mode") or {}
        p(f"  position mode {mode.get('name')} ({mode.get('source')})")
        extra = ident.get("identification_extra") or {}
        if extra:
            accl = extra.get("accl") or {}
            p(f"  0x0600 extra  actuator serial {extra.get('actuator_serial')!r}, manufacture info "
              f"{(extra.get('manufacture_info') or {}).get('hex')}, ACCL {accl.get('hex')} (level "
              f"{accl.get('extensibility_level')} {accl.get('level_name')}), BISD {extra.get('bisd_raw')} "
              f"(date? {extra.get('date_guess') or '-'})")  # fmt: skip
        directory = ident.get("directory") or {}
        header = directory.get("header") or {}
        p(f"  directory     {directory.get('record_count', 0)} records, {directory.get('total_words', 0)} registers "
          f"(MODL {header.get('model_code', 0):#06x}, buffers {header.get('rx_buffer')}/{header.get('tx_buffer')})")  # fmt: skip
        for r in directory.get("records", []):
            words = f"{r['words']}{'' if r['exact'] else '*'}"
            p(f"    {r['class']}  {r['address']}-{r['end']}  {words:>6}  DQ {r['members']:<3} LQ {r['entries']:<4} "
              f"MS {r['member_size']}  id map {r['id_map'] or '-':<6}  {r['name']}")  # fmt: skip
        if any(not r["exact"] for r in directory.get("records", [])):
            p("    * mixed member sizes: only this many registers are certain (the driver reads no further)")
        for note in ident.get("notes", []):
            p(f"  note          {note}")
    params = data.get("parameters")
    if params:
        p("\n[user parameters]")
        model = params.get("model") or {}
        if model:
            p(f"  ({model.get('note')})")
        for prm in params["named"]:
            where = f"{prm['key']} @{prm['address']} = {prm['raw_hex']}" if prm["found"] else prm["key"]
            flag = f"  [{prm['model_note']}]" if prm.get("model_note") else ""
            p(f"  P{prm['no']:<3} {prm['name']:<52} {prm['text']:<28} {where}{flag}")
        for q in params.get("not_located") or []:
            if q.get("listed"):
                seen = f" (IA-OS showed {q['iaos_text']!r}; not read from the EC)" if q.get("iaos_text") else ""
                p(f"  P{q['no']:<3} {q['name']:<52} {'not located' + seen}")
        p(f"  ({params['note']})")
        for block in params["classes"]:
            p(f"\n  class {block['class']} @{block['address']}: {block['name']} ({len(block['items'])} items)")
            for it in block["items"]:
                tags = " ".join([*it.get("params", []), it.get("note", "")]).strip()
                p(f"    {it['key']:<8} {it['address']}  {it['hex']:>8}  {it['signed']:>11}  {it['scaled']:>12.2f}  "
                  f"{tags}")  # fmt: skip
    io = data.get("io")
    if io:
        p(f"\n[I/O] {io['mode']}")
        for name in ("DIPM", "DOPM", "SIPM", "ZONS"):
            p(_bits_line(name, io.get(name)))
        lamps = io.get("lamps", {})
        p("  lamps         " + ", ".join(f"{k.replace('_', ' ')} {'ON' if v else '-'}" for k, v in lamps.items()))
        p(f"  ({io['note']})")
    words = data.get("status_words")
    if words:
        p("\n[status words]")
        for name in ("DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SSSE", "SIPM"):
            if name in words:
                p(_bits_line(name, words[name]))
        if "POSS" in words:
            p(f"  POSS  {words['POSS']['hex']:<10}  position {words['POSS']['position_no']}")
    control = data.get("control_words")
    if control:
        p("\n[control registers 0x0D00]")
        for name, word in control.items():
            p(_bits_line(name, word) if "bits" in word else f"  {name:<5} {word['hex']}")
    mon = data.get("monitor")
    if mon:
        p("\n[monitor]")
        if "position_mm" in mon:
            p(f"  position      {mon['position_mm']:.2f} mm")
        if "velocity_mm_s" in mon:
            p(f"  velocity      {mon['velocity_mm_s']:.2f} mm/s (VNOW, quantized)")
        pct = f", {mon['current_pct']} % of rated (MCUR {mon['rated_current']})" if "current_pct" in mon else ""
        p(f"  current       {mon.get('current', '-')} (CNOW{pct})")
        p(f"  deviation     {mon.get('deviation', '-')} (DEVI)")
        if "uptime_ms" in mon:
            p(f"  uptime        {mon['uptime']} ({mon['uptime_ms']} ms since power-on)")
        if "overload_level" in mon:
            p(f"  overload lvl  {mon['overload_level']} (OLLV, % unconfirmed)")
    alarms = data.get("alarms")
    if alarms:
        p("\n[alarms]")
        cur = alarms.get("current")
        if cur is not None:
            p(f"  current       {cur['hex']} {cur['name']}" + (f" ({cur['level_text']})" if cur["active"] else ""))
            if cur["active"]:
                p(f"  remedy        {cur['remedy']}")
        p(f"  history       {alarms['alarm_count']} alarms, {alarms['power_up_markers']} power-up markers, "
          f"{alarms['empty_slots']} empty of {alarms['slots']} slots ({alarms['notes']['order']})")  # fmt: skip
        for a in alarms["history"]:
            stamp = a.get("time_after_power_on") or "-"
            p(f"  #{a['index']:<3} {a['hex']}  {a['name']:<40} {a['level']:<19} detail {a['detail']:<5} "
              f"{stamp:>17} after power-on  {a['newer_power_ups']} newer power-ups")  # fmt: skip
    maint = data.get("maintenance")
    if maint:
        p("\n[maintenance]")
        p(f"  total moves   {maint['total_moves']}")
        p(f"  distance      {maint['distance_m']} m")
        for it in maint["items"]:
            value = "not set" if it["unset"] else str(it["value"])
            if "hi16" in it:
                value += f" ({it['hi16']} / {it['lo16']})"
            sure = "" if it["confirmed"] else "?"
            p(f"    {it['id']:>3} {it['symbol'] + sure:<6} {it['address']}  {it['hex']}  {value:<20} {it['meaning']}")
        p(f"  ({maint['note']})")
    pos = data.get("positions")
    if pos:
        p(f"\n[positions] {pos['mode']}")
        for r in pos["rows"]:
            push = f", push {r['push_pct']} % to {r['push_end_mm']:.2f} mm ({r['push_direction']})" if r["push"] else ""
            p(f"  row {r['n']:<3} {r['role']:<26} {r['position_mm']:8.2f} mm  {r['speed_mm_s']:7.2f} mm/s  "
              f"A/D {r['accel_g']:.2f}/{r['decel_g']:.2f} G  band {r['band_mm']:.2f}{push}  CTLF {r['control_flags']}  "
              f"checksum {'ok' if r['checksum_ok'] else 'BAD'}")  # fmt: skip
        for r in pos["simple_data_rows"]:
            p(f"  row {r['n']:<3} {r['role']:<26} {r['position_mm']:8.2f} mm  V {r['speed_pct']} %  "
              f"A/D {r['accel_pct']}/{r['decel_pct']} %  band {r['band_mm']:.2f}  "
              f"CTLF {r['control_flags']}")  # fmt: skip
        dm = pos.get("direct_move")
        if dm:
            p(f"  0x9900 last   {dm['target_mm']:.2f} mm, {dm['speed_mm_s']:.2f} mm/s, {dm['accel_g']:.2f} G, band "
              f"{dm['band_mm']:.2f}, PPOW {dm.get('push_power')}, CTLF {dm.get('control_flags')}")  # fmt: skip
        ap = pos.get("active_profile")
        if ap and "target_mm" in ap:
            p(f"  0x0F00 used   {ap['target_mm']:.2f} mm, {ap['speed_mm_s']:.2f} mm/s, A/D {ap['accel_g']:.2f}/"
              f"{ap['decel_g']:.2f} G, band {ap['band_mm']:.2f}")  # fmt: skip
    if data.get("errors"):
        p("\n[not read]")
        for err in data["errors"]:
            p(f"  {err}")


def _print_params(params: list[dict[str, Any]], model: dict[str, Any], out: TextIO) -> None:
    print(f"user parameters ({model['note']})", file=out)
    for prm in params:
        if not prm.get("located", True):
            seen = f" (IA-OS showed {prm['iaos_text']!r})" if prm.get("iaos_text") else ""
            print(f"  P{prm['no']:<3} {prm['name']:<52} {'not located' + seen:<34} read-only", file=out)
            continue
        flags = []
        if prm.get("writable"):
            flags.append("writable")
        if prm.get("read_only"):
            flags.append("READ-ONLY")
        if prm.get("home_parameter"):
            flags.append("HOME, DANGEROUS")
        if prm.get("model_note"):
            flags.append(prm["model_note"])
        where = f"{prm['key']} @{prm['address']} = {prm['raw_hex']}" if prm.get("found") else prm.get("key") or ""
        print(f"  P{prm['no']:<3} {prm['name']:<52} {prm['text']:<34} {where}  {'; '.join(flags)}", file=out)


def _print_generic_params(params: list[dict[str, Any]], out: TextIO) -> None:
    """``Elecylinder.parameters()`` of a CON-family unit (No.-numbers)."""
    if not params:
        print("no named parameter found in this unit's ID map", file=out)
    for prm in params:
        flags = ["writable" if prm.get("writable") else "READ-ONLY"]
        if prm.get("dangerous"):
            flags.append("DANGEROUS")
        rng = ""
        if prm.get("min") is not None and prm.get("max") is not None:
            rng = f"[{prm['min']:g}..{prm['max']:g}]"
        print(f"  No.{prm['number']:<4} {prm['name']:<44} {str(prm.get('text')):<22} {rng:<18} "
              f"{prm['item']} @{prm.get('address')} = {prm.get('raw')}  {'; '.join(flags)}", file=out)  # fmt: skip


def _print_limits(limits: dict[str, Any], out: TextIO) -> None:
    print(f"units      {limits['units']} (kind {limits['kind']}, family {limits['family']})", file=out)
    for name, entry in limits.items():
        if not isinstance(entry, dict):
            continue
        lo, hi = entry.get("min"), entry.get("max")
        rng = f"{'-' if lo is None else f'{lo:g}'} .. {'-' if hi is None else f'{hi:g}'}"
        extra = []
        if "available" in entry:
            extra.append("available" if entry["available"] else "NOT available")
        if entry.get("recommended_min") is not None:
            extra.append(f"recommended min {entry['recommended_min']:g}")
        source = entry.get("source") or ""
        print(f"{name:<14} {rng:<22} {entry.get('unit') or '':<6} {'; '.join(extra + [source] if source else extra)}",
              file=out)  # fmt: skip


def _print_monitor(m: dict[str, Any], out: TextIO) -> None:
    u = m.get("units") or "mm"

    def num(v: Any, fmt: str = ".2f") -> str:
        return "-" if v is None else format(v, fmt)

    pct = "" if m.get("current_pct") is None else f" ({m['current_pct']:g} % of rated)"
    alarm = f"0x{m['alarm']:03X} {m.get('alarm_name') or ''}".rstrip() if m.get("alarm") else "-"
    print(f"position {num(m.get('position'))} {u}  velocity {num(m.get('velocity'))} {u}/s  current "
          f"{num(m.get('current'), 'd')}{pct}  deviation {num(m.get('deviation'), 'd')}  alarm {alarm}  "
          f"uptime {num(m.get('uptime_s'), '.1f')} s", file=out)  # fmt: skip


def _param_key(text: str) -> int | str:
    return int(text) if text.strip().isdigit() else text.strip()


def _print_param_write(result: ParameterWriteResult, out: TextIO, prefix: str = "P", again: str = "") -> None:
    """``again``: the command that commits this value later (``param set KEY VALUE``), for the RAM-only line."""
    label = f"{prefix}{result.no}" if result.no else "item"
    if result.skipped:
        print(f"{label} {result.name} ({result.key} @{result.address}) = {result.after} (raw {result.raw_after}): "
              f"{result.note}; --force writes it anyway (needed when an earlier --no-commit run left this value in "
              "RAM only: this run cannot see that)", file=out)  # fmt: skip
        for warning in result.warnings:
            print(f"warning: {warning}", file=out)
        return
    if not result.changed and not result.committed:  # --no-commit with the value the unit already holds: nothing sent
        print(f"{label} {result.name} ({result.key} @{result.address}) = {result.after} (raw {result.raw_after}): "
              "unchanged, the unit already holds this value; nothing written (--no-commit)", file=out)  # fmt: skip
        for warning in result.warnings:
            print(f"warning: {warning}", file=out)
        return
    print(f"{label} {result.name} ({result.key} @{result.address}): {result.before} -> {result.after}"
          f" (raw {result.raw_before} -> {result.raw_after}, read back OK)", file=out)  # fmt: skip
    if result.committed and result.commit_confirmed:
        print("committed to non-volatile memory (PARW 0x0428: the EC showed the commit, then idle again)", file=out)
    elif result.committed:
        print("PARW 0x0428 sent, but the commit was NOT CONFIRMED (the EC never showed it busy): the value may be "
              "RAM only: check it after a power cycle", file=out)
    else:
        print("RAM ONLY (--no-commit): lost at the next power-up. To save it later run "
              f"'{again or 'param set KEY VALUE'} --force' (without --force a new run sees the value already in the "
              "unit and commits nothing)", file=out)  # fmt: skip
    print(f"takes effect {result.takes_effect}; run 'swreset' to apply it now", file=out)
    for warning in result.warnings:
        print(f"warning: {warning}", file=out)


def _print_reset(result: SoftwareResetResult, out: TextIO) -> None:
    reply = "accepted (01 45 00 00)" if result.accepted else "answer lost; confirmed from the status"
    print(f"software reset {reply}, EC answering again after {result.elapsed_s:.2f} s"
          + (f" at {result.baudrate} baud" if result.baudrate else ""), file=out)  # fmt: skip
    print(f"PMSS {int(result.modbus_enabled)}  SFTY {int(result.safety_speed)}  servo {'ON' if result.servo_on else 'OFF'}"
          f"  HEND {int(result.homed)}  alarm {result.alarm_code:#05x}", file=out)  # fmt: skip
    if result.confirmed:
        print(f"restart confirmed by {result.confirmed_by or 'the status'}", file=out)
    else:
        print("restart NOT confirmed (no PMSS drop, reply gap, uptime restart or HEND loss seen): check the unit",
              file=out)  # fmt: skip
    if result.needs_home:
        print("NEEDS HOME: run 'home' (after 'alarm-reset' if an alarm is active) before the next move", file=out)
    for note in result.notes:
        print(f"note: {note}", file=out)


def _print_baud(result: BaudChangeResult, out: TextIO) -> None:
    how = "verified" if result.verified else "found by probing"
    print(f"baud {result.previous} -> {result.baudrate} ({result.method}; broadcast {result.frame}; {how})", file=out)
    if not result.code_verified:
        print(f"warning: code 0x{result.code:02X} is not verified on hardware (sent anyway)", file=out)
    print(f"note: {result.note}", file=out)


def _print_item(item: CheckItem, out: TextIO) -> None:
    print(f"{item.result:<4}  {item.name:<18} {item.detail}", file=out)


def _save_report(report: CheckReport, path: str, out: TextIO) -> str:
    """Write the JSON report; a reporting failure falls back to the temp dir, then to stdout."""
    try:
        return f"report written to {report.write_json(path)}"
    except OSError as exc:
        print(f"warning: could not write the report to {path}: {exc}", file=sys.stderr)
    fallback = Path(tempfile.gettempdir()) / Path(path).name
    try:
        return f"report written to {report.write_json(fallback)} (fallback location)"
    except OSError as exc:
        print(f"warning: could not write the report to {fallback} either: {exc}", file=sys.stderr)
    print(report.to_json(), file=out)
    return "report printed above"


def _run_check(ec: Elecylinder, label: str, args: argparse.Namespace, out: TextIO) -> int:
    print(BANNER, file=out)
    print(f"compatibility check on {label}, slave {ec.slave}, strategy {ec.strategy.name}", file=out)
    report = run_check(ec, motion=args.motion, port_label=label, on_item=lambda item: _print_item(item, out))
    path = args.report or f"iai-check-{_dt.datetime.now():%Y%m%d-%H%M%S}.json"
    where = _save_report(report, path, out)
    print(f"RESULT {'PASS' if report.passed else 'FAIL'} - {where}", file=out)
    return 0 if report.passed else 1


def _execute(ec: Elecylinder, label: str, args: argparse.Namespace, out: TextIO) -> int:
    cmd = args.command
    result: Status | DeviceInfo | PushResult | list[AlarmRecord] | None = None
    if cmd == "check":
        return _run_check(ec, label, args, out)
    if cmd == "line":
        return _run_line(ec, args, out)
    if cmd == "backup":
        return _run_backup(ec, label, args, out)
    if cmd == "recover":
        return _run_recover(ec, label, args, out)
    if cmd == "reset":
        print(DEPRECATED_RESET_NOTE, file=sys.stderr)
    if cmd in STATE_CHANGING or (cmd == "param" and args.param_cmd == "set"):
        # Session start: identity, directory, soft limits and speed limit; Modbus control only at rest.
        before = ec.status()
        if not before.modbus_enabled and _at_rest_for_control(before):
            # with --json stdout stays one JSON document, so the note goes to stderr (as in the C++ command line)
            print("enabling Modbus control (PMSL) while the axis is at rest", file=sys.stderr if getattr(args, "json", False) else out)
        ec.connect(take_control=True)
        if getattr(args, "fast", False):
            ec.enable_fast(baudrate=None)  # fast completion + the status monitor; the baud rate is kept
    if cmd == "teach":
        entry = ec.teach_position(args.n, method=args.method, force=args.force)  # --allow-nonvolatile: ignored
        if args.json:
            print(json.dumps(dataclasses.asdict(entry), indent=2), file=out)
        elif entry.skipped:
            print(f"row {args.n} already holds the current position {entry.position_mm:.2f} {ec.units}: {NV_SKIP_NOTE}; "
                  "--force writes it anyway", file=out)  # fmt: skip
        else:
            print(f"row {args.n} taught from the current position: {entry.position_mm:.2f} {ec.units}, speed "
                  f"{entry.speed_mm_s:.2f}, accel / decel {entry.accel_g:.2f} / {entry.decel_g:.2f} G, band "
                  f"{entry.band_mm:.2f} (non-volatile, read back)", file=out)  # fmt: skip
        return 0
    if cmd == "param":
        is_ec = ec.is_ec
        if args.param_cmd == "list":
            params = ec.user_parameters() if is_ec else ec.parameters()
            if args.json:
                print(json.dumps(params, indent=2), file=out)
            elif is_ec:
                _print_params(params, ec.parameter_model(), _in_units(out, ec.units))
            else:
                print(f"{ec.product_family} parameters (offered when the item is in the unit's ID map)", file=out)
                _print_generic_params(params, out)
            return 0
        if args.param_cmd == "get":
            print(json.dumps(ec.read_parameter(_param_key(args.key)), indent=2), file=out)
            return 0
        # --allow-dangerous / --allow-home-params / --allow-unlisted: accepted and ignored (no longer needed)
        written = ec.write_parameter(_param_key(args.key), args.value, commit=not args.no_commit, force=args.force)
        if args.json:
            print(json.dumps(written.to_dict(), indent=2), file=out)
        else:
            _print_param_write(written, out, "P" if is_ec else "No.", f"param set {args.key} {args.value}")
        return 0
    if cmd == "limits":
        data = ec.limits()
        if args.json:
            print(json.dumps(data, indent=2), file=out)
        else:
            _print_limits(data, out)
        return 0
    if cmd == "monitor":
        if args.watch:
            return _run_watch(ec, label, args, out)
        for i in range(max(1, args.count)):
            if i:
                time.sleep(max(0.0, args.interval))
            m = ec.read_monitor()
            if args.json:
                print(json.dumps({k: v for k, v in m.items() if k != "words"}), file=out)
            else:
                _print_monitor(m, out)
        return 0
    if cmd in ("swreset", "baud"):
        done = (
            ec.software_reset() if cmd == "swreset"
            else ec.set_baudrate(args.rate, method=args.method)  # --allow-unverified-code: ignored
        )  # fmt: skip
        if args.json:
            print(json.dumps(done.to_dict(), indent=2), file=out)
        elif isinstance(done, SoftwareResetResult):
            _print_reset(done, out)
        else:
            _print_baud(done, out)
        return 0
    if cmd == "info":
        data = ec.unit_info(sections=args.section)
        if args.json:
            print(json.dumps(data, indent=2), file=out)
        else:
            _print_unit_info(data, out)
        return 0
    if cmd == "status":
        result = ec.status()
    elif cmd == "identify":
        result = ec.identify()
    elif cmd == "alarms":
        result = ec.read_alarm_history()
    elif cmd == "servo":
        result = ec.servo(args.state == "on")
    elif cmd == "home":
        result = ec.home(method=args.method)
    elif cmd == "move":
        if args.change_speed is not None or args.change_accel is not None:
            result = _move_and_change(ec, args)
        else:
            result = ec.move_to(args.mm, speed_mm_s=args.speed, accel_g=args.accel, band_mm=args.band,
                                wait=not args.no_wait, decel_g=args.decel)  # fmt: skip
        if args.no_wait:
            result = None  # a MoveHandle: the command is sent and the CLI does not wait
        if result is None:
            print(f"move to {args.mm:.2f} {ec.units} sent", file=out)
    elif cmd == "jog":
        result = ec.jog(args.delta, speed_mm_s=args.speed)
    elif cmd == "push":
        direction = -1 if args.toward_home else 1
        result = ec.push(args.mm, args.band, args.pct, direction, speed_mm_s=args.speed)
    elif cmd == "goto":
        if args.speed_pct is not None and 0 < args.speed_pct <= 100:  # out of range: refused by move_to_position
            _override_note(args.speed_pct)
        result = ec.move_to_position(args.n, wait=not args.no_wait, allow_above_caps=args.allow_fast,
                                     speed_pct=args.speed_pct)  # fmt: skip
        if result is None:
            print(f"stored position {args.n} started", file=out)
    elif cmd == "stop":
        result = ec.stop(wait=True)
    elif cmd == "pause":
        ec.pause()
        print("paused: the move is held and keeps its remaining travel; 'resume' continues it, 'cancel-pause' drops it "
              "(the pause stays set after this command ends)", file=out)
    elif cmd == "resume":
        ec.resume()
        print("pause released: a paused move continues by itself", file=out)
    elif cmd == "cancel-pause":
        result = ec.cancel_paused_move()
    elif cmd in ("alarm-reset", "reset"):
        result = ec.reset_alarm(method=getattr(args, "method", None))
    elif cmd == "mode":
        result = ec.set_action_mode(args.which == "pc", args.safety)
    if result is None:
        return 0
    if isinstance(result, list):
        if args.json:
            print(json.dumps([r.to_dict() for r in result], indent=2), file=out)
        else:
            _print_alarms(result, out)
    elif args.json:
        print(json.dumps(result.to_dict(), indent=2), file=out)
    elif isinstance(result, Status):
        _print_status(result, out, ec.units)
    elif isinstance(result, PushResult):
        print(f"push {result.outcome} at {result.position_mm:.2f} {ec.units}", file=out)
        for warning in result.warnings:
            print(f"warning: {warning}", file=out)
        _print_status(result.status, out, ec.units)
    else:
        _print_info(result, ec, out)
    return 0


def _report_error(exc: BaseException) -> None:
    print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)


PORT_COMMANDS = ("ports", "scan", "stop-all")
_VALUE_OPTIONS = frozenset({
    "--port", "--ports", "--slave", "--axis", "--slaves", "--baud", "--timeout", "--retries", "--strategy", "--stroke",
    "--max-speed", "--max-accel", "--max-push", "--sim-profile", "--sim-time-scale", "--sim-baud", "--sim-unit",
    "--sim-slaves", "--scan-timeout", "--line", "--position-unit", "--sim-line",
})  # fmt: skip


def _port_command(argv: Sequence[str]) -> str | None:
    """``"ports"`` / ``"scan"`` / ``"stop-all"`` when that is the command of ``argv`` (these need no ``--port``)."""
    skip = False
    for token in argv:
        if skip:
            skip = False
            continue
        if token.startswith("-"):
            skip = token in _VALUE_OPTIONS
            continue
        return token if token in PORT_COMMANDS else None
    return None


def _slave_list(text: str) -> list[int]:
    """``"1-4,7"`` -> [1, 2, 3, 4, 7]."""
    out: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        lo, sep, hi = part.partition("-")
        first, last = int(lo), int(hi) if sep else int(lo)
        if not 1 <= first <= last <= 247:
            raise argparse.ArgumentTypeError(f"bad slave range {part!r} (addresses 1..247)")
        out.extend(range(first, last + 1))
    if not out:
        raise argparse.ArgumentTypeError("no slave address given")
    return list(dict.fromkeys(out))


def build_port_parser() -> argparse.ArgumentParser:
    """Parser for the port-level commands ``iai-actuator-driver ports`` and ``iai-actuator-driver scan``."""
    parser = argparse.ArgumentParser(prog="iai-actuator-driver", description=f"{BANNER} - ports and axes")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    sub = parser.add_subparsers(dest="command", required=True)
    ports = sub.add_parser("ports", help="list the serial ports, IAI USB adapters first (opens nothing)")
    ports.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="machine-readable output")
    scan = sub.add_parser("scan", help="find the axes that answer on each port (READ-ONLY)")
    scan.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="machine-readable output")
    where = scan.add_mutually_exclusive_group()
    where.add_argument("--ports", "--port", dest="ports", default=None,
                       help="comma-separated ports, e.g. COM3,COM4 (default: every IAI USB adapter)")  # fmt: skip
    where.add_argument("--sim", action="store_true", help="scan a simulated line instead of hardware")
    scan.add_argument(
        "--slaves", type=_slave_list, default=list(DEFAULT_SLAVES),
        help="addresses to ask, e.g. 1-16 or 1,2,5 (default 1-16 = axis 0..15)",
    )  # fmt: skip
    scan.add_argument("--baud", type=int, default=None, help="fixed baud rate (default: auto-detect per port)")
    scan.add_argument(
        "--scan-timeout", type=float, default=SCAN_TIMEOUT_S,
        help=f"wait per address in s (default {SCAN_TIMEOUT_S})",
    )  # fmt: skip
    scan.add_argument("--log-frames", action="store_true", help=_LOG_FRAMES_HELP)
    scan.add_argument("--sim-slaves", type=_slave_list, default=[1], help=argparse.SUPPRESS)
    scan.add_argument("--sim-profile", choices=sorted(PROFILES), default="slider", help=argparse.SUPPRESS)
    scan.add_argument("--sim-baud", type=int, default=None, help=argparse.SUPPRESS)
    stop = sub.add_parser(
        "stop-all",
        help="decel STOP on every axis of the ports (default: every IAI adapter, addresses 1-16); not a safety function",
    )
    stop.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="machine-readable output")
    where = stop.add_mutually_exclusive_group()
    where.add_argument("--ports", "--port", dest="ports", default=None,
                       help="comma-separated ports, e.g. COM3,COM4 (default: every IAI USB adapter)")  # fmt: skip
    where.add_argument("--sim", action="store_true", help="stop a simulated line instead of hardware")
    stop.add_argument("--slaves", type=_slave_list, default=list(DEFAULT_SLAVES),
                      help="addresses to look for, e.g. 1-16 or 1,2,5 (default 1-16 = axis 0..15)")  # fmt: skip
    stop.add_argument("--baud", type=int, default=None, help="fixed baud rate (default: auto-detect per port)")
    stop.add_argument("--scan-timeout", type=float, default=SCAN_TIMEOUT_S,
                      help=f"wait per address in s (default {SCAN_TIMEOUT_S})")  # fmt: skip
    stop.add_argument(
        "--broadcast", action="store_true",
        help="also send one broadcast STOP (address 0) on every daisy chain whose axes are all stopped; never behind "
        "a gateway; several units acting on one broadcast is UNVERIFIED on hardware",
    )  # fmt: skip
    stop.add_argument("--log-frames", action="store_true", help=_LOG_FRAMES_HELP)
    stop.add_argument("--sim-slaves", type=_slave_list, default=[1], help=argparse.SUPPRESS)
    stop.add_argument("--sim-profile", choices=sorted(PROFILES), default="slider", help=argparse.SUPPRESS)
    stop.add_argument("--sim-baud", type=int, default=None, help=argparse.SUPPRESS)
    return parser


def _print_ports(entries: list[dict[str, Any]], out: TextIO) -> None:
    if not entries:
        print("no serial port found", file=out)
        return
    print(f"{'PORT':<10} {'USB ID':<10} {'IAI':<4} DESCRIPTION", file=out)
    for e in entries:
        usb = f"{e['vid']:04X}:{e['pid']:04X}" if e["vid"] is not None and e["pid"] is not None else "-"
        print(f"{e['port']:<10} {usb:<10} {'yes' if e['is_iai'] else 'no':<4} {e['description']}", file=out)


def _print_scan(name: str, baud: int | None, hits: list[ScanHit], error: str | None, out: TextIO) -> None:
    rate = f"{baud} baud" if baud is not None else "no baud rate (simulated)"
    if error is not None:
        print(f"{name}: error: {error}", file=out)
        return
    if not hits:
        print(f"{name}: no axis answered", file=out)
        print(f"  {NO_ANSWER_HINT}", file=out)
        return
    print(f"{name}: {len(hits)} axis/axes at {rate}", file=out)
    for h in hits:
        if h.error is not None:
            what = h.error
        else:
            family = h.family or h.product_family or "-"
            what = f"{h.system_name}, unit 0x{h.unit_code:02X}, firmware {h.firmware}: {family}"
        print(f"  {h.name:<12} axis {h.axis_number:<3} {what}", file=out)


def _stop_all(targets: list[tuple[str, Any]], args: argparse.Namespace, out: TextIO, frame_logger: Any) -> int:
    """``stop-all``: open the ports (read-only scan), then :meth:`Cell.stop_all` (STOP to every axis first, then wait
    for each to stand still; ``--broadcast`` adds one broadcast STOP on daisy chains, UNVERIFIED)."""
    try:
        cell = Cell.open(dict(targets), scan_slaves=args.slaves, identify=False, baudrate=args.baud,
                         scan_timeout=args.scan_timeout, frame_logger=frame_logger)  # fmt: skip
    except (EcDriverError, OSError, ValueError) as exc:
        _report_error(exc)
        return 2
    with cell:
        results = cell.stop_all(broadcast=args.broadcast, return_exceptions=True)
    failed = {n: r for n, r in results.items() if isinstance(r, BaseException)}
    if args.json:
        json.dump({n: ({"error": f"{type(r).__name__}: {r}"} if n in failed else
                       {"stopped": True, "status": None if r is None else r.to_dict()}) for n, r in results.items()},
                  out, indent=2)  # fmt: skip
        print(file=out)
    else:
        for n, r in results.items():
            if n in failed:
                print(f"{n}: STOP FAILED: {type(r).__name__}: {r}", file=out)
            else:
                where = "" if r is None else f" at {r.position_mm:.2f} {cell.units_of(n)}"
                print(f"{n}: stopped{where}", file=out)
        print("STOP is a decelerate-stop, not a safety function: use the hardware emergency stop for safety.", file=out)
    return 2 if failed else 0


def _main_ports(args: argparse.Namespace, out: TextIO) -> int:
    if args.command == "ports":
        entries = list_ports()
        if args.json:
            json.dump(entries, out, indent=2)
            print(file=out)
        else:
            _print_ports(entries, out)
        return 0

    log_frame = _frame_sink() if args.log_frames else None
    targets: list[tuple[str, Any]]
    if args.sim:
        line = SimLine.with_slaves(args.sim_slaves, name="sim", profile=args.sim_profile, baudrate=args.sim_baud)
        targets = [("sim", line)]
    elif args.ports:
        targets = [(n.strip(), n.strip()) for n in args.ports.split(",") if n.strip()]
    else:
        names = [p["port"] for p in list_ports() if p["is_iai"]]
        if not names:
            print("error: no IAI USB adapter found; name the ports with --ports COM3,COM4", file=sys.stderr)
            return 2
        targets = [(n, n) for n in names]
    if args.command == "stop-all":
        return _stop_all(targets, args, out, log_frame)
    report: dict[str, Any] = {}
    found = 0
    for name, spec in targets:
        baud: int | None = None
        hits: list[ScanHit] = []
        error: str | None = None
        try:
            with Port(spec, name=name, baudrate=args.baud, frame_logger=log_frame) as port:
                hits = port.scan(args.slaves, timeout=args.scan_timeout)
                baud = port.baudrate
        except (EcDriverError, OSError, ValueError) as exc:
            error = f"{type(exc).__name__}: {exc}"
        found += len(hits)
        report[name] = {"baud": baud, "axes": [h.to_dict() for h in hits], "error": error}
        if not args.json:
            _print_scan(name, baud, hits, error, out)
    if args.json:
        json.dump(report, out, indent=2)
        print(file=out)
    errors = [r for r in report.values() if r["error"]]
    if errors and len(errors) == len(report):
        return 2
    return 0 if found else 1


@contextlib.contextmanager
def _break_as_interrupt() -> Iterator[None]:
    """Ctrl+Break (Windows: SIGBREAK) is handled as Ctrl+C while a command runs: ``KeyboardInterrupt``, the decel stop,
    exit 130. Before, it ended the process at once (exit 0xC000013A) with no decel STOP and a move ran on to its target
    (ON-WIRE on a PCON-CB). The previous handler comes back afterwards; nothing changes off the main thread."""
    sigbreak = getattr(signal, "SIGBREAK", None)
    if sigbreak is None or threading.current_thread() is not threading.main_thread():
        yield
        return
    on_int = signal.getsignal(signal.SIGINT)
    previous = signal.signal(sigbreak, on_int if callable(on_int) else signal.default_int_handler)
    try:
        yield
    finally:
        signal.signal(sigbreak, previous)


def main(argv: Sequence[str] | None = None, out: TextIO | None = None) -> int:
    """Entry point for ``python -m iai_actuator_driver`` / ``iai-actuator-driver``."""
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        with _break_as_interrupt():
            return _main_argv(argv, out)
    finally:
        _end_frame_lines()  # every frame line written before the command returns


def _main_argv(argv: list[str], out: TextIO | None) -> int:
    try:
        if _port_command(argv) is not None:
            port_args = build_port_parser().parse_args(argv)
            return _main_ports(port_args, out or sys.stdout)
        args = build_parser().parse_args(argv)
        return _main(args, out or sys.stdout)
    except KeyboardInterrupt:  # before the command opened its axis (scan, ports, stop-all, argument parsing)
        print("interrupted", file=sys.stderr)
        return 130


def check_main(argv: Sequence[str] | None = None, out: TextIO | None = None) -> int:
    """Entry point for ``iai-check`` (same as ``iai-actuator-driver ... check``)."""
    parser = argparse.ArgumentParser(prog="iai-check", description=f"{BANNER} - compatibility self-test")
    _add_connection_args(parser)
    _add_check_args(parser)
    args = parser.parse_args(argv)
    args.command, args.json = "check", False
    try:
        with _break_as_interrupt():
            return _main(args, out or sys.stdout)
    finally:
        _end_frame_lines()


def _main(args: argparse.Namespace, out: TextIO) -> int:
    if getattr(args, "axis", None) is not None:
        args.slave = args.axis + 1
    if not 1 <= args.slave <= 247:
        print(f"error: slave address must be 1..247 (axis 0..246), got {args.slave}", file=sys.stderr)
        return 2
    if args.command == "move" and (args.change_speed is not None or args.change_accel is not None) and (
        args.no_wait or args.decel is not None
    ):
        print("error: --change-speed / --change-accel need a plain move that waits (no --no-wait, no --decel)",
              file=sys.stderr)
        return 2
    if args.command == "recover" and args.rows is not None and not args.backup:
        print("error: --rows needs --backup (it chooses which rows of the file are compared)", file=sys.stderr)
        return 2
    if args.command == "backup":
        try:
            _rows_option(args.rows)
        except ValueError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    try:
        ec, label = _open(args)
    except (EcDriverError, OSError, ValueError) as exc:
        _report_error(exc)
        return 2
    try:
        with ec:
            return _execute(ec, label, args, out)
    except KeyboardInterrupt:
        print("interrupted (a decel stop was attempted if motion had been commanded)", file=sys.stderr)
        return 130
    except (EcDriverError, OSError) as exc:
        _report_error(exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
