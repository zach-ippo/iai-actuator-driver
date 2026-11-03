"""What every subcommand shares: the built-in safety limits, console and log, the results folder, loading the driver,
opening the axis (real port or simulator) and the guard that keeps the harness from writing to the unit.

Nothing here can be switched off from the command line: the limits are constants, the guard has no override.
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import math
import os
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

# ---------------------------------------------------------------------------------------------------- safety limits
ACCEL_G = 0.1  # every move uses at most this acceleration; the driver also refuses anything above it
SPEED_CAP_DEFAULT = 20.0  # mm/s; --speed-cap sets it, never above SPEED_CAP_MAX
SPEED_CAP_MAX = 100.0
BAND_PCT_DEFAULT = 60.0  # the middle 60 % of the unit's stroke
BAND_PCT_RANGE = (10.0, 90.0)
POWER_UP_BAUD = 38400  # the rate a unit comes up at after a power cycle (the power-cycle test needs the link to run at it)

EXIT_OK, EXIT_FAIL, EXIT_REFUSED, EXIT_INTERRUPTED = 0, 1, 2, 130
EXIT_NO_LOSS, EXIT_NO_BACK, EXIT_RECOVER, EXIT_END = 3, 4, 5, 6  # the interactive subcommands

# the driver calls that write to the unit's non-volatile memory, change its baud rate or clear an alarm: the harness never
# makes them (the one same-value parameter write goes through GuardedAxis.same_value_write)
FORBIDDEN_CALLS = frozenset({"write_position_table", "write_parameter", "write_user_parameter", "teach_position",
                             "software_reset", "set_baudrate", "reset_alarm"})


class SetupError(Exception):
    """The run cannot start or must not go on (exit code 2). The unit has not been changed."""


class SafetyError(Exception):
    """A call the harness is not allowed to make."""


# ---------------------------------------------------------------------------------------------------- console
class Console:
    """Prints to the terminal and, when a folder is given, to ``run.log`` in it. Never raises: a closed pipe or a dead
    console must not stop a safety sequence that is printing."""

    def __init__(self, log_path: Path | None = None, stream: Any = None) -> None:
        self.stream = stream if stream is not None else sys.stdout
        self.file = open(log_path, "a", encoding="utf-8", errors="replace", buffering=1, newline="\n") if log_path else None

    def say(self, *parts: object, end: str = "\n") -> None:
        text = " ".join(str(p) for p in parts) + end
        if self.file is not None:
            try:
                self.file.write(text)
            except (OSError, ValueError):
                pass
        try:
            self.stream.write(text)
            self.stream.flush()
        except UnicodeEncodeError:
            try:
                self.stream.write(text.encode("ascii", "replace").decode("ascii"))
                self.stream.flush()
            except (OSError, ValueError):
                pass
        except (OSError, ValueError):
            pass

    def big(self, text: str, char: str = "#") -> None:
        """A line nobody can miss, between two bars."""
        bar = char * max(70, len(text) + 4)
        self.say()
        self.say(bar)
        self.say(text)
        self.say(bar)

    def close(self) -> None:
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None


class LogBridge(logging.Handler):
    """Sends the driver's log messages (warnings such as a speed below the unit's documented minimum, link events) to the
    console and the log file, and keeps them for the summary."""

    def __init__(self, console: Console) -> None:
        super().__init__(level=logging.WARNING)
        self.console = console
        self.messages: list[str] = []
        self.logger = logging.getLogger("iai_actuator_driver")
        self.previous_level = self.logger.level
        self.logger.addHandler(self)
        self.logger.setLevel(min(self.previous_level or logging.WARNING, logging.WARNING))

    def emit(self, record: logging.LogRecord) -> None:
        text = f"{record.levelname}: {record.getMessage()}"
        if text not in self.messages:
            self.messages.append(text)
        self.console.say(f"      [driver] {text}")

    def detach(self) -> None:
        self.logger.removeHandler(self)
        self.logger.setLevel(self.previous_level)


def has_terminal() -> bool:
    """A real terminal on stdin. On Windows ``isatty()`` is also true for the NUL device, so the handle must be a console."""
    try:
        if not sys.stdin or not sys.stdin.isatty():
            return False
        if os.name == "nt":
            import ctypes
            import msvcrt
            from ctypes import wintypes

            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            kernel32.GetConsoleMode.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
            kernel32.GetConsoleMode.restype = wintypes.BOOL
            handle = msvcrt.get_osfhandle(sys.stdin.fileno())
            return bool(kernel32.GetConsoleMode(wintypes.HANDLE(handle), ctypes.byref(wintypes.DWORD())))
        return True
    except Exception:  # noqa: BLE001
        return False


def require_confirmation_path(args: argparse.Namespace) -> None:
    """A run on a real port needs a person at a terminal or an explicit --yes; without either it ends before the port is
    opened. (--sim needs neither.)"""
    if not args.sim and not args.yes and not has_terminal():
        raise SetupError("this is a run on a real port and there is no terminal (stdin is not a TTY), so nobody could confirm "
                         "it. Run it in a terminal, or pass --yes once a person has checked that the axis may move. The "
                         "unit was not touched.")


def confirm(args: argparse.Namespace, console: Console, prompt: str) -> None:
    """Wait for Enter (Ctrl+C aborts). --yes continues without waiting. Without a terminal --sim continues, a real port
    never does."""
    if args.yes:
        console.say(f"{prompt}  [continuing: --yes]")
        return
    if not has_terminal():
        if not args.sim:
            raise SetupError("no terminal at the confirmation prompt and --yes was not given. The unit was not changed.")
        console.say(f"{prompt}  [continuing: --sim, no terminal]")
        return
    console.say()
    try:
        input(f"{prompt}\n>>> press Enter to continue (Ctrl+C aborts) ")
    except EOFError:
        if not args.sim:
            raise SetupError("stdin closed at the confirmation prompt. The unit was not changed.") from None
        console.say("(stdin closed: continuing, --sim)")


# ---------------------------------------------------------------------------------------------------- arguments
def finite_positive(text: str) -> float:
    """argparse type: a finite number above 0 (nan and inf would remove a timeout)."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not (math.isfinite(value) and value > 0.0):
        raise argparse.ArgumentTypeError(f"{text!r} is not allowed: the value must be a finite number above 0")
    return value


def finite_non_negative(text: str) -> float:
    try:
        value = float(text)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not (math.isfinite(value) and value >= 0.0):
        raise argparse.ArgumentTypeError(f"{text!r} is not allowed: the value must be a finite number, 0 or more")
    return value


def speed_cap(text: str) -> float:
    value = finite_positive(text)
    if value > SPEED_CAP_MAX:
        raise argparse.ArgumentTypeError(f"{text} mm/s is not allowed: the speed cap is at most {SPEED_CAP_MAX:g} mm/s")
    return value


def stroke_band(text: str) -> tuple[str, float, float]:
    """``60`` or ``60%``: the middle 60 % of the unit's stroke (10 to 90). ``2.0:7.5``: that range in mm (the driver
    intersects it with the unit's own stroke, so it can only narrow it)."""
    raw = text.strip()
    try:
        if ":" in raw:
            lo_text, hi_text = raw.split(":", 1)
            lo, hi = float(lo_text), float(hi_text)
            if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
                raise ValueError
            return ("mm", lo, hi)
        pct = float(raw.rstrip("%"))
        if not (BAND_PCT_RANGE[0] <= pct <= BAND_PCT_RANGE[1]):
            raise argparse.ArgumentTypeError(f"{text!r}: the band is {BAND_PCT_RANGE[0]:g} to {BAND_PCT_RANGE[1]:g} % of the stroke")
        return ("pct", pct, pct)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a band: use a percentage such as 60, or LOW:HIGH in mm such as 2.0:7.5") from None


def add_connection_arguments(parser: argparse.ArgumentParser) -> None:
    """The options every subcommand has."""
    parser.add_argument("--port", metavar="NAME", help="serial port of the unit, e.g. COM3 or /dev/ttyUSB0 (required unless --sim)")
    parser.add_argument("--slave", type=int, default=1, metavar="N", help="Modbus address of the unit: axis number + 1 (default 1)")
    parser.add_argument("--sim", action="store_true", help="rehearse on the driver's simulator: no serial port is opened")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="do not wait for Enter at the prompts. A run on a real port needs this or a terminal; without "
                             "either it ends before the port is opened")
    parser.add_argument("--speed-cap", type=speed_cap, default=SPEED_CAP_DEFAULT, metavar="MM_S",
                        help=f"fastest speed the run may command (default {SPEED_CAP_DEFAULT:g}, at most {SPEED_CAP_MAX:g}); the "
                             "driver also lowers it to the unit's own maximum")
    parser.add_argument("--stroke-band", type=stroke_band, default=("pct", BAND_PCT_DEFAULT, BAND_PCT_DEFAULT), metavar="BAND",
                        help="where the run may move: a percentage of the unit's stroke around its middle (default 60, "
                             f"{BAND_PCT_RANGE[0]:g} to {BAND_PCT_RANGE[1]:g}) or LOW:HIGH in mm")
    parser.add_argument("--results", default="hw-results", metavar="DIR",
                        help="folder for the results; each run makes a time-stamped folder inside it (default ./hw-results)")
    parser.add_argument("--src", metavar="DIR",
                        help="test the driver in this source tree (the repository root or its bindings/python folder) instead "
                             "of the installed iai_actuator_driver package")
    parser.add_argument("--native", metavar="DIR",
                        help="folder (or file) of the native core library; sets IAI_ACTUATOR_DRIVER_NATIVE for the Python driver")
    parser.add_argument("--backend", choices=("auto", "native"), default="auto",
                        help="Python driver backend (IAI_DRIVER_BACKEND): auto and native are both the native core (the "
                             "pure-Python fallback was removed after 1.0.x)")
    parser.add_argument("--sim-profile", default="gripper", metavar="NAME", help="--sim only: the simulated unit (default gripper)")
    parser.add_argument("--sim-baud", type=int, default=38400, metavar="RATE", help="--sim only: the simulated unit's baud rate (default 38400)")
    parser.add_argument("--sim-time-scale", type=finite_positive, default=1.0, metavar="N",
                        help="--sim only: the simulated unit runs N times faster than real time (default 1)")


def check_connection_arguments(args: argparse.Namespace) -> None:
    if not args.sim and not args.port:
        raise SetupError("give the serial port with --port (for example --port COM3), or rehearse with --sim")
    if args.sim and args.port:
        raise SetupError("--sim and --port exclude each other")
    if not 1 <= args.slave <= 247:
        raise SetupError(f"--slave {args.slave}: the Modbus address is 1 to 247")


# ---------------------------------------------------------------------------------------------------- results folder
def results_folder(base: str, sim: bool) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    root = Path(base)
    for n in range(100):  # two runs in the same second get -2, -3 ...
        out = root / (stamp + ("-sim" if sim else "") + (f"-{n + 1}" if n else ""))
        try:
            out.mkdir(parents=True, exist_ok=False)
            return out
        except FileExistsError:
            continue
        except OSError as exc:
            raise SetupError(f"cannot make the results folder {out}: {exc}") from exc
    raise SetupError(f"cannot make a results folder in {root}")


# ---------------------------------------------------------------------------------------------------- the driver
def find_source(path: str) -> Path:
    """The folder that holds the ``iai_actuator_driver`` package: ``path`` itself, or ``path/bindings/python``."""
    root = Path(path).expanduser()
    for candidate in (root, root / "bindings" / "python"):
        if (candidate / "iai_actuator_driver" / "__init__.py").is_file():
            return candidate.resolve()
    raise SetupError(f"--src {path}: no iai_actuator_driver package in it or in bindings/python below it")


def load_driver(src: str | None, native: str | None, backend: str) -> ModuleType:
    """Import ``iai_actuator_driver`` (the installed one, or the one in ``src``). The environment variables are read once
    by the package, so they are set before the import."""
    if native:
        os.environ["IAI_ACTUATOR_DRIVER_NATIVE"] = str(Path(native).expanduser())
    if backend != "auto":
        os.environ["IAI_DRIVER_BACKEND"] = backend
    folder = find_source(src) if src else None
    if folder is not None and str(folder) not in sys.path[:1]:
        sys.path.insert(0, str(folder))
    hint = ("Install it (pip install iai-actuator-driver, or a wheel file) or point --src at a source tree"
            if folder is None else "Check the folder and --native")
    try:
        import iai_actuator_driver as iad
    except ImportError as exc:
        raise SetupError(f"cannot import iai_actuator_driver: {exc}. {hint}.") from exc
    if getattr(iad, "__file__", None) is None:  # a namespace package: a folder of that name with no __init__.py
        where = ", ".join(str(p) for p in getattr(iad, "__path__", ()))
        raise SetupError(f"cannot import iai_actuator_driver: {where} is a folder with no __init__.py (for example a "
                         f"stray one that holds only __pycache__), not the driver. {hint}; remove the folder if it is stray.")
    here = Path(iad.__file__).resolve()
    if folder is not None and folder not in here.parents:
        raise SetupError(f"iai_actuator_driver was already imported from {here}, not from --src {folder}")
    if backend != "auto" and iad.BACKEND != backend:
        raise SetupError(f"the driver is on the {iad.BACKEND} backend, not {backend}: {iad.backend_info().get('reason')}")
    return iad


def open_axis(ctx: SimpleNamespace, caps: dict[str, Any]) -> tuple[Any, Any]:
    """``(axis, simulated unit or None)``, opened with the speed and acceleration caps (the stroke is set after the unit has
    been identified). With --sim no serial port is touched."""
    iad, args = ctx.iad, ctx.args
    if args.sim:
        from iai_actuator_driver.sim import SimulatedEC

        options: dict[str, Any] = {"profile": args.sim_profile, "baudrate": args.sim_baud, "port_name": "SIM",
                                   "time_scale": args.sim_time_scale, "slave": args.slave}
        if args.sim_profile == "gripper":
            options.update(stroke_mm=9.5, position_mm=3.0)
        options.update(getattr(ctx, "sim_options", {}))
        sim = SimulatedEC(**options)
        return iad.Axis.open(sim, poll_interval_s=0.02, slave=args.slave, **caps), sim
    return iad.Axis.open(args.port, slave=args.slave, **caps), None


class GuardedAxis:
    """The handle every step uses. A call that writes to the non-volatile memory, changes the baud rate or clears an alarm
    raises :class:`SafetyError`; everything else goes to the driver. ``raw`` is the driver's own axis, for the functions
    that take one (``unit_info``, ``backup``, ``run_check``)."""

    def __init__(self, axis: Any) -> None:
        object.__setattr__(self, "raw", axis)

    def __getattr__(self, name: str) -> Any:
        if name in FORBIDDEN_CALLS:
            raise SafetyError(f"{name}() is not allowed: the harness never writes parameters or position rows, changes the "
                              "baud rate or clears an alarm")
        return getattr(self.raw, name)

    def __setattr__(self, name: str, value: Any) -> None:
        setattr(self.raw, name, value)

    def same_value_write(self, key: Any) -> tuple[dict[str, Any], Any, int]:
        """Write a parameter its own current value, once. The driver must skip it (nothing written, no commit); the caller
        judges the result. Returns ``(the parameter as read, the write result, non-volatile writes the call used)``."""
        axis = self.raw
        before = axis.nonvolatile_writes
        current = axis.read_parameter(key)
        result = axis.write_parameter(key, current["value"])
        return current, result, axis.nonvolatile_writes - before
