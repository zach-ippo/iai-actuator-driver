"""Power loss and link loss: notice it, bring the axis back, then decide what to do next.

While the link is up, ``ax.track()`` remembers the session: identity, baud rate, PC control, safety speed, HEND,
uptime, alarm history and (here) a copy of the stored rows. ``ax.check_link()`` compares one status read with it, and a
``recovery.LinkMonitor`` does the same in the background. After a fault, ``ax.recover()`` with a ``RecoveryPolicy``
waits for the unit, finds its baud rate, requires the same unit (serial number, model), restores the baud rate, PC
control and safety speed, and reports HEND, alarm, position and the stored rows. It never moves the axis: what happens
next is the application's decision (here: servo ON if needed, home if HEND was lost, then the work position again).

Run it from the project folder::

    python examples/python/recovery_after_power_loss.py --sim                     # a simulated 24 V power loss
    python examples/python/recovery_after_power_loss.py --sim --fault unplug      # a simulated pulled USB adapter
    python examples/python/recovery_after_power_loss.py --sim --fault break       # a line break: the unit drops to 9600
    python examples/python/recovery_after_power_loss.py --sim --sim-profile rod   # an incremental unit loses its home
    python examples/python/recovery_after_power_loss.py --port COM3               # then switch the unit's 24 V off and on

On hardware the script moves the axis to its work position and watches the link for ``--watch`` seconds (default 60):
switch the unit's 24 V supply off and on again while the axis is at rest, and only on a horizontal axis or one with a
brake (a vertical axis without a brake can drop when the power goes). Recovery is VERIFIED on the EC-GRBP8M gripper
after a real power cycle (in a fresh process) and after a line break, and on the PCON-CB after a line break. A power
cycle seen live by a running process, as here, is VERIFIED on the gripper too (2026-10-30: 24 V off for 3.2 s, PC
control back in 1.44 s, the servo stayed on, HEND, the rows and the parameters unchanged).

Motion: to the middle of the stroke at 20 mm/s (raised to the unit's documented minimum speed where that is higher;
``--speed`` sets it), unloaded. At the end the servo is switched off. On Ctrl+C or an error the axis gets a decel STOP
and the servo is left as it is. The driver is not a safety function: keep the hardware emergency stop within reach.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2] / "bindings" / "python"
if (ROOT / "iai_actuator_driver").is_dir() and str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # a source checkout: use the driver in bindings/python

from iai_actuator_driver import (  # noqa: E402
    Axis,
    EcDriverError,
    EcNotReadyError,
    RecoveryEvent,
    RecoveryPolicy,
    RecoveryReport,
    SimLine,
    sim_line,
)
from iai_actuator_driver.recovery import LinkCheck, LinkMonitor, Session  # noqa: E402
from iai_actuator_driver.sim import PROFILES  # noqa: E402

FIRST_RUN_SPEED = 20.0  # units/s (mm/s, deg/s on a rotary axis) when --speed is not given
SIM_BAUDRATE = 38400  # the simulated line runs at the rate the units power up at
FAULTS = {"power": "a 24 V power loss", "unplug": "a pulled USB adapter", "break": "a line break"}
FAULT_AFTER_S = 1.0  # simulator: the fault starts this long after the link monitor ...
FAULT_LASTS_S = 1.5  # ... and ends this long after it started (power back on, adapter plugged back in)
LINE_BREAK_S = 0.25  # a line break of 150 ms or more drops the unit to 9600 baud (DOC)


def work_position(ax: Axis) -> float:
    """The middle of the unit's own position range."""
    position = ax.limits()["position"]
    if position["min"] is None or position["max"] is None:
        raise EcNotReadyError("the unit's position range is not known (no soft limits read): no work position")
    return round((position["min"] + position["max"]) / 2, 2)


def first_run_speed(ax: Axis, requested: float | None) -> float:
    """``requested``, else 20 units/s raised to the unit's documented minimum speed (below it the unit may run
    unsteadily; the driver warns, never refuses) and kept under the unit's maximum."""
    if requested is not None:
        return requested
    speed = ax.limits()["speed"]
    return min(max(FIRST_RUN_SPEED, speed["recommended_min"] or 0.0), speed["max"])


def go_to_work(ax: Axis, target: float, speed: float) -> None:
    """Servo ON and home only when needed, then the work position (the move records its target for recover())."""
    st = ax.status()
    if not st.servo_on:
        print("servo ON")
        ax.servo(True)
    if not st.homed:
        print("home: the unit reports no home (HEND); it runs to its mechanical end")
        ax.home()
    ax.move_to(target, speed_mm_s=speed)
    print(f"at the work position {target:.2f} {ax.units}")


def on_off(value: bool | None) -> str:
    return "unknown" if value is None else ("on" if value else "off")


def show_session(session: Session) -> None:
    """What track() remembered."""
    uptime = "unknown" if session.uptime_ms is None else f"{session.uptime_ms / 1000:.1f} s"
    target = "none" if session.last_target_mm is None else f"{session.last_target_mm:.2f}"
    rows = 0 if session.rows is None else len(session.rows)
    print(f"track(): serial {session.serial_number}, {session.baudrate or 'no'} baud, PC control "
          f"{on_off(session.modbus_control)}, safety speed {on_off(session.safety_speed)}, servo "
          f"{on_off(session.servo_on)}, HEND {on_off(session.homed)}, uptime {uptime}, last target {target}, "
          f"{rows} stored rows copied")


class SimulatedFault:
    """Simulator only. :meth:`start` begins the fault ``kind`` FAULT_AFTER_S later in a timer thread and ends it
    FAULT_LASTS_S after that (a line break needs no end: the unit stays at 9600 baud); :meth:`cancel` drops what has
    not happened yet."""

    def __init__(self, line: SimLine, slave: int, ax: Axis, kind: str) -> None:
        self.line = line
        self.unit = line[slave]
        self.ax = ax
        self.kind = kind
        self._timers: list[threading.Timer] = []

    def start(self) -> None:
        if self.kind == "power":
            self._later(FAULT_AFTER_S, "24 V off", self.unit.power_off)
            self._later(FAULT_AFTER_S + FAULT_LASTS_S, "24 V on", self.unit.power_on)
        elif self.kind == "unplug":
            self._later(FAULT_AFTER_S, f"USB adapter pulled: port {self.line.name} is gone", self.line.unplug)
            self._later(FAULT_AFTER_S + FAULT_LASTS_S, "USB adapter plugged back in", self.line.replug)
        else:
            self._later(FAULT_AFTER_S, f"line break of {LINE_BREAK_S:g} s: the unit drops to 9600 baud",
                        lambda: self.ax.bus.send_break(LINE_BREAK_S))  # fmt: skip

    def cancel(self) -> None:
        for timer in self._timers:
            timer.cancel()

    def _later(self, delay: float, text: str, action: Callable[[], None]) -> None:
        def act() -> None:
            print(f"  [simulator] {text}")
            action()

        timer = threading.Timer(delay, act)
        timer.daemon = True
        self._timers.append(timer)
        timer.start()


def watch_link(
    ax: Axis, name: str, seconds: float, interval_s: float, start_fault: Callable[[], None] | None
) -> LinkCheck | None:
    """Run a LinkMonitor (check_link every ``interval_s`` in a background thread) until it reports a fault or
    ``seconds`` pass. Returns the check that reported the fault, or None. The monitor only watches: it is stopped
    before recover() works on the link."""
    faults: list[LinkCheck] = []
    seen = threading.Event()

    def on_change(axis_name: str, check: LinkCheck) -> None:  # runs in the monitor's thread: keep it short
        reasons = f": {'; '.join(check.reasons)}" if check.reasons else ""
        print(f"  link monitor: {axis_name} {check.state}{reasons}")
        if not check.ok:
            faults.append(check)
            seen.set()

    with LinkMonitor({name: ax}, interval_s=interval_s, on_change=on_change):
        if start_fault is not None:
            start_fault()
        deadline = time.perf_counter() + seconds
        while not seen.is_set() and time.perf_counter() < deadline:
            seen.wait(0.2)  # short slices, so Ctrl+C is seen at once
    return faults[0] if faults else None


def print_event(event: RecoveryEvent) -> None:
    print(f"  event: {event.kind} ({event.cause}): {event.message}")


def show_report(report: RecoveryReport) -> None:
    """The RecoveryReport in words."""
    print(f"recover(): {report.summary()} ({report.elapsed_s:.1f} s)")
    print(f"  cause {report.cause}; unit restarted: {'yes' if report.restarted else 'no'}; "
          f"same unit: {'yes' if report.same_unit else 'no'}")
    for sign in report.restart_signs:
        print(f"  restart sign: {sign}")
    print(f"  baud rate found {report.baudrate_found or '-'}, now {report.baudrate or '-'}")
    for action in report.actions:
        print(f"  done: {action}")
    for note in dict.fromkeys(report.notes):  # each distinct note once
        print(f"  note: {note}")
    if report.alarm_code:
        print(f"  alarm 0x{report.alarm_code:03X} {report.alarm_name} ({report.alarm_level} level)")
    position = "unknown" if report.position_mm is None else f"{report.position_mm:.2f}"
    target = "none" if report.last_target_mm is None else f"{report.last_target_mm:.2f}"
    print(f"  HEND {'lost' if report.needs_home else 'kept'}, servo {on_off(report.servo_on)}, position {position}, "
          f"last target {target}, at the target: {report.at_target}")
    print(f"  {len(report.rows_checked)} stored rows checked: {'; '.join(report.row_problems) or 'no difference'}")
    print(f"  needs attention: {report.needs_attention}; ready to move: {report.ready_to_move}")


def carry_on(ax: Axis, report: RecoveryReport, target: float, speed: float) -> bool:
    """The application's decision after recover(), which never moves the axis. Returns False when the operator has to
    act first (a fault alarm)."""
    if report.fault_alarm:
        print(f"alarm 0x{report.alarm_code:03X} ({report.alarm_name}) is active: left for the operator, no motion")
        return False
    if report.status is not None and report.status.moving:
        print("the axis is still running the move it had before the fault: wait for it")
        ax.wait_idle(timeout=30.0)
    if not ax.status().servo_on:
        print("servo ON")
        ax.servo(True)
    if report.needs_home:
        print("home: HEND was lost (an incremental unit loses it at a power loss); the axis runs to its mechanical end")
        ax.home()
    if report.needs_home or report.at_target is not True:
        print(f"move_to {target:.2f} {ax.units}: the work position again (recover() never resumes a move)")
        ax.move_to(target, speed_mm_s=speed)
    else:
        print(f"no move needed: the axis is at its last target ({report.position_mm:.2f} {ax.units})")
    return True


def run(ax: Axis, args: argparse.Namespace, line: SimLine | None) -> bool:
    """The whole example on an open axis. Returns False when it ends on a fault alarm left for the operator."""
    info = ax.connect()  # identity and the unit's own ranges; PC control is taken while the axis is at rest
    name = f"{args.port if line is None else line.name}:{args.slave}"
    print(f"{name}: {info.model} ({info.product_family}, {ax.kind}), serial {info.serial_number}, "
          f"{ax.baudrate or 'no'} baud")
    target = work_position(ax)
    speed = first_run_speed(ax, args.speed)
    go_to_work(ax, target, speed)

    show_session(ax.track(name=name, rows=True))  # read-only; the baseline check_link() and recover() compare with
    check = ax.check_link()
    print(f"check_link(): {check.state} ({check.elapsed_s * 1000:.0f} ms, one status read)")

    simulated = None if line is None else SimulatedFault(line, args.slave, ax, args.fault)
    if simulated is not None:
        print(f"\nLinkMonitor every {args.interval:g} s; the simulator starts {FAULTS[args.fault]} in "
              f"{FAULT_AFTER_S:g} s")
    else:
        print(f"\nLinkMonitor every {args.interval:g} s for up to {args.watch:g} s: switch the unit's 24 V off and on "
              "again now, with the axis at rest (Ctrl+C ends)")
    try:
        fault = watch_link(ax, name, args.watch, args.interval, None if simulated is None else simulated.start)
        if fault is None:
            print(f"no link fault within {args.watch:g} s")
        else:
            policy = RecoveryPolicy(check_rows=True)  # also compare the stored rows with the copy track() made
            print(f"\nrecover(): waiting up to {args.wait:g} s for the unit")
            report = ax.recover(timeout=args.wait, policy=policy, on_event=print_event)
            show_report(report)
            if not carry_on(ax, report, target, speed):
                return False
            check = ax.check_link()
            print(f"check_link(): {check.state} (against the session recover() renewed)")
    finally:
        if simulated is not None:
            simulated.cancel()  # what has not happened yet (Ctrl+C, an error, a short --watch)
    print("servo OFF")
    ax.servo(False)
    return True


def stop_axis(ax: Axis) -> None:
    """Decel STOP (Ctrl+C or an error). A STOP that cannot be sent is reported: the unit has no communication
    watchdog, so a move it is running then runs to its end."""
    try:
        ax.stop()
    except (EcDriverError, OSError) as exc:
        print(f"decel STOP not sent: {type(exc).__name__}: {exc}")
    else:
        print("decel STOP sent; the servo is left as it is")


def report_error(exc: BaseException) -> None:
    sys.stdout.flush()  # keep the order when both streams go to one file
    print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
    for note in getattr(exc, "__notes__", []):
        print(f"       {note}", file=sys.stderr)


def positive_float(text: str) -> float:
    value = float(text)
    if not value > 0:
        raise argparse.ArgumentTypeError(f"must be more than 0, got {text}")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="iai-actuator-driver example: recovery after a power or link loss (unofficial; not an IAI product)"
    )
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", metavar="COMx", help="serial port of the unit, e.g. COM3 (close IA-OS first)")
    where.add_argument("--sim", action="store_true", help="use the simulator instead of hardware")
    parser.add_argument("--slave", type=int, default=1, help="Modbus address = axis number + 1 (default 1)")
    parser.add_argument(
        "--speed", type=float, default=None,
        help="mm/s (deg/s on a rotary axis); default 20, raised to the unit's documented minimum where that is higher",
    )  # fmt: skip
    parser.add_argument(
        "--interval", type=positive_float, default=0.5, metavar="S",
        help="seconds between two link checks (default 0.5)",
    )  # fmt: skip
    parser.add_argument(
        "--watch", type=float, default=60.0, metavar="S", help="seconds to watch for a fault (default 60)"
    )
    parser.add_argument(
        "--wait", type=float, default=60.0, metavar="S",
        help="seconds recover() waits for the port and the unit (default 60)",
    )  # fmt: skip
    faults = "; ".join(f"{kind}: {text}" for kind, text in FAULTS.items())
    parser.add_argument(
        "--fault", choices=sorted(FAULTS), default="power",
        help=f"with --sim: the simulated fault, {faults} (default power)",
    )  # fmt: skip
    parser.add_argument(
        "--sim-profile", choices=sorted(PROFILES), default="gripper",
        help="with --sim: the simulated unit (default gripper, the unit recovery was verified on)",
    )  # fmt: skip
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="  [driver %(levelname)s] %(message)s", stream=sys.stdout)
    if sys.platform == "win32":  # Ctrl+Break ends the program like Ctrl+C
        signal.signal(signal.SIGBREAK, signal.default_int_handler)
    logging.getLogger("iai_actuator_driver.recovery").setLevel(logging.ERROR)  # its events are printed here (print_event)
    line: SimLine | None = None
    port: Any = args.port
    if args.sim:
        # The EC-GRBP8M gripper takes the 38400 baud code (VERIFIED); the simulator's default ignores it, as the
        # EC-R6S rod did. It matters after a line break, which drops the unit to 9600 baud.
        ignored = () if args.sim_profile == "gripper" else (0x06,)
        line = sim_line([args.slave], name="SIM", profile=args.sim_profile, baudrate=SIM_BAUDRATE,
                        ignored_baud_codes=ignored)  # fmt: skip
        port = line
    try:
        with Axis.open(port, slave=args.slave) as ax:  # the baud rate is found by itself
            try:
                ok = run(ax, args, line)
            except BaseException:
                stop_axis(ax)  # STOP first; the axis does it again when the with block ends on the error
                raise
    except KeyboardInterrupt:
        sys.stdout.flush()
        print("interrupted", file=sys.stderr)
        return 130
    except (EcDriverError, OSError, ValueError) as exc:
        report_error(exc)
        return 2
    print("done" if ok else "stopped: an alarm needs the operator")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
