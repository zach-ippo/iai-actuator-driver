"""What the two interactive subcommands (power-cycle, pio-watch) share: opening the unit, the read-only backup, remembering the
last state that was read, the hand-back message and the closing of the port."""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any

from . import report
from .python_edition import readonly_backup
from .report import CHECK, PASS, pos_str
from .session import ACCEL_G, open_axis


class Interactive:
    name = "interactive"

    def __init__(self, ctx: SimpleNamespace) -> None:
        self.ctx, self.args, self.say, self.iad = ctx, ctx.args, ctx.console.say, ctx.iad
        self.rec = report.Recorder(self.say)
        self.raw: Any = None
        self.sim: Any = None
        self.start: dict[str, Any] = {}
        self.band: tuple[float, float] | None = None
        self.geo: Any = None
        self.end_info: dict[str, Any] = {}
        self.compare: dict[str, Any] | None = None
        self.notes: list[str] = []
        self.extra: dict[str, Any] = {}
        self.backup_start: dict[str, Any] | None = None
        self.info: Any = None
        self.last: dict[str, Any] | None = None  # the last mode / servo state that was read: {where, t, pmss, servo_on, after_loss}
        self.after_loss = False  # the 24 V was lost since the first read
        self.unanswered: str | None = None
        self.opened = False
        self.t0 = time.perf_counter()

    # ------------------------------------------------------------------------------------------ steps both subcommands have
    def open_unit(self) -> Any:
        caps = {"max_speed_mm_s": self.args.speed_cap, "max_accel_g": ACCEL_G}
        self.raw, self.sim = open_axis(self.ctx, caps)
        self.opened = True
        self.info = self.raw.connect(take_control=False)  # reads only: PC control is neither taken nor dropped
        ident = {k: getattr(self.info, k, None) for k in ("model", "serial_number", "controller", "firmware", "max_positions")}
        self.ctx.unit = ident
        return (PASS if ident["model"] else CHECK), (f"{ident['model']} | serial {ident['serial_number']} | firmware {ident['firmware']} "
                                                     f"| baud {self.raw.baudrate}"), ident

    def backup_step(self) -> Any:
        self.backup_start, _, text, verdict = readonly_backup(self.raw, self.info, self.ctx.out)
        st = self.raw.status()
        self.remember("backup", st)
        return verdict, text

    # ------------------------------------------------------------------------------------------ bookkeeping
    def remember(self, where: str, st: Any) -> None:
        """Keep the last mode / servo state that was read, for the hand-back message."""
        self.last = {"where": where, "t": time.strftime("%H:%M:%S"), "pmss": bool(st.modbus_enabled), "servo_on": bool(st.servo_on),
                     "after_loss": self.after_loss}

    def stamp(self) -> str:
        return f"{time.strftime('%H:%M:%S')} +{time.perf_counter() - self.t0:7.2f} s"

    def hand_back(self, code: int, why: str = "") -> None:
        """Printed on every exit but 0 once the unit was opened: where the unit was last seen and how to take PC control back
        by hand. A unit that lost its 24 V comes up in PIO mode with the servo off and stays so until PC control is taken back."""
        say, last = self.say, self.last
        port = self.args.port or "<port>"
        say("-" * 72)
        say(f"WHERE THE UNIT IS LEFT (exit {code})" + (f": {why}" if why else ""))
        if self.unanswered:
            say(f"  the unit does not answer ({self.unanswered}): if its 24 V is off it comes up in PIO mode with the servo off")
        if last is None:
            say("  last known mode / servo : nothing was read")
        else:
            pc = "ON" if last["pmss"] else "OFF = PIO mode"
            sv = "ON" if last["servo_on"] else "OFF"
            when = "read after the 24 V loss was seen" if last["after_loss"] else "read before any 24 V loss was seen"
            say(f"  last known mode / servo : PC control {pc}, servo {sv}  ({last['where']} at {last['t']}; {when})")
        if self.after_loss and last is not None and not last["after_loss"]:
            say("  The 24 V was lost after that read: assume the unit comes up in PIO mode with the servo off.")
        say("  To take PC control back, only with the axis at rest (a power-cycled unit has its servo off; switching it on is a")
        say("  separate step):")
        say(f"      python -m iai_actuator_driver --port {port} mode pc")
        say(f"  and read the state with:  python -m iai_actuator_driver --port {port} status")
        say("-" * 72)

    def guarded(self, fn: Any) -> None:
        """An end stage: a Ctrl+C in it only repeats it (three times), any other error is printed and the next stage goes on."""
        for _ in range(3):
            try:
                fn()
                return
            except KeyboardInterrupt:
                self.say("Ctrl+C again: still finishing")
            except Exception as exc:  # noqa: BLE001
                self.say(f"end stage failed: {type(exc).__name__}: {exc}")
                return

    def status_at_exit(self) -> None:
        """One best-effort read-only status for the exit report."""
        if self.raw is None or self.last is None and not self.opened:
            return
        try:
            st = self.raw.status()
        except KeyboardInterrupt:
            self.unanswered = "the status read was interrupted by Ctrl+C"
            return
        except Exception as exc:  # noqa: BLE001
            self.unanswered = f"{type(exc).__name__}: {exc}"
            return
        self.unanswered = None
        self.remember("status at exit", st)
        self.say("last read-only status: " + pos_str(st))

    def close(self) -> None:
        if self.raw is not None:
            try:
                self.raw.close()
            except Exception as exc:  # noqa: BLE001
                self.say(f"close: {exc}")
            self.raw = None
