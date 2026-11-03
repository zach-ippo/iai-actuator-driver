"""power-cycle: you switch the unit's 24 V supply off and on by hand while the harness watches; then the driver recovers the unit.

    python tools/hw/run.py power-cycle --port COM3 [--yes]
    python tools/hw/run.py power-cycle --sim [--sim-scenario power|lose-home|unplug|never|no-return|interrupt|blip|alarm|rows-differ]

Steps: identify and a read-only backup -> the start state must be clean (servo on, homed, PC mode, no alarm, not moving, link at
the power-up baud rate 38400) -> track() for a recovery session (rows and parameters copied, read only) -> "READY: switch the 24 V
OFF now, wait about 5 s, then ON" -> check_link() is polled and every change is printed with a time stamp, with whether the port
stays listed -> "POWER OFF SEEN" -> "POWER BACK" -> recover() with a policy that writes nothing non-volatile (the baud rate and PC
control are restored, rows and parameters are only compared) -> servo on if it is off -> home only if the unit lost its home, at
the unit's own home speed -> back to the start position at --return-speed (never above 5 mm/s or the speed cap) -> the end state is
checked (servo, home, PC mode, baud rate, no alarm, start position, no non-volatile write).

Safety: nothing is written to the unit before the power is back; the recovery never writes a parameter or a position row; a recovery
that finds another unit, a fault alarm or changed rows or parameters stops before servo, home and the return move; the return
move is refused (no move) when the start position is outside the test band. On every exit but 0 the harness prints where the unit
was last seen and how to take PC control back by hand: a unit that lost its 24 V comes up in PIO mode with the servo off.

Exit codes: 0 ok, 2 refused or could not start, 3 no power loss was seen, 4 the power did not come back, 5 the recovery failed or
found a different unit / an alarm / changed rows or parameters, 6 a later step failed or the end state is not as expected, 130 Ctrl+C.
"""

from __future__ import annotations

import _thread
import argparse
import sys
import threading
import time
import traceback
from collections.abc import Sequence
from types import SimpleNamespace
from typing import Any

from .common import Geometry, band_from_stroke, near
from .interactive import Interactive
from .report import CHECK, FAIL, PASS, Abort, Check, Skip, pos_str
from .session import (ACCEL_G, EXIT_END, EXIT_INTERRUPTED, EXIT_NO_BACK, EXIT_NO_LOSS, EXIT_RECOVER, EXIT_REFUSED, POWER_UP_BAUD, SetupError,
                      add_connection_arguments, confirm, finite_non_negative, finite_positive)
from .summary import finish

LOST = ("unit_silent", "port_vanished")
SCENARIOS = ("power", "lose-home", "unplug", "never", "no-return", "interrupt", "blip", "alarm", "rows-differ")
RETURN_SPEED_MAX = 5.0


def return_speed(text: str) -> float:
    value = finite_positive(text)
    if value > RETURN_SPEED_MAX:
        raise argparse.ArgumentTypeError(f"{text} mm/s is not allowed: the return speed is at most {RETURN_SPEED_MAX:g} mm/s")
    return value


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tools/hw/run.py power-cycle", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_connection_arguments(p)
    p.set_defaults(sim_time_scale=10.0)
    p.add_argument("--loss-timeout", type=finite_positive, default=120.0, metavar="S", help="give up (exit 3) if no power loss is seen within S s of READY (default 120)")
    p.add_argument("--back-timeout", type=finite_positive, default=120.0, metavar="S", help="give up (exit 4) if the power is not back S s after it was lost (default 120)")
    p.add_argument("--poll", type=finite_positive, default=0.25, metavar="S", help="check_link interval (default 0.25 s)")
    p.add_argument("--settle", type=finite_non_negative, default=1.0, metavar="S", help="wait this long after the unit answers again before recover() (default 1 s)")
    p.add_argument("--recover-timeout", type=finite_positive, default=60.0, metavar="S", help="recover() timeout (default 60 s)")
    p.add_argument("--home-timeout", type=finite_positive, default=300.0, metavar="S", help="timeout of the home return (default 300 s)")
    p.add_argument("--return-speed", type=return_speed, default=RETURN_SPEED_MAX, metavar="MM_S",
                   help=f"speed of the move back to the start position, at most {RETURN_SPEED_MAX:g} mm/s and the speed cap (default {RETURN_SPEED_MAX:g})")
    p.add_argument("--sim-scenario", choices=SCENARIOS, default="power", help="--sim only: what the simulated person at the 24 V switch does (default power)")
    return p


class SimOwner(threading.Thread):
    """--sim only: plays the person at the 24 V switch once the READY line is out."""

    def __init__(self, sim: Any, scenario: str, say: Any) -> None:
        super().__init__(daemon=True, name="sim-owner")
        self.sim, self.scenario, self.say = sim, scenario, say
        self.ready = threading.Event()

    def run(self) -> None:
        sc, sim, say = self.scenario, self.sim, self.say
        self.ready.wait()
        if sc == "never":
            return
        time.sleep(1.5)
        if sc == "blip":
            say("   [sim] the 24 V goes off and on within one poll")
            sim.power_cycle()
            return
        say("   [sim] the 24 V goes OFF" + (" (the USB adapter goes with it)" if sc == "unplug" else ""))
        sim.power_off()
        if sc == "unplug":
            sim.unplug()
        if sc == "no-return":
            return
        if sc == "interrupt":
            time.sleep(1.5)
            say("   [sim] Ctrl+C is pressed")
            _thread.interrupt_main()
            return
        time.sleep(3.0)
        say("   [sim] the 24 V comes back ON")
        if sc == "unplug":
            sim.replug()
            time.sleep(1.0)
        sim.power_on(lose_home=True if sc == "lose-home" else None)
        if sc == "alarm":
            say("   [sim] (the unit comes back with a fault alarm)")
            sim.trigger_alarm(0x0A1)
        if sc == "rows-differ":
            say("   [sim] (the unit comes back with position row 1 changed)")
            words = list(sim.position_row(1))
            words[1] = (words[1] + 500) & 0xFFFF
            sim.set_position_row(1, words)


class PowerCycle(Interactive):
    name = "power-cycle"

    def __init__(self, ctx: SimpleNamespace) -> None:
        super().__init__(ctx)
        self.recovery: Any = None
        self.recover_failed = False
        self.ready_seen = False
        self.return_refused = False
        self.marks: dict[str, float] = {}
        self.wait_result: dict[str, Any] | None = None
        self.owner: SimOwner | None = None
        self.rc: Any = None

    def run(self) -> int:
        code = EXIT_REFUSED
        try:
            code = self._flow()
        except KeyboardInterrupt:
            code = EXIT_INTERRUPTED
            self.ctx.console.big("Ctrl+C: stopped; reading the unit once, then the hand-back", "!")
            self._interrupted()
        except SetupError as exc:
            self.say(f"REFUSED: {exc}")
            code = EXIT_REFUSED
        except Exception as exc:  # noqa: BLE001
            self.say(f"UNEXPECTED {type(exc).__name__}: {exc}")
            self.say(traceback.format_exc())
            code = EXIT_REFUSED
        finally:
            if self.raw is not None:
                if code != 0:
                    self.guarded(self.status_at_exit)
                    self.guarded(lambda: self.hand_back(code))
            self.guarded(self.close)
        if self.marks:
            self.say("TIMELINE (since READY): " + ", ".join(f"{k} +{v:.1f} s" for k, v in self.marks.items()))
        self.extra.update({"timeline_s": self.marks, "state_changes": (self.wait_result or {}).get("changes"), "recovery": self.recovery})
        self.guarded(lambda: finish(self.ctx, self, code))
        return code

    def _interrupted(self) -> None:
        raw = self.raw
        if raw is None:
            return
        try:
            if getattr(raw, "motion_pending", False):
                self.say("an axis move was running: sending the decel stop")
                raw.stop(timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            self.say(f"stop on Ctrl+C failed: {exc}")

    # ------------------------------------------------------------------------------------------ the flow
    def _flow(self) -> int:
        rec, args = self.rec, self.args
        from iai_actuator_driver import recovery as rc

        self.rc = rc
        if args.return_speed > min(RETURN_SPEED_MAX, args.speed_cap):
            raise SetupError(f"--return-speed {args.return_speed:g} is above the speed cap {args.speed_cap:g}")
        scenario = args.sim_scenario if args.sim else "power"
        self.ctx.sim_options = {"cold_boot": True} if scenario == "lose-home" else {}
        rec.step("open", "open the port and identify the unit (read only)", self._open, critical=True)
        if self.raw is None:
            return EXIT_REFUSED
        rec.step("backup", "read-only backup and state", self.backup_step, critical=True)
        rec.step("preflight", "start state as the test needs it", self._preflight, critical=True)
        rec.step("track", "track(): recovery session, rows and parameters copied (read only)", self._track, critical=True)
        if rec.aborted:
            self.say(f"Stopped before the power test: {rec.aborted}. Nothing was written to the unit.")
            return EXIT_REFUSED
        confirm(args, self.ctx.console, "Everything is read and tracked. Be at the 24 V supply; nothing is written to the unit until the power is back.")
        if args.sim:
            self.owner = SimOwner(self.sim, scenario, self.say)
            self.owner.start()
        self.t0 = time.perf_counter()
        self.ready_seen = True
        self.ctx.console.big("READY: switch the 24 V OFF now, wait about 5 s, then ON")
        self.say(f"(polling check_link every {args.poll:g} s; gives up after {args.loss_timeout:.0f} s without a power loss; Ctrl+C stops cleanly)")
        if self.owner is not None:
            self.owner.ready.set()
        rec.step("wait", "poll check_link: power off seen, power back", self._wait)
        if self.wait_result is None or self.wait_result["result"] == "no_loss":
            return EXIT_NO_LOSS
        if self.wait_result["result"] == "no_back":
            return EXIT_NO_BACK
        rec.step("recover", "recover(): restore baud rate and PC control, assess", self._recover, critical=True)
        if rec.aborted:
            return EXIT_RECOVER
        rec.step("servo", "servo back on (if it is off)", self._servo, motion=True, critical=True)
        rec.step("home", "home only if the unit lost its home", self._home, motion=True, critical=True)
        rec.step("return", f"back to the start position at {args.return_speed:g} mm/s (if not there)", self._return, motion=True, critical=True)
        failed = rec.aborted
        rec.aborted = None  # the end check runs after a failed step too
        rec.step("end", "end state: servo, home, PC mode, baud rate, no alarm, start position", self._end, always=True)
        rec.aborted = failed or rec.aborted
        return EXIT_END if any(r["verdict"] == FAIL for r in rec.rows) else 0

    def _open(self) -> Any:
        verdict, text, ident = self.open_unit()
        if self.sim is not None and self.ctx.sim_options.get("cold_boot"):
            self.sim.servo_on = self.sim.homed = self.sim.pmsl = True  # a cold boot is for the power cycle, not for the start
        return verdict, text, ident

    def _preflight(self) -> Any:
        raw, args = self.raw, self.args
        st = raw.status()
        self.remember("preflight", st)
        problems = []
        for ok, text in ((st.servo_on, "servo is off"), (st.homed, "not homed"), (st.modbus_enabled, "not in PC mode"), (not st.alarm, f"alarm 0x{st.alarm_code:03X}"),
                         (not st.moving, "the axis is moving")):
            if not ok:
                problems.append(text)
        if raw.baudrate != POWER_UP_BAUD:
            problems.append(f"the link runs at {raw.baudrate} baud, not {POWER_UP_BAUD}: a unit comes up at {POWER_UP_BAUD} after a power cycle, so a link at "
                            "another rate could not be seen coming back")
        self.start = {"position_mm": st.position_mm, "baudrate": raw.baudrate, "servo_on": st.servo_on, "homed": st.homed}
        self.band = band_from_stroke(args.stroke_band, raw.ec_stroke)
        if self.band is not None:
            raw.stroke = self.band
        eff = raw.effective_stroke
        if eff is not None:
            self.geo = Geometry(eff[0], eff[1], min(args.speed_cap, raw.speed_cap_mm_s), raw.accel_cap_g)
        text = pos_str(st) + f" baud {raw.baudrate}"
        if problems:
            raise Abort("the start state is not what this test needs: " + "; ".join(problems) + " | " + text)
        if eff is None or not (eff[0] - 0.05 <= st.position_mm <= eff[1] + 0.05):
            self.return_refused = True
            return CHECK, text + f" | the start position is outside the test band {eff}: the move back at the end will be refused, no move"
        return PASS, text + f" | as needed; test band {eff[0]:.2f} .. {eff[1]:.2f} mm"

    def _track(self) -> Any:
        sess = self.raw.track(name="unit", rows=True, parameters=True)  # read only
        chk = self.raw.check_link()

        def g(key: str) -> Any:
            return getattr(sess, key, None)

        text = (f"session: serial {g('serial_number')}, baud {g('baudrate')}, PC control {g('modbus_control')}, servo {g('servo_on')}, home {g('homed')}; "
                f"{len(g('rows') or {})} rows and {len(g('parameters') or {})} parameter items copied; check_link -> {chk.state}")
        if chk.state != "ok":
            raise Abort(text + " | the link is not ok: the power test is not started")
        return PASS, text

    # ------------------------------------------------------------------------------------------ waiting for the power cycle
    def _wait(self) -> Any:
        res = self._poll_power()
        self.wait_result = res
        self.marks.update(res["marks"])
        if res["result"] == "no_loss":
            return FAIL, f"no power loss seen within {self.args.loss_timeout:.0f} s ({res['polls']} polls); gave up, the unit was not touched", res
        if res["result"] == "no_back":
            return FAIL, f"the power did not come back within {self.args.back_timeout:.0f} s of the loss ({res['polls']} polls); gave up, the unit was not touched", res
        m = res["marks"]
        return PASS, f"power lost at +{m['off_seen']:.1f} s, back at +{m.get('back', m.get('port_back', 0)):.1f} s ({res['polls']} polls, {len(res['changes'])} state changes)", res

    def _poll_power(self) -> dict[str, Any]:
        args, ax, rc, say, big = self.args, self.raw, self.rc, self.say, self.ctx.console.big
        marks: dict[str, float] = {}
        changes: list[dict[str, Any]] = []
        last_key = None
        phase, streak, polls = "off", 0, 0
        t_lost = None
        via_port = False
        next_beat = 10.0
        yn = lambda v: "?" if v is None else "yes" if v else "no"  # noqa: E731
        while True:
            tick = time.perf_counter()
            try:
                chk = ax.check_link()
                state, answered, reason, took = chk.state, chk.status is not None, "; ".join(chk.reasons), chk.elapsed_s
                if chk.status is not None:
                    self.remember("check_link poll", chk.status)
            except Exception as exc:  # noqa: BLE001 - a half-booted unit answering oddly: not a loss, not yet back
                state, answered, reason, took = f"error:{type(exc).__name__}", False, str(exc), 0.0
            polls += 1
            try:
                listed = rc.port_present(ax)
            except Exception:  # noqa: BLE001
                listed = None
            key = (state, answered, listed)
            if key != last_key:
                say(f"[{self.stamp()}] link {last_key[0] if last_key else '-'} -> {state} | answers: {yn(answered)} | port listed: {yn(listed)} | check took {took:.2f} s"
                    + (f" | {reason}" if reason else ""))
                changes.append({"t": round(time.perf_counter() - self.t0, 3), "wall": time.strftime("%H:%M:%S"), "state": state, "answers": answered,
                                "port_listed": listed, "reason": reason})
                last_key = key
            now = time.perf_counter() - self.t0
            if phase == "off":
                if state in LOST or state == "unit_restarted":
                    self.after_loss = True
                    marks["off_seen"] = now
                    t_lost, phase = time.perf_counter(), "back"
                    if state == "unit_restarted":
                        big("POWER OFF SEEN (restart signs only: the outage was shorter than one poll)")
                        break
                    big("POWER OFF SEEN")
                    say(f"[{self.stamp()}] link {state}, port listed: {yn(listed)}. Now wait about 5 s and switch the 24 V back ON.")
                    next_beat = now + 5.0
                elif now > args.loss_timeout:
                    return {"result": "no_loss", "marks": marks, "changes": changes, "polls": polls}
                elif now >= next_beat:
                    say(f"[{self.stamp()}] still waiting for the power to go off (link {state}; gives up at {args.loss_timeout:.0f} s)")
                    next_beat = now + 10.0
            else:
                if state == "port_vanished" and listed:
                    via_port = True
                    marks["port_back"] = now
                    say(f"[{self.stamp()}] the port is listed again; recover() reopens it and waits for the unit")
                    break
                streak = streak + 1 if answered else 0
                if streak >= 2:
                    break
                if time.perf_counter() - t_lost > args.back_timeout:
                    return {"result": "no_back", "marks": marks, "changes": changes, "polls": polls}
                if now >= next_beat:
                    say(f"[{self.stamp()}] no answer yet, {time.perf_counter() - t_lost:.0f} s since the loss (gives up at {args.back_timeout:.0f} s)")
                    next_beat = now + 5.0
            rest = args.poll - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
        if not via_port:
            marks["back"] = time.perf_counter() - self.t0
            big("POWER BACK")
            say(f"[{self.stamp()}] the unit answers again; the link was lost for {marks['back'] - marks['off_seen']:.1f} s; settling {args.settle:.1f} s, then recover()")
            time.sleep(args.settle)
        return {"result": "ok", "marks": marks, "changes": changes, "polls": polls, "via_port": via_port, "t_lost_perf": t_lost}

    # ------------------------------------------------------------------------------------------ recovery and the way back
    def _recover(self) -> Any:
        rc, ax, args = self.rc, self.raw, self.args
        policy = rc.RecoveryPolicy(restore_baudrate=True, restore_control=True, restore_safety_speed=True, clear_message_alarms=False,
                                   check_rows=True, check_parameters=True, poll_s=0.5)
        timeout = args.recover_timeout
        if self.wait_result and self.wait_result.get("via_port"):
            timeout = max(timeout, args.back_timeout - (time.perf_counter() - self.wait_result["t_lost_perf"]))
        self.say(f"recover(timeout {timeout:.0f} s): restore baud rate and PC control, compare rows and parameters; nothing non-volatile is written")
        try:
            rep = ax.recover(timeout=timeout, policy=policy, on_event=lambda e: self.say(f"   [recovery] {e.kind}: {e.message}"), name="unit")
        except rc.RecoveryError as exc:  # includes UnitReplacedError
            self.recover_failed = True
            rep = getattr(exc, "report", None)
            if rep is not None:
                self.remember("recovery report (failed)", SimpleNamespace(modbus_enabled=rep.modbus_control, servo_on=rep.servo_on))
                self._print_report(rep)
                self.recovery = rep.to_dict()
            raise
        self.remember("recovery report", SimpleNamespace(modbus_enabled=rep.modbus_control, servo_on=rep.servo_on))
        self.marks["recovered"] = time.perf_counter() - self.t0
        if self.wait_result and self.wait_result.get("via_port"):
            self.marks["back"] = self.marks["recovered"]
            self.ctx.console.big("POWER BACK")
        self._print_report(rep)
        self.recovery = rep.to_dict()
        bad = []
        if not rep.same_unit:
            bad.append("not the same unit")
        if rep.fault_alarm:
            bad.append(f"fault alarm 0x{rep.alarm_code:03X}")
        if rep.row_problems or rep.parameter_differences:
            bad.append(f"rows {rep.row_problems} / parameters {rep.parameter_differences} differ from before")
        if bad:
            raise Abort(rep.summary() + " | " + "; ".join(bad) + " | servo, home and the return move are skipped")
        if not rep.restarted:
            return CHECK, rep.summary() + " | no restart sign: did the unit really lose its power?", rep
        return PASS, rep.summary() + " | restart signs: " + "; ".join(rep.restart_signs), rep

    def _print_report(self, rep: Any) -> None:
        say = self.say
        say("-" * 72)
        say(f"RECOVERY REPORT ({rep.elapsed_s:.2f} s): {rep.summary()}")
        say(f"  cause {rep.cause}, restarted {rep.restarted}; same unit {rep.same_unit}; baud {rep.baudrate_found} -> {rep.baudrate} (restored {rep.baud_restored})")
        say(f"  PC control now {rep.modbus_control} (restored {rep.control_restored}, kept {rep.control_kept}); servo was {rep.servo_was_on}, now {rep.servo_on}; "
            f"home {rep.homed}, needs home {rep.needs_home}; position {rep.position_mm} mm")
        say(f"  alarm 0x{rep.alarm_code:03X} {rep.alarm_name or ''}; rows checked {rep.rows_checked}, problems {rep.row_problems}; parameters "
            f"{rep.parameters_checked} compared, differences {rep.parameter_differences}; port reopened {rep.port_reopened}")
        for a in rep.actions:
            say(f"  action: {a}")
        for n in rep.notes:
            say(f"  note: {n}")
        say("-" * 72)

    def _servo(self) -> Any:
        st = self.raw.status()
        if st.servo_on:
            raise Skip("servo is already on")
        if st.alarm:
            raise Abort(f"fault alarm 0x{st.alarm_code:03X} is active: servo not switched on")
        st = self.raw.servo(True)
        self.remember("servo on", st)
        if not st.servo_on:
            raise Abort("servo ON was sent but the servo is still off: " + pos_str(st) + "; home and return skipped")
        return PASS, "servo on -> " + pos_str(st)

    def _home(self) -> Any:
        st = self.raw.status()
        if st.homed:
            raise Skip("the unit kept its home: no home return")
        if not st.servo_on:
            raise Abort("servo is off: cannot home")
        self.ctx.console.big("HOMING: the home was lost, the actuator moves to its home end now at the unit's own home speed", "-")
        st = self.raw.home(timeout=self.args.home_timeout)
        self.remember("home", st)
        if not st.homed:
            raise Abort("the home return finished but home is not set: " + pos_str(st) + "; the return move is skipped")
        return PASS, "home return done: " + pos_str(st)

    def _return(self) -> Any:
        ax, args, start = self.raw, self.args, self.start["position_mm"]
        st = ax.status()
        self.remember("before the return move", st)
        if self.return_refused or self.geo is None:
            self.return_refused = True
            raise Check(f"return move refused, no move: the start position {start:.2f} mm is outside the test band; the unit stays at {st.position_mm:.2f} mm")
        dist = abs(st.position_mm - start)
        if dist <= 0.05:
            raise Skip(f"already at the start position ({st.position_mm:.2f} mm)")
        self.say(f"moving back {st.position_mm:.2f} -> {start:.2f} mm at {args.return_speed:g} mm/s")
        st = ax.move_to(start, speed_mm_s=args.return_speed, accel_g=min(ACCEL_G, ax.accel_cap_g), timeout=dist / args.return_speed * 2.0 + 10.0)
        self.remember("return move", st)
        return (PASS if near(st.position_mm, start) else FAIL), "-> " + pos_str(st)

    def _end(self) -> Any:
        ax, start = self.raw, self.start
        st = ax.status()
        cl = ax.check_link()
        nv = getattr(ax, "nonvolatile_writes", None)
        self.remember("end state", st)
        checks = {"servo on": st.servo_on, "homed": st.homed, "PC mode": st.modbus_enabled, f"baud {POWER_UP_BAUD}": ax.baudrate == POWER_UP_BAUD,
                  "no alarm": not st.alarm, "link ok": cl.state == "ok", "no non-volatile write": nv == 0}
        skipped = ""
        if self.return_refused:
            skipped = f" | start position check skipped: the return move was refused (start {start['position_mm']:.2f} mm)"
        else:
            checks[f"at the start position {start['position_mm']:.2f} mm"] = near(st.position_mm, start["position_mm"])
        bad = [k for k, v in checks.items() if not v]
        self.end_info.update({"status": st, "checks": checks, "link": cl.state, "nonvolatile_writes": nv})
        return (FAIL if bad else PASS), f"{pos_str(st)} baud {ax.baudrate} link {cl.state} nv writes {nv}" + (f" | NOT OK: {bad}" if bad else " | all ok") + skipped, checks


def main(argv: Sequence[str]) -> int:
    from .main import make_context

    args = build_parser().parse_args(list(argv))
    try:
        ctx = make_context(args, "power-cycle", argv)
    except SetupError as exc:
        print(f"REFUSED (exit {EXIT_REFUSED}): {exc}", file=sys.stderr)
        return EXIT_REFUSED
    try:
        return PowerCycle(ctx).run()
    finally:
        ctx.close()

