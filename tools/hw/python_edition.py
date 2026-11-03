"""The Python edition of the acceptance run: the public API of the iai_actuator_driver package on a real unit or the simulator.

Order of a run (the safety rules are built in, none of them can be turned off):

1. open the port, identify the unit (reads only);
2. a read-only backup of the unit (identity, every parameter, the position table, the alarm history) into the results folder;
3. the start state is recorded; a unit with an alarm, a moving axis or an emergency stop is not touched;
4. a person confirms (or --yes), then PC control and the servo are switched on if the unit was found without them;
5. the selected steps, in a fixed order. No step writes a parameter or a position row, except one same-value write that
   the driver's non-volatile guard must skip (a write cycle there stops the run);
6. the end sequence on every path (also after a failure or Ctrl+C): stop a move in progress, fast mode off, PC mode back on,
   back to the start position, servo as found, then the state is checked and compared with a second read-only backup.
"""

from __future__ import annotations

import threading
import time
import traceback
from types import SimpleNamespace
from typing import Any

from . import report
from .common import TOL_MM, Geometry, StepDef, alarm_at_end, band_from_stroke, compare_backups, near, servo_across_toggle
from .report import CHECK, FAIL, PASS, Abort, Check, Skip, jsonable, pos_str
from .session import ACCEL_G, EXIT_FAIL, EXIT_INTERRUPTED, EXIT_OK, EXIT_REFUSED, GuardedAxis, SetupError, confirm, open_axis
from .summary import finish

STEPS: tuple[StepDef, ...] = (
    StepDef("unit_info", "unit_info(): every section", why="the unit's own description is complete and has no read errors"),
    StepDef("status", "status()", why="the status block reads and shows no alarm"),
    StepDef("limits", "limits() and caps", why="the speed cap, the acceleration cap and the stroke band are as requested"),
    StepDef("tables", "position table and parameters (read only)", why="the first rows and every parameter decode"),
    StepDef("home", "home() if the unit is not homed", changes=True, motion=True, critical=True,
            why="a unit that is not homed homes (the unit's own speed); a homed unit is left alone"),
    StepDef("servo", "servo off, servo on", changes=True, motion=True, critical=True,
            why="servo off and on again: homed, no alarm, the axis stays where it was"),
    StepDef("move_to", "move_to()", changes=True, needs="band", motion=True, critical=True, why="an absolute move arrives within 0.1 mm"),
    StepDef("move_by", "move_by()", changes=True, needs="band", motion=True, critical=True, why="a relative move arrives within 0.1 mm"),
    StepDef("jog", "jog() both ways", changes=True, needs="band", motion=True, critical=True, why="small steps out and back"),
    StepDef("refusals", "moves the driver must refuse", needs="band", motion=True,
            why="a speed above the cap, a target outside the band and an acceleration above the cap are refused with no motion"),
    StepDef("fast", "fast mode and monitor events", changes=True, needs="band", motion=True,
            why="fast completion, the status monitor's change and done events, fast mode off again"),
    StepDef("handle", "move handle polled to done", changes=True, needs="band", motion=True, critical=True,
            why="move_to(wait=False) returns a handle that reports done and the result"),
    StepDef("cancel", "move handle cancelled half way", changes=True, needs="slow", motion=True, critical=True,
            why="cancel() stops a slow move short of its target without an alarm"),
    StepDef("stop", "stop() half way", changes=True, needs="slow", motion=True, critical=True,
            why="stop() ends a slow move short of its target without an alarm"),
    StepDef("pause", "pause() and resume()", changes=True, needs="slow", motion=True, critical=True,
            why="a paused move holds its position and finishes after resume()"),
    StepDef("speed", "change_speed() during a move", changes=True, needs="slow", motion=True, critical=True,
            why="the axis speeds up when the speed is raised during the move"),
    StepDef("decel", "move_to() with its own deceleration", changes=True, needs="band", motion=True, critical=True,
            why="a command-profile move with a lower deceleration arrives"),
    StepDef("pattern", "move_to(accel_pattern='s_motion')", changes=True, needs="band", motion=True, critical=True,
            why="the move runs, or the driver refuses it before sending when the unit does not list the pattern"),
    StepDef("push", "push into free air", changes=True, needs="band", motion=True, critical=True,
            why="a push move with nothing in the way ends as a miss (the axis needs free travel for 2 mm past the middle)"),
    StepDef("nv_guard", "same-value parameter write is skipped", critical=True,
            why="writing a parameter its own value writes nothing and uses no non-volatile write cycle"),
    StepDef("baud", "baud rate detection", critical=True, why="the baud rate found on the line is the one the run started at"),
    StepDef("link", "track() and check_link()", critical=True, why="a recovery session can be taken and the link reads ok"),
    StepDef("pc_pio", "PC / PIO toggle", changes=True, motion=True, critical=True,
            why="PIO mode, PC mode, Modbus control off and on; the axis stays still in PIO mode and moves again in PC mode"),
    StepDef("check", "run_check(): the self-test", why="the driver's read-only self-test has no failing line"),
    StepDef("recover", "recover() after a 250 ms line break", default=False, changes=True, motion=True, critical=True,
            why="the unit falls silent, recover() finds it again, restores the baud rate and keeps PC control (opt-in: it "
                "changes the baud rate of the line and back)"),
)
STEP_KEYS = tuple(s.key for s in STEPS)
FIXED = (("open", "open the port and identify the unit (read only)"),
         ("backup", "read-only backup before anything changes"),
         ("preflight", "start state, test band"),
         ("prepare", "PC control and servo on, when the unit was found without them"),
         ("end", "back to the start state, compared with a second read-only backup"))


def default_steps() -> list[str]:
    return [s.key for s in STEPS if s.default]


class PythonEdition:
    name = "python"

    def __init__(self, ctx: SimpleNamespace, selected: list[str]) -> None:
        self.ctx, self.args, self.say, self.iad = ctx, ctx.args, ctx.console.say, ctx.iad
        self.selected = [k for k in STEP_KEYS if k in selected]  # always the fixed order, whatever the command line said
        self.rec = report.Recorder(self.say, refusals=(self.iad.UnsafeCommandError,), stop_motion=self._stop_motion)
        self.raw: Any = None
        self.ax: GuardedAxis | None = None
        self.sim: Any = None
        self.info: Any = None
        self.unit_info_data: dict[str, Any] = {}
        self.backup_start: dict[str, Any] | None = None
        self.start: dict[str, Any] = {}
        self.band: tuple[float, float] | None = None
        self.geo: Geometry | None = None
        self.motion_ok, self.motion_why = False, "the start state was not read"
        self.changed = False  # the run switched PC control or the servo on
        self.took_control = self.servo_was_off = False
        self.pio_handed = False
        self.params_cache: list[dict[str, Any]] | None = None
        self.interrupt_after: float | None = None
        self.timer: threading.Timer | None = None
        self.scenario = getattr(self.args, "sim_scenario", "normal") if self.args.sim else "normal"
        self.end_info: dict[str, Any] = {}
        self.compare: dict[str, Any] | None = None
        self.notes: list[str] = []
        self.recover_cleared: set[int] = set()  # the alarm codes recover() reset (its report's alarms_cleared)

    # ------------------------------------------------------------------------------------------ the run
    def run(self) -> int:
        code = EXIT_OK
        try:
            code = self._flow()
        except KeyboardInterrupt:
            code = EXIT_INTERRUPTED
            self.ctx.console.big("Ctrl+C: stopping; the unit is put back as found", "!")
        except SetupError as exc:
            self.say(f"REFUSED: {exc}")
            code = EXIT_REFUSED
        except Exception as exc:  # noqa: BLE001
            self.say(f"UNEXPECTED {type(exc).__name__}: {exc}")
            self.say(traceback.format_exc())
            code = EXIT_FAIL
        finally:
            if self.timer is not None:
                self.timer.cancel()  # a simulated Ctrl+C that has not happened yet must not land after the run
            self._guarded(self._end_sequence)
            self._guarded(self._close)
        if code in (EXIT_OK, EXIT_FAIL) and any(r["verdict"] == report.INTR for r in self.rec.rows):
            code = EXIT_INTERRUPTED
        elif code == EXIT_OK and any(r["verdict"] == FAIL for r in self.rec.rows):
            code = EXIT_FAIL
        self._guarded(lambda: self._save(code))
        return code

    def _guarded(self, fn: Any) -> None:
        """Run an end stage; a Ctrl+C in it only repeats it (three times), any other error is printed and goes on."""
        for _ in range(3):
            try:
                fn()
                return
            except KeyboardInterrupt:
                self.say("Ctrl+C again: still putting the unit back")
            except Exception as exc:  # noqa: BLE001
                self.say(f"end stage failed: {type(exc).__name__}: {exc}")
                return

    def _flow(self) -> int:
        rec = self.rec
        rec.step("open", FIXED[0][1], self._open, critical=True)
        if self.ax is None:
            return EXIT_REFUSED
        rec.step("backup", FIXED[1][1], self._backup, critical=True)
        rec.step("preflight", FIXED[2][1], self._preflight, critical=True)
        if rec.aborted:
            self.say("The unit was not changed. Nothing else runs.")
            return EXIT_REFUSED
        chosen = [s for s in STEPS if s.key in self.selected]
        if any(s.changes for s in chosen):
            self._plan()
            confirm(self.args, self.ctx.console, "The axis moves next, as listed above. Keep clear of it.")
            rec.step("prepare", FIXED[3][1], self._prepare, critical=True)
            if self.interrupt_after is not None:  # --sim-scenario interrupt: the "person" presses Ctrl+C in the middle of the run
                import _thread

                self.say(f"   [sim] Ctrl+C will be pressed {self.interrupt_after:g} s from now")
                self.timer = threading.Timer(self.interrupt_after, _thread.interrupt_main)
                self.timer.start()
        for sd in chosen:
            rec.step(sd.key, sd.title, lambda sd=sd: self._call(sd), motion=sd.motion, critical=sd.critical)
        return EXIT_OK

    def _call(self, sd: StepDef) -> Any:
        if sd.needs and not self.motion_ok:
            raise Skip(self.motion_why)
        if sd.needs == "slow" and self.geo is not None and not self.geo.slow_usable:
            raise Skip(f"the band is too small for a move that is stopped half way (travel {self.geo.d:.2f} mm, need 2 mm)")
        if self.scenario == "fault" and sd.key == "move_by":  # --sim-scenario fault: the unit stops answering during this step
            self.sim.inject_fault("timeout", 4)
        return getattr(self, "s_" + sd.key)()

    # ------------------------------------------------------------------------------------------ fixed phases
    def _open(self) -> Any:
        caps = {"max_speed_mm_s": self.args.speed_cap, "max_accel_g": ACCEL_G}
        scenario = self.scenario
        self.ctx.sim_options = {"cold": {"cold_boot": True}, "outside": {"position_mm": 9.0}}.get(scenario, {})
        self.raw, self.sim = open_axis(self.ctx, caps)
        self.ax = GuardedAxis(self.raw)
        if scenario == "alarm":
            self.sim.trigger_alarm(0x0A1)
        elif scenario == "warning":  # a message-level alarm (0x069, as on a PCON whose clock is not set): the unit still runs
            self.sim.trigger_alarm(0x069)
        elif scenario == "pio-input":
            from iai_actuator_driver import registers

            self.sim.set_word_override(registers.DIPM, 0x0001)
        elif scenario == "interrupt":
            self.interrupt_after = 3.0
        self.info = self.raw.connect(take_control=False)  # reads only: PC control is neither taken nor dropped
        ident = {k: getattr(self.info, k, None) for k in ("model", "model_name", "serial_number", "controller", "firmware",
                                                          "system_code", "unit_code", "max_positions")}
        self.ctx.unit = ident
        text = f"{ident['model'] or '?'} | serial {ident['serial_number']} | firmware {ident['firmware']} | baud {self.raw.baudrate}"
        return (PASS if ident["model"] else CHECK), text, ident

    def _backup(self) -> Any:
        self.backup_start, self.unit_info_data, text, verdict = readonly_backup(self.raw, self.info, self.ctx.out)
        return verdict, text

    def _preflight(self) -> Any:
        raw = self.raw
        st = raw.status()
        self.start = {"position_mm": st.position_mm, "servo_on": st.servo_on, "homed": st.homed, "pc_mode": st.modbus_enabled,
                      "safety_speed": bool(getattr(st, "safety_speed", False)), "baudrate": raw.baudrate,
                      "alarm_code": st.alarm_code}
        problems = []
        if st.alarm:
            problems.append(f"alarm 0x{st.alarm_code:03X} (the harness never clears an alarm: clear it with the IAI PC software, then run again)")
        if st.emergency_stop:
            problems.append("an emergency stop is active")
        if st.moving:
            problems.append("the axis is moving")
        unit_stroke = raw.ec_stroke
        self.band = band_from_stroke(self.args.stroke_band, unit_stroke)
        if self.band is not None:
            raw.stroke = self.band  # the driver now refuses any target outside it
        eff = raw.effective_stroke
        selected_motion = [s for s in STEPS if s.key in self.selected and s.needs]
        if eff is None:
            self.motion_why = "the unit's stroke is not known: give the band in mm with --stroke-band LOW:HIGH"
        else:
            self.geo = Geometry(eff[0], eff[1], min(self.args.speed_cap, raw.speed_cap_mm_s), raw.accel_cap_g)
            if not self.geo.usable:
                self.motion_why = f"the band {eff[0]:.2f} .. {eff[1]:.2f} mm is too small to test in (travel {self.geo.d:.2f} mm, need 1 mm)"
            elif st.homed and not self.geo.inside(st.position_mm):
                self.motion_why = (f"the unit is at {st.position_mm:.2f} mm, outside the test band {eff[0]:.2f} .. {eff[1]:.2f} mm; move it into the "
                                   "band by hand, or give a band that includes it (for example --stroke-band 0:10000 = the whole stroke) so that "
                                   "it can be put back where it was")
            elif not st.homed and "home" not in self.selected:
                self.motion_why = "the unit is not homed and the home step is not selected"
            else:
                self.motion_ok = True
        text = pos_str(st) + f" baud {raw.baudrate}; " + (self.geo.describe() if self.geo else "no test band")
        if problems:
            raise Abort("not touching the unit: " + "; ".join(problems) + " | " + text)
        if selected_motion and not self.motion_ok:
            return CHECK, f"the moving steps are skipped: {self.motion_why} | {text}"
        return PASS, text

    def _plan(self) -> None:
        g, say = self.geo, self.say
        say("-" * 72)
        say("PLAN: the axis will move")
        say(f"  unit        : {self.ctx.unit.get('model')} (serial {self.ctx.unit.get('serial_number')}), at {self.start['position_mm']:.2f} mm")
        say(f"  moves       : {g.describe() if self.motion_ok else 'none (' + self.motion_why + ')'}")
        say("  steps       : " + ", ".join(self.selected))
        say("  safety      : nothing is written to the unit (one same-value parameter write is skipped by the driver); the unit is")
        say("                put back at its start position with its servo and PC mode as found, also after a failure or Ctrl+C")
        if "servo" in self.selected:
            say("  NOTE        : the servo step switches the servo off for a moment: a vertical axis must hold its load (brake)")
        if "push" in self.selected:
            say("  NOTE        : the push step pushes into free air: nothing may be in the way above the middle of the band")
        if "recover" in self.selected:
            say("  NOTE        : the recover step sends a line break and may change the baud rate of the line and back")
        say("-" * 72)

    def _prepare(self) -> Any:
        ax, st = self.ax, self.ax.status()
        did = []
        self.changed = True
        if self.scenario == "obstacle" and self.geo is not None:  # a workpiece in the push path: the push step must see contact
            self.sim.obstacle_mm = self.geo.c + 0.5
        if not st.modbus_enabled:
            self.took_control = True
            ax.enable_modbus_control()
            did.append("took PC control (the unit was in PIO mode)")
        if not st.servo_on:
            self.servo_was_off = True
            ax.servo(True)
            did.append("servo ON (it was off)")
        now = ax.status()
        if not now.modbus_enabled:
            raise Abort("PC control could not be taken: " + pos_str(now))
        if not did:
            return PASS, "already in PC mode with the servo on: " + pos_str(now)
        return PASS, "; ".join(did) + " | " + pos_str(now)

    # ------------------------------------------------------------------------------------------ helpers for steps
    def go(self, target: float, speed: float | None = None) -> Any:
        """Move to ``target`` inside the band and wait."""
        g, speed = self.geo, speed or self.geo.v
        here = self.ax.status().position_mm
        return self.ax.move_to(target, speed_mm_s=speed, accel_g=g.acc, timeout=g.timeout(here - target, speed))

    def _stop_motion(self) -> Any:
        if self.ax is None:
            return None
        return self.ax.stop(wait=True)

    def _still(self, seconds: float) -> float:
        positions: list[float] = []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < seconds:
            positions.append(round(self.ax.status().position_mm, 2))
            time.sleep(0.1)
        return max(positions) - min(positions)

    @staticmethod
    def slope(samples: list[tuple[float, float]], t_from: float, t_to: float) -> float | None:
        pts = [(t, p) for t, p in samples if t_from <= t <= t_to]
        if len(pts) < 2:
            return None
        (ta, pa), (tb, pb) = pts[0], pts[-1]
        return round((pb - pa) / (tb - ta), 2) if tb > ta else None

    def parameters(self) -> list[dict[str, Any]]:
        """Every parameter, decoded (read once, read only)."""
        if self.params_cache is None:
            self.params_cache = self.ax.parameters()
        return self.params_cache

    # ------------------------------------------------------------------------------------------ read-only steps
    def s_unit_info(self) -> Any:
        ui = self.unit_info_data
        sections = sorted(k for k, v in ui.items() if v is not None)
        errors = ui.get("errors") or (ui.get("meta") or {}).get("errors")
        ident = ui.get("identity") or {}
        ok = bool(ident) and not errors
        return (PASS if ok else CHECK), f"sections {sections}; serial {ident.get('serial_number')}; errors {errors or 'none'}", {"sections": sections}

    def s_status(self) -> Any:
        st = self.ax.status()
        return (CHECK if st.alarm else PASS), pos_str(st) + f" baud {self.ax.baudrate}", st

    def s_limits(self) -> Any:
        ax = self.ax
        lim = ax.limits()
        caps = {"speed_cap_mm_s": ax.speed_cap_mm_s, "accel_cap_g": ax.accel_cap_g, "push_cap_pct": ax.push_cap_pct,
                "effective_stroke": ax.effective_stroke, "unit_stroke": ax.ec_stroke, "kind": ax.kind, "units": ax.units,
                "family": ax.product_family, "max_positions": ax.max_positions(), "limits": lim}
        problems = []
        if ax.speed_cap_mm_s > self.args.speed_cap + 1e-9:
            problems.append(f"speed cap {ax.speed_cap_mm_s} is above the requested {self.args.speed_cap}")
        eff = ax.effective_stroke
        if self.band and (eff is None or not near(eff[0], self.band[0], 0.02) or not near(eff[1], self.band[1], 0.02)):
            problems.append(f"effective stroke {eff} is not the band {self.band}")
        text = (f"{ax.kind} ({ax.product_family}, {ax.units}); speed cap {ax.speed_cap_mm_s:g}, accel cap {ax.accel_cap_g:g} G, "
                f"stroke {ax.ec_stroke}, band {eff}, {ax.max_positions()} rows")
        return (CHECK if problems else PASS), text + (" | " + "; ".join(problems) if problems else ""), caps

    def s_tables(self) -> Any:
        ax = self.ax
        rows = {}
        for n in range(min(int(ax.max_positions() or 0), 8)):
            rows[n] = jsonable(ax.read_position_table(n))
        params = self.parameters()
        if not params:
            raise Check("the unit lists no parameters")
        text = "rows " + "; ".join(f"{n}: {r.get('position_mm')} mm {r.get('speed_mm_s')} mm/s" for n, r in rows.items())
        return PASS, f"{text} | {len(params)} parameters decoded", {"rows": rows}

    # ------------------------------------------------------------------------------------------ moving steps
    def s_home(self) -> Any:
        ax = self.ax
        if ax.status().homed:
            raise Skip("the unit is homed: home() not run")
        self.ctx.console.big("HOMING: the actuator moves to its home end at the unit's own home speed", "-")
        st = ax.home(timeout=300)
        return (PASS if st.homed else FAIL), pos_str(st)

    def s_servo(self) -> Any:
        ax, before = self.ax, self.ax.status()
        off = ax.servo(False)
        time.sleep(0.3)
        on = ax.servo(True)
        ok = (not off.servo_on) and on.servo_on and on.homed == before.homed and not on.alarm and near(on.position_mm, before.position_mm)
        return (PASS if ok else FAIL), (f"servo off -> servo {int(off.servo_on)}; servo on -> servo {int(on.servo_on)} homed {int(on.homed)} "
                                        f"alarm {int(on.alarm)}, position {before.position_mm:.2f} -> {on.position_mm:.2f} mm")

    def s_move_to(self) -> Any:
        g = self.geo
        st = self.go(g.a)
        ok = near(st.position_mm, g.a) and not st.moving
        return (PASS if ok else FAIL), f"move_to({g.a:.2f}, {g.v:g} mm/s) -> {st.position_mm:.2f} mm, moving {int(st.moving)}"

    def s_move_by(self) -> Any:
        g = self.geo
        self.go(g.a)
        delta = g.d / 2.0
        st = self.ax.move_by(delta, speed_mm_s=g.v, accel_g=g.acc, timeout=g.timeout(delta, g.v))
        return (PASS if near(st.position_mm, g.c) else FAIL), f"move_by({delta:+.2f}) from {g.a:.2f} -> {st.position_mm:.2f} mm (want {g.c:.2f})"

    def s_jog(self) -> Any:
        g = self.geo
        self.go(g.c)
        a = self.ax.jog(+g.jog, speed_mm_s=g.v, accel_g=g.acc, timeout=g.timeout(g.jog, g.v))
        b = self.ax.jog(-g.jog, speed_mm_s=g.v, accel_g=g.acc, timeout=g.timeout(g.jog, g.v))
        ok = near(a.position_mm, g.c + g.jog) and near(b.position_mm, g.c)
        return (PASS if ok else FAIL), f"jog({g.jog:+.2f}) -> {a.position_mm:.2f}; jog({-g.jog:+.2f}) -> {b.position_mm:.2f} mm (middle {g.c:.2f})"

    def s_refusals(self) -> Any:
        ax, g = self.ax, self.geo
        before = ax.status().position_mm
        beyond = max(1.0, 0.1 * g.span)
        cases = {f"speed {g.cap * 1.25 + 1:.0f} above the cap {g.cap:g}": {"mm": g.c, "speed_mm_s": g.cap * 1.25 + 1, "accel_g": g.acc},
                 f"target {g.hi + beyond:.2f} above the band": {"mm": g.hi + beyond, "speed_mm_s": g.v, "accel_g": g.acc},
                 f"target {g.lo - beyond:.2f} below the band": {"mm": g.lo - beyond, "speed_mm_s": g.v, "accel_g": g.acc},
                 f"acceleration {g.acc * 2:g} G above the cap": {"mm": g.c, "speed_mm_s": g.v, "accel_g": g.acc * 2}}
        result = {}
        for name, kwargs in cases.items():
            try:
                ax.check_move(**kwargs)
                result[name] = "ACCEPTED"
            except self.iad.SoftLimitError:
                result[name] = "refused"
        after = ax.status().position_mm
        ok = all(v == "refused" for v in result.values()) and near(before, after, 0.05)
        return (PASS if ok else FAIL), f"check_move: {result}; no motion {before:.2f} -> {after:.2f} mm"

    def s_fast(self) -> Any:
        ax, g = self.ax, self.geo
        counts = {"change": 0, "done": 0, "event": 0}
        lock = threading.Lock()

        def bump(name: str) -> Any:
            def handler(*_: Any) -> None:
                with lock:
                    counts[name] += 1

            return handler

        limit = g.timeout(g.d, g.v)
        self.go(g.a)
        enabled = ax.enable_fast(baudrate=None)  # the baud rate is kept
        try:
            fast_on = ax.completion == "fast" and ax.monitor is not None
            ax.stop_monitor()
            mon = ax.start_monitor(on_change=bump("change"), on_done=bump("done"), on_event=bump("event"))
            time.sleep(0.3)
            c0 = dict(counts)
            t0 = time.perf_counter()
            handle = ax.move_to(g.b, speed_mm_s=g.v, accel_g=g.acc, wait=False)
            hit = mon.wait_for(lambda s: s.position_mm > g.c, timeout=limit)
            st = handle.wait(limit)
            took = time.perf_counter() - t0
            time.sleep(0.3)
            settled = ax.status().position_mm
            c1 = dict(counts)
            back = ax.move_to(g.a, speed_mm_s=g.v, accel_g=g.acc, timeout=limit)
            time.sleep(0.3)
            c2 = dict(counts)
        finally:
            ax.disable_fast()
        safe = ax.completion == "safe" and ax.monitor is None
        ok = (fast_on and hit is not None and near(settled, g.b) and near(st.position_mm, g.b, 0.15) and near(back.position_mm, g.a, 0.15)
              and c1["change"] > c0["change"] and c1["done"] > c0["done"] and c2["done"] > c1["done"] and safe)
        return (PASS if ok else CHECK), (f"enable_fast(baudrate=None): fast {fast_on}, baud {ax.baudrate}; {g.a:.2f} -> {g.b:.2f} returned "
                                         f"{st.position_mm:.2f} in {took:.2f} s, settled {settled:.2f}; wait_for(middle) "
                                         f"{round(hit.position_mm, 2) if hit else None}; back {back.position_mm:.2f}; callbacks {c0} -> {c1} -> {c2}; "
                                         f"disable_fast: completion {ax.completion}, monitor {ax.monitor}"), {"enabled": enabled, "counts": [c0, c1, c2]}

    def s_handle(self) -> Any:
        ax, g = self.ax, self.geo
        self.go(g.a)
        handle = ax.move_to(g.b, speed_mm_s=g.v, accel_g=g.acc, wait=False)
        t0, polls, limit = time.perf_counter(), 0, g.timeout(g.d, g.v)
        while not handle.done() and time.perf_counter() - t0 < limit:
            polls += 1
            time.sleep(0.02)
        st = handle.wait(limit)
        ok = handle.done() and near(st.position_mm, g.b) and handle.result is not None
        return (PASS if ok else FAIL), (f"MoveHandle {g.a:.2f} -> {g.b:.2f}: done() after {polls} polls, {time.perf_counter() - t0:.2f} s; "
                                        f"wait() -> {st.position_mm:.2f}"), handle.to_dict()

    def _halfway(self, start: float, target: float, act: Any) -> Any:
        """A slow move from ``start`` towards ``target``; after 0.4 s ``act(handle)`` ends it. Returns what was seen."""
        ax, g = self.ax, self.geo
        self.go(start)
        handle = ax.move_to(target, speed_mm_s=g.v_slow, accel_g=g.acc, wait=False)
        time.sleep(0.4)
        mid = ax.status()
        result = act(handle)
        time.sleep(0.3)
        return mid, result, ax.status(), handle

    def s_cancel(self) -> Any:
        g = self.geo
        mid, st, end, handle = self._halfway(g.b, g.a, lambda h: h.cancel(wait=True, timeout=5))
        margin = 0.05
        ok = mid.moving and g.a + margin < end.position_mm < g.b - margin and not end.moving and not end.alarm and handle.done()
        return (PASS if ok else FAIL), (f"{g.b:.2f} -> {g.a:.2f} at {g.v_slow:g} mm/s; after 0.4 s {mid.position_mm:.2f} moving {int(mid.moving)}; cancel "
                                        f"-> {st.position_mm if st else None}; 0.3 s later {end.position_mm:.2f} moving {int(end.moving)}, done {handle.done()}")

    def s_stop(self) -> Any:
        g = self.geo
        mid, st, end, _ = self._halfway(g.a, g.b, lambda h: self.ax.stop(wait=True))
        margin = 0.05
        ok = (mid.moving and g.a + margin < end.position_mm < g.b - margin and not end.moving and not end.alarm
              and near(st.position_mm, end.position_mm, 0.05))
        return (PASS if ok else FAIL), (f"{g.a:.2f} -> {g.b:.2f} at {g.v_slow:g} mm/s; after 0.4 s {mid.position_mm:.2f} moving {int(mid.moving)}; "
                                        f"stop() -> {st.position_mm:.2f}; 0.3 s later {end.position_mm:.2f} moving {int(end.moving)}")

    def s_pause(self) -> Any:
        ax, g = self.ax, self.geo
        self.go(g.a)
        handle = ax.move_to(g.b, speed_mm_s=g.v_slow, accel_g=g.acc, wait=False)
        time.sleep(0.3)
        ax.pause()
        time.sleep(0.4)
        a = ax.status()
        time.sleep(0.4)
        b = ax.status()
        ax.resume()
        st = handle.wait(g.timeout(g.d, g.v_slow))
        ok = a.paused and near(a.position_mm, b.position_mm, 0.05) and g.a < a.position_mm < g.b and near(st.position_mm, g.b)
        return (PASS if ok else FAIL), (f"{g.a:.2f} -> {g.b:.2f} at {g.v_slow:g} mm/s; pause: paused {a.paused}, {a.position_mm:.2f} then "
                                        f"{b.position_mm:.2f}; resume and wait -> {st.position_mm:.2f}")

    def s_speed(self) -> Any:
        ax, g = self.ax, self.geo
        slow, fast = min(2.0, g.cap / 4.0, g.d / 2.0), min(12.0, g.cap)
        if fast < 2.0 * slow:
            raise Skip(f"the speed cap {g.cap:g} mm/s leaves no room to raise the speed from {slow:g}")
        t_change = max(0.3, 0.25 * g.d / slow)
        self.go(g.b)
        handle = ax.move_to(g.a, speed_mm_s=slow, accel_g=g.acc, wait=False)
        t0, limit = time.perf_counter(), g.timeout(g.d, slow)
        samples: list[tuple[float, float]] = []
        changed_at = pos_at_change = None
        end_t = end_pos = None
        while time.perf_counter() - t0 < limit:
            st = ax.status()
            t = time.perf_counter() - t0
            samples.append((round(t, 3), round(st.position_mm, 3)))
            if changed_at is None and t >= t_change:
                ax.change_speed(fast)
                changed_at, pos_at_change = time.perf_counter() - t0, st.position_mm
            if changed_at is not None and not st.moving and t > changed_at + 0.15:
                end_t, end_pos = t, st.position_mm
                break
            time.sleep(0.02)
        final = handle.wait(limit)
        if changed_at is None or end_t is None or end_t <= changed_at:
            raise Check(f"the move ended before the speed could be changed (samples {len(samples)})")
        before = self.slope(samples, 0.15, changed_at)
        after = (end_pos - pos_at_change) / (end_t - changed_at)
        ok = (near(final.position_mm, g.a) and before is not None and abs(before) < 1.5 * slow and abs(after) > 1.5 * abs(before))
        return (PASS if ok else CHECK), (f"{g.b:.2f} -> {g.a:.2f} at {slow:g} mm/s, change_speed({fast:g}) at {changed_at:.2f} s: about {abs(before):.2f} mm/s "
                                         f"before, about {abs(after):.1f} mm/s on average after (including the stop); end {final.position_mm:.2f}"), samples

    def s_decel(self) -> Any:
        ax, g = self.ax, self.geo
        self.go(g.a)
        dec = min(g.dec, ax.decel_cap_g)
        t0 = time.perf_counter()
        st = ax.move_to(g.b, speed_mm_s=g.v, accel_g=g.acc, decel_g=dec, timeout=g.timeout(g.d, g.v))
        return (PASS if near(st.position_mm, g.b) else FAIL), (f"{g.a:.2f} -> {g.b:.2f} accel {g.acc:g} G decel {dec:g} G -> "
                                                               f"{st.position_mm:.2f} in {time.perf_counter() - t0:.2f} s")

    def s_pattern(self) -> Any:
        ax, g = self.ax, self.geo
        self.go(g.b)
        before = ax.status().position_mm
        try:
            t0 = time.perf_counter()
            st = ax.move_to(g.a, speed_mm_s=g.v, accel_g=g.acc, accel_pattern="s_motion", timeout=g.timeout(g.d, g.v))
        except self.iad.SoftLimitError as exc:
            after = ax.status().position_mm
            return (PASS if near(before, after, 0.05) else FAIL), f"s_motion refused before anything was sent: {exc}; no motion {before:.2f} -> {after:.2f} mm"
        return (PASS if near(st.position_mm, g.a) else FAIL), f"{g.b:.2f} -> {g.a:.2f} s_motion -> {st.position_mm:.2f} in {time.perf_counter() - t0:.2f} s"

    def s_push(self) -> Any:
        ax, g = self.ax, self.geo
        lim = ax.limits().get("push") or {}
        if lim.get("available") is False:
            raise Skip("the unit does not offer push moves")
        try:
            speed, source = ax.push_speed_mm_s()
        except Exception as exc:  # noqa: BLE001
            raise Skip(f"the push speed cannot be read: {exc}") from exc
        if speed > g.cap:
            raise Skip(f"the unit's push speed is {speed:g} mm/s ({source}), above the speed cap {g.cap:g}")
        band = min(2.0, 0.5 * (g.hi - g.c))
        if band < 0.2:
            raise Skip(f"no room to push in the band (only {band:.2f} mm above the middle)")
        force = min(max(20.0, float(lim.get("recommended_min") or 20.0)), ax.push_cap_pct)
        self.go(g.a)  # the approach side: the unit goes to the target, then pushes over the band
        result = ax.push(g.c, band_mm=band, force_pct=force, direction=+1, speed_mm_s=g.v, accel_g=g.acc, timeout=g.timeout(band, g.v) + 10)
        after = ax.status()
        back = self.go(g.c)
        text = (f"push speed {speed:g} mm/s ({source}); push({g.c:.2f}, band {band:.2f}, {force:g} %, +1) from {g.a:.2f}: outcome {result.outcome}, end "
                f"{after.position_mm:.2f}; warnings {list(getattr(result, 'warnings', ()) or ())}; back at {back.position_mm:.2f}")
        if result.outcome != "missed":
            raise Check(text + " | the push met resistance: something is in the way")
        ok = near(after.position_mm, g.c + band, 0.15) and near(back.position_mm, g.c)
        return (PASS if ok else FAIL), text, result

    # ------------------------------------------------------------------------------------------ steps that need no band
    def s_nv_guard(self) -> Any:
        params = self.parameters()
        usable = [p for p in params if p.get("writable") and not p.get("dangerous") and not p.get("home_parameter")
                  and p.get("number", p.get("key")) is not None]
        pick = next((p for p in usable if p.get("number") == 2), usable[0] if usable else None)
        if pick is None:
            raise Skip("no parameter that is writable and not marked dangerous")
        key = pick.get("number", pick.get("key"))
        current, result, used = self.ax.same_value_write(key)
        detail = result.to_dict() if hasattr(result, "to_dict") else jsonable(result)
        if not getattr(result, "skipped", False) or used:
            raise Abort(f"A WRITE CYCLE WAS USED: write_parameter({key}, {current['value']!r}) -> skipped {getattr(result, 'skipped', None)}, "
                        f"non-volatile writes used {used}: {detail}")
        return PASS, (f"write_parameter({key}, {current['value']!r}) with the value it already holds: skipped, committed "
                      f"{getattr(result, 'committed', None)}, non-volatile writes used {used}; {detail.get('note')}"), detail

    def s_baud(self) -> Any:
        ax, bus = self.ax, self.ax.bus
        if not getattr(bus, "can_change_baudrate", False):
            raise Skip("the transport cannot change its baud rate")
        found = bus.detect_baudrate(ax.slave)
        st = ax.status()
        want = self.start["baudrate"]
        ok = found == want and ax.baudrate == want
        return (PASS if ok else FAIL), f"detect_baudrate({ax.slave}) -> {found} (the run started at {want}); {pos_str(st)}"

    def s_link(self) -> Any:
        ax = self.ax
        ax.track()
        state = ax.check_link().state
        return (PASS if state == "ok" else FAIL), f"track() and check_link() -> {state}; link_state {ax.link_state}"

    def s_pc_pio(self) -> Any:
        ax, g = self.ax, self.geo
        st = ax.status()
        dipm = (st.raw or {}).get("DIPM")
        if dipm is None:
            raise Skip("the status has no PIO input word, so nobody can tell whether a PIO input is on: the unit is not handed to PIO")
        if dipm:
            raise Check(f"the unit is not handed to PIO: a PIO input is on (DIPM 0x{dipm:04X}) and the unit would act on it at once; "
                        "switch the PIO inputs off and run the step again")
        self.pio_handed = True
        try:
            ax.set_action_mode(False)
            to_pio = ax.status()
            spread = self._still(1.0)
            ax.set_action_mode(True)
            to_pc = ax.status()
            direct, direct_ok = self._servo_after_action_mode(st, to_pio, to_pc)
            ax.disable_modbus_control()
            off = ax.status()
            ax.enable_modbus_control()
            on = ax.status()
        finally:
            confirmed, rec = restore_pc_mode(self.raw, self.say)
            self.pio_handed = not confirmed
        servo, servo_lost = "", False
        end = ax.status()
        lost, expected = servo_across_toggle(st.servo_on, [
            ("set_action_mode(PIO)", to_pio.servo_on, to_pio.paused), ("set_action_mode(PC)", to_pc.servo_on, to_pc.paused),
            ("disable_modbus_control", off.servo_on, off.paused), ("enable_modbus_control", on.servo_on, on.paused),
            ("the restore to PC mode", end.servo_on, end.paused)])
        if lost and expected:  # a PCON with nothing on its PIO connector: servo off and a pause in PIO mode
            servo = "; the servo went off in PIO mode (with a pause)"
            if not end.servo_on:
                ax.servo(True)
                servo = "; the servo went off in PIO mode and is on again"
        elif lost:
            servo_lost = True
            servo = (f"; the servo was off after {lost} and not as a unit with nothing on its PIO connector has it in PIO mode "
                     "(servo off with a pause): not switched on again")
        moved = ""
        ok = (not to_pio.modbus_enabled and spread <= 0.05 and to_pc.modbus_enabled and not off.modbus_enabled and on.modbus_enabled
              and bool(getattr(to_pc, "safety_speed", False)) == self.start["safety_speed"] and not servo_lost and direct_ok)
        if self.motion_ok and not servo_lost:
            first, second = self.go(g.a), self.go(g.c)
            ok = ok and near(first.position_mm, g.a) and near(second.position_mm, g.c)
            moved = f"; PC control works: moves to {first.position_mm:.2f} and {second.position_mm:.2f} mm"
        return (PASS if ok else FAIL), (f"set_action_mode(PIO) -> PC control {int(to_pio.modbus_enabled)} (axis still, spread {spread:.2f} mm); "
                                        f"set_action_mode(PC) -> {int(to_pc.modbus_enabled)}{direct}; disable_modbus_control -> {int(off.modbus_enabled)}; "
                                        f"enable_modbus_control -> {int(on.modbus_enabled)}{servo}{moved}"), rec

    def _servo_after_action_mode(self, start: Any, to_pio: Any, to_pc: Any) -> tuple[str, bool]:
        """Servo ON straight after set_action_mode(PC), before any PMSL toggle: on the PCON-CB the action-mode command left the
        servo refusing to come on until a PMSL OFF (1.0.1 before its fix, 2026-11-02), which the PMSL toggle of this step hid.
        Tried when the servo is off then, unless it was lost without the pause of a unit with nothing on its PIO connector
        (servo_across_toggle fails the step for that, unrepaired). Returns (the text, whether it passed)."""
        if to_pc.servo_on or not to_pc.modbus_enabled or (start.servo_on and not to_pio.paused):
            return "", True
        try:
            self.ax.servo(True)
            taken = bool(self.ax.status().servo_on)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001
            return f" (servo ON right after it NOT taken: {type(exc).__name__}: {exc})", False
        return (" (servo ON right after it: taken)", True) if taken else (" (servo ON right after it NOT taken: the servo is off)", False)

    def s_check(self) -> Any:
        from iai_actuator_driver.check import run_check

        rep = run_check(self.raw, motion=False, port_label=self.args.port or "simulator")
        items = [(i.name, i.result) for i in rep.items]
        fails = [i for i in items if i[1] == "FAIL"]
        warns = [i for i in items if i[1] == "WARN"]
        text = f"run_check(motion=False): {len(items)} lines, {len(fails)} FAIL, {len(warns)} WARN" + (f" {warns}" if warns else "")
        return (FAIL if fails else CHECK if warns else PASS), text + (f"; failing: {fails}" if fails else ""), rep

    def s_recover(self) -> Any:
        ax = self.ax
        if not getattr(ax.bus, "can_send_break", False):
            raise Skip("the transport cannot hold the line in break")
        want = self.start["baudrate"]
        ax.bus.send_break(0.25)
        time.sleep(0.1)
        before = ax.check_link().state
        t0 = time.perf_counter()
        rep = ax.recover(timeout=30)
        took = time.perf_counter() - t0
        self.recover_cleared.update(int(c) for c in (getattr(rep, "alarms_cleared", None) or ()))
        st = ax.status()
        after = ax.check_link().state
        baud_ok = ax.baudrate == want and (rep.baud_restored or rep.baudrate_found in (None, want))
        control_ok = bool(rep.control_kept or rep.control_restored)  # the session knows PC control was wanted (after the PC / PIO toggle too)
        ok = rep.same_unit and baud_ok and not rep.needs_home and st.modbus_enabled and control_ok and st.servo_on and after == "ok"
        text = (f"250 ms break; link {before}; recover() {took:.2f} s: cause {rep.cause}, same unit {rep.same_unit}, baud {rep.baudrate_found} -> "
                f"{rep.baudrate} (restored {rep.baud_restored}, wanted {want}), PC control kept {rep.control_kept} / restored {rep.control_restored}, needs home {rep.needs_home}; "
                f"link now {after}; {pos_str(st)}")
        return (PASS if ok else FAIL), text, rep

    # ------------------------------------------------------------------------------------------ end of the run
    def _end_sequence(self) -> None:
        if self.raw is None or not self.start:  # not opened, or stopped before the start state was read: nothing was changed
            return
        self._restore()
        self.rec.step("end", FIXED[4][1], self._end_check, always=True)

    def _restore(self) -> None:
        """Put the unit back as it was found. Each action is tried on its own: an error in one does not skip the next."""
        raw, start, notes = self.raw, self.start, self.notes

        def attempt(label: str, fn: Any) -> Any:
            try:
                return fn()
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001
                notes.append(f"{label} failed: {type(exc).__name__}: {exc}")
                self.say(f"  restore: {label} failed: {type(exc).__name__}: {exc}")
                return None

        if getattr(raw, "motion_pending", False):
            self.say("restore: a move is still running: decel stop")
            attempt("stop", lambda: raw.stop(wait=True, timeout=5.0))
        if getattr(raw, "completion", None) == "fast" or getattr(raw, "monitor", None) is not None:
            attempt("fast mode off", raw.disable_fast)
        if self.changed or self.pio_handed:  # a run that changed nothing leaves the mode as it found it
            confirmed, rec = restore_pc_mode(raw, self.say)
            self.pio_handed = not confirmed
            self.end_info["pc_restore"] = rec
        if not start:
            return
        st = attempt("read status", raw.status)
        if st is None:
            return
        if st.alarm:
            notes.append(f"alarm 0x{st.alarm_code:03X} at the end: no servo or position change was attempted")
            return
        move_back = (self.motion_ok and self.geo is not None and start["homed"]
                     and not near(st.position_mm, start["position_mm"], 0.05))
        if (start["servo_on"] or move_back) and not st.servo_on:  # found with the servo off: on for the return move only
            attempt("servo on", lambda: raw.servo(True))
            st = attempt("read status", raw.status) or st
        if move_back:
            self.say(f"restore: back to the start position {start['position_mm']:.2f} mm")
            geo, here = self.geo, st.position_mm
            attempt("return move", lambda: raw.move_to(start["position_mm"], speed_mm_s=geo.v, accel_g=geo.acc,
                                                       timeout=geo.timeout(here - start["position_mm"], geo.v)))
            st = attempt("read status", raw.status) or st
        if not start["servo_on"] and st.servo_on:
            attempt("servo off", lambda: raw.servo(False))

    def _end_check(self) -> Any:
        raw, start = self.raw, self.start
        try:
            st = raw.status()
        except Exception as exc:  # noqa: BLE001
            self.say(f"The unit does not answer ({type(exc).__name__}: {exc}). Where it is left: unknown. If it lost its 24 V it comes up in PIO "
                     f"mode with the servo off; to take PC control back, with the axis at rest:  python -m iai_actuator_driver "
                     f"--port {self.args.port or '<port>'} mode pc")
            raise
        link = raw.check_link().state
        nv = getattr(raw, "nonvolatile_writes", None)
        from iai_actuator_driver import alarms as drv_alarms

        code = start["alarm_code"]
        alarm_label = "no alarm" if not code else f"alarm as found (0x{code:03X})"
        alarm_ok, alarm_note = alarm_at_end(code, st.alarm_code, self.recover_cleared,
                                            bool(code) and drv_alarms.alarm_level(code) == drv_alarms.LEVEL_MESSAGE,
                                            drv_alarms.alarm_name(code) if code else None)
        if alarm_note:
            self.notes.append(alarm_note)
            self.end_info["alarm_reset_by_recover"] = code
        checks = {"PC mode" if self.changed else "mode as found": st.modbus_enabled == (True if self.changed else start["pc_mode"]),
                  alarm_label: alarm_ok,
                  f"baud {start['baudrate']}": raw.baudrate == start["baudrate"],
                  "link ok": link == "ok",
                  "no non-volatile write": nv == 0,
                  "safety speed as found": bool(getattr(st, "safety_speed", False)) == start["safety_speed"]}
        if start["homed"]:
            checks["homed"] = st.homed
        checks["servo as found"] = st.servo_on == start["servo_on"]
        if not start["homed"]:
            self.notes.append(f"the unit was not homed at the start, so its start position meant nothing; it is left at {st.position_mm:.2f} mm")
        elif self.motion_ok:
            checks[f"at the start position {start['position_mm']:.2f} mm"] = near(st.position_mm, start["position_mm"], TOL_MM)
        else:
            checks["not moved"] = near(st.position_mm, start["position_mm"], 0.2)
        bad = [k for k, v in checks.items() if not v]
        compare_text = ""
        if self.backup_start is not None:
            from iai_actuator_driver import info as drv_info

            end_bk = drv_info.backup(raw, self.ctx.out / "backup_end.json", rows=backup_rows(raw))
            self.compare = compare_backups(self.backup_start, end_bk)
            c = self.compare
            if c["parameters_changed"]:
                bad.append(f"parameters changed: {c['parameters_changed']}")
            if c["rows_changed"]:
                bad.append(f"position rows changed: {c['rows_changed']}")
            compare_text = (f" | against backup_end.json: {c['parameter_items']} parameter items compared ({c['live_items_left_out']} items of the live classes 3 and 4 left out), {c['rows_checked']} rows: "
                            + ("unchanged" if not (c["parameters_changed"] or c["rows_changed"]) else "CHANGED"))
            if not c["alarm_history_same"] and not bad:
                self.end_info["alarm_history_changed"] = True
                self.notes.append("the alarm history differs from the start: read it with the PC software")
                compare_text += "; alarm history differs"
        self.end_info.update({"status": jsonable(st), "checks": checks, "link": link, "nonvolatile_writes": nv})
        text = f"{pos_str(st)} baud {raw.baudrate} link {link} nv writes {nv}" + (f" | NOT AS FOUND: {bad}" if bad else " | as found") + compare_text
        if self.notes:
            text += " | " + "; ".join(self.notes)
        if self.took_control:
            text += " | the unit was found in PIO mode and is left in PC mode"
        if bad:
            return FAIL, text, checks
        return (CHECK if self.end_info.get("alarm_history_changed") or alarm_note else PASS), text, checks

    def _close(self) -> None:
        if self.raw is not None:
            try:
                self.raw.close()
            except Exception as exc:  # noqa: BLE001
                self.say(f"close: {exc}")
            self.raw = None

    def _save(self, code: int) -> None:
        finish(self.ctx, self, code)


def backup_rows(raw: Any) -> str:
    """Which position rows a backup reads: all of them when the table is small, else the ones in use. The same for the start
    and the end backup, so that they can be compared."""
    n = raw.max_positions()
    return "all" if n and n <= 128 else "used"


def readonly_backup(raw: Any, info: Any, out: Any) -> tuple[dict[str, Any], dict[str, Any], str, str]:
    """The read-only backup that every run makes before it changes anything: ``unit_info_start.json``, ``start_state.json`` and
    ``backup_start.json`` in ``out`` (identity, every parameter, the position table, the alarm history). Returns ``(backup,
    unit_info, text, verdict)``; raises Abort when the backup file was not written."""
    from iai_actuator_driver import info as drv_info

    st = raw.status()
    unit_info = drv_info.unit_info(raw)
    report.write_json(out / "unit_info_start.json", unit_info)
    state: dict[str, Any] = {"identity": info, "baudrate": raw.baudrate, "status": st, "stroke": raw.stroke, "unit_stroke": raw.ec_stroke,
                             "speed_cap": raw.speed_cap_mm_s, "accel_cap": raw.accel_cap_g, "kind": raw.kind, "units": raw.units,
                             "family": raw.product_family, "max_positions": raw.max_positions(), "limits": raw.limits()}
    for name, fn in (("push_speed", raw.push_speed_mm_s), ("boot_baud", raw.read_boot_baudrate)):
        try:
            state[name] = fn()
        except Exception as exc:  # noqa: BLE001
            state[name] = repr(exc)
    report.write_json(out / "start_state.json", state)
    rows = backup_rows(raw)
    path = out / "backup_start.json"
    bk = drv_info.backup(raw, path, rows=rows)
    if not path.is_file():
        raise Abort(f"the backup file {path} was not written: nothing is changed without a backup")
    n = sum(len(c.get("items") or []) for c in (bk.get("parameters") or {}).get("classes") or [])
    text = f"backup_start.json: rows {rows}, {n} parameter items, errors {bk.get('errors')}; unit_info_start.json, start_state.json"
    if bk.get("errors"):
        return bk, unit_info, text + " (the backup is partial)", CHECK
    return bk, unit_info, text, PASS


def in_motion(st: Any) -> bool:
    """Moving or homing: taking PC control now would stop a move. A pause at a standstill is not motion: a PCON in PIO mode
    with nothing on its PIO connector pauses at rest with the servo off, and the driver takes control back there."""
    return bool(st.moving or getattr(st, "homing", False))


def restore_pc_mode(ax: Any, say: Any, settle_timeout: float = 20.0, attempts: int = 3,
                    action_mode: bool = False) -> tuple[bool, dict[str, Any]]:
    """Take PC control back and confirm it with a status read. Only at rest (taking control mid-move stops a move): a moving
    axis is waited for, then stopped. Tried ``attempts`` times; a failing status read never skips the request."""
    rec: dict[str, Any] = {"attempts": 0}
    for attempt in range(1, attempts + 1):
        rec["attempts"] = attempt
        try:
            st = ax.status()
        except Exception as exc:  # noqa: BLE001
            st = None
            rec[f"status_error_{attempt}"] = f"{type(exc).__name__}: {exc}"
        if st is not None and st.modbus_enabled:
            return True, rec
        t0 = time.perf_counter()
        while st is not None and in_motion(st) and time.perf_counter() - t0 < settle_timeout:
            time.sleep(0.1)
            try:
                st = ax.status()
            except Exception:  # noqa: BLE001
                st = None
        if st is not None and in_motion(st):
            say("restore: still moving: decel stop so that PC control can be taken back")
            try:
                ax.stop(wait=True, timeout=5.0)
            except Exception as exc:  # noqa: BLE001
                rec[f"stop_error_{attempt}"] = f"{type(exc).__name__}: {exc}"
        try:
            if action_mode:
                ax.set_action_mode(True)
            else:
                ax.enable_modbus_control()
        except Exception as exc:  # noqa: BLE001
            rec[f"error_{attempt}"] = f"{type(exc).__name__}: {exc}"
            say(f"restore: taking PC control back failed ({rec[f'error_{attempt}']})")
        try:
            if ax.status().modbus_enabled:
                return True, rec
        except Exception as exc:  # noqa: BLE001
            rec[f"confirm_error_{attempt}"] = f"{type(exc).__name__}: {exc}"
        if attempt < attempts:
            say(f"restore: PC mode not confirmed (attempt {attempt} of {attempts}): trying again")
            time.sleep(0.5)
    rec["error"] = f"PC mode not confirmed after {attempts} attempts"
    return False, rec

