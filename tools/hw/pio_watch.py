"""pio-watch: the unit is handed to PIO mode and every change of its PIO inputs and outputs, of the move and position-complete flags
and of the position is shown while a person operates whatever is wired to the PIO connector.

    python tools/hw/run.py pio-watch --port COM3 [--seconds 60] [--stop-on-move] [--yes]
    python tools/hw/run.py pio-watch --sim [--sim-scenario move|idle|dipm-on|no-restore|interrupt|push|push-done] [--seconds 9]

Steps: identify and a read-only backup -> the unit must be at rest without an alarm -> the PIO input / output words are read once and
printed decoded (bit names from the driver's register maps) together with the stored position rows (a PIO move runs THOSE rows at
THEIR speeds; the speed cap of this harness does not apply to them) -> a person confirms -> PIO mode (ax.disable_modbus_control(), or
ax.set_action_mode(False) with --action-mode) -> for --seconds every change is printed with a time stamp -> at the end, on Ctrl+C or
on any error: wait for the axis to be at rest, switch back to PC mode and confirm Modbus control with a status read.

--stop-on-move: the first time a move starts while in PIO mode the driver STOP is sent once, and the result is recorded: whether the
STOP was accepted in PIO mode, whether and how fast the unit stopped, where it stopped against the stored rows' targets, and whether
a move ran again afterwards. A push row's move that comes to rest at the row's approach position, short of its push end, with the
move-end flag MEND 0 while the start input is still held was cut by the STOP (a push that runs on goes into its push zone); at rest
on a stored target with MEND 1 the move had finished: INCONCLUSIVE. Release the PIO start input right after the STOP and leave the inputs alone: a move after the release says nothing
about a held start input unless the input word is the same before and after and nobody touched an input.

Safety: the PIO input word is read again immediately before the switch to PIO mode and the switch is REFUSED when any input is on (the
unit would run a stored row at its own speed the moment PIO gets control). The restore of PC mode retries, and it runs on every exit
path. Nothing non-volatile is written (the PMSL and STOP coils only). Keep --seconds well under any limit that would kill the
process: a killed process cannot restore PC mode.

Exit codes: 0 PC mode confirmed back, 2 refused or could not start (also: a PIO input on at the hand-over), 6 PC mode NOT confirmed
back, 130 Ctrl+C (PC mode is restored first; 6 if that failed).
"""

from __future__ import annotations

import _thread
import argparse
import csv
import json
import sys
import threading
import time
import traceback
from collections.abc import Sequence
from types import SimpleNamespace
from typing import Any

from . import report
from .interactive import Interactive
from .python_edition import restore_pc_mode
from .report import PASS, Abort, jsonable, pos_str
from .session import EXIT_END, EXIT_INTERRUPTED, EXIT_REFUSED, SetupError, add_connection_arguments, confirm, finite_positive
from .summary import finish

SCENARIOS = ("move", "idle", "dipm-on", "no-restore", "interrupt", "push", "push-done")
WORDS = ("DIPM", "DOPM", "SIPM")
FLAGS = ("moving", "in_position", "target_reached", "pushing", "push_missed", "homing", "paused", "LS0", "LS1", "LS2", "servo_on", "homed",
         "alarm", "emergency_stop", "pmss")
CSV_COLUMNS = ("t_s", "wall", "pos_mm", *WORDS, *FLAGS)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tools/hw/run.py pio-watch", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_connection_arguments(p)
    p.add_argument("--seconds", type=finite_positive, default=60.0, metavar="S", help="how long to watch in PIO mode (default 60)")
    p.add_argument("--stop-on-move", action="store_true", help="send the driver STOP once, the first time a move starts in PIO mode, and record the result")
    p.add_argument("--poll", type=finite_positive, default=0.05, metavar="S", help="status poll interval (default 0.05 s)")
    p.add_argument("--pos-step", type=finite_positive, default=0.5, metavar="MM",
                   help="while moving, print a position change only when it is at least this big (default 0.5 mm); samples.csv has every poll")
    p.add_argument("--action-mode", action="store_true", help="toggle with set_action_mode(False / True) instead of disable / enable_modbus_control()")
    p.add_argument("--settle-timeout", type=finite_positive, default=20.0, metavar="S",
                   help="at the end wait this long for a running move to finish before the driver STOP is sent so that PC control can be taken back (default 20)")
    p.add_argument("--sim-scenario", choices=SCENARIOS, default="move",
                   help="--sim only: what the simulated PLC does (default move: ST1 then ST0; push / push-done: ST1 runs the gripper's push row 1 "
                        "and the STOP finds it in its push phase / after the push ran out, shown through the status words)")
    return p


class SimPlc(threading.Thread):
    """--sim only: a PLC on the PIO connector: a held start input (DIPM word override) and a PIO-commanded move."""

    def __init__(self, sim: Any, scenario: str, registers: Any, say: Any) -> None:
        super().__init__(daemon=True, name="sim-plc")
        self.sim, self.scenario, self.R, self.say = sim, scenario, registers, say
        self.ready = threading.Event()

    def _press(self, bit: int, name: str, target: float) -> None:
        self.say(f"   [sim PLC] {name} on (DIPM 0x{bit:04X}), start a move to {target:.1f} mm at 3 mm/s")
        self.sim.set_word_override(self.R.DIPM, bit)
        if not self.sim.pio_move(target, 3.0, 0.1, hold_start=True):
            self.say("   [sim PLC] (the simulated unit ignored the PIO start: Modbus still has control)")

    def _release(self, name: str) -> None:
        self.say(f"   [sim PLC] {name} off")
        self.sim.pio_release()
        self.sim.set_word_override(self.R.DIPM, 0)

    def _words(self, pos_mm: float, dss1: int, dsse: int) -> None:
        counts = round(pos_mm * 100) & 0xFFFFFFFF
        for address, value in ((self.R.PNOW, counts >> 16), (self.R.PNOW + 1, counts & 0xFFFF), (self.R.DSS1, dss1), (self.R.DSSE, dsse)):
            self.sim.set_word_override(address, value)

    def _push_row(self, done: bool) -> None:
        """Row 1 of the gripper profile (approach 4.10 mm, push end 9.50 mm) run over PIO, shown through the status words: ST1
        held, the move flags on at 3.00 mm. When the driver STOP comes, ``push`` rests at the approach position with MEND 0 and
        ST1 still held (the STOP cut the push phase, as seen on the gripper on 2026-10-31); ``push-done`` rests at the push end
        with MEND 1 and PSFL (the push had run out: the move finished)."""
        R = self.R
        base, pio = R.DSS1_PWR | R.DSS1_SV | R.DSS1_HEND, 0x32C2  # DSSE at rest in PIO mode (PMSS 0)
        self.say("   [sim PLC] ST1 on (DIPM 0x0002): row 1, a push row (approach 4.10 mm, push end 9.50 mm)")
        self.sim.set_word_override(R.DIPM, 0x0002)
        self._words(3.0, base, pio | R.DSSE_MOVE)
        t0 = time.monotonic()
        while not self.sim.stop_held and time.monotonic() - t0 < 5.0:
            time.sleep(0.005)
        if done:
            self._words(9.5, base | R.DSS1_BKRL | R.DSS1_PSFL | R.DSS1_TARGET_REACHED, pio)
        else:
            self._words(4.11, base | R.DSS1_BKRL | R.DSS1_PEND, pio)
        time.sleep(2.0)  # ST1 stays held through the STOP and after it
        self.say("   [sim PLC] ST1 off")
        self.sim.set_word_override(R.DIPM, 0)
        for address in (R.PNOW, R.PNOW + 1, R.DSS1, R.DSSE):
            self.sim.set_word_override(address, None)  # the modelled unit again (at rest where it was)

    def run(self) -> None:
        self.ready.wait()
        if self.scenario in ("idle", "dipm-on"):
            return
        if self.scenario in ("push", "push-done"):
            time.sleep(1.0)
            self._push_row(self.scenario == "push-done")
            return
        if self.scenario == "no-restore":  # the unit never reports PC control again: the failure path of the restore
            self.say("   [sim] the unit's status now always shows PIO mode: PC control cannot be confirmed")
            self.sim.set_word_override(self.R.DSSE, 0x32C2)
            return
        time.sleep(1.0)
        self._press(0x0002, "ST1", 9.0)
        if self.scenario == "interrupt":
            time.sleep(1.2)
            self.say("   [sim PLC] Ctrl+C is pressed in the middle of the move")
            _thread.interrupt_main()
            return
        time.sleep(2.6)
        self._release("ST1")
        time.sleep(1.0)
        self._press(0x0001, "ST0", 3.0)
        time.sleep(2.8)
        self._release("ST0")


def word_text(drv_info: Any, word: int, table: Any) -> str:
    on = drv_info.decode_bits(int(word), table)["on"]
    return f"0x{int(word):04X} [{' '.join(on) if on else '-'}]"


def snapshot(st: Any) -> dict[str, Any]:
    raw = st.raw
    return {"DIPM": int(raw.get("DIPM", 0)), "DOPM": int(raw.get("DOPM", 0)), "SIPM": raw.get("SIPM"), "moving": st.moving,
            "in_position": st.in_position, "target_reached": st.target_reached, "pushing": st.pushing, "push_missed": st.push_missed,
            "homing": st.homing, "paused": st.paused, "LS0": st.at_backward_end, "LS1": st.at_forward_end, "LS2": st.at_middle,
            "servo_on": st.servo_on, "homed": st.homed, "alarm": st.alarm_code, "emergency_stop": st.emergency_stop, "pmss": st.modbus_enabled}


def summarize_moves(samples: list[dict[str, Any]], targets: list[tuple[str, float]]) -> list[dict[str, Any]]:
    """The moves seen in the samples: start and end, positions, mean and peak apparent speed, the input word at the start."""
    moves: list[dict[str, Any]] = []
    cur: dict[str, Any] | None = None
    prev: dict[str, Any] | None = None
    for s in samples:
        if s["moving"] and (prev is None or not prev["moving"]):
            cur = {"t_start_s": s["t_s"], "pos_start_mm": prev["pos_mm"] if prev else s["pos_mm"], "DIPM_at_start": s["DIPM"], "pmss_at_start": s["pmss"], "points": []}
        if cur is not None:
            cur["points"].append((s["t_s"], s["pos_mm"]))
            if not s["moving"]:
                cur["t_end_s"], cur["pos_end_mm"] = s["t_s"], s["pos_mm"]
                moves.append(cur)
                cur = None
        prev = s
    if cur is not None:
        cur["t_end_s"], cur["pos_end_mm"] = cur["points"][-1]
        cur["unfinished"] = True
        moves.append(cur)
    out = []
    for m in moves:
        pts = m.pop("points")
        dur = m["t_end_s"] - m["t_start_s"]
        peak = 0.0
        for i, (t1, p1) in enumerate(pts):
            for t2, p2 in pts[i + 1:]:
                if t2 - t1 >= 0.1:
                    peak = max(peak, abs(p2 - p1) / (t2 - t1))
                    break
        m.update(duration_s=round(dur, 3), distance_mm=round(m["pos_end_mm"] - m["pos_start_mm"], 3),
                 mean_speed_mm_s=round(abs(m["pos_end_mm"] - m["pos_start_mm"]) / dur, 2) if dur > 0 else None,
                 peak_speed_mm_s=round(peak, 2) if peak else None,
                 ended_at=[label for label, mm in targets if abs(mm - m["pos_end_mm"]) <= 0.15] or None)
        out.append(m)
    return out


def push_row_cut(push_rows: list[dict[str, Any]], ends: list[str], final: dict[str, Any], dipm_start: int,
                 dipm_after: int | None) -> dict[str, Any] | None:
    """The push row whose move the STOP cut, or ``None``. ``final``: :func:`snapshot` of the status after the STOP plus
    ``pos_mm``. The axis is at rest at a push row's approach position (the stored target it matched), short of that row's
    push end, with MEND 0, no push in progress or missed, and the start input that began the move still held (the same,
    non-zero input word at the move start, after the STOP and now): a push that runs on goes into its push zone, so the
    STOP stopped it. A move at rest on any other target, or with MEND 1, is not judged here."""
    if final["moving"] or final["target_reached"] or final["pushing"] or final["push_missed"]:
        return None
    if not dipm_start or dipm_after != dipm_start or final["DIPM"] != dipm_start:
        return None
    rows = [p for p in push_rows if p["label"] in ends and abs(final["pos_mm"] - p["end_mm"]) > 0.15]
    if not rows or any(e not in {p["label"] for p in push_rows} for e in ends):
        return None
    return rows[0]


class PioWatch(Interactive):
    name = "pio-watch"

    def __init__(self, ctx: SimpleNamespace) -> None:
        super().__init__(ctx)
        self.tables: dict[str, Any] = {}
        self.targets: list[tuple[str, float]] = []
        self.push_rows: list[dict[str, Any]] = []  # each push row: n, its approach target's label, approach and push end
        self.switched = False
        self.restored: bool | None = None
        self.restore_rec: dict[str, Any] | None = None
        self.summary: dict[str, Any] | None = None
        self.stop_record: dict[str, Any] | None = None
        self.interrupted = False
        self.plc: SimPlc | None = None
        self.drv_info: Any = None

    # ------------------------------------------------------------------------------------------ the run
    def run(self) -> int:
        code = EXIT_REFUSED
        try:
            code = self._flow()
        except KeyboardInterrupt:
            code = EXIT_INTERRUPTED
            self.interrupted = True
            self.ctx.console.big("Ctrl+C: stopping; PC mode is restored next", "!")
        except SetupError as exc:
            self.say(f"REFUSED: {exc}")
            code = EXIT_REFUSED
        except Exception as exc:  # noqa: BLE001
            self.say(f"UNEXPECTED {type(exc).__name__}: {exc}")
            self.say(traceback.format_exc())
            code = EXIT_END
        finally:
            # the end sequence, on every path: PC mode back and confirmed, then the port closed, then the record
            self.guarded(self._restore_stage)
            self.guarded(self.close)
        if self.switched and not self.restored:
            code = EXIT_END
            self.guarded(lambda: self.hand_back(code, "PC mode was not confirmed back"))
        if self.summary is not None:
            self.guarded(self._print_summary)
        self.extra.update({"watch": self.summary, "restore": self.restore_rec, "stop_on_move": self.stop_record})
        self.guarded(lambda: finish(self.ctx, self, code))
        return code

    def _flow(self) -> int:
        rec, args = self.rec, self.args
        from iai_actuator_driver import info as drv_info
        from iai_actuator_driver import registers

        self.drv_info = drv_info
        self.ctx.sim_options = {}
        rec.step("open", "open the port and identify the unit (read only)", self.open_unit, critical=True)
        if self.raw is None:
            return EXIT_REFUSED
        rec.step("backup", "read-only backup and state", self.backup_step, critical=True)
        rec.step("state", "unit at rest, no alarm", self._state, critical=True)
        rec.step("pio_words", "read and decode the PIO words and the stored rows", self._words, critical=True)
        if rec.aborted:
            self.say(f"Stopped before PIO mode: {rec.aborted}. The unit was not changed.")
            return EXIT_REFUSED
        confirm(args, self.ctx.console, f"The unit is switched to PIO mode next and watched for {args.seconds:g} s. Whatever is wired to the PIO "
                                        "connector can then move the axis at the stored speeds (above). Keep fingers and tooling clear.")
        if args.sim and args.sim_scenario == "dipm-on":  # a PIO input goes on while the person is at the prompt: the hand-over must be refused
            self.say("   [sim PLC] ST0 goes on (DIPM 0x0001) after the PIO words were read, before the hand-over")
            self.sim.set_word_override(registers.DIPM, 0x0001)
        rec.step("to_pio", "switch to PIO mode", self._switch, critical=True)
        if rec.aborted:
            return EXIT_END if self.switched else EXIT_REFUSED
        if args.sim:
            self.plc = SimPlc(self.sim, args.sim_scenario, registers, self.say)
            self.plc.start()
        self.ctx.console.big(f"PIO MODE ACTIVE: operate the PIO input / PLC now (watching {args.seconds:g} s, Ctrl+C ends early)")
        if self.plc is not None:
            self.plc.ready.set()
        rec.step("watch", f"watch {args.seconds:g} s in PIO mode", self._watch)
        self.interrupted = bool(self.summary and self.summary["interrupted"])
        return EXIT_INTERRUPTED if self.interrupted else 0

    def _state(self) -> Any:
        st = self.raw.status()
        self.remember("state", st)
        self.start = {"position_mm": st.position_mm, "pc_mode": st.modbus_enabled, "servo_on": st.servo_on}
        problems = []
        if st.moving:
            problems.append("the axis is moving")
        if st.alarm:
            problems.append(f"fault alarm 0x{st.alarm_code:03X} (the harness never clears an alarm)")
        if problems:
            raise Abort("; ".join(problems) + " | " + pos_str(st))
        note = "" if st.modbus_enabled else " | NOTE: PC control was already off (PIO mode) at the start; PC mode is still restored at the end"
        return PASS, pos_str(st) + f" baud {self.raw.baudrate}" + note

    def _words(self) -> Any:
        say, raw, drv = self.say, self.raw, self.drv_info
        ui = drv.unit_info(raw, sections=("identity", "positions"))
        mode = (ui.get("identity") or {}).get("position_mode") or drv.position_mode(None, raw.device_info.model)
        tables = drv.io_bit_tables(mode)
        self.tables = {**tables, "SIPM": drv.BIT_TABLES["SIPM"]}
        rows = ((ui.get("positions") or {}).get("rows")) or []
        say(f"PIO / position mode : {mode.get('name')} (layout {mode.get('layout')}, {mode.get('source')}{'' if mode.get('confirmed') else ', not confirmed'})")
        say("stored rows (a PIO start runs these at THEIR OWN speed and acceleration; the speed cap of this harness does not apply):")
        for r in rows:
            if str(r.get("role", "")).startswith("unused"):
                continue
            line = f"  row {r['n']}: {r.get('role')}: {r.get('position_mm')} mm, speed {r.get('speed_mm_s')} mm/s, accel {r.get('accel_g')} G"
            self.targets.append((f"row {r['n']} {r.get('position_mm')} mm", float(r["position_mm"])))
            if r.get("push"):
                line += f", PUSH {r.get('push_pct')} % force over {r.get('band_mm')} mm to {r.get('push_end_mm')} mm ({r.get('push_direction')})"
                if r.get("push_end_mm") is not None:
                    self.targets.append((f"row {r['n']} push end {r['push_end_mm']} mm", float(r["push_end_mm"])))
                    self.push_rows.append({"n": r["n"], "label": f"row {r['n']} {r.get('position_mm')} mm", "approach_mm": float(r["position_mm"]),
                                           "end_mm": float(r["push_end_mm"])})
            say(line)
        st = raw.status()
        snap = snapshot(st)
        say("PIO words (read once):")
        say(f"  PIO inputs  DIPM = {word_text(drv, snap['DIPM'], self.tables['DIPM'])}")
        say(f"  PIO outputs DOPM = {word_text(drv, snap['DOPM'], self.tables['DOPM'])}")
        say(f"  input status SIPM = {word_text(drv, snap['SIPM'] or 0, self.tables['SIPM'])}")
        for name, table in (("DIPM", tables["DIPM"]), ("DOPM", tables["DOPM"])):
            say(f"  {name} bits:")
            for b in sorted(table, key=lambda d: -d.bit):
                say(f"      b{b.bit:<2} {b.name:<5} {b.meaning}{'  [active low: 1 = normal]' if b.active_low else ''}{'' if b.confirmed else '  (name unconfirmed)'}")
        report.write_json(self.ctx.out / "pio_words_start.json", {"mode": mode, "rows": rows, "DIPM": drv.decode_bits(snap["DIPM"], tables["DIPM"]),
                                                                    "DOPM": drv.decode_bits(snap["DOPM"], tables["DOPM"]), "status": st})
        if snap["DIPM"]:
            raise Abort(f"a PIO input is on right now (DIPM {word_text(drv, snap['DIPM'], self.tables['DIPM'])}): the unit would act on it the moment PIO "
                        "gets control. Switch the PIO inputs off and run again")
        return PASS, f"DIPM 0x{snap['DIPM']:04X}, DOPM 0x{snap['DOPM']:04X}; {len(self.targets)} row targets known"

    def _switch(self) -> Any:
        # The input word is read AGAIN right before the hand-over: the confirmation prompt can wait for any time, and an input that went on
        # meanwhile would make the unit run a stored row (at up to its own maximum speed) the moment PIO gets control.
        st = self.raw.status()
        dipm = (st.raw or {}).get("DIPM")
        if dipm is None:
            raise Abort("the status read right before the hand-over has no DIPM word: nobody can tell whether a PIO input is on, so the unit stays in PC mode")
        if dipm != 0:
            raise Abort(f"a PIO input is on at the hand-over (DIPM {word_text(self.drv_info, int(dipm), self.tables['DIPM'])}): NOT switching to PIO mode, the unit "
                        "stays in PC mode. Switch the PIO inputs off and start again")
        self.switched = True  # from here the end sequence takes PC control back, whatever happens
        if self.args.action_mode:
            self.raw.set_action_mode(False)
            how = "set_action_mode(False)"
        else:
            self.raw.disable_modbus_control()
            how = "disable_modbus_control()"
        st = self.raw.status()
        self.remember("switch to PIO", st)
        if st.modbus_enabled:
            raise Abort(f"{how} was sent but PC control is still on: the unit is not in PIO mode")
        return PASS, f"{how}: PIO mode, " + pos_str(st)

    # ------------------------------------------------------------------------------------------ the watch
    def _state_text(self, snap: dict[str, Any], st: Any) -> str:
        d, t = self.drv_info, self.tables
        return (f"pos {st.position_mm:.2f} mm | DIPM {word_text(d, snap['DIPM'], t['DIPM'])} DOPM {word_text(d, snap['DOPM'], t['DOPM'])} | moving "
                f"{int(snap['moving'])} inpos {int(snap['in_position'])} mend {int(snap['target_reached'])} | PC {int(snap['pmss'])}")

    def _describe(self, key: str, old: Any, new: Any) -> str:
        if key in WORDS and old is not None and new is not None:
            a = set(self.drv_info.decode_bits(int(old), self.tables[key])["on"])
            b = set(self.drv_info.decode_bits(int(new), self.tables[key])["on"])
            names = " ".join([f"+{n}" for n in sorted(b - a)] + [f"-{n}" for n in sorted(a - b)])
            return f"{key} 0x{int(old):04X}->0x{int(new):04X} ({names or 'unnamed bit'})"
        if isinstance(new, bool) or isinstance(old, bool):
            return f"{key} {'?' if old is None else int(old)}->{'?' if new is None else int(new)}"
        return f"{key} {old}->{new}"

    def _watch(self) -> Any:
        args, ax, say, out = self.args, self.raw, self.say, self.ctx.out
        samples: list[dict[str, Any]] = []
        events: list[dict[str, Any]] = []
        errors = consecutive = polls = 0
        prev: dict[str, Any] | None = None
        last_pos_printed: float | None = None
        stop_done = interrupted = False
        inputs_seen: dict[int, float] = {}
        outputs_seen: dict[int, float] = {}
        watch_error = None
        csv_file = open(out / "samples.csv", "w", newline="", encoding="utf-8", buffering=1)
        writer = csv.writer(csv_file)
        writer.writerow(CSV_COLUMNS)
        self.t0 = time.perf_counter()
        deadline = self.t0 + args.seconds
        next_beat = 10.0
        try:
            while time.perf_counter() < deadline:
                tick = time.perf_counter()
                try:
                    st = ax.status()
                    consecutive = 0
                except Exception as exc:  # noqa: BLE001
                    errors += 1
                    consecutive += 1
                    if errors <= 3 or errors % 20 == 0:
                        say(f"[{self.stamp()}] status read failed ({type(exc).__name__}: {exc}) [{errors} so far]")
                    if consecutive >= 10:
                        say("10 failed reads in a row: ending the watch (the unit or the link is gone)")
                        watch_error = f"{consecutive} failed reads in a row: {exc}"
                        break
                    time.sleep(args.poll)
                    continue
                polls += 1
                snap = snapshot(st)
                t = round(time.perf_counter() - self.t0, 3)
                row = {"t_s": t, "wall": time.strftime("%H:%M:%S"), "pos_mm": st.position_mm, **snap}
                samples.append(row)
                writer.writerow([row[c] if row[c] is not None else "" for c in CSV_COLUMNS])
                inputs_seen.setdefault(snap["DIPM"], t)
                outputs_seen.setdefault(snap["DOPM"], t)
                changes = [self._describe(k, prev[k], snap[k]) for k in (*WORDS, *FLAGS) if prev is not None and snap[k] != prev[k]]
                move_start = prev is not None and snap["moving"] and not prev["moving"]
                move_end = prev is not None and prev["moving"] and not snap["moving"]
                pos_changed = last_pos_printed is None or abs(st.position_mm - last_pos_printed) >= args.pos_step
                if prev is None:
                    say(f"[{self.stamp()}] first sample: " + self._state_text(snap, st))
                    last_pos_printed = st.position_mm
                elif changes or (pos_changed and snap["moving"]):
                    tag = "  >>> MOVE STARTED" if move_start else "  >>> MOVE ENDED" if move_end else ""
                    what = "; ".join(changes) if changes else f"pos {last_pos_printed:.2f}->{st.position_mm:.2f}"
                    say(f"[{self.stamp()}] {what}{tag}   | " + self._state_text(snap, st))
                    events.append({"t_s": t, "changes": changes, "pos_mm": st.position_mm, "move_start": move_start, "move_end": move_end})
                    last_pos_printed = st.position_mm
                elif t >= next_beat:
                    say(f"[{self.stamp()}] (no change) " + self._state_text(snap, st) + f"  polls {polls}")
                if t >= next_beat:
                    next_beat = t + 10.0
                if args.stop_on_move and not stop_done and move_start and not snap["pmss"]:
                    stop_done = True
                    self.stop_record = self._stop(st)
                    prev, last_pos_printed = None, None  # the STOP took its own time: restart the change detection
                    continue
                prev = snap
                rest = args.poll - (time.perf_counter() - tick)
                if rest > 0:
                    time.sleep(rest)
        except KeyboardInterrupt:
            interrupted = True
            self.ctx.console.big("Ctrl+C: ending the watch, PC mode is restored next", "!")
        finally:
            csv_file.close()
        moves = summarize_moves(samples, self.targets)
        report.write_json(out / "events.json", events)
        d = self.drv_info
        self.summary = {"polls": polls, "errors": errors, "seconds": round(time.perf_counter() - self.t0, 2), "interrupted": interrupted, "watch_error": watch_error,
                        "inputs_seen": {f"0x{k:04X}": {"first_t_s": v, "on": d.decode_bits(k, self.tables["DIPM"])["on"]} for k, v in inputs_seen.items()},
                        "outputs_seen": {f"0x{k:04X}": {"first_t_s": v, "on": d.decode_bits(k, self.tables["DOPM"])["on"]} for k, v in outputs_seen.items()},
                        "moves": moves, "events": len(events)}
        sm = self.summary
        return PASS, (f"{sm['seconds']:.1f} s, {polls} polls, {errors} read errors, {len(events)} printed changes, {len(moves)} move(s)"
                      + (", ended by Ctrl+C" if interrupted else "")), sm

    def _stop(self, st0: Any) -> dict[str, Any]:
        """The driver STOP on a move that started in PIO mode; what happened, as a record."""
        ax, say = self.raw, self.say
        rec: dict[str, Any] = {"t_s": round(time.perf_counter() - self.t0, 3), "pos_at_detection_mm": st0.position_mm, "pmss_before": st0.modbus_enabled}
        say(f"[{self.stamp()}] STOP-ON-MOVE: a move started in PIO mode (pos {st0.position_mm:.2f} mm): sending the driver STOP now")
        t0 = time.perf_counter()
        try:
            r = ax.stop(wait=True, timeout=3.0)
            rec.update(accepted=True, error=None, stop_call_s=round(time.perf_counter() - t0, 3), pos_after_call_mm=None if r is None else r.position_mm,
                       moving_after_call=None if r is None else r.moving)
        except Exception as exc:  # noqa: BLE001
            timeout = isinstance(exc, self.iad.MotionTimeoutError)
            rec.update(accepted=timeout, error=f"{type(exc).__name__}: {exc}", stop_call_s=round(time.perf_counter() - t0, 3), still_moving_after_stop_timeout=timeout)
        stopped = rec.get("moving_after_call") is False
        restarted, seen_still = False, stopped
        dipm_start = snapshot(st0)["DIPM"]
        dipm_after: int | None = None  # the first input word read after the release (the sampler is blind while the STOP is held)
        t1 = time.perf_counter()
        while time.perf_counter() - t1 < 1.0:
            try:
                s = ax.status()
            except Exception as exc:  # noqa: BLE001
                rec["aftermath_error"] = f"{type(exc).__name__}: {exc}"
                break
            if dipm_after is None:
                dipm_after = snapshot(s)["DIPM"]
            if not s.moving:
                seen_still = True
            elif seen_still:
                restarted = True
            time.sleep(0.05)
        final = ax.status()
        ends = [lbl for lbl, mm in self.targets if abs(mm - final.position_mm) <= 0.15]
        final_snap = {**snapshot(final), "pos_mm": final.position_mm}
        rec.update(final_pos_mm=final.position_mm, final_moving=final.moving, pmss_after=final.modbus_enabled, restarted_after_stop=restarted,
                   stopped_at_row_target=ends or None, dipm_at_move_start=dipm_start, dipm_after_release=dipm_after,
                   mend_after=final_snap["target_reached"], dipm_final=final_snap["DIPM"])
        cut = push_row_cut(self.push_rows, ends, final_snap, dipm_start, dipm_after)
        rec["push_row_cut"] = None if cut is None else cut["n"]
        stop_pos = rec.get("pos_after_call_mm")
        if rec["pmss_after"] != rec["pmss_before"]:
            verdict = f"INVALID: PC control changed {int(rec['pmss_before'])} -> {int(rec['pmss_after'])} during the STOP (switching it on mid-move stops a move by itself)"
        elif not rec["accepted"]:
            verdict = f"STOP NOT ACCEPTED in PIO mode: {rec['error']}"
        elif rec.get("still_moving_after_stop_timeout") or (not stopped and not seen_still):
            verdict = f"STOP written without error, but the axis kept moving afterwards ({rec.get('error') or 'still moving'})"
        elif restarted:
            if dipm_after is None:
                inputs = "the input word was not read after the release"
            elif dipm_after == dipm_start:
                inputs = f"the input word was 0x{dipm_start:04X} at the start of the move and again after the release"
            else:
                inputs = f"the input word was 0x{dipm_start:04X} at the start of the move and 0x{dipm_after:04X} after the release"
            verdict = (f"STOP ACCEPTED in PIO mode: the unit stopped at {stop_pos} mm ({rec['stop_call_s']:.2f} s after the STOP), and a move ran after the "
                       f"release; {inputs} (not sampled while the STOP was held), so the cause of that move (a start input held through the STOP, or an input "
                       "changed meanwhile) is not established")
        elif cut is not None:
            verdict = (f"STOP ACCEPTED in PIO mode: the move of push row {cut['n']} came to rest at {final.position_mm:.2f} mm, its approach position, "
                       f"short of its push end {cut['end_mm']:.2f} mm, with the move-end flag MEND 0 while the start input (0x{dipm_start:04X}) was "
                       f"still held, {rec['stop_call_s']:.2f} s after the STOP was sent (detected at {rec['pos_at_detection_mm']:.2f} mm)")
        elif ends:
            verdict = (f"STOP written without error; the unit is at rest at {final.position_mm:.2f} mm = {', '.join(ends)}: the move had already finished "
                       "(or ignored the STOP): INCONCLUSIVE, repeat with a slower or longer move")
        else:
            verdict = (f"STOP ACCEPTED in PIO mode: the unit stopped at {stop_pos} mm, short of every stored target, {rec['stop_call_s']:.2f} s after the STOP "
                       f"was sent (detected at {rec['pos_at_detection_mm']:.2f} mm)")
        rec["verdict"] = verdict
        say(f"[{self.stamp()}] STOP RESULT: {verdict}")
        return rec

    # ------------------------------------------------------------------------------------------ the end
    def _restore_stage(self) -> None:
        """PC mode back and confirmed (only when this run switched to PIO). Safe to repeat: the restore itself runs once."""
        if not (self.switched and self.raw is not None):
            return
        if self.restored is None:
            self.ctx.console.big("RESTORING PC MODE", "-")
            try:
                self.restored, self.restore_rec = restore_pc_mode(self.raw, self.say, settle_timeout=self.args.settle_timeout,
                                                                  action_mode=self.args.action_mode)
            except KeyboardInterrupt:
                self.say("Ctrl+C again: still restoring PC mode")
                raise
            except BaseException as exc:  # noqa: BLE001
                self.restored, self.restore_rec = False, {"error": f"{type(exc).__name__}: {exc}"}
        if self.restored:
            self.ctx.console.big("PC MODE RESTORED: Modbus control is back, confirmed by a status read", "=")
        else:
            self.ctx.console.big("PC MODE NOT CONFIRMED: the unit may still be in PIO mode", "!")
        if not any(r["key"] == "restore" for r in self.rec.rows):
            self.rec.n += 1
            self.rec.add("restore", "back to PC mode, confirmed", PASS if self.restored else report.FAIL, json.dumps(jsonable(self.restore_rec)))
        nv = getattr(self.raw, "nonvolatile_writes", None)
        self.end_info.update({"nonvolatile_writes": nv, "pc_mode_restored": self.restored})
        if nv:
            self.rec.add("nv", "no non-volatile write", report.FAIL, f"{nv} non-volatile writes")

    def _print_summary(self) -> None:
        sm, say = self.summary, self.say
        say("=" * 72)
        say("PIO WATCH SUMMARY")
        say(f"  watched {sm['seconds']:.1f} s, {sm['polls']} polls, {sm['errors']} read errors" + (", ended by Ctrl+C" if sm["interrupted"] else ""))
        say("  PIO input words seen : " + "; ".join(f"{k} {v['on'] or '-'} (first at +{v['first_t_s']:.1f} s)" for k, v in sm["inputs_seen"].items()))
        say("  PIO output words seen: " + "; ".join(f"{k} {v['on'] or '-'}" for k, v in sm["outputs_seen"].items()))
        if not sm["moves"]:
            say("  moves                : none seen")
        for i, m in enumerate(sm["moves"], 1):
            say(f"  move {i}: +{m['t_start_s']:.2f} s {m['pos_start_mm']:.2f} -> {m['pos_end_mm']:.2f} mm in {m['duration_s']:.2f} s (mean "
                f"{m['mean_speed_mm_s']} mm/s, peak {m['peak_speed_mm_s']} mm/s), input word at start 0x{m['DIPM_at_start']:04X}, PC {int(m['pmss_at_start'])}, "
                f"ended at {m['ended_at'] or 'no row target'}" + (" (still moving at the end)" if m.get("unfinished") else ""))
        if self.stop_record:
            say("  STOP test            : " + self.stop_record["verdict"])
        say(f"  PC mode restored     : {self.restored}")
        say("=" * 72)


def main(argv: Sequence[str]) -> int:
    from .main import make_context

    args = build_parser().parse_args(list(argv))
    try:
        ctx = make_context(args, "pio-watch", argv)
    except SetupError as exc:
        print(f"REFUSED (exit {EXIT_REFUSED}): {exc}", file=sys.stderr)
        return EXIT_REFUSED
    try:
        return PioWatch(ctx).run()
    finally:
        ctx.close()

