"""The command-line editions of the acceptance run: the C++ command line (``--edition cli``) and the C# command line
(``--edition csharp``), each run as a program, one command at a time.

The same rules as the Python edition: a read-only backup first, every move inside the test band at the capped speed, no
command that writes parameters or position rows (only the commands in ALLOWED are ever run), and the unit is put back as it
was found. When the program cannot be found the run ends with one SKIP line.

On a real port the state lives in the unit, so each command sees the effect of the one before. The simulator of the C++
command line keeps the position between runs (hidden ``--sim-state``); the simulator of the C# command line keeps nothing, so
the steps that need the position of the step before are SKIPped there, and the start position is the profile's own.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from . import report
from .common import TOL_MM, Geometry, StepDef, alarm_at_end, band_from_stroke, compare_backups, near, servo_across_toggle
from .report import CHECK, FAIL, PASS, Abort, Check, Skip
from .session import ACCEL_G, EXIT_FAIL, EXIT_INTERRUPTED, EXIT_OK, EXIT_REFUSED, SetupError, confirm
from .summary import finish

# the only commands the harness runs: reads, moves, servo and mode (never param set, teach, swreset, baud, alarm-reset)
ALLOWED = frozenset({"status", "identify", "info", "limits", "monitor", "alarms", "backup", "check", "recover", "home", "move", "jog",
                     "push", "stop", "servo", "mode", "param"})
ALLOWED_PARAM = frozenset({"list", "get"})

STEPS: tuple[StepDef, ...] = (
    StepDef("status", "status", why="the status reads and shows no alarm"),
    StepDef("limits", "limits", why="the speed cap and the stroke band are as requested"),
    StepDef("info", "info --section identity", why="the unit identifies itself"),
    StepDef("home", "home if the unit is not homed", changes=True, motion=True, critical=True, why="a unit that is not homed homes"),
    StepDef("move_to", "move", changes=True, needs="band", motion=True, critical=True, why="an absolute move arrives within 0.1 mm"),
    StepDef("jog", "jog", changes=True, needs="band", motion=True, critical=True, why="small relative steps out and back"),
    StepDef("decel", "move --decel", changes=True, needs="band", motion=True, critical=True, why="a move with its own deceleration arrives"),
    StepDef("pattern", "move --accel-pattern s_motion", changes=True, needs="band", motion=True, critical=True,
            why="the move runs, or the command line refuses the pattern (the C# command line has no such option)"),
    StepDef("speed", "move --change-speed", changes=True, needs="band", motion=True, critical=True, why="a move whose speed is raised half way arrives"),
    StepDef("fast", "move --fast", changes=True, needs="band", motion=True, critical=True, why="fast completion arrives within 0.15 mm"),
    StepDef("stop", "move --no-wait, then stop", changes=True, needs="slow", motion=True, critical=True,
            why="stop from a second process ends a slow move short of its target"),
    StepDef("push", "push into free air", changes=True, needs="band", motion=True, critical=True, why="a push with nothing in the way ends as a miss"),
    StepDef("params", "param list", why="the parameters decode (read only)"),
    StepDef("monitor", "monitor --watch --duration 1", why="the status monitor runs for a second"),
    StepDef("check", "check", why="the read-only self-test has no failing line"),
    StepDef("recover", "recover --backup FILE", why="with the link up recover reports the same unit and changes nothing"),
    StepDef("pc_pio", "mode pio, mode pc", changes=True, motion=True, critical=True, why="PIO mode then PC mode, each reported by the command"),
)
STEP_KEYS = tuple(s.key for s in STEPS)
FIXED = (("open", "find the program and identify the unit (read only)"),
         ("backup", "read-only backup before anything changes"),
         ("preflight", "start state, test band"),
         ("prepare", "PC control and servo on, when the unit was found without them"),
         ("end", "back to the start state, compared with a second read-only backup"))
NAMES = {"cli": ("iai-actuator-driver-cpp.exe", "iai-actuator-driver-cpp"),
         "csharp": ("iai-actuator-driver-cs.exe", "iai-actuator-driver-cs", "iai-actuator-driver-cs.dll")}
NEEDS_POSITION = frozenset({"jog", "push"})  # steps that start from where the step before left the axis
NEEDS_REAL_UNIT = frozenset({"stop"})  # a move that goes on after its process has ended, stopped from another process


def default_steps() -> list[str]:
    return [s.key for s in STEPS if s.default]


@dataclass
class Result:
    rc: int
    out: str
    err: str
    secs: float
    args: list[str]

    def json(self) -> Any:
        """The JSON the command printed under --json. The whole of stdout must be that one document: a note line before it
        is a defect of the command line (a note belongs on stderr), so it gives None and the step that needs the JSON fails."""
        try:
            return json.loads(self.out)
        except ValueError:
            return None

    def tail(self) -> str:
        return (self.err.strip() or self.out.strip())[-300:]


class CommandFailed(Exception):
    pass


def find_program(edition: str, explicit: str | None, native: str | None) -> list[str] | None:
    """The command (a list: ``[exe]`` or ``[dotnet, dll]``) or None."""
    root = Path(__file__).resolve().parents[2]
    candidates: list[Path] = []
    if explicit:  # a named program is the only one that is tried
        path = Path(explicit)
        if not path.is_file():
            return None
        return [shutil.which("dotnet") or "dotnet", str(path)] if path.suffix.lower() == ".dll" else [str(path)]
    env = os.environ.get("IAI_HW_CLI_CPP" if edition == "cli" else "IAI_HW_CLI_CS")
    if env:
        candidates.append(Path(env))
    folders = [Path(native)] if native and Path(native).is_dir() else []
    folders += [root / "core" / "build"] if edition == "cli" else sorted((root / "bindings" / "dotnet" / "src" / "IaiActuatorDriver.Cli" / "bin").glob("*/*"))
    for folder in folders:
        candidates += [folder / n for n in NAMES[edition]]
    for name in NAMES[edition]:
        found = shutil.which(name)
        if found:
            candidates.append(Path(found))
    for path in candidates:
        if path.is_file():
            if path.suffix.lower() == ".dll":
                dotnet = shutil.which("dotnet")
                if dotnet:
                    return [dotnet, str(path)]
                continue
            return [str(path)]
    return None


class CliEdition:
    def __init__(self, ctx: SimpleNamespace, selected: list[str], edition: str) -> None:
        self.ctx, self.args, self.say = ctx, ctx.args, ctx.console.say
        self.name = "cli (C++)" if edition == "cli" else "csharp (C#)"
        self.kind = edition
        self.selected = [k for k in STEP_KEYS if k in selected]
        self.rec = report.Recorder(self.say, stop_motion=self._stop_motion)
        self.program: list[str] | None = None
        self.start: dict[str, Any] = {}
        self.band: tuple[float, float] | None = None
        self.geo: Geometry | None = None
        self.motion_ok, self.motion_why = False, "the start state was not read"
        self.changed = self.pio_handed = False
        self.servo_was_off = self.took_control = False
        self.end_info: dict[str, Any] = {}
        self.compare: dict[str, Any] | None = None
        self.notes: list[str] = []
        self.recover_cleared: set[int] = set()  # the alarm codes recover reset (its report's alarms_cleared)
        self.log: list[dict[str, Any]] = []
        self.backup_start: dict[str, Any] | None = None
        self.caps: list[str] = []
        self.sim_state: Path | None = None
        self.persist = not self.args.sim or self.kind == "cli"
        self.extra: dict[str, Any] = {"cli_log": self.log}

    # ------------------------------------------------------------------------------------------ running commands
    def cli(self, *args: str, timeout: float = 90.0, plain: bool = False) -> Result:
        """Run one command of the program. ``plain``: without --json."""
        assert self.program is not None
        words = list(args)
        command = next((w for w in words if not w.startswith("-")), "")
        if command not in ALLOWED or (command == "param" and (len(words) < 2 or words[words.index("param") + 1] not in ALLOWED_PARAM)):
            raise SetupError(f"the harness does not run the command {' '.join(words)!r}")
        a = self.args
        conn = ["--sim", *( ["--sim-profile", a.sim_profile] if self.kind == "cli" else [a.sim_profile])] if a.sim else ["--port", a.port]
        if a.sim and self.kind == "cli":
            conn += ["--sim-baud", str(a.sim_baud), "--sim-time-scale", f"{a.sim_time_scale:g}"]
            if self.sim_state is not None:
                conn += ["--sim-state", str(self.sim_state)]
        cmd = [*self.program, *conn, "--slave", str(a.slave), *self.caps, *([] if plain else ["--json"]), *words]
        env = dict(os.environ)
        if self.kind == "csharp":  # the C# program finds its own copy of the core unless a core file is named
            native = (Path(a.native) if Path(a.native).suffix.lower() in (".dll", ".so", ".dylib") else Path(a.native) / "iai_actuator_driver.dll") if a.native else None
            if native is not None and native.is_file():
                env["IAI_ACTUATOR_DRIVER_NATIVE"] = str(native)
            else:
                env.pop("IAI_ACTUATOR_DRIVER_NATIVE", None)
        t0 = time.perf_counter()
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env, encoding="utf-8", errors="replace")
            result = Result(p.returncode, p.stdout, p.stderr, time.perf_counter() - t0, cmd)
        except subprocess.TimeoutExpired as exc:
            partial = exc.stdout.decode("utf-8", "replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
            result = Result(-1, partial, f"timed out after {timeout:g} s", time.perf_counter() - t0, cmd)
        self.log.append({"args": words, "rc": result.rc, "s": round(result.secs, 2), "stdout": result.out[-3000:], "stderr": result.err[-1500:]})
        return result

    def status(self) -> dict[str, Any]:
        r = self.cli("status")
        data = r.json()
        if r.rc != 0 or not isinstance(data, dict):
            raise CommandFailed(f"status failed (rc {r.rc}): {r.tail()}")
        return data

    def ok(self, r: Result) -> dict[str, Any]:
        """The JSON a command printed, or CommandFailed."""
        data = r.json()
        if r.rc != 0 or not isinstance(data, dict):
            why = "stdout is not one JSON document" if r.rc == 0 else f"rc {r.rc}"
            raise CommandFailed(f"{' '.join(r.args[-4:])}: {why}: {r.tail()}")
        return data

    def go(self, target: float, speed: float | None = None) -> dict[str, Any]:
        g = self.geo
        return self.ok(self.cli("move", f"{target:.3f}", "--speed", f"{speed or g.v:g}", "--accel", f"{g.acc:g}", timeout=g.timeout(g.d, speed or g.v) + 30))

    def _stop_motion(self) -> Any:
        if self.program is None:
            return None
        return self.cli("stop", timeout=30).tail()

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
            self._guarded(self._end_sequence)
        if code in (EXIT_OK, EXIT_FAIL) and any(r["verdict"] == report.INTR for r in self.rec.rows):
            code = EXIT_INTERRUPTED
        elif code == EXIT_OK and any(r["verdict"] == FAIL for r in self.rec.rows):
            code = EXIT_FAIL
        self._guarded(lambda: finish(self.ctx, self, code))
        return code

    def _guarded(self, fn: Any) -> None:
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
        if self.program is None:
            return EXIT_OK  # the program is not there: one SKIP line, not a failure
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
        for sd in chosen:
            rec.step(sd.key, sd.title, lambda sd=sd: self._call(sd), motion=sd.motion, critical=sd.critical)
        return EXIT_OK

    def _call(self, sd: StepDef) -> Any:
        if sd.needs and not self.motion_ok:
            raise Skip(self.motion_why)
        if sd.needs == "slow" and self.geo is not None and not self.geo.slow_usable:
            raise Skip(f"the band is too small for a move that is stopped half way (travel {self.geo.d:.2f} mm, need 2 mm)")
        if sd.key in NEEDS_REAL_UNIT and self.args.sim:
            raise Skip("a simulated unit ends with the process that started it, so a move cannot be stopped from a second process: run it on a real unit")
        if sd.key in NEEDS_POSITION and not self.persist:
            raise Skip("the simulator of this command line keeps no state between runs: a step that starts where the one before ended cannot run")
        if sd.key == "pattern" and self.kind == "csharp":
            raise Skip("the C# command line has no --accel-pattern option")
        return getattr(self, "s_" + sd.key)()

    # ------------------------------------------------------------------------------------------ fixed phases
    def _open(self) -> Any:
        self.program = find_program(self.kind, self.args.cli_exe, self.args.native)
        if self.program is None:
            raise Skip(f"the {'C++' if self.kind == 'cli' else 'C#'} command line was not found: build it, put it on PATH, or give "
                       "--cli-exe PATH (nothing was run)")
        version = subprocess.run([*self.program, "--version"], capture_output=True, text=True, timeout=30).stdout.strip().splitlines()
        self.ctx.driver = {"program": " ".join(self.program), "version": version[0] if version else ""}
        if self.args.sim and self.kind == "cli" and self.args.sim_profile == "gripper":
            self.sim_state = self.ctx.out / "sim_state.txt"
            self.sim_state.write_text("position 3.0\n", encoding="utf-8", newline="\n")
        r = self.cli("info", "--section", "identity")
        data = r.json()
        if r.rc != 0 or not isinstance(data, dict):
            raise Abort(f"info --section identity failed (rc {r.rc}): {r.tail()}")
        ident = data.get("identity") or {}
        unit = {"model": ident.get("model_name") or ident.get("user_name"), "serial_number": ident.get("serial_number"),
                "firmware": ident.get("firmware")}
        self.ctx.unit = unit
        return PASS, f"{self.ctx.driver['program']} {self.ctx.driver['version']} | {unit['model']} | serial {unit['serial_number']} | firmware {unit['firmware']}", unit

    def _backup(self) -> Any:
        path = self.ctx.out / "backup_start.json"
        r = self.cli("backup", str(path), "--rows", "all", plain=True, timeout=300)
        if r.rc != 0 or not path.is_file():
            raise Abort(f"backup failed (rc {r.rc}): {r.tail()}: nothing is changed without a backup")
        self.backup_start = json.loads(path.read_text(encoding="utf-8"))
        errors = self.backup_start.get("errors")
        text = f"backup_start.json written ({path.stat().st_size} bytes), errors {errors or 'none'}"
        return (CHECK if errors else PASS), text + (" (the backup is partial)" if errors else "")

    def _preflight(self) -> Any:
        st = self.status()
        self.start = {"position_mm": st["position_mm"], "servo_on": st["servo_on"], "homed": st["homed"], "pc_mode": st["modbus_enabled"],
                      "alarm_code": st["alarm_code"], "dipm": (st.get("raw") or {}).get("DIPM")}
        problems = []
        if st["alarm"]:
            problems.append(f"alarm 0x{st['alarm_code']:03X} (the harness never clears an alarm)")
        if st.get("emergency_stop"):
            problems.append("an emergency stop is active")
        if st["moving"]:
            problems.append("the axis is moving")
        limits = self.cli("limits").json() or {}
        position = limits.get("position") or {}
        stroke = (position["min"], position["max"]) if "min" in position and "max" in position else None
        self.band = band_from_stroke(self.args.stroke_band, stroke)
        if self.band is not None:
            self.caps = ["--stroke", f"{self.band[0]:.3f}", f"{self.band[1]:.3f}"]
        self.caps = ["--max-speed", f"{self.args.speed_cap:g}", "--max-accel", f"{ACCEL_G:g}", *self.caps]
        limits = self.cli("limits").json() or {}
        position = limits.get("position") or {}
        eff = (position["min"], position["max"]) if "min" in position else None
        self.limits_json = limits
        if eff is None:
            self.motion_why = "the unit's stroke is not known: give the band in mm with --stroke-band LOW:HIGH"
        else:
            accel_cap = float((limits.get("accel") or {}).get("max") or ACCEL_G)
            unit_speed = float((limits.get("speed") or {}).get("max") or self.args.speed_cap)  # already lowered to the cap and the unit's maximum
            self.geo = Geometry(eff[0], eff[1], min(self.args.speed_cap, unit_speed), accel_cap)
            if not self.geo.usable:
                self.motion_why = f"the band {eff[0]:.2f} .. {eff[1]:.2f} mm is too small to test in (travel {self.geo.d:.2f} mm, need 1 mm)"
            elif st["homed"] and not self.geo.inside(st["position_mm"]):
                self.motion_why = (f"the unit is at {st['position_mm']:.2f} mm, outside the test band {eff[0]:.2f} .. {eff[1]:.2f} mm; move it into "
                                   "the band by hand, or give a band that includes it (for example --stroke-band 0:10000 = the whole stroke) so "
                                   "that it can be put back where it was")
            elif not st["homed"] and "home" not in self.selected:
                self.motion_why = "the unit is not homed and the home step is not selected"
            else:
                self.motion_ok = True
        text = (f"pos {st['position_mm']:.2f} servo {int(st['servo_on'])} homed {int(st['homed'])} alarm 0x{st['alarm_code']:03X} pc "
                f"{int(st['modbus_enabled'])}; " + (self.geo.describe() if self.geo else "no test band"))
        if problems:
            raise Abort("not touching the unit: " + "; ".join(problems) + " | " + text)
        if any(s.needs for s in STEPS if s.key in self.selected) and not self.motion_ok:
            return CHECK, f"the moving steps are skipped: {self.motion_why} | {text}"
        return PASS, text

    def _plan(self) -> None:
        g, say = self.geo, self.say
        say("-" * 72)
        say("PLAN: the axis will move")
        say(f"  unit        : {self.ctx.unit.get('model')} (serial {self.ctx.unit.get('serial_number')}), at {self.start['position_mm']:.2f} mm")
        say(f"  moves       : {g.describe() if self.motion_ok else 'none (' + self.motion_why + ')'}")
        say("  steps       : " + ", ".join(self.selected))
        say("  safety      : no command writes a parameter or a position row; the unit is put back at its start position with its servo")
        say("                and PC mode as found, also after a failure or Ctrl+C")
        if "push" in self.selected:
            say("  NOTE        : the push step pushes into free air: nothing may be in the way above the middle of the band")
        say("-" * 72)

    def _prepare(self) -> Any:
        did = []
        self.changed = True
        if not self.start["pc_mode"]:
            self.took_control = True
            self.ok(self.cli("mode", "pc"))
            did.append("took PC control (the unit was in PIO mode)")
        if not self.start["servo_on"]:
            self.servo_was_off = True
            self.ok(self.cli("servo", "on"))
            did.append("servo ON (it was off)")
        return PASS, "; ".join(did) if did else "already in PC mode with the servo on"

    # ------------------------------------------------------------------------------------------ steps
    def s_status(self) -> Any:
        st = self.status()
        return (CHECK if st["alarm"] else PASS), f"pos {st['position_mm']:.2f} servo {int(st['servo_on'])} homed {int(st['homed'])} alarm 0x{st['alarm_code']:03X} pc {int(st['modbus_enabled'])}"

    def s_limits(self) -> Any:
        lim = getattr(self, "limits_json", {}) or {}
        speed = (lim.get("speed") or {}).get("max")
        pos = lim.get("position") or {}
        problems = []
        if speed is None or speed > self.args.speed_cap + 1e-9:
            problems.append(f"speed maximum {speed} is above the cap {self.args.speed_cap:g}")
        if self.band and not (near(pos.get("min", 1e9), self.band[0], 0.02) and near(pos.get("max", 1e9), self.band[1], 0.02)):
            problems.append(f"position range {pos.get('min')} .. {pos.get('max')} is not the band {self.band}")
        return (CHECK if problems else PASS), f"speed max {speed}, position {pos.get('min')} .. {pos.get('max')}, kind {lim.get('kind')}" + (" | " + "; ".join(problems) if problems else "")

    def s_info(self) -> Any:
        return PASS, f"identity: {self.ctx.unit}"

    def s_home(self) -> Any:
        if self.status()["homed"]:
            raise Skip("the unit is homed: home not run")
        self.ctx.console.big("HOMING: the actuator moves to its home end at the unit's own home speed", "-")
        r = self.cli("home", timeout=300)
        st = self.ok(r)
        return (PASS if st.get("homed") else FAIL), f"home -> homed {st.get('homed')}, pos {st.get('position_mm')}"

    def s_move_to(self) -> Any:
        g = self.geo
        st = self.go(g.a)
        return (PASS if near(st["position_mm"], g.a) else FAIL), f"move {g.a:.2f} --speed {g.v:g} -> {st['position_mm']:.2f} mm"

    def s_jog(self) -> Any:
        g = self.geo
        self.go(g.c)
        a = self.ok(self.cli("jog", f"{g.jog:+.3f}", "--speed", f"{g.v:g}"))
        b = self.ok(self.cli("jog", f"{-g.jog:+.3f}", "--speed", f"{g.v:g}"))
        ok = near(a["position_mm"], g.c + g.jog) and near(b["position_mm"], g.c)
        return (PASS if ok else FAIL), f"jog {g.jog:+.2f} -> {a['position_mm']:.2f}; jog {-g.jog:+.2f} -> {b['position_mm']:.2f} mm (middle {g.c:.2f})"

    def s_decel(self) -> Any:
        g = self.geo
        self.go(g.a)
        st = self.ok(self.cli("move", f"{g.b:.3f}", "--speed", f"{g.v:g}", "--accel", f"{g.acc:g}", "--decel", f"{g.dec:g}"))
        return (PASS if near(st["position_mm"], g.b) else FAIL), f"move {g.b:.2f} --decel {g.dec:g} -> {st['position_mm']:.2f} mm"

    def s_pattern(self) -> Any:
        g = self.geo
        self.go(g.b)
        r = self.cli("move", f"{g.a:.3f}", "--speed", f"{g.v:g}", "--accel", f"{g.acc:g}", "--accel-pattern", "s_motion")
        if r.rc != 0:
            text = r.tail()
            if "pattern" in text.lower() or "s_motion" in text.lower():
                return PASS, f"s_motion refused before anything was sent: {text}"
            raise CommandFailed(f"rc {r.rc}: {text}")
        pos = self.ok(r)["position_mm"]
        return (PASS if near(pos, g.a) else FAIL), f"move {g.a:.2f} --accel-pattern s_motion -> {pos:.2f} mm"

    def s_speed(self) -> Any:
        g = self.geo
        slow, fast = min(2.0, g.cap / 4.0, g.d / 2.0), min(12.0, g.cap)
        self.go(g.b)
        st = self.ok(self.cli("move", f"{g.a:.3f}", "--speed", f"{slow:g}", "--accel", f"{g.acc:g}", "--change-speed", f"{fast:g}",
                              "--change-after", f"{max(0.3, 0.25 * g.d / slow):.2f}", timeout=g.timeout(g.d, slow) + 30))
        ok = near(st["position_mm"], g.a)
        return (PASS if ok else FAIL), (f"move {g.a:.2f} --speed {slow:g} --change-speed {fast:g} -> {st['position_mm']:.2f} mm: ARRIVAL ONLY, the speed change is not "
                                        "measured here (an ignored --change-speed would pass too; the Python edition measures the speed)")

    def s_fast(self) -> Any:
        g = self.geo
        self.go(g.b)
        st = self.ok(self.cli("move", f"{g.a:.3f}", "--speed", f"{g.v:g}", "--accel", f"{g.acc:g}", "--fast"))
        return (PASS if near(st["position_mm"], g.a, 0.15) else FAIL), (f"move {g.a:.2f} --fast -> {st['position_mm']:.2f} mm: ARRIVAL ONLY, the fast-completion path "
                                                                          "and the monitor's events are not measured here (the Python edition does)")

    def s_stop(self) -> Any:
        g = self.geo
        slow = max(0.5, min(g.v_slow, g.d / 3.0))  # slow enough that the second process is in time
        self.go(g.a)
        r1 = self.cli("move", f"{g.b:.3f}", "--speed", f"{slow:g}", "--accel", f"{g.acc:g}", "--no-wait")
        r2 = self.cli("stop", timeout=30)
        time.sleep(0.3)
        s1 = self.status()
        time.sleep(0.3)
        s2 = self.status()
        margin = 0.05
        ok = (r1.rc == 0 and r2.rc == 0 and g.a + margin < s2["position_mm"] < g.b - margin and not s2["moving"]
              and near(s1["position_mm"], s2["position_mm"], 0.05))
        return (PASS if ok else FAIL), (f"move {g.b:.2f} --speed {slow:g} --no-wait (rc {r1.rc}); stop from a second process (rc {r2.rc}, "
                                        f"{r2.secs:.2f} s): at {s2['position_mm']:.2f} mm, moving {int(s2['moving'])}")

    def s_push(self) -> Any:
        g = self.geo
        band = min(2.0, 0.5 * (g.hi - g.c))
        if band < 0.2:
            raise Skip(f"no room to push in the band (only {band:.2f} mm above the middle)")
        self.go(g.a)
        r = self.cli("push", f"{g.c:.3f}", f"{band:.3f}", "50", "--speed", f"{g.v:g}", timeout=g.timeout(g.d + band, g.v) + 60)
        if r.rc != 0:
            raise CommandFailed(f"push: rc {r.rc}: {r.tail()}")
        text = r.out.lower()
        after = self.status() if self.persist else {}
        back = self.go(g.c)
        if "contact" in text and "miss" not in text:
            raise Check(f"push {g.c:.2f} {band:.2f} 50: the push met resistance: something is in the way")
        ok = "miss" in text and near(back["position_mm"], g.c)
        end = f", end {after['position_mm']:.2f}" if after else ""
        return (PASS if ok else CHECK), f"push {g.c:.2f} {band:.2f} 50 %: {'miss' if 'miss' in text else 'no outcome in the output'}{end}; back at {back['position_mm']:.2f} mm"

    def s_params(self) -> Any:
        r = self.cli("param", "list")
        data = r.json()
        if r.rc != 0 or not isinstance(data, list) or not data:
            raise CommandFailed(f"param list: rc {r.rc}: {r.tail()}")
        return PASS, f"{len(data)} parameters decoded"

    def s_monitor(self) -> Any:
        r = self.cli("monitor", "--watch", "--duration", "1", timeout=60)
        lines = [ln for ln in r.out.splitlines() if ln.strip()]
        return (PASS if r.rc == 0 and lines else FAIL), f"rc {r.rc}, {len(lines)} lines: {lines[-1][:120] if lines else r.tail()}"

    def s_check(self) -> Any:
        path = self.ctx.out / "check_report.json"
        r = self.cli("check", "--report", str(path), plain=True, timeout=300)
        last = (r.out.strip().splitlines() or [""])[-1][:160]
        return (PASS if r.rc == 0 else FAIL), f"rc {r.rc}: {last or r.tail()}"

    def s_recover(self) -> Any:
        r = self.cli("recover", "--backup", str(self.ctx.out / "backup_start.json"), timeout=120)
        data = self.ok(r)
        self.recover_cleared.update(int(c) for c in data.get("alarms_cleared") or [])
        ok = data.get("same_unit") and data.get("cause") == "ok"
        return (PASS if ok else FAIL), f"recover with the link up: cause {data.get('cause')}, same unit {data.get('same_unit')}, restarted {data.get('restarted')}"

    def s_pc_pio(self) -> Any:
        dipm = self.start.get("dipm")
        if dipm is None:
            raise Skip("the status has no PIO input word, so nobody can tell whether a PIO input is on: the unit is not handed to PIO")
        st0 = self.status()
        if st0.get("raw", {}).get("DIPM"):
            raise Check("the unit is not handed to PIO: a PIO input is on and the unit would act on it at once")
        self.pio_handed = True
        try:
            a = self.ok(self.cli("mode", "pio"))
        finally:
            b = self.ok(self.cli("mode", "pc"))
            self.pio_handed = not b.get("modbus_enabled", False)
        ok = not a["modbus_enabled"] and b["modbus_enabled"]
        text = f"mode pio -> PC control {int(a['modbus_enabled'])}; mode pc -> {int(b['modbus_enabled'])}"
        servo_lost = False
        now = self.status()
        lost, expected = servo_across_toggle(bool(st0["servo_on"]), [
            ("mode pio", a.get("servo_on"), a.get("paused")), ("mode pc", b.get("servo_on"), b.get("paused")),
            ("the status after mode pc", now.get("servo_on"), now.get("paused"))])
        if lost and expected:  # a PCON with nothing on its PIO connector: servo off and a pause in PIO mode
            text += "; the servo went off in PIO mode (with a pause)"
            if b["modbus_enabled"] and not now.get("servo_on"):
                self.ok(self.cli("servo", "on"))
                text += " and is on again"
        elif lost:
            servo_lost = True
            ok = False
            text += (f"; the servo was off after {lost} and not as a unit with nothing on its PIO connector has it in PIO mode "
                     "(servo off with a pause): not switched on again")
        if self.motion_ok and self.persist and not servo_lost:
            g = self.geo
            back = self.go(g.c)
            ok = ok and near(back["position_mm"], g.c)
            text += f"; PC control works: move {g.c:.2f} -> {back['position_mm']:.2f}"
        return (PASS if ok else FAIL), text

    # ------------------------------------------------------------------------------------------ end of the run
    def _end_sequence(self) -> None:
        if self.program is None or not self.start:
            return
        self._restore()
        self.rec.step("end", FIXED[4][1], self._end_check, always=True)

    def _restore(self) -> None:
        start, notes = self.start, self.notes
        if not self.persist:
            return

        def attempt(label: str, fn: Any) -> Any:
            try:
                return fn()
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001
                notes.append(f"{label} failed: {type(exc).__name__}: {exc}")
                self.say(f"  restore: {label} failed: {type(exc).__name__}: {exc}")
                return None

        st = attempt("read status", self.status)
        if st is None:
            return
        if st["moving"]:
            attempt("stop", lambda: self.cli("stop", timeout=30))
            st = attempt("read status", self.status) or st
        if (self.changed or self.pio_handed) and not st["modbus_enabled"]:
            self.say("restore: taking PC control back")
            attempt("mode pc", lambda: self.ok(self.cli("mode", "pc")))
            st = attempt("read status", self.status) or st
        if st["alarm"]:
            notes.append(f"alarm 0x{st['alarm_code']:03X} at the end: no servo or position change was attempted")
            return
        move_back = (self.motion_ok and self.geo is not None and start["homed"]
                     and not near(st["position_mm"], start["position_mm"], 0.05))
        if (start["servo_on"] or move_back) and not st["servo_on"]:  # found with the servo off: on for the return move only
            attempt("servo on", lambda: self.ok(self.cli("servo", "on")))
            st = attempt("read status", self.status) or st
        if move_back:
            self.say(f"restore: back to the start position {start['position_mm']:.2f} mm")
            attempt("return move", lambda: self.go(start["position_mm"]))
            st = attempt("read status", self.status) or st
        if not start["servo_on"] and st["servo_on"]:
            attempt("servo off", lambda: self.ok(self.cli("servo", "off")))

    def _end_check(self) -> Any:
        start = self.start
        if not self.persist:
            note = "the simulator of this command line keeps no state between runs: the end state cannot be read back"
            self.notes.append(note)
            return PASS, note
        st = self.status()
        # recover() resets only message-level alarms, so a code in its alarms_cleared is a warning
        alarm_ok, alarm_note = alarm_at_end(start["alarm_code"], st["alarm_code"], self.recover_cleared, True)
        checks = {"PC mode" if self.changed else "mode as found": st["modbus_enabled"] == (True if self.changed else start["pc_mode"]),
                  "alarm as found": alarm_ok,
                  "servo as found": st["servo_on"] == start["servo_on"]}
        if start["homed"]:
            checks["homed"] = st["homed"]
            if self.motion_ok:
                checks[f"at the start position {start['position_mm']:.2f} mm"] = near(st["position_mm"], start["position_mm"], TOL_MM)
            else:
                checks["not moved"] = near(st["position_mm"], start["position_mm"], 0.2)
        bad = [k for k, v in checks.items() if not v]
        compare_text = ""
        path = self.ctx.out / "backup_end.json"
        r = self.cli("backup", str(path), "--rows", "all", plain=True, timeout=300)
        if r.rc == 0 and path.is_file() and self.backup_start is not None:
            self.compare = compare_backups(self.backup_start, json.loads(path.read_text(encoding="utf-8")))
            c = self.compare
            if c["parameters_changed"]:
                bad.append(f"parameters changed: {c['parameters_changed']}")
            if c["rows_changed"]:
                bad.append(f"position rows changed: {c['rows_changed']}")
            compare_text = f" | against backup_end.json: {c['parameter_items']} parameter items compared ({c['live_items_left_out']} items of the live classes 3 and 4 left out), {c['rows_checked']} rows: " + ("unchanged" if not (c["parameters_changed"] or c["rows_changed"]) else "CHANGED")
        else:
            bad.append(f"the end backup could not be read (rc {r.rc})")
        self.end_info.update({"status": st, "checks": checks})
        text = f"pos {st['position_mm']:.2f} servo {int(st['servo_on'])} homed {int(st['homed'])} pc {int(st['modbus_enabled'])}" + (f" | NOT AS FOUND: {bad}" if bad else " | as found") + compare_text
        if self.took_control:
            text += " | the unit was found in PIO mode and is left in PC mode"
        if alarm_note:
            self.notes.append(alarm_note)
            text += " | " + alarm_note
        return (FAIL if bad else CHECK if alarm_note else PASS), text, checks

