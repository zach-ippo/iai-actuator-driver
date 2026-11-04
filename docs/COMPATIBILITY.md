# Compatibility

Which IAI products the driver works with, what has run on hardware so far, and what can go wrong. Every IAI unit
that speaks Modbus RTU on its serial port is supported and driven the same way: the driver reads ranges and
capabilities from the unit and keeps no list of models, so a new model needs no driver update.

Evidence words, for features and paths (never for a unit or a model): **VERIFIED** (tried on real hardware with this
driver), **ON-WIRE** (seen in observed serial traffic), **DOC** (written in a public IAI manual), **UNVERIFIED**
(expected, not yet tried).

Unofficial: IAI does not support this driver. See the [README](../README.md).

## At a glance

**Supported**, every one driven the same way:

- ELECYLINDER (EC) 24 V and EC 200 V, every type: slider, dust/drip-proof slider, rod, gripper, rotary, wire,
  2-position and 3-position models.
- CON-family controllers with their actuator: PCON-CB (for example with RCP6), ACON-CB, DCON-CB, SCON-CB.
- RCON driver units through the RCON gateway.
- ROBONET in SIO through mode (SW2 ON).

**Not supported** (details in [Not supported](#not-supported)):

| Product line | Evidence |
|---|---|
| XSEL, SSEL, ASEL, PSEL, MSEL, TTA (SEL protocol) | DOC |
| ELECYLINDER on an RCON-EC / REC connection unit | DOC |
| ROBONET in gateway mode (SW2 OFF) | DOC |

Connection in every run so far: one unit per COM port, through an IAI USB adapter on the unit's RS-485
(SIO / teaching) port, slave address 1.

## Run on hardware so far

A record of the units this driver has run on and what ran, not a list of allowed units: a unit that is not here is
driven the same way. `iai-check` passed on the first three.

| Unit | Kind | Notes |
|---|---|---|
| EC-DS4H-150-MOT-WA-WL2 | slider (small), dust/drip-proof, 3-position mode, absolute | stroke 0 to 150 mm, up to 700 mm/s, 1.00 G; identity, status, moves, stored rows, home, STOP, parameters (details in [REFERENCE.md](REFERENCE.md)) |
| EC-GRBP8M-20-1-1-PN-WA-WL2 | gripper, 2-position mode, absolute | stroke 0 to 9.50 mm, up to 53 mm/s, 0.30 G |
| EC-R6S-150-1-PN | rod, 2-position mode, incremental (loses home at every reset) | stroke 0 to 150 mm, up to 800 mm/s, 1.00 G |
| EC-S6HH-200-1-PN-WL | slider, incremental | home, direct moves at 50 to 100 mm/s, a mid-move STOP, a stored push row |
| PCON-CB with RCP6-WSA10R | RoboCylinder controller, absolute | same register directory, position table (768 rows) and commands as an EC: home, moves, STOP, push, stored rows, position-table write, parameter commit, software reset, baud change (including the 38400 code), command-profile move, fast completion, `MoveHandle`, `change_speed`, the status monitor, `track()` / `check_link()`, `recover()` after a line break, message alarm 0x069 (cleared by an alarm reset; the warning bit stays set) |

All of these answered at 38400 baud after power-up. The driver finds the rate by itself.

### Which editions were run on a unit

One C++ core does the work in every edition. A behaviour VERIFIED through one edition holds for the core paths that
edition called.

| Edition | Unit | VERIFIED on hardware |
|---|---|---|
| Python package on the core (the default) | EC-GRBP8M gripper | identity, unit info, status, limits, servo, moves, jog, refusals, fast mode and monitor events, move handles (polled and cancelled), stop, pause / resume, speed change, deceleration, acceleration pattern, a missed push, position table and parameters (read), the non-volatile guard, backup, baud detection, track and link check, recovery after a 250 ms line break, the PC/PIO toggle, the self-test. The released 1.0.0 wheel repeated all of it on 2026-10-31 ([Hardware results](#hardware-results)). |
| Python `iai_actuator_driver.native`; the .NET core binding | PCON-CB with RCP6-WSA10R | open with baud detection, identify, status, limits, servo, moves, STOP mid-move, pause / resume, a same-value parameter write skipped, baud 230400 and back to 38400 |
| .NET `Axis` on the core (2026-10-30) | EC-GRBP8M gripper | open and identify, unit info, status, limits, servo off and on, absolute and relative moves and jogs, the speed / stroke / acceleration refusals, a move polled by its status and one cancelled with a `CancellationToken`, stop, pause / resume, its own deceleration, a missed push, position table and parameters (read), the non-volatile guard, baud detection, the PC/PIO toggle; the unit ended as found |
| C++ command line (`iai-actuator-driver-cpp`) | EC-GRBP8M gripper | ports, scan, baud detection, status, identify, limits, alarm history, servo, home, moves and jogs (exact to 0.01 mm), STOP mid-move, pause / resume / cancel-pause, a same-value parameter write skipped, baud 230400 and back to 38400, software reset. Re-check of 2026-10-30, command by command: unit info, `--fast` with `monitor --watch` events, a speed change during a move, its own deceleration, an `s_motion` acceleration pattern, a missed push, position table and parameters (read), backup, track and link check, the PC/PIO toggle, the self-test; the unit ended as found. The harness runs of 2026-10-31, the last with the released 1.0.0 SDK, checked only that the `--fast` and speed-change moves arrive: for 1.0.0 those two are UNVERIFIED ([Hardware results](#hardware-results)). |
| C++ command line | PCON-CB with RCP6-WSA10R | status, identify, limits, alarm reset, servo, moves and jogs, STOP mid-move, pause / resume, a same-value parameter write skipped, software reset |
| .NET command line (`iai-actuator-driver-cs`) | EC-GRBP8M gripper | ports, status, identify, limits, alarm history, servo on, home, moves and jogs, STOP mid-move, parameter list and get, a same-value parameter write skipped, baud 230400, software reset. Released 1.0.0, 2026-10-31: [Hardware results](#hardware-results). |
| .NET command line | PCON-CB with RCP6-WSA10R | status, identify, servo on, moves and jogs, STOP, parameter get and a same-value write |
| GUI (`iai-gui.exe` 1.0.0) | EC-GRBP8M gripper | 2026-10-31, through its single-axis HTTP interface: connect with a stroke override, status, a move out and back, the PC/PIO switch (PC, PIO, PC; a move sent in PIO was refused and nothing was sent), disconnect, quit. Everything else in the GUI: UNVERIFIED ([Hardware results](#hardware-results)) |
| GUI (`iai-gui.exe` 1.0.1) | PCON-CB with RCP6-WSA10R | 2026-11-03, through its single-axis HTTP interface: connect, the stroke band enforced, servo on, a 5 mm move out and back, the PC/PIO switch (PC, PIO, PC: the take-back from a pause with the servo off; a move sent in PIO was refused), disconnect, quit ([Hardware results](#runs-of-2026-11-03-on-the-pcon-cb)) |
| C program (the SDK's `examples/c/first_move.c`) | PCON-CB with RCP6-WSA10R | 2026-11-03: open, identify, status, servo on, a move to the middle and moves by +10 and -10 mm at 10 mm/s, servo off; on Windows with the 1.0.1 SDK and on Linux with a build of the 1.1.0 fix ([Hardware results](#runs-of-2026-11-03-on-the-pcon-cb)) |
| C ABI | through the Python, .NET and C program rows above | the steps of those rows; the calls none of these runs used are tested on the simulator only |
| LabVIEW, `Cell` on several units, RCON / ROBONET gateways, macOS serial ports | none | nothing: UNVERIFIED |

Linux serial ports are VERIFIED from 1.1.0: the PCON-CB through the IAI USB adapter (`cp210x`, `/dev/ttyUSB0`) in
WSL 2 with usbipd, with a build of the fix, from the Python package on the core, the C++ command line and the C program
([Hardware results](#runs-of-2026-11-03-on-the-pcon-cb)). The 1.0.1 Linux files fail at the first read after a
request. The .NET editions and a Linux PC without WSL were not run.

In the .NET edition, fast mode, the monitor's events, a change of speed, move handles, `CheckLink` and recovery after a
line break are UNVERIFIED (tested on the simulator). On 1.0.0 (2026-10-31) the .NET command line, which runs on the
`Axis`, ran the monitor, the backup, `recover` with the link up and the self-test; it ran fast mode and a speed change
with only the move's arrival checked.

## Hardware results

Acceptance runs with the harness in [tools/hw](../tools/hw/README.md): the same checks on any unit, a read-only backup
first, short moves mid-stroke at a capped speed, then the unit is put back as found.

- **PASS**: the step behaved as the harness expects (VERIFIED on this unit, for that edition).
- **Arrival only**: the command ran and the move arrived, but the behaviour named in the row was not measured. It stays
  UNVERIFIED for that edition.
- **Not run**: not tried on hardware. It stays UNVERIFIED for that edition.

No step ended in CHECK or FAIL in the gripper runs below. The PCON-CB runs of 2026-11-03 and the runs on the 1.0.x
branch, the PCON-CB's among them, have their own sections: [Runs of 2026-11-03](#runs-of-2026-11-03-on-the-pcon-cb)
and [Runs on the 1.0.x branch](#runs-on-the-10x-branch).

| | |
|---|---|
| Unit | EC-GRBP8M-20-1-1-PN-WA-WL2 gripper, 2-position mode, absolute; firmware 0002; stroke 0 to 9.50 mm |
| Connection | IAI USB adapter on the unit's RS-485 (SIO / teaching) port, slave address 1, 38400 baud |

### Acceptance run of 2026-10-31: the released 1.0.0 files

The files of the GitHub release 1.0.0, checked against `SHA256SUMS.txt`: the Python wheel in a new environment (run
on the core), the C++ command line of the SDK zip, the C# command line and `iai-gui.exe`. The C# command line was
built from the 1.0.0 source and runs on the released native library; the managed library of the NuGet package was not
itself run.

| | |
|---|---|
| Driver | 1.0.0 on core C ABI 1.0 |
| Limits of the run | speed cap 20 mm/s (the moves ran at 10 mm/s, the mid-move steps at 3 mm/s or below), 0.1 G, targets inside 2.0 to 7.5 mm, no write to a parameter or the position table (the Python run made one same-value parameter write, which the driver skipped) |
| End state | back at 3.00 mm, servo on, PC mode, no alarm, 38400 baud. The 102 parameter items outside the live monitor classes, the 768 position rows and the alarm history matched the read-only backup taken before the run, also after the GUI and coordinated runs below. The Python run also checked the link, and the driver's write counter stayed at 0. |

| Function group | Python, core (default) | C++ command line | C# command line |
|---|---|---|---|
| Identify the unit, unit info, read-only backup | PASS | PASS | PASS |
| Status, limits, position table and parameters (read) | PASS | PASS | PASS |
| Servo off and on | PASS | not run | not run |
| Home from a unit that is not homed | not run | not run | not run |
| Absolute and relative moves, jog | PASS | PASS | PASS |
| Refusal of a speed, target or acceleration beyond the limits | PASS | not run | not run |
| Fast completion and the status monitor's events | PASS | arrival only | arrival only |
| Move handle polled; move cancelled | PASS | not run | not run |
| Stop in the middle of a move | PASS | PASS | PASS |
| Pause and resume | PASS | not run | not run |
| Speed raised during a move | PASS | arrival only | arrival only |
| Own deceleration | PASS | PASS | PASS |
| Acceleration pattern `s_motion` | PASS | PASS | not run (no such option) |
| Push into free air (a miss) | PASS | PASS | PASS |
| Non-volatile guard (same-value write skipped) | PASS | not run | not run |
| Baud rate detection | PASS | not run | not run |
| Link check (`track`, `check_link`); in the command lines `recover` with the link up | PASS | PASS | PASS |
| Recovery after a 250 ms line break | PASS | not run | not run |
| PC / PIO mode toggle | PASS | PASS | PASS |
| Self-test | PASS | PASS | PASS |
| End state as found | PASS | PASS | PASS |
| **Totals** | 29 PASS, 1 not run | 19 PASS, 2 arrival only, 1 not run | 18 PASS, 2 arrival only, 2 not run |

Notes:

- Home was not run because the unit was already homed.
- The command lines run one command per step, so they have no servo, refusal, handle, pause, guard, baud or
  line-break step. Their `monitor --watch` step ran the monitor for a second with no read error; it is counted in the
  totals, not in a row.
- Arrival only: the command lines' speed-change step (`move --change-speed 12`) and fast step (`move --fast`) check
  only that the move arrives (within 0.1 mm and 0.15 mm). They would also pass if the speed change were ignored or the
  fast path not used, and neither reads the monitor's events. The Python run measured the speed change: about
  1.4 mm/s before and about 10.5 mm/s on average after, the stop included, with a 12 mm/s request.
- The command lines' stop step starts a slow `move --no-wait` and sends `stop` from a second process. The gripper
  stopped short of its target (3.77 mm with C++, 3.73 mm with C#).
- The mid-move steps (cancel, stop, pause, speed change) ran at 3 mm/s or below, under the 5 mm/s minimum of the GRB
  manual (DOC). The driver warned and ran the moves.
- In the command lines, `recover` with the link up reported the same unit and changed nothing.
- The Python steps for the PC / PIO toggle call `set_action_mode()`, then `disable_modbus_control()` and
  `enable_modbus_control()`. The command lines' `mode pio` and `mode pc` call `set_action_mode()`. The C# command line
  ran them with `--json`, its stdout read strictly as one JSON document: PASS.
- Recovery after a 250 ms line break (the opt-in `recover` step, the Python run only): the unit went silent, the link
  check said so, and `recover()` took 3.87 s, found the unit at 9600 baud and took the
  rate back to 38400. PC control was kept and no home was needed. Taking PC control back was not exercised, because
  the unit stayed in PC mode.
- The power cycle and the PIO inputs were checked by hand with the same 1.0.0 wheel on 2026-10-31:
  [Checked by hand](#checked-by-hand).

#### The GUI and coordinated motion on 1.0.0

- **`iai-gui.exe`**, driven through its HTTP interface (the single-axis API), not in a browser. Connect at 38400 baud
  with a stroke override of 2.0 to 7.5 mm; status; a move to 5.00 mm and back to 3.00 mm at 10 mm/s and 0.1 G (arrived
  at 4.99 mm and 3.01 mm); PC to PIO to PC (PMSS read 0, then 1); a move sent while PIO was chosen was refused and
  nothing was sent; disconnect; quit, exit code 0. 13 PASS, 0 FAIL: VERIFIED. A request without the access key, and
  one with a foreign Host header, got 403. The GUI's own limits on this unit are 53 mm/s and 0.3 G (the unit's): the
  10 mm/s and 0.1 G came from the test script. The workbench page and its Coordinated tab were not run on hardware.
- **Coordinated motion**: in 1.0.0 the core had no `Cell.coordinated`; the core's runner was run later that day on a
  development build: see [On the core](DRIVER.md#on-the-core). Two real axes together stay UNVERIFIED
  ([DRIVER.md](DRIVER.md#hardware-results)).

### Runs of 2026-11-03 on the PCON-CB

The PCON-CB with RCP6-WSA10R, address 1, 38400 baud. Every run ended as found (94.98 or 94.99 mm, servo off, homed, PC
mode, alarm 0) with no non-volatile write. After its power-up the unit showed the message alarm 0x069; one alarm reset
cleared it before the first run.

- **The released 1.0.1 files (07:53 to 08:32)**, checked against `SHA256SUMS.txt`, on Windows: the wheel in a new
  environment (on the core), the C++ command line of the SDK zip, the C# command line built from the 1.0.1 source on
  the NuGet package's managed and native libraries, and `iai-gui.exe`. The harness steps `default,recover`,
  `--speed-cap 20`, band 30 to 120 mm, 0.1 G, test moves at 10 mm/s.
  - Python on the core: 28 PASS, 1 CHECK (the self-test warns about the alarm history and the position table), 1 SKIP
    (home: the unit was homed); `recover()` after a 250 ms line break restored 38400 baud from 9600 in 4.0 to 4.1 s. C++
    command line: 21 PASS, 1 SKIP (home). C# command line: 20 PASS, 2 SKIP (home; it has no `--accel-pattern` option).
    The PC / PIO toggle PASSed in all three, so the 1.0.1 action-mode fix holds on the released files.
  - GUI, through its single-axis HTTP interface: 14 PASS, the first run of the 1.0.1 GUI on the unit. A request without
    the token was refused (403), connect took 1.54 s, the GUI enforced the 30 to 120 mm band, a 5 mm move out and back
    at 10 mm/s, PC to PIO (the PCON paused with the servo off), a move in PIO refused (409), PIO to PC (the 1.0.1
    take-back from the GUI's switch), disconnect, quit.
  - The C example `examples/c/first_move.c` of the SDK zip: exit 0. Servo on, to the middle at 75.00 mm,
    by +10 mm to 85.00, by -10 mm to 75.00, servo off, every move at 10 mm/s; the axis was then put back at 94.98 mm.
  - The read-only backups of every harness edition, at its start and end, were the same as the start backup (133
    items, 768 rows, the alarm history).
- **High-speed check of the 1.0.1 files (09:59 to 10:01)**: the wheel on the core, and the C++ and C# command lines,
  targets only inside 15 to 135 mm, at most 840 mm/s and 1.0 G. The status was read about every 33 ms, so peaks and
  overshoot are sampled values. Start and end backups the same.
  - 15 to 135 mm and back at 200, 400, 600 and 840 mm/s with 1.0 G, and at 840 mm/s with 0.3 and 0.6 G; hops of 2, 5
    and 20 mm at 840 mm/s and 1.0 G: every move at its target (error at most 0.01 mm, no sampled overshoot), no alarm.
    Each move took about 0.3 s beyond the motion itself until the driver returned (in position and settled): 0.553 s
    for the 120 mm at 840 mm/s and 1.0 G (0.229 s in theory), 0.318 s for a 2 mm hop.
  - STOP (Python `stop()`) at 100, 200 and 300 mm/s with 0.3 G and at 500 and 840 mm/s with 1.0 G: the unit braked at
    the move's own deceleration about 50 ms after the call, and the distances repeated to 0.01 mm (6.94 mm after the
    call from 100 mm/s, 31.0 mm from 300 mm/s, 38.86 mm from 490 mm/s). A STOP never carried the axis past the move's
    target: from 786 mm/s it came to rest at the target, 135.00 mm.
  - The C++ and C# command lines: `move 135` and `move 15` at 840 mm/s and 1.0 G, every command exit 0, at 135.0 and
    15.0 mm.
- **Linux serial port (11:37 to 11:52)**: the IAI USB adapter passed into WSL 2 (Ubuntu 24.04) with usbipd-win 5.3.0
  (`bind --force`: USBPcap is installed), the kernel's `cp210x` driver, `/dev/ttyUSB0`. The harness steps
  `default,recover`, `--speed-cap 20`, band 30 to 120 mm, 0.1 G.
  - The 1.0.1 manylinux wheel failed its first status read with `TransportError: /dev/ttyUSB0: the port vanished
    (unplugged?)`; nothing was written and nothing moved. That is the serial read bug 1.1.0 fixes
    ([CHANGELOG](../CHANGELOG.md)).
  - A build of the fix (the 1.0.1 source with it, gcc Release): the Python package on that core 28 PASS, 1 CHECK (the
    self-test's warnings above), 1 SKIP (home); the C++ command line 21 PASS, 1 SKIP (home); the SDK's C example
    `first_move --port /dev/ttyUSB0 --speed 10` exit 0 (75.00, 85.00, 75.00 mm). The same counts as the Windows run of
    the 1.0.1 files; `recover()` (a 250 ms line break, 9600 back to 38400) passed on Linux too.
  - The 1.1.0 files themselves were not run on the unit.

### Runs on the 1.0.x branch

The Python package on the core, built from the 1.0.x maintenance branch before 1.0.1 (it reports 1.0.0), with the
harness steps `default,recover`, the caps and the end checks of the runs above.

- **EC-GRBP8M gripper, 2026-10-31:** 29 PASS, home not run (homed), end as found; the same again with 24 busy
  processes on the 12-CPU PC (the steps took 5 to 40 % longer). Coordinated motion on the core with the gripper as x
  and a simulated y: 9 PASS idle, a stop in the middle of a circle included. Under the same CPU load the planner refused
  the circle before sending anything: coordinated motion under CPU load stays UNVERIFIED.
- **PCON-CB with RCP6-WSA10R, 2026-11-02**, band 30 to 120 mm, two runs of the Python edition (home not run in both:
  the unit was homed). Coordinated motion on the core with the PCON-CB as x and a simulated y: 9 PASS.
  - Run 1 (started 09:37): 25 PASS, 3 SKIP, 2 FAIL. The PC / PIO toggle FAILed (below) and left the unit in PIO mode,
    so the end step FAILed too (NOT AS FOUND: PC mode, at the start position). The self-test and `recover()` were
    SKIPped: the run stops after a failed step.
  - Run 2 (started 09:45, PC / PIO toggle skipped): 26 PASS, 1 SKIP, 1 CHECK, 1 FAIL. The self-test was CHECK (it
    warns about the message alarm 0x069 and the position table). The end step FAILed on "alarm as found (0x069)":
    `recover()` had reset that alarm (last bullet below).
- PC / PIO toggle on the PCON-CB: in PIO mode, with nothing on its PIO connector, the unit switches its servo off and
  reports a pause (DSS1 STP) at a standstill; the 1.0.0 driver refused to take PC control back. PMSL ON cleared the
  pause and the servo stayed off. All VERIFIED on the PCON-CB, 2026-11-02. 1.0.1 takes PC control back while the axis
  stands still with the servo off and nothing moves, pushes or homes; a pause with the servo on is still refused (a PLC
  holding a stop input on a move), and a PLC pause with the servo off is still indistinguishable from an unwired
  stop input. 1.0.1 taking control there: VERIFIED (PCON-CB, 2026-11-02).
- Action mode and servo ON on the PCON-CB, 2026-11-02 (15:29, the 1.0.x branch at the 1.0.1 version commit, Python
  package on the core, no motion, no non-volatile write): every 0x45 cmd 0x0C with op1 = 1 (PC), from PIO and in PC
  mode alike, cleared DSSE bit 12 (5 of 5), and a servo ON was then not taken (no alarm, no cause shown). Of what was
  tried, only a PMSL coil OFF set the bit again (4 of 4; PMSL ON written again did not, 4 of 4); op1 = 0 while the bit
  was 0 and a power cycle were not tried. PMSL ON from PIO kept the bit and the servo came on in 0.12 s (2 of 2). All
  VERIFIED.
  The release-package run before had found it: the C++ command line's `mode pio`, `mode pc` left the servo refusing to
  come on, which the Python toggle (PMSL OFF and ON after the action mode) had hidden. 1.0.1 therefore takes PC control
  with the PMSL coil in `set_action_mode(True)`.
- That `set_action_mode` on the PCON-CB, 2026-11-02 (16:10, a build of ea04991, no non-volatile write):
  `set_action_mode(False)` then `set_action_mode(True)` sent op1 = 0 and then only the PMSL coil, DSSE bit 12 stayed
  set and the servo came on in 0.12 s, from the Python package on the core and from the C++ and C# command lines
  (`mode pio`, `mode pc`, `servo on`); no op1 = 1 was sent. The harness's pc_pio step passed for the Python and the
  C++ command-line editions. VERIFIED. The safety speed stayed off, so the SFTY coil in this form is UNVERIFIED there.
- `recover()` after the line break reset the message alarm 0x069 the PCON-CB started with, and the end check of run 2
  called that not as found (FAIL). The harness now reports it as CHECK, naming the alarm.
- **PCON-CB with RCP6-WSA10R, 2026-11-02 (13:07 to 13:18), the source that became 1.0.1** (it still reported 1.0.0: the
  release commit changes version numbers and text only), the Python edition on the core, no non-volatile write. The
  harness steps `default,recover`: 28 PASS, 1 CHECK (the self-test warns about the alarm history and the position
  table), 1 SKIP (home: the unit was homed), the PC / PIO toggle PASS, end as found. STOP, 10 trials each: at 10 mm/s
  and 0.1 G, idle, the STOP reached the wire a median of 16.5 ms after the call (17.1 ms before the STOP changes) and
  the axis was at rest after 88 ms (88.5 ms); at 840 mm/s and 1.0 G, 7 ms and 254 ms (8.0 and 256 ms). Under CPU load
  (24 busy loops) 2 of 10 STOPs at 10 mm/s took 122 and 134 ms to reach the wire (before: 1 of 10, 162 ms).
  Coordinated motion with the PCON-CB as x and a simulated y, a circle of r = 2 mm at 5 mm/s: idle 9 PASS (deviation at
  most 0.08 mm, no late frame, a stop in mid-circle ended the run in 0.39 s). Under CPU load 5 PASS, then the stopped
  circle FAILed its check: the run ended by itself on a refused late frame before the stop took effect, no frame past
  the limit was sent, and the last 3 steps did not run. Coordinated motion under CPU load stays UNVERIFIED.

### Acceptance run of 2026-10-31, before the release

The same unit with driver 0.12.0b1 on core C ABI 0.8 (the layout released as C ABI 1.0), at most 10 mm/s and 0.1 G,
targets inside 2.0 to 7.5 mm. Every function group had the result of the table above, except that the opt-in recovery
after a line break was not run. Totals: Python core 28 PASS, 1 not run; C++ command line 19
PASS, 2 arrival only, 1 not run; C# command line 18 PASS, 2 arrival only, 2 not run.

### Acceptance run of 2026-10-30

The same unit with driver 0.12.0b1 on core C ABI 0.7, at most 20 mm/s and 0.1 G, targets inside 2.0 to 7.5 mm, and the
same end-state checks, all as found. Every function group had the result of the table above. The Python core also
ran the opt-in recovery after a 250 ms line break: PASS.

- Totals: Python core 29 PASS, 1 not run; C++ command line 19 PASS, 2 arrival only, 1 not run;
  C# command line 18 PASS, 2 arrival only, 2 not run.
- The Python run measured the speed change: about 1.4 mm/s before and about 10 mm/s on average after, with a 12 mm/s
  request.
- After the line break the unit answered at 9600 baud. The driver found it, took the rate back to 38400 and the run
  went on. The unit had not restarted and kept its home.
- The unit stayed in PC mode through the break, so taking PC control back was not exercised. The report said PC
  control was not kept; that reporting fault is fixed and the step now also requires PC control to be reported kept
  or restored. The fixed step has not been run on the unit.
- The first Python-core and C# runs that day failed on a harness fault (a live monitor value counted as a change) and
  on a command-line defect (with `--json`, a note printed on stdout before the JSON of `mode pc`). Both were fixed and
  the totals are from the repeats. The C# repeat ran before the command-line fix, with a harness that skipped the note
  line; the fixed C# command line ran on 2026-10-31.

### Checked by hand

Python package on the core, with the harness's `power-cycle` and `pio-watch` commands run by hand: on 2026-10-30 at an
earlier interface version, and again on 2026-10-31 with the released 1.0.0 wheel.

| Check | Result | Evidence |
|---|---|---|
| 2026-10-30: live power cycle, 24 V switched off by hand for 3.2 s while the driver ran | `check_link()` reported the link lost (unit silent), then a restarted unit (uptime counter reset, PC control back to PIO). `recover()` took PC control back in 1.44 s. The servo stayed on and the home was kept; the 3 rows and 102 parameter items were unchanged; nothing was written to non-volatile memory. | VERIFIED |
| 2026-10-30: PIO inputs, switches wired by hand to the PIO connector, the unit in PIO mode, watched read-only for 150 s | ST0 (input bit 0) sent the gripper to its backward end (row 0, 3 mm) and ST1 (bit 1) to its forward end (row 1, a push to 9.5 mm); 54 moves were seen. The PIO words and the position were read throughout with no read error. The unit was put back in PC mode afterwards. RES (bit 2) was not pressed. | VERIFIED |
| 2026-10-30: STOP in PIO mode, the driver's STOP sent during a PIO move | The unit accepted it and stopped (1.8 mm after the move was seen, 0.35 s after the call). The data does not show whether a start input held through the STOP restarts the move: the input word was not sampled while the STOP was held. | VERIFIED (the STOP accepted, the unit stopped); UNVERIFIED (a restart by a held start input) |
| 1.0.0, 2026-10-31: live power cycle | `check_link()` reported the link lost, then a restarted unit (uptime counter reset, PC control back to PIO). `recover()` took PC control back in 1.44 s; servo on, home kept, back at 3.00 mm; the 3 rows and 102 parameter items unchanged; nothing written to non-volatile memory. | VERIFIED (1.0.0) |
| 1.0.0, 2026-10-31: PIO inputs, watched for 120 s | ST0 and ST1 as above, 23 moves, 2353 status reads with no read error; PC mode taken back at the first attempt. RES (bit 2) is not wired on this bench, so it was never toggled. | VERIFIED (1.0.0: ST0, ST1); UNVERIFIED (RES) |
| 1.0.0, 2026-10-31: STOP in PIO mode on a push row | The STOP, sent as ST1 started the push row 1, was accepted: the unit came to rest at the row's approach position (4.11 mm) with the move-end flag MEND 0 while ST1 was still held; every other push with ST1 held ran on to 9.50 mm. | VERIFIED (1.0.0: the STOP cut the push); UNVERIFIED (a restart by a held start input) |

### Not run on hardware

On this gripper the following stay UNVERIFIED, unless a table above lists them as VERIFIED from an earlier test:

- the RES input of the PIO connector (not wired on the bench);
- coordinated motion with two real axes (on the core, one real axis is VERIFIED on a development build only:
  [DRIVER.md](DRIVER.md#on-the-core));
- a PLC driving the PIO connector; the power cycle through the C++ and C# command lines; the PIO checks through the
  command lines;
- home from a unit that is not homed; writes to parameters and the position table;
- a push that meets a workpiece; a vertical or loaded axis;
- every item the tables above do not list.

A report from your unit moves a path to VERIFIED: see [Sending a report](#sending-a-report).

## Notes per product line

Every supported line uses the same protocol and register layout, so the driver talks to each the same way and reads
its limits from the unit. Results from any unit are welcome: run `iai-check` and open a unit report
([Sending a report](#sending-a-report)).

- **ELECYLINDER types.** A rotary axis is driven in degrees (the unit reports its kind). Wire types and 3-position
  models are driven like any slider or rod. A unit code the driver has a family name for is shown with that name;
  any other code is shown with its product family and codes. Either way the unit is driven the same way.
- **EC 200 V.** Identified from its identity code; same commands.
- **ACON-CB, DCON-CB, SCON-CB.** CON-family controllers with the Modbus RTU serial port IAI documents for the family
  (DOC); the driver drives them as it drives the PCON-CB, which behaves like an EC.
- **RCON driver units through the RCON gateway.** One gateway port carries several axes (address = axis number + 1).
  The driver applies the gateway's documented rules (register path instead of coils, no IAI commands, no broadcast,
  the position unit stated before any mm move) and refuses anything the line cannot carry before sending it. These
  gateway paths are DOC, UNVERIFIED on hardware.
- **ROBONET in SIO through mode (SW2 ON).** State it with `--line robonet` (`line="robonet"` in Python): through mode
  cannot be detected.
- **Several axes on one port** (daisy chain or gateway) is UNVERIFIED; one axis per COM port is VERIFIED.

## Not supported

| Product | Why |
|---|---|
| XSEL, SSEL, ASEL, PSEL, MSEL controllers and the TTA table-top robots | They use IAI's SEL protocol on their serial port, not Modbus RTU. They never answer a Modbus request. |
| ELECYLINDER connected through an RCON-EC or REC connection unit | The unit's own serial port is disabled there. It shows up as linked at the gateway but is silent on Modbus. |
| ROBONET in gateway mode (SW2 OFF) | The gateway gives no axis access over Modbus. The driver raises `UnsupportedLineError`; set SW2 ON (through mode). |

When a port gets no answer at all, the driver, the command line and the GUI say so and name these cases.

## Possible trouble

**Nothing answers.** Check the 24 V supply, the cable, the slave address (axis number + 1) and that no other program
has the port. `iai-actuator-driver ports` lists the serial ports and marks IAI adapters; `iai-actuator-driver scan`
asks every address.

**The baud rate was changed, or a line break was sent.** An EC starts at 38400 baud. After a baud change the unit
stays at the new rate until it is power-cycled. A line break of 150 ms or more drops a controller to 9600 (DOC;
VERIFIED on the EC-GRBP8M gripper and the PCON-CB). The driver finds the rate at connect (115200, 38400, 9600, 230400,
57600, 19200) without closing the port. 14400, 28800 and 76800 are not probed: reconnect with `--baud RATE`, or
power-cycle the unit back to 38400.

**Something else holds the link.** Only one master can use the port. Close IA-OS and any other program that has the
COM port open (the port then reports access denied). A unit that IA-OS has put in its Monitor mode does not accept
moves from another program (VERIFIED). A teaching pendant or a wireless session connected to the unit is expected to
take control the same way (UNVERIFIED). Disconnect the other tool, then connect again.

**Old firmware without the register directory.** Current units describe their own register layout at address 0x0000
(the "directory"), and the driver uses it to find every area and limit. A unit without it is read at the standard
addresses, and the identity notes say `index table: ...`. When its stroke or limits cannot be read, moves are refused
until you state the stroke (`--stroke MIN MAX`), and speed, acceleration and push fall back to conservative defaults
(100 mm/s, 0.3 G, 70 %). Run `iai-check` and open a unit report ([Sending a report](#sending-a-report)).

**CCM conveyor control motor.** Its identity code is recognised and named "CCM conveyor control motor (not an EC
actuator)". It is not a positioning actuator with a stroke, so the mm moves and stroke limits of an axis do not
describe it.

**Vertical axes with a brake.** The driver cannot know how an axis is mounted. The speed and acceleration limits the
unit reports are horizontal, light-load values; a vertical mounting or a payload lowers the real limits, so stay well
below them. Servo OFF, a servo-off alarm, an emergency stop or a software reset releases position control: a vertical
axis without a brake falls, and with a brake it is held only as long as the brake holds. The driver never releases
the brake by hand. Test vertical axes unloaded and slow, with the hardware emergency stop within reach.

**Incremental units lose their home** at every software reset and power-up: run `home` before the next move. Absolute
(battery-less, `WA`) units keep it, except after a change of the home direction or offset.

## Sending a report

Reports go to [GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose). Use **Unit report**
for "my unit works" or "my unit does not work", and **Bug report** for anything else.

`iai-check --port COM3` (read-only) writes `iai-check-<timestamp>.json` with the model, firmware, limits and every
raw value read. `iai-check --port COM3 --motion` adds a short move of at most 5 mm at 10 mm/s and back. Attach that
file to the issue, for any unit. It holds the unit's serial number and the port name, and issues are public.
