# IAI Actuator Driver: reference and evidence notes

How the driver, the command line, the test GUI and the simulator behave, with the evidence behind each statement. For
a first run read [GETTING_STARTED.md](GETTING_STARTED.md); for one line per function (and the same calls in C#, C++
and C) read [DRIVER.md](DRIVER.md); for the supported products read [COMPATIBILITY.md](COMPATIBILITY.md).

Evidence words, for features and paths: **VERIFIED** (tried on hardware), **ON-WIRE** (seen in serial traffic),
**DOC** (public IAI manuals), **UNVERIFIED** (expected, not yet tried). Every unit is driven the same way, whatever its
model. Unofficial: not an IAI product; see the [README](../README.md).

## Run on hardware so far

A record of the units this driver has run on, not a list of allowed units.

Slave address 1, one unit per port. Every unit answered at **38400 baud after power-up** (115200 after another PC tool
had connected); the driver finds the rate by itself.

| Model (0x0600 string) | Kind (AKID 7.33) | FC11 unit code (family) / firmware | Stroke (soft limits 0x0700) | Max speed / accel (MAXV 7.4 / MAXA 7.5) | DOPM layout | `iai-check` |
|---|---|---|---|---|---|---|
| EC-DS4H-150-MOT-WA-WL2 (`MWB-EC-DSRR4-WA-MF`) | slider (small) 9, 3-position mode | 0x02 (EC dust/drip-proof) / 0x0013 | 0..150.00 mm (-0.30 / +150.30) | 700.00 mm/s / 1.00 G | b0 LS0, b1 LS1, b2 LS2, b3 *ALM | PASS |
| EC-GRBP8M-20-1-1-PN-WA-WL2 | gripper 4, 2-position mode | 0x0A (EC(MS) "2-size EC") / 0x0002 | 0..9.50 mm (-0.30 / +9.80) | 53.00 mm/s / 0.30 G | b0 LS0, b1 LS1, b2 *ALM | PASS |
| EC-R6S-150-1-PN (no WA: incremental) | rod 3, 2-position mode | 0x00 (EC standard) / 0x0002 (FC11 answers after 0.2-0.4 s) | 0..150.00 mm (-0.30 / +150.30) | 800.00 mm/s / 1.00 G (VERIFIED) | b0 LS0, b1 LS1, b2 *ALM | PASS |

Also run on hardware:

- **EC-S6HH-200-1-PN-WL** (EC, unit 0x00, firmware 0x300E, incremental, soft limits -0.30 / 200.30): home, direct
  moves at 50-100 mm/s, a mid-move STOP and a stored push row (VERIFIED).
- **PCON-CB + RCP6-WSA10R** (FC11 `AE FF 44 00 00`: system 0xAE PCON, unit 0x44; absolute; 38400 at power-up): the
  same directory, blocks, position table (768 rows) and commands as an EC, including home by coil and by 0x45 cmd
  0x0A, the 0x45 alarm reset / action mode / servo forms, and the software reset, which reloads parameters from
  non-volatile memory and keeps PMSS = 1, the baud rate and the uptime (VERIFIED). What else was run is listed in
  [COMPATIBILITY.md](COMPATIBILITY.md).
- **The core on the EC-GRBP8M gripper**: the Python package (on the core and, up to 1.0.x, on its fallback, with
  nearly the same results), the .NET `Axis` and the C++ and C# command lines each went through the steps of their edition.
  The steps per edition are in [COMPATIBILITY.md](COMPATIBILITY.md#hardware-results). The released 1.0.0 files (the
  Python wheel, the C++ and C# command lines and `iai-gui.exe`) were run the same way on 2026-10-31. No parameter or
  position-table write cycle was used, and the unit ended as found.

The FC11 system code names the product **family** (`EC`, `PCON`, `ACON/DCON`, `SCON`, else `IAI controller`); for an
EC the unit code names the EC family (display only). The actuator **kind** (gripper / rod / slider / rotary ...)
comes from AKID, item 7.33. A family, unit code or kind the driver has no name for is driven exactly the same way and
reported with the codes the unit gives.

**VERIFIED on the ECs:**

- Identity (FC11, index table, 0x0600 model string), the soft limits, the 0x0812 speed, the alarm history, the
  position table (read) and the status reads: position, alarm code, the servo / in-position / homed / pause / push /
  homing / Modbus bits (on the slider and the gripper).
- Direct numeric moves (absolute, relative, re-targeting a move in progress), push moves (with and without contact)
  and stored-position moves.
- Decel stop (also with Modbus control off, on a move commanded over Modbus), pause/resume, home return, servo
  off/on, the alarm-reset pulse, the PMSL coil (Modbus control), the SFTY coil (safety speed caps ~100 mm/s) and the
  ZONS position markers.
- Position-table writes (non-volatile at once).
- The parameter commit (coil 0x0428 PARW) and the software reset (0x45 cmd 9 servo OFF, then 0x45 cmd 5 op1 1), both
  ON-WIRE and VERIFIED on the EC-R6S: a commit sent without waiting on the DRGE / DSSE b7 busy flags survived a power
  cycle, the unit was seen acting on the driver's own `param set` commit, and `swreset` applied it (the new P2 value
  read back after the reset).
- The baud change (broadcast 0x45 cmd 4, ON-WIRE): codes 0x14 (115200) and 0x15 (230400) switched the EC-R6S both
  ways; **0x06 ("38400") was ignored**.
- The 0x45 forms on the EC-R6S: home `01 45 00 0A 00 07 00 00` (op1 = the home pattern ORG, item 9.16; op1 = 0 is
  rejected 0x11 "1st operand error"), alarm reset `01 45 00 05 00 00 00 00` and the action mode (0x45 cmd 0x0C,
  ON-WIRE) `01 45 00 0C 00 <pc> 00 <safety>` (PMSS / SFTY followed). MAXV 7.4 = 800.00 mm/s and MAXA 7.5 = 1.00 G
  were read.
- Up to 1.0.0 the driver's own `set_action_mode(pc)` was the PC/PIO toggle in this form. On the EC-GRBP8M gripper PMSS
  read back 0 after `pc=False` and 1 after `pc=True`, the axis stood still in PIO and moved again in PC mode (VERIFIED
  on 2026-10-30 and with the released 1.0.0 on 2026-10-31; [COMPATIBILITY.md](COMPATIBILITY.md#hardware-results)).
  That test sets `pc` only: the `safety` argument is UNVERIFIED on the gripper.
- On the PCON-CB every 0x45 cmd 0x0C with op1 = 1 cleared DSSE bit 12 (listed as "cannot be used" in ME0162-12A
  p.4-20; its name is unknown), from PIO and in PC mode alike and with either op2. While the bit was 0 the unit took
  no servo ON: SON was written, STAT bit 1 stayed 0, no alarm and no cause showed. Of what was tried, only a PMSL coil
  OFF set the bit again (PMSL ON written again did not; op1 = 0 while the bit was 0 and a power cycle were not tried);
  PMSL ON from PIO kept it, and the servo then came on (VERIFIED on the PCON-CB, 2026-11-02). So from 1.0.1 on
  `set_action_mode(True)` takes PC control with the PMSL coil and changes the safety speed with the SFTY coil; only
  `pc=False` sends this command (op1 = 0), and so does `pc=True` on a unit without FC05 coils. The 1.0.1 form took PC
  control from PIO on the PCON-CB with the PMSL coil alone, and the servo then came on (VERIFIED, 2026-11-02, a build
  of ea04991: the Python package on the core and the C++ and C# command lines); its safety-speed change by the SFTY
  coil is UNVERIFIED on the unit.

**Not yet verified (UNVERIFIED):** the driver's own `home(method="iai")` / `reset_alarm(method="iai")` calls (the
frames are the verified ones), the 0x48 commands, a message-level alarm on an EC (0x069
was seen on the PCON-CB; that the axis keeps running with one active is UNVERIFIED), and a Modbus STOP on a move a PLC
commanded over PIO (only a hand-wired PIO input was tried, VERIFIED; see
[COMPATIBILITY.md](COMPATIBILITY.md#hardware-results)).

**Not supported:** controllers that speak IAI's SEL protocol instead of Modbus RTU (XSEL, SSEL, ASEL, PSEL, MSEL and
the TTA table-top robots) do not answer Modbus, and an ELECYLINDER on an RCON-EC / REC connection unit has its serial
port disabled. The driver says so when a port gets no answer.

Other models: run `iai-check` and open a unit report ([how](COMPATIBILITY.md#sending-a-report)).

## Command line reference

Close IA-OS (IAI's PC software) first: only one program can own the serial port.

```shell
python -m iai_actuator_driver --port COM3 status       # read-only
python -m iai_actuator_driver --port COM3 identify     # family, model / serial, units, soft limits, directory
python -m iai_actuator_driver --port COM3 info         # everything the unit tells about itself, read-only
python -m iai_actuator_driver --port COM3 limits       # every range the driver enforces, from the unit itself
python -m iai_actuator_driver --port COM3 monitor --count 10  # position, velocity, current, deviation, alarm
python -m iai_actuator_driver --port COM3 alarms       # alarm history, read-only
iai-check --port COM3                                  # compatibility self-test, read-only
iai-check --port COM3 --motion                         # ... plus a +/-5 mm move at 10 mm/s and back
python -m iai_actuator_driver --port COM3 servo on
python -m iai_actuator_driver --port COM3 move 80 --speed 20
python -m iai_actuator_driver --port COM3 move 120 --speed 100 --accel 0.3 --decel 0.1  # own deceleration
python -m iai_actuator_driver --port COM3 jog -5       # relative step (never the EC's JOG coils)
python -m iai_actuator_driver --port COM3 push 80 10 30  # approach 80 mm, push 10 mm further at 30 %
python -m iai_actuator_driver --port COM3 stop
python -m iai_actuator_driver --port COM3 move 100 --speed 20 --change-speed 60 --change-after 1.5  # change the running move's speed (VERIFIED on the EC gripper)
python -m iai_actuator_driver --port COM3 pause        # hold a move, keep its travel (stays set); 'resume' continues it
python -m iai_actuator_driver --port COM3 cancel-pause # drop the remaining travel of a paused move (VERIFIED on the EC gripper)
python -m iai_actuator_driver --port COM3 teach 2           # store the current position as row 2 (permanent; an unchanged row is not written, --force writes it)
python -m iai_actuator_driver --port COM3 backup unit1.json  # read-only JSON backup (FC03 / FC11 only)
python -m iai_actuator_driver --port COM3 line --monitor     # the line (direct / rcon / robonet), its rules, the gateway status
python -m iai_actuator_driver --port COM3 move 80 --fast     # fast completion with the status monitor (baud rate kept)
python -m iai_actuator_driver --port COM3 monitor --watch    # status changes and link events until Ctrl+C (read-only)
python -m iai_actuator_driver --port COM3 recover --backup unit1.json --target 80  # re-sync after a power / link loss; never moves (VERIFIED on the EC-GRBP8M gripper and the PCON-CB)
python -m iai_actuator_driver --port COM3 recover --backup unit1.json --rows used  # compare only the rows 'backup --rows used' holds, not all 768 (this command line only)
python -m iai_actuator_driver stop-all --ports COM3,COM4     # STOP every axis of the ports (--broadcast: one broadcast STOP too, UNVERIFIED)
python -m iai_actuator_driver --port COM3 goto 0       # stored position 0 (0x9800) at its stored speed
python -m iai_actuator_driver --port COM3 goto 6 --speed-pct 25  # row 6 at 25 % of its stored speed (0x45 cmd 6)
python -m iai_actuator_driver --port COM3 alarm-reset  # alarm reset ('reset' is a deprecated alias of it)
python -m iai_actuator_driver --port COM3 alarm-reset --method iai  # ... via 0x45 cmd 5 op1 0
python -m iai_actuator_driver --port COM3 home --method iai  # home via 0x45 cmd 0x0A, op1 = ORG (item 9.16)
python -m iai_actuator_driver --port COM3 mode pc --safety   # action mode 0x45 cmd 0x0C (PC/PIO, safety speed)
python -m iai_actuator_driver --port COM3 param list   # this unit's parameters (EC: P-numbers, CON: No.), read-only
python -m iai_actuator_driver --port COM3 param get 8.15     # one parameter or raw item (class.item), read-only
python -m iai_actuator_driver --port COM3 param set 2 0.2    # write + verify + COMMIT (permanent); applies after swreset; a value the unit already holds is not written (--force)
python -m iai_actuator_driver --port COM3 param set No.35 1  # a CON parameter number (a dangerous one is written with a warning)
python -m iai_actuator_driver --port COM3 swreset      # software reset (servo OFF first); incremental units lose home
python -m iai_actuator_driver --port COM3 baud 115200  # broadcast baud change (volatile; a code not VERIFIED is sent with a warning)
python -m iai_actuator_driver --sim status             # same commands against the simulator
python -m iai_actuator_driver --sim --sim-profile gripper identify  # ... simulating the gripper (rod, pcon, rotary, unknown)
python -m iai_actuator_driver ports                    # serial ports, IAI USB adapters first (opens nothing)
python -m iai_actuator_driver scan                     # every IAI adapter, addresses 1-16: who answers (read-only)
python -m iai_actuator_driver scan --ports COM3,COM4 --slaves 1-4
python -m iai_actuator_driver --port COM5 --axis 2 status  # axis 2 on a shared port (= --slave 3)
```

**Baud rate.** Without `--baud` the CLI finds the rate on the open port: 115200 first, then 38400 (the EC's power-up
rate), 9600, 230400, 57600, 19200. The rate is changed in place: the port is never closed and reopened in between
(that, or a long line break, can drop an EC back to its boot rate). The rate found is printed to stderr
(`note: COM3 at 38400 baud (auto-detected)`). `--baud N` fixes the rate (no probing).

| Option | Meaning |
|---|---|
| `--slave N` / `--axis N` | Modbus address (default 1) / axis number (= slave - 1) |
| `--baud N` | a fixed rate (default: auto-detect) |
| `--retries N` | default 3, the manufacturer's recommendation |
| `--line auto\|direct\|rcon\|robonet` | the line the unit sits on (default: probed once, read-only) |
| `--position-unit 0.01\|0.001` | an RCON unit's parameter No.207, needed before mm-based motion there; a direct line uses the unit's own 0.01 and refuses 0.001 (nothing sent) |
| `--strategy coil\|iai\|iai2\|register` | `iai` = IAI function 0x45, `iai2` = 0x48 (experimental), `register` = FC06 DRG1 without coils; default `coil`, `register` behind an RCON gateway |
| `--stroke MIN MAX` | default: the unit's own soft limits; can only narrow them |
| `--max-speed` / `--max-accel` / `--max-push` | CLI defaults: a conservative 100 mm/s (deg/s), 0.3 G and 70 %, each lowered to the unit's own MAXV 7.4 / MAXA 7.5 and MAXD 7.6 / MXPP 7.9 |
| `--unit-max` | lifts the defaults to the unit's own maxima |
| `--allow-fast` | lets `goto` run a row faster than the cap |
| `--json` / `--log-frames` | JSON output / every frame to stderr |
| `--sim-profile slider\|gripper\|rod\|pcon\|rotary\|unknown` | the simulated unit (with `--sim`) |

`ports` and `scan` exit 0 when an axis answered, 1 when none did, and 2 on an error or when no IAI adapter is found.

**What writes.** Commands that change the unit's state (`servo`, `home`, `move`, `jog`, `push`, `goto`,
`alarm-reset`, `mode`, `swreset`, `baud`, `param set`) first read the identity, soft limits and speed limit, and take
Modbus control while the axis is at rest. `status`, `identify`, `info`, `limits`, `monitor` (also `--watch`),
`alarms`, `param list` / `get`, `ports`, `scan` and `iai-check` (without `--motion`) never write to the unit.
`recover` writes only what it restores (PC control, the baud rate with `--restore-baud`, a message-level alarm reset)
and never moves the axis. `stop` works whether or not Modbus has control.

**`reset` is a deprecated alias** of the **alarm** reset `alarm-reset` (with a warning on stderr), so an old script
can never trigger a software reset by surprise. The software reset is `swreset`.

## Examples, configuration and diagnostics

### Python example

```python
from iai_actuator_driver import Elecylinder, SimulatedEC

with Elecylinder.open("COM3") as ec:     # baud auto-detected; or Elecylinder.open(SimulatedEC())
    info = ec.connect()                  # identity, soft limits, speed limit; Modbus control at rest
    print(info.model, info.controller, info.soft_limits_mm, info.max_speed_mm_s, ec.effective_stroke)
    ec.servo(True)
    ec.move_to(80.0, speed_mm_s=20, accel_g=0.1)   # blocks until in position
    ec.jog(-5.0)                                   # relative step at 10 mm/s
    result = ec.push(85.0, band_mm=10.0, force_pct=30, direction=+1)
    print(result.outcome)                          # "contact" or "missed"
    ec.move_to(75.0, speed_mm_s=20)                # retract (ends a push hold)
    ec.servo(False)
```

Pass `stroke=(min_mm, max_mm)` to restrict motion further; it is always intersected with the unit's own limits. For
several axes use a `Cell` (see [Several axes](#several-axes-ports-scan-cell)).

**Position markers.** `ec.status()` gives `at_backward_end` / `at_forward_end` / `at_middle` from ZONS 0x9013 (the
same bits on every EC tested; `None` if ZONS was not read). They follow the position, also while moving: a marker is
lit while the axis is within the LS range (parameter 2, 0.10 mm on both units) of stored position 0 / 1 / 2, or of a
push row's push end (the gripper's grip row, 4.10 mm + 5.40 mm push, lights "forward end" only at 9.50 mm). Row 2
lights "middle" on the 2-position gripper too. The PIO image DOPM is only kept in `status().raw`, because its layout
differs between models (bit 2 is "middle" on the 3-position slider but "no alarm" on the 2-position gripper).

### User parameters, software reset, baud rate, stored positions

These commands change the unit (parameters permanently), so read the notes below first.

```python
with Elecylinder.open("COM3") as ec:
    for p in ec.user_parameters():               # this unit's user parameters, decoded
        print(p["no"], p["name"], p["text"], p["writable"], p["model_note"])
    r = ec.write_user_parameter(2, 0.20)         # write, verify by read-back, commit (PARW)
    print(r.before, "->", r.after, r.committed, r.takes_effect)
    reset = ec.software_reset()                  # the unit applies committed parameters now
    if reset.needs_home:                         # incremental unit, or ABS error after a home-parameter change
        if reset.alarm_code:
            ec.reset_alarm()
        ec.home()
    ec.move_to_position(0)                       # stored row 0 at its stored speed (0x9800 = 0)
    ec.set_baudrate(115200)                      # broadcast baud change; volatile
```

- **`write_user_parameter(no, value, *, commit=True, allow_home_params=False, allow_unlisted=False)`** (EC P-numbers;
  `allow_home_params`, `allow_unlisted` and `allow_unverified_model` are accepted and ignored) accepts only the
  parameters confirmed on hardware (VERIFIED on the gripper): P1 (operation range = upper soft limit - 0.30 mm), P2 (LS
  range), P3 (home direction), P4 (home offset), P5, P6, P7 (1 = Disable, inverted), P9, P10, P12.
  - The item is found by **class + item ID** through the unit's directory (addresses differ per model). Shared items
    (8.1, 8.14, 8.50) are read-modify-written, so only that bit changes.
  - **P3 is the parity of the home pattern ORG (item 9.16)**: odd = 1:Forward, even = 0:Reverse (the gripper's 7 / 8
    and the EC-R6S's 7 fit; the general rule is UNVERIFIED). A change writes ORG - 1 / + 1; P3 is not settable when
    ORG is 0 or FFFFFFFF.
  - `value` is what IAI's PC software shows: mm, or the number / label (`1`, `"1:Forward"`, `"disable"`).
  - Sanity ranges and value rules (UNVERIFIED beyond the values tried on hardware): P1 can only narrow the stroke (up
    to the largest P1 read in the session, not below the current position; the new stroke is enforced at once); P2
    0.01-1.00 mm (and >= MINI 3.67 when CINP / RINP are set); P4 0-5 mm; P9 only from the valid set for the
    extensibility level and <= MXPT 3.16 (the EC-R6S: only 0), 4 / 5 only on a unit already in that mode; P12 != 0
    not while P9 is 3 / 4. The write is verified by read-back.
  - **P8 (power saving, ATYP 7.7 bit 18; the EC-R6S reads "1:Disable", as the PC software showed) and P11, P13-P18
    are decoded (UNVERIFIED) but read-only.** A P8 change is expected to change other settings as well (UNVERIFIED);
    the others were never shown by the PC software on the tested units.
- **Commit semantics.** With `commit=True` (the default) the driver commits with a busy-flag handshake: it waits until
  DRGE b7 PARW = 0 and DSSE b7 = 1 (no parameter write in progress; also checked before the item write, so nothing is
  written while the unit is busy), sets coil 0x0428 ON (no OFF: it clears itself), then polls until DRGE b7 = 0 and
  DSSE b7 = 1 again (2 s). The coil frame and both flags are ON-WIRE (on the EC-R6S both flags were idle again within
  about 235 ms).
  - `result.commit_confirmed` is True only if the unit was seen acting on the coil (DRGE b7 read 1 or DSSE b7 read 0
    after it). b7 reading 0 alone cannot tell "finished" from "never latched", so an unseen commit is reported as
    **NOT confirmed** (a warning, `saved` False; the CLI and GUI say so) rather than as saved.
  - PARW saves **every** parameter value in RAM, including earlier uncommitted writes.
  - The value is in the register at once, but **the unit applies it only after a software reset or a power cycle**
    (`result.takes_effect`).
  - With `commit=False` the write is RAM only and reverts at the next power-up (VERIFIED), probably also at a software
    reset.
  - The CLI's `param set` commits unless `--no-commit` is given or the unit already holds the value. A value an earlier
    `--no-commit` run left in RAM is committed with `param set KEY VALUE --force` (each CLI run is a new process and
    cannot see that earlier write).
- **Unchanged values are not written.** The non-volatile memory lasts about 100,000 write cycles (IAI), so a value the
  unit already holds (same raw value) is neither written nor committed: `result.skipped` is True, `changed` and
  `committed` are False, and the note reads "unchanged: nothing written (one non-volatile write cycle saved)". Two
  exceptions: if a parameter this process wrote to the unit's RAM still waits for a commit (an earlier `commit=False`
  write, or a commit that failed or was not confirmed, through any axis object on the same unit; a serial port is known
  by its name, so a reconnect keeps it), the commit runs and saves it; and `force=True` (CLI `param set --force`)
  writes and commits anyway. The comparison is with the unit's RAM: a RAM-only write by another program or process
  (IA-OS, an earlier CLI run with `--no-commit`) is not seen, and a skip then reports `saved` True on that assumption;
  use `force=True` after one.
- **P3 / P4 (home direction / offset) are written with a warning** (`result.warnings`; no opt-in). After a committed
  P4 change and a
  software reset the WA (battery-less absolute) gripper came up with alarm 0x0EE "ABS error (2)" (home lost); only an
  alarm reset + home return cleared it. An incremental unit loses its home at every software reset anyway. Either way:
  alarm reset (if needed) + home before the next move.
- **Per unit code.** The driver keeps the parameter list IA-OS shows per unit code (every EC family) and applies
  visibility rules (`info.parameter_visibility`: extensibility level ACCL 0x0612, EXFN 8.50 b13, wireless module SUP2
  CNWL for P7, smart-tuning data STN1 for P8, the P9 valid set, SUP3 SORS for P16). On the units run so far that
  reproduces exactly what the PC software showed: P1-P7, P9, P10, P12 on the gripper (unit 0x0A, EC-GRBP8M) and
  P1-P6, P8, P9, P12 on the EC-R6S rod (unit 0x00).
  - A parameter not listed for the unit (e.g. P7 / P10 on the rod, whose items still decode; the hidden ones carry the
    rule as `hidden_reason`) is written with a warning that names the list (`... written anyway`).
  - A list is used for every unit with that unit code, whatever its 0x0600 model string. A unit code without a list,
    or one that cannot be read, offers every mapped item found on the unit. Every model's parameters are written the
    same way, with no model warning and no per-model opt-in. `user_parameters()` / `param list` / the GUI mark a
    parameter that is not listed for the unit, read-only or not settable (`model_note`) before any write;
    `verified_for_model` is kept for compatibility and only says that a list is recorded for the unit code.
- **Any family: `parameters()`, `read_parameter(key)`, `write_parameter(key, value, *, commit=True,
  allow_dangerous=False, allow_unlisted=False)`.** On an EC these are the P-numbers above. On a CON-family controller
  they are the numbered CON parameters (`iai_actuator_driver.con_params`), each offered only when its item is in the
  unit's own ID map (102 on the PCON-CB tested).
  - `key` is a number, `"P5"`, `"No.35"` or a raw item `"class.item"` / `"class.item[entry]"` (class in hex above 9).
    Each entry has its number, name, item, address, decoded value, unit, range / choices, `writable` and `dangerous`.
  - **Dangerous writes go through with a warning** (no opt-in): home direction / offset (EC P3 / P4, CON No.5 /
    No.22), motor / encoder type, lead and axis type, servo gains, PIO / fieldbus / baud configuration, widening a
    soft limit, and raw items of unknown meaning. `allow_dangerous`, `allow_home_params` and `allow_unlisted` (CLI
    `--allow-dangerous`, `--allow-home-params`, `--allow-unlisted`) are accepted and ignored (no longer needed).
  - The write is verified by read-back and committed with PARW like an EC parameter (VERIFIED on the PCON-CB: the
    committed value persisted across a software reset).
- **`software_reset()`** sends the sequence seen on the wire (ON-WIRE; run on the EC-R6S): servo OFF with
  `01 45 00 09 00 00 00 00` (waiting for DSS1 to show it), then `01 45 00 05 00 01 00 00`, answered `01 45 00 00`.
  - It requires the axis at rest (not moving, pushing, paused or homing) and no parameter commit in progress: it first
    waits up to 2 s for DRGE b7 = 0 and DSSE b7 = 1, else refuses without sending anything (a reset must not cut off a
    write to non-volatile memory). It takes Modbus control first. It never uses the SFRS coil 0x0420 (with the servo
    on, that raised alarm 0x090).
  - The link is re-synced in place (the port is never closed): after ~1 s the unit is probed at the current rate, then
    the usual rates, until it answers (`timeout`, default 10 s).
  - The result reports PMSS, safety speed (off), the servo (the EC switches it on again), HEND and `needs_home`. PMSS
    depends on the unit: the EC-R6S rod dropped it to 0, as after a power-up, so the next command takes control at
    rest; the EC-GRBP8M gripper and the PCON-CB keep PMSS 1 (VERIFIED).
  - **Incremental units (no WA option, e.g. the EC-R6S) lose their home at every software reset**: run `home()`. The
    uptime counter 0x9010 is **not** reset. Committed parameters take effect; the soft limits and speed limit are
    re-read.
  - `confirmed` / `confirmed_by` say how the restart was seen: the servo, switched off before the reset, coming back on
    by itself (VERIFIED on the EC-GRBP8M gripper), PMSS dropping to 0 (seen on the EC-R6S rod only), a silent gap
    after the command (CON controllers: the PCON-CB keeps PMSS = 1, its baud rate and its uptime across a reset), the
    uptime restarting, or HEND dropping. A reset that cannot be confirmed is reported, not raised.
- **`set_baudrate(rate, *, method="direct" | "break", allow_unverified_code=False)`** sends the broadcast
  `00 45 00 04 <code> 00 05` (ON-WIRE), switches the open port in place, then verifies with a read. The change is
  **volatile** (the unit is back at 38400 after every power-up; the driver finds the rate at connect), a broadcast
  switches **every** unit on the RS-485 line, and it runs only at rest.
  - **Only 115200 (0x14) and 230400 (0x15) are verified** (the EC-R6S switched both ways when they were sent direct).
  - **38400 (0x06) depends on the unit**: the EC-GRBP8M gripper and the PCON-CB take it (VERIFIED, sent direct from
    230400; the gripper also from 9600), the EC-R6S IGNORED it. So it is sent and the read after the switch
    decides: a unit that ignores it keeps its rate, the port is left there and `VerificationError` is raised.
  - Every other rate with an IAI code is sent with a WARNING log line (`code_verified` False): 9600 0x04 / 19200 0x05
    / 57600 0x13 are UNVERIFIED codes, and 14400 0x11 / 28800 0x12 / 76800 0x07 (also UNVERIFIED) are never probed by
    the auto-baud at connect (after one of those, reconnect with `--baud RATE` or power-cycle the unit back to 38400).
    A rate with no IAI code is refused, nothing sent. `allow_unverified_code` (CLI `--allow-unverified-code`) is
    accepted and ignored (no longer needed).
  - If the verifying read fails, the new rate, the previous one and then **every rate that has a known code** are
    probed (`VerificationError` if the unit answers at another rate; the port is left there).
  - `"direct"` broadcasts at the current rate. `"break"` uses the steps seen on the wire (ON-WIRE: port to 9600, a
    200 ms line BREAK, then the broadcast at 9600; per the CON manual a break of 150 ms or more drops the controller
    to 9600, DOC), so it is meant to work whatever rate the unit was at (UNVERIFIED with the driver).
- **`move_to_position(n, wait=True, timeout=None, *, allow_above_caps=False, speed_pct=None)`** runs stored row `n`
  (0 .. `max_positions()` - 1: MAXP 3.2 capped by the directory; 3 rows on an EC, 64 on the PCON-CB tested) with
  FC06 0x9800 = n, at the row's **own** speed and acceleration (a push row runs its push). With `speed_pct` (1-100) it
  uses 0x45 cmd 6 instead and runs the row at that share of its stored speed, with nothing written to the unit
  (VERIFIED on the EC gripper and the PCON-CB). The share goes in steps of 1/256: the unit runs op2 at (op2 + 1) / 256
  (ON-WIRE on the PCON-CB), so the driver sends op2 = round(256 x pct / 100) - 1 within 1..255 (25 % runs exactly,
  1 % runs 1.17 %, at least 0.78 %); a share rounded by more than 5 % is logged, and the CLIs print a note. CLI:
  `goto N [--speed-pct P] [--allow-fast] [--no-wait]`.
  - The row is checked first: its position (and a push row's push end) must lie in the effective stroke, it must not
    be incremental and its checksum must be valid. Unless `allow_above_caps`, its stored speed / accel / push force
    must also be within the driver's caps (the slider's rows run at 108.5 / 248.5 mm/s and the EC-R6S's backward end
    at 240 mm/s with 0.47 G decel, so those need it); the unit's own MAXV / MAXA / MAXD / MXPP are always checked. A
    push row's own low force is accepted (the EC-R6S: PPOW 26, ~10 %).
  - **The EC silently ignores 0x9800 while PMSS = 0** (VERIFIED), so Modbus control is taken first, at rest. If no
    motion starts within 0.5 s the driver stops and raises `MotionIncompleteError` ("ignored") instead of hanging.
    Completion uses the settle rules (a transient PEND during a push never ends the wait). A push row is refused while
    the axis is inside its push zone (the EC would skip the approach).
  - **On an EC only the rows of its position mode run** (user parameter P9, item 8.2): rows 0 and 1 in 2-position
    mode (P9 0 Double / 3 Single), rows 0-2 in 3-position mode. The EC-GRBP8M gripper in 2-position mode rejected
    `goto 2 --speed-pct 18` (0x45 cmd 6, row 2) with code 0x11 "1st operand error"; nothing moved, no alarm
    (VERIFIED). So the driver reads P9 (once, cached) and refuses an unused row before anything is sent:
    `SoftLimitError: row 2 (middle) is not used in 2-position mode (P9 = 0: 2-position (double solenoid)) ...`.
  - When P9 is not readable or holds an unknown value the row is sent, and a 0x11 rejection of cmd 6 is raised as the
    same `SoftLimitError` (the `IaiCommandRejected` as its cause). What 0x9800 does with an unused row is UNVERIFIED.
  - `teach_position()` and `write_position_table()` write an unused row with a warning (`... written anyway (e.g. to
    prepare a switch to 3-position mode)`); no opt-in. `max_positions()` still reports the 3 rows an EC can read and
    store.
- **Position-table push power.** `write_position_table()` always accepts the unit's own, unchanged push power (the
  EC-R6S's row 1 has PPOW 26), like its own unknown flag bits. A new value must be 1 .. min(`max_push_pct`, MXPP item
  7.9) %; a value below the recommended minimum (MNPP 7.58, or 20 %) is allowed, since MNPP is only a recommendation
  (the EC-R6S's own push row runs at ~10 %). Accel / decel are capped by MAXA / MAXD.
- **0x45 command forms.** `home(method="iai")` sends 0x45 cmd 0x0A with op1 = the unit's home pattern (ORG, read first
  by item ID 9.16; an invalid ORG is refused before anything is sent), `reset_alarm(method="iai")` sends 0x45 cmd 5
  op1 0, and `set_action_mode(False, safety)` sends 0x45 cmd 0x0C (PMSL + SFTY in one frame); `set_action_mode(True,
  safety)` writes the PMSL coil (when PMSS is 0) and the SFTY coil (when it changes) instead, since op1 = 1 left a
  PCON-CB refusing servo ON (above). Both only at rest, verified by reading PMSS / SFTY back. The frames are the ones verified on the EC-R6S; the coils (HOME, ALRS, PMSL, SFTY) stay the
  default. A refused 0x45 command raises `IaiCommandRejected` with the rejection's name and reason (`.rejection`,
  `.reason`, `.hint`).
- **`read_axis_limits()`** returns the unit's own limits (`AxisLimits`: MAXV, MAXA, MAXD, MXPH, MXPP, MNPP, TCPC /
  TCOA, AKID, SUPT push / relative-move support); `read_home_pattern()` the ORG pattern; `read_boot_baudrate()` the
  power-up rate BRSL (item 8.3), which `software_reset()` probes first.

### Unit info (`iai-actuator-driver info`)

Everything the unit tells about itself, decoded where the meaning is known. **Read-only**: only FC03 reads and FC11
Report Slave ID are sent, and every block read stays inside the unit's own directory at 0x0000, because the EC answers
0000 instead of an error for unmapped addresses.

```shell
python -m iai_actuator_driver --port COM3 info            # pretty text
python -m iai_actuator_driver --port COM3 info --json     # one JSON document (every field below)
python -m iai_actuator_driver --port COM3 info --section alarms  # only some parts (repeatable)
```

| Part | Content | Evidence |
|---|---|---|
| identity | 0x0600 model / user-defined unit name (0x061A, not a model field) / serials / ACCL extensibility level, FC11 (or CTCL 0x0300) system / unit / firmware with the family name, actuator kind (AKID 7.33), axis number (AXNO 0x8004), slave, detected baud, soft limits and stroke, MAXV / MAXA / MAXD and the driver's caps, position mode, `capabilities` (class-1 support bits SUPT / MVPT / SPTS / SUP2 / SUP3, push limits MXPP / MNPP / TCPC with a force estimate, push speed PSHV, power-up baud BRSL / RTIM, servo monitor FMNT, collision detection: MVPT b13 CDAS + class-8 items 119-127 read-only when present), the directory | values VERIFIED; many names and decodes UNVERIFIED |
| user parameters | P1-P18 with names, values and texts (`9.50 mm`, `1:Forward`, `0:Enable`, ...) found **by class + item ID** through the unit's ID maps, with per-unit flags (listed for this unit, `hidden_reason`, writable, read-only, home parameter), plus every raw item of classes 3 / 4 / 7 / 8 / 9 / 0x0C (hex, signed, x0.01, symbol) | P1-P7, P9, P10, P12 VERIFIED on the gripper; the P3 parity rule, P8, P11, P13-P18 UNVERIFIED |
| I/O | DIPM / DOPM with bit names for the position mode (user parameter 9: 0 Double, 3 Single = 2 positions, 4 = 3 positions on the slider), SIPM, ZONS, and lamp states (markers from ZONS) | DOPM layouts VERIFIED; DIPM ST0 (bit 0) and ST1 (bit 1) of the 2-position layout VERIFIED on the EC-GRBP8M gripper; RES (bit 2) not yet seen, the 3-position and single-solenoid DIPM names UNVERIFIED |
| status words | DSS1, DSS2, DSSE, ZONS, STAT (32 bit), SSSE, SIPM, POSS, and the DRG1 / DRG2 / DRGE readback, every bit with name, meaning, `confirmed` flag and evidence | VERIFIED where `confirmed`; otherwise DOC (CON manual) or UNVERIFIED |
| monitor | position, VNOW, CNOW (and % of the rated current MCUR 4.5), deviation, uptime (0x9010 ms), overload level | see `notes` in the output |
| alarms | present alarm (ALMC, name, level message / operation cancelled / cold start, fault or warning, group, how to reset, home needed, the driver's own remedy text, class 3 item 10) and the 32-slot history (index 0 = newest) with names, levels, groups, time after power-on (ms) and decoded detail (0x0A2 "row n, field X"); power-up markers and empty slots are counted, not listed | codes and history VERIFIED; names, levels and groups UNVERIFIED unless seen on hardware |
| maintenance | total moves (0x8400) and distance in m (0x8402), every other maintenance item raw with its symbol | TLMC / ODOM VERIFIED; other symbols UNVERIFIED |
| positions | rows 0-2 (mm, mm/s, G, push end, checksum), the % rows 260 / 261 (present %) and 262 / 263 (previous %), plus 266 (the Middle %) in 3-position mode, the last 0x9900 command block and the active profile 0x0F00 | values VERIFIED; the roles of rows 260-266 UNVERIFIED |

From Python:

- `ec.unit_info(sections=None)` returns the same JSON-serialisable dict (schema in
  `iai_actuator_driver.info.unit_info`).
- `iai_actuator_driver.info.read_class(ec, class_id)` reads one class by item ID.
- `read_block(ec, class_id, offset, count)` reads raw words of a directory block (refused outside it). A block of
  uncertain size (mixed member sizes the driver does not know) may be read up to the largest extent its directory
  record allows (`readable_words(record, records)`), because live data sits there, e.g. 0x910F-0x9113.
- `decode_bits(word, table)` decodes a word with one of the named bit tables (`BIT_TABLES`,
  `io_bit_tables(position_mode)`).
- `iai-check` stores the same information in its report (`unit_info`; it never fails the check).

### The compatibility self-test (`iai-check`)

Prints one PASS / FAIL / WARN / SKIP line per item and writes a JSON report (`iai-check-<timestamp>.json`, or
`--report PATH`) with model, serial number, firmware, soft limits, timestamp and all raw data. If the report cannot be
written there, it goes to the temp directory or, failing that, to the console. When you try a new model or firmware,
attach the report to a unit report ([Sending a report](COMPATIBILITY.md#sending-a-report)).

| Item | What it checks (read-only unless noted) |
|---|---|
| link | the unit answers a register read |
| slave_id | Report Slave ID says it is an EC; family, actuator kind and firmware (WARN if FC11 is not answered, in which case the identity comes from CTCL 0x0300) |
| index_table | the unit's self-description puts the monitor, command, position and coil areas where the driver expects them |
| identification | model and serial-number strings (0x0600) |
| soft_limits | the unit's own soft limits (0x0700); WARN if `--stroke` exceeds them (the driver narrows it), FAIL if they do not overlap |
| speed_limit | the unit's maximum speed MAXV (item 7.4; 0x0812 only as the fallback), MAXA / MAXD and the resulting caps; WARN if not readable |
| status / alarm | status bits decode; no fault or emergency stop (a message-level warning is WARN, not FAIL) |
| alarm_history | the unit's alarm history (0x0500) |
| position_table | positions 0-2 readable and inside the stroke |
| direct_move_block | the 0x9900 numeric-move registers are readable |
| modbus_control | whether Modbus currently has command authority (PMSS = 0 after power-up is normal: the driver takes control at rest) |
| safety_speed | WARN if the unit's safety-speed mode (DSS1 bit 14) is on |
| unit_info | everything `iai-actuator-driver info` shows, stored in the report's `unit_info` (WARN if parts could not be read; never FAIL) |
| motion_out / motion_back | **only with `--motion`**: +5 mm (or -5 mm near the end) at 10 mm/s and back, reporting the settled position (`reached_mm`, `error_mm`); servo / Modbus-control state is restored afterwards |

Without `--motion` nothing is written to the unit. The report also records the model string, the FC11 controller
description, the baud rate and the speed limit.

## Any IAI unit: units, ranges, rotary, CON controllers

The driver takes every range and capability from the unit it is talking to and keeps no list of models. The EC and
the PCON-CB (a CON-family controller) behave the same, and every IAI line that speaks Modbus RTU is driven the same
way: any family, unit code or actuator kind gets the same commands, with no driver update for a new model.

```python
from iai_actuator_driver import Axis             # Axis is Elecylinder: one class for every unit

with Axis.open("COM3") as ax:
    info = ax.connect()
    print(info.product_family, info.model, ax.kind, ax.units)   # e.g. "PCON", ..., "slider", "mm"
    lim = ax.limits()                            # every range the driver enforces, with its source
    print(lim["position"], lim["speed"], lim["stored_rows"])
    ax.servo(True)
    ax.move_to(120.0, speed_mm_s=100, accel_g=0.3, decel_g=0.1)  # own deceleration (command profile)
    ax.move_to_position(6, speed_pct=25)        # stored row 6 at 25 % of its stored speed
    print(ax.read_monitor())                    # position, velocity, current, deviation, alarm, uptime
```

- **Units.** `ax.units` is `"deg"` on a rotary axis (AKID item 7.33 = 1) and `"mm"` otherwise (an unknown or
  unreadable kind included). Positions, bands and the `*_mm` arguments are then in degrees and speeds in deg/s;
  accelerations stay in G.
- **Ranges** (`limits()`, CLI `limits`): position = the unit's soft limits 7.1 / 7.2 minus the 0.30 margin
  (intersected with a configured `stroke`); speed up to min(MAXV 7.4, `max_speed_mm_s`); acceleration / deceleration
  up to min(MAXA 7.5 / MAXD 7.6, `max_accel_g`); push force 1 .. min(MXPP 7.9, `max_push_pct`) % with MNPP 7.58 as
  the recommended minimum (below it the driver only warns); the positioning band; stored rows 0 ..
  `max_positions()` - 1; the jog step; the speed override 1-100 %. The `max_*` caps default to `None` (the unit's own
  maxima); a value given can only lower them. Only when neither the unit's maximum nor a cap is known does the driver
  fall back to 100 units/s, 0.3 G and 70 %.
- **Identity.** The family comes from the FC11 system code (`product_family`, `is_ec`); a CON controller's model text
  from its name block at 0x8010 (the PCON-CB reads `PCON-CB (FB)`, `controller_name`). Every string is reduced to
  printable ASCII; unexpected bytes are recorded in `DeviceInfo.errors` instead of failing.
- **Moves.** `move_to(..., decel_g=...)` with a deceleration different from the acceleration uses the command-profile
  path (FC17 write of the 20-word profile at 0x0F00, then the PCRF coil starts it; MAXD is checked first). It is
  VERIFIED on the EC gripper and the PCON-CB (start, re-target mid-move, STOP, incremental). Without `decel_g` the
  direct numeric move (0x9900) is used.
- **Monitor.** `read_monitor()` (CLI `monitor`) reads the whole monitor block once: position, the unit's own velocity,
  current (raw and as % of the rated current MCUR 4.5 when readable), deviation, alarm with name and level, uptime.
- **Writes need no opt-in**: home direction / offset, encoder / motor type, lead / stroke configuration, servo gains,
  baud codes not verified on hardware and non-volatile position-table writes go through, a risky one with a warning.
  What stays: the never-send list of raw IAI commands (`allow_never_send`), the IAI command allow-list
  (`allow_unsafe`), the range checks and the non-volatile write guard.

## Several axes: ports, scan, Cell

The normal setup is **one axis per COM port**: one IAI USB adapter per actuator, all driven in parallel, the fastest
layout (VERIFIED one unit per adapter). A cell takes every axis it finds; `max_axes` caps it if given. Several axes can
also share one port by axis number (an RCON gateway or an SIO daisy chain, slave address = axis number + 1; UNVERIFIED
on hardware), and the two can be mixed.

```python
from iai_actuator_driver import Cell, list_ports, scan

print(list_ports())                    # IAI USB adapters (USB ID 10C4:81D7, VERIFIED) first; opens nothing
print(scan("COM3"))                    # ScanHit per answering address: family, unit code, firmware
with Cell.open() as cell:              # every IAI adapter, every answering address 1..16 (read-only)
    print(cell.names)                  # ("COM3:1", "COM4:1", ...): port:slave
    print(cell.limits_all())           # each unit's own ranges, units and kind
    cell.servo_all(True)
    cell.home_all()
    cell.move_all({"COM3:1": 50.0, "COM4:1": 20.0}, speed_mm_s=50, decel_g={"COM4:1": 0.1, "COM3:1": 0.1})
    print(cell.status_all(), cell.monitor_all())
    cell.servo_all(False)
```

- **Opening.** `Cell.open(ports=None, slaves="scan", ...)`: `ports` is `None` (every IAI adapter), a port name, a
  transport, a `Port`, a list of these or `{name: spec}`; `slaves` is `"scan"`, a list for every port or
  `{port: [slaves]}`. Opening is read-only and identifies every axis (`cell.info`). With a scan a dead port is skipped
  and recorded in `cell.port_errors`; with listed slaves any failure raises.
- **Ports and threads.** A port is opened once (`Port`: one bus with its own lock, baud rate detected once in place).
  Ports work in parallel threads; the axes of one port take turns on its lock, and waiting loops sleep without holding
  it, so axes on a shared line move at the same time.
- **Group commands.** Every `*_all` method takes `axes=None` (all) and `return_exceptions=False`; any failure raises
  `CellError` (with every result and error, keyed by axis name) after all axes were handled. Values for `move_all`
  (speed, accel, band, decel) are one value or `{axis: value}`; positions are in each axis's own units
  (`cell.units_of(axis)`).
- **Stopping.** `stop_all()` writes the STOP coil to every axis first (ports in parallel), then waits for each. A
  failure on one axis during `move_all` / `home_all` / `wait_all` stops the others too (`stop_others_on_error`), and
  leaving a `with Cell` block on an exception after motion stops every axis. `stop_all(broadcast=True)` first sends
  one broadcast STOP on every daisy chain whose axes are all selected (never behind a gateway; UNVERIFIED), then the
  per-axis stop as the fallback; `pause_all()` / `resume_all()` likewise.
- **Lines and gateways.** The line is detected once when a port opens (two read-only FC03 probes): `"direct"`,
  `"rcon"`, `"robonet"` (stated with `line="robonet"`: SIO through mode cannot be probed) or `"robonet-gateway"`
  (`UnsupportedLineError`). Behind a gateway a command the line does not carry is refused before it is sent
  (`LineRuleError`); the rules per line are in [DRIVER.md](DRIVER.md#lines-and-gateways). `Port.line`,
  `Port.gateway_status()`, `Port.gateway_monitor()` and `Cell.gateway_status_all()` read the gateway;
  `Cell.status_all()` takes an RCON line's axes from the gateway monitor in 2 frames for 16 axes. All of it is DOC
  (ME0413, ME0208) and UNVERIFIED on hardware. An ELECYLINDER on an RCON-EC / REC connection unit has its serial port
  disabled and is not supported.
- **Backup.** `Cell.backup(directory)` is a read-only backup of every axis, one JSON file each.
- **Baud rate.** `Port.set_baudrate(rate)` is a broadcast: **every** unit on that line switches. It requires every
  axis at rest and then checks that each answers (several units switching at once is UNVERIFIED).
- **CLI.** `iai-actuator-driver ports [--json]`, `iai-actuator-driver scan [--ports COM3,COM4] [--slaves 1-16]
  [--baud N] [--scan-timeout S] [--json]`, and `--axis N` on every single-axis command and on `iai-check`.
- **Slave 58 is never used.** 58 = 0x3A = ':', the Modbus ASCII start: every IAI unit on the line takes a request to
  it for an ASCII frame and raises alarm 0x05E "Delimiter error" (ON-WIRE on a PCON-CB). Scans and baud detections
  skip it (one warning line), and a request or an open of slave 58 is refused with nothing sent (`ValueError` /
  `ArgumentException` / `std::invalid_argument`).

## Safety notes

- **This software is not a safety function.** Keep a hardware emergency stop and guarding as for any machine. Test
  new setups with the actuator unloaded, mid-stroke, slow (<= 50 mm/s).
- **No communication watchdog.** The EC has none (VERIFIED): if the PC crashes, hangs or the cable is pulled, a move
  in progress is **completed**, and a push keeps pushing. Design the cell so that this is safe, and use the hardware
  emergency stop to stop the axis independently of the PC.
- **One owner per port.** IA-OS (or any other program) must be disconnected. The port is opened once and never
  toggled; the baud rate is found by changing it in place (ECs power up at 38400). Per the CON manual (DOC) a line
  break of 150 ms or more drops the rate to 9600 (VERIFIED on the EC-GRBP8M gripper and the PCON-CB); the driver sends
  a break only in `set_baudrate(method="break")`.
- **Soft limits and caps.** Every target is checked against the *effective stroke*: the configured `stroke`, if any,
  intersected with the unit's own soft limits read from 0x0700 minus the 0.30 mm margin. Every speed is checked
  against min(`max_speed_mm_s`, the unit's MAXV, item 7.4: 53 mm/s on the gripper), acceleration against
  min(`max_accel_g`, MAXA / MAXD, items 7.5 / 7.6: 0.30 G on the gripper) and push force against min(`max_push_pct`,
  MXPP, item 7.9: the hard cap). The `max_*` caps default to the unit's own maxima. All of this happens *before*
  anything is written; violations raise `SoftLimitError` (e.g. "speed 60 mm/s exceeds this EC's maximum speed
  53.00 mm/s (MAXV item 7.4 @0x0706)").
  - The driver reads these items by item ID. 0x0812 is only the *default* speed (item 8.10; that role is UNVERIFIED,
    it equals MAXV on the tested units) and is used only when MAXV cannot be read.
  - If the unit's limits cannot be read and no stroke is configured, motion is refused. The EC itself silently clamps
    out-of-range targets to its stroke +/-0.10 mm (no alarm), so these checks matter.
  - On a rotary unit (AKID = 1) the same checks run in degrees.
- **Alarm levels.** The message level is VERIFIED for 0x069 on a PCON-CB; the other codes, names and groups are
  UNVERIFIED unless seen on hardware.
  - Codes 0x011-0x07F are *messages*: the axis keeps running (maintenance warnings 0x048 / 0x04A / 0x04E / 0x04F,
    deviation warning 0x047, collision warning 0x049). They set `Status.warning`, never `Status.alarm`, never block
    motion and never light the red lamp (the GUI shows an amber Warning lamp). On the PCON-CB the warning bit stayed
    on after an alarm reset cleared 0x069 (VERIFIED), so `Status.warning` can be true with alarm code 0.
  - A *fault* (`Status.alarm`) is DSS1 b10 ALMH or a code of level operation cancelled (alarm reset clears it) or cold
    start (software reset or power cycle; incremental units then need a home return). An unknown-level code without
    the warning bit is a fault (fail safe).
  - `Status.alarm_level` and `iai_actuator_driver.alarms.alarm_info(code)` give the level, group, reset method and the
    driver's own short remedy.
- **Every move states every field.** Moves write the full 9-word 0x9900 block, including PPOW and CTLF. A CTLF bit
  left behind by another tool would otherwise turn the next "absolute" move into a relative move or a push.
- **Completion.** A move is done when the unit reports in-position (PEND, or PSFL for a push), the position is within
  the band **and** the position has stayed unchanged for `settle_s` (0.1 s, two polls).
  - On hardware PEND and MEND rise inside the band while the axis is still moving (the gripper reported 3.04 mm with
    PEND + MEND, then 3.00 mm), so the status `move_to()` returns is the settled one.
  - `completion="fast"` ends on the first in-band sample instead, so its status can be up to `band_mm` short of the
    target (up to 0.08 mm on the PCON-CB, VERIFIED); see [DRIVER.md](DRIVER.md#fast-path).
  - The same rule ends a home return and the STOP hold. The EC's MOVE bit is not trusted: it clears while still
    decelerating.
  - If the axis settles away from the target (stopped by someone else, command ignored, EC clamp),
    `MotionIncompleteError` is raised instead of waiting for the timeout.
  - A `stop()` of the same axis from another thread, or from another Axis object on the same line, ends the wait at
    once: `MotionIncompleteError` ("stopped from outside this wait"), with no second STOP from the wait (ON-WIRE on a
    PCON-CB + RCP6). A STOP of another program shows in the status alone: once the axis was seen moving and then rests
    short of the target with the move cancelled (PEND on, MEND off, not paused) for `settle_s`, the wait ends the same
    way (UNVERIFIED: simulator only). A newer move of the same
    axis (re-targeting, another thread's move, a push, a stored row, home) ends the older move's wait the same way
    ("superseded by a newer move of this axis") and sends no STOP, which would cut the newer move.
- **Preconditions.** Moves require Modbus control, servo on, homed, no alarm or emergency stop, not paused and no home
  return in progress; otherwise `EcNotReadyError` / `EcAlarmError` with a hint.
- **Alarm reset.** `reset_alarm()` leaves a servo the alarm switched off **off**: an alarm keeps the servo-on level
  (SON coil / DRG1 b12) set, and on a PCON-CB the reset alone re-energised the motor (ON-WIRE, 2026-11-03), so the
  driver clears that level first, on the reset's own path. Call `servo(True)` afterwards. A servo that is on stays on.
- **Modbus control (PMSL).** The EC powers up with PMSS = 0. The driver takes control (PMSL coil 0x0427)
  automatically before a command, **but only while the axis is at rest**: on hardware, switching PMSL on in the middle
  of a move stopped that move. With PMSS = 0 and the axis moving, commands raise `EcNotReadyError` instead of touching
  PMSL. A command the driver refuses (servo off, not homed, a blocked servo ON, a moving axis, a push row or push
  started inside its zone, a bad home pattern) is refused before control is taken, so nothing is written, not even
  PMSL (a refused move had switched PMSS 0 -> 1 first: ON-WIRE on the PCON-CB, 2026-11-03). `auto_modbus_control=False`
  makes the driver require an explicit `enable_modbus_control()`. PMSS does not gate
  0x9900 moves on the EC: a move sent by another tool runs whatever PMSS is. A pause at a standstill with the servo off
  (DSS1 STP, no SV, no MOVE, no home return, no push and VNOW 0) does not block taking control; a pause with the servo
  on does (a PLC holding a stop input on a move), and a PLC pause with the servo off is still indistinguishable from an
  unwired stop input. A PCON in PIO mode with nothing on its PIO connector pauses and switches its servo off, and the
  servo stays off once PC control is back (VERIFIED on the PCON-CB, 2026-11-02). 1.0.1 taking control there: VERIFIED
  (PCON-CB, 2026-11-02). Handing control back to PIO is still refused while a pause is on.
- **Stop.** `stop()` and every automatic stop use the hardware-verified sequence: STOP coil 0x042C ON, wait until the
  axis has stopped (position unchanged for `settle_s`), STOP OFF. It never touches PMSL. With PMSS = 0 it stopped a
  move commanded over Modbus (VERIFIED); it has **never been tried on a move a PLC commanded over PIO**. If a PLC holds
  a level start input, the EC might start the move again once STOP is released, so stop PLC-commanded moves from the
  PLC or with the hardware stop. Whether the EC would release STOP by itself is not known, so it is always released
  explicitly (a release that fails is retried before the next command and on `close()`). The experimental IAI
  strategies are never used to stop. The software STOP is not a safety function: it goes over a serial line from a
  general-purpose PC and can be late, so a hardware emergency stop or STO is required wherever a late stop could hurt
  someone.
- **Stop on failure.** If anything goes wrong while a commanded motion may be running (timeout, alarm, lost link,
  Ctrl+C, an exception inside a `with Elecylinder...` block after a motion command), a decel stop is attempted before
  the error propagates; the exception carries a note saying whether it was sent. Read-only use never writes to the
  unit, also on errors.
- **Pause.** STP (0x040A) is a level input on the EC: while it is ON nothing moves and the driver refuses motion
  commands ("call resume()"); on release a paused move continues by itself.
- **Push.** `push()` approaches the target at the given speed, then pushes over the band at the unit's push speed
  (PSHV, item 8.15; ~20 mm/s on the EC-R6S) with the given current limit, up to min(`max_push_pct`, MXPP).
  - Below the recommended minimum (MNPP, or 20 %) it is allowed with a warning (`PushResult.warnings`); it is refused
    when the unit cannot push (SUPT PUSH = 0, MXPP = 0 or TCPC = 0).
  - The axis must start on the approach side of the push zone (the EC would otherwise skip the approach and push from
    where it is), and the band end must lie inside the stroke. After contact the motor keeps pushing until the next
    move command.
  - The outcome (contact or missed) is reported only after it has held for `push_settle_s` (0.15 s) with the position
    unchanged (a holding contact may creep 0.05 mm): on the gripper a push showed the full contact fingerprint (PEND,
    MEND, DSSE 0x37C3) for a moment at 6.76 mm and then missed at 7.00 mm (VERIFIED).
- **Jog.** The EC's JOG coils run at ~100 mm/s from an internal parameter, whatever speed was intended; the driver
  never uses them. `jog()` is a relative move at a low default speed.
- **Non-volatile memory: the position table.** `write_position_table()` needs no opt-in (`allow_nonvolatile` is
  accepted and ignored).
  - Rows 256-267 (the EC's internal store) and rows the table cannot address are refused with
    `NonVolatileWriteError`. A row past the unit's rows (on an EC past 0-2), unknown control-flag bits and a change of
    the unknown 0x0010 / 0x0020 role bits of rows 1 and 2 are written with a warning. Every field is validated (push
    power, load threshold, finite values), the checksum word is written and the row is verified by read-back. Change
    an entry by read-modify-write:
    `ec.write_position_table(2, dataclasses.replace(ec.read_position_table(2), position_mm=60.0))`.
  - A row write is non-volatile at once (VERIFIED) and the unit uses it at once.
  - A row that would not change (the words the write would produce equal the stored ones, checksum included) is not
    written: the stored row comes back with `skipped=True`. `teach_position()` does the same when the row already
    holds PNOW (0.01 resolution) and nothing else would be filled in. `force=True` (CLI `teach --force`) writes
    anyway.
  - The unchanged-row check comes before the stroke / speed / accel / push checks, which only matter for a real write:
    a same-value teach or row write on an axis opened with a lower cap than the row's own speed is skipped, not
    refused. A refused value names the row, the field, its unit and whose limit it is, e.g. `row 0 stored speed
    53 mm/s is above this axis's max_speed_mm_s 10 mm/s; open the axis with a higher max_speed_mm_s (command line:
    --max-speed or --unit-max) to write it`.
  - Back up the unit first (`iai-actuator-driver backup`). Parameters are written only by `write_parameter()` /
    `write_user_parameter()` (see above: committed, applies after a software reset).
- **Write budget.** The non-volatile memory lasts about 100,000 write cycles (IAI; the nominal limit IAI also gives for
  the serial EEPROM of the ROBONET axis units, DOC ME0208-14B p.190 and alarms 0F5 / 0F6 / 0F8 p.367). The PCON-CB /
  ACON-CB / DCON-CB / SCON-CB manuals state "no limitation in number of writing" (DOC ME0342-4H p.2-11, ME0343-3B
  p.2-13, ME0340-9F p.2-12); the guard runs the same on every unit.
  - Written there: a parameter commit (PARW, VERIFIED), a stored-row write (VERIFIED) and a teach (table: the same row
    write; coil: TEAC, which writes the position to the EEPROM, DOC ME0162-12A PWRT).
  - Not written there: direct moves (0x9900, `move_to`, coordinated motion; VERIFIED), stored-row moves (0x9800 /
    0x45 cmd 6 only read the row), the PMSL and SFTY coils and the baud change (back at PMSS 0, safety speed off and
    38400 after every power-up, VERIFIED), the action mode 0x45 cmd 0x0C (the same PMSS / SFTY state; after a power
    cycle the mode IA-OS had set was gone, VERIFIED), servo, alarm reset, software reset and every read.
    `recover()`'s `restore_control` only takes PC control, the safety speed and the baud rate back.
  - The command profile 0x0F00 of `move_to` with a separate deceleration shows the profile of whatever move ran last
    (UNVERIFIED as RAM, but it is rewritten by every move).
  - The unit itself writes its alarm history (every alarm, and a 0x0FFF marker at power-up, VERIFIED) and, on an
    absolute unit, possibly its home data at a home return (UNVERIFIED).
  - Per unit, this process counts its writes over every axis object on it, so a program that opens the axis per job
    keeps counting (`ax.nonvolatile_writes`, `ax.nonvolatile_writes_last_hour`). It logs one WARNING per hour while
    more than `NV_WARN_PER_HOUR` (60) fall in the last hour, with how many days 100,000 writes last at that rate; it
    never refuses a write. For a target that changes every cycle use `move_to`.
- **Dangerous IAI commands are refused.** Only an allow-list of 0x45/0x48 commands (servo, home, stop, alarm reset,
  action mode, ...), addressed to ID group 0, is sent without `allow_unsafe=True`.
  - The **never-send list** (`registers.IAI_NEVER_SEND` / `IAI2_NEVER_SEND`) needs `allow_never_send=True`
    (`allow_unsafe` alone is not enough): 0x01 / 0x02 non-volatile writes, 0x03 / 0x73 axis number, 0x04 baud, 0x08 ID
    group, 0x0F alarm-list clear, 0x10 parameter initialisation, 0x11 DO type, 0x1D absolute initialisation,
    0x20-0x3C maintenance writes, 0x4F, 0x90 / 0x91 firmware update, 0xDD-0xFF inspection incl. 0xE8 / 0xF3, and the
    0x48 equivalents. The command meanings are UNVERIFIED except 0x04 (ON-WIRE); the list errs on the side of
    refusing.
  - The driver never builds 0x47 (encoder FRAM) frames at all. It sends only two such commands, inside dedicated
    methods: the software reset (`software_reset()`, `allow_unsafe`) and the broadcast baud change (`set_baudrate()`,
    the only use of broadcast address 0). It never sends parameter / absolute initialisation, an alarm-list clear or
    an axis-number change.
- **Rejections.** A 0x45 reply with a non-zero rejection raises `IaiCommandRejected` naming it (0x11 1st operand
  error, seen on the EC-R6S; the other names are UNVERIFIED: 0x23 servo OFF, 0x24 during homing, 0x25 software reset
  with the servo ON, 0x26 TEACH mode, 0x27 invalid HOME, 0x3A monitor (PIO) mode, ...; 0x48: 0x1000-0x13FF). It is
  **never retried** (it is an answer, not a lost frame).
- **Retries.** Timeouts and CRC errors are retried (default 3 for `Elecylinder.open`, `Port`, `Cell` and the CLI: the
  manufacturer's recommendation, DOC ME0162-12A p.4-2), with at least 5 ms between requests. Report Slave ID at
  connect is sent once, waiting up to 0.65 s, since the rod answers after 0.2-0.4 s; CTCL is the fallback. Retries
  are required: on hardware, the read sent right after a mid-move write got no reply. All commands the driver sends
  are idempotent (absolute targets, coil levels); pass `retries=0` for anything that is not.
- **Safety speed.** The safety-speed mode (DSS1 / DRG1 bit 14, coil 0x0401) caps motion at about 100 mm/s (VERIFIED);
  it is off after power-up, and the PC software's Teach mode can leave it on. `Status.safety_speed` and `iai-check`
  report it; the driver never changes it unless you call `set_safety_speed(on)`.
- **Teach vs Monitor mode.** In the PC software's Monitor mode the PC cannot move the unit. While Modbus control is
  enabled, PIO inputs are ignored: call `disable_modbus_control()` (or set Monitor mode in the PC software) before
  handing the actuator back to a PLC.

## Command strategies

Moves always use the direct numeric command (FC10, 9 registers at 0x9900) and stops always use the STOP coil. How
servo / home / alarm reset are sent is pluggable, and `home(method=...)` / `reset_alarm(method=...)` pick `"coil"` or
`"iai"` per call:

| `strategy=` | Path | Evidence |
|---|---|---|
| `"coil"` (default) | FC05 coils SON 0x0403, ALRS 0x0407, HOME 0x040B | VERIFIED |
| `"iai"` | IAI function 0x45: servo 9, home 0x0A with op1 = ORG (item 9.16), alarm reset 5 op1 0 | servo 9 ON-WIRE and VERIFIED; home and alarm reset VERIFIED on the EC-R6S (home with op1 = 0 is rejected 0x11 "1st operand error", hence ORG) |
| `"iai2"` (experimental) | IAI function 0x48: servo 0x30, home 0x31, alarm reset 0x23 | UNVERIFIED; never seen on the wire, request layout unresolved (16-bit command assumed) |
| `"register"` | FC06 writes of DRG1 0x0D00 (SON, SFTY, STP, ALRS bits; EMG and BKRL are never set); STOP is pause, wait for standstill, ALRS edge while paused, release | DOC (ME0162-12A p.5-124 to 5-128, ME0413 p.5-11); UNVERIFIED on hardware. Chosen automatically behind an RCON gateway, and when a unit answers FC05 with exception 01 |

## Evidence levels

Every register, bit and command in `bindings/python/iai_actuator_driver/registers.py` is tagged with how well it is
established:

| Level | Meaning | How much to trust it |
|---|---|---|
| **VERIFIED** (also written VERIFIED-LIVE) | Confirmed on hardware: read (or written) on a real unit and the result made sense | High, for that model/firmware |
| **ON-WIRE** | Seen in serial traffic | High for the frame itself; the unit's reaction is VERIFIED only where also tested |
| **BACKUP** | Present in a backup file saved from the unit, so the unit exposes it | High that it exists; meaning may be inferred |
| **DOC** (DOC-CON) | Documented in a public IAI manual; DOC-CON = documented only for the CON controller family (Modbus manual ME0162, which does not list the EC) | Plausible; must be verified on the EC |
| **UNVERIFIED** | Expected behaviour, not yet confirmed on hardware | Treat as an assumption; the driver guards these paths |

The units run so far are listed under [Run on hardware so far](#run-on-hardware-so-far).

**Where the EC differs from the CON documentation:**

- VNOW is 0.01 mm/s but quantized in ~12.5 mm/s steps (the driver derives speed from successive positions).
- DSS1 bit 0 means "move command finished" (`Status.target_reached`; also set after a missed push).
- DSSE bit 0 marks a push holding after contact.
- The MOVE bit clears before the axis has stopped.
- PMSS does not gate 0x9900 moves or STOP, and switching PMSL on mid-move stops the move.

Unit codes are shown with family names (display only: unit 0x0A = EC(MS) "2-size EC", 0x00 = EC standard, 0x02 =
EC dust/drip-proof); the actuator kind (gripper / rod / slider) comes from AKID (item 7.33), and the display name from
the 0x0600 model string. 0x061A holds a user-editable unit name, not a model field.

**Open hardware questions:**

- whether the EC releases STOP by itself;
- the exact duration of the commit busy flags (DRGE b7 / DSSE b7; seen to clear again within ~235 ms);
- whether the EC uses a code other than 0x06 for 38400;
- whether an uncommitted RAM write survives a software reset;
- the EC's baud rate right after a software reset (the driver re-syncs either way);
- whether the EC itself updates the position-entry checksum (the driver writes it);
- the meaning of the 0x0010 / 0x0020 flag bits of rows 1 and 2, and the 0x48 request layout;
- what PMSS does gate (0x9800 yes, 0x9900 and STOP no);
- P8 (power saving) read from ATYP 7.7 bit 18 and the P11 / P13-P18 decodes (UNVERIFIED, read-only here);
- the axis running with a message-level alarm active (0x069 was seen on a PCON-CB: raised at power-up, cleared by an
  alarm reset, DSS1 b9 stays set; expected: it keeps running);
- whether the ZONS LS range really is parameter 2 (0.10 mm on both units; the logged markers fit +/-0.10 mm).

**Open hardware test: Modbus STOP on a PIO-commanded move.** A PLC (or a switch on the PIO start input) starts a move
with PMSS = 0 and **keeps the start input on**; the PC sends STOP (0x042C ON), waits for standstill, releases STOP
(OFF) and logs PNOW / DSS1 / DSSE for 5 s. Record whether the STOP stops the move, whether the move restarts after the
release, and the same with the start input dropped before the release. Until then, treat a Modbus STOP as untested on
PLC-commanded moves.

## Test GUI

A local web GUI to drive axes by hand, with status lamps. Stdlib only, loopback only (127.0.0.1); every `/api`
request needs the per-run token embedded in the page. After `pip install ./bindings/python ./apps/gui`:

```shell
iai-gui                         # http://127.0.0.1:8780 (Ctrl+C to quit)
iai-gui --sim-only              # demo mode: simulators only, no serial port is listed or opened (either page)
iai-gui --port 8790             # another HTTP port
iai-gui --stroke 0,50           # optional default stroke override in mm (can only narrow)
```

`python apps/gui/server.py` does the same from a source checkout. The GUI runs on the Python package, so on the core
([where the library comes from](getting-started/python.md#where-the-native-library-comes-from)).

**Both pages:**

- **Header.** Title, version, the unofficial notice, an **About** button and the large **STOP** (STOP ALL on the
  workbench; Space or Esc) stay on screen. **About** shows the version, the unofficial-use and not-a-safety-function
  notice, the four evidence words and links to the documents (served read-only as plain text from a source checkout;
  listed as not part of the install otherwise).
- **Demo mode (`--sim-only`).** A banner says so, the serial-port controls are hidden, the single-axis page keeps
  *Simulator (no serial port)* on, and the server refuses a connect request for a real port (409).
- **Labels.** The same names as the command line and the function cards: Servo ON / OFF, Home, Alarm reset (never a
  software reset), Software reset, STOP.
- **Limits.** Every limit is checked by the server and again by the driver. Jog steps are relative moves, never the
  EC's JOG coils.

**Workbench** (the page at `/`): several ports and axes, a *Functions* view with a card for every driver function
(range and example under each input), and a *Group* view.

- The cards include *Change speed / acceleration of the running move* and *Cancel a paused move* (also a quick
  button; both VERIFIED on the EC gripper), *Teach current position* (non-volatile: asks first), *Back up this unit*
  (read-only JSON in `backups/`) and *Line and gateway* (the line kind, its rules, the gateway status, the RCON monitor
  record).
- The axis lamps also show the condition flags while they are set (EMG IN, PWR CUT, ENB OFF, COLD, ASOF, NV, AUTO,
  BATT, EXC). A servo ON that cannot work is refused with its reasons. A port opened behind a gateway shows its line.
- *STOP ALL* has an "also broadcast STOP (daisy chains)" option (UNVERIFIED). The no-answer message names the
  unsupported controllers (SEL protocol; an EC on an RCON-EC / REC).

**Single-axis page (`/classic`):**

- **Connect.** Tick *Simulator (no serial port)* for the built-in simulator (*Sim profile*: slider, gripper or the
  EC-R6S rod), or untick it and give the port (close IA-OS first). The selected baud rate is tried first, then the
  others, in place on the open port. Connecting only reads from the unit.
- **Stroke.** Empty fields use the unit's own soft limits (0x0700 minus the 0.30 mm margin). Typed values can only
  narrow them. If the limits cannot be read and no stroke is entered, motion stays disabled for that connection.
- **Caps.** Speed up to min(100 mm/s, MAXV item 7.4), with an error message naming the unit's limit; accel up to
  min(0.3 G, MAXA / MAXD); push force 20 .. min(70 %, MXPP item 7.9).
- **Lamps.** Servo ON, Homed, In position, Moving, Alarm (red: a fault), Warning (amber: a message-level alarm; the
  axis keeps running and motion stays allowed), Emergency stop, Modbus control, Safety speed, Paused, Homing, Target
  reached, Pushing, Push contact, Push missed, and Backward end / Middle / Forward end from ZONS 0x9013 b12 / b14 /
  b13, with the raw DSS1, DSSE, ZONS, DOPM and ALMC below them.
- **Modbus control.** The first command takes it, only at rest; it is handed back to PIO on disconnect if it was off at
  connect. With PIO in control (PMSS = 0) the STOP button still sends the Modbus STOP, which stopped a
  Modbus-commanded move on hardware; it is **untested on a move a PLC commanded over PIO**, so use the PLC or the
  hardware stop for those.
- **STOP** (button, Space or Esc) bypasses queued commands and uses the driver's STOP sequence. It is a Modbus
  decelerate-stop, not a safety function. If the axis does not report idle within 3 s, "STOP NOT CONFIRMED" is shown.
  With PIO in control the axis is watched for 5 s after the standstill, and "AXIS MOVED AGAIN AFTER STOP" is shown if
  it moves (e.g. a PLC still holding its start input). STOP waits for at most one short request of a running Unit
  info / register read, which it then ends.
- **Push result.** Contact / missed is shown only once the flags have held for 0.15 s with the position unchanged
  (the driver's rule; VERIFIED).
- **Run stored** (Move panel) runs rows 0-2 with 0x9800 at the row's own speed / accel (`POST /api/run_stored`). It
  asks first, showing the stored speed, accel / decel and push; a row above the session's caps (100 mm/s, 0.3 G) needs
  a second confirmation.
- **Configuration** (Controls panel, only at rest, each behind a confirm dialog). *Software reset*
  (`POST /api/software_reset`) then shows PMSS / SFTY / servo / HEND and "NEEDS HOME" until a home return. The *baud
  selector* (`POST /api/baud`; volatile, every unit on the line switches) offers only 115200 and 230400: 38400 is left
  out because the EC-R6S rod ignored its code (0x06), though the EC-GRBP8M gripper and the PCON-CB take it (the
  command line sends it: `baud 38400`); power-cycle the unit to get back to 38400. STOP still goes out within 0.25 s
  while a software reset holds the bus (the restarting unit may not answer it; the reset has already switched the
  servo OFF).
- **Unit info** tab: everything `Elecylinder.unit_info()` returns, in collapsible panels, read-only except the
  parameter *Edit* buttons.
  - *Edit* (`params.js`, `POST /api/param`) asks for the value, then confirms a **permanent** write (committed with
    PARW) that applies only after a software reset, with one plain OK / Cancel; P3 / P4 show the home warning in that
    confirm. Parameters not listed for the unit, not located (P8) or read-only are marked before any click. A commit the
    unit was never seen to act on is reported as NOT confirmed.
  - A raw register browser reads any directory block (hex / decimal / signed / ASCII, paged, at most 512 words per
    read; a block of uncertain size up to its largest possible extent, the uncertain words in italics). Unconfirmed
    names show a `?` and a dashed outline; hover for the evidence.
  - *Refresh* re-reads everything (about 110 FC03 / FC11 requests of at most 32 words), taking the bus lock per
    request, so the status poll, commands and STOP get the bus between any two of them; STOP ends the read. The I/O,
    status-word and monitor panels follow the ~5 Hz status poll. *Save JSON* saves the last read (the
    `iai-actuator-driver info --json` schema).
  - Endpoints: `GET /api/unit_info`, `GET /api/unit_live`, `GET /api/read_block?class=0x0700[&offset=N][&count=N]`;
    they only read. `unitinfo.js` sends no request body; the Edit buttons come from `params.js`.

## Simulator

`SimulatedEC` implements the same transport interface as the serial port and answers Modbus RTU byte for byte
(FC03/05/06/10/11/17/45/48, exception 02 for unknown addresses, silence on CRC errors, injectable timeouts / CRC
errors). Passing against the simulator shows the driver is consistent with what was observed, not that every unit
behaves the same way.

- **Profiles.** `SimulatedEC(profile="slider")` (default), `"gripper"`, `"rod"` and `"pcon"` (the PCON-CB + RCP6,
  starting with its 0x069 RTC message alarm) are seeded with the live values of the units above. `"rotary"` (AKID 1,
  degrees) and `"unknown"` (an unnamed unit code) are made-up units for the generic paths. Every profile runs the
  command-profile move (0x0F00 + PCRF, separate decel), the 0x45 cmd 6 speed override and the PCON-style reset.
- **Lines.** `SimLine` / `sim_line()` put several simulated units on one line (every frame reaches every unit; address
  0 reaches all and nobody answers), and `sim_ports(n)` gives n simulated ports, so a `Cell` can be tested without
  hardware.
- **Data.** The EC profiles carry status, index table, soft limits, 0x0800 parameters incl. the 0x0812 speed, alarm
  history, identification with a placeholder serial, position rows and DOPM layout. For `iai-actuator-driver info`
  each profile also serves its unit's real directory, item-ID maps, parameter classes 3 / 4 / 7 / 8 / 9 / 0x0C,
  maintenance block (whose move counter counts the simulator's moves), alarm history and the % rows 260-263
  (`bindings/python/iai_actuator_driver/sim_fixtures.py`, copied from the live reads; serial numbers are
  placeholders). Reads inside a directory block it has no data for answer 0; reads outside every block answer
  exception 02, so tests catch any read that strays past a block.
- **Motion** follows a real-time trapezoidal profile (`time_scale` speeds it up for tests) and mimics what the
  hardware showed: MOVE clearing during deceleration, STOP latching until released (configurable), 0x9900 moves and
  STOP working with PMSS = 0, PMSL ON stopping a move, STP as a level input, the home sequence with its creep phase,
  push approach / contact (`obstacle_mm`) / miss with the verified direction bit, stale CTLF bits, PEND / MEND rising
  inside the band before standstill with a slow settling tail (`early_in_position`, `settle_tail_mm`,
  `settle_speed_mm_s`), the transient contact fingerprint of a push that misses (`push_blip_mm`), and the ZONS / DOPM
  markers following the position (within `ls_range_mm` = 0.10 mm of each row or a push row's push end, moving or not).
- **PLC start input.** `pio_move(..., hold_start=True)` stands in for a PLC-commanded move whose start input stays on
  (a worst-case model that restarts the move when STOP is released; untested on an EC).
- **Configuration.** Parameter writes land in RAM and the PARW coil commits them (`parameter_nv`; `power_cycle()`
  keeps committed values and drops the rest). DRGE b7 reads 1 for `parw_busy_s` (0.1 s real time) before the commit
  completes, `parw_dsse_busy=True` also drops DSSE b7 meanwhile, and a restart inside that window loses the commit
  (worst case).
- **Reset and 0x45.** 0x45 cmd 5 op1 1 resets like a power-up except the uptime (silent for `reset_s`; an incremental
  profile such as the rod loses HEND, an absolute one raises 0x0EE when its committed P3 / P4 changed; with the servo
  on it raises 0x090 instead). The 0x45 forms behave as verified on the EC-R6S (home rejected 0x11 with op1 = 0,
  accepted with op1 = ORG; alarm reset op1 0; action mode 0x0C switches PMSS / SFTY, also from PMSS = 0).
- **Alarms and support words.** Alarms follow the levels described above (a message-level code sets ALML and the axis
  keeps running; anything else sets ALMH and switches the servo off). Every profile serves class-1 support words (the
  rod's partly assumed, see `sim.py`), MAXV follows `max_speed_mm_s`, and `collision_detection=True` sets MVPT b13
  CDAS.
- **Serial line.** `baudrate=` models the line: only the matching rate is answered; a `send_break` of 150 ms or more
  drops the unit to 9600 (VERIFIED on the EC-GRBP8M gripper and the PCON-CB); the broadcast baud change switches it
  except for code 0x06, which it ignores like the EC-R6S (`ignored_baud_codes`); `power_cycle()` brings it back to
  38400. The rod answers FC11 only after `slave_id_delay_s` (0.3 s).
