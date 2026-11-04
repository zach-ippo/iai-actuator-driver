# Compatibility

Which IAI products the driver works with, what has run on hardware so far, and what can go wrong. Every IAI unit
that speaks Modbus RTU on its serial port is supported and driven the same way: the driver reads ranges and
capabilities from the unit and keeps no list of models, so a new model needs no driver update.

The evidence words **VERIFIED**, **ON-WIRE**, **DOC** and **UNVERIFIED** describe features and paths, never a unit or a
model ([Evidence levels](REFERENCE.md#evidence-levels)).

Unofficial: IAI does not support this driver. See the [README](../README.md).

## At a glance

**Supported:**

- ELECYLINDER (EC) 24 V and EC 200 V, every type: slider, dust/drip-proof slider, rod, gripper, rotary, wire,
  2-position and 3-position models.
- CON-family controllers with their actuator: PCON-CB (for example with RCP6), ACON-CB, DCON-CB, SCON-CB.
- RCON driver units through the RCON gateway.
- ROBONET in SIO through mode (SW2 ON).

**Not supported:** SEL-protocol controllers, an ELECYLINDER on an RCON-EC / REC connection unit and ROBONET in
gateway mode ([Not supported](#not-supported)).

Connection in every run so far: one unit per COM port, through an IAI USB adapter on the unit's RS-485
(SIO / teaching) port, slave address 1.

## Run on hardware so far

A record of the units this driver has run on and what ran, not a list of allowed units. `iai-check` passed on the
first three.

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
| Python package | EC-GRBP8M gripper | identity, unit info, status, limits, servo, moves, jog, refusals, fast mode and monitor events, move handles (polled and cancelled), stop, pause / resume, speed change, deceleration, acceleration pattern, a missed push, position table and parameters (read), the non-volatile guard, backup, baud detection, track and link check, recovery after a 250 ms line break, the PC/PIO toggle, the self-test ([Hardware results](#hardware-results)) |
| Python `iai_actuator_driver.native`; the .NET core binding | PCON-CB with RCP6-WSA10R | open with baud detection, identify, status, limits, servo, moves, STOP mid-move, pause / resume, a same-value parameter write skipped, baud 230400 and back to 38400 |
| .NET `Axis` on the core (2026-10-30) | EC-GRBP8M gripper | open and identify, unit info, status, limits, servo off and on, absolute and relative moves and jogs, the speed / stroke / acceleration refusals, a move polled by its status and one cancelled with a `CancellationToken`, stop, pause / resume, its own deceleration, a missed push, position table and parameters (read), the non-volatile guard, baud detection, the PC/PIO toggle; the unit ended as found |
| C++ command line (`iai-actuator-driver-cpp`) | EC-GRBP8M gripper | ports, scan, baud detection, status, identify, limits, alarm history, servo, home, moves and jogs (exact to 0.01 mm), STOP mid-move, pause / resume / cancel-pause, a same-value parameter write skipped, baud 230400 and back to 38400, software reset. Re-check of 2026-10-30, command by command: unit info, `--fast` with `monitor --watch` events, a speed change during a move, its own deceleration, an `s_motion` acceleration pattern, a missed push, position table and parameters (read), backup, track and link check, the PC/PIO toggle, the self-test; the unit ended as found |
| C++ command line | PCON-CB with RCP6-WSA10R | status, identify, limits, alarm reset, servo, moves and jogs, STOP mid-move, pause / resume, a same-value parameter write skipped, software reset |
| .NET command line (`iai-actuator-driver-cs`) | EC-GRBP8M gripper | ports, status, identify, limits, alarm history, servo on, home, moves and jogs, STOP mid-move, parameter list and get, a same-value parameter write skipped, baud 230400, software reset; in the harness also the monitor, the backup, `recover` with the link up and the self-test ([Hardware results](#hardware-results)) |
| .NET command line | PCON-CB with RCP6-WSA10R | status, identify, servo on, moves and jogs, STOP, parameter get and a same-value write |
| GUI (`iai-gui.exe`) | EC-GRBP8M gripper (2026-10-31); PCON-CB with RCP6-WSA10R (2026-11-03) | through its single-axis HTTP interface: connect (with a stroke override on the gripper; the stroke band enforced on the PCON-CB), status, servo on, a move out and back, the PC/PIO switch (PC, PIO, PC; on the PCON-CB the take-back from a pause with the servo off; a move sent in PIO was refused and nothing was sent), disconnect, quit. A request without the access key, or with a foreign Host header, got 403. Everything else in the GUI, the workbench page and its Coordinated tab included: UNVERIFIED |
| C program (the SDK's `examples/c/first_move.c`) | PCON-CB with RCP6-WSA10R | 2026-11-03, on Windows and on Linux: open, identify, status, servo on, a move to the middle and moves by +10 and -10 mm at 10 mm/s, servo off |
| C ABI | through the Python, .NET and C program rows above | the steps of those rows; the calls none of these runs used are tested on the simulator only |
| LabVIEW, `Cell` on several units, RCON / ROBONET gateways, macOS serial ports | none | nothing: UNVERIFIED |

Linux serial ports are VERIFIED: the PCON-CB through the IAI USB adapter (`cp210x`, `/dev/ttyUSB0`) in WSL 2 with
usbipd, from the Python package, the C++ command line and the C program (2026-11-03). The .NET editions and a Linux
PC without WSL were not run.

In the .NET edition, fast mode, the monitor's events, a change of speed, move handles, `CheckLink` and recovery after a
line break are UNVERIFIED (tested on the simulator).

## Hardware results

Acceptance runs with the harness in [tools/hw](../tools/hw/README.md): the same checks on any unit, a read-only backup
first, short moves mid-stroke at a capped speed (at most 20 mm/s and 0.1 G), then the unit is put back as found and
compared with the backup.

- **PASS**: the step behaved as the harness expects (VERIFIED on this unit, for that edition).
- **Arrival only**: the command ran and the move arrived, but the behaviour named in the step was not measured. It
  stays UNVERIFIED for that edition.
- **Not run**: not tried on hardware. It stays UNVERIFIED for that edition.

| Unit | Last full run | Result |
|---|---|---|
| EC-GRBP8M-20-1-1-PN-WA-WL2 gripper, firmware 0002, 38400 baud | 2026-10-31, Windows: the Python package, the C++ and C# command lines, `iai-gui.exe` | Python 29 PASS, 1 not run (home: the unit was homed); C++ command line 19 PASS, 2 arrival only, 1 not run; C# command line 18 PASS, 2 arrival only, 2 not run; GUI 13 PASS. End state as found: the 102 parameter items, the 768 position rows and the alarm history matched the backup |
| PCON-CB with RCP6-WSA10R, 38400 baud | 2026-11-03, Windows and Linux (WSL 2): the Python package, the C++ command line and the SDK's C example; on Windows also the C# command line and `iai-gui.exe` | Python 28 PASS, 1 CHECK (the self-test warns about the alarm history and the position table), 1 not run (home); C++ command line 21 PASS, 1 not run; C# command line 20 PASS, 2 not run (home; it has no `--accel-pattern` option); GUI 14 PASS; the C example exit 0. The same counts on Linux. End state as found (133 items, 768 rows, the alarm history), no non-volatile write |

The arrival-only steps are the command lines' `move --fast` and `move --change-speed`: they check that the move
arrives, not the fast path or the new speed. The command lines run one command per step, so they have no servo,
refusal, handle, pause, guard, baud or line-break step. The numbers measured in these runs (STOP timing, `recover()`
after a line break, moves at up to 840 mm/s) are in [REFERENCE.md](REFERENCE.md#measured-on-hardware); coordinated
motion with one real axis is in [REFERENCE.md](REFERENCE.md#coordinated-motion-on-hardware).

### Checked by hand

The Python package on the EC-GRBP8M gripper, with the harness's `power-cycle` and `pio-watch` commands run by hand
(2026-10-30 and 2026-10-31, the same outcome both days).

| Check | Result | Evidence |
|---|---|---|
| Live power cycle: 24 V switched off by hand for 3.2 s while the driver ran | `check_link()` reported the link lost (unit silent), then a restarted unit (uptime counter reset, PC control back to PIO). `recover()` took PC control back in 1.44 s; servo on, home kept, back at 3.00 mm; the 3 rows and 102 parameter items unchanged; nothing written to non-volatile memory. | VERIFIED |
| PIO inputs: switches wired by hand to the PIO connector, the unit in PIO mode, watched read-only | ST0 (input bit 0) sent the gripper to its backward end (row 0, 3 mm) and ST1 (bit 1) to its forward end (row 1, a push to 9.5 mm); 2353 status reads with no read error; PC mode taken back at the first attempt. RES (bit 2) is not wired on this bench. | VERIFIED (ST0, ST1); UNVERIFIED (RES) |
| STOP in PIO mode: the driver's STOP sent during a PIO move | The unit accepted it and stopped (1.8 mm after the move was seen, 0.35 s after the call); sent as ST1 started the push row 1, it came to rest at the row's approach position (4.11 mm) with MEND 0 while ST1 was still held. Whether a start input held through the STOP restarts the move is not shown. | VERIFIED (the STOP accepted, the unit stopped); UNVERIFIED (a restart by a held start input) |

### Not run on hardware

On the gripper the following stay UNVERIFIED:

- the RES input of the PIO connector (not wired on the bench);
- coordinated motion with two real axes, and coordinated motion under CPU load;
- a PLC driving the PIO connector; the power cycle through the C++ and C# command lines; the PIO checks through the
  command lines;
- home from a unit that is not homed; writes to parameters and the position table;
- a push that meets a workpiece; a vertical or loaded axis;
- every item the tables above do not list.

A report from your unit moves a path to VERIFIED: see [Sending a report](#sending-a-report).

## Notes per product line

Every supported line uses the same protocol and register layout and reports its own limits.

- **ELECYLINDER types.** A rotary axis is driven in degrees (the unit reports its kind). Wire types and 3-position
  models are driven like any slider or rod. A unit code the driver has a family name for is shown with that name,
  any other code with its product family and codes.
- **EC 200 V.** Identified from its identity code; same commands.
- **ACON-CB, DCON-CB, SCON-CB.** CON-family controllers with the Modbus RTU serial port IAI documents for the family
  (DOC); the driver drives them as it drives the PCON-CB, which behaves like an EC.
- **RCON driver units through the RCON gateway.** One gateway port carries several axes (address = axis number + 1).
  The driver applies the gateway's documented rules and refuses anything the line cannot carry before sending it
  (DOC, UNVERIFIED on hardware; the rules are in [HARDWARE_SETUP.md](HARDWARE_SETUP.md#several-axes-on-one-port)).
- **ROBONET in SIO through mode (SW2 ON).** State it with `--line robonet` (`line="robonet"` in Python): through mode
  cannot be detected.
- **Several axes on one port** (daisy chain or gateway) is UNVERIFIED.

## Not supported

| Product | Why |
|---|---|
| XSEL, SSEL, ASEL, PSEL, MSEL controllers and the TTA table-top robots | They use IAI's SEL protocol on their serial port, not Modbus RTU. They never answer a Modbus request. |
| ELECYLINDER connected through an RCON-EC or REC connection unit | The unit's own serial port is disabled there. It shows up as linked at the gateway but is silent on Modbus. |
| ROBONET in gateway mode (SW2 OFF) | The gateway gives no axis access over Modbus. The driver raises `UnsupportedLineError`; set SW2 ON (through mode). |

All three are DOC (public IAI manuals). When a port gets no answer at all, the driver, the command line and the GUI
say so and name these cases.

## Possible trouble

The symptoms and their fixes are in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

- **Nothing answers.** Check the 24 V supply, the cable, the slave address (axis number + 1) and that no other
  program has the port; `iai-actuator-driver ports` and `iai-actuator-driver scan` show what answers.
- **The baud rate was changed, or a line break was sent.** A changed rate stays until a power cycle, and a line break
  of 150 ms or more drops a controller to 9600 (DOC; VERIFIED on the EC-GRBP8M gripper and the PCON-CB). The driver
  finds the rate at connect: [HARDWARE_SETUP.md](HARDWARE_SETUP.md#baud-rate).
- **Something else holds the link.** Close IA-OS and any other program on the port. A unit in IA-OS's Monitor mode
  takes no moves from another program (VERIFIED); a teaching pendant or a wireless session is expected to take control
  the same way (UNVERIFIED). After IA-OS the unit may run at 115200 baud (ON-WIRE). See
  [HARDWARE_SETUP.md](HARDWARE_SETUP.md#close-ia-os-and-other-masters).
- **Old firmware without the register directory** (the unit's own layout at 0x0000): the unit is read at the standard
  addresses and the identity notes say `index table: ...`. When its stroke or limits cannot be read, moves are refused
  until you state the stroke (`--stroke MIN MAX`), and speed, acceleration and push fall back to 100 mm/s, 0.3 G and
  70 %. Send an `iai-check` result ([Sending a report](#sending-a-report)).
- **CCM conveyor control motor.** Recognised and named "CCM conveyor control motor (not an EC actuator)". It is not a
  positioning actuator with a stroke, so the mm moves and stroke limits of an axis do not describe it.
- **Incremental units lose their home** at every software reset and power-up: run `home` before the next move.
  Absolute (battery-less, `WA`) units keep it, except after a change of the home direction or offset.

**Vertical axes with a brake.** The driver cannot know how an axis is mounted. The speed and acceleration limits the
unit reports are horizontal, light-load values; a vertical mounting or a payload lowers the real limits, so stay well
below them. Servo OFF, a servo-off alarm, an emergency stop or a software reset releases position control: a vertical
axis without a brake falls, and with a brake it is held only as long as the brake holds. The driver never releases
the brake by hand. Test vertical axes unloaded and slow, with the hardware emergency stop within reach.

## Sending a report

Reports go to [GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose), in one form. Under
**What is this?** choose **A result from my unit** for "my unit works" or "my unit does not work", and **A bug** for
anything else.

`iai-check --port COM3` (read-only) writes `iai-check-<timestamp>.json` with the model, firmware, limits and every
raw value read. `iai-check --port COM3 --motion` adds a short move of at most 5 mm at 10 mm/s and back. Attach that
file to the issue, for any unit. It holds the unit's serial number and the port name, and issues are public.
