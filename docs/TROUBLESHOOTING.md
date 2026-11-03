# Troubleshooting

Find the symptom or message, then the likely cause and what to do. Messages are quoted as the driver prints them,
shortened with `...`, so you can search this page for a few words of yours.

Unofficial: not an IAI product ([README](../README.md)). The driver is not a safety function: keep the hardware
emergency stop within reach while you troubleshoot, and test with the axis unloaded and slow.

Evidence words: **VERIFIED** (tried on hardware), **ON-WIRE** (seen in serial traffic), **DOC** (public IAI manual),
**UNVERIFIED** (expected, not yet tried). They describe features and paths, never a unit: every unit is driven the
same way.

## How errors look

- **Python:** every driver error derives from `EcDriverError` (`iai_actuator_driver.errors`). `CellError` is in
  `iai_actuator_driver.cell`, the gateway errors in `iai_actuator_driver.gateway`, `RecoveryError` and
  `UnitReplacedError` in `iai_actuator_driver.recovery`. Hints are notes that start with `iai_actuator_driver:`
  (`exc.__notes__`).
- **C#:** the same classes with C# names (`EcDriverException`, `SoftLimitException` ...), with `Message` and `Notes`
  ([csharp.md](getting-started/csharp.md#errors)).
- **C++:** `EcDriverError` with `what()` and `notes()` ([cpp.md](getting-started/cpp.md#errors)).
- **C:** a negative `iad_result` (`iad_result_name()`); read `iad_last_error_message()` on the same thread right after
  the call ([c.md](getting-started/c.md#errors)).
- **Command line:** `error: <ErrorClass>: <message>`, then each note indented. Exit code 2 on an error, 1 when
  `iai-check` fails or `scan` finds nothing, 130 after Ctrl+C.
- **GUI:** `<ErrorClass>: <message> (<notes>)`.

When a message says that nothing was sent or written, the unit was not touched.

## First checks

None of these writes to the unit.

```shell
iai-actuator-driver ports                              # serial ports, IAI USB adapters first (opens nothing)
iai-actuator-driver scan                               # which addresses answer on each IAI adapter, and at which baud rate
iai-actuator-driver --port COM3 status                 # position, alarm code and flags (SERVO, HOMED, ALARM, EMG-INPUT ...)
iai-actuator-driver --port COM3 info --section alarms  # the current alarm with its remedy, and the history
iai-actuator-driver --port COM3 limits                 # every range the driver enforces, read from the unit
iai-check --port COM3                                  # self-test; writes iai-check-<timestamp>.json
iai-actuator-driver --port COM3 --log-frames status    # the same status, with every Modbus frame on stderr
```

## Error classes

| Error | Meaning | Section |
|---|---|---|
| `ModbusTimeout`, `ModbusCRCError`, `ModbusFrameError` | No answer, a garbled answer, or an answer that does not fit, after every retry | [No answer](#no-answer-and-timeouts) |
| `ModbusException`, `IaiCommandRejected` | The unit answered and refused (never retried) | [No answer](#no-answer-and-timeouts) |
| `TransportError`, `RecoveryError`, `UnitReplacedError` | The port failed (USB adapter pulled), or `recover()` failed or found another unit | [Power loss](#power-loss-and-a-pulled-cable) |
| `ServoBlockedError` | The servo cannot come on; `.reasons` says why | [Servo](#servo-will-not-turn-on) |
| `EcNotReadyError` | A precondition is missing: servo off, not homed, paused, PC control | The section that matches the message |
| `SoftLimitError` | A value is outside the unit's range or a cap | [Limit checks](#moves-refused-by-the-limit-checks) |
| `EcAlarmError` | The unit reports an alarm (`.alarm_code`) | [Alarms](#alarms) |
| `MotionTimeoutError`, `MotionIncompleteError` | A move, home or STOP did not finish, or settled away from the target; a decel stop was sent | [Stops and pauses](#stops-and-pauses) |
| `LineRuleError`, `GatewayModeError`, `PositionUnitError`, `UnsupportedLineError`, `CellError` | The line or gateway does not carry the command, or one axis of a group failed | [Several axes](#several-axes-on-one-port-and-gateways) |
| `NonVolatileWriteError`, `UnsafeCommandError`, `VerificationError` | A row or parameter that cannot be written (rows 256-267, read-only), or the read-back did not match | [Non-volatile writes](#non-volatile-writes) |

## Native library

The C++ core is a shared library (`iai_actuator_driver.dll`, `libiai_actuator_driver.so`). Every language needs it,
the Python package too. Nothing here touches the unit.

| Symptom | Likely cause | What to do |
|---|---|---|
| `ImportError: iai_actuator_driver needs its native library, which is not usable: ...` | The library was not found (the message lists where it looked) or has another ABI version. A plain `pip install` of the folder copies only the Python files. | Install a platform wheel, set `IAI_ACTUATOR_DRIVER_NATIVE` to the library file, or build it (`python build.py shared` in `core/`). |
| `ImportError: IAI_DRIVER_BACKEND=python is no longer a choice` | The pure-Python fallback was removed after 1.0.x. | Unset `IAI_DRIVER_BACKEND` and provide the library ([PYTHON_FALLBACK.md](PYTHON_FALLBACK.md)). |
| `NativeLibraryError: cannot load the native library ...` (32-bit / 64-bit) | The library is for another CPU or bitness. | Use the library for this platform (`win-x64`, `linux-x64`, `linux-arm64`). Windows has x64 only. |
| C#: `DllNotFoundException` naming the runtime identifier | The library is not next to the application, or under `runtimes/<rid>/native/` in the package. | Copy or build it. `IAI_ACTUATOR_DRIVER_NATIVE` names one file to load. |
| C#: `NotSupportedException` at the first call; C: `iad_abi_version()` differs from the header | Library and binding or header come from different releases. The ABI major must match (1) and the library's minor be at least the caller's (`IAD_ABI_COMPATIBLE`). | Use the library, header and binding of one release. |
| C#: `BadImageFormatException`, or `PlatformNotSupportedException` | A 32-bit process. Only the 64-bit library is built. | Run the program as 64-bit. |
| C#: the build stops with `The native core ... is missing` | The project copies the library from `core/build/` and it is not there. | Build it, pass `-p:NativeLibraryPath=<file>`, or `-p:IadNativeOptional=true` to build without it (the program then fails at run time). |

## Connection and ports

| Symptom | Likely cause | What to do |
|---|---|---|
| `is not recognized` when you type `iai-actuator-driver`, `iai-check` or `iai-gui` | Not installed, or pip's Scripts folder is not on the PATH. | `pip install ./bindings/python ./apps/gui` in the project folder. Or use `python -m iai_actuator_driver ...` (`... --port COM3 check` for `iai-check`) and, from a checkout, `python apps/gui/server.py`. |
| `error: one of the arguments --port --sim is required` | The command needs a port. | Add `--port COM3` before the command. `ports`, `scan` and `stop-all` need none. |
| `error: unrecognized arguments: --baud 9600` (also `--axis`, `--unit-max`, `--line` ...) | Connection options came after the command. | Put them before it: `iai-actuator-driver --port COM3 --axis 2 status`. |
| `could not open port 'COM3'` with the note `COM3 is in use by another program ...`; Windows adds `Access is denied` | Only one program can own a serial port. | Close IA-OS or disconnect it from the port. Stop the other `iai-actuator-driver` or `iai-gui`, and close terminal programs. A port connected in `iai-gui` stays busy until you disconnect it there. |
| `could not open port 'COM7'` with the note `COM7 does not exist or the USB adapter is unplugged` | Wrong port name, or the adapter is unplugged. | `iai-actuator-driver ports` lists the ports. The IAI adapter shows as "IAI USB to UART Bridge Controller" (USB ID 10C4:81D7, VERIFIED) and is listed first. |
| `ports` prints `no serial port found` | No adapter plugged in, or pyserial is missing. | Plug the adapter in. `pip install ./bindings/python ./apps/gui` installs pyserial. |
| `no IAI USB adapter found; name the ports with --ports COM3,COM4`, or `no IAI USB adapter found (list_ports)` from `Cell.open()` | Ports are picked automatically only by the IAI adapter's USB ID, or a maker name that starts with "IAI". | Name the port: `scan --ports COM5`, `Cell.open(["COM5"])`. Other RS-485 adapters are UNVERIFIED. |
| `PMSL was written but DSSE.PMSS is still 0; the EC may be in Monitor mode (set Teach mode)` | IA-OS left the unit in Monitor mode, where it takes no moves from another program (VERIFIED). A teaching pendant or a wireless session is expected to do the same (UNVERIFIED). With `The unit's mode switch is in AUTO`, the PIO has control. | Set Teach mode in IA-OS before you disconnect it, or power-cycle the unit: the mode is gone after a power cycle (VERIFIED). Disconnect pendants and wireless sessions. Set a mode switch to MANU. |
| `EcNotReadyError: the axis is moving: switching Modbus control (PMSL) on now would stop the move` | A PLC or another program is moving the axis. Switching PC control on mid-move stops the move (VERIFIED), so the driver takes it only at rest. The same message comes while a pause (DSS1 STP) is held with the servo on, even with the axis standing still: a PLC holding a stop input on a move. | Wait until the axis stands still, or stop the move from the PLC. For a held pause, release the stop input from the PLC: waiting does not clear it. A pause with the servo off at a standstill (a PCON with nothing on its PIO connector) is no refusal from 1.0.1 on. Reading works meanwhile. |
| The PLC's PIO commands are ignored after your PC program ended | PC control (PMSS = 1) stays on after the port is closed, and the unit then ignores its PIO inputs. | Hand control back before you close: `ax.disable_modbus_control()` or `iai-actuator-driver --port COM3 mode pio`. A power cycle also does it (PMSS is 0 after every power-up, VERIFIED). |

## Baud rate

Every tested unit answered at 38400 baud after power-up (VERIFIED). Without `--baud`, the driver finds the rate on the
open port, trying 115200, 38400, 9600, 230400, 57600 and 19200, and the command line prints
`note: COM3 at 38400 baud (auto-detected)`. `--baud N` (`baudrate=N`) skips the search.

| Symptom | Likely cause | What to do |
|---|---|---|
| `slave 1 did not answer at any of 115200, 38400, 9600, 230400, 57600, 19200 baud` | Usually nothing answers at all ([No answer](#no-answer-and-timeouts)). Less often the unit is at 14400, 28800 or 76800, which are never probed. | Power-cycle the unit (it comes back at 38400), or connect with `--baud RATE`. |
| The note shows 115200 or 230400 | IA-OS or an earlier `baud` command switched the unit, until the next power-up (VERIFIED). | Nothing: the driver found the rate. |
| `VerificationError: baud change to ... not taken: the EC still answers at ... baud` | The unit ignored the code. The EC-R6S rod ignores 38400; the EC-GRBP8M gripper and the PCON-CB take it (VERIFIED). | The port stays at the rate that answers. Use 115200 or 230400, or power-cycle to get back to 38400. |
| Every unit on the line changed its rate after one `baud` command | The baud change is a broadcast to the whole RS-485 line. | On a shared line use `Port.set_baudrate()`: it needs every axis at rest and checks that each one answers. |
| The unit answers at 9600 although nobody set that rate | A line break of 150 ms or more drops a controller to 9600 (DOC; VERIFIED on the EC-GRBP8M gripper and the PCON-CB). Opening and closing the port in a loop can also change the rate. | The driver finds the rate either way. Keep one `Axis`, `Port` or `Cell` open for the session. `recover()` puts the unit back on the session's rate (VERIFIED on both units after a line break). |
| `set_baudrate (broadcast 0x45 cmd 4): rcon does not carry broadcast frames` | Nothing is broadcast behind an RCON gateway; the RCON line runs at 115200 (DOC ME0413 p.2-1). | Leave the rate as it is. |

## No answer and timeouts

| Symptom | Likely cause | What to do |
|---|---|---|
| `ModbusTimeout: slave 1 did not answer at any of ... baud` with the note `check the 24 V supply, the cable, the slave address ...`; `scan` prints `no axis answered`; `iai-check` fails `link` | No 24 V, the cable is not on the SIO / teaching port, a wrong address, or another program holds the unit. Some products never answer: SEL-protocol controllers (XSEL, SSEL, ASEL, PSEL, MSEL, TTA) and an ELECYLINDER on an RCON-EC / REC connection unit (DOC). | Check the supply and the cable, then `iai-actuator-driver scan` (addresses 1 to 16 at every rate). `--log-frames` shows whether anything comes back. See [COMPATIBILITY.md](COMPATIBILITY.md#not-supported). |
| The unit does not answer at the address you used | Address = axis number + 1. Every tested unit answered at address 1 (axis number 0). | `scan` prints `axis N` for each address that answers; use `--axis N` (or `--slave N+1`). |
| `ModbusTimeout: timeout: got 0 bytes, wanted 7` with `gave up after 4 attempts`, after the unit had answered | The unit stopped answering: 24 V lost, cable pulled, or it restarted at another rate. | `status` shows whether it is back. See [Power loss](#power-loss-and-a-pulled-cable). |
| `timeout: got 3 bytes, wanted 7`, or `ModbusCRCError: bad CRC in response ...` | A cut or garbled answer: wiring, noise, a wrong baud rate, or two units with the same axis number. | Check the wiring and the baud rate; `--log-frames` shows the frames. |
| `--log-frames` shows the same request twice | A retry after a lost answer. Timeouts and CRC errors are retried 3 times, IAI's recommended count (DOC ME0162-12A p.4-2). | Normal. An error is raised only when every attempt failed. |
| `EcNotReadyError: ... a STOP went first ...; not re-sent`, `... a STOP arrived meanwhile ...; nothing sent` or `a STOP is held on this axis (a stop in progress in another thread)` | A STOP (from another thread, or a broadcast STOP) reached the unit while this command was being prepared, waited for the port or was retried. The STOP wins: no frame of the command goes out after it, and no command releases a STOP that a stop still holds. | Normal after a STOP. Send the command again once the stop has ended. |
| `ModbusFrameError: response from slave 2 while talking to slave 1` | A late answer to an earlier request, or a second master on the line. | Make sure only one program drives the line. |
| `identify` prints `identity from CTCL 0x0300 (FC11 not answered)`; `iai-check` warns `Report Slave ID (FC11) not answered` | The unit answered FC11 late. The EC-R6S rod takes 0.2 to 0.4 s (VERIFIED); the driver waits 0.65 s, then reads the identity from CTCL. | Nothing: the identity is complete. Send the `iai-check` report if it happens on another model. |
| `ModbusException: Modbus exception 02h (Illegal Data Address) for function 03h` | The unit refused the request. Exception answers are never retried. | For your own `read_registers()` calls, read inside the unit's directory (`identify` lists it). If a driver command causes it, send the `iai-check` report. |
| `Modbus exception 01h (Illegal Function) for function 05h`, with `this unit has no FC05 coils ...` | The unit or line has no FC05 coils, so the PC-control coil cannot be written. | Put the unit in MANU (Modbus) mode by other means, such as its mode switch. The driver then sends servo, home, alarm reset, STOP and pause over registers (FC06); `--strategy register` does so from the start. |
| `IaiCommandRejected: IAI command 0x0a (function 45h) rejected with code 0x11 (1st operand error): ...` | The unit refused an IAI 0x45 command; the text after the colon is the driver's hint. Only the name of code 0x11 is VERIFIED; the other code names are UNVERIFIED. | Follow the hint. 0x45 is used by `swreset`, `baud`, `mode`, `goto --speed-pct` and the `--method iai` forms. |
| `the EC did not answer within 10 s after the software reset (the servo is off)` | The unit did not come back after `swreset`. | Check the 24 V supply, then connect again. |

## Power loss and a pulled cable

The unit has no communication watchdog (VERIFIED): if the PC, the program or the cable fails, a move in progress is
completed and a push keeps pushing. Only the hardware emergency stop stops the axis independently of the PC.
`recover()` (`iai-actuator-driver --port COM3 recover`) brings an axis back after a fault and never moves it.

| Symptom | Likely cause | What to do |
|---|---|---|
| `TransportError: transport error: ...`; link state `port_vanished`; GUI `the port vanished (USB adapter unplugged?)` | The USB adapter was pulled or lost power (VERIFIED). | Plug it back in, then `recover`: it waits for the port, reopens it and re-syncs the unit. |
| Link state `unit_silent`; GUI `the unit does not answer ...` | The port is there, but the unit does not answer. | Check the 24 V supply and the cable, then `recover`. |
| Link state `unit_restarted`; GUI `the unit restarted (power loss) ...` | The unit lost power. It comes back at 38400 baud with PC control off, its uptime counter restarted and a 0x0FFF power-up record in its alarm history (a 3.2 s outage on the gripper wrote none). An incremental unit has lost its home; a PCON-CB raises 0x069 (all VERIFIED). A move cut off by the loss is not resumed. | `iai-actuator-driver --port COM3 recover --backup unit1.json` (or `ax.recover()`) checks that it is the same unit, restores baud rate, PC control and safety speed, and reports `needs_home`, the alarm and the position. Home if it says so. VERIFIED on the EC-GRBP8M gripper after a power cycle, and on the PCON-CB after a line break. |
| The recovery report says `the axis is still moving (at ...): a move commanded before the fault runs on` | The link failed but the unit did not: it completes its move. | Wait for it or stop it before the next command. `recover()` neither stops nor resumes a move. |
| `UnitReplacedError: another unit answers at slave 1 on COM3: ... Nothing was written to it` | The serial number or model differs from the tracked unit. | Check the machine. Then `track()` the new unit and back it up. |
| `RecoveryError: port COM3 did not come back within 30 s ...` | The port or the unit did not come back in time. | Plug in the adapter or power the unit, then `recover` again; `--wait S` (`timeout=`) waits longer. |

## Servo will not turn on

`status` prints the condition flags. `servo on` is refused before anything is sent when the status shows why
(`ServoBlockedError.reasons`). The flags are DOC (ME0162-12A); their meaning on an ELECYLINDER is UNVERIFIED.

| Symptom | Likely cause | What to do |
|---|---|---|
| `EcNotReadyError: emergency stop is active`, or `servo ON not sent: the emergency-stop input is active ...`; flags `EMG`, `EMG-INPUT` | The stop button or the safety circuit opened the unit's emergency-stop input. | Remove the cause and reset the safety circuit, then `servo on`. The driver never overrides the input. |
| `servo ON not sent: the motor drive power is cut ...`; flag `DRIVE-POWER-CUT` | An external drive cut-off removed the motor power (DOC ME0162-12A p.4-20, 4-21). | Check the drive cut-off circuit and the external stop wiring. |
| `servo ON not sent: the enable switch of a connected teaching tool is released ...`; flag `ENABLE-SWITCH-OFF` | A teaching tool is plugged in with its enable switch released. | Unplug the tool or hold its switch. After unplugging, servo ON and HOME stay locked out for a short time (DOC ME0162-12A p.5-83, 5-91): retry after that. |
| `EcAlarmError: alarm 0x0c1 is active; call reset_alarm()`; flag `ALARM` | An alarm switched the servo off and blocks motion. | See [Alarms](#alarms). |
| `servo did not turn on within 3.0 s: the mode switch is in AUTO ...`, or `... Modbus control is off (DSSE PMSS = 0) ...` | The PIO has control: a mode switch in AUTO, IA-OS's Monitor mode, or a PLC. | Set the switch to MANU or close IA-OS, then retry. The driver takes PC control at rest. |
| `servo did not turn on within ... s: the excitation detection of a first servo ON is still running ...`, or `(the status shows no known cause)`; flag `EXCITATION-RUNNING` | A CON controller runs an excitation detection at the first servo ON after power-up (DOC PCON p.4-24). The driver waits as long as parameters No.29 / No.30 give (about 13 s when unreadable). | Read `status` and repeat `servo on`. If it never comes on, send the `iai-check` report. |
| `servo did not turn on within 3 s (the status shows no known cause)` on a PCON in PC mode, no alarm, DSSE bit 12 at 0 (for example 0x23C2 where the unit read 0x33C2 before) | A 0x45 cmd 0x0C action-mode command with op1 = 1 (PC) cleared DSSE bit 12, and the unit takes no servo ON until a PMSL coil OFF (VERIFIED on the PCON-CB, 2026-11-02). The 1.0.0 `set_action_mode(True, ...)` sent it, and another program can too. From 1.0.1 on `set_action_mode(True)` and `mode pc` take PC control with the PMSL coil instead. | At rest, with nothing on the PIO connector that would act: `ax.disable_modbus_control()`, then `ax.enable_modbus_control()` (PMSL OFF, then ON), then `servo on` (VERIFIED on the PCON-CB). PMSL OFF hands control to the PIO for that moment. |
| `the servo did not come back on after the automatic servo OFF ...`; flag `AUTO-SERVO-OFF` | The unit switched its servo off after a standstill. The driver switches it back on before the next move (UNVERIFIED on hardware). | Read the reasons, then `servo on` and retry. |
| `EcNotReadyError: servo turned off during motion` | An alarm, the emergency-stop input or a drive power cut. | `status` shows which. |
| `EcNotReadyError: servo is off; call servo(True)` | A move or home was sent with the servo off. | `servo on` first. |
| A vertical axis drops when the servo goes off | Servo OFF, a servo-off alarm, an emergency stop or a software reset releases position control. A vertical axis without a brake falls; one with a brake is held only as long as the brake holds. | Test vertical axes unloaded and slow. See [COMPATIBILITY.md](COMPATIBILITY.md#possible-trouble). |

## Homing

| Symptom | Likely cause | What to do |
|---|---|---|
| `EcNotReadyError: axis is not homed; call home()`; `swreset` prints `NEEDS HOME` | An incremental unit (no WA option, such as the EC-R6S rod) loses its home at every power-up and software reset (VERIFIED). An absolute (WA) unit loses it after a change of home direction or offset. | `home` with the servo on and the travel clear: it moves to the mechanical end. |
| `MotionTimeoutError: homing did not finish within 25.0 s`; the axis was stopped | Something blocks the travel, or a STOP or pause from another master held it. The limit is 10 s plus the time to travel the stroke twice at 20 mm/s. | Clear the travel and check `status` (`PAUSED`, `ALARM`). `ax.home(timeout=60)` waits longer. |
| `home return did not start within 2.0 s (HOME ignored?)` | HOME is locked out for a moment after a teaching tool is unplugged (DOC ME0162-12A p.5-83, 5-91), or another master holds a STOP. | Disconnect other tools and retry after a moment. |
| `a home return is in progress; wait for it to finish` | A home return started earlier is still running. | Wait for it, or STOP. |
| `0x45 home needs a valid home pattern ORG (item 9.16) ... Use method='coil'. Nothing sent` | `home --method iai` needs a valid home pattern. | Use the default `home` (coil). |
| Alarm 0x0EE `ABS error (2)` after a parameter change and `swreset` | A home direction or offset change (EC P3 / P4, CON No.5 / No.22) on an absolute unit (VERIFIED on the EC-GRBP8M gripper). | `alarm-reset`, then `home`. |
| Alarm 0x0BE `Time-out error while Homing` | The home return took too long (blocked or overloaded axis). | Check for interference, then `alarm-reset` and `home`. |
| `teach` is refused: `the axis is not homed ...` | Teaching needs a homed axis. | `home` first. |

## Moves refused by the limit checks

Every target, speed, acceleration, push force and stored row is checked against the unit's own limits before anything
is sent. A refusal raises `SoftLimitError` and sends nothing. `limits` (`ax.limits()`) prints every range with its
source; `ax.check_move(...)` runs the checks without moving.

The command line and the single-axis GUI page add caps of 100 mm/s, 0.3 G and 70 % push force; `--unit-max` lifts them
to the unit's own maxima. In Python the unit's own limits apply, unless `max_speed_mm_s`, `max_accel_g` or
`max_push_pct` lower them.

| Symptom | Likely cause | What to do |
|---|---|---|
| `target 200.0 mm outside the effective stroke 0.00..150.00 mm (soft limits -0.30..150.30 mm)` | Outside the soft limits minus their 0.30 mm margin. | Use a target inside the range `limits` prints. |
| `target 130.0 mm outside soft limits 10.00..120.00 mm` | Outside a stroke you configured (`--stroke`, `stroke=`). | Change or drop the override. |
| `speed 150.0 mm/s outside 0.01..100 mm/s (max_speed_mm_s)`, `acceleration 0.5 G outside 0.01..0.3 G (max_accel_g)` | Above the command line's cap, or a cap you set. | `--unit-max` (before the command) lifts the caps; `--max-speed` / `--max-accel` set them. |
| `speed 60 mm/s exceeds this EC's maximum speed 53.00 mm/s (MAXV ...)`, `... exceeds this EC's maximum 0.30 G (MAXA 7.5)` | Above the unit's own maximum. These are horizontal, light-load values: a vertical mounting or a payload lowers the real limit. | Stay below it. It cannot be lifted. |
| `push force 90 % outside 1..60 % (MXPP, item 7.9); nothing written` | Above the unit's push maximum or the 70 % cap. | `--max-push` or `--unit-max` for the cap. MXPP is the unit's hard limit. |
| `push force 10 % is below this EC's recommended minimum 20 % ...` | Below the recommended minimum (MNPP, or 20 %). | A warning only: the push runs. |
| `speed 20 mm/s is below this unit's documented minimum 25 mm/s ...` | Below the documented minimum speed (DOC). The command line's defaults (20 mm/s for `move`, 10 mm/s for `jog` and the push approach) can be below it. | A warning only. Use a higher speed if the axis runs rough. |
| `stroke unknown: this EC's soft limits are not available (...); pass stroke=(min_mm, max_mm)`; `iai-check` warns `motion commands will be refused until a stroke is configured` | The limits could not be read: old firmware without a register directory, or a gateway line where it is silent. | `--stroke MIN MAX` or `stroke=(min, max)`. Speed, acceleration and push then fall back to 100 mm/s, 0.3 G and 70 %. |
| `configured stroke ... does not overlap this EC's soft limits ...` | The `--stroke` belongs to another unit. | Correct or drop it. |
| `band 0.05 mm below this unit's minimum positioning band MINI ...` | The positioning band is below the unit's minimum. | Use a larger band. |
| `stored position 2 runs at its stored speed 248.50 mm/s, above the speed cap 100 mm/s ...` (`goto 2`) | The row's stored speed is above the command line's cap. | `goto 2 --allow-fast` (`allow_above_caps=True`) runs it at its own speed, within the unit's maximum. `--speed-pct 25` runs it slower. |
| `row 2 (middle) is not used in 2-position mode (P9 = 0 ...) ...; nothing sent` | The unit is in 2-position mode (EC P9 = 0 or 3) and runs rows 0 and 1 only. The EC-GRBP8M gripper rejected row 2 (VERIFIED). `teach` and `write_position_table()` write it with a warning (`... written anyway`), so the row can be prepared. | Use row 0 or 1, or `move_to()` for the middle point. To run row 2, set P9 to 3-position mode with IA-OS and `swreset`. |
| `row 2 (middle) is not used in the unit's position mode: the unit rejected it ...; nothing moved` | As above, when P9 could not be read. | As above. `param get 9` shows P9 when readable. |
| `stored position 1 is an incremental row (CTLF bit 3) ...` | The driver does not run relative rows. | Use `move_by()`. |
| `stored position 1: its checksum word +0xF does not match the row's data; not run` | The stored row is inconsistent. | Read it again. If it stays, write the row again from your backup or with IA-OS. |
| `start position 80.00 mm is inside or past the push zone ...` | A push must start on the approach side of its zone. | Move to the approach side first. |
| `push band end ... outside the stroke ...` | The push zone ends outside the stroke. | Shorten the band. |
| `this EC cannot push (SUPT PUSH ..., MXPP ..., TCPC ...)` | The unit reports no push support. | Use plain moves. |
| `accel_pattern 's_motion': this unit does not report it ...`, `P5 smooth acceleration is on: accel / decel time ... above 2 s is not allowed` | The unit lacks that pattern, or smooth acceleration forbids long ramps (DOC ME3816-6B p.6-11). | Use `"trapezoid"`, or raise the acceleration. |
| Positions and speeds look wrong on a rotary unit | A rotary axis works in degrees and deg/s (`ax.units`). | Give targets in degrees. |
| Moves run at about 100 mm/s whatever speed you give; flag `SAFETY-SPEED`; `iai-check` warns `DSS1 bit 14 (SFTY) is ON` | Safety-speed mode caps motion at about 100 mm/s (VERIFIED). IA-OS's Teach mode can leave it on; it is off after a power-up. The driver never changes it by itself. | `iai-actuator-driver --port COM3 mode pc` or `ax.set_safety_speed(False)`. |

## Alarms

`status` shows the alarm code and the flag `ALARM` (a fault: motion is blocked, red lamp) or `WARNING` (a
message-level alarm: the axis keeps running, amber lamp). `info --section alarms` prints the current alarm with its
level and remedy, and the 32-slot history. `alarms` prints the history alone, newest first. In Python:
`ax.status().alarm_code`, `ax.read_alarm_history()`, `iai_actuator_driver.alarms.alarm_info(code)`.

Codes and history are VERIFIED; names, levels and groups are UNVERIFIED unless seen on hardware. A message-level alarm
sets DSS1 b9 (ALML). On the PCON-CB that bit stayed on after the reset cleared the code (VERIFIED), so `WARNING` can
show with alarm code 0.

| Level | What the unit does | What clears it |
|---|---|---|
| message (codes 0x011 to 0x07F) | Keeps running | `alarm-reset` once the cause is dealt with |
| operation cancelled | Decelerates, stops, servo off | `alarm-reset`, then `servo on` |
| cold start | The same stop | `swreset` (servo off first) or a 24 V power cycle; an incremental unit then needs `home` |
| unknown | Treated as a fault unless the warning bit is set | `alarm-reset`; if it stays, `swreset` or a power cycle |

**Alarm reset or software reset.** `alarm-reset` (`ax.reset_alarm()`) pulses the alarm-reset coil (ALRS). It clears
message and operation-cancelled alarms and never restarts the unit (`reset` is a deprecated alias). `swreset`
(`ax.software_reset()`) restarts the firmware: the servo goes off first, committed parameters take effect, and an
incremental unit loses its home. It runs only at rest, never during a parameter commit. PC control survives on the
EC-GRBP8M gripper and the PCON-CB but not on the EC-R6S rod (VERIFIED); the driver takes it again at the next command.

| Symptom | Likely cause | What to do |
|---|---|---|
| `EcAlarmError: alarm 0x0c1 is active; call reset_alarm()`, or `alarm 0x0c1 during motion` | A fault stopped the axis or blocks the command. | Read the alarm, remove the cause, then `alarm-reset` and `servo on`. |
| The alarm is still there after `alarm-reset` | A cold-start alarm, or the cause is still present (such as an emergency-stop input). | Remove the cause. For a cold-start alarm, `swreset` or a power cycle. |
| `software reset only at rest (not moving, pushing, paused or homing)` | `swreset` while the axis was busy. | Let the move end or `stop` it (`cancel-pause` for a paused move), then `swreset`. |
| `a parameter write is still in progress after 2 s ...: software reset not sent ...` | A parameter commit is still writing non-volatile memory. | Wait, then retry. Do not power off meanwhile. |
| `restart NOT confirmed (no PMSS drop, reply gap, uptime restart or HEND loss seen): check the unit` | No restart sign was seen after `swreset`. | Check `status` and the alarm; repeat `swreset` or power-cycle. |
| `swreset` refused with `LineRuleError: ... does not carry IAI 0x45 / 0x48 commands` | A gateway line cannot carry the software reset. | Clear a cold-start alarm with a power cycle. |
| 0x0EE `ABS error (2)` | The unit lost its home, such as after a home direction or offset change. | Absolute unit: `alarm-reset`, then `home` (VERIFIED on the EC-GRBP8M gripper). Incremental unit: `swreset` or a power cycle, then `home` (UNVERIFIED). |
| 0x090 `Software reset command while in SERVO-ON condition` | A software reset reached the unit with the servo on (VERIFIED). The driver's `swreset` switches the servo off first, so it came from elsewhere. | `alarm-reset` (VERIFIED). |
| 0x0C1 `Servo error` | The axis did not move for 2 s after a move command: blocked or held, a cable fault, a brake not released or high sliding resistance (DOC ME0342-4H p.8-19). | Free the axis, then `alarm-reset`. |
| 0x0D9 `Software stroke limit over error` | The axis is outside its soft limits. | After the reset, move it back inside them, slowly or by hand. |
| 0x0CE `Control power voltage reduction`, 0x0D2 `Power supply voltage of the motor is excessive` | Supply voltage too low (0x0CE) or too high (0x0D2). | Check the 24 V supply and its capacity. The driver treats both as cold start; an alarm reset may already clear them. |
| `WARNING` with 0x069 `Real Time Clock oscillation stop detection` (PCON-CB) | The calendar clock stopped. The PCON-CB raises 0x069 at power-up (VERIFIED); the manual names the first power-up after delivery, and about 10 days without power (DOC ME0342-4H p.Intro-9). Moves still run (VERIFIED). `alarm-reset` clears the code, but DSS1 b9 stays on, so `WARNING` and the amber lamp remain with code 0 (VERIFIED for the bit). No history entry is written, and it returns at each power-up while the clock is stopped (VERIFIED; after a software reset, UNVERIFIED). | Nothing is needed for the axis to run: read the code, not the flag. To set the clock, use the teaching tool or the RC PC software (DOC ME0342-4H p.8-15); the driver does not set it. Whether that clears the bit, and whether No.111 = 0 (the manual's fix for 0x06A) affects 0x069, is UNVERIFIED. |
| `WARNING` with 0x04E / 0x04F (maintenance warnings; which is which is UNVERIFIED) | The travel count or distance reached its target in the unit's maintenance information. | Raise the target, or set it to 0 (off) (DOC ME3816-6B p.8-26). |
| The history is full of `0x0fff ... PowerUP (no error)` | A marker written at every power-up, not an alarm (VERIFIED). | Nothing. |

## Stops and pauses

`stop` (`ax.stop()`) is a decelerate-stop over Modbus: STOP on, wait for standstill, STOP off. It works with or without
PC control and stopped Modbus-commanded moves on hardware (VERIFIED). It is UNVERIFIED on a move a PLC commanded over
PIO, and it is not an emergency stop. Behind an RCON gateway (no coils) the stop is pause, wait, release (DOC,
UNVERIFIED on hardware).

Every stop logs one line at info level when it ends, for example `COM3 slave 1: STOP on the wire 7 ms after the call,
standstill confirmed 180 ms later` (log source `iai_actuator_driver.axis`; a broadcast STOP: `iai_actuator_driver.port`).
The first number is how long the STOP frame took to leave the PC: a request already on the line, or a busy PC, makes
it longer. The second is how long the axis took to stand still after it. A stop that does not wait for standstill
(`begin_stop`, a broadcast STOP) gives the first number only; the decel stop sent after an error says `decel stop`.

| Symptom | Likely cause | What to do |
|---|---|---|
| `MotionTimeoutError: axis still moving 5.0 s after STOP`; GUI `STOP NOT CONFIRMED` | A move commanded by a PLC, or a lost link. | Stop it from the PLC or with the hardware emergency stop. Check the link with `status`. |
| GUI: `AXIS MOVED AGAIN AFTER STOP` | The PIO has control and a PLC may still hold its start input. | Stop the move from the PLC or with the hardware stop. |
| `EcNotReadyError: axis is paused (STP is on); call resume() first` | A pause is held. The command line's `pause` leaves it set; another master can hold it too. | `resume` continues the move; `cancel-pause` drops the rest of it. |
| The axis moves on by itself after `resume` | A paused move continues when the pause is released. | Use `cancel-pause` when the rest of the move is not wanted. |
| `no pause is active (DSS1 STP = 0) ...; nothing sent` | `cancel-pause` without a pause. | Nothing to cancel. |
| `alarm 0x... is active: the ALRS edge would also clear it ...; nothing sent` | An alarm ended the paused move. | `alarm-reset`, then `resume`. |
| `MotionIncompleteError: axis settled at ..., ... from the target ...` | Something else stopped the move, or the unit clamped the target to its own limits. | Look for another program, a PLC or a held STOP. Compare the target with `limits`. |
| `MotionIncompleteError: ... stopped from outside this wait ...` | `stop()` of the same axis (or a group stop) from another thread ended the move or push. No second STOP was sent. | Expected when another thread stops the axis. Send the next move once that stop has returned. |
| `MotionIncompleteError: stored position 1 (0x9800 = 1) was ignored ...` | A STOP or pause held elsewhere, Monitor mode, or firmware that does not run stored-row moves over Modbus. | As above, or use `move_to()` to the row's position. |
| A `completion="fast"` move returns slightly short of the target (up to 0.08 mm on the PCON-CB, inside the 0.1 mm band) | The fast rule ends on the first in-position sample while the unit still settles (VERIFIED on the PCON-CB and the EC-GRBP8M gripper). Not a fault. | Use `completion="safe"` when the final position matters ([DRIVER.md](DRIVER.md#fast-path)). |
| `MotionTimeoutError: not done after ... s (target ...): ...` with `decel stop sent after this error` | The move took longer than its estimate, and the axis was stopped. | The flags in the message show the state (`paused=True`: a held pause). Pass a longer `timeout=` for slow moves under load. |
| `interrupted (a decel stop was attempted if motion had been commanded)` | Ctrl+C during a command (exit code 130). | Check `status`. |
| The axis keeps moving, or a push keeps pushing, after a crash or a pulled cable | The unit has no communication watchdog (VERIFIED). | Design the machine for it. The hardware emergency stop is the only stop independent of the PC. |
| The axis keeps pressing after `push` returned `contact` | After contact the motor pushes until the next move command. | Send the next move, such as a retract with `move_to`. |

## Several axes on one port and gateways

One axis per COM port is the VERIFIED layout. Several axes on one port (SIO daisy chain or RCON gateway) and every
gateway rule are DOC and UNVERIFIED on hardware. The address is the axis number plus 1.

| Symptom | Likely cause | What to do |
|---|---|---|
| `scan` finds fewer axes than are wired | Two units share an axis number, or an axis is outside the scanned addresses (default 1 to 16). | Give each unit its own axis number, or scan wider: `scan --ports COM5 --slaves 1-32`. |
| Alarm 0x05E "Delimiter error" on every unit after a wide scan (an older driver) | A request to slave 58 (0x3A = ':') starts like a Modbus ASCII frame (ON-WIRE on a PCON-CB). | Reset the alarm. The driver now skips 58 in scans and refuses it; no IAI unit can use address 58. |
| Commands reach the wrong axis | `--slave` is the address, `--axis` the axis number: `--axis 2` is `--slave 3`. | Use one consistently; `scan` lists `axis N` per address. |
| A `Cell` opens without one of its ports; log `port COM4 skipped: ...` | With `slaves="scan"`, a port that fails or has no answer is skipped and kept in `cell.port_errors`. | Read `cell.port_errors`. List the slaves (`slaves={"COM4": [1]}`) to make it raise instead. |
| `CellError: move_all (check; nothing started) failed on 1 axis/axes: ...` | A group move is checked on every axis before any starts. | Fix that axis's target. `.errors` and `.results` are keyed by axis name; `return_exceptions=True` returns per-axis results. When one axis fails during `move_all`, `home_all` or `wait_all`, the others are stopped. |
| `coordinated move stopped (axis x: LateFrame: the frame of step 28 would reach the wire 702 ms late (limit 98 ms) ...)` | A frame would have left the PC more than one step late: a busy or sleeping PC, or other traffic on the port (another axis's status monitor). On Linux with 1.0.0 also a backward step of the system clock during the run (an NTP step, a VM's time sync, as in WSL). | Keep the PC free and the ports quiet during the run, or allow longer steps (a lower speed, `min_step_s=`). On Linux use 1.0.1 or later: its waits run on the monotonic clock. |
| `COM5:2 is not at rest: baud change refused (nothing sent)` | A baud change switches every unit on the line. | Bring every axis of the port to rest first. |
| Log `COM5: slaves [3] did not answer at 115200 baud after the broadcast` | One unit did not follow; several units switching at once is UNVERIFIED. | Power-cycle that unit (back at 38400) and connect again. |
| `GatewayModeError: the RCON gateway is in AUTO mode ...` | The RCON gateway accepts Modbus control in MANU mode only (DOC ME0413 p.6-1). | Set its AUTO/MANU switch to MANU. |
| `RCON line without an answering gateway (D0h) ...`, or `the RCON gateway (0xD0) did not answer its status read` | The gateway's own status address does not answer. | Check the gateway's power and wiring. |
| `PositionUnitError: slave 1: RCON driver units may count positions in 0.01 mm or 0.001 mm ...`; `line` prints `NOT STATED` | No.207 cannot be read over Modbus. | State it: `--position-unit 0.01` or `0.001` (`position_unit_mm=`, `Port.set_position_unit()`). |
| `ValueError: position unit 0.001 mm refused on a direct line ...` | `--position-unit 0.001` (`position_unit_mm=0.001`) on a line without an RCON gateway. A unit on a direct line counts in 0.01 mm. | Drop the option, or state `--line rcon` when the unit is behind an RCON gateway. |
| Moves on an RCON axis go ten times too far, or a tenth of the distance | The stated position unit does not match No.207. | State the value the unit uses. |
| `LineRuleError: ...: rcon does not carry IAI 0x45 / 0x48 commands` (also coils, broadcasts, `baud`, `mode`, `goto --speed-pct`, `home --method iai`) | The RCON gateway carries function codes 01, 03, 04, 06, 10 and 17 only (DOC ME0413 p.3-1). Nothing was sent. | Moves, servo, home, alarm reset and stop run over registers there. |
| `stroke unknown: ... pass stroke=(min_mm, max_mm)` on an RCON line | A silent register directory means no directory on that line. | `--stroke MIN MAX`. |
| `slave(s) 2 are linked at the RCON gateway but do not answer Modbus ...` | An ELECYLINDER behind an RCON-EC / REC connection unit is not reachable over Modbus (DOC ME0413 p.4-1). | Not supported. |
| `UnsupportedLineError: ... the ROBONET gateway is in gateway mode (SW2 OFF, slave 3Fh) ...` | Gateway mode gives no axis access over Modbus. | Set SW2 ON and open the port with `--line robonet`. |
| `line` shows `direct` on a ROBONET line | SIO through mode cannot be detected. | State `--line robonet`, so that nothing is broadcast: a broadcast baud change could leave every axis unreachable until a power cycle (DOC ME0208 p.26, p.340). |

## The GUI

| Symptom | Likely cause | What to do |
|---|---|---|
| `Cannot listen on 127.0.0.1:8780: ...` | Another program uses that HTTP port. | Close it, or start `iai-gui --port 8790`. |
| `Missing or wrong access key.`, or a red "This page has no access key" line | The page was opened without this run's key (new tab, or an earlier run). | Open the address printed in this run's console (it ends with `#k=` and the key). |
| `The request does not come from this GUI's own page.` | A request came from another web page. | Use the GUI only from its own address. |
| `Untrusted Host header.` | The page was opened under a name other than 127.0.0.1 or localhost. | Open the printed address. The GUI listens on this PC only. |
| `COM3 is not in the list of serial ports` | The PC does not list that port. | Press Refresh and pick a listed port. |
| Red **STOP NOT CONFIRMED** banner | The STOP request was not answered within 2 s (twice) or was refused. | Use the hardware emergency stop. The banner goes when a STOP ALL is answered. |
| `status 12 s old` on an axis | Another operation (a coordinated plan, a backup, a unit-info read) holds the axis. | Wait. Do not rely on the greyed position or Moving flag. |
| `Simulator-only mode (--sim-only): serial ports are not opened.` | Demo mode. | Start `iai-gui` without `--sim-only`. |
| `Cannot open COM3: ... Close IA-OS (or any program using it) first.` | Another program holds the port. | See [Connection and ports](#connection-and-ports). |
| `COM3: no axis answered at addresses 1..16 at any baud rate; ...` | Nothing answers on that port. | See [No answer](#no-answer-and-timeouts). |
| `COM3 is already connected.` | The port is open in this GUI. | Use it, or disconnect it first. |
| `... is busy with ...; try again.` or `driver busy, try again.` | Another request is still running on that axis. | Wait, or press STOP (always in the header; also Space or Esc). |
| Single-axis page: `Motion disabled: this EC's soft limits could not be read ...` | The unit's limits are unreadable. | Disconnect, enter the stroke in the connect form, connect again. |
| Single-axis page: `Motion disabled: the stroke ... lies outside this EC's own stroke ...` | The stroke override does not fit this unit. | Correct it, or leave it empty. |
| Single-axis page: speeds stop at 100 mm/s | The page caps 100 mm/s, 0.3 G and 70 % push force (lower when the unit's maximum is lower). | Use the workbench (`/`) or the command line with `--unit-max`. |
| Fault banner on an axis, with a **Recover** button | The link was lost or the unit restarted. | Press Recover and read the report; home if it says so. |
| Unit info panels stay half filled after STOP | STOP ends a running unit-info read. | Press Refresh. |
| About panel: `Not found: ... (not part of this install).` | The documents are served from a source checkout only. | Read them in the `docs` folder. |
| The backup from *Back up this unit* is not where you expected | It goes to `backups/` in the project folder (source checkout), else in the folder `iai-gui` was started from. | Look there. |

## Non-volatile writes

The units' non-volatile memory lasts about 100,000 write cycles (IAI). Parameter commits, stored-row writes and
teaching write it. Direct moves (`move_to`) are RAM only (VERIFIED), as are stored-row moves, PC control, safety
speed, the baud rate, servo, alarm reset and every read. Back a unit up before a non-volatile write:
`iai-actuator-driver --port COM3 backup unit1.json` (read-only). The driver does not restore backups. Details:
[REFERENCE.md](REFERENCE.md#safety-notes).

| Symptom | Likely cause | What to do |
|---|---|---|
| `NonVolatileWriteError: rows 256-267 are the EC's internal simple-data store and are never written; nothing written`, or `... position N not written` | The row is the EC's internal store, or the table cannot address it. Stored-row writes and teach need no opt-in otherwise. | Write a row of the unit's table (`max_positions()`). |
| Log WARNING `position N: ... written anyway` (also in the C++ result's `warnings`, `iad_axis_last_result_json` and `NativeAxis.last_result()`) | The row was written although the unit does not run it (past its rows, unused in this position mode, a push on the middle row) or the control flags hold unknown or changed role bits. | Nothing if you meant it; otherwise write the row again from `read_position_table(N)`. |
| `teach` refused: `row 0 stored speed 108.5 mm/s is above this axis's max_speed_mm_s 100 mm/s ...` | Teaching keeps the row's speed and acceleration, which must fit the caps when the row changes. | Add `--unit-max` before the command: `iai-actuator-driver --port COM3 --unit-max teach 0`. |
| `unchanged: nothing written (one non-volatile write cycle saved); --force writes it anyway`; `result.skipped` | The unit already holds that value. | Nothing. Use `--force` (`force=True`) only after another program changed the value in RAM: the driver compares with the unit's RAM. |
| Log WARNING `... non-volatile writes in the last hour on COM3 slave 1 ...` | More than 60 writes in one hour on one axis, typically a program writing a row or parameter every cycle. | Use `move_to()` for targets that change every cycle; write rows and parameters at setup only. The driver warns once per hour and never refuses. `ax.nonvolatile_writes` and `ax.nonvolatile_writes_last_hour` count the writes. |
| `PARW 0x0428 sent, but the commit was NOT CONFIRMED ...: the value may be RAM only` | The unit was not seen acting on the commit. | Read the value back after a power cycle (`param get`); write it again with `--force` if it was lost. |
| A new parameter has no effect; `param set` prints `run 'swreset' to apply it now` | Parameters take effect after a software reset or a power cycle. | `swreset` at rest; an incremental unit then needs `home`. |
| `VerificationError: PARW ... handshake not finished 2 s after the commit ...: Do not reset or power off the EC until DRGE b7 reads 0` | The commit is still running, or it hangs. | Wait; do not power off. The `NV-WRITE-BUSY` flag shows a write in progress (DOC). |
| `warning: P3 / P4 are home parameters. ...` after `param set 3` / `param set 4` | A home direction or offset change loses the home: an absolute unit raises 0x0EE after the reset (VERIFIED on the EC-GRBP8M gripper). The value was written. | `swreset`, `alarm-reset` and `home` before the next move. |
| `warning: ... is a dangerous parameter (...)`, `... widens the +/- soft limit ...` or `... is not in the parameter list for unit ...; written anyway` | The write could change how the unit moves or talks (home, motor or encoder type, lead, gains, PIO / fieldbus / baud setup, a wider soft limit, an unknown raw item), or the unit's own list does not show the parameter. The value was written. | Check the value against the unit's manual; write the old value back from the backup if it was wrong. |

## Sending a report

`iai-check --port COM3` (read-only) writes `iai-check-<timestamp>.json` with the model, firmware, limits and every raw
value read. Open a bug report on [GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose)
and attach it with the `--log-frames` output of the failing command and, if you are willing to share it, a backup
(`backup unit1.json`). See [COMPATIBILITY.md](COMPATIBILITY.md#sending-a-report).
