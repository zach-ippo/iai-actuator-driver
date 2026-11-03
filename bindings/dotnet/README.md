# IAI Actuator Driver for .NET

A .NET class library that drives IAI actuators (ELECYLINDER, RoboCylinders on PCON / ACON / SCON controllers, rotary
units, grippers) over the serial port with Modbus RTU. It runs on the same C++ core as the Python package, and reads
every range and capability from the unit itself, not from a per-model table: every IAI unit that speaks Modbus RTU
on its serial port is driven the same way.

Unofficial: not an IAI product and not supported or endorsed by IAI
([README](https://github.com/zach-ippo/iai-actuator-driver#readme)). Bugs and results from your unit go to
[GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose).

Licence: Apache License 2.0 ([LICENSE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/LICENSE), [NOTICE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/licenses/NOTICE)).

## Quick start

Install the package from the [GitHub release](https://github.com/zach-ippo/iai-actuator-driver/releases). It carries
the core for win-x64 and linux-x64:

```shell
dotnet add package IaiActuatorDriver --version 1.1.0 --source <folder with the .nupkg>
```

The library targets `netstandard2.0` and `net8.0`, so it also runs on .NET Framework 4.6.2+ (64-bit) and .NET 6. Close
IA-OS first: only one program can own a serial port.

From source (the .NET 8 SDK and the core's library, which the build copies next to your program):

```shell
cd core && python build.py shared      # Windows x64, once (pip install ziglang): core/build/iai_actuator_driver.dll
cd ../bindings/dotnet
dotnet build                           # stops and names that command when the library is missing
dotnet test                            # simulator only, no hardware
```

One axis:

```csharp
using IaiActuatorDriver;

using var axis = Axis.Open("COM3");           // baud rate found in place
DeviceInfo info = axis.Connect();             // read identity and ranges, take control at rest
axis.Servo(true);
axis.Home();
axis.MoveTo(50.0, speed: 20);                 // waits until in position and settled
axis.Stop();                                  // decel stop, usable at any time and from any thread
axis.Servo(false);
```

Many axes, normally one per COM port, found automatically:

```csharp
using var cell = Cell.Open();                 // every IAI USB adapter, every answering axis (read-only)
Console.WriteLine(string.Join(", ", cell.Names));          // "COM3:1", "COM4:1", ... = port:address
cell.ServoAll(true);
cell.HomeAll();
cell.MoveAll(new Dictionary<string, double> { ["COM3:1"] = 50.0, ["COM4:1"] = 20.0 }, speed: 30);
cell.StopAll();
```

No hardware:

```csharp
using IaiActuatorDriver.Simulation;

using var axis = Axis.Open(new SimulatedUnit(SimProfile.Rod));    // a simulated EC-R6S rod, real time
using var cell = Cell.Open(SimLine.SimPorts(3));                 // "SIM1:1" .. "SIM3:1", one unit per port
```

Command line ([all commands](#command-line)):

```shell
dotnet run --project src/IaiActuatorDriver.Cli -- scan                         # axes on every IAI adapter (read-only)
dotnet run --project src/IaiActuatorDriver.Cli -- --port COM3 status           # read-only
dotnet run --project src/IaiActuatorDriver.Cli -- --port COM3 move 50 --speed 20
dotnet run --project src/IaiActuatorDriver.Cli -- --sim rod move 50 --speed 20 # the same on the simulator
```

After `dotnet build` the tool is `src/IaiActuatorDriver.Cli/bin/Debug/net8.0/iai-actuator-driver-cs.exe`. Exit codes:
0 ok, 1 nothing found by `scan`, 2 error, 130 interrupted.

## Projects

| Project | Target | Contents |
|---|---|---|
| `src/IaiActuatorDriver` | `netstandard2.0`, `net8.0` | The library, with no NuGet dependencies: `Axis`, `Cell`, `Port`, the simulator (`IaiActuatorDriver.Simulation`) and the interop layer over the core (`Interop/`) |
| `src/IaiActuatorDriver.Cli` | `net8.0` | The command line `iai-actuator-driver-cs` |
| `tests/IaiActuatorDriver.Tests` | `net8.0`, xUnit | Simulator, `FakeTransport` and words recorded from real units; no test opens a serial port |
| `tests/IaiActuatorDriver.Net48Smoke` | `net48` (x64), Windows | The `netstandard2.0` build on .NET Framework 4.8; not in the solution: `dotnet test tests/IaiActuatorDriver.Net48Smoke` |

## Multi-axis layouts

- **One axis per COM port** (the normal case, VERIFIED): one `Port` per COM port, or `Cell.Open()` for every IAI USB
  adapter. Every port runs in parallel; each port has its own lock. One axis per USB adapter runs fastest.
- **Several axes on one port** by slave address (RCON gateway, SIO daisy chain; address = axis number + 1;
  UNVERIFIED on hardware): one `Port`, whose lock interleaves the axes' frames, so the axes move at the same time.
  A `Cell` names the axes `"<port>:<address>"`.
- Both can be mixed in one `Cell`. A cell takes every axis it finds (no total limit; `MaxAxes` caps it if given). One
  cable carries at most 16 axes (axis numbers 0..15, addresses 1..16), and the scan asks addresses 1..16 per port by
  default; axes on one cable take turns. `SimLine` models both without hardware.

## API table

`axis` is an `Axis`, `cell` a `Cell`, `port` a `Port`. Positions are in **mm** (**deg** on a rotary axis), speeds in
units/s, accelerations in **G**. Ranges come from the unit (`axis.Limits()` shows each with its source). Every motion
input is checked before a frame is sent: a value out of range raises `SoftLimitException` and nothing is sent.
Besides the `Axis` commands, which apply the [guards](#guards-and-safety), the only writes are the raw `ModbusRtu`
write calls: they send exactly what you ask (IAI commands still pass the safe and never-send lists).

### Connect and discover

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `PortManager.ListPorts()` | Serial ports, IAI USB adapters first (`SerialPortInfo.IsIai`). Opens nothing. | | `PortManager.ListPorts()` |
| `PortManager.Scan(portName, slaves, timeout, identify, baudRate, readTimeout, line)` | Opens a port, finds the addresses that answer (with FC11 identity), closes it. Read-only. | addresses 1..247, default 1..16; 50 ms per missing address | `PortManager.Scan("COM3")` |
| `Axis.Open(portName, baudRate, slave, options, readTimeout, retries, frameLogger)` | Opens a port and returns one axis that owns it. | `baudRate` `null` = found in place; `options` can only narrow the unit's ranges | `var axis = Axis.Open("COM3");` |
| `Axis.Open(transport, slave, options, autoBaud, ...)` | The same on any `ITransport` (`SimulatedUnit`, `SimLine`, `FakeTransport`). | as above | `Axis.Open(new SimulatedUnit())` |
| `axis.Connect(takeControl)` | Reads identity and ranges, takes Modbus control at rest. Returns `DeviceInfo`. | | `var info = axis.Connect();` |
| `axis.Close()` / `Dispose()` | Releases a STOP left on; closes the port if the axis owns it. | | `axis.Dispose();` |
| `axis.Identify()` | Family, model, serials, firmware, axis number, soft limits, kind. Read-only. | | `axis.Identify().Model` |
| `axis.UnitInfo(sections)` | Everything the unit reports, as one JSON-able dictionary in the layout of the Python `unit_info`. Read-only. | `identity`, `parameters`, `monitor`, `alarms`, `maintenance`, `positions`, and this edition's `limits` (all when none is named) | `axis.UnitInfo()` |
| `axis.Backup(path, rows)` / `BackupJson(path, rows)` / `Axis.BackupAll(axes, directory, names, rows)` | Read-only backup (identity, directory, support words, parameters, rows, alarms, maintenance; schema `ec_driver.backup/1`) as a dictionary or JSON, optionally to a file; one file per axis. FC03 / FC11 only. | `rows`: `"used"` (default), `"all"` (768 on an EC: slow), `"0,1,2"` | `axis.Backup("backup.json");` |
| `axis.Limits()` | Every range the driver enforces (`AxisRanges`), with its source. | | `axis.Limits().Speed.Max` |
| `axis.Units` / `Kind` / `ProductFamily` / `IsEc` | `"mm"` or `"deg"`, actuator kind, family (`EC`, `PCON` ...), EC or CON unit. | | `axis.Units` |
| `Port.Open(portName, baudRate, detectSlaves, readTimeout, retries, frameLogger, line)` | Opens a port shared by several axes. | `line`: `LineKind.Auto` (default; read-only gateway probes, about 0.1 s more without a gateway), `Direct` (no probes), `Rcon`, `Robonet`, `RobonetGateway` | `var port = Port.Open("COM5");` |
| `new Port(transport, name, retries, minGap, frameLogger, autoBaud, line)` | The same on any transport. | as above | `new Port(line, "SIM1")` |
| `port.Scan(slaves, timeout)` / `ScanUnits(slaves, timeout, identify)` / `Identify(slave)` | Addresses that answer (with their FC11 identity) / one address's `DeviceInfo`. Read-only; no retry per missing address. | addresses 1..247, default 1..16 | `port.ScanUnits()` |
| `port.Axis(slave, options)` | The axis at that address (one object per address). | 1..247 | `var ax = port.Axis(3);` |
| `Cell.Open(options)` | Opens every IAI USB adapter; finds and identifies every answering axis. Read-only. | `CellOptions`: `MaxAxes` 0 (no limit; a value caps the cell), `ScanSlaves` 1..16, `BaudRate`, `ReadTimeout`, `Retries`, `Identify`, `AxisOptions`, `Line` | `using var cell = Cell.Open();` |
| `Cell.Open(portNames, slaves, options)` / `Cell.Open(slavesByPort, options)` | The named ports; listed slaves must answer. | `"COM3"`, `{ "COM5": [1], "COM6": [2, 3] }` | `Cell.Open(new[] { "COM3", "COM4" })` |
| `Cell.Open(transports, slaves, options)` / `Cell.Open(ports, slaves, options)` | A cell on open transports or `Port`s. An axis a `Port` already has is used as is, so its caps and stroke apply to group commands. | name -> transport | `Cell.Open(SimLine.SimPorts(3))` |
| `cell[name]` / `cell[index]` / `Names` / `Axes` / `Ports` / `Info` | One axis by name or index; all names, axes, ports, identities. | `"COM3:1"`, `0` ... | `cell["COM3:1"].Status()` |

### Status and monitoring

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.Status()` | `Status`: `PositionMm`, `VelocityMmS`, `ServoOn`, `Homed`, `InPosition`, `Moving`, `Alarm`, `AlarmCode`, `Warning`, `EmergencyStop`, `ModbusEnabled`, `Idle`, `Ready`. | | `axis.Status().PositionMm` |
| `axis.ReadMonitor()` | Position, velocity, motor current, deviation, alarm, uptime (`MonitorReading`). | | `axis.ReadMonitor().Current` |
| `axis.WaitIdle(target, band, timeout, cancellationToken)` | Blocks until in position and still; a timeout stops the axis. | optional target and band; `timeout` in s | `axis.WaitIdle(50.0, 0.1)` |
| `axis.ReadRegisters(address, count)` | Raw register read, for diagnostics. | any readable address | `axis.ReadRegisters(0x9000, 4)` |
| `axis.ReadLeanStatus(retries)` | One read of the 8 words 0x9000-0x9007 the monitor polls. | `null` = the port's retries | `axis.ReadLeanStatus()` |
| `axis.EnableFast(baudRate, monitor)` / `DisableFast()` | Fast completion, the status monitor, and the link raised to `baudRate` (default 230400, VERIFIED on the EC-R6S rod). The rate is kept, with the reason in `FastModeSummary.Notes`, when the axis does not own its port, the transport has no baud rate, or the axis is not at rest. `DisableFast` keeps the rate. See also `FastMode`, `FastNotes`, `Connect(takeControl, fast: true)`. | `null` or 0 keeps the rate | `axis.EnableFast();` |
| `axis.Completion` / `AxisOptions.Completion` | Default completion rule: `CompletionRule.Safe` (PEND, in the band, PNOW unchanged over `SettleS`) or `Fast` (one sample: PEND, in the band, DSSE MOVE clear). A move or `WaitIdle` can pass its own (`completion:`). | `Safe` / `Fast` | `axis.Completion = CompletionRule.Fast;` |
| `axis.StartMonitor(options)` / `StopMonitor(timeoutS)` / `Monitor` | Background status monitor: a core thread polls the lean block, with the port lock free between polls. `AxisMonitor`: `Running`, `Latest` / `Status`, `Polls`, `ErrorCount`, `PollRateHz`, `LinkChecks`, `LinkState`, `Error`, `LastError`, `IsDone(status)`, `WaitUpdate(afterSeq, timeout)`, `WaitFor(predicate, timeout, token)`, `WaitDone(timeout, token)`, `Suspend()`. `Axis.Close` stops it. | `MonitorOptions`: `GapS` 0..10 (0.002), `LinkCheckS` 0..3600 (1; `null` = never), `Retries`, `QueueCapacity` | `var mon = axis.StartMonitor();` |
| `axis.StatusChanged` / `MotionDone` / `LinkChanged` | Monitor events: PNOW / ALMC / DSS1 / DSS2 / DSSE changed (`Status`, `Previous`); done by the fast rule; link lost, unit restarted or replaced, or recovered (`RecoveryEvent`: `Kind`, `Axis`, `Message`, `Cause`, `Time`). Raised in order on a binding thread: keep handlers short and never call a blocking axis command from one. Exceptions they throw are ignored. | | `axis.MotionDone += (_, e) => Console.WriteLine(e.Status.PositionMm);` |

### Motion

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.Servo(on, wait, timeout)` | Servo on or off; waits for confirmation. | `true` / `false` | `axis.Servo(true);` |
| `axis.Home(wait, timeout, method, cancellationToken)` | Home return. Cancelling sends the decel STOP sequence. | `method`: `null`, `CommandMethod.Coil` or `.Iai` | `axis.Home();` |
| `axis.MoveTo(position, speed, accelG, band, wait, timeout, decelG, cancellationToken, accelPattern, completion)` | Absolute move (0x9900; the command-profile path when `decelG` differs from `accelG`). `accelPattern`: `AccelPattern.Trapezoid`, `SMotion` or `Filter` (UNVERIFIED on hardware). | position within the soft limits; speed, accel and decel 0.01 .. unit max; defaults 50, 0.1 G, band 0.1 | `axis.MoveTo(80.0, speed: 20);` |
| `axis.MoveBy(delta, speed, accelG, band, wait, timeout, cancellationToken, accelPattern, completion)` | Relative move from the measured position. | as `MoveTo` | `axis.MoveBy(-10.0);` |
| `axis.Jog(step, speed, ...)` | Small relative step at low speed (default 10 units/s; never the unit's JOG coils). | step 0.01 .. stroke | `axis.Jog(-5.0);` |
| `axis.Push(target, band, forcePct, direction, speed, accelG, wait, timeout, cancellationToken)` | Moves to `target`, then pushes up to `band` further. Returns `PushResult` (`Contact`, `PositionMm`, `Warnings`, `Status`). | force 1 .. unit max % (`Limits().Push`); direction +1 away from home, -1 toward | `axis.Push(85.0, band: 10.0, forcePct: 30);` |
| `axis.CheckMove(position, speed, accelG, band, decelG, accelPattern)` | Every check of `MoveTo`, without moving. | as `MoveTo` | `axis.CheckMove(120.0, speed: 100);` |
| `axis.StartMoveTo(position, speed, accelG, band, timeout, decelG, accelPattern, completion)` / `StartMoveBy(...)` / `StartJog(...)` | Starts a move after every check and returns its `MoveHandle`. `Wait(timeout, token)` / `WaitAsync(...)` block until done; a timeout or alarm sends a decel stop, and a cancelled token sends one and throws `OperationCanceledException`. `Poll(maxWaitS)` waits a slice, `IsDone()` checks once, `Cancel(wait, timeout)` is a decel stop (safe from another thread). `Info`, `Result`, `Target`, `Band`, `TimeoutS`. `Dispose` releases the handle only; the move goes on. | as `MoveTo` | `using var move = axis.StartMoveTo(80.0, speed: 20); var st = move.Wait();` |
| `axis.ChangeSpeed(speed, wait, timeout, cancellationToken)` / `ChangeAccel(accelG, wait, timeout, cancellationToken)` | Changes the speed (or acceleration = deceleration) of the running 0x9900 move. Without a running move: `EcNotReadyException`, nothing sent. With `wait`, returns the settled status, else `null`. VERIFIED in the Python edition on the EC-GRBP8M gripper (speed also on the PCON-CB); UNVERIFIED here. | 0.01 .. unit max | `axis.ChangeSpeed(100, wait: true);` |
| `axis.MoveSequence(points, speed, accelG, band, blendBand, completion, timeout, cancellationToken)` | Absolute moves through `SequencePoint`s (or positions) in order, one settled status per point. Every point is checked before the first frame. `blendBand` 0.01 .. 10 sends the next target within that distance (UNVERIFIED). | as `MoveTo`, per point | `axis.MoveSequence(new[] { 20.0, 60.0 }, speed: 50);` |
| `axis.TeachPosition(n, allowNonvolatile, viaCoils, allowUnsafe, force)` | Stores the current position as row `n` (non-volatile; an unchanged row is `Skipped`). `viaCoils` is the EXPERIMENTAL MOD / POSR / TEAC sequence. `allowNonvolatile` and `allowUnsafe` are accepted and ignored (no longer needed). | row 0 .. `MaxPositions() - 1`; an EC row its position mode does not run is written with a warning | `axis.TeachPosition(2);` |
| `axis.CancelPausedMove(timeout)` / `CancelWaits()` | Drops a paused move's remaining travel and releases the pause / ends every blocking call of this axis on other threads (a decel stop first when motion was waited for). | | `axis.CancelPausedMove();` |
| `axis.Stop(wait, timeout)` | Decel stop (STOP coil on, standstill, off). Works with or without Modbus control, from any thread. | | `axis.Stop();` |
| `axis.Pause()` / `Resume()` | Holds a move with its remaining travel / continues it. | | `axis.Pause();` |
| `axis.EnableModbusControl()` / `DisableModbusControl()` | Command authority to the PC / back to PIO. Taken automatically at rest. | | `axis.DisableModbusControl();` |
| `axis.SetActionMode(pc, safety)` / `SetSafetySpeed(on)` | PC / PIO mode and safety speed in one command / safety speed alone. | `true` / `false`; `null` keeps the unit's own | `axis.SetActionMode(true, safety: false);` |

### Stored positions

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.MaxPositions()` | Number of stored rows (3 on the ECs tested, 64 on the PCON). | | `axis.MaxPositions()` |
| `axis.ReadPositionTable(n)` | One stored row (`PositionEntry`). | row 0 .. `MaxPositions() - 1` | `axis.ReadPositionTable(1)` |
| `axis.MoveToPosition(n, speedPct, wait, timeout, allowAboveCaps, cancellationToken)` | Runs a row at its stored speed or a percentage of it (0x45 command 6). | `speedPct` 1..100; a stored speed above the caps needs `allowAboveCaps` | `axis.MoveToPosition(1, speedPct: 25);` |
| `axis.WritePositionTable(n, entry, allowNonvolatile, writeChecksum, allowUnsafe, force)` | Writes a row (permanent) and verifies it. An unchanged row is not written (`Skipped`) unless `force: true`. Counts in `axis.NonVolatileWrites`. `allowNonvolatile` and `allowUnsafe` are accepted and ignored (no longer needed). | position, push power and load threshold within the unit's limits; a row the unit does not run, unknown control-flag bits or a role-bit change is written with a warning; rows 256-267 are refused | `axis.WritePositionTable(2, row with { PositionMm = 60.0 });` |

### Parameters

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.Parameters()` / `UserParameterList()` | Named parameters (CON numbers / EC P-numbers) with value, unit, range. Read-only. | | `axis.UserParameterList()` |
| `axis.ReadParameter(key)` | One parameter. | `2`, `"P2"`, `"No.35"` or `"class.item"` | `axis.ReadParameter("P2")` |
| `axis.WriteParameter(key, value, commit, allowDangerous, allowUnlisted, allowHomeParams, force)` | Writes, verifies and saves one parameter (PARW). Applies after `SoftwareReset()` or a power cycle. A value the unit already holds is skipped (`Skipped`; `Changed` / `Committed` false), unless `force: true` or a RAM-only write from this process still waits for a commit. Counts in `axis.NonVolatileWrites`. | range from the unit; a home, dangerous or unlisted parameter is written with a warning (`Warnings`) | `axis.WriteParameter(2, 0.2);` |
| `axis.NonVolatileWrites` / `NonVolatileWritesLastHour` | Non-volatile writes this process made on this unit, in all / in the last hour, across every axis object. Skipped writes do not count. | | `axis.NonVolatileWrites` |
| `axis.ReadParameterModel()` | This unit's parameter list: the one recorded for its unit code, or every mapped item found on it. | | `axis.ReadParameterModel()` |
| `axis.ReadHomePattern()` / `ReadBootBaudRate()` | Home pattern / power-up baud rate. Read-only. | | `axis.ReadBootBaudRate()` |

No opt-in: a home direction / offset, encoder / motor type, lead / stroke configuration, servo gains, PIO, fieldbus
or baud setup, a wider soft limit, an unknown raw item or an item the unit's own list does not offer is written with a
warning. `allowDangerous`, `allowHomeParams` and `allowUnlisted` are accepted and ignored (no longer needed).

### Alarms

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.Status().Alarm` / `.AlarmCode` / `.Warning` / `.AlarmLevel` | Current alarm (blocks motion), its code, warning (does not block). | | `axis.Status().AlarmCode` |
| `axis.ResetAlarm(method)` | Clears the current alarm (never a software reset). Returns the new status. | `method`: `null`, `CommandMethod.Coil` or `.Iai` | `axis.ResetAlarm();` |
| `axis.ReadAlarmHistory(includeEmpty)` | Alarm history, newest first (`AlarmRecord`: code, name, level, time after power-on). | | `axis.ReadAlarmHistory()[0].Name` |
| `Alarms.AlarmInfoOf(code)` / `Alarms.AlarmName(code)` | Name, level, how to reset and advice for a code. No I/O. | alarm code | `Alarms.AlarmInfoOf(0x0EE)` |

### Unit and link (reset, baud, identity)

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `axis.SoftwareReset(timeoutS, resetWaitS)` | Restarts the firmware and reconnects. At rest only; the servo turns off; an incremental unit must home again. | | `axis.SoftwareReset();` |
| `axis.SetBaudRate(rate, method, settleS, allowUnverifiedCode)` | Changes the unit's and the port's baud rate, until power-off. | 115200 or 230400 (VERIFIED); 38400 is sent and checked (the EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod ignores it: `VerificationException`); another code is sent with a warning (`allowUnverifiedCode` is accepted and ignored); a rate with no code is refused | `axis.SetBaudRate(115200);` |
| `port.SetBaudRate(rate, slaves, method, settleS, allowUnverifiedCode)` | The same for every unit on a shared port (one broadcast); UNVERIFIED on hardware. | as above | `port.SetBaudRate(115200);` |
| `axis.BaudRate` | Baud rate in use (`null` on a transport without one). | | `axis.BaudRate` |
| `axis.Track(name, rows, alarmHistory, parameters, backupJson)` / `TrackBackup(json)` / `TrackBackupFile(path)` / `TrackIdentity()` / `Session` / `Forget()` | The session the axis remembers while the link is up: identity, baud rate, PC control, safety speed, HEND, uptime, and optionally alarm history, rows, parameters and a backup. Recovery uses it to confirm the same unit; without one, the identity from `Connect()` is the baseline. Read-only. | `rows`: `null`, `"all"`, `"0,1,5"` | `axis.Track("line 1");` |
| `axis.CheckLink()` / `LinkState` | One status read compared with the session: `ok`, `port_vanished`, `unit_silent`, `unit_restarted` (with `Reasons`), `recovering`; a change also raises `LinkChanged` / the last known state, no I/O. Read-only. | | `axis.CheckLink().State` |
| `axis.Recover(timeout, policy, name, cancellationToken)` | Brings the axis back after a power or link loss: waits for the port, reopens it, finds the baud rate, requires the same unit (else `UnitReplacedException`, nothing written), restores baud rate, PC control and safety speed, resets a message alarm, and reports (`RecoveryReport`: `Restarted`, `BaudRestored`, `ControlRestored`, `NeedsHome`, `AtTarget`, `ReadyToMove`, `NeedsAttention`, `Actions`, `Notes` ...). **Never moves the axis.** On failure `RecoveryException.Report` says what was done. VERIFIED in the Python edition on the EC-GRBP8M gripper and the PCON-CB; UNVERIFIED here. `NoteTarget(mm)` / `NoteControl(...)` record a deliberate change. | `RecoveryPolicy`: `RestoreBaudRate`, `RestoreControl`, `RestoreSafetySpeed`, `ClearMessageAlarms`, `CheckRows`, `CheckParameters`, `BackupJson` / `BackupFile`, `PositionToleranceMm`, `PollS`, `BootBaudRate`, `RequireSession`; `timeout` default 30 s | `var report = axis.Recover(60);` |
| `axis.RunCheck(options, onItem, cancellationToken)` | The compatibility self-test (`iai-check`): link, slave id, index table, identification, soft limits, speed limit, status, alarm and history, position table, direct-move block, Modbus control, safety speed, unit information, parameter ranges. Read-only unless `Motion` (a short move out and back). A unit that answers differently is a `Fail` line, never an exception. `onItem` sees each line on the calling thread. `CheckReport.Passed`, `.Items`, `.ToJson()`, `.WriteJson(path)`. | `CheckOptions`: `Motion`, `StepMm` (5), `SpeedMmS` (10), `AccelG` (0.1), `PortLabel` | `axis.RunCheck(null, item => Console.WriteLine(item.ToLine()));` |
| `axis.LineName` / `StopPath` / `RecommendedMinSpeed()` | The line (`null` = direct) / how `Stop` stops it / the documented minimum speed and its source. | | `axis.StopPath` |
| `DeviceInfo` (from `Connect()` / `Identify()`) | `ProductFamily`, `Model`, `Controller`, `Kind`, `SerialNumber`, `Firmware`, `AxisNumber`, `Units`, `SoftLimitsMm`, `MaxSpeedMmS`, `Errors`. | | `info.ProductFamily` |

### Caps, timing and read-only state

Settable properties can change after the axis is open. Read-only ones send no frame unless stated.

| Property or function | What it does | Default / range | Example |
|---|---|---|---|
| `axis.MaxSpeedMmS` / `MaxAccelG` / `MaxPushPct` / `Stroke` | Configured caps and stroke (from `AxisOptions`; settable). They can only narrow the unit's own range. | `null` = the unit's own | `axis.MaxSpeedMmS = 100;` |
| `axis.SpeedCapMmS` / `AccelCapG` / `AccelOnlyCapG` / `DecelCapG` / `PushCapPct` | The highest value the driver commands. Speed: the lowest of `MaxSpeedMmS` and MAXV (7.4). Acceleration (= deceleration on a plain move): the lowest of `MaxAccelG`, MAXA and MAXD; with a deceleration of its own, `AccelOnlyCapG` (MAXA) and `DecelCapG` (MAXD). Push: the lowest of `MaxPushPct` and MXPP (7.9). A term counts only when known; while none is, `Axis.FallbackSpeedCap`, `FallbackAccelCapG` and `FallbackPushCapPct` apply. | fallbacks 100 units/s, 0.30 G, 70 % | `axis.SpeedCapMmS` |
| `axis.EffectiveStroke` / `UnitStroke` / `SoftLimitsMm` / `SoftLimitsNote` | The range targets are checked against (configured stroke and the unit's own, intersected; `null` while unknown or when they do not overlap) / the soft limits minus the 0.30 mm margin / the soft limits as read / their source. No I/O. | `(Min, Max)` | `axis.EffectiveStroke` |
| `axis.ReadSoftLimits()` / `ReadSpeedLimit()` / `ReadAxisLimits()` | Reads the soft limits / MAXV (7.4) / class 7 limits (MAXV, MAXA, MAXD, MXPH, MXPP, MNPP ...). FC03; motion calls do it once by themselves. | | `axis.ReadSoftLimits()` |
| `axis.UnitMaxSpeedMmS` / `UnitMaxAccelG` / `UnitMaxDecelG` / `UnitMaxSpeedNote` | The unit's MAXV, MAXA, MAXD once read, and the speed's source. | | `axis.UnitMaxAccelG` |
| `axis.ReadSupportWords()` / `Categories` | Class 1 support-function words by name (SUPT, DIAG, MVPT ...; `null` when unreadable) / the directory records read so far. | | `axis.ReadSupportWords()` |
| `axis.SystemCode` / `UnitCode` / `Slave` / `BaudDetected` / `Strategy` | FC11 codes / address / whether the baud rate was probed / how servo, home and alarm reset are sent (moves always use 0x9900, stops the STOP coil). | | `axis.Slave` |
| `axis.AutoModbusControl` | Take Modbus control automatically at rest (settable). With `false`, such commands throw `EcNotReadyException` until `EnableModbusControl()`. | `true` | `axis.AutoModbusControl = false;` |
| `axis.SettleS` / `PushSettleS` / `PollIntervalS` | How long PNOW must stay unchanged for done / a push outcome must hold / the polling period while waiting (settable). | 0.1 s / 0.15 s / 0.02 s | `axis.SettleS = 0.2;` |
| `axis.FinishHome()` | Releases the HOME coil after `Home(wait: false)` (coil strategy). | | `axis.FinishHome();` |
| `axis.WriteUserParameter(no, value, ...)` / `Axis.WritableParameterClasses` | Writes, verifies and saves EC user parameter P`no` (only parameters confirmed on hardware; same guard as `WriteParameter`) / the classes a raw `"class.item"` write may touch (7, 8, 9, 0x0C). | P number from `UserParameterList()` | `axis.WriteUserParameter(2, 0.2)` |
| `Axis.StoredStartS` / `PushCreepMm` / `MoveBandMax` / `ResetWaitS` / `ResetTimeoutS` / `BreakS` / `BaudSettleS` / `RecoverTimeoutS` / `FastBaudRate` | Constants: 0.5 s for a stored-row move to start / 0.05 mm push creep / largest band 10 / 1 s wait and 10 s give-up after a software reset / 0.2 s line break / 0.2 s after a broadcast baud change / 30 s longest recovery wait / 230400 fast-mode rate. | | `Axis.RecoverTimeoutS` |
| `axis.NvWriteCycles` / `NvWarnPerHour` / `NvWindowS` / `NvSkipNote` | About 100,000 write cycles (IAI) / more than 60 writes in the last hour on one axis give one WARNING per hour, never a refusal / the 3600 s window / the note of a skipped write. | | `Axis.NvWarnPerHour` |
| `axis.UnitInfoSections` / `Axis.UnitInfoExtraSections` / `Axis.UnitInfoSchema` | The sections `UnitInfo()` reads, this edition's extra one (`limits`) and the result's schema id. | | `Axis.UnitInfoSections` |

### Multi-axis group commands (`Cell`)

Ports run in parallel; the axes of one port take turns on its lock. Every `*All` method takes `axes` (`null` = all)
and `returnExceptions` (default `false`). On any failure it handles every axis, then throws `CellException` (`Errors`
and `Results` per axis), or with `returnExceptions: true` returns the failures in `CellResults<T>.Errors`.

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `cell.IdentifyAll()` | Identifies every axis (kept in `cell.Info`). | | `cell.IdentifyAll();` |
| `cell.StatusAll()` | Status of every axis, by name (`CellResults<Status>`). | | `cell.StatusAll()["COM3:1"].PositionMm` |
| `cell.LimitsAll()` / `MonitorAll()` | Each axis's ranges, units and kind / monitor values. | | `cell.LimitsAll()` |
| `cell.UnitsOf(name)` | `"mm"` or `"deg"` for one axis. | axis name | `cell.UnitsOf("COM3:1")` |
| `cell.ServoAll(on, axes, timeout, returnExceptions)` | Servo on or off on every axis. | `true` / `false` | `cell.ServoAll(true);` |
| `cell.HomeAll(axes, timeout, method, stopOthersOnError, returnExceptions, cancellationToken)` | Homes every axis at once and waits for all. | as `Home` | `cell.HomeAll();` |
| `cell.MoveAll(targets, speed, accelG, band, decelG, wait, ...)` / `MoveAll(moves, wait, ...)` | Checks every move first (one bad value refuses all, nothing started), starts them together, waits for all. | `{ axis: position }` with one speed / accel / band, or one `AxisMove(axis, position) { Speed, AccelG, Band, DecelG }` per axis | `cell.MoveAll(new Dictionary<string, double> { ["COM3:1"] = 50.0 });` |
| `cell.WaitAll(axes, timeout, ...)` | Waits until every axis is in position (after `MoveAll(..., wait: false)`). | | `cell.WaitAll();` |
| `cell.StopAll(axes, timeout, returnExceptions)` | Writes STOP to every axis first, then waits for each to stop. | | `cell.StopAll();` |
| `cell.NameOf(axis)` / `PortOf(name)` / `Ports` / `PortErrors` / `Count` / `Contains(name)` / `IsClosed` | Name of an axis object / its `Port` / the ports / ports that failed to open and why / count / membership / closed. | | `cell.PortOf("COM3:1")` |
| `cell.Close()` / `Dispose()` | Releases STOP coils left on and closes every port. | | `cell.Dispose();` |

If one axis fails during `MoveAll`, `HomeAll` or `WaitAll`, the others are stopped too. Cancelling the
`CancellationToken` stops every axis still waiting.

### Simulation (`IaiActuatorDriver.Simulation`)

| Type | What it does | Example |
|---|---|---|
| `SimulatedUnit` | A byte-level simulated unit, usable as an `ITransport`. Profiles (`SimProfile`): `Slider`, `Gripper`, `Rod`, `Pcon`, `Rotary` (deg), `Unknown`. Test hooks: `PositionMm`, `Moving`, `ServoOn`, `TriggerAlarm`, `InjectFault`, `PowerCycle`, `Requests`. | `new SimulatedUnit(new SimulatedUnitOptions { Profile = SimProfile.Gripper, TimeScale = 10 })` |
| `SimLine` | One simulated line carrying several units by address (only the addressed unit answers; address 0 reaches all). | `SimLine.WithSlaves(new[] { 1, 2, 3 })` |
| `SimLine.SimPorts(n, slaves, options, prefix)` | `n` simulated ports "SIM1" .. "SIMn", one unit each by default. | `Cell.Open(SimLine.SimPorts(3))` |

The simulator follows the hardware tests and public manuals, and makes a simple choice where they are silent. Passing
tests against it shows the driver agrees with what was observed on hardware.

### Errors

| Exception | When |
|---|---|
| `SoftLimitException` | A target, speed, accel or force is outside the unit's range. Nothing was sent. |
| `EcNotReadyException` | The axis is not ready (servo off, not homed, moving ...). |
| `EcAlarmException` | The unit has an alarm. |
| `MotionTimeoutException` | A move or home did not finish in time (the axis is stopped). |
| `ModbusTimeoutException` | No answer: check power, cable, address, and that IA-OS is closed. |
| `UnsafeCommandException` | A refused command or write: an IAI command off the safe list without `allowUnsafe`, one on the never-send list, a read-only or unconfirmed parameter. |
| `CellException` | One or more axes failed in a group command (details per axis). |

All derive from `EcDriverException`. The full Python-to-C# mapping is at the top of `Errors.cs` (Python
`ModbusException` is `ModbusSlaveException`).

### Python names

| Python edition | C# edition |
|---|---|
| `Axis`, `MoveHandle`, `AxisMonitor` | `Axis`, `MoveHandle`, `AxisMonitor`, with records such as `AxisOptions`, `AxisRanges`, `PositionEntry`, `PushResult`, `MonitorSample`, `RecoveryReport`, `CheckReport` |
| `cell.py` (`list_ports`, `scan`, `Port`, `Cell`) | `PortManager` (`ListPorts`, `Scan`), `Port`, `Cell`, `CellOptions`, `AxisMove`, `CellResults<T>` |
| `ec.Status`, `ec.DeviceInfo`, `identify()` | `Status.FromRegisters`, `DeviceInfo`, `SlaveIdentity`, `DeviceIdentity.Read` |
| `info.read_directory` and the block readers | `UnitDirectory`, `ClassBlock`, `ClassItem` |
| `alarms.py`, `registers.py`, `con_params.py`, `info` user parameters | `Alarms`, `Registers` (IAI mnemonics keep their names, such as `Registers.DSS1_SV`), `ConParameters`, `UserParameters` |
| `modbus.ModbusRTU`, `Transport`, `crc16` | `ModbusRtu` (FC03 / 05 / 06 / 10 / 11 / 17, IAI 0x45 incl. broadcast, 0x48, retries, one lock per port, `Probe`, `DetectBaudRate`), `ITransport`, `SerialTransport`, `FakeTransport`, `Crc16` |
| `sim.py`, `sim_line.py` | `SimulatedUnit`, `SimLine` |
| `iai-actuator-driver` | `iai-actuator-driver-cs` |

## Command line

`iai-actuator-driver-cs (--port NAME | --sim PROFILE) [--slave N | --axis N] [options] COMMAND [args]`, plus the port
commands `ports` and `scan`. `--help` prints everything. The tool is not a safety function.

| Command | What it does |
|---|---|
| `ports [--json]` | Serial ports, IAI USB adapters first. Opens nothing. |
| `scan [--ports COM3,COM4 \| --sim PROFILE] [--slaves 1-16] [--baud RATE] [--scan-timeout S]` | Axes that answer on each port (default: every IAI adapter). Read-only. |
| `status` / `identify` / `limits` / `alarms` | Status bits / identity and directory / every enforced range / alarm history. Read-only. |
| `info [--section S]...` / `monitor [--count N] [--interval S]` | Everything the unit reports / live position, velocity, current. Read-only. |
| `monitor --watch [--interval S] [--duration S]` | The status monitor with the link watch: a line per status change and per link event until Ctrl+C or `--duration`, then a summary (`--json`: one object per line). Read-only; exit code 2 when it ended on an error (run `recover`). |
| `servo on\|off` | Servo on / off (on also takes Modbus control at rest). |
| `home [--method coil\|iai]` | Home return and wait. |
| `move POS [--speed V] [--accel G] [--decel G] [--band B] [--no-wait] [--fast] [--change-speed V] [--change-accel G] [--change-after S]` | Absolute move (default 20 units/s, 0.1 G). `--decel` uses the command-profile move; `--fast` the fast completion (the rate is kept); `--change-speed` / `--change-accel` change the move after `--change-after` s (default 1). |
| `jog DELTA [--speed V]` / `push POS BAND PCT [--toward-home] [--speed V]` | Relative step / approach, then push at PCT % force. |
| `goto N [--speed-pct P] [--allow-fast] [--no-wait]` | Runs stored row N at its stored speed or P % of it. |
| `stop` | Decel stop (works without Modbus control). |
| `pause` / `resume` / `cancel-pause` | Hold the move (the pause stays set after the command) / release it / drop the rest and release. |
| `teach N [--force] [--method table\|coil]` | Stores the current position as row N (non-volatile); an unchanged row is not written unless `--force`. `--allow-nonvolatile` is still accepted and ignored. The row's stored speed must fit the caps (`--unit-max` lifts them). |
| `alarm-reset [--method coil\|iai]` | Alarm reset (`reset` is a deprecated alias; the software reset is `swreset`). |
| `param list` / `param get KEY` / `param set KEY VALUE [--no-commit] [--force]` | Decoded parameters / one parameter (read-only) / write, verify and commit. A home, dangerous or unlisted parameter is written with a warning (`--allow-dangerous`, `--allow-home-params` and `--allow-unlisted` are accepted and ignored). An unchanged value is not written; `--force` commits it, which is needed for a value an earlier `--no-commit` run left in RAM. |
| `mode pc\|pio [--safety]`, `swreset`, `baud RATE` | Action mode, software reset, baud-rate change. |
| `check [--motion] [--report FILE]` | The self-test: one line per check, a JSON report (default `iai-check-<time>.json`), `RESULT PASS` / `FAIL`; exit code 1 on a failure. Read-only unless `--motion` (a short move out and back, up to 5 mm). |
| `backup [FILE] [--rows used\|all\|0,1,2]` | Read-only JSON backup (FC03 / FC11 only); default file `ec-backup-<port>-s<slave>-<time>.json`. |
| `recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]` | Brings the axis back after a power or link loss and prints what changed. `--restore-baud` switches back to 115200 or 230400 (a broadcast), `--no-control` leaves PC control alone, `--wait` defaults to 30 s. Never moves the axis. |

Options: `--port NAME`, `--sim PROFILE` (`slider`, `gripper`, `rod`, `pcon`, `rotary`, `unknown`; real time),
`--slave N` / `--axis N`, `--baud RATE`, `--timeout S`, `--retries N` (default 3, IAI's recommendation),
`--strategy coil|iai|iai2`, `--stroke MIN MAX` (narrows the soft limits, never widens), `--max-speed V` /
`--max-accel G` / `--max-push PCT` (default caps 100 units/s, 0.3 G, 70 %, always lowered to the unit's maxima),
`--unit-max` (lift the caps to the unit's maxima), `--log-frames`, `--json`. Every number is checked before a port is
opened. Ctrl+C during a move or home sends the decel STOP sequence and exits with 130.

## Guards and safety

**This driver is not a safety function:** keep a hardware emergency stop. The unit has no communication watchdog: if
the PC goes silent, a move in progress is completed.

- Every range comes from the unit (soft limits 7.1 / 7.2, MAXV 7.4, MAXA / MAXD 7.5 / 7.6, MXPP 7.9, MNPP 7.58, AKID
  7.33 for mm / deg, MAXP 3.2), with no model whitelist. `AxisOptions` caps can only narrow them.
- Anything out of range raises `SoftLimitException` before a frame is sent.
- Modbus control is taken only at rest. Completion needs PEND and a settled position; a push outcome must hold.
- Every error, timeout, alarm or cancellation during motion sends the STOP coil sequence (on, standstill, off).
- Non-volatile and dangerous writes need no opt-in: a risky one goes through with a warning. `allowNonvolatile`,
  `allowDangerous`, `allowHomeParams`, `allowUnlisted` and `allowUnverifiedCode` are accepted and ignored. Unchanged
  values are not written, and the writes are counted with a warning above 60 an hour.
- Thread safety: every transaction holds the port's lock. `Stop` and `Pause` may be called from another thread while a
  move waits; waiting calls take a `CancellationToken` (cancelling stops the axis).

### Non-volatile writes

The non-volatile memory lasts about 100,000 write cycles (IAI; the same nominal limit IAI gives for the ROBONET axis
units, DOC ME0208-14B p.190, 367; the CB-series CON manuals state no limit, DOC ME0342-4H p.2-11). For a target that
changes every cycle, use `MoveTo`, not a stored row or a parameter.

- **Writes it:** a parameter commit (PARW coil 0x0428, VERIFIED on the EC and the PCON) and a stored-row write
  (`WritePositionTable`, VERIFIED: it survived a power cycle; `TeachPosition` is a row write too).
- **RAM only:** direct moves (`MoveTo`, the 0x9900 block, coordinated motion; VERIFIED); stored-row moves (they only
  read the row); the PMSL and SFTY coils, the action mode (0x45 command 0x0C) and the baud change (back to PMSS 0,
  safety speed off and 38400 after every power-up, VERIFIED); servo, alarm reset, software reset and every read. The
  command profile 0x0F00 of a move with its own deceleration shows the last move's profile and is rewritten by every
  move (UNVERIFIED as RAM).
- **Written by the unit itself:** its alarm history (every alarm and a 0x0FFF marker at power-up, VERIFIED) and, on
  an absolute unit, possibly its home data at a home return (UNVERIFIED). Avoid provoking alarms in a loop.

The guard writes nothing when the value or row would not change (`Skipped`; `force: true` writes anyway). It compares
with the unit's RAM, so a RAM-only write by another program or process (such as an earlier CLI `--no-commit` run) is
not seen: commit it with `force: true`. A parameter this process wrote to RAM (`commit: false`, or a commit that failed
or was not confirmed) stays pending, and the next commit saves it. A confirmed software reset drops the pending items.
Pending items, write counts (`axis.NonVolatileWrites`) and the rate WARNING are kept per unit for the whole process,
so a reconnect or a new axis object keeps them. More than 60 writes in the last hour on one axis
(`Axis.NvWarnPerHour`) log one WARNING per hour; a write is never refused for the rate. A CON unit's power-up baud
rate is a parameter, guarded like any other. The command line has no row-write command; `param set` takes `--force`.

## Native core (interop layer)

The library runs on the C++ core through its C ABI 1.0 (`iai_actuator_driver.dll`, header
`core/include/iai_actuator_driver.h`,
[docs/NATIVE_ABI.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/NATIVE_ABI.md)). The binding is an internal
layer (`src/IaiActuatorDriver/Interop/`, namespace `IaiActuatorDriver.Interop`); write applications against the
public classes above.

- `ModbusRtu`, `Axis`, `Port`, `Cell` and `SimulatedUnit` are the core's bus, axis, port, cell and simulator. Reads,
  guards, waits, retries, stops and the non-volatile accounting run in the core. A `CancellationToken` sets the core's
  cancel latch (a decel stop first when motion was waited for).
- Managed code covers the records, `Status.FromRegisters`, the `Registers` / `Alarms` / `ConParameters` /
  `UserParameters` tables, `Crc16`, the `UnitDirectory` parsers, `FakeTransport` and the `CommandStrategy` helpers.
- A transport the core does not know (`FakeTransport`, your own `ITransport`) reaches it through callbacks. An I/O
  error comes back as `ModbusException` "transport error" with the original as inner exception; other exceptions come
  back as thrown, also in a `Cell`'s per-axis `Errors`. A transport's members must not call the driver: the core holds
  the bus lock while they run. `SerialTransport` is the core's own serial port.
- An override of `SimulatedUnit`'s protected members runs in the core's request hook; one that throws fails the write
  with its own exception.
- Core errors throw this library's exception classes, with `Notes`, `StopAttempted` and `Attempts` filled in. The
  native details (code, C++ class, last-error JSON) are in `Exception.Data`.
- The core's log lines reach `DriverLog`; a `ModbusRtu` frame logger sees the core's frames.
- Public for diagnostics, in `IaiActuatorDriver.Native`: `NativeDriver` (ABI check: the same `AbiMajor` and at least
  `AbiMinor`; library version; port list as JSON; log handler; `OverrideVariable` / `OverridePath`), `NativeErrors`
  (`ResultOf`, `ErrorJsonOf`, `Name`) and `NativeResult`.

```csharp
using IaiActuatorDriver.Native;

NativeDriver.EnsureLoaded();                      // DllNotFoundException without the DLL, NotSupportedException for another ABI
Console.WriteLine(NativeDriver.LibraryVersion);   // the core's version, e.g. 1.0.0
Console.WriteLine(NativeDriver.ListPortsJson());  // serial ports, IAI USB adapters first (opens nothing)
```

**Where the DLL comes from.** The library project copies `core/build/iai_actuator_driver.dll` next to every program
that references it; `-p:NativeLibraryPath=<file>` takes another file, and `-p:IadNativeOptional=true` builds without
it. At run time, `IAI_ACTUATOR_DRIVER_NATIVE` names one file to load instead of the default search. A missing library
is a `DllNotFoundException` naming the runtime identifier, the places searched and the ABI needed; there is no managed
fallback. A 32-bit process gets `PlatformNotSupportedException` (no 32-bit build). A function missing from a DLL that
loads is a failure. The NuGet package `IaiActuatorDriver` carries the core under `runtimes/<rid>/native/`, with copy
targets for .NET Framework applications.

## Evidence

**VERIFIED** (confirmed on hardware), **ON-WIRE** (observed serial traffic), **DOC** (public IAI manuals; "DOC (CON
manuals)" = documented for CON-family controllers only), **UNVERIFIED** (expected, not yet confirmed on hardware).

- VERIFIED in this edition: the command line on the EC-GRBP8M gripper and on the PCON-CB with an RCP6, the public
  `Axis` on the EC-GRBP8M gripper, and the interop layer on both units. On 2026-10-31 the command line, built from
  the 1.0.0 source and running on the released native library, passed its steps on the gripper again. The managed
  library of this NuGet package was not itself run on hardware.
- UNVERIFIED in this edition (tested on the simulator and on frames recorded from real units): `Cell`, a push that
  makes contact, position-table writes, fast mode, the monitor's events, the speed change, move handles,
  acceleration patterns, `CheckLink`, recovery after a fault, and the core's other calls.

Which steps ran on which unit:
[docs/COMPATIBILITY.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/COMPATIBILITY.md).

## State of this edition

The public `Axis` mirrors the Python edition's `Axis`, and the command line has the same commands. Not public here
(the core and the Python edition have them; the core's functions are bound internally):

- `Cell`: group monitors and the link monitor, `track_all` / `check_links` / `recover_all`, group backup,
  `pause_all` / `resume_all`, the gateway calls, a stop on every axis with an optional broadcast, and coordinated
  motion.
- `Axis`: `MotionPending`, `StatusWords`, `DirectoryAbsent`, `PushSpeedMmS()`, `ServoOnTimeout()`, `BeginStop()`.
- The command line: `--line`, `--position-unit`, the `line` command and `stop-all`.

By design, a cell is addressed by axis name (`"COM3:1"`) or index, the simulator profile is `--sim PROFILE` (Python:
`--sim` plus `--sim-profile`), and group commands return `CellResults<T>`. Every unit is driven the same way,
whatever its model; no unit is labelled or refused by its model.
