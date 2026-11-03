# Getting started: C# and .NET

The .NET library (folder `bindings/dotnet/`) drives ELECYLINDER units and CON-family controllers over serial Modbus
RTU. It runs on the C++ core. Unofficial: not an IAI product ([README](../../README.md)).

Every method with its inputs and an example is in the API table of
[bindings/dotnet/README.md](../../bindings/dotnet/README.md).

## Projects

| Project | Target | Contents |
|---|---|---|
| `src/IaiActuatorDriver` | netstandard2.0 and net8.0 (.NET Framework 4.6.2+, .NET 6, .NET 8) | `Axis`, `Cell`, `Port`, `PortManager`, the exceptions, the simulator (`IaiActuatorDriver.Simulation`) and the core's binding (`Interop/`) |
| `src/IaiActuatorDriver.Cli` | net8.0 | the command line `iai-actuator-driver-cs` |
| `tests/IaiActuatorDriver.Tests` | net8.0, xUnit | simulator and recorded frames only; no test opens a serial port |

## The library and the core

Write applications against `Axis` and `Cell`. `Axis` has the features of the Python `Axis` under C# names, with
`CancellationToken` and `WaitAsync` for waiting, C# events and records for results. The core's binding is an internal
layer (`IaiActuatorDriver.Interop`) of the same assembly and NuGet package. For diagnostics,
`IaiActuatorDriver.Native` has `NativeDriver` (ABI check, library version, port list, the core's log handler),
`NativeErrors` and `NativeResult`.

## Install

**From a release.** The NuGet package `IaiActuatorDriver` carries the core (win-x64, linux-x64). Add it from the
folder you downloaded it to:

```shell
dotnet add package IaiActuatorDriver --version 1.1.0 --source <folder with the .nupkg>
```

**From source.** You need the .NET 8 SDK and the core's library. The library project copies it next to every program
that references it. On Windows x64 (`pip install ziglang` once):

```shell
cd core
python build.py shared        # core/build/iai_actuator_driver.dll + .lib
cd ../bindings/dotnet
dotnet build
dotnet test                   # simulator only; no test opens a serial port
```

- Without the library the build stops with a message that names this command.
- `-p:NativeLibraryPath=<file>` takes a library built elsewhere.
- `-p:IadNativeOptional=true` builds anyway; the program then fails at run time with a load error that names the
  places searched.
- The interop tests alone: `dotnet test tests/IaiActuatorDriver.Tests --filter "FullyQualifiedName~Interop"`.

In your own project, reference the library project (the native library comes along):

```xml
<ItemGroup>
  <ProjectReference Include="..\iai-actuator-driver\bindings\dotnet\src\IaiActuatorDriver\IaiActuatorDriver.csproj" />
</ItemGroup>
```

## A simulated axis

`SimulatedUnit` answers frames like a unit, so the code below runs without hardware and opens no serial port.
`timeScale` runs the simulated clock faster (20 = twenty times real time).

```csharp
using IaiActuatorDriver;
using IaiActuatorDriver.Simulation;

using var axis = Axis.Open(new SimulatedUnit(SimProfile.Gripper, timeScale: 20));
DeviceInfo info = axis.Connect();          // identity and the unit's own ranges; Modbus control taken at rest
Console.WriteLine($"{info.Model} ({axis.Kind}), position {axis.Limits().Position.Min} .. {axis.Limits().Position.Max} {axis.Units}");
```

| Profile (`SimProfile` / string) | Simulates |
|---|---|
| `Slider` / `"slider"` (default) | the EC-DS4H slider |
| `Gripper` / `"gripper"` | the EC-GRBP8M gripper |
| `Rod` / `"rod"` | the EC-R6S rod |
| `Pcon` / `"pcon"` | the PCON-CB with an RCP6 |
| `Rotary`, `Unknown` | made-up units for the generic paths (deg units; a unit the driver has no name for) |

The real profiles serve register blocks read from those units. A pass on the simulator shows the driver agrees with
what was observed on them, not that every unit behaves this way.

On hardware: `Axis.Open("COM3")`. The baud rate is found in place. Close IA-OS first: only one program can use a
serial port.

## First move

```csharp
using IaiActuatorDriver;
using IaiActuatorDriver.Simulation;

using var axis = Axis.Open(new SimulatedUnit(SimProfile.Gripper, timeScale: 20));
axis.Connect();
axis.Servo(true);                          // refused with the reasons when the status shows it cannot come on
axis.Home();                               // moves to the mechanical end: keep the travel clear
Status? st = axis.MoveTo(5.0, speed: 20);  // absolute; waits until in position and settled
Console.WriteLine($"at {st!.PositionMm:F2} {axis.Units}");
axis.MoveBy(-2.0, speed: 20);              // relative to the measured position
axis.Servo(false);
```

- Positions are in mm (deg on a rotary axis), speeds in units/s, accelerations in G.
- Every target, speed, acceleration and band is checked against the unit's own ranges first. A value outside them
  throws `SoftLimitException` and nothing moves.
- The caps default to the unit's own maxima. Narrow them for a first test on hardware:
  `Axis.Open("COM3", options: new AxisOptions { MaxSpeedMmS = 100, MaxAccelG = 0.3 })`. A cap can only narrow the
  unit's range.
- `wait: false` returns at once (`null`); then call `WaitIdle()`.
- An incremental unit must be homed after every power-up and software reset.

On hardware: unloaded, mid-stroke, 20 mm/s or less, a hand on the emergency stop. A vertical axis without a brake can
fall when the servo goes off. Complete programs: [examples/csharp/](../../examples/csharp/) (`FirstMove`,
`MultiAxisCell`).

## Stop

`axis.Stop()` is a decelerate-stop over Modbus. It holds the STOP coil until the axis stands still, then releases it.
It works with or without Modbus control and blocks for the deceleration. With `wait: true` it throws
`MotionTimeoutException` if the axis still moves after `timeout` (default 5 s).

```csharp
using var axis = Axis.Open(new SimulatedUnit(SimProfile.Slider));   // real time: still moving at the stop
axis.Connect();
axis.Servo(true);
axis.Home();

var move = Task.Run(() => axis.MoveTo(140.0, speed: 20));
Thread.Sleep(1000);
axis.Stop();                               // from this thread while MoveTo waits in the task
try
{
    move.Wait();
}
catch (AggregateException e) when (e.InnerException is MotionIncompleteException)
{
    Console.WriteLine("move ended: stopped away from its target");
}
```

- `Pause()` holds the move and keeps the remaining travel. `Resume()` continues it. `CancelPausedMove()` drops it.
- `Dispose` releases a STOP or pause the axis still holds, and closes the port when the axis owns it.
- The waiting calls (`MoveTo`, `Home`, `WaitIdle`, `HomeAll`...) take a `CancellationToken`. Cancelling sends the
  decel stop sequence.
- STOP is VERIFIED on moves the PC commanded and UNVERIFIED on a move a PLC commanded over PIO. Stop those from the
  PLC or with the hardware stop.
- Not a safety function. There is no communication watchdog: if the PC or the cable fails, a move in progress is
  completed.

## Moves you do not wait for

`MoveTo(..., wait: false)` returns at once. To keep a handle on the move, start it with `StartMoveTo`, `StartMoveBy`
or `StartJog`. When the call returns, every check has passed and the command is sent.

```csharp
using var move = axis.StartMoveTo(80.0, speed: 20);
Status? st = null;
while (st is null)
{
    st = move.Poll(0.05);                  // waits a slice; the move runs on between two polls
    // ... other work: read a sensor, update the screen ...
}
// or: Status done = move.Wait(timeout: 30, cancellationToken: token);   Task<Status> t = move.WaitAsync(30);
// move.Cancel() is a decel stop, safe from another thread while a wait runs; move.Info has target, band and result
```

- A timeout or an alarm during `Wait` sends a decel stop and throws. A cancelled token sends one and throws
  `OperationCanceledException`. `Dispose` releases the handle only: the move goes on.
- `axis.ChangeSpeed(100)` and `axis.ChangeAccel(0.2)` change the running move. Without a running move they throw
  `EcNotReadyException` and send nothing. With `wait: true` they return the settled status.
- A move takes `accelPattern` (`AccelPattern.SMotion` or `Filter`, UNVERIFIED on hardware), its own deceleration
  (`decelG`) and its own `completion` (`CompletionRule.Safe` or `Fast`).
- `MoveSequence` runs several absolute targets in turn. Every point is checked before the first frame.

## Fast mode, the status monitor and events

`axis.EnableFast()` turns on three things: the fast completion rule (one sample: in position, in the band, no motion
bit), the status monitor and, when this axis owns its port, 230400 baud. When it cannot raise the rate (no baud rate
on the transport, the axis not at rest) it keeps the rate and says why in `FastModeSummary.Notes`.
`axis.DisableFast()` undoes it.

```csharp
using var axis = Axis.Open("COM3");
axis.Connect(takeControl: true, fast: true);       // or axis.EnableFast();

axis.StatusChanged += (_, e) => Console.WriteLine($"{e.Previous?.PositionMm} -> {e.Status.PositionMm}");
axis.MotionDone += (_, e) => Console.WriteLine($"arrived at {e.Status.PositionMm}");
axis.LinkChanged += (_, e) => Console.WriteLine($"{e.Kind}: {e.Message}");   // link_lost, unit_restarted, recovered ...

axis.MoveTo(50.0, speed: 20);                      // waits on the monitor's samples
AxisMonitor monitor = axis.Monitor!;
Console.WriteLine($"{monitor.Polls} polls, {monitor.PollRateHz} /s, link {monitor.LinkState}");
```

- `axis.StartMonitor(new MonitorOptions { GapS = 0.002, LinkCheckS = 1.0 })` starts the monitor alone.
  `StopMonitor()` and `Close()` stop it. It only reads, and frees the port lock between polls, so a STOP or another
  axis of the port gets the line at once.
- `AxisMonitor` also waits: `WaitUpdate`, `WaitFor(predicate)`, `WaitDone`. `Suspend()` holds it (a baud change, a
  software reset and a recovery hold it themselves).
- Events are raised on a thread the binding owns, one at a time and in order. Keep a handler short and never call a
  blocking axis command from it. An exception it throws is ignored.
- The monitor never recovers anything: when the link fails it raises `LinkChanged` and ends. Call `Recover()`.

## Backup, self-test and recovery

```csharp
IDictionary<string, object?> backup = axis.Backup("unit.json");      // read-only: FC03 and FC11 only
Axis.BackupAll(new[] { axisA, axisB }, "backups", names: new[] { "left", "right" });

CheckReport report = axis.RunCheck(new CheckOptions { Motion = false }, item => Console.WriteLine(item.ToLine()));
Console.WriteLine(report.Passed ? "PASS" : "FAIL");
report.WriteJson("check.json");

axis.Track("line 1");                      // remember identity, baud rate, PC control, HEND, uptime ...
LinkCheck check = axis.CheckLink();        // ok, port_vanished, unit_silent, unit_restarted
if (!check.Ok)
{
    RecoveryReport r = axis.Recover(timeout: 60);   // never moves the axis
    Console.WriteLine(r.Summary);
    if (r.NeedsHome) axis.Home();          // your decision
}
```

- `Backup` returns a dictionary in the Python `ec_driver.backup/1` layout; `BackupJson` gives the text. Pass a backup
  of the same unit to `Recover` (`RecoveryPolicy.BackupFile`) to compare the stored rows and parameters with it.
- `RunCheck` is the Python `iai-check`: read-only unless `Motion`. A unit that answers differently gives a `Fail` line,
  never an exception. `onItem` runs on the calling thread while the test runs on a helper.
- `Recover` waits for the port and the unit (default 30 s), finds the baud rate and requires the same unit
  (`UnitReplacedException` otherwise, nothing written). It takes PC control and the baud rate back, resets a message
  alarm and reports home, alarm and position. It never moves the axis.
- `axis.UnitInfo()` returns the Python sections (`identity`, `parameters`, `monitor`, `status_words`, `io`,
  `control_words`, `alarms`, `maintenance`, `positions`) plus `limits`.

## Errors

Every exception the driver throws on purpose derives from `EcDriverException`. Refusals happen before anything is
sent, and the message says so ("nothing sent", "nothing written").

| Exception | When |
|---|---|
| `SoftLimitException` | a target, speed, acceleration, band or force outside the unit's range or a cap |
| `EcNotReadyException` | servo off, not homed, moving, no Modbus control; the servo cannot come on (EMG input, drive power cut, enable switch) |
| `EcAlarmException` | the unit reports an alarm (`AlarmCode`) |
| `MotionTimeoutException`, `MotionIncompleteException` | not done in time (a decel stop was sent) / settled away from the target |
| `NonVolatileWriteException`, `UnsafeCommandException` | a row that cannot be written (rows 256-267), a read-only parameter, a never-send IAI command |
| `ModbusTimeoutException`, `ModbusCrcException`, `ModbusFrameException`, `ModbusSlaveException`, `ModbusException` | the link: no answer, bad frame, exception reply, port gone |
| `RecoveryException`, `UnitReplacedException` | a recovery could not bring the axis back / another unit answers where the tracked one was (`Report` has what was done; nothing was moved) |
| `CellException` | one or more axes of a group command failed (`Errors` and `Results` per axis name) |

```csharp
try
{
    axis.MoveTo(500.0, speed: 20);
}
catch (SoftLimitException e)
{
    Console.WriteLine("refused, nothing sent: " + e.Message);
}
catch (EcAlarmException e)
{
    Console.WriteLine($"alarm 0x{e.AlarmCode:X3} {Alarms.AlarmName(e.AlarmCode)}");
}
catch (EcDriverException e)
{
    foreach (var note in e.Notes)          // e.g. "decel stop sent after this error"
        Console.WriteLine(note);
    throw;
}
```

The binding maps the core's result code to these classes (table in [../NATIVE_ABI.md](../NATIVE_ABI.md)).
`NativeErrors.ResultOf(e)` gives the result code and `NativeErrors.ErrorJsonOf(e)` the core's error record. An
invalid argument throws `ArgumentException`. A missing library throws `DllNotFoundException`, one built for another
CPU `BadImageFormatException`, and one of another ABI version `NotSupportedException`.

**Non-volatile writes.** Position-table writes, teaching and parameter writes need no opt-in: `allowNonvolatile`,
`allowDangerous`, `allowHomeParams` and `allowUnlisted` are accepted and ignored (no longer needed), and a risky write
goes through with a warning (`Warnings`). They write the unit's non-volatile memory, which lasts about 100,000 write
cycles (IAI). An unchanged value or row is not written again (`Skipped`), and the driver logs a warning above 60 writes
an hour on one axis. For a target that changes every cycle use `MoveTo`: direct moves are RAM only (VERIFIED).

## Threads

- One command at a time per axis: command an `Axis` from one thread at a time (the core serialises frames, not whole
  commands).
- `Stop()`, `Pause()` and the cancel calls may come from another thread while a move, home or `WaitIdle` waits. The
  waiting move then ends with `MotionIncompleteException`, or with a decel stop and `OperationCanceledException` for a
  cancel. That is how a stop button stops a blocking move.
- Axes on different ports run in parallel. Axes that share a port take turns on its lock.
- A cell takes every axis it finds (no total limit; `CellOptions.MaxAxes` caps it if given). One cable carries at most
  16 axes (axis numbers 0..15, addresses 1..16), and the scan asks addresses 1..16 per port by default. One axis per USB
  adapter runs fastest: axes on one cable take turns.
- `Cell` group commands run the ports in parallel and return when every axis is done. `StopAll()` may come from
  another thread while a group command waits.
- The core's last error is per thread. The binding reads it on the calling thread right after the failing call, so
  the caller needs to do nothing. A call on a disposed axis throws `ObjectDisposedException`.
- Logging: subscribe to `DriverLog.Message` (the core's log lines arrive there with their sources).
  `NativeDriver.SetLogHandler` takes the core's lines directly.

## Where the native library comes from

The binding loads `iai_actuator_driver` with P/Invoke: `iai_actuator_driver.dll` on Windows,
`libiai_actuator_driver.so` on Linux, `libiai_actuator_driver.dylib` on macOS. The .NET runtime finds it next to the
application or, in a NuGet package, under `runtimes/<rid>/native/` (`win-x64`, `linux-x64`, `osx-arm64`...).

- **A release:** the NuGet package carries it. Otherwise use the library and the binding of the same release.
- **Windows build:** see [Install](#install). The DLL needs no runtime on Windows 10 / 11.
- **CMake build:** `cmake -S core -B core/build -DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON` and
  `cmake --build core/build` (Linux: `libiai_actuator_driver.so`); see [cpp.md](cpp.md).

`NativeDriver.EnsureLoaded()` runs before the first handle is opened. It checks the library's ABI version: the same
ABI major and a library minor at least the binding's (C ABI 1.0, frozen within major 1). `NativeDriver.AbiVersion`,
`NativeDriver.LibraryVersion` and `NativeDriver.ListPortsJson()` report what was loaded. A 64-bit process needs the
x64 library; no 32-bit build is produced.

## Command line

```shell
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- ports              # serial ports, IAI adapters first
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- --sim rod limits   # the simulator
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- --port COM3 status # read-only
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- --port COM3 check  # the compatibility self-test, JSON report
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- --port COM3 backup unit.json
dotnet run --project bindings/dotnet/src/IaiActuatorDriver.Cli -- --port COM3 recover --wait 60   # after a power or link loss
```

- The commands and options match the Python command line; `--help` lists them.
- `check` is read-only unless `--motion`. `backup` is read-only (FC03 / FC11 only). `recover` never moves the axis.
- Also there: `monitor --watch`, `move --fast`, `move --change-speed` / `--change-accel`, `teach`, `pause`, `resume`
  and `cancel-pause`.
- Not in this command line (the Python and C++ ones have them): `--line`, `--position-unit`, `line` and `stop-all`.

## Evidence

| Part | Evidence |
|---|---|
| The command line `iai-actuator-driver-cs` on the EC-GRBP8M gripper and the PCON-CB with an RCP6 | VERIFIED; steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) |
| The public `Axis` on the EC-GRBP8M gripper | VERIFIED for the steps listed in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) |
| The core's binding on the PCON-CB with an RCP6 | VERIFIED; steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) |
| Fast mode, the status monitor's events, a change of speed, move handles, `CheckLink`, recovery after a line break | UNVERIFIED in this edition (tested on the simulator); the Python edition verified the same core features on hardware |
| `Cell`, a push that makes contact, position-table writes, recovery, gateway lines, everything else | UNVERIFIED (tested on the simulator and on recorded frames) |

## Next

- [bindings/dotnet/README.md](../../bindings/dotnet/README.md): the API table, multi-axis layouts, guards.
- [../NATIVE_ABI.md](../NATIVE_ABI.md): the C ABI under the library's binding.
- [../TROUBLESHOOTING.md](../TROUBLESHOOTING.md): error messages and what to do.
