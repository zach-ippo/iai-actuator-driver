# Architecture

How IAI Actuator Driver is built, for maintainers: the layers, how a command reaches the wire, the rules each layer
keeps, and the one C++ core under every language. The functions are listed in [DRIVER.md](DRIVER.md); behaviour and
its evidence are in [REFERENCE.md](REFERENCE.md). Unofficial: not an IAI product.

## Layers

The module names below are those of the Python package, each a binding over the C++ core in `core/src`, which has
the same layers (`_backend.py` loads the core at import). Each layer calls the one below it. Above the transport,
everything runs the same on a serial port and on the simulator.

```text
apps/gui/     server.py (HTTP), workbench.py (multi-axis), controller.py + backend.py (single-axis page)
cli.py        iai-actuator-driver               check.py   iai-check
   |
cell.py       list_ports(), scan(), Port (one bus per COM port), Cell (group commands, fault handling)
   |
ec.py         Elecylinder = Axis: identity and ranges from the unit, checks, commands, completion, STOP
              beside it: monitor.py (status thread), recovery.py (link loss), info.py (unit info, backup)
   |
registers.py  addresses, bits, scales, coils, IAI command tables, each with its evidence word
              beside it: alarms.py (names, levels), con_params.py (CON parameter numbers), errors.py
   |
gateway.py    Line: what a line carries (direct, rcon, robonet, robonet-gateway); read-only gateway objects
modbus.py     ModbusRTU: framing, CRC, one lock per port, gap, retries, timeouts, IAI 0x45 / 0x48, broadcast
   |
transport     SerialTransport (pyserial) | SimulatedEC | SimLine | SimGatewayLine
```

| Module | Role |
|---|---|
| `modbus.py` | `ModbusRTU`, one per port: retries, a 5 ms gap between requests, a response timeout from IAI's formula (DOC ME0162-12A p.4-1), the IAI command safe list and never-send lists, broadcast, `detect_baudrate()` and `reopen()` in place. `SerialTransport` opens the port once at 8N1; it sends a line break only for `set_baudrate(method="break")`. |
| `gateway.py` | `Line` and its rules, `detect_line()` (two read-only probes), the read-only RCON and ROBONET gateway objects. |
| `ec.py` | `Elecylinder` (`Axis` is the same class): one axis at one slave address on a bus. Command strategies (`coil`, `iai`, `iai2`, `register`) for servo, home and alarm reset. |
| `info.py` | Read-only unit info (FC03 and FC11 only, every block read bounded by the unit's own directory), parameter decoding, `backup()`. |
| `monitor.py`, `recovery.py` | `AxisMonitor`, `fast_done()`, `suspend_bus()`, `timer_resolution()`; sessions, `check_link()`, `recover()`, `LinkMonitor`. |
| `apps/gui/` | The local web GUI `iai-gui`: stdlib HTTP server on 127.0.0.1, a per-run token on every `/api` request. |

## A command on its way to the wire

`ax.move_to(50.0, speed_mm_s=20)` runs these steps in order:

1. **Arguments.** Finite numbers and a known completion rule.
2. **Line.** On an RCON line the gateway must be in MANU and the position unit stated (`GatewayModeError`,
   `PositionUnitError`).
3. **Ranges**, all read from the unit (`ax.limits()`): the target inside the effective stroke (the soft limits 7.1 /
   7.2 minus the 0.30 margin, intersected with a configured `stroke`); speed up to min(MAXV 7.4, `max_speed_mm_s`);
   acceleration up to min(MAXA 7.5 / MAXD 7.6, `max_accel_g`); a band of 0.01 to 10, not below MINI 3.67; an
   acceleration pattern only when the support word MVPT has it (`SoftLimitError`). A speed below the documented
   minimum only warns.
4. **Readiness**, from one status read, or from a monitor sample younger than 50 ms that shows the axis ready: no
   emergency stop, no fault alarm, Modbus control (PMSL, taken only while the axis is at rest), servo on, homed, not
   homing, not paused (`EcNotReadyError`, `EcAlarmError`).
5. **Bus.** `Line.check_request()` refuses a function code, a coil, an IAI command or a broadcast that the line does
   not carry (`LineRuleError`). IAI commands also pass the safe list and the never-send lists (`UnsafeCommandError`).
6. **Frame.** A STOP coil left ON is released first. Then one FC10 writes the full 9-word 0x9900 block, CTLF
   included: a stale CTLF bit left by another tool would turn the move into a relative move or a push.

Every refusal is raised before the motion frame is sent, and steps 1 to 3 write nothing. `check_move()` runs steps 1
to 4 without moving, and `Cell.move_all()` runs it on every axis before any axis starts.

Writes that change the unit have their own checks: they need no opt-in (a home, dangerous or unlisted parameter, a
row the unit does not run and a baud code not VERIFIED are written with a warning), values are range-checked, a
software reset and a baud change need the axis at rest, and the non-volatile write guard skips a write that would not
change anything.

The library's caps default to the unit's own maxima. The command line passes conservative caps (100 units/s, 0.3 G,
70 %) unless `--unit-max` is given; the driver lowers any cap to the unit's own maximum.

## Threads and locks

- **One bus per COM port, one lock per bus.** A transaction (request, retries, reply) holds the `ModbusRTU` lock;
  requests are at least 5 ms apart. The axes of a shared port take turns; ports never wait for each other.
- **Waits release the bus.** Moves, home, servo and the STOP hold sleep between status polls without the lock, so a
  STOP from another thread, a monitor poll or another axis of the same port gets the bus between two polls.
- **No lock in the axis.** An `Elecylinder` keeps its state without a lock: run one command at a time per axis.
  `stop()`, `pause()` and `resume()` may come from another thread while a command waits. The GUI workbench works this
  way: commands run under a per-axis lock; STOP, pause and resume never take it.
- **Cell.** Reads and command sends run per port in parallel threads (the axes of a port one after another);
  blocking waits run one thread per axis. Ctrl+C during a group move stops the axes before the threads are joined.
  `Port` has its own locks for its axes, the one-time baud detection and the one-time line detection.
- **Monitor.** One optional thread per axis. `suspend_bus()` holds every monitor of a port during a baud change, a
  software reset and `recover()`, which change the line under every axis.
- **Recovery.** One recovery at a time per axis (`check_link()` answers `"recovering"` meanwhile). `LinkMonitor` is a
  thread that only checks. `Cell` handles each fault once per axis, also when a monitor thread reports it.
- **Timer.** On Windows the waits and the monitor raise the timer resolution to 1 ms (`timer_resolution()`,
  reference-counted); the default granularity is about 15.6 ms (VERIFIED on the test PC). Other systems: no change.

## Completion rule

`completion="safe"` (the default): a move is done when the unit reports in position (PEND, or PSFL for a push that
missed), the position is within the band and PNOW has stayed unchanged for `settle_s` (0.1 s, two polls). The settle
window is needed: on the EC-GRBP8M gripper PEND and MEND rose inside the band while the axis still moved (VERIFIED).
The DSSE MOVE bit clears while the axis still decelerates (VERIFIED), so the safe rule does not use it. The same rule
ends a home return and the STOP hold.

- An axis that stands in position away from the target for 0.5 s raises `MotionIncompleteError` (the move was
  ignored or cut short) instead of waiting for the timeout.
- A `stop()` of the same axis from another thread ends a move wait at once with `MotionIncompleteError` ("stopped
  from outside this wait"); the wait sends no STOP of its own (ON-WIRE on a PCON-CB + RCP6). A pause does not end it.
- The default timeout is twice the trapezoidal move time plus 2 s. On a timeout, an alarm or a lost link a decel stop
  is attempted before the error is raised.
- A push outcome (contact or miss) must hold for `push_settle_s` (0.15 s) with the position unchanged: a push that
  missed showed the full contact pattern for a moment first (VERIFIED on the EC-GRBP8M gripper).
- `completion="fast"` (opt-in, per axis or per call): done on the first sample with PEND, the position inside the
  band and DSSE MOVE clear, not homing, not paused (`monitor.fast_done()`). The axis may still settle inside the band
  afterwards (up to 0.08 mm short on the PCON-CB, VERIFIED).

## STOP: hold and release

`stop()` writes the STOP coil 0x042C ON, holds it until the axis has stopped (idle, or steady with MOVE clear, and
PNOW unchanged for `settle_s`; servo off counts at once), then writes it OFF and returns the standstill status. It
never touches PMSL.

VERIFIED on moves commanded over Modbus, also with PMSS = 0, and on a move started by a hand-wired PIO input of the
EC-GRBP8M gripper: the unit accepted the STOP and stopped, and a move that matched the changed input ran after the
release. Whether a start input held through the STOP restarts the move is UNVERIFIED (the input word was not sampled
while the STOP was held). UNVERIFIED on a move a PLC commanded over PIO.

- **Why release explicitly.** Whether the unit releases STOP by itself is not known, and a STOP left ON would block
  the next move (the simulator models that worst case). A failed release is retried before the next command and at
  `close()`.
- **Error path.** An exception while motion may run (timeout, alarm, a lost link, Ctrl+C, an exception in a `with`
  block after a motion command) sends the same sequence, holding at most 3 s, before the error propagates. A note on
  the exception says whether the stop went out.
- **Register path** (`ax.stop_path == "register"`): on a line without coils (RCON), or after a unit answered FC05
  with exception 01: DRG1 pause, hold until standstill, an ALRS edge while paused (it cancels the remaining travel),
  release (DOC; UNVERIFIED on an EC). With a fault alarm active the ALRS edge is left out.
- **Cell.** `stop_all()` first writes the STOP coil to every axis on a line with coils (ports in parallel, no
  retries), then runs each axis's full stop in its own thread. `broadcast=True` adds one broadcast STOP per daisy chain
  (UNVERIFIED).
- The IAI 0x45 / 0x48 strategies are never used to stop.

## Status monitor and fast mode

With the defaults, the time around each move goes to the settle window, 22-word status polls of 20 to 50 ms each, the
15.6 ms Windows sleep granularity and a status read before each move. Each part of the fast path below is optional.
Fast completion shortened the median move on the PCON-CB, at 38400 and more at 230400 baud (VERIFIED).

- **`AxisMonitor`** (`ax.start_monitor()`) reads the lean status 0x9000-0x9007 (8 words: PNOW, ALMC, DIPM, DOPM,
  DSS1, DSS2, DSSE) back to back. Each poll is one FC03 without retries under the bus lock, followed by 2 ms with the
  lock free. It publishes `latest` with timestamps, wakes the waiters (`wait_for`, `wait_update`, `wait_done`) and
  calls `on_change`, `on_done` and `on_event` in its own thread: keep callbacks short and never call a blocking axis
  command from one.
- **Use by the axis.** A sample younger than 50 ms that shows the axis ready replaces the pre-move status read, so a
  direct move is one frame. The waits take its samples, only those whose request went out after the command; when the
  monitor publishes nothing for 0.25 s the waiter reads for itself. A lean sample that shows the servo off, an alarm
  or an emergency stop is confirmed with a full read. STOP never waits for the monitor.
- **Errors.** A timeout or CRC error is retried after a back-off (10 ms, doubling up to 0.5 s); three failed polls in
  a row report a lost link; a transport error (the port vanished) ends the thread and every waiter raises it.
- **Link watch.** Every second, while the axis has a recovery session (from `track()`, or made from the identity that
  `connect()` read), the monitor runs `check_link()` and reports `"link_lost"` or `"unit_restarted"`. It never
  recovers anything itself.
- **`enable_fast()`** (or `Axis.open(port, fast=True)`): `completion="fast"`, the monitor, and 230400 baud (VERIFIED
  on the EC-R6S rod, the EC-GRBP8M gripper and the PCON-CB; volatile). The rate is raised only when the axis owns its
  port, the line carries broadcast IAI commands and the axis is at rest; otherwise it is kept and the returned `notes`
  say why. `disable_fast()` keeps the rate.
- `wait=False` returns a `MoveHandle`. `move_sequence(points, blend_band=...)` sends the next point once the axis is
  within `blend_band` of the current one (UNVERIFIED on hardware).

## Recovery

`iai_actuator_driver.recovery` finds out what happened to a link and brings the axis back. It never moves the axis: no
servo ON, no home, no move, and a move cut off by the fault is not resumed.

- `track()` records the session while the link is up: identity, baud rate, PC control, safety speed, servo, HEND,
  uptime, alarm history, and on request the stored rows and parameters.
- `check_link()` reads the status once and answers `"ok"`, `"port_vanished"` (a pulled USB adapter makes the COM port
  disappear: VERIFIED), `"unit_silent"` or `"unit_restarted"`. Restart signs after a power-up (VERIFIED): the uptime
  counter 0x9010 dropped (a software reset does not reset it), a new 0x0FFF power-up record, PC control lost, the rate
  back at 38400; also HEND lost on an incremental unit (VERIFIED at a software reset). Deliberate changes
  (`disable_modbus_control()`, `set_action_mode()`, `software_reset()`, `home()`) are recorded so that they are not
  taken for a restart.
- `recover()` waits until the port is listed again and reopens it in place, finds the rate (38400 first after a power
  loss), requires the same serial number and model (`UnitReplacedError` otherwise, before anything is written),
  restores the baud rate, PC control and the safety-speed choice, re-reads the ranges, resets only a message-level
  alarm, and reports HEND (`needs_home`), the alarm and the position against the last target in a `RecoveryReport`.
- `ax.recover()` holds every monitor on the port while it runs and restarts its own afterwards. `Cell` calls
  `on_fault(name, kind, error)` once per fault and, with `stop_on_fault` (the default), stops every other axis;
  `recover_all()` lets that stop finish first.
- Evidence: VERIFIED on the EC-GRBP8M gripper after a real power cycle (a fresh process with the identity from a
  backup, and live in a running process) and after a line break, and on the PCON-CB after a 250 ms line break. In the
  live power cycle `check_link()` reported `unit_silent`, then `unit_restarted`; `recover()` took PC control back, the
  servo stayed on, HEND was kept, the rows and parameters were unchanged and nothing was written to non-volatile
  memory. That 3.2 s outage wrote no new 0x0FFF power-up record.

## Simulator

`SimulatedEC` is a transport: it parses each request frame and answers byte for byte (FC03, 05, 06, 10, 11, 17 and
IAI 0x45 / 0x48), stays silent on a CRC error and can inject timeouts and CRC errors. The profiles `slider` (the
EC-DS4H), `gripper` (the EC-GRBP8M), `rod` (the EC-R6S) and `pcon` (the PCON-CB with an RCP6) serve register blocks
read from those units (`sim_fixtures.py`; serial numbers are placeholders). `rotary` and `unknown` are made-up units
for the generic paths.

- Motion follows a real-time trapezoid (`time_scale` speeds tests up) and reproduces what the hardware showed: PEND
  early inside the band with a settling tail, MOVE clearing during deceleration, PMSL switched on mid-move stopping
  the move, 0x9800 ignored while PMSS = 0, the brief contact pattern of a push that misses.
- Where the hardware is not known it takes the worst case: STOP latches until released, a held PLC start input
  restarts the move when STOP is released, a restart during a parameter commit loses the commit.
- A read outside every directory block answers exception 02 (stricter than the units), so tests catch stray reads.
- `SimLine` puts several units on one line (every frame reaches every unit; a broadcast reaches all, none answers);
  `sim_ports(n)` gives n lines; `sim_rcon_line()` and `sim_robonet_line()` model the gateway lines (DOC only).
- Faults for recovery tests: `power_off()`, `power_on()`, `power_cycle()`, `unplug()`, `replug()`, `reopen()`, and
  `serial_number=` for a replaced unit.

Passing tests against the simulator show that the driver agrees with what was observed, not that every unit behaves
this way.

## Evidence words and no model allow-list

Every hardware claim in code comments, docstrings, command output, the GUI and these documents carries one of four
words: **VERIFIED** (tried on hardware with this driver), **ON-WIRE** (seen in serial traffic), **DOC** (written in
a public IAI manual) and **UNVERIFIED** (expected, not yet tried). The words describe features and paths, never a unit
or a model. `registers.py` tags every register, bit and command,
and also uses VERIFIED-LIVE (read live on a unit), BACKUP (present in a backup saved from the unit) and DOC-CON
(documented for the CON family only).

There is no list of allowed models. Every range a command is checked against comes from the unit itself: the register
directory at 0x0000 and the ID maps locate the areas, class 7 gives the limits and the support words give the
capabilities. A model table would add no check the unit does not already provide, would go stale with each new model
or firmware, and would refuse units that the same code drives: the ELECYLINDERs and the PCON-CB answered the same
protocol and register directory (VERIFIED). So every family, unit code and actuator kind is driven the same way and
is reported as what it says it is; no unit, model or unit code is labelled, and a new model needs no driver update.
An `iai-check` report from a unit is what moves a feature path to VERIFIED.

What the driver does refuse, before anything is sent: a value outside the unit's own ranges, a command the line does not
carry, motion while the unit's limits are unknown and no `stroke` is given, a read-only parameter, IAI commands off the
safe list without `allow_unsafe=True`, and those on the never-send lists without `allow_never_send=True`. Non-volatile
and dangerous writes need no opt-in; a risky one is written with a warning. The non-volatile write guard skips unchanged
writes, counts writes per axis and warns above 60 an hour; it never refuses one.

When a path is confirmed on hardware, change its word in the code comment and docstring, in DRIVER.md, REFERENCE.md
and COMPATIBILITY.md, and add a CHANGELOG.md entry. The code path itself stays as it was.

## One core under every language

Separate implementations in Python, C# and C++ would cost one fix per language and drift apart. So the driver is
implemented once, in C++ (`core/`), behind a small, stable C ABI. Each language gets a thin binding that keeps its own
idiomatic API. Everything is in this one repository and released from one commit. The ABI's model (handles, errors,
threads, data, versioning, loading per language) is in [NATIVE_ABI.md](NATIVE_ABI.md); the header
`core/include/iai_actuator_driver.h` documents each function. Per-language guides are in
[getting-started/](getting-started/).

```text
apps/gui (iai-gui)      iai-actuator-driver, iai-check     iai-actuator-driver-cs     your program
        \__________________________|                               |                    |
                                   |                               |                    |
bindings/python                    |     bindings/dotnet           |      C, LabVIEW    |    C++ program
  iai_actuator_driver (Axis, Cell) |       IaiActuatorDriver (Axis, Cell)               |         |
  iai_actuator_driver.native       |       IaiActuatorDriver.Interop (internal:         |         |
  (ctypes, no compiler needed)     |       P/Invoke, SafeHandle)                        |         |
           \_______________________|_______________________________________________/              |
                                   |                                                              |
C ABI                    iai_actuator_driver.h, prefix iad_ (core/src/c_api*.cpp)                 |
                         iai_actuator_driver.dll / libiai_actuator_driver.so / .dylib            |
                                   |                                                              |
core/ (C++17)            Axis, Cell, Port, ModbusRtu (one mutex per port), registers, status, alarms, <-+
                         parameters, non-volatile write guard, monitor, recovery, lines and gateways,
                         coordinated motion, self-test, simulator
                                   |
transport                SerialTransport (Win32, POSIX) | SimulatedUnit | SimLine
```

### Rules

1. **Behaviour lives in the core.** The checks before every frame, the completion rule, STOP and its release, the write
   warnings, the non-volatile write guard, recovery, line rules, coordinated motion and the simulator are implemented
   once. A binding adds no behaviour: it turns arguments into option structs, results into the language's types, result
   codes and the last-error record into the language's exceptions, and handles into objects with `close()` /
   `Dispose()`.
2. **Public APIs stay per language.** Python keeps `Axis` (`Elecylinder`), `Cell`, `SimulatedEC`, its keyword
   arguments, dataclasses and exception classes; .NET keeps `Axis`, `Cell`, `SimulatedUnit` and its exception classes;
   C++ keeps its headers. Moving a language onto the core does not change a caller's code.
3. **The Python tests are the conformance specification.** The C++ tests mirror the Python test classes area by area.
   The Python suite and the .NET suite run on the core through their bindings; what only the core's internals show
   is checked by a C++ test.
   The ABI itself is tested by the `capi_` tests, the C99 program `core/tests/c_smoke.c` and each binding's tests on
   the core's simulator.
4. **A change goes core first.** Code and C++ test in `core/`, then the ABI (a new function or a reserved slot given a
   meaning is a minor version), then the bindings, then the documents. Behaviour can be worked out on hardware with
   whichever language is quickest to script; the fix lands in the core. The Python package runs on the core only (its
   pure-Python fallback was removed after 1.0.x: [PYTHON_FALLBACK.md](PYTHON_FALLBACK.md)). The Python tests remain
   the conformance suite: where the core disagrees with one, the core is fixed.

### What stays in each language

| Part | Where |
|---|---|
| Command line `iai-actuator-driver`, self-test `iai-check` | Python (`bindings/python`), over the package API |
| `iai-gui` | `apps/gui`, a Python HTTP server over the package API |
| Command line `iai-actuator-driver-cs` | .NET (`bindings/dotnet`), over `IaiActuatorDriver` |
| Command line `iai-actuator-driver-cpp` | `core/cli` |
| Value types (`Status`, `DeviceInfo`, `PositionEntry`, write results) | per language, filled from the core's structs and JSON; derived flags (ready, idle, fault) are computed in the core so the decoding stays in one place |
| Exception classes | per language; the binding maps each result code to one (table in NATIVE_ABI.md) |
| Constants (register addresses, bits, codes) | the language's own modules; a test compares them with the core |
| Coordinated-motion geometry (lines, arcs, paths) | built in the language and passed as JSON; planning and running are in the core |

### The C ABI

- Plain C99, every name prefixed `iad_`. Opaque handles (`iad_axis*`, `iad_cell*`, `iad_move_handle*`) are registry
  tokens, never addresses, so a stale or doubled handle is detected; closing twice or `NULL` is safe.
- Every function returns an `iad_result` (0 or a negative error kind mirroring the C++ exception classes) and leaves a
  thread-local message and JSON record of the last error. No C++ exception crosses the boundary.
- Frequent data travels in plain structs with fixed-size fields and no implicit padding (the same layout packed and
  natural, so 32-bit LabVIEW clusters match). Output structs have a fixed size for the whole major version. Input
  structs start with their own size (`struct_size`), so a minor version can add fields at the end and a library
  refuses a field it does not know. Every options struct has an `_init` function: a zeroed struct is not the
  defaults.
- Rich data (unit info, limits with their sources, alarm history, parameters) is UTF-8 JSON in a caller buffer with a
  required-size output; its fields are only ever added.
- `iad_axis_open_sim` and `iad_cell_open_sim` run every function on the simulator, so each binding is tested without
  hardware.
- The header is at C ABI 1.0, frozen within major 1: sized inputs, fixed outputs, one rule for the defaults,
  `IAD_UNKNOWN` 2, `IAD_CALL` for 32-bit Windows, and each function once (variants folded into their base functions,
  the axis commands with options structs). The ABI grows by minor versions that only add functions, codes or
  meanings of reserved slots; earlier functions and struct sizes stay (the contract is the first section of
  NATIVE_ABI.md).
- It covers what the Python package has: push, command-profile moves, acceleration patterns, moves started without
  waiting (`iad_axis_start_move_to`, `iad_move_handle_wait`, `iad_move_handle_poll`), the status monitor and fast
  mode, recovery, line and gateway calls, unit info and backup, the self-test, the cell's extras, coordinated motion
  (JSON requests), cancellable waits (`iad_axis_cancel`, `iad_cell_cancel`, result `IAD_ERR_CANCELLED`), events and
  the log and frame sinks. It also covers what the bindings' test suites need on the core: transports and buses as
  handles (a binding's own transport through callbacks), simulated units and lines, the log level and a cancel latch.
  NATIVE_ABI.md describes each block of the header.
- The test hooks are in `iai_actuator_driver_testing.h`, with a version of their own and no stability promise: the
  clock seam (a virtual clock), the simulator's hooks and controls, the driver's time limits, and the test hooks of
  the non-volatile guard and the port list.

### Threads across the layers

- **Core.** One bus mutex per port: axes that share a port take turns, ports run in parallel. One thread at a time
  commands a handle (the bus serialises the frames, but two commands on one handle from two threads are not made into
  one sequence). `iad_axis_stop`, `iad_axis_pause`, `iad_cell_stop_all`, `iad_axis_cancel` and `iad_cell_cancel` are
  meant to come from another thread while a call waits on the same handle. A STOP goes before every request waiting
  for the bus (the one on the wire finishes first). The handle registry has its own mutex, held only to look a handle
  up, never during I/O.
- **Core threads.** A cell's port workers, status monitors, the link monitor and coordinated runs run on threads the
  caller does not own. No binding code runs on them, with one exception: the optional log and frame sinks run on the
  thread that logs, must be short and must not call into the library. Everything else a language would see as a
  callback (monitor changes, link and recovery events, a cell's faults, self-test lines) is queued per handle and
  drained by the binding (`iad_axis_next_event`, `iad_cell_next_event`), so a slow handler never delays a poll or a
  STOP. The cell stops its other axes on a fault inside the core, before the event is queued. Closing a handle stops
  and joins its core threads.
- **Last error.** Per thread: a binding reads it on the calling thread right after the failing call, with nothing in
  between (no `await` in .NET).
- **Python.** ctypes releases the GIL during each call, so a stop from another Python thread reaches the core while a
  move waits. Long waits run in short slices (`iad_move_handle_poll`) so that Ctrl+C still sends a decel stop. At
  interpreter exit the binding stops core threads and closes handles before ctypes is torn down.
- **.NET.** `AxisHandle` and `CellHandle` are `SafeHandle`s: a handle stays alive while a call on it runs, and a
  forgotten one is closed by the finalizer. Queued events are raised as C# events on a thread the binding owns.

### Loading and distribution

One library per OS and CPU, built from `core/` with the C++ runtime linked statically:

| OS | File | Notes |
|---|---|---|
| Windows x64 | `iai_actuator_driver.dll` (+ import library) | imports only Windows' own DLLs and the UCRT (Windows 10 / 11; Windows 7 / 8.1 need KB2999226). Not code-signed yet |
| Windows x86 | `iai_actuator_driver.dll` (32-bit) | for 32-bit LabVIEW and 32-bit applications; not built yet |
| Linux x64 / arm64 | `libiai_actuator_driver.so` | linked against glibc 2.17 symbols (manylinux2014); `serial_posix.cpp` VERIFIED on a real port from 1.1.0 (the IAI USB adapter in WSL 2) |
| macOS arm64 / x64 | `libiai_actuator_driver.dylib` | not built by the release; UNVERIFIED (no Mac in the test setup) |

| Language | Where the binding finds it |
|---|---|
| Python | `IAI_ACTUATOR_DRIVER_NATIVE`, else `iai_actuator_driver/_native/<platform>/` inside the package (a platform wheel puts it there), else `core/build/` of a source checkout |
| .NET | next to the application, or `runtimes/<rid>/native/` of the NuGet package `IaiActuatorDriver` |
| C, C++ (shared), LabVIEW | next to the program; C and C++ link the import library (the SDK zip of a release, or a source build) |
| C++ (static) | none: the core is linked into the program |

Builds: `python build.py shared` in `core/` (zig's clang, Windows x64) and CMake with
`-DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON` (customers, Linux CI). A release builds the libraries for win-x64 and linux-x64
with zig (`.github/release/build_native.py`, run by `local_release.py`); `release.yml`, run by hand, adds linux-arm64.
A Python wheel per platform (`py3-none-<platform>`, one for every Python 3.11+, since ctypes needs no compiled
extension), the NuGet package with `runtimes/<rid>/native/` and the C/C++ SDK zips are built from those same library
files.

### Versioning

- **One release version** for the repository: tags `vX.Y.Z` (pre-releases `vX.Y.ZaN`, `bN`, `rcN`). The Python
  package, the .NET assemblies (NuGet form, for example `1.0.0` or `1.1.0-beta.1`), the CMake project (`X.Y.Z` only)
  and `iad_library_version()` report it. `.github/release/check_version.py` checks every place, in CI and before a
  release. Each release ships the header, the libraries, the import library and the bindings built from the same
  commit.
- **The ABI version is separate**: `IAD_ABI_VERSION_MAJOR` / `MINOR` in the header, `iad_abi_version()` in the
  library. A binding checks it on load. It is 1.0, frozen within major 1: the rule is the same major and a library
  minor at least the caller's (`IAD_ABI_COMPATIBLE`, `IAD_ABI_VERSION`), and a higher minor only adds. Result codes
  are never renumbered, output struct sizes are frozen within a major version, and a function whose signature must
  change gets a new name.

### Repository and CI

| Folder | Contents | Tests |
|---|---|---|
| `core/` | the C++ core, the C ABI, `CMakeLists.txt`, `build.py`, `iai-actuator-driver-cpp` | `python build.py test` (zig) or `ctest`; `python build.py smoke` |
| `bindings/python/` | `iai_actuator_driver`, its tests, `pyproject.toml`, `setup.py` (platform wheels) | `python -m unittest discover -s tests -q` |
| `bindings/dotnet/` | `IaiActuatorDriver` (the core's binding inside, `Interop/`), `IaiActuatorDriver.Cli`, tests, `packaging/` | `dotnet test` |
| `apps/gui/` | `iai-gui` | the GUI server, controller and API tests |
| `examples/{python,csharp,cpp,c}/` | one task per program, simulator or unit | run on the simulator |
| `docs/` | the documents | packaging and wording checks |
| `licenses/` | `NOTICE` and `NATIVE_THIRD_PARTY.txt`: the sources of the copies in `bindings/python/` (both) and `apps/gui/` (`NOTICE`), packed into every release asset; `LICENSE` stays at the root | packaging checks |
| `.github/` | `workflows/ci.yml`, `workflows/release.yml`, `release/` (the release scripts), the issue forms, `CONTRIBUTING.md`, `SECURITY.md` | |

One CI workflow (`ci.yml`), and no CI job opens a serial port. The GitHub workflows run only by hand or on a pull
request, never on a push or a tag (Actions minutes are limited); `python tools/test.py` runs the tests on the
development PC.

- A pull request, or a run started by hand, runs one small job: the version check, the core built on Windows with zig,
  the driver tests on that library.
- The same runs start the full set with path filters (a run by hand: every area). Its first job checks the version. A
  change under `core/` builds the library on Windows (zig) and Linux (CMake, gcc), runs the C++ tests and the C smoke
  test, then the Python and .NET binding tests on that library: every binding depends on the core. A change under
  `bindings/python/`, `bindings/dotnet/` or `apps/gui/` runs that part's tests.

Releases are built locally with `.github/release/local_release.py`: the libraries for win-x64 and linux-x64 with their
tests, the platform wheels and the sdist (source, needs the library), the NuGet package, the C/C++ SDK zips and `iai-gui.exe`, each
tested on the simulator, collected with `SHA256SUMS.txt` and the CHANGELOG section. The script prints the
`gh release create` command and never runs it. `release.yml` is the same build on GitHub's runners, with linux-arm64
added: it runs by hand only, publishes nothing, and a pushed tag does not start it. It is UNVERIFIED on GitHub's
runners.

### Evidence across bindings

- A fact VERIFIED through one binding holds for the core paths that binding called; a path no binding has run on a
  unit stays UNVERIFIED until it has. Every claim keeps its evidence word.
- The first set of C ABI calls is VERIFIED on the PCON-CB with an RCP6 through both bindings (steps in
  NATIVE_ABI.md).
- The Python package and the .NET library run on the core. The acceptance runs on the EC-GRBP8M gripper exercised it
  through the Python package, the .NET `Axis` and the C++ and C# command lines
  ([COMPATIBILITY.md](COMPATIBILITY.md#hardware-results)). The rest of the core is simulator-tested and UNVERIFIED on
  hardware.
- The GUI runs on the core too: its reads go through the core's preemptible read scope, its Coordinated tab through
  the core's coordinated motion.
- The public .NET `Cell` has the group commands only; the group extras it lacks are listed in
  [../bindings/dotnet/README.md](../bindings/dotnet/README.md#state-of-this-edition).

## Working on the driver

Each command runs in its folder; no test opens a serial port.

```shell
python build.py test                                 # core/: the C++ tests (zig); python build.py smoke: the C program
python -m unittest discover -s tests -q              # bindings/python/: driver tests, simulator only
dotnet test                                          # bindings/dotnet/: the C# solution
python -m unittest discover -s apps/gui -t apps -q   # repository root: GUI server, controller and API tests
```

`bindings/python/tests/test_packaging.py` checks the version, the entry points, the document links and the wording of
everything shipped.

- **A release** sets one version in `bindings/python/pyproject.toml`, `bindings/python/iai_actuator_driver/__init__.py`,
  `apps/gui/pyproject.toml` (and its pin on the driver), `core/include/iai_actuator_driver/version.hpp`,
  `core/CMakeLists.txt` and the `<Version>` of the C# projects (`python .github/release/check_version.py` lists
  them), adds a CHANGELOG.md entry and builds the release with `.github/release/local_release.py`.
- **A new public function** gets one line in DRIVER.md; its behaviour and evidence go into REFERENCE.md.
- **Hardware tests** start slow and short, back a unit up before a non-volatile write and restore it afterwards.
