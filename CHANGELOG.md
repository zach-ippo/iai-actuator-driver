# Changelog

Unofficial: not an IAI product. Versions follow PEP 440. Evidence words: VERIFIED (hardware), ON-WIRE (observed
serial traffic), DOC (public IAI manuals), UNVERIFIED.

## 1.1.0 - 2026-11-03

A minor release from main: everything of 1.0.1, plus the changes below. The C ABI stays 1.0. No hardware result is
claimed here for the 1.1.0 files themselves: they are checked after the build.

### Fixed

- On Linux the serial transport took a 0 from `read()` (the port is set VMIN = VTIME = 0, and Linux returns 0 while no
  byte has arrived) for a pulled adapter, so the first read after every request failed with `the port vanished
  (unplugged?)` on a real port, the first status read included, and nothing moved. The 1.0.1 Linux wheel, SDK and
  NuGet runtime have it. A read now waits for the reply: a 0 means nothing yet, and only a 0 right after `poll()` said
  readable, or POLLHUP / POLLERR, is a vanished port. New tests on a pseudo terminal (a late reply is read, nothing
  sent is a timeout, the other end closed is a vanished port). VERIFIED on a PCON-CB with the IAI USB adapter passed
  into WSL 2 with usbipd (2026-11-03, a build of the fix): the harness's Python and C++ command-line editions and the
  SDK's C example.

### Changed

- Removed: the pure-Python fallback (`iai_actuator_driver/_pure/`); 1.0.x is the last release with it. The Python
  package runs on the core only: without a usable native library the import raises `ImportError` saying where it
  looked, and `IAI_DRIVER_BACKEND=python` raises a one-line `ImportError`. The sdist is source that needs the library.
  Code written for the fallback runs unchanged apart from the differences in `docs/PYTHON_FALLBACK.md`.
- The `ec_driver` import name is deprecated and will be removed in 2.0: import `iai_actuator_driver`.
- The driver no longer labels units by model: every IAI unit that speaks Modbus RTU on its serial port is supported
  and driven the same way, with no driver update for a new model. A unit is reported as what it says it is (the
  family name when the driver has one, else the product family and its codes). Gone: the "UNVERIFIED for this model"
  parameter note and write warning, the connect and identify notes about a unit code not in the driver's table or not
  tried on hardware, and the UNVERIFIED wording of the rotary, unknown-family range and RCON / ROBONET line texts.
  An AKID kind without a name now reads "AKID kind <n>: commanded as a linear axis in mm".
  `unit_known` now only says the driver has a family name for the unit code (the PCON-CB reads false); `verified`
  and `verified_for_model` equal `known` (an IA-OS parameter list is recorded for the unit code); `seen_on` is always
  empty; `allow_unverified_model` is accepted and ignored. Every name, JSON key and the C ABI are unchanged; some
  values are not: `UNIT_CODES_CONFIRMED` and `MODEL_PARAMETERS_SEEN_ON` (Python and C#) and the Python
  `UNITS_SEEN_ON_HARDWARE` are empty, `OPT_IN_NOTE` is `""`, and a parameter row has no `note` key where the note
  held only that label. The documents list the units run so far as a record, not a gate.
- Writes to the unit need no opt-in any more. Stored-row writes and teach (both methods), home parameters (EC P3 / P4,
  CON No.5 / No.22), dangerous parameters (motor / encoder, lead, gains, PIO / fieldbus / baud setup, a wider soft
  limit, raw items), a parameter the unit's list does not show and a baud code not VERIFIED are written. A row past
  the unit's rows, a row the EC's position mode does not run, a row at an UNVERIFIED directory layout, a push on the
  middle row and unknown or changed control-flag bits are written too. A risky write logs a WARNING and adds a line to
  the result's warnings. `allow_nonvolatile`, `allow_dangerous`, `allow_home_params`, `allow_unlisted`,
  `allow_unverified_code` and `allow_unsafe` on row writes and teach (C# `allowNonvolatile` ..., the C ABI fields)
  are accepted and ignored; their command-line options (`--allow-nonvolatile`, `--allow-dangerous`,
  `--allow-home-params`, `--allow-unlisted`, `--allow-unverified-code`) still parse but are gone from the help. The
  GUI asks one plain OK / Cancel before a permanent write, with no tick box. What stays: an unchanged value is not
  written (`force` writes it), writes are counted with the hourly warning, the never-send list of raw IAI commands
  (firmware update, parameter initialisation, alarm-list clear, axis number, ...) needs `allow_never_send`, IAI
  commands off the safe list need `allow_unsafe`, and the range checks (stroke, speed, push power, load threshold)
  refuse a value outside the unit's range. Push power and load threshold are now checked even with `allow_unsafe`. A
  row the table cannot address raises `NonVolatileWriteError`; rows 256-267 stay refused. A baud rate with no IAI code
  is still refused. A parameter's `writable` flag no longer needs `listed`, so the GUI offers Edit for a parameter
  the unit's list does not show and names that in the confirm.
- A `Cell` takes every axis it finds or is given: no total cap by default (it was 16 axes and 16 ports). `max_axes`
  still caps it when given (Python `Cell.open(max_axes=N)`, C# `CellOptions.MaxAxes`, the C ABI's
  `iad_cell_options.max_axes`, whose default is now 0 = no limit; a negative value is refused). One cable still
  carries at most 16 axes (axis numbers 0-15, addresses 1-16), and the scan asks addresses 1-16 per port by default;
  one axis per USB adapter is fastest, because the axes on one cable take turns. `MAX_PORTS` / `PortManager.MaxPorts`
  stay public at 16 for compatibility and no longer limit anything.

## 1.0.1 - 2026-11-02

A maintenance release over 1.0.0. The C ABI stays 1.0: no new function, constant, error code or struct field; one
optional request key, `"expect_start"` of `iad_cell_coord_run_json` (a 1.0.0 library refuses it). The hardware
results named here were run on a PCON-CB with an RCP6 actuator on 2026-11-02, with the Python package on the core,
built from the source that became 1.0.1: before its version number was set (it reported 1.0.0), except those naming a
build of ea04991, which also ran the C++ and C# command lines; `docs/COMPATIBILITY.md` lists them. The EC gripper, a
power cycle, the PIO watch, two real axes together and the GUI on hardware were not run on 1.0.1: what is VERIFIED
for them keeps the version it was run on.

### Fixed

- PC control is taken back from PIO while the axis is paused at a standstill with the servo off (DSS1 STP, no SV, no
  MOVE bit, no home return, no push, VNOW 0): a PCON in PIO mode with nothing on its PIO connector shows exactly that,
  and 1.0.0 refused `set_action_mode(PC)`, `enable_modbus_control()`, `connect()` and the automatic take-over there.
  A pause with the servo on (a PLC holding a stop input) is still refused, and handing control to PIO still needs the
  axis at rest. Same rule in the core, the pure-Python fallback, the three command lines (Python, C#, and the C++ one
  in the SDK) and the GUI's control switch. VERIFIED on the PCON-CB with the Python package on the core, and with the
  C++ and C# command lines' `mode pc` on a build of ea04991 (2026-11-02); the fallback, the Python command line and
  the GUI's switch were not run on the unit.
- The servo comes on after `set_action_mode(PC)` (C# `SetActionMode`, `mode pc` of the three command lines), so the
  take-back above works from the command lines too. PC is now taken with the PMSL coil (only when PMSS is 0), and the
  safety speed is changed with the SFTY coil (only when it differs), instead of 0x45 cmd 0x0C with op1 = 1. On the
  PCON-CB every op1 = 1 cleared DSSE bit 12, from PIO and in PC mode alike, and the servo then did not come on
  (`servo did not turn on within 3 s (the status shows no known cause)`, no alarm) until a PMSL coil OFF, the only
  thing tried that set the bit again (PMSL ON written again did not; op1 = 0 with the bit at 0 and a power cycle were
  not tried); PMSL ON from PIO kept the bit and the servo came on (VERIFIED on the PCON-CB, 2026-11-02, with the
  Python package on the core).
  1.0.0 had this too, for `set_action_mode(True, safety=...)` on a unit already in PC mode. PIO is unchanged (0x45 cmd
  0x0C op1 = 0), and a unit without FC05 coils still gets op1 = 1. The changed `set_action_mode` took PC control from
  PIO on the PCON-CB with the PMSL coil alone, the bit stayed set and the servo came on, from the Python package on the
  core and from the C++ and C# command lines (VERIFIED, 2026-11-02, a build of ea04991); the simulator's PCON now
  models the bit. The safety-speed change by the SFTY coil in this form, and its effect on the bit, are UNVERIFIED on
  the unit.
- Python coordinated motion runs on the default core backend: `Cell.coordinated()` and `CoordinatedGroup` were on the
  pure-Python fallback only in 1.0.0 (its known issue). VERIFIED with one real axis (the EC gripper) and a simulated
  second axis on a development build of 569e836, 2026-10-31, and with the PCON-CB as x (below); two real axes together
  stay UNVERIFIED. What differs on the core: `docs/COORDINATED.md`, "On the core".
- Linux packages (wheel, NuGet, SDK): every timed wait of the core runs on the monotonic clock. Before, the Linux
  library (built against glibc 2.17, where libc++ has no `pthread_cond_clockwait`) took the deadline of each wait with
  a timeout from the system clock, so a backward step of that clock during a wait (an NTP step, a VM's time sync)
  stretched the wait by the step: the bus lock, cancellable sleeps, status monitors, the link monitor, event queues
  and the coordinated run's schedule, where a stream could stop with `LateFrame` ("would reach the wire 702 ms late").
  Since 1.0.0. Windows is not affected by design: each wait there passes a relative timeout (from reading the
  implementation; not run with a clock step). In a WSL guest whose clock was stepped back 0.6 to 1 s about every 32 s
  (the PC's own clock was not synchronised), a loop of coordinated streams on two simulated axes stopped with
  `LateFrame` in 4 of 36 runs, each at a clock step (2026-11-02); with the fix, 0 of 36 with 4 steps (2026-11-03). A
  new core test steps the clock back during 200 ms waits: 0.5 s before the fix, 0.2 s with it (WSL, a zig release
  build, run as root). It needs the right to set the clock, so the normal test runs skip it. The release build now
  refuses a Linux library that imports `pthread_cond_timedwait` without a way to wait on the monotonic clock.
- Stop latency: a normal request waits its minimum gap before it takes the port, so a STOP does not wait for that
  sleep, and a reply that arrived while the reading thread was not scheduled is no longer a timeout. Every stop logs
  one info line, e.g. `COM3 slave 1: STOP on the wire 7 ms after the call, standstill confirmed 180 ms later`.
- On a link fault with `stop_on_fault` the cell stops the other axes first, then logs and calls `on_fault`.
- C# command line: `monitor --watch` counts a link check still running when it ends.
- GUI: the start page points to Getting started, section 4 (it named a README section that does not exist).
- GUI: disconnecting sends STOP whenever it got no fresh status (axis busy or the read failed), not only when the last
  status showed motion.

### Changed: behaviour you may notice

- A STOP goes first. A write that a STOP (to the same unit, or broadcast) overtakes between two attempts is no longer
  sent again: it fails with `EcNotReadyError` ("a STOP went first ... not re-sent"; before, a 0x9900 block could go
  out again after the STOP on a retry). A write that a STOP overtakes while it waits for the port, or between the
  command's checks and its frame, is refused the same way, nothing sent. Reads still retry.
- A command that would take control or release a STOP is refused while another thread's STOP holds the axis: no move,
  home, servo ON or `resume()` releases a STOP, or takes PC control (PMSL), while a stop of another thread,
  `Cell.stop_all` or a coordinated group's stop still holds the axis ("a STOP is held", nothing sent).
- Coordinated motion on the core: `run_plan` plans the path again in the core from where the axes are, and they must
  be at the plan's start (else `EcNotReadyError`, nothing sent); `timing()` and running a plan made elsewhere
  (`plan_path`) raise `NotImplementedError`. In the core (and C++ `CoordinatedGroup::run`) an outside stop counts from
  the call, its planning included.
- Coordinated runs: a frame's lateness counts its wait for the bus, and a frame that would reach the wire later than
  the limit (`late_abort_s`, one step by default) is refused, nothing sent, and ends the run as a late frame. A run on
  an axis with a STOP held is refused, and a run no longer waits for a drain before its STOP.
- The log and frame sinks run after the thread lets the port go (same thread, same order), so a sink that blocks never
  delays a STOP. A frame sink therefore gets a request and its reply together, once the request has ended: a time the
  sink takes itself is when it was called, not when the frame was on the wire. In Python the core's log level follows
  the package's loggers.
- The pure-Python fallback does not give the STOP-first and sink guarantees above (`docs/PYTHON_FALLBACK.md`, "STOP
  first and the sinks"): where one thread stops what another commands, run on the core.
- `set_action_mode(PC)` writes the PMSL and SFTY coils, not 0x45 cmd 0x0C (above), and writes nothing when PC control
  and the safety speed are already as asked. Its `VerificationError` now reads `action mode PC written but DSSE PMSS
  reads 0` (or `PIO ... reads 1`, `action mode safety speed 1 written but DSS1 SFTY reads 0`).
- The software STOP is not a safety function: said in the README safety list, REFERENCE, NATIVE_ABI and the
  `iad_axis_stop` header comment.

### New option

- Python command line: `recover --backup FILE --rows used` compares only the rows that `backup --rows used` holds.
  Opt-in; without it every row of the file is compared, as before.

### Licence

- Licensed under the Apache License 2.0: every package carries `LICENSE`, `NOTICE` and `NATIVE_THIRD_PARTY.txt` (the
  licences of what is linked into the native library), the GUI zip its third-party notices too.

### Hardware results (PCON-CB with RCP6-WSA10R, 2026-11-02)

- The harness steps `default,recover` ran 28 PASS, 1 CHECK and 1 SKIP (the unit was homed, so no home return) and
  ended with the unit as found; they include taking PC control back from a paused PCON in PIO mode.
- STOP latency, idle: the same as before the changes. At 10 mm/s the STOP reached the wire a median of 16.5 ms after
  the call and the axis was at rest after 88 ms; at 840 mm/s, 7 ms and 254 ms. Under heavy CPU load (24 busy loops) a
  STOP can still take over 100 ms to reach the wire (2 of 10 did).
- Coordinated motion with the PCON-CB as x and a simulated y: 9 of 9 steps PASS (a circle of radius 2 mm at 5 mm/s
  deviated at most 0.08 mm with no late frame, and a stop in mid-circle ended the run in 0.39 s). Under heavy CPU load
  5 steps passed and the sixth failed its check (the run ended on a late-frame refusal where the test expected a stop;
  no frame past the limit was sent) and the rest did not run: coordinated motion under CPU load stays UNVERIFIED.

## 1.0.0 - 2026-10-31

The first production release. One C++ core with a frozen C ABI 1.0 sits under the Python, C# and C/C++ bindings and
the local web GUI. The calls of the 2026-10-31 hardware run on the EC-GRBP8M gripper are VERIFIED;
`docs/COMPATIBILITY.md` lists them.

### C ABI 1.0

- Stable within major 1; the contract is in `docs/NATIVE_ABI.md`. `IAD_ABI_VERSION` and `IAD_ABI_COMPATIBLE(v)` check a
  library against the header (same major, library minor at least the header's). [EXPERIMENTAL] items may still change.
- Each function exists once: the `_ex`, `_method`, `_values`, `_completion` and `2` variants are folded in, and axis
  commands take an options struct (NULL means the defaults). Renamed: `iad_axis_unit_info_json` to
  `iad_axis_identity_json`, `iad_move_*` to `iad_move_handle_*`, `iad_cell_axis` to `iad_cell_open_axis`.
- Input structs start with `struct_size`: set it to `sizeof` before the `_init` call. A size below the 1.0 struct or
  past the library's bound returns `IAD_ERR_INVALID_ARGUMENT`. Output structs have a fixed size (`iad_status` is 208
  bytes). Axis names are 128 bytes; `iad_cell_result.index` and `iad_event.index` identify an axis whose name was cut.
- 0 is the default of every enumeration and `IAD_UNKNOWN` is 2. `IAD_METHOD_*`: DEFAULT 0, COIL 1, IAI 2, IAI2 3,
  REGISTER 4. `IAD_CALL` is `__stdcall` on 32-bit Windows only.
- The Linux SONAME is `libiai_actuator_driver.so.<ABI major>`; the SDK zip adds a linker script for
  `-liai_actuator_driver`. The Windows DLL carries a version resource.
- Test-only functions are in `core/include/iai_actuator_driver_testing.h`, with an exact-match version of their own and
  no stability promise. The Python and C# public APIs are unchanged.

### Changed

- A `timeout_s` of 0, a negative value or an infinity is refused before anything is sent: `IAD_ERR_INVALID_ARGUMENT`
  (C++ `SoftLimitError`, Python `ValueError`, C# `ArgumentException`). This covers moves, stored-row moves, sequences,
  speed changes of a running move, home, servo, push, the cell's `move_all`, `move_each`, `home_all` and `servo_all`,
  and coordinated runs and plans. NaN or None is still the default, and so is 0 in `iad_axis_move`. A STOP always goes
  out: its timeout is never checked.
- An unknown axis name no longer blocks STOP, pause or cancel. `iad_cell_stop_all`, `iad_cell_pause_all`,
  `iad_cell_cancel`, `iad_cell_cancel_latch` and `iad_cell_cancel_reset` act on every listed axis the cell has, then
  report each unknown name as its own error (`IAD_ERR_CELL`). A list of only unknown names cancels nothing. Python
  `Cell.stop_all` / `pause_all` raise `CellError` and C# `Cell.StopAll` raises `CellException`, each holding a
  `KeyError` / `KeyNotFoundException`.
- The pure-Python fallback is frozen and planned to be removed in 1.1. Choosing it (`IAI_DRIVER_BACKEND=python`) raises
  a `DeprecationWarning` once at import; an `auto` fallback raises a `RuntimeWarning` with the reason. See
  `docs/PYTHON_FALLBACK.md`.

### Fixed

- `recover()` after `disable_modbus_control()`, `enable_modbus_control()` and a power cycle takes PC control back.
- With `--json`, the Python and C# command lines keep stdout to one JSON document.
- C# `--version` prints the full package version.
- No exception leaves a thread the core starts. C++ `CoordinatedGroup::run` throws `CoordinatedMotionError`.
- GUI: STOP ALL also stops axes still being connected or scanned, and a STOP cancels commands still waiting for their
  axis lock. A group move must name its axes. Oversized and reversed scan ranges are refused.

### Known limits

- The unit has no communication watchdog: if the PC or the cable fails, a move in progress completes. The driver is not
  a safety function: keep a hardware emergency stop.
- STOP of a PIO move is VERIFIED for a hand-wired PIO input and UNVERIFIED with a PLC.
- Several axes on one port, RCON, ROBONET and Linux serial ports are UNVERIFIED on hardware.
- Coordinated motion is experimental. `iai-gui.exe` is not code-signed, so SmartScreen or an antivirus may warn.

## Before 1.0

Pre-release builds, newest first.

### 0.12.0b1 - 2026-10-30

One C++ core (C ABI 0.7) under the Python and C# bindings, in one repository with one version. Python runs on the core
by default (`IAI_DRIVER_BACKEND=auto`). First release workflow, examples in four languages, and a GUI PC / PIO switch.

### 0.12.0a1 - 2026-10-29

Native core preview: the first C ABI (prefix `iad_`), a shared library, and C# and Python bindings over it.

### 0.11.0a2 - 2026-10-30

Testing preview update with the 0.10.0b1 fixes merged in.

### 0.11.0a1 - 2026-10-29

Testing preview: a Start here panel, a test report and an experimental Coordinated tab in the GUI, and a standalone
Windows `iai-gui.exe`.

### 0.10.0b1 - 2026-10-29

Renamed from ec-driver to IAI Actuator Driver (package `iai_actuator_driver`; commands `iai-actuator-driver`,
`iai-check`, `iai-gui`). `import ec_driver` still works with a `DeprecationWarning`. To upgrade, uninstall
`ec-driver` first.

### 0.10.0a1 - 2026-10-29

Coordinated multi-axis motion run by the PC (alpha); see [docs/COORDINATED.md](docs/COORDINATED.md). Two real axes
together: UNVERIFIED.

### 0.9.1b1 - 2026-10-28

Fast completion and a status monitor; `recover()` after a power or link loss, which never moves the axis. Unchanged
non-volatile writes are skipped (the memory lasts about 100,000 write cycles, IAI).

### 0.9.0b1 - 2026-10-28

First beta: one `Axis` for ELECYLINDER and CON-family units, several axes through `Port` and `Cell`, the command line,
the self-test, the web GUI and backups. Limits are checked before sending; non-volatile writes need an opt-in.

### 0.2.0 and earlier

Internal alpha builds. Not released.
