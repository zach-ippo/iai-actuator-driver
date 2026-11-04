# iai_actuator_driver: C++ core

The C++17 core of IAI Actuator Driver, for IAI actuators over serial Modbus RTU: ELECYLINDER (EC) units and
CON-family controllers (PCON / ACON / SCON / DCON) with RoboCylinder. It commands every axis it finds (no total limit;
`CellOptions::max_axes` caps it), one per COM port or up to 16 per port by slave address (axis number + 1), instead of
PLC I/O.

Unofficial: not an IAI product; see the [README](../README.md).

- Standard C++17, no third-party dependencies. Windows (Win32 serial, SetupAPI port listing) and Linux (termios,
  `/sys/class/tty`; Linux serial port: VERIFIED, [docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md)).
- This core is the one implementation under the Python package and the .NET library. The Python package's tests are
  the specification it is held to. The design is in [DESIGN.md](DESIGN.md).
- Evidence words (VERIFIED, ON-WIRE, DOC, UNVERIFIED) as in the [README](../README.md). VERIFIED means tried on
  hardware through any binding or the C++ command line; the paths no run used are UNVERIFIED (table below).

## Status

The tests use the simulator and scripted frames only; no test opens a serial port.

| Part | State |
|---|---|
| Bus: CRC, FC03 / 05 / 06 / 10 / 11 / 17, IAI 0x45 subset, 3 retries, timeouts, baud change and detection | done, simulator-tested |
| Registers, status decode, alarms, identify, limits (ranges read from the unit) | done, simulator-tested |
| Axis: connect, servo, home, moves, stop / pause / resume, alarms, position table, teach, parameters, software reset, baud; stored rows follow an EC's position mode (P9) | done, simulator-tested |
| Motion extras: push(), command-profile moves (own deceleration, FC17 + PCRF), acceleration patterns (CTLF b6-b7), change_speed / change_accel of a running move, move_sequence (blended points) | done, simulator-tested |
| Fast path: Completion::Fast, the status monitor (AxisMonitor, lean 8-word reads), MoveHandle (start_move_to / start_move_by / start_jog), enable_fast | done, simulator-tested |
| Power / link-loss recovery: track, check_link, LinkMonitor, recover() (same unit, baud rate, PC control, safety speed, alarm assessment, rows / parameters against a backup); Cell track_all / check_links / monitor_links / recover_all | done, simulator-tested (power cycle, unplug / replug, unit replaced) |
| Unit info (its sections), backup / backup_axes (the Python JSON schema), the compatibility check (run_check) | done, simulator-tested; the backup JSON was compared key for key with the Python package's |
| Gateway lines: line rules and the request guard, RCON (D0h status, MANU, all-axes monitor, 0.001 mm position unit, identity from the 0x9400 serial code, a silent directory treated as absent, the position-table refusal), ROBONET (SIO through mode; gateway mode refused), line detection | done, simulator-tested (DOC only, UNVERIFIED on hardware) |
| Command strategies: coil, IAI 0x45, IAI 0x48 (EXPERIMENTAL), register (FC06 DRG1 servo / home / alarm reset / pause / safety speed and the coil-free STOP: pause, standstill, ALRS edge while paused, release), chosen by the line or after an FC05 exception 01; the action mode (PIO: 0x45 cmd 0x0C; PC: the PMSL / SFTY coils), set_safety_speed, teach with the MOD / TEAC coils (EXPERIMENTAL), the automatic servo OFF switched back on before a move | done, simulator-tested; the register path, 0x48 and the coil teach are UNVERIFIED on hardware |
| Non-volatile write guard (unchanged writes skipped, counter, hourly warning) | done, simulator-tested |
| Cell (every axis found, no total limit; ports in parallel, stop on fault, stop_all with an optional broadcast STOP); start_monitors / stop_monitors (link events to on_fault), status_all from a monitor sample, monitor_all, backup, pause_all / resume_all (optional broadcast), move_all with per-axis speed / accel / band / decel | done, simulator-tested |
| Coordinated motion (`coord.hpp`, `Cell::coordinated`): lines, arcs, circles and polylines over several axes run by the PC, synchronized moves or streamed re-targeting (0x9900), the planner's checks (ranges, speeds, accelerations, the line's timing) with the largest feasible speed; a fault, a link fault, a late frame or an outside stop stops the whole group | done, simulator-tested; UNVERIFIED on hardware (the Python package ran it with one real axis) |
| Simulator (slider, gripper, rod, pcon, rotary, unknown), SimLine, power / unplug faults, simulated RCON and ROBONET gateways | done |
| `iai-actuator-driver-cpp` command line (the Python CLI's commands, see below) | done, tested on the simulator |
| C ABI `include/iai_actuator_driver.h` and the shared library (`python build.py shared`; [docs/dev/NATIVE_ABI.md](../docs/dev/NATIVE_ABI.md)) | done, simulator-tested: C ABI 1.1 (1.0 frozen within major 1; 1.1 adds the serial framing options, 7 data bits, parity and 2 stop bits UNVERIFIED on hardware); sized inputs, fixed outputs, one name per function; test hooks in `include/iai_actuator_driver_testing.h` (no stability promise); the committed ABI manifest and JSON key list (`abi/`) with their checks; a C99 smoke program. VERIFIED for the calls the hardware runs used through the Python and C# bindings, on the PCON-CB with an RCP6 and the EC-GRBP8M gripper ([docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md#hardware-results)); the other calls are simulator-tested, UNVERIFIED on hardware |
| On real hardware: the EC-GRBP8M gripper through the IAI USB adapter (`iai-actuator-driver-cpp`) | **VERIFIED**: ports, scan, baud detection, status, identify, limits, alarm history, servo on, home, moves and jogs (exact to 0.01 mm), STOP mid-move, pause / resume / cancel-pause, a same-value `param set` skipped by the non-volatile guard, baud 230400 and back to 38400, software reset (confirmed, still homed) |
| On real hardware: the PCON-CB with an RCP6 (`iai-actuator-driver-cpp`) | **VERIFIED**: status, identify, limits, alarm reset (message alarm 0x069), servo on, moves and jogs, STOP mid-move, pause / resume, a same-value `param set No.10` skipped by the non-volatile guard, software reset (confirmed, still homed) |
| Cell, teach, parameter writes and gateway lines on real hardware | UNVERIFIED |

### Differences from the Python package

- **Gateway lines:** `Cell::status_all` reads the axes of an RCON port one by one; the all-axes monitor is
  `Port::gateway_monitor` (Python's `status_all(via_gateway=...)` chooses it).
- **Cell:** no `stop_all` when a `Cell` leaves scope on an exception (the Python `with Cell` block does that): call
  `close()` or `stop_all()` yourself. A `Port` keeps no registry of its axes (Python `Port.axes` /
  `Port.axis(slave)`): use `Axis(port, slave)`, so `Port::set_baud_rate` needs its slaves listed.
- **Info:** `unit_info()`, `backup()` and `run_check()` are free functions (`info.hpp`, `check.hpp`), not `Axis`
  members; `write_user_parameter()` (the older EC name) is `write_parameter()` with the same numbers.
- **Simulator:** the S-motion / filter patterns are recorded; the motion stays a trapezoid (as in the Python
  simulator).
- **Command line:** no coordinated motion and no separate `iai-check` program (use `check`).
- **C ABI:** coordinated motion runs on a cell handle, one request per call (no group kept across calls, no run of a
  plan made earlier); docs/dev/NATIVE_ABI.md, "Not in the ABI yet", lists the rest.

## Build

**With CMake 3.14+ (MSVC / MinGW / clang / gcc):**

```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release        # simulator-only tests; no test opens a serial port
```

Targets: `iai_actuator_driver` (static library), `ec_tests`, `iai-actuator-driver-cpp` (command line). In your own
project: `add_subdirectory(path/to/core)` and `target_link_libraries(your_app PRIVATE iai_actuator_driver)`. On
Windows the library links `winmm`, `setupapi` and `advapi32`.

**With zig, no CMake or MSVC** (zig's clang + libc++ via `python -m ziglang c++`, target x86_64-windows-gnu):

```shell
python build.py test                 # library, tests and CLI into build/, then run every test
python build.py test cell_ cli_      # only the tests whose names start with these prefixes
python build.py test --unit bus      # one unit alone: bus | model | sim (axis needs the other three)
python build.py check [FILE...]      # syntax-only check (default: every header)
python build.py shared               # the C ABI: build/iai_actuator_driver.dll + .lib (libc++ linked statically)
python build.py smoke                # the DLL, then tests/c_smoke.c and the frozen 1.0 program (abi/1.0) compiled as C99 against it and run
python build.py lib | tests | cli    # options: -j N, --release, --force, -v
```

The first compile after a zig install builds zig's libc++ cache; later builds are incremental.

`ec_tests` checks memory as it runs: freed memory is poisoned and held back from reuse for a while, so a read after
free fails in the test that does it, every time (`EC_TESTS_POISON=0` switches this off; it is off by default under
AddressSanitizer). `EC_TESTS_THREADS=1` reports the threads a test leaves running. Each run uses its own temp
directory, so several runs at once do not share files; a crash names the test that was running.

## Quick start

```cpp
#include "iai_actuator_driver/iai_actuator_driver.hpp"
using namespace iai_actuator_driver;

Axis ax = Axis::open("COM3");          // baud rate found automatically (115200, 38400, 9600, 230400, 57600, 19200)
ax.connect();                          // identity, ranges, Modbus control (taken at rest)
ax.servo(true);
ax.home();
ax.move_to(50.0, 20.0);                // 50 mm at 20 mm/s, checked against the unit's ranges, waits in position
ax.stop();
ax.servo(false);
```

- No hardware: `Axis::open(std::make_shared<SimulatedUnit>(SimProfile::Gripper))`.
- Several axes: `Cell::open({"COM3", "COM4"})` (or `Cell::open(SimLine::sim_ports(3))`), then `servo_all`,
  `home_all`, `move_all({{"COM3:1", 10.0}, {"COM4:1", 20.0}})`, `wait_all`, `stop_all`. Ports run in parallel
  threads; a link fault on one axis calls `on_fault` and stops the others (`stop_on_fault`).
- Errors derive from `EcDriverError` (`errors.hpp`). Every refusal happens before anything is sent and says so
  ("nothing sent" / "nothing written"). Every unit is driven the same way, with the ranges read from it, whatever
  its model or unit code.
- Logging is silent by default: `set_log_handler(...)` (`log.hpp`).

## C ABI and the shared library

`include/iai_actuator_driver.h` is a plain C99 header over this core, for every language that is not C++: C, C#
(P/Invoke), Python (ctypes), LabVIEW (Call Library Function Node).

- Prefix `iad_`. Opaque handles (`iad_axis*`, `iad_cell*`) come from the open functions and are released by the close
  functions (closing twice or `NULL` is safe).
- Every function returns an `iad_result` (0 = `IAD_OK`, negative = the error kind of the C++ exception; the message
  from `iad_last_error_message()`). No C++ exception crosses the boundary.
- Plain structs with fixed-size fields carry status, limits and identity (input structs carry their size, so a minor
  version can add fields); UTF-8 JSON into a caller buffer carries the rich data.
- `iad_axis_open_sim` and `iad_cell_open_sim` run every function on the simulator.

The model (handles, errors, threads, versioning, loading from each language, distribution) is in
[docs/dev/NATIVE_ABI.md](../docs/dev/NATIVE_ABI.md); the header documents each function.

| File | Role |
|---|---|
| `include/iai_actuator_driver.h` | The ABI (the only header a caller needs) |
| `include/iai_actuator_driver_testing.h` | The test hooks the same library exports (clock, simulator hooks, time limits): no stability promise, a version of its own |
| `abi/manifest.py`, `abi/manifest.txt`, `abi/manifest-1.0.txt`, `abi/json_keys.json` | The ABI manifest generator (`python build.py manifest`), its committed output and the frozen 1.0 baseline the output is held against; the committed key list of every JSON output |
| `src/c_api.cpp`, `src/c_api_*.cpp` | The exported functions and the struct layout and size checks, by area (axis, cell, bus, simulator, recovery, information, ...) |
| `src/c_api_coord.cpp` | Coordinated motion (the JSON request) |
| `src/c_api_impl.hpp`, `src/c_api_support.cpp`, `src/c_api_ext_support.cpp` | Handle registry, sized-struct reader and writer, exception-to-code mapping, thread-local last error, event queue, conversions |
| `src/iai_actuator_driver.rc.in` | The Windows version resource of the DLL: the core's version and both ABI versions |
| `tests/test_c_api*.cpp`, `tests/test_capi_*.cpp` | The `capi_` tests (in `ec_tests`, run by `python build.py test`), among them the sized structs, the defaults and the names |
| `tests/c_smoke.c` | A C99 program against the DLL (`python build.py smoke`; CMake test `iad_c_smoke`) |

Build the DLL with zig:

```shell
python build.py shared              # build/iai_actuator_driver.dll + iai_actuator_driver.lib (+ .pdb)
python build.py shared --release    # -O2
python build.py smoke               # the DLL, then the C99 smoke program against it
```

The DLL exports the `iad_` functions only and imports only Windows' own DLLs (zig links libc++ statically), so it
needs no runtime on Windows 10 / 11. With CMake, `-DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON` adds the target
`iai_actuator_driver_shared` (output name `iai_actuator_driver`; on MSVC the import library is
`iai_actuator_driver_imp.lib`). The Linux CMake build runs in CI (gcc); the MSVC and MinGW CMake builds have not
been run.

```c
#include "iai_actuator_driver.h"

iad_axis* ax = NULL;
iad_status st;
iad_move_options move;
iad_move_options_init(&move);
move.speed = 20.0;                                              /* mm/s */
iad_result r = iad_axis_open("COM3", NULL, NULL, &ax);          /* baud rate detected */
if (r == IAD_OK) r = iad_axis_connect(ax, 1, NULL);             /* identity, ranges, Modbus control */
if (r == IAD_OK) r = iad_axis_servo(ax, 1, NULL, NULL);          /* NULL options: the defaults */
if (r == IAD_OK) r = iad_axis_home(ax, NULL, NULL);
if (r == IAD_OK) r = iad_axis_move_to(ax, 50.0, &move, &st);    /* checked against the unit's ranges, waits */
if (r != IAD_OK) fprintf(stderr, "%s: %s\n", iad_result_name(r), iad_last_error_message());
iad_axis_close(ax);                                             /* NULL-safe */
```

Bindings on this DLL: C# under `bindings/dotnet` ([bindings/dotnet/README.md](../bindings/dotnet/README.md)) and
Python `bindings/python/iai_actuator_driver/native.py` (ctypes; it finds `core/build/iai_actuator_driver.dll` in a
source checkout). Both require ABI 1.0 and apply the loader rule of the header (`IAD_ABI_COMPATIBLE`: the same major
and at least their minor). The calls the hardware runs used through both bindings are VERIFIED
([docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md#hardware-results)); the rest are tested on the simulator only.

## Non-volatile memory

The units' non-volatile memory lasts about 100,000 write cycles (IAI). So parameter commits, position-table writes and
teach first compare with what the unit holds.

- When nothing changes, nothing is written or committed and the result says `skipped` with the note "unchanged:
  nothing written (one non-volatile write cycle saved)". A parameter commit still runs when this axis has uncommitted
  RAM writes from `commit = false` calls. `force = true` writes anyway.
- An unchanged row is skipped before the value checks (stroke, speed and acceleration caps, push force): nothing is
  written either way, so a row stored at 53 mm/s is skipped on an axis opened with `max_speed_mm_s` 10 (a same-value
  teach too). With `force = true` every check applies.
- A refused value names the row, the field, its unit and whose limit it breaks, e.g. "row 0 stored speed 53 mm/s is
  above this axis's max_speed_mm_s 10 mm/s; open the axis with a higher max_speed_mm_s (command line: --max-speed or
  --unit-max) to write it".
- `Axis::nonvolatile_writes()` counts the writes; above 60 in an hour the driver logs one warning per hour (never a
  refusal). For targets that change every cycle use `move_to`: direct moves are RAM only.

## Stored rows and the EC position mode

An EC keeps three rows (0 backward end, 1 forward end, 2 middle; `max_positions()` reports 3: the rows it can read and
store) but runs rows 0 and 1 only in 2-position mode (user parameter 9 = 0 or 3); 3-position mode (P9 = 4) runs rows
0-2. The EC-GRBP8M gripper in 2-position mode rejected row 2 with 0x45 cmd 6 code 0x11 "1st operand error"; nothing
moved, no alarm (VERIFIED). So on an EC whose P9 is read (once, cached; read again after a commit or a software reset):

- `move_to_position` (CLI `goto`) refuses an unused row with SoftLimitError before anything is sent: "row 2 (middle)
  is not used in 2-position mode (P9 = 0: 2-position (double solenoid)): the unit runs row 0 (backward end) and row 1
  (forward end) only; nothing sent".
- `teach_position` (both methods) and `write_position_table` write it with a WARNING log line and a line in
  `PositionWriteResult::warnings` ("... written anyway (e.g. to prepare a switch to 3-position mode)"); no opt-in.
- When P9 is not readable or holds an unknown value, every row is offered and the unit's own answer decides: a 0x11
  rejection of cmd 6 is thrown as SoftLimitError ("row 2 (middle) is not used in the unit's position mode: the unit
  rejected it (0x45 cmd 6 code 0x11 1st operand error); nothing moved. ..."), the rejection nested
  (`std::rethrow_if_nested`), and the decel stop of every failed move is sent as usual.
- A CON unit is not limited by P9 (class 8 item 2 is another parameter there): its rows are what MAXP and the
  directory say (64 on the PCON-CB). 0x9800 with an unused row is UNVERIFIED on hardware.

After a software reset PMSS depends on the unit: the EC-R6S rod dropped it to 0, the EC-GRBP8M gripper and the PCON-CB
kept 1 (VERIFIED). The restart is then confirmed by the servo, switched off before the reset, coming back on by itself.
`Status::manual_mode()` is the unit's MANU / AUTO bit (DSSE b13 RMDS), not PC vs PIO control: that is `modbus_enabled`
(PMSS).

## Command line

```text
iai-actuator-driver-cpp ports                                    list serial ports, IAI adapters first (opens nothing)
iai-actuator-driver-cpp [--ports COM3,COM4 | --sim] scan          find the answering axes (READ-ONLY)
iai-actuator-driver-cpp stop-all [--ports COM3,COM4 | --sim] [--slaves 1-16] [--broadcast]
                                                                 decel STOP on every axis found (not a safety function)
iai-actuator-driver-cpp (--port COMx | --sim [--sim-profile NAME]) [--slave N | --axis N] [--baud RATE] [--json] COMMAND
  status | identify | limits | alarms                         READ-ONLY
  info [--section S]...                                       everything the unit tells about itself (READ-ONLY)
  monitor [--count N] [--interval S]                          monitor block reads (READ-ONLY)
  monitor --watch [--interval S] [--duration S]               the status monitor and link events (READ-ONLY)
  line [--monitor]                                            the line, its rules, the gateway status (READ-ONLY)
  backup [FILE] [--rows used|all|0,1,2]                       read-only JSON backup (FC03 / FC11 only)
  check [--motion] [--report FILE]                            compatibility self-test (read-only unless --motion)
  servo on|off | home [--method coil|iai] | alarm-reset [--method coil|iai]
  mode pc|pio [--safety [on|off]]                             action mode: PC (PMSL coil) or PIO (0x45 cmd 0x0C)
  move POS [--speed V] [--accel G] [--decel G] [--accel-pattern trapezoid|s_motion|filter] [--band B]
       [--no-wait] [--fast] [--change-speed V | --change-accel G] [--change-after S]
                                                              absolute move (mm, or deg on a rotary axis)
  jog DELTA [--speed V] [--accel G] [--fast]                  relative move from the measured position
  push POS BAND PCT [--toward-home] [--speed V]               approach POS, push over BAND at PCT % force
  stop | pause | resume | cancel-pause
  goto N [--speed-pct P] [--allow-fast] [--no-wait]           run stored row N (an EC: rows its position mode runs)
  teach N [--force] [--method table|coil]                     store the current position as row N (non-volatile)
  param list | param get KEY | param set KEY VALUE [--force] [--no-commit]
  baud RATE [--method direct|break] | swreset
  recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]
                                                              bring the axis back after a power or link loss
Caps: --stroke MIN MAX, --max-speed V, --max-accel G, --max-push PCT (default 100 units/s, 0.3 G, 70 %, always
lowered to the unit's own maxima), --unit-max (use the unit's own maxima).
Line: --line auto|direct|rcon|robonet (default auto: probed once, read-only), --position-unit 0.01|0.001 (RCON
parameter No.207, needed before mm-based motion there).
Strategy: --strategy coil|iai|iai2|register (servo / home / alarm reset path; default coil, register behind RCON).
Global: --timeout S, --retries N (default 3), --log-frames, --help, --version.
```

Exit codes: 0 done; 1 the command ran but reports a failure (scan found nothing, a check failed, `line` could not read
the gateway); 2 a driver or usage error (the message on stderr); 130 interrupted (Ctrl+C sends a decel stop first;
`monitor --watch` ends normally on Ctrl+C). `recover` never moves the axis: it reports what is left to do (home,
alarm, the position against `--target`).

Example on the simulator:

```text
iai-actuator-driver-cpp --sim --sim-profile gripper limits
iai-actuator-driver-cpp --sim --sim-profile gripper param set P5 0
  P5 Smooth accel/decel setting (8.14 @0x081A) = 0:Disable (raw 00000400): unchanged: nothing written
  (one non-volatile write cycle saved); --force writes it anyway
```

Each `--sim` run starts a fresh simulated unit. The hidden demo option `--sim-state FILE` keeps the unit's position
and position table in FILE between runs, so `move 7`, then `teach 1` twice shows the second teach
skipped. `--sim-line rcon` (or `robonet`) puts the simulated unit behind a simulated gateway, e.g.
`--sim --sim-line rcon --position-unit 0.01 line --monitor`.

## Simulator

`SimulatedUnit` is a `Transport` that answers frames like a unit, so the whole library, the tests and the CLI
(`--sim`) run without hardware. `FakeTransport` scripts raw frames for bus-level tests.

- **Profiles:** `slider`, `gripper`, `rod`, `pcon`, plus the simulator-only `rotary` and `unknown`. `SimLine` puts
  several simulated units on one in-memory line (`SimLine::sim_ports(n)`).
- **Simulated:** registers, coils, the motion model, the 0x9900 direct move, alarms, position table, parameters,
  command-profile moves (FC17, the 0x0F00 block, PCRF), push, IAI 0x48 (servo, home, decel stop, alarm reset), the
  MOD / TEAC teach coils, the automatic servo OFF and the EMG / drive cut / enable inputs.
- **Faults and lines:** power loss and unplugging (`power_off` / `power_on` / `power_cycle`, `unplug` / `replug`),
  and RCON / ROBONET gateway lines (`sim_rcon_line`, `sim_robonet_line`).
- **Unit behaviour:** an EC profile rejects 0x45 cmd 6 with 0x11 for a row its position mode does not run (row 2 on
  the 2-position gripper and rod); the gripper and the pcon profiles keep PMSS 1 across a software reset, the others
  drop it to 0.
- **Not simulated:** the motion of the S-motion / filter patterns (recorded only).

## Bus rules

- The port is opened once and kept open; the baud rate is changed in place. No line BREAK is ever sent implicitly
  (150 ms or more drops a unit to 9600 baud, DOC).
- One transaction at a time per port, at least 5 ms of silence between requests. Timeouts, CRC and frame errors are
  retried 3 times (manufacturer recommendation, DOC ME0162-12A p.4-2); exception replies, IAI rejections and transport
  errors never are.
- Per-request timeout from IAI's formula, never below 0.2 s, 0.35 s below 38400 baud and 0.5 s at 9600 or less.
- Baud change: broadcast `00 45 00 04 00 <code> 00 05` (ON-WIRE). Units boot at 38400; only 115200, 230400 and 38400
  have codes VERIFIED on hardware; another rate with an IAI code is sent with a WARNING log line (`code_verified`
  false), a rate with no code is refused (SoftLimitError, nothing sent). `allow_unverified_code` is accepted and
  ignored. 38400 (code 0x06) is unit-dependent: the EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod ignores
  it. The check after the switch decides: a unit that ignores it keeps answering at its old
  rate, the port is left there and VerificationError is thrown.
- Baud detection: one read per rate, in the order 115200, 38400, 9600, 230400, 57600, 19200.
- IAI 0x45 commands off the safe list need `allow_unsafe`; never-send commands (non-volatile writes, axis number,
  baud, alarm-list clear, initialisation, firmware update, inspection) need `allow_never_send`.

## Safety

- **This driver is not a safety function.** Keep a hardware emergency stop and guarding.
- **No watchdog.** If the PC or cable fails, a move in progress finishes. Only the hardware E-stop stops the axis
  independently of the PC.
- Test new setups unloaded, mid-stroke and slow.

Not supported: controllers with IAI's SEL protocol (XSEL, SSEL, ASEL, PSEL, MSEL, TTA), an ELECYLINDER on an
RCON-EC / REC unit, and a ROBONET in gateway mode (SW2 OFF). Behind an RCON gateway the axis takes the register
strategy (FC06 DRG1, no coils, no IAI commands; DOC, UNVERIFIED on hardware).
