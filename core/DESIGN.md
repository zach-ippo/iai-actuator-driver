# IAI Actuator Driver C++ core: design

How the C++17 core is laid out and the conventions it follows. The core is the one implementation of the driver;
Python and C# are thin bindings over its C ABI (`include/iai_actuator_driver.h`,
[docs/dev/NATIVE_ABI.md](../docs/dev/NATIVE_ABI.md)). Unofficial: not an IAI product.

The Python package's tests are the conformance specification: the core does what they require, and the matching C++
tests run against the C++ simulator. Hardware evidence (VERIFIED, ON-WIRE, DOC, UNVERIFIED) is in
[README.md](README.md#status) and [docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md).

## Scope

- **Base.** The bus (CRC, FC03/05/06/10/11, IAI 0x45, 3 retries, per-request timeout with the 9600 floor, minimum
  gap, broadcast baud change, baud detection), register map and status decode, alarms, identify / limits (ranges from
  the unit, no tested-model allow-list), Axis (connect, servo, home, move_to via 0x9900 checked before sending,
  move_by / jog, safe completion rule, stop, pause / resume / cancel_paused_move, change_speed, reset_alarm, alarm
  history, position table, stored-row moves, write_position_table / teach, parameters list / read / write with the
  guards, software reset, set_baud_rate), the non-volatile write guard, Cell (every axis found, ports in parallel, stop
  on fault), the simulator (slider / gripper / rod / pcon / rotary / unknown + SimLine) and the
  `iai-actuator-driver-cpp` CLI.
- **The areas below**, with the same checks before any frame, the same completion rules, messages and evidence words
  as the rest.

| Area | Headers | Sources |
|---|---|---|
| Motion extras: push, command-profile moves (FC17 0x0F00 + PCRF), acceleration patterns, change_speed / change_accel, move_sequence | `motion.hpp` (+ `axis.hpp`) | `axis_motion_extras.cpp`, `sim_profile.cpp` |
| Fast path: Completion, AxisMonitor, MoveHandle, enable_fast, suspend_bus | `monitor.hpp`, `move_handle.hpp` | `monitor.cpp`, `move_handle.cpp`, `axis_fast.cpp` |
| Power / link-loss recovery: track, check_link, LinkMonitor, recover, Cell recover_all; simulator power / unplug faults | `recovery.hpp` | `recovery.cpp`, `sim_faults.cpp` |
| Unit info, backup, the compatibility check | `info.hpp`, `check.hpp` | `info.cpp`, `check.cpp` |
| Gateway lines: rules and request guard, RCON / ROBONET gateways, line detection, simulated gateways | `line.hpp`, `gateway.hpp`, `sim_gateway.hpp` | `line.cpp`, `gateway.cpp`, `model_gateway.cpp`, `sim_gateway.cpp` |
| Command line: info, monitor (--watch), line, backup, check, recover, push, move --decel / --accel-pattern / --fast / --change-speed, stop-all | `cli/cli.hpp` | `cli/cli_extra.cpp`, `cli/cli_commands.cpp` |

Two process-wide hooks connect the areas without link cycles:

- The recovery module installs the monitor's link watch (`set_monitor_link_watch`: link_fault / link_state /
  check_link / recovering) once at start-up, so a running status monitor of an axis whose identity was read runs a
  periodic 22-word link check.
- The Axis reports deliberate changes (target, PC control, homing, software reset) through `recovery::hooks`.

Where the C++ API differs from the Python package is listed in the
[README](README.md#differences-from-the-python-package).

## Layout

```text
include/iai_actuator_driver/   public headers (the contract)        src/      implementation + private headers
tests/               harness.hpp, test_main.cpp, tests     cli/      the iai-actuator-driver-cpp command line
build.py             build with zig                        CMakeLists.txt  build for customers
```

## Layers and link dependencies

```text
header-only:  version errors log timing crc reader json(the Json value) registers(constants, word helpers)
              axis_name / default_scan_slaves / SerialPortInfo::is_iai   src/util.hpp
bus:          transport (Win32 / POSIX serial, list_ports)  modbus  commands  baud change  fake_transport  port
model:        registers(lookups) alarms status directory device_info(identify, limits, ranges) position
              parameters(tables, decode, plan_parameter_write) results to_json      -- pure: I/O only via UnitReader
sim:          SimulatedUnit (a Transport) + fixtures, SimLine                       -- frames only, header-only deps
axis:         Axis (+ non-volatile guard), Cell, CLI                                -- links everything
```

bus, model and sim link independently of each other (they share only header-only code), so each unit builds and
tests alone: `python build.py test --unit bus|model|sim`. The model layer never talks to a bus: it reads through the
`UnitReader` interface (`ModbusRtu::reader(slave)` in production, a map of words in its tests) and returns decisions
(`plan_parameter_write`, `make_ranges`) that the Axis executes.

## Conventions

- **Names.** Namespace `iai_actuator_driver` (register constants in `iai_actuator_driver::reg`). Types `PascalCase`,
  functions `snake_case`, constants `kName`. LF line endings. No third-party dependencies.
- **Units.** Positions in mm (deg on a rotary axis), speeds in units/s, accelerations in G, times in seconds
  (`double`). Field names keep the bindings' `_mm`, `_mm_s`, `_g`, `_s` suffixes.
- **Ownership.** Transports and buses are `std::shared_ptr` (a test keeps its `SimulatedUnit` while the bus uses it).
  `Axis`, `Cell`, `ModbusRtu`, `Port`, `SerialTransport`, `SimulatedUnit` and `SimLine` hide their state behind a
  pimpl. `Axis` and `Cell` are movable, not copyable; a Cell hands out `Axis&`.
- **Options.** Optional values are `std::optional` (Python `None`). Options structs replace keyword arguments
  (`MoveOptions`, `OpenOptions`, `ParameterWriteOptions`, `NvWriteOptions`, ...).
- **Errors.** The exception hierarchy of `errors.hpp` (Python names). Every refusal happens before anything is sent
  and says so ("nothing written"). `CellError` carries per-axis messages and `exception_ptr`s.
- **Logging.** `set_log_handler()`; sources `"iai_actuator_driver.<module>"`. Silent by default.
- **Thread safety.** One mutex per bus (transaction level) plus `sequence_mutex()` for multi-frame sequences on a
  shared port; each Axis guards its own state; `stop()` may be called from another thread while a move is awaited.
  The simulator is thread-safe. Cell runs one thread per port per group command.
- **Retries.** 3 on every bus (IAI's recommendation, DOC ME0162-12A p.4-2). Never retried: exception responses, IAI
  rejections, TransportError.
- **Non-volatile writes.** One guard (`src/axis_nv.cpp`) before every parameter commit, row write and teach;
  constants and texts in `results.hpp`, the rules in the [README](README.md#non-volatile-memory).

## Implementation units

Each unit owns its files; no file belongs to two units.

| Unit | Files | Owns headers |
|---|---|---|
| bus | `src/serial_win32.cpp serial_posix.cpp transport.cpp modbus.cpp modbus_tables.cpp baud.cpp commands.cpp fake_transport.cpp port.cpp line.cpp gateway.cpp`, `tests/test_bus_*.cpp`, `README.md` | errors, log, timing, crc, reader, transport, modbus, commands, fake_transport, port, line, gateway, json (the Json class), version, `src/util.hpp`, `src/monotonic_cv.hpp` (the condition variable of every timed wait) |
| model | `src/registers.cpp alarms.cpp alarm_tables.cpp status.cpp directory.cpp device_info.cpp limits.cpp position.cpp user_parameters.cpp con_parameters.cpp parameter_plan.cpp to_json.cpp model_gateway.cpp`, `src/model_detail.hpp` (internal declarations), `tests/test_model_*.cpp` | registers, alarms, status, directory, device_info, position, parameters, results, the `to_json` declarations |
| sim | `src/sim_unit.cpp sim_motion.cpp sim_fixtures.cpp sim_line.cpp sim_profile.cpp sim_faults.cpp sim_gateway.cpp`, `src/sim_fixtures.hpp`, `tests/test_sim_*.cpp` | sim, sim_line, sim_gateway |
| axis | `src/axis_core.cpp axis_motion.cpp axis_motion_extras.cpp axis_fast.cpp axis_config.cpp axis_nv.cpp cell.cpp monitor.cpp move_handle.cpp recovery.cpp info.cpp check.cpp`, `src/axis_impl.hpp`, `cli/*.cpp`, `cli/cli.hpp`, `tests/test_axis_*.cpp test_cell*.cpp test_cli*.cpp test_recovery*.cpp test_info*.cpp test_check*.cpp` | axis, motion, monitor, move_handle, recovery, info, check, cell, nonvolatile, iai_actuator_driver.hpp |
| capi | `src/c_api*.cpp`, `src/c_api_impl.hpp`, `tests/test_capi*.cpp`, `tests/test_c_api*.cpp`, `tests/c_smoke.c` (the C ABI over the other four; [docs/dev/NATIVE_ABI.md](../docs/dev/NATIVE_ABI.md)) | `include/iai_actuator_driver.h` |
| sel | `src/sel_frame.cpp`, `src/sel_frame.hpp` (internal: the SEL protocol's frames, checksum and hex fields), `src/sel_messages.cpp`, `src/sel_messages.hpp` (internal: the phase-1 message codecs), `src/sel_sim.cpp`, `src/sel_sim.hpp` (internal: `SimulatedSel`, a simulated SEL controller with a TTA profile and C++-only controls), `src/sel_link.cpp`, `src/sel_link.hpp` (internal: `SelLink`, the exchange over a Transport with the no-blind-resend retry rules, the re-query verifiers and the completion rule), `src/sel_shim.cpp`, `src/sel_shim.hpp` (internal: `StreamShim`, a byte-stream view of a Transport for the LF framing tests; all UNVERIFIED on a controller), `tests/test_sel_*.cpp` | none yet (nothing SEL in the C ABI) |

- The public headers are the contract: a unit changes one only when a declaration cannot be implemented as written,
  and reports the change.
- New files of a unit match its patterns in `build.py` (`UNITS`; e.g. `bus_*.cpp`, `model_*.cpp`, `sim_*.cpp`,
  `axis_*.cpp`, `c_api*.cpp`).
- Test names start with their unit or area (`bus_`, `model_`, `sim_`, `axis_`, `cell_`, `cli_`, `recovery_`, `info_`,
  `check_`, `capi_`, `sel_`). `tests/test_main.cpp` and `tests/test_harness.cpp` (the harness and the header-only
  helpers) belong to no unit.
