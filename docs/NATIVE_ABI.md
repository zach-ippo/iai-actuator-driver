# Native core and C ABI

The driver is implemented once, in C++ (`core/`), and exported through one C ABI: the header
`core/include/iai_actuator_driver.h` and one library (`iai_actuator_driver.dll`, `libiai_actuator_driver.so`). This page
is the contract and the model of that ABI. The header documents each function.

- C, LabVIEW and any language with a C FFI call the ABI directly. C++ programs can also link the core itself.
- The Python package (`bindings/python/`) and the .NET library (`bindings/dotnet/`) are thin layers over the ABI. Both
  bind every function of both headers, checked against the committed manifest in `core/abi/`.
- The Python package runs on the core only: its pure-Python fallback was removed after 1.0.x.
- `core/include/iai_actuator_driver_testing.h` declares the test hooks the same library exports. It has no stability
  promise.

IAI Actuator Driver is unofficial: it is not an IAI product. See the [README](../README.md).

## The 1.0 contract

The header is at C ABI 1.1. ABI 1.0 is frozen within major 1, and 1.1 only adds to it: the serial framing options
(`iad_transport_open_serial_options`, see [Transports and buses](#transports-and-buses)). Everything declared in
`iai_actuator_driver.h` is the stable ABI. The header's "Conventions" and "Stability" blocks repeat these rules where
they apply.

**Stability.** Nothing a caller built against an earlier minor can see ever changes.

- A function, struct, struct field, constant, enumeration value or JSON field is never removed, renamed, reordered,
  resized or changed in meaning. An exported name never changes.
- A minor only adds: functions, constants, error codes, a meaning for a reserved slot (zero keeps the earlier
  behaviour), fields at the end of a sized input struct, and JSON fields.
- A function whose signature must change gets a new name. The old one stays until the next major version.
- Every non-zero result is a failure: test `!= IAD_OK`. Treat an unknown negative code as a failure of its own kind
  (the bindings map it to their base driver error).

**The version rule.**

- `IAD_ABI_VERSION_MAJOR` and `IAD_ABI_VERSION_MINOR` are in the header. `IAD_ABI_VERSION` is `(major << 16) | minor`
  (usable in `#if`). `iad_abi_version()` returns the same number for the loaded library.
- Accept a library only when `IAD_ABI_COMPATIBLE(iad_abi_version())` holds: the same major, and a library minor at
  least the caller's. A different major is incompatible and is refused with a clear message.
- Every loader checks this before its first other call: the Python binding when it loads, the C#
  `NativeDriver.EnsureLoaded`, the C example and the C smoke test. A C, LabVIEW or Visual Basic caller does it by hand.
- The rule holds because an old library refuses a field it does not know (sized structs), a minor never changes what a
  zero-filled old call does (defaults), and results are only added (error codes).
- The Linux library's `SOVERSION` is the ABI major (`libiai_actuator_driver.so.1`). The Windows DLL's version resource
  carries the core's version and both ABI versions.

**Sized structs.** Every input struct starts with `uint32_t struct_size` (the size the caller was compiled with) and
`uint32_t reserved_header` (0). That covers the options and policy structs, the axis commands' options,
`iad_position_entry`, `iad_sequence_point`, `iad_axis_move`, `iad_bus_options`, `iad_transport_callbacks` and (1.1)
`iad_serial_options`. A minor adds fields at the end.

- The caller always sets `struct_size`: to `sizeof(...)` before an `_init` call or an output call
  (`iad_axis_get_options`, `iad_axis_read_position_table`), or to 0 by zeroing the struct (0 means the first size). A
  struct built by hand gets its `struct_size` first, then its fields.
- The first size is the struct's 1.0 size. `iad_bus_options` and `iad_transport_callbacks` keep an older first size
  (for `iad_transport_callbacks`: 96 bytes on 64-bit, without its last field `set_read_timeout`). A struct a minor adds
  has the size of that minor as its first size: `iad_serial_options` (1.1) 64 bytes. A library older than the struct
  does not have the function that reads it (a 1.0 library has no `iad_transport_open_serial_options`).
- The library reads `min(struct_size, its own size)`.
  - `IAD_ERR_INVALID_ARGUMENT`: a size below the first size, a non-zero `reserved_header`, or a size that cannot be real
    (not a multiple of the struct's alignment, or more than 4096 bytes past the library's own size: an unset field
    holding garbage). This is checked before any byte past the first size is read or an array stride is taken.
  - `IAD_ERR_UNSUPPORTED`: a non-zero byte past the library's own struct (a field of a later minor) or in a reserved
    slot. It is never silently ignored, so a caller that uses a field its library does not know is told so.
- An array (`iad_axis_move_sequence`, `iad_cell_move_each`) takes its stride from the first element's `struct_size`,
  bounded the same way. Every element has that size.
- The library writes a sized struct (`_init`, `iad_axis_get_options`, `iad_axis_read_position_table`) up to the
  caller's `struct_size` when it knows that size, else up to the first size, and leaves the size written in
  `struct_size`. So it never writes past the caller's struct. A caller that uses a field added after the first size
  sets `struct_size = sizeof(...)` before `_init`. A `struct_size` never set (stack garbage that happens to be a known
  size) is the caller's error.
- No input struct is embedded in another: `iad_axis_move` has the move's fields itself, and `iad_position_write_result`
  has the row's fields itself.

**Fixed output structs.** An output struct (status, limits, results, summaries, events) keeps one size for the whole
major version. A minor may only give a reserved slot a meaning, with zero as the earlier behaviour. So a caller need not
initialise an output (LabVIEW) and ignores its reserved slots. Reserves: `iad_status` 64 bytes (it is embedded in eight
other outputs), `iad_range` 32, `iad_limits` 64, `iad_event` 32, every other output at least 32. New output data comes as
a JSON field (for example a new enforced range in `iad_axis_limits_json`) or as a new struct with a new function.

**Layout.** No implicit padding: 8-byte fields first, `int32_t` fields in even numbers (an explicit `reserved0` where
needed), text arrays in multiples of 8 bytes, reserved space at the end. A packed layout (32-bit LabVIEW clusters) and
the natural one (C, C#, ctypes) are therefore identical. `core/src/c_api.cpp` checks every size, alignment and key offset
at compile time, the manifest records every field, and the bindings' tests check the same numbers.

**Text and axis names.** Text fields are fixed `char` arrays: UTF-8, NUL-terminated, cut at a character boundary when
longer. The JSON form has the whole text. `IAD_AXIS_NAME_SIZE` is 128 bytes (room for Linux `/dev/serial/by-id/...`
ports). An input name that does not fit (`iad_axis_move.axis`) is refused, never cut. When an output name was cut,
`iad_cell_result.index` and `iad_event.index` identify the axis: the position in the requested list (the cell order for
all axes); for an event, the axis's position in the cell as `iad_cell_axis_name` names it (`IAD_INT_UNKNOWN` on an
axis's own queue).

**Unknown values.** An unknown output (the core's `std::optional`) is NaN for a `double`, `IAD_INT_UNKNOWN`
(`INT32_MIN`, never -1) for an `int32_t` and `IAD_UNKNOWN` for an `iad_tribool`. In inputs the same values mean "not
set" (the default).

**Booleans.** `iad_bool` is an `int32_t`, not C `bool`, so it has the same size in every language and in C# struct
marshalling. As an input, 0 is false and anything else true; as an output it is 0 or 1. `iad_tribool` is `IAD_FALSE` 0,
`IAD_TRUE` 1, `IAD_UNKNOWN` 2 (not -1, which is `True` in Visual Basic). As an output it is 0, 1 or 2. As an input, 0 is
false, 2 is "unknown / keep what is there" (`iad_axis_note_control`, the `safety` of `iad_axis_set_action_mode`, the
`lose_home` of the simulator's power calls), and any other value is true. So a Visual Basic `True` (-1) and a C `!= 0`
both mean true.

**Defaults.** One rule for every input.

- The value 0 of an enumeration is its default: `IAD_METHOD_DEFAULT`, `IAD_ACCEL_DEFAULT`, `IAD_COMPLETION_DEFAULT`,
  `IAD_LINE_AUTO`, `IAD_TEACH_TABLE`, `IAD_BAUD_DIRECT`. A push `direction` of 0 is +1. `IAD_METHOD_*` is `DEFAULT` 0,
  `COIL` 1, `IAI` 2, `IAI2` 3, `REGISTER` 4. `IAD_METHOD_DEFAULT` is the axis option in a call and COIL in
  `iad_axis_options.method`; `set_strategy`, `strategy` and `get_options` always use a concrete method.
- A field a later minor adds reads 0 as off or as the earlier behaviour, so a caller that never set it keeps the older
  meaning.
- A `double` is NaN for its default, and 0 is a value checked like any other (a move speed of 0 is
  `IAD_ERR_SOFT_LIMIT`). So fill an options struct with its `iad_*_options_init` first: a zeroed one is not the defaults
  wherever 0 is a value (caps and times use NaN for "the unit's own maximum"). A NULL options pointer means the defaults.
- `iad_axis_move` and `iad_sequence_point` have no `_init` and are filled field by field. There NaN or 0 is the default
  of a speed, acceleration, band or timeout, so a zeroed struct with its `struct_size` set is all defaults, and a Visual
  Basic or LabVIEW caller need not write a NaN. The bindings keep the managed APIs' rule that an explicit speed,
  acceleration or band of 0 is an error (`SoftLimitError`, `SoftLimitException`), never "the default".
- Output enumerations have 0 = none (`IAD_ALARM_LEVEL_NONE`, `IAD_EVENT_NONE`, an event source 0 = not stated).

**Calling convention.** `IAD_CALL` is on every function and every callback type. It is `__stdcall` on 32-bit Windows
(x86), where 32-bit Visual Basic, VBA and VB6 can call nothing else and `Declare` and the default `DllImport` assume it.
It expands to nothing everywhere else (x64, arm64 and arm32 Windows, Linux, macOS), where the platform has one
convention. The 32-bit Windows build is not made yet; it will export the undecorated names. Bindings use ctypes
`WinDLL` / `WINFUNCTYPE` on 32-bit Windows and `CDLL` / `CFUNCTYPE` otherwise, and C# `CallingConvention.Winapi`.

**Names.** Each function exists once, under one name. There are no suffixed variants (`_ex`, `_method`, `_values`,
`_completion`, `_flags`, `2`): such options are parameters of the base function. The axis commands likely to take more
settings take a sized options struct: `iad_servo_options`, `iad_home_options`, `iad_stop_options`,
`iad_wait_idle_options`, `iad_change_options` (speed and acceleration changes) and `iad_cancel_paused_move_options`. A
handle type's functions carry its name: `iad_axis_*`, `iad_cell_*`, `iad_bus_*`, `iad_move_handle_*`.

**Experimental items.** An item tagged `[EXPERIMENTAL]` in the header has behaviour that is not settled (UNVERIFIED on
any unit): `IAD_METHOD_IAI2`, `IAD_TEACH_COIL` with `iad_axis_teach_position`'s coil method, the 0x48 function code of
`iad_bus_iai_command`, `iad_bus_broadcast_iai_command` and `blend_band`. It stays exported and declared for the whole
major version, but a minor may change its behaviour or make it return `IAD_ERR_UNSUPPORTED`. Do not build on it without
a fallback. The callback transport (`iad_transport_open_callbacks`) is not experimental: it is stable with its
documented rules.

**The testing header.** `iai_actuator_driver_testing.h` has no stability promise: any release may change, rename or
remove its functions. Its own version, `IAD_TESTING_ABI_VERSION` / `iad_testing_abi_version()`, must match exactly. The
bindings check it before their first call into that header, so a library with another testing version still serves
everything else. Production code never needs it. See [The testing header](#the-testing-header).

**JSON outputs.** Within a major version a JSON field is only added, never removed, renamed or retyped (an integer stays
an integer, a float a float). `null` means unknown. A caller ignores keys it does not know. This holds for every `_json`
function of the stable header, for `iad_call_json` and for the error JSON. `core/abi/json_keys.json` lists the fields and
types of every JSON output, and under "requests" the keys the parser of every JSON request of the stable header reads.

**What checks it.**

- `core/abi/manifest.txt` holds every declaration of both headers (constants, types, struct fields, callbacks, function
  signatures, the `IAD_CALL` rule), generated by `core/abi/manifest.py`. `python build.py manifest` compares the headers
  with it and both bindings with the headers, parameter by parameter; `test_native_binding.py` runs the same check. It
  refuses a removed or changed line of the stable header (`--allow-break` only with a new major).
- `core/abi/manifest-1.0.txt` is the 1.0 manifest, frozen and never edited again. `test_abi_manifest_gate.py` holds
  `manifest.txt` against it: no removed or changed line of the stable header, whatever `--allow-break` was given, and
  the stable section differs from the baseline only once `IAD_ABI_VERSION_MINOR` is above 0.
- `bindings/python/tests/test_abi_json_keys.py` samples every JSON output on the simulator and fails on a removed,
  renamed or retyped field.
- `bindings/python/tests/test_abi_request_keys.py` reads the request parsers in `core/src` and fails when one reads a
  key that `json_keys.json` "requests" or the function's header comment does not name, or the list names a key no
  parser reads.
- The core's `capi_` tests, `core/tests/c_smoke.c` (plain C99 against the DLL) and each binding's tests cover the
  behaviour.
- After a deliberate change, `python build.py manifest --update` and `test_abi_json_keys.py --update` rewrite the
  committed files for review.

**Programs built for 0.x.** A binding, example or program built from a 0.x header refuses a 1.0 library and does not
work with it. Rebuild it against the 1.0 header.

## Evidence

Evidence words are those of the other documents: VERIFIED (hardware), ON-WIRE, DOC, UNVERIFIED. The step-by-step results
are in [COMPATIBILITY.md](COMPATIBILITY.md).

| What | Unit | Evidence |
|---|---|---|
| The C++ core's command line | EC-GRBP8M gripper, PCON-CB with an RCP6 | VERIFIED (`core/README.md`) |
| The 1.0 layout, through the Python package on the core and the C# command line (on the C# `Axis`) | EC-GRBP8M gripper | VERIFIED for the calls of the 1.0 acceptance run (below) |
| A pre-1.0 layout, through both bindings | PCON-CB with an RCP6 (IAI USB adapter) | VERIFIED: open with baud detection, identify, status, limits, servo, moves, STOP mid-move, pause / resume, a same-value parameter write skipped by the non-volatile guard, baud 230400 and back to 38400. Both bindings gave the same positions |
| A pre-1.0 layout, through the Python package and the C# `Axis` | EC-GRBP8M gripper | VERIFIED: in Python, the calls of the 1.0 run plus recovery after a 250 ms line break; in C#, only the steps its public `Axis` had then |
| Everything else, including recovery after a line break on the 1.0 layout | | UNVERIFIED on hardware (simulator-tested) |

Calls of the 1.0 acceptance run:

- Both: open and identify, unit info, status, limits, absolute and relative moves, jog, stop (the C# command line from a
  second process), a deceleration of its own, a missed push, the position table and parameters (read), backup, track and
  link check, the PC/PIO toggle, the self-test.
- Python only: servo off and on, the speed / stroke / acceleration refusals, fast mode with the monitor's events, a move
  handle polled and one cancelled, pause / resume, a speed change during a move, an acceleration pattern, a same-value
  write skipped by the non-volatile guard, baud detection.
- C# command line only: the status monitor for a second, `recover` with the link up. Its fast and speed-change steps
  checked the arrival only.

## Files

| File | Role |
|---|---|
| `core/include/iai_actuator_driver.h` | The C ABI: plain C99, prefix `iad_`. The only header a C, C#, Python or LabVIEW caller needs. |
| `core/include/iai_actuator_driver_testing.h` | The test hooks (no stability promise, own version). Includes the stable header. |
| `core/abi/` | `manifest.py` and its committed output `manifest.txt`; `manifest-1.0.txt`, the frozen 1.0 baseline; `json_keys.json`, the key list of every JSON output. |
| `core/src/c_api.cpp` | The exported functions over the C++ core, the layout assertions and the sized-struct checks. Built into the static library too. |
| `core/src/c_api_*.cpp`, `c_api_objects.hpp`, `axis_state.cpp`, `sim_controls.cpp`, `sim_gateway.cpp` | The other exported functions by area: `ext`, `coord`, `bus`, `sim`, `tables` (`iad_call_json`), `axis`, `monitor`, `line`, `cell`, `recovery` (with the port-list hook), `info`, `results`, `nv_testing`. |
| `core/src/c_api_impl.hpp`, `c_api_support.cpp`, `c_api_ext_support.cpp` | Private helpers: the handle registry, the exception-to-code mapping, the thread-local last error, the per-handle event queue, struct, option and JSON conversions. |
| `core/src/iai_actuator_driver.rc.in` | The DLL's Windows version resource (filled in by `build.py` and CMake). |
| `core/include/iai_actuator_driver/sim_hooks.hpp`, `timing.hpp`, `virtual_clock.hpp` | `HookedSimUnit` (a `SimulatedUnit` with the test hooks of the Python `SimulatedEC`); the clock seam and the virtual clock for tests. |
| `core/tests/test_c_api*.cpp`, `test_capi_*.cpp`, `c_smoke.c` | The `capi_` tests (in `ec_tests`) and a C99 program that calls the DLL (`python build.py smoke`). |
| `bindings/dotnet/src/IaiActuatorDriver/Interop/` | The C# binding, internal to the library. Public for diagnostics: `NativeDriver`, `NativeErrors`, `NativeResult`. Tests in `bindings/dotnet/tests/IaiActuatorDriver.Tests/Interop/`. |
| `bindings/python/iai_actuator_driver/native.py` | The Python binding: `NativeAxis`, `NativeCell`, `NativeTransport`, `NativeBus`, `NativeSim`, `NativeSimLine`, the clock functions, `available()`. Tests in `bindings/python/tests/test_native_binding.py`. |
| `core/build/iai_actuator_driver.dll`, `.lib` | The Windows library and its import library (`python build.py shared`). |

## The model

### The header by topic

The header declares the types first (export and version, basic types and handles, result codes, constants, callback
types, options, data, the structs of motion, monitors, events, recovery and the self-test, and those of transports and
buses), then the functions in sections named by topic. Its "Layout" block lists them. The declarations keep the order of
ABI 1.0, which the manifest records and the bindings bind, so a topic can have several sections. A new function goes
into its topic's section. The functions of ABI 0.1 are those of the sections `library` to `cell`. This page follows the
same topics.

| Topic | Sections of the header | Here |
|---|---|---|
| The library and every call | `library`, `option defaults`, `library: log sink, option defaults`, `library: callback error, log level, defaults`, `stateless functions`, `library: per-thread results; cell: link monitor pass` | [Handles and lifetimes](#handles-and-lifetimes) to [Per-thread results and caller aborts](#per-thread-results-and-caller-aborts) |
| The axis | `axis: session` to `axis: unit` (eight sections), `axis: push, sequence, acceleration change`, `axis: move handles`, `axis: status monitor, events`, `axis: fast mode`, `axis: recovery`, `axis: unit information, backup, self-test`, `axis: line, modes, strategy, cancel`, `axis and cell: cancel latch`, `axis: state without I/O`, `axis: status monitor holds, port owner`, `axis: recovery sessions, link state`, `axis: directory readers, self-test lines` | [Events](#events) to [Directory readers, self-test lines, input ranges](#directory-readers-self-test-lines-input-ranges) |
| The cell | `cell`, `cell: pause, cancel, monitors, recovery, lines, backup`, `cell: coordinated motion`, `axes and cells on transports and buses`, `ports and cells over the caller's buses` | [Cells and group commands](#cells-and-group-commands), [Coordinated motion](#coordinated-motion) |
| Transports and buses | `transports`, `buses`, `buses: bus lock`, `buses: preemptible reads`, `buses: status monitors`, `buses: broadcasts`, `buses: the line`, `buses: response timing`, `buses: shutdown, release`, `buses: another transport` | [Transports and buses](#transports-and-buses) |
| Lines, gateways and ports | `lines and gateways`, `ports and cells over the caller's buses`, `ports: baud detection, device info` | [Lines and gateways](#lines-and-gateways), [Ports and cells over the caller's buses](#ports-and-cells-over-the-callers-buses) |
| Simulators | `simulators`, `simulators: ports, gateway lines` | [Simulators](#simulators) |
| Test hooks | `iai_actuator_driver_testing.h` | [The testing header](#the-testing-header) |

### Handles and lifetimes

- `iad_axis*` is one axis (an ELECYLINDER unit or a CON controller at one slave address). `iad_cell*` is a group of axes
  over one or more ports (no total limit unless `max_axes`). Both are opaque.
- A handle is a token, not an address. The library keeps a registry of live objects keyed by never-reused 64-bit
  numbers, so a stale or doubled handle is detected instead of touching freed memory. Never dereference a handle.
- Open functions create handles: `iad_axis_open` (a serial port, baud rate detected or fixed), `iad_axis_open_sim` (a
  simulated unit), `iad_cell_open`, `iad_cell_open_sim`. `iad_cell_open_axis` opens an axis handle for one axis of a
  cell. `iad_axis_start_move_to` / `start_move_by` / `start_jog` give an `iad_move_handle*`, released by
  `iad_move_handle_close`. A handle of the wrong kind is refused.
- `iad_axis_close` and `iad_cell_close` release them. Closing `NULL` or an already closed handle returns `IAD_OK`. Any
  other call with a closed or unknown handle returns `IAD_ERR_INVALID_ARGUMENT`.
- `iad_axis_close` releases a STOP / pause the axis holds, closes the port when the axis owns it (an axis from
  `iad_axis_open`) and releases the handle. On an axis handle from `iad_cell_open_axis` it releases only the handle: the
  axis stays in the cell, unchanged. `iad_cell_close` closes every axis and port of the cell; its axis handles then
  return `IAD_ERR_INVALID_ARGUMENT` (closing them stays safe).
- A call running on another thread keeps its object alive until it returns. After a close it may fail with
  `IAD_ERR_TRANSPORT` (the port closed underneath).
- Close handles before unloading the library. A binding does it in its finalizer / `SafeHandle.ReleaseHandle`.
  Unloading with open ports leaves them open until the process exits.

### Results and errors

Every function returns an `iad_result` (`int32_t`): `IAD_OK` (0) or a negative error kind. Four functions cannot fail
and return their value instead: `iad_abi_version`, `iad_library_version`, `iad_last_error_message`, `iad_result_name`.
No C++ exception crosses the boundary: every exported function catches everything and turns it into a code and a
message.

| Code | Name | C++ class | Python | C# |
|---|---|---|---|---|
| 0 | `IAD_OK` | | | |
| -1 | `IAD_ERR_SOFT_LIMIT` | `SoftLimitError` (also `coord::InfeasiblePathError`) | `SoftLimitError` | `SoftLimitException` |
| -2 | `IAD_ERR_NOT_READY` | `EcNotReadyError` | `EcNotReadyError` | `EcNotReadyException` |
| -3 | `IAD_ERR_ALARM` | `EcAlarmError` | `EcAlarmError` | `EcAlarmException` |
| -4 | `IAD_ERR_MOTION_TIMEOUT` | `MotionTimeoutError` | `MotionTimeoutError` | `MotionTimeoutException` |
| -5 | `IAD_ERR_MODBUS_TIMEOUT` | `ModbusTimeout` | `ModbusTimeout` | `ModbusTimeoutException` |
| -6 | `IAD_ERR_FRAME` | `ModbusCrcError`, `ModbusFrameError` | `ModbusCRCError`, `ModbusFrameError` | `ModbusCrcException`, `ModbusFrameException` |
| -7 | `IAD_ERR_TRANSPORT` | `TransportError` | `TransportError` | `ModbusException` |
| -8 | `IAD_ERR_VERIFICATION` | `VerificationError` | `VerificationError` | `VerificationException` |
| -9 | `IAD_ERR_NONVOLATILE_REFUSED` | `NonVolatileWriteError` | `NonVolatileWriteError` | `NonVolatileWriteException` |
| -10 | `IAD_ERR_INVALID_ARGUMENT` | `std::invalid_argument` (also `coord::NoMotionError`), `std::out_of_range`, a bad handle / pointer | `ValueError`; kind `std::out_of_range`: `NativeOutOfRangeError` (a `ValueError` and an `IndexError`) | `ArgumentException`; kind `std::out_of_range`: `ArgumentOutOfRangeException` |
| -11 | `IAD_ERR_UNSUPPORTED` | `std::logic_error` (e.g. a line-break baud change on a transport that cannot send a break) | `NotImplementedError` | `NotSupportedException` |
| -12 | `IAD_ERR_INTERNAL` | anything else (`std::bad_alloc`, an unexpected exception) | `RuntimeError` | `InvalidOperationException` |
| -13 | `IAD_ERR_BUFFER_TOO_SMALL` | (the ABI's own: a text output did not fit) | handled inside the binding | handled inside the binding |
| -14 | `IAD_ERR_SERVO_BLOCKED` | `ServoBlockedError` | `ServoBlockedError` | `EcNotReadyException` |
| -15 | `IAD_ERR_MOTION_INCOMPLETE` | `MotionIncompleteError` | `MotionIncompleteError` | `MotionIncompleteException` |
| -16 | `IAD_ERR_SLAVE_EXCEPTION` | `ModbusSlaveError` | `ModbusException` | `ModbusSlaveException` |
| -17 | `IAD_ERR_COMMAND_REJECTED` | `IaiCommandRejected` | `IaiCommandRejected` | `IaiCommandRejectedException` |
| -18 | `IAD_ERR_UNSAFE_REFUSED` | `UnsafeCommandError` | `UnsafeCommandError` | `UnsafeCommandException` |
| -19 | `IAD_ERR_CELL` | `CellError` | `CellError` | `CellException` |
| -20 | `IAD_ERR_MODBUS` | any other `ModbusError` | `ModbusError` | `ModbusException` |
| -21 | `IAD_ERR_DRIVER` | any other `EcError` / `EcDriverError` (also `coord::CoordinatedMotionError`) | `EcError` / `EcDriverError` | `EcException` / `EcDriverException` |
| -22 | `IAD_ERR_RECOVERY` | `recovery::RecoveryError` | `RecoveryError` | `RecoveryException` (an `EcDriverException`; binding class) |
| -23 | `IAD_ERR_UNIT_REPLACED` | `recovery::UnitReplacedError` | `UnitReplacedError` | `UnitReplacedException` (a `RecoveryException`; binding class) |
| -24 | `IAD_ERR_FILE` | `std::ios_base::failure`, `std::filesystem::filesystem_error` (a backup file or directory) | `OSError` | `IOException` |
| -25 | `IAD_ERR_CANCELLED` | `WaitCancelledError` (`iad_axis_cancel`, `iad_cell_cancel`) | `KeyboardInterrupt` | `OperationCanceledException` |

The bindings raise the classes of the last two columns, so code written against the C# library or the Python package
catches the same types on the core. Where a language has no class of its own (C# has no servo-blocked or transport
exception yet), the binding uses the nearest base class shown. Codes are never renumbered.

**Reserved.** The codes from -26 down are kept for coming additions (the last code is -25), and the prefix `iad_sel_`
for the planned SEL header, `iai_actuator_driver_sel.h`, which gets a version of its own. Its codes come from the same
range: one numbering for every header of the library, and a code is never reused. Both bindings map a negative code
they do not know to their base driver error (`EcDriverError`, `EcDriverException`) and keep the code (`iad_code`,
`NativeErrors.ResultOf`), so a code that a newer library adds never breaks an older binding. Their tests check it.

**Last error.** A failing call sets the calling thread's last error; a call that succeeds clears it.

- `iad_last_error_message()` returns the message (the C++ `what()`, with the driver's notes such as "decel stop sent
  after this error"). It is valid until the next call on the same thread.
- `iad_last_error_json()` gives the details a caller may act on (below). It does not change the last error.
- `kind` names the C++ class even where several classes share a code (`InfeasiblePathError`, `NoMotionError`,
  `CoordinatedMotionError`, `GatewayModeError`, `LineRuleError` ...), so a binding can raise a class of its own. It
  keeps `std::out_of_range` (a value outside its range, a row, a slave on a line, a cell axis or port that is not there)
  apart from `std::invalid_argument`.

| Fields of `iad_last_error_json` | When |
|---|---|
| `code`, `name`, `kind`, `message`, `notes`, `stop_attempted` | always |
| `alarm_code`, `attempts`, `exception_code`, `function`, `command`, `rejection`, `reason`, `hint` | when the error carries them |
| `status` | `IAD_ERR_COMMAND_REJECTED`: the reply's status byte (0x45; null when unknown) or its four status words (0x48) |
| `cause` | a position-table row the unit rejects (0x45 code 0x11, returned as `IAD_ERR_SOFT_LIMIT`): the rejection's own error JSON (`IAD_ERR_COMMAND_REJECTED`) |
| `reasons` | servo blocked |
| `operation`, `axes` | a group command: `{name: {code, message, ...}}`, with every detail of each axis's error (notes, `alarm_code` ...) as a call of its own gives them |
| `report` | a recovery: its full report |
| `expected`, `found` | a replaced unit |
| `reasons`, `max_speed` | an infeasible path |
| `axis`, `cause`, `stop`, `result` | a coordinated run that was stopped |

### Threads

The ABI is as thread-safe as the C++ core, no more.

- Calls on different handles may run in parallel. Axes that share a port take turns: one bus mutex per port serialises
  their frames, and each axis guards its own state.
- One thread at a time commands a handle. Two commands on one handle from two threads are not made into one sequence:
  their frames may interleave.
- Meant to come from another thread while a thread waits in a move, home or `wait_idle` on the same handle:
  `iad_axis_stop`, `iad_axis_pause` and `iad_cell_stop_all` (a GUI stop button or a watchdog thread), and
  `iad_axis_cancel`, `iad_cell_cancel` and `iad_move_handle_cancel`. `iad_axis_next_event` / `iad_cell_next_event` may
  be called at any time. The software STOP is not a safety function: it goes over a serial line from a
  general-purpose PC and can be late, so a hardware emergency stop or STO is required wherever a late stop could hurt
  someone.
- A STOP goes first. A write that does not itself stop motion never reaches the wire after a STOP, pause or servo OFF
  to the same unit (or a broadcast STOP) that went while it waited for the port, was being prepared or was being
  retried: an axis command, a raw `iad_bus_write_*` and a broadcast write alike (a broadcast after a STOP to any unit
  of its port). The call fails with `IAD_ERR_NOT_READY` and sends nothing more. Reads are never held back. No command
  releases a STOP that a stop of another thread, or a group stop (`iad_cell_stop_all`, a coordinated group's stop),
  still holds: it is refused, nothing sent.
- The handle registry has its own mutex, held only to look a handle up (never during I/O).
- The last error is thread-local: read it on the thread that made the failing call.
- A cell's ports run on the core's worker threads. A group call returns when every port is done.
- Callbacks into the caller's code:
  - The log and frame sinks (`iad_set_log_sink`, `iad_axis_set_frame_sink`, `iad_bus_set_frame_sink`) run on the
    thread that logs (a cell worker, a monitor thread); a line or frame of a thread that holds a port's bus comes right
    after that thread lets the bus go, never with the bus lock held. A request and its reply therefore reach the frame
    sink together, once the request has ended: a time the sink takes itself is when it was called, not when the frame
    was on the wire.
  - The callbacks the caller installs (a transport's, a simulator's request hook, a clock) run on the thread doing the
    I/O, with the port's bus lock held.
  - A preemptible read's gate runs on the reading thread before each request attempt, without the bus lock.
  - A callback copies what it needs, returns quickly, lets no exception cross into the library and never calls the
    library, except `iad_callback_error`, `iad_sim_handle_frame` (from a request hook) and the cancel functions.
  - Every other callback of the core becomes a queued event ([Events](#events)).
- A hold of the bus lock (`iad_bus_lock`) and a preemptible scope (`iad_bus_preemptible_begin`) belong to the calling OS
  thread. A binding that moves a long call onto a helper thread (for Ctrl+C) runs the calls made inside a hold or a scope
  on the thread that took it; otherwise the helper waits for the lock that thread holds.

### Structs, options and timeouts

The rules are in [The 1.0 contract](#the-10-contract). This is how a caller applies them.

- **Plain structs** carry the frequent data with fixed-size fields: `iad_status`, `iad_limits` (eleven `iad_range`),
  `iad_identity`, `iad_position_entry`, `iad_parameter`, the write results, `iad_cell_result`, `iad_motion_caps`.
- **Filling an input** in C: `T x; x.struct_size = sizeof x; T_init(&x);`. The `_init` writes the defaults and the size.
  A zeroed `x` works too, but set `struct_size` whenever `x` uses a field added after the first size. A struct without
  an `_init` (`iad_axis_move`, `iad_sequence_point`) is zeroed, then `struct_size = sizeof(...)` is set, then the
  fields. A binding sets `struct_size` whenever it builds a struct and before every `_init`, `iad_axis_get_options` and
  `iad_axis_read_position_table` call.
- **Timeouts.** The `timeout_s` of a command that starts motion or a servo change (a move, home, servo, stored-row move,
  push, sequence, a change of the running move, the moves of `iad_cell_move_each` / `_move_all`, `iad_cell_home_all` /
  `_servo_all`) is NaN for the default or a finite number of seconds above 0.
  - 0, a negative value (-1 is "wait forever" in VB and LabVIEW) or an infinity is `IAD_ERR_INVALID_ARGUMENT`, and
    nothing is sent. There is no "wait forever". In `iad_axis_move` a 0 is the default. The C++ core refuses the same
    values with `SoftLimitError` ("nothing sent").
  - A coordinated request's `"timeout_s"` (`iad_cell_coord_run_json`, `iad_cell_coord_plan_json`; the wait for the last
    waypoint) is absent for the default (JSON has no NaN; a number past the double range is refused when the request is
    read) or a finite number above 0. 0 or a negative value is `IAD_ERR_INVALID_ARGUMENT` before anything is sent.
  - A stop's timeout (`iad_axis_stop`, `iad_cell_stop_all`) is not checked this way: a STOP always goes out.
- **Optional outputs.** An output pointer documented "may be NULL" is skipped when NULL. A move that does not wait leaves
  its status output unwritten.
- **Units**: positions in mm (deg on a rotary axis), speeds in units/s, accelerations in G, times in seconds.

### JSON

Rich or variable data is UTF-8 JSON written into a caller buffer: unit info, limits with their sources, alarm history,
parameters with their choices, the full result of a write, the cell's info, the serial port list, the last error. The
field names are those of the C++ `to_json` (`json.hpp`), which follow the Python edition's `to_dict`, so the editions'
outputs compare.

Every JSON function ends with `(char* buffer, size_t buffer_size, size_t* required)`.

- `*required` always gets the bytes needed, including the terminating NUL.
- When `buffer_size` is smaller, the call returns `IAD_ERR_BUFFER_TOO_SMALL` and writes `""` when `buffer_size > 0`.
  `buffer` may be NULL with `buffer_size` 0 to ask for the size.
- JSON functions are read-only, so calling again with a larger buffer is harmless (it repeats the read, if any).
  Bindings start with a 64 KiB buffer and grow it once.

**Kept results.** A too-small buffer never makes a caller repeat a write or a long operation.

- Operations that change a unit (parameter write, position-table write, teach, baud change, software reset) fill a fixed
  result struct, never only JSON. The full record (warnings, notes) is fetched afterwards, without I/O, by
  `iad_axis_last_result_json`. Push, fast mode, the link check, a recovery and the self-test do the same.
- Unit info and a backup (long reads) keep their JSON even when the buffer was too small.
- On a cell, track / check_links / recover / backup and a coordinated run (`iad_cell_coord_run_json`, the one JSON
  function that moves) keep theirs for `iad_cell_last_result_json`.
- For the result of a call on the calling thread, see [Per-thread results](#per-thread-results-and-caller-aborts).

**Numbers.** A whole float is written with a decimal point (`1.0`, as Python's `json.dumps`), so Python and C# readers
keep it a float. Numbers the Python edition holds as ints stay integers: a parameter's code, count or flag and its
`min` / `max`, a CON row's table limits and factory default, the `stored_rows` and `speed_override` ranges of
`iad_axis_limits_json`. Every JSON argument refuses a number outside the range of `long long` or `double` with
`IAD_ERR_INVALID_ARGUMENT`; it is never narrowed or turned into `null`.

**Parameters and unit info.**

- A named parameter's row (`iad_axis_parameters_json`, `iad_axis_read_parameter_json`) has the keys of the Python
  edition's `parameters()`: no raw-item fields (`unsigned`, `signed`, `words`, `named`), and a `symbol` or a `note` only
  when it has one. A raw item keeps every field, and its write result gives the value as an integer, as the read does.
- A parameter that is not offered is named `parameter 15` for a number and `parameter 'P15'` for a text key.
- A unit info's `axis_limits` has no `notes`. The readers' wording and the notes and warnings of a parameter write are
  the Python edition's (`IA-OS` where they name the tool).

### Stateless functions

`iad_call_json(function, args_json, ...)` (`src/c_api_tables.cpp`) runs the alarm and CON parameter tables and rules and
the unit-information decoders by name, JSON in and out. `"functions"` lists them, among them `info.input_ranges`,
`info.product_line` and `info.ec_parameter_table`.

- An unknown function is `IAD_ERR_UNSUPPORTED`.
- A nested argument of the wrong JSON type is `IAD_ERR_INVALID_ARGUMENT`. An integer outside its field's range (outside
  32 bits for an int field, a float outside the 64-bit range) is refused, never narrowed. Where the Python edition
  answers "unknown" for a number (`con_params.con_family`, `con_params.audit`), a huge one is unknown too.

### Per-thread results and caller aborts

Simulator tests only (UNVERIFIED on hardware).

- **A call's own result**: `iad_axis_last_result_json` / `iad_cell_last_result_json` give the handle's last result,
  which a call on another thread can replace (a link monitor's check). `iad_last_result_json` gives the result the last
  call **on the calling thread** kept.
  - Read it on that thread right after the call returned `IAD_OK` (for `iad_cell_recover_all` and `iad_cell_track_all`
    also `IAD_ERR_CELL`). `IAD_ERR_NOT_READY` when no call on the thread kept one. It does not change the last error.
  - A binding that runs `iad_axis_recover`, `iad_axis_self_test` or `iad_cell_recover_all` while link checks go on reads
    the report this way.
- **`iad_cell_check_links`** keeps `error_details` in its result (`{axis: the record of iad_last_error_json}`) for each
  axis whose check failed with an error that is not a link fault. So a binding raises that error's own class.
- **`iad_cell_link_monitor_check_now`**: one pass of the cell's link monitor on the calling thread, as a pass of its own
  thread: states, `passes` and errors are updated and each state change is queued as `IAD_EVENT_LINK_STATE` before it
  returns. An axis whose check threw, or that the monitor does not watch, gets state `"error"` and `ok` 0.
  `IAD_ERR_NOT_READY` when no monitor runs. It is a group command: refused inside a hold of a port's bus lock.
- **Caller aborts**: a simulator request hook may answer `IAD_SIM_ABORT` (4) and a transport callback may return
  `IAD_CALLBACK_ABORT` (2), after giving the text to `iad_callback_error`.
  - The call then fails with `IAD_ERR_DRIVER` (`EcDriverError`) carrying that text. It is never retried, and a link
    check or a recovery does not take it for a link fault. So an assert raised by a test unit or transport reaches the
    caller.
  - `IAD_SIM_FAIL` and any other non-zero callback result stay the port's failure (`IAD_ERR_TRANSPORT`).
  - In the core it is `CallerAbortError` (an `EcDriverError`; the error record's kind stays `EcDriverError`). The
    best-effort steps of a recovery, a unit-info read, a backup and the self-test let it through, so the call ends with
    it instead of noting it and going on.

### Events

The core's callbacks (the status monitor's `on_change` / `on_done` / `on_event`, the link and recovery events, the cell's
`on_fault`, the self-test's lines) are queued per handle instead of calling into the binding.

- `iad_axis_next_event(axis, timeout_s, &event)` / `iad_cell_next_event` return the oldest queued event, waiting up to
  `timeout_s`; `IAD_EVENT_NONE` when none came. A binding runs one loop per handle on a thread of its own and turns the
  events into its language's events there, so no foreign thread ever enters the binding's runtime.
- Kinds: `IAD_EVENT_STATUS` (a monitor sample that changed), `IAD_EVENT_DONE` (the monitor saw the move done by the fast
  rule), `IAD_EVENT_LINK` (a link or recovery event: "link_lost", "unit_restarted", "unit_replaced", "recovered"),
  `IAD_EVENT_FAULT` (the cell's `on_fault`), `IAD_EVENT_LINK_STATE` (a link monitor saw a state change) and
  `IAD_EVENT_CHECK_ITEM` (one line of the self-test).
- A status / done event has the status in `event.status`. `axis` names the axis ("COM3:1"); `index` is its position in
  the cell.
- `source` says what queued the event (`IAD_EVENT_SOURCE_MONITOR`, `_CHECK_LINK`, `_RECOVER`, `_SET_LINK_STATE`,
  `_CELL`; 0 = not stated), so a binding hands a link event to the callback of the call that caused it, never to a
  monitor's, and a monitor's event never to a call. A status monitor's `IAD_EVENT_STATUS` / `IAD_EVENT_DONE` carry
  `IAD_EVENT_SOURCE_MONITOR`.
- `caller_thread` (0 = not stated) is the OS id of the thread that queued the event (`GetCurrentThreadId`, `gettid`,
  `pthread_threadid_np`; Python's `threading.get_native_id()`). A link event of `iad_axis_check_link`,
  `iad_axis_recover` or `iad_axis_set_link_state` is queued on the thread that made the call, so a binding can give each
  event to its own call even when calls of one source overlap.
- `iad_monitor_options.events` (`IAD_EVENTS_*` bits) chooses the kinds a handle queues, and `queue_capacity` how many it
  keeps (default 1024). Beyond that the oldest are dropped and counted in the next event's `dropped`. `sequence`
  numbers the events of a handle.
- `iad_axis_close` / `iad_cell_close` wake a waiting `next_event`, which returns `IAD_ERR_INVALID_ARGUMENT`.
- A cell handle queues its faults and recovery events from the open on, its axes' monitor events after
  `iad_cell_start_monitors`, and its link monitor's after `iad_cell_monitor_links`.

### Move handles

`iad_axis_start_move_to` / `start_move_by` / `start_jog` send the move (every check has passed) and return an
`iad_move_handle*` (C++ `MoveHandle`).

- `iad_move_handle_done`: one check.
- `iad_move_handle_poll`: wait at most a slice without stopping the move. A binding waits in slices so it stays
  interruptible; `iad_axis_cancel` ends a slice at once after a decel stop. A poll uses the monitor's samples since the
  command, as a blocking wait does, and reads the unit itself only once the monitor published nothing for 0.25 s.
- `iad_move_handle_wait`: block; a decel stop on timeout or alarm.
- `iad_move_handle_cancel`: a decel stop, also from another thread while a wait runs.
- `iad_move_handle_result` / `iad_move_handle_json`: no I/O.
- `iad_move_handle_close`: a move handle keeps its axis object alive, not its port. Closing it does not affect the move.

### Cancelling

- `iad_axis_cancel` / `iad_cell_cancel` cancel only the calls running now. They do not end a coordinated run: stop it.
- **Cancel latch** (`iad_axis_cancel_latch` / `_reset` / `_latched`, `iad_cell_cancel_latch` / `_reset`): cancels like
  `iad_axis_cancel`, and while it is set every blocking call of the axis that starts is refused with
  `IAD_ERR_CANCELLED` before anything is sent. A wait that opens meanwhile ends at its first check point (a motion wait
  after its decel stop). So a cancel that fires just before a call starts is never lost. Stop, pause and resume are never
  refused.
- Binding pattern: register the cancellation (it sets the latch), call, unregister, reset.
- The unit info and backup readers open the axis's cancel scope, so a cancel or the latch ends them at their next
  request with `IAD_ERR_CANCELLED`. They are read-only: nothing is sent for the cancel.
- The cell forms reach every listed axis the cell has and report each unknown name as an error of its own
  ([Cells and group commands](#cells-and-group-commands)).

### Axis state without I/O

What the Python `Elecylinder` attributes and methods show of an axis.

- **Raw status words**: `iad_axis_last_status_words(axis, like, ...)` gives the 0x9000 words (8, 14, 20 or 22; 32-bit
  values high word first) of one of the axis's last 16 decoded samples: the newest whose fields equal `like`, or the
  newest with `like` NULL; `IAD_ERR_NOT_READY` when none matches. A binding decodes its own status record from them and
  takes position and velocity from the `iad_status`.
- **State without I/O**: `iad_axis_state_json` (status words, pending motion, held stop, limits and their notes, unit
  codes, model, boot baud rate, the directory read so far, caps, fast mode). `iad_axis_kind_json` (units, kind, family,
  is_ec) reads the limits and FC11 once.
- **Reads of their own**, all READ-ONLY: `iad_axis_read_soft_limits_json`, `iad_axis_read_axis_limits_json`,
  `iad_axis_read_speed_limit`, `iad_axis_read_support_words_json`, `iad_axis_categories_json`,
  `iad_axis_parameter_model_json`, `iad_axis_user_parameters_json`, `iad_axis_push_speed_json`,
  `iad_axis_servo_on_timeout_json`.
- **Commands**: `iad_axis_enable_modbus_control` (PMSL at rest, PMSS read back), `iad_axis_begin_stop` (STOP on until a
  stop or a close), `iad_axis_write_position_row` (`write_checksum` 0: the 15 data words only),
  `iad_axis_release_held` (what closing does to the unit, keeping the handle and its state),
  `iad_axis_set_axis_limits_json` (the settable `axis_limits`, which later checks use).
- **Constants**: `iad_constants_json` (the `ec` module's constants, for a conformance check).
- **Position mode**: `iad_axis_move_to_position` refuses a row the EC's position mode (P9) does not run, with
  `IAD_ERR_SOFT_LIMIT`, before anything is sent. `iad_axis_write_position_table` and `iad_axis_teach_position` write it
  with a warning (`warning_count`, the texts in `iad_axis_last_result_json`). VERIFIED in the Python edition on the
  EC-GRBP8M gripper in 2-position mode.
- **Writes need no opt-in**: `iad_nv_write_options.allow_nonvolatile` and `allow_unsafe`, the
  `iad_param_write_options` opt-ins (`allow_dangerous`, `allow_home_params`, `allow_unlisted`) and
  `iad_baud_options.allow_unverified_code` stay in the structs and are accepted and ignored (no longer needed). A risky
  write goes through with a warning. `IAD_ERR_NONVOLATILE_REFUSED` is still returned for rows 256-267 and a row the
  table cannot address; `IAD_ERR_UNSAFE_REFUSED` for a read-only parameter, the never-send list, an IAI command off
  the safe list without `IAD_IAI_ALLOW_UNSAFE`, and line rules.
- **Core behaviour shared with the Python edition**: a move's configured caps and stroke are checked before anything is
  read; a stored push row is capped in PPOW counts; the status read takes 20 words when the directory announces a
  shorter monitor block; `read_monitor` gives `alarm_level`, `rated_current` and `words`; the speed note names the
  register address (`MAXV item 7.4 @0x0706`); the baud change's note is the Python `BAUD_VOLATILE_NOTE`.

### The status monitor

- `iad_axis_suspend_monitor` / `iad_axis_resume_monitor` hold one axis's polls (a poll in flight is waited for; holds
  nest and are released with the handle). `iad_bus_suspend_monitors` holds every monitor of a bus.
- `iad_axis_monitor_target` gives the last commanded target and band the done rule checks a sample against (NaN when the
  axis knows none). No I/O.
- `iad_axis_set_port_owner` marks the axis of `iad_axis_open_bus` as the one user of that bus, so
  `iad_axis_enable_fast` may change the baud rate (otherwise the note "shared port: baud rate kept"). Closing the handle
  still leaves the bus open.
- `iad_axis_monitor_info_json` includes `suspended`, `started_at_s`, and `error_detail` / `last_error_detail` (the error
  that ended the monitor and the last transient one, in the form of `iad_last_error_json`, so a binding raises the class
  a failed call gives).

### The non-volatile write guard

- `iad_axis_set_nv_unit(axis, key, label)` shares the guard's bookkeeping with every axis of the process given the same
  key: the write counter, the hour window and its warning, and the parameters written to RAM and not committed.
  `iad_nv_forget(prefix)` drops it; a new process starts that way.
- An unchanged value writes no item and commits nothing, unless a parameter written to RAM waits for a commit (a
  `commit = false` write, or a commit that failed or was not confirmed) or `force` is set.
- Above 60 writes an hour the guard warns once. It never refuses.
- The test hooks that count a write at a chosen time are in [The testing header](#the-testing-header).

### Recovery sessions, link state, event sources

Simulator tests only (UNVERIFIED on hardware).

- **Sessions**: `iad_axis_session_from_info` (no I/O; NULL info = the identity the axis read last; a unit with a session
  keeps it) and `iad_axis_session_update` (every field checked before anything is written). `uptime_age_s` is an age,
  not a time: the core's clock (`iad_clock_now`) is not the binding's.
- **Link state and detection**: `iad_axis_set_link_state` (its event is queued on the handle),
  `iad_axis_port_present` (`IAD_TRUE`, `IAD_FALSE` or `IAD_UNKNOWN`: the transport's own presence, else the serial name
  in the port list) and `iad_axis_restart_signs` (the signs of a status block against the unit's session).
- **JSON checks**: these calls refuse a wrongly typed value, a fraction where a whole number belongs (`baudrate`,
  `uptime_ms`, `system_code`, the alarm keys; 9600.0 is still taken) and a number outside its field's range, with
  `IAD_ERR_INVALID_ARGUMENT`.
- **The cell's link monitor**: `iad_cell_link_monitor_json` (`running`, `interval_s`, `passes`, `states`, `errors`; null
  without one). No I/O.
- **Policy and summary**: `iad_recovery_policy.require_session` makes `iad_axis_recover` refuse a unit without a session
  (`IAD_ERR_RECOVERY`) instead of taking the identity the axis read last. `iad_recovery_summary.control_kept` (also in
  the report's JSON): PC control was held before the fault and still is, so nothing was written.
- **Retries**: a recovery that found the unit at another rate and already tried the session's rate on the quick path
  does not try it again (one broadcast, one note). Its identify reads the unit's directory again.

### Directory readers, self-test lines, input ranges

Simulator tests only (UNVERIFIED on hardware).

- **`iad_axis_info_call_json(axis, function, args_json)`**: the readers of the unit's directory by name, READ-ONLY (FC03
  only, every read inside a block the directory declares). The result is kept for `iad_axis_last_result_json`.
  - `read_directory`, `read_block`, `read_class`, `read_support_words` and `read_accl`. All but `read_directory` take
    `records`: null for the directory the axis has read, else the index table read now (not kept), or the records given.
  - `unit_info` (`sections`: null or a list; an empty list reads none).
  - `backup` (`rows`: `"used"`, `"all"` or row numbers), never written to a file. A selection the unit cannot serve
    becomes a `positions: ValueError: ...` entry of `errors`, and the other sections are still read.
    `iad_axis_backup_json` keeps the CLI's strict `--rows`.
  - A block out of range, a class not in the directory, or a caller record whose block passes 65535 registers is
    `IAD_ERR_INVALID_ARGUMENT` (kind `std::out_of_range`), with nothing sent. An unknown function is
    `IAD_ERR_UNSUPPORTED`.
- **`iad_axis_self_test_items_json(axis, first)`**: the lines the running `iad_axis_self_test` has finished so far, from
  index `first`, with `name`, `result`, `detail` and `data`. `IAD_EVENT_CHECK_ITEM` carries `IAD_MESSAGE_SIZE` bytes of
  the detail and no data. `[]` when no test runs (the finished test's lines are its report's `items`). No I/O; called
  from another thread while the test runs.
- **Input ranges and product lines** come through [`iad_call_json`](#stateless-functions).

### Cells and group commands

- Axis lists are one UTF-8 string of names separated by commas (`"COM3:1,COM4:1"`; NULL or `""` = every axis), which
  passes easily from every language, LabVIEW included. `iad_cell_open` takes ports the same way: `"COM3,COM4"` scans
  those ports, `"COM3:1,COM3:2"` opens exactly those slaves, NULL scans every IAI USB adapter.
- Group commands (`status_all`, `servo_all`, `home_all`, `move_all`, `wait_all`, `stop_all`) handle every listed axis
  before returning (the C++ `return_exceptions` semantics): `IAD_OK` when every axis succeeded, `IAD_ERR_CELL` when one
  or more failed, with each axis's code, status and message in the `iad_cell_result` array.
- A results array that is too small never stops a command (a stop must always go out). It is filled as far as it goes,
  and `*count` says how many axes were handled. Size it with `iad_cell_axis_count`.
- An unknown name refuses the command before anything is sent, except in `stop_all` and `pause_all`: a STOP never waits
  on an unrelated bad name. Every listed axis the cell has is stopped (paused) first, then the call returns
  `IAD_ERR_CELL` with each unknown name as a result of its own (`IAD_ERR_INVALID_ARGUMENT`) after the axes.
- `iad_cell_cancel`, `iad_cell_cancel_latch` and `iad_cell_cancel_reset` follow the same rule: they reach every listed
  axis the cell has first, then return `IAD_ERR_CELL` with each unknown name as an error of its own in the `axes` of
  `iad_last_error_json` (they have no results array). With only unknown names nothing is done, never "every axis".
- `iad_cell_open_axis` opens a normal axis handle for one axis of the cell, so every single-axis function works on it.
- `move_each` (an `iad_axis_move` array, each axis with its own options), `move_all`'s per-axis values (speed, accel,
  band and decel arrays in the order of the axis list; NaN = the options' value), `pause_all` / `resume_all` (optionally
  one broadcast per daisy chain first), `cancel`, the monitors and the link monitor, `track_all`, `check_links`,
  `recover_all`, `monitor_all`, the lines, the gateways and `backup` follow the same rules.
- A group command is refused with `IAD_ERR_NOT_READY`, nothing sent, while the calling thread holds the bus lock of one
  of the cell's ports: it runs the axes on core threads, which would wait for that hold forever. This covers status /
  servo / home / move / move_each / wait / stop / pause / resume, `read_all_json`, `monitor_all_json`, `track_all`,
  `check_links`, `link_monitor_check_now`, `recover_all` and `coord_run_json`. Simulator tests only (UNVERIFIED on
  hardware).

### Coordinated motion

Several cell axes follow one path (lines, arcs, circles, polylines and the circle / rectangle / polygon shapes), run by
the PC: the C++ `coord.hpp` (`Cell::coordinated`, `CoordinatedGroup::plan` / `run`). The request is JSON, the one place
besides a backup where the ABI reads JSON, because a path has a variable shape:

```json
{"axes": {"x": "COM3:1", "y": "COM4:1"},
 "path": {"segments": [{"line": {"x": 40, "y": 10}},
                       {"arc": {"center": [40, 30], "end": {"x": 40, "y": 50}, "ccw": true}}]},
 "speed": 20, "accel_g": 0.3, "mode": "auto"}
```

- `iad_cell_coord_plan_json` is the dry run: the plan from where the axes are, checked against their ranges and the line
  timing measured now, as JSON (with a `summary` line). Reads only.
- `iad_cell_coord_run_json` plans, runs and blocks until the path is done. Its result (frames, late frames, deviation,
  samples, trace) is kept for `iad_cell_last_result_json`, so a too-small buffer never repeats a run.
- A path the planner refuses returns `IAD_ERR_SOFT_LIMIT` (`InfeasiblePathError`: `reasons`, `max_speed`); nothing is
  sent.
- An outside stop (`iad_cell_stop_all`, `iad_axis_stop` or `iad_axis_pause` from another thread, from the call on, its
  planning included), a fault alarm, a link fault or a late frame stops every axis of the group and returns
  `IAD_ERR_DRIVER` (`CoordinatedMotionError`: `axis`, `cause`, `stop`, `result`). `iad_axis_cancel` / `iad_cell_cancel` do not end a run: stop it.
- `"expect_start": {"x": 10, "y": 20}` (optional, from 1.0.1; path names): where the plan to run must start. The run
  plans first, then each named axis must be within max(band, 0.05) of the start of that plan (its lead-in's, if any),
  else `IAD_ERR_NOT_READY` and nothing is sent. A binding's `run_plan` sends the start of the plan it holds, so the
  check is inside the run's stop window: a stop sent meanwhile still ends the run.
- Unknown keys are refused, so a misspelt option never runs with its default. The header lists every key.

Evidence: tested on the simulator only (UNVERIFIED on hardware).

### Transports and buses

These calls let a binding drive everything through the core. The header gives each function's arguments.

- **Handles** `iad_transport`, `iad_bus`, `iad_sim`, `iad_sim_line` are tokens from the same counter as the other
  handles; a handle of the wrong kind is refused by name.
  - A transport is shared by whatever uses it. `iad_transport_release` lets go of the handle only;
    `iad_transport_close` closes the transport too.
  - A bus from `iad_bus_open` is owned (closing it closes its transport). One from `iad_axis_bus` / `iad_cell_bus` is
    borrowed (closing releases the handle only).
  - `iad_bus_shutdown` closes the bus as `ModbusRtu::close` does (its monitors stop first, nothing queued is sent, the
    transaction in flight finishes, then the transport closes) and keeps the handle for `iad_bus_reopen`.
    `iad_bus_release` lets go of the handle only and never closes anything, even for a bus of `iad_bus_open`: the bus
    and its transport go with their last user, and a callback transport then gets `release`, never `close`.
  - Closing a simulator or line handle leaves the unit to its users.
- **The callback transport** (`iad_transport_callbacks`, `iad_transport_open_callbacks`):
  - write and read (with a per-read timeout) are required. reset_input, set_baud_rate, send_break, present, reopen,
    close and set_read_timeout are optional. `release(user)` runs once after the last callback.
  - A callback that fails returns non-zero, optionally after `iad_callback_error(text)`. The core raises
    `TransportError` (`IAD_ERR_TRANSPORT`) with that text. `IAD_CALLBACK_ABORT` (2) is different: see
    [caller aborts](#per-thread-results-and-caller-aborts).
  - `set_read_timeout` is called when the bus changes the read timeout (around a probe or a capped read:
    `iad_bus_probe`, `max_wait`) and by `iad_transport_set_read_timeout`. A failure leaves the timeout as it was.
  - `iad_transport_callbacks_init` fills the whole struct only when `struct_size` was set to
    `sizeof(iad_transport_callbacks)` first; otherwise it fills only the first size.
  - While open, a callback transport holds the 1 ms timer resolution on Windows, as a serial port does, so the bus's gap
    and polls sleep in 1 ms steps instead of about 15.6 ms.
- **Requests** (`iad_bus_*`): FC03, FC05, FC06, FC10, FC17, FC11, IAI 0x45, a broadcast coil and a probe; baud detection
  and the PC side's rate; a line BREAK, reopen, retries and the gap; `iad_bus_set_frame_sink` (every request, response
  and partial response). Per-call options are parameters: `iad_bus_read_registers` (`max_wait_s`, `chunk`),
  `iad_bus_report_slave_id` (retries, the FC11 wait), `iad_bus_detect_baud_rate` (targets such as
  `"1,2,0xD0@0x3700,0x3F@0xF700"`, the probe timeout).
- **Objects on objects**: `iad_axis_open_bus` (no I/O; the axis does not own the bus), `iad_axis_open_transport` (as
  `iad_axis_open`, and the axis owns its bus), `iad_cell_open_transports`.
- **Another transport**: `iad_bus_set_transport` keeps the bus's settings, response delays, line, frame sink and closed
  state, and everything keyed by the bus (its axes and cells, their recovery sessions and non-volatile write counts). It
  waits for the transaction in flight and sends nothing. The old transport is neither closed nor released; the bus keeps
  it until the bus goes. Simulator tests only (UNVERIFIED on hardware).
- **Bus lock across calls** (`iad_bus_lock(bus, timeout_s, &locked)` / `iad_bus_unlock`): the calling thread holds the
  fair bus lock (FIFO, stop-class requests first) until it gives it back. No request of another thread (a monitor,
  another axis, a STOP) goes out meanwhile; the holder's own do (the lock is recursive). `timeout_s` NaN waits as long as
  it takes, 0 takes a free bus only, any finite value is honoured. A timeout is `locked = 0` with `IAD_OK`. A closed bus
  is `IAD_ERR_TRANSPORT`.
- **Preemptible reads** (`iad_bus_preemptible_begin(bus, words, gate, user)` / `iad_bus_preemptible_end`; the GUI's
  `_GatedBus`): the calling thread's FC03 reads on that bus go out in frames of at most `words` registers (0 = 32), each
  attempt taking the bus on its own after the gate. The thread's writes, IAI commands, broadcasts, baud changes and line
  breaks on that bus are refused (`IAD_ERR_UNSAFE_REFUSED`). A gate that returns non-zero ends the read with
  `IAD_ERR_CANCELLED`. Scopes nest; other buses are not affected.
- **Monitors on a bus**: `iad_bus_suspend_monitors` / `iad_bus_resume_monitors` stop the polls of every monitor of an
  axis on the bus. Kept on the handle; closing it resumes them.
- **IAI commands**: `iad_bus_iai_command` and `iad_bus_broadcast_iai_command` take `IAD_IAI_ALLOW_UNSAFE`,
  `IAD_IAI_ALLOW_NEVER_SEND` and `IAD_IAI_NO_CHECK` (another bit: `IAD_ERR_UNSUPPORTED`, nothing sent).
  `iad_bus_iai_command` takes the function code 0x45 (or 0) or 0x48 ([EXPERIMENTAL], UNVERIFIED on any unit; the native
  simulator answers it).
- **Broadcasts**: `iad_bus_broadcast_iai_command` (0x45 to address 0) and `iad_bus_broadcast` (an FC05 / FC06 / FC10
  PDU). Refused on a gateway line and inside a preemptible scope; a frame buffer too small is refused before anything is
  sent.
- **The line**: `iad_bus_set_line(bus, IAD_LINE_*, position_unit_mm)` states it (no I/O), `iad_bus_detect_line` probes
  for it (READ-ONLY) and `iad_bus_line_json` reads it. Every axis on the bus follows the new line at once.
- **Response timing**: `iad_bus_set_response_delay` / `iad_bus_response_delay` (RTIM per slave, 0..1000 ms) and
  `iad_bus_response_timeout` (the wait for a given PDU: IAI's formula and the floors).
- **`iad_bus_info_json`** includes `lock_depth`, `lock_waiting`, `preemptible_words` and `suspended_monitors`, and never
  waits for the bus lock.
- **Log level** (`iad_set_log_level`): lines below it never reach the log sink.
- **Serial framing** (1.1; `iad_serial_options`, `iad_serial_options_init`, `iad_transport_open_serial_options`): a
  serial port opened as `iad_transport_open_serial` does, with its settings in a sized struct: the rate, the read
  timeout and the character framing. The default is 8 data bits, no parity, 1 stop bit (8N1), what the Modbus units use
  and what `iad_transport_open_serial` always opens. 7 data bits, `IAD_PARITY_EVEN` / `IAD_PARITY_ODD` and 2 stop bits
  are for a controller set to another framing. The Modbus RTU stacks (buses, axes, cells) need 8 data bits (a byte above
  0x7F does not fit in 7); 7 data bits is only for a caller's own 7-bit protocol on `iad_transport_write` /
  `iad_transport_read`. In the struct, 0 data bits or stop bits mean the default, as a `baud_rate` of 0 means 115200 and
  a NaN `read_timeout_s` means 0.2 s. Another value (data bits other than 7 or 8, stop bits other than 1 or 2, a parity
  that is not an `IAD_PARITY_*`) or a `struct_size` the rule refuses is `IAD_ERR_INVALID_ARGUMENT`, a set reserved slot
  `IAD_ERR_UNSUPPORTED`: nothing is opened. A port whose driver does not keep the framing (POSIX: read back after the
  open; a Linux pseudo terminal keeps only 8 data bits and no parity) is `IAD_ERR_TRANSPORT`. A byte that fails the
  parity check is passed on as received on Windows and read as 0 on POSIX; the Modbus CRC drops the frame either way.
  `iad_transport_reopen` keeps the framing. In C++ it is `SerialTransport(port, baud_rate, read_timeout_s,
  SerialFraming)`. The bindings bind the two functions without a public call yet. Evidence: 8N1 as
  `iad_transport_open_serial`; 7 data bits, parity and 2 stop bits are UNVERIFIED on hardware (no unit here uses them;
  the tests check the Windows DCB and the termios flags each framing sets).

### Lines and gateways

The line and the gateways as handles of their own (`line.hpp`, `gateway.hpp`). Every gateway rule is DOC only (ME0413,
ME0208): UNVERIFIED on hardware.

- **Lines** (`iad_line`, released by `iad_line_close`): `iad_line_open` makes one (`IAD_LINE_DIRECT` ..
  `_ROBONET_GATEWAY`, not attached), and `iad_bus_attach_line` makes it the bus's line (its guard, gateway and position
  units then serve every axis on the bus). `iad_bus_line` gives a handle on the line a bus carries.
  - On a line: `iad_line_json`, `iad_line_allows` / `iad_line_require` by operation name, `iad_line_check_request`,
    `iad_line_require_control`, the position units (`iad_line_position_unit`, `_set_position_unit`,
    `_require_position_unit`), `iad_line_note_unreachable` / `_explain_unreachable`, `iad_line_gateway`.
    `iad_line_rules_json` gives every kind's rules.
  - A refusal is `IAD_ERR_UNSAFE_REFUSED` with kind `LineRuleError`. `GatewayModeError`, `PositionUnitError` and
    `UnsupportedLineError` are `IAD_ERR_NOT_READY` with that kind.
- **Gateways** (`iad_gateway`, read-only, released by `iad_gateway_close`): `iad_gateway_open` (an RCON gateway at D0h
  by default, or the ROBONET gateway at 3Fh, with the status already read), `iad_gateway_info_json`,
  `iad_gateway_status`, `iad_gateway_require_manu`, `iad_gateway_read_alarm`, `iad_gateway_set_cache_s`,
  `iad_gateway_read_monitor_json`. A line or gateway handle keeps the bus its gateway reads through alive.
- **On a bus**: `iad_bus_probe_gateway` (one FC03 read, no retries), `iad_bus_read_serial_code` (0x9400),
  `iad_bus_robonet_hint`.
- **Rules stated for the axes**: `iad_bus_state_line_rules(bus, json)` states what the axes on the bus's line may send
  over the kind's rules (NULL clears). `Line::allows`, the axis's strategy choice, its stop path,
  `iad_axis_line_name` and `Status::line` follow them; the bus's request guard keeps the kind's rules.

### Ports and cells over the caller's buses

A port on a bus, and cells over objects a binding already holds (`port.hpp`, `cell.hpp`). Simulator tests only
(UNVERIFIED on hardware), several axes per port included.

- **A bus as a port**: `iad_bus_set_port` makes the bus a C++ `Port` (its name, line kind, a fixed rate). The port lives
  with that bus handle and in the cells that use it; a bus never made a port gets one with the defaults at its first
  port call. Its `detect_line` callback (`iad_port_line_fn`, may be NULL) lets the binding work out the line itself.
  The port calls it once the rate is settled where it would probe, on the thread doing that work, and never sets the
  bus's line itself.
- **On the port**: `iad_bus_port_info_json`, `iad_bus_ensure_baud_rate` (once, in place), `iad_bus_port_detect_line`,
  `iad_bus_scan` (the addresses that answer one FC03 probe each; an RCON line probes the axes its gateway reports
  linked), `iad_bus_scan_units_json`, `iad_bus_identify_json` (a failure stays in `error`),
  `iad_bus_port_detect_baud_rate` (with the caller's candidates; the port then counts its rate as found, and no answer
  at any rate is `IAD_ERR_MODBUS_TIMEOUT` with the no-answer checklist) and `iad_bus_read_device_info_json` (the
  `DeviceInfo` JSON of `iad_axis_identity_json`, read without an axis and without the limits; parts the unit refuses
  stay in `errors`).
- **Baud change of the whole line**: `iad_bus_set_baud_rate_all` broadcasts it with every listed slave at rest; the
  first verifies, the others are probed. It is refused before anything is sent for a rate with no IAI code, or on
  a gateway line; a code not VERIFIED is sent with a warning.
- **Cells over buses**: `iad_cell_open_buses` scans (`NULL`, `""` or `"scan"`) or takes listed slaves (`[1, 2]` on every
  port, `{"COM3": [1]}` per port; listed slaves must answer). A scanned port where nothing answers or that fails is
  skipped and closed, and `iad_cell_port_errors_json` gives each error. When no rate answers, the error notes the line's
  explanation and the no-answer hint. `iad_cell_axis_ref` is a handle on one of the cell's axes that outlives the cell
  handle and never closes the axis.
- **Cells over axes**: `iad_cell_open_axes` gathers axes that exist already (the axes of the binding's ports, as
  Python's `Cell(ports, axes)`); nothing is scanned or sent unless `options->identify`. `iad_cell_release` lets go of the
  handle without closing anything, so it never closes the caller's ports; `iad_cell_close` closes them.
- **Group options**: `iad_cell_read_all_json(cell, what, axes)` (`"identify"`, `"limits"`, `"monitor"`), and parameters
  of the base functions: `iad_cell_status_all(monitor_max_age_s)`, `iad_cell_home_all(method, stop_others_on_error)`,
  `iad_cell_move_each(moves, wait, timeout_s, stop_others_on_error)` (a refusal names `move_each`, which the bindings
  report as `move_all`'s), `iad_cell_wait_all(stop_others_on_error)`, `iad_cell_stop_all(broadcast)`.

### Simulators

A program runs without hardware through the stable header: `iad_axis_open_sim`, `iad_cell_open_sim`,
`iad_sim_options_init`, `iad_sim_open`, `iad_sim_close`, `iad_sim_transport`, `iad_sim_port`, `iad_sim_line_open`,
`_close`, `_add`, `_remove`, `_transport`, `iad_sim_line_port` and `iad_sim_gateway_line_open`. The power, plug,
override, hook, state, log and control calls and the unit's own clock are in the testing header.

- **Options** of `iad_sim_open` (a JSON object of `SimulatedUnitOptions` and the hooks): among them `auto_servo_off_s`,
  `cold_boot`, `ec_first_son_s`, `rtc`, `function_codes`, `broadcast`, `parw_latches`, `step_s` (a stepped clock) and
  `port_name` (default `"SIM"`). A JSON null resets an optional key. `"slave": 0` is a unit with no address of its own,
  which acts on a broadcast and never answers; a simulated line takes 1..247.
- **Request hook** answers (testing header): `IAD_SIM_PASS`, `_DROP`, `_REPLY`, `IAD_SIM_EXCEPTION(code)`,
  `IAD_SIM_FAIL` (3) and `IAD_SIM_ABORT` (4). `iad_sim_handle_frame` is the unit's own answer. `IAD_SIM_FAIL` fails the
  write with `IAD_ERR_TRANSPORT` and the reply buffer's text; a hook that throws in C++ does the same. For
  `IAD_SIM_ABORT` see [caller aborts](#per-thread-results-and-caller-aborts).
- **The unit's own clock** (`iad_sim_set_clock`, testing header): the unit runs on `now(user)` from then on, offset so
  its time goes on from where it is; NULL is the library's clock again. `now` runs with the unit's lock held and must
  not call the library. A unit opened with `step_s` keeps its stepped clock.
- **Controls** (`iad_sim_control_json`, testing header): faults, alarms, the obstacle, PIO moves, coils, position rows,
  `set_state`, `set_option`, `set_inputs` (emergency stop, drive cut, enable: the servo goes off and stays off while any
  is active) and `set_parameter_word`. `iad_sim_state_json` gives the state.
- **Models**: IAI 0x48 (cmd 0x30 servo, 0x31 home, 0x34 decel stop, 0x23 alarm reset; others rejected 0x21) and the
  teach coils MOD 0x0414 / TEAC 0x0415 (PNOW into row POSR, DSS2 PWRT / MODS, alarm 0x093 before homing). A stop while
  braking late ends at the target, and a pause there rests at it, as in the Python edition.
- **Lines**: `iad_sim_line_*` puts several units on one line. `iad_sim_gateway_line_open` makes an RCON (default),
  ROBONET (SIO through mode) or ROBONET-gateway line, with the Python `SimGatewayLine` options; the other line functions
  take it, and `iad_sim_line_control_json` reads and sets the gateway station. A frame the gateway passes on reaches the
  addressed unit through its port, so the unit's log, hooks and power state apply as on a plain line.
- **Ports**: `iad_sim_port` / `iad_sim_line_port` give the unit's or line's port for a bus. It holds the 1 ms timer
  resolution while open. Closing the bus closes the port; releasing it does not.

Evidence: simulator tests only. Every gateway rule is DOC (ME0413, ME0208), UNVERIFIED on hardware.

### The testing header

`core/include/iai_actuator_driver_testing.h` is the plumbing of the bindings' own test suites: the library's clock
(`iad_clock_*`), the simulator's fault, state and log hooks (`iad_sim_*`), and the time limits, port list and
non-volatile write hooks (`iad_testing_*`). It is in the same library and under none of the stability rules. It includes
the stable header, so a test program includes only it.

| Group | Functions |
|---|---|
| Version | `iad_testing_abi_version` (cannot fail; `IAD_TESTING_ABI_VERSION`, now 1) |
| The library's clock (process-wide) | `iad_clock_use_real`, `_use_virtual`, `_use_callbacks`, `_advance`, `_now` |
| A simulator's state and logs | `iad_sim_state_json`, `iad_sim_requests_json`, `iad_sim_events_json`, `iad_sim_register_value`, `iad_sim_line_requests_json` |
| A simulator's power, plug and faults | `iad_sim_power_cycle`, `_power_off`, `_power_on`, `iad_sim_unplug`, `_replug`, `iad_sim_set_word_override`, `iad_sim_set_unreadable`, `iad_sim_set_stale_status_polls` |
| A simulator's hooks and controls | `iad_sim_set_request_hook`, `iad_sim_handle_frame`, `iad_sim_control_json`, `iad_sim_line_control_json`, `iad_sim_set_clock` (with `IAD_SIM_PASS`, `_DROP`, `_REPLY`, `_EXCEPTION`, `_FAIL`, `_ABORT` and the clock and hook callback types) |
| The driver's time limits | `iad_testing_set_timing` |
| The non-volatile write guard | `iad_testing_axis_nv_record`, `iad_testing_axis_nv_last_hour` |
| The port list | `iad_testing_set_port_list` |

- **Clock seam** (`iad_clock_use_virtual`, `iad_clock_use_callbacks`, `iad_clock_use_real`, `iad_clock_advance`,
  `iad_clock_now`): process-wide, for tests. The bus, the axis, the cancellable sleeps and the simulators' default clock
  read it. Waits on other threads keep real-time slices, so a virtual clock is for single-threaded tests.
- **Time limits**: `iad_testing_set_timing(name, seconds)` sets `home_start_s`, `parw_ready_s` or `parw_clear_s`; NaN
  restores the default.
- **Non-volatile write guard**: `iad_testing_axis_nv_record` counts one non-volatile write of the axis's unit at a given
  time (NaN: now), as a real write is counted (the total of `iad_axis_nonvolatile_writes`, the hour window and the
  warning), so a test sees the hourly warning without an hour of writes. `iad_testing_axis_nv_last_hour` reads the
  hour's count at a given time and changes nothing. Nothing is sent to the unit.
- **Port list**: `iad_testing_set_port_list(ports_json)` replaces the PC's serial-port enumeration process-wide.
  `iad_list_ports_json`, `iad_axis_port_present`, a recovery's port check and the cell's port discovery then see that
  list; NULL restores the PC's. It opens nothing. A bad key or value is `IAD_ERR_INVALID_ARGUMENT`, with the list
  unchanged.

A binding binds the testing functions on first use, after the exact check of the testing version (Python
`native.TESTING_ABI_VERSION`; C# `NativeTestingMethods`, once before the first call). With another testing version every
testing call raises `NativeLibraryError` / `NotSupportedException`, and everything else is served.

### Versioning and the library file

The version rule is in [The 1.0 contract](#the-10-contract).

- The bindings hold the least version they accept as constants: Python `native.ABI_VERSION_MAJOR` /
  `ABI_VERSION_MINOR`, C# `NativeDriver.AbiMajor` / `AbiMinor`. They stay at 1.0 while no public call of a binding
  needs a 1.1 function, so a binding accepts a 1.0 or a 1.1 library and refuses another major; it raises its minor
  when a public call starts to use a later function. The Python tests compare them with the header, the C# tests
  with the built library.
- `iad_library_version()` is the core's version (`kVersion`), the release version of the repository ("1.1.1"),
  independent of the ABI version.
- **The library file.** CMake sets the shared library's `SOVERSION` to the ABI major (`libiai_actuator_driver.so.1`)
  and its `VERSION` to the product version. The release build (`.github/release/build_native.py`) links the Linux
  library with the same SONAME, `libiai_actuator_driver.so.<ABI major>` (read from the header), so a program linked
  against it never finds a library of another ABI major.
  - The wheel and the NuGet package carry the file as `libiai_actuator_driver.so` (the bindings load it by path).
  - The SDK zip ships `lib/libiai_actuator_driver.so.<ABI major>` plus a text linker script
    `lib/libiai_actuator_driver.so` that names it, so `-liai_actuator_driver` works and the program records the SONAME
    (a symbolic link would not survive every unzip tool).
  - On Windows the DLL's version resource (`src/iai_actuator_driver.rc.in`) gives the core's version as file and product
    version, and the C ABI and testing versions in its strings (Properties, Details).

## Loading the library

### C and C++

Include `iai_actuator_driver.h` and link the import library (`iai_actuator_driver.lib` from `build.py`, or
`iai_actuator_driver_imp.lib` from an MSVC CMake build) or `-liai_actuator_driver`. Put the DLL next to the program. A
caller needs no `IAD_*` define. C++ programs can also use the C++ headers and the static library directly; the C ABI is
for every other language and for callers who want one binary that does not depend on their compiler.
`iai_actuator_driver_testing.h` is for test programs only.

```c
#include <math.h>
#include <stdio.h>
#include "iai_actuator_driver.h"

static int ok(iad_result r) {
    if (r != IAD_OK) fprintf(stderr, "%s: %s\n", iad_result_name(r), iad_last_error_message());
    return r == IAD_OK;
}

int main(void) {
    iad_axis* ax = NULL;
    iad_move_options move;
    iad_status st;
    if (!IAD_ABI_COMPATIBLE(iad_abi_version())) {         /* the loader rule: before any other call */
        fprintf(stderr, "library %s does not match this header\n", iad_library_version());
        return 1;
    }
    iad_move_options_init(&move);
    move.speed = 20.0;                                    /* mm/s */
    if (ok(iad_axis_open("COM3", NULL, NULL, &ax))        /* baud rate detected */
        && ok(iad_axis_connect(ax, 1, NULL))              /* identity, ranges, Modbus control */
        && ok(iad_axis_servo(ax, 1, NULL, NULL))         /* NULL options: wait, the core's timeout */
        && ok(iad_axis_home(ax, NULL, NULL))              /* NULL options: wait, the axis's method */
        && ok(iad_axis_move_to(ax, 50.0, &move, &st)))    /* checked against the unit's ranges, waits */
        printf("at %.2f mm\n", st.position_mm);
    iad_axis_close(ax);                                   /* NULL-safe */
    return 0;
}
```

### C# (P/Invoke and SafeHandle)

- `[DllImport("iai_actuator_driver", CallingConvention = CallingConvention.Winapi)]` (or `LibraryImport` on .NET 7+),
  and `[UnmanagedFunctionPointer(CallingConvention.Winapi)]` on every callback delegate. `Winapi` is `__stdcall` on
  32-bit Windows (the header's `IAD_CALL`) and the one convention everywhere else.
- The runtime finds the DLL next to the application or in a NuGet package's `runtimes/win-x64/native/` (`win-x86`,
  `linux-x64`, `osx-arm64` ... for the other builds).
- Handles: `AxisHandle : SafeHandle` with `ReleaseHandle() => iad_axis_close(handle) == 0`, the same for cells. The
  `SafeHandle` closes the axis once even if `Dispose` is forgotten, and keeps it alive across a running call.
- Strings in: `[MarshalAs(UnmanagedType.LPUTF8Str)] string`. Strings and JSON out: a `byte[]` buffer decoded with
  `Encoding.UTF8`, or `Marshal.PtrToStringUTF8` for the `const char*` results.
- Structs: `[StructLayout(LayoutKind.Sequential)]` with `int` for `iad_bool`, and `fixed byte name[128]` (or
  `[MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)] byte[]`) for text, decoded as UTF-8. Never `ByValTStr`: it is
  ANSI, not UTF-8. An input struct starts with `uint StructSize` (set to its size) and `uint ReservedHeader` (0).
- After every non-zero result, read `iad_last_error_json` on the same thread and throw the matching exception. Let
  nothing run on that thread in between: no `await` between the call and the read.
- Apply the loader rule before the first call (the binding does it in `NativeDriver.EnsureLoaded`).
- The repository's binding (`bindings/dotnet/src/IaiActuatorDriver/Interop/`, netstandard2.0 and net8.0) passes strings
  as NUL-terminated UTF-8 byte arrays and `size_t` as `UIntPtr`, so it behaves the same on netstandard2.0, where
  `LPUTF8Str` and `Marshal.PtrToStringUTF8` are missing. Its use:
  [bindings/dotnet/README.md](../bindings/dotnet/README.md).

### Python (ctypes)

- `ctypes.CDLL(path)` with the absolute path of the library shipped inside the package (`iai_actuator_driver/_native/`
  or a platform wheel); `ctypes.WinDLL` and `ctypes.WINFUNCTYPE` on 32-bit Windows. No compiler is needed on the user's
  PC. The DLL depends only on Windows' own DLLs, so no `os.add_dll_directory` is needed.
- Every function gets `argtypes` and `restype`: `c_int32` results, `c_char_p` for the `const char*` ones, `c_void_p`
  handles, `ctypes.Structure` mirrors with `_fields_` in header order. Strings in are encoded UTF-8.
- ctypes releases the GIL during each call, so a stop from another Python thread reaches the core while a move waits.
- A non-zero result raises the Python class of the error table, built from `iad_last_error_json`.
- Handles are wrapped in objects with `close()`, `__enter__` / `__exit__` and a `weakref.finalize` that closes them.
- The repository's binding (`bindings/python/iai_actuator_driver/native.py`) looks for the library in
  `IAI_ACTUATOR_DRIVER_NATIVE` (a file or a folder) when set, else in `iai_actuator_driver/_native/<platform>/` (e.g.
  `win-x64`), `iai_actuator_driver/_native/`, the package folder, and last `core/build/` of a source checkout. Importing
  it never loads the library; `native.available()` checks without raising.

### LabVIEW

- Call Library Function Node, library `iai_actuator_driver.dll`, calling convention **C** ("stdcall (WINAPI)" for a
  32-bit Windows build), "Run in any thread". The library is thread-safe as described above; the UI thread would block
  the front panel during a move.
- Handles: a pointer-sized integer (`uint64` on 64-bit LabVIEW) passed by value, and by pointer for the `**out`
  parameters.
- Strings in: "C String Pointer". JSON out: a `uint8` array pre-allocated to 65536 elements passed as "Array Data
  Pointer", its length as `buffer_size`, then "Byte Array To String".
- Structs map to clusters field by field (no implicit padding, so 32-bit LabVIEW's packing matches). `char name[128]`
  (`IAD_AXIS_NAME_SIZE`) is 128 `uint8` elements. An input cluster starts with `struct_size` (a `uint32` holding the
  cluster's size in bytes, which `_init` fills in from a zeroed cluster) and `reserved_header` (0). An output cluster
  needs no initialising.
- NaN is LabVIEW's NaN constant. Comma-separated axis lists and the `results` array of `iad_cell_result` clusters avoid
  arrays of string pointers.
- The Import Shared Library wizard needs `IAD_API=` as a preprocessor definition, and `IAD_CALL=` on every platform but
  32-bit Windows.
- 32-bit LabVIEW needs a 32-bit build of the library (zig target `x86-windows-gnu`). Only x64 is built today.

## Binding notes

How the repository's bindings sit on the ABI. Both bind every function.

### Python binding

- **Loading.** Importing the package loads the library and checks its ABI (`_backend.py`; `ImportError` when it is
  not usable). `native.py` holds `_SIGNATURES` (stable header) and `_TESTING_SIGNATURES` (testing header). The modules
  `modbus`, `sim`, `sim_line`, `alarms`, `con_params`, `info`, `ec`, `monitor`, `gateway`, `cell`, `coord`,
  `recovery` and `check` are bindings over it.
- **Ctrl+C.** Ctrl+C becomes the core's cancel latch. Long calls made on the main thread run on a helper thread
  (`iad-call:<caller>`), and waits run in short slices, so Ctrl+C is raised as soon as the call has ended. A Ctrl+C that
  lands in a Python callback (a transport, a frame sink, a hook, a clock) is raised as `KeyboardInterrupt` when the call
  returns, never as a `TransportError`.
- **Events.** Each axis has one event thread, `iad-events-<name>`, the only reader of its queue. It routes each event by
  `source` and `caller_thread`, so concurrent calls and monitors never take each other's events. Monitor callbacks run
  there; events of a kind are queued only while a callback of that kind is set. A per-call `on_event` (`check_link`,
  `recover`, `set_link_state`) gets the call's events on the calling thread. A cell's fault and link events reach
  `on_fault`, `on_event` and `events` on the cell's thread `iad-events-cell`.
- **Transports.** Any object with `write(bytes)` / `read(n)` is a transport (`NativeTransport.from_object`), so the
  Python test doubles drive the core. The exception a transport raises becomes the `__cause__`. One that is not an
  `OSError` makes the callback return `IAD_CALLBACK_ABORT`, and the call raises it unchanged. A
  changed `timeout` reaches the core before the next call. `SimulatedEC` and `SimLine` hand a native bus their core
  port directly (`_iad_transport()`), unless a subclass replaces a transport method or a line has `latency_s`.
- **Errors** (`native._make_error`): kind `LineRuleError` raises `LineRuleError`, and the gateway kinds raise the gateway
  module's classes (`native.NOT_READY_KINDS`). `"attempts"` above 1 becomes the note `gave up after N attempts`. A
  `TransportError` of the core's own port gets an `OSError` cause. `IaiCommandRejected.status` is an int or a tuple of
  four; a rejected position-table row raises `SoftLimitError` with the `IaiCommandRejected` as `__cause__`.
- **Reports.** `recover`, `recover_all` and `run_check` read their report with `iad_last_result_json` on the calling
  thread (`native._invoke_kept`).
- **Module notes.** `Session` is a live view of the core's session (`iad_axis_session_json`,
  `iad_axis_session_update`). `Cell.monitor_links()` is a view of the core cell's monitor. The module-level `recover`
  sets `require_session`; `Elecylinder.recover` does not, and keeps the status monitor held until its events are
  delivered. On an RCON line `Cell.status_all` first reads the gateway's all-axes
  monitor. `check.run_check` hands each finished line to `on_item` on the caller's thread while the test runs on a
  helper thread; Ctrl+C or an exception in `on_item` latches the axis's cancel (a decel stop when the motion check
  moves) and is raised once the test has ended.

### C# binding

- **Layout.** Internal to `IaiActuatorDriver`: the stable header in `Interop/NativeMethods*.cs`, the testing header in
  `Interop/NativeTestingMethods.cs`, all with `CallingConvention.Winapi`. The queued events are raised as C# events on a
  binding-owned thread.
- **On the core.** `ModbusRtu`, `SerialTransport`, `PortManager.ListPorts`, `Axis`, `Port`, `Cell`,
  `DeviceIdentity.Read` and `SimulatedUnit` run on the core. `NativeAxis` / `NativeCell` own the handles of `Axis` /
  `Cell`. `Port` and `Cell` take a `Line` option, `LineKind.Auto` by default.
- **Port and Cell.** `Port` is the core port of its `ModbusRtu`'s bus; `Scan` stays one `Probe` per address.
  `Cell.Open` opens the transports itself, then `iad_cell_open_buses` does the rest; `new Cell(ports)` is
  `iad_cell_open_axes`. A `Port` / `Cell` given an axis already made at an address uses that axis's core axis. Every
  group command is the core's, and the `CancellationToken` latches the core's cancel.
- **Simulator.** An override of a `SimulatedUnit`'s protected members that throws fails the write (`IAD_SIM_FAIL`), and
  the write throws the override's own exception.
- **Transports.** A managed transport's exception that is not an `IOException`, `UnauthorizedAccessException` or
  `InvalidOperationException` is answered with `IAD_CALLBACK_ABORT` and thrown again as it was. A cell's group command
  over a managed transport returns the exception the transport threw. Before a probe, a scan or a baud detection the
  core gets the transport's current read timeout.
- **Gaps.** `ModbusRtu.Transport` has no setter, so `iad_bus_set_transport` is bound but unused. The C# library has no
  gateway line of its own. `Port.ReadDirectory` and the public `UnitDirectory` readers stay managed parsers (the ABI has
  no bus-level directory reader).

## Building

**Windows x64 with zig (no MSVC or CMake):**

```sh
cd core
python build.py shared              # build/iai_actuator_driver.dll + build/iai_actuator_driver.lib (+ .pdb)
python build.py shared --release    # -O2
python build.py check               # also checks include/iai_actuator_driver.h as C99 (-pedantic)
python build.py manifest            # both headers and both bindings against abi/manifest.txt (--update: rewrite it)
```

- The shared build reuses the library objects of `python build.py lib` and compiles `src/c_api.cpp` once more with
  `IAD_BUILD_SHARED`, so only the `iad_` functions are exported. It links with `zig c++ -shared -target
  x86_64-windows-gnu`.
- zig links its libc++ and libunwind statically, so the DLL imports only Windows' own DLLs: KERNEL32, WINMM, SETUPAPI,
  ADVAPI32 and the Universal CRT (`api-ms-win-crt-*`, part of Windows 10 and later).
- The import library is COFF: MSVC's linker and MinGW's both take it.
- The link compiles the version resource (`src/iai_actuator_driver.rc.in`) into the DLL.

Check that it loads:

```sh
python -c "import ctypes as c, os; d = c.CDLL(os.path.abspath('build/iai_actuator_driver.dll')); d.iad_abi_version.restype = c.c_uint32; d.iad_library_version.restype = c.c_char_p; v = d.iad_abi_version(); print(v >> 16, v & 0xFFFF, d.iad_library_version().decode())"
```

**CMake** (customers, CI): `-DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON` (on by default with `-DBUILD_SHARED_LIBS=ON`) adds
the target `iai_actuator_driver_shared` (output name `iai_actuator_driver`) next to the static library, which the tests
and the CLI keep linking.

- It defines `IAD_BUILD_SHARED`, hides every other symbol, and links the C++ runtime statically: MSVC `/MT` (no Visual
  C++ redistributable; the import library is `iai_actuator_driver_imp.lib` because `iai_actuator_driver.lib` is the
  static library), MinGW `-static`, Linux `-static-libstdc++ -static-libgcc` with `--exclude-libs,ALL`.
- `cmake --install` installs both headers, the library and the import library.
- The shared library's `SOVERSION` is the ABI major. On Windows the build compiles the version resource when an RC
  compiler is found.
- The Linux job of `.github/workflows/ci.yml` builds it with CMake and gcc and runs `ctest`; the soname was checked on
  Linux. The Windows resource path and the MSVC and MinGW branches are built nowhere yet: UNVERIFIED.

## Distribution per OS

| OS | File | Notes |
|---|---|---|
| Windows x64 | `iai_actuator_driver.dll` (+ `.lib` for C / C++ linking) | No runtime to install on Windows 10 / 11. Windows 7 / 8.1 need the Universal CRT update (KB2999226). Ship the DLL next to the program, in the Python package, or in the NuGet package under `runtimes/win-x64/native/`. Sign the DLL for customer distribution (SmartScreen, endpoint protection). The `.pdb` stays internal. |
| Windows x86 | `iai_actuator_driver.dll` (32-bit) | For 32-bit LabVIEW and 32-bit applications. Not built yet (zig target `x86-windows-gnu`). |
| Linux x64 / arm64 | `libiai_actuator_driver.so` | `serial_posix.cpp` is VERIFIED on a real port from 1.1.0 (the IAI USB adapter, `cp210x`, in WSL 2; the 1.0.1 code fails at the first read). Build against an old glibc for reach (zig target `x86_64-linux-gnu.2.28`, or a manylinux image with CMake). The user needs access to the port (`dialout` group). |
| macOS arm64 / x64 | `libiai_actuator_driver.dylib` | UNVERIFIED (no Mac in the test setup). A universal2 build, code-signed and notarized for distribution. |

There is one library per OS and CPU. The bindings pick it by platform (`RuntimeInformation` in C#, `sys.platform` and
`platform.machine()` in Python). Each release ships the header, the library, the import library and the bindings built
from the same commit, so their ABI versions always match.

## Not in the ABI yet

- Of coordinated motion: a group kept across calls, running a plan made earlier (a run plans again), the start where the
  axes already are on a circle (`CoordinatedGroup::circle`), a group of standalone axis handles (a cell only),
  `iad_cell_cancel` ending a run.
- Port scanning without a cell, `change_speed` on a cell, `identify_all` / `limits_all` on a cell (each axis handle has
  them).
- Of what the bindings asked for: `row_words` / `monitor_record_words` / `missing_items` / `model_string`, and
  `iad_shutdown`.
- What the C++ core does not do yet is listed in [core/README.md](../core/README.md) ("Where this edition is shaped
  differently from the Python one").
