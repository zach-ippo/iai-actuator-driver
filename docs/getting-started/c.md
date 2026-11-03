# Getting started: C (and other languages with a C FFI)

`core/include/iai_actuator_driver.h` is the C ABI of the core: plain C99, every name prefixed `iad_`. One shared
library exports it (`iai_actuator_driver.dll`, `libiai_actuator_driver.so`, `libiai_actuator_driver.dylib`). It is the
only header a C program, LabVIEW or any language with a C FFI needs. Unofficial: not an IAI product
([README](../../README.md)).

The header documents each function. The model (handles, errors, threads, JSON, versioning, LabVIEW) is in
[../NATIVE_ABI.md](../NATIVE_ABI.md).

## The library

| Source | Files |
|---|---|
| A release (the C/C++ SDK zip) | the header, the library and (Windows) its import library, all from the same commit |
| Windows, zig (`pip install ziglang`, then `python build.py shared` in `core/`) | `core/build/iai_actuator_driver.dll` and the import library `core/build/iai_actuator_driver.lib` (COFF: MSVC's and MinGW's linkers take it) |
| CMake (`cmake -S core -B core/build -DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON`, `cmake --build core/build`) | target `iai_actuator_driver_shared`: `libiai_actuator_driver.so` on Linux; on MSVC `iai_actuator_driver.dll` with the import library `iai_actuator_driver_imp.lib` |

- The zig build links libc++ statically, so the DLL imports only Windows' own DLLs. Nothing to install on Windows
  10 / 11; Windows 7 / 8.1 need the Universal CRT update KB2999226.
- Only x64 is built. 32-bit LabVIEW needs a 32-bit build (zig target `x86-windows-gnu`).
- Ship the DLL next to the program. A caller needs no `IAD_*` define.

## Compile

```shell
# Windows, zig (as the core's own C smoke test is built)
python -m ziglang cc -target x86_64-windows-gnu -std=c99 -Icore/include first_move.c core/build/iai_actuator_driver.lib -o first_move.exe

# Linux, gcc, against the CMake build
cc -std=c99 -Icore/include first_move.c -Lcore/build -liai_actuator_driver -lm -o first_move
```

Run it with the library next to the program (Linux: or on `LD_LIBRARY_PATH`).
[examples/c/first_move.c](../../examples/c/first_move.c) is a complete program with `--sim` / `--port COM3` and a
Ctrl+C handler that sends a decel stop.

## A simulated axis and the first move

`iad_axis_open_sim` opens an axis on the core's simulator: no hardware, no serial port. Every other function works on
it as on a real unit.

```c
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "iai_actuator_driver.h"

/* On a failure: the result's name and the message of this thread's last error (read right after the call). */
static int ok(iad_result r, const char* what) {
    if (r != IAD_OK) fprintf(stderr, "%s: %s: %s\n", what, iad_result_name(r), iad_last_error_message());
    return r == IAD_OK;
}

int main(void) {
    iad_sim_options sim;
    iad_move_options move;
    iad_identity id;
    iad_status st;
    iad_axis* ax = NULL;
    int rc = 1;

    /* The loader rule: the same ABI major and at least this header's minor. */
    if (!IAD_ABI_COMPATIBLE(iad_abi_version())) {
        fprintf(stderr, "library %s does not match this header\n", iad_library_version());
        return 1;
    }

    sim.struct_size = sizeof sim;                /* the size first, then the defaults: a zeroed struct is not them */
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;                       /* simulated seconds per real second */
    move.struct_size = sizeof move;
    iad_move_options_init(&move);
    move.speed = 20.0;                           /* mm/s */

    if (ok(iad_axis_open_sim("gripper", &sim, NULL, NULL, &ax), "open")      /* NULL options: the defaults */
        && ok(iad_axis_connect(ax, 1, &id), "connect")                        /* identity, ranges, Modbus control */
        && ok(iad_axis_servo(ax, 1, NULL, NULL), "servo")                     /* on; NULL: wait, default timeout */
        && ok(iad_axis_home(ax, NULL, NULL), "home")                          /* moves to the mechanical end */
        && ok(iad_axis_move_to(ax, 5.0, &move, &st), "move")) {               /* checked first; waits in position */
        printf("%s at %.2f %s\n", id.model, st.position_mm, id.units);
        rc = 0;
    }
    iad_axis_close(ax);                          /* NULL-safe; releases a STOP / pause, closes the port */
    return rc;
}
```

| Profile | Simulates |
|---|---|
| `"slider"` (NULL) | the EC-DS4H slider |
| `"gripper"` | the EC-GRBP8M gripper |
| `"rod"` | the EC-R6S rod |
| `"pcon"` | the PCON-CB with an RCP6 |
| `"rotary"`, `"unknown"` | made-up units for the generic paths |

On hardware, open the port instead: `iad_axis_open("COM3", NULL, NULL, &ax)` (`"/dev/ttyUSB0"` on Linux). The baud
rate is found in place. Close IA-OS first: only one program can use a serial port.

Several axes: `iad_cell_open("COM3,COM4", NULL, NULL, &cell)` or
`iad_cell_open_sim("slider,gripper", 1, &sim, NULL, NULL, &cell)`, then `iad_cell_servo_all`, `iad_cell_home_all`,
`iad_cell_move_all`, `iad_cell_stop_all`. A cell takes every axis it finds (no total limit; `max_axes` in
`iad_cell_options` caps it if given, 0 = no limit). One cable carries at most 16 axes (axis numbers 0..15, addresses
1..16), and the scan asks addresses 1..16 per port by default. One axis per USB adapter runs fastest: axes on one cable
take turns.

- Positions are in mm (deg on a rotary axis), speeds in units/s, accelerations in G, times in seconds.
- `NAN` in a `double` input means "not set" (the default the header names). In an output it means "not known".
- The axis commands take an options struct as the moves do (`iad_servo_options`, `iad_home_options`,
  `iad_stop_options`, `iad_wait_idle_options`, `iad_change_options`, `iad_cancel_paused_move_options`). Fill it with
  its `_init` function and change what you need, or pass NULL for the defaults.
- Every target, speed, acceleration and band is checked against the unit's own ranges first. A value outside them
  returns `IAD_ERR_SOFT_LIMIT` and nothing moves. `iad_axis_check_move` runs the same checks without moving.
- Caps for a first test on hardware: `iad_axis_options_init(&ao); ao.max_speed_mm_s = 100; ao.max_accel_g = 0.3;`
  and pass `&ao` to `iad_axis_open`. A cap can only narrow the unit's range.
- `move.wait = 0` returns right after the command and leaves the status output unwritten. Then call
  `iad_axis_wait_idle` (its options: a target and band to check, the timeout, the completion rule).

On hardware: unloaded, mid-stroke, 20 mm/s or less, a hand on the emergency stop. A vertical axis without a brake can
fall when the servo goes off.

## Stop

`iad_axis_stop(ax, NULL, &st)` is a decelerate-stop over Modbus. It holds the STOP coil until the axis stands still,
then releases it. It works with or without Modbus control and blocks for the deceleration. With `iad_stop_options`
`wait` = 1 it returns `IAD_ERR_MOTION_TIMEOUT` if the axis still moves after `timeout_s` (NaN = 5 s).

`iad_axis_stop`, `iad_axis_pause` and `iad_cell_stop_all` may be called from another thread while a thread waits in a
move, home or `iad_axis_wait_idle` on the same handle. That is how a stop button, a watchdog thread or a Ctrl+C
handler stops a blocking move. The waiting call then returns `IAD_ERR_MOTION_INCOMPLETE` (settled away from the
target). `iad_axis_cancel` (`iad_cell_cancel` for a cell) ends the waiting call instead: it sends a decel stop and
returns `IAD_ERR_CANCELLED`.

- `iad_axis_pause` holds the move and keeps the remaining travel. `iad_axis_resume` continues it.
  `iad_axis_cancel_paused_move` drops it.
- STOP is VERIFIED on moves the PC commanded and UNVERIFIED on a move a PLC commanded over PIO. Stop those from the
  PLC or with the hardware stop.
- Not a safety function. There is no communication watchdog: if the PC or the cable fails, a move in progress is
  completed.

## Errors

Every function returns an `iad_result`: `IAD_OK` (0) or a negative error kind. Four cannot fail:
`iad_abi_version`, `iad_library_version`, `iad_last_error_message` and `iad_result_name`. No C++ exception crosses
the boundary.

| Code | Meaning |
|---|---|
| `IAD_ERR_SOFT_LIMIT` | a value outside the unit's range or a cap; nothing sent |
| `IAD_ERR_NOT_READY`, `IAD_ERR_SERVO_BLOCKED` | servo off, not homed, moving, no Modbus control / EMG input, drive power cut, enable switch |
| `IAD_ERR_ALARM` | the unit reports an alarm (`alarm_code` in the error JSON) |
| `IAD_ERR_MOTION_TIMEOUT`, `IAD_ERR_MOTION_INCOMPLETE` | not done in time (a decel stop was sent) / settled away from the target |
| `IAD_ERR_NONVOLATILE_REFUSED`, `IAD_ERR_UNSAFE_REFUSED` | a row that cannot be written (rows 256-267), a read-only parameter, a never-send IAI command; nothing written |
| `IAD_ERR_MODBUS_TIMEOUT`, `IAD_ERR_FRAME`, `IAD_ERR_SLAVE_EXCEPTION`, `IAD_ERR_TRANSPORT` | the link: no answer, bad frame, exception reply, port busy or gone |
| `IAD_ERR_CELL` | one or more axes of a group command failed (per axis in the `iad_cell_result` array) |
| `IAD_ERR_INVALID_ARGUMENT` | a NULL pointer, a closed handle, an unknown name or key; nothing sent |
| `IAD_ERR_BUFFER_TOO_SMALL` | a JSON output did not fit; `*required` holds the size |

The full list is in the header. A failing call sets the calling thread's last error; a call that succeeds clears it.
Read it on the same thread, right after the call:

- `iad_last_error_message()`: the message, with the core's notes such as "decel stop sent after this error". Valid
  until the next call on this thread; copy it to keep it.
- `iad_last_error_json(buf, size, &required)`: `code`, `name`, `kind`, `message`, `notes`, `stop_attempted` and,
  when the error has them, `alarm_code`, `reasons` (servo blocked), `axes` (a group command) and more.

Outputs of a failed call are unspecified unless the header says otherwise.

Rich data (unit info, limits with their sources, alarm history, parameters, the port list) comes as UTF-8 JSON into a
caller buffer. Ask for the size first, or start with 64 KiB:

```c
size_t need = 0;
char* text;
iad_axis_limits_json(ax, NULL, 0, &need);         /* IAD_ERR_BUFFER_TOO_SMALL; need = bytes with the NUL */
text = malloc(need);
if (text != NULL && iad_axis_limits_json(ax, text, need, &need) == IAD_OK) puts(text);
free(text);
```

**Non-volatile writes.** `iad_axis_write_position_table`, `iad_axis_teach_position` and `iad_axis_write_parameter` need
no opt-in: the `allow_*` fields of `iad_nv_write_options` and `iad_param_write_options` are accepted and ignored (no
longer needed), and a risky write goes through with a warning (`warning_count`). They write the unit's non-volatile
memory, which lasts about 100,000 write cycles (IAI). An unchanged value or row is not written again (`skipped` in the
result), and `iad_axis_nonvolatile_writes` counts the writes. For a target that changes every cycle use
`iad_axis_move_to`: direct moves are RAM only (VERIFIED).

## Handles and threads

- `iad_axis*` and `iad_cell*` are opaque tokens, never addresses. Handle values are never reused.
- Closing `NULL` or an already closed handle returns `IAD_OK`. Any other call with a closed handle returns
  `IAD_ERR_INVALID_ARGUMENT`.
- `iad_cell_open_axis` opens an axis handle for one axis of a cell. Closing it leaves the axis in the cell.
- Calls on different handles may run in parallel. Axes that share a port take turns on its bus mutex.
- One thread at a time commands a handle: the bus serialises the frames, but two commands on one handle from two
  threads are not made into one sequence. `iad_axis_stop`, `iad_axis_pause`, `iad_cell_stop_all`, `iad_axis_cancel`
  and `iad_cell_cancel` are meant to come from another thread. A STOP goes before every request waiting for the bus.
- The last error is per thread.
- The library calls into your code only through the optional log and frame sinks (`iad_set_log_sink`,
  `iad_axis_set_frame_sink`). They run on the thread that logs, must be short and must not call into the library.
- Monitor changes, link and recovery events and a cell's faults are queued per handle: poll `iad_axis_next_event` /
  `iad_cell_next_event`.
- Close every handle before the program exits or the library is unloaded. Closing stops and joins that handle's core
  threads (status monitors, link watches, a cell's workers); an unclosed handle keeps its port open until the process
  exits. Cancel a blocked call first (`iad_axis_cancel`), then close.

## Versions, input structs and stability

The C ABI is 1.1 (1.0 is frozen within major 1; 1.1 only adds). Check the library before the first other call (the example above does):

```c
if (!IAD_ABI_COMPATIBLE(iad_abi_version())) { /* the library is too old or of another major: do not call it */ }
```

- The rule: the same major and a library minor at least the header's (the header is at C ABI 1.1; 1.0 is frozen
  within major 1).
- Within a major version nothing you call is removed, renamed or changed in meaning. A minor only adds. Every non-zero
  result is a failure.
- The shared library's `SOVERSION` is the ABI major. The Windows DLL carries a version resource with the core's
  version and both ABI versions.

**Input structs.** Every input struct starts with its own size, so a later minor can add fields at the end.

- An options struct: set `struct_size = sizeof`, then call its `_init` function (defaults and size). A zeroed struct
  is read as the first size. A `struct_size` that was never set is your error; an input holding a garbage one is
  refused with `IAD_ERR_INVALID_ARGUMENT`.
- A struct without an `_init` function (`iad_axis_move`, `iad_sequence_point`): zero it and set its size. A zero
  speed, acceleration, band or timeout in it is the default.

```c
iad_axis_move moves[2];                  /* <string.h> for memset and strcpy */
iad_cell_result results[2];
size_t count = 0;
memset(moves, 0, sizeof moves);
moves[0].struct_size = moves[1].struct_size = (uint32_t)sizeof moves[0];   /* every element: the same size */
moves[0].position = 10.0;  strcpy(moves[0].axis, "COM3:1");               /* the speed and the rest: defaults */
moves[1].position = 20.0;  strcpy(moves[1].axis, "COM4:1");
iad_cell_move_each(cell, moves, 2, 1, NAN, 1, results, 2, &count);        /* wait, timeout NaN, stop others on error */
```

- A library that does not know a field you filled (a newer header than the library) refuses the call with
  `IAD_ERR_UNSUPPORTED` instead of ignoring it.
- Output structs have a fixed size for the whole major version. Ignore their `reserved` fields.
- The fields of every JSON output are only ever added.
- Parts marked `[EXPERIMENTAL]` in the header (`IAD_METHOD_IAI2`, `IAD_TEACH_COIL`, the 0x48 function code,
  `iad_bus_broadcast_iai_command`, `blend_band`) stay declared for the whole major version, but a minor may change
  their behaviour or answer `IAD_ERR_UNSUPPORTED`.
- `iai_actuator_driver_testing.h` holds the test hooks of the same library (a virtual clock, simulator faults). It has
  no stability promise and a version of its own, and is never needed in production.

## LabVIEW and other FFIs

- Call Library Function Node on `iai_actuator_driver.dll`, calling convention C, "Run in any thread". (A 32-bit
  Windows build, which is not made, would use `__stdcall`.)
- Structs have no implicit padding, so they map to clusters field by field, also on 32-bit LabVIEW.
- Strings in are "C String Pointer". JSON out is a `uint8` array of 65536 elements and its length.
- The Import Shared Library wizard needs `IAD_API=` and `IAD_CALL=` as preprocessor definitions (not on 32-bit
  Windows).
- A tribool input is 0 false, 2 unknown, any other value true.
- A zeroed `iad_axis_move` or `iad_sequence_point` with its `struct_size` set is all defaults. An options struct is
  filled by its `_init` function first.

Details in [../NATIVE_ABI.md](../NATIVE_ABI.md#labview).

## Evidence

| Part | Evidence |
|---|---|
| The C ABI through the Python and .NET bindings on the PCON-CB with an RCP6 and the EC-GRBP8M gripper | VERIFIED for the steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit); the calls none of these runs used are tested on the simulator only |
| A C program on the simulator (`python build.py smoke`, CMake test `iad_c_smoke`) | tested in CI on Windows (zig) and Linux (gcc) |
| A C program on hardware | VERIFIED: the SDK zip's `examples/c/first_move.c` on the PCON-CB with an RCP6 (servo on, a move to the middle, +10 and -10 mm, status, servo off), on Windows with the 1.0.1 SDK and on Linux with a build of the 1.1.0 fix (2026-11-03) |
| LabVIEW on hardware | UNVERIFIED |
| Linux serial port | VERIFIED from 1.1.0 (the 1.0.1 SDK's Linux library fails at the first read): the C example above, through the IAI USB adapter (`cp210x`, `/dev/ttyUSB0`) in WSL 2 with usbipd, a build of the 1.1.0 fix (2026-11-03) |
| macOS | UNVERIFIED |

## Next

- `core/include/iai_actuator_driver.h`: every type, code and function with its comment.
- [../NATIVE_ABI.md](../NATIVE_ABI.md): the model behind the ABI and how each language loads the library.
- [cpp.md](cpp.md): the core itself, for C++ programs.
