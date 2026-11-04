# Getting started: C (and other languages with a C FFI)

`core/include/iai_actuator_driver.h` is the C ABI of the core: plain C99, every name prefixed `iad_`. One shared
library exports it (`iai_actuator_driver.dll`, `libiai_actuator_driver.so`, `libiai_actuator_driver.dylib`). It is the
only header a C program, LabVIEW or any language with a C FFI needs. Unofficial: not an IAI product
([README](../../README.md)).

The header documents each function. The model (handles, errors, threads, JSON, versioning, LabVIEW) is in
[../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md).

## The library

| Source | Files |
|---|---|
| A release (the C/C++ SDK zip) | the header, the library and (Windows) its import library, all of the same release |
| Windows, zig (`python build.py shared` in `core/`) | `core/build/iai_actuator_driver.dll` and the import library `core/build/iai_actuator_driver.lib` (COFF: MSVC's and MinGW's linkers take it) |
| CMake with `-DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON` | target `iai_actuator_driver_shared`: `libiai_actuator_driver.so` on Linux; on MSVC `iai_actuator_driver.dll` with the import library `iai_actuator_driver_imp.lib` |

The build commands: [core/README.md](../../core/README.md#build).

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

Profiles: `"slider"` (also NULL), `"gripper"`, `"rod"`, `"pcon"`, `"rotary"`, `"unknown"`
([what each simulates](python.md#a-simulated-axis)).

On hardware, open the port instead: `iad_axis_open("COM3", NULL, NULL, &ax)` (`"/dev/ttyUSB0"` on Linux). The baud
rate is found in place. Close IA-OS first: only one program can use a serial port.

Several axes: `iad_cell_open("COM3,COM4", NULL, NULL, &cell)` or
`iad_cell_open_sim("slider,gripper", 1, &sim, NULL, NULL, &cell)`, then `iad_cell_servo_all`, `iad_cell_home_all`,
`iad_cell_move_all`, `iad_cell_stop_all`. A cell takes every axis it finds (no total limit; `max_axes` in
`iad_cell_options` caps it if given, 0 = no limit). One cable carries at most 16 axes
([Several axes on one port](../HARDWARE_SETUP.md#several-axes-on-one-port)).

- Positions are in mm (deg on a rotary axis), speeds in units/s, accelerations in G, times in seconds.
- `NAN` in a `double` input means "not set" (the default the header names). In an output it means "not known".
- The axis commands take an options struct as the moves do (`iad_servo_options`, `iad_home_options`,
  `iad_stop_options`, `iad_wait_idle_options`, `iad_change_options`, `iad_cancel_paused_move_options`). Fill it with
  its `_init` function and change what you need, or pass NULL for the defaults.
- A value outside the unit's own ranges returns `IAD_ERR_SOFT_LIMIT` and nothing moves. `iad_axis_check_move` runs
  the same checks without moving.
- Caps for a first test on hardware: `iad_axis_options_init(&ao); ao.max_speed_mm_s = 100; ao.max_accel_g = 0.3;`
  and pass `&ao` to `iad_axis_open`. A cap can only narrow the unit's range.
- `move.wait = 0` returns right after the command and leaves the status output unwritten. Then call
  `iad_axis_wait_idle` (its options: a target and band to check, the timeout, the completion rule).

Before a move on hardware read [Safety](../GETTING_STARTED.md#safety).

## Stop

`iad_axis_stop(ax, NULL, &st)` is a decelerate-stop over Modbus ([how it works](python.md#stop)). With
`iad_stop_options` `wait` = 1 it returns `IAD_ERR_MOTION_TIMEOUT` if the axis still moves after `timeout_s` (NaN = 5 s).

`iad_axis_stop`, `iad_axis_pause` and `iad_cell_stop_all` may be called from another thread while a thread waits in a
move, home or `iad_axis_wait_idle` on the same handle. That is how a stop button, a watchdog thread or a Ctrl+C
handler stops a blocking move. The waiting call then returns `IAD_ERR_MOTION_INCOMPLETE` (settled away from the
target). `iad_axis_cancel` (`iad_cell_cancel` for a cell) ends the waiting call instead: it sends a decel stop and
returns `IAD_ERR_CANCELLED`.

- `iad_axis_pause`, `iad_axis_resume` and `iad_axis_cancel_paused_move` hold, continue and drop a move.
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

**Non-volatile writes.** `iad_axis_write_position_table`, `iad_axis_teach_position` and `iad_axis_write_parameter` go
straight through. A risky write gives a warning (`warning_count`), an unchanged value or row is not written again
(`skipped` in the result) and `iad_axis_nonvolatile_writes` counts the writes. For a target that changes every cycle
use `iad_axis_move_to`. The rules: [Non-volatile writes](../TROUBLESHOOTING.md#non-volatile-writes).

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

The header is at C ABI 1.1; 1.0 is frozen within major 1, and a minor only adds. Check the library before the first
other call (the example above does): the same major and a library minor at least the header's.

```c
if (!IAD_ABI_COMPATIBLE(iad_abi_version())) { /* the library is too old or of another major: do not call it */ }
```

Every input struct starts with its own size. An options struct: set `struct_size = sizeof`, then call its `_init`
function. A struct without one (`iad_axis_move`, `iad_sequence_point`): zero it and set its size; a zero in it is the
default. The full rules (garbage sizes, unknown fields, output structs, JSON fields, `[EXPERIMENTAL]` parts, the
testing header): [../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md#the-10-contract).

## LabVIEW and other FFIs

Call Library Function Node on `iai_actuator_driver.dll`, calling convention C, "Run in any thread"; structs map to
clusters field by field. Strings, JSON buffers, the Import Shared Library wizard and tribools:
[../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md#labview).

## Evidence

| Part | Evidence |
|---|---|
| The C ABI through the Python and .NET bindings on the PCON-CB with an RCP6 and the EC-GRBP8M gripper | VERIFIED for the steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit); the calls none of these runs used are tested on the simulator only |
| A C program on the simulator (`python build.py smoke`, CMake test `iad_c_smoke`) | tested in CI on Windows (zig) and Linux (gcc) |
| A C program on hardware | VERIFIED: the SDK zip's `examples/c/first_move.c` on the PCON-CB with an RCP6 (servo on, a move to the middle, +10 and -10 mm, status, servo off), on Windows and on Linux (2026-11-03) |
| LabVIEW on hardware | UNVERIFIED |
| Linux serial port | VERIFIED ([../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit)) |
| macOS | UNVERIFIED |

## Next

- `core/include/iai_actuator_driver.h`: every type, code and function with its comment.
- [../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md): the model behind the ABI and how each language loads the library.
- [cpp.md](cpp.md): the core itself, for C++ programs.
