# Examples

Short, complete programs, one task each, in Python, C, C++ and C#. Each runs on the built-in simulator with `--sim`
(no hardware, no serial port) or on a unit with `--port COM3` (`/dev/ttyUSB0` on Linux). Run every command from the
repository folder.

Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop and guarding.

| Folder | Language | Uses |
|---|---|---|
| [python/](python/) | Python 3 | the `iai_actuator_driver` package (`Axis`, `Cell`) |
| [c/](c/) | C99 | the C ABI: `core/include/iai_actuator_driver.h` and the shared library |
| [cpp/](cpp/) | C++17 | the C++ API: `core/include/iai_actuator_driver/iai_actuator_driver.hpp` and the static library |
| [csharp/](csharp/) | C# (.NET 8) | the C# library `IaiActuatorDriver` (`Axis`, `Cell`) |

The same task has the same name and options in every language: `first_move` in Python, C and C++ and `FirstMove` in
C# do the same steps in the same order. Each file starts with a comment that says what it shows, how to run it, what
moves and the state it leaves the axis in.

## Python

Install the package first: `pip install ./bindings/python` (the examples also run straight from a source checkout).
`--help` lists every option. Each example runs with `--sim` or with `--port COM3`. `multi_axis_cell.py` and
`machine_sequence.py` take `--port` once per axis.

| Example | What it shows |
|---|---|
| [first_move.py](python/first_move.py) | Connect, servo ON, home when needed, an absolute and two relative moves around mid-stroke, the status, servo OFF |
| [stored_positions.py](python/stored_positions.py) | Read the position table, run rows with `move_to_position` slowed by the speed override, a push row's result |
| [push_press.py](python/push_press.py) | A push with a set force and band, its result (contact or missed) and the retract |
| [parameters_and_backup.py](python/parameters_and_backup.py) | A read-only backup to JSON, the parameter list, and the non-volatile guard: an unchanged value is skipped, `--force` commits it |
| [multi_axis_cell.py](python/multi_axis_cell.py) | Several axes as one `Cell`: servo, home, move together, stop, and the fault policy when one axis's link fails |
| [fast_cycle.py](python/fast_cycle.py) | A repeated A -> B -> A cycle, timed with the default completion rule and then with `enable_fast()` |
| [recovery_after_power_loss.py](python/recovery_after_power_loss.py) | `track()`, `check_link()` and `recover()`, which never moves the axis; on a unit, switch its 24 V off and on |
| [machine_sequence.py](python/machine_sequence.py) | A machine cycle written like a PLC program: steps on a fixed scan, several axes between A and B, step timers, an alarm that stops every axis |
| [draw_circle.py](python/draw_circle.py) | A circle with two axes (coordinated motion, UNVERIFIED on hardware): `--sim`, or `--x COM3:1 --y COM4:1`, with `--dry-run` to plan only; see [docs/DRIVER.md](../docs/DRIVER.md#coordinated-motion-alpha) |

```shell
python examples/python/first_move.py --sim
python examples/python/first_move.py --port COM3
python examples/python/multi_axis_cell.py --port COM3 --port COM4
```

The package runs on the native core and needs its library; `iai-actuator-driver --version` names it
([where the library comes from](../docs/getting-started/python.md#where-the-native-library-comes-from)).

## C

| Example | What it shows |
|---|---|
| [first_move.c](c/first_move.c) | The first move through the C ABI: options from `iad_*_options_init`, open (port or simulator), connect, servo, home, `iad_axis_move_to` / `iad_axis_move_by`, the status, close. Every call is checked; errors print with `iad_result_name` and `iad_last_error_message`, alarm details come from `iad_last_error_json`. Ctrl+C sends `iad_axis_stop` from its own thread. |

Run it after [building](#building-the-c-c-and-c-examples): `first_move_c --sim` or `first_move_c --port COM3`.

## C++

| Example | What it shows |
|---|---|
| [first_move.cpp](cpp/first_move.cpp) | The first move through the C++ API: `Axis::open` on a port or a `SimulatedUnit`, `connect`, `servo`, `home`, `move_to`, `move_by`, `status`. Errors as exceptions (`EcAlarmError` with the alarm's name and how to clear it, `ServoBlockedError` with its reasons). Ctrl+C sends `Axis::stop` from a watcher thread. |
| [multi_axis_cell.cpp](cpp/multi_axis_cell.cpp) | Several axes as one `Cell`: `limits_all`, `status_all`, `servo_all`, `home_all`, `move_all` with one speed, `move_each` with a speed per axis, `stop_all` during a move, `CellError` with every axis's outcome, `on_fault` |

Run them after [building](#building-the-c-c-and-c-examples): `first_move_cpp --sim`, `first_move_cpp --port COM3`,
`multi_axis_cell_cpp --sim`, `multi_axis_cell_cpp --port COM3 --port COM4`.

## C#

| Example | What it shows |
|---|---|
| [FirstMove](csharp/FirstMove/Program.cs) | The first move with `Axis`: `Open` (a port or a `SimulatedUnit`), `Connect`, `Servo`, `Home`, `MoveTo`, `MoveBy`, `Status`, and the exception classes. Ctrl+C (`Console.CancelKeyPress`) sends `Stop` from a worker thread. |
| [MultiAxisCell](csharp/MultiAxisCell/Program.cs) | Several axes as one `Cell`: `StatusAll`, `ServoAll`, `HomeAll`, `MoveAll` (waiting, and with `wait: false` plus `WaitAll`), `StopAll` during a move, `CellException` with every failed axis's error. Ctrl+C stops every axis. |

```shell
dotnet run --project examples/csharp/FirstMove -- --sim
dotnet run --project examples/csharp/FirstMove -- --port COM3
dotnet run --project examples/csharp/MultiAxisCell -- --port COM3 --port COM4
```

## Building the C, C++ and C# examples

All three need the native core.

**The native library.** On Windows without CMake or MSVC, with zig (`pip install ziglang`):

```shell
cd core
python build.py shared      # core/build/iai_actuator_driver.dll and its import library iai_actuator_driver.lib
python build.py lib         # core/build/libiai_actuator_driver.a, the static library for C++
```

On Linux and macOS (and with MSVC), build the shared library with CMake:
`cmake -S core -B core/build -DCMAKE_BUILD_TYPE=Release -DIAI_ACTUATOR_DRIVER_BUILD_SHARED=ON`, then
`cmake --build core/build`.

**C and C++ with CMake** (any platform; builds the core as part of the examples):

```shell
cmake -S examples -B build/examples -DCMAKE_BUILD_TYPE=Release
cmake --build build/examples --config Release
```

The programs (`first_move_c`, `first_move_cpp`, `multi_axis_cell_cpp`) and the shared library land in
`build/examples/bin/` (`bin/Release/` with a multi-config generator such as Visual Studio). Run them from there, for
example `build/examples/bin/first_move_c --sim`.

**C and C++ with zig directly** (Windows; run the programs next to `iai_actuator_driver.dll`):

```shell
python -m ziglang cc -std=c99 -I core/include examples/c/first_move.c core/build/iai_actuator_driver.lib -o first_move_c.exe
python -m ziglang c++ -std=c++17 -I core/include examples/cpp/first_move.cpp core/build/libiai_actuator_driver.a -lwinmm -lsetupapi -ladvapi32 -o first_move_cpp.exe
python -m ziglang c++ -std=c++17 -I core/include examples/cpp/multi_axis_cell.cpp core/build/libiai_actuator_driver.a -lwinmm -lsetupapi -ladvapi32 -o multi_axis_cell_cpp.exe
```

On Linux the C example also needs `-lpthread -lm`, and the C++ examples `-lpthread`. The C program finds
`libiai_actuator_driver.so` through `LD_LIBRARY_PATH` (`LD_LIBRARY_PATH=core/build ./first_move_c --sim`).

**C#** (.NET 8 SDK): `dotnet run` builds and runs the program. Each project references
`bindings/dotnet/src/IaiActuatorDriver`, which copies the native library from `core/build/` next to the program. For a
library elsewhere:
`dotnet build examples/csharp/FirstMove -p:NativeLibraryPath=C:\path\to\iai_actuator_driver.dll`.

## Options and behaviour

| Option | Meaning |
|---|---|
| `--sim` | the built-in simulator: no hardware, no serial port |
| `--sim-profile NAME` | single-axis examples and the Python cell examples: `slider` (default), `gripper`, `rod`, `pcon`, `rotary`, `unknown` |
| `--sim-axes N` | Python cell examples (`multi_axis_cell.py`, `machine_sequence.py`): N simulated ports, one axis each (1 to 16, default 3) |
| `--sim-profiles A,B,...` | C++ and C# cell examples: one simulated port per profile (default `slider,rod,gripper`) |
| `--port COM3` | the unit on this port; the baud rate is found automatically. Cell examples take `--port` more than once; in C++ and C# `COM3:2` names address 2 on COM3 |
| `--slave N` | single-axis examples: the unit at address N on a shared port (address = axis number + 1) |
| `--speed V` | mm/s (deg/s on a rotary axis). Default 20, raised to the unit's documented minimum speed where that is higher |
| `--distance D`, `--step D` | the size of the moves around mid-stroke (default 10, at most a quarter of the stroke) |
| `--home` | a home return even when the status says homed |

- Exit codes: 0 done, 2 error, 130 interrupted. `machine_sequence.py` and `recovery_after_power_loss.py` end with 1
  when an alarm needs the operator.
- Ctrl+C (Ctrl+Break on Windows too) ends an example at any time. A decel STOP goes to every axis that may move, from a
  thread of its own, and the servos are left as they are. A second Ctrl+C ends the program at once.
- An error after a motion command sends a decel STOP before the port is closed.

## On hardware

Follow the [safety rules](../docs/GETTING_STARTED.md#safety): close IA-OS first, a new setup unloaded, mid-stroke and
slow. The examples home an axis that reports no home (every axis with `--home`): keep the travel clear. The first-move
and cell examples switch the servo off at the end, and a vertical axis without a brake can fall then; after Ctrl+C or
an error they leave it as it is. Only `python/parameters_and_backup.py --force` writes non-volatile memory (one commit
of a value the unit already holds); the other examples write none.

The C, C++ and C# examples are UNVERIFIED on hardware: they were run on `--sim` with every simulator profile. What each
language is verified on: [docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md#which-editions-were-run-on-a-unit).
