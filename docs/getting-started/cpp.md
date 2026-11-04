# Getting started: C++

The C++17 core (folder `core/`) is the driver itself: the bus, the checks before every frame, the completion rule,
STOP, the non-volatile write guard, the multi-axis `Cell` and the simulator. Python and .NET call it through its C ABI;
a C++ program can link it directly. Unofficial: not an IAI product ([README](../../README.md)).

Design and scope: [core/DESIGN.md](../../core/DESIGN.md) and [core/README.md](../../core/README.md).

## Two ways to link

| Way | Header | Link | When |
|---|---|---|---|
| The C++ API | `iai_actuator_driver/iai_actuator_driver.hpp` (namespace `iai_actuator_driver`) | the static library `iai_actuator_driver`, built with your compiler | C++ programs built from source with CMake |
| The C ABI | `iai_actuator_driver.h` (`iad_` functions) | the shared library `iai_actuator_driver.dll` / `libiai_actuator_driver.so` | one binary that does not depend on your compiler; see [c.md](c.md) |

- The C++ classes are not a binary interface. Build the core with the same compiler and settings as your program (the
  CMake way below does that). Standard C++17, no third-party dependencies.
- The C ABI is the stable interface: C ABI 1.0, frozen within major 1 ([../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md)).
  A C++ program that must survive a library update without a rebuild calls the C ABI.
- A release's C/C++ SDK zip ships only the C ABI prebuilt. For the C++ API, build `core/` from source.
- The examples in `examples/cpp` link the static library, so no ABI check applies to them.

## Build

CMake 3.14 or newer with MSVC, MinGW, clang or gcc:

```shell
cmake -S core -B core/build -DCMAKE_BUILD_TYPE=Release
cmake --build core/build --config Release
ctest --test-dir core/build -C Release        # simulator and scripted frames only; no test opens a serial port
```

In your own project (the target `iai_actuator_driver` is the static library, alias
`iai_actuator_driver::iai_actuator_driver`; on Windows it links `winmm`, `setupapi` and `advapi32` by itself):

```cmake
set(IAI_ACTUATOR_DRIVER_BUILD_TESTS OFF)      # optional: skip the core's tests and command line
set(IAI_ACTUATOR_DRIVER_BUILD_CLI OFF)
add_subdirectory(path/to/iai-actuator-driver/core iai_actuator_driver)
target_link_libraries(your_app PRIVATE iai_actuator_driver)
```

The other targets, the zig build for a Windows x64 PC without CMake or MSVC (`python build.py` in `core/`) and the
shared library: [core/README.md](../../core/README.md#build).

## A simulated axis

`SimulatedUnit` is a `Transport` that answers frames like a unit, so the code below runs without hardware and opens
no serial port. The third argument is the time scale (20 = twenty times real time).

```cpp
#include <iostream>
#include <memory>

#include "iai_actuator_driver/iai_actuator_driver.hpp"
using namespace iai_actuator_driver;

int main() {
    Axis ax = Axis::open(std::make_shared<SimulatedUnit>(SimProfile::Gripper, 1, 20.0));
    DeviceInfo info = ax.connect();               // identity and the unit's own ranges; Modbus control taken at rest
    AxisRanges r = ax.limits();                   // every range the driver enforces, read from the unit
    std::cout << info.model() << " (" << ax.kind() << "), position up to "
              << r.position.max.value_or(0.0) << " " << ax.units() << "\n";
}
```

- Profiles: `SimProfile::Slider` (default), `Gripper`, `Rod`, `Pcon`, `Rotary`, `Unknown`
  ([what each simulates](python.md#a-simulated-axis)).
- Several units: `SimLine` puts several simulated units on one line. `Cell::open(SimLine::sim_ports(3))` gives a cell
  of three simulated ports ("SIM1:1" .. "SIM3:1").
- On hardware: `Axis::open("COM3")` (`"/dev/ttyUSB0"` on Linux). The baud rate is found in place. Close IA-OS first:
  only one program can use a serial port.

## First move

```cpp
Axis ax = Axis::open(std::make_shared<SimulatedUnit>(SimProfile::Gripper, 1, 20.0));
ax.connect();
ax.servo(true);                                   // refused with reasons() when the status shows it cannot come on
ax.home();                                        // moves to the mechanical end: keep the travel clear
std::optional<Status> st = ax.move_to(5.0, 20.0); // 5 mm at 20 mm/s; waits until in position and settled
std::cout << "at " << st->position_mm << " " << ax.units() << "\n";

MoveOptions o;                                    // speed 50, accel 0.1 G, band 0.1, wait by default
o.speed = 20.0;
ax.move_by(-2.0, o);                              // relative to the measured position
ax.servo(false);
```

- Positions in mm (deg on a rotary axis), speeds in units/s, accelerations in G. A value outside the unit's own
  ranges throws `SoftLimitError` and nothing moves. `ax.check_move(position, o)` runs the same checks without moving.
- The caps default to the unit's own maxima. Narrow them for a first test on hardware: `OpenOptions opt;
  opt.axis.max_speed_mm_s = 100; opt.axis.max_accel_g = 0.3; Axis::open("COM3", opt);`. A cap can only narrow the
  unit's range.
- `o.wait = false` returns `std::nullopt` right after the command; then call `ax.wait_idle(target, band)`.

Before a move on hardware read [Safety](../GETTING_STARTED.md#safety). Complete programs:
[examples/cpp/](../../examples/cpp/).

## Stop

`ax.stop()` is a decelerate-stop over Modbus ([how it works](python.md#stop)). `ax.stop(true, 5.0)` throws
`MotionTimeoutError` if the axis still moves after 5 s.

```cpp
#include <chrono>
#include <thread>

Axis ax = Axis::open(std::make_shared<SimulatedUnit>(SimProfile::Slider));   // real time: still moving at the stop
ax.connect();
ax.servo(true);
ax.home();

std::thread stopper([&ax] {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ax.stop();                                    // from another thread while move_to waits
});
try {
    ax.move_to(140.0, 20.0);
} catch (const MotionIncompleteError& e) {        // settled away from the target: stopped from outside
    std::cout << "move ended: " << e.what() << "\n";
}
stopper.join();
```

- `pause()`, `resume()` and `cancel_paused_move()` hold, continue and drop a move.
- `close()` (and the destructor) releases a STOP or pause the axis still holds, and closes the port when the axis owns
  it.
- `Cell::stop_all()` sends STOP to every axis first, then waits for each.
- Not a safety function. There is no communication watchdog: if the PC or the cable fails, a move in progress is
  completed.

## Errors

Every exception the core throws on purpose derives from `EcDriverError` (`errors.hpp`). Refusals happen before
anything is sent, and the message says so ("nothing sent", "nothing written").

```text
EcDriverError
+- ModbusError: ModbusTimeout, ModbusCrcError, ModbusFrameError, ModbusSlaveError, IaiCommandRejected, TransportError
+- UnsafeCommandError                     a never-send or not-allowed IAI command, a read-only parameter; nothing sent
+- EcError
   +- SoftLimitError                      a value outside the unit's range or a cap; nothing sent
   +- EcNotReadyError                     servo off, not homed, moving, no Modbus control
   |  +- ServoBlockedError                EMG input, drive power cut or enable switch (reasons())
   +- EcAlarmError                        the unit reports an alarm (alarm_code())
   +- MotionTimeoutError                  not done in time (a decel stop was sent)
   +- MotionIncompleteError               settled away from the target
   +- NonVolatileWriteError               a row that cannot be written (rows 256-267, not addressable)
   +- VerificationError                   the read-back after a write did not match
CellError (cell.hpp)                      one or more axes of a group command failed (messages(), errors() per axis)
```

```cpp
try {
    ax.move_to(500.0, 20.0);
} catch (const SoftLimitError& e) {
    std::cerr << "refused, nothing sent: " << e.what() << "\n";
} catch (const EcAlarmError& e) {
    std::cerr << "alarm 0x" << std::hex << e.alarm_code() << "\n";
} catch (const EcDriverError& e) {
    std::cerr << e.what() << "\n";              // message plus notes, e.g. [decel stop sent after this error]
    if (e.stop_attempted()) { /* the axis already sent its decel stop */ }
    throw;
}
```

`e.message()` is the message alone and `e.notes()` the notes. Group commands take `return_exceptions = true` to
return the failures in `CellResults` (`messages`, `errors`) instead of throwing a `CellError`.

**Non-volatile writes.** `write_position_table`, `teach_position` and `write_parameter` go straight through. A risky
write gives a warning (`warnings`), an unchanged value or row is not written again (`skipped`) and
`nonvolatile_writes()` counts the writes. For a target that changes every cycle use `move_to`. The rules:
[Non-volatile writes](../TROUBLESHOOTING.md#non-volatile-writes).

## Threads

- One bus mutex per port: axes that share a port take turns, axes on different ports run in parallel.
- A cell takes every axis it finds (no total limit; `CellOptions::max_axes` caps it if given). One cable carries at
  most 16 axes ([Several axes on one port](../HARDWARE_SETUP.md#several-axes-on-one-port)).
- The `Axis` guards its own state. `stop()` and `pause()` may come from another thread while a move, home or
  `wait_idle()` waits on the same axis. The waiting call then ends with `MotionIncompleteError`.
- `Cell` runs each port in its own thread inside a group command and returns when every axis is done. With
  `stop_on_fault` (the default) a link fault on one axis stops the others.
- `CellOptions::on_fault(name, kind, message)` is called on a core thread. Keep it short and never call a blocking
  axis command from it.
- Logging is silent by default. `set_log_handler([](LogLevel level, const std::string& source, const std::string&
  message) { ... })` (`log.hpp`) receives every message on the thread that logs.

## The native library

A C++ program built the CMake way above needs no native library: the core is linked in. The shared library is for
other languages and for programs that should not depend on the core's compiler: [c.md](c.md#the-library).
`cmake --install` installs the headers (`iai_actuator_driver.h`, and `iai_actuator_driver_testing.h` with the test
hooks), the static library and, when built, the shared library.

## Command line

```shell
iai-actuator-driver-cpp ports                                  # serial ports, IAI adapters first; opens nothing
iai-actuator-driver-cpp --sim --sim-profile gripper limits     # the simulator
iai-actuator-driver-cpp --port COM3 status                     # read-only
iai-actuator-driver-cpp --port COM3 move 20 --speed 10
```

`--help` lists every command; exit codes and the full list: [core/README.md](../../core/README.md#command-line).

## Evidence

| Part | Evidence |
|---|---|
| `iai-actuator-driver-cpp` on the EC-GRBP8M gripper and the PCON-CB with an RCP6 | VERIFIED for the steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) and [Hardware results](../COMPATIBILITY.md#hardware-results) |
| Cell, teach, parameter writes, gateway lines on hardware | UNVERIFIED |
| Linux serial port (`serial_posix.cpp`) | VERIFIED ([../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit)) |
| macOS serial port | UNVERIFIED |

## Next

- [core/README.md](../../core/README.md): status, bus rules, the command line in full.
- [c.md](c.md): the C ABI over this core.
- [../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md): handles, errors, threads and versioning of the C ABI.
