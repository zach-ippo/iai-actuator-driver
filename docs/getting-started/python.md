# Getting started: Python

The package `iai_actuator_driver` (folder `bindings/python/`) drives ELECYLINDER units and CON-family controllers over
serial Modbus RTU. It also installs the command line `iai-actuator-driver` and the read-only self-test `iai-check`.
Unofficial: not an IAI product ([README](../../README.md)).

The cable and the command line are in [../GETTING_STARTED.md](../GETTING_STARTED.md). Every function is in
[../DRIVER.md](../DRIVER.md).

Write programs against `from iai_actuator_driver import Axis, Cell` (`Axis` is `Elecylinder`). It runs on the C++
core, so the package needs its native library
([Where the native library comes from](#where-the-native-library-comes-from)). `iai_actuator_driver.native`
(`NativeAxis`, `NativeCell`) is the raw ctypes binding of the C ABI under it, for tests
([../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md)).

## Install

Python 3.11 or newer. The only dependency is pyserial.

```shell
pip install iai_actuator_driver-<version>-py3-none-<platform>.whl   # from a release: the library is inside
iai-actuator-driver --version
```

From a source checkout, `pip install ./bindings/python ./apps/gui` (the GUI adds `iai-gui`), or
`pip install -e ./bindings/python` to work on the source. A checkout install needs the library built or named
([below](#where-the-native-library-comes-from)).

## A simulated axis

`SimulatedEC` answers frames like a unit, so the code below runs without hardware and opens no serial port.
`time_scale` runs the simulated clock faster (20 = twenty times real time).

```python
from iai_actuator_driver import Axis, SimulatedEC

with Axis.open(SimulatedEC(profile="gripper", time_scale=20)) as ax:
    info = ax.connect()                    # identity and the unit's own ranges; Modbus control taken at rest
    print(info.model, ax.units, ax.kind)
    print(ax.limits()["position"])         # the range every target is checked against
```

| Profile | Simulates |
|---|---|
| `slider` (default) | the EC-DS4H slider |
| `gripper` | the EC-GRBP8M gripper |
| `rod` | the EC-R6S rod |
| `pcon` | the PCON-CB with an RCP6 |
| `rotary`, `unknown` | made-up units for the generic paths (deg units; a unit the driver has no name for) |

The four real profiles serve register blocks read from those units. A pass on the simulator shows the driver agrees
with what was observed on them, not that every unit behaves this way.

On hardware, pass the port name: `Axis.open("COM3")`. The baud rate is found in place.
Close IA-OS first: only one program can use a serial port.

## First move

```python
from iai_actuator_driver import Axis, SimulatedEC

with Axis.open(SimulatedEC(profile="gripper", time_scale=20)) as ax:
    ax.connect()
    ax.servo(True)                         # refused with the reasons when the status shows it cannot come on
    ax.home()                              # moves to the mechanical end: keep the travel clear
    st = ax.move_to(5.0, speed_mm_s=20)    # absolute; waits until in position and settled
    print(f"at {st.position_mm:.2f} {ax.units}")
    st = ax.move_by(-2.0, speed_mm_s=20)   # relative to the measured position
    ax.servo(False)
```

- Positions in mm (deg on a rotary axis), speeds in units/s, accelerations in G. Every target, speed, acceleration and
  band is checked against the unit's own ranges first: a value outside them raises `SoftLimitError` and nothing moves.
- The caps default to the unit's own maxima. Narrow them for a first test on hardware:
  `Axis.open("COM3", max_speed_mm_s=100, max_accel_g=0.3)`. A cap can only narrow the unit's range.
- `wait=False` returns at once with a `MoveHandle`.

Before a move on hardware read [Safety](../GETTING_STARTED.md#safety). The complete program is
[examples/python/first_move.py](../../examples/python/first_move.py) (`--sim` or `--port COM3`).

## Stop

`ax.stop()` is a decelerate-stop over Modbus. It holds the STOP coil until the axis stands still, then releases it. It
works with or without Modbus control and blocks for the deceleration. With `wait=True` it raises
`MotionTimeoutError` if the axis still moves after `timeout` (default 5 s).

```python
import threading
import time

from iai_actuator_driver import Axis, MotionIncompleteError, SimulatedEC

with Axis.open(SimulatedEC(profile="slider")) as ax:   # real time, so the move is still running at the stop
    ax.connect(); ax.servo(True); ax.home()

    def move() -> None:
        try:
            ax.move_to(140.0, speed_mm_s=20)
        except MotionIncompleteError as exc:          # settled away from the target: stopped from outside
            print("move ended:", exc)

    mover = threading.Thread(target=move)
    mover.start()
    time.sleep(1.0)
    ax.stop()                                         # from this thread while move_to waits in the other
    mover.join()
```

- `pause()` holds the move and keeps the remaining travel. `resume()` continues it. `cancel_paused_move()` drops it.
- Leaving a `with` block on an error after a motion command sends a decel stop. The exception's notes say so.
- STOP is VERIFIED on moves the PC commanded and UNVERIFIED on a move a PLC commanded over PIO. Stop those from the
  PLC or with the hardware stop.
- Not a safety function. There is no communication watchdog: if the PC or the cable fails, a move in progress is
  completed.

## Errors

Every error the driver raises on purpose derives from `EcDriverError`. Refusals happen before anything is sent, and
the message says so ("nothing sent", "nothing written").

| Exception | When |
|---|---|
| `SoftLimitError` | a target, speed, acceleration, band or force outside the unit's range or a cap |
| `EcNotReadyError` (`ServoBlockedError`) | servo off, not homed, moving, no Modbus control; `ServoBlockedError.reasons`: EMG input, drive power cut, enable switch |
| `EcAlarmError` | the unit reports an alarm (`alarm_code`) |
| `MotionTimeoutError`, `MotionIncompleteError` | not done in time (a decel stop was sent) / settled away from the target |
| `NonVolatileWriteError`, `UnsafeCommandError` | a row that cannot be written (rows 256-267), a read-only parameter, a never-send IAI command |
| `ModbusTimeout`, `ModbusCRCError`, `ModbusFrameError`, `ModbusException`, `TransportError` | the link: no answer, bad frame, exception reply, port gone |
| `CellError` | one or more axes of a group command failed (`errors` and `results` per axis name) |

```python
from iai_actuator_driver import EcAlarmError, EcDriverError, SoftLimitError
from iai_actuator_driver.alarms import alarm_info

try:
    ax.move_to(500.0, speed_mm_s=20)
except SoftLimitError as exc:
    print("refused, nothing sent:", exc)
except EcAlarmError as exc:
    print("alarm", alarm_info(exc.alarm_code)["name"])
except EcDriverError as exc:
    for note in getattr(exc, "__notes__", []):        # e.g. "decel stop sent after this error"
        print(note)
    raise
```

An error from the core also carries `iad_code` (the C result code) and `iad_details` (its error record). A missing or
mismatched native library raises `NativeLibraryError`.

**Non-volatile writes.** Position-table writes, teach and parameter writes go straight through. A risky write gives a
warning (`warnings`, `last_result()`), and an unchanged value or row is not written again (`skipped`). For a target
that changes every cycle use `move_to`. The rules: [Non-volatile writes](../TROUBLESHOOTING.md#non-volatile-writes).

## Threads

- One command at a time per axis. `stop()`, `pause()` and `resume()` may come from another thread while a command
  waits; the waiting move then ends with `MotionIncompleteError`. That is how a stop button stops a blocking move.
- Axes on different ports run in parallel. Axes that share a port take turns on its bus lock; a STOP from another
  thread gets the bus between two status polls.
- A cell takes every axis it finds (no total limit; `max_axes` caps it if given). One cable carries at most 16 axes
  ([Several axes on one port](../HARDWARE_SETUP.md#several-axes-on-one-port)).
- `Cell` group commands (`servo_all`, `home_all`, `move_all`, `wait_all`, `stop_all`) run the ports in parallel and
  return when every axis is done. `stop_all()` may come from another thread while a group command waits.
- Ctrl+C during a move sends a decel stop before `KeyboardInterrupt` propagates. ctypes releases the GIL during each
  call, so a stop from another thread reaches the core while a move waits.
- Logging uses the standard `logging` module under `iai_actuator_driver.*`. It is silent unless configured.

## Where the native library comes from

Importing the package loads the core with ctypes (`iai_actuator_driver.dll` on Windows,
`libiai_actuator_driver.so` on Linux, `libiai_actuator_driver.dylib` on macOS; no compiler is needed on the PC) and
checks its ABI version. It looks in this order:

1. `IAI_ACTUATOR_DRIVER_NATIVE`: the library file or a folder holding it (only when the variable is set).
2. Inside the package: `iai_actuator_driver/_native/<platform>/` (`win-x64`, `linux-x64`, `linux-arm64`,
   `osx-arm64`...), then `iai_actuator_driver/_native/` and the package folder. A platform wheel puts it here.
3. A source checkout used in place (`pip install -e`): `core/build/` at the repository root.

A `pip install` of the folder copies only the Python files, so it finds the library through 1 or, editable, 3. When
none is usable the import raises `ImportError` saying where it looked. `iai_actuator_driver.backend_info()` and
`iai-actuator-driver --version` name the library loaded and its ABI. To build it from source (zig on Windows x64,
CMake on Linux): [core/README.md](../../core/README.md#build).

Check it:

```python
from iai_actuator_driver import native
print(native.available(), native.find_library())
print(native.abi_version(), native.library_version())
```

The binding checks the library's ABI version when it loads: the same ABI major and a library minor at least its own
(C ABI 1.0, frozen within major 1). Use the library and the package of the same release.

## Evidence

| Part | Evidence |
|---|---|
| The package API | VERIFIED on the EC-GRBP8M gripper; steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) |
| `iai_actuator_driver.native` | VERIFIED on the PCON-CB with an RCP6; steps in [../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit) |
| Everything else | UNVERIFIED (tested on the simulator) |
| Linux serial port | VERIFIED ([../COMPATIBILITY.md](../COMPATIBILITY.md#which-editions-were-run-on-a-unit)) |
| macOS serial port | UNVERIFIED |

## Next

- [../DRIVER.md](../DRIVER.md): every function with its inputs and ranges.
- [../../examples/python/](../../examples/python/): short programs, one task each, on the simulator or a unit.
- [../dev/NATIVE_ABI.md](../dev/NATIVE_ABI.md): the C ABI under `iai_actuator_driver.native`.
- [../TROUBLESHOOTING.md](../TROUBLESHOOTING.md): error messages and what to do.
