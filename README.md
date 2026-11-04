# IAI Actuator Driver

Drive IAI actuators from a PC over the unit's RS-485 port with Modbus RTU, with no PNP/NPN I/O wiring. Python, C#, C
and C++ libraries, a command line and a local web GUI share one C++ core. Version 1.2.0.

> **UNOFFICIAL.** This is not an IAI product and is not supported, documented or endorsed by IAI. It is not a safety
> function: keep a hardware emergency stop.

## What it does

- Drives ELECYLINDER (EC) units, RoboCylinders on CON-family controllers (PCON, ACON, DCON, SCON) and RCON / ROBONET
  lines: servo, home, moves, jog, push, stored positions, stop, pause, alarms, parameters and backup.
- Reads every range from the unit itself and checks each move against it before anything is sent.
- Runs many axes through `Cell` (up to 16 per cable). Coordinated motion is alpha
  ([docs/DRIVER.md](docs/DRIVER.md#coordinated-motion-alpha)).
- Includes a simulator, so everything runs without hardware.

## Install

Download the files from the [releases](https://github.com/zach-ippo/iai-actuator-driver/releases) page (Windows x64,
Linux x64; `<version>` is the release number):

| You use | File | Then |
|---|---|---|
| Python 3.11+ | `iai_actuator_driver-<version>-py3-none-win_amd64.whl` or the `manylinux` wheel | `pip install <file>` |
| C# / .NET | `IaiActuatorDriver.<version>.nupkg` | `dotnet add package IaiActuatorDriver --version <version> --source <folder>` |
| C, LabVIEW, any C FFI | `iai_actuator_driver-sdk-<version>-win-x64.zip` or `-linux-x64.zip` | Unzip; see its `README.txt` |
| The GUI (Windows) | `iai-gui-<version>-win-x64.zip` | Unzip and run `iai-gui.exe` |

## Quick start

Try the simulator first, and close IA-OS before you open a real port.

```python
from iai_actuator_driver import Axis, SimulatedEC

with Axis.open(SimulatedEC()) as ax:       # a real unit: Axis.open("COM3")
    ax.servo(True)
    ax.home()                              # moves to the mechanical end: keep the travel clear
    ax.move_to(20.0, speed_mm_s=20)        # waits until in position; out-of-range values are refused
    ax.servo(False)
```

```sh
iai-actuator-driver --sim status          # the built-in simulator
iai-actuator-driver scan                  # which addresses answer on each IAI USB adapter (read-only)
iai-check --port COM3                     # read-only self-test with a JSON report
```

## Which units work

Every IAI unit that speaks Modbus RTU on its serial port is driven the same way, with no model list. Run on hardware
so far: ELECYLINDER 24 V units and a PCON-CB with an RCP6. Not supported: SEL-protocol controllers (XSEL, SSEL, ASEL,
PSEL, MSEL, TTA), ELECYLINDER on an RCON-EC / REC unit, ROBONET in gateway mode. Details:
[docs/COMPATIBILITY.md](docs/COMPATIBILITY.md).

## Safety

- There is no communication watchdog: if the PC crashes or the cable is pulled, a move in progress completes. The
  software STOP goes over a serial line and can be late: keep a hardware emergency stop or STO.
- Start slow: unloaded, mid-stroke, 20 mm/s or less. Back up first: `iai-actuator-driver --port COM3 backup unit1.json`.

## Documentation

[Getting started](docs/GETTING_STARTED.md), [Hardware setup](docs/HARDWARE_SETUP.md),
[Compatibility](docs/COMPATIBILITY.md), [Driver API](docs/DRIVER.md), [Reference](docs/REFERENCE.md),
[Troubleshooting and questions](docs/TROUBLESHOOTING.md), [Examples](examples/README.md), [GUI](apps/gui/README.md),
[Changelog](CHANGELOG.md). Language guides: [Python](docs/getting-started/python.md),
[C#](docs/getting-started/csharp.md), [C++](docs/getting-started/cpp.md), [C](docs/getting-started/c.md). For
maintainers: [C ABI](docs/dev/NATIVE_ABI.md), [Architecture](docs/dev/ARCHITECTURE.md),
[Contributing](.github/CONTRIBUTING.md).

Claims carry one of four evidence words: **VERIFIED** (tried on hardware), **ON-WIRE** (seen in serial traffic),
**DOC** (in a public IAI manual), **UNVERIFIED** (expected, not yet tried).

## Support and licence

Bugs, questions and results from your unit go to
[GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose) (one form for a bug or a unit
result), not to IAI.
Apache License 2.0: [LICENSE](LICENSE), [NOTICE](licenses/NOTICE),
[NATIVE_THIRD_PARTY.txt](licenses/NATIVE_THIRD_PARTY.txt).
