# IAI Actuator Driver

Drive IAI actuators from a PC over the unit's RS-485 port with Modbus RTU, with no PNP/NPN I/O wiring. Libraries for
Python, C#, C and C++, a command line and a local web GUI share one C++ core, so the same checks hold in every
language. Version 1.1.1.

> **UNOFFICIAL.** This is not an IAI product and is not supported, documented or endorsed by IAI. Driving an
> ELECYLINDER over Modbus from a PC is not an officially supported use. The units it has been run on so far are listed
> in [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md). It is not a safety function: keep a hardware emergency stop.

## What it does

- Drives ELECYLINDER (EC) units, RoboCylinders on CON-family controllers such as the PCON, and RCON / ROBONET lines.
- Uses one axis model for every unit, in mm (degrees on a rotary axis). Ranges are read from the unit itself, never
  from a model list, and every move is checked against them before anything is sent.
- Covers servo, home, moves, jog, push, stored positions, stop, pause, alarms, parameters and backup.
- Runs every axis it finds through `Cell` (no total limit; `max_axes` caps it), one per COM port or up to 16 on one
  port by address. Coordinated motion is
  alpha, UNVERIFIED on two real axes ([docs/COORDINATED.md](docs/COORDINATED.md)).
- Includes a simulator, so every language, the command line and the GUI run without hardware.

## Install

Download the files for Windows x64 or Linux x64 from the
[GitHub releases](https://github.com/zach-ippo/iai-actuator-driver/releases) page.

| You use | File | Then |
|---|---|---|
| Python 3.11+ | `iai_actuator_driver-1.1.1-py3-none-win_amd64.whl` or the `manylinux` wheel | `pip install <file>`; adds the `iai-actuator-driver` and `iai-check` commands |
| C# / .NET | `IaiActuatorDriver.1.1.1.nupkg` | `dotnet add package IaiActuatorDriver --version 1.1.1 --source <folder>` |
| C, LabVIEW, any C FFI | `iai_actuator_driver-sdk-1.1.1-win-x64.zip` or `-linux-x64.zip` | Unzip; its `README.txt` has the CMake and compile lines |
| The GUI (Windows) | `iai-gui-1.1.1-win-x64.zip` | Unzip and run `iai-gui.exe`; nothing else to install |

`iai-gui.exe` is not code-signed, so Windows SmartScreen may warn. To build from source, including the C++ classes,
see [CONTRIBUTING.md](.github/CONTRIBUTING.md) and the language guides.

## Quick start

Try it on the simulator first. Close IA-OS before you open a real port: only one program can use a port.
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) walks through the cable, the first scan and a first slow move.

```python
from iai_actuator_driver import Axis, SimulatedEC

with Axis.open(SimulatedEC()) as ax:       # a real unit: Axis.open("COM3")
    print(ax.connect().model)              # identity and the unit's own limits
    ax.servo(True)
    ax.home()                              # moves to the mechanical end: keep the travel clear
    ax.move_to(20.0, speed_mm_s=20)        # waits until in position; out-of-range values are refused
    ax.stop()                              # decelerate-stop, usable at any time
    ax.servo(False)
```

From the command line:

```sh
iai-actuator-driver --sim status                # the built-in simulator, no hardware
iai-actuator-driver scan                        # which addresses answer on each IAI USB adapter (read-only)
iai-check --port COM3                           # read-only self-test; writes a JSON report
iai-actuator-driver --port COM3 status          # read-only; --help lists every command
```

## Which units work

Every IAI unit that speaks Modbus RTU on its serial port is supported and driven the same way, with no driver update
for a new model: the driver reads ranges and capabilities from the unit and keeps no model list.

| | Units |
|---|---|
| **Supported** | ELECYLINDER (EC) 24 V and 200 V of every type; CON-family controllers (PCON, ACON, DCON, SCON); RCON driver units through the gateway; ROBONET in through mode |
| **Run on hardware so far** | ELECYLINDER 24 V slider, dust/drip-proof slider, rod and gripper; PCON-CB with RCP6 (a record of what ran, not a list of allowed units) |
| **Not supported** | SEL-protocol controllers (XSEL, SSEL, ASEL, PSEL, MSEL, TTA); ELECYLINDER on an RCON-EC / REC unit; ROBONET in gateway mode |

A Linux serial port (from 1.1.0: the IAI USB adapter, in WSL 2) and a C program on a unit are VERIFIED; macOS
serial ports and LabVIEW on a unit are UNVERIFIED. Details and possible trouble:
[docs/COMPATIBILITY.md](docs/COMPATIBILITY.md).

## Languages

| Language | API | Guide |
|---|---|---|
| Python 3.11+ | `iai_actuator_driver`: `Axis`, `Cell` | [python.md](docs/getting-started/python.md) |
| C# (.NET Framework 4.6.2+, .NET 6, .NET 8) | `IaiActuatorDriver`: `Axis`, `Cell` | [csharp.md](docs/getting-started/csharp.md) |
| C++17 | `iai_actuator_driver/iai_actuator_driver.hpp`: `Axis`, `Cell` | [cpp.md](docs/getting-started/cpp.md) |
| C99, LabVIEW, any C FFI | `iai_actuator_driver.h`: `iad_` functions | [c.md](docs/getting-started/c.md) |

## Safety

- **No communication watchdog.** If the PC crashes or the cable is pulled, a move in progress completes and a push
  keeps pushing. Design the machine so that this is safe, and keep a hardware emergency stop.
- **Start slow.** Test a new setup unloaded, mid-stroke, at 20 mm/s or less. A vertical axis can fall when its servo
  goes off.
- **Writes that change the unit need no opt-in:** position-table rows, teaching, parameters (home direction and offset
  included) and baud changes go through, a risky one with a warning. A value outside the unit's range is refused, an
  unchanged value is not written, and raw IAI commands that can disable a unit (firmware update, parameter
  initialisation, alarm-list clear, axis number) are never sent unless `allow_never_send`. Row and parameter writes
  use non-volatile write cycles; the driver counts them and warns above 60 an hour. Back up first:
  `iai-actuator-driver --port COM3 backup unit1.json`.
- **STOP is a Modbus decelerate-stop.** VERIFIED on Modbus moves and on moves from a hand-wired PIO input (1.0.0,
  2026-10-31, a push row's move included), UNVERIFIED with a PLC.
- **The software STOP is not a safety function.** It goes over a serial line from a general-purpose PC and can be
  late: a hardware emergency stop or STO is required wherever a late stop could hurt someone.

## Evidence words

Every claim about how a feature behaves carries one of four words. **VERIFIED**: tried on real hardware with this
driver. **ON-WIRE**: seen in observed serial traffic; the unit's reaction is VERIFIED only where also tested. **DOC**:
written in a public IAI manual, not yet confirmed on a unit. **UNVERIFIED**: expected, not yet tried. The words describe
features and paths, never a unit or a model: every unit is driven the same way.

## Versions and support

- Versions follow PEP 440. The C ABI is frozen within a major version: later 1.x releases add functions and change none.
- 1.0.x releases fix bugs only.
- Bugs, questions and results from your unit go to
  [GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose), with a form for a bug and one
  for a unit report.
- IAI does not support this driver: do not send it questions or reports about it.

## Documentation

| Document | Contents |
|---|---|
| [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) | Cable, install, first scan, GUI demo, first move |
| [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) | Adapter, port, power, emergency stop, baud rates, several axes on one port |
| [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md) | Supported units, hardware results, possible trouble |
| [docs/DRIVER.md](docs/DRIVER.md) | Every function: inputs, ranges, an example |
| [docs/REFERENCE.md](docs/REFERENCE.md) | Behaviour notes and their evidence |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md), [docs/FAQ.md](docs/FAQ.md) | Error messages and short answers |
| [examples/README.md](examples/README.md) | Short programs in every language, one task each |
| [apps/gui/README.md](apps/gui/README.md) | The GUI: launchers and a first run |
| [docs/PYTHON_FALLBACK.md](docs/PYTHON_FALLBACK.md) | The pure-Python fallback: removed after 1.0.x, what to do instead |
| [docs/NATIVE_ABI.md](docs/NATIVE_ABI.md), [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | The C ABI contract; design notes for maintainers |
| [CONTRIBUTING.md](.github/CONTRIBUTING.md), [CHANGELOG.md](CHANGELOG.md) | Building, tests and rules; what changed in each release |

## Licence

Apache License 2.0: see [LICENSE](LICENSE) and [NOTICE](licenses/NOTICE). What is linked into the native library
(libc++, zig's compiler_rt, on Windows the mingw-w64 runtime) is under its own licences:
[NATIVE_THIRD_PARTY.txt](licenses/NATIVE_THIRD_PARTY.txt).
