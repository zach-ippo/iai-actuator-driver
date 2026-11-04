# IAI Actuator Driver: Python package

The package `iai_actuator_driver` drives IAI ELECYLINDER units and CON-family controllers over serial Modbus RTU, every
axis it finds (up to 16 per port). Every IAI unit that speaks Modbus RTU on its serial port is driven the same way, with
its ranges read from the unit. It installs the commands `iai-actuator-driver` and `iai-check`. The old import name
`ec_driver` still works but is deprecated, and is removed in 2.0.

Unofficial: not an IAI product ([README](https://github.com/zach-ippo/iai-actuator-driver/blob/main/README.md)). Start with
[docs/GETTING_STARTED.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/GETTING_STARTED.md) and the
[Python guide](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/getting-started/python.md). Every function is in
[docs/DRIVER.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/DRIVER.md); changes are in [CHANGELOG.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/CHANGELOG.md). Bugs and results from your unit
go to [GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose).

Licence: Apache License 2.0 ([LICENSE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/LICENSE), [NOTICE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/licenses/NOTICE)).

## Install

Python 3.11 or newer; the only dependency is pyserial. The package runs on the shared C++ core, a native library: a
release has platform wheels that carry it. The sdist is source, and needs the native library: like an install of this
folder, it carries none. Build it with `python build.py shared` in `core/` of a checkout, or name the library file with
`IAI_ACTUATOR_DRIVER_NATIVE`. From a checkout, in the repository root:

```shell
pip install ./bindings/python        # iai-actuator-driver and iai-check
pip install -e bindings/python       # editable, to work on the source
```

Without a clone, pip installs this folder from git with `#subdirectory=bindings/python` after the repository URL. The
GUI (`iai-gui`) is in `apps/gui/`.

## Use

```python
from iai_actuator_driver import Axis, SimulatedEC

with Axis.open("COM3") as ax:          # or Axis.open(SimulatedEC()) to try it with no hardware
    ax.connect()                       # identity and the unit's own limits
    ax.servo(True)
    ax.home()                          # moves to the mechanical end: keep the travel clear
    ax.move_to(20.0, speed_mm_s=20)    # waits until in position; out-of-range values are refused
```

Positions are in mm (deg on a rotary axis), speeds in units/s, accelerations in G. `Cell` drives several axes at once.

## The native library

The package runs on the shared C++ core and loads its native library at import. The library is searched in
`IAI_ACTUATOR_DRIVER_NATIVE` (a file or a folder), then inside the package (`_native/<platform>/`, where a platform
wheel puts it), then in `core/build/` of a checkout. When it is missing or has another ABI, the import raises
`ImportError` saying where it looked. `iai-actuator-driver --version` names the core and its ABI;
`iai_actuator_driver.backend_info()` says which library was loaded.

`iai_actuator_driver.native` is the core's binding itself (`NativeAxis`, `NativeCell`). It needs C ABI 1.0 and
accepts any 1.x library (the same ABI major and at least the minor it was written for).

## Tests

The tests use the simulator only and open no serial port. In this folder:

```shell
python -m unittest discover -s tests -q
python tests/run_backend.py                      # parallel workers, a time limit per test
```

Both need the native library (`core/build/` of the checkout, or `IAI_ACTUATOR_DRIVER_NATIVE`).
