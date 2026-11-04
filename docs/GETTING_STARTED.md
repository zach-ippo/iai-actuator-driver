# Getting started

From the cable to the first move, with the Python package and the command line. Read [Safety](#safety) before you
move anything. Guides for C#, C++ and C are in [getting-started/](getting-started/).

IAI Actuator Driver is **unofficial**: not an IAI product, not supported by IAI.

## You need

- A PC with Python 3.11 or newer: Windows x64 or Linux x64 (macOS UNVERIFIED).
- An ELECYLINDER or CON-family controller, powered (24 V for an ELECYLINDER) and wired to your own emergency stop.
- An IAI USB adapter on the unit's RS-485 (SIO / teaching) port.

## 1. Install

Download the wheel for your platform from the
[GitHub release](https://github.com/zach-ippo/iai-actuator-driver/releases) and install it:

```shell
pip install iai_actuator_driver-<version>-py3-none-win_amd64.whl  # Linux: the manylinux wheel
iai-actuator-driver --version
```

This adds `iai-actuator-driver` and `iai-check`, with the C++ core inside. For the GUI on Windows, unzip
`iai-gui-<version>-win-x64.zip` and use `iai-gui.exe` wherever this page says `iai-gui`. To install from a source
checkout instead, see [the Python guide](getting-started/python.md#install).

## 2. Cable and port

Plug the adapter in. It appears as a COM port ("IAI USB to UART Bridge Controller"). The examples in these documents
use `COM3` for the unit and `COM4` for a second port; on Linux the port is `/dev/ttyUSB0`. Use your own port name.

- **Close IA-OS first**, and any teaching pendant or wireless session. Only one program can use the port.
- Do not unplug the cable while a move is running.

```shell
iai-actuator-driver ports  # lists the serial ports, IAI adapters first; opens nothing
```

## 3. Scan

```shell
iai-actuator-driver scan  # asks addresses 1 to 16 on every IAI adapter; read-only
```

Example output:

```text
COM3: 1 axis/axes at 38400 baud
  COM3:1       axis 0   EC 24 V, unit 0x02, firmware 0013: EC dust/drip-proof
```

The baud rate is found automatically. If nothing answers, see [If nothing answers](#if-nothing-answers).

## 4. Try the GUI without hardware

```shell
iai-gui --sim-only
```

1. The browser opens. If not, open the address printed in the console (it ends with `#k=` and this run's access key).
2. Press **Add simulator**, then **Servo ON** and **Home**, and move with the **Operate** panel.
3. **STOP** is always in the header (also Space or Esc). **Quit** or Ctrl+C in the terminal ends the GUI.

The **Functions** tab shows every driver function with the unit's own range and an example. Demo mode opens no
serial port. For real hardware, start `iai-gui` without `--sim-only`, tick your COM port and press **Connect selected
ports**. Connecting only reads from the unit.

## 5. Check your unit

```shell
iai-check --port COM3                   # read-only: identity, limits, status, alarms, positions
iai-actuator-driver --port COM3 limits  # every range the driver will enforce, from the unit itself
```

`iai-check` prints PASS / FAIL / WARN / SKIP per item and writes `iai-check-<timestamp>.json`. Every unit is driven
the same way, whatever its model; results from any unit are welcome: open an issue on
[GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose) ("A result from my unit") and
attach that file.

## 6. First move

Clear the travel, unload the axis and keep a hand on the emergency stop.

```shell
iai-actuator-driver --port COM3 servo on
iai-actuator-driver --port COM3 home                # only if the status says not homed; moves to the mechanical end
iai-actuator-driver --port COM3 move 20 --speed 10  # pick a value inside the range 'limits' showed
iai-actuator-driver --port COM3 stop
iai-actuator-driver --port COM3 servo off
```

- Positions are in mm (deg on a rotary axis).
- A target, speed, acceleration or push force outside the unit's own range is refused before anything is sent.
- Incremental units (no battery-less absolute option) must be homed after every power-up and software reset.

The same in Python:

```python
from iai_actuator_driver import Axis
with Axis.open("COM3") as ax:
    ax.connect(); ax.servo(True); ax.move_to(20.0, speed_mm_s=10); ax.stop()
```

## Safety

- **Not a safety function.** Keep a hardware emergency stop and guarding.
- **No watchdog.** If the PC crashes or the cable is pulled, a move in progress is completed and a push keeps pushing.
- **Start slow:** unloaded, mid-stroke, at 20 mm/s or less. The command line caps speed at 100 mm/s, acceleration at
  0.3 G and push force at 70 % by default. `--unit-max` lifts these to the unit's own maxima.
- **A vertical axis can fall** when the servo goes off, unless a brake holds it. Test it slowly and unloaded.
- **STOP is a decelerate-stop over Modbus.** It is VERIFIED on moves the PC commanded and UNVERIFIED on a move a PLC
  commanded over PIO. Stop those from the PLC or with the hardware stop.
- **Writes go straight through.** Commands that write the unit's non-volatile memory (position rows, teaching,
  parameters) write at once; a risky write (a home or dangerous parameter, a row the unit does not run) is written
  with a warning. A value the unit already holds is not written, and the writes are counted with a warning above 60
  an hour. Back the unit up first: `iai-actuator-driver --port COM3 backup unit1.json` (read-only).

## If nothing answers

- Check the 24 V supply, the cable, the slave address (axis number + 1) and that no other program has the port.
- Some products never answer, whatever the wiring: [Not supported](COMPATIBILITY.md#not-supported).
- More cases (changed baud rate, old firmware, vertical axes): [COMPATIBILITY.md](COMPATIBILITY.md#possible-trouble).

## Next

- [HARDWARE_SETUP.md](HARDWARE_SETUP.md): adapter, baud rates, several axes on one port.
- [DRIVER.md](DRIVER.md): every function with its inputs and ranges.
- [REFERENCE.md](REFERENCE.md): behaviour notes and the evidence behind them.
