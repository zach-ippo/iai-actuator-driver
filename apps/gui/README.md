# IAI Actuator Driver: local web GUI

`iai-gui` is a local web GUI: a standard-library HTTP server on 127.0.0.1 with the multi-axis workbench at `/` and
the single-axis page at `/classic`. Unofficial: not an IAI product. More in the
[README](https://github.com/zach-ippo/iai-actuator-driver/blob/main/README.md), [docs/GETTING_STARTED.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/GETTING_STARTED.md) and
[docs/REFERENCE.md](https://github.com/zach-ippo/iai-actuator-driver/blob/main/docs/REFERENCE.md).

Licence: Apache License 2.0 ([LICENSE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/LICENSE), [NOTICE](https://github.com/zach-ippo/iai-actuator-driver/blob/main/licenses/NOTICE)).

## Start it

Each way opens the browser at http://127.0.0.1:8780 and prints the address in the console. The address ends with
`#k=` and this run's access key. The key is never in a page, so open the whole address, and do not share it. Add
`--sim-only` for a demo with simulators only, no serial port.

- **Release (Windows, no Python):** run `iai-gui.exe` from the release zip. If Windows warns about an unknown
  publisher, choose More info, then Run anyway. Closing its console window also stops the axes and leaves each unit
  as found.
- **Launcher:** in this folder, double-click `run-gui.bat` (Windows) or run `./run-gui.sh` (Linux, macOS; serial ports
  on Linux need the dialout group). The first run creates `.venv` here (needs internet) and installs the driver and
  the GUI into it. Arguments go to `iai-gui`.
- **Installed:** `pip install ./bindings/python ./apps/gui` in the repository root, then `iai-gui`.
- **Source checkout:** in the repository root:

```shell
pip install -e bindings/python                # the driver and pyserial
python apps/gui/server.py --sim-only          # http://127.0.0.1:8780, simulators only, no serial port
```

`python apps/gui/server.py` also runs without installing the driver: it then uses `bindings/python` of the same
checkout (pyserial must be installed). `pip install -e bindings/python -e apps/gui` adds the `iai-gui` command.

Quit with the **Quit** button in the page or Ctrl+C.

## A first run

- **No hardware:** press **Add two simulated axes** (two sliders on two ports, enough for the Group and Coordinated
  tabs). Simulated axes run the same driver code and never open a serial port.
- **With a unit:** close IA-OS first (only one program can use a port), tick the port marked IAI and press **Connect
  selected ports**. The baud rate is detected.

Start slow and short and keep a hand on the emergency stop. Back up a unit (Functions > Back up this unit) before
writing parameters or stored positions. A permanent write (a parameter, a stored row, teaching) asks one plain
OK / Cancel first; nothing needs switching on, and a risky write shows its warning in the answer. Things to try:

1. Axis tab: Servo ON, Home, then a short slow move.
2. During a move press Space or Esc: STOP ALL. It also cancels a command still waiting to be sent (the answer says it
   was cancelled by STOP) and stops axes still being added by a connect or scan.
3. Stored positions: run a row. Unit info & alarms: alarm history and Self-test.
4. Group tab: move two axes together.
5. Coordinated tab (experimental): plan a small circle, check the plot, then Run. A plan runs only from where it was
   made: if an axis moved after Plan, Run refuses and you plan again.
6. **PC / PIO** switch (Axis tab, under the lamps; on the single-axis page in Controls): see below.
7. **Test report**: Copy, then paste it into a
   [GitHub issue](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose) with the steps, what you
   expected and what happened.

## PC / PIO control

The two-button switch shows who has command authority (PMSS): **PC** (Modbus: the unit takes its commands from the
page) or **PIO** (the unit follows its PIO inputs, for example a PLC). Pressing the other button asks first.

- In PIO the unit runs its stored position rows at their own stored speeds and accelerations, not the limits set on
  the page. A Modbus STOP is untested on a move a PLC commanded over PIO: stop those from the PLC or with the hardware
  stop. Pressing PC takes control back.
- The mode switches only at rest. While the axis moves, homes, pushes or is paused, or when its motion state cannot be
  read, the buttons are disabled and a request is refused (PMSL ON during a move stops it). Press STOP first. A pause at
  a standstill with the servo off does not block PC; a pause with the servo on does (a PLC holding a stop input on a
  move). A PCON in PIO mode with nothing on its PIO connector pauses and switches its servo off (VERIFIED on the
  PCON-CB); the driver taking PC control there: VERIFIED (PCON-CB), UNVERIFIED from this page.
- With PIO chosen here, the page sends no motion command: they are refused with the reason until PC is chosen (the
  workbench also refuses group moves and a coordinated run). STOP always works.
- PIO found at connect, as after every power-up, is not a choice: the first command takes PC control at rest.
- Disconnect leaves a unit as found: PIO that had control at connect gets it back; PIO chosen here stays.
- API: `POST /api/control {"mode": "pc" | "pio"}` (single-axis page); the workbench calls its
  `enable_modbus_control` / `disable_modbus_control` functions. The single-axis switch is VERIFIED on the EC-GRBP8M
  gripper (PC to PIO to PC; a move sent in PIO was refused); the workbench's switch is UNVERIFIED on hardware (tested
  on the simulator).

## Known limits

- Unofficial, not an IAI product. STOP is a Modbus decelerate-stop, not a safety function.
- A unit has no communication watchdog: a move in progress completes if the PC or the cable fails.
- Coordinated motion is experimental: one real axis with a simulated second VERIFIED through the Python package
  (EC-GRBP8M gripper, PCON-CB), not through this GUI; two real axes together UNVERIFIED. The Coordinated tab shows the
  measured trace when the run ends, not while it runs.
- Several axes on one port: UNVERIFIED.
- The GUI listens on 127.0.0.1 only.
- Backups of an installed package go to your own data folder (`%LOCALAPPDATA%/IAI Actuator Driver/backups` on
  Windows), not to the folder the GUI was started in. The Back up result shows the path.

## Build the exe

Windows, Python 3.11 or newer with pyserial and PyInstaller. The exe carries the driver's native library, so the build
needs it: the installed win-x64 wheel of `bindings/python`, or, from a source checkout, a core build
(`python core/build.py` writes `core/build/`, or `IAD_BUILD_DIR`; point `IAI_ACTUATOR_DRIVER_NATIVE` at the library or
its folder). Without a library the build stops with a message.

```text
set IAI_ACTUATOR_DRIVER_NATIVE=<folder with iai_actuator_driver.dll>       (not needed with the wheel installed)
pyinstaller --noconfirm --distpath dist --workpath build apps\gui\iai-gui.spec
dist\iai-gui.exe --version
dist\iai-gui.exe --sim-only
```

- `--sim-only` is the smoke test: add two simulated axes, run a move, the PC / PIO switch and a coordinated plan.
- A release builds the exe the same way over the wheel and checks that the library is inside
  (`.github/release/local_release.py`, step `gui`).
- `packaging/README.txt` is the text that goes into the release zip. `dist/` and `build/` are not committed.

## Tests

In the repository root, with the driver installed (`pip install -e bindings/python`):

```shell
python -m unittest discover -s apps/gui -t apps -q
```

They need the core's library, as the driver does. Two tests time how fast STOP reaches the bus during a read; they
allow more time when the environment variable `CI` is set.
