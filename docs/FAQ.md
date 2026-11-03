# FAQ

Short answers to the first questions a controls engineer asks. When something fails, see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## PC control

### Why drive the actuator from a PC instead of a PLC?

Over PIO a PLC starts positions stored in the unit. Over Modbus RTU the PC sends any target inside the stroke,
with its own speed, acceleration and positioning band, and reads back position, alarm and status. No I/O wiring is
needed. Stored positions still work (`move_to_position(n)`, `goto N`; VERIFIED on the EC-GRBP8M gripper and the
PCON-CB). The safety circuit stays in hardware: the driver is not a safety function.

### Can a PLC and the PC share one axis?

Not at the same moment. While PC control is on (PMSS = 1) the unit ignores its PIO inputs.

- The driver takes PC control by itself, only at rest: switching it on mid-move stops the move (VERIFIED).
- It does not hand control back when it closes. Call `ax.disable_modbus_control()` or `mode pio` before the PLC
  takes over, or power-cycle the unit (PC control is off after every power-up, VERIFIED).
- Reading needs no control: `status`, `monitor` and `iai-check` write nothing; `connect(take_control=False)` reads
  without taking it.
- A Modbus STOP on a move the PLC started is UNVERIFIED: stop those from the PLC.

### How fast is a move cycle?

The motion takes what distance, speed and acceleration give. The driver adds settle time, status polls and a status
read around each move. The fast path cuts that overhead:

- `completion="fast"` ends a move on the first sample that is in position, inside the band and not moving.
- The status monitor (`ax.start_monitor()`) replaces the status read before each move.
- 230400 baud shortens every frame (VERIFIED on the EC-R6S rod, the EC-GRBP8M gripper and the PCON-CB; back at 38400
  after a power cycle).
- `ax.move_sequence(points, blend_band=...)` sends the next point while the axis is still within the band of the
  current one.

`ax.enable_fast()` turns on the first three. A fast move can return slightly short of the target while the slider
creeps in: up to 0.08 mm on the PCON-CB, inside the default 0.1 mm band (VERIFIED). Use `completion="safe"` when the
final position matters. Windows is not a real-time system: measure the cycle on your machine and leave margin. See
[DRIVER.md](DRIVER.md#fast-path).

### How many axes can one PC drive?

A `Cell` takes every axis it finds, on any number of COM ports run in parallel: there is no total limit, and
`max_axes` caps the cell if you give it. One axis per COM port, each on its own IAI USB adapter, is the VERIFIED layout
and runs fastest. Several axes can share a port by axis number (SIO daisy chain or RCON gateway): one cable carries at
most 16 axes (axis numbers 0..15, addresses 1..16), and the scan asks addresses 1..16 per port by default. Axes on one
cable take turns, and this is UNVERIFIED on hardware. `Cell.open()` finds every IAI
adapter and answering address by itself.

### Which units work?

Every IAI unit that speaks Modbus RTU on its serial port, driven the same way, with no driver update for a new model.

| | |
|---|---|
| Supported | every ELECYLINDER type (24 V and 200 V), CON-family controllers (PCON-CB, ACON-CB, DCON-CB, SCON-CB), RCON driver units behind the RCON gateway, ROBONET in SIO through mode |
| Run on hardware so far | EC-DS4H slider, EC-GRBP8M gripper, EC-R6S rod, EC-S6HH slider, PCON-CB with an RCP6 (a record, not a list of allowed units) |
| Not supported | SEL-protocol controllers (XSEL, SSEL, ASEL, PSEL, MSEL, TTA), an ELECYLINDER on an RCON-EC / REC connection unit, ROBONET in gateway mode |

Every range is read from the unit, so the driver keeps no model list. Results from other units are welcome: run
`iai-check` and open a unit report ([how](COMPATIBILITY.md#sending-a-report)). Details:
[COMPATIBILITY.md](COMPATIBILITY.md).

## Safety and failures

### Is the driver a safety function?

No. Keep a hardware emergency stop and guarding as for any machine. `stop` is a decelerate-stop sent over Modbus, not
an emergency stop. The limit checks keep wrong numbers away from the unit; they do not make the machine safe. A
vertical axis without a brake falls when its servo goes off. Test every new setup unloaded, mid-stroke and at 20 mm/s
or less.

### Can IA-OS stay open while I use the driver?

Not on the same port: only one program can own a COM port. Close IA-OS, or disconnect it from the port, first.
Afterwards the unit may run at 115200 baud (the driver finds the rate) or have safety speed on (`mode pc` switches it
off). It may also be in IA-OS's Monitor mode, where it takes no moves from another program (VERIFIED): set Teach mode
in IA-OS before you disconnect it, or power-cycle the unit.

### What happens when the unit loses power?

A move cut off by the power loss is not resumed. At the next power-up the unit is at 38400 baud, PC control and safety
speed are off, its uptime counter restarts and its alarm history gets a 0x0FFF power-up record. An incremental unit
has lost its home, and a PCON-CB raises the message alarm 0x069 (all VERIFIED). Stored rows and committed parameters
are kept; parameter values written without a commit are lost (VERIFIED).

The link watch reports `unit_restarted`. `ax.recover()` or `iai-actuator-driver --port COM3 recover` re-syncs the
axis without moving it (VERIFIED on the EC-GRBP8M gripper after a power cycle, and on the PCON-CB after a line break).
Home again when the report says `needs_home`.

### What happens when the PC, the program or the cable fails?

The unit has no communication watchdog (VERIFIED): it completes the move in progress, and a push keeps pushing. Design
the machine so that this is safe, and use the hardware emergency stop. When the link comes back, `recover()` reports
what changed, including a move still running; it neither stops nor resumes a move. In a `Cell`, a link fault on one
axis stops the other axes by default (`stop_on_fault=True`).

### Does writing positions or parameters wear the unit out?

The non-volatile memory lasts about 100,000 write cycles (IAI; the CB-series CON manuals state no limit, DOC
ME0342-4H p.2-11). Parameter commits, stored-row writes and teaching each use a cycle. Direct moves (`move_to`) are RAM
only (VERIFIED) and cost nothing, so a target that changes every cycle belongs in `move_to`.

The driver skips writes that would not change the value, counts writes per axis (`ax.nonvolatile_writes`) and logs
one WARNING per hour while more than 60 fall in the last hour on one axis. At 60 writes an hour, 100,000 cycles last
about 69 days. A write is never refused for the rate.

### Do I have to home after every power-up?

On an incremental unit (no `WA` option, such as the EC-R6S rod): yes, and after every software reset (VERIFIED).
Absolute units (`WA`) keep their home, except after a change of home direction or offset: the unit then raises alarm
0x0EE, and an alarm reset plus a home return clears it (VERIFIED on the EC-GRBP8M gripper). `status().homed` tells
you; a move on an axis that is not homed is refused.

## Setup and platforms

### Do I need to set anything on the unit first?

Not for one unit per port. Every tested unit answered at address 1 (axis number 0) and 38400 baud after power-up, and
the driver takes PC control by itself at rest. Close IA-OS, connect the IAI USB adapter to the unit's SIO / teaching
port and run `iai-actuator-driver scan`. Several units on one line each need their own axis number. Other RS-485
adapters are UNVERIFIED: name their port yourself (`--port`, `Cell.open(["COM5"])`).

### Can I try everything without hardware?

Yes: `iai-actuator-driver --sim ...`, `Axis.open(SimulatedEC())` or `Cell.open(sim_ports(3))` in Python, and
`iai-gui --sim-only`, which opens no serial port. The simulator answers Modbus RTU byte for byte with values copied
from the units run so far. Passing against it shows that your code agrees with what was seen on hardware.

### Does the Python package run on the C++ core or in pure Python?

On the core, always: a platform wheel carries its library, and without one the import raises `ImportError`. The
pure-Python fallback of 1.0.x was removed ([PYTHON_FALLBACK.md](PYTHON_FALLBACK.md)).

### Does it run on Linux?

Yes, from 1.1.0. The core is built for Linux x64 and arm64 too. Its serial port code is VERIFIED on a real port: a
PCON-CB through the IAI USB adapter (`cp210x`, `/dev/ttyUSB0`) in WSL 2 with usbipd, with a build of the 1.1.0 fix
(2026-11-03). The 1.0.1 Linux files fail at the first read after a request ("the port vanished (unplugged?)"): use
1.1.0. Linux on arm64 and macOS serial ports are UNVERIFIED. Name the port as Linux shows it, such as `/dev/ttyUSB0`;
[HARDWARE_SETUP.md](HARDWARE_SETUP.md#linux) has the setup.

### Is there a C#, C++ or C version?

Yes, in this repository under one version. The driver is implemented once in C++ (`core/`) behind a C ABI
(`iai_actuator_driver.dll`, `libiai_actuator_driver.so`). C# uses it through the .NET library in `bindings/dotnet/`
(with the `iai-actuator-driver-cs` command line), C++ programs link the core, and C or LabVIEW call the ABI directly.
Each language has a guide in [getting-started/](getting-started/);
[DRIVER.md](DRIVER.md#names-in-the-other-editions) lists the same calls in each language. These documents describe
the Python package: VERIFIED here means tried on hardware with it. Which editions ran on which unit is in
[COMPATIBILITY.md](COMPATIBILITY.md).

### Is the driver supported by IAI?

No. It is unofficial and not an IAI product ([README](../README.md)). When something does not work, see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md), then open a bug report on
[GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose) with the `iai-check` report.

### What do VERIFIED, ON-WIRE, DOC and UNVERIFIED mean?

| Word | Meaning |
|---|---|
| **VERIFIED** | Tried on real hardware with this driver |
| **ON-WIRE** | Seen in observed serial traffic; the unit's reaction is VERIFIED only where also tested |
| **DOC** | Written in a public IAI manual; not yet confirmed on the unit |
| **UNVERIFIED** | Expected, not yet tried. The words describe features and paths, never a unit or a model: every unit is driven the same way |
