# Hardware setup

How to connect a PC to an IAI unit: the adapter, the port, power and emergency stop, baud rates and several axes on
one port, with a checklist for the first connection. For the first run in software see
[GETTING_STARTED.md](GETTING_STARTED.md); for the supported units see [COMPATIBILITY.md](COMPATIBILITY.md).

Unofficial: not an IAI product, not supported by IAI. The evidence words (VERIFIED, ON-WIRE, DOC, UNVERIFIED) are
defined in [REFERENCE.md](REFERENCE.md#evidence-levels).

## What you need

| Item | Notes |
|---|---|
| PC | Python 3.11 or newer with the driver installed ([Install](GETTING_STARTED.md#1-install)); pyserial comes with it. Windows x64 or Linux x64 ([Linux](#linux)); macOS is UNVERIFIED. A C#, C++ or C program needs no Python ([getting-started/](getting-started/)). |
| Unit | An ELECYLINDER, or a CON-family controller (PCON, ACON, DCON, SCON) with its actuator: every IAI unit that speaks Modbus RTU on its serial port is driven the same way. Units run so far: [COMPATIBILITY.md](COMPATIBILITY.md#run-on-hardware-so-far). |
| Adapter | The IAI USB adapter with its cable to the unit's SIO port (the RS-485 teaching port). It is a Silicon Labs CP210x bridge, USB ID 10C4:81D7, listed by Windows as "IAI USB to UART Bridge Controller" (VERIFIED). |
| Power | The unit's own supply: 24 V on the ELECYLINDERs tested. |
| Emergency stop | A hardware emergency stop or drive cut-off, wired as the unit's IAI manual describes. It is required, as for any PC-commanded axis (EN ISO 13849-1): the driver is not a safety function. |

- The normal setup is one unit per adapter: one axis per COM port, every port in parallel. Each tested unit ran
  alone on its own adapter (VERIFIED). Several units can also share one port: see
  [Several axes on one port](#several-axes-on-one-port).
- A third-party USB to RS-485 adapter has not been tried (UNVERIFIED). `iai-actuator-driver ports` marks only the IAI
  adapter (and ports whose maker string starts with "IAI") as IAI, and `scan` without `--ports` asks only those. Name
  any other adapter's port with `--port` or `--ports`.

## Find the port

### Windows

Plug the adapter in. It appears as a COM port under "Ports (COM & LPT)" in Device Manager, for example "IAI USB to
UART Bridge Controller (COM3)" (VERIFIED). The driver lists the ports without opening any:

```text
iai-actuator-driver ports
PORT       USB ID     IAI  DESCRIPTION
COM3       10C4:81D7  yes  IAI USB to UART Bridge Controller (COM3)
```

Pulling the adapter out makes its COM port disappear (VERIFIED). Do not do it while a move is running: the unit has
no communication watchdog, so the move goes on (see [Power and emergency stop](#power-and-emergency-stop)).

### Linux

The IAI USB adapter appears as `/dev/ttyUSB0` through the kernel's `cp210x` driver (Linux serial port: VERIFIED,
[COMPATIBILITY.md](COMPATIBILITY.md#which-editions-were-run-on-a-unit)). The next adapter is expected as
`/dev/ttyUSB1`, and so on. `iai-actuator-driver ports` lists it with its USB ID. Pass the device name to the commands,
for example `iai-actuator-driver --port /dev/ttyUSB0 status`. Your user needs read and write access to the device;
on Debian and Ubuntu that means membership of the `dialout` group (`sudo usermod -aG dialout $USER`, then log in
again).

In WSL 2 (VERIFIED with Ubuntu 24.04 and usbipd-win 5.3), pass the adapter in with usbipd from an administrator
PowerShell; it is then `/dev/ttyUSB0`, and Windows programs cannot use it until `usbipd detach --busid <BUSID>`:

```text
usbipd list                          # find the adapter's BUSID (USB ID 10C4:81D7)
usbipd bind --busid <BUSID>          # once; add --force where a USB filter such as USBPcap is installed
usbipd attach --wsl --busid <BUSID>  # again after every replug and every WSL restart; keep a WSL shell open
```

## Close IA-OS and other masters

Only one program can own a serial port, and only one master can drive the unit.

- **Close IA-OS** (IAI's PC software), or disconnect it from the unit, before the driver opens the port. Windows
  reports "access denied" for a port that another program holds (VERIFIED). The driver then names the usual holders:
  IA-OS, another IAI Actuator Driver or iai-gui, a terminal program.
- A unit that IA-OS has put in its Monitor mode does not accept moves from another program (VERIFIED). A teaching
  pendant or a wireless session is expected to take control the same way (UNVERIFIED). Disconnect them, then connect
  again.
- After IA-OS, the unit may still run at 115200 baud (IA-OS switches units to it, ON-WIRE). The driver finds the rate
  either way.
- IA-OS's Teach mode can leave the safety-speed mode on, which caps motion at about 100 mm/s (VERIFIED). It is off
  after every power-up (VERIFIED). `iai-check` reports it as WARN; `ax.set_safety_speed(False)` turns it off.
- Every power-up gives control to PIO (PMSS = 0, VERIFIED). The driver takes Modbus control before its first command,
  and only at rest: switching it on mid-move stopped the move (VERIFIED). While the PC holds control the unit's PIO
  inputs are disabled (DOC). Give control back before a PLC takes over: `iai-actuator-driver --port COM3 mode pio`,
  or `ax.disable_modbus_control()`.

## Power and emergency stop

The driver is not a safety function. Before the first move:

- **Wire and test the emergency stop.** The unit has no communication watchdog (VERIFIED). If the PC crashes or hangs,
  or the cable is pulled, a move in progress is completed and a push keeps pushing. Only the hardware emergency stop
  stops the axis independently of the PC.
- **Check the status.** `iai-actuator-driver --port COM3 status` (read-only) prints the condition flags that are set,
  among them `EMG-INPUT`, `DRIVE-POWER-CUT` and `ENABLE-SWITCH-OFF` (DOC; their meaning on an EC is UNVERIFIED). A
  servo ON that the status shows cannot work is refused before anything is sent, with the reasons
  (`ServoBlockedError`).
- **The rest** (start slow, the command line's caps, vertical axes, homing): [Safety](GETTING_STARTED.md#safety).

**What a power cycle resets.**

| Back at the power-up state (RAM only, VERIFIED) | Kept (non-volatile, VERIFIED) |
|---|---|
| Baud rate, PC control (PMSS), safety speed, the action mode, a parameter written with `param set --no-commit` | Committed parameters, stored position rows |

How often the non-volatile memory may be written: [Non-volatile writes](TROUBLESHOOTING.md#non-volatile-writes).

## Baud rate

| Fact | Evidence |
|---|---|
| Every tested unit answered at 38400 baud after power-up. Its power-up rate is its parameter BRSL, item 8.3 (`ax.read_boot_baudrate()`). | VERIFIED |
| IA-OS switches units to 115200. | ON-WIRE |
| The driver changes the rate with one broadcast frame. The codes for 115200 and 230400 worked on the EC-R6S rod and on the PCON-CB. | VERIFIED |
| A changed rate is volatile: the unit is back at 38400 after every power-up. | VERIFIED |
| The 38400 code (0x06) was ignored by the EC-R6S rod and taken by the EC-GRBP8M gripper and the PCON-CB. | VERIFIED |
| A line break of 150 ms or more drops a controller to 9600. After a line break the EC-GRBP8M gripper and the PCON-CB answered at 9600. | DOC; VERIFIED |

**How the driver finds the rate.**

- It opens the port once (8 data bits, no parity, 1 stop bit) and changes the PC side's rate in place. It never
  closes and reopens the port to try another rate.
- It tries 115200, then 38400, 9600, 230400, 57600 and 19200, and prints the result on stderr:
  `note: COM3 at 38400 baud (auto-detected)`.
- `--baud N` fixes the rate and skips the search.
- 14400, 28800 and 76800 are never probed. After one of those, reconnect with `--baud RATE` or power-cycle the unit
  back to 38400.
- The driver sends a line break only in `set_baudrate(method="break")`.

**Raising the rate.** Only at rest. Every unit on the line switches, because the frame is a broadcast:

```shell
iai-actuator-driver --port COM3 baud 230400  # 115200 or 230400; 38400 is checked after the switch
```

In Python, `ax.set_baudrate(230400)`, or `ax.enable_fast()`, which also turns on fast completion and the status
monitor. The next connection finds the new rate by itself. To get back to 38400, power-cycle the unit.

## Several axes on one port

Several units can share one port by axis number: address (slave) = axis number + 1. The scan asks addresses 1 to 16.
Every shared-line setup below is UNVERIFIED on hardware.

```shell
iai-actuator-driver scan --ports COM3             # every address 1-16 on COM3 (read-only)
iai-actuator-driver --port COM3 --axis 2 status   # axis 2, the same as --slave 3
```

In Python a `Port` opens the line once and a `Cell` drives every axis on it ([DRIVER.md](DRIVER.md)). The axes of one
port take turns on it, and their moves still run at the same time.

**SIO daisy chain.**

- Each unit needs its own axis number. The driver reads it (`identify` prints `axis no.`) but never changes it: set it
  with IAI's tools before connecting.
- A baud change is a broadcast, so every unit on the line switches.
- `iai-actuator-driver stop-all --ports COM3 --broadcast` sends one broadcast STOP before the per-axis STOP.

**RCON gateway (DOC, UNVERIFIED).**

- One gateway port carries up to 16 driver units at axis number + 1; the gateway itself answers at address D0h. The
  driver finds it when the port opens (two read-only probes). `iai-actuator-driver --port COM3 line` shows the line
  and its rules.
- Set the gateway's AUTO/MANU switch to MANU: Modbus control works only in MANU (`GatewayModeError` otherwise).
- State the units' position unit, parameter No.207 (0.01 or 0.001 mm), which cannot be read over Modbus:
  `--position-unit 0.01` (in Python `Port(..., position_unit_mm=0.01)`). Moves in mm are refused until then
  (`PositionUnitError`).
- The line carries no coils, no IAI 0x45 / 0x48 commands and no broadcast. The driver uses the register path (STOP
  without a coil) and cannot change the baud rate there.
- The scan asks only the axes the gateway reports as linked, and reports a linked axis that does not answer.
- An ELECYLINDER on an RCON-EC or REC connection unit has its serial port disabled: not supported.

**ROBONET (DOC, UNVERIFIED).**

- Set SW2 ON on the ROBONET SIO gateway (RGW-SIO): SIO through mode. The axes then answer at axis number + 1.
- Through mode cannot be detected: state it with `--line robonet` (`line="robonet"` in Python).
- Through mode carries no IAI 0x45 / 0x48 commands and no broadcast, and the PC side runs at 38400 to 230400 baud, so
  the baud search skips the lower rates.
- SW2 OFF (gateway mode) gives no access to the axes: the driver raises `UnsupportedLineError` and says to set SW2 ON.

## First connection checklist

1. The unit is powered; the emergency stop is wired, tested and released; the axis is unloaded, its travel clear.
2. IA-OS, every other program on the port, teaching pendants and wireless sessions are closed or disconnected.
3. `iai-actuator-driver ports` lists the adapter with `IAI yes`.
4. `iai-actuator-driver scan` finds the unit.
5. `iai-check --port COM3` shows no FAIL ([Check your unit](GETTING_STARTED.md#5-check-your-unit)).
6. `iai-actuator-driver --port COM3 limits` shows the ranges the driver will enforce.
7. `iai-actuator-driver --port COM3 backup unit1.json` saves the unit (read-only).
8. Home if `status` shows the axis not homed, then `iai-check --port COM3 --motion` (at most 5 mm and a quarter of the
   stroke, at 10 mm/s and back) or the [first move](GETTING_STARTED.md#6-first-move), a hand on the emergency stop.
9. A model not in [COMPATIBILITY.md](COMPATIBILITY.md): send the report ([how](COMPATIBILITY.md#sending-a-report)).

Symptoms and messages: [TROUBLESHOOTING.md](TROUBLESHOOTING.md). `--log-frames` prints every Modbus frame to stderr.
