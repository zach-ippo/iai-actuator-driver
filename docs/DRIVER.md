# IAI Actuator Driver: function reference

One line per function of the Python package. Every function works the same on any IAI unit it finds: an ELECYLINDER
or a CON-family controller such as the PCON, any actuator kind.

- **Units.** Positions in **mm** (**deg** on a rotary axis), speeds in units/s, accelerations in **G**. Ranges are
  read from the unit; `ax.limits()` shows them all.
- **Other languages.** C#, C++ and C run the same C++ core: the names differ, the ranges, units, refusals and warnings
  do not. See [Names in the other editions](#names-in-the-other-editions).
- **Native library.** The package runs on the core and needs its library
  ([getting-started/python.md](getting-started/python.md#where-the-native-library-comes-from)).
- **Evidence words.** VERIFIED, ON-WIRE, DOC and UNVERIFIED describe features and paths, never a unit
  ([Evidence levels](REFERENCE.md#evidence-levels)).

Unofficial: not an IAI product. See the [README](../README.md).

## Quick start

Install the release wheel (Python 3.11+; adds `iai-actuator-driver` and `iai-check`, the core's library inside):
[GETTING_STARTED.md](GETTING_STARTED.md#1-install). From a checkout, `pip install ./bindings/python ./apps/gui` also
adds `iai-gui` and needs the core's library built or named
([where it comes from](getting-started/python.md#where-the-native-library-comes-from)).

Close IA-OS first: only one program can use a serial port.

```python
from iai_actuator_driver import Axis, Cell

# One axis
with Axis.open("COM3") as ax:          # baud rate found automatically
    ax.connect()                       # read identity and ranges, take control
    ax.servo(True)
    ax.home()
    ax.move_to(50.0, speed_mm_s=20)    # waits until in position
    ax.stop()                          # decel stop (use it any time)
    ax.servo(False)

# Many axes (normally one per COM port; found automatically)
with Cell.open() as cell:              # every IAI USB adapter, every answering axis
    print(cell.names)                  # ("COM3:1", "COM4:1", ...) = port:address
    cell.servo_all(True)
    cell.home_all()
    cell.move_all({"COM3:1": 50.0, "COM4:1": 20.0}, speed_mm_s=30)
    cell.stop_all()
```

No hardware? Use the simulator: `Axis.open(SimulatedEC())` or `Cell.open(sim_ports(3))`.

`Axis` and `Elecylinder` are the same class. Below, `ax` is one axis and `cell` a group of axes.
Address (slave) = axis number + 1.

## Connect & discover

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `list_ports()` | Lists the PC's serial ports, IAI adapters first. Opens nothing. | none | `list_ports()` |
| `scan(port, slaves=1..16)` | Finds the addresses that answer on one port. Read-only. | port name; addresses 1..16 | `scan("COM3")` |
| `Axis.open(port, *, baudrate=None, auto_baud=None, timeout=0.2, retries=3, frame_logger=None, line="auto", position_unit_mm=None, fast=False, **axis_options)` | Opens a port (a name such as `"COM3"`, or a transport such as `SimulatedEC()`) and returns one axis that owns it. Detects the line once (two read-only probes). `fast=True` then runs `enable_fast()` (see [Fast path](#fast-path)). | baud: found in place when `None` (115200 first, then 38400, 9600, 230400, 57600, 19200); a given `baudrate` is used as it is unless `auto_baud=True`; `timeout`: seconds per request; `retries` 3 = the manufacturer's recommendation; `frame_logger`: called with every frame; `line`: `"auto"`, `"direct"`, `"rcon"`, `"robonet"`; `position_unit_mm`: 0.01 or 0.001 (RCON only); `axis_options`: `slave=1`, `stroke=(min, max)`, `max_speed_mm_s`, `max_accel_g`, `max_push_pct` (limits that can only narrow the unit's own), `strategy="coil"`, `auto_modbus_control=True`, `settle_s=0.1`, `push_settle_s=0.15`, `poll_interval_s=0.02`, `completion="safe"` | `ax = Axis.open("COM3")` |
| `ax.connect(*, take_control=True, fast=False)` | Reads identity and ranges, takes Modbus control. Returns `DeviceInfo` (also kept as `ax.device_info`). `fast=True` then runs `enable_fast()`. | none | `info = ax.connect()` |
| `ax.close()` | Closes the port (also done by `with`). | none | `ax.close()` |
| `ax.identify()` | Reads identity: family, model, serials, firmware, axis number, soft limits. Returns `DeviceInfo`. | none | `ax.identify().model` |
| `ax.unit_info(*, sections=None)` | Everything the unit reports about itself, as one dict. Read-only. | optional list of sections | `ax.unit_info()` |
| `ax.limits()` | Every range the driver enforces, read from the unit, with its source. | none | `ax.limits()["speed"]["max"]` |
| `ax.units` / `ax.kind` / `ax.product_family` / `ax.is_ec` | Unit (`"mm"`/`"deg"`), actuator kind (slider, rod, gripper, rotary...), family (`"EC"`, `"PCON"`...), EC or not. | properties | `ax.units` |
| `Port(port, *, name=None, baudrate=None, timeout=0.2, retries=3, frame_logger=None, line="auto", position_unit_mm=None)` | Opens one port shared by several axes (gateway or daisy chain). | port name or transport; `name`: the port's name in a `Cell`; the rest as in `Axis.open` | `p = Port("COM4")` |
| `Port.scan(slaves=1..16)` | Finds the addresses that answer on this port. Read-only. | addresses 1..16 | `p.scan()` |
| `Port.axis(slave=1)` | The axis at that address on the shared port. | address 1..16 | `ax2 = p.axis(3)` |
| `Cell.open(ports=None, slaves="scan", *, max_axes=None, baudrate=None, timeout=0.2, retries=3, scan_timeout=0.05, scan_slaves=1..16, identify=True, frame_logger=None, line="auto", position_unit_mm=None, on_fault=None, stop_on_fault=True, on_event=None, **axis_options)` | Opens ports, finds and identifies every axis. Read-only. | `ports`: `None` (every IAI adapter), a name, a list, or `{name: port}`; `slaves`: `"scan"`, a list, or `{port: [addresses]}`; `max_axes` `None` (no limit; an int caps the cell); `line` and `position_unit_mm`: one value or `{port: value}`; `axis_options` as in `Axis.open`, for every axis | `cell = Cell.open(["COM3", "COM4"])` |
| `cell[key]` / `cell.names` / `cell.name_of(axis)` / `cell.port_of(key)` | One axis by name, index or object / all axis names / the name of an axis object / the `Port` an axis is on. | `"COM3:1"`, `0`, ... | `cell["COM3:1"].status()` |

## Status & monitoring

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.status()` | Current state: `position_mm`, `velocity_mm_s`, `servo_on`, `homed`, `in_position`, `moving`, `alarm`, `alarm_code`, `warning`, `emergency_stop`, `ready`. One read of 22 words (0x9000-0x9015). | none | `ax.status().position_mm` |
| `ax.status().emg_input` / `.drive_power_cut` / `.enable_off` / `.cold_start_alarm` / `.auto_servo_off` / `.nv_busy` / `.manual_mode` / `.battery_low` / `.rtc_in_use` / `.excitation_done` / `.position_complete_no` | Condition flags: EMG input active, motor drive power cut, teaching-tool enable switch released, cold-start alarm, automatic servo OFF, non-volatile write busy, the unit's MANU / AUTO bit (DSSE RMDS; `False` = AUTO on a unit with a mode switch), battery low, calendar in use, excitation detection done, the stored row last completed. `manual_mode` is not PC vs PIO control: that is `modbus_enabled` (PMSS); on the EC-GRBP8M gripper `manual_mode` stayed `True` in PIO mode (VERIFIED). A flag is `None` when the unit does not define the bit (RCON) or the word was not read. DOC; the meanings on an EC are UNVERIFIED. | properties | `ax.status().emg_input` |
| `ax.status().at_backward_end` / `.at_forward_end` / `.at_middle` / `.idle` / `.push_contact` / `.push_missed` / `.target_reached` / `.vnow_mm_s` / `.move_bit` | Within the LS range of row 0 / row 1 / row 2 (ZONS bits 12-14; `None` when ZONS was not read) / not moving, not paused, not homing (safe for relative decisions and non-volatile writes) / the contact fingerprint of one sample (PEND and DSSE bit 0; one sample is no proof, `push()` waits for it to hold) / DSS1 PSFL: the last push went over its whole band without contact / DSS1 bit 0: the last move ended at its target (0 after a STOP) / the velocity in mm/s (coarse, about 12.5 mm/s steps; needs the 22-word read) / the raw DSSE MOVE bit (clears early while decelerating: prefer `moving`). | properties | `ax.status().idle` |
| `ax.read_monitor()` | Position, velocity, motor current, deviation, alarm, uptime, as a dict. | none | `ax.read_monitor()["current"]` |
| `ax.wait_idle(target_mm=None, band_mm=None, timeout=10.0, *, completion=None, since=None)` | Blocks until the axis is in position and still (`completion="fast"`: in position and in the band on one sample). | optional target and band; `completion` `"safe"` / `"fast"`; `since`: a `time.perf_counter()` value, only monitor samples read after it count (default: this call) | `ax.wait_idle(50.0, 0.1)` |
| `ax.read_registers(address, count)` | Raw register read (diagnostics). | any readable address | `ax.read_registers(0x9000, 4)` |
| `ax.read_lean_status(*, retries=None)` | The lean status: one read of 8 words (0x9000-0x9007: position, alarm, DSS1, DSS2, DSSE). Flags from the other words are `None`. Read-only. | none | `ax.read_lean_status().position_mm` |
| `ax.start_monitor(*, gap_s=0.002, on_change=None, on_done=None, on_event=None, link_check_s=1.0)` / `ax.stop_monitor()` / `ax.monitor` | Starts / stops the background status monitor (`AxisMonitor`): lean status reads back to back, the port lock free between them. `on_change(status, previous)`, `on_done(status)` and `on_event(event)` (link events) run in its thread. Read-only. | `gap_s` 0..10 s; `link_check_s` 0..3600 s or `None` | `mon = ax.start_monitor()` |
| `mon.latest` / `mon.status` / `mon.fresh(max_age_s=0.05)` | The last sample (status, sequence number, times) / its status / the status only if younger than `max_age_s`. | seconds | `mon.fresh()` |
| `mon.age_s` / `mon.timestamp` / `mon.poll_rate_hz` / `mon.is_done` / `mon.suspended` | Seconds since the last sample / `time.perf_counter()` when it was read / answered polls per second since the last start / the fast rule against the axis's last target and band / polls are held (inside `suspend()` or `monitor.suspend_bus`). | properties | `mon.poll_rate_hz` |
| `mon.wait_for(predicate, timeout=None)` / `mon.wait_update(timeout=None)` / `mon.wait_done(timeout=None)` | Block until a sample matches / the next sample / the axis is done by the fast rule. `None` on timeout; raises when the monitor ended on an error (port gone). | seconds | `mon.wait_for(lambda st: st.position_mm > 40)` |
| `mon.suspend()` / `mon.stop()` / `mon.link_state` | No polls inside the `with` block / end the thread / the axis's link state. A baud change, a software reset and `ax.recover()` hold every monitor of the port (`monitor.suspend_bus(bus)`; `axes=` holds only those axes' monitors, as a coordinated run does). | none | `with mon.suspend(): ...` |
| `monitor.fast_done(status, target_mm=None, band_mm=None)` | The fast completion rule on one status: in position, inside the band, not moving. No I/O. | mm | `fast_done(st, 50.0, 0.1)` |
| `monitor.timer_resolution(ms=1)` | Context manager: the Windows timer resolution at 1 ms inside the block (the default is about 15.6 ms, VERIFIED on the test PC); no effect elsewhere. Used by the waits and the monitor. | ms | `with timer_resolution(): ...` |

## Motion

Every motion input is checked against the unit's own limits before anything is sent; out of range raises
`SoftLimitError`.

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.servo(on, *, wait=True, timeout=None)` | Servo on or off. A servo ON that the status shows cannot work (EMG input, drive power cut, enable switch released) is refused with `ServoBlockedError` before anything is sent; its `.reasons` list says why, also after a timeout. | `True`/`False`; `timeout`: 3 s, or on a CON unit before its first servo ON the time its parameters give | `ax.servo(True)` |
| `ax.home(*, wait=True, timeout=None, method=None)` | Home return. | `method`: `None`, `"coil"` or `"iai"` | `ax.home()` |
| `ax.move_to(mm, speed_mm_s=50, accel_g=0.1, band_mm=0.1, wait=True, timeout=None, *, decel_g=None, accel_pattern=None, completion=None)` | Absolute move. A `decel_g` different from `accel_g` runs the command-profile move (separate deceleration; VERIFIED on the EC gripper and the PCON-CB). `accel_pattern`: `"trapezoid"`, `"s_motion"`, `"filter"`; all three run on the EC gripper (curve shape not measured); refused when the unit's support word lacks it. A speed below the documented minimum only warns. | position: unit soft limits; speed: 0.01 .. unit max; accel/decel: 0.01 .. unit max (all from `ax.limits()`) | `ax.move_to(80.0, speed_mm_s=20)` |
| `ax.change_speed(speed_mm_s, *, wait=False, timeout=None)` / `ax.change_accel(accel_g, *, wait=False, timeout=None)` | Changes the speed / acceleration of the move in progress by re-sending the full 0x9900 block with the same target. Refused (nothing sent) for a push, stored-row or profile move, while paused, or with no move running. VERIFIED on the EC gripper and, for the speed, on the PCON-CB. | speed 0.01 .. unit max; accel 0.01 .. unit max | `ax.change_speed(60.0)` |
| `ax.move_by(delta_mm, speed_mm_s=50, accel_g=0.1, band_mm=0.1, wait=True, timeout=None, *, accel_pattern=None, completion=None)` | Relative move from the measured position; right after a move that ended by the fast rule (or was not waited for) and is still within its band, from that move's target, so fast steps add up (ON-WIRE on a PCON-CB + RCP6). | as `move_to` | `ax.move_by(-10.0)` |
| `ax.jog(delta_mm, speed_mm_s=10, accel_g=0.1, band_mm=0.1, wait=True, timeout=None, *, completion=None)` | Small relative step at low speed. | step: 0.01 .. stroke | `ax.jog(-5.0)` |
| `ax.push(target_mm, band_mm, force_pct, direction=1, speed_mm_s=10, accel_g=0.1, *, wait=True, timeout=None)` | Moves to `target_mm`, then pushes up to `band_mm` further. Returns `PushResult` (`outcome`: `"contact"` or `"missed"`). | force: 1 .. unit max % (`ax.limits()["push"]`); direction +1 away from home, -1 toward | `ax.push(85.0, band_mm=10.0, force_pct=30)` |
| `ax.check_move(mm, speed_mm_s=50, accel_g=0.1, band_mm=0.1, *, decel_g=None, accel_pattern=None)` | Runs every check of `move_to` without moving. | as `move_to` | `ax.check_move(120.0, speed_mm_s=100)` |
| `ax.stop(*, wait=False, timeout=5.0)` | Decel stop. Works with or without Modbus control. Behind an RCON gateway (no coils) it takes the register path: pause, wait for standstill, release. | none | `ax.stop()` |
| `ax.pause()` / `ax.resume()` | Holds a move and keeps the remaining travel / continues it. VERIFIED. | none | `ax.pause()` |
| `ax.cancel_paused_move(*, timeout=5.0)` | Drops the remaining travel of a paused move and releases the pause; the axis stays where it stopped. Refused when nothing is paused or a fault alarm is active. VERIFIED on the EC gripper. | `timeout`: seconds to stand still | `ax.cancel_paused_move()` |
| `ax.push_speed_mm_s(speed=None)` / `ax.recommended_min_speed()` | The speed a push really runs at (the lowest of the commanded speed, the unit's push speed and its push-speed limit) / the documented minimum speed with its source. Read-only. | none | `ax.push_speed_mm_s()` |
| `ax.enable_modbus_control(verify=True)` / `ax.disable_modbus_control()` | Gives command authority to the PC / back to PIO. Done automatically at rest. | none | `ax.disable_modbus_control()` |
| `ax.set_action_mode(pc=True, safety=None, *, verify=True)` | Sets PC/PIO mode and safety speed and reads the status back (`verify`). PC: the PMSL coil (when PMSS is 0) and the SFTY coil (when it changes); PIO: 0x45 cmd 0x0C. | `pc`, `safety`: `True`/`False` (`None` keeps the unit's own) | `ax.set_action_mode(True, safety=False)` |
| `ax.set_safety_speed(on, *, verify=True)` | Safety-speed mode on or off. | `True`/`False` | `ax.set_safety_speed(False)` |

## Limits, caps and read-only state

What the checks above use. Nothing here moves the axis.

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.speed_cap_mm_s` / `ax.accel_cap_g` / `ax.accel_only_cap_g` / `ax.decel_cap_g` / `ax.push_cap_pct` | The highest value the driver commands. Speed: the lowest of `max_speed_mm_s` and the unit's MAXV (item 7.4). Acceleration (= deceleration on a plain move): the lowest of `max_accel_g`, MAXA and MAXD. On a move with its own deceleration, acceleration is capped by `max_accel_g` and MAXA (`accel_only_cap_g`), deceleration by `max_accel_g` and MAXD (`decel_cap_g`). Push force: the lowest of `max_push_pct` and the unit's MXPP (item 7.9). Each term counts only when known; while none is, 100 units/s, 0.30 G and 70 % apply. | properties (units/s, G, %) | `ax.speed_cap_mm_s` |
| `ax.effective_stroke` / `ax.ec_stroke` | The range every target is checked against: the configured `stroke` intersected with the unit's own soft limits (`None` while nothing is known, or when they do not overlap) / the unit's soft limits minus the 0.30 mm margin (its nominal stroke), once read. No I/O. | `(min, max)` in mm or deg | `ax.effective_stroke` |
| `ax.read_soft_limits()` / `ax.read_speed_limit()` / `ax.read_axis_limits()` | Read the unit's soft limits `(min, max)` / maximum speed (MAXV, item 7.4) / class 7 limits (MAXV, MAXA, MAXD, MXPH, MXPP, MNPP ...). Read-only (FC03). Motion commands do this once by themselves; `ax.limits()` shows the result with its source. | none | `ax.read_soft_limits()` |
| `ax.ec_max_accel_g` / `ax.ec_max_decel_g` | The unit's MAXA / MAXD (items 7.5 / 7.6), once read. | properties (G) | `ax.ec_max_accel_g` |
| `ax.read_support_words()` | The class 1 support-function words by name (SUPT, DIAG, MVPT, SPTS, CTYP, SUP2 ...); the names are UNVERIFIED. Read-only. | none | `ax.read_support_words()` |
| `ax.categories` / `ax.status_words` / `ax.directory_absent` | The unit's directory records as `identify()` read them (empty before) / how many words `status()` reads (22, 20 or 8) / the directory read timed out on a line where that means the units have none (RCON, ROBONET). | properties | `ax.status_words` |
| `ax.servo_on_timeout(st=None)` | `(seconds, why)`: how long a servo ON is waited for: 3 s, or on a CON unit before its first servo ON after power-up the time its parameters give (DOC PCON p.4-24, 6-20). Read-only. | optional `Status` | `ax.servo_on_timeout()` |
| `ax.motion_pending` / `ax.stop_held` | A motion command (move, push, home, jog) was sent and has not been seen to finish / a STOP this axis wrote ON (`begin_stop`, or a stop that could not be released) is still held. | properties | `ax.motion_pending` |
| `ax.begin_stop()` / `ax.finish_home()` | Sends only the first frame of `stop()` and returns at once (`cell.stop_all()` does this on every axis first; follow it with `stop()` or `close()`) / releases the HOME coil after `home(wait=False)` (coil strategy). | none | `ax.finish_home()` |

## Fast path

Optional ways to cut the time around each move. The default stays `completion="safe"`.

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `Axis(..., completion="safe")` / `completion=` on `move_to`, `move_by`, `jog`, `wait_idle`, `move_sequence` | When a move counts as done: `"safe"` (in position, in the band, position unchanged over the settle time) or `"fast"` (in position, in the band, not moving, on one sample: no settle wait). | `"safe"` / `"fast"` | `ax.move_to(50.0, completion="fast")` |
| `ax.move_to(..., wait=False)` / `ax.move_by(..., wait=False)` / `ax.jog(..., wait=False)` | Returns a `MoveHandle` as soon as the command is sent (every check has passed). | as the move | `h = ax.move_to(50.0, wait=False)` |
| `MoveHandle.wait(timeout=None)` / `.done()` / `.result` / `.cancel()` / `.to_dict()` | Waits for the move by its rule (on timeout or alarm the axis is stopped and the error raised) / checks once without blocking / the final status / decel stop / JSON. `timeout=None`: the time left of the move's estimate, at least `settle_s` + 1 s (a late `wait()` on a finished move returns it). | seconds | `h.wait()` |
| `ax.move_sequence(points, speed_mm_s=50, accel_g=0.1, band_mm=0.1, *, blend_band=None, completion=None, timeout=None)` | Runs points one after another. `blend_band`: the next point is sent once the axis is within that distance of the current one (no stop in between); the last point always completes by the rule. Returns the statuses. | points: positions or `{"mm", "speed_mm_s", ...}`; `blend_band` > 0 | `ax.move_sequence([10, 40, 20], blend_band=2.0)` |
| `ax.enable_fast(*, baudrate=230400, monitor=True)` | All of it in one call: `completion="fast"`, the status monitor on, and the link raised to 230400 baud (VERIFIED on the EC-R6S rod, the EC-GRBP8M gripper and the PCON-CB). The rate is volatile (38400 again after a power cycle) and is raised only when the axis owns the port and is at rest. `baudrate=None` keeps the rate. Returns what was done, with `notes`. | 115200 / 230400 / `None` | `ax.enable_fast(baudrate=None)` |
| `ax.disable_fast()` | Back to `completion="safe"` and the monitor stopped; the baud rate is kept. | none | `ax.disable_fast()` |

With the monitor running, a status younger than 50 ms replaces the status read before a move (the direct move is then
one frame) and the waits use its samples instead of polling.

**Where a fast move ends.** `completion="fast"` returns on the first sample that is in position, inside `band_mm` and
not moving. The returned position can be up to `band_mm` short of the target while the unit still settles (VERIFIED:
3.04 mm, then 3.00 mm on the EC-GRBP8M gripper; up to 0.08 mm on the approach side on the PCON-CB, at the target
within 0.25 s). When the next step needs the final position, use `completion="safe"` for that move or read the status
again. A `band_mm` smaller than the creep makes the fast rule wait for it (UNVERIFIED on the PCON-CB).

On the PCON-CB with an RCP6, fast completion, `MoveHandle` (`wait=False`), `change_speed()` during a move, the status
monitor, `track()` and `check_link()` ran as described here (VERIFIED).

## Stored positions (AVD)

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.max_positions()` | Number of stored rows the unit can read and store. On an EC the position mode (parameter P9) decides which run: rows 0 and 1 in 2-position mode, rows 0-2 in 3-position mode. | none (read from the unit: 3 on the ECs tested, 64 on the PCON-CB) | `ax.max_positions()` |
| `ax.read_position_table(n)` | Reads one stored row as a `PositionEntry`. | row 0 .. `max_positions() - 1` | `ax.read_position_table(1)` |
| `ax.move_to_position(n, wait=True, timeout=None, *, speed_pct=None, allow_above_caps=False)` | Runs a stored row at its stored speed, or at `speed_pct` % of it (0x45 cmd 6; nothing written to the unit; VERIFIED on the EC gripper and the PCON-CB). On an EC a row its position mode does not use is refused with `SoftLimitError` before anything is sent; the EC-GRBP8M gripper in 2-position mode rejected row 2 with code 0x11 (VERIFIED). When P9 is not readable the row is sent and that rejection comes back as the same `SoftLimitError`. | row 0 .. `max_positions() - 1`; on an EC only the rows of its position mode (2-position: 0 and 1); `speed_pct` 1..100 | `ax.move_to_position(1, speed_pct=25)` |
| `ax.teach_position(n, *, allow_nonvolatile=False, method="table", allow_unsafe=False, force=False)` | Stores the current position as row `n` (permanent). Refused before homing and while the axis moves. `method="coil"` is EXPERIMENTAL. A row that already holds the position (0.01 resolution, nothing else to fill in, checksum good) is not written and comes back with `skipped=True`; this is decided before the caps are checked, so a row stored at 53 mm/s is skipped on an axis opened with `max_speed_mm_s=10`. `force=True` teaches anyway. A write counts in `ax.nonvolatile_writes`. `allow_nonvolatile` and `allow_unsafe` are accepted and ignored (no longer needed). | row 0 .. `max_positions() - 1`; on an EC a row its position mode does not run is written with a warning (logged at WARNING on the `iai_actuator_driver` logger); the row's other fields are kept and must fit the caps when it is written | `ax.teach_position(1)` |
| `ax.write_position_table(n, entry, allow_nonvolatile=False, *, write_checksum=True, allow_unsafe=False, allow_unverified_model=False, force=False)` | Writes a stored row (permanent) and verifies it (`allow_unverified_model` is ignored, kept for compatibility). A row that would not change is not written and comes back with `skipped=True`; this is decided before the values are checked against the stroke and caps. `force=True` writes anyway. A refused value names the row, the field, its unit and whose limit it breaks. A write counts in `ax.nonvolatile_writes`. `allow_nonvolatile` and `allow_unsafe` are accepted and ignored (no longer needed). | position, push power and load threshold within the unit's limits; a row past the unit's rows, a row the EC's position mode does not run, a push on the middle row, unknown control-flag bits or a role-bit change is written with a warning (logged at WARNING on the `iai_actuator_driver` logger); rows 256-267 are refused | `ax.write_position_table(1, entry)` |

Change one field of a row: `entry = dataclasses.replace(ax.read_position_table(2), position_mm=60.0)`.

## Backup (read-only)

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `info.backup(ax, path=None, *, rows="used")` | One axis as a JSON-able dict (identity, directory, support words, every parameter item, stored rows raw and decoded, alarm history, maintenance counters); also written to `path`. Sends FC03 and FC11 only; nothing is written to the unit. Restoring is not part of the driver. | `rows`: `"used"`, `"all"` (an EC has 768: slow) or row numbers | `backup(ax, "unit1.json")` |
| `info.backup_axes(axes, directory, *, rows="used")` / `cell.backup(directory, axes=None, *, rows="used")` | The same for several axes, one `<axis>.json` each; an axis that does not answer is reported and the others are still saved. | directory path | `cell.backup("backups")` |

`backup` is in `iai_actuator_driver.info`. Back a unit up before any non-volatile write.

## Parameters

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.parameters()` | Lists this unit's named parameters with value, unit, range. Read-only. | none | `ax.parameters()` |
| `ax.read_parameter(key)` | Reads one parameter. | number (`2`, `"P2"`, `"No.35"`) or item `"class.item"` | `ax.read_parameter(2)` |
| `ax.write_parameter(key, value, *, commit=True, allow_dangerous=False, allow_unlisted=False, allow_home_params=False, force=False)` | Writes one parameter, verifies it, saves it. Applies after `software_reset()` or a power cycle. A value the unit already holds is neither written nor committed (`result.skipped`; `changed` / `committed` False), unless a parameter this process wrote to the unit's RAM still waits for a commit (`commit=False`, or a commit that failed or was not confirmed; the commit then saves it) or `force=True`. A RAM-only write by another program or process (such as an earlier CLI `--no-commit` run) is not seen: use `force=True`. A commit counts in `ax.nonvolatile_writes`. | range from the unit (`read_parameter` shows `min`/`max`); a home, dangerous or unlisted parameter is written with a warning (`result.warnings`) | `ax.write_parameter(2, 0.2)` |
| `ax.parameter_model()` | This unit's parameter list: the one recorded for its unit code, or every mapped item found on it. | none | `ax.parameter_model()` |
| `ax.user_parameters()` | The EC user parameters (IAI's P numbering) of this unit, decoded: one dict per parameter in the unit's own list. Read-only (FC03, FC11 once). | none | `ax.user_parameters()` |
| `ax.write_user_parameter(no, value, *, commit=True, allow_home_params=False, allow_unlisted=False, allow_unverified_model=False, force=False)` | Writes EC user parameter P`no`, verifies it by read-back and (`commit`) saves it. Only parameters confirmed on hardware are accepted, on every model the same way (`allow_unverified_model` is ignored, kept for compatibility). Same non-volatile guard as `write_parameter`. | P number from `user_parameters()`; the value in the parameter's unit | `ax.write_user_parameter(2, 0.2)` |
| `ax.nonvolatile_writes` / `ax.nonvolatile_writes_last_hour` | Non-volatile writes this process made on this unit (parameter commits, stored-row writes, teaching), in all / in the last hour, over every axis object on the same unit (a reconnect or a new object per job keeps counting). Read-only; skipped writes are not counted. | none | `ax.nonvolatile_writes` |
| `ax.read_home_pattern()` / `ax.read_boot_baudrate()` | Home pattern / power-up baud rate. Read-only. | none | `ax.read_boot_baudrate()` |

No opt-in: a home direction / offset, encoder / motor type, lead / stroke configuration, servo gains, PIO, fieldbus
or baud setup, a wider soft limit, an unknown raw item or a parameter the unit's list does not show is written with a
warning. `allow_dangerous`, `allow_home_params` and `allow_unlisted` are accepted and ignored (no longer needed).
Read-only and unconfirmed parameters and a value outside the range are still refused.

## Alarms

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.status().alarm` / `.alarm_code` / `.warning` / `.alarm_level` | Current alarm (blocks motion), its code, warning (does not block), level. | none | `ax.status().alarm_code` |
| `ax.reset_alarm(*, method=None)` | Clears the current alarm. Returns the new status. | `method`: `None`, `"coil"` or `"iai"` | `ax.reset_alarm()` |
| `ax.read_alarm_history(*, include_empty=False)` | Alarm history, newest first, as `AlarmRecord` (`code`, `name`, `level`, `time_after_power_on`). | none | `ax.read_alarm_history()[0].name` |
| `alarms.alarm_info(code)` | Name, level, how to reset, advice for an alarm code. No I/O. | alarm code | `alarm_info(0x0EE)["reset"]` |

`alarm_info` is in `iai_actuator_driver.alarms`.

## Unit & link (reset, baud, identity)

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.software_reset(*, timeout=..., reset_wait_s=...)` | Restarts the unit's firmware and reconnects. The axis must be at rest; the servo turns off. Incremental units must home again. The restart is confirmed by the servo coming back on by itself (VERIFIED on the EC-GRBP8M gripper) or another sign. PMSS afterwards depends on the unit: the EC-R6S rod dropped it to 0, the EC-GRBP8M gripper and the PCON-CB kept 1 (VERIFIED). | none | `ax.software_reset()` |
| `ax.set_baudrate(rate, *, method="direct", settle_s=0.2, allow_unverified_code=False)` | Changes the unit's and the port's baud rate (until power-off). | 115200 or 230400; 38400 is sent and checked afterwards (the EC-GRBP8M gripper and the PCON-CB take it, the EC-R6S rod ignores it); another rate with an IAI code is sent with a warning (`allow_unverified_code` is accepted and ignored); a rate with no code is refused | `ax.set_baudrate(115200)` |
| `Port.set_baudrate(rate, *, slaves=None)` | Same, for every unit on a shared port. | as above | `p.set_baudrate(115200)` |
| `ax.baudrate` | Baud rate in use. | property | `ax.baudrate` |
| `DeviceInfo` (from `connect()` / `identify()`) | `product_family`, `model`, `controller`, `actuator`, `serial_number`, `firmware`, `axis_number`, `units`, `soft_limits_mm`, `max_speed_mm_s`. | fields | `info.product_family` |

## Power & link loss

The driver can tell when an axis's link fails or its unit restarted, bring it back, and say what changed. It **never
moves the axis** while doing so: no servo ON, no home, no move, and a move cut off by the fault is not resumed. Most
functions are in `iai_actuator_driver.recovery`.

- **What a power-up looks like (VERIFIED).** The baud rate is back at 38400, PC control is off, the uptime counter
  0x9010 restarts (a software reset does not restart it), an incremental unit loses HEND and a PCON raises 0x069. The
  alarm history gets a 0x0FFF power-up record (none after a 3.2 s outage on the gripper). A pulled USB adapter makes
  the COM port vanish.
- **Recovery on the EC gripper (VERIFIED)** after a real power cycle, both in a fresh process (identity from a
  backup) and seen live by a running process (`check_link()` reported `unit_silent`, then `unit_restarted`); PC
  control came back and the rows and parameters were unchanged. Also after a line break: the unit drops to 9600 and is
  found and put back on 38400 or 230400, also through a Cell's link monitor, `on_fault` and `recover_all`.
- **Recovery on the PCON-CB (VERIFIED)** after a 250 ms line break: the unit dropped to 9600 with no alarm, servo ON
  and PC control kept, and was put back on 38400 or 230400 with its rows and parameters checked.
- After a line break `RecoveryReport.control_restored` is False because control was never lost: read `control_kept`
  or `modbus_control` for "PC control on".

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.track(**kw)` / `recovery.track(ax, *, name=None, info=None, alarm_history=True, rows=False, parameters=False, backup=None)` | Remembers the session while the link is up: identity, baud rate, PC control, safety speed, servo, HEND, uptime, alarm history; optionally the stored rows / parameters / a backup to compare with later. Read-only. | `rows`: `True` or row numbers | `ax.track()` |
| `ax.check_link()` / `recovery.check_link(ax, *, session=None, on_event=None, name=None, lister=None)` | One status read compared with the session: `"ok"`, `"port_vanished"`, `"unit_silent"` or `"unit_restarted"` (with the reasons). Read-only. | none | `ax.check_link().state` |
| `ax.recover(*, timeout=None, policy=None, session=None, on_event=None, name=None)` / `recovery.recover(ax, ...)` | Waits for the port to be listed again and reopens it, finds the baud rate (38400 first after a power loss), requires the same unit (serial number, model), restores baud rate / PC control / safety speed, resets a message-level alarm, and reports HEND, alarm and position. PC control is reported as taken again or kept (a line break does not drop it, VERIFIED on the PCON-CB). `ax.recover()` holds every monitor on the port meanwhile and restarts its own after. A move still running after a pulled adapter is reported (`still moving`), not stopped. Without `track()` the identity read at `connect()` is the baseline. Returns a `RecoveryReport`. | `timeout` seconds (`None`: 30 s) | `report = ax.recover(timeout=60)` |
| `RecoveryReport` | `cause`, `restarted`, `restart_signs`, `port_reopened`, `baudrate_found` / `baudrate` / `baud_restored`, `same_unit`, `control_restored` / `control_kept`, `alarm_code`, `needs_home`, `position_mm` / `last_target_mm` / `at_target`, `row_problems`, `parameter_differences`, `actions`, `notes`, `needs_attention`, `ready_to_move`, `summary()`, `to_dict()`. | fields | `report.needs_home` |
| `RecoveryPolicy(restore_baudrate=True, restore_control=True, restore_safety_speed=True, clear_message_alarms=True, check_rows=False, check_parameters=False, backup=None, position_tolerance_mm=0.1, poll_s=0.5, boot_baudrate=38400, list_ports=None)` | What `recover` restores and checks. `boot_baudrate`: the rate a unit has after a power-up; `list_ports`: a function that returns the PC's port list (default: the real list). | as listed | `RecoveryPolicy(check_rows=True)` |
| `ax.link_state` / `recovery.link_state(ax)` | The last known link state (`"ok"` when nothing was seen). | property | `ax.link_state` |
| `RecoveryEvent` (`kind`, `axis`, `message`, `cause`, `time`) | Given to `on_event` callbacks: `"link_lost"`, `"unit_restarted"`, `"unit_replaced"`, `"recovered"`, once per change. The status monitor sends them too. | fields | `event.kind` |
| `recovery.note_target(ax, mm)` / `recovery.note_control(ax, *, modbus_control=None, safety_speed=None, baudrate=None, homed=None)` | Records the last commanded target (moves record it themselves) / a deliberate change, so it is not taken for a restart (`disable_modbus_control()`, `set_action_mode()`, `software_reset()` and `home()` record theirs). | mm / flags | `note_control(ax, modbus_control=False)` |
| `recovery.session_from_info(ax, info)` / `recovery.session_from_backup(ax, backup)` / `recovery.session_of(ax)` / `recovery.forget(ax)` | A session from an identity already read / from a `backup()` dict or file (the unit must be that one; rows and parameters compared with it), without I/O / the axis's session / drop it. | `DeviceInfo`; dict or path | `session_from_backup(ax, "unit1.json")` |
| `recovery.classify_failure(exc, ax=None)` / `recovery.restart_signs(session, status)` / `recovery.port_present(ax)` | The link state an exception means (`None`: not a link fault) / the restart signs in a status / whether the axis's port is listed by the PC (opens nothing). | exception; status | `classify_failure(exc)` |
| `recovery.LinkMonitor(axes, *, interval_s=1.0, on_change=None, on_event=None)` | A background thread running `check_link` on several axes; never recovers anything itself. `start()` / `stop()` / `check_now()` / `states`. | seconds | `LinkMonitor([ax]).start()` |
| `RecoveryError` / `UnitReplacedError` | The port / unit did not come back in time, or the identity could not be confirmed (`.report`) / another unit answers where the tracked one was: nothing was written to it (`.expected`, `.found`). | exceptions | `except UnitReplacedError` |
| `ModbusRTU.reopen(*, baudrate=None)` / `.closed` / `.can_reopen` / `SerialTransport.reopen(baudrate=None)` | Opens the same port again in place after it vanished (every axis on the bus keeps working); nothing is sent. | baud rate | `ax.bus.reopen()` |
| `TransportError` | The port failed underneath the bus (USB adapter pulled). A `ModbusError`; never retried. | exception | `except TransportError` |
| `SimulatedEC(serial_number=...)`, `.power_off()` / `.power_on()` / `.power_cycle(lose_home=None)`, `.unplug()` / `.replug()` / `.reopen()`; `SimLine` the same, `power_cycle(slaves=None)` | Simulator faults: a power loss (baud rate, PC control, uptime, HEND, alarm record as above) and a pulled USB adapter (the port vanishes). | as listed | `sim.power_cycle()` |

## Lines and gateways

The line is detected once when the port opens (`Axis.open`, `Port`) with two read-only FC03 probes. Behind a gateway
the line's rules apply, and anything the line does not carry is refused **before** it is sent (`LineRuleError`).
Every gateway rule is DOC and UNVERIFIED on hardware.

| Line (`ax.bus.line.kind`) | What it carries | Notes |
|---|---|---|
| `"direct"` | everything the driver knows | one EC or PCON-CB per port VERIFIED; a daisy chain UNVERIFIED |
| `"rcon"` | FC 01/03/04/06/10/17; no coils, no 0x45/0x48, no broadcast | command strategy `"register"` (FC06 DRG1), stop without a coil, identity from the 0x9400 serial code (FC11 is not sent), a silent 0x0000 directory means "no directory" (pass `stroke=` and speeds), gateway in MANU before control, `position_unit_mm` (0.01 or 0.001, parameter No.207) before any mm command |
| `"robonet"` | FC 01-07, 0F, 10, 11, 17 (SIO through mode, SW2 ON); no 0x45/0x48, no broadcast | strategy `"coil"`; FC10 up to 75 registers; a silent directory means "no directory"; state it with `line="robonet"` (through mode cannot be probed) |
| `"robonet-gateway"` | gateway mode (SW2 OFF): no axis access | `UnsupportedLineError`; set SW2 ON |

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `ax.bus.line` / `Port.line` | The `Line`: `kind`, `rules`, `allows(op)`, `require(op)`, `position_unit_mm(slave)`, `to_dict()`. | properties | `ax.bus.line.kind` |
| `Port.detect_line(*, force=False)` | Works the line out (read-only); done automatically. | none | `p.detect_line()` |
| `Port.broadcast_stop(on=True)` / `Port.broadcast_pause(on=True)` / `Port.broadcast_servo_off()` | One broadcast frame (address 0) to every unit on the line: the STOP coil / the pause coil (`on=False` releases) / servo OFF. No unit answers. Refused behind a gateway (`LineRuleError`). Returns the frame. Whether every unit acts on it is UNVERIFIED. `broadcast_servo_off` lets a vertical axis without a brake fall. | none | `p.broadcast_stop()` |
| `Port.ensure_baudrate(slaves=1..16)` | Finds the baud rate once, in place, by asking `slaves` at each rate; later calls return at once. `scan()` and `Cell.open()` do it. Raises `ModbusTimeout` when nothing answers at any rate. | addresses 1..16 | `p.ensure_baudrate()` |
| `Port.gateway_status()` / `Port.gateway_monitor(slaves=None)` | The gateway's status (MANU/AUTO, linked axes, alarm) / the RCON all-axes monitor, 2 frames for 16 axes. Read-only. | slaves 1..16 | `p.gateway_monitor([1, 2])` |
| `Port.set_position_unit(unit_mm, slaves=None)` | States an RCON unit's position unit. | 0.01 or 0.001 | `p.set_position_unit(0.001)` |
| `gateway.read_serial_code(bus, slave)` | The 0x9400 serial code (identity where FC11 is not documented). Read-only. | slave | `read_serial_code(ax.bus, 1)` |
| `ax.line_name` / `ax.stop_path` | The line kind (`None` on a direct line) / `"coil"` or `"register"`. | properties | `ax.stop_path` |

An ELECYLINDER behind an RCON-EC / REC connection unit has its serial port disabled and is not reachable (DOC). It
shows up as linked at the gateway but silent on Modbus, and the driver says so.

## Multi-axis group commands

Ports run in parallel; axes on one shared port take turns. Every `*_all` takes `axes=None` (all) and
`return_exceptions=False`; on any failure it raises `CellError` after handling every axis. If one axis fails during
`move_all` / `home_all` / `wait_all`, the others are stopped too (`stop_others_on_error=False` leaves them running).

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `cell.identify_all()` | Identifies every axis (stored in `cell.info`). | none | `cell.identify_all()` |
| `cell.status_all(*, via_gateway=None, monitor_max_age_s=0.1)` | Status of every axis, `{name: Status}`. An axis whose status monitor runs gives its latest sample (the lean status) instead of a read; `monitor_max_age_s=None` always reads. On an RCON line the status comes from the gateway monitor (2 frames for 16 axes; no DSS2, ZONS or POSS). | `via_gateway`: `None` auto, `True`, `False`; seconds or `None` | `cell.status_all()["COM3:1"].position_mm` |
| `cell.limits_all()` | Each axis's own ranges, units and kind. | none | `cell.limits_all()` |
| `cell.monitor_all()` | Monitor values of every axis. | none | `cell.monitor_all()` |
| `cell.units_of(key)` | `"mm"` or `"deg"` for one axis. | axis name | `cell.units_of("COM3:1")` |
| `cell.servo_all(on, *, timeout=3.0)` | Servo on/off on every axis. | `True`/`False` | `cell.servo_all(True)` |
| `cell.home_all(*, timeout=None, method=None, stop_others_on_error=True)` | Homes every axis at once, waits for all. | as `home` | `cell.home_all()` |
| `cell.move_all(targets, speed_mm_s=50, accel_g=0.1, band_mm=0.1, *, decel_g=None, wait=True, timeout=None, stop_others_on_error=True)` | Starts an absolute move on every listed axis, waits for all. | `{axis: position}`; each value one number or `{axis: value}`; ranges from each unit | `cell.move_all({"COM3:1": 50.0, "COM4:1": 20.0})` |
| `cell.wait_all(*, timeout=None, stop_others_on_error=True)` | Waits until every axis is in position. | none | `cell.wait_all()` |
| `cell.stop_all(*, timeout=5.0, broadcast=False)` | Decel stop on every axis, as fast as possible. `broadcast=True` first sends one broadcast STOP on every daisy chain whose axes are all selected, then the per-axis stop as the fallback (never behind a gateway; UNVERIFIED on hardware). | none | `cell.stop_all(broadcast=True)` |
| `cell.pause_all(*, broadcast=False)` / `cell.resume_all(*, broadcast=False)` | Pause / release every axis (`ax.pause()` / `ax.resume()`), optionally with one broadcast frame on daisy chains. | none | `cell.pause_all()` |
| `cell.lines` / `cell.gateways` / `cell.gateway_status_all()` | The line kind of every port / the gateway objects / a fresh gateway status per port. Read-only. | properties | `cell.lines` |
| `cell.start_monitors(axes=None, **kw)` / `cell.stop_monitors(axes=None)` | Starts / stops every axis's status monitor; their link events go to `cell.events` and the fault handling below. Read-only. | `ax.start_monitor` options | `cell.start_monitors()` |
| `Cell.open(..., on_fault=None, stop_on_fault=True, on_event=None)` / `cell.faults` / `cell.events` | A link fault on one axis (seen by any command, the checks or a monitor) calls `on_fault(name, kind, error)` once and, with `stop_on_fault`, stops every other axis. `faults` lists the faulted axes until they are recovered, `events` the recent `RecoveryEvent`s. | callables | `Cell.open(on_fault=print)` |
| `cell.track_all(**kw)` / `cell.check_links()` / `cell.monitor_links(interval_s=1.0)` | `track` / `check_link` on every axis (ports in parallel) / a background `LinkMonitor` for the cell. Read-only. | seconds | `cell.track_all()` |
| `cell.recover_all(axes=None, *, timeout=30, policy=None)` | `ax.recover()` on every axis (ports in parallel, the axes of a port one after another); never moves anything. Returns `{name: RecoveryReport}`. | seconds | `cell.recover_all()` |
| `cell.close()` | Closes every port (also done by `with`). | none | `cell.close()` |

`cell.backup()` is under [Backup](#backup-read-only).

## Coordinated motion (alpha)

Lines, arcs and circles over several axes, interpolated by the PC. Alpha: one real axis (the EC gripper, the PCON-CB)
ran a line, a circle and a stop together with a simulated second axis; two or more real axes together are UNVERIFIED.
How a run works, what it checks, what stops it and its limits: [REFERENCE.md](REFERENCE.md#coordinated-motion).

### What it is

An ELECYLINDER or CON unit has no interpolation of its own and no clock shared with other axes.
`iai_actuator_driver.coord` makes several axes follow one path by commanding them from the PC:

```python
from iai_actuator_driver import Cell, Path

with Cell.open(["COM3", "COM4"]) as cell:
    xy = cell.coordinated(x="COM3:1", y="COM4:1")        # names for the path coordinates
    plan = xy.plan(Path.circle({"x": 60, "y": 40}, 10), speed=10, accel_g=0.1)   # dry run: nothing sent
    print(plan.summary())
    result = xy.run_plan(plan)                           # or xy.circle((60, 40), 10, speed=10)
    print(result.summary(), result.max_deviation)
```

The names below are in `iai_actuator_driver.coord` (the main ones also in `iai_actuator_driver`).

- **Coordinates** are `{axis name: position}` in each axis's units (mm, or deg on a rotary axis). An axis a segment
  does not name holds its position.
- **Names:** `iai_actuator_driver.Line` is the gateway line class. The segment is `iai_actuator_driver.coord.Line`, also
  exported as `iai_actuator_driver.LineSegment`.

### Functions

| Function | What it does | Inputs and ranges | Example |
|---|---|---|---|
| `cell.coordinated(*axes, **named)` | A `CoordinatedGroup` of cell axes; the names are the path coordinates. No I/O. | `x="COM3:1"`, a `{name: axis}` mapping, cell names, or nothing (every axis) | `xy = cell.coordinated(x="COM3:1", y="COM4:1")` |
| `CoordinatedGroup({name: axis})` | The same for axes not in a cell; axes sharing a bus are one port. | `Elecylinder` objects | `CoordinatedGroup({"x": ax1, "y": ax2})` |
| `Line(to)` / `Arc(center, end, ccw=True, plane=("x", "y"))` / `Circle(center, radius, start_deg=0, turns=1, ccw=True, plane=...)` / `Polyline(points)` | Path segments. `Arc`: end on the circle through the start (end = start: full circle); axes of `end` outside the plane move linearly (helix). `Circle`: the path must be at its start. `Line` is `iai_actuator_driver.LineSegment` at the top level. | positions in axis units; `turns` > 0 | `Arc((50, 50), {"x": 50, "y": 60})` |
| `Path(start=None, segments)` / `.then(*segments)` / `.length()` / `.pieces()` | Start point + segments; `start=None`: from where the axes are. | mappings | `Path({"x": 0, "y": 0}).then(Line({"x": 10}))` |
| `Path.circle(center, radius, *, plane, start_deg=0, turns=1, ccw=True)` / `Path.rectangle(corner, width, height, *, plane)` / `Path.polygon(center, radius, sides, *, plane, start_deg=0)` | Demo shapes, closed. | sides >= 3 | `Path.circle({"x": 75, "y": 75}, 10)` |
| `xy.plan(path, speed, accel_g=0.1, *, chord_error=0.05, mode="auto", min_step_s=None, band=None, blend=None, stream_accel_g=None)` | Dry run from the current positions against each axis's `limits()` and the line timing, both read now: returns a `MotionPlan`. A path starting elsewhere gets a straight lead-in, with a warning: one more segment of a sync path, `plan.lead_in` (a synchronized line, run first) for a streamed one. A path that moves no axis by a count from its own start is the lead-in alone. Reads only. | `mode`: `"auto"`, `"sync"` (lines), `"stream"`; `chord_error` > 0; `blend` (sync) > 0 | `p = xy.plan(path, 20, 0.3)` |
| `plan_path(path, speed, accel_g=0.1, *, limits=None, timing=None, min_step_s=None, ...)` | The same planner, pure (`iai_actuator_driver.coord.plan`): `limits` `{axis: AxisRange or limits() dict}`. A refusal's `.max_speed` is the largest speed that passes every check (the checks re-run). | as above | `plan_path(path, 20, limits={"x": ax.limits()})` |
| `MotionPlan` | `mode`, `axes`, `points`, `times`, per-step per-axis `speeds` / `accels` (0 = no frame), `duration`, `shortest_step_s`, `min_step_s`, `est_chord_error`, `est_timing_error_s`, `est_deviation`, `warnings`, `lead_in`, `summary()`, `table()`, `to_dict()`. | fields | `p.summary()` |
| `xy.run(path, speed, accel_g=0.1, *, late_abort_s=None, timeout=None, **plan_kw)` / `xy.run_plan(plan, *, late_abort_s=None, timeout=None)` | Runs it (its `lead_in` first) as in [How it runs](REFERENCE.md#how-it-runs), after the [checks](REFERENCE.md#checks-before-anything-is-sent) of `run_plan()` (a streamed plan this group made on the same line is held to the lower of the two line timings). The run's status monitors are held meanwhile (other axes' keep polling). A stop of one of its axes from the moment `run_plan` is called (lead-in included) ends it ([Stops and errors](REFERENCE.md#stops-and-errors)); a link fault stops the group before the cell's `on_fault` runs; one run at a time per axis. Returns a `CoordinatedResult`. | `late_abort_s`: default one step | `xy.run(Path.rectangle((10, 10), 30, 20), 20)` |
| `xy.line_to(point, speed, accel_g=0.1)` / `xy.circle(center, radius, speed, accel_g=0.1, *, plane=None, start_deg=None, turns=1, ccw=True)` | A synchronized line / a streamed circle from the current position (the circle starts where the axes are when they are on it, within 0.5 % of the radius). `line_to` where the axes already are: a completed result, nothing sent. | as `run` | `xy.circle((75, 75), 10, speed=20)` |
| `xy.stop()` / `xy.positions()` / `xy.ranges()` / `xy.timing(names=None, *, measure=False)` | Decel stop on every axis of the group (a run in another thread ends with `CoordinatedMotionError`) / current positions / each axis's `AxisRange` (read now) / `timing()` raises `NotImplementedError`: the core measures the line for each plan and run itself; use `plan.timing`. | none | `xy.stop()` |
| `CoordinatedResult` | `plan`, `samples` (per axis `(t, position)` from the status reads during the run), `trace`, `max_deviation` / `mean_deviation` (from the planned chords: an estimate from the samples, only where every axis was read within one read time), `sample_spacing_s`, `frames`, `skipped_frames`, `late_frames` (measured when a frame's call begins, not when it is on the wire), `worst_late_s`, `worst_skew_s`, `duration_s`, `completed`, `error`, `summary()`, `to_dict()` (JSON, for the GUI). The run reports when it ends: no samples while it runs. | fields | `r.max_deviation` |
| `LineTiming(baudrate=115200, axes_on_port=1, response_ms=5, gap_s=0.005, ..., measured_write_s=None, measured_read_s=None)` | The shortest step one port can carry (a write and a status read per axis, x1.5): the model's value (below 38400 baud every reply taken as ~285 ms, VERIFIED at 9600), or what was measured when that is longer (on the EC gripper the measured step was longer than the model's, VERIFIED). | properties `write_s`, `read_s`, `min_step_s`, `model` | `LineTiming(230400).min_step_s` |

### Demo

```shell
python examples/python/draw_circle.py --sim                              # two simulated axes
python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --dry-run    # plan only (reads positions and ranges)
python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --radius 5 --speed 5
```

On hardware: servo on and home both axes first. Start small and slow, unloaded and mid-stroke, with a hand on the
emergency stop.

## Errors

All derive from `EcDriverError`.

| Exception | When |
|---|---|
| `SoftLimitError` | A target, speed, accel or force is outside the unit's range. Nothing was sent. |
| `EcNotReadyError` | The axis is not ready (servo off, not homed, moving...). |
| `ServoBlockedError` | The servo cannot come on (EMG input, drive power cut, enable switch released ...); `.reasons` lists them. A kind of `EcNotReadyError`. |
| `EcAlarmError` | The unit has an alarm. |
| `MotionTimeoutError` | A move or home did not finish in time (the axis is stopped). |
| `ModbusTimeout` | No answer: check power, cable, address, and that IA-OS is closed. The message also names the product lines that are [not supported](COMPATIBILITY.md#not-supported). |
| `UnsafeCommandError` | A refused command or write: an IAI command off the safe list without `allow_unsafe`, one on the never-send list, a read-only or unconfirmed parameter. |
| `LineRuleError` | The line the unit sits on does not carry the command (no coils, IAI commands or broadcast behind a gateway). Nothing was sent. A kind of `UnsafeCommandError`. |
| `GatewayModeError` / `PositionUnitError` / `UnsupportedLineError` | RCON gateway not in MANU / an mm command on RCON before the position unit is stated / the line gives no axis access (ROBONET gateway mode, an EC behind an RCON-EC or REC). |
| `CellError` | One or more axes failed in a group command (details per axis). |
| `CoordinatedMotionError` | A coordinated run failed or was stopped from outside (`xy.stop()`, `cell.stop_all()`, a stop or pause of one of its axes); every axis of the group was stopped. `.result` holds the trace so far. Ctrl+C is not wrapped: it stops the group the same way, then re-raises `KeyboardInterrupt` with the result in `.coordinated_result`. |
| `TransportError` | The port failed underneath the bus (USB adapter pulled): the axis's link state is `"port_vanished"`; `recover()` reopens it. A kind of `ModbusError`. |
| `RecoveryError` / `UnitReplacedError` | `recover()` could not bring the axis back in time / another unit answers where the tracked one was (nothing was written to it). In `iai_actuator_driver.recovery`. |

## Safety

- **This driver is not a safety function.** Keep a hardware emergency stop and guarding.
- **No communication watchdog.** If the PC or cable fails, a move in progress finishes and a push keeps pushing. Only
  the hardware emergency stop stops the axis independently of the PC.
- Test new setups unloaded, mid-stroke and slow.
- **Non-volatile memory wears out.** It lasts about 100,000 write cycles (IAI; the manuals are cited in
  [REFERENCE.md](REFERENCE.md#safety-notes)). Parameter commits, stored-row writes and teaching write it.
- Direct moves (`move_to`, the 0x9900 block, coordinated motion) are RAM only and cost nothing (VERIFIED). Safety
  speed, Modbus control and the baud rate are back at their power-up state after a power cycle (VERIFIED: RAM only);
  the action mode (`set_action_mode`) sets the same Modbus-control / safety-speed state.
- The driver writes nothing when a value or row would not change (`skipped`; `force=True` writes anyway). A parameter
  left in RAM by another program or process, such as an earlier CLI `--no-commit` run, is not seen: commit it with
  `force=True`. Writes are counted per axis over every axis object of the process (`ax.nonvolatile_writes`), and one
  WARNING per hour is logged while more than 60 (`NV_WARN_PER_HOUR`) fall in the last hour on one axis. A write is
  never refused for the rate. For a target that changes every cycle, use `move_to`, not a stored row or a parameter.

## Supported units

Every IAI unit that speaks Modbus RTU on its RS-485 SIO port: any ELECYLINDER (EC) or CON-family controller, any
actuator kind, RCON driver units through the gateway and ROBONET in through mode. Ranges are read from the unit, so a
new model needs no driver update. What has run on hardware, what is not supported and what can go wrong:
[COMPATIBILITY.md](COMPATIBILITY.md).

## Names in the other editions

The same calls in each language. Arguments, ranges and units are the same. The C names are in
`core/include/iai_actuator_driver.h` ([dev/NATIVE_ABI.md](dev/NATIVE_ABI.md)), the C++ ones in
`core/include/iai_actuator_driver/` ([../core/README.md](../core/README.md)), the C# ones in the API table of
[../bindings/dotnet/README.md](../bindings/dotnet/README.md). C passes options in `iad_*_options` structs (always call
`iad_*_options_init` first) and returns an `iad_result` (0 = `IAD_OK`).

| Task | Python | C# | C++ | C |
|---|---|---|---|---|
| Open a port | `Axis.open("COM3")` | `Axis.Open("COM3")` | `Axis::open("COM3")` | `iad_axis_open("COM3", NULL, NULL, &ax)` |
| Open the simulator | `Axis.open(SimulatedEC())` | `Axis.Open(new SimulatedUnit())` | `Axis::open(std::make_shared<SimulatedUnit>())` | `iad_axis_open_sim("slider", NULL, NULL, NULL, &ax)` |
| Identity and ranges | `ax.connect()`, `ax.identify()`, `ax.limits()` | `Connect()`, `Identify()`, `Limits()` | `connect()`, `identify()`, `limits()` | `iad_axis_connect`, `iad_axis_identify`, `iad_axis_limits_json` |
| Status | `ax.status()` | `axis.Status()` | `ax.status()` | `iad_axis_status` |
| Servo, home | `ax.servo(True)`, `ax.home()` | `Servo(true)`, `Home()` | `servo(true)`, `home()` | `iad_axis_servo`, `iad_axis_home` |
| Absolute, relative move, jog | `move_to`, `move_by`, `jog` | `MoveTo`, `MoveBy`, `Jog` | `move_to`, `move_by`, `jog` | `iad_axis_move_to`, `iad_axis_move_by`, `iad_axis_jog` |
| Check without moving | `ax.check_move(...)` | `CheckMove(...)` | `check_move(...)` | `iad_axis_check_move` |
| Push | `ax.push(...)` | `Push(...)` | `push(...)` | `iad_axis_push` |
| Stop, pause, resume | `stop()`, `pause()`, `resume()` | `Stop()`, `Pause()`, `Resume()` | `stop()`, `pause()`, `resume()` | `iad_axis_stop`, `iad_axis_pause`, `iad_axis_resume` |
| Do not wait for a move | `move_to(..., wait=False)` returns a `MoveHandle` | `StartMoveTo(...)` returns a `MoveHandle` | `start_move_to(...)` returns a `MoveHandle` | `iad_axis_move_to` with `wait = 0`, then `iad_axis_wait_idle` |
| Change speed during a move | `change_speed(v)` | `ChangeSpeed(v)` | `change_speed(v)` | `iad_axis_change_speed` |
| Stored rows | `read_position_table`, `write_position_table`, `teach_position`, `move_to_position` | `ReadPositionTable`, `WritePositionTable`, `TeachPosition`, `MoveToPosition` | the same names in snake case | `iad_axis_read_position_table`, `iad_axis_write_position_table`, `iad_axis_teach_position`, `iad_axis_move_to_position` |
| Parameters | `parameters()`, `read_parameter`, `write_parameter` | `Parameters()`, `ReadParameter`, `WriteParameter` | `parameters()`, `read_parameter`, `write_parameter` | `iad_axis_parameters_json`, `iad_axis_read_parameter`, `iad_axis_write_parameter` |
| Alarms | `reset_alarm()`, `read_alarm_history()` | `ResetAlarm()`, `ReadAlarmHistory()` | `reset_alarm()`, `read_alarm_history()` | `iad_axis_reset_alarm`, `iad_axis_read_alarm_history_json` |
| Baud rate, software reset | `set_baudrate(rate)`, `software_reset()` | `SetBaudRate(rate)`, `SoftwareReset()` | `set_baud_rate(rate)`, `software_reset()` | `iad_axis_set_baud_rate`, `iad_axis_software_reset` |
| Fast mode, status monitor | `enable_fast()`, `start_monitor()` | `EnableFast()`, `StartMonitor()` | `enable_fast()`, `start_monitor()` | `iad_axis_enable_fast`, `iad_axis_start_monitor`, `iad_axis_next_event` |
| Unit info, backup, self-test | `ax.unit_info()`, `info.backup(ax)`, `check.run_check(ax)` | `UnitInfo()`, `Backup(path)`, `RunCheck()` | `info::unit_info(ax)`, `info::backup(ax)`, `run_check(ax)` | `iad_axis_read_unit_info_json`, `iad_axis_backup_json`, `iad_axis_self_test` |
| Power or link loss | `ax.track()`, `ax.check_link()`, `ax.recover()` | `Track()`, `CheckLink()`, `Recover()` | `recovery::track(ax)`, `recovery::check_link(ax)`, `recovery::recover(ax)` | `iad_axis_track`, `iad_axis_check_link`, `iad_axis_recover` |
| Several axes | `Cell.open()`, `servo_all`, `home_all`, `move_all`, `wait_all`, `stop_all` | `Cell.Open()`, `ServoAll`, `HomeAll`, `MoveAll`, `WaitAll`, `StopAll` | `Cell::open()`, `servo_all`, `home_all`, `move_all`, `wait_all`, `stop_all` | `iad_cell_open`, `iad_cell_servo_all`, `iad_cell_home_all`, `iad_cell_move_all`, `iad_cell_wait_all`, `iad_cell_stop_all` |
| Errors | `SoftLimitError` and the rest of the [Errors](#errors) | `SoftLimitException` ... | `SoftLimitError` ... | `IAD_ERR_SOFT_LIMIT` ... and `iad_last_error_message()` |

The C# `Cell` does not have the group monitors and link monitor, group recovery and backup, pause / resume all or
coordinated motion yet (the core has them): see
[../bindings/dotnet/README.md](../bindings/dotnet/README.md#state-of-this-edition).

## Command line

`iai-actuator-driver --port COM3 <command>` (or `python -m iai_actuator_driver`; `--sim` for the simulator, `--json`
for machine-readable output). Every command and option is in [REFERENCE.md](REFERENCE.md#command-line-reference).
