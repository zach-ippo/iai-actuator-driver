# Changelog

Unofficial: not an IAI product. Versions follow PEP 440.

## 1.2.0 - 2026-11-03

A minor release over 1.1.1. The C ABI becomes 1.1 with one addition, `iad_transport_open_serial_options`; ABI 1.0 is
unchanged, so programs built against 1.0 run unchanged, and the bindings accept a 1.0 or a 1.1 library. The core
also carries the internal groundwork for SEL controllers (frames, messages, a link and a simulator): not a customer
API yet, nothing of it in the C ABI, Python or C#, and UNVERIFIED on a controller. The documents are shorter and
fewer.

### Added

- C ABI 1.1: `iad_transport_open_serial_options` opens a serial port with another character framing (7 or 8 data
  bits, parity none / even / odd, 1 or 2 stop bits), with `iad_serial_options`, `iad_serial_options_init` and the
  `IAD_PARITY_*` constants. 8N1 stays the default; the Modbus RTU stacks need 8 data bits. ABI 1.0 is unchanged and
  both bindings still need only 1.0, so they accept a 1.0 or a 1.1 library. A C program built against the 1.2.0
  header asks for ABI 1.1 and so needs a 1.2.0 library or later. 7 data bits, parity and 2 stop bits are UNVERIFIED on
  hardware.
- C++: the same framing as `SerialFraming`, taken by a `SerialTransport` constructor; `reopen()` keeps it.
- C++: the non-volatile write tracker (`NvWriteTracker`) has a public header, `iai_actuator_driver/nonvolatile.hpp`.

### Changed

- Servo on: on a CON unit whose DSSE bit 12 stays 0 with Modbus control on, the servo-blocked message now names the
  cause and the remedy (PMSL OFF, then ON, at rest) instead of "no known cause" (VERIFIED on the PCON-CB).
- Coordinated runs: a late-frame refusal (`LateFrame`) says how far the system clock moved against the steady clock
  during the step, so a clock step shows apart from a slow PC or line.
- Examples: an alarm or warning bit with code 0 prints "warning (no code)" or "alarm (no code)", not "warning
  0x000".
- Actuator kind: a unit without AKID (item 7.33), such as a CON controller, is "linear" when ATYP 7.7 bit 8 says it is
  not rotary, instead of "unknown" (seen on the PCON-CB with an RCP6); the GUI's workbench line leaves out a kind that
  is still unknown. Behaviour change: code that took "unknown" for such a unit now sees "linear"; "unknown" stays for
  a unit whose registers could not be read.
- C ABI documents: `core/abi/json_keys.json` also lists, under "requests", the keys that each JSON request of the
  stable header reads (the coordinated request's `expect_start` among them), and the header comment of
  `iad_axis_session_from_info` names its keys. No function changed.
- Documents: a shorter README and shorter guides, with nothing dropped. `docs/COORDINATED.md` is now the "Coordinated
  motion (alpha)" section of `docs/DRIVER.md`, `docs/FAQ.md` the "Questions" section of `docs/TROUBLESHOOTING.md`,
  `docs/PYTHON_FALLBACK.md` is gone (the pure-Python edition went in 1.1.0), and `ARCHITECTURE.md` and `NATIVE_ABI.md`
  moved to `docs/dev/`. The CHANGELOG starts at 1.1.0.
- Issues: one form, with a "What is this?" choice (a bug, or a result from my unit) and the unit-report fields as
  optional ones. Bugs and questions go to GitHub Issues.
- Releases: no `SHA256SUMS.txt` file among the release files.

## 1.1.1 - 2026-11-03

A maintenance release over 1.1.0: bug fixes only, C ABI 1.0; "seen on the PCON-CB" means seen on a PCON-CB with an
RCP6 in a hardware test on 2026-11-03 (VERIFIED), and "Behaviour change" marks a fix a program may rely on. Three
fixes were checked on that PCON-CB with the 1.1.1 code (VERIFIED: a 1 % override ran an 840 mm/s row at 9.80 mm/s,
1.1.0 13.1; a wait ended 98 ms after another thread's `stop()`, 1.1.0 about 3.1 s; a scan of slaves 57-59 sent
nothing to 58), the others on the simulator only.

### Fixed

- Motion waits: a thread waiting in `move_to(wait)`, `MoveHandle.wait()`, `poll()` or `done()` went on after another
  thread's `stop()` of the same axis, ended about 3 s later with `MotionIncompleteError` and then sent a STOP of its
  own (seen on the PCON-CB). The wait now ends at once with `MotionIncompleteError` ("stopped from outside this
  wait") and sends no second STOP. A wait already at rest at its target returns its status; a pause does not end a
  wait.
- Push waits: `push(wait)` and a stored push row ignored another thread's `stop()` of the same axis, timed out 9-10 s
  later and then sent a STOP of their own (seen on the PCON-CB). They now end at once with `MotionIncompleteError`
  ("push (approach ..., zone ...) stopped from outside this wait"), with no second STOP; a stored row's move wait
  does the same.
- Motion waits: a STOP sent through another `Axis` object of the same unit was not seen as a stop from outside, so the
  wait ran on and sent a STOP of its own. It now ends the wait at once as above, with no second STOP. A STOP from
  another program shows in the status only: a move wait that saw the axis move and then sees it rest short of its
  target with the move cancelled (PEND on, MEND off, not paused, velocity 0) for `settle_s` now ends the same way,
  instead of at its settled-away check (UNVERIFIED: simulator only).
- A superseded move: when a newer move re-targeted a move, the older `MoveHandle.wait()` ran to its own timeout,
  raised `MotionTimeoutError` and sent a STOP that could cut the newer move; a `move_to(wait)` or push wait in another
  thread did the same (seen on the PCON-CB). Such a wait now ends at once with `MotionIncompleteError` ("superseded
  by a newer move of this axis") and sends no STOP. `change_speed` and `change_accel` of the running move do not
  supersede it.
- `change_speed` from another thread: a `move_to(wait)` whose timeout was estimated for 100 mm/s timed out, and
  stopped the axis, after another thread's `change_speed(10)` (seen on the PCON-CB). `change_speed` and
  `change_accel` now re-estimate the remaining time of the running move, and every wait of that move with an
  estimated timeout takes the longer one. A timeout you give stays as given.
- `move_by` after a fast move: 50 x `move_by(-0.02, completion fast)` covered 0.31 mm of 1.00 mm (seen on the
  PCON-CB): the fast rule ends on the first in-band sample, and the next step was added to the measured position,
  which still lagged behind the target. `move_by`, `jog` and `start_move_by` now add to the last commanded target
  while that move is still finishing (the newest move of this object, not a push, no stop since, within its band);
  otherwise they count from the measured position as before.
- Speed override of a stored row (0x45 cmd 6, `goto N --speed-pct P`): the driver sent code = round(255 x % / 100),
  but the unit runs (code + 1) / 256 of the row's speed (measured on the PCON-CB, VERIFIED: code 13 ran an 840 mm/s
  row at 46.0 mm/s, code 3 at 13.1 mm/s), so a row ran faster than asked (1 % ran 1.56 %). Behaviour change: the
  driver now sends code = round(256 x % / 100) - 1 within 1..255, so the unit runs the asked speed to the nearest
  1/256 (25 % exactly; 1 % runs 1.17 %; the least is 0.78 %). The row's speed cap and the timeout use the share the
  unit runs; when that share is more than 5 % off the request the driver logs a warning and the C++, Python and C#
  command lines print a note. The simulator runs (code + 1) / 256 too.
- Slave address 58: a request to slave 58 starts with 0x3A (':'), the start of a Modbus ASCII frame, so every unit on
  the line takes it for one and raises alarm 0x05E "Delimiter error" (seen on the PCON-CB: slaves 33 and 57 no alarm,
  58 alarm 0x05E; a 1-247 scan raised it). Behaviour change: scans (`Port.scan`, `scan_units`) and baud detections
  never address slave 58 and say so in one warning line; a request to slave 58, or an open of it, is refused with a
  clear message and nothing sent.
- Position unit: `--position-unit 0.001` (`LineOptions.position_unit_mm`, `set_position_unit` of `Line` and `Port`,
  the C API) was applied on a direct line (no RCON gateway): on the PCON-CB the status read 94.99 mm as 9.499 mm, and
  a move would have sent its target x 10 (seen on the PCON-CB). Behaviour change: a direct line uses the unit's own
  0.01 mm and refuses any other unit, whether the line was stated or detected (`ValueError`, `std::invalid_argument`,
  `ArgumentException`; nothing sent); the C++ and Python command lines refuse `--position-unit 0.001` there.
- Alarm reset: on the PCON-CB an alarm that switches the servo off leaves the servo-on level (SON coil / DRG1 b12)
  set, and one alarm reset at rest switched the motor back on by itself (seen on the PCON-CB, after a push back
  error 0x0DC). Behaviour change: `reset_alarm()` now clears that level first, on the reset's own path, when the
  status shows an alarm with the servo off, so a servo the alarm switched off stays off until `servo(True)`. A servo
  that is on stays on.
- PC control: with `auto_modbus_control`, a `move_to` refused in PIO mode because the servo was off had first written
  the PMSL coil and switched the unit to PC control (seen on the PCON-CB). Behaviour change: every refusal the status
  can show (control, servo off, homing, not homed, a blocked servo ON, a moving axis, a start inside the push zone, a
  bad home pattern) now runs before control is taken, so a refused command writes nothing, not even PMSL. A command
  that passes takes PC control at rest as before.
- `check_move` is a pure check: in PIO mode with `auto_modbus_control` it took PC control (wrote PMSL) when its checks
  passed. Behaviour change: it never writes (no PMSL, no coil); it runs every refusal on the status and returns
  `modbus_enabled` false, with a log line saying that a move would take PC control first. `move_to` still takes it at
  rest.
- Refusal texts: a software reset, PMSL off, the action mode, a change during a move and `check --motion` said the
  axis was moving or paused ("resume it") when the unit was in PIO mode with the PIO holding the pause, or paused by
  a command (seen on the PCON-CB). The message now names the real cause and the way out.
- Moves: a PCON refused a speed, acceleration or deceleration above its maximum with "this EC's maximum" (seen on the
  PCON-CB). It now says "this unit's maximum".
- Parameters: No.25 (PIO pattern) took 6, 7 or 32767 although the unit's highest pattern (MXPT) reads 5 (seen on the
  PCON-CB). Behaviour change: a value above MXPT is refused, nothing sent, when MXPT reads.
- Parameters: No.25 took -1 (the parameter's range is -1..32767), although no pattern below 0 exists. Behaviour
  change: a value below 0 is refused, nothing sent.
- Parameters: No.3 (soft limit +) below No.4 (soft limit -) was written, and `audit` did not find the pair (seen on
  the PCON-CB). Behaviour change: such a write is refused, nothing sent, against the other limit as the unit holds
  it, and `audit` flags an inverted pair.
- Parameters: setting No.3 (soft limit +) below the axis's position gave no warning, and the unit raises alarm 0x0D9
  on the next move (seen on the PCON-CB). The write now warns.
- Parameters: a `commit=False` write of the value the unit already holds sends nothing, but its note read "RAM ONLY -
  NOT SAVED" (seen on the PCON-CB). It now says the value is unchanged and nothing was written.
- Command lines: `param set KEY VALUE --no-commit` with the value the unit already holds sent nothing, but the C++,
  Python and C# command lines printed the read-back line and RAM ONLY. They now say the value is unchanged and
  nothing was written.
- Push: the warning that the unit switches the servo off by itself always named No.36; stop mode 1 / 2 / 3 (No.53)
  uses No.36 / No.37 / No.38 (seen on the PCON-CB with No.53 = 2 and No.37 = 5 s). It now names the delay No.53
  selects.
- C++ command line: `backup --rows 4294967296` wrapped to row 0 (seen on the PCON-CB). A number beyond the int range is
  now refused, exit 2.
- C++ command line: Ctrl+C before the move frame went out stopped nothing: the tool sent STOP ON and OFF, then still
  sent the 0x9900 move about 1.3 s after Ctrl+C, and the axis ran its full travel (exit 130; seen on the PCON-CB). Once
  Ctrl+C is seen the tool now sends no motion frame: a command not started yet does not start, every later write of
  the command is refused (nothing sent), a move already sent is stopped, and the STOP is held until the command has
  ended and released at exit (the fix is checked on the simulator).
- C++ command line: `move --change-speed` sent its 0x9900 change frame after Ctrl+C, ahead of the STOP (seen on the
  PCON-CB). No change frame goes out after Ctrl+C (the fix is checked on the simulator).
- C++ and Python command lines: Ctrl+Break ended the tool at once (exit 0xC000013A) with no decel STOP, and the move
  ran on to its target (seen on the PCON-CB). Ctrl+Break is now handled as Ctrl+C: the decel STOP, exit 130 (the C#
  command line already did this). The Python command line also exits 130, not 0xC000013A, on Ctrl+C or Ctrl+Break
  before the axis is open (the fix is checked on the simulator).
- C# command line: `check --motion` ignored Ctrl+C: it moved out and back and exited 0 (seen on the PCON-CB). Ctrl+C
  now ends the check with the decel stop, no report, exit 130, as the C++ and Python command lines (the fix is checked
  on the simulator).
- Python and C#: the public `ParameterWriteResult` and its C# record had no `forced`, which the native result has.
  It is added as the last field, with a default, so positional use holds.
- Python: `recover(policy=...)` raised `AttributeError` for a dict (the native binding already took one). A
  `RecoveryPolicy` or a mapping of its fields is accepted; anything else, or an unknown field, is a `TypeError` that
  says so.

## 1.1.0 - 2026-11-03

A minor release. The C ABI stays 1.0.

### Fixed

- On Linux the serial transport took a 0 from `read()` (the port is set VMIN = VTIME = 0, and Linux returns 0 while no
  byte has arrived) for a pulled adapter, so the first read after every request failed with `the port vanished
  (unplugged?)` on a real port, the first status read included, and nothing moved. A read now waits for the reply: a 0
  means nothing yet, and only a 0 right after `poll()` said readable, or POLLHUP / POLLERR, is a vanished port. New
  tests on a pseudo terminal (a late reply is read, nothing sent is a timeout, the other end closed is a vanished port).
  VERIFIED on a PCON-CB with the IAI USB adapter passed into WSL 2 with usbipd (2026-11-03, a build of the fix): the
  harness's Python and C++ command-line editions and the SDK's C example.

### Changed

- The Python package runs on the native core: without a usable native library the import raises `ImportError` saying
  where it looked. The sdist is source that needs the library.
- The `ec_driver` import name is deprecated and will be removed in 2.0: import `iai_actuator_driver`.
- The driver no longer labels units by model: every IAI unit that speaks Modbus RTU on its serial port is driven the
  same way and reported as what it says it is. The "UNVERIFIED for this model" notes and warnings are gone and
  `allow_unverified_model` is accepted and ignored. Names, JSON keys and the C ABI are unchanged; `unit_known` only
  says the driver has a family name, `verified` / `verified_for_model` equal `known`, `seen_on` and the model-list
  constants (`UNIT_CODES_CONFIRMED`, `MODEL_PARAMETERS_SEEN_ON`, `UNITS_SEEN_ON_HARDWARE`, `OPT_IN_NOTE`) are empty.
- Writes to the unit need no opt-in: stored rows, teach, home and dangerous parameters, unlisted parameters and
  unverified baud codes are written; a risky write logs a WARNING and adds a line to the result's warnings. The old
  flags (`allow_nonvolatile`, `allow_dangerous`, `allow_home_params`, `allow_unlisted`, `allow_unverified_code`, and
  `allow_unsafe` on row writes and teach) and their command-line options are accepted and ignored; Python names in
  docs/DRIVER.md, command-line options in docs/REFERENCE.md. What stays: an unchanged value is not written (`force`
  writes it), the hourly write warning, `allow_never_send` and `allow_unsafe` for raw IAI commands, and the range
  checks (push power and load threshold now even with `allow_unsafe`). The GUI asks one OK / Cancel before a permanent
  write.
- A `Cell` takes every axis it finds or is given: no total cap by default (it was 16 axes and 16 ports). `max_axes`
  still caps it when given (Python `Cell.open(max_axes=N)`, C# `CellOptions.MaxAxes`, the C ABI's
  `iad_cell_options.max_axes`, whose default is now 0 = no limit; a negative value is refused). One cable still
  carries at most 16 axes (axis numbers 0-15, addresses 1-16), and the scan asks addresses 1-16 per port by default;
  one axis per USB adapter is fastest, because the axes on one cable take turns. `MAX_PORTS` / `PortManager.MaxPorts`
  stay public at 16 for compatibility and no longer limit anything.
