# Hardware acceptance harness

One command that runs the same checks on any IAI unit the driver supports (ELECYLINDER, CON-family controllers), on
the simulator or on a real serial port. It prints one `PASS` / `CHECK` / `FAIL` / `SKIP` line per step and writes a
JSON and a markdown summary, so someone else can repeat a check on another day.

- It tests the installed `iai_actuator_driver` package by default, so it can test a released wheel. The C++ and C#
  command lines can be tested the same way.
- The checks are the ones run by hand on an EC-GRBP8M gripper; the results of the harness on that gripper are in
  [COMPATIBILITY.md](../../docs/COMPATIBILITY.md#hardware-results). On any other unit its behaviour stays UNVERIFIED
  until a first run there.
- Unofficial: not an IAI product.

## Quick start

From the repository root (Python 3.11 or newer; the driver installed, or point `--src` at a source tree):

```shell
python tools/hw/run.py --sim                       # rehearsal on the simulator: no serial port, nothing to harm
python tools/hw/run.py --port COM3                 # a real unit: asks for Enter before the axis moves
python tools/hw/run.py --port COM3 --yes           # no prompt (needed when there is no terminal)
python -m tools.hw --list-steps                    # the steps and what each checks
```

Before a run on a real unit:

- Close IA-OS and anything else that holds the port: only one program can own a serial port.
- Put the axis where it can move freely over the middle of its stroke, unloaded, with a hand on the emergency stop.
- A vertical axis can fall when its servo goes off (the `servo` step): leave that step out with `--skip servo` unless
  the axis holds its load.

To test a released wheel on a PC without the source, run
`pip install iai_actuator_driver-<version>-py3-none-win_amd64.whl`, then this folder's `run.py` (no `--src`). To test
a source tree, pass `--src <repository root>` (or its `bindings/python` folder) and `--native <folder>` with the core
library you built (`python build.py shared` in `core/`).

## Options

| Option | Meaning |
|---|---|
| `--port NAME` / `--sim` | The serial port, or the simulator (no port is opened). One of them is required. |
| `--slave N` | Modbus address of the unit (axis number + 1), default 1. |
| `--edition python\|csharp\|cli` | `python`: the Python package (default). `cli`: the C++ command line. `csharp`: the C# command line. The command lines are run as programs; when one is not found the run ends with one SKIP line. `--cli-exe PATH` names it (a `.dll` is run with `dotnet`); otherwise it is looked for next to `--native`, in the build folders of this checkout and on `PATH`. |
| `--speed-cap MM_S` | The fastest speed the run commands. Default 20, at most 100. The driver also lowers it to the unit's own maximum. |
| `--stroke-band BAND` | Where the run may move: a percentage of the stroke around its middle (default `60`, allowed 10 to 90) or `LOW:HIGH` in mm (cut to the unit's stroke, never wider). |
| `--steps LIST` / `--skip LIST` | Which steps run. `default` is the safe motion set, `all` adds the opt-in step; `default,recover` adds one. The order is fixed. |
| `--results DIR` | Where the results go. Each run makes a time-stamped folder in it. Default `./hw-results`. |
| `--src DIR` / `--native DIR` / `--backend` | Test a source tree instead of the installed package / the native core library to load / `auto` or `native` (both the core: the pure-Python fallback was removed after 1.0.x). |
| `--yes` | Do not wait for Enter. A run on a real port needs this or a terminal; with neither it ends before the port is opened. |
| `--sim-profile`, `--sim-baud`, `--sim-time-scale` | `--sim` only: the simulated unit (`gripper` by default; `slider`, `rod`, `pcon`, `rotary`), its baud rate, its speed. The rod, rotary and unknown profiles start at 0, outside the default band: add `--stroke-band 0:10000`. |
| `--sim-scenario NAME` | `--sim` only, to rehearse the safety paths: `alarm`, `warning`, `other-warning`, `cold`, `outside`, `pio-input`, `obstacle`, `fault`, `interrupt`. |

On a rotary unit read mm as degrees everywhere.

A unit parked outside the default band (for example at its home end, 0 mm) gets only the steps that do not move it,
because the run could not put it back where it was found. Give a band that includes the start position, and the run
moves in the middle of that band and puts the axis back (`--stroke-band 0:10000` is the whole stroke), or move the
axis into the band by hand.

## What the run does, in order

1. **open**: opens the port (the baud rate is found in place) and identifies the unit. Reads only.
2. **backup**: a read-only backup of the unit (identity, every parameter, the position table, the alarm history) into
   the results folder: `backup_start.json`, `unit_info_start.json`, `start_state.json`. If the backup cannot be
   written, nothing else runs.
3. **preflight**: records the start state (position, servo, home, PC mode, safety speed, baud rate) and works out the
   test band from the unit's stroke. A unit with an alarm, an active emergency stop or a moving axis is not touched
   (exit 2). A unit outside the band, or with no known stroke, only gets the steps that do not move it.
4. **prepare**: after the prompt, switches PC control and the servo on if the unit was found without them.
5. the selected **steps**, below.
6. **end**: on every path, also after a failed step or Ctrl+C, a move in progress is stopped, fast mode is switched
   off, PC mode is taken back, the axis goes back to its start position and the servo goes back to how it was found.
   Then the state is checked and a second read-only backup is compared with the first.

## Safety rules (built in, not options)

- A read-only backup is made before anything changes; no backup, no run.
- No parameter, position row or baud rate is written; the calls that would write are refused inside the harness. The
  one exception is a single same-value write (`nv_guard`) that the driver's non-volatile guard must skip; a write cycle
  used there stops the run. The end check reads the driver's write counter and compares the parameters (all but the
  live monitor classes 3 and 4) and every position row with the start backup.
- Every move stays inside the test band, at most at the speed cap and 0.1 G, and is checked by the driver before it
  is sent. The unit is put back at its start position.
- PC mode is taken back on every exit path. The unit is not handed to PIO while any PIO input is on, or when the input
  word cannot be read.
- The harness never clears an alarm and never starts with one, except the message-level alarm 0x069 (the calendar
  clock stopped) that a unit with its calendar on shows after a power-up (VERIFIED on the PCON-CB): that one is the
  unit's normal state; it is recorded and the end is compared against it (gone at the end, cleared by an alarm reset
  such as recover() sends, is as found with a note: CHECK). Any other start alarm, also of the message level, is
  refused.
- A run on a real port with no terminal and no `--yes` ends before the port is opened. No option skips the backup, the
  band or the end sequence.

## Steps of the Python edition

Opt-in steps run only when named (`--steps all` or `--steps default,recover`). Steps marked M move the axis or change
its servo or mode.

| Step | | What it checks |
|---|---|---|
| `unit_info` | | the unit's own description has every section and no read errors |
| `status` | | the status block reads and shows no alarm |
| `limits` | | the speed cap, the acceleration cap and the stroke band are the ones requested |
| `tables` | | the first position rows and every parameter decode (read only) |
| `home` | M | a unit that is not homed homes at its own speed; a homed unit is left alone (SKIP) |
| `servo` | M | servo off and on again: homed, no alarm, the axis stays where it was |
| `move_to` | M | an absolute move arrives within 0.1 mm |
| `move_by` | M | a relative move arrives within 0.1 mm |
| `jog` | M | small steps out and back |
| `refusals` | | a speed above the cap, a target outside the band and an acceleration above the cap are refused, with no motion |
| `fast` | M | fast completion, the monitor's change and done events, fast mode off again |
| `handle` | M | `move_to(wait=False)` returns a handle that reports done and the result |
| `cancel` | M | cancelling a slow move stops it short of its target without an alarm |
| `stop` | M | `stop()` ends a slow move short of its target without an alarm |
| `pause` | M | a paused move holds its position and finishes after `resume()` |
| `speed` | M | the axis speeds up when the speed is raised during a move |
| `decel` | M | a move with its own deceleration arrives |
| `pattern` | M | the s_motion move runs, or the driver refuses it before sending when the unit does not list it |
| `push` | M | a push into free air ends as a miss (the axis needs free travel for up to 2 mm above the middle of the band) |
| `nv_guard` | | a parameter written its own value is skipped: nothing written, no write cycle used |
| `baud` | | the baud rate found on the line is the one the run started at |
| `link` | | a recovery session can be taken and the link reads ok |
| `pc_pio` | M | PIO mode, PC mode, Modbus control off and on; the axis stays still in PIO mode and moves again in PC mode (a unit that shows a pause in PIO mode and whose servo went off by the first status in PC mode gets it back on first, also when the first status still showed it on; any other servo loss across the toggle, a later one too, fails the step). The Python edition tries a servo ON straight after `set_action_mode(PC)`, before the PMSL toggle, when the servo is off then: a servo ON that is not taken fails the step. That servo ON switches the servo on in the middle of the step, also on a unit found with its servo off; the restore at the end of the run switches such a unit's servo off again |
| `check` | | the driver's read-only self-test has no failing line |
| `recover` | M, opt-in | a 250 ms line break; the unit falls silent, `recover()` finds it again, restores the baud rate and keeps PC control. It changes the baud rate of the line and back, so it is not in the default set. On the simulator use `--sim-baud 115200` |

The mid-move steps (`cancel`, `stop`, `pause`, `speed`) need a band at least 4 mm wide and use 3 mm/s, which is below
the minimum speed some series document; the driver says so in a warning, which the run prints and keeps in the
summary.

### Command-line editions

The command-line editions run the matching commands: `status`, `limits`, `info`, `home`, `move`, `jog`,
`move --decel`, `move --accel-pattern` (C++ only), `move --change-speed`, `move --fast`, `move --no-wait` then `stop`,
`push`, `param list`, `monitor --watch`, `check`, `recover --backup`, `mode pio` and `mode pc`.

- They run only commands that read, move or switch servo and mode; `param set`, `teach`, `swreset`, `baud` and
  `alarm-reset` are refused inside the harness.
- Their `speed` and `fast` steps check only that the move arrives (the step text says "ARRIVAL ONLY"). They do not
  measure the speed change, the fast-completion path or the monitor's events, so they would pass if `--change-speed`
  or `--fast` were ignored. List them as "arrival only", not as verified.
- The simulator of a command line ends with its process. The C++ one keeps the position between runs. The C# one keeps
  nothing: steps that start where the one before ended are SKIPped, and the unit stays where the profile starts it, so
  use `--sim-profile slider` for it. No simulator can show a move being stopped from a second process (`stop` is
  SKIPped there).

## Interactive extras

Both are separate subcommands with their own `--help`, and run with `--sim` too.

### `power-cycle`

You switch the unit's 24 V supply off and on by hand.

```shell
python tools/hw/run.py power-cycle --port COM3
```

1. The unit must start clean: servo on, homed, PC mode, no alarm, at rest, link at 38400 baud (the rate a unit comes
   up at).
2. The harness reads and backs it up, takes a recovery session (`track`) and prints
   `READY: switch the 24 V OFF now, wait about 5 s, then ON`.
3. It watches `check_link()` with time stamps, then runs `recover()` with a policy that writes nothing non-volatile.
   A recovery that finds another unit, a fault alarm or changed rows or parameters stops before any servo, home or
   move.
4. Otherwise the servo goes on, the unit homes only if it lost its home, and it goes back to the start position at
   `--return-speed` (at most 5 mm/s). The end state is checked.

A unit that lost its 24 V comes up in PIO mode with the servo off. On any non-zero exit the harness prints where the
unit was last seen and the command that takes PC control back: `python -m iai_actuator_driver --port COM3 mode pc`.

Exit codes: 0 ok, 2 refused, 3 no power loss seen (`--loss-timeout`), 4 the power did not come back
(`--back-timeout`), 5 recovery failed or found something wrong, 6 a later step failed or the end state is not as
expected, 130 Ctrl+C.

### `pio-watch`

You operate what is wired to the PIO connector.

```shell
python tools/hw/run.py pio-watch --port COM3 --seconds 60 [--stop-on-move]
```

- The harness reads and prints the PIO input and output words and the stored position rows (a PIO move runs those
  rows at their own speed, not at the speed cap), asks for Enter and switches to PIO mode. The switch is refused if
  any PIO input is on.
- For `--seconds` it prints every change of the PIO inputs and outputs, the move and position-complete flags and the
  position (`samples.csv` keeps every poll).
- At the end, on Ctrl+C or on any error it waits for the axis to be at rest, takes PC control back and confirms it
  with a status read.
- With `--stop-on-move` the first move seen in PIO mode gets one driver STOP and the result is recorded. A push row's
  move that comes to rest at the row's approach position, short of its push end, with MEND 0 while the start input
  is still held counts as stopped; a move at rest on a stored target with MEND 1 had finished (INCONCLUSIVE).
  `--sim-scenario push` / `push-done` rehearse both.

Exit codes: 0 ok, 2 refused, 6 PC mode not confirmed back, 130 Ctrl+C.

## Results

Each run writes a folder `<--results>/<date>-<time>` (with `-sim` after a simulator run) holding:

- `summary.md`: the result, the set-up and one table of steps, for people;
- `summary.json`: the same with the start and end state, the backup comparison, driver messages and each step's data;
- `run.log`: everything that was printed;
- `backup_start.json`, `backup_end.json`, `unit_info_start.json`, `start_state.json`: the read-only backups and the
  start state (they hold the unit's serial number: keep them with the unit's records);
- extras: `recovery` and timeline data in `summary.json` (power-cycle); `samples.csv`, `events.json` and
  `pio_words_start.json` (pio-watch).

| Verdict | Meaning |
|---|---|
| `PASS` | as expected |
| `CHECK` | not a failure, but a person should look (a push that met resistance, a partial backup, a step the driver refused as not offered by this unit) |
| `FAIL` | not as expected |
| `SKIP` | does not apply, was not selected or could not be run (the reason is in the line) |

A failed step that moves the axis stops the later steps; the end sequence still runs. Exit codes of the main run: 0 no
failure, 1 at least one failed step, 2 refused or could not start (the unit was not changed), 130 Ctrl+C.

## Tests

The harness is tested on the simulator (no port). From the repository root:

```shell
python -m unittest discover -s tools/hw/tests -t .
```

The driver comes from `bindings/python` of the checkout, and the native core from `IAI_ACTUATOR_DRIVER_NATIVE` when
that is set (else `core/build`). The driver needs the core: the pure-Python fallback was removed after 1.0.x.
