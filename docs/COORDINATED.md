# Coordinated motion (alpha)

Lines, arcs and circles over several axes, interpolated by the PC. One real axis ran the lines, circles and stops
below together with a simulated second axis ([Hardware results](#hardware-results)); two or more real axes together
are UNVERIFIED. Function list: [DRIVER.md](DRIVER.md#coordinated-motion).

## What it is

An ELECYLINDER or CON unit has no interpolation of its own and no clock shared with other axes. `iai_actuator_driver.coord`
makes several axes follow one path by commanding them from the PC:

```python
from iai_actuator_driver import Cell, Path

with Cell.open(["COM3", "COM4"]) as cell:
    xy = cell.coordinated(x="COM3:1", y="COM4:1")        # names for the path coordinates
    plan = xy.plan(Path.circle({"x": 60, "y": 40}, 10), speed=10, accel_g=0.1)   # dry run: nothing sent
    print(plan.summary())
    result = xy.run_plan(plan)                           # or xy.circle((60, 40), 10, speed=10)
    print(result.summary(), result.max_deviation)
```

In Python this runs on the core: the core plans, checks, runs and stops; the shape of the C ABI limits a few calls
([On the core](#on-the-core)). The JSON calls of the C ABI are
`coordinated_plan` / `coordinated_run` of `NativeCell` in `iai_actuator_driver.native` ([NATIVE_ABI.md](NATIVE_ABI.md)).

- **Coordinates** are `{axis name: position}` in each axis's units (mm, or deg on a rotary axis). An axis a segment
  does not name holds its position.
- **Segments:** `Line(to)`, `Arc(center, end, ccw=True, plane=("x", "y"))`,
  `Circle(center, radius, start_deg=0, turns=1, ccw=True, plane=...)`, `Polyline(points)`.
- **Demo shapes:** `Path.circle`, `Path.rectangle`, `Path.polygon`.
- **Names:** `iai_actuator_driver.Line` is the gateway line class. The segment is `iai_actuator_driver.coord.Line`, also
  exported as `iai_actuator_driver.LineSegment`.

## How it runs

| Mode | Used for | How |
|---|---|---|
| `"sync"` | lines, polylines | One move per axis per segment, started on every port at once. Axis i gets speed `v * \|d_i\| / L` and acceleration = deceleration `a * \|d_i\| / L`, so every axis runs the same trapezoid in time and the point stays on the line. The next segment starts when every axis is done (with `blend=`, when each is within that distance). |
| `"stream"` | arcs, circles, curves | The path is cut into chords (angle step `2 * acos(1 - e / r)` for chord error `e`). Each waypoint gets a time from a trapezoid speed profile, and at each time every axis is re-targeted one waypoint ahead with the speed that gets it there on time. One thread per port, one start time, 1 ms Windows timer. Status is read about every 20 ms while slack allows, and at least every 0.15 s (longer on a slow line). |
| `"auto"` (default) | anything | sync for lines only, stream otherwise |

A 0x9900 block re-sent during a move re-targets it, and a new speed takes effect mid-move (both VERIFIED on the EC
gripper). About 15 re-targets per second on one EC are VERIFIED; several real axes doing it together are UNVERIFIED.

**Start point.** A path that does not start where the axes are gets a straight lead-in. For a sync path it is one more
segment; for a streamed path it is `plan.lead_in`, run first as its own synchronized line. `xy.circle()` needs no
lead-in when the axes are already on the circle (within 0.5 % of the radius). `xy.line_to()` to where the axes already
are returns a completed result at once (nothing sent). A path whose segments move no axis by a whole count runs as its
lead-in alone, with a warning.

### Checks before anything is sent

The planner checks:

- every commanded waypoint against each axis's range (soft limits minus their 0.30 margin, within a configured
  `stroke`; the start point is not checked);
- every per-axis speed and acceleration against that axis's `limits()`;
- the centripetal acceleration `v^2 / r` against the smallest axis acceleration;
- the step time against what the line can carry.

An infeasible path raises `SoftLimitError` with the reasons and the largest feasible speed (`.max_speed`; a plan at
that speed passes); nothing is sent. A speed below an axis's documented minimum is a warning, never a refusal.
Waypoints that change no axis by a whole count are merged away. A step too short for the line merges with its
neighbour where the path is straight there (or the step is within the chord error).

`run_plan()` checks again against the axes as they are then:

- the ranges, read fresh (a narrowed `stroke` counts; a failed soft-limit read refuses);
- each moving axis's lowest and highest waypoint, the way `move_to()` checks a target;
- every axis of the plan, held ones too: ready (EMG, alarm, servo, homed, not paused, no home return), standing still
  and at the start;
- a streamed plan's steps against the line timing measured then.

An axis in automatic servo OFF is switched back on the documented way just before it gets a frame (not by the 0x9900
frame: that wake is UNVERIFIED on an EC). One coordinated run at a time per axis: a second one is refused (nothing
sent).

### Stops and errors

Any of these stops **every axis of the group** (decel stop) and raises `CoordinatedMotionError`, with the trace so far
in `.result`:

- an error, a fault alarm, EMG, servo off, a pause or a home return;
- a link fault, or a frame later than one full step;
- an outside stop: `xy.stop()`, `cell.stop_all()`, the cell's stop after a link fault, `ax.stop()`, `ax.pause()` or a
  servo OFF from another thread. The run then sends no further frame and never releases a STOP it did not send. This
  holds from the moment `run_plan()` is called.

**Ctrl+C** stops every axis of the group the same way, then re-raises `KeyboardInterrupt` (result in
`.coordinated_result`).

Axes outside the group are not commanded, except on a link fault in a cell: three failed status reads in a row, both
attempts of a frame lost, or the port failing. Then the group's STOP frames go out first, and the cell's
`stop_on_fault` handling stops every other axis, as for `move_all`. One lost echo or a late frame is not a link fault:
the group stops, but the cell's fault handling is not called.

On the core, a stop counts from the moment the core's run is called, its planning included (`run`, `line_to`,
`circle`, `run_plan`: the core checks the plan's start itself, see [On the core](#on-the-core)). `circle` without
`start_deg` reads where the axes are just before that call, to choose its start.

In C++, `CoordinatedGroup::run` (and `run_plan`, `line_to`, `circle`) throws `coord::CoordinatedMotionError` the same
way. A caller that runs it on its own `std::thread` must catch it there: an exception that leaves a thread function
ends the process.

### Timeout

The run's timeout is the wait for the last waypoint; left out, it comes from the plan. In the core (C++
`RunOptions::timeout_s`; `"timeout_s"` of the C ABI's JSON request, so `NativeCell.coordinated_run` and C#
`CoordRunJson` too) it must be a finite number of seconds above 0. Zero, a negative value (-1 is "wait forever" in VB
and LabVIEW) or infinity is refused before anything is sent (`SoftLimitError` in C++, `IAD_ERR_INVALID_ARGUMENT` for a
request). A path that moves no axis is refused the same way. JSON has no NaN: leave the key out for the default. A
stop's timeout is not checked: a STOP always goes out.

### On the core

In Python, `cell.coordinated()` and `CoordinatedGroup({name: axis})` give the core's group
(`iai_actuator_driver.coord.CoordinatedGroup`): `plan()` and every run go to the core (`iad_cell_coord_plan_json` /
`iad_cell_coord_run_json`), on the cell's core cell, or on one opened over the axes for a group built directly. What
differs from the runner of the 1.0.x fallback, from the shape of the C ABI:

- `xy.run_plan(plan)` runs a plan the group made: the core plans the same path again from where the axes are, and they
  must be at the plan's start (its lead-in's, if any; the request's `"expect_start"`), else `EcNotReadyError` and
  nothing is sent; then it runs it, with every check of a run. `result.plan` is the plan that ran. A plan made
  elsewhere (`plan_path`) raises `NotImplementedError`: plan it with `xy.plan()`, or run the path with `xy.run()`.
- `xy.timing()` raises `NotImplementedError`: the core measures the line inside each plan and run; the timing a plan was
  made with is `plan.timing`.
- The run reports when it ends: no samples while it runs. Its status monitors are held from the core's run on; they
  poll while it plans.
- `result.worst_skew_s` and the late frames are measured when a frame's call begins, not when the frame is on the wire.

The core's runner, from Python, is VERIFIED (2026-10-31, a development build of 569e836; 1.0.0 has no
`Cell.coordinated` on the core) with one real axis (the EC gripper) and a simulated second axis for a sync line, a
streamed circle r = 2 mm at 5 mm/s (deviation max 0.10 / mean 0.03 mm, no late frame) and `xy.stop()` mid-circle
(`CoordinatedMotionError`; the gripper stood still, no alarm). The other [hardware results](#hardware-results) below
are those of the 1.0.x fallback's runner; two real axes together are UNVERIFIED.

## Limits of PC-side coordination

- **No shared clock.** Axes meet in time only through the PC sending frames on schedule. Axes on separate ports start
  within a few ms of each other. Axes sharing a port take turns, one transaction apart, and unit replies take tens of
  ms (VERIFIED; up to about 285 ms at 9600 baud). That skew times the speed is a path error;
  `plan.est_timing_error_s` and `plan.est_deviation` estimate it.
- **Frame timing sets the smoothness.** Each axis runs at constant speed between waypoints. The shortest step comes
  from `LineTiming` (baud rate, frame lengths, response delay, bus gap, axes per port, one status read per step,
  x1.5). By the model alone that is about 60 ms at 38400 baud, 44 ms at 115200 and 40 ms at 230400 for one axis per
  port ([measured](#hardware-results): 82 and 53 ms). Below 38400 a stream is effectively ruled out. `plan()`,
  `run()` and `run_plan()` also time status reads of every axis inside the core's plan or run, and the measured time
  counts when it is longer. For shorter steps raise the rate: `Port.set_baudrate(230400)` on a cell port,
  `ax.set_baudrate()` on an axis that owns its port (`ax.enable_fast()` on a cell axis keeps the rate). `min_step_s=`
  overrides the estimate.
- **Status monitors are held.** `run_plan()` holds the status monitors of its own axes and reads its own samples.
  Other axes' monitors on the same ports keep polling; that traffic delays frames and shows as late frames.
- **Chord error vs speed.** A smaller chord error means shorter steps at the same speed. When steps get shorter than
  the line allows, the plan is refused with the largest feasible speed: raise `chord_error` or lower the speed.
- **Lag.** Each axis gets the next waypoint one step ahead. With a low acceleration cap the axis brakes before each
  waypoint and the path cuts inside; the plan warns when the cap bites.
- **The PC is in the loop.** A busy or sleeping PC delays frames. Late frames are counted (`result.late_frames`;
  `result.worst_skew_s` for a sync segment), and a frame that would reach the wire later than one full step is not
  sent and aborts the run. A frame whose echo is lost is re-sent once; with steps shorter than that re-send the run
  stops as a late frame. On Linux, 1.0.0 also lost frames to a backward step of the system clock during a run (an NTP
  step, a VM's time sync: a WSL guest was seen stepping its clock back by up to about 1 s every 32 s): its waits took
  their deadline from that clock, and the stream stopped with `LateFrame`. Since 1.0.1 every wait of the core runs on
  the monotonic clock. A lost status reply is
  skipped; three in a row are a link fault. The unit has no watchdog: if the PC or the cable fails mid-path, each axis
  completes the waypoint it was last sent.
- **The deviation is an estimate.** `result.max_deviation` and `mean_deviation` come from status samples, interpolated
  per axis. They are blind between samples (`result.sample_spacing_s` is the median spacing).

## Hardware results

VERIFIED with the runner of the 1.0.x fallback ([PYTHON_FALLBACK.md](PYTHON_FALLBACK.md)), one real axis (EC-GRBP8M-20
gripper, 9.5 mm stroke) as x and a simulated gripper on a second port as y. Two or more real axes together are
UNVERIFIED.

| Check | 38400 baud | 230400 baud |
|---|---|---|
| Shortest step measured | 82 ms (model: 60 ms) | 53 ms (model: 40 ms) |
| Streamed circle r = 3 mm, 10 mm/s, 0.1 G | deviation max 0.17 / mean 0.08 mm; 0 late frames | max 0.14 / mean 0.06 mm; 0 late |
| Fastest feasible circle r = 3 mm, 0.3 G | 12.2 mm/s: max 0.24 / mean 0.09 mm; 0 late | 18.3 mm/s: max 0.34 / mean 0.12 mm; 0 late |

- A sync line at 5 mm/s landed on its target.
- A circle faster than the line allows was refused before any frame, with the largest feasible speed.
- `xy.stop()` from another thread and `pause()` of the real axis mid-circle ended the run with
  `CoordinatedMotionError` within 0.3-0.4 s; the gripper stood still, with no alarm.
- Deviations are estimates from status samples, and half the path is the simulated axis.

Run again on 2026-10-31 with the released 1.0.0 wheel, on its fallback only, at 38400 baud, at most 10 mm/s and 0.1 G, x
inside 2.79 to 6.75 mm: the shortest step was 81 ms; a line from 3.00 to 6.75 mm at 5 mm/s landed; a circle of radius
2 mm at 5 mm/s took 2.73 s (deviation at most 0.10 mm, mean 0.05, 0 late frames) and at 9.5 mm/s 1.54 s (at most
0.17 mm, mean 0.07, 0 late); a line back landed at 3.00 mm. At 10 mm/s the planner refused the circle before any frame:
one step of y would pass the cap. In 1.0.0 the core had no `Cell.coordinated`; the core's runner was run later that day
on a development build: see [On the core](#on-the-core).

On the simulator with replies 30 ms late, a fault alarm on one axis mid-circle put the other axis's STOP on the wire
about 0.15 s later. With 10 ms replies, a dead link on one axis stopped the other within about 0.2 s.

## Demo

```shell
python examples/python/draw_circle.py --sim                              # two simulated axes
python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --dry-run    # plan only (reads positions and ranges)
python examples/python/draw_circle.py --x COM3:1 --y COM4:1 --radius 5 --speed 5
```

On hardware: servo on and home both axes first. Start small and slow, unloaded and mid-stroke, with a hand on the
emergency stop.
