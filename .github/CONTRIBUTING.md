# Contributing to IAI Actuator Driver

How to build, test and change the driver, the command line and the GUI. To use the driver, start with the
[README](../README.md). The project is unofficial: it is not an IAI product. Contributions are accepted under the same
licence as the project, the Apache License 2.0 ([LICENSE](../LICENSE)).

## Set up

You need Python 3.11 or newer. From the project folder:

```sh
python -m venv .venv
.venv\Scripts\activate                 # on Linux: source .venv/bin/activate
python -m pip install -e bindings/python -e apps/gui
pip install ziglang                    # then, in core/: python build.py shared
```

The editable install adds pyserial (the only dependency) and the commands `iai-actuator-driver`, `iai-check` and
`iai-gui`, all running from the source tree. The Python package runs on the C++ core: it loads the library from
`core/build/` (or `IAI_ACTUATOR_DRIVER_NATIVE`) and does not import without it.

## Run the tests

From the project root, one command builds what the tests need (the core with zig, the C# library) and runs the suites
in parallel:

```sh
python tools/test.py                    # the suites the changes since the merge base with main need
python tools/test.py --all              # every suite
python tools/test.py --suites core,gui  # these suites (core, smoke, python, gui, dotnet, docs, hw, release)
```

- **Per change:** `python tools/test.py` (`--changed BASE` compares with another commit). The path map at the top of
  `tools/test.py` picks the suites; a path it does not name selects every suite.
- **Before merging to main:** `python tools/test.py --all`, and the suites on Linux once (CMake and gcc, the
  sanitizer builds of the C++ tests, the Python and .NET suites against the `.so`).
- **A change to the library's file name or SONAME:** run the two release steps that build and test the Linux library
  and its SDK. They need WSL 2 and the Linux test kit (`--wsl-kit` in `--help`):

  ```sh
  python .github/release/local_release.py --only linux-native,linux-sdk-smoke
  ```

  `--only` continues the output folder of an earlier run of the same commit. With none, run the script without it.
- **A release:** `python .github/release/local_release.py` (see [Release](#release)).

`-j N` sets the number of worker processes. A few tests measure short delays, such as a STOP within 50 ms, so run the
suites when nothing heavy runs beside them. Those whose subject is throughput or real-time latency are listed in
`tools/load_sensitive.txt`: the runners keep them out of their parallel workers, run them afterwards one after another,
and run a failed one once more. A test that only needs a move to be under way, or a window to be seen, uses a stepped
simulator clock (`step_s`) or the virtual clock (`testing.virtual_clock`) instead. Each suite also runs alone:

| Suite | Command | Folder |
|---|---|---|
| Core | `python build.py test` (`--shards N`), `python build.py smoke` | `core/` |
| Python | `python tests/run_backend.py` (`--jobs N`) | `bindings/python/` |
| GUI | `python tools/unittest_jobs.py -s apps/gui -t apps` (`--jobs N`) | the root |
| .NET | `dotnet test` | `bindings/dotnet/` |
| Release scripts | `python .github/release/test_local_release.py` | the root |

The Python package runs on the core only, so its tests need the native library (`core/build/`, or
`IAI_ACTUATOR_DRIVER_NATIVE`). The pure-Python edition was removed after 1.0.x, its last release.

The tests never open a serial port, and a new test must not either. Use the simulator (`SimulatedEC`, `sim_line()`,
`sim_ports()`) and the helpers in `bindings/python/tests/ec_testutil.py`.

[CI](workflows/ci.yml) does not run on a push (GitHub Actions minutes are limited): run the tests above before
you push. It runs on a pull request and by hand (Actions, CI, Run workflow). Both start one small job, the core built on
Windows with zig and then the driver tests on the core with Python 3.13, and the full set on Windows and Linux with
Python 3.11 and 3.13, one job per changed area (a manual run: every job). When every job that ran has passed, CI builds the sdists and wheels as the `dist` artifact.

## Try a change without hardware

- Command line: `--sim` in place of `--port COM3`. `--sim-profile` picks the unit: `slider` (the default), `gripper`,
  `rod`, `pcon`, `rotary` or `unknown`.
- GUI: `iai-gui --sim-only` serves the GUI at <http://127.0.0.1:8780> with simulators only, no serial port.
- Python: `Axis.open(SimulatedEC(profile="gripper"))`, `Cell.open(sim_ports(3))` for three ports, `sim_line()` for
  several units on one line.

The simulator answers Modbus RTU byte for byte and mimics what the hardware tests showed
([docs/REFERENCE.md](../docs/REFERENCE.md#simulator)). Passing on it is not proof that every unit behaves the same way.

## Layout

| Path | Contents |
|---|---|
| `core/` | The C++ core and its C ABI ([core/README.md](../core/README.md), [core/DESIGN.md](../core/DESIGN.md)) |
| `bindings/python/` | The Python package `iai_actuator_driver` and its tests |
| `bindings/dotnet/` | The C# library, its command line and tests ([bindings/dotnet/README.md](../bindings/dotnet/README.md)) |
| `apps/gui/` | The GUI: `server.py` (HTTP, loopback only), `workbench.py` (page `/`), `controller.py` (page `/classic`), `test_gui.py` |
| `examples/` | Short programs, one task each, runnable with `--sim` ([examples/README.md](../examples/README.md)) |
| `docs/` | User and maintainer documents |
| `tools/` | `test.py`, the parallel unittest runner and the hardware harness ([tools/hw](../tools/hw/README.md)) |
| `licenses/` | `NOTICE` and `NATIVE_THIRD_PARTY.txt`: the sources of the copies in `bindings/python/` (both) and `apps/gui/` (`NOTICE`), packed into every release asset; `LICENSE` stays at the root |
| `.github/` | CI, the release workflow and the release scripts, the issue forms and `CONTRIBUTING.md` |

In `bindings/python/iai_actuator_driver/`:

| Module | Contents |
|---|---|
| `modbus.py`, `registers.py` | Modbus RTU master and the IAI 0x45 / 0x48 commands; the register map, each entry with its evidence word |
| `ec.py` | `Axis` (also named `Elecylinder`): every axis command and the non-volatile write guard |
| `cell.py`, `gateway.py` | Port discovery, address scan and `Cell`; line rules (direct, RCON, ROBONET) and gateway status |
| `info.py`, `con_params.py` | Unit information, parameters and backup; the CON-family parameter numbers |
| `alarms.py`, `errors.py` | Alarm names and levels; the exception hierarchy |
| `monitor.py`, `recovery.py` | Status monitor and fast completion; power and link loss (`recover()` never moves the axis) |
| `values.py`, `_protocol.py` | The records the driver returns (`Status`, `DeviceInfo` ...); CRC, frames and codes |
| `sim.py`, `sim_fixtures.py`, `sim_line.py` | The simulator, its register data, several simulated units on one line |
| `check.py`, `cli.py` | The `iai-check` self-test and the `iai-actuator-driver` command line |
| `native.py`, `_backend.py` | The ctypes binding of the C ABI; the library loaded at import (`backend_info()`) |
| `_native/` | The log bridge, the ABI's table calls and the folder of the library a platform wheel carries |
| `_shared/` | The Python parts of the modules: records, constants, the coordinated-motion planner, `info`, `check` |

Every module above is a binding over the shared C++ core: the core decides, the module converts arguments and results.
`bindings/python/tests/test_packaging.py` checks the version, the entry points, the links and the wording.

## Rules

### Evidence words

Every statement about how a unit behaves (in code, comments, docstrings, documents and commit messages) carries one of
these words:

| Word | Meaning |
|---|---|
| **VERIFIED** | Tried on real hardware with this driver |
| **ON-WIRE** | Seen in observed serial traffic; the unit's reaction is VERIFIED only where also tested |
| **DOC** | Written in a public IAI manual; give the manual and the page, for example `DOC ME0162-12A p.4-2` |
| **UNVERIFIED** | Expected, never tried |

A word changes only with the test or the manual behind it. Name a unit by its model (the EC-DS4H slider, the EC-GRBP8M
gripper, the EC-R6S rod, the EC-S6HH slider, the PCON-CB with an RCP6). Add new hardware results, with the date, to
[docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md#hardware-results). For a new unit, the `iai-check` report is the most
useful result: it goes into a unit report on
[GitHub Issues](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose)
([Sending a report](../docs/COMPATIBILITY.md#sending-a-report)). Lab notes, serial captures and unit backups stay out of
the repository; `.gitignore` covers `backups/` and capture files.

### One code path for every unit

The driver adapts to whatever unit is connected. Ranges and capabilities are read from the unit itself.

- Do not add a model allow-list.
- Do not refuse or label a unit by its model, unit code or kind: drive it the same way and report it as what it says
  it is. UNVERIFIED describes a feature or path, never a unit.
- Limits passed by the caller can only narrow the unit's own. Every move is checked against them before it is sent.

### No opt-ins for writes

The users are experienced: a write to the unit needs no flag. Stored rows, teaching, parameters (home direction and
offset, motor and encoder type, lead, gains, PIO, fieldbus and baud setup, a wider soft limit, a parameter the unit's
list does not show) and baud codes not VERIFIED are written; a risky one logs a WARNING and adds a note to the
result. The old flags (`allow_nonvolatile`, `allow_dangerous`, `allow_home_params`, `allow_unlisted`,
`allow_unverified_code`, and `allow_unsafe` on row writes and teach) are accepted and ignored, and their CLI options
still parse but are hidden from the help.

What stays:

- The never-send list of raw IAI commands (firmware update, parameter initialisation, alarm-list clear, axis number,
  ...): `allow_never_send=True` only. IAI 0x45 / 0x48 commands outside the allow-list need `allow_unsafe=True`.
- Range checks: a value outside the unit's range (stroke, speed, push power, load threshold) is refused.
- Read-only, unconfirmed and not-offered parameters, the simple-data rows 256-267 and rows the table cannot address.
- The non-volatile write guard below, and one plain OK / Cancel in the GUI before a permanent write.

Never add a permission flag for a write, a read, a routine command or one model; a risky write gets a warning.

### Non-volatile memory

The units' non-volatile memory lasts about 100,000 write cycles (IAI). A parameter commit, a stored-row write and
teaching write it; direct moves (`move_to`, the 0x9900 block) are RAM only (VERIFIED). Every path that writes
non-volatile memory follows the guard of the core (`core/src/axis_nv.cpp`):

- It writes nothing when the value or the row would not change, and reports `skipped`. `force=True` writes anyway.
- It counts each write (`ax.nonvolatile_writes`, `ax.nonvolatile_writes_last_hour`)
  and logs one WARNING per hour while more than `NV_WARN_PER_HOUR` (60) writes fall in the last hour on one axis.
- It never refuses a write for the rate: the driver warns, the integrator decides.

A new write path gets its test in `bindings/python/tests/test_nv_guard.py`.

### Hardware tests

The test suites never touch hardware. Runs on a real unit are done by hand:

- Close IA-OS, and anything else that holds the COM port, first.
- Start read-only: `iai-check --port COM3`, then `iai-actuator-driver --port COM3 status` and `limits`.
- Start slow and short: unloaded, mid-stroke, at 20 mm/s or less, with a hand on the emergency stop. A vertical axis
  can fall when its servo goes off.
- Never pull a locking cable.
- Back up the unit before any non-volatile write (`iai-actuator-driver --port COM3 backup unit1.json`, read-only) and
  restore the original values afterwards. Keep such writes few: each one uses a write cycle.

[tools/hw](../tools/hw/README.md) runs these checks: a read-only backup, short moves mid-stroke at a capped speed, then
the unit is put back as found. Rehearse with `--sim` first.

## Code style

- Follow the module you edit: `from __future__ import annotations`, type hints, and docstrings like those in
  `ec.py` (what it does, units and ranges, what is refused, the evidence word for each hardware claim).
- Command-line options use argparse as in `cli.py`: a help text with the range or the default, and a note when a
  command is read-only.
- The only runtime dependency is pyserial, imported only when a real port is used. The GUI uses the standard library
  only; the tests are plain `unittest`.
- Whitespace follows [.editorconfig](../.editorconfig): 4 spaces in Python, 2 in JavaScript, CSS and HTML.
- Text that ships (code, comments, documents, commit messages) is plain and factual, in short sentences.

## Documentation

- A new public function or command gets its line in [docs/DRIVER.md](../docs/DRIVER.md): inputs, ranges, an example.
- Behaviour notes and evidence go to [docs/REFERENCE.md](../docs/REFERENCE.md), product support to
  [docs/COMPATIBILITY.md](../docs/COMPATIBILITY.md).
- A new error message or warning gets a row in [docs/TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md), quoted as printed.
- [README.md](../README.md) stays a short front page: `test_packaging.py` keeps it under 160 lines with a quick start of
  at most 10 lines, and checks that every relative link in the documents resolves.
- Every change a user can notice gets a line in [CHANGELOG.md](../CHANGELOG.md) under the newest version.

## Release

One version for the whole repository.

1. Set the version (PEP 440, for example `1.1.0b1` for a pre-release) in `bindings/python/pyproject.toml`,
   `apps/gui/pyproject.toml` (its version and its `iai-actuator-driver==` pin),
   `bindings/python/iai_actuator_driver/__init__.py`, `core/include/iai_actuator_driver/version.hpp` (`kVersion`),
   `core/CMakeLists.txt` (`project(... VERSION X.Y.Z)`, without the pre-release part) and the `<Version>` of the C#
   projects in `bindings/dotnet/src/` (NuGet form, for example `1.1.0-beta.1`).
2. Add a `## <version> - <date>` section at the top of `CHANGELOG.md`. The release notes are taken from it.
3. Update the version in `README.md` (the intro and the file names under Install) and the expected version in
   `bindings/python/tests/test_packaging.py`.
4. Run `python .github/release/check_version.py` (it fails on a mismatch), then the test suites.
5. Run `python .github/release/local_release.py`. It builds the wheels and sdist, the NuGet package, the C/C++ SDK zips
   and `iai-gui.exe` for win-x64 and linux-x64, tests them on the simulator and prints the `gh release create` command
   that creates the tag `v<version>` and attaches them to the GitHub release; it never runs `gh`, pushes or tags.
   Releases are not built by GitHub: a pushed tag starts no workflow, and `.github/workflows/release.yml` runs by hand
   only (Actions, Release, Run workflow), a dry run that publishes nothing.

A local build of the Python packages (no native library inside: the driver then needs one named to import):

```sh
python -m pip install build
python -m build bindings/python --outdir dist
python -m build apps/gui --outdir dist
```
