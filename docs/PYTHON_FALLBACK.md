# The pure-Python fallback (removed)

Up to 1.0.x the Python package also carried a pure-Python edition of its modules (`iai_actuator_driver/_pure/`). It
ran when the core's native library was not found, or when `IAI_DRIVER_BACKEND=python` chose it. It was frozen in 1.0
and has been removed: **1.0.x is the last release with it**. The package now runs on the C++ core only.

## What changed

- Importing the package loads the native library. When it is missing or has another ABI version, the import raises
  `ImportError` saying where it looked. No warning, no fallback.
- `IAI_DRIVER_BACKEND=python` raises a one-line `ImportError` that says so. Unset, `auto` and `native` are the core.
- The sdist is source that needs the native library. A platform wheel carries it.
- `iai_actuator_driver.BACKEND` and `backend_info()["backend"]` are always `"native"`.

## What to do instead

Give the package its library: install a platform wheel, set `IAI_ACTUATOR_DRIVER_NATIVE` to the library file (or its
folder), or build it (`python build.py shared` in `core/`; CMake on Linux). Then unset `IAI_DRIVER_BACKEND`. The search
order is in [getting-started/python.md](getting-started/python.md#where-the-native-library-comes-from).

On a platform with no library, stay on a 1.0.x release.

## Code that ran on the fallback

Module, class and function names, signatures, ranges, refusals and warnings are the same on the core. What differs:

- **Exceptions.** A serial port that does not exist raises `TransportError` (a `ModbusError`), not pyserial's
  `SerialException`. `read_parameter("class.item")` for a class the unit does not list raises `UnsafeCommandError`,
  not `LookupError`. A table index out of range raises `NativeOutOfRangeError`, both a `ValueError` and an
  `IndexError`.
- **Messages.** The core words them: refusals that send nothing end with `; nothing sent`, a timed-out wait adds
  `[decel stop sent after this error]`. Do not match on message text.
- **Results.** `parameters()` rows of an EC carry a `note` key. `read_parameter("class.item")` of a raw item returns
  the keys of a named row. `read_monitor()["raw"]` writes 32-bit items with eight hex digits.
- **Threads.** Monitor and cell callbacks run on a binding thread (`iad-events-...`); a long call made on the main
  thread runs on a helper thread (`iad-call:MainThread`), so Ctrl+C cancels the wait in the core.
- **STOP first and the sinks (1.0.1).** A write of another thread that a STOP (to the same unit, or broadcast)
  overtakes while it waits for the port, is being prepared or is being retried is refused, nothing sent, and no
  command releases a STOP that a stop still holds. The log and frame sinks never run with the port held, so a sink
  that blocks never delays a STOP. The fallback guaranteed neither.
- **Coordinated motion.** The core plans, runs and stops. `run_plan` plans the path again, and
  `CoordinatedGroup.timing()` and `run_plan` of a plan made elsewhere raise `NotImplementedError`
  ([COORDINATED.md](COORDINATED.md#on-the-core)).
- **Recovery.** `RecoveryPolicy.list_ports`, the `lister` of `check_link` and `track(info=...)` are not used (the core
  reads its own port list and the identity), and `Cell.recover_all` ignores the policy's `backup` and `list_ports`.
