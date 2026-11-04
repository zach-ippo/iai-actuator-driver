# Frozen C ABI 1.0

Two files that are never edited again:

- `iai_actuator_driver.h` - the stable header of C ABI 1.0, byte for byte as released in 1.0.0.
- `c_smoke.c` - the C smoke program of 1.0 (plain C99, simulator only), which includes that header and no other.

Together they answer one question: does a C program built against the 1.0 header still run on a later 1.x library? The
program is built against this header only (never `core/include/`) and run on the current library by

- `python build.py smoke` in `core/` (the `smoke` suite of `python tools/test.py`), and
- both SDK smoke steps of the local release (`.github/release/local_release.py`, `local_release_wsl.sh`) and the SDK
  step of `release.yml`, through `.github/release/sdk_consumer` (the test `c_smoke_1_0`).

A test (`bindings/python/tests/test_abi_frozen_1_0.py`) pins the sha256 of both files. If the program fails against a new
library, the library broke the 1.0 contract (`docs/dev/NATIVE_ABI.md`): fix the library, not these files. The live
program and header (`core/tests/c_smoke.c`, `core/include/iai_actuator_driver.h`) follow the current ABI as before.
