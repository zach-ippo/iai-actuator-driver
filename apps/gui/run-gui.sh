#!/bin/sh
# Starts the iai-gui web GUI from this folder (Linux, macOS):  ./run-gui.sh
# The first run creates .venv here and installs the package into it (needs internet for pyserial).
# Arguments go to iai-gui, for example:  ./run-gui.sh --sim-only   (demo: simulators only, no serial port)
# Serial ports on Linux: add your user to the dialout group once (sudo usermod -aG dialout "$USER"; uucp on
# some distributions), then log out and in again.

fail() {
    echo "$1" >&2
    if [ -t 0 ]; then
        printf 'Press Enter to close. '
        read -r _
    fi
    exit 1
}

cd "$(dirname "$0")" || fail "Cannot change to the folder of run-gui.sh."

if [ ! -x .venv/bin/python ]; then
    PY=""
    for candidate in python3 python; do
        if command -v "$candidate" >/dev/null 2>&1; then
            PY="$candidate"
            break
        fi
    done
    [ -n "$PY" ] || fail "Python was not found. Install Python 3.11 or newer (python3), then run ./run-gui.sh again."
    if ! "$PY" -c 'import sys; raise SystemExit(sys.version_info < (3, 11))'; then
        fail "This needs Python 3.11 or newer. Found: $("$PY" --version 2>&1). Install a newer Python and run again."
    fi
    echo "Creating .venv with $("$PY" --version 2>&1)"
    "$PY" -m venv .venv || fail "Could not create .venv. On Debian or Ubuntu install the venv module first
(sudo apt install python3-venv), remove the .venv folder if one was left behind and run ./run-gui.sh again."
fi

.venv/bin/python -c 'import sys; raise SystemExit(sys.version_info < (3, 11))' ||
    fail "The .venv folder uses a Python older than 3.11 or does not work. Remove or rename it and run again."

if [ ! -x .venv/bin/iai-gui ]; then
    echo "Installing the package into .venv (first run; needs internet for pyserial and setuptools) ..."
    .venv/bin/python -m pip install --disable-pip-version-check -e ../../bindings/python -e . ||
        fail "The install failed: see the messages above. Check the internet connection or proxy and run again.
If it keeps failing, remove the .venv folder and try once more."
fi

.venv/bin/iai-gui "$@" || fail "iai-gui stopped with an error: see the messages above."
