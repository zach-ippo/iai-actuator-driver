#!/usr/bin/env bash
# The Linux half of .github/release/local_release.py: the checks of the linux-x64 deliverables that need a Linux
# system. It runs inside a WSL distro (Ubuntu 24.04), started by the run.ps1 of the Linux test kit, which copies the
# folder local_release.py prepared (src/, in/, consumer/, release.env) to ~/work/<folder name> and runs this script
# there. Not meant to be run by hand.
#
#   local_release_wsl.sh native        ec_tests, c_smoke and the command line, cross-built with zig, run on Linux
#   local_release_wsl.sh sdk-smoke     the SDK zip unpacked: CMake find_package (the C smoke test and the frozen ABI 1.0
#                                      program), and the compile line of its README.txt
#   local_release_wsl.sh wheel         the manylinux wheel built, checked (check_package.py, auditwheel), installed in
#                                      a new venv, and the Python suite run against it with the source package moved
#   local_release_wsl.sh nuget-smoke   a new net8.0 program references the package from a local feed and loads the
#                                      packaged libiai_actuator_driver.so
#
# What each step prints for local_release.py to read: lines starting WSL-ENV:, AUDITWHEEL:, PACKAGED-LIBRARY:.
# Every test runs on the simulator: nothing opens a serial port.
set -euo pipefail

STEP=${1:?usage: local_release_wsl.sh native|sdk-smoke|wheel|nuget-smoke}
ROOT=$PWD
# shellcheck disable=SC1091
. "$ROOT/release.env"   # VERSION NUGET_VERSION RID LIB WHEEL_PLATFORM SDK_NAME OUT (the release run's work folder on Windows)
SRC=$ROOT/src
WORK=$HOME/work/_iai-release       # outside the synced copy: the tools environment is kept between runs
SCRATCH=$WORK/$STEP
TOOLS=$WORK/tools

# what the programs below must not inherit: it would point a test at another library or backend
unset IAI_ACTUATOR_DRIVER_NATIVE IAI_DRIVER_BACKEND PYTHONPATH PYTHONHOME VIRTUAL_ENV IAD_WHEEL_PLATFORM IAD_BUILD_DIR
export PYTHONUTF8=1 PIP_DISABLE_PIP_VERSION_CHECK=1 DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1 \
    DOTNET_SKIP_FIRST_TIME_EXPERIENCE=1 MSBUILDDISABLENODEREUSE=1 DOTNET_CLI_USE_MSBUILD_SERVER=0

rm -rf "$SCRATCH"
mkdir -p "$SCRATCH"

say() { printf '\n== %s\n' "$*"; }

environment() {
    echo "WSL-ENV: $(. /etc/os-release && echo "$PRETTY_NAME"), kernel $(uname -r), $(ldd --version | head -n 1)," \
        "$(gcc --version | head -n 1), $(python3 --version), dotnet $(dotnet --version 2>/dev/null || echo none)"
}

step_native() {
    local t=$ROOT/in/native-build
    chmod +x "$t/test/ec_tests" "$t/test/c_smoke" "$t/dist/iai-actuator-driver-cpp"
    environment
    say "ec_tests --quiet (the C++ tests of the cross-built core)"
    (cd "$SRC/core" && "$t/test/ec_tests" --quiet)
    say "c_smoke (plain C99 against libiai_actuator_driver.so)"
    (cd "$t/test" && ./c_smoke)
    say "the command line on the simulator"
    "$t/dist/iai-actuator-driver-cpp" --version
    "$t/dist/iai-actuator-driver-cpp" --sim status
}

step_sdk_smoke() {
    local zip=$ROOT/in/sdk/$SDK_NAME.zip unpacked=$SCRATCH/sdk build=$SCRATCH/build
    local prefix=$unpacked/$SDK_NAME smoke=$SRC/core/tests/c_smoke.c frozen=$SRC/core/abi/1.0/c_smoke.c
    environment
    say "unpack the SDK zip ($SDK_NAME.zip)"
    mkdir -p "$unpacked"
    cmake -E chdir "$unpacked" cmake -E tar xf "$zip"
    ls -l "$prefix" "$prefix/lib" "$prefix/bin"
    # lib/: the library under its SONAME ($LIB.<ABI major>, executable) and $LIB, a text linker script for -l
    local real
    real=$(ls "$prefix/lib/$LIB".[0-9]* 2>/dev/null | head -n 1)
    test -n "$real" || { echo "FAILED: $prefix/lib has no $LIB.<ABI major> (the library under its SONAME)"; return 1; }
    test -x "$real" || { echo "FAILED: $real is not executable (the zip lost the mode)"; return 1; }
    test -f "$prefix/lib/$LIB" || { echo "FAILED: $prefix/lib/$LIB (the linker script for -l) is missing"; return 1; }

    say "CMake find_package(iai_actuator_driver), the C smoke test and the frozen ABI 1.0 program (gcc)"
    cmake -S "$SRC/.github/release/sdk_consumer" -B "$build" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="$prefix" -DIAD_SMOKE_SOURCE="$smoke" -DIAD_FROZEN_SMOKE_SOURCE="$frozen"
    cmake --build "$build"
    ctest --test-dir "$build" --output-on-failure

    say "the compile line of the SDK's README.txt, as written, run from the SDK folder (the program lands in bin/)"
    # the line is taken from the README and run unchanged: the only thing put in place is the app.c it names, which here
    # is the C smoke test. LD_LIBRARY_PATH is dropped so that the rpath is what finds the library.
    local line
    line=$(sed -n 's/^  \(cc -Iinclude .*\)$/\1/p' "$prefix/README.txt")
    case $line in
        "") echo "FAILED: $prefix/README.txt has no line starting with 'cc -Iinclude'"; return 1 ;;
        *$'\n'*) echo "FAILED: $prefix/README.txt has more than one line starting with 'cc -Iinclude'"; return 1 ;;
    esac
    echo "README.txt: $line"
    cp "$smoke" "$prefix/app.c"
    (cd "$prefix" && eval "$line")
    local app=$prefix/bin/app
    test -x "$app" || { echo "FAILED: the README.txt compile line did not make $app"; return 1; }
    (cd / && env -u LD_LIBRARY_PATH "$app")
}

tools_venv() {   # build and auditwheel: kept in WORK/tools between runs (release.yml: pip install build / auditwheel)
    if [ ! -x "$TOOLS/venv/bin/python" ]; then
        mkdir -p "$TOOLS"
        python3 -m venv "$TOOLS/venv"
    fi
    "$TOOLS/venv/bin/python" -m pip install --quiet build auditwheel
    "$TOOLS/venv/bin/python" -m pip list --format=freeze | grep -iE '^(build|auditwheel|pyelftools|setuptools|wheel)=='
}

step_wheel() {
    local stage=$SCRATCH/wheel-src dist=$SCRATCH/dist venv=$SCRATCH/venv-wheel
    local native=$stage/iai_actuator_driver/_native/$RID
    local pkg=$SRC/bindings/python parked=$SCRATCH/source-package
    environment
    say "tools: build and auditwheel"
    tools_venv

    say "build the wheel (py3-none-$WHEEL_PLATFORM) from a copy of bindings/python with the library inside"
    mkdir -p "$stage" "$dist"
    cp -a "$pkg/." "$stage/"
    find "$stage" -name __pycache__ -type d -prune -exec rm -rf {} +
    mkdir -p "$native"
    cp "$ROOT/in/native/$RID/$LIB" "$native/$LIB"
    # the copy synced into this distro has every file executable; a checkout has 644 and the library 755
    find "$stage" -type f -exec chmod 644 {} +
    chmod 755 "$native/$LIB"
    IAD_WHEEL_PLATFORM=$WHEEL_PLATFORM "$TOOLS/venv/bin/python" -m build --wheel --outdir "$dist" "$stage"
    local wheels=("$dist"/*.whl)
    [ "${#wheels[@]}" -eq 1 ] || { echo "FAILED: expected one wheel in $dist, found ${#wheels[@]}"; return 1; }
    local wheel=${wheels[0]}
    case $wheel in *"-py3-none-$WHEEL_PLATFORM.whl") ;; *) echo "FAILED: unexpected wheel name ${wheel##*/}"; return 1 ;; esac

    say "check the wheel (check_package.py wheel)"
    python3 "$SRC/.github/release/check_package.py" wheel "$wheel" --rid "$RID" --version "$VERSION"

    say "manylinux check (auditwheel show)"
    "$TOOLS/venv/bin/auditwheel" show "$wheel" | tee "$SCRATCH/auditwheel.txt"
    # the oldest manylinux policy the wheel still satisfies must not need more than glibc 2.17 (the wheel's tag)
    local policy   # auditwheel wraps its lines: join them before looking for the sentence
    policy=$(tr '\n' ' ' < "$SCRATCH/auditwheel.txt" | tr -s ' ' \
        | grep -oE 'is consistent with the following platform tag: "manylinux_2_[0-9]+_x86_64"' \
        | grep -oE 'manylinux_2_[0-9]+_x86_64' | head -n 1 || true)
    if [ -z "$policy" ]; then
        echo "FAILED: auditwheel names no manylinux platform tag for the wheel"
        return 1
    fi
    local minor=${policy#manylinux_2_}; minor=${minor%%_*}
    if [ "$minor" -gt 17 ]; then
        echo "FAILED: auditwheel: the wheel needs $policy, newer than manylinux_2_17"
        return 1
    fi
    echo "AUDITWHEEL: $policy (the wheel is tagged $WHEEL_PLATFORM)"

    say "install the wheel in a new venv"
    python3 -m venv "$venv"
    "$venv/bin/python" -m pip install --quiet "$wheel"

    # the tests must import the installed wheel, not the source package next to them
    say "move the source package out of the way, then check the installed package and run the Python suite"
    mv "$pkg/iai_actuator_driver" "$parked"
    trap "mv '$parked' '$pkg/iai_actuator_driver'" EXIT   # expanded now: the variables are local to this function
    (cd "$pkg" && "$venv/bin/python" "$SRC/.github/release/check_package.py" installed --rid "$RID" --version "$VERSION")
    (cd "$pkg" && "$venv/bin/python" -m unittest discover -s tests -q)
    mv "$parked" "$pkg/iai_actuator_driver"
    trap - EXIT

    say "hand the wheel to the release run"
    mkdir -p "$OUT/dist/wheel-$RID"
    cp "$wheel" "$OUT/dist/wheel-$RID/"
    ls -l "$OUT/dist/wheel-$RID/"
}

step_nuget_smoke() {
    local feed=$SCRATCH/feed consumer=$SCRATCH/consumer packages=$SCRATCH/nuget-packages
    environment
    mkdir -p "$feed" "$packages"
    cp "$ROOT/in/nuget/IaiActuatorDriver.$NUGET_VERSION.nupkg" "$feed/"
    cp -a "$ROOT/consumer" "$consumer"
    sed -i "s|@FEED@|$feed|" "$consumer/nuget.config"
    say "a new net8.0 program references IaiActuatorDriver $NUGET_VERSION from a local feed and loads the packaged library"
    # a package folder of its own: a version restored by an earlier run must not stand in for this package
    (cd "$consumer" && NUGET_PACKAGES=$packages dotnet run --project Consumer.csproj --configuration Release)
    local packaged=$consumer/bin/Release/net8.0/runtimes/$RID/native/$LIB
    test -f "$packaged" || { echo "FAILED: the program's output has no $packaged"; return 1; }
    echo "PACKAGED-LIBRARY: runtimes/$RID/native/$LIB ($(stat -c %s "$packaged") bytes)"
}

case $STEP in
    native) step_native ;;
    sdk-smoke) step_sdk_smoke ;;
    wheel) step_wheel ;;
    nuget-smoke) step_nuget_smoke ;;
    *) echo "unknown step: $STEP" >&2; exit 64 ;;
esac
echo "step $STEP: OK"
