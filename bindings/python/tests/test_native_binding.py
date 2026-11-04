"""The Python binding of the native core (iai_actuator_driver.native) on the native simulator.

The layout, signature and error-mapping checks run everywhere: they need no library. The rest is skipped when the
native library is not found (build it with 'python build.py shared' in core/, or set IAI_ACTUATOR_DRIVER_NATIVE) or does
not export the simulator yet. A few results are compared with the package API on ``SimulatedEC`` (the limits and stored
rows of each profile). No hardware and no serial port: every axis is simulated.
"""

from __future__ import annotations

import _thread
import ctypes
import dataclasses
import logging
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from typing import Any
from unittest import mock

from iai_actuator_driver import native
from iai_actuator_driver.cell import CellError
from iai_actuator_driver.ec import BaudChangeResult, Elecylinder, PositionEntry
from iai_actuator_driver.errors import (
    EcAlarmError,
    EcDriverError,
    EcError,
    EcNotReadyError,
    IaiCommandRejected,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    NonVolatileWriteError,
    ServoBlockedError,
    SoftLimitError,
    UnsafeCommandError,
    VerificationError,
)
from iai_actuator_driver.check import CheckItem
from iai_actuator_driver.modbus import TransportError
from iai_actuator_driver.recovery import RecoveryError, RecoveryPolicy, UnitReplacedError
from iai_actuator_driver.sim import SimulatedEC

ROOT = Path(__file__).resolve().parent.parent
HEADER = ROOT.parents[1] / "core" / "include" / "iai_actuator_driver.h"
MANIFEST_TOOL = ROOT.parents[1] / "core" / "abi" / "manifest.py"
TIME_SCALE = 20.0  # simulated seconds per real second
# A cell polls the axes of one line in turn: at 20x a short home return can end between two polls of an axis and
# look as if it never started, so the cell runs slower.
CELL_TIME_SCALE = 5.0
PROFILES = ("slider", "gripper", "rod", "pcon", "rotary")

# ctypes mirror of every struct of the header
STRUCTS = {
    "iad_axis_options": native.AxisOptions, "iad_open_options": native.OpenOptions,
    "iad_sim_options": native.SimOptions,
    "iad_move_options": native.MoveOptions, "iad_position_move_options": native.PositionMoveOptions,
    "iad_nv_write_options": native.NvWriteOptions, "iad_param_write_options": native.ParamWriteOptions,
    "iad_baud_options": native.BaudOptions, "iad_reset_options": native.ResetOptions,
    "iad_cell_options": native.CellOptions, "iad_servo_options": native.ServoOptions,
    "iad_home_options": native.HomeOptions, "iad_stop_options": native.StopOptions,
    "iad_wait_idle_options": native.WaitIdleOptions, "iad_change_options": native.ChangeOptions,
    "iad_cancel_paused_move_options": native.CancelPausedMoveOptions,
    "iad_status": native.Status, "iad_range": native.Range,
    "iad_limits": native.Limits, "iad_identity": native.Identity, "iad_position_entry": native.CPositionEntry,
    "iad_parameter": native.Parameter, "iad_param_write_result": native.ParamWriteResult,
    "iad_position_write_result": native.PositionWriteResult, "iad_reset_result": native.ResetResult,
    "iad_baud_result": native.BaudResult, "iad_cell_result": native.CellResult,
    # 0.2
    "iad_push_options": native.PushOptions, "iad_push_result": native.PushResult,
    "iad_sequence_point": native.SequencePoint, "iad_sequence_options": native.SequenceOptions,
    "iad_move_info": native.MoveInfo, "iad_monitor_options": native.MonitorOptions,
    "iad_monitor_sample": native.MonitorSample, "iad_event": native.Event, "iad_fast_summary": native.FastSummary,
    "iad_track_options": native.TrackOptions, "iad_link_check": native.LinkCheck,
    "iad_recovery_policy": native.CRecoveryPolicy, "iad_recovery_summary": native.RecoverySummary,
    "iad_check_options": native.CheckOptions, "iad_check_summary": native.CheckSummary,
    "iad_axis_move": native.AxisMove, "iad_motion_caps": native.MotionCaps,
    # 0.3
    "iad_transport_callbacks": native.TransportCallbacks, "iad_bus_options": native.BusOptions,
    # 1.1
    "iad_serial_options": native.SerialOptions,
}


def _manifest_tool() -> Any:
    """core/abi/manifest.py, loaded from the source tree (it is not part of the package)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("iad_abi_manifest", MANIFEST_TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("iad_abi_manifest", module)
    spec.loader.exec_module(module)
    return module


def _header_text() -> str:
    """The header without its comments."""
    text = HEADER.read_text(encoding="utf-8")
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def _native_ready() -> str | None:
    """``None`` when the binding can run on the native simulator, else the reason the simulator tests skip."""
    try:
        native.load()
    except native.NativeLibraryError as exc:
        return str(exc).splitlines()[0]
    if "iad_axis_open_sim" in native.missing_functions():
        return f"{native.library_path()} does not export the simulator yet (a partial build of the C ABI)"
    return None


SKIP_REASON = _native_ready()
needs_native = unittest.skipIf(SKIP_REASON is not None, SKIP_REASON or "")


# ====================================================================== no library needed


class LayoutTest(unittest.TestCase):
    """The ctypes mirrors match the header: sizes, alignment, field order, functions, codes."""

    def test_struct_sizes_and_alignment(self) -> None:
        for cls, size in native.STRUCT_SIZES.items():
            with self.subTest(struct=cls.__name__):
                self.assertEqual(ctypes.sizeof(cls), size)
                self.assertEqual(ctypes.alignment(cls), 8)
        self.assertEqual(native.Status.alarm_code.offset, 32)
        self.assertEqual(native.Status.reserved.offset, 144)
        self.assertEqual((native.CellResult.code.offset, native.CellResult.index.offset, native.CellResult.axis.offset,
                          native.CellResult.message.offset), (208, 212, 216, 344))  # fmt: skip
        # 0.8: every input is sized (struct_size, reserved_header first); c_api.cpp checks the same offsets
        for cls in (native.AxisOptions, native.OpenOptions, native.SimOptions, native.MoveOptions,
                    native.PositionMoveOptions, native.NvWriteOptions, native.ParamWriteOptions, native.BaudOptions,
                    native.ResetOptions, native.CellOptions, native.CPositionEntry, native.PushOptions,
                    native.SequencePoint, native.SequenceOptions, native.MonitorOptions, native.TrackOptions,
                    native.CRecoveryPolicy, native.CheckOptions, native.AxisMove, native.ServoOptions,
                    native.HomeOptions, native.StopOptions, native.WaitIdleOptions, native.ChangeOptions,
                    native.CancelPausedMoveOptions, native.SerialOptions):  # fmt: skip
            with self.subTest(sized=cls.__name__):
                self.assertEqual((cls.struct_size.offset, cls.reserved_header.offset), (0, 4))
                self.assertEqual(cls().struct_size, ctypes.sizeof(cls))  # constructing one sizes it
        self.assertEqual((native.AxisOptions.push_settle_s.offset, native.AxisOptions.completion.offset), (72, 80))
        self.assertEqual((native.OpenOptions.line.offset, native.OpenOptions.position_unit_mm.offset,
                          native.OpenOptions.fast.offset), (36, 40, 48))  # fmt: skip
        self.assertEqual((native.MoveOptions.accel_pattern.offset, native.MoveOptions.decel_g.offset,
                          native.MoveOptions.completion.offset), (44, 48, 56))  # fmt: skip
        self.assertEqual((native.CellOptions.line.offset, native.CellOptions.position_unit_mm.offset), (60, 64))
        move = native.AxisMove
        self.assertEqual((move.position.offset, move.decel_g.offset, move.accel_pattern.offset, move.axis.offset),
                         (8, 48, 56, 64))  # fmt: skip
        self.assertEqual((native.Event.kind.offset, native.Event.axis.offset, native.Event.message.offset,
                          native.Event.index.offset, native.Event.caller_thread.offset),
                         (232, 240, 432, 692, 696))  # fmt: skip
        written = native.PositionWriteResult
        self.assertEqual((written.push_power.offset, written.row.offset), (56, 84))
        self.assertEqual(native.Limits.units.offset, 968)
        self.assertEqual(native.AXIS_NAME_SIZE, 128)
        self.assertEqual([item.struct_size for item in native._sized_array(native.AxisMove, 3)], [224] * 3)

    @unittest.skipUnless(HEADER.is_file(), "the C header is in a source checkout only")
    def test_struct_fields_follow_the_header(self) -> None:
        text = _header_text()
        found = dict(re.findall(r"typedef struct (iad_\w+) \{(.*?)\} \1;", text, flags=re.S))
        self.assertEqual(set(found), set(STRUCTS))
        for c_name, body in found.items():
            with self.subTest(struct=c_name):
                names = [m.group(1) for m in re.finditer(r"(\w+)\s*(?:\[[^\]]*\])?\s*;", body)]
                self.assertEqual([f[0] for f in STRUCTS[c_name]._fields_], names)

    @unittest.skipUnless(MANIFEST_TOOL.is_file(), "the ABI manifest is in a source checkout only")
    def test_every_function_has_a_signature(self) -> None:
        # core/abi/manifest.py: both headers against native._SIGNATURES / _TESTING_SIGNATURES, in header order,
        # parameter by parameter (no hand-kept function count)
        tool = _manifest_tool()
        self.assertEqual(tool.check_python(native), [])
        headers = tool.parse_headers()
        self.assertEqual([n for n, _, _ in headers[tool.STABLE].functions], [n for n, _, _ in native._SIGNATURES])
        self.assertEqual([n for n, _, _ in headers[tool.TESTING].functions],
                         [n for n, _, _ in native._TESTING_SIGNATURES])  # fmt: skip

    @unittest.skipUnless(MANIFEST_TOOL.is_file(), "the ABI manifest is in a source checkout only")
    def test_the_manifest_and_the_csharp_imports_follow_the_headers(self) -> None:
        tool = _manifest_tool()
        self.assertEqual(tool.check_manifest(), [])  # core/abi/manifest.txt is current
        self.assertEqual(tool.check_csharp(), [])  # every function imported once at least, kinds equal, Winapi

    def test_the_manifest_check_catches_a_changed_signature(self) -> None:
        if not MANIFEST_TOOL.is_file():
            self.skipTest("the ABI manifest is in a source checkout only")
        tool = _manifest_tool()
        headers = tool.parse_headers()
        name, ret, params = headers[tool.STABLE].functions[0]
        headers[tool.STABLE].functions[0] = (name, ret, [*params, "double"])
        self.assertTrue(any(name in p for p in tool.check_python(native, headers)))
        self.assertTrue(any(name in p for p in tool.check_csharp(headers)))
        self.assertNotEqual(tool.check_manifest(headers), [])
        old = tool.manifest_text(tool.parse_headers())  # abi 1.x
        new = old.replace(f"function {name} ", f"function {name}_renamed ")
        self.assertTrue(tool.breaking_changes(old, new))  # within a major from 1 on: only additions pass
        self.assertEqual(tool.breaking_changes(old, old + "function iad_added void(void)\n"), [])

    @unittest.skipUnless(HEADER.is_file(), "the C header is in a source checkout only")
    def test_result_codes_and_version_follow_the_header(self) -> None:
        text = _header_text()
        enum = re.search(r"enum iad_result_code \{(.*?)\};", text, flags=re.S)
        assert enum is not None
        codes = {int(value): name for name, value in re.findall(r"(IAD_\w+)\s*=\s*(-?\d+)", enum.group(1))}
        self.assertEqual(codes, native.RESULT_NAMES)
        major = re.search(r"#define IAD_ABI_VERSION_MAJOR (\d+)", text)
        minor = re.search(r"#define IAD_ABI_VERSION_MINOR (\d+)", text)
        assert major is not None and minor is not None
        header_version = (int(major.group(1)), int(minor.group(1)))
        # the binding needs the header's major and a minor it does not exceed (1.0: no call needs a 1.1 function yet)
        self.assertEqual(header_version[0], native.ABI_VERSION_MAJOR)
        self.assertLessEqual(native.ABI_VERSION_MINOR, header_version[1])
        self.assertEqual((native.ABI_VERSION_MAJOR, native.ABI_VERSION_MINOR), (1, 0))
        testing = HEADER.with_name("iai_actuator_driver_testing.h").read_text(encoding="utf-8")
        found = re.search(r"#define IAD_TESTING_ABI_VERSION (\d+)", testing)
        assert found is not None
        self.assertEqual(int(found.group(1)), native.TESTING_ABI_VERSION)

    @unittest.skipUnless(HEADER.is_file(), "the C header is in a source checkout only")
    def test_defaults_unknown_and_the_calling_convention_follow_the_header(self) -> None:
        from iai_actuator_driver import ec as native_ec

        text = _header_text()

        def const(name: str) -> int:
            found = re.search(rf"#define {name} (-?\d+)\s*$", text, flags=re.M)
            assert found is not None, name
            return int(found.group(1))

        # (0.8) the tribool: 2, typeable everywhere; an input 0 is false and any other value but 2 is true
        self.assertEqual(native.IAD_UNKNOWN, const("IAD_UNKNOWN"))
        self.assertEqual(native.IAD_UNKNOWN, 2)
        self.assertIsNone(native._tri(native.IAD_UNKNOWN))
        self.assertIs(native._tri(0), False)
        self.assertIs(native._tri(1), True)
        self.assertEqual(native._tri_in(None), native.IAD_UNKNOWN)
        self.assertEqual((native._tri_in(False), native._tri_in(True)), (0, 1))
        # (0.8) 0 is the default of every enumeration: the methods start at 1 and IAD_METHOD_DEFAULT is 0
        methods = {name: const(f"IAD_METHOD_{name.upper()}") for name in ("coil", "iai", "iai2", "register")}
        self.assertEqual(native._METHODS, methods)
        self.assertEqual(native_ec._METHODS, methods)
        self.assertEqual((native._METHOD_DEFAULT, native_ec._METHOD_DEFAULT), (const("IAD_METHOD_DEFAULT"),) * 2)
        self.assertEqual(native._METHOD_DEFAULT, 0)
        for name in ("IAD_TEACH_TABLE", "IAD_BAUD_DIRECT", "IAD_ACCEL_DEFAULT", "IAD_COMPLETION_DEFAULT", "IAD_LINE_AUTO"):
            self.assertEqual(const(name), 0, name)
        self.assertEqual(native._TEACH_METHODS["table"], 0)
        self.assertEqual(native._BAUD_METHODS["direct"], 0)
        self.assertEqual(native._LINES["auto"], 0)
        # (0.8) IAD_CALL: stdcall on 32-bit Windows only (WinDLL, WINFUNCTYPE), the one convention elsewhere
        self.assertEqual(native._STDCALL, sys.platform == "win32" and struct.calcsize("P") == 4)
        self.assertEqual(native._CALLBACK, ctypes.WINFUNCTYPE if native._STDCALL else ctypes.CFUNCTYPE)
        header = HEADER.read_text(encoding="utf-8")
        self.assertRegex(header, r"#\s*if defined\(_WIN32\) && \(defined\(_M_IX86\) \|\| defined\(__i386__\)\)\s+#\s*define IAD_CALL __stdcall")

    def test_an_explicit_zero_is_refused_where_the_core_reads_it_as_the_default(self) -> None:
        # iad_axis_move and iad_sequence_point have no _init: the core reads a 0 there as "the default" (0.8)
        self.assertTrue(math.isnan(native._slot("speed_mm_s", None)))
        self.assertEqual(native._slot("speed_mm_s", 12), 12.0)
        self.assertTrue(math.isnan(native._slot("speed_mm_s", math.nan)))
        with self.assertRaises(SoftLimitError):
            native._slot("speed_mm_s", 0)
        with self.assertRaises(SoftLimitError):
            native._slot("band_mm", 0.0)


class ErrorMappingTest(unittest.TestCase):
    """Each result code raises the package's own class (docs/dev/NATIVE_ABI.md, the Python column)."""

    def test_codes_map_to_the_package_classes(self) -> None:
        cases = {
            native.IAD_ERR_SOFT_LIMIT: SoftLimitError,
            native.IAD_ERR_NOT_READY: EcNotReadyError,
            native.IAD_ERR_ALARM: EcAlarmError,
            native.IAD_ERR_MOTION_TIMEOUT: MotionTimeoutError,
            native.IAD_ERR_MODBUS_TIMEOUT: ModbusTimeout,
            native.IAD_ERR_FRAME: ModbusFrameError,
            native.IAD_ERR_TRANSPORT: TransportError,
            native.IAD_ERR_VERIFICATION: VerificationError,
            native.IAD_ERR_NONVOLATILE_REFUSED: NonVolatileWriteError,
            native.IAD_ERR_INVALID_ARGUMENT: ValueError,
            native.IAD_ERR_UNSUPPORTED: NotImplementedError,
            native.IAD_ERR_INTERNAL: RuntimeError,
            native.IAD_ERR_SERVO_BLOCKED: ServoBlockedError,
            native.IAD_ERR_MOTION_INCOMPLETE: MotionIncompleteError,
            native.IAD_ERR_SLAVE_EXCEPTION: ModbusException,
            native.IAD_ERR_COMMAND_REJECTED: IaiCommandRejected,
            native.IAD_ERR_UNSAFE_REFUSED: UnsafeCommandError,
            native.IAD_ERR_CELL: CellError,
            native.IAD_ERR_MODBUS: ModbusError,
            native.IAD_ERR_DRIVER: EcDriverError,
            native.IAD_ERR_RECOVERY: RecoveryError,
            native.IAD_ERR_UNIT_REPLACED: UnitReplacedError,
            native.IAD_ERR_FILE: OSError,
            native.IAD_ERR_CANCELLED: KeyboardInterrupt,
            -99: EcDriverError,  # a code newer than the binding
        }
        for code, cls in cases.items():
            with self.subTest(code=code):
                exc = native._make_error(code, {"message": "from the core", "stop_attempted": True})
                self.assertIs(type(exc), cls)
                self.assertEqual(exc.iad_code, code)
                self.assertTrue(exc.stop_attempted)
                if cls is not CellError:
                    self.assertEqual(str(exc), "from the core")
        self.assertIsInstance(native._make_error(native.IAD_ERR_DRIVER, {"kind": "EcError", "message": "m"}), EcError)
        self.assertIsInstance(native._make_error(native.IAD_ERR_FRAME, {"kind": "ModbusCrcError"}), ModbusCRCError)

    def test_an_unknown_negative_code_is_the_base_driver_error_with_its_code(self) -> None:
        # iai_actuator_driver.h reserves the codes from -26 down for later additions: a code that a newer library
        # returns is the base driver error with its code kept, never a crash of this binding
        self.assertEqual(min(native.RESULT_NAMES), native.IAD_ERR_CANCELLED)  # -25, the last code this binding knows
        for code in (-26, -27, -1000, -(2**31)):
            with self.subTest(code=code):
                exc = native._make_error(code, {"code": code, "kind": "NewerError", "message": "a newer kind"})
                self.assertIs(type(exc), EcDriverError)
                self.assertEqual((exc.iad_code, str(exc), exc.stop_attempted), (code, "a newer kind", False))
                bare = native._make_error(code, {})  # no error record: the name an unknown code has
                self.assertIs(type(bare), EcDriverError)
                self.assertEqual((bare.iad_code, str(bare)), (code, "IAD_ERR_UNKNOWN"))
                # one axis's error of a group command
                axes = {"SIM1:1": {"code": code, "message": "m"}}
                cell = native._make_error(native.IAD_ERR_CELL, {"operation": "home_all", "axes": axes})
                self.assertIsInstance(cell, CellError)
                self.assertIs(type(cell.errors["SIM1:1"]), EcDriverError)
                self.assertEqual(cell.errors["SIM1:1"].iad_code, code)

    def test_details_become_attributes(self) -> None:
        alarm = native._make_error(native.IAD_ERR_ALARM, {"message": "alarm 0D9", "alarm_code": 0x0D9})
        self.assertEqual(alarm.alarm_code, 0x0D9)
        blocked = native._make_error(native.IAD_ERR_SERVO_BLOCKED, {"message": "m", "reasons": ["EMG input"]})
        self.assertEqual(blocked.reasons, ("EMG input",))
        slave = native._make_error(
            native.IAD_ERR_SLAVE_EXCEPTION, {"message": "m", "exception_code": 2, "function": 5}
        )  # fmt: skip
        self.assertEqual((slave.code, slave.function), (2, 5))
        rejected = native._make_error(
            native.IAD_ERR_COMMAND_REJECTED,
            {"message": "rejected 0x11", "function": 0x45, "command": 0x0A, "rejection": 0x11, "reason": "op1"},
        )
        self.assertEqual((rejected.function, rejected.command, rejected.rejection), (0x45, 0x0A, 0x11))
        self.assertEqual((str(rejected), rejected.reason, rejected.hint), ("rejected 0x11", "op1", ""))
        cell = native._make_error(
            native.IAD_ERR_CELL,
            {"operation": "home_all", "axes": {"SIM1:2": {"code": native.IAD_ERR_ALARM, "message": "alarm"}}},
        )
        self.assertIsInstance(cell.errors["SIM1:2"], EcAlarmError)
        self.assertEqual(cell.operation, "home_all")
        replaced = native._make_error(
            native.IAD_ERR_UNIT_REPLACED,
            {"message": "m", "expected": {"serial_number": "A"}, "found": {"serial_number": "B"}, "report": {"x": 1}},
        )
        self.assertEqual((replaced.expected, replaced.found, replaced.report), ({"serial_number": "A"},
                                                                                {"serial_number": "B"}, {"x": 1}))
        self.assertIsInstance(replaced, RecoveryError)


class LoadingTest(unittest.TestCase):
    """A missing library says where it looked, and the package does not import without one."""

    def test_missing_library_is_a_clear_error(self) -> None:
        missing = str(ROOT / "no-such-dir" / native.library_name())
        with mock.patch.dict(os.environ, {native.ENV_VAR: missing}), mock.patch.object(native, "_library", None):
            self.assertIsNone(native.find_library())
            self.assertFalse(native.available())
            with self.assertRaises(native.NativeLibraryError) as ctx:
                native.NativeAxis.open_sim()
            self.assertIn(missing, str(ctx.exception))
            self.assertIn("build.py shared", str(ctx.exception))
            self.assertIsInstance(ctx.exception, EcDriverError)
            # the pure-Python edition is gone: the text names the last release with it and offers it no more
            self.assertIn("1.0.x is the last release with it", str(ctx.exception))
            self.assertNotIn("does not need it", str(ctx.exception))

    def test_the_package_does_not_import_without_the_library(self) -> None:
        env = dict(os.environ, **{native.ENV_VAR: str(ROOT / "no-such-dir")})
        env.pop("IAI_DRIVER_BACKEND", None)
        out = subprocess.run([sys.executable, "-c", "import iai_actuator_driver"], cwd=ROOT, env=env,
                             capture_output=True, text=True, timeout=60)  # fmt: skip
        self.assertNotEqual(out.returncode, 0)
        self.assertIn("ImportError: iai_actuator_driver needs its native library", out.stderr)
        self.assertIn("not found", out.stderr)
        self.assertIn("1.0.x is the last release with it", out.stderr)


class HelperCallTest(unittest.TestCase):
    """The helper thread of an interruptible call: a Ctrl+C in the wait for it must not make it look ended."""

    def test_wait_after_ctrl_c_still_waits_for_the_call(self) -> None:
        release = threading.Event()

        def work() -> None:
            release.wait(10.0)

        helper = native._Helper("iad-call:test", work)
        self.addCleanup(release.set)
        timer = threading.Timer(0.1, _thread.interrupt_main)
        timer.start()
        self.addCleanup(timer.join)
        with self.assertRaises(KeyboardInterrupt):
            while not helper.wait(0.05):  # a simulated SIGINT is seen between two waits, as in the driver's slices
                pass
        # Thread.join() on CPython 3.12 returns at once here and is_alive() says False while the call still runs
        self.assertFalse(helper.wait(0.2))
        self.assertFalse(helper.ended)
        release.set()
        self.assertTrue(helper.wait(5.0))
        self.assertTrue(helper.ended)


# ====================================================================== on the native simulator


@needs_native
class NativeLibraryTest(unittest.TestCase):
    def test_versions_and_exports(self) -> None:
        major, minor = native.abi_version()
        if HEADER.is_file():  # a source checkout: the library of this tree has the header's version
            text = HEADER.read_text(encoding="utf-8")
            found = [re.search(rf"#define IAD_ABI_VERSION_{part} (\d+)", text) for part in ("MAJOR", "MINOR")]
            assert found[0] is not None and found[1] is not None
            self.assertEqual((major, minor), (int(found[0].group(1)), int(found[1].group(1))))
        self.assertTrue(major == native.ABI_VERSION_MAJOR and minor >= native.ABI_VERSION_MINOR)
        self.assertRegex(native.library_version(), r"^\d+\.\d+\.\d+")
        self.assertEqual(native.missing_functions(), [])
        self.assertEqual(native.result_name(native.IAD_ERR_SOFT_LIMIT), "IAD_ERR_SOFT_LIMIT")
        self.assertIsInstance(native.list_ports(), list)  # opens nothing

    def test_serial_options_calls_go_through_the_signatures(self) -> None:
        # 1.1: iad_serial_options_init and iad_transport_open_serial_options called through the ctypes signatures; the
        # port never exists, so a refused framing stays unopened and a valid one fails only in the open
        if {"iad_serial_options_init", "iad_transport_open_serial_options"} & set(native.missing_functions()):
            self.skipTest("a C ABI 1.0 library")
        lib = native.load()
        options = native.SerialOptions()
        self.assertEqual(lib.fn("iad_serial_options_init")(ctypes.byref(options)), native.IAD_OK)
        self.assertEqual((options.struct_size, options.read_timeout_s, options.baud_rate), (64, 0.2, 115200))
        self.assertEqual((options.data_bits, options.parity, options.stop_bits), (8, 0, 1))
        open_options = lib.fn("iad_transport_open_serial_options")
        handle = ctypes.c_void_p(1)  # overwritten with NULL first
        options.data_bits = 6
        self.assertEqual(open_options(b"IAD-NO-SUCH-PORT", ctypes.byref(options), ctypes.byref(handle)),
                         native.IAD_ERR_INVALID_ARGUMENT)  # fmt: skip
        self.assertIsNone(handle.value)
        options.data_bits, options.parity, options.stop_bits = 7, 1, 2  # 7E2
        handle = ctypes.c_void_p(1)
        self.assertEqual(open_options(b"IAD-NO-SUCH-PORT", ctypes.byref(options), ctypes.byref(handle)),
                         native.IAD_ERR_TRANSPORT)  # fmt: skip
        self.assertIsNone(handle.value)

    def test_stale_handles_are_detected(self) -> None:
        ax = native.NativeAxis.open_sim()
        handle = ax._h()
        ax.close()
        ax.close()  # twice: fine
        lib = native.load()
        with self.assertRaises(ValueError) as ctx:  # IAD_ERR_INVALID_ARGUMENT from the library's registry
            native._check(lib, "iad_axis_status", handle, ctypes.byref(native.Status()))
        self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_INVALID_ARGUMENT)
        native._check(lib, "iad_axis_close", handle)  # closing a closed handle: OK
        native._check(lib, "iad_axis_close", None)  # and NULL
        with self.assertRaises(ValueError):
            ax.status()

    def test_unknown_profile_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            native.NativeAxis.open_sim("no-such-unit")

    def test_the_testing_header_is_bound_after_its_exact_version_check(self) -> None:
        lib = native.load()
        self.assertTrue(lib.has("iad_sim_unplug"))  # bound on first use
        self.assertEqual(int(lib.fn("iad_testing_abi_version")()), native.TESTING_ABI_VERSION)
        self.assertFalse(lib.has("iad_no_such_function"))
        other = native._Library(lib.path)  # a second binding of the same file, written for another testing version
        with mock.patch.object(native, "TESTING_ABI_VERSION", native.TESTING_ABI_VERSION + 1):
            for _ in range(2):  # refused every time, not only the first
                with self.assertRaises(native.NativeLibraryError) as ctx:
                    other.fn("iad_sim_unplug")
                self.assertIn("testing ABI", str(ctx.exception))
            self.assertIsNotNone(other.fn("iad_abi_version"))  # the stable functions are not affected


@needs_native
class NativeAxisTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ax = native.NativeAxis.open_sim("slider", time_scale=TIME_SCALE)
        self.addCleanup(self.ax.close)

    def ready(self) -> None:
        self.ax.connect()
        self.assertTrue(self.ax.servo(True).servo_on)
        self.assertTrue(self.ax.home().homed)

    def test_connect_identify_and_status(self) -> None:
        identity = self.ax.connect()
        self.assertEqual(identity.product_family, "EC")
        self.assertEqual(identity.units, "mm")
        self.assertEqual(identity.max_positions, 3)
        self.assertEqual(self.ax.identify().model, identity.model)
        self.assertEqual(self.ax.max_positions(), 3)
        self.assertIsInstance(self.ax.device_info, dict)
        status = self.ax.status()
        self.assertIsInstance(status, native.NativeStatus)
        self.assertTrue(status.modbus_enabled)
        self.assertIn(status.raw_words, (8, 14, 20, 22))
        self.assertEqual(status.alarm_code, 0)
        self.assertIsNone(status.alarm_level)
        self.assertIsNone(self.ax.baudrate)  # no rate modelled

    def test_moves(self) -> None:
        self.ready()
        self.assertAlmostEqual(self.ax.move_to(50.0, 100.0).position_mm, 50.0, delta=0.1)
        self.assertAlmostEqual(self.ax.move_by(-10.0, 100.0).position_mm, 40.0, delta=0.1)
        self.assertAlmostEqual(self.ax.jog(1.0).position_mm, 41.0, delta=0.1)
        self.assertIsNotNone(self.ax.check_move(100.0))
        before = self.ax.status().position_mm
        for call in (lambda: self.ax.check_move(1000.0), lambda: self.ax.move_to(1000.0)):
            with self.assertRaises(SoftLimitError) as ctx:
                call()
            self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_SOFT_LIMIT)
        self.assertAlmostEqual(self.ax.status().position_mm, before, delta=0.02)  # nothing sent

    def test_pause_resume_and_cancel(self) -> None:
        self.ready()
        self.assertIsInstance(self.ax.move_to(100.0, 20.0, wait=False), native.NativeMoveHandle)
        time.sleep(0.05)
        self.ax.pause()
        self.assertTrue(self.ax.status().paused)
        self.ax.resume()
        self.assertAlmostEqual(self.ax.wait_idle(100.0, 0.1, timeout=30.0).position_mm, 100.0, delta=0.1)
        self.ax.move_to(0.0, 20.0, wait=False)
        time.sleep(0.05)
        self.ax.pause()
        after = self.ax.cancel_paused_move()
        status = after if after is not None else self.ax.status()
        self.assertFalse(status.paused)
        self.assertGreater(status.position_mm, 1.0)  # the remaining travel was dropped
        with self.assertRaises(EcNotReadyError):  # no pause to cancel
            self.ax.cancel_paused_move()

    def test_stop_from_another_thread(self) -> None:
        self.ready()
        outcome: list[object] = []

        def run() -> None:
            try:
                outcome.append(self.ax.move_to(150.0, 10.0, timeout=60.0))
            except EcDriverError as exc:
                outcome.append(exc)

        mover = threading.Thread(target=run)
        mover.start()
        time.sleep(0.1)
        self.ax.stop()
        mover.join(10.0)
        self.assertFalse(mover.is_alive(), "the move did not end after a stop from another thread")
        self.assertEqual(len(outcome), 1)
        self.assertLess(self.ax.status().position_mm, 149.0)

    def test_limits_json_and_struct_agree(self) -> None:
        self.ax.connect()
        limits, ranges = self.ax.limits(), self.ax.ranges()
        self.assertEqual(limits["units"], ranges.units)
        for name in ("position", "speed", "accel_decel"):
            self.assertAlmostEqual(limits[name]["min"], getattr(ranges, name).min)
            self.assertAlmostEqual(limits[name]["max"], getattr(ranges, name).max)
        self.assertEqual(ranges.stored_rows.count, 3)
        with mock.patch.object(native, "_JSON_START", 16):  # too small: the binding grows the buffer once
            self.assertEqual(self.ax.limits(), limits)

    def test_options(self) -> None:
        self.ax.set_options(max_speed_mm_s=100.0)
        self.assertEqual(self.ax.get_options()["max_speed_mm_s"], 100.0)
        self.ready()
        self.assertAlmostEqual(self.ax.ranges().speed.max, 100.0)
        with self.assertRaises(SoftLimitError):
            self.ax.move_to(20.0, 200.0)
        with self.assertRaises(TypeError):
            self.ax.set_options(no_such_option=1)

    def test_alarm_history(self) -> None:
        history = self.ax.read_alarm_history(include_empty=True)
        self.assertIsInstance(history, list)
        self.assertTrue(all(isinstance(slot, dict) for slot in history))

    def test_position_table_and_the_nonvolatile_guard(self) -> None:
        self.ready()
        stored = self.ax.read_position_table(2)
        self.assertIsInstance(stored, PositionEntry)
        self.assertTrue(stored.checksum_ok)
        with self.assertRaises(NonVolatileWriteError):
            self.ax.write_position_table(256, stored)  # the simple-data rows: never
        same = self.ax.write_position_table(2, stored)  # allow_nonvolatile: no longer needed
        self.assertTrue(same.skipped)
        self.assertEqual(self.ax.nonvolatile_writes(), 0)
        moved = self.ax.write_position_table(2, dataclasses.replace(stored, position_mm=61.0), allow_nonvolatile=True)
        self.assertFalse(moved.skipped)
        self.assertAlmostEqual(moved.position_mm, 61.0)
        self.assertEqual(self.ax.nonvolatile_writes(), 1)
        self.assertEqual(self.ax.nonvolatile_writes_last_hour(), 1)
        self.assertIsInstance(self.ax.last_result(), dict)
        self.assertAlmostEqual(self.ax.read_position_table(2).position_mm, 61.0)
        self.assertAlmostEqual(self.ax.move_to_position(2).position_mm, 61.0, delta=0.1)
        taught = self.ax.teach_position(2)  # at the stored position: nothing written
        self.assertTrue(taught.skipped)
        self.assertEqual(self.ax.nonvolatile_writes(), 1)

    def test_parameters(self) -> None:
        self.ax.connect()
        params = self.ax.parameters()
        self.assertTrue(params)
        self.assertTrue(all("name" in p for p in params))
        param = next(p for p in params if p.get("writable") and not p.get("dangerous"))
        read = self.ax.read_parameter(param["number"])
        self.assertEqual(read["name"], param["name"])
        value = read["value"] if isinstance(read.get("value"), (int, float)) else read["text"]
        result = self.ax.write_parameter(param["number"], value)
        self.assertIsInstance(result, native.NativeParameterWriteResult)
        self.assertTrue(result.skipped)  # unchanged: nothing written or committed
        self.assertEqual(self.ax.nonvolatile_writes(), 0)
        self.assertFalse(self.ax.has_uncommitted_parameters())

    def test_context_manager_closes(self) -> None:
        with native.NativeAxis.open_sim("gripper", time_scale=TIME_SCALE) as ax:
            self.assertEqual(ax.connect().units, "mm")
        self.assertTrue(ax.closed)
        with self.assertRaises(ValueError):
            ax.status()


@needs_native
class NativeUnitTest(unittest.TestCase):
    def test_software_reset_rod_loses_hend(self) -> None:
        with native.NativeAxis.open_sim("rod", time_scale=TIME_SCALE) as ax:
            ax.connect()
            result = ax.software_reset(timeout=5.0, reset_wait_s=0.1)
            self.assertIsInstance(result, native.NativeResetResult)
            self.assertTrue(result.confirmed)
            self.assertTrue(result.confirmed_by)
            self.assertTrue(result.needs_home)
            self.assertFalse(ax.status().homed)
            with self.assertRaises(EcNotReadyError):
                ax.move_to(10.0, 20.0)

    def test_baud_rate_detected_and_changed(self) -> None:
        with native.NativeAxis.open_sim("rod", time_scale=TIME_SCALE, sim_baudrate=38400) as ax:
            self.assertEqual(ax.baudrate, 38400)
            self.assertTrue(ax.baud_detected)
            with self.assertRaises(SoftLimitError):  # no IAI code: nothing sent
                ax.set_baudrate(12345)
            result = ax.set_baudrate(115200)
            self.assertIsInstance(result, BaudChangeResult)
            self.assertEqual((result.previous, result.baudrate, result.method), (38400, 115200, "direct"))
            self.assertTrue(result.verified)
            self.assertEqual(ax.baudrate, 115200)


@needs_native
class PythonEditionParityTest(unittest.TestCase):
    """``NativeAxis.open_sim`` and the package API on ``SimulatedEC`` read the same ranges and stored rows."""

    def test_limits_and_rows_of_each_profile(self) -> None:
        for profile in PROFILES:
            with self.subTest(profile=profile):
                with Elecylinder.open(SimulatedEC(profile=profile)) as py, native.NativeAxis.open_sim(profile) as nat:
                    py.connect()
                    nat.connect()
                    expected, got = py.limits(), nat.limits()
                    self.assertEqual(got["units"], expected["units"])
                    self.assertEqual(got["family"], expected["family"])
                    for name in ("position", "speed", "accel", "decel", "accel_decel", "stored_rows"):
                        for end in ("min", "max"):
                            self.assertAlmostEqual(got[name][end], expected[name][end], places=6, msg=f"{name} {end}")
                    self.assertEqual(nat.max_positions(), py.max_positions())
                    for row in (0, 1):
                        a, b = nat.read_position_table(row), py.read_position_table(row)
                        for f in ("position_mm", "band_mm", "speed_mm_s", "accel_g", "decel_g", "push_power",
                                  "control_flags"):
                            self.assertAlmostEqual(getattr(a, f), getattr(b, f), places=6, msg=f"row {row} {f}")


@needs_native
class NativeCellTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cell = native.NativeCell.open_sim("slider,gripper", 2, time_scale=CELL_TIME_SCALE)
        self.addCleanup(self.cell.close)

    def ready(self) -> None:
        self.assertTrue(all(s.servo_on for s in self.cell.servo_all(True).values()))
        self.assertTrue(all(s.homed for s in self.cell.home_all().values()))

    def test_names_and_axes(self) -> None:
        self.assertEqual(self.cell.names, ("SIM1:1", "SIM1:2", "SIM2:1", "SIM2:2"))
        self.assertEqual(len(self.cell), 4)
        self.assertIn("SIM2:1", self.cell)
        self.assertIs(self.cell["SIM1:2"], self.cell[1])
        self.assertEqual(self.cell["SIM2:2"].connect().product_family, "EC")
        self.assertEqual(self.cell.info()["axes"], list(self.cell.names))
        statuses = self.cell.status_all()
        self.assertEqual(list(statuses), list(self.cell.names))
        self.assertTrue(all(isinstance(s, native.NativeStatus) for s in statuses.values()))
        self.assertEqual(list(self.cell.status_all(["SIM2:2", 0])), ["SIM2:2", "SIM1:1"])
        with self.assertRaises(KeyError):
            self.cell["SIM9:1"]

    def test_group_moves(self) -> None:
        self.ready()
        done = self.cell.move_all({"SIM1:1": 50.0, "SIM2:1": 5.0}, 40.0)
        self.assertAlmostEqual(done["SIM1:1"].position_mm, 50.0, delta=0.1)
        self.assertAlmostEqual(done["SIM2:1"].position_mm, 5.0, delta=0.1)
        self.assertIsNone(self.cell.move_all({"SIM1:2": 20.0, "SIM2:2": 3.0}, 40.0, wait=False))
        waited = self.cell.wait_all(["SIM1:2", "SIM2:2"])
        self.assertAlmostEqual(waited["SIM1:2"].position_mm, 20.0, delta=0.1)
        self.assertAlmostEqual(waited["SIM2:2"].position_mm, 3.0, delta=0.1)
        self.assertEqual(len(self.cell.stop_all()), 4)

    def test_one_bad_target_refuses_the_whole_move(self) -> None:
        self.ready()
        with self.assertRaises(CellError) as ctx:
            self.cell.move_all({"SIM1:1": 60.0, "SIM2:1": 50.0})  # the gripper's stroke is 9.50 mm
        self.assertIsInstance(ctx.exception.errors["SIM2:1"], SoftLimitError)
        self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_CELL)
        self.assertAlmostEqual(self.cell["SIM1:1"].status().position_mm, 0.0, delta=0.1)  # nothing sent
        out = self.cell.move_all({"SIM1:1": 60.0, "SIM2:1": 50.0}, return_exceptions=True)
        self.assertIsInstance(out["SIM2:1"], SoftLimitError)

    def test_stop_all_from_another_thread(self) -> None:
        self.ready()
        outcome: list[object] = []

        def run() -> None:
            try:
                outcome.append(self.cell.move_all({"SIM1:1": 150.0, "SIM1:2": 150.0}, 10.0, timeout=60.0))
            except EcDriverError as exc:
                outcome.append(exc)

        mover = threading.Thread(target=run)
        mover.start()
        time.sleep(0.1)
        self.cell.stop_all()
        mover.join(10.0)
        self.assertFalse(mover.is_alive(), "move_all did not end after stop_all from another thread")
        self.assertLess(self.cell["SIM1:1"].status().position_mm, 149.0)

    def test_stop_all_and_pause_all_with_an_unknown_name_act_on_the_axes_they_have(self) -> None:
        self.ready()
        self.assertIsNone(self.cell.move_all({"SIM1:1": 150.0, "SIM1:2": 150.0}, 2.0, wait=False))
        with self.assertRaises(CellError) as ctx:
            self.cell.stop_all(["SIM1:1", "SIM9:9"])
        self.assertEqual(list(ctx.exception.errors), ["SIM9:9"])
        self.assertIsInstance(ctx.exception.errors["SIM9:9"], KeyError)
        self.assertFalse(self.cell["SIM1:1"].status().moving)  # stopped although the list named an unknown axis
        self.assertTrue(self.cell["SIM1:2"].status().moving)  # not listed
        out = self.cell.pause_all("SIM9:9, SIM1:2", return_exceptions=True)
        self.assertEqual(list(out), ["SIM1:2", "SIM9:9"])
        self.assertIsNone(out["SIM1:2"])
        self.assertIsInstance(out["SIM9:9"], KeyError)
        self.assertTrue(self.cell["SIM1:2"].status().paused)
        self.cell.resume_all(["SIM1:2"])
        self.assertEqual(list(self.cell.stop_all([7], return_exceptions=True)), ["7"])  # nothing else stopped
        self.assertTrue(self.cell["SIM1:2"].status().moving)
        self.assertEqual(len(self.cell.stop_all()), 4)

    def test_cancel_calls_reach_every_listed_axis_then_report_the_unknown_names(self) -> None:
        # a remembered list naming an axis that dropped out of a reopened cell: the cancel still reaches the others
        with self.assertRaises(CellError) as ctx:
            self.cell.cancel_latch(["SIM9:9", "SIM1:1", 7])
        self.assertEqual(sorted(ctx.exception.errors), ["7", "SIM9:9"])
        self.assertIsInstance(ctx.exception.errors["SIM9:9"], KeyError)
        self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_CELL)
        self.assertTrue(self.cell["SIM1:1"].cancel_latched)
        self.assertFalse(self.cell["SIM1:2"].cancel_latched)  # not listed
        self.cell.cancel_latch("SIM1:2")
        with self.assertRaises(CellError) as ctx:
            self.cell.cancel_reset("SIM1:1, SIM9:9")
        self.assertEqual(list(ctx.exception.errors), ["SIM9:9"])
        self.assertFalse(self.cell["SIM1:1"].cancel_latched)
        self.assertTrue(self.cell["SIM1:2"].cancel_latched)
        with self.assertRaises(CellError):  # only unknown names: nothing is touched, never "every axis"
            self.cell.cancel_reset(["SIM9:9"])
        self.assertTrue(self.cell["SIM1:2"].cancel_latched)
        self.cell.cancel_reset()
        self.assertFalse(any(self.cell[n].cancel_latched for n in self.cell.names))

    def test_cancel_waits_reaches_a_waiting_axis_though_the_list_names_an_unknown_one(self) -> None:
        self.ready()
        outcome: list[object] = []

        def run() -> None:
            try:
                outcome.append(self.cell.move_all({"SIM1:1": 150.0}, 10.0, timeout=60.0))
            except EcDriverError as exc:
                outcome.append(exc)

        mover = threading.Thread(target=run)
        mover.start()
        self.addCleanup(mover.join, 30.0)
        time.sleep(0.3)
        with self.assertRaises(CellError) as ctx:
            self.cell.cancel_waits(["SIM9:9", "SIM1:1"])
        self.assertEqual(list(ctx.exception.errors), ["SIM9:9"])
        mover.join(10.0)
        self.assertFalse(mover.is_alive(), "move_all did not end after cancel_waits from another thread")
        self.assertEqual(len(outcome), 1)
        self.assertIsInstance(outcome[0], CellError)  # the wait was cancelled: KeyboardInterrupt on its axis
        self.assertIsInstance(outcome[0].errors["SIM1:1"], KeyboardInterrupt)  # type: ignore[attr-defined]
        self.assertLess(self.cell["SIM1:1"].status().position_mm, 149.0)

    def test_coordinated_timeout_must_be_above_zero(self) -> None:
        request = {"axes": {"x": "SIM1:1", "y": "SIM1:2"}, "path": {"segments": [{"line": {"x": 20, "y": 10}}]},
                   "speed": 20, "accel_g": 0.1}  # fmt: skip
        start = self.cell["SIM1:1"].status().position_mm
        for bad in (0, 0.0, -1, -0.5):
            with self.subTest(timeout_s=bad):
                with self.assertRaises(ValueError) as ctx:
                    self.cell.coordinated_run({**request, "timeout_s": bad})
                self.assertIn("nothing sent", str(ctx.exception))
                self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_INVALID_ARGUMENT)
                with self.assertRaises(ValueError):  # the same request as a dry run
                    self.cell.coordinated_plan({**request, "timeout_s": bad})
        self.assertAlmostEqual(self.cell["SIM1:1"].status().position_mm, start, delta=0.01)  # nothing was sent
        self.assertTrue(self.cell.coordinated_run({**request, "timeout_s": 30})["completed"])
        self.assertAlmostEqual(self.cell["SIM1:1"].status().position_mm, 20.0, delta=0.2)

    def test_close_invalidates_its_axes(self) -> None:
        axis = self.cell["SIM1:1"]
        self.cell.close()
        self.cell.close()  # twice: fine
        self.assertTrue(axis.closed)
        with self.assertRaises(ValueError):
            axis.status()
        with self.assertRaises(ValueError):
            self.cell.status_all()


# ====================================================================== ABI 0.2 on the native simulator

# Ctrl+C tests run the move slower so that the interrupt always lands while the axis moves (the PC may be busy).
SLOW_TIME_SCALE = 5.0


def _wait_until(predicate: Any, timeout: float = 5.0) -> bool:
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return bool(predicate())


def _interrupt_main_after(test: unittest.TestCase, delay: float) -> None:
    """Ctrl+C (a simulated SIGINT on the main thread) after ``delay`` s; the timer is joined at cleanup."""
    timer = threading.Timer(delay, _thread.interrupt_main)
    timer.start()
    test.addCleanup(timer.join)


@needs_native
class NativeMotion02Test(unittest.TestCase):
    def setUp(self) -> None:
        self.ax = native.NativeAxis.open_sim("slider", time_scale=TIME_SCALE)
        self.addCleanup(self.ax.close)
        self.ax.connect()
        self.ax.servo(True)
        self.ax.home()

    def test_profile_move_pattern_and_completion(self) -> None:
        self.assertAlmostEqual(self.ax.move_to(40.0, 100.0, decel_g=0.2).position_mm, 40.0, delta=0.1)
        self.assertAlmostEqual(self.ax.move_to(20.0, 100.0, completion="fast").position_mm, 20.0, delta=0.2)
        self.assertAlmostEqual(self.ax.move_by(5.0, 100.0, completion="safe").position_mm, 25.0, delta=0.2)
        self.assertIsNotNone(self.ax.check_move(30.0, decel_g=0.2))
        with self.assertRaises(ValueError):
            self.ax.move_to(30.0, accel_pattern="no-such-pattern")
        self.assertAlmostEqual(self.ax.wait_idle(25.0, 0.2, completion="fast").position_mm, 25.0, delta=0.2)

    def test_move_handle(self) -> None:
        handle = self.ax.move_to(80.0, 100.0, wait=False)
        self.addCleanup(handle.close)
        self.assertIsInstance(handle, native.NativeMoveHandle)
        self.assertEqual((handle.target_mm, handle.band_mm, handle.completion), (80.0, 0.1, "safe"))
        self.assertIsNone(handle.result)
        final = handle.wait(timeout=30.0)
        self.assertAlmostEqual(final.position_mm, 80.0, delta=0.1)
        self.assertTrue(handle.done())
        self.assertEqual(handle.result, final)
        self.assertTrue(handle.to_dict()["done"])
        with self.ax.move_by(-10.0, 100.0, wait=False) as step:
            self.assertAlmostEqual(step.wait().position_mm, 70.0, delta=0.1)
        self.assertIn("closed", repr(step))
        jog = self.ax.jog(1.0, wait=False)
        self.assertAlmostEqual(jog.wait().position_mm, 71.0, delta=0.1)
        jog.close()
        slow = self.ax.move_to(0.0, 10.0, wait=False)
        stopped = slow.cancel(wait=True)
        self.assertTrue(slow.cancelled)
        self.assertFalse((stopped or self.ax.status()).moving)
        slow.close()

    def test_change_accel_and_speed_and_sequence(self) -> None:
        self.ax.move_to(150.0, 20.0, wait=False).close()
        self.assertIsNone(self.ax.change_accel(0.2))
        final = self.ax.change_speed(100.0, wait=True, timeout=30.0)
        self.assertAlmostEqual(final.position_mm, 150.0, delta=0.1)
        with self.assertRaises(EcNotReadyError):  # no move running: refused, nothing sent
            self.ax.change_accel(0.2)
        statuses = self.ax.move_sequence([10.0, {"mm": 20.0, "speed_mm_s": 80.0}, 5.0], 100.0)
        self.assertEqual([round(s.position_mm) for s in statuses], [10, 20, 5])
        with self.assertRaises(SoftLimitError):
            self.ax.move_sequence([10.0, 5000.0])
        with self.assertRaises(SoftLimitError):
            self.ax.move_sequence([{"position": 10.0}])

    def test_push_on_the_gripper(self) -> None:
        with native.NativeAxis.open_sim("gripper", time_scale=TIME_SCALE) as grip:
            grip.connect()
            grip.servo(True)
            grip.home()
            result = grip.push(2.0, 5.0, 50)
            self.assertIsInstance(result, native.NativePushResult)
            self.assertIn(result.outcome, ("contact", "missed"))
            self.assertEqual(result.missed, result.outcome == "missed")
            self.assertIsInstance(result.to_dict()["status"], dict)
            with self.assertRaises(SoftLimitError):  # band end outside the stroke: nothing sent
                grip.push(2.0, 50.0, 50)

    def test_ctrl_c_during_a_move_stops_the_axis(self) -> None:
        with native.NativeAxis.open_sim("slider", time_scale=SLOW_TIME_SCALE) as ax:
            ax.connect()
            ax.servo(True)
            ax.home()
            _interrupt_main_after(self, 0.3)
            with self.assertRaises(KeyboardInterrupt) as ctx:
                ax.move_to(150.0, 10.0, timeout=60.0)
            self.assertTrue(any("decel stop" in note for note in getattr(ctx.exception, "__notes__", ())))
            self.assertTrue(_wait_until(lambda: not ax.status().moving))
            self.assertLess(ax.status().position_mm, 149.0)

    def test_ctrl_c_during_a_handle_wait_stops_the_axis(self) -> None:
        with native.NativeAxis.open_sim("slider", time_scale=SLOW_TIME_SCALE) as ax:
            ax.connect()
            ax.servo(True)
            ax.home()
            handle = ax.move_to(150.0, 10.0, wait=False)
            _interrupt_main_after(self, 0.3)
            with self.assertRaises(KeyboardInterrupt):
                handle.wait()
            self.assertTrue(handle.cancelled)
            self.assertTrue(_wait_until(lambda: not ax.status().moving))
            self.assertLess(ax.status().position_mm, 149.0)
            handle.close()

    def test_cancel_waits_from_another_thread(self) -> None:
        with native.NativeAxis.open_sim("slider", time_scale=SLOW_TIME_SCALE) as ax:
            ax.connect()
            ax.servo(True)
            ax.home()
            timer = threading.Timer(0.3, ax.cancel_waits)
            timer.start()
            self.addCleanup(timer.join)
            with self.assertRaises(KeyboardInterrupt) as ctx:
                ax.move_to(150.0, 10.0, timeout=60.0)
            self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_CANCELLED)
            self.assertTrue(_wait_until(lambda: not ax.status().moving))
            self.assertAlmostEqual(ax.move_to(10.0, 100.0).position_mm, 10.0, delta=0.1)  # later calls work


@needs_native
class NativeMonitor02Test(unittest.TestCase):
    def setUp(self) -> None:
        self.ax = native.NativeAxis.open_sim("slider", time_scale=TIME_SCALE)
        self.addCleanup(self.ax.close)
        self.ax.connect()
        self.ax.servo(True)
        self.ax.home()

    def test_monitor_callbacks_run_on_the_binding_thread(self) -> None:
        changes: list[tuple[float, str]] = []
        dones: list[float] = []
        self.assertIsNone(self.ax.monitor)
        mon = self.ax.start_monitor(
            on_change=lambda st, prev: changes.append((st.position_mm, threading.current_thread().name)),
            on_done=lambda st: dones.append(st.position_mm),
        )
        self.assertIs(self.ax.monitor, mon)
        self.assertTrue(_wait_until(lambda: mon.latest is not None))
        self.assertTrue(mon.running)
        sample = mon.wait_update(timeout=5.0)
        self.assertIsInstance(sample, native.NativeMonitorSample)
        self.assertEqual(sample.status.raw_words, 8)
        self.ax.move_to(30.0, 100.0)
        self.assertTrue(_wait_until(lambda: any(abs(p - 30.0) < 0.2 for p, _ in changes)))
        self.assertTrue(_wait_until(lambda: any(abs(p - 30.0) < 0.2 for p in dones)))
        self.assertTrue(all(name.startswith("iad-events") for _, name in changes))
        self.assertIsNotNone(mon.wait_done(timeout=5.0))
        self.assertGreater(mon.polls, 0)
        self.assertEqual(mon.info()["link_state"], "ok")
        with self.assertRaises(TypeError):
            self.ax.start_monitor(no_such_option=1)
        self.ax.stop_monitor()
        self.assertFalse(mon.running)
        self.assertIsNone(self.ax.monitor)

    def test_fast_mode(self) -> None:
        summary = self.ax.enable_fast()
        self.assertEqual((summary["completion"], summary["monitor"]), ("fast", True))
        self.assertTrue(summary["notes"])  # the simulator has no baud rate: kept, and said why
        self.assertTrue(self.ax.fast_mode)
        self.assertEqual(self.ax.get_options()["completion"], "fast")
        self.assertAlmostEqual(self.ax.move_to(20.0, 100.0).position_mm, 20.0, delta=0.2)
        self.assertEqual(self.ax.disable_fast()["completion"], "safe")
        self.assertFalse(self.ax.fast_mode)
        self.assertIsNone(self.ax.monitor)

    def test_frame_logger_and_log_sink(self) -> None:
        frames: list[tuple[str, bytes]] = []
        lines: list[tuple[int, str, str]] = []
        native.set_log_sink(lambda level, source, message: lines.append((level, source, message)))
        from iai_actuator_driver import _native as bridge  # the package's bridge goes back

        self.addCleanup(lambda: native.set_log_sink(bridge._forward if bridge._bridge_installed else None))
        self.ax.set_frame_logger(lambda direction, frame: frames.append((direction, frame)))
        self.ax.status()
        self.ax.set_frame_logger(None)
        self.assertIn("tx", [d for d, _ in frames])
        self.assertIn("rx", [d for d, _ in frames])
        self.assertTrue(all(isinstance(f, bytes) and len(f) >= 4 for _, f in frames))
        count = len(frames)
        self.ax.status()
        self.assertEqual(len(frames), count)  # removed
        native.NativeAxis.open_sim(time_scale=TIME_SCALE).close()  # the opening logs
        self.assertTrue(all(isinstance(level, int) and source for level, source, _ in lines))


@needs_native
class NativeRecoveryInfo02Test(unittest.TestCase):
    def setUp(self) -> None:
        self.ax = native.NativeAxis.open_sim("rod", time_scale=TIME_SCALE)
        self.addCleanup(self.ax.close)
        self.ax.connect()

    def test_track_check_and_recover(self) -> None:
        self.assertIsNone(self.ax.session())
        session = self.ax.track(rows=[0, 1])
        self.assertIn("serial_number", session)
        self.assertEqual(self.ax.session()["name"], session["name"])
        events: list[Any] = []
        check = self.ax.check_link(on_event=events.append)
        self.assertIsInstance(check, native.NativeLinkCheck)
        self.assertTrue(check.ok)
        self.assertIsNotNone(check.status)
        self.assertEqual(self.ax.link_state, "ok")
        report = self.ax.recover(policy=RecoveryPolicy(check_parameters=False), timeout=10.0)
        self.assertIsInstance(report, native.NativeRecoveryReport)
        self.assertTrue(report.same_unit)
        self.assertEqual(report.cause, "ok")
        self.assertIn("actions", report.to_dict())
        self.ax.note_target(None)
        self.ax.note_control(modbus_control=True, homed=False)
        self.ax.forget()
        self.assertIsNone(self.ax.session())

    def test_unit_info_backup_and_session_from_backup(self) -> None:
        info = self.ax.unit_info(sections=["identity"])
        self.assertIsInstance(info["identity"], dict)
        self.assertIsNone(info.get("parameters"))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "rod.json"
            backup = self.ax.backup(path, rows=[0, 1])
            self.assertTrue(path.is_file())
            self.assertIn("positions", backup)
            self.assertEqual(self.ax.session_from_backup(path)["serial_number"], backup["identity"]["serial_number"])
            self.assertIn("serial_number", self.ax.session_from_backup(backup, name="rod"))
            saved = native.backup_axes({"rod": self.ax}, folder)
            self.assertTrue(Path(saved["rod"]).is_file())
        with self.assertRaises(ValueError):
            self.ax.session_from_backup({"no": "identity"})

    def test_run_check_streams_its_lines(self) -> None:
        items: list[CheckItem] = []
        report = self.ax.run_check(port_label="SIM", on_item=items.append)
        self.assertIsInstance(report, native.NativeCheckReport)
        self.assertTrue(report.passed)
        self.assertEqual(len(items), report.items)
        self.assertEqual([i.name for i in items], [i.name for i in report.lines])
        self.assertEqual(report.to_dict()["port"], "SIM")

    def test_line_modes_and_reads(self) -> None:
        self.assertIsNone(self.ax.line_name)
        self.assertEqual(self.ax.line()["kind"], "direct")
        self.assertIsNone(self.ax.gateway_status())
        with self.assertRaises(EcNotReadyError):
            self.ax.gateway_monitor()
        self.assertEqual(self.ax.stop_path, "coil")
        self.assertEqual(self.ax.strategy, "coil")
        self.ax.strategy = "iai"
        self.assertEqual(self.ax.strategy, "iai")
        self.ax.strategy = "coil"
        with self.assertRaises(ValueError):
            self.ax.strategy = "no-such-strategy"
        self.assertEqual(self.ax.read_lean_status().raw_words, 8)
        self.assertIn("position", self.ax.read_monitor())
        self.assertIsInstance(self.ax.read_boot_baudrate(), (int, type(None)))
        self.assertIsInstance(self.ax.read_home_pattern(), (int, type(None)))
        caps = self.ax.motion_caps()
        self.assertGreater(caps["speed_cap"], 0)
        self.assertEqual(self.ax.speed_cap_mm_s, caps["speed_cap"])
        self.assertEqual(self.ax.accel_cap_g, caps["accel_cap"])
        self.assertEqual(self.ax.accel_only_cap_g, caps["accel_only_cap"])
        self.assertEqual(self.ax.decel_cap_g, caps["decel_cap"])
        self.assertEqual(self.ax.push_cap_pct, caps["push_cap"])
        self.assertEqual(self.ax.recommended_min_speed(), (caps["min_speed"], caps["min_speed_source"]))
        self.assertIsNotNone(self.ax.ranges().push_speed)
        self.ax.disable_modbus_control()  # PIO control (PMSL off), then PC control again
        self.assertFalse(self.ax.status().modbus_enabled)
        self.ax.enable_modbus_control()
        self.assertTrue(self.ax.status().modbus_enabled)
        self.assertFalse(self.ax.set_action_mode(False, False).modbus_enabled)  # the 0x45 cmd 0x0C toggle
        self.assertTrue(self.ax.set_action_mode(True, False).modbus_enabled)
        self.assertTrue(self.ax.set_safety_speed(True).safety_speed)
        self.assertFalse(self.ax.set_safety_speed(False).safety_speed)
        self.ax.servo(True)
        self.ax.home(wait=False)
        self.assertTrue(_wait_until(lambda: self.ax.status().homed, 10.0))
        self.ax.finish_home()


@needs_native
class NativeCell02Test(unittest.TestCase):
    def setUp(self) -> None:
        self.faults: list[tuple[str, str, Any]] = []
        self.cell = native.NativeCell.open_sim(
            "slider,slider", 1, time_scale=CELL_TIME_SCALE, on_fault=lambda *args: self.faults.append(args)
        )
        self.addCleanup(self.cell.close)
        self.cell.servo_all(True)
        self.cell.home_all()

    def test_per_axis_values_move_each_and_pause(self) -> None:
        done = self.cell.move_all({"SIM1:1": 20.0, "SIM2:1": 30.0}, {"SIM1:1": 100.0, "SIM2:1": 50.0},
                                  decel_g={"SIM1:1": 0.2})  # fmt: skip
        self.assertAlmostEqual(done["SIM1:1"].position_mm, 20.0, delta=0.1)
        self.assertAlmostEqual(done["SIM2:1"].position_mm, 30.0, delta=0.1)
        each = self.cell.move_each({"SIM1:1": {"mm": 10.0, "speed_mm_s": 80.0, "decel_g": 0.3}, "SIM2:1": 5.0})
        self.assertAlmostEqual(each["SIM1:1"].position_mm, 10.0, delta=0.1)
        self.assertAlmostEqual(each["SIM2:1"].position_mm, 5.0, delta=0.1)
        with self.assertRaises(ValueError):
            self.cell.move_each({"SIM1:1": {"position": 10.0}})
        self.assertIsNone(self.cell.move_all({"SIM1:1": 100.0, "SIM2:1": 100.0}, 50.0, wait=False))
        self.assertEqual(self.cell.pause_all(), {"SIM1:1": None, "SIM2:1": None})
        self.assertTrue(all(s.paused for s in self.cell.status_all().values()))
        self.cell.resume_all()
        waited = self.cell.wait_all(timeout=60.0)
        self.assertAlmostEqual(waited["SIM1:1"].position_mm, 100.0, delta=0.1)

    def test_monitors_links_and_recovery(self) -> None:
        changes: list[float] = []
        monitors = self.cell.start_monitors(on_change=lambda st, prev: changes.append(st.position_mm))
        self.assertEqual(set(monitors), {"SIM1:1", "SIM2:1"})
        self.cell.move_all({"SIM1:1": 40.0}, 100.0)
        self.assertTrue(_wait_until(lambda: any(abs(p - 40.0) < 0.2 for p in changes)))
        self.cell.stop_monitors()
        sessions = self.cell.track_all()
        self.assertEqual(set(sessions), {"SIM1:1", "SIM2:1"})
        checks = self.cell.check_links()
        self.assertTrue(all(isinstance(c, native.NativeLinkCheck) and c.ok for c in checks.values()))
        self.cell.monitor_links(0.2, on_change=lambda name, check: None)
        self.cell.stop_link_monitor()
        reports = self.cell.recover_all(timeout=10.0)
        self.assertTrue(all(r["same_unit"] for r in reports.values()))
        self.assertEqual(self.faults, [])

    def test_information_lines_and_backup(self) -> None:
        readings = self.cell.monitor_all()
        self.assertTrue(all("position" in r for r in readings.values()))
        self.assertEqual(self.cell.lines(), {"SIM1": "direct", "SIM2": "direct"})
        self.assertEqual(self.cell.gateway_status_all(), {})
        with tempfile.TemporaryDirectory() as folder:
            saved = self.cell.backup(folder, ["SIM2:1"])
            self.assertTrue(Path(saved["SIM2:1"]).is_file())
            self.assertEqual(self.cell.last_result(), saved)

    def test_coordinated_plan_and_run(self) -> None:
        request = {"axes": {"x": "SIM1:1", "y": "SIM2:1"}, "path": {"segments": [{"line": {"x": 20, "y": 10}}]},
                   "speed": 20, "accel_g": 0.1}  # fmt: skip
        plan = self.cell.coordinated_plan(request)
        self.assertIn("points", plan)
        result = self.cell.coordinated_run(request)
        self.assertTrue(result["completed"])
        self.assertAlmostEqual(self.cell["SIM1:1"].status().position_mm, 20.0, delta=0.2)
        with self.assertRaises(SoftLimitError):
            self.cell.coordinated_plan({**request, "path": {"segments": [{"line": {"x": 5000}}]}})

    def test_ctrl_c_during_a_group_move_stops_the_axes(self) -> None:
        _interrupt_main_after(self, 0.3)
        with self.assertRaises(KeyboardInterrupt):
            self.cell.move_all({"SIM1:1": 150.0, "SIM2:1": 150.0}, 10.0, timeout=60.0)
        self.assertTrue(_wait_until(lambda: not any(s.moving for s in self.cell.status_all().values())))
        self.assertTrue(all(s.position_mm < 149.0 for s in self.cell.status_all().values()))


# ====================================================================== 0.3: transports, buses, simulators, clock, latch


class _SilentPort:
    """A Python transport that never answers (its reads time out)."""

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self.timeout = 0.2

    def write(self, data: bytes) -> None:
        self.written.append(bytes(data))

    def read(self, n: int) -> bytes:
        return b""

    def reset_input_buffer(self) -> None:
        pass


class _BrokenPort(_SilentPort):
    def write(self, data: bytes) -> None:
        raise OSError("adapter gone (test)")


class Abi03LayoutTest(unittest.TestCase):
    def test_sized_structs(self) -> None:
        self.assertEqual(ctypes.sizeof(native.BusOptions), 56)
        self.assertEqual(ctypes.sizeof(native.TransportCallbacks), native.TRANSPORT_CALLBACKS_SIZE)
        self.assertEqual(native.TransportCallbacks.write.offset, 24)
        self.assertEqual(native.sim_exception(2), 0x102)

    def test_out_of_range_kind_is_also_an_index_error(self) -> None:
        exc = native._make_error(native.IAD_ERR_INVALID_ARGUMENT, {"kind": "std::out_of_range", "message": "row 999"})
        self.assertIsInstance(exc, native.NativeOutOfRangeError)
        self.assertIsInstance(exc, ValueError)
        self.assertIsInstance(exc, IndexError)
        plain = native._make_error(native.IAD_ERR_INVALID_ARGUMENT, {"kind": "std::invalid_argument"})
        self.assertNotIsInstance(plain, IndexError)


@needs_native
class NativeAbi03Test(unittest.TestCase):
    def test_python_transport_drives_an_axis(self) -> None:
        sim = SimulatedEC(profile="slider")  # a Python transport behind the core's bus
        ax = native.NativeAxis.open_transport(sim, line="direct", min_gap=0.0)
        self.addCleanup(ax.close)
        self.assertTrue(ax.connect().model)
        frames: list[tuple[str, bytes]] = []
        bus = ax.bus()
        bus.set_frame_sink(lambda direction, frame: frames.append((direction, frame)))
        st = ax.status()
        self.assertGreater(len(frames), 0)
        self.assertEqual({d for d, _ in frames}, {"tx", "rx"})
        self.assertAlmostEqual(st.position_mm, sim.position_mm, delta=0.01)
        bus.set_frame_sink(None)
        bus.close()  # borrowed: the axis keeps its bus
        self.assertIsNotNone(ax.status())

    def test_transport_errors_carry_the_python_cause(self) -> None:
        bus = native.NativeBus.open(_BrokenPort(), retries=0, min_gap=0.0)
        self.addCleanup(bus.close)
        with self.assertRaises(TransportError) as ctx:
            bus.read_registers(1, 0x9000, 1)
        self.assertIsInstance(ctx.exception.__cause__, OSError)
        self.assertIn("adapter gone (test)", str(ctx.exception))

    def test_virtual_clock_makes_timeouts_free(self) -> None:
        port = _SilentPort()
        bus = native.NativeBus.open(port, retries=3, min_gap=0.0)
        self.addCleanup(bus.close)
        with native.virtual_clock(500.0) as clock:
            self.assertEqual(clock.now, 500.0)
            started = time.perf_counter()
            with self.assertRaises(ModbusTimeout):
                bus.read_registers(1, 0x9000, 1)
            self.assertLess(time.perf_counter() - started, 0.5)
            self.assertGreaterEqual(clock.now - 500.0, 0.8)  # four attempts of at least 0.2 s each
            self.assertEqual(len(port.written), 4)
            clock.advance(10.0)
            self.assertGreaterEqual(clock.now, 510.8)
        with self.assertRaises(EcNotReadyError):
            native.advance_clock(1.0)  # the real clock again

    def test_virtual_clock_runs_a_whole_move(self) -> None:
        with native.virtual_clock() as clock:
            started = time.perf_counter()
            with native.NativeAxis.open_sim("slider", poll_interval_s=0.01) as ax:  # time_scale 1: real seconds
                ax.connect()
                ax.servo(True)
                ax.home()
                self.assertAlmostEqual(ax.move_to(140.0, 20.0).position_mm, 140.0, delta=0.11)
            self.assertGreater(clock.now - 1000.0, 7.0)  # 140 mm at 20 mm/s on the virtual clock
            self.assertLess(time.perf_counter() - started, 5.0)

    def test_bus_requests_and_sim_hooks(self) -> None:
        sim = native.NativeSim.open("slider", slave=3, word_overrides={0x9002: 0x1234}, unreadable=[(0x0100, 4)])
        self.addCleanup(sim.close)
        bus = native.NativeBus.open(sim, retries=0, min_gap=0.0)
        self.addCleanup(bus.close)
        self.assertEqual(bus.read_registers(3, 0x9000, 4)[2], 0x1234)
        self.assertEqual(sim.register_value(0x9002), 0x1234)
        with self.assertRaises(ModbusException) as ctx:
            bus.read_registers(3, 0x0101, 1)
        self.assertEqual(ctx.exception.code, 2)
        with self.assertRaises(native.NativeOutOfRangeError):
            sim.register_value(0x0101)
        with self.assertRaises(IndexError):
            sim.position_row(9999)
        sim.set_word_override(0x9002, None)
        self.assertNotEqual(bus.read_registers(3, 0x9000, 4)[2], 0x1234)
        self.assertGreaterEqual(len(bus.report_slave_id(3)), 3)
        self.assertTrue(bus.probe(3))
        self.assertFalse(bus.probe(9, 0.02))
        bus.write_coil(3, 0x042C, True)
        self.assertTrue(sim.state()["stop_held"])
        bus.write_coil(3, 0x042C, False)
        self.assertEqual(bus.info()["retries"], 0)
        bus.retries = 2
        self.assertEqual(bus.retries, 2)
        self.assertTrue(any(frame[:2] == bytes([3, 0x03]) for _, frame in sim.requests()))
        with self.assertRaises(ValueError):
            native.NativeSim.open("slider", colour="red")

    def test_request_hook_drops_replaces_and_refuses(self) -> None:
        sim = native.NativeSim.open("slider")
        self.addCleanup(sim.close)
        bus = native.NativeBus.open(sim, retries=0, min_gap=0.0)
        self.addCleanup(bus.close)

        def hook(frame: bytes) -> Any:
            if frame[1] == 0x11:
                return None  # no answer
            if frame[1] == 0x03 and frame[2:4] == b"\x01\x00":
                return 4  # exception 04
            return NotImplemented

        sim.set_request_hook(hook)
        with self.assertRaises(ModbusTimeout):
            bus.report_slave_id(1)
        with self.assertRaises(ModbusException) as ctx:
            bus.read_registers(1, 0x0100, 1)
        self.assertEqual(ctx.exception.code, 4)
        self.assertEqual(len(bus.read_registers(1, 0x9000, 2)), 2)
        sim.set_request_hook(None)
        self.assertGreaterEqual(len(bus.report_slave_id(1)), 3)

    def test_stale_status_polls(self) -> None:
        sim = native.NativeSim.open("slider", time_scale=TIME_SCALE, stale_status_polls=50)
        self.addCleanup(sim.close)
        bus = native.NativeBus.open(sim, min_gap=0.0)
        self.addCleanup(bus.close)
        ax = native.NativeAxis.open_bus(bus, 1, poll_interval_s=0.005)
        self.addCleanup(ax.close)
        ax.connect()
        ax.move_to(100.0, 100.0, wait=False)
        words = bus.read_registers(1, 0x9000, 8)
        self.assertTrue(words[5] & (1 << 3))  # DSS1 PEND still set
        self.assertFalse(words[7] & (1 << 5))  # DSSE MOVE still clear
        self.assertGreaterEqual(sim.state()["stale_polls_served"], 1)
        ax.stop(wait=True)

    def test_power_unplug_and_line_break(self) -> None:
        sim = native.NativeSim.open("slider", baud_rate=115200)
        self.addCleanup(sim.close)
        bus = native.NativeBus.open(sim, retries=0, min_gap=0.0)
        self.addCleanup(bus.close)
        bus.read_registers(1, 0x9000, 1)
        sim.unplug()
        with self.assertRaises(TransportError):
            bus.read_registers(1, 0x9000, 1)
        sim.replug()
        bus.reopen()
        bus.read_registers(1, 0x9000, 1)
        bus.send_break(0.2)  # 150 ms or more: the unit drops to 9600 baud
        self.assertEqual(sim.state()["unit_baud_rate"], 9600)
        self.assertEqual(bus.detect_baudrate([1]), 9600)
        sim.power_cycle(lose_home=False)
        self.assertEqual(sim.state()["unit_baud_rate"], 38400)
        sim.power_off()
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(1, 0x9000, 1)
        sim.power_on()

    def test_sim_line_and_cell_on_transports(self) -> None:
        units = [native.NativeSim.open("slider", slave=1, time_scale=CELL_TIME_SCALE),
                 native.NativeSim.open("gripper", slave=2, time_scale=CELL_TIME_SCALE)]  # fmt: skip
        for unit in units:
            self.addCleanup(unit.close)
        line = native.NativeSimLine.open(units, name="LINE7")
        self.addCleanup(line.close)
        with self.assertRaises(ValueError):
            line.add(native.NativeSim.open("slider", slave=2))  # address 2 is taken
        cell = native.NativeCell.open_transports({"LINE7": line}, [1, 2], line="direct", poll_interval_s=0.005)
        self.addCleanup(cell.close)
        self.assertEqual(list(cell.names), ["LINE7:1", "LINE7:2"])
        bus = cell.bus("LINE7")
        self.assertEqual(len(bus.read_registers(2, 0x9000, 1)), 1)
        bus.close()
        self.assertTrue(any(slave == 2 for _, slave, _ in line.requests()))
        self.assertEqual(line.state()["slaves"], [1, 2])
        with self.assertRaises(IndexError):
            line.remove(9)
        with self.assertRaises(IndexError):
            cell.bus("COM9")

    def test_cancel_latch(self) -> None:
        with native.NativeAxis.open_sim("slider", time_scale=SLOW_TIME_SCALE) as ax:
            ax.connect()
            self.assertFalse(ax.cancel_latched)
            ax.cancel_latch()
            self.assertTrue(ax.cancel_latched)
            with self.assertRaises(KeyboardInterrupt) as ctx:
                ax.move_to(100.0, 10.0)
            self.assertEqual(ctx.exception.iad_code, native.IAD_ERR_CANCELLED)
            self.assertFalse(ax.status().moving)  # nothing was sent
            ax.stop()  # never refused
            ax.cancel_reset()
            timer = threading.Timer(0.3, ax.cancel_latch)
            timer.start()
            self.addCleanup(timer.join)
            with self.assertRaises(KeyboardInterrupt):
                ax.move_to(150.0, 10.0, timeout=60.0)
            ax.cancel_reset()
            self.assertAlmostEqual(ax.move_to(10.0, 100.0).position_mm, 10.0, delta=0.11)

    def test_log_level(self) -> None:
        native.set_log_level("error")
        native.set_log_level(logging.DEBUG)
        with self.assertRaises(ValueError):
            native.set_log_level("chatty")


@needs_native
class SinksOffTheStopPathTest(unittest.TestCase):
    """The core's log lines and frame entries never keep a STOP waiting (the load plan's A14): the core passes on only
    the lines some logger of the package lets through (its level follows the logging levels), and a Python frame
    logger that blocks runs after the bus is let go."""

    def setUp(self) -> None:
        from iai_actuator_driver import _native as bridge

        if not bridge._bridge_installed:
            self.skipTest("the logging bridge is not the core's log sink in this process")
        self.bridge = bridge
        # the package's loggers at their defaults (NOTSET: the root's WARNING), the bridge following them
        loggers = [logging.getLogger("iai_actuator_driver")] + [
            lg for name, lg in list(logging.Logger.manager.loggerDict.items())
            if isinstance(lg, logging.Logger) and name.startswith("iai_actuator_driver.")
        ]
        levels = [(lg, lg.level) for lg in loggers]
        explicit = native._log_level_explicit

        def restore() -> None:
            for lg, level in levels:
                lg.setLevel(level)
            native._log_level_explicit = explicit
            native.set_log_sink(bridge._forward)
            if explicit:  # as set_log_level left it in the other tests: every line
                native.load().fn("iad_set_log_level")(0)

        self.addCleanup(restore)
        for lg in loggers:
            lg.setLevel(logging.NOTSET)
        native._log_level_explicit = False
        native.set_log_sink(bridge._forward)
        self.assertGreaterEqual(logging.getLogger("iai_actuator_driver").getEffectiveLevel(), logging.WARNING)

    def test_lines_no_logger_lets_through_never_cross_into_python(self) -> None:
        crossed: list[str] = []

        class Counting(dict):  # _forward looks each line's source up here: one lookup per line that crossed
            def get(self, key: Any, default: Any = None) -> Any:
                crossed.append(key)
                return super().get(key, default)

        with mock.patch.object(self.bridge, "LOG_SOURCES", Counting(self.bridge.LOG_SOURCES)):
            native.NativeAxis.open_sim(time_scale=TIME_SCALE).close()  # the opening logs debug lines
            self.assertEqual(crossed, [])  # no logger keeps them: none crossed
            with self.assertLogs("iai_actuator_driver.modbus", logging.DEBUG) as logs:  # a level changed: followed
                native.NativeAxis.open_sim(time_scale=TIME_SCALE).close()
            self.assertTrue(crossed)
            self.assertTrue(any(r.levelno == logging.DEBUG for r in logs.records))
            crossed.clear()
            native.NativeAxis.open_sim(time_scale=TIME_SCALE).close()  # back to WARNING after assertLogs
            self.assertEqual(crossed, [])
            mine: list[int] = []
            native.set_log_sink(lambda level, source, message: mine.append(level))  # another sink: every line
            native.NativeAxis.open_sim(time_scale=TIME_SCALE).close()
            self.assertIn(logging.DEBUG, mine)

    def test_a_frame_logger_that_blocks_never_keeps_a_stop_waiting(self) -> None:
        # The reader's frame logger is parked on its first entry (a console write that blocks). Before, that entry was
        # handed over with the bus held: the STOP from another thread waited for it. Now it comes after the bus is let
        # go: the stop() ends while the logger is still parked.
        with native.NativeAxis.open_sim("slider", time_scale=TIME_SCALE) as ax:
            ax.connect()
            parked, release = threading.Event(), threading.Event()

            def frame_logger(direction: str, frame: bytes) -> None:
                if threading.current_thread().name == "reader" and not parked.is_set():
                    parked.set()
                    release.wait(10)

            ax.set_frame_logger(frame_logger)
            self.addCleanup(release.set)
            reader = threading.Thread(target=ax.status, name="reader", daemon=True)
            reader.start()
            self.assertTrue(parked.wait(5))
            stopper = threading.Thread(target=ax.stop, name="stopper", daemon=True)
            stopper.start()
            stopper.join(5)
            stopped_while_parked = not stopper.is_alive()
            release.set()
            reader.join(5)
            stopper.join(5)
            ax.set_frame_logger(None)
            self.assertTrue(stopped_while_parked)
            self.assertFalse(reader.is_alive())


# ====================================================================== 0.4: bus lock, preemptible reads, IAI, line


class _ReadInterrupted(Exception):
    """What a preemptible read's gate raises in the tests (the GUI raises its own ReadInterrupted)."""


class Abi04LayoutTest(unittest.TestCase):
    def test_iai_flags(self) -> None:
        self.assertEqual(native._iai_flags(False, False, True), 0)
        self.assertEqual(native._iai_flags(True, True, False),
                         native.IAD_IAI_ALLOW_UNSAFE | native.IAD_IAI_ALLOW_NEVER_SEND | native.IAD_IAI_NO_CHECK)

    def test_a_gate_exception_is_raised_as_is(self) -> None:
        cause = _ReadInterrupted("STOP")
        exc = native._make_error(native.IAD_ERR_CANCELLED, {"callback_exception": cause, "callback_propagate": True})
        self.assertIs(exc, cause)
        other = native._make_error(native.IAD_ERR_TRANSPORT, {"callback_exception": cause})
        self.assertIsInstance(other, TransportError)
        self.assertIs(other.__cause__, cause)


@needs_native
class NativeAbi04Test(unittest.TestCase):
    def _bus(self, **sim_options: Any) -> tuple[native.NativeSim, native.NativeBus]:
        sim = native.NativeSim.open("slider", **sim_options)
        self.addCleanup(sim.close)
        bus = native.NativeBus.open(sim, retries=0, min_gap=0.0)
        self.addCleanup(bus.close)
        return sim, bus

    def test_exclusive_holds_other_threads_off(self) -> None:
        _, bus = self._bus()
        done = threading.Event()

        def other() -> None:
            bus.read_registers(1, 0x9000, 1)
            done.set()

        with bus.exclusive() as held:
            self.assertIs(held, bus)
            self.assertEqual(bus.lock_depth, 1)
            self.assertEqual(len(bus.read_registers(1, 0x9000, 2)), 2)  # this thread's own requests go through
            worker = threading.Thread(target=other)
            worker.start()
            self.assertFalse(done.wait(0.2))
            infos: list[dict[str, Any]] = []
            reader = threading.Thread(target=lambda: infos.append(bus.info()))
            reader.start()
            reader.join(2.0)
            self.assertEqual(len(infos), 1)  # an info read never waits for the hold
            self.assertEqual(infos[0]["lock_waiting"]["normal"], 1)  # `other` waits for it
            results: list[bool] = []
            timed = threading.Thread(target=lambda: results.append(bus.lock(0.05)))
            timed.start()
            timed.join()
            self.assertEqual(results, [False])
            self.assertTrue(bus.lock(0))  # recursive
            bus.unlock()
        worker.join(5.0)
        self.assertTrue(done.is_set())
        self.assertEqual(bus.lock_depth, 0)
        with self.assertRaises(ValueError):
            bus.unlock()

    def test_preemptible_reads_and_the_gate(self) -> None:
        sim, bus = self._bus()
        frames: list[bytes] = []
        bus.set_frame_sink(lambda direction, frame: frames.append(frame) if direction == "tx" else None)
        calls: list[int] = []
        stop_after: list[int] = []

        def gate() -> None:
            calls.append(1)
            if stop_after and len(calls) > stop_after[0]:
                raise _ReadInterrupted("STOP pressed")

        with bus.preemptible(gate):
            self.assertEqual(bus.info()["preemptible_words"], 32)
            self.assertEqual(len(bus.read_registers(1, 0x9000, 34)), 34)
            self.assertEqual(len(calls), 2)  # the gate before every request
            with self.assertRaises(UnsafeCommandError):
                bus.write_coil(1, 0x042C, True)
            with self.assertRaises(UnsafeCommandError):
                bus.broadcast_coil(0x042C, True)
            stop_after.append(len(calls) + 1)
            with self.assertRaises(_ReadInterrupted):
                bus.read_registers(1, 0x9000, 34)
        self.assertIsNone(bus.info()["preemptible_words"])
        bus.write_coil(1, 0x042C, False)  # outside: writes again
        sizes = [f[5] if f[1] == 0x03 else -1 for f in frames]
        self.assertEqual(sizes, [32, 2, 32, -1])
        self.assertFalse(sim.state()["stop_held"])

    def test_suspend_monitors(self) -> None:
        _, bus = self._bus(time_scale=TIME_SCALE)
        with bus.suspend_monitors() as held:
            self.assertEqual(held, 0)  # no monitor yet
        ax = native.NativeAxis.open_bus(bus, 1, poll_interval_s=0.005)
        self.addCleanup(ax.close)
        ax.connect()
        monitor = ax.start_monitor()
        deadline = time.monotonic() + 5.0
        while monitor.polls < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        with bus.suspend_monitors() as held:
            self.assertEqual(held, 1)
            self.assertEqual(bus.info()["suspended_monitors"], 1)
            polls = monitor.polls
            time.sleep(0.15)
            self.assertLessEqual(monitor.polls, polls + 1)
        self.assertEqual(bus.info()["suspended_monitors"], 0)
        deadline = time.monotonic() + 5.0
        while monitor.polls <= polls + 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertGreater(monitor.polls, polls + 2)
        ax.stop_monitor()

    def test_iai_flags_0x48_and_broadcasts(self) -> None:
        sim, bus = self._bus()
        seen: list[bytes] = []

        def hook(frame: bytes) -> Any:  # answer 0x48 here, with known status words
            if frame[1] != 0x48:
                return NotImplemented
            seen.append(frame)
            body = bytes([frame[0], 0x48, 0x00, 0x00]) + b"".join(struct.pack(">I", v) for v in (1, 2, 3, 4))
            return body + struct.pack("<H", _crc16(body))

        sim.set_request_hook(hook)
        with self.assertRaises(UnsafeCommandError):
            bus.iai_command(1, 0, 0x04, 0x14, 5, allow_unsafe=True)  # the baud change is on the never-send list
        self.assertEqual(bus.iai_command(1, 0, 0x0B, check=False)[0], 0)  # decel stop at rest
        self.assertEqual(bus.iai_command2(1, 0x30, [1]), (0, (1, 2, 3, 4)))
        self.assertEqual(seen[-1][:9], bytes([0x01, 0x48, 0x00, 0x00, 0x30, 0, 0, 0, 1]))
        self.assertEqual(len(seen[-1]), 23)
        with self.assertRaises(UnsafeCommandError):
            bus.iai_command2(1, 0x7777)
        with self.assertRaises(ValueError):
            bus.iai_command2(1, 0x30, [1, 2, 3, 4, 5])
        frame = bus.broadcast_iai_command(0, 0x09, 0, settle_s=0.0)
        self.assertEqual(frame[:8], bytes([0x00, 0x45, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00]))
        self.assertEqual(len(frame), 10)
        self.assertEqual(bus.broadcast_coil(0x042C, True, settle_s=0.0), bytes.fromhex("00 05 04 2C FF 00 4D 12"))
        self.assertTrue(sim.state()["stop_held"])
        bus.broadcast(bytes.fromhex("05 04 2C 00 00"), settle_s=0.0)
        self.assertFalse(sim.state()["stop_held"])
        with self.assertRaises(ValueError):
            bus.broadcast(bytes.fromhex("03 90 00 00 01"))  # never a read

    def test_line_and_response_timing(self) -> None:
        _, bus = self._bus()
        fc03 = bytes.fromhex("03 90 00 00 01")
        self.assertAlmostEqual(bus.response_timeout(1, fc03, 7), 0.2)
        bus.set_line("rcon", position_unit_mm=0.001)
        self.assertEqual(bus.line_kind, "rcon")
        info = bus.line_info()
        self.assertEqual(info["kind"], "rcon")
        self.assertAlmostEqual(info["position_units"]["all"]["unit_mm"], 0.001)
        self.assertAlmostEqual(bus.response_timeout(1, fc03, 7), 0.3)  # a gateway line's floor
        with self.assertRaises(UnsafeCommandError):  # LineRuleError on the native side: nothing sent
            bus.broadcast_coil(0x042C, True)
        with self.assertRaises(ValueError):
            bus.set_line("auto")
        with self.assertRaises(ValueError):
            bus.set_line("canbus")
        self.assertEqual(bus.detect_line(timeout=0.02), "direct")  # nothing answers D0h / 3Fh on the simulator
        self.assertEqual(bus.line_kind, "direct")
        self.assertEqual(bus.response_delay_ms(1), 5.0)
        fc03_60 = bytes.fromhex("03 90 00 00 3C")
        base = bus.response_timeout(1, fc03_60, 125)
        bus.set_response_delay(1, 405.0)
        self.assertEqual(bus.response_delay_ms(1), 405.0)
        self.assertAlmostEqual(bus.response_timeout(1, fc03_60, 125) - base, 0.4, places=6)
        with self.assertRaises(ValueError):
            bus.set_response_delay(1, -1.0)
        self.assertEqual(bus.unsupported_functions(1), frozenset())

    def test_per_call_read_options_and_baud_targets(self) -> None:
        _, bus = self._bus(baud_rate=38400, unit_baud_rate=115200)
        frames: list[bytes] = []
        bus.set_frame_sink(lambda direction, frame: frames.append(frame) if direction == "tx" else None)
        self.assertEqual(bus.detect_baudrate([9, (0xD0, 0x3700), 1], [115200, 38400], probe_timeout=0.02), 115200)
        self.assertTrue(any(f[0] == 0xD0 and f[2:4] == b"\x37\x00" for f in frames))  # the gateway probe went out
        with self.assertRaises(ModbusTimeout) as ctx:
            bus.detect_baudrate([(0xD0, 0x3700)], [38400], current_first=False, probe_timeout=0.02)
        self.assertIn("slave 0xD0 did not answer", str(ctx.exception))
        frames.clear()
        self.assertEqual(len(bus.read_registers(1, 0x9000, 34, chunk=10)), 34)
        self.assertEqual([f[5] for f in frames], [10, 10, 10, 4])
        started = time.perf_counter()
        with self.assertRaises(ModbusTimeout):
            bus.read_registers(9, 0x9000, 1, retries=0, max_wait=0.02)
        self.assertLess(time.perf_counter() - started, 1.0)
        self.assertGreaterEqual(len(bus.report_slave_id(1, timeout=None)), 3)

    def test_call_json_lists_its_functions(self) -> None:
        names = native.call_json("functions")
        self.assertIn("functions", names)
        with self.assertRaises(NotImplementedError):
            native.call_json("no_such_function")


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


if __name__ == "__main__":
    unittest.main()
