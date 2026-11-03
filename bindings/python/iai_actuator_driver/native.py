"""Python binding of the native core: the C ABI of ``core/include/iai_actuator_driver.h`` over ctypes.

The native library (``iai_actuator_driver.dll``, ``libiai_actuator_driver.so``, ``libiai_actuator_driver.dylib``) is
the C++ core built as one shared library (``python build.py shared`` in ``core/``). This module loads it with ctypes, so
no compiler is needed on the user's PC, and wraps its handles in :class:`NativeAxis` and :class:`NativeCell`, whose
method names follow :class:`~iai_actuator_driver.ec.Elecylinder` and :class:`~iai_actuator_driver.cell.Cell`. A failing
call raises this package's own exception classes (:mod:`iai_actuator_driver.errors`, ``TransportError``, ``CellError``)
as mapped in ``docs/NATIVE_ABI.md``, so code written against the package API catches the same types; each
raised exception also carries ``iad_code`` (the C result code) and ``iad_details`` (the library's last-error record).

Where the library is looked for (first match):

1. the environment variable ``IAI_ACTUATOR_DRIVER_NATIVE``: the library file, or a folder holding it (only there when
   it is set);
2. next to the package: ``iai_actuator_driver/_native/<platform>/`` (e.g. ``win-x64``), ``iai_actuator_driver/_native/``
   and ``iai_actuator_driver/``;
3. a source checkout: ``core/build/`` at the repository root (two folders above
   ``bindings/python/``).

Importing this module never loads the library; importing the package does (``_backend.py``), as does the first call
that needs it. :class:`NativeLibraryError` then says where it looked. :func:`available` asks without raising.

Scope (C ABI 1.0): open by port, on a transport or bus (:class:`NativeTransport`, :class:`NativeBus`, a Python
transport object the core drives) or on the native simulator (:class:`NativeSim`, :class:`NativeSimLine`, with the test
hooks of SimulatedEC), connect, identify, status, limits, servo, home, absolute /
relative / jog / stored-row moves (with a deceleration of their own, acceleration patterns and a completion rule),
push, speed and acceleration changes, move sequences, move handles (``wait=False``), stop, pause, resume, alarm reset
and history, position table, teach, parameters, the non-volatile write counter, baud change, software reset, the status
monitor with its callbacks, fast mode, link-loss recovery, unit information, backup, the self-test, gateway lines, the
action mode and safety speed, and the cell's group commands, link monitor, recovery and coordinated motion; the
clock seam for tests (:func:`virtual_clock`) and the cancel latch; (C ABI 0.4) on a :class:`NativeBus` the bus lock
held across calls (:meth:`NativeBus.exclusive`), preemptible read-only reads with a gate
(:meth:`NativeBus.preemptible`), suspending the bus's status monitors, the IAI opt-ins (``allow_never_send``), IAI
0x48, broadcasts, the line (stated or probed) and the response delay / timeout; (C ABI 0.5) the simulator clock,
the axis state without I/O, bus shutdown and release, the line and gateway objects, and ports and cells on the
caller's buses (the low-level calls behind the driver modules); (C ABI 0.6) a bus moved onto another
transport, recovery sessions and the link state, the information readers, the self-test's finished lines and the
simulator's gateway lines and ports; (C ABI 0.7) a call's own result read on its thread, one pass of a
cell's link monitor now, an abort answer for a hook or transport callback whose error is the caller's own, and the
test hooks of the non-volatile write guard and the port list. Method names
and shapes follow the package API's: the Python tests are the specification. Options the
core does not have are not accepted here (a ``TypeError`` for an unknown keyword, ``ValueError`` for an unknown
choice).

Threads: ctypes releases the GIL during each call, so :meth:`NativeAxis.stop`, :meth:`NativeAxis.pause` and
:meth:`NativeCell.stop_all` from another thread reach the core while a move waits on the same handle. Callbacks
(``on_change``, ``on_done``, ``on_event``, ``on_item``, ``on_fault``) run on a thread of this module that drains the
handle's event queue (``iad_axis_next_event`` / ``iad_cell_next_event``); no thread of the core ever calls into Python
except the optional log and frame sinks. Ctrl+C: a blocking motion call made on the main thread runs on a helper thread
while the main thread waits in short slices, so Ctrl+C reaches Python; the binding then ends the call with
``iad_axis_cancel`` / ``iad_cell_cancel`` (a decel stop) and raises ``KeyboardInterrupt``. A move handle waits in slices
of ``iad_move_handle_poll`` the same way. The library's last error is per thread; this module reads it on the thread
that made the failing call.

Handles are closed by ``close()``, by leaving a ``with`` block, or (when forgotten) when the object is collected. Close
them before the interpreter exits: an open port otherwise stays open until the process ends.

Evidence: the C++ core's command line is VERIFIED on the EC-GRBP8M gripper and on the PCON-CB with an RCP6; the ABI
0.1 part of this binding is VERIFIED on the PCON-CB with an RCP6 (steps in docs/NATIVE_ABI.md); the 0.2 additions are
tested on the native simulator only (UNVERIFIED on hardware); everything else UNVERIFIED.

Example::

    from iai_actuator_driver.native import NativeAxis

    with NativeAxis.open_sim("gripper", time_scale=20) as ax:    # or NativeAxis.open("COM3")
        print(ax.connect().model, ax.limits()["position"])
        ax.servo(True)
        ax.home()
        print(ax.move_to(5.0, speed_mm_s=20.0).position_mm)
"""

from __future__ import annotations

import atexit
import contextlib
import ctypes
import json
import logging
import math
import os
import platform
import struct
import sys
import threading
import time
import weakref
from collections.abc import Callable, Iterable, Iterator, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

# Only the shared errors at import: _backend imports this module before the driver modules, so it must build none of
# them (alarms, check, recovery, ec ...). Those (and values, which imports alarms) are imported where used.
from .errors import (
    CellError,
    EcAlarmError,
    EcDriverError,
    EcError,
    EcNotReadyError,
    IaiCommandRejected,
    LineRuleError,
    ModbusCRCError,
    ModbusError,
    ModbusException,
    ModbusFrameError,
    ModbusTimeout,
    MotionIncompleteError,
    MotionTimeoutError,
    NonVolatileWriteError,
    RecoveryError,
    ServoBlockedError,
    SoftLimitError,
    TransportError,
    UnitReplacedError,
    UnsafeCommandError,
    VerificationError,
)

if TYPE_CHECKING:
    from .check import CheckItem
    from .recovery import RecoveryEvent, RecoveryPolicy
    from .values import BaudChangeResult, PositionEntry

__all__ = [
    "ABI_VERSION_MAJOR",
    "ABI_VERSION_MINOR",
    "TESTING_ABI_VERSION",
    "ENV_VAR",
    "FAST_BAUDRATE",
    "NativeAxis",
    "NativeBus",
    "NativeCell",
    "NativeCheckReport",
    "NativeIdentity",
    "NativeLibraryError",
    "NativeLimits",
    "NativeLinkCheck",
    "NativeMonitor",
    "NativeMonitorSample",
    "NativeMoveHandle",
    "NativeOutOfRangeError",
    "NativeParameterWriteResult",
    "NativePushResult",
    "NativeRange",
    "NativeRecoveryReport",
    "NativeResetResult",
    "NativeSim",
    "NativeSimLine",
    "NativeStatus",
    "NativeTransport",
    "NativeVirtualClock",
    "abi_version",
    "advance_clock",
    "available",
    "backup_axes",
    "call_json",
    "clock_now",
    "find_library",
    "library_name",
    "library_path",
    "library_version",
    "list_ports",
    "load",
    "missing_functions",
    "result_name",
    "set_log_level",
    "set_log_sink",
    "sim_exception",
    "use_clock",
    "use_python_logging",
    "use_real_clock",
    "use_virtual_clock",
    "virtual_clock",
]

log = logging.getLogger(__name__)

# ====================================================================== ABI constants (iai_actuator_driver.h)

ABI_VERSION_MAJOR = 1
"""The ABI this binding is written for: the library must report this major and at least this minor. The minor stays 0
while no call of the binding needs a later one: the 1.1 functions (iad_transport_open_serial_options,
iad_serial_options_init) are bound when the library has them, and a 1.0 library lacks them (:func:`missing_functions`)
but is still accepted."""
ABI_VERSION_MINOR = 0

TESTING_ABI_VERSION = 1
"""The version of iai_actuator_driver_testing.h this binding is written for (IAD_TESTING_ABI_VERSION): the library's
must be exactly this before a test hook is bound (no stability promise between numbers)."""

ENV_VAR = "IAI_ACTUATOR_DRIVER_NATIVE"
"""Environment variable naming the native library file (or the folder holding it)."""

PURE_PYTHON_REMOVED = (
    "The pure-Python edition, which ran without the library, was removed: 1.0.x is the last release with it."
)
"""Said where the library is not found (:func:`load`) and to ``IAI_DRIVER_BACKEND=python`` (``_backend.py``)."""

IAD_INT_UNKNOWN = -(2**31)  # an int32_t output that is not known (INT32_MIN)
IAD_UNKNOWN = 2  # an iad_tribool that is not known (0.8: 2, as the header; an input 0 is false, any other value true)

IAD_OK = 0
IAD_ERR_SOFT_LIMIT = -1
IAD_ERR_NOT_READY = -2
IAD_ERR_ALARM = -3
IAD_ERR_MOTION_TIMEOUT = -4
IAD_ERR_MODBUS_TIMEOUT = -5
IAD_ERR_FRAME = -6
IAD_ERR_TRANSPORT = -7
IAD_ERR_VERIFICATION = -8
IAD_ERR_NONVOLATILE_REFUSED = -9
IAD_ERR_INVALID_ARGUMENT = -10
IAD_ERR_UNSUPPORTED = -11
IAD_ERR_INTERNAL = -12
IAD_ERR_BUFFER_TOO_SMALL = -13
IAD_ERR_SERVO_BLOCKED = -14
IAD_ERR_MOTION_INCOMPLETE = -15
IAD_ERR_SLAVE_EXCEPTION = -16
IAD_ERR_COMMAND_REJECTED = -17
IAD_ERR_UNSAFE_REFUSED = -18
IAD_ERR_CELL = -19
IAD_ERR_MODBUS = -20
IAD_ERR_DRIVER = -21
IAD_ERR_RECOVERY = -22
IAD_ERR_UNIT_REPLACED = -23
IAD_ERR_FILE = -24
IAD_ERR_CANCELLED = -25

RESULT_NAMES: dict[int, str] = {
    value: name for name, value in globals().items() if name == "IAD_OK" or name.startswith("IAD_ERR_")
}
"""Result code -> its name in the header ("IAD_ERR_SOFT_LIMIT"...)."""

# Event kinds (iad_event.kind) and the IAD_EVENTS_* bits of iad_monitor_options.events
IAD_EVENT_NONE = 0
IAD_EVENT_STATUS = 1
IAD_EVENT_DONE = 2
IAD_EVENT_LINK = 3
IAD_EVENT_FAULT = 4
IAD_EVENT_LINK_STATE = 5
IAD_EVENT_CHECK_ITEM = 6
IAD_EVENTS_ALL = 0x0F
# (0.6, py-recovery) what queued an event (iad_event.source; 0: not stated)
IAD_EVENT_SOURCE_MONITOR = 1
IAD_EVENT_SOURCE_CHECK_LINK = 2
IAD_EVENT_SOURCE_RECOVER = 3
IAD_EVENT_SOURCE_SET_LINK_STATE = 4
IAD_EVENT_SOURCE_CELL = 5

FAST_BAUDRATE = 230400
"""The rate :meth:`NativeAxis.enable_fast` raises the link to by default (IAD_FAST_BAUD_RATE)."""

# 0.4: the opt-ins of an IAI command (IAD_IAI_*)
IAD_IAI_ALLOW_UNSAFE = 0x1
IAD_IAI_ALLOW_NEVER_SEND = 0x2
IAD_IAI_NO_CHECK = 0x4

# CommandMethod; None = IAD_METHOD_DEFAULT (0, the axis option; 0.8: every enumeration's 0 is its default).
# "iai2" is EXPERIMENTAL (UNVERIFIED on any unit).
_METHODS = {"coil": 1, "iai": 2, "iai2": 3, "register": 4}
_METHOD_DEFAULT = 0
_BAUD_METHODS = {"direct": 0, "break": 1}
_TEACH_METHODS = {"table": 0, "coil": 1}
_ACCEL_PATTERNS = {"trapezoid": 1, "s_motion": 2, "filter": 3}  # None = IAD_ACCEL_DEFAULT (0, trapezoid)
_COMPLETIONS = {"safe": 1, "fast": 2}  # None = IAD_COMPLETION_DEFAULT (0, the axis option)
_LINES = {"auto": 0, "direct": 1, "rcon": 2, "robonet": 3, "robonet-gateway": 4}
_LOG_LEVELS = {0: 10, 1: 20, 2: 30, 3: 40}  # IAD_LOG_* -> logging.DEBUG .. logging.ERROR
_FRAME_DIRECTIONS = {0: "tx", 1: "rx", 2: "rx-partial"}  # IAD_FRAME_*, as the frame logger names them
_SLICE_S = 0.1  # longest wait in one native call while Ctrl+C must reach Python
_PUMP_WAIT_S = 0.02  # longest wait of an event thread for its queue: a flush waits at most this long for it
# IAD_ALARM_LEVEL_* -> the name of its alarms.LEVEL_* constant (alarms is imported at the first use, _alarm_level)
_ALARM_LEVELS = ("LEVEL_NONE", "LEVEL_MESSAGE", "LEVEL_OPERATION_CANCELLED", "LEVEL_COLD_START", "LEVEL_UNKNOWN")
_JSON_START = 64 * 1024  # first buffer size of a JSON output (docs/NATIVE_ABI.md: start with 64 KiB, grow once)

# ====================================================================== structs (header order, no implicit padding)

_c_bool = ctypes.c_int32  # iad_bool
_c_tribool = ctypes.c_int32  # iad_tribool
_c_reserved4 = ctypes.c_uint64 * 4
_c_reserved2 = ctypes.c_uint64 * 2
_c_reserved8 = ctypes.c_uint64 * 8
AXIS_NAME_SIZE = 128
"""IAD_AXIS_NAME_SIZE (0.8: 128 bytes with the NUL; a longer name is refused, never cut)."""
_SIZED_HEADER = [("struct_size", ctypes.c_uint32), ("reserved_header", ctypes.c_uint32)]


class _Sized(ctypes.Structure):
    """(0.8) A sized input struct: ``struct_size`` and ``reserved_header`` first (the header's "Sized structs").
    Constructing one sets ``struct_size`` to this binding's size, so the library reads it whole and an ``_init``
    function fills it whole. An array made with ``(T * n)()`` is not constructed element by element: set each
    element's ``struct_size`` (:func:`_sized_array`)."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        if not self.struct_size:
            self.struct_size = ctypes.sizeof(self)


def _sized_array(cls: type[_Sized], n: int) -> ctypes.Array[Any]:
    """``n`` zeroed elements of a sized struct, each with its ``struct_size`` (the array's stride)."""
    items = (cls * n)()
    for item in items:
        item.struct_size = ctypes.sizeof(cls)
    return items


class AxisOptions(_Sized):
    """iad_axis_options (104 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("stroke_min", ctypes.c_double),
        ("stroke_max", ctypes.c_double),
        ("max_speed_mm_s", ctypes.c_double),
        ("max_accel_g", ctypes.c_double),
        ("max_push_pct", ctypes.c_double),
        ("poll_interval_s", ctypes.c_double),
        ("settle_s", ctypes.c_double),
        ("method", ctypes.c_int32),
        ("auto_modbus_control", _c_bool),
        ("push_settle_s", ctypes.c_double),
        ("completion", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved2),
    ]


class OpenOptions(_Sized):
    """iad_open_options (72 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("read_timeout_s", ctypes.c_double),
        ("min_gap_s", ctypes.c_double),
        ("slave", ctypes.c_int32),
        ("baud_rate", ctypes.c_int32),
        ("retries", ctypes.c_int32),
        ("line", ctypes.c_int32),
        ("position_unit_mm", ctypes.c_double),
        ("fast", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved2),
    ]


class SimOptions(_Sized):
    """iad_sim_options (80 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("time_scale", ctypes.c_double),
        ("position_mm", ctypes.c_double),
        ("stroke_mm", ctypes.c_double),
        ("max_speed_mm_s", ctypes.c_double),
        ("baud_rate", ctypes.c_int32),
        ("pmsl", _c_bool),
        ("reserved", _c_reserved4),
    ]


class MoveOptions(_Sized):
    """iad_move_options (80 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("speed", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("band", ctypes.c_double),
        ("timeout_s", ctypes.c_double),
        ("wait", _c_bool),
        ("accel_pattern", ctypes.c_int32),
        ("decel_g", ctypes.c_double),
        ("completion", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved2),
    ]


class PositionMoveOptions(_Sized):
    """iad_position_move_options (64 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("timeout_s", ctypes.c_double),
        ("speed_pct", ctypes.c_double),
        ("wait", _c_bool),
        ("allow_above_caps", _c_bool),
        ("reserved", _c_reserved4),
    ]


class NvWriteOptions(_Sized):
    """iad_nv_write_options (56 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("allow_nonvolatile", _c_bool),
        ("force", _c_bool),
        ("allow_unsafe", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class ParamWriteOptions(_Sized):
    """iad_param_write_options (64 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("commit", _c_bool),
        ("allow_dangerous", _c_bool),
        ("allow_unlisted", _c_bool),
        ("allow_home_params", _c_bool),
        ("force", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class BaudOptions(_Sized):
    """iad_baud_options (56 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("settle_s", ctypes.c_double),
        ("method", ctypes.c_int32),
        ("allow_unverified_code", _c_bool),
        ("reserved", _c_reserved4),
    ]


class ResetOptions(_Sized):
    """iad_reset_options (56 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("timeout_s", ctypes.c_double),
        ("reset_wait_s", ctypes.c_double),
        ("reserved", _c_reserved4),
    ]


class CellOptions(_Sized):
    """iad_cell_options (96 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("read_timeout_s", ctypes.c_double),
        ("min_gap_s", ctypes.c_double),
        ("scan_timeout_s", ctypes.c_double),
        ("max_axes", ctypes.c_int32),
        ("baud_rate", ctypes.c_int32),
        ("retries", ctypes.c_int32),
        ("scan_first", ctypes.c_int32),
        ("scan_last", ctypes.c_int32),
        ("identify", _c_bool),
        ("stop_on_fault", _c_bool),
        ("line", ctypes.c_int32),
        ("position_unit_mm", ctypes.c_double),
        ("reserved", ctypes.c_uint64 * 3),
    ]


class ServoOptions(_Sized):
    """iad_servo_options (56 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("timeout_s", ctypes.c_double), ("wait", _c_bool), ("reserved0", ctypes.c_int32),
                ("reserved", _c_reserved4)]  # fmt: skip


class HomeOptions(_Sized):
    """iad_home_options (56 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("timeout_s", ctypes.c_double), ("wait", _c_bool), ("method", ctypes.c_int32),
                ("reserved", _c_reserved4)]  # fmt: skip


class StopOptions(_Sized):
    """iad_stop_options (56 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("timeout_s", ctypes.c_double), ("wait", _c_bool), ("reserved0", ctypes.c_int32),
                ("reserved", _c_reserved4)]  # fmt: skip


class WaitIdleOptions(_Sized):
    """iad_wait_idle_options (72 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("target", ctypes.c_double), ("band", ctypes.c_double), ("timeout_s", ctypes.c_double),
                ("completion", ctypes.c_int32), ("reserved0", ctypes.c_int32), ("reserved", _c_reserved4)]  # fmt: skip


class ChangeOptions(_Sized):
    """iad_change_options (56 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("timeout_s", ctypes.c_double), ("wait", _c_bool), ("reserved0", ctypes.c_int32),
                ("reserved", _c_reserved4)]  # fmt: skip


class CancelPausedMoveOptions(_Sized):
    """iad_cancel_paused_move_options (48 bytes; sized, 0.8)."""

    _fields_ = [*_SIZED_HEADER, ("timeout_s", ctypes.c_double), ("reserved", _c_reserved4)]


class Status(ctypes.Structure):
    """iad_status (208 bytes)."""

    _fields_ = [
        ("position_mm", ctypes.c_double),
        ("velocity_mm_s", ctypes.c_double),
        ("current_ma", ctypes.c_double),
        ("uptime_s", ctypes.c_double),
        ("alarm_code", ctypes.c_int32),
        ("alarm_level", ctypes.c_int32),
        ("servo_on", _c_bool),
        ("homed", _c_bool),
        ("in_position", _c_bool),
        ("moving", _c_bool),
        ("alarm", _c_bool),
        ("warning", _c_bool),
        ("modbus_enabled", _c_bool),
        ("emergency_stop", _c_bool),
        ("paused", _c_bool),
        ("homing", _c_bool),
        ("safety_speed", _c_bool),
        ("ready", _c_bool),
        ("idle", _c_bool),
        ("target_reached", _c_bool),
        ("push_missed", _c_bool),
        ("pushing", _c_bool),
        ("emg_input", _c_tribool),
        ("drive_power_cut", _c_tribool),
        ("enable_off", _c_tribool),
        ("raw_words", ctypes.c_int32),
        ("raw_dss1", ctypes.c_uint32),
        ("raw_dss2", ctypes.c_uint32),
        ("raw_dsse", ctypes.c_uint32),
        ("raw_dipm", ctypes.c_uint32),
        ("raw_dopm", ctypes.c_uint32),
        ("rcon", _c_bool),
        ("reserved", _c_reserved8),  # (0.8) 64 bytes
    ]


class Range(ctypes.Structure):
    """iad_range (88 bytes)."""

    _fields_ = [
        ("min", ctypes.c_double),
        ("max", ctypes.c_double),
        ("unit_max", ctypes.c_double),
        ("configured", ctypes.c_double),
        ("recommended_min", ctypes.c_double),
        ("available", _c_tribool),
        ("count", ctypes.c_int32),
        ("unit", ctypes.c_char * 8),
        ("reserved", _c_reserved4),  # (0.8) 32 bytes
    ]


_RANGE_NAMES = (
    "position", "speed", "accel", "decel", "accel_decel", "push", "band", "push_band", "stored_rows", "jog_step",
    "speed_override",
)


class Limits(ctypes.Structure):
    """iad_limits (1096 bytes)."""

    _fields_ = [
        *((name, Range) for name in _RANGE_NAMES),
        ("units", ctypes.c_char * 8),
        ("kind", ctypes.c_char * 24),
        ("family", ctypes.c_char * 24),
        ("push_speed", ctypes.c_double),
        ("reserved", _c_reserved8),  # (0.8) 64 bytes
    ]


class Identity(ctypes.Structure):
    """iad_identity (616 bytes)."""

    _fields_ = [
        ("soft_min_mm", ctypes.c_double),
        ("soft_max_mm", ctypes.c_double),
        ("max_speed_mm_s", ctypes.c_double),
        ("max_accel_g", ctypes.c_double),
        ("max_decel_g", ctypes.c_double),
        ("slave", ctypes.c_int32),
        ("system_code", ctypes.c_int32),
        ("unit_code", ctypes.c_int32),
        ("run_status", ctypes.c_int32),
        ("axis_number", ctypes.c_int32),
        ("max_positions", ctypes.c_int32),
        ("actuator_kind", ctypes.c_int32),
        ("unit_known", _c_tribool),
        ("rotary", _c_tribool),
        ("reserved0", ctypes.c_int32),
        ("product_family", ctypes.c_char * 16),
        ("model", ctypes.c_char * 64),
        ("model_name", ctypes.c_char * 64),
        ("controller_name", ctypes.c_char * 32),
        ("controller", ctypes.c_char * 64),
        ("unit_name", ctypes.c_char * 48),
        ("system_name", ctypes.c_char * 48),
        ("firmware", ctypes.c_char * 8),
        ("serial_number", ctypes.c_char * 32),
        ("actuator_serial", ctypes.c_char * 32),
        ("user_name", ctypes.c_char * 48),
        ("units", ctypes.c_char * 8),
        ("kind", ctypes.c_char * 24),
        ("identity_source", ctypes.c_char * 16),
        ("reserved", _c_reserved4),
    ]


class CPositionEntry(_Sized):
    """iad_position_entry (128 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("position_mm", ctypes.c_double),
        ("band_mm", ctypes.c_double),
        ("speed_mm_s", ctypes.c_double),
        ("zone_plus_mm", ctypes.c_double),
        ("zone_minus_mm", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("decel_g", ctypes.c_double),
        ("push_power", ctypes.c_int32),
        ("load_threshold", ctypes.c_int32),
        ("control_flags", ctypes.c_int32),
        ("checksum", ctypes.c_int32),
        ("checksum_ok", _c_tribool),
        ("is_push", _c_bool),
        ("incremental", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class Parameter(ctypes.Structure):
    """iad_parameter (288 bytes)."""

    _fields_ = [
        ("value", ctypes.c_double),
        ("min", ctypes.c_double),
        ("max", ctypes.c_double),
        ("number", ctypes.c_int32),
        ("writable", _c_bool),
        ("dangerous", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("name", ctypes.c_char * 64),
        ("symbol", ctypes.c_char * 16),
        ("item", ctypes.c_char * 16),
        ("address", ctypes.c_char * 16),
        ("text", ctypes.c_char * 64),
        ("unit", ctypes.c_char * 16),
        ("raw", ctypes.c_char * 16),
        ("family", ctypes.c_char * 8),
        ("reserved", _c_reserved4),
    ]


class ParamWriteResult(ctypes.Structure):
    """iad_param_write_result (320 bytes)."""

    _fields_ = [
        ("number", ctypes.c_int32),
        ("changed", _c_bool),
        ("verified", _c_bool),
        ("committed", _c_bool),
        ("commit_confirmed", _c_bool),
        ("saved", _c_bool),
        ("skipped", _c_bool),
        ("forced", _c_bool),
        ("listed_for_model", _c_bool),
        ("verified_for_model", _c_bool),
        ("home_parameter", _c_bool),
        ("warning_count", ctypes.c_int32),
        ("key", ctypes.c_char * 16),
        ("address", ctypes.c_char * 16),
        ("name", ctypes.c_char * 64),
        ("before", ctypes.c_char * 64),
        ("after", ctypes.c_char * 64),
        ("unit", ctypes.c_char * 16),
        ("reserved", _c_reserved4),
    ]


class PositionWriteResult(ctypes.Structure):
    """iad_position_write_result (136 bytes; 0.8: the row read back in fields of its own, those of
    :class:`CPositionEntry`, so :func:`_entry_from_c` reads it as it is)."""

    _fields_ = [
        ("position_mm", ctypes.c_double),
        ("band_mm", ctypes.c_double),
        ("speed_mm_s", ctypes.c_double),
        ("zone_plus_mm", ctypes.c_double),
        ("zone_minus_mm", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("decel_g", ctypes.c_double),
        ("push_power", ctypes.c_int32),
        ("load_threshold", ctypes.c_int32),
        ("control_flags", ctypes.c_int32),
        ("checksum", ctypes.c_int32),
        ("checksum_ok", _c_tribool),
        ("is_push", _c_bool),
        ("incremental", _c_bool),
        ("row", ctypes.c_int32),
        ("skipped", _c_bool),
        ("forced", _c_bool),
        ("warning_count", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class ResetResult(ctypes.Structure):
    """iad_reset_result (352 bytes)."""

    _fields_ = [
        ("status", Status),
        ("uptime_before_s", ctypes.c_double),
        ("uptime_after_s", ctypes.c_double),
        ("elapsed_s", ctypes.c_double),
        ("accepted", _c_bool),
        ("reply_lost", _c_bool),
        ("confirmed", _c_bool),
        ("baud_rate", ctypes.c_int32),
        ("baud_changed", _c_bool),
        ("needs_home", _c_bool),
        ("confirmed_by", ctypes.c_char * 64),
        ("reserved", _c_reserved4),
    ]


class BaudResult(ctypes.Structure):
    """iad_baud_result (128 bytes)."""

    _fields_ = [
        ("previous", ctypes.c_int32),
        ("baud_rate", ctypes.c_int32),
        ("method", ctypes.c_int32),
        ("code", ctypes.c_int32),
        ("verified", _c_bool),
        ("probed", _c_bool),
        ("code_verified", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("frame", ctypes.c_char * 64),
        ("reserved", _c_reserved4),
    ]


class CellResult(ctypes.Structure):
    """iad_cell_result (632 bytes)."""

    _fields_ = [
        ("status", Status),
        ("code", ctypes.c_int32),
        ("index", ctypes.c_int32),  # (0.8) the position in the requested list (cell order for all axes)
        ("axis", ctypes.c_char * AXIS_NAME_SIZE),
        ("message", ctypes.c_char * 256),
        ("reserved", _c_reserved4),
    ]


# 0.2 structs


class PushOptions(_Sized):
    """iad_push_options (72 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("speed", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("timeout_s", ctypes.c_double),
        ("direction", ctypes.c_int32),
        ("wait", _c_bool),
        ("reserved", _c_reserved4),
    ]


class PushResult(ctypes.Structure):
    """iad_push_result (272 bytes)."""

    _fields_ = [
        ("status", Status),
        ("position_mm", ctypes.c_double),
        ("outcome", ctypes.c_int32),
        ("warning_count", ctypes.c_int32),
        ("outcome_name", ctypes.c_char * 16),
        ("reserved", _c_reserved4),
    ]


class SequencePoint(_Sized):
    """iad_sequence_point (56 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("position", ctypes.c_double),
        ("speed", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("band", ctypes.c_double),
        ("reserved", _c_reserved2),
    ]


class SequenceOptions(_Sized):
    """iad_sequence_options (88 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("speed", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("band", ctypes.c_double),
        ("blend_band", ctypes.c_double),
        ("timeout_s", ctypes.c_double),
        ("completion", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class MoveInfo(ctypes.Structure):
    """iad_move_info (296 bytes)."""

    _fields_ = [
        ("result", Status),
        ("target", ctypes.c_double),
        ("band", ctypes.c_double),
        ("timeout_s", ctypes.c_double),
        ("sent_at_s", ctypes.c_double),
        ("deadline_s", ctypes.c_double),
        ("completion", ctypes.c_int32),
        ("has_result", _c_bool),
        ("cancelled", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class MonitorOptions(_Sized):
    """iad_monitor_options (72 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("gap_s", ctypes.c_double),
        ("link_check_s", ctypes.c_double),
        ("retries", ctypes.c_int32),
        ("events", ctypes.c_int32),
        ("queue_capacity", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class MonitorSample(ctypes.Structure):
    """iad_monitor_sample (304 bytes)."""

    _fields_ = [
        ("status", Status),
        ("t_start_s", ctypes.c_double),
        ("t_s", ctypes.c_double),
        ("age_s", ctypes.c_double),
        ("poll_rate_hz", ctypes.c_double),
        ("seq", ctypes.c_int64),
        ("polls", ctypes.c_int64),
        ("error_count", ctypes.c_int64),
        ("running", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class Event(ctypes.Structure):
    """iad_event (736 bytes)."""

    _fields_ = [
        ("status", Status),
        ("time_s", ctypes.c_double),
        ("wall_time_s", ctypes.c_double),
        ("sequence", ctypes.c_int64),
        ("kind", ctypes.c_int32),
        ("dropped", ctypes.c_int32),
        ("axis", ctypes.c_char * AXIS_NAME_SIZE),
        ("name", ctypes.c_char * 32),
        ("cause", ctypes.c_char * 32),
        ("message", ctypes.c_char * 256),
        ("source", ctypes.c_int32),  # (0.6, py-recovery) IAD_EVENT_SOURCE_*
        ("index", ctypes.c_int32),  # (0.8) a cell's event: the axis's position in the cell; else IAD_INT_UNKNOWN
        ("caller_thread", ctypes.c_uint64),  # (0.7) the OS id of the thread that queued it; 0: not stated
        ("reserved", _c_reserved4),
    ]


class FastSummary(ctypes.Structure):
    """iad_fast_summary (304 bytes)."""

    _fields_ = [
        ("baud_rate", ctypes.c_int32),
        ("completion", ctypes.c_int32),
        ("monitor", _c_bool),
        ("note_count", ctypes.c_int32),
        ("notes", ctypes.c_char * 256),
        ("reserved", _c_reserved4),
    ]


class TrackOptions(_Sized):
    """iad_track_options (48 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("alarm_history", _c_bool),
        ("parameters", _c_bool),
        ("reserved", _c_reserved4),
    ]


class LinkCheck(ctypes.Structure):
    """iad_link_check (800 bytes)."""

    _fields_ = [
        ("status", Status),
        ("elapsed_s", ctypes.c_double),
        ("ok", _c_bool),
        ("reason_count", ctypes.c_int32),
        ("state", ctypes.c_char * 32),
        ("reasons", ctypes.c_char * 256),
        ("error", ctypes.c_char * 256),
        ("reserved", _c_reserved4),
    ]


class CRecoveryPolicy(_Sized):
    """iad_recovery_policy (80 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("position_tolerance_mm", ctypes.c_double),
        ("poll_s", ctypes.c_double),
        ("boot_baud_rate", ctypes.c_int32),
        ("restore_baud_rate", _c_bool),
        ("restore_control", _c_bool),
        ("restore_safety_speed", _c_bool),
        ("clear_message_alarms", _c_bool),
        ("check_parameters", _c_bool),
        ("require_session", _c_bool),  # (0.6, py-recovery)
        ("reserved0", ctypes.c_int32),
        ("reserved", ctypes.c_uint64 * 3),
    ]


class RecoverySummary(ctypes.Structure):
    """iad_recovery_summary (768 bytes)."""

    _fields_ = [
        ("status", Status),
        ("position_mm", ctypes.c_double),
        ("last_target_mm", ctypes.c_double),
        ("position_error_mm", ctypes.c_double),
        ("elapsed_s", ctypes.c_double),
        ("baud_rate", ctypes.c_int32),
        ("baud_rate_found", ctypes.c_int32),
        ("alarm_code", ctypes.c_int32),
        ("restarted", _c_bool),
        ("same_unit", _c_bool),
        ("port_reopened", _c_bool),
        ("baud_restored", _c_bool),
        ("control_restored", _c_bool),
        ("safety_speed_restored", _c_bool),
        ("needs_home", _c_bool),
        ("needs_attention", _c_bool),
        ("ready_to_move", _c_bool),
        ("fault_alarm", _c_bool),
        ("at_target", _c_tribool),
        ("servo_on", _c_tribool),
        ("homed", _c_tribool),
        ("modbus_control", _c_tribool),
        ("safety_speed", _c_tribool),
        ("note_count", ctypes.c_int32),
        ("control_kept", _c_bool),  # (0.6, py-recovery)
        ("axis", ctypes.c_char * AXIS_NAME_SIZE),
        ("cause", ctypes.c_char * 32),
        ("summary", ctypes.c_char * 256),
        ("reserved", _c_reserved4),
    ]


class CheckOptions(_Sized):
    """iad_check_options (72 bytes; sized, 0.8)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("step_mm", ctypes.c_double),
        ("speed_mm_s", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("motion", _c_bool),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class CheckSummary(ctypes.Structure):
    """iad_check_summary (56 bytes)."""

    _fields_ = [
        ("passed", _c_bool),
        ("items", ctypes.c_int32),
        ("failed", ctypes.c_int32),
        ("warned", ctypes.c_int32),
        ("skipped", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class AxisMove(_Sized):
    """iad_axis_move (224 bytes; sized, 0.8: fields of its own, NaN or 0 = the default)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("position", ctypes.c_double),
        ("speed", ctypes.c_double),
        ("accel_g", ctypes.c_double),
        ("band", ctypes.c_double),
        ("timeout_s", ctypes.c_double),
        ("decel_g", ctypes.c_double),
        ("accel_pattern", ctypes.c_int32),
        ("completion", ctypes.c_int32),
        ("axis", ctypes.c_char * AXIS_NAME_SIZE),
        ("reserved", _c_reserved4),
    ]


class MotionCaps(ctypes.Structure):
    """iad_motion_caps (336 bytes)."""

    _fields_ = [
        ("speed_cap", ctypes.c_double),
        ("accel_cap", ctypes.c_double),
        ("accel_only_cap", ctypes.c_double),
        ("decel_cap", ctypes.c_double),
        ("push_cap", ctypes.c_double),
        ("min_speed", ctypes.c_double),
        ("min_speed_source", ctypes.c_char * 256),
        ("reserved", _c_reserved4),
    ]


class BusOptions(ctypes.Structure):
    """iad_bus_options (0.3, 56 bytes; sized: struct_size first)."""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("reserved_header", ctypes.c_uint32),
        ("min_gap_s", ctypes.c_double),
        ("retries", ctypes.c_int32),
        ("reserved0", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


class SerialOptions(_Sized):
    """iad_serial_options (1.1, 64 bytes; sized): the settings of iad_transport_open_serial_options, the character
    framing among them (data bits 7 or 8, parity IAD_PARITY_NONE 0 / EVEN 1 / ODD 2, stop bits 1 or 2; 8N1 by
    default)."""

    _fields_ = [
        *_SIZED_HEADER,
        ("read_timeout_s", ctypes.c_double),
        ("baud_rate", ctypes.c_int32),
        ("data_bits", ctypes.c_int32),
        ("parity", ctypes.c_int32),
        ("stop_bits", ctypes.c_int32),
        ("reserved", _c_reserved4),
    ]


STRUCT_SIZES: dict[type[ctypes.Structure], int] = {
    # 0.8 inputs (sized: struct_size first)
    AxisOptions: 104, OpenOptions: 72, SimOptions: 80, MoveOptions: 80, PositionMoveOptions: 64, NvWriteOptions: 56,
    ParamWriteOptions: 64, BaudOptions: 56, ResetOptions: 56, CellOptions: 96, CPositionEntry: 128, PushOptions: 72,
    SequencePoint: 56, SequenceOptions: 88, MonitorOptions: 72, TrackOptions: 48, CRecoveryPolicy: 80,
    CheckOptions: 72, AxisMove: 224, ServoOptions: 56, HomeOptions: 56, StopOptions: 56, WaitIdleOptions: 72,
    ChangeOptions: 56, CancelPausedMoveOptions: 48,
    # 0.8 outputs (fixed for the major)
    Status: 208, Range: 88, Limits: 1096, Identity: 616, Parameter: 288, ParamWriteResult: 320,
    PositionWriteResult: 136, ResetResult: 352, BaudResult: 128, CellResult: 632, PushResult: 272, MoveInfo: 296,
    MonitorSample: 304, Event: 736, FastSummary: 304, LinkCheck: 800, RecoverySummary: 768, CheckSummary: 56,
    MotionCaps: 336,
    # 0.3 (iad_transport_callbacks holds pointers: its size follows the pointer size, TRANSPORT_CALLBACKS_SIZE)
    BusOptions: 56,
    # 1.1 inputs (sized; the first size is the 1.1 size)
    SerialOptions: 64,
}
"""The size of every struct (the same numbers c_api.cpp checks): an output's is frozen within an ABI major version, a
sized input's is its first size (0.8; a later minor only appends)."""

# ====================================================================== function signatures (header order)

_P = ctypes.POINTER
_R = ctypes.c_int32  # iad_result
_H = ctypes.c_void_p  # iad_axis* / iad_cell* (a token, passed by value)
_HOUT = _P(ctypes.c_void_p)  # iad_axis** / iad_cell**
_I = ctypes.c_int32
_D = ctypes.c_double
_STR = ctypes.c_char_p  # const char* in (UTF-8)
_Z = ctypes.c_size_t
_BUF = [_P(ctypes.c_char), _Z, _P(_Z)]  # (char* buffer, size_t buffer_size, size_t* required)
_RESULTS = [_P(CellResult), _Z, _P(_Z)]  # (iad_cell_result* results, size_t capacity, size_t* count)
# IAD_CALL (0.8): __stdcall on 32-bit Windows only (WinDLL, WINFUNCTYPE), the platform's one convention elsewhere (CDLL,
# CFUNCTYPE). No 32-bit Windows library is built yet; the rule is the header's.
_STDCALL = sys.platform == "win32" and struct.calcsize("P") == 4
_CALLBACK = ctypes.WINFUNCTYPE if _STDCALL else ctypes.CFUNCTYPE  # type: ignore[attr-defined]
# The sinks (IAD_CALL): iad_log_sink(level, source, message, user) and iad_frame_sink(direction, bytes, length, user)
LOG_SINK = _CALLBACK(None, ctypes.c_int32, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p)
FRAME_SINK = _CALLBACK(None, ctypes.c_int32, _P(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_void_p)
# 0.3 callbacks (IAD_CALL): a transport's (the user pointer first), the clock's and a simulator's request hook
TRANSPORT_WRITE = _CALLBACK(ctypes.c_int32, ctypes.c_void_p, _P(ctypes.c_uint8), ctypes.c_size_t)
TRANSPORT_READ = _CALLBACK(
    ctypes.c_int32, ctypes.c_void_p, _P(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_double, _P(ctypes.c_size_t)
)
TRANSPORT_RESET_INPUT = _CALLBACK(ctypes.c_int32, ctypes.c_void_p)
TRANSPORT_SET_BAUD_RATE = _CALLBACK(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int32)
TRANSPORT_SEND_BREAK = _CALLBACK(ctypes.c_int32, ctypes.c_void_p, ctypes.c_double)
TRANSPORT_PRESENT = _CALLBACK(ctypes.c_int32, ctypes.c_void_p)
TRANSPORT_REOPEN = _CALLBACK(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int32)
TRANSPORT_CLOSE = _CALLBACK(None, ctypes.c_void_p)
TRANSPORT_RELEASE = _CALLBACK(None, ctypes.c_void_p)
CLOCK_NOW = _CALLBACK(ctypes.c_double, ctypes.c_void_p)
CLOCK_SLEEP = _CALLBACK(None, ctypes.c_double, ctypes.c_void_p)
SIM_REQUEST_HOOK = _CALLBACK(
    ctypes.c_int32, _P(ctypes.c_uint8), ctypes.c_size_t, _P(ctypes.c_uint8), ctypes.c_size_t, _P(ctypes.c_size_t),
    ctypes.c_void_p,
)
# 0.4: a preemptible read's gate (iad_read_gate_fn)
READ_GATE = _CALLBACK(ctypes.c_int32, ctypes.c_void_p)
# 0.5 (core-bus): a transport's set_read_timeout (iad_transport_set_read_timeout_fn)
TRANSPORT_SET_READ_TIMEOUT = _CALLBACK(ctypes.c_int32, ctypes.c_void_p, ctypes.c_double)
# 0.5 (py-cell): a port's line detection by its owner (iad_port_line_fn)
PORT_LINE = _CALLBACK(ctypes.c_int32, ctypes.c_void_p)


class TransportCallbacks(ctypes.Structure):
    """iad_transport_callbacks (0.3; sized: struct_size first; holds function pointers, so 104 bytes on a 64-bit build,
    64 on a 32-bit one). (0.5) set_read_timeout is appended: set struct_size to this size before
    iad_transport_callbacks_init, else the library fills and reads the 0.3 part only (96 bytes on 64-bit)."""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("reserved_header", ctypes.c_uint32),
        ("read_timeout_s", ctypes.c_double),
        ("baud_rate", ctypes.c_int32),
        ("real_timeout", _c_bool),
        ("write", TRANSPORT_WRITE),
        ("read", TRANSPORT_READ),
        ("reset_input", TRANSPORT_RESET_INPUT),
        ("set_baud_rate", TRANSPORT_SET_BAUD_RATE),
        ("send_break", TRANSPORT_SEND_BREAK),
        ("present", TRANSPORT_PRESENT),
        ("reopen", TRANSPORT_REOPEN),
        ("close", TRANSPORT_CLOSE),
        ("release", TRANSPORT_RELEASE),
        ("set_read_timeout", TRANSPORT_SET_READ_TIMEOUT),  # 0.5 (core-bus)
    ]


TRANSPORT_CALLBACKS_SIZE = (24 + 10 * ctypes.sizeof(ctypes.c_void_p) + 7) // 8 * 8
"""sizeof(iad_transport_callbacks) on this build (104 on 64-bit)."""

_SIGNATURES: tuple[tuple[str, Any, list[Any]], ...] = (
    # library
    ("iad_abi_version", ctypes.c_uint32, []),
    ("iad_library_version", ctypes.c_char_p, []),
    ("iad_last_error_message", ctypes.c_char_p, []),
    ("iad_last_error_json", _R, [*_BUF]),
    ("iad_result_name", ctypes.c_char_p, [_R]),
    ("iad_list_ports_json", _R, [*_BUF]),
    # option defaults
    ("iad_axis_options_init", _R, [_P(AxisOptions)]),
    ("iad_open_options_init", _R, [_P(OpenOptions)]),
    ("iad_sim_options_init", _R, [_P(SimOptions)]),
    ("iad_move_options_init", _R, [_P(MoveOptions)]),
    ("iad_jog_options_init", _R, [_P(MoveOptions)]),
    ("iad_position_move_options_init", _R, [_P(PositionMoveOptions)]),
    ("iad_nv_write_options_init", _R, [_P(NvWriteOptions)]),
    ("iad_param_write_options_init", _R, [_P(ParamWriteOptions)]),
    ("iad_baud_options_init", _R, [_P(BaudOptions)]),
    ("iad_reset_options_init", _R, [_P(ResetOptions)]),
    ("iad_cell_options_init", _R, [_P(CellOptions)]),
    ("iad_position_entry_init", _R, [_P(CPositionEntry)]),
    ("iad_servo_options_init", _R, [_P(ServoOptions)]),  # 0.8: the options of the axis commands
    ("iad_home_options_init", _R, [_P(HomeOptions)]),
    ("iad_stop_options_init", _R, [_P(StopOptions)]),
    ("iad_wait_idle_options_init", _R, [_P(WaitIdleOptions)]),
    ("iad_change_options_init", _R, [_P(ChangeOptions)]),
    ("iad_cancel_paused_move_options_init", _R, [_P(CancelPausedMoveOptions)]),
    # axis: session
    ("iad_axis_open", _R, [_STR, _P(OpenOptions), _P(AxisOptions), _HOUT]),
    ("iad_axis_open_sim", _R, [_STR, _P(SimOptions), _P(OpenOptions), _P(AxisOptions), _HOUT]),
    ("iad_axis_close", _R, [_H]),
    ("iad_axis_connect", _R, [_H, _c_bool, _P(Identity)]),
    ("iad_axis_identify", _R, [_H, _P(Identity)]),
    ("iad_axis_identity_json", _R, [_H, *_BUF]),
    ("iad_axis_get_options", _R, [_H, _P(AxisOptions)]),
    ("iad_axis_set_options", _R, [_H, _P(AxisOptions)]),
    ("iad_axis_baud_rate", _R, [_H, _P(ctypes.c_int32), _P(_c_bool)]),
    ("iad_axis_modbus_control", _R, [_H, _c_bool]),
    # axis: reading
    ("iad_axis_status", _R, [_H, _P(Status)]),
    ("iad_axis_limits", _R, [_H, _P(Limits)]),
    ("iad_axis_limits_json", _R, [_H, *_BUF]),
    ("iad_axis_read_alarm_history_json", _R, [_H, _c_bool, *_BUF]),
    ("iad_axis_max_positions", _R, [_H, _P(ctypes.c_int32)]),
    ("iad_axis_read_position_table", _R, [_H, _I, _P(CPositionEntry)]),
    # axis: control
    ("iad_axis_servo", _R, [_H, _c_bool, _P(ServoOptions), _P(Status)]),
    ("iad_axis_home", _R, [_H, _P(HomeOptions), _P(Status)]),
    ("iad_axis_reset_alarm", _R, [_H, _I, _P(Status)]),
    # axis: motion
    ("iad_axis_check_move", _R, [_H, _D, _P(MoveOptions), _P(Status)]),
    ("iad_axis_move_to", _R, [_H, _D, _P(MoveOptions), _P(Status)]),
    ("iad_axis_move_by", _R, [_H, _D, _P(MoveOptions), _P(Status)]),
    ("iad_axis_jog", _R, [_H, _D, _P(MoveOptions), _P(Status)]),
    ("iad_axis_move_to_position", _R, [_H, _I, _P(PositionMoveOptions), _P(Status)]),
    ("iad_axis_change_speed", _R, [_H, _D, _P(ChangeOptions), _P(Status)]),
    ("iad_axis_wait_idle", _R, [_H, _P(WaitIdleOptions), _P(Status)]),
    ("iad_axis_stop", _R, [_H, _P(StopOptions), _P(Status)]),
    ("iad_axis_pause", _R, [_H]),
    ("iad_axis_resume", _R, [_H]),
    ("iad_axis_cancel_paused_move", _R, [_H, _P(CancelPausedMoveOptions), _P(Status)]),
    # axis: position table
    ("iad_axis_write_position_table", _R, [_H, _I, _P(CPositionEntry), _P(NvWriteOptions), _P(PositionWriteResult)]),
    ("iad_axis_teach_position", _R, [_H, _I, _I, _P(NvWriteOptions), _P(PositionWriteResult)]),
    # axis: parameters
    ("iad_axis_parameters_json", _R, [_H, *_BUF]),
    ("iad_axis_read_parameter", _R, [_H, _STR, _P(Parameter)]),
    ("iad_axis_read_parameter_json", _R, [_H, _STR, *_BUF]),
    ("iad_axis_write_parameter", _R, [_H, _STR, _STR, _P(ParamWriteOptions), _P(ParamWriteResult)]),
    ("iad_axis_write_parameter_number", _R, [_H, _STR, _D, _P(ParamWriteOptions), _P(ParamWriteResult)]),
    # axis: non-volatile guard
    ("iad_axis_nonvolatile_writes", _R, [_H, _P(ctypes.c_int64), _P(ctypes.c_int32), _P(_c_bool)]),
    # axis: unit
    ("iad_axis_set_baud_rate", _R, [_H, _I, _P(BaudOptions), _P(BaudResult)]),
    ("iad_axis_software_reset", _R, [_H, _P(ResetOptions), _P(ResetResult)]),
    ("iad_axis_last_result_json", _R, [_H, *_BUF]),
    # cell
    ("iad_cell_open", _R, [_STR, _P(CellOptions), _P(AxisOptions), _HOUT]),
    ("iad_cell_open_sim", _R, [_STR, _I, _P(SimOptions), _P(CellOptions), _P(AxisOptions), _HOUT]),
    ("iad_cell_close", _R, [_H]),
    ("iad_cell_axis_count", _R, [_H, _P(_Z)]),
    ("iad_cell_axis_name", _R, [_H, _Z, *_BUF]),
    ("iad_cell_open_axis", _R, [_H, _STR, _HOUT]),
    ("iad_cell_info_json", _R, [_H, *_BUF]),
    ("iad_cell_clear_fault", _R, [_H, _STR]),
    ("iad_cell_status_all", _R, [_H, _STR, _D, *_RESULTS]),
    ("iad_cell_servo_all", _R, [_H, _c_bool, _STR, _D, *_RESULTS]),
    ("iad_cell_home_all", _R, [_H, _STR, _D, _I, _c_bool, *_RESULTS]),
    ("iad_cell_move_all", _R, [_H, _STR, _P(ctypes.c_double), _Z, _P(MoveOptions), _P(ctypes.c_double),
                               _P(ctypes.c_double), _P(ctypes.c_double), _P(ctypes.c_double), *_RESULTS]),
    ("iad_cell_wait_all", _R, [_H, _STR, _D, _c_bool, *_RESULTS]),
    ("iad_cell_stop_all", _R, [_H, _STR, _D, _c_bool, *_RESULTS]),
    # 0.2: library
    ("iad_set_log_sink", _R, [LOG_SINK, ctypes.c_void_p]),
    ("iad_push_options_init", _R, [_P(PushOptions)]),
    ("iad_sequence_options_init", _R, [_P(SequenceOptions)]),
    ("iad_monitor_options_init", _R, [_P(MonitorOptions)]),
    ("iad_track_options_init", _R, [_P(TrackOptions)]),
    ("iad_recovery_policy_init", _R, [_P(CRecoveryPolicy)]),
    ("iad_check_options_init", _R, [_P(CheckOptions)]),
    # 0.2: axis motion
    ("iad_axis_push", _R, [_H, _D, _D, _D, _P(PushOptions), _P(PushResult)]),
    ("iad_axis_change_accel", _R, [_H, _D, _P(ChangeOptions), _P(Status)]),
    ("iad_axis_move_sequence", _R, [_H, _P(SequencePoint), _Z, _P(SequenceOptions), _P(Status), _Z, _P(_Z)]),
    # 0.2: move handles
    ("iad_axis_start_move_to", _R, [_H, _D, _P(MoveOptions), _HOUT]),
    ("iad_axis_start_move_by", _R, [_H, _D, _P(MoveOptions), _HOUT]),
    ("iad_axis_start_jog", _R, [_H, _D, _P(MoveOptions), _HOUT]),
    ("iad_move_handle_done", _R, [_H, _P(_c_bool), _P(Status)]),
    ("iad_move_handle_wait", _R, [_H, _D, _P(Status)]),
    ("iad_move_handle_poll", _R, [_H, _D, _P(_c_bool), _P(Status)]),
    ("iad_move_handle_cancel", _R, [_H, _c_bool, _D, _P(Status)]),
    ("iad_move_handle_result", _R, [_H, _P(MoveInfo)]),
    ("iad_move_handle_json", _R, [_H, *_BUF]),
    ("iad_move_handle_close", _R, [_H]),
    # 0.2: status monitor and events
    ("iad_axis_start_monitor", _R, [_H, _P(MonitorOptions)]),
    ("iad_axis_stop_monitor", _R, [_H, _D]),
    ("iad_axis_monitor_latest", _R, [_H, _P(MonitorSample)]),
    ("iad_axis_monitor_wait_update", _R, [_H, ctypes.c_int64, _D, _P(_c_bool), _P(MonitorSample)]),
    ("iad_axis_monitor_wait_done", _R, [_H, _D, _P(_c_bool), _P(Status)]),
    ("iad_axis_monitor_info_json", _R, [_H, *_BUF]),
    ("iad_axis_next_event", _R, [_H, _D, _P(Event)]),
    ("iad_axis_set_frame_sink", _R, [_H, FRAME_SINK, ctypes.c_void_p]),
    # 0.2: fast mode
    ("iad_axis_enable_fast", _R, [_H, _I, _c_bool, _P(FastSummary)]),
    ("iad_axis_disable_fast", _R, [_H, _P(FastSummary)]),
    ("iad_axis_fast_mode", _R, [_H, _P(_c_bool)]),
    # 0.2: recovery
    ("iad_axis_track", _R, [_H, _P(TrackOptions), _STR, _STR, _STR]),
    ("iad_axis_session_from_backup", _R, [_H, _STR, _STR]),
    ("iad_axis_session_from_backup_file", _R, [_H, _STR, _STR]),
    ("iad_axis_session_json", _R, [_H, *_BUF]),
    ("iad_axis_forget", _R, [_H]),
    ("iad_axis_note_target", _R, [_H, _D]),
    ("iad_axis_note_control", _R, [_H, _c_tribool, _c_tribool, _I, _c_tribool]),
    ("iad_axis_check_link", _R, [_H, _P(LinkCheck)]),
    ("iad_axis_link_state", _R, [_H, *_BUF]),
    ("iad_axis_recover", _R, [_H, _D, _P(CRecoveryPolicy), _STR, _STR, _STR, _P(RecoverySummary)]),
    # 0.2: information
    ("iad_axis_read_unit_info_json", _R, [_H, _STR, *_BUF]),
    ("iad_axis_backup_json", _R, [_H, _STR, _STR, *_BUF]),
    ("iad_backup_axes_json", _R, [_P(ctypes.c_void_p), _Z, _STR, _STR, _STR, *_BUF]),
    ("iad_axis_self_test", _R, [_H, _P(CheckOptions), _STR, _P(CheckSummary)]),
    # 0.2: line, unit modes
    ("iad_axis_line_json", _R, [_H, *_BUF]),
    ("iad_axis_gateway_status_json", _R, [_H, *_BUF]),
    ("iad_axis_set_position_unit", _R, [_H, _D]),
    ("iad_axis_set_action_mode", _R, [_H, _c_bool, _c_tribool, _c_bool, _P(Status)]),
    ("iad_axis_set_safety_speed", _R, [_H, _c_bool, _c_bool, _P(Status)]),
    ("iad_axis_cancel", _R, [_H]),
    ("iad_axis_set_strategy", _R, [_H, _I]),
    ("iad_axis_strategy", _R, [_H, _P(ctypes.c_int32)]),
    ("iad_axis_stop_path", _R, [_H, *_BUF]),
    ("iad_axis_line_name", _R, [_H, *_BUF]),
    ("iad_axis_read_monitor_json", _R, [_H, *_BUF]),
    ("iad_axis_read_lean_status", _R, [_H, _I, _P(Status)]),
    ("iad_axis_read_boot_baud_rate", _R, [_H, _P(ctypes.c_int32)]),
    ("iad_axis_gateway_monitor_json", _R, [_H, _STR, *_BUF]),
    ("iad_axis_motion_caps", _R, [_H, _P(MotionCaps)]),
    ("iad_axis_read_home_pattern", _R, [_H, _P(_c_bool), _P(ctypes.c_uint32)]),
    ("iad_axis_finish_home", _R, [_H]),
    # 0.2: cell
    ("iad_cell_move_each", _R, [_H, _P(AxisMove), _Z, _c_bool, _D, _c_bool, *_RESULTS]),
    ("iad_cell_pause_all", _R, [_H, _STR, _c_bool, *_RESULTS]),
    ("iad_cell_resume_all", _R, [_H, _STR, _c_bool, *_RESULTS]),
    ("iad_cell_cancel", _R, [_H, _STR]),
    ("iad_cell_start_monitors", _R, [_H, _STR, _P(MonitorOptions)]),
    ("iad_cell_stop_monitors", _R, [_H, _STR]),
    ("iad_cell_next_event", _R, [_H, _D, _P(Event)]),
    ("iad_cell_track_all", _R, [_H, _STR, _P(TrackOptions), _STR, *_RESULTS]),
    ("iad_cell_check_links", _R, [_H, _STR, _P(LinkCheck), _Z, _P(_Z)]),
    ("iad_cell_monitor_links", _R, [_H, _D, _STR]),
    ("iad_cell_stop_link_monitor", _R, [_H]),
    ("iad_cell_recover_all", _R, [_H, _STR, _D, _P(CRecoveryPolicy), _STR, *_RESULTS]),
    ("iad_cell_monitor_all_json", _R, [_H, _STR, *_BUF]),
    ("iad_cell_lines_json", _R, [_H, *_BUF]),
    ("iad_cell_gateway_status_json", _R, [_H, *_BUF]),
    ("iad_cell_set_position_unit", _R, [_H, _STR, _D, _STR]),
    ("iad_cell_gateway_monitor_json", _R, [_H, _STR, _STR, *_BUF]),
    ("iad_cell_backup_json", _R, [_H, _STR, _STR, _STR, *_BUF]),
    ("iad_cell_last_result_json", _R, [_H, *_BUF]),
    # 0.2: coordinated motion
    ("iad_cell_coord_plan_json", _R, [_H, _STR, *_BUF]),
    ("iad_cell_coord_run_json", _R, [_H, _STR, *_BUF]),
    # 0.3: library
    ("iad_callback_error", _R, [_STR]),
    ("iad_set_log_level", _R, [_I]),
    ("iad_transport_callbacks_init", _R, [_P(TransportCallbacks)]),
    ("iad_bus_options_init", _R, [_P(BusOptions)]),
    ("iad_serial_options_init", _R, [_P(SerialOptions)]),  # 1.1
    # 0.3: transports
    ("iad_transport_open_callbacks", _R, [_P(TransportCallbacks), _STR, ctypes.c_void_p, _HOUT]),
    ("iad_transport_open_serial", _R, [_STR, _I, _D, _HOUT]),
    ("iad_transport_open_serial_options", _R, [_STR, _P(SerialOptions), _HOUT]),  # 1.1
    ("iad_transport_close", _R, [_H]),
    ("iad_transport_release", _R, [_H]),
    ("iad_transport_info_json", _R, [_H, *_BUF]),
    ("iad_transport_write", _R, [_H, _P(ctypes.c_uint8), _Z]),
    ("iad_transport_read", _R, [_H, _P(ctypes.c_uint8), _Z, _P(_Z)]),
    ("iad_transport_reset_input", _R, [_H]),
    ("iad_transport_set_baud_rate", _R, [_H, _I]),
    ("iad_transport_set_read_timeout", _R, [_H, _D]),
    ("iad_transport_send_break", _R, [_H, _D]),
    ("iad_transport_reopen", _R, [_H, _I]),
    # 0.3: buses
    ("iad_bus_open", _R, [_H, _P(BusOptions), _HOUT]),
    ("iad_bus_close", _R, [_H]),
    ("iad_bus_transport", _R, [_H, _HOUT]),
    ("iad_bus_info_json", _R, [_H, *_BUF]),
    ("iad_bus_set_retries", _R, [_H, _I]),
    ("iad_bus_set_min_gap", _R, [_H, _D]),
    ("iad_bus_set_frame_sink", _R, [_H, FRAME_SINK, ctypes.c_void_p]),
    ("iad_bus_read_registers", _R, [_H, _I, _I, _I, _I, _D, _I, _P(ctypes.c_uint16)]),
    ("iad_bus_write_coil", _R, [_H, _I, _I, _c_bool, _I]),
    ("iad_bus_write_register", _R, [_H, _I, _I, ctypes.c_uint16]),
    ("iad_bus_write_registers", _R, [_H, _I, _I, _P(ctypes.c_uint16), _Z]),
    ("iad_bus_read_write_registers", _R, [_H, _I, _I, _I, _I, _P(ctypes.c_uint16), _Z, _P(ctypes.c_uint16)]),
    ("iad_bus_report_slave_id", _R, [_H, _I, _I, _D, _P(ctypes.c_uint8), _Z, _P(_Z)]),
    ("iad_bus_iai_command", _R, [_H, _I, _I, _I, _I, _P(ctypes.c_uint32), _Z, ctypes.c_uint32, _I,
                                 _P(ctypes.c_int32), _P(ctypes.c_uint32), _Z]),
    ("iad_bus_broadcast_coil", _R, [_H, _I, _c_bool]),
    ("iad_bus_probe", _R, [_H, _I, _D, _I, _P(_c_bool)]),
    ("iad_bus_detect_baud_rate", _R, [_H, _STR, _STR, _I, _c_bool, _D, _P(ctypes.c_int32)]),
    ("iad_bus_set_baud_rate", _R, [_H, _I]),
    ("iad_bus_send_break", _R, [_H, _D]),
    ("iad_bus_reopen", _R, [_H, _I]),
    # 0.3: simulators
    ("iad_sim_open", _R, [_STR, _HOUT]),
    ("iad_sim_close", _R, [_H]),
    ("iad_sim_transport", _R, [_H, _HOUT]),
    ("iad_sim_line_open", _R, [_STR, _P(ctypes.c_void_p), _Z, _HOUT]),
    ("iad_sim_line_close", _R, [_H]),
    ("iad_sim_line_add", _R, [_H, _H]),
    ("iad_sim_line_remove", _R, [_H, _I]),
    ("iad_sim_line_transport", _R, [_H, _HOUT]),
    # 0.3: axes and cells on transports and buses
    ("iad_axis_open_bus", _R, [_H, _I, _P(AxisOptions), _HOUT]),
    ("iad_axis_open_transport", _R, [_H, _P(OpenOptions), _P(AxisOptions), _HOUT]),
    ("iad_axis_bus", _R, [_H, _HOUT]),
    ("iad_cell_open_transports", _R, [_P(ctypes.c_void_p), _Z, _STR, _STR, _P(CellOptions), _P(AxisOptions),
                                      _HOUT]),
    ("iad_cell_bus", _R, [_H, _STR, _HOUT]),
    # 0.3: cancel latch
    ("iad_axis_cancel_latch", _R, [_H]),
    ("iad_axis_cancel_reset", _R, [_H]),
    ("iad_axis_cancel_latched", _R, [_H, _P(_c_bool)]),
    ("iad_cell_cancel_latch", _R, [_H, _STR]),
    ("iad_cell_cancel_reset", _R, [_H, _STR]),
    # 0.4: the bus lock, preemptible reads, status monitors
    ("iad_bus_lock", _R, [_H, _D, _P(_c_bool)]),
    ("iad_bus_unlock", _R, [_H]),
    ("iad_bus_preemptible_begin", _R, [_H, _I, READ_GATE, ctypes.c_void_p]),
    ("iad_bus_preemptible_end", _R, [_H]),
    ("iad_bus_suspend_monitors", _R, [_H, _D, _P(ctypes.c_int32)]),
    ("iad_bus_resume_monitors", _R, [_H]),
    # 0.4: broadcasts
    ("iad_bus_broadcast_iai_command", _R, [_H, _I, _I, _I, _I, ctypes.c_uint32, _D, _P(ctypes.c_uint8), _Z, _P(_Z)]),
    ("iad_bus_broadcast", _R, [_H, _P(ctypes.c_uint8), _Z, _D, _P(ctypes.c_uint8), _Z, _P(_Z)]),
    # 0.4: the line, response timing
    ("iad_bus_set_line", _R, [_H, _I, _D]),
    ("iad_bus_detect_line", _R, [_H, _I, _D, _D, _P(ctypes.c_int32)]),
    ("iad_bus_line_json", _R, [_H, *_BUF]),
    ("iad_bus_set_response_delay", _R, [_H, _I, _D]),
    ("iad_bus_response_delay", _R, [_H, _I, _P(ctypes.c_double)]),
    ("iad_bus_response_timeout", _R, [_H, _I, _P(ctypes.c_uint8), _Z, _I, _P(ctypes.c_double)]),
    # 0.4: stateless functions
    ("iad_call_json", _R, [_STR, _STR, *_BUF]),
    # 0.5 (py-axis): axis state
    ("iad_axis_last_status_words", _R, [_H, _P(Status), _P(ctypes.c_uint16), _Z, _P(_Z)]),
    ("iad_axis_state_json", _R, [_H, *_BUF]),
    ("iad_axis_kind_json", _R, [_H, *_BUF]),
    ("iad_axis_read_soft_limits_json", _R, [_H, *_BUF]),
    ("iad_axis_read_axis_limits_json", _R, [_H, *_BUF]),
    ("iad_axis_read_speed_limit", _R, [_H, _P(ctypes.c_double)]),
    ("iad_axis_read_support_words_json", _R, [_H, *_BUF]),
    ("iad_axis_categories_json", _R, [_H, *_BUF]),
    ("iad_axis_parameter_model_json", _R, [_H, _c_bool, *_BUF]),
    ("iad_axis_user_parameters_json", _R, [_H, *_BUF]),
    ("iad_axis_write_position_row", _R, [_H, _I, _P(CPositionEntry), _P(NvWriteOptions), _c_bool,
                                         _P(PositionWriteResult)]),
    ("iad_axis_enable_modbus_control", _R, [_H, _c_bool, _P(Status)]),
    ("iad_axis_begin_stop", _R, [_H]),
    ("iad_axis_push_speed_json", _R, [_H, _D, *_BUF]),
    ("iad_axis_servo_on_timeout_json", _R, [_H, _P(ctypes.c_uint16), _Z, *_BUF]),
    ("iad_axis_release_held", _R, [_H]),
    ("iad_axis_set_axis_limits_json", _R, [_H, _STR]),
    ("iad_axis_set_nv_unit", _R, [_H, _STR, _STR]),
    ("iad_nv_forget", _R, [_STR]),
    ("iad_constants_json", _R, [*_BUF]),
    # 0.5 (core-bus): buses
    ("iad_bus_shutdown", _R, [_H]),
    ("iad_bus_release", _R, [_H]),
    # 0.5 (py-monitor): status monitor
    ("iad_axis_suspend_monitor", _R, [_H, _D]),
    ("iad_axis_resume_monitor", _R, [_H]),
    ("iad_axis_monitor_target", _R, [_H, _P(ctypes.c_double), _P(ctypes.c_double)]),
    ("iad_axis_set_port_owner", _R, [_H, _c_bool]),
    # 0.5 (py-gateway): lines
    ("iad_line_open", _R, [_I, _H, _c_bool, _D, _HOUT]),
    ("iad_line_close", _R, [_H]),
    ("iad_bus_line", _R, [_H, _HOUT]),
    ("iad_bus_attach_line", _R, [_H, _H]),
    ("iad_bus_state_line_rules", _R, [_H, _STR]),
    ("iad_line_json", _R, [_H, *_BUF]),
    ("iad_line_allows", _R, [_H, _STR, _P(_c_bool)]),
    ("iad_line_require", _R, [_H, _STR, _STR]),
    ("iad_line_check_request", _R, [_H, _I, _P(ctypes.c_uint8), _Z]),
    ("iad_line_require_control", _R, [_H, _I]),
    ("iad_line_position_unit", _R, [_H, _I, _P(ctypes.c_double), *_BUF]),
    ("iad_line_set_position_unit", _R, [_H, _D, _I, _STR]),
    ("iad_line_require_position_unit", _R, [_H, _I, _c_bool, _P(ctypes.c_double)]),
    ("iad_line_note_unreachable", _R, [_H, _STR]),
    ("iad_line_explain_unreachable", _R, [_H, _STR, *_BUF]),
    ("iad_line_gateway", _R, [_H, _HOUT]),
    ("iad_line_rules_json", _R, [*_BUF]),
    ("iad_gateway_open", _R, [_H, _I, _I, _P(ctypes.c_uint16), _Z, _D, _D, _HOUT]),
    ("iad_gateway_close", _R, [_H]),
    ("iad_gateway_info_json", _R, [_H, *_BUF]),
    ("iad_gateway_status", _R, [_H, _D, _P(ctypes.c_uint16), _Z, _P(_Z), _P(ctypes.c_double)]),
    ("iad_gateway_require_manu", _R, [_H, _P(ctypes.c_uint16), _Z, _P(_Z), _P(ctypes.c_double)]),
    ("iad_gateway_read_alarm", _R, [_H, _P(ctypes.c_uint32)]),
    ("iad_gateway_set_cache_s", _R, [_H, _D]),
    ("iad_gateway_read_monitor_json", _R, [_H, _STR, *_BUF]),
    ("iad_bus_probe_gateway", _R, [_H, _I, _I, _D, _P(ctypes.c_uint16), _Z, _P(_Z), _P(ctypes.c_double)]),
    ("iad_bus_read_serial_code", _R, [_H, _I, *_BUF]),
    ("iad_bus_robonet_hint", _R, [_H, _I, _c_bool, *_BUF]),
    # 0.5 (py-cell): ports and cells
    ("iad_bus_set_port", _R, [_H, _STR, _I, _D, _c_bool, PORT_LINE, ctypes.c_void_p]),
    ("iad_bus_port_info_json", _R, [_H, *_BUF]),
    ("iad_bus_ensure_baud_rate", _R, [_H, _STR, _P(ctypes.c_int32), _P(_c_bool)]),
    ("iad_bus_port_detect_line", _R, [_H, _c_bool, _P(ctypes.c_int32)]),
    ("iad_bus_scan", _R, [_H, _STR, _D, _c_bool, _P(ctypes.c_int32), _Z, _P(_Z)]),
    ("iad_bus_scan_units_json", _R, [_H, _STR, _D, _c_bool, _c_bool, *_BUF]),
    ("iad_bus_identify_json", _R, [_H, _I, *_BUF]),
    ("iad_bus_set_baud_rate_all", _R, [_H, _I, _STR, _P(BaudOptions), *_BUF]),
    ("iad_cell_open_buses", _R, [_P(ctypes.c_void_p), _Z, _STR, _STR, _P(CellOptions), _P(AxisOptions), _HOUT]),
    ("iad_cell_open_axes", _R, [_P(ctypes.c_void_p), _Z, _STR, _P(CellOptions), _HOUT]),
    ("iad_cell_axis_ref", _R, [_H, _STR, _HOUT]),
    ("iad_cell_release", _R, [_H]),
    ("iad_cell_port_errors_json", _R, [_H, *_BUF]),
    ("iad_cell_read_all_json", _R, [_H, _STR, _STR, *_BUF]),
    # ABI 0.6 (csharp): a port's baud detection with candidates, one address's identity on a bus
    ("iad_bus_port_detect_baud_rate", _R, [_H, _STR, _STR, _D, _P(ctypes.c_int32)]),
    ("iad_bus_read_device_info_json", _R, [_H, _I, *_BUF]),
    # ABI 0.6 (fixes): a bus moved onto another transport in place
    ("iad_bus_set_transport", _R, [_H, _H]),
    # ABI 0.6 (py-recovery): a session from an identity and its fields, the link state, the port's presence, the
    # restart signs, the cell's link monitor
    ("iad_axis_session_from_info", _R, [_H, _STR, _STR]),
    ("iad_axis_session_update", _R, [_H, _STR]),
    ("iad_axis_set_link_state", _R, [_H, _STR, _STR, _STR, _P(_c_bool)]),
    ("iad_axis_port_present", _R, [_H, _P(ctypes.c_int32)]),
    ("iad_axis_restart_signs", _R, [_H, _P(ctypes.c_uint16), _Z, _STR, *_BUF]),
    ("iad_cell_link_monitor_json", _R, [_H, *_BUF]),
    # ABI 0.6 (py-info-cli): the directory readers by name, the lines of a running self-test
    ("iad_axis_info_call_json", _R, [_H, _STR, _STR, *_BUF]),
    ("iad_axis_self_test_items_json", _R, [_H, _I, *_BUF]),
    # ABI 0.6 (core-sim-gw): a simulator's port for a bus, the gateway lines
    ("iad_sim_port", _R, [_H, _HOUT]),
    ("iad_sim_line_port", _R, [_H, _HOUT]),
    ("iad_sim_gateway_line_open", _R, [_STR, _STR, _P(ctypes.c_void_p), _Z, _HOUT]),
    # ABI 0.7 (py-fixes): a call's own result, one pass of a cell's link monitor now
    ("iad_last_result_json", _R, [*_BUF]),
    ("iad_cell_link_monitor_check_now", _R, [_H, _STR, _P(LinkCheck), _Z, _P(_Z)]),
)
"""(name, restype, argtypes) of every function of iai_actuator_driver.h, in header order."""

_TESTING_SIGNATURES: tuple[tuple[str, Any, list[Any]], ...] = (
    ("iad_testing_abi_version", ctypes.c_uint32, []),
    # the library's clock
    ("iad_clock_use_real", _R, []),
    ("iad_clock_use_virtual", _R, [_D]),
    ("iad_clock_use_callbacks", _R, [CLOCK_NOW, CLOCK_SLEEP, ctypes.c_void_p]),
    ("iad_clock_advance", _R, [_D]),
    ("iad_clock_now", _R, [_P(ctypes.c_double)]),
    # simulator hooks
    ("iad_sim_state_json", _R, [_H, *_BUF]),
    ("iad_sim_requests_json", _R, [_H, *_BUF]),
    ("iad_sim_events_json", _R, [_H, *_BUF]),
    ("iad_sim_power_cycle", _R, [_H, _I]),
    ("iad_sim_power_off", _R, [_H]),
    ("iad_sim_power_on", _R, [_H, _I]),
    ("iad_sim_unplug", _R, [_H]),
    ("iad_sim_replug", _R, [_H]),
    ("iad_sim_set_word_override", _R, [_H, _I, _I]),
    ("iad_sim_set_unreadable", _R, [_H, _I, _I, _c_bool]),
    ("iad_sim_set_stale_status_polls", _R, [_H, _I]),
    ("iad_sim_register_value", _R, [_H, _I, _P(ctypes.c_uint16)]),
    ("iad_sim_handle_frame", _R, [_H, _P(ctypes.c_uint8), _Z, _P(ctypes.c_uint8), _Z, _P(_Z)]),
    ("iad_sim_set_request_hook", _R, [_H, SIM_REQUEST_HOOK, ctypes.c_void_p]),
    ("iad_sim_control_json", _R, [_H, _STR, *_BUF]),
    ("iad_sim_line_requests_json", _R, [_H, *_BUF]),
    ("iad_sim_line_control_json", _R, [_H, _STR, *_BUF]),
    ("iad_sim_set_clock", _R, [_H, CLOCK_NOW, ctypes.c_void_p]),
    # driver hooks
    ("iad_testing_set_timing", _R, [_STR, _D]),
    ("iad_testing_axis_nv_record", _R, [_H, _D, _STR, _P(_c_bool)]),
    ("iad_testing_axis_nv_last_hour", _R, [_H, _D, _P(ctypes.c_int32)]),
    ("iad_testing_set_port_list", _R, [_STR]),
)
"""(name, restype, argtypes) of every function of iai_actuator_driver_testing.h, in header order: the test hooks the
same library exports without a stability promise, bound on their first use after the exact-match check of
:data:`TESTING_ABI_VERSION`."""
_TESTING_NAMES = frozenset(name for name, _, _ in _TESTING_SIGNATURES)

# ====================================================================== finding and loading the library


class NativeLibraryError(EcDriverError, OSError):
    """The native library is not found, does not load, or has another ABI version than this binding."""


def library_name() -> str:
    """The library's file name on this OS."""
    if sys.platform == "win32":
        return "iai_actuator_driver.dll"
    if sys.platform == "darwin":
        return "libiai_actuator_driver.dylib"
    return "libiai_actuator_driver.so"


def _platform_tag() -> str:
    """``win-x64``, ``win-x86``, ``linux-x64``, ``linux-arm64``, ``osx-arm64``... (the folder of a packaged build)."""
    machine = platform.machine().lower()
    bits = struct.calcsize("P") * 8
    if machine in ("amd64", "x86_64", "x64"):
        arch = "x64" if bits == 64 else "x86"
    elif machine in ("arm64", "aarch64"):
        arch = "arm64"
    elif machine in ("x86", "i386", "i686"):
        arch = "x86"
    else:
        arch = machine or "unknown"
    os_name = {"win32": "win", "darwin": "osx"}.get(sys.platform, "linux")
    return f"{os_name}-{arch}"


def _candidates() -> list[Path]:
    """Where the library is looked for, in order (the environment variable alone when it is set)."""
    name = library_name()
    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        path = Path(env).expanduser()
        return [path / name if path.is_dir() else path]
    package = Path(__file__).resolve().parent
    return [
        package / "_native" / _platform_tag() / name,
        package / "_native" / name,
        package / name,
        package.parents[2] / "core" / "build" / name,
    ]


def find_library() -> Path | None:
    """The library file this module would load, or ``None`` when there is none (nothing is loaded)."""
    for path in _candidates():
        if path.is_file():
            return path
    return None


class _Library:
    """One loaded native library with argtypes / restype set on every function it exports."""

    def __init__(self, path: Path) -> None:
        self.path = path
        try:
            self.dll = (ctypes.WinDLL if _STDCALL else ctypes.CDLL)(str(path))  # type: ignore[attr-defined]
        except OSError as exc:
            bits = struct.calcsize("P") * 8
            raise NativeLibraryError(
                f"cannot load the native library {path}: {exc} (this Python is {bits}-bit; the library must be built "
                f"for the same CPU and bitness: {_platform_tag()})"
            ) from exc
        self.functions: dict[str, Any] = {}
        self.missing: list[str] = []
        self._bind(_SIGNATURES)
        # the testing header's functions: bound on their first use, after the exact-match check of their version
        self._testing_lock = threading.Lock()
        self._testing_bound = False
        self._testing_error: str | None = None

    def _bind(self, signatures: tuple[tuple[str, Any, list[Any]], ...]) -> None:
        for name, restype, argtypes in signatures:
            try:
                function = getattr(self.dll, name)
            except AttributeError:
                self.missing.append(name)
                continue
            function.restype = restype
            function.argtypes = argtypes
            self.functions[name] = function

    def _bind_testing(self, name: str) -> bool:
        """Bind the testing header's functions once, when one of them is first asked for (``name``), and only when
        the library's testing version is exactly :data:`TESTING_ABI_VERSION` (no compatibility between numbers;
        :class:`NativeLibraryError` otherwise, on every call). False when ``name`` is not a function of that header."""
        if name not in _TESTING_NAMES:
            return False
        with self._testing_lock:
            if self._testing_error is None and not self._testing_bound:
                version = getattr(self.dll, "iad_testing_abi_version", None)
                found = None
                if version is not None:
                    version.restype, version.argtypes = ctypes.c_uint32, []
                    found = int(version())
                if found != TESTING_ABI_VERSION:
                    self._testing_error = (
                        f"{self.path} has the testing ABI {found}; this binding needs exactly {TESTING_ABI_VERSION} "
                        "(iai_actuator_driver_testing.h). Use the library and the binding of the same release."
                    )
                else:
                    self._bind(_TESTING_SIGNATURES)
                    self._testing_bound = True
        if self._testing_error is not None:
            raise NativeLibraryError(self._testing_error)
        return True

    def has(self, name: str) -> bool:
        return name in self.functions or (self._bind_testing(name) and name in self.functions)

    def fn(self, name: str) -> Any:
        function = self.functions.get(name)
        if function is None and self._bind_testing(name):
            function = self.functions.get(name)
        if function is None:
            raise NotImplementedError(
                f"{name} is not exported by {self.path} (a partial or older build of the native library; rebuild it "
                "with 'python build.py shared' in core/)"
            )
        return function


_lock = threading.Lock()
_library: _Library | None = None


def load(path: str | os.PathLike[str] | None = None) -> _Library:
    """Load the native library (once; later calls return it) and check its ABI version.

    ``path`` loads that file instead of searching (only on the first load). Raises :class:`NativeLibraryError` when no
    library is found (the message lists where it looked), when it does not load, or when its ABI version does not
    serve this binding: the library must report ``ABI_VERSION_MAJOR`` and a minor at least ``ABI_VERSION_MINOR`` (a
    higher minor only adds).
    """
    global _library
    with _lock:
        if _library is not None:
            return _library
        if path is not None:
            found: Path | None = Path(path)
            if not found.is_file():
                raise NativeLibraryError(f"native library not found: {found}")
        else:
            found = find_library()
            if found is None:
                tried = "\n  ".join(str(p) for p in _candidates())
                raise NativeLibraryError(
                    f"the native library ({library_name()}) was not found. Looked in:\n  {tried}\nInstall a "
                    f"platform wheel (it carries the library), build it with 'python build.py shared' in core/, or set "
                    f"{ENV_VAR} to the library file. {PURE_PYTHON_REMOVED}"
                )
        lib = _Library(found.resolve())
        if not lib.has("iad_abi_version"):
            raise NativeLibraryError(f"{found} is not the IAI Actuator Driver native library (no iad_abi_version)")
        version = int(lib.fn("iad_abi_version")())
        major, minor = version >> 16, version & 0xFFFF
        wanted = f"{ABI_VERSION_MAJOR}.{ABI_VERSION_MINOR}"
        if major != ABI_VERSION_MAJOR or minor < ABI_VERSION_MINOR:
            rule = f"major {ABI_VERSION_MAJOR}, minor at least"
            raise NativeLibraryError(
                f"{found} has C ABI {major}.{minor}; this binding needs {rule} {wanted}. Use the library and the "
                "binding of the same release."
            )
        _library = lib
        _install_unraisablehook()  # before any callback can run: a Ctrl+C at a callback's entry is not lost
        return lib


def available() -> bool:
    """Whether the native library is found, loads and has this binding's ABI version (loads it when it does)."""
    try:
        load()
    except NativeLibraryError:
        return False
    return True


def library_path() -> Path:
    """The loaded library's file (loads it)."""
    return load().path


def missing_functions() -> list[str]:
    """Functions of the header the loaded library does not export (empty for a complete build; loads it)."""
    return list(load().missing)


def abi_version() -> tuple[int, int]:
    """(major, minor) of the loaded library's C ABI."""
    version = int(load().fn("iad_abi_version")())
    return version >> 16, version & 0xFFFF


def library_version() -> str:
    """Version of the native core, e.g. ``"1.0.0"`` (independent of the ABI version)."""
    return _text(load().fn("iad_library_version")())


def result_name(code: int) -> str:
    """Name of a result code (``"IAD_ERR_SOFT_LIMIT"``...), from the library when it is loaded."""
    try:
        lib = load()
        if lib.has("iad_result_name"):
            return _text(lib.fn("iad_result_name")(int(code)))
    except NativeLibraryError:
        pass
    return RESULT_NAMES.get(int(code), "IAD_ERR_UNKNOWN")


def list_ports() -> list[dict[str, Any]]:
    """The PC's serial ports, IAI USB adapters first (``port``, ``description``, ``manufacturer``, ``hardware_id``,
    ``vid``, ``pid``, ``is_iai``). Opens nothing."""
    lib = load()
    return _read_json(lib, "iad_list_ports_json")


def call_json(function: str, args: Mapping[str, Any] | Iterable[Mapping[str, Any]] | None = None) -> Any:
    """One of the core's stateless functions by name (iad_call_json, ABI 0.4): the alarm and CON parameter tables and
    rules, the decoders of the unit information. ``args``: a dict of named arguments, or a list of dicts (then a list
    of results, in order). ``call_json("functions")`` lists the names; a name the loaded library lacks raises
    ``NotImplementedError``. No I/O."""
    lib = load()
    text = None if args is None else json.dumps(args if isinstance(args, Mapping) else list(args))
    return _read_json(lib, "iad_call_json", _utf8(function), _utf8(text))


# ====================================================================== calls, errors, handles

_tls = threading.local()  # depth: > 0 between a call and the read of its last error on this thread
_pending: list[tuple[_Library, str, int]] = []  # handles a finalizer could not close at once
# The logging bridge's check (``_native.install_log_bridge``), run before each call: while the bridge's sink is the one
# installed and set_log_level was never called, it puts the core's log level in step with the :mod:`logging` levels
# when they changed since its last look (cheap otherwise). None: no bridge.
_log_level_check: Callable[[], None] | None = None


def _text(raw: bytes | None) -> str:
    return "" if not raw else raw.decode("utf-8", errors="replace")


def _invoke(lib: _Library, name: str, *args: Any) -> tuple[int, dict[str, Any] | None]:
    """Call ``name``; return its result and, when it failed, the thread's last error (read at once, same thread).

    A Ctrl+C that landed in a Python callback of the call (on the main thread) is raised here even when the call went
    on and succeeded. A call made from inside a callback (a transport object that calls the library itself) keeps the
    record of the call around it."""
    function = lib.fn(name)
    depth = getattr(_tls, "depth", 0)
    if depth == 0 and _pending:
        _drain_pending()
    level_check = _log_level_check
    if depth == 0 and level_check is not None:
        level_check()
    outer = (getattr(_tls, "callback_exc", None), getattr(_tls, "callback_propagate", False))
    _tls.depth = depth + 1
    _tls.callback_exc = None
    _tls.callback_propagate = False
    try:
        code = int(function(*args))
        if code >= 0:
            if isinstance(_tls.callback_exc, KeyboardInterrupt):
                raise _tls.callback_exc
            if _tls.callback_propagate and _tls.callback_exc is not None:
                # (0.7) a callback whose own exception is the error (a SimulatedEC.on_request that raised) failed, and
                # the core went on (it took the failure for a link fault): the caller gets it
                raise _tls.callback_exc
            return code, None
        details = _read_error(lib, code)
        if _tls.callback_exc is not None:  # 0.3: a Python callback raised during the call
            details["callback_exception"] = _tls.callback_exc
            if _tls.callback_propagate:  # 0.4: a preemptible read's gate: its own exception is raised
                details["callback_propagate"] = True
        return code, details
    finally:
        _tls.callback_exc, _tls.callback_propagate = outer
        _tls.depth = depth


def _read_error(lib: _Library, code: int) -> dict[str, Any]:
    """The last error of this thread: the iad_last_error_json record, else the message alone."""
    message = _text(lib.fn("iad_last_error_message")()) if lib.has("iad_last_error_message") else ""
    details: dict[str, Any] = {}
    if lib.has("iad_last_error_json"):
        function = lib.fn("iad_last_error_json")
        size = _JSON_START
        for _ in range(3):
            buffer = ctypes.create_string_buffer(size)
            required = ctypes.c_size_t(0)
            result = int(function(buffer, size, ctypes.byref(required)))
            if result == IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
                size = required.value
                continue
            if result == IAD_OK:
                try:
                    parsed = json.loads(buffer.value.decode("utf-8", errors="replace"))
                except ValueError:
                    parsed = None
                if isinstance(parsed, dict):
                    details = parsed
            break
    details.setdefault("code", code)
    if not details.get("message"):
        details["message"] = message or f"{result_name(code)} ({code})"
    return details


def _check(lib: _Library, name: str, *args: Any) -> None:
    """Call ``name`` and raise the matching exception when it fails."""
    code, details = _invoke(lib, name, *args)
    if code != IAD_OK:
        raise _make_error(code, details or {})


def _invoke_kept(
    lib: _Library, name: str, *args: Any, also: tuple[int, ...] = ()
) -> tuple[int, dict[str, Any] | None, str | None]:
    """:func:`_invoke` of a call that keeps its result on its handle (iad_axis_recover, iad_axis_self_test,
    iad_cell_recover_all ...), and that result as the call kept it: iad_last_result_json (0.7) read on this thread at
    once, so a call on another thread that replaced the handle's result meanwhile (a link monitor's check) does not
    change it. ``None`` when the call failed (IAD_ERR_CELL aside: the cell kept its result first; ``also``: other codes
    after which the call kept it, such as IAD_ERR_BUFFER_TOO_SMALL of a reader) or the library is older (the caller then
    reads the handle's)."""
    code, details = _invoke(lib, name, *args)
    if code not in (IAD_OK, IAD_ERR_CELL, *also) or not lib.has("iad_last_result_json"):
        return code, details, None
    function = lib.fn("iad_last_result_json")
    size = _JSON_START
    for _ in range(3):
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        result = int(function(buffer, size, ctypes.byref(required)))
        if result == IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
            size = required.value
            continue
        return code, details, buffer.value.decode("utf-8", errors="replace") if result == IAD_OK else None
    return code, details, None


def _invoke_kept_buffered(lib: _Library, name: str, *args: Any) -> tuple[int, dict[str, Any] | None, str | None]:
    """:func:`_invoke_kept` of a reader whose output goes to a caller's buffer (iad_axis_info_call_json): it keeps its
    result also when the buffer was too small, and that text is this thread's own."""
    return _invoke_kept(lib, name, *args, also=(IAD_ERR_BUFFER_TOO_SMALL,))


class _Helper:
    """One native call on its own thread, with an event that says when the call has ended.

    The thread waiting for it must use :meth:`wait` and :attr:`ended`, never ``Thread.join()`` or ``is_alive()``: when
    a Ctrl+C interrupts ``join()``, CPython 3.12 (and 3.11) takes the lock of the still running thread for one it had
    just acquired, releases it and marks the thread finished. The next ``join()`` then returns at once and
    ``is_alive()`` is False while the call still runs, so the driver would raise the ``KeyboardInterrupt`` before the
    core had sent its decel stop, and clear the cancel latch before the core had seen it (3.13 and later are not
    affected). An :class:`threading.Event` waits correctly however often it is interrupted."""

    def __init__(self, name: str, work: Callable[[], None]) -> None:
        self._done = threading.Event()

        def run() -> None:
            try:
                work()
            finally:
                self._done.set()  # after everything ``work`` stored: the waiting thread reads it once this is set

        threading.Thread(target=run, name=name, daemon=True).start()

    @property
    def ended(self) -> bool:
        return self._done.is_set()

    def wait(self, timeout: float) -> bool:
        """Wait up to ``timeout`` seconds; True once the call has ended."""
        return self._done.wait(timeout)


def _invoke_interruptible(
    lib: _Library, name: str, args: tuple[Any, ...], cancel: Callable[[], Any] | None,
    reset: Callable[[], Any] | None = None, *, call: Callable[..., Any] | None = None, cancelled: str | None = None,
) -> Any:
    """:func:`_invoke` of a call that may wait long (a move, home, push...), so that Ctrl+C reaches Python.

    Off the main thread (where no KeyboardInterrupt arrives) this is a plain call. On the main thread the call runs on a
    helper thread (which also reads the last error, so the error belongs to the call) while this thread waits in slices
    of 0.1 s; a KeyboardInterrupt then runs ``cancel`` (iad_axis_cancel / iad_cell_cancel: the core sends a decel stop
    and ends the wait), lets the call return and re-raises the KeyboardInterrupt with a note. ``reset`` (0.3: the
    cancel is the latch, iad_axis_cancel_latch, so a Ctrl+C just before the call reaches the core is not lost) runs
    after the call returned. ``cancel`` None (0.5: a bus call such as baud detection, which nothing can end early):
    the KeyboardInterrupt is raised once the call has ended on its own timeouts. The helper thread is named
    ``iad-call:<caller's name>``, and Python callbacks of the call (a transport object, a frame logger) run on it.
    ``call`` (default :func:`_invoke`) makes the call and returns what is returned here (0.7: :func:`_invoke_kept`).
    ``cancelled``: the note of a Ctrl+C that the core ended (default: a decel stop was sent; a read says its own)."""
    invoke = _invoke if call is None else call
    if threading.current_thread() is not threading.main_thread():
        return invoke(lib, name, *args)
    box: list[Any] = []

    def run() -> None:  # holds ``args`` (and so the structs behind byref) until the call returns
        try:
            box.append(invoke(lib, name, *args))
        except BaseException as exc:  # handed to the waiting thread
            box.append(exc)

    helper = _Helper(f"iad-call:{threading.current_thread().name}", run)
    try:
        while not helper.wait(_SLICE_S):
            pass
    except KeyboardInterrupt as interrupt:
        try:
            if cancel is None:
                interrupt.add_note(f"iai_actuator_driver: Ctrl+C during {name}, which ends on its own timeouts")
            else:
                cancel()
        except Exception as exc:  # never hide the interrupt
            interrupt.add_note(f"iai_actuator_driver: cancelling {name} failed: {exc}")
        else:
            if cancel is not None:
                interrupt.add_note(
                    cancelled or f"iai_actuator_driver: {name} cancelled on Ctrl+C (a decel stop was sent)"
                )
        helper.wait(10.0)  # the core ends the wait right after the stop; a second Ctrl+C leaves at once
        if reset is not None and helper.ended:
            try:
                reset()
            except Exception:
                pass
        raise
    outcome = box[0]
    if isinstance(outcome, BaseException):
        raise outcome
    return outcome


def _check_interruptible(
    lib: _Library, name: str, args: tuple[Any, ...], cancel: Callable[[], Any] | None,
    reset: Callable[[], Any] | None = None,
) -> None:
    code, details = _invoke_interruptible(lib, name, args, cancel, reset)
    if code != IAD_OK:
        raise _make_error(code, details or {})


class NativeOutOfRangeError(ValueError, IndexError):
    """IAD_ERR_INVALID_ARGUMENT with kind ``std::out_of_range`` (0.3): a value outside its range or a name / index that
    is not there. Both a ``ValueError`` (as every invalid argument) and an ``IndexError``."""


# ---- 0.5 (py-gateway)
NOT_READY_KINDS: dict[str, type[EcNotReadyError]] = {}
"""IAD_ERR_NOT_READY by the core's kind -> its class (``GatewayModeError``, ``PositionUnitError``,
``UnsupportedLineError``: the gateway module registers its own at import); any other kind is an ``EcNotReadyError``."""


def _make_error(code: int, details: Mapping[str, Any]) -> BaseException:
    """The package's exception for a result code (docs/NATIVE_ABI.md, the Python column of the error table)."""
    message = str(details.get("message") or result_name(code))
    kind = str(details.get("kind") or "")
    attempts = details.get("attempts")
    gave_up = f"gave up after {attempts} attempts" if isinstance(attempts, int) and attempts > 1 else None
    if gave_up is not None:  # the core appends it to the message; the package says it in a note
        message = message.replace(f" ({gave_up})", "", 1)
    exc: BaseException
    if code == IAD_ERR_SOFT_LIMIT:
        exc = SoftLimitError(message)
        rejection = details.get("cause")  # (0.6) a row the unit rejected: the IaiCommandRejected behind it
        if isinstance(rejection, Mapping):
            exc.__cause__ = _make_error(_int(rejection.get("code"), IAD_ERR_DRIVER), rejection)
    elif code == IAD_ERR_NOT_READY:  # (0.5 py-gateway) the line's kinds: the gateway module's classes
        exc = NOT_READY_KINDS.get(kind, EcNotReadyError)(message)
    elif code == IAD_ERR_ALARM:
        exc = EcAlarmError(_int(details.get("alarm_code")), message)
    elif code == IAD_ERR_MOTION_TIMEOUT:
        exc = MotionTimeoutError(message)
    elif code == IAD_ERR_MODBUS_TIMEOUT:
        exc = ModbusTimeout(message)
    elif code == IAD_ERR_FRAME:
        exc = ModbusCRCError(message) if "crc" in kind.lower() else ModbusFrameError(message)
    elif code == IAD_ERR_TRANSPORT:
        exc = TransportError(message)
    elif code == IAD_ERR_VERIFICATION:
        exc = VerificationError(message)
    elif code == IAD_ERR_NONVOLATILE_REFUSED:
        exc = NonVolatileWriteError(message)
    elif code == IAD_ERR_INVALID_ARGUMENT:
        exc = NativeOutOfRangeError(message) if kind == "std::out_of_range" else ValueError(message)
    elif code == IAD_ERR_UNSUPPORTED:
        exc = NotImplementedError(message)
    elif code in (IAD_ERR_INTERNAL, IAD_ERR_BUFFER_TOO_SMALL):
        exc = RuntimeError(message)
    elif code == IAD_ERR_SERVO_BLOCKED:
        exc = ServoBlockedError(message, tuple(str(r) for r in details.get("reasons") or ()))
    elif code == IAD_ERR_MOTION_INCOMPLETE:
        exc = MotionIncompleteError(message)
    elif code == IAD_ERR_SLAVE_EXCEPTION:
        exc = ModbusException(_int(details.get("exception_code")), _int(details.get("function")), message)
    elif code == IAD_ERR_COMMAND_REJECTED:
        rejected = IaiCommandRejected.__new__(IaiCommandRejected)  # keep the core's message (it has the hint)
        Exception.__init__(rejected, message)
        rejected.function = _int(details.get("function"), 0x45)
        rejected.command = _int(details.get("command"))
        rejected.rejection = _int(details.get("rejection"))
        status = details.get("status")  # the reply's status byte (0x45) or its four status words (0x48)
        rejected.status = tuple(status) if isinstance(status, list) else status
        rejected.reason = str(details.get("reason") or "")
        rejected.hint = str(details.get("hint") or "")
        exc = rejected
    elif code == IAD_ERR_UNSAFE_REFUSED:  # the line's refusal is its own class (the same code)
        exc = LineRuleError(message) if kind == "LineRuleError" else UnsafeCommandError(message)
    elif code == IAD_ERR_CELL:
        axes = details.get("axes") if isinstance(details.get("axes"), Mapping) else {}
        errors = {
            str(name): _make_error(_int(entry.get("code"), IAD_ERR_DRIVER), entry)
            for name, entry in axes.items()
            if isinstance(entry, Mapping)
        }
        exc = CellError(str(details.get("operation") or "cell operation"), errors, {})
    elif code == IAD_ERR_MODBUS:
        exc = ModbusError(message)
    elif code == IAD_ERR_DRIVER:
        exc = EcError(message) if kind == "EcError" else EcDriverError(message)
    elif code == IAD_ERR_UNIT_REPLACED:
        expected = details.get("expected") if isinstance(details.get("expected"), Mapping) else {}
        found = details.get("found") if isinstance(details.get("found"), Mapping) else {}
        exc = UnitReplacedError(message, expected=expected, found=found)
        exc.report = details.get("report")  # the core's report as a dict (RecoveryReport.to_dict layout)
    elif code == IAD_ERR_RECOVERY:
        exc = RecoveryError(message)
        exc.report = details.get("report")
    elif code == IAD_ERR_FILE:
        exc = OSError(message)
    elif code == IAD_ERR_CANCELLED:  # iad_axis_cancel / iad_cell_cancel ended the wait (a decel stop was sent first)
        exc = KeyboardInterrupt(message)
    else:  # a code newer than this binding: the base driver error
        exc = EcDriverError(message)
    cause = details.get("callback_exception")  # 0.3: what a Python callback (a transport) raised
    if isinstance(cause, KeyboardInterrupt) or (details.get("callback_propagate") and isinstance(cause, BaseException)):
        exc = cause
    elif isinstance(cause, BaseException):
        exc.__cause__ = cause
    elif code == IAD_ERR_TRANSPORT:  # the core's own port failed: an OSError cause
        exc.__cause__ = OSError(message.removeprefix("transport error: "))
    if gave_up is not None and exc is not cause:
        exc.add_note(gave_up)
    exc.iad_code = code  # type: ignore[attr-defined]
    exc.iad_details = dict(details)  # type: ignore[attr-defined]
    exc.stop_attempted = bool(details.get("stop_attempted", False))  # type: ignore[attr-defined]
    return exc


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _read_json(lib: _Library, name: str, *args: Any) -> Any:
    """A JSON output: 64 KiB first, then the size the library asked for (a read-only call, so repeating is harmless)."""
    return json.loads(_read_text(lib, name, *args))


def _read_text(lib: _Library, name: str, *args: Any) -> str:
    size = _JSON_START
    for _ in range(3):
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        code, details = _invoke(lib, name, *args, buffer, size, ctypes.byref(required))
        if code == IAD_ERR_BUFFER_TOO_SMALL and required.value > size:
            size = required.value
            continue
        if code != IAD_OK:
            raise _make_error(code, details or {})
        return buffer.value.decode("utf-8", errors="replace")
    raise RuntimeError(f"{name}: the output kept growing; try again")


def _release(lib: _Library, close_name: str, handle: int) -> None:
    """Finalizer of a forgotten handle: close it now, or after the call running on this thread has read its error."""
    try:
        if getattr(_tls, "depth", 0):
            _pending.append((lib, close_name, handle))
        else:
            _close_quietly(lib, close_name, handle)
    except Exception:  # a finalizer never raises
        pass


def _close_quietly(lib: _Library, close_name: str, handle: int) -> None:
    try:
        lib.fn(close_name)(handle)
    except Exception:
        pass


def _drain_pending() -> None:
    while _pending:
        try:
            lib, close_name, handle = _pending.pop()
        except IndexError:
            break
        _close_quietly(lib, close_name, handle)


atexit.register(_drain_pending)


# ====================================================================== results as Python objects


def _opt(value: float) -> float | None:
    return None if math.isnan(value) else float(value)


def _opt_int(value: int) -> int | None:
    return None if value == IAD_INT_UNKNOWN else int(value)


def _tri(value: int) -> bool | None:
    return None if value == IAD_UNKNOWN else bool(value)


def _num(value: float | None) -> float:
    return math.nan if value is None else float(value)


def _slot(name: str, value: float | None) -> float:
    """A speed, acceleration or band for a struct without an ``_init`` (``iad_axis_move``, ``iad_sequence_point``):
    ``None`` is NaN (the default). The core reads a 0 in those structs as "the default" (0.8), so an explicit 0 is
    refused here as the managed API always did."""
    if value is None:
        return math.nan
    number = float(value)
    if number == 0.0:
        raise SoftLimitError(f"{name} must be > 0, got 0")
    return number


def _alarm_level(level: int) -> str:
    """alarms.LEVEL_* of an IAD_ALARM_LEVEL_* value (LEVEL_UNKNOWN for one this binding does not know)."""
    from . import alarms  # at the first use: see the imports

    return str(getattr(alarms, _ALARM_LEVELS[level] if 0 <= level < len(_ALARM_LEVELS) else "LEVEL_UNKNOWN"))


@dataclass(frozen=True)
class NativeStatus:
    """One decoded status sample (iad_status). Field names follow :class:`~iai_actuator_driver.ec.Status`; ``None``
    where the core does not know the value (NaN / IAD_UNKNOWN). ``raw`` holds DSS1, DSS2, DSSE, DIPM and DOPM."""

    position_mm: float
    velocity_mm_s: float | None
    current_ma: float | None
    uptime_s: float | None
    alarm_code: int
    alarm_level: str | None  # alarms.LEVEL_* of alarm_code; None without an alarm code (as Status.alarm_level)
    servo_on: bool
    homed: bool
    in_position: bool
    moving: bool
    alarm: bool
    warning: bool
    modbus_enabled: bool
    emergency_stop: bool
    paused: bool
    homing: bool
    safety_speed: bool
    ready: bool
    idle: bool
    target_reached: bool
    push_missed: bool
    pushing: bool
    emg_input: bool | None
    drive_power_cut: bool | None
    enable_off: bool | None
    raw_words: int
    raw: dict[str, int] = field(default_factory=dict)
    line: str | None = None  # "rcon" when read behind an RCON gateway (the bits RCON leaves undefined are not decoded)

    @property
    def fault(self) -> bool:
        """Same as :attr:`alarm`: a fault that blocks motion."""
        return self.alarm

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)

    @classmethod
    def from_c(cls, s: Status) -> NativeStatus:
        return cls(
            position_mm=float(s.position_mm),
            velocity_mm_s=_opt(s.velocity_mm_s),
            current_ma=_opt(s.current_ma),
            uptime_s=_opt(s.uptime_s),
            alarm_code=int(s.alarm_code),
            alarm_level=_alarm_level(int(s.alarm_level)) if s.alarm_code else None,
            servo_on=bool(s.servo_on),
            homed=bool(s.homed),
            in_position=bool(s.in_position),
            moving=bool(s.moving),
            alarm=bool(s.alarm),
            warning=bool(s.warning),
            modbus_enabled=bool(s.modbus_enabled),
            emergency_stop=bool(s.emergency_stop),
            paused=bool(s.paused),
            homing=bool(s.homing),
            safety_speed=bool(s.safety_speed),
            ready=bool(s.ready),
            idle=bool(s.idle),
            target_reached=bool(s.target_reached),
            push_missed=bool(s.push_missed),
            pushing=bool(s.pushing),
            emg_input=_tri(s.emg_input),
            drive_power_cut=_tri(s.drive_power_cut),
            enable_off=_tri(s.enable_off),
            raw_words=int(s.raw_words),
            raw={"DSS1": s.raw_dss1, "DSS2": s.raw_dss2, "DSSE": s.raw_dsse, "DIPM": s.raw_dipm, "DOPM": s.raw_dopm},
            line="rcon" if s.rcon else None,
        )


def _status_out() -> Status:
    """A status output that tells "not written" apart: raw_words starts at IAD_INT_UNKNOWN (a written one is 8..22)."""
    out = Status()
    out.raw_words = IAD_INT_UNKNOWN
    return out


def _status_or_none(s: Status) -> NativeStatus | None:
    return None if s.raw_words == IAD_INT_UNKNOWN else NativeStatus.from_c(s)


@dataclass(frozen=True)
class NativeRange:
    """One range the driver enforces (iad_range); ``None`` where not known. The source of each range, in words, is
    in :meth:`NativeAxis.limits`."""

    min: float | None
    max: float | None
    unit_max: float | None
    configured: float | None
    recommended_min: float | None
    available: bool | None
    count: int | None
    unit: str

    @classmethod
    def from_c(cls, r: Range) -> NativeRange:
        return cls(
            min=_opt(r.min), max=_opt(r.max), unit_max=_opt(r.unit_max), configured=_opt(r.configured),
            recommended_min=_opt(r.recommended_min), available=_tri(r.available), count=_opt_int(r.count),
            unit=_text(r.unit),
        )


@dataclass(frozen=True)
class NativeLimits:
    """Every range the driver enforces on one axis (iad_limits), from the fixed struct."""

    position: NativeRange
    speed: NativeRange
    accel: NativeRange
    decel: NativeRange
    accel_decel: NativeRange
    push: NativeRange
    band: NativeRange
    push_band: NativeRange
    stored_rows: NativeRange
    jog_step: NativeRange
    speed_override: NativeRange
    units: str
    kind: str
    family: str
    push_speed: float | None = None  # min(PSHV / CON No.34, MXPH) in units/s; None when not readable

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)

    @classmethod
    def from_c(cls, lim: Limits) -> NativeLimits:
        ranges = {name: NativeRange.from_c(getattr(lim, name)) for name in _RANGE_NAMES}
        return cls(
            **ranges, units=_text(lim.units), kind=_text(lim.kind), family=_text(lim.family),
            push_speed=_opt(lim.push_speed),
        )


@dataclass(frozen=True)
class NativeIdentity:
    """Identity of one unit (iad_identity); ``None`` where not known. The full record is
    :attr:`NativeAxis.device_info`."""

    slave: int
    system_code: int | None
    unit_code: int | None
    run_status: int | None
    axis_number: int | None
    max_positions: int | None
    actuator_kind: int | None
    unit_known: bool | None
    rotary: bool | None
    soft_min_mm: float | None
    soft_max_mm: float | None
    max_speed_mm_s: float | None
    max_accel_g: float | None
    max_decel_g: float | None
    product_family: str
    model: str
    model_name: str
    controller_name: str
    controller: str
    unit_name: str
    system_name: str
    firmware: str
    serial_number: str
    actuator_serial: str
    user_name: str
    units: str
    kind: str
    identity_source: str

    @property
    def family(self) -> str:
        """Same as :attr:`product_family` ("EC", "PCON"...)."""
        return self.product_family

    @property
    def soft_limits_mm(self) -> tuple[float, float] | None:
        """The unit's soft limits as stored (with the 0.30 margin), or ``None`` when not read."""
        if self.soft_min_mm is None or self.soft_max_mm is None:
            return None
        return (self.soft_min_mm, self.soft_max_mm)

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)

    @classmethod
    def from_c(cls, i: Identity) -> NativeIdentity:
        return cls(
            slave=int(i.slave), system_code=_opt_int(i.system_code), unit_code=_opt_int(i.unit_code),
            run_status=_opt_int(i.run_status), axis_number=_opt_int(i.axis_number),
            max_positions=_opt_int(i.max_positions), actuator_kind=_opt_int(i.actuator_kind),
            unit_known=_tri(i.unit_known), rotary=_tri(i.rotary), soft_min_mm=_opt(i.soft_min_mm),
            soft_max_mm=_opt(i.soft_max_mm), max_speed_mm_s=_opt(i.max_speed_mm_s), max_accel_g=_opt(i.max_accel_g),
            max_decel_g=_opt(i.max_decel_g), product_family=_text(i.product_family), model=_text(i.model),
            model_name=_text(i.model_name), controller_name=_text(i.controller_name), controller=_text(i.controller),
            unit_name=_text(i.unit_name), system_name=_text(i.system_name), firmware=_text(i.firmware),
            serial_number=_text(i.serial_number), actuator_serial=_text(i.actuator_serial),
            user_name=_text(i.user_name), units=_text(i.units), kind=_text(i.kind),
            identity_source=_text(i.identity_source),
        )


@dataclass(frozen=True)
class NativeParameterWriteResult:
    """Outcome of :meth:`NativeAxis.write_parameter` (iad_param_write_result; values as the PC tool shows them), with
    the warnings and the note of the full record (``record``: the C++ result as a dict)."""

    number: int
    key: str
    address: str
    name: str
    before: str
    after: str
    unit: str
    changed: bool
    verified: bool
    committed: bool
    commit_confirmed: bool
    saved: bool
    skipped: bool
    forced: bool
    listed_for_model: bool
    verified_for_model: bool
    home_parameter: bool
    warnings: tuple[str, ...] = ()
    note: str = ""
    record: dict[str, Any] = field(default_factory=dict, compare=False)

    @property
    def no(self) -> int:
        """Same as :attr:`number` (the package API's name)."""
        return self.number

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)


@dataclass(frozen=True)
class NativeResetResult:
    """Outcome of :meth:`NativeAxis.software_reset` (iad_reset_result) with the notes of the full record."""

    status: NativeStatus
    accepted: bool
    reply_lost: bool
    confirmed: bool
    confirmed_by: str
    baudrate: int | None  # None on a transport without a baud rate
    baud_changed: bool
    needs_home: bool
    uptime_before_s: float | None
    uptime_after_s: float | None
    elapsed_s: float | None
    notes: tuple[str, ...] = ()

    @property
    def homed(self) -> bool:
        return self.status.homed

    @property
    def servo_on(self) -> bool:
        return self.status.servo_on

    @property
    def alarm_code(self) -> int:
        return self.status.alarm_code

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return asdict(self)


@dataclass(frozen=True)
class NativePushResult:
    """Outcome of :meth:`NativeAxis.push` (iad_push_result), as :class:`~iai_actuator_driver.ec.PushResult`:
    ``"contact"`` (holding against the work) or ``"missed"``; ``warnings`` from the full record."""

    outcome: str
    position_mm: float
    status: NativeStatus
    warnings: tuple[str, ...] = ()

    @property
    def contact(self) -> bool:
        """The push met the work (PEND and DSSE bit 0); the motor keeps pushing until the next move."""
        return self.outcome == "contact"

    @property
    def missed(self) -> bool:
        """The push went over the whole band without contact (DSS1 PSFL)."""
        return self.outcome == "missed"

    def to_dict(self) -> dict[str, Any]:
        """JSON-friendly dict."""
        return {"outcome": self.outcome, "position_mm": self.position_mm, "status": self.status.to_dict(),
                "warnings": list(self.warnings)}  # fmt: skip


@dataclass(frozen=True)
class NativeMonitorSample:
    """One published poll of the status monitor (iad_monitor_sample): the lean ``status``, its sequence number, the
    monotonic times just before the request went out (``t_start``) and after the answer was decoded (``t``), and the
    monitor's counters at that moment."""

    status: NativeStatus
    seq: int
    t_start: float
    t: float
    age_s: float | None
    poll_rate_hz: float | None
    polls: int
    error_count: int
    running: bool

    @classmethod
    def from_c(cls, m: MonitorSample) -> NativeMonitorSample:
        return cls(
            status=NativeStatus.from_c(m.status), seq=int(m.seq), t_start=float(m.t_start_s), t=float(m.t_s),
            age_s=_opt(m.age_s), poll_rate_hz=_opt(m.poll_rate_hz), polls=int(m.polls),
            error_count=int(m.error_count), running=bool(m.running),
        )


def _joined(text: bytes) -> tuple[str, ...]:
    """A "; "-joined list of a fixed text field."""
    return tuple(part for part in _text(text).split("; ") if part)


@dataclass(frozen=True)
class NativeLinkCheck:
    """Result of :meth:`NativeAxis.check_link` (iad_link_check), as :class:`~iai_actuator_driver.recovery.LinkCheck`:
    ``state`` (``"ok"``, ``"port_vanished"``, ``"unit_silent"``, ``"unit_restarted"``, ``"recovering"``; ``"error"``
    when a cell's check itself failed), ``reasons`` in words, the ``status`` read (``None`` when the unit did not
    answer) and the ``error`` (``"<Type>: <message>"``) when it did not."""

    state: str
    reasons: tuple[str, ...] = ()
    status: NativeStatus | None = None
    error: str | None = None
    elapsed_s: float = 0.0
    axis: str | None = None

    @property
    def ok(self) -> bool:
        return self.state == "ok"

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": self.axis, "state": self.state, "ok": self.ok, "reasons": list(self.reasons), "error": self.error,
            "elapsed_s": self.elapsed_s, "position_mm": None if self.status is None else self.status.position_mm,
        }  # fmt: skip

    @classmethod
    def from_c(cls, c: LinkCheck, axis: str | None = None) -> NativeLinkCheck:
        return cls(
            state=_text(c.state), reasons=_joined(c.reasons),
            status=NativeStatus.from_c(c.status) if c.status.raw_words > 0 else None,
            error=_text(c.error) or None, elapsed_s=float(c.elapsed_s), axis=axis,
        )


@dataclass(frozen=True)
class NativeRecoveryReport:
    """What :meth:`NativeAxis.recover` found and did (iad_recovery_summary), with the attribute names of
    :class:`~iai_actuator_driver.recovery.RecoveryReport`. ``record`` is the full report (restart signs, actions,
    notes, row and parameter differences, new alarm records) as a dict; ``to_dict()`` returns it."""

    axis: str | None
    cause: str
    restarted: bool
    same_unit: bool
    port_reopened: bool
    baudrate: int | None
    baudrate_found: int | None
    baud_restored: bool
    control_restored: bool
    safety_speed_restored: bool
    alarm_code: int
    needs_home: bool
    needs_attention: bool
    ready_to_move: bool
    fault_alarm: bool
    at_target: bool | None
    servo_on: bool | None
    homed: bool | None
    modbus_control: bool | None
    safety_speed: bool | None
    position_mm: float | None
    last_target_mm: float | None
    position_error_mm: float | None
    elapsed_s: float
    summary: str
    status: NativeStatus | None = None
    notes: tuple[str, ...] = ()
    record: dict[str, Any] = field(default_factory=dict, compare=False, repr=False)

    def to_dict(self) -> dict[str, Any]:
        """The full report as the core wrote it (the layout of RecoveryReport.to_dict)."""
        return dict(self.record) if self.record else asdict(self)

    @classmethod
    def from_c(cls, s: RecoverySummary, record: Mapping[str, Any] | None) -> NativeRecoveryReport:
        record = dict(record or {})
        return cls(
            axis=_text(s.axis) or None, cause=_text(s.cause), restarted=bool(s.restarted), same_unit=bool(s.same_unit),
            port_reopened=bool(s.port_reopened), baudrate=int(s.baud_rate) or None,
            baudrate_found=int(s.baud_rate_found) or None, baud_restored=bool(s.baud_restored),
            control_restored=bool(s.control_restored), safety_speed_restored=bool(s.safety_speed_restored),
            alarm_code=int(s.alarm_code), needs_home=bool(s.needs_home), needs_attention=bool(s.needs_attention),
            ready_to_move=bool(s.ready_to_move), fault_alarm=bool(s.fault_alarm), at_target=_tri(s.at_target),
            servo_on=_tri(s.servo_on), homed=_tri(s.homed), modbus_control=_tri(s.modbus_control),
            safety_speed=_tri(s.safety_speed), position_mm=_opt(s.position_mm),
            last_target_mm=_opt(s.last_target_mm), position_error_mm=_opt(s.position_error_mm),
            elapsed_s=float(s.elapsed_s), summary=_text(s.summary),
            status=NativeStatus.from_c(s.status) if s.status.raw_words > 0 else None,
            notes=tuple(str(n) for n in record.get("notes") or ()), record=record,
        )


@dataclass(frozen=True)
class NativeCheckReport:
    """Summary of :meth:`NativeAxis.run_check` (iad_check_summary); ``record`` is the JSON report in the package's
    layout (``items``: one dict per line with ``name``, ``result``, ``detail``); ``to_dict()`` returns it."""

    passed: bool
    items: int
    failed: int
    warned: int
    skipped: int
    record: dict[str, Any] = field(default_factory=dict, compare=False, repr=False)

    @property
    def lines(self) -> list[CheckItem]:
        """The report's lines as :class:`~iai_actuator_driver.check.CheckItem`."""
        from .check import CheckItem  # at the first use: see the imports

        out: list[CheckItem] = []
        for item in self.record.get("items") or ():
            if isinstance(item, Mapping):
                data = item.get("data")
                out.append(CheckItem(str(item.get("name", "")), str(item.get("result", "")),
                                     str(item.get("detail", "")), dict(data) if isinstance(data, Mapping) else {}))
        return out

    def to_dict(self) -> dict[str, Any]:
        return dict(self.record)


def _entry_from_c(e: CPositionEntry | PositionWriteResult, *, skipped: bool = False) -> PositionEntry:
    """The package's :class:`~iai_actuator_driver.ec.PositionEntry` (values at 0.01 resolution)."""
    from .values import PositionEntry  # at the first use: see the imports

    return PositionEntry(
        position_mm=round(e.position_mm, 2),
        band_mm=round(e.band_mm, 2),
        speed_mm_s=round(e.speed_mm_s, 2),
        zone_plus_mm=round(e.zone_plus_mm, 2),
        zone_minus_mm=round(e.zone_minus_mm, 2),
        accel_g=round(e.accel_g, 2),
        decel_g=round(e.decel_g, 2),
        push_power=int(e.push_power),
        load_threshold=int(e.load_threshold),
        control_flags=int(e.control_flags),
        checksum=_opt_int(e.checksum),
        skipped=skipped,
    )


# ====================================================================== options


_AXIS_OPTION_KEYS = (
    "stroke", "max_speed_mm_s", "max_accel_g", "max_push_pct", "poll_interval_s", "settle_s", "strategy",
    "auto_modbus_control", "push_settle_s", "completion",
)


def _code(value: str | None, choices: Mapping[str, int], what: str, default: int = 0) -> int:
    """The C code of an optional choice (``None``: ``default``, the header's "not set")."""
    return default if value is None else _choice(value, choices, what)


def _name_of(code: int, choices: Mapping[str, int], default: str | None = None) -> str | None:
    return next((name for name, value in choices.items() if value == code), default)


def _choice(value: str, choices: Mapping[str, int], what: str) -> int:
    try:
        return choices[str(value).lower()]
    except KeyError:
        raise ValueError(
            f"{what} {value!r} is not in the native core; choose from {sorted(choices)}"
        ) from None


def _apply_axis_options(o: AxisOptions, options: Mapping[str, Any]) -> None:
    unknown = sorted(set(options) - set(_AXIS_OPTION_KEYS))
    if unknown:
        raise TypeError(f"unknown axis option(s) {unknown}; the native core takes {', '.join(_AXIS_OPTION_KEYS)}")
    if "stroke" in options:
        stroke = options["stroke"]
        if stroke is None:
            o.stroke_min = o.stroke_max = math.nan
        else:
            low, high = stroke
            o.stroke_min, o.stroke_max = float(low), float(high)
    for name in ("max_speed_mm_s", "max_accel_g", "max_push_pct"):
        if name in options:
            setattr(o, name, _num(options[name]))
    for name in ("poll_interval_s", "settle_s"):
        if options.get(name) is not None:
            setattr(o, name, float(options[name]))
    if options.get("strategy") is not None:
        o.method = _choice(options["strategy"], _METHODS, "strategy")
    if options.get("auto_modbus_control") is not None:
        o.auto_modbus_control = int(bool(options["auto_modbus_control"]))
    if options.get("push_settle_s") is not None:
        o.push_settle_s = float(options["push_settle_s"])
    if options.get("completion") is not None:
        o.completion = _choice(options["completion"], _COMPLETIONS, "completion")


def _axis_options(lib: _Library, options: Mapping[str, Any]) -> AxisOptions | None:
    """Axis options from keywords (``None``: the defaults, a NULL pointer)."""
    if not options:
        return None
    o = AxisOptions()
    _check(lib, "iad_axis_options_init", ctypes.byref(o))
    _apply_axis_options(o, options)
    return o


def _open_options(
    lib: _Library, *, slave: int | None, baudrate: int | None, timeout: float | None, retries: int | None,
    min_gap: float | None, line: str | None = None, position_unit_mm: float | None = None, fast: bool = False,
) -> OpenOptions | None:
    values = (slave, baudrate, timeout, retries, min_gap, line, position_unit_mm)
    if all(v is None for v in values) and not fast:
        return None
    o = OpenOptions()
    _check(lib, "iad_open_options_init", ctypes.byref(o))
    if line is not None:
        o.line = _choice(line, _LINES, "line")
    if position_unit_mm is not None:
        o.position_unit_mm = float(position_unit_mm)
    o.fast = int(bool(fast))
    if slave is not None:
        o.slave = int(slave)
    if baudrate is not None:
        o.baud_rate = int(baudrate)
    if timeout is not None:
        o.read_timeout_s = float(timeout)
    if retries is not None:
        o.retries = int(retries)
    if min_gap is not None:
        o.min_gap_s = float(min_gap)
    return o


def _sim_options(
    lib: _Library, *, time_scale: float | None, position_mm: float | None, stroke_mm: float | None,
    max_speed_mm_s: float | None, baudrate: int | None, pmsl: bool | None,
) -> SimOptions | None:
    values = (time_scale, position_mm, stroke_mm, max_speed_mm_s, baudrate, pmsl)
    if all(v is None for v in values):
        return None
    o = SimOptions()
    _check(lib, "iad_sim_options_init", ctypes.byref(o))
    if time_scale is not None:
        o.time_scale = float(time_scale)
    if position_mm is not None:
        o.position_mm = float(position_mm)
    if stroke_mm is not None:
        o.stroke_mm = float(stroke_mm)
    if max_speed_mm_s is not None:
        o.max_speed_mm_s = float(max_speed_mm_s)
    if baudrate is not None:
        o.baud_rate = int(baudrate)
    if pmsl is not None:
        o.pmsl = int(bool(pmsl))
    return o


def _cell_options(lib: _Library, **values: Any) -> CellOptions | None:
    if all(v is None for v in values.values()):
        return None
    o = CellOptions()
    _check(lib, "iad_cell_options_init", ctypes.byref(o))
    fields = {
        "timeout": ("read_timeout_s", float), "min_gap": ("min_gap_s", float),
        "scan_timeout": ("scan_timeout_s", float), "max_axes": ("max_axes", int), "baudrate": ("baud_rate", int),
        "retries": ("retries", int), "scan_first": ("scan_first", int), "scan_last": ("scan_last", int),
        "identify": ("identify", lambda v: int(bool(v))), "stop_on_fault": ("stop_on_fault", lambda v: int(bool(v))),
        "line": ("line", lambda v: _choice(v, _LINES, "line")), "position_unit_mm": ("position_unit_mm", float),
    }
    for key, value in values.items():
        if value is not None:
            name, convert = fields[key]
            setattr(o, name, convert(value))
    return o


def _move_options(
    lib: _Library, init: str, speed: float | None, accel: float | None, band: float | None, wait: bool,
    timeout: float | None, *, decel_g: float | None = None, accel_pattern: str | None = None,
    completion: str | None = None,
) -> MoveOptions:
    o = MoveOptions()
    _check(lib, init, ctypes.byref(o))
    if speed is not None:
        o.speed = float(speed)
    if accel is not None:
        o.accel_g = float(accel)
    if band is not None:
        o.band = float(band)
    o.timeout_s = _num(timeout)
    o.wait = int(bool(wait))
    o.decel_g = _num(decel_g)  # NaN: = accel_g (the 0x9900 block); else the command-profile path
    o.accel_pattern = _code(accel_pattern, _ACCEL_PATTERNS, "accel_pattern")
    o.completion = _code(completion, _COMPLETIONS, "completion")
    return o


def _rows(rows: str | Iterable[int] | None, none: str | None = None) -> str | None:
    """A row list of the recovery / backup functions: ``None`` -> ``none``, "used" -> NULL (the core's default),
    "all", or row numbers -> "0,1,5"."""
    if rows is None:
        return none
    if isinstance(rows, str):
        return None if rows == "used" else rows
    if isinstance(rows, bool):
        return "all" if rows else None
    return ",".join(str(int(n)) for n in rows)


def _backup_text(backup: Mapping[str, Any] | str | os.PathLike[str] | None) -> str | None:
    """The JSON text of a backup: a dict (an info.backup result) or a backup file."""
    if backup is None:
        return None
    if isinstance(backup, Mapping):
        return json.dumps(backup)
    return Path(backup).read_text(encoding="utf-8")


def _recovery_policy(lib: _Library, policy: RecoveryPolicy | Mapping[str, Any] | None) -> CRecoveryPolicy | None:
    """iad_recovery_policy from a :class:`~iai_actuator_driver.recovery.RecoveryPolicy` or a mapping of its fields
    (``check_rows`` and ``backup`` are arguments of the call, ``list_ports`` is not in the core)."""
    if policy is None:
        return None
    values = asdict(policy) if _is_recovery_policy(policy) else dict(policy)
    o = CRecoveryPolicy()
    _check(lib, "iad_recovery_policy_init", ctypes.byref(o))
    for key in ("restore_baudrate", "restore_control", "restore_safety_speed", "clear_message_alarms",
                "check_parameters"):
        if key in values:
            setattr(o, key.replace("baudrate", "baud_rate"), int(bool(values[key])))
    for key in ("position_tolerance_mm", "poll_s"):
        if values.get(key) is not None:
            setattr(o, key, float(values[key]))
    if values.get("boot_baudrate") is not None:
        o.boot_baud_rate = int(values["boot_baudrate"])
    return o


def _monitor_options(lib: _Library, values: Mapping[str, Any]) -> MonitorOptions:
    o = MonitorOptions()
    _check(lib, "iad_monitor_options_init", ctypes.byref(o))
    for key in ("gap_s", "link_check_s"):
        if values.get(key) is not None:
            setattr(o, key, float(values[key]))
    for key in ("retries", "events", "queue_capacity"):
        if values.get(key) is not None:
            setattr(o, key, int(values[key]))
    return o


def _nv_options(lib: _Library, allow_nonvolatile: bool, force: bool, allow_unsafe: bool) -> NvWriteOptions:
    o = NvWriteOptions()
    _check(lib, "iad_nv_write_options_init", ctypes.byref(o))
    o.allow_nonvolatile = int(bool(allow_nonvolatile))
    o.force = int(bool(force))
    o.allow_unsafe = int(bool(allow_unsafe))
    return o


def _utf8(value: str | None) -> bytes | None:
    return None if value is None else str(value).encode("utf-8")


# ====================================================================== events, move handles, the status monitor


def _call_back(callback: Callable[..., Any], *args: Any) -> None:
    """A user callback on the event thread: a broken callback never stops the events."""
    try:
        callback(*args)
    except Exception:
        log.exception("callback %r raised", callback)


def _is_recovery_policy(value: Any) -> bool:
    from .recovery import RecoveryPolicy  # at the first use: see the imports

    return isinstance(value, RecoveryPolicy)


def _recovery_event(e: Event) -> RecoveryEvent:
    from .recovery import RecoveryEvent  # at the first use: see the imports

    return RecoveryEvent(
        _text(e.name), _text(e.axis) or None, _text(e.message), _text(e.cause) or None,
        float(e.wall_time_s) if math.isfinite(e.wall_time_s) else time.time(),
    )


class _EventPump:
    """Drains one handle's event queue (iad_axis_next_event / iad_cell_next_event) on a thread of this module and hands
    each event to ``dispatch`` there, so the core never calls into Python for its events. The backlog queued before
    the pump started (nobody listened) is dropped. The pump ends with :meth:`stop` or when the handle is closed (the
    close wakes the waiting call)."""

    def __init__(
        self, lib: _Library, function: str, handle: int, dispatch: Callable[[Event], None], name: str
    ) -> None:
        self._lib = lib
        self._function = function
        self._handle = handle
        self._dispatch = dispatch
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._cond = threading.Condition()
        self._waits = 0  # waits on the queue started so far (each numbered)
        self._seen_empty = 0  # the number of the last wait that found the queue empty
        self._flushers = 0  # flush() calls waiting: the next wait does not block
        self.dropped = 0  # events the core dropped (queue full)
        self._thread = threading.Thread(target=self._run, name=f"iad-events-{name}", daemon=True)

    def start(self) -> _EventPump:
        self._thread.start()
        self._ready.wait(2.0)
        return self

    @property
    def alive(self) -> bool:
        return self._thread.is_alive() and not self._stop.is_set()

    def on_thread(self) -> bool:
        """The calling thread is this pump's (a callback it runs)."""
        return self._thread is threading.current_thread()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread is not threading.current_thread() and self._thread.ident is not None:
            self._thread.join(timeout)

    def flush(self, timeout: float = 1.0) -> bool:
        """Wait until the events queued before this call have been dispatched (a wait of the pump that started after
        this call found the queue empty; the pump stops blocking meanwhile). Called on the pump's own thread (from a
        callback, e.g. one that recovers), the events queued so far are dispatched here, at once. False when that did
        not happen within ``timeout`` (a monitor that keeps the queue busy)."""
        if self.on_thread():
            self._drain()
            return True
        deadline = time.perf_counter() + timeout
        with self._cond:
            target = self._waits + 1
            self._flushers += 1
            try:
                while self._seen_empty < target and self._thread.is_alive():
                    remaining = deadline - time.perf_counter()
                    if remaining <= 0:
                        return False
                    self._cond.wait(remaining)
            finally:
                self._flushers -= 1
        return True

    def _next(self, event: Event, timeout: float) -> bool:
        code, _ = _invoke(self._lib, self._function, self._handle, timeout, ctypes.byref(event))
        return code == IAD_OK

    def _deliver(self, event: Event) -> None:
        self.dropped += int(event.dropped)
        try:
            self._dispatch(event)
        except Exception:
            log.exception("event dispatch failed")

    def _drain(self) -> None:
        """Dispatch what is queued now, on the pump's thread (inside a callback of an earlier event)."""
        event = Event()
        while not self._stop.is_set() and self._next(event, 0.0) and event.kind != IAD_EVENT_NONE:
            self._deliver(event)

    def _run(self) -> None:
        event = Event()
        try:
            while self._next(event, 0.0) and event.kind != IAD_EVENT_NONE:
                pass
        finally:
            self._ready.set()
        try:
            while not self._stop.is_set():
                with self._cond:
                    self._waits += 1
                    number = self._waits
                    wait = 0.0 if self._flushers else _PUMP_WAIT_S
                if not self._next(event, wait):
                    break
                if event.kind == IAD_EVENT_NONE:
                    with self._cond:
                        self._seen_empty = number
                        self._cond.notify_all()
                    continue
                self._deliver(event)
        finally:
            with self._cond:
                self._seen_empty = sys.maxsize  # nothing more is dispatched: no flush waits for it
                self._cond.notify_all()


class NativeMoveHandle:
    """A move started with ``wait=False`` (iad_move_handle; :class:`~iai_actuator_driver.ec.MoveHandle`): the command
    has been sent and every check has passed.

    * :meth:`wait` blocks until the move is done by its completion rule and returns the final status (on timeout or
      alarm a decel stop is sent and :class:`MotionTimeoutError` / :class:`EcAlarmError` raised); ``timeout=None`` =
      the time left of the move's own estimate. It waits in slices of 0.1 s (iad_move_handle_poll), so Ctrl+C reaches
      Python: the move is then stopped (decel) and ``KeyboardInterrupt`` raised.
    * :meth:`done` checks once without blocking; :attr:`result` is the final status once known (``None`` before).
    * :meth:`cancel` is a decel stop and makes the stop status the result.

    ``to_dict()`` makes it JSON-able (no I/O). The handle keeps its axis object alive; :meth:`close` (or leaving a
    ``with`` block, or the collector) releases it without touching the move.
    """

    def __init__(self, axis: NativeAxis, handle: int) -> None:
        self.axis = axis
        self._lib = axis._lib
        self._handle: int | None = handle
        self._finalizer = weakref.finalize(self, _release, self._lib, "iad_move_handle_close", handle)
        info = self._info()
        self.target_mm = float(info.target)
        self.band_mm = float(info.band)
        self.timeout = float(info.timeout_s)
        self.completion = _name_of(int(info.completion), _COMPLETIONS, "safe")
        self.sent_at = float(info.sent_at_s)  # monotonic seconds of the core's clock
        self.deadline = float(info.deadline_s)

    def _h(self) -> int:
        handle = self._handle
        if handle is None:
            raise ValueError("the move handle is closed")
        return handle

    def _info(self) -> MoveInfo:
        out = MoveInfo()
        _check(self._lib, "iad_move_handle_result", self._h(), ctypes.byref(out))
        return out

    @property
    def result(self) -> NativeStatus | None:
        info = self._info()
        return NativeStatus.from_c(info.result) if info.has_result else None

    @property
    def cancelled(self) -> bool:
        return bool(self._info().cancelled)

    def done(self) -> bool:
        """True once the move is done (one check; the safe rule needs its settle window over several calls). Raises
        like :meth:`wait` on an alarm or a lost servo (the axis stopped)."""
        flag = _c_bool(0)
        _check(self._lib, "iad_move_handle_done", self._h(), ctypes.byref(flag), None)
        return bool(flag.value)

    def wait(self, timeout: float | None = None) -> NativeStatus:
        """Block until done; returns the final status."""
        out = Status()
        end = None if timeout is None else time.perf_counter() + float(timeout)
        try:
            while True:
                if end is not None and end - time.perf_counter() <= _SLICE_S:  # the last part: stop on timeout
                    _check(self._lib, "iad_move_handle_wait", self._h(), max(end - time.perf_counter(), 0.0),
                           ctypes.byref(out))
                    return NativeStatus.from_c(out)
                flag = _c_bool(0)
                _check(self._lib, "iad_move_handle_poll", self._h(), _SLICE_S, ctypes.byref(flag), ctypes.byref(out))
                if flag.value:
                    return NativeStatus.from_c(out)
        except KeyboardInterrupt as interrupt:
            if getattr(interrupt, "iad_code", None) is None:  # Ctrl+C here, not a cancel from another thread
                try:
                    self.cancel()
                except Exception as exc:
                    interrupt.add_note(f"iai_actuator_driver: stopping the move on Ctrl+C failed: {exc}")
                else:
                    interrupt.add_note("iai_actuator_driver: decel stop sent on Ctrl+C")
            raise

    def cancel(self, *, wait: bool = False, timeout: float = 5.0) -> NativeStatus | None:
        """Decel stop (:meth:`NativeAxis.stop`); the status where it stopped becomes :attr:`result`."""
        out = _status_out()
        _check(self._lib, "iad_move_handle_cancel", self._h(), int(bool(wait)), _num(timeout), ctypes.byref(out))
        return _status_or_none(out)

    def to_dict(self) -> dict[str, Any]:
        """``target_mm``, ``band_mm``, ``completion``, ``timeout_s``, ``done``, ``cancelled``, ``position_mm`` (no
        I/O)."""
        return _read_json(self._lib, "iad_move_handle_json", self._h())

    def close(self) -> None:
        """Release the handle (the move itself is not affected). Safe to call twice."""
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_move_handle_close", handle)

    def __enter__(self) -> NativeMoveHandle:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __repr__(self) -> str:
        if self._handle is None:
            return f"NativeMoveHandle(target={self.target_mm}, closed)"
        info = self._info()
        state = "cancelled" if info.cancelled else ("done" if info.has_result else "running")
        return f"NativeMoveHandle(target={self.target_mm}, band={self.band_mm}, {self.completion}, {state})"


class NativeMonitor:
    """The status monitor of one axis on the native core (C++ AxisMonitor; as
    :class:`~iai_actuator_driver.monitor.AxisMonitor`): lean 8-word polls back to back on a thread of the core,
    read-only. Its callbacks run on this module's event thread: ``on_change(status, previous)`` when PNOW / ALMC / DSS1
    / DSS2 / DSSE change, ``on_done(status)`` when the axis becomes done by the fast rule, ``on_event(event)`` with a
    :class:`~iai_actuator_driver.recovery.RecoveryEvent` for each link event. Append to the lists to add callbacks."""

    def __init__(self, axis: NativeAxis) -> None:
        self.axis = axis
        self.on_change: list[Callable[[NativeStatus, NativeStatus | None], Any]] = []
        self.on_done: list[Callable[[NativeStatus], Any]] = []
        self.on_event: list[Callable[[RecoveryEvent], Any]] = []
        self.name: str | None = None
        self._previous: NativeStatus | None = None

    def _add(self, on_change: Any, on_done: Any, on_event: Any) -> None:
        for target, value in ((self.on_change, on_change), (self.on_done, on_done), (self.on_event, on_event)):
            if value is not None:
                target.extend([value] if callable(value) else list(value))

    @property
    def has_callbacks(self) -> bool:
        return bool(self.on_change or self.on_done or self.on_event)

    def _sample(self) -> MonitorSample | None:
        out = MonitorSample()
        code, details = _invoke(self.axis._lib, "iad_axis_monitor_latest", self.axis._h(), ctypes.byref(out))
        if code == IAD_ERR_NOT_READY:
            return None
        if code != IAD_OK:
            raise _make_error(code, details or {})
        return out

    def info(self) -> dict[str, Any] | None:
        """The monitor as a dict (``running``, ``polls``, ``error_count``, ``link_checks``, ``poll_rate_hz``,
        ``age_s``, ``link_state``, ``gap_s``, ``retries``, ``link_check_s``, ``error``...); ``None`` without one."""
        return self.axis._json("iad_axis_monitor_info_json")

    def to_dict(self) -> dict[str, Any]:
        return self.info() or {"running": False}

    @property
    def running(self) -> bool:
        info = self.info()
        return bool(info and info.get("running"))

    @property
    def latest(self) -> NativeMonitorSample | None:
        """The last published poll, ``None`` before the first."""
        sample = self._sample()
        return None if sample is None or sample.seq <= 0 else NativeMonitorSample.from_c(sample)

    @property
    def status(self) -> NativeStatus | None:
        sample = self.latest
        return None if sample is None else sample.status

    @property
    def age_s(self) -> float | None:
        sample = self.latest
        return None if sample is None else sample.age_s

    @property
    def poll_rate_hz(self) -> float | None:
        sample = self._sample()
        return None if sample is None else _opt(sample.poll_rate_hz)

    @property
    def polls(self) -> int:
        sample = self._sample()
        return 0 if sample is None else int(sample.polls)

    @property
    def error_count(self) -> int:
        sample = self._sample()
        return 0 if sample is None else int(sample.error_count)

    @property
    def link_state(self) -> str:
        return self.axis.link_state

    def fresh(self, max_age_s: float = 0.05) -> NativeStatus | None:
        """The latest status when it is younger than ``max_age_s``, else ``None``."""
        sample = self.latest
        if sample is None or sample.age_s is None or sample.age_s > max_age_s:
            return None
        return sample.status

    def wait_update(self, after_seq: int | None = None, *, timeout: float | None = None) -> NativeMonitorSample | None:
        """The first sample with ``seq > after_seq`` (default: the next one). ``None`` on timeout or when the monitor
        stops; raises when it ended on an error (port gone). Waits in slices (Ctrl+C reaches Python)."""
        end = None if timeout is None else time.perf_counter() + float(timeout)
        after = 0 if after_seq is None else int(after_seq)
        if after <= 0:  # "the next one": fixed now, so that the slices wait for the same sample
            sample = self._sample()
            after = 0 if sample is None else int(sample.seq)
        out = MonitorSample()
        while True:
            remaining = _SLICE_S if end is None else min(_SLICE_S, max(end - time.perf_counter(), 0.0))
            got = _c_bool(0)
            self.axis._call("iad_axis_monitor_wait_update", after, remaining, ctypes.byref(got), ctypes.byref(out))
            if got.value:
                return NativeMonitorSample.from_c(out)
            if (end is not None and time.perf_counter() >= end) or not self.running:
                return None

    def wait_done(self, timeout: float | None = None) -> NativeStatus | None:
        """Wait until a sample shows the axis done by the fast rule against its last target; ``None`` on timeout.
        Waits in slices (Ctrl+C reaches Python)."""
        end = None if timeout is None else time.perf_counter() + float(timeout)
        out = Status()
        while True:
            remaining = _SLICE_S if end is None else min(_SLICE_S, max(end - time.perf_counter(), 0.0))
            done = _c_bool(0)
            self.axis._call("iad_axis_monitor_wait_done", remaining, ctypes.byref(done), ctypes.byref(out))
            if done.value:
                return NativeStatus.from_c(out)
            if (end is not None and time.perf_counter() >= end) or not self.running:
                return None

    def stop(self, timeout: float = 1.0) -> bool:
        """Stop the monitor (:meth:`NativeAxis.stop_monitor`); True when it has stopped."""
        self.axis.stop_monitor(timeout)
        return not self.running

    def __enter__(self) -> NativeMonitor:
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def __repr__(self) -> str:
        return f"NativeMonitor({self.axis.name!r}, {'running' if self.running else 'stopped'})"


# ====================================================================== axis


class NativeAxis:
    """One axis on the native core: an ELECYLINDER or a CON controller at one slave address (C++ ``Axis``).

    Build it with :meth:`open` (a serial port), :meth:`open_sim` (the native simulator, no hardware) or from a cell
    (``cell["COM3:1"]``). Method names follow :class:`~iai_actuator_driver.ec.Elecylinder`; statuses come back as
    :class:`NativeStatus`, rows as the package's :class:`~iai_actuator_driver.ec.PositionEntry`, rich data as dicts.
    Every check, the completion rule, the STOP handling and the non-volatile write guard are the core's.
    """

    def __init__(self, lib: _Library, handle: int, *, name: str = "", owner: NativeCell | None = None) -> None:
        """Wrap a handle (use :meth:`open`, :meth:`open_sim` or a cell instead)."""
        self._lib = lib
        self._handle: int | None = handle
        self.name = name
        self._owner = owner  # keeps the cell (and so this axis) alive while this object lives
        self._motion_commanded = False
        self._finalizer = weakref.finalize(self, _release, lib, "iad_axis_close", handle)
        self._monitor = NativeMonitor(self)
        self._pump: _EventPump | None = None
        self._pump_lock = threading.Lock()
        self._link_listeners: list[Callable[[RecoveryEvent], Any]] = []  # one call's on_event (check_link, recover)
        self._item_listeners: list[Callable[[CheckItem], Any]] = []  # run_check's on_item
        self._frame_sink: Any = None  # the FRAME_SINK object the core holds (kept alive here)
        self.fast_notes: list[str] = []

    # ------------------------------------------------------------------ construction

    @classmethod
    def open(
        cls,
        port: str,
        *,
        baudrate: int | None = None,
        slave: int | None = None,
        timeout: float | None = None,
        retries: int | None = None,
        min_gap: float | None = None,
        line: str | None = None,
        position_unit_mm: float | None = None,
        fast: bool = False,
        **axis_options: Any,
    ) -> NativeAxis:
        """Open serial port ``port`` ("COM3", "/dev/ttyUSB0") once and return an axis that owns it (Axis::open).

        ``baudrate`` ``None`` finds the rate in place (115200, 38400, 9600, 230400...); ``slave`` 1..247 (default 1);
        ``timeout`` the read timeout (default 0.2 s); ``retries`` extra attempts (default 3); ``min_gap`` the silence
        between transactions (default 0.005 s). ``line``: ``"auto"`` (default: probe once for an RCON or ROBONET
        gateway), ``"direct"``, ``"rcon"``, ``"robonet"`` or ``"robonet-gateway"``; ``position_unit_mm`` the axes'
        position unit on an RCON line (0.01 or 0.001); ``fast`` then runs :meth:`enable_fast`. ``axis_options``:
        ``stroke`` (min, max), ``max_speed_mm_s``, ``max_accel_g``, ``max_push_pct``, ``poll_interval_s``,
        ``settle_s``, ``strategy`` ("coil", "iai", "iai2", "register"), ``auto_modbus_control``, ``push_settle_s``,
        ``completion`` ("safe" / "fast"). No identity is read: call :meth:`connect`. :class:`ModbusTimeout` when no
        rate answers, :class:`~iai_actuator_driver.modbus.TransportError` when the port is busy or absent.
        """
        lib = load()
        oo = _open_options(
            lib, slave=slave, baudrate=baudrate, timeout=timeout, retries=retries, min_gap=min_gap, line=line,
            position_unit_mm=position_unit_mm, fast=fast,
        )
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        _check(lib, "iad_axis_open", _utf8(port), _ref(oo), _ref(ao), ctypes.byref(out))
        return cls(lib, int(out.value or 0), name=f"{port}:{slave or 1}")

    @classmethod
    def open_sim(
        cls,
        profile: str = "slider",
        *,
        time_scale: float | None = None,
        position_mm: float | None = None,
        stroke_mm: float | None = None,
        max_speed_mm_s: float | None = None,
        sim_baudrate: int | None = None,
        pmsl: bool | None = None,
        slave: int | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        retries: int | None = None,
        line: str | None = None,
        fast: bool = False,
        **axis_options: Any,
    ) -> NativeAxis:
        """An axis on the native simulator (no hardware; never opens a port).

        ``profile``: "slider", "gripper", "rod", "pcon", "rotary" or "unknown" (the same units as
        :class:`~iai_actuator_driver.sim.SimulatedEC`). ``time_scale`` simulated seconds per real second (default 1;
        e.g. 20 for fast tests); ``position_mm`` / ``stroke_mm`` / ``max_speed_mm_s`` override the profile's;
        ``sim_baudrate`` models the line at that rate (the rate is then detected unless ``baudrate`` is given);
        ``pmsl`` the initial Modbus control (default on); ``slave`` the address the unit answers at. Axis options as
        :meth:`open`.
        """
        lib = load()
        so = _sim_options(
            lib, time_scale=time_scale, position_mm=position_mm, stroke_mm=stroke_mm, max_speed_mm_s=max_speed_mm_s,
            baudrate=sim_baudrate, pmsl=pmsl,
        )
        oo = _open_options(
            lib, slave=slave, baudrate=baudrate, timeout=timeout, retries=retries, min_gap=None, line=line, fast=fast
        )
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        _check(lib, "iad_axis_open_sim", _utf8(profile), _ref(so), _ref(oo), _ref(ao), ctypes.byref(out))
        return cls(lib, int(out.value or 0), name=f"SIM:{slave or 1}")

    @classmethod
    def open_bus(cls, bus: NativeBus, slave: int = 1, **axis_options: Any) -> NativeAxis:
        """(0.3) One axis at ``slave`` on an open bus (no I/O: call :meth:`connect`). The axis does not own the bus:
        closing it leaves the bus open. Axis options as :meth:`open`."""
        lib = load()
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        _check(lib, "iad_axis_open_bus", bus._h(), int(slave), _ref(ao), ctypes.byref(out))
        ax = cls(lib, int(out.value or 0), name=f"{bus.name}:{int(slave)}")
        ax._bus_ref = bus  # type: ignore[attr-defined]
        return ax

    @classmethod
    def open_transport(
        cls,
        transport: Any,
        *,
        baudrate: int | None = None,
        slave: int | None = None,
        timeout: float | None = None,
        retries: int | None = None,
        min_gap: float | None = None,
        line: str | None = None,
        position_unit_mm: float | None = None,
        fast: bool = False,
        **axis_options: Any,
    ) -> NativeAxis:
        """(0.3) :meth:`open` on any transport (a :class:`NativeTransport`, a :class:`NativeSim` /
        :class:`NativeSimLine` or a Python transport object): the rate is detected when the transport has one, the
        line probed. The axis owns its bus on the transport: closing the axis closes the transport."""
        lib = load()
        handle, made = _as_transport(transport)
        oo = _open_options(
            lib, slave=slave, baudrate=baudrate, timeout=timeout, retries=retries, min_gap=min_gap, line=line,
            position_unit_mm=position_unit_mm, fast=fast,
        )
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        try:
            _check(lib, "iad_axis_open_transport", handle._h(), _ref(oo), _ref(ao), ctypes.byref(out))
        finally:
            if made:
                handle.release()
        return cls(lib, int(out.value or 0), name=f"{handle.name}:{slave or 1}")

    def close(self) -> None:
        """Release a STOP / pause this axis holds, close the port when the axis owns it and release the handle
        (Axis::close; the status monitor stops with an owned axis). Safe to call twice. An axis of a cell is only
        released: it stays in the cell."""
        handle, self._handle = self._handle, None
        if handle is None:
            return
        self._finalizer.detach()
        pump, self._pump = self._pump, None
        try:
            _check(self._lib, "iad_axis_close", handle)  # wakes the event thread's waiting call
        finally:
            if pump is not None:
                pump.stop()

    @property
    def closed(self) -> bool:
        return self._handle is None

    def __enter__(self) -> NativeAxis:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, tb: object) -> None:
        try:
            # As Elecylinder: stop only when this object commanded motion (read-only use never writes to the unit).
            stopped = getattr(exc, "stop_attempted", False)
            if exc is not None and self._motion_commanded and not stopped and self._handle is not None:
                try:
                    self.stop()
                except Exception as stop_exc:  # never hide the original error
                    exc.add_note(f"iai_actuator_driver: stop on leaving the NativeAxis context failed: {stop_exc}")
                else:
                    exc.add_note("iai_actuator_driver: decel stop sent on leaving the NativeAxis context")
        finally:
            self.close()

    def __repr__(self) -> str:
        state = "closed" if self._handle is None else "open"
        return f"NativeAxis({self.name!r}, {state})"

    def _h(self) -> int:
        handle = self._handle
        if handle is None:
            raise ValueError(f"{self!r}: the axis is closed")
        return handle

    def _call(self, name: str, *args: Any) -> None:
        _check(self._lib, name, self._h(), *args)

    def _json(self, name: str, *args: Any) -> Any:
        return _read_json(self._lib, name, self._h(), *args)

    def _json_or_none(self, name: str) -> Any:
        try:
            return self._json(name)
        except EcNotReadyError:
            return None

    def _call_waiting(self, name: str, *args: Any) -> None:
        """A call that may wait long: Ctrl+C on the main thread becomes the cancel latch (a decel stop; 0.2 libraries:
        iad_axis_cancel), reset once the call returned."""
        handle = self._h()
        lib = self._lib
        if lib.has("iad_axis_cancel_latch"):
            _check_interruptible(
                lib, name, (handle, *args), lambda: _invoke(lib, "iad_axis_cancel_latch", handle),
                lambda: _invoke(lib, "iad_axis_cancel_reset", handle),
            )
        else:
            _check_interruptible(lib, name, (handle, *args), lambda: _invoke(lib, "iad_axis_cancel", handle))

    def _text_out(self, name: str) -> str:
        return _read_text(self._lib, name, self._h())

    def _events(self) -> _EventPump:
        """The event thread of this handle (started on first use)."""
        with self._pump_lock:
            pump = self._pump
            if pump is None or not pump.alive:
                pump = _EventPump(self._lib, "iad_axis_next_event", self._h(), self._dispatch, self.name).start()
                self._pump = pump
            return pump

    def _dispatch(self, e: Event) -> None:
        mon = self._monitor
        if e.kind == IAD_EVENT_STATUS:
            status = NativeStatus.from_c(e.status)
            previous, mon._previous = mon._previous, status
            for cb in list(mon.on_change):
                _call_back(cb, status, previous)
        elif e.kind == IAD_EVENT_DONE:
            status = NativeStatus.from_c(e.status)
            for cb in list(mon.on_done):
                _call_back(cb, status)
        elif e.kind in (IAD_EVENT_LINK, IAD_EVENT_FAULT, IAD_EVENT_LINK_STATE):
            event = _recovery_event(e)
            for cb in [*mon.on_event, *self._link_listeners]:
                _call_back(cb, event)
        elif e.kind == IAD_EVENT_CHECK_ITEM:
            from .check import CheckItem  # at the first use: see the imports

            item = CheckItem(_text(e.name), _text(e.cause), _text(e.message))
            for cb in list(self._item_listeners):
                _call_back(cb, item)

    def _with_listener(self, listeners: list[Any], callback: Any, call: Callable[[], Any]) -> Any:
        """Run ``call`` with ``callback`` getting this handle's events of that kind, delivered before returning."""
        if callback is None:
            return call()
        pump = self._events()
        listeners.append(callback)
        try:
            return call()
        finally:
            pump.flush()
            listeners.remove(callback)

    # ------------------------------------------------------------------ session

    def connect(self, *, take_control: bool = True) -> NativeIdentity:
        """Read identity and ranges and, with ``take_control``, take Modbus control at rest (Axis::connect)."""
        out = Identity()
        self._call("iad_axis_connect", int(bool(take_control)), ctypes.byref(out))
        return NativeIdentity.from_c(out)

    def identify(self) -> NativeIdentity:
        """Read the identity again (Axis::identify; READ-ONLY)."""
        out = Identity()
        self._call("iad_axis_identify", ctypes.byref(out))
        return NativeIdentity.from_c(out)

    @property
    def device_info(self) -> dict[str, Any] | None:
        """The full identity record of the last :meth:`connect` / :meth:`identify` (``None`` before; no I/O)."""
        return self._json_or_none("iad_axis_identity_json")

    def get_options(self) -> dict[str, Any]:
        """The axis settings: ``stroke``, ``max_speed_mm_s``, ``max_accel_g``, ``max_push_pct`` (``None``: the unit's
        own), ``poll_interval_s``, ``settle_s``, ``strategy``, ``auto_modbus_control``, ``push_settle_s`` (``None``:
        the default 0.15 s), ``completion``."""
        o = AxisOptions()
        self._call("iad_axis_get_options", ctypes.byref(o))
        methods = {v: k for k, v in _METHODS.items()}
        stroke = None if math.isnan(o.stroke_min) or math.isnan(o.stroke_max) else (o.stroke_min, o.stroke_max)
        return {
            "stroke": stroke,
            "max_speed_mm_s": _opt(o.max_speed_mm_s),
            "max_accel_g": _opt(o.max_accel_g),
            "max_push_pct": _opt(o.max_push_pct),
            "poll_interval_s": float(o.poll_interval_s),
            "settle_s": float(o.settle_s),
            "strategy": methods.get(int(o.method), str(o.method)),
            "auto_modbus_control": bool(o.auto_modbus_control),
            "push_settle_s": _opt(o.push_settle_s) or None,
            "completion": _name_of(int(o.completion), _COMPLETIONS, "safe"),
        }

    def set_options(self, **changes: Any) -> None:
        """Change axis settings (keywords as :meth:`open`); the others keep their values. A cap can only narrow the
        unit's own range."""
        o = AxisOptions()
        self._call("iad_axis_get_options", ctypes.byref(o))
        _apply_axis_options(o, changes)
        self._call("iad_axis_set_options", ctypes.byref(o))

    def _baud(self) -> tuple[int, bool]:
        rate, detected = ctypes.c_int32(0), _c_bool(0)
        self._call("iad_axis_baud_rate", ctypes.byref(rate), ctypes.byref(detected))
        return int(rate.value), bool(detected.value)

    @property
    def baudrate(self) -> int | None:
        """The port's baud rate (``None`` on a transport without one, such as the simulator without a modelled rate)."""
        return self._baud()[0] or None

    @property
    def baud_detected(self) -> bool:
        """Whether the rate was found by probing rather than given."""
        return self._baud()[1]

    def enable_modbus_control(self) -> None:
        """Take Modbus control (PMSL ON at rest, verified by PMSS)."""
        self._call("iad_axis_modbus_control", 1)

    def disable_modbus_control(self) -> None:
        """Give command authority back to PIO (PMSL OFF)."""
        self._call("iad_axis_modbus_control", 0)

    # ------------------------------------------------------------------ reading (READ-ONLY)

    def status(self) -> NativeStatus:
        """One status read (Axis::status)."""
        out = Status()
        self._call("iad_axis_status", ctypes.byref(out))
        return NativeStatus.from_c(out)

    def limits(self) -> dict[str, Any]:
        """Every range the driver enforces, read from the unit, with each range's source in words, as a dict with the
        keys of :meth:`Elecylinder.limits <iai_actuator_driver.ec.Elecylinder.limits>`."""
        return self._json("iad_axis_limits_json")

    def ranges(self) -> NativeLimits:
        """The same ranges from the fixed struct (iad_limits), without the sources."""
        out = Limits()
        self._call("iad_axis_limits", ctypes.byref(out))
        return NativeLimits.from_c(out)

    def read_alarm_history(self, *, include_empty: bool = False) -> list[dict[str, Any]]:
        """Alarm history, newest first: one dict per slot (``index``, ``code``, ``name``, ``level``, ``detail``,
        ``address``, ``time_ms``...). Empty slots are left out unless ``include_empty``."""
        return self._json("iad_axis_read_alarm_history_json", int(bool(include_empty)))

    def max_positions(self) -> int:
        """Stored rows on this unit (rows 0 .. n - 1)."""
        count = ctypes.c_int32(0)
        self._call("iad_axis_max_positions", ctypes.byref(count))
        return int(count.value)

    def read_position_table(self, n: int) -> PositionEntry:
        """Stored row ``n``, in the axis's units."""
        out = CPositionEntry()
        self._call("iad_axis_read_position_table", int(n), ctypes.byref(out))
        return _entry_from_c(out)

    # ------------------------------------------------------------------ control

    def servo(self, on: bool, *, wait: bool = True, timeout: float | None = None) -> NativeStatus | None:
        """Servo on / off (Axis::servo). A servo ON the status shows cannot work is refused before anything is sent
        (:class:`ServoBlockedError` with ``reasons``). Returns the status when ``wait``."""
        out = _status_out()
        o = ServoOptions(timeout_s=_num(timeout), wait=int(bool(wait)))
        self._call_waiting("iad_axis_servo", int(bool(on)), ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def home(
        self, *, wait: bool = True, timeout: float | None = None, method: str | None = None
    ) -> NativeStatus | None:
        """Home return (Axis::home); with ``wait`` waits for HEND and standstill and returns the status. ``method``:
        ``"coil"``, ``"iai"``, ``"iai2"``, ``"register"`` or ``None`` (the axis's strategy). With ``wait=False`` call
        :meth:`finish_home` once the axis is homed."""
        code = _METHOD_DEFAULT if method is None else _choice(method, _METHODS, "method")
        out = _status_out()
        self._motion_commanded = True
        o = HomeOptions(timeout_s=_num(timeout), wait=int(bool(wait)), method=code)
        self._call_waiting("iad_axis_home", ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def reset_alarm(self, *, method: str | None = None) -> NativeStatus:
        """Clear the current alarm (not a software reset) and return the new status (Axis::reset_alarm). A servo the
        alarm switched off stays off (its servo-on level is cleared first); a servo that is on stays on."""
        code = _METHOD_DEFAULT if method is None else _choice(method, _METHODS, "method")
        out = Status()
        self._call("iad_axis_reset_alarm", code, ctypes.byref(out))
        return NativeStatus.from_c(out)

    # ------------------------------------------------------------------ motion

    def _move(
        self, name: str, init: str, value: float, speed: float, accel: float, band: float, wait: bool,
        timeout: float | None, **extra: Any,
    ) -> NativeStatus | NativeMoveHandle | None:
        o = _move_options(self._lib, init, speed, accel, band, wait, timeout, **extra)
        if name == "iad_axis_check_move":
            out = _status_out()
            self._call(name, float(value), ctypes.byref(o), ctypes.byref(out))
            return _status_or_none(out)
        self._motion_commanded = True
        if not wait:  # the move handle of iad_axis_start_move_to / _by / start_jog
            handle = ctypes.c_void_p()
            self._call(name.replace("iad_axis_", "iad_axis_start_"), float(value), ctypes.byref(o),
                       ctypes.byref(handle))
            return NativeMoveHandle(self, int(handle.value or 0))
        out = _status_out()
        self._call_waiting(name, float(value), ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def check_move(
        self, mm: float, speed_mm_s: float = 50.0, accel_g: float = 0.1, band_mm: float = 0.1, *,
        decel_g: float | None = None, accel_pattern: str | None = None,
    ) -> NativeStatus | None:
        """Every check :meth:`move_to` runs, with nothing sent; raises what it would (Axis::check_move; READ-ONLY)."""
        return self._move(  # type: ignore[return-value]
            "iad_axis_check_move", "iad_move_options_init", mm, speed_mm_s, accel_g, band_mm, True, None,
            decel_g=decel_g, accel_pattern=accel_pattern,
        )

    def move_to(
        self,
        mm: float,
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        decel_g: float | None = None,
        accel_pattern: str | None = None,
        completion: str | None = None,
    ) -> NativeStatus | NativeMoveHandle | None:
        """Absolute move (Axis::move_to), checked against the unit's ranges before anything is sent
        (:class:`SoftLimitError`).

        Without ``decel_g`` (or equal to ``accel_g``) the direct block 0x9900 is used (one value for acceleration and
        deceleration); a different ``decel_g`` uses the command-profile path (FC17 0x0F00 + PCRF; acceleration
        checked against MAXA, deceleration against MAXD). ``accel_pattern``: ``None`` / ``"trapezoid"``,
        ``"s_motion"`` or ``"filter"`` (CTLF b6-b7; refused when the unit's MVPT lacks it). ``completion``: ``"safe"``
        / ``"fast"`` for this call (default: the axis option). With ``wait`` blocks until done and returns the final
        status (Ctrl+C on the main thread stops the axis, decel); with ``wait=False`` returns a
        :class:`NativeMoveHandle` right after the command. ``timeout`` ``None``: 2 x the move's estimate + 2 s."""
        return self._move(
            "iad_axis_move_to", "iad_move_options_init", mm, speed_mm_s, accel_g, band_mm, wait, timeout,
            decel_g=decel_g, accel_pattern=accel_pattern, completion=completion,
        )

    def move_by(
        self,
        delta_mm: float,
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        accel_pattern: str | None = None,
        completion: str | None = None,
    ) -> NativeStatus | NativeMoveHandle | None:
        """Move relative to the measured position, sent as an absolute target (Axis::move_by). Right after a move that
        ended by the fast rule (or was not waited for) and is still within its band, relative to that move's target
        instead, so fast steps add up. As :meth:`move_to`."""
        return self._move(
            "iad_axis_move_by", "iad_move_options_init", delta_mm, speed_mm_s, accel_g, band_mm, wait, timeout,
            accel_pattern=accel_pattern, completion=completion,
        )

    def jog(
        self,
        delta_mm: float,
        speed_mm_s: float = 10.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        wait: bool = True,
        timeout: float | None = None,
        *,
        completion: str | None = None,
    ) -> NativeStatus | NativeMoveHandle | None:
        """A small relative step at a low default speed (Axis::jog; limits ``jog_step``). As :meth:`move_to`."""
        return self._move(
            "iad_axis_jog", "iad_jog_options_init", delta_mm, speed_mm_s, accel_g, band_mm, wait, timeout,
            completion=completion,
        )

    def push(
        self,
        target_mm: float,
        band_mm: float,
        force_pct: float,
        direction: int = 1,
        speed_mm_s: float = 10.0,
        accel_g: float = 0.1,
        *,
        wait: bool = True,
        timeout: float | None = None,
    ) -> NativePushResult | None:
        """Push move (Axis::push): approach ``target_mm`` at ``speed_mm_s``, then push over ``band_mm`` at
        ``force_pct`` (PPOW = floor(pct x 255 / 100)) up to min(``max_push_pct``, MXPP 7.9). ``direction`` +1 pushes
        away from home, -1 toward home. Refused (nothing sent) when the unit cannot push, the band end is outside the
        stroke, the axis is moving, or it stands inside or past the push zone. With ``wait`` returns the outcome
        (``"contact"`` / ``"missed"``, each after it held for ``push_settle_s``) with its warnings; otherwise
        ``None`` right after the command."""
        o = PushOptions()
        _check(self._lib, "iad_push_options_init", ctypes.byref(o))
        o.direction = int(direction)
        o.speed = float(speed_mm_s)
        o.accel_g = float(accel_g)
        o.timeout_s = _num(timeout)
        o.wait = int(bool(wait))
        out = PushResult()
        self._motion_commanded = True
        self._call_waiting("iad_axis_push", float(target_mm), float(band_mm), float(force_pct), ctypes.byref(o),
                           ctypes.byref(out))
        if not wait:
            return None
        record = (self.last_result() or {}) if out.warning_count else {}
        return NativePushResult(
            outcome=_text(out.outcome_name), position_mm=float(out.position_mm),
            status=NativeStatus.from_c(out.status), warnings=tuple(str(w) for w in record.get("warnings") or ()),
        )

    def move_to_position(
        self,
        n: int,
        wait: bool = True,
        timeout: float | None = None,
        *,
        allow_above_caps: bool = False,
        speed_pct: float | None = None,
    ) -> NativeStatus | None:
        """Run stored row ``n`` (Axis::move_to_position; the row is read and checked first). ``speed_pct`` 1..100 % of
        the row's speed (nothing written); ``allow_above_caps``: only the unit's own limits apply to the row."""
        o = PositionMoveOptions()
        _check(self._lib, "iad_position_move_options_init", ctypes.byref(o))
        o.timeout_s = _num(timeout)
        o.speed_pct = _num(speed_pct)
        o.wait = int(bool(wait))
        o.allow_above_caps = int(bool(allow_above_caps))
        out = _status_out()
        self._motion_commanded = True
        self._call_waiting("iad_axis_move_to_position", int(n), ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def change_speed(
        self, speed_mm_s: float, *, wait: bool = False, timeout: float | None = None
    ) -> NativeStatus | None:
        """Change the speed of the move in progress (Axis::change_speed). Refused (nothing sent) without a move of this
        axis running, while paused, or outside the limits. With ``wait`` waits for the move and returns the status. A
        wait for this move with the estimated timeout (also in another thread) gets the estimate of the remaining
        distance at the new speed when that ends later; a ``timeout`` given to that wait stays."""
        out = _status_out()
        o = ChangeOptions(timeout_s=_num(timeout), wait=int(bool(wait)))
        self._call_waiting("iad_axis_change_speed", float(speed_mm_s), ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def change_accel(
        self, accel_g: float, *, wait: bool = False, timeout: float | None = None
    ) -> NativeStatus | None:
        """Change the acceleration (= deceleration) of the 0x9900 move in progress (Axis::change_accel; as
        :meth:`change_speed`)."""
        out = _status_out()
        o = ChangeOptions(timeout_s=_num(timeout), wait=int(bool(wait)))
        self._call_waiting("iad_axis_change_accel", float(accel_g), ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def move_sequence(
        self,
        points: Iterable[float | Mapping[str, float]],
        speed_mm_s: float = 50.0,
        accel_g: float = 0.1,
        band_mm: float = 0.1,
        *,
        blend_band: float | None = None,
        completion: str | None = None,
        timeout: float | None = None,
    ) -> list[NativeStatus]:
        """Absolute moves through ``points`` in order (Axis::move_sequence); one status per point.

        A point is a position, or a mapping with ``"mm"`` and optionally ``"speed_mm_s"``, ``"accel_g"``,
        ``"band_mm"`` (defaults: the arguments). Every point is checked before the first command
        (:class:`SoftLimitError` names the point; nothing sent). ``blend_band`` ``None``: each point completes by the
        completion rule before the next is sent; 0.01..10 units: the next target is sent once the axis is within
        ``blend_band`` of the current one (UNVERIFIED on hardware). ``timeout`` is per point."""
        plan: list[SequencePoint] = []
        for i, point in enumerate(points):
            p = SequencePoint()
            p.speed = p.accel_g = p.band = math.nan
            if isinstance(point, Mapping):
                unknown = set(point) - {"mm", "speed_mm_s", "accel_g", "band_mm"}
                if "mm" not in point or unknown:
                    raise SoftLimitError(f"point {i}: a mapping needs 'mm' and only speed_mm_s / accel_g / band_mm")
                p.position = float(point["mm"])
                p.speed = _slot("speed_mm_s", point.get("speed_mm_s"))
                p.accel_g = _slot("accel_g", point.get("accel_g"))
                p.band = _slot("band_mm", point.get("band_mm"))
            else:
                p.position = float(point)
            plan.append(p)
        o = SequenceOptions()
        _check(self._lib, "iad_sequence_options_init", ctypes.byref(o))
        o.speed, o.accel_g, o.band = float(speed_mm_s), float(accel_g), float(band_mm)
        o.blend_band = _num(blend_band)
        o.timeout_s = _num(timeout)
        o.completion = _code(completion, _COMPLETIONS, "completion")
        points_c = (SequencePoint * max(len(plan), 1))(*plan)
        statuses = (Status * max(len(plan), 1))()
        count = ctypes.c_size_t(0)
        self._motion_commanded = True
        self._call_waiting("iad_axis_move_sequence", points_c, len(plan), ctypes.byref(o), statuses, len(plan),
                           ctypes.byref(count))
        return [NativeStatus.from_c(s) for s in statuses[: min(count.value, len(plan))]]

    def wait_idle(
        self, target_mm: float | None = None, band_mm: float | None = None, timeout: float = 10.0, *,
        completion: str | None = None,
    ) -> NativeStatus:
        """Block until the move is done by the completion rule (Axis::wait_idle; ``completion`` ``"safe"`` /
        ``"fast"``, default the axis option), at ``target_mm`` within ``band_mm`` when given.
        :class:`MotionIncompleteError` when the axis settles away from the target."""
        out = Status()
        rule = 0 if completion is None else _choice(completion, _COMPLETIONS, "completion")  # 0: the axis option
        o = WaitIdleOptions(target=_num(target_mm), band=_num(band_mm), timeout_s=_num(timeout), completion=rule)
        self._call_waiting("iad_axis_wait_idle", ctypes.byref(o), ctypes.byref(out))
        return NativeStatus.from_c(out)

    def stop(self, *, wait: bool = False, timeout: float = 5.0) -> NativeStatus | None:
        """Decel stop (Axis::stop), with or without Modbus control; blocks for the deceleration. With ``wait``,
        :class:`MotionTimeoutError` when still moving after ``timeout``. May be called from another thread while a move
        waits on this axis. Returns the last status the stop read (``None`` when the core gave none)."""
        out = _status_out()
        o = StopOptions(timeout_s=_num(timeout), wait=int(bool(wait)))
        self._call("iad_axis_stop", ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    def pause(self) -> None:
        """Hold the move and keep the remaining travel (Axis::pause, STP ON). Callable from another thread."""
        self._call("iad_axis_pause")

    def resume(self) -> None:
        """Release the pause; a paused move continues by itself (Axis::resume, STP OFF)."""
        self._motion_commanded = True
        self._call("iad_axis_resume")

    def cancel_paused_move(self, *, timeout: float = 5.0) -> NativeStatus | None:
        """Drop the remaining travel of a paused move and release the pause (Axis::cancel_paused_move). Refused
        (nothing sent) without an active pause (:class:`EcNotReadyError`) or with a fault alarm
        (:class:`EcAlarmError`). Returns the status after the release."""
        out = _status_out()
        o = CancelPausedMoveOptions(timeout_s=_num(timeout))
        self._call("iad_axis_cancel_paused_move", ctypes.byref(o), ctypes.byref(out))
        return _status_or_none(out)

    # ------------------------------------------------------------------ position table (non-volatile)

    def write_position_table(
        self,
        n: int,
        entry: PositionEntry,
        allow_nonvolatile: bool = False,
        *,
        allow_unsafe: bool = False,
        force: bool = False,
    ) -> PositionEntry:
        """Write stored row ``n`` (16 words with the checksum) and verify it (Axis::write_position_table).

        Non-volatile, at rest only. When the row already holds these words nothing is written and the returned row has
        ``skipped=True``, unless ``force``. Rows past :meth:`max_positions`, unknown flag bits and role-bit changes are
        written with a warning; the simple-data rows 256-267 raise :class:`NonVolatileWriteError`.
        ``allow_nonvolatile`` / ``allow_unsafe``: accepted and ignored (no longer needed). Returns the row read back.
        The full result (warnings, note) is :meth:`last_result`."""
        c_entry = CPositionEntry()
        _check(self._lib, "iad_position_entry_init", ctypes.byref(c_entry))
        for name in ("position_mm", "band_mm", "speed_mm_s", "zone_plus_mm", "zone_minus_mm", "accel_g", "decel_g"):
            setattr(c_entry, name, float(getattr(entry, name)))
        for name in ("push_power", "load_threshold", "control_flags"):
            setattr(c_entry, name, int(getattr(entry, name)))
        c_entry.checksum = IAD_INT_UNKNOWN  # ignored in inputs: the core writes the checksum of the data
        o = _nv_options(self._lib, allow_nonvolatile, force, allow_unsafe)
        out = PositionWriteResult()
        self._call("iad_axis_write_position_table", int(n), ctypes.byref(c_entry), ctypes.byref(o), ctypes.byref(out))
        return _entry_from_c(out, skipped=bool(out.skipped))

    def teach_position(
        self,
        n: int,
        *,
        allow_nonvolatile: bool = False,
        allow_unsafe: bool = False,
        force: bool = False,
        method: str | None = None,
    ) -> PositionEntry:
        """Store the current position as row ``n`` (Axis::teach_position). Non-volatile, as
        :meth:`write_position_table`; refused before homing and while moving. ``method``: ``"table"`` (default: read
        the row, replace the position, write it back) or ``"coil"`` (EXPERIMENTAL MOD / POSR / TEAC coils; DOC for
        CON units, UNVERIFIED on an EC). Returns the row read back (``skipped=True`` when it already held that
        position). ``allow_nonvolatile`` / ``allow_unsafe``: accepted and ignored (no longer needed)."""
        o = _nv_options(self._lib, allow_nonvolatile, force, allow_unsafe)
        out = PositionWriteResult()
        code = 0 if method is None else _choice(method, _TEACH_METHODS, "method")  # 0: IAD_TEACH_TABLE
        self._call("iad_axis_teach_position", int(n), code, ctypes.byref(o), ctypes.byref(out))
        return _entry_from_c(out, skipped=bool(out.skipped))

    # ------------------------------------------------------------------ parameters

    def parameters(self) -> list[dict[str, Any]]:
        """The unit's named parameters, decoded (EC: P1-P18, CON: No.x; READ-ONLY): one dict each, keys as
        :meth:`Elecylinder.parameters <iai_actuator_driver.ec.Elecylinder.parameters>`."""
        return self._json("iad_axis_parameters_json")

    def read_parameter(self, key: int | str) -> dict[str, Any]:
        """One parameter (READ-ONLY): a number (``2``, ``"P2"``, ``"No.35"``) or a raw item (``"8.15"``), with its
        choices and the raw-item fields."""
        return self._json("iad_axis_read_parameter_json", _utf8(str(key)))

    def write_parameter(
        self,
        key: int | str,
        value: Any,
        *,
        commit: bool = True,
        allow_dangerous: bool = False,
        allow_unlisted: bool = False,
        allow_home_params: bool = False,
        force: bool = False,
    ) -> NativeParameterWriteResult:
        """Write one parameter, verify it by read-back and (``commit``) save it with PARW (Axis::write_parameter).

        ``value`` as the PC tool shows it: a number or a label (``"1:Forward"``, ``"disable"``). The guards refuse
        before anything is written (:class:`UnsafeCommandError`, :class:`SoftLimitError`); at rest only. A dangerous,
        home or unlisted parameter is written with a warning (``allow_dangerous`` / ``allow_unlisted`` /
        ``allow_home_params``: accepted and ignored, no longer needed). An unchanged value is not written or committed
        (``skipped``) unless ``force``. It applies after :meth:`software_reset` or a power cycle."""
        o = ParamWriteOptions()
        _check(self._lib, "iad_param_write_options_init", ctypes.byref(o))
        o.commit = int(bool(commit))
        o.allow_dangerous = int(bool(allow_dangerous))
        o.allow_unlisted = int(bool(allow_unlisted))
        o.allow_home_params = int(bool(allow_home_params))
        o.force = int(bool(force))
        out = ParamWriteResult()
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            self._call("iad_axis_write_parameter_number", _utf8(str(key)), float(value), ctypes.byref(o),
                       ctypes.byref(out))
        else:
            self._call("iad_axis_write_parameter", _utf8(str(key)), _utf8(str(value)), ctypes.byref(o),
                       ctypes.byref(out))
        record = self.last_result() or {}
        return NativeParameterWriteResult(
            number=int(out.number), key=_text(out.key), address=_text(out.address), name=_text(out.name),
            before=_text(out.before), after=_text(out.after), unit=_text(out.unit), changed=bool(out.changed),
            verified=bool(out.verified), committed=bool(out.committed), commit_confirmed=bool(out.commit_confirmed),
            saved=bool(out.saved), skipped=bool(out.skipped), forced=bool(out.forced),
            listed_for_model=bool(out.listed_for_model), verified_for_model=bool(out.verified_for_model),
            home_parameter=bool(out.home_parameter), warnings=tuple(str(w) for w in record.get("warnings") or ()),
            note=str(record.get("note") or ""), record=record,
        )

    # ------------------------------------------------------------------ non-volatile guard

    def _nv_counts(self) -> tuple[int, int, bool]:
        total, last_hour, uncommitted = ctypes.c_int64(0), ctypes.c_int32(0), _c_bool(0)
        self._call(
            "iad_axis_nonvolatile_writes", ctypes.byref(total), ctypes.byref(last_hour), ctypes.byref(uncommitted)
        )
        return int(total.value), int(last_hour.value), bool(uncommitted.value)

    def nonvolatile_writes(self) -> int:
        """Non-volatile write cycles this axis handle caused (parameter commits + position-table writes); skipped
        writes are not counted. The units' non-volatile memory lasts about 100,000 write cycles (IAI)."""
        return self._nv_counts()[0]

    def nonvolatile_writes_last_hour(self) -> int:
        """The same, counted over the last hour."""
        return self._nv_counts()[1]

    def has_uncommitted_parameters(self) -> bool:
        """Whether a :meth:`write_parameter` without ``commit`` left a value unsaved."""
        return self._nv_counts()[2]

    # ------------------------------------------------------------------ unit

    def set_baudrate(
        self,
        rate: int,
        *,
        method: str = "direct",
        settle_s: float | None = None,
        allow_unverified_code: bool = False,
    ) -> BaudChangeResult:
        """Change the unit's and the port's baud rate (Axis::set_baud_rate; volatile: until power-off). At rest only,
        and only when this axis is the only one on the bus. ``method`` ``"direct"`` or ``"break"``; a code not
        verified on hardware is sent with a warning (``allow_unverified_code``: accepted and ignored, no longer
        needed). Returns the package's :class:`~iai_actuator_driver.ec.BaudChangeResult`."""
        o = BaudOptions()
        _check(self._lib, "iad_baud_options_init", ctypes.byref(o))
        o.method = _choice(method, _BAUD_METHODS, "method")
        if settle_s is not None:
            o.settle_s = float(settle_s)
        o.allow_unverified_code = int(bool(allow_unverified_code))
        out = BaudResult()
        self._call("iad_axis_set_baud_rate", int(rate), ctypes.byref(o), ctypes.byref(out))
        methods = {v: k for k, v in _BAUD_METHODS.items()}
        from .values import BaudChangeResult  # at the first use: see the imports

        return BaudChangeResult(
            previous=int(out.previous), baudrate=int(out.baud_rate), method=methods.get(int(out.method), "direct"),
            code=int(out.code), frame=_text(out.frame), verified=bool(out.verified), probed=bool(out.probed),
            code_verified=bool(out.code_verified),
        )

    def software_reset(self, *, timeout: float | None = None, reset_wait_s: float | None = None) -> NativeResetResult:
        """Restart the unit's firmware and wait until it answers again (Axis::software_reset; at rest only, servo off
        first). Committed parameters apply afterwards. ``timeout`` default 10 s, ``reset_wait_s`` default 1 s."""
        o = ResetOptions()
        _check(self._lib, "iad_reset_options_init", ctypes.byref(o))
        if timeout is not None:
            o.timeout_s = float(timeout)
        if reset_wait_s is not None:
            o.reset_wait_s = float(reset_wait_s)
        out = ResetResult()
        self._call("iad_axis_software_reset", ctypes.byref(o), ctypes.byref(out))
        record = self.last_result() or {}
        return NativeResetResult(
            status=NativeStatus.from_c(out.status), accepted=bool(out.accepted), reply_lost=bool(out.reply_lost),
            confirmed=bool(out.confirmed), confirmed_by=_text(out.confirmed_by), baudrate=int(out.baud_rate) or None,
            baud_changed=bool(out.baud_changed), needs_home=bool(out.needs_home),
            uptime_before_s=_opt(out.uptime_before_s), uptime_after_s=_opt(out.uptime_after_s),
            elapsed_s=_opt(out.elapsed_s), notes=tuple(str(n) for n in record.get("notes") or ()),
        )

    def last_result(self) -> dict[str, Any] | None:
        """The full result (warnings, notes) of the last successful parameter write, position-table write, teach, baud
        change, software reset, push, link check, recovery, unit-info read, backup or self-test on this axis, as a
        dict; ``None`` when there is none. No I/O."""
        return self._json_or_none("iad_axis_last_result_json")

    # ------------------------------------------------------------------ status monitor

    @property
    def monitor(self) -> NativeMonitor | None:
        """The status monitor (:meth:`start_monitor`): ``None`` when none runs (one that ended on an error, the port
        gone, is still returned)."""
        info = self._monitor.info()
        return self._monitor if info is not None and (info.get("running") or info.get("error")) else None

    def start_monitor(self, **kwargs: Any) -> NativeMonitor:
        """Start the background status monitor (Axis::start_monitor; read-only lean polls on a thread of the core).

        ``kwargs``: ``gap_s`` (pause between polls, default 0.002), ``link_check_s`` (default 1 s, 0 = never),
        ``retries`` (default 0), ``on_change(status, previous)``, ``on_done(status)``, ``on_event(event)`` (link
        events as :class:`~iai_actuator_driver.recovery.RecoveryEvent`), ``events`` (IAD_EVENTS_* bits) and
        ``queue_capacity`` of the handle's event queue, ``name``. Callbacks run on this module's event thread.
        Already running: the callbacks given are added and the running monitor is returned. Stopped by
        :meth:`stop_monitor` and :meth:`close`."""
        allowed = {"gap_s", "link_check_s", "retries", "on_change", "on_done", "on_event", "events", "queue_capacity",
                   "name"}  # fmt: skip
        unknown = sorted(set(kwargs) - allowed)
        if unknown:
            raise TypeError(f"unknown monitor option(s) {unknown}; the native core takes {', '.join(sorted(allowed))}")
        mon = self._monitor
        mon._add(kwargs.get("on_change"), kwargs.get("on_done"), kwargs.get("on_event"))
        if kwargs.get("name") is not None:
            mon.name = str(kwargs["name"])
        if mon.has_callbacks:
            self._events()  # before the start: no event of the new monitor is dropped as backlog
        o = _monitor_options(self._lib, kwargs)
        self._call("iad_axis_start_monitor", ctypes.byref(o))
        return mon

    def stop_monitor(self, timeout: float = 1.0) -> None:
        """Stop the status monitor (no-op without one); its callbacks are dropped."""
        self._call("iad_axis_stop_monitor", _num(timeout))
        mon = self._monitor
        mon.on_change.clear()
        mon.on_done.clear()
        mon.on_event.clear()
        mon._previous = None

    # ------------------------------------------------------------------ fast mode

    def enable_fast(self, *, baudrate: int | None = FAST_BAUDRATE, monitor: bool = True) -> dict[str, Any]:
        """The fast path in one call (Axis::enable_fast): ``completion="fast"``, the status monitor on and the link
        raised to ``baudrate`` (230400 by default; ``None`` keeps the rate). The baud change is tried only when this
        axis owns the port, the transport has a baud rate and the axis is at rest; otherwise the rate is kept and the
        reason is in ``notes`` (also :attr:`fast_notes`). Safe to call again. Returns ``{"baudrate", "completion",
        "monitor", "notes"}``."""
        out = FastSummary()
        self._call("iad_axis_enable_fast", 0 if baudrate is None else int(baudrate), int(bool(monitor)),
                   ctypes.byref(out))
        return self._fast_summary(out)

    def disable_fast(self) -> dict[str, Any]:
        """Undo :meth:`enable_fast` (Axis::disable_fast): safe completion again and the monitor stopped; the baud rate
        is kept. Returns ``{"baudrate", "completion", "monitor", "notes"}``."""
        out = FastSummary()
        self._call("iad_axis_disable_fast", ctypes.byref(out))
        return self._fast_summary(out)

    def _fast_summary(self, out: FastSummary) -> dict[str, Any]:
        self.fast_notes = list(_joined(out.notes))
        return {
            "baudrate": int(out.baud_rate) or None, "completion": _name_of(int(out.completion), _COMPLETIONS, "safe"),
            "monitor": bool(out.monitor), "notes": list(self.fast_notes),
        }  # fmt: skip

    @property
    def fast_mode(self) -> bool:
        """:meth:`enable_fast` ran and no :meth:`disable_fast` since."""
        on = _c_bool(0)
        self._call("iad_axis_fast_mode", ctypes.byref(on))
        return bool(on.value)

    def set_frame_logger(self, logger: Callable[[str, bytes], Any] | None) -> None:
        """Install (or remove, ``None``) a ``logger(direction, frame)`` callback for every Modbus frame of this axis's
        port (``"tx"``, ``"rx"``, ``"rx-partial"``; on a shared port the frames of every axis of the port). It runs on
        the thread that sends: copy what you need and return quickly; never call the driver from it."""
        if logger is None:
            self._call("iad_axis_set_frame_sink", FRAME_SINK(), None)
            self._frame_sink = None
            return

        def sink(direction: int, data: Any, length: int, _user: Any) -> None:
            try:
                logger(_FRAME_DIRECTIONS.get(int(direction), str(direction)), ctypes.string_at(data, length))
            except Exception:  # a broken logger must never break communication
                log.exception("frame logger raised")

        c_sink = FRAME_SINK(sink)
        self._call("iad_axis_set_frame_sink", c_sink, None)
        self._frame_sink = c_sink  # the core holds the pointer: keep the object alive

    # ------------------------------------------------------------------ power / link loss (recovery)

    def track(
        self,
        *,
        name: str | None = None,
        alarm_history: bool = True,
        rows: bool | Iterable[int] | None = None,
        parameters: bool = False,
        backup: Mapping[str, Any] | str | os.PathLike[str] | None = None,
    ) -> dict[str, Any]:
        """Remember what the application has set up while the link is up (recovery::track; READ-ONLY): identity, baud
        rate, PC control, safety speed, HEND, uptime, the alarm history and optionally ``rows`` (``True`` = every
        row, or row numbers) and the ``parameters``; ``backup`` (a backup dict or file) to compare with later.
        Returns the session as a dict."""
        o = TrackOptions()
        _check(self._lib, "iad_track_options_init", ctypes.byref(o))
        o.alarm_history = int(bool(alarm_history))
        o.parameters = int(bool(parameters))
        self._call("iad_axis_track", ctypes.byref(o), _utf8(name), _utf8(_rows(rows)), _utf8(_backup_text(backup)))
        return self.session() or {}

    def session_from_backup(
        self, backup: Mapping[str, Any] | str | os.PathLike[str], *, name: str | None = None
    ) -> dict[str, Any]:
        """A session from a backup (dict or file; no I/O): a recovery then requires the unit the backup was taken
        from. Returns the session as a dict."""
        if isinstance(backup, Mapping):
            self._call("iad_axis_session_from_backup", _utf8(json.dumps(backup)), _utf8(name))
        else:
            self._call("iad_axis_session_from_backup_file", _utf8(os.fspath(backup)), _utf8(name))
        return self.session() or {}

    def session(self) -> dict[str, Any] | None:
        """The unit's recovery session as a dict (no I/O), ``None`` when it has none."""
        return self._json_or_none("iad_axis_session_json")

    def forget(self) -> None:
        """Drop the unit's session, link state and pending target (recovery::forget)."""
        self._call("iad_axis_forget")

    def note_target(self, target_mm: float | None) -> None:
        """Record the last commanded target (``None``: none, e.g. after a home return) for the recovery's position
        check (the axis records its own moves)."""
        self._call("iad_axis_note_target", _num(target_mm))

    def note_control(
        self, *, modbus_control: bool | None = None, safety_speed: bool | None = None,
        baudrate: int | None = None, homed: bool | None = None,
    ) -> None:
        """Tell the session about a deliberate change (``None`` = unchanged)."""

        def tri(value: bool | None) -> int:
            return IAD_UNKNOWN if value is None else int(bool(value))

        self._call("iad_axis_note_control", tri(modbus_control), tri(safety_speed), int(baudrate or 0), tri(homed))

    def check_link(self, *, on_event: Callable[[RecoveryEvent], Any] | None = None) -> NativeLinkCheck:
        """One cheap link check (recovery::check_link: one status read compared with the session; READ-ONLY).
        ``on_event`` gets the link events it raises (``"link_lost"``, ``"unit_restarted"``...) before this returns."""
        out = LinkCheck()
        self._with_listener(
            self._link_listeners, on_event, lambda: self._call("iad_axis_check_link", ctypes.byref(out))
        )  # fmt: skip
        return NativeLinkCheck.from_c(out, axis=self.name or None)

    @property
    def link_state(self) -> str:
        """The last known link state (``"ok"``, ``"port_vanished"``, ``"unit_silent"``, ``"unit_restarted"``,
        ``"unit_replaced"``, ``"recovering"``; no I/O)."""
        return self._text_out("iad_axis_link_state")

    def recover(
        self,
        *,
        timeout: float | None = None,
        policy: RecoveryPolicy | Mapping[str, Any] | None = None,
        check_rows: bool | Iterable[int] | None = None,
        backup: Mapping[str, Any] | str | os.PathLike[str] | None = None,
        name: str | None = None,
        on_event: Callable[[RecoveryEvent], Any] | None = None,
    ) -> NativeRecoveryReport:
        """Bring this axis back after a power or link loss and report what changed (recovery::recover_axis: wait for
        the port, reopen it, find the baud rate, require the same unit, restore baud rate / PC control / safety speed,
        assess alarm, HEND and position). **Never starts motion.** Without a :meth:`track` session the identity read
        by the last :meth:`connect` / :meth:`identify` is the baseline. ``timeout`` default 30 s; ``policy`` a
        :class:`~iai_actuator_driver.recovery.RecoveryPolicy` or a mapping of its fields (its ``check_rows`` and
        ``backup`` are used when the arguments are not given). Raises
        :class:`~iai_actuator_driver.recovery.RecoveryError` / :class:`~iai_actuator_driver.recovery.UnitReplacedError`
        (``report``: the core's report as a dict)."""
        values = asdict(policy) if _is_recovery_policy(policy) else dict(policy or {})
        if check_rows is None:
            check_rows = values.get("check_rows") or None
        if backup is None:
            backup = values.get("backup")
        c_policy = _recovery_policy(self._lib, policy)
        out = RecoverySummary()
        self._with_listener(
            self._link_listeners, on_event,
            lambda: self._call("iad_axis_recover", _num(timeout), _ref(c_policy), _utf8(_rows(check_rows)),
                               _utf8(_backup_text(backup)), _utf8(name), ctypes.byref(out)),
        )
        return NativeRecoveryReport.from_c(out, self.last_result())

    # ------------------------------------------------------------------ information (READ-ONLY)

    def unit_info(self, *, sections: Iterable[str] | None = None) -> dict[str, Any]:
        """Everything the unit tells about itself (info::unit_info; FC03 + FC11), the package's layout.
        ``sections``: some of "identity", "parameters", "monitor", "alarms", "maintenance", "positions" (default
        all)."""
        chosen = None if sections is None else ",".join(str(s) for s in sections)
        return self._json("iad_axis_read_unit_info_json", _utf8(chosen))

    def backup(
        self, path: str | os.PathLike[str] | None = None, *, rows: str | Iterable[int] = "used"
    ) -> dict[str, Any]:
        """A read-only backup in the package's schema (info::backup; FC03 + FC11 only). ``rows``:
        ``"used"`` (default), ``"all"`` or row numbers. ``path``: also written there (UTF-8 JSON)."""
        target = None if path is None else os.fspath(path)
        return self._json("iad_axis_backup_json", _utf8(_rows(rows)), _utf8(target))

    def run_check(
        self,
        *,
        motion: bool = False,
        port_label: str = "",
        step_mm: float = 5.0,
        speed_mm_s: float = 10.0,
        accel_g: float = 0.1,
        on_item: Callable[[CheckItem], Any] | None = None,
    ) -> NativeCheckReport:
        """The compatibility self-test (run_check; read-only unless ``motion``: then a short move out and back).
        ``on_item`` sees each line (:class:`~iai_actuator_driver.check.CheckItem`) as it completes, on the event
        thread. Never fails for what the unit answers (a FAIL line)."""
        o = CheckOptions()
        _check(self._lib, "iad_check_options_init", ctypes.byref(o))
        o.motion = int(bool(motion))
        o.step_mm, o.speed_mm_s, o.accel_g = float(step_mm), float(speed_mm_s), float(accel_g)
        out = CheckSummary()
        if motion:
            self._motion_commanded = True
        self._with_listener(
            self._item_listeners, on_item,
            lambda: self._call_waiting("iad_axis_self_test", ctypes.byref(o), _utf8(port_label or None),
                                       ctypes.byref(out)),
        )
        return NativeCheckReport(
            passed=bool(out.passed), items=int(out.items), failed=int(out.failed), warned=int(out.warned),
            skipped=int(out.skipped), record=self.last_result() or {},
        )

    def read_monitor(self) -> dict[str, Any]:
        """One read of the whole 0x9000 monitor block, decoded (position, velocity, current, current_pct, deviation,
        alarm, uptime_s, units, raw)."""
        return self._json("iad_axis_read_monitor_json")

    def read_lean_status(self, *, retries: int | None = None) -> NativeStatus:
        """The lean status: one read of 0x9000-0x9007 (8 words: no current or uptime, no velocity while moving)."""
        out = Status()
        self._call("iad_axis_read_lean_status", IAD_INT_UNKNOWN if retries is None else int(retries),
                   ctypes.byref(out))
        return NativeStatus.from_c(out)

    def read_boot_baudrate(self) -> int | None:
        """The power-up baud rate (BRSL 8.3 on an EC, No.16 on a CON unit); ``None`` when not readable."""
        rate = ctypes.c_int32(0)
        self._call("iad_axis_read_boot_baud_rate", ctypes.byref(rate))
        return _opt_int(rate.value)

    def read_home_pattern(self) -> int | None:
        """The unit's home pattern ORG (EC item 9.16); ``None`` when the item is not in the unit's directory."""
        known, pattern = _c_bool(0), ctypes.c_uint32(0)
        self._call("iad_axis_read_home_pattern", ctypes.byref(known), ctypes.byref(pattern))
        return int(pattern.value) if known.value else None

    def motion_caps(self) -> dict[str, Any]:
        """The caps moves are checked against and the documented minimum speed (iad_motion_caps): ``speed_cap``,
        ``accel_cap`` (0x9900 move), ``accel_only_cap`` / ``decel_cap`` (a move with its own deceleration),
        ``push_cap`` (%), ``min_speed`` and ``min_speed_source`` (``None`` where not known)."""
        out = MotionCaps()
        self._call("iad_axis_motion_caps", ctypes.byref(out))
        caps: dict[str, Any] = {
            name: _opt(getattr(out, name))
            for name in ("speed_cap", "accel_cap", "accel_only_cap", "decel_cap", "push_cap", "min_speed")
        }
        caps["min_speed_source"] = _text(out.min_speed_source)
        return caps

    @property
    def speed_cap_mm_s(self) -> float | None:
        return self.motion_caps()["speed_cap"]

    @property
    def accel_cap_g(self) -> float | None:
        return self.motion_caps()["accel_cap"]

    @property
    def accel_only_cap_g(self) -> float | None:
        """Highest acceleration of a move with its own deceleration (the profile path): min(max_accel_g, MAXA)."""
        return self.motion_caps()["accel_only_cap"]

    @property
    def decel_cap_g(self) -> float | None:
        """Highest deceleration of a move with its own deceleration (the profile path): min(max_accel_g, MAXD)."""
        return self.motion_caps()["decel_cap"]

    @property
    def push_cap_pct(self) -> float | None:
        return self.motion_caps()["push_cap"]

    def recommended_min_speed(self) -> tuple[float | None, str]:
        """(the documented minimum speed or ``None``, where it comes from or why it is unknown)."""
        caps = self.motion_caps()
        return caps["min_speed"], caps["min_speed_source"]

    # ------------------------------------------------------------------ line and gateway

    def line(self) -> dict[str, Any]:
        """The line this axis is on (kind, rules, the gateway's last status, position units, unreachable slaves,
        explanation; no I/O)."""
        return self._json("iad_axis_line_json")

    @property
    def line_name(self) -> str | None:
        """``"rcon"``, ``"robonet"``, ``"robonet-gateway"``, or ``None`` on a direct line."""
        return self._text_out("iad_axis_line_name") or None

    def gateway_status(self) -> dict[str, Any] | None:
        """A fresh read of the line's gateway status (RCON / ROBONET gateway; READ-ONLY); ``None`` without one."""
        return self._json("iad_axis_gateway_status_json")

    def gateway_monitor(self, slaves: Iterable[int] | None = None) -> dict[str, Any]:
        """The RCON all-axes monitor of this axis's line (at most two FC03 frames for 16 axes; READ-ONLY), keyed by
        slave. :class:`EcNotReadyError` on a line without an RCON gateway. DOC, UNVERIFIED on hardware."""
        chosen = None if slaves is None else ",".join(str(int(s)) for s in slaves)
        return self._json("iad_axis_gateway_monitor_json", _utf8(chosen))

    def set_position_unit(self, unit_mm: float) -> None:
        """State this axis's position unit (0.01 or 0.001 mm; RCON No.207) on its line; needed before mm-based motion
        on an RCON line."""
        self._call("iad_axis_set_position_unit", float(unit_mm))

    # ------------------------------------------------------------------ unit modes and strategy

    def set_action_mode(self, pc: bool = True, safety: bool | None = None, *, verify: bool = True) -> NativeStatus:
        """The action mode: ``pc`` PC / Modbus control (the PMSL coil, and the SFTY coil when the safety speed changes)
        or PIO (0x45 cmd 0x0C op1 = 0), ``safety`` the safety speed (``None`` keeps it). Refused while the axis moves,
        pauses, homes or pushes and on a line without 0x45 (nothing sent). With ``verify`` the result is read back
        (:class:`VerificationError`). Returns the status."""
        out = _status_out()
        safety_c = IAD_UNKNOWN if safety is None else int(bool(safety))
        self._call("iad_axis_set_action_mode", int(bool(pc)), safety_c, int(bool(verify)), ctypes.byref(out))
        return _status_or_none(out) or self.status()

    def set_safety_speed(self, on: bool, *, verify: bool = True) -> NativeStatus:
        """Switch the safety-speed mode (~100 mm/s cap when on; volatile): the SFTY coil, or DRG1 b14 on the register
        path. Takes Modbus control first if needed (at rest only). The driver never changes it by itself."""
        out = _status_out()
        self._call("iad_axis_set_safety_speed", int(bool(on)), int(bool(verify)), ctypes.byref(out))
        return _status_or_none(out) or self.status()

    def cancel_waits(self) -> None:
        """End every blocking call of this axis running now in another thread (a move, jog, home, push, stored row,
        wait_idle, a move handle's wait, the servo wait, a cell wait on this axis): it raises ``KeyboardInterrupt``
        soon, a motion wait after sending a decel stop. Calls started afterwards are not affected. Sends nothing
        itself; safe from any thread."""
        self._call("iad_axis_cancel")

    def cancel_latch(self) -> None:
        """(0.3) :meth:`cancel_waits` now, and every blocking call of this axis that starts while the latch is set is
        refused (``KeyboardInterrupt``, nothing sent) until :meth:`cancel_reset`. Stop, pause and resume still work.
        Safe from any thread."""
        self._call("iad_axis_cancel_latch")

    def cancel_reset(self) -> None:
        """(0.3) Clear the cancel latch."""
        self._call("iad_axis_cancel_reset")

    @property
    def cancel_latched(self) -> bool:
        """(0.3) Is the cancel latch set?"""
        value = _c_bool(0)
        self._call("iad_axis_cancel_latched", ctypes.byref(value))
        return bool(value.value)

    def bus(self) -> NativeBus:
        """(0.3) The axis's bus (borrowed: closing the returned object releases the handle only)."""
        out = ctypes.c_void_p()
        self._call("iad_axis_bus", ctypes.byref(out))
        return NativeBus(self._lib, int(out.value or 0), name=self.name.rsplit(":", 1)[0], owned=False)

    @property
    def strategy(self) -> str:
        """The command strategy for servo / home / alarm reset: ``"coil"``, ``"iai"``, ``"iai2"`` (EXPERIMENTAL) or
        ``"register"``. The coil strategy adapts by itself (register behind RCON or after an FC05 exception 01).
        Assign a name to change it."""
        method = ctypes.c_int32(0)
        self._call("iad_axis_strategy", ctypes.byref(method))
        return _name_of(int(method.value), _METHODS) or str(method.value)

    @strategy.setter
    def strategy(self, name: str) -> None:
        self._call("iad_axis_set_strategy", _choice(name, _METHODS, "strategy"))

    @property
    def stop_path(self) -> str:
        """How a stop is sent: ``"coil"`` (the STOP coil) or ``"register"`` (the coil-free substitute)."""
        return self._text_out("iad_axis_stop_path")

    def finish_home(self) -> None:
        """Release what a home start left set (the HOME coil, the DRG1 HOME bit): call it after ``home(wait=False)``
        once the axis is homed."""
        self._call("iad_axis_finish_home")


def _ref(struct_value: ctypes.Structure | None) -> Any:
    return None if struct_value is None else ctypes.byref(struct_value)


# ====================================================================== cell


class NativeCell:
    """Several axes over one or more ports on the native core, driven together (C++ ``Cell``).

    Build it with :meth:`open` or :meth:`open_sim`. ``cell.names`` are in port order, then slave order;
    ``cell["COM3:1"]`` or ``cell[0]`` is one axis (a :class:`NativeAxis`; every single-axis method works on it).
    Group methods take ``axes`` as names, indices or axes (default: all) and handle every listed axis before
    returning: a failure on some raises :class:`~iai_actuator_driver.cell.CellError` (``errors`` / ``results`` keyed by
    axis name), or with ``return_exceptions=True`` is returned in the dict in place of that axis's status.
    """

    def __init__(self, lib: _Library, handle: int) -> None:
        """Wrap a handle (use :meth:`open` or :meth:`open_sim` instead)."""
        self._lib = lib
        self._handle: int | None = handle
        self._axes: dict[str, NativeAxis] = {}
        self._motion_commanded = False
        self._finalizer = weakref.finalize(self, _release, lib, "iad_cell_close", handle)
        self._pump: _EventPump | None = None
        self._pump_lock = threading.Lock()
        self._previous: dict[str, NativeStatus] = {}
        self.on_fault: list[Callable[[str, str, BaseException | None], Any]] = []  # (name, kind, error)
        self.on_event: list[Callable[[RecoveryEvent], Any]] = []  # monitors' link events and recovery events
        self.on_change: list[Callable[[NativeStatus, NativeStatus | None], Any]] = []  # start_monitors
        self.on_done: list[Callable[[NativeStatus], Any]] = []
        self.on_link_change: list[Callable[[str, NativeLinkCheck], Any]] = []  # monitor_links
        try:
            self._names = self._read_names()
        except BaseException:
            self.close()
            raise

    # ------------------------------------------------------------------ construction

    @classmethod
    def open(
        cls,
        ports: str | Iterable[str] | None = None,
        slaves: str | Iterable[int] | Mapping[str, Iterable[int]] = "scan",
        *,
        max_axes: int | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        retries: int | None = None,
        scan_timeout: float | None = None,
        scan_slaves: Iterable[int] | None = None,
        identify: bool | None = None,
        stop_on_fault: bool | None = None,
        min_gap: float | None = None,
        line: str | None = None,
        position_unit_mm: float | None = None,
        on_fault: Callable[[str, str, BaseException | None], Any] | None = None,
        **axis_options: Any,
    ) -> NativeCell:
        """Open the ports, find the axes and return the cell (Cell::open; READ-ONLY: nothing is written to a unit).

        ``ports``: ``None`` = every IAI USB adapter of the PC, a port name, ``"COM3,COM4"`` or a list of names.
        ``slaves``: ``"scan"`` (every address of ``scan_slaves``, default 1..16, that answers), a list of addresses
        used on every port, or ``{port: [addresses]}`` (``ports`` may then be ``None``); listed addresses must answer.
        Ports that cannot be opened or where nothing answers are skipped when scanning (:meth:`info`
        ``port_errors``); :class:`EcNotReadyError` when no axis at all is found. ``line`` / ``position_unit_mm``
        every port's line kind and RCON position unit (as :meth:`NativeAxis.open`). ``on_fault(name, kind, error)``
        is called (on the event thread) once per axis when its link fails; with ``stop_on_fault`` (default) every
        other axis is then stopped. Axis options as :meth:`NativeAxis.open`.
        """
        lib = load()
        spec = _cell_ports(ports, slaves)
        scan_first = scan_last = None
        if scan_slaves is not None:
            addresses = sorted({int(s) for s in scan_slaves})
            if not addresses or addresses != list(range(addresses[0], addresses[-1] + 1)):
                raise ValueError("scan_slaves must be one contiguous range of addresses in the native core")
            scan_first, scan_last = addresses[0], addresses[-1]
        co = _cell_options(
            lib, timeout=timeout, min_gap=min_gap, scan_timeout=scan_timeout, max_axes=max_axes, baudrate=baudrate,
            retries=retries, scan_first=scan_first, scan_last=scan_last, identify=identify,
            stop_on_fault=stop_on_fault, line=line, position_unit_mm=position_unit_mm,
        )
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        _check(lib, "iad_cell_open", _utf8(spec), _ref(co), _ref(ao), ctypes.byref(out))
        return cls(lib, int(out.value or 0))._with_fault_handler(on_fault)

    def _with_fault_handler(self, on_fault: Callable[[str, str, BaseException | None], Any] | None) -> NativeCell:
        if on_fault is not None:
            self.on_fault.append(on_fault)
            self._events()
        return self

    @classmethod
    def open_sim(
        cls,
        profiles: str | Iterable[str] | None = None,
        slaves_per_line: int = 1,
        *,
        time_scale: float | None = None,
        position_mm: float | None = None,
        stroke_mm: float | None = None,
        max_speed_mm_s: float | None = None,
        pmsl: bool | None = None,
        max_axes: int | None = None,
        identify: bool | None = None,
        stop_on_fault: bool | None = None,
        timeout: float | None = None,
        retries: int | None = None,
        on_fault: Callable[[str, str, BaseException | None], Any] | None = None,
        **axis_options: Any,
    ) -> NativeCell:
        """A cell on simulated lines: one line ``"SIM1"``.. per profile (``"gripper,pcon"`` or a list; ``None`` = one
        ``"slider"`` line), each with ``slaves_per_line`` units (1..16) at addresses 1..n. Axis names ``"SIM1:1"``...
        Simulator settings as :meth:`NativeAxis.open_sim`; ``on_fault`` as :meth:`open`."""
        lib = load()
        if profiles is not None and not isinstance(profiles, str):
            profiles = ",".join(str(p) for p in profiles)
        so = _sim_options(
            lib, time_scale=time_scale, position_mm=position_mm, stroke_mm=stroke_mm, max_speed_mm_s=max_speed_mm_s,
            baudrate=None, pmsl=pmsl,
        )
        co = _cell_options(
            lib, timeout=timeout, min_gap=None, scan_timeout=None, max_axes=max_axes, baudrate=None, retries=retries,
            scan_first=None, scan_last=None, identify=identify, stop_on_fault=stop_on_fault,
        )
        ao = _axis_options(lib, axis_options)
        out = ctypes.c_void_p()
        _check(lib, "iad_cell_open_sim", _utf8(profiles), int(slaves_per_line), _ref(so), _ref(co), _ref(ao),
               ctypes.byref(out))
        return cls(lib, int(out.value or 0))._with_fault_handler(on_fault)

    @classmethod
    def open_transports(
        cls,
        transports: Mapping[str, Any] | Iterable[Any],
        slaves: Iterable[int] | None = None,
        *,
        timeout: float | None = None,
        retries: int | None = None,
        max_axes: int | None = None,
        identify: bool | None = None,
        stop_on_fault: bool | None = None,
        line: str | None = None,
        on_fault: Callable[[str, str, BaseException | None], Any] | None = None,
        **axis_options: Any,
    ) -> NativeCell:
        """(0.3) A cell on ready-made transports (Cell::open on named transports): ``{name: transport}`` or transports
        named by themselves; each a :class:`NativeTransport`, a :class:`NativeSim` / :class:`NativeSimLine` or a Python
        transport object. ``slaves`` ``None`` scans each, else these on every one. Closing the cell closes them."""
        lib = load()
        items = list(transports.items()) if isinstance(transports, Mapping) else [(None, t) for t in transports]
        made: list[NativeTransport] = []
        handles: list[NativeTransport] = []
        try:
            for _, target in items:
                handle, own = _as_transport(target)
                handles.append(handle)
                if own:
                    made.append(handle)
            names = ",".join(str(n) for n, _ in items) if isinstance(transports, Mapping) else None
            co = _cell_options(lib, timeout=timeout, retries=retries, max_axes=max_axes, identify=identify,
                               stop_on_fault=stop_on_fault, line=line)
            ao = _axis_options(lib, axis_options)
            array = (ctypes.c_void_p * max(len(handles), 1))(*(h._h() for h in handles))
            out = ctypes.c_void_p()
            _check(lib, "iad_cell_open_transports", array, len(handles), _utf8(names), _int_list(slaves), _ref(co),
                   _ref(ao), ctypes.byref(out))
        finally:
            for handle in made:
                handle.release()
        return cls(lib, int(out.value or 0))._with_fault_handler(on_fault)

    def close(self) -> None:
        """Release every STOP / pause the axes hold, close every port and release the handle (Cell::close; the
        monitors and the link monitor stop). Its axes stop working. Safe to call twice."""
        handle, self._handle = self._handle, None
        if handle is None:
            return
        self._finalizer.detach()
        pump, self._pump = self._pump, None
        try:
            for axis in list(self._axes.values()):
                try:
                    axis.close()
                except Exception:  # releasing an axis handle never stops the cell from closing
                    pass
        finally:
            try:
                _check(self._lib, "iad_cell_close", handle)  # wakes the event thread's waiting call
            finally:
                if pump is not None:
                    pump.stop()

    @property
    def closed(self) -> bool:
        return self._handle is None

    def __enter__(self) -> NativeCell:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None, tb: object) -> None:
        try:
            if exc is not None and self._motion_commanded and self._handle is not None:
                try:
                    self.stop_all(return_exceptions=True)
                except Exception as stop_exc:  # never hide the original error
                    exc.add_note(f"iai_actuator_driver: stop_all on leaving the cell failed: {stop_exc}")
                else:
                    exc.add_note("iai_actuator_driver: stop_all sent on leaving the cell")
        finally:
            self.close()

    def __repr__(self) -> str:
        state = "closed" if self._handle is None else "open"
        return f"NativeCell({list(self._names)!r}, {state})"

    def _h(self) -> int:
        handle = self._handle
        if handle is None:
            raise ValueError("the cell is closed")
        return handle

    def _read_names(self) -> tuple[str, ...]:
        count = ctypes.c_size_t(0)
        _check(self._lib, "iad_cell_axis_count", self._h(), ctypes.byref(count))
        return tuple(_read_text(self._lib, "iad_cell_axis_name", self._h(), i) for i in range(count.value))

    # ------------------------------------------------------------------ axes

    @property
    def names(self) -> tuple[str, ...]:
        """Axis names in port order, then slave order ("COM3:1"...)."""
        return self._names

    @property
    def axes(self) -> tuple[NativeAxis, ...]:
        return tuple(self[name] for name in self._names)

    def items(self) -> list[tuple[str, NativeAxis]]:
        return [(name, self[name]) for name in self._names]

    def __len__(self) -> int:
        return len(self._names)

    def __iter__(self) -> Iterator[NativeAxis]:
        return iter(self.axes)

    def __contains__(self, key: object) -> bool:
        return isinstance(key, str) and key in self._names

    def __getitem__(self, key: str | int) -> NativeAxis:
        """One axis by name or index (the same object each time; every :class:`NativeAxis` method works on it)."""
        name = self._name(key)
        axis = self._axes.get(name)
        if axis is None or axis.closed:
            out = ctypes.c_void_p()
            _check(self._lib, "iad_cell_open_axis", self._h(), _utf8(name), ctypes.byref(out))
            axis = NativeAxis(self._lib, int(out.value or 0), name=name, owner=self)
            self._axes[name] = axis
        return axis

    def _name(self, key: str | int | NativeAxis) -> str:
        if isinstance(key, NativeAxis):
            key = key.name
        if isinstance(key, bool):
            raise TypeError("an axis is a name or an index")
        if isinstance(key, int):
            try:
                return self._names[key]
            except IndexError:
                raise KeyError(f"no axis {key} in this cell of {len(self._names)}") from None
        if key not in self._names:
            raise KeyError(f"no axis {key!r} in this cell; axes: {', '.join(self._names)}")
        return key

    def _select(self, axes: Iterable[str | int | NativeAxis] | str | int | NativeAxis | None) -> list[str] | None:
        """The listed names (``None``: every axis)."""
        if axes is None:
            return None
        if isinstance(axes, str):
            items: list[Any] = [a for a in (part.strip() for part in axes.split(",")) if a]
        elif isinstance(axes, (int, NativeAxis)):
            items = [axes]
        else:
            items = list(axes)
        return [self._name(a) for a in items]

    def _select_known(
        self, axes: Iterable[str | int | NativeAxis] | str | int | NativeAxis | None
    ) -> tuple[list[str] | None, dict[str, BaseException]]:
        """:meth:`_select` of the stop path (stop_all, pause_all): the listed axes this cell has, and each key it does
        not have as an error of its own (keyed by the name, or the key's text), never a refusal of the whole command."""
        if axes is None:
            return None, {}
        if isinstance(axes, str):
            items: list[Any] = [a for a in (part.strip() for part in axes.split(",")) if a]
        elif isinstance(axes, (int, NativeAxis)):
            items = [axes]
        else:
            items = list(axes)
        names: list[str] = []
        unknown: dict[str, BaseException] = {}
        for item in items:
            try:
                name = self._name(item)
            except KeyError as exc:
                unknown.setdefault(item if isinstance(item, str) else repr(item), exc)
                continue
            if name not in names:
                names.append(name)
        return names, unknown

    def _stop_group(
        self, operation: str, function: str, axes: Iterable[str | int | NativeAxis] | str | int | NativeAxis | None,
        after: tuple[Any, ...], return_exceptions: bool,
    ) -> dict[str, NativeStatus | BaseException]:  # fmt: skip
        """A group command of the stop path: the known axes first (an unknown name never holds back their STOP or
        pause), then each unknown key as an error of its own; one CellError for all unless ``return_exceptions``."""
        names, unknown = self._select_known(axes)
        out = self._group(operation, function, names, (), after, True) if names is None or names else {}
        for key, exc in unknown.items():
            out.setdefault(key, exc)
        errors = {n: v for n, v in out.items() if isinstance(v, BaseException)}
        if errors and not return_exceptions:
            exc = CellError(operation, errors, {n: v for n, v in out.items() if isinstance(v, NativeStatus)})
            exc.iad_code = IAD_ERR_CELL  # type: ignore[attr-defined]
            raise exc
        return out

    def info(self) -> dict[str, Any]:
        """The cell (no I/O): ``axes`` (names), ``info`` ({name: identity}), ``port_errors`` ({port: message}),
        ``faults`` ({name: "port_vanished" | "unit_silent"})."""
        return _read_json(self._lib, "iad_cell_info_json", self._h())

    def clear_fault(self, axis: str | int | NativeAxis) -> None:
        """Forget a fault of one axis (Cell::clear_fault)."""
        _check(self._lib, "iad_cell_clear_fault", self._h(), _utf8(self._name(axis)))

    # ------------------------------------------------------------------ group commands

    def _group(
        self, operation: str, function: str, names: list[str] | None, before: tuple[Any, ...],
        after: tuple[Any, ...], return_exceptions: bool, *, waits: bool = False, axes_param: bool = True,
    ) -> dict[str, NativeStatus | BaseException]:
        """Run one group command: ``function(cell, *before, axes, *after, results, capacity, count)`` (no ``axes``
        without ``axes_param``). ``waits``: the call may wait long (Ctrl+C on the main thread becomes iad_cell_cancel
        on these axes, a decel stop)."""
        listed = names if names is not None else list(self._names)
        capacity = max(len(listed), 1)
        results = (CellResult * capacity)()
        count = ctypes.c_size_t(0)
        axes_arg = None if names is None else _utf8(",".join(names))
        args = (self._h(), *before, *((axes_arg,) if axes_param else ()), *after, results, capacity,
                ctypes.byref(count))
        if waits:
            handle = self._h()
            lib = self._lib
            latch = lib.has("iad_cell_cancel_latch")  # 0.3: a Ctrl+C just before the call is not lost
            code, details = _invoke_interruptible(
                lib, function, args,
                lambda: _invoke(lib, "iad_cell_cancel_latch" if latch else "iad_cell_cancel", handle, axes_arg),
                (lambda: _invoke(lib, "iad_cell_cancel_reset", handle, axes_arg)) if latch else None,
            )
        else:
            code, details = _invoke(self._lib, function, *args)
        details = details or {}
        if code not in (IAD_OK, IAD_ERR_CELL):
            raise _make_error(code, details)
        axis_details = details.get("axes") if isinstance(details.get("axes"), Mapping) else {}
        out: dict[str, NativeStatus | BaseException] = {}
        statuses: dict[str, NativeStatus] = {}
        errors: dict[str, BaseException] = {}
        for r in results[: min(count.value, capacity)]:
            name = _text(r.axis)
            if name not in listed and 0 <= r.index < len(listed):
                name = listed[r.index]  # (0.8) a name longer than the field: the index is the key
            if r.code == IAD_OK:
                statuses[name] = NativeStatus.from_c(r.status)
                out[name] = statuses[name]
            else:
                entry = dict(axis_details.get(name) or {}) if isinstance(axis_details.get(name), Mapping) else {}
                entry["code"] = int(r.code)
                entry["message"] = str(entry.get("message") or "") or _text(r.message)  # the record is not truncated
                errors[name] = out[name] = _make_error(int(r.code), entry)
        if code == IAD_ERR_CELL and not errors:  # no per-axis outcome came back: the record of the last error
            raise _make_error(code, details)
        if errors and not return_exceptions:
            exc = CellError(operation, errors, statuses)
            exc.iad_code = IAD_ERR_CELL  # type: ignore[attr-defined]
            exc.iad_details = dict(details)  # type: ignore[attr-defined]
            raise exc
        return out

    def status_all(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None, *, return_exceptions: bool = False
    ) -> dict[str, NativeStatus | BaseException]:
        """One status read per axis (Cell::status_all; ports in parallel)."""
        return self._group("status_all", "iad_cell_status_all", self._select(axes), (), (math.nan,), return_exceptions)

    def servo_all(
        self,
        on: bool,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        timeout: float | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException]:
        """Servo on / off on every listed axis and wait for each (Cell::servo_all)."""
        return self._group(
            "servo_all", "iad_cell_servo_all", self._select(axes), (int(bool(on)),), (_num(timeout),),
            return_exceptions, waits=True,
        )

    def home_all(
        self,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        timeout: float | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException]:
        """Home every listed axis at once and wait for all (Cell::home_all)."""
        self._motion_commanded = True
        return self._group(
            "home_all", "iad_cell_home_all", self._select(axes), (), (_num(timeout), _METHOD_DEFAULT, 1),
            return_exceptions, waits=True,
        )  # fmt: skip

    def move_all(
        self,
        targets: Mapping[str | int | NativeAxis, float],
        speed_mm_s: float | Mapping[str | int | NativeAxis, float] = 50.0,
        accel_g: float | Mapping[str | int | NativeAxis, float] = 0.1,
        band_mm: float | Mapping[str | int | NativeAxis, float] = 0.1,
        *,
        decel_g: float | Mapping[str | int | NativeAxis, float] | None = None,
        accel_pattern: str | None = None,
        wait: bool = True,
        timeout: float | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException] | None:
        """Start an absolute move on every axis of ``targets`` (``{axis: position}``), with as little delay as possible
        (Cell::move_all); with ``wait`` wait for all.

        ``speed_mm_s`` / ``accel_g`` / ``band_mm`` / ``decel_g`` are one value or ``{axis: value}`` (each in that
        axis's own units; an axis left out of a mapping gets the default); a ``decel_g`` different from the axis's
        ``accel_g`` uses the command-profile move. Every move is checked first: one bad value refuses the whole move
        and nothing is sent. A start that fails stops the axes already started. With ``wait=False`` returns ``None``
        once every move started: call :meth:`wait_all`."""
        if not targets:
            raise ValueError("move_all needs at least one target")
        names = [self._name(key) for key in targets]
        positions = (ctypes.c_double * len(names))(*(float(v) for v in targets.values()))
        mappings = {
            what: value for what, value in
            (("speed_mm_s", speed_mm_s), ("accel_g", accel_g), ("band_mm", band_mm), ("decel_g", decel_g))
            if isinstance(value, Mapping)
        }  # fmt: skip

        def scalar(value: Any, default: float | None) -> float | None:
            return default if isinstance(value, Mapping) else value

        o = _move_options(
            self._lib, "iad_move_options_init", scalar(speed_mm_s, 50.0), scalar(accel_g, 0.1),
            scalar(band_mm, 0.1), wait, timeout, decel_g=scalar(decel_g, None), accel_pattern=accel_pattern,
        )
        self._motion_commanded = True
        if not mappings:
            out = self._group(
                "move_all", "iad_cell_move_all", names, (),
                (positions, len(names), ctypes.byref(o), None, None, None, None), return_exceptions, waits=wait,
            )
            return out if wait or out else None
        arrays: list[Any] = []
        for what in ("speed_mm_s", "accel_g", "band_mm", "decel_g"):
            mapping = mappings.get(what)
            if mapping is None:
                arrays.append(None)
                continue
            by_name = {self._name(key): value for key, value in mapping.items()}
            unknown = sorted(set(by_name) - set(names))
            if unknown:
                raise KeyError(f"{what} names axes without a target: {', '.join(unknown)}")
            arrays.append((ctypes.c_double * len(names))(*(_num(by_name.get(n)) for n in names)))
        out = self._group(
            "move_all", "iad_cell_move_all", names, (),
            (positions, len(names), ctypes.byref(o), *arrays), return_exceptions, waits=wait,
        )
        return out if wait or out else None

    def move_each(
        self,
        moves: Mapping[str | int | NativeAxis, float | Mapping[str, Any]],
        *,
        wait: bool = True,
        timeout: float | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException] | None:
        """Start an absolute move on each axis with its own options (Cell::move_each): ``{axis: position}`` or
        ``{axis: {"mm", "speed_mm_s", "accel_g", "decel_g", "band_mm", "accel_pattern"}}`` (defaults 50 / 0.1 / the
        accel / 0.1 / trapezoid). Checked first like :meth:`move_all` (one bad value refuses all, nothing sent). With
        ``wait`` waits for all (``timeout`` ``None``: per axis); otherwise returns ``None`` once every move started."""
        if not moves:
            raise ValueError("move_each needs at least one move")
        names = [self._name(key) for key in moves]
        c_moves = _sized_array(AxisMove, len(names))
        for slot, name, spec in zip(c_moves, names, moves.values(), strict=True):
            spec = {"mm": spec} if not isinstance(spec, Mapping) else dict(spec)
            unknown = sorted(set(spec) - {"mm", "speed_mm_s", "accel_g", "decel_g", "band_mm", "accel_pattern"})
            if "mm" not in spec or unknown:
                raise ValueError(f"{name}: a move needs 'mm' (and only speed_mm_s / accel_g / decel_g / band_mm / "
                                 f"accel_pattern), got {sorted(spec)}")
            # (0.8) the move's own fields: NaN = the default (iad_move_options_init's); an explicit 0 is refused here
            slot.speed = _slot("speed_mm_s", spec.get("speed_mm_s"))
            slot.accel_g = _slot("accel_g", spec.get("accel_g"))
            slot.band = _slot("band_mm", spec.get("band_mm"))
            slot.timeout_s, slot.decel_g = math.nan, _num(spec.get("decel_g"))
            slot.accel_pattern = _code(spec.get("accel_pattern"), _ACCEL_PATTERNS, "accel_pattern")
            slot.position = float(spec["mm"])
            slot.axis = name.encode("utf-8")
        self._motion_commanded = True
        out = self._group(
            "move_each", "iad_cell_move_each", names, (c_moves, len(names), int(bool(wait)), _num(timeout), 1), (),
            return_exceptions, waits=wait, axes_param=False,
        )
        return out if wait or out else None

    def wait_all(
        self,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        timeout: float | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException]:
        """Wait until every listed axis is in position and settled (Cell::wait_all; an axis started by
        :meth:`move_all`: at its target and band). ``timeout`` ``None``: per axis from the move's estimate, else
        10 s."""
        return self._group(
            "wait_all", "iad_cell_wait_all", self._select(axes), (), (_num(timeout), 1), return_exceptions, waits=True
        )  # fmt: skip

    def stop_all(
        self,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        timeout: float = 5.0,
        return_exceptions: bool = False,
    ) -> dict[str, NativeStatus | BaseException]:
        """Decel stop on every listed axis as fast as possible (Cell::stop_all: STOP to all first, then each waits).
        May be called from another thread while a group command waits."""
        return self._stop_group("stop_all", "iad_cell_stop_all", axes, (_num(timeout), 0), return_exceptions)

    def _no_status(self, out: dict[str, NativeStatus | BaseException]) -> dict[str, None | BaseException]:
        return {name: value if isinstance(value, BaseException) else None for name, value in out.items()}

    def pause_all(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None, *, broadcast: bool = False,
        return_exceptions: bool = False,
    ) -> dict[str, None | BaseException]:
        """Pause every listed axis, keeping the remaining travel (Cell::pause_all; ports in parallel). ``broadcast``:
        first one broadcast pause on every daisy chain whose axes are all selected (never behind a gateway;
        UNVERIFIED), then the per-axis pause."""
        return self._no_status(
            self._stop_group("pause_all", "iad_cell_pause_all", axes, (int(bool(broadcast)),), return_exceptions)
        )

    def resume_all(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None, *, broadcast: bool = False,
        return_exceptions: bool = False,
    ) -> dict[str, None | BaseException]:
        """Release the pause on every listed axis (Cell::resume_all: the paused moves continue); ``broadcast`` as
        :meth:`pause_all`."""
        self._motion_commanded = True
        return self._no_status(self._group(
            "resume_all", "iad_cell_resume_all", self._select(axes), (), (int(bool(broadcast)),), return_exceptions
        ))  # fmt: skip

    def _cancel_call(
        self, operation: str, function: str, axes: Iterable[str | int | NativeAxis] | str | int | NativeAxis | None
    ) -> None:
        """One of the cancel calls (iad_cell_cancel, _cancel_latch, _cancel_reset): it reaches every listed axis this
        cell has, then each key it does not have is an error of its own (``KeyError``) in one :class:`CellError`, as
        the core reports them. A cancel (a Ctrl+C, a watchdog) never waits on an unrelated bad name; with only unknown
        keys nothing is called (an empty list would mean every axis)."""
        names, unknown = self._select_known(axes)
        if names is None or names:
            _check(self._lib, function, self._h(), None if names is None else _utf8(",".join(names)))
        if unknown:
            exc = CellError(operation, dict(unknown), {})
            exc.iad_code = IAD_ERR_CELL  # type: ignore[attr-defined]
            raise exc

    def cancel_waits(self, axes: Iterable[str | int | NativeAxis] | str | int | None = None) -> None:
        """:meth:`NativeAxis.cancel_waits` on every listed axis: a group command waiting on them ends with a
        :class:`~iai_actuator_driver.cell.CellError` whose axes say ``KeyboardInterrupt``. Safe from any thread. A listed
        key the cell does not have never holds the cancel back: the axes it has are cancelled, then each unknown key is
        an error of its own (``KeyError``) in one :class:`CellError`."""
        self._cancel_call("cancel", "iad_cell_cancel", axes)

    def cancel_latch(self, axes: Iterable[str | int | NativeAxis] | str | int | None = None) -> None:
        """(0.3) :meth:`NativeAxis.cancel_latch` on every listed axis (default: all); unknown keys as
        :meth:`cancel_waits`."""
        self._cancel_call("cancel_latch", "iad_cell_cancel_latch", axes)

    def cancel_reset(self, axes: Iterable[str | int | NativeAxis] | str | int | None = None) -> None:
        """(0.3) :meth:`NativeAxis.cancel_reset` on every listed axis (default: all); unknown keys as
        :meth:`cancel_waits`."""
        self._cancel_call("cancel_reset", "iad_cell_cancel_reset", axes)

    def bus(self, port: str) -> NativeBus:
        """(0.3) The bus of port ``port`` (borrowed: closing the returned object releases the handle only)."""
        out = ctypes.c_void_p()
        _check(self._lib, "iad_cell_bus", self._h(), _utf8(port), ctypes.byref(out))
        return NativeBus(self._lib, int(out.value or 0), name=port, owned=False)

    # ------------------------------------------------------------------ events, monitors, link loss

    def _events(self) -> _EventPump:
        with self._pump_lock:
            pump = self._pump
            if pump is None or not pump.alive:
                pump = _EventPump(self._lib, "iad_cell_next_event", self._h(), self._dispatch, "cell").start()
                self._pump = pump
            return pump

    def _dispatch(self, e: Event) -> None:
        name = _text(e.axis)
        if e.kind == IAD_EVENT_STATUS:
            status = NativeStatus.from_c(e.status)
            previous = self._previous.get(name)
            self._previous[name] = status
            for cb in list(self.on_change):
                _call_back(cb, status, previous)
        elif e.kind == IAD_EVENT_DONE:
            status = NativeStatus.from_c(e.status)
            for cb in list(self.on_done):
                _call_back(cb, status)
        elif e.kind == IAD_EVENT_LINK:
            event = _recovery_event(e)
            for cb in list(self.on_event):
                _call_back(cb, event)
        elif e.kind == IAD_EVENT_FAULT:
            message = _text(e.message)
            for cb in list(self.on_fault):
                _call_back(cb, name, _text(e.name), EcDriverError(message) if message else None)
        elif e.kind == IAD_EVENT_LINK_STATE:
            check = NativeLinkCheck(state=_text(e.name), reasons=_joined(e.message), axis=name or None)
            for cb in list(self.on_link_change):
                _call_back(cb, name, check)

    def start_monitors(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None, **monitor_kwargs: Any
    ) -> dict[str, NativeMonitor]:
        """Start the status monitor of every listed axis (Cell::start_monitors; READ-ONLY). Their link events also go
        to the fault handling (``on_fault``; with ``stop_on_fault`` every other axis is stopped). ``monitor_kwargs``:
        ``gap_s``, ``link_check_s``, ``retries``, ``events``, ``queue_capacity`` and the callbacks ``on_change(status,
        previous)``, ``on_done(status)``, ``on_event(event)`` (added to :attr:`on_change`, :attr:`on_done`,
        :attr:`on_event`; they run on the cell's event thread). Returns ``{name: NativeMonitor}``."""
        allowed = {"gap_s", "link_check_s", "retries", "on_change", "on_done", "on_event", "events", "queue_capacity"}
        unknown = sorted(set(monitor_kwargs) - allowed)
        if unknown:
            raise TypeError(f"unknown monitor option(s) {unknown}; the native core takes {', '.join(sorted(allowed))}")
        for target, key in ((self.on_change, "on_change"), (self.on_done, "on_done"), (self.on_event, "on_event")):
            value = monitor_kwargs.get(key)
            if value is not None:
                target.extend([value] if callable(value) else list(value))
        if self.on_change or self.on_done or self.on_event:
            self._events()
        names = self._select(axes)
        o = _monitor_options(self._lib, monitor_kwargs)
        _check(self._lib, "iad_cell_start_monitors", self._h(), None if names is None else _utf8(",".join(names)),
               ctypes.byref(o))
        return {name: self[name]._monitor for name in (names if names is not None else self._names)}

    def stop_monitors(self, axes: Iterable[str | int | NativeAxis] | str | int | None = None) -> None:
        """Stop the status monitor of every listed axis (no-op where none runs)."""
        names = self._select(axes)
        _check(self._lib, "iad_cell_stop_monitors", self._h(), None if names is None else _utf8(",".join(names)))

    def track_all(
        self,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        alarm_history: bool = True,
        parameters: bool = False,
        rows: bool | Iterable[int] | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, dict[str, Any] | BaseException]:
        """recovery::track on every listed axis (Cell::track_all; READ-ONLY). Returns ``{name: session dict}``."""
        o = TrackOptions()
        _check(self._lib, "iad_track_options_init", ctypes.byref(o))
        o.alarm_history = int(bool(alarm_history))
        o.parameters = int(bool(parameters))
        out = self._group(
            "track_all", "iad_cell_track_all", self._select(axes), (), (ctypes.byref(o), _utf8(_rows(rows))), True
        )  # fmt: skip
        sessions = self.last_result() or {}
        return self._finish("track_all", out, sessions, return_exceptions)

    def _finish(
        self, operation: str, out: Mapping[str, Any], records: Mapping[str, Any], return_exceptions: bool
    ) -> dict[str, Any]:
        """Per-axis records in place of the statuses; a CellError for the failed axes unless ``return_exceptions``."""
        result = {name: value if isinstance(value, BaseException) else records.get(name) for name, value in out.items()}
        errors = {name: value for name, value in result.items() if isinstance(value, BaseException)}
        if errors and not return_exceptions:
            raise CellError(operation, errors, {n: v for n, v in result.items() if n not in errors})
        return result

    def check_links(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None
    ) -> dict[str, NativeLinkCheck | BaseException]:
        """recovery::check_link on every listed axis (Cell::check_links; one status read each, ports in parallel). A
        fault goes to ``on_fault`` (and with ``stop_on_fault`` stops the other axes). Returns ``{name: link check}``
        (an error in place of a check whose check itself failed)."""
        names = self._select(axes)
        listed = names if names is not None else list(self._names)
        checks = (LinkCheck * max(len(listed), 1))()
        count = ctypes.c_size_t(0)
        _check(self._lib, "iad_cell_check_links", self._h(), None if names is None else _utf8(",".join(names)),
               checks, len(listed), ctypes.byref(count))
        record = self.last_result() or {}
        errors = record.get("errors") if isinstance(record.get("errors"), Mapping) else {}
        out: dict[str, NativeLinkCheck | BaseException] = {}
        for name, check in zip(listed, checks[: min(count.value, len(listed))], strict=False):
            if _text(check.state) == "error":
                out[name] = EcDriverError(str(errors.get(name) or _text(check.error) or "link check failed"))
            else:
                out[name] = NativeLinkCheck.from_c(check, axis=name)
        return out

    def monitor_links(
        self,
        interval_s: float = 1.0,
        *,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        on_change: Callable[[str, NativeLinkCheck], Any] | None = None,
    ) -> None:
        """Start a background link monitor on the listed axes every ``interval_s`` (Cell::monitor_links): faults go to
        ``on_fault``, ``on_change(name, check)`` sees every state change (on the cell's event thread). It never
        recovers or moves anything; it replaces one started earlier; stopped by :meth:`stop_link_monitor` and
        :meth:`close`."""
        if on_change is not None:
            self.on_link_change.append(on_change)
            self._events()
        names = self._select(axes)
        _check(self._lib, "iad_cell_monitor_links", self._h(), float(interval_s),
               None if names is None else _utf8(",".join(names)))

    def stop_link_monitor(self) -> None:
        """Stop the link monitor (no-op without one)."""
        _check(self._lib, "iad_cell_stop_link_monitor", self._h())
        self.on_link_change.clear()

    def recover_all(
        self,
        axes: Iterable[str | int | NativeAxis] | str | int | None = None,
        *,
        timeout: float | None = None,
        policy: RecoveryPolicy | Mapping[str, Any] | None = None,
        check_rows: bool | Iterable[int] | None = None,
        return_exceptions: bool = False,
    ) -> dict[str, dict[str, Any] | BaseException]:
        """recovery::recover on every listed axis (Cell::recover_all; ports in parallel). **Never starts motion.**
        Returns ``{name: report dict}`` (the RecoveryReport.to_dict layout); a failure raises
        :class:`~iai_actuator_driver.cell.CellError` unless ``return_exceptions``."""
        values = asdict(policy) if _is_recovery_policy(policy) else dict(policy or {})
        if check_rows is None:
            check_rows = values.get("check_rows") or None
        c_policy = _recovery_policy(self._lib, policy)
        out = self._group(
            "recover_all", "iad_cell_recover_all", self._select(axes), (),
            (_num(timeout), _ref(c_policy), _utf8(_rows(check_rows))), True,
        )
        record = self.last_result() or {}
        reports = record.get("reports") if isinstance(record.get("reports"), Mapping) else {}
        return self._finish("recover_all", out, reports, return_exceptions)

    # ------------------------------------------------------------------ information, lines, backup (READ-ONLY)

    def monitor_all(
        self, axes: Iterable[str | int | NativeAxis] | str | int | None = None, *, return_exceptions: bool = False
    ) -> dict[str, dict[str, Any] | BaseException]:
        """:meth:`NativeAxis.read_monitor` of every listed axis (Cell::monitor_all; ports in parallel)."""
        names = self._select(axes)
        record = _read_json(self._lib, "iad_cell_monitor_all_json", self._h(),
                            None if names is None else _utf8(",".join(names)))
        readings = record.get("readings") or {}
        errors = {name: EcDriverError(str(message)) for name, message in (record.get("errors") or {}).items()}
        out: dict[str, Any] = {name: errors.get(name, readings.get(name))
                               for name in (names if names is not None else self._names)}
        if errors and not return_exceptions:
            raise CellError("monitor_all", errors, {n: v for n, v in out.items() if n not in errors})
        return out

    def lines(self) -> dict[str, str]:
        """The line kind of every port (``"direct"``, ``"rcon"``, ``"robonet"``, ``"robonet-gateway"``; no I/O)."""
        return {port: str(line.get("kind")) for port, line in self.line_info().items()}

    def line_info(self) -> dict[str, dict[str, Any]]:
        """Every port's line in full (kind, rules, the gateway's last status, position units, unreachable slaves,
        explanation; no I/O)."""
        return _read_json(self._lib, "iad_cell_lines_json", self._h())

    def gateway_status_all(self) -> dict[str, Any]:
        """A fresh read of every gateway's status (``{port: status}``; READ-ONLY)."""
        return _read_json(self._lib, "iad_cell_gateway_status_json", self._h())

    def set_position_unit(self, port: str, unit_mm: float, slaves: Iterable[int] | None = None) -> None:
        """State the position unit (0.01 or 0.001 mm) of the listed slaves (default: every axis) on ``port``."""
        chosen = None if slaves is None else ",".join(str(int(s)) for s in slaves)
        _check(self._lib, "iad_cell_set_position_unit", self._h(), _utf8(port), float(unit_mm), _utf8(chosen))

    def gateway_monitor(self, port: str, slaves: Iterable[int] | None = None) -> dict[str, Any]:
        """The RCON all-axes monitor of ``port`` (READ-ONLY), as :meth:`NativeAxis.gateway_monitor`."""
        chosen = None if slaves is None else ",".join(str(int(s)) for s in slaves)
        return _read_json(self._lib, "iad_cell_gateway_monitor_json", self._h(), _utf8(port), _utf8(chosen))

    def backup(
        self, directory: str | os.PathLike[str], axes: Iterable[str | int | NativeAxis] | str | int | None = None, *,
        rows: str | Iterable[int] = "used",
    ) -> dict[str, str]:
        """Read-only backup of every listed axis into ``directory``, one ``<axis>.json`` each (Cell::backup). Returns
        ``{axis name: file path}``, or ``{name: "error: ..."}`` for an axis that did not answer."""
        names = self._select(axes)
        return _read_json(self._lib, "iad_cell_backup_json", self._h(),
                          None if names is None else _utf8(",".join(names)), _utf8(os.fspath(directory)),
                          _utf8(_rows(rows)))

    def last_result(self) -> dict[str, Any] | None:
        """The record of the last :meth:`track_all`, :meth:`check_links`, :meth:`recover_all`, :meth:`backup` or
        coordinated run on this cell (no I/O); ``None`` when there is none."""
        try:
            return _read_json(self._lib, "iad_cell_last_result_json", self._h())
        except EcNotReadyError:
            return None

    # ------------------------------------------------------------------ coordinated motion

    def coordinated_plan(self, request: Mapping[str, Any] | str) -> dict[str, Any]:
        """Dry run of a coordinated path (CoordinatedGroup::plan; reads only, no motion): the request is the JSON
        object of the header (``axes``, ``path``, ``speed``, ``accel_g``, ``mode``...). Returns the plan.
        :class:`SoftLimitError` for an infeasible path (``iad_details``: ``reasons``, ``max_speed``)."""
        text = request if isinstance(request, str) else json.dumps(request)
        return _read_json(self._lib, "iad_cell_coord_plan_json", self._h(), _utf8(text))

    def coordinated_run(self, request: Mapping[str, Any] | str) -> dict[str, Any]:
        """Plan and run a coordinated path (CoordinatedGroup::run); blocks until it is done and returns the result.
        An outside stop, a fault, a link fault or a late frame stops every axis of the group (decel) and raises
        :class:`EcDriverError` (``iad_details``: ``axis``, ``cause``, ``stop``, ``result``). Ctrl+C on the main thread
        stops every axis (decel). UNVERIFIED on hardware."""
        text = request if isinstance(request, str) else json.dumps(request)
        handle = self._h()
        self._motion_commanded = True
        size = _JSON_START
        buffer = ctypes.create_string_buffer(size)
        required = ctypes.c_size_t(0)
        capacity = max(len(self._names), 1)

        def stop_all() -> Any:  # iad_cell_cancel does not end a run; an outside stop does
            results, count = (CellResult * capacity)(), ctypes.c_size_t(0)
            return _invoke(self._lib, "iad_cell_stop_all", handle, None, 5.0, 0, results, capacity, ctypes.byref(count))

        code, details = _invoke_interruptible(
            self._lib, "iad_cell_coord_run_json", (handle, _utf8(text), buffer, size, ctypes.byref(required)), stop_all
        )
        if code == IAD_ERR_BUFFER_TOO_SMALL:  # the run is kept: read it without running again
            return self.last_result() or {}
        if code != IAD_OK:
            raise _make_error(code, details or {})
        return json.loads(buffer.value.decode("utf-8", errors="replace"))


def _cell_ports(
    ports: str | Iterable[str] | None, slaves: str | Iterable[int] | Mapping[str, Iterable[int]]
) -> str | None:
    """The ``ports`` string of iad_cell_open: NULL, "COM3,COM4" (scanned) or "COM3:1,COM3:2" (exactly these)."""
    if isinstance(slaves, str):
        if slaves != "scan":
            raise ValueError('slaves must be "scan", a list of addresses or {port: [addresses]}')
        if ports is None or isinstance(ports, str):
            return ports
        return ",".join(str(p) for p in ports)
    if isinstance(slaves, Mapping):
        pairs = [(str(port), int(s)) for port, addresses in slaves.items() for s in addresses]
    else:
        if ports is None:
            raise ValueError("a list of slaves needs the ports it applies to (or give {port: [addresses]})")
        names = [p.strip() for p in ports.split(",")] if isinstance(ports, str) else [str(p) for p in ports]
        pairs = [(port, int(s)) for port in names if port for s in slaves]
    if not pairs:
        raise ValueError("no slave addresses given")
    return ",".join(f"{port}:{slave}" for port, slave in pairs)



# ====================================================================== 0.3: callbacks into Python

IAD_SIM_PASS = 0
IAD_SIM_DROP = 1
IAD_SIM_REPLY = 2
IAD_SIM_FAIL = 3  # ABI 0.6 (core-sim-gw): the request fails (IAD_ERR_TRANSPORT with the text in the reply buffer)
IAD_SIM_ABORT = 4  # ABI 0.7 (py-fixes): the request fails with the hook's own error (IAD_ERR_DRIVER, not a link fault)
IAD_CALLBACK_ABORT = 2  # ABI 0.7 (py-fixes): a transport callback's own error (IAD_ERR_DRIVER, not the port's failure)


def sim_exception(code: int) -> int:
    """IAD_SIM_EXCEPTION(code): a request hook's answer "Modbus exception ``code``" (1..255)."""
    return 0x100 | (int(code) & 0xFF)


_LOG_LEVEL_NAMES = {"debug": 0, "info": 1, "warning": 2, "error": 3}

# Objects the core calls back (a Python transport, a request hook, a clock, a frame sink), by the id passed as the
# callback's user pointer. The trampolines below are module globals: they live as long as the process (a collected
# callback pointer called later would crash it). A transport's entry goes when the core releases it; the others stay
# (a few bytes each).
_callback_objects: dict[int, Any] = {}
_callback_ids = iter(range(1, 2**62))
_callback_lock = threading.Lock()


def _keep_callback(obj: Any) -> int:
    with _callback_lock:
        key = next(_callback_ids)
        _callback_objects[key] = obj
    return key


def _callback_object(user: Any) -> Any:
    return _callback_objects[int(user or 0)]


def _callback_failed(exc: BaseException, *, propagate: bool = False) -> None:
    """A callback raised: keep the exception for the call that ran it (it becomes the cause of the error the core
    raises; with ``propagate`` it is the error raised) and give the core its text (iad_callback_error). A Ctrl+C
    (KeyboardInterrupt) is never replaced by a later failure: the call raises it (:func:`_invoke`). Never raises."""
    try:
        if not isinstance(getattr(_tls, "callback_exc", None), KeyboardInterrupt):
            _tls.callback_exc = exc
            _tls.callback_propagate = propagate
        lib = _library
        if lib is not None and lib.has("iad_callback_error"):
            lib.fn("iad_callback_error")(_utf8(f"{type(exc).__name__}: {exc}"))
    except BaseException as late:  # a Ctrl+C right here is kept too
        _interrupted(late)


def _interrupted(exc: BaseException) -> None:
    """A callback that must not fail (a sink, a hook, a clock) caught ``exc``: a Ctrl+C is kept for the call that ran
    it, which raises it when it returns (:func:`_invoke`); anything else is the caller's to report."""
    if isinstance(exc, KeyboardInterrupt):
        _tls.callback_exc = exc
        _tls.callback_propagate = False


def _unraisable(unraisable: Any) -> None:
    """``sys.unraisablehook`` while the library is loaded. A Ctrl+C that arrives just as ctypes enters one of the
    callbacks below is raised before the callback's own ``try``: ctypes then drops it (and returns an undefined
    result to the core). Inside a call of this module that Ctrl+C is kept for the call (:func:`_interrupted`) instead;
    everything else goes to the hook installed before."""
    if (
        isinstance(unraisable.exc_value, KeyboardInterrupt)
        and getattr(_tls, "depth", 0)
        and "ctypes callback" in str(getattr(unraisable, "err_msg", "") or "")
    ):
        _interrupted(unraisable.exc_value)
        return
    _previous_unraisablehook(unraisable)


_previous_unraisablehook: Callable[[Any], Any] = sys.__unraisablehook__


def _install_unraisablehook() -> None:
    """Put :func:`_unraisable` in front of the hook in place (once, when the library is loaded)."""
    global _previous_unraisablehook
    if sys.unraisablehook is not _unraisable:
        _previous_unraisablehook = sys.unraisablehook
        sys.unraisablehook = _unraisable


class _PyTransport:
    """A Python transport object behind iad_transport_callbacks: ``write(bytes)``, ``read(n) -> bytes``, and when it
    has them ``reset_input_buffer()``, ``set_baudrate(rate)`` / ``baudrate``, ``send_break(seconds)``, ``present``,
    ``reopen(baudrate)``, ``close()``; a settable ``timeout`` gets the core's per-read timeout."""

    def __init__(self, target: Any) -> None:
        self.target = target
        self.timeout: float | None = None

    def read(self, n: int, timeout: float) -> bytes:
        if timeout != self.timeout:
            self.set_timeout(timeout)
        return bytes(self.target.read(n) or b"")

    def set_timeout(self, seconds: float) -> None:
        """The core's read timeout (0.5: before the request when the bus shortens it, else before a read)."""
        if hasattr(self.target, "timeout"):
            try:
                self.target.timeout = seconds
            except (AttributeError, TypeError):
                pass
        self.timeout = seconds


def _py_set_read_timeout(user: Any, seconds: float) -> int:
    try:
        _callback_object(user).set_timeout(float(seconds))
        return 0
    except BaseException as exc:
        _callback_failed(exc)
        return -1


def _callback_raised(exc: BaseException) -> int:
    """The result of a transport callback that raised ``exc``: an OSError is the port's
    failure (the cause of the call's TransportError, retried and a link fault as any), anything else (an assert in a
    test transport, a Ctrl+C) is what the call raises, and the core takes it for no failure of the port
    (IAD_CALLBACK_ABORT: no retry, no link fault)."""
    port = isinstance(exc, OSError)
    _callback_failed(exc, propagate=not port)
    return -1 if port else IAD_CALLBACK_ABORT


def _py_write(user: Any, data: Any, size: int) -> int:
    try:
        _callback_object(user).target.write(ctypes.string_at(data, size) if size else b"")
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_read(user: Any, buffer: Any, size: int, timeout: float, got: Any) -> int:
    try:
        got[0] = 0
        data = _callback_object(user).read(int(size), float(timeout))
        n = min(len(data), int(size))
        if n:
            ctypes.memmove(buffer, data, n)
        got[0] = n
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_reset_input(user: Any) -> int:
    try:
        target = _callback_object(user).target
        (getattr(target, "reset_input_buffer", None) or target.reset_input)()
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_set_baud_rate(user: Any, rate: int) -> int:
    try:
        _callback_object(user).target.set_baudrate(int(rate))
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_send_break(user: Any, seconds: float) -> int:
    try:
        _callback_object(user).target.send_break(float(seconds))
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_present(user: Any) -> int:
    try:
        present = _callback_object(user).target.present
        value = present() if callable(present) else present
        return IAD_UNKNOWN if value is None else int(bool(value))
    except BaseException as exc:
        _interrupted(exc)
        return IAD_UNKNOWN


def _py_reopen(user: Any, rate: int) -> int:
    try:
        target = _callback_object(user).target
        if rate:
            target.reopen(int(rate))
        else:
            target.reopen()
        return 0
    except BaseException as exc:
        return _callback_raised(exc)


def _py_close(user: Any) -> None:
    try:
        close = getattr(_callback_object(user).target, "close", None)
        if close is not None:
            close()
    except BaseException as exc:
        _interrupted(exc)
        if not isinstance(exc, KeyboardInterrupt):
            log.warning("iai_actuator_driver: closing a Python transport failed: %s", exc)


def _py_release(user: Any) -> None:
    with _callback_lock:
        _callback_objects.pop(int(user or 0), None)


def _py_clock_now(user: Any) -> float:
    try:
        return float(_callback_object(user)[0]())
    except BaseException as exc:
        _interrupted(exc)
        return time.monotonic()


def _py_clock_sleep(seconds: float, user: Any) -> None:
    try:
        _callback_object(user)[1](float(seconds))
    except BaseException as exc:
        _interrupted(exc)


def _py_frame(direction: int, data: Any, length: int, user: Any) -> None:
    try:
        _callback_object(user)(_FRAME_DIRECTIONS.get(int(direction), "rx"), ctypes.string_at(data, length))
    except BaseException as exc:  # a broken frame recorder must never break the bus (a Ctrl+C is kept for the call)
        _interrupted(exc)


def _py_request_hook(request: Any, size: int, reply: Any, capacity: int, length: Any, user: Any) -> int:
    try:
        length[0] = 0
        answer = _callback_object(user)(ctypes.string_at(request, size))
        if answer is NotImplemented:
            return IAD_SIM_PASS
        if answer is None:
            return IAD_SIM_DROP
        if isinstance(answer, int) and not isinstance(answer, bool):
            return sim_exception(answer)
        data = bytes(answer)[: int(capacity)]
        ctypes.memmove(reply, data, len(data))
        length[0] = len(data)
        return IAD_SIM_REPLY
    except BaseException as exc:  # (0.6) the request fails with it: an OSError becomes the cause
        # of the call's TransportError, anything else (an assert, a Ctrl+C) is what the call raises; (0.7) the core
        # then takes it for no failure of the line (IAD_SIM_ABORT: no retry, no link fault, the link state unchanged)
        port = isinstance(exc, OSError)
        _callback_failed(exc, propagate=not port)
        try:
            text = str(exc).encode("utf-8", errors="replace")[: int(capacity)]
            ctypes.memmove(reply, text, len(text))
            length[0] = len(text)
        except BaseException as late:
            _interrupted(late)
        return IAD_SIM_FAIL if port else IAD_SIM_ABORT


def _py_read_gate(user: Any) -> int:
    try:
        _callback_object(user)()
        return 0
    except BaseException as exc:  # ends the read; the read call raises this exception itself
        _callback_failed(exc, propagate=True)
        return 1


_c_py_write = TRANSPORT_WRITE(_py_write)
_c_py_read = TRANSPORT_READ(_py_read)
_c_py_reset_input = TRANSPORT_RESET_INPUT(_py_reset_input)
_c_py_set_baud_rate = TRANSPORT_SET_BAUD_RATE(_py_set_baud_rate)
_c_py_send_break = TRANSPORT_SEND_BREAK(_py_send_break)
_c_py_present = TRANSPORT_PRESENT(_py_present)
_c_py_reopen = TRANSPORT_REOPEN(_py_reopen)
_c_py_close = TRANSPORT_CLOSE(_py_close)
_c_py_release = TRANSPORT_RELEASE(_py_release)
_c_py_clock_now = CLOCK_NOW(_py_clock_now)
_c_py_clock_sleep = CLOCK_SLEEP(_py_clock_sleep)
_c_py_frame = FRAME_SINK(_py_frame)
_c_py_request_hook = SIM_REQUEST_HOOK(_py_request_hook)
_c_py_read_gate = READ_GATE(_py_read_gate)
_c_py_set_read_timeout = TRANSPORT_SET_READ_TIMEOUT(_py_set_read_timeout)


# ====================================================================== 0.3: transports, buses, simulators


def _int_list(values: Iterable[int] | None) -> bytes | None:
    return None if values is None else _utf8(",".join(str(int(v)) for v in values))


def _retries(retries: int | None) -> int:
    return IAD_INT_UNKNOWN if retries is None else int(retries)


def _tri_in(value: bool | None) -> int:
    return IAD_UNKNOWN if value is None else int(bool(value))


def _iai_flags(allow_unsafe: bool, allow_never_send: bool, check: bool) -> int:
    """The IAD_IAI_* flags of an IAI command's opt-ins (0.4)."""
    return ((IAD_IAI_ALLOW_UNSAFE if allow_unsafe else 0) | (IAD_IAI_ALLOW_NEVER_SEND if allow_never_send else 0)
            | (0 if check else IAD_IAI_NO_CHECK))  # fmt: skip


class NativeTransport:
    """A byte pipe on the native core (iad_transport, C++ ``Transport``): a serial port, a simulated unit or line, or
    a Python object (:meth:`from_object`: a test transport the core then drives). Pass it to :meth:`NativeBus.open`,
    :meth:`NativeAxis.open_transport` or :meth:`NativeCell.open_transports`; those keep it, so this handle can be
    released (:meth:`release`, or when the object is collected). :meth:`close` closes the transport itself."""

    def __init__(self, lib: _Library, handle: int, name: str = "") -> None:
        self._lib = lib
        self._handle: int | None = handle
        self.name = name
        self._finalizer = weakref.finalize(self, _release, lib, "iad_transport_release", handle)

    @classmethod
    def from_object(cls, transport: Any, *, name: str | None = None, real_timeout: bool = True) -> NativeTransport:
        """The core drives ``transport`` (``write(bytes)``, ``read(n) -> bytes``; optional ``reset_input_buffer()``,
        ``set_baudrate(rate)`` with ``baudrate``, ``send_break(seconds)``, ``present``, ``reopen(baudrate)``,
        ``close()``). Its methods run on the thread doing the bus I/O, with the bus lock held: they must not call the
        driver. An exception in them becomes a :class:`~iai_actuator_driver.modbus.TransportError` whose cause it is.
        ``real_timeout`` False: ``read`` answers at once (in memory) and the bus polls for its own deadline."""
        lib = load()
        cb = TransportCallbacks()
        cb.struct_size = ctypes.sizeof(cb)  # 0.5: the whole struct (set_read_timeout)
        _check(lib, "iad_transport_callbacks_init", ctypes.byref(cb))
        timeout = getattr(transport, "timeout", None)
        if isinstance(timeout, (int, float)) and not isinstance(timeout, bool) and timeout >= 0:
            cb.read_timeout_s = float(timeout)
        cb.real_timeout = int(bool(real_timeout))
        cb.write, cb.read, cb.release = _c_py_write, _c_py_read, _c_py_release
        if hasattr(transport, "reset_input_buffer") or hasattr(transport, "reset_input"):
            cb.reset_input = _c_py_reset_input
        if callable(getattr(transport, "set_baudrate", None)):
            cb.set_baud_rate = _c_py_set_baud_rate
            rate = getattr(transport, "baudrate", None)
            cb.baud_rate = int(rate) if isinstance(rate, int) and not isinstance(rate, bool) and rate > 0 else 0
        if callable(getattr(transport, "send_break", None)):
            cb.send_break = _c_py_send_break
        if hasattr(transport, "present"):
            cb.present = _c_py_present
        if callable(getattr(transport, "reopen", None)):
            cb.reopen = _c_py_reopen
        if callable(getattr(transport, "close", None)):
            cb.close = _c_py_close
        if hasattr(transport, "timeout"):
            cb.set_read_timeout = _c_py_set_read_timeout
        label = name or str(getattr(transport, "port", None) or getattr(transport, "name", None) or "")
        key = _keep_callback(_PyTransport(transport))
        out = ctypes.c_void_p()
        try:
            _check(lib, "iad_transport_open_callbacks", ctypes.byref(cb), _utf8(label or None), key, ctypes.byref(out))
        except BaseException:
            _py_release(key)  # nothing was taken
            raise
        return cls(lib, int(out.value or 0), label or "CALLBACK")

    @classmethod
    def serial(cls, port: str, *, baudrate: int | None = None, timeout: float | None = None) -> NativeTransport:
        """Open serial port ``port`` (default 115200 baud, a 0.2 s read timeout)."""
        lib = load()
        out = ctypes.c_void_p()
        _check(lib, "iad_transport_open_serial", _utf8(port), int(baudrate or 0), _num(timeout), ctypes.byref(out))
        return cls(lib, int(out.value or 0), port)

    def _h(self) -> int:
        if self._handle is None:
            raise ValueError(f"{self!r}: the transport handle is released")
        return self._handle

    def _call(self, name: str, *args: Any) -> None:
        _check(self._lib, name, self._h(), *args)

    def __repr__(self) -> str:
        return f"NativeTransport({self.name!r}, {'released' if self._handle is None else 'open'})"

    def __enter__(self) -> NativeTransport:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def release(self) -> None:
        """Release this handle only: buses, axes and cells using the transport keep it. Safe to call twice."""
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_transport_release", handle)

    def close(self) -> None:
        """Close the transport (what uses it then fails until it is reopened) and release this handle."""
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_transport_close", handle)

    def info(self) -> dict[str, Any]:
        """name, open, baud_rate (None: none), read_timeout_s, real_timeout, present (None: unknown), can_reopen."""
        return _read_json(self._lib, "iad_transport_info_json", self._h())

    def write(self, data: bytes) -> None:
        raw = bytes(data)
        self._call("iad_transport_write", (ctypes.c_uint8 * max(len(raw), 1)).from_buffer_copy(raw or b"\0"),
                   len(raw))

    def read(self, n: int) -> bytes:
        buffer = (ctypes.c_uint8 * max(int(n), 1))()
        got = ctypes.c_size_t(0)
        self._call("iad_transport_read", buffer, int(n), ctypes.byref(got))
        return bytes(buffer[: got.value])

    def reset_input_buffer(self) -> None:
        self._call("iad_transport_reset_input")

    def set_baudrate(self, rate: int) -> None:
        self._call("iad_transport_set_baud_rate", int(rate))

    def set_timeout(self, seconds: float) -> None:
        self._call("iad_transport_set_read_timeout", float(seconds))

    def send_break(self, duration: float) -> None:
        self._call("iad_transport_send_break", float(duration))

    def reopen(self, baudrate: int | None = None) -> None:
        self._call("iad_transport_reopen", int(baudrate or 0))


def _as_transport(target: Any) -> tuple[NativeTransport, bool]:
    """(a transport handle, whether it was made here and can be released after use) for a NativeTransport, a
    NativeSim / NativeSimLine or a Python transport object."""
    if isinstance(target, NativeTransport):
        return target, False
    if isinstance(target, (NativeSim, NativeSimLine)):
        return target.transport(), True
    return NativeTransport.from_object(target), True


class NativeBus:
    """A Modbus RTU master on the native core (iad_bus, C++ ``ModbusRtu``): one transaction at a time, retries, the
    frame sink, raw requests. :meth:`open` makes one on a transport (closing it closes the transport); an axis's or a
    cell port's bus (:meth:`NativeAxis.bus`, :meth:`NativeCell.bus`) is borrowed (closing it releases the handle
    only). Requests take ``slave`` 1..247; ``retries`` ``None`` = the bus's."""

    def __init__(self, lib: _Library, handle: int, *, name: str = "", owned: bool = True) -> None:
        self._lib = lib
        self._handle: int | None = handle
        self.name = name
        self.owned = owned
        self._finalizer = weakref.finalize(self, _release, lib, "iad_bus_close", handle)

    @classmethod
    def open(cls, transport: Any, *, retries: int | None = None, min_gap: float | None = None) -> NativeBus:
        """A bus on ``transport`` (a :class:`NativeTransport`, a :class:`NativeSim` / :class:`NativeSimLine` or a
        Python transport object). No I/O."""
        lib = load()
        handle, made = _as_transport(transport)
        o = BusOptions()
        _check(lib, "iad_bus_options_init", ctypes.byref(o))
        if retries is not None:
            o.retries = int(retries)
        if min_gap is not None:
            o.min_gap_s = float(min_gap)
        out = ctypes.c_void_p()
        try:
            _check(lib, "iad_bus_open", handle._h(), ctypes.byref(o), ctypes.byref(out))
        finally:
            if made:
                handle.release()
        return cls(lib, int(out.value or 0), name=handle.name)

    def _h(self) -> int:
        if self._handle is None:
            raise ValueError(f"{self!r}: the bus handle is closed")
        return self._handle

    def _call(self, name: str, *args: Any) -> None:
        _check(self._lib, name, self._h(), *args)

    def __repr__(self) -> str:
        return f"NativeBus({self.name!r}, {'closed' if self._handle is None else 'open'})"

    def __enter__(self) -> NativeBus:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        """An owned bus: close it and its transport; a borrowed one: release the handle. Safe to call twice."""
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_bus_close", handle)

    @property
    def closed(self) -> bool:
        return self._handle is None or bool(self.info().get("closed"))

    def info(self) -> dict[str, Any]:
        """name, retries, min_gap_s, baud_rate (None: none), can_change_baud_rate, closed, line,
        unsupported_functions ({slave: [function codes answered with exception 01]})."""
        return _read_json(self._lib, "iad_bus_info_json", self._h())

    @property
    def retries(self) -> int:
        return int(self.info()["retries"])

    @retries.setter
    def retries(self, value: int) -> None:
        self._call("iad_bus_set_retries", int(value))

    @property
    def min_gap(self) -> float:
        return float(self.info()["min_gap_s"])

    @min_gap.setter
    def min_gap(self, seconds: float) -> None:
        self._call("iad_bus_set_min_gap", float(seconds))

    def set_frame_sink(self, sink: Callable[[str, bytes], Any] | None) -> None:
        """Every request (``"tx"``), response (``"rx"``) and partial response (``"rx-partial"``) of the bus to
        ``sink(direction, frame)`` (``None`` removes it). It runs on the thread doing the I/O, with the bus lock held:
        return quickly, never call the driver from it; an exception in it is ignored."""
        if sink is None:
            self._call("iad_bus_set_frame_sink", FRAME_SINK(), None)
            return
        self._call("iad_bus_set_frame_sink", _c_py_frame, _keep_callback(sink))

    def transport(self) -> NativeTransport:
        """A new handle on the bus's transport."""
        out = ctypes.c_void_p()
        self._call("iad_bus_transport", ctypes.byref(out))
        return NativeTransport(self._lib, int(out.value or 0), self.name)

    def read_registers(
        self, slave: int, address: int, count: int, *, retries: int | None = None, max_wait: float | None = None,
        chunk: int | None = None,
    ) -> list[int]:
        """FC03 (READ-ONLY). ``max_wait`` caps the wait per attempt, ``chunk`` the registers per frame (default 60)."""
        values = (ctypes.c_uint16 * max(int(count), 1))()
        self._call("iad_bus_read_registers", int(slave), int(address), int(count), _retries(retries),
                   _num(max_wait), IAD_INT_UNKNOWN if chunk is None else int(chunk), values)
        return list(values[: int(count)])

    def write_coil(self, slave: int, address: int, on: bool, *, retries: int | None = None) -> None:
        self._call("iad_bus_write_coil", int(slave), int(address), int(bool(on)), _retries(retries))

    def write_register(self, slave: int, address: int, value: int) -> None:
        self._call("iad_bus_write_register", int(slave), int(address), int(value))

    def write_registers(self, slave: int, address: int, values: Iterable[int]) -> None:
        words = [int(v) for v in values]
        self._call("iad_bus_write_registers", int(slave), int(address), (ctypes.c_uint16 * max(len(words), 1))(*words),
                   len(words))

    def read_write_registers(
        self, slave: int, read_address: int, read_count: int, write_address: int, values: Iterable[int]
    ) -> list[int]:
        """FC17: write ``values`` at ``write_address``, then read ``read_count`` registers at ``read_address``."""
        words = [int(v) for v in values]
        out = (ctypes.c_uint16 * max(int(read_count), 1))()
        self._call("iad_bus_read_write_registers", int(slave), int(read_address), int(read_count),
                   int(write_address), (ctypes.c_uint16 * max(len(words), 1))(*words), len(words), out)
        return list(out[: int(read_count)])

    def report_slave_id(self, slave: int, *, retries: int | None = None, timeout: float | None = 0.65) -> bytes:
        """FC11: the data bytes after the byte count (no retries by default; READ-ONLY). Each attempt waits at least
        ``timeout`` for the answer (``None``: the transport's timeout only)."""
        buffer = (ctypes.c_uint8 * 256)()
        length = ctypes.c_size_t(0)
        self._call("iad_bus_report_slave_id", int(slave), int(retries or 0), _num(timeout), buffer, 256,
                   ctypes.byref(length))
        return bytes(buffer[: length.value])

    def iai_command(
        self, slave: int, group: int, command: int, op1: int = 0, op2: int = 0, *, allow_unsafe: bool = False,
        allow_never_send: bool = False, check: bool = True, retries: int | None = None,
    ) -> tuple[int, int]:
        """IAI 0x45: (rejection, status). A command not on the safe list (or to another ID group) needs
        ``allow_unsafe``, one on the never-send list ``allow_never_send``."""
        rejection, status = ctypes.c_int32(0), ctypes.c_uint32(0)
        ops = (ctypes.c_uint32 * 2)(int(op1), int(op2))  # 16 bits each: the core checks
        self._call("iad_bus_iai_command", int(slave), 0x45, int(group), int(command), ops, 2,
                   _iai_flags(allow_unsafe, allow_never_send, check), _retries(retries), ctypes.byref(rejection),
                   ctypes.byref(status), 1)
        return rejection.value, status.value

    def iai_command2(
        self, slave: int, command: int, operands: Iterable[int] = (), *, allow_unsafe: bool = False,
        allow_never_send: bool = False, check: bool = True, group: int = 0, retries: int | None = None,
    ) -> tuple[int, tuple[int, int, int, int]]:
        """IAI 0x48 "ControlCommand2" (0.4; UNVERIFIED on any unit): (rejection, the four u32 status words). Up to 4
        u32 operands; the 16-bit command must be on the 0x48 safe list and ``group`` 0 unless ``allow_unsafe``."""
        ops = [int(v) for v in operands]
        rejection, status = ctypes.c_int32(0), (ctypes.c_uint32 * 4)()
        self._call("iad_bus_iai_command", int(slave), 0x48, int(group), int(command),
                   (ctypes.c_uint32 * max(len(ops), 1))(*ops) if ops else None, len(ops),
                   _iai_flags(allow_unsafe, allow_never_send, check), _retries(retries), ctypes.byref(rejection),
                   status, 4)
        return rejection.value, (status[0], status[1], status[2], status[3])

    def broadcast_iai_command(
        self, group: int, command: int, op1: int = 0, op2: int = 0, *, allow_unsafe: bool = False,
        allow_never_send: bool = False, settle_s: float = 0.2,
    ) -> bytes:
        """IAI 0x45 to the broadcast address 0 (0.4): every unit on the line acts, none answers; the bus is held for
        ``settle_s`` after the frame. The same opt-ins as :meth:`iai_command`. Returns the frame sent."""
        frame, length = (ctypes.c_uint8 * 16)(), ctypes.c_size_t(0)
        self._call("iad_bus_broadcast_iai_command", int(group), int(command), int(op1), int(op2),
                   _iai_flags(allow_unsafe, allow_never_send, True), float(settle_s), frame, 16, ctypes.byref(length))
        return bytes(frame[: length.value])

    def broadcast(self, pdu: bytes, *, settle_s: float = 0.02) -> bytes:
        """An FC05 / FC06 / FC10 PDU (function code first) to the broadcast address 0 (0.4); never a read. Returns the
        frame sent."""
        data = bytes(pdu)
        size = len(data) + 3
        frame, length = (ctypes.c_uint8 * size)(), ctypes.c_size_t(0)
        self._call("iad_bus_broadcast", (ctypes.c_uint8 * max(len(data), 1)).from_buffer_copy(data.ljust(1, b"\0")),
                   len(data), float(settle_s), frame, size, ctypes.byref(length))
        return bytes(frame[: length.value])

    def broadcast_coil(self, address: int, on: bool, *, settle_s: float = 0.02) -> bytes:
        """FC05 to every unit on the line (address 0), e.g. STOP ``00 05 04 2C FF 00 4D 12``. Returns the frame
        sent."""
        if not 0 <= int(address) <= 0xFFFF:
            raise ValueError(f"address must be 0..0xFFFF, got {address!r}")
        return self.broadcast(struct.pack(">BHH", 0x05, int(address), 0xFF00 if on else 0x0000), settle_s=settle_s)

    # ---- 0.4: the bus lock, preemptible reads, status monitors

    def lock(self, timeout: float | None = None) -> bool:
        """Hold the bus lock for the calling thread (iad_bus_lock): no request of another thread goes out until
        :meth:`unlock`; this thread's own requests go through (recursive: each ``True`` needs one :meth:`unlock`).
        ``timeout`` ``None`` waits as long as it takes (in short slices on the main thread, so Ctrl+C reaches Python),
        0 takes a free bus only; ``False`` when the time ran out. The hold belongs to the calling OS thread."""
        locked = _c_bool(0)
        if timeout is None and threading.current_thread() is threading.main_thread():
            while not locked.value:
                self._call("iad_bus_lock", _SLICE_S, ctypes.byref(locked))
            return True
        self._call("iad_bus_lock", _num(timeout), ctypes.byref(locked))
        return bool(locked.value)

    def unlock(self) -> None:
        """Give one hold of :meth:`lock` back (ValueError when this thread holds none)."""
        self._call("iad_bus_unlock")

    @contextlib.contextmanager
    def exclusive(self, timeout: float | None = None) -> Iterator[NativeBus]:
        """Hold the bus lock for a block (the package's ``ModbusRTU.exclusive()``): no request of another thread
        (a status monitor, another axis on the line, a STOP) goes out until the block ends; keep it short.
        ``TimeoutError`` when the bus was not free within ``timeout``."""
        if not self.lock(timeout):
            raise TimeoutError(f"{self.name}: the bus lock was not free within {timeout} s")
        try:
            yield self
        finally:
            self.unlock()

    @contextlib.contextmanager
    def preemptible(self, gate: Callable[[], Any] | None = None, *, words: int = 32) -> Iterator[NativeBus]:
        """Preemptible read-only reads of the calling thread on this bus for a block (iad_bus_preemptible_begin; the
        GUI's ``_GatedBus``): FC03 reads go out in frames of at most ``words`` registers, every attempt takes the bus on
        its own after ``gate()`` (outside the bus lock; raising ends the read and the read call raises that
        exception), and this thread's writes, IAI commands and broadcasts on the bus are refused
        (``UnsafeCommandError``). A STOP of another thread so waits for one short request at most."""
        key = _keep_callback(gate) if gate is not None else 0
        try:
            handle = self._h()
            _check(self._lib, "iad_bus_preemptible_begin", handle, int(words),
                   _c_py_read_gate if gate is not None else READ_GATE(), key or None)
            try:
                yield self
            finally:
                _check(self._lib, "iad_bus_preemptible_end", handle)  # also after this handle was closed
        finally:
            if key:
                with _callback_lock:
                    _callback_objects.pop(key, None)

    @contextlib.contextmanager
    def suspend_monitors(self, timeout: float = 1.0) -> Iterator[int]:
        """Suspend every running status monitor of an axis on this bus for a block (iad_bus_suspend_monitors; the
        package's ``monitor.suspend_bus``): no polls inside it (a poll in flight is waited for, at most
        ``timeout``). Yields how many were suspended."""
        count = ctypes.c_int32(0)
        self._call("iad_bus_suspend_monitors", float(timeout), ctypes.byref(count))
        try:
            yield count.value
        finally:
            if self._handle is not None:  # closing the handle resumed them
                self._call("iad_bus_resume_monitors")

    @property
    def lock_depth(self) -> int:
        """The calling thread's holds of the bus lock."""
        return int(self.info().get("lock_depth", 0))

    # ---- 0.4: the line, response timing

    def set_line(self, kind: str, *, position_unit_mm: float | None = None) -> None:
        """State the line ("direct", "rcon", "robonet", "robonet-gateway"; not probed, no I/O; the package's
        ``bus.line = Line(kind)``). Every axis on the bus follows it at once."""
        code = _choice(kind, _LINES, "line")
        if code == _LINES["auto"]:
            raise ValueError("line 'auto' probes the line: use detect_line()")
        self._call("iad_bus_set_line", code, _num(position_unit_mm))

    def detect_line(
        self, kind: str = "auto", *, timeout: float | None = None, position_unit_mm: float | None = None
    ) -> str:
        """Work out the line and attach it (READ-ONLY FC03 probes for an RCON gateway at D0h and a ROBONET gateway at
        3Fh; a stated kind probes its gateway only). Returns the kind attached."""
        found = ctypes.c_int32(0)
        self._call("iad_bus_detect_line", _choice(kind, _LINES, "line"), _num(timeout), _num(position_unit_mm),
                   ctypes.byref(found))
        return _name_of(found.value, _LINES, "direct") or "direct"

    @property
    def line_kind(self) -> str:
        """"direct", "rcon", "robonet" or "robonet-gateway"."""
        return str(self.info().get("line") or "direct")

    def line_info(self) -> dict[str, Any]:
        """The line as a dict (kind, detected, rules, the gateway's last status, position units, unreachable slaves,
        explanation); no I/O."""
        return _read_json(self._lib, "iad_bus_line_json", self._h())

    def unsupported_functions(self, slave: int) -> frozenset[int]:
        """Function codes ``slave`` answered with exception 01 (Illegal Function) so far on this bus."""
        return frozenset(int(c) for c in (self.info().get("unsupported_functions") or {}).get(str(int(slave)), ()))

    def set_response_delay(self, slave: int, delay_ms: float) -> None:
        """The unit's response delay RTIM (EC item 8.4 / CON No.17, 0..1000 ms) for :meth:`response_timeout`."""
        self._call("iad_bus_set_response_delay", int(slave), float(delay_ms))

    def response_delay_ms(self, slave: int) -> float:
        out = ctypes.c_double(0.0)
        self._call("iad_bus_response_delay", int(slave), ctypes.byref(out))
        return out.value

    def response_timeout(self, slave: int, pdu: bytes, expected_len: int | None = None) -> float:
        """How long the bus waits for the answer to ``pdu`` (seconds; IAI's formula with the 0.2 s, slow-rate and line
        floors). ``expected_len``: the whole response frame (``None``: 64 bytes assumed)."""
        data = bytes(pdu)
        out = ctypes.c_double(0.0)
        self._call("iad_bus_response_timeout", int(slave),
                   (ctypes.c_uint8 * max(len(data), 1)).from_buffer_copy(data.ljust(1, b"\0")), len(data),
                   int(expected_len or 0), ctypes.byref(out))
        return out.value

    def probe(self, slave: int, timeout: float | None = None, address: int = 0x9000) -> bool:
        """One FC03 of one register, no retries: did a valid frame come (an exception response counts)?"""
        answered = _c_bool(0)
        self._call("iad_bus_probe", int(slave), _num(timeout), int(address), ctypes.byref(answered))
        return bool(answered.value)

    def detect_baudrate(
        self, slaves: Iterable[int | tuple[int, int]] = (1,), candidates: Iterable[int] | None = None, *,
        probe_address: int = 0x9000, current_first: bool = True, probe_timeout: float | None = None,
    ) -> int:
        """The rate one of ``slaves`` answers at; the transport is left there. An entry ``(slave, address)`` probes that
        register instead of ``probe_address`` (a gateway probe)."""
        targets = ",".join(f"{int(e[0])}@{int(e[1])}" if isinstance(e, tuple) else str(int(e)) for e in slaves)
        rate = ctypes.c_int32(0)
        self._call("iad_bus_detect_baud_rate", _utf8(targets), _int_list(candidates), int(probe_address),
                   int(bool(current_first)), _num(probe_timeout), ctypes.byref(rate))
        return rate.value

    def set_baudrate(self, rate: int) -> None:
        """The PC side's rate in place (the units keep theirs)."""
        self._call("iad_bus_set_baud_rate", int(rate))

    def send_break(self, duration: float) -> None:
        """A line BREAK (0 < duration <= 2 s; 150 ms or more drops a unit to 9600 baud)."""
        self._call("iad_bus_send_break", float(duration))

    def reopen(self, baudrate: int | None = None) -> None:
        self._call("iad_bus_reopen", int(baudrate or 0))


def _sim_json(options: Mapping[str, Any]) -> str:
    out: dict[str, Any] = {}
    for key, value in options.items():
        if value is None:
            continue
        if key == "word_overrides" and isinstance(value, Mapping):
            value = {str(int(a)): int(v) for a, v in value.items()}
        elif key in ("unreadable",):
            value = [[int(a), int(b)] for a, b in value]
        elif key in ("firmware", "ignored_baud_codes"):
            value = [int(v) for v in value]
        out[key] = value
    return json.dumps(out)


class NativeSim:
    """A simulated unit on the native core with the test hooks of :class:`~iai_actuator_driver.sim.SimulatedEC`
    (iad_sim, C++ ``HookedSimUnit``). Use it through :meth:`NativeBus.open` / :meth:`NativeAxis.open_transport` (or
    :meth:`transport`) or on a :class:`NativeSimLine`. Never opens a port."""

    def __init__(self, lib: _Library, handle: int) -> None:
        self._lib = lib
        self._handle: int | None = handle
        self._finalizer = weakref.finalize(self, _release, lib, "iad_sim_close", handle)

    @classmethod
    def open(cls, profile: str = "slider", **options: Any) -> NativeSim:
        """A unit of ``profile`` ("slider", "gripper", "rod", "pcon", "rotary", "unknown"). ``options`` are the C++
        ``SimulatedUnitOptions`` names (``slave``, ``time_scale``, ``position_mm``, ``stroke_mm``, ``baud_rate``,
        ``unit_baud_rate``, ``serial_number``...) and the hooks: ``word_overrides`` ({address: value}),
        ``unreadable`` ([(start, count)]), ``stale_status_polls``, ``step_s``. ``None`` values are left out."""
        lib = load()
        out = ctypes.c_void_p()
        _check(lib, "iad_sim_open", _utf8(_sim_json({"profile": profile, **options})), ctypes.byref(out))
        return cls(lib, int(out.value or 0))

    def _h(self) -> int:
        if self._handle is None:
            raise ValueError(f"{self!r}: the simulator handle is closed")
        return self._handle

    def _call(self, name: str, *args: Any) -> None:
        _check(self._lib, name, self._h(), *args)

    def __repr__(self) -> str:
        return f"NativeSim({'closed' if self._handle is None else 'open'})"

    def __enter__(self) -> NativeSim:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        """Release the handle (the unit lives on while a transport, bus, axis or line uses it)."""
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_sim_close", handle)

    def transport(self) -> NativeTransport:
        """A transport handle on the unit (the simulated port)."""
        out = ctypes.c_void_p()
        self._call("iad_sim_transport", ctypes.byref(out))
        return NativeTransport(self._lib, int(out.value or 0), "SIM")

    def state(self) -> dict[str, Any]:
        """The unit's state, counters and hooks (iad_sim_state_json; no I/O)."""
        return _read_json(self._lib, "iad_sim_state_json", self._h())

    def requests(self) -> list[tuple[float, bytes]]:
        """(time on the unit's clock, frame) of every frame written to the unit, oldest first (the last 2000)."""
        return [(float(r["t"]), bytes.fromhex(r["frame"])) for r in _read_json(self._lib, "iad_sim_requests_json",
                                                                               self._h())]

    def events(self) -> list[str]:
        return list(_read_json(self._lib, "iad_sim_events_json", self._h()))

    def power_cycle(self, *, lose_home: bool | None = None) -> None:
        self._call("iad_sim_power_cycle", _tri_in(lose_home))

    def power_off(self) -> None:
        self._call("iad_sim_power_off")

    def power_on(self, *, lose_home: bool | None = None) -> None:
        self._call("iad_sim_power_on", _tri_in(lose_home))

    def unplug(self) -> None:
        self._call("iad_sim_unplug")

    def replug(self) -> None:
        self._call("iad_sim_replug")

    def set_word_override(self, address: int, value: int | None) -> None:
        """Serve ``value`` for register ``address`` (``None``: model it again)."""
        self._call("iad_sim_set_word_override", int(address), IAD_INT_UNKNOWN if value is None else int(value))

    def set_unreadable(self, start: int, count: int = 1, unreadable: bool = True) -> None:
        self._call("iad_sim_set_unreadable", int(start), int(count), int(bool(unreadable)))

    def set_stale_status_polls(self, polls: int) -> None:
        self._call("iad_sim_set_stale_status_polls", int(polls))

    def register_value(self, address: int) -> int:
        """What a one-register read returns now (the hooks applied); an error where a read is refused."""
        value = ctypes.c_uint16(0)
        self._call("iad_sim_register_value", int(address), ctypes.byref(value))
        return value.value

    def handle(self, frame: bytes) -> bytes | None:
        """The unit's own answer to ``frame`` (it acts on it; ``None``: no answer). Usable from a request hook."""
        raw = bytes(frame)
        buffer = (ctypes.c_uint8 * 256)()
        length = ctypes.c_size_t(0)
        self._call("iad_sim_handle_frame", (ctypes.c_uint8 * max(len(raw), 1)).from_buffer_copy(raw or b"\0"),
                   len(raw), buffer, 256, ctypes.byref(length))
        return bytes(buffer[: length.value]) if length.value else None

    def set_request_hook(self, hook: Callable[[bytes], Any] | None) -> None:
        """``hook(frame)`` sees every frame written to the unit and returns ``NotImplemented`` (answer as usual),
        ``None`` (no answer), bytes (this answer) or an int (that Modbus exception code); :meth:`handle` is the unit's
        own answer (act, then drop or replace it). It runs on the thread doing the bus I/O: no other driver call from
        it. ``None`` removes it."""
        if hook is None:
            self._call("iad_sim_set_request_hook", SIM_REQUEST_HOOK(), None)
            return
        self._call("iad_sim_set_request_hook", _c_py_request_hook, _keep_callback(hook))

    def control(self, command: str, **arguments: Any) -> dict[str, Any]:
        """One of the other controls (iad_sim_control_json): inject_fault, trigger_alarm, set_obstacle, pio_move,
        pio_release, coil_state, set_coil_state, position_row, set_position_row, clear_requests, clear_events,
        set_step_s."""
        return _read_json(self._lib, "iad_sim_control_json", self._h(), _utf8(json.dumps({"command": command,
                                                                                         **arguments})))

    def inject_fault(self, kind: str, count: int = 1) -> None:
        self.control("inject_fault", kind=kind, count=count)

    def trigger_alarm(self, code: int) -> None:
        self.control("trigger_alarm", code=code)

    def coil_state(self, address: int) -> bool:
        return bool(self.control("coil_state", address=address)["on"])

    def set_coil_state(self, address: int, on: bool) -> None:
        self.control("set_coil_state", address=address, on=bool(on))

    def position_row(self, row: int) -> list[int]:
        return [int(w) for w in self.control("position_row", row=row)["words"]]

    def set_position_row(self, row: int, words: Iterable[int]) -> None:
        self.control("set_position_row", row=row, words=[int(w) for w in words])

    def clear_requests(self) -> None:
        self.control("clear_requests")


class NativeSimLine:
    """Several simulated units on one line by slave address (iad_sim_line, C++ ``SimLine``)."""

    def __init__(self, lib: _Library, handle: int, name: str) -> None:
        self._lib = lib
        self._handle: int | None = handle
        self.name = name
        self._finalizer = weakref.finalize(self, _release, lib, "iad_sim_line_close", handle)

    @classmethod
    def open(cls, units: Iterable[NativeSim] = (), name: str = "SIM") -> NativeSimLine:
        lib = load()
        listed = list(units)
        handles = (ctypes.c_void_p * max(len(listed), 1))(*(u._h() for u in listed))
        out = ctypes.c_void_p()
        _check(lib, "iad_sim_line_open", _utf8(name), handles, len(listed), ctypes.byref(out))
        return cls(lib, int(out.value or 0), name)

    def _h(self) -> int:
        if self._handle is None:
            raise ValueError(f"{self!r}: the line handle is closed")
        return self._handle

    def __repr__(self) -> str:
        return f"NativeSimLine({self.name!r}, {'closed' if self._handle is None else 'open'})"

    def __enter__(self) -> NativeSimLine:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            self._finalizer.detach()
            _check(self._lib, "iad_sim_line_close", handle)

    def add(self, unit: NativeSim) -> None:
        _check(self._lib, "iad_sim_line_add", self._h(), unit._h())

    def remove(self, slave: int) -> None:
        _check(self._lib, "iad_sim_line_remove", self._h(), int(slave))

    def transport(self) -> NativeTransport:
        out = ctypes.c_void_p()
        _check(self._lib, "iad_sim_line_transport", self._h(), ctypes.byref(out))
        return NativeTransport(self._lib, int(out.value or 0), self.name)

    def requests(self) -> list[tuple[float, int | None, bytes]]:
        """(time, slave, frame) of every request on the line, oldest first."""
        return [(float(r["t"]), r["slave"], bytes.fromhex(r["frame"]))
                for r in _read_json(self._lib, "iad_sim_line_requests_json", self._h())]

    def control(self, command: str, **arguments: Any) -> dict[str, Any]:
        """unplug, replug, power_cycle (slaves, lose_home), clear_requests, state."""
        return _read_json(self._lib, "iad_sim_line_control_json", self._h(),
                          _utf8(json.dumps({"command": command, **arguments})))

    def unplug(self) -> None:
        self.control("unplug")

    def replug(self) -> None:
        self.control("replug")

    def power_cycle(self, slaves: Iterable[int] | None = None, *, lose_home: bool | None = None) -> None:
        arguments: dict[str, Any] = {"lose_home": lose_home}
        if slaves is not None:
            arguments["slaves"] = [int(s) for s in slaves]
        self.control("power_cycle", **arguments)

    def state(self) -> dict[str, Any]:
        return self.control("state")


# ====================================================================== 0.3: clock seam, log level


def clock_now() -> float:
    """The core's clock now (seconds, arbitrary origin)."""
    value = ctypes.c_double(0.0)
    _check(load(), "iad_clock_now", ctypes.byref(value))
    return value.value


def use_real_clock() -> None:
    """The core on the real (steady) clock again."""
    _check(load(), "iad_clock_use_real")


def use_virtual_clock(start: float = 1000.0) -> None:
    """The core on a virtual clock starting at ``start``: it only moves when slept on or advanced, so a timeout costs
    no real time. Process-wide; for single-threaded tests."""
    _check(load(), "iad_clock_use_virtual", float(start))


def advance_clock(seconds: float) -> None:
    """Move the virtual clock forward (an error when it is not installed)."""
    _check(load(), "iad_clock_advance", float(seconds))


def use_clock(now: Callable[[], float], sleep: Callable[[float], None]) -> None:
    """The core on Python's own clock functions (they run on whatever thread the core waits on)."""
    _check(load(), "iad_clock_use_callbacks", _c_py_clock_now, _c_py_clock_sleep, _keep_callback((now, sleep)))


class NativeVirtualClock:
    """The core's virtual clock as an object with the shape of ``testing.VirtualClock``."""

    @property
    def now(self) -> float:
        return clock_now()

    def perf_counter(self) -> float:
        return clock_now()

    monotonic = perf_counter

    def sleep(self, seconds: float) -> None:
        advance_clock(max(0.0, float(seconds)))

    def advance(self, seconds: float) -> None:
        advance_clock(float(seconds))


@contextlib.contextmanager
def virtual_clock(start: float = 1000.0) -> Iterator[NativeVirtualClock]:
    """Run the core on a new virtual clock for the block (the real clock again afterwards). Not reentrant."""
    use_virtual_clock(start)
    try:
        yield NativeVirtualClock()
    finally:
        use_real_clock()


def set_log_level(level: int | str) -> None:
    """Lines below ``level`` are not passed to the log sink (a :mod:`logging` level or "debug" .. "error"). From then
    on the level is the caller's: the logging bridge no longer follows the :mod:`logging` levels."""
    global _log_level_explicit
    if isinstance(level, str):
        try:
            code = _LOG_LEVEL_NAMES[level.lower()]
        except KeyError:
            raise ValueError(f"unknown log level {level!r}; choose from {', '.join(_LOG_LEVEL_NAMES)}") from None
    else:
        code = _log_level_code(int(level))
    with _log_level_lock:
        _log_level_explicit = True
        _check(load(), "iad_set_log_level", code)


def _log_level_code(level: int) -> int:
    """The IAD_LOG_* code of a :mod:`logging` level: the lowest core level whose lines that level lets through."""
    return 0 if level <= logging.DEBUG else 1 if level <= logging.INFO else 2 if level <= logging.WARNING else 3


# ====================================================================== library-wide: log sink, multi-axis backup

_log_sink: Any = None  # the LOG_SINK object the core holds (kept alive here)
# The sink set_log_sink installed (None: none) and how often one was installed; whether set_log_level chose the core's
# level (the logging bridge then leaves it alone). _log_level_lock orders the level changes.
_log_sink_function: Callable[[int, str, str], Any] | None = None
_log_sink_generation = 0
_log_level_explicit = False
_log_level_lock = threading.RLock()


def set_log_sink(sink: Callable[[int, str, str], Any] | None) -> None:
    """Install (or remove, ``None``) the process-wide log sink of the core: ``sink(level, source, message)`` with
    ``level`` a :mod:`logging` level (DEBUG .. ERROR) and ``source`` such as ``"iai_actuator_driver.axis"``. It runs on
    the thread that logs (a cell worker, a monitor thread): copy what you need, return quickly, never call the driver
    from it. Replaces an earlier sink. It gets every line unless :func:`set_log_level` says otherwise."""
    global _log_sink, _log_sink_function, _log_sink_generation
    lib = load()
    with _log_level_lock:
        if sink is None:
            _check(lib, "iad_set_log_sink", LOG_SINK(), None)
            _log_sink = None
        else:

            def forward(level: int, source: bytes | None, message: bytes | None, _user: Any) -> None:
                try:
                    sink(_LOG_LEVELS.get(int(level), logging.ERROR), _text(source), _text(message))
                except Exception:  # a broken sink must never break the core
                    pass

            c_sink = LOG_SINK(forward)
            _check(lib, "iad_set_log_sink", c_sink, None)
            _log_sink = c_sink
        _log_sink_function = sink
        _log_sink_generation += 1
        if not _log_level_explicit:  # every line, the core's default (the logging bridge's check sets its own level)
            lib.fn("iad_set_log_level")(0)


def use_python_logging() -> None:
    """Send the core's log lines to :mod:`logging`, each to the logger named by its source."""
    set_log_sink(lambda level, source, message: logging.getLogger(source or __name__).log(level, "%s", message))


def backup_axes(
    axes: Mapping[str, NativeAxis] | Iterable[NativeAxis], directory: str | os.PathLike[str], *,
    rows: str | Iterable[int] = "used",
) -> dict[str, str]:
    """:meth:`NativeAxis.backup` of several axes into ``directory`` (created if missing), one ``<name>.json`` each
    (info::backup_axes; read-only). ``axes``: ``{name: axis}`` or axes (named by their ``port:slave``). Returns
    ``{name: file path}``, or ``{name: "error: ..."}`` for an axis that did not answer; the others are still saved."""
    items = list(axes.items()) if isinstance(axes, Mapping) else [(None, a) for a in axes]
    if not items:
        return {}
    lib = items[0][1]._lib
    handles = (ctypes.c_void_p * len(items))(*(axis._h() for _, axis in items))
    names = ",".join(str(name) for name, _ in items) if isinstance(axes, Mapping) else None
    return _read_json(lib, "iad_backup_axes_json", handles, len(items), _utf8(names), _utf8(os.fspath(directory)),
                      _utf8(_rows(rows)))
