/*
 * iai_actuator_driver.h - C ABI of the IAI Actuator Driver native core.
 *
 * Unofficial driver for IAI actuators (ELECYLINDER units and CON controllers with RoboCylinder) over serial Modbus RTU.
 * Not an IAI product. This plain C99 header is the binary interface of the native library (iai_actuator_driver.dll,
 * libiai_actuator_driver.so, libiai_actuator_driver.dylib) for C, C#, Python, LabVIEW and any language with a C FFI.
 * The C++ classes behind it (the headers in include/iai_actuator_driver/) are not part of the binary interface. The
 * model (handles, errors, threading, JSON outputs, versioning, loading per language, distribution) is in
 * docs/NATIVE_ABI.md.
 *
 * Behaviour: each function does what the C++ method named in its comment does (axis.hpp, cell.hpp); the checks, the
 * completion rule, the STOP handling and the non-volatile write guard are the core's. Evidence: the C++ core's command
 * line is VERIFIED on the EC-GRBP8M gripper and on the PCON-CB with an RCP6; the ABI 0.1 functions (see "Layout") and
 * the bindings on them are VERIFIED on the PCON-CB with an RCP6 (docs/NATIVE_ABI.md); the rest is UNVERIFIED on
 * hardware.
 *
 * Conventions
 * - Results: every function returns an iad_result (IAD_OK = 0, an error < 0), except iad_abi_version,
 *   iad_library_version, iad_last_error_message and iad_result_name, which cannot fail. A call that fails sets the
 *   calling thread's last error (iad_last_error_message, iad_last_error_json); a call that succeeds clears it. Outputs
 *   of a failed call are unspecified unless its comment says otherwise. No C++ exception crosses this boundary.
 * - Pointers: outputs go through caller-provided pointers. A pointer documented "may be NULL" is skipped when NULL;
 *   any other NULL pointer gives IAD_ERR_INVALID_ARGUMENT and nothing is sent.
 * - Handles: iad_axis* and iad_cell* are opaque tokens, not addresses (never dereference them). An open function
 *   creates one, its close function releases it. Closing NULL or an already closed handle returns IAD_OK; any other
 *   call with a closed or unknown handle returns IAD_ERR_INVALID_ARGUMENT. Handle values are never reused.
 * - Units: positions in mm (deg on a rotary axis), speeds in units/s, accelerations in G, times in seconds.
 * - Unknown values: a double is NaN, an int32_t is IAD_INT_UNKNOWN (INT32_MIN, never -1), an iad_tribool is
 *   IAD_UNKNOWN. In inputs the same values mean "not set" (the default the comment names).
 * - Booleans: iad_bool is an int32_t; inputs 0 = false, any other value = true; outputs 0 or 1. (0.8) An iad_tribool
 *   is IAD_FALSE (0), IAD_TRUE (1) or IAD_UNKNOWN (2) in an output; in an input 0 = false, IAD_UNKNOWN (2) = unknown /
 *   keep what is there, any other value = true, so a Visual Basic True (-1) is true.
 * - Defaults: in an input the value 0 of an enumeration is its default (IAD_METHOD_DEFAULT, IAD_ACCEL_DEFAULT,
 *   IAD_COMPLETION_DEFAULT, IAD_LINE_AUTO, IAD_TEACH_TABLE, IAD_BAUD_DIRECT, a push direction of 0 = +1), and a field a
 *   later minor adds reads 0 as off / the earlier behaviour, so a slot the caller never set keeps the older meaning.
 *   A double is NaN for its default, and a 0 is a value, checked as any other (a speed of 0 is IAD_ERR_SOFT_LIMIT),
 *   except in the two structs that have no _init function and that a caller fills field by field, iad_axis_move and
 *   iad_sequence_point: there NaN or 0 is the default of a speed, acceleration, band or timeout, so a struct zeroed with
 *   its struct_size set is all defaults (a VB or LabVIEW caller need not write a NaN).
 * - Timeouts: the timeout_s of a command that starts motion or a servo change (a move, home, servo, stored-row move,
 *   push, sequence, change of the running move, the moves of iad_cell_move_each / _move_all, iad_cell_home_all /
 *   _servo_all) is NaN for the default or a finite number of seconds above 0. 0, a negative value (-1 is "wait
 *   forever" in VB and LabVIEW) or an infinity is IAD_ERR_INVALID_ARGUMENT and nothing is sent (in iad_axis_move, 0
 *   is the default, as above). There is no "wait forever": give the longest wait the application accepts. The same
 *   for the "timeout_s" of a coordinated request (iad_cell_coord_run_json, iad_cell_coord_plan_json): absent (JSON has
 *   no NaN) is the default. The timeout of a stop (iad_axis_stop, iad_cell_stop_all) is not checked this way: a STOP
 *   always goes out.
 * - Strings: UTF-8, NUL-terminated. A fixed char array in a struct is truncated at a character boundary when the text
 *   is longer; the JSON form has the full text. (0.8) An input name longer than its field (iad_axis_move.axis) is
 *   IAD_ERR_INVALID_ARGUMENT, never cut; iad_cell_result and iad_event carry an index beside the axis name, the
 *   reliable key when a long port name was cut.
 * - JSON outputs (rich data): (char* buffer, size_t buffer_size, size_t* required). *required gets the bytes needed,
 *   including the terminating NUL, whatever the result. When buffer_size is smaller the call returns
 *   IAD_ERR_BUFFER_TOO_SMALL and writes "" (when buffer_size > 0); buffer may be NULL with buffer_size 0. The JSON
 *   functions are read-only, so calling again with a larger buffer is harmless (it repeats the read, if any); the one
 *   that moves, iad_cell_coord_run_json, keeps its result for iad_cell_last_result_json instead. Field
 *   names follow the C++ to_json (json.hpp), which follows the Python edition's to_dict. (0.7) A whole float is
 *   written with a decimal point (1.0), as the Python edition does, so a reader keeps it a float; the numbers the
 *   Python edition holds as ints stay integers (a parameter's code, count or flag and its limits, the stored rows and
 *   the speed override of iad_axis_limits_json, a CON row's table limits and factory default).
 * - Structs: plain, laid out without implicit padding (8-byte fields first, explicit reserved fields), so a packed
 *   layout (32-bit LabVIEW) and the natural one agree. Within an ABI major version fields are never removed,
 *   reordered or resized. (0.8) Outputs (data, results, summaries, events) have a fixed size for the whole major: a
 *   minor may give a reserved slot a meaning (zero = the earlier behaviour), nothing more, so a caller need not
 *   initialise them; ignore their reserved slots. New output data comes as a JSON field or a new struct and function.
 *   Inputs (options, policies, iad_position_entry, iad_sequence_point, iad_axis_move) are sized structs, and no input
 *   struct is embedded in another struct.
 * - Sized structs (0.8; iad_bus_options and iad_transport_callbacks since 0.3): an input struct starts with uint32_t
 *   struct_size (the size of the struct the caller was compiled with) and uint32_t reserved_header (0); a minor version
 *   adds fields at its end. The caller always sets struct_size: to sizeof(the struct) before its _init function or an
 *   output call (iad_axis_get_options, iad_axis_read_position_table), or by _init itself for a struct that is zeroed
 *   (struct_size 0 is a zeroed struct: the library takes the first size); a struct built by hand has its struct_size
 *   set, then its fields. The library reads min(struct_size, its own size): a struct_size below the struct's first
 *   size (its 1.0 size; 0.3 for those two), a non-zero reserved_header, or a struct_size that cannot be a struct's
 *   size (not a multiple of the struct's alignment, or more than 4096 bytes beyond the library's own size: an unset
 *   field holding garbage) is IAD_ERR_INVALID_ARGUMENT; non-zero bytes beyond its own size (a field of a later ABI)
 *   and non-zero reserved slots are IAD_ERR_UNSUPPORTED, never silently ignored. An array of sized structs
 *   (iad_axis_move_sequence, iad_cell_move_each) takes its stride from the first element's struct_size, and every
 *   element has that struct_size. The library writes a sized struct (its _init function and those outputs) up to the
 *   size the caller put in struct_size before the call when that is a size this library knows for the struct, else (0,
 *   or a size it does not know) up to its first size, which every caller's struct has; struct_size then holds the size
 *   written. So a caller that uses a field added after the first size sets struct_size = sizeof(the struct) before
 *   calling _init, and a struct_size that was never set (stack garbage) is the caller's error.
 * - Options: every iad_*_options struct has an iad_*_options_init function that writes its defaults (a zeroed options
 *   struct is NOT the defaults wherever 0 is a value: caps and times use NaN). A NULL options pointer means the
 *   defaults. iad_sequence_point and (0.8) iad_axis_move have no _init: zero them, set struct_size, then the fields
 *   (each comment names its default value).
 * - Threads: as the C++ core. Calls on different handles may run in parallel (one bus mutex per port serialises the
 *   frames of the axes that share a port). iad_axis_stop, iad_axis_pause and iad_cell_stop_all may be called from
 *   another thread while a thread waits for a move on the same handle. The last error is per thread.
 * - Events (0.2): the core's callbacks (the status monitor's on_change / on_done / on_event, the link and recovery
 *   events, the cell's on_fault, the self-test's lines) are queued per handle; the binding drains the queue with
 *   iad_axis_next_event / iad_cell_next_event (a timeout, never a callback). The only calls into the caller's code are
 *   the optional log and frame sinks (iad_set_log_sink, iad_axis_set_frame_sink), run on the thread that logs (a
 *   line or frame of a thread that holds a port's bus comes right after it lets the bus go), and
 *   (0.3) the callbacks the caller installs: a transport's (rules at "callback types"; the testing header adds a
 *   simulator's request hook and a clock), and (0.4) a preemptible read's gate.
 * - Results kept on the handle (0.2): an operation that changes the unit or runs long returns a fixed struct, and its
 *   full record is fetched afterwards without I/O by iad_axis_last_result_json / iad_cell_last_result_json.
 *
 * Stability
 * - Everything declared here is the stable ABI: within an ABI major version a function, struct, constant or JSON field
 *   is never removed, renamed or changed in meaning; a minor only adds (functions, constants, reserved slots given a
 *   meaning, JSON fields). Error codes are only added; every non-zero result is a failure. The version check a caller
 *   makes is IAD_ABI_COMPATIBLE below.
 * - Names: each function exists once, under one name (the variants with a suffix, _ex, _method, _values,
 *   _completion, _flags, 2, are folded into their base functions); the axis commands likely to take more settings
 *   take a sized options struct, where a minor adds a field. Each handle type's functions carry its name (iad_axis_*,
 *   iad_cell_*, iad_bus_*, iad_move_handle_*). The prefix iad_sel_ is reserved for the planned SEL header (see the
 *   result codes).
 * - [EXPERIMENTAL] marks a function, constant or field whose behaviour is not settled (UNVERIFIED on any unit): it
 *   stays exported and declared for the whole major, but a minor may change its behaviour or make it return
 *   IAD_ERR_UNSUPPORTED. Do not build on it without a fallback.
 * - JSON outputs: within a major, fields are only added, never removed, renamed or retyped (an integer stays an
 *   integer, a float a float); null means unknown. core/abi/json_keys.json lists the fields of every JSON output and a
 *   test of the source tree fails on a removed, renamed or retyped one.
 * - iai_actuator_driver_testing.h declares the test hooks the same library exports (the library's clock, the
 *   simulator's fault and state hooks, the driver's time limits, the port list): no stability promise, a version of
 *   its own (exact match), never needed in production.
 *
 * Layout
 * - Types first: export and version, basic types and handles, result codes, constants, callback types, options
 *   (inputs), data (outputs), then the structs of motion, monitors, events, recovery and the self-test, and those of
 *   transports and buses. The functions follow, in sections named by topic:
 *     library:     "library", "option defaults", "library: log sink, option defaults", "library: callback error, log
 *                  level, defaults", "stateless functions", "library: per-thread results; cell: link monitor pass"
 *     axis:        "axis: session" to "axis: unit" (eight sections), "axis: push, sequence, acceleration change",
 *                  "axis: move handles", "axis: status monitor, events", "axis: fast mode", "axis: recovery", "axis:
 *                  unit information, backup, self-test", "axis: line, modes, strategy, cancel", "axis and cell:
 *                  cancel latch", "axis: state without I/O", "axis: status monitor holds, port owner", "axis:
 *                  recovery sessions, link state", "axis: directory readers, self-test lines"
 *     cell:        "cell", "cell: pause, cancel, monitors, recovery, lines, backup", "cell: coordinated motion", "axes
 *                  and cells on transports and buses", "axis and cell: cancel latch", "ports and cells over the
 *                  caller's buses", "library: per-thread results; cell: link monitor pass"
 *     buses:       "transports", "buses", "buses: bus lock", "buses: preemptible reads", "buses: status monitors",
 *                  "buses: broadcasts", "buses: the line", "buses: response timing", "buses: shutdown, release",
 *                  "buses: another transport"
 *     lines:       "lines and gateways", "ports and cells over the caller's buses", "ports: baud detection, device
 *                  info"
 *     simulators:  "simulators", "simulators: ports, gateway lines"
 * - The declarations keep the order of ABI 1.0 (core/abi/manifest-1.0.txt; the bindings bind the functions in that
 *   order), so a topic can have several sections, listed above; a new function goes into its topic's section. The ABI
 *   0.1 functions are those of the sections "library" to "cell"; the sections after "cell" came with 0.2 to 0.7, and a
 *   "(0.x)" tag marks what a later version added to an earlier section, struct or comment.
 */
#ifndef IAI_ACTUATOR_DRIVER_H
#define IAI_ACTUATOR_DRIVER_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------------------------- export, version */

/* IAD_API marks an exported function: dllexport while building the shared library (IAD_BUILD_SHARED), default
 * visibility on ELF / Mach-O. Callers need no definition: a function call through the import library works without
 * dllimport. */
#if defined(IAD_BUILD_SHARED)
#  if defined(_WIN32) || defined(__CYGWIN__)
#    define IAD_API __declspec(dllexport)
#  elif defined(__GNUC__) || defined(__clang__)
#    define IAD_API __attribute__((visibility("default")))
#  else
#    define IAD_API
#  endif
#else
#  define IAD_API
#endif

/* Calling convention. IAD_CALL is __stdcall on 32-bit Windows (x86) only, where 32-bit Visual Basic / VBA / VB6
 * can call nothing else and Declare / default DllImport assume it; the library exports the undecorated names there.
 * Everywhere else (x64, arm64 and arm32 Windows, Linux, macOS) it is the platform's one convention and expands to
 * nothing. It is on every function and on every callback type. The 32-bit Windows build itself is not made yet (the
 * macro is written because the header froze at 1.0). Bindings: ctypes WinDLL / WINFUNCTYPE on 32-bit Windows,
 * CDLL / CFUNCTYPE otherwise; C# CallingConvention.Winapi (stdcall on 32-bit Windows, cdecl elsewhere). */
#if !defined(IAD_CALL)
#  if defined(_WIN32) && (defined(_M_IX86) || defined(__i386__))
#    define IAD_CALL __stdcall
#  else
#    define IAD_CALL
#  endif
#endif

/* ABI version: 1.0, frozen within major 1. A different major means an incompatible ABI: a binding refuses to load it.
 * A higher minor only adds functions or gives reserved slots a meaning, so a library of the same major and at least
 * the minor a caller was compiled with serves it. 1.0 is the layout of 0.8 relabelled, with no layout change; the
 * "(0.8)" tags below mark what that layout added, the other 0.x tags where a function first appeared. */
#define IAD_ABI_VERSION_MAJOR 1
#define IAD_ABI_VERSION_MINOR 0
/** Both in one number, as iad_abi_version() returns it: (major << 16) | minor. Usable in #if. */
#define IAD_ABI_VERSION ((IAD_ABI_VERSION_MAJOR << 16) | IAD_ABI_VERSION_MINOR)
/** The loader rule: whether a library whose iad_abi_version() is `library_version` serves a caller compiled with
 *  this header: the same major, and a library minor at least this header's minor (a higher minor only adds). The
 *  macro's exact match for a major 0 served the 0.x development versions only. Every loader applies it before the
 *  first other call (the bindings do it when they load the library):
 *      if (!IAD_ABI_COMPATIBLE(iad_abi_version())) { refuse to run: the library is too old or of another major } */
#define IAD_ABI_COMPATIBLE(library_version)                                                                          \
    (IAD_ABI_VERSION_MAJOR == 0                                                                                       \
         ? (uint32_t)(library_version) == (uint32_t)IAD_ABI_VERSION                                                   \
         : ((uint32_t)(library_version) >> 16) == (uint32_t)IAD_ABI_VERSION_MAJOR &&                                  \
               (uint32_t)(library_version) >= (uint32_t)IAD_ABI_VERSION)

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------ basic types, handles */

/** Result of every function: IAD_OK (0) or an error kind (< 0). */
typedef int32_t iad_result;
/** 0 = false, otherwise true (outputs: 0 or 1). */
typedef int32_t iad_bool;
/** IAD_FALSE, IAD_TRUE or IAD_UNKNOWN (outputs). Inputs: 0 = false, IAD_UNKNOWN = unknown / keep, any other = true. */
typedef int32_t iad_tribool;

/** An int32_t output that is not known (the C++ core's nullopt). */
#define IAD_INT_UNKNOWN INT32_MIN

#define IAD_FALSE 0
#define IAD_TRUE 1
/** 2 (not -1, which is True in Visual Basic): typeable in every language; an input value of 0 is false and any other
 *  value except this one is true. */
#define IAD_UNKNOWN 2

/** One axis: an ELECYLINDER or a CON controller at one slave address (C++ Axis). Opaque. */
typedef struct iad_axis iad_axis;
/** Several axes over one or more ports, driven together (C++ Cell). Opaque. */
typedef struct iad_cell iad_cell;
/** (0.2) A move started without waiting (C++ MoveHandle). Opaque, released by iad_move_handle_close. */
typedef struct iad_move_handle iad_move_handle;
/** (0.3) A byte pipe (C++ Transport): a serial port, a simulated unit or line, or the binding's own (callbacks).
 *  Opaque, released by iad_transport_close / iad_transport_release. */
typedef struct iad_transport iad_transport;
/** (0.3) A Modbus RTU master on a transport (C++ ModbusRtu): one transaction at a time, the retries, the frame sink.
 *  Opaque, released by iad_bus_close. */
typedef struct iad_bus iad_bus;
/** (0.3) A simulated unit with test hooks (C++ HookedSimUnit, sim_hooks.hpp). Opaque, released by iad_sim_close. */
typedef struct iad_sim iad_sim;
/** (0.3) Several simulated units on one line by slave address (C++ SimLine). Opaque, released by
 *  iad_sim_line_close. */
typedef struct iad_sim_line iad_sim_line;
/** (0.5) A Line (C++ shared, thread-safe). Opaque, released by iad_line_close. */
typedef struct iad_line iad_line;
/** (0.5) An RCON or ROBONET gateway object of a line (read-only: never written). Opaque, released by
 *  iad_gateway_close. */
typedef struct iad_gateway iad_gateway;

/* ------------------------------------------------------------------------------------------- result codes */

/* The error kinds mirror the C++ exception classes (errors.hpp). Where the core has a subclass, the more specific
 * code is returned (a servo that cannot come on is IAD_ERR_SERVO_BLOCKED, which a binding maps to a subclass of its
 * not-ready error). Codes are never renumbered. */
enum iad_result_code {
    IAD_OK = 0,
    IAD_ERR_SOFT_LIMIT = -1,          /* SoftLimitError: a value outside the unit's range or a cap; nothing sent */
    IAD_ERR_NOT_READY = -2,           /* EcNotReadyError: servo off, not homed, moving, no Modbus control... */
    IAD_ERR_ALARM = -3,               /* EcAlarmError: the unit reports an alarm (code in iad_last_error_json) */
    IAD_ERR_MOTION_TIMEOUT = -4,      /* MotionTimeoutError: motion did not finish in time (a decel stop was sent) */
    IAD_ERR_MODBUS_TIMEOUT = -5,      /* ModbusTimeout: no (complete) answer after all retries */
    IAD_ERR_FRAME = -6,               /* ModbusCrcError / ModbusFrameError: bad CRC or mismatched answer */
    IAD_ERR_TRANSPORT = -7,           /* TransportError: the port failed (busy, absent, USB adapter pulled) */
    IAD_ERR_VERIFICATION = -8,        /* VerificationError: the read-back after a write did not match */
    IAD_ERR_NONVOLATILE_REFUSED = -9, /* NonVolatileWriteError: a protected row; nothing written */
    IAD_ERR_INVALID_ARGUMENT = -10,   /* a NULL pointer, a closed handle, an unknown name or key, a value of the wrong
                                         form (std::invalid_argument, std::out_of_range); nothing sent */
    IAD_ERR_UNSUPPORTED = -11,        /* not available in this build or on this handle (e.g. a line-break baud
                                         change on a transport that cannot send a break: std::logic_error) */
    IAD_ERR_INTERNAL = -12,           /* anything else (out of memory, an unexpected exception): a bug, please report */
    IAD_ERR_BUFFER_TOO_SMALL = -13,   /* a JSON or text output did not fit; *required holds the size needed */
    IAD_ERR_SERVO_BLOCKED = -14,      /* ServoBlockedError (a not-ready kind): EMG input, drive power cut or enable
                                         switch released; reasons in iad_last_error_json */
    IAD_ERR_MOTION_INCOMPLETE = -15,  /* MotionIncompleteError: settled away from the target */
    IAD_ERR_SLAVE_EXCEPTION = -16,    /* ModbusSlaveError: the unit answered with a Modbus exception response */
    IAD_ERR_COMMAND_REJECTED = -17,   /* IaiCommandRejected: an IAI 0x45 command answered with a rejection code */
    IAD_ERR_UNSAFE_REFUSED = -18,     /* UnsafeCommandError: a raw IAI command off the safe list or on the
                                         never-send list, a read-only, unconfirmed, not-offered or manual-rule
                                         refused parameter, or a class that is not a parameter class; nothing
                                         sent */
    IAD_ERR_CELL = -19,               /* CellError: one or more axes of a group command failed (per axis in the
                                         results array and in iad_last_error_json) */
    IAD_ERR_MODBUS = -20,             /* any other ModbusError */
    IAD_ERR_DRIVER = -21,             /* any other EcError / EcDriverError */
    IAD_ERR_RECOVERY = -22,           /* (0.2) RecoveryError: the port / unit did not come back in time, or the
                                         identity could not be confirmed ("report" in iad_last_error_json) */
    IAD_ERR_UNIT_REPLACED = -23,      /* (0.2) UnitReplacedError (a recovery kind): another unit answers where the
                                         tracked one was; nothing written ("expected", "found", "report") */
    IAD_ERR_FILE = -24,               /* (0.2) a file could not be read or written (std::ios_base::failure; a
                                         backup file, a backup directory) */
    IAD_ERR_CANCELLED = -25           /* (0.2) WaitCancelledError: a blocking call was ended by iad_axis_cancel /
                                         iad_cell_cancel from another thread (a motion wait sent a decel stop first;
                                         the notes say so). Bindings: KeyboardInterrupt / OperationCanceledException */
};

/* (0.3) IAD_ERR_INVALID_ARGUMENT carries the C++ class in the "kind" of iad_last_error_json: "std::out_of_range" for a
 * value outside its range or a name / index that is not there (a register row, a slave on a line, a cell axis),
 * "std::invalid_argument" for anything else (a NULL pointer, a closed handle, a malformed value). Bindings map the
 * first to their out-of-range error (Python: an error that is both ValueError and IndexError; C#:
 * ArgumentOutOfRangeException). */

/* Reserved: the codes from -26 down (the last one above is -25) for coming additions, and the prefix iad_sel_ (IAD_SEL_
 * for its constants) for the planned SEL header, iai_actuator_driver_sel.h, a header of its own with its own version.
 * Its codes come from the same range: one numbering for every header of the library, and a code is never reused. A
 * binding maps a negative code it does not know to its base driver error and keeps the code (Python EcDriverError, C#
 * EcDriverException), so a code a newer library adds never breaks an older binding. */

/* ------------------------------------------------------------------------------------------- constants */

/* How servo / home / alarm reset are sent (C++ CommandMethod). (0.8) 0 is the default, as in every enumeration:
 * IAD_METHOD_DEFAULT is the axis option in a call (iad_axis_home, iad_axis_reset_alarm, iad_cell_home_all) and the
 * library's own choice (COIL; the line's strategy where it names one) in iad_axis_options.method; where a method is
 * set or read back (iad_axis_set_strategy, iad_axis_strategy, iad_axis_get_options) it is always a concrete one. */
#define IAD_METHOD_DEFAULT 0
#define IAD_METHOD_COIL 1     /* FC05 coils SON / HOME / ALRS (what the default gives; the line's own where it names one) */
#define IAD_METHOD_IAI 2      /* IAI 0x45 commands */
#define IAD_METHOD_IAI2 3     /* (0.2) [EXPERIMENTAL] IAI 0x48 commands (UNVERIFIED on any unit) */
#define IAD_METHOD_REGISTER 4 /* (0.2) FC06 writes of DRG1 0x0D00 (the RCON path; DOC, UNVERIFIED on hardware) */

/* (0.2) How a teach stores the current position (C++ TeachMethod). */
#define IAD_TEACH_TABLE 0 /* read the row, replace the position, write it back (the default) */
#define IAD_TEACH_COIL 1  /* [EXPERIMENTAL] MOD / POSR / TEAC coils (DOC for CON units; UNVERIFIED on an EC) */

/* How a baud change is sent (C++ BaudChangeMethod). */
#define IAD_BAUD_DIRECT 0 /* broadcast at the current rate, then switch the port in place (the default) */
#define IAD_BAUD_BREAK 1  /* port to 9600, line BREAK, broadcast at 9600 (UNVERIFIED on an EC) */

/* Alarm level (C++ AlarmLevel). */
#define IAD_ALARM_LEVEL_NONE 0                /* no alarm, or the 0x0FFF power-up marker */
#define IAD_ALARM_LEVEL_MESSAGE 1             /* a warning: the axis keeps running */
#define IAD_ALARM_LEVEL_OPERATION_CANCELLED 2 /* stop + servo off; an alarm reset clears it */
#define IAD_ALARM_LEVEL_COLD_START 3          /* stop + servo off; only a software reset or a power cycle clears it */
#define IAD_ALARM_LEVEL_UNKNOWN 4             /* no known level: a fault unless the warning bit ALML is set */

/* (0.2) Acceleration pattern of a move (C++ AccelPattern, CTLF b6-b7). */
#define IAD_ACCEL_DEFAULT 0   /* not set: trapezoid (CTLF b6-b7 = 00, as before 0.2) */
#define IAD_ACCEL_TRAPEZOID 1
#define IAD_ACCEL_S_MOTION 2  /* S-curve (CON No.56 ratio); refused above an acceleration time of 2 s */
#define IAD_ACCEL_FILTER 3    /* first-order filter (CON No.55 time constant) */

/* (0.2) When a move counts as done (C++ Completion). */
#define IAD_COMPLETION_DEFAULT 0 /* a move: the axis option; the axis option: safe */
#define IAD_COMPLETION_SAFE 1    /* PEND, the position within the band and PNOW unchanged over settle_s */
#define IAD_COMPLETION_FAST 2    /* one sample: PEND, within the band, DSSE MOVE clear, not homing, not paused */

/* (0.2) The line a port is on (C++ LineOptions::kind). */
#define IAD_LINE_AUTO 0            /* probe once for an RCON gateway (D0h) and a ROBONET gateway (3Fh) */
#define IAD_LINE_DIRECT 1          /* no gateway, no probe */
#define IAD_LINE_RCON 2
#define IAD_LINE_ROBONET 3         /* SIO through mode (cannot be probed) */
#define IAD_LINE_ROBONET_GATEWAY 4 /* gateway mode: no axis access */

/* (0.2) Outcome of a push (an output; 0 is not an outcome). */
#define IAD_PUSH_CONTACT 1 /* holding against the work (PEND and DSSE b0); the motor keeps pushing */
#define IAD_PUSH_MISSED 2  /* went over the whole band without contact (DSS1 PSFL) */

/* (0.2) Kinds of a queued event (iad_event.kind). */
#define IAD_EVENT_NONE 0       /* the timeout passed with no event */
#define IAD_EVENT_STATUS 1     /* the monitor saw PNOW / ALMC / DSS1 / DSS2 / DSSE change (on_change) */
#define IAD_EVENT_DONE 2       /* the axis became done by the fast rule against its last target (on_done) */
#define IAD_EVENT_LINK 3       /* a link event: name "link_lost", "unit_restarted", "unit_replaced", "recovered" */
#define IAD_EVENT_FAULT 4      /* a cell's on_fault: name = the kind ("port_vanished", "unit_silent"...) */
#define IAD_EVENT_LINK_STATE 5 /* a link monitor saw an axis's state change: name = the state, message = reasons */
#define IAD_EVENT_CHECK_ITEM 6 /* one line of iad_axis_self_test: name = the item, cause = "PASS" / "FAIL"... */

/* (0.2) Which events a handle queues (iad_monitor_options.events). */
#define IAD_EVENTS_STATUS 0x01
#define IAD_EVENTS_DONE 0x02
#define IAD_EVENTS_LINK 0x04  /* IAD_EVENT_LINK, IAD_EVENT_FAULT and IAD_EVENT_LINK_STATE */
#define IAD_EVENTS_CHECK 0x08
#define IAD_EVENTS_ALL 0x0F

/* (0.2) Log levels (C++ LogLevel) and frame directions (C++ FrameDirection) of the sinks. */
#define IAD_LOG_DEBUG 0
#define IAD_LOG_INFO 1
#define IAD_LOG_WARNING 2
#define IAD_LOG_ERROR 3
#define IAD_FRAME_TX 0
#define IAD_FRAME_RX 1
#define IAD_FRAME_RX_PARTIAL 2

/** (0.2) The rate iad_axis_enable_fast raises the link to by default (VERIFIED on the EC-R6S rod in the Python
 *  edition). */
#define IAD_FAST_BAUD_RATE 230400

/* Sizes of the fixed text fields. */
#define IAD_AXIS_NAME_SIZE 128 /* "COM3:1" = port:slave; (0.8) 128 (was 64): Linux /dev/serial/by-id/... names */
#define IAD_MESSAGE_SIZE 256
#define IAD_STATE_SIZE 32     /* (0.2) link states, event names, outcomes */

/* (0.4) Opt-ins of an IAI command (iad_bus_iai_command, iad_bus_broadcast_iai_command); 0 = none: the safe list, ID
 * group 0, a rejection is an error. Another bit: IAD_ERR_UNSUPPORTED (nothing sent). */
#define IAD_IAI_ALLOW_UNSAFE 0x1     /* a command not on the safe list, another ID group, 0x45/0x05 with op1 != 0 */
#define IAD_IAI_ALLOW_NEVER_SEND 0x2 /* a command on the never-send list (only on the user's explicit request) */
#define IAD_IAI_NO_CHECK 0x4         /* a rejection is returned in *rejection, not as IAD_ERR_COMMAND_REJECTED */

/* (0.6) What queued an event (iad_event.source). (0.7) A status monitor's IAD_EVENT_STATUS / IAD_EVENT_DONE carry
 * IAD_EVENT_SOURCE_MONITOR. */
#define IAD_EVENT_SOURCE_MONITOR 1        /* the status monitor (its polls and its link watch) */
#define IAD_EVENT_SOURCE_CHECK_LINK 2     /* iad_axis_check_link */
#define IAD_EVENT_SOURCE_RECOVER 3        /* iad_axis_recover */
#define IAD_EVENT_SOURCE_SET_LINK_STATE 4 /* iad_axis_set_link_state */
#define IAD_EVENT_SOURCE_CELL 5           /* a cell's fault handling, link monitor and recovery (iad_cell_next_event) */

/* ------------------------------------------------------------------------------------------- callback types */

/* Callbacks (0.3): a transport's callbacks (and the testing header's request hook and clock) run synchronously on the
 * thread doing the work (the caller's thread for a single axis, a core worker thread for a cell's group command), with
 * the port's bus lock held. They must return quickly, must not let an exception (or a longjmp) cross into the library
 * and must not call the library, except iad_callback_error, iad_sim_handle_frame on the simulator being hooked (from
 * its request hook) and the cancel functions (iad_axis_cancel, iad_axis_cancel_latch, iad_cell_cancel...). Bindings
 * keep every callback pointer alive until the release callback ran or the library is no longer used. */

/** (0.2) A log line of the core (level IAD_LOG_*, source such as "iai_actuator_driver.axis"). Runs on the thread
 *  that logs (any thread: a cell worker, a monitor thread): copy what you need, return quickly, never call the
 *  library from it. */
typedef void(IAD_CALL* iad_log_sink)(int32_t level, const char* source, const char* message, void* user);

/** (0.2) One Modbus frame on a port (direction IAD_FRAME_*; `bytes` valid during the call only). Rules of
 *  iad_log_sink. */
typedef void(IAD_CALL* iad_frame_sink)(int32_t direction, const uint8_t* bytes, size_t length, void* user);

/* (0.3) Transport callbacks (iad_transport_callbacks). "user" is the pointer given to iad_transport_open_callbacks. A
 * result of 0 is success; any other value is a failure of the port, raised as IAD_ERR_TRANSPORT (TransportError) with
 * the text the callback gave iad_callback_error, if any (IAD_CALLBACK_ABORT, below, is not a failure of the port). */
/** Send all `size` bytes. */
typedef int32_t(IAD_CALL* iad_transport_write_fn)(void* user, const uint8_t* data, size_t size);
/** Read up to `size` bytes, waiting at most timeout_s (the transport's read timeout); *got = the bytes read (0 = none
 *  came: a timeout is not an error). */
typedef int32_t(IAD_CALL* iad_transport_read_fn)(void* user, uint8_t* buffer, size_t size, double timeout_s,
                                                 size_t* got);
/** Discard received bytes not read yet. */
typedef int32_t(IAD_CALL* iad_transport_reset_input_fn)(void* user);
/** Change the PC side's rate in place. */
typedef int32_t(IAD_CALL* iad_transport_set_baud_rate_fn)(void* user, int32_t rate);
/** Hold the line in BREAK for `seconds`. */
typedef int32_t(IAD_CALL* iad_transport_send_break_fn)(void* user, double seconds);
/** Whether the port is listed by the PC now: IAD_TRUE, IAD_FALSE or IAD_UNKNOWN. */
typedef int32_t(IAD_CALL* iad_transport_present_fn)(void* user);
/** Open the same port again in place (`baud_rate` 0: as before). */
typedef int32_t(IAD_CALL* iad_transport_reopen_fn)(void* user, int32_t baud_rate);
/** The port is closed (Transport::close; called once). */
typedef void(IAD_CALL* iad_transport_close_fn)(void* user);
/** The library lets go of `user` (called once, after the last callback; the binding frees what `user` holds). */
typedef void(IAD_CALL* iad_transport_release_fn)(void* user);
/** (0.5) The bus changes the read timeout: a probe or a capped read (iad_bus_probe, max_wait) shortens it
 *  before the request is written and puts it back after, and iad_transport_set_read_timeout sets it. A port that waits
 *  on a timeout of its own takes `seconds` here; read's timeout_s follows it either way. A failure leaves the timeout as
 *  it was (the request then waits as before). */
typedef int32_t(IAD_CALL* iad_transport_set_read_timeout_fn)(void* user, double seconds);

/** (0.7) A transport callback's result "the callback failed with an error that is not the port's"
 *  (iad_transport_*_fn): raised as IAD_ERR_DRIVER (EcDriverError) with the text the callback gave iad_callback_error,
 *  never retried nor taken for a link fault. Any other non-zero result stays a failure of the port
 *  (IAD_ERR_TRANSPORT). */
#define IAD_CALLBACK_ABORT 2

/** (0.4) A preemptible read's gate (iad_bus_preemptible_begin): called on the reading thread before every request
 *  attempt of the scope, outside the bus lock. 0 = send it; any other value ends the read with IAD_ERR_CANCELLED
 *  (nothing more is sent; the text given to iad_callback_error, if any, is the message). It may wait (e.g. for the
 *  binding's own lock) but must not use this bus. */
typedef int32_t(IAD_CALL* iad_read_gate_fn)(void* user);

/** (0.5) How a binding works out the line of its port itself (its own detect_line, which states or probes the line and
 *  attaches it to the bus): the port calls it once the baud rate is settled, where it would probe for a gateway (its
 *  first scan, iad_bus_port_detect_line, a cell's open), on the thread doing that work (a cell's port thread). Return
 *  IAD_OK; another code leaves the line undetected (logged, asked again next time). It may use the bus. */
typedef iad_result (IAD_CALL *iad_port_line_fn)(void* user);

/* (0.8) The clock callbacks (iad_clock_now_fn, iad_clock_sleep_fn), a simulator's request hook
 * (iad_sim_request_hook) and its answers (IAD_SIM_*) are in iai_actuator_driver_testing.h. */

/* ------------------------------------------------------------------------------------------- options (inputs) */

/** Settings of one axis (C++ AxisOptions). A cap can only narrow the unit's own range, never widen it. */
typedef struct iad_axis_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double stroke_min;           /* application soft limits (both or neither), intersected with the unit's */
    double stroke_max;
    double max_speed_mm_s;       /* cap for any commanded speed (units/s); NaN = the unit's own maximum */
    double max_accel_g;          /* cap for any acceleration / deceleration (G); NaN = the unit's own maximum */
    double max_push_pct;         /* cap for push power (%); NaN = the unit's own maximum */
    double poll_interval_s;      /* status poll interval while waiting; default 0.02 */
    double settle_s;             /* "done" needs the position unchanged this long; default 0.1 */
    int32_t method;              /* IAD_METHOD_DEFAULT (0: = COIL) or a concrete IAD_METHOD_*; get_options: a concrete one */
    iad_bool auto_modbus_control;/* take Modbus control (PMSL) automatically at rest before a command; default 1 */
    double push_settle_s;        /* (0.2) a push outcome must hold this long; NaN or 0 = 0.15 s (the default); a
                                    negative value = no settle (the first sample counts) */
    int32_t completion;          /* (0.2) IAD_COMPLETION_SAFE (default; 0 = the default) or IAD_COMPLETION_FAST */
    int32_t reserved0;
    uint64_t reserved[2];
} iad_axis_options;

/** Settings of iad_axis_open (C++ OpenOptions without its AxisOptions). */
typedef struct iad_open_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double read_timeout_s;       /* default 0.2 */
    double min_gap_s;            /* minimum silence between transactions; default 0.005 */
    int32_t slave;               /* 1..247 (axis number + 1); default 1 */
    int32_t baud_rate;           /* 0 = detect in place (default; 115200, 38400, 9600, 230400...), else this */
    int32_t retries;             /* extra attempts after a timeout / CRC / mismatched frame; default 3 (IAI's advice) */
    int32_t line;                /* (0.2) IAD_LINE_AUTO (default: probe once for a gateway), IAD_LINE_DIRECT... */
    double position_unit_mm;     /* (0.2) RCON line: the axes' position unit, 0.01 or 0.001; NaN or 0 = not stated */
    iad_bool fast;               /* (0.2) then iad_axis_enable_fast (230400 baud, fast completion, the monitor);
                                    default 0 */
    int32_t reserved0;
    uint64_t reserved[2];
} iad_open_options;

/** Settings of a simulated unit (C++ SimulatedUnitOptions, the part a binding test needs). It answers at the open
 *  options' slave address. */
typedef struct iad_sim_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double time_scale;           /* simulated seconds per real second; default 1 (e.g. 20 makes tests fast) */
    double position_mm;          /* initial position; NaN = the profile's */
    double stroke_mm;            /* soft limits -0.30 .. stroke + 0.30; NaN = the profile's */
    double max_speed_mm_s;       /* MAXV; NaN = the profile's */
    int32_t baud_rate;           /* 0 = no baud rate modelled (default); else the unit listens at this rate */
    iad_bool pmsl;               /* initial Modbus control; default 1 (the hardware boots with 0) */
    uint64_t reserved[4];
} iad_sim_options;

/** One absolute / relative move (C++ MoveOptions). */
typedef struct iad_move_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double speed;                /* units/s: 0.01 .. limits speed.max; default 50 (jog: 10) */
    double accel_g;              /* G (= decel on a 0x9900 move): 0.01 .. limits accel_decel.max; default 0.1 */
    double band;                 /* positioning band (units): limits band; default 0.1 */
    double timeout_s;            /* NaN (default) = 2 x the move's estimate + 2 s */
    iad_bool wait;               /* block until done (default 1); 0 = return right after the command */
    int32_t accel_pattern;       /* (0.2) IAD_ACCEL_DEFAULT (0: trapezoid, as before), _TRAPEZOID, _S_MOTION, _FILTER
                                    (CTLF b6-b7; refused when the unit's MVPT lacks it) */
    double decel_g;              /* (0.2) a deceleration other than accel_g: the command-profile path (FC17 0x0F00 +
                                    PCRF; accel checked against MAXA, decel against MAXD). NaN or 0 = accel_g (the
                                    0x9900 block, the default) */
    int32_t completion;          /* (0.2) IAD_COMPLETION_DEFAULT (0: the axis option), _SAFE or _FAST */
    int32_t reserved0;
    uint64_t reserved[2];
} iad_move_options;

/** A stored-row move (C++ PositionMoveOptions). */
typedef struct iad_position_move_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double timeout_s;            /* NaN = the core's default */
    double speed_pct;            /* 1..100 % of the row's speed (0x45 cmd 6, nothing written); NaN = the row's */
    iad_bool wait;               /* default 1 */
    iad_bool allow_above_caps;   /* only the unit's own limits apply to the row's speed / accel; default 0 */
    uint64_t reserved[4];
} iad_position_move_options;

/** Options of a non-volatile write: position table, teach (C++ NvWriteOptions). All default 0. */
typedef struct iad_nv_write_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    iad_bool allow_nonvolatile;  /* accepted and ignored (no longer needed) */
    iad_bool force;              /* write even when the row already holds these words (uses a write cycle) */
    iad_bool allow_unsafe;       /* accepted and ignored (no longer needed): such rows get a warning */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_nv_write_options;

/** Options of a parameter write (C++ ParameterWriteOptions). */
typedef struct iad_param_write_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    iad_bool commit;             /* save with PARW (applies after a software reset or a power cycle); default 1 */
    iad_bool allow_dangerous;    /* accepted and ignored (no longer needed): a dangerous write (home, motor, lead,
                                    gains, PIO / fieldbus / baud, widening a soft limit, raw items) gets a warning */
    iad_bool allow_unlisted;     /* accepted and ignored (no longer needed): an unlisted parameter gets a warning */
    iad_bool allow_home_params;  /* accepted and ignored (no longer needed): EC P3 / P4, CON No.5 / No.22 */
    iad_bool force;              /* write and commit even when the value is unchanged; default 0 */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_param_write_options;

/** Options of a baud change (C++ BaudChangeOptions). */
typedef struct iad_baud_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double settle_s;             /* after the broadcast, before the port switches; default 0.2 */
    int32_t method;              /* IAD_BAUD_DIRECT (default) or IAD_BAUD_BREAK */
    iad_bool allow_unverified_code; /* accepted and ignored (no longer needed): an UNVERIFIED code is sent */
    uint64_t reserved[4];
} iad_baud_options;

/** Options of a software reset (C++ SoftwareResetOptions). */
typedef struct iad_reset_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double timeout_s;            /* give up when the unit has not answered this long; default 10 */
    double reset_wait_s;         /* after the reset, before probing; default 1 */
    uint64_t reserved[4];
} iad_reset_options;

/** Settings of iad_cell_open (C++ CellOptions without its AxisOptions and the callback). */
typedef struct iad_cell_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double read_timeout_s;       /* per port; default 0.2 */
    double min_gap_s;            /* per port; default 0.005 */
    double scan_timeout_s;       /* per probed address; default 0.05 */
    int32_t max_axes;            /* the most axes the cell takes; 0 = no limit (default); < 0 refused */
    int32_t baud_rate;           /* 0 = detect each port once (default), else this rate for every port */
    int32_t retries;             /* per port; default 3 */
    int32_t scan_first;          /* addresses scanned: scan_first .. scan_last; default 1 .. 16 */
    int32_t scan_last;
    iad_bool identify;           /* identify every axis at open (READ-ONLY); default 1 */
    iad_bool stop_on_fault;      /* a link fault on one axis stops the others; default 1 */
    int32_t line;                /* (0.2) every port's line kind: IAD_LINE_AUTO (default), IAD_LINE_DIRECT... */
    double position_unit_mm;     /* (0.2) RCON lines: the axes' position unit, 0.01 or 0.001; NaN or 0 = not stated */
    uint64_t reserved[3];
} iad_cell_options;

/* (0.8) The options of the axis commands that took scalars before (iad_axis_servo, _home, _stop, _wait_idle,
 * _change_speed / _change_accel, _cancel_paused_move), so that they grow by a field instead of a new function. */

/** Options of iad_axis_servo (C++ Axis::servo). */
typedef struct iad_servo_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double timeout_s;            /* NaN (default) = the core's: 3 s, longer before a CON unit's first servo ON */
    iad_bool wait;               /* wait until DSS1 shows it (default 1); 0 = return right after the command */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_servo_options;

/** Options of iad_axis_home (C++ Axis::home). */
typedef struct iad_home_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double timeout_s;            /* NaN (default) = the core's */
    iad_bool wait;               /* wait for HEND and standstill (default 1); 0 = return right after the command */
    int32_t method;              /* IAD_METHOD_DEFAULT (0: the axis option) or a concrete IAD_METHOD_* */
    uint64_t reserved[4];
} iad_home_options;

/** Options of iad_axis_stop (C++ Axis::stop). */
typedef struct iad_stop_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double timeout_s;            /* NaN (default) = 5 s */
    iad_bool wait;               /* IAD_ERR_MOTION_TIMEOUT when still moving after timeout_s (default 0) */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_stop_options;

/** Options of iad_axis_wait_idle (C++ Axis::wait_idle). */
typedef struct iad_wait_idle_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double target;               /* NaN (default) = not checked; else the axis must settle at it within `band` */
    double band;                 /* NaN (default) = 0.1 units */
    double timeout_s;            /* NaN (default) = 10 s */
    int32_t completion;          /* IAD_COMPLETION_DEFAULT (0: the axis option), _SAFE or _FAST */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_wait_idle_options;

/** Options of a change of the move in progress (iad_axis_change_speed, iad_axis_change_accel). */
typedef struct iad_change_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double timeout_s;            /* NaN (default) = the core's */
    iad_bool wait;               /* wait until the move is done (default 0) */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_change_options;

/** Options of iad_axis_cancel_paused_move (C++ Axis::cancel_paused_move). */
typedef struct iad_cancel_paused_move_options {
    uint32_t struct_size;        /* sized: see "Sized structs" above */
    uint32_t reserved_header;    /* 0 */
    double timeout_s;            /* NaN (default) = 5 s */
    uint64_t reserved[4];
} iad_cancel_paused_move_options;

/* ------------------------------------------------------------------------------------------- data (outputs) */

/** One decoded status sample (C++ Status: fields and derived flags). */
typedef struct iad_status {
    double position_mm;          /* PNOW (units: mm, or deg on a rotary axis), 0.01 resolution */
    double velocity_mm_s;        /* 0 when not moving; while moving from PNOW samples; NaN when not known */
    double current_ma;           /* CNOW (signed); NaN with fewer than 14 words read */
    double uptime_s;             /* STIM / 1000; NaN when not read */
    int32_t alarm_code;          /* ALMC, 0 = none */
    int32_t alarm_level;         /* IAD_ALARM_LEVEL_* of alarm_code */
    iad_bool servo_on;           /* DSS1 SV */
    iad_bool homed;              /* DSS1 HEND */
    iad_bool in_position;        /* DSS1 PEND */
    iad_bool moving;             /* in motion: DSSE MOVE, a home return in progress, or servo on without PEND while
                                    homed (the raw MOVE bit alone clears early on the EC, VERIFIED) */
    iad_bool alarm;              /* a fault that blocks motion */
    iad_bool warning;            /* a message-level alarm (or ALML) and no fault */
    iad_bool modbus_enabled;     /* DSSE PMSS: Modbus has control */
    iad_bool emergency_stop;     /* DSS1 EMGS */
    iad_bool paused;             /* DSS1 STP */
    iad_bool homing;             /* DSSE GHMS */
    iad_bool safety_speed;       /* DSS1 SFTY */
    iad_bool ready;              /* servo on, homed, Modbus control, no fault / EMG / pause / homing */
    iad_bool idle;               /* not moving, not paused, not homing */
    iad_bool target_reached;     /* DSS1 MEND */
    iad_bool push_missed;        /* DSS1 PSFL */
    iad_bool pushing;            /* DSSE b10 */
    iad_tribool emg_input;       /* DSSE EMGP (DOC, UNVERIFIED on an EC) */
    iad_tribool drive_power_cut; /* STAT MPOW = 0 or DSSE MPUV */
    iad_tribool enable_off;      /* DSS2 ENBS = 0 */
    int32_t raw_words;           /* words decoded (8, 14, 20 or 22) */
    uint32_t raw_dss1;           /* raw status words, for diagnostics */
    uint32_t raw_dss2;
    uint32_t raw_dsse;
    uint32_t raw_dipm;
    uint32_t raw_dopm;
    iad_bool rcon;               /* (0.2) read behind an RCON gateway: the bits RCON leaves undefined are not decoded */
    uint64_t reserved[8];        /* (0.8) 64 bytes (was 32): embedded in eight outputs, it can only grow into these */
} iad_status;

/** One range the driver enforces (C++ LimitRange). The source of each range, in words, is in iad_axis_limits_json. */
typedef struct iad_range {
    double min;                  /* NaN when not known */
    double max;
    double unit_max;             /* the unit's own maximum */
    double configured;           /* the cap configured on the axis */
    double recommended_min;      /* push: MNPP; speed: the documented minimum speed */
    iad_tribool available;       /* push: the unit can push */
    int32_t count;               /* stored rows: how many; IAD_INT_UNKNOWN otherwise */
    char unit[8];                /* "mm", "mm/s", "G", "%"... */
    uint64_t reserved[4];        /* (0.8) 32 bytes (was 8) */
} iad_range;

/** Every range the driver enforces on one axis (C++ AxisRanges, Axis::limits()). */
typedef struct iad_limits {
    iad_range position;
    iad_range speed;
    iad_range accel;
    iad_range decel;
    iad_range accel_decel;       /* one value for both on a 0x9900 move: min(MAXA, MAXD) */
    iad_range push;
    iad_range band;
    iad_range push_band;
    iad_range stored_rows;
    iad_range jog_step;
    iad_range speed_override;    /* 1..100 % (0x45 cmd 6) */
    char units[8];               /* "mm" or "deg" */
    char kind[24];               /* "slider", "rod", "gripper", "rotary"... "unknown" */
    char family[24];             /* "EC", "PCON", ... */
    double push_speed;           /* (0.2) the push speed, min(PSHV 8.15 / CON No.34, MXPH 7.8) (units/s); NaN when not
                                    readable (its source is in iad_axis_limits_json) */
    uint64_t reserved[8];        /* (0.8) 64 bytes (was 24); a range added later comes from iad_axis_limits_json */
} iad_limits;

/** Identity of one unit (C++ DeviceInfo, Axis::identify()). The full record is iad_axis_identity_json. */
typedef struct iad_identity {
    double soft_min_mm;          /* soft limits 7.1 / 7.2 as stored (with the 0.30 margin); NaN when not known */
    double soft_max_mm;
    double max_speed_mm_s;       /* MAXV 7.4 (or the default speed 8.10) */
    double max_accel_g;
    double max_decel_g;
    int32_t slave;
    int32_t system_code;         /* FC11 byte 0 (0xEC EC, 0xAE PCON...) */
    int32_t unit_code;           /* FC11 byte 2 */
    int32_t run_status;          /* FC11 byte 1 */
    int32_t axis_number;         /* AXNO (= slave - 1) */
    int32_t max_positions;       /* stored rows */
    int32_t actuator_kind;       /* AKID 7.33 */
    iad_tribool unit_known;      /* the driver has a family name for this unit code (display only, never a gate) */
    iad_tribool rotary;
    int32_t reserved0;
    char product_family[16];     /* "EC", "PCON", ... */
    char model[64];              /* the model string, else the controller name, else the controller description */
    char model_name[64];         /* e.g. "EC-R6S-150-1-PN" ("" when the unit keeps none) */
    char controller_name[32];    /* "PCON-CB" on a CON controller */
    char controller[64];         /* "EC 24 V unit 0x02 fw 0013"-style description */
    char unit_name[48];          /* EC family name of the (system, unit) pair */
    char system_name[48];
    char firmware[8];            /* 4 hex digits */
    char serial_number[32];
    char actuator_serial[32];
    char user_name[48];          /* the user-editable unit name */
    char units[8];               /* "mm" or "deg" */
    char kind[24];               /* "slider", "rod", "gripper", "rotary"... */
    char identity_source[16];    /* "FC11" or "CTCL 0x0300" */
    uint64_t reserved[4];
} iad_identity;

/** One stored row of the position table, in the axis's units (C++ PositionEntry). Input (sized) and the output of
 *  iad_axis_read_position_table (written as "Sized structs" says). */
typedef struct iad_position_entry {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double position_mm;          /* PCMD */
    double band_mm;              /* INP (push rows: the push band); default 0.10 */
    double speed_mm_s;           /* VCMD; default 50 */
    double zone_plus_mm;         /* ZNMP */
    double zone_minus_mm;        /* ZNLP */
    double accel_g;              /* ACMD; default 0.30 */
    double decel_g;              /* DCMD; default 0.30 */
    int32_t push_power;          /* PPOW counts (255 = 100 %) */
    int32_t load_threshold;      /* LPOW */
    int32_t control_flags;       /* CTLF (push / direction / role bits) */
    int32_t checksum;            /* +0xF as read; IAD_INT_UNKNOWN for an entry built in code (ignored in inputs) */
    iad_tribool checksum_ok;     /* output: the stored checksum matches the data */
    iad_bool is_push;            /* output */
    iad_bool incremental;        /* output */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_position_entry;

/** One parameter, decoded (C++ ParameterInfo). Choices and the raw-item fields are in iad_axis_read_parameter_json. */
typedef struct iad_parameter {
    double value;                /* the number; NaN when the value is only a label */
    double min;                  /* NaN when not known */
    double max;
    int32_t number;              /* P-number / No.; 0 for a raw item */
    iad_bool writable;
    iad_bool dangerous;
    int32_t reserved0;
    char name[64];
    char symbol[16];
    char item[16];               /* "class.item", e.g. "8.15" */
    char address[16];            /* "0x0810" */
    char text[64];               /* as the PC tool shows it: "1:Forward", "disable", "12.50" */
    char unit[16];
    char raw[16];                /* hex */
    char family[8];              /* "EC" or "CON" */
    uint64_t reserved[4];
} iad_parameter;

/** Outcome of a parameter write (C++ ParameterWriteResult). Warnings and the note are in iad_axis_last_result_json. */
typedef struct iad_param_write_result {
    int32_t number;              /* P-number / No.; 0 for a raw item */
    iad_bool changed;            /* the item's raw value changed */
    iad_bool verified;           /* read back equal to what was written (1 when skipped: the unit holds it) */
    iad_bool committed;          /* PARW sent and the handshake ended idle */
    iad_bool commit_confirmed;   /* the unit was seen acting on PARW */
    iad_bool saved;              /* committed and confirmed: in non-volatile memory as far as the unit showed */
    iad_bool skipped;            /* non-volatile guard: unchanged, nothing written or committed */
    iad_bool forced;             /* unchanged but written anyway (force) */
    iad_bool listed_for_model;
    iad_bool verified_for_model; /* kept for compatibility: true when an IA-OS list is recorded for the unit code */
    iad_bool home_parameter;
    int32_t warning_count;
    char key[16];                /* "8.9" */
    char address[16];            /* "0x0810" */
    char name[64];
    char before[64];             /* as the PC tool shows it */
    char after[64];
    char unit[16];
    uint64_t reserved[4];
} iad_param_write_result;

/** Outcome of a position-table write or a teach (C++ PositionWriteResult). (0.8) The row read back after the write
 *  (when skipped, what the unit holds) is in fields of its own, those of iad_position_entry: no input struct is
 *  embedded in an output. */
typedef struct iad_position_write_result {
    double position_mm;          /* the row read back: PCMD */
    double band_mm;              /* INP */
    double speed_mm_s;           /* VCMD */
    double zone_plus_mm;         /* ZNMP */
    double zone_minus_mm;        /* ZNLP */
    double accel_g;              /* ACMD */
    double decel_g;              /* DCMD */
    int32_t push_power;          /* PPOW counts (255 = 100 %) */
    int32_t load_threshold;      /* LPOW */
    int32_t control_flags;       /* CTLF */
    int32_t checksum;            /* +0xF as read */
    iad_tribool checksum_ok;     /* the stored checksum matches the data */
    iad_bool is_push;
    iad_bool incremental;
    int32_t row;
    iad_bool skipped;            /* non-volatile guard: the row already held these 16 words */
    iad_bool forced;
    int32_t warning_count;
    int32_t reserved0;
    uint64_t reserved[4];
} iad_position_write_result;

/** Outcome of a software reset (C++ SoftwareResetResult). Notes are in iad_axis_last_result_json. */
typedef struct iad_reset_result {
    iad_status status;           /* the first status after the reset */
    double uptime_before_s;      /* NaN when not read */
    double uptime_after_s;
    double elapsed_s;
    iad_bool accepted;           /* answered the reset with rejection 0 */
    iad_bool reply_lost;         /* no (complete) answer; the reset was confirmed from the status */
    iad_bool confirmed;          /* the restart was seen */
    int32_t baud_rate;           /* the rate the unit answered at afterwards; 0 on a transport without one */
    iad_bool baud_changed;
    iad_bool needs_home;         /* HEND is 0 after the reset: home (after an alarm reset if an alarm is active) */
    char confirmed_by[64];       /* how, in words */
    uint64_t reserved[4];
} iad_reset_result;

/** Outcome of a baud change (C++ BaudChangeResult). The change is volatile: until the unit is powered off. */
typedef struct iad_baud_result {
    int32_t previous;            /* the rate before */
    int32_t baud_rate;           /* the rate the port is at now and the unit answered at */
    int32_t method;              /* IAD_BAUD_* */
    int32_t code;                /* IAI baud code sent */
    iad_bool verified;           /* the first read at the new rate was answered */
    iad_bool probed;             /* that read failed and the rate was found by probing instead */
    iad_bool code_verified;      /* the rate's code is verified on hardware on at least some units */
    int32_t reserved0;
    char frame[64];              /* the broadcast frame sent, hex */
    uint64_t reserved[4];
} iad_baud_result;

/** One axis's outcome of a group command. */
typedef struct iad_cell_result {
    iad_status status;           /* valid when code is IAD_OK and status.raw_words > 0 (a refused move_all
                                    leaves the axes that did not fail themselves at IAD_OK with raw_words 0) */
    iad_result code;             /* IAD_OK or that axis's error kind */
    int32_t index;               /* (0.8) the axis's position in the requested list (duplicates dropped; for all axes,
                                    the cell order, as iad_cell_axis_name counts): the key when `axis` was cut */
    char axis[IAD_AXIS_NAME_SIZE]; /* "COM3:1" */
    char message[IAD_MESSAGE_SIZE]; /* that axis's error message ("" when IAD_OK) */
    uint64_t reserved[4];
} iad_cell_result;

/* -------------------------------------------------------- structs: motion, monitors, events, recovery, self-test */

/* (0.2) The inputs and outputs of the push, the sequence, the move handles, the status monitor and its event queue,
 * fast mode, recovery and the self-test, of the cell's move_each, and the motion caps. */

/** Settings of a push (C++ PushOptions). */
typedef struct iad_push_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double speed;                /* approach speed (units/s); default 10 (the push itself runs at the push speed:
                                    min(PSHV 8.15, MXPH 7.8), 20 when neither is readable) */
    double accel_g;              /* default 0.1 */
    double timeout_s;            /* NaN (default) = 2 x the approach estimate + 2 x band / push speed + 3 s */
    int32_t direction;           /* +1 or 0 (default) pushes away from home (increasing position), -1 toward home */
    iad_bool wait;               /* default 1; 0 = return right after the command */
    uint64_t reserved[4];
} iad_push_options;

/** Outcome of a push (C++ PushResult). The warnings are in iad_axis_last_result_json. */
typedef struct iad_push_result {
    iad_status status;
    double position_mm;
    int32_t outcome;             /* IAD_PUSH_CONTACT or IAD_PUSH_MISSED */
    int32_t warning_count;       /* e.g. a force below the unit's recommended minimum (allowed) */
    char outcome_name[16];       /* "contact" / "missed" */
    uint64_t reserved[4];
} iad_push_result;

/** One point of a sequence (C++ SequencePoint). (0.8) Sized, and without an _init function: zero it, set struct_size and
 *  position; NaN or 0 in a double means the sequence's value. */
typedef struct iad_sequence_point {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double position;
    double speed;                /* NaN or 0 = the sequence's */
    double accel_g;              /* NaN or 0 = the sequence's */
    double band;                 /* NaN or 0 = the sequence's */
    uint64_t reserved[2];
} iad_sequence_point;

/** Settings of a sequence (C++ SequenceOptions). */
typedef struct iad_sequence_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double speed;                /* default 50 */
    double accel_g;              /* default 0.1 */
    double band;                 /* default 0.1 */
    double blend_band;           /* [EXPERIMENTAL] NaN (default): every point completes before the next is sent;
                                    0.01..10 units: the next target is sent once within blend_band of the current
                                    one (UNVERIFIED) */
    double timeout_s;            /* per point; NaN = 2 x the move's estimate + 2 s */
    int32_t completion;          /* IAD_COMPLETION_DEFAULT (the axis option), _SAFE or _FAST */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_sequence_options;

/** What a move handle knows (C++ MoveHandle; no I/O). */
typedef struct iad_move_info {
    iad_status result;           /* the final status: valid when has_result */
    double target;               /* rounded to 0.001 */
    double band;
    double timeout_s;
    double sent_at_s;            /* monotonic seconds right after the command was answered */
    double deadline_s;           /* sent_at_s + timeout_s */
    int32_t completion;          /* IAD_COMPLETION_SAFE or IAD_COMPLETION_FAST */
    iad_bool has_result;
    iad_bool cancelled;
    int32_t reserved0;
    uint64_t reserved[4];
} iad_move_info;

/** Settings of the status monitor (C++ MonitorOptions) and of the handle's event queue. */
typedef struct iad_monitor_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double gap_s;                /* pause between two polls, port lock free: 0..10 s (0 back to back); default 0.002 */
    double link_check_s;         /* the link check (with a recovery session) at most this often: 0..3600 s, 0 = never;
                                    NaN = 1 s (the default) */
    int32_t retries;             /* per poll; default 0 (one attempt, then the lock is released) */
    int32_t events;              /* IAD_EVENTS_* the handle queues; default IAD_EVENTS_ALL */
    int32_t queue_capacity;      /* events kept, 1..65536; the oldest are dropped beyond it (counted); default 1024 */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_monitor_options;

/** The monitor's last published poll (C++ MonitorSample) and its counters. */
typedef struct iad_monitor_sample {
    iad_status status;           /* the lean status (8 words: no velocity while moving); raw_words 0 before a poll */
    double t_start_s;            /* monotonic seconds just before the request went out */
    double t_s;                  /* monotonic seconds after the answer was decoded */
    double age_s;                /* seconds since t_s; NaN before the first poll */
    double poll_rate_hz;         /* answered polls per second since the start; NaN when not known */
    int64_t seq;                 /* the sample's sequence number; 0 before the first poll */
    int64_t polls;               /* answered polls */
    int64_t error_count;         /* transient errors in total */
    iad_bool running;
    int32_t reserved0;
    uint64_t reserved[4];
} iad_monitor_sample;

/** One queued event (iad_axis_next_event, iad_cell_next_event). */
typedef struct iad_event {
    iad_status status;           /* IAD_EVENT_STATUS / IAD_EVENT_DONE: the sample; otherwise raw_words 0 */
    double time_s;               /* monotonic seconds when it was queued (the clock of iad_monitor_sample) */
    double wall_time_s;          /* seconds since the Unix epoch (a link event: the core's own time stamp) */
    int64_t sequence;            /* events queued on this handle so far, this one included: 1, 2, ... */
    int32_t kind;                /* IAD_EVENT_* (IAD_EVENT_NONE: the timeout passed; nothing else is meaningful) */
    int32_t dropped;             /* events dropped (queue full) since the previous event was returned */
    char axis[IAD_AXIS_NAME_SIZE];
    char name[IAD_STATE_SIZE];   /* see IAD_EVENT_* */
    char cause[IAD_STATE_SIZE];  /* a link event: the link state behind it; a self-test line: its result */
    char message[IAD_MESSAGE_SIZE];
    int32_t source;              /* (0.6) IAD_EVENT_SOURCE_*: what queued it; 0 = not stated */
    int32_t index;               /* (0.8) a cell's event: the axis's position in the cell (iad_cell_axis_name), the key
                                    when `axis` was cut; IAD_INT_UNKNOWN on an axis's queue or for no axis of the
                                    cell */
    uint64_t caller_thread;      /* (0.7) the OS id of the thread that queued it (GetCurrentThreadId, gettid,
                                    pthread_threadid_np: Python's threading.get_native_id()); 0 = not stated. A link
                                    event of a call is queued on the thread that made the call. */
    uint64_t reserved[4];        /* (0.8) 32 bytes (was 16) */
} iad_event;

/** What iad_axis_enable_fast / iad_axis_disable_fast did (C++ FastModeSummary). */
typedef struct iad_fast_summary {
    int32_t baud_rate;           /* the port's rate afterwards; 0 on a transport without one */
    int32_t completion;          /* IAD_COMPLETION_SAFE or IAD_COMPLETION_FAST */
    iad_bool monitor;            /* a status monitor runs */
    int32_t note_count;
    char notes[IAD_MESSAGE_SIZE];/* what could not be done (the baud rate kept...), joined with "; " */
    uint64_t reserved[4];
} iad_fast_summary;

/** Settings of iad_axis_track (C++ TrackOptions; the name, the rows and a backup are arguments). */
typedef struct iad_track_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    iad_bool alarm_history;      /* keep the history to spot a new power-up record (192 words); default 1 */
    iad_bool parameters;         /* also copy the parameter classes 7, 8, 9, 0x0C; default 0 */
    uint64_t reserved[4];
} iad_track_options;

/** Result of a link check (C++ recovery::LinkCheck). */
typedef struct iad_link_check {
    iad_status status;           /* the status read; raw_words 0 when the unit did not answer */
    double elapsed_s;
    iad_bool ok;                 /* state is "ok" */
    int32_t reason_count;
    char state[IAD_STATE_SIZE];  /* "ok", "port_vanished", "unit_silent", "unit_restarted", "recovering" ("error":
                                    the check itself failed, iad_cell_check_links only) */
    char reasons[IAD_MESSAGE_SIZE]; /* the restart signs, joined with "; " */
    char error[IAD_MESSAGE_SIZE];   /* "<Type>: <message>" when the unit did not answer */
    uint64_t reserved[4];
} iad_link_check;

/** What a recovery restores and checks (C++ recovery::RecoveryPolicy; the rows and a backup are arguments). It never
 *  starts motion. */
typedef struct iad_recovery_policy {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double position_tolerance_mm;  /* |position - last target| that still counts as at the target; default 0.1 */
    double poll_s;                 /* how often to look for the port / the unit while waiting; default 0.5 */
    int32_t boot_baud_rate;        /* the power-up rate probed first after a power loss; default 38400 */
    iad_bool restore_baud_rate;    /* default 1 */
    iad_bool restore_control;      /* take PC control (PMSL) again when the application held it; default 1 */
    iad_bool restore_safety_speed; /* default 1 */
    iad_bool clear_message_alarms; /* reset a message-level alarm (others are left for the operator); default 1 */
    iad_bool check_parameters;     /* compare the parameter classes with the session's copy / the backup; default 0 */
    iad_bool require_session;      /* (0.6) iad_axis_recover without a session (none tracked): IAD_ERR_RECOVERY instead
                                      of the identity the axis read last as the baseline; default 0 */
    int32_t reserved0;
    uint64_t reserved[3];
} iad_recovery_policy;

/** What a recovery found and did (C++ recovery::RecoveryReport). The full report (restart signs, actions, notes, row
 *  and parameter differences, new alarm records) is in iad_axis_last_result_json. */
typedef struct iad_recovery_summary {
    iad_status status;           /* the last status read; raw_words 0 when none */
    double position_mm;          /* NaN when not known */
    double last_target_mm;
    double position_error_mm;
    double elapsed_s;
    int32_t baud_rate;           /* the rate in use afterwards; 0 when not known */
    int32_t baud_rate_found;     /* the rate the unit was found at; 0 when not searched */
    int32_t alarm_code;
    iad_bool restarted;
    iad_bool same_unit;
    iad_bool port_reopened;
    iad_bool baud_restored;
    iad_bool control_restored;
    iad_bool safety_speed_restored;
    iad_bool needs_home;         /* HEND lost: home before the next move */
    iad_bool needs_attention;    /* anything to decide before the next move */
    iad_bool ready_to_move;      /* servo ON, homed, PC control, no alarm, no E-stop */
    iad_bool fault_alarm;        /* an alarm that blocks motion is still active */
    iad_tribool at_target;
    iad_tribool servo_on;
    iad_tribool homed;
    iad_tribool modbus_control;
    iad_tribool safety_speed;
    int32_t note_count;
    iad_bool control_kept;       /* (0.6) PC control was held before the fault and still is: nothing written */
    char axis[IAD_AXIS_NAME_SIZE];
    char cause[IAD_STATE_SIZE];  /* the link state the recovery started from */
    char summary[IAD_MESSAGE_SIZE]; /* "G: recovered from unit_silent, unit restarted, needs home" */
    uint64_t reserved[4];
} iad_recovery_summary;

/** Settings of iad_axis_self_test (C++ CheckOptions). */
typedef struct iad_check_options {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double step_mm;              /* the move with motion (at most a quarter of the stroke, at least 0.1); default 5 */
    double speed_mm_s;           /* default 10 */
    double accel_g;              /* default 0.1 */
    iad_bool motion;             /* also a short move out and back; default 0 (read-only) */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_check_options;

/** Summary of a self-test (C++ CheckReport). The JSON report (the Python edition's layout) is in
 *  iad_axis_last_result_json. */
typedef struct iad_check_summary {
    iad_bool passed;             /* no line failed (WARN and SKIP do not fail the check) */
    int32_t items;
    int32_t failed;
    int32_t warned;
    int32_t skipped;
    int32_t reserved0;
    uint64_t reserved[4];
} iad_check_summary;

/** One axis's move of iad_cell_move_each (C++ AxisMove). Sized, with fields of its own: zero it, set struct_size,
 *  position and axis; NaN or 0 in a double and 0 in an enum field mean the default named (iad_move_options_init's).
 *  The call's wait applies to every axis. */
typedef struct iad_axis_move {
    uint32_t struct_size;        /* (0.8) sized: see "Sized structs" above */
    uint32_t reserved_header;    /* (0.8) 0 */
    double position;             /* the absolute target (units) */
    double speed;                /* units/s; NaN or 0 = 50 */
    double accel_g;              /* G (= decel on a 0x9900 move); NaN or 0 = 0.1 */
    double band;                 /* positioning band (units); NaN or 0 = 0.1 */
    double timeout_s;            /* this axis's wait when the call's timeout_s is NaN; NaN or 0 = 2 x the move's
                                    estimate + 2 s */
    double decel_g;              /* a deceleration other than accel_g (the command-profile path); NaN or 0 = accel_g */
    int32_t accel_pattern;       /* IAD_ACCEL_DEFAULT (0: trapezoid), _TRAPEZOID, _S_MOTION, _FILTER */
    int32_t completion;          /* IAD_COMPLETION_DEFAULT (0: the axis option), _SAFE or _FAST */
    char axis[IAD_AXIS_NAME_SIZE]; /* "COM3:1"; a name that does not fit (with its NUL) is IAD_ERR_INVALID_ARGUMENT */
    uint64_t reserved[4];
} iad_axis_move;

/** The caps the driver checks moves against and the documented minimum speed (Axis::speed_cap, accel_cap,
 *  accel_only_cap, decel_cap, push_cap, recommended_min_speed), in the axis's units. */
typedef struct iad_motion_caps {
    double speed_cap;            /* units/s: min(max_speed_mm_s, MAXV 7.4), each only when known */
    double accel_cap;            /* G: acceleration = deceleration of a 0x9900 move: min(max_accel_g, MAXA, MAXD) */
    double accel_only_cap;       /* G: acceleration of a move with its own deceleration: min(max_accel_g, MAXA) */
    double decel_cap;            /* G: deceleration of such a move: min(max_accel_g, MAXD) */
    double push_cap;             /* %: min(max_push_pct, MXPP 7.9) */
    double min_speed;            /* units/s: the documented minimum speed (a warning below it, never a refusal); NaN
                                    when nothing tells */
    char min_speed_source[IAD_MESSAGE_SIZE]; /* where min_speed comes from, or why it is unknown */
    uint64_t reserved[4];
} iad_motion_caps;

/* ------------------------------------------------------------------------------------ structs: transports, buses */

/* (0.3) A binding's own transport and the settings of a bus. */

/** A binding's own transport (C++ Transport through callbacks). Not plain data: it holds function pointers, so its size
 *  differs between 32-bit and 64-bit builds; never embedded in another struct. write and read are required, the rest
 *  optional (NULL): without reset_input nothing is discarded; without set_baud_rate the transport has no baud rate
 *  (baud_rate is then ignored); without send_break, present or reopen those are unsupported / unknown; without
 *  set_read_timeout (0.5) only read's timeout_s carries the timeout. (0.5) set_read_timeout is appended: a caller built
 *  with the 0.3 / 0.4 header passes the 0.3 struct_size and has none. */
typedef struct iad_transport_callbacks {
    uint32_t struct_size;        /* sizeof(iad_transport_callbacks) of the caller (iad_transport_callbacks_init) */
    uint32_t reserved_header;    /* 0 */
    double read_timeout_s;       /* the first per-read timeout; default 0.2 (the bus sets its own per request) */
    int32_t baud_rate;           /* the PC side's rate now (with set_baud_rate); default 0 = none */
    iad_bool real_timeout;       /* 1 (default): read blocks up to timeout_s, as a serial port; 0: read returns at
                                    once (in-memory) and the bus polls for its own deadline */
    iad_transport_write_fn write;
    iad_transport_read_fn read;
    iad_transport_reset_input_fn reset_input;
    iad_transport_set_baud_rate_fn set_baud_rate;
    iad_transport_send_break_fn send_break;
    iad_transport_present_fn present;
    iad_transport_reopen_fn reopen;
    iad_transport_close_fn close;
    iad_transport_release_fn release;
    iad_transport_set_read_timeout_fn set_read_timeout; /* (0.5) */
} iad_transport_callbacks;

/** Settings of a bus (C++ BusOptions). */
typedef struct iad_bus_options {
    uint32_t struct_size;        /* sizeof(iad_bus_options) of the caller (iad_bus_options_init) */
    uint32_t reserved_header;    /* 0 */
    double min_gap_s;            /* minimum silence between transactions; default 0.005 */
    int32_t retries;             /* extra attempts after a timeout / CRC / mismatched frame; default 3 */
    int32_t reserved0;
    uint64_t reserved[4];
} iad_bus_options;

/* ------------------------------------------------------------------------------------------- library */

/** (IAD_ABI_VERSION_MAJOR << 16) | IAD_ABI_VERSION_MINOR of the loaded library. Compare with the header's. */
IAD_API uint32_t IAD_CALL iad_abi_version(void);

/** Version of the native core, e.g. "0.1.0" (static string). */
IAD_API const char* IAD_CALL iad_library_version(void);

/** Message of the last failed call on this thread ("" after a call that succeeded). Valid until the next call on this
 *  thread; copy it to keep it. Never NULL. */
IAD_API const char* IAD_CALL iad_last_error_message(void);

/** The last error of this thread as JSON: {"code", "name", "kind" (the C++ class), "message", "notes",
 *  "stop_attempted"} plus, when the error carries them, "alarm_code", "attempts", "exception_code", "function",
 *  "command", "rejection", "reason", "hint", "reasons" (servo blocked), "operation" and "axes" ({name: {"code",
 *  "message"}}, a group command), (0.5) "status" (IAD_ERR_COMMAND_REJECTED: the reply's status byte, 0x45, null when
 *  not known, or its four status words, 0x48, as iad_bus_iai_command returns them with IAD_IAI_NO_CHECK), (0.6)
 *  "cause" (a row the unit rejected, IAD_ERR_SOFT_LIMIT: the rejection's own JSON, IAD_ERR_COMMAND_REJECTED).
 *  {"code": 0} when the last call succeeded. Does not change the last error. */
IAD_API iad_result IAD_CALL iad_last_error_json(char* buffer, size_t buffer_size, size_t* required);

/** Name of a result code: "IAD_OK", "IAD_ERR_SOFT_LIMIT"... ("IAD_ERR_UNKNOWN" for another value). Static string. */
IAD_API const char* IAD_CALL iad_result_name(iad_result code);

/** The PC's serial ports, IAI USB adapters first: [{"port", "description", "manufacturer", "hardware_id", "vid",
 *  "pid", "is_iai"}]. Opens nothing. (0.7) A test replaces the list with iad_testing_set_port_list
 *  (iai_actuator_driver_testing.h). */
IAD_API iad_result IAD_CALL iad_list_ports_json(char* buffer, size_t buffer_size, size_t* required);

/* ------------------------------------------------------------------------------------------- option defaults */

/** Write the defaults into an options struct (IAD_ERR_INVALID_ARGUMENT for NULL). */
IAD_API iad_result IAD_CALL iad_axis_options_init(iad_axis_options* options);
IAD_API iad_result IAD_CALL iad_open_options_init(iad_open_options* options);
IAD_API iad_result IAD_CALL iad_sim_options_init(iad_sim_options* options);
IAD_API iad_result IAD_CALL iad_move_options_init(iad_move_options* options);
/** The jog defaults: iad_move_options_init with speed 10 units/s. */
IAD_API iad_result IAD_CALL iad_jog_options_init(iad_move_options* options);
IAD_API iad_result IAD_CALL iad_position_move_options_init(iad_position_move_options* options);
IAD_API iad_result IAD_CALL iad_nv_write_options_init(iad_nv_write_options* options);
IAD_API iad_result IAD_CALL iad_param_write_options_init(iad_param_write_options* options);
IAD_API iad_result IAD_CALL iad_baud_options_init(iad_baud_options* options);
IAD_API iad_result IAD_CALL iad_reset_options_init(iad_reset_options* options);
IAD_API iad_result IAD_CALL iad_cell_options_init(iad_cell_options* options);
/** The defaults of a new row (C++ PositionEntry{}): band 0.10, speed 50, accel / decel 0.30, the rest 0. */
IAD_API iad_result IAD_CALL iad_position_entry_init(iad_position_entry* entry);
/** (0.8) The options of the axis commands. */
IAD_API iad_result IAD_CALL iad_servo_options_init(iad_servo_options* options);
IAD_API iad_result IAD_CALL iad_home_options_init(iad_home_options* options);
IAD_API iad_result IAD_CALL iad_stop_options_init(iad_stop_options* options);
IAD_API iad_result IAD_CALL iad_wait_idle_options_init(iad_wait_idle_options* options);
IAD_API iad_result IAD_CALL iad_change_options_init(iad_change_options* options);
IAD_API iad_result IAD_CALL iad_cancel_paused_move_options_init(iad_cancel_paused_move_options* options);

/* ------------------------------------------------------------------------------------------- axis: session */

/** Open serial port `port` ("COM3", "/dev/ttyUSB0") once and return an axis that owns it (Axis::open). With
 *  options->baud_rate 0 the rate is found in place. IAD_ERR_MODBUS_TIMEOUT (port closed again) when no rate answers;
 *  IAD_ERR_TRANSPORT when the port is busy or absent. No identity is read: call iad_axis_connect. */
IAD_API iad_result IAD_CALL iad_axis_open(const char* port, const iad_open_options* options,
                                          const iad_axis_options* axis_options, iad_axis** out);

/** An axis on a simulated unit (Axis::open on a SimulatedUnit): no hardware, never opens a port. `profile`: "slider"
 *  (NULL), "gripper", "rod", "pcon", "rotary", "unknown" (IAD_ERR_INVALID_ARGUMENT otherwise). The unit answers at
 *  options->slave. The rate is detected only when sim->baud_rate models one and options->baud_rate is 0. */
IAD_API iad_result IAD_CALL iad_axis_open_sim(const char* profile, const iad_sim_options* sim,
                                              const iad_open_options* options, const iad_axis_options* axis_options,
                                              iad_axis** out);

/** Release a STOP / pause this axis holds, close the port when the axis owns it and release the handle
 *  (Axis::close). NULL or a closed handle: IAD_OK. A handle from iad_cell_open_axis is only released: the axis stays in
 *  the cell, unchanged. A call still running on another thread keeps the axis alive until it returns. */
IAD_API iad_result IAD_CALL iad_axis_close(iad_axis* axis);

/** Read identity and ranges and, with take_control, take Modbus control at rest (Axis::connect). */
IAD_API iad_result IAD_CALL iad_axis_connect(iad_axis* axis, iad_bool take_control,
                                             iad_identity* out /* may be NULL */);

/** Read the identity again (Axis::identify; READ-ONLY). */
IAD_API iad_result IAD_CALL iad_axis_identify(iad_axis* axis, iad_identity* out);

/** The full identity record of the last connect / identify as JSON (to_json(DeviceInfo)). No I/O; IAD_ERR_NOT_READY
 *  before the first connect / identify. The unit's whole record, with I/O, is iad_axis_read_unit_info_json. */
IAD_API iad_result IAD_CALL iad_axis_identity_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** The axis settings (Axis::options / set_options). */
IAD_API iad_result IAD_CALL iad_axis_get_options(iad_axis* axis, iad_axis_options* out);
IAD_API iad_result IAD_CALL iad_axis_set_options(iad_axis* axis, const iad_axis_options* options);

/** The port's baud rate (0 on a transport without one) and whether it was detected rather than given. */
IAD_API iad_result IAD_CALL iad_axis_baud_rate(iad_axis* axis, int32_t* rate, iad_bool* detected /* may be NULL */);

/** Take (on: PMSL ON at rest, verified by PMSS) or release (off: command authority back to PIO) Modbus control
 *  (Axis::enable_modbus_control / disable_modbus_control). */
IAD_API iad_result IAD_CALL iad_axis_modbus_control(iad_axis* axis, iad_bool on);

/* ------------------------------------------------------------------------------------- axis: reading (READ-ONLY) */

/** One status read (Axis::status). */
IAD_API iad_result IAD_CALL iad_axis_status(iad_axis* axis, iad_status* out);

/** Every range the driver enforces, read from the unit (Axis::limits). */
IAD_API iad_result IAD_CALL iad_axis_limits(iad_axis* axis, iad_limits* out);

/** The same with each range's source in words (to_json(AxisRanges)). */
IAD_API iad_result IAD_CALL iad_axis_limits_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** Alarm history, newest first: [{"index", "code", "name", "level", "detail", "address", "time_ms", ...}]
 *  (Axis::read_alarm_history). include_empty: every slot, not only up to the first empty one. */
IAD_API iad_result IAD_CALL iad_axis_read_alarm_history_json(iad_axis* axis, iad_bool include_empty, char* buffer,
                                                             size_t buffer_size, size_t* required);

/** Stored rows on this unit (Axis::max_positions: 3 on the ECs tested, 64 on the PCON). */
IAD_API iad_result IAD_CALL iad_axis_max_positions(iad_axis* axis, int32_t* count);

/** One stored row (Axis::read_position_table). */
IAD_API iad_result IAD_CALL iad_axis_read_position_table(iad_axis* axis, int32_t row, iad_position_entry* out);

/* ------------------------------------------------------------------------------------------- axis: control */

/** Servo on / off (Axis::servo). A servo ON the status shows cannot work is refused before anything is sent
 *  (IAD_ERR_SERVO_BLOCKED). NULL options: iad_servo_options_init's (wait, the core's timeout). `out` is written only
 *  when options->wait. */
IAD_API iad_result IAD_CALL iad_axis_servo(iad_axis* axis, iad_bool on, const iad_servo_options* options,
                                           iad_status* out /* may be NULL */);

/** Home return (Axis::home); waits for HEND and standstill when options->wait (NULL options: iad_home_options_init's:
 *  wait, the core's timeout, the axis's method). `out` is written only when it waits. */
IAD_API iad_result IAD_CALL iad_axis_home(iad_axis* axis, const iad_home_options* options,
                                          iad_status* out /* may be NULL */);

/** Clear the current alarm (Axis::reset_alarm); `out` gets the new status. */
IAD_API iad_result IAD_CALL iad_axis_reset_alarm(iad_axis* axis, int32_t method, iad_status* out /* may be NULL */);

/* ------------------------------------------------------------------------------------------- axis: motion */

/** Every check of iad_axis_move_to, nothing sent (Axis::check_move); the errors move_to would give. */
IAD_API iad_result IAD_CALL iad_axis_check_move(iad_axis* axis, double position, const iad_move_options* options,
                                                iad_status* out /* may be NULL */);

/** Absolute move (Axis::move_to, the 0x9900 direct block). Checked against the unit's ranges before anything is sent
 *  (IAD_ERR_SOFT_LIMIT). With options->wait, blocks until in position and still and writes the final status to
 *  `out`; otherwise returns right after the command and leaves `out` unwritten. */
IAD_API iad_result IAD_CALL iad_axis_move_to(iad_axis* axis, double position, const iad_move_options* options,
                                             iad_status* out /* may be NULL */);

/** Relative to the measured position, sent as an absolute target (Axis::move_by). As iad_axis_move_to. */
IAD_API iad_result IAD_CALL iad_axis_move_by(iad_axis* axis, double delta, const iad_move_options* options,
                                             iad_status* out /* may be NULL */);

/** A small relative step (Axis::jog; limits jog_step). NULL options = the jog defaults (10 units/s). */
IAD_API iad_result IAD_CALL iad_axis_jog(iad_axis* axis, double delta, const iad_move_options* options,
                                         iad_status* out /* may be NULL */);

/** Run stored row `row` (Axis::move_to_position; the row is read and checked first). `out` is written only when
 *  options->wait. Position mode: an EC runs only the rows of its position mode (P9, read once and again after a commit
 *  or a software reset): rows 0 and 1 in 2-position mode (P9 0 / 3), rows 0-2 in 3-position mode. Another row is
 *  refused with IAD_ERR_SOFT_LIMIT before anything is sent (the EC-GRBP8M gripper in 2-position mode rejected row 2
 *  with 0x45 cmd 6 code 0x11; VERIFIED in the Python edition). When P9 is not readable or has another value the row
 *  is sent, and a 0x11 rejection of cmd 6 (options->speed_pct) comes back as the same IAD_ERR_SOFT_LIMIT (on a CON
 *  unit too, which has no position mode). */
IAD_API iad_result IAD_CALL iad_axis_move_to_position(iad_axis* axis, int32_t row,
                                                      const iad_position_move_options* options,
                                                      iad_status* out /* may be NULL */);

/** Change the speed of the move in progress (Axis::change_speed). Refused (nothing sent) without a move of this
 *  axis running, while paused, or outside the limits. NULL options: iad_change_options_init's (no wait). `out` is
 *  written only when options->wait. */
IAD_API iad_result IAD_CALL iad_axis_change_speed(iad_axis* axis, double speed, const iad_change_options* options,
                                                  iad_status* out /* may be NULL */);

/** Block until the move is done by a completion rule (Axis::wait_idle), at options->target within options->band when
 *  the target is not NaN (NULL options: iad_wait_idle_options_init's: no target, 10 s, the axis's rule). The rules:
 *  IAD_COMPLETION_DEFAULT = the axis option (iad_axis_options.completion), _SAFE (PEND, within the band, PNOW
 *  unchanged over settle_s) or _FAST (one lean sample: PEND, within the band, DSSE MOVE clear; the status monitor's
 *  samples when it runs). */
IAD_API iad_result IAD_CALL iad_axis_wait_idle(iad_axis* axis, const iad_wait_idle_options* options,
                                               iad_status* out /* may be NULL */);

/** Decel stop (Axis::stop), with or without Modbus control. Blocks for the deceleration; with options->wait,
 *  IAD_ERR_MOTION_TIMEOUT when still moving after options->timeout_s (NULL options: iad_stop_options_init's: no wait,
 *  5 s). May be called from another thread while a move waits on this handle. Not a safety function: it goes over a
 *  serial line from a general-purpose PC and can be late, so a hardware emergency stop or STO is required wherever a
 *  late stop could hurt someone. */
IAD_API iad_result IAD_CALL iad_axis_stop(iad_axis* axis, const iad_stop_options* options,
                                          iad_status* out /* may be NULL */);

/** Hold the move and keep the remaining travel (Axis::pause, STP ON). */
IAD_API iad_result IAD_CALL iad_axis_pause(iad_axis* axis);

/** Release the pause; a paused move continues by itself (Axis::resume, STP OFF). */
IAD_API iad_result IAD_CALL iad_axis_resume(iad_axis* axis);

/** Drop the remaining travel of a paused move and release the pause (Axis::cancel_paused_move). Refused (nothing
 *  sent) without an active pause (IAD_ERR_NOT_READY) or with a fault alarm (IAD_ERR_ALARM). NULL options:
 *  iad_cancel_paused_move_options_init's (5 s). */
IAD_API iad_result IAD_CALL iad_axis_cancel_paused_move(iad_axis* axis, const iad_cancel_paused_move_options* options,
                                                        iad_status* out /* may be NULL */);

/* ------------------------------------------------------------------------------------------- axis: position table */

/** Write stored row `row` (16 words with the checksum) and verify it (Axis::write_position_table). Non-volatile; at
 *  rest only. When the row already holds these words nothing is written (out->skipped) unless options->force. The
 *  simple-data rows 256-267 and a row the table cannot address: IAD_ERR_NONVOLATILE_REFUSED, nothing written. A row
 *  past the unit's stored rows, on an EC a row its position mode (P9) does not run (row 2 in 2-position mode), an
 *  UNVERIFIED layout, a middle-row push, unknown flag bits or changed role bits are written with a warning
 *  (out->warning_count; the texts in iad_axis_last_result_json). See iad_axis_move_to_position. */
IAD_API iad_result IAD_CALL iad_axis_write_position_table(iad_axis* axis, int32_t row,
                                                          const iad_position_entry* entry,
                                                          const iad_nv_write_options* options,
                                                          iad_position_write_result* out /* may be NULL */);

/** Store the current position as row `row` (Axis::teach_position). Non-volatile, as iad_axis_write_position_table;
 *  refused before homing and while moving. A row outside the unit's stored rows: IAD_ERR_SOFT_LIMIT. Position mode: on
 *  an EC a row its position mode (P9) does not run is taught with a warning (as iad_axis_write_position_table).
 *  `method`: IAD_TEACH_TABLE (0, the default: read the row, replace the position, write it back) or IAD_TEACH_COIL
 *  ([EXPERIMENTAL]; needs coils); same refusals and warnings. */
IAD_API iad_result IAD_CALL iad_axis_teach_position(iad_axis* axis, int32_t row, int32_t method,
                                                    const iad_nv_write_options* options,
                                                    iad_position_write_result* out /* may be NULL */);

/* ------------------------------------------------------------------------------------------- axis: parameters */

/* (0.7) The row of a named parameter (iad_axis_parameters_json, iad_axis_read_parameter_json) has the keys of the
 * Python edition's parameters(): no unsigned, signed, words or named, and a symbol or a note only when it has one; a
 * raw item keeps every field. */

/** The unit's named parameters, decoded (Axis::parameters; EC: P1-P18, CON: No.x; READ-ONLY):
 *  [to_json(ParameterInfo)...]. */
IAD_API iad_result IAD_CALL iad_axis_parameters_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                     size_t* required);

/** One parameter (Axis::read_parameter; READ-ONLY). `key`: a number ("2", "P2", "No.35") or a raw item ("8.15"). */
IAD_API iad_result IAD_CALL iad_axis_read_parameter(iad_axis* axis, const char* key, iad_parameter* out);

/** The same as JSON, with the choices and the raw-item fields. */
IAD_API iad_result IAD_CALL iad_axis_read_parameter_json(iad_axis* axis, const char* key, char* buffer,
                                                         size_t buffer_size, size_t* required);

/** Write one parameter, verify it by read-back and (options->commit) save it with PARW (Axis::write_parameter).
 *  `value` as the PC tool shows it: a number ("12.5") or a label ("1:Forward", "disable"). Guards refuse before
 *  anything is written (IAD_ERR_UNSAFE_REFUSED for a read-only or not-offered parameter, IAD_ERR_SOFT_LIMIT); a home,
 *  dangerous or unlisted parameter is written with a warning; at rest only. Non-volatile guard: an unchanged
 *  value is not written or committed (out->skipped) unless options->force. Applies after iad_axis_software_reset or a
 *  power cycle. */
IAD_API iad_result IAD_CALL iad_axis_write_parameter(iad_axis* axis, const char* key, const char* value,
                                                     const iad_param_write_options* options,
                                                     iad_param_write_result* out /* may be NULL */);

/** The same with a numeric value. */
IAD_API iad_result IAD_CALL iad_axis_write_parameter_number(iad_axis* axis, const char* key, double value,
                                                            const iad_param_write_options* options,
                                                            iad_param_write_result* out /* may be NULL */);

/* -------------------------------------------------------------------------------------- axis: non-volatile guard */

/** Non-volatile write cycles this axis handle caused (parameter commits + position-table writes), those of the last
 *  hour, and whether a write_parameter without commit left a value unsaved (Axis::nonvolatile_writes,
 *  nonvolatile_writes_last_hour, has_uncommitted_parameters). */
IAD_API iad_result IAD_CALL iad_axis_nonvolatile_writes(iad_axis* axis, int64_t* total,
                                                        int32_t* last_hour /* may be NULL */,
                                                        iad_bool* uncommitted /* may be NULL */);

/* (0.8) The guard's test hooks (iad_testing_axis_nv_record, iad_testing_axis_nv_last_hour) are in
 * iai_actuator_driver_testing.h; its bookkeeping shared per unit (iad_axis_set_nv_unit, iad_nv_forget) is in "axis:
 * state without I/O". */

/* ------------------------------------------------------------------------------------------- axis: unit */

/** Change the unit's and the port's baud rate (Axis::set_baud_rate; volatile: until power-off). At rest only, and
 *  only when this axis is the only one on the bus. IAD_ERR_SOFT_LIMIT (nothing sent) for a rate without an IAI code
 *  (an UNVERIFIED code is sent with a logged warning, out->code_verified 0);
 *  IAD_ERR_NOT_READY (nothing sent) on a transport without a baud rate, on a shared port or when not at rest;
 *  IAD_ERR_VERIFICATION when the unit answers at another rate afterwards. */
IAD_API iad_result IAD_CALL iad_axis_set_baud_rate(iad_axis* axis, int32_t rate, const iad_baud_options* options,
                                                   iad_baud_result* out /* may be NULL */);

/** Restart the unit's firmware and wait until it answers again (Axis::software_reset; at rest only, servo off first).
 *  Committed parameters apply afterwards. */
IAD_API iad_result IAD_CALL iad_axis_software_reset(iad_axis* axis, const iad_reset_options* options,
                                                    iad_reset_result* out /* may be NULL */);

/** The full result (warnings, notes) of the last successful iad_axis_write_parameter(_number),
 *  iad_axis_write_position_table, iad_axis_teach_position, iad_axis_set_baud_rate or iad_axis_software_reset on this
 *  handle, as JSON (to_json of the C++ result). No I/O; IAD_ERR_NOT_READY when there is none. 0.2: also of
 *  iad_axis_push (with wait), iad_axis_enable_fast / disable_fast, iad_axis_check_link, iad_axis_recover,
 *  iad_axis_self_test, and the JSON of iad_axis_read_unit_info_json / iad_axis_backup_json (kept even when their buffer
 *  was too small). */
IAD_API iad_result IAD_CALL iad_axis_last_result_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                      size_t* required);

/* ------------------------------------------------------------------------------------------- cell */

/* Axis lists are one UTF-8 string of axis names separated by commas ("COM3:1,COM4:1"); NULL or "" = every axis of the
 * cell. Group commands handle every listed axis before returning (C++ return_exceptions semantics): IAD_OK when every
 * axis succeeded, IAD_ERR_CELL when one or more failed. Per-axis outcomes go to `results` (may be NULL): at most
 * `capacity` entries in the order of the list (cell order for all), and *count (may be NULL) gets the number of axes
 * handled. A results array that is too small never stops a command: it is filled as far as it goes. */

/** Open a cell (Cell::open). `ports`: NULL or "" = every IAI USB adapter of the PC, scanned; "COM3,COM4" = these
 *  ports, scanned (options scan_first .. scan_last); "COM3:1,COM3:2,COM4:1" = exactly these slaves (each must answer).
 *  Either every entry names a slave or none does. Ports that cannot be opened or where nothing answers are skipped
 *  when scanning (iad_cell_info_json "port_errors"); IAD_ERR_NOT_READY when no axis at all is found. */
IAD_API iad_result IAD_CALL iad_cell_open(const char* ports, const iad_cell_options* options,
                                          const iad_axis_options* axis_options, iad_cell** out);

/** A cell on simulated lines: one line "SIM1".."SIMn" per entry of `profiles` ("gripper,pcon"; NULL or "" = one
 *  "slider" line), each with `slaves_per_line` units (1..16) of that profile at addresses 1..n. Axis names
 *  "SIM1:1"... */
IAD_API iad_result IAD_CALL iad_cell_open_sim(const char* profiles, int32_t slaves_per_line,
                                              const iad_sim_options* sim, const iad_cell_options* options,
                                              const iad_axis_options* axis_options, iad_cell** out);

/** Release every STOP / pause the axes hold, close every port and release the handle (Cell::close). NULL or a
 *  closed handle: IAD_OK. Axis handles from iad_cell_open_axis stop working (IAD_ERR_INVALID_ARGUMENT; closing them
 *  stays safe). */
IAD_API iad_result IAD_CALL iad_cell_close(iad_cell* cell);

/** Number of axes in the cell. */
IAD_API iad_result IAD_CALL iad_cell_axis_count(iad_cell* cell, size_t* count);

/** Name of axis `index` (0-based, port order then slave order), e.g. "COM3:1". Text output: the buffer rules of the
 *  JSON outputs. */
IAD_API iad_result IAD_CALL iad_cell_axis_name(iad_cell* cell, size_t index, char* buffer, size_t buffer_size,
                                               size_t* required);

/** Open an axis handle for one axis of the cell (every iad_axis_* function works on it). Release it with
 *  iad_axis_close (which leaves the axis in the cell); it stops working when the cell is closed. */
IAD_API iad_result IAD_CALL iad_cell_open_axis(iad_cell* cell, const char* name, iad_axis** out);

/** The cell as JSON (no I/O): {"axes": [names], "info": {name: identity}, "port_errors": {port: message},
 *  "faults": {name: "port_vanished" | "unit_silent"}}. */
IAD_API iad_result IAD_CALL iad_cell_info_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required);

/** Forget a fault of one axis (Cell::clear_fault). */
IAD_API iad_result IAD_CALL iad_cell_clear_fault(iad_cell* cell, const char* axis);

/** One status read per axis (Cell::status_all; ports in parallel). An axis whose status monitor has a sample younger
 *  than monitor_max_age_s is not read (its lean sample is returned); NaN: always read. */
IAD_API iad_result IAD_CALL iad_cell_status_all(iad_cell* cell, const char* axes, double monitor_max_age_s,
                                                iad_cell_result* results, size_t capacity, size_t* count);

/** Servo on / off on every listed axis (Cell::servo_all). timeout_s NaN = the core's default. */
IAD_API iad_result IAD_CALL iad_cell_servo_all(iad_cell* cell, iad_bool on, const char* axes, double timeout_s,
                                               iad_cell_result* results, size_t capacity, size_t* count);

/** Home every listed axis at once and wait for all (Cell::home_all). timeout_s NaN = the core's default. `method`:
 *  the home command's path (IAD_METHOD_DEFAULT: each axis's option). stop_others_on_error: a failure on one axis stops
 *  the others. */
IAD_API iad_result IAD_CALL iad_cell_home_all(iad_cell* cell, const char* axes, double timeout_s, int32_t method,
                                              iad_bool stop_others_on_error, iad_cell_result* results,
                                              size_t capacity, size_t* count);

/** Start an absolute move on each listed axis to positions[i] (the i-th name of `axes`, which is required here;
 *  position_count must equal the number of names) with one set of options, with as little delay as possible
 *  (Cell::move_all). Every move is checked first: one bad value refuses the whole move, nothing sent (IAD_ERR_CELL
 *  with the per-axis codes). A start that fails stops the axes already started. With options->wait the call waits
 *  for all; otherwise it returns once every move started, *count is 0 and `results` is not written: call
 *  iad_cell_wait_all. Per-axis values (Cell::move_all with PerAxisMoveValues, the Python edition's mapping form of
 *  speed / accel / band / decel): `speeds`, `accels_g`, `bands` and `decels_g` (each may be NULL) hold position_count
 *  values in the order of `axes`, each in that axis's own units: NULL = options' value for every axis, a NaN entry =
 *  options' value for that axis (decel: options->decel_g, else that axis's accel). An axis whose decel differs from
 *  its accel runs the command-profile path. */
IAD_API iad_result IAD_CALL iad_cell_move_all(iad_cell* cell, const char* axes, const double* positions,
                                              size_t position_count, const iad_move_options* options,
                                              const double* speeds, const double* accels_g, const double* bands,
                                              const double* decels_g, iad_cell_result* results, size_t capacity,
                                              size_t* count);

/** Wait until every listed axis is in position and settled (Cell::wait_all; an axis started by iad_cell_move_all: at
 *  its target and band). timeout_s NaN = per axis from the move's estimate, 10 s otherwise. stop_others_on_error: a
 *  failure on one axis stops the others. */
IAD_API iad_result IAD_CALL iad_cell_wait_all(iad_cell* cell, const char* axes, double timeout_s,
                                              iad_bool stop_others_on_error, iad_cell_result* results,
                                              size_t capacity, size_t* count);

/** Decel stop on every listed axis as fast as possible (Cell::stop_all: STOP to all first, then each waits).
 *  timeout_s NaN = 5 s. `broadcast`: on a daisy chain whose axes are all selected one broadcast STOP goes out first
 *  (never behind a gateway; UNVERIFIED on hardware), then each axis's stop, then the broadcast release. May be called
 *  from another thread while a group command waits. A listed name the cell does not have never holds back the STOP:
 *  every listed axis it has is stopped, then the call returns IAD_ERR_CELL with each unknown name a result of its own
 *  (IAD_ERR_INVALID_ARGUMENT), after the axes; *count counts both. */
IAD_API iad_result IAD_CALL iad_cell_stop_all(iad_cell* cell, const char* axes, double timeout_s, iad_bool broadcast,
                                              iad_cell_result* results, size_t capacity, size_t* count);

/* ---------------------------------------------------------------------------- library: log sink, option defaults */

/** Install (sink not NULL) or remove (NULL) the process-wide log sink (C++ set_log_handler). Replaces an earlier one. */
IAD_API iad_result IAD_CALL iad_set_log_sink(iad_log_sink sink, void* user);

/** Option defaults (IAD_ERR_INVALID_ARGUMENT for NULL). */
IAD_API iad_result IAD_CALL iad_push_options_init(iad_push_options* options);
IAD_API iad_result IAD_CALL iad_sequence_options_init(iad_sequence_options* options);
IAD_API iad_result IAD_CALL iad_monitor_options_init(iad_monitor_options* options);
IAD_API iad_result IAD_CALL iad_track_options_init(iad_track_options* options);
IAD_API iad_result IAD_CALL iad_recovery_policy_init(iad_recovery_policy* policy);
IAD_API iad_result IAD_CALL iad_check_options_init(iad_check_options* options);

/* --------------------------------------------------------------------- axis: push, sequence, acceleration change */

/** Push move (Axis::push): approach `target` at options->speed, then push over `band` (signed by options->direction)
 *  at force_pct (PPOW = floor(pct x 255 / 100)) up to min(max_push_pct, MXPP 7.9). Refused (nothing sent) when the
 *  unit cannot push, the band end is outside the stroke, the axis is moving, or it stands inside or past the push
 *  zone. `out` is written only when options->wait; the full result (warnings) is then in iad_axis_last_result_json. */
IAD_API iad_result IAD_CALL iad_axis_push(iad_axis* axis, double target, double band, double force_pct,
                                          const iad_push_options* options, iad_push_result* out /* may be NULL */);

/** Change the acceleration (= deceleration) of the 0x9900 move in progress (Axis::change_accel; as
 *  iad_axis_change_speed). `out` is written only when options->wait. */
IAD_API iad_result IAD_CALL iad_axis_change_accel(iad_axis* axis, double accel_g, const iad_change_options* options,
                                                  iad_status* out /* may be NULL */);

/** Absolute moves through `points` in order (Axis::move_sequence); every point is checked before the first frame
 *  (IAD_ERR_SOFT_LIMIT names the point). One status per point goes to `statuses` (at most `capacity`; may be NULL);
 *  *count (may be NULL) gets the number of points done. A too small array never stops the sequence. */
IAD_API iad_result IAD_CALL iad_axis_move_sequence(iad_axis* axis, const iad_sequence_point* points,
                                                   size_t point_count, const iad_sequence_options* options,
                                                   iad_status* statuses, size_t capacity, size_t* count);

/* ------------------------------------------------------------------------------------------- axis: move handles */

/* A move started without waiting: the command is sent (every check has passed) and a handle returned; options->wait is
 * ignored. With the status monitor running, a fresh monitor status replaces the pre-move read. A move handle keeps its
 * axis object alive (not its port): release it with iad_move_handle_close. One handle is used by one thread at a time,
 * except iad_move_handle_cancel, which may be called from another thread while iad_move_handle_wait /
 * iad_move_handle_poll runs. */

/** Axis::start_move_to / start_move_by / start_jog (NULL options: iad_move_options_init; jog: the jog defaults). */
IAD_API iad_result IAD_CALL iad_axis_start_move_to(iad_axis* axis, double position, const iad_move_options* options,
                                                   iad_move_handle** out);
IAD_API iad_result IAD_CALL iad_axis_start_move_by(iad_axis* axis, double delta, const iad_move_options* options,
                                                   iad_move_handle** out);
IAD_API iad_result IAD_CALL iad_axis_start_jog(iad_axis* axis, double delta, const iad_move_options* options,
                                               iad_move_handle** out);

/** One check, no waiting (MoveHandle::done; the safe rule needs its settle window over several calls). `out` (may be
 *  NULL) gets the final status once done. Fails like iad_move_handle_wait on an alarm or a lost servo (the axis
 *  stopped). */
IAD_API iad_result IAD_CALL iad_move_handle_done(iad_move_handle* move, iad_bool* done,
                                                 iad_status* out /* may be NULL */);

/** Block until done (MoveHandle::wait). timeout_s NaN = the time left of the move's estimate, at least settle_s + 1 s.
 *  On timeout or alarm a decel stop is sent and IAD_ERR_MOTION_TIMEOUT / IAD_ERR_ALARM returned. */
IAD_API iad_result IAD_CALL iad_move_handle_wait(iad_move_handle* move, double timeout_s,
                                                 iad_status* out /* may be NULL */);

/** Wait at most max_wait_s for the move to finish, without stopping it when that slice ends (*done = 0): a binding
 *  waits in short slices so that it stays interruptible. Once the move's deadline has passed this is
 *  iad_move_handle_wait (decel stop and IAD_ERR_MOTION_TIMEOUT when it still has not finished). iad_axis_cancel on its
 *  axis ends a slice at once: a decel stop, then IAD_ERR_CANCELLED (MoveHandle::poll). */
IAD_API iad_result IAD_CALL iad_move_handle_poll(iad_move_handle* move, double max_wait_s, iad_bool* done,
                                                 iad_status* out /* may be NULL */);

/** Decel stop (MoveHandle::cancel = Axis::stop); the stop status becomes the result unless one is known. timeout_s
 *  NaN = 5 s. May be called from another thread while a wait runs on this handle (that wait then ends as a move does
 *  when iad_axis_stop is called during it). */
IAD_API iad_result IAD_CALL iad_move_handle_cancel(iad_move_handle* move, iad_bool wait, double timeout_s,
                                                   iad_status* out /* may be NULL */);

/** What the handle knows (no I/O): target, band, deadline and the result once known. */
IAD_API iad_result IAD_CALL iad_move_handle_result(iad_move_handle* move, iad_move_info* out);

/** The same as JSON (MoveHandle::to_json: {"target_mm", "band_mm", "completion", "timeout_s", "done", "cancelled",
 *  "position_mm"}; no I/O). */
IAD_API iad_result IAD_CALL iad_move_handle_json(iad_move_handle* move, char* buffer, size_t buffer_size,
                                                 size_t* required);

/** Release the handle (the move itself is not affected). NULL or a closed handle: IAD_OK. */
IAD_API iad_result IAD_CALL iad_move_handle_close(iad_move_handle* move);

/* ---------------------------------------------------------------------------------- axis: status monitor, events */

/** Start the background status monitor (Axis::start_monitor; read-only, lean 8-word polls) and queue its events on
 *  this handle (options->events). Already running: the options other than the queue's are ignored and the running
 *  monitor is kept. Stopped by iad_axis_stop_monitor and iad_axis_close (an owned axis). */
IAD_API iad_result IAD_CALL iad_axis_start_monitor(iad_axis* axis, const iad_monitor_options* options);

/** Stop the status monitor (no-op without one). timeout_s NaN = 1 s. */
IAD_API iad_result IAD_CALL iad_axis_stop_monitor(iad_axis* axis, double timeout_s);

/** The monitor's last sample (no I/O). IAD_ERR_NOT_READY when no monitor was started. */
IAD_API iad_result IAD_CALL iad_axis_monitor_latest(iad_axis* axis, iad_monitor_sample* out);

/** Wait for the first sample with seq > after_seq (after_seq <= 0: the next one) (AxisMonitor::wait_update). *got = 0
 *  on timeout or when the monitor stopped; IAD_ERR_TRANSPORT / IAD_ERR_MODBUS... when it ended on an error (port
 *  gone). timeout_s NaN = 1 s. */
IAD_API iad_result IAD_CALL iad_axis_monitor_wait_update(iad_axis* axis, int64_t after_seq, double timeout_s,
                                                         iad_bool* got, iad_monitor_sample* out /* may be NULL */);

/** Wait until a sample shows the axis done by the fast rule against its last target (AxisMonitor::wait_done). *done =
 *  0 on timeout. timeout_s NaN = 10 s. */
IAD_API iad_result IAD_CALL iad_axis_monitor_wait_done(iad_axis* axis, double timeout_s, iad_bool* done,
                                                       iad_status* out /* may be NULL */);

/** The monitor as JSON (no I/O): {"running", "name", "polls", "error_count", "link_checks", "poll_rate_hz",
 *  "age_s", "link_state", "gap_s", "retries", "link_check_s", "error", "last_error"}; null without a monitor. */
IAD_API iad_result IAD_CALL iad_axis_monitor_info_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                       size_t* required);

/** The next queued event of this handle: waits up to timeout_s (NaN or <= 0: no wait) and gives IAD_EVENT_NONE when
 *  none came. Status / done events need a monitor started on this handle; link events come from the monitor, from
 *  iad_axis_check_link and from iad_axis_recover. May be called from another thread at any time; iad_axis_close wakes a
 *  waiting call (IAD_ERR_INVALID_ARGUMENT). */
IAD_API iad_result IAD_CALL iad_axis_next_event(iad_axis* axis, double timeout_s, iad_event* out);

/** Install (sink not NULL) or remove (NULL) the frame sink of this axis's port (ModbusRtu::set_frame_logger; on a
 *  shared port it sees the frames of every axis of the port). */
IAD_API iad_result IAD_CALL iad_axis_set_frame_sink(iad_axis* axis, iad_frame_sink sink, void* user);

/* ------------------------------------------------------------------------------------------- axis: fast mode */

/** The fast path in one call (Axis::enable_fast): fast completion, the status monitor (with `monitor`) and the link
 *  raised to baud_rate (IAD_FAST_BAUD_RATE; 0 keeps the rate). The baud change is tried only when this axis owns the
 *  port, the transport has a baud rate and the axis is at rest; otherwise the rate is kept and the reason is in the
 *  notes. Safe to call again. */
IAD_API iad_result IAD_CALL iad_axis_enable_fast(iad_axis* axis, int32_t baud_rate, iad_bool monitor,
                                                 iad_fast_summary* out /* may be NULL */);

/** Undo it (Axis::disable_fast): safe completion again and the monitor stopped; the baud rate is kept. */
IAD_API iad_result IAD_CALL iad_axis_disable_fast(iad_axis* axis, iad_fast_summary* out /* may be NULL */);

/** iad_axis_enable_fast ran and no disable since (Axis::fast_mode). */
IAD_API iad_result IAD_CALL iad_axis_fast_mode(iad_axis* axis, iad_bool* on);

/* ------------------------------------------------------------------------------------------- axis: recovery */

/* Row lists are text: NULL or "" = none, "all" = every row, "0,1,5" = these rows. A backup is the JSON text of a
 * backup (iad_axis_backup_json, the Python edition's backup files). Link events go to the handle's event queue. */

/** Remember what the application has set up while the link is up (recovery::track; READ-ONLY): identity, baud rate,
 *  PC control, safety speed, HEND, uptime, alarm history and optionally rows / parameters. `name`: NULL keeps the
 *  earlier session's name. */
IAD_API iad_result IAD_CALL iad_axis_track(iad_axis* axis, const iad_track_options* options, const char* name,
                                           const char* rows, const char* backup_json);

/** A session from a backup (recovery::session_from_backup; no I/O): a recovery then requires the unit the backup was
 *  taken from. IAD_ERR_INVALID_ARGUMENT when the backup holds no identity or is not JSON. */
IAD_API iad_result IAD_CALL iad_axis_session_from_backup(iad_axis* axis, const char* backup_json, const char* name);
/** The same from a backup file (UTF-8 JSON). */
IAD_API iad_result IAD_CALL iad_axis_session_from_backup_file(iad_axis* axis, const char* path, const char* name);

/** The unit's session as JSON (Session::to_json; no I/O). IAD_ERR_NOT_READY when it has none. */
IAD_API iad_result IAD_CALL iad_axis_session_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** Drop the unit's session, link state and pending target (recovery::forget). */
IAD_API iad_result IAD_CALL iad_axis_forget(iad_axis* axis);

/** Record the last commanded target (NaN: none, e.g. after a home return) for the recovery's position check
 *  (recovery::note_target; the axis records its own moves). */
IAD_API iad_result IAD_CALL iad_axis_note_target(iad_axis* axis, double target_mm);

/** Tell the session about a deliberate change (recovery::note_control): IAD_UNKNOWN = unchanged for a tribool, 0 =
 *  unchanged for baud_rate. */
IAD_API iad_result IAD_CALL iad_axis_note_control(iad_axis* axis, iad_tribool modbus_control, iad_tribool safety_speed,
                                                  int32_t baud_rate, iad_tribool homed);

/** One cheap link check (recovery::check_link; one status read compared with the session; READ-ONLY). The record is
 *  also kept for iad_axis_last_result_json. A failure that is not a link fault is returned as an error. */
IAD_API iad_result IAD_CALL iad_axis_check_link(iad_axis* axis, iad_link_check* out);

/** The unit's last known link state ("ok", "port_vanished", "unit_silent", "unit_restarted", "unit_replaced",
 *  "recovering"; no I/O). Text output: the buffer rules of the JSON outputs. */
IAD_API iad_result IAD_CALL iad_axis_link_state(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** Bring the axis back after a power or link loss and report what changed (recovery::recover_axis: without a session
 *  the identity read by the last connect / identify is the baseline). NEVER starts motion. timeout_s NaN = 30 s.
 *  `check_rows`: the rows to compare; `backup_json` (may be NULL): compare with this backup. The full report is kept
 *  for iad_axis_last_result_json; IAD_ERR_RECOVERY / IAD_ERR_UNIT_REPLACED carry it in iad_last_error_json ("report"). */
IAD_API iad_result IAD_CALL iad_axis_recover(iad_axis* axis, double timeout_s, const iad_recovery_policy* policy,
                                             const char* check_rows, const char* backup_json, const char* name,
                                             iad_recovery_summary* out /* may be NULL */);

/* --------------------------------------------------------------------- axis: unit information, backup, self-test */

/* These reads can be long (whole parameter classes, every stored row). Their JSON is also kept on the handle, even
 * when the buffer was too small: fetch it with iad_axis_last_result_json (no I/O) instead of reading again. (0.7) The
 * unit info and backup readers (also iad_axis_info_call_json "unit_info" / "backup") end with IAD_ERR_CANCELLED at
 * their next request after iad_axis_cancel / iad_axis_cancel_latch (nothing is sent for it). */

/** Everything the unit tells about itself (info::unit_info; READ-ONLY). `sections`: NULL or "" = all, else names of
 *  "identity,parameters,monitor,alarms,maintenance,positions". */
IAD_API iad_result IAD_CALL iad_axis_read_unit_info_json(iad_axis* axis, const char* sections, char* buffer,
                                                         size_t buffer_size, size_t* required);

/** A read-only backup in the Python edition's schema (info::backup; FC03 + FC11 only). `rows`: NULL = "used", "all"
 *  or "0,1,2". `path` (may be NULL): also written there (UTF-8 JSON). */
IAD_API iad_result IAD_CALL iad_axis_backup_json(iad_axis* axis, const char* rows, const char* path, char* buffer,
                                                 size_t buffer_size, size_t* required);

/** iad_axis_backup_json of several axis handles into `directory` (info::backup_axes; created if missing), one
 *  "<name>.json" each (characters other than letters, digits, '-', '_', '.' become '_'). `names`: NULL or "" = each
 *  axis's "port:slave" name, else axis_count comma-separated names. An axis that does not answer is reported and the
 *  others are still saved. The result {name: file path or "error: <Type>: <message>"} is written as JSON; it is not
 *  kept on a handle (call again to read it: the files are written again). */
IAD_API iad_result IAD_CALL iad_backup_axes_json(iad_axis* const* axes, size_t axis_count, const char* names,
                                                 const char* directory, const char* rows, char* buffer,
                                                 size_t buffer_size, size_t* required);

/** The compatibility self-test (run_check; read-only unless options->motion). Each line is also queued as an
 *  IAD_EVENT_CHECK_ITEM event. Never fails for what the unit answers (a FAIL line); the JSON report is kept for
 *  iad_axis_last_result_json. `port_label` (may be NULL) is recorded as the report's port. */
IAD_API iad_result IAD_CALL iad_axis_self_test(iad_axis* axis, const iad_check_options* options,
                                               const char* port_label, iad_check_summary* out /* may be NULL */);

/* --------------------------------------------------------------------------- axis: line, modes, strategy, cancel */

/** The line this axis is on (Line::snapshot as JSON: kind, rules, the gateway's last status, position units,
 *  unreachable slaves, explanation; no I/O). */
IAD_API iad_result IAD_CALL iad_axis_line_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** A fresh read of the line's gateway status (RCON / ROBONET gateway; READ-ONLY); null on a line without one. */
IAD_API iad_result IAD_CALL iad_axis_gateway_status_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                         size_t* required);

/** State this axis's position unit (0.01 or 0.001 mm; RCON parameter No.207) on its line (Line::set_position_unit).
 *  Needed before mm-based motion on an RCON line. */
IAD_API iad_result IAD_CALL iad_axis_set_position_unit(iad_axis* axis, double unit_mm);

/** The action mode (Axis::set_action_mode): pc = PC / Modbus control (PMSS 1) or PIO (PMSS 0); `safety` IAD_UNKNOWN
 *  keeps the present safety speed. PIO is 0x45 cmd 0x0C op1 = 0; PC is the PMSL coil (when PMSS is 0) and the SFTY coil
 *  (when the safety speed changes), since 0x45 op1 = 1 left a PCON-CB refusing servo ON (VERIFIED, 2026-11-02); a unit
 *  without coils still gets op1 = 1. Refused while the axis moves, pauses, homes or pushes (IAD_ERR_NOT_READY) and on
 *  a line without 0x45 (IAD_ERR_UNSAFE_REFUSED), nothing sent. With verify the result is read back
 *  (IAD_ERR_VERIFICATION). */
IAD_API iad_result IAD_CALL iad_axis_set_action_mode(iad_axis* axis, iad_bool pc, iad_tribool safety, iad_bool verify,
                                                     iad_status* out /* may be NULL */);

/** Switch the safety-speed mode (Axis::set_safety_speed; ~100 mm/s cap when on; volatile): the SFTY coil, or DRG1 b14
 *  on the register path. Takes Modbus control first if needed (at rest only). The driver never changes it by itself. */
IAD_API iad_result IAD_CALL iad_axis_set_safety_speed(iad_axis* axis, iad_bool on, iad_bool verify,
                                                      iad_status* out /* may be NULL */);

/** End every blocking call of this axis running now in another thread (a move, jog, home, push, stored row, wait_idle,
 *  a move handle's wait, the servo wait, a cell wait on this axis): it returns IAD_ERR_CANCELLED soon, a motion wait
 *  after sending a decel stop (Axis::cancel_waits). Calls started afterwards are not affected. Sends nothing itself;
 *  safe from any thread (a binding's Ctrl+C handler). */
IAD_API iad_result IAD_CALL iad_axis_cancel(iad_axis* axis);

/** The command strategy for servo / home / alarm reset (Axis::set_strategy / strategy): IAD_METHOD_COIL, _IAI, _IAI2
 *  ([EXPERIMENTAL]) or _REGISTER. The coil strategy adapts by itself (register behind RCON or after an FC05
 *  exception 01). */
IAD_API iad_result IAD_CALL iad_axis_set_strategy(iad_axis* axis, int32_t method);
IAD_API iad_result IAD_CALL iad_axis_strategy(iad_axis* axis, int32_t* method);

/** How a stop is sent (Axis::stop_path): "coil" (the STOP coil) or "register" (the coil-free substitute). Text output. */
IAD_API iad_result IAD_CALL iad_axis_stop_path(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** The line kind ("rcon", "robonet", "robonet-gateway"; "" on a direct line) (Axis::line_name). Text output. */
IAD_API iad_result IAD_CALL iad_axis_line_name(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** One read of the whole 0x9000 monitor block, decoded (monitor_reading: position, velocity, current, current_pct,
 *  deviation, alarm, uptime_s, units, raw; READ-ONLY). */
IAD_API iad_result IAD_CALL iad_axis_read_monitor_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                       size_t* required);

/** The lean status: one read of 0x9000-0x9007 (Axis::read_lean_status; READ-ONLY), the 8 words with position, alarm
 *  code, DSS1, DSS2 and DSSE that the status monitor and the fast completion use: raw_words 8, current_ma and uptime_s
 *  NaN, velocity_mm_s NaN while moving. retries IAD_INT_UNKNOWN = the port's. */
IAD_API iad_result IAD_CALL iad_axis_read_lean_status(iad_axis* axis, int32_t retries, iad_status* out);

/** The power-up baud rate (Axis::read_boot_baud_rate: BRSL 8.3 on an EC, No.16 on a CON unit; READ-ONLY).
 *  *rate = IAD_INT_UNKNOWN when it is not readable or an unknown code. */
IAD_API iad_result IAD_CALL iad_axis_read_boot_baud_rate(iad_axis* axis, int32_t* rate);

/** The RCON all-axes monitor of this axis's line (Port::gateway_monitor; at most two FC03 frames for 16 axes;
 *  READ-ONLY): {"<slave>": {"axis", "slave", "position_counts", "velocity_counts", "current_ma", "alarm_code", "dss1",
 *  "dsse", "sipm", "overload_pct", "deviation_pulses", "stat"}}. Positions are counts in the axis's position unit,
 *  speeds 0.01 mm/s. `slaves`: NULL or "" = the slaves the gateway reports linked, else "1,2" (1..16).
 *  IAD_ERR_NOT_READY on a line without an RCON gateway. DOC (ME0413 p.5-2), UNVERIFIED on hardware. */
IAD_API iad_result IAD_CALL iad_axis_gateway_monitor_json(iad_axis* axis, const char* slaves, char* buffer,
                                                          size_t buffer_size, size_t* required);

/** The caps moves are checked against and the documented minimum speed (iad_motion_caps). May read MAXV / MAXA / MAXD
 *  / MXPP once (READ-ONLY). */
IAD_API iad_result IAD_CALL iad_axis_motion_caps(iad_axis* axis, iad_motion_caps* out);

/** The unit's home pattern ORG (Axis::read_home_pattern; EC item 9.16; READ-ONLY). *known = 0 and *pattern = 0 when
 *  the item is not in the unit's directory. */
IAD_API iad_result IAD_CALL iad_axis_read_home_pattern(iad_axis* axis, iad_bool* known, uint32_t* pattern);

/** Release what a home start left set (Axis::finish_home: the HOME coil of the coil strategy, the DRG1 HOME bit of the
 *  register path; nothing with the IAI strategies): call it after iad_axis_home without wait once the axis is homed. */
IAD_API iad_result IAD_CALL iad_axis_finish_home(iad_axis* axis);

/* -------------------------------------------------------- cell: pause, cancel, monitors, recovery, lines, backup */

/** Start an absolute move on each axis of `moves` with its own options (Cell::move_each; a decel_g other than accel_g
 *  runs the command-profile path), checked first like iad_cell_move_all (one bad value refuses all, nothing sent). With
 *  wait the call waits for all (timeout_s NaN = per axis); otherwise it returns once every move started, *count 0.
 *  stop_others_on_error: a start that fails stops the axes already started, a failed wait the others (0: neither). */
IAD_API iad_result IAD_CALL iad_cell_move_each(iad_cell* cell, const iad_axis_move* moves, size_t move_count,
                                               iad_bool wait, double timeout_s, iad_bool stop_others_on_error,
                                               iad_cell_result* results, size_t capacity, size_t* count);

/** Pause every listed axis, keeping the remaining travel (Cell::pause_all; ports in parallel). `broadcast`: first one
 *  broadcast pause on every daisy chain whose axes are all selected (never behind a gateway; UNVERIFIED), then the
 *  per-axis pause. The statuses in `results` are not read (raw_words 0). Unknown names as iad_cell_stop_all: the axes
 *  the cell has are paused, each unknown name is a result of its own. */
IAD_API iad_result IAD_CALL iad_cell_pause_all(iad_cell* cell, const char* axes, iad_bool broadcast,
                                               iad_cell_result* results, size_t capacity, size_t* count);
/** Release the pause on every listed axis (Cell::resume_all: the paused moves continue); `broadcast` as above. */
IAD_API iad_result IAD_CALL iad_cell_resume_all(iad_cell* cell, const char* axes, iad_bool broadcast,
                                                iad_cell_result* results, size_t capacity, size_t* count);

/** iad_axis_cancel on every listed axis (Axis::cancel_waits each): a group command waiting on them ends with
 *  IAD_ERR_CELL whose axes say IAD_ERR_CANCELLED. Safe from any thread. A listed name the cell does not have never
 *  holds the cancel back: every listed axis it has is cancelled, then the call returns IAD_ERR_CELL with each unknown
 *  name an error of its own (IAD_ERR_INVALID_ARGUMENT, in the "axes" of iad_last_error_json; operation "cancel"), as
 *  iad_cell_stop_all reports them. Only unknown names: nothing is cancelled (never "every axis"). */
IAD_API iad_result IAD_CALL iad_cell_cancel(iad_cell* cell, const char* axes);

/** Start the status monitor of every listed axis (Cell::start_monitors: their link events also go to the cell's fault
 *  handling) and queue their events on the cell handle (iad_cell_next_event; the event's axis names it). The cell's
 *  faults (on_fault) and recovery events are queued there from the open on. */
IAD_API iad_result IAD_CALL iad_cell_start_monitors(iad_cell* cell, const char* axes,
                                                    const iad_monitor_options* options);
/** Stop the status monitors of the listed axes. */
IAD_API iad_result IAD_CALL iad_cell_stop_monitors(iad_cell* cell, const char* axes);

/** The next queued event of the cell (as iad_axis_next_event). */
IAD_API iad_result IAD_CALL iad_cell_next_event(iad_cell* cell, double timeout_s, iad_event* out);

/** recovery::track on every listed axis (Cell::track_all; READ-ONLY). The sessions as JSON ({name: session}) are kept
 *  for iad_cell_last_result_json. */
IAD_API iad_result IAD_CALL iad_cell_track_all(iad_cell* cell, const char* axes, const iad_track_options* options,
                                               const char* rows, iad_cell_result* results, size_t capacity,
                                               size_t* count);

/** recovery::check_link on every listed axis (Cell::check_links; a fault goes to on_fault: queued, and with
 *  stop_on_fault the other axes stop). One iad_link_check per axis in list order (state "error" when the check itself
 *  failed). The records ({"checks": {name: ...}, "errors": {name: message}}) are kept for iad_cell_last_result_json. */
IAD_API iad_result IAD_CALL iad_cell_check_links(iad_cell* cell, const char* axes, iad_link_check* checks,
                                                 size_t capacity, size_t* count);

/** Start a background link monitor on the listed axes every interval_s (Cell::monitor_links; state changes are
 *  queued as IAD_EVENT_LINK_STATE, faults as IAD_EVENT_FAULT). It never recovers or moves anything. Replaces a
 *  monitor started earlier on this handle. */
IAD_API iad_result IAD_CALL iad_cell_monitor_links(iad_cell* cell, double interval_s, const char* axes);
/** Stop the link monitor of this handle (no-op without one). */
IAD_API iad_result IAD_CALL iad_cell_stop_link_monitor(iad_cell* cell);

/** recovery::recover on every listed axis (Cell::recover_all; ports in parallel). NEVER starts motion. Per-axis codes
 *  and statuses go to `results`; the reports ({"reports": {name: ...}, "errors": {name: message}}) are kept for
 *  iad_cell_last_result_json. timeout_s NaN = 30 s. */
IAD_API iad_result IAD_CALL iad_cell_recover_all(iad_cell* cell, const char* axes, double timeout_s,
                                                 const iad_recovery_policy* policy, const char* check_rows,
                                                 iad_cell_result* results, size_t capacity, size_t* count);

/** monitor_reading of every listed axis (Cell::monitor_all; READ-ONLY): {"readings": {name: reading}, "errors":
 *  {name: message}}. */
IAD_API iad_result IAD_CALL iad_cell_monitor_all_json(iad_cell* cell, const char* axes, char* buffer,
                                                      size_t buffer_size, size_t* required);

/** Every port's line as JSON ({port: Line::snapshot}; no I/O). */
IAD_API iad_result IAD_CALL iad_cell_lines_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required);

/** A fresh read of every gateway's status ({port: status}, Cell::gateway_status_all; READ-ONLY). */
IAD_API iad_result IAD_CALL iad_cell_gateway_status_json(iad_cell* cell, char* buffer, size_t buffer_size,
                                                         size_t* required);

/** State the position unit (0.01 or 0.001 mm) of the listed slaves ("1,2"; NULL = every axis) on `port`
 *  (Port::set_position_unit). */
IAD_API iad_result IAD_CALL iad_cell_set_position_unit(iad_cell* cell, const char* port, double unit_mm,
                                                       const char* slaves);

/** The RCON all-axes monitor of `port` (Port::gateway_monitor; READ-ONLY), as iad_axis_gateway_monitor_json. */
IAD_API iad_result IAD_CALL iad_cell_gateway_monitor_json(iad_cell* cell, const char* port, const char* slaves,
                                                          char* buffer, size_t buffer_size, size_t* required);

/** A read-only backup of every listed axis into `directory` (Cell::backup; one "<name>.json" each). The result
 *  ({name: file path or "error: ..."}) is written as JSON and kept for iad_cell_last_result_json. `rows` as
 *  iad_axis_backup_json. */
IAD_API iad_result IAD_CALL iad_cell_backup_json(iad_cell* cell, const char* axes, const char* directory,
                                                 const char* rows, char* buffer, size_t buffer_size,
                                                 size_t* required);

/** The JSON kept by the last iad_cell_track_all, iad_cell_check_links, iad_cell_recover_all or iad_cell_backup_json on
 *  this handle (no I/O). IAD_ERR_NOT_READY when there is none. */
IAD_API iad_result IAD_CALL iad_cell_last_result_json(iad_cell* cell, char* buffer, size_t buffer_size,
                                                      size_t* required);

/* -------------------------------------------------------------------------------------- cell: coordinated motion */

/* Coordinated (interpolated) motion of several cell axes along one path, run by the PC (C++ coord.hpp,
 * Cell::coordinated). A request is one JSON object:
 *   {"axes": {"x": "COM3:1", "y": "COM4:1"}   the path names and their cell axes; or ["COM3:1", "COM4:1"] (the cell
 *                                             names are the path names); absent: every axis under its cell name
 *    "band": 0.1,                             the positioning band of every frame (optional)
 *    "path": PATH, "speed": 20, "accel_g": 0.1, path speed (units/s, required) and ramps (G)
 *    "mode": "auto" | "sync" | "stream", "chord_error": 0.05, "min_step_s", "blend", "stream_accel_g",
 *    "late_abort_s", "timeout_s",             all optional (C++ PlanOptions and RunOptions); timeout_s: the wait for
 *                                             the last waypoint, absent = from the plan, else a finite number of seconds
 *                                             above 0: 0 or a negative value is IAD_ERR_INVALID_ARGUMENT, nothing sent
 *    "expect_start": {"x": 10, "y": 20}}      optional (from release 1.0.1; the run's, which the dry run only reads):
 *                                             where the plan to run must start, by path names. The run plans first,
 *                                             then each named axis must be within max(band, 0.05) of the start of that
 *                                             plan (its lead-in's, if any), else IAD_ERR_NOT_READY, nothing sent; a
 *                                             name the path lacks is IAD_ERR_INVALID_ARGUMENT
 * PATH = {"start": {"x": 0, "y": 0} (optional), "segments": [SEGMENT, ...]}, or a shape:
 *   {"shape": "circle", "center": C, "radius": r, "start_deg": 0, "turns": 1, "ccw": true, "plane": ["x", "y"]}
 *   {"shape": "rectangle", "corner": C, "width": w, "height": h, "plane": ["x", "y"]}
 *   {"shape": "polygon", "center": C, "radius": r, "sides": n, "start_deg": 0, "plane": ["x", "y"]}
 * SEGMENT = {"line": POINT} | {"polyline": [POINT, ...]}
 *   | {"arc": {"center": C, "end": POINT, "ccw": true, "plane": ["x", "y"]}}
 *   | {"circle": {"center": C, "radius": r, "start_deg": 0, "turns": 1, "ccw": true, "plane": ["x", "y"]}}
 * POINT = {"x": 10, "y": 20} (axes not named hold their position); C = [a, b] in the plane, or a POINT naming both
 * plane axes. "plane" defaults to ["x", "y"]. Positions in each axis's own units. Unknown keys are refused.
 * Evidence: the C++ coordinated runs are tested on the simulator only (UNVERIFIED on hardware). */

/** Dry run (CoordinatedGroup::plan): plan the request's path from the axes' current positions against their ranges
 *  and the line timing measured now. Reads only, no motion. Writes the plan as JSON (MotionPlan: mode, axes, points,
 *  times, speeds, accels, warnings... plus "summary"). An infeasible path: IAD_ERR_SOFT_LIMIT, kind
 *  "InfeasiblePathError", with "reasons" and "max_speed" (the largest feasible speed, null when none) in
 *  iad_last_error_json; a path that moves no axis: IAD_ERR_INVALID_ARGUMENT, kind "NoMotionError". */
IAD_API iad_result IAD_CALL iad_cell_coord_plan_json(iad_cell* cell, const char* request, char* buffer,
                                                     size_t buffer_size, size_t* required);

/** Plan and run the request's path (CoordinatedGroup::run): blocks until the path is done. The result (C++
 *  CoordinatedResult: completed, frames, late frames, deviation, samples, trace... plus "summary") is written as JSON
 *  and kept for iad_cell_last_result_json, even when the buffer is too small. An outside stop (iad_cell_stop_all,
 *  iad_axis_stop, iad_axis_pause from another thread; iad_cell_cancel / iad_axis_cancel do not end a run), a fault
 *  alarm, a link fault or a late frame stops every axis of the group (decel stop) and returns
 *  IAD_ERR_DRIVER with kind "CoordinatedMotionError"; iad_last_error_json then has "axis" (the axis that failed, or
 *  null), "cause" ({code, name, kind, message}), "stop" ({"stopped": [axes], "errors": {axis: message}}) and "result"
 *  (the result so far, also kept for iad_cell_last_result_json). Refusals before anything is sent as
 *  iad_cell_coord_plan_json; an axis in another coordinated run: IAD_ERR_NOT_READY. One run at a time per axis. */
IAD_API iad_result IAD_CALL iad_cell_coord_run_json(iad_cell* cell, const char* request, char* buffer,
                                                    size_t buffer_size, size_t* required);

/* ------------------------------------------------------------------ library: callback error, log level, defaults */

/** From inside a transport callback that then returns a failure: the text of the TransportError the library raises
 *  for it (copied; this thread only; cleared before each callback). Does not touch the thread's last error. */
IAD_API iad_result IAD_CALL iad_callback_error(const char* message);

/** Log lines below `level` (IAD_LOG_*) are not passed to the log sink (default IAD_LOG_DEBUG: every line). */
IAD_API iad_result IAD_CALL iad_set_log_level(int32_t level);

/* (0.8) The clock seam (iad_clock_*) is in iai_actuator_driver_testing.h. */

/** Defaults; struct_size = sizeof the caller's struct (IAD_ERR_INVALID_ARGUMENT for NULL). (0.5)
 *  iad_transport_callbacks grew and the library cannot see the caller's sizeof: a caller that uses set_read_timeout
 *  sets struct_size = sizeof(iad_transport_callbacks) before the call (then all of it is filled); any other value (0,
 *  a zeroed struct, a 0.3 / 0.4 caller) fills and sizes the 0.3 part only, so a struct built with an older header is
 *  never written past its end. */
IAD_API iad_result IAD_CALL iad_transport_callbacks_init(iad_transport_callbacks* callbacks);
IAD_API iad_result IAD_CALL iad_bus_options_init(iad_bus_options* options);

/* ------------------------------------------------------------------------------------------- transports */

/* (0.3) Transports and buses as handles, so that a binding runs everything through the core: a serial port or the
 * binding's own transport through callbacks (callback types), every request and response frame to a sink (buses),
 * simulated units and lines (simulators), and axes and cells opened on them. Evidence: simulator tests only
 * (UNVERIFIED on hardware). */

/** A transport on the binding's callbacks (copied; `name` for messages and axis names, NULL or "" = "CALLBACK").
 *  release(user) is called once when the library lets go of it (not when this call fails: nothing was taken). */
IAD_API iad_result IAD_CALL iad_transport_open_callbacks(const iad_transport_callbacks* callbacks, const char* name,
                                                         void* user, iad_transport** out);
/** Open a serial port (SerialTransport): baud_rate 0 = 115200; read_timeout_s NaN = 0.2. IAD_ERR_TRANSPORT when it is
 *  busy or absent. */
IAD_API iad_result IAD_CALL iad_transport_open_serial(const char* port, int32_t baud_rate, double read_timeout_s,
                                                      iad_transport** out);
/** Close the transport (Transport::close: a bus, axis or cell on it then fails with IAD_ERR_TRANSPORT until it is
 *  reopened) and release the handle. NULL or a closed handle: IAD_OK. */
IAD_API iad_result IAD_CALL iad_transport_close(iad_transport* transport);
/** Release the handle only: the buses, axes and cells using the transport keep it. NULL or a closed handle: IAD_OK. */
IAD_API iad_result IAD_CALL iad_transport_release(iad_transport* transport);
/** {"name", "open", "baud_rate" (null: none), "read_timeout_s", "real_timeout", "present" (null: unknown),
 *  "can_reopen"}. No I/O. */
IAD_API iad_result IAD_CALL iad_transport_info_json(iad_transport* transport, char* buffer, size_t buffer_size,
                                                    size_t* required);
/** Raw bytes (no bus: nothing is framed, locked or retried; for tests and tools). */
IAD_API iad_result IAD_CALL iad_transport_write(iad_transport* transport, const uint8_t* data, size_t size);
/** Read up to `size` bytes within the read timeout; *got = the bytes read. */
IAD_API iad_result IAD_CALL iad_transport_read(iad_transport* transport, uint8_t* buffer, size_t size, size_t* got);
IAD_API iad_result IAD_CALL iad_transport_reset_input(iad_transport* transport);
/** IAD_ERR_UNSUPPORTED on a transport without a baud rate. */
IAD_API iad_result IAD_CALL iad_transport_set_baud_rate(iad_transport* transport, int32_t rate);
IAD_API iad_result IAD_CALL iad_transport_set_read_timeout(iad_transport* transport, double seconds);
/** A line BREAK of 0 < seconds <= 2 (150 ms or more drops a unit to 9600 baud, DOC). IAD_ERR_UNSUPPORTED where the
 *  transport cannot. */
IAD_API iad_result IAD_CALL iad_transport_send_break(iad_transport* transport, double seconds);
/** Open the same port again in place after it vanished or was closed (baud_rate 0: as before). */
IAD_API iad_result IAD_CALL iad_transport_reopen(iad_transport* transport, int32_t baud_rate);

/* ------------------------------------------------------------------------------------------- buses */

/** A bus on a transport (ModbusRtu; options NULL = the defaults). No I/O. The bus shares the transport: closing the
 *  bus closes the transport; the transport handle stays valid (release it when no longer needed). */
IAD_API iad_result IAD_CALL iad_bus_open(iad_transport* transport, const iad_bus_options* options, iad_bus** out);
/** A bus from iad_bus_open: close it (and its transport) and release the handle. A bus from iad_axis_bus or
 *  iad_cell_bus: release the handle only. NULL or a closed handle: IAD_OK. */
IAD_API iad_result IAD_CALL iad_bus_close(iad_bus* bus);
/** A new handle on the bus's transport (iad_transport_release when done). */
IAD_API iad_result IAD_CALL iad_bus_transport(iad_bus* bus, iad_transport** out);
/** {"name", "retries", "min_gap_s", "baud_rate" (null: none), "can_change_baud_rate", "closed", "line",
 *  "unsupported_functions" ({"slave": [function codes answered with exception 01]})}; (0.4) "lock_depth" (the calling
 *  thread's holds of the bus lock), "lock_waiting" ({"normal", "stop"}: threads waiting for it), "preemptible_words"
 *  (the calling thread's preemptible scope on this bus, null: none), "suspended_monitors" (held by this handle). No
 *  I/O; (0.4) never waits for the bus lock. */
IAD_API iad_result IAD_CALL iad_bus_info_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required);
IAD_API iad_result IAD_CALL iad_bus_set_retries(iad_bus* bus, int32_t retries);
IAD_API iad_result IAD_CALL iad_bus_set_min_gap(iad_bus* bus, double seconds);
/** Install (sink not NULL) or remove (NULL) the frame sink of the bus (ModbusRtu::set_frame_logger): every request
 *  (IAD_FRAME_TX), response (IAD_FRAME_RX) and partial response (IAD_FRAME_RX_PARTIAL) of every axis on it. */
IAD_API iad_result IAD_CALL iad_bus_set_frame_sink(iad_bus* bus, iad_frame_sink sink, void* user);

/* Requests (slave 1..247). `retries`: extra attempts for this call; IAD_INT_UNKNOWN = the bus's. An exception
 * response is IAD_ERR_SLAVE_EXCEPTION (its code and function in iad_last_error_json); silence after the retries
 * IAD_ERR_MODBUS_TIMEOUT. Each request passes the line's guard first (nothing sent on a refusal). */
/** FC03: `count` (1..) registers at `address` into values[0 .. count) (ModbusRtu::read_registers with ReadOptions; the
 *  Python edition's per-call options): `chunk` registers per FC03 frame (IAD_INT_UNKNOWN: 60; 1..125; longer reads are
 *  split), max_wait_s caps the wait per attempt (NaN: no cap; else > 0; a probe: a missing unit costs about that).
 *  READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_read_registers(iad_bus* bus, int32_t slave, int32_t address, int32_t count,
                                                   int32_t retries, double max_wait_s, int32_t chunk,
                                                   uint16_t* values);
/** FC05 (FF00 / 0000; the echo is checked). */
IAD_API iad_result IAD_CALL iad_bus_write_coil(iad_bus* bus, int32_t slave, int32_t address, iad_bool on,
                                               int32_t retries);
/** FC06 (the echo is checked). */
IAD_API iad_result IAD_CALL iad_bus_write_register(iad_bus* bus, int32_t slave, int32_t address, uint16_t value);
/** FC10: count (1..123) values (the echo is checked). */
IAD_API iad_result IAD_CALL iad_bus_write_registers(iad_bus* bus, int32_t slave, int32_t address, const uint16_t* values,
                                                    size_t count);
/** FC17: write `count` values at write_address, then read read_count registers at read_address into out. */
IAD_API iad_result IAD_CALL iad_bus_read_write_registers(iad_bus* bus, int32_t slave, int32_t read_address,
                                                         int32_t read_count, int32_t write_address,
                                                         const uint16_t* values, size_t count, uint16_t* out);
/** FC11: the data bytes after the byte count (ModbusRtu::report_slave_id). Each attempt waits at least timeout_s for
 *  the answer when that is longer than the transport's timeout (NaN: the transport's timeout only). Bytes:
 *  buffer[0 .. *length) (*length gets the size whatever the result; IAD_ERR_BUFFER_TOO_SMALL when it does not fit,
 *  nothing written). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_report_slave_id(iad_bus* bus, int32_t slave, int32_t retries, double timeout_s,
                                                    uint8_t* buffer, size_t buffer_size, size_t* length);
/** An IAI command (ModbusRtu::iai_command, iai_command2) by its function code: 0 or 0x45 = IAI 0x45, PDU 45 [group]
 *  [command] [op1 u16] [op2 u16] (operand_count 0..2, each 0..0xFFFF), the reply's status byte in status[0];
 *  [EXPERIMENTAL] 0x48 = IAI "ControlCommand2", PDU 48 [group u8] [command u16] [op1..op4 u32] (operand_count 0..4),
 *  21 bytes + CRC, the reply's four u32 words in status[0..3] (UNVERIFIED: none of the units tested takes 0x48, the
 *  frame layout included). Another code: IAD_ERR_UNSUPPORTED, nothing sent. Missing operands are 0; `operands` may be
 *  NULL when operand_count is 0. `flags` IAD_IAI_*: a command not on the function's safe list or another group than 0
 *  needs IAD_IAI_ALLOW_UNSAFE, one on its never-send list IAD_IAI_ALLOW_NEVER_SEND; a rejection is
 *  IAD_ERR_COMMAND_REJECTED unless IAD_IAI_NO_CHECK (a 0x48 rejection, 0x1000-0x13FF, is never retried). Refused before
 *  anything is sent on a line without the function (IAD_ERR_UNSAFE_REFUSED, kind LineRuleError). rejection may be
 *  NULL; status (may be NULL) gets at most status_count words. */
IAD_API iad_result IAD_CALL iad_bus_iai_command(iad_bus* bus, int32_t slave, int32_t function_code, int32_t group,
                                                int32_t command, const uint32_t* operands, size_t operand_count,
                                                uint32_t flags, int32_t retries, int32_t* rejection, uint32_t* status,
                                                size_t status_count);
/** FC05 to the broadcast address 0 (no answer; the bus is held for its turnaround). */
IAD_API iad_result IAD_CALL iad_bus_broadcast_coil(iad_bus* bus, int32_t address, iad_bool on);
/** One FC03 of one register at `address` (no retries, at most timeout_s; NaN = 0.05): *answered = a valid frame came
 *  (an exception response counts). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_probe(iad_bus* bus, int32_t slave, double timeout_s, int32_t address,
                                          iad_bool* answered);
/** Find the rate one of `targets` answers at and leave the transport there (ModbusRtu::detect_baud_rate_targets; the
 *  Python edition's detect_baudrate with (slave, address) entries): `targets` such as "1,2,0xD0@0x3700,0x3F@0xF700" (a
 *  slave, or slave@register; numbers decimal or 0x hex; NULL or "" = "1"), asked in order at each rate, the plain ones
 *  at probe_address (IAD_INT_UNKNOWN = 0x9000); `candidates` "115200,38400" (NULL = the default order). current_first:
 *  the present rate first, else the candidates in their order (the present rate where it appears, or last).
 *  probe_timeout_s caps each probe (NaN: a target with its own register 0.05 s; the plain ones 0.05 s when there are
 *  several, else one read with the transport's timeout). READ-ONLY probes; the rate is changed in place;
 *  IAD_ERR_MODBUS_TIMEOUT when nothing answers (the start rate restored). */
IAD_API iad_result IAD_CALL iad_bus_detect_baud_rate(iad_bus* bus, const char* targets, const char* candidates,
                                                     int32_t probe_address, iad_bool current_first,
                                                     double probe_timeout_s, int32_t* rate);
/** The PC side's rate in place (the units keep theirs: iad_axis_set_baud_rate changes both). */
IAD_API iad_result IAD_CALL iad_bus_set_baud_rate(iad_bus* bus, int32_t rate);
/** A line BREAK of 0 < seconds <= 2 under the bus lock (150 ms or more drops a unit to 9600 baud, DOC). */
IAD_API iad_result IAD_CALL iad_bus_send_break(iad_bus* bus, double seconds);
/** Open the transport again in place (ModbusRtu::reopen; baud_rate 0: as before). Every axis on the bus keeps
 *  working with it. */
IAD_API iad_result IAD_CALL iad_bus_reopen(iad_bus* bus, int32_t baud_rate);

/* ------------------------------------------------------------------------------------------- simulators */

/* Simulator options are one JSON object (NULL or "" = the defaults); unknown keys are refused. The C++
 * SimulatedUnitOptions: "profile" ("slider", "gripper", "rod", "pcon", "rotary", "unknown"), "slave" (1), "time_scale"
 * (1), "position_mm", "stroke_mm", "max_speed_mm_s", "max_accel_g", "system_code", "unit_code", "firmware" ([major,
 * minor]), "dopm" ("three_position" / "two_position"), "pmsl" (true), "recompute_position_checksum",
 * "drop_reply_after_retarget", "stop_latches", "move_bit_clears_in_decel", "safety_speed", "soft_limits",
 * "ls_range_mm", "early_in_position", "settle_tail_mm", "settle_speed_mm_s", "push_blip_mm", "push_blip_len_mm",
 * "absolute", "baud_rate" (the PC side's rate: models the line), "unit_baud_rate", "boot_baud_rate",
 * "reset_baud_rate", "reset_s", "slave_id_delay_s", "parw_busy_s", "parw_dsse_busy", "ignored_baud_codes" ([6]),
 * "serial_number", "move_patterns"; the hooks: "word_overrides" ({"address": value} or [[address, value]]),
 * "unreadable" ([[start, count]]), "stale_status_polls" (0), "step_s" (the unit's own clock: 1000.0 s, plus step_s
 * per request). Addresses are numbers or strings ("0x9005"). Without "step_s" the unit runs on the library's clock
 * (iad_clock_* in iai_actuator_driver_testing.h). */

/* (0.5) More iad_sim_open options (the rest of the Python edition's SimulatedEC): "auto_servo_off_s" (null: off; the
 * servo goes off by itself after that standstill, STAT b17 ASOF; SON or a move command switches it back on),
 * "cold_boot" (servo off and DSSE PSNS 0 at power-up; the first servo ON takes the excitation detection, on an EC
 * "ec_first_son_s", 0.3), "rtc" (SSSE b8), "function_codes" ([3, 6, 16]: only those are answered, others exception 01;
 * null: all), "broadcast" (frames to address 0 are acted on; null: unless "function_codes"), "parw_latches" (true;
 * false: the PARW coil is acknowledged and ignored). The unit also answers IAI function 0x48 (cmd 0x30 servo, 0x31
 * home, 0x34 decel stop, 0x23 alarm reset; others rejected 0x21) and the teach coils MOD 0x0414 / TEAC 0x0415. (0.6)
 * "port_name" (the port's name, default "SIM"). (0.7) "slave" 0 (as the Python edition's simulator): a unit with no
 * address of its own, which acts on broadcasts and never answers; a simulated line still takes only 1..247. (0.8) The
 * state JSON's additions, the further control commands and iad_sim_set_clock are in iai_actuator_driver_testing.h. */

/** A simulated unit (HookedSimUnit). Use it through iad_sim_transport (a bus, an axis) or on a line. */
IAD_API iad_result IAD_CALL iad_sim_open(const char* options_json, iad_sim** out);
/** Release the handle (the unit lives on while a transport, bus, axis or line uses it). NULL or closed: IAD_OK. */
IAD_API iad_result IAD_CALL iad_sim_close(iad_sim* sim);
/** A transport handle on the unit (the simulated port; iad_transport_close closes it, iad_transport_reopen opens it
 *  again). */
IAD_API iad_result IAD_CALL iad_sim_transport(iad_sim* sim, iad_transport** out);

/* (0.8) The unit's test hooks (state, request and event logs, power, unplug, word overrides, unreadable ranges,
 * stale polls, a register's value, its own answer to a frame, the request hook, the other controls) are in
 * iai_actuator_driver_testing.h. */

/** A simulated line `name` (NULL or "" = "SIM") with `units` on it (count 0: none yet). Two units cannot share a
 *  slave address (IAD_ERR_INVALID_ARGUMENT). */
IAD_API iad_result IAD_CALL iad_sim_line_open(const char* name, iad_sim* const* units, size_t unit_count,
                                              iad_sim_line** out);
/** Release the handle (the line lives on while a transport, bus, axis or cell uses it). NULL or closed: IAD_OK. */
IAD_API iad_result IAD_CALL iad_sim_line_close(iad_sim_line* line);
/** Add a unit (IAD_ERR_INVALID_ARGUMENT when its address is taken). */
IAD_API iad_result IAD_CALL iad_sim_line_add(iad_sim_line* line, iad_sim* sim);
/** Remove the unit at `slave` (IAD_ERR_INVALID_ARGUMENT, kind "std::out_of_range", when absent). */
IAD_API iad_result IAD_CALL iad_sim_line_remove(iad_sim_line* line, int32_t slave);
/** A transport handle on the line. */
IAD_API iad_result IAD_CALL iad_sim_line_transport(iad_sim_line* line, iad_transport** out);

/* (0.8) The line's request log and controls (iad_sim_line_requests_json, iad_sim_line_control_json) are in
 * iai_actuator_driver_testing.h. */

/* ------------------------------------------------------------------------ axes and cells on transports and buses */

/** One axis on a bus (C++ Axis(bus, slave, options)): no I/O (iad_axis_connect next). The axis does not own the bus:
 *  closing the axis releases its STOP / pause and leaves the bus open. */
IAD_API iad_result IAD_CALL iad_axis_open_bus(iad_bus* bus, int32_t slave, const iad_axis_options* axis_options,
                                              iad_axis** out);
/** iad_axis_open on any transport (Axis::open: baud detection when the transport has a rate, the line probe). The
 *  axis owns its own bus on the transport: closing the axis closes the transport. */
IAD_API iad_result IAD_CALL iad_axis_open_transport(iad_transport* transport, const iad_open_options* options,
                                                    const iad_axis_options* axis_options, iad_axis** out);
/** A handle on the bus of any axis (iad_bus_close releases the handle only). */
IAD_API iad_result IAD_CALL iad_axis_bus(iad_axis* axis, iad_bus** out);
/** A cell on transports (Cell::open on named transports): `names` ("SIM1,SIM2"; NULL or "" = each transport's own
 *  name) names them in order, `slaves` ("1,2"; NULL or "" = scan each). Closing the cell closes the transports. */
IAD_API iad_result IAD_CALL iad_cell_open_transports(iad_transport* const* transports, size_t transport_count,
                                                     const char* names, const char* slaves,
                                                     const iad_cell_options* options,
                                                     const iad_axis_options* axis_options, iad_cell** out);
/** A handle on the bus of the cell's port `port` (iad_bus_close releases the handle only). */
IAD_API iad_result IAD_CALL iad_cell_bus(iad_cell* cell, const char* port, iad_bus** out);

/* ----------------------------------------------------------------------------------- axis and cell: cancel latch */

/** The cancel latch (Axis::set_cancel_latch): iad_axis_cancel now, and while the latch is set every blocking call of
 *  this axis that starts (a move, jog, home, push, servo, stored row, sequence, wait, a move handle's wait) is refused
 *  with IAD_ERR_CANCELLED before anything is sent, and a wait that opens meanwhile ends at its first check point (a
 *  motion wait after a decel stop). So a cancel that comes just before a call starts is not lost (a binding: register
 *  the cancellation, call, unregister, iad_axis_cancel_reset). Stop, pause and resume are never refused. Safe from any
 *  thread. */
IAD_API iad_result IAD_CALL iad_axis_cancel_latch(iad_axis* axis);
/** Clear the latch (calls work again). */
IAD_API iad_result IAD_CALL iad_axis_cancel_reset(iad_axis* axis);
/** Is the latch set? */
IAD_API iad_result IAD_CALL iad_axis_cancel_latched(iad_axis* axis, iad_bool* latched);
/** iad_axis_cancel_latch / iad_axis_cancel_reset on every listed axis of the cell (NULL or "" = all). A group command
 *  of the cell is not refused up front: on a latched axis it ends at that axis's first wait check point (a motion wait
 *  after its decel stop), with the axis's error in the IAD_ERR_CELL details. A listed name the cell does not have never
 *  holds the call back, as for iad_cell_cancel: every listed axis it has is latched (reset), then IAD_ERR_CELL names
 *  each unknown name (operation "cancel_latch" / "cancel_reset"). */
IAD_API iad_result IAD_CALL iad_cell_cancel_latch(iad_cell* cell, const char* axes);
IAD_API iad_result IAD_CALL iad_cell_cancel_reset(iad_cell* cell, const char* axes);

/* ------------------------------------------------------------------------------------------- buses: bus lock */

/* (0.4) A hold of the bus lock and a preemptible scope belong to the calling OS thread. A binding that runs a call on
 * a helper thread (for Ctrl+C) runs the calls made inside a hold or a scope on the thread that took it. Evidence:
 * simulator tests only (UNVERIFIED on hardware). */

/** Hold the bus lock for the calling thread (ModbusRtu::bus_lock, bus_lock.hpp; the Python edition's
 *  ModbusRTU.exclusive()): until iad_bus_unlock no request of another thread (a status monitor, another axis on the
 *  line, a STOP) goes out. This thread's own requests go through (the lock is recursive: each successful iad_bus_lock
 *  needs one iad_bus_unlock). Waits in turn (FIFO; stop-class requests first) at most timeout_s (NaN: as long as it
 *  takes; 0: only when the bus is free now and nobody waits): *locked = IAD_FALSE when the time ran out (IAD_OK,
 *  nothing held; any finite timeout_s is honoured, however large). IAD_ERR_TRANSPORT when the bus is or gets closed.
 *  Keep holds short: a STOP from another thread waits for the whole hold. Give every hold back before the thread ends;
 *  closing the bus from another thread waits for it. Only this thread's own requests go through: a call that sends
 *  from threads of the core is refused inside a hold, since those threads would wait for it forever. That is every
 *  group command of a cell that uses this bus (iad_cell_status_all .. _stop_all, move_each, pause / resume_all,
 *  read_all_json, monitor_all_json, track_all, check_links,
 *  link_monitor_check_now, recover_all, coord_run_json): IAD_ERR_NOT_READY, nothing sent (0.6). */
IAD_API iad_result IAD_CALL iad_bus_lock(iad_bus* bus, double timeout_s, iad_bool* locked);
/** Give one hold back (IAD_ERR_INVALID_ARGUMENT when the calling thread holds none). */
IAD_API iad_result IAD_CALL iad_bus_unlock(iad_bus* bus);

/* -------------------------------------------------------------------------------------- buses: preemptible reads */

/** Begin a preemptible read-only scope of the calling thread on this bus (PreemptibleReads, bus_lock.hpp; the Python
 *  GUI's _GatedBus): until iad_bus_preemptible_end this thread's FC03 reads on the bus go out in frames of at most
 *  `words` registers (0 = 32; 1..125), every attempt takes the bus on its own (retries outside the lock) after `gate`
 *  (may be NULL), and this thread's writes, IAI commands, broadcasts, baud changes and line breaks on the bus are
 *  refused (IAD_ERR_UNSAFE_REFUSED, nothing sent). A long read (unit info, a backup, a register browser) so never makes
 *  a STOP of another thread wait for more than one short request; a stop path of this thread (the core's stop-class
 *  scope) still goes out. Scopes nest (the innermost one covering the bus applies); other buses are not affected.
 *  FC11 is sent as usual. */
IAD_API iad_result IAD_CALL iad_bus_preemptible_begin(iad_bus* bus, int32_t words, iad_read_gate_fn gate, void* user);
/** End the calling thread's innermost scope, which must be on this bus (IAD_ERR_INVALID_ARGUMENT otherwise, nothing
 *  ended). */
IAD_API iad_result IAD_CALL iad_bus_preemptible_end(iad_bus* bus);

/* ---------------------------------------------------------------------------------------- buses: status monitors */

/** Suspend every running status monitor of an axis on this bus (suspend_bus, monitor.hpp; the Python edition's
 *  monitor.suspend_bus): no polls until iad_bus_resume_monitors (a poll in flight is waited for, at most timeout_s;
 *  NaN = 1 s). For a sequence that changes the line under every axis (a broadcast, a rate change, a reopen). Nests;
 *  kept on this handle (closing the handle resumes them). *monitors (may be NULL) = how many were suspended. A monitor
 *  started afterwards is not suspended. */
IAD_API iad_result IAD_CALL iad_bus_suspend_monitors(iad_bus* bus, double timeout_s, int32_t* monitors);
/** Resume the monitors of this handle's last iad_bus_suspend_monitors (IAD_ERR_INVALID_ARGUMENT when none is held). */
IAD_API iad_result IAD_CALL iad_bus_resume_monitors(iad_bus* bus);

/* ------------------------------------------------------------------------------------------- buses: broadcasts */

/* A broadcast goes to address 0: every unit on the line acts, none answers; the bus is held for settle_s after the
 * frame. Refused on a gateway line (IAD_ERR_UNSAFE_REFUSED, kind LineRuleError) and inside a preemptible scope, nothing
 * sent. The frame sent goes to frame[0 .. *frame_length) when frame is not NULL; a frame_size too small for it is
 * IAD_ERR_BUFFER_TOO_SMALL before anything is sent (frame_length may be NULL). */
/** [EXPERIMENTAL] IAI 0x45 to address 0 (ModbusRtu::broadcast_iai_command; the frame IA-OS sends for the baud
 *  change, ON-WIRE): settle_s NaN = 0.2. The same safe-list and never-send rules as iad_bus_iai_command with 0x45 (the
 *  baud change, cmd 4, is on the never-send list); IAD_IAI_NO_CHECK has no effect. 10 bytes. */
IAD_API iad_result IAD_CALL iad_bus_broadcast_iai_command(iad_bus* bus, int32_t group, int32_t command, int32_t op1,
                                                          int32_t op2, uint32_t flags, double settle_s, uint8_t* frame,
                                                          size_t frame_size, size_t* frame_length);
/** An FC05 / FC06 / FC10 PDU (function code first; no address, no CRC) to address 0 (ModbusRtu::broadcast, e.g. STOP
 *  05 04 2C FF 00): never a read (IAD_ERR_INVALID_ARGUMENT). settle_s NaN = 0.02. size + 3 bytes. A CON unit may need
 *  PMSS = 1 to act on a coil (DOC); several units acting on one broadcast is UNVERIFIED. */
IAD_API iad_result IAD_CALL iad_bus_broadcast(iad_bus* bus, const uint8_t* pdu, size_t size, double settle_s,
                                              uint8_t* frame, size_t frame_size, size_t* frame_length);

/* ------------------------------------------------------------------------------------------- buses: the line */

/** State the line of this bus (ModbusRtu::set_line with a new Line; the Python edition's bus.line = Line(kind)):
 *  IAD_LINE_DIRECT, _RCON, _ROBONET or _ROBONET_GATEWAY, not probed (no gateway object; IAD_LINE_AUTO is refused: see
 *  iad_bus_detect_line). position_unit_mm: the position unit of every axis on it (0.01 or 0.001; NaN = not stated).
 *  No I/O. Every axis on the bus follows it at once (the request guard, the timeouts, the command strategy). */
IAD_API iad_result IAD_CALL iad_bus_set_line(iad_bus* bus, int32_t line, double position_unit_mm);
/** Work out the line and attach it (detect_line, gateway.hpp): IAD_LINE_AUTO probes for an RCON gateway (D0h), then a
 *  ROBONET gateway in gateway mode (3Fh), "direct" when neither answers; _RCON / _ROBONET_GATEWAY probe that gateway
 *  only (a stated kind is kept when it does not answer); _DIRECT and _ROBONET send nothing. READ-ONLY: FC03 probes,
 *  each at most timeout_s (NaN = 0.05; an RCON probe of a stated kind waits at least 0.65 s). *found (may be NULL) =
 *  the IAD_LINE_* kind attached. Run it once the baud rate is settled. */
IAD_API iad_result IAD_CALL iad_bus_detect_line(iad_bus* bus, int32_t line, double timeout_s, double position_unit_mm,
                                                int32_t* found);
/** The line of this bus (Line::snapshot as JSON, as iad_axis_line_json). No I/O. */
IAD_API iad_result IAD_CALL iad_bus_line_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required);

/* ---------------------------------------------------------------------------------------- buses: response timing */

/** The unit's response delay RTIM (EC item 8.4 / CON No.17), 0..1000 ms, in the response timeout of `slave`'s
 *  requests (ModbusRtu::set_response_delay_ms; 5 ms until set). No I/O. */
IAD_API iad_result IAD_CALL iad_bus_set_response_delay(iad_bus* bus, int32_t slave, double delay_ms);
IAD_API iad_result IAD_CALL iad_bus_response_delay(iad_bus* bus, int32_t slave, double* delay_ms);
/** How long the bus waits for the answer to `pdu` (function code first; seconds; ModbusRtu::response_timeout, DOC
 *  ME0162-12A p.4-1): 3 ms per register + RTIM + 10 x (response bytes + 8) / kbps + the USB margin, never below 0.2 s,
 *  the slow-rate floors or the line's minimum (0.3 s on a gateway line). expected_length: the whole response frame
 *  (0 = 64 bytes assumed). No I/O (takes the bus lock for a moment). */
IAD_API iad_result IAD_CALL iad_bus_response_timeout(iad_bus* bus, int32_t slave, const uint8_t* pdu, size_t size,
                                                     int32_t expected_length, double* seconds);

/* ------------------------------------------------------------------------------------------- stateless functions */

/** The core's stateless functions by name, JSON in and out (src/c_api_tables.cpp): the alarm tables and rules, the CON
 *  parameter tables and rules and the pure decoders of info.hpp, so a binding decides as the core does without a copy
 *  of its own. `args_json`: an object of named arguments (NULL or "" = {}), or an array of such objects (the output is
 *  then the array of the results, in order); "functions" lists the names. An unknown function is IAD_ERR_UNSUPPORTED
 *  (a binding can ask what a library lacks), bad arguments IAD_ERR_INVALID_ARGUMENT ((0.6) a wrongly typed nested
 *  argument and an integer outside its field's range included). Integers stay JSON integers, floats are written with a
 *  decimal point or an exponent. No I/O. */
IAD_API iad_result IAD_CALL iad_call_json(const char* function, const char* args_json, char* buffer,
                                          size_t buffer_size, size_t* required);

/* --------------------------------------------------------------------------------------- axis: state without I/O */

/* What a binding shows of an axis as the Python edition's Elecylinder attributes and methods (src/c_api_axis.cpp,
 * src/axis_state.cpp): the raw words of a status it was handed, the state the axis keeps without I/O, the reads the
 * Python edition has as methods of their own, a row write without the checksum, begin_stop and the core's constants.
 * Evidence: simulator tests only (UNVERIFIED on hardware). */

/** The raw words of a status sample this axis decoded (Axis::recent_statuses, the last 16: status reads, the samples of
 *  its waits, the status monitor's lean polls): the newest one whose decoded fields equal `like` (raw_words, DSS1, DSS2,
 *  DSSE, DIPM, DOPM, the alarm code and the position), or the newest one when `like` is NULL. `words` gets them in
 *  0x9000 order, 32-bit values high word first (PNOW, ALMC, DIPM, DOPM, DSS1, DSS2, DSSE[, STAT, VNOW, CNOW[, DEVI, STIM,
 *  SIPM, ZONS[, POSS, SSSE]]]): 8, 14, 20 or 22 words, `*count` how many (22 always fits). IAD_ERR_NOT_READY when no
 *  kept sample matches (none read yet, or 16 newer since); IAD_ERR_BUFFER_TOO_SMALL (with *count) when capacity is short.
 *  A binding builds its own status record from these words and takes the velocity from the iad_status. No I/O. */
IAD_API iad_result IAD_CALL iad_axis_last_status_words(iad_axis* axis, const iad_status* like /* may be NULL */,
                                                       uint16_t* words, size_t capacity, size_t* count);

/** What the axis keeps without I/O (Axis::state), as JSON: status_words (22, 20 or 8), motion_pending, stop_held,
 *  directory_absent, soft_limits_known, soft_limits_mm ([min, max] as stored with the 0.30 margin, or null),
 *  soft_limits_note ("not read yet" before), axis_limits (the read_axis_limits fields, or null before), ec_max_speed_mm_s
 *  and ec_max_speed_note (MAXV 7.4, else the default speed 8.10), system_code, unit_code, model_name, boot_baud_rate,
 *  max_positions (null until read), categories (the directory records read so far, as iad_axis_categories_json),
 *  p1_ceiling_mm, speed_cap_mm_s, accel_cap_g, accel_only_cap_g, decel_cap_g, push_cap_pct (from what is known; the
 *  fallbacks 100 units/s, 0.30 G, 70 % while nothing is), fast_mode, fast_notes. */
IAD_API iad_result IAD_CALL iad_axis_state_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** What kind of unit this is, as JSON {"units", "kind", "product_family", "is_ec"} (Axis::units, kind, product_family,
 *  is_ec): "mm" / "deg"; "slider", "rod", "gripper", "rotary" ... ("unknown" when the limits are not readable); "EC",
 *  "PCON", "ACON/DCON", "SCON" or "IAI controller". Reads the limits and the FC11 system code once. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_kind_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required);

/** Read the soft limits (Axis::read_soft_limits; items 7.1 / 7.2 by item ID, else the first 4 words of class 7) as JSON
 *  {"soft_limits_mm": [min, max] or null, "note"}. A slave exception gives null with the reason in note. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_read_soft_limits_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                           size_t* required);

/** Read the unit's own limits (Axis::read_axis_limits: class 7 by item ID, and 3, 8, 1) as JSON with the Python
 *  edition's AxisLimits fields (max_speed_mm_s, max_accel_g, max_decel_g, max_push_speed_mm_s, max_push_pct,
 *  min_push_pct, min_push_from_ec, push_force_factor, push_force_offset, actuator_kind, rotary_atyp, default_speed_mm_s,
 *  supports_push, supports_relative, source, max_positions, min_band, lead_mm, move_patterns). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_read_axis_limits_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                           size_t* required);

/** The unit's speed cap (Axis::read_speed_limit; reads the limits again): MAXV 7.4, else the default speed 8.10. NaN
 *  when neither is plausible (the reason is ec_max_speed_note of iad_axis_state_json). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_read_speed_limit(iad_axis* axis, double* speed);

/** Class 1 support words by name (Axis::read_support_words) as JSON {"SUPT": n, "DIAG": n, ...}, or null when the class
 *  is not in the directory or not readable. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_read_support_words_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                             size_t* required);

/** The directory records (index table at 0x0000, read once; Axis::categories) as a JSON array of {category_id,
 *  model_code, member_count, list_count, data_address, id_list_address, coil_base, member_size}. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_categories_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                     size_t* required);

/** This unit's parameter list (Axis::parameter_model(visibility)) as JSON (unit_code, system_code, model_name, family,
 *  seen_on, known, candidates, listed, hidden, verified, note): visibility 0 from the unit code only
 *  (FC11), 1 with the unit's runtime visibility rules (what the parameter writes use). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_parameter_model_json(iad_axis* axis, iad_bool visibility, char* buffer,
                                                          size_t buffer_size, size_t* required);

/** The EC user parameters of the unit's list, decoded and annotated (Axis::user_parameters), as a JSON array: per
 *  parameter no, name, key, unit, encoding, evidence, address, raw_hex, value, text, found, listed, verified_for_model,
 *  home_parameter, writable, choices, model_note, located ... (the Python edition's Elecylinder.user_parameters).
 *  iad_axis_write_parameter with key "P<no>" writes one. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_user_parameters_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                          size_t* required);

/** iad_axis_write_position_table with the checksum choice: write_checksum 0 sends the 15 data words only (the unit must
 *  update word +0xF itself; IAD_ERR_VERIFICATION when it does not), 1 is iad_axis_write_position_table. */
IAD_API iad_result IAD_CALL iad_axis_write_position_row(iad_axis* axis, int32_t row, const iad_position_entry* entry,
                                                        const iad_nv_write_options* options, iad_bool write_checksum,
                                                        iad_position_write_result* out /* may be NULL */);

/** Take Modbus control (Axis::enable_modbus_control: PMSL ON at rest; nothing written when PMSS is already 1) and, with
 *  verify, read PMSS back (IAD_ERR_NOT_READY when it stays 0). `out` gets the status read last (NULL allowed; not
 *  written without verify when PMSL was written). Refused while the axis moves, homes or pushes; a pause (DSS1 STP) at
 *  a standstill with the servo off is no refusal, a pause with the servo on is. */
IAD_API iad_result IAD_CALL iad_axis_enable_modbus_control(iad_axis* axis, iad_bool verify,
                                                           iad_status* out /* may be NULL */);

/** Send the STOP and leave it on (Axis::begin_stop): the axis decelerates and stays stopped until iad_axis_stop (which
 *  waits for standstill and releases it) or iad_axis_close; stop_held of iad_axis_state_json says it is held. Never
 *  cancelled. */
IAD_API iad_result IAD_CALL iad_axis_begin_stop(iad_axis* axis);

/** The push speed (Axis::push_speed) as JSON {"speed_mm_s", "source"}: min(speed, PSHV 8.15 / CON No.34, MXPH 7.8);
 *  speed NaN = none given. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_push_speed_json(iad_axis* axis, double speed, char* buffer, size_t buffer_size,
                                                     size_t* required);

/** How long a servo ON waits by default (Axis::servo_on_timeout) as JSON {"timeout_s", "reason"}: 3 s ("default"), or
 *  on a CON unit before its first servo ON after power-up the time from No.29 / No.30. `words` / `count`: a status block
 *  read at 0x9000 (8 to 22 words, as iad_axis_last_status_words gives them), or NULL / 0 to read the status. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_axis_servo_on_timeout_json(iad_axis* axis, const uint16_t* words, size_t count,
                                                           char* buffer, size_t buffer_size, size_t* required);

/** What closing does to the unit, keeping the handle and what the axis read (Axis::release_held): the status monitor
 *  stops and a STOP or pause this axis holds is released (a failed release is logged, not returned). The Python
 *  edition's Elecylinder.close keeps the object's state; iad_axis_close releases the handle. */
IAD_API iad_result IAD_CALL iad_axis_release_held(iad_axis* axis);

/** Replace the unit's limits this axis keeps (Axis::set_axis_limits; the Python edition's settable axis_limits) with
 *  `limits_json`, an object with the fields of iad_axis_read_axis_limits_json (absent or null: not known; min_push_pct
 *  20 and min_push_from_ec false when absent). Every later check uses them until the next read. No I/O. */
IAD_API iad_result IAD_CALL iad_axis_set_axis_limits_json(iad_axis* axis, const char* limits_json);

/** Share the non-volatile write guard's bookkeeping (the counter, the hour window and its warning, the parameters
 *  written to RAM and not yet committed) with every axis of this process given the same `unit_key`, as the Python
 *  edition keeps it per unit (a reconnect or a new object per job keeps counting). The binding chooses the key: e.g.
 *  "port:com3:1" for slave 1 on a serial port (by its name), "obj:<id>:1" for a transport object of its own. `label`
 *  names the axis in the rate warning ("COM3 slave 1", "slave 1"; NULL: "<port> slave <n>"). NULL or "" key: a fresh
 *  bookkeeping of this axis's own (the default). No I/O. */
IAD_API iad_result IAD_CALL iad_axis_set_nv_unit(iad_axis* axis, const char* unit_key, const char* label);

/** Drop the shared bookkeeping of iad_axis_set_nv_unit for every key starting with `unit_key_prefix` (NULL or "" =
 *  all), as a new process starts without it (an axis still holding one keeps it). No I/O. */
IAD_API iad_result IAD_CALL iad_nv_forget(const char* unit_key_prefix);

/** The core's constants a binding mirrors, as JSON {name: value} (the Python edition's ec module names: SETTLE_S,
 *  PUSH_SETTLE_S, SERVO_TIMEOUT_S, PARW_READY_S, NV_WARN_PER_HOUR, FAST_BAUDRATE ...): a binding can check its own copy
 *  against the core. No I/O. */
IAD_API iad_result IAD_CALL iad_constants_json(char* buffer, size_t buffer_size, size_t* required);

/* -------------------------------------------------------------------------------------- buses: shutdown, release */

/* (0.5) The rest of the bus for the Python edition's ModbusRTU on the core (src/c_api_bus.cpp): closing a bus without
 * letting go of its handle (close, then reopen), and letting go without closing (a binding's object that is collected
 * never closes a port the caller still uses). Evidence: simulator tests only (UNVERIFIED on hardware). */

/** Close the bus as ModbusRtu::close does and keep the handle: the bus's status monitors stop first (no "link lost"
 *  from a deliberate close), nothing queued is sent, the transaction in flight finishes, then the transport closes.
 *  Requests then fail with IAD_ERR_TRANSPORT until iad_bus_reopen. Any bus handle (one of iad_axis_bus / iad_cell_bus
 *  closes that axis's or port's bus). Closed already, or NULL: IAD_OK. */
IAD_API iad_result IAD_CALL iad_bus_shutdown(iad_bus* bus);
/** Release the handle only, never closing anything (also for a bus of iad_bus_open: the bus and its transport stay
 *  open while an axis, a cell or another handle uses them, and are let go with the last of them: a serial port then
 *  closes as it goes, a transport of iad_transport_open_callbacks gets release, never close). NULL or a closed handle:
 *  IAD_OK. */
IAD_API iad_result IAD_CALL iad_bus_release(iad_bus* bus);

/* ------------------------------------------------------------------------ axis: status monitor holds, port owner */

/* The rest of the status monitor for the Python edition's AxisMonitor on the core (src/c_api_monitor.cpp): holding one
 * axis's monitor (AxisMonitor.suspend; iad_bus_suspend_monitors holds every monitor of a bus) and the target and band
 * its done rule checks a sample against (AxisMonitor.is_done), and who owns the port for iad_axis_enable_fast's baud
 * change. iad_axis_monitor_info_json also gives "suspended" (polls
 * held, by this call or by a bus's), "started_at_s" (monotonic seconds of the last start, null before), and
 * "error_detail" / "last_error_detail": the error that ended the monitor and the last transient one as
 * iad_last_error_json describes an error ({"code", "kind", "message", ...}; null without one), so a binding raises the
 * same exception class a failed call gives. Evidence: simulator tests only (UNVERIFIED on hardware). */

/** Hold the axis's status monitor (AxisMonitor::suspend): no polls until iad_axis_resume_monitor (a poll in flight is
 *  waited for, at most timeout_s; NaN = 1 s). Nests; kept on this handle (closing the handle releases its holds).
 *  IAD_ERR_NOT_READY when the axis has no monitor. */
IAD_API iad_result IAD_CALL iad_axis_suspend_monitor(iad_axis* axis, double timeout_s);
/** Release this handle's innermost iad_axis_suspend_monitor (IAD_ERR_INVALID_ARGUMENT when none is held). */
IAD_API iad_result IAD_CALL iad_axis_resume_monitor(iad_axis* axis);
/** The target and band the fast done rule checks a monitor sample against (AxisMonitor::is_done): the axis's last
 *  commanded 0x9900 target and band in mm; both NaN when it knows none (no move yet, or a home return, a push or a stored
 *  row since). No monitor needed; no I/O. */
IAD_API iad_result IAD_CALL iad_axis_monitor_target(iad_axis* axis, double* target_mm, double* band_mm);
/** This handle's axis is the one user of the bus it was opened on (Axis::set_port_owner; a binding's axis object that
 *  owns its bus, the Python edition's owns_bus): iad_axis_enable_fast may then change the port's baud rate. Closing the
 *  handle still leaves the bus open. Default 0 for iad_axis_open_bus; no I/O. */
IAD_API iad_result IAD_CALL iad_axis_set_port_owner(iad_axis* axis, iad_bool owner);

/* ------------------------------------------------------------------------------------------- lines and gateways */

/* The line and the gateway objects as handles of their own, for the Python edition's gateway.Line, RconGateway and
 * RoboNetGateway on the core (src/c_api_line.cpp; line.hpp, gateway.hpp): a line made by the caller and attached to a
 * bus (its position units, the gateway and the explanations then reach every axis on that bus), the line a bus carries,
 * the read-only gateways, the probes, the serial code and the ROBONET hint. A line or gateway handle keeps the bus its
 * gateway reads through alive. Operation names are the line.hpp ones ("broadcast", "baud_change", "iai_command",
 * "iai_command2", "software_reset", "action_mode", "home_iai", "coil", "report_slave_id", "axis_control"). Line
 * refusals are IAD_ERR_UNSAFE_REFUSED (kind LineRuleError); GatewayModeError, PositionUnitError and UnsupportedLineError
 * are IAD_ERR_NOT_READY with that kind. Every gateway rule is DOC only (ME0413, ME0208): UNVERIFIED on hardware. */

/** A new line of `kind` (IAD_LINE_DIRECT, _RCON, _ROBONET or _ROBONET_GATEWAY; Line's constructor), not attached to a
 *  bus. `gateway`: NULL or the gateway behind it; `detected`: a probe found it; position_unit_mm: NaN = not stated. */
IAD_API iad_result IAD_CALL iad_line_open(int32_t kind, iad_gateway* gateway, iad_bool detected,
                                          double position_unit_mm, iad_line** out);
/** Release the handle (the line lives on while a bus carries it). NULL or closed: IAD_OK. */
IAD_API iad_result IAD_CALL iad_line_close(iad_line* line);
/** A handle on the line the bus carries now (after iad_bus_set_line / iad_bus_detect_line / iad_bus_attach_line). */
IAD_API iad_result IAD_CALL iad_bus_line(iad_bus* bus, iad_line** out);
/** Attach `line` to the bus (ModbusRtu::set_line): its guard, gateway and position units serve every axis on the bus. */
IAD_API iad_result IAD_CALL iad_bus_attach_line(iad_bus* bus, iad_line* line);
/** State what the axes on the bus's line may send over its kind's rules (Line::state_rules; the Python edition's
 *  duck-typed bus.line_rules): {"name": "rcon", "coils": false, "iai_commands": false, "broadcast": false,
 *  "report_slave_id": true}, each key optional (null: the kind's answer). The bus's request guard keeps the kind's rules.
 *  NULL, "" or "null" clears. */
IAD_API iad_result IAD_CALL iad_bus_state_line_rules(iad_bus* bus, const char* rules_json);
/** Line::snapshot as JSON (the Python edition's Line.to_dict: kind, detected, rules, gateway, position_units,
 *  unreachable_slaves, explanation). No I/O. */
IAD_API iad_result IAD_CALL iad_line_json(iad_line* line, char* buffer, size_t buffer_size, size_t* required);
/** Line::allows by operation name (std::invalid_argument for an unknown one). */
IAD_API iad_result IAD_CALL iad_line_allows(iad_line* line, const char* operation, iad_bool* allowed);
/** Line::require: IAD_ERR_UNSAFE_REFUSED with the line's message when not carried. `what`: NULL = the operation. */
IAD_API iad_result IAD_CALL iad_line_require(iad_line* line, const char* operation, const char* what);
/** The bus's guard (Line::check_request) on a PDU for `slave` (0 = broadcast). */
IAD_API iad_result IAD_CALL iad_line_check_request(iad_line* line, int32_t slave, const uint8_t* pdu, size_t size);
/** Line::require_control (slave 0: none named): RCON reads the gateway's MANU flag (cached), nothing else is sent. */
IAD_API iad_result IAD_CALL iad_line_require_control(iad_line* line, int32_t slave);
/** Line::position_unit_mm and position_unit_source of `slave` (0: the line's): NaN and "" when not known. */
IAD_API iad_result IAD_CALL iad_line_position_unit(iad_line* line, int32_t slave, double* unit_mm, char* source,
                                                   size_t source_size, size_t* required);
/** Line::set_position_unit: 0.01 or 0.001 of `slave` (0: every axis); `source` "stated" (NULL) or "read". */
IAD_API iad_result IAD_CALL iad_line_set_position_unit(iad_line* line, double unit_mm, int32_t slave,
                                                       const char* source);
/** Line::require_position_unit (control false) or Line::require_motion (control true) of `slave`. */
IAD_API iad_result IAD_CALL iad_line_require_position_unit(iad_line* line, int32_t slave, iad_bool control,
                                                           double* unit_mm);
/** Line::note_unreachable: "5,6". */
IAD_API iad_result IAD_CALL iad_line_note_unreachable(iad_line* line, const char* slaves);
/** Line::explain_unreachable of `slaves` ("5,6"; NULL: the recorded ones); "" when there is nothing to explain. */
IAD_API iad_result IAD_CALL iad_line_explain_unreachable(iad_line* line, const char* slaves, char* buffer,
                                                         size_t buffer_size, size_t* required);
/** A handle on the line's gateway; *out NULL when it has none. */
IAD_API iad_result IAD_CALL iad_line_gateway(iad_line* line, iad_gateway** out);
/** Every kind's rules, in IAD_LINE_DIRECT .. _ROBONET_GATEWAY order (line_rules_table; LineRules as JSON). No I/O. */
IAD_API iad_result IAD_CALL iad_line_rules_json(char* buffer, size_t buffer_size, size_t* required);

/** A gateway object on the bus (RconGateway: kind IAD_LINE_RCON, address 0 = D0h, cache_s NaN = 2 s;
 *  RoboNetGateway: IAD_LINE_ROBONET_GATEWAY, address and cache_s ignored). `words` / `count` (0: none) and read_at: the
 *  status already read. Nothing is sent. */
IAD_API iad_result IAD_CALL iad_gateway_open(iad_bus* bus, int32_t kind, int32_t address, const uint16_t* words,
                                             size_t count, double read_at, double cache_s, iad_gateway** out);
/** Release the handle. NULL or closed: IAD_OK. */
IAD_API iad_result IAD_CALL iad_gateway_close(iad_gateway* gateway);
/** {"kind", "address", "cache_s" (null on ROBONET), "words" (the last status read, null before one), "read_at"}. */
IAD_API iad_result IAD_CALL iad_gateway_info_json(iad_gateway* gateway, char* buffer, size_t buffer_size,
                                                  size_t* required);
/** The status (READ-ONLY): the cached one when younger than max_age_s, else a fresh read. max_age_s NaN: the
 *  gateway's default (RCON: its cache_s; ROBONET: always a fresh read); negative: a fresh read. */
IAD_API iad_result IAD_CALL iad_gateway_status(iad_gateway* gateway, double max_age_s, uint16_t* words,
                                               size_t capacity, size_t* count, double* read_at);
/** RconGateway::require_manu: the status, or IAD_ERR_NOT_READY (GatewayModeError) unless MANU. */
IAD_API iad_result IAD_CALL iad_gateway_require_manu(iad_gateway* gateway, uint16_t* words, size_t capacity,
                                                     size_t* count, double* read_at);
/** RconGateway::read_alarm (FC03 D0 03 37 08 00 02; 0 = none). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_gateway_read_alarm(iad_gateway* gateway, uint32_t* code);
/** How long the RCON gateway's MANU read is trusted (>= 0). */
IAD_API iad_result IAD_CALL iad_gateway_set_cache_s(iad_gateway* gateway, double cache_s);
/** RconGateway::read_monitor of `axes` ("0,1"; NULL or "" = all 16): {"<axis>": [15 words]}. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_gateway_read_monitor_json(iad_gateway* gateway, const char* axes, char* buffer,
                                                          size_t buffer_size, size_t* required);
/** probe_rcon_gateway (line IAD_LINE_RCON, address 0 = D0h) or probe_robonet_gateway (IAD_LINE_ROBONET_GATEWAY): one
 *  FC03 read, no retries, at most timeout_s (NaN: 0.05 s). *count 0 when no gateway answered. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_probe_gateway(iad_bus* bus, int32_t line, int32_t address, double timeout_s,
                                                  uint16_t* words, size_t capacity, size_t* count, double* read_at);
/** read_serial_code: the 0x9400 serial code of `slave` (8 words ASCII); "" when refused or empty. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_read_serial_code(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                                     size_t* required);
/** robonet_hint (UNVERIFIED heuristic); "" when there is none. No I/O. */
IAD_API iad_result IAD_CALL iad_bus_robonet_hint(iad_bus* bus, int32_t slave, iad_bool directory_timed_out,
                                                 char* buffer, size_t buffer_size, size_t* required);

/* ----------------------------------------------------------------------- ports and cells over the caller's buses */

/* The port of a bus and cells over objects a binding holds already (src/c_api_cell.cpp; port.hpp, cell.hpp), for the
 * Python and C# Cell / Port facades: a bus becomes a port (its name, its line kind, the baud rate found once), scanned,
 * identified and switched to another rate as a whole; a cell opens over such buses (the scan or the listed slaves, the
 * axes, max_axes) or gathers axes that exist already, and its group commands take the options those facades have.
 * Evidence: simulator tests only; several axes per port is UNVERIFIED on hardware. */

/** Make the bus a port (Port over an open bus; replaces an earlier one): `name` (NULL or "": the transport's), `line`
 *  (IAD_LINE_AUTO: the gateways are probed at the first scan; the kind also picks the gateway addresses and rates of the
 *  baud detection), position_unit_mm (RCON; NaN = not stated), fixed_rate (the bus's rate is used as it is, never
 *  detected). `detect_line` (may be NULL) / `user`: the binding states or detects the line itself (iad_port_line_fn);
 *  the port then never sets the bus's line. A bus not made a port gets one with the defaults at its first port call.
 *  The port lives while this bus handle does (and in the cells that use it). No I/O. */
IAD_API iad_result IAD_CALL iad_bus_set_port(iad_bus* bus, const char* name, int32_t line, double position_unit_mm,
                                             iad_bool fixed_rate, iad_port_line_fn detect_line, void* user);
/** The port of the bus as JSON {"name", "baud_rate" (null without one), "baud_detected", "line", "closed"}. No I/O. */
IAD_API iad_result IAD_CALL iad_bus_port_info_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required);
/** Detect the port's baud rate once, in place, by asking `slaves` ("1,2"; NULL or "": 1..16) and the line's gateway
 *  addresses at each rate (Port::ensure_baud_rate; nothing sent for a fixed rate or once found). *rate (may be NULL):
 *  the rate (IAD_INT_UNKNOWN for a transport without one), *detected: found by this port's detection.
 *  IAD_ERR_MODBUS_TIMEOUT when no address answers at any rate. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_ensure_baud_rate(iad_bus* bus, const char* slaves, int32_t* rate,
                                                     iad_bool* detected);
/** Work out the port's line once (Port::detect_line: the rate first, then the gateway probes or the binding's
 *  iad_port_line_fn; `force` asks again). *kind (may be NULL): the IAD_LINE_* kind the bus carries. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_port_detect_line(iad_bus* bus, iad_bool force, int32_t* kind);
/** The addresses of `slaves` ("1,5"; NULL or "": 1..16) that answer one FC03 probe each (no retries, timeout_s each;
 *  NaN = 0.05 s, a gateway line at least its own). The rate and the line are settled first (a line where no rate
 *  answers gives none); on an RCON line only the axes the gateway reports as linked are probed unless use_link_map is
 *  0 (a linked axis that does not answer is noted on the line). `found` gets up to `capacity` of them, *count how
 *  many answered. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_scan(iad_bus* bus, const char* slaves, double timeout_s, iad_bool use_link_map,
                                         int32_t* found, size_t capacity, size_t* count);
/** iad_bus_scan, then (identify) what each address says about itself (iad_bus_identify_json), as a JSON array of
 *  {"name", "port", "slave", "axis_number", "slave_id_raw", "system_code", "system_name", "unit_code", "firmware",
 *  "product_family", "family", "error", "line", "serial_code"} (the Python edition's ScanHit.to_dict). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_scan_units_json(iad_bus* bus, const char* slaves, double timeout_s,
                                                    iad_bool identify, iad_bool use_link_map, char* buffer,
                                                    size_t buffer_size, size_t* required);
/** One address's identity as one element of iad_bus_scan_units_json: FC11 (no retries; waited for up to the FC11
 *  wait), or the 0x9400 serial code on a line without FC11 (RCON). A failure is kept in "error", never returned.
 *  READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_identify_json(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                                  size_t* required);
/** The broadcast baud change of the whole line (Port::set_baud_rate): every unit on it switches. `slaves` ("1,2,3",
 *  required) must be at rest; the first verifies the switch, the others are probed at the new rate. Refused before
 *  anything is sent for a rate without an IAI code, on a gateway line, or while a listed axis moves. JSON
 *  {"port", "rate", "change" (as iad_axis_set_baud_rate's last result), "answered" {"<slave>": bool}}. */
IAD_API iad_result IAD_CALL iad_bus_set_baud_rate_all(iad_bus* bus, int32_t rate, const char* slaves,
                                                      const iad_baud_options* options, char* buffer,
                                                      size_t buffer_size, size_t* required);

/** A cell over these buses' ports (Cell::open on ports), in this order: `slaves_json` NULL, "" or "scan" scans every
 *  port (`scan_slaves` "1,2,3"; NULL or "": options' scan range; a port where nothing answers or that fails is skipped:
 *  iad_cell_port_errors_json), "[1,2]" takes these slaves on every port, {"COM3": [1], "COM4": [2, 3]} these per port
 *  (listed slaves must answer; a port not named takes none). The rates and lines are settled as each port does it
 *  (iad_bus_set_port); max_axes, identify and stop_on_fault from `options` (the rest is the buses'); every axis gets
 *  `axis_options` (NULL: the defaults). Ports left without an axis are closed. IAD_ERR_NOT_READY when no axis is found.
 *  The cell's axes are its own (iad_cell_open_axis, iad_cell_axis_ref). READ-ONLY. */
IAD_API iad_result IAD_CALL iad_cell_open_buses(iad_bus* const* buses, size_t bus_count, const char* slaves_json,
                                                const char* scan_slaves, const iad_cell_options* options,
                                                const iad_axis_options* axis_options, iad_cell** out);
/** A cell of axes that exist already (Cell::assemble), named by `names` ("COM3:1,COM3:2"; NULL or "": "<port>:<slave>"
 *  from each bus's port): nothing is scanned or sent unless options->identify. Names must be unique, at most
 *  options->max_axes (0: no limit). The cell shares the axes and their ports with their other users; iad_cell_close
 *  closes them, iad_cell_release leaves them open. */
IAD_API iad_result IAD_CALL iad_cell_open_axes(iad_axis* const* axes, size_t axis_count, const char* names,
                                               const iad_cell_options* options, iad_cell** out);
/** A handle of its own on the cell's axis `name`: unlike iad_cell_open_axis it outlives the cell handle, and closing
 *  it (iad_axis_close) never closes the axis. */
IAD_API iad_result IAD_CALL iad_cell_axis_ref(iad_cell* cell, const char* name, iad_axis** out);
/** Let go of the cell handle without closing anything (iad_cell_close closes the axes and ports): the axes and the
 *  buses stay open for their other handles (iad_cell_axis_ref, the binding's bus objects). Its iad_cell_open_axis
 *  handles go with it. NULL or released: IAD_OK. */
IAD_API iad_result IAD_CALL iad_cell_release(iad_cell* cell);
/** The ports a scanning open skipped, as JSON {port: the error as iad_last_error_json gives it}. No I/O. */
IAD_API iad_result IAD_CALL iad_cell_port_errors_json(iad_cell* cell, char* buffer, size_t buffer_size,
                                                      size_t* required);
/** A read-only group read of `axes` ("COM3:1,COM4:1"; NULL or "": all; ports in parallel): `what` "identify"
 *  (Cell::identify_all, DeviceInfo as iad_axis_identity_json), "limits" (Cell::limits_all, as iad_axis_limits_json) or
 *  "monitor" (Cell::monitor_all, as iad_axis_read_monitor_json). JSON {"results": {axis: value}, "errors": {axis: the
 *  error as iad_last_error_json gives it}}: a failed axis is reported, never returned as the call's error. */
IAD_API iad_result IAD_CALL iad_cell_read_all_json(iad_cell* cell, const char* what, const char* axes, char* buffer,
                                                   size_t buffer_size, size_t* required);

/* ---------------------------------------------------------------------------- ports: baud detection, device info */

/* (0.6) What the C# Port and DeviceIdentity facades need beyond "ports and cells over the caller's buses"
 * (src/c_api_cell.cpp): the port's own baud detection with the caller's candidates (C# Port.DetectBaudRate) and one
 * address's whole identity on a bus without an axis (C# DeviceIdentity.Read). Evidence: simulator tests only
 * (UNVERIFIED on hardware). */

/** Find the rate one of `slaves` ("1,2"; NULL or "": 1..16) answers at, in place (Port::detect_baud_rate): the bus's
 *  rate first, then `candidates` ("38400,9600"; NULL or "": the default order) in their order, one FC03 probe per
 *  address and rate, each capped by probe_timeout_s (NaN: 0.05 s for several slaves, else the transport's timeout).
 *  The port then counts its rate as found (iad_bus_port_info_json "baud_detected"; iad_bus_ensure_baud_rate sends
 *  nothing more). IAD_ERR_MODBUS_TIMEOUT (with the no-answer checklist) when nothing answers at any rate; the starting
 *  rate is restored. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_port_detect_baud_rate(iad_bus* bus, const char* slaves, const char* candidates,
                                                          double probe_timeout_s, int32_t* rate);
/** What `slave` tells about itself, read on the bus without an axis (read_device_info: FC11 once with the CTCL 3.1
 *  fallback, the index table, the identification block, the CON controller name, AXNO), as JSON with the fields of
 *  iad_axis_identity_json (no limits: nothing of class 7 is read). Parts the unit refuses or does not answer go to
 *  "errors", never returned; a transport failure is. READ-ONLY. */
IAD_API iad_result IAD_CALL iad_bus_read_device_info_json(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                                          size_t* required);

/* -------------------------------------------------------------------------------------- buses: another transport */

/* (0.6) A bus moved onto another transport in place (src/c_api_bus.cpp), for the Python edition's
 * `bus.transport = other`. Evidence: simulator tests only (UNVERIFIED on hardware). */

/** Move the bus onto `transport` in place (ModbusRtu::set_transport): the same bus keeps its settings (retries, gap,
 *  response delays), its line, frame sink and closed state, and everything keyed by it (the axes and cells on it, their
 *  recovery sessions and non-volatile write counts), so the next request of any of them goes out on `transport`. Waits
 *  for the transaction in flight; nothing is sent. The old transport is neither closed nor released (the bus keeps it
 *  until the bus goes; close it through a handle of its own). The bus takes its own reference to `transport`: release
 *  the handle when done with it. Any handle of the bus. The same transport again: IAD_OK, nothing changes. */
IAD_API iad_result IAD_CALL iad_bus_set_transport(iad_bus* bus, iad_transport* transport);

/* --------------------------------------------------------------------------- axis: recovery sessions, link state */

/* (0.6) What the Python recovery facade needs beyond "axis: recovery" (src/c_api_recovery.cpp): a session made from
 * an identity, its volatile fields written, the link state set, the port's presence and the restart signs asked of
 * the core, the cell's link monitor read back, and what queued each event (iad_event.source, IAD_EVENT_SOURCE_*), so
 * that a binding hands a link event to the callback of the call that caused it and not to a monitor's (or the
 * reverse). In the structs, from their reserved space: iad_event.source, iad_recovery_policy.require_session and
 * iad_recovery_summary.control_kept (also "control_kept" in the report's JSON). (0.7) iad_axis_session_from_info,
 * iad_axis_session_update and iad_axis_restart_signs refuse a value of the wrong JSON type, a fraction where a whole
 * number belongs and a number outside its field's range with IAD_ERR_INVALID_ARGUMENT. Evidence: simulator tests
 * only (UNVERIFIED on hardware). */

/** A session from an identity already read (recovery::session_from_info; no I/O): a recovery can then check the unit
 *  is the same one, with no PC-control / safety-speed / uptime baseline. `info_json`: a unit info's JSON
 *  (iad_axis_identity_json: "serial_number", "model_name", "system_code", "unit_code", "firmware", "product_family",
 *  "soft_limits_mm", "max_speed_mm_s", "units"; other keys are not read); NULL = the identity the axis read last
 *  (IAD_ERR_NOT_READY when it read none). Registered unless the unit already has a session, which is then kept. */
IAD_API iad_result IAD_CALL iad_axis_session_from_info(iad_axis* axis, const char* info_json, const char* name);

/** Write volatile fields of the unit's session (recovery::update_session; no I/O): a JSON object with any of "name",
 *  "baudrate", "modbus_control", "safety_speed", "servo_on", "homed", "uptime_ms", "uptime_age_s" (the uptime reading
 *  is that many seconds old), "last_target_mm" and "recovering" (check_link answers "recovering" while it is set).
 *  IAD_ERR_NOT_READY without a session; IAD_ERR_INVALID_ARGUMENT for another key or a wrong value (nothing written). */
IAD_API iad_result IAD_CALL iad_axis_session_update(iad_axis* axis, const char* fields_json);

/** Record the unit's link state (recovery::set_link_state; no I/O) and queue the matching event on the handle
 *  (IAD_EVENT_SOURCE_SET_LINK_STATE) when it changed: "link_lost" on entering "port_vanished" / "unit_silent" from a
 *  working link, "unit_restarted", "unit_replaced", "recovered" (back to "ok" from a fault). `message` / `name` (may
 *  be NULL): the event's message and axis. *changed (may be NULL): the state changed. IAD_ERR_INVALID_ARGUMENT for an
 *  unknown state. */
IAD_API iad_result IAD_CALL iad_axis_set_link_state(iad_axis* axis, const char* state, const char* message,
                                                    const char* name, iad_bool* changed /* may be NULL */);

/** Whether the axis's port is listed by the PC right now (recovery::port_present; READ-ONLY, nothing is opened): the
 *  transport's own presence (a simulator, a callback transport with a `present` callback), else a serial port's name
 *  in the port list (iad_list_ports_json, which iad_testing_set_port_list of iai_actuator_driver_testing.h
 *  replaces). *present: IAD_TRUE, IAD_FALSE, or IAD_UNKNOWN when it cannot be told (a transport without a name). */
IAD_API iad_result IAD_CALL iad_axis_port_present(iad_axis* axis, iad_tribool* present);

/** The signs that the unit restarted since its session was refreshed (recovery::restart_signs; no I/O), in words, as a
 *  JSON list of strings (empty: none). `words`: a status block as read from 0x9000 (8 to 22 words). `options_json`
 *  (may be NULL): {"new_records": [[code, detail, address, time_ms or null], ...] (alarm-history slots added since),
 *  "baud_rate_found": rate or null, "boot_baud_rate": rate (default 38400), "now_s": the core's clock
 *  (iad_clock_now of iai_actuator_driver_testing.h; default now)}. IAD_ERR_NOT_READY without a session. */
IAD_API iad_result IAD_CALL iad_axis_restart_signs(iad_axis* axis, const uint16_t* words, size_t word_count,
                                                   const char* options_json, char* buffer, size_t buffer_size,
                                                   size_t* required);

/** The cell's link monitor (iad_cell_monitor_links) as JSON (no I/O): {"running", "interval_s", "passes", "states":
 *  {name: the last LinkCheck as iad_axis_last_result_json gives it}, "errors": ["<name>: <Type>: <message>", ...]};
 *  null without one (a monitor stopped by iad_cell_stop_link_monitor is no longer reported). */
IAD_API iad_result IAD_CALL iad_cell_link_monitor_json(iad_cell* cell, char* buffer, size_t buffer_size,
                                                       size_t* required);

/* ---------------------------------------------------------------------- axis: directory readers, self-test lines */

/* (0.6) What the Python info and check facades need beyond "axis: unit information, backup, self-test"
 * (src/c_api_info.cpp): the readers of a unit's directory behind one entry point, and the lines of a running
 * self-test, so that a binding hands each line to the caller's on_item on the caller's thread with its whole detail
 * and data (the IAD_EVENT_CHECK_ITEM event carries IAD_MESSAGE_SIZE bytes of the detail and no data). The input ranges
 * per product line are iad_call_json functions ("info.input_ranges", "info.product_line", "info.ec_parameter_table").
 * (0.7) A caller record whose block passes the 65535 registers is IAD_ERR_INVALID_ARGUMENT. Evidence: simulator tests
 * only (UNVERIFIED on hardware). */

/** One reader of the unit's directory by name, with its arguments as a JSON object (NULL or "": none); READ-ONLY, FC03
 *  only, every read inside a block the unit's directory declares. The result is also kept for
 *  iad_axis_last_result_json (no second read when the buffer was too small). `records` (an argument of every function
 *  but read_directory and backup): null = the directory the axis has read (iad_axis_state_json "categories"), else read
 *  from the unit (not kept); a list of 8-number records [CI, MD, DQ, LQ, DA, IA, BA, MS] = exactly those.
 *  - "read_directory" {}: {"header": {"model_code", "words_per_record", "record_count", "rx_buffer", "tx_buffer"},
 *    "records": [[8 numbers], ...]} read from the unit; IAD_ERR_INVALID_ARGUMENT ("index table: unexpected layout
 *    IndexHeader(...)") when the header is not 8 words per record and 1..64 records.
 *  - "read_block" {"class_id", "offset" (default 0), "count" (null: to the end of the certain extent), "records"}: the
 *    raw words [n, ...]. Out of the block's readable extent: IAD_ERR_INVALID_ARGUMENT (kind std::out_of_range) and
 *    nothing is sent; a class not in the directory likewise.
 *  - "read_class" {"class_id", "max_words" (default 1024), "records"}: {"class_id", "address", "words", "member_size",
 *    "entries", "id_map_address" (null: no ID map), "items": [{"class_id", "item_id" (null: no ID map), "entry",
 *    "index", "address", "size", "raw"}, ...]}. Larger than max_words: IAD_ERR_INVALID_ARGUMENT.
 *  - "read_support_words" {"records"}: {name: value} of the class-1 words (SUPT, DIAG, ...) in member order; null when
 *    the class is not in the directory.
 *  - "read_accl" {"records"}: ACCL (identification block + 0x12, u32), null when the block is not in the directory.
 *  - "unit_info" {"sections": null (all) | [names]}: iad_axis_read_unit_info_json with the sections as a list (an
 *    empty list reads none).
 *  - "backup" {"rows": "used" (default) | "all" | [row numbers]}: iad_axis_backup_json without a file, where a row
 *    selection the unit cannot serve (a number outside its table, a negative number, any other text) is reported in
 *    "errors" ("positions: ValueError: ...") and the other sections are still read, as the Python edition's backup.
 *  An unknown function: IAD_ERR_UNSUPPORTED. */
IAD_API iad_result IAD_CALL iad_axis_info_call_json(iad_axis* axis, const char* function, const char* args_json,
                                                    char* buffer, size_t buffer_size, size_t* required);

/** The lines the iad_axis_self_test running on this handle has finished so far, from index `first` (0: all), as a
 *  JSON list of {"name", "result", "detail", "data"} in order (no I/O); [] when none runs (the finished test's lines
 *  are in its report, iad_axis_last_result_json "items"). Called from another thread while the self-test runs. */
IAD_API iad_result IAD_CALL iad_axis_self_test_items_json(iad_axis* axis, int32_t first, char* buffer,
                                                          size_t buffer_size, size_t* required);

/* ------------------------------------------------------------------------------ simulators: ports, gateway lines */

/* (0.6) The gateway lines of the simulator (C++ SimGatewayLine, sim_gateway.hpp; src/c_api_sim.cpp) and a port for a
 * bus that holds the timer resolution as a serial port does. Test and demo tooling; every gateway rule is DOC (ME0413,
 * ME0208), UNVERIFIED on hardware. (0.8) The gateway line's controls and the request hook's answer IAD_SIM_FAIL are in
 * iai_actuator_driver_testing.h. */

/** A transport handle for a bus on the unit (iad_bus_open, iad_axis_open_transport, iad_cell_open_transports): the
 *  unit's port as iad_sim_transport gives it, which also holds the 1 ms timer resolution while open, as a serial port
 *  and a callback transport do (Windows: the bus's gap and polls then sleep in 1 ms steps, not 15.6 ms). Release it once
 *  the bus has it; closing the bus closes the unit's port, releasing the bus does not. */
IAD_API iad_result IAD_CALL iad_sim_port(iad_sim* sim, iad_transport** out);
/** The same on a simulated line (iad_sim_line_open, iad_sim_gateway_line_open). */
IAD_API iad_result IAD_CALL iad_sim_line_port(iad_sim_line* line, iad_transport** out);

/** A simulated line behind a gateway, as an iad_sim_line (the other iad_sim_line_* functions take it): `kind` "rcon"
 *  (NULL or ""), "robonet" (SIO through mode) or "robonet-gateway" (gateway mode: only 3Fh answers), `units` as for
 *  iad_sim_line_open. A frame the gateway passes on reaches the addressed unit through its port (its request log and
 *  hooks apply). options_json (NULL or "" = {}): "name" ("GW"), the RCON gateway's "manu" (true), "gateway_alarm" (0),
 *  "emergency_stop" (false), "linked_only" ([]: slaves linked at the gateway that never answer), "position_unit_mm"
 *  (RCON No.207: 0.01 or 0.001, or {"slave": unit} with 0.01 for the others), "serials" ({"slave": the serial code at
 *  0x9400}; default "SIMRCON-<axis, 2 digits>"), "silent_no_access" (false; true: the axes never answer FC03 reads of
 *  the RCON no-access areas, below 0x0D00 and 0x8000-0x8FFF, so the directory times out). IAD_ERR_INVALID_ARGUMENT for
 *  another kind or key, a wrong value or an address taken twice. */
IAD_API iad_result IAD_CALL iad_sim_gateway_line_open(const char* kind, const char* options_json,
                                                      iad_sim* const* units, size_t unit_count, iad_sim_line** out);

/* ---------------------------------------------------------- library: per-thread results; cell: link monitor pass */

/* (0.7) A call's own result, and one pass of a cell's link monitor on the caller's thread (src/c_api_results.cpp).
 * iad_axis_last_result_json / iad_cell_last_result_json give the handle's last result, which a call on another thread
 * replaces (a link monitor's iad_axis_check_link, iad_cell_check_links): a binding that runs iad_axis_recover,
 * iad_axis_self_test or iad_cell_recover_all while such calls go on reads its own call's result with
 * iad_last_result_json instead. iad_cell_check_links' kept result also gives "error_details": {axis: the record of
 * iad_last_error_json} for each axis whose check failed with an error that is not a link fault (the per-axis "error"
 * state), so a binding raises that error's own class. Evidence: simulator tests only (UNVERIFIED on hardware). */

/** The result the last call on the calling thread kept on its handle: the text iad_axis_last_result_json /
 *  iad_cell_last_result_json gave right after that call, whatever calls on other threads kept since. Read it on the
 *  thread that made the call, right after it returned IAD_OK (iad_cell_recover_all and iad_cell_track_all also
 *  IAD_ERR_CELL: they keep their result before they report the failed axes). IAD_ERR_NOT_READY when no call on this
 *  thread has kept one. Does not change the last error (not even when it fails), as iad_last_error_json. */
IAD_API iad_result IAD_CALL iad_last_result_json(char* buffer, size_t buffer_size, size_t* required);

/** One pass of the cell's link monitor (iad_cell_monitor_links) on the calling thread, as a pass of its own thread:
 *  every axis it watches checked (a fault goes to the cell's fault handling), its states, passes and errors updated
 *  (iad_cell_link_monitor_json), and each state change queued as IAD_EVENT_LINK_STATE. Waits for a pass of the
 *  monitor's thread in progress. `axes` (NULL or "": every axis of the pass, by name) names checks[i]; an axis whose
 *  check threw (kept in the monitor's errors) or that the monitor does not watch gets state "error" and ok 0.
 *  *count = the axes named. IAD_ERR_NOT_READY when no link monitor runs on the cell (none started, or stopped). */
IAD_API iad_result IAD_CALL iad_cell_link_monitor_check_now(iad_cell* cell, const char* axes, iad_link_check* checks,
                                                            size_t capacity, size_t* count);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* IAI_ACTUATOR_DRIVER_H */
