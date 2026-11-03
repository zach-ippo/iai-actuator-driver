/*
 * iai_actuator_driver_testing.h - test hooks of the IAI Actuator Driver native core.
 *
 * Unofficial driver for IAI actuators. Not an IAI product. The functions declared here are exported by the same
 * library as iai_actuator_driver.h (iai_actuator_driver.dll, libiai_actuator_driver.so, .dylib), but they are NOT part
 * of its stable ABI: they are the plumbing of the bindings' own test suites (the Python edition's testing module and
 * SimulatedEC hooks, the C# simulator), a seam for the library's clock and the driver's time limits. There is no
 * stability promise: any release of the library may change, rename or remove them, whatever its ABI version says.
 * Never needed in production.
 *
 * Version rule: IAD_TESTING_ABI_VERSION counts the changes of this header. A caller compiled with it uses these
 * functions only when iad_testing_abi_version() returns exactly that number (the bindings check it before their first
 * call into this header, and refuse a library with another number); iad_testing_abi_version itself never changes.
 * The stable header's conventions apply (results, pointers, handles, strings, JSON outputs, the last error).
 *
 * Callbacks: a simulator's request hook and the clock callbacks run synchronously on the thread doing the work (the
 * caller's thread for a single axis, a core worker thread for a cell's group command), with the port's bus lock held,
 * under the rules of the stable header's transport callbacks ("callback types"): return quickly, let no exception (or
 * longjmp) cross into the library, call no library function except iad_callback_error, iad_sim_handle_frame on the
 * simulator being hooked (from its request hook) and the cancel functions. Bindings keep every callback pointer alive
 * until it is removed or the library is no longer used.
 */
#ifndef IAI_ACTUATOR_DRIVER_TESTING_H
#define IAI_ACTUATOR_DRIVER_TESTING_H

#include "iai_actuator_driver.h"

/** The version of this header: exact match with iad_testing_abi_version() (no compatibility between numbers). */
#define IAD_TESTING_ABI_VERSION 1

#ifdef __cplusplus
extern "C" {
#endif

/** IAD_TESTING_ABI_VERSION of the library's build of this header. Cannot fail. */
IAD_API uint32_t IAD_CALL iad_testing_abi_version(void);

/* ------------------------------------------------------------------------------------------- callbacks */

/** Seconds on a monotonic clock (iad_clock_use_callbacks, iad_sim_set_clock). */
typedef double(IAD_CALL* iad_clock_now_fn)(void* user);
/** Sleep at least `seconds` (a virtual clock: move on by them). */
typedef void(IAD_CALL* iad_clock_sleep_fn)(double seconds, void* user);

/* What a simulator's request hook answers (iad_sim_request_hook). */
#define IAD_SIM_PASS 0  /* the simulated unit answers as usual */
#define IAD_SIM_DROP 1  /* no answer; the frame does not reach the unit */
#define IAD_SIM_REPLY 2 /* answer with reply[0 .. *reply_length); the frame does not reach the unit */
/** Answer the Modbus exception `code` (1..255); the frame does not reach the unit. */
#define IAD_SIM_EXCEPTION(code) (0x100 | ((code)&0xFF))
/** The request fails: the write fails with IAD_ERR_TRANSPORT (TransportError, as a port that fails) whose message is
 *  the text in reply[0 .. *reply_length) (empty: a default text); the frame reaches the unit no further. A binding
 *  answers it when its hook raised, so the error reaches the caller of the bus call. */
#define IAD_SIM_FAIL 3
/** The request fails with an error of the caller's own: the write fails with IAD_ERR_DRIVER (EcDriverError) whose
 *  message is the text in reply[0 .. *reply_length), which is never retried nor taken for a link fault (a link check or
 *  a recovery ends with it; the link state stays as it was). A binding answers it when its hook raised an error that is
 *  not the port's (an assert of a test), so the caller gets that error as in the Python edition; IAD_SIM_FAIL stays
 *  the port's failure. */
#define IAD_SIM_ABORT 4

/** A simulator's request hook (iad_sim_set_request_hook; the Python edition's SimulatedEC.on_request): sees every
 *  frame written to the unit (on a line: the frames to other slaves too) and returns IAD_SIM_PASS, IAD_SIM_DROP,
 *  IAD_SIM_REPLY (reply_capacity is 256 bytes), IAD_SIM_EXCEPTION(code), IAD_SIM_FAIL or IAD_SIM_ABORT.
 *  iad_sim_handle_frame gives the unit's own answer from inside the hook (act, then drop or replace the reply). Runs
 *  after the line checks it can see (closed, unplugged, powered off, a rate the unit does not listen at); any other
 *  result is IAD_SIM_PASS. */
typedef int32_t(IAD_CALL* iad_sim_request_hook)(const uint8_t* request, size_t size, uint8_t* reply,
                                                size_t reply_capacity, size_t* reply_length, void* user);

/* ------------------------------------------------------------------------------------------- the library's clock */

/* Clock seam (C++ timing.hpp set_clock; process-wide): the bus (gaps, response deadlines, retries), the axis (polls,
 * waits, the non-volatile write window), the cancellable sleeps and the simulators' default clock read it. Waits on
 * another thread (a status monitor between polls, the bus lock, the event queues) keep real-time slices: a virtual
 * clock is for single-threaded tests (the Python edition's testing.virtual_clock). The stable header's "library's
 * clock" is this clock. */

/** The real (steady) clock again. */
IAD_API iad_result IAD_CALL iad_clock_use_real(void);
/** A virtual clock starting at start_s (NaN: 1000.0) that only moves when slept on or advanced: a timeout costs no
 *  real time. Replaces any clock installed before. */
IAD_API iad_result IAD_CALL iad_clock_use_virtual(double start_s);
/** The binding's own clock (both callbacks, or both NULL for the real clock). */
IAD_API iad_result IAD_CALL iad_clock_use_callbacks(iad_clock_now_fn now, iad_clock_sleep_fn sleep, void* user);
/** Move the virtual clock forward (IAD_ERR_NOT_READY when the virtual clock is not installed). */
IAD_API iad_result IAD_CALL iad_clock_advance(double seconds);
/** The library's clock now (seconds, arbitrary origin). */
IAD_API iad_result IAD_CALL iad_clock_now(double* out);

/* ------------------------------------------------------------------------------------------- simulator hooks */

/* The test hooks of a simulated unit (iad_sim_open in the stable header; the Python edition's SimulatedEC). */

/** The unit's state (no I/O): profile, slave, stroke_mm, max_speed_mm_s, system_code, unit_code, position_count,
 *  absolute, time_scale, servo_on, homed, pmsl, safety_speed, paused, stop_held, alarm_code, position_mm, moving,
 *  soft_limits_mm, obstacle_mm, unit_baud_rate, baud_rate, powered, present, open, reopens, serial_number, the
 *  counters (nonvolatile_writes, parameter_commits, parameter_ram_writes, software_resets, override_moves,
 *  profile_moves), accel_pattern, command_speed_mm_s, push_speed_mm_s, parw_busy, parameter_ram / parameter_nv
 *  ({"address": value}), the hooks (word_overrides, unreadable, stale_status_polls, stale_polls_served, step_s),
 *  clock_s (the unit's clock), "ls_range_mm" (as applied: EC parameter 2 takes effect at a restart),
 *  "auto_servo_off_active", "push_blip_mm" (null: none), "push_blip_len_mm", "port_name" (null: none) and
 *  "baud_history" (the PC side's rates in order: the modelled one, then every change; [] when not modelled). */
IAD_API iad_result IAD_CALL iad_sim_state_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required);
/** Every frame written to the unit, oldest first (the last 2000): [{"t": seconds on the unit's clock, "frame": "01 03
 *  90 00 ..."}]. */
IAD_API iad_result IAD_CALL iad_sim_requests_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required);
/** What the unit did ("stop", "home complete"...), oldest first (the last 2000), as a JSON array of strings. */
IAD_API iad_result IAD_CALL iad_sim_events_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required);
/** A 24 V power cycle (boot baud rate, PC control off, uptime restarted, uncommitted parameters lost). lose_home:
 *  IAD_TRUE / IAD_FALSE, IAD_UNKNOWN = lost on an incremental unit only. */
IAD_API iad_result IAD_CALL iad_sim_power_cycle(iad_sim* sim, iad_tribool lose_home);
/** The supply cut: the axis stops where it is and the unit is silent until iad_sim_power_on. */
IAD_API iad_result IAD_CALL iad_sim_power_off(iad_sim* sim);
IAD_API iad_result IAD_CALL iad_sim_power_on(iad_sim* sim, iad_tribool lose_home);
/** The USB adapter pulled: the port vanishes and the open handle dies (IAD_ERR_TRANSPORT) until replugged and
 *  reopened (iad_transport_reopen / iad_bus_reopen). */
IAD_API iad_result IAD_CALL iad_sim_unplug(iad_sim* sim);
IAD_API iad_result IAD_CALL iad_sim_replug(iad_sim* sim);
/** Serve `value` (0..0xFFFF) for register `address` instead of the modelled value; IAD_INT_UNKNOWN models it again. */
IAD_API iad_result IAD_CALL iad_sim_set_word_override(iad_sim* sim, int32_t address, int32_t value);
/** Make [start, start + count) unreadable (exception 02 on a read touching it), or readable again (every unreadable
 *  range overlapping it is removed). */
IAD_API iad_result IAD_CALL iad_sim_set_unreadable(iad_sim* sim, int32_t start, int32_t count, iad_bool unreadable);
/** After each FC10 move command at 0x9900, this many FC03 reads at 0x9000 still show PEND with MOVE clear (0 = off). */
IAD_API iad_result IAD_CALL iad_sim_set_stale_status_polls(iad_sim* sim, int32_t polls);
/** The value a one-register read returns now, the hooks applied (IAD_ERR_INVALID_ARGUMENT, kind "std::out_of_range",
 *  where a read is refused). */
IAD_API iad_result IAD_CALL iad_sim_register_value(iad_sim* sim, int32_t address, uint16_t* value);
/** The unit's own answer to one request frame (SimulatedUnit::handle_frame: it acts on it; no answer = *length 0).
 *  *length gets the size whatever the result; IAD_ERR_BUFFER_TOO_SMALL when it does not fit. */
IAD_API iad_result IAD_CALL iad_sim_handle_frame(iad_sim* sim, const uint8_t* request, size_t size, uint8_t* buffer,
                                                 size_t buffer_size, size_t* length);
/** Install (hook not NULL) or remove (NULL) the request hook. */
IAD_API iad_result IAD_CALL iad_sim_set_request_hook(iad_sim* sim, iad_sim_request_hook hook, void* user);
/** The other controls, one JSON command {"command": NAME, ...}; the result as JSON ({} when there is none):
 *  "inject_fault" {"kind": "timeout" | "crc" | "partial" | "wrong_slave", "count": 1}, "trigger_alarm" {"code"},
 *  "set_obstacle" {"mm": number | null}, "pio_move" {"target_mm", "speed_mm_s": 20, "accel_g": 0.3, "hold_start":
 *  false} -> {"started"}, "pio_release", "coil_state" {"address"} -> {"on"}, "set_coil_state" {"address", "on"},
 *  "position_row" {"row"} -> {"words"}, "set_position_row" {"row", "words": [16 numbers]}, "clear_requests",
 *  "clear_events", "set_step_s" {"step_s"} (only on a unit opened with "step_s"), "set_state" {"servo_on", "homed",
 *  "pmsl", "safety_speed"} (the level only, no action), "set_option" {"name", "value"} (an option key of iad_sim_open;
 *  not the fixed facts profile, slave, time_scale, position_mm, stroke_mm, max_speed_mm_s, system_code, unit_code,
 *  absolute, baud_rate, unit_baud_rate, serial_number, rtc, the initial pmsl / safety_speed and the hooks),
 *  "set_inputs" {"emg", "drive_cut", "enable"} (DSS1 EMGS + DSSE EMGP, STAT MPOW 0 + DSSE MPUV, DSS2 ENBS: each that
 *  becomes active switches the servo off; a servo ON is ignored while any is active; absent or null: as it is),
 *  "set_parameter_word" {"address", "value", "nv": false} (a parameter word as if written to RAM, or committed with
 *  nv; acted on at the next restart). */
IAD_API iad_result IAD_CALL iad_sim_control_json(iad_sim* sim, const char* command_json, char* buffer,
                                                 size_t buffer_size, size_t* required);

/** Every request on a simulated line, oldest first: [{"t", "slave", "frame"}] (t on the library's clock). */
IAD_API iad_result IAD_CALL iad_sim_line_requests_json(iad_sim_line* line, char* buffer, size_t buffer_size,
                                                       size_t* required);
/** The line's controls, one JSON command: "unplug", "replug", "power_cycle" {"slaves": [..] (default: all),
 *  "lose_home": true | false | null}, "clear_requests", "state" -> {"name", "slaves", "present", "open", "reopens",
 *  "baud_rate", "kind" ("direct" on a plain line)}. On a gateway line (iad_sim_gateway_line_open) also
 *  (IAD_ERR_INVALID_ARGUMENT on a plain line):
 *  - "gateway" -> {"kind", "silent_no_access", "function_codes" (the units answer these; the rest: exception 01),
 *    "dropped_broadcasts" (the broadcast frames nobody received), "rejected" (the frames a unit answered with
 *    exception 01), "gateway": null (ROBONET through mode) or {"slave", "requests" (the frames addressed to it), for
 *    RCON "manu", "alarm_code", "emergency_stop", "linked_only", "linked_slaves", "status_block" (the 11 words at
 *    0x3700), for a ROBONET in gateway mode "running"}}; frames as hex text ("D0 03 37 00 00 0B 18 38");
 *  - "set_gateway" {any of "manu", "alarm_code", "emergency_stop", "slave" (e.g. 0xD7: the gateway stops answering at
 *    D0h), "linked_only", "running", "silent_no_access"};
 *  - "gateway_handle" {"frame": hex} -> {"reply": hex or null}: the gateway station's own answer to one frame. */
IAD_API iad_result IAD_CALL iad_sim_line_control_json(iad_sim_line* line, const char* command_json, char* buffer,
                                                      size_t buffer_size, size_t* required);

/** Run the unit on `now(user)` (monotonic seconds; NULL: the library's clock, iad_clock_*, again) from now on, e.g. a
 *  test's fake clock. The unit's time goes on from where it is (the new clock is offset, nothing jumps). `now` is
 *  called with the unit's lock held, from any thread using the unit: it must not call the library. A unit opened with "step_s"
 *  keeps its stepped clock. */
IAD_API iad_result IAD_CALL iad_sim_set_clock(iad_sim* sim, iad_clock_now_fn now, void* user);

/* ------------------------------------------------------------------------------------------- driver hooks */

/** Shorten a driver time limit process-wide (the Python edition's testing.timings), so that a test of a limit that
 *  runs out does not wait the production value. `name`: "home_start_s" (a home return must visibly start within it,
 *  2 s), "parw_ready_s" (the parameter handshake must be idle before a write, 2 s), "parw_clear_s" (it must end after
 *  the PARW coil, 2 s); `seconds` >= 0, NaN restores the default; NULL or "" name restores all.
 *  IAD_ERR_INVALID_ARGUMENT for another name. */
IAD_API iad_result IAD_CALL iad_testing_set_timing(const char* name, double seconds);

/** The non-volatile write guard (Axis::testing_note_nonvolatile_write): count one non-volatile write of this axis's
 *  unit at `now_s` (seconds on the library's clock, iad_clock_now; NaN: now) as a real write is counted: the total of
 *  iad_axis_nonvolatile_writes, the hour window and its rate WARNING (one per hour above NV_WARN_PER_HOUR, logged). The
 *  bookkeeping is the unit's (iad_axis_set_nv_unit). `what` names the write in the debug log (NULL or "": "test");
 *  *warned (may be NULL) = this write gave the warning. Nothing is sent. */
IAD_API iad_result IAD_CALL iad_testing_axis_nv_record(iad_axis* axis, double now_s, const char* what,
                                                       iad_bool* warned /* may be NULL */);
/** The same bookkeeping's writes in the hour before `now_s` (the library's clock; NaN: now). No I/O, and it changes
 *  nothing: asking for a later time does not drop the counts of the time before it (iad_testing_axis_nv_record at a
 *  later time does end the window before it: it forgets the writes more than an hour before the newest one). */
IAD_API iad_result IAD_CALL iad_testing_axis_nv_last_hour(iad_axis* axis, double now_s, int32_t* count);

/** Replace the PC's serial-port enumeration process-wide (the Python edition's testing.fake_ports), so
 *  iad_list_ports_json, iad_axis_port_present, a recovery's port check and the cell's port discovery see `ports_json`:
 *  a JSON list of {"port" (the name, required), "description", "manufacturer", "hardware_id" (text, default ""),
 *  "vid", "pid" (0..65535 or null)}, ordered as the PC's list ("is_iai" may be given and is derived anyway). NULL
 *  restores the PC's enumeration. Opens nothing. IAD_ERR_INVALID_ARGUMENT for another key or a wrong value (the list
 *  is then unchanged). */
IAD_API iad_result IAD_CALL iad_testing_set_port_list(const char* ports_json);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* IAI_ACTUATOR_DRIVER_TESTING_H */
