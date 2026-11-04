// Private helpers of the C ABI (include/iai_actuator_driver.h): the handle registry, the thread-local last error and
// the mapping of C++ exceptions to iad_result codes, the struct and option conversions, the text outputs. Internal:
// not installed, not part of the ABI. Implemented in src/c_api_support.cpp; the exported functions are in
// src/c_api.cpp. The model is in docs/dev/NATIVE_ABI.md.
//
// Evidence: UNVERIFIED on hardware (the C ABI is tested on the simulator only).
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver_testing.h"  // the test hooks are exported by the same library
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "monotonic_cv.hpp"

namespace iai_actuator_driver {
namespace capi {

// ------------------------------------------------------------------------------------------ errors

/// A text or JSON output did not fit the caller's buffer (IAD_ERR_BUFFER_TOO_SMALL).
class BufferTooSmall : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// One exception as the ABI reports it: the code, the C++ class name ("SoftLimitError", "std::invalid_argument"...),
/// the message (what()) and the details of iad_last_error_json.
struct ErrorDescription {
    iad_result code = IAD_ERR_INTERNAL;
    std::string kind;
    std::string message;
    Json json;
};

/// Classify an exception (the most specific code wins: ServoBlockedError -> IAD_ERR_SERVO_BLOCKED, not NOT_READY).
ErrorDescription describe_error(const std::exception_ptr& error);

/// The code alone (never throws; IAD_ERR_INTERNAL when even the classification fails).
iad_result error_code(const std::exception_ptr& error) noexcept;

/// The calling thread's last error. set_last_error records an exception and returns its code; it never throws.
void clear_last_error() noexcept;
iad_result set_last_error(const std::exception_ptr& error) noexcept;
const char* last_error_message() noexcept;
/// The last error as JSON text; {"code":0} after a call that succeeded.
std::string last_error_json();

/// (0.7, py-fixes) The result the last call on this thread kept on its handle (iad_last_result_json): every place that
/// keeps a handle's last result passes the same text here, so the caller reads its own call's result even when a call
/// on another thread has replaced the handle's since. thread_result() is empty until a call on the thread kept one.
void keep_thread_result(const std::string& text);
std::optional<std::string> thread_result();

/// Run `body` as one exported function: IAD_OK and the last error cleared when it returns, the exception's code and
/// the last error set when it throws. Nothing escapes.
template <class F>
iad_result guarded(F&& body) noexcept {
    try {
        body();
        clear_last_error();
        return IAD_OK;
    } catch (...) {
        return set_last_error(std::current_exception());
    }
}

/// `pointer` itself, std::invalid_argument("<name> must not be NULL") when NULL.
template <class T>
T* need(T* pointer, const char* name) {
    if (!pointer) throw std::invalid_argument(std::string(name) + " must not be NULL");
    return pointer;
}

// ------------------------------------------------------------------------------------------ event queue (0.2)

/// One queued event (iad_event): the core's callbacks turned into data the binding polls.
struct Event {
    std::int32_t kind = IAD_EVENT_NONE;
    std::optional<Status> status;
    double time_s = 0.0;       ///< now_s() when queued
    double wall_time_s = 0.0;  ///< seconds since the Unix epoch
    std::string axis;
    std::string name;
    std::string cause;
    std::string message;
    std::int32_t source = 0;   ///< (0.6, py-recovery) IAD_EVENT_SOURCE_*: what queued it (0: not stated)
    std::uint64_t caller_thread = 0;  ///< (0.7) the OS id of the thread that queued it (EventQueue::push fills it when 0)
};

/// (0.7) The calling thread's OS id: GetCurrentThreadId on Windows, gettid on Linux, pthread_threadid_np on macOS (the
/// values Python's threading.get_native_id() gives), 0 where a platform has none. A binding gives a link event to the
/// call it ran on this thread (iad_event.caller_thread).
std::uint64_t native_thread_id() noexcept;

/// A bounded, thread-safe queue of events per handle (the core's threads push, the binding pops with a timeout). The
/// oldest event is dropped when full (counted and reported with the next event returned). close() wakes every waiter.
class EventQueue {
public:
    /// The kinds queued (IAD_EVENTS_*) and the capacity (1..65536); std::invalid_argument otherwise.
    void configure(std::int32_t mask, std::int32_t capacity);
    /// Queue `event` when its kind is in the mask (the time stamps are filled when 0). Never throws.
    void push(Event event) noexcept;
    /// The next event, waiting up to timeout_s (<= 0 or NaN: no wait); nullopt when none came. std::invalid_argument
    /// once closed. `sequence` / `dropped` get the event's number and the events dropped before it.
    std::optional<Event> pop(double timeout_s, std::int64_t* sequence, std::int32_t* dropped);
    void close() noexcept;

    static constexpr std::int32_t kDefaultCapacity = 1024;

private:
    struct Item {
        Event event;
        std::int64_t sequence;
    };
    std::mutex mutex_;
    ::iai_actuator_driver::detail::MonotonicCv wake_;
    std::deque<Item> items_;
    std::int32_t mask_ = IAD_EVENTS_ALL;
    std::size_t capacity_ = kDefaultCapacity;
    std::int64_t next_ = 0;
    std::int32_t dropped_ = 0;
    bool closed_ = false;
};

/// The callbacks that queue an axis monitor's events (on_change, on_done, on_event) into `queue` under `axis_name`;
/// they hold the queue weakly (a released handle makes them no-ops).
MonitorOptions queueing_callbacks(const std::shared_ptr<EventQueue>& queue, const std::string& axis_name,
                                  MonitorOptions base = {});
/// A recovery::EventCallback that queues RecoveryEvents (IAD_EVENT_LINK) with `source` (IAD_EVENT_SOURCE_*: the call
/// they belong to; 0.6, py-recovery).
recovery::EventCallback queueing_recovery_events(const std::shared_ptr<EventQueue>& queue,
                                                 std::int32_t source = IAD_EVENT_SOURCE_CELL);
/// `options` with on_fault (IAD_EVENT_FAULT) and on_event (IAD_EVENT_LINK) queueing into `queue`.
CellOptions queueing_cell_options(CellOptions options, const std::shared_ptr<EventQueue>& queue);

// ------------------------------------------------------------------------------------------ handles

/// One live axis handle. `owned`: from iad_axis_open / iad_axis_open_sim (closing the handle closes the axis);
/// otherwise from iad_cell_open_axis (`cell` = the cell's token; closing the handle leaves the axis in the cell).
struct AxisEntry {
    std::shared_ptr<Axis> axis;
    bool owned = false;
    std::uintptr_t cell = 0;
    std::mutex mutex;                        ///< guards last_result
    std::optional<std::string> last_result;  ///< JSON of the last successful write operation (iad_axis_last_result_json)
    std::shared_ptr<EventQueue> events = std::make_shared<EventQueue>();  ///< 0.2: iad_axis_next_event
    std::mutex monitor_mutex;                ///< guards hooked (and monitor_holds)
    std::weak_ptr<AxisMonitor> hooked;       ///< the monitor whose callbacks feed `events`
    /// (0.5, py-monitor) iad_axis_suspend_monitor's holds, innermost last (released by iad_axis_resume_monitor or with
    /// the handle)
    std::vector<AxisMonitor::Suspension> monitor_holds;
    /// (0.6, py-info-cli) The lines of the running or last iad_axis_self_test, in order (guarded by mutex; read by
    /// iad_axis_self_test_items_json while the test runs on another thread)
    std::vector<CheckItem> check_items;
};

/// One live cell handle.
struct CellEntry {
    std::shared_ptr<Cell> cell;
    std::shared_ptr<EventQueue> events = std::make_shared<EventQueue>();  ///< 0.2: on_fault, on_event, monitors
    std::mutex mutex;                        ///< guards the members below
    std::optional<std::string> last_result;  ///< iad_cell_last_result_json
    std::map<std::string, std::weak_ptr<AxisMonitor>> hooked;  ///< per axis: the monitor feeding `events`
    std::shared_ptr<recovery::LinkMonitor> link_monitor;      ///< iad_cell_monitor_links
};

/// One live move handle (0.2): the C++ MoveHandle and the axis object it needs alive. `call` serialises the handle's
/// own calls (MoveHandle is not thread-safe); a cancel that finds it held stops the axis directly and records it in
/// `cancelled` / `cancel_status` (guarded by `state`).
struct MoveEntry {
    MoveEntry(std::shared_ptr<Axis> a, MoveHandle h) : axis(std::move(a)), handle(std::move(h)) {}
    std::shared_ptr<Axis> axis;
    MoveHandle handle;
    std::mutex call;
    std::mutex state;
    bool cancelled = false;
    std::optional<Status> cancel_status;
};

/// Register a new handle (tokens are never reused). register_cell_axis fails with std::invalid_argument when the
/// cell was closed meanwhile.
iad_axis* register_axis(std::shared_ptr<Axis> axis);
iad_axis* register_cell_axis(std::shared_ptr<Axis> axis, iad_cell* cell);
iad_cell* register_cell(std::shared_ptr<Cell> cell);
/// 0.2: a cell entry made before the open (its event queue is wired into the CellOptions callbacks).
iad_cell* register_cell(std::shared_ptr<CellEntry> entry);
iad_move_handle* register_move(std::shared_ptr<MoveEntry> entry);

/// The entry of a live handle; std::invalid_argument for NULL, a closed or an unknown handle. The shared_ptr keeps the
/// object alive for the call even when another thread closes the handle meanwhile.
std::shared_ptr<AxisEntry> find_axis(iad_axis* handle);
std::shared_ptr<CellEntry> find_cell(iad_cell* handle);
std::shared_ptr<MoveEntry> find_move(iad_move_handle* handle);
/// 0.6 (fixes): find_cell for a group command (one that runs the axes on core threads): EcNotReadyError
/// (IAD_ERR_NOT_READY) while the calling thread holds the bus lock of one of the cell's ports (iad_bus_lock), whose
/// requests the core threads would wait for forever (c_api_cell.cpp).
std::shared_ptr<CellEntry> find_cell_to_run(iad_cell* handle);

/// Remove a handle from the registry and return its entry (nullptr when it was not live). `other_kind` is set when the
/// token is a live handle of the other kind (an axis passed to iad_cell_close or the reverse): nothing is removed.
/// release_cell also removes the axis handles iad_cell_open_axis gave for that cell (`released_axes`, may be NULL, gets
/// their entries so that their event queues can be closed).
std::shared_ptr<AxisEntry> release_axis(iad_axis* handle, bool* other_kind);
std::shared_ptr<CellEntry> release_cell(iad_cell* handle, bool* other_kind,
                                        std::vector<std::shared_ptr<AxisEntry>>* released_axes = nullptr);
std::shared_ptr<MoveEntry> release_move(iad_move_handle* handle, bool* other_kind);

// ------------------------------------------------------------------------------------------ text outputs

/// Copy `text` into a fixed char array (NUL-terminated, truncated at a UTF-8 character boundary).
void copy_text(char* dst, std::size_t size, const std::string& text) noexcept;
template <std::size_t N>
void copy_text(char (&dst)[N], const std::string& text) noexcept {
    copy_text(dst, N, text);
}

/// Check the (buffer, buffer_size, required) tail before any I/O: required must not be NULL, buffer only when
/// buffer_size is 0.
void check_text_output(const char* buffer, std::size_t buffer_size, const std::size_t* required);
/// Write `text` with its NUL, or "" (when buffer_size > 0) and BufferTooSmall. *required = text size + 1 either way.
void write_text_output(const std::string& text, char* buffer, std::size_t buffer_size, std::size_t* required);

// ------------------------------------------------------------------------------------------ conversions (inputs)

/// "COM3:1, COM4:1" -> {"COM3:1", "COM4:1"} (trimmed; with `unique`, duplicates dropped keeping the first); NULL or
/// "" -> {}. std::invalid_argument for an empty entry ("a,,b").
std::vector<std::string> split_list(const char* text, bool unique = true);
/// The cancel entry points (iad_cell_cancel, iad_cell_cancel_latch, iad_cell_cancel_reset): `act` runs on every axis of
/// `axes` (NULL or "" = all) the cell has; afterwards each listed name it does not have is an error of its own (a
/// std::out_of_range each) and the call ends with a CellError named `operation` (IAD_ERR_CELL; the names are the "axes"
/// of iad_last_error_json). One bad name never refuses the whole call: a cancel (a Ctrl+C, a watchdog) must reach the
/// waiting axes whatever else the list holds, as iad_cell_stop_all does. Only unknown names: nothing is done.
void cancel_listed(Cell& cell, const char* axes, const char* operation, const std::function<void(Axis&)>& act);

/// IAD_METHOD_DEFAULT -> nullopt, IAD_METHOD_COIL / IAD_METHOD_IAI -> the method; std::invalid_argument otherwise.
std::optional<CommandMethod> to_method(std::int32_t method);
/// 0.2: the IAD_METHOD_* of a method.
std::int32_t from_method(CommandMethod method) noexcept;
/// 0.2: IAD_COMPLETION_DEFAULT -> nullopt, _SAFE / _FAST; std::invalid_argument otherwise. And back.
std::optional<Completion> to_completion(std::int32_t completion);
std::int32_t from_completion(Completion completion) noexcept;
/// 0.2: IAD_ACCEL_DEFAULT -> nullopt, _TRAPEZOID / _S_MOTION / _FILTER; std::invalid_argument otherwise.
std::optional<AccelPattern> to_accel_pattern(std::int32_t pattern);
/// 0.2: NaN or 0 -> nullopt, 0.01 / 0.001 -> the unit; std::invalid_argument otherwise.
std::optional<double> to_position_unit(double unit_mm);
/// 0.2: IAD_LINE_* and a position unit -> LineOptions; std::invalid_argument for an unknown kind or unit.
LineOptions to_line_options(std::int32_t line, double position_unit_mm);
/// NaN -> nullopt; other values unchanged.
std::optional<double> opt(double value);
/// 0.8: opt() for the structs without an _init function (iad_sequence_point, iad_axis_move), where 0 also means
/// "not set": NaN or 0 -> nullopt.
std::optional<double> opt_nonzero(double value);
/// NaN -> `fallback`.
double or_default(double value, double fallback);
/// A timeout input: NaN -> nullopt (the default); otherwise finite and > 0, else std::invalid_argument naming `what`
/// ("nothing sent": every converter runs before the command). 0 and -1 ("wait forever" in VB / LabVIEW) are refused.
std::optional<double> timeout_arg(double timeout_s, const char* what);
/// Non-finite -> std::invalid_argument("<name> must be a finite number").
double finite(double value, const char* name);

AxisOptions to_axis_options(const iad_axis_options* options);
/// Every check of the open options happens here, before any port is opened.
OpenOptions to_open_options(const iad_open_options* options, const iad_axis_options* axis_options);
SimulatedUnitOptions to_sim_options(const iad_sim_options* options, SimProfile profile, int slave);
/// NULL -> the profile name "slider"; std::invalid_argument for an unknown name.
SimProfile to_sim_profile(const char* profile);
MoveOptions to_move_options(const iad_move_options* options, const MoveOptions& defaults);
PositionMoveOptions to_position_move_options(const iad_position_move_options* options);
NvWriteOptions to_nv_options(const iad_nv_write_options* options);
ParameterWriteOptions to_param_options(const iad_param_write_options* options);
BaudChangeOptions to_baud_options(const iad_baud_options* options);
SoftwareResetOptions to_reset_options(const iad_reset_options* options);

/// (0.8) The settings of the axis commands with an options struct (NULL: the defaults below, what the _init functions
/// write; NaN in a double = its default).
struct ServoArgs {
    bool wait = true;
    std::optional<double> timeout_s;  ///< nullopt: the core's default
};
struct HomeArgs {
    bool wait = true;
    std::optional<double> timeout_s;     ///< nullopt: the core's default
    std::optional<CommandMethod> method;  ///< nullopt: the axis option
};
struct StopArgs {
    bool wait = false;
    double timeout_s = 5.0;
};
struct WaitIdleArgs {
    std::optional<double> target;
    std::optional<double> band;
    double timeout_s = 10.0;
    std::optional<Completion> completion;  ///< nullopt: the axis option
};
struct ChangeArgs {
    bool wait = false;
    std::optional<double> timeout_s;  ///< nullopt: the core's default
};
/// iad_axis_cancel_paused_move's timeout when none is given.
inline constexpr double kCancelPausedTimeoutS = 5.0;
ServoArgs to_servo_args(const iad_servo_options* options);
HomeArgs to_home_args(const iad_home_options* options);
StopArgs to_stop_args(const iad_stop_options* options);
WaitIdleArgs to_wait_idle_args(const iad_wait_idle_options* options);
ChangeArgs to_change_args(const iad_change_options* options);
double to_cancel_paused_timeout(const iad_cancel_paused_move_options* options);
/// (1.1) The settings of iad_transport_open_serial_options (NULL: the defaults iad_serial_options_init writes; 0 in
/// baud_rate, data_bits, stop_bits and NaN in read_timeout_s = its default). Every check happens here, before any port
/// is opened: std::invalid_argument for a value the port cannot take.
struct SerialArgs {
    int baud_rate = kDefaultBaudRate;
    double read_timeout_s = kDefaultReadTimeoutS;
    SerialFraming framing;  ///< 8N1
};
SerialArgs to_serial_args(const iad_serial_options* options);

CellOptions to_cell_options(const iad_cell_options* options, const iad_axis_options* axis_options);
PositionEntry from_c(const iad_position_entry& entry);

// ------------------------------------------------------------------------------------------ conversions (outputs)

void to_c(const AxisOptions& options, iad_axis_options* out);
void to_c(const Status& status, iad_status* out);
/// A status that was not read (raw_words 0, unknown values NaN / IAD_UNKNOWN).
void unknown_status(iad_status* out);
void to_c(const AxisRanges& ranges, iad_limits* out);
void to_c(const DeviceInfo& info, iad_identity* out);
void to_c(const PositionEntry& entry, iad_position_entry* out);
void to_c(const ParameterInfo& info, iad_parameter* out);
void to_c(const ParameterWriteResult& result, iad_param_write_result* out);
void to_c(const PositionWriteResult& result, iad_position_write_result* out);
void to_c(const SoftwareResetResult& result, iad_reset_result* out);
void to_c(const BaudChangeResult& result, iad_baud_result* out);

/// Fill a group command's results in the order of `names` (at most `capacity`; `results` may be NULL) and set *count
/// (may be NULL) to the number of axes handled. An axis neither in `ok` nor in `errors` (a move refused because of
/// another axis) gets IAD_OK with an unknown status (raw_words 0). Returns true when one or more axes failed.
bool fill_cell_results(const std::vector<std::string>& names, const std::map<std::string, Status>& ok,
                       const std::map<std::string, std::exception_ptr>& errors, iad_cell_result* results,
                       std::size_t capacity, std::size_t* count);

// ------------------------------------------------------------------------------------------ 0.2 conversions

/// "all" -> every row, "0,1,5" -> these rows, NULL / "" -> none; std::invalid_argument for anything else.
recovery::RowSelection to_row_selection(const char* rows);
/// "1,2,3" -> {1, 2, 3}; NULL / "" -> {}; std::invalid_argument for anything that is not a list of integers.
std::vector<int> to_int_list(const char* text, const char* what);
/// The JSON text of a backup (std::invalid_argument when it is not a JSON object).
Json parse_backup(const char* text);

PushOptions to_push_options(const iad_push_options* options);
SequenceOptions to_sequence_options(const iad_sequence_options* options);
SequencePoint from_c(const iad_sequence_point& point);
/// (0.8) The moves of iad_cell_move_each (an array of sized iad_axis_move): each axis NUL-terminated within its
/// field (never cut), NaN or 0 in a double / 0 in an enum = the iad_move_options_init default.
std::vector<AxisMove> to_axis_moves(const iad_axis_move* moves, std::size_t count);
MonitorOptions to_monitor_options(const iad_monitor_options* options);
recovery::TrackOptions to_track_options(const iad_track_options* options);
recovery::RecoveryPolicy to_recovery_policy(const iad_recovery_policy* policy);
CheckOptions to_check_options(const iad_check_options* options);

void to_c(const PushResult& result, iad_push_result* out);
void to_c(const MoveHandle& handle, iad_move_info* out);
void to_c(const MonitorSample& sample, iad_monitor_sample* out);
void to_c(const FastModeSummary& summary, iad_fast_summary* out);
void to_c(const recovery::LinkCheck& check, iad_link_check* out);
void to_c(const recovery::RecoveryReport& report, iad_recovery_summary* out);
void to_c(const CheckReport& report, iad_check_summary* out);
/// One popped event (the time stamps, sequence and dropped count included).
void to_c(const Event& event, std::int64_t sequence, std::int32_t dropped, iad_event* out);
/// IAD_EVENT_NONE and nothing else.
void no_event(iad_event* out);
/// `parts` joined with "; ".
std::string join(const std::vector<std::string>& parts);

// ------------------------------------------------------------------------------------------ 0.3: object handles

/// The kinds of the 0.3 handles (the registry's error messages name them: "a transport handle"...).
enum class ObjectKind { Transport, Bus, Sim, SimLine, Line /* 0.5 py-gateway */, Gateway /* 0.5 py-gateway */ };

/// Register a 0.3 handle (a token from the same counter as axis / cell / move handles, never reused).
std::uintptr_t register_object(ObjectKind kind, std::shared_ptr<void> entry);
/// The entry of a live 0.3 handle of `kind`; std::invalid_argument for NULL, a closed, unknown or other-kind handle.
std::shared_ptr<void> find_object(const void* handle, ObjectKind kind);
/// Remove a 0.3 handle and return its entry (nullptr when it was not live; `other_kind` set, nothing removed, when
/// the token is a live handle of another kind).
std::shared_ptr<void> release_object(const void* handle, ObjectKind kind, bool* other_kind);

/// (0.8) How far past this library's struct a caller's struct_size may reach (a later ABI's fields): the "Sized structs"
/// rule of the header. A struct_size beyond it, or not a multiple of the struct's alignment (every sized struct's size
/// is one), is not a size the caller was compiled with (an unset field holding stack garbage): std::invalid_argument
/// before any byte of the caller's memory past its first size is read or a stride is taken from it.
constexpr std::size_t kSizedMaxExtra = 4096;

/// std::invalid_argument for a struct_size that cannot be a struct's size: `what` names the field ("iad_move_options",
/// "points[0]"), `own` is this library's sizeof(T) and `alignment` its alignof(T).
inline void check_struct_size_bound(std::uint32_t size, std::size_t own, std::size_t alignment, const std::string& what) {
    if (size % alignment != 0 || size > own + kSizedMaxExtra)
        throw std::invalid_argument(what + ".struct_size is " + std::to_string(size) +
                                    ", not the size of a struct this library can read (a multiple of " +
                                    std::to_string(alignment) + ", at most " + std::to_string(own + kSizedMaxExtra) +
                                    "): set it to sizeof the struct, then call the _init function");
}

/// Copy a caller's sized input struct (it starts with uint32_t struct_size, uint32_t reserved_header): the bytes up to
/// min(struct_size, sizeof(T)) over a zeroed T. std::invalid_argument when struct_size is below `minimum` (the size of
/// the ABI that introduced T), is not a size check_struct_size_bound accepts, or the header's reserved word is set;
/// std::logic_error (IAD_ERR_UNSUPPORTED) when bytes beyond sizeof(T) are not zero: a field of a later ABI this library
/// does not know (docs/dev/NATIVE_ABI.md).
template <class T>
T read_sized(const T* input, const char* name, std::size_t minimum = sizeof(T)) {
    T out;
    std::memset(&out, 0, sizeof out);
    std::uint32_t size = 0;
    std::memcpy(&size, input, sizeof size);
    if (size < minimum)
        throw std::invalid_argument(std::string(name) + ".struct_size is " + std::to_string(size) + ", needs at least " +
                                    std::to_string(minimum) + " (set it with the _init function)");
    check_struct_size_bound(size, sizeof(T), alignof(T), name);
    std::memcpy(&out, input, std::min<std::size_t>(size, sizeof(T)));
    if (out.reserved_header != 0) throw std::invalid_argument(std::string(name) + ".reserved_header must be 0");
    const auto* bytes = reinterpret_cast<const unsigned char*>(input);
    for (std::size_t i = sizeof(T); i < size; ++i)
        if (bytes[i] != 0)
            throw std::logic_error(std::string(name) + " byte " + std::to_string(i) +
                                   " is set: a field of a later ABI than this library's");
    return out;
}

/// Throw std::logic_error naming `field` when any of the `count` reserved 64-bit slots is set (finding 1 of the ABI
/// review: a later ABI's field is never ignored silently).
void check_reserved(const std::uint64_t* slots, std::size_t count, const char* field);

// ------------------------------------------------------------------------------------------ 0.8: sized input structs

/// (0.8) The first size of each sized input struct (its 0.8 size: the 1.0 layout reset), the "Sized structs" rule of
/// the header: read_input refuses a smaller struct_size, and write_sized writes this much into a caller's struct whose
/// struct_size it does not know. A later minor that appends fields keeps these numbers and adds the new sizes to
/// write_sized's known sizes (c_api.cpp checks them against sizeof today).
template <class T>
struct SizedBase;
#define IAD_SIZED_BASE(type, size)                   \
    template <>                                      \
    struct SizedBase<type> {                         \
        static constexpr std::size_t value = (size); \
    }
IAD_SIZED_BASE(iad_axis_options, 104);
IAD_SIZED_BASE(iad_open_options, 72);
IAD_SIZED_BASE(iad_sim_options, 80);
IAD_SIZED_BASE(iad_move_options, 80);
IAD_SIZED_BASE(iad_position_move_options, 64);
IAD_SIZED_BASE(iad_nv_write_options, 56);
IAD_SIZED_BASE(iad_param_write_options, 64);
IAD_SIZED_BASE(iad_baud_options, 56);
IAD_SIZED_BASE(iad_reset_options, 56);
IAD_SIZED_BASE(iad_cell_options, 96);
IAD_SIZED_BASE(iad_position_entry, 128);
IAD_SIZED_BASE(iad_push_options, 72);
IAD_SIZED_BASE(iad_sequence_point, 56);
IAD_SIZED_BASE(iad_sequence_options, 88);
IAD_SIZED_BASE(iad_monitor_options, 72);
IAD_SIZED_BASE(iad_track_options, 48);
IAD_SIZED_BASE(iad_recovery_policy, 80);
IAD_SIZED_BASE(iad_check_options, 72);
IAD_SIZED_BASE(iad_axis_move, 224);
IAD_SIZED_BASE(iad_servo_options, 56);
IAD_SIZED_BASE(iad_home_options, 56);
IAD_SIZED_BASE(iad_stop_options, 56);
IAD_SIZED_BASE(iad_wait_idle_options, 72);
IAD_SIZED_BASE(iad_change_options, 56);
IAD_SIZED_BASE(iad_cancel_paused_move_options, 48);
IAD_SIZED_BASE(iad_serial_options, 64);  // (1.1) added by 1.1: its first size is its 1.1 size
#undef IAD_SIZED_BASE

namespace detail_sized {
template <class T, class = void>
struct has_reserved0 : std::false_type {};
template <class T>
struct has_reserved0<T, std::void_t<decltype(std::declval<const T&>().reserved0)>> : std::true_type {};
}  // namespace detail_sized

/// (0.8) A caller's sized input struct (read_sized with its first size), its reserved slots (reserved0, reserved[])
/// checked: std::invalid_argument for a struct_size below the first size or a set reserved_header, std::logic_error
/// (IAD_ERR_UNSUPPORTED) for a set reserved slot or a set byte beyond this library's struct.
template <class T>
T read_input(const T* input, const std::string& name) {
    T in = read_sized(input, name.c_str(), SizedBase<T>::value);
    if constexpr (detail_sized::has_reserved0<T>::value)
        if (in.reserved0 != 0)
            throw std::logic_error(name + ".reserved0 is set: a field of a later ABI than this library's");
    check_reserved(in.reserved, sizeof in.reserved / sizeof in.reserved[0], (name + ".reserved").c_str());
    return in;
}

/// (0.8) An array of sized input structs: the stride is the first element's struct_size (at least the first size, and a
/// size check_struct_size_bound accepts), and every element has that struct_size (std::invalid_argument otherwise);
/// each element is read as read_input does.
/// `name` names the array ("moves", "points").
template <class T>
std::vector<T> read_input_array(const T* first, std::size_t count, const std::string& name) {
    std::vector<T> out;
    if (count == 0) return out;
    std::uint32_t stride = 0;
    std::memcpy(&stride, first, sizeof stride);
    if (stride < SizedBase<T>::value)
        throw std::invalid_argument(name + "[0].struct_size is " + std::to_string(stride) + ", needs at least " +
                                    std::to_string(SizedBase<T>::value) + " (the array's stride: sizeof the element)");
    check_struct_size_bound(stride, sizeof(T), alignof(T), name + "[0]");  // before a stride is taken from it
    out.reserve(count);
    const auto* bytes = reinterpret_cast<const unsigned char*>(first);
    for (std::size_t i = 0; i < count; ++i) {
        const auto* item = reinterpret_cast<const T*>(bytes + i * static_cast<std::size_t>(stride));
        std::uint32_t size = 0;
        std::memcpy(&size, item, sizeof size);
        const std::string at = name + "[" + std::to_string(i) + "]";
        if (size != stride)
            throw std::invalid_argument(at + ".struct_size is " + std::to_string(size) + ", " + name +
                                        "[0]'s is " + std::to_string(stride) + ": every element has the same size");
        out.push_back(read_input(item, at));
    }
    return out;
}

/// (0.8) Write `value` into a caller's sized struct (an _init function; the outputs iad_axis_get_options and
/// iad_axis_read_position_table). The caller sets struct_size = sizeof(its struct) before the call, or leaves the struct
/// zeroed (0: the first size, which every caller's struct has): the library writes up to the size it finds there when
/// that is a size of T it knows (today only sizeof(T); a later minor adds the sizes T grew to), else up to the first
/// size. struct_size then holds the size written; the caller's bytes past it are not touched. A struct_size that is
/// neither 0 nor a size the caller was compiled with (stack garbage that happens to be a known size) is the caller's
/// error: the header says to set it first.
template <class T>
void write_sized(T* out, T value) {
    static_assert(sizeof(T) >= SizedBase<T>::value, "a sized struct only grows");
    std::uint32_t size = 0;
    std::memcpy(&size, out, sizeof size);
    const std::size_t n = size == sizeof(T) ? sizeof(T) : SizedBase<T>::value;
    value.struct_size = static_cast<std::uint32_t>(n);
    value.reserved_header = 0;
    std::memcpy(out, &value, n);
}

/// (0.8) A zero-filled T (a sized struct before its fields are set; write_sized sets struct_size).
template <class T>
T zeroed() {
    T value;
    std::memset(&value, 0, sizeof value);
    return value;
}

}  // namespace capi
}  // namespace iai_actuator_driver
