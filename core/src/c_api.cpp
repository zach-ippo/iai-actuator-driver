// C ABI over the C++ core (include/iai_actuator_driver.h; the model is in docs/NATIVE_ABI.md). Built into the static
// library and, with IAD_BUILD_SHARED, exported from the shared library (python build.py shared, or the CMake option
// IAI_ACTUATOR_DRIVER_BUILD_SHARED).
//
// Phase 1: every function of the header. Each one is a thin call of the C++ method its header comment names, wrapped
// in capi::guarded (c_api_impl.hpp): every exception is caught, turned into an iad_result and the calling thread's last
// error; nothing crosses the boundary. Pointers, handles and options are checked before any I/O, so a refused call
// sends nothing (and iad_axis_open / iad_cell_open open no port). The registry, the error mapping and the struct
// conversions are in src/c_api_support.cpp.
//
// Evidence: UNVERIFIED on hardware (tested on the simulator: tests/test_c_api.cpp, tests/c_smoke.c against the DLL).

#include "iai_actuator_driver.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "iai_actuator_driver/version.hpp"

// ------------------------------------------------------------------------------------------ layout checks
// The header promises structs without implicit padding (a packed layout, as 32-bit LabVIEW uses, equals the natural
// one). (0.8) Outputs are frozen within an ABI major version; a sized input only grows at its end, from the first size
// recorded in capi::SizedBase (checked here to be today's size). A size here changes only with IAD_ABI_VERSION_MAJOR.
#define IAD_CHECK_LAYOUT(type, size)                                                                       \
    static_assert(sizeof(type) == (size), #type ": size changed (the ABI freezes it) or padding appeared"); \
    static_assert(alignof(type) == 8, #type ": alignment must be 8")
#define IAD_CHECK_SIZED(type, size)                                                                          \
    IAD_CHECK_LAYOUT(type, size);                                                                            \
    static_assert(iai_actuator_driver::capi::SizedBase<type>::value == (size), #type ": first size");        \
    static_assert(offsetof(type, struct_size) == 0 && offsetof(type, reserved_header) == 4, #type ": header")

// inputs (0.8: sized, a struct_size + reserved_header prefix)
IAD_CHECK_SIZED(iad_axis_options, 104);
IAD_CHECK_SIZED(iad_open_options, 72);
IAD_CHECK_SIZED(iad_sim_options, 80);
IAD_CHECK_SIZED(iad_move_options, 80);
IAD_CHECK_SIZED(iad_position_move_options, 64);
IAD_CHECK_SIZED(iad_nv_write_options, 56);
IAD_CHECK_SIZED(iad_param_write_options, 64);
IAD_CHECK_SIZED(iad_baud_options, 56);
IAD_CHECK_SIZED(iad_reset_options, 56);
IAD_CHECK_SIZED(iad_cell_options, 96);
IAD_CHECK_SIZED(iad_position_entry, 128);
IAD_CHECK_SIZED(iad_push_options, 72);
IAD_CHECK_SIZED(iad_sequence_point, 56);
IAD_CHECK_SIZED(iad_sequence_options, 88);
IAD_CHECK_SIZED(iad_monitor_options, 72);
IAD_CHECK_SIZED(iad_track_options, 48);
IAD_CHECK_SIZED(iad_recovery_policy, 80);
IAD_CHECK_SIZED(iad_check_options, 72);
IAD_CHECK_SIZED(iad_axis_move, 224);
IAD_CHECK_SIZED(iad_servo_options, 56);
IAD_CHECK_SIZED(iad_home_options, 56);
IAD_CHECK_SIZED(iad_stop_options, 56);
IAD_CHECK_SIZED(iad_wait_idle_options, 72);
IAD_CHECK_SIZED(iad_change_options, 56);
IAD_CHECK_SIZED(iad_cancel_paused_move_options, 48);
IAD_CHECK_SIZED(iad_serial_options, 64);  // (1.1)
// outputs (0.8: fixed for the major; iad_status 64 reserved bytes, iad_range 32, iad_limits 64, iad_event 32)
IAD_CHECK_LAYOUT(iad_status, 208);
IAD_CHECK_LAYOUT(iad_range, 88);
IAD_CHECK_LAYOUT(iad_limits, 1096);
IAD_CHECK_LAYOUT(iad_identity, 616);
IAD_CHECK_LAYOUT(iad_parameter, 288);
IAD_CHECK_LAYOUT(iad_param_write_result, 320);
IAD_CHECK_LAYOUT(iad_position_write_result, 136);
IAD_CHECK_LAYOUT(iad_reset_result, 352);
IAD_CHECK_LAYOUT(iad_baud_result, 128);
IAD_CHECK_LAYOUT(iad_cell_result, 632);
IAD_CHECK_LAYOUT(iad_push_result, 272);
IAD_CHECK_LAYOUT(iad_move_info, 296);
IAD_CHECK_LAYOUT(iad_monitor_sample, 304);
IAD_CHECK_LAYOUT(iad_event, 736);
IAD_CHECK_LAYOUT(iad_fast_summary, 304);
IAD_CHECK_LAYOUT(iad_link_check, 800);
IAD_CHECK_LAYOUT(iad_recovery_summary, 768);
IAD_CHECK_LAYOUT(iad_check_summary, 56);
IAD_CHECK_LAYOUT(iad_motion_caps, 336);
static_assert(IAD_AXIS_NAME_SIZE == 128, "IAD_AXIS_NAME_SIZE is 128 from 0.8");
static_assert(offsetof(iad_axis_options, stroke_min) == 8 && offsetof(iad_axis_options, push_settle_s) == 72 &&
                  offsetof(iad_axis_options, completion) == 80 && offsetof(iad_open_options, line) == 36 &&
                  offsetof(iad_open_options, position_unit_mm) == 40 && offsetof(iad_open_options, fast) == 48 &&
                  offsetof(iad_move_options, accel_pattern) == 44 && offsetof(iad_move_options, decel_g) == 48 &&
                  offsetof(iad_move_options, completion) == 56 && offsetof(iad_cell_options, line) == 60 &&
                  offsetof(iad_cell_options, position_unit_mm) == 64 && offsetof(iad_position_entry, push_power) == 64,
              "inputs: field offsets changed");
static_assert(offsetof(iad_axis_move, position) == 8 && offsetof(iad_axis_move, decel_g) == 48 &&
                  offsetof(iad_axis_move, accel_pattern) == 56 && offsetof(iad_axis_move, axis) == 64 &&
                  offsetof(iad_axis_move, reserved) == 192,
              "iad_axis_move: field offsets changed");
static_assert(offsetof(iad_event, kind) == 232 && offsetof(iad_event, axis) == 240 &&
                  offsetof(iad_event, message) == 432 && offsetof(iad_event, source) == 688 &&
                  offsetof(iad_event, index) == 692 && offsetof(iad_event, caller_thread) == 696,
              "iad_event: field offsets changed");
static_assert(offsetof(iad_status, position_mm) == 0 && offsetof(iad_status, alarm_code) == 32 &&
                  offsetof(iad_status, rcon) == 140 && offsetof(iad_status, reserved) == 144,
              "iad_status: field offsets changed");
static_assert(offsetof(iad_cell_result, code) == 208 && offsetof(iad_cell_result, index) == 212 &&
                  offsetof(iad_cell_result, axis) == 216 && offsetof(iad_cell_result, message) == 344,
              "iad_cell_result: field offsets changed");
static_assert(offsetof(iad_position_write_result, push_power) == 56 && offsetof(iad_position_write_result, row) == 84 &&
                  offsetof(iad_position_write_result, reserved) == 104,
              "iad_position_write_result: field offsets changed");
static_assert(offsetof(iad_limits, units) == 968 && offsetof(iad_recovery_summary, axis) == 320,
              "iad_limits / iad_recovery_summary: field offsets changed");
static_assert(sizeof(iad_result) == 4 && sizeof(iad_bool) == 4 && sizeof(iad_tribool) == 4, "32-bit result / bool");
// The alarm levels are the C++ AlarmLevel in declaration order.
static_assert(static_cast<int>(iai_actuator_driver::AlarmLevel::None) == IAD_ALARM_LEVEL_NONE &&
                  static_cast<int>(iai_actuator_driver::AlarmLevel::Message) == IAD_ALARM_LEVEL_MESSAGE &&
                  static_cast<int>(iai_actuator_driver::AlarmLevel::OperationCancelled) ==
                      IAD_ALARM_LEVEL_OPERATION_CANCELLED &&
                  static_cast<int>(iai_actuator_driver::AlarmLevel::ColdStart) == IAD_ALARM_LEVEL_COLD_START &&
                  static_cast<int>(iai_actuator_driver::AlarmLevel::Unknown) == IAD_ALARM_LEVEL_UNKNOWN,
              "IAD_ALARM_LEVEL_* must follow AlarmLevel");
// (0.8) The rule of the defaults: 0 is the default of every enumeration an input takes, and IAD_UNKNOWN is 2 (an input
// value of 0 is false and any other is true, so a Visual Basic True, -1, is true); unknown ints are IAD_INT_UNKNOWN.
static_assert(IAD_FALSE == 0 && IAD_TRUE == 1 && IAD_UNKNOWN == 2 && IAD_INT_UNKNOWN == INT32_MIN,
              "iad_tribool: 0 false, 1 true, 2 unknown");
static_assert(IAD_METHOD_DEFAULT == 0 && IAD_METHOD_COIL == 1 && IAD_METHOD_IAI == 2 && IAD_METHOD_IAI2 == 3 &&
                  IAD_METHOD_REGISTER == 4,
              "IAD_METHOD_*: 0 is the default");
static_assert(IAD_TEACH_TABLE == 0 && IAD_BAUD_DIRECT == 0 && IAD_ACCEL_DEFAULT == 0 && IAD_COMPLETION_DEFAULT == 0 &&
                  IAD_LINE_AUTO == 0 && IAD_EVENT_NONE == 0 && IAD_ALARM_LEVEL_NONE == 0,
              "every enumeration's 0 is its default / none");

#undef IAD_CHECK_SIZED
#undef IAD_CHECK_LAYOUT

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Keep the JSON of a write operation's result on the handle (iad_axis_last_result_json reads it without I/O).
template <class T>
void keep_result(AxisEntry& entry, const T& result) {
    std::string text = to_json(result).dump();
    keep_thread_result(text);  // (0.7, py-fixes) iad_last_result_json
    std::lock_guard<std::mutex> lock(entry.mutex);
    entry.last_result = std::move(text);
}

/// One row of iad_axis_parameters_json / iad_axis_read_parameter_json as the Python edition's parameters() has it: a
/// named parameter has no raw-item fields (unsigned, signed, words, named), a note only when there is one and a symbol
/// only when it has one (the CON parameters'). A raw item keeps every field.
Json parameter_row_json(const ParameterInfo& p) {
    const Json full = to_json(p);
    if (p.number == 0) return full;
    Json row = Json::object();
    for (const auto& kv : full.as_object()) {
        const std::string& key = kv.first;
        if (key == "unsigned" || key == "signed" || key == "words" || key == "named") continue;
        if ((key == "symbol" || key == "note") && (kv.second.is_null() || kv.second == Json(""))) continue;
        row.set(key, kv.second);
    }
    return row;
}

/// Write `json` into the caller's buffer (the JSON output rules).
void write_json(const Json& json, char* buffer, std::size_t buffer_size, std::size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

/// A group command's CellResults into the results array; CellError when one or more axes failed.
void finish(const CellResults<Status>& r, iad_cell_result* results, std::size_t capacity, std::size_t* count) {
    if (fill_cell_results(r.names, r.results, r.errors, results, capacity, count))
        throw CellError(r.operation, r.messages, r.errors);
}

/// "COM3:1" -> ("COM3", 1); std::invalid_argument for a slave outside 1..247 or not a number.
std::pair<std::string, int> port_and_slave(const std::string& item) {
    const std::size_t colon = item.rfind(':');
    const std::string port = item.substr(0, colon);
    const std::string number = item.substr(colon + 1);
    bool digits = !number.empty() && number.size() <= 3;
    for (char c : number) digits = digits && c >= '0' && c <= '9';
    const int slave = digits ? std::stoi(number) : 0;
    if (port.empty() || slave < 1 || slave > 247)
        throw std::invalid_argument("'" + item + "': expected PORT:SLAVE with a slave address 1..247");
    return {port, slave};
}

/// The defaults of a move (iad_move_options_init, iad_jog_options_init).
void move_defaults(const MoveOptions& d, iad_move_options* options) {
    iad_move_options c = zeroed<iad_move_options>();
    c.speed = d.speed;
    c.accel_g = d.accel_g;
    c.band = d.band;
    c.timeout_s = d.timeout_s ? *d.timeout_s : std::numeric_limits<double>::quiet_NaN();
    c.wait = d.wait ? IAD_TRUE : IAD_FALSE;
    c.accel_pattern = IAD_ACCEL_DEFAULT;
    c.decel_g = std::numeric_limits<double>::quiet_NaN();
    c.completion = IAD_COMPLETION_DEFAULT;
    write_sized(options, c);
}

/// One parameter write (iad_axis_write_parameter, iad_axis_write_parameter_number).
void write_parameter(AxisEntry& e, const char* key, const ParamValue& value, const iad_param_write_options* options,
                     iad_param_write_result* out) {
    if (value.empty()) throw std::invalid_argument("value is empty");
    const ParameterWriteResult r = e.axis->write_parameter(std::string(key), value, to_param_options(options));
    keep_result(e, r);
    if (out) to_c(r, out);
}

}  // namespace

extern "C" {

// ============================================================================================ library

uint32_t IAD_CALL iad_abi_version(void) {
    static_assert(IAD_ABI_VERSION == ((IAD_ABI_VERSION_MAJOR << 16) | IAD_ABI_VERSION_MINOR), "IAD_ABI_VERSION");
    return static_cast<uint32_t>(IAD_ABI_VERSION);
}

uint32_t IAD_CALL iad_testing_abi_version(void) { return static_cast<uint32_t>(IAD_TESTING_ABI_VERSION); }

const char* IAD_CALL iad_library_version(void) { return kVersion; }

const char* IAD_CALL iad_last_error_message(void) { return last_error_message(); }

iad_result IAD_CALL iad_last_error_json(char* buffer, size_t buffer_size, size_t* required) {
    // Does not change the last error (not even when it fails), so it is not guarded.
    try {
        check_text_output(buffer, buffer_size, required);
        write_text_output(last_error_json(), buffer, buffer_size, required);
        return IAD_OK;
    } catch (...) {
        return error_code(std::current_exception());
    }
}

const char* IAD_CALL iad_result_name(iad_result code) {
    switch (code) {
        case IAD_OK: return "IAD_OK";
        case IAD_ERR_SOFT_LIMIT: return "IAD_ERR_SOFT_LIMIT";
        case IAD_ERR_NOT_READY: return "IAD_ERR_NOT_READY";
        case IAD_ERR_ALARM: return "IAD_ERR_ALARM";
        case IAD_ERR_MOTION_TIMEOUT: return "IAD_ERR_MOTION_TIMEOUT";
        case IAD_ERR_MODBUS_TIMEOUT: return "IAD_ERR_MODBUS_TIMEOUT";
        case IAD_ERR_FRAME: return "IAD_ERR_FRAME";
        case IAD_ERR_TRANSPORT: return "IAD_ERR_TRANSPORT";
        case IAD_ERR_VERIFICATION: return "IAD_ERR_VERIFICATION";
        case IAD_ERR_NONVOLATILE_REFUSED: return "IAD_ERR_NONVOLATILE_REFUSED";
        case IAD_ERR_INVALID_ARGUMENT: return "IAD_ERR_INVALID_ARGUMENT";
        case IAD_ERR_UNSUPPORTED: return "IAD_ERR_UNSUPPORTED";
        case IAD_ERR_INTERNAL: return "IAD_ERR_INTERNAL";
        case IAD_ERR_BUFFER_TOO_SMALL: return "IAD_ERR_BUFFER_TOO_SMALL";
        case IAD_ERR_SERVO_BLOCKED: return "IAD_ERR_SERVO_BLOCKED";
        case IAD_ERR_MOTION_INCOMPLETE: return "IAD_ERR_MOTION_INCOMPLETE";
        case IAD_ERR_SLAVE_EXCEPTION: return "IAD_ERR_SLAVE_EXCEPTION";
        case IAD_ERR_COMMAND_REJECTED: return "IAD_ERR_COMMAND_REJECTED";
        case IAD_ERR_UNSAFE_REFUSED: return "IAD_ERR_UNSAFE_REFUSED";
        case IAD_ERR_CELL: return "IAD_ERR_CELL";
        case IAD_ERR_MODBUS: return "IAD_ERR_MODBUS";
        case IAD_ERR_DRIVER: return "IAD_ERR_DRIVER";
        case IAD_ERR_RECOVERY: return "IAD_ERR_RECOVERY";
        case IAD_ERR_UNIT_REPLACED: return "IAD_ERR_UNIT_REPLACED";
        case IAD_ERR_FILE: return "IAD_ERR_FILE";
        case IAD_ERR_CANCELLED: return "IAD_ERR_CANCELLED";
        default: return "IAD_ERR_UNKNOWN";
    }
}

iad_result IAD_CALL iad_list_ports_json(char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        Json ports = Json::array();
        for (const SerialPortInfo& p : list_ports()) ports.push_back(to_json(p));
        write_json(ports, buffer, buffer_size, required);
    });
}

// ============================================================================================ option defaults

iad_result IAD_CALL iad_axis_options_init(iad_axis_options* options) {
    return guarded([&] { to_c(AxisOptions{}, need(options, "options")); });
}

iad_result IAD_CALL iad_open_options_init(iad_open_options* options) {
    return guarded([&] {
        need(options, "options");
        const OpenOptions d;
        iad_open_options c = zeroed<iad_open_options>();
        c.read_timeout_s = d.read_timeout_s;
        c.min_gap_s = d.bus.min_gap_s;
        c.slave = d.slave;
        c.baud_rate = 0;
        c.retries = d.bus.retries;
        c.line = IAD_LINE_AUTO;
        c.position_unit_mm = std::numeric_limits<double>::quiet_NaN();
        c.fast = d.fast ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_sim_options_init(iad_sim_options* options) {
    return guarded([&] {
        need(options, "options");
        const SimulatedUnitOptions d;
        iad_sim_options c = zeroed<iad_sim_options>();
        c.time_scale = d.time_scale;
        c.position_mm = std::numeric_limits<double>::quiet_NaN();
        c.stroke_mm = std::numeric_limits<double>::quiet_NaN();
        c.max_speed_mm_s = std::numeric_limits<double>::quiet_NaN();
        c.baud_rate = 0;
        c.pmsl = d.pmsl ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_move_options_init(iad_move_options* options) {
    return guarded([&] { move_defaults(MoveOptions{}, need(options, "options")); });
}

iad_result IAD_CALL iad_jog_options_init(iad_move_options* options) {
    return guarded([&] { move_defaults(MoveOptions::jog(), need(options, "options")); });
}

iad_result IAD_CALL iad_position_move_options_init(iad_position_move_options* options) {
    return guarded([&] {
        need(options, "options");
        const PositionMoveOptions d;
        iad_position_move_options c = zeroed<iad_position_move_options>();
        c.timeout_s = std::numeric_limits<double>::quiet_NaN();
        c.speed_pct = std::numeric_limits<double>::quiet_NaN();
        c.wait = d.wait ? IAD_TRUE : IAD_FALSE;
        c.allow_above_caps = d.allow_above_caps ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_nv_write_options_init(iad_nv_write_options* options) {
    return guarded([&] {
        need(options, "options");
        const NvWriteOptions d;
        iad_nv_write_options c = zeroed<iad_nv_write_options>();
        c.allow_nonvolatile = d.allow_nonvolatile ? IAD_TRUE : IAD_FALSE;
        c.force = d.force ? IAD_TRUE : IAD_FALSE;
        c.allow_unsafe = d.allow_unsafe ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_param_write_options_init(iad_param_write_options* options) {
    return guarded([&] {
        need(options, "options");
        const ParameterWriteOptions d;
        iad_param_write_options c = zeroed<iad_param_write_options>();
        c.commit = d.commit ? IAD_TRUE : IAD_FALSE;
        c.allow_dangerous = d.allow_dangerous ? IAD_TRUE : IAD_FALSE;
        c.allow_unlisted = d.allow_unlisted ? IAD_TRUE : IAD_FALSE;
        c.allow_home_params = d.allow_home_params ? IAD_TRUE : IAD_FALSE;
        c.force = d.force ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_baud_options_init(iad_baud_options* options) {
    return guarded([&] {
        need(options, "options");
        const BaudChangeOptions d;
        iad_baud_options c = zeroed<iad_baud_options>();
        c.settle_s = d.settle_s;
        c.method = d.method == BaudChangeMethod::Break ? IAD_BAUD_BREAK : IAD_BAUD_DIRECT;
        c.allow_unverified_code = d.allow_unverified_code ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_reset_options_init(iad_reset_options* options) {
    return guarded([&] {
        need(options, "options");
        const SoftwareResetOptions d;
        iad_reset_options c = zeroed<iad_reset_options>();
        c.timeout_s = d.timeout_s;
        c.reset_wait_s = d.reset_wait_s;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_cell_options_init(iad_cell_options* options) {
    return guarded([&] {
        need(options, "options");
        const CellOptions d;
        iad_cell_options c = zeroed<iad_cell_options>();
        c.read_timeout_s = d.read_timeout_s;
        c.min_gap_s = d.bus.min_gap_s;
        c.scan_timeout_s = d.scan_timeout_s;
        c.max_axes = d.max_axes.value_or(0);  // 0 = no limit
        c.baud_rate = 0;
        c.retries = d.bus.retries;
        c.scan_first = d.scan_slaves.empty() ? 1 : d.scan_slaves.front();
        c.scan_last = d.scan_slaves.empty() ? kMaxAxes : d.scan_slaves.back();
        c.identify = d.identify ? IAD_TRUE : IAD_FALSE;
        c.stop_on_fault = d.stop_on_fault ? IAD_TRUE : IAD_FALSE;
        c.line = IAD_LINE_AUTO;
        c.position_unit_mm = std::numeric_limits<double>::quiet_NaN();
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_position_entry_init(iad_position_entry* entry) {
    return guarded([&] { to_c(PositionEntry{}, need(entry, "entry")); });
}

iad_result IAD_CALL iad_servo_options_init(iad_servo_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_servo_options c = zeroed<iad_servo_options>();
        c.timeout_s = kNaN;
        c.wait = ServoArgs{}.wait ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_home_options_init(iad_home_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_home_options c = zeroed<iad_home_options>();
        c.timeout_s = kNaN;
        c.wait = HomeArgs{}.wait ? IAD_TRUE : IAD_FALSE;
        c.method = IAD_METHOD_DEFAULT;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_stop_options_init(iad_stop_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_stop_options c = zeroed<iad_stop_options>();
        c.timeout_s = kNaN;
        c.wait = StopArgs{}.wait ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_wait_idle_options_init(iad_wait_idle_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_wait_idle_options c = zeroed<iad_wait_idle_options>();
        c.target = kNaN;
        c.band = kNaN;
        c.timeout_s = kNaN;
        c.completion = IAD_COMPLETION_DEFAULT;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_change_options_init(iad_change_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_change_options c = zeroed<iad_change_options>();
        c.timeout_s = kNaN;
        c.wait = ChangeArgs{}.wait ? IAD_TRUE : IAD_FALSE;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_cancel_paused_move_options_init(iad_cancel_paused_move_options* options) {
    return guarded([&] {
        need(options, "options");
        iad_cancel_paused_move_options c = zeroed<iad_cancel_paused_move_options>();
        c.timeout_s = kNaN;
        write_sized(options, c);
    });
}

// ============================================================================================ axis: session

iad_result IAD_CALL iad_axis_open(const char* port, const iad_open_options* options,
                                  const iad_axis_options* axis_options, iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        need(port, "port");
        if (!*port) throw std::invalid_argument("port name is empty");
        const OpenOptions o = to_open_options(options, axis_options);  // every check before the port opens
        auto axis = std::make_shared<Axis>(Axis::open(std::string(port), o));
        *out = register_axis(std::move(axis));
    });
}

iad_result IAD_CALL iad_axis_open_sim(const char* profile, const iad_sim_options* sim,
                                      const iad_open_options* options, const iad_axis_options* axis_options,
                                      iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const SimProfile p = to_sim_profile(profile);
        const OpenOptions o = to_open_options(options, axis_options);
        auto unit = std::make_shared<SimulatedUnit>(to_sim_options(sim, p, o.slave));
        auto axis = std::make_shared<Axis>(Axis::open(std::static_pointer_cast<Transport>(unit), o));
        *out = register_axis(std::move(axis));
    });
}

iad_result IAD_CALL iad_axis_close(iad_axis* axis) {
    return guarded([&] {
        if (!axis) return;
        bool other_kind = false;
        const std::shared_ptr<AxisEntry> entry = release_axis(axis, &other_kind);
        if (other_kind)
            throw std::invalid_argument("another kind of handle was passed to iad_axis_close (use iad_cell_close / "
                                        "iad_move_handle_close)");
        if (!entry) return;
        entry->events->close();  // wakes an iad_axis_next_event waiting on another thread
        if (entry->owned) entry->axis->close();  // a cell's axis stays in the cell, unchanged
    });
}

iad_result IAD_CALL iad_axis_connect(iad_axis* axis, iad_bool take_control, iad_identity* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const DeviceInfo info = e->axis->connect(take_control != 0);
        if (out) to_c(info, out);
    });
}

iad_result IAD_CALL iad_axis_identify(iad_axis* axis, iad_identity* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        to_c(e->axis->identify(), out);
    });
}

iad_result IAD_CALL iad_axis_identity_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const std::optional<DeviceInfo> info = e->axis->device_info();
        if (!info) throw EcNotReadyError("no identity read yet: call iad_axis_connect or iad_axis_identify first");
        write_json(to_json(*info), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_get_options(iad_axis* axis, iad_axis_options* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        to_c(e->axis->options(), need(out, "out"));
    });
}

iad_result IAD_CALL iad_axis_set_options(iad_axis* axis, const iad_axis_options* options) {
    return guarded([&] {
        const auto e = find_axis(axis);
        e->axis->set_options(to_axis_options(options));
    });
}

iad_result IAD_CALL iad_axis_baud_rate(iad_axis* axis, int32_t* rate, iad_bool* detected) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(rate, "rate");
        *rate = e->axis->baud_rate().value_or(0);
        if (detected) *detected = e->axis->baud_detected() ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_axis_modbus_control(iad_axis* axis, iad_bool on) {
    return guarded([&] {
        const auto e = find_axis(axis);
        if (on)
            e->axis->enable_modbus_control(true);
        else
            e->axis->disable_modbus_control();
    });
}

// ============================================================================================ axis: reading

iad_result IAD_CALL iad_axis_status(iad_axis* axis, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        to_c(e->axis->status(), out);
    });
}

iad_result IAD_CALL iad_axis_limits(iad_axis* axis, iad_limits* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        to_c(e->axis->limits(), out);
    });
}

iad_result IAD_CALL iad_axis_limits_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(to_json(e->axis->limits()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_alarm_history_json(iad_axis* axis, iad_bool include_empty, char* buffer,
                                                     size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        Json records = Json::array();
        for (const AlarmRecord& r : e->axis->read_alarm_history(include_empty != 0)) records.push_back(to_json(r));
        write_json(records, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_max_positions(iad_axis* axis, int32_t* count) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(count, "count");
        *count = e->axis->max_positions();
    });
}

iad_result IAD_CALL iad_axis_read_position_table(iad_axis* axis, int32_t row, iad_position_entry* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        to_c(e->axis->read_position_table(row), out);
    });
}

// ============================================================================================ axis: control

iad_result IAD_CALL iad_axis_servo(iad_axis* axis, iad_bool on, const iad_servo_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = on ? find_axis_to_wait(axis) : find_axis(axis);  // 0.3: the latch
        const ServoArgs a = to_servo_args(options);
        const std::optional<Status> st = e->axis->servo(on != 0, a.wait, a.timeout_s);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_home(iad_axis* axis, const iad_home_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const HomeArgs a = to_home_args(options);
        const std::optional<Status> st = e->axis->home(a.wait, a.timeout_s, a.method);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_reset_alarm(iad_axis* axis, int32_t method, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const std::optional<CommandMethod> m = to_method(method);
        const Status st = e->axis->reset_alarm(m);
        if (out) to_c(st, out);
    });
}

// ============================================================================================ axis: motion

iad_result IAD_CALL iad_axis_check_move(iad_axis* axis, double position, const iad_move_options* options,
                                        iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const MoveOptions o = to_move_options(options, MoveOptions{});
        const Status st = e->axis->check_move(finite(position, "position"), o);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_axis_move_to(iad_axis* axis, double position, const iad_move_options* options,
                                     iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions{});
        const std::optional<Status> st = e->axis->move_to(finite(position, "position"), o);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_move_by(iad_axis* axis, double delta, const iad_move_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions{});
        const std::optional<Status> st = e->axis->move_by(finite(delta, "delta"), o);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_jog(iad_axis* axis, double delta, const iad_move_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions::jog());
        const std::optional<Status> st = e->axis->jog(finite(delta, "delta"), o);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_move_to_position(iad_axis* axis, int32_t row, const iad_position_move_options* options,
                                              iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const PositionMoveOptions o = to_position_move_options(options);
        const std::optional<Status> st = e->axis->move_to_position(row, o);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_change_speed(iad_axis* axis, double speed, const iad_change_options* options,
                                          iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const ChangeArgs a = to_change_args(options);
        const std::optional<Status> st = e->axis->change_speed(finite(speed, "speed"), a.wait, a.timeout_s);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_wait_idle(iad_axis* axis, const iad_wait_idle_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const WaitIdleArgs a = to_wait_idle_args(options);
        // IAD_COMPLETION_DEFAULT: the axis option (the overload without a Completion follows it)
        const Status st = a.completion ? e->axis->wait_idle(a.target, a.band, a.timeout_s, *a.completion)
                                       : e->axis->wait_idle(a.target, a.band, a.timeout_s);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_axis_stop(iad_axis* axis, const iad_stop_options* options, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const StopArgs a = to_stop_args(options);
        const Status st = e->axis->stop(a.wait, a.timeout_s);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_axis_pause(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->pause(); });
}

iad_result IAD_CALL iad_axis_resume(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->resume(); });
}

iad_result IAD_CALL iad_axis_cancel_paused_move(iad_axis* axis, const iad_cancel_paused_move_options* options,
                                                iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const Status st = e->axis->cancel_paused_move(to_cancel_paused_timeout(options));
        if (out) to_c(st, out);
    });
}

// ============================================================================================ axis: position table

iad_result IAD_CALL iad_axis_write_position_table(iad_axis* axis, int32_t row, const iad_position_entry* entry,
                                                  const iad_nv_write_options* options,
                                                  iad_position_write_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const PositionEntry pe = from_c(*need(entry, "entry"));
        const PositionWriteResult r = e->axis->write_position_table(row, pe, to_nv_options(options));
        keep_result(*e, r);
        if (out) to_c(r, out);
    });
}

iad_result IAD_CALL iad_axis_teach_position(iad_axis* axis, int32_t row, int32_t method,
                                            const iad_nv_write_options* options, iad_position_write_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        TeachMethod m = TeachMethod::Table;
        if (method == IAD_TEACH_COIL)
            m = TeachMethod::Coil;
        else if (method != IAD_TEACH_TABLE)
            throw std::invalid_argument("method must be IAD_TEACH_TABLE or IAD_TEACH_COIL, got " +
                                        std::to_string(method));
        const PositionWriteResult r = e->axis->teach_position(row, m, to_nv_options(options));
        keep_result(*e, r);
        if (out) to_c(r, out);
    });
}

// ============================================================================================ axis: parameters

iad_result IAD_CALL iad_axis_parameters_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        Json list = Json::array();
        for (const ParameterInfo& p : e->axis->parameters()) list.push_back(parameter_row_json(p));
        write_json(list, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_parameter(iad_axis* axis, const char* key, iad_parameter* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(key, "key");
        need(out, "out");
        to_c(e->axis->read_parameter(std::string(key)), out);
    });
}

iad_result IAD_CALL iad_axis_read_parameter_json(iad_axis* axis, const char* key, char* buffer, size_t buffer_size,
                                                 size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(key, "key");
        check_text_output(buffer, buffer_size, required);
        write_json(parameter_row_json(e->axis->read_parameter(std::string(key))), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_write_parameter(iad_axis* axis, const char* key, const char* value,
                                             const iad_param_write_options* options, iad_param_write_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(key, "key");
        write_parameter(*e, key, ParamValue::parse(need(value, "value")), options, out);
    });
}

iad_result IAD_CALL iad_axis_write_parameter_number(iad_axis* axis, const char* key, double value,
                                                    const iad_param_write_options* options,
                                                    iad_param_write_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(key, "key");
        write_parameter(*e, key, ParamValue::of(finite(value, "value")), options, out);
    });
}

// ============================================================================================ axis: non-volatile guard

iad_result IAD_CALL iad_axis_nonvolatile_writes(iad_axis* axis, int64_t* total, int32_t* last_hour,
                                                iad_bool* uncommitted) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(total, "total");
        *total = static_cast<int64_t>(e->axis->nonvolatile_writes());
        if (last_hour) *last_hour = e->axis->nonvolatile_writes_last_hour();
        if (uncommitted) *uncommitted = e->axis->has_uncommitted_parameters() ? IAD_TRUE : IAD_FALSE;
    });
}

// ============================================================================================ axis: unit

iad_result IAD_CALL iad_axis_set_baud_rate(iad_axis* axis, int32_t rate, const iad_baud_options* options,
                                           iad_baud_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const BaudChangeOptions o = to_baud_options(options);
        const BaudChangeResult r = e->axis->set_baud_rate(rate, o);
        keep_result(*e, r);
        if (out) to_c(r, out);
    });
}

iad_result IAD_CALL iad_axis_software_reset(iad_axis* axis, const iad_reset_options* options, iad_reset_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const SoftwareResetOptions o = to_reset_options(options);
        const SoftwareResetResult r = e->axis->software_reset(o);
        keep_result(*e, r);
        if (out) to_c(r, out);
    });
}

iad_result IAD_CALL iad_axis_last_result_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        std::string text;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            if (!e->last_result) throw EcNotReadyError("no write operation has succeeded on this handle yet");
            text = *e->last_result;
        }
        write_text_output(text, buffer, buffer_size, required);
    });
}

// ============================================================================================ cell

iad_result IAD_CALL iad_cell_open(const char* ports, const iad_cell_options* options,
                                  const iad_axis_options* axis_options, iad_cell** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        // every check before a port opens
        auto entry = std::make_shared<CellEntry>();
        const CellOptions o = queueing_cell_options(to_cell_options(options, axis_options), entry->events);
        const std::vector<std::string> items = split_list(ports);
        std::size_t with_slave = 0;
        for (const auto& item : items) with_slave += item.find(':') != std::string::npos;
        std::map<std::string, std::vector<int>> slaves_by_port;
        if (with_slave != 0) {
            if (with_slave != items.size())
                throw std::invalid_argument("ports: either every entry names a slave (\"COM3:1\") or none does");
            for (const auto& item : items) {
                const auto ps = port_and_slave(item);
                slaves_by_port[ps.first].push_back(ps.second);
            }
        }
        if (items.empty())
            entry->cell = std::make_shared<Cell>(Cell::open(o));
        else if (with_slave == 0)
            entry->cell = std::make_shared<Cell>(Cell::open(items, o));
        else
            entry->cell = std::make_shared<Cell>(Cell::open(slaves_by_port, o));
        *out = register_cell(std::move(entry));
    });
}

iad_result IAD_CALL iad_cell_open_sim(const char* profiles, int32_t slaves_per_line, const iad_sim_options* sim,
                                      const iad_cell_options* options, const iad_axis_options* axis_options,
                                      iad_cell** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (slaves_per_line < 1 || slaves_per_line > kMaxAxes)
            throw std::invalid_argument("slaves_per_line must be 1..16, got " + std::to_string(slaves_per_line));
        std::vector<std::string> names = split_list(profiles, false);
        if (names.empty()) names.push_back("slider");
        std::vector<SimProfile> kinds;
        for (const auto& n : names) kinds.push_back(to_sim_profile(n.c_str()));
        auto entry = std::make_shared<CellEntry>();
        const CellOptions o = queueing_cell_options(to_cell_options(options, axis_options), entry->events);
        std::vector<int> slaves;
        for (int s = 1; s <= slaves_per_line; ++s) slaves.push_back(s);
        std::map<std::string, std::shared_ptr<Transport>> lines;
        for (std::size_t i = 0; i < kinds.size(); ++i) {
            const std::string line = "SIM" + std::to_string(i + 1);
            lines[line] = SimLine::with_slaves(slaves, to_sim_options(sim, kinds[i], 1), line);
        }
        entry->cell = std::make_shared<Cell>(Cell::open(lines, slaves, o));
        *out = register_cell(std::move(entry));
    });
}

iad_result IAD_CALL iad_cell_close(iad_cell* cell) {
    return guarded([&] {
        if (!cell) return;
        bool other_kind = false;
        std::vector<std::shared_ptr<AxisEntry>> axes;
        const std::shared_ptr<CellEntry> entry = release_cell(cell, &other_kind, &axes);
        if (other_kind)
            throw std::invalid_argument("another kind of handle was passed to iad_cell_close (use iad_axis_close / "
                                        "iad_move_handle_close)");
        for (const auto& a : axes) a->events->close();
        if (!entry) return;
        entry->events->close();
        std::shared_ptr<recovery::LinkMonitor> links;
        {
            std::lock_guard<std::mutex> lock(entry->mutex);
            links = std::move(entry->link_monitor);
        }
        if (links) links->stop();  // before the ports close under it (Cell::close stops its own too)
        entry->cell->close();
    });
}

iad_result IAD_CALL iad_cell_axis_count(iad_cell* cell, size_t* count) {
    return guarded([&] {
        const auto e = find_cell(cell);
        need(count, "count");
        *count = e->cell->size();
    });
}

iad_result IAD_CALL iad_cell_axis_name(iad_cell* cell, size_t index, char* buffer, size_t buffer_size,
                                       size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        const std::vector<std::string> names = e->cell->names();
        if (index >= names.size())
            throw std::out_of_range("axis index " + std::to_string(index) + " outside 0.." +
                                    std::to_string(names.size()) + " (exclusive)");
        write_text_output(names[index], buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_open_axis(iad_cell* cell, const char* name, iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto e = find_cell(cell);
        need(name, "name");
        *out = register_cell_axis(e->cell->axis_ptr(name), cell);
    });
}

iad_result IAD_CALL iad_cell_info_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        Json j = Json::object();
        j.set("axes", Json(e->cell->names()));
        Json info = Json::object();
        for (const auto& kv : e->cell->info()) info.set(kv.first, to_json(kv.second));
        j.set("info", std::move(info));
        Json port_errors = Json::object();
        for (const auto& kv : e->cell->port_errors()) port_errors.set(kv.first, kv.second);
        j.set("port_errors", std::move(port_errors));
        Json faults = Json::object();
        for (const auto& kv : e->cell->faults()) faults.set(kv.first, kv.second);
        j.set("faults", std::move(faults));
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_clear_fault(iad_cell* cell, const char* axis) {
    return guarded([&] {
        const auto e = find_cell(cell);
        need(axis, "axis");
        if (!e->cell->contains(axis)) throw std::out_of_range(std::string("no axis '") + axis + "' in this cell");
        e->cell->clear_fault(axis);
    });
}

iad_result IAD_CALL iad_cell_servo_all(iad_cell* cell, iad_bool on, const char* axes, double timeout_s,
                                       iad_cell_result* results, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        finish(e->cell->servo_all(on != 0, list, timeout_arg(timeout_s, "timeout_s").value_or(kServoTimeoutS), true),
               results, capacity, count);
    });
}

}  // extern "C"
