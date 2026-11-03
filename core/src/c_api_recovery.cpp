// C ABI 0.6 additions of the recovery facade (include/iai_actuator_driver.h, "axis: recovery sessions, link state"; the
// 0.2 recovery functions are in c_api_ext.cpp): a session made from an identity, its volatile fields written, the link
// state set, the port's presence and the restart signs asked of the core, and the cell's link monitor read back. Each
// function is a thin call of the C++ function its header comment names, wrapped in capi::guarded like the other
// exported functions. What queued an event (iad_event.source) is set where the events are queued
// (c_api_ext_support.cpp). Also the 0.7 test hook iad_testing_set_port_list (iai_actuator_driver_testing.h), the port
// list iad_axis_port_present reads.
//
// Evidence: UNVERIFIED on hardware (tested on the simulator: tests/test_c_api_recovery.cpp).

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/status.hpp"
#include "iai_actuator_driver/transport.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

/// A JSON object parsed from `text` (std::invalid_argument naming `what` otherwise).
Json parse_object(const char* text, const char* what) {
    need(text, what);
    Json j = Json::parse(text);
    if (j.type() != Json::Type::Object) throw std::invalid_argument(std::string(what) + ": a JSON object is expected");
    return j;
}

// Every value is type-checked before it is read (a Json accessor of the wrong type throws std::logic_error, which is
// IAD_ERR_UNSUPPORTED, "not in this build"): a wrong value is std::invalid_argument naming its key.

/// A number (finite: the parser holds no NaN or infinity), std::invalid_argument naming `what` otherwise.
double number_value(const Json& v, const std::string& what) {
    if (v.type() != Json::Type::Int && v.type() != Json::Type::Number)
        throw std::invalid_argument(what + " must be a number");
    return v.as_number();
}

/// A whole number in lo..hi (an integral float such as 9600.0 too). A fraction, a value outside the range and another
/// type are refused naming `what`, never narrowed (converting a double outside the target type is undefined).
long long whole_value(const Json& v, const std::string& what, long long lo, long long hi) {
    const std::optional<long long> n = v.as_whole();
    if (!n || *n < lo || *n > hi)
        throw std::invalid_argument(what + " must be a whole number in " + std::to_string(lo) + ".." +
                                    std::to_string(hi));
    return *n;
}

int int32_value(const Json& v, const std::string& what) {
    return static_cast<int>(whole_value(v, what, INT32_MIN, INT32_MAX));
}

std::optional<std::string> text_of(const Json& j, const char* key) {
    const Json* v = j.find(key);
    if (!v || v->is_null()) return std::nullopt;
    if (v->type() != Json::Type::String)
        throw std::invalid_argument(std::string("info_json: ") + key + " must be text");
    return v->as_string();
}

std::optional<double> number_of(const Json& j, const char* key) {
    const Json* v = j.find(key);
    if (!v || v->is_null()) return std::nullopt;
    return number_value(*v, std::string("info_json: ") + key);
}

std::optional<int> int_of(const Json& j, const char* key) {
    const Json* v = j.find(key);
    if (!v || v->is_null()) return std::nullopt;
    return int32_value(*v, std::string("info_json: ") + key);
}

/// The identity fields of a unit info's JSON (iad_axis_identity_json; the Python DeviceInfo) that a session keeps.
DeviceInfo identity_from_json(const Json& j) {
    DeviceInfo info;
    info.serial_number = text_of(j, "serial_number");
    info.model_name = text_of(j, "model_name");
    info.system_code = int_of(j, "system_code");
    info.unit_code = int_of(j, "unit_code");
    info.firmware = text_of(j, "firmware");
    info.product_family = text_of(j, "product_family");
    info.max_speed_mm_s = number_of(j, "max_speed_mm_s");
    if (const auto units = text_of(j, "units")) info.units = *units;
    if (const Json* limits = j.find("soft_limits_mm"); limits && !limits->is_null()) {
        if (limits->type() != Json::Type::Array || limits->size() != 2)
            throw std::invalid_argument("info_json: soft_limits_mm must be [min, max] or null");
        const auto& pair = limits->as_array();
        info.soft_limits_mm = Span{number_value(pair[0], "info_json: soft_limits_mm[0]"),
                                   number_value(pair[1], "info_json: soft_limits_mm[1]")};
    }
    return info;
}

/// The alarm-history slots of iad_axis_restart_signs' "new_records" ([[code, detail, address, time_ms or null]]).
std::vector<recovery::AlarmKey> alarm_keys(const Json* list) {
    std::vector<recovery::AlarmKey> out;
    if (!list || list->is_null()) return out;
    if (list->type() != Json::Type::Array) throw std::invalid_argument("options_json: new_records must be a list");
    for (const Json& item : list->as_array()) {
        if (item.type() != Json::Type::Array || item.size() < 3)
            throw std::invalid_argument("options_json: a new record is [code, detail, address, time_ms or null]");
        const auto& a = item.as_array();
        recovery::AlarmKey key;
        key.code = int32_value(a[0], "options_json: new_records code");
        key.detail = int32_value(a[1], "options_json: new_records detail");
        key.address = int32_value(a[2], "options_json: new_records address");
        if (a.size() > 3 && !a[3].is_null())
            key.time_ms = static_cast<std::int64_t>(whole_value(a[3], "options_json: new_records time_ms", INT64_MIN,
                                                                INT64_MAX));
        out.push_back(key);
    }
    return out;
}

}  // namespace

iad_result IAD_CALL iad_axis_session_from_info(iad_axis* axis, const char* info_json, const char* name) {
    return guarded([&] {
        const auto e = find_axis(axis);
        DeviceInfo info;
        if (info_json) {
            info = identity_from_json(parse_object(info_json, "info_json"));
        } else {
            const std::optional<DeviceInfo> read = e->axis->device_info();
            if (!read) throw EcNotReadyError("the axis has read no identity yet: connect() or identify() it first");
            info = *read;
        }
        recovery::session_from_info(*e->axis, info, name ? name : "");
    });
}

iad_result IAD_CALL iad_axis_session_update(iad_axis* axis, const char* fields_json) {
    return guarded([&] {
        const auto e = find_axis(axis);
        recovery::update_session(*e->axis, parse_object(fields_json, "fields_json"));
    });
}

iad_result IAD_CALL iad_axis_set_link_state(iad_axis* axis, const char* state, const char* message, const char* name,
                                            iad_bool* changed) {
    return guarded([&] {
        if (changed) *changed = IAD_FALSE;
        const auto e = find_axis(axis);
        need(state, "state");
        const bool c = recovery::set_link_state(*e->axis, state, message ? message : "",
                                                queueing_recovery_events(e->events, IAD_EVENT_SOURCE_SET_LINK_STATE),
                                                name ? name : "");
        if (changed) *changed = c ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_axis_port_present(iad_axis* axis, iad_tribool* present) {
    return guarded([&] {
        need(present, "present");
        *present = IAD_UNKNOWN;
        const auto e = find_axis(axis);
        const std::optional<bool> listed = recovery::port_present(*e->axis);
        *present = listed ? (*listed ? IAD_TRUE : IAD_FALSE) : IAD_UNKNOWN;
    });
}

// (0.7, core-fixes) the test hook iad_axis_port_present names: the port list every caller of list_ports() sees.
iad_result IAD_CALL iad_testing_set_port_list(const char* ports_json) {
    return guarded([&] {
        if (ports_json == nullptr) {
            detail::set_test_port_list(std::nullopt);
            return;
        }
        const Json list = Json::parse(ports_json);
        if (list.type() != Json::Type::Array) throw std::invalid_argument("ports_json: a JSON list of ports or NULL");
        std::vector<SerialPortInfo> ports;
        for (const Json& p : list.as_array()) {
            if (p.type() != Json::Type::Object) throw std::invalid_argument("ports_json: each port is a JSON object");
            SerialPortInfo info;
            bool named = false;
            for (const auto& kv : p.as_object()) {
                const std::string what = "ports_json: '" + kv.first + "'";
                const Json& v = kv.second;
                if (kv.first == "port" || kv.first == "description" || kv.first == "manufacturer" ||
                    kv.first == "hardware_id") {
                    if (kv.first != "port" && v.is_null()) continue;
                    if (v.type() != Json::Type::String) throw std::invalid_argument(what + " must be text");
                    std::string& field = kv.first == "port"           ? info.port
                                         : kv.first == "description"  ? info.description
                                         : kv.first == "manufacturer" ? info.manufacturer
                                                                      : info.hardware_id;
                    field = v.as_string();
                    named = named || kv.first == "port";
                } else if (kv.first == "vid" || kv.first == "pid") {
                    std::optional<int>& id = kv.first == "vid" ? info.vid : info.pid;
                    if (v.is_null()) id.reset();
                    else id = static_cast<int>(whole_value(v, what, 0, 0xFFFF));
                } else if (kv.first != "is_iai") {  // is_iai (iad_list_ports_json) follows from the others
                    throw std::invalid_argument(what + " is not a port field (port, description, manufacturer, "
                                                       "hardware_id, vid, pid)");
                }
            }
            if (!named || info.port.empty()) throw std::invalid_argument("ports_json: each port needs a 'port' name");
            ports.push_back(std::move(info));
        }
        detail::set_test_port_list(std::move(ports));
    });
}

iad_result IAD_CALL iad_axis_restart_signs(iad_axis* axis, const uint16_t* words, size_t word_count,
                                           const char* options_json, char* buffer, size_t buffer_size,
                                           size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        need(words, "words");
        if (word_count < 8 || word_count > 64)
            throw std::invalid_argument("words: a status block of 8 to 22 words, got " + std::to_string(word_count));
        const Status status = Status::from_registers(std::vector<std::uint16_t>(words, words + word_count));
        std::vector<recovery::AlarmKey> records;
        std::optional<int> found;
        int boot = recovery::kBootBaudRate;
        std::optional<double> now;
        if (options_json) {
            const Json o = parse_object(options_json, "options_json");
            records = alarm_keys(o.find("new_records"));
            if (const Json* v = o.find("baud_rate_found"); v && !v->is_null())
                found = int32_value(*v, "options_json: baud_rate_found");
            if (const Json* v = o.find("boot_baud_rate"); v && !v->is_null())
                boot = int32_value(*v, "options_json: boot_baud_rate");
            if (const Json* v = o.find("now_s"); v && !v->is_null())
                now = finite(number_value(*v, "options_json: now_s"), "now_s");
        }
        const std::shared_ptr<recovery::Session> sess = recovery::session_of(*e->axis);
        if (!sess)
            throw EcNotReadyError("no recovery session: call iad_axis_track or iad_axis_session_from_info first");
        std::vector<std::string> signs;
        {
            std::lock_guard<std::recursive_mutex> lock(sess->mutex);
            signs = recovery::restart_signs(*sess, status, records, found, boot, now);
        }
        Json list = Json::array();
        for (const auto& s : signs) list.push_back(s);
        write_text_output(list.dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_link_monitor_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        std::shared_ptr<recovery::LinkMonitor> monitor;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            monitor = e->link_monitor;
        }
        Json j;
        if (monitor) {
            j = Json::object();
            j.set("running", monitor->running());
            j.set("interval_s", monitor->interval_s());
            j.set("passes", static_cast<long long>(monitor->passes()));
            Json states = Json::object();
            for (const auto& kv : monitor->states()) states.set(kv.first, kv.second.to_json());
            j.set("states", std::move(states));
            Json errors = Json::array();
            for (const auto& text : monitor->errors()) errors.push_back(text);
            j.set("errors", std::move(errors));
        }
        write_text_output(j.dump(), buffer, buffer_size, required);
    });
}
