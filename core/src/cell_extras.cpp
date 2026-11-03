// Cell extras that need no Cell internals: monitor_reading() (one decoded 0x9000 monitor block, the Python edition's
// Elecylinder.read_monitor() fields; Cell::monitor_all reads it on every axis), Cell::backup (the Python edition's
// Cell.backup) and the Port broadcasts of the Python edition's cell.Port that Cell::pause_all uses (broadcast_pause,
// broadcast_servo_off; broadcast_stop is in port.cpp).
// UNVERIFIED on hardware in this edition (the monitor block itself is VERIFIED in the Python edition up to ZONS).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"

namespace iai_actuator_driver {

namespace {

Json member(const Json& m, const char* key) {
    const Json* v = m.find(key);
    return v ? *v : Json();
}

}  // namespace

Json monitor_reading(Axis& axis) {
    const Json data = info::unit_info(axis, std::vector<std::string>{"monitor"});
    const Json* m = data.find("monitor");
    if (!m || m->is_null()) {
        std::string why = "the monitor block was not read";
        if (const Json* errors = data.find("errors"))
            if (errors->type() == Json::Type::Array && errors->size() &&
                errors->as_array().front().type() == Json::Type::String)
                why = errors->as_array().front().as_string();
        throw EcNotReadyError(why);
    }
    const Json* code = m->find("alarm_code");
    const int alarm = code && (code->type() == Json::Type::Int || code->type() == Json::Type::Number)
                          ? static_cast<int>(code->as_int())
                          : 0;
    Json r = Json::object();
    r.set("units", axis.units());
    Json position = member(*m, "position_mm");
    // mm per PNOW count: 0.01, or an RCON unit's 0.001 (No.207, DOC ME0384 p.8-56) as the line knows it
    const std::optional<double> unit = axis.bus()->line()->position_unit_mm(axis.slave());
    if (unit && *unit > 0.0 && *unit < reg::kPositionScale &&
        (position.type() == Json::Type::Number || position.type() == Json::Type::Int)) {
        const double counts = std::round(position.as_number() / reg::kPositionScale);  // PNOW, exact
        position = Json(std::round(counts * *unit * 1000.0) / 1000.0);
    }
    r.set("position", position);
    r.set("velocity", member(*m, "velocity_mm_s"));
    r.set("current", member(*m, "current"));
    r.set("current_pct", member(*m, "current_pct"));
    r.set("rated_current", member(*m, "rated_current"));
    r.set("deviation", member(*m, "deviation"));
    r.set("alarm", alarm);
    // as the Python edition's read_monitor: "" without an alarm
    const std::optional<AlarmLevel> level = alarm ? std::optional<AlarmLevel>(alarm_level(alarm)) : std::nullopt;
    r.set("alarm_name", alarm ? Json(alarm_name(alarm)) : Json(""));
    r.set("alarm_level", level ? Json(level_key(*level)) : Json(""));
    r.set("uptime_s", member(*m, "uptime_s"));
    const Json* raw = m->find("raw");
    r.set("raw", raw ? *raw : Json::object());
    // (0.5) the words read, in block order (the raw members are in layout order: "0x%08X" two words, "0x%04X" one)
    Json words = Json::array();
    if (raw && raw->type() == Json::Type::Object)
        for (const auto& kv : raw->as_object()) {
            if (kv.second.type() != Json::Type::String) continue;
            const std::string& hex = kv.second.as_string();
            const unsigned long value = std::strtoul(hex.c_str(), nullptr, 16);
            if (hex.size() > 6) words.push_back(static_cast<long long>((value >> 16) & 0xFFFF));
            words.push_back(static_cast<long long>(value & 0xFFFF));
        }
    r.set("words", words);
    return r;
}

// ------------------------------------------------------------------------------------------ Cell::backup

std::vector<std::pair<std::string, std::string>> Cell::backup(const std::string& directory,
                                                              const std::vector<std::string>& axes) {
    return backup(directory, axes, info::BackupRows::used());
}

std::vector<std::pair<std::string, std::string>> Cell::backup(const std::string& directory,
                                                              const std::vector<std::string>& axes,
                                                              const info::BackupRows& rows) {
    if (closed()) throw EcNotReadyError("cell is closed");
    std::vector<std::string> which;
    for (const auto& n : axes.empty() ? names() : axes) {
        (*this)[n];  // std::out_of_range for an unknown axis, before anything is read
        if (std::find(which.begin(), which.end(), n) == which.end()) which.push_back(n);
    }
    std::vector<std::pair<std::string, Axis*>> selected;
    for (const auto& n : which) selected.emplace_back(n, &(*this)[n]);
    return info::backup_axes(selected, directory, rows);
}

// ------------------------------------------------------------------------------------------ Port broadcasts

std::vector<std::uint8_t> Port::broadcast_pause(bool on) {
    require_line(LineOperation::Broadcast, name() + ": broadcast");
    return bus()->broadcast_coil(reg::kCoilStp, on);
}

std::vector<std::uint8_t> Port::broadcast_servo_off() {
    require_line(LineOperation::Broadcast, name() + ": broadcast");
    return bus()->broadcast_coil(reg::kCoilSon, false);
}

}  // namespace iai_actuator_driver
