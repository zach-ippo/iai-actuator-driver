// The gateway records' alarm names and JSON forms (model unit: alarm tables and Json only; the gateway objects and the
// line live in the bus unit, src/gateway.cpp and src/line.cpp). Field names follow the Python edition's to_dict() of
// gateway.LineRules / Line / RconGatewayStatus / RconAxisMonitor / RoboNetGatewayStatus.
#include <variant>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

Json int_list(const std::vector<int>& values) {
    Json out = Json::array();
    for (int v : values) out.push_back(v);
    return out;
}

Json hex_words(const std::vector<std::uint16_t>& words) {
    Json out = Json::array();
    for (std::uint16_t w : words) out.push_back(util::format("0x%04X", w));
    return out;
}

}  // namespace

std::optional<std::string> RconGatewayStatus::alarm_name() const {
    const std::uint32_t code = alarm_code();
    if (!code) return std::nullopt;
    return iai_actuator_driver::alarm_name(static_cast<int>(code));
}

Json to_json(const LineRules& value) {
    Json o = Json::object();
    o.set("kind", to_string(value.kind));
    o.set("title", value.title);
    o.set("broadcast", value.broadcast);
    o.set("iai_commands", value.iai_commands);
    o.set("function_codes", value.function_codes ? int_list(*value.function_codes) : Json());
    o.set("command_strategy", value.command_strategy ? Json(*value.command_strategy) : Json());
    o.set("requires_manu", value.requires_manu);
    o.set("requires_position_unit", value.requires_position_unit);
    o.set("axis_access", value.axis_access);
    o.set("max_write_registers", value.max_write_registers);
    o.set("probe_timeout_s", value.probe_timeout_s);
    o.set("min_frame_timeout_s", value.min_frame_timeout_s);
    if (value.baud_range) {
        Json range = Json::array();
        range.push_back(value.baud_range->first);
        range.push_back(value.baud_range->second);
        o.set("baud_range", range);
    } else {
        o.set("baud_range", Json());
    }
    o.set("gateway_addresses", int_list(value.gateway_addresses));
    o.set("directory_timeout_means_absent", value.directory_timeout_means_absent);
    o.set("evidence", value.evidence);
    return o;
}

Json to_json(const RconGatewayStatus& value) {
    Json o = Json::object();
    o.set("kind", to_string(LineKind::Rcon));
    o.set("words", hex_words(value.words));
    o.set("manu", value.manu());
    o.set("monitor_active", value.monitor_active());
    o.set("link_error", value.link_error());
    o.set("emergency_stop", value.emergency_stop());
    o.set("linked_axes", int_list(value.linked_axes()));
    o.set("alarm_code", static_cast<long long>(value.alarm_code()));
    const std::optional<std::string> name = value.alarm_name();
    o.set("alarm_name", name ? Json(*name) : Json());
    o.set("system_timer_ms", static_cast<long long>(value.system_timer_ms()));
    return o;
}

Json to_json(const RconAxisMonitor& value) {
    Json o = Json::object();
    o.set("axis", value.axis);
    o.set("slave", value.slave());
    o.set("position_counts", value.position_counts());
    o.set("velocity_counts", value.velocity_counts());
    o.set("current_ma", value.current_ma());
    o.set("alarm_code", value.alarm_code());
    o.set("dss1", value.dss1());
    o.set("dsse", value.dsse());
    o.set("sipm", value.sipm());
    o.set("overload_pct", value.overload_pct());
    o.set("deviation_pulses", value.deviation_pulses());
    o.set("stat", static_cast<long long>(value.stat()));
    return o;
}

Json to_json(const RoboNetGatewayStatus& value) {
    Json o = Json::object();
    o.set("kind", to_string(LineKind::RobonetGateway));
    o.set("words", hex_words(value.words));
    o.set("running", value.running());
    o.set("manu", value.manu());
    o.set("direct_axes", value.direct_axes());
    o.set("positioner_axes", value.positioner_axes());
    o.set("linked_axes", int_list(value.linked_axes()));
    o.set("explanation", kRoboNetSw2);
    return o;
}

Json to_json(const GatewayStatus& value) {
    return std::visit([](const auto& status) { return to_json(status); }, value);
}

Json to_json(const LineSnapshot& value) {
    Json o = Json::object();
    o.set("kind", to_string(value.kind));
    o.set("detected", value.detected);
    o.set("rules", to_json(value.rules));
    o.set("gateway", value.gateway ? to_json(*value.gateway) : Json());
    Json units = Json::object();
    for (const auto& kv : value.position_units) {
        Json entry = Json::object();
        entry.set("unit_mm", kv.second.first);
        entry.set("source", kv.second.second);
        units.set(kv.first == 0 ? std::string("all") : std::to_string(kv.first), entry);
    }
    o.set("position_units", units);
    o.set("unreachable_slaves", int_list(value.unreachable_slaves));
    o.set("explanation", value.explanation ? Json(*value.explanation) : Json());
    return o;
}

}  // namespace iai_actuator_driver
