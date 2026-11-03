// C ABI 0.5 line and gateway handles (include/iai_actuator_driver.h, "lines and gateways"): a Line made by the
// caller and attached to a bus, the line a bus carries, the rules a caller states for the axes, the read-only RCON and
// ROBONET gateway objects, the probes, the serial code and the ROBONET hint. The work is in src/line.cpp and
// src/gateway.cpp; this file converts. Every gateway rule is DOC only (ME0413, ME0208): UNVERIFIED on hardware.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

/// iad_line. `bus`: the bus its gateway reads through (the gateway keeps a reference to it), kept alive with the handle.
struct LineHandle {
    std::shared_ptr<Line> line;
    std::shared_ptr<ModbusRtu> bus;
};

/// iad_gateway. `bus`: the bus it reads through, kept alive with the handle.
struct GatewayHandle {
    std::shared_ptr<Gateway> gateway;
    std::shared_ptr<ModbusRtu> bus;
};

std::shared_ptr<BusEntry> bus_of(iad_bus* handle) {
    return std::static_pointer_cast<BusEntry>(find_object(handle, ObjectKind::Bus));
}

std::shared_ptr<LineHandle> line_of(iad_line* handle) {
    return std::static_pointer_cast<LineHandle>(find_object(handle, ObjectKind::Line));
}

std::shared_ptr<GatewayHandle> gateway_of(iad_gateway* handle) {
    return std::static_pointer_cast<GatewayHandle>(find_object(handle, ObjectKind::Gateway));
}

iad_line* new_line(std::shared_ptr<Line> line, std::shared_ptr<ModbusRtu> bus) {
    auto entry = std::make_shared<LineHandle>();
    entry->line = std::move(line);
    entry->bus = std::move(bus);
    return reinterpret_cast<iad_line*>(register_object(ObjectKind::Line, entry));
}

iad_gateway* new_gateway(std::shared_ptr<Gateway> gateway, std::shared_ptr<ModbusRtu> bus) {
    auto entry = std::make_shared<GatewayHandle>();
    entry->gateway = std::move(gateway);
    entry->bus = std::move(bus);
    return reinterpret_cast<iad_gateway*>(register_object(ObjectKind::Gateway, entry));
}

LineKind kind_arg(int32_t kind) {
    switch (kind) {
        case IAD_LINE_DIRECT: return LineKind::Direct;
        case IAD_LINE_RCON: return LineKind::Rcon;
        case IAD_LINE_ROBONET: return LineKind::Robonet;
        case IAD_LINE_ROBONET_GATEWAY: return LineKind::RobonetGateway;
        default: break;
    }
    throw std::invalid_argument("line must be IAD_LINE_DIRECT, _RCON, _ROBONET or _ROBONET_GATEWAY, got " +
                                std::to_string(kind));
}

std::optional<int> slave_or_none(int32_t slave) {
    if (slave == 0) return std::nullopt;
    if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
    return slave;
}

LineOperation operation_arg(const char* operation) {
    const std::string name = need(operation, "operation");
    if (const std::optional<LineOperation> op = line_operation_from_string(name)) return *op;
    Line probe;  // its refusal names the operations
    probe.allows(name);
    throw std::invalid_argument("unknown line operation '" + name + "'");
}

/// A status's words and read time to the caller (*count first: a short buffer is IAD_ERR_BUFFER_TOO_SMALL).
void words_out(const std::vector<std::uint16_t>& in, double at, uint16_t* words, size_t capacity, size_t* count,
               double* read_at) {
    need(count, "count");
    if (capacity > 0) need(words, "words");
    *count = in.size();
    if (read_at) *read_at = at;
    if (capacity < in.size())
        throw BufferTooSmall("words: " + std::to_string(in.size()) + " needed, capacity " + std::to_string(capacity));
    for (std::size_t i = 0; i < in.size(); ++i) words[i] = in[i];
}

void status_out(const GatewayStatus& status, uint16_t* words, size_t capacity, size_t* count, double* read_at) {
    std::visit([&](const auto& st) { words_out(st.words, st.read_at, words, capacity, count, read_at); }, status);
}

std::vector<std::uint16_t> words_in(const uint16_t* words, size_t count) {
    if (count > 0) need(words, "words");
    return count > 0 ? std::vector<std::uint16_t>(words, words + count) : std::vector<std::uint16_t>();
}

std::optional<bool> opt_flag(const Json& rules, const char* key) {
    const Json* value = rules.find(key);
    if (!value || value->is_null()) return std::nullopt;
    return value->as_bool();
}

}  // namespace

// ============================================================================================ lines

iad_result IAD_CALL iad_line_open(int32_t kind, iad_gateway* gateway, iad_bool detected, double position_unit_mm,
                                  iad_line** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        std::shared_ptr<Gateway> gw;
        std::shared_ptr<ModbusRtu> bus;
        if (gateway) {
            const auto g = gateway_of(gateway);
            gw = g->gateway;
            bus = g->bus;
        }
        *out = new_line(std::make_shared<Line>(kind_arg(kind), gw, detected != 0, to_position_unit(position_unit_mm)),
                        bus);
    });
}

iad_result IAD_CALL iad_line_close(iad_line* line) {
    return guarded([&] {
        if (!line) return;
        bool other_kind = false;
        release_object(line, ObjectKind::Line, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_line_close");
    });
}

iad_result IAD_CALL iad_bus_line(iad_bus* bus, iad_line** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto b = bus_of(bus);
        *out = new_line(b->bus->line(), b->bus);
    });
}

iad_result IAD_CALL iad_bus_attach_line(iad_bus* bus, iad_line* line) {
    return guarded([&] {
        const auto b = bus_of(bus);
        b->bus->set_line(line_of(line)->line);
    });
}

iad_result IAD_CALL iad_bus_state_line_rules(iad_bus* bus, const char* rules_json) {
    return guarded([&] {
        const auto b = bus_of(bus);
        const Json rules = rules_json && *rules_json ? Json::parse(rules_json) : Json();
        if (rules.is_null()) {
            b->bus->line()->state_rules(std::nullopt);
            return;
        }
        if (rules.type() != Json::Type::Object) throw std::invalid_argument("rules_json must be a JSON object or null");
        StatedLineRules stated;
        if (const Json* name = rules.find("name"); name && !name->is_null()) stated.name = name->as_string();
        stated.coils = opt_flag(rules, "coils");
        stated.iai_commands = opt_flag(rules, "iai_commands");
        stated.broadcast = opt_flag(rules, "broadcast");
        stated.report_slave_id = opt_flag(rules, "report_slave_id");
        b->bus->line()->state_rules(stated);
    });
}

iad_result IAD_CALL iad_line_json(iad_line* line, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        write_text_output(to_json(line_of(line)->line->snapshot()).dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_line_allows(iad_line* line, const char* operation, iad_bool* allowed) {
    return guarded([&] {
        need(allowed, "allowed");
        *allowed = line_of(line)->line->allows(operation_arg(operation)) ? 1 : 0;
    });
}

iad_result IAD_CALL iad_line_require(iad_line* line, const char* operation, const char* what) {
    return guarded([&] {
        const auto l = line_of(line);
        l->line->require(operation_arg(operation), what ? std::string(what) : std::string());
    });
}

iad_result IAD_CALL iad_line_check_request(iad_line* line, int32_t slave, const uint8_t* pdu, size_t size) {
    return guarded([&] {
        const auto l = line_of(line);
        if (size > 0) need(pdu, "pdu");
        if (slave < 0 || slave > 255) throw std::invalid_argument("slave address must be 0..255, got " + std::to_string(slave));
        l->line->check_request(slave, std::vector<std::uint8_t>(pdu, pdu + size));
    });
}

iad_result IAD_CALL iad_line_require_control(iad_line* line, int32_t slave) {
    return guarded([&] { line_of(line)->line->require_control(slave_or_none(slave)); });
}

iad_result IAD_CALL iad_line_position_unit(iad_line* line, int32_t slave, double* unit_mm, char* source,
                                           size_t source_size, size_t* required) {
    return guarded([&] {
        need(unit_mm, "unit_mm");
        *unit_mm = std::nan("");
        check_text_output(source, source_size, required);
        const auto l = line_of(line);
        const std::optional<int> s = slave_or_none(slave);
        const std::optional<double> unit = l->line->position_unit_mm(s);
        const std::optional<std::string> where = l->line->position_unit_source(s);
        if (unit) *unit_mm = *unit;
        write_text_output(where.value_or(std::string()), source, source_size, required);
    });
}

iad_result IAD_CALL iad_line_set_position_unit(iad_line* line, double unit_mm, int32_t slave, const char* source) {
    return guarded([&] {
        line_of(line)->line->set_position_unit(unit_mm, slave_or_none(slave),
                                               source ? std::string(source) : std::string(kUnitStated));
    });
}

iad_result IAD_CALL iad_line_require_position_unit(iad_line* line, int32_t slave, iad_bool control, double* unit_mm) {
    return guarded([&] {
        need(unit_mm, "unit_mm");
        *unit_mm = std::nan("");
        const auto l = line_of(line);
        if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
        *unit_mm = control ? l->line->require_motion(slave) : l->line->require_position_unit(slave);
    });
}

iad_result IAD_CALL iad_line_note_unreachable(iad_line* line, const char* slaves) {
    return guarded([&] { line_of(line)->line->note_unreachable(to_int_list(slaves, "slaves")); });
}

iad_result IAD_CALL iad_line_explain_unreachable(iad_line* line, const char* slaves, char* buffer, size_t buffer_size,
                                                 size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto l = line_of(line);
        const std::optional<std::vector<int>> which =
            slaves ? std::optional<std::vector<int>>(to_int_list(slaves, "slaves")) : std::nullopt;
        write_text_output(l->line->explain_unreachable(which).value_or(std::string()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_line_gateway(iad_line* line, iad_gateway** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto l = line_of(line);
        if (std::shared_ptr<Gateway> gw = l->line->gateway()) *out = new_gateway(std::move(gw), l->bus);
    });
}

iad_result IAD_CALL iad_line_rules_json(char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        Json out = Json::array();
        for (const LineRules& rules : line_rules_table()) out.push_back(to_json(rules));
        write_text_output(out.dump(), buffer, buffer_size, required);
    });
}

// ============================================================================================ gateways

iad_result IAD_CALL iad_gateway_open(iad_bus* bus, int32_t kind, int32_t address, const uint16_t* words, size_t count,
                                     double read_at, double cache_s, iad_gateway** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto b = bus_of(bus);
        const std::vector<std::uint16_t> known = words_in(words, count);
        const double at = std::isnan(read_at) ? 0.0 : read_at;
        std::shared_ptr<Gateway> gw;
        if (kind_arg(kind) == LineKind::Rcon) {
            if (address < 0 || address > 255) throw std::invalid_argument("address must be 0..255, got " + std::to_string(address));
            const double cache = std::isnan(cache_s) ? kGwsrCacheS : cache_s;
            if (!(cache >= 0.0)) throw std::invalid_argument("cache_s must be >= 0 (NaN: 2 s)");
            std::optional<RconGatewayStatus> st;
            if (count > 0) st = RconGatewayStatus{known, at};
            gw = std::make_shared<RconGateway>(*b->bus, address == 0 ? kRconGatewaySlave : address, st, cache);
        } else if (kind_arg(kind) == LineKind::RobonetGateway) {
            std::optional<RoboNetGatewayStatus> st;
            if (count > 0) st = RoboNetGatewayStatus{known, at};
            gw = std::make_shared<RoboNetGateway>(*b->bus, st);
        } else {
            throw std::invalid_argument("a gateway is IAD_LINE_RCON or IAD_LINE_ROBONET_GATEWAY");
        }
        *out = new_gateway(std::move(gw), b->bus);
    });
}

iad_result IAD_CALL iad_gateway_close(iad_gateway* gateway) {
    return guarded([&] {
        if (!gateway) return;
        bool other_kind = false;
        release_object(gateway, ObjectKind::Gateway, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_gateway_close");
    });
}

iad_result IAD_CALL iad_gateway_info_json(iad_gateway* gateway, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto g = gateway_of(gateway);
        Json j = Json::object();
        j.set("kind", iai_actuator_driver::to_string(g->gateway->kind()));
        j.set("address", g->gateway->address());
        const auto* rcon = dynamic_cast<const RconGateway*>(g->gateway.get());
        j.set("cache_s", rcon ? Json(rcon->cache_s()) : Json());
        const std::optional<GatewayStatus> last = g->gateway->last_gateway_status();
        Json words;
        Json at;
        if (last) {
            std::visit([&](const auto& st) {
                words = Json::array();
                for (std::uint16_t w : st.words) words.push_back(static_cast<int>(w));
                at = Json(st.read_at);
            }, *last);
        }
        j.set("words", words);
        j.set("read_at", at);
        write_text_output(j.dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_gateway_status(iad_gateway* gateway, double max_age_s, uint16_t* words, size_t capacity,
                                       size_t* count, double* read_at) {
    return guarded([&] {
        if (count) *count = 0;
        const auto g = gateway_of(gateway);
        const std::optional<double> age = std::isnan(max_age_s) ? std::nullopt : std::optional<double>(max_age_s);
        if (auto* rcon = dynamic_cast<RconGateway*>(g->gateway.get())) {
            const RconGatewayStatus st = age && *age < 0.0 ? rcon->read_status() : rcon->status(age);
            words_out(st.words, st.read_at, words, capacity, count, read_at);
        } else if (auto* robonet = dynamic_cast<RoboNetGateway*>(g->gateway.get())) {
            const RoboNetGatewayStatus st = age && *age < 0.0 ? robonet->read_status() : robonet->status(age);
            words_out(st.words, st.read_at, words, capacity, count, read_at);
        } else {
            status_out(g->gateway->read_gateway_status(), words, capacity, count, read_at);
        }
    });
}

iad_result IAD_CALL iad_gateway_require_manu(iad_gateway* gateway, uint16_t* words, size_t capacity, size_t* count,
                                             double* read_at) {
    return guarded([&] {
        if (count) *count = 0;
        auto* rcon = dynamic_cast<RconGateway*>(gateway_of(gateway)->gateway.get());
        if (!rcon) throw std::invalid_argument("require_manu: not an RCON gateway");
        const RconGatewayStatus st = rcon->require_manu();
        words_out(st.words, st.read_at, words, capacity, count, read_at);
    });
}

iad_result IAD_CALL iad_gateway_read_alarm(iad_gateway* gateway, uint32_t* code) {
    return guarded([&] {
        need(code, "code");
        *code = 0;
        auto* rcon = dynamic_cast<RconGateway*>(gateway_of(gateway)->gateway.get());
        if (!rcon) throw std::invalid_argument("read_alarm: not an RCON gateway");
        *code = rcon->read_alarm();
    });
}

iad_result IAD_CALL iad_gateway_set_cache_s(iad_gateway* gateway, double cache_s) {
    return guarded([&] {
        auto* rcon = dynamic_cast<RconGateway*>(gateway_of(gateway)->gateway.get());
        if (!rcon) throw std::invalid_argument("cache_s: not an RCON gateway");
        if (!(cache_s >= 0.0) || !std::isfinite(cache_s)) throw std::invalid_argument("cache_s must be >= 0");
        rcon->set_cache_s(cache_s);
    });
}

iad_result IAD_CALL iad_gateway_read_monitor_json(iad_gateway* gateway, const char* axes, char* buffer,
                                                  size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        auto* rcon = dynamic_cast<RconGateway*>(gateway_of(gateway)->gateway.get());
        if (!rcon) throw std::invalid_argument("read_monitor: not an RCON gateway");
        std::optional<std::vector<int>> which;
        if (axes && *axes) which = to_int_list(axes, "axes");
        Json out = Json::object();
        for (const auto& kv : rcon->read_monitor(which)) {
            Json words = Json::array();
            for (std::uint16_t w : kv.second.words) words.push_back(static_cast<int>(w));
            out.set(std::to_string(kv.first), words);
        }
        write_text_output(out.dump(), buffer, buffer_size, required);
    });
}

// ============================================================================================ probes on a bus

iad_result IAD_CALL iad_bus_probe_gateway(iad_bus* bus, int32_t line, int32_t address, double timeout_s,
                                          uint16_t* words, size_t capacity, size_t* count, double* read_at) {
    return guarded([&] {
        need(count, "count");
        *count = 0;
        const auto b = bus_of(bus);
        const double timeout = std::isnan(timeout_s) ? kGatewayProbeTimeoutS : timeout_s;
        if (!(timeout > 0.0) || !std::isfinite(timeout)) throw std::invalid_argument("timeout_s must be > 0 (NaN: 0.05)");
        const LineKind kind = kind_arg(line);
        if (kind == LineKind::Rcon) {
            if (address < 0 || address > 255) throw std::invalid_argument("address must be 0..255, got " + std::to_string(address));
            if (auto st = probe_rcon_gateway(*b->bus, address == 0 ? kRconGatewaySlave : address, timeout))
                words_out(st->words, st->read_at, words, capacity, count, read_at);
        } else if (kind == LineKind::RobonetGateway) {
            if (auto st = probe_robonet_gateway(*b->bus, timeout))
                words_out(st->words, st->read_at, words, capacity, count, read_at);
        } else {
            throw std::invalid_argument("a gateway probe is IAD_LINE_RCON or IAD_LINE_ROBONET_GATEWAY");
        }
    });
}

iad_result IAD_CALL iad_bus_read_serial_code(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                             size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto b = bus_of(bus);
        if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
        write_text_output(read_serial_code(*b->bus, slave).value_or(std::string()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_bus_robonet_hint(iad_bus* bus, int32_t slave, iad_bool directory_timed_out, char* buffer,
                                         size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto b = bus_of(bus);
        write_text_output(robonet_hint(*b->bus, slave, directory_timed_out != 0).value_or(std::string()), buffer,
                          buffer_size, required);
    });
}
