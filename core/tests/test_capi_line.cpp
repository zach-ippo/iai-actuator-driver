// C ABI 0.5 lines and gateways (include/iai_actuator_driver.h "lines and gateways", src/c_api_line.cpp): line
// handles made by the caller and attached to a bus, the line a bus carries, the rules stated for the axes, the
// read-only gateway objects and the probes, on the simulated RCON and ROBONET lines. The Python facade
// (bindings/python/iai_actuator_driver/_native/gateway.py) runs the Python edition's gateway tests over these.
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "c_api_objects.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 20; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

Json line_json(iad_line* line) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_line_json(line, b, s, r); }));
}

Json gateway_json(iad_gateway* gw) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_gateway_info_json(gw, b, s, r); }));
}

/// A bus without gaps or retries on a C++ transport (a simulated gateway line).
iad_bus* bus_on(std::shared_ptr<Transport> transport) {
    iad_transport* t = capi::register_transport(std::move(transport));
    iad_bus_options o;
    iad_bus_options_init(&o);
    o.min_gap_s = 0.0;
    o.retries = 0;
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(t, &o, &bus), IAD_OK);
    REQUIRE(bus != nullptr);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    return bus;
}

iad_line* line_of(iad_bus* bus) {
    iad_line* line = nullptr;
    CHECK_EQ(iad_bus_line(bus, &line), IAD_OK);
    REQUIRE(line != nullptr);
    return line;
}

bool allows(iad_line* line, const char* operation) {
    iad_bool flag = IAD_FALSE;
    CHECK_EQ(iad_line_allows(line, operation, &flag), IAD_OK);
    return flag != IAD_FALSE;
}

std::string kind_of_error() {
    const Json j = Json::parse(last_json());
    const Json* kind = j.find("kind");
    return kind && !kind->is_null() ? kind->as_string() : std::string();
}

}  // namespace

TEST(capi_line_rules_messages_and_units_without_io) {
    std::string table = read_text([](char* b, size_t s, size_t* r) { return iad_line_rules_json(b, s, r); });
    const Json rules = Json::parse(table);
    CHECK_EQ(rules.size(), 4u);
    CHECK_EQ(rules.as_array()[1].find("kind")->as_string(), std::string("rcon"));
    CHECK_EQ(rules.as_array()[1].find("command_strategy")->as_string(), std::string("register"));
    CHECK_EQ(rules.as_array()[3].find("axis_access")->as_bool(), false);

    iad_line* rcon = nullptr;
    CHECK_EQ(iad_line_open(IAD_LINE_AUTO, nullptr, IAD_FALSE, kNaN, &rcon), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_line_open(IAD_LINE_RCON, nullptr, IAD_FALSE, 0.005, &rcon), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_line_open(IAD_LINE_RCON, nullptr, IAD_FALSE, kNaN, &rcon), IAD_OK);
    REQUIRE(rcon != nullptr);
    CHECK(!allows(rcon, "coil"));
    CHECK(!allows(rcon, "iai_command"));
    CHECK(!allows(rcon, "baud_change"));
    CHECK(allows(rcon, "axis_control"));
    iad_bool flag = IAD_FALSE;
    CHECK_EQ(iad_line_allows(rcon, "teleport", &flag), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_line_require(rcon, "software_reset", nullptr), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(kind_of_error(), std::string("LineRuleError"));
    CHECK(has(last_json(), "ME0413 p.3-1"));
    CHECK(has(last_json(), "software reset: not sent on this RCON gateway line"));
    CHECK_EQ(iad_line_require(rcon, "broadcast", "the STOP broadcast"), IAD_ERR_UNSAFE_REFUSED);
    CHECK(has(last_json(), "the STOP broadcast: not sent"));
    // the guard: FC05 and broadcasts refused, a read passes, FC06 to the gateway's own address refused
    const uint8_t coil[5] = {0x05, 0x04, 0x2C, 0xFF, 0x00};
    const uint8_t read[5] = {0x03, 0x90, 0x00, 0x00, 0x08};
    const uint8_t drg1[5] = {0x06, 0x0D, 0x00, 0x10, 0x00};
    CHECK_EQ(iad_line_check_request(rcon, 1, coil, sizeof coil), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(iad_line_check_request(rcon, 0, drg1, sizeof drg1), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(iad_line_check_request(rcon, 1, read, sizeof read), IAD_OK);
    CHECK_EQ(iad_line_check_request(rcon, 1, drg1, sizeof drg1), IAD_OK);
    CHECK_EQ(iad_line_check_request(rcon, 0xD0, drg1, sizeof drg1), IAD_ERR_UNSAFE_REFUSED);
    // no gateway: control refused as GatewayModeError, the position unit unknown until stated
    CHECK_EQ(iad_line_require_control(rcon, 1), IAD_ERR_NOT_READY);
    CHECK_EQ(kind_of_error(), std::string("GatewayModeError"));
    double unit = 0.0;
    CHECK_EQ(iad_line_require_position_unit(rcon, 1, IAD_FALSE, &unit), IAD_ERR_NOT_READY);
    CHECK_EQ(kind_of_error(), std::string("PositionUnitError"));
    CHECK(has(last_json(), "No.207"));
    char source[16] = {0};
    size_t need = 0;
    CHECK_EQ(iad_line_position_unit(rcon, 1, &unit, source, sizeof source, &need), IAD_OK);
    CHECK(std::isnan(unit));
    CHECK_EQ(std::string(source), std::string(""));
    CHECK_EQ(iad_line_set_position_unit(rcon, 0.1, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_line_set_position_unit(rcon, 0.01, 0, nullptr), IAD_OK);
    CHECK_EQ(iad_line_set_position_unit(rcon, 0.001, 1, "read"), IAD_OK);
    CHECK_EQ(iad_line_require_position_unit(rcon, 1, IAD_FALSE, &unit), IAD_OK);
    CHECK_NEAR(unit, 0.001, 1e-12);
    CHECK_EQ(iad_line_position_unit(rcon, 1, &unit, source, sizeof source, &need), IAD_OK);
    CHECK_EQ(std::string(source), std::string("read"));
    CHECK_EQ(iad_line_require_position_unit(rcon, 2, IAD_FALSE, &unit), IAD_OK);
    CHECK_NEAR(unit, 0.01, 1e-12);
    // the explanations
    CHECK_EQ(iad_line_note_unreachable(rcon, "5,6"), IAD_OK);
    const std::string why =
        read_text([&](char* b, size_t s, size_t* r) { return iad_line_explain_unreachable(rcon, nullptr, b, s, r); });
    CHECK(has(why, "slave(s) 5, 6 are linked"));
    CHECK(has(why, "ME0413 p.4-1"));
    const Json j = line_json(rcon);
    CHECK_EQ(j.find("kind")->as_string(), std::string("rcon"));
    CHECK_EQ(j.find("detected")->as_bool(), false);
    CHECK(j.find("gateway")->is_null());
    CHECK_EQ(j.find("unreachable_slaves")->size(), 2u);
    CHECK_NEAR(j.find("position_units")->find("1")->find("unit_mm")->as_number(), 0.001, 1e-12);
    iad_gateway* none = reinterpret_cast<iad_gateway*>(1);
    CHECK_EQ(iad_line_gateway(rcon, &none), IAD_OK);
    CHECK(none == nullptr);
    CHECK_EQ(iad_line_close(rcon), IAD_OK);
    CHECK_EQ(iad_line_close(rcon), IAD_OK);  // closed: nothing to do

    iad_line* direct = nullptr;
    CHECK_EQ(iad_line_open(IAD_LINE_DIRECT, nullptr, IAD_FALSE, kNaN, &direct), IAD_OK);
    CHECK(allows(direct, "coil") && allows(direct, "iai_command") && allows(direct, "broadcast"));
    CHECK_EQ(iad_line_require_position_unit(direct, 1, IAD_TRUE, &unit), IAD_OK);
    CHECK_NEAR(unit, reg::kPositionScale, 1e-12);
    CHECK_EQ(iad_line_position_unit(direct, 0, &unit, source, sizeof source, &need), IAD_OK);
    CHECK_EQ(std::string(source), std::string("default"));
    CHECK_EQ(read_text([&](char* b, size_t s, size_t* r) { return iad_line_explain_unreachable(direct, "5", b, s, r); }),
             std::string(""));
    CHECK_EQ(iad_line_close(direct), IAD_OK);
}

TEST(capi_line_detected_rcon_line_and_its_gateway) {
    SimulatedUnitOptions unit = sim_gateway_unit_defaults();
    unit.time_scale = 20.0;
    SimGatewayOptions options;
    options.gateway_alarm = 0x8BA;
    options.serials = {{2, "RC-0042"}};
    auto sim = sim_rcon_line({1, 2}, options, unit);
    iad_bus* bus = bus_on(sim);
    // the probes: one read each, nothing written
    uint16_t words[16] = {0};
    size_t count = 0;
    double read_at = 0.0;
    CHECK_EQ(iad_bus_probe_gateway(bus, IAD_LINE_RCON, 0, kNaN, words, 16, &count, &read_at), IAD_OK);
    CHECK_EQ(count, 11u);
    CHECK((words[0] & 0x8000) != 0);  // MANU
    CHECK_EQ(iad_bus_probe_gateway(bus, IAD_LINE_ROBONET_GATEWAY, 0, 0.02, words, 16, &count, &read_at), IAD_OK);
    CHECK_EQ(count, 0u);
    CHECK_EQ(iad_bus_probe_gateway(bus, IAD_LINE_DIRECT, 0, kNaN, words, 16, &count, &read_at), IAD_ERR_INVALID_ARGUMENT);
    // detection attaches the line with its gateway
    int32_t found = -1;
    CHECK_EQ(iad_bus_detect_line(bus, IAD_LINE_AUTO, kNaN, kNaN, &found), IAD_OK);
    CHECK_EQ(found, IAD_LINE_RCON);
    iad_line* line = line_of(bus);
    const Json j = line_json(line);
    CHECK_EQ(j.find("detected")->as_bool(), true);
    CHECK_EQ(j.find("gateway")->find("manu")->as_bool(), true);
    iad_gateway* gw = nullptr;
    CHECK_EQ(iad_line_gateway(line, &gw), IAD_OK);
    REQUIRE(gw != nullptr);
    Json info = gateway_json(gw);
    CHECK_EQ(info.find("kind")->as_string(), std::string("rcon"));
    CHECK_EQ(info.find("address")->as_int(), 0xD0);
    CHECK_NEAR(info.find("cache_s")->as_number(), 2.0, 1e-12);
    CHECK_EQ(info.find("words")->size(), 11u);
    // the cached status is not read again inside its window; a negative age reads
    const std::size_t before = sim->rcon_gateway()->requests().size();
    CHECK_EQ(iad_gateway_status(gw, kNaN, words, 16, &count, &read_at), IAD_OK);
    CHECK_EQ(sim->rcon_gateway()->requests().size(), before);
    CHECK_EQ(iad_gateway_status(gw, -1.0, words, 16, &count, &read_at), IAD_OK);
    CHECK_EQ(sim->rcon_gateway()->requests().size(), before + 1);
    uint16_t small[2] = {0};
    CHECK_EQ(iad_gateway_status(gw, kNaN, small, 2, &count, &read_at), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(count, 11u);
    uint32_t alarm = 0;
    CHECK_EQ(iad_gateway_read_alarm(gw, &alarm), IAD_OK);
    CHECK_EQ(alarm, 0x8BAu);
    CHECK_EQ(iad_gateway_require_manu(gw, words, 16, &count, &read_at), IAD_OK);
    CHECK_EQ(iad_line_require_control(line, 1), IAD_OK);
    // AUTO: refused, and a cached AUTO is read again before refusing
    sim->rcon_gateway()->set_manu(false);
    CHECK_EQ(iad_gateway_set_cache_s(gw, 0.0), IAD_OK);
    CHECK_EQ(iad_line_require_control(line, 1), IAD_ERR_NOT_READY);
    CHECK_EQ(kind_of_error(), std::string("GatewayModeError"));
    CHECK(has(last_json(), "AUTO mode"));
    sim->rcon_gateway()->set_manu(true);
    CHECK_EQ(iad_line_require_control(line, 1), IAD_OK);
    CHECK_EQ(iad_gateway_set_cache_s(gw, -1.0), IAD_ERR_INVALID_ARGUMENT);
    // the all-axes monitor and the serial codes
    const std::string mon =
        read_text([&](char* b, size_t s, size_t* r) { return iad_gateway_read_monitor_json(gw, "0,1", b, s, r); });
    const Json records = Json::parse(mon);
    CHECK_EQ(records.find("0")->size(), 15u);
    CHECK_EQ(records.find("1")->size(), 15u);
    CHECK_EQ(read_text([&](char* b, size_t s, size_t* r) { return iad_gateway_read_monitor_json(gw, "16", b, s, r); }),
             std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(read_text([&](char* b, size_t s, size_t* r) { return iad_bus_read_serial_code(bus, 2, b, s, r); }),
             std::string("RC-0042"));
    // a line attached by the caller: its position units serve the bus's line
    iad_line* stated = nullptr;
    CHECK_EQ(iad_line_open(IAD_LINE_RCON, gw, IAD_TRUE, 0.01, &stated), IAD_OK);
    CHECK_EQ(iad_bus_attach_line(bus, stated), IAD_OK);
    iad_line* now = line_of(bus);
    double unit_mm = 0.0;
    CHECK_EQ(iad_line_require_position_unit(now, 2, IAD_TRUE, &unit_mm), IAD_OK);
    CHECK_NEAR(unit_mm, 0.01, 1e-12);
    CHECK_EQ(iad_line_set_position_unit(stated, 0.001, 2, nullptr), IAD_OK);
    CHECK_EQ(iad_line_require_position_unit(now, 2, IAD_TRUE, &unit_mm), IAD_OK);
    CHECK_NEAR(unit_mm, 0.001, 1e-12);
    // the gateway handle outlives the line handles; closing the bus ends its reads
    CHECK_EQ(iad_line_close(now), IAD_OK);
    CHECK_EQ(iad_line_close(stated), IAD_OK);
    CHECK_EQ(iad_line_close(line), IAD_OK);
    CHECK_EQ(iad_gateway_read_alarm(gw, &alarm), IAD_OK);
    CHECK_EQ(iad_gateway_close(gw), IAD_OK);
    CHECK_EQ(iad_gateway_close(gw), IAD_OK);
    CHECK_EQ(iad_gateway_read_alarm(gw, &alarm), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}

TEST(capi_line_gateway_objects_made_by_the_caller) {
    SimulatedUnitOptions unit = sim_gateway_unit_defaults();
    unit.time_scale = 20.0;
    auto sim = sim_robonet_line({1, 2}, true, unit);
    iad_bus* bus = bus_on(sim);
    iad_gateway* gw = nullptr;
    CHECK_EQ(iad_gateway_open(bus, IAD_LINE_ROBONET, 0, nullptr, 0, kNaN, kNaN, &gw), IAD_ERR_INVALID_ARGUMENT);
    const uint16_t known[2] = {0x8021, 0x0003};
    CHECK_EQ(iad_gateway_open(bus, IAD_LINE_ROBONET_GATEWAY, 0, known, 2, 1.5, kNaN, &gw), IAD_OK);
    REQUIRE(gw != nullptr);
    Json info = gateway_json(gw);
    CHECK_EQ(info.find("kind")->as_string(), std::string("robonet-gateway"));
    CHECK_EQ(info.find("address")->as_int(), 0x3F);
    CHECK(info.find("cache_s")->is_null());
    CHECK_NEAR(info.find("read_at")->as_number(), 1.5, 1e-12);
    uint16_t words[16] = {0};
    size_t count = 0;
    double read_at = 0.0;
    CHECK_EQ(iad_gateway_status(gw, kNaN, words, 16, &count, &read_at), IAD_OK);  // ROBONET: always a fresh read
    CHECK_EQ(count, 2u);
    CHECK((words[0] & 0x8000) != 0);  // RUN
    CHECK_EQ(words[1], 0x0003);      // LNK0 + LNK1
    uint32_t alarm = 0;
    CHECK_EQ(iad_gateway_read_alarm(gw, &alarm), IAD_ERR_INVALID_ARGUMENT);  // RCON only
    // a line in gateway mode gives no axis access
    iad_line* line = nullptr;
    CHECK_EQ(iad_line_open(IAD_LINE_ROBONET_GATEWAY, gw, IAD_TRUE, kNaN, &line), IAD_OK);
    CHECK_EQ(iad_bus_attach_line(bus, line), IAD_OK);
    CHECK_EQ(iad_line_require_control(line, 1), IAD_ERR_NOT_READY);
    CHECK_EQ(kind_of_error(), std::string("UnsupportedLineError"));
    CHECK(has(last_json(), "SW2 ON"));
    const uint8_t drg1[5] = {0x06, 0x0D, 0x00, 0x10, 0x00};
    CHECK_EQ(iad_line_check_request(line, 1, drg1, sizeof drg1), IAD_ERR_UNSAFE_REFUSED);
    CHECK(has(line_json(line).find("explanation")->as_string(), "SW2 ON"));
    CHECK_EQ(iad_line_close(line), IAD_OK);
    CHECK_EQ(iad_gateway_close(gw), IAD_OK);
    // an RCON gateway object over a line where nothing answers D0h: its MANU check refuses
    CHECK_EQ(iad_gateway_open(bus, IAD_LINE_RCON, 0, nullptr, 0, kNaN, 0.5, &gw), IAD_OK);
    CHECK_NEAR(gateway_json(gw).find("cache_s")->as_number(), 0.5, 1e-12);
    CHECK(gateway_json(gw).find("words")->is_null());
    CHECK_EQ(iad_gateway_require_manu(gw, words, 16, &count, &read_at), IAD_ERR_NOT_READY);
    CHECK_EQ(kind_of_error(), std::string("GatewayModeError"));
    CHECK(has(last_json(), "did not answer its status read"));
    CHECK_EQ(iad_gateway_close(gw), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}

TEST(capi_line_rules_stated_for_the_axes) {
    SimulatedUnitOptions unit = sim_gateway_unit_defaults();
    unit.time_scale = 20.0;
    auto sim = sim_robonet_line({1}, false, unit);  // through mode answers FC05 and FC11: the bus is a plain one
    iad_bus* bus = bus_on(sim);
    iad_axis_options o;
    iad_axis_options_init(&o);
    o.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, &o, &ax), IAD_OK);
    REQUIRE(ax != nullptr);
    auto line_name = [&] {
        return read_text([&](char* b, size_t s, size_t* r) { return iad_axis_line_name(ax, b, s, r); });
    };
    auto stop_path = [&] {
        return read_text([&](char* b, size_t s, size_t* r) { return iad_axis_stop_path(ax, b, s, r); });
    };
    CHECK_EQ(line_name(), std::string(""));
    CHECK_EQ(stop_path(), std::string("coil"));
    CHECK_EQ(iad_bus_state_line_rules(bus, "[1]"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_state_line_rules(bus, "{\"name\": \"rcon\", \"coils\": false, \"iai_commands\": false}"), IAD_OK);
    CHECK_EQ(line_name(), std::string("rcon"));
    CHECK_EQ(stop_path(), std::string("register"));
    iad_line* line = line_of(bus);
    CHECK(!allows(line, "coil"));
    CHECK(!allows(line, "software_reset"));
    CHECK(allows(line, "broadcast"));  // not stated: the kind's
    const uint8_t coil[5] = {0x05, 0x04, 0x2C, 0xFF, 0x00};
    CHECK_EQ(iad_line_check_request(line, 1, coil, sizeof coil), IAD_OK);  // the bus's guard keeps the kind's rules
    iad_reset_options reset;
    iad_reset_options_init(&reset);
    CHECK_EQ(iad_axis_software_reset(ax, &reset, nullptr), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(kind_of_error(), std::string("LineRuleError"));
    CHECK_EQ(iad_bus_state_line_rules(bus, nullptr), IAD_OK);  // cleared
    CHECK_EQ(line_name(), std::string(""));
    CHECK(allows(line, "coil"));
    CHECK_EQ(iad_line_close(line), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}
