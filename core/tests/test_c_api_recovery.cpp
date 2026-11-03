// C ABI 0.6 additions of the recovery facade (include/iai_actuator_driver.h "axis: recovery sessions, link state",
// src/c_api_recovery.cpp) on the simulator: the session from an identity and its volatile fields, the link state set
// with its event, the port's presence, the restart signs of a status block, the event sources, the report's
// control_kept, the policy's require_session and the cell's link monitor read back. Simulator only: no COM port is
// opened. Timing: time_scale 50, generous deadlines (the PC may be busy).
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// A JSON / text output read the way a binding does (size query, then the text); "<IAD_...>" when it failed.
template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 50; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

iad_axis* sim_axis(const char* profile) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 50.0;
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim(profile, &sim, &open, &ao, &ax) == IAD_OK);
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    return ax;
}

std::shared_ptr<SimulatedUnit> sim_of(iad_axis* ax) {
    return std::dynamic_pointer_cast<SimulatedUnit>(capi::find_axis(ax)->axis->bus()->transport_ptr());
}

std::string axis_text(iad_axis* ax, iad_result (*f)(iad_axis*, char*, size_t, size_t*)) {
    return read_text([&](char* b, size_t n, size_t* r) { return f(ax, b, n, r); });
}

/// The events queued on the handle now (no wait beyond `seconds`).
std::vector<iad_event> events_of(iad_axis* ax, double seconds = 0.2) {
    std::vector<iad_event> out;
    const double end = now_s() + seconds;
    while (now_s() < end) {
        iad_event ev;
        if (iad_axis_next_event(ax, 0.02, &ev) != IAD_OK) break;
        if (ev.kind != IAD_EVENT_NONE) out.push_back(ev);
    }
    return out;
}

iad_recovery_policy quick_policy() {
    iad_recovery_policy p;
    iad_recovery_policy_init(&p);
    p.poll_s = 0.02;
    return p;
}

/// The restart signs of the unit's present status block.
std::string signs_now(iad_axis* ax, const char* options) {
    iad_status st;
    REQUIRE(iad_axis_status(ax, &st) == IAD_OK);
    uint16_t words[22];
    size_t count = 0;
    REQUIRE(iad_axis_last_status_words(ax, &st, words, 22, &count) == IAD_OK);
    return read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_restart_signs(ax, words, count, options, b, n, r); });
}

}  // namespace

TEST(capi_recovery_session_from_info_and_its_fields) {
    iad_axis* ax = sim_axis("gripper");
    CHECK_EQ(iad_axis_session_from_info(ax, "[1]", "G"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_from_info(ax, "{\"serial_number\":\"SN-1\",\"soft_limits_mm\":[0,20]}", "G"), IAD_OK);
    std::string s = axis_text(ax, iad_axis_session_json);
    CHECK(has(s, "\"serial_number\":\"SN-1\"") && has(s, "\"name\":\"G\"") && has(s, "\"modbus_control\":null"));
    // a unit with a session keeps it
    CHECK_EQ(iad_axis_session_from_info(ax, nullptr, "other"), IAD_OK);
    CHECK(has(axis_text(ax, iad_axis_session_json), "\"name\":\"G\""));
    CHECK_EQ(iad_axis_forget(ax), IAD_OK);
    CHECK_EQ(iad_axis_session_from_info(ax, nullptr, nullptr), IAD_OK);  // the identity connect() read
    CHECK(!has(axis_text(ax, iad_axis_session_json), "\"serial_number\":null"));
    // the volatile fields, all checked before anything is written
    CHECK_EQ(iad_axis_session_update(ax, "{\"recovering\":true,\"homed\":true,\"baudrate\":115200}"), IAD_OK);
    s = axis_text(ax, iad_axis_session_json);
    CHECK(has(s, "\"recovering\":true") && has(s, "\"homed\":true") && has(s, "\"baudrate\":115200"));
    iad_link_check lc;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(std::string(lc.state), std::string("recovering"));
    CHECK_EQ(iad_axis_session_update(ax, "{\"recovering\":false,\"colour\":1}"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(axis_text(ax, iad_axis_session_json), "\"recovering\":true"));  // nothing written
    CHECK_EQ(iad_axis_session_update(ax, "{\"homed\":3}"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_update(ax, "{\"recovering\":false,\"baudrate\":38400,\"name\":null}"), IAD_OK);
    CHECK(has(axis_text(ax, iad_axis_session_json), "\"name\":null"));
    CHECK_EQ(iad_axis_forget(ax), IAD_OK);
    CHECK_EQ(iad_axis_session_update(ax, "{}"), IAD_ERR_NOT_READY);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_recovery_uptime_age_restart_signs_and_link_state) {
    iad_axis* ax = sim_axis("gripper");
    CHECK_EQ(signs_now(ax, nullptr), std::string("<IAD_ERR_NOT_READY>"));
    CHECK_EQ(iad_axis_track(ax, nullptr, "G", nullptr, nullptr), IAD_OK);
    CHECK_EQ(signs_now(ax, nullptr), std::string("[]"));
    CHECK(has(signs_now(ax, "{\"new_records\":[[4095,0,65535,null]]}"), "power-up record"));
    CHECK(has(signs_now(ax, "{\"baud_rate_found\":38400}"), "[]"));  // the session's own rate
    // an uptime reading an hour old: the unit's counter is behind the PC's clock
    CHECK_EQ(iad_axis_session_update(ax, "{\"uptime_age_s\":3600}"), IAD_OK);
    CHECK(has(signs_now(ax, nullptr), "behind"));
    iad_link_check lc;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(std::string(lc.state), std::string("unit_restarted"));
    std::vector<iad_event> events = events_of(ax);
    REQUIRE(events.size() == 1);
    CHECK_EQ(std::string(events[0].name), std::string("unit_restarted"));
    CHECK_EQ(events[0].source, IAD_EVENT_SOURCE_CHECK_LINK);
    // the state set by hand: the event says where it came from
    iad_bool changed = IAD_FALSE;
    CHECK_EQ(iad_axis_set_link_state(ax, "nonsense", nullptr, nullptr, &changed), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_link_state(ax, "ok", nullptr, "X", &changed), IAD_OK);
    CHECK_EQ(changed, IAD_TRUE);
    CHECK_EQ(iad_axis_set_link_state(ax, "ok", nullptr, nullptr, &changed), IAD_OK);
    CHECK_EQ(changed, IAD_FALSE);
    events = events_of(ax);
    REQUIRE(events.size() == 1);
    CHECK_EQ(std::string(events[0].name), std::string("recovered"));
    CHECK_EQ(std::string(events[0].axis), std::string("X"));
    CHECK_EQ(events[0].source, IAD_EVENT_SOURCE_SET_LINK_STATE);
    // the simulator tells its presence
    int32_t present = 7;
    CHECK_EQ(iad_axis_port_present(ax, &present), IAD_OK);
    CHECK_EQ(present, 1);
    sim_of(ax)->unplug();
    CHECK_EQ(iad_axis_port_present(ax, &present), IAD_OK);
    CHECK_EQ(present, 0);
    sim_of(ax)->replug();
    CHECK_EQ(iad_axis_port_present(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_recovery_control_kept_require_session_and_the_recover_source) {
    iad_axis* ax = sim_axis("gripper");
    iad_recovery_policy policy = quick_policy();
    policy.require_session = IAD_TRUE;
    iad_recovery_summary rs;
    CHECK_EQ(iad_axis_recover(ax, 1.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_ERR_RECOVERY);
    CHECK(has(iad_last_error_message(), "track(axis)"));
    CHECK_EQ(axis_text(ax, iad_axis_session_json), std::string("<IAD_ERR_NOT_READY>"));  // no session made
    policy.require_session = IAD_FALSE;  // the identity connect() read is the baseline
    CHECK_EQ(iad_axis_recover(ax, 1.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_OK);
    CHECK_EQ(rs.same_unit, IAD_TRUE);
    // PC control held before a power cycle: taken again, not kept
    CHECK_EQ(iad_axis_track(ax, nullptr, "G", nullptr, nullptr), IAD_OK);
    events_of(ax);
    sim_of(ax)->power_cycle();
    policy.require_session = IAD_TRUE;
    CHECK_EQ(iad_axis_recover(ax, 5.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_OK);
    CHECK(rs.control_restored == IAD_TRUE && rs.control_kept == IAD_FALSE);
    CHECK(has(axis_text(ax, iad_axis_last_result_json), "\"control_kept\":false"));
    std::vector<iad_event> events = events_of(ax);
    REQUIRE(!events.empty());
    for (const auto& e : events) CHECK_EQ(e.source, IAD_EVENT_SOURCE_RECOVER);
    // still held after the recovery (nothing to take again): kept
    CHECK_EQ(iad_axis_recover(ax, 5.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_OK);
    CHECK(rs.control_restored == IAD_FALSE && rs.control_kept == IAD_TRUE);
    CHECK(has(axis_text(ax, iad_axis_last_result_json), "\"control_kept\":true"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_recovery_cell_link_monitor_json) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 50.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, &ao, &cell) == IAD_OK);
    auto cell_text = [&] {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_cell_link_monitor_json(cell, b, n, r); });
    };
    CHECK_EQ(cell_text(), std::string("null"));
    CHECK_EQ(iad_cell_monitor_links(cell, 0.02, nullptr), IAD_OK);
    const double end = now_s() + 5.0;
    std::string j = cell_text();
    while (now_s() < end && !(has(j, "\"SIM1:1\":{") && has(j, "\"SIM2:1\":{"))) {
        sleep_at_least(0.01);
        j = cell_text();
    }
    CHECK(has(j, "\"running\":true") && has(j, "\"interval_s\":0.02") && has(j, "\"errors\":[]"));
    CHECK(has(j, "\"SIM1:1\":{") && has(j, "\"state\":\"ok\""));
    iad_event ev;
    CHECK_EQ(iad_cell_stop_link_monitor(cell), IAD_OK);
    CHECK_EQ(cell_text(), std::string("null"));
    CHECK_EQ(iad_cell_next_event(cell, 0.0, &ev), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi_recovery_a_value_of_the_wrong_type_or_range_is_an_invalid_argument) {
    // (0.7) a Json accessor of the wrong type used to throw std::logic_error (IAD_ERR_UNSUPPORTED, "not in this
    // build"), and a number past int was narrowed (undefined: x86 and ARM64 differed)
    iad_axis* ax = sim_axis("gripper");
    CHECK_EQ(iad_axis_session_from_info(ax, "{\"soft_limits_mm\":[null,20]}", "G"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(iad_last_error_message(), "soft_limits_mm[0]"));
    CHECK_EQ(iad_axis_session_from_info(ax, "{\"system_code\":1e10}", "G"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(iad_last_error_message(), "system_code"));
    CHECK_EQ(iad_axis_session_from_info(ax, "{\"unit_code\":2.5}", "G"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(axis_text(ax, iad_axis_session_json), std::string("<IAD_ERR_NOT_READY>"));  // none made
    CHECK_EQ(iad_axis_session_from_info(ax, "{\"serial_number\":\"SN-1\",\"system_code\":236.0}", "G"), IAD_OK);
    CHECK(has(axis_text(ax, iad_axis_session_json), "\"system_code\":236,"));
    // the session's whole-number fields: a fraction, a number past the field's range or text is refused, not narrowed
    for (const char* fields : {"{\"baudrate\":3e9}", "{\"baudrate\":9600.7}", "{\"baudrate\":\"9600\"}",
                               "{\"baudrate\":-9600}", "{\"uptime_ms\":1e30}", "{\"uptime_ms\":1.5}"}) {
        CHECK_EQ(iad_axis_session_update(ax, fields), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(read_text([](char* b, size_t n, size_t* r) { return iad_last_error_json(b, n, r); }),
                  "\"kind\":\"std::invalid_argument\""));
    }
    CHECK_EQ(iad_axis_session_update(ax, "{\"baudrate\":9600.0,\"uptime_ms\":1000}"), IAD_OK);
    const std::string s = axis_text(ax, iad_axis_session_json);
    CHECK(has(s, "\"baudrate\":9600,") && has(s, "\"uptime_ms\":1000,"));
    // every option of the restart signs
    for (const char* options : {"{\"baud_rate_found\":\"38400\"}", "{\"boot_baud_rate\":true}", "{\"now_s\":\"1\"}",
                                "{\"new_records\":[[null,0,0]]}", "{\"new_records\":[[1,2,3,\"t\"]]}",
                                "{\"baud_rate_found\":3e9}", "{\"boot_baud_rate\":38400.5}"})
        CHECK_EQ(signs_now(ax, options), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(signs_now(ax, "{\"baud_rate_found\":9600.0,\"new_records\":[]}"), std::string("[]"));  // the session's
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_testing_set_port_list_replaces_the_pcs_ports) {
    auto ports = [] { return read_text([](char* b, size_t n, size_t* r) { return iad_list_ports_json(b, n, r); }); };
    CHECK_EQ(iad_testing_set_port_list(
                 "[{\"port\":\"COM42\"},{\"port\":\"COM7\",\"vid\":4292,\"pid\":33239,\"description\":\"IAI USB\","
                 "\"manufacturer\":null,\"is_iai\":true}]"),
             IAD_OK);
    const std::string listed = ports();
    // ordered as the PC's: the IAI adapter first
    CHECK(has(listed, "[{\"port\":\"COM7\",\"description\":\"IAI USB\",\"manufacturer\":\"\",\"hardware_id\":\"\","
                      "\"vid\":4292,\"pid\":33239,\"is_iai\":true},{\"port\":\"COM42\""));
#ifdef _WIN32
    CHECK(port_present("com42"));  // a port name is not case-sensitive on Windows
#endif
    CHECK(port_present("COM42") && !port_present("COM43"));
    CHECK(iai_port_names() == std::vector<std::string>{"COM7"});
    // a wrong list leaves the one set
    for (const char* bad : {"{}", "[1]", "[{}]", "[{\"port\":\"\"}]", "[{\"port\":\"COM1\",\"vid\":70000}]",
                            "[{\"port\":\"COM1\",\"vid\":1.5}]", "[{\"port\":\"COM1\",\"colour\":1}]",
                            "[{\"port\":3}]", "[{\"port\":\"COM1\",\"description\":1}]", "nonsense"})
        CHECK_EQ(iad_testing_set_port_list(bad), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(ports(), listed);
    CHECK_EQ(iad_testing_set_port_list("[]"), IAD_OK);
    CHECK_EQ(ports(), std::string("[]"));
    CHECK(!port_present("COM42"));
    CHECK_EQ(iad_testing_set_port_list(nullptr), IAD_OK);  // the PC's enumeration again
    CHECK(!has(ports(), "COM42"));
}
