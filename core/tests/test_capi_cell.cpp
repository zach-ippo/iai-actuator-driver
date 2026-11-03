// C ABI 0.5 ports and cells (src/c_api_cell.cpp) on simulated lines: a bus made a port (its owner's line detection,
// the scan, the identities, the rate found once), cells over buses (the scan, the listed slaves, the skipped ports, the
// axes of their own) and over axes that exist already, and the group commands' options. The Python facade
// (bindings/python/iai_actuator_driver/_native/cell.py) runs the Python edition's tests over these.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A simulated line of EC sliders at `slaves` and a bus on it: the line (closed by the caller) and the bus.
struct SimBus {
    std::vector<iad_sim*> units;
    iad_sim_line* line = nullptr;
    iad_bus* bus = nullptr;
};

SimBus open_line(const char* name, const std::vector<int>& slaves, const std::string& more = "") {
    SimBus out;
    for (int s : slaves) {
        iad_sim* sim = nullptr;
        const std::string options = R"({"time_scale": 20, "slave": )" + std::to_string(s) + more + "}";
        REQUIRE(iad_sim_open(options.c_str(), &sim) == IAD_OK);
        out.units.push_back(sim);
    }
    REQUIRE(iad_sim_line_open(name, out.units.data(), out.units.size(), &out.line) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_line_transport(out.line, &t) == IAD_OK);
    REQUIRE(iad_bus_open(t, nullptr, &out.bus) == IAD_OK);
    iad_transport_release(t);
    return out;
}

void close_line(SimBus& b) {
    iad_bus_close(b.bus);
    iad_sim_line_close(b.line);
    for (iad_sim* s : b.units) iad_sim_close(s);
}

template <class F>
Json json_of(F call) {
    std::string buffer(64 * 1024, '\0');
    size_t required = 0;
    REQUIRE(call(&buffer[0], buffer.size(), &required) == IAD_OK);
    return Json::parse(buffer.c_str());
}

std::string last_kind() {
    std::string buffer(8192, '\0');
    size_t required = 0;
    iad_last_error_json(&buffer[0], buffer.size(), &required);
    const Json j = Json::parse(buffer.c_str());
    const Json* kind = j.find("kind");
    return kind ? kind->as_string() : std::string();
}

int g_detections = 0;

iad_result IAD_CALL count_detection(void* user) {
    ++g_detections;
    iad_bus* bus = static_cast<iad_bus*>(user);
    return iad_bus_set_line(bus, IAD_LINE_DIRECT, kNaN);  // the owner states the line itself
}

}  // namespace

TEST(capi_cell_port_scan_identify_and_owner_detection) {
    SimBus b = open_line("L", {1, 5});
    g_detections = 0;
    CHECK_EQ(iad_bus_set_port(b.bus, "L", IAD_LINE_AUTO, kNaN, IAD_FALSE, count_detection, b.bus), IAD_OK);
    const Json hits = json_of([&](char* p, size_t n, size_t* r) {
        return iad_bus_scan_units_json(b.bus, "1,2,5", 0.01, IAD_TRUE, IAD_TRUE, p, n, r);
    });
    REQUIRE(hits.size() == 2u);
    CHECK_EQ(hits.as_array()[0].find("name")->as_string(), std::string("L:1"));
    CHECK_EQ(hits.as_array()[1].find("slave")->as_int(), 5LL);
    CHECK_EQ(hits.as_array()[0].find("system_code")->as_int(), 0xECLL);
    CHECK_EQ(g_detections, 1);  // the owner was asked once, no gateway probe of the port's own
    int32_t found[4] = {0};
    size_t count = 0;
    CHECK_EQ(iad_bus_scan(b.bus, "1,2,3", 0.01, IAD_TRUE, found, 4, &count), IAD_OK);
    CHECK_EQ(count, 1u);
    CHECK_EQ(found[0], 1);
    int32_t kind = 0;
    CHECK_EQ(iad_bus_port_detect_line(b.bus, IAD_FALSE, &kind), IAD_OK);
    CHECK_EQ(kind, IAD_LINE_DIRECT);
    CHECK_EQ(g_detections, 1);
    CHECK_EQ(iad_bus_port_detect_line(b.bus, IAD_TRUE, &kind), IAD_OK);  // force: asked again
    CHECK_EQ(g_detections, 2);
    const Json one = json_of([&](char* p, size_t n, size_t* r) { return iad_bus_identify_json(b.bus, 5, p, n, r); });
    CHECK_EQ(one.find("firmware")->type(), Json::Type::String);
    const Json silent = json_of([&](char* p, size_t n, size_t* r) { return iad_bus_identify_json(b.bus, 9, p, n, r); });
    CHECK(silent.find("error")->type() == Json::Type::String);  // kept in the hit, never returned
    const Json info = json_of([&](char* p, size_t n, size_t* r) { return iad_bus_port_info_json(b.bus, p, n, r); });
    CHECK_EQ(info.find("name")->as_string(), std::string("L"));
    CHECK_EQ(info.find("line")->as_string(), std::string("direct"));
    CHECK_EQ(iad_bus_scan_units_json(b.bus, "0,1", kNaN, IAD_FALSE, IAD_TRUE, nullptr, 0, &count),
             IAD_ERR_INVALID_ARGUMENT);
    close_line(b);
}

TEST(capi_cell_open_buses_scans_skips_dead_ports_and_lends_its_axes) {
    SimBus a = open_line("A", {1, 2});
    SimBus dead = open_line("B", {});
    CHECK_EQ(iad_bus_set_port(a.bus, "A", IAD_LINE_DIRECT, kNaN, IAD_FALSE, nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_bus_set_port(dead.bus, "B", IAD_LINE_DIRECT, kNaN, IAD_FALSE, nullptr, nullptr), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    co.scan_timeout_s = 0.01;
    iad_bus* buses[] = {a.bus, dead.bus};
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open_buses(buses, 2, nullptr, "1,2,3", &co, nullptr, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    size_t count = 0;
    CHECK_EQ(iad_cell_axis_count(cell, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    const Json errors =
        json_of([&](char* p, size_t n, size_t* r) { return iad_cell_port_errors_json(cell, p, n, r); });
    REQUIRE(errors.find("B") != nullptr);
    CHECK_EQ(errors.find("B")->find("kind")->as_string(), std::string("ModbusTimeout"));
    CHECK(errors.find("B")->find("notes")->size() >= 1u);  // the no-answer hint
    iad_axis* a2 = nullptr;
    CHECK_EQ(iad_cell_axis_ref(cell, "A:2", &a2), IAD_OK);
    const Json limits = json_of([&](char* p, size_t n, size_t* r) {
        return iad_cell_read_all_json(cell, "limits", nullptr, p, n, r);
    });
    CHECK(limits.find("results")->find("A:1") != nullptr);
    CHECK_EQ(limits.find("errors")->size(), 0u);
    CHECK_EQ(iad_cell_read_all_json(cell, "teleport", nullptr, nullptr, 0, &count), IAD_ERR_INVALID_ARGUMENT);
    iad_cell_result results[2];
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    CHECK_EQ(iad_cell_release(cell), IAD_OK);
    iad_status st;
    CHECK_EQ(iad_axis_status(a2, &st), IAD_OK);  // the reference outlives the cell handle
    CHECK_EQ(iad_axis_close(a2), IAD_OK);
    close_line(a);
    close_line(dead);
}

TEST(capi_cell_open_buses_listed_slaves_must_answer) {
    SimBus a = open_line("L", {1});
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    co.scan_timeout_s = 0.01;
    iad_bus* buses[] = {a.bus};
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open_buses(buses, 1, R"({"L": [1, 2]})", nullptr, &co, nullptr, &cell), IAD_ERR_MODBUS_TIMEOUT);
    CHECK(cell == nullptr);
    CHECK_EQ(iad_cell_open_buses(buses, 1, R"({"M": [1]})", nullptr, &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_buses(buses, 1, "[1, true]", nullptr, &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    iad_bus* twice[] = {a.bus, a.bus};
    CHECK_EQ(iad_cell_open_buses(twice, 2, nullptr, nullptr, &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    close_line(a);
}

TEST(capi_cell_open_axes_moves_with_the_options_and_leaves_its_axes_open) {
    SimBus b = open_line("L", {1, 2});
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* axes[2] = {nullptr, nullptr};
    CHECK_EQ(iad_axis_open_bus(b.bus, 1, &ao, &axes[0]), IAD_OK);
    CHECK_EQ(iad_axis_open_bus(b.bus, 2, &ao, &axes[1]), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open_axes(axes, 2, "L:1,L:1", &co, &cell), IAD_ERR_INVALID_ARGUMENT);  // two axes named L:1
    CHECK_EQ(iad_cell_open_axes(axes, 2, nullptr, &co, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);
    for (int i = 0; i < 2; ++i) {
        moves[i].struct_size = sizeof moves[i];
        moves[i].speed = 100.0;
        moves[i].accel_g = 0.3;
        moves[i].position = 20.0 + 30.0 * i;
        std::snprintf(moves[i].axis, sizeof moves[i].axis, "L:%d", i + 1);
    }
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    CHECK(std::fabs(results[1].status.position_mm - 50.0) < 0.1);
    moves[0].position = 9999.0;  // refused at the check: nothing started (0.8: named move_each, the bindings rename it)
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_FALSE, kNaN, IAD_TRUE, results, 2, &count), IAD_ERR_CELL);
    CHECK_EQ(results[0].code, IAD_ERR_SOFT_LIMIT);
    std::string buffer(8192, '\0');
    size_t required = 0;
    iad_last_error_json(&buffer[0], buffer.size(), &required);
    const Json err = Json::parse(buffer.c_str());
    CHECK_EQ(err.find("operation")->as_string(), std::string("move_each (check; nothing started)"));
    CHECK(err.find("axes")->find("L:1")->find("notes") != nullptr);  // every detail of that axis's error
    CHECK_EQ(iad_cell_home_all(cell, "L:2", kNaN, IAD_METHOD_DEFAULT, IAD_FALSE, results, 2, &count), IAD_OK);
    CHECK_EQ(count, 1u);
    CHECK(results[0].status.homed);
    CHECK_EQ(iad_cell_wait_all(cell, nullptr, 2.0, IAD_FALSE, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_home_all(cell, nullptr, kNaN, 7, IAD_TRUE, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(last_kind(), std::string("std::invalid_argument"));
    CHECK_EQ(iad_cell_release(cell), IAD_OK);
    iad_status st;
    CHECK_EQ(iad_axis_status(axes[0], &st), IAD_OK);  // a released cell of iad_cell_open_axes leaves its axes open
    iad_axis_close(axes[0]);
    iad_axis_close(axes[1]);
    close_line(b);
}

TEST(capi_cell_stop_all_and_pause_all_with_an_unknown_name_still_act_on_the_axes_they_have) {
    // a remembered list naming an axis that dropped out of a reopened cell: the STOP still goes out to the others
    SimBus b = open_line("L", {1, 2});
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* axes[2] = {nullptr, nullptr};
    CHECK_EQ(iad_axis_open_bus(b.bus, 1, &ao, &axes[0]), IAD_OK);
    CHECK_EQ(iad_axis_open_bus(b.bus, 2, &ao, &axes[1]), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_axes(axes, 2, nullptr, &co, &cell) == IAD_OK);
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);
    for (int i = 0; i < 2; ++i) {
        moves[i].struct_size = sizeof moves[i];
        moves[i].speed = 2.0;  // 3 s of real time at the simulator's x20 (120 mm at 2 mm/s)
        moves[i].position = 140.0;
        std::snprintf(moves[i].axis, sizeof moves[i].axis, "L:%d", i + 1);
    }
    iad_cell_result results[3];
    size_t count = 0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_FALSE, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_stop_all(cell, "L:1,NOSUCH:9", kNaN, IAD_FALSE, results, 3, &count), IAD_ERR_CELL);
    REQUIRE(count == 2u);
    CHECK_EQ(std::string(results[0].axis), std::string("L:1"));
    CHECK_EQ(results[0].code, IAD_OK);
    CHECK_EQ(std::string(results[1].axis), std::string("NOSUCH:9"));
    CHECK_EQ(results[1].code, IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(results[1].index, 1);
    CHECK(std::string(results[1].message).find("no axis 'NOSUCH:9'") != std::string::npos);
    iad_status st;
    CHECK_EQ(iad_axis_status(axes[0], &st), IAD_OK);
    CHECK(!st.moving);
    CHECK(st.position_mm < 139.0);
    CHECK_EQ(iad_axis_status(axes[1], &st), IAD_OK);
    CHECK(st.moving);  // not listed: still on its way
    // pause_all the same way: the axis it has is paused, the unknown name reported after it
    CHECK_EQ(iad_cell_pause_all(cell, "NOSUCH:9,L:2", IAD_FALSE, results, 3, &count), IAD_ERR_CELL);
    REQUIRE(count == 2u);
    CHECK_EQ(std::string(results[0].axis), std::string("L:2"));
    CHECK_EQ(results[0].code, IAD_OK);
    CHECK_EQ(results[1].code, IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_status(axes[1], &st), IAD_OK);
    CHECK(st.paused);
    CHECK_EQ(iad_cell_resume_all(cell, "L:2", IAD_FALSE, results, 3, &count), IAD_OK);
    // only unknown names: nothing to stop, each one reported (never "every axis")
    CHECK_EQ(iad_cell_stop_all(cell, "NOSUCH:9,NOSUCH:8", kNaN, IAD_FALSE, results, 3, &count), IAD_ERR_CELL);
    CHECK_EQ(count, 2u);
    CHECK_EQ(iad_axis_status(axes[1], &st), IAD_OK);
    CHECK(st.moving);
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_FALSE, results, 3, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    CHECK_EQ(iad_cell_release(cell), IAD_OK);
    iad_axis_close(axes[0]);
    iad_axis_close(axes[1]);
    close_line(b);
}

TEST(capi_cell_baud_change_of_the_whole_line) {
    SimBus b = open_line("L", {1, 2}, R"(, "baud_rate": 115200)");
    CHECK_EQ(iad_bus_set_port(b.bus, "L", IAD_LINE_DIRECT, kNaN, IAD_FALSE, nullptr, nullptr), IAD_OK);
    int32_t rate = 0;
    iad_bool detected = IAD_FALSE;
    CHECK_EQ(iad_bus_ensure_baud_rate(b.bus, "1,2", &rate, &detected), IAD_OK);
    CHECK_EQ(rate, 115200);
    iad_baud_options o;
    iad_baud_options_init(&o);
    size_t required = 0;
    std::string buffer(8192, '\0');
    CHECK_EQ(iad_bus_set_baud_rate_all(b.bus, 12345, "1,2", &o, &buffer[0], buffer.size(), &required),
             IAD_ERR_SOFT_LIMIT);  // a rate without an IAI code: nothing sent
    CHECK_EQ(iad_bus_set_baud_rate_all(b.bus, 230400, "", &o, &buffer[0], buffer.size(), &required),
             IAD_ERR_NOT_READY);  // no axis to verify it with
    const Json r = json_of([&](char* p, size_t n, size_t* q) {
        return iad_bus_set_baud_rate_all(b.bus, 230400, "1,2", &o, p, n, q);
    });
    CHECK_EQ(r.find("rate")->as_int(), 230400LL);
    CHECK(r.find("answered")->find("2")->as_bool());
    close_line(b);
}
