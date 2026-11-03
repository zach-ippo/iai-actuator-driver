// C ABI: two rules of the follow-up to the final review of 1.0, each the same kind as one already fixed there.
//  - The cancel calls (iad_cell_cancel, iad_cell_cancel_latch, iad_cell_cancel_reset) reach every listed axis the cell
//    has and then report each unknown name as an error of its own (IAD_ERR_CELL, the "axes" of iad_last_error_json), as
//    iad_cell_stop_all does; before, one unknown name refused the whole call and a cancel (a Ctrl+C, a watchdog) never
//    reached the axes that were waiting.
//  - The "timeout_s" of a coordinated request (iad_cell_coord_run_json, iad_cell_coord_plan_json) is absent for the
//    default, else a finite number of seconds above 0: 0 or a negative value is IAD_ERR_INVALID_ARGUMENT before
//    anything is sent (before, a run with 0 went out and was decel-stopped at once).
// Simulator only: no COM port is opened.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <thread>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void sleep_s(double seconds) { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }

/// Two simulated sliders on two ports (SIM1:1, SIM2:1), both at `start`; the simulators run `scale` times faster.
iad_cell* open_cell(double start, double scale) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = scale;
    sim.position_mm = start;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, &ao, &cell) == IAD_OK);
    return cell;
}

bool latched(iad_cell* cell, const char* name) {
    iad_axis* ax = nullptr;
    REQUIRE(iad_cell_open_axis(cell, name, &ax) == IAD_OK);
    iad_bool on = IAD_FALSE;
    REQUIRE(iad_axis_cancel_latched(ax, &on) == IAD_OK);
    iad_axis_close(ax);
    return on != IAD_FALSE;
}

double position(iad_cell* cell, const char* name) {
    iad_axis* ax = nullptr;
    if (iad_cell_open_axis(cell, name, &ax) != IAD_OK) return kNaN;
    iad_status st;
    const double p = iad_axis_status(ax, &st) == IAD_OK ? st.position_mm : kNaN;
    iad_axis_close(ax);
    return p;
}

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

/// The "operation" and the per-axis codes and messages of the last error, as a binding reads them.
struct CellFailure {
    std::string operation;
    std::map<std::string, long long> codes;
    std::map<std::string, std::string> messages;
};

CellFailure last_cell_failure() {
    const Json j = Json::parse(last_json());
    CellFailure out;
    if (const Json* op = j.find("operation")) out.operation = op->as_string();
    if (const Json* axes = j.find("axes"))
        for (const auto& kv : axes->as_object()) {
            out.codes[kv.first] = kv.second.find("code")->as_int();
            out.messages[kv.first] = kv.second.find("message")->as_string();
        }
    return out;
}

/// The code of one axis of a failure (0 when it has none): a missing entry is a failed check, never an exception.
long long code_of(const CellFailure& failure, const std::string& name) {
    const auto it = failure.codes.find(name);
    return it == failure.codes.end() ? 0 : it->second;
}

std::string message_of(const CellFailure& failure, const std::string& name) {
    const auto it = failure.messages.find(name);
    return it == failure.messages.end() ? std::string() : it->second;
}

/// Joins its thread when the test ends, however it ends (a failed check never leaves a running thread behind).
struct Joiner {
    std::thread& thread;
    ~Joiner() {
        if (thread.joinable()) thread.join();
    }
};

}  // namespace

TEST(capi_cancel_latch_and_reset_reach_every_listed_axis_then_report_the_unknown_names) {
    iad_cell* cell = open_cell(50.0, 20.0);
    REQUIRE(cell != nullptr);
    // a remembered list naming axes that dropped out of a reopened cell: the axes the cell has are latched all the same
    CHECK_EQ(iad_cell_cancel_latch(cell, "NOSUCH:9,SIM1:1,NOSUCH:8,NOSUCH:9"), IAD_ERR_CELL);
    const CellFailure latch = last_cell_failure();
    CHECK_EQ(latch.operation, std::string("cancel_latch"));
    CHECK_EQ(latch.codes.size(), 2u);  // each unknown name once, the axis it has not at all
    CHECK_EQ(latch.codes.count("SIM1:1"), 0u);
    CHECK_EQ(code_of(latch, "NOSUCH:9"), static_cast<long long>(IAD_ERR_INVALID_ARGUMENT));
    CHECK_EQ(code_of(latch, "NOSUCH:8"), static_cast<long long>(IAD_ERR_INVALID_ARGUMENT));
    CHECK(has(message_of(latch, "NOSUCH:9"), "no axis 'NOSUCH:9' in this cell"));
    CHECK(latched(cell, "SIM1:1"));
    CHECK(!latched(cell, "SIM2:1"));  // not listed
    CHECK_EQ(iad_cell_cancel_latch(cell, "SIM2:1"), IAD_OK);
    CHECK(latched(cell, "SIM2:1"));
    // the reset the same way: SIM1:1 is cleared although the list names an unknown axis
    CHECK_EQ(iad_cell_cancel_reset(cell, "SIM1:1,NOSUCH:9"), IAD_ERR_CELL);
    const CellFailure reset = last_cell_failure();
    CHECK_EQ(reset.operation, std::string("cancel_reset"));
    CHECK_EQ(reset.codes.size(), 1u);
    CHECK_EQ(code_of(reset, "NOSUCH:9"), static_cast<long long>(IAD_ERR_INVALID_ARGUMENT));
    CHECK(!latched(cell, "SIM1:1"));
    CHECK(latched(cell, "SIM2:1"));
    // only unknown names: nothing is touched, never "every axis"
    CHECK_EQ(iad_cell_cancel_reset(cell, "NOSUCH:9"), IAD_ERR_CELL);
    CHECK(latched(cell, "SIM2:1"));
    CHECK_EQ(iad_cell_cancel_latch(cell, "NOSUCH:9,NOSUCH:7"), IAD_ERR_CELL);
    CHECK_EQ(last_cell_failure().codes.size(), 2u);
    CHECK(!latched(cell, "SIM1:1"));
    // NULL and "" are every axis; a plain call still succeeds
    CHECK_EQ(iad_cell_cancel_latch(cell, nullptr), IAD_OK);
    CHECK(latched(cell, "SIM1:1") && latched(cell, "SIM2:1"));
    CHECK_EQ(iad_cell_cancel_reset(cell, ""), IAD_OK);
    CHECK(!latched(cell, "SIM1:1") && !latched(cell, "SIM2:1"));
    CHECK_EQ(iad_cell_cancel_latch(nullptr, "SIM1:1"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi_cancel_ends_a_waiting_axis_though_the_list_names_an_unknown_one) {
    iad_cell* cell = open_cell(20.0, 1.0);  // real time: 120 mm at 20 mm/s is about 6 s
    REQUIRE(cell != nullptr);
    iad_axis* ax = nullptr;
    REQUIRE(iad_cell_open_axis(cell, "SIM1:1", &ax) == IAD_OK);
    std::atomic<int> moved{1};  // IAD_OK is 0, every error negative: 1 = still running
    std::thread mover([&] {
        iad_move_options slow;
        iad_move_options_init(&slow);
        slow.speed = 20.0;
        iad_status st;
        moved = iad_axis_move_to(ax, 140.0, &slow, &st);
    });
    const Joiner join{mover};
    sleep_s(0.5);
    // only unknown names: nothing is cancelled, the move goes on
    CHECK_EQ(iad_cell_cancel(cell, "NOSUCH:9"), IAD_ERR_CELL);
    sleep_s(0.2);
    CHECK_EQ(moved.load(), 1);
    // an unknown name beside the waiting axis: the axis is cancelled (a decel stop), the unknown name reported
    const double t0 = steady_s();
    CHECK_EQ(iad_cell_cancel(cell, "NOSUCH:9,SIM1:1,NOSUCH:9"), IAD_ERR_CELL);
    const CellFailure failure = last_cell_failure();
    CHECK_EQ(failure.operation, std::string("cancel"));
    CHECK_EQ(failure.codes.size(), 1u);
    CHECK_EQ(code_of(failure, "NOSUCH:9"), static_cast<long long>(IAD_ERR_INVALID_ARGUMENT));
    mover.join();
    CHECK(steady_s() - t0 < 3.0);
    CHECK_EQ(moved.load(), static_cast<int>(IAD_ERR_CANCELLED));
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.position_mm < 139.0);
    CHECK_EQ(iad_cell_cancel(cell, nullptr), IAD_OK);  // nothing waits: nothing happens
    iad_axis_close(ax);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi_coord_timeout_of_a_request_is_refused_before_anything_is_sent_when_it_is_not_above_zero) {
    iad_cell* cell = open_cell(50.0, 20.0);
    REQUIRE(cell != nullptr);
    const std::string head = R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                                 "path": {"segments": [{"line": {"x": 70, "y": 60}}]}, "speed": 40, "accel_g": 0.3,
                                 "timeout_s": )";
    std::string buffer(64 * 1024, '\0');
    std::size_t need = 0;
    for (const char* bad : {"0", "0.0", "-1", "-0.5"}) {
        const std::string request = head + bad + "}";
        CHECK_EQ(iad_cell_coord_run_json(cell, request.c_str(), &buffer[0], buffer.size(), &need), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(iad_last_error_message(), "timeout_s"));
        CHECK(has(iad_last_error_message(), "nothing sent"));
        // the same request as a dry run: a request that cannot be run is not a plan either
        CHECK_EQ(iad_cell_coord_plan_json(cell, request.c_str(), &buffer[0], buffer.size(), &need), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(iad_last_error_message(), "nothing sent"));
        CHECK_NEAR(position(cell, "SIM1:1"), 50.0, 0.01);
        CHECK_NEAR(position(cell, "SIM2:1"), 50.0, 0.01);
    }
    // JSON has no infinity: a number past the double range is refused when the request is read
    const std::string huge = head + "1e999}";
    CHECK_EQ(iad_cell_coord_run_json(cell, huge.c_str(), &buffer[0], buffer.size(), &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_NEAR(position(cell, "SIM1:1"), 50.0, 0.01);
    // absent is the default; a positive value is the run's own
    const std::string absent = R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                                   "path": {"segments": [{"line": {"x": 70, "y": 60}}]}, "speed": 40, "accel_g": 0.3})";
    CHECK_EQ(iad_cell_coord_plan_json(cell, absent.c_str(), &buffer[0], buffer.size(), &need), IAD_OK);
    const std::string positive = head + "30}";
    CHECK_EQ(iad_cell_coord_run_json(cell, positive.c_str(), &buffer[0], buffer.size(), &need), IAD_OK);
    CHECK(has(buffer.c_str(), "\"completed\":true"));
    CHECK_NEAR(position(cell, "SIM1:1"), 70.0, 0.11);
    CHECK_NEAR(position(cell, "SIM2:1"), 60.0, 0.11);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}
