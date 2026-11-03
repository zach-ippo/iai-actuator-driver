// Fixes of the round-5 review (the "polish" commit), on the simulator only (no COM port is opened):
//  - the caller's own error (IAD_SIM_ABORT / IAD_CALLBACK_ABORT) is not swallowed by the best-effort steps of a
//    recovery, a unit-info read or a backup: the call ends with it, nothing more is sent;
//  - Json::parse refuses a number it cannot hold (it was narrowed to LLONG_MAX, or turned into null), so the range
//    checks of the session and the identity calls cannot be bypassed;
//  - the non-volatile write guard's test hook counts at a time of the caller's choosing without erasing the writes.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

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

/// A simulator with an axis on it, opened the way a binding does (bus over the simulator's transport).
struct SimAxis {
    iad_sim* sim = nullptr;
    iad_bus* bus = nullptr;
    iad_axis* ax = nullptr;

    explicit SimAxis(const char* options_json) {
        REQUIRE(iad_sim_open(options_json, &sim) == IAD_OK);
        iad_transport* t = nullptr;
        REQUIRE(iad_sim_transport(sim, &t) == IAD_OK);
        iad_bus_options bo;
        iad_bus_options_init(&bo);
        bo.min_gap_s = 0.0;
        REQUIRE(iad_bus_open(t, &bo, &bus) == IAD_OK);
        CHECK_EQ(iad_transport_release(t), IAD_OK);
        iad_axis_options ao;
        iad_axis_options_init(&ao);
        ao.poll_interval_s = 0.005;
        REQUIRE(iad_axis_open_bus(bus, 1, &ao, &ax) == IAD_OK);
        CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, nullptr), IAD_OK);
    }
    ~SimAxis() {
        CHECK_EQ(iad_axis_close(ax), IAD_OK);
        CHECK_EQ(iad_bus_close(bus), IAD_OK);
        CHECK_EQ(iad_sim_close(sim), IAD_OK);
    }
};

/// A request hook that answers IAD_SIM_ABORT to one chosen request while armed: the `abort_at`-th request seen since
/// arming (any function), or the first FC05 write of the coil `coil` (>= 0).
struct AbortAt {
    std::atomic<bool> armed{false};
    std::atomic<int> seen{0};
    std::atomic<int> aborted{0};
    int abort_at = 0;
    int coil = -1;
    static int32_t IAD_CALL fn(const uint8_t* request, size_t size, uint8_t* reply, size_t, size_t* length, void* user) {
        auto* h = static_cast<AbortAt*>(user);
        if (!h->armed.load() || size < 4) return IAD_SIM_PASS;
        const int index = ++h->seen;
        const bool hit = h->coil >= 0 ? (request[1] == 0x05 && ((request[2] << 8) | request[3]) == h->coil)
                                      : index == h->abort_at;
        if (!hit) return IAD_SIM_PASS;
        ++h->aborted;
        const char text[] = "assert in the test hook";
        std::memcpy(reply, text, sizeof text - 1);
        *length = sizeof text - 1;
        return IAD_SIM_ABORT;
    }
};

}  // namespace

TEST(polish_abort_in_a_best_effort_recovery_step_ends_the_recovery) {
    SimAxis s(R"({"profile": "gripper", "slave": 1, "time_scale": 50})");
    CHECK_EQ(iad_axis_track(s.ax, nullptr, "G", nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_sim_power_cycle(s.sim, IAD_UNKNOWN), IAD_OK);  // PC control (PMSL) is lost: the recovery takes it again
    AbortAt hook;
    hook.coil = 0x0427;  // PMSL
    CHECK_EQ(iad_sim_set_request_hook(s.sim, &AbortAt::fn, &hook), IAD_OK);
    hook.armed = true;
    iad_recovery_policy policy;
    iad_recovery_policy_init(&policy);
    policy.poll_s = 0.02;
    iad_recovery_summary rs;
    // the best-effort step "take PC control again" used to note the error and go on: IAD_OK
    CHECK_EQ(iad_axis_recover(s.ax, 5.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_ERR_DRIVER);
    CHECK(has(iad_last_error_message(), "assert in the test hook"));
    CHECK_EQ(hook.aborted.load(), 1);
    const int sent_at_abort = hook.seen.load();
    // nothing more goes out for that call
    sleep_at_least(0.1);
    CHECK_EQ(hook.seen.load(), sent_at_abort);
    // the hook is the only reason: without it the same recovery takes control again
    hook.armed = false;
    CHECK_EQ(iad_axis_recover(s.ax, 5.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_OK);
    CHECK(rs.control_restored == IAD_TRUE || rs.control_kept == IAD_TRUE);
}

TEST(polish_abort_in_a_unit_info_read_or_backup_ends_the_call) {
    SimAxis s(R"({"profile": "gripper", "slave": 1, "time_scale": 50})");
    AbortAt hook;
    CHECK_EQ(iad_sim_set_request_hook(s.sim, &AbortAt::fn, &hook), IAD_OK);
    // a read well into the call, not the first: the sections after it must not be read
    for (const char* sections : {"parameters", "monitor,alarms", "positions", "maintenance", static_cast<const char*>(nullptr)}) {
        hook.seen = 0;
        hook.aborted = 0;
        hook.abort_at = 4;
        hook.armed = true;
        size_t need = 0;
        std::vector<char> buffer(1 << 20);
        const iad_result r = iad_axis_read_unit_info_json(s.ax, sections, buffer.data(), buffer.size(), &need);
        hook.armed = false;
        CHECK_EQ(r, IAD_ERR_DRIVER);
        CHECK(has(iad_last_error_message(), "assert in the test hook"));
        CHECK_EQ(hook.aborted.load(), 1);
        CHECK_EQ(hook.seen.load(), 4);  // nothing was asked after the abort
    }
    // the backup ends the same way
    hook.seen = 0;
    hook.aborted = 0;
    hook.abort_at = 6;
    hook.armed = true;
    size_t need = 0;
    std::vector<char> buffer(1 << 20);
    CHECK_EQ(iad_axis_backup_json(s.ax, "all", nullptr, buffer.data(), buffer.size(), &need), IAD_ERR_DRIVER);
    hook.armed = false;
    CHECK_EQ(hook.aborted.load(), 1);
    CHECK_EQ(hook.seen.load(), 6);
    // and without the hook the read gives its text
    CHECK(has(read_text([&](char* b, size_t n, size_t* r) { return iad_axis_read_unit_info_json(s.ax, "identity", b, n, r); }),
              "identity"));
}

TEST(polish_json_parse_refuses_a_number_it_cannot_hold) {
    for (const char* text : {"99999999999999999999", "-99999999999999999999", "9223372036854775808", "1e999", "-1e999",
                             "1e309", "[1, 1e999]", "{\"a\": 99999999999999999999}"}) {
        bool refused = false;
        try {
            Json::parse(text);
        } catch (const std::invalid_argument&) {
            refused = true;
        }
        CHECK(refused);
    }
    // the edges that do fit
    CHECK_EQ(Json::parse("9223372036854775807").as_int(), 9223372036854775807LL);
    CHECK_EQ(Json::parse("-9223372036854775808").as_int(), -9223372036854775807LL - 1);
    CHECK(Json::parse("1e308").type() == Json::Type::Number);
    CHECK(Json::parse("1e-999").type() == Json::Type::Number);  // underflow to zero is not an overflow
    CHECK(Json::parse("3e9").type() == Json::Type::Number);
}

TEST(polish_an_overflowing_number_cannot_bypass_the_range_checks_of_the_c_api) {
    SimAxis s(R"({"profile": "gripper", "slave": 1, "time_scale": 50})");
    CHECK_EQ(iad_axis_track(s.ax, nullptr, "G", nullptr, nullptr), IAD_OK);
    auto session = [&] {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_axis_session_json(s.ax, b, n, r); });
    };
    CHECK_EQ(iad_axis_session_update(s.ax, "{\"baudrate\":115200}"), IAD_OK);
    CHECK(has(session(), "\"baudrate\":115200"));
    // 1e999 became null (baud rate cleared), 99999999999999999999 became LLONG_MAX
    CHECK_EQ(iad_axis_session_update(s.ax, "{\"baudrate\":1e999}"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_update(s.ax, "{\"uptime_ms\":99999999999999999999}"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(session(), "\"baudrate\":115200"));  // nothing written
    // the 64-bit time of a new alarm record
    iad_status st;
    REQUIRE(iad_axis_status(s.ax, &st) == IAD_OK);
    uint16_t words[22];
    size_t count = 0;
    REQUIRE(iad_axis_last_status_words(s.ax, &st, words, 22, &count) == IAD_OK);
    const std::string signs = read_text([&](char* b, size_t n, size_t* r) {
        return iad_axis_restart_signs(s.ax, words, count, "{\"new_records\":[[4095,0,65535,99999999999999999999]]}", b, n, r);
    });
    CHECK_EQ(signs, std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    // a vid that is not a number is "no vid" no more
    CHECK_EQ(iad_testing_set_port_list("[{\"port\":\"COM77\",\"vid\":1e999}]"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_testing_set_port_list(nullptr), IAD_OK);
}

TEST(polish_nv_hook_query_for_a_later_time_keeps_the_real_counts) {
    SimAxis s(R"({"profile": "gripper", "slave": 1, "time_scale": 20})");
    CHECK_EQ(iad_axis_set_nv_unit(s.ax, "test:polish:1", "SIM"), IAD_OK);
    double t0 = 0.0;
    CHECK_EQ(iad_clock_now(&t0), IAD_OK);
    for (int i = 0; i < 62; ++i) CHECK_EQ(iad_testing_axis_nv_record(s.ax, t0, nullptr, nullptr), IAD_OK);
    int32_t count = -1;
    // a query two hours on: nothing counts then, but the writes are still there
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 7200.0, &count), IAD_OK);
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, NAN, &count), IAD_OK);
    CHECK_EQ(count, 62);
    int64_t total = 0;
    int32_t last_hour = 0;
    CHECK_EQ(iad_axis_nonvolatile_writes(s.ax, &total, &last_hour, nullptr), IAD_OK);
    CHECK_EQ(total, 62);
    CHECK_EQ(last_hour, 62);
    // the window's edges, a second either side of the hour: not the exact edge, where `now - stamp` is a float
    // subtraction that can come out one ulp over 3600 (about 1 time in 6 for a clock reading of 500..10000 s)
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 3599.0, &count), IAD_OK);
    CHECK_EQ(count, 62);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 3601.0, &count), IAD_OK);
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0, &count), IAD_OK);
    CHECK_EQ(count, 62);
    // a record at a time before the newest one is kept in order: one more than an hour older is forgotten at once (an
    // unsorted list kept it behind the newer one and counted it)
    CHECK_EQ(iad_testing_axis_nv_record(s.ax, t0 + 4000.0, nullptr, nullptr), IAD_OK);  // the 62 at t0 are out of its window
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 4000.0, &count), IAD_OK);
    CHECK_EQ(count, 1);
    CHECK_EQ(iad_testing_axis_nv_record(s.ax, t0, nullptr, nullptr), IAD_OK);  // older than an hour before the newest
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 4000.0, &count), IAD_OK);
    CHECK_EQ(count, 1);
    CHECK_EQ(iad_testing_axis_nv_record(s.ax, t0 + 3900.0, nullptr, nullptr), IAD_OK);  // inside it: kept, in order
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 4000.0, &count), IAD_OK);
    CHECK_EQ(count, 2);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 7550.0, &count), IAD_OK);  // t0 + 3900 is 3650 s old by then
    CHECK_EQ(count, 1);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, t0 + 7601.0, &count), IAD_OK);
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_nv_forget("test:polish:"), IAD_OK);
}

TEST(polish_nv_window_edge_is_inclusive_for_exact_times) {
    // whole seconds are exact in a double, so the edge is the same for any clock: a write exactly an hour old still counts
    SimAxis s(R"({"profile": "gripper", "slave": 1, "time_scale": 20})");
    CHECK_EQ(iad_axis_set_nv_unit(s.ax, "test:polish:edge", "SIM"), IAD_OK);
    CHECK_EQ(iad_testing_axis_nv_record(s.ax, 1000.0, nullptr, nullptr), IAD_OK);
    int32_t count = -1;
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, 4600.0, &count), IAD_OK);
    CHECK_EQ(count, 1);
    CHECK_EQ(iad_testing_axis_nv_last_hour(s.ax, 4601.0, &count), IAD_OK);
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_nv_forget("test:polish:"), IAD_OK);
}
