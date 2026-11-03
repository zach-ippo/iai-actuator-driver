// C ABI 0.5 axis state (src/c_api_axis.cpp, src/axis_state.cpp) on the simulator: the raw words of a returned status,
// the state without I/O, the reads of their own, the row write without the checksum, begin_stop / release_held, the
// settable axis limits, the non-volatile guard shared per unit and the test timings. The Python facade
// (bindings/python/iai_actuator_driver/_native/ec.py) runs the Python edition's tests over these.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver_testing.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/status.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

iad_axis* open_sim(const char* profile = "slider") {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_open_options open;
    iad_open_options_init(&open);
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim(profile, &sim, &open, nullptr, &ax) == IAD_OK);
    REQUIRE(ax != nullptr);
    return ax;
}

using JsonCall = iad_result (*)(iad_axis*, char*, size_t, size_t*);

Json json_of(iad_axis* ax, JsonCall call) {
    std::string buffer(64 * 1024, '\0');
    size_t required = 0;
    REQUIRE(call(ax, &buffer[0], buffer.size(), &required) == IAD_OK);
    return Json::parse(buffer.c_str());
}

std::vector<uint16_t> words_of(iad_axis* ax, const iad_status* like) {
    std::vector<uint16_t> words(22);
    size_t count = 0;
    REQUIRE(iad_axis_last_status_words(ax, like, words.data(), words.size(), &count) == IAD_OK);
    words.resize(count);
    return words;
}

}  // namespace

TEST(capi_axis_state_last_status_words_match_the_returned_status) {
    iad_axis* ax = open_sim();
    std::vector<uint16_t> none(22);
    size_t count = 7;
    CHECK_EQ(iad_axis_last_status_words(ax, nullptr, none.data(), none.size(), &count), IAD_ERR_NOT_READY);
    CHECK_EQ(count, 0u);
    iad_status st;
    REQUIRE(iad_axis_status(ax, &st) == IAD_OK);
    const std::vector<uint16_t> words = words_of(ax, &st);
    CHECK_EQ(words.size(), 22u);
    CHECK_EQ(words[5], st.raw_dss1);
    CHECK_EQ(words[7], st.raw_dsse);
    const Status again = Status::from_registers(words);
    CHECK_NEAR(again.position_mm, st.position_mm, 1e-9);
    // a newer lean read does not hide the 22-word sample the caller holds
    iad_status lean;
    REQUIRE(iad_axis_read_lean_status(ax, IAD_INT_UNKNOWN, &lean) == IAD_OK);
    CHECK_EQ(words_of(ax, nullptr).size(), 8u);
    CHECK_EQ(words_of(ax, &st).size(), 22u);
    std::vector<uint16_t> small(4);
    CHECK_EQ(iad_axis_last_status_words(ax, &st, small.data(), small.size(), &count), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(count, 22u);
    iad_axis_close(ax);
}

TEST(capi_axis_state_without_io_and_the_reads) {
    iad_axis* ax = open_sim("gripper");
    Json state = json_of(ax, iad_axis_state_json);
    CHECK_EQ(state.find("soft_limits_note")->as_string(), std::string("not read yet"));
    CHECK(state.find("axis_limits")->is_null());
    CHECK_NEAR(state.find("speed_cap_mm_s")->as_number(), 100.0, 1e-9);  // the fallback while nothing is known
    CHECK_EQ(state.find("status_words")->as_int(), 22);
    const Json limits = json_of(ax, iad_axis_read_axis_limits_json);
    CHECK_NEAR(limits.find("max_speed_mm_s")->as_number(), 53.0, 1e-9);
    state = json_of(ax, iad_axis_state_json);
    CHECK_NEAR(state.find("speed_cap_mm_s")->as_number(), 53.0, 1e-9);
    CHECK(state.find("ec_max_speed_note")->as_string().find("MAXV item 7.4 @0x") == 0);
    const Json soft = json_of(ax, iad_axis_read_soft_limits_json);
    CHECK_EQ(soft.find("soft_limits_mm")->size(), 2u);
    const Json kind = json_of(ax, iad_axis_kind_json);
    CHECK_EQ(kind.find("units")->as_string(), std::string("mm"));
    CHECK(kind.find("is_ec")->as_bool());
    CHECK(json_of(ax, iad_axis_categories_json).size() > 5u);
    CHECK(json_of(ax, iad_axis_read_support_words_json).find("SUPT") != nullptr);
    const Json users = json_of(ax, iad_axis_user_parameters_json);
    REQUIRE(users.size() > 0u);
    CHECK_EQ(users.as_array().front().find("no")->as_int(), 1);
    CHECK(users.as_array().front().find("located")->as_bool());
    double speed = 0.0;
    REQUIRE(iad_axis_read_speed_limit(ax, &speed) == IAD_OK);
    CHECK_NEAR(speed, 53.0, 1e-9);
    std::string buffer(4096, '\0');
    size_t required = 0;
    REQUIRE(iad_axis_servo_on_timeout_json(ax, nullptr, 0, &buffer[0], buffer.size(), &required) == IAD_OK);
    CHECK_EQ(Json::parse(buffer.c_str()).find("reason")->as_string(), std::string("default"));
    REQUIRE(iad_constants_json(&buffer[0], buffer.size(), &required) == IAD_OK);
    CHECK_EQ(Json::parse(buffer.c_str()).find("NV_WARN_PER_HOUR")->as_int(), 60);
    iad_axis_close(ax);
}

TEST(capi_axis_state_settable_limits_and_begin_stop) {
    iad_axis* ax = open_sim();
    REQUIRE(iad_axis_connect(ax, IAD_TRUE, nullptr) == IAD_OK);
    REQUIRE(iad_axis_servo(ax, IAD_TRUE, ptr_of(servo_opts(IAD_TRUE, kNaN)), nullptr) == IAD_OK);
    REQUIRE(iad_axis_home(ax, ptr_of(home_opts(IAD_TRUE, kNaN, IAD_METHOD_DEFAULT)), nullptr) == IAD_OK);
    // MVPT trapezoid only: the filter pattern is refused, nothing sent
    REQUIRE(iad_axis_set_axis_limits_json(ax, R"({"max_speed_mm_s": 700, "max_accel_g": 1, "max_decel_g": 1,
                                                 "move_patterns": 1})") == IAD_OK);
    iad_move_options o;
    iad_move_options_init(&o);
    o.accel_pattern = IAD_ACCEL_FILTER;
    CHECK_EQ(iad_axis_move_to(ax, 50.0, &o, nullptr), IAD_ERR_SOFT_LIMIT);
    CHECK(std::string(iad_last_error_message()).find("MVPT") != std::string::npos);
    CHECK_EQ(iad_axis_set_axis_limits_json(ax, "[1]"), IAD_ERR_INVALID_ARGUMENT);
    REQUIRE(iad_axis_begin_stop(ax) == IAD_OK);
    CHECK(json_of(ax, iad_axis_state_json).find("stop_held")->as_bool());
    REQUIRE(iad_axis_release_held(ax) == IAD_OK);
    CHECK(!json_of(ax, iad_axis_state_json).find("stop_held")->as_bool());
    iad_axis_close(ax);
}

TEST(capi_axis_state_row_without_checksum_is_verified) {
    iad_axis* ax = open_sim();
    REQUIRE(iad_axis_connect(ax, IAD_TRUE, nullptr) == IAD_OK);
    iad_position_entry row;
    REQUIRE(iad_axis_read_position_table(ax, 2, &row) == IAD_OK);
    row.position_mm = 61.0;
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    nv.allow_nonvolatile = IAD_TRUE;
    // the 15 data words only: the simulated unit updates word +0xF itself, as the EC does
    iad_position_write_result out;
    REQUIRE(iad_axis_write_position_row(ax, 2, &row, &nv, IAD_FALSE, &out) == IAD_OK);
    CHECK(!out.skipped);
    CHECK_NEAR(out.position_mm, 61.0, 1e-9);
    REQUIRE(iad_axis_write_position_row(ax, 2, &row, &nv, IAD_TRUE, &out) == IAD_OK);
    CHECK(out.skipped);  // the row now holds the 16 words: nothing written
    iad_axis_close(ax);
}

TEST(capi_axis_state_nv_guard_shared_per_unit_key) {
    iad_axis* a = open_sim();
    iad_axis* b = open_sim();
    REQUIRE(iad_axis_set_nv_unit(a, "test:unit:1", "slave 1") == IAD_OK);
    REQUIRE(iad_axis_set_nv_unit(b, "test:unit:1", "slave 1") == IAD_OK);
    iad_position_entry row;
    REQUIRE(iad_axis_read_position_table(a, 2, &row) == IAD_OK);
    row.position_mm = 62.0;
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    nv.allow_nonvolatile = IAD_TRUE;
    REQUIRE(iad_axis_write_position_table(a, 2, &row, &nv, nullptr) == IAD_OK);
    int64_t total = 0;
    int32_t hour = 0;
    REQUIRE(iad_axis_nonvolatile_writes(b, &total, &hour, nullptr) == IAD_OK);
    CHECK_EQ(total, 1);  // counted for the unit, seen by the other axis object
    REQUIRE(iad_nv_forget("test:unit:") == IAD_OK);
    iad_axis* c = open_sim();
    REQUIRE(iad_axis_set_nv_unit(c, "test:unit:1", nullptr) == IAD_OK);
    REQUIRE(iad_axis_nonvolatile_writes(c, &total, &hour, nullptr) == IAD_OK);
    CHECK_EQ(total, 0);  // a new process starts without it
    iad_axis_close(a);
    iad_axis_close(b);
    iad_axis_close(c);
}

TEST(capi_axis_state_test_timings) {
    CHECK_EQ(iad_testing_set_timing("home_start_s", 0.2), IAD_OK);
    CHECK_EQ(iad_testing_set_timing("nope", 1.0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_testing_set_timing("parw_ready_s", -1.0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_testing_set_timing(nullptr, kNaN), IAD_OK);  // every default back
}
