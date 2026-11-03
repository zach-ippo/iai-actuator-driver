// C ABI 1.0 (the 0.8 layout): the names (include/iai_actuator_driver.h "Names"), on the simulator only:
//  - the axis commands that took scalars take a sized options struct (servo, home, stop, wait_idle, change_speed /
//    change_accel, cancel_paused_move): its _init writes the defaults, NULL means them, the sized rules apply;
//  - the variants are folded into their base functions (teach_position's method, wait_idle's completion rule, the cell
//    group commands' options, move_all's per-axis values);
//  - the renamed functions: iad_axis_identity_json, iad_move_handle_*, iad_cell_open_axis.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

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

iad_axis* open_slider() {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim("slider", &sim, nullptr, nullptr, &ax) == IAD_OK);
    REQUIRE(iad_axis_connect(ax, 1, nullptr) == IAD_OK);
    return ax;
}

/// An options struct filled with garbage first: its _init must write every byte of the first size.
template <class T>
T garbage() {
    T value;
    std::memset(&value, 0xAB, sizeof value);
    return value;
}

}  // namespace

TEST(capi08_axis_command_options_init_writes_the_defaults) {
    iad_servo_options servo = garbage<iad_servo_options>();
    REQUIRE(iad_servo_options_init(&servo) == IAD_OK);
    CHECK_EQ(servo.struct_size, static_cast<uint32_t>(sizeof servo));
    CHECK_EQ(servo.reserved_header, 0u);
    CHECK(std::isnan(servo.timeout_s));
    CHECK_EQ(servo.wait, IAD_TRUE);
    CHECK_EQ(servo.reserved0, 0);
    CHECK_EQ(servo.reserved[3], UINT64_C(0));
    iad_home_options home = garbage<iad_home_options>();
    REQUIRE(iad_home_options_init(&home) == IAD_OK);
    CHECK_EQ(home.struct_size, static_cast<uint32_t>(sizeof home));
    CHECK(std::isnan(home.timeout_s));
    CHECK_EQ(home.wait, IAD_TRUE);
    CHECK_EQ(home.method, IAD_METHOD_DEFAULT);
    iad_stop_options stop = garbage<iad_stop_options>();
    REQUIRE(iad_stop_options_init(&stop) == IAD_OK);
    CHECK_EQ(stop.struct_size, static_cast<uint32_t>(sizeof stop));
    CHECK(std::isnan(stop.timeout_s));
    CHECK_EQ(stop.wait, IAD_FALSE);
    iad_wait_idle_options wait = garbage<iad_wait_idle_options>();
    REQUIRE(iad_wait_idle_options_init(&wait) == IAD_OK);
    CHECK_EQ(wait.struct_size, static_cast<uint32_t>(sizeof wait));
    CHECK(std::isnan(wait.target) && std::isnan(wait.band) && std::isnan(wait.timeout_s));
    CHECK_EQ(wait.completion, IAD_COMPLETION_DEFAULT);
    iad_change_options change = garbage<iad_change_options>();
    REQUIRE(iad_change_options_init(&change) == IAD_OK);
    CHECK_EQ(change.struct_size, static_cast<uint32_t>(sizeof change));
    CHECK(std::isnan(change.timeout_s));
    CHECK_EQ(change.wait, IAD_FALSE);
    iad_cancel_paused_move_options cancel = garbage<iad_cancel_paused_move_options>();
    REQUIRE(iad_cancel_paused_move_options_init(&cancel) == IAD_OK);
    CHECK_EQ(cancel.struct_size, static_cast<uint32_t>(sizeof cancel));
    CHECK(std::isnan(cancel.timeout_s));
    CHECK_EQ(iad_servo_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cancel_paused_move_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
}

TEST(capi08_axis_commands_take_their_options_null_is_the_defaults) {
    iad_axis* ax = open_slider();
    iad_status st;
    CHECK_EQ(iad_axis_servo(ax, IAD_TRUE, nullptr, &st), IAD_OK);  // waits: `out` written
    CHECK_EQ(st.servo_on, IAD_TRUE);
    CHECK_EQ(iad_axis_home(ax, nullptr, &st), IAD_OK);
    CHECK_EQ(st.homed, IAD_TRUE);
    iad_move_options mo;
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    mo.speed = 5.0;  // slow: the move still runs when its speed is changed
    mo.wait = IAD_FALSE;
    CHECK_EQ(iad_axis_move_to(ax, 60.0, &mo, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_change_speed(ax, 40.0, nullptr, nullptr), IAD_OK);  // no wait
    CHECK_EQ(iad_axis_change_accel(ax, 0.2, nullptr, nullptr), IAD_OK);
    // wait_idle: the target and the completion rule in one struct (iad_axis_wait_idle_completion before 0.8)
    iad_wait_idle_options wait;
    REQUIRE(iad_wait_idle_options_init(&wait) == IAD_OK);
    wait.target = 60.0;
    wait.completion = IAD_COMPLETION_FAST;
    CHECK_EQ(iad_axis_wait_idle(ax, &wait, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 60.0, 0.11);
    CHECK_EQ(iad_axis_wait_idle(ax, nullptr, &st), IAD_OK);
    wait.completion = 7;
    CHECK_EQ(iad_axis_wait_idle(ax, &wait, &st), IAD_ERR_INVALID_ARGUMENT);
    wait.completion = IAD_COMPLETION_SAFE;
    wait.timeout_s = -1.0;
    CHECK_EQ(iad_axis_wait_idle(ax, &wait, &st), IAD_ERR_INVALID_ARGUMENT);
    wait.timeout_s = kNaN;
    wait.target = std::numeric_limits<double>::infinity();
    CHECK_EQ(iad_axis_wait_idle(ax, &wait, &st), IAD_ERR_INVALID_ARGUMENT);
    // pause, then drop the rest of the move with the default timeout
    CHECK_EQ(iad_axis_move_to(ax, 10.0, &mo, nullptr), IAD_OK);
    iai_actuator_driver::sleep_at_least(0.05);
    CHECK_EQ(iad_axis_pause(ax), IAD_OK);
    CHECK_EQ(iad_axis_cancel_paused_move(ax, nullptr, &st), IAD_OK);
    CHECK_EQ(st.paused, IAD_FALSE);
    CHECK_EQ(iad_axis_stop(ax, nullptr, &st), IAD_OK);
    CHECK_EQ(st.moving, IAD_FALSE);
    // the sized rules: a struct below its first size, a set reserved slot
    iad_stop_options stop;
    REQUIRE(iad_stop_options_init(&stop) == IAD_OK);
    stop.wait = IAD_TRUE;
    stop.timeout_s = 2.0;
    CHECK_EQ(iad_axis_stop(ax, &stop, &st), IAD_OK);
    stop.reserved[0] = 1;
    CHECK_EQ(iad_axis_stop(ax, &stop, &st), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "iad_stop_options.reserved[0]"));
    iad_servo_options servo;
    REQUIRE(iad_servo_options_init(&servo) == IAD_OK);
    servo.struct_size = 8;
    CHECK_EQ(iad_axis_servo(ax, IAD_TRUE, &servo, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "iad_servo_options.struct_size is 8"));
    iad_home_options home;
    REQUIRE(iad_home_options_init(&home) == IAD_OK);
    home.method = 9;
    CHECK_EQ(iad_axis_home(ax, &home, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "method must be"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_teach_position_takes_its_method) {
    iad_axis* ax = open_slider();
    REQUIRE(iad_axis_servo(ax, IAD_TRUE, nullptr, nullptr) == IAD_OK);
    REQUIRE(iad_axis_home(ax, nullptr, nullptr) == IAD_OK);
    iad_nv_write_options nv;
    REQUIRE(iad_nv_write_options_init(&nv) == IAD_OK);
    iad_position_write_result out;
    CHECK_EQ(iad_axis_teach_position(ax, 0, 5, &nv, &out), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "IAD_TEACH_TABLE or IAD_TEACH_COIL"));
    CHECK_EQ(iad_axis_teach_position(ax, 0, IAD_TEACH_TABLE, &nv, &out), IAD_OK);  // no opt-in needed
    CHECK_EQ(out.row, 0);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_renamed_functions) {
    iad_axis* ax = open_slider();
    static char buf[65536];
    std::size_t need = 0;
    CHECK_EQ(iad_axis_identity_json(ax, buf, sizeof buf, &need), IAD_OK);  // was iad_axis_unit_info_json
    CHECK(has(buf, "\"system_code\""));
    REQUIRE(iad_axis_servo(ax, IAD_TRUE, nullptr, nullptr) == IAD_OK);
    REQUIRE(iad_axis_home(ax, nullptr, nullptr) == IAD_OK);
    iad_move_handle* mh = nullptr;
    CHECK_EQ(iad_axis_start_move_to(ax, 30.0, nullptr, &mh), IAD_OK);
    iad_bool done = IAD_FALSE;
    iad_status st;
    CHECK_EQ(iad_move_handle_poll(mh, 5.0, &done, &st), IAD_OK);
    CHECK_EQ(done, IAD_TRUE);
    CHECK_EQ(iad_move_handle_done(mh, &done, nullptr), IAD_OK);
    CHECK_EQ(iad_move_handle_wait(mh, kNaN, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 30.0, 0.11);
    iad_move_info info;
    CHECK_EQ(iad_move_handle_result(mh, &info), IAD_OK);
    CHECK_EQ(info.has_result, IAD_TRUE);
    CHECK_EQ(iad_move_handle_json(mh, buf, sizeof buf, &need), IAD_OK);
    CHECK(has(buf, "\"target_mm\""));
    CHECK_EQ(iad_move_handle_cancel(mh, IAD_FALSE, kNaN, nullptr), IAD_OK);
    CHECK_EQ(iad_move_handle_close(mh), IAD_OK);
    CHECK_EQ(iad_move_handle_close(mh), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // a cell's axis handle: iad_cell_open_axis (was iad_cell_axis)
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, nullptr, &cell) == IAD_OK);
    iad_axis* one = nullptr;
    CHECK_EQ(iad_cell_open_axis(cell, "SIM2:1", &one), IAD_OK);
    CHECK_EQ(iad_axis_status(one, &st), IAD_OK);
    CHECK_EQ(iad_axis_close(one), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi08_cell_group_commands_take_the_folded_options) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, nullptr, &cell) == IAD_OK);
    iad_cell_result results[2];
    std::size_t count = 0;
    CHECK_EQ(iad_cell_servo_all(cell, IAD_TRUE, nullptr, kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_home_all(cell, nullptr, kNaN, IAD_METHOD_DEFAULT, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_home_all(cell, nullptr, kNaN, 9, IAD_TRUE, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(iad_cell_status_all(cell, nullptr, -1.0, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    // move_all with per-axis values (iad_cell_move_all_values before 0.8) and without (NULL arrays)
    iad_move_options mo;
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    const double targets[2] = {20.0, 30.0};
    const double speeds[2] = {40.0, kNaN};
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", targets, 2, &mo, speeds, nullptr, nullptr, nullptr, results, 2,
                               &count),
             IAD_OK);
    CHECK_EQ(count, std::size_t(2));
    CHECK_NEAR(results[1].status.position_mm, 30.0, 0.11);
    const double back[2] = {10.0, 10.0};
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", back, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 2,
                               &count),
             IAD_OK);
    // move_each with stop_others_on_error; a refusal names move_each (the bindings' move_all renames it)
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);
    for (auto& m : moves) m.struct_size = static_cast<uint32_t>(sizeof m);
    moves[0].position = 15.0;
    std::strcpy(moves[0].axis, "SIM1:1");
    moves[1].position = 500.0;  // outside the stroke: the whole move is refused
    std::strcpy(moves[1].axis, "SIM2:1");
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_FALSE, results, 2, &count), IAD_ERR_CELL);
    CHECK(has(last_json(), "\"operation\":\"move_each (check; nothing started)\""));
    moves[1].position = 15.0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_FALSE, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_EQ(count, std::size_t(0));  // started without waiting
    CHECK_EQ(iad_cell_wait_all(cell, nullptr, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_NEAR(results[0].status.position_mm, 15.0, 0.11);
    CHECK_EQ(iad_cell_wait_all(cell, nullptr, 0.0, IAD_TRUE, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_FALSE, results, 2, &count), IAD_OK);
    CHECK_EQ(count, std::size_t(2));
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}
