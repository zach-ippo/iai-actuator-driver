// C ABI 1.0 (the 0.8 layout): the rule of the defaults (docs/NATIVE_ABI.md, "Defaults"), on the simulator only:
//  - IAD_UNKNOWN is 2; a tribool input is 0 false, 2 unknown / keep, any other value true (a VB True, -1, too);
//  - 0 is the default of every enumeration (IAD_METHOD_* renumbered: DEFAULT 0, COIL 1, IAI 2, IAI2 3, REGISTER 4);
//  - NaN is the default of a double and a 0 a value (checked), except in the two structs without an _init, where NaN or
//    0 is the default of a speed, acceleration, band or timeout, so a zeroed iad_axis_move or iad_sequence_point with its
//    struct_size set is all defaults; a push direction of 0 is +1.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "iai_actuator_driver.h"
#include "harness.hpp"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

iad_axis* open_sim(const char* profile) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim(profile, &sim, nullptr, nullptr, &ax) == IAD_OK);
    REQUIRE(iad_axis_connect(ax, 1, nullptr) == IAD_OK);
    return ax;
}

}  // namespace

TEST(capi08_defaults_constants_and_the_tribool_rule) {
    CHECK_EQ(IAD_FALSE, 0);
    CHECK_EQ(IAD_TRUE, 1);
    CHECK_EQ(IAD_UNKNOWN, 2);
    CHECK_EQ(IAD_INT_UNKNOWN, INT32_MIN);
    iad_axis* ax = open_sim("slider");
    iad_status st;
    // an input: 0 false, 2 keep, any other value true (1, 5, -1 = a Visual Basic True)
    const int32_t truths[] = {IAD_TRUE, 5, -1, INT32_MIN, INT32_MAX};
    for (const int32_t value : truths) {
        CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, value, IAD_TRUE, &st), IAD_OK);
        CHECK_EQ(st.safety_speed, IAD_TRUE);
        CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_FALSE, IAD_TRUE, &st), IAD_OK);
        CHECK_EQ(st.safety_speed, IAD_FALSE);
    }
    CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_TRUE, IAD_TRUE, &st), IAD_OK);
    CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_UNKNOWN, IAD_TRUE, &st), IAD_OK);
    CHECK_EQ(st.safety_speed, IAD_TRUE);  // unknown keeps what the unit has
    CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_FALSE, IAD_TRUE, &st), IAD_OK);
    CHECK_EQ(iad_axis_set_action_mode(ax, IAD_TRUE, IAD_UNKNOWN, IAD_TRUE, &st), IAD_OK);
    CHECK_EQ(st.safety_speed, IAD_FALSE);
    // an output: 0, 1 or 2
    iad_tribool present = 77;
    CHECK_EQ(iad_axis_port_present(ax, &present), IAD_OK);
    CHECK(present == IAD_TRUE || present == IAD_FALSE || present == IAD_UNKNOWN);
    CHECK_EQ(iad_axis_port_present(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK(st.emg_input == IAD_UNKNOWN || st.emg_input == IAD_TRUE || st.emg_input == IAD_FALSE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_defaults_method_zero_is_the_default) {
    CHECK_EQ(IAD_METHOD_DEFAULT, 0);
    CHECK_EQ(IAD_METHOD_COIL, 1);
    CHECK_EQ(IAD_METHOD_IAI, 2);
    CHECK_EQ(IAD_METHOD_IAI2, 3);
    CHECK_EQ(IAD_METHOD_REGISTER, 4);
    iad_axis* ax = open_sim("slider");
    // the options: a default method is COIL, and what is read back is concrete
    iad_axis_options ao;
    REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
    CHECK_EQ(ao.method, IAD_METHOD_COIL);
    ao.method = IAD_METHOD_DEFAULT;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    REQUIRE(iad_axis_get_options(ax, &ao) == IAD_OK);
    CHECK_EQ(ao.method, IAD_METHOD_COIL);
    ao.method = -1;  // the 0.7 value of IAD_METHOD_DEFAULT is not a method now
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_ERR_INVALID_ARGUMENT);
    ao.method = 5;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_ERR_INVALID_ARGUMENT);
    // a strategy is always a concrete method
    int32_t method = -99;
    CHECK_EQ(iad_axis_strategy(ax, &method), IAD_OK);
    CHECK_EQ(method, IAD_METHOD_COIL);
    CHECK_EQ(iad_axis_set_strategy(ax, 0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_strategy(ax, IAD_METHOD_IAI), IAD_OK);
    CHECK_EQ(iad_axis_strategy(ax, &method), IAD_OK);
    CHECK_EQ(method, IAD_METHOD_IAI);
    CHECK_EQ(iad_axis_set_strategy(ax, IAD_METHOD_COIL), IAD_OK);
    // a call's method: 0 the axis option, -1 and 5 refused before anything is sent
    iad_status st;
    CHECK_EQ(iad_axis_reset_alarm(ax, -1, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_reset_alarm(ax, 5, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_reset_alarm(ax, IAD_METHOD_DEFAULT, &st), IAD_OK);
    // the other enumerations the inputs take have 0 as their default too
    iad_move_options mo;
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    CHECK_EQ(mo.accel_pattern, IAD_ACCEL_DEFAULT);
    CHECK_EQ(mo.completion, IAD_COMPLETION_DEFAULT);
    iad_open_options oo;
    REQUIRE(iad_open_options_init(&oo) == IAD_OK);
    CHECK_EQ(oo.line, IAD_LINE_AUTO);
    iad_baud_options bo;
    REQUIRE(iad_baud_options_init(&bo) == IAD_OK);
    CHECK_EQ(bo.method, IAD_BAUD_DIRECT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_defaults_zero_is_a_value_in_options_and_the_default_without_an_init) {
    iad_axis* ax = open_sim("slider");
    iad_status st;
    // an options struct is filled by its _init: a zero in it is a value, checked like any other
    iad_move_options mo;
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    CHECK(std::isnan(mo.timeout_s));
    mo.speed = 0.0;
    CHECK_EQ(iad_axis_move_to(ax, 20.0, &mo, &st), IAD_ERR_SOFT_LIMIT);
    mo.speed = kNaN;  // NaN is the default
    CHECK_EQ(iad_axis_move_to(ax, 20.0, &mo, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 20.0, 0.11);
    iad_position_move_options pm;
    REQUIRE(iad_position_move_options_init(&pm) == IAD_OK);
    pm.speed_pct = 0.0;
    CHECK_EQ(iad_axis_move_to_position(ax, 0, &pm, &st), IAD_ERR_SOFT_LIMIT);
    // a push: direction 0 is +1 (the default), another direction is refused
    iad_push_options push;
    REQUIRE(iad_push_options_init(&push) == IAD_OK);
    push.direction = 0;
    iad_push_result pr;
    CHECK_EQ(iad_axis_push(ax, 80.0, 10.0, 30.0, &push, &pr), IAD_OK);
    CHECK_EQ(pr.outcome, IAD_PUSH_MISSED);
    push.direction = -2;
    CHECK_EQ(iad_axis_push(ax, 80.0, 10.0, 30.0, &push, &pr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);

    // iad_sequence_point has no _init: zeroed with its struct_size set, a point is all defaults (NaN is the same)
    ax = open_sim("slider");
    iad_sequence_point pts[3];
    std::memset(pts, 0, sizeof pts);
    for (auto& p : pts) p.struct_size = sizeof p;
    pts[0].position = 30.0;
    pts[1].position = 50.0;
    pts[2].position = 70.0;
    pts[2].speed = pts[2].accel_g = pts[2].band = kNaN;
    iad_sequence_options so;
    REQUIRE(iad_sequence_options_init(&so) == IAD_OK);
    iad_status statuses[3];
    size_t count = 0;
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, statuses, 3, &count), IAD_OK);
    CHECK_EQ(count, size_t(3));
    CHECK_NEAR(statuses[2].position_mm, 70.0, 0.11);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);

    // iad_axis_move has none either: a zeroed one (struct_size, position and axis set) moves at the defaults
    iad_cell* cell = nullptr;
    iad_sim_options sim;
    REQUIRE(iad_sim_options_init(&sim) == IAD_OK);
    sim.time_scale = 20.0;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, nullptr, &cell) == IAD_OK);
    size_t axes = 0;
    REQUIRE(iad_cell_axis_count(cell, &axes) == IAD_OK);
    REQUIRE(axes == 2);
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);
    for (size_t i = 0; i < 2; ++i) {
        moves[i].struct_size = sizeof moves[i];
        moves[i].position = 20.0 + 10.0 * static_cast<double>(i);
        size_t need = 0;
        REQUIRE(iad_cell_axis_name(cell, i, moves[i].axis, sizeof moves[i].axis, &need) == IAD_OK);
    }
    iad_cell_result results[2];
    size_t result_count = 0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_TRUE, results, 2, &result_count), IAD_OK);
    CHECK_EQ(result_count, size_t(2));
    CHECK_NEAR(results[1].status.position_mm, 30.0, 0.11);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}
