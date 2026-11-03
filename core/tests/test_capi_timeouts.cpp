// C ABI: the timeout of a command that starts motion or a servo change is NaN (the default) or a finite number of
// seconds above 0. 0, a negative value (-1 is "wait forever" in VB and LabVIEW) or an infinity is
// IAD_ERR_INVALID_ARGUMENT before anything is sent, in every options struct (move, stored row, servo, home, change,
// push, sequence) and in the per-axis moves of a cell; before 1.0 the command went out and was then cut short (a move
// decel-stopped partway, a home return left the axis un-homed). In iad_axis_move a 0 stays the default. Simulator only.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "capi_options.hpp"
#include "harness.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver_testing.h"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
const double kBad[] = {-1.0, 0.0, kInf};

template <class F>
std::string text_of(F call) {
    size_t required = 0;
    call(nullptr, 0, &required);
    std::string buffer(required + 1, '\0');
    REQUIRE(call(&buffer[0], buffer.size(), &required) == IAD_OK);
    return std::string(buffer.c_str());
}

/// The frames that command something (FC05, FC06, FC0F, FC10, 0x45) the unit got since the last clear_requests.
int commands_sent(iad_sim* sim) {
    const Json frames = Json::parse(text_of([&](char* b, size_t s, size_t* r) { return iad_sim_requests_json(sim, b, s, r); }));
    int n = 0;
    for (const Json& request : frames.as_array()) {
        const Json* frame = request.find("frame");
        const std::string f = frame ? frame->as_string() : std::string();  // "01 10 99 00 ..."
        const std::string fc = f.size() >= 5 ? f.substr(3, 2) : std::string();
        if (fc == "05" || fc == "06" || fc == "0F" || fc == "0f" || fc == "10" || fc == "45") ++n;
    }
    return n;
}

void clear_requests(iad_sim* sim) {
    char buffer[256];
    size_t required = 0;
    REQUIRE(iad_sim_control_json(sim, R"({"command": "clear_requests"})", buffer, sizeof buffer, &required) == IAD_OK);
}

/// The last error's kind and message.
Json last_error() {
    return Json::parse(text_of([](char* b, size_t s, size_t* r) { return iad_last_error_json(b, s, r); }));
}

bool refused_before_sending() {
    const Json e = last_error();
    const Json* kind = e.find("kind");
    const Json* message = e.find("message");
    return kind && message && kind->as_string() == "std::invalid_argument" &&
           message->as_string().find("nothing sent") != std::string::npos;
}

/// One simulated slider on its own bus, connected (homed, servo on, PC control: the simulator starts ready).
struct SimAxis {
    iad_sim* sim = nullptr;
    iad_bus* bus = nullptr;
    iad_axis* ax = nullptr;

    SimAxis() {
        REQUIRE(iad_sim_open(R"({"time_scale": 20})", &sim) == IAD_OK);
        iad_transport* t = nullptr;
        REQUIRE(iad_sim_transport(sim, &t) == IAD_OK);
        REQUIRE(iad_bus_open(t, nullptr, &bus) == IAD_OK);
        iad_transport_release(t);
        iad_axis_options ao;
        REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
        ao.poll_interval_s = 0.005;
        REQUIRE(iad_axis_open_bus(bus, 1, &ao, &ax) == IAD_OK);
        REQUIRE(iad_axis_connect(ax, IAD_TRUE, nullptr) == IAD_OK);
    }
    ~SimAxis() {
        iad_axis_close(ax);
        iad_bus_close(bus);
        iad_sim_close(sim);
    }
    iad_status status() const {
        iad_status st;
        REQUIRE(iad_axis_status(ax, &st) == IAD_OK);
        return st;
    }
};

iad_move_options move_opts(double timeout_s, iad_bool wait = IAD_TRUE) {
    iad_move_options o;
    iad_move_options_init(&o);
    o.timeout_s = timeout_s;
    o.wait = wait;
    return o;
}

}  // namespace

TEST(capi_timeouts_move_options_refused_before_anything_is_sent) {
    SimAxis s;
    iad_status st;
    CHECK_EQ(iad_axis_move_to(s.ax, 20.0, ptr_of(move_opts(kNaN)), &st), IAD_OK);  // NaN: the default
    clear_requests(s.sim);
    for (const double bad : kBad) {
        CHECK_EQ(iad_axis_move_to(s.ax, 80.0, ptr_of(move_opts(bad)), &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK(refused_before_sending());
        CHECK_EQ(iad_axis_move_to(s.ax, 80.0, ptr_of(move_opts(bad, IAD_FALSE)), &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_axis_move_by(s.ax, 60.0, ptr_of(move_opts(bad)), &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_axis_jog(s.ax, 60.0, ptr_of(move_opts(bad)), &st), IAD_ERR_INVALID_ARGUMENT);
        iad_move_handle* h = nullptr;
        CHECK_EQ(iad_axis_start_move_to(s.ax, 80.0, ptr_of(move_opts(bad)), &h), IAD_ERR_INVALID_ARGUMENT);
        CHECK(h == nullptr);
        CHECK_EQ(iad_axis_start_move_by(s.ax, 60.0, ptr_of(move_opts(bad)), &h), IAD_ERR_INVALID_ARGUMENT);
        CHECK(h == nullptr);
    }
    CHECK_EQ(commands_sent(s.sim), 0);
    st = s.status();
    CHECK(!st.moving);
    CHECK_NEAR(st.position_mm, 20.0, 0.11);
    CHECK_EQ(iad_axis_move_to(s.ax, 80.0, ptr_of(move_opts(30.0)), &st), IAD_OK);  // a finite timeout above 0
    CHECK_NEAR(st.position_mm, 80.0, 0.11);
}

TEST(capi_timeouts_servo_and_home_options_refused_before_anything_is_sent) {
    SimAxis s;
    iad_status st = s.status();
    REQUIRE(st.homed && st.servo_on);
    CHECK_EQ(iad_axis_move_to(s.ax, 60.0, ptr_of(move_opts(kNaN)), &st), IAD_OK);  // off home: a home return would show
    clear_requests(s.sim);
    for (const double bad : kBad) {
        CHECK_EQ(iad_axis_home(s.ax, ptr_of(home_opts(IAD_TRUE, bad, IAD_METHOD_DEFAULT)), &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK(refused_before_sending());
        CHECK_EQ(iad_axis_home(s.ax, ptr_of(home_opts(IAD_FALSE, bad, IAD_METHOD_DEFAULT)), &st),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_axis_servo(s.ax, IAD_FALSE, ptr_of(servo_opts(IAD_TRUE, bad)), &st), IAD_ERR_INVALID_ARGUMENT);
    }
    CHECK_EQ(commands_sent(s.sim), 0);
    st = s.status();
    CHECK(st.homed);  // a home return cut short leaves the axis un-homed partway
    CHECK(st.servo_on);
    CHECK_NEAR(st.position_mm, 60.0, 0.11);
    CHECK_EQ(iad_axis_servo(s.ax, IAD_FALSE, ptr_of(servo_opts(IAD_TRUE, kNaN)), &st), IAD_OK);
    clear_requests(s.sim);
    for (const double bad : kBad)
        CHECK_EQ(iad_axis_servo(s.ax, IAD_TRUE, ptr_of(servo_opts(IAD_TRUE, bad)), &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(commands_sent(s.sim), 0);
    CHECK(!s.status().servo_on);
    CHECK_EQ(iad_axis_servo(s.ax, IAD_TRUE, ptr_of(servo_opts(IAD_TRUE, 10.0)), &st), IAD_OK);
    CHECK(st.servo_on);
    CHECK_EQ(iad_axis_home(s.ax, ptr_of(home_opts(IAD_TRUE, kNaN, IAD_METHOD_DEFAULT)), &st), IAD_OK);
    CHECK(st.homed);
}

TEST(capi_timeouts_row_push_sequence_and_change_options_refused_before_anything_is_sent) {
    SimAxis s;
    iad_status st;
    clear_requests(s.sim);
    for (const double bad : kBad) {
        iad_position_move_options pm;
        REQUIRE(iad_position_move_options_init(&pm) == IAD_OK);
        pm.timeout_s = bad;
        CHECK_EQ(iad_axis_move_to_position(s.ax, 1, &pm, &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK(refused_before_sending());
        iad_push_options push;
        REQUIRE(iad_push_options_init(&push) == IAD_OK);
        push.timeout_s = bad;
        iad_push_result pr;
        CHECK_EQ(iad_axis_push(s.ax, 80.0, 10.0, 30.0, &push, &pr), IAD_ERR_INVALID_ARGUMENT);
        iad_sequence_options so;
        REQUIRE(iad_sequence_options_init(&so) == IAD_OK);
        so.timeout_s = bad;
        iad_sequence_point points[2];
        std::memset(points, 0, sizeof points);
        for (auto& p : points) p.struct_size = sizeof p;
        points[0].position = 30.0;
        points[1].position = 50.0;
        iad_status statuses[2];
        size_t count = 0;
        CHECK_EQ(iad_axis_move_sequence(s.ax, points, 2, &so, statuses, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    }
    CHECK_EQ(commands_sent(s.sim), 0);
    // a change of the running move: refused, the move runs on as it was sent
    iad_move_options slow = move_opts(kNaN, IAD_FALSE);
    slow.speed = 5.0;
    CHECK_EQ(iad_axis_move_to(s.ax, 140.0, &slow, nullptr), IAD_OK);
    clear_requests(s.sim);
    for (const double bad : kBad) {
        CHECK_EQ(iad_axis_change_speed(s.ax, 10.0, ptr_of(change_opts(IAD_TRUE, bad)), &st), IAD_ERR_INVALID_ARGUMENT);
        CHECK(refused_before_sending());
        CHECK_EQ(iad_axis_change_accel(s.ax, 0.2, ptr_of(change_opts(IAD_FALSE, bad)), &st), IAD_ERR_INVALID_ARGUMENT);
    }
    CHECK_EQ(commands_sent(s.sim), 0);
    CHECK(s.status().moving);
    CHECK_EQ(iad_axis_stop(s.ax, ptr_of(stop_opts(IAD_TRUE, 5.0)), &st), IAD_OK);
}

TEST(capi_timeouts_cell_moves_servo_all_and_home_all_refused_before_anything_is_sent) {
    std::vector<iad_sim*> sims(2, nullptr);
    for (int i = 0; i < 2; ++i) {
        const std::string options = R"({"time_scale": 20, "slave": )" + std::to_string(i + 1) + "}";
        REQUIRE(iad_sim_open(options.c_str(), &sims[i]) == IAD_OK);
    }
    iad_sim_line* line = nullptr;
    REQUIRE(iad_sim_line_open("L", sims.data(), sims.size(), &line) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_line_transport(line, &t) == IAD_OK);
    iad_bus* bus = nullptr;
    REQUIRE(iad_bus_open(t, nullptr, &bus) == IAD_OK);
    iad_transport_release(t);
    iad_axis_options ao;
    REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
    ao.poll_interval_s = 0.005;
    iad_axis* axes[2] = {nullptr, nullptr};
    REQUIRE(iad_axis_open_bus(bus, 1, &ao, &axes[0]) == IAD_OK);
    REQUIRE(iad_axis_open_bus(bus, 2, &ao, &axes[1]) == IAD_OK);
    iad_cell_options co;
    REQUIRE(iad_cell_options_init(&co) == IAD_OK);
    co.identify = IAD_FALSE;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_axes(axes, 2, nullptr, &co, &cell) == IAD_OK);
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);
    for (int i = 0; i < 2; ++i) {
        moves[i].struct_size = sizeof moves[i];
        moves[i].position = 20.0;
        moves[i].timeout_s = kNaN;
        std::snprintf(moves[i].axis, sizeof moves[i].axis, "L:%d", i + 1);
    }
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    for (iad_sim* sim : sims) clear_requests(sim);
    moves[0].position = moves[1].position = 120.0;
    for (const double bad : {-1.0, kInf}) {  // one axis's own timeout: no axis starts
        moves[1].timeout_s = bad;
        CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_TRUE, results, 2, &count),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK(refused_before_sending());
        CHECK_EQ(count, 0u);
    }
    for (const double bad : kBad) {
        moves[1].timeout_s = kNaN;
        CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, bad, IAD_TRUE, results, 2, &count),
                 IAD_ERR_INVALID_ARGUMENT);  // the call's
        const double positions[2] = {120.0, 120.0};
        CHECK_EQ(iad_cell_move_all(cell, "L:1,L:2", positions, 2, ptr_of(move_opts(bad)), nullptr, nullptr, nullptr,
                                   nullptr, results, 2, &count),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_cell_servo_all(cell, IAD_FALSE, nullptr, bad, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_cell_home_all(cell, nullptr, bad, IAD_METHOD_DEFAULT, IAD_TRUE, results, 2, &count),
                 IAD_ERR_INVALID_ARGUMENT);
    }
    for (iad_sim* sim : sims) CHECK_EQ(commands_sent(sim), 0);
    for (iad_axis* ax : axes) {
        iad_status st;
        REQUIRE(iad_axis_status(ax, &st) == IAD_OK);
        CHECK(!st.moving);
        CHECK(st.servo_on);
        CHECK(st.homed);
        CHECK_NEAR(st.position_mm, 20.0, 0.11);
    }
    moves[1].timeout_s = 0.0;  // in iad_axis_move (no _init) a 0 is the default, as documented
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, IAD_TRUE, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    CHECK_NEAR(results[1].status.position_mm, 120.0, 0.11);
    CHECK_EQ(iad_cell_release(cell), IAD_OK);
    iad_axis_close(axes[0]);
    iad_axis_close(axes[1]);
    iad_bus_close(bus);
    iad_sim_line_close(line);
    for (iad_sim* sim : sims) iad_sim_close(sim);
}
