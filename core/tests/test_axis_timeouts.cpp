// Axis unit: a timeout of 0, a negative one or an infinity is refused by the core itself (SoftLimitError, nothing
// sent) for every command that starts motion or a servo change, so C++ callers and every binding get the rule the C
// ABI's converters apply; nullopt stays the default. Cell::move_each refuses one axis's bad timeout in its check pass
// (no axis starts), and Cell::stop_all / pause_all act on the axes they have when a listed name is unknown.
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

const double kBad[] = {-1.0, 0.0, std::numeric_limits<double>::infinity()};

std::shared_ptr<SimulatedUnit> make_sim(double scale = 20.0) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = scale;
    return std::make_shared<SimulatedUnit>(o);
}

int commands(const std::vector<Bytes>& requests) {
    int n = 0;
    for (const auto& f : requests)
        if (f.size() > 1 && (f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x0F || f[1] == 0x10 || f[1] == 0x45)) ++n;
    return n;
}

MoveOptions with_timeout(double timeout_s, bool wait = true) {
    MoveOptions o;
    o.timeout_s = timeout_s;
    o.wait = wait;
    return o;
}

}  // namespace

TEST(axis_timeouts_refused_by_the_core_before_anything_is_sent) {
    auto sim = make_sim();
    OpenOptions oo;
    oo.slave = sim->slave();
    Axis ax = Axis::open(sim, oo);
    ax.connect();
    MoveOptions slow;
    slow.speed = 5.0;
    slow.wait = false;
    ax.move_to(60.0, MoveOptions{});
    sim->clear_requests();
    for (const double bad : kBad) {
        CHECK_THROWS(ax.check_move(80.0, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(ax.move_to(80.0, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(ax.move_to(80.0, with_timeout(bad, false)), SoftLimitError);
        MoveOptions profile = with_timeout(bad);
        profile.decel_g = 0.2;  // the command-profile path
        CHECK_THROWS(ax.move_to(80.0, profile), SoftLimitError);
        CHECK_THROWS(ax.move_by(10.0, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(ax.jog(10.0, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(ax.start_move_to(80.0, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(ax.home(true, bad), SoftLimitError);
        CHECK_THROWS(ax.servo(false, true, bad), SoftLimitError);
        PositionMoveOptions pm;
        pm.timeout_s = bad;
        CHECK_THROWS(ax.move_to_position(1, pm), SoftLimitError);
        PushOptions push;
        push.timeout_s = bad;
        CHECK_THROWS(ax.push(80.0, 10.0, 30.0, push), SoftLimitError);
        SequenceOptions so;
        so.timeout_s = bad;
        CHECK_THROWS(ax.move_sequence({30.0, 50.0}, so), SoftLimitError);
    }
    CHECK_EQ(commands(sim->requests()), 0);
    Status st = ax.status();
    CHECK(st.homed);
    CHECK(st.servo_on);
    CHECK_NEAR(st.position_mm, 60.0, 0.11);
    try {
        ax.move_to(80.0, with_timeout(-1.0));
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("nothing sent") != std::string::npos);
    }
    ax.move_to(140.0, slow);  // a change of the running move: refused, the move runs on as sent
    sim->clear_requests();
    for (const double bad : kBad) {
        CHECK_THROWS(ax.change_speed(10.0, true, bad), SoftLimitError);
        CHECK_THROWS(ax.change_accel(0.2, false, bad), SoftLimitError);
    }
    CHECK_EQ(commands(sim->requests()), 0);
    CHECK(ax.status().moving);
    ax.stop(true);
    ax.servo(false);
    sim->clear_requests();
    for (const double bad : kBad) CHECK_THROWS(ax.servo(true, true, bad), SoftLimitError);
    CHECK_EQ(commands(sim->requests()), 0);
    CHECK(!ax.status().servo_on);
    CHECK(ax.servo(true, true, 10.0).has_value());  // a finite timeout above 0
}

TEST(axis_timeouts_cell_refuses_a_bad_timeout_before_any_axis_starts) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Slider;
    so.time_scale = 20.0;
    auto lines = SimLine::sim_ports(2, {1}, so);
    Cell cell = Cell::open(lines);
    auto count = [&] {
        int n = 0;
        for (const auto& kv : lines) n += commands(kv.second->unit(1)->requests());
        return n;
    };
    for (const auto& kv : lines) kv.second->unit(1)->clear_requests();
    for (const double bad : kBad) {
        std::vector<AxisMove> moves{{"SIM1:1", 100.0, MoveOptions{}}, {"SIM2:1", 100.0, with_timeout(bad)}};
        CHECK_THROWS(cell.move_each(moves), CellError);  // the check pass: nothing started
        moves[1].options.timeout_s.reset();
        CHECK_THROWS(cell.move_each(moves, true, bad), SoftLimitError);  // the call's own
        CHECK_THROWS(cell.move_all({{"SIM1:1", 100.0}, {"SIM2:1", 100.0}}, with_timeout(bad)), SoftLimitError);
        CHECK_THROWS(cell.home_all({}, bad), SoftLimitError);
        CHECK_THROWS(cell.servo_all(false, {}, bad), SoftLimitError);
    }
    CHECK_EQ(count(), 0);
    for (const auto& n : cell.names()) {
        const Status st = cell[n].status();
        CHECK(!st.moving);
        CHECK(st.servo_on);
        CHECK(st.homed);
    }
    try {
        cell.move_each({{"SIM1:1", 100.0, MoveOptions{}}, {"SIM2:1", 100.0, with_timeout(-1.0)}});
        CHECK(false);
    } catch (const CellError& e) {
        CHECK_EQ(e.operation(), std::string("move_each (check; nothing started)"));
        CHECK_EQ(e.errors().size(), std::size_t(1));
        CHECK(e.errors().count("SIM2:1") == 1);
    }
    cell.close();
}

TEST(axis_timeouts_cell_stop_all_and_pause_all_act_on_the_axes_they_have) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Slider;
    so.time_scale = 20.0;
    auto lines = SimLine::sim_ports(2, {1}, so);
    Cell cell = Cell::open(lines);
    MoveOptions slow;
    slow.speed = 2.0;  // 3 s of real time at x20
    slow.wait = false;
    cell.move_all({{"SIM1:1", 140.0}, {"SIM2:1", 140.0}}, slow);
    try {
        cell.stop_all({"SIM1:1", "NOSUCH:9"});
        CHECK(false);
    } catch (const CellError& e) {
        CHECK_EQ(e.errors().size(), std::size_t(1));
        REQUIRE(e.errors().count("NOSUCH:9") == 1);
        CHECK_THROWS(std::rethrow_exception(e.errors().at("NOSUCH:9")), std::out_of_range);
    }
    CHECK(!lines.at("SIM1")->unit(1)->moving());  // stopped although the list named an unknown axis
    CHECK(lines.at("SIM2")->unit(1)->moving());   // not listed
    const CellResults<bool> paused = cell.pause_all({"NOSUCH:9", "SIM2:1", "NOSUCH:9"}, true);
    CHECK(paused.names == (std::vector<std::string>{"SIM2:1", "NOSUCH:9"}));
    CHECK(paused.contains("SIM2:1"));
    CHECK_EQ(paused.errors.size(), std::size_t(1));
    CHECK(lines.at("SIM2")->unit(1)->paused());
    cell.resume_all({"SIM2:1"});
    const CellResults<Status> none = cell.stop_all({"NOSUCH:9"}, 5.0, true);  // never "every axis"
    CHECK(none.results.empty());
    CHECK(lines.at("SIM2")->unit(1)->moving());
    CHECK(cell.stop_all().succeeded());
    cell.close();
}
