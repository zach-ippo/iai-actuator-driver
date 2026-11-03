// Cell: open over several simulated ports, the group commands, the check-before-send rule of move_all and the link-fault
// handling (on_fault once, the other axes stopped). Simulator only (no COM port).
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

std::map<std::string, std::shared_ptr<SimLine>> sim_lines(int ports, const std::vector<int>& slaves) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 20.0;
    return SimLine::sim_ports(ports, slaves, o);
}

bool contains_text(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool sent_direct_move(const std::shared_ptr<SimulatedUnit>& unit) {
    for (const auto& frame : unit->requests())
        if (frame.size() > 3 && frame[1] == 0x10 && frame[2] == 0x99 && frame[3] == 0x00) return true;
    return false;
}

// A braced list of port names picks the ports overload (unevaluated: no port is opened).
static_assert(std::is_same<decltype(Cell::open({"COM3", "COM4"})), Cell>::value, "Cell::open({...}) is ambiguous");

}  // namespace

TEST(cell_open_names_axes_in_port_then_slave_order) {
    auto lines = sim_lines(2, {1, 2});
    Cell cell = Cell::open(lines);
    CHECK_EQ(cell.size(), std::size_t(4));
    const std::vector<std::string> expected{"SIM1:1", "SIM1:2", "SIM2:1", "SIM2:2"};
    CHECK(cell.names() == expected);
    CHECK(cell.contains("SIM2:2"));
    CHECK(!cell.contains("SIM3:1"));
    CHECK_EQ(cell.info().size(), std::size_t(4));
    CHECK_EQ(cell.units_of("SIM1:1"), std::string("mm"));
    CHECK_THROWS(cell["nope"], std::out_of_range);
    cell.close();
    CHECK(cell.closed());
    cell.close();  // safe twice
}

TEST(cell_group_commands_servo_home_move_status_stop) {
    auto lines = sim_lines(2, {1, 2});
    Cell cell = Cell::open(lines);
    CHECK(cell.servo_all(true).succeeded());
    CHECK(cell.home_all().succeeded());
    std::map<std::string, double> targets{{"SIM1:1", 20.0}, {"SIM1:2", 30.0}, {"SIM2:1", 40.0}, {"SIM2:2", 50.0}};
    MoveOptions mo;
    mo.speed = 100.0;
    const auto moved = cell.move_all(targets, mo);
    CHECK(moved.has_value());
    CHECK(moved->succeeded());
    const CellResults<Status> st = cell.status_all();
    for (const auto& t : targets) CHECK_NEAR(st.at(t.first).position_mm, t.second, 0.11);
    // per-axis options, no wait, then wait_all
    std::vector<AxisMove> each{{"SIM1:1", 25.0, mo}, {"SIM2:2", 45.0, mo}};
    CHECK(!cell.move_each(each, false).has_value());
    const CellResults<Status> waited = cell.wait_all({"SIM1:1", "SIM2:2"});
    CHECK_NEAR(waited.at("SIM1:1").position_mm, 25.0, 0.11);
    CHECK_NEAR(waited.at("SIM2:2").position_mm, 45.0, 0.11);
    CHECK(cell.stop_all().succeeded());
    CHECK(cell.faults().empty());
    cell.close();
}

TEST(cell_move_all_one_bad_target_refuses_the_whole_move) {
    auto lines = sim_lines(2, {1});
    Cell cell = Cell::open(lines);
    cell.servo_all(true);
    cell.home_all();
    for (const auto& l : lines) l.second->unit(1)->clear_requests();
    try {
        cell.move_all({{"SIM1:1", 10.0}, {"SIM2:1", 100000.0}});
        CHECK(false);
    } catch (const CellError& e) {
        CHECK(e.messages().count("SIM2:1") == 1);
        CHECK(std::string(e.what()).find("nothing started") != std::string::npos);
    }
    for (const auto& l : lines) CHECK(!sent_direct_move(l.second->unit(1)));  // nothing sent to either axis
    cell.close();
}

TEST(cell_link_fault_calls_on_fault_once_and_stops_the_others) {
    auto lines = sim_lines(2, {1});
    std::atomic<int> calls{0};
    std::string faulted_kind;
    CellOptions o;
    o.on_fault = [&](const std::string& name, const std::string& kind, const std::string&) {
        ++calls;
        if (name == "SIM2:1") faulted_kind = kind;
    };
    Cell cell = Cell::open(lines, {}, o);
    cell.servo_all(true);
    cell.home_all();
    MoveOptions slow;
    slow.speed = 5.0;  // long enough to still be moving when the other axis goes silent
    CHECK(!cell.move_all({{"SIM1:1", 80.0}, {"SIM2:1", 80.0}}, [&] {
        MoveOptions m = slow;
        m.wait = false;
        return m;
    }()).has_value());
    lines.at("SIM2")->unit(1)->inject_fault(SimFault::Timeout, 4);  // SIM2:1 misses one request and its 3 retries
    CHECK_THROWS(cell.wait_all(), CellError);
    CHECK_EQ(calls.load(), 1);
    CHECK_EQ(faulted_kind, std::string("unit_silent"));
    CHECK(cell.faults().count("SIM2:1") == 1);
    const auto sim1 = lines.at("SIM1")->unit(1);
    CHECK(!sim1->moving());             // the other axis was stopped
    CHECK(sim1->position_mm() < 79.0);  // before it reached its target
    CHECK(!sim1->stop_held());          // and its STOP released again
    cell.clear_fault("SIM2:1");
    CHECK(cell.faults().empty());
    cell.close();
}

TEST(cell_link_fault_stops_the_others_before_it_logs_or_calls_on_fault) {
    // STOP first (the load plan's A14): before, the fault was logged, the link state set and on_fault called before
    // the other axes got their STOP, so a log sink or an on_fault that blocks (a console, a binding waiting for its
    // interpreter's lock) delayed it. Now the fault's log line and on_fault both come after the STOP frame.
    auto lines = sim_lines(2, {1});
    const auto sim1 = lines.at("SIM1")->unit(1);
    const auto stop_sent = [&sim1] {
        for (const auto& f : sim1->requests())
            if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop && f[4] == 0xFF) return true;
        return false;
    };
    std::atomic<int> on_fault_saw_stop{-1};
    std::atomic<int> log_saw_stop{-1};
    CellOptions o;
    o.on_fault = [&](const std::string& name, const std::string&, const std::string&) {
        if (name == "SIM2:1") on_fault_saw_stop = stop_sent() ? 1 : 0;
    };
    const LogHandler previous = set_log_handler([&](LogLevel, const std::string&, const std::string& message) {
        if (message.rfind("SIM2:1: unit_silent: ", 0) == 0 && message.find("STOP sent to") == std::string::npos &&
            log_saw_stop.load() < 0)
            log_saw_stop = stop_sent() ? 1 : 0;
    });
    Cell cell = Cell::open(lines, {}, o);
    cell.servo_all(true);
    cell.home_all();
    sim1->clear_requests();
    MoveOptions slow;
    slow.speed = 5.0;
    slow.wait = false;
    CHECK(!cell.move_all({{"SIM1:1", 80.0}, {"SIM2:1", 80.0}}, slow).has_value());
    lines.at("SIM2")->unit(1)->inject_fault(SimFault::Timeout, 4);
    CHECK_THROWS(cell.wait_all(), CellError);
    set_log_handler(previous);
    CHECK_EQ(on_fault_saw_stop.load(), 1);
    CHECK_EQ(log_saw_stop.load(), 1);
    CHECK(!sim1->moving());
    cell.close();
}

TEST(cell_stop_all_holds_each_axis_until_its_full_stop_no_move_releases_it_meanwhile) {
    // Cell::stop_all sends the first STOP frame to every axis, then each axis's full stop. Parked in between (the stop
    // latency line of the first frame, logged once the first pass is done), a move from another thread used to find
    // no stop sequence running: it released the STOP (STOP OFF) and sent its 0x9900 block in the middle of the STOP
    // ALL. Now the axis is held by the group stop until its full stop ends: the move is refused, nothing sent.
    auto lines = sim_lines(1, {1});
    const auto sim = lines.at("SIM1")->unit(1);
    Cell cell = Cell::open(lines);
    cell.servo_all(true);
    cell.home_all();
    std::atomic<bool> armed{true};
    std::atomic<bool> parked{false};
    std::atomic<bool> released{false};
    const LogHandler previous = set_log_handler([&](LogLevel, const std::string&, const std::string& message) {
        if (message.find("STOP on the wire") == std::string::npos || !armed.exchange(false)) return;
        parked = true;
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!released.load() && std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });
    sim->clear_requests();
    const auto stop_trace = [&sim] {
        std::string out;
        for (const auto& f : sim->requests())
            if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop) out += f[4] == 0xFF ? '1' : '0';
        return out;
    };
    std::string what;
    {
        ec_test::Thread stopper([&] { cell.stop_all(); });
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!parked.load() && std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(parked.load());
        CHECK(sim->stop_held());  // the first STOP frame; the full stop has not begun
        MoveOptions o;
        o.speed = 20.0;
        o.wait = false;
        try {
            cell["SIM1:1"].move_to(80.0, o);
        } catch (const EcNotReadyError& e) {
            what = e.what();
        }
        CHECK(sim->stop_held());
        CHECK_EQ(stop_trace(), std::string("1"));
        CHECK(!sent_direct_move(sim));
        released = true;
        stopper.join();
    }
    set_log_handler(previous);
    CHECK(contains_text(what, "a STOP is held"));
    CHECK(!sim->stop_held());  // the full stop released it when it ended
    CHECK_EQ(stop_trace(), std::string("110"));
    CHECK(!sent_direct_move(sim));
    cell.close();
}
