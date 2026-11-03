// Cell extras: status monitors (start_monitors / stop_monitors, status_all from a fresh sample, link events routed to
// on_fault, recover_all restarting a monitor), move_all with per-axis values (a deceleration of its own: the
// command-profile move), monitor_all, backup, pause_all / resume_all with the broadcast, the Port broadcasts and the
// open plumbing. Ports of the Python edition's test_fast_recovery.py CellMonitorTest (+ the shared-line edge case),
// test_integration.py MixedCellTest, test_link.py CellBroadcastTest and test_cell.py CellPlumbingTest. Simulator only
// (no COM port).
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
namespace rc = iai_actuator_driver::recovery;

namespace {

using Frame = std::vector<std::uint8_t>;
using Pairs = std::vector<std::pair<std::string, std::string>>;

CellOptions quick_cell() {
    CellOptions o;
    o.axis.poll_interval_s = 0.005;
    o.scan_timeout_s = 0.01;
    o.bus.min_gap_s = 0.0;
    return o;
}

std::map<std::string, std::shared_ptr<SimLine>> lines(int n, double time_scale, const std::vector<int>& slaves = {1}) {
    SimulatedUnitOptions o;
    o.time_scale = time_scale;
    return SimLine::sim_ports(n, slaves, o);
}

std::shared_ptr<SimulatedUnit> unit(int slave, SimProfile profile, double time_scale,
                                    std::function<void(SimulatedUnitOptions&)> more = {}) {
    SimulatedUnitOptions o;
    o.slave = slave;
    o.profile = profile;
    o.time_scale = time_scale;
    if (more) more(o);
    return std::make_shared<SimulatedUnit>(o);
}

rc::RecoveryPolicy quick_policy() {
    rc::RecoveryPolicy p;
    p.poll_s = 0.02;
    return p;
}

/// Generous waits: the PC may be busy with other test runs.
bool wait_until(const std::function<bool()>& condition, double timeout_s = 8.0) {
    const double end = now_s() + timeout_s;
    while (now_s() < end) {
        if (condition()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
}

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

/// on_fault calls, thread-safe (the monitors' fault handling runs in threads of its own).
struct Faults {
    std::mutex mutex;
    Pairs items;
    FaultCallback callback() {
        return [this](const std::string& name, const std::string& kind, const std::string&) {
            std::lock_guard<std::mutex> lock(mutex);
            items.emplace_back(name, kind);
        };
    }
    Pairs list() {
        std::lock_guard<std::mutex> lock(mutex);
        return items;
    }
};

std::vector<Frame> frames_since(const SimLine& line, std::size_t mark) {
    std::vector<Frame> out;
    const auto requests = line.requests();
    for (std::size_t i = mark; i < requests.size(); ++i) out.push_back(requests[i].second);
    return out;
}

bool sent_motion(const std::shared_ptr<SimulatedUnit>& u) {
    for (const auto& f : u->requests()) {
        if (f.size() < 4) continue;
        if (f[1] == 0x17) return true;                                    // the command-profile block
        if (f[1] == 0x10 && f[2] == 0x99 && f[3] == 0x00) return true;    // the 0x9900 block
    }
    return false;
}

std::shared_ptr<SimLine> mixed_line(double time_scale = 20.0) {
    return std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{unit(1, SimProfile::Slider, time_scale),
                                                    unit(2, SimProfile::Pcon, time_scale),
                                                    unit(3, SimProfile::Rotary, time_scale)},
        "MIX");
}

using LineMap = std::map<std::string, std::shared_ptr<SimLine>>;

}  // namespace

// ============================================================================================ status monitors

TEST(cell_status_all_takes_a_running_monitors_sample) {
    auto ports = lines(2, 50.0);
    Cell cell = Cell::open(ports, {}, quick_cell());
    MonitorOptions mo;
    mo.link_check_s = std::nullopt;  // no full status read by a link check while the frames are counted
    const auto mons = cell.start_monitors({"SIM1:1"}, mo);
    REQUIRE(mons.size() == 1 && mons.count("SIM1:1") == 1);
    CHECK(wait_until([&] { return mons.at("SIM1:1")->polls() >= 3; }));
    const std::size_t mark = ports.at("SIM1")->requests().size();
    const CellResults<Status> res = cell.status_all({}, false, 2.0);  // a generous age: a busy PC may delay a poll
    CHECK_EQ(res.at("SIM1:1").raw.words, 8);  // the monitor's lean sample
    CHECK(!res.at("SIM1:1").raw.stim.has_value());
    CHECK(res.at("SIM2:1").raw.stim.has_value());  // read
    int full = 0;
    for (const Frame& f : frames_since(*ports.at("SIM1"), mark))
        if (f.size() > 5 && f[0] == 1 && f[1] == 0x03 && f[5] == 22) ++full;
    CHECK_EQ(full, 0);  // no full status read for the monitored axis
    CHECK(cell.status_all({}, false, std::nullopt).at("SIM1:1").raw.stim.has_value());  // always read
    cell.stop_monitors();
    CHECK(cell["SIM1:1"].monitor() == nullptr);
    CHECK(!mons.at("SIM1:1")->running());
    CHECK(cell.status_all().at("SIM1:1").raw.stim.has_value());
    cell.close();
}

TEST(cell_monitor_link_events_go_to_the_fault_handling_and_recover_all_restarts_it) {
    auto ports = lines(2, 50.0);
    Faults faults;
    CellOptions o = quick_cell();
    o.on_fault = faults.callback();
    Cell cell = Cell::open(ports, {}, o);
    cell.track_all();
    const auto mons = cell.start_monitors();
    REQUIRE(mons.size() == 2);
    const auto again = cell.start_monitors();  // the running monitors: their events still reach the cell once
    CHECK(again.at("SIM1:1") == mons.at("SIM1:1"));
    CHECK(wait_until([&] { return mons.at("SIM1:1")->polls() >= 3 && mons.at("SIM2:1")->polls() >= 3; }));
    ports.at("SIM1")->unplug();
    const std::map<std::string, std::string> vanished{{"SIM1:1", rc::kPortVanished}};
    CHECK(wait_until([&] { return cell.faults() == vanished; }));
    CHECK(wait_until([&] { return faults.list() == Pairs{{"SIM1:1", rc::kPortVanished}}; }));
    const auto events = cell.events();
    REQUIRE(events.size() == 1);
    CHECK_EQ(events[0].kind, std::string("link_lost"));
    CHECK_EQ(events[0].axis, std::string("SIM1:1"));
    CHECK_EQ(events[0].cause, std::string(rc::kPortVanished));
    CHECK(wait_until([&] { return !mons.at("SIM1:1")->running(); }));
    ports.at("SIM1")->replug();
    const auto reports = cell.recover_all({}, 3.0, quick_policy());
    CHECK(reports.at("SIM1:1").port_reopened);
    CHECK(cell.faults().empty());
    CHECK(mons.at("SIM1:1")->running());  // recover_all restarted the monitor
    CHECK(cell["SIM1:1"].monitor() == mons.at("SIM1:1"));
    CHECK(mons.at("SIM2:1")->running());
    CHECK_EQ(faults.list().size(), std::size_t(1));
    cell.close();
    CHECK(!mons.at("SIM1:1")->running());  // close() stops every monitor
    CHECK(!mons.at("SIM2:1")->running());
}

TEST(cell_monitor_fault_stops_the_other_axes) {
    auto ports = lines(2, 2.0);
    Faults faults;
    CellOptions o = quick_cell();
    o.on_fault = faults.callback();
    Cell cell = Cell::open(ports, {}, o);
    cell.start_monitors({"SIM1:1"});
    MoveOptions slow;
    slow.speed = 30.0;
    slow.wait = false;
    cell.move_all({{"SIM2:1", 140.0}}, slow);
    ports.at("SIM1")->unplug();
    CHECK(wait_until([&] { return faults.list().size() == 1; }));
    const auto sim2 = ports.at("SIM2")->unit(1);
    CHECK(wait_until([&] { return !sim2->moving(); }));  // stopped by the fault handling, from its own thread
    CHECK(sim2->position_mm() < 139.0);
    CHECK(wait_until([&] { return !sim2->stop_held(); }));  // and its STOP released again
    cell.close();
}

TEST(cell_recovering_one_axis_of_a_shared_line_holds_the_other_monitors) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    for (int s : {1, 2})
        units.push_back(unit(s, SimProfile::Gripper, 50.0, [s](SimulatedUnitOptions& u) {
            u.baud_rate = 115200;
            u.serial_number = "SN-000" + std::to_string(s);
        }));
    auto line = std::make_shared<SimLine>(units, "LS");
    Faults faults;
    CellOptions o = quick_cell();
    o.on_fault = faults.callback();
    Cell cell = Cell::open(LineMap{{"LS", line}}, {1, 2}, o);
    cell.track_all();
    const auto mons = cell.start_monitors();
    CHECK(wait_until([&] { return mons.at("LS:1")->polls() >= 3 && mons.at("LS:2")->polls() >= 3; }));
    line->power_cycle({1});  // unit 1 only: back at 38400, unit 2 still at 115200
    CHECK(wait_until([&] { return cell.faults().count("LS:1") == 1; }));
    const auto reports = cell.recover_all({"LS:1"}, 5.0, quick_policy());
    CHECK(reports.at("LS:1").baud_restored);
    const long polls = mons.at("LS:2")->polls();
    CHECK(wait_until([&] { return mons.at("LS:2")->polls() > polls + 5; }));
    sleep_s(0.2);  // a fault thread started by a failed sibling poll would have run by now
    CHECK(faults.list() == (Pairs{{"LS:1", rc::kUnitSilent}}));
    CHECK(cell.faults().empty());
    CHECK_EQ(rc::link_state(cell["LS:2"]), std::string(rc::kLinkOk));
    cell.close();
}

// ============================================================================================ mixed line, decel, monitor_all

TEST(cell_mixed_line_names_every_family_and_monitor_all) {
    auto line = mixed_line();
    Cell cell = Cell::open(LineMap{{"MIX", line}}, {}, quick_cell());
    CHECK(cell.names() == (std::vector<std::string>{"MIX:1", "MIX:2", "MIX:3"}));
    const auto info = cell.info();
    CHECK_EQ(info.at("MIX:1").product_family.value_or(""), std::string("EC"));
    CHECK_EQ(info.at("MIX:2").product_family.value_or(""), std::string("PCON"));
    CHECK_EQ(info.at("MIX:3").product_family.value_or(""), std::string("EC"));
    CHECK_EQ(cell.units_of("MIX:1"), std::string("mm"));
    CHECK_EQ(cell.units_of("MIX:2"), std::string("mm"));
    CHECK_EQ(cell.units_of("MIX:3"), std::string("deg"));
    const auto limits = cell.limits_all();
    CHECK_EQ(limits.at("MIX:3").units, std::string("deg"));
    CHECK_EQ(limits.at("MIX:2").family, std::string("PCON"));
    CHECK(limits.at("MIX:2").stored_rows.max.value_or(0) > 2);  // the PCON uses more than 3 rows
    const CellResults<Json> mon = cell.monitor_all();
    CHECK(mon.succeeded());
    CHECK_EQ(mon.results.size(), std::size_t(3));
    for (const auto& n : cell.names()) CHECK(mon.contains(n));
    const Json* units = mon.at("MIX:3").find("units");
    REQUIRE(units != nullptr);
    CHECK_EQ(units->as_string(), std::string("deg"));
    for (const char* key : {"position", "velocity", "current", "current_pct", "deviation", "alarm", "alarm_name",
                            "uptime_s", "raw"})
        CHECK(mon.at("MIX:1").find(key) != nullptr);
    cell.close();
}

TEST(cell_mixed_moves_with_their_own_deceleration) {
    auto line = mixed_line();
    Cell cell = Cell::open(LineMap{{"MIX", line}}, {}, quick_cell());
    MoveOptions mo;
    mo.speed = 100.0;
    mo.accel_g = 0.3;
    PerAxisMoveValues per;
    per.decel_g = {{"MIX:1", 0.3}, {"MIX:2", 0.1}, {"MIX:3", 0.2}};
    const auto done = cell.move_all({{"MIX:1", 60.0}, {"MIX:2", 100.0}, {"MIX:3", 45.0}}, mo, per);
    REQUIRE(done.has_value());
    CHECK_NEAR(done->at("MIX:1").position_mm, 60.0, 0.1);
    CHECK_NEAR(done->at("MIX:2").position_mm, 100.0, 0.1);
    CHECK_NEAR(done->at("MIX:3").position_mm, 45.0, 0.1);
    CHECK(line->unit(2)->profile_moves() >= 1);  // the PCON took the command-profile path
    CHECK(line->unit(3)->profile_moves() >= 1);
    CHECK_EQ(line->unit(1)->profile_moves(), 0);  // decel == accel: the direct numeric move
    // per-axis speeds and bands too; no wait, then wait_all
    PerAxisMoveValues each;
    each.speed = {{"MIX:1", 50.0}, {"MIX:2", 80.0}};
    each.band = {{"MIX:1", 0.2}, {"MIX:2", 0.1}};
    MoveOptions nowait;
    nowait.wait = false;
    CHECK(!cell.move_all({{"MIX:1", 70.0}, {"MIX:2", 90.0}}, nowait, each).has_value());
    const auto waited = cell.wait_all({"MIX:1", "MIX:2"});
    CHECK_NEAR(waited.at("MIX:1").position_mm, 70.0, 0.2);
    CHECK_NEAR(waited.at("MIX:2").position_mm, 90.0, 0.1);
    cell.close();
}

TEST(cell_move_all_per_axis_values_refused_before_anything_is_sent) {
    auto line = mixed_line();
    Cell cell = Cell::open(LineMap{{"MIX", line}}, {}, quick_cell());
    for (int s : {1, 2, 3}) line->unit(s)->clear_requests();
    PerAxisMoveValues missing;
    missing.decel_g = {{"MIX:1", 0.2}};  // no value for MIX:2
    CHECK_THROWS(cell.move_all({{"MIX:1", 60.0}, {"MIX:2", 100.0}}, MoveOptions{}, missing), std::out_of_range);
    PerAxisMoveValues unknown;
    unknown.speed = {{"MIX:1", 20.0}, {"MIX:9", 20.0}};
    CHECK_THROWS(cell.move_all({{"MIX:1", 60.0}}, MoveOptions{}, unknown), std::out_of_range);
    PerAxisMoveValues too_fast;
    too_fast.decel_g = {{"MIX:1", 0.1}, {"MIX:2", 5.0}};  // above MAXD: the check pass refuses the whole move
    MoveOptions mo;
    mo.accel_g = 0.2;
    try {
        cell.move_all({{"MIX:1", 60.0}, {"MIX:2", 100.0}}, mo, too_fast);
        CHECK(false);
    } catch (const CellError& e) {
        CHECK(e.messages().count("MIX:2") == 1);
        CHECK(std::string(e.what()).find("nothing started") != std::string::npos);
    }
    for (int s : {1, 2, 3}) CHECK(!sent_motion(line->unit(s)));
    cell.close();
}

TEST(cell_backup_one_file_per_axis_read_only) {
    auto ports = lines(2, 20.0);
    Cell cell = Cell::open(ports, {}, quick_cell());
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() /
        ("iad_cell_backup_" + std::to_string(static_cast<long long>(now_s() * 1000.0)));
    std::map<std::string, std::size_t> marks;
    for (const auto& kv : ports) marks[kv.first] = kv.second->requests().size();
    const auto saved = cell.backup(dir.string());
    REQUIRE(saved.size() == 2);
    CHECK_EQ(saved[0].first, std::string("SIM1:1"));
    CHECK_EQ(saved[1].first, std::string("SIM2:1"));
    for (const auto& kv : saved) CHECK(std::filesystem::exists(kv.second));
    for (const auto& kv : ports)
        for (const Frame& f : frames_since(*kv.second, marks[kv.first]))
            CHECK(f.size() > 1 && (f[1] == 0x03 || f[1] == 0x11));  // FC03 / FC11 only: nothing written
    CHECK_THROWS(cell.backup(dir.string(), {"SIM9:1"}), std::out_of_range);
    cell.close();
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ============================================================================================ pause / resume, broadcasts

TEST(cell_stop_all_logs_no_stop_line_between_two_first_stop_frames) {
    // C1: every stop logs one latency line, but the first STOP frames of a port's axes go out back to back: their
    // lines are logged once the port's last STOP frame is out, never between two of them (a log handler may wait for
    // an interpreter lock). The full stop that follows logs each axis's standstill.
    auto line = std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{unit(1, SimProfile::Slider, 1.0), unit(2, SimProfile::Slider, 1.0)},
        "L");
    Cell cell = Cell::open(LineMap{{"L", line}}, {}, quick_cell());
    MoveOptions mo;
    mo.speed = 20.0;
    mo.wait = false;
    cell.move_all({{"L:1", 140.0}, {"L:2", 140.0}}, mo);
    const std::size_t mark = line->requests().size();
    const auto stops_on_the_line = [&] {
        int n = 0;
        for (const Frame& f : frames_since(*line, mark))
            if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == 0x042C && f[4] == 0xFF) ++n;
        return n;
    };
    std::mutex mutex;
    std::vector<std::pair<std::string, int>> seen;  // (line, STOP ON frames on the line when it was logged)
    const LogHandler previous = set_log_handler([&](LogLevel level, const std::string&, const std::string& message) {
        if (level != LogLevel::Info || message.find(" on the wire ") == std::string::npos) return;
        const int stops = stops_on_the_line();
        std::lock_guard<std::mutex> lock(mutex);
        seen.emplace_back(message, stops);
    });
    const CellResults<Status> out = cell.stop_all();
    set_log_handler(previous);
    CHECK(out.succeeded());
    int first = 0, full = 0;
    for (const auto& s : seen) {
        if (s.first.find("standstill confirmed") != std::string::npos) {
            ++full;
        } else {
            ++first;
            CHECK_EQ(s.second, 2);  // both first STOP frames were out before the first line
        }
    }
    CHECK_EQ(first, 2);
    CHECK_EQ(full, 2);
    cell.close();
}

TEST(cell_pause_and_resume_all_broadcast) {
    auto line = std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{unit(1, SimProfile::Slider, 1.0), unit(2, SimProfile::Slider, 1.0)},
        "L");
    Cell cell = Cell::open(LineMap{{"L", line}}, {}, quick_cell());
    MoveOptions mo;
    mo.speed = 20.0;
    mo.wait = false;
    cell.move_all({{"L:1", 140.0}, {"L:2", 140.0}}, mo);
    std::size_t mark = line->requests().size();
    CHECK(cell.pause_all({}, false, true).succeeded());
    auto after = frames_since(*line, mark);
    REQUIRE(!after.empty());
    CHECK(after.front() == (Frame{0x00, 0x05, 0x04, 0x0A, 0xFF, 0x00, 0xAC, 0xD9}));  // the broadcast pause first
    for (int s : {1, 2}) CHECK(line->unit(s)->paused());
    sleep_s(0.3);
    const double held1 = line->unit(1)->position_mm();
    const double held2 = line->unit(2)->position_mm();
    sleep_s(0.2);
    CHECK_EQ(line->unit(1)->position_mm(), held1);  // no axis moved while paused
    CHECK_EQ(line->unit(2)->position_mm(), held2);
    mark = line->requests().size();
    CHECK(cell.resume_all({}, false, true).succeeded());
    after = frames_since(*line, mark);
    REQUIRE(!after.empty());
    CHECK(after.back() == (Frame{0x00, 0x05, 0x04, 0x0A, 0x00, 0x00, 0xED, 0x29}));  // the broadcast release last
    CHECK(wait_until([&] { return line->unit(1)->moving() && line->unit(2)->moving(); }));
    cell.stop_all();
    cell.close();
}

TEST(cell_pause_all_without_broadcast_and_on_a_single_axis_port) {
    auto ports = lines(2, 1.0, {1, 2});
    ports.at("SIM2")->remove(2);
    Cell cell = Cell::open(ports, {}, quick_cell());
    std::map<std::string, std::size_t> marks;
    for (const auto& kv : ports) marks[kv.first] = kv.second->requests().size();
    CHECK(cell.pause_all().succeeded());  // the default sends no broadcast
    CHECK(cell.resume_all({"SIM1:1", "SIM2:1"}, false, true).succeeded());  // SIM1:2 not selected; SIM2 one axis
    for (const auto& kv : ports)
        for (const Frame& f : frames_since(*kv.second, marks[kv.first])) CHECK(f.empty() || f[0] != 0);
    for (const auto& n : cell.names()) CHECK(!cell[n].status().paused() || n == "SIM1:2");
    cell.resume_all();
    cell.close();
}

TEST(cell_port_broadcasts_refused_on_rcon) {
    auto line = sim_rcon_line({1, 2});
    Port port(line, "R", BusOptions{}, LineOptions{});
    port.scan();
    CHECK_THROWS(port.broadcast_stop(), LineRuleError);
    CHECK_THROWS(port.broadcast_pause(), LineRuleError);
    CHECK_THROWS(port.broadcast_pause(false), LineRuleError);
    CHECK_THROWS(port.broadcast_servo_off(), LineRuleError);
    for (int s : line->request_slaves()) CHECK(s != 0);
    CHECK(line->dropped_broadcasts().empty());
    port.close();
}

TEST(cell_port_broadcast_frames_on_a_daisy_chain) {
    SimulatedUnitOptions u;
    u.time_scale = 1.0;
    auto line = SimLine::with_slaves({1, 2}, u, "D");
    Port port(line, "D", BusOptions{}, LineOptions{});
    port.scan();
    AxisOptions ao;
    ao.poll_interval_s = 0.005;
    Axis a1(port, 1, ao);
    Axis a2(port, 2, ao);
    MoveOptions mo;
    mo.speed = 20.0;
    mo.wait = false;
    a1.move_to(140.0, mo);
    a2.move_to(140.0, mo);
    CHECK(port.broadcast_pause(true) == (Frame{0x00, 0x05, 0x04, 0x0A, 0xFF, 0x00, 0xAC, 0xD9}));
    for (int s : {1, 2}) CHECK(line->unit(s)->paused());  // one frame paused both moves
    CHECK(port.broadcast_pause(false) == (Frame{0x00, 0x05, 0x04, 0x0A, 0x00, 0x00, 0xED, 0x29}));
    for (int s : {1, 2}) CHECK(!line->unit(s)->paused());
    CHECK(wait_until([&] { return line->unit(1)->moving() && line->unit(2)->moving(); }));
    a1.stop();
    a2.stop();
    CHECK(port.broadcast_servo_off() == (Frame{0x00, 0x05, 0x04, 0x03, 0x00, 0x00, 0x3D, 0x2B}));
    for (int s : {1, 2}) CHECK(!line->unit(s)->servo_on());
    a1.close();
    a2.close();
    port.close();
}

// ============================================================================================ open plumbing (test_cell.py)

TEST(cell_plumbing_names_lookup_and_one_bus_per_port) {
    auto ports = lines(2, 20.0, {1, 3});
    CellOptions o = quick_cell();
    o.identify = false;
    Cell cell = Cell::open(ports, {}, o);
    CHECK(cell.names() == (std::vector<std::string>{"SIM1:1", "SIM1:3", "SIM2:1", "SIM2:3"}));
    CHECK(&cell["SIM2:3"] == &cell[3]);
    CHECK_EQ(cell.port_of("SIM2:1")->name(), std::string("SIM2"));
    CHECK(cell[0].bus() == cell[1].bus());  // one bus per port, shared by its axes
    CHECK(cell[0].bus() != cell[2].bus());
    CHECK_THROWS(cell["COM3:1"], std::out_of_range);
    cell.close();
    for (const auto& kv : ports) CHECK(!kv.second->is_open());
}

TEST(cell_plumbing_identify_reads_each_units_own_ranges) {
    auto line = std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{unit(1, SimProfile::Slider, 20.0), unit(2, SimProfile::Gripper, 20.0)},
        "L");
    Cell cell = Cell::open(LineMap{{"L", line}}, {}, quick_cell());
    const auto info = cell.info();
    REQUIRE(info.at("L:1").soft_limits_mm.has_value() && info.at("L:2").soft_limits_mm.has_value());
    CHECK_NEAR(info.at("L:1").soft_limits_mm->min, -0.3, 1e-9);
    CHECK_NEAR(info.at("L:1").soft_limits_mm->max, 150.3, 1e-9);
    CHECK_NEAR(info.at("L:2").soft_limits_mm->min, -0.3, 1e-9);
    CHECK_NEAR(info.at("L:2").soft_limits_mm->max, 9.8, 1e-9);
    cell.close();
}

TEST(cell_plumbing_listed_slaves_must_answer_and_skip_the_scan) {
    {
        SimulatedUnitOptions u;
        auto line = SimLine::with_slaves({1}, u, "L");
        CellOptions o = quick_cell();
        o.identify = false;
        try {
            Cell::open(LineMap{{"L", line}}, {1, 2}, o);
            CHECK(false);
        } catch (const ModbusTimeout& e) {
            CHECK(std::string(e.what()).find("slave(s) 2") != std::string::npos);
        }
        CHECK(!line->is_open());
    }
    auto line = SimLine::with_slaves({1, 2, 3}, SimulatedUnitOptions{}, "L");
    CellOptions o = quick_cell();
    o.identify = false;
    Cell cell = Cell::open(LineMap{{"L", line}}, {2}, o);
    CHECK(cell.names() == std::vector<std::string>{"L:2"});
    for (int s : line->request_slaves()) CHECK(s != 9);  // no scan
    cell.close();
}

TEST(cell_plumbing_max_axes_scan_skips_dead_ports_and_nothing_found) {
    CellOptions o = quick_cell();
    o.identify = false;
    o.max_axes = 2;
    CHECK_THROWS(Cell::open(LineMap{{"L", SimLine::with_slaves({1, 2, 3}, SimulatedUnitOptions{}, "L")}}, {1, 2, 3}, o),
                 std::invalid_argument);
    {
        Cell cell = Cell::open(LineMap{{"M", SimLine::with_slaves({1, 2, 3}, SimulatedUnitOptions{}, "M")}}, {}, o);
        CHECK(cell.names() == (std::vector<std::string>{"M:1", "M:2"}));  // the first max_axes kept
    }
    o.max_axes.reset();  // no limit
    o.scan_slaves = {1, 2, 3};
    auto dead = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{}, "B");
    {
        Cell cell = Cell::open(LineMap{{"A", SimLine::with_slaves({1}, SimulatedUnitOptions{}, "A")}, {"B", dead}}, {}, o);
        CHECK(cell.names() == std::vector<std::string>{"A:1"});
        CHECK(cell.port_errors().count("B") == 1);
    }
    CHECK(!dead->is_open());
    o.scan_slaves = {1, 2};
    CHECK_THROWS(Cell::open(LineMap{{"E", std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{}, "E")}},
                            {}, o),
                 EcNotReadyError);
}

TEST(cell_takes_every_axis_by_default_and_max_axes_caps_it) {
    CellOptions o = quick_cell();
    o.identify = false;
    o.scan_slaves = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    CHECK(!o.max_axes);  // the default: no limit
    std::vector<int> nine(o.scan_slaves);
    {
        Cell cell = Cell::open(lines(2, 1.0, nine), {}, o);  // 2 lines x 9 = 18 axes, more than one line carries
        CHECK_EQ(cell.names().size(), std::size_t(18));
    }
    {
        Cell cell = Cell::open(lines(2, 1.0, nine), nine, o);  // listed: all 18 taken
        CHECK_EQ(cell.names().size(), std::size_t(18));
    }
    o.max_axes = 17;
    {
        Cell cell = Cell::open(lines(2, 1.0, nine), {}, o);  // a scan keeps the first 17
        CHECK_EQ(cell.names().size(), std::size_t(17));
    }
    CHECK_THROWS(Cell::open(lines(2, 1.0, nine), nine, o), std::invalid_argument);
    o.max_axes = 0;
    CHECK_THROWS(Cell::open(lines(1, 1.0), {}, o), std::invalid_argument);
}
