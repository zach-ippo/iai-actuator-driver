// Power-loss / link-loss recovery (recovery.hpp; Cell track_all / check_links / monitor_links / recover_all) and the
// simulator's faults (power_off / power_on / power_cycle, unplug / replug / reopen, a replaced unit). Port of the
// Python edition's tests/test_recovery.py and the recovery parts of tests/test_fast_recovery.py. Simulator only: no
// COM port is opened.
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
namespace rc = iai_actuator_driver::recovery;

namespace {

using Frame = std::vector<std::uint8_t>;

/// The request frames an axis sent (its bus's frame logger).
struct Recorder {
    std::mutex mutex;
    std::vector<Frame> tx;
    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        tx.clear();
    }
    std::vector<Frame> frames() {
        std::lock_guard<std::mutex> lock(mutex);
        return tx;
    }
};

std::shared_ptr<SimulatedUnit> sim_unit(SimProfile profile, double time_scale = 50.0,
                                        std::optional<int> baud_rate = std::nullopt,
                                        std::function<void(SimulatedUnitOptions&)> more = {}) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = time_scale;
    o.baud_rate = baud_rate;
    if (more) more(o);
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_axis(std::shared_ptr<Transport> transport, const std::shared_ptr<Recorder>& rec = nullptr) {
    OpenOptions o;
    o.axis.poll_interval_s = 0.005;
    o.bus.min_gap_s = 0.0;
    if (rec)
        o.bus.frame_logger = [rec](FrameDirection d, const Frame& f) {
            if (d != FrameDirection::Tx) return;
            std::lock_guard<std::mutex> lock(rec->mutex);
            rec->tx.push_back(f);
        };
    Axis ax = Axis::open(std::move(transport), o);
    ax.connect();
    return ax;
}

std::shared_ptr<ModbusRtu> plain_bus(std::shared_ptr<Transport> transport, int retries = 3) {
    BusOptions o;
    o.min_gap_s = 0.0;
    o.retries = retries;
    return std::make_shared<ModbusRtu>(std::move(transport), o);
}

rc::RecoveryPolicy quick() {
    rc::RecoveryPolicy p;
    p.poll_s = 0.02;
    return p;
}

rc::RecoverOptions ropts(double timeout_s, rc::RecoveryPolicy policy = quick(), rc::EventCallback on_event = {}) {
    rc::RecoverOptions o;
    o.timeout_s = timeout_s;
    o.policy = std::move(policy);
    o.on_event = std::move(on_event);
    return o;
}

struct Events {
    std::mutex mutex;
    std::vector<rc::RecoveryEvent> items;
    rc::EventCallback callback() {
        return [this](const rc::RecoveryEvent& e) {
            std::lock_guard<std::mutex> lock(mutex);
            items.push_back(e);
        };
    }
    std::vector<std::string> kinds() {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::string> out;
        for (const auto& e : items) out.push_back(e.kind);
        return out;
    }
};

using Strings = std::vector<std::string>;

bool any_contains(const Strings& texts, const std::string& needle) {
    for (const auto& t : texts)
        if (t.find(needle) != std::string::npos) return true;
    return false;
}

bool has(const std::string& text, const std::string& needle) { return text.find(needle) != std::string::npos; }

bool wait_until(const std::function<bool()>& condition, double timeout_s = 3.0) {
    const double end = now_s() + timeout_s;
    while (now_s() < end) {
        if (condition()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
}

int be16(const Frame& f, std::size_t at) { return (f[at] << 8) | f[at + 1]; }

/// Every frame that could start motion or switch the servo on: the 0x9900 / 0x0F00 blocks, a 0x9800 row number, the
/// HOME / SON / CSTR / PCRF coils, and the IAI motion / servo / home commands.
std::vector<Frame> motion_frames(const std::vector<Frame>& frames) {
    const std::set<int> coils{reg::kCoilHome, reg::kCoilSon, 0x040C, 0x042E};
    const std::set<int> registers{0x9900, 0x9800, 0x0F00, 0x0D00, 0x9801};
    std::vector<Frame> bad;
    for (const auto& f : frames) {
        if (f.size() < 5) continue;
        const int fc = f[1];
        const int address = be16(f, 2);
        if (fc == 0x05 && coils.count(address) && f[4] == 0xFF) bad.push_back(f);
        else if ((fc == 0x06 || fc == 0x10 || fc == 0x17) && registers.count(address)) bad.push_back(f);
        else if (fc == 0x45 && f[0] != 0 && (address == 0x06 || address == 0x07 || address == 0x09 || address == 0x0A))
            bad.push_back(f);
    }
    return bad;
}

std::set<int> function_codes(const std::vector<Frame>& frames) {
    std::set<int> out;
    for (const auto& f : frames)
        if (f.size() > 1) out.insert(f[1]);
    return out;
}

std::uint32_t read_uptime(ModbusRtu& bus) {
    const auto w = bus.read_registers(1, reg::kStim, 2);
    return (static_cast<std::uint32_t>(w[0]) << 16) | w[1];
}

/// A backup in the info::backup layout (identity, positions.rows, parameters.classes), read like the Python
/// edition's info.backup(ax, rows=...).
Json make_backup(Axis& ax, const std::vector<int>& rows) {
    Json b = Json::object();
    b.set("identity", to_json(ax.identify()));
    Json list = Json::array();
    for (int n : rows) {
        Json row = Json::object();
        row.set("n", n);
        Json words = Json::array();
        for (std::uint16_t w : ax.read_registers(reg::position_address(n), reg::kPositionStride)) {
            char hex[8];
            std::snprintf(hex, sizeof hex, "%04X", w);
            words.push_back(std::string(hex));
        }
        row.set("words", std::move(words));
        list.push_back(std::move(row));
    }
    Json positions = Json::object();
    positions.set("rows", std::move(list));
    b.set("positions", std::move(positions));
    auto reader = ax.bus()->reader(ax.slave());
    const UnitDirectory dir = UnitDirectory::read(*reader);
    Json classes = Json::array();
    for (int cid : rc::kParameterClasses) {
        if (!dir.find(cid)) continue;
        const ClassBlock block = dir.read_class(*reader, cid);
        Json c = Json::object();
        c.set("class_id", cid);
        Json items = Json::array();
        for (const auto& item : block.items) {
            Json one = Json::object();
            one.set("key", item.key());
            one.set("hex", item.hex());
            items.push_back(std::move(one));
        }
        c.set("items", std::move(items));
        classes.push_back(std::move(c));
    }
    Json parameters = Json::object();
    parameters.set("classes", std::move(classes));
    b.set("parameters", std::move(parameters));
    return b;
}

Json with_serial(const Json& backup, const std::string& serial) {
    Json copy = Json::parse(backup.dump());
    Json identity = *copy.find("identity");
    identity.set("serial_number", serial);
    copy.set("identity", identity);
    return copy;
}

/// A transport that only has a serial-port name (like a SerialTransport), for port_present(); it cannot be reopened.
class NamedPort : public Transport {
public:
    explicit NamedPort(std::string name) : name_(std::move(name)) {}
    void write(const std::uint8_t*, std::size_t) override { throw TransportError("gone"); }
    std::size_t read(std::uint8_t*, std::size_t) override { return 0; }
    void reset_input() override {}
    int baud_rate() const override { return 0; }
    void set_baud_rate(int) override { throw std::logic_error("no baud rate"); }
    double read_timeout() const override { return 0.1; }
    void set_read_timeout(double) override {}
    void send_break(double) override {}
    void close() override {}
    bool is_open() const override { return true; }
    std::string name() const override { return name_; }

private:
    std::string name_;
};

std::vector<SerialPortInfo> ports_named(const Strings& names) {
    std::vector<SerialPortInfo> out;
    for (const auto& n : names) {
        SerialPortInfo p;
        p.port = n;
        out.push_back(p);
    }
    return out;
}

template <class E>
bool rethrows_as(const std::exception_ptr& error) {
    try {
        std::rethrow_exception(error);
    } catch (const E&) {
        return true;
    } catch (...) {
        return false;
    }
}

CellOptions cell_options() {
    CellOptions o;
    o.axis.poll_interval_s = 0.005;
    o.scan_timeout_s = 0.01;
    o.bus.min_gap_s = 0.0;
    return o;
}

std::map<std::string, std::shared_ptr<SimLine>> sim_ports(int n, double time_scale) {
    SimulatedUnitOptions o;
    o.time_scale = time_scale;
    return SimLine::sim_ports(n, {1}, o);
}

MoveOptions no_wait(double speed) {
    MoveOptions m;
    m.speed = speed;
    m.wait = false;
    return m;
}

}  // namespace

// ====================================================================== simulator fault injection

TEST(recovery_sim_power_cycle_resets_baud_pmss_uptime_home_and_logs_power_up) {
    auto sim = sim_unit(SimProfile::Rod, 50.0, 115200);
    auto bus = plain_bus(sim);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const std::uint32_t before = read_uptime(*bus);
    sim->power_cycle();
    CHECK_EQ(sim->unit_baud_rate().value_or(0), 38400);
    CHECK(!sim->pmsl());
    CHECK(!sim->homed());
    CHECK(!sim->safety_speed());
    bus->set_baud_rate(38400);
    CHECK(read_uptime(*bus) < before);
    CHECK_EQ(bus->read_registers(1, reg::kAlarmHistoryBase, 6)[3], reg::kAlarmPowerUp);
    const Strings events = sim->events();
    CHECK(std::find(events.begin(), events.end(), "HEND lost at power-up") != events.end());
}

TEST(recovery_sim_absolute_units_keep_home_and_lose_home_can_be_forced_either_way) {
    auto gripper = sim_unit(SimProfile::Gripper);
    gripper->power_cycle();
    CHECK(gripper->homed());  // WA absolute
    gripper->power_cycle(true);
    CHECK(!gripper->homed());
    auto rod = sim_unit(SimProfile::Rod);
    rod->power_cycle(false);  // one power-up of the EC-R6S rod still read HEND 1
    CHECK(rod->homed());
}

TEST(recovery_sim_power_off_is_silent_until_power_on) {
    auto sim = sim_unit(SimProfile::Slider);
    auto bus = plain_bus(sim, 0);
    bus->write_coil(1, reg::kCoilPmsl, true);
    sim->power_off();
    CHECK(!sim->powered());
    CHECK_THROWS(bus->read_registers(1, reg::kPnow, 2), ModbusTimeout);
    sim->power_on();
    CHECK_EQ(bus->read_registers(1, reg::kDsse, 1)[0] & reg::kDssePmss, 0);  // PMSS 0 after power-up
}

TEST(recovery_sim_power_off_stops_a_move_where_it_is) {
    auto sim = sim_unit(SimProfile::Slider, 5.0);
    Axis ax = open_axis(sim);
    ax.move_to(140.0, no_wait(100.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sim->power_off();
    const double held = sim->position_mm();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK_EQ(sim->position_mm(), held);
    CHECK(!sim->moving());
    sim->power_on();
    CHECK_EQ(sim->position_mm(), held);  // absolute slider: position kept
}

TEST(recovery_sim_unplug_kills_the_handle_until_replug_and_reopen) {
    auto sim = sim_unit(SimProfile::Slider);
    auto bus = plain_bus(sim);
    sim->unplug();
    CHECK(sim->present() == std::optional<bool>(false));
    CHECK_THROWS(bus->read_registers(1, reg::kPnow, 2), TransportError);
    CHECK_THROWS(sim->reopen(), TransportError);  // still missing
    sim->replug();
    CHECK_THROWS(bus->read_registers(1, reg::kPnow, 2), TransportError);  // the old handle stays dead
    CHECK(bus->can_reopen());
    bus->reopen();
    CHECK_EQ(bus->read_registers(1, reg::kPnow, 2).size(), std::size_t(2));
    CHECK_EQ(sim->reopens(), 1);
    CHECK(sim->pmsl());  // the unit kept its power and state
}

TEST(recovery_sim_line_unplug_replug_reopen_and_power_cycle) {
    SimulatedUnitOptions o;
    o.time_scale = 50.0;
    auto line = SimLine::with_slaves({1, 2}, o, "L");
    auto bus = plain_bus(line);
    line->unplug();
    CHECK_THROWS(bus->read_registers(2, reg::kPnow, 2), TransportError);
    CHECK_THROWS(bus->reopen(), TransportError);
    line->replug();
    bus->reopen();
    CHECK_EQ(bus->read_registers(2, reg::kPnow, 2).size(), std::size_t(2));
    line->power_cycle({2});
    CHECK(line->unit(1)->pmsl());
    CHECK(!line->unit(2)->pmsl());
    bus->close();
    CHECK(!line->is_open());
    bus->reopen();  // a closed line opens again, its units too
    CHECK_EQ(bus->read_registers(1, reg::kPnow, 2).size(), std::size_t(2));
}

TEST(recovery_sim_gateway_line_unplug) {
    auto line = sim_rcon_line({1});
    auto bus = plain_bus(line);
    line->unplug();
    CHECK_THROWS(bus->read_registers(1, reg::kPnow, 2), TransportError);
    line->replug();
    bus->reopen();
    CHECK_EQ(bus->read_registers(1, reg::kPnow, 2).size(), std::size_t(2));
}

TEST(recovery_sim_serial_number_override) {
    auto sim = sim_unit(SimProfile::Gripper, 50.0, std::nullopt,
                        [](SimulatedUnitOptions& o) { o.serial_number = "AB-123"; });
    CHECK_EQ(sim->serial_number(), std::string("AB-123"));
    Axis ax = open_axis(sim);
    const DeviceInfo info = ax.identify();
    CHECK(info.serial_number == std::optional<std::string>("AB-123"));
    CHECK(info.model_name == std::optional<std::string>("EC-GRBP8M-20-1-1-PN-WA-WL2"));  // the rest is unchanged
}

TEST(recovery_sim_pcon_raises_0x069_again_at_power_up) {
    auto sim = sim_unit(SimProfile::Pcon);
    CHECK_EQ(sim->alarm_code(), reg::kAlarmRtcStop);
    sim->power_cycle();
    CHECK_EQ(sim->alarm_code(), reg::kAlarmRtcStop);
}

// ====================================================================== bus reopen

TEST(recovery_reopen_needs_a_reopenable_transport) {
    auto bus = plain_bus(std::make_shared<NamedPort>("COM9"));
    CHECK(!bus->can_reopen());
    CHECK_THROWS(bus->reopen(), std::logic_error);
}

TEST(recovery_a_dead_handle_is_a_transport_error_and_not_retried) {
    auto sim = sim_unit(SimProfile::Slider);
    auto bus = plain_bus(sim, 3);
    sim->unplug();
    const std::size_t before = sim->requests().size();
    CHECK_THROWS(bus->write_coil(1, reg::kCoilStop, true), TransportError);
    CHECK_EQ(sim->requests().size(), before);
}

// ====================================================================== detection

TEST(recovery_classify_failure_kinds) {
    CHECK_EQ(rc::classify_failure(TransportError("transport error: gone")), std::string(rc::kPortVanished));
    CHECK_EQ(rc::classify_failure(std::system_error(std::make_error_code(std::errc::no_such_device))),
             std::string(rc::kPortVanished));  // an OS error on a dead handle
    CHECK_EQ(rc::classify_failure(ModbusTimeout("no answer")), std::string(rc::kUnitSilent));
    CHECK_EQ(rc::classify_failure(ModbusCrcError("bad crc")), std::string(rc::kUnitSilent));
    CHECK_EQ(rc::classify_failure(ModbusSlaveError(2, 3)), std::string());
    CHECK_EQ(rc::classify_failure(SoftLimitError("too far")), std::string());
    CHECK_EQ(rc::classify_failure(EcAlarmError(0x0C0)), std::string());
    CHECK_EQ(rc::classify_failure(MotionTimeoutError("slow")), std::string());
    CHECK_EQ(rc::classify_failure(TransportError("bus is closed")), std::string());  // close() was called
    CHECK_EQ(rc::classify_failure(std::runtime_error("other")), std::string());
    CHECK_EQ(rc::classify_failure(rc::UnitReplacedError("x", {}, {})), std::string(rc::kUnitReplaced));
    CHECK_EQ(rc::classify_failure(std::exception_ptr()), std::string());
}

TEST(recovery_a_silent_unit_on_a_port_no_longer_listed_is_a_vanished_port) {
    Axis ax(plain_bus(std::make_shared<NamedPort>("COM7")));
    const auto listed = [] { return ports_named({"COM3", "com7"}); };
    const auto missing = [] { return ports_named({"COM3"}); };
    CHECK(rc::port_present(ax, listed) == std::optional<bool>(true));
    CHECK_EQ(rc::classify_failure(ModbusTimeout("x"), &ax, listed), std::string(rc::kUnitSilent));
    CHECK(rc::port_present(ax, missing) == std::optional<bool>(false));
    CHECK_EQ(rc::classify_failure(ModbusTimeout("x"), &ax, missing), std::string(rc::kPortVanished));
    CHECK(!rc::port_present(Axis(plain_bus(std::make_shared<FakeTransport>()))).has_value());
    CHECK_EQ(rc::port_name(ax), std::string("COM7"));
}

TEST(recovery_new_alarm_records) {
    const rc::AlarmKey pu{reg::kAlarmPowerUp, 0, 0xFFFF, std::nullopt};
    const rc::AlarmKey empty{0, 0, 0xFFFF, std::nullopt};
    const std::vector<rc::AlarmKey> before{pu, {0x0EE, 3, 0xFFFF, 900}, empty, empty};
    const std::vector<rc::AlarmKey> after{pu, pu, {0x0EE, 3, 0xFFFF, 900}, empty};
    const auto fresh = rc::new_alarm_records(before, after);
    REQUIRE(fresh.size() == 1);
    CHECK(fresh[0] == pu);
    CHECK(rc::new_alarm_records(before, before).empty());
    CHECK(rc::new_alarm_records(std::nullopt, after).empty());
}

// ====================================================================== check_link

namespace {

struct CheckLinkFixture {
    std::shared_ptr<SimulatedUnit> sim = sim_unit(SimProfile::Gripper);
    std::shared_ptr<Recorder> rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    std::shared_ptr<rc::Session> session;
    Events events;

    CheckLinkFixture() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        rc::TrackOptions t;
        t.name = "G";
        session = rc::track(ax, t);
    }

    rc::LinkCheck check() {
        rc::CheckLinkOptions o;
        o.on_event = events.callback();
        return rc::check_link(ax, o);
    }
};

}  // namespace

TEST(recovery_check_link_healthy_link_moves_the_baseline_forward) {
    CheckLinkFixture f;
    const auto first = f.session->uptime_ms;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    f.rec->clear();
    const rc::LinkCheck chk = f.check();
    CHECK(chk.ok());
    CHECK(chk.reasons.empty());
    CHECK(f.session->uptime_ms.value_or(0) > first.value_or(0));
    CHECK(f.events.kinds().empty());
    const auto frames = f.rec->frames();
    REQUIRE(frames.size() == 1);
    CHECK_EQ(frames[0][1], 0x03);  // one status read, nothing written
}

TEST(recovery_check_link_power_cycle_is_a_restart) {
    CheckLinkFixture f;
    f.sim->power_cycle();
    const rc::LinkCheck chk = f.check();
    CHECK_EQ(chk.state, std::string(rc::kUnitRestarted));
    CHECK(any_contains(chk.reasons, "uptime counter dropped"));
    CHECK(any_contains(chk.reasons, "PMSS"));
    f.check();
    CHECK(f.events.kinds() == Strings{"unit_restarted"});  // once per change
    CHECK_EQ(rc::link_state(f.ax), std::string(rc::kUnitRestarted));
}

TEST(recovery_check_link_power_off_is_a_silent_unit_and_unplug_a_vanished_port) {
    CheckLinkFixture f;
    f.sim->power_off();
    rc::LinkCheck chk = f.check();
    CHECK_EQ(chk.state, std::string(rc::kUnitSilent));
    CHECK(rethrows_as<ModbusTimeout>(chk.error));
    f.check();
    CHECK(f.events.kinds() == Strings{"link_lost"});
    f.sim->power_on();
    f.sim->unplug();
    chk = f.check();
    CHECK_EQ(chk.state, std::string(rc::kPortVanished));
    CHECK(f.events.kinds() == Strings{"link_lost"});  // still lost: no new event
}

TEST(recovery_check_link_uptime_behind_the_pc_clock_is_a_restart) {
    CheckLinkFixture f;
    f.session->uptime_at = now_s() - 3600.0;  // the PC was away for an hour; the unit shows less
    const rc::LinkCheck chk = f.check();
    CHECK_EQ(chk.state, std::string(rc::kUnitRestarted));
    REQUIRE(!chk.reasons.empty());
    CHECK(has(chk.reasons[0], "behind"));
}

TEST(recovery_check_link_a_deliberate_control_change_noted_is_not_a_restart) {
    CheckLinkFixture f;
    f.ax.disable_modbus_control();  // the axis notes its own deliberate change in the session
    CHECK_EQ(f.check().state, std::string(rc::kLinkOk));
    f.ax.enable_modbus_control();
    CHECK_EQ(f.check().state, std::string(rc::kLinkOk));
    f.ax.bus()->write_coil(f.ax.slave(), reg::kCoilPmsl, false);  // dropped behind the axis's back
    CHECK_EQ(f.check().state, std::string(rc::kUnitRestarted));    // PMSS dropped while the PC held it
    rc::set_link_state(f.ax, rc::kLinkOk);
    rc::ControlChange change;
    change.modbus_control = false;
    rc::note_control(f.ax, change);
    CHECK_EQ(f.check().state, std::string(rc::kLinkOk));
}

TEST(recovery_check_link_home_in_progress_is_not_a_lost_home) {
    auto sim = sim_unit(SimProfile::Gripper, 1.0);
    Axis ax = open_axis(sim);
    const auto sess = rc::track(ax);
    ax.home(false);
    const Status st = ax.status();
    CHECK(st.homing() && !st.homed);
    CHECK(rc::restart_signs(*sess, st).empty());
    ax.stop(true);
}

TEST(recovery_check_link_recovering_and_untracked) {
    CheckLinkFixture f;
    f.session->recovering = true;
    CHECK_EQ(f.check().state, std::string(rc::kRecovering));
    f.session->recovering = false;
    auto other = sim_unit(SimProfile::Slider);
    Axis ax = open_axis(other);
    CHECK(rc::check_link(ax).ok());  // no session: the link only
    other->power_off();
    CHECK_EQ(rc::check_link(ax).state, std::string(rc::kUnitSilent));
}

TEST(recovery_link_monitor_reports_changes) {
    CheckLinkFixture f;
    std::mutex mutex;
    std::vector<std::pair<std::string, std::string>> changes;
    rc::LinkMonitor::Options o;
    o.interval_s = 0.01;
    o.on_change = [&](const std::string& n, const rc::LinkCheck& c) {
        std::lock_guard<std::mutex> lock(mutex);
        changes.emplace_back(n, c.state);
    };
    auto count = [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return changes.size();
    };
    {
        rc::LinkMonitor monitor({{"G", &f.ax}}, o);
        monitor.start();
        CHECK(monitor.running());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        f.sim->power_off();
        CHECK(wait_until([&] { return count() >= 1; }));
        f.sim->power_on();
        CHECK(wait_until([&] { return count() >= 2; }));
        monitor.stop();
        CHECK(!monitor.running());
        CHECK(monitor.passes() > 2);
    }
    REQUIRE(changes.size() >= 2);
    CHECK(changes[0] == std::make_pair(std::string("G"), std::string(rc::kUnitSilent)));
    CHECK(changes[1] == std::make_pair(std::string("G"), std::string(rc::kUnitRestarted)));
    CHECK_THROWS(rc::LinkMonitor(f.ax, [] {
                     rc::LinkMonitor::Options bad;
                     bad.interval_s = 0.0;
                     return bad;
                 }()),
                 std::invalid_argument);
}

// ====================================================================== recover()

TEST(recovery_power_loss_restores_baud_control_and_safety_and_never_moves) {
    auto sim = sim_unit(SimProfile::Rod, 50.0, 115200);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    ax.bus()->write_coil(ax.slave(), reg::kCoilSfty, true);  // the application's safety-speed choice
    MoveOptions mo;
    mo.speed = 100.0;
    ax.move_to(80.0, mo);
    rc::note_target(ax, 80.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    rc::TrackOptions t;
    t.name = "ROD";
    rc::track(ax, t);
    sim->power_cycle();
    Events events;
    rec->clear();
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0, quick(), events.callback()));
    CHECK_EQ(report.cause, std::string(rc::kUnitSilent));  // 115200 frames are not understood at 38400
    CHECK(report.restarted && report.same_unit);
    for (const char* needle : {"uptime counter dropped", "power-up record", "PMSS", "HEND", "fell back to 38400"})
        CHECK(any_contains(report.restart_signs, needle));
    CHECK(report.baud_rate_found == std::optional<int>(38400));
    CHECK(report.baud_rate == std::optional<int>(115200));
    CHECK_EQ(sim->unit_baud_rate().value_or(0), 115200);
    CHECK(report.baud_restored && report.control_restored && report.safety_speed_restored);
    CHECK(!report.control_kept);  // lost at the power-up: taken again, not kept
    CHECK(sim->pmsl() && sim->safety_speed());
    CHECK(report.needs_home && report.needs_attention());
    CHECK(!report.ready_to_move());
    CHECK(report.at_target == std::optional<bool>(true));
    CHECK(report.position_error_mm == std::optional<double>(0.0));
    CHECK(events.kinds() == (Strings{"link_lost", "unit_restarted", "recovered"}));
    CHECK(motion_frames(rec->frames()).empty());
    CHECK(!sim->moving());
    CHECK(rc::check_link(ax).ok());  // new baseline
    CHECK(has(report.summary(), "needs home"));
    const Json j = report.to_json();
    REQUIRE(j.find("needs_attention") != nullptr);
    CHECK(j.find("needs_attention")->as_bool());
}

TEST(recovery_waits_for_the_power_to_come_back) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    rc::track(ax);
    sim->power_off();
    ec_test::Thread later([sim] {  // joined even when the recovery throws
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        sim->power_on();
    });
    const double started = now_s();
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0));
    later.join();
    CHECK(now_s() - started >= 0.25);
    CHECK_EQ(report.cause, std::string(rc::kUnitSilent));
    CHECK(report.restarted && report.control_restored);
    CHECK(!report.needs_home);  // WA absolute
}

TEST(recovery_power_cycle_after_disable_then_enable_control_takes_pc_control_back) {
    // disable_modbus_control() notes "no PC control" in the session; enable_modbus_control() must note it again, or a recover()
    // after a power cycle leaves the unit in PIO mode (no check_link ran between the enable and the power cycle)
    for (const bool verify : {true, false}) {
        auto sim = sim_unit(SimProfile::Gripper);
        Axis ax = open_axis(sim);
        rc::track(ax);
        ax.disable_modbus_control();
        CHECK(!sim->pmsl());
        ax.enable_modbus_control(verify);
        CHECK(sim->pmsl());
        sim->power_cycle();  // PMSS = 0 after the power-up
        const rc::RecoveryReport report = rc::recover(ax, ropts(5.0, quick()));
        CHECK(report.restarted);
        CHECK(report.control_restored);
        CHECK(!report.control_kept);
        CHECK(sim->pmsl());
    }
}

TEST(recovery_recover_after_disable_then_enable_control_without_a_restart_keeps_pc_control) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    rc::track(ax);
    ax.disable_modbus_control();
    ax.enable_modbus_control();
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0, quick()));
    CHECK(report.control_kept);
    CHECK(!report.control_restored);
    CHECK(sim->pmsl());
}

TEST(recovery_port_vanished_and_back_reopens_in_place) {
    auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{sim_unit(SimProfile::Gripper)},
                                          "L1");
    Axis ax = open_axis(line);
    rc::track(ax);
    line->unplug();
    ec_test::Thread later([line] {  // joined even when the recovery throws
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        line->replug();
    });
    Events events;
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0, quick(), events.callback()));
    later.join();
    CHECK_EQ(report.cause, std::string(rc::kPortVanished));
    CHECK(report.port_reopened);
    CHECK(!report.restarted);  // the unit kept its power
    CHECK(report.actions == Strings{"port L1 reopened"});
    CHECK(any_contains(report.notes, "waited for port L1 to be listed again"));
    CHECK_EQ(line->reopens(), 1);
    CHECK(events.kinds() == (Strings{"link_lost", "recovered"}));
}

TEST(recovery_a_port_that_never_comes_back_times_out) {
    auto sim = sim_unit(SimProfile::Slider);
    Axis ax = open_axis(sim);
    rc::track(ax);
    sim->unplug();
    try {
        rc::recover(ax, ropts(0.2));
        CHECK(false);
    } catch (const rc::RecoveryError& e) {
        CHECK(has(e.what(), "did not come back within 0.2 s"));
        REQUIRE(e.report() != nullptr);
        CHECK_EQ(e.report()->cause, std::string(rc::kPortVanished));
    }
    CHECK_EQ(rc::link_state(ax), std::string(rc::kPortVanished));
}

TEST(recovery_a_replaced_unit_is_refused_and_nothing_is_written) {
    for (const std::string field : {"serial_number", "model_name"}) {
        auto gripper = sim_unit(SimProfile::Gripper);
        auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{gripper}, "L1");
        Axis ax = open_axis(line);
        rc::track(ax);
        line->unplug();
        line->remove(1);
        std::shared_ptr<SimulatedUnit> replacement;
        if (field == "serial_number") {
            replacement = sim_unit(SimProfile::Gripper, 50.0, std::nullopt,
                                   [](SimulatedUnitOptions& o) { o.serial_number = "SIM-0000099"; });
        } else {
            const std::string same_serial = gripper->serial_number();
            replacement = sim_unit(SimProfile::Slider, 50.0, std::nullopt,
                                   [same_serial](SimulatedUnitOptions& o) { o.serial_number = same_serial; });
        }
        line->add(replacement);
        line->replug();
        Events events;
        try {
            rc::recover(ax, ropts(2.0, quick(), events.callback()));
            CHECK(false);
        } catch (const rc::UnitReplacedError& e) {
            CHECK(has(e.what(), field));
            CHECK(has(e.what(), "Nothing was written"));
            if (field == "serial_number") CHECK(e.expected().serial_number != e.found().serial_number);
            else CHECK(e.expected().model_name != e.found().model_name);
            REQUIRE(e.report() != nullptr);
            CHECK(!e.report()->same_unit);
            CHECK_EQ(rc::classify_failure(e), std::string(rc::kUnitReplaced));
        }
        std::set<int> codes;
        for (const auto& frame : replacement->requests()) codes.insert(frame[1]);
        for (int fc : codes) CHECK(fc == 0x03 || fc == 0x11);  // read-only
        CHECK(events.kinds() == (Strings{"link_lost", "unit_replaced"}));
        CHECK_EQ(rc::link_state(ax), std::string(rc::kUnitReplaced));
    }
}

TEST(recovery_needs_a_session_and_an_open_bus) {
    auto sim = sim_unit(SimProfile::Slider);
    Axis ax = open_axis(sim);
    rc::forget(ax);
    try {
        rc::recover(ax);
        CHECK(false);
    } catch (const rc::RecoveryError& e) {
        CHECK(has(e.what(), "track(axis)"));
    }
    rc::track(ax);
    ax.close();
    CHECK_THROWS(rc::recover(ax), rc::RecoveryError);
}

TEST(recovery_healthy_link_reports_nothing_to_do) {
    auto sim = sim_unit(SimProfile::Gripper);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    rc::track(ax);
    Events events;
    rec->clear();
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0, quick(), events.callback()));
    CHECK_EQ(report.cause, std::string(rc::kLinkOk));
    CHECK(!(report.restarted || report.port_reopened || report.needs_attention()));
    CHECK(report.actions.empty());
    CHECK(report.ready_to_move());
    CHECK(events.kinds().empty());
    CHECK(function_codes(rec->frames()) == (std::set<int>{0x03, 0x11}));
}

TEST(recovery_pcon_message_alarm_is_reset_and_comes_back) {
    auto sim = sim_unit(SimProfile::Pcon);
    Axis ax = open_axis(sim);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    rc::track(ax);
    sim->power_cycle();
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0));
    CHECK_EQ(report.cause, std::string(rc::kUnitRestarted));
    CHECK(report.alarms_cleared == std::vector<int>{reg::kAlarmRtcStop});
    CHECK_EQ(report.alarm_code, reg::kAlarmRtcStop);
    CHECK(!report.fault_alarm());
    CHECK(any_contains(report.notes, "came back after the reset"));
    bool seen = false;
    for (const auto& r : report.new_alarm_records) seen = seen || r.code == reg::kAlarmRtcStop;
    CHECK(seen);
}

TEST(recovery_a_fault_alarm_is_left_for_the_operator) {
    auto sim = sim_unit(SimProfile::Gripper);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    rc::track(ax);
    sim->trigger_alarm(0x0C0);
    rec->clear();
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0));
    CHECK_EQ(report.alarm_code, 0x0C0);
    CHECK(report.alarms_cleared.empty());
    CHECK(report.fault_alarm() && report.needs_attention());
    CHECK(any_contains(report.notes, "left for the operator"));
    for (const auto& f : rec->frames()) CHECK(!(f[1] == 0x05 && be16(f, 2) == reg::kCoilAlrs));
}

TEST(recovery_a_move_cut_off_is_reported_not_resumed) {
    auto sim = sim_unit(SimProfile::Slider, 5.0);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    rc::track(ax);
    ax.move_to(140.0, no_wait(100.0));  // the axis notes its target itself
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sim->power_cycle();
    rec->clear();
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0));
    CHECK(report.at_target == std::optional<bool>(false));
    CHECK(report.position_error_mm.value_or(0.0) < -1.0);
    CHECK(any_contains(report.notes, "NOT resumed"));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(!sim->moving());
    CHECK(motion_frames(rec->frames()).empty());
}

TEST(recovery_integrity_of_rows_and_parameters) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    ParameterWriteOptions ram;
    ram.commit = false;
    ax.write_parameter(2, 0.30, ram);  // RAM only: lost at power-off
    rc::TrackOptions t;
    t.rows = rc::RowSelection::every();
    t.parameters = true;
    rc::track(ax, t);
    std::vector<std::uint16_t> row1 = sim->position_row(1);
    row1[reg::kPosChecksum] ^= 0x55;  // a damaged checksum in row 1
    sim->set_position_row(1, row1);
    sim->power_cycle();
    rc::RecoveryPolicy policy = quick();
    policy.check_rows = rc::RowSelection::every();
    policy.check_parameters = true;
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0, policy));
    std::vector<int> all;
    for (int n = 0; n < ax.max_positions(); ++n) all.push_back(n);
    CHECK(report.rows_checked == all);
    bool checksum = false;
    for (const auto& p : report.row_problems) checksum = checksum || p.rfind("row 1: checksum", 0) == 0;
    CHECK(checksum);
    CHECK(report.parameters_checked > 50);
    REQUIRE(report.parameter_differences.size() == 1);
    CHECK(has(report.parameter_differences[0], "8.9"));
    CHECK(report.needs_attention());
}

TEST(recovery_integrity_against_a_backup) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    ParameterWriteOptions ram;
    ram.commit = false;
    ax.write_parameter(2, 0.30, ram);
    const Json saved = make_backup(ax, {0, 1, 2});
    rc::track(ax);
    sim->power_cycle();
    rc::RecoveryPolicy policy = quick();
    policy.check_rows = rc::RowSelection::every();
    policy.check_parameters = true;
    policy.backup = saved;
    rc::RecoveryReport report = rc::recover(ax, ropts(2.0, policy));
    CHECK(report.row_problems.empty());
    REQUIRE(report.parameter_differences.size() == 1);
    {
        const std::string& d = report.parameter_differences[0];  // "0x0008:8.9: <before> -> <after>"
        const std::size_t a = d.find(':');
        CHECK_EQ(d.substr(a + 1, d.find(':', a + 1) - a - 1), std::string("8.9"));
    }
    rc::RecoveryPolicy other = quick();
    other.check_parameters = true;
    other.backup = with_serial(saved, "ELSEWHERE");
    report = rc::recover(ax, ropts(2.0, other));
    CHECK(any_contains(report.notes, "another unit"));
    CHECK(report.parameter_differences.empty());
}

TEST(recovery_policy_can_leave_the_session_alone) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    ax.bus()->write_coil(ax.slave(), reg::kCoilSfty, true);
    rc::track(ax);
    sim->power_cycle();
    rc::RecoveryPolicy policy = quick();
    policy.restore_control = false;
    policy.restore_safety_speed = false;
    rc::RecoveryReport report = rc::recover(ax, ropts(2.0, policy));
    CHECK(!(report.control_restored || report.safety_speed_restored));
    CHECK(report.actions.empty());
    CHECK(!(sim->pmsl() || sim->safety_speed()));
    CHECK(rc::check_link(ax).ok());  // the new baseline is what the unit has now
    ax.enable_modbus_control();
    CHECK(rc::check_link(ax).ok());  // a healthy check moves the baseline forward: PC control held again
    rc::ControlChange change;
    change.safety_speed = true;  // the application's choice, restored next time
    rc::note_control(ax, change);
    sim->power_cycle();
    report = rc::recover(ax, ropts(2.0));
    CHECK(report.control_restored && report.safety_speed_restored);
    CHECK(sim->pmsl() && sim->safety_speed());
}

TEST(recovery_an_unverified_rate_is_not_restored) {
    auto sim = sim_unit(SimProfile::Gripper, 50.0, 57600);
    Axis ax = open_axis(sim);
    rc::track(ax);
    sim->power_cycle();
    const rc::RecoveryReport report = rc::recover(ax, ropts(3.0));
    CHECK(report.baud_rate_found == std::optional<int>(38400));
    CHECK(report.baud_rate == std::optional<int>(38400));
    CHECK(!report.baud_restored);
    CHECK(any_contains(report.notes, "no verified baud-change code"));
    CHECK_EQ(std::count_if(report.notes.begin(), report.notes.end(),
                           [](const std::string& n) { return n.find("no verified baud-change code") != std::string::npos; }),
             std::ptrdiff_t(1));  // tried once: the quick path's try is not repeated
}

TEST(recovery_line_break_back_to_38400_before_the_full_identify) {
    // the gripper takes code 0x06 (VERIFIED in the Python edition)
    auto sim = sim_unit(SimProfile::Gripper, 50.0, 38400, [](SimulatedUnitOptions& o) { o.ignored_baud_codes = {}; });
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    rc::track(ax);
    ax.bus()->send_break(0.25);  // the unit drops to 9600
    rec->clear();
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0));
    CHECK_EQ(report.cause, std::string(rc::kUnitSilent));
    CHECK(!report.restarted);
    CHECK(report.same_unit);
    CHECK(report.baud_rate_found == std::optional<int>(9600));
    CHECK(report.baud_rate == std::optional<int>(38400));
    CHECK(report.baud_restored);
    CHECK(report.control_kept && !report.control_restored);  // the unit kept PMSS: nothing taken again
    CHECK(report.to_json().find("control_kept")->as_bool());
    const auto frames = rec->frames();
    std::size_t baud = frames.size(), directory = frames.size();
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const Frame& f = frames[i];
        if (baud == frames.size() && f.size() >= 4 && f[0] == 0x00 && f[1] == 0x45 && f[2] == 0x00 && f[3] == 0x04)
            baud = i;
        if (directory == frames.size() && f.size() >= 4 && f[1] == 0x03 && f[2] == 0x00 && f[3] == 0x00) directory = i;
    }
    CHECK(baud < frames.size());
    CHECK(directory < frames.size());  // identify() reads the directory again
    CHECK(baud < directory);  // the many identify() reads run at 38400, not 9600
}

TEST(recovery_a_unit_that_ignores_the_38400_code_stays_reachable) {
    auto sim = sim_unit(SimProfile::Gripper, 50.0, 38400);  // ignores 0x06, as the EC-R6S rod did
    Axis ax = open_axis(sim);
    rc::track(ax);
    ax.bus()->send_break(0.25);
    const rc::RecoveryReport report = rc::recover(ax, ropts(5.0));
    CHECK(report.same_unit && report.ready_to_move());
    CHECK(report.baud_rate_found == std::optional<int>(9600));
    CHECK(report.baud_rate == std::optional<int>(9600));
    CHECK(!report.baud_restored);
    CHECK(any_contains(report.notes, "not restored"));
}

TEST(recovery_session_from_info_without_a_baseline) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    rc::forget(ax);
    const auto sess = rc::session_from_info(ax, ax.identify(), "G");
    CHECK(rc::session_of(ax) == sess);
    sim->power_cycle();
    const rc::RecoveryReport report = rc::recover(ax, ropts(2.0));
    CHECK(report.same_unit);
    CHECK(!report.control_restored);
    CHECK(any_contains(report.notes, "unknown (no track())"));
    CHECK(sess->to_json().type() == Json::Type::Object);
}

// ====================================================================== the Python edition's Axis.recover / track

TEST(recovery_axis_without_track_the_identity_read_at_connect_is_the_baseline) {
    auto sim = sim_unit(SimProfile::Rod);
    Axis ax = open_axis(sim);
    CHECK(rc::session_of(ax) == nullptr);
    REQUIRE(ax.device_info().has_value());
    sim->power_cycle();
    const rc::RecoveryReport report = rc::recover_axis(ax, ropts(5.0));
    CHECK(report.same_unit);
    CHECK(report.needs_home);  // an incremental rod lost HEND
    const auto sess = rc::session_of(ax);
    REQUIRE(sess != nullptr);
    CHECK(sess->serial_number == ax.device_info()->serial_number);
    CHECK_EQ(rc::link_state(ax), std::string(rc::kLinkOk));
}

TEST(recovery_axis_track_and_check_link) {
    auto sim = sim_unit(SimProfile::Gripper);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    rc::TrackOptions t;
    t.name = "G";
    const auto sess = rc::track(ax, t);
    CHECK(rc::session_of(ax) == sess);
    rec->clear();
    CHECK(rc::check_link(ax).ok());
    CHECK(function_codes(rec->frames()) == std::set<int>{0x03});
    CHECK_EQ(rec->frames().size(), std::size_t(1));  // one status read, nothing written
    sim->power_cycle();
    CHECK_EQ(rc::check_link(ax).state, std::string(rc::kUnitRestarted));
    CHECK_EQ(rc::link_state(ax), std::string(rc::kUnitRestarted));
}

TEST(recovery_axis_a_replaced_unit_is_refused) {
    auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{sim_unit(SimProfile::Gripper)},
                                          "L");
    Axis ax = open_axis(line);
    line->remove(1);  // the cable now leads to another unit
    line->add(sim_unit(SimProfile::Gripper, 50.0, std::nullopt,
                       [](SimulatedUnitOptions& o) { o.serial_number = "OTHER-UNIT-0001"; }));
    CHECK_THROWS(rc::recover_axis(ax, ropts(1.0)), rc::UnitReplacedError);
    CHECK_EQ(rc::link_state(ax), std::string(rc::kUnitReplaced));
}

TEST(recovery_axis_a_stopped_home_return_is_not_a_restart) {
    auto sim = sim_unit(SimProfile::Gripper, 1.0);
    Axis ax = open_axis(sim);
    rc::track(ax);
    Events events;
    ax.home(false);
    CHECK(wait_until([&] { return ax.status().homing(); }));
    ax.stop(true);
    CHECK(wait_until([&] {
        const Status st = ax.status();
        return !st.homing() && !st.moving;
    }));
    CHECK(!ax.status().homed);  // HEND 0, not homing: the home return was stopped
    rc::CheckLinkOptions o;
    o.on_event = events.callback();
    for (int i = 0; i < 3; ++i) CHECK(rc::check_link(ax, o).ok());
    CHECK(events.kinds().empty());
    CHECK_EQ(rc::link_state(ax), std::string(rc::kLinkOk));
    const auto sess = rc::session_of(ax);
    REQUIRE(sess != nullptr);
    CHECK(sess->homed == std::optional<bool>(false));  // the home return's loss of HEND is recorded as deliberate
}

TEST(recovery_axis_a_move_still_running_after_a_vanished_port_is_reported_as_running) {
    auto sim = sim_unit(SimProfile::Gripper, 1.0);
    auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{sim}, "L5");
    Axis ax = open_axis(line);
    rc::track(ax);
    const Status st = ax.status();
    const Span stroke = ax.effective_stroke().value_or(Span{0.0, 9.5});
    const double far = (st.position_mm - stroke.min > stroke.max - st.position_mm) ? stroke.min + 0.5 : stroke.max - 0.5;
    ax.move_to(far, no_wait(2.0));
    line->unplug();  // the unit keeps power: the move runs on (no communication watchdog)
    line->replug();
    const rc::RecoveryReport report = rc::recover(ax, ropts(3.0));
    REQUIRE(report.status.has_value());
    CHECK(report.status->moving);
    CHECK(report.needs_attention());
    CHECK(has(report.summary(), "still moving"));
    CHECK(any_contains(report.notes, "still moving"));
    CHECK(!any_contains(report.notes, "cut off"));
    ax.stop(true);
}

TEST(recovery_session_from_backup_the_unit_must_be_the_one_backed_up) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    const Json data = make_backup(ax, {0, 1, 2});
    const auto sess = rc::session_from_backup(ax, data, "G");
    CHECK(sess->serial_number == std::optional<std::string>(data.find("identity")->find("serial_number")->as_string()));
    CHECK(rc::session_of(ax) == sess);
    rc::RecoveryPolicy policy = quick();
    policy.check_rows = rc::RowSelection::every();
    policy.check_parameters = true;
    const rc::RecoveryReport report = rc::recover_axis(ax, ropts(2.0, policy));
    CHECK(report.same_unit);
    CHECK(!report.rows_checked.empty());
    CHECK(report.parameters_checked > 0);
    CHECK(report.row_problems.empty());
    CHECK(report.parameter_differences.empty());
    rc::session_from_backup(ax, with_serial(data, "SOMEONE-ELSE"));
    CHECK_THROWS(rc::recover_axis(ax, ropts(1.0)), rc::UnitReplacedError);
    Json empty = Json::object();
    empty.set("identity", Json::object());
    CHECK_THROWS(rc::session_from_backup(ax, empty), std::invalid_argument);
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "iad_recovery_test_unit.json";
    {
        std::ofstream out(path, std::ios::binary);
        out << data.dump();
    }
    CHECK(rc::session_from_backup_file(ax, path.string())->model_name ==
          std::optional<std::string>(data.find("identity")->find("model_name")->as_string()));
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

// ====================================================================== Cell

TEST(recovery_cell_a_vanished_port_stops_the_others_and_recover_all_brings_it_back) {
    auto ports = sim_ports(3, 2.0);
    std::mutex mutex;
    std::vector<std::pair<std::string, std::string>> faults;
    CellOptions o = cell_options();
    o.on_fault = [&](const std::string& n, const std::string& k, const std::string&) {
        std::lock_guard<std::mutex> lock(mutex);
        faults.emplace_back(n, k);
    };
    Cell cell = Cell::open(ports, {}, o);
    cell.track_all();
    std::map<std::string, double> targets;
    for (const auto& n : cell.names()) targets[n] = 140.0;
    cell.move_all(targets, no_wait(60.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ports.at("SIM2")->unplug();
    try {
        cell.wait_all();
        CHECK(false);
    } catch (const CellError& e) {
        REQUIRE(e.errors().count("SIM2:1") == 1);
        CHECK(rethrows_as<TransportError>(e.errors().at("SIM2:1")));
    }
    CHECK(faults == (std::vector<std::pair<std::string, std::string>>{{"SIM2:1", rc::kPortVanished}}));
    CHECK(cell.faults() == (std::map<std::string, std::string>{{"SIM2:1", rc::kPortVanished}}));
    for (const char* name : {"SIM1", "SIM3"}) {
        CHECK(!ports.at(name)->unit(1)->moving());
        CHECK(ports.at(name)->unit(1)->position_mm() < 139.0);
    }
    ports.at("SIM2")->replug();
    ports.at("SIM3")->power_cycle();
    std::map<std::string, std::size_t> mark;
    for (const auto& kv : ports) mark[kv.first] = kv.second->requests().size();
    const auto reports = cell.recover_all({}, 3.0, quick());
    CHECK(cell.faults().empty());
    CHECK(reports.at("SIM2:1").port_reopened);
    CHECK(reports.at("SIM3:1").restarted);
    CHECK(!reports.at("SIM1:1").restarted);
    CHECK(reports.at("SIM1:1").at_target == std::optional<bool>(false));  // stopped short: reported, not resumed
    const auto events = cell.events();
    REQUIRE(events.size() == 4);
    CHECK(events[0].kind == "link_lost" && events[0].axis == "SIM2:1");
    std::multiset<std::pair<std::string, std::string>> rest;
    for (std::size_t i = 1; i < events.size(); ++i) rest.emplace(events[i].kind, events[i].axis);
    CHECK(rest == (std::multiset<std::pair<std::string, std::string>>{
                      {"unit_restarted", "SIM3:1"}, {"recovered", "SIM2:1"}, {"recovered", "SIM3:1"}}));
    std::size_t restarted_at = 0, recovered_at = 0;
    for (std::size_t i = 0; i < events.size(); ++i) {
        if (events[i].axis != "SIM3:1") continue;
        if (events[i].kind == "unit_restarted") restarted_at = i;
        if (events[i].kind == "recovered") recovered_at = i;
    }
    CHECK(restarted_at < recovered_at);
    for (const auto& kv : ports) {
        std::vector<Frame> after;
        const auto requests = kv.second->requests();
        for (std::size_t i = mark[kv.first]; i < requests.size(); ++i) after.push_back(requests[i].second);
        CHECK(motion_frames(after).empty());
    }
    cell.close();
}

TEST(recovery_cell_stop_on_fault_can_be_switched_off) {
    auto ports = sim_ports(2, 2.0);
    CellOptions o = cell_options();
    o.stop_on_fault = false;
    Cell cell = Cell::open(ports, {}, o);
    std::map<std::string, double> targets;
    for (const auto& n : cell.names()) targets[n] = 140.0;
    cell.move_all(targets, no_wait(60.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ports.at("SIM1")->unplug();
    const auto res = cell.status_all({}, true);
    REQUIRE(res.errors.count("SIM1:1") == 1);
    CHECK(rethrows_as<TransportError>(res.errors.at("SIM1:1")));
    CHECK(cell.faults() == (std::map<std::string, std::string>{{"SIM1:1", rc::kPortVanished}}));
    CHECK(ports.at("SIM2")->unit(1)->moving());  // left running
    cell.stop_all({"SIM2:1"});
    cell.close();
}

TEST(recovery_cell_check_links_tracks_and_detects_a_restart) {
    auto ports = sim_ports(2, 50.0);
    Events seen;
    CellOptions o = cell_options();
    o.on_event = seen.callback();
    Cell cell = Cell::open(ports, {}, o);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto checks = cell.check_links();
    CHECK(checks.succeeded());
    for (const auto& kv : checks.results) CHECK(kv.second.ok());
    CHECK(rc::session_of(cell["SIM1:1"]) != nullptr);
    ports.at("SIM1")->power_cycle();
    checks = cell.check_links();
    CHECK_EQ(checks.at("SIM1:1").state, std::string(rc::kUnitRestarted));
    CHECK(checks.at("SIM2:1").ok());
    CHECK(cell.faults() == (std::map<std::string, std::string>{{"SIM1:1", rc::kUnitRestarted}}));
    CHECK(seen.kinds() == Strings{"unit_restarted"});
    const auto reports = cell.recover_all({"SIM1:1"}, 2.0, quick());
    CHECK(reports.at("SIM1:1").restarted && reports.at("SIM1:1").control_restored);
    CHECK(seen.kinds() == (Strings{"unit_restarted", "recovered"}));
    for (const auto& kv : cell.check_links().results) CHECK(kv.second.ok());
    cell.close();
}

TEST(recovery_cell_shared_line_power_loss_restores_the_baud_rate_once) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    for (int s : {1, 2})
        units.push_back(sim_unit(SimProfile::Slider, 50.0, 115200, [s](SimulatedUnitOptions& o) { o.slave = s; }));
    auto line = std::make_shared<SimLine>(units, "L");
    Cell cell = Cell::open(std::map<std::string, std::shared_ptr<SimLine>>{{"L", line}}, {}, cell_options());
    cell.track_all();
    line->power_cycle();
    for (const auto& u : units) CHECK_EQ(u->unit_baud_rate().value_or(0), 38400);
    const auto reports = cell.recover_all({}, 5.0, quick());
    CHECK(reports.at("L:1").baud_restored);
    CHECK(!reports.at("L:2").baud_restored);  // the broadcast already switched it
    for (const auto& u : units) CHECK_EQ(u->unit_baud_rate().value_or(0), 115200);
    for (const auto& kv : reports.results) CHECK(kv.second.restarted && kv.second.control_restored);
    for (const auto& u : units) CHECK(u->pmsl());
    cell.close();
}

TEST(recovery_cell_recover_all_refuses_a_replaced_unit) {
    auto ports = sim_ports(2, 50.0);
    Cell cell = Cell::open(ports, {}, cell_options());
    auto line = ports.at("SIM2");
    line->unplug();
    line->remove(1);
    line->add(sim_unit(SimProfile::Slider, 50.0, std::nullopt, [](SimulatedUnitOptions& o) { o.serial_number = "OTHER"; }));
    line->replug();
    try {
        cell.recover_all({}, 2.0, quick());  // never tracked: compared with Cell::open's identity
        CHECK(false);
    } catch (const CellError& e) {
        CHECK_EQ(e.errors().size(), std::size_t(1));
        REQUIRE(e.errors().count("SIM2:1") == 1);
        CHECK(rethrows_as<rc::UnitReplacedError>(e.errors().at("SIM2:1")));
    }
    CHECK(cell.faults() == (std::map<std::string, std::string>{{"SIM2:1", rc::kUnitReplaced}}));
    bool replaced = false;
    for (const auto& e : cell.events()) replaced = replaced || e.kind == "unit_replaced";
    CHECK(replaced);
    cell.close();
}

TEST(recovery_cell_monitor_links_reports_a_fault_and_stops_with_the_cell) {
    auto ports = sim_ports(2, 2.0);
    std::mutex mutex;
    std::vector<std::pair<std::string, std::string>> faults;
    CellOptions o = cell_options();
    o.on_fault = [&](const std::string& n, const std::string& k, const std::string&) {
        std::lock_guard<std::mutex> lock(mutex);
        faults.emplace_back(n, k);
    };
    auto fault_count = [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return faults.size();
    };
    Cell cell = Cell::open(ports, {}, o);
    std::shared_ptr<rc::LinkMonitor> monitor = cell.monitor_links(0.02);
    cell.move_all({{"SIM2:1", 140.0}}, no_wait(60.0));
    ports.at("SIM1")->unplug();
    CHECK(wait_until([&] { return fault_count() >= 1; }));
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(faults == (std::vector<std::pair<std::string, std::string>>{{"SIM1:1", rc::kPortVanished}}));
    }
    const auto sim2 = ports.at("SIM2")->unit(1);
    CHECK(wait_until([&] { return !sim2->moving(); }));
    CHECK(!sim2->moving());  // stopped by the fault handling
    CHECK(sim2->position_mm() < 139.0);
    // Every pass stores its checks, so an earlier pass may have left "ok"; the fault is stored once the stop has finished
    CHECK(wait_until([&] {
        const auto states = monitor->states();
        const auto it = states.find("SIM1:1");
        return it != states.end() && it->second.state == rc::kPortVanished;
    }));
    CHECK_EQ(monitor->states().at("SIM1:1").state, std::string(rc::kPortVanished));
    cell.close();
    CHECK(!monitor->running());
}

// ====================================================================== the status monitor (monitor.hpp) and recovery

namespace {

struct LinkEvents {
    std::mutex mutex;
    std::vector<LinkEvent> items;
    LinkEventCallback callback() {
        return [this](const LinkEvent& e) {
            std::lock_guard<std::mutex> lock(mutex);
            items.push_back(e);
        };
    }
    std::vector<std::pair<std::string, std::string>> pairs() {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::pair<std::string, std::string>> out;
        for (const auto& e : items) out.emplace_back(e.kind, e.cause);
        return out;
    }
    Strings kinds() {
        std::lock_guard<std::mutex> lock(mutex);
        Strings out;
        for (const auto& e : items) out.push_back(e.kind);
        return out;
    }
};

MonitorOptions monitor_options(LinkEvents* events, std::optional<double> link_check_s = kLinkCheckS) {
    MonitorOptions o;
    if (events) o.on_event.push_back(events->callback());
    o.link_check_s = link_check_s;
    return o;
}

}  // namespace

TEST(recovery_monitor_that_ended_on_the_vanished_port_is_restarted) {
    auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{sim_unit(SimProfile::Gripper)},
                                          "L1");
    Axis ax = open_axis(line);
    LinkEvents events;
    auto mon = ax.start_monitor(monitor_options(&events));
    CHECK(wait_until([&] { return mon->polls() >= 3; }));
    line->unplug();
    CHECK(wait_until([&] { return !mon->running(); }));
    CHECK(rethrows_as<TransportError>(mon->error()));
    CHECK(events.pairs() == (std::vector<std::pair<std::string, std::string>>{{"link_lost", rc::kPortVanished}}));
    line->replug();
    Events recovered;
    const rc::RecoveryReport report = rc::recover_axis(ax, ropts(5.0, quick(), recovered.callback()));
    CHECK(report.port_reopened);
    CHECK(ax.monitor() == mon);
    CHECK(mon->running());
    const long polls = mon->polls();
    CHECK(wait_until([&] { return mon->polls() > polls + 3; }));  // the monitor polls again
    CHECK(recovered.kinds() == Strings{"recovered"});
    ax.close();
}

TEST(recovery_monitor_a_running_monitor_is_held_and_restarted) {
    auto sim = sim_unit(SimProfile::Rod);
    Axis ax = open_axis(sim);
    rc::track(ax);
    auto mon = ax.start_monitor();
    CHECK(wait_until([&] { return mon->polls() >= 3; }));
    sim->power_off();
    ec_test::Thread later([sim] {  // joined even when the recovery throws
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sim->power_on();
    });
    const rc::RecoveryReport report = rc::recover_axis(ax, ropts(5.0));
    later.join();
    CHECK(report.restarted);
    CHECK(mon->running());
    const long polls = mon->polls();
    CHECK(wait_until([&] { return mon->polls() > polls + 3; }));
    ax.close();
}

TEST(recovery_monitor_a_failed_recovery_leaves_the_monitor_as_it_was) {
    auto sim = sim_unit(SimProfile::Gripper);
    Axis ax = open_axis(sim);
    rc::track(ax);
    auto mon = ax.start_monitor();
    sim->power_off();
    CHECK_THROWS(rc::recover_axis(ax, ropts(0.2)), rc::RecoveryError);
    CHECK(mon->running());  // still running (polls fail until the power is back)
    sim->power_on();
    ax.close();
}

TEST(recovery_monitor_link_watch_power_off_is_link_lost_and_power_on_a_restart) {
    auto sim = sim_unit(SimProfile::Rod);
    auto rec = std::make_shared<Recorder>();
    Axis ax = open_axis(sim, rec);
    LinkEvents events;
    rec->clear();
    auto mon = ax.start_monitor(monitor_options(&events, 0.05));
    CHECK(wait_until([&] { return mon->link_checks() >= 2; }, 5.0));  // link checks run once the axis is identified
    CHECK(rc::session_of(ax) != nullptr);  // a session made from the identity connect() read
    sim->power_off();
    CHECK(wait_until([&] { return events.kinds() == Strings{"link_lost"}; }, 5.0));
    CHECK(events.pairs().front().second == rc::kUnitSilent);
    CHECK(mon->running());  // a silent unit does not end the monitor
    CHECK_EQ(mon->link_state(), std::string(rc::kUnitSilent));
    sim->power_on();
    CHECK(wait_until([&] { return any_contains(events.kinds(), "unit_restarted"); }, 5.0));
    CHECK(events.kinds() == (Strings{"link_lost", "unit_restarted"}));
    {
        std::lock_guard<std::mutex> lock(events.mutex);
        bool dropped = false;
        for (const auto& e : events.items) dropped = dropped || has(e.message, "uptime counter dropped");
        CHECK(dropped);
    }
    CHECK_EQ(rc::link_state(ax), std::string(rc::kUnitRestarted));
    CHECK(function_codes(rec->frames()) == std::set<int>{0x03});  // the link watch only reads
    ax.close();
}

TEST(recovery_monitor_link_watch_deliberate_changes_by_the_axis_are_not_restarts) {
    auto sim = sim_unit(SimProfile::Rod);
    Axis ax = open_axis(sim);
    rc::track(ax);
    LinkEvents events;
    auto mon = ax.start_monitor(monitor_options(&events, 0.02));
    CHECK(wait_until([&] { return mon->link_checks() >= 2; }, 5.0));
    ax.disable_modbus_control();
    long checks = mon->link_checks();
    CHECK(wait_until([&] { return mon->link_checks() >= checks + 3; }, 5.0));
    ax.enable_modbus_control();
    const SoftwareResetResult result = ax.software_reset();
    CHECK(!result.homed());  // the rod lost HEND at the reset: noted, not a restart
    checks = mon->link_checks();
    CHECK(wait_until([&] { return mon->link_checks() >= checks + 3; }, 5.0));
    CHECK(events.kinds().empty());
    CHECK_EQ(rc::link_state(ax), std::string(rc::kLinkOk));
    ax.close();
}
