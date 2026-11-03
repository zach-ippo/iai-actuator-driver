// Axis unit: the command strategies end to end (a servo / home / stop / alarm-reset cycle with the coil and the IAI
// 0x45 strategies and their exact frames), servo OFF while moving, the PMSL frame, the action mode keeping the present
// safety speed, the safety speed and its verification, the lost / refused replies of the 0x45 home, the automatic
// servo OFF before a move, and the 0.001 mm unit in monitor_reading(). Port of the Python edition's tests/test_ec.py
// (CommandStrategyTest), tests/test_ec_guards.py (IaiHomeRepliesTest, ActionModeSafetyTest), tests/test_motion.py
// (SafetySpeedTest), tests/test_axis_features.py (AutoServoOffTest: the move part; LineRulesTest: the line object)
// and the read_monitor scaling of ec.py. Simulator and scripted frames only.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "filter_transport.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
using ec_test::Bytes;
using ec_test::FilterTransport;

namespace {

Bytes hexbytes(const std::string& text) {
    Bytes out;
    std::size_t i = 0;
    while (i < text.size()) {
        if (text[i] == ' ') {
            ++i;
            continue;
        }
        out.push_back(static_cast<std::uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
        i += 2;
    }
    return out;
}

/// 0x45 home, op1 = ORG 7 (VERIFIED in the Python edition on the EC-R6S rod).
const Bytes kHomeFrame = hexbytes("01 45 00 0A 00 07 00 00 BC C3");

std::shared_ptr<SimulatedUnit> make_sim(double scale = 50.0, SimProfile profile = SimProfile::Slider,
                                        bool pmsl = true, bool safety_speed = false) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    o.pmsl = pmsl;
    o.safety_speed = safety_speed;
    return std::make_shared<SimulatedUnit>(o);
}

AxisOptions axis_options() {
    AxisOptions a;
    a.stroke = Span{0.0, 150.0};
    a.max_speed_mm_s = 100.0;
    a.max_accel_g = 0.3;
    a.poll_interval_s = 0.005;
    return a;
}

Axis open_axis(const std::shared_ptr<Transport>& t, AxisOptions a = axis_options(), int retries = 2) {
    OpenOptions o;
    o.slave = 1;
    o.axis = a;
    o.bus.min_gap_s = 0.0;
    o.bus.retries = retries;
    o.line.kind = "direct";
    return Axis::open(t, o);
}

bool has_frame(const std::vector<Bytes>& frames, const Bytes& frame) {
    return std::find(frames.begin(), frames.end(), frame) != frames.end();
}

std::vector<Bytes> of_function(const std::vector<Bytes>& frames, int fc) {
    std::vector<Bytes> out;
    for (const Bytes& f : frames)
        if (f.size() > 1 && f[1] == fc) out.push_back(f);
    return out;
}

std::vector<Bytes> home_frames(const std::vector<Bytes>& frames) {
    std::vector<Bytes> out;
    for (const Bytes& f : of_function(frames, 0x45))
        if (f.size() > 3 && f[3] == reg::kIaiCmdHome) out.push_back(f);
    return out;
}

/// FC05 frames of coil `address`, "1" = ON, "0" = OFF.
std::string coil_trace(const std::vector<Bytes>& frames, int address) {
    std::string out;
    for (const Bytes& f : frames)
        if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == address) out += f[4] == 0xFF ? '1' : '0';
    return out;
}

bool is_home_frame(const Bytes& f) { return f.size() > 3 && f[1] == 0x45 && f[3] == reg::kIaiCmdHome; }

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool any_contains(const std::vector<std::string>& texts, const std::string& part) {
    for (const auto& t : texts)
        if (contains(t, part)) return true;
    return false;
}

std::ptrdiff_t index_of(const std::vector<std::string>& events, const std::string& event) {
    const auto it = std::find(events.begin(), events.end(), event);
    return it == events.end() ? -1 : it - events.begin();
}

bool wait_for(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_at_least(0.005);
    }
    return predicate();
}

/// Collects the warnings logged while it lives.
class Warnings {
public:
    Warnings() {
        previous_ = set_log_handler([this](LogLevel level, const std::string&, const std::string& message) {
            if (level < LogLevel::Warning) return;
            std::lock_guard<std::mutex> lock(mutex_);
            lines_.push_back(message);
        });
    }
    ~Warnings() { set_log_handler(previous_); }
    std::vector<std::string> lines() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lines_;
    }

private:
    LogHandler previous_;
    mutable std::mutex mutex_;
    std::vector<std::string> lines_;
};

/// The servo / home / stop / alarm-reset cycle of the Python edition's CommandStrategyTest.run_cycle; returns every
/// frame written.
std::vector<Bytes> run_cycle(CommandMethod method) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    ax.set_strategy(method);
    sim->clear_requests();
    ax.servo(false);
    CHECK(!sim->servo_on());
    ax.servo(true);
    CHECK(sim->servo_on());
    const auto st = ax.home();
    REQUIRE(st.has_value());
    CHECK(st->homed);
    CHECK_NEAR(st->position_mm, 0.0, 0.005);
    MoveOptions m;
    m.speed = 100.0;
    m.accel_g = 0.3;
    m.wait = false;
    ax.move_to(20.0, m);
    ax.stop(true);
    sim->trigger_alarm(0x0E0);
    CHECK(!sim->servo_on());
    const Status after = ax.reset_alarm();
    CHECK(!after.alarm);
    CHECK_EQ(after.alarm_code, 0);
    return sim->requests();
}

/// The FC03 answer `reply` to a 0x9000 status read with STAT b17 ASOF set (when the read covers STAT), CRC renewed.
Bytes with_asof(const Bytes& reply) {
    // FC03 answer: slave, 3, byte count, words...; STAT is words 8-9 (high word first): b17 = high word bit 1
    if (reply.size() < 5 || reply[1] != 0x03 || reply[2] < 20) return reply;
    std::vector<std::uint8_t> pdu(reply.begin() + 1, reply.end() - 2);
    pdu[2 + 2 * 8 + 1] |= 0x02;
    return build_frame(reply[0], pdu);
}

bool is_status_read(const Bytes& f) {
    return f.size() >= 6 && f[1] == 0x03 && ((f[2] << 8) | f[3]) == reg::kMonitorBase;
}

}  // namespace

// ------------------------------------------------------------------------------------------ strategy cycles

TEST(axis_command_strategy_coil_cycle_frames) {
    const std::vector<Bytes> tx = run_cycle(CommandMethod::Coil);
    for (const char* frame : {
             "01 05 04 03 00 00 3C FA",  // servo off
             "01 05 04 03 FF 00 7D 0A",  // servo on
             "01 05 04 0B 00 00 BD 38",  // home: 0 ...
             "01 05 04 0B FF 00 FC C8",  // ... then the rising edge
             "01 05 04 2C FF 00 4C C3",  // stop
             "01 05 04 07 FF 00 3C CB",  // alarm reset pulse
             "01 05 04 07 00 00 7D 3B",
         }) {
        CHECK(has_frame(tx, hexbytes(frame)));
    }
    const auto on = std::find(tx.begin(), tx.end(), hexbytes("01 05 04 0B FF 00 FC C8"));
    REQUIRE(on != tx.end());
    CHECK(std::find(on + 1, tx.end(), hexbytes("01 05 04 0B 00 00 BD 38")) != tx.end());  // released after homing
}

TEST(axis_command_strategy_iai_cycle_frames) {
    const std::vector<Bytes> tx = run_cycle(CommandMethod::Iai);
    CHECK(has_frame(tx, hexbytes("01 45 00 09 00 01 00 00 18 C2")));   // 0x45 servo on (VERIFIED on the gripper)
    CHECK(has_frame(tx, kHomeFrame));                                   // 0x45 home: op1 = ORG (9.16) = 7 (VERIFIED)
    CHECK(!has_frame(tx, hexbytes("01 45 00 0A 00 00 00 00 0D 02")));  // op1 = 0: rejected 0x11 by the gripper
    CHECK(has_frame(tx, hexbytes("01 45 00 05 00 00 00 00 59 03")));   // 0x45 alarm reset, op1 0 (VERIFIED)
    for (const Bytes& f : of_function(tx, 0x45)) CHECK(f[3] != 0x0B);  // the 0x45 decel stop is never used
    std::set<int> coils;
    for (const Bytes& f : of_function(tx, 0x05)) coils.insert((f[2] << 8) | f[3]);
    CHECK(coils == std::set<int>{reg::kCoilStop});  // the only coil: the VERIFIED STOP
}

TEST(axis_command_strategy_servo_off_while_moving_stops_first) {
    auto sim = make_sim(2.0);
    Axis ax = open_axis(sim);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    CHECK(wait_for([&] { return sim->moving(); }));
    sim->clear_events();
    ax.servo(false);
    const std::vector<std::string> events = sim->events();
    const std::ptrdiff_t stop = index_of(events, "stop");
    const std::ptrdiff_t off = index_of(events, "servo off");
    CHECK(stop >= 0);
    CHECK(off >= 0);
    CHECK(stop < off);
}

TEST(axis_command_strategy_servo_on_refused_with_an_alarm) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    sim->trigger_alarm(0x0E0);
    CHECK_THROWS(ax.servo(true), EcAlarmError);
}

TEST(axis_command_strategy_enable_modbus_control_frame) {
    auto sim = make_sim(50.0, SimProfile::Slider, false);
    Axis ax = open_axis(sim);
    CHECK(!ax.status().modbus_enabled);
    const auto st = ax.enable_modbus_control();
    REQUIRE(st.has_value());
    CHECK(st->modbus_enabled);
    CHECK(has_frame(sim->requests(), hexbytes("01 05 04 27 FF 00 3D 01")));
    ax.disable_modbus_control();
    CHECK(!sim->pmsl());
}

TEST(axis_command_strategy_pmsl_refused_with_exception_01_says_why) {
    auto sim = make_sim(50.0, SimProfile::Slider, false);
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (f.size() > 3 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilPmsl)
            return FilterTransport::exception(f[0], 0x05, 0x01);
        return std::nullopt;
    });
    Axis ax = open_axis(t);
    try {
        ax.enable_modbus_control();
        CHECK(false);  // must raise
    } catch (const ModbusSlaveError& e) {
        CHECK(any_contains(e.notes(), "no FC05 coils"));
    }
}

TEST(axis_command_strategy_line_object_on_an_open_axis) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    CHECK_EQ(ax.stop_path(), "coil");
    ax.bus()->set_line(std::make_shared<Line>(LineKind::Rcon));
    CHECK_EQ(ax.stop_path(), "register");
    CHECK(ax.line_name() == std::optional<std::string>("rcon"));
    CHECK(ax.status().line == std::optional<std::string>("rcon"));
    sim->clear_requests();
    CHECK_THROWS(ax.software_reset(), UnsafeCommandError);
    CHECK(of_function(sim->requests(), 0x05).empty());
}

// ------------------------------------------------------------------------------------------ action mode, safety speed

TEST(axis_command_strategy_action_mode_keeps_the_present_safety_speed) {
    auto sim = make_sim(50.0, SimProfile::Slider, false, true);  // e.g. left on by IA-OS Teach mode
    Axis ax = open_axis(sim);
    // PC is the PMSL coil (PMSS 0 only) and the SFTY coil (only when it changes), never 0x45 op1 = 1: on the PCON-CB
    // that left the servo refusing to come on (VERIFIED, 2026-11-02); PIO stays 0x45 cmd 0x0C op1 = 0
    using Coil = std::pair<int, bool>;
    auto action_frames = [&]() {
        std::vector<Bytes> out;
        for (const Bytes& f : of_function(sim->requests(), 0x45))
            if (f[3] == reg::kIaiCmdActionMode) out.push_back(f);
        return out;
    };
    auto coils = [&]() {
        std::vector<Coil> out;
        for (const Bytes& f : of_function(sim->requests(), 0x05)) out.emplace_back((f[2] << 8) | f[3], f[4] == 0xFF);
        CHECK(action_frames().empty());
        sim->clear_requests();
        return out;
    };
    sim->clear_requests();
    Status st = ax.set_action_mode(true);  // nullopt: keep SFTY as it is
    CHECK((coils() == std::vector<Coil>{{reg::kCoilPmsl, true}}));
    CHECK(st.modbus_enabled);
    CHECK(st.safety_speed());
    CHECK(sim->safety_speed());
    st = ax.set_action_mode(true, false);  // explicit off
    CHECK((coils() == std::vector<Coil>{{reg::kCoilSfty, false}}));
    CHECK(!st.safety_speed());
    st = ax.set_action_mode(true);  // off stays off: nothing written
    CHECK(coils().empty());
    CHECK(!st.safety_speed());
    st = ax.set_action_mode(true, true);  // explicit on
    CHECK((coils() == std::vector<Coil>{{reg::kCoilSfty, true}}));
    CHECK(st.safety_speed());
    st = ax.set_action_mode(false);  // PIO: the safety speed is kept too (op2 = 1)
    const std::vector<Bytes> pio = action_frames();
    REQUIRE(pio.size() == 1);
    CHECK(Bytes(pio[0].begin() + 2, pio[0].begin() + 8) == (Bytes{0x00, 0x0C, 0x00, 0x00, 0x00, 0x01}));
    CHECK(!st.modbus_enabled);
    CHECK(sim->safety_speed());
}

TEST(axis_command_strategy_safety_speed_reported_and_switched) {
    auto sim = make_sim(50.0, SimProfile::Slider, true, true);
    Axis ax = open_axis(sim);
    const Status st = ax.status();
    CHECK(st.safety_speed());
    CHECK((st.raw.dss1 & (1u << 14)) != 0);  // DSS1 SFTY, the live bit
    const Status off = ax.set_safety_speed(false);
    CHECK(!off.safety_speed());
    CHECK(!sim->safety_speed());
    CHECK(ax.set_safety_speed(true).safety_speed());
}

TEST(axis_command_strategy_safety_speed_is_verified) {
    // a unit that answers the SFTY coil write but ignores it
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (f.size() > 3 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilSfty) return f;  // the echo, no action
        return std::nullopt;
    });
    Axis ax = open_axis(t);
    CHECK_THROWS(ax.set_safety_speed(true), VerificationError);
    CHECK(!sim->safety_speed());
}

// ------------------------------------------------------------------------------------------ 0x45 home replies

TEST(axis_iai_home_lost_reply_while_the_home_runs_is_waited_for) {
    auto sim = make_sim(50.0, SimProfile::Rod);
    // the unit runs the home frame, but its answer is lost on the line
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (!is_home_frame(f)) return std::nullopt;
        FilterTransport::exchange(inner, f);
        return Bytes();  // silence
    });
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    ax.move_to(30.0, m);
    t->clear_frames();
    std::optional<Status> st;
    std::vector<std::string> logged;
    {
        Warnings warnings;
        st = ax.home(true, std::nullopt, CommandMethod::Iai);
        logged = warnings.lines();
    }
    CHECK(any_contains(logged, "seen running"));
    REQUIRE(st.has_value());
    CHECK(st->homed);
    CHECK_NEAR(st->position_mm, 0.0, 0.005);
    CHECK(home_frames(t->frames()) == std::vector<Bytes>{kHomeFrame});  // sent once: no retry rejected 0x24
    CHECK(coil_trace(t->frames(), reg::kCoilStop).empty());  // the home was seen running: nothing stopped
}

TEST(axis_iai_home_lost_reply_without_a_home_stops_and_raises) {
    auto sim = make_sim(50.0, SimProfile::Rod);
    // the home frame never reaches the unit (nothing runs, nothing is answered)
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (is_home_frame(f)) return Bytes();
        return std::nullopt;
    });
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    ax.move_to(30.0, m);
    t->clear_frames();
    Warnings warnings;
    try {
        ax.home(true, std::nullopt, CommandMethod::Iai);  // waits kHomeStartS for a home return to show
        CHECK(false);                                     // must raise
    } catch (const ModbusTimeout& e) {
        CHECK(any_contains(e.notes(), "no home return was seen"));
        CHECK(any_contains(e.notes(), "decel stop sent"));
    }
    CHECK_EQ(home_frames(t->frames()).size(), 1u);
    CHECK_EQ(coil_trace(t->frames(), reg::kCoilStop), "10");  // on doubt: decel stop (the frame may have run)
    CHECK_NEAR(ax.status().position_mm, 30.0, 0.005);
}

TEST(axis_iai_home_rejection_0x24_stops_the_running_home) {
    auto sim = make_sim(1.0, SimProfile::Rod);  // real time: the home is still running when the STOP goes out
    // the home frame starts a home return, but the answer is rejection 0x24 (as a repeated frame would get)
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (!is_home_frame(f)) return std::nullopt;
        FilterTransport::exchange(inner, f);
        return build_frame(f[0], {0x45, 0x24, 0x00});
    });
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    m.timeout_s = 20.0;
    ax.move_to(30.0, m);
    t->clear_frames();
    sim->clear_events();
    try {
        ax.home(true, std::nullopt, CommandMethod::Iai);
        CHECK(false);  // must raise
    } catch (const IaiCommandRejected& e) {
        CHECK_EQ(e.rejection(), 0x24);
        CHECK(any_contains(e.notes(), "home return already in progress"));
    }
    CHECK_EQ(coil_trace(t->frames(), reg::kCoilStop), "10");  // never "nothing moves" while it homes
    const std::vector<std::string> events = sim->events();
    const std::ptrdiff_t home = index_of(events, "home");
    const std::ptrdiff_t stop = index_of(events, "stop");
    CHECK(home >= 0);
    CHECK(stop >= 0);
    CHECK(home < stop);
    const Status st = ax.status();
    CHECK(!st.homing());
    CHECK(!st.moving);
    CHECK(!st.homed);  // the home was cut short
}

TEST(axis_iai_home_answered_refusal_sends_no_stop) {
    auto sim = make_sim(50.0, SimProfile::Rod);
    // the home frame is answered with rejection 0x11 and nothing runs
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (is_home_frame(f)) return build_frame(f[0], {0x45, 0x11, 0x00});
        return std::nullopt;
    });
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    ax.move_to(30.0, m);
    t->clear_frames();
    try {
        ax.home(true, std::nullopt, CommandMethod::Iai);
        CHECK(false);  // must raise
    } catch (const IaiCommandRejected& e) {
        CHECK_EQ(e.rejection(), 0x11);
    }
    CHECK_EQ(home_frames(t->frames()).size(), 1u);
    CHECK(coil_trace(t->frames(), reg::kCoilStop).empty());  // this frame did not run: nothing to stop
    CHECK_NEAR(ax.status().position_mm, 30.0, 0.005);
}

// ------------------------------------------------------------------------------------------ automatic servo OFF

TEST(axis_asof_move_switches_the_servo_back_on_first) {
    // the simulator has no automatic servo OFF: the servo is switched off and the status reads show STAT ASOF until
    // the next servo ON frame
    auto sim = make_sim(50.0);
    auto asof = std::make_shared<std::atomic<bool>>(false);
    auto t = std::make_shared<FilterTransport>(sim, [asof](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilSon && f[4] == 0xFF) *asof = false;
        if (!*asof || !is_status_read(f)) return std::nullopt;
        return with_asof(FilterTransport::exchange(inner, f));
    });
    Axis ax = open_axis(t);
    ax.servo(false);
    *asof = true;
    const Status st = ax.status();
    CHECK(!st.servo_on);
    CHECK(st.auto_servo_off() == std::optional<bool>(true));
    t->clear_frames();
    MoveOptions m;
    m.speed = 50.0;
    ax.check_move(90.0, m);  // a check only: accepted, nothing written
    for (const Bytes& f : t->frames()) CHECK(f[1] != 0x05 && f[1] != 0x06 && f[1] != 0x10);
    const auto done = ax.move_to(90.0, m);
    REQUIRE(done.has_value());
    CHECK_NEAR(done->position_mm, 90.0, 0.011);
    const std::vector<Bytes> tx = t->frames();
    std::ptrdiff_t son = -1, move = -1;
    for (std::size_t i = 0; i < tx.size(); ++i) {
        const Bytes& f = tx[i];
        if (son < 0 && f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilSon && f[4] == 0xFF)
            son = static_cast<std::ptrdiff_t>(i);
        if (move < 0 && f.size() >= 4 && f[1] == 0x10 && ((f[2] << 8) | f[3]) == reg::kDirectMoveBase)
            move = static_cast<std::ptrdiff_t>(i);
    }
    CHECK(son >= 0);
    CHECK(move >= 0);
    CHECK(son < move);  // SON goes out before the 0x9900 write
}

TEST(axis_asof_servo_that_stays_off_is_a_servo_blocked_error) {
    // STAT ASOF and a servo that never comes back (the SON frames are answered but ignored)
    auto sim = make_sim(50.0);
    auto asof = std::make_shared<std::atomic<bool>>(false);
    auto t = std::make_shared<FilterTransport>(sim, [asof](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (!*asof) return std::nullopt;
        if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilSon) return f;  // echo, no action
        if (!is_status_read(f)) return std::nullopt;
        return with_asof(FilterTransport::exchange(inner, f));
    });
    Axis ax = open_axis(t);
    ax.servo(false);
    *asof = true;
    t->clear_frames();
    MoveOptions m;
    m.speed = 50.0;
    CHECK_THROWS(ax.move_to(90.0, m), ServoBlockedError);
    CHECK_EQ(coil_trace(t->frames(), reg::kCoilSon), "101");  // ON, then one OFF / ON cycle
    for (const Bytes& f : t->frames()) CHECK(!(f[1] == 0x10 && ((f[2] << 8) | f[3]) == reg::kDirectMoveBase));
}

// ------------------------------------------------------------------------------------------ monitor_reading

TEST(axis_rcon_monitor_reading_uses_the_0_001_unit) {
    SimGatewayOptions g;
    g.position_unit_mm = 0.001;
    SimulatedUnitOptions u = sim_gateway_unit_defaults();
    u.time_scale = 20.0;
    auto line = sim_rcon_line({1}, g, u);
    OpenOptions o;
    o.slave = 1;
    o.bus.min_gap_s = 0.0;
    o.axis.poll_interval_s = 0.005;
    o.axis.stroke = Span{0.0, 150.0};
    o.line.position_unit_mm = 0.001;
    Axis ax = Axis::open(line, o);
    MoveOptions m;
    m.speed = 50.0;
    m.timeout_s = 10.0;
    ax.move_to(45.67, m);
    const Json reading = monitor_reading(ax);
    const Json* position = reading.find("position");
    REQUIRE(position != nullptr);
    REQUIRE(position->type() == Json::Type::Number);
    CHECK_NEAR(position->as_number(), ax.status().position_mm, 1e-9);  // the status's 0.001 mm scale
    CHECK_NEAR(position->as_number(), 45.67, 0.0015);
}

// ------------------------------------------------------------------------------------------ public accessors

TEST(axis_accessors_home_pattern_caps_and_finish_home) {
    auto sim = make_sim(50.0);
    Axis ax = open_axis(sim);
    CHECK(ax.read_home_pattern() == std::optional<std::uint32_t>(7));
    const auto minimum = ax.recommended_min_speed();
    CHECK(!minimum.second.empty());
    CHECK(ax.speed_cap() > 0.0);
    CHECK(ax.speed_cap() <= 100.0 + 1e-9);  // AxisOptions::max_speed_mm_s
    CHECK(ax.accel_cap() <= 0.3 + 1e-9);    // AxisOptions::max_accel_g
    CHECK(ax.accel_only_cap() <= 0.3 + 1e-9);
    CHECK(ax.decel_cap() <= 0.3 + 1e-9);
    CHECK(ax.accel_cap() <= ax.accel_only_cap() + 1e-9);
    CHECK(ax.push_cap() > 0.0);
    CHECK(ax.push_cap() <= 100.0);
    sim->clear_requests();
    ax.home(false);
    CHECK_EQ(coil_trace(sim->requests(), reg::kCoilHome), "01");  // the edge, still held
    CHECK(wait_for([&] { return sim->homed() && !sim->moving(); }, 10.0));
    ax.finish_home();
    CHECK_EQ(coil_trace(sim->requests(), reg::kCoilHome), "010");  // released
}

TEST(axis_accessors_limits_carry_the_move_patterns_word) {
    SimulatedUnitOptions o;
    o.time_scale = 50.0;
    o.move_patterns = 0x0007;  // MVPT: trapezoid, S-motion, filter
    auto sim = std::make_shared<SimulatedUnit>(o);
    Axis ax = open_axis(sim);
    const AxisLimits limits = ax.read_axis_limits();
    CHECK(limits.move_patterns == std::optional<std::uint32_t>(0x0007));
    const Json j = to_json(limits);
    const Json* mvpt = j.find("move_patterns");
    REQUIRE(mvpt != nullptr);
    CHECK_EQ(mvpt->as_int(), 7);
}
