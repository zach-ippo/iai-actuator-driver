// Axis unit: the command strategies (coil / iai / iai2 / register), the register path's frames (FC06 DRG1 keeping SON
// and SFTY), the coil-free STOP substitute (pause, standstill, ALRS edge while paused, release), the switch to the
// register path after an FC05 exception 01, the line's say in the strategy, the RCON status masks, the action mode,
// the safety speed, the coil teach and the push-speed entry of limits(). Port of the Python edition's
// tests/test_axis_features.py (RegisterStrategyTest, LineRulesTest, StatusFlagsTest RCON parts) and
// tests/test_safety.py (StopPathStrategyTest). Simulator and scripted frames only; the register path is DOC and
// UNVERIFIED on hardware.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
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

const Bytes kServoOff = hexbytes("01 06 0D 00 00 00 8B 66");
const Bytes kServoOn = hexbytes("01 06 0D 00 10 00 86 A6");
const Bytes kHomeEdge = hexbytes("01 06 0D 00 10 10 87 6A");
const Bytes kAlarmReset = hexbytes("01 06 0D 00 11 00 87 36");
const Bytes kPause = hexbytes("01 06 0D 00 10 20 87 7E");
const Bytes kCancelPaused = hexbytes("01 06 0D 00 11 20 86 EE");

std::shared_ptr<SimulatedUnit> make_sim(double scale = 50.0, SimProfile profile = SimProfile::Slider) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
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

Axis open_axis(const std::shared_ptr<Transport>& t, AxisOptions a = axis_options(), int retries = 3) {
    OpenOptions o;
    o.slave = 1;
    o.axis = a;
    o.bus.min_gap_s = 0.0;
    o.bus.retries = retries;
    o.line.kind = "direct";
    return Axis::open(t, o);
}

std::vector<Bytes> drg1_writes(const std::vector<Bytes>& frames) {
    std::vector<Bytes> out;
    for (const Bytes& f : frames)
        if (f.size() >= 6 && f[1] == 0x06 && f[2] == 0x0D && f[3] == 0x00) out.push_back(f);
    return out;
}

std::vector<std::uint16_t> drg1_values(const std::vector<Bytes>& frames) {
    std::vector<std::uint16_t> out;
    for (const Bytes& f : drg1_writes(frames)) out.push_back(static_cast<std::uint16_t>((f[4] << 8) | f[5]));
    return out;
}

int count_fc(const std::vector<Bytes>& frames, int fc) {
    int n = 0;
    for (const Bytes& f : frames)
        if (f.size() > 1 && f[1] == fc) ++n;
    return n;
}

/// FC05 frames of coil `address`, "1" = ON, "0" = OFF.
std::string coil_trace(const std::vector<Bytes>& frames, int address) {
    std::string out;
    for (const Bytes& f : frames)
        if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == address) out += f[4] == 0xFF ? '1' : '0';
    return out;
}

bool wait_for(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_at_least(0.005);
    }
    return predicate();
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// A unit that answers every FC05 write with exception 01 (no coils).
std::shared_ptr<FilterTransport> without_coils(const std::shared_ptr<SimulatedUnit>& sim) {
    return std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (f.size() > 1 && f[1] == 0x05) return FilterTransport::exception(f[0], 0x05, 0x01);
        return std::nullopt;
    });
}

}  // namespace

// ------------------------------------------------------------------------------------------ register path frames

TEST(axis_strategy_register_frames_match_the_documented_ones) {
    const std::pair<std::uint16_t, const Bytes*> cases[] = {
        {0x1000, &kServoOn}, {0x0000, &kServoOff}, {0x1010, &kHomeEdge},
        {0x1100, &kAlarmReset}, {0x1020, &kPause}, {0x1120, &kCancelPaused},
    };
    for (const auto& c : cases) {
        const Bytes frame = build_frame(1, {0x06, 0x0D, 0x00, static_cast<std::uint8_t>(c.first >> 8),
                                            static_cast<std::uint8_t>(c.first & 0xFF)});
        CHECK(frame == *c.second);
    }
    CHECK_EQ(std::string(to_string(CommandMethod::Register)), "register");
    CHECK_EQ(std::string(to_string(CommandMethod::Iai2)), "iai2");
    CHECK(command_method_from_string("register") == std::optional<CommandMethod>(CommandMethod::Register));
    CHECK(command_method_from_string("iai2") == std::optional<CommandMethod>(CommandMethod::Iai2));
    CHECK(!command_method_from_string("bogus").has_value());
}

TEST(axis_strategy_register_servo_home_reset_pause_through_fc06) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    CHECK(ax.strategy() == CommandMethod::Register);
    CHECK_EQ(ax.stop_path(), "register");
    sim->clear_requests();
    ax.servo(false);
    ax.servo(true);
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kServoOff, kServoOn}));
    sim->clear_requests();
    const auto st = ax.home();
    REQUIRE(st.has_value());
    CHECK(st->homed);
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kHomeEdge, kServoOn}));
    sim->clear_requests();
    sim->trigger_alarm(0x0E0);
    const Status after = ax.reset_alarm();
    CHECK_EQ(after.alarm_code, 0);
    const auto first = drg1_values(sim->requests());
    REQUIRE(!first.empty());
    // the alarm dropped the servo: its level is cleared first (SON 0), then ALRS alone (a PCON-CB re-energised the
    // motor at the reset while the level stayed set: ON-WIRE, 2026-11-03)
    REQUIRE(first.size() >= 2);
    CHECK_EQ(first[0], 0);
    CHECK_EQ(first[1], reg::kDrg1Alrs);
    ax.servo(true);
    sim->clear_requests();
    ax.reset_alarm();  // with the servo on: SON is kept in both frames
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kAlarmReset, kServoOn}));
    sim->clear_requests();
    ax.pause();
    CHECK(ax.status().paused());
    ax.resume();
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kPause, kServoOn}));
    CHECK_EQ(count_fc(sim->requests(), 0x05), 0);
}

TEST(axis_strategy_register_safety_speed_is_kept_in_every_write) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    ax.set_safety_speed(true);
    CHECK(sim->safety_speed());
    sim->clear_requests();
    ax.pause();
    ax.resume();
    CHECK(drg1_values(sim->requests()) == (std::vector<std::uint16_t>{0x5020, 0x5000}));
    CHECK_EQ(count_fc(sim->requests(), 0x05), 0);
    ax.set_safety_speed(false);
    CHECK(!sim->safety_speed());
}

TEST(axis_strategy_register_stop_substitute_ends_at_standstill_without_resuming) {
    auto sim = make_sim(5.0);
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    CHECK(wait_for([&] { return sim->position_mm() > 75.0; }, 10.0));
    sim->clear_requests();
    const Status st = ax.stop();
    CHECK(!st.moving);
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kPause, kCancelPaused, kPause, kServoOn}));
    CHECK_EQ(count_fc(sim->requests(), 0x05), 0);
    const double stopped = sim->position_mm();
    sleep_at_least(0.3);  // 1.5 s simulated: the old target must not be resumed
    CHECK_NEAR(sim->position_mm(), stopped, 0.005);
    CHECK(!sim->moving());
    CHECK(stopped < 140.0);
    const Status final_ = ax.status();
    CHECK(final_.in_position && !final_.paused());  // PEND back once the pause is released
}

TEST(axis_strategy_register_begin_stop_then_stop) {
    auto sim = make_sim(5.0);
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    sim->clear_requests();
    ax.begin_stop();
    CHECK(drg1_writes(sim->requests()) == (std::vector<Bytes>{kPause}));
    ax.stop();
    CHECK(!sim->moving());
    CHECK(!ax.status().paused());
}

TEST(axis_strategy_register_error_path_uses_the_substitute) {
    auto sim = make_sim(5.0);
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    sim->clear_requests();
    bool threw = false;
    try {
        ax.wait_idle(140.0, std::nullopt, 0.05);  // MotionTimeoutError: the guard stops the axis
    } catch (const EcDriverError& e) {
        threw = true;
        bool noted = false;
        for (const auto& n : e.notes())
            if (contains(n, "decel stop (DRG1 pause + cancel) sent")) noted = true;
        CHECK(noted);
    }
    CHECK(threw);
    CHECK(!sim->moving());
    CHECK_EQ(count_fc(sim->requests(), 0x05), 0);
    CHECK(!ax.status().paused());
}

TEST(axis_strategy_register_resume_after_a_held_stop_pause_never_resumes_the_move) {
    auto sim = make_sim(5.0);
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Register);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    ax.begin_stop();  // the pause is held; resume() cancels it instead of continuing the move
    // the simulator keeps the travel pending while paused (moving() stays true): wait for standstill instead
    CHECK(wait_for([&] {
        const double a = sim->position_mm();
        sleep_at_least(0.05);
        return std::fabs(sim->position_mm() - a) < 0.001;
    }, 10.0));
    CHECK(ax.status().paused());
    ax.resume();
    CHECK(!sim->moving());  // the ALRS edge while paused dropped the remaining travel
    const double where = sim->position_mm();
    sleep_at_least(0.2);
    CHECK_NEAR(sim->position_mm(), where, 0.005);
    CHECK(!ax.status().paused());
}

// ------------------------------------------------------------------------------------------ FC05 exception 01

TEST(axis_strategy_fc05_exception_01_switches_to_register) {
    auto sim = make_sim(50.0);
    auto t = without_coils(sim);
    Axis ax = open_axis(t);
    CHECK(ax.strategy() == CommandMethod::Coil);
    ax.servo(false);
    CHECK(!sim->servo_on());
    CHECK(ax.strategy() == CommandMethod::Register);
    CHECK_EQ(ax.stop_path(), "register");
    ax.servo(true);
    MoveOptions m;
    m.speed = 50.0;
    m.wait = false;
    ax.move_to(140.0, m);
    t->clear_frames();
    ax.stop();
    CHECK_EQ(count_fc(t->frames(), 0x05), 0);
    CHECK(!sim->moving());
}

TEST(axis_strategy_stop_coil_refused_switches_the_stop_path) {
    auto sim = make_sim(5.0);
    auto t = without_coils(sim);
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    ax.stop();
    CHECK_EQ(ax.stop_path(), "register");
    CHECK(!sim->moving());
    CHECK(ax.strategy() == CommandMethod::Register);
}

TEST(axis_strategy_pause_coil_refused_switches_to_the_register_pause) {
    auto sim = make_sim(5.0);
    auto t = without_coils(sim);
    Axis ax = open_axis(t);
    ax.pause();
    CHECK(ax.status().paused());
    ax.resume();
    CHECK(!ax.status().paused());
    CHECK_EQ(ax.stop_path(), "register");
}

TEST(axis_strategy_explicit_register_method) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    sim->trigger_alarm(0x0E0);
    sim->clear_requests();
    ax.reset_alarm(CommandMethod::Register);
    CHECK_EQ(sim->alarm_code(), 0);
    CHECK_EQ(count_fc(sim->requests(), 0x05), 0);
    CHECK(ax.strategy() == CommandMethod::Coil);  // a per-call method does not change the strategy
}

TEST(axis_strategy_register_image_read_failure_reuses_the_last_image_only) {
    // DRG1 0x5000 | ALRS while readable; nothing (timeout) or an exception response when it fails
    bool fail = false;
    std::vector<std::uint16_t> writes;
    auto fake = std::make_shared<FakeTransport>([&](const Bytes& f) -> std::optional<Bytes> {
        if (f.size() < 6) return std::nullopt;
        if (f[1] == 0x03) {
            if (fail) return FilterTransport::exception(f[0], 0x03, 0x04);
            return build_frame(f[0], {0x03, 0x02, 0x51, 0x00});
        }
        if (f[1] == 0x06) {
            writes.push_back(static_cast<std::uint16_t>((f[4] << 8) | f[5]));
            return Bytes(f.begin(), f.end());  // the echo
        }
        return std::nullopt;
    });
    BusOptions bo;
    bo.retries = 0;
    bo.min_gap_s = 0.0;
    ModbusRtu bus(fake, bo);
    RegisterCommands strategy;
    fail = true;
    // no image and DSS1 unreadable: a pause (the first frame of a STOP) is still sent, with SON alone kept
    strategy.pause(bus, 1, true);
    fail = false;
    strategy.pause(bus, 1, true);
    fail = true;
    strategy.pause(bus, 1, false);
    CHECK(writes == (std::vector<std::uint16_t>{0x1020, 0x5020, 0x5000}));  // ALRS left high dropped, SON / SFTY kept
    CHECK(strategy.last_image(1) == std::optional<std::uint16_t>(0x5000));
    RegisterCommands fresh;
    CHECK_THROWS(fresh.servo(bus, 1, true), ModbusSlaveError);  // no image to fall back on: never a guess
}

// ------------------------------------------------------------------------------------------ the line decides

TEST(axis_strategy_rcon_line_register_path_and_refusals) {
    auto line = sim_rcon_line({1}, {}, [] {
        SimulatedUnitOptions u = sim_gateway_unit_defaults();
        u.time_scale = 5.0;
        return u;
    }());
    OpenOptions o;
    o.axis = axis_options();
    o.axis.stroke = Span{0.0, 150.0};
    o.bus.min_gap_s = 0.0;
    o.line.position_unit_mm = 0.01;
    Axis ax = Axis::open(line, o);
    CHECK_EQ(ax.stop_path(), "register");
    CHECK(ax.line_name() == std::optional<std::string>("rcon"));
    ax.servo(true);
    CHECK(ax.strategy() == CommandMethod::Register);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    CHECK(wait_for([&] { return line->unit(1)->moving(); }));
    ax.stop();
    CHECK(!line->unit(1)->moving());
    line->clear_requests();
    CHECK_THROWS(ax.set_baud_rate(115200), LineRuleError);
    CHECK_THROWS(ax.software_reset(), LineRuleError);
    CHECK_THROWS(ax.home(true, std::nullopt, CommandMethod::Iai), LineRuleError);
    CHECK_THROWS(ax.home(true, std::nullopt, CommandMethod::Coil), LineRuleError);
    CHECK_THROWS(ax.set_action_mode(true), LineRuleError);
    PositionMoveOptions pm;
    pm.speed_pct = 50.0;
    CHECK_THROWS(ax.move_to_position(0, pm), LineRuleError);
    MoveOptions profile;
    profile.speed = 20.0;
    profile.decel_g = 0.2;
    CHECK_THROWS(ax.move_to(80.0, profile), LineRuleError);
    for (const auto& r : line->requests()) {
        const Bytes& f = r.second;
        CHECK(f[0] != 0);
        CHECK(f[1] != 0x05 && f[1] != 0x45 && f[1] != 0x48);
    }
}

TEST(axis_strategy_iai_strategy_falls_back_on_a_line_without_0x45) {
    auto line = sim_robonet_line({1}, false, [] {
        SimulatedUnitOptions u = sim_gateway_unit_defaults();
        u.time_scale = 50.0;
        return u;
    }());
    OpenOptions o;
    o.axis = axis_options();
    o.axis.method = CommandMethod::Iai;
    o.bus.min_gap_s = 0.0;
    o.line.kind = "robonet";
    Axis ax = Axis::open(line, o);
    ax.servo(false);
    CHECK(!line->unit(1)->servo_on());
    CHECK(ax.strategy() == CommandMethod::Coil);
    CHECK_EQ(ax.stop_path(), "coil");
    for (const auto& r : line->requests()) CHECK(r.second[1] != 0x45);
    CHECK_THROWS(ax.reset_alarm(CommandMethod::Iai), LineRuleError);  // an explicit IAI method is refused there
}

TEST(axis_strategy_rcon_status_bits_are_nullopt) {
    auto line = sim_rcon_line({1});
    OpenOptions o;
    o.axis = axis_options();
    o.bus.min_gap_s = 0.0;
    o.line.position_unit_mm = 0.01;
    Axis ax = Axis::open(line, o);
    const Status st = ax.status();
    CHECK(st.line == std::optional<std::string>("rcon"));
    CHECK(!st.nv_busy().has_value());
    CHECK(!st.at_backward_end().has_value());
    CHECK(st.manual_mode().has_value());
    std::vector<std::uint16_t> words(22, 0);
    words[5] = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Pend;
    words[7] = reg::kDssePush | reg::kDssePmss;
    words[20] = 0x03FF;
    Status rcon = Status::from_registers(words);
    rcon.line = "rcon";
    CHECK(rcon.position_complete_no() == std::optional<int>(0x7F));  // 7 bits on RCON (DOC ME0413 p.5-10)
    CHECK(rcon.push_contact());                                       // DSSE b0 undefined there: PEND + PUSH
    CHECK(Status::from_registers(words).position_complete_no() == std::optional<int>(0x3FF));
    CHECK(!Status::from_registers(words).push_contact());
    const Json j = to_json(rcon);
    REQUIRE(j.find("line") != nullptr);
    CHECK_EQ(j.find("line")->as_string(), "rcon");
}

TEST(axis_strategy_port_axis_picks_the_line_strategy) {
    auto rcon = sim_rcon_line({1});
    LineOptions lo;
    lo.position_unit_mm = 0.01;
    BusOptions bo;
    bo.min_gap_s = 0.0;
    Port port(rcon, "R", bo, lo);
    Axis ax(port, 1);
    CHECK(ax.strategy() == CommandMethod::Register);
    CHECK(ax.line_name() == std::optional<std::string>("rcon"));
    CHECK_EQ(ax.stop_path(), "register");
    CHECK(!ax.bus()->line()->allows(LineOperation::Coil));
    CHECK(!ax.bus()->line()->allows(LineOperation::IaiCommand));
    CHECK(!ax.bus()->line()->allows(LineOperation::Broadcast));
    auto direct = SimLine::with_slaves({1});
    Port plain(direct, "D", bo, LineOptions{});
    Axis d(plain, 1);
    CHECK(!d.line_name().has_value());
    CHECK_EQ(d.stop_path(), "coil");
    CHECK(d.strategy() == CommandMethod::Coil);
}

TEST(axis_strategy_port_axis_refuses_a_robonet_gateway) {
    auto line = sim_robonet_line({1}, true);
    BusOptions bo;
    bo.min_gap_s = 0.0;
    Port port(line, "G", bo, LineOptions{});
    CHECK_THROWS(Axis(port, 1), UnsupportedLineError);
}

// ------------------------------------------------------------------------------------------ the safety stop path

TEST(axis_strategy_timeout_stop_uses_the_coil_with_iai_strategies) {
    for (CommandMethod method : {CommandMethod::Iai, CommandMethod::Iai2}) {
        auto sim = make_sim(1.0);
        Axis ax = open_axis(sim);
        ax.set_strategy(method);
        sim->clear_requests();
        MoveOptions m;
        m.speed = 10.0;
        m.accel_g = 0.1;
        m.timeout_s = 0.2;
        bool noted = false;
        try {
            ax.move_to(140.0, m);
        } catch (const MotionTimeoutError& e) {
            for (const auto& n : e.notes())
                if (contains(n, "decel stop sent after this error")) noted = true;
        }
        CHECK(noted);
        CHECK_EQ(coil_trace(sim->requests(), reg::kCoilStop), "10");
        CHECK_EQ(count_fc(sim->requests(), 0x45) + count_fc(sim->requests(), 0x48), 0);
    }
}

TEST(axis_strategy_stop_works_when_the_unit_rejects_iai_commands) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 5.0;
    o.function_codes = std::vector<int>{0x03, 0x05, 0x06, 0x0F, 0x10, 0x11};  // 0x45 / 0x48: exception 01
    auto sim = std::make_shared<SimulatedUnit>(o);
    Axis ax = open_axis(sim);
    ax.set_strategy(CommandMethod::Iai2);
    MoveOptions m;
    m.speed = 50.0;
    m.wait = false;
    ax.move_to(140.0, m);
    sleep_at_least(0.05);
    const Status st = ax.stop(true);
    CHECK(!st.moving);
    CHECK(!sim->moving());
}

// ------------------------------------------------------------------------------------------ IAI 0x48 strategy

TEST(axis_strategy_iai2_servo_home_reset_frames) {
    // the frames are answered here (accepted) and their action done with the coils, so they are checked as sent
    auto sim = make_sim(50.0);
    auto t = std::make_shared<FilterTransport>(sim, [](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (f.size() < 5 || f[1] != 0x48) return std::nullopt;
        const int cmd = (f[3] << 8) | f[4];
        const int op1 = (f[5] << 24) | (f[6] << 16) | (f[7] << 8) | f[8];
        auto coil = [&](int address, bool on) {
            FilterTransport::exchange(inner, build_frame(f[0], {0x05, static_cast<std::uint8_t>(address >> 8),
                                                                 static_cast<std::uint8_t>(address & 0xFF),
                                                                 static_cast<std::uint8_t>(on ? 0xFF : 0x00), 0x00}));
        };
        if (cmd == reg::kIai2CmdServo) coil(reg::kCoilSon, op1 != 0);
        if (cmd == reg::kIai2CmdHome) {
            coil(reg::kCoilHome, false);
            coil(reg::kCoilHome, true);
            coil(reg::kCoilHome, false);
        }
        if (cmd == reg::kIai2CmdAlarmReset) {
            coil(reg::kCoilAlrs, true);
            coil(reg::kCoilAlrs, false);
        }
        Bytes pdu = {0x48, 0x00, 0x00};
        pdu.resize(19, 0x00);
        return build_frame(f[0], pdu);
    });
    Axis ax = open_axis(t);
    ax.set_strategy(CommandMethod::Iai2);
    t->clear_frames();
    ax.servo(false);
    ax.servo(true);
    ax.home();
    sim->trigger_alarm(0x0E0);
    ax.reset_alarm();
    std::vector<int> codes;
    for (const Bytes& f : t->frames())
        if (f.size() == 23 && f[1] == 0x48) codes.push_back((f[3] << 8) | f[4]);
    CHECK(codes == (std::vector<int>{0x30, 0x30, 0x31, 0x30, 0x23}));  // the alarm's servo OFF cleared before 0x23
    CHECK_EQ(count_fc(t->frames(), 0x05), 0);  // the axis itself wrote no coil
    CHECK(sim->homed());
    CHECK_EQ(sim->alarm_code(), 0);
}

// ------------------------------------------------------------------------------------------ action mode, safety speed

TEST(axis_strategy_action_mode_pc_and_pio) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    sim->clear_requests();
    Status st = ax.set_action_mode(true, true);
    CHECK(st.modbus_enabled);
    CHECK(st.safety_speed());
    st = ax.set_action_mode(true, false);
    CHECK(!st.safety_speed());
    st = ax.set_action_mode(true);  // nullopt keeps the present safety speed (off)
    CHECK(!st.safety_speed());
    const auto frames = sim->requests();
    int action = 0;
    for (const Bytes& f : frames)
        if (f.size() >= 8 && f[1] == 0x45 && f[3] == 0x0C) ++action;
    CHECK_EQ(action, 0);  // PC: the PMSL / SFTY coils, not 0x45 op1 = 1
    CHECK_EQ(count_fc(frames, 0x05), 2);  // SFTY on, SFTY off (PMSS was 1 already: no PMSL)
    sim->clear_requests();
    st = ax.set_action_mode(false, false);
    CHECK(!st.modbus_enabled);
    CHECK(!sim->pmsl());
    CHECK_EQ(count_fc(sim->requests(), 0x45), 1);  // PIO: 0x45 cmd 0x0C op1 = 0
    sim->clear_requests();
    st = ax.set_action_mode(true);  // from PIO: the PMSL coil
    CHECK(st.modbus_enabled);
    CHECK_EQ(count_fc(sim->requests(), 0x05), 1);
    CHECK_EQ(count_fc(sim->requests(), 0x45), 0);
}

TEST(axis_strategy_action_mode_pc_without_coils_falls_back_to_the_command) {
    // A unit that answers the PMSL coil with FC05 exception 01: PC by 0x45 cmd 0x0C op1 = 1, the one way left, and
    // PMSS still read back. Once the bus knows FC05 is refused (no_coils()), no coil is tried at all.
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 50.0;
    o.pmsl = false;
    auto sim = std::make_shared<SimulatedUnit>(o);
    auto hold_pio = std::make_shared<std::atomic<bool>>(false);  // op1 = 1 answered as accepted, PMSS stays 0
    auto t = std::make_shared<FilterTransport>(sim, [hold_pio](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (f.size() > 1 && f[1] == 0x05) return FilterTransport::exception(f[0], 0x05, 0x01);
        if (*hold_pio && f.size() == 10 && f[1] == 0x45 && f[3] == reg::kIaiCmdActionMode && f[5] == 1)
            return build_frame(f[0], {0x45, 0x00, 0x00});
        return std::nullopt;
    });
    auto action_ops = [&t] {  // op1 * 10 + op2 of every 0x45 cmd 0x0C frame
        std::vector<int> out;
        for (const Bytes& f : t->frames())
            if (f.size() == 10 && f[1] == 0x45 && f[3] == reg::kIaiCmdActionMode) out.push_back(f[5] * 10 + f[7]);
        return out;
    };
    Axis ax = open_axis(t);
    REQUIRE(!ax.status().modbus_enabled);
    t->clear_frames();
    Status st = ax.set_action_mode(true, false);
    CHECK(st.modbus_enabled);
    CHECK(sim->pmsl());
    CHECK_EQ(coil_trace(t->frames(), reg::kCoilPmsl), "1");  // tried once: exception 01
    CHECK(action_ops() == std::vector<int>{10});
    ax.set_action_mode(false, false);
    CHECK(!sim->pmsl());
    t->clear_frames();
    st = ax.set_action_mode(true, true);  // no_coils(): straight to the command, op2 = the safety speed
    CHECK(st.modbus_enabled);
    CHECK(st.safety_speed());
    CHECK_EQ(count_fc(t->frames(), 0x05), 0);
    CHECK(action_ops() == std::vector<int>{11});
    ax.set_action_mode(false, false);
    hold_pio->store(true);
    t->clear_frames();
    CHECK_THROWS(ax.set_action_mode(true, false), VerificationError);  // the fallback is verified too
    CHECK(action_ops() == std::vector<int>{10});
    CHECK(!sim->pmsl());
}

TEST(axis_strategy_action_mode_is_refused_while_moving) {
    auto sim = make_sim(5.0);
    Axis ax = open_axis(sim);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    CHECK(wait_for([&] { return sim->moving(); }));
    sim->clear_requests();
    CHECK_THROWS(ax.set_action_mode(false), EcNotReadyError);
    CHECK_EQ(count_fc(sim->requests(), 0x45), 0);
    ax.stop();
}

TEST(axis_strategy_safety_speed_by_coil) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    sim->clear_requests();
    Status st = ax.set_safety_speed(true);
    CHECK(st.safety_speed());
    CHECK(sim->safety_speed());
    CHECK_EQ(coil_trace(sim->requests(), reg::kCoilSfty), "1");
    st = ax.set_safety_speed(false);
    CHECK(!st.safety_speed());
    CHECK_EQ(coil_trace(sim->requests(), reg::kCoilSfty), "10");
    CHECK(drg1_writes(sim->requests()).empty());
}

TEST(axis_strategy_safety_speed_on_rcon_writes_drg1) {
    auto line = sim_rcon_line({1});
    OpenOptions o;
    o.axis = axis_options();
    o.bus.min_gap_s = 0.0;
    o.line.position_unit_mm = 0.01;
    Axis ax = Axis::open(line, o);
    line->clear_requests();
    const Status st = ax.set_safety_speed(true);
    CHECK(st.safety_speed());
    CHECK(line->unit(1)->safety_speed());
    bool drg1 = false;
    for (const auto& r : line->requests()) {
        CHECK(r.second[1] != 0x05);
        if (r.second.size() >= 6 && r.second[1] == 0x06 && r.second[2] == 0x0D && r.second[3] == 0x00) drg1 = true;
    }
    CHECK(drg1);
}

// ------------------------------------------------------------------------------------------ teach with the coils

namespace {

/// The MOD / TEAC coils and POSR answered here (the simulator does not model them); DSS2 b10 PWRT shows while TEAC is
/// on (pwrt false: it never rises). The row is written into the simulator directly when TEAC rises.
std::shared_ptr<FilterTransport> teach_line(const std::shared_ptr<SimulatedUnit>& sim, bool pwrt = true) {
    struct State {
        bool teac = false;
        int posr = 0;
    };
    auto state = std::make_shared<State>();
    return std::make_shared<FilterTransport>(sim, [state, sim, pwrt](const Bytes& f, Transport& inner) -> std::optional<Bytes> {
        if (f.size() < 8) return std::nullopt;
        const int address = (f[2] << 8) | f[3];
        if (f[1] == 0x05 && (address == reg::kCoilMod || address == reg::kCoilTeac)) {
            if (address == reg::kCoilTeac) {
                const bool on = f[4] == 0xFF;
                if (on && !state->teac) {
                    std::vector<std::uint16_t> row = sim->position_row(state->posr);
                    const auto words = reg::to_i32_words(static_cast<std::int32_t>(std::lround(sim->position_mm() * 100)));
                    row[0] = words.first;
                    row[1] = words.second;
                    if (row[4] == 0 && row[5] == 0) row[5] = 5000;  // speed 50.00
                    if (row[2] == 0 && row[3] == 0) row[3] = 10;    // band 0.10
                    if (row[6] == 0) row[6] = 30;
                    if (row[7] == 0) row[7] = 30;
                    row[reg::kPosChecksum] = reg::position_checksum(row);  // the unit writes a valid row
                    sim->set_position_row(state->posr, row);
                }
                state->teac = on;
            }
            return Bytes(f.begin(), f.end());  // the echo
        }
        if (f[1] == 0x06 && address == reg::kPosr) {
            state->posr = (f[4] << 8) | f[5];
            return Bytes(f.begin(), f.end());
        }
        if (f[1] == 0x03 && address == reg::kDss2 && ((f[4] << 8) | f[5]) == 1) {
            Bytes reply = FilterTransport::exchange(inner, f);
            if (reply.size() == 7 && state->teac && pwrt) {
                reply[3] = static_cast<std::uint8_t>(reply[3] | (reg::kDss2Pwrt >> 8));
                reply.resize(5);
                append_crc(reply);
            }
            return reply;
        }
        return std::nullopt;
    });
}

std::vector<std::string> teach_sequence(const std::vector<Bytes>& frames) {
    std::vector<std::string> out;
    for (const Bytes& f : frames) {
        if (f.size() < 6) continue;
        const int address = (f[2] << 8) | f[3];
        if (f[1] == 0x05 && address == reg::kCoilMod) out.push_back(f[4] == 0xFF ? "MOD on" : "MOD off");
        if (f[1] == 0x05 && address == reg::kCoilTeac) out.push_back(f[4] == 0xFF ? "TEAC on" : "TEAC off");
        if (f[1] == 0x06 && address == reg::kPosr) out.push_back("POSR " + std::to_string((f[4] << 8) | f[5]));
    }
    return out;
}

}  // namespace

TEST(axis_strategy_teach_by_coils_sequence) {
    auto sim = make_sim(50.0);
    auto t = teach_line(sim);
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    ax.move_to(42.0, m);
    NvWriteOptions nv;  // no opt-in needed
    t->clear_frames();
    const long before = ax.nonvolatile_writes();
    const PositionWriteResult r = ax.teach_position(1, TeachMethod::Coil, nv);
    CHECK(!r.skipped);
    CHECK_NEAR(r.entry.position_mm, 42.0, 0.011);
    CHECK(teach_sequence(t->frames()) ==
          (std::vector<std::string>{"MOD on", "POSR 1", "TEAC on", "TEAC off", "MOD off"}));
    CHECK_EQ(ax.nonvolatile_writes(), before + 1);
    // the row now holds the position and is complete: nothing is written again
    t->clear_frames();
    const PositionWriteResult again = ax.teach_position(1, TeachMethod::Coil, nv);
    CHECK(again.skipped);
    CHECK_EQ(again.note, std::string(kNvUnchangedNote));
    CHECK(teach_sequence(t->frames()).empty());
    nv.force = true;
    const PositionWriteResult forced = ax.teach_position(1, TeachMethod::Coil, nv);
    CHECK(!forced.skipped);
    CHECK(forced.forced);
}

TEST(axis_strategy_teach_by_coils_releases_the_coils_when_pwrt_stays_low) {
    auto sim = make_sim(50.0);
    auto t = teach_line(sim, false);
    Axis ax = open_axis(t);
    MoveOptions m;
    m.speed = 50.0;
    ax.move_to(33.0, m);
    NvWriteOptions nv;
    nv.allow_nonvolatile = true;
    t->clear_frames();
    CHECK_THROWS(ax.teach_position(2, TeachMethod::Coil, nv), VerificationError);
    CHECK(teach_sequence(t->frames()) ==
          (std::vector<std::string>{"MOD on", "POSR 2", "TEAC on", "TEAC off", "MOD off"}));
}

TEST(axis_strategy_teach_by_coils_is_refused_without_coils_and_before_homing) {
    auto line = sim_rcon_line({1});
    OpenOptions o;
    o.axis = axis_options();
    o.bus.min_gap_s = 0.0;
    o.line.position_unit_mm = 0.01;
    Axis ax = Axis::open(line, o);
    NvWriteOptions nv;
    nv.allow_nonvolatile = true;
    line->clear_requests();
    CHECK_THROWS(ax.teach_position(0, TeachMethod::Coil, nv), LineRuleError);
    for (const auto& r : line->requests()) CHECK(r.second[1] == 0x03);  // reads only
}

// ------------------------------------------------------------------------------------------ limits: the push speed

TEST(axis_strategy_limits_push_speed_entry) {
    auto sim = make_sim();
    Axis ax = open_axis(sim);
    const AxisRanges r = ax.limits();
    REQUIRE(r.push.speed.has_value());
    CHECK(*r.push.speed > 0.0);
    CHECK_EQ(r.push.speed_unit, "mm/s");
    CHECK(!r.push.speed_source.empty());
    CHECK(ax.push_speed().first == *r.push.speed);
    const Json j = to_json(r);
    const Json* push = j.find("push");
    REQUIRE(push != nullptr);
    CHECK(push->find("speed") != nullptr);
    CHECK(push->find("speed_source") != nullptr);
    CHECK(j.find("speed")->find("speed_source") == nullptr);  // only the push entry has it
}
