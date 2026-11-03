// Axis unit: the motion extras on the simulator - push (PPOW, direction, contact / miss with the settle rule, the
// refusals that send nothing, the force recommendation, the unit's push speed, the CON automatic servo OFF warning),
// the command-profile move with its own deceleration (FC17 0x0F00 + PCRF), acceleration patterns, change_speed /
// change_accel of a running move and move_sequence with and without blending. Ports of the Python edition's
// test_motion.py PushTest, test_settle.py PushSettleTest, test_axis_features.py (PushPower, AutoServoOff, ChangeMove,
// AccelPattern, PushSpeed), test_universal.py (profile moves) and test_fast.py (move_sequence).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile, double scale = 50.0) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    return std::make_shared<SimulatedUnit>(o);
}

/// The caps of the Python tests' make_ec: stroke 0..150, 100 mm/s, 0.3 G, 5 ms polls.
AxisOptions ec_caps(double max_speed = 100.0) {
    AxisOptions a;
    a.stroke = Span{0.0, 150.0};
    a.max_speed_mm_s = max_speed;
    a.max_accel_g = 0.3;
    a.poll_interval_s = 0.005;
    return a;
}

AxisOptions no_caps(double poll = 0.005) {
    AxisOptions a;
    a.poll_interval_s = poll;
    return a;
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim, AxisOptions axis = ec_caps(), FrameLogger logger = {}) {
    OpenOptions o;
    o.slave = sim->slave();
    o.axis = axis;
    o.bus.min_gap_s = 0.0;
    o.bus.frame_logger = std::move(logger);
    return Axis::open(sim, o);
}

int be16(const Bytes& f, std::size_t at) { return (f[at] << 8) | f[at + 1]; }
std::uint32_t be32(const Bytes& f, std::size_t at) {
    return (static_cast<std::uint32_t>(be16(f, at)) << 16) | static_cast<std::uint32_t>(be16(f, at + 2));
}

/// One 0x9900 block write: (pcmd, inp, vcmd, acmd, ppow, ctlf).
struct Move {
    std::int32_t pcmd;
    std::uint32_t inp, vcmd;
    int acmd, ppow, ctlf;
};

std::vector<Move> moves(const std::vector<Bytes>& frames) {
    std::vector<Move> out;
    for (const auto& f : frames)
        if (f.size() >= 27 && f[1] == 0x10 && be16(f, 2) == reg::kDirectMoveBase && be16(f, 4) == 9)
            out.push_back({static_cast<std::int32_t>(be32(f, 7)), be32(f, 11), be32(f, 15), be16(f, 19), be16(f, 21),
                           be16(f, 23)});
    return out;
}

bool same(const Move& m, std::int32_t pcmd, std::uint32_t inp, std::uint32_t vcmd, int acmd, int ppow, int ctlf) {
    return m.pcmd == pcmd && m.inp == inp && m.vcmd == vcmd && m.acmd == acmd && m.ppow == ppow && m.ctlf == ctlf;
}

std::vector<Bytes> function_frames(const std::vector<Bytes>& frames, int fc) {
    std::vector<Bytes> out;
    for (const auto& f : frames)
        if (f.size() > 1 && f[1] == fc) out.push_back(f);
    return out;
}

bool is_write(const Bytes& f) {
    return f.size() > 1 && (f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x10 || f[1] == 0x17 || f[1] == 0x45);
}

int write_frames(const std::vector<Bytes>& frames) {
    return static_cast<int>(std::count_if(frames.begin(), frames.end(), is_write));
}

/// FC05 values of coil `address` in order ("1" ON, "0" OFF).
std::string coil_trace(const std::vector<Bytes>& frames, int address) {
    std::string out;
    for (const auto& f : function_frames(frames, 0x05))
        if (be16(f, 2) == address) out += f[4] == 0xFF ? '1' : '0';
    return out;
}

/// FC10 straight to the unit (no Axis): parameter RAM, as the Python tests poke sim._param_ram.
void write_ram(SimulatedUnit& sim, int address, const std::vector<std::uint16_t>& words) {
    Bytes pdu = {0x10, static_cast<std::uint8_t>(address >> 8), static_cast<std::uint8_t>(address & 0xFF), 0,
                 static_cast<std::uint8_t>(words.size()), static_cast<std::uint8_t>(2 * words.size())};
    for (std::uint16_t w : words) {
        pdu.push_back(static_cast<std::uint8_t>(w >> 8));
        pdu.push_back(static_cast<std::uint8_t>(w & 0xFF));
    }
    const Bytes answer = sim.handle_frame(build_frame(sim.slave(), pdu));
    REQUIRE(answer.size() > 2 && answer[1] == 0x10);
}

/// The register address of a parameter item ("8.14") as the axis reports it.
int item_address(Axis& ax, const std::string& key) {
    const ParameterInfo info = ax.read_parameter(key);
    REQUIRE(info.address.has_value());
    return std::stoi(*info.address, nullptr, 16);
}

bool wait_for(const std::function<bool()>& predicate, double timeout_s = 3.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// Collects warnings while alive (the Python tests' assertLogs).
struct LogCapture {
    std::vector<std::string> lines;
    LogHandler previous;
    LogCapture() {
        previous = set_log_handler([this](LogLevel level, const std::string&, const std::string& message) {
            if (level >= LogLevel::Warning) lines.push_back(message);
        });
    }
    ~LogCapture() { set_log_handler(previous); }
    bool any(const std::string& part) const {
        return std::any_of(lines.begin(), lines.end(), [&](const std::string& l) { return contains(l, part); });
    }
};

std::string what_of(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "nothing thrown";
}

/// A SimulatedUnit on a fake clock that advances step_s on every request (Python SteppedClockSim): each poll sees the
/// axis exactly step_s later, whatever the host's speed.
class SteppedClockSim : public SimulatedUnit {
public:
    using SimulatedUnit::write;
    static std::shared_ptr<SteppedClockSim> make(SimulatedUnitOptions options, double step_s) {
        auto t = std::make_shared<double>(100.0);
        options.clock = [t] { return *t; };
        return std::shared_ptr<SteppedClockSim>(new SteppedClockSim(options, t, step_s));
    }
    void write(const std::uint8_t* data, std::size_t size) override {
        if (on_write) on_write(Bytes(data, data + size));  // the unit as the request reaches it
        *t_ += step_;
        SimulatedUnit::write(data, size);
    }
    /// Called with every request frame as it reaches the unit, before the clock steps (set before the unit is used).
    std::function<void(const Bytes&)> on_write;

private:
    SteppedClockSim(const SimulatedUnitOptions& options, std::shared_ptr<double> t, double step_s)
        : SimulatedUnit(options), t_(std::move(t)), step_(step_s) {}
    std::shared_ptr<double> t_;
    double step_;
};

/// The statuses the axis read (decoded from the 22-word 0x9000 answers the frame logger saw).
struct StatusRecorder {
    std::shared_ptr<std::vector<Status>> seen = std::make_shared<std::vector<Status>>();
    std::shared_ptr<bool> pending = std::make_shared<bool>(false);
    FrameLogger logger() {
        auto s = seen;
        auto p = pending;
        return [s, p](FrameDirection d, const Bytes& f) {
            if (d == FrameDirection::Tx) {
                *p = f.size() >= 6 && f[1] == 0x03 && be16(f, 2) == reg::kMonitorBase && be16(f, 4) == 22;
            } else if (d == FrameDirection::Rx && *p && f.size() == 5 + 44 && f[1] == 0x03) {
                std::vector<std::uint16_t> words;
                for (int i = 0; i < 22; ++i) words.push_back(static_cast<std::uint16_t>(be16(f, 3 + 2 * i)));
                s->push_back(Status::from_registers(words));
                *p = false;
            }
        };
    }
};

PushOptions push_opts(int direction = 1, double speed = 10.0, bool wait = true) {
    PushOptions p;
    p.direction = direction;
    p.speed = speed;
    p.wait = wait;
    return p;
}

MoveOptions mv(double speed, double accel = 0.1, bool wait = true) {
    MoveOptions o;
    o.speed = speed;
    o.accel_g = accel;
    o.wait = wait;
    return o;
}

}  // namespace

// ============================================================================================ push (test_motion.py)

TEST(axis_push_missed_away_from_home) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    const auto result = ax.push(80.0, 10.0, 30.0);
    REQUIRE(result.has_value());
    CHECK(result->missed());
    CHECK(!result->contact());
    CHECK_NEAR(result->position_mm, 90.0, 1e-9);  // PCMD + band
    CHECK(result->status.push_missed());
    CHECK(!result->status.in_position);
    const Json json = to_json(*result);
    CHECK(contains(json.dump(), "\"outcome\": \"missed\"") || contains(json.dump(), "\"outcome\":\"missed\""));
    const auto m = moves(sim->requests());
    REQUIRE(!m.empty());
    // PPOW 30 % = 76 (as sent on hardware, VERIFIED in the Python edition), CTLF push + away from home
    CHECK(same(m.back(), 8000, 1000, 1000, 10, 76, 0x0006));
    CHECK_EQ(reg::push_power(20), reg::kPpowMin);
    CHECK_EQ(reg::push_power(70), reg::kPpowMax);
}

TEST(axis_push_toward_home) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    const auto result = ax.push(60.0, 10.0, 25.0, push_opts(-1));
    REQUIRE(result.has_value());
    CHECK(result->missed());
    CHECK_NEAR(result->position_mm, 50.0, 1e-9);
    CHECK_EQ(moves(sim->requests()).back().ctlf, reg::kCtlfPush);
}

TEST(axis_push_contact_then_retract) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    sim->set_obstacle_mm(85.0);
    const auto result = ax.push(80.0, 20.0, 30.0);  // a push band longer than a move band (10)
    REQUIRE(result.has_value());
    CHECK(result->contact());
    CHECK_NEAR(result->position_mm, 85.0, 1e-9);
    CHECK_EQ(result->status.raw.dsse, 0x37C3);  // live value after contact
    CHECK_EQ(result->status.raw.dss1, 0x3099);  // live 0x7099 minus SFTY
    CHECK_NEAR(ax.move_to(75.0, 20.0)->position_mm, 75.0, 0.1);
}

TEST(axis_push_start_inside_or_past_the_zone_is_refused) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);  // at 73.95
    sim->clear_requests();
    const std::pair<double, int> cases[] = {{70.0, 1}, {65.0, 1}, {78.0, -1}, {90.0, -1}};
    for (const auto& c : cases) {
        const std::string text = what_of([&] { ax.push(c.first, 10.0, 30.0, push_opts(c.second)); });
        CHECK(contains(text, "push zone"));
        CHECK_THROWS(ax.push(c.first, 10.0, 30.0, push_opts(c.second)), EcNotReadyError);
    }
    CHECK(moves(sim->requests()).empty());
}

TEST(axis_push_validation_sends_nothing) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    sim->clear_requests();
    CHECK_THROWS(ax.push(145.0, 10.0, 30.0), SoftLimitError);                 // band end 155 outside
    CHECK_THROWS(ax.push(80.0, 10.0, 75.0), SoftLimitError);                  // above the 70 % fallback
    CHECK_THROWS(ax.push(80.0, 10.0, 61.0), SoftLimitError);                  // above the slider's MXPP (7.9) 60 %
    CHECK_THROWS(ax.push(80.0, 10.0, 0.1), SoftLimitError);                   // PPOW 0: no force at all
    CHECK_THROWS(ax.push(80.0, 10.0, std::nan("")), SoftLimitError);
    CHECK_THROWS(ax.push(80.0, 10.0, 30.0, push_opts(0)), SoftLimitError);   // direction
    CHECK_THROWS(ax.push(80.0, 0.0, 30.0), SoftLimitError);                   // band 0
    CHECK_THROWS(ax.push(80.0, 10.0, 30.0, push_opts(1, 500.0)), SoftLimitError);  // above max_speed_mm_s 100
    CHECK(contains(what_of([&] { ax.push(80.0, 10.0, 61.0); }), "MXPP, item 7.9"));
    CHECK(moves(sim->requests()).empty());
    CHECK_EQ(write_frames(sim->requests()), 0);
    AxisOptions capped = ec_caps();
    capped.max_push_pct = 40.0;
    ax.set_options(capped);
    CHECK_THROWS(ax.push(80.0, 10.0, 50.0), SoftLimitError);
    CHECK(contains(what_of([&] { ax.push(80.0, 10.0, 50.0); }), "max_push_pct"));
    CHECK(moves(sim->requests()).empty());
}

TEST(axis_push_force_below_the_recommendation_warns) {
    // MNPP (item 7.58; 20 % on the slider) is only a recommendation; the EC-R6S rod ships a 10 % push row (VERIFIED in
    // the Python edition)
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    std::optional<PushResult> result;
    {
        LogCapture logs;
        result = ax.push(80.0, 10.0, 10.0);
        CHECK(logs.any("recommended minimum"));
    }
    REQUIRE(result.has_value());
    CHECK(result->missed());
    CHECK_EQ(moves(sim->requests()).back().ppow, 25);  // floor(10 x 255 / 100) (DOC ME0162-12A p.5-132)
    REQUIRE(result->warnings.size() == 1);
    CHECK(contains(result->warnings[0], "recommended minimum 20 % (MNPP, item 7.58)"));
    const auto push_range = ax.limits().push;
    REQUIRE(push_range.max.has_value());
    CHECK_NEAR(*push_range.max, 60.0, 1e-9);  // the slider's MXPP
    const auto again = ax.push(100.0, 10.0, 60.0);  // from 90.00: the approach side of 100..110
    REQUIRE(again.has_value());
    CHECK(again->warnings.empty());
}

TEST(axis_push_refused_when_the_unit_cannot_push) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis first = open_sim(sim);
    const int mxpp = item_address(first, "7.9");  // 0x0710 on the slider
    CHECK_EQ(mxpp, 0x0710);
    write_ram(*sim, mxpp, {0, 0});  // MXPP = 0: no push (UNVERIFIED rule)
    Axis ax(first.bus(), sim->slave(), ec_caps());  // fresh caches on the same bus
    sim->clear_requests();
    const std::string text = what_of([&] { ax.push(80.0, 10.0, 30.0); });
    CHECK(contains(text, "cannot push"));
    CHECK_THROWS(ax.push(80.0, 10.0, 30.0), SoftLimitError);
    CHECK(moves(sim->requests()).empty());
    CHECK(ax.limits().push.available == std::optional<bool>(false));
}

TEST(axis_push_needs_a_stopped_axis) {
    auto sim = make_sim(SimProfile::Slider, 20.0);
    Axis ax = open_sim(sim);
    ax.move_to(140.0, mv(20.0, 0.1, false));
    const std::string text = what_of([&] { ax.push(145.0, 3.0, 30.0); });
    CHECK(contains(text, "stopped axis"));
    ax.stop();
}

TEST(axis_push_sends_0x7f_for_50_percent) {
    auto sim = make_sim(SimProfile::Rod);
    Axis ax = open_sim(sim);
    ax.push(80.0, 10.0, 50.0);
    const auto m = moves(sim->requests());
    REQUIRE(!m.empty());
    CHECK_EQ(m.back().ppow, 0x007F);
}

// ============================================================================================ push settle (test_settle.py)

namespace {

struct SettleRig {
    std::shared_ptr<SteppedClockSim> sim;
    StatusRecorder rec;
    Axis ax;
};

std::unique_ptr<SettleRig> settle_rig(double push_settle_s = kPushSettleSDefault, bool blip = true) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 1.0;
    if (blip) o.push_blip_mm = 6.76;
    auto sim = SteppedClockSim::make(o, 0.005);
    StatusRecorder rec;
    AxisOptions a = no_caps(0.002);
    a.push_settle_s = push_settle_s;
    Axis ax = open_sim(sim, a, rec.logger());
    return std::unique_ptr<SettleRig>(new SettleRig{sim, rec, std::move(ax)});
}

/// The push of the settle tests: + 30 %, band 5 mm from 2.00 to 7.00 mm at 10 mm/s, as on the gripper. The timeout is
/// generous: on the stepped clock simulated time advances per request, not with the real time the timeout counts.
std::optional<PushResult> settle_push(Axis& ax) {
    PushOptions o = push_opts(1, 10.0);
    o.timeout_s = 60.0;
    return ax.push(2.0, 5.0, 30.0, o);
}

}  // namespace

TEST(axis_push_settle_transient_contact_blip_is_a_miss) {
    // On the EC-GRBP8M gripper a 30 % push with a 5 mm band showed the full contact fingerprint (PEND + MEND, DSSE
    // 0x37C3) at 6.76 mm and then missed at 7.00 mm (VERIFIED in the Python edition).
    auto rig = settle_rig();
    const auto result = settle_push(rig->ax);
    REQUIRE(result.has_value());
    int blips = 0;
    for (const Status& s : *rig->rec.seen) {
        if (!s.push_contact()) continue;
        ++blips;
        CHECK_EQ(s.raw.dsse, 0x37C3);
        CHECK_EQ(s.raw.dss1 & 0x0809, 0x0009);
    }
    CHECK(blips > 0);  // the poller saw the transient contact fingerprint
    CHECK_EQ(result->outcome, std::string(kPushMissed));
    CHECK_NEAR(result->position_mm, 7.0, 1e-9);
    CHECK(result->status.push_missed());
    CHECK(!rig->sim->moving());
}

TEST(axis_push_settle_without_settling_the_blip_reads_as_contact) {
    auto rig = settle_rig(0.0);  // push_settle_s = 0: an empty grip reads as "contact"
    const auto result = settle_push(rig->ax);
    REQUIRE(result.has_value());
    CHECK_EQ(result->outcome, std::string(kPushContact));
    CHECK(result->position_mm >= 6.76 && result->position_mm <= 6.86);
    CHECK(rig->sim->moving());
    rig->ax.stop();
}

TEST(axis_push_settle_real_contact_still_reported) {
    auto rig = settle_rig(kPushSettleSDefault, false);
    rig->sim->set_obstacle_mm(5.0);
    const auto result = settle_push(rig->ax);
    REQUIRE(result.has_value());
    CHECK_EQ(result->outcome, std::string(kPushContact));
    CHECK_NEAR(result->position_mm, 5.0, 1e-9);
    CHECK(result->status.push_contact());
    MoveOptions retract = mv(10.0);
    retract.timeout_s = 60.0;
    CHECK_NEAR(rig->ax.move_to(1.0, retract)->position_mm, 1.0, 0.1);  // retract ends the hold
}

// ============================================================================ push extras (test_axis_features.py)

TEST(axis_push_con_warns_when_auto_servo_off_is_enabled) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.identify();
    const int address = item_address(ax, "8.14");
    const auto words = ax.read_registers(address, 2);
    const std::uint32_t raw = reg::from_u32_words(words[0], words[1]) | (1u << 8);  // No.53 = 1
    const auto w = reg::to_u32_words(raw);
    write_ram(*sim, address, {w.first, w.second});
    const auto result = ax.push(80.0, 10.0, 30.0, push_opts(1, 20.0));
    REQUIRE(result.has_value());
    const bool warned = std::any_of(result->warnings.begin(), result->warnings.end(),
                                    [](const std::string& t) { return contains(t, "automatic servo OFF"); });
    CHECK(warned);
}

TEST(axis_push_speed_from_the_unit) {
    const std::pair<SimProfile, double> expected[] = {
        {SimProfile::Slider, 30.0}, {SimProfile::Rod, 20.0}, {SimProfile::Gripper, 5.0}, {SimProfile::Pcon, 20.0}};
    for (const auto& e : expected) {
        auto sim = make_sim(e.first);
        Axis ax = open_sim(sim);
        const auto speed = ax.push_speed();
        CHECK_NEAR(speed.first, e.second, 1e-9);
        CHECK_EQ(speed.second, std::string("PSHV 8.15"));
        const auto commanded = ax.push_speed(3.0);
        CHECK_NEAR(commanded.first, 3.0, 1e-9);
        CHECK_EQ(commanded.second, std::string("the commanded speed"));
    }
}

TEST(axis_push_gripper_runs_at_5_mm_s_and_the_timeout_covers_it) {
    auto sim = make_sim(SimProfile::Gripper, 20.0);
    Axis ax = open_sim(sim);
    ax.move_to(3.0, 20.0);
    const auto started = std::chrono::steady_clock::now();
    const auto result = ax.push(4.0, 5.0, 60.0, push_opts(1, 20.0));  // 5 mm at 5 mm/s = 1 s simulated
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    REQUIRE(result.has_value());
    CHECK(result->missed());
    CHECK_NEAR(sim->push_speed_mm_s(), 5.0, 1e-9);
    CHECK(elapsed * 20.0 >= 1.0);
}

TEST(axis_push_speed_fallback_without_the_items) {
    auto sim = make_sim(SimProfile::Slider);
    Axis first = open_sim(sim);
    write_ram(*sim, item_address(first, "8.15"), {0, 0});  // PSHV 0: not a speed
    write_ram(*sim, item_address(first, "7.8"), {0, 0});   // MXPH 0
    Axis ax(first.bus(), sim->slave(), ec_caps());
    const auto speed = ax.push_speed();
    CHECK_NEAR(speed.first, reg::kPushSpeedMaxMmS, 1e-9);
    CHECK(contains(speed.second, "fallback"));
}

// ============================================================================================ mid-move changes

TEST(axis_change_speed_and_accel_resend_the_full_block) {
    auto sim = make_sim(SimProfile::Slider, 5.0);
    Axis ax = open_sim(sim);
    ax.move_to(140.0, mv(20.0, 0.1, false));
    CHECK(wait_for([&] { return sim->position_mm() > 75.0; }));
    CHECK(!ax.change_speed(60.0).has_value());
    auto m = moves(sim->requests());
    REQUIRE(m.size() >= 2);
    CHECK(same(m[m.size() - 2], 14000, 10, 2000, 10, 0, 0));
    CHECK(same(m.back(), 14000, 10, 6000, 10, 0, 0));  // same target, band, accel, CTLF; new VCMD
    CHECK_NEAR(sim->command_speed_mm_s(), 60.0, 1e-9);
    ax.change_accel(0.2);
    CHECK(same(moves(sim->requests()).back(), 14000, 10, 6000, 20, 0, 0));
    const Status st = ax.wait_idle(140.0, std::nullopt, 10.0);
    CHECK_NEAR(st.position_mm, 140.0, 0.1);
}

TEST(axis_change_refused_without_a_move) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    CHECK_THROWS(ax.change_speed(30.0), EcNotReadyError);
    CHECK_THROWS(ax.change_accel(0.2), EcNotReadyError);
    ax.push(80.0, 10.0, 30.0, push_opts(1, 10.0, false));
    sim->clear_requests();
    CHECK_THROWS(ax.change_speed(5.0), EcNotReadyError);  // a push
    CHECK(moves(sim->requests()).empty());
    ax.stop();
    ax.move_to(140.0, mv(20.0, 0.1, false));
    sim->clear_requests();
    CHECK_THROWS(ax.change_speed(1000.0), SoftLimitError);  // above the cap: nothing sent
    CHECK(moves(sim->requests()).empty());
    ax.stop();
    ax.move_to(100.0, mv(50.0, 0.3, true));
    MoveOptions profile = mv(50.0, 0.3, false);
    profile.decel_g = 0.1;
    ax.move_to(140.0, profile);
    CHECK_THROWS(ax.change_speed(20.0), EcNotReadyError);  // a command-profile move
    ax.stop();
}

TEST(axis_change_ec_smooth_accel_guard) {
    auto sim = make_sim(SimProfile::Slider, 1.0);
    Axis ax = open_sim(sim, ec_caps(700.0));
    REQUIRE(ax.is_ec());
    write_ram(*sim, item_address(ax, "8.14"), {0x0000, 0x0040});  // P5 smooth acceleration on
    ax.move_to(140.0, mv(250.0, 0.3, false));
    {
        LogCapture logs;
        ax.change_speed(200.0);
        CHECK(logs.any("overload"));
    }
    const std::string text = what_of([&] { ax.change_accel(0.01); });  // 200 / (0.01 x 9800) = 2.04 s
    CHECK(contains(text, "2 s"));
    CHECK(contains(text, "smooth acceleration"));
    ax.stop();
}

TEST(axis_change_con_s_motion_warns) {
    auto sim = make_sim(SimProfile::Pcon, 5.0);
    Axis ax = open_sim(sim);
    ax.identify();
    MoveOptions o = mv(20.0, 0.3, false);
    o.accel_pattern = AccelPattern::SMotion;
    ax.move_to(140.0, o);
    {
        LogCapture logs;
        ax.change_speed(40.0);
        CHECK(logs.any("trapezoid"));
    }
    CHECK_EQ(moves(sim->requests()).back().ctlf, reg::kCtlfModSMotion);
    ax.stop();
}

// ============================================================================================ acceleration patterns

TEST(axis_accel_pattern_ctlf_bits) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    const std::pair<std::optional<AccelPattern>, int> cases[] = {
        {AccelPattern::SMotion, 0x0040}, {AccelPattern::Filter, 0x0080}, {AccelPattern::Trapezoid, 0}, {std::nullopt, 0}};
    for (const auto& c : cases) {
        MoveOptions o = mv(50.0, 0.3);
        o.accel_pattern = c.first;
        ax.move_to(c.first == AccelPattern::Filter ? 100.0 : 80.0, o);
        CHECK_EQ(moves(sim->requests()).back().ctlf, c.second);
        CHECK_EQ(sim->accel_pattern(), std::string(c.first ? to_string(*c.first) : "trapezoid"));
        ax.move_to(60.0, 50.0);
    }
    CHECK(accel_pattern_from_string("s_motion") == std::optional<AccelPattern>(AccelPattern::SMotion));
    CHECK(!accel_pattern_from_string("jerk").has_value());
}

TEST(axis_accel_pattern_refusals) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim, ec_caps(700.0));
    sim->clear_requests();
    MoveOptions slow = mv(250.0, 0.01);  // 2.55 s of acceleration
    slow.accel_pattern = AccelPattern::SMotion;
    CHECK_THROWS(ax.move_to(100.0, slow), SoftLimitError);
    CHECK_THROWS(ax.check_move(100.0, slow), SoftLimitError);
    CHECK(contains(what_of([&] { ax.move_to(100.0, slow); }), "S-motion"));
    CHECK(moves(sim->requests()).empty());
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 50.0;
    o.move_patterns = 0x0001;  // MVPT: trapezoid only
    auto trapezoid_only = std::make_shared<SimulatedUnit>(o);
    Axis ax2 = open_sim(trapezoid_only, ec_caps(700.0));
    trapezoid_only->clear_requests();
    MoveOptions filter = mv(50.0);
    filter.accel_pattern = AccelPattern::Filter;
    CHECK(contains(what_of([&] { ax2.move_to(100.0, filter); }), "MVPT"));
    CHECK_THROWS(ax2.move_to(100.0, filter), SoftLimitError);
    CHECK(moves(trapezoid_only->requests()).empty());
}

TEST(axis_accel_pattern_runs_on_an_ec) {  // VERIFIED in the Python edition on the EC-GRBP8M gripper
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    REQUIRE(ax.is_ec());
    MoveOptions o = mv(50.0);
    o.accel_pattern = AccelPattern::SMotion;
    const auto st = ax.move_to(100.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 100.0, 0.1);
    CHECK_EQ(sim->accel_pattern(), std::string("s_motion"));
}

TEST(axis_accel_pattern_pause_warns_during_s_motion) {
    auto sim = make_sim(SimProfile::Pcon, 5.0);
    Axis ax = open_sim(sim);
    MoveOptions o = mv(20.0, 0.3, false);
    o.accel_pattern = AccelPattern::SMotion;
    ax.move_to(140.0, o);
    {
        LogCapture logs;
        ax.pause();
        CHECK(logs.any("S-motion"));
    }
    ax.resume();
    ax.stop();
}

// ============================================================================================ command-profile move

TEST(axis_profile_move_with_its_own_deceleration) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim, no_caps());
    ax.identify();
    sim->clear_requests();
    MoveOptions o = mv(100.0, 0.3);
    o.decel_g = 0.1;
    const auto st = ax.move_to(120.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 120.0, 0.1);
    CHECK_EQ(sim->profile_moves(), 1);
    const auto fc17 = function_frames(sim->requests(), 0x17);
    REQUIRE(fc17.size() == 1);
    const Bytes& f = fc17[0];
    REQUIRE(f.size() == 11 + 40 + 2);
    CHECK_EQ(be16(f, 2), 0x9000);
    CHECK_EQ(be16(f, 4), 34);
    CHECK_EQ(be16(f, 6), 0x0F00);
    CHECK_EQ(be16(f, 8), 20);
    std::vector<int> words;
    for (int i = 0; i < 20; ++i) words.push_back(be16(f, static_cast<std::size_t>(11 + 2 * i)));
    CHECK_EQ(reg::from_i32_words(words[0], words[1]), 12000);  // PCMD 120.00
    CHECK_EQ(reg::from_u32_words(words[4], words[5]), 30u);    // ACMD
    CHECK_EQ(reg::from_u32_words(words[6], words[7]), 10u);    // DCMD
    CHECK_EQ(words[10], 0x59);  // SPOW / DPOW kept from the unit's block
    CHECK_EQ(words[11], 0xFF);
    for (int i = 12; i < 20; ++i) CHECK_EQ(words[i], 0);  // PPOW, CTLF, LPOW, CTF2, zones 0
    CHECK_EQ(coil_trace(sim->requests(), reg::kCoilPcrf), std::string("1"));  // ON only: the coil clears itself
    CHECK(function_frames(sim->requests(), 0x10).empty());  // not the 0x9900 path
}

TEST(axis_profile_move_retarget_and_stop) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim, no_caps());
    MoveOptions o = mv(50.0, 0.3, false);
    o.decel_g = 0.2;
    ax.move_to(140.0, o);
    CHECK(sim->moving());
    o.wait = true;
    const auto st = ax.move_to(70.0, o);  // a new profile + PCRF retargets
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 70.0, 0.1);
    o.wait = false;
    ax.move_to(140.0, o);
    ax.stop();
    CHECK(!sim->moving());
    CHECK(ax.status().position_mm < 140.0);
}

TEST(axis_profile_move_checks_maxd_first) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim, no_caps());
    ax.identify();
    sim->clear_requests();
    MoveOptions o = mv(50.0, 0.3);
    o.decel_g = 1.5;
    CHECK(contains(what_of([&] { ax.move_to(100.0, o); }), "MAXD"));
    CHECK_THROWS(ax.check_move(100.0, o), SoftLimitError);
    o.decel_g = std::nan("");
    CHECK_THROWS(ax.move_to(100.0, o), SoftLimitError);
    CHECK(function_frames(sim->requests(), 0x17).empty());
    CHECK(function_frames(sim->requests(), 0x05).empty());
}

TEST(axis_profile_move_on_an_ec) {  // UNVERIFIED on an EC other than the gripper; modelled like the PCON
    auto sim = make_sim(SimProfile::Rod);
    Axis ax = open_sim(sim, no_caps());
    MoveOptions o = mv(200.0, 0.5);
    o.decel_g = 0.2;
    sim->clear_requests();
    const auto st = ax.move_to(100.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 100.0, 0.1);
    CHECK_EQ(sim->profile_moves(), 1);
    CHECK_EQ(function_frames(sim->requests(), 0x17).size(), 1u);
    // decel_g equal to accel_g: the direct numeric move
    o.decel_g = 0.5;
    ax.move_to(50.0, o);
    CHECK_EQ(sim->profile_moves(), 1);
}

// ============================================================================================ move_sequence (test_fast.py)

namespace {

/// Each 0x9900 write with the simulated position and motion at the moment it went out.
/// The 0x9900 block writes, each with the unit's position and motion as the frame reached it (at the unit: a frame
/// logger entry comes after its transaction).
struct WriteLog {
    struct Entry {
        Bytes frame;
        double position;
        bool moving;
    };
    std::shared_ptr<std::vector<Entry>> entries = std::make_shared<std::vector<Entry>>();
    void attach(const std::shared_ptr<SteppedClockSim>& sim) {
        auto e = entries;
        std::weak_ptr<SteppedClockSim> s = sim;
        sim->on_write = [e, s](const Bytes& f) {
            if (f.size() < 6 || f[1] != 0x10 || be16(f, 2) != reg::kDirectMoveBase) return;
            if (auto unit = s.lock()) e->push_back({f, unit->position_mm(), unit->moving()});
        };
    }
};

std::shared_ptr<SteppedClockSim> stepped_slider() {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    return SteppedClockSim::make(o, 0.01);
}

struct SequenceRig {
    // A blend sample must land in the ~0.14 s (simulated) between entering the blend band and the end of the settling
    // tail: the unit runs on its stepped clock (10 ms per request, so about fourteen polls fall in that window whatever
    // the PC's load) and the driver on a virtual one (no timeout counts the real time a busy PC takes for the polls).
    VirtualClockScope clock;
    std::shared_ptr<SteppedClockSim> sim;
    WriteLog log;
    std::unique_ptr<Axis> ax;
    SequenceRig() : sim(stepped_slider()) {
        log.attach(sim);
        ax.reset(new Axis(open_sim(sim, no_caps())));
        ax->move_to(40.0, 100.0, 0.3);
        log.entries->clear();
        sim->clear_requests();
    }
};

SequenceOptions seq(double speed = 100.0, double accel = 0.3, std::optional<double> blend = std::nullopt) {
    SequenceOptions o;
    o.speed = speed;
    o.accel_g = accel;
    o.blend_band = blend;
    return o;
}

}  // namespace

TEST(axis_sequence_without_blend_each_point_completes) {
    SequenceRig rig;
    const auto results = rig.ax->move_sequence({50.0, 60.0, 55.0}, seq());
    REQUIRE(results.size() == 3);
    const double targets[] = {50.0, 60.0, 55.0};
    for (int i = 0; i < 3; ++i) CHECK_NEAR(results[static_cast<std::size_t>(i)].position_mm, targets[i], 0.1);
    const auto& writes = *rig.log.entries;
    REQUIRE(writes.size() == 3);
    for (std::size_t i = 1; i < writes.size(); ++i) CHECK(!writes[i].moving);  // sent after the previous completed
}

TEST(axis_sequence_blend_sends_the_next_target_inside_the_band) {
    SequenceRig rig;
    const std::vector<SequencePoint> points = {60.0, SequencePoint(90.0, 80.0), 70.0};
    const auto results = rig.ax->move_sequence(points, seq(100.0, 0.3, 2.0));
    REQUIRE(results.size() == 3);
    CHECK(std::fabs(results[0].position_mm - 60.0) <= 2.0 + 1e-6);
    CHECK(std::fabs(results[1].position_mm - 90.0) <= 2.0 + 1e-6);
    CHECK_NEAR(results[2].position_mm, 70.0, 0.1);  // the last point completes by the safe rule
    CHECK(results[2].in_position);
    const auto& writes = *rig.log.entries;
    REQUIRE(writes.size() == 3);
    const double previous[] = {60.0, 90.0};
    for (std::size_t i = 1; i < 3; ++i) {
        CHECK(writes[i].moving);  // a blended point must not stop the axis
        CHECK(std::fabs(writes[i].position - previous[i - 1]) <= 2.5);
    }
    CHECK_EQ(be32(writes[1].frame, 15), 8000u);  // VCMD 80.00 mm/s for point 1
}

TEST(axis_sequence_every_point_is_checked_before_anything_is_sent) {
    SequenceRig rig;
    const std::string text = what_of([&] { rig.ax->move_sequence({50.0, 60.0, 151.0}, seq(50.0, 0.1, 1.0)); });
    CHECK(contains(text, "point 2"));
    CHECK_THROWS(rig.ax->move_sequence({50.0, 60.0, 151.0}, seq(50.0, 0.1, 1.0)), SoftLimitError);
    CHECK_THROWS(rig.ax->move_sequence({50.0, SequencePoint(60.0, 900.0)}, seq(50.0)), SoftLimitError);
    for (double band : {0.0, 20.0, std::nan("")})
        CHECK_THROWS(rig.ax->move_sequence({50.0, 60.0}, seq(50.0, 0.1, band)), SoftLimitError);
    CHECK_THROWS(rig.ax->move_sequence({}, seq()), std::invalid_argument);
    CHECK_EQ(write_frames(rig.sim->requests()), 0);
    CHECK(rig.log.entries->empty());
}
