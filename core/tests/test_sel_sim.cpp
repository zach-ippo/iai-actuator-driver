// SEL unit: the simulated SEL controller (src/sel_sim.hpp). Each worst case a SEL driver must survive reproduces here:
// silence on a frame the controller cannot accept, the stale bit 4 right after an operation command, refusals in MANU /
// during an emergency stop / while not ready, the reply buffer clip of a position read, injected faults and silence,
// and the frames-per-command counter a link's retry tests count with. The simulator's clock is a plain variable here,
// so no test sleeps. Nothing here ran against a SEL controller (UNVERIFIED). No serial port is opened.
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "sel_frame.hpp"
#include "sel_messages.hpp"
#include "sel_sim.hpp"

using namespace iai_actuator_driver;
using sel::SelSimFault;
using sel::SelSimMode;
using sel::SimulatedSel;
using sel::SimulatedSelOptions;

namespace {

constexpr int kStation = 0x99;
constexpr sel::AxisPattern kX = 0x01;
constexpr sel::AxisPattern kY = 0x02;
constexpr sel::AxisPattern kXY = 0x03;
constexpr sel::AxisPattern kAll = 0x0F;

/// A simulator on a clock that only moves when the test says so.
struct Bench {
    double t = 0.0;
    SimulatedSel sim;

    explicit Bench(SimulatedSelOptions o = {}) : sim(clocked(std::move(o), &t)) {}
    static SimulatedSelOptions clocked(SimulatedSelOptions o, double* t) {
        o.clock = [t] { return *t; };
        return o;
    }

    void run(double seconds) { t += seconds; }

    /// Every byte readable now.
    std::string drain() {
        std::string out;
        std::uint8_t buffer[4096];
        for (;;) {
            const std::size_t n = sim.read(buffer, sizeof buffer);
            if (n == 0) return out;
            out.append(reinterpret_cast<const char*>(buffer), n);
        }
    }
    std::string raw(const std::string& frame) {
        sim.write(reinterpret_cast<const std::uint8_t*>(frame.data()), frame.size());
        return drain();
    }
    std::string send(const sel::Request& r) { return raw(sel::encode_request(r, kStation)); }

    /// The data of the normal reply to `r`.
    std::string data(const sel::Request& r) {
        const sel::DecodeResult d = sel::decode_reply(send(r), kStation, r.id);
        REQUIRE(d.ok());
        REQUIRE(d.frame.kind == sel::FrameKind::reply);
        return d.frame.data;
    }
    /// The code of the error reply to `r`.
    int refusal(const sel::Request& r) {
        const sel::DecodeResult d = sel::decode_reply(send(r), kStation, r.id);
        REQUIRE(d.ok());
        REQUIRE(d.frame.kind == sel::FrameKind::error_reply);
        return d.frame.id;
    }
    void accept(const sel::Request& r) { sel::parse_no_data(r.id, data(r)); }

    sel::AxisStatus status(int axis) {
        const sel::AxisStatusReply r = sel::parse_axis_status(data(sel::axis_status_query(sel::axis_bit(axis))),
                                                              sel::axis_bit(axis));
        REQUIRE(r.axes.size() == 1);
        return r.axes[0];
    }
    sel::SystemStatus system() { return sel::parse_system_status(data(sel::system_status_query())); }

    /// Servo ON and home every axis of `axes`, then wait until done.
    void prepare(sel::AxisPattern axes = kAll) {
        accept(sel::servo(axes, true));
        run(1.0);
        accept(sel::home(axes));
        run(30.0);
        for (int axis : sel::axes_of(axes)) {
            REQUIRE(status(axis).homed());
            REQUIRE(status(axis).completed());
        }
    }
};

SimulatedSelOptions tt() {
    SimulatedSelOptions o;
    o.profile = sel::SelSimProfile::tt;
    return o;
}

sel::PositionData row4(double base) {
    sel::PositionData d;
    d.axes = kAll;
    d.profile = {0.3, 0.3, 100.0};
    d.positions = {base, base + 1.0, 5.0, 90.0};
    return d;
}

}  // namespace

TEST(sel_sim_identity_and_status_queries) {
    SimulatedSelOptions o;
    o.settings[sel::kSettingPositionOutput] = 0x08;
    Bench b(o);
    CHECK(sel::to_string(b.sim.profile()) == std::string("tta"));
    CHECK_EQ(b.sim.axis_count(), 4);
    CHECK_EQ(sel::parse_test_call(b.data(sel::test_call("0123456789")), "0123456789"), std::string("0123456789"));
    const sel::VersionInfo v = sel::parse_version(b.data(sel::version_query()), 0, 0);
    CHECK_EQ(v.model_code, 0xB9);
    CHECK_EQ(sel::parse_setting(b.data(sel::setting_query(sel::kSettingPositionOutput))), 0x08u);
    CHECK_EQ(sel::parse_setting(b.data(sel::setting_query(sel::kSettingCoordinateSystem))), 0u);
    const sel::SystemStatus s = b.system();
    CHECK(s.auto_mode());
    CHECK(s.system_ready());
    CHECK(!s.emergency_stop());
    // FFh asks for every axis: the four present ones answer
    const sel::AxisStatusReply all = sel::parse_axis_status(b.data(sel::axis_status_query(0xFF)), 0xFF);
    CHECK_EQ(static_cast<int>(all.present), 0x0F);
    REQUIRE(all.axes.size() == 4);
    CHECK_NEAR(all.axes[0].position, 25.0, 1e-9);
    CHECK(!all.axes[0].servo_on());
    CHECK(!all.axes[0].homed());
    CHECK_EQ(sel::parse_position_count(sel::kPositionCountQuery, b.data(sel::position_count_query())), 0);
    CHECK_EQ(b.sim.replies(), 7);
    CHECK_EQ(b.sim.dropped(), 0);
}

TEST(sel_sim_silence_on_frames_it_cannot_accept) {
    Bench b;
    const std::string good = sel::encode_request(sel::test_call("ABCDEFGHIJ"), kStation);
    std::string bad = good;
    bad[bad.size() - 3] = bad[bad.size() - 3] == '0' ? '1' : '0';  // the checksum
    CHECK(b.raw(bad).empty());
    CHECK(b.raw(sel::encode_request(sel::test_call("ABCDEFGHIJ"), 0x98)).empty());  // another station
    std::string no_cr = good;
    no_cr.erase(no_cr.size() - 2, 1);
    CHECK(b.raw(no_cr).empty());
    CHECK(b.raw("#99200ABCDEFGHIJ" + std::string("00\r\n")).empty());  // not a command
    CHECK_EQ(b.sim.dropped(), 4);
    CHECK_EQ(b.sim.frames(), 0);  // none passed the frame check
    CHECK_EQ(b.sim.replies(), 0);
    // "@@" turns the check off (DOC); a good frame after the bad ones is answered
    CHECK(sel::decode_reply(b.raw(sel::encode_request(sel::test_call("ABCDEFGHIJ"), kStation, false)), kStation,
                            sel::kTestCall).ok());
    CHECK(sel::decode_reply(b.raw(good), kStation, sel::kTestCall).ok());
    CHECK_EQ(b.sim.frames(sel::kTestCall), 2);
    // a frame split over several writes is one frame
    b.sim.write(reinterpret_cast<const std::uint8_t*>(good.data()), 5);
    CHECK(b.drain().empty());
    CHECK(sel::decode_reply(b.raw(good.substr(5)), kStation, sel::kTestCall).ok());
}

TEST(sel_sim_receive_buffer_overflow_is_silence) {
    Bench b(tt());
    CHECK_EQ(b.sim.max_command_bytes(), static_cast<std::size_t>(1023));
    CHECK_EQ(b.sim.max_reply_bytes(), static_cast<std::size_t>(1024));
    // a test call padded to exactly the buffer: a valid frame, so an error reply for its data
    const std::size_t data_len = 1023 - 10;
    sel::Request r;
    r.id = sel::kTestCall;
    r.data = std::string(data_len, 'A');
    CHECK_EQ(b.refusal(r), sel::kSelSimErrorFormat);
    r.data.push_back('A');  // one byte over: a receive error, no reply
    CHECK(b.send(r).empty());
    CHECK_EQ(b.sim.dropped(), 1);
    // the controller reads on normally afterwards
    CHECK(sel::decode_reply(b.send(sel::test_call("0123456789")), kStation, sel::kTestCall).ok());
}

TEST(sel_sim_stale_bit4_right_after_a_move) {
    Bench b;
    b.prepare(kX);
    CHECK(b.status(1).completed());  // the home's completion
    b.accept(sel::move_absolute(kX, {}, {100.0}));
    // the first poll right after the reply: bit 0 still OFF, bit 4 still the home's, not moved: looks "done"
    sel::AxisStatus s = b.status(1);
    CHECK(!s.in_use());
    CHECK(s.completed());
    CHECK_NEAR(s.position, 0.0, 1e-9);
    b.run(0.06);  // past in_use_delay_s
    s = b.status(1);
    CHECK(s.in_use());
    CHECK(!s.completed());
    b.run(0.5);
    s = b.status(1);
    CHECK(s.in_use());
    CHECK(s.position > 1.0 && s.position < 99.0);
    b.run(1.0);
    s = b.status(1);
    CHECK(!s.in_use());
    CHECK(s.completed());
    CHECK_NEAR(s.position, 100.0, 1e-9);
    CHECK_EQ(b.sim.frames(sel::kMoveAbsolute), 1);
    CHECK_EQ(b.sim.executed(sel::kMoveAbsolute), 1);
}

TEST(sel_sim_without_the_delay_bit0_is_on_at_once) {
    SimulatedSelOptions o;
    o.in_use_delay_s = 0.0;
    Bench b(o);
    b.prepare(kX);
    b.accept(sel::move_absolute(kX, {}, {50.0}));
    const sel::AxisStatus s = b.status(1);
    CHECK(s.in_use());
    CHECK(!s.completed());
}

TEST(sel_sim_refusals_manual_emergency_stop_not_ready) {
    Bench b;
    b.prepare(kXY);
    // MANU: every execute but stop is refused; queries are answered
    b.sim.set_mode(SelSimMode::manual);
    CHECK_EQ(b.refusal(sel::servo(kX, false)), sel::kSelSimErrorManualMode);
    CHECK_EQ(b.refusal(sel::move_absolute(kX, {}, {10.0})), sel::kSelSimErrorManualMode);
    CHECK_EQ(b.refusal(sel::alarm_reset()), sel::kSelSimErrorManualMode);
    CHECK_EQ(b.system().mode, 2);
    CHECK(b.system().mode_switch_manual());
    b.accept(sel::stop(kX));
    b.sim.set_mode(SelSimMode::automatic);
    // not ready
    b.sim.set_ready(false);
    CHECK(!b.system().system_ready());
    CHECK_EQ(b.refusal(sel::move_absolute(kX, {}, {10.0})), sel::kSelSimErrorNotReady);
    b.sim.set_ready(true);
    // emergency stop: motion refused, servo OFF accepted
    b.sim.set_emergency_stop(true);
    CHECK(b.system().emergency_stop());
    CHECK(!b.status(1).servo_on());  // the stop switched the servos OFF
    CHECK_EQ(b.refusal(sel::servo(kX, true)), sel::kSelSimErrorEmergencyStop);
    CHECK_EQ(b.refusal(sel::home(kX)), sel::kSelSimErrorEmergencyStop);
    b.accept(sel::servo(kX, false));
    b.sim.set_emergency_stop(false);
    CHECK(!b.status(1).servo_on());  // released: still OFF
    CHECK_EQ(b.sim.executed(sel::kMoveAbsolute), 0);
}

TEST(sel_sim_emergency_stop_and_mode_switch_cancel_motion) {
    Bench b;
    b.prepare(kXY);
    b.accept(sel::move_absolute(kX, {}, {150.0}));
    b.run(0.5);
    CHECK(b.status(1).in_use());
    b.sim.set_emergency_stop(true);
    sel::AxisStatus s = b.status(1);
    CHECK(!s.in_use());
    CHECK(!s.completed());  // bit 0 OFF, bit 4 OFF: cancelled (DOC)
    CHECK(s.position > 1.0 && s.position < 149.0);
    b.sim.set_emergency_stop(false);
    b.accept(sel::servo(kXY, true));
    b.run(1.0);
    b.accept(sel::move_absolute(kY, {}, {150.0}));
    b.run(0.5);
    b.sim.set_mode(SelSimMode::manual);
    s = b.status(2);
    CHECK(!s.in_use());
    CHECK(!s.completed());
    CHECK(s.servo_on());  // a mode switch keeps the servo
}

TEST(sel_sim_motion_refusals_move_nothing) {
    Bench b;
    CHECK_EQ(b.refusal(sel::home(kX)), sel::kSelSimErrorServoOff);
    b.accept(sel::servo(kX, true));
    CHECK_EQ(b.refusal(sel::home(kX)), sel::kSelSimErrorAxisInUse);  // the servo command still owns the axis
    b.run(1.0);
    CHECK_EQ(b.refusal(sel::move_absolute(kX, {}, {10.0})), sel::kSelSimErrorNotHomed);
    b.accept(sel::home(kX));
    b.run(30.0);
    CHECK_EQ(b.refusal(sel::move_absolute(kX, {}, {200.5})), sel::kSelSimErrorOutOfStroke);
    CHECK_EQ(b.refusal(sel::move_absolute(0x10, {}, {10.0})), sel::kSelSimErrorNoAxis);
    CHECK_EQ(b.refusal(sel::move_absolute(kXY, {}, {10.0, 10.0})), sel::kSelSimErrorServoOff);  // Y is OFF
    b.accept(sel::move_absolute(kX, {}, {20.0}));
    CHECK_EQ(b.refusal(sel::move_absolute(kX, {}, {30.0})), sel::kSelSimErrorAxisInUse);  // inside the delay too
    CHECK_EQ(b.sim.executed(sel::kMoveAbsolute), 1);
    sel::Request bad = sel::move_absolute(kX, {}, {10.0});
    bad.data.pop_back();
    CHECK_EQ(b.refusal(bad), sel::kSelSimErrorFormat);
    CHECK_EQ(b.refusal(sel::program_control(sel::ProgramAction::run, 1)), sel::kSelSimErrorUnsupported);
}

TEST(sel_sim_reply_buffer_clips_position_reads) {
    for (const bool older : {false, true}) {
        Bench b(older ? tt() : SimulatedSelOptions{});
        for (int n = 1; n <= 100; ++n) b.sim.set_position_data(n, row4(n));
        CHECK_EQ(sel::parse_position_count(sel::kPositionCountQuery, b.data(sel::position_count_query())), 100);
        const std::string frame = b.send(sel::position_query(1, 100));
        CHECK(frame.size() <= b.sim.max_reply_bytes());
        const sel::DecodeResult d = sel::decode_reply(frame, kStation, sel::kPositionQuery);
        REQUIRE(d.ok());
        const std::vector<sel::PositionRecord> rows = sel::parse_positions(sel::kPositionQuery, d.frame.data, 1, 100);
        // 13 fixed bytes + 49 per 4-axis record: 41 fit in 2048, 20 in 1024
        CHECK_EQ(static_cast<int>(rows.size()), older ? 20 : 41);
        CHECK_EQ(rows.front().number, 1);
        CHECK_NEAR(rows.back().data.positions[0], static_cast<double>(rows.size()), 1e-9);
        CHECK_NEAR(rows.front().data.positions[3], 90.0, 1e-9);
        // the next chunk continues where the clip ended
        const std::vector<sel::PositionRecord> next = sel::parse_positions(
            sel::kPositionQuery, b.data(sel::position_query(rows.back().number + 1, 5)), rows.back().number + 1, 5);
        REQUIRE(next.size() == 5);
        CHECK_EQ(next.front().number, rows.back().number + 1);
        if (older) {
            CHECK_EQ(b.refusal(sel::position_query_2(1, 10)), sel::kSelSimErrorUnsupported);
        } else {
            const std::vector<sel::PositionRecord> four = sel::parse_positions(
                sel::kPositionQuery2, b.data(sel::position_query_2(1, 100)), 1, 100);
            CHECK_EQ(static_cast<int>(four.size()), 40);  // 14 fixed + 50 per record
        }
    }
}

TEST(sel_sim_injected_faults_and_the_frame_counter) {
    Bench b;
    // lost on the way in: not acted on, no reply
    b.sim.inject_fault(SelSimFault::silence);
    CHECK(b.send(sel::servo(kX, true)).empty());
    b.run(1.0);
    CHECK(!b.status(1).servo_on());
    CHECK_EQ(b.sim.frames(sel::kServo), 1);
    CHECK_EQ(b.sim.executed(sel::kServo), 0);
    // acted on, the reply lost: a blind re-send would be a second command
    b.sim.inject_fault(SelSimFault::silent_after_acting);
    CHECK(b.send(sel::servo(kX, true)).empty());
    b.run(1.0);
    CHECK(b.status(1).servo_on());
    CHECK_EQ(b.sim.frames(sel::kServo), 2);
    CHECK_EQ(b.sim.executed(sel::kServo), 1);
    // a damaged reply is dropped by the host's check
    const sel::Request q = sel::test_call("0123456789");
    b.sim.inject_fault(SelSimFault::bad_checksum);
    CHECK(sel::decode_reply(b.send(q), kStation, q.id).status == sel::FrameStatus::bad_checksum);
    b.sim.inject_fault(SelSimFault::wrong_station);
    CHECK(sel::decode_reply(b.send(q), kStation, q.id).status == sel::FrameStatus::wrong_station);
    b.sim.inject_fault(SelSimFault::wrong_id);
    CHECK(sel::decode_reply(b.send(q), kStation, q.id).status == sel::FrameStatus::wrong_id);
    b.sim.inject_fault(SelSimFault::partial);
    CHECK(!sel::decode_reply(b.send(q), kStation, q.id).ok());
    b.sim.inject_fault(SelSimFault::error_reply);
    CHECK_EQ(b.refusal(q), sel::kSelSimErrorInjected);
    CHECK(sel::decode_reply(b.send(q), kStation, q.id).ok());  // faults are used up
    // total silence until released; the frames still count
    b.sim.set_silent(true);
    for (int i = 0; i < 3; ++i) CHECK(b.send(q).empty());
    b.sim.set_silent(false);
    CHECK_EQ(b.sim.frames(sel::kTestCall), 9);
    // a channel not enabled for execution: executes get silence and are not acted on, queries are answered
    b.sim.set_execute_enabled(false);
    CHECK(b.send(sel::servo(kX, false)).empty());
    b.run(1.0);
    CHECK(b.status(1).servo_on());
    b.sim.reset_counters();
    CHECK_EQ(b.sim.frames(), 0);
    CHECK_EQ(b.sim.executed(sel::kServo), 0);
}

TEST(sel_sim_stop_decelerates_and_cancels) {
    Bench b;
    b.prepare(kX);
    b.accept(sel::move_absolute(kX, {0.3, 0.3, 100.0}, {150.0}));
    b.run(0.06 + 0.5);
    const double before = b.status(1).position;
    b.accept(sel::stop(kX));
    sel::AxisStatus s = b.status(1);
    CHECK(s.in_use());  // still decelerating
    b.run(1.0);
    s = b.status(1);
    CHECK(!s.in_use());
    CHECK(!s.completed());  // cancelled
    CHECK(s.position > before && s.position < before + 5.0);
    // a stop inside the delay drops the command: bit 4 OFF, nothing moved
    b.accept(sel::move_absolute(kX, {}, {10.0}));
    const double at = b.status(1).position;
    b.accept(sel::stop(kX));
    b.run(1.0);
    s = b.status(1);
    CHECK(!s.in_use());
    CHECK(!s.completed());
    CHECK_NEAR(s.position, at, 1e-9);
}

TEST(sel_sim_speed_change_and_clamps) {
    Bench b;
    b.prepare(kX);
    b.accept(sel::move_absolute(kX, {0.3, 0.3, 50.0}, {150.0}));  // 3 s at 50 mm/s
    b.run(0.06 + 0.5);
    b.accept(sel::change_speed(kX, 300.0));
    b.run(1.0);
    sel::AxisStatus s = b.status(1);
    CHECK(!s.in_use());
    CHECK(s.completed());
    CHECK_NEAR(s.position, 150.0, 1e-9);
    // 5000 mm/s is clamped to the axis' maximum (50 mm/s here): 150 mm take 3 s
    SimulatedSelOptions o;
    o.axes = std::vector<sel::SelSimAxis>{{0.0, 200.0, 50.0, 0.0, false}};
    Bench c(o);
    c.prepare(kX);
    c.accept(sel::move_absolute(kX, {1.0, 1.0, 5000.0}, {150.0}));
    c.run(0.06 + 2.5);
    CHECK(c.status(1).in_use());
    c.run(1.0);
    CHECK(c.status(1).completed());
    CHECK_NEAR(c.status(1).position, 150.0, 1e-9);
}

TEST(sel_sim_relative_jog_and_stored_positions) {
    Bench b;
    b.prepare(kXY);
    b.accept(sel::move_relative(kXY, {}, {10.0, 20.0}));
    b.run(5.0);
    CHECK_NEAR(b.status(1).position, 10.0, 1e-9);
    CHECK_NEAR(b.status(2).position, 20.0, 1e-9);
    CHECK_EQ(b.refusal(sel::move_relative(kX, {}, {-10.5})), sel::kSelSimErrorOutOfStroke);
    // inching beyond 1000 mm is taken as 1000 (DOC): out of the stroke here
    CHECK_EQ(b.refusal(sel::jog(kX, {}, 1500.0, true)), sel::kSelSimErrorOutOfStroke);
    b.accept(sel::jog(kX, {}, 5.0, true));
    b.run(5.0);
    CHECK_NEAR(b.status(1).position, 15.0, 1e-9);
    // a jog runs to the soft limit unless stopped
    b.accept(sel::jog(kX, {}, 0.0, true));
    b.run(10.0);
    CHECK_NEAR(b.status(1).position, 200.0, 1e-9);
    CHECK(b.status(1).completed());
    // 237H: the pattern is ANDed with the stored one's; 0 takes the row's profile
    sel::PositionData row;
    row.axes = kX;
    row.profile = {0.0, 0.0, 200.0};
    row.positions = {42.0};
    b.sim.set_position_data(7, row);
    b.accept(sel::move_to_position(kXY, {}, 7));
    b.run(5.0);
    CHECK_NEAR(b.status(1).position, 42.0, 1e-9);
    CHECK_NEAR(b.status(2).position, 20.0, 1e-9);
    CHECK_EQ(b.refusal(sel::move_to_position(kY, {}, 7)), sel::kSelSimErrorNoAxis);
    CHECK_EQ(b.refusal(sel::move_to_position_2(kX, {}, 8)), sel::kSelSimErrorNoPosition);
}

TEST(sel_sim_axis_error_until_alarm_reset) {
    Bench b;
    b.prepare(kX);
    b.sim.set_axis_error(1, 0x0C0);
    sel::AxisStatus s = b.status(1);
    CHECK_EQ(s.error_code, 0x0C0);
    CHECK(!s.servo_on());
    CHECK_EQ(b.system().latest_error, 0x0C0);
    const sel::ErrorDetail e = sel::parse_error_detail(b.data(sel::error_detail_query(1, 1)));
    CHECK_EQ(e.error_number, 0x0C0);
    CHECK_EQ(e.detail[2], 1);
    CHECK_EQ(b.refusal(sel::servo(kX, true)), sel::kSelSimErrorAxisError);
    b.accept(sel::alarm_reset());
    CHECK_EQ(b.status(1).error_code, 0);
    CHECK_EQ(b.system().latest_error, 0);
    b.accept(sel::servo(kX, true));
}

TEST(sel_sim_software_reset_and_power_cycle) {
    Bench b;
    b.prepare(kX);
    CHECK(b.send(sel::software_reset()).empty());  // no reply (DOC)
    CHECK_EQ(b.sim.executed(sel::kSoftwareReset), 1);
    CHECK(b.send(sel::test_call("0123456789")).empty());  // re-initialising
    b.run(1.1);
    const sel::AxisStatus s = b.status(1);
    CHECK(!s.servo_on());
    CHECK(!s.homed());
    // an absolute machine keeps its homes over a power cycle
    SimulatedSelOptions o;
    o.absolute = true;
    Bench a(o);
    a.prepare(kX);
    a.sim.power_cycle();
    CHECK(a.send(sel::system_status_query()).empty());
    a.run(1.1);
    CHECK(a.status(1).homed());
    CHECK(!a.status(1).servo_on());
}

TEST(sel_sim_reply_latency_and_a_late_reply) {
    SimulatedSelOptions o;
    o.reply_latency_s = 0.1;
    Bench b(o);
    CHECK(b.send(sel::test_call("0123456789")).empty());
    b.sim.reset_input();  // the reply is not in yet: the flush cannot drop it
    b.run(0.1);
    CHECK(sel::decode_reply(b.drain(), kStation, sel::kTestCall).ok());
    CHECK(!b.sim.has_real_timeout());
}

TEST(sel_sim_unplug_and_reopen) {
    Bench b;
    b.sim.unplug();
    CHECK(b.sim.present() == std::optional<bool>(false));
    CHECK(!b.sim.is_open());
    CHECK_THROWS(b.send(sel::test_call("0123456789")), TransportError);
    CHECK_THROWS(b.sim.reopen(), TransportError);
    b.sim.replug();
    CHECK_THROWS(b.send(sel::test_call("0123456789")), TransportError);  // the old handle stays dead
    b.sim.reopen();
    CHECK(sel::decode_reply(b.send(sel::test_call("0123456789")), kStation, sel::kTestCall).ok());
    CHECK_THROWS(b.sim.set_baud_rate(9600), std::logic_error);
}
