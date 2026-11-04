// SEL unit: the SEL link (src/sel_link.hpp) over the simulated controller (src/sel_sim.hpp), FakeTransport and the
// StreamShim (src/sel_shim.hpp). Every retry rule is proved by counting command frames: a query is sent again on
// silence (3 more times, then a clear error), stop and alarm reset are retried hard, and an execute command that meets
// silence is re-queried, never re-sent - exactly one frame per command - unless retry_execute is set. A stale bit 4
// never reports done early. A virtual clock moves only when the link sleeps, so a 3 s timeout costs no real time.
// Nothing here ran against a SEL controller (UNVERIFIED). No serial port is opened.
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"
#include "sel_frame.hpp"
#include "sel_link.hpp"
#include "sel_messages.hpp"
#include "sel_shim.hpp"
#include "sel_sim.hpp"

using namespace iai_actuator_driver;
using sel::Completion;
using sel::SelLink;
using sel::SelLinkOptions;
using sel::SelSimFault;
using sel::SimulatedSel;
using sel::SimulatedSelOptions;

namespace {

constexpr sel::AxisPattern kX = 0x01;
constexpr sel::AxisPattern kXY = 0x03;

SimulatedSelOptions ready() {
    SimulatedSelOptions o;
    o.servo_on = true;
    o.homed = true;
    return o;
}

/// A simulated controller on the virtual clock, behind a link (optionally through a StreamShim).
struct Bench {
    VirtualClockScope clock;  // first: the simulator reads now_s() from its construction on
    std::shared_ptr<SimulatedSel> sim;
    std::shared_ptr<Transport> wire;
    SelLink link;

    explicit Bench(SimulatedSelOptions o = ready(), SelLinkOptions l = {}, std::vector<std::size_t> read_segments = {},
                   std::vector<std::size_t> write_segments = {}, bool shim = false)
        : sim(std::make_shared<SimulatedSel>(o)),
          wire(shim ? std::shared_ptr<Transport>(std::make_shared<sel::StreamShim>(sim, std::move(read_segments),
                                                                                   std::move(write_segments)))
                    : std::shared_ptr<Transport>(sim)),
          link(wire, std::move(l)) {}

    double now() { return clock.clock().now(); }
    void run(double seconds) { clock.clock().advance(seconds); }
    sel::AxisStatus status(int axis) {
        const sel::AxisStatusReply r =
            sel::parse_axis_status(link.query(sel::axis_status_query(sel::axis_bit(axis))), sel::axis_bit(axis));
        REQUIRE(r.axes.size() == 1);
        return r.axes[0];
    }
};

std::vector<std::uint8_t> bytes(const std::string& text) { return {text.begin(), text.end()}; }

/// A FakeTransport controller: `answer` gets each decoded command, returns the bytes to send back (nullopt: silence).
std::shared_ptr<FakeTransport> fake(std::function<std::optional<std::string>(const sel::Frame&)> answer) {
    return std::make_shared<FakeTransport>([answer](const std::vector<std::uint8_t>& written)
                                               -> std::optional<std::vector<std::uint8_t>> {
        const sel::DecodeResult d = sel::decode(std::string(written.begin(), written.end()));
        if (!d.ok()) return std::nullopt;
        const std::optional<std::string> reply = answer(d.frame);
        if (!reply) return std::nullopt;
        return bytes(*reply);
    });
}

int writes_of(const FakeTransport& t, int id) {
    int n = 0;
    for (const auto& w : t.writes()) {
        const sel::DecodeResult d = sel::decode(std::string(w.begin(), w.end()));
        if (d.ok() && d.frame.id == id) ++n;
    }
    return n;
}

}  // namespace

TEST(sel_link_silence_rules_by_message) {
    CHECK(sel::silence_rule(sel::axis_status_query(kX)) == sel::SilenceRule::resend);
    CHECK(sel::silence_rule(sel::stop(kX)) == sel::SilenceRule::resend);
    CHECK(sel::silence_rule(sel::alarm_reset()) == sel::SilenceRule::resend);
    CHECK(sel::silence_rule(sel::servo(kX, true)) == sel::SilenceRule::verify);
    CHECK(sel::silence_rule(sel::move_absolute(kX, {}, {10.0})) == sel::SilenceRule::verify);
    CHECK(sel::silence_rule(sel::move_relative(kX, {}, {10.0})) == sel::SilenceRule::verify);
    CHECK(sel::silence_rule(sel::program_control(sel::ProgramAction::run, 1)) == sel::SilenceRule::verify);
    CHECK(sel::silence_rule(sel::set_output(301, true)) == sel::SilenceRule::verify_then_resend);
    CHECK(sel::silence_rule(sel::position_clear(1, 1)) == sel::SilenceRule::verify_then_resend);
    CHECK(sel::silence_rule(sel::software_reset()) == sel::SilenceRule::no_reply);
}

TEST(sel_link_query_round_trip_and_counters) {
    Bench b;
    CHECK_EQ(sel::parse_test_call(b.link.query(sel::test_call("0123456789")), "0123456789"), std::string("0123456789"));
    CHECK(b.status(1).servo_on());
    CHECK_EQ(b.link.frames_sent(), 2);
    CHECK_EQ(b.link.frames_sent(sel::kTestCall), 1);
    CHECK_EQ(b.link.silences(), 0);
    CHECK_THROWS(b.link.query(sel::servo(kX, true)), std::invalid_argument);  // nothing sent
    CHECK_EQ(b.sim->frames(sel::kServo), 0);
    // the frame sink sees each frame both ways
    std::vector<std::string> seen;
    SelLinkOptions l;
    l.frame_sink = [&seen](bool sent, const std::string& line) { seen.push_back((sent ? ">" : "<") + line); };
    SelLink logged(b.sim, l);
    logged.query(sel::system_status_query());
    REQUIRE(seen.size() == 2);
    CHECK(seen[0].rfind(">!99215", 0) == 0);
    CHECK(seen[1].rfind("<#99215", 0) == 0);
}

TEST(sel_link_query_retries_three_times_then_a_clear_error) {
    VirtualClockScope clock;
    auto t = fake([](const sel::Frame&) { return std::nullopt; });
    SelLink link(t);
    const double start = clock.clock().now();
    try {
        link.query(sel::system_status_query());
        CHECK(false);
    } catch (const sel::SelSilentError& e) {
        CHECK_EQ(e.frames(), 4);
        CHECK_EQ(e.message_id(), sel::kSystemStatusQuery);
        CHECK(std::string(e.what()).find("215H after 4 frames") != std::string::npos);
        CHECK(std::string(e.what()).find("station number") != std::string::npos);
    }
    CHECK_EQ(t->writes().size(), 4u);
    CHECK_EQ(link.silences(), 4);
    CHECK(clock.clock().now() - start >= 12.0);  // 3 s each
    // a controller that answers on the third frame
    int calls = 0;
    auto late = fake([&calls](const sel::Frame& f) -> std::optional<std::string> {
        if (++calls < 3) return std::nullopt;
        return sel::encode_reply(f.station, f.id, "0123456789");
    });
    SelLink again(late);
    CHECK_EQ(again.query(sel::test_call("0123456789")), std::string("0123456789"));
    CHECK_EQ(late->writes().size(), 3u);
}

TEST(sel_link_drops_damaged_and_late_replies) {
    Bench b;
    const sel::Request q = sel::test_call("0123456789");
    for (SelSimFault f : {SelSimFault::bad_checksum, SelSimFault::wrong_station, SelSimFault::wrong_id,
                          SelSimFault::partial}) {
        b.sim->inject_fault(f);
        b.link.reset_counters();
        CHECK_EQ(b.link.send(q).frames, 2);  // the damaged reply is dropped: silence, then the retry is answered
        CHECK_EQ(b.link.silences(), 1);
    }
    // a late reply to an earlier command arriving with the right one, coalesced in one segment: skipped, not a retry
    auto t = fake([](const sel::Frame& f) -> std::optional<std::string> {
        return sel::encode_reply(f.station, sel::kTestCall, "0123456789") + sel::encode_reply(f.station, f.id, "1");
    });
    SelLink link(t);
    const sel::SendOutcome out = link.send(sel::setting_query(185));
    REQUIRE(out.reply.has_value());
    CHECK_EQ(out.frames, 1);
    CHECK_EQ(link.lines_dropped(), 1);
}

TEST(sel_link_error_reply_is_never_retried) {
    Bench b;
    b.sim->inject_fault(SelSimFault::error_reply);
    try {
        b.link.query(sel::system_status_query());
        CHECK(false);
    } catch (const sel::SelRejectedError& e) {
        CHECK_EQ(e.code(), sel::kSelSimErrorInjected);
        CHECK_EQ(e.message_id(), sel::kSystemStatusQuery);
    }
    CHECK_EQ(b.sim->frames(sel::kSystemStatusQuery), 1);
    b.sim->set_mode(sel::SelSimMode::manual);
    CHECK_THROWS(b.link.send(sel::move_absolute(kX, {}, {10.0}), sel::verify_motion(kX, {10.0})),
                 sel::SelRejectedError);
    CHECK_EQ(b.sim->frames(sel::kMoveAbsolute), 1);
    CHECK_EQ(b.sim->executed(sel::kMoveAbsolute), 0);
}

TEST(sel_link_execute_silence_requeries_never_resends) {
    Bench b(SimulatedSelOptions{});  // servo OFF
    // lost on the way in: the re-query shows nothing happened; still not re-sent (it cannot tell a lost command from a
    // lost reply)
    b.sim->inject_fault(SelSimFault::silence);
    try {
        b.link.send(sel::servo(kX, true), sel::verify_servo(kX, true));
        CHECK(false);
    } catch (const sel::SelSilentError& e) {
        CHECK_EQ(e.frames(), 1);
        CHECK(std::string(e.what()).find("not sent again") != std::string::npos);
    }
    CHECK_EQ(b.sim->frames(sel::kServo), 1);
    CHECK_EQ(b.sim->executed(sel::kServo), 0);
    CHECK(!b.status(1).servo_on());
    // acted on, the reply lost: the re-query confirms it
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    const sel::SendOutcome out = b.link.send(sel::servo(kX, true), sel::verify_servo(kX, true));
    CHECK(out.confirmed_by_requery());
    CHECK_EQ(out.frames, 1);
    CHECK_EQ(out.requeries, 1);
    CHECK_EQ(b.sim->frames(sel::kServo), 2);
    CHECK_EQ(b.sim->executed(sel::kServo), 1);
    CHECK(b.status(1).servo_on());
    // a channel not enabled for execution: silence, nothing done, one frame
    b.sim->set_execute_enabled(false);
    CHECK_THROWS(b.link.send(sel::servo(kX, false), sel::verify_servo(kX, false)), sel::SelSilentError);
    CHECK_EQ(b.sim->frames(sel::kServo), 3);
    CHECK(b.status(1).servo_on());
}

TEST(sel_link_execute_without_verifier_is_one_frame) {
    VirtualClockScope clock;
    auto t = fake([](const sel::Frame&) { return std::nullopt; });
    SelLink link(t);
    for (const sel::Request& r : {sel::move_absolute(kX, {}, {10.0}), sel::move_relative(kX, {}, {5.0}),
                                  sel::jog(kX, {}, 1.0, true), sel::program_control(sel::ProgramAction::run, 1),
                                  sel::change_speed(kX, 50.0)}) {
        CHECK_THROWS(link.send(r), sel::SelSilentError);
        CHECK_EQ(writes_of(*t, r.id), 1);
    }
    CHECK_EQ(t->writes().size(), 5u);
}

TEST(sel_link_move_silence_confirmed_by_requery) {
    Bench b;
    // a slow move still running when the re-query comes (after the 3 s timeout): in use = took effect
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    sel::SendOutcome out =
        b.link.send(sel::move_absolute(kX, {0.3, 0.3, 10.0}, {100.0}), sel::verify_motion(kX, {100.0}));
    CHECK(out.confirmed_by_requery());
    CHECK_EQ(b.sim->frames(sel::kMoveAbsolute), 1);
    CHECK_EQ(wait_done(b.link, kX, {100.0}), Completion::done);
    CHECK_NEAR(b.status(1).position, 100.0, 1e-6);
    // a short move already finished by then: at its target = took effect
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    out = b.link.send(sel::move_absolute(kX, {}, {90.0}), sel::verify_motion(kX, {90.0}));
    CHECK(out.confirmed_by_requery());
    CHECK_EQ(b.sim->frames(sel::kMoveAbsolute), 2);
    CHECK_EQ(b.sim->executed(sel::kMoveAbsolute), 2);
}

TEST(sel_link_stale_bit4_requery_is_inconclusive) {
    SimulatedSelOptions o = ready();
    o.in_use_delay_s = 10.0;  // bit 0 turns ON only after the link's 3 s timeout
    Bench b(o);
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    CHECK_THROWS(b.link.send(sel::move_absolute(kX, {}, {50.0}), sel::verify_motion(kX, {50.0})),
                 sel::SelSilentError);
    CHECK_EQ(b.sim->frames(sel::kMoveAbsolute), 1);
    CHECK_EQ(b.sim->executed(sel::kMoveAbsolute), 1);  // it was acted on: a re-send would have been a second move
    b.run(20.0);
    CHECK_NEAR(b.status(1).position, 50.0, 1e-6);
}

TEST(sel_link_retry_execute_resends_on_request) {
    SelLinkOptions l;
    l.retry_execute = true;
    Bench b(ready(), l);
    b.sim->inject_fault(SelSimFault::silence);
    const sel::SendOutcome out = b.link.send(sel::move_absolute(kX, {}, {20.0}), sel::verify_motion(kX, {20.0}));
    CHECK(out.reply.has_value());
    CHECK_EQ(out.frames, 2);
    CHECK_EQ(out.requeries, 1);
    CHECK_EQ(b.sim->executed(sel::kMoveAbsolute), 1);
    CHECK_EQ(wait_done(b.link, kX, {20.0}), Completion::done);
    // confirmed by the re-query: not re-sent even with retry_execute
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    CHECK_EQ(b.link.send(sel::move_absolute(kX, {}, {30.0}), sel::verify_motion(kX, {30.0})).frames, 1);
    CHECK_EQ(b.sim->executed(sel::kMoveAbsolute), 2);
}

TEST(sel_link_stop_and_alarm_reset_are_retried) {
    Bench b;
    b.sim->inject_fault(SelSimFault::silence, 2);
    CHECK_EQ(b.link.send(sel::stop(kX)).frames, 3);
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    CHECK_EQ(b.link.send(sel::alarm_reset()).frames, 2);
    b.sim->set_silent(true);
    try {
        b.link.send(sel::stop(kX));
        CHECK(false);
    } catch (const sel::SelSilentError& e) {
        CHECK_EQ(e.frames(), 6);  // 1 + hard_retries
    }
    CHECK_EQ(b.sim->frames(sel::kStop), 9);
}

TEST(sel_link_data_write_resends_only_when_not_written) {
    VirtualClockScope clock;
    int silent = 1;
    auto t = fake([&silent](const sel::Frame& f) -> std::optional<std::string> {
        if (silent-- > 0) return std::nullopt;
        return sel::encode_reply(f.station, f.id, "");
    });
    SelLink link(t);
    const sel::Request w = sel::set_output(301, true);
    // the re-read shows the old data: sent again
    sel::SendOutcome out = link.send(w, [](SelLink&) { return sel::Verdict::not_taken; });
    CHECK(out.reply.has_value());
    CHECK_EQ(out.frames, 2);
    // the re-read shows it written: success on one frame
    silent = 1;
    out = link.send(w, [](SelLink&) { return sel::Verdict::took_effect; });
    CHECK(out.confirmed_by_requery());
    CHECK_EQ(out.frames, 1);
    // cannot tell (the re-read itself failed): not sent again
    silent = 1;
    CHECK_THROWS(link.send(w, [](SelLink&) -> sel::Verdict { throw sel::SelSilentError("re-read lost", 0x20C, 4); }),
                 sel::SelSilentError);
    CHECK_EQ(writes_of(*t, sel::kSetOutput), 4);
    // a move never re-sends on not_taken without retry_execute
    silent = 1;
    CHECK_THROWS(link.send(sel::move_absolute(kX, {}, {1.0}), [](SelLink&) { return sel::Verdict::not_taken; }),
                 sel::SelSilentError);
    CHECK_EQ(writes_of(*t, sel::kMoveAbsolute), 1);
}

TEST(sel_link_program_run_confirmed_by_program_status) {
    VirtualClockScope clock;
    auto t = fake([](const sel::Frame& f) -> std::optional<std::string> {
        if (f.id == sel::kProgramRun) return std::nullopt;  // the reply is lost
        if (f.id == sel::kProgramStatusQuery) return sel::encode_reply(f.station, f.id, "0310005000" "0000");
        return std::nullopt;
    });
    SelLink link(t);
    const sel::SendOutcome out =
        link.send(sel::program_control(sel::ProgramAction::run, 3), sel::verify_program_started(3));
    CHECK(out.confirmed_by_requery());
    CHECK_EQ(writes_of(*t, sel::kProgramRun), 1);
    CHECK_EQ(writes_of(*t, sel::kProgramStatusQuery), 1);
}

TEST(sel_link_home_requery) {
    SimulatedSelOptions o;
    o.servo_on = true;
    o.home_speed = 5.0;
    Bench b(o);
    b.sim->set_axis(1, std::nullopt, std::nullopt, 50.0);
    b.sim->inject_fault(SelSimFault::silent_after_acting);
    CHECK(b.link.send(sel::home(kX), sel::verify_home(kX)).confirmed_by_requery());  // still homing at 3 s
    CHECK_EQ(b.sim->frames(sel::kHome), 1);
    CHECK_EQ(wait_done(b.link, kX, {}, 60.0), Completion::done);
    // homed already before: a finished home cannot be told from no home
    b.sim->inject_fault(SelSimFault::silence);
    CHECK_THROWS(b.link.send(sel::home(kX), sel::verify_home(kX, kX)), sel::SelSilentError);
    CHECK_EQ(b.sim->frames(sel::kHome), 2);
}

TEST(sel_link_software_reset_waits_for_the_controller) {
    SimulatedSelOptions o = ready();
    o.reset_s = 4.0;
    Bench b(o);
    b.link.software_reset();
    CHECK_EQ(b.link.frames_sent(sel::kSoftwareReset), 1);
    CHECK(b.link.frames_sent(sel::kTestCall) >= 2);  // the first ones meet the re-initialisation
    CHECK(!b.status(1).servo_on());
    // a controller that never comes back
    b.sim->set_silent(true);
    CHECK_THROWS(b.link.software_reset(5.0), sel::SelSilentError);
}

TEST(sel_link_stream_shim_splits_and_coalesces) {
    Bench b(ready(), {}, {1, 0, 3, 0, 7, 64}, {1, 2, 5}, true);
    CHECK_EQ(b.link.query(sel::test_call("ABCDEFGHIJ")), std::string("ABCDEFGHIJ"));
    CHECK(b.status(1).homed());
    b.link.send(sel::move_absolute(kXY, {}, {10.0, 20.0}));
    CHECK_EQ(wait_done(b.link, kXY, {10.0, 20.0}), Completion::done);
    CHECK_NEAR(b.status(2).position, 20.0, 1e-6);
    CHECK_EQ(b.link.silences(), 0);
    const auto& shim = static_cast<const sel::StreamShim&>(*b.wire);
    CHECK(shim.reads() > b.link.frames_sent());
    CHECK(shim.write_pieces() > b.link.frames_sent());
    // a reply naming another id, whole in one segment: dropped, then the retry is answered
    Bench c(ready(), {}, {4096}, {}, true);
    c.sim->inject_fault(SelSimFault::wrong_id);
    CHECK_EQ(c.link.send(sel::test_call("0123456789")).frames, 2);
}

TEST(sel_link_stale_bit4_never_reports_done_early) {
    SimulatedSelOptions o = ready();
    o.in_use_delay_s = 0.5;  // 25 polls at 20 ms see the previous command's bit 4
    for (bool with_target : {false, true}) {
        Bench b(o);
        b.link.send(sel::move_absolute(kX, {}, {10.0}));
        b.run(1.0);
        REQUIRE(wait_done(b.link, kX, {10.0}) == Completion::done);
        const double before = b.now();
        b.link.send(sel::move_absolute(kX, {}, {60.0}));
        const sel::AxisStatus stale = b.status(1);
        CHECK(!stale.in_use());
        CHECK(stale.completed());  // looks done
        const Completion c = with_target ? wait_done(b.link, kX, {60.0}) : wait_done(b.link, kX);
        CHECK_EQ(c, Completion::done);
        CHECK_NEAR(b.sim->axis(1).position, 60.0, 1e-6);  // only once there
        CHECK(b.now() - before > 0.5);
    }
    // the tracker alone: stale samples stay pending, with or without a target
    sel::AxisStatusReply r;
    r.present = kX;
    sel::AxisStatus s;
    s.axis = 1;
    s.status = 0x18;  // servo ON, completed, not in use
    s.position = 10.0;
    r.axes = {s};
    sel::CompletionTracker blind(kX);
    sel::CompletionTracker aimed(kX, {60.0});
    for (int i = 0; i < 100; ++i) {
        CHECK_EQ(blind.update(r, i * 0.02), Completion::pending);
        CHECK_EQ(aimed.update(r, i * 0.02), Completion::pending);
    }
    r.axes[0].status = 0x09;  // in use
    CHECK_EQ(blind.update(r, 3.0), Completion::pending);
    r.axes[0].status = 0x18;
    CHECK_EQ(blind.update(r, 3.1), Completion::done);
}

TEST(sel_link_wait_done_reports_cancel_and_timeout) {
    Bench b;
    b.link.send(sel::move_absolute(kX, {}, {150.0}));
    b.run(0.5);
    b.link.send(sel::stop(kX));
    CHECK_EQ(wait_done(b.link, kX, {150.0}), Completion::cancelled);
    CHECK(b.status(1).position < 150.0);
    b.link.send(sel::move_absolute(kX, {0.3, 0.3, 1.0}, {0.0}));  // over 1 mm/s: far beyond 2 s
    CHECK_THROWS(wait_done(b.link, kX, {0.0}, 2.0), MotionTimeoutError);
    CHECK_EQ(to_string(Completion::push_error), std::string("push_error"));
}
