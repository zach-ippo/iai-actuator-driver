// Axis unit: STOP first (the load plan's A4) on the simulator - a move frame never reaches the wire after a STOP from
// another thread that it did not see: not on a retry (the yield between two attempts lets the STOP out; the frame is
// not sent again), not when the STOP lands between the command's checks and its frame (the frame is refused, nothing
// sent), and no command releases a STOP that a stop sequence of another thread still holds. The interleavings are
// forced with latches on real time (a frame held on the wire, a request parked before it waits for the bus); the
// only timings are generous upper bounds on those latches. Also the stop latency line (C1), on the virtual clock with
// a delay injected before the STOP frame. No serial port is opened.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "axis_impl.hpp"
#include "filter_transport.hpp"
#include "harness.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"

using namespace iai_actuator_driver;
using ec_test::FilterTransport;

namespace {

using Bytes = std::vector<std::uint8_t>;

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

bool wait_for(const std::function<bool()>& predicate, double timeout_s = 10.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_s(0.001);
    }
    return predicate();
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// Parks the first thread that reaches it while armed, until released (10 s at most).
struct Latch {
    std::atomic<bool> armed{false};
    std::atomic<bool> parked{false};
    std::atomic<bool> released{false};
    /// Park here when armed (only the first caller); false when it was not armed.
    bool park() {
        if (!armed.exchange(false)) return false;
        parked = true;
        const double until = now_s() + 10.0;
        while (!released.load() && now_s() < until) sleep_s(0.001);
        return true;
    }
};

/// The bus test hooks for one test (bus_test_hooks is global): cleared again at the end.
struct BusHooks {
    BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
    ~BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
};

bool is_move_block(const Bytes& f) { return f.size() > 4 && f[1] == 0x10 && ((f[2] << 8) | f[3]) == reg::kDirectMoveBase; }
bool is_move_pdu(const Bytes& pdu) { return pdu.size() > 3 && pdu[0] == 0x10 && ((pdu[1] << 8) | pdu[2]) == reg::kDirectMoveBase; }

bool is_coil(const Bytes& f, int address, bool on) {
    return f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == address && f[4] == (on ? 0xFF : 0x00);
}

/// FC05 frames of the STOP coil in order ("1" = ON, "0" = OFF).
std::string stop_trace(const std::vector<Bytes>& frames) {
    std::string out;
    for (const Bytes& f : frames)
        if (f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop) out += f[4] == 0xFF ? '1' : '0';
    return out;
}

int count_moves(const std::vector<Bytes>& frames) {
    int n = 0;
    for (const Bytes& f : frames)
        if (is_move_block(f)) ++n;
    return n;
}

/// Index of the first STOP coil ON frame (to this slave or broadcast); -1 when none.
int first_stop_on(const std::vector<Bytes>& frames) {
    for (std::size_t i = 0; i < frames.size(); ++i)
        if (is_coil(frames[i], reg::kCoilStop, true)) return static_cast<int>(i);
    return -1;
}

std::shared_ptr<SimulatedUnit> make_sim() {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 20.0;
    o.position_mm = 50.0;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_axis(const std::shared_ptr<Transport>& t) {
    OpenOptions o;
    o.slave = 1;
    o.axis.stroke = Span{0.0, 150.0};
    o.axis.max_speed_mm_s = 100.0;
    o.axis.max_accel_g = 0.3;
    o.axis.poll_interval_s = 0.005;
    o.bus.min_gap_s = 0.0;
    o.bus.retries = 3;  // the default: the retry path is live
    o.line.kind = "direct";
    return Axis::open(t, o);
}

MoveOptions no_wait() {
    MoveOptions o;
    o.speed = 50.0;
    o.wait = false;
    return o;
}

/// A move_to whose 0x9900 block is held on the wire (the bus held) and gets no answer while `stop` waits for the bus:
/// the STOP goes between the block's attempts. Returns the frames written meanwhile and the mover's error text ("" when
/// it did not end with EcNotReadyError).
std::pair<std::vector<Bytes>, std::string> move_overtaken_between_attempts(const std::function<void(Axis&)>& stop) {
    auto sim = make_sim();
    Latch latch;
    auto t = std::make_shared<FilterTransport>(sim, [&latch](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (!is_move_block(f) || !latch.park()) return std::nullopt;
        return Bytes{};  // the block never reaches the unit: attempt 1 gets no answer
    });
    Axis ax = open_axis(t);
    t->clear_frames();
    latch.armed = true;
    std::string what;
    ec_test::Thread mover([&] {
        try {
            ax.move_to(100.0, no_wait());
        } catch (const EcNotReadyError& e) {
            what = e.what();
        }
    });
    // CHECK, not REQUIRE: the mover must be let go and joined whatever happens
    CHECK(wait_for([&] { return latch.parked.load(); }));  // attempt 1 on the wire, the bus held
    ec_test::Thread stopper([&] { stop(ax); });
    CHECK(wait_for([&] { return ax.bus()->bus_lock().waiting(BusPriority::Stop) == 1; }));
    latch.released = true;
    mover.join();
    stopper.join();
    CHECK(!sim->moving());
    return {t->frames(), what};
}

}  // namespace

TEST(axis_stop_first_a_move_is_never_sent_again_after_a_stop_between_its_attempts) {
    // Before: the 0x9900 block's retry (3 by default) went out again after the STOP the yield let through - a move
    // after the STOP. Now the block goes out once, before the STOP, and move_to says why it ended.
    const auto result = move_overtaken_between_attempts([](Axis& ax) { ax.stop(); });
    const std::vector<Bytes>& frames = result.first;
    CHECK_EQ(count_moves(frames), 1);
    const int stop_at = first_stop_on(frames);
    REQUIRE(stop_at > 0);
    for (std::size_t i = static_cast<std::size_t>(stop_at); i < frames.size(); ++i) CHECK(!is_move_block(frames[i]));
    CHECK_EQ(stop_trace(frames), std::string("10"));  // the stop's own ON and OFF, nothing else
    CHECK(contains(result.second, "a STOP went first"));
    CHECK(contains(result.second, "not re-sent"));
}

TEST(axis_stop_first_a_move_is_never_sent_again_after_a_broadcast_stop_between_its_attempts) {
    // the same with a broadcast STOP (Port::broadcast_stop sends it so): it counts for every unit of the line
    const auto result = move_overtaken_between_attempts([](Axis& ax) {
        const StopPriority stop_priority;
        ax.bus()->broadcast_coil(reg::kCoilStop, true, 0.0);
    });
    const std::vector<Bytes>& frames = result.first;
    CHECK_EQ(count_moves(frames), 1);
    const int stop_at = first_stop_on(frames);
    REQUIRE(stop_at > 0);
    CHECK(frames[static_cast<std::size_t>(stop_at)][0] == 0x00);  // the broadcast
    for (std::size_t i = static_cast<std::size_t>(stop_at); i < frames.size(); ++i) CHECK(!is_move_block(frames[i]));
    CHECK(contains(result.second, "a STOP went first"));
}

TEST(axis_stop_first_a_move_decided_before_a_stop_from_another_thread_is_refused) {
    // The mover passed its checks and is parked before its 0x9900 block waits for the bus; meanwhile another thread's
    // stop() runs to its end (STOP ON, standstill, STOP OFF). Before: the block went out after the STOP and the axis
    // moved. Now it is refused: nothing sent, the axis stays where it stopped.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (is_move_pdu(pdu)) latch.park();
    };
    t->clear_frames();
    latch.armed = true;
    std::string what;
    ec_test::Thread mover([&] {
        try {
            ax.move_to(100.0, no_wait());
        } catch (const EcNotReadyError& e) {
            what = e.what();
        }
    });
    CHECK(wait_for([&] { return latch.parked.load(); }));
    try {
        ax.stop(true);
    } catch (const std::exception& e) {
        std::printf("  stop failed: %s\n", e.what());
    }
    latch.released = true;
    mover.join();
    const std::vector<Bytes> frames = t->frames();
    CHECK_EQ(count_moves(frames), 0);
    CHECK_EQ(stop_trace(frames), std::string("10"));
    CHECK(contains(what, "a STOP arrived meanwhile"));
    CHECK(contains(what, "nothing sent"));
    sleep_s(0.05);
    CHECK(!sim->moving());
    CHECK(sim->position_mm() < 51.0);
}

TEST(axis_stop_first_a_command_never_releases_a_stop_another_thread_holds) {
    // A stop sequence holds STOP ON until the axis stands still (parked here at its first status read after the STOP
    // frame). Before: a move from another thread released that STOP (STOP OFF) and went out under the stopping thread.
    // Now the move is refused ("a STOP is held"), nothing sent; the stop releases its own STOP when it ends.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (StopPriority::active() && !pdu.empty() && pdu[0] == 0x03) latch.park();
    };
    t->clear_frames();
    latch.armed = true;
    ec_test::Thread stopper([&] { ax.stop(); });
    CHECK(wait_for([&] { return latch.parked.load(); }));
    CHECK(sim->stop_held());
    std::string what;
    try {
        ax.move_to(100.0, no_wait());
    } catch (const EcNotReadyError& e) {
        what = e.what();
    }
    CHECK(contains(what, "a STOP is held"));
    CHECK(sim->stop_held());  // still the stopping thread's
    CHECK_EQ(stop_trace(t->frames()), std::string("1"));
    CHECK_EQ(count_moves(t->frames()), 0);
    latch.released = true;
    stopper.join();
    CHECK(!sim->stop_held());
    CHECK_EQ(stop_trace(t->frames()), std::string("10"));
    CHECK_EQ(count_moves(t->frames()), 0);
}

TEST(axis_stop_first_a_command_never_takes_control_while_another_thread_holds_a_stop) {
    // PIO mode (PMSS 0), at rest, and a stop sequence of another thread holds STOP ON (parked at its first status read
    // after the STOP frame). Before: servo(true) / move_to took control (PMSL ON: PIO -> PC) and only then were refused
    // ("a STOP is held"). Now the take-over is refused first: no PMSL write, the control mode stays PIO.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    ax.disable_modbus_control();
    REQUIRE(!sim->pmsl());
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (StopPriority::active() && !pdu.empty() && pdu[0] == 0x03) latch.park();
    };
    t->clear_frames();
    latch.armed = true;
    ec_test::Thread stopper([&] { ax.stop(); });
    CHECK(wait_for([&] { return latch.parked.load(); }));
    const std::function<void()> commands[] = {[&] { ax.servo(true); }, [&] { ax.move_to(100.0, no_wait()); }};
    for (const auto& command : commands) {
        std::string what;
        try {
            command();
        } catch (const EcNotReadyError& e) {
            what = e.what();
        }
        CHECK(contains(what, "a STOP is held"));
    }
    for (const Bytes& f : t->frames()) CHECK(!is_coil(f, reg::kCoilPmsl, true));
    CHECK(!sim->pmsl());
    CHECK_EQ(count_moves(t->frames()), 0);
    latch.released = true;
    stopper.join();
    CHECK(!sim->stop_held());
    CHECK(!sim->pmsl());
}

TEST(axis_stop_first_two_stops_at_once_release_the_stop_once_after_both) {
    // Two threads stop the axis at once: neither releases the STOP while the other still holds it; the last to end
    // releases it, and no STOP is left held.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (StopPriority::active() && !pdu.empty() && pdu[0] == 0x03) latch.park();
    };
    t->clear_frames();
    latch.armed = true;
    ec_test::Thread first([&] { ax.stop(); });   // parked at its first status read, STOP held
    CHECK(wait_for([&] { return latch.parked.load(); }));
    ax.stop();  // the second ends first: it leaves the STOP to the first
    CHECK(sim->stop_held());
    CHECK_EQ(stop_trace(t->frames()), std::string("11"));
    latch.released = true;
    first.join();
    CHECK(!sim->stop_held());
    CHECK_EQ(stop_trace(t->frames()), std::string("110"));
}

namespace {

bool is_write(const Bytes& f) { return f.size() > 1 && (f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x0F || f[1] == 0x10); }

bool is_drg1_read(const Bytes& pdu) {
    return pdu.size() >= 5 && pdu[0] == 0x03 && ((pdu[1] << 8) | pdu[2]) == reg::kDrg1;
}

}  // namespace

TEST(axis_stop_first_a_resume_decided_before_a_stop_from_another_thread_is_refused) {
    // The register STOP (no coils, e.g. behind an RCON gateway) is the DRG1 pause. A resume() has decided and is parked
    // at its DRG1 read, before its STP-off write; meanwhile another thread's stop() runs to its end. Before: the
    // resume's write went out after that STOP and released its pause. Now it is refused: nothing written after it.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    ax.set_strategy(CommandMethod::Register);
    ax.pause();
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (is_drg1_read(pdu)) latch.park();
    };
    latch.armed = true;
    std::string what;
    bool attempted = false;
    ec_test::Thread resumer([&] {
        try {
            ax.resume();
        } catch (const EcNotReadyError& e) {
            what = e.what();
            attempted = e.stop_attempted();
        }
    });
    CHECK(wait_for([&] { return latch.parked.load(); }));
    try {
        ax.stop(true);
    } catch (const std::exception& e) {
        std::printf("  stop failed: %s\n", e.what());
    }
    const std::size_t after_stop = t->frames().size();
    latch.released = true;
    resumer.join();
    const std::vector<Bytes> frames = t->frames();
    for (std::size_t i = after_stop; i < frames.size(); ++i) CHECK(!is_write(frames[i]));  // the DRG1 read only
    CHECK(contains(what, "a STOP arrived meanwhile"));
    CHECK(contains(what, "nothing sent"));
    CHECK(attempted);
}

TEST(axis_stop_first_a_resume_never_releases_a_stop_another_thread_holds) {
    // A stop sequence holds STOP ON until the axis stands still (parked at its first status read after the STOP
    // frame). Before: a resume() from another thread sent STP OFF under it. Now it is refused, nothing sent.
    BusHooks hooks;
    auto sim = make_sim();
    auto t = std::make_shared<FilterTransport>(sim, nullptr);
    Axis ax = open_axis(t);
    ax.pause();
    Latch latch;
    detail::bus_test_hooks().before_bus_wait = [&latch](int, const Bytes& pdu) {
        if (StopPriority::active() && !pdu.empty() && pdu[0] == 0x03) latch.park();
    };
    t->clear_frames();
    latch.armed = true;
    ec_test::Thread stopper([&] { ax.stop(); });
    CHECK(wait_for([&] { return latch.parked.load(); }));
    std::string what;
    try {
        ax.resume();
    } catch (const EcNotReadyError& e) {
        what = e.what();
    }
    CHECK(contains(what, "a STOP is held"));
    CHECK_EQ(stop_trace(t->frames()), std::string("1"));
    for (const Bytes& f : t->frames()) CHECK(!is_coil(f, reg::kCoilStp, false));
    latch.released = true;
    stopper.join();
    CHECK_EQ(stop_trace(t->frames()), std::string("10"));
    for (const Bytes& f : t->frames()) CHECK(!is_coil(f, reg::kCoilStp, false));
}

TEST(axis_stop_first_a_stale_stop_release_whose_answer_is_lost_is_sent_again) {
    // begin_stop leaves STOP ON; the next move releases it first (STOP OFF, outside a stop scope: counted and gated).
    // Before: its own count refused its retry ("a STOP went first ... not re-sent"), so one lost answer failed the
    // move and left the STOP held. Now the release is sent again and the move goes out.
    auto sim = make_sim();
    std::atomic<int> releases{0};
    auto t = std::make_shared<FilterTransport>(sim, [&releases](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (is_coil(f, reg::kCoilStop, false) && releases.fetch_add(1) == 0) return Bytes{};  // the first: no answer
        return std::nullopt;
    });
    Axis ax = open_axis(t);
    ax.begin_stop();
    CHECK(sim->stop_held());
    t->clear_frames();
    std::string what;
    try {
        ax.move_to(100.0, no_wait());
    } catch (const std::exception& e) {
        what = e.what();
    }
    CHECK_EQ(what, std::string());
    CHECK_EQ(stop_trace(t->frames()), std::string("00"));  // the release and its retry
    CHECK_EQ(count_moves(t->frames()), 1);
    CHECK(!sim->stop_held());
    ax.stop();
}

// ============================================================================================ the stop latency line

namespace {

/// The stop latency lines (bus_stop_timing.hpp) logged while it lives.
struct StopLines {
    std::mutex mutex;
    std::vector<std::string> lines;
    LogHandler previous;
    StopLines() {
        previous = set_log_handler([this](LogLevel level, const std::string&, const std::string& message) {
            if (level != LogLevel::Info || message.find(" on the wire ") == std::string::npos) return;
            std::lock_guard<std::mutex> lock(mutex);
            lines.push_back(message);
        });
    }
    ~StopLines() { set_log_handler(previous); }
    std::vector<std::string> all() {
        std::lock_guard<std::mutex> lock(mutex);
        return lines;
    }
};

/// The number in `text` right after `before` (-1 when absent).
int number_after(const std::string& text, const std::string& before) {
    const std::size_t at = text.find(before);
    if (at == std::string::npos) return -1;
    return std::atoi(text.c_str() + at + before.size());
}

}  // namespace

TEST(axis_stop_logs_its_latency_from_the_call_to_the_wire_and_on_to_standstill) {
    // C1 on the virtual clock (delay injection): the PC is 40 ms late between the stop() call and its STOP frame (the
    // bus hook moves the clock on there). The stop's one line names that delay and the time from the frame to the
    // standstill it confirmed; the clock moves only on the stop's own polls, so that is the time the stop took after
    // the frame.
    BusHooks hooks;
    VirtualClockScope clock;
    StopLines lines;
    SimulatedUnitOptions so;
    so.profile = SimProfile::Slider;
    so.position_mm = 50.0;  // time_scale 1 on the installed clock
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = open_axis(sim);
    ax.move_to(100.0, no_wait());
    clock.clock().advance(0.3);
    REQUIRE(sim->moving());
    detail::bus_test_hooks().before_bus_wait = [&clock](int, const Bytes& pdu) {
        if (pdu.size() >= 5 && pdu[0] == 0x05 && ((pdu[1] << 8) | pdu[2]) == reg::kCoilStop && pdu[3] == 0xFF)
            clock.clock().advance(0.040);
    };
    const double called = now_s();
    ax.stop();
    const double ended = now_s();
    CHECK(!sim->moving());
    const std::vector<std::string> got = lines.all();
    REQUIRE(got.size() == 1u);
    CHECK(contains(got[0], "slave 1: STOP on the wire 40 ms after the call, standstill confirmed "));
    const int still_ms = number_after(got[0], "standstill confirmed ");
    const double expected_ms = (ended - called - 0.040) * 1000.0;
    CHECK(expected_ms > 10.0);  // the axis was moving: deceleration and settle took clock time
    CHECK(std::abs(still_ms - expected_ms) <= 1.0);
    // begin_stop: the frame only (it does not wait for standstill)
    ax.move_to(60.0, no_wait());
    clock.clock().advance(0.3);
    ax.begin_stop();
    const std::vector<std::string> after = lines.all();
    REQUIRE(after.size() == 2u);
    CHECK(contains(after[1], "slave 1: STOP on the wire 40 ms after the call"));
    CHECK(!contains(after[1], "standstill"));
    ax.stop();
}
