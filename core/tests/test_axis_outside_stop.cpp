// Axis unit: a motion wait and what other threads do to its move (the hardware findings of 2026-11-03 on a PCON-CB +
// RCP6, ON-WIRE): a stop() of the same axis from another thread ends a wait in move_to(wait) / MoveHandle::wait() /
// MoveHandle::poll() / push(wait) at once, and the wait sends no STOP of its own. Simulator on real time (no COM
// port); every timing is a generous upper bound for a busy PC.
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/timing.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

bool wait_until(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_s(0.002);
    }
    return predicate();
}

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// The Tx frames of a bus, in wire order (the frame logger runs under the bus lock).
struct TxLog {
    std::mutex mutex;
    std::vector<Bytes> frames;
    FrameLogger logger() {
        return [this](FrameDirection d, const Bytes& f) {
            if (d != FrameDirection::Tx) return;
            std::lock_guard<std::mutex> lock(mutex);
            frames.push_back(f);
        };
    }
    /// STOP coil ON frames so far.
    int stops_on() {
        std::lock_guard<std::mutex> lock(mutex);
        int n = 0;
        for (const auto& f : frames)
            n += f.size() > 4 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop && f[4] == 0xFF;
        return n;
    }
    /// FC03 reads after the last FC16 write (a move's frame): the status reads of a wait since its move went out.
    int reads_since_write() {
        std::lock_guard<std::mutex> lock(mutex);
        int n = 0;
        for (const auto& f : frames) {
            if (f.size() < 2) continue;
            if (f[1] == 0x10) n = 0;
            if (f[1] == 0x03) ++n;
        }
        return n;
    }
};

/// One simulated unit on real time behind a bus that logs its frames.
struct Rig {
    std::shared_ptr<SimulatedUnit> sim;
    TxLog tx;
    std::shared_ptr<ModbusRtu> bus;
    std::unique_ptr<Axis> ax;
    explicit Rig(std::optional<double> position_mm = std::nullopt) {
        SimulatedUnitOptions so;
        so.time_scale = 1.0;
        so.position_mm = position_mm;
        sim = std::make_shared<SimulatedUnit>(so);
        BusOptions bo;
        bo.frame_logger = tx.logger();
        bus = std::make_shared<ModbusRtu>(sim, bo);
        ax = std::make_unique<Axis>(bus, 1);
        ax->connect();
    }
};

MoveOptions slow(double speed, bool wait = true) {
    MoveOptions o;
    o.speed = speed;
    o.accel_g = 0.3;
    o.wait = wait;
    return o;
}

/// What a waiting thread ended with.
struct Outcome {
    std::atomic<bool> ended{false};
    std::atomic<double> at{0.0};
    std::string type;
    std::string what;
    bool stop_attempted = false;
};

/// Run `wait` in a thread, stop the axis from this one once `ready` (default: it moves), and return when the waiter
/// ended.
void stop_from_another_thread(Rig& r, const std::function<void()>& wait, Outcome& out, double& stop_returned,
                              std::function<bool()> ready = {}) {
    ec_test::Thread waiter([&] {
        try {
            wait();
            out.type = "none";
        } catch (const MotionIncompleteError& e) {
            out.type = "MotionIncompleteError";
            out.what = e.what();
            out.stop_attempted = e.stop_attempted();
        } catch (const EcDriverError& e) {
            out.type = "other";
            out.what = e.what();
        }
        out.at = now_s();
        out.ended = true;
    });
    if (!ready) ready = [&] { return r.sim->moving() && r.sim->position_mm() > 2.0; };
    REQUIRE(wait_until(ready));
    r.ax->stop();
    stop_returned = now_s();
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
}

void check_ended_on_the_stop(Rig& r, const Outcome& out, double stop_returned) {
    CHECK_EQ(out.type, std::string("MotionIncompleteError"));
    CHECK(has(out.what, "stopped from outside this wait"));
    CHECK(has(out.what, "sent no STOP"));
    CHECK(!has(out.what, "decel stop sent"));
    CHECK(out.stop_attempted);
    // at once: before, the wait ran into its settled-away check (0.5 s) or its timeout, then sent its own STOP
    CHECK(out.at.load() - stop_returned < 0.3);
    CHECK_EQ(r.tx.stops_on(), 1);  // stop()'s own STOP, no second one from the wait
    sleep_s(0.6);                  // a late STOP of the wait would come by now
    CHECK_EQ(r.tx.stops_on(), 1);
    CHECK(!r.sim->stop_held());
    CHECK(!r.sim->moving());
}

}  // namespace

TEST(axis_outside_stop_ends_a_move_to_wait_at_once_without_a_second_stop) {
    Rig r;
    Outcome out;
    double stop_returned = 0.0;
    stop_from_another_thread(r, [&] { r.ax->move_to(140.0, slow(20.0)); }, out, stop_returned);
    check_ended_on_the_stop(r, out, stop_returned);
    // the axis takes the next move as usual
    const std::optional<Status> st = r.ax->move_to(r.sim->position_mm() + 1.0, slow(20.0));
    REQUIRE(st.has_value());
    CHECK_EQ(r.tx.stops_on(), 1);
}

TEST(axis_outside_stop_ends_a_move_handle_wait_at_once_without_a_second_stop) {
    Rig r;
    MoveHandle handle = r.ax->start_move_to(140.0, slow(20.0, false));
    Outcome out;
    double stop_returned = 0.0;
    stop_from_another_thread(r, [&] { handle.wait(); }, out, stop_returned);
    check_ended_on_the_stop(r, out, stop_returned);
}

TEST(axis_outside_stop_ends_a_move_handle_poll_at_once_without_a_second_stop) {
    // the bindings wait in poll() slices (iad_move_handle_poll)
    Rig r;
    MoveHandle handle = r.ax->start_move_to(140.0, slow(20.0, false));
    Outcome out;
    double stop_returned = 0.0;
    stop_from_another_thread(
        r,
        [&] {
            while (!handle.poll(0.1)) {
            }
        },
        out, stop_returned);
    check_ended_on_the_stop(r, out, stop_returned);
}

PushOptions push_at(double speed) {
    PushOptions o;
    o.speed = speed;
    o.accel_g = 0.3;
    return o;
}

TEST(axis_outside_stop_ends_a_push_wait_in_its_approach_at_once) {
    Rig r(10.0);
    Outcome out;
    double stop_returned = 0.0;
    stop_from_another_thread(r, [&] { r.ax->push(60.0, 40.0, 30.0, push_at(20.0)); }, out, stop_returned,
                             [&] { return r.sim->moving() && r.sim->position_mm() > 15.0; });
    check_ended_on_the_stop(r, out, stop_returned);
    CHECK(has(out.what, "push (approach 60.00 mm"));
    CHECK(r.sim->position_mm() < 59.0);
}

TEST(axis_outside_stop_ends_a_push_wait_in_its_push_at_once) {
    Rig r(50.0);
    Outcome out;
    double stop_returned = 0.0;
    stop_from_another_thread(r, [&] { r.ax->push(55.0, 40.0, 30.0, push_at(20.0)); }, out, stop_returned,
                             [&] { return r.sim->moving() && r.sim->position_mm() > 57.0; });
    check_ended_on_the_stop(r, out, stop_returned);
    CHECK(has(out.what, "push (approach 55.00 mm"));
    CHECK(r.sim->position_mm() < 94.0);
}

TEST(axis_outside_stop_a_pause_does_not_end_the_wait) {
    // a pause from another thread is not a stop: the wait goes on and the move finishes after resume()
    Rig r;
    Outcome out;
    ec_test::Thread waiter([&] {
        try {
            r.ax->move_to(20.0, slow(20.0));
            out.type = "none";
        } catch (const EcDriverError& e) {
            out.type = "error";
            out.what = e.what();
        }
        out.ended = true;
    });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 2.0; }));
    r.ax->pause();
    sleep_s(0.3);
    CHECK(!out.ended.load());
    r.ax->resume();
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    CHECK_EQ(out.type, std::string("none"));
    CHECK_NEAR(r.sim->position_mm(), 20.0, 0.11);
}

// ============================================================================================ superseded moves

namespace {

/// Run `body` and record how it ended.
void record(Outcome& out, const std::function<void()>& body) {
    try {
        body();
        out.type = "none";
    } catch (const MotionIncompleteError& e) {
        out.type = "MotionIncompleteError";
        out.what = e.what();
        out.stop_attempted = e.stop_attempted();
    } catch (const EcDriverError& e) {
        out.type = "other";
        out.what = e.what();
    }
    out.at = now_s();
    out.ended = true;
}

void check_superseded(const Outcome& out) {
    CHECK_EQ(out.type, std::string("MotionIncompleteError"));
    CHECK(has(out.what, "superseded by a newer move of this axis"));
    CHECK(!has(out.what, "decel stop sent"));
    CHECK(out.stop_attempted);
}

}  // namespace

TEST(axis_outside_stop_a_superseded_handle_ends_at_once_without_a_stop) {
    // a handle whose move a newer move_to re-targeted: wait() ended only at its own timeout, then sent a STOP that
    // would cut the newer move
    Rig r(20.0);
    MoveHandle first = r.ax->start_move_to(130.0, slow(50.0, false));
    REQUIRE(wait_until([&] { return r.sim->moving(); }));
    MoveHandle second = r.ax->start_move_to(30.0, slow(50.0, false));
    Outcome out;
    const double t0 = now_s();
    record(out, [&] { first.wait(1.0); });
    check_superseded(out);
    CHECK(out.at.load() - t0 < 0.3);
    CHECK(r.sim->moving() || std::fabs(r.sim->position_mm() - 30.0) < 0.2);  // the newer move runs on
    const Status st = second.wait();
    CHECK_NEAR(st.position_mm, 30.0, 0.11);
    CHECK_EQ(r.tx.stops_on(), 0);
    // poll() and done() end the same way
    Outcome polled;
    record(polled, [&] {
        while (!first.poll(0.1)) {
        }
    });
    check_superseded(polled);
    CHECK_EQ(r.tx.stops_on(), 0);
}

TEST(axis_outside_stop_a_move_from_another_thread_ends_a_move_to_wait_at_once) {
    Rig r(20.0);
    Outcome out;
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->move_to(140.0, slow(20.0)); }); });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 22.0; }));
    const double t0 = now_s();
    Outcome newer;
    ec_test::Thread mover([&] { record(newer, [&] { r.ax->move_to(15.0, slow(20.0)); }); });
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    check_superseded(out);
    CHECK(out.at.load() - t0 < 0.3);
    mover.join();
    CHECK_EQ(newer.type, std::string("none"));
    CHECK_NEAR(r.sim->position_mm(), 15.0, 0.11);
    CHECK_EQ(r.tx.stops_on(), 0);
}

TEST(axis_outside_stop_a_move_from_another_thread_ends_a_push_wait_at_once) {
    Rig r(10.0);
    Outcome out;
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->push(60.0, 40.0, 30.0, push_at(20.0)); }); });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 15.0; }));
    const double t0 = now_s();
    Outcome newer;
    ec_test::Thread mover([&] { record(newer, [&] { r.ax->move_to(12.0, slow(20.0)); }); });
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    check_superseded(out);
    CHECK(has(out.what, "push (approach 60.00 mm"));
    CHECK(out.at.load() - t0 < 0.3);
    mover.join();
    CHECK_EQ(newer.type, std::string("none"));
    CHECK_NEAR(r.sim->position_mm(), 12.0, 0.11);
    CHECK_EQ(r.tx.stops_on(), 0);
}

TEST(axis_outside_stop_a_speed_change_does_not_supersede_the_move) {
    Rig r(20.0);
    Outcome out;
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->move_to(60.0, slow(20.0)); }); });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 22.0; }));
    r.ax->change_speed(40.0, false);
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    CHECK_EQ(out.type, std::string("none"));
    CHECK_NEAR(r.sim->position_mm(), 60.0, 0.11);
}

// ================================================================================== change_speed and the deadline

TEST(axis_outside_stop_a_slower_speed_from_another_thread_extends_the_estimated_timeout) {
    // A's timeout was estimated for 100 mm/s; B slows the move to 5 mm/s: before, A timed out (and stopped the axis)
    Rig r(20.0);
    MoveOptions fast = slow(100.0);
    Outcome out;
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->move_to(40.0, fast); }); });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 21.0; }));
    r.ax->change_speed(5.0, false);
    CHECK(wait_until([&] { return out.ended.load(); }, 15.0));
    waiter.join();
    CHECK_EQ(out.type, std::string("none"));
    CHECK(out.what.empty());
    CHECK_NEAR(r.sim->position_mm(), 40.0, 0.11);
    CHECK_EQ(r.tx.stops_on(), 0);
}

TEST(axis_outside_stop_a_slower_speed_extends_a_handle_deadline_but_not_an_explicit_timeout) {
    Rig r(20.0);
    MoveHandle handle = r.ax->start_move_to(40.0, slow(100.0, false));
    const double deadline = handle.deadline();
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 21.0; }));
    r.ax->change_speed(5.0, false);
    Outcome out;
    record(out, [&] {
        while (!handle.poll(0.1)) {
        }
    });
    CHECK_EQ(out.type, std::string("none"));
    CHECK(handle.deadline() > deadline + 1.0);
    CHECK_NEAR(r.sim->position_mm(), 40.0, 0.11);
    CHECK_EQ(r.tx.stops_on(), 0);
    // a timeout the caller passes stays the caller's
    MoveHandle second = r.ax->start_move_to(60.0, slow(100.0, false));
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 41.0; }));
    r.ax->change_speed(5.0, false);
    CHECK_THROWS(second.wait(0.5), MotionTimeoutError);
}

// ======================================================================================= move_by after a fast step

TEST(axis_outside_stop_fast_relative_steps_add_to_the_commanded_target) {
    // 50 x move_by(-0.02, completion fast): each step returns on the first in-band sample while PNOW still lags; added
    // to PNOW the steps were lost (0.69 mm of 1.00 mm on hardware). Added to the last target they all arrive.
    Rig r(60.0);
    MoveOptions step = slow(5.0);
    step.accel_g = 0.1;
    step.completion = Completion::Fast;
    const double start = r.ax->status().position_mm;
    for (int i = 0; i < 50; ++i) r.ax->move_by(-0.02, step);
    const Status st = r.ax->wait_idle(start - 1.0, 0.02, 10.0);
    CHECK_NEAR(st.position_mm, start - 1.0, 0.015);
    CHECK_EQ(r.tx.stops_on(), 0);
}

TEST(axis_outside_stop_relative_steps_count_from_the_position_after_a_safe_move_or_a_stop) {
    Rig r(60.0);
    // a move the safe rule confirmed: the step counts from PNOW (the documented rule)
    MoveOptions o = slow(20.0);
    r.ax->move_to(50.0, o);
    CHECK_NEAR(r.ax->move_by(1.0, o)->position_mm, 51.0, 0.11);
    // a fast move cut short by a stop: the step counts from where it stopped, not from the old target
    MoveOptions fast = slow(20.0, false);
    fast.completion = Completion::Fast;
    r.ax->move_to(90.0, fast);
    REQUIRE(wait_until([&] { return r.sim->position_mm() > 55.0; }));
    const Status stopped = r.ax->stop();
    const std::optional<Status> st = r.ax->move_by(1.0, o);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, stopped.position_mm + 1.0, 0.15);
}

// ============================================================================================ stops of other masters

TEST(axis_outside_stop_a_stop_of_another_axis_object_on_the_line_ends_the_wait_at_once) {
    // another Axis object of the same slave on this bus (another part of the program): its STOP ends this wait too,
    // and the wait sends no second STOP
    Rig r;
    Axis other(r.bus, 1);
    Outcome out;
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->move_to(140.0, slow(20.0)); }); });
    REQUIRE(wait_until([&] { return r.sim->moving() && r.sim->position_mm() > 2.0; }));
    other.stop();
    const double stop_returned = now_s();
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    check_ended_on_the_stop(r, out, stop_returned);
    CHECK(has(out.what, "another Axis object on this line"));
}

TEST(axis_outside_stop_a_stop_of_another_program_seen_in_the_status_ends_the_wait) {
    // UNVERIFIED (simulator only): another program (another master: its own bus on the same unit) sends the STOP coil;
    // this bus sees nothing of it, the status shows the move cancelled short of the target. The wait ends with the
    // stopped-from-outside outcome before the settled-away check (0.5 s), and sends no STOP of its own.
    Rig r;
    auto other = std::make_shared<ModbusRtu>(r.sim);
    Outcome out;
    const double start = r.sim->position_mm();  // 73.95 mm: "> 2.0" held at once, and the STOP could come first
    ec_test::Thread waiter([&] { record(out, [&] { r.ax->move_to(140.0, slow(20.0)); }); });
    // the STOP once the wait saw the axis moving (two status reads since the move: the first one completed), whatever
    // the host's timing; before, a fast host stopped the axis ahead of the wait's first read (a wait that never saw
    // the move cannot tell it from an ignored one and keeps the settled-away check)
    REQUIRE(wait_until([&] {
        return r.sim->moving() && r.sim->position_mm() > start + 2.0 && r.tx.reads_since_write() >= 2;
    }));
    other->write_coil(1, reg::kCoilStop, true);
    REQUIRE(wait_until([&] { return !r.sim->moving(); }));
    const double stopped_at = now_s();
    other->write_coil(1, reg::kCoilStop, false);
    CHECK(wait_until([&] { return out.ended.load(); }, 10.0));
    waiter.join();
    CHECK_EQ(out.type, std::string("MotionIncompleteError"));
    CHECK(has(out.what, "stopped from outside this wait"));
    CHECK(has(out.what, "another program or master"));
    CHECK(has(out.what, "sent no STOP"));
    CHECK(out.stop_attempted);
    CHECK(out.at.load() - stopped_at < 0.45);  // the settle window, not the settled-away check
    CHECK_EQ(r.tx.stops_on(), 0);               // no STOP from this bus
    sleep_s(0.6);
    CHECK_EQ(r.tx.stops_on(), 0);
    CHECK(r.sim->position_mm() < 139.0);
}
