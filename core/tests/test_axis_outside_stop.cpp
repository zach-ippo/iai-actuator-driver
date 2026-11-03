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
