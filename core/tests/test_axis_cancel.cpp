// Axis unit: what switching the bindings onto the core needs - a STOP first in the bus queue while status monitors
// and other threads poll the port flat out, cancel_waits() ending a move / home wait (with its decel stop) from another
// thread, the status monitors stopped and joined promptly when their bus closes or they stop while waiting for the bus,
// and home completion with polls too sparse to see the home return start (two axes polled in turn on one line).
// Simulator only (no COM port). Ordering is checked with the bus held on purpose (deterministic on a busy PC); every
// timing is a generous upper bound.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
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

bool is_stop_on(const Bytes& f) {
    return f.size() > 4 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop && f[4] == 0xFF;
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
    std::vector<Bytes> all() {
        std::lock_guard<std::mutex> lock(mutex);
        return frames;
    }
    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return frames.size();
    }
};

/// A transport passed through, `before_write` run before every request (a stepped clock).
class Wrapped final : public Transport {
public:
    Wrapped(std::shared_ptr<Transport> inner, std::function<void()> before_write)
        : inner_(std::move(inner)), before_write_(std::move(before_write)) {}
    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override {
        if (before_write_) before_write_();
        inner_->write(data, size);
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override { return inner_->read(buffer, size); }
    void reset_input() override { inner_->reset_input(); }
    int baud_rate() const override { return inner_->baud_rate(); }
    void set_baud_rate(int rate) override { inner_->set_baud_rate(rate); }
    double read_timeout() const override { return inner_->read_timeout(); }
    void set_read_timeout(double seconds) override { inner_->set_read_timeout(seconds); }
    bool has_real_timeout() const override { return inner_->has_real_timeout(); }
    void send_break(double seconds) override { inner_->send_break(seconds); }
    void close() override { inner_->close(); }
    bool is_open() const override { return inner_->is_open(); }
    std::string name() const override { return inner_->name(); }

private:
    std::shared_ptr<Transport> inner_;
    std::function<void()> before_write_;
};

/// A simulated clock that only moves `step_s` per request: the axis is seen exactly step_s later at every poll,
/// whatever the host's speed (polls as sparse as wanted, deterministically).
struct SteppedClock {
    std::shared_ptr<double> t = std::make_shared<double>(1000.0);
    std::shared_ptr<std::mutex> m = std::make_shared<std::mutex>();
    double step_s;
    explicit SteppedClock(double step) : step_s(step) {}
    std::function<double()> clock() const {
        auto t_ = t;
        auto m_ = m;
        return [t_, m_] {
            std::lock_guard<std::mutex> lock(*m_);
            return *t_;
        };
    }
    std::function<void()> stepper() const {
        auto t_ = t;
        auto m_ = m;
        const double s = step_s;
        return [t_, m_, s] {
            std::lock_guard<std::mutex> lock(*m_);
            *t_ += s;
        };
    }
};

MoveOptions slow(double speed, bool wait = true) {
    MoveOptions o;
    o.speed = speed;
    o.accel_g = 0.3;
    o.wait = wait;
    return o;
}

/// Collects warnings of one log source for the scope.
struct Warnings {
    std::mutex mutex;
    std::vector<std::string> items;
    LogHandler previous;
    Warnings() {
        previous = set_log_handler([this](LogLevel level, const std::string&, const std::string& message) {
            if (level < LogLevel::Warning) return;
            std::lock_guard<std::mutex> lock(mutex);
            items.push_back(message);
        });
    }
    ~Warnings() { set_log_handler(previous); }
    bool any(const std::string& part) {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& m : items)
            if (has(m, part)) return true;
        return false;
    }
};

}  // namespace

// ============================================================================================ STOP priority

TEST(axis_stop_goes_out_first_while_monitors_and_pollers_run_flat_out) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto line = SimLine::with_slaves({1, 2}, so);
    TxLog tx;
    BusOptions bo;
    bo.frame_logger = tx.logger();
    auto bus = std::make_shared<ModbusRtu>(line, bo);
    Axis a1(bus, 1);
    Axis a2(bus, 2);
    a1.connect();
    a2.connect();
    MonitorOptions mo;
    mo.gap_s = 0.0;  // worst case: back to back
    auto m1 = a1.start_monitor(mo);
    auto m2 = a2.start_monitor(mo);
    std::atomic<bool> run{true};
    std::vector<std::thread> pollers;
    for (int i = 0; i < 2; ++i) {
        pollers.emplace_back([&] {
            while (run.load()) {
                try {
                    a2.status();
                } catch (const std::exception&) {
                }
            }
        });
    }
    REQUIRE(wait_until([&] { return m1->polls() >= 10 && m2->polls() >= 10; }));
    for (int round = 0; round < 3; ++round) {
        CHECK(!a1.move_to(100.0 - 20.0 * round, slow(10.0, false)).has_value());
        REQUIRE(wait_until([&] { return line->unit(1)->moving(); }));
        BusLock& lock = bus->bus_lock();
        lock.lock();  // hold the port: the monitors and pollers queue behind
        REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) >= 3; }));
        ec_test::Thread stopper([&] { a1.stop(); });
        // CHECK, not REQUIRE: the stop waits for the bus held here, so the test goes on to give it back and join
        CHECK(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
        const std::size_t marker = tx.size();
        lock.unlock();
        stopper.join();
        const auto frames = tx.all();
        REQUIRE(frames.size() > marker);
        CHECK(is_stop_on(frames[marker]));  // the next frame on the wire: no queued poll went before it
        CHECK(!line->unit(1)->moving());
    }
    CHECK(m1->running());
    CHECK(m2->running());
    run = false;
    for (auto& t : pollers) t.join();
}

// ============================================================================================ cancel_waits

TEST(axis_cancel_waits_ends_a_move_wait_with_a_decel_stop) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    std::atomic<bool> cancelled{false};
    std::string what;
    ec_test::Thread mover([&] {
        try {
            ax.move_to(100.0, slow(10.0));
        } catch (const WaitCancelledError& e) {
            cancelled = true;
            what = e.what();
        }
    });
    REQUIRE(wait_until([&] { return sim->moving() && sim->position_mm() > 1.0; }));
    const double t0 = now_s();
    ax.cancel_waits();
    mover.join();
    CHECK(cancelled.load());
    CHECK(has(what, "decel stop sent"));
    CHECK(now_s() - t0 < 5.0);  // a slice + the decel stop hold; generous for a busy PC
    CHECK(!sim->moving());
    CHECK(sim->position_mm() < 90.0);
    CHECK(!sim->stop_held());  // the STOP was released after the hold
    // calls started afterwards are not affected
    MoveOptions fast;
    fast.speed = 100.0;
    const std::optional<Status> st = ax.move_to(5.0, fast);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 5.0, 0.11);
    ax.cancel_waits();  // nothing waits: no effect
    CHECK(ax.move_to(6.0, fast).has_value());
}

TEST(axis_cancel_waits_ends_a_wait_fed_by_the_status_monitor) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    auto mon = ax.start_monitor();
    std::atomic<bool> cancelled{false};
    ec_test::Thread mover([&] {
        try {
            MoveOptions o = slow(10.0);
            o.completion = Completion::Fast;  // the wait takes the monitor's samples
            ax.move_to(100.0, o);
        } catch (const WaitCancelledError&) {
            cancelled = true;
        }
    });
    REQUIRE(wait_until([&] { return sim->moving() && sim->position_mm() > 1.0; }));
    ax.cancel_waits();
    mover.join();
    CHECK(cancelled.load());
    CHECK(!sim->moving());
    CHECK(mon->running());  // the monitor is not the axis's wait: it polls on
}

TEST(axis_cancel_waits_ends_a_home_wait_and_releases_home) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    so.position_mm = 60.0;  // about 2 s back to the mechanical end at the homing speed
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    std::atomic<bool> cancelled{false};
    ec_test::Thread homer([&] {
        try {
            ax.home();
        } catch (const WaitCancelledError&) {
            cancelled = true;
        }
    });
    REQUIRE(wait_until([&] { return sim->moving() && sim->position_mm() < 59.0; }));
    ax.cancel_waits();
    homer.join();
    CHECK(cancelled.load());
    CHECK(!sim->moving());
    CHECK(!sim->coil_state(reg::kCoilHome));  // the HOME coil was released after the cancel
}

// ============================================================================================ clean shutdown

TEST(axis_closing_the_bus_stops_its_monitors_first_without_a_link_event) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto line = SimLine::with_slaves({1, 2}, so);
    auto bus = std::make_shared<ModbusRtu>(line);
    Axis a1(bus, 1);
    Axis a2(bus, 2);
    std::mutex events_mutex;
    std::vector<std::string> events;
    MonitorOptions mo;
    mo.gap_s = 0.0;
    mo.on_event.push_back([&](const LinkEvent& e) {
        std::lock_guard<std::mutex> lock(events_mutex);
        events.push_back(e.kind);
    });
    auto m1 = a1.start_monitor(mo);
    auto m2 = a2.start_monitor(mo);
    REQUIRE(wait_until([&] { return m1->polls() >= 5 && m2->polls() >= 5; }));
    const double t0 = now_s();
    bus->close();  // e.g. Port::close / Cell::close
    CHECK(now_s() - t0 < 2.0);
    CHECK(!m1->running());
    CHECK(!m2->running());
    CHECK(m1->error() == nullptr);  // stopped, not failed
    std::lock_guard<std::mutex> lock(events_mutex);
    CHECK(events.empty());
}

TEST(axis_monitor_stop_ends_a_poll_waiting_for_the_bus_at_once) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    MonitorOptions mo;
    mo.gap_s = 0.0;
    auto mon = ax.start_monitor(mo);
    REQUIRE(wait_until([&] { return mon->polls() >= 3; }));
    BusLock& lock = ax.bus()->bus_lock();
    lock.lock();  // a long hold (a baud detection, a recovery) keeps the monitor's poll waiting
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    const double t0 = now_s();
    const bool ended = mon->stop(2.0);
    const double took = now_s() - t0;
    lock.unlock();
    CHECK(ended);
    CHECK(took < 1.0);  // a slice (20 ms) in practice, with the bus still held
    CHECK(!mon->running());
    CHECK(mon->error() == nullptr);
}

TEST(axis_link_monitor_stop_ends_a_pass_waiting_for_the_bus_at_once) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    recovery::LinkMonitor::Options lo;
    lo.interval_s = 0.005;
    recovery::LinkMonitor links(ax, lo);
    links.start();
    REQUIRE(wait_until([&] { return links.passes() >= 2; }));
    BusLock& lock = ax.bus()->bus_lock();
    lock.lock();  // the next pass's status read waits for the bus
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    std::atomic<bool> stopped{false};
    const double t0 = now_s();
    ec_test::Thread stopper([&] {  // in a thread: a stop that waited for the bus would never end while it is held here
        links.stop();
        stopped = true;
    });
    const bool ended = wait_until([&] { return stopped.load(); }, 2.0);
    const double took = now_s() - t0;
    lock.unlock();
    stopper.join();
    CHECK(ended);
    CHECK(took < 1.0);  // a slice (20 ms) in practice, with the bus still held
    CHECK(!links.running());
    CHECK(links.errors().empty());  // stopped, not a failed check
    const long before = links.passes();
    links.start();  // started again: its passes are not cancelled
    CHECK(wait_until([&] { return links.passes() > before + 1; }));
    links.stop();
    CHECK(links.errors().empty());
}

TEST(axis_cell_close_stops_and_joins_every_monitor_promptly) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto line = SimLine::with_slaves({1, 2}, so);
    std::map<std::string, std::shared_ptr<Transport>> transports{{"SIM1", line}};
    Cell cell = Cell::open(transports, {1, 2});
    MonitorOptions mo;
    mo.gap_s = 0.0;  // flat out
    const auto monitors = cell.start_monitors({}, mo);
    REQUIRE(monitors.size() == 2u);
    const auto links = cell.monitor_links(0.005);
    REQUIRE(wait_until([&] {
        for (const auto& kv : monitors)
            if (kv.second->polls() < 5) return false;
        return links->passes() >= 2;
    }));
    const double t0 = now_s();
    cell.close();
    CHECK(now_s() - t0 < 3.0);  // a few short polls in practice
    for (const auto& kv : monitors) {
        CHECK(!kv.second->running());
        CHECK(kv.second->error() == nullptr);  // stopped, not failed
    }
    CHECK(!links->running());
    const long polls = monitors.begin()->second->polls();
    sleep_s(0.05);
    CHECK_EQ(monitors.begin()->second->polls(), polls);  // joined: nothing polls any more
}

TEST(axis_cancel_waits_ends_a_slow_polling_wait_within_a_slice) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    OpenOptions oo;
    oo.axis.poll_interval_s = 2.0;  // the wait sleeps 2 s between two status reads
    Axis ax = Axis::open(sim, oo);
    std::atomic<bool> cancelled{false};
    ec_test::Thread mover([&] {
        try {
            ax.move_to(100.0, slow(10.0));
        } catch (const WaitCancelledError&) {
            cancelled = true;
        }
    });
    REQUIRE(wait_until([&] { return sim->moving() && sim->position_mm() > 1.0; }));
    sleep_s(0.1);  // the wait is in its 2 s sleep now
    const double t0 = now_s();
    ax.cancel_waits();
    const bool stopped = wait_until([&] { return !sim->moving(); }, 1.5);
    const double took = now_s() - t0;
    mover.join();  // the decel stop's hold polls at 2 s (not cancellable): a few seconds
    CHECK(cancelled.load());
    CHECK(stopped);
    CHECK(took < 1.0);  // the decel stop went out at once, not after the 2 s sleep
}

// ============================================================================================ home with sparse polls

TEST(axis_home_all_completes_when_the_polls_miss_the_whole_home_return) {
    // Every request moves the simulated clock 1 s: a whole home return (about 0.5 s from 0, 1.5 s from 30 mm) fits
    // between the HOME command and the next status read of its axis, as when several axes are polled in turn.
    SteppedClock clock(1.0);
    SimulatedUnitOptions o1;
    o1.slave = 1;
    o1.clock = clock.clock();
    o1.position_mm = 0.0;  // homed at the home position: nothing in the status changes
    SimulatedUnitOptions o2 = o1;
    o2.slave = 2;
    o2.position_mm = 30.0;  // homed at 30 mm: the position changes
    auto line = std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{std::make_shared<SimulatedUnit>(o1), std::make_shared<SimulatedUnit>(o2)},
        "SIM1");
    std::map<std::string, std::shared_ptr<Transport>> transports{
        {"SIM1", std::make_shared<Wrapped>(line, clock.stepper())}};
    Cell cell = Cell::open(transports, {1, 2});
    Warnings warnings;
    const CellResults<Status> homed = cell.home_all();
    CHECK(homed.succeeded());
    CHECK(homed.at("SIM1:1").homed);
    CHECK_NEAR(homed.at("SIM1:2").position_mm, 0.0, 0.02);
    CHECK(warnings.any("home return not seen between the polls"));  // the axis at 0 (taken as done, logged)
    CHECK(line->unit(1)->homed());
    CHECK(line->unit(2)->homed());
    cell.close();
}

TEST(axis_home_after_lost_home_completes_when_the_polls_miss_the_start) {
    SteppedClock clock(1.0);
    SimulatedUnitOptions so;
    so.clock = clock.clock();
    so.position_mm = 0.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(std::make_shared<Wrapped>(sim, clock.stepper()));
    sim->power_cycle(true);  // HEND lost; the next home return starts and ends between two polls
    ax.servo(true);
    const double t0 = now_s();
    const std::optional<Status> st = ax.home();
    REQUIRE(st.has_value());
    CHECK(st->homed);
    CHECK(now_s() - t0 < kHomeStartS);  // HEND rising is the evidence: no wait for the start window
}
