// Axis unit: the fast path on the simulator (no serial port) - the 1 ms timer resolution, the lean status read, the
// status monitor (samples, waiters, callbacks, transient and fatal errors, restart, suspend, shared port, link watch),
// Completion::Fast (sooner than safe, never before the band: early PEND and stale PEND), the pre-move status taken
// from a fresh monitor sample, MoveHandle, enable_fast / disable_fast and Axis::open with fast. Port of the Python
// edition's tests/test_fast.py and the fast parts of tests/test_fast_recovery.py (VERIFIED in the Python edition where
// its hardware notes say so; this edition is UNVERIFIED on hardware).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint16_t kDsseDst1 = 1u << 4;  ///< seen set while moving (the Python StalePendSim clears it with MOVE)

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

bool wait_until(const std::function<bool()>& predicate, double timeout_s = 2.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_s(0.002);
    }
    return predicate();
}

int word_at(const Bytes& f, int offset) { return (f[offset] << 8) | f[offset + 1]; }

bool is_status_read(const Bytes& f, int words = 0) {
    if (f.size() < 6 || f[1] != 0x03 || word_at(f, 2) != reg::kMonitorBase) return false;
    return words == 0 || word_at(f, 4) == words;
}

bool is_move_write(const Bytes& f) { return f.size() > 4 && f[1] == 0x10 && word_at(f, 2) == reg::kDirectMoveBase; }

bool is_stop_on(const Bytes& f) {
    return f.size() > 4 && f[1] == 0x05 && word_at(f, 2) == reg::kCoilStop && f[4] == 0xFF;
}

/// Frame logger noting the sending thread and the time of every frame.
struct Recorder {
    struct Entry {
        FrameDirection dir;
        Bytes frame;
        std::thread::id thread;
        double t;
    };
    mutable std::mutex mutex;
    std::vector<Entry> entries;

    FrameLogger logger() {
        return [this](FrameDirection d, const Bytes& f) {
            std::lock_guard<std::mutex> lock(mutex);
            entries.push_back({d, f, std::this_thread::get_id(), now_s()});
        };
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        entries.clear();
    }
    std::vector<Entry> all() const {
        std::lock_guard<std::mutex> lock(mutex);
        return entries;
    }
    std::vector<Bytes> tx() const {
        std::vector<Bytes> out;
        for (const auto& e : all())
            if (e.dir == FrameDirection::Tx) out.push_back(e.frame);
        return out;
    }
    std::vector<Bytes> tx_of(std::thread::id thread) const {
        std::vector<Bytes> out;
        for (const auto& e : all())
            if (e.dir == FrameDirection::Tx && e.thread == thread) out.push_back(e.frame);
        return out;
    }
    /// The statuses of every answered 8-word read (any thread), in order.
    std::vector<Status> lean_statuses() const {
        std::vector<Status> out;
        for (const auto& e : all()) {
            const Bytes& f = e.frame;
            if (e.dir != FrameDirection::Rx || f.size() != 3 + 16 + 2 || f[1] != 0x03 || f[2] != 16) continue;
            std::vector<std::uint16_t> words;
            for (int i = 0; i < 8; ++i) words.push_back(static_cast<std::uint16_t>(word_at(f, 3 + 2 * i)));
            out.push_back(Status::from_registers(words));
        }
        return out;
    }
};

bool same_words(const Status& a, const Status& b) {
    return a.raw.pnow == b.raw.pnow && a.raw.almc == b.raw.almc && a.raw.dss1 == b.raw.dss1 &&
           a.raw.dss2 == b.raw.dss2 && a.raw.dsse == b.raw.dsse;
}

/// A SimulatedUnit on a clock that advances step_s on every request (each poll sees the axis exactly step_s later,
/// whatever the host's speed). With stale_polls, after each 0x9900 move command the next stale_polls status reads
/// still show PEND with the MOVE bit clear at the old position (the unit has not taken the command yet): the worst case
/// for a one-sample rule. Port of the Python tests' SteppedClockSim / StalePendSim.
class SteppedSim : public Transport {
public:
    SteppedSim(SimulatedUnitOptions options, double step_s, int stale_polls = 0)
        : step_(step_s), stale_polls_(stale_polls) {
        auto clock = clock_;
        auto m = clock_mutex_;
        options.clock = [clock, m] {
            std::lock_guard<std::mutex> lock(*m);
            return *clock;
        };
        unit_ = std::make_shared<SimulatedUnit>(options);
    }
    SimulatedUnit& unit() { return *unit_; }
    int stale_served() const { return stale_served_.load(); }

    void write(const std::uint8_t* data, std::size_t size) override {
        {
            std::lock_guard<std::mutex> lock(*clock_mutex_);
            *clock_ += step_;
        }
        const Bytes request(data, data + size);
        bool stale = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stale = size > 4 && is_status_read(request) && stale_left_ > 0;
            if (stale) {
                --stale_left_;
                ++stale_served_;
            }
        }
        Bytes response = unit_->handle_frame(request);
        if (stale && response.size() >= 3 + 16 + 2 && response[1] == 0x03) {
            const int dss1 = word_at(response, 3 + 2 * 5) | reg::kDss1Pend;
            const int dsse = word_at(response, 3 + 2 * 7) & ~(reg::kDsseMove | kDsseDst1);
            response[3 + 10] = static_cast<std::uint8_t>(dss1 >> 8);
            response[3 + 11] = static_cast<std::uint8_t>(dss1);
            response[3 + 14] = static_cast<std::uint8_t>(dsse >> 8);
            response[3 + 15] = static_cast<std::uint8_t>(dsse);
            response.resize(response.size() - 2);
            append_crc(response);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_move_write(request)) stale_left_ = stale_polls_;
        pending_.insert(pending_.end(), response.begin(), response.end());
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::size_t n = std::min(size, pending_.size());
        std::copy(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
        pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n));
        return n;
    }
    void reset_input() override {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.clear();
    }
    int baud_rate() const override { return 0; }
    void set_baud_rate(int) override { throw std::logic_error("SteppedSim has no baud rate"); }
    double read_timeout() const override { return unit_->read_timeout(); }
    void set_read_timeout(double seconds) override { unit_->set_read_timeout(seconds); }
    void send_break(double seconds) override { unit_->send_break(seconds); }
    void close() override { unit_->close(); }
    bool is_open() const override { return unit_->is_open(); }
    std::string name() const override { return "SIM"; }

private:
    std::shared_ptr<double> clock_ = std::make_shared<double>(1000.0);
    std::shared_ptr<std::mutex> clock_mutex_ = std::make_shared<std::mutex>();
    double step_;
    int stale_polls_;
    std::shared_ptr<SimulatedUnit> unit_;
    mutable std::mutex mutex_;
    int stale_left_ = 0;
    std::atomic<int> stale_served_{0};
    Bytes pending_;
};

/// An axis on a simulator with a frame recorder (the Python make_ec / open_sim).
struct Rig {
    std::shared_ptr<Recorder> rec;
    Axis ax;
};

/// The Python make_ec: a slider at time_scale, stroke 0..150, 100 mm/s, 0.3 G, polls 5 ms; the axis owns the bus.
struct EcRig {
    std::shared_ptr<SimulatedUnit> sim;
    std::shared_ptr<Recorder> rec;
    Axis ax;
};

EcRig make_ec(double time_scale = 50.0) {
    SimulatedUnitOptions so;
    so.time_scale = time_scale;
    auto sim = std::make_shared<SimulatedUnit>(so);
    auto rec = std::make_shared<Recorder>();
    OpenOptions o;
    o.bus.frame_logger = rec->logger();
    o.axis.stroke = Span{0.0, 150.0};
    o.axis.max_speed_mm_s = 100.0;
    o.axis.max_accel_g = 0.3;
    o.axis.poll_interval_s = 0.005;
    return EcRig{sim, rec, Axis::open(sim, o)};
}

/// The Python open_sim: no bus gap, polls 2 ms, the axis owns the bus.
Rig open_rig(const std::shared_ptr<Transport>& transport, Completion completion = Completion::Safe,
             double min_gap_s = 0.0, double poll_s = 0.002) {
    auto rec = std::make_shared<Recorder>();
    OpenOptions o;
    o.bus.frame_logger = rec->logger();
    o.bus.min_gap_s = min_gap_s;
    o.axis.poll_interval_s = poll_s;
    o.axis.completion = completion;
    return Rig{rec, Axis::open(transport, o)};
}

MoveOptions mv(double speed, double accel = 0.1, std::optional<Completion> completion = std::nullopt,
               double band = 0.1) {
    MoveOptions o;
    o.speed = speed;
    o.accel_g = accel;
    o.band = band;
    o.completion = completion;
    return o;
}

/// A fake winmm recording its calls.
struct FakeWinmm {
    std::mutex mutex;
    std::vector<std::pair<std::string, unsigned>> calls;
    unsigned result = 0;

    TimerApi api() {
        TimerApi a;
        a.begin = [this](unsigned ms) {
            std::lock_guard<std::mutex> lock(mutex);
            calls.emplace_back("begin", ms);
            return result;
        };
        a.end = [this](unsigned ms) {
            std::lock_guard<std::mutex> lock(mutex);
            calls.emplace_back("end", ms);
            return 0u;
        };
        return a;
    }
    int count(const std::string& what) {
        std::lock_guard<std::mutex> lock(mutex);
        return static_cast<int>(std::count_if(calls.begin(), calls.end(), [&](const auto& c) { return c.first == what; }));
    }
    std::vector<std::pair<std::string, unsigned>> snapshot() {
        std::lock_guard<std::mutex> lock(mutex);
        return calls;
    }
};

/// Installs a fake winmm on the process timer for the scope (the system one afterwards).
struct ProcessTimerFake {
    explicit ProcessTimerFake(FakeWinmm& fake) { process_timer().set_api(fake.api()); }
    ~ProcessTimerFake() {
        wait_until([] { return process_timer().users() == 0; }, 2.0);
        try {
            process_timer().set_api(system_timer_api());
        } catch (...) {
        }
    }
};

/// Records link events.
struct Events {
    std::mutex mutex;
    std::vector<LinkEvent> items;
    LinkEventCallback callback() {
        return [this](const LinkEvent& e) {
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
    std::vector<LinkEvent> all() {
        std::lock_guard<std::mutex> lock(mutex);
        return items;
    }
};

/// Installs a monitor link watch for the scope (the previous one afterwards).
struct WatchScope {
    explicit WatchScope(MonitorLinkWatch watch) : previous(set_monitor_link_watch(std::move(watch))) {}
    ~WatchScope() { set_monitor_link_watch(previous); }
    MonitorLinkWatch previous;
};

}  // namespace

// ============================================================================================ 1 ms timer resolution

TEST(axis_fast_timer_reference_counted_begin_and_end) {
    FakeWinmm api;
    TimerResolution timer(api.api());
    {
        ScopedTimerResolution outer(timer);
        CHECK(outer.active() && timer.active());
        {
            ScopedTimerResolution inner(timer);
            CHECK(inner.active());
        }
        CHECK(timer.active());  // the inner scope must not end the period of the outer one
    }
    CHECK(!timer.active());
    const auto calls = api.snapshot();
    REQUIRE(calls.size() == 2);
    CHECK(calls[0] == std::make_pair(std::string("begin"), 1u));
    CHECK(calls[1] == std::make_pair(std::string("end"), 1u));
    CHECK_EQ(timer.users(), 0);
}

TEST(axis_fast_timer_refused_begin_is_not_ended) {
    FakeWinmm api;
    api.result = 97;  // TIMERR_NOCANDO
    TimerResolution timer(api.api());
    {
        ScopedTimerResolution scope(timer);
        CHECK(!scope.active());
    }
    const auto calls = api.snapshot();
    REQUIRE(calls.size() == 1);
    CHECK_EQ(calls[0].first, std::string("begin"));
}

TEST(axis_fast_timer_no_op_without_winmm) {
    TimerResolution timer{TimerApi{}};  // what system_timer_api() gives off Windows
    {
        ScopedTimerResolution scope(timer);
        CHECK(!scope.active());
        CHECK_EQ(timer.users(), 1);
    }
    CHECK_EQ(timer.users(), 0);
#ifdef _WIN32
    CHECK(static_cast<bool>(system_timer_api().begin));
#else
    CHECK(!system_timer_api().begin);
#endif
    TimerResolution busy{TimerApi{}};
    busy.acquire();
    CHECK_THROWS(busy.set_api(TimerApi{}), std::logic_error);
    busy.release();
}

TEST(axis_fast_timer_waits_raise_the_resolution) {
    FakeWinmm api;
    {
        ProcessTimerFake fake(api);
        EcRig r = make_ec();
        r.ax.move_to(90.0, mv(100.0, 0.3));
        r.ax.move_to(80.0, mv(100.0, 0.3, Completion::Fast));
        r.ax.wait_idle(80.0, 0.1, 5.0, Completion::Fast);
    }
    CHECK(api.count("begin") >= 3);
    CHECK_EQ(api.count("begin"), api.count("end"));
    CHECK_EQ(process_timer().users(), 0);
}

TEST(axis_fast_timer_monitor_thread_holds_it_while_running) {
    FakeWinmm api;
    {
        ProcessTimerFake fake(api);
        EcRig r = make_ec();
        auto mon = r.ax.start_monitor();
        CHECK(wait_until([] { return process_timer().active(); }));
        r.ax.stop_monitor();
        CHECK(!mon->running());
        CHECK(wait_until([] { return !process_timer().active(); }));
    }
    const auto calls = api.snapshot();
    REQUIRE(calls.size() == 2);
    CHECK_EQ(calls[0].first, std::string("begin"));
    CHECK_EQ(calls[1].first, std::string("end"));
}

// ============================================================================================ the lean status

TEST(axis_fast_lean_status_is_one_read_of_eight_words) {
    EcRig r = make_ec();
    r.rec->clear();
    const Status st = r.ax.read_lean_status();
    const auto tx = r.rec->tx();
    REQUIRE(tx.size() == 1);
    CHECK(tx[0] == build_frame(1, {0x03, 0x90, 0x00, 0x00, 0x08}));
    CHECK_NEAR(st.position_mm, std::round(r.sim->position_mm() * 100.0) / 100.0, 1e-9);
    CHECK(st.servo_on && st.homed && st.in_position && st.modbus_enabled);
    CHECK(!st.auto_servo_off().has_value());  // STAT is not in the lean block
    CHECK_EQ(st.raw.words, 8);
    CHECK(!st.raw.stat.has_value());
}

// ============================================================================================ the status monitor

TEST(axis_fast_monitor_publishes_lean_samples_and_stops_cleanly) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    CHECK(r.ax.monitor() == mon);
    CHECK(r.ax.start_monitor() == mon);  // a second start returns the running monitor
    REQUIRE(wait_until([&] { return mon->polls() >= 20; }));
    const auto first = mon->latest();
    REQUIRE(first.has_value());
    CHECK(wait_until([&] { return mon->latest()->seq > first->seq; }));
    const MonitorSample sample = *mon->latest();
    CHECK(sample.t_start <= sample.t);
    CHECK(*mon->timestamp() >= sample.t);
    CHECK(mon->status().has_value());
    CHECK(mon->fresh().has_value());
    CHECK(mon->poll_rate_hz().value_or(0.0) > 0.0);
    int reads = 0, lean = 0;
    for (const auto& f : r.rec->tx())
        if (is_status_read(f)) {
            ++reads;
            if (is_status_read(f, kLeanStatusWords)) ++lean;
        }
    CHECK(reads > 0);
    CHECK_EQ(lean, reads);  // the monitor reads 8 words
    CHECK(mon->stop());
    CHECK(!mon->running());
    const long polls = mon->polls();
    sleep_s(0.05);
    CHECK_EQ(mon->polls(), polls);   // no poll after stop()
    CHECK(!mon->fresh().has_value());  // a stopped monitor has no fresh status
    CHECK(!mon->wait_update(std::nullopt, std::nullopt, 0.01).has_value());
    CHECK(mon->to_string().find("stopped") != std::string::npos);
}

TEST(axis_fast_monitor_bus_is_free_between_polls) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 5; }));
    double worst = 0.0;
    for (int i = 0; i < 50; ++i) {
        const double t0 = now_s();
        r.ax.read_registers(reg::kMonitorBase, 1);  // what another thread / axis needs to send
        worst = std::max(worst, now_s() - t0);
        sleep_s(0.001);
    }
    CHECK(worst < 0.05);
    const long before = mon->polls();
    CHECK(wait_until([&] { return mon->polls() > before + 5; }));  // the monitor kept polling
}

TEST(axis_fast_monitor_waiters_wake_on_the_matching_sample) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    r.ax.start_move_to(30.0, mv(100.0, 0.3));
    const auto st = mon->wait_for([](const Status& s) { return s.position_mm < 50.0; }, 5.0);
    REQUIRE(st.has_value());
    CHECK(st->position_mm < 50.0);
    const auto done = mon->wait_done(5.0);
    REQUIRE(done.has_value());
    CHECK(fast_done(*done, 30.0, 0.1));
    CHECK(!mon->wait_for([](const Status&) { return false; }, 0.05).has_value());  // timeout gives nullopt
    // a predicate error reaches the waiting thread
    CHECK_THROWS(mon->wait_for([](const Status&) -> bool { throw std::runtime_error("predicate bug"); }, 1.0),
                 std::runtime_error);
}

TEST(axis_fast_monitor_on_change_and_on_done_callbacks) {
    // a stepped clock: the move's wait is fed by the monitor, so the monitor's polls alone move the unit on and see
    // every step of the move (on the wall clock a busy PC could leave it no sample before the end, and no done edge)
    SimulatedUnitOptions so;
    so.position_mm = 0.0;
    Rig r = open_rig(std::make_shared<SteppedSim>(so, 0.02));
    std::mutex m;
    std::vector<std::pair<Status, std::optional<Status>>> changes;
    std::vector<Status> dones;
    std::atomic<int> warnings{0};
    const LogHandler previous = set_log_handler([&](LogLevel level, const std::string& source, const std::string& msg) {
        if (level == LogLevel::Warning && source == "iai_actuator_driver.monitor" &&
            msg.find("callback") != std::string::npos)
            ++warnings;
    });
    MonitorOptions mo;
    mo.on_change.push_back([&](const Status& st, const std::optional<Status>& prev) {
        std::lock_guard<std::mutex> lock(m);
        changes.emplace_back(st, prev);
    });
    mo.on_change.push_back([](const Status&, const std::optional<Status>&) { throw std::runtime_error("callback bug"); });
    auto mon = r.ax.start_monitor(mo);
    MonitorOptions more;
    more.on_done.push_back([&](const Status& st) {
        std::lock_guard<std::mutex> lock(m);
        dones.push_back(st);
    });
    CHECK(r.ax.start_monitor(more) == mon);  // added to the running monitor
    CHECK(wait_until([&] { return mon->polls() >= 3; }));
    MoveOptions o = mv(100.0, 0.3, Completion::Fast);
    o.timeout_s = 60.0;  // real time: a busy PC slows the polls, not the move they see
    r.ax.move_to(40.0, o);
    CHECK(wait_until(
        [&] {
            std::lock_guard<std::mutex> lock(m);
            return !dones.empty();
        },
        3.0));
    sleep_s(0.05);
    set_log_handler(previous);
    CHECK_EQ(warnings.load(), 1);  // a failing callback is logged once
    CHECK(mon->running());         // and does not stop the monitor
    std::lock_guard<std::mutex> lock(m);
    REQUIRE(dones.size() == 1);
    CHECK(std::fabs(dones[0].position_mm - 40.0) <= 0.1 + 1e-6);
    CHECK(changes.size() > 5);
    REQUIRE(!changes.empty());
    CHECK(!changes[0].second.has_value());
    for (std::size_t i = 1; i < changes.size(); ++i) {
        REQUIRE(changes[i].second.has_value());
        CHECK(!same_words(changes[i].first, *changes[i].second));
    }
}

TEST(axis_fast_monitor_transient_errors_are_retried) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 3; }));
    r.sim->inject_fault(SimFault::Timeout, 3);
    CHECK(wait_until([&] { return mon->error_count() >= 3; }));
    const long before = mon->polls();
    CHECK(wait_until([&] { return mon->polls() > before + 3; }));  // polling resumes after timeouts
    CHECK(mon->running());
    CHECK(!mon->error());
    CHECK(mon->last_error() != nullptr);
}

TEST(axis_fast_monitor_port_vanishing_ends_it_and_wakes_waiters) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 3; }));
    std::thread unplug([&] {
        sleep_s(0.05);
        r.sim->close();  // the USB adapter is unplugged
    });
    const double t0 = now_s();
    CHECK_THROWS(mon->wait_for([](const Status&) { return false; }, 5.0), ModbusError);
    CHECK(now_s() - t0 < 2.0);
    unplug.join();
    CHECK(wait_until([&] { return !mon->running(); }));
    CHECK(mon->error() != nullptr);
    CHECK_THROWS(mon->wait_update(std::nullopt, std::nullopt, 0.01), ModbusError);
    CHECK(mon->to_string().find("failed") != std::string::npos);
}

TEST(axis_fast_monitor_an_unexpected_error_in_its_thread_ends_it_with_the_error) {
    // a failure where the monitor expects none (here: publishing a sample, out of memory) ends the monitor with that
    // error recorded and its waiters woken - never the process; it starts again afterwards
    struct ClearHooks {
        ~ClearHooks() { detail::monitor_test_hooks() = detail::MonitorTestHooks{}; }
    } clear_hooks;  // after the rig (declared below) stopped the monitor
    std::atomic<int> published{0};
    detail::monitor_test_hooks().before_publish = [&](const std::string&) {
        if (++published == 3) throw std::bad_alloc();
    };
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    CHECK_THROWS(mon->wait_for([](const Status&) { return false; }, 5.0), ModbusError);
    CHECK(wait_until([&] { return !mon->running(); }));
    REQUIRE(mon->error() != nullptr);
    CHECK_THROWS(std::rethrow_exception(mon->error()), std::bad_alloc);
    CHECK(mon->to_string().find("failed") != std::string::npos);
    detail::monitor_test_hooks() = detail::MonitorTestHooks{};  // the thread has ended: no reader
    CHECK(r.ax.start_monitor() == mon);
    CHECK(mon->running());
    CHECK(!mon->error());
    const long polls = mon->polls();
    CHECK(wait_until([&] { return mon->polls() > polls; }));
}

TEST(axis_fast_monitor_failed_restarts_with_its_callbacks) {
    EcRig r = make_ec(5.0);  // slow enough to see the move start
    std::atomic<int> seen{0};
    MonitorOptions mo;
    mo.on_done.push_back([&](const Status&) { ++seen; });
    auto mon = r.ax.start_monitor(mo);
    REQUIRE(wait_until([&] { return mon->polls() >= 2; }));
    mon->fail(std::make_exception_ptr(ModbusError("transport error: test")));  // as when the port vanished
    CHECK(wait_until([&] { return !mon->running(); }));
    CHECK(r.ax.start_monitor() == mon);
    CHECK(mon->running());
    CHECK(!mon->error());
    r.ax.move_to(60.0, mv(100.0, 0.3));
    CHECK(wait_until([&] { return seen.load() >= 1; }));
}

TEST(axis_fast_monitor_suspend_pauses_polling) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 3; }));
    long polls;
    {
        AxisMonitor::Suspension held = mon->suspend();
        polls = mon->polls();
        sleep_s(0.08);
        CHECK_EQ(mon->polls(), polls);
        CHECK(!mon->fresh().has_value());  // a suspended monitor goes stale
        AxisMonitor::Suspension nested = mon->suspend();
        nested.release();
        sleep_s(0.02);
        CHECK_EQ(mon->polls(), polls);  // still held by the outer one
    }
    CHECK(wait_until([&] { return mon->polls() > polls; }));
}

TEST(axis_fast_monitor_close_stops_it) {
    EcRig r = make_ec();
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 1; }));
    r.ax.close();
    CHECK(!mon->running());
    CHECK(!r.ax.monitor());
    CHECK(!mon->error());  // a clean stop is not an error
    CHECK_THROWS(mon->start(), std::logic_error);  // detached from the closed axis
}

TEST(axis_fast_monitor_bad_arguments) {
    EcRig r = make_ec();
    const auto bad = [&](std::function<void(MonitorOptions&)> edit) {
        MonitorOptions o;
        edit(o);
        CHECK_THROWS(AxisMonitor::create(r.ax, o), std::invalid_argument);
    };
    bad([](MonitorOptions& o) { o.gap_s = -1.0; });
    bad([](MonitorOptions& o) { o.gap_s = std::nan(""); });
    bad([](MonitorOptions& o) { o.gap_s = 11.0; });
    bad([](MonitorOptions& o) { o.retries = -1; });
    // the link check interval (test_fast_recovery: test_link_check_interval_is_validated)
    bad([](MonitorOptions& o) { o.link_check_s = -1.0; });
    bad([](MonitorOptions& o) { o.link_check_s = 4000.0; });
    MonitorOptions none;
    none.link_check_s = std::nullopt;
    auto mon = r.ax.start_monitor(none);
    CHECK(!mon->link_check_s().has_value());
    MonitorOptions zero;
    zero.link_check_s = 0.0;
    CHECK(!AxisMonitor::create(r.ax, zero)->link_check_s().has_value());
    CHECK_EQ(mon->name(), std::string("ec-monitor-1"));
}

TEST(axis_fast_monitor_two_axes_on_one_port) {
    SimulatedUnitOptions so;
    so.time_scale = 5.0;
    auto line = SimLine::with_slaves({1, 2}, so, "L");
    auto bus = std::make_shared<ModbusRtu>(line);
    AxisOptions ao;
    ao.poll_interval_s = 0.005;
    ao.completion = Completion::Fast;
    std::vector<std::unique_ptr<Axis>> axes;
    axes.push_back(std::make_unique<Axis>(bus, 1, ao));
    axes.push_back(std::make_unique<Axis>(bus, 2, ao));
    const std::map<int, double> targets{{1, 20.0}, {2, 130.0}};
    std::vector<std::shared_ptr<AxisMonitor>> monitors;
    for (auto& a : axes) monitors.push_back(a->start_monitor());
    REQUIRE(wait_until([&] { return monitors[0]->polls() >= 3 && monitors[1]->polls() >= 3; }));
    line->clear_requests();
    std::map<int, std::optional<Status>> results;
    std::map<int, std::string> errors;
    std::mutex m;
    std::vector<std::thread> threads;
    for (auto& a : axes) {
        Axis* ax = a.get();
        threads.emplace_back([&, ax] {
            try {
                const auto st = ax->move_to(targets.at(ax->slave()), mv(100.0, 0.3));
                std::lock_guard<std::mutex> lock(m);
                results[ax->slave()] = st;
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(m);
                errors[ax->slave()] = e.what();
            }
        });
    }
    for (auto& t : threads) t.join();
    CHECK(errors.empty());
    for (const auto& t : targets) {
        REQUIRE(results[t.first].has_value());
        CHECK(std::fabs(results[t.first]->position_mm - t.second) <= 0.1 + 1e-6);
        CHECK(std::fabs(line->unit(t.first)->position_mm() - t.second) <= 0.1 + 1e-6);
    }
    std::vector<int> slaves;
    for (int s : line->request_slaves())
        if (s == 1 || s == 2) slaves.push_back(s);
    int switches = 0;
    for (std::size_t i = 1; i < slaves.size(); ++i)
        if (slaves[i] != slaves[i - 1]) ++switches;
    CHECK(switches > 10);  // the frames interleaved
    for (auto& mon : monitors) CHECK(mon->running() && !mon->error());
    {  // suspend_bus holds every monitor of the line
        BusSuspension held = suspend_bus(*bus);
        CHECK_EQ(held.monitors().size(), 2u);
        const long p1 = monitors[0]->polls(), p2 = monitors[1]->polls();
        sleep_s(0.06);
        CHECK_EQ(monitors[0]->polls(), p1);
        CHECK_EQ(monitors[1]->polls(), p2);
    }
    const long p1 = monitors[0]->polls();
    CHECK(wait_until([&] { return monitors[0]->polls() > p1; }));
    for (auto& a : axes) a->stop_monitor();
    axes.clear();
    bus->close();
}

// ============================================================================================ Completion::Fast
//
// The unit runs on its stepped clock (SteppedSim) and the driver on a virtual one (VirtualClockScope): the move's
// timeout, the settle hold and the settled-away rule count the polls' own sleeps, not the real time a busy PC takes for
// the polls (a wait fed by the monitor sleeps on nothing, so for it they never run out).

TEST(axis_fast_completion_returns_sooner_than_safe) {
    std::map<Completion, int> counts;
    for (Completion mode : {Completion::Safe, Completion::Fast}) {
        VirtualClockScope clock;
        SimulatedUnitOptions so;
        so.profile = SimProfile::Gripper;
        so.position_mm = 3.0;
        auto sim = std::make_shared<SteppedSim>(so, 0.01);
        Rig r = open_rig(sim);
        r.ax.move_to(3.0, mv(10.0));  // limits read, axis at rest
        r.rec->clear();
        const auto st = r.ax.move_to(8.0, mv(10.0, 0.1, mode));
        REQUIRE(st.has_value());
        CHECK(std::fabs(st->position_mm - 8.0) <= 0.1 + 1e-6);
        const auto tx = r.rec->tx();
        const auto first_move = std::find_if(tx.begin(), tx.end(), is_move_write);
        REQUIRE(first_move != tx.end());
        int reads = 0, lean = 0;
        for (auto it = first_move + 1; it != tx.end(); ++it)
            if (is_status_read(*it)) {
                ++reads;
                if (is_status_read(*it, kLeanStatusWords)) ++lean;
            }
        counts[mode] = reads;
        if (mode == Completion::Fast) {
            CHECK_EQ(lean, reads);
            CHECK(st->in_position && !st->move_bit());
        }
    }
    CHECK(counts[Completion::Fast] < counts[Completion::Safe]);
}

TEST(axis_fast_completion_never_before_the_band_early_pend) {
    // gripper: PEND rises inside the band while the axis still moves (VERIFIED on the EC-GRBP8M in the Python
    // edition); fast accepts the first sample inside the band with PEND and MOVE clear, never an earlier one
    VirtualClockScope clock;
    SimulatedUnitOptions so;
    so.profile = SimProfile::Gripper;
    so.position_mm = 3.0;
    auto sim = std::make_shared<SteppedSim>(so, 0.01);
    Rig r = open_rig(sim, Completion::Fast);
    r.ax.move_to(3.0, mv(10.0));
    r.rec->clear();
    const auto st = r.ax.move_to(8.0, mv(10.0, 0.1, std::nullopt, 0.1));
    REQUIRE(st.has_value());
    const auto seen = r.rec->lean_statuses();
    REQUIRE(!seen.empty());
    CHECK(same_words(seen.back(), *st));
    CHECK(std::fabs(st->position_mm - 8.0) <= 0.1 + 1e-6);
    bool any_in_position = false;
    for (std::size_t i = 0; i + 1 < seen.size(); ++i) {
        CHECK(!fast_done(seen[i], 8.0, 0.1));
        any_in_position = any_in_position || seen[i].in_position;
    }
    CHECK(any_in_position || st->position_mm < 8.0 || !sim->unit().moving());
}

TEST(axis_fast_completion_never_before_the_band_stale_pend) {
    // a stale PEND with MOVE clear right after the command (old position) never ends a fast wait; on the virtual clock
    // the six stale polls cannot outlast the settled-away rule however slowly a busy PC sends them
    for (bool use_monitor : {false, true}) {
        VirtualClockScope clock;
        SimulatedUnitOptions so;
        so.position_mm = 20.0;
        auto sim = std::make_shared<SteppedSim>(so, 0.005, 6);
        Rig r = open_rig(sim, Completion::Fast);
        r.ax.move_to(20.0, mv(100.0, 0.3));
        if (use_monitor) r.ax.start_monitor();
        for (double target : {60.0, 25.0, 25.5}) {
            const int served = sim->stale_served();
            const auto st = r.ax.move_to(target, mv(100.0, 0.3, std::nullopt, 0.1));
            REQUIRE(st.has_value());
            CHECK(std::fabs(st->position_mm - target) <= 0.1 + 1e-6);
            CHECK(std::fabs(sim->unit().position_mm() - target) <= 0.1 + 1e-6);
            // the stale answers were read, counted over both pollers (the wait's own polls, or the monitor's that feed
            // the wait): all six on the long moves; the 0.5 mm move reaches its band while they last, and a stale
            // answer inside the band may end it
            CHECK(sim->stale_served() - served >= (target == 25.5 ? 1 : 6));
        }
    }
}

TEST(axis_fast_completion_per_axis_and_per_call) {
    VirtualClockScope clock;
    SimulatedUnitOptions so;
    so.profile = SimProfile::Gripper;
    so.position_mm = 3.0;
    auto sim = std::make_shared<SteppedSim>(so, 0.01);
    Rig r = open_rig(sim, Completion::Fast);
    r.ax.move_to(3.0, mv(10.0));
    r.rec->clear();
    const auto st = r.ax.move_to(8.0, mv(10.0, 0.1, Completion::Safe));  // per call beats the axis
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 8.0, 1e-9);
    CHECK(!sim->unit().moving());
    bool full = false;
    for (const auto& f : r.rec->tx()) full = full || is_status_read(f, reg::kStatusReadWords);
    CHECK(full);
    AxisOptions o = r.ax.options();
    o.completion = Completion::Safe;
    r.ax.set_options(o);
    r.rec->clear();
    r.ax.move_to(3.0, mv(10.0, 0.1, Completion::Fast));
    const auto tx = r.rec->tx();
    const auto first_move = std::find_if(tx.begin(), tx.end(), is_move_write);
    REQUIRE(first_move != tx.end());
    int reads = 0, lean = 0;
    for (auto it = first_move + 1; it != tx.end(); ++it)
        if (is_status_read(*it)) {
            ++reads;
            if (is_status_read(*it, kLeanStatusWords)) ++lean;
        }
    CHECK(reads > 0);
    CHECK_EQ(lean, reads);
    const Status done = r.ax.wait_idle(3.0, 0.1, 5.0, Completion::Fast);
    CHECK(fast_done(done, 3.0, 0.1));
}

TEST(axis_fast_completion_names) {
    CHECK(parse_completion("safe") == Completion::Safe);
    CHECK(parse_completion("fast") == Completion::Fast);
    CHECK_EQ(std::string(to_string(Completion::Fast)), std::string("fast"));
    CHECK_THROWS(parse_completion("quick"), std::invalid_argument);
    try {
        parse_completion("quick");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("completion must be one of") != std::string::npos);
    }
    Status st;
    CHECK(!fast_done(st));  // no PEND
    st.in_position = true;
    CHECK(fast_done(st));   // no target: no position check
    st.position_mm = 10.05;
    CHECK(fast_done(st, 10.0));
    CHECK(!fast_done(st, 10.0, 0.04));
    st.raw.dsse = reg::kDsseMove;
    CHECK(!fast_done(st, 10.0));
}

TEST(axis_fast_completion_alarm_during_a_fast_wait_stops_and_raises) {
    EcRig r = make_ec(2.0);
    r.ax.start_monitor();
    MoveHandle handle = r.ax.start_move_to(140.0, mv(20.0, 0.1, Completion::Fast));
    sleep_s(0.05);
    r.sim->trigger_alarm(0x0C0);
    CHECK_THROWS(handle.wait(), EcAlarmError);
    bool stop = false;
    for (const auto& f : r.rec->tx()) stop = stop || is_stop_on(f);
    CHECK(stop);
}

// ============================================================================================ pre-move status

TEST(axis_fast_premove_fresh_monitor_status_makes_a_one_frame_move) {
    SimulatedUnitOptions so;
    so.time_scale = 50.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Rig r = open_rig(sim, Completion::Fast, 0.0005);
    r.ax.move_to(60.0, mv(100.0, 0.3));  // limits read once
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->fresh().has_value(); }));
    r.rec->clear();
    const auto me = std::this_thread::get_id();
    MoveHandle handle = r.ax.start_move_to(80.0, mv(100.0, 0.3));
    const auto main = r.rec->tx_of(me);
    REQUIRE(main.size() == 1);
    CHECK(is_move_write(main[0]));
    const Status st = handle.wait();
    CHECK(std::fabs(st.position_mm - 80.0) <= 0.1 + 1e-6);
    bool read = false;
    for (const auto& f : r.rec->tx_of(me)) read = read || is_status_read(f);
    CHECK(!read);  // the wait used the monitor's samples
}

TEST(axis_fast_premove_stale_monitor_status_is_read_again) {
    SimulatedUnitOptions so;
    so.time_scale = 50.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Rig r = open_rig(sim);
    r.ax.move_to(60.0, mv(100.0, 0.3));
    auto mon = r.ax.start_monitor();
    REQUIRE(wait_until([&] { return mon->polls() >= 2; }));
    std::optional<MoveHandle> handle;
    {
        AxisMonitor::Suspension held = mon->suspend();
        sleep_s(0.06);  // older than kFreshS
        r.rec->clear();
        handle = r.ax.start_move_to(70.0, mv(100.0, 0.3));
        const auto main = r.rec->tx_of(std::this_thread::get_id());
        REQUIRE(main.size() >= 2);
        CHECK(is_status_read(main.front(), reg::kStatusReadWords));
        CHECK(is_move_write(main.back()));
    }
    CHECK(std::fabs(handle->wait().position_mm - 70.0) <= 0.1 + 1e-6);
}

TEST(axis_fast_premove_monitor_status_not_ready_is_read_again) {
    SimulatedUnitOptions so;
    so.time_scale = 50.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Rig r = open_rig(sim);
    r.ax.move_to(60.0, mv(100.0, 0.3));
    auto mon = r.ax.start_monitor();
    r.ax.servo(false);
    REQUIRE(wait_until([&] { return mon->status().has_value() && !mon->status()->servo_on; }));
    try {
        r.ax.move_to(70.0, mv(100.0, 0.3));
        CHECK(false);  // refused
    } catch (const EcNotReadyError& e) {
        CHECK(std::string(e.what()).find("servo") != std::string::npos);
    }
    CHECK(!sim->moving());
}

// ============================================================================================ MoveHandle

TEST(axis_fast_handle_wait_done_result) {
    EcRig r = make_ec(20.0);
    MoveHandle handle = r.ax.start_move_to(120.0, mv(100.0, 0.3));
    CHECK(!handle.done());
    CHECK(!handle.result().has_value());
    const Status st = handle.wait();
    REQUIRE(handle.result().has_value());
    CHECK(same_words(*handle.result(), st));
    CHECK(handle.done());
    CHECK(same_words(handle.wait(), st));
    CHECK(std::fabs(st.position_mm - 120.0) <= 0.1);
    const Json j = handle.to_json();
    CHECK(j.dump().find("\"target_mm\":120") != std::string::npos);
    CHECK(j.dump().find("\"done\":true") != std::string::npos);
    CHECK(j.dump().find("\"cancelled\":false") != std::string::npos);
    CHECK(handle.to_string().find("done") != std::string::npos);
    CHECK(handle.completion() == Completion::Safe);
}

TEST(axis_fast_handle_done_polling_fast_with_monitor) {
    EcRig r = make_ec(20.0);
    r.ax.start_monitor();
    MoveHandle handle = r.ax.start_move_to(30.0, mv(100.0, 0.3, Completion::Fast));
    CHECK(wait_until([&] { return handle.done(); }, 10.0));
    REQUIRE(handle.result().has_value());
    CHECK(fast_done(*handle.result(), 30.0, 0.1));
}

TEST(axis_fast_handle_done_polling_safe) {
    EcRig r = make_ec(20.0);
    MoveHandle handle = r.ax.start_move_to(30.0, mv(100.0, 0.3));
    CHECK(wait_until([&] { return handle.done(); }, 10.0));
    REQUIRE(handle.result().has_value());
    CHECK(std::fabs(handle.result()->position_mm - 30.0) <= 0.1);
    CHECK(!r.sim->moving());
}

TEST(axis_fast_handle_cancel_is_stop) {
    EcRig r = make_ec(20.0);
    r.ax.start_monitor();
    MoveHandle handle = r.ax.start_move_to(140.0, mv(20.0));
    sleep_s(0.05);
    r.rec->clear();
    const Status st = handle.cancel();
    std::vector<Bytes> own;
    for (const auto& f : r.rec->tx())
        if (!is_status_read(f, kLeanStatusWords)) own.push_back(f);  // without the monitor's polls
    REQUIRE(!own.empty());
    CHECK(is_stop_on(own[0]));  // STOP is the first frame of cancel()
    CHECK(!r.sim->moving());
    CHECK(handle.cancelled());
    REQUIRE(handle.result().has_value());
    CHECK(same_words(*handle.result(), st));
    CHECK(same_words(handle.wait(), st));
    CHECK(st.position_mm < 139.0);
    CHECK(handle.to_string().find("cancelled") != std::string::npos);
}

TEST(axis_fast_handle_wait_timeout_stops) {
    EcRig r = make_ec(20.0);
    MoveHandle handle = r.ax.start_move_to(140.0, mv(20.0));
    CHECK_THROWS(handle.wait(0.05), MotionTimeoutError);
    CHECK(!r.sim->moving());
}

TEST(axis_fast_handle_relative_moves_and_checks_still_run) {
    EcRig r = make_ec(20.0);
    r.ax.start_move_by(5.0, mv(50.0));
    MoveOptions jog = MoveOptions::jog();
    jog.completion = Completion::Fast;
    MoveHandle handle = r.ax.start_jog(-1.0, jog);
    CHECK(handle.completion() == Completion::Fast);
    handle.wait();
    r.rec->clear();
    CHECK_THROWS(r.ax.start_move_to(150.5), SoftLimitError);
    CHECK_THROWS(r.ax.start_move_to(80.0, mv(500.0)), SoftLimitError);
    bool moved = false;
    for (const auto& f : r.rec->tx()) moved = moved || is_move_write(f);
    CHECK(!moved);
}

TEST(axis_fast_handle_late_wait_on_a_finished_move_returns_its_status) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Gripper;
    so.time_scale = 10.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Rig r = open_rig(sim, Completion::Safe, 0.0, 0.005);
    r.ax.connect();
    const double target = r.ax.status().position_mm < 4.75 ? 9.0 : 0.5;
    MoveHandle handle = r.ax.start_move_to(target, mv(50.0, 0.1, Completion::Safe));
    sleep_s(handle.timeout_s() + 0.1);  // the application looked away past the move's estimate
    r.rec->clear();
    const Status st = handle.wait();
    CHECK(std::fabs(st.position_mm - target) <= 0.1);
    for (const auto& f : r.rec->tx()) CHECK(f[1] != 0x05);  // no STOP sent
}

// ============================================================================================ enable_fast / open fast

TEST(axis_fast_open_fast_raises_baud_and_turns_everything_on) {
    SimulatedUnitOptions so;
    so.baud_rate = 38400;
    so.time_scale = 50.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    OpenOptions o;
    o.fast = true;
    o.axis.poll_interval_s = 0.005;
    Axis ax = Axis::open(sim, o);
    std::shared_ptr<AxisMonitor> mon;
    CHECK(ax.baud_rate() == std::optional<int>(230400));
    CHECK(sim->unit_baud_rate() == std::optional<int>(230400));
    CHECK(ax.options().completion == Completion::Fast);
    CHECK(ax.fast_mode());
    CHECK(ax.fast_notes().empty());
    mon = ax.monitor();
    REQUIRE(mon != nullptr);
    CHECK(mon->running());
    ax.connect(true, true);  // idempotent
    CHECK(ax.monitor() == mon);
    const auto st = ax.move_to(100.0, 100.0, 0.3);
    REQUIRE(st.has_value());
    CHECK(fast_done(*st, 100.0, 0.1));
    ax.close();
    CHECK(!mon->running());
}

TEST(axis_fast_connect_fast_without_a_baud_rate) {
    EcRig r = make_ec();
    r.ax.connect(true, true);
    CHECK(r.ax.options().completion == Completion::Fast);
    REQUIRE(r.ax.monitor() != nullptr);
    CHECK(r.ax.monitor()->running());
    bool noted = false;
    for (const auto& n : r.ax.fast_notes()) noted = noted || n.find("no baud rate") != std::string::npos;
    CHECK(noted);
    CHECK(!r.ax.baud_rate().has_value());
}

TEST(axis_fast_shared_port_keeps_the_rate) {
    SimulatedUnitOptions so;
    so.baud_rate = 38400;
    so.time_scale = 50.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    auto bus = std::make_shared<ModbusRtu>(sim);
    AxisOptions ao;
    ao.poll_interval_s = 0.005;
    {
        Axis ax(bus, 1, ao);
        const FastModeSummary summary = ax.enable_fast(kFastBaudRate, false);
        CHECK(summary.baud_rate == std::optional<int>(38400));
        CHECK(!summary.monitor);
        bool noted = false;
        for (const auto& n : summary.notes) noted = noted || n.find("shared port") != std::string::npos;
        CHECK(noted);
        CHECK(ax.options().completion == Completion::Fast);
        CHECK(summary.to_json().dump().find("\"baudrate\":38400") != std::string::npos);
    }
    bus->close();
}

TEST(axis_fast_disable_fast_undoes_enable_fast) {
    EcRig r = make_ec();
    const FastModeSummary result = r.ax.enable_fast(std::nullopt);
    CHECK(result.monitor && r.ax.fast_mode());
    CHECK(r.ax.options().completion == Completion::Fast);
    auto mon = r.ax.monitor();
    REQUIRE(mon != nullptr);
    FastModeSummary expected;
    expected.baud_rate = r.ax.baud_rate();
    expected.completion = Completion::Safe;
    expected.monitor = false;
    CHECK(r.ax.disable_fast() == expected);
    CHECK(!r.ax.fast_mode());
    CHECK(!r.ax.monitor());
    CHECK(!mon->running());
    CHECK(r.ax.options().completion == Completion::Safe);
}

TEST(axis_fast_stop_stays_immediate_with_the_monitor) {
    SimulatedUnitOptions so;
    so.time_scale = 1.0;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Rig r = open_rig(sim, Completion::Fast);
    MonitorOptions mo;
    mo.gap_s = 0.0;  // worst case: polls back to back
    auto mon = r.ax.start_monitor(mo);
    r.ax.start_move_to(140.0, mv(20.0, 0.3));
    REQUIRE(wait_until([&] { return mon->polls() >= 20; }));
    const double t0 = now_s();
    r.ax.stop();
    std::optional<double> stop_at;
    for (const auto& e : r.rec->all())
        if (!stop_at && e.dir == FrameDirection::Tx && e.t >= t0 && is_stop_on(e.frame)) stop_at = e.t;
    REQUIRE(stop_at.has_value());
    CHECK(*stop_at - t0 < 0.05);
    CHECK(!sim->moving());
    CHECK(mon->running());
}

// ============================================================================================ the link watch

TEST(axis_fast_link_no_check_without_a_session_but_a_lost_link_is_seen) {
    EcRig r = make_ec();
    Events events;
    MonitorOptions mo;
    mo.on_event.push_back(events.callback());
    mo.link_check_s = 0.01;
    auto mon = r.ax.start_monitor(mo);
    REQUIRE(wait_until([&] { return mon->polls() >= 20; }));
    CHECK_EQ(mon->link_checks(), 0);  // no recovery module installed: no session, no check
    CHECK_EQ(mon->link_state(), std::string(kLinkOk));
    r.sim->inject_fault(SimFault::Timeout, 1000);  // the unit goes silent
    CHECK(wait_until([&] { return events.kinds() == std::vector<std::string>{"link_lost"}; }, 5.0));
    const auto items = events.all();
    REQUIRE(!items.empty());
    CHECK_EQ(items[0].cause, std::string(kLinkUnitSilent));
    CHECK_EQ(items[0].axis, std::string("SIM:1"));
    CHECK(items[0].message.find("ModbusTimeout") != std::string::npos);
    CHECK(mon->error_count() >= kSilentAfter);
    CHECK(mon->running());  // a silent unit does not end the monitor
    CHECK_EQ(mon->link_state(), std::string(kLinkUnitSilent));
    sleep_s(0.1);
    CHECK_EQ(events.kinds().size(), 1u);  // once per change
}

TEST(axis_fast_link_unplug_is_port_vanished_and_ends_the_monitor) {
    EcRig r = make_ec();
    Events events;
    MonitorOptions mo;
    mo.on_event.push_back(events.callback());
    auto mon = r.ax.start_monitor(mo);
    REQUIRE(wait_until([&] { return mon->polls() >= 3; }));
    r.sim->close();
    CHECK(wait_until([&] { return !mon->running(); }));
    const auto items = events.all();
    REQUIRE(items.size() == 1);
    CHECK_EQ(items[0].kind, std::string(kEventLinkLost));
    CHECK_EQ(items[0].cause, std::string(kLinkPortVanished));
    CHECK_EQ(mon->link_state(), std::string(kLinkPortVanished));
    CHECK(items[0].to_json().dump().find("\"cause\":\"port_vanished\"") != std::string::npos);
}

TEST(axis_fast_link_watch_hooks_are_used) {
    std::atomic<int> checks{0};
    std::atomic<int> faults{0};
    std::atomic<bool> fail_next{false};
    std::atomic<Axis*> seen{nullptr};
    MonitorLinkWatch watch;
    watch.check_link = [&](Axis& axis, const LinkEventCallback& emit) -> std::optional<MonitorLinkCheck> {
        seen = &axis;
        ++checks;
        MonitorLinkCheck c;
        if (fail_next.load()) {
            c.state = kLinkPortVanished;
            c.error = std::make_exception_ptr(TransportError("port gone (test)"));
            emit(LinkEvent{kEventLinkLost, "test", "port gone (test)", kLinkPortVanished, 0.0});
        }
        return c;
    };
    watch.link_fault = [&](Axis&, const std::exception&, const LinkEventCallback&) { ++faults; };
    watch.link_state = [](Axis&) { return std::string("unit_restarted"); };
    WatchScope scope(watch);
    EcRig r = make_ec();
    Events events;
    MonitorOptions mo;
    mo.on_event.push_back(events.callback());
    mo.link_check_s = 0.02;
    auto mon = r.ax.start_monitor(mo);
    CHECK(wait_until([&] { return mon->link_checks() >= 3; }));
    CHECK(seen.load() == &r.ax);  // the hooks get the Axis itself
    CHECK_EQ(mon->link_state(), std::string("unit_restarted"));
    REQUIRE(mon->last_link_check().has_value());
    CHECK_EQ(mon->last_link_check()->state, std::string(kLinkOk));
    fail_next = true;  // a check that finds the port gone ends the monitor
    CHECK(wait_until([&] { return !mon->running(); }));
    CHECK(mon->error() != nullptr);
    CHECK_EQ(events.kinds(), std::vector<std::string>{"link_lost"});
    CHECK_EQ(faults.load(), 0);
}
