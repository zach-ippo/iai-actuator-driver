// Coordinated motion on the simulator: sync and streamed runs over two simulated axes on two sim lines and over two
// axes on one sim line, every stop path (an alarm, an outside stop from another thread, a late frame, link faults, the
// cell's stop_on_fault), the checks before anything is sent, lead-ins, lost echoes, the monitors held during a run.
// Port of the Python edition's tests/test_coord.py (CoordinatedRunTest, SlowLineTest, CellCoordinatedTest). Simulator
// only (no COM port). The PC may be busy: the bounds are generous and the tests assert behaviour, not tight timings.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
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

#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/coord.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::coord;

namespace {

using Bytes = std::vector<std::uint8_t>;
const PlanePoint kCenter{75.0, 75.0};

/// The model's shortest step for one axis per port: tests whose subject is not the measured line pass it, so a plan is
/// not refused because a busy PC stretched the measurement (min_step_s also skips run_plan's timing check).
PlanOptions step_options(std::optional<double> min_step = std::nullopt) {
    PlanOptions o;
    o.min_step_s = min_step.value_or(LineTiming{}.min_step_s());
    return o;
}

CircleOptions turns(double n) {
    CircleOptions c;
    c.turns = n;
    return c;
}

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// A SimLine with a delay before every request reaches it (replies tens of ms late, as hardware answers: VERIFIED in
/// the Python edition), and optional extra delays of the full status reads (FC03 at 0x9000, more than 8 words).
class LatencyLine : public Transport {
public:
    LatencyLine(std::shared_ptr<SimLine> inner, double latency_s) : latency(latency_s), inner_(std::move(inner)) {}
    std::atomic<double> latency;
    std::atomic<double> full_read_extra{0.0};  ///< every full status read this much slower
    std::atomic<double> full_read_once{0.0};   ///< the next full status read this much slower

    void write(const std::uint8_t* data, std::size_t size) override {
        double extra = latency.load();
        if (size >= 6 && data[1] == 0x03 && ((data[2] << 8) | data[3]) == 0x9000 && ((data[4] << 8) | data[5]) > 8)
            extra += full_read_extra.load() + full_read_once.exchange(0.0);
        if (extra > 0) sleep_at_least(extra);
        inner_->write(data, size);
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override { return inner_->read(buffer, size); }
    /// Every attempt of the bus on this line, in order, as the read timeout in force for it: each attempt clears the
    /// input first (ModbusRtu's transaction), also when the port is gone, with the timeout shortened to the caller's
    /// bounded wait (max_wait_s) for that attempt.
    std::vector<double> attempt_waits() const {
        std::lock_guard<std::mutex> l(attempts_mutex_);
        return attempt_waits_;
    }
    long attempts() const {
        std::lock_guard<std::mutex> l(attempts_mutex_);
        return static_cast<long>(attempt_waits_.size());
    }
    void reset_input() override {
        {
            std::lock_guard<std::mutex> l(attempts_mutex_);
            attempt_waits_.push_back(inner_->read_timeout());
        }
        inner_->reset_input();
    }
    int baud_rate() const override { return inner_->baud_rate(); }
    void set_baud_rate(int rate) override { inner_->set_baud_rate(rate); }
    double read_timeout() const override { return inner_->read_timeout(); }
    void set_read_timeout(double seconds) override { inner_->set_read_timeout(seconds); }
    void send_break(double seconds) override { inner_->send_break(seconds); }
    void close() override { inner_->close(); }
    bool is_open() const override { return inner_->is_open(); }
    std::string name() const override { return inner_->name(); }
    std::optional<bool> present() const override { return inner_->present(); }
    bool can_reopen() const override { return inner_->can_reopen(); }
    void reopen(std::optional<int> baud_rate) override { inner_->reopen(baud_rate); }

private:
    std::shared_ptr<SimLine> inner_;
    mutable std::mutex attempts_mutex_;
    std::vector<double> attempt_waits_;
};

CellOptions open_options() {
    CellOptions o;
    o.axis.poll_interval_s = 0.005;
    o.scan_timeout_s = 0.05;
    o.scan_slaves = {1, 2};
    return o;
}

/// One simulated line per entry of `starts` ("SIM1".."SIMn", one slider at slave 1 each, at that position), a cell on
/// them (each line behind a LatencyLine when latency_s > 0) and the group x = SIM1:1, y = SIM2:1.
struct Rig {
    explicit Rig(const std::vector<double>& starts, double latency_s = -1.0, CellOptions o = open_options()) {
        std::map<std::string, std::shared_ptr<Transport>> transports;
        for (std::size_t i = 0; i < starts.size(); ++i) {
            const std::string name = "SIM" + std::to_string(i + 1);
            SimulatedUnitOptions u;
            u.slave = 1;
            u.position_mm = starts[i];
            auto line = std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{
                                                      std::make_shared<SimulatedUnit>(u)},
                                                  name);
            lines[name] = line;
            if (latency_s >= 0.0) {
                auto slow = std::make_shared<LatencyLine>(line, latency_s);
                latency[name] = slow;
                transports[name] = slow;
            } else {
                transports[name] = line;
            }
        }
        cell = std::make_unique<Cell>(Cell::open(transports, {1}, o));
        xy = std::make_unique<CoordinatedGroup>(cell->coordinated({{"x", "SIM1:1"}, {"y", "SIM2:1"}}));
        sx = lines["SIM1"]->unit(1);
        sy = lines["SIM2"]->unit(1);
    }
    ~Rig() {
        try {
            cell->close();
        } catch (...) {
        }
    }
    void clear() {
        for (auto& kv : lines) kv.second->clear_requests();
    }
    std::map<std::string, std::shared_ptr<SimLine>> lines;
    std::map<std::string, std::shared_ptr<LatencyLine>> latency;
    std::unique_ptr<Cell> cell;
    std::unique_ptr<CoordinatedGroup> xy;
    std::shared_ptr<SimulatedUnit> sx, sy;
};

struct Move {
    double t = 0.0;
    double target = 0.0;
    double band = 0.0;
    double speed = 0.0;
    double accel = 0.0;
};

/// Whether `f` writes the STOP or the pause coil ON to `slave` (or broadcast).
bool is_halt(const Bytes& f, int slave) {
    if (f.size() < 6 || (f[0] != 0 && f[0] != slave) || f[1] != 0x05 || f[4] != 0xFF) return false;
    const int address = (f[2] << 8) | f[3];
    return address == reg::kCoilStop || address == reg::kCoilStp;
}

/// Whether `f` writes a 0x9900 block to `slave`.
bool is_move(const Bytes& f, int slave) {
    return f.size() >= 7 + 18 && f[0] == slave && f[1] == 0x10 && ((f[2] << 8) | f[3]) == reg::kDirectMoveBase;
}

/// Every 0x9900 block written to `slave` on `line` (after the time `after`).
std::vector<Move> moves(const SimLine& line, int slave = 1, std::optional<double> after = std::nullopt) {
    std::vector<Move> out;
    for (const auto& tf : line.requests()) {
        const Bytes& f = tf.second;
        if (!is_move(f, slave)) continue;
        if (after && tf.first <= *after) continue;
        std::vector<std::uint16_t> w;
        for (std::size_t i = 7; i + 1 < f.size() - 2; i += 2) w.push_back(static_cast<std::uint16_t>((f[i] << 8) | f[i + 1]));
        Move m;
        m.t = tf.first;
        m.target = reg::from_i32_words(w[0], w[1]) * 0.01;
        m.band = reg::from_u32_words(w[2], w[3]) * 0.01;
        m.speed = reg::from_u32_words(w[4], w[5]) * 0.01;
        m.accel = w[6] * 0.01;
        out.push_back(m);
    }
    return out;
}

/// The time of the first STOP or pause coil written ON to `slave` (or broadcast) on `line`.
std::optional<double> first_halt(const SimLine& line, int slave = 1) {
    for (const auto& tf : line.requests())
        if (is_halt(tf.second, slave)) return tf.first;
    return std::nullopt;
}

/// The 0x9900 blocks written to `slave` on `line` from its request number `from` on, before the first STOP or pause
/// coil after that point; nullopt when no such coil came.
std::optional<int> moves_before_halt(const SimLine& line, std::size_t from, int slave = 1) {
    const auto requests = line.requests();
    int count = 0;
    for (std::size_t i = from; i < requests.size(); ++i) {
        if (is_halt(requests[i].second, slave)) return count;
        if (is_move(requests[i].second, slave)) ++count;
    }
    return std::nullopt;
}

std::optional<double> first_halt_any(const Rig& rig) {
    std::optional<double> first;
    for (const auto& kv : rig.lines)
        if (auto t = first_halt(*kv.second))
            if (!first || *t < *first) first = t;
    return first;
}

/// A status read (FC03 at 0x9000) of one unit as the simulated line saw it: the arrival time and the number of words.
struct StatusRead {
    double t = 0.0;
    int words = 0;
};

std::vector<StatusRead> status_reads(const SimLine& line, int slave = 1) {
    std::vector<StatusRead> out;
    for (const auto& tf : line.requests()) {
        const Bytes& f = tf.second;
        if (f.size() >= 8 && f[0] == slave && f[1] == 0x03 && ((f[2] << 8) | f[3]) == reg::kMonitorBase)
            out.push_back({tf.first, (f[4] << 8) | f[5]});
    }
    return out;
}

/// `base` with the line measurement run_plan made, rebuilt from what each simulated line saw before the run's first
/// frame. The measurement is one lean read, then three pairs of a full and a lean read, back to back (the run's last
/// check of the axis, one full read, follows it): the arrivals of two consecutive reads are as far apart as the earlier
/// read took, so the medians of those gaps are the write_s and read_s of the run itself. A busy PC stretches that
/// measurement at its own moment (the run measures the line again, whatever the test measured before), and the run's
/// sample bound follows it. `base` as it was when a line shows something else.
LineTiming measured_by_the_run(const Rig& rig, LineTiming base) {
    auto median = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    std::optional<double> full, lean;
    for (const auto& kv : rig.lines) {
        const auto sent = moves(*kv.second);
        if (sent.empty()) continue;
        std::vector<StatusRead> reads;
        for (const auto& r : status_reads(*kv.second))
            if (r.t < sent.front().t) reads.push_back(r);
        if (!reads.empty() && reads.back().words > 8) reads.pop_back();  // the check after the measurement
        if (reads.size() < 7) continue;
        reads.erase(reads.begin(), reads.end() - 7);  // lean, then (full, lean) x 3
        std::vector<double> fulls, leans;
        for (std::size_t i = 0; i < 6; ++i) {
            const bool is_full = reads[i].words > 8;
            if (is_full != (i % 2 == 1)) return base;  // not the pattern: leave the test's own measurement
            (is_full ? fulls : leans).push_back(reads[i + 1].t - reads[i].t);
        }
        full = std::max(full.value_or(0.0), median(fulls));
        lean = std::max(lean.value_or(0.0), median(leans));
    }
    if (!full || !lean) return base;
    base.measured_write_s = *full;
    base.measured_read_s = *lean;
    return base;
}

bool has_event(const SimulatedUnit& unit, const std::string& event) {
    for (const auto& e : unit.events())
        if (e == event) return true;
    return false;
}

/// Wait (at most 3 s) until no unit of `units` moves; false when one still does.
bool stood_still(const std::vector<std::shared_ptr<SimulatedUnit>>& units) {
    const double deadline = now_s() + 3.0;
    for (;;) {
        bool moving = false;
        for (const auto& u : units) moving = moving || u->moving();
        if (!moving) return true;
        if (now_s() > deadline) return false;
        sleep_s(0.01);
    }
}

/// The simulated axes' true positions, read back to back in a background thread during a run: the path the axes
/// really drew, independent of the driver's status samples.
class Truth {
public:
    explicit Truth(std::vector<std::shared_ptr<SimulatedUnit>> units) : units_(std::move(units)) {
        thread_ = std::thread([this] {
            while (!stop_.load()) {
                std::vector<double> p;
                const double begin = now_s();
                for (const auto& u : units_) p.push_back(u->position_mm());
                const double took = now_s() - begin;  // the units were read this far apart (a busy PC stretches it)
                {
                    std::lock_guard<std::mutex> l(m_);
                    points_.push_back(p);
                    spread_ = std::max(spread_, took);
                }
                sleep_s(0.003);
            }
        });
    }
    ~Truth() { end(); }
    void end() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
    }
    double deviation(const std::vector<std::vector<double>>& polyline) {
        end();
        return path_deviation(points_, polyline).first;
    }
    /// The longest time between reading the first and the last unit of one point: the point is off the path by up to
    /// the path speed times this (the position reads are not simultaneous). Call after deviation().
    double spread() const { return spread_; }

private:
    std::vector<std::shared_ptr<SimulatedUnit>> units_;
    std::vector<std::vector<double>> points_;
    double spread_ = 0.0;
    std::mutex m_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

/// Run `fn` in another thread `extra` seconds after one of `units` starts moving (the run under way, however long its
/// planning and line measurement took on a busy PC; at most 20 s of waiting); joined by the destructor.
class WhenMoving {
public:
    WhenMoving(std::vector<std::shared_ptr<SimulatedUnit>> units, double extra, std::function<void()> fn)
        : thread_([units, extra, fn] {
              const double deadline = now_s() + 20.0;
              for (bool moving = false; !moving && now_s() < deadline;) {
                  for (const auto& u : units) moving = moving || u->moving();
                  if (!moving) sleep_s(0.002);
              }
              sleep_s(extra);
              try {
                  fn();
              } catch (...) {
              }
          }) {}
    ~WhenMoving() {
        if (thread_.joinable()) thread_.join();
    }

private:
    std::thread thread_;
};

/// A run that a busy PC does not abort for late frames or for a slow arrival (tests whose subject is not the frame
/// timing): both limits are wall-clock margins, and a loaded PC can stretch the last frame and the wait for the axis.
RunOptions tolerant() {
    RunOptions o;
    o.late_abort_s = 10.0;
    o.timeout_s = 30.0;
    return o;
}

/// The test hooks for one scope.
struct Hooks {
    Hooks() { coord::detail::coord_test_hooks() = coord::detail::CoordTestHooks{}; }
    ~Hooks() { coord::detail::coord_test_hooks() = coord::detail::CoordTestHooks{}; }
    coord::detail::CoordTestHooks& operator*() { return coord::detail::coord_test_hooks(); }
    coord::detail::CoordTestHooks* operator->() { return &coord::detail::coord_test_hooks(); }
};

/// Run `fn` in another thread once the run under test has sent `frames` 0x9900 frames (counted by the before_frame
/// hook): the run is registered on its axes and streaming, however long its planning and line measurement took on a
/// busy PC and whatever the axes did before it (a state, not a delay). Never runs `fn` when the frames do not come
/// (30 s at most). Set up and joined while no run is active (the hook is read by the run's port threads).
class AfterFrames {
public:
    AfterFrames(int frames, std::function<void()> fn) {
        coord::detail::coord_test_hooks().before_frame = [this](const std::string&) { ++sent_; };
        thread_ = std::thread([this, frames, fn] {
            const double deadline = now_s() + 30.0;
            while (sent_.load() < frames) {
                if (done_.load() || now_s() > deadline) return;  // the run ended (or was refused) before the frames
                sleep_s(0.001);
            }
            try {
                fn();
            } catch (...) {
            }
        });
    }
    ~AfterFrames() {
        done_ = true;
        if (thread_.joinable()) thread_.join();
        coord::detail::coord_test_hooks().before_frame = nullptr;
    }
    int sent() const { return sent_.load(); }

private:
    std::atomic<int> sent_{0};
    std::atomic<bool> done_{false};
    std::thread thread_;
};

template <class F>
std::optional<CoordinatedMotionError> motion_error(F&& fn) {
    try {
        fn();
    } catch (const CoordinatedMotionError& e) {
        return e;
    }
    return std::nullopt;
}

}  // namespace

// ============================================================================================ runs on two sim lines

TEST(coord_run_sync_line_scales_speeds_and_lands_together) {
    Rig rig({50.0, 50.0});
    rig.xy->line_to({{"x", 20.0}, {"y", 30.0}}, 60, 0.5);
    rig.clear();
    Truth truth({rig.sx, rig.sy});
    const CoordinatedResult result = rig.xy->line_to({{"x", 80.0}, {"y", 50.0}}, 40, 0.3);
    const double deviation = truth.deviation(result.plan.points);
    CHECK(result.completed);
    const auto fx = moves(*rig.lines["SIM1"]);
    const auto fy = moves(*rig.lines["SIM2"]);
    REQUIRE(fx.size() == 1);
    REQUIRE(fy.size() == 1);
    CHECK_NEAR(fx[0].speed / fy[0].speed, 60.0 / 20.0, 0.01);
    CHECK_NEAR(std::hypot(fx[0].speed, fy[0].speed), 40, 0.02);
    CHECK_NEAR(fx[0].accel / fy[0].accel, 3, 0.35);  // 0.01 G steps
    CHECK_NEAR(rig.sx->position_mm(), 80.0, 0.1);
    CHECK_NEAR(rig.sy->position_mm(), 50.0, 0.1);
    // the point stayed on the straight line: the true path within 0.1 plus what the start skew explains. The skew is
    // measured where it counts, at the simulated lines (worst_skew_s stamps the call, not the wire: the bus gap and
    // a busy PC come between), and the reads of the true positions are not simultaneous either (Truth::spread)
    const double wire_skew = std::fabs(fx[0].t - fy[0].t);
    CHECK(deviation < 0.1 + 40 * (wire_skew + truth.spread()));
    // the estimate from the samples stays in the same range, plus what a read held up by a busy PC explains (the
    // longest gap between two samples of one axis beyond 30 ms: a sample is stamped at the middle of its read)
    double widest = 0.0;
    for (const auto& kv : result.samples)
        for (std::size_t i = 0; i + 1 < kv.second.size(); ++i)
            widest = std::max(widest, kv.second[i + 1].first - kv.second[i].first);
    CHECK(result.max_deviation < 0.5 + 40 * (wire_skew + std::max(0.0, widest - 0.03)));
    CHECK(result.worst_skew_s < 0.1);
    CHECK(!result.to_json().dump().empty());
    CHECK(contains(result.summary(), "sync run completed"));
}

TEST(coord_run_streamed_circle_on_two_sims) {
    Rig rig({85.0, 75.0});
    rig.clear();
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0), 30, 0.3, step_options());
    CHECK(p.mode == Mode::Stream);
    CHECK(!p.lead_in);
    for (const auto& w : p.warnings) CHECK(!contains(w, "lead-in"));
    Truth truth({rig.sx, rig.sy});
    const CoordinatedResult result = rig.xy->run_plan(p, tolerant());
    const double deviation = truth.deviation(p.points);
    CHECK(result.completed);
    // under PC load frames slip (counted, below the abort limit): the bound grows by what the lateness explains
    // (and the true positions are read one after the other: Truth::spread)
    CHECK(deviation < 0.3 + p.path_speed * (result.worst_late_s + truth.spread()));
    CHECK(result.max_deviation < 0.5 + p.path_speed * result.worst_late_s);
    // sampled every ~20 ms in the slack between frames; a run that slipped (late frames, counted) has no slack to
    // read in
    CHECK(result.trace.size() > 20 || result.late_frames > 0);
    CHECK(result.sample_spacing_s > 0);
    CHECK_NEAR(rig.sx->position_mm(), 85.0, 0.1);
    CHECK_NEAR(rig.sy->position_mm(), 75.0, 0.1);
    const auto ranges = rig.xy->ranges();
    for (const auto& name : {"SIM1", "SIM2"}) {
        const auto frames = moves(*rig.lines[name]);
        CHECK(static_cast<int>(frames.size()) > p.steps() / 2);
        for (const auto& f : frames) {
            CHECK(0.0 <= f.target && f.target <= 150.0);
            CHECK(0.01 <= f.speed && f.speed <= 700.0);
            CHECK(0.01 <= f.accel && f.accel <= ranges.at("x").max_accel_g.value_or(3.0) + 1e-9);
            CHECK(std::fabs(f.target - 75.0) <= 10.01);  // every target on the circle's span
        }
    }
    const Json data = Json::parse(result.to_json().dump());
    CHECK_EQ(data.find("axes")->size(), std::size_t(2));
    CHECK_EQ(data.find("plan")->find("mode")->as_string(), std::string("stream"));
    CHECK_EQ(data.find("trace")->size(), result.trace.size());  // the trace is in the JSON as it is in the result
}

TEST(coord_run_sync_rectangle_across_ports_stays_on_path) {
    // every segment moves one axis on its own port: only the barrier keeps the other port from running ahead
    Rig rig({20.0, 20.0});
    Truth truth({rig.sx, rig.sy});
    const CoordinatedResult result = rig.xy->run(Path::rectangle({20.0, 20.0}, 20, 15), 40, 0.3);
    const double deviation = truth.deviation(result.plan.points);
    CHECK(result.completed);
    CHECK(result.plan.mode == Mode::Sync);
    CHECK_EQ(result.plan.steps(), 4);
    CHECK(deviation < 0.3 + 40 * truth.spread());  // the true positions are read one after the other
    CHECK(result.max_deviation < 0.3);
    CHECK_NEAR(rig.sx->position_mm(), 20.0, 0.1);
    CHECK_NEAR(rig.sy->position_mm(), 20.0, 0.1);
}

TEST(coord_run_fault_alarm_stops_every_axis_of_the_group) {
    Rig rig({85.0, 75.0});
    std::optional<CoordinatedMotionError> err;
    {
        WhenMoving alarm({rig.sx, rig.sy}, 0.4, [&] { rig.sy->trigger_alarm(0x0C0); });
        err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 25, 0.3, {}, step_options(), tolerant()); });
    }
    REQUIRE(err.has_value());
    CHECK(err->axis() == std::optional<std::string>("y"));
    CHECK(!err->result().completed);
    CHECK(contains(err->result().error.value_or(""), "alarm"));
    CHECK(err->result().samples.at("x").size() > 2);
    CHECK(stood_still({rig.sx, rig.sy}));
    CHECK(has_event(*rig.sx, "stop"));
    CHECK(std::fabs(rig.sx->position_mm() - 85.0) > 0.5);  // stopped on the way, not at the end
    CHECK(!err->stop().names.empty());
}

TEST(coord_run_outside_stop_ends_the_run) {
    // a stop from another thread (the group's, the cell's, one axis's, a pause) must end the run: no frame after the
    // first STOP / pause frame, every axis stopped, CoordinatedMotionError - never completed. The stop is triggered by
    // the run's state (its 6th frame sent: registered and streaming, most of the circle still ahead), not by a delay:
    // a stop that lands before the run (a busy PC) would test a refusal instead, a STOP still held after the previous
    // case would refuse the next run.
    Rig rig({85.0, 75.0});
    Hooks hooks;
    const std::vector<std::pair<std::string, std::function<void()>>> stops = {
        {"group stop", [&] { rig.xy->stop(); }},
        {"cell stop_all", [&] { rig.cell->stop_all(); }},
        {"axis stop", [&] { (*rig.cell)["SIM2:1"].stop(); }},
        {"axis pause", [&] { (*rig.cell)["SIM1:1"].pause(); }},
    };
    for (const auto& kv : stops) {
        (*rig.cell)["SIM1:1"].resume();
        (*rig.cell)["SIM2:1"].resume();
        REQUIRE(stood_still({rig.sx, rig.sy}));
        rig.xy->line_to({{"x", 85.0}, {"y", 75.0}}, 60, 0.5);
        REQUIRE(stood_still({rig.sx, rig.sy}));  // the circle starts from rest at its first point
        rig.clear();
        std::optional<CoordinatedMotionError> err;
        int sent = 0;
        {
            AfterFrames stop(6, kv.second);
            err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant()); });
            sent = stop.sent();
        }
        CHECK(sent >= 6);  // the stop came from outside while the run was streaming
        REQUIRE(err.has_value());
        CHECK(!err->result().completed);
        if (!contains(err->what(), "outside")) std::printf("  %s: %s\n", kv.first.c_str(), err->what());
        CHECK(contains(err->what(), "outside"));
        CHECK(stood_still({rig.sx, rig.sy}));
        const std::optional<double> first = first_halt_any(rig);
        REQUIRE(first.has_value());
        for (const auto& line : rig.lines) CHECK(moves(*line.second, 1, *first).empty());
    }
    (*rig.cell)["SIM1:1"].resume();
}

TEST(coord_run_a_port_parked_after_its_checks_sends_nothing_after_an_outside_stop) {
    // A4 (vi): x's port thread has passed its abort and stop-count checks and is parked right before its frame (a
    // latch). An outside stop of x goes out at once (no drain waits for the port), then the port is let go: its frame
    // meets the STOP-first gate under the bus lock and is refused. Before: the STOP waited out the drain, and the parked
    // frame went out after it.
    Rig rig({85.0, 75.0});
    Hooks hooks;
    std::atomic<int> frames{0};
    std::atomic<bool> parked{false};
    std::atomic<bool> released{false};
    hooks->before_frame = [&](const std::string& axis) {
        if (axis != "x" || ++frames != 4) return;
        parked = true;
        const double until = now_s() + 10.0;
        while (!released.load() && now_s() < until) sleep_s(0.001);
    };
    std::optional<CoordinatedMotionError> err;
    {
        ec_test::Thread stopper([&] {
            const double until = now_s() + 30.0;
            while (!parked.load() && now_s() < until) sleep_s(0.001);
            if (!parked.load()) return;  // the run ended before the frame: the checks below fail
            ec_test::Thread stop([&] { (*rig.cell)["SIM1:1"].stop(); });
            while (!first_halt(*rig.lines["SIM1"]) && now_s() < until) sleep_s(0.001);  // the STOP is on the wire
            released = true;
            stop.join();
        });
        err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant()); });
        released = true;  // whatever happened: never leave the port parked
    }
    CHECK(parked.load());
    REQUIRE(err.has_value());
    CHECK(!err->result().completed);
    CHECK(contains(err->what(), "outside"));
    const std::optional<double> halt = first_halt(*rig.lines["SIM1"]);
    REQUIRE(halt.has_value());
    CHECK(moves(*rig.lines["SIM1"], 1, *halt).empty());  // the parked frame never went out after the STOP
    CHECK(stood_still({rig.sx, rig.sy}));
}

TEST(coord_run_stop_from_another_thread_ends_a_run_in_a_worker_thread) {
    Rig rig({85.0, 75.0});
    std::exception_ptr error;
    std::thread worker([&] {
        try {
            rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant());
        } catch (...) {
            error = std::current_exception();
        }
    });
    {
        WhenMoving stop({rig.sx, rig.sy}, 0.3, [&] { rig.xy->stop(); });
    }
    worker.join();
    bool coordinated = false;
    try {
        if (error) std::rethrow_exception(error);
    } catch (const CoordinatedMotionError&) {
        coordinated = true;
    } catch (...) {
    }
    CHECK(coordinated);
    CHECK(stood_still({rig.sx, rig.sy}));
    const double x = rig.sx->position_mm(), y = rig.sy->position_mm();
    sleep_s(0.2);
    CHECK_EQ(rig.sx->position_mm(), x);  // nothing restarted them
    CHECK_EQ(rig.sy->position_mm(), y);
    REQUIRE(rig.xy->last_result().has_value());
    CHECK(!rig.xy->last_result()->completed);
}

TEST(coord_run_a_failed_run_in_a_worker_thread_is_reported) {
    // y alarms just before its 4th frame: the run in the worker thread fails with a CoordinatedMotionError. The test
    // gets it as data from ec_test::Thread; a raw std::thread lets it escape the thread and ends the process
    // (std::terminate), as a run that a loaded PC aborts for a late frame did
    Rig rig({85.0, 75.0});
    Hooks hooks;
    std::atomic<int> frames{0};
    hooks->before_frame = [&](const std::string& axis) {
        if (axis == "y" && ++frames == 4) rig.sy->trigger_alarm(0x0C0);
    };
    ec_test::Thread worker([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant()); });
    const std::exception_ptr error = worker.error();  // joined; taken as data, not reported as this test's failure
    REQUIRE(error != nullptr);
    std::optional<CoordinatedMotionError> err;
    std::string other;
    try {
        std::rethrow_exception(error);
    } catch (const CoordinatedMotionError& e) {
        err = e;
    } catch (const std::exception& e) {
        other = e.what();
    }
    if (!err) std::printf("  not a CoordinatedMotionError: %s\n", other.c_str());
    REQUIRE(err.has_value());
    CHECK(err->axis() == std::optional<std::string>("y"));
    CHECK(contains(err->result().error.value_or(""), "alarm"));
    CHECK(!err->result().completed);
    CHECK(stood_still({rig.sx, rig.sy}));
}

TEST(coord_run_a_frame_later_than_its_step_aborts) {
    Rig rig({85.0, 75.0});
    Hooks hooks;
    hooks->schedule_delay_s = 0.3;
    const auto err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 25, 0.3, {}, step_options()); });
    REQUIRE(err.has_value());
    CHECK(contains(err->what(), "late"));
    CHECK(stood_still({rig.sx, rig.sy}));
}

TEST(coord_run_refused_plan_sends_nothing) {
    Rig rig({50.0, 50.0});
    rig.clear();
    CHECK_THROWS(rig.xy->run(Path::circle({145.0, 75.0}, 10.0), 20, 0.3), SoftLimitError);
    CHECK_THROWS(rig.xy->run(Path::circle(kCenter, 10.0), 200, 0.3), SoftLimitError);  // steps too short for the line
    // a plan made elsewhere: its middle waypoint leaves the range
    const MotionPlan elsewhere =
        plan(Path(Point{{"x", 73.95}, {"y", 73.95}}, {Polyline({Point{{"x", 160.0}}, Point{{"x", 73.95}}})}), 20, 0.3);
    CHECK_THROWS(rig.xy->run_plan(elsewhere), SoftLimitError);
    CHECK(moves(*rig.lines["SIM1"]).empty());
    CHECK(moves(*rig.lines["SIM2"]).empty());
    CHECK_THROWS(rig.xy->run(Path(Point{{"z", 0.0}}, {LineSegment({{"z", 5.0}})}), 10, 0.1), std::invalid_argument);
}

TEST(coord_run_a_timeout_that_is_not_above_zero_is_refused_before_anything_is_sent) {
    // 0, a negative value ("wait forever" in some languages) and an infinity: before, the run went out and was
    // decel-stopped at once (a timeout of 0 ended the wait for the last waypoint before it began); now nothing is sent,
    // as for every command timeout (detail::check_timeout). Absent (nullopt) is the default.
    Rig rig({50.0, 50.0});
    const Point target{{"x", 70.0}, {"y", 60.0}};
    const Point here{{"x", 50.0}, {"y", 50.0}};
    const Path line(std::nullopt, {LineSegment(target)});
    const MotionPlan ready = rig.xy->plan(line, 40, 0.3);
    rig.clear();
    const double bad[] = {0.0, -1.0, -0.001, std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity()};
    for (const double timeout : bad) {
        RunOptions o;
        o.timeout_s = timeout;
        CHECK_THROWS(rig.xy->run_plan(ready, o), SoftLimitError);
        CHECK_THROWS(rig.xy->run(line, 40, 0.3, {}, o), SoftLimitError);
        CHECK_THROWS(rig.xy->line_to(target, 40, 0.3, {}, o), SoftLimitError);
        CHECK_THROWS(rig.xy->circle(kCenter, 10.0, 25, 0.3, {}, step_options(), o), SoftLimitError);
        CHECK_THROWS(rig.xy->line_to(here, 40, 0.3, {}, o), SoftLimitError);  // no motion either: never a completed result
    }
    try {
        RunOptions o;
        o.timeout_s = 0.0;
        rig.xy->run_plan(ready, o);
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(contains(e.what(), "timeout_s"));
        CHECK(contains(e.what(), "nothing sent"));
    }
    CHECK(moves(*rig.lines["SIM1"]).empty());
    CHECK(moves(*rig.lines["SIM2"]).empty());
    CHECK(!first_halt_any(rig).has_value());  // no STOP either: the axes were never started
    CHECK_NEAR(rig.sx->position_mm(), 50.0, 0.01);
    // a positive timeout is the run's own, and the plan was not spoilt by the refusals
    RunOptions fine;
    fine.timeout_s = 30.0;
    const CoordinatedResult result = rig.xy->run_plan(ready, fine);
    CHECK(result.completed);
    CHECK_NEAR(result.final.at("x"), 70.0, 0.11);
    CHECK_NEAR(result.final.at("y"), 60.0, 0.11);
}

TEST(coord_run_a_stroke_narrowed_after_planning_is_refused) {
    Rig rig({85.0, 75.0});
    const auto ranges = rig.xy->ranges();
    REQUIRE(ranges.at("x").position.has_value());
    CHECK(*ranges.at("x").position == (Span{0.0, 150.0}));
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0), 20, 0.3, step_options());
    AxisOptions o = (*rig.cell)["SIM1:1"].options();
    o.stroke = Span{70.0, 150.0};  // e.g. a fixture added: the circle reaches x = 65
    (*rig.cell)["SIM1:1"].set_options(o);
    rig.clear();
    try {
        rig.xy->run_plan(p);
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(contains(e.what(), "70..150"));
    }
    CHECK(moves(*rig.lines["SIM1"]).empty());
    CHECK(moves(*rig.lines["SIM2"]).empty());
    try {  // the dry run reads the ranges again too
        rig.xy->plan(Path::circle(kCenter, 10.0), 20, 0.3, step_options());
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(contains(e.what(), "70..150"));
    }
}

TEST(coord_run_lead_in_runs_first_as_a_synchronized_line) {
    Rig rig({50.0, 50.0});
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0, 0.0, 0.5), 20, 0.3, step_options());
    REQUIRE(!p.warnings.empty());
    CHECK(contains(p.warnings[0], "lead-in"));
    REQUIRE(p.lead_in != nullptr);
    CHECK(p.lead_in->mode == Mode::Sync);
    CHECK_NEAR(p.lead_in->points[0][0], rig.sx->position_mm(), 1e-6);
    CHECK_NEAR(p.points[0][0], 85.0, 1e-6);
    CHECK(contains(p.summary(), "lead-in first"));
    Truth truth({rig.sx, rig.sy});
    const CoordinatedResult result = rig.xy->run_plan(p, tolerant());
    std::vector<std::vector<double>> polyline{p.lead_in->points[0]};
    polyline.insert(polyline.end(), p.points.begin(), p.points.end());
    const double deviation = truth.deviation(polyline);
    CHECK(result.completed);
    CHECK(deviation < 0.3 + 20 * (result.worst_late_s + truth.spread()));
    CHECK_NEAR(rig.sx->position_mm(), 65.0, 0.1);  // half a turn from 0 deg
}

TEST(coord_run_a_circle_a_little_off_radius_needs_no_lead_in) {
    // 0.15 mm outside a 40 mm circle: within its 0.5 % tolerance, so it starts where the axes are
    Rig rig({115.15, 75.0});
    const CoordinatedResult result = rig.xy->circle(kCenter, 40.0, 40, 0.3, turns(0.25), step_options(), tolerant());
    CHECK(result.completed);
    CHECK(!result.plan.lead_in);
    for (const auto& w : result.warnings) CHECK(!contains(w, "lead-in"));
}

TEST(coord_run_line_to_where_the_axes_are_returns_at_once) {
    Rig rig({85.0, 75.0});
    rig.clear();
    const CoordinatedResult result = rig.xy->line_to({{"x", 85.0}, {"y", 75.0}}, 60, 0.5);
    CHECK(result.completed);
    CHECK_EQ(result.frames, 0);
    CHECK(moves(*rig.lines["SIM1"]).empty());
    CHECK(moves(*rig.lines["SIM2"]).empty());
    CHECK_NEAR(result.final.at("x"), 85.0, 0.01);
}

TEST(coord_run_link_faults_stop_the_group) {
    // y's link fails just before the first attempt of its 6th frame: a state of the run (streaming, most of the circle
    // ahead), not a delay. The run finds the fault on bounded waits, not on the bus's retries x timeouts: from the
    // fault to the error that ends y's port, y gets that frame once (a TransportError is never retried) or the frame
    // and one re-send (a lost echo is sent again once, both lost is a link fault), each attempt with the run's own
    // wait (the line's read timeout shortened for it). Then the group stops at once: x gets its STOP before a second
    // frame of its own (the one in flight when y's port failed may still go out) and stands still. Counted in frames,
    // not seconds: a run that kept streaming x after the fault would send it several before the STOP.
    const double line_timeout = 60.0;  // what an attempt without the run's bounded wait would be given
    for (const std::string kind : {"unplug", "power_off"}) {
        Rig rig({85.0, 75.0}, 0.01);
        LatencyLine& y_line = *rig.latency.at("SIM2");
        SimLine& x_line = *rig.lines.at("SIM1");
        rig.lines["SIM2"]->set_read_timeout(line_timeout);  // only stored: a simulated line answers at once or never
        std::atomic<long> fault_at{-1}, ended_at{-1}, x_requests_at{-1};
        std::optional<CoordinatedMotionError> err;
        {
            int y_attempts = 0;  // y's port thread only
            Hooks hooks;
            hooks->before_frame = [&](const std::string& axis) {
                if (axis != "y" || ++y_attempts != 6) return;
                fault_at = y_line.attempts();
                if (kind == "unplug")
                    rig.lines["SIM2"]->unplug();
                else
                    rig.sy->power_off();
            };
            hooks->on_port_error = [&] {  // in the port thread that failed, before the run sees the error
                long none = -1;
                if (ended_at.compare_exchange_strong(none, y_line.attempts()))
                    x_requests_at = static_cast<long>(x_line.requests().size());
            };
            err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 20, 0.3, {}, step_options(), tolerant()); });
        }
        REQUIRE(err.has_value());
        CHECK(err->axis() == std::optional<std::string>("y"));
        REQUIRE(fault_at.load() >= 0);
        REQUIRE(ended_at.load() >= fault_at.load());
        const std::vector<double> waits = y_line.attempt_waits();
        const std::vector<double> after(waits.begin() + fault_at.load(), waits.begin() + ended_at.load());
        CHECK_EQ(static_cast<int>(after.size()), kind == "unplug" ? 1 : 2);
        for (const double w : after) {
            CHECK(w >= kWaitMinS);
            CHECK(w < line_timeout);
        }
        REQUIRE(x_requests_at.load() >= 0);
        const std::optional<int> x_moves = moves_before_halt(x_line, static_cast<std::size_t>(x_requests_at.load()));
        REQUIRE(x_moves.has_value());  // x got its STOP after y's port failed
        CHECK(*x_moves <= 1);          // at most the frame in flight went out before it
        CHECK(stood_still({rig.sx}));
        CHECK(has_event(*rig.sx, "stop"));
    }
}

TEST(coord_run_one_lost_echo_is_sent_again) {
    // steps (~200 ms) well longer than the echo wait (50 ms) plus a frame: the re-send fits, the path goes on. The late
    // limit is off here (tolerant: a busy PC stretches wall-clock time, and the limit has its own tests), so the run
    // completing says nothing about when the re-send reached the wire: that is read from the line's request log below.
    Rig rig({85.0, 75.0}, 0.01);
    rig.lines["SIM2"]->clear_requests();
    Hooks hooks;
    int count = 0;
    hooks->before_frame = [&](const std::string& axis) {
        if (axis == "y" && ++count == 3) rig.sy->inject_fault(SimFault::Timeout);
    };
    const CoordinatedResult result = rig.xy->circle(kCenter, 10.0, 10, 0.3, turns(0.25), step_options(), tolerant());
    CHECK(result.completed);
    int planned = 0;
    for (const auto& sp : result.plan.speeds)
        if (sp[1] > 0) ++planned;
    const std::vector<Move> sent = moves(*rig.lines["SIM2"]);
    CHECK_EQ(static_cast<int>(sent.size()), planned + 1);  // the lost one went out twice
    // The re-send went out at once: the two identical frames are an echo wait apart (the run's own, rebuilt from the
    // line it measured) plus what a busy PC adds, which stays well short of a step. A re-send held back for the step
    // (or longer) is a step late, and it still completes the path here.
    std::vector<double> gaps;
    for (std::size_t i = 0; i + 1 < sent.size(); ++i)
        if (sent[i].target == sent[i + 1].target && sent[i].speed == sent[i + 1].speed)
            gaps.push_back(sent[i + 1].t - sent[i].t);
    REQUIRE(gaps.size() == 1);  // the frame whose echo was lost, and no other repeat
    const LineTiming ran = measured_by_the_run(rig, LineTiming{});
    const double echo_wait = std::max(kWaitMinS, kWaitFactor * ran.write_s());
    CHECK(gaps.front() < echo_wait + 0.5 * result.plan.shortest_step_s());
}

TEST(coord_run_a_lost_echo_without_time_to_resend_is_a_late_frame_not_a_link_fault) {
    // steps (~50 ms) shorter than the echo wait plus the bus gap: the frame is late, the group stops; the cell's fault
    // handling is not involved (one lost echo is no link fault), so z outside the group keeps moving
    std::mutex m;
    std::vector<std::string> faults;
    CellOptions o = open_options();
    o.on_fault = [&](const std::string& name, const std::string&, const std::string&) {
        std::lock_guard<std::mutex> l(m);
        faults.push_back(name);
    };
    Rig rig({85.0, 75.0, 20.0}, 0.01, o);
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0), 40, 0.3, step_options(0.04));
    CHECK(p.shortest_step_s() < kWaitMinS + 0.005);
    MoveOptions mo;
    mo.speed = 10;
    mo.accel_g = 0.3;
    mo.wait = false;
    (*rig.cell)["SIM3:1"].move_to(140.0, mo);
    Hooks hooks;
    int count = 0;
    hooks->before_frame = [&](const std::string& axis) {
        if (axis == "y" && ++count == 3) rig.sy->inject_fault(SimFault::Timeout);
    };
    const auto err = motion_error([&] { rig.xy->run_plan(p); });
    REQUIRE(err.has_value());
    CHECK(contains(err->what(), "late"));
    CHECK(err->axis() == std::optional<std::string>("y"));
    sleep_s(0.1);
    {
        std::lock_guard<std::mutex> l(m);
        CHECK(faults.empty());
    }
    CHECK(rig.cell->faults().empty());
    CHECK_EQ(recovery::link_state((*rig.cell)["SIM2:1"]), std::string("ok"));
    CHECK(rig.lines["SIM3"]->unit(1)->moving());
}

TEST(coord_run_a_stop_between_the_lead_in_and_the_path_ends_the_run) {
    // the lead-in has ended, the path's own checks run: a stop then must refuse the path (nothing sent after it)
    Rig rig({50.0, 50.0});
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0, 0.0, 0.5), 20, 0.3, step_options());
    REQUIRE(p.lead_in != nullptr);
    Hooks hooks;
    int calls = 0;
    hooks->before_execute = [&](const MotionPlan&) {
        if (++calls == 2) rig.xy->stop();  // the path, after the lead-in
    };
    rig.clear();
    const auto err = motion_error([&] { rig.xy->run_plan(p); });
    REQUIRE(err.has_value());
    CHECK_EQ(calls, 2);
    CHECK(contains(err->what(), "outside"));
    CHECK(!err->result().completed);
    const std::optional<double> first = first_halt_any(rig);
    REQUIRE(first.has_value());
    for (const auto& line : rig.lines) CHECK(moves(*line.second, 1, *first).empty());
    CHECK_NEAR(rig.sx->position_mm(), 85.0, 0.1);  // the lead-in's end, no further
}

TEST(coord_run_a_stop_while_run_plans_ends_the_run) {
    // run() plans first (position reads and a line measurement: a while on a slow line); a stop in that time must end
    // the run before its first frame, as one after run_plan began does (a binding's run_plan plans again in the core)
    Rig rig({85.0, 75.0});
    Hooks hooks;
    int calls = 0;
    hooks->after_plan = [&](const MotionPlan&) {
        ++calls;
        rig.xy->stop();
    };
    rig.clear();
    const auto err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options()); });
    REQUIRE(err.has_value());
    CHECK_EQ(calls, 1);
    CHECK(contains(err->what(), "outside"));
    CHECK(!err->result().completed);
    for (const auto& line : rig.lines) CHECK(moves(*line.second).empty());
    CHECK_NEAR(rig.sx->position_mm(), 85.0, 0.01);
}

TEST(coord_run_a_broadcast_stop_while_run_plans_ends_the_run) {
    // A broadcast STOP (or one through another handle on the same unit) counts on no axis's stop counter: only the
    // bus's stop mark sees it. Before, the run took that mark when it registered, after the STOP, and streamed its
    // frames after it. Now the mark is taken with the counters when run() began: x's frames are refused.
    Rig rig({85.0, 75.0});
    Hooks hooks;
    std::shared_ptr<ModbusRtu> bus = (*rig.cell)["SIM1:1"].bus();
    hooks->after_plan = [&](const MotionPlan&) {
        const StopPriority stop_priority;
        bus->broadcast_coil(reg::kCoilStop, true, 0.0);
    };
    rig.clear();
    const auto err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options()); });
    hooks->after_plan = nullptr;
    {
        const StopPriority stop_priority;
        bus->broadcast_coil(reg::kCoilStop, false, 0.0);
    }
    REQUIRE(err.has_value());
    CHECK(!err->result().completed);
    const std::optional<double> halt = first_halt(*rig.lines["SIM1"]);
    REQUIRE(halt.has_value());
    CHECK(moves(*rig.lines["SIM1"], 1, *halt).empty());  // nothing to x after the broadcast STOP
    CHECK(stood_still({rig.sx, rig.sy}));
}

TEST(coord_run_the_expected_start_is_checked_inside_the_stop_window) {
    // a binding's run_plan sends the start of the plan it holds (RunOptions::expect_start) instead of reading the axes
    // before the call: the core checks it after planning, so a stop sent meanwhile still ends the run before its first
    // frame; an axis away from that start is refused, nothing sent
    Rig rig({85.0, 75.0});
    Hooks hooks;
    int calls = 0;
    hooks->after_plan = [&](const MotionPlan&) {
        ++calls;
        rig.xy->stop();
    };
    RunOptions o;
    o.expect_start = Point{{"x", 85.0}, {"y", 75.0}};
    rig.clear();
    const auto err = motion_error([&] { rig.xy->run(Path::circle(kCenter, 10.0), 15, 0.3, step_options(), o); });
    REQUIRE(err.has_value());
    CHECK_EQ(calls, 1);
    CHECK(contains(err->what(), "outside"));
    CHECK(!err->result().completed);
    for (const auto& line : rig.lines) CHECK(moves(*line.second).empty());

    hooks->after_plan = nullptr;
    o.expect_start = Point{{"x", 80.0}, {"y", 75.0}};
    try {
        rig.xy->run(Path::circle(kCenter, 10.0), 15, 0.3, step_options(), o);
        CHECK(false);
    } catch (const EcNotReadyError& e) {
        CHECK(contains(e.what(), "the plan starts at 80.000"));
    }
    o.expect_start = Point{{"z", 0.0}};
    CHECK_THROWS(rig.xy->run(Path::circle(kCenter, 10.0), 15, 0.3, step_options(), o), std::invalid_argument);
    for (const auto& line : rig.lines) CHECK(moves(*line.second).empty());
    CHECK_NEAR(rig.sx->position_mm(), 85.0, 0.01);
}

TEST(coord_run_a_path_that_moves_only_through_its_lead_in_gets_there) {
    // the path itself moves no axis by a count (a sub-count arc, a stream line to its own start): the lead-in still
    // takes the axes to it - completed never stands for a move that did not happen
    Rig rig({85.0, 75.0});
    const std::vector<std::pair<Path, Mode>> cases = {
        {Path(Point{{"x", 60.0}, {"y", 60.0}}, {Arc({50.0, 60.0}, {{"x", 60.0}, {"y", 60.004}})}), Mode::Auto},
        {Path(Point{{"x", 40.0}, {"y", 50.0}}, {LineSegment({{"x", 40.0}, {"y", 50.0}})}), Mode::Stream},
    };
    for (const auto& c : cases) {
        PlanOptions o = step_options();
        o.mode = c.second;
        const CoordinatedResult result = rig.xy->run(c.first, 20, 0.3, o);
        CHECK(result.completed);
        CHECK(result.frames > 0);
        REQUIRE(!result.warnings.empty());
        CHECK(contains(result.warnings[0], "lead-in"));
        CHECK_NEAR(rig.sx->position_mm(), c.first.start()->at("x"), 0.1);
        CHECK_NEAR(rig.sy->position_mm(), c.first.start()->at("y"), 0.1);
    }
}

TEST(coord_run_one_slow_transaction_does_not_set_the_measured_line) {
    // a busy PC stretches one transaction now and then: the measurement takes the median, not that one
    Rig rig({50.0, 50.0}, 0.0);
    rig.latency["SIM1"]->full_read_once = 0.2;
    const LineTiming timing = rig.xy->timing({}, true);
    REQUIRE(timing.measured_write_s.has_value());
    CHECK(*timing.measured_write_s < 0.1);
    CHECK_EQ(rig.latency["SIM1"]->full_read_once.load(), 0.0);  // the slow one was taken
}

TEST(coord_run_one_run_at_a_time) {
    Rig rig({85.0, 75.0});
    rig.clear();
    std::optional<CoordinatedResult> first;
    ec_test::Thread worker(  // a run that fails anyway is this test's failure, not a crash
        [&] { first = rig.xy->run(Path::circle(kCenter, 10.0, 0.0, 0.5), 15, 0.3, step_options(), tolerant()); });
    const double give_up = now_s() + 5.0;
    while (!rig.sx->moving() && !rig.sy->moving() && now_s() < give_up) sleep_s(0.002);
    CHECK_THROWS(rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options()), EcNotReadyError);
    worker.join();
    REQUIRE(first.has_value());
    CHECK(first->completed);
    // only the first run's frames went out: one per step that moves x
    int planned = 0;
    for (const auto& sp : first->plan.speeds)
        if (sp[0] > 0) ++planned;
    CHECK_EQ(static_cast<int>(moves(*rig.lines["SIM1"]).size()), planned);
}

TEST(coord_run_axes_held_by_the_path_must_be_ready_too) {
    Rig rig({85.0, 75.0, 20.0});
    CoordinatedGroup xyz = rig.cell->coordinated({{"x", "SIM1:1"}, {"y", "SIM2:1"}, {"z", "SIM3:1"}});
    Axis& z = (*rig.cell)["SIM3:1"];
    const std::vector<std::pair<std::function<void()>, std::function<void()>>> spoils = {
        {[&] { z.servo(false); }, [&] { z.servo(true); }},
        {[&] { z.pause(); }, [&] { z.resume(); }},
    };
    for (const auto& kv : spoils) {
        kv.first();
        rig.clear();
        CHECK_THROWS(xyz.circle(PlanePoint{{"x", 75.0}, {"y", 75.0}, {"z", 20.0}}, 10.0, 20, 0.3, {}, step_options()),
                     EcNotReadyError);  // z held at 20
        CHECK(moves(*rig.lines["SIM1"]).empty());
        CHECK(moves(*rig.lines["SIM2"]).empty());
        kv.second();
    }
}

TEST(coord_run_the_status_monitors_are_held_during_a_run) {
    Rig rig({85.0, 75.0});
    const auto mx = (*rig.cell)["SIM1:1"].start_monitor();
    const auto my = (*rig.cell)["SIM2:1"].start_monitor();
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0, 0.0, 0.5), 25, 0.3, step_options());
    const long bx = mx->polls(), by = my->polls();
    const CoordinatedResult result = rig.xy->run_plan(p, tolerant());
    CHECK(result.completed);
    CHECK(mx->polls() - bx <= 3);  // one in flight at each end
    CHECK(my->polls() - by <= 3);
    CHECK(mx->running());  // running again afterwards
    CHECK(my->running());
    (*rig.cell)["SIM1:1"].stop_monitor();
    (*rig.cell)["SIM2:1"].stop_monitor();
}

TEST(coord_run_a_failure_while_recording_a_port_error_still_fails_the_run) {
    // x's frame fails, and recording that error fails too (out of memory, once): the run still ends in an error the
    // caller sees, after the group stop - never a thread that takes the process down, never a completed result
    Rig rig({50.0, 50.0});
    Hooks hooks;
    hooks->before_frame = [](const std::string& axis) {
        if (axis == "x") throw std::runtime_error("test: the frame failed");
    };
    hooks->on_port_error = [once = std::make_shared<std::atomic<bool>>(false)] {
        if (!once->exchange(true)) throw std::bad_alloc();
    };
    const auto err = motion_error([&] { rig.xy->line_to({{"x", 80.0}, {"y", 30.0}}, 40, 0.3); });
    REQUIRE(err.has_value());
    CHECK(!err->result().completed);
    CHECK(contains(err->what(), "could not be recorded"));
    CHECK(contains(err->result().error.value_or(""), "could not be recorded"));
    REQUIRE(rig.xy->last_result().has_value());
    CHECK(!rig.xy->last_result()->completed);
    CHECK(stood_still({rig.sx, rig.sy}));
}

// ============================================================================================ a slow line

TEST(coord_slow_the_measured_line_sets_the_shortest_step) {
    Rig rig({85.0, 75.0}, 0.03);  // replies 30 ms late, as hardware answers (tens of ms, VERIFIED in the Python edition)
    const LineTiming timing = rig.xy->timing({}, true);
    CHECK(timing.read_s() > 0.03);
    CHECK(timing.min_step_s() > 1.5 * timing.model().min_step_s());
    PlanOptions o;
    o.chord_error = 0.05;
    CHECK_THROWS(rig.xy->plan(Path::circle(kCenter, 10.0), 40, 0.3, o), SoftLimitError);  // fine on a fast line
}

TEST(coord_slow_an_alarm_stops_the_group_when_no_read_fits_between_frames) {
    // steps (~49 ms, min_step_s forced) shorter than a frame plus a read on this line: no read fits in the slack, only
    // the reads forced every sample bound see the alarm (without them x would finish the circle). A late frame is not
    // the subject here: late_abort_s keeps a busy PC from ending the run first.
    Rig rig({85.0, 75.0}, 0.03);
    const MotionPlan p = rig.xy->plan(Path::circle(kCenter, 10.0), 40, 0.3, step_options(0.045));
    const LineTiming line = rig.xy->timing({}, true);
    CHECK(p.shortest_step_s() < line.write_s() + line.read_s());
    std::atomic<double> mark{0.0}, mark_end{0.0};  // the alarm took effect between these two
    ec_test::Thread alarm([&] {  // joined however the run ends (motion_error lets other exceptions through)
        const double give_up = now_s() + 5.0;
        while (!rig.sx->moving() && now_s() < give_up) sleep_s(0.002);
        sleep_s(0.4);  // well into the circle (about 1.7 s at 40 mm/s)
        mark = now_s();
        rig.sy->trigger_alarm(0x0C0);
        mark_end = now_s();
    });
    const RunOptions ro = tolerant();
    const auto err = motion_error([&] { rig.xy->run_plan(p, ro); });
    alarm.join();
    REQUIRE(err.has_value());
    CHECK(err->axis() == std::optional<std::string>("y"));
    CHECK(contains(err->result().error.value_or(""), "alarm"));
    // the run's sample bound follows the line measurement the run itself made, which a busy PC stretches at its own
    // moment
    const LineTiming ran = measured_by_the_run(rig, line);
    const double bound = std::max(kSampleBoundS, 2 * ran.command_period_s());
    REQUIRE(!err->result().samples.at("y").empty());  // the read that saw the alarm
    const double seen_at = err->result().samples.at("y").back().first;  // s from the run's start
    for (const auto& kv : err->result().samples) {  // the forced reads: every axis read within the bound
        // an axis counts as read at the start of the run (its preflight read); a run that the alarm ended before an
        // axis's first forced read was due (a long bound on a busy PC) has nothing more to show for it
        if (seen_at > bound + p.shortest_step_s() + 0.3) CHECK(!kv.second.empty());
        double gap = 0.0, last = 0.0;
        for (const auto& t : kv.second) {
            gap = std::max(gap, t.first - last);
            last = t.first;
        }
        CHECK(gap < 2 * bound + 2 * ran.read_s() + err->result().worst_late_s + 0.1);
    }
    const std::optional<double> halt = first_halt(*rig.lines["SIM1"]);
    REQUIRE(halt.has_value());
    // alarm -> y's next forced read (the bound plus a step and a frame) -> x's frame in flight -> x's STOP frame
    const double allowed = bound + p.shortest_step_s() + 3 * ran.read_s() + err->result().worst_late_s + 0.3 +
                           (mark_end.load() - mark.load());  // + a busy PC's pause of this thread before the alarm
    CHECK(*halt - mark.load() < allowed);
    CHECK(stood_still({rig.sx}));
    CHECK(std::fabs(rig.sx->position_mm() - 85.0) > 0.5);  // x stopped short of the end
}

TEST(coord_slow_a_plan_made_moments_before_is_not_refused_by_a_busy_moment) {
    // run_plan measures the line again; a measurement stretched by a busy PC must not refuse a plan this group made on
    // this line (a plan made for another line still is: coord_cell_a_plan_for_a_faster_line_is_refused)
    Rig rig({85.0, 75.0}, 0.03);
    // a loaded PC stretches the plan's own measurement too: then the plan is made at the speed that line allows
    std::optional<MotionPlan> made;
    double speed = 12.0;
    for (int attempt = 0; attempt < 4 && !made; ++attempt) {
        try {
            made = rig.xy->plan(Path::circle(kCenter, 10.0, 0.0, 0.25), speed, 0.3);
        } catch (const InfeasiblePathError& e) {
            if (!e.max_speed() || !(*e.max_speed() > 0.0)) throw;
            speed = 0.9 * *e.max_speed();
        }
    }
    REQUIRE(made.has_value());
    const MotionPlan& p = *made;
    REQUIRE(p.timing && p.timing->measured_write_s);
    CHECK(*p.timing->measured_write_s > 0.03);
    // the full status reads (the measured frame time) slower now by at least the plan's shortest step: the new
    // minimum step alone would refuse it, whatever the load was when the plan was made
    const double extra = std::max(0.1, p.shortest_step_s());
    for (auto& kv : rig.latency) kv.second->full_read_extra = extra;
    CHECK(kMeasureTolerance * rig.xy->timing({}, true).min_step_s() > p.shortest_step_s());
    const CoordinatedResult result = rig.xy->run_plan(p, tolerant());  // a late frame is not the subject here
    CHECK(result.completed);
    for (auto& kv : rig.latency) kv.second->full_read_extra = 0.0;
}

// ============================================================================================ cells and one shared line

TEST(coord_cell_naming_forms) {
    Rig rig({10.0, 20.0, 30.0});
    Cell& cell = *rig.cell;
    CHECK(cell.coordinated({{"x", "SIM1:1"}, {"y", "SIM3:1"}}).names() == (std::vector<std::string>{"x", "y"}));
    CHECK(cell.coordinated({{"a", "SIM2:1"}}).names() == (std::vector<std::string>{"a"}));
    CHECK(cell.coordinated_names({"SIM1:1", "SIM2:1"}).names() == (std::vector<std::string>{"SIM1:1", "SIM2:1"}));
    CHECK(cell.coordinated().names() == cell.names());
    const CoordinatedGroup group = cell.coordinated({{"x", "SIM1:1"}, {"y", "SIM2:1"}});
    CHECK(group.axis_ptr("y") == cell.axis_ptr("SIM2:1"));
    CHECK(group.cell() == &cell);
    CHECK(contains(group.to_string(), "y: SIM2:1"));
    CHECK_THROWS(cell.coordinated({{"x", "SIM1:1"}, {"y", "SIM1:1"}}), std::invalid_argument);
    CHECK_THROWS(cell.coordinated({{"x", "SIM9:1"}}), std::out_of_range);
    CHECK_THROWS(group.axis("z"), std::out_of_range);
}

TEST(coord_cell_two_axes_on_one_port_and_a_standalone_group) {
    SimulatedUnitOptions u;
    u.position_mm = 73.95;
    auto line = SimLine::with_slaves({1, 2}, u, "L");
    auto bus = std::make_shared<ModbusRtu>(line);
    AxisOptions ao;
    ao.poll_interval_s = 0.005;
    CoordinatedGroup group({{"x", std::make_shared<Axis>(bus, 1, ao)}, {"y", std::make_shared<Axis>(bus, 2, ao)}});
    CHECK_EQ(group.timing().axes_on_port, 2);
    CHECK_NEAR(group.timing().min_step_s(), (LineTiming{115200, 2}.min_step_s()), 1e-12);
    PlanOptions blend;
    blend.blend = 0.5;
    const CoordinatedResult result = group.run(Path::rectangle({73.95, 73.95}, 10, 5), 40, 0.3, blend);
    CHECK(result.completed);
    CHECK_EQ(result.plan.steps(), 4);
    CHECK_NEAR(line->unit(1)->position_mm(), 73.95, 0.1);
    CHECK_NEAR(line->unit(2)->position_mm(), 73.95, 0.1);
    CHECK(result.max_deviation < 0.6);  // blended corners cut inside by up to about the blend
    // a streamed circle with both axes on the one port: they take turns, one frame apart
    group.line_to({{"x", 85.0}, {"y", 75.0}}, 60, 0.5);
    Truth truth({line->unit(1), line->unit(2)});
    const CoordinatedResult circle =
        group.circle(kCenter, 10.0, 15, 0.3, turns(0.5), step_options(LineTiming{115200, 2}.min_step_s()), tolerant());
    const double deviation = truth.deviation(circle.plan.points);
    CHECK(circle.completed);
    CHECK(deviation < 0.3 + 15 * (circle.worst_late_s + circle.worst_skew_s + truth.spread()));
    const CellResults<Status> stopped = group.stop();
    CHECK(stopped.names == (std::vector<std::string>{"x", "y"}));
    CHECK_EQ(stopped.results.size(), std::size_t(2));
    bus->close();
}

TEST(coord_cell_a_plan_for_a_faster_line_is_refused) {
    // a pure plan with the default timing (one axis per port), run on two axes sharing one port
    SimulatedUnitOptions u;
    u.position_mm = 73.95;
    auto line = SimLine::with_slaves({1, 2}, u, "L");
    auto bus = std::make_shared<ModbusRtu>(line);
    AxisOptions ao;
    ao.poll_interval_s = 0.005;
    CoordinatedGroup group({{"x", std::make_shared<Axis>(bus, 1, ao)}, {"y", std::make_shared<Axis>(bus, 2, ao)}});
    group.line_to({{"x", 85.0}, {"y", 75.0}}, 60, 0.5);
    PlanOptions o;
    o.limits = {{"x", AxisRange::make(0, 150, 700, 1.0)}, {"y", AxisRange::make(0, 150, 700, 1.0)}};
    const MotionPlan made = plan(Path::circle(kCenter, 10.0), 40, 0.3, o);
    CHECK(made.shortest_step_s() < group.timing().min_step_s());
    line->clear_requests();
    try {
        group.run_plan(made);
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(contains(e.what(), "this line carries now"));
    }
    CHECK(moves(*line, 1).empty());
    CHECK(moves(*line, 2).empty());
    bus->close();
}

TEST(coord_cell_a_link_fault_follows_the_cells_stop_on_fault) {
    // z is outside the group and moving on its own: the cell's stop_on_fault stops it too
    std::mutex m;
    std::vector<std::string> faults;
    CellOptions o = open_options();
    o.on_fault = [&](const std::string& name, const std::string&, const std::string&) {
        std::lock_guard<std::mutex> l(m);
        faults.push_back(name);
    };
    Rig rig({85.0, 75.0, 20.0}, -1.0, o);
    const auto z = rig.lines["SIM3"]->unit(1);
    MoveOptions mo;
    mo.speed = 10;
    mo.accel_g = 0.3;
    mo.wait = false;
    (*rig.cell)["SIM3:1"].move_to(140.0, mo);
    std::optional<CoordinatedMotionError> err;
    {
        WhenMoving unplug({rig.sx, rig.sy}, 0.3, [&] { rig.lines["SIM2"]->unplug(); });
        err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant()); });
    }
    REQUIRE(err.has_value());
    CHECK(stood_still({z}));
    {
        std::lock_guard<std::mutex> l(m);
        CHECK(faults == (std::vector<std::string>{"SIM2:1"}));
    }
    CHECK(z->position_mm() < 139.0);
    CHECK(has_event(*z, "stop"));
}

TEST(coord_cell_a_link_fault_in_a_sync_segment_stops_the_group_before_a_slow_on_fault) {
    // y's unit is powered off in the middle of one long sync segment: only the status reads see it (three failed in a
    // row). x's STOP must go out at once - before the cell's fault handling, whose on_fault takes 0.5 s here
    std::mutex m;
    std::vector<std::pair<double, std::string>> calls;
    CellOptions o = open_options();
    o.on_fault = [&](const std::string& name, const std::string&, const std::string&) {
        {
            std::lock_guard<std::mutex> l(m);
            calls.emplace_back(now_s(), name);
        }
        sleep_s(0.5);  // a log, a dialog
    };
    Rig rig({10.0, 10.0}, 0.01, o);
    std::atomic<double> mark{0.0};
    ec_test::Thread fault([&] {  // joined however the run ends (motion_error lets other exceptions through)
        const double give_up = now_s() + 5.0;
        while (!rig.sx->moving() && now_s() < give_up) sleep_s(0.002);
        sleep_s(0.3);
        mark = now_s();
        rig.sy->power_off();
    });
    const auto err = motion_error([&] { rig.xy->line_to({{"x", 140.0}, {"y", 60.0}}, 40, 0.3); });  // about 3.6 s
    fault.join();
    REQUIRE(err.has_value());
    CHECK(err->axis() == std::optional<std::string>("y"));
    const std::optional<double> halt = first_halt(*rig.lines["SIM1"]);
    REQUIRE(halt.has_value());
    CHECK(*halt - mark.load() < 1.0);  // 0.4 s on an idle PC
    std::lock_guard<std::mutex> l(m);
    REQUIRE(calls.size() == 1);
    CHECK_EQ(calls[0].second, std::string("SIM2:1"));
    CHECK(*halt < calls[0].first);  // the STOP was on the wire before on_fault ran
    CHECK(stood_still({rig.sx}));
    CHECK(rig.sx->position_mm() < 100.0);
}

TEST(coord_cell_the_first_stop_threads_end_before_the_run_returns) {
    // y's port is unplugged: x's first STOP frame goes out from a thread of its own, which lingers 1 s after it here;
    // the run returns only once that thread ended (no thread of the run outlives it)
    Rig rig({85.0, 75.0});
    Hooks hooks;
    const auto started = std::make_shared<std::atomic<int>>(0);
    const auto finished = std::make_shared<std::atomic<int>>(0);
    hooks->after_stop_frame = [started, finished](const std::string&) {
        ++*started;
        sleep_s(1.0);
        ++*finished;
    };
    std::optional<CoordinatedMotionError> err;
    int ended_at_return = -1;
    {
        WhenMoving unplug({rig.sx, rig.sy}, 0.3, [&] { rig.lines["SIM2"]->unplug(); });
        err = motion_error([&] { rig.xy->circle(kCenter, 10.0, 15, 0.3, {}, step_options(), tolerant()); });
        ended_at_return = finished->load();
    }
    REQUIRE(err.has_value());
    CHECK(err->axis() == std::optional<std::string>("y"));
    CHECK_EQ(started->load(), 1);
    CHECK_EQ(ended_at_return, started->load());
    CHECK(has_event(*rig.sx, "stop"));
    CHECK(stood_still({rig.sx}));
}

TEST(coord_cell_monitors_of_other_axes_on_a_shared_port_keep_polling) {
    // z (SIM1:2) is outside the group but on x's port: the run holds only its own axes' monitors
    SimulatedUnitOptions u1, u2, u3;
    u1.slave = 1;
    u1.position_mm = 85.0;
    u2.slave = 2;
    u2.position_mm = 50.0;
    u3.slave = 1;
    u3.position_mm = 75.0;
    std::map<std::string, std::shared_ptr<SimLine>> lines = {
        {"SIM1", std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{
                                               std::make_shared<SimulatedUnit>(u1), std::make_shared<SimulatedUnit>(u2)},
                                           "SIM1")},
        {"SIM2", std::make_shared<SimLine>(std::vector<std::shared_ptr<SimulatedUnit>>{std::make_shared<SimulatedUnit>(u3)},
                                           "SIM2")},
    };
    Cell cell = Cell::open(lines, {}, open_options());
    CoordinatedGroup xy = cell.coordinated({{"x", "SIM1:1"}, {"y", "SIM2:1"}});
    const auto own_x = cell["SIM1:1"].start_monitor();
    const auto own_y = cell["SIM2:1"].start_monitor();
    const auto other = cell["SIM1:2"].start_monitor();
    const MotionPlan p = xy.plan(Path::circle(kCenter, 10.0, 0.0, 0.25), 15, 0.3, step_options());
    const long bx = own_x->polls(), by = own_y->polls(), bo = other->polls();
    const CoordinatedResult result = xy.run_plan(p, tolerant());
    CHECK(result.completed);
    CHECK(own_x->polls() - bx < 5);  // one in flight at each end
    CHECK(own_y->polls() - by < 5);
    CHECK(other->polls() - bo > 20);  // a poll every few ms over ~1.3 s
    cell.close();
}
