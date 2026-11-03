// Coordinated motion: CoordinatedGroup and the runners (coord.hpp). Port of the Python edition's coord.py
// (CoordinatedGroup, _Run, _sleep_until, _link_fault, _check_status, the sync and stream port workers, every stop path)
// and Cell.coordinated. The behaviour is VERIFIED in the Python edition with one real axis (the EC-GRBP8M gripper) and
// a simulated second one where coord.hpp says so; this C++ port is UNVERIFIED on hardware (simulator only).
//
// Hooks into the axis (src/axis_impl.hpp): Axis::Impl::note_stop() runs before every STOP, pause or servo OFF and calls
// the run's StopHook (the run sends no frame after it), stop_count is compared before every frame, stop_coil_on /
// stop_pause_on refuse a run on an axis with a STOP held. The frames go out with ModbusRtu::write_registers(...,
// WriteOptions{0 retries, a bounded wait}) inside the bus's STOP-first gate (modbus.hpp detail::SendGate): under the bus
// lock, right before the frame goes to the wire, a run that is aborting, a STOP to the unit (or a broadcast one) since
// the run began, or a frame later than its limit refuses it - nothing sent. The status samples go out with
// read_registers(..., ReadOptions{same}).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/coord.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "bus_stop_timing.hpp"
#include "monotonic_cv.hpp"
#include "threads.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace coord {

namespace {

using AxisImpl = ::iai_actuator_driver::detail::AxisAccess::Impl;
using Settle = ::iai_actuator_driver::detail::Settle;
using StopHookBase = ::iai_actuator_driver::detail::StopHook;
using SendGate = ::iai_actuator_driver::detail::SendGate;
using MonotonicCv = ::iai_actuator_driver::detail::MonotonicCv;
using StopMark = ::iai_actuator_driver::detail::StopMark;

constexpr const char* kLog = "iai_actuator_driver.coord";
constexpr double kPi = 3.14159265358979323846;

AxisImpl& impl_of(Axis& axis) { return ::iai_actuator_driver::detail::AxisAccess::impl(axis); }

std::string g6(double v) { return util::format("%g", v); }
std::string f(double v, int decimals) { return util::fixed(v, decimals); }

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    if (n == 0) return 0.0;
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

std::string bracket(const std::vector<std::string>& names) { return "[" + util::join(names, ", ") + "]"; }

/// The steady clock and the system clock read together, when a step of a stream began.
struct ClockMark {
    double steady = 0.0;
    double wall = 0.0;
};

ClockMark clock_mark() {
    using wall_clock = std::chrono::system_clock;
    return {steady_now_s(), std::chrono::duration<double>(wall_clock::now().time_since_epoch()).count()};
}

/// The end of a LateFrame message: how far the system clock moved against the steady clock since `since` (wall
/// delta - steady delta): about 0 for a late PC or line; about minus the lateness when a backward step of the system
/// clock delayed the frame; positive after a forward jump (a suspend, a time sync ahead).
std::string clock_drift(const ClockMark& since) {
    const ClockMark now = clock_mark();
    const long ms = std::lround(((now.wall - since.wall) - (now.steady - since.steady)) * 1000);
    return util::format("; over the step the system clock moved %+ld ms against the steady clock", ms);
}

// ------------------------------------------------------------------------------------------ internal errors

/// A stream frame reached the wire later than the abort threshold.
class LateFrame : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};
/// A stop, pause or servo OFF was sent to an axis of the run from outside it (or the group's stop() was called).
class StoppedFromOutside : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};
/// An axis went unread for longer than the run allows: a fault on it would go unseen.
class NoStatus : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};
/// Another port failed or the run was stopped: this port ends quietly.
struct Aborted {};
/// An error of a port worker with the axis it happened on and whether it is a link fault (the Python edition's
/// coord_axis / coord_link attributes).
struct Tagged {
    std::string axis;
    bool link = false;
    std::exception_ptr error;
};

std::string type_name(const std::exception& e) {
    if (dynamic_cast<const LateFrame*>(&e)) return "LateFrame";
    if (dynamic_cast<const StoppedFromOutside*>(&e)) return "StoppedFromOutside";
    if (dynamic_cast<const NoStatus*>(&e)) return "NoStatus";
    return ::iai_actuator_driver::detail::error_type_name(e);
}

std::string describe(const std::exception_ptr& error) {
    try {
        std::rethrow_exception(error);
    } catch (const std::exception& e) {
        return type_name(e) + ": " + e.what();
    } catch (...) {
        return "unknown error";
    }
}

/// A timeout, a bad CRC or a mismatched frame: a lost answer (TransportError is a ModbusError too, but never this).
bool lost_answer(const std::exception_ptr& error) {
    try {
        std::rethrow_exception(error);
    } catch (const TransportError&) {
        return false;
    } catch (const ModbusTimeout&) {
        return true;
    } catch (const ModbusCrcError&) {
        return true;
    } catch (const ModbusFrameError&) {
        return true;
    } catch (...) {
        return false;
    }
}

bool is_transport_error(const std::exception_ptr& error) {
    try {
        std::rethrow_exception(error);
    } catch (const TransportError&) {
        return true;
    } catch (...) {
        return false;
    }
}

// ------------------------------------------------------------------------------------------ the run's shared state

/// A barrier for the sync ports that can be broken (every waiter, and every later wait, then fails).
class Barrier {
public:
    explicit Barrier(int parties) : parties_(std::max(1, parties)) {}
    /// False when broken.
    bool wait() {
        std::unique_lock<std::mutex> lock(m_);
        if (broken_) return false;
        const long gen = generation_;
        if (++count_ == parties_) {
            count_ = 0;
            ++generation_;
            cv_.notify_all();
            return true;
        }
        cv_.wait(lock, [&] { return generation_ != gen || broken_; });
        return generation_ != gen;
    }
    void abort() {
        std::lock_guard<std::mutex> lock(m_);
        broken_ = true;
        cv_.notify_all();
    }

private:
    std::mutex m_;
    MonotonicCv cv_;
    int parties_;
    int count_ = 0;
    long generation_ = 0;
    bool broken_ = false;
};

struct Frame {
    int step = 0;
    std::string axis;
    double wire = 0.0;  ///< on the wire at (s from t0)
    double late = 0.0;
    bool operator<(const Frame& o) const {
        if (step != o.step) return step < o.step;
        if (axis != o.axis) return axis < o.axis;
        if (wire != o.wire) return wire < o.wire;
        return late < o.late;
    }
};

struct Failure {
    std::optional<std::string> axis;
    std::exception_ptr error;
    bool link = false;
};

/// The shared state of one run: abort flag, port send locks, frames and samples, and the hook every stop path of an
/// axis of the run calls (halt).
struct Run {
    Run(const MotionPlan& plan_, std::vector<std::vector<std::string>> ports_, const LineTiming& timing_,
        std::optional<double> late_abort, std::optional<double> timeout)
        : plan(plan_),
          ports(std::move(ports_)),
          timing(timing_),
          late_abort_s(late_abort),
          timeout_s(timeout),
          barrier(static_cast<int>(ports.size())),
          frame_wait_s(std::max(kWaitMinS, kWaitFactor * timing_.write_s())),
          read_wait_s(std::max(kWaitMinS, kWaitFactor * timing_.read_s())),
          sample_bound_s(std::max(kSampleBoundS, 2 * timing_.command_period_s())) {
        for (const auto& a : plan.axes) samples[a];
    }

    MotionPlan plan;
    std::vector<std::vector<std::string>> ports;
    LineTiming timing;
    std::optional<double> late_abort_s;
    std::optional<double> timeout_s;
    std::mutex lock;
    std::atomic<bool> abort{false};
    std::mutex abort_mutex;
    MonotonicCv abort_cv;
    std::mutex changed_mutex;
    MonotonicCv changed_cv;
    bool changed = false;              ///< (changed_mutex) a port failed or ended: the main thread looks at once
    std::atomic<bool> lost_error{false};  ///< a port failed and recording its error failed too: the run still fails
    Barrier barrier;
    std::vector<Failure> errors;                                            ///< (lock)
    std::map<std::string, std::vector<std::pair<double, double>>> samples;  ///< each axis written by its port only
    std::vector<Frame> frames;                                              ///< (lock)
    std::atomic<int> skipped{0};
    std::map<std::string, int> failures;                  ///< (lock)
    std::map<std::string, std::exception_ptr> read_errors;  ///< (lock)
    std::set<std::pair<int, int>> woke;                   ///< (lock) sync: (segment, round) in which a port woke an axis
    std::map<std::string, long> stop_counts;              ///< per axis: its stop counter when run / run_plan began
    std::map<std::string, StopMark> stop_marks;           ///< per axis: its bus's stop mark when run / run_plan began
    std::vector<std::shared_ptr<StopHookBase>> hooks;     ///< the hooks set on the axes (unregister compares them)
    double t0 = 0.0;
    const double frame_wait_s;
    const double read_wait_s;
    const double sample_bound_s;

    bool aborted() const { return abort.load(); }
    /// Wait up to `seconds`; true when the run is aborting.
    bool wait_abort(double seconds) {
        std::unique_lock<std::mutex> l(abort_mutex);
        return abort_cv.wait_for(l, std::chrono::duration<double>(std::max(0.0, seconds)),
                                 [&] { return abort.load(); });
    }
    void set_abort() {
        {
            std::lock_guard<std::mutex> l(abort_mutex);
            abort = true;
        }
        abort_cv.notify_all();
        barrier.abort();
    }
    void mark_changed() {
        {
            std::lock_guard<std::mutex> l(changed_mutex);
            changed = true;
        }
        changed_cv.notify_all();
    }
    void wait_changed(double seconds) {
        std::unique_lock<std::mutex> l(changed_mutex);
        changed_cv.wait_for(l, std::chrono::duration<double>(seconds), [&] { return changed; });
        changed = false;
    }
    bool has_errors() {
        if (lost_error.load()) return true;
        std::lock_guard<std::mutex> l(lock);
        return !errors.empty();
    }
    std::optional<Failure> first_error() {
        std::lock_guard<std::mutex> l(lock);
        if (errors.empty()) return std::nullopt;
        return errors.front();
    }
    /// Set the abort flag and return at once (a STOP never waits for a port): afterwards no move frame goes out - every
    /// frame passes the STOP-first gate under its bus lock right before the wire (write_frame), which refuses it once
    /// the flag is set, and a frame already on the wire is wholly before any STOP (the STOP waits for the bus).
    void stop_sending() { set_abort(); }
    void fail(std::optional<std::string> axis, std::exception_ptr error, bool link) {
        {
            std::lock_guard<std::mutex> l(lock);
            errors.push_back({std::move(axis), error, link});
        }
        mark_changed();
        stop_sending();
    }
    /// A stop from outside the run: recorded (unless the run is already ending), then nothing more is sent.
    void halt(std::optional<std::string> axis, const std::string& reason) {
        {
            std::lock_guard<std::mutex> l(lock);
            if (!abort.load()) errors.push_back({std::move(axis), std::make_exception_ptr(StoppedFromOutside(reason)), false});
        }
        mark_changed();
        stop_sending();
    }
};

/// The hook set on each axis of a run: a stop, pause or servo OFF of that axis ends the run first.
struct AxisStopHook final : StopHookBase {
    AxisStopHook(std::weak_ptr<Run> run_, std::string name_) : run(std::move(run_)), name(std::move(name_)) {}
    void external_stop() override {
        if (auto r = run.lock())
            r->halt(name, "a stop, pause or servo OFF was sent to this axis from outside the coordinated move");
    }
    std::weak_ptr<Run> run;
    std::string name;
};

std::mutex& runs_mutex() {  // guards Axis::Impl::coord_run of every axis (one coordinated run per axis)
    static std::mutex m;
    return m;
}

void sleep_until(double deadline, Run& run) {
    deadline += detail::coord_test_hooks().schedule_delay_s;
    for (;;) {
        const double remaining = deadline - now_s();
        if (run.aborted()) throw Aborted{};
        if (remaining <= 0) return;
        if (remaining > 0.003)
            run.wait_abort(remaining - 0.002);
        else
            std::this_thread::yield();
    }
}

/// A sample during the run: alarm, EMG, servo off, pause (STP) or a home return ends it; an automatic servo OFF too
/// while asof_ok is false (the axis still has frames to come: the driver does not rely on a 0x9900 frame to switch the
/// servo back on, UNVERIFIED on an EC).
void check_status(const std::string& axis, const Status& st, bool asof_ok) {
    if (st.alarm)
        throw EcAlarmError(st.alarm_code, util::format("axis %s: alarm 0x%03x during the coordinated move", axis.c_str(),
                                                       st.alarm_code));
    if (st.emergency_stop) throw EcNotReadyError("axis " + axis + ": emergency stop during the coordinated move");
    if (!st.servo_on) {
        if (!st.auto_servo_off().value_or(false))
            throw EcNotReadyError("axis " + axis + ": servo turned off during the coordinated move");
        if (!asof_ok)
            throw EcNotReadyError("axis " + axis + ": automatic servo OFF while the path still has frames for it");
    }
    if (st.paused()) throw EcNotReadyError("axis " + axis + ": paused (STP) during the coordinated move");
    if (st.homing()) throw EcNotReadyError("axis " + axis + ": a home return started during the coordinated move");
}

/// One lean status read (0x9000 x8) with no retry and a bounded wait; PNOW in the driver's 0.01 counts.
Status lean_read(AxisImpl& a, double max_wait) {
    ReadOptions o;
    o.retries = 0;
    o.max_wait_s = std::max(0.001, max_wait);
    std::vector<std::uint16_t> words = a.bus->read_registers(a.slave, reg::kMonitorBase, kLeanStatusWords, o);
    if (const int factor = a.bus->line()->wire_factor(a.slave); factor != 1 && words.size() >= 2) {
        const std::int64_t wire = reg::from_i32_words(words[0], words[1]);
        const std::int64_t counts = (wire >= 0 ? wire + factor / 2 : wire - factor / 2) / factor;
        const std::uint32_t raw = static_cast<std::uint32_t>(static_cast<std::int32_t>(counts));
        words[0] = static_cast<std::uint16_t>(raw >> 16);
        words[1] = static_cast<std::uint16_t>(raw & 0xFFFF);
    }
    return Status::from_registers(words);
}

/// One 0x9900 frame (the whole block: target, band, speed, accel; absolute), no retry inside the bus, the echo waited
/// for at most max_wait. A STOP left on the axis is never released here (Axis::Impl::send_direct would). The frame goes
/// out inside the STOP-first gate with the run's stop mark of the axis (`expected`) and `check` (abort, outside STOP,
/// lateness): run under the bus lock right before the wire, and while the frame waits for the bus; whatever it throws
/// refuses the frame, nothing sent. The target is recorded only for a frame that went past the gate.
void write_frame(AxisImpl& a, const DirectMove& move, double max_wait, const StopMark& expected,
                 std::function<void()> check) {
    DirectMove wire = move;  // target and band in the unit's own counts (x10 on a 0.001 mm RCON unit; line.hpp)
    const int factor = a.bus->line()->wire_factor(a.slave);
    wire.pcmd *= factor;
    wire.inp *= static_cast<std::uint32_t>(factor);
    WriteOptions o;
    o.retries = 0;
    o.max_wait_s = max_wait;
    auto record = [&] {
        a.motion_pending = true;
        {
            std::lock_guard<std::mutex> lock(a.mutex);
            a.last_move = move;
            a.last_target = reg::from_counts(move.pcmd);
        }
        recovery::hooks::note_target(a.bus, a.slave, reg::from_counts(move.pcmd));
    };
    const SendGate gate(*a.bus, a.slave, expected, std::move(check));
    try {
        a.bus->write_registers(a.slave, reg::kDirectMoveBase, wire.words(), o);
    } catch (...) {
        if (gate.passed() > 0) record();  // its echo was lost: it may have arrived
        throw;
    }
    record();
}

}  // namespace

// ============================================================================================ the group's state

struct CoordinatedGroup::State {
    AxisList axes;
    Cell* cell = nullptr;
    std::map<std::string, std::string> cell_names;
    double band = 0.1;
    mutable std::mutex mutex;
    std::optional<CoordinatedResult> last_result;  ///< (mutex)
    std::shared_ptr<Run> active_run;               ///< (mutex)

    std::vector<std::string> names() const {
        std::vector<std::string> out;
        for (const auto& kv : axes) out.push_back(kv.first);
        return out;
    }
    std::shared_ptr<Axis> ptr(const std::string& name) const {
        for (const auto& kv : axes)
            if (kv.first == name) return kv.second;
        throw std::out_of_range("axis '" + name + "' is not in this group (axes: " + util::join(names(), ", ") + ")");
    }
    Axis& ax(const std::string& name) const { return *ptr(name); }
    AxisImpl& im(const std::string& name) const { return impl_of(*ptr(name)); }
    void set_result(const CoordinatedResult& r) {
        std::lock_guard<std::mutex> lock(mutex);
        last_result = r;
    }

    void check_names(const std::vector<std::string>& wanted) const {
        std::vector<std::string> unknown;
        for (const auto& n : wanted) {
            bool found = false;
            for (const auto& kv : axes)
                if (kv.first == n) found = true;
            if (!found) unknown.push_back(n);
        }
        if (!unknown.empty())
            throw std::invalid_argument("axis/axes " + bracket(unknown) + " are not in this group (axes: " +
                                        util::join(names(), ", ") + ")");
    }

    /// The names grouped by their bus (a port), in order of appearance.
    std::vector<std::vector<std::string>> ports(const std::vector<std::string>& wanted) const {
        std::vector<std::pair<const ModbusRtu*, std::vector<std::string>>> groups;
        for (const auto& n : wanted) {
            const ModbusRtu* bus = ax(n).bus().get();
            auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& g) { return g.first == bus; });
            if (it == groups.end())
                groups.push_back({bus, {n}});
            else
                it->second.push_back(n);
        }
        std::vector<std::vector<std::string>> out;
        for (auto& g : groups) out.push_back(std::move(g.second));
        return out;
    }

    /// Hold the status monitors of these axes (the run reads its own samples; a poll in flight would delay a frame by
    /// a whole transaction). Other axes' monitors on the same ports keep polling.
    std::vector<AxisMonitor::Suspension> hold_monitors(const std::vector<std::string>& wanted) const {
        std::vector<AxisMonitor::Suspension> held;
        for (const auto& n : wanted) {
            std::shared_ptr<AxisMonitor> mon = ax(n).monitor();
            if (mon && mon->running()) held.push_back(mon->suspend(1.0));
        }
        return held;
    }

    std::map<std::string, AxisRange> ranges(const std::vector<std::string>& wanted) const {
        std::map<std::string, AxisRange> out;
        for (const auto& n : wanted) {
            im(n).motion_stroke();  // the soft limits read (again until a read succeeds); refuses an unknown stroke
            out[n] = AxisRange::of(ax(n).limits());
        }
        return out;
    }

    /// (full status read, lean status read) on one port: per axis the median of kMeasureSamples of each, back to back
    /// as during a run, the slowest axis. The monitors of these axes are held meanwhile.
    std::pair<double, double> measure(const std::vector<std::string>& group) const {
        double full = 0.0, lean = 0.0;
        auto held = hold_monitors(group);
        for (const auto& n : group) {
            Axis& a = ax(n);
            a.read_lean_status();
            std::vector<double> fulls, leans;
            for (int i = 0; i < kMeasureSamples; ++i) {
                double t = now_s();
                a.status();
                fulls.push_back(now_s() - t);
                t = now_s();
                a.read_lean_status();
                leans.push_back(now_s() - t);
            }
            full = std::max(full, median(fulls));
            lean = std::max(lean, median(leans));
        }
        return {full, lean};
    }

    std::vector<std::pair<double, double>> measure_ports(const std::vector<std::vector<std::string>>& groups) const {
        std::vector<std::pair<double, double>> out(groups.size());
        if (groups.size() == 1) {
            out[0] = measure(groups[0]);
            return out;
        }
        std::vector<std::exception_ptr> errors(groups.size());
        std::vector<std::thread> threads;
        threads.reserve(groups.size());
        {
            const util::JoinThreads join(threads);  // also when a later thread cannot start
            for (std::size_t i = 0; i < groups.size(); ++i)
                threads.emplace_back([&, i] {
                    try {
                        out[i] = measure(groups[i]);
                    } catch (...) {
                        errors[i] = std::current_exception();
                    }
                });
        }
        for (const auto& e : errors)
            if (e) std::rethrow_exception(e);
        return out;
    }

    LineTiming timing(const std::vector<std::string>& wanted, bool measure_line) const {
        const auto groups = ports(wanted);
        std::vector<std::pair<double, double>> measured;
        if (measure_line) measured = measure_ports(groups);
        std::optional<LineTiming> worst;
        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto bus = ax(groups[i][0]).bus();
            LineTiming t;
            const std::optional<int> rate = bus->baud_rate();
            t.baud_rate = rate && *rate > 0 ? *rate : 115200;
            t.axes_on_port = static_cast<int>(groups[i].size());
            double response = 0.0;
            for (const auto& n : groups[i]) response = std::max(response, bus->response_delay_ms(ax(n).slave()));
            t.response_ms = response;
            t.gap_s = bus->min_gap_s();
            if (measure_line) {
                t.measured_write_s = measured[i].first;
                t.measured_read_s = measured[i].second;
            }
            if (!worst || t.min_step_s() > worst->min_step_s()) worst = t;
        }
        return *worst;
    }

    Point positions(const std::vector<std::string>& wanted) const {
        Point out;
        for (const auto& n : wanted) out.set(n, ax(n).status().position_mm);
        return out;
    }

    std::vector<std::string> or_all(const std::vector<std::string>& wanted) const {
        return wanted.empty() ? names() : wanted;
    }
};

namespace {

using State = CoordinatedGroup::State;

// ============================================================================================ stops

/// The first STOP frame (Axis::begin_stop) to every axis of `names`, ports in parallel, the axes of a port back to
/// back; waits at most `wait` for them. A full stop must follow (it holds STOP until each axis stands still); the
/// caller joins the returned threads after it, so no thread of a stop outlives it. A thread that cannot start leaves
/// its axes to that full stop.
std::vector<std::thread> first_stop_frames(const State& s, const std::vector<std::string>& names, double wait) {
    struct Shared {
        std::mutex m;
        MonotonicCv cv;
        std::size_t left = 0;
    };
    using Axes = std::vector<std::pair<std::string, std::shared_ptr<Axis>>>;
    auto shared = std::make_shared<Shared>();
    std::vector<Axes> work;
    for (const auto& group : s.ports(names)) {
        Axes axes;
        for (const auto& n : group) axes.emplace_back(n, s.ptr(n));
        work.push_back(std::move(axes));
    }
    const auto after = detail::coord_test_hooks().after_stop_frame;  // a copy: the threads never read the hooks
    shared->left = work.size();
    std::vector<std::thread> threads;
    threads.reserve(work.size());
    for (const auto& axes : work) {
        try {
            threads.emplace_back([shared, axes, after] {
                // the stop latency lines once every axis of the port has its first STOP frame and the caller was told
                // (bus_stop_timing.hpp)
                std::optional<iai_actuator_driver::detail::StopLinesLater> lines_later;
                lines_later.emplace();
                for (const auto& kv : axes) {
                    try {
                        kv.second->begin_stop();
                    } catch (const std::exception& e) {  // the full stop retries and reports it
                        log_debug(kLog, kv.first + ": first STOP frame failed: " + e.what());
                    } catch (...) {
                        log_debug(kLog, kv.first + ": first STOP frame failed");
                    }
                    if (after) {
                        try {
                            after(kv.first);
                        } catch (...) {
                        }
                    }
                }
                try {
                    {
                        std::lock_guard<std::mutex> l(shared->m);
                        --shared->left;
                    }
                    shared->cv.notify_all();
                } catch (...) {  // the caller's wait is bounded
                }
                lines_later.reset();  // logged once the caller can go on to the full stop
            });
        } catch (...) {  // no thread for this port: the full stop that follows stops its axes
            log_warning(kLog, "a first STOP thread could not start (" + describe(std::current_exception()) +
                                  "): the full stop follows");
            std::lock_guard<std::mutex> l(shared->m);
            --shared->left;
        }
    }
    try {
        std::unique_lock<std::mutex> l(shared->m);
        shared->cv.wait_for(l, std::chrono::duration<double>(wait), [&] { return shared->left == 0; });
    } catch (...) {  // the full stop follows at once
    }
    return threads;
}

CellResults<Status> group_stop(State& s, double timeout_s) {
    std::shared_ptr<Run> run;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        run = s.active_run;
    }
    if (run) run->halt(std::nullopt, "the group's stop() was called from outside the coordinated move");
    const std::vector<std::string> names = s.names();
    CellResults<Status> out;
    out.operation = "stop";
    out.names = names;
    bool in_cell = s.cell != nullptr;
    for (const auto& n : names)
        if (!s.cell_names.count(n)) in_cell = false;
    if (in_cell) {
        std::vector<std::string> cell_axes;
        for (const auto& n : names) cell_axes.push_back(s.cell_names.at(n));
        const CellResults<Status> by_cell = s.cell->stop_all(cell_axes, timeout_s, true);
        for (const auto& n : names) {
            const std::string& c = s.cell_names.at(n);
            if (by_cell.results.count(c)) out.results[n] = by_cell.results.at(c);
            if (by_cell.errors.count(c)) out.errors[n] = by_cell.errors.at(c);
            if (by_cell.messages.count(c)) out.messages[n] = by_cell.messages.at(c);
        }
        return out;
    }
    // every axis is held by this stop from before its first STOP frame until its full stop ends: no command of
    // another thread releases its STOP meanwhile (Axis::Impl::group_stops)
    ::iai_actuator_driver::detail::GroupStopHolds holds;
    for (const auto& n : names) holds.add(n, s.ptr(n));
    std::vector<std::thread> stops = first_stop_frames(s, names, timeout_s + 5.0);
    const util::JoinThreads join_stops(stops);  // after the full stop below
    std::mutex m;
    std::vector<std::thread> threads;
    threads.reserve(names.size());
    {
        const util::JoinThreads join(threads);  // also when a later thread cannot start
        for (const auto& n : names) {
            std::shared_ptr<Axis> a = s.ptr(n);
            threads.emplace_back([&, n, a] {
                struct End {
                    ::iai_actuator_driver::detail::GroupStopHolds& holds;
                    const std::string& name;
                    ~End() { holds.end(name); }
                } end{holds, n};
                try {
                    const Status st = a->stop(false, timeout_s);
                    std::lock_guard<std::mutex> l(m);
                    out.results[n] = st;
                } catch (...) {
                    const std::exception_ptr e = std::current_exception();
                    try {
                        std::lock_guard<std::mutex> l(m);
                        out.errors[n] = e;
                        out.messages[n] = describe(e);
                    } catch (...) {
                        log_warning(kLog, n + ": the stop failed and could not be recorded");
                    }
                }
            });
        }
    }
    return out;
}

/// The group stop after a failure or an outside stop. A link fault of an axis of a cell: first the STOP frame to
/// every other axis of the group (ports in parallel; the silent axis left out), then the cell's fault handling (its
/// link check of that axis: on_fault and, with stop_on_fault, every other axis of the cell stopped), then the full
/// stop - the cell's handling never delays the group's STOP frames. Otherwise the full stop at once.
CellResults<Status> stop_after_failure(State& s, Run& run) {
    const std::optional<Failure> first = run.first_error();
    if (s.cell && first && first->axis && s.cell_names.count(*first->axis) && first->error && first->link) {
        std::vector<std::string> others;
        for (const auto& n : s.names())
            if (n != *first->axis) others.push_back(n);
        // held from before the first STOP frames until the group stop below has ended (Axis::Impl::group_stops)
        ::iai_actuator_driver::detail::GroupStopHolds holds;
        for (const auto& n : others) holds.add(n, s.ptr(n));
        std::vector<std::thread> stops = first_stop_frames(s, others, 2 * run.frame_wait_s + 0.1);
        const util::JoinThreads join_stops(stops);  // after the full stop below
        const std::string name = s.cell_names.at(*first->axis);
        try {
            s.cell->check_links({name});
        } catch (...) {  // the group stop below must still go out
            log_warning(kLog, name + ": the cell's fault handling failed: " + describe(std::current_exception()));
        }
        return group_stop(s, 5.0);
    }
    return group_stop(s, 5.0);
}

// ============================================================================================ the port workers

/// One 0x9900 frame, unless the run is ending or the axis was stopped, paused or switched off since the run began
/// (its stop counter, and its bus's stop mark under the bus lock). A lost echo is sent again at once (the frame is
/// idempotent). A frame that would reach the wire after `limit` is refused under the bus lock (a late frame, nothing
/// sent; not a link fault), both attempts lost is a link fault. Returns how late the frame went out against `planned`
/// (when the call began: this bus stamps no per-thread request time). A late frame's message ends with the clock drift
/// since `since` (when the step began).
double send_frame(State& s, Run& run, std::size_t port, int k, const std::string& name, std::int64_t target,
                  double speed, double accel, double planned, std::optional<double> limit, const ClockMark& since) {
    AxisImpl& a = s.im(name);
    DirectMove move;
    move.pcmd = static_cast<std::int32_t>(target);
    move.inp = static_cast<std::uint32_t>(reg::to_counts(run.plan.band));
    move.vcmd = static_cast<std::uint32_t>(std::max<std::int64_t>(1, reg::to_counts(speed, reg::kVelocityScale)));
    move.acmd = static_cast<std::uint16_t>(std::max<std::int64_t>(1, reg::to_counts(accel, reg::kAccelScale)));
    move.ppow = 0;
    move.ctlf = reg::kCtlfAbsolute;
    (void)port;
    int attempt = 0;
    double begin = now_s();
    std::exception_ptr lost;
    const StopMark& expected = run.stop_marks.at(name);  // filled before the port threads start, never changed
    // the STOP-first gate's check (under the bus lock right before the wire, and while the frame waits for the bus)
    auto check = [&run, &a, &name, &expected, &attempt, &lost, &since, k, planned, limit] {
        if (run.aborted()) throw Aborted{};
        if (::iai_actuator_driver::detail::stop_mark(*a.bus, a.slave) != expected)
            throw StoppedFromOutside("axis " + name + ": a STOP went to this unit from outside the coordinated move (to "
                                     "its slave address or broadcast)");
        if (!limit) return;
        const double late = now_s() - planned;
        if (late <= *limit) return;
        if (attempt == 0)
            throw LateFrame("the frame of step " + std::to_string(k) + " would reach the wire " + f(late * 1000, 0) +
                            " ms late (limit " + f(*limit * 1000, 0) +
                            " ms): the PC or the line cannot keep this schedule; not sent" + clock_drift(since));
        throw LateFrame("the frame of step " + std::to_string(k) + " got no valid echo (" + describe(lost) +
                        "); sent again it would reach the wire " + f(late * 1000, 0) + " ms late (limit " +
                        f(*limit * 1000, 0) + " ms); not sent again" + clock_drift(since));
    };
    for (attempt = 0; attempt < 2; ++attempt) {
        if (run.aborted()) throw Aborted{};
        if (a.stop_count.load() != run.stop_counts[name])
            throw Tagged{name, false,
                         std::make_exception_ptr(StoppedFromOutside(
                             "axis " + name + ": stopped, paused or switched off from outside the coordinated move"))};
        begin = now_s();
        const auto& hook = detail::coord_test_hooks().before_frame;
        if (hook) hook(name);
        try {
            write_frame(a, move, run.frame_wait_s, expected, check);
            break;
        } catch (const Aborted&) {
            throw;  // the run is ending: this port ends quietly
        } catch (...) {
            const std::exception_ptr e = std::current_exception();
            if (lost_answer(e)) {
                if (attempt == 0) {
                    lost = e;
                    log_debug(kLog, name + ": the frame of step " + std::to_string(k) + " got no valid echo (" +
                                        describe(e) + "): sent again");
                    continue;
                }
                throw Tagged{name, true, e};  // both attempts lost: a link fault
            }
            throw Tagged{name, is_transport_error(e), e};  // a refused late frame: LateFrame, not a link fault
        }
    }
    const double wire = begin;
    const double late = wire - planned;
    {
        std::lock_guard<std::mutex> l(run.lock);
        run.frames.push_back({k, name, wire - run.t0, late});
    }
    return late;
}

/// One lean status read of `name` (confirmed with the full block when it shows the servo off, an alarm or EMG),
/// recorded and checked. nullopt after a failed read (kSilentReads in a row throw, a link fault).
std::optional<Status> sample(State& s, Run& run, const std::string& name, std::optional<double> max_wait = std::nullopt,
                             bool asof_ok = true) {
    AxisImpl& a = s.im(name);
    const double t1 = now_s();
    Status st;
    double t = t1;
    try {
        st = lean_read(a, max_wait.value_or(run.read_wait_s));
        t = (t1 + now_s()) / 2;
        st = a.confirmed(st);
    } catch (...) {
        const std::exception_ptr e = std::current_exception();
        if (!lost_answer(e)) throw Tagged{name, is_transport_error(e), e};
        int count = 0;
        {
            std::lock_guard<std::mutex> l(run.lock);
            count = ++run.failures[name];
            run.read_errors[name] = e;
        }
        if (count >= kSilentReads) throw Tagged{name, true, e};  // kSilentReads failed reads in a row: a link fault
        return std::nullopt;
    }
    {
        std::lock_guard<std::mutex> l(run.lock);
        run.failures[name] = 0;
    }
    auto& series = run.samples.at(name);
    if (series.empty() || series.back().first < t - run.t0) series.emplace_back(t - run.t0, st.position_mm);
    try {
        check_status(name, st, asof_ok);
    } catch (...) {
        throw Tagged{name, false, std::current_exception()};
    }
    return st;
}

/// Wait until every axis of `names` is done at its target by its completion rule (fast: one sample with PEND, MOVE
/// clear and inside the band; safe: that held with the position unchanged over the axis's settle_s; an automatic servo
/// OFF inside the band counts as done; blend: only within that distance), sampling every axis of the port meanwhile.
void finish(State& s, Run& run, const std::vector<std::string>& names, const std::vector<double>& targets,
            double timeout, std::optional<double> blend = std::nullopt) {
    std::map<std::string, double> pending;
    std::map<std::string, Settle> settles;
    for (std::size_t i = 0; i < names.size(); ++i) {
        pending[names[i]] = targets[i];
        settles.emplace(names[i], Settle(s.ax(names[i]).options().settle_s));
    }
    const double deadline = now_s() + timeout;
    const double band = run.plan.band;
    while (!pending.empty()) {
        for (const auto& name : names) {  // the axes already done are sampled too: the trace needs every axis
            if (run.aborted()) throw Aborted{};
            const std::optional<Status> st = sample(s, run, name);
            if (!st || !pending.count(name)) continue;
            const double target = pending[name];
            if (blend) {
                if (std::fabs(st->position_mm - target) <= *blend + 1e-6) pending.erase(name);
                continue;
            }
            const bool asof = st->auto_servo_off().value_or(false) && !st->servo_on;
            if (asof && std::fabs(st->position_mm - target) > band + 1e-6)  // the frame did not switch it back on
                throw Tagged{name, false,
                             std::make_exception_ptr(EcNotReadyError("axis " + name + ": automatic servo OFF at " +
                                                                     f(st->position_mm, 3) + ", away from its target " +
                                                                     f(target, 3)))};
            const bool at_rest = (fast_done(*st, target, band) || asof) && !st->homing();
            const bool done = s.ax(name).options().completion == Completion::Fast
                                  ? at_rest
                                  : settles.at(name).update(at_rest, st->position_mm);
            if (done) {
                s.im(name).motion_pending = false;
                pending.erase(name);
            }
        }
        if (pending.empty()) return;
        if (now_s() > deadline) {
            std::string name;
            for (const auto& n : names)
                if (pending.count(n)) {
                    name = n;
                    break;
                }
            throw Tagged{name, false,
                         std::make_exception_ptr(MotionTimeoutError("axis " + name + ": not at " +
                                                                    f(pending[name], 3) + " within " + f(timeout, 1) +
                                                                    " s"))};
        }
        if (run.wait_abort(kPollS)) throw Aborted{};
    }
}

void stream_port(State& s, Run& run, std::size_t port, const std::vector<std::string>& names) {
    const MotionPlan& p = run.plan;
    std::vector<std::size_t> idx;
    for (const auto& n : names) idx.push_back(static_cast<std::size_t>(p.index_of(n)));
    std::map<std::string, std::int64_t> sent;
    std::map<std::string, int> last_frame;
    std::map<std::string, double> last_read, last_try;
    const int steps = p.steps();
    for (std::size_t m = 0; m < names.size(); ++m) {
        sent[names[m]] = reg::to_counts(p.points[0][idx[m]]);
        int last = -1;
        for (int k = 0; k < steps; ++k)
            if (p.speeds[static_cast<std::size_t>(k)][idx[m]] > 0) last = k;
        last_frame[names[m]] = last;
        last_read[names[m]] = run.t0;  // the last good read (the preflight read every axis before the start)
        last_try[names[m]] = run.t0 - kSampleGapS;
    }
    double read_cost = run.timing.read_s();
    auto oldest = [&](const std::map<std::string, double>& stamps) {
        std::string best = names[0];
        for (const auto& n : names)
            if (stamps.at(n) < stamps.at(best)) best = n;
        return best;
    };
    for (int k = 0; k < steps; ++k) {
        const std::size_t ku = static_cast<std::size_t>(k);
        const double planned = run.t0 + p.times[ku];
        const ClockMark since = clock_mark();  // a late frame of this step says how far the system clock moved since
        sleep_until(planned, run);
        const double step_s = p.times[ku + 1] - p.times[ku];
        const double limit = run.late_abort_s.value_or(step_s);
        for (std::size_t m = 0; m < names.size(); ++m) {
            const std::string& name = names[m];
            const std::int64_t target = reg::to_counts(p.points[ku + 1][idx[m]]);
            if (target == sent[name]) {
                ++run.skipped;
                continue;
            }
            const double late = send_frame(s, run, port, k, name, target, p.speeds[ku][idx[m]], p.accels[ku][idx[m]],
                                           planned, limit, since);
            sent[name] = target;
            if (late > limit)
                throw Tagged{name, false,
                             std::make_exception_ptr(LateFrame(
                                 "the frame of step " + std::to_string(k) + " reached the wire " + f(late * 1000, 0) +
                                 " ms late (limit " + f(limit * 1000, 0) +
                                 " ms): the PC or the line cannot keep this schedule" + clock_drift(since)))};
        }
        if (k + 1 >= steps) break;
        const double next_at = run.t0 + p.times[ku + 1];
        for (;;) {  // status reads in the slack before the next frame: each axis about every kSampleGapS
            const double now = now_s();
            const double slack = next_at - now - 0.001;
            const std::string stale = oldest(last_read);
            std::string name;
            double wait = 0.0;
            if (now - last_read[stale] > run.sample_bound_s) {  // read even if the next frame goes late
                name = stale;
                wait = std::max(slack, 2 * read_cost);
            } else {
                if (slack < 2 * read_cost) break;
                name = oldest(last_try);
                const double due = last_try[name] + kSampleGapS - now;
                if (due > 0) {
                    if (run.wait_abort(std::min(due, slack - 2 * read_cost))) throw Aborted{};
                    continue;
                }
                wait = slack;
            }
            const double t1 = last_try[name] = now_s();
            const std::optional<Status> st =
                sample(s, run, name, std::min(wait, run.read_wait_s), last_frame[name] <= k);
            if (st) {
                last_read[name] = now_s();
                read_cost = 0.7 * read_cost + 0.3 * (last_read[name] - t1);
            }
        }
        const double now = now_s();
        for (const auto& name : names) {
            if (now - last_read[name] <= 2 * run.sample_bound_s) continue;
            std::string cause;
            {
                std::lock_guard<std::mutex> l(run.lock);
                const auto it = run.read_errors.find(name);
                if (it != run.read_errors.end() && it->second) cause = " (last read: " + describe(it->second) + ")";
            }
            throw Tagged{name, false,
                         std::make_exception_ptr(NoStatus("axis " + name + ": no status for " +
                                                          f((now - last_read[name]) * 1000, 0) + " ms (at most " +
                                                          f(2 * run.sample_bound_s * 1000, 0) +
                                                          " ms): a fault would go unseen" + cause))};
        }
    }
    const double last_step = steps ? p.times[p.times.size() - 1] - p.times[p.times.size() - 2] : 0.0;
    const double timeout = run.timeout_s.value_or(3 * last_step + 2.0);
    std::vector<double> targets;
    for (std::size_t m = 0; m < names.size(); ++m) targets.push_back(p.points.back()[idx[m]]);
    finish(s, run, names, targets, timeout);
}

void barrier(Run& run) {
    if (!run.barrier.wait()) throw Aborted{};
}

/// Just before a sync segment's frame: an axis the unit switched off while it stood (automatic servo OFF) is switched
/// on the documented way (the others wait at the barrier, so the segments stay synchronized).
bool wake(State& s, Run& run, const std::string& name) {
    std::optional<Status> st = sample(s, run, name);
    if (!st) st = sample(s, run, name);
    if (!st || st->servo_on || !st->auto_servo_off().value_or(false)) return false;
    log_info(kLog, name + ": automatic servo OFF before a segment: servo on first");
    try {
        s.im(name).servo(true, true, std::nullopt);
    } catch (...) {
        const std::exception_ptr e = std::current_exception();
        throw Tagged{name, lost_answer(e) || is_transport_error(e), e};  // no answer after the bus's own retries: link
    }
    return true;
}

void sync_port(State& s, Run& run, std::size_t port, const std::vector<std::string>& names) {
    const MotionPlan& p = run.plan;
    std::vector<std::size_t> idx;
    for (const auto& n : names) idx.push_back(static_cast<std::size_t>(p.index_of(n)));
    std::map<std::string, std::int64_t> sent;
    for (std::size_t m = 0; m < names.size(); ++m) sent[names[m]] = reg::to_counts(p.points[0][idx[m]]);
    for (int k = 0; k < p.steps(); ++k) {
        const std::size_t ku = static_cast<std::size_t>(k);
        std::map<std::string, std::int64_t> targets;
        for (std::size_t m = 0; m < names.size(); ++m) targets[names[m]] = reg::to_counts(p.points[ku + 1][idx[m]]);
        barrier(run);  // every axis of every port done with the previous segment
        for (int rnd = 0; rnd < kWakeRounds; ++rnd) {  // a wake takes a while: every port checks again until none woke
            bool any = false;
            for (const auto& n : names)
                if (targets[n] != sent[n] && wake(s, run, n)) any = true;
            if (any) {
                std::lock_guard<std::mutex> l(run.lock);
                run.woke.insert({k, rnd});
            }
            barrier(run);
            bool woke = false;
            {
                std::lock_guard<std::mutex> l(run.lock);
                woke = run.woke.count({k, rnd}) != 0;
            }
            if (!woke) break;
        }
        const double go = now_s();
        for (std::size_t m = 0; m < names.size(); ++m) {
            const std::string& name = names[m];
            if (targets[name] == sent[name]) {
                ++run.skipped;
                continue;
            }
            send_frame(s, run, port, k, name, targets[name], p.speeds[ku][idx[m]], p.accels[ku][idx[m]], go,
                       std::nullopt, ClockMark{});  // no limit: never a late frame
            sent[name] = targets[name];
        }
        const bool last = k == p.steps() - 1;
        const double seg_s = p.times[ku + 1] - p.times[ku];
        const double timeout = run.timeout_s.value_or(2 * seg_s + 2.0);
        std::vector<double> want;
        for (std::size_t m = 0; m < names.size(); ++m) want.push_back(p.points[ku + 1][idx[m]]);
        finish(s, run, names, want, timeout, last ? std::nullopt : p.blend);
    }
}

// ============================================================================================ the run

constexpr const char* kLostError = "a port thread failed and its error could not be recorded";

CoordinatedResult make_result(Run& run, double end, const std::exception_ptr& interrupted) {
    const MotionPlan& p = run.plan;
    CoordinatedResult r;
    r.plan = p;
    r.samples = run.samples;
    const double window = std::max(0.003, run.timing.read_s());
    r.trace = detail::merge_trace(run.samples, p.axes, p.points[0], window);
    std::vector<std::vector<double>> positions;
    for (const auto& row : r.trace) positions.emplace_back(row.begin() + 1, row.end());
    const auto dev = path_deviation(positions, p.points);
    r.max_deviation = dev.first;
    r.mean_deviation = dev.second;
    std::vector<Frame> frames;
    {
        std::lock_guard<std::mutex> l(run.lock);
        frames = run.frames;
    }
    std::sort(frames.begin(), frames.end());
    int late = 0;
    double worst_late = 0.0;
    if (p.mode == Mode::Stream) {
        for (const auto& fr : frames) {
            const std::size_t k = static_cast<std::size_t>(fr.step);
            const double step_s = p.times[k + 1] - p.times[k];
            if (fr.late > std::max(0.002, kLateFraction * step_s)) ++late;
            worst_late = std::max(worst_late, fr.late);
        }
    }
    std::map<int, std::vector<double>> by_step;
    for (const auto& fr : frames) by_step[fr.step].push_back(fr.wire);
    double skew = 0.0;
    for (const auto& kv : by_step)
        if (kv.second.size() > 1)
            skew = std::max(skew, *std::max_element(kv.second.begin(), kv.second.end()) -
                                      *std::min_element(kv.second.begin(), kv.second.end()));
    std::optional<std::string> error;
    const std::optional<Failure> first = run.first_error();
    if (interrupted) {
        error = "interrupted (" + describe(interrupted) + ")";
    } else if (first) {
        error = (first->axis ? "axis " + *first->axis + ": " : std::string()) + describe(first->error);
    } else if (run.lost_error.load()) {
        error = std::string(kLostError);
    }
    for (const auto& a : p.axes) {
        const auto& series = run.samples.at(a);
        if (!series.empty()) r.final.set(a, series.back().second);
    }
    r.frames = static_cast<int>(frames.size());
    r.skipped_frames = run.skipped.load();
    r.late_frames = late;
    r.worst_late_s = std::max(0.0, worst_late);
    r.worst_skew_s = skew;
    r.duration_s = std::max(0.0, end - run.t0);
    r.completed = !error;
    r.error = error;
    r.warnings = p.warnings;
    r.sample_spacing_s = detail::sample_spacing(run.samples);
    return r;
}

CoordinatedResult drive(State& s, Run& run) {
    const bool stream = run.plan.mode == Mode::Stream;
    std::atomic<int> alive{0};
    std::vector<std::thread> threads;
    std::exception_ptr interrupted;
    CellResults<Status> stop_report;
    bool stopped = false;
    double end = 0.0;
    {
        ScopedTimerResolution timer;
        run.t0 = now_s() + (stream ? kStartLeadS : 0.0);
        threads.reserve(run.ports.size());
        try {
            for (std::size_t i = 0; i < run.ports.size(); ++i) {
                ++alive;
                try {
                    threads.emplace_back([&s, &run, &alive, stream, i] {
                        // Nothing leaves this thread: every way out counts it down and wakes the main thread, and a
                        // failure whose error could not be recorded still fails the run (lost_error).
                        struct Exit {
                            Run& run;
                            std::atomic<int>& alive;
                            ~Exit() {
                                --alive;
                                try {
                                    run.mark_changed();
                                } catch (...) {  // the main thread looks again within kWatchS
                                }
                            }
                        } done{run, alive};
                        try {
                            try {
                                if (stream)
                                    stream_port(s, run, i, run.ports[i]);
                                else
                                    sync_port(s, run, i, run.ports[i]);
                            } catch (const Aborted&) {
                                run.mark_changed();
                                run.stop_sending();
                            } catch (const Tagged& t) {
                                if (const auto& hook = detail::coord_test_hooks().on_port_error) hook();
                                run.fail(t.axis, t.error, t.link);
                            } catch (...) {
                                if (const auto& hook = detail::coord_test_hooks().on_port_error) hook();
                                const std::exception_ptr e = std::current_exception();
                                run.fail(std::nullopt, e, is_transport_error(e));
                            }
                        } catch (...) {
                            run.lost_error = true;
                            try {
                                run.set_abort();
                                log_error(kLog, std::string(kLostError) + " (" + describe(std::current_exception()) +
                                                    "): the run stops");
                            } catch (...) {  // the main thread sees lost_error and stops sending
                            }
                        }
                    });
                } catch (...) {  // not started: not counted
                    --alive;
                    throw;
                }
            }
            while (alive.load() > 0) {
                if (run.has_errors() && !stopped) {  // a port failed or an outside stop: STOP every axis now
                    run.stop_sending();
                    stop_report = stop_after_failure(s, run);
                    stopped = true;
                }
                run.wait_changed(kWatchS);
            }
        } catch (...) {  // e.g. a thread that could not start: no frame after this point, then STOP every axis
            interrupted = std::current_exception();
            run.stop_sending();
            try {
                stop_report = group_stop(s, 5.0);
            } catch (...) {
                log_warning(kLog, "group stop failed: " + describe(std::current_exception()));
            }
        }
        util::join_all(threads);
        end = now_s();
        if (!interrupted && run.has_errors() && !stopped) {
            run.stop_sending();
            stop_report = stop_after_failure(s, run);
        }
    }
    CoordinatedResult result = make_result(run, end, interrupted);
    s.set_result(result);
    if (interrupted) std::rethrow_exception(interrupted);
    if (const std::optional<Failure> first = run.first_error()) {
        const std::string where = first->axis ? "axis " + *first->axis + ": " : std::string();
        throw CoordinatedMotionError("coordinated move stopped (" + where + describe(first->error) +
                                         "); every axis of the group was sent a decel stop",
                                     result, first->axis, stop_report, first->error);
    }
    if (run.lost_error.load())
        throw CoordinatedMotionError(std::string("coordinated move stopped (") + kLostError +
                                         "); every axis of the group was sent a decel stop",
                                     result, std::nullopt, stop_report, nullptr);
    return result;
}

/// The axis ready for a run (Axis::Impl::require), an automatic servo OFF not refused: woken when wake_asof.
Status require_ready(AxisImpl& a, const Status& st, bool wake_asof) {
    const bool asof = !st.servo_on && st.auto_servo_off().value_or(false);
    Status out = a.require(st, !asof, true, false);
    if (asof && wake_asof) {
        log_info(kLog, "automatic servo OFF (STAT ASOF): switching the servo back on before the move (UNVERIFIED path)");
        a.servo(true, true, std::nullopt);
        out = a.status();
    }
    return out;
}

/// Every check before anything is sent (see CoordinatedGroup::run_plan). Returns the measured timing.
LineTiming preflight(State& s, const MotionPlan& p, bool at_start) {
    s.check_names(p.axes);
    detail::verify_plan(p, s.ranges(p.axes));
    for (std::size_t i = 0; i < p.axes.size(); ++i) {
        const std::string& axis = p.axes[i];
        AxisImpl& a = s.im(axis);
        if (p.moves(axis)) {
            double lo = p.points[1][i], hi = p.points[1][i];
            for (std::size_t k = 1; k < p.points.size(); ++k) {
                lo = std::min(lo, p.points[k][i]);
                hi = std::max(hi, p.points[k][i]);
            }
            const double speed = p.max_speed(axis);
            const double accel = std::max(0.01, p.max_accel(axis));
            a.validate_motion(lo, speed, accel, p.band);
            if (hi != lo) a.validate_motion(hi, speed, accel, p.band);
        }
        const Status st = require_ready(a, a.status(), false);
        if (st.moving) throw EcNotReadyError("axis " + axis + " is moving: wait until it stands still; nothing sent");
        if (a.stop_coil_on.load() || a.stop_pause_on.load() || a.group_stops.load() > 0)
            throw EcNotReadyError("axis " + axis +
                                  ": a STOP is held on it (a stop in progress, or its release failed): finish it with "
                                  "stop() first; nothing sent");
        if (at_start && std::fabs(st.position_mm - p.points[0][i]) > std::max(p.band, 0.05))
            throw EcNotReadyError("axis " + axis + " is at " + f(st.position_mm, 3) + ", the plan starts at " +
                                  f(p.points[0][i], 3) + ": plan again from the current position; nothing sent");
    }
    const LineTiming timing = s.timing(p.axes, true);
    if (p.mode == Mode::Stream && p.steps() > 0 && !p.min_step_override) {
        double line_s = timing.min_step_s();
        if (p.timing && p.timing->measured_write_s && p.timing->model() == timing.model())
            line_s = std::min(line_s, p.timing->min_step_s());
        const double need = std::max(timing.model().min_step_s(), kMeasureTolerance * line_s);
        if (p.shortest_step_s() < need - 1e-9)
            throw SoftLimitError("coordinated move refused, nothing sent: steps down to " +
                                 f(p.shortest_step_s() * 1000, 1) + " ms, shorter than the " +
                                 f(timing.min_step_s() * 1000, 1) + " ms this line carries now (measured: frame " +
                                 f(timing.write_s() * 1000, 1) + " ms, read " + f(timing.read_s() * 1000, 1) + " ms, " +
                                 std::to_string(timing.axes_on_port) +
                                 " axis/axes on the port): plan it again from this group, or lower the speed");
    }
    return timing;
}

/// An axis in automatic servo OFF is switched on the documented way before a stream; read again now and again after a
/// wake (it takes a while: another axis may cross its delay meanwhile).
void wake_all(State& s, const std::vector<std::string>& names) {
    for (int round = 0; round < kWakeRounds; ++round) {
        bool woke = false;
        for (const auto& name : names) {
            AxisImpl& a = s.im(name);
            const Status st = a.status();
            if (st.auto_servo_off().value_or(false) && !st.servo_on) {
                log_info(kLog, name + ": automatic servo OFF: servo on before the coordinated move");
                if (!require_ready(a, st, true).servo_on)
                    throw EcNotReadyError("axis " + name + ": the servo did not come back on; nothing moved");
                woke = true;
            }
        }
        if (!woke) return;
    }
}

/// Per axis, when run / run_plan began: its stop counter and its bus's stop mark. A stop, pause or servo OFF sent to it
/// after that (or any stop-class frame to its unit, a broadcast one included) ends the run before its next frame.
struct StopsSince {
    std::map<std::string, long> counts;
    std::map<std::string, StopMark> marks;
};

/// One run per axis: the run's hook is set on every axis of the plan (their stop paths call it) and on the group.
void register_run(State& s, const std::shared_ptr<Run>& run, const StopsSince& since) {
    std::lock_guard<std::mutex> lock(runs_mutex());
    std::vector<std::string> busy, held;
    for (const auto& n : run->plan.axes) {
        AxisImpl& a = s.im(n);
        {
            std::lock_guard<std::mutex> l(a.coord_mutex);
            if (a.coord_run) busy.push_back(n);
        }
        if (a.stop_coil_on.load() || a.stop_pause_on.load() || a.group_stops.load() > 0) held.push_back(n);
    }
    if (!busy.empty())
        throw EcNotReadyError("axis/axes " + bracket(busy) + " are in another coordinated move: one at a time; nothing "
                              "sent");
    if (!held.empty())
        throw EcNotReadyError("axis/axes " + bracket(held) + ": a STOP is held on them (a stop in progress); nothing "
                              "sent");
    for (const auto& n : run->plan.axes) {
        AxisImpl& a = s.im(n);
        auto hook = std::make_shared<AxisStopHook>(run, n);
        run->hooks.push_back(hook);
        {
            std::lock_guard<std::mutex> l(a.coord_mutex);
            a.coord_run = hook;
        }
        const auto mark = since.marks.find(n);  // the mark first: a stop counted after it is refused by both
        run->stop_marks[n] =
            mark != since.marks.end() ? mark->second : ::iai_actuator_driver::detail::stop_mark(*a.bus, a.slave);
        const auto it = since.counts.find(n);
        run->stop_counts[n] = it != since.counts.end() ? it->second : a.stop_count.load();
    }
    std::lock_guard<std::mutex> l(s.mutex);
    s.active_run = run;
}

void unregister_run(State& s, const std::shared_ptr<Run>& run) {
    std::lock_guard<std::mutex> lock(runs_mutex());
    for (std::size_t i = 0; i < run->plan.axes.size() && i < run->hooks.size(); ++i) {
        AxisImpl& a = s.im(run->plan.axes[i]);
        std::lock_guard<std::mutex> l(a.coord_mutex);
        if (a.coord_run == run->hooks[i]) a.coord_run.reset();
    }
    std::lock_guard<std::mutex> l(s.mutex);
    if (s.active_run == run) s.active_run.reset();
}

CoordinatedResult execute(State& s, const MotionPlan& p, const RunOptions& o, const StopsSince& since) {
    if (const auto& hook = detail::coord_test_hooks().before_execute) hook(p);
    const LineTiming timing = preflight(s, p, true);
    auto run = std::make_shared<Run>(p, s.ports(p.axes), timing, o.late_abort_s, o.timeout_s);
    register_run(s, run, since);
    struct Unregister {
        State& s;
        std::shared_ptr<Run> run;
        ~Unregister() {
            try {
                unregister_run(s, run);
            } catch (...) {
            }
        }
    } guard{s, run};
    std::vector<std::string> stopped;
    for (const auto& n : p.axes)
        if (s.im(n).stop_count.load() != run->stop_counts[n]) stopped.push_back(n);
    if (!stopped.empty()) {  // since run_plan began (e.g. between the lead-in and the path): the run ends, nothing sent
        run->halt(stopped[0], "a stop, pause or servo OFF was sent to this axis from outside the coordinated move "
                              "since it began");
    } else if (p.mode == Mode::Stream) {  // sync: before each segment (sync_port)
        std::vector<std::string> moving;
        for (const auto& n : p.axes)
            if (p.moves(n)) moving.push_back(n);
        wake_all(s, moving);
    }
    return drive(s, *run);
}

std::pair<double, double> center_pair(const PlanePoint& value, const Plane& plane) {
    if (value.point) {
        for (const auto& a : {plane.first, plane.second})
            if (!value.point->contains(a))
                throw std::invalid_argument("circle center must give both plane axes (" + plane.first + ", " +
                                            plane.second + "), missing " + a);
        return {value.point->at(plane.first), value.point->at(plane.second)};
    }
    if (!value.pair) throw std::invalid_argument("circle center must be (a, b) or {axis: position}");
    return *value.pair;
}

}  // namespace

// ============================================================================================ CoordinatedGroup

CoordinatedGroup::CoordinatedGroup(AxisList axes, double band_mm)
    : CoordinatedGroup(std::move(axes), nullptr, {}, band_mm) {}

CoordinatedGroup::CoordinatedGroup(AxisList axes, Cell* cell, std::map<std::string, std::string> cell_names,
                                   double band_mm)
    : s_(std::make_shared<State>()) {
    if (axes.empty()) throw std::invalid_argument("a coordinated group needs at least one axis");
    std::set<const Axis*> seen;
    std::set<std::string> names;
    for (const auto& kv : axes) {
        if (kv.first.empty()) throw std::invalid_argument("axis names in a coordinated group must not be empty");
        if (!kv.second) throw std::invalid_argument("axis '" + kv.first + "' is null");
        if (!seen.insert(kv.second.get()).second) throw std::invalid_argument("the same axis appears twice in the group");
        if (!names.insert(kv.first).second)
            throw std::invalid_argument("the name '" + kv.first + "' appears twice in the group");
    }
    if (!std::isfinite(band_mm) || band_mm <= 0)
        throw std::invalid_argument("band_mm must be > 0, got " + g6(band_mm));
    s_->axes = std::move(axes);
    s_->cell = cell;
    s_->cell_names = std::move(cell_names);
    s_->band = band_mm;
}

std::vector<std::string> CoordinatedGroup::names() const { return s_->names(); }
Axis& CoordinatedGroup::axis(const std::string& name) const { return s_->ax(name); }
std::shared_ptr<Axis> CoordinatedGroup::axis_ptr(const std::string& name) const { return s_->ptr(name); }
double CoordinatedGroup::band() const { return s_->band; }
Cell* CoordinatedGroup::cell() const { return s_->cell; }

std::optional<CoordinatedResult> CoordinatedGroup::last_result() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->last_result;
}

std::string CoordinatedGroup::to_string() const {
    std::vector<std::string> parts;
    for (const auto& kv : s_->axes) {
        const auto it = s_->cell_names.find(kv.first);
        parts.push_back(kv.first + ": " + (it != s_->cell_names.end() ? it->second : kv.second->to_string()));
    }
    return "CoordinatedGroup({" + util::join(parts, ", ") + "})";
}

std::map<std::string, AxisRange> CoordinatedGroup::ranges() { return s_->ranges(s_->names()); }

LineTiming CoordinatedGroup::timing(const std::vector<std::string>& names, bool measure) {
    const auto wanted = s_->or_all(names);
    s_->check_names(wanted);
    return s_->timing(wanted, measure);
}

Point CoordinatedGroup::positions(const std::vector<std::string>& names) {
    const auto wanted = s_->or_all(names);
    s_->check_names(wanted);
    return s_->positions(wanted);
}

MotionPlan CoordinatedGroup::plan(const Path& path, double speed, double accel_g, const PlanOptions& options) {
    State& s = *s_;
    const std::vector<std::string> names = path.axes();
    s.check_names(names);
    const double band = options.band.value_or(s.band);
    const Point current = s.positions(names);
    PlanOptions common = options;
    common.limits = s.ranges(names);
    common.timing = s.timing(names, true);
    common.band = band;
    common.start.reset();
    bool far = false;
    if (path.start())
        for (const auto& kv : *path.start())
            if (std::fabs(current.at(kv.first) - kv.second) > std::max(band, 0.01)) far = true;
    if (!far) {
        PlanOptions o = common;
        o.start = current.merged(path.start().value_or(Point()));
        return coord::plan(path, speed, accel_g, o);
    }
    Point here;
    for (const auto& kv : current) here.set(kv.first, std::round(kv.second * 1000) / 1000);
    const std::string note = "the axes are not at the path's start: a straight lead-in from " + here.to_string();
    if (!(options.mode == Mode::Stream || (options.mode == Mode::Auto && !path.straight()))) {
        Path with_lead(current, {Line(*path.start())});
        with_lead.then(path.segments());
        PlanOptions o = common;
        o.start = current;
        MotionPlan result = coord::plan(with_lead, speed, accel_g, o);
        result.warnings.insert(result.warnings.begin(), note + " was added");
        return result;
    }
    PlanOptions lead_o = common;
    lead_o.mode = Mode::Sync;
    lead_o.start = current;
    lead_o.blend.reset();
    lead_o.stream_accel_g.reset();
    MotionPlan lead_in = coord::plan(Path(current, {Line(*path.start())}), speed, accel_g, lead_o);
    MotionPlan result;
    try {
        PlanOptions o = common;
        o.start = current.merged(*path.start());
        result = coord::plan(path, speed, accel_g, o);
    } catch (const NoMotionError&) {  // the path itself moves no axis by a count: the lead-in alone takes them there
        lead_in.warnings.insert(lead_in.warnings.begin(),
                                note + " is all that runs: the path itself moves no axis by a whole position count "
                                       "(0.01) from its start");
        return lead_in;
    }
    result.lead_in = std::make_shared<MotionPlan>(lead_in);
    result.warnings.insert(result.warnings.begin(), note + " runs first, as a synchronized line");
    return result;
}

namespace {

/// RunOptions::timeout_s: nullopt is the default (from the plan); a value must be finite and above 0, else
/// SoftLimitError before anything is sent, as for every command timeout (0 or -1 is "wait forever" in some languages:
/// a run with it would end its wait before it began and decel-stop at once). A path that moves no axis is refused too,
/// never answered with a completed result.
void check_run_timeout(const RunOptions& options) { iai_actuator_driver::detail::check_timeout(options.timeout_s); }

/// Every axis's stop mark and stop counter now: a stop, pause or servo OFF sent to one of them after this (or any
/// stop-class frame to its unit, a broadcast one included) ends the run.
StopsSince stops_now(State& s, const std::vector<std::string>& names) {
    StopsSince now;
    for (const auto& n : names) {
        AxisImpl& a = s.im(n);
        now.marks[n] = ::iai_actuator_driver::detail::stop_mark(*a.bus, a.slave);  // before the counter (register_run)
        now.counts[n] = a.stop_count.load();
    }
    return now;
}

/// Where `p` starts: its lead-in's first waypoint, if any, else its own ({} for a plan without waypoints).
Point start_of(const MotionPlan& p) {
    const MotionPlan& first = p.lead_in ? *p.lead_in : p;
    Point at;
    if (!first.points.empty())
        for (std::size_t i = 0; i < first.axes.size() && i < first.points[0].size(); ++i)
            at.set(first.axes[i], first.points[0][i]);
    return at;
}

/// RunOptions::expect_start against `start` (where the plan that runs starts, or where the axes are): each named axis
/// within max(band, 0.05), else EcNotReadyError; an axis the plan does not name, std::invalid_argument. Nothing sent.
void check_expected_start(const RunOptions& options, const Point& start, double band) {
    if (!options.expect_start) return;
    for (const auto& kv : *options.expect_start) {
        if (!start.contains(kv.first))
            throw std::invalid_argument("expect_start names axis " + kv.first +
                                        ", which the path does not; nothing sent");
        if (std::fabs(start.at(kv.first) - kv.second) > std::max(band, 0.05))
            throw EcNotReadyError("axis " + kv.first + " is at " + f(start.at(kv.first), 3) + ", the plan starts at " +
                                  f(kv.second, 3) + ": plan again from the current position; nothing sent");
    }
}

/// run_plan with the stop counters and marks `since` the caller began (run(): before its planning); an axis not in
/// `since` counts from now. One snapshot for the lead-in and the path.
CoordinatedResult run_plan_since(State& s, const MotionPlan& plan, const RunOptions& options, StopsSince since) {
    s.check_names(plan.axes);
    if (options.late_abort_s && !(std::isfinite(*options.late_abort_s) && *options.late_abort_s > 0))
        throw std::invalid_argument("late_abort_s must be > 0, got " + g6(*options.late_abort_s));
    if (plan.points.empty() || plan.times.size() != plan.points.size())
        throw std::invalid_argument("the plan has no waypoints");
    std::vector<std::string> names = plan.axes;
    if (plan.lead_in)
        for (const auto& n : plan.lead_in->axes)
            if (std::find(names.begin(), names.end(), n) == names.end()) names.push_back(n);
    s.check_names(names);
    const StopsSince now = stops_now(s, names);  // emplace keeps the earlier ones
    for (const auto& kv : now.marks) since.marks.emplace(kv.first, kv.second);
    for (const auto& kv : now.counts) since.counts.emplace(kv.first, kv.second);
    auto held = s.hold_monitors(names);
    if (plan.lead_in) {
        preflight(s, *plan.lead_in, true);
        preflight(s, plan, false);
        execute(s, *plan.lead_in, options, since);
    }
    return execute(s, plan, options, since);
}

}  // namespace

CoordinatedResult CoordinatedGroup::run(const Path& path, double speed, double accel_g,
                                        const PlanOptions& plan_options, const RunOptions& run_options) {
    check_run_timeout(run_options);
    State& s = *s_;
    const std::vector<std::string> names = path.axes();
    s.check_names(names);
    // an outside stop counts from here, as from the start of run_plan: one that comes while the path is planned (the
    // reads and the line measurement take a while on a slow line) ends the run before its first frame
    const StopsSince since = stops_now(s, names);
    MotionPlan p;
    try {
        p = plan(path, speed, accel_g, plan_options);
    } catch (const NoMotionError& why) {
        // the completed, empty result of a path that moves no axis - only when every axis is at the path's end
        // (within the band): completed must never stand for a move that did not happen
        const double band = plan_options.band.value_or(s.band);
        const Point current = s.positions(names);
        check_expected_start(run_options, current, band);
        const Point origin = current.merged(path.start().value_or(Point()));
        const std::vector<Piece> pieces = path.pieces(origin);
        const Point end = pieces.empty() ? origin : pieces.back().end;
        Point away;
        for (const auto& n : names)
            if (std::fabs(current.at(n) - end.at(n)) > std::max(band, 0.01))
                away.set(n, std::round(current.at(n) * 1000) / 1000);
        if (!away.empty())
            throw std::invalid_argument(std::string(why.what()) + ", but axis/axes " + away.to_string() +
                                        " are not at its end " + end.to_string() + ": nothing sent");
        MotionPlan empty;
        empty.mode = Mode::Sync;
        empty.axes = names;
        std::vector<double> here;
        for (const auto& n : names) here.push_back(current.at(n));
        empty.points = {here};
        empty.times = {0.0};
        empty.path_speed = speed;
        empty.accel_g = accel_g;
        empty.band = band;
        CoordinatedResult result;
        result.plan = empty;
        for (const auto& n : names) result.samples[n];
        result.completed = true;
        result.warnings = {"the axes are at the path's end already: nothing sent"};
        result.final = current;
        s.set_result(result);
        return result;
    }
    // a binding's run_plan: the start of the plan it holds, checked here, inside the stop window (not by the binding
    // before this call, where a stop sent meanwhile would not be counted)
    check_expected_start(run_options, start_of(p), p.band);
    if (const auto& hook = detail::coord_test_hooks().after_plan) hook(p);
    return run_plan_since(s, p, run_options, since);
}

CoordinatedResult CoordinatedGroup::run_plan(const MotionPlan& plan, const RunOptions& options) {
    check_run_timeout(options);
    return run_plan_since(*s_, plan, options, {});
}

CoordinatedResult CoordinatedGroup::line_to(const Point& point, double speed, double accel_g,
                                            const PlanOptions& plan_options, const RunOptions& run_options) {
    return run(Path(std::nullopt, {Line(point)}), speed, accel_g, plan_options, run_options);
}

CoordinatedResult CoordinatedGroup::circle(const PlanePoint& center, double radius, double speed, double accel_g,
                                           const CircleOptions& c, const PlanOptions& plan_options,
                                           const RunOptions& run_options) {
    check_run_timeout(run_options);
    State& s = *s_;
    Plane plane;
    if (c.plane) {
        plane = *c.plane;
    } else {
        if (s.axes.size() < 2) throw std::invalid_argument("a circle needs two axes");
        plane = {s.axes[0].first, s.axes[1].first};
    }
    if (plane.first.empty() || plane.second.empty() || plane.first == plane.second)
        throw std::invalid_argument("plane must be two different axis names");
    s.check_names({plane.first, plane.second});
    if (c.start_deg)
        return run(Path::circle(center, radius, *c.start_deg, c.turns, c.ccw, plane), speed, accel_g, plan_options,
                   run_options);
    const auto cxy = center_pair(center, plane);
    Point held;
    if (center.point)
        for (const auto& kv : *center.point)
            if (kv.first != plane.first && kv.first != plane.second) held.set(kv.first, kv.second);
    s.check_names(held.axes());
    std::vector<std::string> read{plane.first, plane.second};
    for (const auto& a : held.axes()) read.push_back(a);
    const Point pos = s.positions(read);
    const double dx = pos.at(plane.first) - cxy.first;
    const double dy = pos.at(plane.second) - cxy.second;
    const bool on_circle = std::fabs(std::hypot(dx, dy) - radius) <= std::max(0.01, 0.005 * radius);
    const double band = plan_options.band.value_or(s.band);
    bool held_there = true;
    for (const auto& kv : held)
        if (std::fabs(pos.at(kv.first) - kv.second) > std::max(band, 0.01)) held_there = false;
    if (on_circle && held_there) {  // from where the axes are (held ones too): no lead-in for a radius a little off
        const Circle seg(center, radius, std::atan2(dy, dx) * 180.0 / kPi, c.turns, c.ccw, plane);
        return run(Path(pos, {seg}), speed, accel_g, plan_options, run_options);
    }
    return run(Path::circle(center, radius, 0.0, c.turns, c.ccw, plane), speed, accel_g, plan_options, run_options);
}

CellResults<Status> CoordinatedGroup::stop(double timeout_s) { return group_stop(*s_, timeout_s); }

namespace detail {
CoordTestHooks& coord_test_hooks() {
    static CoordTestHooks hooks;
    return hooks;
}
}  // namespace detail

}  // namespace coord

// ============================================================================================ Cell::coordinated

coord::CoordinatedGroup Cell::coordinated(const std::vector<std::pair<std::string, std::string>>& axes) {
    std::vector<std::pair<std::string, std::string>> mapping = axes;
    if (mapping.empty())
        for (const auto& n : names()) mapping.emplace_back(n, n);
    std::set<std::string> cell_axes;
    std::set<std::string> aliases;
    coord::CoordinatedGroup::AxisList list;
    std::map<std::string, std::string> cell_names;
    for (const auto& kv : mapping) {
        if (!contains(kv.second))
            throw std::out_of_range("no axis '" + kv.second + "' in this cell (axes: " + util::join(names(), ", ") + ")");
        if (!cell_axes.insert(kv.second).second)
            throw std::invalid_argument("an axis is named twice: '" + kv.second + "'");
        if (!aliases.insert(kv.first).second) throw std::invalid_argument("the name '" + kv.first + "' is given twice");
        list.emplace_back(kv.first, axis_ptr(kv.second));
        cell_names[kv.first] = kv.second;
    }
    return coord::CoordinatedGroup(std::move(list), this, std::move(cell_names));
}

coord::CoordinatedGroup Cell::coordinated_names(const std::vector<std::string>& names_) {
    std::vector<std::pair<std::string, std::string>> mapping;
    for (const auto& n : names_) mapping.emplace_back(n, n);
    return coordinated(mapping);
}

}  // namespace iai_actuator_driver
