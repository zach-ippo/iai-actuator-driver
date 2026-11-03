// The fast path of an Axis (axis unit): the lean status read, the waits by the completion rule (safe or fast) fed by
// the status monitor or by own polls, the pre-move status taken from a fresh monitor sample, moves started without
// waiting (MoveHandle), start_monitor / stop_monitor and enable_fast / disable_fast. Port of the Python edition's
// ec.py (read_lean_status, wait_idle, _is_done, _confirmed, _status_feed, _sample_since, _pre_move_status,
// start_monitor / stop_monitor, enable_fast / disable_fast, open / connect with fast=True). The behaviour is VERIFIED in
// the Python edition where its documentation says so; this edition is UNVERIFIED on hardware (simulator only).
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

using detail::kAxisLog;
using detail::kPosEps;

std::string f2(double v) { return util::fixed(v, 2); }
std::string tf(bool v) { return v ? "true" : "false"; }

}  // namespace

// ============================================================================================ Impl: samples

Completion Axis::Impl::completion_mode(std::optional<Completion> completion) const {
    return completion ? *completion : opts().completion;
}

std::shared_ptr<AxisMonitor> Axis::Impl::live_monitor() const {
    std::shared_ptr<AxisMonitor> mon;
    {
        std::lock_guard<std::mutex> lock(mutex);
        mon = monitor;
    }
    return mon && mon->running() ? mon : nullptr;
}

void Axis::Impl::stop_monitors() {
    std::shared_ptr<AxisMonitor> own;
    {
        std::lock_guard<std::mutex> lock(mutex);
        own = std::move(monitor);
    }
    detail::stop_monitors_of(this);
}

Status Axis::Impl::read_lean_status(std::optional<int> retries) {
    const std::vector<std::uint16_t> words = bus->read_registers(slave, reg::kMonitorBase, kLeanStatusWords, retries);
    // PNOW in the line's position unit (an RCON unit counting 0.001 mm: 3 decimals) and the line kind (axis_rcon.cpp)
    return decode_status(words);
}

Status Axis::Impl::confirmed(const Status& st) {
    if (!st.raw.stat && (!st.servo_on || st.alarm || st.emergency_stop)) return status();
    return st;
}

Status Axis::Impl::pre_move_status() {
    if (auto mon = live_monitor()) {
        const std::optional<Status> st = mon->fresh(kFreshS);
        if (st && st->ready()) return *st;
    }
    return status();
}

Status Axis::Impl::measured_status() {
    if (auto mon = live_monitor()) {
        if (std::optional<Status> st = mon->fresh(kFreshS)) return *st;
    }
    return status();
}

bool Axis::Impl::is_done(const Status& st, std::optional<double> target, double band, Completion mode,
                         detail::Settle& settle) {
    const bool arrived = !target || std::fabs(st.position_mm - *target) <= band + kPosEps;
    const bool asof = st.auto_servo_off().value_or(false) && !st.servo_on;  // PEND drops at an ASOF (PCON p.5-8)
    if (mode == Completion::Fast) return fast_done(st, target, band) || (asof && arrived && !st.homing());
    return settle.update((st.in_position || st.push_missed() || asof) && arrived && !st.homing(), st.position_mm);
}

Axis::Impl::Feed Axis::Impl::make_feed(bool lean, double since) const {
    Feed feed;
    feed.lean = lean;
    feed.since = since;
    const double poll = opts().poll_interval_s;
    feed.gap_s = lean ? std::min(poll, kFastPollS) : poll;
    return feed;
}

Status Axis::Impl::next_status(Feed& feed) {
    // a check point on every sample: a monitor that keeps delivering never lets wait_update reach its own check
    check_cancel();
    if (auto mon = live_monitor()) {
        std::optional<MonitorSample> sample;
        try {
            sample = mon->wait_update(feed.seq, feed.since, kStallS);
        } catch (const ModbusError&) {
            // the monitor ended on an error: read directly
        }
        if (sample) {
            feed.seq = sample->seq;
            feed.first = false;
            return confirmed(sample->status);
        }
    } else if (!feed.first) {
        cancellable_sleep(feed.gap_s);  // cancel_waits ends it at once (cancel.hpp)
    }
    feed.first = false;
    return feed.lean ? confirmed(read_lean_status()) : status();
}

Status Axis::Impl::sample_since(double since, bool lean) {
    if (auto mon = live_monitor()) {
        const std::optional<MonitorSample> sample = mon->latest();
        if (sample && sample->t_start >= since && sample->age_s() <= kFreshS) return confirmed(sample->status);
    }
    return lean ? confirmed(read_lean_status()) : status();
}

void Axis::Impl::note_move(double target, double band, double timeout_s, Completion mode, double sent_at,
                           const MoveWatch& watch) {
    StartedMove m;
    m.target = target;
    m.band = band;
    m.timeout_s = timeout_s;
    m.completion = mode;
    m.sent_at = sent_at;
    m.watch = watch;
    std::lock_guard<std::mutex> lock(mutex);
    started_move = m;
}

// ============================================================================================ Impl: the wait

bool Axis::Impl::stopped_from_outside(const MoveWatch& watch) const {
    // a stop of this object (another thread's stop(), a group stop, another wait's decel stop) after the move's frame
    return halts.load() != watch.halt && detail::stop_mark(*bus, slave) != watch.mark;
}

void Axis::Impl::raise_stopped_from_outside(const Status& st, const std::string& what) {
    // the wait ends now instead of at its settled-away check or its timeout, and sends no STOP (one is under way)
    MotionIncompleteError error(what + " stopped from outside this wait: a stop of this axis (stop() from another "
                                       "thread, a group stop) reached it at " +
                                f2(st.position_mm) + " " + units_label() + "; this wait sent no STOP of its own");
    error.set_stop_attempted();  // that stop is the decel stop: none from guard()
    throw error;
}

bool Axis::Impl::ended_from_outside(const MoveWatch& watch, const Status& st, std::optional<double> target,
                                    double band) {
    if (!stopped_from_outside(watch)) return false;
    const bool arrived = !target || std::fabs(st.position_mm - *target) <= band + kPosEps;
    if (arrived && st.in_position && !st.moving) {
        motion_pending = false;
        return true;
    }
    const std::string u = units_label();
    raise_stopped_from_outside(st, target ? "move to " + f2(*target) + " " + u : std::string("move"));
}

Status Axis::Impl::wait_done(std::optional<double> target, std::optional<double> band, double timeout_s,
                             Completion mode, std::optional<double> since, const MoveWatch* watch) {
    const double b = band.value_or(0.1);
    const double start = now_s();
    const double deadline = start + timeout_s;
    std::optional<double> away_since;
    detail::Settle settle(opts().settle_s);
    Feed feed = make_feed(mode == Completion::Fast, since.value_or(start));
    ScopedTimerResolution timer;
    return guard([&]() -> Status {
        for (;;) {
            const Status st = next_status(feed);
            check_during_motion(st);
            const bool arrived = !target || std::fabs(st.position_mm - *target) <= b + kPosEps;
            if (is_done(st, target, b, mode, settle)) {
                motion_pending = false;
                return st;
            }
            if (watch && ended_from_outside(*watch, st, target, b)) return st;
            const double now = now_s();
            const std::string u = units_label();
            if (target && st.in_position && !arrived && !st.moving) {
                if (!away_since) away_since = now;
                if (now - *away_since >= kSettledAwayS) {
                    std::optional<Span> soft_limits;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        soft_limits = soft.limits;
                    }
                    stop_and_raise(MotionIncompleteError(
                        "axis settled at " + f2(st.position_mm) + " " + u + ", " +
                        f2(std::fabs(st.position_mm - *target)) + " " + u + " from the target " + f2(*target) + " " + u +
                        ": the move was ignored or cut short (STOP held, another master, or the unit's own soft "
                        "limits " +
                        detail::fmt_range(soft_limits, u) + ")"));
                }
            } else {
                away_since.reset();
            }
            if (now >= deadline) {
                const std::string where = target ? " (target " + f2(*target) + " " + u + ")" : std::string();
                stop_and_raise(MotionTimeoutError(util::format("not done after %.1f s", timeout_s) + where +
                                                  ": position " + f2(st.position_mm) + " " + u + ", moving=" +
                                                  tf(st.moving) + ", in_position=" + tf(st.in_position) +
                                                  ", paused=" + tf(st.paused())));
            }
        }
    });
}

// ============================================================================================ Axis: fast path

DeviceInfo Axis::connect(bool take_control, bool fast) {
    DeviceInfo info = connect(take_control);
    if (fast) enable_fast();
    return info;
}

Status Axis::read_lean_status(std::optional<int> retries) { return impl_->read_lean_status(retries); }

Status Axis::wait_idle(std::optional<double> target, std::optional<double> band, double timeout_s,
                       Completion completion) {
    return impl_->wait_done(target, band, timeout_s, completion);
}

MoveHandle Axis::start_move_to(double position, const MoveOptions& options) {
    MoveOptions o = options;
    o.wait = false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->started_move.reset();
    }
    impl_->move_to(position, o);
    return started_handle(o, reg::from_counts(reg::to_counts(position)));
}

MoveHandle Axis::start_move_by(double delta, const MoveOptions& options) {
    MoveOptions o = options;
    o.wait = false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->started_move.reset();
    }
    impl_->move_by(delta, o);
    std::optional<double> target;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        target = impl_->last_target;
    }
    return started_handle(o, target.value_or(0.0));
}

MoveHandle Axis::start_jog(double delta, const MoveOptions& options) { return start_move_by(delta, options); }

MoveHandle Axis::started_handle(const MoveOptions& o, double target) {
    std::optional<Impl::StartedMove> m;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        m = impl_->started_move;
    }
    if (!m) {
        // a path that does not record its move (the command-profile move): the target as sent, the timeout of the
        // longest move the stroke allows
        const Span stroke = impl_->motion_stroke();
        Impl::StartedMove f;
        f.target = target;
        f.band = o.band;
        const double accel = o.decel_g ? std::min(o.accel_g, *o.decel_g) : o.accel_g;
        f.timeout_s =
            o.timeout_s ? *o.timeout_s : detail::default_move_timeout_s(stroke.max - stroke.min, o.speed, accel);
        f.completion = impl_->completion_mode(o.completion);
        f.sent_at = now_s();
        m = f;
    }
    return MoveHandle(*this, m->target, m->band, m->timeout_s, m->completion, m->sent_at);
}

std::shared_ptr<AxisMonitor> Axis::start_monitor(std::optional<MonitorOptions> options) {
    if (std::shared_ptr<AxisMonitor> live = impl_->live_monitor()) {
        if (options) {
            for (auto& f : options->on_change) live->add_on_change(f);
            for (auto& f : options->on_done) live->add_on_done(f);
            for (auto& f : options->on_event) live->add_on_event(f);
        }
        return live;
    }
    std::shared_ptr<AxisMonitor> existing;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        existing = impl_->monitor;
    }
    if (existing && !options) {  // ended on an error (port gone): restarted with its callbacks
        existing->start();
        return existing;
    }
    std::shared_ptr<AxisMonitor> mon = AxisMonitor::create(*this, options ? std::move(*options) : MonitorOptions{});
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->monitor = mon;
    }
    mon->start();
    return mon;
}

void Axis::stop_monitor(double timeout_s) {
    std::shared_ptr<AxisMonitor> mon;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        mon = std::move(impl_->monitor);
    }
    if (mon) mon->stop(timeout_s);
}

std::shared_ptr<AxisMonitor> Axis::monitor() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->monitor;
}

void Axis::set_port_owner(bool owner) { impl_->port_owner = owner; }

FastModeSummary Axis::enable_fast(std::optional<int> baud_rate, bool with_monitor) {
    std::vector<std::string> notes;
    const std::optional<int> current = impl_->bus->baud_rate();
    const std::shared_ptr<Line> line = impl_->bus->line();
    if (baud_rate && current != baud_rate) {
        const auto kept = [&](const std::exception& e) {
            const std::optional<int> now = impl_->bus->baud_rate();
            notes.push_back("baud rate kept at " + (now ? std::to_string(*now) : std::string("none")) + ": " + e.what());
        };
        if (!impl_->owns_bus && !impl_->port_owner.load()) {
            notes.push_back("shared port: baud rate kept (use Port::set_baud_rate for every unit on the line)");
        } else if (!impl_->bus->can_change_baud_rate()) {
            notes.push_back("this transport has no baud rate to change");
        } else if (line && !(line->allows(LineOperation::Broadcast) && line->allows(LineOperation::IaiCommand))) {
            notes.push_back(std::string(iai_actuator_driver::to_string(line->kind())) +
                            " carries no broadcast IAI command: baud rate kept");
        } else {
            try {
                impl_->set_baud_rate(*baud_rate, BaudChangeOptions{});
            } catch (const EcNotReadyError& e) {
                kept(e);
            } catch (const VerificationError& e) {
                kept(e);
            } catch (const SoftLimitError& e) {
                kept(e);
            } catch (const LineRuleError& e) {
                kept(e);
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->options.completion = Completion::Fast;
    }
    if (with_monitor) start_monitor();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->fast_mode = true;
        impl_->fast_notes = notes;
    }
    for (const auto& n : notes) log_info(kAxisLog, "fast mode: " + n);
    FastModeSummary out;
    out.baud_rate = impl_->bus->baud_rate();
    out.completion = Completion::Fast;
    out.monitor = impl_->live_monitor() != nullptr;
    out.notes = std::move(notes);
    return out;
}

FastModeSummary Axis::disable_fast() {
    stop_monitor();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->options.completion = Completion::Safe;
        impl_->fast_mode = false;
        impl_->fast_notes.clear();
    }
    FastModeSummary out;
    out.baud_rate = impl_->bus->baud_rate();
    out.completion = Completion::Safe;
    out.monitor = false;
    return out;
}

bool Axis::fast_mode() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->fast_mode;
}

std::vector<std::string> Axis::fast_notes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->fast_notes;
}

}  // namespace iai_actuator_driver
