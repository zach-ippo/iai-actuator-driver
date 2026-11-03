// MoveHandle: a move started without waiting (axis unit). Port of the Python edition's ec.MoveHandle; the waits are the
// axis's own (Axis::Impl::wait_done, sample_since in axis_fast.cpp). UNVERIFIED on hardware in this edition.
#include "iai_actuator_driver/move_handle.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "axis_impl.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

struct MoveHandle::State {
    State(void* axis_, double target_, double band_, double timeout_, Completion completion_, double sent_at_,
          double settle_s, const detail::MoveWatch& watch_)
        : axis(axis_),
          target(std::round(target_ * 1000.0) / 1000.0),
          band(band_),
          timeout_s(timeout_),
          completion(completion_),
          sent_at(sent_at_),
          deadline(sent_at_ + timeout_),
          settle(settle_s),
          watch(watch_) {}

    void* axis;  ///< the Axis::Impl (MoveHandle members cast it back)
    double target;
    double band;
    double timeout_s;
    Completion completion;
    double sent_at;
    double deadline;
    bool cancelled = false;
    std::optional<Status> result;
    detail::Settle settle;  ///< the safe rule's window over done() calls
    detail::MoveWatch watch;  ///< a stop of the axis from outside ends the waits at once (ended_from_outside)
    long seq = 0;           ///< (ABI 0.5, py-monitor) the last status monitor sample poll() took
    double sample_at = -1.0;  ///< (ABI 0.5, py-monitor) now_s() when poll() last took one (< 0: never)
};

namespace {
/// The axis state of a handle (only MoveHandle, a friend of Axis, names the type).
template <class Impl>
Impl& axis_of(void* p) {
    return *static_cast<Impl*>(p);
}

/// The watch of the move sent at `sent_at` (start_move_to recorded it), else the stops of the axis from now on.
template <class Impl>
detail::MoveWatch watch_of(Impl& ax, double sent_at) {
    std::lock_guard<std::mutex> lock(ax.mutex);
    if (ax.started_move && ax.started_move->sent_at == sent_at) return ax.started_move->watch;
    return ax.watch_now();
}
}  // namespace

MoveHandle::MoveHandle(Axis& axis, double target, double band, double timeout_s, Completion completion, double sent_at)
    : s_(std::make_shared<State>(axis.impl_.get(), target, band, timeout_s, completion, sent_at,
                                 axis.impl_->opts().settle_s, watch_of(*axis.impl_, sent_at))) {}

double MoveHandle::target() const { return s_->target; }
double MoveHandle::band() const { return s_->band; }
double MoveHandle::timeout_s() const { return s_->timeout_s; }
Completion MoveHandle::completion() const { return s_->completion; }
double MoveHandle::sent_at() const { return s_->sent_at; }
double MoveHandle::deadline() const { return s_->deadline; }
bool MoveHandle::cancelled() const { return s_->cancelled; }
std::optional<Status> MoveHandle::result() const { return s_->result; }

bool MoveHandle::done() {
    if (s_->result) return true;
    Axis::Impl& ax = axis_of<Axis::Impl>(s_->axis);
    const Status st = ax.sample_since(s_->sent_at, s_->completion == Completion::Fast);
    ax.check_during_motion(st);
    if (ax.is_done(st, s_->target, s_->band, s_->completion, s_->settle)) {
        if (s_->completion == Completion::Safe) ax.note_settled(&s_->watch);  // move_by adds to the position again
        s_->result = st;
        ax.motion_pending = false;
        return true;
    }
    if (ax.ended_from_outside(s_->watch, st, s_->target, s_->band)) {
        s_->result = st;
        ax.motion_pending = false;
        return true;
    }
    return false;
}

Status MoveHandle::wait(std::optional<double> timeout_s) {
    if (s_->result) return *s_->result;
    Axis::Impl& ax = axis_of<Axis::Impl>(s_->axis);
    s_->deadline = ax.retimed(s_->watch, s_->deadline);  // a change_speed of the move moved its estimate
    // the time left of the estimate, but always long enough to confirm a move that already ended (a wait() called
    // late must not time out, and so stop the axis, on the settle window of a finished move)
    const double timeout =
        timeout_s ? *timeout_s : std::max(s_->deadline - now_s(), ax.opts().settle_s + kLateWaitS);
    detail::MoveWatch watch = s_->watch;
    watch.estimated = watch.estimated && !timeout_s;  // the caller's own timeout stays
    const Status st = ax.wait_done(s_->target, s_->band, timeout, s_->completion, s_->sent_at, &watch);
    s_->result = st;
    return st;
}

bool MoveHandle::poll(double max_wait_s) {
    if (s_->result) return true;
    Axis::Impl& ax = axis_of<Axis::Impl>(s_->axis);
    const double slice = std::isnan(max_wait_s) || max_wait_s < 0.0 ? 0.0 : std::min(max_wait_s, 86400.0);
    const double poll_s = std::max(ax.opts().poll_interval_s, 0.001);
    const double end = now_s() + slice;
    return ax.guard([&] {  // cancellable (cancel_waits), a decel stop on any error as in wait()
        for (;;) {
            const bool fed = monitored(end);  // a monitor sample taken (or waited for), else done()'s own read
            if (fed ? s_->result.has_value() : done()) return true;
            const double now = now_s();
            s_->deadline = ax.retimed(s_->watch, s_->deadline);  // a change_speed of the move moved its estimate
            if (now >= s_->deadline) {  // past the move's own deadline: the wait with its timeout rule
                wait(std::nullopt);
                return true;
            }
            if (now >= end) return false;  // the slice ended: the move runs on
            if (!fed) cancellable_sleep(std::min({poll_s, end - now, s_->deadline - now}));
        }
    });
}

bool MoveHandle::monitored(double until) {
    // (ABI 0.5, py-monitor) poll() on the status monitor's samples since the command, as wait()'s feed takes them
    // (Axis::Impl::next_status): no read of the polling thread's own while the monitor delivers; done()'s read once it
    // published nothing for kStallS (suspended, stalled) or ended on an error.
    Axis::Impl& ax = axis_of<Axis::Impl>(s_->axis);
    const std::shared_ptr<AxisMonitor> mon = ax.live_monitor();
    if (!mon) return false;
    check_cancel();  // a check point on every sample, as next_status
    const double now = now_s();
    if (s_->sample_at < 0.0) s_->sample_at = now;
    const double stall_at = s_->sample_at + kStallS;
    if (now >= stall_at) return false;
    std::optional<MonitorSample> sample;
    try {
        sample = mon->wait_update(s_->seq, s_->sent_at, std::max(0.0, std::min({until, s_->deadline, stall_at}) - now));
    } catch (const ModbusError&) {
        return false;  // the monitor ended on an error: read directly
    }
    if (!sample) return true;  // none yet: the slice, the deadline or the stall check decides
    s_->seq = sample->seq;
    s_->sample_at = now_s();
    const Status st = ax.confirmed(sample->status);
    ax.check_during_motion(st);
    const bool done = ax.is_done(st, s_->target, s_->band, s_->completion, s_->settle);
    if (done && s_->completion == Completion::Safe) ax.note_settled(&s_->watch);  // move_by adds to the position again
    if (done || ax.ended_from_outside(s_->watch, st, s_->target, s_->band)) {
        s_->result = st;
        ax.motion_pending = false;
    }
    return true;
}

Status MoveHandle::cancel(bool wait, double timeout_s) {
    Axis::Impl& ax = axis_of<Axis::Impl>(s_->axis);
    const Status st = ax.stop(wait, timeout_s);
    s_->cancelled = true;
    if (!s_->result) s_->result = st;
    return st;
}

Json MoveHandle::to_json() const {
    Json j = Json::object();
    j.set("target_mm", s_->target);
    j.set("band_mm", s_->band);
    j.set("completion", iai_actuator_driver::to_string(s_->completion));
    j.set("timeout_s", std::round(s_->timeout_s * 1000.0) / 1000.0);
    j.set("done", s_->result.has_value());
    j.set("cancelled", s_->cancelled);
    j.set("position_mm", s_->result ? Json(s_->result->position_mm) : Json());
    return j;
}

std::string MoveHandle::to_string() const {
    const char* state = s_->cancelled ? "cancelled" : (s_->result ? "done" : "running");
    return "MoveHandle(target=" + util::g(s_->target) + ", band=" + util::g(s_->band) + ", " +
           iai_actuator_driver::to_string(s_->completion) + ", " + state + ")";
}

}  // namespace iai_actuator_driver
