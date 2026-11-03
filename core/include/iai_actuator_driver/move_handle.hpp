// A move started without waiting: Axis::start_move_to / start_move_by / start_jog send the command (every limit check
// has passed) and return a MoveHandle. Port of the Python edition's ec.MoveHandle (move_to(wait=False)); implemented
// by src/move_handle.cpp. UNVERIFIED on hardware in this edition (simulator only).
//
// * wait() blocks until the move is done by its completion rule (as move_to with wait: on timeout or alarm a decel stop
//   is sent and MotionTimeoutError / EcAlarmError thrown) and returns the final status; timeout nullopt = the time
//   left of the move's own estimate, at least settle_s + kLateWaitS (a late wait() on a finished move returns it).
// * done() checks once without blocking (one fresh monitor sample, else one status read); Completion::Safe needs the
//   settle window over several calls.
// * result() is the final status once known (nullopt before).
// * cancel() is Axis::stop (decel stop, immediate) and makes the stop status the result.
//
// A handle is a cheap copyable reference to one move; copies share the state. Valid while its Axis lives (not
// thread-safe: use one handle from one thread at a time).
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

class Axis;

class MoveHandle {
public:
    /// The handle of a move already sent to `axis` (Axis::start_move_to makes it): target and band in the axis's
    /// units, the wait's timeout, the completion rule and now_s() right after the command was answered.
    MoveHandle(Axis& axis, double target, double band, double timeout_s, Completion completion, double sent_at);

    double target() const;           ///< rounded to 0.001
    double band() const;
    double timeout_s() const;
    Completion completion() const;
    double sent_at() const;          ///< now_s() right after the command was answered
    double deadline() const;         ///< sent_at + timeout_s
    bool cancelled() const;
    /// The final status once known.
    std::optional<Status> result() const;

    /// True once the move is done (checked on one sample). Throws like wait() on an alarm, an emergency stop or the
    /// servo dropping (the axis is stopped first).
    bool done();
    /// Block until done; returns the final status (see the file comment).
    Status wait(std::optional<double> timeout_s = std::nullopt);
    /// done() every poll interval for up to max_wait_s (<= 0 or NaN: one check), then return whether the move is done;
    /// past the move's deadline it finishes as wait() does. A polling loop in slices (the C ABI's
    /// iad_move_handle_poll): Axis::cancel_waits ends it with WaitCancelledError after a decel stop, as it ends wait().
    /// With the status monitor running it takes the monitor's samples instead of reading (as wait()), reading itself
    /// only once the monitor published nothing for kStallS.
    bool poll(double max_wait_s);
    /// Decel stop (Axis::stop); the status where it stopped becomes the result unless one is known.
    Status cancel(bool wait = false, double timeout_s = 5.0);

    /// {"target_mm", "band_mm", "completion", "timeout_s", "done", "cancelled", "position_mm"} (no I/O).
    Json to_json() const;
    /// "MoveHandle(target=120, band=0.1, safe, done)".
    std::string to_string() const;

    struct State;  ///< private (src/move_handle.cpp)

private:
    /// (ABI 0.5, py-monitor) One step of poll() on the status monitor's samples, waiting until `until` at most: false
    /// when no monitor runs, it ended or it stalled (done() reads then).
    bool monitored(double until);

    std::shared_ptr<State> s_;
};

}  // namespace iai_actuator_driver
