// Fast status path: a background status monitor per axis, the one-sample completion rule ("fast") and the 1 ms timer.
// Port of the Python edition's monitor.py (and the fast-path parts of ec.py); implemented by src/monitor.cpp and
// src/axis_fast.cpp.
//
// AxisMonitor is an optional thread per axis that reads the lean status block as fast as the link allows: PNOW, ALMC,
// DSS1 and DSSE in the smallest contiguous range, 0x9000-0x9007, 8 words (01 03 90 00 00 08; every word VERIFIED
// readable on the EC in the Python edition). Each poll is one FC03 transaction with no retries under the bus lock;
// between two polls the lock is free and the thread pauses gap_s (2 ms), so a STOP or a command from another thread,
// and the other axes of a shared port (each with its own monitor or none), get the port at once. It publishes the
// latest Status with its timestamps (latest()), wakes waiters (wait_for, wait_update, wait_done) and calls the
// on_change / on_done callbacks (in the monitor thread). A timeout / CRC / frame error or an exception response is
// counted and retried after a short back-off; a TransportError (the USB adapter unplugged: the COM port is gone), a
// closed bus or any other error ends the thread and every waiter throws it. stop() ends it cleanly (also done by
// Axis::close). A baud change and a software reset hold every monitor of the port meanwhile (suspend_bus): on a
// shared line they may change the rate under every axis.
//
// Link watch: the monitor also tells the application when the link fails, through on_event callbacks that get a
// LinkEvent. A TransportError, or kSilentAfter failed polls in a row, sets the axis's link state ("port_vanished" /
// "unit_silent": event "link_lost"). Every link_check_s (1 s) it runs the link check of the installed
// MonitorLinkWatch (the recovery module's: one full status read compared with the axis's session for restart signs,
// event "unit_restarted"); with no watch installed, or no session for the axis, no check runs and the monitor keeps
// the link state of its own polls. The monitor never recovers anything.
//
// The monitor only reads. It never decides anything for the axis: move_to uses its samples (a status younger than
// kFreshS that shows the axis ready replaces the pre-move status read; the waits take its samples instead of polling)
// and stop() never waits for it.
//
// fast_done is the Completion::Fast rule on one sample: PEND, |position - target| <= band and the DSSE MOVE bit clear
// (not homing, not paused), confirmed once (no settle window). On hardware PEND rose inside the band while the axis was
// still settling (EC-GRBP8M gripper: 3.04 mm, then 3.00 mm; VERIFIED in the Python edition): the fast rule accepts
// that sample by design (the position is already inside the band the caller asked for); the default Completion::Safe
// keeps the settle rule.
//
// ScopedTimerResolution raises the Windows timer resolution to 1 ms (winmm timeBeginPeriod / timeEndPeriod,
// reference-counted) while a wait or a monitor runs: the default granularity is about 15.6 ms (VERIFIED on the test PC
// in the Python edition), which every sleep of a polling loop would otherwise round up to. Nothing elsewhere.
//
// Evidence: the behaviour above is VERIFIED in the Python edition where it says so; this C++ code is UNVERIFIED on
// hardware (simulator only).
#pragma once

#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

class Axis;
class ModbusRtu;
class BusSuspension;

namespace detail {
/// Stop (joining the threads) and detach every monitor made on the axis state `axis_key` (Axis::close). Private.
void stop_monitors_of(const void* axis_key, double timeout_s = 1.0);
/// Stop (joining the threads) every running monitor of an axis on `bus` (ModbusRtu::close, Port::close). Private.
void stop_monitors_of_bus(const ModbusRtu& bus, double timeout_s = 1.0);
/// Process-wide test hooks of the status monitors. Tests only: never set in an application. Set them before a monitor
/// starts and clear them after it stopped.
struct MonitorTestHooks {
    /// Called in the monitor thread before each sample is published; a throw stands for an unexpected failure there.
    std::function<void(const std::string& monitor)> before_publish;
};
MonitorTestHooks& monitor_test_hooks();
}  // namespace detail

// ------------------------------------------------------------------------------------------ constants
inline constexpr int kLeanStatusWords = 8;      ///< 0x9000..0x9007: PNOW (2), ALMC, DIPM, DOPM, DSS1, DSS2, DSSE
inline constexpr double kFreshS = 0.05;         ///< a monitor status younger than this replaces the pre-move read
inline constexpr double kPollGapS = 0.002;      ///< pause between two monitor polls, with the port lock free
inline constexpr double kStallS = 0.25;         ///< a waiter reads itself when the monitor published nothing this long
inline constexpr double kBackoffMinS = 0.01;    ///< after a transient error (timeout / CRC / frame / exception) ...
inline constexpr double kBackoffMaxS = 0.5;     ///< ... doubling up to this
inline constexpr double kLinkCheckS = 1.0;      ///< the link check at most this often (when the axis has a session)
inline constexpr int kSilentAfter = 3;          ///< failed polls in a row that count as a lost link
inline constexpr double kFastPollS = 0.002;     ///< Completion::Fast without a monitor: pause between two lean reads
inline constexpr double kLateWaitS = 1.0;       ///< MoveHandle::wait past the estimate: still this long (+ settle_s)
inline constexpr int kFastBaudRate = 230400;    ///< enable_fast: the rate raised to (VERIFIED on the EC-R6S rod)

// ------------------------------------------------------------------------------------------ the completion rule

/// When a move counts as done.
enum class Completion {
    Safe,  ///< PEND (or PSFL), |position - target| <= band and PNOW unchanged over settle_s (the default)
    Fast,  ///< one sample: PEND, |position - target| <= band, DSSE MOVE clear, not homing, not paused (fast_done)
};

/// "safe" / "fast".
const char* to_string(Completion completion) noexcept;
/// The rule of a name ("safe", "fast"); std::invalid_argument otherwise (the Python ValueError text).
Completion parse_completion(const std::string& name);

/// The Completion::Fast rule on one sample: PEND set, |position - target| <= band (band 0.1 when nullopt; no position
/// check without a target), DSSE MOVE clear, not homing and not paused. No I/O.
bool fast_done(const Status& status, std::optional<double> target = std::nullopt,
               std::optional<double> band = std::nullopt);

// ------------------------------------------------------------------------------------------ 1 ms timer resolution

/// The two winmm calls (timeBeginPeriod / timeEndPeriod: 0 = TIMERR_NOERROR). Empty members: no timer to change.
struct TimerApi {
    std::function<unsigned(unsigned)> begin;
    std::function<unsigned(unsigned)> end;
};

/// winmm on Windows; an empty TimerApi elsewhere.
TimerApi system_timer_api();

/// Reference-counted timeBeginPeriod / timeEndPeriod: the first user raises the resolution, the last one restores it.
/// Never throws from acquire / release: a failure only leaves the default resolution. Thread-safe.
class TimerResolution {
public:
    explicit TimerResolution(TimerApi api = system_timer_api());
    /// One more user; true while a raised resolution is in force.
    bool acquire(int ms = 1);
    void release();
    /// A raised resolution is in force.
    bool active() const;
    int users() const;
    /// Replace the API (tests); std::logic_error while there are users.
    void set_api(TimerApi api);

private:
    mutable std::mutex mutex_;
    TimerApi api_;
    int users_ = 0;
    std::optional<unsigned> period_;  ///< the period begun, while active
};

/// The process-wide timer the waits and the monitors use.
TimerResolution& process_timer();

/// Hold the timer resolution at `ms` (1 ms) for the scope. A no-op (active() false) off Windows or when winmm refuses.
class ScopedTimerResolution {
public:
    explicit ScopedTimerResolution(int ms = 1);
    explicit ScopedTimerResolution(TimerResolution& timer, int ms = 1);
    ~ScopedTimerResolution();
    ScopedTimerResolution(const ScopedTimerResolution&) = delete;
    ScopedTimerResolution& operator=(const ScopedTimerResolution&) = delete;
    bool active() const { return active_; }

private:
    TimerResolution& timer_;
    bool active_ = false;
};

// ------------------------------------------------------------------------------------------ samples and events

/// One published poll: the decoded lean status, its sequence number and the monotonic times (now_s()) just before the
/// request went out (t_start) and after the answer was decoded (t).
struct MonitorSample {
    Status status;
    long seq = 0;
    double t_start = 0.0;
    double t = 0.0;
    /// Seconds since t.
    double age_s() const;
};

/// One change of an axis's link state (the Python edition's recovery.RecoveryEvent).
struct LinkEvent {
    std::string kind;     ///< "link_lost", "unit_restarted", "unit_replaced", "recovered"
    std::string axis;     ///< the axis ("COM3:1")
    std::string message;
    std::string cause;    ///< the link state behind it ("port_vanished", "unit_silent", ...); "" when none
    double time = 0.0;    ///< seconds since the Unix epoch
    Json to_json() const;
};

/// Link states the monitor records by itself.
inline constexpr const char* kLinkOk = "ok";
inline constexpr const char* kLinkPortVanished = "port_vanished";
inline constexpr const char* kLinkUnitSilent = "unit_silent";
inline constexpr const char* kEventLinkLost = "link_lost";

using StatusChangeCallback = std::function<void(const Status& status, const std::optional<Status>& previous)>;
using StatusDoneCallback = std::function<void(const Status& status)>;
using LinkEventCallback = std::function<void(const LinkEvent& event)>;

/// The result of one link check of an installed MonitorLinkWatch.
struct MonitorLinkCheck {
    std::string state = kLinkOk;       ///< "ok", "port_vanished", "unit_silent", "unit_restarted", "recovering"
    std::vector<std::string> reasons;
    std::exception_ptr error;          ///< the read error when the unit did not answer
};

/// The link watch every monitor runs, installed once per process by a recovery module (set_monitor_link_watch).
/// Every member is optional; none installed: the monitor records "port_vanished" / "unit_silent" itself (event
/// "link_lost" on entering a lost state) and runs no link check. Members run in the monitor thread; an exception they
/// throw is counted as a transient error (check_link) or logged (the others) and never ends the monitor.
struct MonitorLinkWatch {
    /// A recovery runs on the axis: no link fault is recorded and no check runs.
    std::function<bool(Axis& axis)> recovering;
    /// A failed poll that means a lost link (a TransportError, or kSilentAfter failures in a row): record the axis's
    /// link state and emit "link_lost" on a change.
    std::function<void(Axis& axis, const std::exception& error, const LinkEventCallback& emit)> link_fault;
    /// The axis's link state ("ok", ...).
    std::function<std::string(Axis& axis)> link_state;
    /// One link check (a full status read against the axis's session; a session may be made without I/O from the
    /// identity the axis read). nullopt: the axis has no session, no check ran.
    std::function<std::optional<MonitorLinkCheck>(Axis& axis, const LinkEventCallback& emit)> check_link;
};

/// Install (or, with an empty watch, remove) the process-wide link watch. Returns the previous one.
MonitorLinkWatch set_monitor_link_watch(MonitorLinkWatch watch);
/// The installed link watch.
MonitorLinkWatch monitor_link_watch();

// ------------------------------------------------------------------------------------------ the monitor

/// Settings of an AxisMonitor (Axis::start_monitor).
struct MonitorOptions {
    double gap_s = kPollGapS;                         ///< pause between two polls, port lock free: 0..10 s (0 back to back)
    std::vector<StatusChangeCallback> on_change;      ///< f(status, previous) when PNOW / ALMC / DSS1 / DSS2 / DSSE change
    std::vector<StatusDoneCallback> on_done;          ///< f(status) when the axis becomes done by is_done (a rising edge)
    std::vector<LinkEventCallback> on_event;          ///< link events ("link_lost", "unit_restarted", ...)
    int retries = 0;                                  ///< retries per poll (0: one attempt, then the lock is released)
    std::string name;                                 ///< thread / log name; "" = "ec-monitor-<slave>"
    std::optional<double> link_check_s = kLinkCheckS; ///< 0..3600 s; nullopt or 0: never (a lost link is still seen)
};

/// Background status thread for one axis (see the file comment). Usually made by Axis::start_monitor; read-only.
/// Made only through create() (shared ownership: suspend_bus and the axis hold it). Callbacks run in the monitor thread:
/// keep them short, never call a blocking axis command from them; an exception in a callback is logged once and does
/// not stop the monitor. A wait_for predicate runs with the monitor's lock held: it must not call the monitor.
/// The monitor is tied to its axis: Axis::close() stops it and detaches it (start() then throws std::logic_error).
class AxisMonitor : public std::enable_shared_from_this<AxisMonitor> {
public:
    /// A monitor for `axis` (not started). std::invalid_argument for an option out of range (nothing started).
    static std::shared_ptr<AxisMonitor> create(Axis& axis, MonitorOptions options = {});
    ~AxisMonitor();
    AxisMonitor(const AxisMonitor&) = delete;
    AxisMonitor& operator=(const AxisMonitor&) = delete;

    // ---- lifecycle
    /// Start the thread (no-op while it runs). std::runtime_error when the previous thread has not ended within 1 s,
    /// std::logic_error when the axis was closed.
    AxisMonitor& start();
    /// Stop the thread and wake every waiter (they get nullopt). True once the thread has ended within timeout_s. A poll
    /// waiting for the bus lock gives up at once (cancel.hpp); one on the wire ends within its response timeout. Also
    /// done by Axis::close and by closing the bus (ModbusRtu::close, Port::close, Cell::close): no "link_lost" event
    /// comes from a deliberate close.
    bool stop(double timeout_s = 1.0);
    bool running() const;

    /// While alive: no polls (it waits up to the timeout for a poll in flight). Nests. Movable.
    class Suspension {
    public:
        Suspension() = default;
        Suspension(Suspension&& other) noexcept;
        Suspension& operator=(Suspension&& other) noexcept;
        Suspension(const Suspension&) = delete;
        Suspension& operator=(const Suspension&) = delete;
        ~Suspension();
        /// End the suspension now (idempotent).
        void release();

    private:
        friend class AxisMonitor;
        explicit Suspension(std::shared_ptr<AxisMonitor> monitor) : monitor_(std::move(monitor)) {}
        std::shared_ptr<AxisMonitor> monitor_;
    };
    /// No polls while the returned Suspension lives: for a baud change or a software reset, where a poll at the wrong
    /// moment would only cost a timeout.
    Suspension suspend(double timeout_s = 1.0);

    // ---- the published state
    /// The last published sample (status + timestamps), nullopt before the first poll.
    std::optional<MonitorSample> latest() const;
    std::optional<Status> status() const;
    /// now_s() when the last status was read.
    std::optional<double> timestamp() const;
    std::optional<double> age_s() const;
    /// The latest status if the monitor runs and it is younger than max_age_s, else nullopt.
    std::optional<Status> fresh(double max_age_s = kFreshS) const;
    /// Answered polls per second since the last start().
    std::optional<double> poll_rate_hz() const;
    /// The axis's link state ("ok", "port_vanished", "unit_silent", ...): the installed watch's, else the monitor's own.
    std::string link_state() const;
    /// fast_done against the axis's last commanded 0x9900 target and band (when the axis knows them).
    bool is_done(const Status& status) const;
    /// (ABI 0.5, py-monitor) Polls are held: inside suspend() or suspend_bus().
    bool suspended() const;
    /// (ABI 0.5, py-monitor) The target and band is_done checks a sample against: the axis's last commanded 0x9900
    /// target and band, nullopt when it knows none. No monitor needed.
    static std::optional<std::pair<double, double>> done_target(const Axis& axis);

    // ---- waiting
    /// The first sample with seq > after_seq (nullopt: the next one) whose request went out at or after `since`
    /// (now_s()). nullopt on timeout or when the monitor stops; ModbusError when it failed (port gone).
    std::optional<MonitorSample> wait_update(std::optional<long> after_seq = std::nullopt,
                                             std::optional<double> since = std::nullopt,
                                             std::optional<double> timeout_s = std::nullopt);
    /// Block until a sample (the latest one included) satisfies `predicate`; returns its status. nullopt on timeout or
    /// when the monitor stops; ModbusError when it failed; the predicate's own exception when it threw.
    std::optional<Status> wait_for(const std::function<bool(const Status&)>& predicate,
                                   std::optional<double> timeout_s = std::nullopt,
                                   std::optional<double> since = std::nullopt);
    /// wait_for with is_done.
    std::optional<Status> wait_done(std::optional<double> timeout_s = std::nullopt,
                                    std::optional<double> since = std::nullopt);

    // ---- callbacks (thread-safe; added to the running monitor)
    void add_on_change(StatusChangeCallback callback);
    void add_on_done(StatusDoneCallback callback);
    void add_on_event(LinkEventCallback callback);

    // ---- counters and errors
    long polls() const;             ///< answered polls
    long error_count() const;       ///< transient errors in total
    long link_checks() const;       ///< link checks run
    std::optional<MonitorLinkCheck> last_link_check() const;
    std::exception_ptr error() const;       ///< the error that ended the thread (port gone, bus closed), else null
    std::exception_ptr last_error() const;  ///< the last transient error
    std::optional<double> started_at() const;
    std::string name() const;
    double gap_s() const;
    int retries() const;
    std::optional<double> link_check_s() const;
    /// "AxisMonitor('ec-monitor-1', running, polls=12, errors=0)".
    std::string to_string() const;

    /// End the thread on `error` as when the port vanished: waiters throw ModbusError, error() is set (tests; the
    /// Python edition's _fail).
    void fail(std::exception_ptr error);

    struct State;  ///< private (src/monitor.cpp)

private:
    friend BusSuspension suspend_bus(const ModbusRtu& bus, double timeout_s);
    friend void detail::stop_monitors_of(const void* axis_key, double timeout_s);
    friend void detail::stop_monitors_of_bus(const ModbusRtu& bus, double timeout_s);
    explicit AxisMonitor(std::unique_ptr<State> state);
    void run(long generation);
    bool poll_once(double& backoff);
    void publish(Status status, double t_start, double t);
    void link_fault(const std::exception& error);
    std::exception_ptr link_check();
    void emit(const LinkEvent& event);
    void wake_all(std::exception_ptr error);
    struct Waiter;
    std::optional<MonitorSample> wait_on(const std::shared_ptr<Waiter>& waiter, std::optional<double> timeout_s);
    std::unique_ptr<State> s_;
};

/// The monitors held by suspend_bus, released together. Movable.
class BusSuspension {
public:
    BusSuspension() = default;
    explicit BusSuspension(std::vector<AxisMonitor::Suspension> held, std::vector<std::shared_ptr<AxisMonitor>> monitors)
        : held_(std::move(held)), monitors_(std::move(monitors)) {}
    /// The monitors held.
    const std::vector<std::shared_ptr<AxisMonitor>>& monitors() const { return monitors_; }
    void release() { held_.clear(); }

private:
    std::vector<AxisMonitor::Suspension> held_;
    std::vector<std::shared_ptr<AxisMonitor>> monitors_;
};

/// Suspend every running monitor of an axis on `bus` while the result lives: for a broadcast baud change, a software
/// reset or a recovery, which change the line's rate or reopen the port under every axis of a shared line (a
/// sibling's poll at that moment would only fail and report a lost link).
BusSuspension suspend_bus(const ModbusRtu& bus, double timeout_s = 1.0);

/// What Axis::enable_fast / disable_fast did (the Python dict {"baudrate", "completion", "monitor", "notes"}).
struct FastModeSummary {
    std::optional<int> baud_rate;      ///< the bus's rate afterwards (nullopt: a transport without one)
    Completion completion = Completion::Safe;
    bool monitor = false;              ///< a status monitor runs
    std::vector<std::string> notes;    ///< what could not be done (the baud rate kept ...)
    Json to_json() const;
    bool operator==(const FastModeSummary& o) const {
        return baud_rate == o.baud_rate && completion == o.completion && monitor == o.monitor && notes == o.notes;
    }
};

}  // namespace iai_actuator_driver
