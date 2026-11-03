// The status monitor thread, the fast completion rule and the 1 ms timer resolution (axis unit). Port of the Python
// edition's monitor.py: AxisMonitor (lean polls with the port lock free between them, waiters, callbacks, suspend,
// the link watch), fast_done, suspend_bus and the reference-counted timeBeginPeriod. The behaviour is VERIFIED in the
// Python edition where its documentation says so; this edition is UNVERIFIED on hardware (simulator only).
#include "iai_actuator_driver/monitor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#endif

#include "axis_impl.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "monotonic_cv.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr const char* kMonitorLog = "iai_actuator_driver.monitor";
constexpr unsigned kTimerrNoError = 0;
constexpr double kPosEpsilon = 1e-6;  ///< positions are 0.01 counts; this only absorbs float noise
constexpr double kSuspendedPollS = 0.05;

std::chrono::duration<double> seconds(double s) { return std::chrono::duration<double>(s > 0.0 ? s : 0.0); }

/// A timeout / CRC / frame error or an exception response: counted and retried.
bool transient(const std::exception& e) {
    return dynamic_cast<const ModbusTimeout*>(&e) || dynamic_cast<const ModbusCrcError*>(&e) ||
           dynamic_cast<const ModbusFrameError*>(&e) || dynamic_cast<const ModbusSlaveError*>(&e);
}

/// The error of a link check that ends the monitor: a Modbus error that is not transient (port gone, bus closed).
bool check_fatal(const std::exception_ptr& error) {
    if (!error) return false;
    try {
        std::rethrow_exception(error);
    } catch (const ModbusError& e) {
        return !transient(e);
    } catch (...) {
        return false;
    }
}

std::string what_of(const std::exception_ptr& error) {
    if (!error) return "no error";
    try {
        std::rethrow_exception(error);
    } catch (const std::exception& e) {
        return e.what();
    } catch (...) {
        return "unknown error";
    }
}

double wall_clock_s() {
    using clock = std::chrono::system_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

/// The words a change is seen on: PNOW, ALMC, DSS1, DSS2, DSSE.
bool same_key(const Status& a, const Status& b) {
    return a.raw.pnow == b.raw.pnow && a.raw.almc == b.raw.almc && a.raw.dss1 == b.raw.dss1 &&
           a.raw.dss2 == b.raw.dss2 && a.raw.dsse == b.raw.dsse;
}

template <class F>
struct Callback {
    explicit Callback(F f) : fn(std::move(f)) {}
    F fn;
    std::atomic<bool> failed{false};  ///< raised once: logged in full once
};

// ---- every monitor made, for suspend_bus() and Axis::close()
std::mutex& registry_mutex() {
    static std::mutex m;
    return m;
}
std::vector<std::weak_ptr<AxisMonitor>>& registry() {
    static std::vector<std::weak_ptr<AxisMonitor>> monitors;
    return monitors;
}

std::mutex& watch_mutex() {
    static std::mutex m;
    return m;
}
MonitorLinkWatch& watch_slot() {
    static MonitorLinkWatch watch;
    return watch;
}

}  // namespace

// ============================================================================================ the completion rule

const char* to_string(Completion completion) noexcept { return completion == Completion::Fast ? "fast" : "safe"; }

Completion parse_completion(const std::string& name) {
    if (name == "safe") return Completion::Safe;
    if (name == "fast") return Completion::Fast;
    throw std::invalid_argument("completion must be one of ('safe', 'fast'), got '" + name + "'");
}

bool fast_done(const Status& st, std::optional<double> target, std::optional<double> band) {
    if (!st.in_position || st.move_bit() || st.homing() || st.paused()) return false;
    if (!target) return true;
    return std::fabs(st.position_mm - *target) <= band.value_or(0.1) + kPosEpsilon;
}

// ============================================================================================ 1 ms timer resolution

TimerApi system_timer_api() {
    TimerApi api;
#ifdef _WIN32
    api.begin = [](unsigned ms) { return static_cast<unsigned>(timeBeginPeriod(ms)); };
    api.end = [](unsigned ms) { return static_cast<unsigned>(timeEndPeriod(ms)); };
#endif
    return api;
}

TimerResolution::TimerResolution(TimerApi api) : api_(std::move(api)) {}

bool TimerResolution::acquire(int ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++users_;
    if (users_ == 1 && api_.begin) {
        try {
            if (api_.begin(static_cast<unsigned>(ms)) == kTimerrNoError) period_ = static_cast<unsigned>(ms);
        } catch (const std::exception& e) {  // a timer tweak must never break a wait
            log_debug(kMonitorLog, util::format("timeBeginPeriod(%d) failed: %s", ms, e.what()));
        } catch (...) {
            log_debug(kMonitorLog, util::format("timeBeginPeriod(%d) failed", ms));
        }
    }
    return period_.has_value();
}

void TimerResolution::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (users_ == 0) return;
    --users_;
    if (users_ == 0 && period_) {
        const unsigned period = *period_;
        period_.reset();
        try {
            if (api_.end) api_.end(period);
        } catch (...) {
            log_debug(kMonitorLog, util::format("timeEndPeriod(%u) failed", period));
        }
    }
}

bool TimerResolution::active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return period_.has_value();
}

int TimerResolution::users() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return users_;
}

void TimerResolution::set_api(TimerApi api) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (users_ != 0) throw std::logic_error("TimerResolution::set_api while the timer has users");
    api_ = std::move(api);
}

TimerResolution& process_timer() {
    static TimerResolution timer;
    return timer;
}

ScopedTimerResolution::ScopedTimerResolution(int ms) : ScopedTimerResolution(process_timer(), ms) {}

ScopedTimerResolution::ScopedTimerResolution(TimerResolution& timer, int ms) : timer_(timer) {
    active_ = timer_.acquire(ms);
}

ScopedTimerResolution::~ScopedTimerResolution() { timer_.release(); }

// ============================================================================================ samples, events, results

double MonitorSample::age_s() const { return now_s() - t; }

Json LinkEvent::to_json() const {
    Json j = Json::object();
    j.set("kind", kind);
    j.set("axis", axis);
    j.set("message", message);
    j.set("cause", cause.empty() ? Json() : Json(cause));
    j.set("time", time);
    return j;
}

Json FastModeSummary::to_json() const {
    Json j = Json::object();
    j.set("baudrate", baud_rate);
    j.set("completion", iai_actuator_driver::to_string(completion));
    j.set("monitor", monitor);
    j.set("notes", notes);
    return j;
}

MonitorLinkWatch set_monitor_link_watch(MonitorLinkWatch watch) {
    std::lock_guard<std::mutex> lock(watch_mutex());
    MonitorLinkWatch previous = std::move(watch_slot());
    watch_slot() = std::move(watch);
    return previous;
}

MonitorLinkWatch monitor_link_watch() {
    std::lock_guard<std::mutex> lock(watch_mutex());
    return watch_slot();
}

// ============================================================================================ the monitor's state

struct AxisMonitor::Waiter {
    std::function<bool(const Status&)> predicate;  ///< empty: any sample
    long after_seq = 0;
    std::optional<double> since;
    bool done = false;                             ///< (mutex)
    std::optional<MonitorSample> sample;           ///< (mutex)
    std::exception_ptr error;                      ///< (mutex) the predicate's, or the monitor's failure

    /// Called with the monitor's mutex held.
    bool matches(const MonitorSample& s) {
        if (s.seq <= after_seq || (since && s.t_start < *since)) return false;
        if (!predicate) return true;
        try {
            return predicate(s.status);
        } catch (...) {  // handed to the waiting thread
            error = std::current_exception();
            return true;
        }
    }
};

struct AxisMonitor::State {
    std::atomic<Axis::Impl*> axis{nullptr};  ///< null once detached (Axis::close)
    // fixed settings
    double gap_s = kPollGapS;
    int retries = 0;
    std::optional<double> link_check_s;
    std::string name;

    // published state, waiters, lifecycle (mutex; cv wakes waiters, suspend(), stop() and the thread)
    mutable std::mutex mutex;
    detail::MonotonicCv cv;
    std::optional<MonitorSample> latest;
    long seq = 0;
    std::vector<std::shared_ptr<Waiter>> waiters;
    bool stop = false;
    int suspended = 0;
    bool idle = true;              ///< no poll in flight
    bool thread_alive = false;
    long generation = 0;           ///< start() count: a thread of an older start ends at its next check
    /// Cancelled by stop(): the thread's poll ends at once when it waits for the bus (cancel.hpp; a sticky scope)
    std::shared_ptr<CancelToken> cancel = std::make_shared<CancelToken>();
    std::thread thread;
    std::thread::id thread_id;
    std::exception_ptr error;      ///< the error that ended the thread
    std::exception_ptr last_error;
    long error_count = 0;
    long polls = 0;
    long polls_at_start = 0;
    long link_checks = 0;
    std::optional<MonitorLinkCheck> last_link_check;
    std::optional<double> started_at;
    std::string own_link_state = kLinkOk;

    // thread-only
    std::optional<bool> done_state;
    int fail_streak = 0;
    double next_link_check = 0.0;

    // callbacks (callbacks_mutex; called outside every lock)
    std::mutex callbacks_mutex;
    std::vector<std::shared_ptr<Callback<StatusChangeCallback>>> on_change;
    std::vector<std::shared_ptr<Callback<StatusDoneCallback>>> on_done;
    std::vector<std::shared_ptr<Callback<LinkEventCallback>>> on_event;

    template <class E, class... A>
    void call(E& entry, const A&... args) {
        try {
            entry.fn(args...);
        } catch (const std::exception& e) {  // a broken callback must never stop the monitor
            if (entry.failed.exchange(true))
                log_debug(kMonitorLog, name + ": callback failed again: " + e.what());
            else
                log_warning(kMonitorLog, name + ": callback failed (logged once): " + e.what());
        } catch (...) {
            if (!entry.failed.exchange(true)) log_warning(kMonitorLog, name + ": callback failed (logged once)");
        }
    }

    /// "COM3:1" (the Python edition's recovery._default_name without a session).
    std::string axis_label() const {
        Axis::Impl* ax = axis.load();
        if (!ax) return name;
        return ax->bus->name() + ":" + std::to_string(ax->slave);
    }
};

// ============================================================================================ AxisMonitor

AxisMonitor::AxisMonitor(std::unique_ptr<State> state) : s_(std::move(state)) {}

std::shared_ptr<AxisMonitor> AxisMonitor::create(Axis& axis, MonitorOptions o) {
    if (!axis.impl_) throw std::logic_error("AxisMonitor: the axis was moved from");
    if (!std::isfinite(o.gap_s) || !(o.gap_s >= 0.0 && o.gap_s <= 10.0))
        throw std::invalid_argument("gap_s must be 0..10 s, got " + util::g(o.gap_s));
    if (o.link_check_s && (!std::isfinite(*o.link_check_s) || !(*o.link_check_s >= 0.0 && *o.link_check_s <= 3600.0)))
        throw std::invalid_argument("link_check_s must be 0..3600 s or nullopt, got " + util::g(*o.link_check_s));
    if (o.retries < 0) throw std::invalid_argument("retries must be >= 0, got " + std::to_string(o.retries));
    auto st = std::make_unique<State>();
    st->axis = axis.impl_.get();
    st->gap_s = o.gap_s;
    st->retries = o.retries;
    if (o.link_check_s && *o.link_check_s > 0.0) st->link_check_s = o.link_check_s;
    st->name = o.name.empty() ? "ec-monitor-" + std::to_string(axis.impl_->slave) : o.name;
    for (auto& f : o.on_change)
        if (f) st->on_change.push_back(std::make_shared<Callback<StatusChangeCallback>>(std::move(f)));
    for (auto& f : o.on_done)
        if (f) st->on_done.push_back(std::make_shared<Callback<StatusDoneCallback>>(std::move(f)));
    for (auto& f : o.on_event)
        if (f) st->on_event.push_back(std::make_shared<Callback<LinkEventCallback>>(std::move(f)));
    std::shared_ptr<AxisMonitor> monitor(new AxisMonitor(std::move(st)));
    static std::once_flag close_hook_installed;  // closing a bus stops its monitors first (ModbusRtu::close)
    std::call_once(close_hook_installed, [] {
        detail::set_bus_close_hook([](const ModbusRtu& bus) { detail::stop_monitors_of_bus(bus); });
    });
    std::lock_guard<std::mutex> lock(registry_mutex());
    auto& all = registry();
    all.erase(std::remove_if(all.begin(), all.end(), [](const std::weak_ptr<AxisMonitor>& w) { return w.expired(); }),
              all.end());
    all.push_back(monitor);
    return monitor;
}

AxisMonitor::~AxisMonitor() {
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        s_->stop = true;
    }
    s_->cancel->cancel();
    s_->cv.notify_all();
    if (s_->thread.joinable()) {
        if (s_->thread.get_id() == std::this_thread::get_id())
            s_->thread.detach();  // the thread released the last reference while ending
        else
            s_->thread.join();
    }
}

AxisMonitor& AxisMonitor::start() {
    std::unique_lock<std::mutex> lock(s_->mutex);
    if (s_->thread_alive && !s_->stop) return *this;
    if (!s_->axis.load()) throw std::logic_error(s_->name + ": the axis of this monitor is closed");
    if (s_->thread.joinable()) {
        if (s_->thread.get_id() == std::this_thread::get_id()) {
            s_->thread.detach();  // restarted from its own callback: this thread ends after it
        } else {
            s_->cv.wait_for(lock, seconds(1.0), [&] { return !s_->thread_alive; });  // finishing its last poll
            if (s_->thread_alive) throw std::runtime_error(s_->name + ": the previous monitor thread has not ended yet");
            std::thread old = std::move(s_->thread);
            lock.unlock();
            old.join();
            lock.lock();
        }
    }
    s_->stop = false;
    s_->error = nullptr;
    s_->done_state.reset();
    s_->fail_streak = 0;
    s_->polls_at_start = s_->polls;
    s_->started_at = now_s();
    s_->next_link_check = *s_->started_at + s_->link_check_s.value_or(0.0);
    s_->thread_alive = true;
    const long generation = ++s_->generation;
    try {
        s_->thread = std::thread([self = shared_from_this(), generation] { self->run(generation); });
    } catch (...) {  // not started: not running
        s_->thread_alive = false;
        throw;
    }
    s_->thread_id = s_->thread.get_id();
    return *this;
}

bool AxisMonitor::stop(double timeout_s) {
    bool ended;
    {
        std::unique_lock<std::mutex> lock(s_->mutex);
        s_->stop = true;
        s_->cancel->cancel();  // a poll waiting for the bus lock ends at once
        s_->cv.notify_all();
        if (s_->thread.joinable() && s_->thread.get_id() != std::this_thread::get_id()) {
            s_->cv.wait_for(lock, seconds(timeout_s), [&] { return !s_->thread_alive; });
            if (!s_->thread_alive) {
                std::thread done = std::move(s_->thread);
                lock.unlock();
                done.join();
                lock.lock();
            }
        }
        ended = !s_->thread_alive;
    }
    wake_all(nullptr);
    return ended;
}

bool AxisMonitor::running() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->thread_alive && !s_->stop;
}

// ---- suspension

AxisMonitor::Suspension::Suspension(Suspension&& other) noexcept : monitor_(std::move(other.monitor_)) {}

AxisMonitor::Suspension& AxisMonitor::Suspension::operator=(Suspension&& other) noexcept {
    if (this != &other) {
        release();
        monitor_ = std::move(other.monitor_);
    }
    return *this;
}

AxisMonitor::Suspension::~Suspension() { release(); }

void AxisMonitor::Suspension::release() {
    if (!monitor_) return;
    State& s = *monitor_->s_;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        if (s.suspended > 0) --s.suspended;
    }
    s.cv.notify_all();
    monitor_.reset();
}

AxisMonitor::Suspension AxisMonitor::suspend(double timeout_s) {
    std::unique_lock<std::mutex> lock(s_->mutex);
    ++s_->suspended;
    s_->cv.wait_for(lock, seconds(timeout_s), [&] { return s_->idle; });
    return Suspension(shared_from_this());
}

bool AxisMonitor::suspended() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->suspended > 0;
}

// ---- published state

std::optional<MonitorSample> AxisMonitor::latest() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->latest;
}

std::optional<Status> AxisMonitor::status() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (!s_->latest) return std::nullopt;
    return s_->latest->status;
}

std::optional<double> AxisMonitor::timestamp() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (!s_->latest) return std::nullopt;
    return s_->latest->t;
}

std::optional<double> AxisMonitor::age_s() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (!s_->latest) return std::nullopt;
    return s_->latest->age_s();
}

std::optional<Status> AxisMonitor::fresh(double max_age_s) const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (!s_->latest || !(s_->thread_alive && !s_->stop) || s_->latest->age_s() > max_age_s) return std::nullopt;
    return s_->latest->status;
}

std::optional<double> AxisMonitor::poll_rate_hz() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    if (!s_->started_at) return std::nullopt;
    const double elapsed = now_s() - *s_->started_at;
    if (elapsed <= 0.0) return std::nullopt;
    return static_cast<double>(s_->polls - s_->polls_at_start) / elapsed;
}

std::string AxisMonitor::link_state() const {
    const MonitorLinkWatch watch = monitor_link_watch();
    Axis::Impl* ax = s_->axis.load();
    Axis* owner = ax ? ax->owner.load() : nullptr;
    if (watch.link_state && owner) {
        try {
            return watch.link_state(*owner);
        } catch (const std::exception& e) {
            log_debug(kMonitorLog, s_->name + ": link state not readable: " + e.what());
        }
    }
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->own_link_state;
}

bool AxisMonitor::is_done(const Status& st) const {
    std::optional<double> target, band;
    if (Axis::Impl* ax = s_->axis.load()) {
        std::lock_guard<std::mutex> lock(ax->mutex);
        if (ax->last_move) {
            target = reg::from_counts(ax->last_move->pcmd);
            band = reg::from_counts(ax->last_move->inp);
        }
    }
    return fast_done(st, target, band);
}

std::optional<std::pair<double, double>> AxisMonitor::done_target(const Axis& axis) {
    const Axis::Impl& ax = *axis.impl_;
    std::lock_guard<std::mutex> lock(ax.mutex);
    if (!ax.last_move) return std::nullopt;
    return std::make_pair(reg::from_counts(ax.last_move->pcmd), reg::from_counts(ax.last_move->inp));
}

// ---- waiting

std::optional<MonitorSample> AxisMonitor::wait_on(const std::shared_ptr<Waiter>& w, std::optional<double> timeout_s) {
    // a check point on every wait, also when the sample is there at once: a wait fed by a fast monitor never reaches
    // the slice check below and takes no bus request of its own (cancel.hpp)
    check_cancel();
    std::unique_lock<std::mutex> lock(s_->mutex);
    if (s_->error) throw ModbusError("status monitor stopped: " + what_of(s_->error));
    if (s_->latest && w->matches(*s_->latest)) {
        if (w->error) std::rethrow_exception(w->error);
        return s_->latest;
    }
    if (!(s_->thread_alive && !s_->stop)) return std::nullopt;
    s_->waiters.push_back(w);
    const auto ready = [&] { return w->done; };
    const auto give_up = [&] {
        auto& ws = s_->waiters;
        ws.erase(std::remove(ws.begin(), ws.end(), w), ws.end());
    };
    // in slices: a cancelled wait (Axis::cancel_waits; cancel.hpp) ends within kBusWaitSliceS
    const double deadline = timeout_s ? now_s() + *timeout_s : HUGE_VAL;
    while (!ready()) {
        const double left = deadline - now_s();
        if (left <= 0.0) {
            give_up();
            break;
        }
        s_->cv.wait_for(lock, seconds(std::min(left, kBusWaitSliceS)), ready);
        if (!ready() && cancel_pending()) {
            give_up();
            lock.unlock();
            check_cancel();  // throws WaitCancelledError
            return std::nullopt;
        }
    }
    if (w->error) std::rethrow_exception(w->error);
    return w->sample;
}

std::optional<MonitorSample> AxisMonitor::wait_update(std::optional<long> after_seq, std::optional<double> since,
                                                      std::optional<double> timeout_s) {
    auto w = std::make_shared<Waiter>();
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        w->after_seq = after_seq ? *after_seq : s_->seq;
    }
    w->since = since;
    return wait_on(w, timeout_s);
}

std::optional<Status> AxisMonitor::wait_for(const std::function<bool(const Status&)>& predicate,
                                            std::optional<double> timeout_s, std::optional<double> since) {
    auto w = std::make_shared<Waiter>();
    w->predicate = predicate ? predicate : [](const Status&) { return true; };
    w->since = since;
    const std::optional<MonitorSample> sample = wait_on(w, timeout_s);
    if (!sample) return std::nullopt;
    return sample->status;
}

std::optional<Status> AxisMonitor::wait_done(std::optional<double> timeout_s, std::optional<double> since) {
    return wait_for([this](const Status& st) { return is_done(st); }, timeout_s, since);
}

void AxisMonitor::wake_all(std::exception_ptr error) {
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        for (auto& w : s_->waiters) {
            if (error) w->error = std::make_exception_ptr(ModbusError("status monitor stopped: " + what_of(error)));
            w->done = true;
        }
        s_->waiters.clear();
    }
    s_->cv.notify_all();
}

// ---- callbacks

void AxisMonitor::add_on_change(StatusChangeCallback callback) {
    if (!callback) return;
    std::lock_guard<std::mutex> lock(s_->callbacks_mutex);
    s_->on_change.push_back(std::make_shared<Callback<StatusChangeCallback>>(std::move(callback)));
}

void AxisMonitor::add_on_done(StatusDoneCallback callback) {
    if (!callback) return;
    std::lock_guard<std::mutex> lock(s_->callbacks_mutex);
    s_->on_done.push_back(std::make_shared<Callback<StatusDoneCallback>>(std::move(callback)));
}

void AxisMonitor::add_on_event(LinkEventCallback callback) {
    if (!callback) return;
    std::lock_guard<std::mutex> lock(s_->callbacks_mutex);
    s_->on_event.push_back(std::make_shared<Callback<LinkEventCallback>>(std::move(callback)));
}

// ---- counters

long AxisMonitor::polls() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->polls;
}

long AxisMonitor::error_count() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->error_count;
}

long AxisMonitor::link_checks() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->link_checks;
}

std::optional<MonitorLinkCheck> AxisMonitor::last_link_check() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->last_link_check;
}

std::exception_ptr AxisMonitor::error() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->error;
}

std::exception_ptr AxisMonitor::last_error() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->last_error;
}

std::optional<double> AxisMonitor::started_at() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    return s_->started_at;
}

std::string AxisMonitor::name() const { return s_->name; }
double AxisMonitor::gap_s() const { return s_->gap_s; }
int AxisMonitor::retries() const { return s_->retries; }
std::optional<double> AxisMonitor::link_check_s() const { return s_->link_check_s; }

std::string AxisMonitor::to_string() const {
    std::lock_guard<std::mutex> lock(s_->mutex);
    const char* state = (s_->thread_alive && !s_->stop) ? "running" : (s_->error ? "failed" : "stopped");
    return "AxisMonitor('" + s_->name + "', " + state + ", polls=" + std::to_string(s_->polls) +
           ", errors=" + std::to_string(s_->error_count) + ")";
}

void AxisMonitor::fail(std::exception_ptr error) {
    if (!error) error = std::make_exception_ptr(ModbusError("status monitor failed"));
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        s_->error = error;
        s_->stop = true;
    }
    log_warning(kMonitorLog, s_->name + ": status monitor stopped: " + what_of(error));
    s_->cv.notify_all();
    wake_all(error);
}

// ---- the thread

void AxisMonitor::run(long generation) {
    // Every way out clears thread_alive (of this start) and wakes stop() and the waiters; an unexpected error ends the
    // monitor with that error recorded (error(), the waiters woken with it), never the process.
    struct Exit {
        State& s;
        long generation;
        ~Exit() {
            try {
                {
                    std::lock_guard<std::mutex> lock(s.mutex);
                    if (s.generation == generation) {
                        s.thread_alive = false;
                        s.idle = true;
                    }
                }
                s.cv.notify_all();
            } catch (...) {
            }
        }
    } done{*s_, generation};
    try {
        ScopedTimerResolution timer;
        const CancelScope cancel_scope(s_->cancel, true);  // stop(): a poll waiting for the bus ends at once
        double backoff = 0.0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(s_->mutex);
                if (s_->stop || s_->generation != generation) break;
                if (s_->suspended > 0) {
                    s_->cv.wait_for(lock, seconds(kSuspendedPollS), [&] { return s_->stop || s_->suspended == 0; });
                    continue;
                }
                s_->idle = false;
            }
            const bool end = poll_once(backoff);
            {
                std::lock_guard<std::mutex> lock(s_->mutex);
                s_->idle = true;
            }
            s_->cv.notify_all();
            if (end) break;
            const double pause = backoff > 0.0 ? backoff : s_->gap_s;  // the port lock is free here
            if (pause > 0.0) {
                std::unique_lock<std::mutex> lock(s_->mutex);
                s_->cv.wait_for(lock, seconds(pause), [&] { return s_->stop || s_->generation != generation; });
            }
        }
    } catch (...) {  // e.g. out of memory while publishing a sample
        const std::exception_ptr error = std::current_exception();
        try {
            bool current = false;
            {
                std::lock_guard<std::mutex> lock(s_->mutex);
                current = s_->generation == generation;
            }
            if (current) fail(error);  // a thread of an older start leaves the new one alone
        } catch (...) {  // recording it failed too: stopped on this error all the same
            try {
                std::lock_guard<std::mutex> lock(s_->mutex);
                if (s_->generation == generation) {
                    s_->error = error;
                    s_->stop = true;
                }
            } catch (...) {
            }
        }
    }
}

bool AxisMonitor::poll_once(double& backoff) {
    Axis::Impl* ax = s_->axis.load();
    if (!ax) return true;
    const double t0 = now_s();
    Status st;
    try {
        st = ax->read_lean_status(s_->retries);
    } catch (const WaitCancelledError&) {
        return true;  // stop(): the poll gave up waiting for the bus; nothing to report
    } catch (const std::exception& e) {  // sorted below; the thread must never die silently
        if (!transient(e)) {
            link_fault(e);
            fail(std::current_exception());
            return true;
        }
        int streak;
        {
            std::lock_guard<std::mutex> lock(s_->mutex);
            ++s_->error_count;
            s_->last_error = std::current_exception();
            streak = ++s_->fail_streak;
        }
        if (streak == kSilentAfter) link_fault(e);
        backoff = std::min(kBackoffMaxS, std::max(kBackoffMinS, backoff * 2.0));
        log_debug(kMonitorLog, s_->name + ": poll failed (" + e.what() + "), retrying in " + util::fixed(backoff, 3) +
                                   " s");
        return false;
    } catch (...) {
        fail(std::current_exception());
        return true;
    }
    backoff = 0.0;
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        s_->fail_streak = 0;
    }
    publish(st, t0, now_s());
    if (s_->link_check_s && now_s() >= s_->next_link_check) {
        {
            std::lock_guard<std::mutex> lock(s_->mutex);
            if (s_->stop) return true;  // stopped from a callback
        }
        const std::exception_ptr fatal = link_check();
        s_->next_link_check = now_s() + *s_->link_check_s;
        if (fatal) {
            fail(fatal);
            return true;
        }
    }
    return false;
}

void AxisMonitor::publish(Status st, double t0, double t1) {
    if (const auto& hook = detail::monitor_test_hooks().before_publish) hook(s_->name);
    std::optional<MonitorSample> previous;
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        previous = s_->latest;
        if (st.moving && previous) {
            const double dt = (t0 + t1) / 2.0 - (previous->t_start + previous->t) / 2.0;
            if (dt >= 0.005 && dt <= 1.0)
                st.velocity_mm_s = util::round2((st.position_mm - previous->status.position_mm) / dt);
        }
        MonitorSample sample;
        sample.status = st;
        sample.seq = ++s_->seq;
        sample.t_start = t0;
        sample.t = t1;
        s_->latest = sample;
        ++s_->polls;
        std::vector<std::shared_ptr<Waiter>> keep;
        for (auto& w : s_->waiters) {
            if (w->matches(sample)) {
                w->sample = sample;
                w->done = true;
            } else {
                keep.push_back(w);
            }
        }
        s_->waiters.swap(keep);
    }
    s_->cv.notify_all();
    std::vector<std::shared_ptr<Callback<StatusChangeCallback>>> changes;
    std::vector<std::shared_ptr<Callback<StatusDoneCallback>>> dones;
    {
        std::lock_guard<std::mutex> lock(s_->callbacks_mutex);
        changes = s_->on_change;
        dones = s_->on_done;
    }
    if (!changes.empty() && (!previous || !same_key(previous->status, st))) {
        std::optional<Status> before;
        if (previous) before = previous->status;
        for (auto& cb : changes) s_->call(*cb, st, before);
    }
    const bool done = is_done(st);
    if (done && s_->done_state == std::optional<bool>(false))
        for (auto& cb : dones) s_->call(*cb, st);
    s_->done_state = done;
}

// ---- the link watch

void AxisMonitor::emit(const LinkEvent& event) {
    std::vector<std::shared_ptr<Callback<LinkEventCallback>>> callbacks;
    {
        std::lock_guard<std::mutex> lock(s_->callbacks_mutex);
        callbacks = s_->on_event;
    }
    for (auto& cb : callbacks) s_->call(*cb, event);
}

void AxisMonitor::link_fault(const std::exception& error) {
    try {
        const MonitorLinkWatch watch = monitor_link_watch();
        Axis::Impl* ax = s_->axis.load();
        Axis* owner = ax ? ax->owner.load() : nullptr;
        if (owner && watch.recovering && watch.recovering(*owner)) return;
        if (owner && watch.link_fault) {
            watch.link_fault(*owner, error, [this](const LinkEvent& e) { emit(e); });
            return;
        }
        const std::string kind = detail::link_fault_kind(error);
        if (kind.empty()) return;
        std::string previous;
        {
            std::lock_guard<std::mutex> lock(s_->mutex);
            previous = s_->own_link_state;
            s_->own_link_state = kind;
        }
        if (previous == kLinkPortVanished || previous == kLinkUnitSilent) return;  // already lost: no new event
        LinkEvent event;
        event.kind = kEventLinkLost;
        event.axis = s_->axis_label();
        event.message = detail::error_type_name(error) + ": " + error.what();
        event.cause = kind;
        event.time = wall_clock_s();
        log_warning(kMonitorLog, event.axis + ": " + event.kind + ": " + event.message);
        emit(event);
    } catch (const std::exception& e) {  // the link watch must never break the monitor
        log_debug(kMonitorLog, s_->name + ": link state not recorded: " + e.what());
    } catch (...) {
        log_debug(kMonitorLog, s_->name + ": link state not recorded");
    }
}

std::exception_ptr AxisMonitor::link_check() {
    const MonitorLinkWatch watch = monitor_link_watch();
    if (!watch.check_link) return nullptr;  // no recovery module: no session, no check
    Axis::Impl* ax = s_->axis.load();
    Axis* owner = ax ? ax->owner.load() : nullptr;
    if (!owner) return nullptr;
    std::optional<MonitorLinkCheck> check;
    try {
        if (watch.recovering && watch.recovering(*owner)) return nullptr;
        check = watch.check_link(*owner, [this](const LinkEvent& e) { emit(e); });
    } catch (const WaitCancelledError&) {
        return nullptr;  // stop() during the check: the thread ends at its next loop check
    } catch (const std::exception& e) {  // not a link fault (the check reports those): counted, the monitor goes on
        std::lock_guard<std::mutex> lock(s_->mutex);
        ++s_->error_count;
        s_->last_error = std::current_exception();
        log_debug(kMonitorLog, s_->name + ": link check failed: " + e.what());
        return nullptr;
    } catch (...) {
        std::lock_guard<std::mutex> lock(s_->mutex);
        ++s_->error_count;
        s_->last_error = std::current_exception();
        return nullptr;
    }
    if (!check) return nullptr;
    {
        std::lock_guard<std::mutex> lock(s_->mutex);
        ++s_->link_checks;
        s_->last_link_check = check;
    }
    return check_fatal(check->error) ? check->error : nullptr;
}

// ============================================================================================ the bus and the axis

BusSuspension suspend_bus(const ModbusRtu& bus, double timeout_s) {
    std::vector<std::shared_ptr<AxisMonitor>> held;
    {
        std::lock_guard<std::mutex> lock(registry_mutex());
        for (const auto& w : registry()) {
            std::shared_ptr<AxisMonitor> m = w.lock();
            if (!m) continue;
            auto* ax = m->s_->axis.load();
            if (ax && ax->bus.get() == &bus && m->running()) held.push_back(m);
        }
    }
    std::vector<AxisMonitor::Suspension> suspensions;
    suspensions.reserve(held.size());
    for (auto& m : held) suspensions.push_back(m->suspend(timeout_s));
    return BusSuspension(std::move(suspensions), std::move(held));
}

namespace detail {

MonitorTestHooks& monitor_test_hooks() {
    static MonitorTestHooks hooks;
    return hooks;
}

void stop_monitors_of(const void* axis_key, double timeout_s) {
    std::vector<std::shared_ptr<AxisMonitor>> mine;
    {
        std::lock_guard<std::mutex> lock(registry_mutex());
        for (const auto& w : registry()) {
            std::shared_ptr<AxisMonitor> m = w.lock();
            if (m && static_cast<const void*>(m->s_->axis.load()) == axis_key) mine.push_back(m);
        }
    }
    for (auto& m : mine) {
        m->stop(timeout_s);
        AxisMonitor::State& s = *m->s_;
        std::unique_lock<std::mutex> lock(s.mutex);
        if (s.thread.joinable() && s.thread.get_id() != std::this_thread::get_id()) {
            std::thread t = std::move(s.thread);  // a poll in flight: it ends within its read timeout
            lock.unlock();
            t.join();
            lock.lock();
        }
        s.axis = nullptr;
    }
}

void stop_monitors_of_bus(const ModbusRtu& bus, double timeout_s) {
    std::vector<std::shared_ptr<AxisMonitor>> on_bus;
    {
        std::lock_guard<std::mutex> lock(registry_mutex());
        for (const auto& w : registry()) {
            std::shared_ptr<AxisMonitor> m = w.lock();
            if (!m) continue;
            auto* ax = m->s_->axis.load();
            if (ax && ax->bus.get() == &bus) on_bus.push_back(m);
        }
    }
    for (auto& m : on_bus) {
        m->stop(timeout_s);
        AxisMonitor::State& s = *m->s_;
        std::unique_lock<std::mutex> lock(s.mutex);
        if (s.thread.joinable() && s.thread.get_id() != std::this_thread::get_id()) {
            std::thread t = std::move(s.thread);  // a poll on the wire: it ends within its response timeout
            lock.unlock();
            t.join();
        }
    }
}

}  // namespace detail

}  // namespace iai_actuator_driver
