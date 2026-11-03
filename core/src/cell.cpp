// Cell: any number of axes over several ports, driven together. Ports run in parallel threads; the axes of one port
// take turns on its bus (read-only group commands and the move starts run port by port, the waits run one thread per
// axis and interleave on the shared bus). A link fault (TransportError, or no valid answer after the retries) calls
// on_fault once per axis and, with stop_on_fault, stops every other axis. The status monitors of start_monitors()
// feed the same fault handling with their link events (from a thread of its own), and status_all() takes their fresh
// samples. Port of the Python edition's cell.Cell (its group commands, _fault and _monitor_event; C# Cell.cs).
// UNVERIFIED on hardware in this edition (several axes per port is UNVERIFIED in every edition).
#include "iai_actuator_driver/cell.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include "axis_impl.hpp"
#include "iai_actuator_driver/log.hpp"
#include "bus_stop_timing.hpp"
#include "threads.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr const char* kCellLog = "iai_actuator_driver.cell";

std::string build_cell_message(const std::string& operation, const std::map<std::string, std::string>& messages) {
    std::vector<std::string> parts;
    for (const auto& kv : messages) parts.push_back(kv.first + ": " + kv.second);
    return operation + " failed on " + std::to_string(messages.size()) + " axis/axes: " + util::join(parts, "; ");
}

/// "1-4, 7".
std::string ranges_text(const std::vector<int>& values) {
    std::vector<int> v(values);
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    std::vector<std::string> parts;
    for (std::size_t i = 0; i < v.size();) {
        std::size_t j = i;
        while (j + 1 < v.size() && v[j + 1] == v[j] + 1) ++j;
        parts.push_back(j == i ? std::to_string(v[i]) : std::to_string(v[i]) + "-" + std::to_string(v[j]));
        i = j + 1;
    }
    return util::join(parts, ", ");
}

/// Run jobs in parallel threads (one runs inline) and wait for all. The jobs record their own errors; one that throws
/// anyway has its exception rethrown here once every job has ended, as is one of a thread that could not start (after
/// the started ones ended).
void run_threads(std::vector<std::function<void()>> jobs) {
    if (jobs.empty()) return;
    if (jobs.size() == 1) {
        jobs[0]();
        return;
    }
    std::vector<std::exception_ptr> errors(jobs.size());
    std::vector<std::thread> threads;
    threads.reserve(jobs.size());
    {
        const util::JoinThreads join(threads);  // also when a later thread cannot start
        for (std::size_t i = 0; i < jobs.size(); ++i)
            threads.emplace_back([&jobs, &errors, i] {
                try {
                    jobs[i]();
                } catch (...) {
                    errors[i] = std::current_exception();
                }
            });
    }
    for (const auto& e : errors)
        if (e) std::rethrow_exception(e);
}

template <class T>
struct Outcome {
    std::map<std::string, T> results;
    std::map<std::string, std::exception_ptr> errors;
};

}  // namespace

// ============================================================================================ CellError

CellError::CellError(std::string operation, std::map<std::string, std::string> messages,
                     std::map<std::string, std::exception_ptr> errors)
    : EcDriverError(build_cell_message(operation, messages)),
      operation_(std::move(operation)),
      messages_(std::move(messages)),
      errors_(std::move(errors)) {}

// ============================================================================================ Cell::Impl

struct Cell::Impl {
    struct Target {
        double position = 0.0;
        double band = 0.1;
        double timeout = 10.0;
    };
    struct PortSpec {
        std::string name;
        std::optional<std::vector<int>> listed;  ///< nullopt: scan
        std::function<std::shared_ptr<Port>()> create;
        bool fixed_rate = false;
    };
    struct Opened {
        std::shared_ptr<Port> port;
        std::vector<int> found;
        std::exception_ptr error;
    };

    mutable std::mutex mutex;
    std::vector<std::pair<std::string, std::shared_ptr<Port>>> ports;
    std::vector<std::string> names;
    std::map<std::string, std::shared_ptr<Axis>> axes;
    std::map<std::string, std::string> port_of;
    std::map<std::string, DeviceInfo> info;
    std::map<std::string, std::string> port_errors;
    std::map<std::string, std::exception_ptr> port_error_details;  ///< (py-cell) the exceptions of port_errors
    std::map<std::string, std::string> faults;
    std::map<std::string, Target> targets;
    bool closed = false;
    bool borrowed = false;  ///< (py-cell) Cell::assemble: destroyed without close(), it leaves its axes and ports open
    int stop_depth = 0;
    bool stop_on_fault = true;
    FaultCallback on_fault;
    // power / link loss (recovery.hpp)
    recovery::EventCallback on_event;
    std::deque<recovery::RecoveryEvent> events;                         ///< (mutex) the last 200
    std::vector<std::shared_ptr<recovery::LinkMonitor>> link_monitors;  ///< (mutex) stopped by close()
    // status monitors (start_monitors): their link events reach the cell through `route` (cut by close(), so a
    // monitor that outlives the cell never calls into it); the fault handling of such an event runs in a thread of
    // its own (it may stop every other axis), joined by recover_all() and close().
    struct Route {
        std::mutex mutex;
        Impl* impl = nullptr;
    };
    struct FaultThread {
        std::thread thread;
        std::shared_ptr<std::atomic<bool>> done;
    };
    std::shared_ptr<Route> route = std::make_shared<Route>();
    std::vector<FaultThread> fault_threads;                       ///< (mutex)
    std::map<std::string, std::weak_ptr<AxisMonitor>> routed;     ///< (mutex) monitors whose events reach the cell

    Impl() { route->impl = this; }

    static Cell open_core(std::vector<PortSpec> specs, bool strict, const CellOptions& o);
    static Opened open_one(const PortSpec& spec, const std::vector<int>& scan_list, double scan_timeout);

    void check_open() const {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) throw EcNotReadyError("cell is closed");
    }

    std::string no_axis(const std::string& n) const {
        return "no axis '" + n + "' in this cell (axes: " + util::join(names, ", ") + ")";
    }

    std::vector<std::string> select(const std::vector<std::string>& wanted) const {
        if (wanted.empty()) return names;
        std::vector<std::string> out;
        for (const auto& n : wanted) {
            if (!axes.count(n)) throw std::out_of_range(no_axis(n));
            if (std::find(out.begin(), out.end(), n) == out.end()) out.push_back(n);
        }
        return out;
    }

    /// The selection of the stop path (stop_all, pause_all): the listed axes this cell has, and the names it does not
    /// have in `unknown`. A STOP is never refused because of an unrelated bad name (an axis that dropped out of a
    /// reopened cell): the known axes are stopped first, the unknown names are reported afterwards (add_unknown).
    std::vector<std::string> select_known(const std::vector<std::string>& wanted, std::vector<std::string>& unknown) const {
        if (wanted.empty()) return names;
        std::vector<std::string> out;
        for (const auto& n : wanted) {
            std::vector<std::string>& to = axes.count(n) ? out : unknown;
            if (std::find(to.begin(), to.end(), n) == to.end()) to.push_back(n);
        }
        return out;
    }

    /// select_known's unknown names as errors of their own (std::out_of_range each), listed after the axes.
    template <class T>
    void add_unknown(Outcome<T>& out, std::vector<std::string>& listed, const std::vector<std::string>& unknown) const {
        for (const auto& n : unknown) {
            out.errors[n] = std::make_exception_ptr(std::out_of_range(no_axis(n)));
            listed.push_back(n);
        }
    }

    void forget_targets(const std::vector<std::string>& which) {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& n : which) targets.erase(n);
    }

    /// A failure on axis `name`: a link fault is recorded, on_fault is called once and - the first time, with
    /// stop_on_fault and not inside stop_all - every axis not faulted is stopped. Returns true when it stopped them.
    bool fault(const std::string& name, const std::exception_ptr& error) {
        std::string kind;
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& e) {
            const auto it = axes.find(name);
            kind = recovery::classify_failure(e, it != axes.end() ? it->second.get() : nullptr);
        } catch (...) {
        }
        if (kind.empty()) return false;
        return fault_kind(name, kind, detail::describe(error));
    }

    /// fault() inside a job's error handler: a failure of the fault handling itself is logged, never thrown (the job's
    /// own error is recorded already). False then.
    bool fault_quietly(const std::string& name, const std::exception_ptr& error) noexcept {
        try {
            return fault(name, error);
        } catch (...) {
            try {
                log_error(kCellLog, name + ": fault handling failed: " + detail::describe(std::current_exception()));
            } catch (...) {
            }
            return false;
        }
    }

    /// The fault handling of fault() for a known link state (`kind`: "port_vanished", "unit_silent",
    /// "unit_restarted", "unit_replaced"; "ok" / "recovering" are not faults). Also records the axis's link state and
    /// emits its event (recovery::set_link_state).
    bool fault_kind(const std::string& name, const std::string& kind, const std::string& message) {
        if (kind.empty() || kind == recovery::kLinkOk || kind == recovery::kRecovering) return false;
        bool first = false;
        bool stop = false;
        FaultCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            first = faults.count(name) == 0;
            faults[name] = kind;
            stop = first && stop_on_fault && stop_depth == 0 && !closed;
            targets.erase(name);
            callback = on_fault;
        }
        if (!first) return false;
        // STOP first: the other axes are stopped before anything is logged or reported (a log sink or the caller's
        // on_fault may block or wait for an interpreter's lock; the STOP never waits for them)
        std::vector<std::string> others;
        std::string stop_error;
        if (stop) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                for (const auto& n : names)
                    if (!faults.count(n)) others.push_back(n);
            }
            if (!others.empty()) {
                try {
                    stop_all(others, 5.0, true);
                } catch (const std::exception& e) {
                    stop_error = e.what();
                } catch (...) {
                    stop_error = "unknown error";
                }
            }
        }
        log_warning(kCellLog, name + ": " + kind + ": " + message);
        if (!others.empty()) {
            log_warning(kCellLog, name + ": " + kind + ": STOP sent to " + util::join(others, ", "));
            if (!stop_error.empty())
                log_debug(kCellLog, name + ": stop of the other axes not sent: " + stop_error);
        }
        if (const auto it = axes.find(name); it != axes.end())
            recovery::set_link_state(*it->second, kind, message, [this](const recovery::RecoveryEvent& e) { emit(e); },
                                     name);
        if (callback) {
            try {
                callback(name, kind, message);
            } catch (...) {
                log_error(kCellLog, "cell on_fault callback raised");
            }
        }
        return stop;
    }

    /// Keep a recovery event (the last 200) and pass it to on_event.
    void emit(const recovery::RecoveryEvent& event) {
        recovery::EventCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            events.push_back(event);
            while (events.size() > 200) events.pop_front();
            callback = on_event;
        }
        if (!callback) return;
        try {
            callback(event);
        } catch (...) {
            log_error(kCellLog, "cell on_event callback raised");  // a broken callback must never break the cell
        }
    }

    /// check_links' check of one axis: tracked first when it never was (the identity from open when read), a fault
    /// handed to fault_kind.
    recovery::LinkCheck check_link_one(const std::string& name, Axis& ax) {
        bool faulted = false;
        std::optional<DeviceInfo> known;
        {
            std::lock_guard<std::mutex> lock(mutex);
            faulted = faults.count(name) != 0;
            const auto it = info.find(name);
            if (it != info.end()) known = it->second;
        }
        if (!recovery::session_of(ax) && !faulted) {
            try {  // the baseline restart detection needs
                recovery::TrackOptions t;
                t.name = name;
                t.info = known;
                recovery::track(ax, t);
            } catch (const std::exception& e) {  // the link is down: check_link below says how
                log_debug(kCellLog, name + ": not tracked (" + e.what() + ")");
            }
        }
        recovery::CheckLinkOptions o;
        o.name = name;
        o.on_event = [this](const recovery::RecoveryEvent& e) { emit(e); };
        recovery::LinkCheck chk = recovery::check_link(ax, o);
        if (!chk.ok() && chk.state != recovery::kRecovering)
            fault_kind(name, chk.state, chk.error_text.empty() ? chk.state : chk.error_text);
        return chk;
    }

    /// A link event from an axis's status monitor (runs in the monitor thread): kept and passed to on_event; a fault
    /// ("port_vanished", "unit_silent", "unit_restarted", "unit_replaced") goes to fault_kind in a thread of its own,
    /// since that may stop every other axis. Port of the Python edition's Cell._monitor_event.
    void monitor_event(const std::string& name, const LinkEvent& event) {
        recovery::RecoveryEvent e;
        e.kind = event.kind;
        e.axis = name;  // the cell's name ("COM3:1"), as in check_links' and recover_all's events
        e.message = event.message;
        e.cause = event.cause;
        e.time = event.time;
        emit(e);
        const std::string kind = event.kind != recovery::kEventRecovered ? event.cause : std::string();
        if (kind != recovery::kPortVanished && kind != recovery::kUnitSilent && kind != recovery::kUnitRestarted &&
            kind != recovery::kUnitReplaced)
            return;
        const std::string message = event.message.empty() ? kind : event.message;
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) return;
        reap_fault_threads();
        fault_threads.reserve(fault_threads.size() + 1);  // the push_back below cannot throw with the thread started
        FaultThread t;
        t.done = std::make_shared<std::atomic<bool>>(false);
        const std::shared_ptr<std::atomic<bool>> done = t.done;
        t.thread = std::thread([this, name, kind, message, done]() {
            try {
                fault_kind(name, kind, message);
            } catch (...) {
                try {
                    log_error(kCellLog, name + ": fault handling failed: " + detail::describe(std::current_exception()));
                } catch (...) {
                }
            }
            done->store(true);
        });
        fault_threads.push_back(std::move(t));
    }

    /// Join the fault threads that have finished (mutex held).
    void reap_fault_threads() {
        for (auto it = fault_threads.begin(); it != fault_threads.end();) {
            if (it->done->load()) {
                if (it->thread.joinable()) it->thread.join();
                it = fault_threads.erase(it);
            } else {
                ++it;
            }
        }
    }

    /// Let the fault handling started by monitor events finish (the stop of the other axes) before a recovery changes
    /// the line under it: on a shared line that stop's frames would fail during the baud-rate search. A thread still
    /// running after timeout_s is left running (joined later). The calling thread is never waited for.
    void join_fault_threads(double timeout_s) {
        const double deadline = now_s() + timeout_s;
        for (;;) {
            bool pending = false;
            {
                std::lock_guard<std::mutex> lock(mutex);
                reap_fault_threads();
                for (const auto& t : fault_threads)
                    if (t.thread.get_id() != std::this_thread::get_id()) pending = true;
            }
            if (!pending || now_s() >= deadline) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    /// Run fn on every axis: ports in parallel threads, the axes of one port one after another.
    template <class T>
    Outcome<T> per_port(const std::vector<std::string>& which, const std::function<T(const std::string&, Axis&)>& fn,
                        bool watch = true) {
        std::vector<std::vector<std::string>> groups;
        std::map<std::string, std::size_t> group_of;
        for (const auto& n : which) {
            const std::string& p = port_of.at(n);
            auto it = group_of.find(p);
            if (it == group_of.end()) {
                group_of[p] = groups.size();
                groups.push_back({n});
            } else {
                groups[it->second].push_back(n);
            }
        }
        Outcome<T> out;
        std::mutex out_mutex;
        std::vector<std::function<void()>> jobs;
        for (const auto& group : groups) {
            jobs.push_back([&, group]() {
                // the axes of a port one after another: a stop latency line waits until the last one is done, so no
                // log handler runs between two first STOP frames (bus_stop_timing.hpp)
                const detail::StopLinesLater lines_later;
                for (const auto& n : group) {
                    try {
                        T value = fn(n, *axes.at(n));
                        std::lock_guard<std::mutex> lock(out_mutex);
                        out.results.emplace(n, std::move(value));
                    } catch (...) {
                        const std::exception_ptr e = std::current_exception();
                        {
                            std::lock_guard<std::mutex> lock(out_mutex);
                            out.errors[n] = e;
                        }
                        if (watch) fault_quietly(n, e);
                    }
                }
            });
        }
        run_threads(std::move(jobs));
        return out;
    }

    /// Run fn on every axis in its own thread (blocking waits on a shared port interleave on its bus).
    template <class T>
    Outcome<T> per_axis(const std::vector<std::string>& which, const std::function<T(const std::string&, Axis&)>& fn,
                        bool stop_others_on_error) {
        Outcome<T> out;
        std::mutex out_mutex;
        std::atomic<bool> stopped{false};
        std::vector<std::function<void()>> jobs;
        for (const auto& n : which) {
            jobs.push_back([&, n]() {
                try {
                    T value = fn(n, *axes.at(n));
                    std::lock_guard<std::mutex> lock(out_mutex);
                    out.results.emplace(n, std::move(value));
                } catch (...) {
                    const std::exception_ptr e = std::current_exception();
                    {
                        std::lock_guard<std::mutex> lock(out_mutex);
                        out.errors[n] = e;
                    }
                    if (fault_quietly(n, e)) stopped = true;  // the fault handling stopped every other axis already
                    if (stop_others_on_error && !stopped.exchange(true)) {
                        try {
                            std::vector<std::string> others;
                            for (const auto& o : which)
                                if (o != n) others.push_back(o);
                            if (!others.empty()) {
                                log_warning(kCellLog, n + " failed (" + detail::describe(e) + "): stopping " +
                                                          util::join(others, ", "));
                                stop_all(others, 5.0, true);
                            }
                        } catch (...) {  // the error of this axis is recorded above
                            log_warning(kCellLog, "stopping the other axes failed: " +
                                                      detail::describe(std::current_exception()));
                        }
                    }
                }
            });
        }
        run_threads(std::move(jobs));
        return out;
    }

    template <class T>
    CellResults<T> finish(const std::string& operation, const std::vector<std::string>& which, Outcome<T> out,
                          bool return_exceptions) {
        CellResults<T> r;
        r.operation = operation;
        r.names = which;
        for (const auto& kv : out.errors) r.messages[kv.first] = detail::describe(kv.second);
        r.errors = out.errors;
        r.results = std::move(out.results);
        if (!r.errors.empty() && !return_exceptions) throw CellError(operation, r.messages, r.errors);
        return r;
    }

    std::shared_ptr<Port> port_named(const std::string& port_name) const {
        for (const auto& kv : ports)
            if (kv.first == port_name) return kv.second;
        return nullptr;
    }

    /// The daisy-chain ports whose every axis is in `which`: {port name: axis names}. Only where the line carries
    /// broadcasts (never behind a gateway) and the port has two or more axes.
    std::map<std::string, std::vector<std::string>> broadcast_ports(const std::vector<std::string>& which) const {
        std::map<std::string, std::vector<std::string>> groups;
        for (const auto& n : which) groups[port_of.at(n)].push_back(n);
        std::map<std::string, std::vector<std::string>> out;
        for (const auto& kv : groups) {
            const std::shared_ptr<Port> port = port_named(kv.first);
            if (!port) continue;
            std::size_t on_port = 0;
            for (const auto& n : names)
                if (port_of.at(n) == kv.first) ++on_port;
            const std::shared_ptr<Line> line = port->line();
            if (line && line->allows(LineOperation::Broadcast) && on_port >= 2 && kv.second.size() == on_port)
                out[kv.first] = kv.second;
        }
        return out;
    }

    CellResults<Status> stop_all(const std::vector<std::string>& wanted, double timeout_s, bool return_exceptions,
                                 bool broadcast = false) {
        check_open();
        std::vector<std::string> unknown;
        const std::vector<std::string> which = select_known(wanted, unknown);
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++stop_depth;
            for (const auto& n : which) targets.erase(n);
        }
        struct Depth {
            Impl* self;
            ~Depth() {
                std::lock_guard<std::mutex> lock(self->mutex);
                --self->stop_depth;
            }
        } depth{this};
        // every axis is held by this stop from here until its full stop ends: no command of another thread releases
        // its STOP meanwhile (while it waits for its turn after the first STOP frame; Axis::Impl::group_stops)
        detail::GroupStopHolds holds;
        for (const auto& n : which) holds.add(n, axes.at(n));
        // broadcast: one STOP to every unit of a daisy chain first (UNVERIFIED on hardware: several units acting on it).
        std::map<std::string, std::vector<std::string>> sent;
        // no log handler between the broadcast STOPs and the first STOP frames (bus_stop_timing.hpp)
        std::optional<detail::StopLinesLater> lines_later;
        lines_later.emplace();
        if (broadcast) {
            for (const auto& kv : broadcast_ports(which)) {
                try {
                    port_named(kv.first)->broadcast_stop(true);
                    sent[kv.first] = kv.second;
                } catch (const EcDriverError& e) {
                    log_warning(kCellLog, kv.first + ": broadcast STOP not sent: " + detail::error_type_name(e) + ": " + e.what());
                }
            }
        }
        std::vector<std::string> first_names;
        for (const auto& n : which) {
            bool done = false;
            for (const auto& kv : sent)
                done = done || std::find(kv.second.begin(), kv.second.end(), n) != kv.second.end();
            const std::shared_ptr<Port> port = port_named(port_of.at(n));
            const std::shared_ptr<Line> line = port ? port->line() : nullptr;
            if (!done && (!line || line->rules().coils())) first_names.push_back(n);
        }
        // First the STOP coil ON to every axis (ports in parallel, the axes of a port back to back, nothing else in
        // between), then each axis holds STOP until it stands still and releases it. A line without coils (RCON) skips
        // the first pass: the axis's own stop sequence does the work.
        Outcome<bool> first = per_port<bool>(first_names, [](const std::string&, Axis& ax) {
            ax.begin_stop();
            return true;
        });
        lines_later.reset();
        Outcome<Status> out = per_axis<Status>(
            which,
            [timeout_s, &holds](const std::string& n, Axis& ax) {
                struct End {
                    detail::GroupStopHolds& holds;
                    const std::string& name;
                    ~End() { holds.end(name); }
                } end{holds, n};
                return ax.stop(false, timeout_s);
            },
            false);
        for (const auto& kv : sent) {
            bool failed = false;
            for (const auto& n : kv.second) failed = failed || out.errors.count(n) != 0;
            if (failed) {
                log_warning(kCellLog, kv.first + ": STOP left ON on the line (an axis did not confirm its stop)");
                continue;
            }
            try {
                port_named(kv.first)->broadcast_stop(false);
            } catch (const EcDriverError& e) {
                log_warning(kCellLog, kv.first + ": broadcast STOP release not sent: " + detail::error_type_name(e) + ": " + e.what());
            }
        }
        for (auto& kv : first.errors) {
            auto it = out.errors.find(kv.first);
            if (it == out.errors.end()) continue;
            try {
                std::rethrow_exception(it->second);
            } catch (EcDriverError& e) {
                e.add_note("the first STOP frame failed too: " + detail::describe(kv.second));
            } catch (...) {
            }
        }
        std::vector<std::string> listed = which;
        add_unknown(out, listed, unknown);  // after the STOPs: an unknown name never holds back the others
        return finish("stop_all", listed, std::move(out), return_exceptions);
    }

    std::optional<CellResults<Status>> move_each(const std::string& operation, const std::vector<AxisMove>& moves,
                                                 bool wait, std::optional<double> timeout_s, bool return_exceptions,
                                                 bool stop_others_on_error = true);

    CellResults<Status> wait_all(const std::vector<std::string>& wanted, std::optional<double> timeout_s,
                                 bool return_exceptions, bool stop_others_on_error = true) {
        check_open();
        const std::vector<std::string> which = select(wanted);
        Outcome<Status> out = per_axis<Status>(
            which,
            [this, timeout_s](const std::string& n, Axis& ax) {
                std::optional<Target> target;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    auto it = targets.find(n);
                    if (it != targets.end()) target = it->second;
                }
                if (!target) return ax.wait_idle(std::nullopt, std::nullopt, timeout_s.value_or(10.0));
                const Status st = ax.wait_idle(target->position, target->band, timeout_s.value_or(target->timeout));
                std::lock_guard<std::mutex> lock(mutex);
                targets.erase(n);
                return st;
            },
            stop_others_on_error);
        return finish("wait_all", which, std::move(out), return_exceptions);
    }

    /// `members` false (py-cell: a borrowed cell destroyed without close()): the axes and ports stay open.
    void close(bool members = true) {
        std::vector<std::shared_ptr<recovery::LinkMonitor>> monitors;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (closed) return;
            closed = true;
            monitors.swap(link_monitors);
        }
        {
            std::lock_guard<std::mutex> lock(route->mutex);  // waits for a monitor event being handed over
            route->impl = nullptr;
        }
        for (const auto& m : monitors) m->stop();  // outside the mutex: a pass in progress may need it
        // the fault handling started by monitor events ends before the axes close (its stop of the other axes is
        // refused once closed; one in progress finishes). Not the calling thread (an on_fault callback closing the cell).
        std::vector<FaultThread> threads;
        {
            std::lock_guard<std::mutex> lock(mutex);
            threads.swap(fault_threads);
        }
        for (auto& t : threads) {
            if (!t.thread.joinable()) continue;
            if (t.thread.get_id() == std::this_thread::get_id())
                t.thread.detach();
            else
                t.thread.join();
        }
        if (!members) return;
        for (const auto& n : names) {
            try {
                axes.at(n)->close();  // releases a STOP coil it holds; the shared bus stays with the port
            } catch (const std::exception& e) {
                log_warning(kCellLog, "closing " + n + ": " + e.what());
            }
        }
        for (const auto& p : ports) {
            try {
                p.second->close();
            } catch (const std::exception& e) {
                log_warning(kCellLog, "closing " + p.first + ": " + e.what());
            }
        }
    }
};

std::optional<CellResults<Status>> Cell::Impl::move_each(const std::string& operation, const std::vector<AxisMove>& moves,
                                                         bool wait, std::optional<double> timeout_s,
                                                         bool return_exceptions, bool stop_others_on_error) {
    check_open();
    detail::check_timeout(timeout_s);  // the call's; each move's own is checked in pass 1 (Axis::check_move)
    std::map<std::string, AxisMove> plan;
    std::vector<std::string> which;
    for (const auto& m : moves) {
        if (!axes.count(m.axis))
            throw std::out_of_range("no axis '" + m.axis + "' in this cell (axes: " + util::join(names, ", ") + ")");
        if (plan.count(m.axis)) throw std::invalid_argument("two moves for axis " + m.axis);
        plan[m.axis] = m;
        which.push_back(m.axis);
    }
    // Pass 1, nothing sent: every value against its own unit's ranges, every axis ready to move.
    Outcome<Status> refused = per_port<Status>(which, [&plan](const std::string& n, Axis& ax) {
        const AxisMove& m = plan.at(n);
        return ax.check_move(m.position, m.options);
    });
    if (!refused.errors.empty()) {
        std::map<std::string, std::string> messages;
        for (const auto& kv : refused.errors) messages[kv.first] = detail::describe(kv.second);
        throw CellError(operation + " (check; nothing started)", messages, refused.errors);
    }
    // Pass 2: the starts, with as little delay as possible (ports in parallel, the axes of a port back to back).
    Outcome<bool> started = per_port<bool>(which, [this, &plan, timeout_s](const std::string& n, Axis& ax) {
        const AxisMove& m = plan.at(n);
        {
            std::lock_guard<std::mutex> lock(mutex);
            targets.erase(n);
        }
        const Status before = ax.status();
        MoveOptions o = m.options;
        o.wait = false;
        recovery::note_target(ax, m.position);  // the target recover() compares with (also for a command-profile move)
        ax.move_to(m.position, o);
        // a command-profile move (decel_g other than accel_g) is estimated with the slower of the two
        const double accel = o.decel_g ? std::min(o.accel_g, *o.decel_g) : o.accel_g;
        const double estimate =
            detail::default_move_timeout_s(std::fabs(m.position - before.position_mm), o.speed, accel);
        Target t;
        t.position = m.position;
        t.band = o.band;
        t.timeout = timeout_s ? *timeout_s : (m.options.timeout_s ? *m.options.timeout_s : estimate);
        std::lock_guard<std::mutex> lock(mutex);
        targets[n] = t;
        return true;
    });
    if (!started.errors.empty()) {
        std::vector<std::string> running;
        for (const auto& n : which)
            if (!started.errors.count(n)) running.push_back(n);
        if (!running.empty() && stop_others_on_error) {
            try {
                stop_all(running, 5.0, true);
            } catch (const std::exception& e) {
                log_warning(kCellLog, std::string("stopping the started axes failed: ") + e.what());
            }
        }
        std::map<std::string, std::string> messages;
        for (const auto& kv : started.errors) messages[kv.first] = detail::describe(kv.second);
        throw CellError(operation + " (start)", messages, started.errors);
    }
    if (!wait) return std::nullopt;
    return wait_all(which, timeout_s, return_exceptions, stop_others_on_error);
}

Cell::Impl::Opened Cell::Impl::open_one(const PortSpec& spec, const std::vector<int>& scan_list, double scan_timeout) {
    Opened r;
    try {
        r.port = spec.create();
    } catch (...) {
        r.error = std::current_exception();
        return r;
    }
    try {
        if (!spec.listed) {
            // (py-cell) Port::scan settles the rate: a line where no rate answers is a scan that found nothing
            r.found = r.port->scan(scan_list, scan_timeout);
            if (r.found.empty() && !r.port->line()->rules().axis_access)  // ROBONET gateway mode (line.hpp)
                throw UnsupportedLineError(spec.name + ": " + r.port->line()->explain().value_or(kRoboNetSw2));
            if (r.found.empty()) {
                ModbusTimeout error(spec.name + ": no axis answered at addresses " + ranges_text(scan_list));
                if (const std::optional<std::string> why = r.port->line()->explain()) error.add_note(*why);
                error.add_note(kNoAnswerHint);
                throw error;
            }
        } else if (!spec.listed->empty()) {
            if (!spec.fixed_rate) r.port->ensure_baud_rate(*spec.listed);
            const std::shared_ptr<Line> line = r.port->detect_line();  // READ-ONLY probes, once per port
            if (!line->rules().axis_access)
                throw UnsupportedLineError(spec.name + ": " + line->explain().value_or(kRoboNetSw2));
            const double read_timeout = r.port->bus()->transport().read_timeout();
            double wait = std::max(read_timeout, scan_timeout);
            if (line->is_gateway()) wait = std::max(wait, line->rules().probe_timeout_s);
            std::vector<int> missing;
            for (int s : *spec.listed)
                if (!r.port->bus()->probe(s, wait)) missing.push_back(s);
            if (!missing.empty()) {
                ModbusTimeout error(spec.name + ": slave(s) " + ranges_text(missing) + " did not answer");
                if (line->kind() == LineKind::Rcon) line->note_unreachable(missing);  // (py-cell) as a scan notes them
                if (const std::optional<std::string> why = line->explain_unreachable(missing)) error.add_note(*why);
                error.add_note(kNoAnswerHint);
                throw error;
            }
            r.found = *spec.listed;
        }
    } catch (...) {
        r.error = std::current_exception();
        try {
            r.port->close();
        } catch (...) {
        }
        r.port.reset();
    }
    return r;
}

Cell Cell::Impl::open_core(std::vector<PortSpec> specs, bool strict, const CellOptions& o) {
    if (o.max_axes && *o.max_axes < 1) throw std::invalid_argument("max_axes must be >= 1");
    if (specs.empty()) throw EcNotReadyError("no port given");
    std::set<std::string> seen;
    for (const auto& s : specs) {
        if (s.name.empty()) throw std::invalid_argument("port name required");
        if (!seen.insert(s.name).second) throw std::invalid_argument("port " + s.name + " given twice");
    }
    std::vector<int> scan_list;
    for (int s : o.scan_slaves)
        if (std::find(scan_list.begin(), scan_list.end(), s) == scan_list.end()) scan_list.push_back(s);
    std::vector<int> all(scan_list);
    for (const auto& s : specs)
        if (s.listed) all.insert(all.end(), s.listed->begin(), s.listed->end());
    for (int s : all)
        if (s < 1 || s > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(s));

    std::vector<Opened> outcomes(specs.size());
    std::vector<std::function<void()>> jobs;
    for (std::size_t i = 0; i < specs.size(); ++i)
        jobs.push_back([&, i]() { outcomes[i] = open_one(specs[i], scan_list, o.scan_timeout_s); });
    run_threads(std::move(jobs));

    std::vector<std::pair<std::string, Opened>> opened;
    std::vector<std::pair<std::string, std::exception_ptr>> errors;
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (outcomes[i].error)
            errors.emplace_back(specs[i].name, outcomes[i].error);
        else
            opened.emplace_back(specs[i].name, outcomes[i]);
    }
    auto close_opened = [&opened]() {
        for (auto& op : opened) {
            try {
                op.second.port->close();
            } catch (...) {
            }
        }
    };
    for (const auto& e : errors) {  // a non-standard exception is a bug: raised as it is
        try {
            std::rethrow_exception(e.second);
        } catch (const std::exception&) {
        } catch (...) {
            close_opened();
            throw;
        }
    }
    if (!errors.empty() && strict) {
        close_opened();
        if (errors.size() > 1) {
            std::vector<std::string> others;
            for (const auto& e : errors) others.push_back(e.first + ": " + detail::describe(e.second));
            try {
                std::rethrow_exception(errors.front().second);
            } catch (EcDriverError& first) {
                first.add_note("other ports failed too: " + util::join(others, "; "));
                throw;
            }
        }
        std::rethrow_exception(errors.front().second);
    }
    std::map<std::string, std::string> port_errors;
    std::map<std::string, std::exception_ptr> port_error_details;
    for (const auto& e : errors) {
        port_error_details[e.first] = e.second;
        port_errors[e.first] = detail::describe(e.second);
        log_warning(kCellLog, "port " + e.first + " skipped: " + port_errors[e.first]);
    }
    std::vector<std::pair<std::size_t, int>> pairs;  // (index in opened, slave)
    for (std::size_t i = 0; i < opened.size(); ++i) {
        std::vector<int> found = opened[i].second.found;
        std::sort(found.begin(), found.end());
        found.erase(std::unique(found.begin(), found.end()), found.end());
        for (int s : found) pairs.emplace_back(i, s);
    }
    if (o.max_axes && static_cast<int>(pairs.size()) > *o.max_axes) {
        if (strict) {
            close_opened();
            throw std::invalid_argument(std::to_string(pairs.size()) + " axes listed, max_axes is " +
                                        std::to_string(*o.max_axes));
        }
        log_warning(kCellLog, "scan found " + std::to_string(pairs.size()) + " axes; the cell keeps the first " +
                                  std::to_string(*o.max_axes) + " (max_axes)");
        pairs.resize(static_cast<std::size_t>(*o.max_axes));
    }
    if (pairs.empty()) {
        close_opened();
        std::vector<std::string> detail_parts;
        for (const auto& kv : port_errors) detail_parts.push_back(kv.first + ": " + kv.second);
        throw EcNotReadyError("no axis found (" + (detail_parts.empty() ? std::string("no slave listed")
                                                                          : util::join(detail_parts, "; ")) +
                              ")");
    }
    Cell cell;
    Impl& im = *cell.impl_;
    im.port_errors = port_errors;
    im.port_error_details = port_error_details;
    im.stop_on_fault = o.stop_on_fault;
    im.on_fault = o.on_fault;
    im.on_event = o.on_event;
    std::set<std::size_t> used;
    try {
        for (const auto& pr : pairs) {
            const std::string& pname = opened[pr.first].first;
            const std::shared_ptr<Port>& port = opened[pr.first].second.port;
            const std::string name = axis_name(pname, pr.second);
            im.axes[name] = std::make_shared<Axis>(*port, pr.second, o.axis);
            im.names.push_back(name);
            im.port_of[name] = pname;
            used.insert(pr.first);
        }
    } catch (...) {
        close_opened();
        throw;
    }
    for (std::size_t i = 0; i < opened.size(); ++i) {
        if (used.count(i))
            im.ports.emplace_back(opened[i].first, opened[i].second.port);
        else
            try {
                opened[i].second.port->close();
            } catch (...) {
            }
    }
    if (o.identify) {
        try {
            cell.identify_all();
        } catch (...) {
            cell.close();
            throw;
        }
    }
    return cell;
}

// ============================================================================================ Cell: construction

Cell::Cell() : impl_(std::make_unique<Impl>()) {}

Cell::~Cell() {
    if (!impl_) return;
    try {
        impl_->close(!impl_->borrowed);
    } catch (...) {
    }
}

Cell::Cell(Cell&& other) noexcept = default;

Cell& Cell::operator=(Cell&& other) noexcept {
    if (this != &other) {
        if (impl_) {
            try {
                impl_->close(!impl_->borrowed);
            } catch (...) {
            }
        }
        impl_ = std::move(other.impl_);
    }
    return *this;
}

namespace {
PortOptions port_options(const CellOptions& o) {
    PortOptions p;
    p.baud_rate = o.baud_rate;
    p.read_timeout_s = o.read_timeout_s;
    p.bus = o.bus;
    p.line = o.line;
    return p;
}
}  // namespace

Cell Cell::open(const CellOptions& options) {
    const std::vector<std::string> names = iai_port_names();
    if (names.empty()) throw EcNotReadyError("no IAI USB adapter found (list_ports); name the ports to open");
    return open(names, options);
}

Cell Cell::open(const std::vector<std::string>& ports, const CellOptions& options) {
    std::vector<Impl::PortSpec> specs;
    for (const auto& name : ports) {
        Impl::PortSpec s;
        s.name = name;
        s.fixed_rate = options.baud_rate.has_value();
        const PortOptions po = port_options(options);
        s.create = [name, po]() { return Port::open(name, po); };
        specs.push_back(std::move(s));
    }
    return Impl::open_core(std::move(specs), false, options);
}

Cell Cell::open(const std::map<std::string, std::vector<int>>& slaves_by_port, const CellOptions& options) {
    std::vector<Impl::PortSpec> specs;
    for (const auto& kv : slaves_by_port) {
        Impl::PortSpec s;
        s.name = kv.first;
        s.listed = kv.second;
        s.fixed_rate = options.baud_rate.has_value();
        const PortOptions po = port_options(options);
        const std::string name = kv.first;
        s.create = [name, po]() { return Port::open(name, po); };
        specs.push_back(std::move(s));
    }
    return Impl::open_core(std::move(specs), true, options);
}

Cell Cell::open(const std::map<std::string, std::shared_ptr<Transport>>& transports, const std::vector<int>& slaves,
                const CellOptions& options) {
    std::vector<Impl::PortSpec> specs;
    for (const auto& kv : transports) {
        if (!kv.second) throw std::invalid_argument("no transport for " + kv.first);
        Impl::PortSpec s;
        s.name = kv.first;
        if (!slaves.empty()) s.listed = slaves;
        const bool fixed = options.baud_rate.has_value() && kv.second->baud_rate() != 0;
        s.fixed_rate = fixed;
        const std::shared_ptr<Transport> transport = kv.second;
        const std::string name = kv.first;
        const BusOptions bus = options.bus;
        const std::optional<int> rate = options.baud_rate;
        const LineOptions line = options.line;
        s.create = [transport, name, bus, rate, fixed, line]() {
            if (fixed && transport->baud_rate() != *rate) transport->set_baud_rate(*rate);
            return std::make_shared<Port>(transport, name, bus, line);
        };
        specs.push_back(std::move(s));
    }
    return Impl::open_core(std::move(specs), !slaves.empty(), options);
}

// (py-cell) A cell over ports that are open already (iad_cell_open_buses), and a cell of axes that exist already
// (iad_cell_open_axes: the bindings' Cell over their own Port::axis objects).
Cell Cell::open(const std::vector<std::shared_ptr<Port>>& ports,
                const std::optional<std::map<std::string, std::vector<int>>>& slaves, const CellOptions& options) {
    std::vector<Impl::PortSpec> specs;
    for (const auto& port : ports) {
        if (!port) throw std::invalid_argument("no port");
        Impl::PortSpec s;
        s.name = port->name();
        if (slaves) {
            const auto it = slaves->find(s.name);
            s.listed = it == slaves->end() ? std::vector<int>() : it->second;
        }
        const std::shared_ptr<Port> given = port;
        s.create = [given]() { return given; };
        specs.push_back(std::move(s));
    }
    if (slaves) {
        for (const auto& kv : *slaves) {
            bool known = false;
            for (const auto& s : specs) known = known || s.name == kv.first;
            if (!known) throw std::invalid_argument("slaves names port " + kv.first + ", which is not given");
        }
    }
    Cell cell = Impl::open_core(std::move(specs), slaves.has_value(), options);
    cell.impl_->borrowed = true;  // the ports are the caller's: destroyed without close(), the cell leaves them open
    return cell;
}

Cell Cell::assemble(const std::vector<CellMember>& members, const CellOptions& options) {
    if (options.max_axes && *options.max_axes < 1) throw std::invalid_argument("max_axes must be >= 1");
    if (members.empty()) throw std::invalid_argument("no axis given");
    if (options.max_axes && static_cast<int>(members.size()) > *options.max_axes)
        throw std::invalid_argument(std::to_string(members.size()) + " axes listed, max_axes is " +
                                    std::to_string(*options.max_axes));
    Cell cell;
    Impl& im = *cell.impl_;
    im.borrowed = true;  // a refusal below leaves the given axes and ports as they are
    for (const auto& m : members) {
        if (!m.axis || !m.port) throw std::invalid_argument("axis " + m.name + ": no axis or no port");
        if (m.name.empty()) throw std::invalid_argument("an axis needs a name");
        if (im.axes.count(m.name)) throw std::invalid_argument("two axes named " + m.name);
        if (m.axis->bus() != m.port->bus())
            throw std::invalid_argument("axis " + m.name + " is not on the bus of port " + m.port->name());
        bool listed = false;
        for (const auto& p : im.ports) {
            if (p.second == m.port) listed = true;
            else if (p.first == m.port->name())
                throw std::invalid_argument("two ports named " + p.first + ": port names must be unique in a cell");
        }
        if (!listed) im.ports.emplace_back(m.port->name(), m.port);
        im.axes[m.name] = m.axis;
        im.names.push_back(m.name);
        im.port_of[m.name] = m.port->name();
    }
    im.stop_on_fault = options.stop_on_fault;
    im.on_fault = options.on_fault;
    im.on_event = options.on_event;
    if (options.identify) cell.identify_all();
    return cell;
}

// ============================================================================================ Cell: access

std::vector<std::string> Cell::names() const { return impl_->names; }
std::size_t Cell::size() const { return impl_->names.size(); }
bool Cell::contains(const std::string& name) const { return impl_->axes.count(name) != 0; }

Axis& Cell::operator[](const std::string& name) {
    auto it = impl_->axes.find(name);
    if (it == impl_->axes.end())
        throw std::out_of_range("no axis '" + name + "' in this cell (axes: " + util::join(impl_->names, ", ") + ")");
    return *it->second;
}

Axis& Cell::operator[](std::size_t index) { return *impl_->axes.at(impl_->names.at(index)); }

std::shared_ptr<Axis> Cell::axis_ptr(const std::string& name) const {
    auto it = impl_->axes.find(name);
    if (it == impl_->axes.end()) throw std::out_of_range("no axis '" + name + "' in this cell");
    return it->second;
}

std::shared_ptr<Port> Cell::port_of(const std::string& name) const {
    const std::string& pname = impl_->port_of.at(name);
    for (const auto& p : impl_->ports)
        if (p.first == pname) return p.second;
    throw std::out_of_range("no port for axis " + name);
}

std::map<std::string, std::shared_ptr<Port>> Cell::ports() const {
    std::map<std::string, std::shared_ptr<Port>> out;
    for (const auto& p : impl_->ports) out[p.first] = p.second;
    return out;
}

std::map<std::string, std::string> Cell::lines() const {
    std::map<std::string, std::string> out;
    for (const auto& p : impl_->ports) out[p.first] = to_string(p.second->line()->kind());
    return out;
}

std::map<std::string, std::shared_ptr<Gateway>> Cell::gateways() const {
    std::map<std::string, std::shared_ptr<Gateway>> out;
    for (const auto& p : impl_->ports)
        if (auto gw = p.second->gateway()) out[p.first] = gw;
    return out;
}

std::map<std::string, GatewayStatus> Cell::gateway_status_all() {
    std::map<std::string, GatewayStatus> out;
    for (const auto& kv : gateways()) out.emplace(kv.first, kv.second->read_gateway_status());
    return out;
}

std::string Cell::units_of(const std::string& name) { return (*this)[name].units(); }

std::map<std::string, DeviceInfo> Cell::info() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->info;
}

std::map<std::string, std::string> Cell::port_errors() const { return impl_->port_errors; }
std::map<std::string, std::exception_ptr> Cell::port_error_details() const { return impl_->port_error_details; }

std::map<std::string, std::string> Cell::faults() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->faults;
}

void Cell::clear_fault(const std::string& name) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->faults.erase(name);
}

// ============================================================================================ Cell: group commands

CellResults<DeviceInfo> Cell::identify_all(const std::vector<std::string>& axes, bool return_exceptions) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    auto out = impl_->per_port<DeviceInfo>(which, [](const std::string&, Axis& ax) { return ax.identify(); });
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        for (const auto& kv : out.results) impl_->info[kv.first] = kv.second;
    }
    return impl_->finish("identify_all", which, std::move(out), return_exceptions);
}

CellResults<Status> Cell::status_all(const std::vector<std::string>& axes, bool return_exceptions,
                                     std::optional<double> monitor_max_age_s) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    // an axis whose status monitor has a fresh sample is not read: no bus traffic for it
    std::map<std::string, Status> sampled;
    if (monitor_max_age_s) {
        for (const auto& n : which) {
            const std::shared_ptr<AxisMonitor> mon = impl_->axes.at(n)->monitor();
            if (!mon) continue;
            if (std::optional<Status> st = mon->fresh(*monitor_max_age_s)) sampled.emplace(n, std::move(*st));
        }
    }
    std::vector<std::string> to_read;
    for (const auto& n : which)
        if (!sampled.count(n)) to_read.push_back(n);
    auto out = impl_->per_port<Status>(to_read, [](const std::string&, Axis& ax) { return ax.status(); });
    for (auto& kv : sampled) out.results.emplace(kv.first, std::move(kv.second));
    return impl_->finish("status_all", which, std::move(out), return_exceptions);
}

CellResults<Json> Cell::monitor_all(const std::vector<std::string>& axes, bool return_exceptions) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    auto out = impl_->per_port<Json>(which, [](const std::string&, Axis& ax) { return monitor_reading(ax); });
    return impl_->finish("monitor_all", which, std::move(out), return_exceptions);
}

CellResults<AxisRanges> Cell::limits_all(const std::vector<std::string>& axes, bool return_exceptions) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    auto out = impl_->per_port<AxisRanges>(which, [](const std::string&, Axis& ax) { return ax.limits(); });
    return impl_->finish("limits_all", which, std::move(out), return_exceptions);
}

CellResults<Status> Cell::servo_all(bool on, const std::vector<std::string>& axes, double timeout_s,
                                    bool return_exceptions) {
    impl_->check_open();
    detail::check_timeout(timeout_s);  // nothing sent to any axis
    const auto which = impl_->select(axes);
    auto out = impl_->per_axis<Status>(
        which,
        [on, timeout_s](const std::string&, Axis& ax) {
            const std::optional<Status> st = ax.servo(on, true, timeout_s);
            return st ? *st : ax.status();
        },
        false);
    return impl_->finish("servo_all", which, std::move(out), return_exceptions);
}

CellResults<Status> Cell::home_all(const std::vector<std::string>& axes, std::optional<double> timeout_s,
                                   bool return_exceptions, std::optional<CommandMethod> method,
                                   bool stop_others_on_error) {
    impl_->check_open();
    detail::check_timeout(timeout_s);  // nothing sent to any axis (a refusal per axis would stop the others)
    const auto which = impl_->select(axes);
    impl_->forget_targets(which);
    auto out = impl_->per_axis<Status>(
        which,
        [timeout_s, method](const std::string&, Axis& ax) {
            const std::optional<Status> st = ax.home(true, timeout_s, method);
            return st ? *st : ax.status();
        },
        stop_others_on_error);
    return impl_->finish("home_all", which, std::move(out), return_exceptions);
}

std::optional<CellResults<Status>> Cell::move_all(const std::map<std::string, double>& targets,
                                                  const MoveOptions& options, bool return_exceptions) {
    std::vector<AxisMove> moves;
    for (const auto& kv : targets) {
        AxisMove m;
        m.axis = kv.first;
        m.position = kv.second;
        m.options = options;
        moves.push_back(m);
    }
    // keep the cell order (a map is name order already; names are "PORT:slave")
    return impl_->move_each("move_all", moves, options.wait, options.timeout_s, return_exceptions);
}

std::optional<CellResults<Status>> Cell::move_all(const std::map<std::string, double>& targets,
                                                  const MoveOptions& options, const PerAxisMoveValues& per_axis,
                                                  bool return_exceptions) {
    impl_->check_open();
    // every map names only axes of this cell and, when not empty, every axis of the move (the Python edition's _pick)
    const std::pair<const char*, const std::map<std::string, double>*> maps[] = {
        {"speed", &per_axis.speed}, {"accel_g", &per_axis.accel_g}, {"band", &per_axis.band},
        {"decel_g", &per_axis.decel_g}};
    for (const auto& m : maps) {
        for (const auto& kv : *m.second)
            if (!impl_->axes.count(kv.first))
                throw std::out_of_range(std::string(m.first) + ": no axis '" + kv.first + "' in this cell (axes: " +
                                        util::join(impl_->names, ", ") + "); nothing sent");
        if (m.second->empty()) continue;
        for (const auto& t : targets)
            if (!m.second->count(t.first))
                throw std::out_of_range(std::string("no ") + m.first + " for axis " + t.first + "; nothing sent");
    }
    std::vector<AxisMove> moves;
    for (const auto& kv : targets) {
        AxisMove m;
        m.axis = kv.first;
        m.position = kv.second;
        m.options = options;
        if (!per_axis.speed.empty()) m.options.speed = per_axis.speed.at(kv.first);
        if (!per_axis.accel_g.empty()) m.options.accel_g = per_axis.accel_g.at(kv.first);
        if (!per_axis.band.empty()) m.options.band = per_axis.band.at(kv.first);
        if (!per_axis.decel_g.empty()) m.options.decel_g = per_axis.decel_g.at(kv.first);
        moves.push_back(m);
    }
    return impl_->move_each("move_all", moves, options.wait, options.timeout_s, return_exceptions);
}

std::optional<CellResults<Status>> Cell::move_each(const std::vector<AxisMove>& moves, bool wait,
                                                   std::optional<double> timeout_s, bool return_exceptions,
                                                   bool stop_others_on_error) {
    return impl_->move_each("move_each", moves, wait, timeout_s, return_exceptions, stop_others_on_error);
}

CellResults<Status> Cell::wait_all(const std::vector<std::string>& axes, std::optional<double> timeout_s,
                                   bool return_exceptions, bool stop_others_on_error) {
    return impl_->wait_all(axes, timeout_s, return_exceptions, stop_others_on_error);
}

CellResults<Status> Cell::stop_all(const std::vector<std::string>& axes, double timeout_s, bool return_exceptions,
                                   bool broadcast) {
    return impl_->stop_all(axes, timeout_s, return_exceptions, broadcast);
}

CellResults<bool> Cell::pause_all(const std::vector<std::string>& axes, bool return_exceptions, bool broadcast) {
    impl_->check_open();
    std::vector<std::string> unknown;  // as stop_all: the known axes are paused, the unknown names reported afterwards
    const auto which = impl_->select_known(axes, unknown);
    if (broadcast) {  // UNVERIFIED on hardware: several units acting on one broadcast
        for (const auto& kv : impl_->broadcast_ports(which)) {
            try {
                impl_->port_named(kv.first)->broadcast_pause(true);
            } catch (const EcDriverError& e) {
                log_warning(kCellLog, kv.first + ": broadcast pause not sent: " + detail::error_type_name(e) + ": " +
                                          e.what());
            }
        }
    }
    auto out = impl_->per_port<bool>(which, [](const std::string&, Axis& ax) {
        ax.pause();
        return true;
    });
    std::vector<std::string> listed = which;
    impl_->add_unknown(out, listed, unknown);
    return impl_->finish("pause_all", listed, std::move(out), return_exceptions);
}

CellResults<bool> Cell::resume_all(const std::vector<std::string>& axes, bool return_exceptions, bool broadcast) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    auto out = impl_->per_port<bool>(which, [](const std::string&, Axis& ax) {
        ax.resume();
        return true;
    });
    if (broadcast) {  // after the per-axis releases: no unit of that line stays paused
        for (const auto& kv : impl_->broadcast_ports(which)) {
            try {
                impl_->port_named(kv.first)->broadcast_pause(false);
            } catch (const EcDriverError& e) {
                log_warning(kCellLog, kv.first + ": broadcast pause release not sent: " + detail::error_type_name(e) +
                                          ": " + e.what());
            }
        }
    }
    return impl_->finish("resume_all", which, std::move(out), return_exceptions);
}

std::map<std::string, std::shared_ptr<AxisMonitor>> Cell::start_monitors(const std::vector<std::string>& axes,
                                                                          const MonitorOptions& options) {
    impl_->check_open();
    const auto which = impl_->select(axes);
    Impl& im = *impl_;
    std::map<std::string, std::shared_ptr<AxisMonitor>> out;
    for (const auto& n : which) {
        Axis& ax = *im.axes.at(n);
        // a running monitor whose events reach the cell already is not routed twice (the monitor is asked outside the
        // cell's mutex: its callbacks take that mutex)
        std::shared_ptr<AxisMonitor> before;
        {
            std::lock_guard<std::mutex> lock(im.mutex);
            const auto it = im.routed.find(n);
            if (it != im.routed.end()) before = it->second.lock();
        }
        const std::shared_ptr<AxisMonitor> current = ax.monitor();
        const bool routed = before && current == before && current->running();
        MonitorOptions o = options;
        if (!routed) {
            const std::shared_ptr<Impl::Route> route = im.route;
            o.on_event.push_back([route, n](const LinkEvent& e) {
                std::lock_guard<std::mutex> lock(route->mutex);
                if (route->impl) route->impl->monitor_event(n, e);
            });
        }
        if (o.name.empty()) o.name = "ec-monitor-" + n;
        std::shared_ptr<AxisMonitor> mon = ax.start_monitor(std::move(o));
        {
            std::lock_guard<std::mutex> lock(im.mutex);
            im.routed[n] = mon;
        }
        out[n] = mon;
    }
    return out;
}

void Cell::stop_monitors(const std::vector<std::string>& axes) {
    const auto which = impl_->select(axes);
    for (const auto& n : which) impl_->axes.at(n)->stop_monitor();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (const auto& n : which) impl_->routed.erase(n);
}

void Cell::close() { impl_->close(); }

bool Cell::closed() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->closed;
}

// ============================================================================================ power / link loss

CellResults<std::shared_ptr<recovery::Session>> Cell::track_all(const std::vector<std::string>& axes,
                                                                const recovery::TrackOptions& options,
                                                                bool return_exceptions) {
    impl_->check_open();
    const std::vector<std::string> which = impl_->select(axes);
    Impl& im = *impl_;
    auto out = im.per_port<std::shared_ptr<recovery::Session>>(
        which,
        [&im, &options](const std::string& n, Axis& ax) {
            recovery::TrackOptions t = options;
            t.name = n;
            {
                std::lock_guard<std::mutex> lock(im.mutex);
                const auto it = im.info.find(n);
                if (it != im.info.end()) t.info = it->second;
            }
            return recovery::track(ax, t);
        },
        false);
    return im.finish("track_all", which, std::move(out), return_exceptions);
}

CellResults<recovery::LinkCheck> Cell::check_links(const std::vector<std::string>& axes) {
    impl_->check_open();
    const std::vector<std::string> which = impl_->select(axes);
    Impl& im = *impl_;
    auto out = im.per_port<recovery::LinkCheck>(
        which, [&im](const std::string& n, Axis& ax) { return im.check_link_one(n, ax); }, false);
    return im.finish("check_links", which, std::move(out), true);
}

std::shared_ptr<recovery::LinkMonitor> Cell::monitor_links(double interval_s, const std::vector<std::string>& axes,
                                                           recovery::LinkMonitor::ChangeFn on_change) {
    impl_->check_open();
    const std::vector<std::string> which = impl_->select(axes);
    Impl* im = impl_.get();
    std::vector<std::pair<std::string, Axis*>> selected;
    for (const auto& n : which) selected.emplace_back(n, im->axes.at(n).get());
    recovery::LinkMonitor::Options o;
    o.interval_s = interval_s;
    o.on_change = std::move(on_change);
    o.check = [im](const std::string& n, Axis& ax) { return im->check_link_one(n, ax); };
    auto monitor = std::make_shared<recovery::LinkMonitor>(std::move(selected), std::move(o));
    {
        std::lock_guard<std::mutex> lock(im->mutex);
        if (im->closed) throw EcNotReadyError("cell is closed");
        im->link_monitors.push_back(monitor);
    }
    monitor->start();
    return monitor;
}

CellResults<recovery::RecoveryReport> Cell::recover_all(const std::vector<std::string>& axes, double timeout_s,
                                                        const recovery::RecoveryPolicy& policy, bool return_exceptions) {
    impl_->check_open();
    const std::vector<std::string> which = impl_->select(axes);
    Impl& im = *impl_;
    im.join_fault_threads(10.0);  // a stop of the other axes started by a monitor's fault event finishes first
    auto out = im.per_port<recovery::RecoveryReport>(
        which,
        [&im, timeout_s, &policy](const std::string& n, Axis& ax) {
            std::shared_ptr<recovery::Session> sess = recovery::session_of(ax);
            if (!sess) {
                std::optional<DeviceInfo> known;
                {
                    std::lock_guard<std::mutex> lock(im.mutex);
                    const auto it = im.info.find(n);
                    if (it != im.info.end()) known = it->second;
                }
                if (!known)
                    throw recovery::RecoveryError(n + ": never tracked and not identified: the identity to compare "
                                                  "with is unknown (track_all() while the links are up)");
                sess = recovery::session_from_info(ax, *known, n);
            }
            recovery::RecoverOptions o;
            o.timeout_s = timeout_s;
            o.policy = policy;
            o.session = sess;
            o.on_event = [&im](const recovery::RecoveryEvent& e) { im.emit(e); };
            o.name = n;
            recovery::RecoveryReport report = recovery::recover_axis(ax, o);  // holds the axis's status monitor
            std::lock_guard<std::mutex> lock(im.mutex);
            im.faults.erase(n);
            im.targets.erase(n);
            return report;
        },
        false);
    {
        std::lock_guard<std::mutex> lock(im.mutex);
        for (const auto& kv : out.errors) {
            try {
                std::rethrow_exception(kv.second);
            } catch (const recovery::UnitReplacedError&) {
                im.faults[kv.first] = recovery::kUnitReplaced;
            } catch (...) {
            }
        }
    }
    return im.finish("recover_all", which, std::move(out), return_exceptions);
}

std::vector<recovery::RecoveryEvent> Cell::events() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return std::vector<recovery::RecoveryEvent>(impl_->events.begin(), impl_->events.end());
}

}  // namespace iai_actuator_driver
