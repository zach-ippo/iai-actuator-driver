// C ABI 0.2 additions over the C++ core (include/iai_actuator_driver.h, "library: log sink, option defaults" to "cell:
// pause, cancel, monitors, recovery, lines, backup"; the model is in docs/NATIVE_ABI.md): push, command-profile moves
// and acceleration patterns (through iad_move_options), change_accel, move_sequence, move handles, the status monitor
// with the per-handle event queue, fast mode, power / link-loss recovery, unit info / backup / backup_axes / the
// self-test, the lean status and the boot baud rate, the line and gateway (with the RCON all-axes monitor), the command
// strategy, the action mode, the safety speed, the cancel entry and the Cell extras (per-axis move values in
// iad_cell_move_all, pause / resume with broadcast, monitors, links, backup). Each function is a thin call of the C++
// function its header comment names, wrapped in capi::guarded like the phase-1 functions in c_api.cpp: every exception
// is caught and turned into an iad_result and the calling thread's last error. The event queue and the 0.2 conversions
// are in src/c_api_ext_support.cpp.
//
// Evidence: UNVERIFIED on hardware (tested on the simulator: tests/test_c_api_ext.cpp, tests/c_smoke.c).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/timing.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

static_assert(static_cast<int>(LogLevel::Debug) == IAD_LOG_DEBUG && static_cast<int>(LogLevel::Info) == IAD_LOG_INFO &&
                  static_cast<int>(LogLevel::Warning) == IAD_LOG_WARNING &&
                  static_cast<int>(LogLevel::Error) == IAD_LOG_ERROR,
              "IAD_LOG_* must follow LogLevel");
static_assert(static_cast<int>(FrameDirection::Tx) == IAD_FRAME_TX && static_cast<int>(FrameDirection::Rx) == IAD_FRAME_RX &&
                  static_cast<int>(FrameDirection::RxPartial) == IAD_FRAME_RX_PARTIAL,
              "IAD_FRAME_* must follow FrameDirection");
static_assert(IAD_FAST_BAUD_RATE == kFastBaudRate, "IAD_FAST_BAUD_RATE must follow kFastBaudRate");

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

iad_bool flag(bool value) { return value ? IAD_TRUE : IAD_FALSE; }

/// Keep a JSON text on the handle (iad_axis_last_result_json / iad_cell_last_result_json read it without I/O).
void keep(AxisEntry& e, std::string text) {
    keep_thread_result(text);  // (0.7, py-fixes) iad_last_result_json: this call's own, whatever another thread keeps
    std::lock_guard<std::mutex> lock(e.mutex);
    e.last_result = std::move(text);
}
void keep(CellEntry& e, std::string text) {
    keep_thread_result(text);
    std::lock_guard<std::mutex> lock(e.mutex);
    e.last_result = std::move(text);
}

void write_json(const Json& json, char* buffer, std::size_t buffer_size, std::size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

/// "COM3:1": the axis's port and slave (the name its events carry).
std::string label(const Axis& axis) { return axis_name(axis.bus()->name(), axis.slave()); }

/// A timeout input: NaN -> `fallback`, otherwise >= 0 and finite.
double timeout_input(double value, double fallback, const char* name) {
    if (std::isnan(value)) return fallback;
    if (!(value >= 0.0) || !std::isfinite(value)) throw std::invalid_argument(std::string(name) + " must be >= 0");
    return value;
}

/// An iad_tribool input (0.8): IAD_UNKNOWN (2) -> nullopt, 0 -> false, any other value -> true (a VB True is -1).
std::optional<bool> tribool_input(iad_tribool value, const char* /*name*/) {
    if (value == IAD_UNKNOWN) return std::nullopt;
    return value != IAD_FALSE;
}

/// The queue settings of a monitor options struct (defaults without one).
void configure_queue(EventQueue& queue, const iad_monitor_options* options) {
    if (options)
        queue.configure(options->events, options->queue_capacity);
    else
        queue.configure(IAD_EVENTS_ALL, EventQueue::kDefaultCapacity);
}

/// Add the queueing callbacks to a running monitor that does not feed `queue` yet.
void add_callbacks(AxisMonitor& monitor, const std::shared_ptr<EventQueue>& queue, const std::string& name) {
    MonitorOptions hooks = queueing_callbacks(queue, name);
    for (auto& f : hooks.on_change) monitor.add_on_change(std::move(f));
    for (auto& f : hooks.on_done) monitor.add_on_done(std::move(f));
    for (auto& f : hooks.on_event) monitor.add_on_event(std::move(f));
}

/// Start (or join) the status monitor of `axis` so that it feeds `queue` (`hooked` remembers the monitor fed).
void hook_monitor(Axis& axis, const std::shared_ptr<EventQueue>& queue, const std::string& name,
                  std::weak_ptr<AxisMonitor>& hooked, const MonitorOptions& base) {
    const std::shared_ptr<AxisMonitor> current = axis.monitor();
    if (current && current->running()) {  // running already: its own options stay, the callbacks are added once
        if (hooked.lock() != current) add_callbacks(*current, queue, name);
        hooked = current;
        return;
    }
    hooked = axis.start_monitor(queueing_callbacks(queue, name, base));
}

/// Feed `queue` from the monitor running on `axis`, if any (enable_fast starts one without the ABI's callbacks).
void hook_running(AxisEntry& e) {
    std::lock_guard<std::mutex> lock(e.monitor_mutex);
    const std::shared_ptr<AxisMonitor> current = e.axis->monitor();
    if (current && current->running() && e.hooked.lock() != current) {
        add_callbacks(*current, e.events, label(*e.axis));
        e.hooked = current;
    }
}

/// The monitor of the axis; EcNotReadyError without one.
std::shared_ptr<AxisMonitor> need_monitor(Axis& axis) {
    std::shared_ptr<AxisMonitor> mon = axis.monitor();
    if (!mon) throw EcNotReadyError("no status monitor: call iad_axis_start_monitor first");
    return mon;
}

/// A sample (or none yet) and the monitor's counters.
void fill_sample(const AxisMonitor& mon, const std::optional<MonitorSample>& sample, iad_monitor_sample* out) {
    if (sample) {
        to_c(*sample, out);
    } else {
        std::memset(out, 0, sizeof *out);
        unknown_status(&out->status);
        out->t_start_s = kNaN;
        out->t_s = kNaN;
        out->age_s = kNaN;
    }
    const std::optional<double> rate = mon.poll_rate_hz();
    out->poll_rate_hz = rate ? *rate : kNaN;
    out->polls = static_cast<std::int64_t>(mon.polls());
    out->error_count = static_cast<std::int64_t>(mon.error_count());
    out->running = flag(mon.running());
}

/// "<kind>: <message>" of an exception, null without one.
Json error_text(const std::exception_ptr& error) {
    if (!error) return Json();
    const ErrorDescription d = describe_error(error);
    return Json(d.kind + ": " + d.message);
}

/// The next event of a queue into `out` (IAD_EVENT_NONE when none came within the timeout). `cell_names` (a cell's
/// queue): (0.8) iad_event.index = the event's axis's position in them (IAD_INT_UNKNOWN for none of them).
void next_event(EventQueue& queue, double timeout_s, iad_event* out,
                const std::vector<std::string>* cell_names = nullptr) {
    std::int64_t sequence = 0;
    std::int32_t dropped = 0;
    const std::optional<Event> event = queue.pop(timeout_s, &sequence, &dropped);
    if (!event) {
        no_event(out);
        return;
    }
    to_c(*event, sequence, dropped, out);
    if (cell_names) {
        const auto it = std::find(cell_names->begin(), cell_names->end(), event->axis);
        if (it != cell_names->end()) out->index = static_cast<std::int32_t>(it - cell_names->begin());
    }
}

/// The listed axes (every axis of the cell for NULL / ""), each checked to exist before anything is done.
std::vector<std::string> cell_axes(Cell& cell, const char* axes) {
    std::vector<std::string> list = split_list(axes);
    if (list.empty()) return cell.names();
    for (const auto& n : list)
        if (!cell.contains(n)) throw std::out_of_range("no axis '" + n + "' in this cell");
    return list;
}

/// pause_all / resume_all: per-axis codes (no status read); CellError when one or more axes failed.
void bool_results(const CellResults<bool>& r, iad_cell_result* results, std::size_t capacity, std::size_t* count) {
    if (fill_cell_results(r.names, {}, r.errors, results, capacity, count))
        throw CellError(r.operation, r.messages, r.errors);
}

/// The RCON all-axes monitor of `line` (Port::gateway_monitor's rules: `slaves` NULL / "" = the slaves the gateway
/// reports linked) as {"<slave>": record}; EcNotReadyError without an RCON gateway. `where` names the port.
Json gateway_monitor_json(const std::shared_ptr<Line>& line, const char* slaves, const std::string& where) {
    const auto gw = line ? std::dynamic_pointer_cast<RconGateway>(line->gateway()) : nullptr;
    if (!gw)
        throw EcNotReadyError(where + ": no RCON gateway on this line (line '" +
                              (line ? iai_actuator_driver::to_string(line->kind()) : std::string("unknown")) + "')");
    std::vector<int> wanted = slaves && *slaves ? to_int_list(slaves, "slaves") : gw->status().linked_slaves();
    std::sort(wanted.begin(), wanted.end());
    wanted.erase(std::unique(wanted.begin(), wanted.end()), wanted.end());
    std::vector<int> axes;
    for (int s : wanted) {
        if (s < 1 || s > kRconMaxAxes)
            throw std::invalid_argument("an axis behind an RCON gateway is slave 1..16, got " + std::to_string(s));
        axes.push_back(s - 1);
    }
    Json j = Json::object();
    if (axes.empty()) return j;
    for (const auto& kv : gw->read_monitor(axes)) j.set(std::to_string(kv.second.slave()), to_json(kv.second));
    return j;
}

/// A move handle's JSON without blocking: MoveHandle::to_json when the handle is free, otherwise (a wait runs on
/// another thread) the fields that never change after the start; the ABI's own cancel record merged in.
Json move_json(MoveEntry& m) {
    Json j;
    std::unique_lock<std::mutex> lock(m.call, std::try_to_lock);
    if (lock.owns_lock()) {
        j = m.handle.to_json();
    } else {
        j = Json::object();
        j.set("target_mm", m.handle.target());
        j.set("band_mm", m.handle.band());
        j.set("completion", iai_actuator_driver::to_string(m.handle.completion()));
        j.set("timeout_s", std::round(m.handle.timeout_s() * 1000.0) / 1000.0);
        j.set("done", false);
        j.set("cancelled", false);
        j.set("position_mm", Json());
    }
    std::lock_guard<std::mutex> state(m.state);
    if (m.cancelled) j.set("cancelled", true);
    if (m.cancel_status && !j.find("done")->as_bool()) {
        j.set("done", true);
        j.set("position_mm", m.cancel_status->position_mm);
    }
    return j;
}

}  // namespace

extern "C" {

// ============================================================================================ library

iad_result IAD_CALL iad_set_log_sink(iad_log_sink sink, void* user) {
    return guarded([&] {
        if (!sink) {
            set_log_handler(LogHandler());
            return;
        }
        set_log_handler([sink, user](LogLevel level, const std::string& source, const std::string& message) {
            if (static_cast<int32_t>(level) < log_level().load()) return;  // 0.3: iad_set_log_level
            sink(static_cast<int32_t>(level), source.c_str(), message.c_str(), user);
        });
    });
}

iad_result IAD_CALL iad_push_options_init(iad_push_options* options) {
    return guarded([&] {
        need(options, "options");
        const PushOptions d;
        iad_push_options c = zeroed<iad_push_options>();
        c.speed = d.speed;
        c.accel_g = d.accel_g;
        c.timeout_s = d.timeout_s ? *d.timeout_s : kNaN;
        c.direction = d.direction;
        c.wait = flag(d.wait);
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_sequence_options_init(iad_sequence_options* options) {
    return guarded([&] {
        need(options, "options");
        const SequenceOptions d;
        iad_sequence_options c = zeroed<iad_sequence_options>();
        c.speed = d.speed;
        c.accel_g = d.accel_g;
        c.band = d.band;
        c.blend_band = d.blend_band ? *d.blend_band : kNaN;
        c.timeout_s = d.timeout_s ? *d.timeout_s : kNaN;
        c.completion = IAD_COMPLETION_DEFAULT;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_monitor_options_init(iad_monitor_options* options) {
    return guarded([&] {
        need(options, "options");
        const MonitorOptions d;
        iad_monitor_options c = zeroed<iad_monitor_options>();
        c.gap_s = d.gap_s;
        c.link_check_s = d.link_check_s ? *d.link_check_s : 0.0;
        c.retries = d.retries;
        c.events = IAD_EVENTS_ALL;
        c.queue_capacity = EventQueue::kDefaultCapacity;
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_track_options_init(iad_track_options* options) {
    return guarded([&] {
        need(options, "options");
        const recovery::TrackOptions d;
        iad_track_options c = zeroed<iad_track_options>();
        c.alarm_history = flag(d.alarm_history);
        c.parameters = flag(d.parameters);
        write_sized(options, c);
    });
}

iad_result IAD_CALL iad_recovery_policy_init(iad_recovery_policy* policy) {
    return guarded([&] {
        need(policy, "policy");
        const recovery::RecoveryPolicy d;
        iad_recovery_policy c = zeroed<iad_recovery_policy>();
        c.position_tolerance_mm = d.position_tolerance_mm;
        c.poll_s = d.poll_s;
        c.boot_baud_rate = d.boot_baud_rate;
        c.restore_baud_rate = flag(d.restore_baud_rate);
        c.restore_control = flag(d.restore_control);
        c.restore_safety_speed = flag(d.restore_safety_speed);
        c.clear_message_alarms = flag(d.clear_message_alarms);
        c.check_parameters = flag(d.check_parameters);
        write_sized(policy, c);
    });
}

iad_result IAD_CALL iad_check_options_init(iad_check_options* options) {
    return guarded([&] {
        need(options, "options");
        const CheckOptions d;
        iad_check_options c = zeroed<iad_check_options>();
        c.step_mm = d.step_mm;
        c.speed_mm_s = d.speed_mm_s;
        c.accel_g = d.accel_g;
        c.motion = flag(d.motion);
        write_sized(options, c);
    });
}

// ============================================================================================ axis motion

iad_result IAD_CALL iad_axis_push(iad_axis* axis, double target, double band, double force_pct,
                                  const iad_push_options* options, iad_push_result* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const PushOptions o = to_push_options(options);
        const std::optional<PushResult> r =
            e->axis->push(finite(target, "target"), finite(band, "band"), finite(force_pct, "force_pct"), o);
        if (!r) return;  // started without waiting
        keep(*e, to_json(*r).dump());
        if (out) to_c(*r, out);
    });
}

iad_result IAD_CALL iad_axis_change_accel(iad_axis* axis, double accel_g, const iad_change_options* options,
                                          iad_status* out) {
    return guarded([&] {
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const ChangeArgs a = to_change_args(options);
        const std::optional<Status> st = e->axis->change_accel(finite(accel_g, "accel_g"), a.wait, a.timeout_s);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_move_sequence(iad_axis* axis, const iad_sequence_point* points, size_t point_count,
                                           const iad_sequence_options* options, iad_status* statuses, size_t capacity,
                                           size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        if (point_count > 0) need(points, "points");
        std::vector<SequencePoint> list;
        list.reserve(point_count);
        for (const iad_sequence_point& p : read_input_array(points, point_count, "points")) list.push_back(from_c(p));
        const SequenceOptions o = to_sequence_options(options);
        const std::vector<Status> done = e->axis->move_sequence(list, o);
        if (statuses)
            for (std::size_t i = 0; i < done.size() && i < capacity; ++i) to_c(done[i], &statuses[i]);
        if (count) *count = done.size();
    });
}

// ============================================================================================ move handles

iad_result IAD_CALL iad_axis_start_move_to(iad_axis* axis, double position, const iad_move_options* options,
                                           iad_move_handle** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions{});
        MoveHandle h = e->axis->start_move_to(finite(position, "position"), o);
        *out = register_move(std::make_shared<MoveEntry>(e->axis, std::move(h)));
    });
}

iad_result IAD_CALL iad_axis_start_move_by(iad_axis* axis, double delta, const iad_move_options* options,
                                           iad_move_handle** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions{});
        MoveHandle h = e->axis->start_move_by(finite(delta, "delta"), o);
        *out = register_move(std::make_shared<MoveEntry>(e->axis, std::move(h)));
    });
}

iad_result IAD_CALL iad_axis_start_jog(iad_axis* axis, double delta, const iad_move_options* options,
                                       iad_move_handle** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto e = find_axis_to_wait(axis);  // 0.3: refused while latched
        const MoveOptions o = to_move_options(options, MoveOptions::jog());
        MoveHandle h = e->axis->start_jog(finite(delta, "delta"), o);
        *out = register_move(std::make_shared<MoveEntry>(e->axis, std::move(h)));
    });
}

iad_result IAD_CALL iad_move_handle_done(iad_move_handle* move, iad_bool* done, iad_status* out) {
    return guarded([&] {
        const auto m = find_move(move);
        need(done, "done");
        *done = IAD_FALSE;
        std::lock_guard<std::mutex> lock(m->call);
        if (!m->handle.done()) return;
        *done = IAD_TRUE;
        const std::optional<Status> r = m->handle.result();
        if (r && out) to_c(*r, out);
    });
}

iad_result IAD_CALL iad_move_handle_wait(iad_move_handle* move, double timeout_s, iad_status* out) {
    return guarded([&] {
        const auto m = find_move(move);
        check_cancel_latch(*m->axis);  // 0.3
        std::optional<double> t;
        if (!std::isnan(timeout_s)) t = timeout_input(timeout_s, 0.0, "timeout_s");
        std::lock_guard<std::mutex> lock(m->call);
        const Status st = m->handle.wait(t);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_move_handle_poll(iad_move_handle* move, double max_wait_s, iad_bool* done, iad_status* out) {
    return guarded([&] {
        const auto m = find_move(move);
        check_cancel_latch(*m->axis);  // 0.3
        need(done, "done");
        *done = IAD_FALSE;
        std::lock_guard<std::mutex> lock(m->call);
        // MoveHandle::poll: done() every poll interval for the slice, the wait past the deadline; iad_axis_cancel ends
        // the slice at once (IAD_ERR_CANCELLED after a decel stop)
        if (!m->handle.poll(max_wait_s)) return;  // the slice ended: the move runs on
        *done = IAD_TRUE;
        const std::optional<Status> r = m->handle.result();
        if (r && out) to_c(*r, out);
    });
}

iad_result IAD_CALL iad_move_handle_cancel(iad_move_handle* move, iad_bool wait, double timeout_s, iad_status* out) {
    return guarded([&] {
        const auto m = find_move(move);
        const double t = timeout_input(timeout_s, 5.0, "timeout_s");
        std::unique_lock<std::mutex> lock(m->call, std::try_to_lock);
        if (lock.owns_lock()) {
            const Status st = m->handle.cancel(wait != 0, t);
            if (out) to_c(st, out);
            return;
        }
        // a wait runs on this handle in another thread: stop the axis directly (Axis::stop is the call that may come
        // from another thread) and record the cancel here; that wait ends as a move does when it is stopped
        const Status st = m->axis->stop(wait != 0, t);
        {
            std::lock_guard<std::mutex> state(m->state);
            m->cancelled = true;
            if (!m->cancel_status) m->cancel_status = st;
        }
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_move_handle_result(iad_move_handle* move, iad_move_info* out) {
    return guarded([&] {
        const auto m = find_move(move);
        need(out, "out");
        {
            std::unique_lock<std::mutex> lock(m->call, std::try_to_lock);
            if (lock.owns_lock()) {
                to_c(m->handle, out);
            } else {  // a wait runs on another thread: the fields that never change after the start
                std::memset(out, 0, sizeof *out);
                unknown_status(&out->result);
                out->target = m->handle.target();
                out->band = m->handle.band();
                out->timeout_s = m->handle.timeout_s();
                out->sent_at_s = m->handle.sent_at();
                out->deadline_s = m->handle.deadline();
                out->completion = from_completion(m->handle.completion());
            }
        }
        std::lock_guard<std::mutex> state(m->state);
        if (m->cancelled) out->cancelled = IAD_TRUE;
        if (!out->has_result && m->cancel_status) {
            to_c(*m->cancel_status, &out->result);
            out->has_result = IAD_TRUE;
        }
    });
}

iad_result IAD_CALL iad_move_handle_json(iad_move_handle* move, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto m = find_move(move);
        check_text_output(buffer, buffer_size, required);
        write_json(move_json(*m), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_move_handle_close(iad_move_handle* move) {
    return guarded([&] {
        if (!move) return;
        bool other_kind = false;
        release_move(move, &other_kind);
        if (other_kind)
            throw std::invalid_argument("another kind of handle was passed to iad_move_handle_close (use "
                                        "iad_axis_close / iad_cell_close)");
    });
}

// ============================================================================================ status monitor

iad_result IAD_CALL iad_axis_start_monitor(iad_axis* axis, const iad_monitor_options* options) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const MonitorOptions base = to_monitor_options(options);
        configure_queue(*e->events, options);  // checks the queue settings before anything starts
        std::lock_guard<std::mutex> lock(e->monitor_mutex);
        hook_monitor(*e->axis, e->events, label(*e->axis), e->hooked, base);
    });
}

iad_result IAD_CALL iad_axis_stop_monitor(iad_axis* axis, double timeout_s) {
    return guarded([&] {
        const auto e = find_axis(axis);
        e->axis->stop_monitor(timeout_input(timeout_s, 1.0, "timeout_s"));
    });
}

iad_result IAD_CALL iad_axis_monitor_latest(iad_axis* axis, iad_monitor_sample* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        const auto mon = need_monitor(*e->axis);
        fill_sample(*mon, mon->latest(), out);
    });
}

iad_result IAD_CALL iad_axis_monitor_wait_update(iad_axis* axis, int64_t after_seq, double timeout_s, iad_bool* got,
                                                 iad_monitor_sample* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(got, "got");
        *got = IAD_FALSE;
        const double t = timeout_input(timeout_s, 1.0, "timeout_s");
        const auto mon = need_monitor(*e->axis);
        const std::optional<long> after = after_seq > 0 ? std::optional<long>(static_cast<long>(after_seq)) : std::nullopt;
        const std::optional<MonitorSample> s = mon->wait_update(after, std::nullopt, t);
        if (!s) return;
        *got = IAD_TRUE;
        if (out) fill_sample(*mon, s, out);
    });
}

iad_result IAD_CALL iad_axis_monitor_wait_done(iad_axis* axis, double timeout_s, iad_bool* done, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(done, "done");
        *done = IAD_FALSE;
        const double t = timeout_input(timeout_s, 10.0, "timeout_s");
        const auto mon = need_monitor(*e->axis);
        const std::optional<Status> st = mon->wait_done(t);
        if (!st) return;
        *done = IAD_TRUE;
        if (out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_monitor_info_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const auto mon = e->axis->monitor();
        Json j;
        if (mon) {
            j = Json::object();
            j.set("running", mon->running());
            j.set("name", mon->name());
            j.set("polls", mon->polls());
            j.set("error_count", mon->error_count());
            j.set("link_checks", mon->link_checks());
            j.set("poll_rate_hz", mon->poll_rate_hz());
            j.set("age_s", mon->age_s());
            j.set("link_state", mon->link_state());
            j.set("gap_s", mon->gap_s());
            j.set("retries", mon->retries());
            j.set("link_check_s", mon->link_check_s());
            j.set("error", error_text(mon->error()));
            j.set("last_error", error_text(mon->last_error()));
            // (0.5, py-monitor) the holds, the start time and the errors as iad_last_error_json describes them
            j.set("suspended", mon->suspended());
            j.set("started_at_s", mon->started_at());
            j.set("error_detail", mon->error() ? describe_error(mon->error()).json : Json());
            j.set("last_error_detail", mon->last_error() ? describe_error(mon->last_error()).json : Json());
        }
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_next_event(iad_axis* axis, double timeout_s, iad_event* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        next_event(*e->events, timeout_s, out);
    });
}

iad_result IAD_CALL iad_axis_set_frame_sink(iad_axis* axis, iad_frame_sink sink, void* user) {
    return guarded([&] {
        const auto e = find_axis(axis);
        if (!sink) {
            e->axis->bus()->set_frame_logger(FrameLogger());
            return;
        }
        e->axis->bus()->set_frame_logger([sink, user](FrameDirection direction, const std::vector<std::uint8_t>& bytes) {
            sink(static_cast<int32_t>(direction), bytes.data(), bytes.size(), user);
        });
    });
}

// ============================================================================================ fast mode

iad_result IAD_CALL iad_axis_enable_fast(iad_axis* axis, int32_t baud_rate, iad_bool monitor, iad_fast_summary* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        if (baud_rate < 0) throw std::invalid_argument("baud_rate must be a rate, or 0 to keep the present one");
        const std::optional<int> rate = baud_rate > 0 ? std::optional<int>(baud_rate) : std::nullopt;
        const FastModeSummary s = e->axis->enable_fast(rate, monitor != 0);
        if (monitor) hook_running(*e);  // its events go to this handle's queue
        keep(*e, s.to_json().dump());
        if (out) to_c(s, out);
    });
}

iad_result IAD_CALL iad_axis_disable_fast(iad_axis* axis, iad_fast_summary* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const FastModeSummary s = e->axis->disable_fast();
        keep(*e, s.to_json().dump());
        if (out) to_c(s, out);
    });
}

iad_result IAD_CALL iad_axis_fast_mode(iad_axis* axis, iad_bool* on) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(on, "on");
        *on = flag(e->axis->fast_mode());
    });
}

// ============================================================================================ recovery

iad_result IAD_CALL iad_axis_track(iad_axis* axis, const iad_track_options* options, const char* name,
                                   const char* rows, const char* backup_json) {
    return guarded([&] {
        const auto e = find_axis(axis);
        recovery::TrackOptions o = to_track_options(options);
        if (name) o.name = name;
        o.rows = to_row_selection(rows);
        if (backup_json) o.backup = parse_backup(backup_json);
        recovery::track(*e->axis, o);
    });
}

iad_result IAD_CALL iad_axis_session_from_backup(iad_axis* axis, const char* backup_json, const char* name) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const Json backup = parse_backup(need(backup_json, "backup_json"));
        recovery::session_from_backup(*e->axis, backup, name ? name : "");
    });
}

iad_result IAD_CALL iad_axis_session_from_backup_file(iad_axis* axis, const char* path, const char* name) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(path, "path");
        const Json backup = info::load_backup(path);  // std::ios_base::failure (IAD_ERR_FILE) when it cannot be read
        if (backup.type() != Json::Type::Object) throw std::invalid_argument("the backup file holds no JSON object");
        recovery::session_from_backup(*e->axis, backup, name ? name : "");
    });
}

iad_result IAD_CALL iad_axis_session_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<recovery::Session> s = recovery::session_of(*e->axis);
        if (!s) throw EcNotReadyError("no recovery session: call iad_axis_track or iad_axis_session_from_backup first");
        Json j;
        {
            std::lock_guard<std::recursive_mutex> lock(s->mutex);
            j = s->to_json();
        }
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_forget(iad_axis* axis) {
    return guarded([&] { recovery::forget(*find_axis(axis)->axis); });
}

iad_result IAD_CALL iad_axis_note_target(iad_axis* axis, double target_mm) {
    return guarded([&] {
        const auto e = find_axis(axis);
        std::optional<double> t = opt(target_mm);
        if (t) finite(*t, "target_mm");
        recovery::note_target(*e->axis, t);
    });
}

iad_result IAD_CALL iad_axis_note_control(iad_axis* axis, iad_tribool modbus_control, iad_tribool safety_speed,
                                          int32_t baud_rate, iad_tribool homed) {
    return guarded([&] {
        const auto e = find_axis(axis);
        recovery::ControlChange c;
        c.modbus_control = tribool_input(modbus_control, "modbus_control");
        c.safety_speed = tribool_input(safety_speed, "safety_speed");
        c.homed = tribool_input(homed, "homed");
        if (baud_rate < 0 && baud_rate != IAD_INT_UNKNOWN)
            throw std::invalid_argument("baud_rate must be a rate, or 0 for unchanged");
        if (baud_rate > 0) c.baud_rate = baud_rate;
        recovery::note_control(*e->axis, c);
    });
}

iad_result IAD_CALL iad_axis_check_link(iad_axis* axis, iad_link_check* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        recovery::CheckLinkOptions o;
        o.on_event = queueing_recovery_events(e->events, IAD_EVENT_SOURCE_CHECK_LINK);
        const recovery::LinkCheck c = recovery::check_link(*e->axis, o);
        keep(*e, c.to_json().dump());
        if (out) to_c(c, out);
    });
}

iad_result IAD_CALL iad_axis_link_state(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_text_output(recovery::link_state(*e->axis), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_recover(iad_axis* axis, double timeout_s, const iad_recovery_policy* policy,
                                     const char* check_rows, const char* backup_json, const char* name,
                                     iad_recovery_summary* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        recovery::RecoverOptions o;
        o.timeout_s = timeout_input(timeout_s, recovery::kRecoverTimeoutS, "timeout_s");
        o.policy = to_recovery_policy(policy);
        o.policy.check_rows = to_row_selection(check_rows);
        if (backup_json) o.policy.backup = parse_backup(backup_json);
        if (name) o.name = name;
        o.on_event = queueing_recovery_events(e->events, IAD_EVENT_SOURCE_RECOVER);
        o.require_session = policy && policy->require_session != 0;  // (0.6, py-recovery)
        const recovery::RecoveryReport r = recovery::recover_axis(*e->axis, o);
        keep(*e, r.to_json().dump());
        if (out) to_c(r, out);
    });
}

// ============================================================================================ information

iad_result IAD_CALL iad_axis_read_unit_info_json(iad_axis* axis, const char* sections, char* buffer,
                                                 size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        std::optional<std::vector<std::string>> wanted;
        if (sections && *sections) wanted = split_list(sections);
        std::string text = info::unit_info(*e->axis, wanted).dump();
        keep(*e, text);  // kept even when the buffer is too small (no second read)
        write_text_output(text, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_backup_json(iad_axis* axis, const char* rows, const char* path, char* buffer,
                                         size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const info::BackupRows which = rows && *rows ? info::BackupRows::parse(rows) : info::BackupRows::used();
        std::string text = info::backup(*e->axis, which, path ? std::string(path) : std::string()).dump();
        keep(*e, text);
        write_text_output(text, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_backup_axes_json(iad_axis* const* axes, size_t axis_count, const char* names,
                                         const char* directory, const char* rows, char* buffer, size_t buffer_size,
                                         size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        if (axis_count == 0) throw std::invalid_argument("axes: iad_backup_axes_json needs at least one axis");
        need(axes, "axes");
        need(directory, "directory");
        if (!*directory) throw std::invalid_argument("directory is empty");
        const std::vector<std::string> given = split_list(names, false);
        if (!given.empty() && given.size() != axis_count)
            throw std::invalid_argument(std::to_string(axis_count) + " axes given but " + std::to_string(given.size()) +
                                        " names");
        const info::BackupRows which = rows && *rows ? info::BackupRows::parse(rows) : info::BackupRows::used();
        // every handle and name checked before anything is read
        std::vector<std::shared_ptr<AxisEntry>> entries;
        std::vector<std::pair<std::string, Axis*>> selected;
        for (std::size_t i = 0; i < axis_count; ++i) {
            entries.push_back(find_axis(axes[i]));
            const std::string name = given.empty() ? label(*entries.back()->axis) : given[i];
            for (const auto& s : selected)
                if (s.first == name) throw std::invalid_argument("two axes named '" + name + "' (one file each)");
            selected.emplace_back(name, entries.back()->axis.get());
        }
        Json j = Json::object();
        for (const auto& kv : info::backup_axes(selected, directory, which)) j.set(kv.first, kv.second);
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_self_test(iad_axis* axis, const iad_check_options* options, const char* port_label,
                                       iad_check_summary* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        CheckOptions o = to_check_options(options);
        if (port_label) o.port_label = port_label;
        const std::weak_ptr<EventQueue> weak = e->events;
        const std::string name = label(*e->axis);
        // (0.6, py-info-cli) the lines of the test while it runs, for iad_axis_self_test_items_json (e outlives the call)
        AxisEntry* entry = e.get();
        const auto clear_lines = [entry] {
            std::lock_guard<std::mutex> lock(entry->mutex);
            entry->check_items.clear();
        };
        clear_lines();
        o.on_item = [weak, name, entry](const CheckItem& item) {
            {
                std::lock_guard<std::mutex> lock(entry->mutex);
                entry->check_items.push_back(item);
            }
            if (const auto q = weak.lock()) {
                Event ev;
                ev.kind = IAD_EVENT_CHECK_ITEM;
                ev.axis = name;
                ev.name = item.name;
                ev.cause = item.result;
                ev.message = item.detail;
                q->push(std::move(ev));
            }
        };
        CheckReport r;
        try {
            r = run_check(*e->axis, o);
        } catch (...) {
            clear_lines();
            throw;
        }
        clear_lines();
        keep(*e, to_json(r).dump());
        if (out) to_c(r, out);
    });
}

// ============================================================================================ line, unit modes

iad_result IAD_CALL iad_axis_line_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Line> line = e->axis->bus()->line();
        write_json(line ? to_json(line->snapshot()) : Json(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_gateway_status_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Line> line = e->axis->bus()->line();
        const std::shared_ptr<Gateway> gateway = line ? line->gateway() : nullptr;
        write_json(gateway ? to_json(gateway->read_gateway_status()) : Json(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_set_position_unit(iad_axis* axis, double unit_mm) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const std::optional<double> unit = to_position_unit(unit_mm);
        if (!unit) throw std::invalid_argument("unit_mm must be 0.01 or 0.001");
        const std::shared_ptr<Line> line = e->axis->bus()->line();
        if (!line) throw EcNotReadyError("the axis has no line yet");
        line->set_position_unit(*unit, e->axis->slave());
    });
}

iad_result IAD_CALL iad_axis_set_action_mode(iad_axis* axis, iad_bool pc, iad_tribool safety, iad_bool verify,
                                             iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const std::optional<bool> sfty = tribool_input(safety, "safety");
        const Status st = e->axis->set_action_mode(pc != 0, sfty, verify != 0);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_axis_set_safety_speed(iad_axis* axis, iad_bool on, iad_bool verify, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const Status st = e->axis->set_safety_speed(on != 0, verify != 0);
        if (out) to_c(st, out);
    });
}

iad_result IAD_CALL iad_axis_cancel(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->cancel_waits(); });
}

iad_result IAD_CALL iad_axis_set_strategy(iad_axis* axis, int32_t method) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const std::optional<CommandMethod> m = to_method(method);
        if (!m) throw std::invalid_argument("method must be IAD_METHOD_COIL, _IAI, _IAI2 or _REGISTER");
        e->axis->set_strategy(*m);
    });
}

iad_result IAD_CALL iad_axis_strategy(iad_axis* axis, int32_t* method) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(method, "method");
        *method = from_method(e->axis->strategy());
    });
}

iad_result IAD_CALL iad_axis_stop_path(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_text_output(e->axis->stop_path(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_line_name(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_text_output(e->axis->line_name().value_or(std::string()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_monitor_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(monitor_reading(*e->axis), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_lean_status(iad_axis* axis, int32_t retries, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        std::optional<int> r;
        if (retries != IAD_INT_UNKNOWN) {
            if (retries < 0) throw std::invalid_argument("retries must be >= 0, or IAD_INT_UNKNOWN for the port's");
            r = retries;
        }
        to_c(e->axis->read_lean_status(r), out);
    });
}

iad_result IAD_CALL iad_axis_read_boot_baud_rate(iad_axis* axis, int32_t* rate) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(rate, "rate");
        *rate = IAD_INT_UNKNOWN;
        const std::optional<int> r = e->axis->read_boot_baud_rate();
        if (r) *rate = static_cast<int32_t>(*r);
    });
}

iad_result IAD_CALL iad_axis_gateway_monitor_json(iad_axis* axis, const char* slaves, char* buffer, size_t buffer_size,
                                                  size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(gateway_monitor_json(e->axis->bus()->line(), slaves, e->axis->bus()->name()), buffer, buffer_size,
                   required);
    });
}

iad_result IAD_CALL iad_axis_motion_caps(iad_axis* axis, iad_motion_caps* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(out, "out");
        iad_motion_caps c;
        std::memset(&c, 0, sizeof c);
        c.speed_cap = e->axis->speed_cap();
        c.accel_cap = e->axis->accel_cap();
        c.accel_only_cap = e->axis->accel_only_cap();
        c.decel_cap = e->axis->decel_cap();
        c.push_cap = e->axis->push_cap();
        const std::pair<std::optional<double>, std::string> min_speed = e->axis->recommended_min_speed();
        c.min_speed = min_speed.first ? *min_speed.first : kNaN;
        copy_text(c.min_speed_source, min_speed.second);
        *out = c;
    });
}

iad_result IAD_CALL iad_axis_read_home_pattern(iad_axis* axis, iad_bool* known, uint32_t* pattern) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(known, "known");
        need(pattern, "pattern");
        *known = IAD_FALSE;
        *pattern = 0;
        const std::optional<std::uint32_t> org = e->axis->read_home_pattern();
        if (org) {
            *known = IAD_TRUE;
            *pattern = *org;
        }
    });
}

iad_result IAD_CALL iad_axis_finish_home(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->finish_home(); });
}

// ============================================================================================ cell

iad_result IAD_CALL iad_cell_move_all(iad_cell* cell, const char* axes, const double* positions, size_t position_count,
                                      const iad_move_options* options, const double* speeds, const double* accels_g,
                                      const double* bands, const double* decels_g, iad_cell_result* results,
                                      size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes, false);
        if (list.empty()) throw std::invalid_argument("axes: iad_cell_move_all needs the axis names");
        if (position_count != list.size())
            throw std::invalid_argument(std::to_string(list.size()) + " axes listed but " +
                                        std::to_string(position_count) + " positions given");
        need(positions, "positions");
        const MoveOptions o = to_move_options(options, MoveOptions{});
        std::map<std::string, double> targets;
        PerAxisMoveValues per_axis;
        for (std::size_t i = 0; i < list.size(); ++i) {
            const std::string& name = list[i];
            if (!targets.emplace(name, finite(positions[i], "positions[i]")).second)
                throw std::invalid_argument("two moves for axis " + name);
            // NaN (or no array) = the options' value; the maps name every axis of the move or none (PerAxisMoveValues)
            const double speed = speeds && !std::isnan(speeds[i]) ? finite(speeds[i], "speeds[i]") : o.speed;
            const double accel = accels_g && !std::isnan(accels_g[i]) ? finite(accels_g[i], "accels_g[i]") : o.accel_g;
            if (speeds) per_axis.speed[name] = speed;
            if (accels_g) per_axis.accel_g[name] = accel;
            if (bands) per_axis.band[name] = !std::isnan(bands[i]) ? finite(bands[i], "bands[i]") : o.band;
            if (decels_g) {
                const double d = decels_g[i];
                // NaN or 0: the options' decel, else this axis's accel (decel == accel: the 0x9900 block)
                per_axis.decel_g[name] = !std::isnan(d) && d != 0.0 ? finite(d, "decels_g[i]") : o.decel_g.value_or(accel);
            }
        }
        std::optional<CellResults<Status>> r;
        try {
            r = e->cell->move_all(targets, o, per_axis, true);
        } catch (const CellError& refused) {
            // refused at the check (nothing started) or a start failed (the others stopped): every axis is reported
            fill_cell_results(list, {}, refused.errors(), results, capacity, count);
            throw;
        }
        if (!r) return;  // started without waiting: *count stays 0, results unwritten
        if (fill_cell_results(list, r->results, r->errors, results, capacity, count))
            throw CellError(r->operation, r->messages, r->errors);
    });
}

iad_result IAD_CALL iad_cell_pause_all(iad_cell* cell, const char* axes, iad_bool broadcast, iad_cell_result* results,
                                       size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        bool_results(e->cell->pause_all(list, true, broadcast != 0), results, capacity, count);
    });
}

iad_result IAD_CALL iad_cell_resume_all(iad_cell* cell, const char* axes, iad_bool broadcast, iad_cell_result* results,
                                        size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        bool_results(e->cell->resume_all(list, true, broadcast != 0), results, capacity, count);
    });
}

iad_result IAD_CALL iad_cell_cancel(iad_cell* cell, const char* axes) {
    return guarded([&] {
        const auto e = find_cell(cell);
        cancel_listed(*e->cell, axes, "cancel", [](Axis& axis) { axis.cancel_waits(); });
    });
}

iad_result IAD_CALL iad_cell_start_monitors(iad_cell* cell, const char* axes, const iad_monitor_options* options) {
    return guarded([&] {
        const auto e = find_cell(cell);
        const std::vector<std::string> list = cell_axes(*e->cell, axes);
        const MonitorOptions base = to_monitor_options(options);
        configure_queue(*e->events, options);
        // One call per axis: the callbacks carry the axis name (a monitor's on_change does not). A monitor that already
        // feeds this handle is started without them (Cell::start_monitors routes its events to the cell once).
        std::lock_guard<std::mutex> lock(e->mutex);
        for (const auto& name : list) {
            const std::shared_ptr<AxisMonitor> current = e->cell->axis_ptr(name)->monitor();
            std::weak_ptr<AxisMonitor>& hooked = e->hooked[name];
            const bool fed = current && current->running() && hooked.lock() == current;
            const auto started = e->cell->start_monitors({name}, fed ? base : queueing_callbacks(e->events, name, base));
            const auto it = started.find(name);
            if (it != started.end()) hooked = it->second;
        }
    });
}

iad_result IAD_CALL iad_cell_stop_monitors(iad_cell* cell, const char* axes) {
    return guarded([&] {
        const auto e = find_cell(cell);
        e->cell->stop_monitors(cell_axes(*e->cell, axes));
    });
}

iad_result IAD_CALL iad_cell_next_event(iad_cell* cell, double timeout_s, iad_event* out) {
    return guarded([&] {
        const auto e = find_cell(cell);
        need(out, "out");
        const std::vector<std::string> names = e->cell->names();
        next_event(*e->events, timeout_s, out, &names);
    });
}

iad_result IAD_CALL iad_cell_track_all(iad_cell* cell, const char* axes, const iad_track_options* options,
                                       const char* rows, iad_cell_result* results, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        recovery::TrackOptions o = to_track_options(options);
        o.rows = to_row_selection(rows);
        const auto r = e->cell->track_all(list, o, true);
        Json sessions = Json::object();
        for (const auto& kv : r.results) {
            if (!kv.second) {
                sessions.set(kv.first, Json());
                continue;
            }
            std::lock_guard<std::recursive_mutex> lock(kv.second->mutex);
            sessions.set(kv.first, kv.second->to_json());
        }
        keep(*e, sessions.dump());
        if (fill_cell_results(r.names, {}, r.errors, results, capacity, count))
            throw CellError(r.operation, r.messages, r.errors);
    });
}

iad_result IAD_CALL iad_cell_check_links(iad_cell* cell, const char* axes, iad_link_check* checks, size_t capacity,
                                         size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        const auto r = e->cell->check_links(list);
        Json all = Json::object();
        Json errors = Json::object();
        Json details = Json::object();  // (0.7, py-fixes) each failure as iad_last_error_json describes it
        for (std::size_t i = 0; i < r.names.size(); ++i) {
            const std::string& name = r.names[i];
            const auto ok = r.results.find(name);
            if (ok != r.results.end()) {
                all.set(name, ok->second.to_json());
                if (checks && i < capacity) to_c(ok->second, &checks[i]);
            } else {
                const auto msg = r.messages.find(name);
                const std::string text = msg != r.messages.end() ? msg->second : std::string("check failed");
                errors.set(name, text);
                const auto error = r.errors.find(name);
                if (error != r.errors.end() && error->second) details.set(name, describe_error(error->second).json);
                if (checks && i < capacity) {
                    recovery::LinkCheck failed;
                    failed.state = "error";
                    failed.error_text = text;
                    failed.axis = name;
                    to_c(failed, &checks[i]);
                    checks[i].ok = IAD_FALSE;
                }
            }
        }
        Json kept = Json::object();
        kept.set("checks", std::move(all));
        kept.set("errors", std::move(errors));
        kept.set("error_details", std::move(details));
        keep(*e, kept.dump());
        if (count) *count = r.names.size();
    });
}

iad_result IAD_CALL iad_cell_monitor_links(iad_cell* cell, double interval_s, const char* axes) {
    return guarded([&] {
        const auto e = find_cell(cell);
        const double interval = std::isnan(interval_s) ? 1.0 : interval_s;
        if (!(interval > 0.0) || !std::isfinite(interval)) throw std::invalid_argument("interval_s must be > 0");
        const std::vector<std::string> list = split_list(axes);
        const std::weak_ptr<EventQueue> weak = e->events;
        recovery::LinkMonitor::ChangeFn on_change = [weak](const std::string& name, const recovery::LinkCheck& check) {
            if (const auto q = weak.lock()) {
                Event ev;
                ev.kind = IAD_EVENT_LINK_STATE;
                ev.source = IAD_EVENT_SOURCE_CELL;
                ev.axis = name;
                ev.name = check.state;
                ev.status = check.status;
                ev.message = check.reasons.empty() ? check.error_text : join(check.reasons);
                q->push(std::move(ev));
            }
        };
        std::shared_ptr<recovery::LinkMonitor> started = e->cell->monitor_links(interval, list, on_change);
        std::shared_ptr<recovery::LinkMonitor> previous;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            previous = std::exchange(e->link_monitor, started);
        }
        if (previous && previous != started) previous->stop();
    });
}

iad_result IAD_CALL iad_cell_stop_link_monitor(iad_cell* cell) {
    return guarded([&] {
        const auto e = find_cell(cell);
        std::shared_ptr<recovery::LinkMonitor> monitor;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            monitor = std::move(e->link_monitor);
        }
        if (monitor) monitor->stop();
    });
}

iad_result IAD_CALL iad_cell_recover_all(iad_cell* cell, const char* axes, double timeout_s,
                                         const iad_recovery_policy* policy, const char* check_rows,
                                         iad_cell_result* results, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        const std::vector<std::string> list = split_list(axes);
        recovery::RecoveryPolicy p = to_recovery_policy(policy);
        p.check_rows = to_row_selection(check_rows);
        const double t = timeout_input(timeout_s, recovery::kRecoverTimeoutS, "timeout_s");
        const auto r = e->cell->recover_all(list, t, p, true);
        Json reports = Json::object();
        std::map<std::string, Status> ok;
        for (const auto& kv : r.results) {
            reports.set(kv.first, kv.second.to_json());
            if (kv.second.status) ok[kv.first] = *kv.second.status;
        }
        Json errors = Json::object();
        for (const auto& kv : r.messages) errors.set(kv.first, kv.second);
        Json kept = Json::object();
        kept.set("reports", std::move(reports));
        kept.set("errors", std::move(errors));
        keep(*e, kept.dump());
        if (fill_cell_results(r.names, ok, r.errors, results, capacity, count))
            throw CellError(r.operation, r.messages, r.errors);
    });
}

iad_result IAD_CALL iad_cell_monitor_all_json(iad_cell* cell, const char* axes, char* buffer, size_t buffer_size,
                                              size_t* required) {
    return guarded([&] {
        const auto e = find_cell_to_run(cell);
        check_text_output(buffer, buffer_size, required);
        const auto r = e->cell->monitor_all(split_list(axes), true);
        Json readings = Json::object();
        for (const auto& kv : r.results) readings.set(kv.first, kv.second);
        Json errors = Json::object();
        for (const auto& kv : r.messages) errors.set(kv.first, kv.second);
        Json j = Json::object();
        j.set("readings", std::move(readings));
        j.set("errors", std::move(errors));
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_lines_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        Json j = Json::object();
        for (const auto& kv : e->cell->ports()) {
            const std::shared_ptr<Line> line = kv.second->line();
            j.set(kv.first, line ? to_json(line->snapshot()) : Json());
        }
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_gateway_status_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        Json j = Json::object();
        for (const auto& kv : e->cell->gateway_status_all()) j.set(kv.first, to_json(kv.second));
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_set_position_unit(iad_cell* cell, const char* port, double unit_mm, const char* slaves) {
    return guarded([&] {
        const auto e = find_cell(cell);
        need(port, "port");
        const std::optional<double> unit = to_position_unit(unit_mm);
        if (!unit) throw std::invalid_argument("unit_mm must be 0.01 or 0.001");
        std::optional<std::vector<int>> which;
        if (slaves && *slaves) which = to_int_list(slaves, "slaves");
        const auto ports = e->cell->ports();
        const auto it = ports.find(port);
        if (it == ports.end()) throw std::out_of_range(std::string("no port '") + port + "' in this cell");
        it->second->set_position_unit(*unit, which);
    });
}

iad_result IAD_CALL iad_cell_gateway_monitor_json(iad_cell* cell, const char* port, const char* slaves, char* buffer,
                                                  size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        need(port, "port");
        const auto ports = e->cell->ports();
        const auto it = ports.find(port);
        if (it == ports.end()) throw std::out_of_range(std::string("no port '") + port + "' in this cell");
        write_json(gateway_monitor_json(it->second->line(), slaves, it->first), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_backup_json(iad_cell* cell, const char* axes, const char* directory, const char* rows,
                                         char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        need(directory, "directory");
        if (!*directory) throw std::invalid_argument("directory is empty");
        const info::BackupRows which = rows && *rows ? info::BackupRows::parse(rows) : info::BackupRows::used();
        Json j = Json::object();
        for (const auto& kv : e->cell->backup(directory, cell_axes(*e->cell, axes), which)) j.set(kv.first, kv.second);
        std::string text = j.dump();
        keep(*e, text);
        write_text_output(text, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_last_result_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_cell(cell);
        check_text_output(buffer, buffer_size, required);
        std::string text;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            if (!e->last_result) throw EcNotReadyError("no result kept on this cell handle yet");
            text = *e->last_result;
        }
        write_text_output(text, buffer, buffer_size, required);
    });
}

}  // extern "C"
