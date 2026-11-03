// Private helpers of the C ABI 0.2 additions (c_api_impl.hpp): the per-handle event queue that turns the core's
// callbacks into data the binding polls, and the conversions of the 0.2 structs (push, sequences, move handles, the
// status monitor, fast mode, recovery, the self-test). The exported functions are in c_api_ext.cpp.
//
// Evidence: UNVERIFIED on hardware (the C ABI is tested on the simulator only).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include "c_api_impl.hpp"
#include "iai_actuator_driver/timing.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace iai_actuator_driver {
namespace capi {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double num(const std::optional<double>& value) { return value ? *value : kNaN; }
iad_tribool tri(const std::optional<bool>& value) { return value ? (*value ? IAD_TRUE : IAD_FALSE) : IAD_UNKNOWN; }
iad_bool flag(bool value) { return value ? IAD_TRUE : IAD_FALSE; }

double wall_now_s() {
    using clock = std::chrono::system_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

/// The IAD_EVENTS_* bit of an event kind.
std::int32_t mask_bit(std::int32_t kind) {
    switch (kind) {
        case IAD_EVENT_STATUS: return IAD_EVENTS_STATUS;
        case IAD_EVENT_DONE: return IAD_EVENTS_DONE;
        case IAD_EVENT_LINK:
        case IAD_EVENT_FAULT:
        case IAD_EVENT_LINK_STATE: return IAD_EVENTS_LINK;
        case IAD_EVENT_CHECK_ITEM: return IAD_EVENTS_CHECK;
        default: return 0;
    }
}

/// The longest wait of one pop (a binding waits in slices anyway; this keeps the deadline arithmetic finite).
constexpr double kMaxWaitS = 86400.0;

}  // namespace

// ============================================================================================ event queue

void EventQueue::configure(std::int32_t mask, std::int32_t capacity) {
    if (mask < 0 || mask > IAD_EVENTS_ALL)
        throw std::invalid_argument("events must be a combination of IAD_EVENTS_* (0..15), got " + std::to_string(mask));
    if (capacity < 1 || capacity > 65536)
        throw std::invalid_argument("queue_capacity must be 1..65536, got " + std::to_string(capacity));
    std::lock_guard<std::mutex> lock(mutex_);
    mask_ = mask;
    capacity_ = static_cast<std::size_t>(capacity);
    while (items_.size() > capacity_) {
        items_.pop_front();
        ++dropped_;
    }
}

std::uint64_t native_thread_id() noexcept {
#if defined(_WIN32)
    return static_cast<std::uint64_t>(GetCurrentThreadId());
#elif defined(__linux__)
    return static_cast<std::uint64_t>(::syscall(SYS_gettid));
#elif defined(__APPLE__)
    std::uint64_t id = 0;
    return pthread_threadid_np(nullptr, &id) == 0 ? id : 0;
#else
    return 0;
#endif
}

void EventQueue::push(Event event) noexcept {
    try {
        if (event.time_s == 0.0) event.time_s = now_s();
        if (event.caller_thread == 0) event.caller_thread = native_thread_id();
        if (event.wall_time_s == 0.0) event.wall_time_s = wall_now_s();
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || (mask_ & mask_bit(event.kind)) == 0) return;
        if (items_.size() >= capacity_) {
            items_.pop_front();
            ++dropped_;
        }
        items_.push_back(Item{std::move(event), ++next_});
    } catch (...) {
        return;  // out of memory: the event is lost (a callback must never break the monitor)
    }
    wake_.notify_all();
}

std::optional<Event> EventQueue::pop(double timeout_s, std::int64_t* sequence, std::int32_t* dropped) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!closed_ && items_.empty() && timeout_s > 0.0) {  // NaN compares false: no wait
        const auto wait = std::chrono::duration<double>(std::min(timeout_s, kMaxWaitS));
        wake_.wait_for(lock, wait, [this] { return closed_ || !items_.empty(); });
    }
    if (closed_) throw std::invalid_argument("the handle was closed while waiting for an event");
    if (items_.empty()) return std::nullopt;
    Item item = std::move(items_.front());
    items_.pop_front();
    *sequence = item.sequence;
    *dropped = dropped_;
    dropped_ = 0;
    return std::move(item.event);
}

void EventQueue::close() noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        items_.clear();
    }
    wake_.notify_all();
}

MonitorOptions queueing_callbacks(const std::shared_ptr<EventQueue>& queue, const std::string& axis_name,
                                  MonitorOptions base) {
    const std::weak_ptr<EventQueue> weak = queue;
    base.on_change.push_back([weak, axis_name](const Status& status, const std::optional<Status>&) {
        if (const auto q = weak.lock()) {
            Event e;
            e.kind = IAD_EVENT_STATUS;
            e.source = IAD_EVENT_SOURCE_MONITOR;  // the monitor's polls, as its link watch's events below
            e.status = status;
            e.axis = axis_name;
            q->push(std::move(e));
        }
    });
    base.on_done.push_back([weak, axis_name](const Status& status) {
        if (const auto q = weak.lock()) {
            Event e;
            e.kind = IAD_EVENT_DONE;
            e.source = IAD_EVENT_SOURCE_MONITOR;
            e.status = status;
            e.axis = axis_name;
            q->push(std::move(e));
        }
    });
    base.on_event.push_back([weak, axis_name](const LinkEvent& link) {
        if (const auto q = weak.lock()) {
            Event e;
            e.kind = IAD_EVENT_LINK;
            e.source = IAD_EVENT_SOURCE_MONITOR;
            e.axis = link.axis.empty() ? axis_name : link.axis;
            e.name = link.kind;
            e.cause = link.cause;
            e.message = link.message;
            e.wall_time_s = link.time;
            q->push(std::move(e));
        }
    });
    return base;
}

recovery::EventCallback queueing_recovery_events(const std::shared_ptr<EventQueue>& queue, std::int32_t source) {
    const std::weak_ptr<EventQueue> weak = queue;
    return [weak, source](const recovery::RecoveryEvent& event) {
        if (const auto q = weak.lock()) {
            Event e;
            e.kind = IAD_EVENT_LINK;
            e.source = source;
            e.axis = event.axis;
            e.name = event.kind;
            e.cause = event.cause;
            e.message = event.message;
            e.wall_time_s = event.time;
            q->push(std::move(e));
        }
    };
}

CellOptions queueing_cell_options(CellOptions options, const std::shared_ptr<EventQueue>& queue) {
    const std::weak_ptr<EventQueue> weak = queue;
    options.on_fault = [weak](const std::string& name, const std::string& kind, const std::string& message) {
        if (const auto q = weak.lock()) {
            Event e;
            e.kind = IAD_EVENT_FAULT;
            e.source = IAD_EVENT_SOURCE_CELL;
            e.axis = name;
            e.name = kind;
            e.cause = kind;
            e.message = message;
            q->push(std::move(e));
        }
    };
    options.on_event = queueing_recovery_events(queue, IAD_EVENT_SOURCE_CELL);
    return options;
}

// ============================================================================================ inputs

std::vector<int> to_int_list(const char* text, const char* what) {
    std::vector<int> out;
    for (const std::string& item : split_list(text, false)) {
        std::size_t used = 0;
        int value = 0;
        try {
            value = std::stoi(item, &used);
        } catch (const std::exception&) {
            used = 0;
        }
        if (used != item.size() || used == 0)
            throw std::invalid_argument(std::string(what) + ": '" + item + "' is not an integer");
        out.push_back(value);
    }
    return out;
}

recovery::RowSelection to_row_selection(const char* rows) {
    if (!rows || !*rows) return recovery::RowSelection::none();
    if (std::string(rows) == "all") return recovery::RowSelection::every();
    std::vector<int> numbers = to_int_list(rows, "rows (\"all\" or row numbers such as \"0,1,2\")");
    for (int n : numbers)
        if (n < 0) throw std::invalid_argument("rows: a row number must be >= 0, got " + std::to_string(n));
    return recovery::RowSelection::of(std::move(numbers));
}

Json parse_backup(const char* text) {
    need(text, "backup_json");
    Json j = Json::parse(text);
    if (j.type() != Json::Type::Object) throw std::invalid_argument("backup_json: a backup is a JSON object");
    return j;
}

PushOptions to_push_options(const iad_push_options* input) {
    PushOptions o;
    if (input) {
        const iad_push_options in = read_input(input, "iad_push_options");
        const iad_push_options* p = &in;
        o.speed = or_default(p->speed, o.speed);
        o.accel_g = or_default(p->accel_g, o.accel_g);
        o.timeout_s = timeout_arg(p->timeout_s, "timeout_s");
        if (p->direction != 0 && p->direction != 1 && p->direction != -1)
            throw std::invalid_argument("direction must be +1, -1 or 0 (= +1), got " + std::to_string(p->direction));
        o.direction = p->direction == 0 ? 1 : p->direction;
        o.wait = p->wait != 0;
    }
    return o;
}

SequenceOptions to_sequence_options(const iad_sequence_options* input) {
    SequenceOptions o;
    if (input) {
        const iad_sequence_options in = read_input(input, "iad_sequence_options");
        const iad_sequence_options* p = &in;
        o.speed = or_default(p->speed, o.speed);
        o.accel_g = or_default(p->accel_g, o.accel_g);
        o.band = or_default(p->band, o.band);
        o.blend_band = opt(p->blend_band);
        o.timeout_s = timeout_arg(p->timeout_s, "timeout_s");
        o.completion = to_completion(p->completion);
    }
    return o;
}

SequencePoint from_c(const iad_sequence_point& p) {  // p: read by read_input_array (struct_size, reserved checked)
    // 0.8: NaN or 0 = the sequence's
    return SequencePoint(finite(p.position, "points[i].position"), opt_nonzero(p.speed), opt_nonzero(p.accel_g),
                         opt_nonzero(p.band));
}

std::vector<AxisMove> to_axis_moves(const iad_axis_move* moves, std::size_t count) {
    std::vector<AxisMove> list;
    const std::vector<iad_axis_move> in = read_input_array(moves, count, "moves");
    for (std::size_t i = 0; i < in.size(); ++i) {
        const iad_axis_move& m = in[i];
        const std::string at = "moves[" + std::to_string(i) + "]";
        if (!std::memchr(m.axis, '\0', sizeof m.axis))
            throw std::invalid_argument(at + ".axis does not fit its " + std::to_string(sizeof m.axis) +
                                        " bytes with its NUL (a name is never cut)");
        // NaN or 0 = the default (a struct without an _init function; 0 means nothing for these)
        const auto given = [](double v) { return !std::isnan(v) && v != 0.0; };
        AxisMove a;
        a.axis = m.axis;
        a.position = finite(m.position, (at + ".position").c_str());
        MoveOptions o;
        if (given(m.speed)) o.speed = m.speed;
        if (given(m.accel_g)) o.accel_g = m.accel_g;
        if (given(m.band)) o.band = m.band;
        // a timeout: NaN or 0 the default (as documented for this struct), any other value finite and > 0
        if (given(m.timeout_s)) o.timeout_s = timeout_arg(m.timeout_s, (at + ".timeout_s").c_str());
        if (given(m.decel_g)) o.decel_g = finite(m.decel_g, (at + ".decel_g").c_str());
        o.accel_pattern = to_accel_pattern(m.accel_pattern);
        o.completion = to_completion(m.completion);
        a.options = o;
        list.push_back(std::move(a));
    }
    return list;
}

MonitorOptions to_monitor_options(const iad_monitor_options* input) {
    MonitorOptions o;
    if (input) {
        const iad_monitor_options in = read_input(input, "iad_monitor_options");
        const iad_monitor_options* p = &in;
        o.gap_s = or_default(p->gap_s, o.gap_s);
        if (std::isnan(p->link_check_s))
            o.link_check_s = kLinkCheckS;
        else
            o.link_check_s = p->link_check_s;  // 0: never (checked by AxisMonitor::create)
        if (p->retries < 0 || p->retries > 100) throw std::invalid_argument("retries must be 0..100");
        o.retries = p->retries;
    }
    return o;
}

recovery::TrackOptions to_track_options(const iad_track_options* input) {
    recovery::TrackOptions o;
    if (input) {
        const iad_track_options in = read_input(input, "iad_track_options");
        const iad_track_options* p = &in;
        o.alarm_history = p->alarm_history != 0;
        o.parameters = p->parameters != 0;
    }
    return o;
}

recovery::RecoveryPolicy to_recovery_policy(const iad_recovery_policy* input) {
    recovery::RecoveryPolicy o;
    if (input) {
        const iad_recovery_policy in = read_input(input, "iad_recovery_policy");
        const iad_recovery_policy* p = &in;
        o.position_tolerance_mm = or_default(p->position_tolerance_mm, o.position_tolerance_mm);
        if (!(o.position_tolerance_mm >= 0.0) || !std::isfinite(o.position_tolerance_mm))
            throw std::invalid_argument("position_tolerance_mm must be >= 0");
        o.poll_s = or_default(p->poll_s, o.poll_s);
        if (!(o.poll_s > 0.0) || !std::isfinite(o.poll_s)) throw std::invalid_argument("poll_s must be > 0");
        if (p->boot_baud_rate != 0 && p->boot_baud_rate != IAD_INT_UNKNOWN) {
            if (p->boot_baud_rate < 0) throw std::invalid_argument("boot_baud_rate must be a positive rate");
            o.boot_baud_rate = p->boot_baud_rate;
        }
        o.restore_baud_rate = p->restore_baud_rate != 0;
        o.restore_control = p->restore_control != 0;
        o.restore_safety_speed = p->restore_safety_speed != 0;
        o.clear_message_alarms = p->clear_message_alarms != 0;
        o.check_parameters = p->check_parameters != 0;
    }
    return o;
}

CheckOptions to_check_options(const iad_check_options* input) {
    CheckOptions o;
    if (input) {
        const iad_check_options in = read_input(input, "iad_check_options");
        const iad_check_options* p = &in;
        o.step_mm = or_default(p->step_mm, o.step_mm);
        o.speed_mm_s = or_default(p->speed_mm_s, o.speed_mm_s);
        o.accel_g = or_default(p->accel_g, o.accel_g);
        if (!(o.step_mm > 0.0) || !std::isfinite(o.step_mm)) throw std::invalid_argument("step_mm must be > 0");
        if (!(o.speed_mm_s > 0.0) || !std::isfinite(o.speed_mm_s)) throw std::invalid_argument("speed_mm_s must be > 0");
        if (!(o.accel_g > 0.0) || !std::isfinite(o.accel_g)) throw std::invalid_argument("accel_g must be > 0");
        o.motion = p->motion != 0;
    }
    return o;
}

// ============================================================================================ outputs

std::string join(const std::vector<std::string>& parts) {
    std::string out;
    for (const auto& p : parts) {
        if (!out.empty()) out += "; ";
        out += p;
    }
    return out;
}

void to_c(const PushResult& r, iad_push_result* out) {
    std::memset(out, 0, sizeof *out);
    to_c(r.status, &out->status);
    out->position_mm = r.position_mm;
    out->outcome = r.contact() ? IAD_PUSH_CONTACT : r.missed() ? IAD_PUSH_MISSED : 0;
    out->warning_count = static_cast<std::int32_t>(r.warnings.size());
    copy_text(out->outcome_name, r.outcome);
}

void to_c(const MoveHandle& h, iad_move_info* out) {
    std::memset(out, 0, sizeof *out);
    const std::optional<Status> result = h.result();
    if (result)
        to_c(*result, &out->result);
    else
        unknown_status(&out->result);
    out->target = h.target();
    out->band = h.band();
    out->timeout_s = h.timeout_s();
    out->sent_at_s = h.sent_at();
    out->deadline_s = h.deadline();
    out->completion = from_completion(h.completion());
    out->has_result = flag(result.has_value());
    out->cancelled = flag(h.cancelled());
}

void to_c(const MonitorSample& s, iad_monitor_sample* out) {
    std::memset(out, 0, sizeof *out);
    to_c(s.status, &out->status);
    out->t_start_s = s.t_start;
    out->t_s = s.t;
    out->age_s = s.age_s();
    out->poll_rate_hz = kNaN;
    out->seq = static_cast<std::int64_t>(s.seq);
}

void to_c(const FastModeSummary& s, iad_fast_summary* out) {
    std::memset(out, 0, sizeof *out);
    out->baud_rate = s.baud_rate.value_or(0);
    out->completion = from_completion(s.completion);
    out->monitor = flag(s.monitor);
    out->note_count = static_cast<std::int32_t>(s.notes.size());
    copy_text(out->notes, join(s.notes));
}

void to_c(const recovery::LinkCheck& c, iad_link_check* out) {
    std::memset(out, 0, sizeof *out);
    if (c.status)
        to_c(*c.status, &out->status);
    else
        unknown_status(&out->status);
    out->elapsed_s = c.elapsed_s;
    out->ok = flag(c.ok());
    out->reason_count = static_cast<std::int32_t>(c.reasons.size());
    copy_text(out->state, c.state);
    copy_text(out->reasons, join(c.reasons));
    copy_text(out->error, c.error_text);
}

void to_c(const recovery::RecoveryReport& r, iad_recovery_summary* out) {
    std::memset(out, 0, sizeof *out);
    if (r.status)
        to_c(*r.status, &out->status);
    else
        unknown_status(&out->status);
    out->position_mm = num(r.position_mm);
    out->last_target_mm = num(r.last_target_mm);
    out->position_error_mm = num(r.position_error_mm);
    out->elapsed_s = r.elapsed_s;
    out->baud_rate = r.baud_rate.value_or(0);
    out->baud_rate_found = r.baud_rate_found.value_or(0);
    out->alarm_code = r.alarm_code;
    out->restarted = flag(r.restarted);
    out->same_unit = flag(r.same_unit);
    out->port_reopened = flag(r.port_reopened);
    out->baud_restored = flag(r.baud_restored);
    out->control_restored = flag(r.control_restored);
    out->safety_speed_restored = flag(r.safety_speed_restored);
    out->needs_home = flag(r.needs_home);
    out->needs_attention = flag(r.needs_attention());
    out->ready_to_move = flag(r.ready_to_move());
    out->fault_alarm = flag(r.fault_alarm());
    out->at_target = tri(r.at_target);
    out->servo_on = tri(r.servo_on);
    out->homed = tri(r.homed);
    out->modbus_control = tri(r.modbus_control);
    out->safety_speed = tri(r.safety_speed);
    out->note_count = static_cast<std::int32_t>(r.notes.size());
    out->control_kept = flag(r.control_kept);
    copy_text(out->axis, r.axis);
    copy_text(out->cause, r.cause);
    copy_text(out->summary, r.summary());
}

void to_c(const CheckReport& r, iad_check_summary* out) {
    std::memset(out, 0, sizeof *out);
    out->passed = flag(r.passed());
    out->items = static_cast<std::int32_t>(r.items.size());
    for (const CheckItem& item : r.items) {
        if (item.result == kCheckFail) ++out->failed;
        if (item.result == kCheckWarn) ++out->warned;
        if (item.result == kCheckSkip) ++out->skipped;
    }
}

void to_c(const Event& e, std::int64_t sequence, std::int32_t dropped, iad_event* out) {
    std::memset(out, 0, sizeof *out);
    if (e.status)
        to_c(*e.status, &out->status);
    else
        unknown_status(&out->status);
    out->time_s = e.time_s;
    out->wall_time_s = e.wall_time_s;
    out->source = e.source;
    out->index = IAD_INT_UNKNOWN;  // (0.8) a cell's queue sets the axis's position in the cell (next_event)
    out->caller_thread = e.caller_thread;
    out->sequence = sequence;
    out->kind = e.kind;
    out->dropped = dropped;
    copy_text(out->axis, e.axis);
    copy_text(out->name, e.name);
    copy_text(out->cause, e.cause);
    copy_text(out->message, e.message);
}

void no_event(iad_event* out) {
    std::memset(out, 0, sizeof *out);
    unknown_status(&out->status);
    out->kind = IAD_EVENT_NONE;
    out->index = IAD_INT_UNKNOWN;
}

}  // namespace capi
}  // namespace iai_actuator_driver
