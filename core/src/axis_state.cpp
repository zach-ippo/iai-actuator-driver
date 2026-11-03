// What the bindings show of an axis on top of its commands (ABI 0.5): the state it keeps without I/O (AxisState: the
// facts read so far, the command state, the caps from what is known), the status samples it decoded last (a binding
// finds the raw words of a status it was handed), and the reads the Python edition offers as methods of its own:
// read_speed_limit, read_support_words, the directory, servo_on_timeout, user_parameters and the parameter model with
// the runtime visibility rules. Port of the Python edition's ec.py (status_words, motion_pending, stop_held,
// directory_absent, soft_limits_mm / soft_limits_note, ec_max_speed_mm_s / ec_max_speed_note, axis_limits, unit_code,
// categories, the *_cap properties, read_speed_limit, read_support_words, servo_on_timeout, user_parameters,
// parameter_model(visibility=True)). The reads are FC03 / FC11 only.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/info.hpp"

namespace iai_actuator_driver {

// ============================================================================================ test timings

namespace detail {

namespace {
std::atomic<double> g_home_start_s{kHomeStartS};
std::atomic<double> g_parw_ready_s{kParwReadyS};
std::atomic<double> g_parw_clear_s{kParwClearS};
}  // namespace

double home_start_s() { return g_home_start_s.load(); }
double parw_ready_s() { return g_parw_ready_s.load(); }
double parw_clear_s() { return g_parw_clear_s.load(); }

void set_test_timing(const std::string& name, std::optional<double> seconds) {
    if (seconds && !(*seconds >= 0.0 && std::isfinite(*seconds)))
        throw std::invalid_argument(name + " must be a number of seconds >= 0");
    const struct {
        const char* name;
        std::atomic<double>* value;
        double fallback;
    } timings[] = {{"home_start_s", &g_home_start_s, kHomeStartS},
                   {"parw_ready_s", &g_parw_ready_s, kParwReadyS},
                   {"parw_clear_s", &g_parw_clear_s, kParwClearS}};
    bool known = name.empty();
    for (const auto& t : timings) {
        if (!name.empty() && name != t.name) continue;
        known = true;
        t.value->store(seconds && !name.empty() ? *seconds : t.fallback);
    }
    if (!known)
        throw std::invalid_argument("unknown timing '" + name + "'; choose from home_start_s, parw_ready_s, parw_clear_s");
}

}  // namespace detail

namespace {

/// The smallest of the known caps, `fallback` while none is known.
double min_known(std::initializer_list<std::optional<double>> caps, double fallback) {
    std::optional<double> low;
    for (const std::optional<double>& c : caps)
        if (c && (!low || *c < *low)) low = c;
    return low ? *low : fallback;
}

}  // namespace

RangeInputs Axis::Impl::known_cap_inputs() const {
    const AxisOptions o = opts();
    RangeInputs in;
    in.configured_stroke = o.stroke;
    in.configured_max_speed = o.max_speed_mm_s;
    in.configured_max_accel = o.max_accel_g;
    in.configured_max_push = o.max_push_pct;
    std::lock_guard<std::mutex> lock(mutex);
    in.limits = limits;
    in.soft_limits = soft.limits;
    return in;
}

AxisState Axis::state() const {
    const AxisOptions o = impl_->opts();
    AxisState s;
    RangeInputs in;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        s.status_words = impl_->status_words;
        s.directory_absent = impl_->directory_absent;
        s.soft_limits_known = impl_->soft_known;
        s.soft_limits = impl_->soft.limits;
        if (impl_->soft_known) s.soft_limits_note = impl_->soft.note;
        s.limits = impl_->limits;
        s.system_code = impl_->system_code;
        s.unit_code = impl_->unit_code;
        s.model_name = impl_->model_name;
        s.boot_baud_rate = impl_->boot_baud_rate;
        s.max_positions = impl_->max_positions_cache;
        if (impl_->directory) s.categories = impl_->directory->records();
        s.p1_ceiling_mm = impl_->p1_ceiling_mm;
    }
    s.motion_pending = impl_->motion_pending.load();
    s.stop_held = impl_->stop_coil_on.load();
    if (s.limits) {
        const auto cap = s.limits->unit_speed_cap();
        s.unit_max_speed = cap.first;
        s.unit_max_speed_note = cap.second;
    }
    // the caps of speed_cap() / accel_cap() ... from what is known: no read (the fallbacks while nothing is)
    in.limits = s.limits;
    in.soft_limits = s.soft_limits;
    in.configured_stroke = o.stroke;
    in.configured_max_speed = o.max_speed_mm_s;
    in.configured_max_accel = o.max_accel_g;
    in.configured_max_push = o.max_push_pct;
    s.speed_cap = iai_actuator_driver::speed_cap(in);
    s.accel_cap = iai_actuator_driver::accel_cap(in);
    s.push_cap = iai_actuator_driver::push_cap_pct(in);
    s.accel_only_cap = min_known({o.max_accel_g, s.limits ? s.limits->max_accel_g : std::nullopt}, kFallbackAccelCapG);
    s.decel_cap = min_known({o.max_accel_g, s.limits ? s.limits->max_decel_g : std::nullopt}, kFallbackAccelCapG);
    return s;
}

std::vector<Status> Axis::recent_statuses() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return std::vector<Status>(impl_->recent_statuses.begin(), impl_->recent_statuses.end());
}

std::optional<double> Axis::read_speed_limit() {
    const AxisLimits limits = impl_->read_limits();
    return limits.unit_speed_cap().first;
}

std::optional<std::map<std::string, std::uint32_t>> Axis::read_support_words() {
    const UnitDirectory dir = impl_->ensure_directory();
    if (dir.empty()) return std::nullopt;
    try {
        return dir.read_support_words(impl_->rd());
    } catch (const ModbusSlaveError&) {
        return std::nullopt;
    }
}

std::vector<IndexCategory> Axis::categories() { return impl_->ensure_directory().records(); }

void Axis::set_axis_limits(const AxisLimits& limits) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->limits = limits;
    impl_->mvpt = limits.move_patterns;  // the acceleration patterns are checked against these limits' MVPT too
    impl_->mvpt_known = true;
}

void Axis::release_held() { impl_->release_held(); }

std::pair<double, std::string> Axis::servo_on_timeout(const std::optional<Status>& status) {
    const Status st = status ? *status : impl_->status();
    std::string why;
    const double timeout = impl_->servo_on_timeout(st, &why);
    return {timeout, why};
}

Json Axis::user_parameters() {
    const ParameterContext ctx = impl_->parameter_context(std::nullopt);
    Json rows = info::user_parameter_rows(ctx);
    for (const Json& row : rows.as_array()) {  // P1 read: the ceiling a later P1 write may not exceed
        const Json* no = row.find("no");
        const Json* found = row.find("found");
        const Json* value = row.find("value");
        if (no && no->type() == Json::Type::Int && no->as_int() == 1 && found && found->as_bool() && value &&
            (value->type() == Json::Type::Int || value->type() == Json::Type::Number))
            impl_->note_p1(value->as_number());
    }
    return rows;
}

ParameterModel Axis::parameter_model(bool visibility) {
    if (visibility) return model_parameter_list(impl_->parameter_context(std::nullopt));
    // the unit code and the model string only (FC11 + 0x0600), as the Python edition's parameter_model()
    ParameterContext ctx;
    ctx.is_ec = impl_->is_ec();
    ctx.system_code = impl_->read_system_code();
    ctx.model_name = impl_->read_model_name();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        ctx.unit_code = impl_->unit_code;
    }
    return model_parameter_list(ctx);
}

}  // namespace iai_actuator_driver
