// C ABI 0.5 axis additions (include/iai_actuator_driver.h, "axis: state without I/O"): the raw words of a status a
// binding was handed, the state an axis keeps without I/O, the reads the Python edition has as methods of their own
// (soft limits, axis limits, speed limit, support words, directory, parameter model, user parameters, push speed, servo
// ON timeout), a row write without the checksum, begin_stop and the core's constants. The work is in
// src/axis_state.cpp and the axis unit; this file converts. Evidence: UNVERIFIED on hardware (simulator tests only).
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/results.hpp"
#include "iai_actuator_driver/transport.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

void write_json(const Json& json, char* buffer, std::size_t buffer_size, std::size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

template <class T>
Json opt_json(const std::optional<T>& value) {
    return value ? Json(*value) : Json();
}

Json span_or_null(const std::optional<Span>& value) { return value ? to_json(*value) : Json(); }

/// A Status's raw words in 0x9000 order (the inverse of Status::from_registers): 8, 14, 20 or 22 words.
std::vector<std::uint16_t> status_words(const StatusRaw& raw) {
    std::vector<std::uint16_t> w;
    auto u32 = [&w](std::uint32_t value) {
        const auto hl = reg::to_u32_words(value);
        w.push_back(hl.first);
        w.push_back(hl.second);
    };
    u32(raw.pnow);
    for (std::uint16_t v : {raw.almc, raw.dipm, raw.dopm, raw.dss1, raw.dss2, raw.dsse}) w.push_back(v);
    if (raw.words >= 14) {
        u32(raw.stat.value_or(0));
        u32(raw.vnow.value_or(0));
        u32(raw.cnow.value_or(0));
    }
    if (raw.words >= 20) {
        u32(raw.devi.value_or(0));
        u32(raw.stim.value_or(0));
        w.push_back(raw.sipm.value_or(0));
        w.push_back(raw.zons.value_or(0));
    }
    if (raw.words >= 22) {
        w.push_back(raw.poss.value_or(0));
        w.push_back(raw.ssse.value_or(0));
    }
    return w;
}

/// The fields a binding's iad_status carries that identify the sample.
bool same_sample(const Status& s, const iad_status& like) {
    return s.raw.words == like.raw_words && s.raw.dss1 == like.raw_dss1 && s.raw.dss2 == like.raw_dss2 &&
           s.raw.dsse == like.raw_dsse && s.raw.dipm == like.raw_dipm && s.raw.dopm == like.raw_dopm &&
           s.alarm_code == like.alarm_code && s.position_mm == like.position_mm;
}

Json axis_limits_json(const AxisLimits& l) {
    Json j = Json::object();
    j.set("max_speed_mm_s", opt_json(l.max_speed_mm_s));
    j.set("max_accel_g", opt_json(l.max_accel_g));
    j.set("max_decel_g", opt_json(l.max_decel_g));
    j.set("max_push_speed_mm_s", opt_json(l.max_push_speed_mm_s));
    j.set("max_push_pct", opt_json(l.max_push_pct));
    j.set("min_push_pct", l.min_push_pct);
    j.set("min_push_from_ec", l.min_push_from_unit);
    j.set("push_force_factor", opt_json(l.push_force_factor));
    j.set("push_force_offset", opt_json(l.push_force_offset));
    j.set("actuator_kind", opt_json(l.actuator_kind));
    j.set("rotary_atyp", opt_json(l.rotary_atyp));
    j.set("default_speed_mm_s", opt_json(l.default_speed_mm_s));
    j.set("supports_push", opt_json(l.supports_push));
    j.set("supports_relative", opt_json(l.supports_relative));
    j.set("source", l.source);
    j.set("max_positions", opt_json(l.max_positions));
    j.set("min_band", opt_json(l.min_band));
    j.set("lead_mm", opt_json(l.lead_mm));
    j.set("move_patterns", opt_json(l.move_patterns));
    j.set("notes", l.notes);
    return j;
}

Json categories_json(const std::vector<IndexCategory>& records) {
    Json list = Json::array();
    for (const IndexCategory& c : records) list.push_back(to_json(c));
    return list;
}

Json support_json(const std::optional<std::map<std::string, std::uint32_t>>& words) {
    if (!words) return Json();
    Json j = Json::object();
    for (const auto& w : *words) j.set(w.first, w.second);
    return j;
}

}  // namespace

extern "C" {

iad_result IAD_CALL iad_axis_last_status_words(iad_axis* axis, const iad_status* like, uint16_t* words,
                                               size_t capacity, size_t* count) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(count, "count");
        if (capacity > 0) need(words, "words");
        for (const Status& s : e->axis->recent_statuses()) {
            if (like && !same_sample(s, *like)) continue;
            const std::vector<std::uint16_t> w = status_words(s.raw);
            *count = w.size();
            if (capacity < w.size())
                throw capi::BufferTooSmall("words: " + std::to_string(w.size()) + " needed, capacity " +
                                           std::to_string(capacity));
            for (std::size_t i = 0; i < w.size(); ++i) words[i] = w[i];
            return;
        }
        *count = 0;
        throw EcNotReadyError(like ? "no status sample of this axis matches (16 newer ones were read since)"
                                   : "no status read on this axis yet");
    });
}

iad_result IAD_CALL iad_axis_state_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const AxisState s = e->axis->state();
        Json j = Json::object();
        j.set("status_words", s.status_words);
        j.set("motion_pending", s.motion_pending);
        j.set("stop_held", s.stop_held);
        j.set("directory_absent", s.directory_absent);
        j.set("soft_limits_known", s.soft_limits_known);
        j.set("soft_limits_mm", span_or_null(s.soft_limits));
        j.set("soft_limits_note", s.soft_limits_note);
        j.set("axis_limits", s.limits ? axis_limits_json(*s.limits) : Json());
        j.set("ec_max_speed_mm_s", opt_json(s.unit_max_speed));
        j.set("ec_max_speed_note", s.unit_max_speed_note);
        j.set("system_code", opt_json(s.system_code));
        j.set("unit_code", opt_json(s.unit_code));
        j.set("model_name", opt_json(s.model_name));
        j.set("boot_baud_rate", opt_json(s.boot_baud_rate));
        j.set("max_positions", opt_json(s.max_positions));
        j.set("categories", categories_json(s.categories));
        j.set("p1_ceiling_mm", opt_json(s.p1_ceiling_mm));
        j.set("speed_cap_mm_s", s.speed_cap);
        j.set("accel_cap_g", s.accel_cap);
        j.set("accel_only_cap_g", s.accel_only_cap);
        j.set("decel_cap_g", s.decel_cap);
        j.set("push_cap_pct", s.push_cap);
        j.set("fast_mode", e->axis->fast_mode());
        j.set("fast_notes", e->axis->fast_notes());
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_kind_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        Json j = Json::object();
        j.set("units", e->axis->units());
        j.set("kind", e->axis->kind());
        j.set("product_family", e->axis->product_family());
        j.set("is_ec", e->axis->is_ec());
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_soft_limits_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                   size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const SoftLimits s = e->axis->read_soft_limits();
        Json j = Json::object();
        j.set("soft_limits_mm", span_or_null(s.limits));
        j.set("note", s.note);
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_axis_limits_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                   size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(axis_limits_json(e->axis->read_axis_limits()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_read_speed_limit(iad_axis* axis, double* speed) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(speed, "speed");
        const std::optional<double> v = e->axis->read_speed_limit();
        *speed = v ? *v : std::numeric_limits<double>::quiet_NaN();
    });
}

iad_result IAD_CALL iad_axis_read_support_words_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                     size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(support_json(e->axis->read_support_words()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_categories_json(iad_axis* axis, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(categories_json(e->axis->categories()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_parameter_model_json(iad_axis* axis, iad_bool visibility, char* buffer,
                                                  size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(to_json(e->axis->parameter_model(visibility != 0)), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_user_parameters_json(iad_axis* axis, char* buffer, size_t buffer_size,
                                                  size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        write_json(e->axis->user_parameters(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_write_position_row(iad_axis* axis, int32_t row, const iad_position_entry* entry,
                                                const iad_nv_write_options* options, iad_bool write_checksum,
                                                iad_position_write_result* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const PositionEntry pe = from_c(*need(entry, "entry"));
        NvWriteOptions o = to_nv_options(options);
        o.write_checksum = write_checksum != 0;
        const PositionWriteResult r = e->axis->write_position_table(row, pe, o);
        {
            std::string text = to_json(r).dump();
            keep_thread_result(text);  // (0.7, py-fixes) iad_last_result_json
            std::lock_guard<std::mutex> lock(e->mutex);
            e->last_result = std::move(text);
        }
        if (out) to_c(r, out);
    });
}

iad_result IAD_CALL iad_axis_enable_modbus_control(iad_axis* axis, iad_bool verify, iad_status* out) {
    return guarded([&] {
        const auto e = find_axis(axis);
        if (out) unknown_status(out);
        const std::optional<Status> st = e->axis->enable_modbus_control(verify != 0);
        if (st && out) to_c(*st, out);
    });
}

iad_result IAD_CALL iad_axis_begin_stop(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->begin_stop(); });
}

iad_result IAD_CALL iad_axis_push_speed_json(iad_axis* axis, double speed, char* buffer, size_t buffer_size,
                                             size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const auto r = e->axis->push_speed(std::isnan(speed) ? std::nullopt : std::optional<double>(speed));
        Json j = Json::object();
        j.set("speed_mm_s", r.first);
        j.set("source", r.second);
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_servo_on_timeout_json(iad_axis* axis, const uint16_t* words, size_t count, char* buffer,
                                                   size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        std::optional<Status> st;
        if (words && count > 0) st = Status::from_registers(std::vector<std::uint16_t>(words, words + count));
        const auto r = e->axis->servo_on_timeout(st);
        Json j = Json::object();
        j.set("timeout_s", r.first);
        j.set("reason", r.second);
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_release_held(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->release_held(); });
}

iad_result IAD_CALL iad_axis_set_axis_limits_json(iad_axis* axis, const char* limits_json) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const Json j = Json::parse(need(limits_json, "limits_json"));
        if (j.type() != Json::Type::Object) throw std::invalid_argument("limits_json must be a JSON object");
        auto number = [&j](const char* key) -> std::optional<double> {
            const Json* v = j.find(key);
            if (!v || v->is_null()) return std::nullopt;
            return v->as_number();
        };
        auto integer = [&j](const char* key) -> std::optional<int> {
            const Json* v = j.find(key);
            if (!v || v->is_null()) return std::nullopt;
            return static_cast<int>(v->as_int());
        };
        auto flag = [&j](const char* key) -> std::optional<bool> {
            const Json* v = j.find(key);
            if (!v || v->is_null()) return std::nullopt;
            return v->as_bool();
        };
        AxisLimits l;
        l.max_speed_mm_s = number("max_speed_mm_s");
        l.max_accel_g = number("max_accel_g");
        l.max_decel_g = number("max_decel_g");
        l.max_push_speed_mm_s = number("max_push_speed_mm_s");
        l.max_push_pct = number("max_push_pct");
        l.min_push_pct = number("min_push_pct").value_or(reg::kMnppDefaultPct);
        l.min_push_from_unit = flag("min_push_from_ec").value_or(false);
        l.push_force_factor = integer("push_force_factor");
        l.push_force_offset = integer("push_force_offset");
        l.actuator_kind = integer("actuator_kind");
        l.rotary_atyp = flag("rotary_atyp");
        l.default_speed_mm_s = number("default_speed_mm_s");
        l.supports_push = flag("supports_push");
        l.supports_relative = flag("supports_relative");
        if (const Json* s = j.find("source"); s && s->type() == Json::Type::String) l.source = s->as_string();
        l.max_positions = integer("max_positions");
        l.min_band = number("min_band");
        l.lead_mm = number("lead_mm");
        if (const std::optional<long long> mvpt = integer("move_patterns")) l.move_patterns = static_cast<std::uint32_t>(*mvpt);
        e->axis->set_axis_limits(l);
    });
}

iad_result IAD_CALL iad_axis_set_nv_unit(iad_axis* axis, const char* unit_key, const char* label) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const std::string key = unit_key ? unit_key : "";
        const std::string text =
            label ? label : e->axis->bus()->name() + " slave " + std::to_string(e->axis->slave());
        e->axis->share_nonvolatile_state(key, text);
    });
}

iad_result IAD_CALL iad_nv_forget(const char* unit_key_prefix) {
    return guarded([&] { Axis::forget_nonvolatile_state(unit_key_prefix ? unit_key_prefix : ""); });
}

iad_result IAD_CALL iad_testing_set_timing(const char* name, double seconds) {
    return guarded([&] {
        detail::set_test_timing(name ? name : "", std::isnan(seconds) ? std::nullopt : std::optional<double>(seconds));
    });
}

iad_result IAD_CALL iad_constants_json(char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        Json j = Json::object();
        j.set("SETTLE_S", kSettleSDefault);
        j.set("PUSH_SETTLE_S", kPushSettleSDefault);
        j.set("SERVO_TIMEOUT_S", kServoTimeoutS);
        j.set("SERVO_FIRST_ON_MAX_S", kServoFirstOnMaxS);
        j.set("STORED_START_S", kStoredStartS);
        j.set("PARW_READY_S", kParwReadyS);
        j.set("PARW_CLEAR_S", kParwClearS);
        j.set("PARW_OBSERVE_S", kParwObserveS);
        j.set("RESET_WAIT_S", kResetWaitS);
        j.set("RESET_TIMEOUT_S", kResetTimeoutS);
        j.set("TEACH_WRITE_S", kTeachWriteS);
        j.set("BAUD_SETTLE_S", kBaudSettleS);
        j.set("BREAK_S", kBreakS);
        j.set("FAST_BAUDRATE", kFastBaudRate);
        j.set("FAST_POLL_S", kFastPollS);
        j.set("FRESH_S", kFreshS);
        j.set("STALL_S", kStallS);
        j.set("LEAN_STATUS_WORDS", kLeanStatusWords);
        j.set("FALLBACK_SPEED_CAP", kFallbackSpeedCap);
        j.set("FALLBACK_ACCEL_CAP_G", kFallbackAccelCapG);
        j.set("FALLBACK_PUSH_CAP_PCT", kFallbackPushCapPct);
        j.set("MOVE_BAND_MAX", kMoveBandMax);
        j.set("NV_WRITE_CYCLES", static_cast<long long>(kNvWriteCycles));
        j.set("NV_WARN_PER_HOUR", kNvWarnPerHour);
        j.set("NV_WINDOW_S", kNvWindowS);
        j.set("NV_SKIP_NOTE", kNvUnchangedNote);
        j.set("P1_MIN_MM", kP1MinMm);
        j.set("P2_RANGE_MM", std::vector<double>{kP2MinMm, kP2MaxMm});
        j.set("P4_RANGE_MM", std::vector<double>{kP4MinMm, kP4MaxMm});
        j.set("RECOMMENDED_RETRIES", kRecommendedRetries);
        j.set("DEFAULT_BAUDRATE", kDefaultBaudRate);
        write_json(j, buffer, buffer_size, required);
    });
}

}  // extern "C"
