// The JSON form of every public result type (json.hpp declares the overloads). Field names follow the Python edition's
// to_dict() so the outputs of the editions compare; keys that the C++ edition does not have (the RCON line fields) are
// left out. Values the unit did not give are null. No I/O.
#include "iai_actuator_driver/json.hpp"

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/results.hpp"
#include "iai_actuator_driver/status.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

/// "ec ff 02 00 13" (lower-case, space separated: Python's bytes.hex(" ")).
std::string hex_text(const std::vector<std::uint8_t>& bytes) {
    std::string text;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) text += ' ';
        text += util::format("%02x", bytes[i]);
    }
    return text;
}

Json string_or_null(const std::string& text) { return text.empty() ? Json() : Json(text); }

Json choices_json(const std::map<std::string, std::string>& choices) {
    Json out = Json::object();
    for (const auto& kv : choices) out.set(kv.first, kv.second);
    return out;
}

}  // namespace

Json to_json(const Span& value) {
    Json out = Json::array();
    out.push_back(value.min);
    out.push_back(value.max);
    return out;
}

Json to_json(const Status& st) {
    Json raw = Json::object();
    raw.set("PNOW", st.raw.pnow);
    raw.set("ALMC", st.raw.almc);
    raw.set("DIPM", st.raw.dipm);
    raw.set("DOPM", st.raw.dopm);
    raw.set("DSS1", st.raw.dss1);
    raw.set("DSS2", st.raw.dss2);
    raw.set("DSSE", st.raw.dsse);
    if (st.raw.stat) raw.set("STAT", *st.raw.stat);
    if (st.raw.vnow) raw.set("VNOW", *st.raw.vnow);
    if (st.raw.cnow) raw.set("CNOW", *st.raw.cnow);
    if (st.raw.devi) raw.set("DEVI", *st.raw.devi);
    if (st.raw.stim) raw.set("STIM", *st.raw.stim);
    if (st.raw.sipm) raw.set("SIPM", *st.raw.sipm);
    if (st.raw.zons) raw.set("ZONS", *st.raw.zons);
    if (st.raw.poss) raw.set("POSS", *st.raw.poss);
    if (st.raw.ssse) raw.set("SSSE", *st.raw.ssse);

    Json out = Json::object();
    out.set("position_mm", st.position_mm);
    out.set("alarm_code", st.alarm_code);
    out.set("servo_on", st.servo_on);
    out.set("homed", st.homed);
    out.set("in_position", st.in_position);
    out.set("moving", st.moving);
    out.set("alarm", st.alarm);
    out.set("modbus_enabled", st.modbus_enabled);
    out.set("emergency_stop", st.emergency_stop);
    out.set("velocity_mm_s", st.velocity_mm_s);
    out.set("current_ma", st.current_ma);
    out.set("raw", raw);
    out.set("line", st.line ? Json(*st.line) : Json());
    out.set("warning", st.warning());
    const std::optional<AlarmLevel> level = st.alarm_level();
    out.set("alarm_level", level ? Json(level_key(*level)) : Json());
    out.set("move_bit", st.move_bit());
    out.set("homing", st.homing());
    out.set("paused", st.paused());
    out.set("target_reached", st.target_reached());
    out.set("push_missed", st.push_missed());
    out.set("pushing", st.pushing());
    out.set("push_contact", st.push_contact());
    out.set("safety_speed", st.safety_speed());
    out.set("at_backward_end", st.at_backward_end());
    out.set("at_forward_end", st.at_forward_end());
    out.set("at_middle", st.at_middle());
    out.set("vnow_mm_s", st.vnow_mm_s());
    out.set("idle", st.idle());
    out.set("ready", st.ready());
    out.set("position_complete_no", st.position_complete_no());
    out.set("cold_start_alarm", st.cold_start_alarm());
    out.set("rtc_in_use", st.rtc_in_use());
    out.set("drive_power_cut", st.drive_power_cut());
    out.set("emg_input", st.emg_input());
    out.set("enable_off", st.enable_off());
    out.set("auto_servo_off", st.auto_servo_off());
    out.set("nv_busy", st.nv_busy());
    out.set("manual_mode", st.manual_mode());
    out.set("battery_low", st.battery_low());
    out.set("excitation_done", st.excitation_done());
    return out;
}

Json to_json(const AlarmRecord& r) {
    Json out = Json::object();
    out.set("index", r.index);
    out.set("code", r.code);
    out.set("detail", r.detail);
    out.set("address", r.address);
    out.set("time_raw", r.time_ms);
    out.set("name", r.name());
    out.set("power_up", r.power_up());
    out.set("level", level_key(r.level()));
    out.set("time_ms", r.time_ms);
    out.set("time_after_power_on", r.time_after_power_on());
    return out;
}

Json to_json(const AlarmInfo& a) {
    Json out = Json::object();
    out.set("code", a.code);
    out.set("hex", a.hex);
    out.set("name", a.name);
    out.set("level", level_key(a.level));
    out.set("level_text", a.level_text);
    out.set("group", a.group);
    out.set("group_name", a.group_name);
    out.set("fault", a.fault);
    out.set("warning", a.warning);
    out.set("reset", a.reset);
    out.set("home_needed", a.home_needed);
    out.set("remedy", a.remedy);
    out.set("detail_text", a.detail_text);
    out.set("family", a.family);
    return out;
}

Json to_json(const IndexCategory& c) {
    Json out = Json::object();
    out.set("category_id", c.category_id);
    out.set("model_code", c.model_code);
    out.set("member_count", c.member_count);
    out.set("list_count", c.list_count);
    out.set("data_address", c.data_address);
    out.set("id_list_address", c.id_list_address);
    out.set("coil_base", c.coil_base);
    out.set("member_size", c.member_size);
    return out;
}

Json to_json(const DeviceInfo& d) {
    Json out = Json::object();
    out.set("slave", d.slave);
    out.set("system_code", d.system_code);
    out.set("system_name", d.system_name);
    out.set("run_status", d.run_status);
    out.set("unit_code", d.unit_code);
    out.set("unit_name", d.unit_name);
    out.set("firmware", d.firmware);
    out.set("slave_id_raw", d.slave_id_raw.empty() ? Json() : Json(hex_text(d.slave_id_raw)));
    if (d.index_header) {
        Json header = Json::object();
        header.set("model_code", d.index_header->model_code);
        header.set("words_per_record", d.index_header->words_per_record);
        header.set("record_count", d.index_header->record_count);
        header.set("rx_buffer", d.index_header->rx_buffer);
        header.set("tx_buffer", d.index_header->tx_buffer);
        out.set("index_header", header);
    } else {
        out.set("index_header", Json());
    }
    Json categories = Json::array();
    for (const IndexCategory& c : d.categories) categories.push_back(to_json(c));
    out.set("categories", categories);
    out.set("serial_number", d.serial_number);
    out.set("user_name", d.user_name);
    out.set("model_name", d.model_name);
    out.set("soft_limits_mm", d.soft_limits_mm ? to_json(*d.soft_limits_mm) : Json());
    out.set("max_speed_mm_s", d.max_speed_mm_s);
    out.set("errors", d.errors);
    out.set("identity_source", d.identity_source);
    out.set("unit_known", d.unit_known);
    out.set("actuator_kind", d.actuator_kind);
    out.set("actuator_kind_name", d.actuator_kind_name);
    out.set("rotary_atyp", d.rotary_atyp);
    out.set("axis_number", d.axis_number);
    out.set("actuator_serial", d.actuator_serial);
    out.set("accl", d.accl);
    out.set("max_accel_g", d.max_accel_g);
    out.set("max_decel_g", d.max_decel_g);
    out.set("speed_limit_source", d.speed_limit_source);
    out.set("product_family", d.product_family);
    out.set("controller_name", d.controller_name);
    out.set("units", d.units);
    out.set("max_positions", d.max_positions);
    out.set("model", d.model());
    out.set("controller", d.controller());
    out.set("short_model", d.user_name);
    out.set("family", d.unit_name);
    out.set("actuator", d.actuator());
    out.set("extensibility_level", d.extensibility_level());
    out.set("rotary", d.rotary());
    out.set("kind", d.kind());
    return out;
}

Json to_json(const AxisLimits& l) {
    Json out = Json::object();
    out.set("max_speed_mm_s", l.max_speed_mm_s);
    out.set("max_accel_g", l.max_accel_g);
    out.set("max_decel_g", l.max_decel_g);
    out.set("max_push_speed_mm_s", l.max_push_speed_mm_s);
    out.set("max_push_pct", l.max_push_pct);
    out.set("min_push_pct", l.min_push_pct);
    out.set("min_push_from_ec", l.min_push_from_unit);
    out.set("push_force_factor", l.push_force_factor);
    out.set("push_force_offset", l.push_force_offset);
    out.set("actuator_kind", l.actuator_kind);
    out.set("rotary_atyp", l.rotary_atyp);
    out.set("default_speed_mm_s", l.default_speed_mm_s);
    out.set("supports_push", l.supports_push);
    out.set("supports_relative", l.supports_relative);
    out.set("source", l.source);
    out.set("max_positions", l.max_positions);
    out.set("min_band", l.min_band);
    out.set("lead_mm", l.lead_mm);
    out.set("move_patterns", l.move_patterns);
    out.set("notes", l.notes);
    out.set("push_available", l.push_available());
    out.set("rotary", l.rotary());
    out.set("units", l.units());
    out.set("kind", l.kind());
    return out;
}

Json to_json(const LimitRange& r) {
    // the keys that do not apply to a range are left out
    Json out = Json::object();
    out.set("min", r.min);
    out.set("max", r.max);
    out.set("unit", r.unit);
    out.set("source", r.source);
    if (r.unit_max || r.configured) {
        out.set("unit_max", r.unit_max);
        out.set("configured", r.configured);
    }
    if (!r.recommended_min_source.empty()) {  // speed: the documented minimum and its source
        out.set("recommended_min", r.recommended_min);
        out.set("recommended_min_source", r.recommended_min_source);
    } else if (r.available || r.recommended_min) {
        out.set("available", r.available);
        out.set("recommended_min", r.recommended_min);
    }
    if (r.count) out.set("count", r.count);
    if (!r.speed_unit.empty()) {  // push: the push speed entry (Axis::limits)
        out.set("speed", r.speed);
        out.set("speed_unit", r.speed_unit);
        out.set("speed_source", r.speed_source);
    }
    return out;
}

/// A range whose limits are ints in the Python edition (the stored rows, the speed override): a whole value is written
/// as an integer, not 100.0.
static Json whole_range_json(const LimitRange& r) {
    Json out = to_json(r);
    for (const char* key : {"min", "max"}) {
        const Json* v = out.find(key);
        const double value = v && v->type() == Json::Type::Number ? v->as_number() : 0.0;
        if (v && v->type() == Json::Type::Number) out.set(key, Json::whole_or_number(value, false));
    }
    return out;
}

Json to_json(const AxisRanges& r) {
    Json out = Json::object();
    out.set("units", r.units);
    out.set("kind", r.kind);
    out.set("family", r.family);
    out.set("position", to_json(r.position));
    out.set("speed", to_json(r.speed));
    out.set("accel", to_json(r.accel));
    out.set("decel", to_json(r.decel));
    out.set("accel_decel", to_json(r.accel_decel));
    out.set("push", to_json(r.push));
    out.set("band", to_json(r.band));
    out.set("push_band", to_json(r.push_band));
    out.set("stored_rows", whole_range_json(r.stored_rows));
    out.set("jog_step", to_json(r.jog_step));
    out.set("speed_override", whole_range_json(r.speed_override));
    return out;
}

Json to_json(const PositionEntry& e) {
    Json out = Json::object();
    out.set("position_mm", e.position_mm);
    out.set("band_mm", e.band_mm);
    out.set("speed_mm_s", e.speed_mm_s);
    out.set("zone_plus_mm", e.zone_plus_mm);
    out.set("zone_minus_mm", e.zone_minus_mm);
    out.set("accel_g", e.accel_g);
    out.set("decel_g", e.decel_g);
    out.set("push_power", e.push_power);
    out.set("load_threshold", e.load_threshold);
    out.set("control_flags", e.control_flags);
    out.set("checksum", e.checksum);
    std::optional<bool> ok;
    try {
        ok = e.checksum_ok();
    } catch (const std::exception&) {
        ok = std::nullopt;  // a value that does not encode has no checksum to compare
    }
    out.set("checksum_ok", ok);
    return out;
}

Json to_json(const ParamValue& v) {
    if (v.number) return Json::whole_or_number(*v.number, !v.whole);
    if (!v.text.empty()) return Json(v.text);
    return Json();
}

Json to_json(const ParameterInfo& p) {
    Json out = Json::object();
    out.set("number", p.number != 0 ? std::optional<int>(p.number) : std::nullopt);  // null for a raw item
    out.set("name", p.name);
    out.set("symbol", p.symbol);
    out.set("item", p.item);
    out.set("address", p.address);
    out.set("value", to_json(p.value));
    out.set("text", p.text);
    out.set("unit", p.unit);
    out.set("min", p.min ? Json::whole_or_number(*p.min, p.min_decimal) : Json());
    out.set("max", p.max ? Json::whole_or_number(*p.max, p.max_decimal) : Json());
    out.set("choices", choices_json(p.choices));
    out.set("writable", p.writable);
    out.set("dangerous", p.dangerous);
    out.set("raw", p.raw);
    out.set("family", p.family);
    out.set("unsigned", p.unsigned_value);
    out.set("signed", p.signed_value);
    out.set("words", p.words);
    out.set("named", p.named);
    out.set("note", p.note);
    return out;
}

Json to_json(const ParameterModel& m) {
    Json out = Json::object();
    out.set("unit_code", m.unit_code);
    out.set("system_code", m.system_code);
    out.set("model_name", m.model_name);
    out.set("family", m.family);
    out.set("seen_on", m.seen_on);
    out.set("known", m.known);
    out.set("candidates", m.candidates);
    out.set("listed", m.listed);
    Json hidden = Json::array();
    for (const auto& h : m.hidden) {
        Json row = Json::object();
        row.set("no", h.first);
        row.set("reason", h.second);
        hidden.push_back(row);
    }
    out.set("hidden", hidden);
    out.set("verified", m.verified);
    out.set("note", m.note);
    return out;
}

Json to_json(const ParameterWriteResult& r) {
    Json out = Json::object();
    out.set("no", r.number);
    out.set("name", r.name);
    out.set("key", r.key);
    out.set("address", r.address);
    out.set("unit", r.unit);
    out.set("before", r.before);
    out.set("after", r.after);
    out.set("value", to_json(r.value));
    out.set("raw_before", r.raw_before);
    out.set("raw_after", r.raw_after);
    out.set("changed", r.changed);
    out.set("verified", r.verified);
    out.set("committed", r.committed);
    out.set("listed_for_model", r.listed_for_model);
    out.set("verified_for_model", r.verified_for_model);
    out.set("home_parameter", r.home_parameter);
    out.set("warnings", r.warnings);
    out.set("note", r.note);
    out.set("commit_confirmed", r.commit_confirmed);
    out.set("skipped", r.skipped);
    out.set("forced", r.forced);
    out.set("saved", r.saved());
    out.set("needs_reset", r.needs_reset());
    out.set("takes_effect", r.takes_effect());
    return out;
}

Json to_json(const PositionWriteResult& r) {
    Json out = Json::object();
    out.set("row", r.row);
    out.set("entry", to_json(r.entry));
    out.set("skipped", r.skipped);
    out.set("forced", r.forced);
    out.set("note", r.note);
    out.set("warnings", r.warnings);
    return out;
}

Json to_json(const SoftwareResetResult& r) {
    Json out = Json::object();
    out.set("accepted", r.accepted);
    out.set("reply_lost", r.reply_lost);
    out.set("baudrate", r.baud_rate);
    out.set("baud_changed", r.baud_changed);
    out.set("modbus_enabled", r.modbus_enabled());
    out.set("safety_speed", r.safety_speed());
    out.set("servo_on", r.servo_on());
    out.set("homed", r.homed());
    out.set("needs_home", r.needs_home());
    out.set("alarm_code", r.alarm_code());
    out.set("uptime_before_s", r.uptime_before_s);
    out.set("uptime_after_s", r.uptime_after_s);
    out.set("elapsed_s", r.elapsed_s);
    out.set("notes", r.notes);
    out.set("confirmed", r.confirmed);
    out.set("confirmed_by", r.confirmed_by);
    out.set("status", to_json(r.status));
    return out;
}

Json to_json(const BaudChangeResult& r) {
    Json out = Json::object();
    out.set("previous", r.previous);
    out.set("baudrate", r.baud_rate);
    out.set("method", iai_actuator_driver::to_string(r.method));
    out.set("code", r.code);
    out.set("frame", r.frame);
    out.set("verified", r.verified);
    out.set("probed", r.probed);
    out.set("note", r.note);
    out.set("code_verified", r.code_verified);
    return out;
}

Json to_json(const PortBaudResult& r) {
    Json answered = Json::object();
    for (const auto& kv : r.answered) answered.set(std::to_string(kv.first), kv.second);
    Json out = Json::object();
    out.set("port", r.port);
    out.set("rate", r.rate);
    out.set("change", to_json(r.change));
    out.set("answered", answered);
    return out;
}

Json to_json(const ScanHit& h) {
    const std::string system_name = reg::system_name(h.system_code);
    const std::optional<std::string> family = reg::ec_family(h.system_code, h.unit_code);
    Json out = Json::object();
    out.set("name", h.name());
    out.set("port", h.port);
    out.set("slave", h.slave);
    out.set("axis_number", h.axis_number());
    out.set("slave_id_raw", h.slave_id_raw.empty() ? Json() : Json(hex_text(h.slave_id_raw)));
    out.set("system_code", h.system_code);
    out.set("system_name", string_or_null(system_name));
    out.set("unit_code", h.unit_code);
    out.set("firmware", string_or_null(h.firmware));
    out.set("product_family", h.system_code ? Json(reg::product_family(h.system_code))
                                            : h.line == "rcon" ? Json("RCON") : Json());  // a unit behind RCON
    out.set("family", family);
    out.set("error", string_or_null(h.error));
    out.set("line", h.line);
    out.set("serial_code", h.serial_code ? Json(*h.serial_code) : Json());
    return out;
}

Json to_json(const SerialPortInfo& p) {
    Json out = Json::object();
    out.set("port", p.port);
    out.set("description", p.description);
    out.set("manufacturer", p.manufacturer);
    out.set("hardware_id", p.hardware_id);
    out.set("vid", p.vid);
    out.set("pid", p.pid);
    out.set("is_iai", p.is_iai());
    return out;
}

}  // namespace iai_actuator_driver
