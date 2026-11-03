// Customer compatibility self-test (check.hpp). Port of the Python edition's check.py: the same checks in the same
// order, the same results and texts (callers and the customer reports match on them) and the same report fields.
// Read-only unless motion is requested. UNVERIFIED on hardware (the Python edition's behaviour is VERIFIED there).
#include "iai_actuator_driver/check.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <ios>
#include <stdexcept>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/version.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr double kStrokeToleranceMm = 0.005;

const char* py_bool(bool value) { return value ? "True" : "False"; }

/// Python's f"{code:#05x}": "0x0e0".
std::string hex3(int code) { return util::format("0x%03x", code); }

/// Python's f"{value:#06x}": "0x0101".
std::string hex4(int value) { return util::format("0x%04x", value); }

/// An acceleration limit for a check line: "0.30", or "n/a" when the unit did not give it.
std::string fmt_g(std::optional<double> value) { return value ? util::fixed(*value, 2) : std::string("n/a"); }

std::string span_text(const Span& s) { return util::fixed(s.min, 2) + ".." + util::fixed(s.max, 2); }

Json span_json(const std::optional<Span>& s) { return s ? to_json(*s) : Json(); }

/// "<Type>: <message>" of a driver error (the Python edition's f"{type(exc).__name__}: {exc}").
std::string describe(const std::exception& e) { return info::error_name(e) + ": " + e.what(); }

/// The axis's position unit (mm, or deg on a rotary axis); mm when it cannot be read.
std::string units_of(Axis& axis) {
    try {
        return axis.units();
    } catch (const std::exception&) {
        return "mm";
    }
}

Json position_entry_json(const PositionEntry& e) {
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
    out.set("skipped", false);
    return out;
}

/// Merge the keys of `extra` into the object `data` (Python's **data).
Json merged(Json data, const Json& extra) {
    if (extra.type() == Json::Type::Object)
        for (const auto& kv : extra.as_object()) data.set(kv.first, kv.second);
    return data;
}

class Checker {
public:
    Checker(Axis& axis, const CheckOptions& options, CheckReport& report)
        : axis_(axis), options_(options), report_(report) {}

    CheckItem& add(const std::string& name, const char* result, const std::string& detail, Json data = Json::object()) {
        CheckItem item;
        item.name = name;
        item.result = result;
        item.detail = detail;
        item.data = data.is_null() ? Json::object() : std::move(data);
        report_.items.push_back(std::move(item));
        if (options_.on_item) options_.on_item(report_.items.back());
        return report_.items.back();
    }

    void run() {
        // 1. link
        try {
            const std::vector<std::uint16_t> words = axis_.read_registers(reg::kPnow, 2);
            add("link", kCheckPass,
                util::format("answered FC03 (PNOW = %.2f mm)", reg::from_i32_words(words[0], words[1]) / 100.0));
        } catch (const ModbusTimeout& e) {
            add("link", kCheckFail,
                std::string("no answer (") + e.what() +
                    "); check port, baud rate, slave address, and that IA-OS is closed");
            skip_rest("no link");
            return;
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("link", kCheckFail, e.what());
            skip_rest("no link");
            return;
        }

        // 2-5. identity, index table, identification, soft limits, speed limit
        const std::optional<DeviceInfo> dev = check_identity();
        check_index(dev);
        check_identification(dev);
        check_soft_limits();
        check_speed_limit();

        // 6-12. status, alarm, history, tables, control
        std::optional<Status> st;
        try {
            st = axis_.status();
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("status", kCheckFail, describe(e));
        }
        if (st) {
            const std::string u = units_of(axis_);
            if (st->emergency_stop) {
                add("status", kCheckFail, "emergency stop is active", to_json(*st));
            } else {
                add("status", kCheckPass,
                    util::format("pos %.2f %s, servo %s, homed %s, in position %s, moving %s, paused %s",
                                 st->position_mm, u.c_str(), st->servo_on ? "on" : "off", py_bool(st->homed),
                                 py_bool(st->in_position), py_bool(st->moving), py_bool(st->paused())),
                    to_json(*st));
            }
            Json code = Json::object();
            code.set("alarm_code", st->alarm_code);
            if (st->alarm) {
                add("alarm", kCheckFail, "alarm " + hex3(st->alarm_code) + " active", code);
            } else if (st->warning() || st->alarm_code) {
                add("alarm", kCheckWarn, "warning " + hex3(st->alarm_code), code);
            } else {
                Json none = Json::object();
                none.set("alarm_code", 0);
                add("alarm", kCheckPass, "no alarm", none);
            }
        }
        check_alarm_history();
        check_position_table();
        std::optional<std::vector<std::uint16_t>> block;
        try {
            block = axis_.read_registers(reg::kDirectMoveBase, reg::kDirectMoveWords);
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("direct_move_block", kCheckFail, describe(e));
        }
        if (block && block->size() >= 9) {
            const auto& b = *block;
            Json data = Json::object();
            data.set("words", b);
            add("direct_move_block", kCheckPass,
                util::format("0x9900 readable: PCMD %.2f mm, INP %.2f mm, VCMD %.2f mm/s, ACMD %.2f G, PPOW %d, "
                             "CTLF %s",
                             reg::from_i32_words(b[0], b[1]) / 100.0, reg::from_u32_words(b[2], b[3]) / 100.0,
                             reg::from_u32_words(b[4], b[5]) / 100.0, b[6] / 100.0, static_cast<int>(b[7]),
                             util::format("0x%04x", b[8]).c_str()),
                data);
        }
        if (st) {
            if (st->modbus_enabled)
                add("modbus_control", kCheckPass, "DSSE.PMSS = 1 (Modbus has command authority)");
            else
                add("modbus_control", kCheckPass,
                    "DSSE.PMSS = 0 (as after power-up); the driver takes control (PMSL) at rest before the first "
                    "command. STOP works either way");
            Json data = Json::object();
            data.set("safety_speed", st->safety_speed());
            if (st->safety_speed())
                add("safety_speed", kCheckWarn,
                    "DSS1 bit 14 (SFTY) is ON: safety-speed mode set on the EC (e.g. by IA-OS Teach mode). The driver "
                    "does not change it; use set_safety_speed() only if you mean to",
                    data);
            else
                add("safety_speed", kCheckPass, "safety speed off", data);
        }

        // 13. everything the unit tells about itself (read-only; never fails the check)
        check_unit_info();
        check_parameter_ranges();

        // 14-15. optional motion
        if (!options_.motion) {
            add("motion", kCheckSkip, "not requested (run with --motion for a short move of up to 5 mm / deg)");
        } else if (!st || !report_.passed()) {
            add("motion", kCheckSkip, "skipped because a read-only check failed");
        } else if (!effective_stroke()) {
            add("motion", kCheckSkip, "stroke unknown (EC soft limits not readable): pass --stroke MIN MAX");
        } else {
            check_motion(*st);
        }
    }

private:
    void skip_rest(const std::string& why) {
        for (const char* name : {"slave_id", "index_table", "identification", "soft_limits", "speed_limit", "status",
                                 "alarm", "alarm_history", "position_table", "direct_move_block", "unit_info",
                                 "parameter_ranges", "motion"})
            add(name, kCheckSkip, why);
    }

    std::optional<Span> effective_stroke() {
        try {
            return axis_.effective_stroke();
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError&) {
            return std::nullopt;
        }
    }

    std::optional<DeviceInfo> check_identity() {
        std::optional<DeviceInfo> dev;
        try {
            dev = axis_.identify();
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("slave_id", kCheckFail, describe(e));
            return std::nullopt;
        }
        report_.model = dev->model();
        report_.controller = dev->controller();
        report_.system_code = dev->system_code;
        report_.unit_code = dev->unit_code;
        report_.firmware = dev->firmware;
        report_.model_name = dev->model_name;
        report_.serial_number = dev->serial_number;
        Json raw;
        if (!dev->slave_id_raw.empty()) {
            std::string text;
            for (std::size_t i = 0; i < dev->slave_id_raw.size(); ++i)
                text += (i ? " " : "") + util::format("%02x", dev->slave_id_raw[i]);
            raw = Json(text);
        }
        const bool known_system = dev->system_code && !reg::system_name(dev->system_code).empty();
        if (!dev->system_code) {
            // Not answered or rejected: some firmware may not support FC11. The index table decides.
            Json data = Json::object();
            data.set("errors", dev->errors);
            add("slave_id", kCheckWarn, "Report Slave ID (FC11) not answered; model not read", data);
        } else if (dev->identity_source.value_or("") != "FC11" && known_system) {
            Json data = Json::object();
            data.set("system_code", dev->system_code);
            data.set("unit_code", dev->unit_code);
            data.set("firmware", dev->firmware);
            data.set("errors", dev->errors);
            add("slave_id", kCheckWarn,
                "Report Slave ID (FC11) not answered; identity read from " + dev->identity_source.value_or("?") + ": " +
                    dev->controller() + ", firmware " + dev->firmware.value_or("None"),
                data);
        } else {
            Json data = Json::object();
            data.set("system_code", dev->system_code);
            data.set("unit_code", dev->unit_code);
            data.set("firmware", dev->firmware);
            data.set("actuator_kind", dev->actuator_kind);
            data.set("axis_number", dev->axis_number);
            data.set("raw", raw);
            const std::optional<std::string> actuator = dev->actuator();
            add("slave_id", kCheckPass,
                dev->controller() + ", firmware " + dev->firmware.value_or("None") +
                    (actuator ? "; " + *actuator : std::string()),
                data);
        }
        return dev;
    }

    void check_index(const std::optional<DeviceInfo>& dev) {
        if (!dev || !dev->index_header || dev->categories.empty()) {
            Json data = Json::object();
            data.set("errors", dev ? dev->errors : std::vector<std::string>());
            add("index_table", kCheckFail, "index table at 0x0000 not readable", data);
            return;
        }
        struct Expected {
            int category;
            int base;
        };
        static const Expected kExpected[] = {
            {reg::kIoControlCategory, 0x0D00}, {reg::kPositionTableCategory, 0x1000}, {reg::kMonitorCategory, 0x9000},
            {reg::kCommand1Category, 0x9800},  {reg::kCommand2Category, 0x9900},
        };
        std::vector<std::string> problems;
        for (const Expected& e : kExpected) {
            const IndexCategory* cat = dev->category(e.category);
            if (cat == nullptr)
                problems.push_back("category " + hex4(e.category) + " missing");
            else if (cat->data_address != e.base)
                problems.push_back("category " + hex4(e.category) + " at " + hex4(cat->data_address) +
                                   ", driver expects " + hex4(e.base));
        }
        const IndexCategory* io = dev->category(reg::kIoControlCategory);
        if (io != nullptr && io->coil_base != reg::kExpectedCoilBase)
            problems.push_back("coil base " + hex4(io->coil_base) + ", driver expects " + hex4(reg::kExpectedCoilBase));
        const IndexHeader& h = *dev->index_header;
        Json header = Json::object();
        header.set("model_code", h.model_code);
        header.set("words_per_record", h.words_per_record);
        header.set("record_count", h.record_count);
        header.set("rx_buffer", h.rx_buffer);
        header.set("tx_buffer", h.tx_buffer);
        Json data = Json::object();
        data.set("header", header);
        data.set("categories", static_cast<int>(dev->categories.size()));
        if (!problems.empty())
            add("index_table", kCheckFail, "register layout differs from the driver's: " + util::join(problems, "; "),
                data);
        else
            add("index_table", kCheckPass,
                std::to_string(dev->categories.size()) + " categories; layout matches the driver", data);
    }

    void check_identification(const std::optional<DeviceInfo>& dev) {
        if (!dev) {
            add("identification", kCheckSkip, "identify() failed");
        } else if (dev->model_name || dev->serial_number) {
            Json data = Json::object();
            data.set("model_name", dev->model_name);
            data.set("short_model", dev->user_name);
            data.set("serial_number", dev->serial_number);
            add("identification", kCheckPass,
                "model " + dev->model_name.value_or("?") + ", serial " + dev->serial_number.value_or("?"), data);
        } else {
            Json data = Json::object();
            data.set("errors", dev->errors);
            add("identification", kCheckWarn, "model / serial strings at 0x0600 not available", data);
        }
    }

    void check_soft_limits() {
        std::optional<Span> limits;
        std::string note;
        try {
            const SoftLimits s = axis_.read_soft_limits();
            limits = s.limits;
            note = s.note;
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            note = describe(e);
        }
        report_.ec_soft_limits_mm = limits;
        const std::optional<Span> eff = effective_stroke();
        report_.stroke = eff;
        const std::optional<Span> cfg = axis_.options().stroke;
        const std::optional<Span> unit = unit_stroke(limits);
        Json data = Json::object();
        data.set("configured_stroke", span_json(cfg));
        data.set("ec_soft_limits_mm", span_json(limits));
        data.set("ec_stroke", span_json(unit));
        data.set("effective_stroke", span_json(eff));
        data.set("note", note);
        const std::string u = units_of(axis_);
        if (!limits || !unit) {
            if (!cfg)
                add("soft_limits", kCheckWarn,
                    "EC soft limits not available (" + note +
                        ") and no --stroke given: motion commands will be refused until a stroke is configured",
                    data);
            else
                add("soft_limits", kCheckWarn,
                    "EC soft limits not available (" + note + "); using the configured stroke " + span_text(*cfg) +
                        " " + u + " unchecked",
                    data);
            return;
        }
        const std::string text = "soft limits " + span_text(*limits) + " " + u + " (stroke " + span_text(*unit) + " " +
                                 u + ")";
        if (!cfg) {
            add("soft_limits", kCheckPass, text + "; used as the stroke", data);
        } else if (!eff) {
            add("soft_limits", kCheckFail,
                text + "; the configured stroke " + span_text(*cfg) + " " + u +
                    " does not overlap it: wrong EC or wrong --stroke",
                data);
        } else if (cfg->min < unit->min - kStrokeToleranceMm || cfg->max > unit->max + kStrokeToleranceMm) {
            add("soft_limits", kCheckWarn,
                text + "; the configured stroke " + span_text(*cfg) + " " + u + " exceeds it, the driver narrows it to " +
                    span_text(*eff) + " " + u + ". Check --stroke for this EC",
                data);
        } else {
            add("soft_limits", kCheckPass, text + "; configured stroke " + span_text(*cfg) + " " + u + " lies inside",
                data);
        }
    }

    void check_speed_limit() {
        std::optional<double> speed;
        std::string note;
        std::optional<AxisLimits> limits;
        try {
            limits = axis_.read_axis_limits();
            const auto cap = limits->unit_speed_cap();
            speed = cap.first;
            note = cap.second;
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            note = describe(e);
        }
        std::optional<double> speed_cap, accel_cap, push_cap;
        try {
            const AxisRanges ranges = axis_.limits();
            speed_cap = ranges.speed.max;
            accel_cap = ranges.accel_decel.max;
            push_cap = ranges.push.max;
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError&) {
        }
        report_.ec_max_speed_mm_s = speed;
        report_.speed_cap_mm_s = speed_cap;
        Json data = Json::object();
        data.set("ec_max_speed_mm_s", speed);
        data.set("configured_max_speed_mm_s", axis_.options().max_speed_mm_s);
        data.set("speed_cap_mm_s", speed_cap);
        data.set("note", note);
        const std::string u = units_of(axis_);
        if (!speed) {
            add("speed_limit", kCheckWarn,
                "unit maximum speed (MAXV item 7.4) not available (" + note + "); speeds capped at " +
                    (speed_cap ? util::format("%g", *speed_cap) : std::string("None")) + " " + u + "/s only",
                data);
            return;
        }
        std::string accel;
        if (limits && (limits->max_accel_g || limits->max_decel_g))
            accel = ", accel / decel max " + fmt_g(limits->max_accel_g) + " / " + fmt_g(limits->max_decel_g) + " G";
        Json first = Json::object();
        first.set("accel_cap_g", accel_cap);
        first.set("push_cap_pct", push_cap);
        add("speed_limit", kCheckPass,
            "unit maximum speed " + util::fixed(*speed, 2) + " " + u + "/s (" + note + ")" + accel +
                "; the driver refuses speeds above " + util::fixed(speed_cap.value_or(0.0), 2) + " " + u +
                "/s and accelerations above " + util::fixed(accel_cap.value_or(0.0), 2) + " G",
            merged(first, data));
    }

    void check_alarm_history() {
        std::vector<AlarmRecord> records;
        try {
            records = axis_.read_alarm_history();
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("alarm_history", kCheckWarn, "not readable: " + describe(e));
            return;
        }
        int alarms = 0;
        for (const AlarmRecord& r : records)
            if (!r.power_up()) ++alarms;
        std::string detail = std::to_string(records.size()) + " entries, " + std::to_string(alarms) + " alarms";
        if (!records.empty()) detail += "; newest (#0): " + records.front().name();
        Json entries = Json::array();
        for (const AlarmRecord& r : records) entries.push_back(to_json(r));
        Json data = Json::object();
        data.set("entries", entries);
        add("alarm_history", kCheckPass, detail, data);
    }

    void check_position_table() {
        std::vector<PositionEntry> entries;
        try {
            for (int n = 0; n < reg::kPositionActiveRows; ++n) entries.push_back(axis_.read_position_table(n));
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("position_table", kCheckFail, describe(e));
            return;
        } catch (const std::out_of_range& e) {
            add("position_table", kCheckFail, describe(e));
            return;
        }
        const std::string u = units_of(axis_);
        std::vector<std::string> parts;
        Json list = Json::array();
        for (std::size_t n = 0; n < entries.size(); ++n) {
            parts.push_back("#" + std::to_string(n) + " " + util::fixed(entries[n].position_mm, 2) + " " + u + " @ " +
                            util::fixed(entries[n].speed_mm_s, 2) + " " + u + "/s");
            list.push_back(position_entry_json(entries[n]));
        }
        const std::string text = util::join(parts, ", ");
        Json data = Json::object();
        data.set("entries", list);
        const std::optional<Span> eff = effective_stroke();
        if (!eff) {
            add("position_table", kCheckWarn, text + " (stroke unknown: not checked)", data);
            return;
        }
        bool inside = true;
        for (const PositionEntry& e : entries)
            if (e.position_mm < eff->min || e.position_mm > eff->max) inside = false;
        if (inside)
            add("position_table", kCheckPass, text, data);
        else
            add("position_table", kCheckWarn, text + " (outside the stroke " + span_text(*eff) + " " + u + ")", data);
    }

    void check_unit_info() {
        Json data;
        try {
            data = info::unit_info(axis_);
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("unit_info", kCheckWarn, "not read: " + describe(e));
            return;
        } catch (const std::out_of_range& e) {
            add("unit_info", kCheckWarn, "not read: " + describe(e));
            return;
        } catch (const std::invalid_argument& e) {
            add("unit_info", kCheckWarn, "not read: " + describe(e));
            return;
        }
        report_.unit_info = data;
        std::vector<std::string> gaps;
        if (const Json* errors = data.find("errors"))
            for (const Json& e : errors->as_array()) gaps.push_back(e.as_string());
        if (!gaps.empty()) {
            std::vector<std::string> first(gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(3, gaps.size())));
            Json extra = Json::object();
            extra.set("errors", gaps);
            add("unit_info", kCheckWarn,
                "read with " + std::to_string(gaps.size()) + " gap(s): " + util::join(first, "; "), extra);
        } else {
            add("unit_info", kCheckPass,
                "identity, parameters, I/O, status words, alarms, maintenance and positions read (report field "
                "unit_info)");
        }
    }

    /// The parameter values the unit holds against the documented ranges (no I/O: from unit_info; WARN at most).
    void check_parameter_ranges() {
        const Json* params = report_.unit_info.type() == Json::Type::Object ? report_.unit_info.find("parameters")
                                                                             : nullptr;
        if (params == nullptr || params->type() != Json::Type::Object || params->size() == 0) {
            add("parameter_ranges", kCheckSkip, "parameters not read");
            return;
        }
        const Json* check = params->find("range_check");
        if (check != nullptr && check->type() == Json::Type::Object) {
            const Json* findings = check->find("findings");
            const Json* family_json = check->find("family");
            const bool has_family = family_json && family_json->type() == Json::Type::String;
            const std::string family = has_family ? family_json->as_string() : std::string("PCON-CB");
            const std::string fallback =
                has_family ? std::string() : std::string(" (the family has no range table of its own in the driver)");
            std::string band_text;
            const Json* band = check->find("min_band");
            if (band != nullptr && band->type() == Json::Type::Object)
                band_text = "; minimum positioning band " + util::fixed(band->find("mm")->as_number(), 2) + " mm (" +
                            band->find("formula")->as_string() + ")";
            const std::size_t count = findings ? findings->size() : 0;
            if (count) {
                std::vector<std::string> problems;
                for (std::size_t i = 0; i < count && i < 3; ++i)
                    problems.push_back(findings->as_array()[i].find("problem")->as_string());
                add("parameter_ranges", kCheckWarn,
                    std::to_string(count) + " value(s) outside the documented " + family + " ranges" + fallback +
                        ": " + util::join(problems, "; ") + band_text,
                    *check);
            } else {
                add("parameter_ranges", kCheckPass,
                    "every parameter value inside the documented " + family + " ranges" + fallback + band_text,
                    *check);
            }
            return;
        }
        const Json* p4 = nullptr;
        if (const Json* named = params->find("named"))
            for (const Json& p : named->as_array()) {
                const Json* no = p.find("no");
                if (no && no->type() == Json::Type::Int && no->as_int() == 4) p4 = &p;
            }
        const Json* found = p4 ? p4->find("found") : nullptr;
        if (found && found->type() == Json::Type::Bool && found->as_bool()) {
            const Json* text = p4->find("text");
            Json data = Json::object();
            data.set("p4_value", *p4->find("value"));
            add("parameter_ranges", kCheckPass,
                "EC user parameters read (categories per ME3816-6B p.6-6); P4 home position adjustment is " +
                    (text && text->type() == Json::Type::String ? text->as_string() : std::string("None")) +
                    ": keep this report, ME3816-6B p.6-10 forbids a P4 below the value at shipping",
                data);
        } else {
            add("parameter_ranges", kCheckPass, "EC user parameters read (categories per ME3816-6B p.6-6)");
        }
    }

    void check_motion(const Status& st) {
        if (st.alarm || !st.homed) {
            add("motion", kCheckFail, st.alarm ? "axis has an alarm" : "axis is not homed; run 'home' first");
            return;
        }
        if (st.paused()) {
            add("motion", kCheckFail, st.modbus_enabled ? "axis is paused (STP on); resume it first"
                                                        : "unit is in PIO mode (PMSS 0): the PIO holds the pause (STP on); "
                                                          "take PC control or release *STP on the PIO connector first");
            return;
        }
        const Span eff = *effective_stroke();
        const double lo = eff.min, hi = eff.max;
        const std::string u = units_of(axis_);
        const double start = st.position_mm;
        // The step adapts to the unit: at most a quarter of its stroke (a short gripper or a 4 mm unit too), at least
        // 0.1, toward the side with more room.
        const double step = util::round2(std::max(0.1, std::min(options_.step_mm, 0.25 * (hi - lo))));
        const double target = hi - start >= start - lo ? start + step : start - step;
        if (!(lo <= target && target <= hi) || !(lo <= start && start <= hi)) {
            add("motion", kCheckFail,
                "no room for a " + util::format("%g", step) + " " + u + " move inside " + util::fixed(lo, 2) + ".." +
                    util::fixed(hi, 2) + " " + u + " from " + util::fixed(start, 2) + " " + u);
            return;
        }
        bool enabled_here = false, servo_here = false;
        try {
            if (!st.modbus_enabled) {
                axis_.enable_modbus_control();
                enabled_here = true;
            }
            if (!st.servo_on) {
                axis_.servo(true);
                servo_here = true;
            }
            const std::pair<const char*, double> legs[] = {{"motion_out", target}, {"motion_back", start}};
            for (const auto& leg : legs) {
                const double goal = leg.second;
                const double begin = axis_.status().position_mm;
                // move_to returns only once PNOW has stopped changing (PEND / MEND rise inside the band before
                // standstill on the EC), so the reached position is the settled one.
                MoveOptions move;
                move.speed = options_.speed_mm_s;
                move.accel_g = options_.accel_g;
                move.band = 0.1;
                const std::optional<Status> reached = axis_.move_to(goal, move);
                if (!reached) throw EcNotReadyError("move_to returned no status");
                const double settled = util::round2(reached->position_mm);
                Json data = Json::object();
                data.set("target_mm", goal);
                data.set("reached_mm", settled);
                data.set("error_mm", util::round2(std::fabs(settled - goal)));
                data.set("settled", true);
                data.set("settle_s", axis_.options().settle_s);
                data.set("target_reached", reached->target_reached());
                add(leg.first, kCheckPass,
                    util::fixed(begin, 2) + " -> " + util::fixed(settled, 2) + " " + u + " settled (target " +
                        util::fixed(goal, 2) + ", off by " + util::fixed(std::fabs(settled - goal), 2) + " " + u +
                        ") at " + util::format("%g", options_.speed_mm_s) + " " + u + "/s",
                    data);
            }
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            const std::string notes = util::join(e.notes(), "; ");
            add("motion", kCheckFail,
                info::error_name(e) + ": " + (notes.empty() ? std::string(e.what()) : e.message() + " (" + notes + ")"));
        }
        try {
            if (servo_here) axis_.servo(false);
            if (enabled_here) axis_.disable_modbus_control();
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            add("motion_restore", kCheckWarn, std::string("could not restore servo/PMSL state: ") + e.what());
        }
    }

    Axis& axis_;
    const CheckOptions& options_;
    CheckReport& report_;
};

}  // namespace

// ============================================================================================ CheckReport

bool CheckReport::passed() const {
    for (const CheckItem& item : items)
        if (item.result == kCheckFail) return false;
    return true;
}

const CheckItem* CheckReport::item(const std::string& name) const {
    const CheckItem* found = nullptr;
    for (const CheckItem& i : items)
        if (i.name == name) found = &i;
    return found;
}

void CheckReport::write_json(const std::string& path) const {
    std::ofstream file(std::filesystem::path(path), std::ios::binary | std::ios::trunc);
    if (!file) throw std::ios_base::failure("could not open '" + path + "' for writing");
    file << to_json(*this).dump(2);
    file.flush();
    if (!file) throw std::ios_base::failure("could not write '" + path + "'");
}

Json to_json(const CheckItem& item) {
    Json out = Json::object();
    out.set("name", item.name);
    out.set("result", item.result);
    out.set("detail", item.detail);
    out.set("data", item.data);
    return out;
}

Json to_json(const CheckReport& r) {
    Json out = Json::object();
    out.set("timestamp", r.timestamp);
    out.set("port", r.port);
    out.set("slave", r.slave);
    out.set("strategy", r.strategy);
    out.set("driver_version", r.driver_version);
    out.set("stroke", span_json(r.stroke));
    out.set("motion_requested", r.motion_requested);
    out.set("configured_stroke", span_json(r.configured_stroke));
    out.set("ec_soft_limits_mm", span_json(r.ec_soft_limits_mm));
    out.set("ec_max_speed_mm_s", r.ec_max_speed_mm_s);
    out.set("speed_cap_mm_s", r.speed_cap_mm_s);
    out.set("baud", r.baud);
    out.set("model", r.model);
    out.set("controller", r.controller);
    out.set("model_name", r.model_name);
    out.set("serial_number", r.serial_number);
    out.set("system_code", r.system_code);
    out.set("unit_code", r.unit_code);
    out.set("firmware", r.firmware);
    Json items = Json::array();
    for (const CheckItem& item : r.items) items.push_back(to_json(item));
    out.set("items", items);
    out.set("unit_info", r.unit_info);
    out.set("result", r.passed() ? kCheckPass : kCheckFail);
    out.set("tool", kCheckTool);
    return out;
}

// ============================================================================================ run_check

CheckReport run_check(Axis& axis, const CheckOptions& options) {
    CheckReport report;
    report.timestamp = info::local_timestamp();
    report.port = options.port_label;
    report.slave = axis.slave();
    report.strategy = to_string(axis.options().method);
    report.driver_version = kVersion;
    report.configured_stroke = axis.options().stroke;
    report.motion_requested = options.motion;
    report.baud = axis.baud_rate();
    // the configured stroke until the soft-limit check reads the unit's (no I/O before the link check)
    report.stroke = effective_stroke(report.configured_stroke, std::nullopt);
    Checker(axis, options, report).run();
    return report;
}

std::string format_check_item(const CheckItem& item) {
    return util::format("%-4s  %-18s %s", item.result.c_str(), item.name.c_str(), item.detail.c_str());
}

std::string save_check_report(const CheckReport& report, const std::string& path, std::ostream& out,
                              std::ostream& err) {
    try {
        report.write_json(path);
        return "report written to " + path;
    } catch (const std::exception& e) {
        err << "warning: could not write the report to " << path << ": " << e.what() << '\n';
    }
    std::string fallback;
    try {
        fallback = (std::filesystem::temp_directory_path() / std::filesystem::path(path).filename()).string();
        report.write_json(fallback);
        return "report written to " + fallback + " (fallback location)";
    } catch (const std::exception& e) {
        err << "warning: could not write the report to " << (fallback.empty() ? std::string("the temp directory") : fallback)
            << " either: " << e.what() << '\n';
    }
    out << to_json(report).dump(2) << '\n';
    return "report printed above";
}

}  // namespace iai_actuator_driver
