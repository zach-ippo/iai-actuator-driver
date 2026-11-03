// Text output of iai-actuator-driver-cpp (--json carries every field instead). Port of the C# CliOutput.cs / the Python cli.py
// printers.
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace cli {
namespace text {

namespace {

std::string g(double v) { return util::format("%.6g", v); }

std::string pad(const std::string& s, std::size_t width) {
    return s.size() >= width ? s : s + std::string(width - s.size(), ' ');
}

std::string range(const std::optional<Span>& r, const std::string& units, const std::string& missing) {
    if (!r) return missing;
    return util::fixed(r->min) + ".." + util::fixed(r->max) + " " + units;
}

std::string rtrim(std::string s) {
    while (!s.empty() && s.back() == ' ') s.pop_back();
    return s;
}

}  // namespace

std::string hex(const std::vector<std::uint8_t>& bytes) {
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) out += ' ';
        out += util::format("%02x", bytes[i]);
    }
    return out;
}

void ports(const std::vector<SerialPortInfo>& entries, std::ostream& out) {
    if (entries.empty()) {
        out << "no serial port found\n";
        return;
    }
    out << pad("PORT", 10) << ' ' << pad("USB ID", 10) << ' ' << pad("IAI", 4) << " DESCRIPTION\n";
    for (const auto& e : entries) {
        const std::string usb = e.vid && e.pid ? util::format("%04X:%04X", *e.vid, *e.pid) : "-";
        out << pad(e.port, 10) << ' ' << pad(usb, 10) << ' ' << pad(e.is_iai() ? "yes" : "no", 4) << ' '
            << e.description << '\n';
    }
}

void scan(const std::string& name, std::optional<int> baud, const std::vector<ScanHit>& hits, const std::string& error,
          std::ostream& out) {
    if (!error.empty()) {
        out << name << ": error: " << error << '\n';
        return;
    }
    if (hits.empty()) {
        out << name << ": no axis answered\n";
        return;
    }
    const std::string rate = baud ? std::to_string(*baud) + " baud" : "no baud rate (simulated)";
    out << name << ": " << hits.size() << " axis/axes at " << rate << '\n';
    for (const auto& h : hits) {
        std::string what;
        if (!h.error.empty()) {
            what = h.error;
        } else {
            const std::optional<std::string> fam = reg::ec_family(h.system_code, h.unit_code);
            const std::string product = reg::product_family(h.system_code);
            const std::string family = fam ? *fam : product;
            const std::string system = reg::system_name(h.system_code);
            what = (system.empty() ? std::string("unknown system") : system) +
                   util::format(", unit 0x%02X, firmware ", h.unit_code.value_or(0)) +
                   (h.firmware.empty() ? std::string("-") : h.firmware) + ": " + family;
        }
        out << "  " << pad(h.name(), 12) << " axis " << pad(std::to_string(h.axis_number()), 3) << ' ' << what << '\n';
    }
}

void status(const Status& st, std::ostream& out, const std::string& units, const std::string& line) {
    std::vector<std::string> flags;
    const std::pair<const char*, bool> named[] = {
        {"SERVO", st.servo_on},          {"HOMED", st.homed},
        {"IN-POS", st.in_position},      {"MOVING", st.moving},
        {"PAUSED", st.paused()},         {"HOMING", st.homing()},
        {"PUSHING", st.pushing()},       {"PUSH-MISSED", st.push_missed()},
        {"MODBUS", st.modbus_enabled},   {"SAFETY-SPEED", st.safety_speed()},
        {"ALARM", st.alarm},             {"WARNING", st.warning()},
        {"EMG", st.emergency_stop},      {"EMG-INPUT", st.emg_input().value_or(false)},
        {"DRIVE-POWER-CUT", st.drive_power_cut().value_or(false)},
        {"ENABLE-SWITCH-OFF", st.enable_off().value_or(false)},
        {"COLD-START-ALARM", st.cold_start_alarm().value_or(false)},
        {"AUTO-SERVO-OFF", st.auto_servo_off().value_or(false)},
        {"NV-WRITE-BUSY", st.nv_busy().value_or(false)},
        {"AUTO-MODE", st.manual_mode() == std::optional<bool>(false)},
        {"BATTERY-LOW", st.battery_low().value_or(false)},
    };
    for (const auto& f : named)
        if (f.second) flags.push_back(f.first);
    std::vector<std::string> markers;
    if (st.at_backward_end().value_or(false)) markers.push_back("BACKWARD-END");
    if (st.at_middle().value_or(false)) markers.push_back("MIDDLE");
    if (st.at_forward_end().value_or(false)) markers.push_back("FORWARD-END");
    out << util::format("position   %9.2f ", st.position_mm) << units << '\n';
    if (st.velocity_mm_s) out << util::format("velocity   %9.2f ", *st.velocity_mm_s) << units << "/s\n";
    if (st.current_ma) out << util::format("current    %9d", *st.current_ma) << '\n';
    out << util::format("alarm code %9s", util::format("0x%x", st.alarm_code).c_str()) << '\n';
    out << "flags      " << (flags.empty() ? std::string("-") : util::join(flags, " ")) << '\n';
    if (const auto done = st.position_complete_no())
        out << "row done   " << *done << " (POSS: the stored row last completed; 0 after a numeric move)\n";
    if (!line.empty()) out << "line       " << line << '\n';
    out << "markers    "
        << (!st.at_backward_end() ? std::string("not read (ZONS)")
                                  : (markers.empty() ? std::string("-") : util::join(markers, " ")))
        << '\n';
    const StatusRaw& r = st.raw;
    std::vector<std::string> raw{util::format("PNOW=0x%08x", static_cast<unsigned>(r.pnow)),
                                 util::format("ALMC=0x%04x", r.almc), util::format("DIPM=0x%04x", r.dipm),
                                 util::format("DOPM=0x%04x", r.dopm), util::format("DSS1=0x%04x", r.dss1),
                                 util::format("DSS2=0x%04x", r.dss2), util::format("DSSE=0x%04x", r.dsse)};
    if (r.stat) raw.push_back(util::format("STAT=0x%08x", static_cast<unsigned>(*r.stat)));
    if (r.zons) raw.push_back(util::format("ZONS=0x%04x", *r.zons));
    if (r.poss) raw.push_back(util::format("POSS=0x%04x", *r.poss));
    if (r.ssse) raw.push_back(util::format("SSSE=0x%04x", *r.ssse));
    out << "raw        " << util::join(raw, " ") << '\n';
}

void info(const DeviceInfo& info, Axis& axis, std::ostream& out) {
    out << "model      " << info.model() << '\n';
    out << "controller " << info.controller() << '\n';
    out << "family     " << info.product_family.value_or("-")
        << (info.controller_name ? " (" + *info.controller_name + ")" : std::string()) << '\n';
    if (info.identity_source && *info.identity_source != "FC11")
        out << "identity   from " << *info.identity_source << " (FC11 not answered)\n";
    out << "actuator   " << info.actuator().value_or("-") << '\n';
    const std::string kind = info.kind();
    out << "units      " << info.units << " (" << (util::starts_with(kind, "kind ") ? kind : "kind " + kind) << ")\n";
    if (info.max_positions && *info.max_positions > 0)
        out << "rows       " << *info.max_positions << " stored positions (0.." << *info.max_positions - 1 << ")\n";
    out << "model name " << info.model_name.value_or("-") << '\n';
    out << "unit name  " << info.user_name.value_or("-") << " (0x061A, user-defined)\n";
    out << "serial     " << info.serial_number.value_or("-") << '\n';
    out << "firmware   " << info.firmware.value_or("-") << '\n';
    if (info.axis_number) out << "axis no.   " << *info.axis_number << " (AXNO 0x8004; slave " << axis.slave() << ")\n";
    if (!info.slave_id_raw.empty()) out << "slave id   " << hex(info.slave_id_raw) << '\n';
    if (axis.baud_rate())
        out << "baud       " << *axis.baud_rate() << (axis.baud_detected() ? " (auto-detected)" : "") << '\n';
    const std::string& u = info.units;
    out << "soft limit " << range(info.soft_limits_mm, u, "-") << '\n';
    out << "stroke     " << range(axis.effective_stroke(), u, "unknown") << " (used for soft-limit checks)\n";
    const AxisRanges ranges = axis.limits();
    const std::string source =
        util::starts_with(info.speed_limit_source.value_or(""), "MAXV") ? "MAXV 7.4" : "0x0812 fallback";
    const std::string unit_speed =
        info.max_speed_mm_s ? util::fixed(*info.max_speed_mm_s) + " " + u + "/s (" + source + ")" : "not readable";
    out << "max speed  unit max " << unit_speed << "; driver cap " << util::fixed(ranges.speed.max.value_or(0)) << ' '
        << u << "/s\n";
    if (info.max_accel_g || info.max_decel_g)
        out << "max accel  unit max " << (info.max_accel_g ? util::fixed(*info.max_accel_g) : "n/a") << " / "
            << (info.max_decel_g ? util::fixed(*info.max_decel_g) : "n/a") << " G (MAXA / MAXD); driver cap "
            << util::fixed(ranges.accel_decel.max.value_or(0)) << " G\n";
    if (ranges.push.unit_max) {
        const std::string push = ranges.push.available == std::optional<bool>(false)
                                     ? "not available"
                                     : g(ranges.push.recommended_min.value_or(reg::kMnppDefaultPct)) +
                                           " % (recommended) .. " + g(*ranges.push.unit_max) + " % (MXPP); driver cap " +
                                           g(ranges.push.max.value_or(0)) + " %";
        out << "push       " << push << '\n';
    }
    if (info.index_header)
        out << util::format("index      MODL=0x%04x records=%d rx/tx buffer=%d/%d", info.index_header->model_code,
                            info.index_header->record_count, info.index_header->rx_buffer, info.index_header->tx_buffer)
            << '\n';
    for (const auto& c : info.categories)
        out << util::format("  CI=0x%04x DA=0x%04x DQ=%-4d LQ=%-4d BA=0x%04x", c.category_id, c.data_address,
                            c.member_count, c.list_count, c.coil_base)
            << '\n';
    for (const auto& e : info.errors) out << "note       " << e << '\n';
}

void alarms(const std::vector<AlarmRecord>& records, std::ostream& out) {
    if (records.empty()) out << "alarm history empty\n";
    for (const auto& r : records) {
        const std::string stamp = r.time_after_power_on().value_or("-");
        out << util::format("#%-2d code 0x%04x %-19s detail %-5d %17s after power-on  ", r.index, r.code,
                            level_key(r.level()), r.detail, stamp.c_str())
            << r.name() << '\n';
    }
}

void limits(const AxisRanges& ranges, std::ostream& out) {
    out << "units      " << ranges.units << " ("
        << (util::starts_with(ranges.kind, "kind ") ? ranges.kind : "kind " + ranges.kind) << ", family "
        << ranges.family << ")\n";
    const std::pair<const char*, const LimitRange*> rows[] = {
        {"position", &ranges.position},   {"speed", &ranges.speed},
        {"accel", &ranges.accel},         {"decel", &ranges.decel},
        {"accel_decel", &ranges.accel_decel}, {"push", &ranges.push},
        {"band", &ranges.band},           {"push_band", &ranges.push_band},
        {"stored_rows", &ranges.stored_rows}, {"jog_step", &ranges.jog_step},
        {"speed_override", &ranges.speed_override},
    };
    for (const auto& row : rows) {
        const LimitRange& r = *row.second;
        const std::string span = (r.min ? g(*r.min) : "-") + " .. " + (r.max ? g(*r.max) : "-");
        std::vector<std::string> extra;
        if (r.available) extra.push_back(*r.available ? "available" : "NOT available");
        if (r.recommended_min)
            extra.push_back("recommended min " + g(*r.recommended_min) +
                            (r.recommended_min_source.empty() ? std::string() : " (" + r.recommended_min_source + ")"));
        if (!r.source.empty()) extra.push_back(r.source);
        out << rtrim(pad(row.first, 14) + ' ' + pad(span, 22) + ' ' + pad(r.unit, 6) + ' ' + util::join(extra, "; "))
            << '\n';
    }
}

void parameters(const std::vector<ParameterInfo>& list, const ParameterModel* model, std::ostream& out,
                const std::string& units, bool is_ec, const std::string& family) {
    if (is_ec)
        out << "user parameters (" << (model ? model->note : std::string()) << ")\n";
    else
        out << family << " parameters (offered when the item is in the unit's ID map)\n";
    if (list.empty()) out << "no named parameter found in this unit's ID map\n";
    for (const auto& p : list) {
        std::vector<std::string> flags{p.writable ? "writable" : "READ-ONLY"};
        if (p.dangerous) flags.push_back(is_ec ? "HOME, DANGEROUS" : "DANGEROUS");
        if (!p.note.empty()) flags.push_back(p.note);
        std::string value = p.text.value_or(p.value.to_string());
        if (units != "mm") {
            const std::size_t at = value.find(" mm");
            if (at != std::string::npos) value.replace(at, 3, " " + units);
        }
        const std::string label = (is_ec ? "P" : "No.") + std::to_string(p.number);
        const std::string span = p.min && p.max ? "[" + g(*p.min) + ".." + g(*p.max) + "]" : "";
        out << rtrim("  " + pad(label, 7) + ' ' + pad(p.name, 44) + ' ' + pad(value.empty() ? "-" : value, 22) + ' ' +
                     pad(span, 18) + ' ' + p.item + " @" + p.address.value_or("-") + " = " + p.raw.value_or("-") +
                     "  " + util::join(flags, "; "))
            << '\n';
    }
}

void parameter_write(const ParameterWriteResult& r, std::ostream& out, const std::string& prefix) {
    const std::string label = r.number ? prefix + std::to_string(r.number) : "item";
    if (r.skipped) {
        out << label << ' ' << r.name << " (" << r.key << " @" << r.address << ") = " << r.after << " (raw "
            << r.raw_after << "): " << r.note << "; --force writes it anyway\n";
        for (const auto& w : r.warnings) out << "warning: " << w << '\n';
        return;
    }
    out << label << ' ' << r.name << " (" << r.key << " @" << r.address << "): " << r.before << " -> " << r.after
        << " (raw " << r.raw_before << " -> " << r.raw_after << ", read back OK)\n";
    if (r.committed && r.commit_confirmed)
        out << "committed to non-volatile memory (PARW 0x0428: the unit showed the commit, then idle again)\n";
    else if (r.committed)
        out << "PARW 0x0428 sent, but the commit was NOT CONFIRMED (the unit never showed it busy): the value may be RAM "
               "only - check it after a power cycle\n";
    else
        out << "RAM ONLY (--no-commit): lost at the next power-up\n";
    if (r.forced || (!r.changed && r.committed)) out << "note: " << r.note << '\n';
    out << "takes effect " << r.takes_effect() << "; run 'swreset' to apply it now\n";
    for (const auto& w : r.warnings) out << "warning: " << w << '\n';
}

void row_write(const PositionWriteResult& r, std::ostream& out, const std::string& units) {
    const PositionEntry& e = r.entry;
    if (r.skipped) {
        out << "row " << r.row << " already holds " << util::fixed(e.position_mm) << ' ' << units << ": " << r.note
            << "; --force writes it anyway\n";
    } else {
        out << "row " << r.row << " taught from the current position: " << util::fixed(e.position_mm) << ' ' << units
            << ", speed " << util::fixed(e.speed_mm_s) << ", accel / decel " << util::fixed(e.accel_g) << " / "
            << util::fixed(e.decel_g) << " G, band " << util::fixed(e.band_mm) << " (non-volatile, read back)\n";
        if (r.forced) out << "note: " << r.note << '\n';
    }
    for (const auto& w : r.warnings) out << "warning: " << w << '\n';
}

void reset(const SoftwareResetResult& r, std::ostream& out) {
    out << "software reset " << (r.accepted ? "accepted (45 00 00)" : "answer lost; confirmed from the status")
        << ", unit answering again after " << util::fixed(r.elapsed_s) << " s"
        << (r.baud_rate ? " at " + std::to_string(*r.baud_rate) + " baud" : std::string()) << '\n';
    out << "PMSS " << (r.modbus_enabled() ? 1 : 0) << "  SFTY " << (r.safety_speed() ? 1 : 0) << "  servo "
        << (r.servo_on() ? "ON" : "OFF") << "  HEND " << (r.homed() ? 1 : 0)
        << util::format("  alarm 0x%03x", r.alarm_code()) << '\n';
    if (r.confirmed)
        out << "restart confirmed by " << (r.confirmed_by.empty() ? std::string("the status") : r.confirmed_by) << '\n';
    else
        out << "restart NOT confirmed (no PMSS drop, reply gap, uptime restart or HEND loss seen): check the unit\n";
    if (r.needs_home()) out << "NEEDS HOME: run 'home' (after 'alarm-reset' if an alarm is active) before the next move\n";
    for (const auto& n : r.notes) out << "note: " << n << '\n';
}

void baud(const BaudChangeResult& r, std::ostream& out) {
    out << "baud " << r.previous << " -> " << r.baud_rate << " (" << to_string(r.method) << "; broadcast " << r.frame
        << "; " << (r.verified ? "verified" : "found by probing") << ")\n";
    if (!r.code_verified)
        out << util::format("warning: code 0x%02X is not verified on hardware (sent anyway)", r.code) << '\n';
    out << "note: " << r.note << '\n';
}

}  // namespace text
}  // namespace cli
}  // namespace iai_actuator_driver
