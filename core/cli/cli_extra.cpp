// The commands of iai-actuator-driver-cpp added for parity with the Python CLI: info, monitor (--watch), push, move with a
// change of the running move, backup, recover, line and check, with their text output (--json carries every field).
// Port of the Python edition's cli.py (_print_unit_info, _run_watch, _run_backup, _run_recover, _print_recovery,
// _run_line, _run_check, _move_and_change). This edition is UNVERIFIED on hardware.
#include <algorithm>
#include <cmath>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "axis_impl.hpp"
#include "cli.hpp"
#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "monotonic_cv.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace cli {

namespace {

// ------------------------------------------------------------------------------------------ Python-style formatting

/// str() of a JSON value the way the Python edition prints it: None, True / False, 12, 12.5, text, [1, 2].
std::string pystr(const Json* v) {
    if (!v || v->is_null()) return "None";
    switch (v->type()) {
        case Json::Type::Bool: return v->as_bool() ? "True" : "False";
        case Json::Type::Int: return std::to_string(v->as_int());
        case Json::Type::Number: return util::format("%.10g", v->as_number());  // as the JSON output writes it
        case Json::Type::String: return v->as_string();
        case Json::Type::Array: {
            std::vector<std::string> parts;
            for (const Json& e : v->as_array())
                parts.push_back(e.type() == Json::Type::String ? "'" + e.as_string() + "'" : pystr(&e));
            return "[" + util::join(parts, ", ") + "]";
        }
        default: return v->dump();
    }
}

const Json* field(const Json* j, const char* key) { return j ? j->find(key) : nullptr; }

/// Python truthiness: null, false, 0, "" and an empty list / object are false.
bool truthy(const Json* v) {
    if (!v || v->is_null()) return false;
    switch (v->type()) {
        case Json::Type::Bool: return v->as_bool();
        case Json::Type::Int: return v->as_int() != 0;
        case Json::Type::Number: return v->as_number() != 0.0;
        case Json::Type::String: return !v->as_string().empty();
        default: return v->size() != 0;
    }
}

/// `value or '-'`.
std::string or_dash(const Json* v, const std::string& missing = "-") { return truthy(v) ? pystr(v) : missing; }

double num(const Json* v, double fallback = 0.0) {
    if (!v || (v->type() != Json::Type::Int && v->type() != Json::Type::Number)) return fallback;
    return v->as_number();
}

bool is_num(const Json* v) { return v && (v->type() == Json::Type::Int || v->type() == Json::Type::Number); }

std::string left(const std::string& s, std::size_t width) {
    return s.size() >= width ? s : s + std::string(width - s.size(), ' ');
}
std::string right(const std::string& s, std::size_t width) {
    return s.size() >= width ? s : std::string(width - s.size(), ' ') + s;
}

std::string rstrip(std::string s) {
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.pop_back();
    return s;
}

std::string replace_all(std::string text, const std::string& from, const std::string& to) {
    std::size_t at = 0;
    while ((at = text.find(from, at)) != std::string::npos) {
        text.replace(at, from.size(), to);
        at += to.size();
    }
    return text;
}

std::string mm_range(const Json* v) {
    if (!truthy(v) || v->type() != Json::Type::Array || v->size() < 2) return "-";
    return util::fixed(v->as_array()[0].as_number()) + " .. " + util::fixed(v->as_array()[1].as_number()) + " mm";
}

std::string bits_line(const std::string& name, const Json* word) {
    if (!word || word->is_null()) return "  " + left(name, 5) + " -";
    std::vector<std::string> on;
    if (const Json* list = field(word, "on"))
        if (list->type() == Json::Type::Array)
            for (const Json& e : list->as_array()) on.push_back(pystr(&e));
    return "  " + left(name, 5) + " " + left(or_dash(field(word, "hex"), ""), 10) + "  " +
           (on.empty() ? std::string("-") : util::join(on, " "));
}

/// Sleep up to `seconds`, waking early on Ctrl+C.
void nap(double seconds) {
    const double end = now_s() + std::max(0.0, seconds);
    while (now_s() < end && !interrupted()) sleep_at_least(std::min(0.02, end - now_s()));
}

}  // namespace

// ============================================================================================ text printers

namespace text {

std::string status_line(const Status& st, const std::string& units) {
    std::vector<std::string> flags;
    const std::pair<const char*, bool> named[] = {
        {"SERVO", st.servo_on},          {"HOMED", st.homed},       {"IN-POS", st.in_position},
        {"MOVING", st.moving},           {"PAUSED", st.paused()},   {"HOMING", st.homing()},
        {"PUSHING", st.pushing()},       {"MODBUS", st.modbus_enabled}, {"ALARM", st.alarm},
        {"WARNING", st.warning()},       {"EMG", st.emergency_stop},
    };
    for (const auto& f : named)
        if (f.second) flags.push_back(f.first);
    const std::string velocity =
        st.velocity_mm_s ? util::format("  velocity %8.2f ", *st.velocity_mm_s) + units + "/s" : std::string();
    const std::string alarm = st.alarm_code ? util::format("  alarm 0x%03X", st.alarm_code) : std::string();
    return util::format("position %9.2f ", st.position_mm) + units + velocity + "  " +
           (flags.empty() ? std::string("-") : util::join(flags, " ")) + alarm;
}

void unit_info(const Json& data, std::ostream& sink) {
    std::ostringstream out;
    auto p = [&out](const std::string& text = std::string()) { out << rstrip(text) << '\n'; };
    const Json* ident = data.find("identity");
    const Json* meta = data.find("meta");
    p(std::string(kBanner) + " - unit info, read-only (FC03 + FC11), " + or_dash(field(meta, "taken_at"), "") + ", " +
      util::fixed(num(field(meta, "duration_s"))) + " s");
    if (truthy(ident)) {
        p("\n[identity]");
        p("  model         " + or_dash(field(ident, "model")));
        p("  unit name     " + or_dash(field(ident, "user_name")) + " (0x061A, user-defined; not a model field)");
        p("  serial        " + or_dash(field(ident, "serial_number")));
        const Json* source = field(ident, "identity_source");
        const bool other_source = truthy(source) && pystr(source) != "FC11";
        p("  controller    " + pystr(field(ident, "controller")) + ", firmware " + or_dash(field(ident, "firmware")) +
          (other_source ? " (identity from " + pystr(source) + ")" : std::string()));
        p("  actuator      " + or_dash(field(ident, "actuator")));
        if (const Json* axno = field(ident, "axis_number"))
            if (!axno->is_null()) p("  axis number   " + pystr(axno) + " (AXNO 0x8004)");
        if (truthy(field(ident, "slave_id_raw"))) p("  slave id      " + pystr(field(ident, "slave_id_raw")) + " (FC11)");
        const Json* baud = field(ident, "baudrate");
        const std::string detected = truthy(field(ident, "baud_detected")) ? " (auto-detected)" : "";
        const std::string link = truthy(baud) ? "baud " + pystr(baud) + detected : "no baud rate (simulator)";
        p("  link          slave " + pystr(field(ident, "slave")) + ", " + link);
        p("  soft limits   " + mm_range(field(ident, "soft_limits_mm")) + " (stroke " + mm_range(field(ident, "stroke_mm")) +
          ", used " + mm_range(field(ident, "effective_stroke_mm")) + ")");
        const Json* ec_speed = field(ident, "ec_max_speed_mm_s");
        const Json* accel_cap = field(ident, "accel_cap_g");
        if (!accel_cap) accel_cap = field(ident, "max_accel_g");
        p("  speed         unit max " + (is_num(ec_speed) ? util::fixed(ec_speed->as_number()) : std::string("-")) +
          " mm/s (MAXV 7.4), driver cap " + util::fixed(num(field(ident, "speed_cap_mm_s"))) + " mm/s, accel cap " +
          util::fixed(num(accel_cap)) + " G, push cap " + or_dash(field(ident, "push_cap_pct")) + " %");
        const Json* caps = field(ident, "capabilities");
        const Json* flags = field(field(caps, "support"), "flags");
        if (truthy(flags) && flags->type() == Json::Type::Object) {
            std::vector<std::string> on;
            for (const auto& kv : flags->as_object())
                if (kv.second.type() == Json::Type::Bool && kv.second.as_bool()) on.push_back(replace_all(kv.first, "_", " "));
            p("  features      " + (on.empty() ? std::string("-") : util::join(on, ", ")) + " (class 1 support bits)");
        }
        const Json* coll = field(caps, "collision");
        if (truthy(coll)) {
            std::vector<std::string> items;
            if (const Json* list = field(coll, "parameter_items"))
                if (list->type() == Json::Type::Array)
                    for (const Json& e : list->as_array()) items.push_back(pystr(&e));
            p("  collision     " + pystr(field(coll, "reported")) + "; parameter items " +
              (items.empty() ? std::string("none") : util::join(items, ", ")));
        }
        const Json* link_caps = field(caps, "link");
        if (truthy(field(link_caps, "boot_baudrate")))
            p("  boot baud     " + pystr(field(link_caps, "boot_baudrate")) + " (BRSL item 8.3), transmit delay " +
              pystr(field(link_caps, "transmit_delay_ms")) + " ms");
        const Json* sm = field(caps, "servo_monitor");
        if (truthy(sm))
            p("  servo monitor " + pystr(field(sm, "mode_name")) + " (FMNT 8.54 = " + pystr(field(sm, "fmnt_hex")) + ")");
        const Json* mode = field(ident, "position_mode");
        p("  position mode " + pystr(field(mode, "name")) + " (" + pystr(field(mode, "source")) + ")");
        const Json* extra = field(ident, "identification_extra");
        if (truthy(extra)) {
            const Json* accl = field(extra, "accl");
            const Json* serial = field(extra, "actuator_serial");
            p("  0x0600 extra  actuator serial " +
              (serial && serial->type() == Json::Type::String ? "'" + serial->as_string() + "'" : pystr(serial)) +
              ", manufacture info " + pystr(field(field(extra, "manufacture_info"), "hex")) + ", ACCL " +
              pystr(field(accl, "hex")) + " (level " + pystr(field(accl, "extensibility_level")) + " " +
              pystr(field(accl, "level_name")) + "), BISD " + pystr(field(extra, "bisd_raw")) + " (date? " +
              or_dash(field(extra, "date_guess")) + ")");
        }
        const Json* directory = field(ident, "directory");
        const Json* header = field(directory, "header");
        p("  directory     " + util::format("%lld", static_cast<long long>(num(field(directory, "record_count")))) +
          " records, " + util::format("%lld", static_cast<long long>(num(field(directory, "total_words")))) +
          " registers (MODL " + util::format("0x%04x", static_cast<unsigned>(num(field(header, "model_code")))) +
          ", buffers " + pystr(field(header, "rx_buffer")) + "/" + pystr(field(header, "tx_buffer")) + ")");
        bool inexact = false;
        if (const Json* records = field(directory, "records")) {
            if (records->type() == Json::Type::Array) {
                for (const Json& r : records->as_array()) {
                    const bool exact = truthy(r.find("exact"));
                    inexact = inexact || !exact;
                    const std::string words = pystr(r.find("words")) + (exact ? "" : "*");
                    p("    " + pystr(r.find("class")) + "  " + pystr(r.find("address")) + "-" + pystr(r.find("end")) +
                      "  " + right(words, 6) + "  DQ " + left(pystr(r.find("members")), 3) + " LQ " +
                      left(pystr(r.find("entries")), 4) + " MS " + pystr(r.find("member_size")) + "  id map " +
                      left(or_dash(r.find("id_map")), 6) + "  " + pystr(r.find("name")));
                }
            }
        }
        if (inexact) p("    * mixed member sizes: only this many registers are certain (the driver reads no further)");
        if (const Json* notes = field(ident, "notes"))
            if (notes->type() == Json::Type::Array)
                for (const Json& n : notes->as_array()) p("  note          " + pystr(&n));
    }
    const Json* params = data.find("parameters");
    if (truthy(params)) {
        p("\n[user parameters]");
        const Json* model = field(params, "model");
        if (truthy(model)) p("  (" + pystr(field(model, "note")) + ")");
        if (const Json* named = field(params, "named")) {
            if (named->type() == Json::Type::Array) {
                for (const Json& prm : named->as_array()) {
                    const std::string where = truthy(prm.find("found"))
                                                  ? pystr(prm.find("key")) + " @" + pystr(prm.find("address")) + " = " +
                                                        pystr(prm.find("raw_hex"))
                                                  : pystr(prm.find("key"));
                    const std::string flag =
                        truthy(prm.find("model_note")) ? "  [" + pystr(prm.find("model_note")) + "]" : std::string();
                    p("  P" + left(pystr(prm.find("no")), 3) + " " + left(pystr(prm.find("name")), 52) + " " +
                      left(pystr(prm.find("text")), 28) + " " + where + flag);
                }
            }
        }
        if (const Json* missing = field(params, "not_located")) {
            if (missing->type() == Json::Type::Array) {
                for (const Json& q : missing->as_array()) {
                    if (!truthy(q.find("listed"))) continue;
                    const std::string seen = truthy(q.find("iaos_text"))
                                                 ? " (IA-OS showed '" + pystr(q.find("iaos_text")) + "'; not read from the EC)"
                                                 : std::string();
                    p("  P" + left(pystr(q.find("no")), 3) + " " + left(pystr(q.find("name")), 52) + " not located" + seen);
                }
            }
        }
        p("  (" + pystr(field(params, "note")) + ")");
        if (const Json* classes = field(params, "classes")) {
            if (classes->type() == Json::Type::Array) {
                for (const Json& block : classes->as_array()) {
                    const Json* items = block.find("items");
                    p("\n  class " + pystr(block.find("class")) + " @" + pystr(block.find("address")) + ": " +
                      pystr(block.find("name")) + " (" + std::to_string(items ? items->size() : 0) + " items)");
                    if (!items || items->type() != Json::Type::Array) continue;
                    for (const Json& it : items->as_array()) {
                        std::vector<std::string> tags;
                        if (const Json* list = it.find("params"))
                            if (list->type() == Json::Type::Array)
                                for (const Json& e : list->as_array()) tags.push_back(pystr(&e));
                        tags.push_back(it.find("note") ? pystr(it.find("note")) : std::string());
                        p("    " + left(pystr(it.find("key")), 8) + " " + pystr(it.find("address")) + "  " +
                          right(pystr(it.find("hex")), 8) + "  " + right(pystr(it.find("signed")), 11) + "  " +
                          right(util::fixed(num(it.find("scaled"))), 12) + "  " + util::trim(util::join(tags, " ")));
                    }
                }
            }
        }
    }
    const Json* io = data.find("io");
    if (truthy(io)) {
        p("\n[I/O] " + pystr(field(io, "mode")));
        for (const char* name : {"DIPM", "DOPM", "SIPM", "ZONS"}) p(bits_line(name, field(io, name)));
        std::vector<std::string> lamps;
        if (const Json* l = field(io, "lamps"))
            if (l->type() == Json::Type::Object)
                for (const auto& kv : l->as_object())
                    lamps.push_back(replace_all(kv.first, "_", " ") + " " + (truthy(&kv.second) ? "ON" : "-"));
        p("  lamps         " + util::join(lamps, ", "));
        p("  (" + pystr(field(io, "note")) + ")");
    }
    const Json* words = data.find("status_words");
    if (truthy(words)) {
        p("\n[status words]");
        for (const char* name : {"DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SSSE", "SIPM"})
            if (const Json* w = field(words, name)) p(bits_line(name, w));
        if (const Json* poss = field(words, "POSS"))
            p("  POSS  " + left(pystr(field(poss, "hex")), 10) + "  position " + pystr(field(poss, "position_no")));
    }
    const Json* control = data.find("control_words");
    if (truthy(control) && control->type() == Json::Type::Object) {
        p("\n[control registers 0x0D00]");
        for (const auto& kv : control->as_object())
            p(kv.second.find("bits") ? bits_line(kv.first, &kv.second)
                                     : "  " + left(kv.first, 5) + " " + pystr(kv.second.find("hex")));
    }
    const Json* mon = data.find("monitor");
    if (truthy(mon)) {
        p("\n[monitor]");
        if (const Json* v = field(mon, "position_mm")) p("  position      " + util::fixed(num(v)) + " mm");
        if (const Json* v = field(mon, "velocity_mm_s")) p("  velocity      " + util::fixed(num(v)) + " mm/s (VNOW, quantized)");
        const std::string pct = field(mon, "current_pct") ? ", " + pystr(field(mon, "current_pct")) +
                                                                " % of rated (MCUR " + pystr(field(mon, "rated_current")) + ")"
                                                          : std::string();
        p("  current       " + (field(mon, "current") ? pystr(field(mon, "current")) : std::string("-")) + " (CNOW" + pct +
          ")");
        p("  deviation     " + (field(mon, "deviation") ? pystr(field(mon, "deviation")) : std::string("-")) + " (DEVI)");
        if (field(mon, "uptime_ms"))
            p("  uptime        " + pystr(field(mon, "uptime")) + " (" + pystr(field(mon, "uptime_ms")) + " ms since power-on)");
        if (field(mon, "overload_level"))
            p("  overload lvl  " + pystr(field(mon, "overload_level")) + " (OLLV, % unconfirmed)");
    }
    const Json* alarms = data.find("alarms");
    if (truthy(alarms)) {
        p("\n[alarms]");
        const Json* cur = field(alarms, "current");
        if (cur && !cur->is_null()) {
            const bool active = truthy(cur->find("active"));
            p("  current       " + pystr(cur->find("hex")) + " " + pystr(cur->find("name")) +
              (active ? " (" + pystr(cur->find("level_text")) + ")" : std::string()));
            if (active) p("  remedy        " + pystr(cur->find("remedy")));
        }
        p("  history       " + pystr(field(alarms, "alarm_count")) + " alarms, " + pystr(field(alarms, "power_up_markers")) +
          " power-up markers, " + pystr(field(alarms, "empty_slots")) + " empty of " + pystr(field(alarms, "slots")) +
          " slots (" + pystr(field(field(alarms, "notes"), "order")) + ")");
        if (const Json* history = field(alarms, "history")) {
            if (history->type() == Json::Type::Array) {
                for (const Json& a : history->as_array()) {
                    p("  #" + left(pystr(a.find("index")), 3) + " " + pystr(a.find("hex")) + "  " +
                      left(pystr(a.find("name")), 40) + " " + left(pystr(a.find("level")), 19) + " detail " +
                      left(pystr(a.find("detail")), 5) + " " + right(or_dash(a.find("time_after_power_on")), 17) +
                      " after power-on  " + pystr(a.find("newer_power_ups")) + " newer power-ups");
                }
            }
        }
    }
    const Json* maint = data.find("maintenance");
    if (truthy(maint)) {
        p("\n[maintenance]");
        p("  total moves   " + pystr(field(maint, "total_moves")));
        p("  distance      " + pystr(field(maint, "distance_m")) + " m");
        if (const Json* items = field(maint, "items")) {
            if (items->type() == Json::Type::Array) {
                for (const Json& it : items->as_array()) {
                    std::string value = truthy(it.find("unset")) ? "not set" : pystr(it.find("value"));
                    if (it.find("hi16")) value += " (" + pystr(it.find("hi16")) + " / " + pystr(it.find("lo16")) + ")";
                    const std::string sure = truthy(it.find("confirmed")) ? "" : "?";
                    p("    " + right(pystr(it.find("id")), 3) + " " + left(pystr(it.find("symbol")) + sure, 6) + " " +
                      pystr(it.find("address")) + "  " + pystr(it.find("hex")) + "  " + left(value, 20) + " " +
                      pystr(it.find("meaning")));
                }
            }
        }
        p("  (" + pystr(field(maint, "note")) + ")");
    }
    const Json* pos = data.find("positions");
    if (truthy(pos)) {
        p("\n[positions] " + pystr(field(pos, "mode")));
        if (const Json* rows = field(pos, "rows")) {
            if (rows->type() == Json::Type::Array) {
                for (const Json& r : rows->as_array()) {
                    const std::string push = truthy(r.find("push"))
                                                 ? ", push " + pystr(r.find("push_pct")) + " % to " +
                                                       util::fixed(num(r.find("push_end_mm"))) + " mm (" +
                                                       pystr(r.find("push_direction")) + ")"
                                                 : std::string();
                    p("  row " + left(pystr(r.find("n")), 3) + " " + left(pystr(r.find("role")), 26) + " " +
                      util::format("%8.2f", num(r.find("position_mm"))) + " mm  " +
                      util::format("%7.2f", num(r.find("speed_mm_s"))) + " mm/s  A/D " + util::fixed(num(r.find("accel_g"))) +
                      "/" + util::fixed(num(r.find("decel_g"))) + " G  band " + util::fixed(num(r.find("band_mm"))) + push +
                      "  CTLF " + pystr(r.find("control_flags")) + "  checksum " +
                      (truthy(r.find("checksum_ok")) ? "ok" : "BAD"));
                }
            }
        }
        if (const Json* rows = field(pos, "simple_data_rows")) {
            if (rows->type() == Json::Type::Array) {
                for (const Json& r : rows->as_array()) {
                    p("  row " + left(pystr(r.find("n")), 3) + " " + left(pystr(r.find("role")), 26) + " " +
                      util::format("%8.2f", num(r.find("position_mm"))) + " mm  V " + pystr(r.find("speed_pct")) +
                      " %  A/D " + pystr(r.find("accel_pct")) + "/" + pystr(r.find("decel_pct")) + " %  band " +
                      util::fixed(num(r.find("band_mm"))) + "  CTLF " + pystr(r.find("control_flags")));
                }
            }
        }
        const Json* dm = field(pos, "direct_move");
        if (truthy(dm))
            p("  0x9900 last   " + util::fixed(num(field(dm, "target_mm"))) + " mm, " +
              util::fixed(num(field(dm, "speed_mm_s"))) + " mm/s, " + util::fixed(num(field(dm, "accel_g"))) + " G, band " +
              util::fixed(num(field(dm, "band_mm"))) + ", PPOW " + pystr(field(dm, "push_power")) + ", CTLF " +
              pystr(field(dm, "control_flags")));
        const Json* ap = field(pos, "active_profile");
        if (truthy(ap) && field(ap, "target_mm"))
            p("  0x0F00 used   " + util::fixed(num(field(ap, "target_mm"))) + " mm, " +
              util::fixed(num(field(ap, "speed_mm_s"))) + " mm/s, A/D " + util::fixed(num(field(ap, "accel_g"))) + "/" +
              util::fixed(num(field(ap, "decel_g"))) + " G, band " + util::fixed(num(field(ap, "band_mm"))));
    }
    const Json* errors = data.find("errors");
    if (truthy(errors) && errors->type() == Json::Type::Array) {
        p("\n[not read]");
        for (const Json& e : errors->as_array()) p("  " + pystr(&e));
    }
    // Positions and speeds in the axis's units (deg / deg/s on a rotary axis): only the unit label differs.
    std::string text = out.str();
    const Json* units = field(ident, "units");
    if (units && units->type() == Json::Type::String && !units->as_string().empty() && units->as_string() != "mm") {
        text = replace_all(text, " mm/s", " " + units->as_string() + "/s");
        text = replace_all(text, " mm", " " + units->as_string());
    }
    sink << text;
}

void recovery(const recovery::RecoveryReport& report, const std::string& u, std::ostream& out) {
    out << report.summary() << '\n';
    if (!report.restart_signs.empty()) out << "restart signs: " << util::join(report.restart_signs, "; ") << '\n';
    if (report.status && report.status->raw.stim)
        out << util::format("uptime     %9.1f s (the unit's power-on time; a power-up restarts it)",
                            *report.status->raw.stim / 1000.0)
            << '\n';
    out << "baud rate  " << (report.baud_rate ? std::to_string(*report.baud_rate) : std::string("n/a"))
        << (report.baud_restored ? " (restored)" : "") << '\n';
    out << "PC control " << (report.modbus_control.value_or(false) ? "yes" : "no")
        << (report.control_restored ? " (taken again)" : "") << '\n';
    out << "servo      " << (report.servo_on.value_or(false) ? "ON" : "OFF") << '\n';
    out << "homed      " << (report.homed.value_or(false) ? "yes" : "NO: home() before the next move") << '\n';
    if (report.position_mm) {
        out << util::format("position   %9.2f ", *report.position_mm) << u;
        if (report.last_target_mm && report.position_error_mm)
            out << " (target " << util::fixed(*report.last_target_mm) << ", " << util::format("%+.2f", *report.position_error_mm)
                << ")";
        out << '\n';
    }
    if (report.alarm_code)
        out << util::format("alarm      0x%03X ", report.alarm_code) << report.alarm_name.value_or("") << " ("
            << report.alarm_level.value_or("None") << ")\n";
    for (const auto& action : report.actions) out << "done:      " << action << '\n';
    for (const auto* list : {&report.limits_changed, &report.row_problems, &report.parameter_differences})
        for (const auto& problem : *list) out << "differs:   " << problem << '\n';
    for (const auto& note : report.notes) out << "note:      " << note << '\n';
    out << "ready to move: " << (report.ready_to_move() ? "yes" : "no")
        << "; needs attention: " << (report.needs_attention() ? "YES" : "no") << " (nothing was moved)\n";
}

}  // namespace text

// ============================================================================================ commands

namespace extra {

std::string file_stamp() {
    const std::time_t t = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &t);
#else
    localtime_r(&t, &local);
#endif
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &local);
    return buf;
}

int info(Axis& axis, const Args& args, std::ostream& out) {
    const std::vector<std::string> wanted = args.every("--section");
    const Json data = info::unit_info(axis, wanted.empty() ? std::nullopt : std::optional<std::vector<std::string>>(wanted));
    if (args.has("--json"))
        out << data.dump(2) << '\n';
    else
        text::unit_info(data, out);
    return 0;
}

namespace {

/// One `monitor` read (0x9000 monitor block, read-only): the Python edition's read_monitor() fields.
Json monitor_reading(Axis& axis) {
    const Json data = info::unit_info(axis, std::vector<std::string>{"monitor"});
    const Json* m = data.find("monitor");
    if (!m || m->is_null()) {
        std::string why = "the monitor block was not read";
        if (const Json* errors = data.find("errors"))
            if (errors->size()) why = pystr(&errors->as_array().front());
        throw EcNotReadyError(why);
    }
    Json r = Json::object();
    r.set("position", m->find("position_mm") ? *m->find("position_mm") : Json());
    r.set("velocity", m->find("velocity_mm_s") ? *m->find("velocity_mm_s") : Json());
    r.set("current", m->find("current") ? *m->find("current") : Json());
    r.set("current_pct", m->find("current_pct") ? *m->find("current_pct") : Json());
    r.set("deviation", m->find("deviation") ? *m->find("deviation") : Json());
    const int alarm = static_cast<int>(num(m->find("alarm_code")));
    r.set("alarm", alarm);
    r.set("alarm_name", alarm ? Json(alarm_name(alarm)) : Json());
    r.set("uptime_s", m->find("uptime_s") ? *m->find("uptime_s") : Json());
    r.set("units", axis.units());
    r.set("raw", m->find("raw") ? *m->find("raw") : Json::object());
    return r;
}

void print_monitor(const Json& m, std::ostream& out) {
    const std::string u = or_dash(m.find("units"), "mm");
    auto f2 = [](const Json* v) { return is_num(v) ? util::fixed(v->as_number()) : std::string("-"); };
    auto d = [](const Json* v) { return is_num(v) ? std::to_string(static_cast<long long>(v->as_number())) : std::string("-"); };
    const Json* pct = m.find("current_pct");
    const std::string pct_text = is_num(pct) ? " (" + util::format("%g", pct->as_number()) + " % of rated)" : "";
    const int alarm = static_cast<int>(num(m.find("alarm")));
    const std::string alarm_text =
        alarm ? rstrip(util::format("0x%03X ", alarm) + (truthy(m.find("alarm_name")) ? pystr(m.find("alarm_name")) : ""))
              : std::string("-");
    const Json* up = m.find("uptime_s");
    out << "position " << f2(m.find("position")) << ' ' << u << "  velocity " << f2(m.find("velocity")) << ' ' << u
        << "/s  current " << d(m.find("current")) << pct_text << "  deviation " << d(m.find("deviation")) << "  alarm "
        << alarm_text << "  uptime " << (is_num(up) ? util::format("%.1f", up->as_number()) : std::string("-")) << " s\n";
}

/// monitor --watch: the status monitor with the link watch, printed until Ctrl+C or --duration.
int watch(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err) {
    try {
        recovery::TrackOptions t;
        t.name = label;
        recovery::track(axis, t);  // read-only: the baseline restart detection compares with
    } catch (const EcDriverError& e) {
        err << "note: not tracked (" << detail::error_type_name(e) << ": " << e.what()
            << "): a restart is not detected, a lost link still is\n";
    }
    struct Item {
        bool event = false;
        std::optional<Status> status;
        std::optional<LinkEvent> link;
    };
    struct Queue {
        std::mutex mutex;
        ::iai_actuator_driver::detail::MonotonicCv cv;
        std::deque<Item> items;
        void put(Item item) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                items.push_back(std::move(item));
            }
            cv.notify_one();
        }
    };
    auto queue = std::make_shared<Queue>();
    MonitorOptions mo;
    mo.on_change.push_back([queue](const Status& st, const std::optional<Status>&) {
        Item it;
        it.status = st;
        queue->put(std::move(it));
    });
    mo.on_event.push_back([queue](const LinkEvent& e) {
        Item it;
        it.event = true;
        it.link = e;
        queue->put(std::move(it));
    });
    const std::shared_ptr<AxisMonitor> mon = axis.start_monitor(mo);
    const bool json = args.has("--json");
    const double started = now_s();
    const std::optional<double> duration = args.number("--duration");
    const std::optional<double> deadline =
        duration ? std::optional<double>(started + std::max(0.0, *duration)) : std::nullopt;
    const double interval = std::max(0.0, args.number("--interval").value_or(0.5));
    std::optional<Status> pending;
    double last_print = -1e300;
    int code = 0;
    auto emit = [&](const std::string& kind, Json payload, const std::string& line) {
        const double t = std::round((now_s() - started) * 1000.0) / 1000.0;
        if (json) {
            Json j = Json::object();
            j.set("t", t).set("type", kind);
            if (payload.type() == Json::Type::Object)
                for (const auto& kv : payload.as_object()) j.set(kv.first, kv.second);
            out << j.dump() << std::endl;
        } else {
            out << util::format("%8.3f s  ", t) << line << std::endl;
        }
    };
    auto show = [&](const Status& st) {
        Json payload = Json::object();
        payload.set("status", to_json(st));
        emit("status", payload, text::status_line(st, axis.units()));
    };
    while ((!deadline || now_s() < *deadline) && !interrupted()) {
        Item item;
        bool got = false;
        {
            std::unique_lock<std::mutex> lock(queue->mutex);
            queue->cv.wait_for(lock, std::chrono::milliseconds(50), [&] { return !queue->items.empty(); });
            if (!queue->items.empty()) {
                item = std::move(queue->items.front());
                queue->items.pop_front();
                got = true;
            }
        }
        if (got && item.event && item.link) {
            std::string kind = item.link->kind;
            std::transform(kind.begin(), kind.end(), kind.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            emit("event", item.link->to_json(), kind + ": " + item.link->message);
        } else if (got && item.status) {
            pending = item.status;
        }
        if (pending && now_s() - last_print >= interval) {
            show(*pending);
            pending.reset();
            last_print = now_s();
        }
        bool empty = false;
        {
            std::lock_guard<std::mutex> lock(queue->mutex);
            empty = queue->items.empty();
        }
        if (!mon->running() && empty) {
            if (const std::exception_ptr error = mon->error()) {
                const std::string why = detail::describe(error);
                Json payload = Json::object();
                payload.set("error", why);
                emit("stopped", payload, "status monitor stopped: " + why + " (run 'recover')");
                code = 2;
            }
            break;
        }
    }
    axis.stop_monitor();
    if (pending) show(*pending);
    const std::optional<double> rate = mon->poll_rate_hz();
    const double rate_1 = rate ? std::round(*rate * 10.0) / 10.0 : 0.0;
    const std::string state = recovery::link_state(axis);
    Json summary = Json::object();
    summary.set("polls", mon->polls())
        .set("poll_rate_hz", rate ? Json(rate_1) : Json())
        .set("errors", mon->error_count())
        .set("link_checks", mon->link_checks())
        .set("link_state", state);
    emit("summary", summary,
         std::to_string(mon->polls()) + " polls (" + util::format("%g", rate_1) + " /s), " +
             std::to_string(mon->error_count()) + " errors, " + std::to_string(mon->link_checks()) +
             " link checks, link " + state);
    return code;
}

}  // namespace

int monitor(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err) {
    if (args.has("--watch")) return watch(axis, label, args, out, err);
    const int count = std::max(1, args.integer("--count").value_or(1));
    const double interval = std::max(0.0, args.number("--interval").value_or(0.5));
    for (int i = 0; i < count && !interrupted(); ++i) {
        if (i) nap(interval);
        if (interrupted()) break;
        const Json m = monitor_reading(axis);
        if (args.has("--json")) {
            out << m.dump() << '\n';
        } else {
            print_monitor(m, out);
        }
    }
    return 0;
}

int push(Axis& axis, const std::vector<std::string>& pos, const Args& args, std::ostream& out) {
    const double target = parse_number("push POS", pos.at(0));
    const double band = parse_number("push BAND", pos.at(1));
    const double pct = parse_number("push PCT", pos.at(2));
    PushOptions o;
    o.direction = args.has("--toward-home") ? -1 : 1;
    o.speed = args.number("--speed").value_or(10.0);
    const std::optional<PushResult> r = axis.push(target, band, pct, o);
    if (!r) return 0;
    if (args.has("--json")) {
        out << to_json(*r).dump(2) << '\n';
        return 0;
    }
    out << "push " << r->outcome << " at " << util::fixed(r->position_mm) << ' ' << axis.units() << '\n';
    for (const auto& w : r->warnings) out << "warning: " << w << '\n';
    text::status(r->status, out, axis.units());
    return 0;
}

std::optional<Status> move_and_change(Axis& axis, double target, const MoveOptions& options, const Args& args,
                                      std::ostream& err) {
    MoveOptions start = options;
    start.wait = false;
    axis.move_to(target, start);
    nap(args.number("--change-after").value_or(1.0));
    const std::optional<double> speed = args.number("--change-speed");
    const std::optional<double> accel = args.number("--change-accel");
    std::optional<Status> result;
    if (speed) {
        err << "changing the speed of the running move to " << util::format("%g", *speed) << ' ' << axis.units() << "/s\n";
        result = axis.change_speed(*speed, !accel);
    }
    if (accel) {
        err << "changing the acceleration of the running move to " << util::format("%g", *accel) << " G\n";
        result = axis.change_accel(*accel, true);
    }
    return result;
}

int backup(Axis& axis, const std::string& label, const std::vector<std::string>& pos, const Args& args,
           std::ostream& out) {
    const info::BackupRows rows = info::BackupRows::parse(args.value("--rows").value_or("used"));
    std::string safe = label;
    for (char& c : safe)
        if (!std::isalnum(static_cast<unsigned char>(c))) c = '-';
    const std::string path = pos.empty() ? "ec-backup-" + safe + "-s" + std::to_string(axis.slave()) + "-" + file_stamp() +
                                               ".json"
                                         : pos[0];
    const Json data = info::backup(axis, rows, path);
    if (args.has("--json")) {
        out << data.dump(2) << '\n';
        return 0;
    }
    const Json* meta = data.find("meta");
    const Json* parameters = data.find("parameters");
    std::size_t items = 0;
    if (const Json* classes = field(parameters, "classes"))
        if (classes->type() == Json::Type::Array)
            for (const Json& block : classes->as_array()) items += block.find("items") ? block.find("items")->size() : 0;
    const Json* named = field(parameters, "named");
    const Json* rows_json = field(data.find("positions"), "rows");
    const Json* history = field(data.find("alarms"), "history");
    out << "backup written to " << (field(meta, "path") ? pystr(field(meta, "path")) : path)
        << " (read-only: FC03 / FC11 only; " << pystr(field(meta, "duration_s")) << " s)\n";
    out << "contents   identity, directory, " << items << " parameter items (" << (named ? named->size() : 0)
        << " named), " << (rows_json ? rows_json->size() : 0) << " position rows, " << (history ? history->size() : 0)
        << " alarm entries, maintenance counters\n";
    if (const Json* errors = data.find("errors"))
        if (errors->type() == Json::Type::Array)
            for (const Json& e : errors->as_array()) out << "not read   " << pystr(&e) << '\n';
    return 0;
}

int recover(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err) {
    std::optional<Json> data;
    if (const auto file = args.value("--backup")) {
        try {
            data = info::load_backup(*file);
            recovery::session_from_backup(axis, *data, label);
        } catch (const EcDriverError&) {
            throw;
        } catch (const std::exception& e) {  // unreadable file, not JSON, no identity in it
            err << "error: backup " << *file << ": " << e.what() << '\n';
            return 2;
        }
    } else {
        recovery::forget(axis);
        recovery::session_from_info(axis, axis.identify(), label);
    }
    recovery::ControlChange change;
    change.modbus_control = !args.has("--no-control");
    change.baud_rate = args.integer("--restore-baud");
    recovery::note_control(axis, change);
    if (const auto target = args.number("--target")) recovery::note_target(axis, *target);
    recovery::RecoverOptions o;
    o.timeout_s = std::max(0.0, args.number("--wait").value_or(30.0));
    o.name = label;
    if (data) {
        o.policy.check_rows = recovery::RowSelection::every();
        o.policy.check_parameters = true;
        o.policy.backup = data;
    }
    const recovery::RecoveryReport report = recovery::recover_axis(axis, o);
    if (args.has("--json"))
        out << report.to_json().dump(2) << '\n';
    else
        text::recovery(report, axis.units(), out);
    return 0;
}

int line(Axis& axis, const Args& args, std::ostream& out) {
    const std::shared_ptr<ModbusRtu> bus = axis.bus();
    const std::shared_ptr<Line> ln = bus ? bus->line() : nullptr;
    if (!ln) throw EcNotReadyError("the axis has no bus");
    Json data = Json::object();
    Json errors = Json::array();
    Json gateway_status;
    Json monitor;
    const std::shared_ptr<Gateway> gateway = ln->gateway();
    if (gateway) {
        try {
            gateway_status = to_json(gateway->read_gateway_status());
        } catch (const ModbusError& e) {
            errors.push_back("gateway status: " + detail::error_type_name(e) + ": " + e.what());
        }
    }
    if (args.has("--monitor")) {
        const auto rcon = std::dynamic_pointer_cast<RconGateway>(gateway);
        if (!rcon) {
            errors.push_back("--monitor: the all-axes monitor exists on an RCON gateway only");
        } else {
            try {
                const auto records = rcon->read_monitor(std::vector<int>{axis.slave() - 1});
                const auto it = records.find(axis.slave() - 1);
                if (it != records.end()) {
                    monitor = to_json(it->second);
                    if (const auto unit = ln->position_unit_mm(axis.slave()))
                        monitor.set("position_mm", std::round(it->second.position_counts() * *unit * 1000.0) / 1000.0);
                }
            } catch (const ModbusError& e) {
                errors.push_back("gateway monitor: " + detail::error_type_name(e) + ": " + e.what());
            } catch (const std::invalid_argument& e) {
                errors.push_back(std::string("gateway monitor: ValueError: ") + e.what());
            }
        }
    }
    data.set("line", to_json(ln->snapshot())).set("gateway_status", gateway_status).set("monitor", monitor).set("errors", errors);
    if (args.has("--json")) {
        out << data.dump(2) << '\n';
        return errors.size() ? 1 : 0;
    }
    const LineRules& rules = ln->rules();
    out << "line       " << to_string(ln->kind()) << ": " << rules.title << (ln->detected() ? " (detected)" : "") << '\n';
    out << "evidence   " << rules.evidence << '\n';
    std::string codes = "any";
    if (rules.function_codes) {
        std::vector<std::string> hex;
        for (int c : *rules.function_codes) hex.push_back(util::format("%02X", c));
        codes = util::join(hex, " ");
    }
    auto yes = [](bool b) { return b ? "yes" : "NO"; };
    out << "carries    function codes " << codes << "; coils " << yes(rules.coils()) << "; IAI 0x45 / 0x48 "
        << yes(rules.iai_commands) << "; broadcast " << yes(rules.broadcast) << "; Report Slave ID "
        << yes(rules.report_slave_id()) << '\n';
    out << "strategy   " << rules.command_strategy.value_or("the axis chooses (coil by default)") << "; stop path "
        << axis.stop_path() << '\n';
    const auto unit = ln->position_unit_mm(axis.slave());
    const auto source = ln->position_unit_source(axis.slave());
    out << "pos. unit  "
        << (unit ? util::format("%g", *unit) + " mm (" + source.value_or("unknown") + ")"
                 : std::string("NOT STATED (pass --position-unit 0.01|0.001)"))
        << '\n';
    if (!gateway_status.is_null()) {
        const bool manu = truthy(gateway_status.find("manu"));
        const Json* code = gateway_status.find("alarm_code");
        std::string alarm;
        if (truthy(code))
            alarm = util::format(", alarm 0x%llx ", static_cast<unsigned long long>(code->as_int())) +
                    pystr(gateway_status.find("alarm_name"));
        out << "gateway    " << gateway->to_string() << ": " << (manu ? "MANU" : "AUTO (Modbus control is refused)")
            << ", linked axes " << pystr(gateway_status.find("linked_axes")) << alarm << '\n';
    }
    if (!monitor.is_null()) {
        const std::string where = monitor.find("position_mm")
                                      ? util::format("%.3f mm", monitor.find("position_mm")->as_number())
                                      : pystr(monitor.find("position_counts")) + " counts";
        out << "monitor    axis " << pystr(monitor.find("axis")) << ": position " << where << ", current "
            << pystr(monitor.find("current_ma")) << " mA, alarm "
            << util::format("0x%x", static_cast<unsigned>(num(monitor.find("alarm_code")))) << ", DSS1 "
            << util::format("0x%04x", static_cast<unsigned>(num(monitor.find("dss1")))) << ", DSSE "
            << util::format("0x%04x", static_cast<unsigned>(num(monitor.find("dsse")))) << '\n';
    }
    if (const auto note = ln->explain()) out << "note       " << *note << '\n';
    for (const Json& e : errors.as_array()) out << "error      " << e.as_string() << '\n';
    return errors.size() ? 1 : 0;
}

int check(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err) {
    out << kBanner << '\n';
    out << "compatibility check on " << label << ", slave " << axis.slave() << ", strategy "
        << to_string(axis.strategy()) << '\n';
    CheckOptions o;
    o.motion = args.has("--motion");
    o.port_label = label;
    o.on_item = [&out](const CheckItem& item) { out << format_check_item(item) << '\n'; };
    const CheckReport report = run_check(axis, o);
    const std::string path = args.value("--report").value_or("iai-check-" + file_stamp() + ".json");
    const std::string where = save_check_report(report, path, out, err);
    out << "RESULT " << (report.passed() ? "PASS" : "FAIL") << " - " << where << '\n';
    return report.passed() ? 0 : 1;
}

}  // namespace extra

}  // namespace cli
}  // namespace iai_actuator_driver
