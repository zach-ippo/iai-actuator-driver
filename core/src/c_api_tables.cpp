// iad_call_json: the core's stateless functions by name, with JSON in and out (the "A5" entry of the binding plan): the
// alarm tables and rules (alarms.hpp), the CON parameter tables and rules (con_parameters.cpp), the pure decoders of
// info.hpp and the status decode (Status::from_registers). A binding calls these instead of keeping its own copy of the tables and rules, so every edition decides
// the same way; its conformance tests compare its old tables with these outputs.
//
// Values keep their Python types where the Python edition's results depend on them: an integer is a JSON integer, a
// float a JSON number written with a decimal point or an exponent ("1.0", "0.01"), so a binding reads 250 and 250.0
// apart. A value the binding hands in for a CON parameter may be a JSON integer, number, string, true / false or null,
// or {"float": "inf" | "-inf" | "nan"} / {"repr": "<text>"} for a value JSON cannot carry; the messages name it as
// Python's repr() does.
//
// Exported from the shared library without a header declaration until the header owner adds it:
//   iad_result iad_call_json(const char* function, const char* args_json, char* buffer, size_t buffer_size,
//                            size_t* required);
// `args_json` is an object of named arguments (NULL or "" = {}), or an array of such objects: then the output is the
// array of the results, in order. An unknown function is IAD_ERR_UNSUPPORTED (std::logic_error), so a binding can ask
// what a library lacks; "functions" lists the names. Bad arguments are IAD_ERR_INVALID_ARGUMENT.
#include "iai_actuator_driver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/status.hpp"
#include "model_detail.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

// ============================================================================================ JSON with Python types

/// Json::dump with every number as Python writes it: an integer as an integer, a float with its shortest repr.
void write_py(std::string& out, const Json& j);

void quote(std::string& out, const std::string& s) {
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) out += util::format("\\u%04x", c);
                else out += static_cast<char>(c);
        }
    }
    out += '"';
}

void write_py(std::string& out, const Json& j) {
    switch (j.type()) {
        case Json::Type::Null: out += "null"; break;
        case Json::Type::Bool: out += j.as_bool() ? "true" : "false"; break;
        case Json::Type::Int: out += std::to_string(j.as_int()); break;
        case Json::Type::Number: out += util::py_float(j.as_number()); break;  // finite: Json keeps no NaN / inf
        case Json::Type::String: quote(out, j.as_string()); break;
        case Json::Type::Array: {
            out += '[';
            bool first = true;
            for (const Json& v : j.as_array()) {
                if (!first) out += ',';
                first = false;
                write_py(out, v);
            }
            out += ']';
            break;
        }
        case Json::Type::Object: {
            out += '{';
            bool first = true;
            for (const auto& kv : j.as_object()) {
                if (!first) out += ',';
                first = false;
                quote(out, kv.first);
                out += ':';
                write_py(out, kv.second);
            }
            out += '}';
            break;
        }
    }
}

/// A whole value as a JSON integer unless it is a decimal; any other value as a JSON number.
Json number_json(double value, bool decimal) {
    if (!decimal && std::isfinite(value) && value == std::floor(value) && std::fabs(value) < 9.0e15)
        return Json(static_cast<long long>(value));
    return Json(value);
}

Json number_json(const std::optional<double>& value, bool decimal) {
    return value ? number_json(*value, decimal) : Json();
}

// ============================================================================================ arguments

const Json& arg(const Json& args, const char* key) {
    static const Json kNull;
    const Json* found = args.find(key);
    return found ? *found : kNull;
}

[[noreturn]] void bad(const char* key, const char* what) {
    throw std::invalid_argument(std::string("iad_call_json: argument '") + key + "' must be " + what);
}

std::optional<long long> opt_int(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    switch (v.type()) {
        case Json::Type::Null: return std::nullopt;
        case Json::Type::Bool: return v.as_bool() ? 1 : 0;
        case Json::Type::Int: return v.as_int();
        case Json::Type::Number: {
            // a whole float in the long long range only: converting one outside it is undefined (1e300)
            const double d = v.as_number();
            if (d == std::floor(d) && d >= -9223372036854775808.0 && d < 9223372036854775808.0)
                return static_cast<long long>(d);
            break;
        }
        default: break;
    }
    bad(key, "an integer (in the 64-bit range) or null");
}

long long need_int(const Json& args, const char* key) {
    const std::optional<long long> v = opt_int(args, key);
    if (!v) bad(key, "an integer");
    return *v;
}

/// An integer for an int field: one outside the 32-bit range is refused, never narrowed (4294967553 is not 0x101).
int int32_of(long long value, const char* key) {
    if (value < INT32_MIN || value > INT32_MAX) bad(key, "an integer in the 32-bit range");
    return static_cast<int>(value);
}

std::optional<int> opt_int32(const Json& a, const char* key) {
    const std::optional<long long> v = opt_int(a, key);
    return v ? std::optional<int>(int32_of(*v, key)) : std::nullopt;
}

int need_int32(const Json& a, const char* key) { return int32_of(need_int(a, key), key); }

/// An unsigned 32-bit count (encoder pulses): outside 0 .. 2^32 - 1 refused.
std::optional<std::uint32_t> opt_count32(const Json& a, const char* key) {
    const std::optional<long long> v = opt_int(a, key);
    if (!v) return std::nullopt;
    if (*v < 0 || *v > 0xFFFFFFFFll) bad(key, "an integer in 0 .. 4294967295 or null");
    return static_cast<std::uint32_t>(*v);
}

std::optional<double> opt_number(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    if (v.is_null()) return std::nullopt;
    if (v.type() == Json::Type::Int || v.type() == Json::Type::Number) return v.as_number();
    if (v.type() == Json::Type::Bool) return v.as_bool() ? 1.0 : 0.0;
    bad(key, "a number or null");
}

/// Python truthiness of a flag argument (null = None).
std::optional<bool> opt_bool(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    switch (v.type()) {
        case Json::Type::Null: return std::nullopt;
        case Json::Type::Bool: return v.as_bool();
        case Json::Type::Int: return v.as_int() != 0;
        case Json::Type::Number: return v.as_number() != 0.0;
        default: break;
    }
    bad(key, "true, false or null");
}

std::optional<std::string> opt_string(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    if (v.is_null()) return std::nullopt;
    if (v.type() == Json::Type::String) return v.as_string();
    bad(key, "a string or null");
}

std::vector<std::uint16_t> words_arg(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    if (v.type() != Json::Type::Array) bad(key, "an array of 16-bit words");
    std::vector<std::uint16_t> out;
    for (const Json& w : v.as_array()) {
        if (w.type() != Json::Type::Int || w.as_int() < 0 || w.as_int() > 0xFFFF) bad(key, "an array of 16-bit words");
        out.push_back(static_cast<std::uint16_t>(w.as_int()));
    }
    return out;
}

std::optional<std::uint32_t> opt_u32(const Json& args, const char* key) {
    const std::optional<long long> v = opt_int(args, key);
    if (!v) return std::nullopt;
    return static_cast<std::uint32_t>(*v & 0xFFFFFFFFll);
}

/// A name -> word mapping (the class-1 support words); null entries are left out (Python: "not read").
std::optional<std::map<std::string, std::uint32_t>> support_arg(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    if (v.is_null()) return std::nullopt;
    if (v.type() != Json::Type::Object) bad(key, "an object of words or null");
    std::map<std::string, std::uint32_t> out;
    for (const auto& kv : v.as_object()) {
        if (kv.second.is_null()) continue;
        if (kv.second.type() != Json::Type::Int) bad(key, "an object of words or null");
        out[kv.first] = static_cast<std::uint32_t>(kv.second.as_int() & 0xFFFFFFFFll);
    }
    return out;
}

/// The CON family of a family argument (con_params.con_family): a family name or an FC11 system code.
std::optional<std::string> con_family_arg(const Json& args, const char* key) {
    const Json& v = arg(args, key);
    switch (v.type()) {
        case Json::Type::Int:
            if (v.as_int() < INT32_MIN || v.as_int() > INT32_MAX) return std::nullopt;  // no such system code
            return con_detail::family_of(reg::product_family(static_cast<int>(v.as_int())));
        case Json::Type::String: return con_detail::family_of(v.as_string());
        default: return std::nullopt;  // None, a bool, anything else
    }
}

// ============================================================================================ a value with its Python type

struct PyValue {
    enum class Kind { None, Bool, Int, Float, Str, Other } kind = Kind::None;
    double number = 0.0;
    bool flag = false;
    std::string text;  ///< Str: the text; Other: its repr
};

PyValue py_value(const Json& j) {
    PyValue v;
    switch (j.type()) {
        case Json::Type::Null: break;
        case Json::Type::Bool:
            v.kind = PyValue::Kind::Bool;
            v.flag = j.as_bool();
            v.number = v.flag ? 1.0 : 0.0;
            break;
        case Json::Type::Int:
            v.kind = PyValue::Kind::Int;
            v.number = static_cast<double>(j.as_int());
            break;
        case Json::Type::Number:
            v.kind = PyValue::Kind::Float;
            v.number = j.as_number();
            break;
        case Json::Type::String:
            v.kind = PyValue::Kind::Str;
            v.text = j.as_string();
            break;
        case Json::Type::Object: {
            if (const Json* f = j.find("float")) {
                v.kind = PyValue::Kind::Float;
                const std::string t = f->type() == Json::Type::String ? f->as_string() : "nan";
                v.number = t == "inf" ? HUGE_VAL : t == "-inf" ? -HUGE_VAL : std::nan("");
                break;
            }
            v.kind = PyValue::Kind::Other;
            const Json* r = j.find("repr");
            v.text = r && r->type() == Json::Type::String ? r->as_string() : "<object>";
            break;
        }
        default:
            v.kind = PyValue::Kind::Other;
            v.text = "<object>";
    }
    return v;
}

/// Python's repr() of a str.
std::string str_repr(const std::string& s) {
    const bool single = s.find('\'') != std::string::npos && s.find('"') == std::string::npos;
    const char q = single ? '"' : '\'';
    std::string out(1, q);
    for (char c : s) {
        if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else if (c == q) out += std::string("\\") + q;
        else out += c;
    }
    return out + q;
}

std::string repr(const PyValue& v) {
    switch (v.kind) {
        case PyValue::Kind::None: return "None";
        case PyValue::Kind::Bool: return v.flag ? "True" : "False";
        case PyValue::Kind::Int: return util::py_number(v.number, false);
        case PyValue::Kind::Float: return util::py_float(v.number);
        case PyValue::Kind::Str: return str_repr(v.text);
        case PyValue::Kind::Other: break;
    }
    return v.text;
}

ParamValue param_value(const PyValue& v) {
    if (v.kind == PyValue::Kind::Str) {
        ParamValue p;
        p.text = v.text;
        return p;
    }
    return ParamValue::of(v.number);
}

// ============================================================================================ CON parameters

ConParam con_param_arg(const Json& args, const char* key) {
    const Json& j = arg(args, key);
    if (j.type() != Json::Type::Object) bad(key, "a CON parameter object");
    ConParam p;
    p.no = need_int32(j, "no");
    p.name = opt_string(j, "name").value_or("");
    p.symbol = opt_string(j, "symbol").value_or("");
    p.class_id = need_int32(j, "class_id");
    p.item_id = need_int32(j, "item_id");
    p.kind = opt_string(j, "kind").value_or("");
    p.entry = opt_int32(j, "entry").value_or(0);
    p.scale = opt_number(j, "scale").value_or(1.0);
    p.is_signed = opt_bool(j, "signed").value_or(false);
    const Json& bits = arg(j, "bits");
    if (bits.type() == Json::Type::Array) {
        for (const Json& b : bits.as_array()) {
            if (b.type() != Json::Type::Int || b.as_int() < 0 || b.as_int() > 31)
                bad("bits", "a list of bit numbers 0..31");
            p.bits.push_back(static_cast<int>(b.as_int()));
        }
    }
    p.unit = opt_string(j, "unit").value_or("");
    p.lo = opt_number(j, "lo");
    p.hi = opt_number(j, "hi");
    p.lo_decimal = arg(j, "lo").type() == Json::Type::Number;
    p.hi_decimal = arg(j, "hi").type() == Json::Type::Number;
    const Json& choices = arg(j, "choices");
    if (choices.type() == Json::Type::Array) {
        for (const Json& c : choices.as_array()) {
            if (c.type() != Json::Type::Array || c.size() != 2 || c.as_array()[0].type() != Json::Type::Int ||
                c.as_array()[1].type() != Json::Type::String)
                bad("choices", "[integer value, label text] pairs");
            p.choices[int32_of(c.as_array()[0].as_int(), "choices")] = c.as_array()[1].as_string();
        }
    }
    p.dangerous = opt_bool(j, "dangerous").value_or(false);
    p.writable = opt_bool(j, "writable").value_or(true);
    return p;
}

Json con_param_json(const ConParam& p) {
    Json out = Json::object();
    out.set("no", p.no);
    out.set("name", p.name);
    out.set("symbol", p.symbol);
    out.set("class_id", p.class_id);
    out.set("item_id", p.item_id);
    out.set("kind", p.kind);
    out.set("entry", p.entry);
    out.set("scale", p.scale);  // a float in the Python edition's table (1.0, 0.01)
    out.set("signed", p.is_signed);
    out.set("bits", Json(p.bits));
    out.set("unit", p.unit);
    out.set("lo", number_json(p.lo, p.lo_decimal));
    out.set("hi", number_json(p.hi, p.hi_decimal));
    Json choices = Json::array();
    for (const auto& kv : p.choices) {
        Json pair = Json::array();
        pair.push_back(kv.first);
        pair.push_back(kv.second);
        choices.push_back(pair);
    }
    out.set("choices", choices);
    out.set("dangerous", p.dangerous);
    out.set("writable", p.writable);
    out.set("note", con_detail::note_of(p.no));
    out.set("key", p.key());
    return out;
}

/// A decoded / parsed value as the Python edition returns it: a float for a scaled parameter, text for an address,
/// an int otherwise, None for no value.
Json typed_value(const ConParam& p, const ParamValue& v) {
    if (v.empty()) return Json();
    if (!v.number) return Json(v.text);
    if (p.kind == "scaled") return Json(*v.number);
    return Json(static_cast<long long>(*v.number));
}

/// con_params.parse for a value with its Python type.
ParamValue parse_value(const ConParam& p, const PyValue& v) {
    if (v.kind == PyValue::Kind::Bool)
        throw std::invalid_argument(util::format("No.%d: pass a number or a label, not %s", p.no, repr(v).c_str()));
    if (p.kind == "ipv4" && v.kind != PyValue::Kind::Int && v.kind != PyValue::Kind::Str)
        throw std::invalid_argument(util::format("No.%d: %s is not an IPv4 address", p.no, repr(v).c_str()));
    if (v.kind == PyValue::Kind::None || v.kind == PyValue::Kind::Other ||
        (v.kind == PyValue::Kind::Float && !std::isfinite(v.number)))
        throw std::invalid_argument(util::format("No.%d: %s is not a finite number", p.no, repr(v).c_str()));
    return con_detail::parse(p, param_value(v));
}

con_detail::UnitLimits unit_arg(const Json& args, const char* key) {
    const Json& j = arg(args, key);
    con_detail::UnitLimits u;
    if (j.is_null()) return u;
    if (j.type() != Json::Type::Object) bad(key, "a unit-limits object or null");
    u.max_speed = opt_number(j, "max_speed");
    u.max_accel = opt_number(j, "max_accel");
    u.max_push_speed = opt_number(j, "max_push_speed");
    u.lead_mm = opt_number(j, "lead_mm");
    if (const std::optional<std::uint32_t> pulses = opt_count32(j, "encoder_pulses")) u.encoder_pulses = *pulses;
    u.min_band_mm = opt_number(j, "min_band_mm");
    u.rotary = opt_bool(j, "rotary").value_or(false);
    return u;
}

Json unit_json(const con_detail::UnitLimits& u) {
    Json out = Json::object();
    out.set("max_speed", u.max_speed);
    out.set("max_accel", u.max_accel);
    out.set("max_push_speed", u.max_push_speed);
    out.set("lead_mm", u.lead_mm);
    out.set("encoder_pulses", u.encoder_pulses);
    out.set("min_band_mm", u.min_band_mm);
    out.set("rotary", u.rotary);
    return out;
}

Json default_json(const std::optional<con_detail::Number>& n) {
    return n ? number_json(n->value, n->decimal) : Json();
}

/// One class block from its JSON (info.ClassBlock: class_id, address, words, member_size, entries, id_map_address and
/// the items with class_id, item_id, entry, index, address, size, raw).
ClassBlock block_from(const Json& j) {
    if (j.type() != Json::Type::Object) throw std::invalid_argument("iad_call_json: a class block must be an object");
    ClassBlock b;
    b.class_id = opt_int32(j, "class_id").value_or(0);
    b.address = opt_int32(j, "address").value_or(0);
    b.words = opt_int32(j, "words").value_or(0);
    b.member_size = opt_int32(j, "member_size").value_or(0);
    b.entries = opt_int32(j, "entries").value_or(0);
    const std::optional<int> map = opt_int32(j, "id_map_address");
    if (map) b.id_map_address = *map;
    const Json& items = arg(j, "items");
    if (items.type() == Json::Type::Array) {
        for (const Json& i : items.as_array()) {
            ClassItem it;
            it.class_id = opt_int32(i, "class_id").value_or(b.class_id);
            const std::optional<int> id = opt_int32(i, "item_id");
            if (id) it.item_id = *id;
            it.entry = opt_int32(i, "entry").value_or(0);
            it.index = opt_int32(i, "index").value_or(0);
            it.address = opt_int32(i, "address").value_or(0);
            it.size = opt_int32(i, "size").value_or(1);
            it.raw = static_cast<std::uint32_t>(opt_int(i, "raw").value_or(0) & 0xFFFFFFFFll);
            b.items.push_back(it);
        }
    }
    return b;
}

IndexCategory record_from(const Json& j) {
    if (j.type() != Json::Type::Object) throw std::invalid_argument("iad_call_json: a directory record must be an object");
    IndexCategory c;
    c.category_id = opt_int32(j, "category_id").value_or(0);
    c.model_code = opt_int32(j, "model_code").value_or(0);
    c.member_count = opt_int32(j, "member_count").value_or(0);
    c.list_count = opt_int32(j, "list_count").value_or(0);
    c.data_address = opt_int32(j, "data_address").value_or(0);
    c.id_list_address = opt_int32(j, "id_list_address").value_or(0);
    c.coil_base = opt_int32(j, "coil_base").value_or(0);
    c.member_size = opt_int32(j, "member_size").value_or(0);
    return c;
}

Json bit_def_json(const info::BitDef& d) {
    Json out = Json::object();
    out.set("bit", d.bit);
    out.set("name", d.name);
    out.set("meaning", d.meaning);
    out.set("confirmed", d.confirmed);
    out.set("evidence", d.evidence);
    out.set("active_low", d.active_low);
    return out;
}

/// The name of a bit table (bit_table_names) by identity, "" when it is none of them.
std::string table_name(const std::vector<info::BitDef>* table) {
    for (const std::string& name : info::bit_table_names())
        if (&info::bit_table(name) == table) return name;
    return std::string();
}

template <class T>
Json pairs_json(const std::vector<std::pair<int, T>>& pairs) {
    Json out = Json::array();
    for (const auto& kv : pairs) {
        Json pair = Json::array();
        pair.push_back(kv.first);
        pair.push_back(Json(kv.second));
        out.push_back(pair);
    }
    return out;
}

Json int_list(const std::vector<int>& values) { return Json(values); }

// ============================================================================================ the functions

using Function = std::function<Json(const Json&)>;

/// alarms.alarm_level: the CON-only message codes need the family of system_code / family.
AlarmLevel level_for(const Json& a, int code) {
    const std::optional<std::string> con = alarm_family(opt_int32(a, "system_code"), opt_string(a, "family"));
    return alarm_level(code, opt_bool(a, "absolute_encoder"), con_system_code(con));
}

Json alarms_tables() {
    Json out = Json::object();
    for (const char* name : {"controller_names", "curated_names", "con_names", "groups", "home_needed",
                             "warning_clearing", "abs_error_details", "row_fields", "command_fields",
                             "specific_remedies"})
        out.set(name, pairs_json(alarm_table(name)));
    Json groups = Json::array();
    for (const auto& kv : alarm_group_table("group_names")) {
        Json pair = Json::array();
        pair.push_back(kv.first);
        pair.push_back(kv.second);
        groups.push_back(pair);
    }
    out.set("group_names", groups);
    Json advice = Json::array();
    for (const auto& kv : alarm_group_table("group_advice")) {
        Json pair = Json::array();
        pair.push_back(kv.first);
        pair.push_back(kv.second);
        advice.push_back(pair);
    }
    out.set("group_advice", advice);
    Json clearing = Json::array();
    for (const auto& t : alarm_family_clearing_table()) {
        Json row = Json::array();
        row.push_back(std::get<0>(t));
        row.push_back(std::get<1>(t));
        row.push_back(std::get<2>(t));
        clearing.push_back(row);
    }
    out.set("family_clearing", clearing);
    out.set("family_clearing_any", alarm_family_clearing_any());
    // the level rules as code sets: message level on every unit, and on a CON unit only
    Json message = Json::array();
    Json con_message = Json::array();
    for (int code = 0x011; code <= 0x07F; ++code) {
        if (alarm_level(code) == AlarmLevel::Message) message.push_back(code);
        else if (alarm_level(code, std::nullopt, 0xAE) == AlarmLevel::Message) con_message.push_back(code);
    }
    out.set("message_codes", message);
    out.set("con_message_codes", con_message);
    Json cancelled = Json::array();
    for (int code = 0x080; code <= 0x0FF; ++code)
        if (code != 0x0EE && alarm_level(code) == AlarmLevel::OperationCancelled) cancelled.push_back(code);
    out.set("operation_cancelled_codes", cancelled);
    Json systems = Json::array();
    for (int code = 0; code <= 0xFF; ++code)
        if (is_con_system_code(code)) systems.push_back(code);
    out.set("con_system_codes", systems);
    Json levels = Json::array();
    for (AlarmLevel level : {AlarmLevel::None, AlarmLevel::Message, AlarmLevel::OperationCancelled,
                             AlarmLevel::ColdStart, AlarmLevel::Unknown}) {
        Json pair = Json::array();
        pair.push_back(level_key(level));
        pair.push_back(level_text(level));
        levels.push_back(pair);
    }
    out.set("level_text", levels);
    Json fault = Json::array();
    fault.push_back(level_key(AlarmLevel::OperationCancelled));
    fault.push_back(level_key(AlarmLevel::ColdStart));
    out.set("fault_levels", fault);
    out.set("power_up_code", reg::kAlarmPowerUp);
    out.set("history_stride", reg::kAlarmHistoryStride);
    out.set("rtc_epoch", info::alarm_time(0, true).find("occurred_at")->as_string());
    return out;
}

Json con_tables() {
    Json out = Json::object();
    Json params = Json::array();
    for (const ConParam& p : con_parameters()) params.push_back(con_param_json(p));
    out.set("parameters", params);
    out.set("baud_codes", pairs_json(con_detail::baud_codes()));
    out.set("con_baud_rates", int_list(con_detail::con_baud_rates()));
    out.set("families", Json(con_detail::families()));
    Json ranges = Json::object();
    Json docs = Json::object();
    Json family_categories = Json::object();
    Json family_defaults = Json::object();
    for (const std::string& f : con_detail::families()) {
        Json rows = Json::array();
        for (const auto& r : con_detail::family_ranges(f)) {
            Json row = Json::array();
            row.push_back(std::get<0>(r));
            row.push_back(number_json(std::get<1>(r), false));
            row.push_back(number_json(std::get<2>(r), false));
            rows.push_back(row);
        }
        ranges.set(f, rows);
        docs.set(f, con_detail::range_doc(f));
        const auto cats = con_detail::categories(f);
        if (!cats.empty()) family_categories.set(f, pairs_json(cats));
        Json defaults = Json::array();
        for (const auto& kv : con_detail::factory_defaults(f)) {
            Json pair = Json::array();
            pair.push_back(kv.first);
            pair.push_back(number_json(kv.second.value, kv.second.decimal));
            defaults.push_back(pair);
        }
        if (defaults.size()) family_defaults.set(f, defaults);
    }
    out.set("family_ranges", ranges);
    out.set("family_range_doc", docs);
    out.set("unknown_family_note", con_detail::unknown_family_note());
    out.set("categories", pairs_json(con_detail::categories(std::nullopt)));
    out.set("family_categories", family_categories);
    Json texts = Json::object();
    for (const char* c : {"A", "B", "C", "D", "E"}) texts.set(c, con_detail::category_text(c));
    out.set("category_text", texts);
    Json defaults = Json::array();
    for (const auto& kv : con_detail::factory_defaults(std::nullopt)) {
        Json pair = Json::array();
        pair.push_back(kv.first);
        pair.push_back(number_json(kv.second.value, kv.second.decimal));
        defaults.push_back(pair);
    }
    out.set("factory_defaults", defaults);
    out.set("family_defaults", family_defaults);
    Json home = Json::array();
    for (int no = 0; no < 1000; ++no)
        if (is_con_home_parameter(no)) home.push_back(no);
    out.set("home_parameters", home);
    return out;
}

Json con_check(const Json& a) {
    const ConParam p = con_param_arg(a, "p");
    const std::optional<std::string> fam = con_family_arg(a, "family");
    const ConParam q = con_detail::for_family(p, fam.value_or(""));
    const PyValue v = py_value(arg(a, "value"));
    parse_value(q, v);  // the Python types' own refusals first (a bool, None, inf...)
    const con_detail::Check c = con_detail::check_value(p, param_value(v), fam.value_or(""), unit_arg(a, "unit"));
    Json out = Json::object();
    out.set("value", typed_value(q, c.value));
    out.set("warnings", Json(c.warnings));
    out.set("family", c.family);
    out.set("lo", number_json(q.lo, q.lo_decimal));
    out.set("hi", number_json(q.hi, q.hi_decimal));
    return out;
}

const std::map<std::string, Function>& functions() {
    static const std::map<std::string, Function> table = {
        // ---------------------------------------------------------------- alarms
        {"alarms.tables", [](const Json&) { return alarms_tables(); }},
        {"alarms.alarm_name", [](const Json& a) { return Json(alarm_name(need_int32(a, "code"))); }},
        {"alarms.alarm_family",
         [](const Json& a) { return Json(alarm_family(opt_int32(a, "system_code"), opt_string(a, "family"))); }},
        {"alarms.alarm_level",
         [](const Json& a) {
             const int code = need_int32(a, "code");
             return Json(level_key(level_for(a, code)));
         }},
        {"alarms.alarm_is_fault",
         [](const Json& a) {
             const std::optional<std::string> con = alarm_family(opt_int32(a, "system_code"), opt_string(a, "family"));
             return Json(alarm_is_fault(need_int32(a, "code"), opt_bool(a, "alarm_bit").value_or(false),
                                        opt_bool(a, "warning_bit").value_or(false), con_system_code(con)));
         }},
        {"alarms.alarm_group", [](const Json& a) { return Json(alarm_group(need_int32(a, "code"))); }},
        {"alarms.alarm_info",
         [](const Json& a) {
             return to_json(alarm_info(need_int32(a, "code"), opt_bool(a, "absolute_encoder"),
                                       opt_int32(a, "detail"), opt_int32(a, "address"), opt_int32(a, "system_code"),
                                       opt_string(a, "family")));
         }},
        {"alarms.describe_alarm_detail",
         [](const Json& a) {
             return Json(describe_alarm_detail(need_int32(a, "code"), opt_int32(a, "detail"),
                                               opt_int32(a, "address")));
         }},
        {"alarms.absolute_encoder_from_support",
         [](const Json& a) {
             const auto support = support_arg(a, "support");
             return Json(absolute_encoder_from_support(support ? &*support : nullptr));
         }},
        {"alarms.alarm_history_slots",
         [](const Json& a) {
             Json out = Json::array();
             for (const AlarmRecord& r : alarm_history_slots(words_arg(a, "words"),
                                                              opt_bool(a, "include_empty").value_or(false))) {
                 Json row = Json::array();
                 row.push_back(r.index);
                 row.push_back(r.code);
                 row.push_back(r.detail);
                 row.push_back(r.address);
                 row.push_back(Json(r.time_ms));
                 out.push_back(row);
             }
             return out;
         }},
        {"alarms.alarm_time",
         [](const Json& a) { return info::alarm_time(opt_int(a, "time_raw"), opt_bool(a, "rtc_in_use")); }},
        {"alarms.rtc_datetime",
         [](const Json& a) { return *info::alarm_time(need_int(a, "seconds"), true).find("occurred_at"); }},
        // ---------------------------------------------------------------- CON parameters
        {"con_params.tables", [](const Json&) { return con_tables(); }},
        {"con_params.con_family", [](const Json& a) { return Json(con_family_arg(a, "family")); }},
        {"con_params.for_family",
         [](const Json& a) {
             const ConParam p = con_param_arg(a, "p");
             const std::optional<std::string> fam = con_family_arg(a, "family");
             return con_param_json(fam ? con_detail::for_family(p, *fam) : p);
         }},
        {"con_params.con_parameters",
         [](const Json& a) {
             Json out = Json::array();
             for (const ConParam& p : con_parameters(con_family_arg(a, "family").value_or("")))
                 out.push_back(con_param_json(p));
             return out;
         }},
        {"con_params.category",
         [](const Json& a) {
             return Json(con_detail::category(need_int32(a, "no"), con_family_arg(a, "family")));
         }},
        {"con_params.factory_default",
         [](const Json& a) {
             return default_json(
                 con_detail::factory_default(need_int32(a, "no"), con_family_arg(a, "family")));
         }},
        {"con_params.con_unit_limits",
         [](const Json& a) {
             std::map<int, ClassBlock> classes;
             const Json& blocks = arg(a, "classes");
             if (blocks.type() == Json::Type::Array)
                 for (const Json& b : blocks.as_array()) {
                     const ClassBlock block = block_from(b);
                     classes[block.class_id] = block;
                 }
             return unit_json(con_detail::unit_limits(classes, support_arg(a, "support")));
         }},
        {"con_params.min_positioning_band",
         [](const Json& a) {
             const auto band = con_detail::min_band(con_family_arg(a, "family"), opt_number(a, "lead_mm"),
                                                    opt_count32(a, "encoder_pulses"), opt_number(a, "mini_mm"));
             if (!band) return Json();
             Json out = Json::object();
             out.set("mm", band->mm);
             out.set("formula", band->formula);
             out.set("doc", band->doc);
             out.set("certain", band->certain);
             return out;
         }},
        {"con_params.check_value", [](const Json& a) { return con_check(a); }},
        {"con_params.audit",
         [](const Json& a) {
             const std::optional<std::string> fam = con_family_arg(a, "family");
             const con_detail::UnitLimits unit = unit_arg(a, "unit");
             std::vector<std::pair<long long, PyValue>> values;
             const Json& given = arg(a, "values");
             if (given.type() == Json::Type::Array)
                 for (const Json& pair : given.as_array()) {
                     if (pair.type() != Json::Type::Array || pair.size() != 2 ||
                         pair.as_array()[0].type() != Json::Type::Int)
                         bad("values", "[number, value] pairs");
                     values.emplace_back(pair.as_array()[0].as_int(), py_value(pair.as_array()[1]));
                 }
             std::stable_sort(values.begin(), values.end(),
                              [](const auto& x, const auto& y) { return x.first < y.first; });
             Json findings = Json::array();
             for (const auto& kv : values) {
                 // a number outside the int range is no parameter (as in the Python edition), never a narrowed one
                 const bool in_range = kv.first >= INT32_MIN && kv.first <= INT32_MAX;
                 const ConParam* p = in_range ? con_parameter(static_cast<int>(kv.first)) : nullptr;
                 if (p == nullptr || kv.second.kind == PyValue::Kind::None || !p->writable) continue;
                 try {
                     const ConParam q = con_detail::for_family(*p, fam.value_or(""));
                     parse_value(q, kv.second);
                     con_detail::check_value(*p, param_value(kv.second), fam.value_or(""), unit);
                 } catch (const std::invalid_argument& e) {
                     Json f = Json::object();
                     f.set("number", kv.first);
                     f.set("problem", e.what());
                     findings.push_back(f);
                 }
             }
             return findings;
         }},
        {"con_params.parameter_table",
         [](const Json& a) { return info::con_parameter_table(con_family_arg(a, "family")); }},  // (0.6) info.cpp
        {"con_params.decode",
         [](const Json& a) {
             const ConParam p = con_param_arg(a, "p");
             return typed_value(p, con_detail::decode(p, static_cast<std::uint32_t>(need_int(a, "raw") & 0xFFFFFFFFll)));
         }},
        {"con_params.encode",
         [](const Json& a) {
             const ConParam p = con_param_arg(a, "p");
             const PyValue v = py_value(arg(a, "value"));
             return Json(static_cast<long long>(con_detail::encode(
                 p, static_cast<std::uint32_t>(need_int(a, "raw") & 0xFFFFFFFFll), param_value(v))));
         }},
        {"con_params.text",
         [](const Json& a) {
             const ConParam p = con_param_arg(a, "p");
             const PyValue v = py_value(arg(a, "value"));
             const std::string unit = p.unit.empty() ? std::string() : " " + p.unit;
             if (v.kind == PyValue::Kind::None) return Json("not set");
             if (v.kind == PyValue::Kind::Float && p.kind != "scaled") return Json(util::py_float(v.number) + unit);
             if (v.kind == PyValue::Kind::Other) return Json(v.text + unit);
             return Json(con_detail::text(p, param_value(v)));
         }},
        {"con_params.parse",
         [](const Json& a) {
             ConParam p = con_param_arg(a, "p");
             const Json& family = arg(a, "family");
             if (!family.is_null()) {
                 const std::optional<std::string> fam = con_family_arg(a, "family");
                 if (fam) p = con_detail::for_family(p, *fam);
             }
             return typed_value(p, parse_value(p, py_value(arg(a, "value"))));
         }},
        // ---------------------------------------------------------------- info decoders
        {"info.bit_tables",
         [](const Json&) {
             Json out = Json::object();
             for (const std::string& name : info::bit_table_names()) {
                 Json defs = Json::array();
                 for (const info::BitDef& d : info::bit_table(name)) defs.push_back(bit_def_json(d));
                 out.set(name, defs);
             }
             return out;
         }},
        {"info.decode_bits",
         [](const Json& a) {
             const long long word = need_int(a, "word");
             const int width = opt_int32(a, "width").value_or(16);
             if (word < 0 || width < 1 || width > 32 || word >= (1ll << width))
                 throw std::invalid_argument(std::to_string(word) + " does not fit in " + std::to_string(width) +
                                             " bits");
             const Json& table = arg(a, "table");
             if (table.type() == Json::Type::String)
                 return info::decode_bits(static_cast<std::uint32_t>(word), table.as_string(), width);
             if (table.type() != Json::Type::Array) bad("table", "a table name or an array of bit definitions");
             std::vector<info::BitDef> defs;
             for (const Json& d : table.as_array()) {
                 info::BitDef b;
                 b.bit = need_int32(d, "bit");
                 b.name = opt_string(d, "name").value_or("");
                 b.meaning = opt_string(d, "meaning").value_or("");
                 b.confirmed = opt_bool(d, "confirmed").value_or(false);
                 b.evidence = opt_string(d, "evidence").value_or(b.evidence);
                 b.active_low = opt_bool(d, "active_low").value_or(false);
                 defs.push_back(b);
             }
             return info::decode_bits(static_cast<std::uint32_t>(word), defs, width);
         }},
        {"info.position_mode",
         [](const Json& a) { return info::position_mode(opt_u32(a, "code"), opt_string(a, "model")); }},
        {"info.io_bit_tables",
         [](const Json& a) {
             const Json& mode = arg(a, "mode");
             const Json* code = mode.type() == Json::Type::Object ? mode.find("code") : nullptr;
             if (code && code->type() == Json::Type::Number && !(std::fabs(code->as_number()) < 9.0e18))
                 bad("mode", "a position_mode() result (its code an integer)");  // no undefined integer cast
             const info::IoTables t = mode.type() == Json::Type::String ? info::io_bit_tables(mode.as_string())
                                                                         : info::io_bit_tables(mode);
             Json out = Json::object();
             out.set("DIPM", table_name(t.dipm));
             out.set("DOPM", table_name(t.dopm));
             return out;
         }},
        {"info.rtc_in_use", [](const Json& a) { return Json(info::rtc_in_use(opt_u32(a, "ssse"))); }},
        // ---------------------------------------------------------------- the status decode (values.Status)
        {"status.decode", [](const Json& a) { return to_json(Status::from_registers(words_arg(a, "words"))); }},
        {"info.decode_maintenance",
         [](const Json& a) {
             return info::maintenance_json(block_from(arg(a, "block")), opt_bool(a, "rtc"), con_family_arg(a, "family"));
         }},
        {"info.decode_alarm_history",
         [](const Json& a) {
             return info::decode_alarm_history(record_from(arg(a, "record")), words_arg(a, "words"),
                                               opt_u32(a, "current"), opt_u32(a, "dss1"), opt_u32(a, "c3"),
                                               opt_bool(a, "absolute_encoder"), opt_bool(a, "rtc_in_use"),
                                               opt_int32(a, "system_code"));
         }},
        // ---------------------------------------------------------------- (0.6, py-info-cli) input ranges
        {"info.product_line",
         [](const Json& a) { return Json(info::product_line(opt_string(a, "family"), opt_string(a, "gateway"))); }},
        {"info.ec_parameter_table", [](const Json&) { return info::ec_parameter_table(); }},
        {"info.input_ranges",
         [](const Json& a) { return info::input_ranges(opt_string(a, "line"), con_family_arg(a, "family")); }},
    };
    return table;
}

Json call(const std::string& name, const Json& args) {
    if (name == "functions") {
        Json out = Json::array();
        out.push_back("functions");
        for (const auto& kv : functions()) out.push_back(kv.first);
        return out;
    }
    const auto it = functions().find(name);
    if (it == functions().end()) throw std::logic_error("iad_call_json: no function '" + name + "' in this library");
    // IAD_ERR_UNSUPPORTED (std::logic_error) is kept for a function this library lacks: a nested argument of the
    // wrong JSON type (Json's accessors throw std::logic_error) is a bad argument
    const auto run = [&](const Json& one) {
        try {
            return it->second(one);
        } catch (const std::invalid_argument&) {
            throw;
        } catch (const std::out_of_range&) {
            throw;
        } catch (const std::logic_error& e) {
            throw std::invalid_argument("iad_call_json: " + name + ": an argument has the wrong type (" + e.what() +
                                        ")");
        }
    };
    if (args.type() == Json::Type::Array) {
        Json out = Json::array();
        for (const Json& one : args.as_array()) out.push_back(run(one));
        return out;
    }
    if (!args.is_null() && args.type() != Json::Type::Object)
        throw std::invalid_argument("iad_call_json: the arguments must be a JSON object or an array of objects");
    return run(args.is_null() ? Json::object() : args);
}

}  // namespace

extern "C" IAD_API iad_result IAD_CALL iad_call_json(const char* function, const char* args_json, char* buffer,
                                                     size_t buffer_size, size_t* required);

extern "C" IAD_API iad_result IAD_CALL iad_call_json(const char* function, const char* args_json, char* buffer,
                                                     size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::string name = need(function, "function");
        const Json args = args_json && *args_json ? Json::parse(args_json) : Json();
        std::string out;
        write_py(out, call(name, args));
        write_text_output(out, buffer, buffer_size, required);
    });
}
