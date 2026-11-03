// Parameter values and keys, the listing / description of a unit's parameters and plan_parameter_write(): every write
// guard of the Python edition's write_parameter / write_user_parameter, decided before anything is sent. Nothing is
// sent from here: the Axis reads the class blocks (ParameterContext), calls plan_parameter_write(), then writes,
// verifies and commits. Port of the checks of ec.py (parameters, read_parameter, write_parameter, write_user_parameter,
// _write_item, _widens_soft_limit) and info_item_note; C# Axis.Config.cs.
//
// Guards, all before anything is written: read-only parameters (EC P8, P11, P13-P18; CON No.166, No.188) are refused
// (UnsafeCommandError); values outside the ranges give SoftLimitError. Home, dangerous and unlisted parameters,
// widening a soft limit and raw items of unknown meaning are written with a warning (plan.warnings):
// allow_home_params, allow_dangerous and allow_unlisted are accepted and ignored (no longer needed). Every unit's
// parameters are written the same way: there is no per-model opt-in and no per-model warning.
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "model_detail.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

// internal API (ec_detail, con_detail): declared in model_detail.hpp

namespace {

constexpr int kItemZrmk = 8;  // 8.8 (b8 CORG: the driver warns before a P3 change)
constexpr int kItemTcp2 = 62; // 7.62 second push conversion factor (the driver warns before a P3 change when != 0)

const char* const kPioWiringNote = "changes how the EC reads its PIO inputs (PLC wiring) once applied";

bool writable_class(int class_id) {
    for (int c : reg::kWritableParameterClasses)
        if (c == class_id) return true;
    return false;
}

const ClassItem* find_item(const std::map<int, ClassBlock>& classes, int class_id, int item_id, int entry = 0) {
    const auto it = classes.find(class_id);
    return it == classes.end() ? nullptr : it->second.item(item_id, entry);
}

/// The CON parameter that lives at class.item[entry] (the first of the table: several bit fields share an item).
const ConParam* named_for_item(int class_id, int item_id, int entry) {
    for (int no = 1; no <= 300; ++no) {
        const ConParam* p = con_parameter(no);
        if (p != nullptr && p->class_id == class_id && p->item_id == item_id && p->entry == entry) return p;
    }
    return nullptr;
}

/// A soft-limit write (7.1 / 7.2) that widens the range is a stroke change: returns why, else "".
std::string widens_soft_limit(int class_id, int item_id, std::int32_t old_value, std::int32_t new_value) {
    if (class_id != reg::kLimitParamsCategory) return std::string();
    if (item_id == reg::kAxisItemLimm && new_value > old_value)
        return util::format("widens the + soft limit 7.1 (%.2f -> %.2f): a stroke change", old_value * 0.01, new_value * 0.01);
    if (item_id == reg::kAxisItemLiml && new_value < old_value)
        return util::format("widens the - soft limit 7.2 (%.2f -> %.2f): a stroke change", old_value * 0.01, new_value * 0.01);
    return std::string();
}

std::string deg_text(std::string text, const ConParam& p, const std::string& units) {
    if ((p.unit == "mm" || p.unit == "mm/s") && units == "deg") {
        std::size_t pos = 0;
        while ((pos = text.find(" mm", pos)) != std::string::npos) {
            text.replace(pos, 3, " deg");
            pos += 4;
        }
    }
    return text;
}

std::string con_unit(const ConParam& p, const std::string& units) {
    if ((p.unit == "mm" || p.unit == "mm/s") && units == "deg") {
        std::string out = p.unit;
        const std::size_t pos = out.find("mm");
        if (pos != std::string::npos) out.replace(pos, 2, "deg");
        return out;
    }
    return p.unit;
}

std::map<std::string, std::string> choices_map(const std::map<int, std::string>& choices) {
    std::map<std::string, std::string> out;
    for (const auto& kv : choices) out[std::to_string(kv.first)] = kv.second;
    return out;
}

std::vector<ParameterInfo> list_ec(const ParameterContext& ctx) {
    std::vector<ParameterInfo> out;
    const ParameterModel model = model_parameter_list(ctx);
    const bool deg = ctx.units == "deg";
    std::optional<double> p1_ceiling = ctx.p1_ceiling_mm;
    for (int no : model.listed) {
        const ParamDef* p = ec_user_parameter(no);
        if (p == nullptr) continue;
        const ClassItem* item = find_item(ctx.classes, p->class_id, p->item_id, p->entry);
        const ClassItem* item2 = p->item_id2 ? find_item(ctx.classes, p->class_id, *p->item_id2, p->entry) : nullptr;
        const ec_detail::Decoded dec = ec_detail::decode(*p, item, item2);
        if (!(model.known || dec.found)) continue;
        if (no == 1 && dec.found && dec.value.number)
            p1_ceiling = p1_ceiling ? std::max(*p1_ceiling, *dec.value.number) : *dec.value.number;
        // settable: a parity parameter whose home pattern is 0 / FFFFFFFF has no value and cannot be set
        const bool settable = !(p->kind == "parity" && dec.found && dec.value.empty());
        ParameterInfo info;
        info.number = no;
        info.name = p->name;
        info.item = dec.key;
        info.address = dec.address;
        info.value = dec.value;
        std::string text = dec.text;
        if (deg) {
            std::size_t pos = 0;
            while ((pos = text.find(" mm", pos)) != std::string::npos) {
                text.replace(pos, 3, " deg");
                pos += 4;
            }
        }
        info.text = text;
        info.unit = deg && p->unit == "mm" ? "deg" : p->unit;
        if (no == 1) {
            info.min = kP1MinMm;
            info.max = p1_ceiling;
        } else if (no == 2) {
            info.min = kP2MinMm;
            info.max = kP2MaxMm;
        } else if (no == 4) {
            info.min = kP4MinMm;
            info.max = kP4MaxMm;
        }
        info.min_decimal = info.max_decimal = true;  // the mm limits are floats (the choices' limits below are ints)
        const std::map<int, std::string> choices = p->kind == "mm" ? std::map<int, std::string>() : p->choices;
        if (!choices.empty()) {
            info.min = static_cast<double>(choices.begin()->first);
            info.max = static_cast<double>(choices.rbegin()->first);
            info.min_decimal = info.max_decimal = false;
        }
        info.choices = choices_map(choices);
        info.writable = dec.found && !p->read_only && settable;
        info.dangerous = ec_user_parameter(no) != nullptr && is_ec_home_parameter(no);
        info.raw = dec.raw_hex;
        info.family = "EC";
        if (p->read_only)
            info.note = p->note.empty() ? std::string("read-only in this driver") : p->note;
        else if (!settable)
            info.note = "not settable: home pattern ORG (item 9.16) is 0 / FFFFFFFF";
        out.push_back(info);
    }
    return out;
}

std::vector<ParameterInfo> list_con(const ParameterContext& ctx) {
    std::vector<ParameterInfo> out;
    for (const ConParam& p : con_parameters(ctx.family)) {
        const ClassItem* item = find_item(ctx.classes, p.class_id, p.item_id, p.entry);
        if (item == nullptr || item->size != 2) continue;
        ParameterInfo info;
        info.number = p.no;
        info.name = p.name;
        info.symbol = p.symbol;
        info.item = p.key();
        info.address = util::format("0x%04X", item->address);
        const ParamValue value = con_detail::decode(p, item->raw);
        info.value = value;
        info.text = deg_text(con_detail::text(p, value), p, ctx.units);
        info.unit = con_unit(p, ctx.units);
        if (p.kind == "baud") {
            const auto range = con_detail::baud_range();
            info.min = range.first;
            info.max = range.second;
        } else {
            info.min = p.lo;
            info.max = p.hi;
            info.min_decimal = p.lo_decimal;  // a float in the Python edition's table, even when whole
            info.max_decimal = p.hi_decimal;
        }
        info.choices = choices_map(p.choices);
        info.writable = p.writable;
        info.dangerous = p.dangerous;
        info.raw = item->hex();
        info.family = "CON";
        info.note = con_detail::note_of(p.no);
        out.push_back(info);
    }
    return out;
}

}  // namespace

// ------------------------------------------------------------------------------------------ ParamValue / ParameterKey

ParamValue ParamValue::of(double value) {
    ParamValue v;
    v.number = value;
    v.text = util::g(value);
    return v;
}

ParamValue ParamValue::of_whole(double value) {
    ParamValue v = of(value);
    v.whole = true;
    return v;
}

ParamValue ParamValue::parse(const std::string& text) {
    ParamValue v;
    const std::string t = util::trim(text);
    if (t.empty()) return v;
    v.text = t;
    // a hexadecimal integer ("0x1F", for the raw item values)
    std::string lowered = util::lower(t);
    bool negative = false;
    std::string body = lowered;
    if (!body.empty() && (body[0] == '-' || body[0] == '+')) {
        negative = body[0] == '-';
        body.erase(0, 1);
    }
    if (util::starts_with(body, "0x")) {
        const auto hex = util::parse_int(body.substr(2), 16);
        if (hex && body.size() > 2 && body.find_first_not_of("0123456789abcdef", 2) == std::string::npos) {
            v.number = negative ? -static_cast<double>(*hex) : static_cast<double>(*hex);
            return v;
        }
        return v;  // a label
    }
    const auto number = util::parse_double(t);
    if (number) v.number = *number;
    return v;
}

std::string ParamValue::to_string() const {
    if (number) return util::g(*number);
    return text;
}

ParameterKey ParameterKey::of(int number) {
    ParameterKey k;
    k.number = number;
    k.text = std::to_string(number);
    return k;
}

ParameterKey ParameterKey::parse(const std::string& key) {
    ParameterKey k;
    const std::string text = util::trim(key);
    k.text = text;
    auto all_digits = [](const std::string& s) {
        if (s.empty()) return false;
        for (char c : s)
            if (c < '0' || c > '9') return false;
        return true;
    };
    // a named parameter: 5, "5", "P5", "p5", "No.5", "no.5"
    for (const char* prefix : {"No.", "no.", "NO.", "P", "p"}) {
        if (util::starts_with(text, prefix)) {
            const std::string rest = util::trim(text.substr(std::string(prefix).size()));
            if (all_digits(rest)) {
                k.number = static_cast<int>(util::parse_int(rest).value_or(0));
                return k;
            }
        }
    }
    if (all_digits(text)) {
        k.number = static_cast<int>(util::parse_int(text).value_or(0));
        return k;
    }
    // a raw item: "class.item" or "class.item[entry]", the class in hexadecimal ("8.15", "C.1[1]", "0x0C.1")
    std::string body = text;
    int entry = 0;
    if (!body.empty() && body.back() == ']' && body.find('[') != std::string::npos) {
        const std::size_t open = body.find('[');
        const std::string entry_text = util::trim(body.substr(open + 1, body.size() - open - 2));
        if (!all_digits(entry_text))
            throw std::invalid_argument("parameter key '" + key + "': use a parameter number or 'class.item' such as '8.15'");
        entry = static_cast<int>(util::parse_int(entry_text).value_or(0));
        body = body.substr(0, open);
    }
    const std::size_t dot = body.find('.');
    if (dot == std::string::npos || !all_digits(util::trim(body.substr(dot + 1))))
        throw std::invalid_argument("parameter key '" + key + "': use a parameter number or 'class.item' such as '8.15'");
    const std::string cls = util::trim(body.substr(0, dot));
    const auto class_id = util::parse_int(cls, 16);
    if (cls.empty() || !class_id || *class_id < 0 || *class_id > 0xFFFF)
        throw std::invalid_argument("parameter key '" + key + "': class '" + cls + "' is not a number");
    k.class_id = static_cast<int>(*class_id);
    k.item_id = static_cast<int>(util::parse_int(util::trim(body.substr(dot + 1))).value_or(0));
    k.entry = entry;
    return k;
}

// ------------------------------------------------------------------------------------------ listing

std::vector<int> parameter_classes_needed(bool is_ec) {
    // EC: the classes of the named parameters (7, 8, 9, 0xC) and class 3 (MXPT, MINI) for the rules; the CON table uses
    // the same four classes and class 3 for MINI, so both need the same set
    (void)is_ec;
    return {reg::kControllerMonitorCategory, reg::kLimitParamsCategory, reg::kMotionParamsCategory,
            reg::kFactoryParamsCategory, reg::kZonesCategory};
}

std::vector<ParameterInfo> list_parameters(const ParameterContext& ctx) {
    return ctx.is_ec ? list_ec(ctx) : list_con(ctx);
}

ParameterInfo describe_parameter(const ParameterContext& ctx, const ParameterKey& key) {
    if (key.number) {
        for (ParameterInfo& info : list_parameters(ctx))
            if (info.number == *key.number) return info;
        // a number is named as the Python edition's repr of an int key (parameter 15), anything else as a string ('P15')
        const bool digits = key.text.find_first_not_of("0123456789") == std::string::npos;
        throw UnsafeCommandError("parameter " + (digits ? key.text : "'" + key.text + "'") + " is not offered on this unit");
    }
    const auto block = ctx.classes.find(key.class_id);
    if (block == ctx.classes.end())
        throw UnsafeCommandError(util::format("class 0x%04X was not read for this unit: item '%s' is not available",
                                              key.class_id, key.text.c_str()));
    const ClassItem* item = block->second.item(key.item_id, key.entry);
    if (item == nullptr)
        throw UnsafeCommandError(util::format("item '%s' is not in this unit's ID map of class 0x%04X", key.text.c_str(),
                                              key.class_id));
    const ConParam* named = named_for_item(key.class_id, key.item_id, key.entry);
    ParameterInfo info;
    info.number = 0;
    info.name = item->key();
    info.item = item->key();
    info.address = util::format("0x%04X", item->address);
    info.value = ParamValue::of_whole(static_cast<double>(item->raw));
    info.writable = writable_class(key.class_id) && item->size == 2;
    info.dangerous = named == nullptr || named->dangerous;
    info.raw = item->hex();
    info.family = "raw";
    info.unsigned_value = item->raw;
    info.signed_value = item->signed_value();
    info.words = item->size;
    if (named != nullptr) info.named = named->no;
    info.note = ec_detail::item_note(key.class_id, key.item_id);
    return info;
}

// ------------------------------------------------------------------------------------------ plan

std::vector<std::uint16_t> ParameterWritePlan::words() const {
    if (size == 1) return {static_cast<std::uint16_t>(raw_after & 0xFFFF)};
    const auto pair = reg::to_u32_words(raw_after);
    return {pair.first, pair.second};
}

namespace {

ParameterWritePlan plan_ec(const ParameterContext& ctx, int no, const ParamValue& value) {
    const ParamDef* p = ec_user_parameter(no);
    if (p == nullptr) {
        std::string confirmed;
        for (const ParamDef& d : ec_user_parameters()) confirmed += (confirmed.empty() ? "P" : ", P") + std::to_string(d.no);
        throw UnsafeCommandError(util::format("P%d is not a confirmed EC user parameter (mapped: %s); nothing written", no,
                                              confirmed.c_str()));
    }
    if (p->read_only)
        throw UnsafeCommandError(util::format("P%d (%s) is read-only in this driver%s. Nothing written", p->no, p->name.c_str(),
                                              p->note.empty() ? "" : (": " + p->note).c_str()));
    const bool home = is_ec_home_parameter(p->no);  // written with ec_detail::kHomeWarning

    const ParameterModel model = model_parameter_list(ctx);
    const bool listed = std::find(model.listed.begin(), model.listed.end(), p->no) != model.listed.end();
    std::string unlisted;  // the warning of a parameter this unit's list does not show (written anyway)
    if (!listed) {
        std::string hidden;
        for (const auto& h : model.hidden)
            if (h.first == p->no) hidden = h.second;
        std::string list;
        for (int n : model.listed) list += (list.empty() ? "P" : ", P") + std::to_string(n);
        const std::string unit = ctx.unit_code ? util::format("unit 0x%02X", *ctx.unit_code) : std::string("this unit");
        unlisted = "P" + std::to_string(p->no) + " (" + p->name + ") is not in the parameter list for " + unit +
                   (hidden.empty() ? "" : " (hidden by the visibility rule: " + hidden + ")") + " (listed: " + list +
                   "); written anyway";
    }
    const bool rotary = ec_detail::is_rotary(ctx);

    double number;
    try {
        number = ec_detail::parse_value(*p, value).number.value_or(0.0);
    } catch (const std::invalid_argument& e) {
        throw SoftLimitError(std::string(e.what()) + "; nothing written");
    }

    const ClassItem* item = find_item(ctx.classes, p->class_id, p->item_id, p->entry);
    if (item == nullptr || item->size != 2)
        throw UnsafeCommandError(util::format("P%d (item %d.%d) is not a 32-bit item in this EC's directory; nothing written",
                                              p->no, p->class_id, p->item_id));
    const ec_detail::Decoded before = ec_detail::decode(*p, item, nullptr);
    // P1 can only narrow the stroke: up to the largest value read (the present one counts)
    std::optional<double> p1_ceiling = ctx.p1_ceiling_mm;
    if (p->no == 1 && before.value.number)
        p1_ceiling = p1_ceiling ? std::max(*p1_ceiling, *before.value.number) : *before.value.number;
    ec_detail::check_value(*p, number, before, ctx, p1_ceiling);  // SoftLimitError

    std::vector<std::string> warnings = ec_detail::guard(*p, number, rotary, ctx.model_name);  // UnsafeCommandError

    std::uint32_t new_raw;
    try {
        new_raw = ec_detail::encode(*p, item->raw, number);
    } catch (const std::invalid_argument& e) {
        throw SoftLimitError(std::string(e.what()) + "; nothing written");
    }
    ClassItem after_item = *item;
    after_item.raw = new_raw;
    const ec_detail::Decoded after = ec_detail::decode(*p, &after_item, nullptr);
    const bool changed = new_raw != item->raw;

    if (rotary && p->kind == "mm")
        warnings.push_back(util::format("P%d is in deg on this rotary axis (same 0.01 encoding)", p->no));
    if (!listed) warnings.push_back(unlisted);
    if (home) warnings.push_back(ec_detail::kHomeWarning);
    if (p->no == 3 && changed) {
        const ClassItem* zrmk = find_item(ctx.classes, reg::kMotionParamsCategory, kItemZrmk);
        if (zrmk != nullptr && (zrmk->raw & (1u << 8)) != 0)
            warnings.push_back(
                "ZRMK (8.8) b8 CORG is set: a home-direction change may cause a malfunction unless a mechanical stopper "
                "is installed externally (UNVERIFIED)");
        const ClassItem* tcp2 = find_item(ctx.classes, reg::kLimitParamsCategory, kItemTcp2);
        if (tcp2 != nullptr && tcp2->raw != 0)
            warnings.push_back(
                "TCP2 (7.62) != 0: the meaning of the push force in the position data may change with the home direction "
                "(a reset is required; UNVERIFIED)");
        warnings.push_back(util::format("home pattern ORG %u -> %u (the parity is the direction)",
                                        static_cast<unsigned>(item->raw), static_cast<unsigned>(new_raw)));
    }
    if (p->no == 1)
        warnings.push_back(
            "P1 is the upper soft limit: the driver checks targets against the new value at once; the EC applies it "
            "after a software reset / power cycle");
    if (p->no == 9 || p->no == 12) warnings.push_back("P" + std::to_string(p->no) + " " + kPioWiringNote);

    ParameterWritePlan plan;
    plan.number = p->no;
    plan.name = p->name;
    plan.key = before.key;
    plan.unit = rotary && p->unit == "mm" ? "deg" : p->unit;
    plan.class_id = p->class_id;
    plan.item_id = p->item_id;
    plan.entry = p->entry;
    plan.address = item->address;
    plan.size = 2;
    plan.raw_before = item->raw;
    plan.raw_after = new_raw;
    plan.before = before.text;
    plan.after = after.text;
    plan.value = after.value;
    plan.listed_for_model = listed;
    plan.verified_for_model = model.verified;  // kept for compatibility: an IA-OS list is recorded for the unit code
    plan.home_parameter = home;
    plan.dangerous = home;
    plan.rereads_soft_limits = p->no == 1;
    plan.warnings = warnings;
    return plan;
}

ParameterWritePlan plan_con(const ParameterContext& ctx, int no, const ParamValue& value) {
    const ConParam* base = con_parameter(no);
    const ClassItem* item = base == nullptr ? nullptr : find_item(ctx.classes, base->class_id, base->item_id, base->entry);
    if (base == nullptr || item == nullptr || item->size != 2)
        throw UnsafeCommandError(util::format(
            "parameter No.%d is not offered on this unit (its item is not in the ID map); nothing written", no));
    if (!base->writable)
        throw UnsafeCommandError(util::format("No.%d (%s) is read-only in this driver; nothing written", base->no,
                                              base->name.c_str()));
    const ConParam p = con_detail::for_family(*base, ctx.family);

    con_detail::Check check;
    try {
        check = con_detail::check_value(*base, value, ctx.family, con_detail::unit_limits(ctx.classes, ctx.support));
    } catch (const std::invalid_argument& e) {
        throw SoftLimitError(std::string(e.what()) + "; nothing written");
    }
    std::vector<std::string> warnings = check.warnings;
    // No.25 PIO pattern: not above the unit's own highest pattern MXPT 3.16 (ON-WIRE: a PCON-CB reads 5), when it reads
    if (p.no == 25 && check.value.number) {
        const ClassItem* mxpt = find_item(ctx.classes, reg::kControllerMonitorCategory, reg::kControllerItemMxpt);
        if (mxpt != nullptr && mxpt->raw != 0 && mxpt->raw != 0xFFFFFFFFu && *check.value.number > mxpt->raw)
            throw SoftLimitError(util::format("No.25: PIO pattern %s is above this unit's highest pattern MXPT (3.16) %u; "
                                              "nothing written",
                                              util::g(*check.value.number).c_str(), static_cast<unsigned>(mxpt->raw)));
    }
    if (p.dangerous)
        warnings.push_back(util::format("No.%d (%s) is a dangerous parameter (home, motor / encoder, lead / axis type, "
                                        "gains, PIO / fieldbus / baud configuration): check it against the unit's "
                                        "manual",
                                        p.no, p.name.c_str()));

    std::uint32_t new_raw;
    try {
        new_raw = con_detail::encode(p, item->raw, check.value);
    } catch (const std::invalid_argument& e) {
        throw SoftLimitError(std::string(e.what()) + "; nothing written");
    }
    const std::string widen =
        widens_soft_limit(p.class_id, p.item_id, item->signed_value(), static_cast<std::int32_t>(new_raw));
    if (!widen.empty()) warnings.push_back(widen);
    if (p.no == 3 || p.no == 4) {
        // the soft limits: + (No.3) must not fall below - (No.4); the other one is read from the unit (ON-WIRE, PCON-CB)
        const bool plus = p.no == 3;
        const ClassItem* other = find_item(ctx.classes, reg::kLimitParamsCategory,
                                           plus ? reg::kAxisItemLiml : reg::kAxisItemLimm);
        const std::int32_t mine = static_cast<std::int32_t>(new_raw);
        const std::string unit_text = " " + con_unit(p, ctx.units);
        if (other != nullptr && (plus ? mine < other->signed_value() : mine > other->signed_value())) {
            throw SoftLimitError(util::format(
                "No.%d: the soft limit %s %s%s would lie %s the soft limit %s (No.%d, %s%s): the limits would be inverted; "
                "write No.%d first; nothing written",
                p.no, plus ? "+" : "-", util::fixed(mine * 0.01, 2).c_str(), unit_text.c_str(),
                plus ? "below" : "above", plus ? "-" : "+", plus ? 4 : 3,
                util::fixed(other->signed_value() * 0.01, 2).c_str(), unit_text.c_str(), plus ? 4 : 3));
        }
    }
    // the reset warning is the Axis's: added only to a write that is not skipped (axis_config.cpp)

    const ParamValue before_value = con_detail::decode(p, item->raw);
    const ParamValue after_value = con_detail::decode(p, new_raw);
    ParameterWritePlan plan;
    plan.number = p.no;
    plan.name = p.name;
    plan.key = p.key();
    plan.unit = con_unit(p, ctx.units);
    plan.class_id = p.class_id;
    plan.item_id = p.item_id;
    plan.entry = p.entry;
    plan.address = item->address;
    plan.size = 2;
    plan.raw_before = item->raw;
    plan.raw_after = new_raw;
    plan.before = deg_text(con_detail::text(p, before_value), p, ctx.units);
    plan.after = deg_text(con_detail::text(p, after_value), p, ctx.units);
    plan.value = after_value;
    plan.listed_for_model = true;
    plan.verified_for_model = false;
    plan.home_parameter = is_con_home_parameter(p.no);
    plan.dangerous = p.dangerous;
    plan.rereads_soft_limits = p.class_id == reg::kLimitParamsCategory &&
                               (p.item_id == reg::kAxisItemLimm || p.item_id == reg::kAxisItemLiml);
    plan.warnings = warnings;
    return plan;
}

ParameterWritePlan plan_raw(const ParameterContext& ctx, const ParameterKey& key, const ParamValue& value) {
    if (!writable_class(key.class_id))
        throw UnsafeCommandError(util::format(
            "class 0x%04X is not a parameter class (writable: 7, 8, 9, 0x0C); nothing written", key.class_id));
    if (!value.number || !(*value.number == std::floor(*value.number)) || !std::isfinite(*value.number))
        throw SoftLimitError("a raw item takes the raw integer, got '" + (value.number ? value.to_string() : value.text) +
                             "'; nothing written");
    const auto block = ctx.classes.find(key.class_id);
    if (block == ctx.classes.end())
        throw UnsafeCommandError(util::format("class 0x%04X was not read for this unit: item '%s' is not available; nothing "
                                              "written",
                                              key.class_id, key.text.c_str()));
    const ClassItem* item = block->second.item(key.item_id, key.entry);
    if (item == nullptr)
        throw UnsafeCommandError("item '" + key.text + "' is not in this unit's ID map; nothing written");
    const int bits = 16 * item->size;
    const double number = *value.number;
    const double lo = -std::ldexp(1.0, bits - 1);
    const double hi = std::ldexp(1.0, bits);
    if (!(number >= lo && number < hi))
        throw SoftLimitError(util::format("raw value %s does not fit %d bits; nothing written", util::g(number).c_str(), bits));
    const std::uint32_t mask = bits >= 32 ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const std::int64_t integer = static_cast<std::int64_t>(number);
    const std::uint32_t new_raw = static_cast<std::uint32_t>(integer & static_cast<std::int64_t>(mask));
    const ConParam* named = named_for_item(key.class_id, key.item_id, key.entry);
    const bool whole_item = named != nullptr && (named->kind == "scaled" || named->kind == "raw");
    const std::int64_t half = std::int64_t(1) << (bits - 1);
    const std::int32_t new_signed = static_cast<std::int32_t>(integer < half ? integer : integer - (std::int64_t(1) << bits));
    const std::string widen = widens_soft_limit(key.class_id, key.item_id, item->signed_value(), new_signed);
    const bool dangerous = !whole_item || named->dangerous || !widen.empty();
    std::string why;  // the warning of a dangerous raw write that does not widen a soft limit (written anyway)
    if (dangerous && widen.empty()) {
        if (named == nullptr)
            why = "an item of unknown meaning";
        else if (named->dangerous)
            why = util::format("No.%d (%s) is dangerous", named->no, named->name.c_str());
        else
            why = util::format("No.%d (%s) shares this item", named->no, named->name.c_str());
    }
    if (item->size != 2)
        throw UnsafeCommandError("item " + item->key() + " is not a 32-bit item; nothing written");

    ParameterWritePlan plan;
    plan.number = named ? named->no : 0;
    plan.name = item->key();
    plan.key = item->key();
    plan.unit = "raw";
    plan.class_id = key.class_id;
    plan.item_id = key.item_id;
    plan.entry = key.entry;
    plan.address = item->address;
    plan.size = item->size;
    plan.raw_before = item->raw;
    plan.raw_after = new_raw;
    plan.before = item->hex();
    plan.after = util::format("%0*X", 4 * item->size, static_cast<unsigned>(new_raw));
    plan.value = ParamValue::of_whole(static_cast<double>(new_raw));  // as describe_parameter: a raw item is an int
    plan.listed_for_model = true;
    plan.verified_for_model = false;
    plan.home_parameter = false;
    plan.dangerous = dangerous;
    plan.rereads_soft_limits = key.class_id == reg::kLimitParamsCategory &&
                               (key.item_id == reg::kAxisItemLimm || key.item_id == reg::kAxisItemLiml);
    plan.warnings.push_back("raw item write: its meaning is UNVERIFIED on this unit");
    if (!why.empty()) plan.warnings.push_back("raw write of '" + key.text + "': " + why + "; written anyway");
    if (!widen.empty()) plan.warnings.push_back(widen);
    return plan;
}

}  // namespace

ParameterWritePlan plan_parameter_write(const ParameterContext& ctx, const ParameterKey& key, const ParamValue& value,
                                        const ParameterWriteOptions& options) {
    (void)options;  // allow_dangerous / allow_unlisted / allow_home_params: accepted and ignored (no longer needed)
    if (key.number) return ctx.is_ec ? plan_ec(ctx, *key.number, value) : plan_con(ctx, *key.number, value);
    return plan_raw(ctx, key, value);
}

}  // namespace iai_actuator_driver
