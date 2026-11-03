// The unit's own limits (class 7 by item ID), soft limits, stroke, stored rows and the ranges the driver enforces.
// Port of Axis.read_axis_limits / read_soft_limits / max_positions / limits() (Python ec.py; C# Axis.cs, AxisTypes.cs).
// Everything here is READ-ONLY and pure apart from the UnitReader. A class missing or refused (an exception response)
// leaves its fields empty and adds a note; a timeout propagates. The driver adapts to whatever unit is connected: every
// range comes from the unit itself, nothing is refused because a model is unknown.
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

const char* const kLogSource = "iai_actuator_driver.limits";

constexpr double kMaxSensibleSpeed = 100000.0;

std::string join_notes(const std::vector<std::string>& notes) { return util::join(notes, "; "); }

}  // namespace

// ------------------------------------------------------------------------------------------ AxisLimits

std::string AxisLimits::units() const { return reg::axis_units(actuator_kind, rotary_atyp); }

std::string AxisLimits::kind() const { return reg::axis_kind_name(actuator_kind, rotary_atyp); }

std::optional<bool> AxisLimits::rotary() const {
    if (!actuator_kind) return rotary_atyp;
    return *actuator_kind == reg::kAkidRotary;
}

std::optional<bool> AxisLimits::push_available() const {
    if (!max_push_pct && !push_force_factor && !supports_push) return std::nullopt;
    const bool refused = (supports_push.has_value() && !*supports_push) || (max_push_pct.has_value() && *max_push_pct == 0) ||
                         (push_force_factor.has_value() && *push_force_factor == 0);
    return !refused;
}

std::optional<double> AxisLimits::push_force_estimate(double pct) const {
    if (!max_push_pct || *max_push_pct == 0 || !push_force_factor) return std::nullopt;
    const double value = *push_force_factor * pct / *max_push_pct + (push_force_offset ? *push_force_offset : 0);
    return std::round(value * 10.0) / 10.0;
}

std::pair<std::optional<double>, std::string> AxisLimits::unit_speed_cap() const {
    if (max_speed_mm_s)
        return {max_speed_mm_s,
                max_speed_address ? util::format("MAXV item 7.4 @0x%04X", *max_speed_address) : "MAXV item 7.4"};
    if (default_speed_mm_s)
        return {default_speed_mm_s,
                util::format("0x%04X (item 8.10 VCMD, the default speed: MAXV 7.4 not readable)",
                             default_speed_address.value_or(0x0812))};
    return {std::nullopt, "MAXV 7.4 not readable (" + (notes.empty() ? std::string("item missing") : join_notes(notes)) + ")"};
}

// ------------------------------------------------------------------------------------------ read_axis_limits

AxisLimits read_axis_limits(UnitReader& reader, const UnitDirectory& directory) {
    AxisLimits out;
    std::vector<std::string> notes;

    ClassBlock block;
    bool have_block = false;
    try {
        if (directory.empty()) throw std::out_of_range("the index table is not readable");
        block = directory.read_class(reader, reg::kLimitParamsCategory);
        have_block = true;
    } catch (const ModbusSlaveError& e) {
        notes.push_back(std::string("class 7 not readable: ") + e.what());
    } catch (const std::out_of_range& e) {
        notes.push_back(std::string("class 7: ") + e.what());
    } catch (const std::invalid_argument& e) {
        notes.push_back(std::string("class 7: ") + e.what());
    }
    if (have_block && !block.id_map_address) {
        notes.push_back("class 7 has no ID map");
        have_block = false;
    }

    auto item = [&](int item_id) -> const ClassItem* { return have_block ? block.item(item_id) : nullptr; };
    auto scaled = [&](int item_id, double scale, double lo, double hi) -> std::optional<double> {
        const ClassItem* it = item(item_id);
        if (it == nullptr) return std::nullopt;
        const double value = reg::from_counts(it->signed_value(), scale);
        if (value >= lo && value <= hi) return value;
        return std::nullopt;
    };

    out.max_speed_mm_s = scaled(reg::kAxisItemMaxv, reg::kVelocityScale, 0.01, kMaxSensibleSpeed);
    if (out.max_speed_mm_s) out.max_speed_address = item(reg::kAxisItemMaxv)->address;
    out.max_accel_g = scaled(reg::kAxisItemMaxa, reg::kAccelScale, 0.01, 10.0);
    out.max_decel_g = scaled(reg::kAxisItemMaxd, reg::kAccelScale, 0.01, 10.0);
    out.max_push_speed_mm_s = scaled(reg::kAxisItemMxph, reg::kVelocityScale, 0.0, kMaxSensibleSpeed);
    if (const ClassItem* mxpp = item(reg::kAxisItemMxpp); mxpp != nullptr && mxpp->raw <= 1000)
        out.max_push_pct = static_cast<double>(mxpp->raw);
    if (const ClassItem* mnpp = item(reg::kAxisItemMnpp); mnpp != nullptr && mnpp->raw > 0 && mnpp->raw <= 100) {
        out.min_push_pct = static_cast<double>(mnpp->raw);
        out.min_push_from_unit = true;
    }
    if (const ClassItem* tcpc = item(reg::kAxisItemTcpc)) out.push_force_factor = tcpc->signed_value();
    if (const ClassItem* tcoa = item(reg::kAxisItemTcoa)) out.push_force_offset = tcoa->signed_value();
    if (const ClassItem* akid = item(reg::kAxisItemAkid)) out.actuator_kind = static_cast<int>(akid->raw & 0xFF);

    // class 1 support words (SUPT push / relative, SUP2 CINP): an exception response leaves them unknown
    std::optional<std::map<std::string, std::uint32_t>> support;
    if (!directory.empty()) {
        try {
            support = directory.read_support_words(reader);
        } catch (const ModbusSlaveError&) {
            support = std::nullopt;
        }
    }
    if (support) {
        const auto supt = support->find("SUPT");
        if (supt != support->end()) {
            out.supports_push = (supt->second & reg::kSuptPush) != 0;
            out.supports_relative = (supt->second & reg::kSuptIncm) != 0;
        }
        const auto mvpt = support->find("MVPT");
        if (mvpt != support->end()) out.move_patterns = mvpt->second;
    }
    out.lead_mm = scaled(reg::kAxisItemLead, reg::kPositionScale, 0.01, 10000.0);
    const ClassItem* atyp = item(reg::kAxisItemAtyp);
    if (atyp != nullptr) out.rotary_atyp = (atyp->raw & reg::kAtypRotary) != 0;
    std::optional<std::uint32_t> sup2;
    if (support) {
        const auto it = support->find("SUP2");
        if (it != support->end()) sup2 = it->second;
    }
    if (atyp != nullptr && sup2 && (*sup2 & reg::kSup2Cinp) != 0 && (atyp->raw & reg::kAtypRinp) != 0 &&
        !directory.empty()) {
        try {
            const ClassBlock c3 = directory.read_class(reader, reg::kControllerMonitorCategory);
            const ClassItem* mini = c3.item(reg::kControllerItemMini);
            if (mini != nullptr && mini->raw > 0 && mini->raw < 100000)
                out.min_band = util::round2(static_cast<double>(mini->raw) * reg::kPositionScale);
        } catch (const ModbusSlaveError& e) {
            notes.push_back(std::string("MINI 3.67 not readable: ") + e.what());
        } catch (const std::out_of_range& e) {
            notes.push_back(std::string("MINI 3.67 not readable: ") + e.what());
        } catch (const std::invalid_argument& e) {
            notes.push_back(std::string("MINI 3.67 not readable: ") + e.what());
        }
    }

    // MAXV not readable: the default command speed (item 8.10 at 0x0812 on every unit tested) is the fallback cap
    if (!out.max_speed_mm_s) {
        std::string why;
        if (directory.empty()) {
            why = "the index table is not readable";
        } else if (const IndexCategory* motion = directory.find(reg::kMotionParamsCategory); motion == nullptr) {
            why = "the index table has no parameter category 8";
        } else {
            const int address = motion->data_address + 0x12;  // item 8.10 VCMD
            try {
                const std::vector<std::uint16_t> words = reader.read_registers(address, 2);  // a timeout propagates
                if (words.size() >= 2) {
                    const double value = reg::from_counts(reg::from_u32_words(words[0], words[1]), reg::kVelocityScale);
                    if (value >= 0.01 && value <= kMaxSensibleSpeed) {
                        out.default_speed_mm_s = value;
                        out.default_speed_address = address;
                    } else
                        why = util::format("implausible value at 0x%04X: %s mm/s", address, util::g(value).c_str());
                }
            } catch (const ModbusSlaveError& e) {
                why = util::format("0x%04X not readable: ", address) + e.what();
            }
        }
        if (!out.default_speed_mm_s && !why.empty()) notes.push_back("default speed 8.10: " + why);
    }

    out.source = notes.empty() ? "class 7 by item ID" : join_notes(notes);
    out.notes = notes;
    return out;
}

// ------------------------------------------------------------------------------------------ soft limits and stroke

SoftLimits read_soft_limits(UnitReader& reader, const UnitDirectory& directory) {
    SoftLimits result;
    std::optional<int> address;
    if (directory.empty()) {
        address = reg::kSoftLimits;  // the index table is unknown: the default address
    } else if (const IndexCategory* rec = directory.find(reg::kLimitParamsCategory)) {
        address = rec->data_address;
    }
    if (!address) {
        result.note = "the index table has no soft-limit category (7)";
        return result;
    }

    // The 4 words of LIMM 7.1 / LIML 7.2: by item ID from class 7's ID map (as every other class-7 value), the first 4
    // words of the class only when it has no readable ID map.
    std::vector<std::uint16_t> words;
    std::string how;
    try {
        if (!directory.empty() && directory.find(reg::kLimitParamsCategory) != nullptr) {
            try {
                const ClassBlock block = directory.read_class(reader, reg::kLimitParamsCategory);
                const ClassItem* limm = block.item(reg::kAxisItemLimm);
                const ClassItem* liml = block.item(reg::kAxisItemLiml);
                if (limm != nullptr && liml != nullptr && limm->size == 2 && liml->size == 2) {
                    const auto a = reg::to_u32_words(limm->raw);
                    const auto b = reg::to_u32_words(liml->raw);
                    words = {a.first, a.second, b.first, b.second};
                    how = util::format("items 7.1 @0x%04X / 7.2 @0x%04X by ID", limm->address, liml->address);
                } else {
                    log_info(kLogSource, "class 7 has no items 7.1 / 7.2 in its ID map: soft limits from its first 4 words");
                }
            } catch (const ModbusSlaveError& e) {
                log_info(kLogSource, std::string("class 7 ID map not readable (") + e.what() +
                                         "): soft limits from its first 4 words");
            } catch (const std::out_of_range& e) {
                log_info(kLogSource, std::string("class 7 ID map not readable (") + e.what() +
                                         "): soft limits from its first 4 words");
            } catch (const std::invalid_argument& e) {
                log_info(kLogSource, std::string("class 7 ID map not readable (") + e.what() +
                                         "): soft limits from its first 4 words");
            }
        }
        if (words.empty()) {
            words = reader.read_registers(*address, 4);  // a timeout propagates, nothing changes
            how = "first 4 words of class 7";
        }
    } catch (const ModbusSlaveError& e) {
        result.note = util::format("0x%04X not readable: ", *address) + e.what();
        return result;
    }
    if (words.size() < 4) {
        result.note = util::format("0x%04X not readable: short answer (%d words)", *address, static_cast<int>(words.size()));
        return result;
    }
    const double a = reg::from_counts(reg::from_i32_words(words[0], words[1]), reg::kPositionScale);
    const double b = reg::from_counts(reg::from_i32_words(words[2], words[3]), reg::kPositionScale);
    const double lo = std::min(a, b);
    const double hi = std::max(a, b);
    if (hi - lo >= 0.01 && lo >= -1000000.0 && hi <= 1000000.0) {
        result.limits = Span{lo, hi};
        result.note = util::format("read from 0x%04X (%s)", *address, how.c_str());
        result.p1_mm = util::round2(a - reg::kSoftLimitMarginMm);  // 0x0700 = item 7.1 = P1 + 0.30
    } else {
        result.note = util::format("implausible values at 0x%04X (%s): %s / %s mm", *address, how.c_str(),
                                   util::g(a).c_str(), util::g(b).c_str());
    }
    return result;
}

std::optional<Span> unit_stroke(const std::optional<Span>& soft_limits) {
    if (!soft_limits) return std::nullopt;
    const double m = reg::kSoftLimitMarginMm;
    if (soft_limits->max - soft_limits->min > 2 * m + 0.01)
        return Span{util::round2(soft_limits->min + m), util::round2(soft_limits->max - m)};
    return soft_limits;
}

std::optional<Span> effective_stroke(const std::optional<Span>& configured, const std::optional<Span>& unit) {
    if (!configured || !unit) return configured ? configured : unit;
    const double lo = std::max(configured->min, unit->min);
    const double hi = std::min(configured->max, unit->max);
    if (lo < hi) return Span{lo, hi};
    return std::nullopt;
}

// ------------------------------------------------------------------------------------------ stored rows

int read_max_positions(UnitReader& reader, const UnitDirectory& directory, std::optional<int> system_code) {
    std::optional<int> rows;
    if (const IndexCategory* table = directory.find(reg::kPositionTableCategory); table != nullptr && table->list_count > 0)
        rows = table->list_count;
    std::optional<int> maxp;
    if (!directory.empty()) {
        try {
            const ClassBlock block = directory.read_class(reader, reg::kControllerMonitorCategory);
            const ClassItem* item = block.item(reg::kControllerItemMaxp);
            if (item != nullptr && item->raw > 0 && item->raw <= 0xFFFF) maxp = static_cast<int>(item->raw);
        } catch (const ModbusSlaveError& e) {
            log_info(kLogSource, std::string("MAXP (3.2) not readable: ") + e.what());
        } catch (const std::out_of_range& e) {
            log_info(kLogSource, std::string("MAXP (3.2) not readable: ") + e.what());
        } catch (const std::invalid_argument& e) {
            log_info(kLogSource, std::string("MAXP (3.2) not readable: ") + e.what());
        }
    }
    int count = reg::kPositionActiveRows;
    if (maxp && rows)
        count = std::min(*maxp, *rows);
    else if (maxp)
        count = *maxp;
    else if (rows)
        count = *rows;
    // An EC always keeps its three rows 0-2 (backward end, forward end, middle; MAXP reads 2 in 2-position mode)
    if (!system_code || reg::is_ec_family(system_code)) return std::max(count, reg::kPositionActiveRows);
    return count;
}

// ------------------------------------------------------------------------------------------ caps and ranges

namespace {

std::optional<double> unit_speed(const RangeInputs& in) {
    if (!in.limits) return std::nullopt;
    return in.limits->unit_speed_cap().first;
}

double min_of(const std::vector<std::optional<double>>& caps, double fallback) {
    bool any = false;
    double best = 0.0;
    for (const auto& c : caps) {
        if (!c) continue;
        if (!any || *c < best) best = *c;
        any = true;
    }
    return any ? best : fallback;
}

}  // namespace

double speed_cap(const RangeInputs& in) { return min_of({in.configured_max_speed, unit_speed(in)}, kFallbackSpeedCap); }

double accel_cap(const RangeInputs& in) {
    return min_of({in.configured_max_accel, in.limits ? in.limits->max_accel_g : std::nullopt,
                   in.limits ? in.limits->max_decel_g : std::nullopt},
                  kFallbackAccelCapG);
}

double push_cap_pct(const RangeInputs& in) {
    return min_of({in.configured_max_push, in.limits ? in.limits->max_push_pct : std::nullopt}, kFallbackPushCapPct);
}

AxisRanges make_ranges(const RangeInputs& in) {
    const std::optional<AxisLimits>& limits = in.limits;
    AxisRanges r;
    r.units = limits ? limits->units() : "mm";
    r.kind = limits ? limits->kind() : "unknown";
    r.family = in.family;

    const std::optional<Span> eff = effective_stroke(in.configured_stroke, unit_stroke(in.soft_limits));
    const std::optional<double> span = eff ? std::optional<double>(util::round2(eff->max - eff->min)) : std::nullopt;
    const std::optional<double> maxa = limits ? limits->max_accel_g : std::nullopt;
    const std::optional<double> maxd = limits ? limits->max_decel_g : std::nullopt;
    const std::optional<double> mxpp = limits ? limits->max_push_pct : std::nullopt;
    const std::optional<bool> push_available = limits ? limits->push_available() : std::nullopt;
    const double min_band = limits && limits->min_band ? *limits->min_band : 0.01;
    const std::string& u = r.units;

    r.position.min = eff ? std::optional<double>(eff->min) : std::nullopt;
    r.position.max = eff ? std::optional<double>(eff->max) : std::nullopt;
    r.position.unit = u;
    r.position.source = "soft limits 7.1 / 7.2 (" + in.soft_limits_note + ") minus the " +
                        util::fixed(reg::kSoftLimitMarginMm, 2) + " margin" +
                        (in.configured_stroke ? ", intersected with the configured stroke" : "");

    r.speed.min = 0.01;
    r.speed.max = speed_cap(in);
    r.speed.unit = u + "/s";
    r.speed.unit_max = unit_speed(in);
    r.speed.configured = in.configured_max_speed;
    r.speed.source = limits ? limits->unit_speed_cap().second : std::string("MAXV 7.4 not read");

    // a move with its own deceleration (profile path): accel is min(configured, MAXA), decel min(configured, MAXD)
    r.accel.min = 0.01;
    r.accel.max = min_of({in.configured_max_accel, maxa}, kFallbackAccelCapG);
    r.accel.unit = "G";
    r.accel.unit_max = maxa;
    r.accel.configured = in.configured_max_accel;
    r.accel.source = "MAXA 7.5";

    r.decel.min = 0.01;
    r.decel.max = min_of({in.configured_max_accel, maxd}, kFallbackAccelCapG);
    r.decel.unit = "G";
    r.decel.unit_max = maxd;
    r.decel.configured = in.configured_max_accel;
    r.decel.source = "MAXD 7.6";

    r.accel_decel.min = 0.01;
    r.accel_decel.max = accel_cap(in);
    r.accel_decel.unit = "G";
    r.accel_decel.source = "min(MAXA 7.5, MAXD 7.6): one value for both on a 0x9900 move";

    r.push.available = push_available;
    r.push.min = 1.0;
    r.push.max = push_available.has_value() && !*push_available ? 0.0 : push_cap_pct(in);
    r.push.recommended_min = limits ? limits->min_push_pct : reg::kMnppDefaultPct;
    r.push.unit = "%";
    r.push.unit_max = mxpp;
    r.push.configured = in.configured_max_push;
    r.push.source = "MXPP 7.9 (hard cap), MNPP 7.58 (recommendation), SUPT PUSH / TCPC 7.34";

    r.band.min = min_band;
    r.band.max = span ? std::min(kMoveBandMax, *span) : kMoveBandMax;
    r.band.unit = u;
    r.band.source = std::string("driver limit 10") + (min_band > 0.01 ? " ; MINI 3.67" : "");

    r.push_band.min = 0.01;
    r.push_band.max = span;
    r.push_band.unit = u;
    r.push_band.source = "the stroke";

    r.stored_rows.min = 0.0;
    r.stored_rows.max = in.rows - 1;
    r.stored_rows.count = in.rows;
    r.stored_rows.unit = "row";
    r.stored_rows.source = "MAXP 3.2 / the directory's position-table row count";

    r.jog_step.min = 0.01;
    r.jog_step.max = span;
    r.jog_step.unit = u;
    r.jog_step.source = "the stroke";

    r.speed_override.min = 1.0;
    r.speed_override.max = 100.0;
    r.speed_override.unit = "%";
    r.speed_override.source = "0x45 cmd 6 op2 = 255 x % / 100";
    return r;
}

}  // namespace iai_actuator_driver
