// The EC user parameters P1-P18 (as IAI's PC tool shows them): the table, decoding by class + item ID, the runtime
// visibility rules, the per-model parameter list, value parsing / encoding and the driver's write rules. Port of
// info.py (user parameters, visibility, model_parameter_list, ec_parameter_guard) and the value checks of ec.py
// (_check_parameter_value); C# UserParameters.cs. No I/O.
//
// Evidence: P1-P7, P9, P10 and P12 were located on the EC gripper by changing one parameter at a time and comparing the
// registers by class + item ID (VERIFIED in the Python edition; the same items are assumed on every EC family). P3
// decodes by the parity of the ORG home pattern. P8, P11 and P13-P18 are UNVERIFIED locations and read-only here.
//
// The functions of namespace ec_detail are internal to the model unit (declared in src/model_detail.hpp, used by
// parameter_plan.cpp); the public ones are ec_user_parameters(), ec_user_parameter(), is_ec_home_parameter() and
// model_parameter_list().
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "model_detail.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

// internal API (ec_detail): declared in model_detail.hpp

namespace {

constexpr double kPosEps = 1e-6;  // mm; positions are 0.01 mm counts, this only absorbs float noise

const char* const kEvidence =
    "VERIFIED on the EC-GRBP8M gripper (fw 0002): changed one at a time in IA-OS and the registers compared; same "
    "item IDs assumed elsewhere";
const char* const kUnverifiedItem = "UNVERIFIED: location expected, never tested on hardware (IA-OS hid it on all units tested)";
const char* const kReadOnlyUnverified = "read-only in this driver: location expected, never tested on hardware";

// P11 "Multi I/O function" (UNVERIFIED): the pair (byte 1 of FI00 8.25, byte 1 of FO00 8.29) -> the number shown.
struct PairRow {
    int fi, fo, no;
};
const PairRow kMultiIoPairs[] = {{0, 0, 0}, {15, 15, 1}, {13, 0, 2}, {0, 18, 3}, {0, 32, 4}, {0, 87, 5}, {22, 28, 6}, {26, 43, 7}};

const std::map<int, std::string> kEnable = {{0, "0:Disable"}, {1, "1:Enable"}};
const std::map<int, std::string> kEnableInverted = {{0, "0:Enable"}, {1, "1:Disable"}};

ParamDef def(int no, const char* name, int class_id, int item_id, const char* kind, const char* unit = "",
             std::optional<int> bit = std::nullopt, double offset_mm = 0.0, std::map<int, std::string> choices = {},
             const char* encoding = "", bool read_only = false, int entry = 0, std::optional<int> item_id2 = std::nullopt,
             const char* evidence = "", const char* note = "") {
    ParamDef p;
    p.no = no;
    p.name = name;
    p.class_id = class_id;
    p.item_id = item_id;
    p.kind = kind;
    p.unit = unit;
    p.bit = bit;
    p.offset_mm = offset_mm;
    p.choices = std::move(choices);
    p.encoding = encoding;
    p.read_only = read_only;
    p.entry = entry;
    p.item_id2 = item_id2;
    p.evidence = evidence[0] ? evidence : kEvidence;
    p.note = note;
    return p;
}

std::vector<ParamDef> build_table() {
    std::vector<ParamDef> t;
    t.push_back(def(1, "Operation range adjustment", 7, 1, "mm", "mm", std::nullopt, -0.30, {},
                    "s32 0.01 mm, raw = (P1 + 0.30 mm) x 100 (the upper soft limit)"));
    t.push_back(def(2, "Auto-switch \"LS\" signal detection range adjustment", 8, 9, "mm", "mm", std::nullopt, 0.0, {},
                    "s32 0.01 mm"));
    t.push_back(def(3, "HOME direction change", 9, 16, "parity", "", std::nullopt, 0.0,
                    {{1, "1:Forward"}, {0, "0:Reverse"}},
                    "ORG home pattern number (item 9.16): parity = direction (odd = 1:Forward, even = 0:Reverse); a "
                    "change writes ORG -1 / +1; not settable when ORG is 0 or FFFFFFFF",
                    false, 0, std::nullopt,
                    "VERIFIED on the gripper (IA-OS's 1:Forward <-> 0:Reverse changed ORG 7 <-> 8); the parity rule for "
                    "other patterns is UNVERIFIED (the EC-R6S rod holds 7)"));
    t.push_back(def(4, "HOME position adjustment", 9, 21, "mm", "mm", std::nullopt, 0.0, {}, "s32 0.01 mm"));
    t.push_back(def(5, "Smooth accel/decel setting", 8, 14, "bit", "", 6, 0.0, kEnable, "bit 6: 1 = Enable"));
    t.push_back(def(6, "Current control setting while stop", 8, 14, "bit", "", 10, 0.0, kEnable, "bit 10: 1 = Enable"));
    t.push_back(def(7, "Wireless function setting", 8, 50, "bit", "", 23, 0.0, kEnableInverted,
                    "bit 23, inverted sense: 0 = Enable, 1 = Disable"));
    t.push_back(def(8, "Power-saving setting", 7, 7, "bit", "", 18, 0.0, kEnableInverted,
                    "ATYP item 7.7 bit 18 BUEN: 0 = Enable, 1 = Disable", true, 0, std::nullopt,
                    "UNVERIFIED location; fits the EC-R6S rod (ATYP 0x05350000, IA-OS showed 1:Disable)",
                    "read-only until hardware-verified: a P8 change is expected to also rewrite the STN1 (7.15) tuning "
                    "data, set 8.50 b15 INIT and reset CLD0 / CLD1 to FFFFFFFF (speeds / accelerations re-initialised)"));
    t.push_back(def(9, "Solenoid operation type selection", 8, 2, "enum", "", std::nullopt, 0.0,
                    {{0, "0:Double"}, {3, "3:Single"}, {4, "4:3-position"}, {5, "5:Double (Multi I/O)"}},
                    "enum = the number IA-OS shows (valid set by extensibility level, then <= MXPT 3.16)"));
    t.push_back(def(10, "LED lighting method auto SW setting", 8, 50, "bit", "", 18, 0.0, kEnable, "bit 18: 1 = Enable"));
    t.push_back(def(11, "Multi I/O function selection", 8, 25, "multi_io", "", std::nullopt, 0.0,
                    {{0, "0:Not exist"}, {1, "1:Homing"}, {2, "2:Pause"}, {3, "3:Zone 1"}, {4, "4:Miss-pressing"},
                     {5, "5:Minor fault"}, {6, "6:AUTO/MANU switching"}, {7, "7:Move middle point"}},
                    "byte 1 (bits 8-15) of FI00 8.25 and FO00 8.29 as a pair", true, 0, 29, kUnverifiedItem,
                    kReadOnlyUnverified));
    t.push_back(def(12, "Move command type", 8, 1, "bit", "", 16, 0.0,
                    {{0, "0:Continuous energization"}, {1, "1:Instantaneous energization"}},
                    "bit 16: 1 = instantaneous (momentary)"));
    t.push_back(def(13, "Push back error", 9, 15, "bit", "", 10, 0.0, kEnable, "FSTP item 9.15 bit 10 PSER", true, 0,
                    std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    t.push_back(def(14, "Zone output position (1) +", 0x0C, 1, "mm", "mm", std::nullopt, 0.0, {},
                    "ZNM 0xC.1 entry 0, s32 0.01 mm", true, 0, std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    t.push_back(def(15, "Zone output position (1) -", 0x0C, 2, "mm", "mm", std::nullopt, 0.0, {},
                    "ZNL 0xC.2 entry 0, s32 0.01 mm", true, 0, std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    t.push_back(def(16, "ACR operation ready signal selection", 8, 50, "bit", "", 10, 0.0, kEnable,
                    "SPIO item 8.50 bit 10 ORSS", true, 0, std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    t.push_back(def(17, "Zone output position (2) +", 0x0C, 1, "mm", "mm", std::nullopt, 0.0, {},
                    "ZNM 0xC.1 entry 1, s32 0.01 mm", true, 1, std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    t.push_back(def(18, "Zone output position (2) -", 0x0C, 2, "mm", "mm", std::nullopt, 0.0, {},
                    "ZNL 0xC.2 entry 1, s32 0.01 mm", true, 1, std::nullopt, kUnverifiedItem, kReadOnlyUnverified));
    return t;
}

// The user-parameter candidates IA-OS lists per (system, unit) code. Some are then hidden by the runtime rules; those
// rules predict what the PC tool showed on the gripper and on the rod. A code without a row gets every mapped item.
struct Candidates {
    int system, unit;
    std::vector<int> numbers;
};
const std::vector<Candidates>& candidate_table() {
    static const std::vector<Candidates> table = {
        {0xEC, 0x00, {1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x01, {1, 2, 3, 4, 5, 6, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x02, {1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x03, {1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x08, {1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x09, {1, 2, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x0A, {1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xEC, 0x0B, {1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
        {0xED, 0x00, {1, 2, 3, 4, 5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18}},
    };
    return table;
}

// P9 valid values per extensibility level (UNVERIFIED); level >= 2 with EXFN = 0 -> {0, 3}.
std::vector<int> p9_valid_for_level(int level) {
    switch (level) {
        case 0: return {0, 3};
        case 1: return {0, 3, 4};
        case 2: return {0, 3, 4, 5};
        case 3: return {0, 3, 4};
        case 4: return {0, 3, 4, 5};
        default: return {0, 3};
    }
}

constexpr std::uint32_t kSpioExfn = 1u << 13;
constexpr std::uint32_t kSup2Cnwl = 1u << 23;  // wireless module (P7 shown only with it)
constexpr std::uint32_t kSup3Sors = 1u << 5;   // ACR operation ready signal (P16 shown only with it)
constexpr int kAxisItemStn1 = 15;              // smart tuning actuator / lead numbers (FFFFFFFF = none)
constexpr int kParamItemSpio = reg::kParamItemSpio;

const ClassItem* class_item(const std::map<int, ClassBlock>& classes, int class_id, int item_id, int entry = 0) {
    const auto it = classes.find(class_id);
    return it == classes.end() ? nullptr : it->second.item(item_id, entry);
}

std::string numbers_text(const std::vector<int>& values) {
    std::string out = "(";
    for (std::size_t i = 0; i < values.size(); ++i) out += (i ? ", " : "") + std::to_string(values[i]);
    if (values.size() == 1) out += ",";
    return out + ")";
}

bool has_acr_option(const std::optional<std::string>& model_name) {
    // the model string carries option ACR (RCON-EC connection, ME3816-6B p.1-9) as an option code: a hyphen-separated
    // token after the type and stroke; the ACR inside a clean-room type name such as S6SACR does not count
    if (!model_name || model_name->empty()) return false;
    const std::vector<std::string> tokens = util::split(*model_name, '-');
    for (std::size_t i = 2; i < tokens.size(); ++i)
        if (util::upper(util::trim(tokens[i])) == "ACR") return true;
    return false;
}

constexpr double kSmoothAccelLimitS = 2.0;  // P5: no ramp longer than this (DOC ME3816-6B p.6-11)

}  // namespace

namespace ec_detail {

const char* const kHomeWarning =
    "P3 / P4 are home parameters. After a committed P4 change + software reset the WA (battery-less absolute) gripper "
    "came up with alarm 0x0EE 'ABS error (2)' (home lost); only an alarm reset + home return cleared it "
    "(2026-10-27). Expect the same for P3 / P4 on any absolute (WA) unit; an incremental unit loses its home at every "
    "software reset anyway. Either way: alarm reset (if any) + home return before the next move.";

bool home_pattern_settable(std::optional<std::uint32_t> raw) { return raw && *raw != 0 && *raw != 0xFFFFFFFFu; }

bool is_rotary(const ParameterContext& ctx) {
    // AKID (item 7.33) bits 0-7 = 1, from class 7; without AKID the ATYP 7.7 rotary bit; false when neither is known
    if (const ClassItem* akid = class_item(ctx.classes, reg::kLimitParamsCategory, reg::kAxisItemAkid))
        return (akid->raw & 0xFF) == static_cast<std::uint32_t>(reg::kAkidRotary);
    if (const ClassItem* atyp = class_item(ctx.classes, reg::kLimitParamsCategory, reg::kAxisItemAtyp))
        return (atyp->raw & reg::kAtypRotary) != 0;
    return ctx.units == "deg";
}

std::string item_note(int class_id, int item_id) {
    struct Row {
        int cls, item;
        const char* text;
    };
    static const Row rows[] = {
        {0x0003, 1, "CTCL: system / unit code + firmware (same as FC11)"},
        {0x0003, 2, "MAXP: maximum position count (3 = 3-position)"},
        {0x0003, 10, "present alarm code (followed ALMC 0x0EE on the gripper)"},
        {0x0003, 16, "MXPT: highest allowed P9 value"},
        {0x0003, 52, "SRWT: software reset wait, ms"},
        {0x0003, 67, "MINI: minimum positioning band (P2 lower limit with CINP + RINP)"},
        {0x0004, 4, "CNOW: motor current"},
        {0x0004, 5, "MCUR: rated current (IA-OS shows CNOW / MCUR x 100 %)"},
        {0x0007, 1, "LIMM: upper soft limit = P1 + 0.30 mm (0.01 mm)"},
        {0x0007, 2, "LIML: lower soft limit (0.01 mm)"},
        {0x0007, 3, "LEAD: screw lead (0.01 mm)"},
        {0x0007, 4, "MAXV: maximum speed (0.01 mm/s; the driver's speed cap)"},
        {0x0007, 5, "MAXA: maximum acceleration (0.01 G; the driver's accel cap)"},
        {0x0007, 6, "MAXD: maximum deceleration (0.01 G; the driver's decel cap)"},
        {0x0007, 7, "ATYP: axis type bits (b18 BUEN = P8, b24 RINP)"},
        {0x0007, 8, "MXPH: maximum push speed (0.01 mm/s)"},
        {0x0007, 9, "MXPP: maximum push current % (the hard push cap; 0 = no push)"},
        {0x0007, 15, "STN1: smart tuning actuator / lead numbers"},
        {0x0007, 33, "AKID: actuator kind (bits 0-7)"},
        {0x0007, 34, "TCPC: push force conversion factor (0 = no push)"},
        {0x0007, 35, "TCOA: push force conversion offset"},
        {0x0007, 58, "MNPP: recommended minimum push % (a warning, not a limit)"},
        {0x0008, 3, "BRSL: power-up baud code (6 = 38400)"},
        {0x0008, 4, "RTIM: min. delay before the EC's transmitter turns on, ms"},
        {0x0008, 10, "VCMD: default command speed (0x0812; not the maximum, which is MAXV 7.4)"},
        {0x0008, 11, "ACMD: default acceleration (0.01 G)"},
        {0x0008, 15, "PSHV: push speed (0.01 mm/s, capped by MXPH)"},
        {0x0008, 54, "FMNT: servo monitor mode (b0-3, 0 = off) / buffering cycle (b16-31)"},
        {0x0009, 16, "ORG: home pattern number (P3 = parity); 0x45 home op1"},
        {0x000C, 1, "zone boundary ZNM (0.01 mm)"},
        {0x000C, 2, "zone boundary ZNL (0.01 mm)"},
    };
    for (const Row& r : rows)
        if (r.cls == class_id && r.item == item_id) return r.text;
    return std::string();
}

Decoded decode(const ParamDef& p, const ClassItem* item, const ClassItem* item2) {
    Decoded out;
    out.key = p.key();
    if (item == nullptr || (p.kind == "multi_io" && item2 == nullptr)) {
        out.found = false;
        out.text = "not found on this EC";
        return out;
    }
    out.found = true;
    out.address = util::format("0x%04X", item->address);
    out.raw_hex = item->hex();
    const auto choice = [&](int value, const std::string& fallback) {
        const auto it = p.choices.find(value);
        return it == p.choices.end() ? fallback : it->second;
    };
    if (p.kind == "mm") {
        const double value = util::round2(item->signed_value() * reg::kPositionScale + p.offset_mm);
        out.value = ParamValue::of(value);
        out.text = util::fixed(value, 2) + " mm";
    } else if (p.kind == "bit") {
        const int value = static_cast<int>((item->raw >> p.bit.value_or(0)) & 1u);
        out.value = ParamValue::of_whole(value);  // the codes are ints in the Python edition, only the mm value a float
        out.text = choice(value, std::to_string(value));
    } else if (p.kind == "parity") {
        out.pattern = item->raw;
        if (!home_pattern_settable(item->raw)) {
            out.text = util::format("not settable (home pattern ORG = 0x%08X)", static_cast<unsigned>(item->raw));
        } else {
            const int value = static_cast<int>(item->raw & 1u);
            out.value = ParamValue::of_whole(value);
            out.text = choice(value, std::to_string(value));
        }
    } else if (p.kind == "multi_io") {
        const int fi = static_cast<int>((item->raw >> 8) & 0xFF);
        const int fo = static_cast<int>((item2->raw >> 8) & 0xFF);
        out.raw_hex = item->hex() + "/" + item2->hex();
        out.address = util::format("0x%04X/0x%04X", item->address, item2->address);
        bool known = false;
        int number = 0;
        for (const PairRow& row : kMultiIoPairs)
            if (row.fi == fi && row.fo == fo) {
                known = true;
                number = row.no;
            }
        if (known) {
            out.value = ParamValue::of_whole(number);
            out.text = choice(number, "");
        } else {
            out.text = util::format("unknown pair (%d, %d)", fi, fo);
        }
    } else {
        out.value = ParamValue::of_whole(static_cast<double>(item->raw));
        out.text = choice(static_cast<int>(item->raw), std::to_string(item->raw) + ": unknown");
    }
    return out;
}

std::optional<std::vector<int>> p9_valid_values(std::optional<int> level, std::optional<bool> exfn,
                                                std::optional<int> mxpt) {
    if (!level) return std::nullopt;
    std::vector<int> valid = p9_valid_for_level(*level);
    if (*level >= 2 && exfn.has_value() && !*exfn) valid = {0, 3};
    if (mxpt) {
        std::vector<int> kept;
        for (int v : valid)
            if (v <= *mxpt) kept.push_back(v);
        valid = kept;
    }
    return valid;
}

std::map<int, ParameterVisibility> visibility(const ParameterContext& ctx) {
    const auto& classes = ctx.classes;
    std::optional<int> level;
    if (ctx.accl) level = static_cast<int>((*ctx.accl >> 8) & 0xF);
    const ClassItem* spio = class_item(classes, reg::kMotionParamsCategory, kParamItemSpio);
    const std::optional<bool> exfn = spio ? std::optional<bool>((spio->raw & kSpioExfn) != 0) : std::nullopt;
    std::optional<std::uint32_t> sup2, sup3;
    if (ctx.support) {
        const auto a = ctx.support->find("SUP2");
        if (a != ctx.support->end()) sup2 = a->second;
        const auto b = ctx.support->find("SUP3");
        sup3 = b != ctx.support->end() ? b->second : 0u;
    }
    const ClassItem* stn1 = class_item(classes, reg::kLimitParamsCategory, kAxisItemStn1);
    const ClassItem* p9 = class_item(classes, reg::kMotionParamsCategory, 2);
    const ClassItem* mxpt = class_item(classes, reg::kControllerMonitorCategory, reg::kControllerItemMxpt);
    const std::string lvl = level ? std::to_string(*level) : "unknown";

    auto rule = [](std::optional<bool> shown, const std::string& why) {
        ParameterVisibility v;
        v.shown = shown;
        v.reason = why;
        return v;
    };
    const std::optional<bool> ext = (!level || !exfn) ? std::nullopt : std::optional<bool>(*level > 1 && *exfn);

    std::map<int, ParameterVisibility> out;
    for (const ParamDef& p : ec_user_parameters()) out[p.no] = rule(true, "");
    out[2] = rule(ext ? std::optional<bool>(!*ext) : std::nullopt, "hidden when level >= Phase1 and EXFN = 1");
    out[7] = rule(sup2 ? std::optional<bool>((*sup2 & kSup2Cnwl) != 0) : std::nullopt,
                  "shown only with a wireless module (SUP2 CNWL)");
    out[8] = rule(stn1 ? std::optional<bool>(stn1->raw != 0xFFFFFFFFu) : std::nullopt,
                  "shown only with smart-tuning data (STN1 7.15 set; an approximation)");
    const std::optional<std::vector<int>> valid =
        p9_valid_values(level, exfn, mxpt ? std::optional<int>(static_cast<int>(mxpt->raw)) : std::nullopt);
    std::optional<bool> p9_shown;
    if (valid && p9) p9_shown = std::find(valid->begin(), valid->end(), static_cast<int>(p9->raw)) != valid->end();
    out[9] = rule(p9_shown, "the present value must be in the valid set " + (valid ? numbers_text(*valid) : "None") +
                                " (level " + lvl + ", <= MXPT)");
    const ClassItem* fi = class_item(classes, reg::kMotionParamsCategory, 25);
    const ClassItem* fo = class_item(classes, reg::kMotionParamsCategory, 29);
    bool known_pair = false;
    if (fi != nullptr && fo != nullptr) {
        const int a = static_cast<int>((fi->raw >> 8) & 0xFF);
        const int b = static_cast<int>((fo->raw >> 8) & 0xFF);
        for (const PairRow& row : kMultiIoPairs)
            if (row.fi == a && row.fo == b) known_pair = true;
    }
    const std::optional<bool> p11 = ext ? std::optional<bool>(*ext && *level != 3 && known_pair) : std::nullopt;
    out[11] = rule(p11, "needs level > Position3, not Phase2, EXFN = 1 and a known FI00/FO00 pair (level " + lvl + ")");
    for (int no : {13, 14, 15}) out[no] = rule(ext, "needs level > Position3 and EXFN = 1 (level " + lvl + ")");
    out[16] = rule(ctx.support ? std::optional<bool>((sup3.value_or(0) & kSup3Sors) != 0) : std::nullopt,
                   "shown only with SUP3 SORS");
    for (int no : {17, 18})
        out[no] = rule(ext ? std::optional<bool>(*ext && *level > 2) : std::nullopt,
                       "needs level > Phase1 and EXFN = 1 (level " + lvl + ")");
    return out;
}

ParamValue parse_value(const ParamDef& p, const ParamValue& value) {
    if (p.kind == "mm") {
        double number;
        if (value.number) {
            number = *value.number;
        } else {
            const auto parsed = util::parse_double(value.text);
            if (!parsed)
                throw std::invalid_argument(util::format("P%d needs a number in mm, got '%s'", p.no, value.text.c_str()));
            number = *parsed;
        }
        if (!std::isfinite(number))
            throw std::invalid_argument(
                util::format("P%d needs a finite number in mm, got %s", p.no, util::g(number).c_str()));
        return ParamValue::of(util::round2(number));
    }
    std::string allowed;
    for (const auto& kv : p.choices) allowed += (allowed.empty() ? "" : ", ") + kv.second;
    const std::string shown = value.number ? util::g(*value.number) : "'" + value.text + "'";
    int number = 0;
    if (!value.number) {
        const std::string text = util::lower(util::trim(value.text));
        std::string digits = text;
        while (!digits.empty() && (digits[0] == '+' || digits[0] == '-')) digits.erase(0, 1);
        bool numeric = !digits.empty();
        for (char c : digits)
            if (c < '0' || c > '9') numeric = false;
        if (numeric) {
            number = static_cast<int>(util::parse_int(text).value_or(0));
        } else {
            std::vector<int> exact, prefix;
            for (const auto& kv : p.choices) {
                const std::string label = util::lower(kv.second);
                const std::size_t colon = label.find(':');
                const std::string name = util::trim(colon == std::string::npos ? label : label.substr(colon + 1));
                if (text == label || text == name) exact.push_back(kv.first);
                if (!text.empty() && util::starts_with(name, text)) prefix.push_back(kv.first);
            }
            const std::vector<int>& found = !exact.empty() ? exact : prefix;
            if (found.size() != 1)
                throw std::invalid_argument(
                    util::format("P%d (%s): %s is not one of %s", p.no, p.name.c_str(), shown.c_str(), allowed.c_str()));
            number = found[0];
        }
    } else {
        const double n = *value.number;
        if (!std::isfinite(n) || n != std::floor(n))
            throw std::invalid_argument(
                util::format("P%d (%s): %s is not one of %s", p.no, p.name.c_str(), shown.c_str(), allowed.c_str()));
        number = static_cast<int>(n);
    }
    if (p.choices.find(number) == p.choices.end())
        throw std::invalid_argument(
            util::format("P%d (%s): %s is not one of %s", p.no, p.name.c_str(), shown.c_str(), allowed.c_str()));
    return ParamValue::of(number);
}

std::uint32_t encode(const ParamDef& p, std::uint32_t raw, double number) {
    if (p.read_only || p.kind == "multi_io")
        throw std::invalid_argument(util::format("P%d (%s) is read-only in this driver", p.no, p.name.c_str()));
    if (p.kind == "mm") {
        const std::int64_t counts = reg::to_counts(number - p.offset_mm);
        if (counts < -(std::int64_t(1) << 31) || counts >= (std::int64_t(1) << 31))
            throw std::invalid_argument(
                util::format("P%d: %s mm does not fit a 32-bit item", p.no, util::g(number).c_str()));
        return static_cast<std::uint32_t>(counts & 0xFFFFFFFFll);
    }
    if (p.choices.find(static_cast<int>(number)) == p.choices.end()) {
        std::string allowed;
        for (const auto& kv : p.choices) allowed += (allowed.empty() ? "" : ", ") + kv.second;
        throw std::invalid_argument(
            util::format("P%d: %s is not one of its numbers (%s)", p.no, util::g(number).c_str(), allowed.c_str()));
    }
    const int n = static_cast<int>(number);
    if (p.kind == "bit") {
        const std::uint32_t mask = 1u << p.bit.value_or(0);
        return (raw & ~mask) | (n ? mask : 0u);
    }
    if (p.kind == "parity") {
        if (!home_pattern_settable(raw))
            throw std::invalid_argument(util::format(
                "P%d: the home pattern ORG is 0x%08X: the home direction is not settable", p.no, static_cast<unsigned>(raw)));
        if (static_cast<int>(raw & 1u) == n) return raw;  // already the requested direction
        return n == 1 ? raw - 1 : raw + 1;
    }
    return static_cast<std::uint32_t>(n);  // enum: the whole item
}

std::vector<std::string> guard(const ParamDef& p, double number, bool rotary,
                               const std::optional<std::string>& model_name) {
    // The manual's rules for writing EC user parameter p = number (ME3816-6B). Returns the warnings to report; throws
    // UnsafeCommandError (nothing written) when a rule refuses. A P4 write may go below the value at shipping (the
    // home parameters need no opt-in), so the floor is not checked here.
    std::vector<std::string> warnings;
    if (p.no == 4)
        warnings.push_back(
            "P4: adjust P1 (operation range) together with P4 - P1's datum is the home position - and do a home return "
            "(absolute reset) after the change (ME3816-6B p.6-10)");
    if (p.no == 3) {
        if (rotary)
            throw UnsafeCommandError(
                "P3 (home direction) on a rotary axis: changing P3 alone does not reverse the home position (the "
                "stopper must be swapped, ME3800-5E p.1-36; some models cannot reverse at all, ME3816-6B Intro-17); nothing written");
        warnings.push_back(
            "P3: the direction of every move reverses; a home return (absolute reset) is required afterwards "
            "(ME3816-6B p.6-9)");
    }
    if (p.no == 9 && static_cast<int>(number) == 3 && has_acr_option(model_name))
        throw UnsafeCommandError(
            "P9 = Single on a unit with option ACR (RCON-EC connection): it must stay Double, or the EC may not operate as "
            "commanded (ME3816-6B p.6-15, Intro-9); nothing written");
    if (p.no == 5 && static_cast<int>(number) == 1)
        warnings.push_back(util::format(
            "P5 enabled: no acceleration or deceleration may take longer than %s s, avoid pausing during a ramp "
            "(ME3816-6B p.6-11), and a new command issued too soon during a move can raise an overload alarm "
            "(ME3816-6B p.8-21 cause 7)",
            util::g(kSmoothAccelLimitS).c_str()));
    if (p.no == 10 && static_cast<int>(number) == 1)
        warnings.push_back("P10: may have no effect outside the small models listed in ME3816-6B p.6-16");
    if (p.no >= 11)  // P11-P18 are in IA-OS's list, not in ME3816-6B (info's iaos_only)
        warnings.push_back(util::format("P%d is not in ME3816-6B (IA-OS lists it): its meaning is UNVERIFIED", p.no));
    return warnings;
}

void check_value(const ParamDef& p, double number, const Decoded& before, const ParameterContext& ctx,
                 std::optional<double> p1_ceiling) {
    // Driver sanity ranges and value rules for a parameter write (SoftLimitError, nothing written). The value rules are
    // UNVERIFIED on hardware and applied when their inputs are readable: P9 only from the valid set for the
    // extensibility level and <= MXPT; P12 != 0 refused while P9 is 3 or 4; P2 >= MINI when SUP2 CINP and ATYP RINP are
    // set; P3 only when the home pattern ORG is neither 0 nor FFFFFFFF.
    const auto outside = [&](const std::string& text) {
        return SoftLimitError(util::format("P%d (%s) = %s: %s; nothing written", p.no, p.name.c_str(),
                                           util::g(number).c_str(), text.c_str()));
    };
    const auto& classes = ctx.classes;
    if (p.no == 3 && !home_pattern_settable(before.pattern))
        throw outside("the home pattern ORG (item 9.16) is 0 / FFFFFFFF: P3 is not settable on this unit");
    if (p.no == 9) {
        const ClassItem* spio = class_item(classes, reg::kMotionParamsCategory, kParamItemSpio);
        const ClassItem* mxpt = class_item(classes, reg::kControllerMonitorCategory, reg::kControllerItemMxpt);
        std::optional<int> level;
        if (ctx.accl) level = static_cast<int>((*ctx.accl >> 8) & 0xF);
        const auto valid = p9_valid_values(level, spio ? std::optional<bool>((spio->raw & kSpioExfn) != 0) : std::nullopt,
                                           mxpt ? std::optional<int>(static_cast<int>(mxpt->raw)) : std::nullopt);
        if (valid && std::find(valid->begin(), valid->end(), static_cast<int>(number)) == valid->end()) {
            std::string list;
            for (int v : *valid) list += (list.empty() ? "" : ", ") + std::to_string(v);
            throw outside("this unit accepts only " + list + " (extensibility level, MXPT; UNVERIFIED rule)");
        }
    }
    if (p.no == 12 && number != 0) {
        const ClassItem* p9 = class_item(classes, reg::kMotionParamsCategory, 2);
        if (p9 != nullptr && (p9->raw == 3 || p9->raw == 4))
            throw outside(util::format(
                "the driver refuses P12 != 0 while P9 is %u (3 single / 4 3-position; UNVERIFIED rule)",
                static_cast<unsigned>(p9->raw)));
    }
    if (p.no == 2 && ctx.support) {
        const auto sup2 = ctx.support->find("SUP2");
        const ClassItem* atyp = class_item(classes, reg::kLimitParamsCategory, reg::kAxisItemAtyp);
        const ClassItem* mini = class_item(classes, reg::kControllerMonitorCategory, reg::kControllerItemMini);
        if (sup2 != ctx.support->end() && (sup2->second & reg::kSup2Cinp) != 0 && atyp != nullptr &&
            (atyp->raw & reg::kAtypRinp) != 0 && mini != nullptr && mini->raw != 0) {
            const double minimum = mini->raw * reg::kPositionScale;
            if (number < minimum - kPosEps)
                throw outside(util::format("below this EC's minimum positioning band MINI (3.67) %.2f mm", minimum));
        }
    }

    if (p.no == 1) {
        if (!p1_ceiling || !(kP1MinMm <= number && number <= *p1_ceiling + kPosEps))
            throw outside(util::format("allowed %.2f..%s mm (P1 can only narrow the stroke, up to the largest value read in "
                                       "this session; widen it again with IA-OS)",
                                       kP1MinMm, p1_ceiling ? util::fixed(*p1_ceiling, 2).c_str() : "nan"));
        if (ctx.position_mm && number < *ctx.position_mm - kPosEps)
            throw outside(util::format("below the current position %.2f mm (the axis would be outside the range)",
                                       *ctx.position_mm));
    } else if (p.no == 2 && !(kP2MinMm <= number && number <= kP2MaxMm)) {
        throw outside(util::format("allowed %.2f..%.2f mm (driver sanity range)", kP2MinMm, kP2MaxMm));
    } else if (p.no == 4 && !(kP4MinMm <= number && number <= kP4MaxMm)) {
        throw outside(util::format("allowed %.2f..%.2f mm (driver sanity range)", kP4MinMm, kP4MaxMm));
    } else if (p.no == 9 && (number == 4 || number == 5) &&
               !(before.value.number && *before.value.number == number)) {
        throw outside(util::format(
            "%d is accepted only on a unit already in that mode (driver rule: it changes the PIO layout)",
            static_cast<int>(number)));
    }
}

}  // namespace ec_detail

// ------------------------------------------------------------------------------------------ public

std::string ParamDef::key() const {
    const std::string cls = class_id > 9 ? util::format("%X", class_id) : std::to_string(class_id);
    std::string out = cls + "." + std::to_string(item_id) + (entry != 0 ? "[" + std::to_string(entry) + "]" : std::string());
    if (item_id2) out += "+" + cls + "." + std::to_string(*item_id2);
    return out;
}

const std::vector<ParamDef>& ec_user_parameters() {
    static const std::vector<ParamDef> table = build_table();
    return table;
}

const ParamDef* ec_user_parameter(int no) {
    for (const ParamDef& p : ec_user_parameters())
        if (p.no == no) return &p;
    return nullptr;
}

bool is_ec_home_parameter(int no) { return no == 3 || no == 4; }

ParameterModel model_parameter_list(const ParameterContext& ctx) {
    ParameterModel model;
    model.unit_code = ctx.unit_code;
    model.system_code = ctx.system_code;
    model.model_name = ctx.model_name;

    if (!ctx.is_ec) {
        // A CON-family controller: the numbered parameters whose item is in the unit's ID map (con_parameters.cpp); there is
        // no per-model list.
        model.family = ctx.family;
        model.known = false;
        model.verified = false;
        for (const ConParam& p : con_parameters(ctx.family)) {
            const auto it = ctx.classes.find(p.class_id);
            if (it == ctx.classes.end()) continue;
            const ClassItem* item = it->second.item(p.item_id, p.entry);
            if (item != nullptr && item->size == 2) model.listed.push_back(p.no);
        }
        model.note = "the numbered parameters of the unit's ID map (" + ctx.family + ")";
        return model;
    }

    const int system = ctx.system_code.value_or(reg::kSystemEc);
    const std::map<int, ParameterVisibility> vis = ec_detail::visibility(ctx);
    const std::vector<int>* candidates = nullptr;
    if (ctx.unit_code)
        for (const Candidates& c : candidate_table())
            if (c.system == system && c.unit == *ctx.unit_code) candidates = &c.numbers;
    model.family = reg::ec_family(system, ctx.unit_code);

    if (candidates != nullptr) {
        model.known = true;
        model.verified = true;  // kept for compatibility: equals known
        model.candidates = *candidates;
        for (int no : *candidates) {
            const auto it = vis.find(no);
            if (it != vis.end() && it->second.shown.has_value() && !*it->second.shown)
                model.hidden.emplace_back(no, it->second.reason);
            else
                model.listed.push_back(no);
        }
        const std::string family = model.family ? *model.family : std::string("?");
        model.note = util::format("the parameters IA-OS lists for unit 0x%02X (%s)", *ctx.unit_code, family.c_str());
        return model;
    }

    // a unit code without an IA-OS list here (a new family), or not readable: every mapped parameter is offered
    model.known = false;
    model.verified = false;
    for (const ParamDef& p : ec_user_parameters()) model.listed.push_back(p.no);
    if (!ctx.unit_code)
        model.note = "unit code not readable (FC11 / CTCL): every mapped item found on the unit is shown";
    else
        model.note = util::format("unit code 0x%02X: every mapped item found on the unit is shown", *ctx.unit_code);
    return model;
}

}  // namespace iai_actuator_driver
