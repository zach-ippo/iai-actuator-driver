// Lookups of the register map (registers.hpp): product family, EC family, actuator kind, directory category names and
// extents, text decoding, EC series and the documented minimum speed. Port of registers.py (C# Registers.cs +
// Registers.Tables.cs). No I/O.
#include "iai_actuator_driver/registers.hpp"

#include <algorithm>
#include <cmath>

#include "util.hpp"

namespace iai_actuator_driver {
namespace reg {

namespace {

struct IntText {
    int key;
    const char* text;
};

const char* find(const IntText* table, std::size_t count, int key) {
    for (std::size_t i = 0; i < count; ++i)
        if (table[i].key == key) return table[i].text;
    return nullptr;
}
template <std::size_t N>
const char* find(const IntText (&table)[N], int key) {
    return find(table, N, key);
}

/// a x b x c held at the int range: a directory record's words multiply past 2^31 (DQ 65535 x MS 65535 x LQ 4096),
/// and a signed int overflow is undefined. The double only decides whether the exact product fits.
int words_product(long long a, long long b, long long c = 1) {
    const double p = static_cast<double>(a) * static_cast<double>(b) * static_cast<double>(c);
    if (p >= 2147483647.0) return 2147483647;
    if (p <= -2147483648.0) return -2147483647 - 1;
    return static_cast<int>(a * b * c);
}

// FC11 system code (byte 0) names (DOC, from IAI manuals).
const IntText kSystemNames[] = {
    {0xEC, "EC 24 V"}, {0xED, "EC 200 V"}, {0xAE, "PCON"}, {0xAC, "ACON / DCON"}, {0xAA, "SCON"},
};

// Product family per FC11 system code. Any other code is a generic IAI controller and works the same way: every range and
// capability comes from the unit's own directory, never from this table.
const IntText kProductFamilies[] = {
    {0xEC, "EC"}, {0xED, "EC"}, {0xAE, "PCON"}, {0xAC, "ACON/DCON"}, {0xAA, "SCON"},
};

struct EcFamilyRow {
    int system;
    int unit;
    const char* name;
};

// EC family names per (system, unit) code (from IA-OS); any other code is driven the same way. The family is NOT the
// actuator kind (that is AKID, item 7.33).
const EcFamilyRow kEcFamilies[] = {
    {0xEC, 0x00, "EC standard"},
    {0xEC, 0x01, "EC slim/small"},
    {0xEC, 0x02, "EC dust/drip-proof"},
    {0xEC, 0x03, "EC high thrust"},
    {0xEC, 0x08, "EC(UM) ultra-compact"},
    {0xEC, 0x09, "EC(T) thin"},
    {0xEC, 0x0A, "EC(MS) \"2-size EC\""},
    {0xEC, 0x0B, "EC(UC) ultra-ultra-compact"},
    {0xED, 0x00, "EC(200V)"},
};

// AKID (item 7.33) bits 0-7 -> actuator kind (VERIFIED: slider 9, gripper 4, rod 3; the other values UNVERIFIED).
const IntText kActuatorKinds[] = {
    {0, "slider"}, {1, "rotary"}, {2, "arm"},   {3, "rod"},           {4, "gripper"}, {5, "other"},
    {6, "linear"}, {7, "scalar"}, {8, "table"}, {9, "slider (small)"}, {10, "rod (small)"}, {11, "wire"},
};

// What each directory category holds ("?" = content not understood).
const IntText kCategoryNames[] = {
    {0x0001, "support functions (SUPT, MVPT, SPTS, SUP2, SUP3)"},
    {0x0002, "status image (DSS1 .. SSSE)"},
    {0x0003, "controller monitor (CTL, current alarm, ...)"},
    {0x0004, "second monitor block"},
    {0x0005, "alarm history"},
    {0x0006, "identification (serials, ACCL, unit name, model)"},
    {0x0007, "axis parameters (soft limits = P1, MAXV / MAXA / MAXD, push limits, AKID)"},
    {0x0008, "parameters (user parameters 2, 5-7, 9-12, 16; BRSL, move defaults, FMNT)"},
    {0x0009, "parameters (user parameters 3, 4, 13; ORG home pattern)"},
    {0x000C, "zone boundaries ZNM / ZNL (user parameters 14, 15, 17, 18)"},
    {0x000D, "control registers DRG1 / DRG2 / DRGE / POSR (coil base)"},
    {0x000F, "active move profile"},
    {0x0010, "position table"},
    {0x0012, "?"},
    {0x0013, "PIO input map?"},
    {0x0014, "PIO output map?"},
    {0x0100, "controller system info (AXNO, PCB, FVER, UNIT)"},
    {0x0101, "monitor"},
    {0x0102, "position-number move (POSR)"},
    {0x0103, "direct numeric move"},
    {0x0104, "PCMD / PPOS mirror"},
    {0x0109, "parameter copies"},
    {0x010A, "?"},
    {0x0111, "?"},
    {0x0112, "?"},
    {0x0114, "compact monitor"},
    {0x0200, "maintenance counters"},
};

struct MixedExtent {
    int category;
    int member_count;
    int words;
};

// Blocks with MS = 0 (members of mixed sizes): (DQ, register count) where the count is known. Member names UNVERIFIED;
// the counts are confirmed by reads (VERIFIED).
const MixedExtent kKnownMixedExtents[] = {
    {0x0006, 36, 0x48},  // identification 0x0600-0x0647
    {0x000F, 13, 20},    // active move profile 0x0F00-0x0F13
    {0x0013, 16, 16},    // 0x8100-0x810F, 16 one-word members (PI00..PI15)
    {0x0014, 16, 16},    // 0x8200-0x820F (PO00..PO15)
    {0x0100, 16, 32},    // 0x8000-0x801F, 16 two-word members (FLGS .. CSUM)
    {0x0101, 22, 34},    // monitor 0x9000-0x9021
    {0x0102, 3, 3},      // 0x9800-0x9802
    {0x0103, 7, 10},     // 0x9900-0x9909 (0x9909 is the DRG1 mirror: never write it)
    {0x0104, 3, 4},      // 0x9A00-0x9A03
    {0x0114, 20, 40},    // compact monitor 0x9F00-0x9F27
};

bool is_text_byte(unsigned b) { return (b >= 0x20 && b <= 0x7E) || b == 0; }

// ------------------------------------------------------------------------------------------ minimum speeds
// Documented minimum speeds (units/s) per EC series and lead (mm): a RECOMMENDATION only - below it the unit "may cause
// abnormal noise or unstable speed" (DOC ME3793-10E p.1-24). The driver warns, never refuses.
struct LeadSpeed {
    double lead;
    double speed;
};
struct SeriesTable {
    const char* series;
    const LeadSpeed* rows;
    std::size_t count;
};
const LeadSpeed kS3[] = {{6.0, 8.0}, {4.0, 5.0}, {2.0, 3.0}};
const LeadSpeed kS4[] = {{16.0, 40.0}, {10.0, 30.0}, {5.0, 7.0}, {2.5, 4.0}};
const LeadSpeed kS6[] = {{20.0, 25.0}, {12.0, 15.0}, {6.0, 8.0}, {3.0, 4.0}};
const LeadSpeed kS7[] = {{24.0, 30.0}, {16.0, 20.0}, {8.0, 10.0}, {4.0, 5.0}};
const LeadSpeed kS8[] = {{30.0, 38.0}, {20.0, 25.0}, {10.0, 13.0}, {5.0, 7.0}};
const LeadSpeed kR6[] = {{20.0, 25.0}, {12.0, 15.0}, {6.0, 8.0}, {3.0, 4.0}};
const LeadSpeed kR7[] = {{24.0, 30.0}, {16.0, 20.0}, {8.0, 10.0}, {4.0, 5.0}};
const LeadSpeed kRR8[] = {{10.0, 13.0}, {5.0, 7.0}};
const LeadSpeed kRR10[] = {{5.0, 7.0}, {2.5, 4.0}};
const SeriesTable kMinSpeedByLead[] = {
    {"S3", kS3, 3}, {"S4", kS4, 4}, {"S6", kS6, 4}, {"S7", kS7, 4}, {"S8", kS8, 4},
    {"R6", kR6, 4}, {"R7", kR7, 4}, {"RR8", kRR8, 2}, {"RR10", kRR10, 2},
};
struct SeriesSpeed {
    const char* series;
    double speed;
};
const SeriesSpeed kMinSpeedBySeries[] = {
    {"RTB", 20.0}, {"RTC", 20.0}, {"GRB", 5.0}, {"GRC", 10.0}, {"WER", 5.0}, {"WEGR", 5.0},
};
constexpr double kMinSpeedPerLead = 1.25;  // generic: lead / 800 pulses / 0.001 s (DOC ME0155 p.139)

bool is_digit(char c) { return c >= '0' && c <= '9'; }

/// The series alternatives of EC-D?(RR\d+|R\d|S\d|RT[BC]|GR[BC]|WEGR|WER) at `pos` (text is upper case), in the order of
/// the Python edition's regular expression; empty when none matches.
std::string match_series(const std::string& text, std::size_t pos) {
    auto at = [&](std::size_t i) -> char { return i < text.size() ? text[i] : '\0'; };
    if (at(pos) == 'R' && at(pos + 1) == 'R' && is_digit(at(pos + 2))) {
        std::size_t end = pos + 2;
        while (is_digit(at(end))) ++end;
        return text.substr(pos, end - pos);
    }
    if (at(pos) == 'R' && is_digit(at(pos + 1))) return text.substr(pos, 2);
    if (at(pos) == 'S' && is_digit(at(pos + 1))) return text.substr(pos, 2);
    if (at(pos) == 'R' && at(pos + 1) == 'T' && (at(pos + 2) == 'B' || at(pos + 2) == 'C')) return text.substr(pos, 3);
    if (at(pos) == 'G' && at(pos + 1) == 'R' && (at(pos + 2) == 'B' || at(pos + 2) == 'C')) return text.substr(pos, 3);
    if (text.compare(pos, 4, "WEGR") == 0) return "WEGR";
    if (text.compare(pos, 3, "WER") == 0) return "WER";
    return std::string();
}

}  // namespace

// ------------------------------------------------------------------------------------------ identity

std::string product_family(std::optional<int> system_code) {
    if (!system_code) return kGenericFamily;
    const char* family = find(kProductFamilies, *system_code);
    return family ? family : kGenericFamily;
}

bool is_ec_family(std::optional<int> system_code) {
    return system_code.has_value() && (*system_code == kSystemEc || *system_code == kSystemEc200);
}

std::string system_name(std::optional<int> system_code) {
    if (!system_code) return std::string();
    const char* name = find(kSystemNames, *system_code);
    return name ? name : "";
}

std::optional<std::string> ec_family(std::optional<int> system_code, std::optional<int> unit_code) {
    if (!system_code || !unit_code) return std::nullopt;
    for (const EcFamilyRow& row : kEcFamilies)
        if (row.system == *system_code && row.unit == *unit_code) return std::string(row.name);
    return std::nullopt;
}

std::string axis_units(std::optional<int> actuator_kind, std::optional<bool> rotary_atyp) {
    if (!actuator_kind) return rotary_atyp.value_or(false) ? "deg" : "mm";
    return *actuator_kind == kAkidRotary ? "deg" : "mm";
}

std::string axis_kind_name(std::optional<int> actuator_kind, std::optional<bool> rotary_atyp) {
    if (!actuator_kind) {  // a unit without AKID (a CON controller): the ATYP TYPE bit says rotary or not
        if (!rotary_atyp) return "unknown";
        return *rotary_atyp ? find(kActuatorKinds, kAkidRotary) : "linear";
    }
    const char* name = find(kActuatorKinds, *actuator_kind);
    return name ? std::string(name) : "kind " + std::to_string(*actuator_kind);
}

// ------------------------------------------------------------------------------------------ directory

std::string category_name(int category_id) {
    const char* name = find(kCategoryNames, category_id);
    return name ? name : "?";
}

BlockExtent category_extent(int category_id, int member_count, int list_count, int member_size) {
    BlockExtent extent;
    if (member_size != 0) {
        extent.words = words_product(member_count, member_size, list_count);
        extent.exact = true;
        extent.words_max = extent.words;
        return extent;
    }
    const auto& members = support_members();
    if (category_id == kSupportCategory && member_count > 0 && member_count <= static_cast<int>(members.size())) {
        // class 1 has a fixed member order with known sizes (18 words for 11 members, 8 for the EC-R6S rod's 6)
        int words = 0;
        for (int i = 0; i < member_count; ++i) words += members[static_cast<std::size_t>(i)].second;
        extent.words = words_product(words, list_count);
        extent.exact = true;
        extent.words_max = extent.words;
        return extent;
    }
    for (const MixedExtent& known : kKnownMixedExtents) {
        if (known.category == category_id && known.member_count == member_count) {
            extent.words = words_product(known.words, list_count);
            extent.exact = true;
            extent.words_max = extent.words;
            return extent;
        }
    }
    extent.words = words_product(member_count, list_count);
    extent.exact = false;
    extent.words_max = words_product(2, member_count, list_count);
    return extent;
}

const std::vector<std::pair<std::string, int>>& support_members() {
    static const std::vector<std::pair<std::string, int>> members = {
        {"SUPT", 2}, {"DIAG", 1}, {"MVPT", 1}, {"SPTS", 1}, {"CTYP", 1}, {"SUP2", 2},
        {"AEHS", 2}, {"ACHS", 2}, {"ARSF", 2}, {"SPTM", 2}, {"SUP3", 2},
    };
    return members;
}

// ------------------------------------------------------------------------------------------ text

std::string words_to_ascii(const std::vector<std::uint16_t>& words) {
    std::string text;
    for (std::uint16_t w : words) {
        for (unsigned b : {static_cast<unsigned>(w >> 8), static_cast<unsigned>(w & 0xFF)}) {
            if (b >= 0x20 && b <= 0x7E)
                text += static_cast<char>(b);
            else if (b == 0)
                text += ' ';
        }
    }
    std::size_t begin = 0, end = text.size();
    while (begin < end && text[begin] == ' ') ++begin;
    while (end > begin && text[end - 1] == ' ') --end;
    return text.substr(begin, end - begin);
}

std::optional<std::string> words_to_text(const std::vector<std::uint16_t>& words) {
    for (std::uint16_t w : words)
        if (!is_text_byte(w >> 8) || !is_text_byte(w & 0xFF)) return std::nullopt;
    std::string text = words_to_ascii(words);
    if (text.empty()) return std::nullopt;
    return text;
}

std::string printable(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char c : text) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (u >= 0x20 && u <= 0x7E) out += c;
    }
    return out;
}

// ------------------------------------------------------------------------------------------ EC series / speeds

std::optional<std::string> ec_series(const std::optional<std::string>& model) {
    if (!model || model->empty()) return std::nullopt;
    const std::string text = util::upper(*model);
    for (std::size_t i = 0; i + 3 <= text.size(); ++i) {
        if (text.compare(i, 3, "EC-") != 0) continue;
        const std::size_t after = i + 3;
        if (after < text.size() && text[after] == 'D') {  // D? is greedy; fall back to no D when nothing follows it
            const std::string with_d = match_series(text, after + 1);
            if (!with_d.empty()) return with_d;
        }
        const std::string plain = match_series(text, after);
        if (!plain.empty()) return plain;
    }
    return std::nullopt;
}

std::pair<std::optional<double>, std::string> recommended_min_speed(const std::optional<std::string>& model,
                                                                    std::optional<double> lead_mm) {
    const std::optional<std::string> series = ec_series(model);
    if (series) {
        for (const SeriesSpeed& row : kMinSpeedBySeries)
            if (*series == row.series) return {row.speed, "series " + *series + " (DOC)"};
        if (lead_mm) {
            for (const SeriesTable& table : kMinSpeedByLead) {
                if (*series != table.series) continue;
                for (std::size_t i = 0; i < table.count; ++i) {
                    if (std::fabs(table.rows[i].lead - *lead_mm) < 0.005)
                        return {table.rows[i].speed,
                                util::format("series %s, lead %g mm (DOC)", series->c_str(), table.rows[i].lead)};
                }
            }
        }
    }
    if (lead_mm && *lead_mm > 0)
        return {util::round2(*lead_mm * kMinSpeedPerLead),
                util::format("lead %g mm x 1.25 (DOC ME0155 p.139)", *lead_mm)};
    return {std::nullopt, "no documented minimum known for this unit"};
}

}  // namespace reg
}  // namespace iai_actuator_driver
