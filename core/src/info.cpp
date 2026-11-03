// Read-only unit information and the read-only backup (info.hpp). Port of the Python edition's info.py (unit_info,
// backup, backup_axes, the bit tables and decoders, the CON range check of the parameters section) and of the backup
// readers of recovery.py. READ-ONLY: FC03 + FC11 only. The JSON layout and the texts are the Python edition's, so
// reports and backups interchange; this C++ code is UNVERIFIED on hardware.
#include "iai_actuator_driver/info.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/version.hpp"
#include "axis_impl.hpp"
#include "model_detail.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace info {

namespace {

using Words = std::vector<std::uint16_t>;
using SupportWords = std::optional<std::map<std::string, std::uint32_t>>;

// ------------------------------------------------------------------------------------------ constants

constexpr int kActiveProfileCategory = 0x000F;  // DA 0x0F00: the move profile the unit applied
constexpr int kIdentPrdt = 0x10;                 // PRDT u32 "manufacture information" (UNVERIFIED)
constexpr int kParamItemFmnt = 54;               // 8.54 servo monitor: MODE b0-3, BFCY b16-31
constexpr int kCollisionFirst = 119;             // 8.119-8.127: CDC1 CDD1 CDB1 .. CDB3 (UNVERIFIED)
constexpr int kCollisionLast = 127;
constexpr std::uint32_t kSpioSlcd = 1u << 28;    // collision detection output: 0 alarm, 1 warning
constexpr std::uint32_t kMvptVctl = 1u << 10;    // vibration control
constexpr std::uint32_t kMvptCdas = 1u << 13;    // collision detection
constexpr std::uint32_t kSup2Cnwl = 1u << 23;    // wireless module
constexpr std::uint32_t kSup3Ivc1 = 1u << 16;    // the unit takes function 0x48 instead of 0x45
constexpr std::uint32_t kSsseUrtc = 1u << 8;     // SSSE b8: calendar (RTC) in use (DOC ME0162-12A p.4-25)
constexpr int kCurrentAlarmItem = 10;            // class 3 item 10 = present alarm code (VERIFIED-LIVE)
constexpr int kPositionModeItem = 2;             // class 8 item 2 = user parameter 9
constexpr int kMonitor2ItemMcur = 5;             // 4.5 rated current
constexpr int kMaintenanceTotalMovesId = 1;
constexpr int kMaintenanceDistanceId = 2;
constexpr int kSsseOffset = 0x15;                // SSSE at 0x9015

const char* const kParamEvidence =
    "VERIFIED on the EC-GRBP8M gripper (fw 0002): changed one at a time in IA-OS and the registers compared; same "
    "item IDs assumed elsewhere";
const char* const kParamCommitNote =
    "Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the coil IA-OS's Transfer writes (ON-WIRE: "
    "USB capture of IA-OS). PARW saves every parameter value in RAM, including earlier uncommitted writes. The new "
    "value is in the register at once, but the EC applies it only after a software reset or a power cycle "
    "(IA-OS also asks for a software reset after a parameter change).";
const char* const kUnknownFamilyNote =
    "the unit's family has no range table of its own in the driver: the PCON-CB range (ME0342-4H p.6-2 to 6-6) is used";

// ------------------------------------------------------------------------------------------ small helpers

double round_to(double value, int decimals) {
    const double f = std::pow(10.0, decimals);
    return std::round(value * f) / f;
}

std::int32_t s32(std::uint32_t value) { return static_cast<std::int32_t>(value); }

Words slice(const Words& words, std::size_t from, std::size_t to) {
    from = std::min(from, words.size());
    to = std::min(to, words.size());
    return from < to ? Words(words.begin() + static_cast<std::ptrdiff_t>(from), words.begin() + static_cast<std::ptrdiff_t>(to))
                     : Words();
}

std::string hex_words(const Words& words, const char* sep = " ") {
    std::string out;
    for (std::size_t i = 0; i < words.size(); ++i) out += (i ? sep : "") + util::format("%04X", words[i]);
    return out;
}

Json hex_word_list(const Words& words) {
    Json out = Json::array();
    for (std::uint16_t w : words) out.push_back(util::format("%04X", w));
    return out;
}

Json span_json(const std::optional<Span>& value) { return value ? to_json(*value) : Json(); }

Json text_or_null(const std::string& text) { return text.empty() ? Json() : Json(text); }

bool contains(const std::vector<std::string>& list, const std::string& value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

/// A JSON object without some keys (DeviceInfo.to_dict minus categories / index_header / errors).
Json without(const Json& object, std::initializer_list<const char*> keys) {
    Json out = Json::object();
    for (const auto& kv : object.as_object()) {
        bool skip = false;
        for (const char* k : keys)
            if (kv.first == k) skip = true;
        if (!skip) out.set(kv.first, kv.second);
    }
    return out;
}

/// "[1, 2]" as Python prints a list of ints.
std::string py_list(const std::vector<int>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) out += (i ? ", " : "") + std::to_string(values[i]);
    return out + "]";
}

/// Days since 1970-01-01 of a civil date (proleptic Gregorian).
long long days_from_civil(long long y, unsigned m, unsigned d) {
    y -= m <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<long long>(doe) - 719468;
}

/// Civil date of days since 1970-01-01.
void civil_from_days(long long z, long long& y, unsigned& m, unsigned& d) {
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    y = static_cast<long long>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : -9);
    y += m <= 2;
}

/// The calendar time of an RTC time stamp: seconds since 2000-01-01 00:00:00 (ME0162-12A p.4-10), "YYYY-MM-DD
/// HH:MM:SS" in the unit's own local time.
std::string rtc_datetime(std::int64_t seconds) {
    const long long total = days_from_civil(2000, 1, 1) * 86400LL + seconds;
    long long days = total / 86400;
    long long rest = total % 86400;
    if (rest < 0) {
        rest += 86400;
        --days;
    }
    long long y = 0;
    unsigned m = 0, d = 0;
    civil_from_days(days, y, m, d);
    return util::format("%04lld-%02u-%02u %02lld:%02lld:%02lld", y, m, d, rest / 3600, (rest / 60) % 60, rest % 60);
}

std::string after_power_on(std::int64_t ms) {
    const std::int64_t seconds = ms / 1000, rest_ms = ms % 1000;
    const std::int64_t days = seconds / 86400, rest = seconds % 86400;
    return (days ? std::to_string(days) + " d " : std::string()) +
           util::format("%02lld:%02lld:%02lld.%03lld", static_cast<long long>(rest / 3600),
                        static_cast<long long>((rest % 3600) / 60), static_cast<long long>(rest % 60),
                        static_cast<long long>(rest_ms));
}

std::string uptime_text(std::uint32_t ms) {
    const std::uint32_t seconds = ms / 1000;
    const std::uint32_t days = seconds / 86400, rest = seconds % 86400;
    return (days ? std::to_string(days) + " d " : std::string()) +
           util::format("%02u:%02u:%02u", rest / 3600, (rest % 3600) / 60, rest % 60);
}

// ------------------------------------------------------------------------------------------ bit tables

const char* const kLive = "VERIFIED-LIVE";
const char* const kValue = "value seen live; meaning UNVERIFIED";
const char* const kDerived = "IAI bit name from a list for another IAI model; meaning UNVERIFIED on the EC";
const char* const kDocEc = "EC manuals ME3816 / ME3837 / ME0384 (PIO); bit order in this word unconfirmed (PIO not wired)";
const char* const kDocEcRes = "EC manuals ME3816 / ME3837 / ME0384 (PIO); bit 2 not yet seen live (RES was not pressed)";
const char* const kPioLive =
    "VERIFIED-LIVE (EC-GRBP8M gripper, 2026-10-30: DIPM read while the inputs were pressed by hand, no program "
    "holding PC control; 0x0001 = ST0, 0x0002 = ST1, the unit moved with them)";
const char* const kDefaultEvidence = "IAI bit name; meaning UNVERIFIED on the EC";

BitDef bd(int bit, std::string name, std::string meaning = std::string(), bool confirmed = false,
          std::string evidence = kDefaultEvidence, bool active_low = false) {
    BitDef d;
    d.bit = bit;
    d.name = std::move(name);
    d.meaning = std::move(meaning);
    d.confirmed = confirmed;
    d.evidence = std::move(evidence);
    d.active_low = active_low;
    return d;
}

/// High bit first (a stable sort, as the Python edition's _bits).
std::vector<BitDef> sorted_bits(std::vector<BitDef> defs) {
    std::stable_sort(defs.begin(), defs.end(), [](const BitDef& a, const BitDef& b) { return a.bit > b.bit; });
    return defs;
}

std::vector<BitDef> named_bits(std::initializer_list<std::pair<int, const char*>> names, const char* evidence) {
    std::vector<BitDef> out;
    for (const auto& n : names) out.push_back(bd(n.first, n.second, "", false, evidence));
    return sorted_bits(out);
}

const std::string kL = kLive;

struct Tables {
    std::vector<std::pair<std::string, std::vector<BitDef>>> all;
    std::vector<std::string> names;

    Tables() {
        add("DSS1", {
            bd(15, "EMGS", "emergency stop", false, "DOC-CON (always 0: the IAI PC cable shorts EMG)"),
            bd(14, "SFTY", "safety speed on (mirrors DRG1 b14; caps ~100 mm/s)", true, kLive),
            bd(13, "PWR", "controller ready (always 1)", false, kValue),
            bd(12, "SV", "servo on", true, kLive),
            bd(11, "PSFL", "push missed (no contact over the band)", true, kLive),
            bd(10, "ALMH", "alarm, operation-cancel level (servo off)", true, kL + " (gripper, alarm 0x0EE)"),
            bd(9, "ALML", "message-level alarm / warning", false, "DOC-CON"),
            bd(8, "ABER", "absolute error", false, "DOC-CON"),
            bd(7, "BKRL", "brake released (1 whenever the servo is on)", true, kLive),
            bd(6, "TQLM", "torque limit"),
            bd(5, "STP", "pause (STP) held", true, kLive),
            bd(4, "HEND", "home established (0 only during a home return / after home loss)", true, kLive),
            bd(3, "PEND", "in position: inside the INP band (also after STOP, push contact, homing)", true, kLive),
            bd(2, "CEND", "load-cell bit (never set)"),
            bd(1, "CLBS", "load-cell bit (never set)"),
            bd(0, "MEND", "move command finished (also after a push miss: check PSFL)", true, kLive),
        });
        std::vector<BitDef> dss2 = {
            bd(15, "ENBS", "enable (always 1)", false, kValue),
            bd(14, "TSFC"),
            bd(13, "LOAD"),
            bd(12, "TRQS", "push current at its limit (contact push)", true, kL + " (EC-DS4H slider, seen once)"),
            bd(11, "MODS", "teaching mode", false, "DOC-CON"),
            bd(10, "TEAC", "teach command", false, "DOC-CON"),
            bd(9, "JOG+", "JOG+ held", true, kL + " (EC-DS4H slider, seen once)"),
            bd(8, "JOG-", "JOG- held", false, "DOC-CON"),
        };
        for (int n : {7, 6, 5, 4, 3, 1, 0})
            dss2.push_back(bd(n, "PE" + std::to_string(n), "position " + std::to_string(n) + " complete", false, "DOC-CON"));
        dss2.push_back(bd(2, "PE2", "position 2 complete (after a 0x9800 = 2 move)", true, kL + " (EC-DS4H slider, seen once)"));
        add("DSS2", dss2);
        add("DSSE", {
            bd(15, "EMGP", "emergency stop input", false, "DOC-CON"),
            bd(14, "MPUV", "motor drive power low / cut off", false, "DOC-CON"),
            bd(13, "RMDS", "MANU mode (always 1)", false, kValue),
            bd(12, "SSON", "1 with the servo on", false, kValue),
            bd(11, "GHMS", "home return in progress", true, kLive),
            bd(10, "PUSH", "pushing / holding after contact", true, kLive),
            bd(9, "PSNS", "excitation detection complete (0 during the homing creep)", false, kValue),
            bd(8, "PMSS", "Modbus has control (PMSL); 0 after power-up", true, kLive),
            bd(7, "PARW", "1 = parameter write idle / complete (the parameter commit handshake)", false,
               "value seen live (1 when idle); meaning UNVERIFIED"),
            bd(6, "POSW", "always 1", false, kValue),
            bd(5, "MOVE", "profile generator running (clears before standstill)", true, kLive),
            bd(4, "DST1", "motion phase? (set while cruising)", false, kValue),
            bd(3, "DST0", "motion phase? (first sample after a command)", false, kValue),
            bd(2, "REND", "never set"),
            bd(1, "PCRF", "always 1", false, kValue),
            bd(0, "PHDT", "holding after push contact", true, kL + " (EC-DS4H slider)"),
        });
        add("ZONS", {
            bd(15, "LS3", "LS of position 3", false, kDerived),
            bd(14, "LS2", "within the LS range of row 2 (middle)", true, kLive),
            bd(13, "LS1", "within the LS range of row 1 (forward end; a push row: its push end)", true, kLive),
            bd(12, "LS0", "within the LS range of row 0 (backward end)", true, kLive),
            bd(8, "ZP", "", false, kDerived),
            bd(1, "Z2", "inside zone 2 (spans the whole stroke on both units)", true, kLive),
            bd(0, "Z1", "inside zone 1 (spans the whole stroke on both units)", true, kLive),
        });
        all.emplace_back("STAT", named_bits({{31, "BATL"}, {30, "OVLW"}, {29, "RVLF"}, {28, "SSTO"}, {27, "EECP"},
                                             {26, "DSLC"}, {25, "MTRG"}, {24, "BOLW"}, {23, "PRTC"}, {22, "CLDC"},
                                             {21, "MONI"}, {20, "AABS"}, {19, "SFD"}, {18, "PLOS"}, {17, "ASOF"},
                                             {16, "AEEP"}, {15, "CARS"}, {14, "ZENB"}, {13, "ASEN"}, {12, "FSRS"},
                                             {8, "FAN"}, {6, "LRUN"}, {5, "LINK"}, {4, "RMDS"}, {3, "HEND"},
                                             {2, "SV"}, {1, "SON"}, {0, "MPOW"}},
                                            kDerived));
        all.emplace_back("SSSE", named_bits({{15, "RENB"}, {14, "AIRC"}, {13, "PTEP"}, {12, "ABUE"}, {11, "ALMC"},
                                             {8, "URTC"}, {7, "CL3"}, {6, "CL2"}, {5, "CL1"}, {4, "CL0"}, {3, "CM8"},
                                             {2, "CM4"}, {1, "CM2"}, {0, "CM1"}},
                                            kDerived));
        all.emplace_back("SIPM", named_bits({{15, "SWJ-"}, {14, "NP"}, {13, "SWJ+"}, {12, "PP"}, {11, "SDWN"},
                                             {10, "FANE"}, {9, "PTSW"}, {8, "MDSW"}, {7, "ENBL"}, {6, "RSV1"},
                                             {5, "RSV0"}, {4, "BLCT"}, {3, "HMCK"}, {2, "OT"}, {1, "CREP"}, {0, "LS"}},
                                            kDerived));
        add("DRG1", {
            bd(15, "EMG", "software emergency stop", false, "DOC-CON"),
            bd(14, "SFTY", "safety-speed command", true, kL + " (readback 0x5000 with SFTY)"),
            bd(12, "SON", "servo-on command", true, kL + " (readback 0x1000)"),
            bd(11, "CSTP"),
            bd(8, "ALRS", "alarm reset", false, "DOC-CON"),
            bd(7, "BKRL", "forced brake release", false, "DOC-CON"),
            bd(6, "TQLM"),
            bd(5, "STP", "pause", false, "DOC-CON"),
            bd(4, "HOME", "home return", false, "DOC-CON"),
            bd(3, "CSTR", "start move to POSR", false, "DOC-CON"),
        });
        std::vector<BitDef> drg2 = {
            bd(15, "JVEL"), bd(14, "JISL"), bd(13, "EXTA"), bd(12, "PMOD"), bd(11, "MOD"),
            bd(10, "TEAC", "teach (writes the position table)", false, "DOC-CON"),
            bd(9, "JOG+", "jog + (runs at ~100 mm/s; never used by the driver)", true, kLive),
            bd(8, "JOG-", "jog -", false, "DOC-CON"),
        };
        for (int n = 7; n >= 0; --n)
            drg2.push_back(bd(n, "ST" + std::to_string(n), "start input " + std::to_string(n) + " (solenoid-valve mode)",
                              false, "DOC-CON"));
        add("DRG2", drg2);
        add("DRGE", {
            bd(15, "SFRS"), bd(14, "BRCG"), bd(13, "RMOD"), bd(12, "DCLR"), bd(11, "DSTR"), bd(10, "WAIT"),
            bd(9, "CLBR"),
            bd(8, "PMSL", "Modbus control requested (DSSE PMSS follows)", true, kL + " (readback 0x0100)"),
            bd(7, "PARW"), bd(4, "MIWE"),
            bd(3, "STOP", "decel stop", true, kLive),
            bd(2, "RSTR"), bd(1, "PCRF"),
        });
        add("CTLF", {
            bd(15, "PRG"), bd(14, "SPOS"), bd(11, "PTOF"), bd(10, "SMOD"), bd(9, "ASO1"), bd(8, "ASO0"),
            bd(7, "MOD1"), bd(6, "MOD0"),
            bd(5, "GSL1", "row role bit (0x0020 on the slider's row 2); meaning unknown", false, kValue),
            bd(4, "GSL0", "row role bit (0x0010 on the slider's row 1); meaning unknown", false, kValue),
            bd(3, "INC", "relative move: PCMD is a signed distance", true, kLive),
            bd(2, "DIR", "push direction: 1 = away from home, 0 = toward home", true, kLive),
            bd(1, "PUSH", "push move over the INP band at PPOW", true, kLive),
        });
        // PIO images: DOPM layouts VERIFIED-LIVE (3POS on the EC-DS4H slider, 2POS on the EC-GRBP8M gripper). DIPM:
        // ST0 = bit 0 and ST1 = bit 1 of the 2POS layout VERIFIED-LIVE on the gripper (inputs pressed by hand); RES
        // (bit 2) was not pressed and the other layouts were never seen (no PIO wired on the slider): DOC only.
        add("DOPM_3POS", {
            bd(0, "LS0", "backward end (row 0) within the LS range / PE0", true, kL + " (0x0009 at 0.00 mm)"),
            bd(1, "LS1", "forward end (row 1) within the LS range / PE1", true, kL + " (0x000A at 150.00 mm)"),
            bd(2, "LS2", "middle (row 2) within the LS range", true, kL + " (0x000C at 75.00 mm)"),
            bd(3, "*ALM", "alarm output, active low: 1 = no alarm", true, kL + " (seen 1 only)", true),
        });
        add("DOPM_2POS", {
            bd(0, "LS0", "backward end (row 0) within the LS range / PE0 push complete", true, kL + " (gripper 0x0005)"),
            bd(1, "LS1", "forward end (row 1) / PE1; a push row's bit stays 0 after a missed push", true,
               kL + " (gripper 0x0006 at 9.50 mm, 0x0004 after a miss)"),
            bd(2, "*ALM", "alarm output, active low: 1 = no alarm", true, kL + " (seen 1 only)", true),
        });
        add("DIPM_3POS", {
            bd(0, "ST0", "go to the backward end (ST0 + ST1 = middle)", false, kDocEc),
            bd(1, "ST1", "go to the forward end (ST0 + ST1 = middle)", false, kDocEc),
            bd(3, "RES", "alarm reset", false, kDocEc),
        });
        add("DIPM_2POS", {
            bd(0, "ST0", "go to the backward end (starts homing if home is not established)", true, kPioLive),
            bd(1, "ST1", "go to the forward end", true, kPioLive),
            bd(2, "RES", "alarm reset", false, kDocEcRes),
        });
        add("DIPM_SINGLE", {
            bd(0, "FWBW", "single solenoid: ON = forward, OFF = backward", false, kDocEc),
            bd(2, "RES", "alarm reset", false, kDocEc),
        });
        for (const auto& t : all) names.push_back(t.first);
    }

    void add(const std::string& name, std::vector<BitDef> defs) { all.emplace_back(name, sorted_bits(std::move(defs))); }
};

const Tables& tables() {
    static const Tables t;
    return t;
}

// ------------------------------------------------------------------------------------------ monitor layout

struct Member {
    const char* name;
    int size;
};
// 0x9000 monitor members (IAI names and order; offsets VERIFIED-LIVE up to ZONS on both units, OLLV on the slider).
const Member kMonitorLayout[] = {
    {"PNOW", 2}, {"ALMC", 1}, {"DIPM", 1}, {"DOPM", 1}, {"DSS1", 1}, {"DSS2", 1}, {"DSSE", 1}, {"STAT", 2},
    {"VNOW", 2}, {"CNOW", 2}, {"DEVI", 2}, {"STIM", 2}, {"SIPM", 1}, {"ZONS", 1}, {"POSS", 1}, {"SSSE", 1},
    {"PNWP", 2}, {"VNWP", 2}, {"CNWF", 2}, {"PDNW", 2}, {"FBFC", 2}, {"OLLV", 2},
};

bool has(const MonitorWords& m, const char* name) { return m.find(name) != m.end(); }
std::uint32_t get(const MonitorWords& m, const char* name, std::uint32_t fallback = 0) {
    const auto it = m.find(name);
    return it == m.end() ? fallback : it->second;
}
std::optional<std::uint32_t> opt(const MonitorWords& m, const char* name) {
    const auto it = m.find(name);
    return it == m.end() ? std::nullopt : std::optional<std::uint32_t>(it->second);
}

Json monitor_json(const MonitorWords& m, std::optional<std::uint32_t> mcur) {
    Json raw = Json::object();
    for (const Member& member : kMonitorLayout)
        if (has(m, member.name))
            raw.set(member.name, util::format(member.size == 2 ? "0x%08X" : "0x%04X", get(m, member.name)));
    Json out = Json::object();
    out.set("raw", raw);
    if (has(m, "PNOW")) out.set("position_mm", util::round2(s32(get(m, "PNOW")) * reg::kPositionScale));
    if (has(m, "ALMC")) out.set("alarm_code", get(m, "ALMC"));
    if (has(m, "VNOW")) out.set("velocity_mm_s", util::round2(s32(get(m, "VNOW")) * reg::kVelocityScale));
    if (has(m, "CNOW")) {
        const std::int32_t cnow = s32(get(m, "CNOW"));
        out.set("current", cnow);
        if (mcur && *mcur != 0) {
            out.set("rated_current", *mcur);
            out.set("current_pct", round_to(cnow * 100.0 / *mcur, 1));
        }
    }
    if (has(m, "DEVI")) out.set("deviation", s32(get(m, "DEVI")));
    if (has(m, "STIM")) {
        const std::uint32_t stim = get(m, "STIM");
        out.set("uptime_ms", stim);
        out.set("uptime_s", round_to(stim / 1000.0, 3));
        out.set("uptime", uptime_text(stim));
    }
    if (has(m, "POSS")) out.set("position_complete_no", get(m, "POSS") & 0x3FF);
    if (has(m, "OLLV")) out.set("overload_level", s32(get(m, "OLLV")));
    Json extra = Json::object();
    for (const char* k : {"PNWP", "VNWP", "CNWF", "PDNW", "FBFC"})
        if (has(m, k)) extra.set(k, s32(get(m, k)));
    if (extra.size()) out.set("other", extra);
    Json notes = Json::object();
    notes.set("velocity_mm_s", "VNOW, 0.01 mm/s but quantized in ~12.5 mm/s steps (VERIFIED-LIVE): not for control");
    notes.set("current", "CNOW, unit unconfirmed (643 servo-on standstill, 464 holding a 30 % push); current_pct = CNOW "
                         "/ MCUR (item 4.5, rated current) x 100, as IA-OS shows it");
    notes.set("deviation", "DEVI, position deviation in pulses (DOC-CON; 0 at rest)");
    notes.set("uptime", "STIM, ms since power-on (VERIFIED-LIVE)");
    notes.set("position_complete_no", "POSS (DOC-CON; read 0 after numeric moves)");
    notes.set("overload_level", "OLLV, probably the overload level in % (unconfirmed)");
    notes.set("other", "PNWP / VNWP / CNWF / PDNW / FBFC (IAI names, meanings UNVERIFIED; PNWP = -0.8 x PNOW, PDNW = "
                       "+0.8 x PNOW live)");
    out.set("notes", notes);
    return out;
}

Json status_words_json(const MonitorWords& m) {
    Json out = Json::object();
    for (const char* name : {"DSS1", "DSS2", "DSSE", "ZONS", "SSSE", "SIPM"})
        if (has(m, name)) out.set(name, decode_bits(get(m, name), bit_table(name)));
    if (has(m, "STAT")) out.set("STAT", decode_bits(get(m, "STAT"), bit_table("STAT"), 32));
    if (has(m, "POSS")) {
        const std::uint32_t poss = get(m, "POSS");
        Json p = Json::object();
        p.set("value", poss);
        p.set("hex", util::format("0x%04X", poss));
        p.set("position_no", poss & 0x3FF);
        p.set("evidence", "DOC-CON (binary position number PM1..PM512)");
        out.set("POSS", p);
    }
    return out;
}

Json io_json(const MonitorWords& m, const Json& mode) {
    const IoTables t = io_bit_tables(mode);
    Json out = Json::object();
    const Json* name = mode.find("name");
    const Json* layout = mode.find("layout");
    out.set("mode", name ? *name : Json());
    out.set("layout", layout ? *layout : Json());
    if (has(m, "DIPM")) out.set("DIPM", decode_bits(get(m, "DIPM"), *t.dipm));
    if (has(m, "DOPM")) out.set("DOPM", decode_bits(get(m, "DOPM"), *t.dopm));
    for (const char* n : {"SIPM", "ZONS"})
        if (has(m, n)) out.set(n, decode_bits(get(m, n), bit_table(n)));
    Json lamps = Json::object();
    if (has(m, "ZONS")) {
        const std::uint32_t zons = get(m, "ZONS");
        lamps.set("backward_end", (zons & reg::kZonsLs0) != 0);
        lamps.set("forward_end", (zons & reg::kZonsLs1) != 0);
        lamps.set("middle", (zons & reg::kZonsLs2) != 0);  // row 2 lights it on the 2-position gripper too
    }
    if (has(m, "ALMC") || has(m, "DSS1")) {
        const int code = static_cast<int>(get(m, "ALMC"));
        const std::uint32_t dss1 = get(m, "DSS1");
        const bool fault = alarm_is_fault(code, (dss1 & reg::kDss1Almh) != 0, (dss1 & reg::kDss1Alml) != 0);
        lamps.set("alarm", fault);
        lamps.set("warning", !fault && ((dss1 & reg::kDss1Alml) != 0 || (code != 0 && alarm_info(code).warning)));
    }
    out.set("lamps", lamps);
    out.set("note", "position lamps come from ZONS b12-b14 (same on every unit tested); the DOPM layout depends on "
                    "the position mode; DIPM: ST0 / ST1 of the 2-position layout are VERIFIED-LIVE (EC-GRBP8M "
                    "gripper), the other bit names are unconfirmed. Alarm lamp = a fault "
                    "(DSS1 ALMH, an alarm of level operation cancelled / cold start, or a code of unknown level without "
                    "ALML); a known message-level code (e.g. a maintenance warning) lights the warning lamp only; the "
                    "axis keeps running");
    return out;
}

Json control_words_json(const ClassBlock& block) {
    static const std::map<int, const char*> kNames = {{1, "DRG1"}, {2, "DRG2"}, {3, "DRGE"}, {4, "POSR"}, {5, "DOCR"},
                                                      {6, "GTV"},  {7, "SRGE"}, {8, "RMDI"}, {9, "PEDI"}, {12, "RCEC"}};
    Json out = Json::object();
    for (const ClassItem& it : block.items) {
        std::string name;
        if (it.item_id && kNames.count(*it.item_id)) name = kNames.at(*it.item_id);
        else name = "item " + (it.item_id ? std::to_string(*it.item_id) : std::string("None"));
        Json entry;
        if (name == "DRG1" || name == "DRG2" || name == "DRGE") {
            entry = decode_bits(it.raw, bit_table(name));
        } else {
            entry = Json::object();
            entry.set("value", it.raw);
            entry.set("hex", util::format("0x%04X", static_cast<unsigned>(it.raw)));
        }
        entry.set("address", util::format("0x%04X", it.address));
        out.set(name, entry);
    }
    return out;
}

// ------------------------------------------------------------------------------------------ alarms

Json alarms_json(const IndexCategory& rec, const Words& words, std::optional<std::uint32_t> current,
                 std::optional<std::uint32_t> dss1, std::optional<std::uint32_t> c3, std::optional<bool> absolute,
                 std::optional<bool> rtc, std::optional<int> system_code) {
    const int stride = rec.member_count * std::max(rec.member_size, 1);
    const int slots = stride ? static_cast<int>(words.size()) / stride : 0;
    const bool decodable = stride == reg::kAlarmHistoryStride;
    std::vector<AlarmRecord> used;
    if (decodable) used = alarm_history_slots(words);  // ends at the first empty slot
    Json history = Json::array();
    int power_ups = 0;
    for (const AlarmRecord& r : used) {
        if (r.code == reg::kAlarmPowerUp) {
            ++power_ups;
            continue;
        }
        const AlarmInfo about = alarm_info(r.code, absolute, r.detail, r.address, system_code);
        const Json when = alarm_time(r.time_ms, rtc);
        Json entry = to_json(r);
        entry.set("hex", util::format("0x%03X", r.code));
        entry.set("name", alarm_name(r.code));
        entry.set("newer_power_ups", power_ups);
        entry.set("level", level_key(about.level));
        entry.set("group", about.group);
        entry.set("detail_text", about.detail_text);
        entry.set("clock", *when.find("clock"));
        entry.set("occurred_at", *when.find("occurred_at"));
        if (when.find("clock")->type() == Json::Type::String && when.find("clock")->as_string() == "rtc")
            entry.set("time_after_power_on", Json());
        history.push_back(entry);
    }
    Json cur;
    if (current) {
        const int code = static_cast<int>(*current);
        const bool alarm_bit = dss1 && (*dss1 & reg::kDss1Almh) != 0;
        const bool warning_bit = dss1 && (*dss1 & reg::kDss1Alml) != 0;
        const AlarmInfo about = alarm_info(code, absolute, std::nullopt, std::nullopt, system_code);
        const bool fault = alarm_is_fault(code, alarm_bit, warning_bit, system_code);
        cur = Json::object();
        cur.set("code", code);
        cur.set("hex", util::format("0x%03X", code));
        cur.set("name", alarm_name(code));
        cur.set("active", code != 0);
        cur.set("level", code ? Json(level_key(about.level)) : Json());
        cur.set("level_text", code ? Json(about.level_text) : Json());
        cur.set("fault", fault);
        cur.set("warning", code != 0 && !fault);
        if (dss1) {
            Json bits = Json::object();
            bits.set("ALMH", alarm_bit);
            bits.set("ALML", warning_bit);
            cur.set("dss1_bits", bits);
        } else {
            cur.set("dss1_bits", Json());
        }
        cur.set("group", code ? Json(about.group) : Json());
        cur.set("group_name", code ? Json(about.group_name) : Json());
        cur.set("reset", about.reset);
        cur.set("home_needed", about.home_needed);
        cur.set("remedy", about.remedy);
        cur.set("class3_code", c3);
    }
    Json out = Json::object();
    out.set("current", cur);
    out.set("history", history);
    out.set("slots", slots);
    out.set("alarm_count", static_cast<int>(history.size()));
    out.set("power_up_markers", power_ups);
    out.set("empty_slots", decodable ? slots - static_cast<int>(used.size()) : 0);
    out.set("absolute_encoder", absolute);
    out.set("rtc_in_use", rtc);
    out.set("address", util::format("0x%04X", rec.data_address));
    Json notes = Json::object();
    notes.set("order", "index 0 is the newest (consistent with the time stamps on hardware); the list ends at the "
                       "first empty slot (code 0), so empty_slots counts the slots from there on; newer_power_ups = "
                       "power-up markers at lower indices");
    notes.set("time_raw", "with the unit's calendar in use (SSSE b8 = 1, rtc_in_use): seconds since 2000-01-01 "
                          "00:00:00, shown as occurred_at (clock 'rtc'; DOC ME0162-12A p.4-10); otherwise the time "
                          "since power-on (clock 'power_on'): milliseconds on the EC (VERIFIED; it has no clock), "
                          "time_after_power_on");
    notes.set("names", "IAI's controller alarm names, as IA-OS shows them");
    notes.set("levels", "message (a known warning code: the axis keeps running), operation cancelled (alarm reset), "
                        "cold start (software reset / power cycle), unknown (a fault unless ALML is set); UNVERIFIED "
                        "except 0x090 and 0x0EE on an absolute unit (VERIFIED); 0x0EE depends on absolute_encoder "
                        "(SUPT ABSE / SUP2 MABS); remedies are the driver's own words");
    out.set("notes", notes);
    return out;
}

// ------------------------------------------------------------------------------------------ positions

const char* simple_role(int n) {
    switch (n) {
        case 256: return "backward end, present (real)";
        case 257: return "forward end, present (real)";
        case 258: return "backward end, previous (real)";
        case 259: return "forward end, previous (real)";
        case 260: return "backward end, present (IA-OS %)";
        case 261: return "forward end, present (IA-OS %)";
        case 262: return "backward end, previous (IA-OS %)";
        case 263: return "forward end, previous (IA-OS %)";
        case 264: return "middle, present (real)";
        case 265: return "middle, previous (real)";
        case 266: return "middle, present (IA-OS %)";
        case 267: return "middle, previous (IA-OS %)";
        default: return "?";
    }
}

Json row_json(int n, int address, const Words& words, const std::string& role) {
    const PositionEntry e = PositionEntry::from_registers(words);
    const bool push = (e.control_flags & reg::kCtlfPush) != 0;
    const bool away = (e.control_flags & reg::kCtlfPushAway) != 0;
    Json out = Json::object();
    out.set("n", n);
    out.set("role", role);
    out.set("address", util::format("0x%04X", address));
    out.set("position_mm", e.position_mm);
    out.set("band_mm", e.band_mm);
    out.set("speed_mm_s", e.speed_mm_s);
    out.set("zone_plus_mm", e.zone_plus_mm);
    out.set("zone_minus_mm", e.zone_minus_mm);
    out.set("accel_g", e.accel_g);
    out.set("decel_g", e.decel_g);
    out.set("push_power", e.push_power);
    out.set("push_pct", round_to(e.push_power * 100.0 / 255.0, 1));
    out.set("load_threshold", e.load_threshold);
    out.set("control_flags", util::format("0x%04X", e.control_flags));
    out.set("ctlf_on", *decode_bits(static_cast<std::uint32_t>(e.control_flags) & 0xFFFF, bit_table("CTLF")).find("on"));
    out.set("push", push);
    out.set("push_direction", push ? Json(away ? "away from home" : "toward home") : Json());
    out.set("push_end_mm", push ? Json(util::round2(e.position_mm + (away ? e.band_mm : -e.band_mm))) : Json());
    out.set("checksum", e.checksum ? Json(util::format("0x%04X", *e.checksum)) : Json());
    out.set("checksum_ok", e.checksum_ok());
    out.set("raw", hex_words(words));
    return out;
}

Json simple_row_json(int n, int address, const Words& w) {
    Json out = Json::object();
    out.set("n", n);
    out.set("role", simple_role(n));
    out.set("address", util::format("0x%04X", address));
    out.set("position_mm", util::round2(reg::from_i32_words(w[0], w[1]) * reg::kPositionScale));
    out.set("band_mm", util::round2(reg::from_u32_words(w[2], w[3]) * reg::kPositionScale));
    out.set("speed_pct", reg::from_u32_words(w[4], w[5]));
    out.set("accel_pct", w[10]);
    out.set("decel_pct", w[11]);
    out.set("push_power", w[12]);
    out.set("control_flags", util::format("0x%04X", w[14]));
    out.set("checksum_ok", reg::position_checksum(w) == w[15]);
    out.set("raw", hex_words(w));
    return out;
}

Json direct_move_json(const Words& w) {
    if (w.size() < 6) throw std::invalid_argument("the 0x9900 block has fewer than 6 words");
    Json out = Json::object();
    out.set("address", "0x9900");
    out.set("target_mm", util::round2(reg::from_i32_words(w[0], w[1]) * reg::kPositionScale));
    out.set("band_mm", util::round2(reg::from_u32_words(w[2], w[3]) * reg::kPositionScale));
    out.set("speed_mm_s", util::round2(reg::from_u32_words(w[4], w[5]) * reg::kVelocityScale));
    out.set("accel_g", w.size() > 6 ? Json(util::round2(w[6] * reg::kAccelScale)) : Json());
    if (w.size() > 8) {
        out.set("push_power", w[7]);
        out.set("push_pct", round_to(w[7] * 100.0 / 255.0, 1));
        out.set("control_flags", util::format("0x%04X", w[8]));
        out.set("ctlf_on", *decode_bits(w[8], bit_table("CTLF")).find("on"));
    }
    if (w.size() > 9) out.set("drg1_mirror", util::format("0x%04X", w[9]));
    out.set("raw", hex_words(w));
    out.set("note", "the last values written by any client (volatile; CTLF clears after each move start)");
    return out;
}

Json active_profile_json(const Words& w) {
    Json out = Json::object();
    out.set("address", "0x0F00");
    out.set("raw", hex_words(w));
    if (w.size() >= 13) {
        out.set("target_mm", util::round2(reg::from_i32_words(w[0], w[1]) * reg::kPositionScale));
        out.set("speed_mm_s", util::round2(reg::from_u32_words(w[2], w[3]) * reg::kVelocityScale));
        out.set("accel_g", util::round2(reg::from_u32_words(w[4], w[5]) * reg::kAccelScale));
        out.set("decel_g", util::round2(reg::from_u32_words(w[6], w[7]) * reg::kAccelScale));
        out.set("band_mm", util::round2(reg::from_u32_words(w[8], w[9]) * reg::kPositionScale));
        out.set("spow", w[10]);
        out.set("dpow", w[11]);
        out.set("push_power", w[12]);
    }
    if (w.size() >= 14) out.set("control_flags", util::format("0x%04X", w[13]));
    out.set("note", "the move profile the EC applied (IAI names; VERIFIED-LIVE values in one snapshot)");
    return out;
}

// ------------------------------------------------------------------------------------------ identity

Json identification_extra(const Words& w) {
    Json out = Json::object();
    if (w.size() < 0x1A) return out;
    const std::string date_text = util::format("%04X-%02X-%02X", w[0x14], w[0x15] >> 8, w[0x15] & 0xFF);
    const std::uint32_t prdt = reg::from_u32_words(w[kIdentPrdt], w[kIdentPrdt + 1]);
    const std::uint32_t accl = reg::from_u32_words(w[reg::kIdentAcclOffset], w[reg::kIdentAcclOffset + 1]);
    const int level = static_cast<int>((accl >> 8) & 0xF);
    static const std::map<int, const char*> kLevels = {{0, "Normal"}, {1, "Position3"}, {2, "Extensibility phase 1"},
                                                       {3, "Extensibility phase 2"}, {4, "Multi-I/O public release"}};
    const std::string serial = reg::words_to_ascii(slice(w, 0x08, 0x10));
    const std::string prdt_text = reg::words_to_ascii(slice(w, 0x10, 0x12));
    out.set("actuator_serial", text_or_null(serial));
    out.set("second_id", text_or_null(serial));  // compatibility alias of actuator_serial
    Json manufacture = Json::object();
    manufacture.set("hex", util::format("%08X", prdt));
    manufacture.set("mrev", util::format("%04X", prdt >> 16));
    manufacture.set("frev", util::format("%04X", prdt & 0xFFFF));
    manufacture.set("ascii", text_or_null(prdt_text));
    out.set("manufacture_info", manufacture);
    out.set("code", text_or_null(prdt_text));  // compatibility alias (PRDT as text)
    Json a = Json::object();
    a.set("hex", util::format("0x%08X", accl));
    a.set("extensibility_level", level);
    a.set("level_name", kLevels.count(level) ? kLevels.at(level) : "?");
    out.set("accl", a);
    out.set("bisd_raw", util::format("%08X", reg::from_u32_words(w[0x14], w[0x15])));
    out.set("user_name", text_or_null(reg::words_to_ascii(slice(w, 0x1A, 0x20))));
    out.set("words_0612", hex_word_list(slice(w, 0x12, 0x18)));
    bool digits = true;
    for (char c : date_text)
        if (c != '-' && !(c >= '0' && c <= '9')) digits = false;
    out.set("date_guess", digits ? Json(date_text) : Json());
    out.set("note", "IAI field names (meanings UNVERIFIED): 0x0608 ACS actuator serial, 0x0610 PRDT manufacture "
                    "information (MREV + FREV), 0x0612 ACCL (bits 8-11 extensibility level), 0x0614 BISD (meaning "
                    "unknown; the date reading is a guess), 0x061A ANA1-3 the user-editable unit name (not a model "
                    "field)");
    return out;
}

const ClassItem* class_item(const std::map<int, ClassBlock>& classes, int class_id, int item_id) {
    const auto it = classes.find(class_id);
    return it == classes.end() ? nullptr : it->second.item(item_id);
}

Json capabilities_json(const std::map<int, ClassBlock>& classes, const SupportWords& support,
                       const std::optional<AxisLimits>& limits) {
    Json out = Json::object();
    const Json decoded = decode_support_words(support);
    out.set("support", decoded);
    if (limits) {
        Json lim = without(to_json(*limits), {"notes"});  // the Python edition's AxisLimits.to_dict() has no notes
        if (limits->max_push_pct && *limits->max_push_pct != 0.0) {
            lim.set("push_force_at_max_estimate", limits->push_force_estimate(*limits->max_push_pct));
            lim.set("push_force_note", "estimate TCPC x % / MXPP + TCOA; unit probably N (UNVERIFIED)");
        }
        if (limits->actuator_kind) {
            std::string name = reg::axis_kind_name(limits->actuator_kind);
            if (util::starts_with(name, "kind ")) name = "unknown";
            lim.set("actuator_kind_name", name);
        } else {
            lim.set("actuator_kind_name", Json());
        }
        out.set("axis_limits", lim);
    }
    const int c8 = reg::kMotionParamsCategory;
    if (const ClassItem* pshv = class_item(classes, c8, reg::kParamItemPshv))
        out.set("push_speed_mm_s", util::round2(pshv->signed_value() * reg::kVelocityScale));
    const ClassItem* brsl = class_item(classes, c8, reg::kParamItemBrsl);
    const ClassItem* rtim = class_item(classes, c8, reg::kParamItemRtim);
    Json link = Json::object();
    link.set("boot_baud_code", brsl ? Json(brsl->raw) : Json());
    link.set("boot_baudrate", brsl && brsl->raw <= 0xFFFF ? Json(baud_by_code(static_cast<int>(brsl->raw))) : Json());
    link.set("transmit_delay_ms", rtim ? Json(rtim->raw) : Json());
    link.set("note", "BRSL (item 8.3) = the power-up rate (volatile changes return to it after a power cycle or "
                     "software reset); RTIM (8.4) = minimum delay before the EC's transmitter turns on (UNVERIFIED)");
    out.set("link", link);
    if (const ClassItem* fmnt = class_item(classes, c8, kParamItemFmnt)) {
        static const char* const kModes[] = {"off", "4 channels", "8 channels", "2 channels"};
        const unsigned mode = fmnt->raw & 0xF;
        Json s = Json::object();
        s.set("fmnt_hex", util::format("0x%08X", fmnt->raw));
        s.set("mode", mode);
        s.set("mode_name", mode < 4 ? kModes[mode] : "?");
        s.set("buffering_cycle", fmnt->raw >> 16);
        s.set("note", "FMNT (item 8.54): high-speed sampling needs mode != 0, a non-volatile change (not done by this "
                      "driver); 0x46 is not used on the EC");
        out.set("servo_monitor", s);
    } else {
        out.set("servo_monitor", Json());
    }
    std::optional<bool> supported, vibration;
    if (decoded.type() == Json::Type::Object) {
        const Json* flags = decoded.find("flags");
        if (flags) {
            const Json* cd = flags->find("collision_detection");
            const Json* vc = flags->find("vibration_control");
            if (cd && cd->type() == Json::Type::Bool) supported = cd->as_bool();
            if (vc && vc->type() == Json::Type::Bool) vibration = vc->as_bool();
        }
    }
    std::vector<std::string> present;
    for (int i = kCollisionFirst; i <= kCollisionLast; ++i)
        if (class_item(classes, c8, i)) present.push_back("8." + std::to_string(i));
    Json sets = Json::array();
    for (int n = 0; n < 3; ++n) {
        const ClassItem* cdc = class_item(classes, c8, kCollisionFirst + 3 * n);
        const ClassItem* cdd = class_item(classes, c8, kCollisionFirst + 3 * n + 1);
        const ClassItem* cdb = class_item(classes, c8, kCollisionFirst + 3 * n + 2);
        if (!cdc && !cdd && !cdb) continue;
        Json s = Json::object();
        s.set("set", n + 1);
        s.set("current_pct", cdc ? Json(static_cast<long long>(std::lround(cdc->raw * 100.0 / 255.0))) : Json());
        s.set("distance_mm", cdd ? Json(util::round2(cdd->signed_value() * reg::kPositionScale)) : Json());
        s.set("time_ms", cdb ? Json(cdb->raw >> 16) : Json());
        if (cdb) {
            const unsigned evac = cdb->raw & 0xFFFF;
            s.set("evacuation_position", evac == 0xFFFF ? Json(-1) : Json(evac));
        } else {
            s.set("evacuation_position", Json());
        }
        Json raw = Json::array();
        for (const ClassItem* i : {cdc, cdd, cdb}) raw.push_back(i ? Json(i->hex()) : Json());
        s.set("raw", raw);
        sets.push_back(s);
    }
    const ClassItem* spio = class_item(classes, c8, reg::kParamItemSpio);
    Json collision = Json::object();
    collision.set("supported", supported);
    collision.set("reported", supported ? (*supported ? "supported (MVPT b13 CDAS = 1)" : "not reported (MVPT b13 CDAS = 0)")
                                        : "unknown (class 1 not readable)");
    collision.set("vibration_control", vibration);
    collision.set("parameter_items", present);
    collision.set("sets", sets);
    collision.set("output_on_collision",
                  !spio || present.empty() ? Json() : Json((spio->raw & kSpioSlcd) ? "warning" : "alarm"));
    collision.set("note", "READ-ONLY: collision sets are class 8 items 119-127 (current % = raw x 100 / 255, distance "
                          "0.01 mm, time = CDB b16-31 ms, evacuation position = CDB b0-15; layout UNVERIFIED); never "
                          "written by this driver");
    out.set("collision", collision);
    return out;
}

// ------------------------------------------------------------------------------------------ user parameters

struct ManualView {
    const char* category;
    const char* manual_name;
    const char* manual_range;
    const char* shipping_default;
    const char* doc;
};

// The public parameter list (DOC ME3816-6B p.6-6; details p.6-7 to 6-17). Only P1-P10 are in it.
const ManualView* manual_view(int no) {
    static const ManualView kRows[] = {
        {"A", "Operation range adjustment",
         "0 to 9999.69 mm (deg); the controller adds 0.30 (p.6-7); above the maximum stroke the axis runs into the "
         "mechanical stopper",
         "maximum stroke", "ME3816-6B p.6-6, 6-7"},
        {"A", "Auto switch \"LS\" signal detection range adjustment",
         "the actuator's minimum to 9999.99 mm (deg); below the encoder resolution LS never lights (p.8-17)",
         "per EC specification", "ME3816-6B p.6-6, 6-8"},
        {"C", "Home return direction change", "Reverse, Forward", "per EC specification",
         "ME3816-6B p.6-6, 6-9; Intro-17; ME3800-5E p.1-36"},
        {"B", "Home position adjustment", "0 to 9999.99 mm (deg); never below the value at shipping (p.6-10)",
         "per EC specification", "ME3816-6B p.6-6, 6-10"},
        {"A", "Smooth accel/decel setting", "Disabled, Enabled", "Disabled", "ME3816-6B p.6-6, 6-11, 8-21"},
        {"A", "Current control setting at stop", "Disabled (powerful stop), Enabled (energy-saving stop)", "Disabled",
         "ME3816-6B p.6-6, 6-12"},
        {"A", "Wireless function setting", "Disabled, Enabled", "Enabled",
         "ME3816-6B p.6-6, 6-13 (shown only with option WL / WL2)"},
        {"A", "Energy-saving setting", "Disabled, Enabled", "Disabled", "ME3816-6B p.6-6, 6-14"},
        {"A", "Electromagnetic valve system select",
         "Double, Single (3-position with option MF); Double only on an ACR (RCON-EC) unit", "Double",
         "ME3816-6B p.6-6, 6-15, Intro-9; ME3837-1D p.3-1"},
        {"A", "LED lighting system automatic switch setting",
         "Disabled, Enabled; may have no effect outside the small models listed on p.6-16", "Disabled",
         "ME3816-6B p.6-6, 6-16"},
    };
    return no >= 1 && no <= 10 ? &kRows[no - 1] : nullptr;
}

const char* ec_category_text(const std::string& category) {
    if (category == "A") return "customer-adjustable: check or change for the use";
    if (category == "B") return "use as shipped, as a rule";
    if (category == "C") return "factory setting for the EC's specification: normally not changed";
    return "";
}

Json param_value_json(const ParamValue& v) { return v.empty() ? Json() : to_json(v); }

/// info._decode_param: one user parameter decoded from its items.
Json decode_user_parameter(const ParamDef& p, const std::map<int, ClassBlock>& classes) {
    const auto block = classes.find(p.class_id);
    const ClassItem* item = block == classes.end() ? nullptr : block->second.item(p.item_id, p.entry);
    const ClassItem* item2 =
        block == classes.end() || !p.item_id2 ? nullptr : block->second.item(*p.item_id2, p.entry);
    const ec_detail::Decoded d = ec_detail::decode(p, item, item2);
    const ManualView* manual = manual_view(p.no);
    Json out = Json::object();
    out.set("no", p.no);
    out.set("name", p.name);
    out.set("key", p.key());
    out.set("unit", p.unit);
    out.set("encoding", p.encoding);
    out.set("evidence", p.evidence.empty() ? std::string(kParamEvidence) : p.evidence);
    out.set("read_only", p.read_only);
    const std::string category = manual ? manual->category : "";
    out.set("category", category);
    out.set("category_text", ec_category_text(category));
    out.set("manual_name", manual ? manual->manual_name : "");
    out.set("manual_range", manual ? manual->manual_range : "");
    out.set("shipping_default", manual ? manual->shipping_default : "");
    out.set("doc", manual ? manual->doc : "not in ME3816-6B (the PC tool lists it, the manual does not)");
    out.set("iaos_only", manual == nullptr);
    if (!p.note.empty()) out.set("note", p.note);
    out.set("found", d.found);
    out.set("address", d.address);
    out.set("raw_hex", d.raw_hex);
    if (d.found && p.kind == "parity" && d.pattern) out.set("pattern", *d.pattern);
    out.set("value", param_value_json(d.value));
    out.set("text", d.text);
    return out;
}

/// info.annotate_parameters: the per-model flags.
Json annotate_parameter(Json q, const ParamDef& p, const ParameterModel& model,
                        const std::map<int, ParameterVisibility>& visibility) {
    const bool found = q.find("found")->as_bool();
    const bool listed = std::find(model.listed.begin(), model.listed.end(), p.no) != model.listed.end() &&
                        (model.known || found);
    const bool settable = !(p.kind == "parity" && found && q.find("value")->is_null());
    const bool read_only = p.read_only;
    const auto vis = visibility.find(p.no);
    const std::optional<bool> shown = vis == visibility.end() ? std::nullopt : vis->second.shown;
    const std::string reason = vis == visibility.end() ? std::string() : vis->second.reason;
    q.set("listed", listed);
    q.set("verified_for_model", model.verified);  // kept for compatibility: an IA-OS list is recorded for the code
    q.set("home_parameter", is_ec_home_parameter(p.no));
    q.set("writable", found && !read_only && settable);  // an unlisted parameter is written too, with a warning
    q.set("needs_model_opt_in", false);  // no per-model opt-in exists (kept for compatibility)
    Json choices = Json::object();
    if (p.kind != "mm")
        for (const auto& c : p.choices) choices.set(std::to_string(c.first), c.second);
    q.set("choices", choices);
    q.set("read_only", read_only);
    q.set("shown_by_iaos", shown);
    const std::string hidden_reason = shown.has_value() && !*shown ? reason : std::string();
    q.set("hidden_reason", hidden_reason);
    std::string note;
    if (model.known && !listed) {
        note = util::format("not listed by IA-OS for unit 0x%02X", model.unit_code.value_or(0)) +
               (hidden_reason.empty() ? std::string() : " (" + hidden_reason + ")");
    } else if (read_only) {
        note = !p.note.empty() ? p.note : std::string("read-only in this driver");
    } else if (!settable) {
        note = "not settable: home pattern ORG (item 9.16) is 0 / FFFFFFFF";
    }
    q.set("model_note", note);
    return q;
}

Json annotated_class_json(const ClassBlock& block) {
    Json data = class_block_json(block);
    std::map<std::pair<int, int>, std::vector<std::string>> params;
    for (const ParamDef& p : ec_user_parameters()) {
        if (p.class_id != block.class_id) continue;
        params[{p.item_id, p.entry}].push_back("P" + std::to_string(p.no));
        if (p.item_id2) params[{*p.item_id2, p.entry}].push_back("P" + std::to_string(p.no));
    }
    Json items = Json::array();
    const auto& raw_items = data.find("items")->as_array();
    for (std::size_t i = 0; i < raw_items.size() && i < block.items.size(); ++i) {
        Json item = raw_items[i];
        const ClassItem& it = block.items[i];
        if (it.item_id) {
            const auto found = params.find({*it.item_id, it.entry});
            item.set("params", found == params.end() ? std::vector<std::string>() : found->second);
            item.set("note", ec_detail::item_note(block.class_id, *it.item_id));
        }
        items.push_back(item);
    }
    data.set("items", items);
    return data;
}

// ------------------------------------------------------------------------------------------ CON view (con_params.py)

const char* con_category(int no, const std::optional<std::string>& fam) {
    if (fam && *fam == "SCON") {
        static const std::map<int, const char*> kScon = {{14, "E"}, {19, "E"}, {20, "E"}, {72, "E"},
                                                         {73, "D"}, {75, "D"}, {76, "D"}, {89, "D"}};
        const auto it = kScon.find(no);
        if (it != kScon.end()) return it->second;
    }
    // IAI's category per parameter number (DOC ME0342-4H p.6-2 to 6-6).
    static const std::map<int, const char*> kCategories = {
        {1, "B"},   {2, "B"},   {3, "A"},   {4, "A"},   {5, "D"},   {6, "C"},   {7, "C"},   {8, "B"},   {9, "B"},
        {10, "B"},  {12, "B"},  {13, "C"},  {15, "B"},  {16, "B"},  {17, "B"},  {18, "E"},  {21, "B"},  {22, "C"},
        {23, "B"},  {24, "B"},  {25, "A"},  {26, "B"},  {27, "B"},  {28, "B"},  {29, "B"},  {30, "B"},  {31, "C"},
        {32, "C"},  {33, "C"},  {34, "C"},  {35, "C"},  {36, "B"},  {37, "B"},  {38, "B"},  {39, "B"},  {40, "C"},
        {41, "C"},  {42, "C"},  {43, "B"},  {45, "B"},  {46, "B"},  {47, "B"},  {48, "B"},  {49, "B"},  {50, "C"},
        {51, "B"},  {52, "B"},  {53, "B"},  {55, "B"},  {56, "B"},  {57, "B"},  {58, "E"},  {59, "C"},  {60, "B"},
        {61, "B"},  {62, "B"},  {63, "B"},  {64, "B"},  {65, "B"},  {66, "B"},  {67, "B"},  {71, "B"},  {77, "D"},
        {78, "D"},  {79, "B"},  {80, "B"},  {83, "B"},  {84, "A"},  {85, "A"},  {86, "A"},  {87, "E"},  {88, "D"},
        {90, "C"},  {91, "C"},  {110, "B"}, {111, "B"}, {112, "B"}, {113, "B"}, {140, "B"}, {141, "B"}, {142, "B"},
        {143, "B"}, {144, "B"}, {145, "C"}, {146, "C"}, {147, "B"}, {148, "B"}, {149, "B"}, {151, "B"}, {152, "B"},
        {153, "B"}, {154, "B"}, {155, "A"}, {156, "B"}, {159, "B"}, {165, "B"}, {166, "B"}, {167, "B"}, {168, "B"},
        {182, "B"}, {187, "C"}, {188, "A"},
    };
    const auto it = kCategories.find(no);
    return it == kCategories.end() ? "" : it->second;
}

const char* con_category_text(const std::string& category) {
    if (category == "A") return "check before use";
    if (category == "B") return "set per application";
    if (category == "C") return "use as shipped";
    if (category == "D") return "actuator-specific, set at shipment (do not change)";
    if (category == "E") return "manufacturer use only (never change)";
    return "";
}

const char* con_range_doc(const std::optional<std::string>& fam) {
    const std::string f = fam.value_or("PCON");
    if (f == "ACON/DCON") return "ME0343-3B p.6-2 to 6-6 (No.53: p.4-17 and the No.53 detail page)";
    if (f == "SCON") return "ME0340-9F p.6-2 to 6-9";
    if (f == "PCON") return "ME0342-4H p.6-2 to 6-6";
    return "";
}

struct MinBand {
    double mm = 0.0;
    std::string formula;
    std::string doc;
    bool certain = true;
};

double ceil_hundredth(double value) { return std::ceil(std::round(value * 100.0 * 1e6) / 1e6) / 100.0; }

/// con_params.min_positioning_band: the smallest positioning band (No.10) the unit accepts.
std::optional<MinBand> min_positioning_band(const std::optional<std::string>& fam, std::optional<double> lead_mm,
                                            std::optional<std::uint32_t> encoder_pulses, std::optional<double> mini_mm) {
    std::vector<MinBand> floors;
    floors.push_back({0.01, "0.01 mm (the input range)", "ME0342-4H p.6-12", true});
    bool certain = true;
    const bool mini = mini_mm && *mini_mm != 0.0;
    if (mini) floors.push_back({*mini_mm, "MINI (3.67), enforced by this unit", "the unit's own limit", true});
    if (lead_mm && *lead_mm != 0.0) {
        const std::uint32_t pulses = encoder_pulses && *encoder_pulses != 0 ? *encoder_pulses : 800u;
        certain = encoder_pulses.has_value();
        const bool low_res = pulses < 4096;
        int factor = 1;
        std::string doc;
        if (fam && *fam == "SCON") {
            doc = "ME0340-9F p.4-20, 4-93";
        } else if (fam && *fam == "ACON/DCON") {
            factor = low_res ? 4 : 1;
            doc = low_res ? "ME0343-3B p.4-13" : "ME0343-3B p.8-21";
        } else {
            factor = low_res ? 3 : 1;
            doc = low_res ? "ME0342-4H p.6-12" : "ME0342-4H p.6-12, p.4-16";
            if (!fam) doc += "; the PCON rule (the family has no rule of its own in the driver)";
        }
        std::string how = "lead " + util::format("%g", *lead_mm) + " / " + std::to_string(pulses) + " pulses" +
                          (factor != 1 ? " x " + std::to_string(factor) : std::string());
        if (!certain) how += " (encoder count not known: 800 assumed)";
        floors.push_back({ceil_hundredth(*lead_mm / pulses * factor), how, doc, true});
    } else if (!mini) {
        return std::nullopt;
    }
    const MinBand* best = &floors[0];
    for (const MinBand& f : floors)
        if (f.mm > best->mm) best = &f;  // the first of equal floors, as Python's max()
    MinBand out = *best;
    out.mm = util::round2(best->mm);
    out.certain = certain || util::starts_with(best->formula, "0.01") || util::starts_with(best->formula, "MINI");
    return out;
}

/// info.annotate_con_parameters: one numbered CON row with the family's documented view.
Json annotate_con_row(Json q, int no, const std::string& family) {
    const ConParam* p = con_parameter(no);
    if (p == nullptr) return q;
    const std::optional<std::string> fam = con_detail::family_of(family);
    const ConParam f = con_detail::for_family(*p, family);
    const std::string cat = con_category(no, fam);
    std::optional<double> lo = f.lo, hi = f.hi;
    bool lo_decimal = f.lo_decimal, hi_decimal = f.hi_decimal;  // floats in the Python edition's table, even when whole
    if (p->kind == "baud") {
        const auto range = con_detail::baud_range();
        lo = range.first;
        hi = range.second;
        lo_decimal = hi_decimal = false;
    }
    const Json* dangerous = q.find("dangerous");
    const bool was_dangerous = dangerous && dangerous->type() == Json::Type::Bool && dangerous->as_bool();
    q.set("family_min", lo ? Json::whole_or_number(*lo, lo_decimal) : Json());
    q.set("family_max", hi ? Json::whole_or_number(*hi, hi_decimal) : Json());
    q.set("category", cat);
    q.set("category_text", con_category_text(cat));
    const std::optional<con_detail::Number> preset = con_detail::factory_default(no, fam);
    q.set("factory_default", preset ? Json::whole_or_number(preset->value, preset->decimal) : Json());
    q.set("range_doc", con_range_doc(fam));
    q.set("dangerous", was_dangerous || f.dangerous);
    q.set("note", con_detail::note_of(no));
    return q;
}

/// info.con_range_check / con_params.audit: what a CON unit holds against the documented ranges (no I/O).
Json con_range_check(const std::vector<ParameterInfo>& numbered, const std::string& family,
                     const con_detail::UnitLimits& limits) {
    const std::optional<std::string> fam = con_detail::family_of(family);
    std::map<int, ParamValue> values;
    for (const ParameterInfo& row : numbered)
        if (row.number != 0) values[row.number] = row.value;
    const std::optional<MinBand> band = min_positioning_band(fam, limits.lead_mm, limits.encoder_pulses,
                                                             limits.min_band_mm);
    Json unit = Json::object();
    unit.set("max_speed", limits.max_speed);
    unit.set("max_accel", limits.max_accel);
    unit.set("max_push_speed", limits.max_push_speed);
    unit.set("lead_mm", limits.lead_mm);
    unit.set("encoder_pulses", limits.encoder_pulses);
    unit.set("min_band_mm", limits.min_band_mm);
    unit.set("rotary", limits.rotary);
    Json band_json;
    if (band) {
        band_json = Json::object();
        band_json.set("mm", band->mm);
        band_json.set("formula", band->formula);
        band_json.set("doc", band->doc);
        band_json.set("certain", band->certain);
    }
    Json findings = Json::array();
    for (const auto& kv : values) {
        const ConParam* p = con_parameter(kv.first);
        if (p == nullptr || kv.second.empty() || !p->writable) continue;
        try {
            con_detail::check_value(*p, kv.second, family, limits);
        } catch (const std::invalid_argument& e) {
            Json f = Json::object();
            f.set("number", kv.first);
            f.set("value", to_json(kv.second));
            f.set("problem", e.what());
            findings.push_back(f);
        }
    }
    Json out = Json::object();
    out.set("family", fam);
    out.set("unit_limits", unit);
    out.set("min_band", band_json);
    out.set("findings", findings);
    out.set("note", fam ? "" : kUnknownFamilyNote);
    return out;
}

// ------------------------------------------------------------------------------------------ section runner

/// Runs the sections of unit_info / backup: a timeout ends the remaining sections ("no answer"); a driver error, a
/// LookupError or a ValueError is reported as "<section>: <Type>: <message>".
class Runner {
public:
    explicit Runner(std::vector<std::string>& errors) : errors_(errors) {}

    void run(const std::string& name, const std::function<void()>& fn) {
        if (lost_) {
            errors_.push_back(name + ": skipped (link lost)");
            return;
        }
        try {
            fn();
        } catch (const WaitCancelledError&) {
            throw;  // Ctrl+C (Axis::cancel_waits) ends the whole read, never one section of it
        } catch (const ModbusTimeout& e) {
            lost_ = true;
            errors_.push_back(name + ": no answer (" + e.what() + ")");
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            errors_.push_back(name + ": " + error_name(e) + ": " + e.what());
        } catch (const std::out_of_range& e) {
            errors_.push_back(name + ": " + error_name(e) + ": " + e.what());
        } catch (const std::invalid_argument& e) {
            errors_.push_back(name + ": " + error_name(e) + ": " + e.what());
        }
    }

    bool lost() const { return lost_; }

private:
    std::vector<std::string>& errors_;
    bool lost_ = false;
};

/// SSSE from monitor words already read, else one FC03 read of 0x9015 (nullopt when it does not answer).
std::optional<std::uint32_t> read_ssse(Axis& axis, const MonitorWords& monitor) {
    if (has(monitor, "SSSE")) return get(monitor, "SSSE");
    try {
        const Words w = axis.read_registers(reg::kMonitorBase + kSsseOffset, 1);
        if (!w.empty()) return w[0];
    } catch (const ModbusSlaveError&) {
    } catch (const ModbusTimeout&) {
    }
    return std::nullopt;
}

Json support_words_json(const SupportWords& support) {
    if (!support) return Json();
    Json out = Json::object();
    for (const auto& member : reg::support_members()) {
        const auto it = support->find(member.first);
        if (it != support->end()) out.set(member.first, it->second);
    }
    return out;
}

/// The rows a backup reads (info._row_selection).
std::vector<int> row_selection(Axis& axis, const IndexCategory& table, const BackupRows& rows) {
    const int count = std::max(table.list_count, 0);
    std::vector<int> out;
    if (rows.selection == "all") {
        for (int n = 0; n < count; ++n) out.push_back(n);
        return out;
    }
    if (rows.selection == "used") {
        std::set<int> used;
        for (int n = 0; n < std::min(axis.max_positions(), count); ++n) used.insert(n);
        if (axis.is_ec())
            for (int n = reg::kSimpleDataFirstRow; n <= reg::kSimpleDataLastRow; ++n)
                if (n < count) used.insert(n);
        return std::vector<int>(used.begin(), used.end());
    }
    if (rows.selection != "list")
        throw std::invalid_argument("rows must be one of ('used', 'all') or row numbers, got '" + rows.selection + "'");
    const std::set<int> chosen(rows.numbers.begin(), rows.numbers.end());
    std::vector<int> bad;
    for (int n : chosen)
        if (n < 0 || n >= count) bad.push_back(n);
    if (!bad.empty())
        throw std::invalid_argument("rows " + py_list(bad) + " are outside 0.." + std::to_string(count - 1) +
                                    " (the directory's position table)");
    return std::vector<int>(chosen.begin(), chosen.end());
}

Json position_entry_fields(const PositionEntry& e) {
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

Json backup_rows_json(Axis& axis, UnitReader& rd, const UnitDirectory& dir, const BackupRows& rows) {
    const IndexCategory& table = dir.get(reg::kPositionTableCategory);
    const int stride = table.member_count * std::max(table.member_size, 1);
    const std::vector<int> chosen = row_selection(axis, table, rows);
    // contiguous (first, count) runs
    std::vector<std::pair<int, int>> spans;
    for (int n : chosen) {
        if (!spans.empty() && spans.back().first + spans.back().second == n) ++spans.back().second;
        else spans.emplace_back(n, 1);
    }
    Json out_rows = Json::array();
    for (const auto& span : spans) {
        const Words words = dir.read_block(rd, reg::kPositionTableCategory, span.first * stride, span.second * stride);
        for (int i = 0; i < span.second; ++i) {
            const int n = span.first + i;
            const Words row = slice(words, static_cast<std::size_t>(i * stride), static_cast<std::size_t>((i + 1) * stride));
            Json entry = Json::object();
            entry.set("n", n);
            entry.set("address", util::format("0x%04X", table.data_address + n * stride));
            entry.set("words", hex_word_list(row));
            if (stride == reg::kPositionStride) {
                const PositionEntry e = PositionEntry::from_registers(row);
                Json decoded = position_entry_fields(e);
                decoded.set("checksum_ok", e.checksum_ok());
                entry.set("decoded", decoded);
            }
            out_rows.push_back(entry);
        }
    }
    Json out = Json::object();
    out.set("stride_words", stride);
    out.set("rows_in_table", table.list_count);
    out.set("selection", rows.selection == "used" || rows.selection == "all" ? rows.selection : std::string("list"));
    out.set("rows", out_rows);
    return out;
}

Json rows_meta(const BackupRows& rows) {
    if (rows.selection != "list") return Json(rows.selection);
    const std::set<int> sorted(rows.numbers.begin(), rows.numbers.end());
    return Json(std::vector<int>(sorted.begin(), sorted.end()));
}

void write_text(const std::string& path, const std::string& text) {
    std::ofstream file(std::filesystem::path(path), std::ios::binary | std::ios::trunc);
    if (!file) throw std::ios_base::failure("could not open '" + path + "' for writing");
    file << text;
    file.flush();
    if (!file) throw std::ios_base::failure("could not write '" + path + "'");
}

std::optional<std::string> json_string(const Json& object, const char* key) {
    const Json* v = object.find(key);
    if (v == nullptr || v->type() != Json::Type::String) return std::nullopt;
    return v->as_string();
}

std::optional<double> json_number(const Json& object, const char* key) {
    const Json* v = object.find(key);
    if (v == nullptr || (v->type() != Json::Type::Int && v->type() != Json::Type::Number)) return std::nullopt;
    return v->as_number();
}

/// An int field of a backup file: a whole number inside the int range, else nullopt (like a value of another type). A
/// fraction or 1e10 is never narrowed: converting a double outside the target type is undefined.
std::optional<int> json_int_field(const Json& object, const char* key) {
    const Json* v = object.find(key);
    const std::optional<long long> n = v ? v->as_whole() : std::nullopt;
    if (!n || *n < INT_MIN || *n > INT_MAX) return std::nullopt;
    return static_cast<int>(*n);
}

const Json* json_object(const Json& object, const char* key) {
    const Json* v = object.find(key);
    return v != nullptr && v->type() == Json::Type::Object ? v : nullptr;
}

const Json* json_array(const Json& object, const char* key) {
    const Json* v = object.find(key);
    return v != nullptr && v->type() == Json::Type::Array ? v : nullptr;
}

}  // namespace

// ============================================================================================ public: tables

const std::vector<std::string>& sections() {
    static const std::vector<std::string> s = {"identity", "parameters", "monitor", "alarms", "maintenance", "positions"};
    return s;
}

const std::vector<int>& parameter_classes() {
    static const std::vector<int> c = {0x0003, 0x0004, 0x0007, 0x0008, 0x0009, 0x000C};
    return c;
}

const std::vector<BitDef>& bit_table(const std::string& name) {
    for (const auto& t : tables().all)
        if (t.first == name) return t.second;
    throw std::out_of_range("no bit table named '" + name + "'");
}

const std::vector<std::string>& bit_table_names() { return tables().names; }

Json decode_bits(std::uint32_t word, const std::vector<BitDef>& table, int width) {
    if (width < 1 || width > 32 || (width < 32 && (static_cast<std::uint64_t>(word) >> width) != 0))
        throw std::invalid_argument(std::to_string(word) + " does not fit in " + std::to_string(width) + " bits");
    std::vector<BitDef> defs = sorted_bits(table);
    std::set<int> named;
    for (const BitDef& d : defs) named.insert(d.bit);
    Json bits = Json::array();
    Json on = Json::array();
    for (const BitDef& d : defs) {
        const bool set = ((word >> d.bit) & 1u) != 0;
        Json b = Json::object();
        b.set("bit", d.bit);
        b.set("name", d.name);
        b.set("on", set);
        b.set("meaning", d.meaning);
        b.set("confirmed", d.confirmed);
        b.set("evidence", d.evidence);
        b.set("active_low", d.active_low);
        bits.push_back(b);
        if (set) on.push_back(d.name);
    }
    Json unknown = Json::array();
    for (int b = width - 1; b >= 0; --b) {
        if (((word >> b) & 1u) == 0 || named.count(b)) continue;
        unknown.push_back(b);
        Json x = Json::object();
        x.set("bit", b);
        x.set("name", "b" + std::to_string(b));
        x.set("on", true);
        x.set("meaning", "");
        x.set("confirmed", false);
        x.set("evidence", "not named");
        x.set("active_low", false);
        bits.push_back(x);
        on.push_back("b" + std::to_string(b));
    }
    Json out = Json::object();
    out.set("value", word);
    out.set("hex", util::format("0x%0*X", width / 4, word));
    out.set("on", on);
    out.set("bits", bits);
    out.set("unknown_set", unknown);
    return out;
}

Json decode_bits(std::uint32_t word, const std::string& table, int width) {
    return decode_bits(word, bit_table(table), width);
}

Json position_mode(std::optional<std::uint32_t> code, const std::optional<std::string>& model) {
    auto make = [](Json code_json, const char* name, int positions, const char* layout, std::string source,
                   bool confirmed, std::string note) {
        Json out = Json::object();
        out.set("code", std::move(code_json));
        out.set("name", name);
        out.set("positions", positions);
        out.set("layout", layout);
        out.set("source", std::move(source));
        out.set("confirmed", confirmed);
        out.set("note", std::move(note));
        return out;
    };
    if (code && *code == 4)
        return make(4, "3-position", 3, "3pos", "user parameter 9 = 4", false,
                    "4 seen on the 3-position slider only (model option MF)");
    if (code && *code == 0)
        return make(0, "2-position (double solenoid)", 2, "2pos", "user parameter 9 = 0", true, "");
    if (code && *code == 3)
        return make(3, "2-position (single solenoid)", 2, "2pos", "user parameter 9 = 3", true, "");
    const std::string upper = util::upper(model.value_or(""));
    const bool mf = !upper.empty() && (upper.find("-MF") != std::string::npos ||
                                       (upper.size() >= 2 && upper.compare(upper.size() - 2, 2, "MF") == 0));
    if (!code && mf)
        return make(Json(), "3-position", 3, "3pos", "model string option MF", false, "user parameter 9 not readable");
    const std::string why = code ? "unknown user parameter 9 value " + std::to_string(*code)
                                 : std::string("user parameter 9 not readable");
    return make(code ? Json(*code) : Json(), "2-position (assumed)", 2, "2pos", "assumed", false, why);
}

IoTables io_bit_tables(const std::string& layout) {
    IoTables t;
    if (layout == "3pos") {
        t.dipm = &bit_table("DIPM_3POS");
        t.dopm = &bit_table("DOPM_3POS");
    } else {
        t.dipm = &bit_table("DIPM_2POS");
        t.dopm = &bit_table("DOPM_2POS");
    }
    return t;
}

IoTables io_bit_tables(const Json& mode) {
    const Json* layout = mode.find("layout");
    const std::string name = layout && layout->type() == Json::Type::String ? layout->as_string() : std::string();
    if (name == "3pos") return io_bit_tables(name);
    IoTables t = io_bit_tables(std::string("2pos"));
    const Json* code = mode.find("code");
    if (code && (code->type() == Json::Type::Int || code->type() == Json::Type::Number) && code->as_int() == 3)
        t.dipm = &bit_table("DIPM_SINGLE");
    return t;
}

// ============================================================================================ public: decoders

MonitorWords split_monitor(const std::vector<std::uint16_t>& words) {
    MonitorWords out;
    std::size_t offset = 0;
    for (const Member& m : kMonitorLayout) {
        if (offset + static_cast<std::size_t>(m.size) > words.size()) break;
        out[m.name] = m.size == 1 ? words[offset] : reg::from_u32_words(words[offset], words[offset + 1]);
        offset += static_cast<std::size_t>(m.size);
    }
    return out;
}

MonitorWords monitor_words(const StatusRaw& raw) {
    MonitorWords m;
    m["PNOW"] = raw.pnow;
    m["ALMC"] = raw.almc;
    m["DIPM"] = raw.dipm;
    m["DOPM"] = raw.dopm;
    m["DSS1"] = raw.dss1;
    m["DSS2"] = raw.dss2;
    m["DSSE"] = raw.dsse;
    if (raw.stat) m["STAT"] = *raw.stat;
    if (raw.vnow) m["VNOW"] = *raw.vnow;
    if (raw.cnow) m["CNOW"] = *raw.cnow;
    if (raw.devi) m["DEVI"] = *raw.devi;
    if (raw.stim) m["STIM"] = *raw.stim;
    if (raw.sipm) m["SIPM"] = *raw.sipm;
    if (raw.zons) m["ZONS"] = *raw.zons;
    if (raw.poss) m["POSS"] = *raw.poss;
    if (raw.ssse) m["SSSE"] = *raw.ssse;
    return m;
}

Json decode_monitor(const MonitorWords& words, const Json& mode, std::optional<std::uint32_t> rated_current) {
    const Json m = mode.is_null() ? position_mode(std::nullopt) : mode;
    Json out = Json::object();
    out.set("monitor", monitor_json(words, rated_current));
    out.set("status_words", status_words_json(words));
    out.set("io", io_json(words, m));
    return out;
}

Json directory_summary(const UnitDirectory& directory) {
    Json rows = Json::array();
    long long total = 0;
    for (const IndexCategory& rec : directory.records()) {
        const reg::BlockExtent ext = rec.extent();
        Json row = Json::object();
        row.set("class_id", rec.category_id);
        row.set("class", util::format("0x%04X", rec.category_id));
        row.set("name", reg::category_name(rec.category_id));
        row.set("address", util::format("0x%04X", rec.data_address));
        row.set("end", util::format("0x%04X", rec.data_address + ext.words - 1));
        row.set("words", ext.words);
        row.set("exact", ext.exact);
        row.set("words_max", ext.words_max);
        row.set("words_readable", directory.readable_words(rec));
        row.set("members", rec.member_count);
        row.set("entries", rec.list_count);
        row.set("member_size", rec.member_size);
        row.set("id_map", rec.id_list_address ? Json(util::format("0x%04X", rec.id_list_address)) : Json());
        row.set("coil_base", rec.coil_base ? Json(util::format("0x%04X", rec.coil_base)) : Json());
        row.set("flags", util::format("0x%04X", rec.model_code));
        rows.push_back(row);
        total += ext.words;
    }
    Json header;
    if (directory.header()) {
        const IndexHeader& h = *directory.header();
        header = Json::object();
        header.set("model_code", h.model_code);
        header.set("words_per_record", h.words_per_record);
        header.set("record_count", h.record_count);
        header.set("rx_buffer", h.rx_buffer);
        header.set("tx_buffer", h.tx_buffer);
    }
    Json out = Json::object();
    out.set("header", header);
    out.set("record_count", static_cast<int>(rows.size()));
    out.set("total_words", total);
    out.set("records", rows);
    return out;
}

Json class_block_json(const ClassBlock& block) {
    Json items = Json::array();
    for (const ClassItem& it : block.items) {
        Json i = Json::object();
        i.set("key", it.key());
        i.set("id", it.item_id);
        i.set("entry", it.entry);
        i.set("index", it.index);
        i.set("address", util::format("0x%04X", it.address));
        i.set("words", it.size);
        i.set("hex", it.hex());
        i.set("unsigned", it.raw);
        i.set("signed", it.signed_value());
        i.set("scaled", util::round2(it.signed_value() * 0.01));  // x 0.01: the usual mm / mm/s / G unit
        items.push_back(i);
    }
    Json out = Json::object();
    out.set("class_id", block.class_id);
    out.set("class", util::format("0x%04X", block.class_id));
    out.set("name", reg::category_name(block.class_id));
    out.set("address", util::format("0x%04X", block.address));
    out.set("words", block.words);
    out.set("member_size", block.member_size);
    out.set("entries", block.entries);
    out.set("id_map", block.id_map_address ? Json(util::format("0x%04X", *block.id_map_address)) : Json());
    out.set("items", items);
    return out;
}

Json decode_support_words(const std::optional<std::map<std::string, std::uint32_t>>& words) {
    if (!words) return Json();
    // Bit names of the class-1 words (IAI bit names, meanings UNVERIFIED; values VERIFIED-LIVE on the slider / gripper).
    static const std::map<std::string, std::map<int, const char*>> kBits = {
        {"SUPT", {{0, "STOP"}, {1, "EMG"}, {2, "SRST software reset"}, {3, "ROAX index mode (not 'rotary')"},
                  {4, "JOGH"}, {5, "AUNT absolute unit"}, {6, "SFTY safety speed"}, {7, "PIO"},
                  {8, "INCM relative moves"}, {9, "ABSM absolute moves"}, {10, "PULS"}, {11, "PUSH push motion"},
                  {12, "DRAT"}, {13, "ABSE absolute encoder"}, {14, "CPIO"}, {15, "AXSW"}, {29, "MBRT Modbus RTU"},
                  {30, "MBAS Modbus ASCII"}, {31, "MBTC Modbus TCP"}}},
        {"MVPT", {{0, "TRAP"}, {1, "SCUR"}, {2, "PFIL"}, {4, "PTPM"}, {5, "VELM"}, {6, "TRQM"}, {7, "DDRV"},
                  {8, "ASOF"}, {9, "SMOD"}, {10, "VCTL vibration control"}, {11, "GSLC"}, {12, "ASOE"},
                  {13, "CDAS collision detection"}}},
        {"SPTS", {{0, "LS"}, {1, "CREP"}, {2, "OT"}, {3, "HMCK home check sensor"}, {4, "BLCT"},
                  {7, "ENBL enable switch"}, {8, "MDSW"}, {9, "PTSW"}, {10, "FANE"}, {11, "SDWN"}, {12, "PP"},
                  {13, "SWJ+"}, {14, "NP"}, {15, "SWJ-"}}},
        {"SUP2", {{0, "FCTL"}, {1, "MONI monitoring"}, {2, "RTC"}, {3, "OVLW overload warning"}, {4, "TKTM tact time"},
                  {5, "MACA"}, {6, "UNIT"}, {7, "TRG monitoring trigger"}, {12, "BTRG"}, {13, "MNSC 0x46"},
                  {18, "MABS battery-less ABS"}, {19, "MAPI pairing ID"}, {23, "CNWL wireless module"},
                  {24, "CINP min. band limit"}, {25, "CVEL min. speed limit"}, {28, "STO"}}},
        {"SUP3", {{0, "VAX virtual axis"}, {1, "SLUP"}, {2, "TNF"}, {3, "DREC"}, {4, "RALM"},
                  {5, "SORS ACR ready signal"}, {16, "IVC1 uses 0x48"}}},
    };
    Json out = Json::object();
    Json decoded = Json::object();
    for (const auto& member : reg::support_members()) {
        const auto it = words->find(member.first);
        if (it == words->end()) continue;
        const std::string& name = it->first;
        const std::uint32_t value = it->second;
        const bool wide = name == "SUPT" || name == "SUP2" || name == "AEHS" || name == "ACHS" || name == "ARSF" ||
                          name == "SPTM" || name == "SUP3";
        const int width = wide ? 8 : 4;
        const auto names = kBits.find(name);
        Json on = Json::array();
        for (int b = 0; b < 4 * width; ++b) {
            if (((value >> b) & 1u) == 0) continue;
            const char* label = nullptr;
            if (names != kBits.end()) {
                const auto n = names->second.find(b);
                if (n != names->second.end()) label = n->second;
            }
            on.push_back(label ? std::string(label) : "b" + std::to_string(b));
        }
        Json w = Json::object();
        w.set("value", value);
        w.set("hex", util::format("0x%0*X", width, value));
        w.set("on", on);
        decoded.set(name, w);
    }
    out.set("words", decoded);
    out.set("evidence", "IAI bit names (meanings UNVERIFIED); values read from the EC");
    auto word = [&](const char* key) -> std::optional<std::uint32_t> {
        const auto it = words->find(key);
        return it == words->end() ? std::nullopt : std::optional<std::uint32_t>(it->second);
    };
    const auto supt = word("SUPT"), mvpt = word("MVPT"), sup2 = word("SUP2"), sup3 = word("SUP3");
    auto flag = [](std::optional<std::uint32_t> w, std::uint32_t bit) {
        return w ? Json((*w & bit) != 0) : Json();
    };
    Json flags = Json::object();
    flags.set("push", flag(supt, reg::kSuptPush));
    flags.set("relative_moves", flag(supt, reg::kSuptIncm));
    flags.set("software_reset", flag(supt, reg::kSuptSrst));
    flags.set("collision_detection", flag(mvpt, kMvptCdas));
    flags.set("vibration_control", flag(mvpt, kMvptVctl));
    flags.set("wireless_module", flag(sup2, kSup2Cnwl));
    flags.set("absolute_encoder", absolute_encoder_from_support(&*words));
    flags.set("uses_function_0x48", sup3 ? (*sup3 & kSup3Ivc1) != 0 : false);
    flags.set("has_sup3", sup3.has_value());
    out.set("flags", flags);
    return out;
}

std::optional<bool> rtc_in_use(std::optional<std::uint32_t> ssse) {
    if (!ssse) return std::nullopt;
    return (*ssse & kSsseUrtc) != 0;
}

Json alarm_time(std::optional<std::int64_t> time_raw, std::optional<bool> rtc) {
    Json out = Json::object();
    if (!time_raw) {
        out.set("time_raw", Json());
        out.set("clock", Json());
        out.set("occurred_at", Json());
        out.set("time_after_power_on", Json());
        return out;
    }
    out.set("time_raw", *time_raw);
    if (rtc.value_or(false)) {
        out.set("clock", "rtc");
        out.set("occurred_at", rtc_datetime(*time_raw));
        out.set("time_after_power_on", Json());
    } else {
        out.set("clock", "power_on");
        out.set("occurred_at", Json());
        out.set("time_after_power_on", after_power_on(*time_raw));
    }
    return out;
}

Json decode_alarm_history(const IndexCategory& record, const std::vector<std::uint16_t>& words,
                          std::optional<std::uint32_t> current, std::optional<std::uint32_t> dss1,
                          std::optional<std::uint32_t> c3, std::optional<bool> absolute_encoder,
                          std::optional<bool> rtc_in_use, std::optional<int> system_code) {
    return alarms_json(record, words, current, dss1, c3, absolute_encoder, rtc_in_use, system_code);
}

Json maintenance_json(const ClassBlock& block, std::optional<bool> rtc, const std::optional<std::string>& family) {
    struct Item {
        const char* symbol;
        const char* meaning;
        bool confirmed;
    };
    // Item ID -> (symbol, meaning, confirmed). TLMC / ODOM VERIFIED-LIVE (TLMC +1 per move start); the other symbols
    // are IAI's maintenance item names in their expected order (UNVERIFIED).
    static const std::map<int, Item> kItems = {
        {1, {"TLMC", "total moves (move starts, retargets included)", true}},
        {2, {"ODOM", "total distance (m)", true}},
        {3, {"FMAX", "", false}}, {4, {"VMAX", "", false}}, {5, {"MMAX", "", false}}, {6, {"CMAX", "", false}},
        {7, {"TTIM", "", false}}, {8, {"EMGC", "", false}},
        {9, {"TEMP", "two 16-bit values (rose by 2 after moves); temperatures?", false}},
        {10, {"TTPM", "", false}}, {11, {"ADMX", "", false}}, {12, {"TARM", "", false}}, {13, {"TACM", "", false}},
        {14, {"OLMX", "", false}}, {15, {"TOLM", "", false}}, {16, {"TLM2", "", false}}, {17, {"ODO2", "", false}},
        {18, {"TIMN", "", false}}, {19, {"TFMX", "", false}}, {20, {"TVMX", "", false}}, {21, {"TMMX", "", false}},
        {22, {"TCMX", "", false}}, {23, {"TLM3", "", false}}, {24, {"ODO3", "", false}}, {25, {"ACHT", "", false}},
        {26, {"TFAN", "", false}}, {27, {"CAOC", "", false}}, {28, {"CAOH", "", false}}, {29, {"CAMV", "", false}},
    };
    // TIMN (the calendar's current time) per CON family (DOC ME0162-12A p.4-16).
    std::optional<int> timn;
    const std::optional<std::string> fam = con_detail::family_of(family.value_or(""));
    if (fam) timn = *fam == "SCON" ? 0x841E : *fam == "PCON" ? 0x8420 : 0x8422;
    Json calendar_time;
    Json items = Json::array();
    for (const ClassItem& it : block.items) {
        const auto known = it.item_id ? kItems.find(*it.item_id) : kItems.end();
        const Item def = known == kItems.end() ? Item{"?", "", false} : known->second;
        Json entry = Json::object();
        entry.set("id", it.item_id);
        entry.set("symbol", def.symbol);
        entry.set("meaning", def.meaning);
        entry.set("confirmed", def.confirmed);
        entry.set("address", util::format("0x%04X", it.address));
        entry.set("hex", it.hex());
        entry.set("value", it.raw);
        entry.set("unset", it.raw == 0xFFFFFFFFu);
        if (std::string(def.symbol) == "TEMP") {
            entry.set("hi16", it.raw >> 16);
            entry.set("lo16", it.raw & 0xFFFF);
        }
        if (timn && it.address == *timn && it.size == 2) {
            entry.set("meaning", "TIMN: the calendar's current time (DOC ME0162-12A p.4-16)");
            if (rtc.value_or(false) && it.raw != 0xFFFFFFFFu) {
                calendar_time = Json(rtc_datetime(it.raw));
                entry.set("time", calendar_time);
            }
        }
        items.push_back(entry);
    }
    const ClassItem* moves = block.item(kMaintenanceTotalMovesId);
    const ClassItem* dist = block.item(kMaintenanceDistanceId);
    Json out = Json::object();
    out.set("address", util::format("0x%04X", block.address));
    out.set("total_moves", moves ? Json(moves->raw) : Json());
    out.set("distance_m", dist ? Json(dist->raw) : Json());
    out.set("calendar_time", calendar_time);
    out.set("items", items);
    out.set("note", "TLMC / ODOM VERIFIED-LIVE (total distance in m: DOC ME0162-12A p.4-15); the other symbols are in "
                    "their expected order (UNVERIFIED); FFFFFFFF = not set; calendar_time = TIMN as seconds since "
                    "2000-01-01 when the unit's calendar is in use (SSSE b8)");
    return out;
}

// ============================================================================================ public: unit_info

Json unit_info(Axis& axis, const std::optional<std::vector<std::string>>& requested) {
    std::vector<std::string> wanted;
    if (!requested) {
        wanted = sections();
    } else {
        for (const std::string& s : *requested)
            if (!contains(wanted, s)) wanted.push_back(s);
    }
    std::vector<std::string> unknown;
    for (const std::string& s : wanted)
        if (!contains(sections(), s)) unknown.push_back(s);
    if (!unknown.empty()) {
        auto quoted = [](const std::vector<std::string>& list) {
            std::string out = "[";
            for (std::size_t i = 0; i < list.size(); ++i) out += (i ? ", '" : "'") + list[i] + "'";
            return out + "]";
        };
        throw std::invalid_argument("unknown section(s) " + quoted(unknown) + "; choose from " + quoted(sections()));
    }
    auto want = [&](const char* s) { return contains(wanted, s); };
    const double started = now_s();
    // cancellable like a motion wait (Axis::cancel_waits, a binding's Ctrl+C): the next request ends the read with
    // WaitCancelledError; READ-ONLY, so nothing is sent for it
    const CancelScope cancel_scope(detail::AxisAccess::impl(axis).cancel_token);

    Json meta = Json::object();
    meta.set("schema", kUnitInfoSchema);
    meta.set("read_only", true);
    meta.set("function_codes", std::vector<std::string>{"03", "11"});
    meta.set("taken_at", local_timestamp());
    meta.set("driver_version", kVersion);
    meta.set("sections", wanted);
    Json result = Json::object();
    result.set("meta", meta);
    for (const char* key : {"identity", "parameters", "monitor", "status_words", "io", "control_words", "alarms",
                            "maintenance", "positions"})
        result.set(key, Json());
    result.set("errors", Json::array());
    std::vector<std::string> errors;

    axis.read_registers(reg::kPnow, 2);  // link probe: a timeout here propagates (nothing answers)

    std::unique_ptr<UnitReader> reader = axis.bus()->reader(axis.slave());
    UnitReader& rd = *reader;
    Runner runner(errors);
    std::optional<DeviceInfo> dev;
    UnitDirectory dir;
    std::map<int, ClassBlock> classes;
    MonitorWords monitor;
    SupportWords support;
    bool rtc_known = false;
    std::optional<bool> rtc_value;

    runner.run("directory", [&] {
        if (want("identity")) {
            dev = axis.identify();
            dir = UnitDirectory(dev->index_header, dev->categories);
            if (dev->categories.empty())
                throw std::out_of_range("index table not readable: " + util::join(dev->errors, "; "));
        } else {
            dir = UnitDirectory::read(rd);
        }
    });
    const bool records = !dir.empty();

    if (records && (want("parameters") || want("identity") || want("monitor") || want("positions"))) {
        runner.run("parameters", [&] {
            const bool full = want("parameters") || want("identity");
            std::vector<int> wanted_classes;
            if (full) {
                wanted_classes = parameter_classes();
            } else {
                wanted_classes.push_back(reg::kMotionParamsCategory);
                if (want("monitor")) wanted_classes.push_back(reg::kMonitor2Category);
            }
            for (int ci : wanted_classes) {
                try {
                    classes[ci] = dir.read_class(rd, ci);
                } catch (const ModbusSlaveError& e) {
                    errors.push_back(util::format("parameters: class 0x%04X: ", ci) + e.what());
                } catch (const std::out_of_range& e) {
                    errors.push_back(std::string("parameters: ") + e.what());
                }
            }
            if (full) {
                try {
                    support = dir.read_support_words(rd);
                } catch (const ModbusSlaveError& e) {
                    errors.push_back(std::string("identity: class 1 support words: ") + e.what());
                }
            }
        });
    }
    const ClassItem* p9 = class_item(classes, reg::kMotionParamsCategory, kPositionModeItem);
    const Json mode = position_mode(p9 ? std::optional<std::uint32_t>(p9->raw) : std::nullopt,
                                    dev ? std::optional<std::string>(dev->model()) : std::nullopt);

    auto system_code = [&]() -> std::optional<int> {
        if (dev && dev->system_code) return dev->system_code;
        const std::optional<DeviceInfo> cached = axis.device_info();
        return cached ? cached->system_code : std::nullopt;
    };
    auto rtc = [&](const MonitorWords& m) -> std::optional<bool> {
        if (!rtc_known) {
            rtc_value = rtc_in_use(read_ssse(axis, m));
            rtc_known = true;
        }
        return rtc_value;
    };

    if (want("identity") && dev) {
        runner.run("identity", [&] {
            Json data = without(to_json(*dev), {"categories", "index_header", "errors"});
            Json extra = Json::object();
            try {
                extra = identification_extra(dir.read_block(rd, reg::kIdentificationCategory));
            } catch (const ModbusSlaveError& e) {
                errors.push_back(std::string("identity: identification block: ") + e.what());
            } catch (const std::out_of_range& e) {
                errors.push_back(std::string("identity: identification block: ") + e.what());
            } catch (const std::invalid_argument& e) {
                errors.push_back(std::string("identity: identification block: ") + e.what());
            }
            std::optional<AxisLimits> limits;
            try {
                limits = axis.read_axis_limits();
            } catch (const ModbusTimeout&) {
                throw;
            } catch (const WaitCancelledError&) {
                throw;
            } catch (const CallerAbortError&) {
                throw;  // the caller's own error ends the call
            } catch (const EcDriverError&) {
                limits.reset();  // like an Elecylinder without axis_limits: the capabilities leave it out
            }
            const AxisRanges ranges = axis.limits();
            const AxisOptions options = axis.options();
            data.set("slave", axis.slave());
            data.set("baudrate", axis.baud_rate());
            data.set("baud_detected", axis.baud_detected());
            data.set("stroke_mm", span_json(unit_stroke(dev->soft_limits_mm)));
            data.set("configured_stroke_mm", span_json(options.stroke));
            data.set("effective_stroke_mm", span_json(axis.effective_stroke()));
            data.set("ec_max_speed_mm_s", dev->max_speed_mm_s);
            data.set("ec_max_speed_source", dev->speed_limit_source);
            data.set("ec_max_accel_g", dev->max_accel_g);
            data.set("ec_max_decel_g", dev->max_decel_g);
            data.set("speed_cap_mm_s", ranges.speed.max);
            data.set("max_accel_g", options.max_accel_g);
            data.set("accel_cap_g", ranges.accel_decel.max);
            data.set("push_cap_pct", ranges.push.max);
            data.set("units", dev->units);
            data.set("kind", dev->kind());
            data.set("product_family", dev->product_family);
            data.set("position_mode", mode);
            data.set("identification_extra", extra);
            data.set("capabilities", capabilities_json(classes, support, limits));
            data.set("directory", directory_summary(dir));
            data.set("notes", dev->errors);
            result.set("identity", data);
        });
    }

    if (want("parameters") && !classes.empty()) {
        runner.run("parameters", [&] {
            std::optional<int> unit_code, sys;
            std::optional<std::string> model_name;
            if (dev) {
                unit_code = dev->unit_code;
                model_name = dev->model_name;
                sys = dev->system_code;
            } else {
                const ParameterModel found = axis.parameter_model();  // FC11 / CTCL + the model string (read-only)
                unit_code = found.unit_code;
                model_name = found.model_name;
                sys = found.system_code;
            }
            std::optional<std::uint32_t> accl = dev ? dev->accl : std::nullopt;
            if (!accl) {
                try {
                    accl = dir.read_accl(rd);
                } catch (const ModbusSlaveError& e) {
                    errors.push_back(std::string("parameters: ACCL (0x0612): ") + e.what());
                }
            }
            ParameterContext ctx;
            ctx.is_ec = true;  // the EC P-number list and its rules, as the Python edition's unit_info
            ctx.classes = classes;
            ctx.support = support;
            ctx.accl = accl;
            ctx.unit_code = unit_code;
            ctx.system_code = sys;
            ctx.model_name = model_name;
            const std::map<int, ParameterVisibility> visibility = ec_detail::visibility(ctx);
            const ParameterModel model = model_parameter_list(ctx);
            Json named = Json::array();
            for (const ParamDef& p : ec_user_parameters())
                named.push_back(annotate_parameter(decode_user_parameter(p, classes), p, model, visibility));
            Json numbered = Json::array();
            std::vector<ParameterInfo> numbered_rows;
            if (sys && !reg::is_ec_family(sys)) {
                try {
                    numbered_rows = axis.parameters();  // the CON parameter numbers this unit's ID map has
                } catch (const ModbusSlaveError& e) {
                    errors.push_back(std::string("parameters: numbered CON parameters: ") + e.what());
                } catch (const std::out_of_range& e) {
                    errors.push_back(std::string("parameters: numbered CON parameters: ") + e.what());
                } catch (const std::invalid_argument& e) {
                    errors.push_back(std::string("parameters: numbered CON parameters: ") + e.what());
                }
                named = Json::array();  // the EC P-numbers do not apply to a CON controller
            }
            Json range_check;
            if (!numbered_rows.empty()) {
                const std::string family = reg::product_family(sys);
                const con_detail::UnitLimits limits = con_detail::unit_limits(classes, support);
                for (const ParameterInfo& row : numbered_rows)  // the raw-item fields do not apply to a numbered row
                    numbered.push_back(annotate_con_row(without(to_json(row), {"unsigned", "signed", "words", "named"}),
                                                        row.number, family));
                range_check = con_range_check(numbered_rows, family, limits);
            }
            Json class_list = Json::array();
            for (int ci : parameter_classes()) {
                const auto it = classes.find(ci);
                if (it != classes.end()) class_list.push_back(annotated_class_json(it->second));
            }
            Json params = Json::object();
            params.set("named", named);
            params.set("numbered", numbered);
            params.set("range_check", range_check);
            params.set("model", to_json(model));
            params.set("not_located", Json::array());  // P8 / P11 / P13-P18 are located (read-only)
            params.set("classes", class_list);
            params.set("note", std::string("named parameters: P1-P7, P9, P10, P12 ") + kParamEvidence +
                                   "; P3 decoded by the parity of the ORG home pattern; P8, P11, P13-P18 at UNVERIFIED "
                                   "locations and read-only; listed = what IA-OS's screen shows (parameter_visibility "
                                   "rules); every item is also listed raw by class");
            params.set("write_note", kParamCommitNote);
            result.set("parameters", params);
        });
    }

    if (want("monitor")) {
        runner.run("monitor", [&] {
            Words words;
            try {
                words = dir.read_block(rd, reg::kMonitorCategory);
            } catch (const std::out_of_range&) {
                words = axis.read_registers(reg::kMonitorBase, reg::kStatusBlockWords);  // verified on both units
            }
            monitor = split_monitor(words);
            const ClassItem* mcur = class_item(classes, reg::kMonitor2Category, kMonitor2ItemMcur);
            const Json live = decode_monitor(monitor, mode, mcur ? std::optional<std::uint32_t>(mcur->raw) : std::nullopt);
            result.set("monitor", *live.find("monitor"));
            result.set("status_words", *live.find("status_words"));
            result.set("io", *live.find("io"));
        });
        if (records)
            runner.run("control_words",
                       [&] { result.set("control_words", control_words_json(dir.read_class(rd, reg::kIoControlCategory))); });
    }

    if (want("alarms") && records) {
        runner.run("alarms", [&] {
            const IndexCategory& rec = dir.get(reg::kAlarmHistoryCategory);
            const Words words = dir.read_block(rd, reg::kAlarmHistoryCategory);
            const MonitorWords m =
                !monitor.empty() ? monitor : split_monitor(axis.read_registers(reg::kMonitorBase, reg::kStatusMinWords));
            std::optional<ClassBlock> c3_block;
            const auto known = classes.find(reg::kControllerMonitorCategory);
            if (known != classes.end()) {
                c3_block = known->second;
            } else {
                try {
                    c3_block = dir.read_class(rd, reg::kControllerMonitorCategory);
                } catch (const ModbusSlaveError&) {
                } catch (const std::out_of_range&) {
                }
            }
            const ClassItem* c3 = c3_block ? c3_block->item(kCurrentAlarmItem) : nullptr;
            if (!support) {  // for the 0x0EE level (absolute vs incremental unit)
                try {
                    support = dir.read_support_words(rd);
                } catch (const ModbusSlaveError&) {
                    // absolute_encoder stays unknown
                }
            }
            const std::optional<bool> absolute = absolute_encoder_from_support(support ? &*support : nullptr);
            result.set("alarms", alarms_json(rec, words, opt(m, "ALMC"), opt(m, "DSS1"),
                                             c3 ? std::optional<std::uint32_t>(c3->raw) : std::nullopt, absolute, rtc(m),
                                             system_code()));
        });
    }

    if (want("maintenance") && records) {
        runner.run("maintenance", [&] {
            const std::string family = reg::product_family(system_code());
            result.set("maintenance", maintenance_json(dir.read_class(rd, reg::kMaintenanceCategory), rtc(monitor), family));
        });
    }

    if (want("positions") && records) {
        runner.run("positions", [&] {
            const Json* positions = mode.find("positions");
            const bool three = positions && positions->type() == Json::Type::Int && positions->as_int() == 3;
            static const char* const kRoles3[] = {"backward end", "forward end", "middle"};
            static const char* const kRoles2[] = {"backward end", "forward end", "unused in 2-position mode"};
            const IndexCategory& table = dir.get(reg::kPositionTableCategory);
            const int stride = reg::kPositionStride;
            Json out = Json::object();
            out.set("mode", *mode.find("name"));
            Json rows = Json::array();
            Words words = dir.read_block(rd, reg::kPositionTableCategory, 0, 3 * stride);
            for (int n = 0; n < 3; ++n)
                rows.push_back(row_json(n, table.data_address + n * stride,
                                        slice(words, static_cast<std::size_t>(n * stride), static_cast<std::size_t>((n + 1) * stride)),
                                        three ? kRoles3[n] : kRoles2[n]));
            out.set("rows", rows);
            Json simple = Json::array();
            const int first = reg::kSimpleDataFirstRow + 4;  // rows 260-263: present / previous % (Bwd, Fwd)
            words = dir.read_block(rd, reg::kPositionTableCategory, first * stride, 4 * stride);
            for (int i = 0; i < 4; ++i)
                simple.push_back(simple_row_json(first + i, table.data_address + (first + i) * stride,
                                                 slice(words, static_cast<std::size_t>(i * stride), static_cast<std::size_t>((i + 1) * stride))));
            if (three) {  // the Middle % row is 266, not 263 (UNVERIFIED)
                const int n = 266;
                simple.push_back(simple_row_json(n, table.data_address + n * stride,
                                                 dir.read_block(rd, reg::kPositionTableCategory, n * stride, stride)));
            }
            out.set("simple_data_rows", simple);
            out.set("simple_data_note",
                    "rows of IA-OS's 'simple data' page: 260 / 261 present % (Bwd / Fwd), 262 / 263 the PREVIOUS % "
                    "values, 266 the Middle present % (roles UNVERIFIED); speed = % of MAXV (7.4), accel / decel (+0x0A "
                    "/ +0x0B) = % of MAXA / MAXD (7.5 / 7.6), floor on writing. IA-OS shows these rows, not rows 0-2, "
                    "and they are not updated when rows 0-2 are written (VERIFIED); an IA-OS Transfer is expected to "
                    "rewrite rows 0-2 from them (UNVERIFIED); never written by this driver");
            auto guarded = [&](const char* what, const std::function<Json()>& fn) {
                try {
                    return fn();
                } catch (const ModbusSlaveError& e) {
                    errors.push_back(std::string("positions: ") + what + ": " + e.what());
                } catch (const std::out_of_range& e) {
                    errors.push_back(std::string("positions: ") + what + ": " + e.what());
                } catch (const std::invalid_argument& e) {
                    errors.push_back(std::string("positions: ") + what + ": " + e.what());
                }
                return Json();
            };
            const Json direct = guarded("0x9900 block", [&] { return direct_move_json(dir.read_block(rd, reg::kCommand2Category)); });
            if (!direct.is_null()) out.set("direct_move", direct);
            const Json profile =
                guarded("0x0F00 block", [&] { return active_profile_json(dir.read_block(rd, kActiveProfileCategory)); });
            if (!profile.is_null()) out.set("active_profile", profile);
            result.set("positions", out);
        });
    }

    for (const std::string& name : wanted)
        if (name != "identity" && !records && name != "monitor") errors.push_back(name + ": skipped (no directory)");
    meta.set("duration_s", round_to(now_s() - started, 3));
    result.set("meta", meta);
    result.set("errors", errors);
    return result;
}

// ============================================================================================ public: backup

BackupRows BackupRows::all() {
    BackupRows r;
    r.selection = "all";
    return r;
}

BackupRows BackupRows::list(std::vector<int> numbers) {
    BackupRows r;
    r.selection = "list";
    r.numbers = std::move(numbers);
    return r;
}

BackupRows BackupRows::parse(const std::string& text) {
    if (text == "used") return used();
    if (text == "all") return all();
    const std::string why = "--rows must be used, all or row numbers such as 0,1,2 (got '" + text + "')";
    std::string compact;
    for (char c : text)
        if (c != ' ') compact += c;
    std::vector<int> rows;
    for (const std::string& part : util::split(compact, ',')) {
        if (part.empty()) continue;
        const std::optional<long long> n = util::parse_int(part);
        if (!n) throw std::invalid_argument(why);
        rows.push_back(static_cast<int>(*n));
    }
    if (rows.empty()) throw std::invalid_argument(why);
    for (int r : rows)
        if (r < 0) throw std::invalid_argument(why);
    return list(rows);
}

Json backup(Axis& axis, const BackupRows& rows, const std::string& path) {
    const double started = now_s();
    const CancelScope cancel_scope(detail::AxisAccess::impl(axis).cancel_token);  // as unit_info: Ctrl+C ends it
    std::vector<std::string> errors;
    Json meta = Json::object();
    meta.set("schema", kBackupSchema);
    meta.set("read_only", true);
    meta.set("function_codes", std::vector<std::string>{"03", "11"});
    meta.set("taken_at", local_timestamp());
    meta.set("driver_version", kVersion);
    meta.set("rows", rows_meta(rows));
    Json out = Json::object();
    out.set("meta", meta);
    for (const char* key : {"identity", "directory", "support", "accl", "parameters", "positions", "alarms",
                            "maintenance"})
        out.set(key, Json());
    out.set("errors", Json::array());

    axis.read_registers(reg::kPnow, 2);  // link probe: a timeout here propagates (nothing answers)

    std::unique_ptr<UnitReader> reader = axis.bus()->reader(axis.slave());
    UnitReader& rd = *reader;
    Runner runner(errors);
    UnitDirectory dir;
    std::map<int, ClassBlock> classes;
    std::optional<int> system;

    runner.run("identity", [&] {
        const DeviceInfo dev = axis.identify();
        Json data = without(to_json(dev), {"categories", "index_header", "errors"});
        data.set("slave", axis.slave());
        data.set("baudrate", axis.baud_rate());
        out.set("identity", data);
        system = dev.system_code;
        if (!dev.categories.empty()) {
            dir = UnitDirectory(dev.index_header, dev.categories);
        } else {
            const UnitDirectory read = UnitDirectory::read(rd);
            dir = UnitDirectory(dev.index_header, read.records());
        }
        out.set("directory", directory_summary(dir));
    });

    if (!dir.empty()) {
        runner.run("support", [&] {
            out.set("support", support_words_json(dir.read_support_words(rd)));
            out.set("accl", dir.read_accl(rd));
        });
        runner.run("parameters", [&] {
            Json class_list = Json::array();
            for (int ci : parameter_classes()) {
                ClassBlock block;
                try {
                    block = dir.read_class(rd, ci);
                } catch (const std::out_of_range&) {
                    continue;  // not in this unit's directory
                }
                classes[ci] = block;
                Json items = Json::array();
                for (const ClassItem& it : block.items) {
                    Json i = Json::object();
                    i.set("key", it.key());
                    i.set("id", it.item_id);
                    i.set("entry", it.entry);
                    i.set("address", util::format("0x%04X", it.address));
                    i.set("words", it.size);
                    i.set("hex", it.hex());
                    items.push_back(i);
                }
                Json c = Json::object();
                c.set("class_id", ci);
                c.set("class", util::format("0x%04X", ci));
                c.set("address", util::format("0x%04X", block.address));
                c.set("words", block.words);
                c.set("items", items);
                class_list.push_back(c);
            }
            Json named = Json::array();
            if (system && !reg::is_ec_family(system)) {
                for (const ConParam& p : con_parameters(reg::product_family(system))) {
                    const auto block = classes.find(p.class_id);
                    const ClassItem* item = block == classes.end() ? nullptr : block->second.item(p.item_id, p.entry);
                    if (item == nullptr || item->size != 2) continue;
                    const ParamValue value = con_detail::decode(p, item->raw);
                    Json row = Json::object();
                    row.set("number", p.no);
                    row.set("name", p.name);
                    row.set("item", p.key());
                    row.set("value", param_value_json(value));
                    row.set("text", con_detail::text(p, value));
                    row.set("raw", item->hex());
                    named.push_back(row);
                }
            } else {
                for (const ParamDef& p : ec_user_parameters()) {
                    const Json d = decode_user_parameter(p, classes);
                    Json row = Json::object();
                    for (const char* k : {"no", "name", "key", "value", "text", "raw_hex", "found"}) row.set(k, *d.find(k));
                    named.push_back(row);
                }
            }
            Json params = Json::object();
            params.set("classes", class_list);
            params.set("named", named);
            out.set("parameters", params);
        });
        runner.run("positions", [&] { out.set("positions", backup_rows_json(axis, rd, dir, rows)); });
        runner.run("alarms", [&] {
            const Words words = dir.read_block(rd, reg::kAlarmHistoryCategory);
            const std::optional<bool> rtc = rtc_in_use(read_ssse(axis, MonitorWords()));
            Json history = Json::array();
            for (const AlarmRecord& r : alarm_history_slots(words)) {
                Json h = Json::object();
                h.set("index", r.index);
                h.set("code", r.code);
                h.set("hex", util::format("0x%03X", r.code));
                h.set("name", alarm_name(r.code));
                h.set("detail", r.detail);
                h.set("address", r.address);
                const Json when = alarm_time(r.time_ms, rtc);
                for (const auto& kv : when.as_object()) h.set(kv.first, kv.second);
                history.push_back(h);
            }
            Json alarms = Json::object();
            alarms.set("words", hex_word_list(words));
            alarms.set("rtc_in_use", rtc);
            alarms.set("history", history);
            out.set("alarms", alarms);
        });
        runner.run("maintenance", [&] {
            const ClassBlock block = dir.read_class(rd, reg::kMaintenanceCategory);
            std::optional<bool> rtc;
            const Json* alarms = out.find("alarms");
            if (alarms && alarms->type() == Json::Type::Object) {
                const Json* r = alarms->find("rtc_in_use");
                if (r && r->type() == Json::Type::Bool) rtc = r->as_bool();
            }
            out.set("maintenance", maintenance_json(block, rtc, reg::product_family(system)));
        });
    } else if (!runner.lost()) {
        errors.push_back("directory: not readable; only the identity was saved");
    }
    meta.set("duration_s", round_to(now_s() - started, 3));
    if (!path.empty()) meta.set("path", path);
    out.set("meta", meta);
    out.set("errors", errors);
    if (!path.empty()) write_text(path, out.dump(2));
    return out;
}

std::vector<std::pair<std::string, std::string>> backup_axes(const std::vector<std::pair<std::string, Axis*>>& axes,
                                                             const std::string& directory, const BackupRows& rows) {
    const std::filesystem::path folder(directory);
    std::filesystem::create_directories(folder);
    std::vector<std::pair<std::string, std::string>> result;
    for (const auto& entry : axes) {
        std::string safe;
        for (char c : entry.first) {
            const bool keep = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
                              c == '_' || c == '.';
            safe += keep ? c : '_';
        }
        if (safe.empty()) safe = "axis";
        const std::string target = (folder / (safe + ".json")).string();
        try {
            if (entry.second == nullptr) throw std::invalid_argument("no axis");
            backup(*entry.second, rows, target);
            result.emplace_back(entry.first, target);
        } catch (const WaitCancelledError&) {
            throw;  // Ctrl+C ends the backup of every axis, not the one it reached
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError& e) {
            result.emplace_back(entry.first, "error: " + error_name(e) + ": " + e.what());
        } catch (const std::ios_base::failure& e) {
            result.emplace_back(entry.first, "error: " + error_name(e) + ": " + e.what());
        } catch (const std::filesystem::filesystem_error& e) {
            result.emplace_back(entry.first, "error: " + error_name(e) + ": " + e.what());
        }
    }
    return result;
}

Json load_backup(const std::string& path) {
    std::ifstream file(std::filesystem::path(path), std::ios::binary);
    if (!file) throw std::ios_base::failure("could not open '" + path + "'");
    std::ostringstream text;
    text << file.rdbuf();
    std::string s = text.str();
    if (s.size() >= 3 && static_cast<unsigned char>(s[0]) == 0xEF && static_cast<unsigned char>(s[1]) == 0xBB &&
        static_cast<unsigned char>(s[2]) == 0xBF)
        s.erase(0, 3);  // a UTF-8 byte order mark
    return Json::parse(s);
}

BackupIdentity backup_identity(const Json& backup) {
    static const Json kEmpty = Json::object();
    const Json* found = json_object(backup, "identity");
    const Json& ident = found ? *found : kEmpty;
    BackupIdentity out;
    out.serial_number = json_string(ident, "serial_number");
    out.model_name = json_string(ident, "model_name");
    out.system_code = json_int_field(ident, "system_code");
    out.unit_code = json_int_field(ident, "unit_code");
    bool any = out.serial_number || out.model_name || out.system_code || out.unit_code;
    for (const char* key : {"serial_number", "model_name", "system_code", "unit_code"}) {
        const Json* v = ident.find(key);
        if (v != nullptr && !v->is_null()) any = true;
    }
    if (!any) throw std::invalid_argument("the backup holds no identity (serial number, model, system / unit code)");
    out.firmware = json_string(ident, "firmware");
    out.product_family = json_string(ident, "product_family");
    const Json* limits = json_array(ident, "soft_limits_mm");
    if (limits && limits->size() == 2) {
        const auto& v = limits->as_array();
        for (const Json& end : v)  // a wrong type is a bad argument, not std::logic_error ("not in this build")
            if (end.type() != Json::Type::Int && end.type() != Json::Type::Number)
                throw std::invalid_argument("the backup's identity soft_limits_mm must be [min, max], two numbers");
        out.soft_limits_mm = Span{v[0].as_number(), v[1].as_number()};
    }
    out.max_speed_mm_s = json_number(ident, "max_speed_mm_s");
    const std::optional<std::string> units = json_string(ident, "units");
    if (units && !units->empty()) out.units = *units;
    return out;
}

std::map<int, PositionEntry> backup_rows(const Json& backup) {
    std::map<int, PositionEntry> out;
    const Json* positions = json_object(backup, "positions");
    const Json* rows = positions ? json_array(*positions, "rows") : nullptr;
    if (rows == nullptr) return out;
    for (const Json& row : rows->as_array()) {
        if (row.type() != Json::Type::Object) continue;
        const Json* list = json_array(row, "words");
        const std::optional<int> n = json_int_field(row, "n");
        if (list == nullptr || !n) continue;
        Words words;
        for (const Json& w : list->as_array()) {
            if (w.type() != Json::Type::String)  // not std::logic_error ("not in this build")
                throw std::invalid_argument("backup row " + std::to_string(*n) + ": the words are hex text");
            const std::optional<long long> value = util::parse_int(w.as_string(), 16);
            if (!value) throw std::invalid_argument("backup row " + std::to_string(*n) + ": '" + w.as_string() + "' is not hex");
            words.push_back(static_cast<std::uint16_t>(*value));
        }
        if (words.size() >= static_cast<std::size_t>(reg::kPositionWordsWritable))
            out[*n] = PositionEntry::from_registers(slice(words, 0, reg::kPositionStride));
    }
    return out;
}

std::map<std::string, std::string> backup_parameters(const Json& backup) {
    std::map<std::string, std::string> out;
    const Json* parameters = json_object(backup, "parameters");
    const Json* classes = parameters ? json_array(*parameters, "classes") : nullptr;
    if (classes == nullptr) return out;
    for (const Json& block : classes->as_array()) {
        if (block.type() != Json::Type::Object) continue;
        const int class_id = json_int_field(block, "class_id").value_or(-1);
        if (std::find(parameter_classes().begin(), parameter_classes().end(), class_id) == parameter_classes().end())
            continue;
        const Json* items = json_array(block, "items");
        if (items == nullptr) continue;
        for (const Json& item : items->as_array()) {
            const std::optional<std::string> key = json_string(item, "key");
            const std::optional<std::string> hex = json_string(item, "hex");
            if (key && hex) out[util::format("0x%04X:", class_id) + *key] = util::upper(*hex);
        }
    }
    return out;
}

// ============================================================================================ public: helpers

std::string error_name(const std::exception& e) {
#define IAD_NAME(T, N) \
    if (dynamic_cast<const T*>(&e)) return N
    IAD_NAME(ModbusTimeout, "ModbusTimeout");
    IAD_NAME(ModbusCrcError, "ModbusCRCError");
    IAD_NAME(ModbusFrameError, "ModbusFrameError");
    IAD_NAME(ModbusSlaveError, "ModbusException");
    IAD_NAME(IaiCommandRejected, "IaiCommandRejected");
    IAD_NAME(TransportError, "TransportError");
    IAD_NAME(ModbusError, "ModbusError");
    IAD_NAME(UnsafeCommandError, "UnsafeCommandError");
    IAD_NAME(SoftLimitError, "SoftLimitError");
    IAD_NAME(ServoBlockedError, "ServoBlockedError");
    IAD_NAME(EcNotReadyError, "EcNotReadyError");
    IAD_NAME(EcAlarmError, "EcAlarmError");
    IAD_NAME(MotionTimeoutError, "MotionTimeoutError");
    IAD_NAME(MotionIncompleteError, "MotionIncompleteError");
    IAD_NAME(NonVolatileWriteError, "NonVolatileWriteError");
    IAD_NAME(VerificationError, "VerificationError");
    IAD_NAME(EcError, "EcError");
    IAD_NAME(EcDriverError, "EcDriverError");
    IAD_NAME(std::ios_base::failure, "OSError");
    IAD_NAME(std::filesystem::filesystem_error, "OSError");
    IAD_NAME(std::out_of_range, "LookupError");
    IAD_NAME(std::invalid_argument, "ValueError");
#undef IAD_NAME
    return "Exception";
}

std::string local_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    std::tm utc{};
    {
        static std::mutex mutex;  // std::localtime / std::gmtime share a static buffer
        std::lock_guard<std::mutex> lock(mutex);
        if (const std::tm* l = std::localtime(&now)) local = *l;
        if (const std::tm* u = std::gmtime(&now)) utc = *u;
    }
    auto seconds = [](const std::tm& t) {
        return days_from_civil(t.tm_year + 1900LL, static_cast<unsigned>(t.tm_mon + 1), static_cast<unsigned>(t.tm_mday)) *
                   86400LL +
               t.tm_hour * 3600LL + t.tm_min * 60LL + t.tm_sec;
    };
    const long long offset_min = (seconds(local) - seconds(utc)) / 60;
    const long long abs_min = offset_min < 0 ? -offset_min : offset_min;
    return util::format("%04d-%02d-%02dT%02d:%02d:%02d%c%02lld:%02lld", local.tm_year + 1900, local.tm_mon + 1,
                        local.tm_mday, local.tm_hour, local.tm_min, local.tm_sec, offset_min < 0 ? '-' : '+',
                        abs_min / 60, abs_min % 60);
}

// ------------------------------------------------------------------------------------------ (0.5, py-axis) user parameters

Json user_parameter_rows(const ParameterContext& ctx) {
    const ParameterModel model = model_parameter_list(ctx);
    const std::map<int, ParameterVisibility> visibility = ec_detail::visibility(ctx);
    Json rows = Json::array();
    for (int no : model.listed) {
        const ParamDef* p = ec_user_parameter(no);
        if (!p) continue;
        Json row = annotate_parameter(decode_user_parameter(*p, ctx.classes), *p, model, visibility);
        if (!model.known && !row.find("found")->as_bool()) continue;
        row.set("located", true);
        rows.push_back(row);
    }
    return rows;
}

// ------------------------------------------------------------------------------------------ (0.6, py-info-cli) input ranges

namespace {

/// One direct-move / row field of the input ranges (info._MOVE_FIELDS).
struct RangeMoveField {
    const char* field;
    const char* reg;
    const char* raw_range;
    const char* range;
    const char* limit;
    const char* doc;
};

/// One other setting of a product line (field, range, doc).
struct RangeOther {
    const char* field;
    const char* range;
    const char* doc;
};

/// One product line of info.INPUT_RANGES; input_ranges() fills in its parameters.
struct ProductLineRanges {
    const char* name;
    const char* title;
    const char* rule;
    std::vector<RangeOther> other;
};

const RangeMoveField kMoveFields[] = {
    {"Target (PCMD)", "0x9900 s32", "-999,999 .. 999,999", "+/-9,999.99 mm (deg)",
     "the soft limits minus the 0.30 margin; outside is refused (the unit would clamp to 0.2 mm inside its soft limit)",
     "ME0162-12A p.5-131"},
    {"Band (INP)", "0x9902 u32", "1 .. 999,999 (not 0)", "0.01 .. 9,999.99",
     ">= MINI 3.67 when enforced; CON: >= lead / pulses x 3 on low-resolution steppers; push band = push distance, "
     "at most the distance to the stroke end", "ME0162-12A p.5-131, 5-151; ME0342-4H p.6-12"},
    {"Speed (VCMD)", "0x9904 u32", "1 .. 999,999", "0.01 .. 9,999.99 mm/s",
     "<= MAXV (7.4); above the unit's maximum the unit alarms at start; below the documented minimum speed: a warning",
     "ME0162-12A p.5-131"},
    {"Acceleration (ACMD)", "0x9906", "1 .. 300", "0.01 .. 3.00 G",
     "<= MAXA / MAXD (7.5 / 7.6); 0.3 G while the unit's maximum is not known", "ME0162-12A p.5-132"},
    {"Push current (PPOW)", "0x9907", "0x33 .. 0xB2 = 20-70 % (RA13R / RA15R / RA20R: 0x33 .. 0x1FE)",
     "raw = floor(% x 255 / 100)",
     "<= MXPP (7.9); below MNPP (7.58, else 20 %): a warning; push speed = min(VCMD, PSHV, MXPH)",
     "ME0162-12A p.5-132"},
    {"Control flags (CTLF)", "0x9908", "b1 PUSH, b2 DIR (1 = away from home), b3 INC, b4-5 GSL, b6-7 MOD, b12-13 NTC",
     "-", "GSL / NTC only on the ACON / SCON models listed in the manual; DIR = 1 pushes away from home (VERIFIED)",
     "ME0162-12A p.5-134"},
    {"Zones (rows)", "ZNMP / ZNLP s32", "s32", "mm", "any order: Zone+ < Zone- is an inverted zone; equal = off",
     "ME0342-4H p.4-31, 6-7"},
    {"Rows", "-", "No.25: 64 / 64 / 256 / 512 / 7 / 3 for patterns 0-5", "-",
     "from the unit (MAXP 3.2 / the directory)", "ME0342-4H p.4-9, 6-17"},
};

const std::vector<ProductLineRanges>& product_line_ranges() {
    static const std::vector<ProductLineRanges> kLines = {
        {"EC", "ELECYLINDER (own SIO port)",
         "the unit's own limits first (soft limits, MAXV, MAXA / MAXD, MXPP, MXPH, MINI), then the manual range, "
         "then a driver default; a documented minimum is a warning, never a refusal",
         {
             {"Operation noise", "0-14 (Ultra Mini 1-9); needs a reboot", "ME3816-6B p.4-16, 4-17"},
             {"Automatic servo OFF delay",
              "0.1 s steps, 0 = off, one per direction; not used with pushing (the work is released)",
              "ME3816-6B p.4-20, 4-21, 4-52"},
             {"Maintenance targets",
              "travel count 0-999,999,999, distance 0-999,999,999 m, 0 = off; overload warning level 50-100 %, 100 = "
              "off", "ME3816-6B p.5-1 to 5-7"},
             {"Homing speed", "the unit's own (20 mm/s in the manual; about 30 mm/s on an EC-DS4H, VERIFIED)",
              "ME3816-6B p.4-33"},
         }},
        {"CON-CB", "PCON / ACON / DCON / SCON-CB controllers",
         "per-family parameter ranges (con_params.parameter_table(family)); the unit's maxima cap No.8 / 26 / 35 / "
         "47 (MAXV), No.9 (MAXA), No.34 (MXPH); No.10 >= the minimum positioning band",
         {
             {"No.16 SIO baud",
              "9,600 / 14,400 / 19,200 / 28,800 / 38,400 / 76,800 / 115,200 / 230,400; default 38,400",
              "ME0342-4H p.6-14; ME0343-3B p.6-14; ME0340-9F p.6-16"},
             {"No.29 excitation detection time", "1-999 ms (ACON 50-999); drives the first servo-on time",
              "ME0342-4H p.6-20; ME0343-3B p.6-3"},
             {"No.30 excitation detection type", "0 / 1 / 2; 2 on a vertical axis may drop the load",
              "ME0342-4H p.6-20"},
             {"No.42 enable function", "default 1 on p.6-3 but 0 on p.6-25 (manual inconsistency)",
              "ME0342-4H p.6-3, 6-25"},
             {"No.168 collision detection", "0-7 (PCON only)", "ME0342-4H p.6-53, 5-3"},
             {"Rotary index mode", "No.79 = 1: 0-359.99 deg, soft limits off, no push", "ME0342-4H p.4-11, 6-35"},
             {"Parameter commit",
              "non-volatile write + software reset or power cycle; no write limit on CB units (FeRAM)",
              "ME0342-4H p.6-1, 2-11"},
         }},
        {"RCON", "RCON driver units behind an RCON-GW",
         "only the documented path: FC 03 / 06 / 10 / 17 (+ 01 / 04), no broadcast, MANU mode required",
         {
             {"Line", "RTU only, 115,200 bps fixed (a PC tool may leave its own rate)",
              "ME0413-2B p.2-1, 6-1; ME0384-9B p.2-4"},
             {"Addresses", "gateway D0h (D1h) or FFh; axes 01h-10h; at most 16 axes incl. EC axes",
              "ME0413-2B p.4-1; ME0384-9B p.2-3, 3-117"},
             {"Function codes", "01, 03, 04, 06, 10, 17 only; no broadcast", "ME0413-2B p.3-1, 6-1"},
             {"Position unit", "No.207: 0.01 or 0.001 mm (target, band, zones, PNOW)", "ME0384-9B p.8-56"},
             {"Push", "255 = 100 %, 510 = 200 % (fieldbus)", "ME0384-9B p.6-32; ME0413-2B p.5-12"},
             {"No.35 safety speed", "1-250 mm/s, default 100", "ME0384-9B p.8-2"},
             {"Completed position (POSS)", "7 bits (row <= 127)", "ME0413-2B p.5-10 to 5-23"},
         }},
        {"ROBONET", "ROBONET (RGW-SIO gateway)",
         "SIO through mode (SW2 ON) for direct access; gateway mode is not supported",
         {
             {"Baud", "through mode 38,400-230,400; gateway mode 9,600-230,400", "ME0208-14B p.26"},
             {"Slave / function codes", "through: axis + 1, FC 01-07, 0F, 10, 11, 17; gateway: 3Fh, FC 03, 06, 10",
              "ME0208-14B p.26, 119-123"},
             {"Buffer", "160 bytes (at most 75 registers per FC10)", "ME0208-14B p.26"},
             {"Gateway-mode units",
              "speed 1 mm/s (or 0.1), accel = decel <= 2 G (0.01 G), push 0-FFh, 32-bit low word first",
              "ME0208-14B p.71-75, 131"},
             {"Internal link", "230.4 kbps fixed; external-SIO SCON / PCON-CF need No.16 = 230,400 and No.17 = 2 ms",
              "ME0208-14B p.340, 348"},
         }},
    };
    return kLines;
}

/// A number with its Python type: an int unless the table has a decimal (con_params.parameter_table's min / max).
Json py_number_json(const std::optional<double>& value, bool decimal) {
    return value ? Json::whole_or_number(*value, decimal) : Json();
}

}  // namespace

const std::vector<std::string>& product_lines() {
    static const std::vector<std::string> kNames = [] {
        std::vector<std::string> names;
        for (const ProductLineRanges& line : product_line_ranges()) names.emplace_back(line.name);
        return names;
    }();
    return kNames;
}

std::string product_line(const std::optional<std::string>& family, const std::optional<std::string>& gateway) {
    if (gateway && !gateway->empty()) {
        const std::string gw = util::lower(*gateway);
        if (util::starts_with(gw, "rcon")) return "RCON";
        if (util::starts_with(gw, "robonet")) return "ROBONET";
    }
    return util::upper(family.value_or("")) == "EC" ? "EC" : "CON-CB";
}

Json ec_parameter_table() {
    Json rows = Json::array();
    for (const ParamDef& p : ec_user_parameters()) {
        const ManualView* manual = manual_view(p.no);
        const std::string category = manual ? manual->category : "";
        Json driver_range;  // the narrower limit the driver enforces (P1: up to the largest P1 read from the unit)
        if (p.no == 1 || p.no == 2 || p.no == 4) {
            driver_range = Json::object();
            driver_range.set("min", p.no == 1 ? kP1MinMm : p.no == 2 ? kP2MinMm : kP4MinMm);
            driver_range.set("max", p.no == 1 ? Json() : Json(p.no == 2 ? kP2MaxMm : kP4MaxMm));
        }
        Json choices = Json::object();
        if (p.kind != "mm")
            for (const auto& c : p.choices) choices.set(std::to_string(c.first), c.second);
        Json row = Json::object();
        row.set("no", p.no);
        row.set("name", p.name);
        row.set("manual_name", manual ? manual->manual_name : "");
        row.set("category", category);
        row.set("category_text", ec_category_text(category));
        row.set("unit", p.unit);
        row.set("manual_range", manual ? manual->manual_range : "");
        row.set("shipping_default", manual ? manual->shipping_default : "");
        row.set("driver_range", driver_range);
        row.set("choices", choices);
        row.set("read_only", p.read_only);
        row.set("home_parameter", is_ec_home_parameter(p.no));
        row.set("iaos_only", manual == nullptr);
        row.set("doc", manual ? manual->doc : "not in ME3816-6B (the PC tool lists it, the manual does not)");
        rows.push_back(row);
    }
    // No.143 (overload warning level): the manual names it, the driver does not map its register
    Json row = Json::object();
    row.set("no", 143);
    row.set("name", "Overload warning level");
    row.set("manual_name", "Overload Warning Load Level Ratio");
    row.set("category", "");
    row.set("category_text", "");
    row.set("unit", "%");
    row.set("manual_range", "50-99 %, 100 = off");
    row.set("shipping_default", "100");
    row.set("driver_range", Json());
    row.set("choices", Json::object());
    row.set("read_only", true);
    row.set("home_parameter", false);
    row.set("iaos_only", false);
    row.set("doc", "ME3816-6B p.5-3 (maintenance information; register not mapped by the driver)");
    rows.push_back(row);
    return rows;
}

Json con_parameter_table(const std::optional<std::string>& family) {
    const std::optional<std::string> fam = family ? con_detail::family_of(*family) : std::nullopt;
    Json rows = Json::array();
    for (const ConParam& p : con_parameters(fam.value_or(""))) {
        const std::string cat = con_detail::category(p.no, fam);
        Json lo = py_number_json(p.lo, p.lo_decimal);
        Json hi = py_number_json(p.hi, p.hi_decimal);
        if (p.kind == "baud") {
            const auto range = con_detail::baud_range();
            lo = range.first;
            hi = range.second;
        }
        Json choices = Json::object();
        for (const auto& kv : p.choices) choices.set(std::to_string(kv.first), kv.second);
        const std::optional<con_detail::Number> preset = con_detail::factory_default(p.no, fam);
        Json row = Json::object();
        row.set("number", p.no);
        row.set("name", p.name);
        row.set("symbol", p.symbol);
        row.set("item", p.key());
        row.set("unit", p.unit);
        row.set("min", lo);
        row.set("max", hi);
        row.set("choices", choices);
        row.set("category", cat);
        row.set("category_text", con_detail::category_text(cat));
        row.set("factory_default", preset ? py_number_json(preset->value, preset->decimal) : Json());
        row.set("dangerous", p.dangerous);
        row.set("writable", p.writable);
        row.set("note", con_detail::note_of(p.no));
        row.set("range_doc", con_detail::range_doc(fam));
        row.set("verified_family", fam.has_value());  // the driver has a range table for this family (false: PCON-CB)
        rows.push_back(row);
    }
    return rows;
}

Json input_ranges(const std::optional<std::string>& line, const std::optional<std::string>& family) {
    const std::vector<std::string>& names = product_lines();
    if (line && std::find(names.begin(), names.end(), *line) == names.end()) {
        std::vector<std::string> quoted;
        for (const std::string& n : names) quoted.push_back("'" + n + "'");
        throw std::invalid_argument("unknown product line '" + *line + "'; choose from [" + util::join(quoted, ", ") +
                                    "]");
    }
    Json moves = Json::array();
    for (const RangeMoveField& f : kMoveFields) {
        Json m = Json::object();
        m.set("field", f.field);
        m.set("register", f.reg);
        m.set("raw_range", f.raw_range);
        m.set("range", f.range);
        m.set("limit", f.limit);
        m.set("doc", f.doc);
        moves.push_back(m);
    }
    Json out = Json::object();
    for (const ProductLineRanges& ranges : product_line_ranges()) {
        const std::string name = ranges.name;
        if (line && name != *line) continue;
        Json other = Json::array();
        for (const RangeOther& o : ranges.other) {
            Json r = Json::object();
            r.set("field", o.field);
            r.set("range", o.range);
            r.set("doc", o.doc);
            other.push_back(r);
        }
        Json entry = Json::object();
        entry.set("title", ranges.title);
        entry.set("rule", ranges.rule);
        entry.set("moves", moves);
        entry.set("parameters", name == "EC"       ? ec_parameter_table()
                                : name == "CON-CB" ? con_parameter_table(family)
                                                   : Json());
        entry.set("other", other);
        if (name == "CON-CB") entry.set("family", family ? con_detail::family_of(*family) : std::nullopt);
        if (line) return entry;
        out.set(name, entry);
    }
    return out;
}

}  // namespace info
}  // namespace iai_actuator_driver
