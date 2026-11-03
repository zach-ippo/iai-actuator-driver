// Controller alarm names, levels, groups and the alarm history (ALMC 0x9002, history 0x0500). Port of the Python
// edition's alarms.py (C# Alarms.cs). The tables are in alarm_tables.cpp. No I/O.
//
// The CON-only message codes 0x02C-0x02E are message level when a CON system code or family is given, and alarm_info
// words the per-family clearing rule of 0x0CE / 0x0D2 / 0x0EE (as in the Python edition).
#include "iai_actuator_driver/alarms.hpp"

#include <cctype>
#include <cstdint>
#include <stdexcept>

#include "iai_actuator_driver/registers.hpp"
#include "model_detail.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr std::uint32_t kSuptAbse = 1u << 13;  // SUPT b13 ABSE: absolute encoder
constexpr std::uint32_t kSup2Mabs = 1u << 18;  // SUP2 b18 MABS: battery-less absolute

const char* const kLevelText0eeUnknown = "operation cancelled on absolute units / cold start on incremental units";

bool in(int code, int first, int last) { return code >= first && code <= last; }

/// The named codes of 0x011-0x07F (message level: a warning, the axis keeps running).
bool is_message_code(int code) {
    return code == 0x040 || code == 0x041 || in(code, 0x047, 0x04A) || in(code, 0x04C, 0x04F) ||
           in(code, 0x05A, 0x05E) || in(code, 0x061, 0x065) || in(code, 0x068, 0x06B) || in(code, 0x070, 0x07C) ||
           code == 0x07F;
}

/// The 0x080-0x0FF codes that only cancel the operation (0x0EE is handled apart; 0x0CE / 0x0D2 are cold start).
bool is_operation_cancelled_code(int code) {
    return code == 0x080 || in(code, 0x082, 0x08A) || in(code, 0x090, 0x096) || in(code, 0x0A2, 0x0A5) ||
           code == 0x0A7 || in(code, 0x0AB, 0x0AE) || code == 0x0B5 || code == 0x0B6 || code == 0x0BA ||
           in(code, 0x0BE, 0x0C2) || code == 0x0C5 || code == 0x0CF || code == 0x0D6 || in(code, 0x0D8, 0x0DC) ||
           code == 0x0DE || code == 0x0DF || code == 0x0E1 || code == 0x0ED || code == 0x0EF || code == 0x0F1;
}

std::string capitalize(const std::string& text) {
    if (text.empty()) return text;
    std::string out = text;
    out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

}  // namespace

// ------------------------------------------------------------------------------------------ levels

const char* level_key(AlarmLevel level) {
    switch (level) {
        case AlarmLevel::None: return "none";
        case AlarmLevel::Message: return "message";
        case AlarmLevel::OperationCancelled: return "operation_cancelled";
        case AlarmLevel::ColdStart: return "cold_start";
        case AlarmLevel::Unknown: break;
    }
    return "unknown";
}

std::string level_text(AlarmLevel level) {
    switch (level) {
        case AlarmLevel::None: return "no alarm";
        case AlarmLevel::Message: return "message (warning: the axis keeps running)";
        case AlarmLevel::OperationCancelled: return "operation cancelled (stop, servo off; alarm reset)";
        case AlarmLevel::ColdStart: return "cold start (stop, servo off; software reset or power cycle)";
        case AlarmLevel::Unknown: break;
    }
    return "unknown level (treated as a fault unless the EC sets the warning bit ALML)";
}

std::string alarm_name(int code) {
    if (const char* text = alarm_data::curated_name(code)) return text;
    if (const char* text = alarm_data::controller_name(code)) return text;
    if (const char* text = alarm_data::con_name(code)) return text;
    return util::format("alarm 0x%03x", code);
}

bool is_con_system_code(std::optional<int> system_code) {
    return system_code && (*system_code == 0xAE || *system_code == 0xAC || *system_code == 0xAA);
}

std::optional<std::string> alarm_family(std::optional<int> system_code, const std::optional<std::string>& family) {
    if (family && !family->empty()) {
        const std::string name = util::upper(util::trim(*family));
        if (name == "ACON" || name == "DCON" || name == "ACON/DCON") return std::string("ACON/DCON");
        if (name == "PCON" || name == "SCON") return name;
        return std::nullopt;
    }
    if (is_con_system_code(system_code)) return reg::product_family(*system_code);
    return std::nullopt;
}

std::optional<int> con_system_code(const std::optional<std::string>& con_family) {
    if (!con_family) return std::nullopt;
    if (*con_family == "PCON") return 0xAE;
    if (*con_family == "ACON/DCON") return 0xAC;
    if (*con_family == "SCON") return 0xAA;
    return std::nullopt;
}

AlarmLevel alarm_level(int code, std::optional<bool> absolute_encoder, std::optional<int> system_code) {
    if (code == 0 || code == reg::kAlarmPowerUp) return AlarmLevel::None;
    if (in(code, 0x001, 0x010)) return AlarmLevel::ColdStart;  // Modbus exception / reject range (not raised on an EC)
    if (in(code, 0x011, 0x07F)) {
        if (is_message_code(code)) return AlarmLevel::Message;
        if (in(code, 0x02C, 0x02E) && is_con_system_code(system_code)) return AlarmLevel::Message;  // CON-only, DOC
        return AlarmLevel::Unknown;
    }
    if (in(code, 0x080, 0x0FF)) {
        if (code == 0x0EE) return absolute_encoder.has_value() && !*absolute_encoder ? AlarmLevel::ColdStart : AlarmLevel::OperationCancelled;
        return is_operation_cancelled_code(code) ? AlarmLevel::OperationCancelled : AlarmLevel::ColdStart;
    }
    return AlarmLevel::Unknown;
}

bool alarm_is_fault(int code, bool alarm_bit, bool warning_bit, std::optional<int> system_code) {
    if (alarm_bit) return true;
    if (code == 0) return false;
    const AlarmLevel level = alarm_level(code, std::nullopt, system_code);
    if (level == AlarmLevel::Message || level == AlarmLevel::None) return false;
    if (level == AlarmLevel::OperationCancelled || level == AlarmLevel::ColdStart) return true;
    return !warning_bit;  // unknown level: a fault unless ALML says it is a warning (fail safe)
}

std::string alarm_group(int code) {
    const char* letter = alarm_data::group_letter(code);
    return letter ? letter : "Other";
}

std::optional<bool> absolute_encoder_from_support(const std::map<std::string, std::uint32_t>* support) {
    if (support == nullptr || support->empty()) return std::nullopt;
    const auto supt = support->find("SUPT");
    const auto sup2 = support->find("SUP2");
    if ((supt != support->end() && (supt->second & kSuptAbse) != 0) ||
        (sup2 != support->end() && (sup2->second & kSup2Mabs) != 0))
        return true;
    if (supt != support->end()) return false;
    return std::nullopt;
}

// ------------------------------------------------------------------------------------------ history

std::optional<std::string> AlarmRecord::time_after_power_on() const {
    if (!time_ms) return std::nullopt;
    const std::int64_t total = *time_ms;
    const std::int64_t ms = total % 1000;
    const std::int64_t seconds = total / 1000;
    const std::int64_t days = seconds / 86400;
    const std::int64_t rest = seconds % 86400;
    const std::string clock =
        util::format("%02lld:%02lld:%02lld.%03lld", static_cast<long long>(rest / 3600),
                     static_cast<long long>((rest % 3600) / 60), static_cast<long long>(rest % 60),
                     static_cast<long long>(ms));
    if (days > 0) return std::to_string(static_cast<long long>(days)) + " d " + clock;
    return clock;
}

std::vector<AlarmRecord> alarm_history_slots(const std::vector<std::uint16_t>& words, bool include_empty) {
    const std::size_t stride = static_cast<std::size_t>(reg::kAlarmHistoryStride);
    std::vector<AlarmRecord> slots;
    for (std::size_t k = 0; k < words.size() / stride; ++k) {
        const std::size_t o = k * stride;
        const int code = words[o + 3];
        if (code == 0 && !include_empty) break;  // the list ends at the first empty slot
        AlarmRecord record;
        record.index = static_cast<int>(k);
        record.code = code;
        record.detail = words[o];
        record.address = words[o + 1];
        const std::uint32_t stamp = reg::from_u32_words(words[o + 4], words[o + 5]);
        if (stamp != 0xFFFFFFFFu) record.time_ms = static_cast<std::int64_t>(stamp);
        slots.push_back(record);
    }
    return slots;
}

// ------------------------------------------------------------------------------------------ alarm_info

std::string describe_alarm_detail(int code, std::optional<int> detail, std::optional<int> address) {
    if (code == 0x0EE && detail) {
        if (const char* text = alarm_data::abs_error_detail(*detail))
            return util::format("detail %04X: %s (ME0342-4H p.8-21)", *detail, text);
    }
    if (code == 0x0A2 && address && *address >= 0x1000 && *address < 0x4000) {
        const int row = (*address - 0x1000) / 16;
        const int field = (*address - 0x1000) % 16;
        const char* name = alarm_data::row_field(field);
        return util::format("row %d, field %s", detail ? *detail : row,
                            name ? name : util::format("+0x%X", field).c_str());
    }
    if (code == 0x0A3 && address) {
        if (const char* name = alarm_data::command_field(*address))
            return util::format("direct-move field %s (0x%04X)", name, *address);
    }
    if (code == 0x0A1 && address && *address != 0xFFFF) return util::format("parameter at 0x%04X", *address);
    return std::string();
}

AlarmInfo alarm_info(int code, std::optional<bool> absolute_encoder, std::optional<int> detail,
                     std::optional<int> address, std::optional<int> system_code,
                     const std::optional<std::string>& family) {
    AlarmInfo info;
    const std::optional<std::string> con = alarm_family(system_code, family);
    const AlarmLevel level = alarm_level(code, absolute_encoder, con_system_code(con));
    const std::string group = alarm_group(code);
    std::string text = level_text(level);
    std::string reset;
    std::string remedy;
    if (level == AlarmLevel::None) {
        // nothing to clear
    } else if (level == AlarmLevel::Message) {
        const char* clearing = con ? nullptr : alarm_data::warning_clearing(code);  // the EC's maintenance warnings 1-4
        reset = clearing ? clearing : "alarm reset (the axis keeps running meanwhile)";
        remedy = "warning only: the axis keeps running; alarm reset once the cause is dealt with";
    } else if (code == 0x0EE && !absolute_encoder.has_value()) {
        text = kLevelText0eeUnknown;
        reset = "absolute units: alarm reset, then servo on; incremental units: software reset (servo off first) or "
                "24 V power cycle";
        remedy = "the EC stopped and switched the servo off: alarm reset; if it stays, software reset or power cycle";
    } else if (level == AlarmLevel::OperationCancelled) {
        reset = "alarm reset, then servo on";
        remedy = "the EC stopped and switched the servo off: remove the cause, alarm reset, servo on";
    } else if (level == AlarmLevel::ColdStart) {
        reset = "software reset (servo off first) or 24 V power cycle; an incremental unit then needs a home return";
        remedy = "an alarm reset will probably not clear it: remove the cause, then software reset or power cycle";
    } else {
        reset = "alarm reset; if it stays, software reset (servo off first) or power cycle";
        remedy = "unknown alarm level: treated as a fault unless the EC sets the warning bit ALML";
    }
    std::string advice;
    if (const char* specific = alarm_data::specific_remedy(code)) {
        advice = specific;
    } else if (const char* general = alarm_data::group_advice(group)) {
        advice = general;
    }
    if (!advice.empty() && level != AlarmLevel::None) remedy += ". " + capitalize(advice);
    // 0x0CE / 0x0D2 / 0x0EE: what clears them differs per CON family; both rules are named when it is not known
    if ((code == 0x0CE || code == 0x0D2 || code == 0x0EE) && level != AlarmLevel::None) {
        const char* clearing = con ? alarm_data::family_clearing(code, *con) : alarm_data::family_clearing_any();
        if (clearing) remedy += ". " + capitalize(clearing);
    }

    std::string home;
    if (code == 0x0EE) {
        if (!absolute_encoder.has_value())
            home = "home lost: a home return after the reset (absolute units: alarm reset; incremental units: "
                   "software reset or power cycle)";
        else if (*absolute_encoder)
            home = "home lost: alarm reset, then a home return";
        else
            home = "home lost: software reset (servo off first) or power cycle, then a home return";
    } else if (const char* needed = alarm_data::home_needed(code)) {
        home = needed;
    }
    if (home.empty() && level == AlarmLevel::ColdStart) home = "incremental units: home return after the reset";

    info.code = code;
    info.hex = util::format("0x%03X", code);
    info.name = alarm_name(code);
    info.level = level;
    info.level_text = text;
    info.group = group;
    const char* group_text = alarm_data::group_name(group);
    info.group_name = group_text ? group_text : group;
    info.fault = level == AlarmLevel::OperationCancelled || level == AlarmLevel::ColdStart ||
                 level == AlarmLevel::Unknown;
    info.warning = level == AlarmLevel::Message;
    info.reset = reset;
    info.home_needed = home;
    info.remedy = remedy;
    info.detail_text = describe_alarm_detail(code, detail, address);
    info.family = con;
    return info;
}

// ------------------------------------------------------------------------------------------ tables

std::vector<std::pair<int, std::string>> alarm_table(const std::string& name) {
    bool known = false;
    const std::vector<std::pair<int, const char*>> entries = alarm_data::table_entries(name, &known);
    if (!known) throw std::invalid_argument("unknown alarm table '" + name + "'");
    std::vector<std::pair<int, std::string>> out;
    for (const auto& e : entries) out.emplace_back(e.first, e.second);
    return out;
}

std::vector<std::pair<std::string, std::string>> alarm_group_table(const std::string& name) {
    static const char* const kKeys[] = {"A", "B", "C", "D", "E", "F", "Warning", "Other"};
    const bool names = name == "group_names";
    if (!names && name != "group_advice") throw std::invalid_argument("unknown alarm group table '" + name + "'");
    std::vector<std::pair<std::string, std::string>> out;
    for (const char* key : kKeys) {
        const char* text = names ? alarm_data::group_name(key) : alarm_data::group_advice(key);
        if (text) out.emplace_back(key, text);
    }
    return out;
}

std::vector<std::tuple<int, std::string, std::string>> alarm_family_clearing_table() {
    static const char* const kFamilies[] = {"PCON", "ACON/DCON", "SCON"};
    std::vector<std::tuple<int, std::string, std::string>> out;
    for (int code : {0x0CE, 0x0D2, 0x0EE})
        for (const char* family : kFamilies)
            if (const char* text = alarm_data::family_clearing(code, family)) out.emplace_back(code, family, text);
    return out;
}

std::string alarm_family_clearing_any() { return alarm_data::family_clearing_any(); }

}  // namespace iai_actuator_driver
