// Controller alarm names, levels, groups and the alarm history (ALMC 0x9002, history 0x0500). Port of alarms.py
// (C# Alarms.cs + Alarms.Tables.cs); implemented by the model unit (src/alarms.cpp, src/alarm_tables.cpp).
//
// Levels (UNVERIFIED unless marked): message = a warning, the axis keeps running (the named codes of 0x011-0x07F);
// operation cancelled = stop + servo off, an alarm reset clears it (VERIFIED for 0x090); cold start = every other code
// of 0x080-0x0FF (and 0x001-0x010): only a software reset or a power cycle clears it; 0x0EE "ABS error (2)" is
// operation cancelled on an absolute unit (VERIFIED on the WA gripper) and cold start on an incremental one; unknown =
// any other code: a fault unless the warning bit DSS1 b9 ALML is set (fail safe). CON-family units (FC11 system code
// 0xAE / 0xAC / 0xAA, is_con_system_code): 0x02C-0x02E are message level there (DOC ME0342-4H p.8-14, ME0343-3B p.8-14,
// ME0340-9F p.8-11); without a system code (an EC, the default) they keep the unknown level.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace iai_actuator_driver {

/// Level of a controller alarm.
enum class AlarmLevel {
    None,                ///< 0x000 or the 0x0FFF power-up marker
    Message,             ///< a warning: the axis keeps running
    OperationCancelled,  ///< stop + servo off; an alarm reset clears it
    ColdStart,           ///< stop + servo off; only a software reset or a power cycle clears it
    Unknown,             ///< no known level: a fault unless ALML is set
};

/// "none", "message", "operation_cancelled", "cold_start", "unknown" (the JSON key).
const char* level_key(AlarmLevel level);
/// The level in words, e.g. "operation cancelled (stop, servo off; alarm reset)".
std::string level_text(AlarmLevel level);

/// Name of an alarm code: the driver's curated name, else IAI's controller alarm name, else "alarm 0x0nn".
std::string alarm_name(int code);

/// FC11 system codes of the CON-family controllers (PCON 0xAE, ACON / DCON 0xAC, SCON 0xAA).
bool is_con_system_code(std::optional<int> system_code);

/// The CON family an alarm is read for: "PCON", "ACON/DCON" or "SCON", else nullopt (an EC, or a unit whose family is
/// not known). `family` (a product-family name such as "PCON", "ACON", "DCON"; case-insensitive) wins over
/// `system_code` (FC11): a family name that is not a CON family gives nullopt (alarms.alarm_family).
std::optional<std::string> alarm_family(std::optional<int> system_code,
                                        const std::optional<std::string>& family = std::nullopt);

/// The FC11 system code of a CON family name ("PCON" 0xAE, "ACON/DCON" 0xAC, "SCON" 0xAA), nullopt for anything
/// else: what alarm_level() and alarm_is_fault() take for a family read by alarm_family().
std::optional<int> con_system_code(const std::optional<std::string>& con_family);

/// Level of an alarm code. `absolute_encoder` (from the support words) decides 0x0EE; nullopt = unknown.
/// `system_code` (FC11) only matters for the CON-only message codes 0x02C-0x02E: message level on a CON unit.
AlarmLevel alarm_level(int code, std::optional<bool> absolute_encoder = std::nullopt,
                       std::optional<int> system_code = std::nullopt);

/// The fault rule of the status decode: ALMH set, or a code of level operation cancelled / cold start, or an unknown
/// level without ALML. A message-level code is never a fault. `system_code` as in alarm_level().
bool alarm_is_fault(int code, bool alarm_bit = false, bool warning_bit = false,
                    std::optional<int> system_code = std::nullopt);

/// Group letter of a code ("A".."F", "Warning", "Other"; UNVERIFIED).
std::string alarm_group(int code);

/// SUPT b13 ABSE / SUP2 b18 MABS from the class-1 support words (name -> value), nullopt when not readable.
std::optional<bool> absolute_encoder_from_support(const std::map<std::string, std::uint32_t>* support);

/// Everything the driver knows about one code (no I/O).
struct AlarmInfo {
    int code = 0;
    std::string hex;          ///< "0x0EE"
    std::string name;
    AlarmLevel level = AlarmLevel::None;
    std::string level_text;
    std::string group;
    std::string group_name;
    bool fault = false;
    bool warning = false;
    std::string reset;        ///< how to clear it, in words
    std::string home_needed;  ///< whether a home return is needed afterwards, in words
    std::string remedy;       ///< the driver's short advice
    std::string detail_text;  ///< decoded detail / address of a history record ("" when none)
    std::optional<std::string> family;  ///< the CON family it was read for (alarm_family), nullopt on an EC
};

/// AlarmInfo of a code (detail / address from a history record when known).
/// `system_code` / `family` (alarm_family): the CON-only message codes and the per-family clearing rule of 0x0CE /
/// 0x0D2 / 0x0EE (the level itself stays conservative); the EC's maintenance-warning clearing texts when not a CON.
AlarmInfo alarm_info(int code, std::optional<bool> absolute_encoder = std::nullopt,
                     std::optional<int> detail = std::nullopt, std::optional<int> address = std::nullopt,
                     std::optional<int> system_code = std::nullopt,
                     const std::optional<std::string>& family = std::nullopt);

/// Decode an alarm's detail / address words where their meaning is known ("" otherwise; UNVERIFIED):
/// alarms.describe_alarm_detail.
std::string describe_alarm_detail(int code, std::optional<int> detail, std::optional<int> address);

/// A table of the alarm module as (code, text) pairs in code order, for the bindings' tables:
/// "controller_names" (IAI's names), "curated_names" (the driver's), "con_names" (the CON-only message codes),
/// "groups" (group letter), "home_needed", "warning_clearing" (the EC's maintenance warnings), "abs_error_details",
/// "row_fields", "command_fields", "specific_remedies". std::invalid_argument for another name.
std::vector<std::pair<int, std::string>> alarm_table(const std::string& name);

/// The (group key, text) pairs of "group_names" or "group_advice" (std::invalid_argument for another name), and the
/// per-family clearing texts of 0x0CE / 0x0D2 / 0x0EE as (code, family, text) ("family_clearing").
std::vector<std::pair<std::string, std::string>> alarm_group_table(const std::string& name);
std::vector<std::tuple<int, std::string, std::string>> alarm_family_clearing_table();
/// The family-independent sentence used for 0x0CE / 0x0D2 / 0x0EE when the family is not known.
std::string alarm_family_clearing_any();

/// One slot of the alarm history (0x0500: 32 x {detail, address, 0, code, time hi, time lo}; index 0 newest, VERIFIED).
struct AlarmRecord {
    int index = 0;
    int code = 0;
    int detail = 0;   ///< 0000 = none
    int address = 0;  ///< FFFF = none
    std::optional<std::int64_t> time_ms;  ///< ms after power-on, nullopt when FFFFFFFF

    std::string name() const { return alarm_name(code); }
    AlarmLevel level() const { return alarm_level(code); }
    /// The 0x0FFF "PowerUP No Error" marker the unit writes at every power-up.
    bool power_up() const { return code == 0x0FFF; }
    /// "[<d> d ]hh:mm:ss.mmm" after power-on, nullopt without a time stamp.
    std::optional<std::string> time_after_power_on() const;
};

/// Decode the history words (any multiple of 6). The list ends at the first empty slot (code 0), as in the Python
/// and C# editions, unless include_empty (then every slot is returned).
std::vector<AlarmRecord> alarm_history_slots(const std::vector<std::uint16_t>& words, bool include_empty = false);

}  // namespace iai_actuator_driver
