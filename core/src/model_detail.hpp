// Internal declarations shared by the model unit's sources (not part of the public API): the alarm tables
// (alarm_tables.cpp, used by alarms.cpp), the EC user-parameter helpers (user_parameters.cpp) and the CON parameter
// helpers (con_parameters.cpp), both used by parameter_plan.cpp.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "iai_actuator_driver/parameters.hpp"

namespace iai_actuator_driver {

namespace alarm_data {
const char* controller_name(int code);
const char* curated_name(int code);
const char* con_name(int code);
const char* group_letter(int code);
const char* home_needed(int code);
const char* specific_remedy(int code);
const char* warning_clearing(int code);
const char* abs_error_detail(int code);
const char* row_field(int code);
const char* command_field(int code);
const char* group_name(const std::string& group);
const char* group_advice(const std::string& group);
const char* family_clearing_any();
/// What clears 0x0CE / 0x0D2 / 0x0EE on a CON family ("PCON", "ACON/DCON", "SCON"), nullptr when not documented.
const char* family_clearing(int code, const std::string& family);
/// Every (code, text) entry of one of the tables above, in code order ("controller_names", "curated_names", "con_names",
/// "groups", "home_needed", "specific_remedies", "warning_clearing", "abs_error_details", "row_fields",
/// "command_fields"); `known` is false (and the list empty) for another name.
std::vector<std::pair<int, const char*>> table_entries(const std::string& name, bool* known);
}  // namespace alarm_data

namespace ec_detail {

/// One user parameter decoded from its class item(s) (info._decode_param).
struct Decoded {
    bool found = false;
    std::string key;                      ///< "8.14", "C.1[1]", "8.25+8.29"
    std::optional<std::string> address;   ///< "0x0810" ("0x0818/0x081C" for a pair)
    std::optional<std::string> raw_hex;
    ParamValue value;                     ///< empty = None
    std::string text;
    std::optional<std::uint32_t> pattern; ///< P3: the ORG home pattern
};

Decoded decode(const ParamDef& p, const ClassItem* item, const ClassItem* item2);
bool home_pattern_settable(std::optional<std::uint32_t> raw);
std::map<int, ParameterVisibility> visibility(const ParameterContext& ctx);
std::optional<std::vector<int>> p9_valid_values(std::optional<int> level, std::optional<bool> exfn,
                                                std::optional<int> mxpt);
ParamValue parse_value(const ParamDef& p, const ParamValue& value);
std::uint32_t encode(const ParamDef& p, std::uint32_t raw, double number);
std::vector<std::string> guard(const ParamDef& p, double number, bool rotary,
                               const std::optional<std::string>& model_name);
void check_value(const ParamDef& p, double number, const Decoded& before, const ParameterContext& ctx,
                 std::optional<double> p1_ceiling);
bool is_rotary(const ParameterContext& ctx);
std::string item_note(int class_id, int item_id);
extern const char* const kHomeWarning;

}  // namespace ec_detail

namespace con_detail {

/// What a CON unit says about itself, for check_value() (nullopt = not known).
struct UnitLimits {
    std::optional<double> max_speed;      ///< MAXV 7.4 (units/s)
    std::optional<double> max_accel;      ///< MAXA 7.5 (G)
    std::optional<double> max_push_speed; ///< MXPH 7.8 (units/s)
    std::optional<double> lead_mm;        ///< LEAD 7.3 (= No.77)
    std::optional<std::uint32_t> encoder_pulses;  ///< EPLS 9.12 (item meaning UNVERIFIED)
    std::optional<double> min_band_mm;    ///< MINI 3.67 when the unit enforces it (SUP2 CINP and ATYP RINP set)
    bool rotary = false;                  ///< ATYP 7.7 b8
};

/// The result of check_value().
struct Check {
    ParamValue value;
    std::vector<std::string> warnings;
    std::optional<std::string> family;  ///< the range table used (nullopt = PCON-CB range for an unknown family)
    std::optional<double> lo, hi;
};

std::optional<std::string> family_of(const std::string& family);
ConParam for_family(const ConParam& p, const std::string& family);
ParamValue decode(const ConParam& p, std::uint32_t raw);
std::string text(const ConParam& p, const ParamValue& value);
ParamValue parse(const ConParam& p, const ParamValue& value);
std::uint32_t encode(const ConParam& p, std::uint32_t raw, const ParamValue& value);
UnitLimits unit_limits(const std::map<int, ClassBlock>& classes,
                       const std::optional<std::map<std::string, std::uint32_t>>& support);
Check check_value(const ConParam& p, const ParamValue& value, const std::string& family, const UnitLimits& unit);
std::string note_of(int no);
/// The audit's finding when the soft limit + (No.3) lies below the soft limit - (No.4), else "" (no unit read).
std::string inverted_soft_limits(double plus, double minus);
std::pair<int, int> baud_range();

/// A number with its Python type: `decimal` = a float in the Python edition's table (1.00, 0.0), else an int.
struct Number {
    double value = 0.0;
    bool decimal = false;
};
/// The smallest positioning band a unit accepts (con_params.MinBand).
struct Band {
    double mm = 0.0;
    std::string formula;
    std::string doc;
    bool certain = true;
};

// The tables and the table functions of con_params.py, for the bindings (src/c_api_tables.cpp).
std::vector<std::pair<int, int>> baud_codes();
std::vector<int> con_baud_rates();
std::vector<std::string> families();
/// The ranges that differ from the PCON-CB table for "ACON/DCON" / "SCON" (empty for PCON / another name).
std::vector<std::tuple<int, double, double>> family_ranges(const std::string& family);
std::string range_doc(const std::optional<std::string>& fam);
std::string unknown_family_note();
/// IAI's category letter of No.`no` for a family (family_of() result), "" when not listed.
std::string category(int no, const std::optional<std::string>& fam);
/// nullopt: the PCON-CB categories; a family: only that family's own entries (the SCON-only numbers).
std::vector<std::pair<int, std::string>> categories(const std::optional<std::string>& fam);
std::string category_text(const std::string& category);
std::optional<Number> factory_default(int no, const std::optional<std::string>& fam);
/// nullopt: the PCON-CB defaults; a family: only the defaults that differ for it.
std::vector<std::pair<int, Number>> factory_defaults(const std::optional<std::string>& fam);
std::optional<Band> min_band(const std::optional<std::string>& fam, std::optional<double> lead_mm,
                             std::optional<std::uint32_t> encoder_pulses, std::optional<double> mini_mm);

}  // namespace con_detail

}  // namespace iai_actuator_driver
