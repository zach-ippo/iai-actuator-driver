// Named parameters: the EC user parameters P1-P18 (as IA-OS shows them) and the numbered CON parameters (PCON / ACON /
// SCON / DCON No.1..), their decoding, value parsing / encoding and every write guard - as pure functions with no I/O.
// The Axis reads the class blocks, calls plan_parameter_write(), then writes, verifies and commits (axis.hpp).
// Port of info.py (user parameters, visibility, model lists), con_params.py and the checks of ec.py parameters() /
// read_parameter() / write_user_parameter() / write_parameter() (C# UserParameters.cs, ConParameters.cs,
// Axis.Config.cs); implemented by the model unit (src/user_parameters.cpp, src/con_parameters.cpp,
// src/parameter_plan.cpp).
//
// Guards (all refuse before anything is written): read-only / unconfirmed parameters (UnsafeCommandError); values
// outside the ranges give SoftLimitError. Dangerous writes (home direction / offset, motor / encoder type, lead and
// axis type, gains, PIO / fieldbus / baud, widening a soft limit, raw items of unknown meaning) and a parameter not in
// this unit's list are written with a warning: allow_dangerous, allow_unlisted and allow_home_params are accepted and
// ignored (no longer needed). There is no per-model opt-in: every unit's parameters are written the same way.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/directory.hpp"

namespace iai_actuator_driver {

/// A value as the PC tool shows it: a number, or a label such as "1:Forward" / "disable".
struct ParamValue {
    std::optional<double> number;
    std::string text;  ///< the label, or the number's text
    bool whole = false;  ///< the Python edition holds it as an int (a code, a count, a flag): its JSON is an integer

    static ParamValue of(double number);
    /// A code, a count or a flag (the Python edition's int): the same value, written as an integer in the JSON.
    static ParamValue of_whole(double number);
    /// A string that is a whole number / decimal becomes a number; anything else a label. "" = no value.
    static ParamValue parse(const std::string& text);
    bool empty() const { return !number && text.empty(); }
    /// The number, or the text.
    std::string to_string() const;
};

/// A parameter key: a named parameter number (2, "2", "P2", "No.35") or a raw item "class.item[entry]" ("8.15",
/// "C.1[1]"; class in hex above 9).
struct ParameterKey {
    std::optional<int> number;
    int class_id = 0;
    int item_id = 0;
    int entry = 0;
    std::string text;  ///< as given

    bool is_item() const { return !number.has_value(); }
    /// std::invalid_argument for a key that is neither form.
    static ParameterKey parse(const std::string& key);
    static ParameterKey of(int number);
};

// ------------------------------------------------------------------------------------------ EC user parameters

/// One EC user parameter (P-number) definition.
struct ParamDef {
    int no = 0;
    std::string name;
    int class_id = 0;
    int item_id = 0;
    std::string kind;         ///< "mm", "parity", "bit", "enum", "multi_io"
    std::string unit;         ///< "mm" or ""
    std::optional<int> bit;   ///< for kind "bit"
    double offset_mm = 0.0;   ///< P1: -0.30 (raw = P1 + 0.30 mm)
    std::map<int, std::string> choices;
    std::string encoding;
    bool read_only = false;
    int entry = 0;
    std::optional<int> item_id2;  ///< P11: the second item
    std::string evidence;
    std::string note;

    std::string key() const;  ///< "class.item" ("C.1[1]" with an entry)
};

/// P1-P18 (C# UserParameters.USER_PARAMETERS).
const std::vector<ParamDef>& ec_user_parameters();
/// The definition of P`no`, nullptr when unknown.
const ParamDef* ec_user_parameter(int no);
/// P3 / P4: home direction / home offset (ABS error 0x0EE after a reset on an absolute unit, VERIFIED on the gripper).
bool is_ec_home_parameter(int no);

/// Driver sanity ranges of the EC mm parameters (Python P1_MIN_MM, P2_RANGE_MM, P4_RANGE_MM).
inline constexpr double kP1MinMm = 1.00;
inline constexpr double kP2MinMm = 0.01;
inline constexpr double kP2MaxMm = 1.00;
inline constexpr double kP4MinMm = 0.00;
inline constexpr double kP4MaxMm = 5.00;

/// Why a parameter is shown / hidden on this unit (the runtime visibility rules).
struct ParameterVisibility {
    std::optional<bool> shown;
    std::string reason;
};

/// The unit's parameter list for its (system, unit) code and model, filtered by the visibility rules.
struct ParameterModel {
    std::optional<int> unit_code;
    std::optional<int> system_code;
    std::optional<std::string> model_name;
    std::optional<std::string> family;
    bool known = false;              ///< an IA-OS parameter list is recorded for the unit code (candidate table)
    std::vector<int> candidates;
    std::vector<int> listed;
    std::vector<std::pair<int, std::string>> hidden;  ///< (P-number, why)
    bool verified = false;           ///< kept for compatibility: equals known
    std::vector<std::string> seen_on;  ///< kept for compatibility; no longer used by the driver (always empty)
    std::string note;
};

// ------------------------------------------------------------------------------------------ CON parameters

/// One numbered CON parameter (con_params.ConParam).
struct ConParam {
    int no = 0;
    std::string name;
    std::string symbol;
    int class_id = 0;
    int item_id = 0;
    std::string kind;          ///< "num", "bits", "enum", ...
    int entry = 0;
    double scale = 1.0;
    bool is_signed = false;
    std::vector<int> bits;     ///< [lo, hi] bit field, empty for the whole item
    std::string unit;
    std::optional<double> lo;
    std::optional<double> hi;
    std::map<int, std::string> choices;
    bool dangerous = false;
    bool writable = true;
    bool lo_decimal = false;   ///< lo is a decimal (a float in the Python edition's table) even when whole
    bool hi_decimal = false;   ///< hi likewise ("0.0", "1.0", "100.0" in the messages)

    std::string key() const;
};

/// Every CON parameter definition (con_params.CON_PARAMETERS), optionally adapted to a family ("PCON", "ACON/DCON"...).
std::vector<ConParam> con_parameters(const std::string& family = std::string());
/// No.`no`, nullptr when unknown.
const ConParam* con_parameter(int no);
/// No.5 / No.22: home direction / home offset.
bool is_con_home_parameter(int no);

// ------------------------------------------------------------------------------------------ listing / planning

/// One row of Axis::parameters() / read_parameter() (C# ParameterInfo).
struct ParameterInfo {
    int number = 0;                 ///< P-number / No.; 0 for a raw item
    std::string name;
    std::optional<std::string> symbol;
    std::string item;               ///< "class.item"
    std::optional<std::string> address;  ///< "0x0810"
    ParamValue value;
    std::optional<std::string> text;
    std::string unit;
    std::optional<double> min;
    std::optional<double> max;
    bool min_decimal = false;        ///< min / max are floats in the Python edition (written 1.0 even when whole), else ints
    bool max_decimal = false;
    std::map<std::string, std::string> choices;
    bool writable = false;
    bool dangerous = false;
    std::optional<std::string> raw;  ///< hex
    std::string family = "EC";       ///< "EC" or "CON"
    // raw items only:
    std::optional<std::uint32_t> unsigned_value;
    std::optional<std::int32_t> signed_value;
    std::optional<int> words;
    std::optional<int> named;        ///< the named parameter living at this item, if any
    std::string note;
};

/// What the parameter functions need to know about the unit (read by the Axis through the directory).
struct ParameterContext {
    bool is_ec = true;
    std::string units = "mm";
    std::string family = "EC";                     ///< product family
    std::map<int, ClassBlock> classes;             ///< parameter_classes_needed() read with read_class
    std::optional<std::map<std::string, std::uint32_t>> support;  ///< class-1 support words
    std::optional<std::uint32_t> accl;
    std::optional<int> unit_code;
    std::optional<int> system_code;
    std::optional<std::string> model_name;
    std::optional<double> p1_ceiling_mm;           ///< the largest P1 this axis has read (P1 can only narrow)
    std::optional<double> position_mm;             ///< current position (P1 not below it)
};

/// The classes the Axis must read into ParameterContext::classes (EC: 3, 7, 8, 9, 0xC; CON: those of the table).
std::vector<int> parameter_classes_needed(bool is_ec);

/// The model's parameter list (info.model_parameter_list + parameter_visibility).
ParameterModel model_parameter_list(const ParameterContext& ctx);

/// Every named parameter of the unit, decoded (Axis::parameters()).
std::vector<ParameterInfo> list_parameters(const ParameterContext& ctx);

/// One parameter (named or raw item), decoded. UnsafeCommandError when not offered on this unit.
ParameterInfo describe_parameter(const ParameterContext& ctx, const ParameterKey& key);

/// Options of a parameter write.
struct ParameterWriteOptions {
    bool commit = true;             ///< save with PARW (applies after software_reset() or a power cycle)
    bool allow_dangerous = false;   ///< accepted and ignored (no longer needed): a dangerous write gets a warning
    bool allow_unlisted = false;    ///< accepted and ignored (no longer needed): an unlisted parameter gets a warning
    bool allow_home_params = false; ///< accepted and ignored (no longer needed): a home parameter gets a warning
    bool force = false;             ///< write and commit even when the value is unchanged (the non-volatile guard)
};

/// Everything the Axis needs to perform one parameter write, decided before anything is sent.
struct ParameterWritePlan {
    int number = 0;              ///< 0 for a raw item
    std::string name;
    std::string key;             ///< "8.9"
    std::string unit;
    int class_id = 0;
    int item_id = 0;
    int entry = 0;
    int address = 0;             ///< where the item is on this unit
    int size = 2;                ///< words (1 or 2)
    std::uint32_t raw_before = 0;
    std::uint32_t raw_after = 0; ///< read-modify-write of shared items: only the parameter's bits change
    std::string before;          ///< as the PC tool shows it
    std::string after;
    ParamValue value;
    bool listed_for_model = true;
    bool verified_for_model = false;  ///< kept for compatibility: an IA-OS list is recorded for the unit code
    bool home_parameter = false;
    bool dangerous = false;
    bool rereads_soft_limits = false;  ///< P1: the Axis re-reads the soft limits after the write
    std::vector<std::string> warnings;

    bool changed() const { return raw_before != raw_after; }
    /// The words to write: 1 word (FC06) or 2 words high first (FC10).
    std::vector<std::uint16_t> words() const;
};

/// Decide one write (every guard above). Throws UnsafeCommandError / SoftLimitError; nothing is sent by this function.
/// The opt-ins of `options` are not read (no longer needed).
ParameterWritePlan plan_parameter_write(const ParameterContext& ctx, const ParameterKey& key, const ParamValue& value,
                                        const ParameterWriteOptions& options);

}  // namespace iai_actuator_driver
