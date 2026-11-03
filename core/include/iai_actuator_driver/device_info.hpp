// What a unit tells about itself (identity, soft limits, the class-7 limits) and the ranges the driver enforces.
// Port of Axis.identify / read_soft_limits / read_axis_limits / limits / max_positions (ec.py) and info.py (C#
// DeviceInfo.cs, Axis.cs, AxisTypes.cs); implemented by the model unit (src/device_info.cpp, src/limits.cpp).
// Everything here is READ-ONLY.
//
// The driver adapts to whatever unit is connected: every range comes from the unit itself, and every unit is driven
// the same way.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/reader.hpp"

namespace iai_actuator_driver {

/// A closed range [min, max] of positions (mm, or deg on a rotary axis).
struct Span {
    double min = 0.0;
    double max = 0.0;
    bool operator==(const Span& o) const { return min == o.min && max == o.max; }
};

/// FC11 identity (or the same fields from CTCL 3.1).
struct SlaveIdentity {
    int system_code = 0;           ///< byte 0: 0xEC EC 24 V, 0xAE PCON...
    std::optional<int> run_status; ///< byte 1 (0xFF on every unit tested); nullopt from CTCL
    int unit_code = 0;             ///< byte 2
    std::string firmware;          ///< 4 hex digits
    std::string source;            ///< "FC11" or "CTCL 0x0300"

    /// [system, run status, unit, fw hi, fw lo] (VERIFIED: EC FF 02 00 13 slider, EC FF 0A 00 02 gripper,
    /// EC FF 00 00 02 rod, AE FF 44 00 00 PCON); nullopt for fewer than 5 bytes.
    static std::optional<SlaveIdentity> from_slave_id(const std::vector<std::uint8_t>& data);
    /// CTCL u32 (b31-24 system, b23-16 unit, b15-0 firmware); nullopt when the system byte is 0.
    static std::optional<SlaveIdentity> from_ctcl(std::uint32_t value);
};

/// The identification block (0x0600, 72 words). Text fields nullopt when blank or not text (a CON controller keeps
/// binary data where an EC keeps its model string).
struct IdentificationBlock {
    std::optional<std::string> serial_number;
    std::optional<std::string> actuator_serial;
    std::optional<std::string> user_name;   ///< the user-editable unit name (renamed in IA-OS), not a model field
    std::optional<std::string> model_name;  ///< e.g. "EC-R6S-150-1-PN"
    std::optional<std::uint32_t> accl;

    /// Fields past the end of a short block are nullopt.
    static IdentificationBlock from_words(const std::vector<std::uint16_t>& words);
    /// ACCL bits 8-11 (0 Normal, 1 Position3, 2 Phase1, 3 Phase2, 4 MultiIo; UNVERIFIED).
    std::optional<int> extensibility_level() const;
};

/// Identity of one unit (identify()).
struct DeviceInfo {
    int slave = 1;
    std::optional<int> system_code;
    std::optional<std::string> system_name;
    std::optional<int> run_status;
    std::optional<int> unit_code;
    std::optional<std::string> unit_name;      ///< EC family name for the (system, unit) pair
    std::optional<std::string> firmware;
    std::vector<std::uint8_t> slave_id_raw;
    std::optional<IndexHeader> index_header;
    std::vector<IndexCategory> categories;
    std::optional<std::string> serial_number;
    std::optional<std::string> user_name;
    std::optional<std::string> model_name;
    std::optional<std::string> actuator_serial;
    std::optional<std::uint32_t> accl;
    std::optional<std::string> identity_source;  ///< "FC11" / "CTCL 0x0300"
    std::optional<bool> unit_known;              ///< the driver has a family name for this code (display only)
    std::optional<std::string> product_family;   ///< "EC", "PCON", ... kGenericFamily
    std::optional<std::string> controller_name;  ///< 0x8010.. on a CON controller ("PCON-CB")
    std::optional<int> axis_number;              ///< AXNO 0x8004 (= slave - 1)
    // filled by the Axis from the limits:
    std::optional<Span> soft_limits_mm;          ///< 7.1 / 7.2 as stored (with the 0.30 margin)
    std::optional<double> max_speed_mm_s;        ///< MAXV 7.4 (or the default speed 8.10)
    std::optional<double> max_accel_g;
    std::optional<double> max_decel_g;
    std::optional<std::string> speed_limit_source;
    std::optional<int> actuator_kind;            ///< AKID 7.33
    std::optional<std::string> actuator_kind_name;
    std::optional<bool> rotary_atyp;
    std::string units = "mm";                    ///< "mm" or "deg"
    std::optional<int> max_positions;
    std::vector<std::string> errors;             ///< parts the unit refused / did not answer (never thrown)

    std::string kind() const;                    ///< reg::axis_kind_name(actuator_kind, rotary_atyp)
    std::optional<bool> rotary() const;
    std::optional<int> extensibility_level() const;
    /// "EC 24 V unit 0x02 fw 0013"-style controller description.
    std::string controller() const;
    /// "slider (AKID 9)", nullopt without AKID.
    std::optional<std::string> actuator() const;
    /// The model string, else the controller name, else controller().
    std::string model() const;
    /// The directory record of a class, nullptr when absent.
    const IndexCategory* category(int category_id) const;
};

/// Read what `reader`'s slave tells about itself: FC11 once (kSlaveIdTimeoutS; CTCL 3.1 when silent), the directory,
/// the identification block, AXNO, the controller name. Parts the unit rejects or does not answer are listed in
/// DeviceInfo::errors - never thrown (a TransportError still propagates). The limit fields stay empty (the Axis fills
/// them from read_axis_limits / read_soft_limits).
DeviceInfo read_device_info(UnitReader& reader, const UnitDirectory* directory = nullptr);

/// The unit's own limits, from class 7 by item ID (no per-model table in the driver). MAXV / MAXA VERIFIED on the EC-R6S rod;
/// the push items UNVERIFIED. Positions / speeds in the axis's units.
struct AxisLimits {
    std::optional<double> max_speed_mm_s;      ///< MAXV 7.4
    std::optional<double> max_accel_g;         ///< MAXA 7.5
    std::optional<double> max_decel_g;         ///< MAXD 7.6
    std::optional<double> max_push_speed_mm_s; ///< MXPH 7.8
    std::optional<double> max_push_pct;        ///< MXPP 7.9 (the hard cap; 0 = no push)
    double min_push_pct = reg::kMnppDefaultPct;///< MNPP 7.58 (a recommendation)
    bool min_push_from_unit = false;
    std::optional<int> push_force_factor;      ///< TCPC 7.34
    std::optional<int> push_force_offset;      ///< TCOA 7.35
    std::optional<int> actuator_kind;          ///< AKID 7.33
    std::optional<bool> rotary_atyp;           ///< ATYP 7.7 TYPE bit (units without AKID)
    std::optional<double> lead_mm;             ///< 7.3
    std::optional<double> default_speed_mm_s;  ///< 8.10 (the speed cap when MAXV is unreadable)
    std::optional<int> max_speed_address;      ///< (0.5) where MAXV was read (for the source in words)
    std::optional<int> default_speed_address;  ///< (0.5) where the default speed 8.10 was read
    std::optional<bool> supports_push;         ///< SUPT b11
    std::optional<bool> supports_relative;     ///< SUPT b8
    std::optional<int> max_positions;          ///< MAXP 3.2
    std::optional<double> min_band;            ///< MINI 3.67 (with SUP2 CINP + ATYP RINP)
    std::optional<std::uint32_t> move_patterns;///< MVPT (class 1): b1 SCUR S-motion, b2 PFIL filter, b8 ASOF auto servo off
    std::string source;                        ///< where the values came from, in words
    std::vector<std::string> notes;            ///< classes missing / refused

    std::string units() const;                 ///< reg::axis_units
    std::string kind() const;                  ///< reg::axis_kind_name
    std::optional<bool> rotary() const;
    std::optional<bool> push_available() const;  ///< MXPP > 0 and TCPC != 0 and SUPT PUSH, where known
    std::optional<double> push_force_estimate(double pct) const;  ///< N, from TCPC / TCOA (UNVERIFIED)
    /// The speed cap the unit gives: MAXV, else the default speed 8.10, with its source in words.
    std::pair<std::optional<double>, std::string> unit_speed_cap() const;
};

/// Read class 7 (and 3, 8, 1) through the directory. A class missing or refused leaves its fields empty and adds a
/// note; a timeout propagates.
AxisLimits read_axis_limits(UnitReader& reader, const UnitDirectory& directory);

/// The unit's own soft limits (items 7.1 / 7.2 as stored, with the margin).
struct SoftLimits {
    std::optional<Span> limits;   ///< nullopt when the unit gives no plausible limits
    std::string note;             ///< where they were read from, or why not
    std::optional<double> p1_mm;  ///< item 7.1 minus the 0.30 margin (user parameter P1)
};
SoftLimits read_soft_limits(UnitReader& reader, const UnitDirectory& directory);

/// Soft limits minus the 0.30 margin at each end (unchanged when the span is too small for it).
std::optional<Span> unit_stroke(const std::optional<Span>& soft_limits);
/// Configured stroke intersected with the unit stroke; nullopt when nothing is known or they do not overlap.
std::optional<Span> effective_stroke(const std::optional<Span>& configured, const std::optional<Span>& unit);

/// Number of stored rows: min(MAXP 3.2, the position table's row count), at least 3 on an EC; 3 when neither is known.
int read_max_positions(UnitReader& reader, const UnitDirectory& directory, std::optional<int> system_code);

/// One range of limits().
struct LimitRange {
    std::optional<double> min;
    std::optional<double> max;
    std::string unit;
    std::string source;
    std::optional<double> unit_max;        ///< the unit's own maximum
    std::optional<double> configured;      ///< the cap configured on the Axis
    std::optional<bool> available;         ///< push: the unit can push
    std::optional<double> recommended_min; ///< push: MNPP; speed: the documented minimum speed (set by the Axis)
    std::string recommended_min_source;    ///< speed: where recommended_min comes from (reg::recommended_min_speed)
    std::optional<int> count;              ///< stored rows: how many
    std::optional<double> speed;           ///< push: the push speed (min(PSHV 8.15 / CON No.34, MXPH 7.8); set by the Axis)
    std::string speed_unit;                ///< push: "mm/s" / "deg/s" (empty: no push speed entry)
    std::string speed_source;              ///< push: where the push speed comes from, or why it is not readable
};

/// Every range the driver enforces on one axis (Axis::limits()).
struct AxisRanges {
    std::string units = "mm";
    std::string kind = "unknown";
    std::string family = reg::kGenericFamily;
    LimitRange position;
    LimitRange speed;
    LimitRange accel;
    LimitRange decel;
    LimitRange accel_decel;    ///< one value for both on a 0x9900 move: min(MAXA, MAXD)
    LimitRange push;
    LimitRange band;
    LimitRange push_band;
    LimitRange stored_rows;
    LimitRange jog_step;
    LimitRange speed_override; ///< 1..100 % (0x45 cmd 6)
};

/// Fallback caps while the unit's own maximum is unknown (Python FALLBACK_*).
inline constexpr double kFallbackSpeedCap = 100.0;
inline constexpr double kFallbackAccelCapG = 0.30;
inline constexpr double kFallbackPushCapPct = 70.0;
/// Driver sanity limit of a positioning band (units).
inline constexpr double kMoveBandMax = 10.0;

/// Inputs of make_ranges (what the Axis knows; every field may be unknown).
struct RangeInputs {
    std::optional<AxisLimits> limits;
    std::optional<Span> soft_limits;        ///< as stored
    std::string soft_limits_note;
    std::optional<Span> configured_stroke;  ///< AxisOptions::stroke
    std::optional<double> configured_max_speed;
    std::optional<double> configured_max_accel;
    std::optional<double> configured_max_push;
    std::string family = reg::kGenericFamily;
    int rows = reg::kPositionActiveRows;
};

/// The caps the driver commands with: the min of the configured cap and the unit's maximum; the kFallback* value only
/// while neither is known (as in the Python edition: a 100 mm/s fallback must not cap an 800 mm/s unit).
double speed_cap(const RangeInputs& in);
double accel_cap(const RangeInputs& in);        ///< min(MAXA, MAXD, configured): the 0x9900 accel = decel
double push_cap_pct(const RangeInputs& in);

/// Every range, with its source (Axis::limits()). No I/O.
AxisRanges make_ranges(const RangeInputs& in);

}  // namespace iai_actuator_driver
