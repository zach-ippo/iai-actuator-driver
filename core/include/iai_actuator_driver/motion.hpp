// Motion extras of an Axis: the push move, the command-profile move with its own deceleration (FC17 0x0F00 + the PCRF
// coil), the acceleration patterns (CTLF b6-b7), mid-move changes and point sequences with blending. Options, results
// and the register facts they use. Port of the Python edition's ec.py (push, move_to with decel_g / accel_pattern,
// change_speed / change_accel, move_sequence) and registers.py; implemented by src/axis_motion_extras.cpp.
//
// Evidence: the push move (PPOW = floor(pct x 255 / 100), contact / miss with the settle rule), the command profile on
// the EC-GRBP8M gripper and the PCON-CB, and the s_motion / filter patterns on the EC-GRBP8M gripper are VERIFIED in
// the Python edition; acceleration patterns on other ECs and blending (move_sequence with blend_band) are UNVERIFIED.
// On hardware, VERIFIED only where docs/COMPATIBILITY.md says so (the 2026-10-31 run covers this C++ edition).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

namespace reg {

// ------------------------------------------------------------------------------------------ command profile
// Class 0x0F at 0x0F00, 20 words: PCMD s32, VCMD u32, ACMD u32, DCMD u32, INP u32, SPOW, DPOW, PPOW, CTLF, LPOW, CTF2
// (u16), ZNMP s32, ZNLP s32. Written with FC17 (0x9000 x34 read in the same frame), then started with the PCRF coil
// (self-clearing; STOP stops it; a new profile + PCRF retargets). VERIFIED in the Python edition on the EC-GRBP8M
// gripper and the PCON-CB with an RCP6.
inline constexpr int kActiveProfileCategory = 0x000F;
inline constexpr int kProfileBase = 0x0F00;
inline constexpr int kProfileWords = 20;
inline constexpr int kProfileSpow = 10;          ///< word offsets inside the block
inline constexpr int kProfileDpow = 11;
inline constexpr int kProfilePpow = 12;
inline constexpr int kProfileCtlf = 13;
inline constexpr int kProfileMonitorWords = 34;  ///< 0x9000 x34 read back in the FC17 frame (the monitor block)
inline constexpr int kCoilPcrf = 0x042E;         ///< DRGE b1 PCRF: run the profile at 0x0F00 (written first, FC17)

// ------------------------------------------------------------------------------------------ acceleration patterns
// CTLF b6-b7 MOD: 00 trapezoid, 01 S-motion (ratio CON No.56), 10 first-order filter (time constant No.55); 11 is not
// allowed (DOC ME0162-12A p.5-134; ME0384 p.6-102 to 6-105). Offered when the unit's MVPT support word says so.
inline constexpr std::uint16_t kCtlfModMask = 0x00C0;
inline constexpr std::uint16_t kCtlfModSMotion = 0x0040;
inline constexpr std::uint16_t kCtlfModFilter = 0x0080;
inline constexpr std::uint32_t kMvptSMotion = 1u << 1;  ///< MVPT b1 SCUR
inline constexpr std::uint32_t kMvptFilter = 1u << 2;   ///< MVPT b2 PFIL
/// S-motion is invalid above this acceleration time (DOC PCON p.4-18).
inline constexpr double kSMotionMaxAccelTimeS = 2.0;

// ------------------------------------------------------------------------------------------ push
inline constexpr double kPushSpeedMaxMmS = 20.0;  ///< push speed when neither PSHV 8.15 nor MXPH 7.8 is readable

}  // namespace reg

/// A push outcome must hold this long with the position unchanged (hardware test G5: a contact fingerprint showed for
/// a moment during a push that then missed). AxisOptions::push_settle_s.
inline constexpr double kPushSettleSDefault = 0.15;

/// The acceleration pattern of a move (CTLF b6-b7).
enum class AccelPattern {
    Trapezoid,  ///< 00, the default
    SMotion,    ///< 01: S-curve (CON No.56 ratio); refused above an acceleration time of 2 s
    Filter,     ///< 10: first-order filter (CON No.55 time constant)
};

/// "trapezoid", "s_motion", "filter".
inline const char* to_string(AccelPattern p) noexcept {
    return p == AccelPattern::SMotion ? "s_motion" : p == AccelPattern::Filter ? "filter" : "trapezoid";
}
/// The pattern of a name ("trapezoid", "s_motion", "filter"), nullopt when unknown.
inline std::optional<AccelPattern> accel_pattern_from_string(const std::string& name) {
    if (name == "trapezoid") return AccelPattern::Trapezoid;
    if (name == "s_motion") return AccelPattern::SMotion;
    if (name == "filter") return AccelPattern::Filter;
    return std::nullopt;
}
/// CTLF b6-b7 of a pattern.
inline std::uint16_t accel_pattern_bits(AccelPattern p) noexcept {
    return p == AccelPattern::SMotion ? reg::kCtlfModSMotion : p == AccelPattern::Filter ? reg::kCtlfModFilter : 0;
}

/// Settings of Axis::push.
struct PushOptions {
    int direction = 1;                ///< +1 pushes away from home (increasing position), -1 toward home
    double speed = 10.0;              ///< approach speed (units/s); the push itself runs at Axis::push_speed()
    double accel_g = 0.1;
    bool wait = true;                 ///< false: return nullopt right after the command
    std::optional<double> timeout_s;  ///< default: 2 x the approach estimate + 2 x band / push speed + 3 s
};

/// Outcome names of a push.
inline constexpr const char* kPushContact = "contact";
inline constexpr const char* kPushMissed = "missed";

/// Outcome of Axis::push.
struct PushResult {
    std::string outcome;               ///< kPushContact (holding against the work) or kPushMissed
    double position_mm = 0.0;
    Status status;
    std::vector<std::string> warnings; ///< e.g. a force below the unit's recommended minimum (allowed)

    /// The push met the work (PEND and DSSE bit 0); the motor keeps pushing until the next move command.
    bool contact() const { return outcome == kPushContact; }
    /// The push went over the whole band without contact (DSS1 PSFL).
    bool missed() const { return outcome == kPushMissed; }
};

/// {"outcome", "position_mm", "status", "warnings"} (Python PushResult.to_dict). Implemented in axis_motion_extras.cpp.
Json to_json(const PushResult& value);

/// One point of Axis::move_sequence: a position, optionally with its own speed / accel / band (default: the
/// SequenceOptions values). Converts from a plain position.
struct SequencePoint {
    double position = 0.0;
    std::optional<double> speed;
    std::optional<double> accel_g;
    std::optional<double> band;

    SequencePoint() = default;
    SequencePoint(double position_, std::optional<double> speed_ = std::nullopt,  // NOLINT: implicit on purpose
                  std::optional<double> accel_g_ = std::nullopt, std::optional<double> band_ = std::nullopt)
        : position(position_), speed(speed_), accel_g(accel_g_), band(band_) {}
};

/// Settings of Axis::move_sequence.
struct SequenceOptions {
    double speed = 50.0;
    double accel_g = 0.1;
    double band = 0.1;
    /// nullopt: every point completes by the safe rule before the next is sent. 0.01..10 units: the next target is
    /// sent once the axis is within blend_band of the current one (the move is re-targeted; UNVERIFIED on hardware).
    std::optional<double> blend_band;
    std::optional<double> timeout_s;  ///< per point (default: 2 x the move estimate + 2 s)
    std::optional<Completion> completion;  ///< the rule of every completed point (nullopt: AxisOptions::completion)
};

}  // namespace iai_actuator_driver
