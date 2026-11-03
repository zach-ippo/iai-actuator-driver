// Result types of the Axis operations that change a unit (parameter writes, position-table writes / teach, software
// reset). Plain structs: the Axis fills them; their JSON form is in json.hpp (model unit). Port of ec.py
// ParameterWriteResult / SoftwareResetResult (C# AxisTypes.cs) plus the non-volatile write guard fields.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

// ------------------------------------------------------------------------------------------ non-volatile guard
// The units' non-volatile memory lasts about 100,000 write cycles (IAI). Before a parameter commit, a position-table
// write or a teach the Axis compares with what the unit holds: equal -> nothing is written or committed and the result
// says skipped with kNvUnchangedNote. A parameter commit still runs when this axis has uncommitted RAM writes from
// commit = false calls. force = true writes anyway. Each Axis counts its non-volatile writes (nonvolatile_writes())
// over a one-hour window: above kNvWarnPerHour it logs one warning per hour (kNvRateWarning). Never refused because of
// the rate.

/// Write cycles the non-volatile memory lasts (IAI).
inline constexpr long kNvWriteCycles = 100000;
/// Non-volatile writes per hour above which the Axis warns.
inline constexpr int kNvWarnPerHour = 60;
/// The rate window (s).
inline constexpr double kNvWindowS = 3600.0;
/// Note of a skipped (unchanged) non-volatile write.
inline constexpr const char* kNvUnchangedNote = "unchanged: nothing written (one non-volatile write cycle saved)";
/// Note of a skipped RAM-only parameter write (commit = false, value unchanged).
inline constexpr const char* kRamUnchangedNote = "unchanged: nothing written (RAM only)";
/// Note of a forced write of an unchanged value.
inline constexpr const char* kNvForcedNote = "unchanged, written anyway (force): one non-volatile write cycle used";
/// The once-per-hour warning (formatted by the Axis): the writes in the last hour, the axis ("COM3 slave 1", "slave 1")
/// and the days the memory lasts at this rate (the Python edition's wording).
inline constexpr const char* kNvRateWarning =
    "%d non-volatile writes in the last hour on %s: the unit's non-volatile memory lasts about 100,000 write cycles "
    "(about %.0f days at this rate); for targets that change every cycle use move_to (direct moves are RAM only)";

// ------------------------------------------------------------------------------------------ parameters

/// Outcome of Axis::write_parameter (values as the PC tool shows them).
struct ParameterWriteResult {
    int number = 0;                  ///< P-number / No.; 0 for a raw item
    std::string name;
    std::string key;                 ///< "8.9"
    std::string address;             ///< "0x0810"
    std::string unit;
    std::string before;
    std::string after;
    ParamValue value;
    std::string raw_before;          ///< hex
    std::string raw_after;
    bool changed = false;            ///< the item's raw value changed
    bool verified = false;           ///< read back equal to what was written (true when skipped: the unit holds it)
    bool committed = false;          ///< PARW sent and the handshake ended idle
    bool commit_confirmed = false;   ///< the unit was seen acting on PARW (DRGE b7 = 1 or DSSE b7 = 0 after it)
    bool listed_for_model = true;
    bool verified_for_model = false;  ///< kept for compatibility: an IA-OS list is recorded for the unit code
    bool home_parameter = false;
    bool skipped = false;            ///< non-volatile guard: unchanged, nothing written or committed
    bool forced = false;             ///< unchanged but written anyway (force)
    std::vector<std::string> warnings;
    std::string note;                ///< kNvUnchangedNote / kRamUnchangedNote / kNvForcedNote / other

    /// In non-volatile memory as far as the unit showed: committed and confirmed.
    bool saved() const { return committed && commit_confirmed; }
    /// Always true: a parameter applies only after a software reset or a power cycle.
    bool needs_reset() const { return true; }
    /// When the new value applies, in words.
    std::string takes_effect() const {
        if (skipped) return "unchanged: the unit already holds this value";
        if (committed && !commit_confirmed)
            return "after a software reset or a power cycle IF the commit took - it was NOT confirmed (the unit never "
                   "showed PARW busy): check the value after a power cycle";
        if (committed) return "after a software reset (software_reset()) or a power cycle";
        return "RAM only: lost at the next power-up (probably at a software reset too); untested before then";
    }
};

// ------------------------------------------------------------------------------------------ position table

/// Outcome of Axis::write_position_table / teach_position.
struct PositionWriteResult {
    int row = 0;
    PositionEntry entry;      ///< read back after the write; when skipped, what the unit holds
    bool skipped = false;     ///< non-volatile guard: the row already held these 16 words
    bool forced = false;
    std::string note;         ///< kNvUnchangedNote / kNvForcedNote / ""
    std::vector<std::string> warnings;
};

// ------------------------------------------------------------------------------------------ software reset

/// Outcome of Axis::software_reset: the status once the unit answered again.
struct SoftwareResetResult {
    bool accepted = false;      ///< answered the reset with rejection 0 (01 45 00 00)
    bool reply_lost = false;    ///< no (complete) answer; the reset was confirmed from the status
    std::optional<int> baud_rate;  ///< the rate the unit answered at afterwards (nullopt: transport without one)
    bool baud_changed = false;
    Status status;              ///< first status after the reset
    std::optional<double> uptime_before_s;
    std::optional<double> uptime_after_s;
    double elapsed_s = 0.0;
    std::vector<std::string> notes;
    bool confirmed = true;      ///< the restart was seen (PMSS 1 -> 0, a reply gap, the uptime restarting, HEND lost,
                                ///< or the servo coming back on by itself)
    std::string confirmed_by;   ///< how, in words

    /// PMSS after the reset. It depends on the unit: the EC-R6S rod dropped it to 0 (like a power-up: PIO has control
    /// again), the EC-GRBP8M gripper and the PCON-CB kept 1 (VERIFIED in the Python edition).
    bool modbus_enabled() const { return status.modbus_enabled; }
    bool safety_speed() const { return status.safety_speed(); }
    bool servo_on() const { return status.servo_on; }
    bool homed() const { return status.homed; }
    /// HEND is 0 after the reset: run home() (after reset_alarm() if an alarm is active).
    bool needs_home() const { return !status.homed; }
    int alarm_code() const { return status.alarm_code; }
};

}  // namespace iai_actuator_driver
