// Customer compatibility self-test ("iai-check" / "ec-check"). Port of the Python edition's check.py (the source of
// truth); implemented by src/check.cpp. The report fields and the JSON report are the Python edition's.
//
// Read-only by default (nothing is written to the unit): link, slave id, index table layout, identification, soft
// limits vs the configured stroke, speed limit (MAXV item 7.4) vs the speed cap, status, alarm, alarm history,
// position table, direct move block, Modbus control, safety speed, the full unit information (info::unit_info, kept in
// the report's unit_info; it never fails the check) and the parameter values against the documented ranges
// (parameter_ranges: WARN at most; on a CON unit a No.10 below the minimum positioning band would raise cold-start
// alarm 0x0A1 at the next reset). With motion it also makes a short move (default +/-5 mm at 10 mm/s; at most a
// quarter of the stroke) from the current position and back, inside the effective stroke, and then restores the
// servo / Modbus-control state it found. VERIFIED in the Python edition on the EC units tested; this C++ code is
// UNVERIFIED on hardware.
#pragma once

#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/json.hpp"

namespace iai_actuator_driver {

class Axis;

/// Check results.
inline constexpr const char* kCheckPass = "PASS";
inline constexpr const char* kCheckFail = "FAIL";
inline constexpr const char* kCheckWarn = "WARN";
inline constexpr const char* kCheckSkip = "SKIP";
/// The report's "tool" field.
inline constexpr const char* kCheckTool = "iai-check (unofficial)";

/// One line of the self-test.
struct CheckItem {
    std::string name;
    std::string result;  ///< kCheckPass / kCheckFail / kCheckWarn / kCheckSkip
    std::string detail;
    Json data = Json::object();
};

/// Everything the self-test found; serialised to the JSON report (to_json).
struct CheckReport {
    std::string timestamp;
    std::string port;
    int slave = 1;
    std::string strategy;                        ///< "coil" / "iai"
    std::string driver_version;
    std::optional<Span> stroke;                  ///< effective stroke used for the checks
    bool motion_requested = false;
    std::optional<Span> configured_stroke;
    std::optional<Span> ec_soft_limits_mm;       ///< the unit's soft limits as stored
    std::optional<double> ec_max_speed_mm_s;     ///< the unit's maximum speed (MAXV item 7.4; 8.10 only as fallback)
    std::optional<double> speed_cap_mm_s;        ///< what the driver allows: min(configured, unit maximum)
    std::optional<int> baud;
    std::optional<std::string> model;            ///< display name: the model string when the unit has one
    std::optional<std::string> controller;       ///< Report Slave ID in words (system, unit code, firmware)
    std::optional<std::string> model_name;
    std::optional<std::string> serial_number;
    std::optional<int> system_code;
    std::optional<int> unit_code;
    std::optional<std::string> firmware;
    std::vector<CheckItem> items;
    Json unit_info;                              ///< info::unit_info(); null when not read

    /// True when no item failed (WARN and SKIP do not fail the check).
    bool passed() const;
    /// The item with this name, nullptr when absent (the last one when a name repeats).
    const CheckItem* item(const std::string& name) const;
    /// Write the JSON report (UTF-8, indented); std::ios_base::failure when the file cannot be written.
    void write_json(const std::string& path) const;
};

/// The JSON form of a report: every field above plus "result" ("PASS" / "FAIL") and "tool" (kCheckTool).
Json to_json(const CheckReport& report);
Json to_json(const CheckItem& item);

/// Settings of run_check.
struct CheckOptions {
    bool motion = false;          ///< also the short move out and back
    std::string port_label;       ///< recorded as the report's port
    double step_mm = 5.0;         ///< the move (at most a quarter of the stroke, at least 0.1)
    double speed_mm_s = 10.0;
    double accel_g = 0.1;
    std::function<void(const CheckItem&)> on_item;  ///< sees each line as it completes
};

/// Run the self-test against `axis` and return the report. Never throws for what the unit answers or refuses (that
/// becomes a FAIL / WARN line); the first line is the link check.
CheckReport run_check(Axis& axis, const CheckOptions& options = CheckOptions());

/// One report line as the command line prints it: "PASS  link               answered FC03 (...)".
std::string format_check_item(const CheckItem& item);

/// Write the report to `path`; when that fails, to the temp directory under the same file name (a warning on `err`),
/// and when that fails too, print the JSON on `out`. Returns where it went: "report written to <path>", "... (fallback
/// location)" or "report printed above" (the command line ends with "RESULT PASS - <that>").
std::string save_check_report(const CheckReport& report, const std::string& path, std::ostream& out,
                              std::ostream& err);

}  // namespace iai_actuator_driver
