// Read-only unit information and the read-only backup: everything a unit tells about itself, decoded where known, as
// JSON in the same layout as the Python edition (info.py: unit_info, backup, backup_axes; the source of truth), so
// reports and backups interchange between the editions. Implemented by src/info.cpp.
//
// Nothing here writes to a unit: the only function codes used are FC03 (read holding registers) and FC11 (report slave
// id, through Axis::identify). Every block read is bounded by the unit's own directory (the index table at 0x0000):
// a unit answers 0000 for unmapped addresses instead of an exception, so the directory is the only guard.
//
// Entry points:
// * unit_info(): one JSON object with identity, user parameters, every raw parameter item, I/O images, decoded status
//   words, monitor values, alarms (current + history), maintenance counters and position data.
// * backup() / backup_axes(): a read-only JSON backup (schema kBackupSchema) of identity, directory, every parameter
//   item, the position rows, the alarm history and the maintenance counters. Restoring is not part of the driver.
// * load_backup(), backup_identity(), backup_rows(), backup_parameters(): read a backup back (the input of a recovery
//   session from a backup, recovery.session_from_backup in the Python edition). No I/O to a unit.
// * decode_bits(), position_mode(), io_bit_tables(), decode_monitor(): pure decoders (no I/O) for a GUI's live view.
//
// Evidence (README "Evidence levels"): bit and item meanings marked confirmed were seen on hardware by the Python
// edition (VERIFIED-LIVE in the bit tables); the other names are IAI's register / bit names (some lists were derived
// for another IAI model) or come from the CON manuals (DOC-CON) and are UNVERIFIED on the EC. This C++ code itself is
// UNVERIFIED on hardware.
#pragma once

#include <cstdint>
#include <exception>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

class Axis;
struct ParameterContext;  // parameters.hpp (user_parameter_rows)

namespace info {

/// meta.schema of unit_info() (the Python edition's info.SCHEMA).
inline constexpr const char* kUnitInfoSchema = "ec_driver.unit_info/1";
/// meta.schema of backup() (the Python edition's info.BACKUP_SCHEMA; the same files are read by both editions).
inline constexpr const char* kBackupSchema = "ec_driver.backup/1";

/// The sections of unit_info(), in order: identity, parameters, monitor, alarms, maintenance, positions.
const std::vector<std::string>& sections();
/// The parameter classes read by unit_info() and backup(): 3, 4, 7, 8, 9, 0x0C.
const std::vector<int>& parameter_classes();

// ------------------------------------------------------------------------------------------ bit tables

/// One named bit of a status / I/O word. confirmed = the meaning was seen on hardware (VERIFIED-LIVE in the Python
/// edition); otherwise evidence says where the name comes from. An active_low bit (e.g. *ALM) is 1 in the normal state.
struct BitDef {
    int bit = 0;
    std::string name;
    std::string meaning;
    bool confirmed = false;
    std::string evidence = "IAI bit name; meaning UNVERIFIED on the EC";
    bool active_low = false;
};

/// A named bit table, high bit first: DSS1, DSS2, DSSE, ZONS, STAT, SSSE, SIPM, DRG1, DRG2, DRGE, CTLF, DOPM_3POS,
/// DOPM_2POS, DIPM_3POS, DIPM_2POS, DIPM_SINGLE. std::out_of_range for another name.
const std::vector<BitDef>& bit_table(const std::string& name);
/// The names bit_table() knows, in the order above.
const std::vector<std::string>& bit_table_names();

/// Decode `word` with a bit table: {"value", "hex", "on": [names of set bits], "bits": [...], "unknown_set": [...]}.
/// bits lists every bit of the table, high bit first, as {"bit", "name", "on", "meaning", "confirmed", "evidence",
/// "active_low"}, followed by any set bit the table does not name ("b<n>", also listed in unknown_set). on is the raw
/// bit (an active_low bit is 1 when normal). std::invalid_argument when the word does not fit in `width` bits.
Json decode_bits(std::uint32_t word, const std::vector<BitDef>& table, int width = 16);
/// The same with a table by name (std::out_of_range for an unknown name).
Json decode_bits(std::uint32_t word, const std::string& table, int width = 16);

/// PIO / position mode from user parameter 9 (class 8 item 2), else from the model string (option MF):
/// {"code", "name", "positions", "layout" ("3pos" / "2pos"), "source", "confirmed", "note"}. 0 = Double solenoid and
/// 3 = Single solenoid (2 positions) VERIFIED on the gripper; 4 = 3-position inferred from the slider.
Json position_mode(std::optional<std::uint32_t> code, const std::optional<std::string>& model = std::nullopt);

/// The DIPM / DOPM tables of a position mode.
struct IoTables {
    const std::vector<BitDef>* dipm = nullptr;
    const std::vector<BitDef>* dopm = nullptr;
};
/// For a position_mode() result (layout "3pos", else 2-position; code 3 = the single-solenoid DIPM).
IoTables io_bit_tables(const Json& mode);
/// For a layout name ("3pos" or anything else = 2-position).
IoTables io_bit_tables(const std::string& layout);

// ------------------------------------------------------------------------------------------ pure decoders

/// Monitor members by name (unsigned; 32-bit members combined, high word first).
using MonitorWords = std::map<std::string, std::uint32_t>;
/// The 0x9000 words by member name (PNOW, ALMC, ... OLLV); only the members fully inside `words`.
MonitorWords split_monitor(const std::vector<std::uint16_t>& words);
/// The members a status poll read (Status::raw: 8 to 22 words).
MonitorWords monitor_words(const StatusRaw& raw);
/// The live sections of unit_info() from monitor words already read (no I/O): {"monitor", "status_words", "io"} in the
/// same layout as unit_info(). Missing members are left out. `mode` is a position_mode() result (null: unknown, the
/// assumed 2-position layout); `rated_current` (MCUR, item 4.5) adds current_pct.
Json decode_monitor(const MonitorWords& words, const Json& mode = Json(),
                    std::optional<std::uint32_t> rated_current = std::nullopt);

/// The directory as JSON: header and one entry per record with its register extent (info.directory_summary).
Json directory_summary(const UnitDirectory& directory);
/// One class block as JSON (ClassBlock.to_dict of the Python edition: every item with key, id, entry, index, address,
/// words, hex, unsigned, signed, scaled).
Json class_block_json(const ClassBlock& block);
/// Class-1 support words decoded with IAI's bit names plus the driver-relevant flags; null when not read.
Json decode_support_words(const std::optional<std::map<std::string, std::uint32_t>>& words);
/// SSSE b8 (the unit's calendar is in use: time stamps are seconds since 2000-01-01), nullopt when not read.
std::optional<bool> rtc_in_use(std::optional<std::uint32_t> ssse);
/// An alarm-history time stamp: {"time_raw", "clock" ("rtc" / "power_on" / null), "occurred_at",
/// "time_after_power_on"} (alarms.alarm_time). nullopt time_raw = the unit stored FFFFFFFF.
Json alarm_time(std::optional<std::int64_t> time_raw, std::optional<bool> rtc_in_use);
/// The maintenance class (0x200) decoded: total_moves (TLMC, VERIFIED-LIVE), distance_m (ODOM), calendar_time and
/// every item. `family` is the product family (a CON unit's TIMN item is shown as the calendar's time).
Json maintenance_json(const ClassBlock& block, std::optional<bool> rtc = std::nullopt,
                      const std::optional<std::string>& family = std::nullopt);
/// The alarm history as unit_info() reports it (info.decode_alarm_history), from the class-5 record and its words:
/// the used slots (up to the first empty one) and the power-up markers, and the current alarm `current` (ALMC) with its
/// level from DSS1 (ALMH / ALML) and the class-3 code `c3`; times are calendar times when `rtc_in_use`. No I/O.
Json decode_alarm_history(const IndexCategory& record, const std::vector<std::uint16_t>& words,
                          std::optional<std::uint32_t> current, std::optional<std::uint32_t> dss1,
                          std::optional<std::uint32_t> c3, std::optional<bool> absolute_encoder = std::nullopt,
                          std::optional<bool> rtc_in_use = std::nullopt, std::optional<int> system_code = std::nullopt);

/// (0.5, py-axis) The EC user parameters of the unit's list, decoded and annotated (the Python edition's
/// Elecylinder.user_parameters): every listed P-number the unit has (or every mapped one found, on a unit whose list is
/// not known), each with the decode_user_parameters fields, the per-model flags and "located": true. No I/O.
Json user_parameter_rows(const ParameterContext& ctx);

// ------------------------------------------------------------------------------------------ unit_info

/// Everything the unit tells about itself, as one JSON object. Read-only (FC03 + FC11).
///
/// `sections` limits the work to some of sections() (nullopt: all; std::invalid_argument for an unknown name). Keys
/// (a section not requested is null): meta (schema, read_only, function_codes, taken_at, driver_version, sections,
/// duration_s), identity, parameters, monitor, status_words, io, control_words, alarms, maintenance, positions and
/// errors ("section: message" for every part that could not be read). The layout of each section is the Python
/// edition's (info.unit_info docstring). A link that does not answer at all throws ModbusTimeout; once the unit has
/// answered, read problems go to errors and a later timeout ends the remaining sections.
Json unit_info(Axis& axis, const std::optional<std::vector<std::string>>& sections = std::nullopt);

// ------------------------------------------------------------------------------------------ backup

/// Which position rows a backup reads.
struct BackupRows {
    /// "used" = rows 0 .. max_positions() - 1 plus, on an EC, the simple-data rows 256-267 inside the table; "all" =
    /// every row of the directory record (an EC has 768: slow); "list" = `numbers`. Any other text is reported as a
    /// positions error.
    std::string selection = "used";
    std::vector<int> numbers;

    static BackupRows used() { return BackupRows(); }
    static BackupRows all();
    static BackupRows list(std::vector<int> numbers);
    /// "used", "all" or row numbers such as "0,1,2" (the CLI's --rows); std::invalid_argument otherwise.
    static BackupRows parse(const std::string& text);
};

/// A read-only backup of one axis as JSON (FC03 + FC11 only; nothing is written to the unit), in the Python edition's
/// schema: meta (schema kBackupSchema, read_only, function_codes, taken_at, driver_version, rows, duration_s[, path]),
/// identity (the identify() fields, slave, baudrate), directory (directory_summary: the addresses a later restore must
/// find unchanged), support (class-1 words) and accl, parameters (classes: every item of the parameter classes with
/// class, item ID, entry, address and raw value; named: the EC user parameters, or on a CON unit the numbered
/// parameters decoded for its family), positions (raw words and the decoded entry per row), alarms (the raw history
/// block and the decoded entries), maintenance (class 0x200) and errors ("section: message").
///
/// `path` (non-empty): also written there as UTF-8 JSON (the result then carries meta.path); a file that cannot be
/// written throws std::ios_base::failure. A unit that does not answer at all throws ModbusTimeout. This is the read
/// side of the rule "back up a unit before non-volatile writes" (like the PC tool's "Save all data").
Json backup(Axis& axis, const BackupRows& rows = BackupRows(), const std::string& path = std::string());

/// backup() of several axes into `directory` (created if missing), one "<name>.json" each (characters other than
/// letters, digits, '-', '_', '.' become '_'). Returns (name, file path) per axis in order, or (name, "error: <Type>:
/// <message>") for an axis that did not answer or could not be written; the other axes are still saved.
std::vector<std::pair<std::string, std::string>> backup_axes(const std::vector<std::pair<std::string, Axis*>>& axes,
                                                             const std::string& directory,
                                                             const BackupRows& rows = BackupRows());

/// Read a backup file (UTF-8 JSON). std::ios_base::failure when it cannot be read, std::invalid_argument when it is
/// not JSON.
Json load_backup(const std::string& path);

/// The identity a recovery session takes from a backup (recovery.session_from_backup): no I/O.
struct BackupIdentity {
    std::optional<std::string> serial_number;
    std::optional<std::string> model_name;
    std::optional<int> system_code;
    std::optional<int> unit_code;
    std::optional<std::string> firmware;
    std::optional<std::string> product_family;
    std::optional<Span> soft_limits_mm;
    std::optional<double> max_speed_mm_s;
    std::string units = "mm";
};
/// std::invalid_argument ("the backup holds no identity (serial number, model, system / unit code)") when none of
/// serial_number, model_name, system_code and unit_code is present.
BackupIdentity backup_identity(const Json& backup);
/// The stored rows of a backup with at least 15 words, decoded, by row number (recovery._backup_rows).
std::map<int, PositionEntry> backup_rows(const Json& backup);
/// Every parameter item of the parameter classes as "0x0008:8.15" -> upper-case hex (recovery._backup_parameters).
std::map<std::string, std::string> backup_parameters(const Json& backup);

// ------------------------------------------------------------------------------------------ helpers

/// The Python edition's class name of an error, for the "section: Type: message" texts: "ModbusTimeout",
/// "ModbusException" (a slave exception response), "SoftLimitError", ...; std::out_of_range -> "LookupError",
/// std::invalid_argument -> "ValueError", a file error -> "OSError".
std::string error_name(const std::exception& error);
/// Local time with its UTC offset, seconds resolution ("2026-10-29T14:03:12+01:00"), as the Python edition writes
/// taken_at / timestamp.
std::string local_timestamp();

// ------------------------------------------------------------------------------------------ (0.6, py-info-cli) input ranges

/// The product lines of input_ranges(), in order: EC, CON-CB, RCON, ROBONET (info.PRODUCT_LINES).
const std::vector<std::string>& product_lines();
/// The input_ranges() key for a unit (info.product_line): "RCON" / "ROBONET" behind those gateways (a gateway name
/// starting with "rcon" / "robonet", any case), "EC" for an EC, "CON-CB" for every other family (a generic IAI
/// controller included).
std::string product_line(const std::optional<std::string>& family,
                         const std::optional<std::string>& gateway = std::nullopt);
/// The EC user parameters as rows a GUI can show (info.ec_parameter_table; no I/O): no, name, manual_name, category /
/// category_text, unit, manual_range, shipping_default, driver_range (the narrower limit the driver enforces, null
/// when it has none), choices, read_only, home_parameter, iaos_only and doc; plus No.143 (overload warning level), which
/// the manual names but the driver does not map.
Json ec_parameter_table();
/// The CON parameters of a family as rows a GUI can show (con_params.parameter_table; no I/O): number, name, symbol,
/// item, unit, min / max (the family range), choices, category / category_text, factory_default, dangerous, writable,
/// note, range_doc and verified_family (false: a family that is not PCON / ACON / DCON / SCON, the PCON-CB range used).
Json con_parameter_table(const std::optional<std::string>& family);
/// The input ranges per product line, for a GUI (info.input_ranges; no I/O): per line its title, rule, moves (the
/// direct-move and row fields), parameters (ec_parameter_table() for the EC, con_parameter_table(family) and the
/// family for CON-CB, null for RCON and ROBONET) and other settings. `line`: one line's entry instead of every line;
/// std::invalid_argument for an unknown line.
Json input_ranges(const std::optional<std::string>& line = std::nullopt,
                  const std::optional<std::string>& family = std::nullopt);

}  // namespace info
}  // namespace iai_actuator_driver
