// Axis configuration: the position table (read, write with its guards and verify, teach by the table method; the rows
// an EC's position mode runs, P9), the
// parameters (list / read / write through the model's plan, the read-back verify and the PARW handshake), the software
// reset with its restart confirmation, and the baud change. The non-volatile write guard decides here which writes are
// skipped (results.hpp). Port of ec.py Elecylinder (write_position_table 4249, teach_position 3969, write_user_parameter /
// the PARW handshake 4401-4680, write_parameter 5087, software_reset 4799, set_baudrate 5307; C# Axis.Config.cs).
// VERIFIED in the Python edition where its documentation says so; this edition is UNVERIFIED on hardware.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace detail {
int check_baud_change(int rate);  // baud.cpp
}  // namespace detail

namespace {

using detail::kAxisLog;

/// The commit (PARW) and RAM-only notes of a parameter write (Python info.PARAM_COMMIT_NOTE / PARAM_RAM_ONLY_NOTE, the
/// same words).
constexpr const char* kParamCommitNote =
    "Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the coil IA-OS's Transfer writes (ON-WIRE: "
    "USB capture of IA-OS). PARW saves every parameter value in RAM, including earlier uncommitted writes. The new "
    "value is in the register at once, but the EC applies it only after a software reset or a power cycle "
    "(IA-OS also asks for a software reset after a parameter change).";
constexpr const char* kParamRamOnlyNote =
    "RAM ONLY - NOT SAVED (commit=False): a parameter written over Modbus (FC10) changes the EC's RAM only and "
    "reverts to the committed value at the next power-up (VERIFIED: P2 0.20 mm went back to 0.10 mm after a "
    "24 V power cycle); probably also at a software reset (untested). A later commit (PARW) saves it too.";
constexpr int kPositionModeItem = 2;  ///< EC 8.2: P9 position mode (4 = 3-position)
using detail::kRowWrittenAnyway;
/// The PMSS note of a software reset when Modbus control stayed on.
constexpr const char* kResetKeptPmssNote =
    "Modbus control (PMSS) is still 1 (kept across the reset, as on the EC-GRBP8M gripper and the PCON-CB; the "
    "EC-R6S rod dropped it to 0)";

/// An EC's stored rows 0-2 and their roles.
const char* ec_row_role(int row) {
    switch (row) {
        case 0: return "backward end";
        case 1: return "forward end";
        case 2: return "middle";
        default: return nullptr;
    }
}

/// The 16 words a write of `entry` would send (checksum included), or nullopt when it cannot be encoded (the value
/// checks of the write then name the problem).
std::optional<std::vector<std::uint16_t>> encoded_row(const PositionEntry& entry) {
    for (double v : {entry.position_mm, entry.band_mm, entry.speed_mm_s, entry.zone_plus_mm, entry.zone_minus_mm,
                     entry.accel_g, entry.decel_g})
        if (!std::isfinite(v)) return std::nullopt;
    try {
        std::vector<std::uint16_t> words = entry.to_registers();
        words.push_back(reg::position_checksum(words));
        return words;
    } catch (const std::logic_error&) {  // a value that does not fit its field
        return std::nullopt;
    }
}

std::string gs(double v) { return util::g(v); }

bool not_readable(const std::exception& e) {
    return dynamic_cast<const ModbusSlaveError*>(&e) || dynamic_cast<const std::out_of_range*>(&e) ||
           dynamic_cast<const std::invalid_argument*>(&e) || dynamic_cast<const EcNotReadyError*>(&e);
}

std::string words_text(const std::vector<std::uint16_t>& words) {
    std::string out = "[";
    for (std::size_t i = 0; i < words.size(); ++i) {
        if (i) out += ", ";
        out += std::to_string(words[i]);
    }
    return out + "]";
}

std::uint32_t raw_of(const std::vector<std::uint16_t>& words) {
    return words.size() >= 2 ? reg::from_u32_words(words[0], words[1]) : (words.empty() ? 0u : words[0]);
}

std::string raw_hex(std::uint32_t raw, int size) {
    return size >= 2 ? util::format("%08X", static_cast<unsigned>(raw)) : util::format("%04X", static_cast<unsigned>(raw));
}

}  // namespace

// ============================================================================================ position table

std::vector<std::uint16_t> Axis::Impl::validate_entry(const PositionEntry& entry, int unit_flags, int unit_push_power,
                                                      std::vector<std::string>& warnings, std::optional<int> row,
                                                      const PositionEntry* stored) {
    // The messages follow the Python edition's _validate_entry: the row, the field, its unit and whose limit it is;
    // "stored" when the value is the row's own, unchanged one (a teach keeps the row's speed).
    const std::string who = row ? "row " + std::to_string(*row) : std::string("the row");
    const auto what = [&](double PositionEntry::*field, const char* label) {
        const bool kept = stored != nullptr && stored->*field == entry.*field;
        return who + (kept ? " stored " : " ") + label;
    };
    const std::pair<const char*, double PositionEntry::*> floats[] = {
        {"position_mm", &PositionEntry::position_mm},   {"band_mm", &PositionEntry::band_mm},
        {"speed_mm_s", &PositionEntry::speed_mm_s},     {"zone_plus_mm", &PositionEntry::zone_plus_mm},
        {"zone_minus_mm", &PositionEntry::zone_minus_mm}, {"accel_g", &PositionEntry::accel_g},
        {"decel_g", &PositionEntry::decel_g}};
    for (const auto& f : floats)
        if (!std::isfinite(entry.*f.second))
            throw SoftLimitError(what(f.second, f.first) + " " + gs(entry.*f.second) + " is not a finite number");
    const Span stroke = motion_stroke();
    const std::string u = units_label();
    if (!(stroke.min <= entry.position_mm && entry.position_mm <= stroke.max))
        throw SoftLimitError(what(&PositionEntry::position_mm, "position") + " " + gs(entry.position_mm) + " " + u +
                             " is outside the stroke " + detail::fmt_range(stroke, u));
    const AxisOptions o = opts();
    const double speed = entry.speed_mm_s;
    const std::string speed_what = what(&PositionEntry::speed_mm_s, "speed");
    if (speed <= 0) throw SoftLimitError(speed_what + " " + gs(speed) + " " + u + "/s: a row needs a speed above 0");
    if (o.max_speed_mm_s && speed > *o.max_speed_mm_s + 1e-9)
        throw SoftLimitError(speed_what + " " + gs(speed) + " " + u + "/s is above this axis's max_speed_mm_s " +
                             gs(*o.max_speed_mm_s) + " " + u + "/s; open the axis with a higher max_speed_mm_s "
                             "(command line: --max-speed or --unit-max) to write it");
    try {
        check_unit_speed(speed);
    } catch (const SoftLimitError& e) {
        const bool kept = stored != nullptr && stored->speed_mm_s == speed;
        throw SoftLimitError(who + (kept ? " (its stored speed)" : "") + ": " + e.message());
    }
    const double scap = speed_cap();
    if (speed > scap + 1e-9)
        throw SoftLimitError(speed_what + " " + gs(speed) + " " + u + "/s is above " + gs(scap) + " " + u +
                             "/s: this unit's maximum speed is not readable and no max_speed_mm_s is configured");
    const std::optional<AxisLimits> lim = limits_read();
    const double accel_limit = !o.max_accel_g && !lim ? kFallbackAccelCapG : o.max_accel_g.value_or(HUGE_VAL);
    const std::pair<const char*, double PositionEntry::*> accels[] = {{"accel_g", &PositionEntry::accel_g},
                                                                      {"decel_g", &PositionEntry::decel_g}};
    for (const auto& a : accels) {
        const double value = entry.*a.second;
        if (value <= 0)
            throw SoftLimitError(what(a.second, a.first) + " " + gs(value) + " G: a row needs a value above 0");
        if (value > accel_limit)
            throw SoftLimitError(what(a.second, a.first) + " " + gs(value) + " G is above " +
                                 (o.max_accel_g ? "this axis's max_accel_g"
                                                : "the driver's limit while the unit's own maximum is not readable") +
                                 " " + gs(accel_limit) + " G; open the axis with a higher max_accel_g (command line: "
                                 "--max-accel or --unit-max) to write it");
    }
    check_unit_accel(entry.accel_g, what(&PositionEntry::accel_g, "accel_g"), false);
    if (lim && lim->max_decel_g && entry.decel_g > *lim->max_decel_g + 1e-9)
        throw SoftLimitError(what(&PositionEntry::decel_g, "decel_g") + " " + gs(entry.decel_g) +
                             " G exceeds this EC's maximum " + util::fixed(*lim->max_decel_g) + " G (MAXD 7.6)");
    if (!(entry.band_mm > 0 && entry.band_mm <= stroke.max - stroke.min))
        throw SoftLimitError(what(&PositionEntry::band_mm, "band") + " " + gs(entry.band_mm) + " " + u +
                             " is outside 0.." + gs(stroke.max - stroke.min) + " " + u + " (the stroke)");
    // Unknown bits pass when they are the unit's own, unchanged (the gripper's rows carry 0x0400); others: a warning.
    const int unknown = entry.control_flags & ~static_cast<int>(reg::kPositionCtlfKnown);
    if (unknown && unknown != (unit_flags & ~static_cast<int>(reg::kPositionCtlfKnown)))
        warnings.push_back(util::format("control flags 0x%04x use unknown bits (known: 0x%04x) that differ from the "
                                        "EC's own value; written anyway",
                                        entry.control_flags, reg::kPositionCtlfKnown));
    const double pcap = push_cap();
    const int ppow_limit = std::max(0xFF, std::min(reg::kPpowRawMax, reg::push_power(pcap)));
    if (entry.push_power < 0 || entry.push_power > ppow_limit)
        throw SoftLimitError("push_power " + std::to_string(entry.push_power) + " outside 0.." +
                             std::to_string(ppow_limit) + "; nothing written");
    if (entry.load_threshold < 0 || entry.load_threshold > 0xFF)
        throw SoftLimitError("load_threshold " + std::to_string(entry.load_threshold) +
                             " outside 0..255; nothing written");
    // The unit's own, unchanged push power is always accepted like its own flag bits (the EC-R6S rod's row 1: PPOW 26).
    const bool own_power = entry.push_power == unit_push_power && entry.push_power != 0;
    if ((entry.control_flags & reg::kCtlfPush) && !own_power) {
        if (lim && lim->push_available() == std::optional<bool>(false))
            throw SoftLimitError("this EC cannot push (SUPT PUSH = 0, MXPP = 0 or TCPC = 0); nothing written");
        const int cap_ppow = reg::push_power(pcap);
        if (entry.push_power < 1 || entry.push_power > cap_ppow)
            throw SoftLimitError("push power " + std::to_string(entry.push_power) + " outside 1.." +
                                 std::to_string(cap_ppow) + " (1.." + gs(pcap) +
                                 " %: min(max_push_pct, the EC's MXPP)) and different from the EC's own value (" +
                                 std::to_string(unit_push_power) + "); nothing written");
    }
    try {
        return entry.to_registers();
    } catch (const std::invalid_argument& e) {
        throw SoftLimitError(std::string("position entry cannot be encoded: ") + e.what() + "; nothing written");
    }
}

PositionWriteResult Axis::Impl::write_position_table(int row, const PositionEntry& entry, const NvWriteOptions& o) {
    const std::string n = std::to_string(row);
    if (row >= reg::kSimpleDataFirstRow && row <= reg::kSimpleDataLastRow)
        throw NonVolatileWriteError(util::format("rows %d-%d are the EC's internal simple-data store and are never "
                                                 "written; nothing written",
                                                 reg::kSimpleDataFirstRow, reg::kSimpleDataLastRow));
    // o.allow_nonvolatile and o.allow_unsafe are accepted and ignored (no longer needed): a row past the unit's rows,
    // one its position mode does not run, an UNVERIFIED layout, a middle-row push or changed role bits are written
    // with a warning (logged and in result.warnings); a row the table cannot address is refused.
    rows_writable_here("write_position_table");  // RCON: 'system, no access' unless the directory has it (axis_rcon.cpp)
    std::lock_guard<std::recursive_mutex> sequence(bus->sequence_mutex());
    std::pair<int, std::string> where;
    try {
        where = position_address(row);
    } catch (const std::out_of_range& e) {
        throw NonVolatileWriteError(std::string(e.what()) + "; position " + n + " not written");
    }
    const int address = where.first;
    std::vector<std::string> warnings;
    const int rows = max_positions();
    if (row >= rows)
        warnings.push_back("this unit uses position rows 0-" + std::to_string(rows - 1) +
                           " (MAXP 3.2 / the directory); row " + n + " is past them: written anyway");
    const std::string inactive = inactive_row_warning(row);  // row 2 of an EC in 2-position mode
    if (!inactive.empty()) warnings.push_back(inactive);
    if (!where.second.empty()) warnings.push_back(where.second + "; position " + n + " written anyway");
    if (row == 2 && (entry.control_flags & reg::kCtlfPush) && is_ec()) {
        bool three = false;
        try {
            const ClassBlock block = read_class_block(reg::kMotionParamsCategory);
            if (const ClassItem* item = block.item(kPositionModeItem)) three = item->raw == 4;
        } catch (const TransportError&) {
            throw;
        } catch (const std::exception& e) {
            if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
        }
        if (three)
            warnings.push_back("row 2 is the middle position of this 3-position unit: a push on the middle point is "
                               "not supported (DOC ME3837-1D p.2-5, 2-11, 2-19); written anyway");
    }
    if (row >= 0 && row < reg::kPositionMaxEntries && address != reg::position_address(row))
        log_warning(kAxisLog, util::format("position table at 0x%04X (directory), not 0x1000: UNVERIFIED layout",
                                           address - reg::kPositionStride * row));
    const Status st = status();
    if (!st.idle())
        throw EcNotReadyError("stop the axis (and resume or stop a paused move) before writing the position table; "
                              "nothing written");
    const std::vector<std::uint16_t> current = bus->read_registers(slave, address, reg::kPositionStride);
    const int old_flags = current.size() > static_cast<std::size_t>(reg::kPosCtlf) ? current[reg::kPosCtlf] : 0;
    const int old_ppow = current.size() > static_cast<std::size_t>(reg::kPosPpow) ? current[reg::kPosPpow] : 0;
    const PositionEntry stored = PositionEntry::from_registers(current);
    PositionWriteResult result;
    result.row = row;
    for (const std::string& w : warnings) log_warning(kAxisLog, "position " + n + ": " + w);
    result.warnings = warnings;
    // Non-volatile write guard: the row already holds these 16 words -> nothing written. It comes before the value
    // checks: nothing is written either way, so a row stored above this axis's caps (a 53 mm/s row on an axis opened
    // with max_speed_mm_s 10) or outside a narrowed stroke is not refused for a write that would not happen.
    if (!o.force && stored.checksum_ok() != std::optional<bool>(false)) {
        auto same = encoded_row(entry);
        if (same && !o.write_checksum) same->pop_back();  // (0.5) the 15 data words only
        if (same && same->size() <= current.size() && std::equal(same->begin(), same->end(), current.begin())) {
            log_info(kAxisLog, "position " + n + ": " + kNvUnchangedNote);
            result.entry = stored;
            result.skipped = true;
            result.note = kNvUnchangedNote;
            return result;
        }
    }
    std::vector<std::string> value_warnings;
    std::vector<std::uint16_t> words = validate_entry(entry, old_flags, old_ppow, value_warnings, row, &stored);
    if (o.write_checksum) words.push_back(reg::position_checksum(words));
    if ((old_flags ^ entry.control_flags) & reg::kPositionCtlfRoleBits)
        value_warnings.push_back(util::format("control flags 0x%04x change the unknown role bits 0x0030 of the unit's "
                                              "0x%04x (read_position_table(%d) has the unit's own); written anyway",
                                              entry.control_flags, old_flags, row));
    for (const std::string& w : value_warnings) {
        log_warning(kAxisLog, "position " + n + ": " + w);
        result.warnings.push_back(w);
    }
    if (std::vector<std::uint16_t>(current.begin(), current.begin() + std::min(words.size(), current.size())) == words) {
        // only with force (an unchanged row was skipped above)
        result.forced = true;
        result.note = kNvForcedNote;
    }
    bus->write_registers(slave, address, words);
    note_nonvolatile_write("stored row " + n);
    const std::vector<std::uint16_t> back = bus->read_registers(slave, address, reg::kPositionStride);
    const std::vector<std::uint16_t> echoed(back.begin(), back.begin() + std::min(words.size(), back.size()));
    if (echoed != words)
        throw VerificationError("position " + n + " read-back " + words_text(echoed) + " != written " + words_text(words));
    result.entry = PositionEntry::from_registers(back);
    if (result.entry.checksum_ok() != std::optional<bool>(true))
        throw VerificationError(util::format("position %d: data written, but checksum word +0xF (0x%04x) does not match "
                                             "the new data (0x%04x); the unit did not update it. %s",
                                             row, back[reg::kPosChecksum], reg::position_checksum(back),
                                             o.write_checksum ? "Restore the row from the IA-OS backup"
                                                              : "Retry with write_checksum=True or restore the entry "
                                                                "from the IA-OS backup"));
    return result;
}

PositionWriteResult Axis::Impl::teach_position(int row, const NvWriteOptions& o) {
    // o.allow_nonvolatile / o.allow_unsafe are ignored (no longer needed); a row the position mode does not run is
    // taught with write_position_table's warning
    const int rows = max_positions();
    if (row < 0 || row >= rows)
        throw SoftLimitError("row must be 0.." + std::to_string(rows - 1) + " (this unit's stored rows), got " +
                             std::to_string(row) + "; nothing written");
    const Status st = status();
    if (!st.homed)
        throw EcNotReadyError("the axis is not homed: a taught position would be meaningless; home() first; nothing "
                              "written");
    if (!st.idle()) throw EcNotReadyError("teach at rest only (the axis moves, pauses or homes); nothing written");
    PositionEntry entry = read_row(row);
    entry.position_mm = st.position_mm;
    if (entry.speed_mm_s <= 0) {
        std::optional<double> default_speed;
        try {
            const ClassBlock block = read_class_block(reg::kMotionParamsCategory);
            if (const ClassItem* item = block.item(reg::kParamItemVcmd)) {
                const double v = util::round2(item->raw * reg::kVelocityScale);
                if (v >= 0.01 && v <= 100000.0) default_speed = v;
            }
        } catch (const TransportError&) {
            throw;
        } catch (const std::exception& e) {
            if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
        }
        entry.speed_mm_s = std::min(default_speed.value_or(detail::kTeachFallbackSpeed), speed_cap());
    }
    if (entry.band_mm <= 0) {
        const std::optional<AxisLimits> lim = limits_read();
        entry.band_mm = std::max(0.10, lim && lim->min_band ? *lim->min_band : 0.0);
    }
    if (entry.accel_g <= 0) entry.accel_g = std::min(0.30, accel_cap());
    if (entry.decel_g <= 0) entry.decel_g = std::min(0.30, accel_cap());
    return write_position_table(row, entry, o);
}

// ============================================================================================ rows and the position mode
// An EC keeps three rows (backward end, forward end, middle) but runs rows 0 and 1 only in 2-position mode (P9 0 / 3):
// the EC-GRBP8M gripper in 2-position mode rejected row 2 with 0x45 cmd 6 code 0x11 (VERIFIED in the Python edition).
// max_positions() stays 3 there (rows it can read and store); moves, teach and row writes refuse an unused row.

std::optional<Axis::Impl::RowMode> Axis::Impl::active_position_mode() {
    // only on an EC known by its system code: class 8 item 2 is another parameter on a CON unit
    if (!reg::is_ec_family(read_system_code())) return std::nullopt;
    bool known;
    std::optional<std::uint32_t> code;
    {
        std::lock_guard<std::mutex> lock(mutex);
        known = position_mode_known;
        code = position_mode_raw;
    }
    if (!known) {
        bool keep = true;  // a refusal is kept; a timeout, CRC or frame error is tried again next time
        try {
            const ClassBlock block = read_class_block(reg::kMotionParamsCategory);
            if (const ClassItem* item = block.item(kPositionModeItem)) code = item->raw;
        } catch (const TransportError&) {
            throw;
        } catch (const std::exception& e) {
            if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
            keep = !dynamic_cast<const ModbusTimeout*>(&e) && !dynamic_cast<const ModbusCrcError*>(&e) &&
                   !dynamic_cast<const ModbusFrameError*>(&e);
            log_info(kAxisLog, std::string("position mode (P9) not readable: ") + e.what());
        }
        if (keep) {
            std::lock_guard<std::mutex> lock(mutex);
            position_mode_known = true;
            position_mode_raw = code;
        }
    }
    if (!code) return std::nullopt;
    // P9 decoded (0 / 3 -> 2 rows, 4 -> 3 rows); an unknown value or a model-string guess leaves the rows open
    const Json mode = info::position_mode(code);
    const Json* source = mode.find("source");
    if (!source || source->type() != Json::Type::String || !util::starts_with(source->as_string(), "user parameter 9 ="))
        return std::nullopt;
    RowMode out;
    out.rows = static_cast<int>(mode.find("positions")->as_int());
    out.code = *code;
    out.name = mode.find("name")->as_string();
    return out;
}

std::string Axis::Impl::row_label(int row) {
    bool con;
    {
        std::lock_guard<std::mutex> lock(mutex);
        con = system_code.has_value() && !reg::is_ec_family(system_code);  // no I/O: the identity already read
    }
    const char* role = con ? nullptr : ec_row_role(row);
    return "row " + std::to_string(row) + (role ? std::string(" (") + role + ")" : std::string());
}

void Axis::Impl::check_active_row(int row, const std::string& consequence) {
    const std::optional<RowMode> mode = active_position_mode();
    if (mode && row >= mode->rows) throw SoftLimitError(inactive_row_text(row, *mode) + "; " + consequence);
}

std::string Axis::Impl::inactive_row_warning(int row) {
    const std::optional<RowMode> mode = active_position_mode();
    return mode && row >= mode->rows ? inactive_row_text(row, *mode) + "; " + kRowWrittenAnyway : std::string();
}

std::string Axis::Impl::inactive_row_text(int row, const RowMode& mode) {
    std::string used;
    if (mode.rows <= 3) {
        for (int i = 0; i < mode.rows; ++i) used += (i ? " and " : "") + row_label(i);
    } else {
        used = "rows 0-" + std::to_string(mode.rows - 1);
    }
    return row_label(row) + " is not used in " + std::to_string(mode.rows) + "-position mode (P9 = " +
           std::to_string(mode.code) + ": " + mode.name + "): the unit runs " + used + " only";
}

SoftLimitError Axis::Impl::row_rejected(int row, const IaiCommandRejected& rejection) {
    const std::optional<RowMode> mode = active_position_mode();
    const std::string sent = util::format("the unit rejected it (0x45 cmd 6 code 0x%02x ", rejection.rejection()) +
                             rejection.reason() + "); nothing moved";
    if (mode && row >= mode->rows) return SoftLimitError(inactive_row_text(row, *mode) + ": " + sent);
    bool con;
    {
        std::lock_guard<std::mutex> lock(mutex);
        con = system_code.has_value() && !reg::is_ec_family(system_code);
    }
    if (con) return SoftLimitError(row_label(row) + " is not a row this unit runs: " + sent);
    return SoftLimitError(row_label(row) + " is not used in the unit's position mode: " + sent +
                          ". An EC runs rows 0 and 1 in 2-position mode (P9 0 / 3) and rows 0-2 in 3-position mode; "
                          "read P9 with read_parameter(9)");
}

// ============================================================================================ parameters

ParameterContext Axis::Impl::parameter_context(std::optional<double> position_mm) {
    ParameterContext ctx;
    ctx.is_ec = is_ec();
    if (ctx.is_ec) ensure_soft();  // the P1 ceiling (P1 can only narrow)
    limits_read();
    ctx.units = units_label();
    ctx.family = family();
    const UnitDirectory dir = ensure_directory();
    if (!dir.empty()) {
        for (int ci : parameter_classes_needed(ctx.is_ec)) {
            try {
                ctx.classes[ci] = dir.read_class(rd(), ci);
            } catch (const TransportError&) {
                throw;
            } catch (const ModbusTimeout&) {
                throw;
            } catch (const std::exception& e) {
                if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
                log_info(kAxisLog, util::format("parameter class 0x%04X not readable: ", ci) + e.what());
            }
        }
        try {
            ctx.support = dir.read_support_words(rd());
        } catch (const ModbusSlaveError& e) {
            log_info(kAxisLog, std::string("support words not readable: ") + e.what());
        }
        try {
            ctx.accl = dir.read_accl(rd());
        } catch (const ModbusSlaveError& e) {
            log_info(kAxisLog, std::string("ACCL (0x0612) not readable: ") + e.what());
        }
    }
    ctx.system_code = read_system_code();
    ctx.model_name = read_model_name();
    {
        std::lock_guard<std::mutex> lock(mutex);
        ctx.unit_code = unit_code;
        ctx.p1_ceiling_mm = p1_ceiling_mm;
    }
    ctx.position_mm = position_mm;
    return ctx;
}

std::vector<ParameterInfo> Axis::Impl::parameters() {
    const ParameterContext ctx = parameter_context(std::nullopt);
    std::vector<ParameterInfo> list = list_parameters(ctx);
    for (const auto& p : list)
        if (ctx.is_ec && p.number == 1 && p.value.number) note_p1(*p.value.number);
    return list;
}

ParameterInfo Axis::Impl::read_parameter(const ParameterKey& key) {
    ParameterContext ctx = parameter_context(std::nullopt);
    if (!key.number && !ctx.classes.count(key.class_id)) {
        // a raw item of another class (e.g. 1.5): read that class on demand, as the Python edition does
        const UnitDirectory dir = ensure_directory();
        if (dir.find(key.class_id) != nullptr) ctx.classes[key.class_id] = dir.read_class(rd(), key.class_id);
    }
    return describe_parameter(ctx, key);
}

ParameterModel Axis::Impl::parameter_model() { return model_parameter_list(parameter_context(std::nullopt)); }

std::pair<int, int> Axis::Impl::parameter_handshake() {
    const int base = block_address(reg::kIoControlCategory, reg::kDrg1).value_or(reg::kDrg1);
    const int drge = bus->read_registers(slave, base + (reg::kDrge - reg::kDrg1), 1).at(0);
    const int dsse = bus->read_registers(slave, reg::kDsse, 1).at(0);
    return {drge, dsse};
}

namespace {
bool parameter_idle(int drge, int dsse) { return !(drge & reg::kDrgeParw) && (dsse & reg::kDsseParw); }
}  // namespace

void Axis::Impl::wait_parameter_idle(const std::string& consequence) {
    const double deadline = now_s() + detail::parw_ready_s();
    for (;;) {
        const auto hs = parameter_handshake();
        if (parameter_idle(hs.first, hs.second)) return;
        if (now_s() >= deadline)
            throw EcNotReadyError(util::format("a parameter write is still in progress after %g s (DRGE 0x%04X b7 PARW "
                                               "= %d, DSSE 0x%04X b7 = %d; idle is DRGE b7 = 0 and DSSE b7 = 1): ",
                                               detail::parw_ready_s(), hs.first, (hs.first & reg::kDrgeParw) ? 1 : 0, hs.second,
                                               (hs.second & reg::kDsseParw) ? 1 : 0) +
                                  consequence);
        sleep_poll();
    }
}

std::pair<bool, std::string> Axis::Impl::commit_parameters() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        ctlf_item_known = false;  // parameter values read for motion checks are stale now
        position_mode_known = false;
    }
    wait_parameter_idle("nothing committed (the item write is in RAM only)");
    send_parameter_commit(*bus, slave);
    note_nonvolatile_write("parameter commit (PARW)");
    const double started = now_s();
    const double deadline = started + detail::parw_clear_s();
    bool saw_parw = false;
    bool saw_dsse_busy = false;
    for (;;) {
        const auto hs = parameter_handshake();
        saw_parw = saw_parw || (hs.first & reg::kDrgeParw);
        saw_dsse_busy = saw_dsse_busy || !(hs.second & reg::kDsseParw);
        const double now = now_s();
        if (parameter_idle(hs.first, hs.second)) {
            if (saw_parw || saw_dsse_busy) {
                std::vector<std::string> seen;
                if (saw_parw) seen.push_back("DRGE b7 set");
                if (saw_dsse_busy) seen.push_back("DSSE b7 = 0");
                nv()->set_uncommitted(false);  // PARW saved every parameter in RAM
                return {true, "commit confirmed: " + util::join(seen, " and ") +
                                  util::format(" after the coil, idle again after %.3f s", now - started)};
            }
            if (now - started >= kParwObserveS)
                return {false, util::format("commit NOT confirmed: after the PARW coil (0x%04X) DRGE b7 never read 1 "
                                            "and DSSE b7 never read 0 within %g s, so the unit may not have latched it "
                                            "(b7 = 0 cannot tell 'finished' from 'never set'). The value may be RAM "
                                            "only: check it after a power cycle, or write it again",
                                            reg::kCoilParw, kParwObserveS)};
        } else if (now >= deadline) {
            throw VerificationError(util::format("PARW (DRGE b7, coil 0x%04X) handshake not finished %g s after the "
                                                 "commit (DRGE 0x%04X, DSSE 0x%04X; idle is DRGE b7 = 0 and DSSE b7 = "
                                                 "1): the commit is not confirmed. Do not reset or power off the unit "
                                                 "until DRGE b7 reads 0",
                                                 reg::kCoilParw, detail::parw_clear_s(), hs.first, hs.second));
        }
        sleep_poll();
    }
}

ParameterWriteResult Axis::Impl::write_parameter(const ParameterKey& key, const ParamValue& value,
                                                 const ParameterWriteOptions& o) {
    static const char* const kAtRest =
        "parameters are written only while the axis is at rest (not moving, pushing, paused or homing); nothing "
        "written";
    std::lock_guard<std::recursive_mutex> sequence(bus->sequence_mutex());
    Status st = status();
    if (!st.idle() || st.pushing()) throw EcNotReadyError(kAtRest);  // fail fast, before the tables are read
    const ParameterContext ctx = parameter_context(st.position_mm);
    const ParameterWritePlan plan = plan_parameter_write(ctx, key, value, o);  // every guard; nothing sent
    st = status();
    if (!st.idle() || st.pushing()) throw EcNotReadyError(kAtRest);
    const bool changed = plan.changed();
    const bool pending = nv()->uncommitted();
    // The non-volatile write guard (the Python edition's): the item is written only when its value changes; the commit
    // is skipped when the value is unchanged and no parameter written to RAM on this unit by this process waits for a
    // commit (a commit = false write, or a commit that failed or was not confirmed), unless force.
    const bool write_item = changed;
    const bool skipped = o.commit && !changed && !o.force && !pending;
    const bool commit = o.commit && !skipped;
    const bool forced = !changed && o.force;
    if (commit) {
        st = require_control(st);  // the PARW coil needs Modbus control; taken at rest only
        wait_parameter_idle("nothing written");  // no write while a commit is running
    }
    const std::vector<std::uint16_t> words = plan.words();
    if (write_item) {
        log_info(kAxisLog, (plan.number ? "parameter " + std::to_string(plan.number) : "item " + plan.key) + " " +
                               plan.name + ": write " + plan.before + " -> " + plan.after + " (" +
                               (o.commit ? "commit follows" : "RAM only") + ")");
        nv()->set_uncommitted(true);  // until a confirmed commit: a failed one leaves it waiting (PARW saves all RAM)
        if (words.size() == 1)
            bus->write_register(slave, plan.address, words[0]);
        else
            bus->write_registers(slave, plan.address, words);
    } else if (skipped) {
        log_info(kAxisLog, plan.key + " (" + plan.name + ") = " + plan.before + ": " + kNvUnchangedNote);
    }
    const std::vector<std::uint16_t> back =
        bus->read_registers(slave, plan.address, static_cast<int>(words.size()));
    if (back != words)
        throw VerificationError(util::format("%s @0x%04X: read back %s, wrote %s (the unit did not take the write); "
                                             "nothing committed",
                                             plan.key.c_str(), plan.address,
                                             raw_hex(raw_of(back), plan.size).c_str(),
                                             raw_hex(plan.raw_after, plan.size).c_str()));
    ParameterWriteResult result;
    result.number = plan.number;
    result.name = plan.name;
    result.key = plan.key;
    result.address = util::format("0x%04X", plan.address);
    result.unit = plan.unit;
    result.before = plan.before;
    result.after = plan.after;
    result.value = plan.value;
    result.raw_before = raw_hex(plan.raw_before, plan.size);
    result.raw_after = raw_hex(raw_of(back), plan.size);
    result.changed = changed;
    result.verified = true;
    result.listed_for_model = plan.listed_for_model;
    result.verified_for_model = plan.verified_for_model;
    result.home_parameter = plan.home_parameter;
    result.skipped = skipped;
    result.forced = forced;
    result.warnings = plan.warnings;
    if (commit) {
        const auto done = commit_parameters();
        result.committed = true;
        result.commit_confirmed = done.first;
        if (!done.first) result.warnings.push_back(done.second);
    }
    if (plan.rereads_soft_limits && changed) {
        try {
            read_soft();  // the driver enforces the new (narrower) stroke at once
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusError& e) {
            result.warnings.push_back(std::string("soft limits not re-read after the write: ") + e.what());
        }
    }
    if (!ctx.is_ec && !skipped)
        result.warnings.push_back("the unit applies a committed parameter after a software reset or a power cycle "
                                  "(VERIFIED on a PCON-CB)");
    // the Python edition's notes: skipped, else what was done with the value
    // (a commit = false write of an unchanged value sent nothing: kRamUnchangedNote, not the RAM-only warning)
    result.note = skipped ? kNvUnchangedNote
                  : o.commit ? kParamCommitNote
                  : changed  ? kParamRamOnlyNote
                             : kRamUnchangedNote;
    return result;
}

// ============================================================================================ software reset

bool Axis::Impl::watch_reset_gap(double window_s) {
    const double deadline = now_s() + window_s;
    while (now_s() < deadline) {
        try {
            bus->read_registers(slave, reg::kPnow, 1, 0);
        } catch (const ModbusTimeout&) {
            return true;
        } catch (const ModbusCrcError&) {
            return true;
        } catch (const ModbusFrameError&) {
            return true;
        } catch (const ModbusSlaveError&) {
        }
        sleep_at_least(detail::kResetGapProbeS);
    }
    return false;
}

SoftwareResetResult Axis::Impl::software_reset(const SoftwareResetOptions& o) {
    // a gateway line carries no 0x45 (line.hpp): refused before anything (the servo off included) is sent
    bus->line()->require(LineOperation::SoftwareReset, "software_reset (0x45 cmd 9 / cmd 5)");
    const BusSuspension quiet = suspend_bus(*bus);  // every status monitor of this port (monitor.hpp)
    const double started = now_s();
    Status st = status();
    if (!st.idle() || st.pushing())
        throw EcNotReadyError("software reset only at rest (not moving, pushing, paused or homing): the reset switches "
                              "the servo off first. Wait for the move to finish or stop() it; nothing sent");
    std::lock_guard<std::recursive_mutex> sequence(bus->sequence_mutex());
    {
        std::lock_guard<std::mutex> lock(mutex);
        ctlf_item_known = false;  // committed parameters apply after the reset: read them again
        position_mode_known = false;
    }
    // A commit (PARW) still writing non-volatile memory must not be cut off.
    wait_parameter_idle("software reset not sent (it could interrupt the non-volatile write)");
    const bool con_unit = !is_ec();  // read before the reset: the unit is silent afterwards
    st = require_control(st);
    const std::optional<double> uptime_before = st.uptime_s();
    bool boot_known;
    {
        std::lock_guard<std::mutex> lock(mutex);
        boot_known = boot_baud_rate.has_value();
    }
    if (!boot_known && bus->can_change_baud_rate()) read_boot_baud();  // the unit may come back at its power-up rate
    release_stale_stop();
    // 1. Servo OFF with 0x45 cmd 9 op1 0 (ON-WIRE), and wait until DSS1 shows it.
    send_servo(*bus, slave, false, CommandMethod::Iai);
    const double servo_deadline = now_s() + kResetServoOffS;
    while (st.servo_on) {
        if (now_s() >= servo_deadline)
            throw EcNotReadyError("servo OFF (0x45 cmd 9) did not show in DSS1 within " + gs(kResetServoOffS) +
                                  " s; software reset not sent");
        sleep_poll();
        st = status();
    }
    // 2. Software reset (0x45 cmd 5 op1 1), sent once.
    SoftwareResetResult result;
    result.uptime_before_s = uptime_before;
    try {
        send_software_reset(*bus, slave);
        result.accepted = true;
    } catch (IaiCommandRejected& e) {
        e.add_note("the servo is OFF now (switched off before the reset); servo(true) turns it on");
        throw;
    } catch (const ModbusTimeout& e) {
        result.reply_lost = true;
        result.notes.push_back(std::string("no complete answer to the reset command (") + e.what() +
                               "); reset confirmed from the status instead");
    } catch (const ModbusCrcError& e) {
        result.reply_lost = true;
        result.notes.push_back(std::string("no complete answer to the reset command (") + e.what() +
                               "); reset confirmed from the status instead");
    } catch (const ModbusFrameError& e) {
        result.reply_lost = true;
        result.notes.push_back(std::string("no complete answer to the reset command (") + e.what() +
                               "); reset confirmed from the status instead");
    }
    stop_coil_on = false;  // the unit's control state starts afresh
    motion_pending = false;
    nv()->set_uncommitted(false);  // the unit reloads its parameters: RAM-only writes are gone
    {
        std::lock_guard<std::mutex> lock(mutex);
        last_sample.reset();
        last_move.reset();
        last_target.reset();
        limits.reset();
        soft_known = false;
    }
    // 3. Watch for the reply gap, then re-sync the link in place (never close / reopen).
    const std::optional<int> baud_before = bus->baud_rate();
    const double watch_started = now_s();
    bool gap_seen = false;
    if (o.reset_wait_s > 0 && con_unit) gap_seen = watch_reset_gap(std::min(o.reset_wait_s, 1.5));
    sleep_at_least(o.reset_wait_s - (now_s() - watch_started));
    const double deadline = now_s() + o.timeout_s;
    for (;;) {
        try {
            if (bus->can_change_baud_rate()) {
                std::optional<int> boot;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    boot = boot_baud_rate;
                }
                if (boot) {
                    std::vector<int> order{*boot};
                    if (baud_before) order.push_back(*baud_before);
                    for (int r : baud_detect_order()) order.push_back(r);
                    std::vector<int> unique;
                    for (int r : order)
                        if (std::find(unique.begin(), unique.end(), r) == unique.end()) unique.push_back(r);
                    bus->detect_baud_rate(slave, unique, reg::kPnow, false);
                } else {
                    bus->detect_baud_rate(slave, baud_detect_order());
                }
            } else {
                bus->read_registers(slave, reg::kPnow, 1, 0);
            }
            break;
        } catch (ModbusError& e) {
            if (!dynamic_cast<ModbusTimeout*>(&e) && !dynamic_cast<ModbusCrcError*>(&e) &&
                !dynamic_cast<ModbusFrameError*>(&e))
                throw;
            gap_seen = gap_seen || !bus->can_change_baud_rate();  // silent at the unchanged rate: restarting
            if (now_s() >= deadline) {
                e.add_note("the unit did not answer within " + gs(o.timeout_s) +
                           " s after the software reset (the servo is off); check the 24 V supply and reconnect");
                throw;
            }
            sleep_at_least(std::max(opts().poll_interval_s, 0.05));
        }
    }
    st = status();
    std::vector<std::string> signs;
    if (!st.modbus_enabled) signs.push_back("PMSS 1 -> 0");
    if (gap_seen) signs.push_back("the unit was silent after the command");
    const std::optional<double> uptime_now = st.uptime_s();
    if (uptime_before && uptime_now && *uptime_now < *uptime_before) signs.push_back("the uptime counter restarted");
    if (!st.homed) signs.push_back("HEND 0");
    const std::optional<int> baud_after = bus->baud_rate();
    if (baud_before && baud_after != baud_before) signs.push_back("the baud rate changed");
    const double servo_on_deadline = now_s() + kResetServoOnS;
    while (!st.servo_on && !st.alarm && now_s() < servo_on_deadline) {
        sleep_poll();
        st = status();
    }
    // the servo was switched off before the reset: only a restart brings it back by itself
    if (st.servo_on) signs.push_back("the servo came back on by itself");
    result.confirmed = !signs.empty();
    result.confirmed_by = util::join(signs, ", ");
    if (!result.confirmed) {
        result.notes.push_back(std::string("restart NOT confirmed: PMSS stayed 1, no reply gap, uptime, HEND and servo "
                                           "unchanged. The command was ") +
                               (result.accepted ? "accepted" : "sent") +
                               "; a committed parameter shows whether the reset took effect");
        log_warning(kAxisLog, "software reset: restart not confirmed (no PMSS drop, no reply gap, servo still off)");
    }
    try {
        read_soft();  // a committed P1 applies now
        read_limits();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        result.notes.push_back(std::string("limits not re-read after the reset: ") + e.what());
    }
    if (baud_after != baud_before) {
        result.notes.push_back("the unit answered at " + (baud_after ? std::to_string(*baud_after) : std::string("?")) +
                               " baud after the reset (was " +
                               (baud_before ? std::to_string(*baud_before) : std::string("?")) + ")");
        baud_detected = true;
    }
    result.notes.push_back("the uptime counter 0x9010 is not reset by a software reset (only by a power-up)");
    if (!st.homed)
        result.notes.push_back(std::string("HEND = 0: home lost (an incremental unit loses it at every software reset; "
                                           "an absolute unit after a home-parameter change). Run home() before the "
                                           "next move") +
                               (st.alarm ? " - reset_alarm() first" : ""));
    if (st.alarm)
        result.notes.push_back(util::format("alarm 0x%03x (", st.alarm_code) + alarm_name(st.alarm_code) +
                               ") after the reset");
    result.notes.push_back(st.modbus_enabled ? kResetKeptPmssNote
                                             : "Modbus control (PMSS) is 0 again, like after a power-up: the next "
                                               "command takes it at rest");
    {
        recovery::ControlChange change;  // deliberate: not a restart sign for the link check (recovery.hpp)
        change.modbus_control = st.modbus_enabled;
        change.homed = st.homed;
        change.baud_rate = baud_after;
        recovery::hooks::note_control(bus, slave, change);
    }
    result.baud_rate = baud_after;
    result.baud_changed = baud_after != baud_before;
    result.status = st;
    result.uptime_after_s = st.uptime_s();
    result.elapsed_s = std::round((now_s() - started) * 1000.0) / 1000.0;
    return result;
}

// ============================================================================================ baud rate

BaudChangeResult Axis::Impl::set_baud_rate(int rate, const BaudChangeOptions& o) {
    // nothing is broadcast behind a gateway (line.hpp): refused before anything is sent
    bus->line()->require(LineOperation::BaudChange, "set_baudrate (broadcast 0x45 cmd 4)");
    bus->line()->require(LineOperation::IaiCommand, "set_baudrate (broadcast 0x45 cmd 4)");
    const BusSuspension quiet = suspend_bus(*bus);  // every status monitor of this port (monitor.hpp)
    detail::check_baud_change(rate);  // the refusal of every baud change (baud.cpp)
    if (!bus->can_change_baud_rate())
        throw EcNotReadyError("this transport (" + bus->name() + ") has no baud rate to change; nothing sent");
    if (on_port)
        throw EcNotReadyError("this axis is on a shared Port: a baud change reaches every unit on the line; use "
                              "Port::set_baud_rate; nothing sent");
    std::lock_guard<std::recursive_mutex> sequence(bus->sequence_mutex());
    const Status st = status();
    if (!st.idle() || st.pushing())
        throw EcNotReadyError("baud change only at rest (not moving, pushing, paused or homing); nothing sent");
    log_info(kAxisLog, util::format("changing the baud rate %d -> %d (%s, broadcast 0x45 cmd 4)",
                                    bus->baud_rate().value_or(0), rate, iai_actuator_driver::to_string(o.method)));
    BaudChangeResult result = change_baud_rate(*bus, rate, {slave}, o);
    if (result.baud_rate != rate)
        throw VerificationError(util::format("baud change to %d (code 0x%02X) not taken: the unit %s at %d baud (the "
                                             "port is left at %d)%s",
                                             rate, result.code, result.baud_rate == result.previous ? "still answers"
                                                                                                    : "answers",
                                             result.baud_rate, result.baud_rate,
                                             baud_not_taken_hint(rate, result.baud_rate, result.previous).c_str()));
    baud_detected = false;
    return result;
}

// ============================================================================================ Axis: public forwarding

PositionWriteResult Axis::write_position_table(int row, const PositionEntry& entry, const NvWriteOptions& options) {
    return impl_->write_position_table(row, entry, options);
}

PositionWriteResult Axis::teach_position(int row, const NvWriteOptions& options) {
    return impl_->teach_position(row, options);
}

std::vector<ParameterInfo> Axis::parameters() { return impl_->parameters(); }

namespace {
ParameterKey parse_key(const std::string& key) {
    try {
        return ParameterKey::parse(key);
    } catch (const std::invalid_argument& e) {
        throw UnsafeCommandError("parameter key '" + key + "': " + e.what() + "; nothing sent");
    }
}
}  // namespace

ParameterInfo Axis::read_parameter(const std::string& key) { return impl_->read_parameter(parse_key(key)); }

ParameterInfo Axis::read_parameter(int number) { return impl_->read_parameter(ParameterKey::of(number)); }

ParameterWriteResult Axis::write_parameter(const std::string& key, const ParamValue& value,
                                           const ParameterWriteOptions& options) {
    return impl_->write_parameter(parse_key(key), value, options);
}

ParameterWriteResult Axis::write_parameter(int number, double value, const ParameterWriteOptions& options) {
    return impl_->write_parameter(ParameterKey::of(number), ParamValue::of(value), options);
}

ParameterModel Axis::parameter_model() { return impl_->parameter_model(); }

SoftwareResetResult Axis::software_reset(const SoftwareResetOptions& options) { return impl_->software_reset(options); }

BaudChangeResult Axis::set_baud_rate(int rate, const BaudChangeOptions& options) {
    return impl_->set_baud_rate(rate, options);
}

}  // namespace iai_actuator_driver
