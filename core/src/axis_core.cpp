// Axis: construction, open (baud detection in place), connect / close, properties, status (22 / 20 / 8 words, velocity
// derived from PNOW samples), identify / limits / the read-only reads, Modbus control. Port of ec.py Elecylinder
// (open, connect, status, identify, limits, enable_modbus_control; C# Axis.cs). The model layer decides (read_device_info,
// read_axis_limits, read_soft_limits, make_ranges, alarm_history_slots); this file does the I/O around it.
// This edition is UNVERIFIED on hardware; the behaviour it ports is VERIFIED in the Python edition where marked there.
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

// ============================================================================================ detail helpers

namespace detail {

std::string error_type_name(const std::exception& e) {
#define EC_TYPE(T) \
    if (dynamic_cast<const T*>(&e)) return #T
    EC_TYPE(CellError);
    if (dynamic_cast<const recovery::UnitReplacedError*>(&e)) return "UnitReplacedError";
    if (dynamic_cast<const recovery::RecoveryError*>(&e)) return "RecoveryError";
    EC_TYPE(ModbusTimeout);
    EC_TYPE(ModbusCrcError);
    EC_TYPE(ModbusFrameError);
    EC_TYPE(ModbusSlaveError);
    EC_TYPE(IaiCommandRejected);
    EC_TYPE(TransportError);
    EC_TYPE(ModbusError);
    EC_TYPE(LineRuleError);
    EC_TYPE(UnsafeCommandError);
    EC_TYPE(SoftLimitError);
    EC_TYPE(ServoBlockedError);
    EC_TYPE(GatewayModeError);
    EC_TYPE(PositionUnitError);
    EC_TYPE(UnsupportedLineError);
    EC_TYPE(EcNotReadyError);
    EC_TYPE(EcAlarmError);
    EC_TYPE(MotionTimeoutError);
    EC_TYPE(MotionIncompleteError);
    EC_TYPE(NonVolatileWriteError);
    EC_TYPE(VerificationError);
    EC_TYPE(EcError);
    EC_TYPE(EcDriverError);
#undef EC_TYPE
    if (dynamic_cast<const std::invalid_argument*>(&e)) return "invalid_argument";
    if (dynamic_cast<const std::out_of_range*>(&e)) return "out_of_range";
    if (dynamic_cast<const std::logic_error*>(&e)) return "logic_error";
    if (dynamic_cast<const std::runtime_error*>(&e)) return "runtime_error";
    return "exception";
}

std::string link_fault_kind(const std::exception& e) {
    if (dynamic_cast<const TransportError*>(&e)) return "port_vanished";
    if (dynamic_cast<const ModbusTimeout*>(&e) || dynamic_cast<const ModbusCrcError*>(&e) ||
        dynamic_cast<const ModbusFrameError*>(&e))
        return "unit_silent";
    return std::string();
}

std::string describe(const std::exception_ptr& error) {
    if (!error) return "no error";
    try {
        std::rethrow_exception(error);
    } catch (const std::exception& e) {
        return error_type_name(e) + ": " + e.what();
    } catch (...) {
        return "unknown error";
    }
}

void add_note(EcDriverError* e, const std::string& note) {
    log_info(kAxisLog, note);
    if (e) e->add_note(note);
}

std::string fmt_range(const std::optional<Span>& range, const std::string& units, const std::string& missing) {
    if (!range) return missing;
    return util::fixed(range->min) + ".." + util::fixed(range->max) + " " + units;
}

}  // namespace detail

namespace {

using detail::kAxisLog;

void validate_options(const AxisOptions& o) {
    if (o.stroke) {
        if (!std::isfinite(o.stroke->min) || !std::isfinite(o.stroke->max) || !(o.stroke->min < o.stroke->max))
            throw std::invalid_argument("invalid stroke (" + util::g(o.stroke->min) + ", " + util::g(o.stroke->max) +
                                        "): min must be below max");
    }
    if (o.max_speed_mm_s && !(*o.max_speed_mm_s > 0.0 && *o.max_speed_mm_s <= 100000.0))
        throw std::invalid_argument("max_speed_mm_s must be > 0 (or nullopt: the unit's own maximum)");
    if (o.max_accel_g && !(*o.max_accel_g > 0.0 && *o.max_accel_g <= reg::kAcmdMaxRaw * reg::kAccelScale + 1e-9))
        throw std::invalid_argument("max_accel_g must be in (0, 3.0] (or nullopt: the unit's own maximum)");
    if (o.max_push_pct && !(*o.max_push_pct > 0.0 && *o.max_push_pct <= 100.0))
        throw std::invalid_argument("max_push_pct must be in (0, 100] (or nullopt: the unit's own maximum)");
    if (!(o.poll_interval_s >= 0.0) || !std::isfinite(o.poll_interval_s))
        throw std::invalid_argument("poll_interval_s must be >= 0");
    if (!(o.settle_s >= 0.0) || !std::isfinite(o.settle_s)) throw std::invalid_argument("settle_s must be >= 0");
}

/// A model / directory error that means "not readable here" (never a link fault).
bool not_readable(const std::exception& e) {
    return dynamic_cast<const ModbusSlaveError*>(&e) || dynamic_cast<const std::out_of_range*>(&e) ||
           dynamic_cast<const std::invalid_argument*>(&e) || dynamic_cast<const EcNotReadyError*>(&e);
}

}  // namespace

// ============================================================================================ Impl: core

Axis::Impl::Impl(std::shared_ptr<ModbusRtu> bus_, int slave_, AxisOptions options_, bool owns, bool port)
    : bus(std::move(bus_)), slave(slave_), owns_bus(owns), on_port(port), options(std::move(options_)) {
    if (!bus) throw std::invalid_argument("Axis: no bus");
    if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
    if (slave == kAsciiStartSlave) throw std::invalid_argument(std::string(kAsciiStartSlaveNote) + "; nothing sent");
    validate_options(options);
    reader = bus->reader(slave);
    nv_tracker->set_label(bus->name() + " slave " + std::to_string(slave));
}

AxisOptions Axis::Impl::opts() const {
    std::lock_guard<std::mutex> lock(mutex);
    return options;
}

// inside a wait (a cancel scope; cancel.hpp) cancel_waits ends it at once, whatever poll_interval_s is
void Axis::Impl::sleep_poll() const { cancellable_sleep(opts().poll_interval_s); }

Status Axis::Impl::status() {
    int count;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!status_size_checked && directory && !directory->empty()) {
            // a unit whose directory announces a shorter monitor block (record 0x0101) is read with 20 words from the
            // start; checked once, when the directory is already known (no extra read just for this)
            status_size_checked = true;
            if (const IndexCategory* rec = directory->find(reg::kMonitorCategory)) {
                const int words =
                    reg::category_extent(rec->category_id, rec->member_count, rec->list_count, rec->member_size).words;
                if (words < status_words && words >= reg::kStatusBlockWords) status_words = reg::kStatusBlockWords;
            }
        }
        count = status_words;
    }
    std::vector<std::uint16_t> words;
    for (;;) {
        try {
            words = bus->read_registers(slave, reg::kMonitorBase, count);
            break;
        } catch (const ModbusSlaveError& e) {
            if (e.code() != 0x02 || count <= reg::kStatusMinWords) throw;
            const int smaller = count > reg::kStatusBlockWords ? reg::kStatusBlockWords : reg::kStatusMinWords;
            log_info(kAxisLog, util::format("0x9000 x%d rejected; falling back to %d words", count, smaller));
            count = smaller;
            std::lock_guard<std::mutex> lock(mutex);
            status_words = smaller;
        }
    }
    const double now = now_s();
    // PNOW in the line's position unit (an RCON unit counting 0.001 mm, No.207: 3 decimals) and the line kind ("rcon":
    // the bits RCON units leave undefined read as nullopt, status.hpp); axis_rcon.cpp
    Status st = decode_status(words);
    std::optional<std::pair<double, double>> previous;
    {
        std::lock_guard<std::mutex> lock(mutex);
        previous = last_sample;
        last_sample = std::make_pair(now, st.position_mm);
    }
    if (st.moving && previous) {
        const double dt = now - previous->first;
        if (dt >= 0.005 && dt <= 1.0) st.velocity_mm_s = util::round2((st.position_mm - previous->second) / dt);
    }
    return st;
}

UnitDirectory Axis::Impl::ensure_directory() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (directory && !directory->empty()) return *directory;
        if (directory_absent) return UnitDirectory();  // settled: no directory on this line (axis_rcon.cpp)
    }
    try {
        UnitDirectory dir = UnitDirectory::read(rd());
        std::lock_guard<std::mutex> lock(mutex);
        directory = dir;
        return dir;
    } catch (const ModbusSlaveError& e) {
        log_info(kAxisLog, std::string("index table not readable: ") + e.what());
    } catch (const std::invalid_argument& e) {
        log_info(kAxisLog, std::string("index table not readable: ") + e.what());
    } catch (const ModbusTimeout& e) {
        // RCON / ROBONET axes: a timed-out 0x0000 read means "no directory" (DOC ME0413 p.5-10; ME0391 p.1), read once
        if (!directory_timeout_is_absence()) throw;
        log_info(kAxisLog, std::string("index table not answered on this line: treated as absent (") + e.what() + ")");
        std::lock_guard<std::mutex> lock(mutex);
        directory_absent = true;
    }
    return UnitDirectory();
}

ClassBlock Axis::Impl::read_class_block(int class_id) {
    UnitDirectory dir = ensure_directory();
    if (dir.empty()) throw EcNotReadyError("this unit's index table (0x0000) is not readable");
    return dir.read_class(rd(), class_id);
}

std::optional<int> Axis::Impl::block_address(int category, int fallback) {
    UnitDirectory dir = ensure_directory();
    // no directory on this line: only the control areas (DRG1, the command profile) keep their default address; every
    // other area is 'system, no access' there (DOC ME0413 p.5-10)
    if (dir.empty() && no_directory() && category != reg::kIoControlCategory &&
        category != reg::kActiveProfileCategory)
        return std::nullopt;
    if (dir.empty()) return fallback;
    const IndexCategory* rec = dir.find(category);
    if (!rec) return std::nullopt;
    return rec->data_address;
}

std::optional<int> Axis::Impl::read_system_code() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (unit_code_known) return system_code;
    }
    std::optional<SlaveIdentity> id;
    try {
        // FC11 is not sent where the line does not document it (RCON, DOC ME0413 p.3-1): CTCL only
        if (bus->line()->rules().report_slave_id()) id = SlaveIdentity::from_slave_id(bus->report_slave_id(slave));
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        log_info(kAxisLog, std::string("report slave id not answered (") + e.what() + "): trying CTCL (0x0300)");
    }
    if (!id) {
        try {
            const ClassBlock block = read_class_block(reg::kControllerMonitorCategory);
            if (const ClassItem* item = block.item(reg::kControllerItemCtcl)) id = SlaveIdentity::from_ctcl(item->raw);
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusError& e) {
            log_info(kAxisLog, std::string("CTCL (0x0300) not readable: ") + e.what());
        } catch (const std::exception& e) {
            if (!not_readable(e)) throw;
            log_info(kAxisLog, std::string("CTCL (0x0300) not readable: ") + e.what());
        }
    }
    if (!id) return std::nullopt;  // retried at the next call
    std::lock_guard<std::mutex> lock(mutex);
    system_code = id->system_code;
    unit_code = id->unit_code;
    unit_code_known = true;
    return system_code;
}

std::optional<std::string> Axis::Impl::read_model_name() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (model_name_known) return model_name;
    }
    const std::optional<int> address = block_address(reg::kIdentificationCategory, reg::kIdentificationBase);
    if (!address) return std::nullopt;
    try {
        const auto words = bus->read_registers(slave, *address + reg::kIdentModelOffset, reg::kIdentModelWords);
        std::optional<std::string> text = reg::words_to_text(words);
        std::lock_guard<std::mutex> lock(mutex);
        model_name = text;
        model_name_known = true;
        return text;
    } catch (const ModbusSlaveError& e) {
        log_info(kAxisLog, std::string("model string not readable: ") + e.what());
    } catch (const ModbusTimeout& e) {
        log_info(kAxisLog, std::string("model string not readable: ") + e.what());
    }
    return std::nullopt;
}

bool Axis::Impl::is_ec() {
    const std::optional<int> system = read_system_code();
    return !system || reg::is_ec_family(system);
}

std::string Axis::Impl::family() { return reg::product_family(read_system_code()); }

DeviceInfo Axis::Impl::identify() {
    // gateway lines (axis_rcon.cpp): no FC11 on RCON (the 0x9400 serial code instead), a silent directory = none
    LineIdentity line_id;
    if (std::optional<DeviceInfo> early = identify_on_line(line_id)) return *early;
    std::optional<UnitDirectory> cached;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (directory && !directory->empty()) cached = directory;
    }
    DeviceInfo info_ = read_device_info(line_id.reader ? *line_id.reader : rd(), cached ? &*cached : nullptr);
    std::vector<std::string> errors = info_.errors;
    finish_line_identity(line_id, info_, errors);
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!info_.categories.empty()) directory = UnitDirectory(info_.index_header, info_.categories);
        if (info_.system_code && info_.unit_code) {
            system_code = info_.system_code;
            unit_code = info_.unit_code;
            unit_code_known = true;
        }
        bool ident_error = false;
        for (const auto& e : errors)
            if (util::starts_with(e, "identification block")) ident_error = true;
        if (!ident_error && (info_.model_name || !info_.categories.empty())) {
            model_name = info_.model_name;
            model_name_known = true;
        }
    }
    try {
        try {
            info_.soft_limits_mm = read_soft().limits;
        } catch (const ModbusTimeout& e) {
            std::lock_guard<std::mutex> lock(mutex);
            soft = SoftLimits();
            soft.note = std::string("not answered (") + e.what() + ")";
            soft_known = false;
            info_.soft_limits_mm.reset();
        }
        if (!info_.soft_limits_mm) {
            std::lock_guard<std::mutex> lock(mutex);
            errors.push_back("soft limits: " + soft.note);
        }
        std::string speed_note;
        try {
            const AxisLimits lim = read_limits();
            const auto cap = lim.unit_speed_cap();
            info_.max_speed_mm_s = cap.first;
            speed_note = cap.second;
            info_.speed_limit_source = cap.second;
            info_.max_accel_g = lim.max_accel_g;
            info_.max_decel_g = lim.max_decel_g;
            info_.actuator_kind = lim.actuator_kind;
            info_.rotary_atyp = lim.rotary_atyp;
            info_.units = lim.units();
            if (!lim.actuator_kind && lim.rotary_atyp.value_or(false))
                errors.push_back("ATYP 7.7 bit 8 (rotary axis, no AKID 7.33): positions in deg, speeds in deg/s");
            if (lim.actuator_kind) {
                const std::string name = reg::axis_kind_name(lim.actuator_kind, lim.rotary_atyp);
                const bool known = !util::starts_with(name, "kind ") && name != "unknown";
                if (known) info_.actuator_kind_name = name;
                if (lim.rotary().value_or(false))
                    errors.push_back("AKID = 1 (rotary): positions in deg, speeds in deg/s");
                else if (!known)
                    errors.push_back("AKID kind " + std::to_string(*lim.actuator_kind) +
                                     ": commanded as a linear axis in mm");
            }
        } catch (const ModbusTimeout& e) {
            speed_note = std::string("not answered (") + e.what() + ")";
            info_.max_speed_mm_s.reset();
        }
        if (!info_.max_speed_mm_s) errors.push_back("speed limit (MAXV 7.4): " + speed_note);
        try {
            info_.max_positions = max_positions();
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("stored rows (MAXP 3.2): not answered (") + e.what() + ")");
        }
    } catch (const TransportError&) {
        throw;
    } catch (const std::exception& e) {
        if (!dynamic_cast<const ModbusError*>(&e) && !not_readable(e)) throw;
        errors.push_back("identify stopped early: " + detail::error_type_name(e) + ": " + reg::printable(e.what()));
    }
    for (auto& e : errors) e = reg::printable(e);
    info_.errors = errors;
    if (!info_.product_family) info_.product_family = reg::product_family(info_.system_code);
    std::lock_guard<std::mutex> lock(mutex);
    info = info_;
    return info_;
}

SoftLimits Axis::Impl::read_soft() {
    const UnitDirectory dir = ensure_directory();
    SoftLimits s;
    if (dir.empty() && no_directory())  // nothing read: the limits area is 'system, no access' there (axis_rcon.cpp)
        s.note = "no directory on this line (the limits area is not readable): pass stroke=";
    else
        s = iai_actuator_driver::read_soft_limits(rd(), dir);
    {
        std::lock_guard<std::mutex> lock(mutex);
        soft = s;
        soft_known = true;
    }
    if (s.p1_mm) note_p1(*s.p1_mm);
    return s;
}

AxisLimits Axis::Impl::read_limits() {
    const UnitDirectory dir = ensure_directory();
    AxisLimits l = iai_actuator_driver::read_axis_limits(rd(), dir);
    std::lock_guard<std::mutex> lock(mutex);
    if (!l.max_positions && max_positions_cache) l.max_positions = max_positions_cache;
    limits = l;
    return l;
}

std::optional<AxisLimits> Axis::Impl::limits_read() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (limits) return limits;
    }
    try {
        return read_limits();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        log_info(kAxisLog, std::string("axis limits not readable: ") + e.what());
    }
    return std::nullopt;
}

void Axis::Impl::ensure_soft() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (soft_known) return;
    }
    try {
        read_soft();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        log_info(kAxisLog, std::string("soft limits not readable: ") + e.what());
    }
}

int Axis::Impl::max_positions() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (max_positions_cache) return *max_positions_cache;
    }
    const std::optional<int> system = read_system_code();
    const UnitDirectory dir = ensure_directory();
    const int rows = read_max_positions(rd(), dir, system);
    std::lock_guard<std::mutex> lock(mutex);
    max_positions_cache = rows;
    return rows;
}

RangeInputs Axis::Impl::cap_inputs() {
    RangeInputs in;
    in.limits = limits_read();
    const AxisOptions o = opts();
    in.configured_stroke = o.stroke;
    in.configured_max_speed = o.max_speed_mm_s;
    in.configured_max_accel = o.max_accel_g;
    in.configured_max_push = o.max_push_pct;
    std::lock_guard<std::mutex> lock(mutex);
    in.soft_limits = soft.limits;
    in.soft_limits_note = soft_known ? soft.note : std::string("not read yet");
    return in;
}

RangeInputs Axis::Impl::range_inputs() {
    ensure_soft();
    RangeInputs in = cap_inputs();
    in.family = family();
    try {
        in.rows = max_positions();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        log_info(kAxisLog, std::string("stored rows not readable: ") + e.what());
        in.rows = reg::kPositionActiveRows;
    }
    return in;
}

AxisRanges Axis::Impl::ranges() {
    AxisRanges r = make_ranges(range_inputs());
    try {
        const auto minimum = recommended_min_speed();
        r.speed.recommended_min = minimum.first;
        r.speed.recommended_min_source = minimum.second;
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        r.speed.recommended_min_source = std::string("not readable: ") + e.what();
    }
    r.push.speed_unit = r.units + "/s";  // the push speed entry: min(PSHV 8.15 / CON No.34, MXPH 7.8) (motion extras)
    try {
        const auto push = push_speed(std::nullopt);
        r.push.speed = push.first;
        r.push.speed_source = push.second;
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        r.push.speed_source = std::string("not readable: ") + e.what();
    }
    return r;
}

std::pair<std::optional<double>, std::string> Axis::Impl::recommended_min_speed() {
    const std::optional<AxisLimits> lim = limits_read();
    const std::optional<std::string> model = read_model_name();  // an EC model string; a CON unit keeps binary data
    std::optional<double> lead;
    if (lim && !lim->rotary().value_or(false)) lead = lim->lead_mm;
    return reg::recommended_min_speed(model, lead);
}

void Axis::Impl::warn_min_speed(double speed) {
    std::pair<std::optional<double>, std::string> minimum;
    try {
        minimum = recommended_min_speed();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError&) {
        return;
    }
    if (!minimum.first || speed >= *minimum.first - 1e-9) return;
    const std::string u = units_label();
    // never a refusal (the driver's design rule): the unit may run noisily or unsteadily below it
    log_warning(kAxisLog, util::format("speed %g %s/s is below this unit's documented minimum %g %s/s (%s): allowed, "
                                       "but it may run with abnormal noise or an unstable speed (DOC ME3793-10E "
                                       "p.1-24)",
                                       speed, u.c_str(), *minimum.first, u.c_str(), minimum.second.c_str()));
}

double Axis::Impl::speed_cap() { return iai_actuator_driver::speed_cap(cap_inputs()); }
double Axis::Impl::accel_cap() { return iai_actuator_driver::accel_cap(cap_inputs()); }
double Axis::Impl::push_cap() { return iai_actuator_driver::push_cap_pct(cap_inputs()); }

std::optional<Span> Axis::Impl::stroke_now() {
    ensure_soft();
    const AxisOptions o = opts();
    std::lock_guard<std::mutex> lock(mutex);
    return iai_actuator_driver::effective_stroke(o.stroke, unit_stroke(soft.limits));
}

Span Axis::Impl::motion_stroke() {
    limits_read();
    bool known;
    {
        std::lock_guard<std::mutex> lock(mutex);
        known = soft_known;
    }
    if (!known) read_soft();
    const AxisOptions o = opts();
    std::optional<Span> soft_limits;
    std::string note;
    {
        std::lock_guard<std::mutex> lock(mutex);
        soft_limits = soft.limits;
        note = soft.note;
    }
    const std::optional<Span> eff = iai_actuator_driver::effective_stroke(o.stroke, unit_stroke(soft_limits));
    if (eff) return *eff;
    if (!o.stroke)
        throw EcNotReadyError("stroke unknown: this unit's soft limits are not available (" + note +
                              "); configure AxisOptions::stroke = {min, max}");
    const std::string units = units_label();
    throw SoftLimitError("configured stroke " + detail::fmt_range(o.stroke, units) +
                         " does not overlap this unit's soft limits " + detail::fmt_range(soft_limits, units));
}

std::string Axis::Impl::units_label() {
    std::lock_guard<std::mutex> lock(mutex);
    return limits ? limits->units() : std::string("mm");
}

void Axis::Impl::note_p1(double value) {
    if (!std::isfinite(value)) return;
    std::lock_guard<std::mutex> lock(mutex);
    p1_ceiling_mm = p1_ceiling_mm ? std::max(*p1_ceiling_mm, value) : value;
}

std::pair<int, std::string> Axis::Impl::position_address(int row) {
    const UnitDirectory dir = ensure_directory();
    if (dir.empty()) return {reg::position_address(row), std::string()};
    const IndexCategory* rec = dir.find(reg::kPositionTableCategory);
    if (!rec) throw EcNotReadyError("this unit's directory has no position table (class 0x10)");
    const int stride = rec->member_count * rec->member_size;
    const int rows = rec->list_count != 0 ? rec->list_count : reg::kPositionMaxEntries;
    if (row < 0 || row >= rows)
        throw std::out_of_range("position number must be 0.." + std::to_string(rows - 1) + " (LQ of class 0x10), got " +
                                std::to_string(row));
    std::string problem;
    if (stride != reg::kPositionStride)
        problem = util::format("this unit's position-table rows are %d words (DQ %d x MS %d), not the known %d: the "
                               "field order is UNVERIFIED",
                               stride, rec->member_count, rec->member_size, reg::kPositionStride);
    return {rec->data_address + stride * row, problem};
}

PositionEntry Axis::Impl::read_row(int row) {
    const int address = position_address(row).first;
    return PositionEntry::from_registers(bus->read_registers(slave, address, reg::kPositionStride));
}

std::vector<AlarmRecord> Axis::Impl::alarm_history(bool include_empty) {
    const std::optional<int> address = block_address(reg::kAlarmHistoryCategory, reg::kAlarmHistoryBase);
    if (!address) throw EcNotReadyError("this unit's index table has no alarm-history category (5)");
    const auto words =
        bus->read_registers(slave, *address, reg::kAlarmHistoryStride * reg::kAlarmHistoryEntries);
    return alarm_history_slots(words, include_empty);
}

std::optional<int> Axis::Impl::read_boot_baud() {
    std::optional<int> rate;
    try {
        const ClassBlock block = read_class_block(reg::kMotionParamsCategory);
        if (const ClassItem* item = block.item(reg::kParamItemBrsl)) {
            if (item->raw <= 0xFFFF) rate = baud_by_code(static_cast<int>(item->raw));
        }
    } catch (const TransportError&) {
        throw;
    } catch (const std::exception& e) {
        if (!not_readable(e)) throw;
        log_info(kAxisLog, std::string("BRSL (item 8.3) not readable: ") + e.what());
        return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(mutex);
    boot_baud_rate = rate;
    return rate;
}

std::optional<Status> Axis::Impl::enable_control(bool verify) {
    const std::shared_ptr<Line> line = bus->line();
    line->require_control(slave);  // RCON: the gateway in MANU (DOC ME0413 p.6-1 #5); nothing sent when refused
    Status st = status();
    if (st.modbus_enabled) return st;
    if (!detail::at_rest_for_control(st))  // a pause at a standstill with the servo off is no refusal (axis_impl.hpp)
        throw EcNotReadyError(std::string("the axis is moving: switching Modbus control (PMSL) on now would stop the "
                                          "move. Wait until it is at rest and try again (") +
                              detail::kPioStopHint + ")");
    const auto auto_text = [](const Status& s) -> std::string {
        return s.manual_mode() == std::optional<bool>(false)
                   ? " The unit's mode switch is in AUTO (DSSE RMDS = 0): the PIO has control; set it to MANU (DOC "
                     "ME0162-12A p.4-20)."
                   : "";
    };
    if (!line->allows(LineOperation::Coil))
        throw LineRuleError(std::string("Modbus control is off (PMSS = 0) and ") +
                            iai_actuator_driver::to_string(line->kind()) +
                            " carries no FC05 coils, so the PMSL coil cannot be written: put the unit / gateway in "
                            "MANU (Modbus) mode." + auto_text(st) + " Nothing sent");
    try {
        send_modbus_control(*bus, slave, true);
    } catch (ModbusSlaveError& e) {
        if (coil_refused(e))  // exception 01: this unit has no coils (axis_strategy.cpp)
            e.add_note("this unit has no FC05 coils: put it in MANU (Modbus) mode by other means" + auto_text(st));
        throw;
    }
    const auto note_enabled = [this] {
        // the recovery session must know PC control is wanted again: after disable_modbus_control() it holds False, and a
        // recover() after a power cycle would not take PC control back (recovery.hpp)
        recovery::ControlChange change;
        change.modbus_control = true;
        recovery::hooks::note_control(bus, slave, change);
    };
    if (!verify) {
        note_enabled();
        return std::nullopt;
    }
    st = status();
    if (!st.modbus_enabled) {
        const std::string automatic = auto_text(st);
        throw EcNotReadyError("PMSL was written but DSSE.PMSS is still 0; the unit may be in Monitor mode (set Teach "
                              "mode)" + automatic);
    }
    note_enabled();
    return st;
}

void Axis::Impl::disable_control() {
    if (!status().idle())
        throw EcNotReadyError("the axis is moving: PMSL is never switched during motion; wait or stop() first");
    send_modbus_control(*bus, slave, false);
    recovery::ControlChange change;  // deliberate: not a restart sign for the link check (recovery.hpp)
    change.modbus_control = false;
    recovery::hooks::note_control(bus, slave, change);
}

void Axis::Impl::release_held() {
    stop_monitors();  // the status monitors first (axis_fast.cpp)
    try {
        if (stop_coil_on.load() || stop_pause_on.load()) release_stale_stop();  // a register STOP's pause too
    } catch (const std::exception& e) {
        log_warning(kAxisLog, std::string("could not release the STOP on close: ") + e.what());
    }
}

void Axis::Impl::close() {
    stop_monitors();  // the status monitors first (axis_fast.cpp)
    if (closed.exchange(true)) return;
    try {
        if (stop_coil_on.load() || stop_pause_on.load()) release_stale_stop();  // a register STOP's pause too
    } catch (const std::exception& e) {
        log_warning(kAxisLog, std::string("could not release the STOP on close: ") + e.what());
    }
    if (owns_bus) bus->close();
}

std::string Axis::Impl::describe() const {
    AxisOptions o;
    std::optional<Span> soft_limits;
    std::string units = "mm";
    {
        std::lock_guard<std::mutex> lock(mutex);
        o = options;
        soft_limits = soft.limits;
        if (limits) units = limits->units();
    }
    return "Axis(" + bus->name() + ", slave=" + std::to_string(slave) +
           ", stroke=" + detail::fmt_range(o.stroke, units, "the unit's") +
           ", soft_limits=" + detail::fmt_range(soft_limits, units) + ", method=" + iai_actuator_driver::to_string(o.method) +
           ")";
}

// ============================================================================================ Axis: construction

Axis::Axis(std::shared_ptr<ModbusRtu> bus, int slave, AxisOptions options)
    : impl_(std::make_unique<Impl>(std::move(bus), slave, std::move(options), false, false)) {
    impl_->owner = this;
}

Axis::Axis(Port& port, int slave, AxisOptions options)
    : impl_(std::make_unique<Impl>(port.bus(), slave, std::move(options), false, true)) {
    impl_->owner = this;
    // the port's line, detected once (READ-ONLY probes; axis_rcon.cpp): no axis in ROBONET gateway mode
    impl_->apply_line(port.detect_line());
}

Axis Axis::open(std::shared_ptr<Transport> transport, const OpenOptions& options) {
    if (!transport) throw std::invalid_argument("Axis::open: no transport");
    if (options.slave < 1 || options.slave > 247)
        throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(options.slave));
    if (options.slave == kAsciiStartSlave)  // before the port is touched (modbus.hpp)
        throw std::invalid_argument(std::string(kAsciiStartSlaveNote) + "; nothing sent");
    const bool has_rate = transport->baud_rate() != 0;
    if (options.baud_rate && has_rate && transport->baud_rate() != *options.baud_rate)
        transport->set_baud_rate(*options.baud_rate);
    auto bus = std::make_shared<ModbusRtu>(transport, options.bus);
    try {
        bool detected = false;
        if (!options.baud_rate && bus->can_change_baud_rate()) {
            try {
                bus->detect_baud_rate(options.slave, baud_detect_order());
            } catch (ModbusTimeout& e) {
                e.add_note(kNoAnswerHint);
                throw;
            }
            detected = true;
        }
        // the line (READ-ONLY FC03 probes, line.hpp / gateway.hpp); an unknown kind: std::invalid_argument
        const std::shared_ptr<Line> line = detect_line(*bus, parse_line_kind(options.line.kind),
                                                       kGatewayProbeTimeoutS, options.line.position_unit_mm);
        if (!line->rules().axis_access) throw UnsupportedLineError(line->explain().value_or(kRoboNetSw2));
        Axis axis(bus, options.slave, options.axis);
        axis.impl_->apply_line(line);  // the line's command strategy ("register" behind RCON; axis_rcon.cpp)
        axis.impl_->owns_bus = true;
        axis.impl_->baud_detected = detected;
        if (options.fast) axis.enable_fast();  // axis_fast.cpp (an error closes the axis and the port)
        return axis;
    } catch (...) {
        try {
            bus->close();
        } catch (...) {
        }
        throw;
    }
}

Axis Axis::open(const std::string& port_name, const OpenOptions& options) {
    if (options.slave == kAsciiStartSlave)  // refused before the port is opened (modbus.hpp)
        throw std::invalid_argument(std::string(kAsciiStartSlaveNote) + "; nothing sent");
    auto transport = std::make_shared<SerialTransport>(port_name, options.baud_rate.value_or(kDefaultBaudRate),
                                                       options.read_timeout_s);
    return open(std::static_pointer_cast<Transport>(transport), options);
}

Axis::~Axis() {
    if (!impl_) return;
    try {
        impl_->close();
    } catch (...) {
    }
}

Axis::Axis(Axis&& other) noexcept : impl_(std::move(other.impl_)) {
    if (impl_) impl_->owner = this;  // the status monitor's link watch reaches the axis through it
}

Axis& Axis::operator=(Axis&& other) noexcept {
    if (this != &other) {
        if (impl_) {
            try {
                impl_->close();
            } catch (...) {
            }
        }
        impl_ = std::move(other.impl_);
        if (impl_) impl_->owner = this;
    }
    return *this;
}

// ============================================================================================ Axis: session

DeviceInfo Axis::connect(bool take_control) {
    DeviceInfo result = impl_->identify();
    if (take_control) {
        const Status st = impl_->status();
        if (!st.modbus_enabled) {
            if (detail::at_rest_for_control(st))
                impl_->enable_control(true);
            else
                log_warning(kAxisLog, "axis is moving: Modbus control (PMSL) not taken now (it would stop the move)");
        }
    }
    return result;
}

void Axis::close() { impl_->close(); }
bool Axis::owns_bus() const { return impl_->owns_bus; }
std::shared_ptr<ModbusRtu> Axis::bus() const { return impl_->bus; }
int Axis::slave() const { return impl_->slave; }
std::optional<int> Axis::baud_rate() const { return impl_->bus->baud_rate(); }
bool Axis::baud_detected() const { return impl_->baud_detected.load(); }
AxisOptions Axis::options() const { return impl_->opts(); }

void Axis::set_options(const AxisOptions& options) {
    validate_options(options);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (options.method != impl_->options.method) impl_->strategy.reset();  // a new method replaces set_strategy()'s
    impl_->options = options;
}

std::optional<DeviceInfo> Axis::device_info() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->info;
}

std::string Axis::units() {
    const auto l = impl_->limits_read();
    return l ? l->units() : std::string("mm");
}

std::string Axis::kind() {
    const auto l = impl_->limits_read();
    return l ? l->kind() : std::string("unknown");
}

std::string Axis::product_family() { return impl_->family(); }
bool Axis::is_ec() { return impl_->is_ec(); }
std::string Axis::to_string() const { return impl_->describe(); }

// ============================================================================================ Axis: reading

Status Axis::status() { return impl_->status(); }

std::vector<std::uint16_t> Axis::read_registers(int address, int count) {
    return impl_->bus->read_registers(impl_->slave, address, count);
}

DeviceInfo Axis::identify() { return impl_->identify(); }
AxisRanges Axis::limits() { return impl_->ranges(); }
AxisLimits Axis::read_axis_limits() { return impl_->read_limits(); }
SoftLimits Axis::read_soft_limits() { return impl_->read_soft(); }
int Axis::max_positions() { return impl_->max_positions(); }
std::optional<Span> Axis::effective_stroke() { return impl_->stroke_now(); }
std::optional<int> Axis::read_boot_baud_rate() { return impl_->read_boot_baud(); }
std::vector<AlarmRecord> Axis::read_alarm_history(bool include_empty) { return impl_->alarm_history(include_empty); }
PositionEntry Axis::read_position_table(int row) { return impl_->read_row(row); }

// ============================================================================================ Axis: control

std::optional<Status> Axis::enable_modbus_control(bool verify) { return impl_->enable_control(verify); }
void Axis::disable_modbus_control() { impl_->disable_control(); }

}  // namespace iai_actuator_driver
