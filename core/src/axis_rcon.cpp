// The axis on a gateway line: identity without FC11 (the 0x9400 serial code behind an RCON gateway), "a directory
// timeout means no directory" (RCON / ROBONET axes: nothing else of the no-access areas is read then), the position-
// table refusal on RCON, the line's position unit (0.01 / 0.001 mm, No.207) and kind in every decoded status, and the
// line's command strategy for a new axis. Port of the Python edition's ec.py (_identify_by_serial_code,
// _directory_timeout_is_absence, _identify_without_directory, _rows_writable_here, _decode_status) and cell.Port.axis.
// Every gateway rule is DOC (RCON ME0413 p.3-1, p.5-10 to 5-23; ROBONET ME0208, ME0391) and UNVERIFIED on hardware in
// both editions; this C++ code is UNVERIFIED on hardware.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

using detail::kAxisLog;

constexpr const char* kSerialCodeSource = "serial code 0x9400";

/// A unit reader that does not send FC11 (Report Slave ID) on a line that does not document it: report_slave_id()
/// raises a ModbusError saying so (read_device_info then takes the identity from CTCL 0x0300). READ-ONLY.
class NoSlaveIdReader final : public UnitReader {
public:
    NoSlaveIdReader(UnitReader& inner, std::string line) : inner_(inner), line_(std::move(line)) {}
    int slave() const override { return inner_.slave(); }
    std::vector<std::uint16_t> read_registers(int address, int count) override {
        return inner_.read_registers(address, count);
    }
    std::vector<std::uint8_t> report_slave_id() override {
        throw ModbusError("FC11 not sent: " + line_ + " does not document Report Slave ID");
    }

private:
    UnitReader& inner_;
    std::string line_;
};

}  // namespace

// ============================================================================================ directory

bool Axis::Impl::directory_timeout_is_absence() const { return bus->line()->rules().directory_timeout_means_absent; }

bool Axis::Impl::no_directory() const {
    std::lock_guard<std::mutex> lock(mutex);
    return directory_absent;
}

void Axis::Impl::rows_writable_here(const std::string& action) {
    if (line_name() != std::optional<std::string>("rcon")) return;
    const UnitDirectory dir = ensure_directory();  // READ-ONLY; a silent directory = none on this line
    if (!dir.empty() && dir.find(reg::kPositionTableCategory) != nullptr) return;
    throw LineRuleError(action +
                        ": on an RCON line the position table is 'system, no access' (DOC ME0413 p.5-10) and this "
                        "unit's directory does not describe it: use move_to() with explicit values; nothing sent");
}

// ============================================================================================ identity

std::optional<DeviceInfo> Axis::Impl::identify_on_line(LineIdentity& prep) {
    const std::shared_ptr<Line> line = bus->line();
    const bool fc11 = line->rules().report_slave_id();
    const bool may_be_absent = line->rules().directory_timeout_means_absent;
    if (fc11 && !may_be_absent) return std::nullopt;  // a direct line: the usual identify, nothing done here
    const std::string name = line_name().value_or("this line");
    if (!fc11) {  // RCON (DOC ME0413 p.3-1): the serial code is the identity; FC11 is not sent
        prep.reader.reset(new NoSlaveIdReader(rd(), name));
        prep.serial_code = read_serial_code(*bus, slave);
        prep.errors.push_back(name + " does not document Report Slave ID (FC11): not sent; the identity is the serial "
                                     "code (0x9400); system / unit code and firmware are not available");
        if (!prep.serial_code) prep.errors.push_back("serial code 0x9400: not readable");
    }
    // Settle "is there a directory?" first: one read (with the bus's retries), not a timeout per area.
    bool absent = no_directory();
    std::string why;
    if (!absent) {
        bool known = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            known = directory && !directory->empty();
        }
        if (!known) {
            try {
                UnitDirectory dir = UnitDirectory::read(rd());
                std::lock_guard<std::mutex> lock(mutex);
                directory = dir;
            } catch (const ModbusTimeout& e) {
                why = e.what();
                log_info(kAxisLog, "index table not answered on this line: treated as absent (" + why + ")");
                std::lock_guard<std::mutex> lock(mutex);
                directory_absent = true;
                absent = true;
            } catch (const ModbusSlaveError&) {
                // not readable: read_device_info reports it on the usual path
            } catch (const std::invalid_argument&) {
            }
        }
    }
    if (!absent) return std::nullopt;

    // No directory on this line: every other area is 'system, no access' (DOC ME0413 p.5-10); nothing else is read.
    DeviceInfo out;
    out.slave = slave;
    std::vector<std::string> errors = prep.errors;
    if (fc11) {  // ROBONET through mode carries FC11 (ME0208 p.26)
        try {
            const std::vector<std::uint8_t> sid = bus->report_slave_id(slave, 0);
            out.slave_id_raw = sid;
            if (const std::optional<SlaveIdentity> id = SlaveIdentity::from_slave_id(sid)) {
                out.system_code = id->system_code;
                out.run_status = id->run_status;
                out.unit_code = id->unit_code;
                out.firmware = id->firmware;
                out.identity_source = id->source;
                std::lock_guard<std::mutex> lock(mutex);
                system_code = id->system_code;
                unit_code = id->unit_code;
                unit_code_known = true;
            } else {
                errors.push_back("report slave id: short answer");
            }
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("report slave id: not answered (") + e.what() + ")");
        } catch (const ModbusError& e) {
            errors.push_back(std::string("report slave id: ") + e.what());
        }
    } else if (prep.serial_code) {
        out.serial_number = prep.serial_code;
        out.identity_source = std::string(kSerialCodeSource);
    }
    errors.push_back("index table: not answered on this line" + (why.empty() ? std::string() : " (" + why + ")") +
                     ": treated as absent (DOC ME0413 p.5-10); the identification block, soft limits, speed limit and "
                     "stored rows are not read: pass stroke= and speeds explicitly");
    out.product_family = reg::product_family(out.system_code);
    read_soft();  // no I/O here: records "no directory on this line" as the soft-limit note
    out.soft_limits_mm.reset();
    out.max_speed_mm_s.reset();
    for (auto& e : errors) e = reg::printable(e);
    out.errors = errors;
    std::lock_guard<std::mutex> lock(mutex);
    info = out;
    return out;
}

void Axis::Impl::finish_line_identity(const LineIdentity& prep, DeviceInfo& out, std::vector<std::string>& errors) {
    if (!prep.reader) return;  // FC11 was sent as usual
    // FC11 was not sent: read_device_info's note about it gives way to the line's explanation
    errors.erase(std::remove_if(errors.begin(), errors.end(),
                                [](const std::string& e) { return util::starts_with(e, "report slave id"); }),
                 errors.end());
    errors.insert(errors.begin(), prep.errors.begin(), prep.errors.end());
    if (prep.serial_code) {
        if (!out.serial_number) out.serial_number = prep.serial_code;
        if (!out.identity_source) out.identity_source = std::string(kSerialCodeSource);
    }
}

// ============================================================================================ the line of a new axis

void Axis::Impl::apply_line(const std::shared_ptr<Line>& line) {
    if (!line) return;
    if (!line->rules().axis_access) throw UnsupportedLineError(line->explain().value_or(kRoboNetSw2));
    if (const std::optional<std::string> wanted = line->command_strategy()) {
        if (!command_method_from_string(*wanted))
            log_warning(kAxisLog, "the " + line->rules().title + " needs command strategy '" + *wanted +
                                      "', which this axis does not have");
        else
            log_debug(kAxisLog, "line " + std::string(iai_actuator_driver::to_string(line->kind())) +
                                    ": command strategy '" + *wanted + "' unless one is set (Axis::set_strategy)");
    }
}

// ============================================================================================ status decoding

Status Axis::Impl::decode_status(const std::vector<std::uint16_t>& words) {
    Status st = Status::from_registers(words);
    const std::shared_ptr<Line> line = bus->line();
    const std::optional<double> unit = line->position_unit_mm(slave);
    if (unit && std::fabs(*unit - reg::kPositionScale) > 1e-12 && words.size() >= 2) {
        // an RCON unit counting 0.001 mm (No.207, DOC ME0384 p.8-56): PNOW is in those counts
        const double wire = static_cast<double>(reg::from_i32_words(words[0], words[1]));
        st.position_mm = std::round(wire * *unit * 1000.0) / 1000.0;
    }
    st.line = line->axis_name();  // the kind, or the name stated for the axes (Line::state_rules)
    {
        // (0.5) kept for Axis::recent_statuses (a binding finds the raw words of a status it was handed)
        std::lock_guard<std::mutex> lock(mutex);
        recent_statuses.push_front(st);
        if (recent_statuses.size() > Axis::kRecentStatuses) recent_statuses.pop_back();
    }
    return st;
}

}  // namespace iai_actuator_driver
