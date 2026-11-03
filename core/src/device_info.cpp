// What a unit tells about itself: the FC11 identity (with the CTCL fallback), the index table, the identification block,
// the CON controller name and AXNO. Port of Axis.identify (Python ec.py, the identity part; C# DeviceInfo.cs). READ-ONLY:
// every read goes through the UnitReader (FC03 and FC11). Parts the unit rejects or does not answer are collected in
// DeviceInfo::errors, never thrown; only a TransportError (the port itself failed) propagates.
#include <memory>
#include <stdexcept>
#include <typeinfo>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

const char* const kLogSource = "iai_actuator_driver.device_info";

/// (system, unit) codes of IAI units that are not EC actuators.
const char* not_an_actuator(int system, int unit) {
    if (system == 0xEC && unit == 0x10) return "CCM conveyor control motor (not an EC actuator)";
    return nullptr;
}

std::string hex_text(const std::vector<std::uint8_t>& bytes) {
    std::string text;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) text += ' ';
        text += util::format("%02x", bytes[i]);
    }
    return text;
}

std::string error_type_name(const std::exception& e) {
    if (dynamic_cast<const ModbusTimeout*>(&e)) return "ModbusTimeout";
    if (dynamic_cast<const ModbusCrcError*>(&e)) return "ModbusCrcError";
    if (dynamic_cast<const ModbusFrameError*>(&e)) return "ModbusFrameError";
    if (dynamic_cast<const ModbusSlaveError*>(&e)) return "ModbusSlaveError";
    if (dynamic_cast<const ModbusError*>(&e)) return "ModbusError";
    if (dynamic_cast<const std::out_of_range*>(&e)) return "out_of_range";
    if (dynamic_cast<const std::invalid_argument*>(&e)) return "invalid_argument";
    return "error";
}

/// Data address of a category: the default when the directory is unknown (empty), nullopt when the unit has no such one.
std::optional<int> block_address(const UnitDirectory& directory, int category_id, int fallback) {
    if (directory.empty()) return fallback;
    const IndexCategory* rec = directory.find(category_id);
    if (rec == nullptr) return std::nullopt;
    return rec->data_address;
}

/// CTCL (class 3 item 1, 0x0300 on all units tested): system << 24 | unit << 16 | firmware (VERIFIED equal to FC11).
std::optional<std::uint32_t> read_ctcl(UnitReader& reader, const UnitDirectory& directory) {
    const IndexCategory* rec = directory.find(reg::kControllerMonitorCategory);
    if (rec == nullptr || rec->member_size != 2) return std::nullopt;
    try {
        if (rec->id_list_address != 0) {
            const std::vector<std::uint16_t> ids = reader.read_registers(rec->id_list_address, 2);
            if (!ids.empty() && ids[0] != reg::kControllerItemCtcl) {
                const ClassBlock block = directory.read_class(reader, reg::kControllerMonitorCategory);
                const ClassItem* item = block.item(reg::kControllerItemCtcl);
                if (item == nullptr) return std::nullopt;
                return item->raw;
            }
        }
        const std::vector<std::uint16_t> words = reader.read_registers(rec->data_address, 2);
        if (words.size() < 2) return std::nullopt;
        const std::uint32_t value = reg::from_u32_words(words[0], words[1]);
        return (value >> 24) != 0 ? std::optional<std::uint32_t>(value) : std::nullopt;
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusTimeout& e) {
        log_info(kLogSource, std::string("CTCL (0x0300) not readable: ") + e.what());
    } catch (const ModbusSlaveError& e) {
        log_info(kLogSource, std::string("CTCL (0x0300) not readable: ") + e.what());
    } catch (const std::out_of_range& e) {
        log_info(kLogSource, std::string("CTCL (0x0300) not readable: ") + e.what());
    } catch (const std::invalid_argument& e) {
        log_info(kLogSource, std::string("CTCL (0x0300) not readable: ") + e.what());
    }
    return std::nullopt;
}

/// The controller name block of a CON controller (class 0x0100 at 0x8010, e.g. "PCON" / "CB" / "(FB)" -> "PCON-CB (FB)";
/// the split of the three fields is UNVERIFIED). nullopt on an EC or when not readable / not text.
std::optional<std::string> read_controller_name(UnitReader& reader, std::optional<int> system_code,
                                                const UnitDirectory& directory) {
    if (!system_code || reg::is_ec_family(system_code) || directory.empty()) return std::nullopt;
    const IndexCategory* rec = directory.find(reg::kSystemCategory);
    if (rec == nullptr) return std::nullopt;
    const int address = rec->data_address + reg::kControllerNameOffset;
    std::vector<std::uint16_t> words;
    try {
        words = reader.read_registers(address, reg::kControllerNameWords);
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusTimeout& e) {
        log_info(kLogSource, util::format("controller name (0x%04X) not readable: ", address) + e.what());
        return std::nullopt;
    } catch (const ModbusSlaveError& e) {
        log_info(kLogSource, util::format("controller name (0x%04X) not readable: ", address) + e.what());
        return std::nullopt;
    }
    if (words.size() < 8) return std::nullopt;
    const auto name = reg::words_to_text({words.begin(), words.begin() + 4});
    if (!name) return std::nullopt;
    const auto type = reg::words_to_text({words.begin() + 4, words.begin() + 6});
    const auto option = reg::words_to_text({words.begin() + 6, words.begin() + 8});
    const std::string text = reg::printable(*name + (type ? "-" + *type : std::string()) + (option ? " " + *option : std::string()));
    if (text.empty()) return std::nullopt;
    return text;
}

/// AXNO (u32 at 0x8004, class 0x0100 member 3; read-only): the axis number the unit believes it has.
std::optional<int> read_axno(UnitReader& reader, const UnitDirectory& directory) {
    if (directory.empty()) return std::nullopt;
    const std::optional<int> base = block_address(directory, reg::kSystemCategory, 0x8000);
    if (!base) return std::nullopt;
    const int address = *base + reg::kAxnoOffset;
    try {
        const std::vector<std::uint16_t> words = reader.read_registers(address, 2);
        if (words.size() < 2) return std::nullopt;
        return static_cast<int>(reg::from_u32_words(words[0], words[1]));
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusTimeout& e) {
        log_info(kLogSource, util::format("AXNO (0x%04X) not readable: ", address) + e.what());
    } catch (const ModbusSlaveError& e) {
        log_info(kLogSource, util::format("AXNO (0x%04X) not readable: ", address) + e.what());
    }
    return std::nullopt;
}

/// FC11 once (the bus waits kSlaveIdTimeoutS); one more try after a damaged answer (FC11 is read-only).
void read_identity(UnitReader& reader, DeviceInfo& info, std::vector<std::string>& errors,
                   std::optional<SlaveIdentity>& identity) {
    auto attempt = [&]() {
        const std::vector<std::uint8_t> raw = reader.report_slave_id();
        info.slave_id_raw = raw;
        identity = SlaveIdentity::from_slave_id(raw);
        if (!identity) errors.push_back("report slave id: short answer " + hex_text(raw));
    };
    try {
        attempt();
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusCrcError&) {
        try {
            attempt();
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("report slave id: not answered (") + e.what() + ")");
        } catch (const ModbusError& e) {
            errors.push_back("report slave id: " + error_type_name(e) + ": " + e.what());
        }
    } catch (const ModbusFrameError&) {
        try {
            attempt();
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("report slave id: not answered (") + e.what() + ")");
        } catch (const ModbusError& e) {
            errors.push_back("report slave id: " + error_type_name(e) + ": " + e.what());
        }
    } catch (const ModbusTimeout& e) {
        errors.push_back(std::string("report slave id: not answered (") + e.what() + ")");
    } catch (const ModbusError& e) {
        errors.push_back(std::string("report slave id: ") + e.what());
    }
}

/// read_device_info() after FC11: directory, CTCL fallback, family, identification / name blocks, AXNO.
void read_rest(UnitReader& reader, const UnitDirectory* given, DeviceInfo& info, std::vector<std::string>& errors,
               bool have_identity) {
    UnitDirectory directory;
    if (given != nullptr) {
        directory = *given;
    } else {
        try {
            directory = UnitDirectory::read(reader);
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("index table: ") + e.what());
        } catch (const ModbusSlaveError& e) {
            errors.push_back(std::string("index table: ") + e.what());
        }
    }
    if (directory.header()) {
        info.index_header = *directory.header();
        if (directory.empty()) {
            const IndexHeader& h = *directory.header();
            errors.push_back(util::format(
                "index table: unexpected layout (model_code=%d, words_per_record=%d, record_count=%d, rx_buffer=%d, "
                "tx_buffer=%d)",
                h.model_code, h.words_per_record, h.record_count, h.rx_buffer, h.tx_buffer));
        } else {
            info.categories = directory.records();
        }
    }

    if (!have_identity) {
        const std::optional<std::uint32_t> ctcl = read_ctcl(reader, directory);
        const std::optional<SlaveIdentity> from_ctcl = ctcl ? SlaveIdentity::from_ctcl(*ctcl) : std::nullopt;
        if (from_ctcl) {
            info.system_code = from_ctcl->system_code;
            info.unit_code = from_ctcl->unit_code;
            info.firmware = from_ctcl->firmware;
            info.identity_source = from_ctcl->source;
            errors.push_back("identity read from CTCL (class 3 item 1) instead of FC11");
        }
    }

    const std::optional<int> system = info.system_code;
    info.product_family = reg::product_family(system);
    if (system && info.unit_code) {
        const int unit = *info.unit_code;
        const std::optional<std::string> family = reg::ec_family(system, unit);
        const std::string sys_name = reg::system_name(system);
        if (!sys_name.empty()) info.system_name = sys_name;
        if (family) {
            info.unit_name = family;
        } else if (const char* other = not_an_actuator(*system, unit)) {
            info.unit_name = std::string(other);
        }
        info.unit_known = family.has_value();  // a family name for display only: every unit is driven the same way
    }

    const std::optional<int> ident_address = block_address(directory, reg::kIdentificationCategory,
                                                           reg::kIdentificationBase);
    if (ident_address) {
        try {
            const IdentificationBlock block =
                IdentificationBlock::from_words(reader.read_registers(*ident_address, reg::kIdentificationWords));
            info.serial_number = block.serial_number;
            info.actuator_serial = block.actuator_serial;
            info.user_name = block.user_name;
            info.model_name = block.model_name;
            info.accl = block.accl;
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusTimeout& e) {
            errors.push_back(std::string("identification block: ") + e.what());
        } catch (const ModbusSlaveError& e) {
            errors.push_back(std::string("identification block: ") + e.what());
        }
    }

    const std::optional<std::string> controller_name = read_controller_name(reader, system, directory);
    if (controller_name && !controller_name->empty()) info.controller_name = controller_name;

    const std::optional<int> axno = read_axno(reader, directory);
    if (axno) {
        info.axis_number = axno;
        if (*axno != reader.slave() - 1)
            errors.push_back(util::format("AXNO (0x8004) = %d, but slave address %d means axis %d", *axno,
                                          reader.slave(), reader.slave() - 1));
    }
}

}  // namespace

// ------------------------------------------------------------------------------------------ SlaveIdentity

std::optional<SlaveIdentity> SlaveIdentity::from_slave_id(const std::vector<std::uint8_t>& data) {
    if (data.size() < 5) return std::nullopt;
    SlaveIdentity id;
    id.system_code = data[0];
    id.run_status = data[1];
    id.unit_code = data[2];
    id.firmware = util::format("%02X%02X", data[3], data[4]);
    id.source = "FC11";
    return id;
}

std::optional<SlaveIdentity> SlaveIdentity::from_ctcl(std::uint32_t value) {
    if ((value >> 24) == 0) return std::nullopt;
    SlaveIdentity id;
    id.system_code = static_cast<int>(value >> 24);
    id.unit_code = static_cast<int>((value >> 16) & 0xFF);
    id.firmware = util::format("%04X", static_cast<unsigned>(value & 0xFFFF));
    id.source = "CTCL 0x0300";
    return id;
}

// ------------------------------------------------------------------------------------------ IdentificationBlock

IdentificationBlock IdentificationBlock::from_words(const std::vector<std::uint16_t>& words) {
    auto text = [&](int offset, int count) -> std::optional<std::string> {
        if (words.size() < static_cast<std::size_t>(offset + count)) return std::nullopt;
        return reg::words_to_text({words.begin() + offset, words.begin() + offset + count});
    };
    IdentificationBlock block;
    block.serial_number = text(reg::kIdentSerialOffset, 8);
    block.actuator_serial = text(reg::kIdentActuatorSerialOffset, 8);
    block.user_name = text(reg::kIdentUserNameOffset, 6);
    block.model_name = text(reg::kIdentModelOffset, reg::kIdentModelWords);
    if (words.size() > static_cast<std::size_t>(reg::kIdentAcclOffset + 1))
        block.accl = reg::from_u32_words(words[reg::kIdentAcclOffset], words[reg::kIdentAcclOffset + 1]);
    return block;
}

std::optional<int> IdentificationBlock::extensibility_level() const {
    if (!accl) return std::nullopt;
    return static_cast<int>((*accl >> 8) & 0xF);
}

// ------------------------------------------------------------------------------------------ DeviceInfo

std::string DeviceInfo::kind() const { return reg::axis_kind_name(actuator_kind, rotary_atyp); }

std::optional<bool> DeviceInfo::rotary() const {
    if (!actuator_kind) return rotary_atyp;
    return *actuator_kind == reg::kAkidRotary;
}

std::optional<int> DeviceInfo::extensibility_level() const {
    if (!accl) return std::nullopt;
    return static_cast<int>((*accl >> 8) & 0xF);
}

std::string DeviceInfo::controller() const {
    if (!system_code) return "unknown";
    std::string name = system_name && !system_name->empty() ? *system_name : util::format("system code 0x%02x", *system_code);
    if (unit_code) {
        name += util::format(", unit 0x%02X", *unit_code);
        if (unit_name && !unit_name->empty()) name += " (" + *unit_name + ")";
    }
    return name;
}

std::optional<std::string> DeviceInfo::actuator() const {
    if (!actuator_kind) return std::nullopt;
    const std::string name = actuator_kind_name && !actuator_kind_name->empty() ? *actuator_kind_name : "unknown kind";
    return name + " (AKID " + std::to_string(*actuator_kind) + ")";
}

std::string DeviceInfo::model() const {
    if (model_name && !model_name->empty()) return reg::printable(*model_name);
    if (controller_name && !controller_name->empty()) return reg::printable(*controller_name);
    return reg::printable(controller());
}

const IndexCategory* DeviceInfo::category(int category_id) const {
    for (const IndexCategory& c : categories)
        if (c.category_id == category_id) return &c;
    return nullptr;
}

// ------------------------------------------------------------------------------------------ read_device_info

DeviceInfo read_device_info(UnitReader& reader, const UnitDirectory* directory) {
    DeviceInfo info;
    info.slave = reader.slave();
    std::vector<std::string> errors;
    std::optional<SlaveIdentity> identity;
    read_identity(reader, info, errors, identity);
    if (identity) {
        info.system_code = identity->system_code;
        info.run_status = identity->run_status;
        info.unit_code = identity->unit_code;
        info.firmware = identity->firmware;
        info.identity_source = identity->source;
    }
    try {
        read_rest(reader, directory, info, errors, identity.has_value());
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        errors.push_back("identify stopped early: " + error_type_name(e) + ": " + reg::printable(e.what()));
    } catch (const std::out_of_range& e) {
        errors.push_back("identify stopped early: " + error_type_name(e) + ": " + reg::printable(e.what()));
    } catch (const std::invalid_argument& e) {
        errors.push_back("identify stopped early: " + error_type_name(e) + ": " + reg::printable(e.what()));
    }
    info.errors.clear();
    for (const std::string& e : errors) info.errors.push_back(reg::printable(e));
    return info;
}

}  // namespace iai_actuator_driver
