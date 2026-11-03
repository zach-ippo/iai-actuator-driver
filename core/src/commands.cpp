// Thin senders of the unit commands (port of the Python edition's StandardCoilStrategy / IaiCommandStrategy and the
// fixed frames the axis sends; C# CommandStrategy.cs). No waiting and no checks beyond the frame itself: the Axis
// does every check and every wait. Frames VERIFIED in the Python edition unless noted in commands.hpp.
#include "iai_actuator_driver/commands.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {
constexpr const char* kCommandsLog = "iai_actuator_driver.axis";  ///< the register path's warnings go to the axis's log
}  // namespace

void send_servo(ModbusRtu& bus, int slave, bool on, CommandMethod method, RegisterCommands* registers) {
    switch (method) {
        case CommandMethod::Iai: bus.iai_command(slave, 0, reg::kIaiCmdServo, on ? 1 : 0); return;
        case CommandMethod::Iai2: bus.iai_command2(slave, reg::kIai2CmdServo, {on ? 1u : 0u}); return;
        case CommandMethod::Register: {
            RegisterCommands fresh;
            (registers ? *registers : fresh).servo(bus, slave, on);
            return;
        }
        default: bus.write_coil(slave, reg::kCoilSon, on);
    }
}

void start_home(ModbusRtu& bus, int slave, CommandMethod method, std::optional<std::uint32_t> org,
                RegisterCommands* registers) {
    if (method == CommandMethod::Iai) {
        if (!org || (*org & 0xFFFFu) == 0)
            throw EcNotReadyError(
                "the 0x45 home needs op1 = the unit's home pattern ORG (item 9.16); none given (op1 = 0 is rejected "
                "0x11 '1st operand error'). Nothing sent");
        // Sent once: a lost reply may hide a home that runs (a repeated frame would be rejected 0x24 meanwhile).
        IaiOptions options;
        options.retries = 0;
        bus.iai_command(slave, 0, reg::kIaiCmdHome, static_cast<int>(*org & 0xFFFFu), 0, options);
        return;
    }
    if (method == CommandMethod::Iai2) {  // 0x48 home without an operand, sent once (UNVERIFIED)
        IaiOptions options;
        options.retries = 0;
        bus.iai_command2(slave, reg::kIai2CmdHome, {}, options);
        return;
    }
    if (method == CommandMethod::Register) {
        RegisterCommands fresh;
        (registers ? *registers : fresh).start_home(bus, slave);
        return;
    }
    bus.write_coil(slave, reg::kCoilHome, false);  // HOME acts on the rising edge
    bus.write_coil(slave, reg::kCoilHome, true);
}

void finish_home(ModbusRtu& bus, int slave, CommandMethod method, RegisterCommands* registers) {
    if (method == CommandMethod::Coil) {
        bus.write_coil(slave, reg::kCoilHome, false);
    } else if (method == CommandMethod::Register) {
        RegisterCommands fresh;
        (registers ? *registers : fresh).finish_home(bus, slave);
    }
}

void send_alarm_reset(ModbusRtu& bus, int slave, CommandMethod method, RegisterCommands* registers) {
    switch (method) {
        case CommandMethod::Iai:
            bus.iai_command(slave, 0, reg::kIaiCmdReset, reg::kIaiResetAlarm);  // op1 0 = alarm reset (1 = software reset)
            return;
        case CommandMethod::Iai2: bus.iai_command2(slave, reg::kIai2CmdAlarmReset); return;
        case CommandMethod::Register: {
            RegisterCommands fresh;
            (registers ? *registers : fresh).reset_alarm(bus, slave);
            return;
        }
        default:
            bus.write_coil(slave, reg::kCoilAlrs, true);  // ALRS acts on the rising edge
            bus.write_coil(slave, reg::kCoilAlrs, false);
    }
}

void send_stop_coil(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilStop, on); }

void send_pause(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilStp, on); }

void send_modbus_control(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilPmsl, on); }

IaiResponse send_action_mode(ModbusRtu& bus, int slave, bool pc, bool safety) {
    return bus.iai_command(slave, 0, reg::kIaiCmdActionMode, pc ? 1 : 0, safety ? 1 : 0);
}

void send_safety_speed(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilSfty, on); }

void send_teach_mode(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilMod, on); }

void send_teach(ModbusRtu& bus, int slave, bool on) { bus.write_coil(slave, reg::kCoilTeac, on); }

void send_position_select(ModbusRtu& bus, int slave, int row) {
    if (row < 0 || row > 0xFFFF) throw std::invalid_argument("position row must be 0..65535");
    bus.write_register(slave, reg::kPosr, static_cast<std::uint16_t>(row));
}

IaiResponse send_software_reset(ModbusRtu& bus, int slave) {
    IaiOptions options;
    options.allow_unsafe = true;
    options.retries = 0;
    return bus.iai_command(slave, 0, reg::kIaiCmdReset, reg::kIaiResetSoftware, 0, options);
}

void send_position_move(ModbusRtu& bus, int slave, int row) {
    if (row < 0 || row > 0xFFFF) throw std::invalid_argument("position row must be 0..65535");
    bus.write_register(slave, reg::kCommand1Base, static_cast<std::uint16_t>(row));
}

IaiResponse send_position_move_override(ModbusRtu& bus, int slave, int row, double speed_pct) {
    if (!std::isfinite(speed_pct)) throw std::invalid_argument("speed_pct must be a finite number");
    // round half to even, as the Python edition's round() (255 x 30 % = 76.5 -> 76)
    const double scaled = std::nearbyint(255.0 * speed_pct / 100.0);
    const int override_value = static_cast<int>(std::max(1.0, std::min(scaled, 65535.0)));
    IaiOptions options;
    options.retries = 0;
    return bus.iai_command(slave, 0, reg::kIaiCmdMovePosition, row, override_value, options);
}

std::vector<std::uint16_t> DirectMove::words() const {
    const auto p = reg::to_i32_words(pcmd);
    const auto i = reg::to_u32_words(inp);
    const auto v = reg::to_u32_words(vcmd);
    return {p.first, p.second, i.first, i.second, v.first, v.second, acmd, ppow, ctlf};
}

void write_direct_move(ModbusRtu& bus, int slave, const DirectMove& move) {
    bus.write_registers(slave, reg::kDirectMoveBase, move.words());
}

void send_parameter_commit(ModbusRtu& bus, int slave) { bus.write_coil(slave, reg::kCoilParw, true); }

// ------------------------------------------------------------------------------------------ the command methods

std::optional<CommandMethod> command_method_from_string(const std::string& name) {
    if (name == "coil") return CommandMethod::Coil;
    if (name == "iai") return CommandMethod::Iai;
    if (name == "iai2") return CommandMethod::Iai2;
    if (name == "register") return CommandMethod::Register;
    return std::nullopt;
}

// ------------------------------------------------------------------------------------------ RegisterCommands

std::pair<std::uint16_t, std::uint16_t> RegisterCommands::image(ModbusRtu& bus, int slave) {
    std::uint16_t raw = 0;
    try {
        raw = bus.read_registers(slave, reg::kDrg1, 1).at(0);
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = images_.find(slave);
        if (it == images_.end()) throw;
        log_warning(kCommandsLog, util::format("DRG1 not readable (%s): using the last DRG1 image 0x%04X", e.what(),
                                               static_cast<unsigned>(it->second)));
        return {it->second, it->second};
    }
    const std::uint16_t keep = static_cast<std::uint16_t>(raw & reg::kDrg1Keep);
    std::lock_guard<std::mutex> lock(mutex_);
    images_[slave] = keep;
    return {keep, raw};
}

void RegisterCommands::write(ModbusRtu& bus, int slave, std::uint16_t value) {
    value = static_cast<std::uint16_t>(value & ~(reg::kDrg1Emg | reg::kDrg1Bkrl));
    bus.write_register(slave, reg::kDrg1, value);
    std::lock_guard<std::mutex> lock(mutex_);
    images_[slave] = static_cast<std::uint16_t>(value & reg::kDrg1Keep);
}

void RegisterCommands::servo(ModbusRtu& bus, int slave, bool on) {
    const std::uint16_t keep = image(bus, slave).first;
    write(bus, slave, static_cast<std::uint16_t>(on ? keep | reg::kDrg1Son : keep & ~reg::kDrg1Son));
}

void RegisterCommands::start_home(ModbusRtu& bus, int slave) {
    const auto img = image(bus, slave);
    if (img.second & reg::kDrg1Home) write(bus, slave, img.first);  // HOME acts on the rising edge: drop a level left high
    write(bus, slave, static_cast<std::uint16_t>(img.first | reg::kDrg1Home));
}

void RegisterCommands::finish_home(ModbusRtu& bus, int slave) { write(bus, slave, image(bus, slave).first); }

void RegisterCommands::reset_alarm(ModbusRtu& bus, int slave) {
    write(bus, slave, static_cast<std::uint16_t>(image(bus, slave).first | reg::kDrg1Alrs));
    write(bus, slave, image(bus, slave).first);  // re-read: an alarm reset may bring the servo back by itself
}

void RegisterCommands::pause(ModbusRtu& bus, int slave, bool on) {
    std::uint16_t keep = 0;
    try {
        keep = image(bus, slave).first;
    } catch (const TransportError&) {
        throw;
    } catch (const ModbusError&) {
        if (!on) throw;
        try {
            const std::uint16_t dss1 = bus.read_registers(slave, reg::kDss1, 1).at(0);
            log_warning(kCommandsLog, util::format("DRG1 not readable: pausing with SON / SFTY taken from DSS1 0x%04X",
                                                   static_cast<unsigned>(dss1)));
            keep = static_cast<std::uint16_t>(((dss1 & reg::kDss1Sv) ? reg::kDrg1Son : 0) |
                                              ((dss1 & reg::kDss1Sfty) ? reg::kDrg1Sfty : 0));
        } catch (const TransportError&) {
            throw;
        } catch (const ModbusError&) {
            log_warning(kCommandsLog, "DRG1 and DSS1 not readable: pausing with SON kept");
            keep = reg::kDrg1Son;
        }
    }
    write(bus, slave, static_cast<std::uint16_t>(on ? keep | reg::kDrg1Stp : keep & ~reg::kDrg1Stp));
}

void RegisterCommands::safety_speed(ModbusRtu& bus, int slave, bool on) {
    const std::uint16_t keep = image(bus, slave).first;
    write(bus, slave, static_cast<std::uint16_t>(on ? keep | reg::kDrg1Sfty : keep & ~reg::kDrg1Sfty));
}

void RegisterCommands::cancel_paused(ModbusRtu& bus, int slave, bool release) {
    const std::uint16_t held = static_cast<std::uint16_t>(image(bus, slave).first | reg::kDrg1Stp);
    write(bus, slave, static_cast<std::uint16_t>(held | reg::kDrg1Alrs));
    write(bus, slave, held);
    if (release) write(bus, slave, static_cast<std::uint16_t>(held & ~reg::kDrg1Stp));
}

std::optional<std::uint16_t> RegisterCommands::last_image(int slave) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = images_.find(slave);
    return it == images_.end() ? std::nullopt : std::optional<std::uint16_t>(it->second);
}

}  // namespace iai_actuator_driver
