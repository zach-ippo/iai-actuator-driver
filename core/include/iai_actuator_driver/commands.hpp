// Thin, stateless senders of the unit commands the axis uses (one or two frames each, no waiting, no checks): the
// command strategies of the Python edition (StandardCoilStrategy / IaiCommandStrategy in ec.py; C# CommandStrategy.cs)
// and the fixed frames of the direct move, STOP, pause, Modbus control, action mode, software reset and the stored-row
// move. Coil and register addresses are in registers.hpp. The Axis does every check and every wait.
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/modbus.hpp"

namespace iai_actuator_driver {

/// How servo / home / alarm-reset commands are sent (the command strategy of the Python edition). Moves always use
/// 0x9900; stops use the STOP coil, except on the register path (Register, or a line / unit without FC05 coils), whose
/// stop is the coil-free substitute (Axis::stop_path()).
enum class CommandMethod {
    Coil,      ///< FC05 coils SON / HOME / ALRS (VERIFIED in the Python edition). The default.
    Iai,       ///< IAI 0x45: servo cmd 9 and home cmd 0x0A VERIFIED on an EC; alarm reset cmd 5 op1 0 accepted
               ///< (clearing a real alarm UNVERIFIED).
    Iai2,      ///< EXPERIMENTAL IAI 0x48: servo 0x30, home 0x31, alarm reset 0x23 (UNVERIFIED on any unit)
    Register,  ///< FC06 writes of DRG1 0x0D00 (RegisterCommands): the path RCON documents (DOC ME0413 p.5-11, 5-13)
               ///< and every CON unit too (DOC ME0162-12A p.5-124 to 5-128); UNVERIFIED on hardware.
};

/// "coil" / "iai" / "iai2" / "register".
inline const char* to_string(CommandMethod m) noexcept {
    switch (m) {
        case CommandMethod::Iai: return "iai";
        case CommandMethod::Iai2: return "iai2";
        case CommandMethod::Register: return "register";
        default: return "coil";
    }
}
/// The method of a name ("coil", "iai", "iai2", "register"), nullopt when unknown.
std::optional<CommandMethod> command_method_from_string(const std::string& name);
/// True for the methods whose servo / home / reset are FC05 coil writes (Coil).
inline bool uses_coils(CommandMethod m) noexcept { return m == CommandMethod::Coil; }
/// True for the methods whose servo / home / reset are IAI 0x45 / 0x48 commands (Iai, Iai2).
inline bool uses_iai(CommandMethod m) noexcept { return m == CommandMethod::Iai || m == CommandMethod::Iai2; }

/// The register command path: FC06 writes of DRG1 0x0D00 (no FC05 coil, no IAI command). Port of the Python edition's
/// RegisterCommandStrategy (ec.py). Every write is a read-modify-write of the unit's own DRG1 image: SON, SFTY and a
/// held STP are carried over (reg::kDrg1Keep), EMG and BKRL are never set. Frames (slave 01, safety speed off):
///   servo on / off            01 06 0D 00 10 00 / 01 06 0D 00 00 00
///   home (edge)               10 10, released 10 00 once homing ended
///   alarm reset (servo kept)  11 00 then 10 00 (not IAI's sample 01 00: it drops SON)
///   pause / resume            10 20 / 10 00
///   stop substitute           pause 10 20, wait for standstill (the Axis), ALRS edge while paused 11 20 then 10 20
///                             (cancels the remaining travel: DOC ME0384 p.6-76, ME0162-12A p.4-11, 5-85), release 10 00
/// The image is read with FC03 before each write (one extra frame); when that read fails the last image read is used,
/// never a guess (a guessed SON = 0 would switch the servo off). Thread-safe (one image per slave). On an EC the DRG1
/// readback is VERIFIED (0x1000 / 0x5000), an FC06 write of it is UNVERIFIED; RCON and CON: DOC, UNVERIFIED here.
class RegisterCommands {
public:
    /// (levels to keep, raw DRG1) read now; on a failed read the last image (raw unknown: = the levels). Throws the
    /// read's error when no image was saved yet.
    std::pair<std::uint16_t, std::uint16_t> image(ModbusRtu& bus, int slave);
    /// FC06 DRG1 = value (EMG and BKRL always cleared); the image follows.
    void write(ModbusRtu& bus, int slave, std::uint16_t value);
    void servo(ModbusRtu& bus, int slave, bool on);
    /// HOME rising edge (a HOME level left high is dropped first).
    void start_home(ModbusRtu& bus, int slave);
    /// HOME released (after homing ended or failed).
    void finish_home(ModbusRtu& bus, int slave);
    /// ALRS edge keeping SON (the image is read again before the release: a reset may bring the servo back).
    void reset_alarm(ModbusRtu& bus, int slave);
    /// STP level: `on` pauses (keeps the remaining travel), off resumes. A pause (the first frame of a stop) is always
    /// sent: when DRG1 cannot be read and no image is saved, SON / SFTY come from DSS1 (0x9005), else SON alone is kept
    /// (an axis that moves has its servo on).
    void pause(ModbusRtu& bus, int slave, bool on);
    /// SFTY level, SON kept.
    void safety_speed(ModbusRtu& bus, int slave, bool on);
    /// ALRS edge with STP held (drops the remaining travel), then with `release` STP off.
    void cancel_paused(ModbusRtu& bus, int slave, bool release = true);
    /// The last image of `slave` (nullopt: none read or written yet).
    std::optional<std::uint16_t> last_image(int slave) const;

private:
    mutable std::mutex mutex_;
    std::map<int, std::uint16_t> images_;
};

/// Servo on / off: coil SON 0x0403, 0x45 cmd 9 op1 1/0, 0x48 cmd 0x30 op1 1/0, or DRG1 SON. `registers`: the DRG1
/// image keeper of the Register method (nullptr: a fresh one, which reads DRG1 first and has no saved image).
void send_servo(ModbusRtu& bus, int slave, bool on, CommandMethod method = CommandMethod::Coil,
                RegisterCommands* registers = nullptr);

/// Start a home return: coil HOME 0x040B 0000 then FF00 (rising edge), 0x45 cmd 0x0A op1 = org & 0xFFFF sent once
/// without retries (op1 0 is rejected 0x11: EcNotReadyError without a usable org, nothing sent), 0x48 cmd 0x31 without
/// an operand (UNVERIFIED), or the DRG1 HOME edge.
void start_home(ModbusRtu& bus, int slave, CommandMethod method = CommandMethod::Coil,
                std::optional<std::uint32_t> org = std::nullopt, RegisterCommands* registers = nullptr);

/// Release HOME after HEND: the coil (Coil) or DRG1 HOME (Register); no-op for Iai / Iai2.
void finish_home(ModbusRtu& bus, int slave, CommandMethod method = CommandMethod::Coil,
                 RegisterCommands* registers = nullptr);

/// Alarm reset: coil ALRS 0x0407 FF00 then 0000 (rising edge), 0x45 cmd 5 op1 0, 0x48 cmd 0x23, or the DRG1 ALRS edge
/// keeping SON.
void send_alarm_reset(ModbusRtu& bus, int slave, CommandMethod method = CommandMethod::Coil,
                      RegisterCommands* registers = nullptr);

/// STOP coil 0x042C (decel stop; held ON until standstill, then OFF - the Axis does the hold).
void send_stop_coil(ModbusRtu& bus, int slave, bool on);

/// Pause coil STP 0x040A (level: ON holds the move and keeps the remaining travel; OFF resumes it).
void send_pause(ModbusRtu& bus, int slave, bool on);

/// PMSL coil 0x0427: ON = Modbus control (DSSE PMSS = 1), OFF = back to PIO. Never switched during motion.
void send_modbus_control(ModbusRtu& bus, int slave, bool on);

/// 0x45 cmd 0x0C: op1 1 = PC (Modbus) / 0 = PIO, op2 1 = safety speed on (VERIFIED on the EC-R6S rod).
IaiResponse send_action_mode(ModbusRtu& bus, int slave, bool pc, bool safety);

/// Safety-speed coil SFTY 0x0401 (level; volatile).
void send_safety_speed(ModbusRtu& bus, int slave, bool on);

/// Teach coils (CON units, DOC ME0162-12A p.5-97; EXPERIMENTAL, UNVERIFIED on an EC): MOD 0x0414 (teaching mode,
/// level) and TEAC 0x0415 (write the current position into row POSR, level held >= 20 ms).
void send_teach_mode(ModbusRtu& bus, int slave, bool on);
void send_teach(ModbusRtu& bus, int slave, bool on);
/// POSR 0x0D03 = row (the row TEAC / CSTR act on; FC06; it does not start a move, unlike 0x9800).
void send_position_select(ModbusRtu& bus, int slave, int row);

/// How Axis::teach_position stores the current position.
enum class TeachMethod {
    Table,  ///< read the row, replace the position, write it back (write_position_table; the VERIFIED pieces)
    Coil,   ///< EXPERIMENTAL MOD / POSR / TEAC coils (DOC for CON units, ME0162-12A p.5-97; UNVERIFIED on an EC)
};
/// "table" / "coil".
inline const char* to_string(TeachMethod m) noexcept { return m == TeachMethod::Coil ? "coil" : "table"; }
/// How long the coil teach waits for DSS2 b10 PWRT (s).
inline constexpr double kTeachWriteS = 1.0;

/// Software reset 01 45 00 05 00 01 00 00 (ON-WIRE + VERIFIED; allow_unsafe), sent once (retries 0). The reply may be
/// lost while the unit restarts: ModbusTimeout then means "maybe accepted".
IaiResponse send_software_reset(ModbusRtu& bus, int slave);

/// Stored-row move: FC06 0x9800 = row (starts at once; ignored by an EC while PMSS = 0, VERIFIED).
void send_position_move(ModbusRtu& bus, int slave, int row);

/// The 0x45 cmd 6 speed override code of `speed_pct` (finite; std::invalid_argument otherwise): the unit runs the row
/// at (code + 1) / 256 of its stored speed (ON-WIRE on the PCON-CB: code 13 ran an 840 mm/s row at 46.0 mm/s, code 3
/// at 13.1 mm/s; other units UNVERIFIED, the same register), so code = round(256 x pct / 100) - 1, clamped to 1..255
/// (100 % -> 255, 25 % -> 63, 5 % -> 12; 0.97 % and below -> 1 = 0.78 %, the slowest the unit runs).
int speed_override_code(double speed_pct);
/// The share of the stored speed (%) the unit runs with override `code`: (code + 1) / 256 x 100.
double speed_override_pct(int code);

/// Stored-row move with a speed override: 0x45 cmd 6 op1 = row, op2 = speed_override_code(pct) (VERIFIED on the EC
/// gripper and the PCON).
IaiResponse send_position_move_override(ModbusRtu& bus, int slave, int row, double speed_pct);

/// The 9-word direct numeric move block at 0x9900 (DOC-CON layout, VERIFIED frame shape): an FC10 write starts the
/// move at once and re-targets a running one. Never write past 0x9908 (0x9909 is a DRG1 mirror).
struct DirectMove {
    std::int32_t pcmd = 0;    ///< target, 0.01 mm counts (or 0.01 deg)
    std::uint32_t inp = 0;    ///< positioning band, counts
    std::uint32_t vcmd = 0;   ///< speed, 0.01 units/s
    std::uint16_t acmd = 0;   ///< accel = decel, 0.01 G
    std::uint16_t ppow = 0;   ///< push power (0 for a normal move)
    std::uint16_t ctlf = 0;   ///< control flags (0 = absolute)

    /// PCMD hi lo, INP hi lo, VCMD hi lo, ACMD, PPOW, CTLF (high word first).
    std::vector<std::uint16_t> words() const;
    bool operator==(const DirectMove& other) const {
        return pcmd == other.pcmd && inp == other.inp && vcmd == other.vcmd && acmd == other.acmd &&
               ppow == other.ppow && ctlf == other.ctlf;
    }
};

/// FC10 of the 9-word block at 0x9900.
void write_direct_move(ModbusRtu& bus, int slave, const DirectMove& move);

/// PARW coil 0x0428 ON (commit RAM parameters to non-volatile memory; self-clearing, no OFF is sent; ON-WIRE). The
/// Axis waits for the handshake (DRGE b7 / DSSE b7) around it.
void send_parameter_commit(ModbusRtu& bus, int slave);

}  // namespace iai_actuator_driver
