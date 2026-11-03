// IAI function 0x45 tables: the safe list, the never-send list, the rejection names and the driver's hints.
// Ported from the Python edition: modbus.py (IAI_SAFE_COMMANDS) and registers.py (IAI_NEVER_SEND, IAI_REJECTIONS,
// IAI_REJECTION_HINTS, IAI_COMMAND_REJECTION_HINTS). Rejection names are UNVERIFIED except 0x11 "1st operand error"
// (VERIFIED: the 0x45 home with op1 = 0 on the EC-R6S rod).
#include <string>

#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"

namespace iai_actuator_driver {

std::string iai_rejection_name(int code) {
    switch (code) {
        case 0x01: return "illegal function";
        case 0x02: return "illegal data address";
        case 0x03: return "illegal data value";
        case 0x04: return "cannot execute: slave recovery disabled";
        case 0x11: return "1st operand error";
        case 0x12: return "2nd operand error";
        case 0x1F: return "password mismatch";
        case 0x21: return "command code error";
        case 0x22: return "EEPROM write already in progress";
        case 0x23: return "motion command while the servo is OFF";
        case 0x24: return "motion command during homing";
        case 0x25: return "software reset while the servo is ON";
        case 0x26: return "motion command in TEACH mode";
        case 0x27: return "invalid HOME command";
        case 0x28: return "parameter initialisation before EEPROM init completed";
        case 0x29: return "software reset with ABS unit and Z phase enabled";
        case 0x2A: return "load-cell command without a load cell";
        case 0x2B: return "load-cell command with the servo ON";
        case 0x2C: return "monitoring data type changed during monitoring";
        case 0x2D: return "monitoring command while monitoring is invalid";
        case 0x2E: return "calendar command while the calendar is invalid";
        case 0x2F: return "release of safety speed prohibited";
        case 0x30: return "monitoring conditions incorrect";
        case 0x31: return "battery-less absolute encoder command error";
        case 0x32: return "encoder FRAM access with the servo ON";
        case 0x33: return "encoder FRAM: bad page";
        case 0x34: return "encoder FRAM: bad unique code";
        case 0x35: return "encoder FRAM write error";
        case 0x38: return "invalid command during virtual axis";
        case 0x39: return "software reset during homing";
        case 0x3A: return "command prohibited in monitor (PIO) mode";
        case 0x3B: return "software reset while software reset is inhibited";
        case 0x3C: return "servo ON after encoder FRAM access";
        case 0x3D:
        case 0x3E:
        case 0x3F: return "inspection command error";
        default: return "unknown rejection code";
    }
}

std::string iai_rejection_hint(int code, int command) {
    // command-specific hints first (VERIFIED: home op1 = 0 rejected 0x11, op1 = ORG accepted on the EC-R6S rod)
    if (command == reg::kIaiCmdHome && code == 0x11)
        return "the home return needs op1 = the EC's home pattern ORG (item 9.16)";
    // VERIFIED in the Python edition: row 2 rejected by the EC-GRBP8M gripper in 2-position mode (P9 = 0), nothing moved
    if (command == reg::kIaiCmdMovePosition && code == 0x11)
        return "the unit does not run this stored row in its position mode (an EC runs rows 0 and 1 in 2-position "
               "mode, P9 0 / 3; rows 0-2 in 3-position mode)";
    switch (code) {
        case 0x23: return "turn the servo on first";
        case 0x24: return "wait for the home return to finish";
        case 0x25: return "turn the servo off first (0x45 cmd 9 op1 0)";
        case 0x26: return "the unit is in TEACH mode";
        case 0x3A: return "the EC is in PIO (monitor) mode: take Modbus control first (PMSL coil, or action mode op1 = 1)";
        default: return std::string();
    }
}

std::string iai_safe_command(int command) {
    // IAI's control-command numbering. 4 / 5 / 9 / 0x0C are ON-WIRE; 5 / 9 / 0x0A / 0x0B / 0x0C are VERIFIED on an EC
    // (6 on an EC gripper and a PCON); the others UNVERIFIED.
    switch (command) {
        case 0x05: return "reset (alarm reset only: op1 must be 0)";
        case 0x06: return "move to position index with override";
        case 0x07: return "move to position index with speed";
        case 0x09: return "servo on/off";
        case 0x0A: return "return home";
        case 0x0B: return "decel stop";
        case 0x0C: return "change action mode";
        case 0x0D: return "change speed while moving";
        case 0x0E: return "change acceleration while moving";
        default: return std::string();
    }
}

std::string iai_never_send(int command) {
    switch (command) {
        case 0x01: return "write parameters to non-volatile memory (the driver commits with the PARW coil instead)";
        case 0x02: return "write positions to non-volatile memory";
        case 0x03: return "change the axis number (slave address)";
        case 0x04: return "change the baud rate (set_baud_rate opts in)";
        case 0x08: return "change the ID group";
        case 0x0F: return "clear the alarm list (wipes the non-volatile alarm history)";
        case 0x10: return "initialise parameters (factory reset)";
        case 0x11: return "change the DO type";
        case 0x1D: return "initialise the absolute encoder";
        case 0x4F: return "change the maintenance update mode";
        case 0x73: return "set the axis index";
        case 0x90:
        case 0x91: return "switch to firmware update mode";
        case 0xF3: return "clear position data";
        default: break;
    }
    if (command >= 0x20 && command < 0x3D) return "write maintenance information";
    if (command >= 0xDD && command < 0x100) return "factory inspection command";
    return std::string();
}

}  // namespace iai_actuator_driver
