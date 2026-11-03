// The IAI 0x48 tables (port of modbus.py IAI_SAFE_COMMANDS2 and registers.py IAI2_NEVER_SEND / IAI2_REJECTIONS /
// IAI2_REJECTION_HINTS):
// which 16-bit codes ModbusRtu::iai_command2 sends without an opt-in, which it never sends, and the names of the 0x48
// rejection words. Every 0x48 code and name is UNVERIFIED (none of the units tested takes 0x48).
#include <map>
#include <string>

#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"

namespace iai_actuator_driver {

namespace {

const std::map<int, std::string>& safe_commands2() {
    static const std::map<int, std::string> table = {
        {0x22, "change action mode"},
        {0x23, "alarm reset"},
        {0x30, "servo on/off"},
        {0x31, "return home"},
        {0x32, "move to position index with override"},
        {0x33, "move to position index with speed"},
        {0x34, "decel stop"},
        {0x35, "change speed while moving"},
        {0x36, "change acceleration while moving"},
    };
    return table;
}

const std::map<int, std::string>& never_send2() {
    static const std::map<int, std::string> table = {
        {0x0001, "update maintenance information"},
        {0x0002, "update maintenance information"},
        {0x0010, "write parameters to non-volatile memory"},
        {0x0011, "write positions to non-volatile memory"},
        {0x0012, "change the axis number"},
        {0x0020, "change the baud rate"},
        {0x0021, "change the ID group"},
        {0x0040, "clear the alarm list"},
        {0x0041, "initialise parameters"},
        {0x0042, "change the DO type"},
        {0x0061, "load-cell key command"},
        {0x0080, "initialise the absolute encoder"},
        {0x0090, "safety unit command"},
        {0x0091, "safety unit command"},
        {0x0092, "safety unit command"},
        {0x0093, "safety unit command"},
        {0x00A1, "change the drive mode"},
        {0x00B3, "change the axis ID (converter)"},
        {0x1000, "switch to firmware update mode"},
        {0xFFFD, "clear position data"},
    };
    return table;
}

const std::map<int, std::string>& rejections2() {
    static const std::map<int, std::string> table = {
        {0x1000, "command code error"},
        {0x1001, "1st operand error"},
        {0x1002, "2nd operand error"},
        {0x1003, "3rd operand error"},
        {0x1004, "4th operand error"},
        {0x1005, "motion command while the servo is OFF"},
        {0x1006, "motion command during homing"},
        {0x1007, "software reset while the servo is ON"},
        {0x1008, "motion command in TEACH mode"},
        {0x1009, "invalid HOME command"},
        {0x100A, "parameter initialisation before EEPROM init completed"},
        {0x100B, "load-cell command without a load cell"},
        {0x100C, "load-cell command with the servo ON"},
        {0x100D, "monitoring data type changed during monitoring"},
        {0x100E, "monitoring command while monitoring is invalid"},
        {0x100F, "monitoring conditions incorrect"},
        {0x1010, "servo monitor not available"},
        {0x1011, "drive recorder not available"},
        {0x1012, "calendar command while the calendar is invalid"},
        {0x1013, "release of safety speed prohibited"},
        {0x1014, "invalid command during virtual axis"},
        {0x1015, "software reset during homing"},
        {0x1016, "inspection command error"},
        {0x1017, "inspection command error"},
        {0x101C, "monitoring command in drive-recorder mode"},
        {0x101D, "software reset while software reset is inhibited"},
        {0x101E, "battery-less absolute encoder command error"},
        {0x101F, "movement during collision-detection reversal"},
    };
    return table;
}

std::string lookup(const std::map<int, std::string>& table, int key, const std::string& missing) {
    const auto it = table.find(key);
    return it == table.end() ? missing : it->second;
}

}  // namespace

std::string iai2_rejection_name(int code) { return lookup(rejections2(), code, "unknown rejection code"); }

std::string iai2_rejection_hint(int code, int command) {
    if (command == reg::kIai2CmdHome && code == 0x1001) return iai_rejection_hint(0x11, reg::kIaiCmdHome);  // UNVERIFIED
    switch (code) {  // the 0x45 hints of the same conditions
        case 0x1005: return iai_rejection_hint(0x23, -1);
        case 0x1006: return iai_rejection_hint(0x24, -1);
        case 0x1007: return iai_rejection_hint(0x25, -1);
        case 0x1008: return iai_rejection_hint(0x26, -1);
        case 0x101F: return "wait for the collision-detection reversal to finish";
        default: return std::string();
    }
}
std::string iai2_safe_command(int command) { return lookup(safe_commands2(), command, std::string()); }
std::string iai2_never_send(int command) { return lookup(never_send2(), command, std::string()); }

}  // namespace iai_actuator_driver
