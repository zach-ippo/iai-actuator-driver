// Alarm tables (data only), generated from the Python edition's alarms.py / registers.py: IAI's English controller
// alarm names, the driver's curated names, the alarm groups, the home-return advice, the driver's own remedies and the
// decoding tables of the history detail / address words. Lookups for src/alarms.cpp (declared in model_detail.hpp).
// Evidence words as in alarms.hpp: names are DOC (as the PC tool shows them), levels / groups UNVERIFIED unless marked.
#include <cstddef>
#include <cstdint>
#include <string>

#include "model_detail.hpp"

namespace iai_actuator_driver {
namespace alarm_data {

namespace {
struct NameEntry {
    int code;
    const char* text;
};

const char* find(const NameEntry* table, std::size_t count, int code) {
    std::size_t lo = 0, hi = count;
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (table[mid].code == code) return table[mid].text;
        if (table[mid].code < code) lo = mid + 1; else hi = mid;
    }
    return nullptr;
}

const NameEntry kControllerNames[] = {
    {0x000, "No alarm (in normal condition)"},
    {0x040, "Stop detected"},
    {0x041, "Motor voltage reduction"},
    {0x047, "Over deviation warning"},
    {0x048, "Driver overload warning"},
    {0x049, "Collision warning"},
    {0x04A, "Estimated service life exceeded warning"},
    {0x04C, "Drop in number of fan revolution"},
    {0x04D, "Total driving time of fan is exceeded threshold."},
    {0x04E, "Total moving count is exceeded threshold."},
    {0x04F, "Total moving distance is exceeded threshold."},
    {0x05A, "Receive overrun"},
    {0x05B, "Receive framing error"},
    {0x05C, "Receive timeout error"},
    {0x05D, "Header error"},
    {0x05E, "Delimiter error"},
    {0x061, "Wrong character string or wrong address error"},
    {0x062, "1 Operand error"},
    {0x063, "2 Operand error"},
    {0x064, "3 Operand error"},
    {0x065, "EEPROM WRITE time-out"},
    {0x068, "Abnormal SRAM access"},
    {0x069, "Real Time Clock oscillation stop detection"},
    {0x06A, "Abnormal Real Time Clock access"},
    {0x06B, "Abnormal maintenance information data."},
    {0x070, "Motion command while in RUN-OFF condition"},
    {0x071, "Not yet Homed PTP"},
    {0x072, "Invalid HOME command"},
    {0x073, "Alarm reset while in SERVO-ON condition"},
    {0x074, "Pole sense error"},
    {0x075, "Motion command during Homing"},
    {0x076, "Software reset command while in SERVO-ON condition"},
    {0x077, "Motion command while in TEACH MODE"},
    {0x078, "Writing instruction to EEPROM while in TEACH MODE"},
    {0x079, "Inspection command while in SERVO-ON condition"},
    {0x07A, "ABS Data Backup Battery Voltage Low Warning"},
    {0x07B, "Access error"},
    {0x07C, "Can't shift coordinate system at the ABS encoder specification"},
    {0x07F, "BCC error"},
    {0x080, "Motion command while in SERVO-OFF condition"},
    {0x082, "Position specification movement command when Homing is not completed"},
    {0x083, "Absolute positional movement command when Homing is not completed"},
    {0x084, "Motion command during Homing"},
    {0x085, "Position No. error when moving"},
    {0x086, "Motion command while in PULSE MODE"},
    {0x087, "Movement instruction in Load Cell calibration"},
    {0x088, "Load cell invalid when force sensor pressing directive"},
    {0x089, "Load cell invalid during the press program start-up command"},
    {0x08A, "Deviation Error Non-monitoring Condition Failure"},
    {0x08B, "Safety unit initializing error"},
    {0x08C, "Safety unit usage conditional error"},
    {0x08D, "Safety unit error"},
    {0x08E, "Safety unit microcomputer A error"},
    {0x08F, "Safety unit microcomputer B error"},
    {0x090, "Software reset command while in SERVO-ON condition"},
    {0x091, "Position No. error when teaching"},
    {0x092, "The PWRT signal was detected while moving"},
    {0x093, "The PWRT signal was detected when Homing is not completed"},
    {0x094, "Press Program Alarm detected"},
    {0x095, "DCLR signal detection at the datum position movement command"},
    {0x096, "Homing error"},
    {0x097, "Software reset command in the state of software reset inhibit"},
    {0x098, "Software overcurrent"},
    {0x09A, "I/O 24V Overcurrent"},
    {0x09B, "Internal communication error"},
    {0x09C, "Servo-on w/out power-cycle after flashing the encoder FRAM"},
    {0x09D, "Actuator mismatch"},
    {0x09E, "Actuator misconnection detection"},
    {0x09F, "Synchronous turning error"},
    {0x0A0, "PIO function allocation error"},
    {0x0A1, "Parameter error when executing"},
    {0x0A2, "Position data error when executing"},
    {0x0A3, "Positional instruction information error when executing"},
    {0x0A4, "Command count overflow"},
    {0x0A5, "Electromagnetic brake unrelease error"},
    {0x0A6, "Dynamic brake unrelease error"},
    {0x0A7, "The instructed deceleration is abnormal."},
    {0x0A8, "Unsupport motor/encoder type"},
    {0x0A9, "Abnormal Load Cell data"},
    {0x0AA, "Abnormal motor power line connection"},
    {0x0AB, "The instructed speed is abnormal."},
    {0x0AC, "Estimated service life exceeded error"},
    {0x0AD, "Press Program data incorrect"},
    {0x0AE, "Press Direct Value Motion Error"},
    {0x0AF, "Unsupported function detection"},
    {0x0B0, "Data error in memory Bank30(Parameter)"},
    {0x0B1, "Data error in memory Bank31(Position)"},
    {0x0B2, "Homing beginning position error"},
    {0x0B3, "Linear ABS error."},
    {0x0B4, "Electric angle mismatch"},
    {0x0B5, "Z phase position error"},
    {0x0B6, "Timeout error in Z phase"},
    {0x0B7, "Pole is uncertain"},
    {0x0B8, "Pole sense error (1)"},
    {0x0B9, "Pole sense error (2)"},
    {0x0BA, "Home sensor error"},
    {0x0BB, "Z pulse detection error while Homing"},
    {0x0BC, "A, B pulse detection error while Homing"},
    {0x0BD, "ORG-Speed error before search"},
    {0x0BE, "Time-out error while Homing"},
    {0x0BF, "Creep sensor error"},
    {0x0C0, "Over speed error"},
    {0x0C1, "Servo error"},
    {0x0C2, "Overrun error"},
    {0x0C3, "Motion command when being S-curve accelerating or being decelerating"},
    {0x0C4, "Continuous pushing capable torque excess permissible time over"},
    {0x0C5, "Incorrect control system transition command"},
    {0x0C6, "Torque current/force feedback data mismatching"},
    {0x0C7, "Abnormal regenerative discharge circuit"},
    {0x0C8, "Excess current"},
    {0x0C9, "OverVoltage error"},
    {0x0CA, "Over heat error"},
    {0x0CB, "Error in current sensor offset adjustment"},
    {0x0CC, "Control power voltage error"},
    {0x0CD, "Shutdown relay contact abnormaly"},
    {0x0CE, "Control power voltage reduction"},
    {0x0CF, "I/O 24V power failure"},
    {0x0D0, "Over main power"},
    {0x0D1, "Over regenerate power"},
    {0x0D2, "Power supply voltage of the motor is excessive"},
    {0x0D3, "Motor power voltage reduction"},
    {0x0D4, "Motor power error"},
    {0x0D5, "Deviation counter overflow when homing is not completed"},
    {0x0D6, "Fan error detection"},
    {0x0D7, "The belt cutting sensor was detected."},
    {0x0D8, "Deviation error"},
    {0x0D9, "Software stroke limit over error"},
    {0x0DA, "Feedback pulse error"},
    {0x0DB, "The external alarm signal was detected."},
    {0x0DC, "Push back error"},
    {0x0DD, "Driving mode error"},
    {0x0DE, "External output power source error"},
    {0x0DF, "Collision detection"},
    {0x0E0, "Over load"},
    {0x0E1, "Abnormal Load Cell calibration"},
    {0x0E2, "Abnormal Load Cell communication"},
    {0x0E3, "Abnormal Load Cell"},
    {0x0E4, "Encoder data send error"},
    {0x0E5, "Encoder data receive error"},
    {0x0E6, "Encoder count error"},
    {0x0E7, "Disconnection of Encoder A, B and Z pulse"},
    {0x0E8, "Disconnection of Encoder A and B pulse"},
    {0x0E9, "Disconnection of Encoder A pulse"},
    {0x0EA, "Disconnection of Encoder B pulse"},
    {0x0EB, "Battery-less ABS problem"},
    {0x0EC, "Disconnection of Encoder PS pulse"},
    {0x0ED, "ABS error (1)"},
    {0x0EE, "ABS error (2)"},
    {0x0EF, "ABS error (3)"},
    {0x0F0, "Driver logic error"},
    {0x0F1, "Fieldbus link error"},
    {0x0F2, "Fieldbus module error"},
    {0x0F3, "Fieldbus module was not detected"},
    {0x0F4, "Mismatched PCB"},
    {0x0F5, "EEPROM WRITE verification error"},
    {0x0F6, "EEPROM WRITE timeout"},
    {0x0F7, "EEPROM writing error"},
    {0x0F8, "EEPROM Data Corrupt"},
    {0x0F9, "PLD error"},
    {0x0FA, "CPU error"},
    {0x0FB, "FPGA error"},
    {0x0FC, "Logic error"},
    {0x0FD, "Expanded device prroblem"},
    {0x842, "Detect driver board shutdown signal"},
    {0x843, "ABS charge circuit err."},
    {0x848, "Fan Round slow."},
    {0x849, "Time notice error."},
    {0x84A, "Detect RTC stop."},
    {0x84B, "RTC access error."},
    {0x84C, "Estimated life expectancy of the backup condensor for calendar function exceeded."},
    {0x850, "ERR C"},
    {0x855, "LC minor failure"},
    {0x860, "ERR T"},
    {0x861, "DB comm send error."},
    {0x862, "DB comm recive error."},
    {0x86A, "DB Parameter type mismatch."},
    {0x86B, "Driver board mode transition failure"},
    {0x86C, "Error in initial parameter reading of the motion driver"},
    {0x86D, "Power supply unit communication error"},
    {0x86E, "Driver parameter reading error"},
    {0x870, "Battery low voltage"},
    {0x871, "Battery unconnected"},
    {0x872, "Battery overheat"},
    {0x873, "Battery overcharge"},
    {0x880, "GW Parameter Error"},
    {0x881, "Parameter checksum error."},
    {0x890, "Driver-Board mount error."},
    {0x891, "Driver error."},
    {0x892, "Controller type mismatch."},
    {0x893, "Communication error for Driver board mode modification"},
    {0x894, "DB Parameter read error."},
    {0x895, "Motor power voltage error/monitoring timeout"},
    {0x896, "Connection Unit Configuration Error"},
    {0x89A, "FieldBus link error"},
    {0x89B, "FieldBus modure error"},
    {0x89C, "FieldBus modure not found"},
    {0x89D, "FieldBus Init timeout"},
    {0x89E, "Fan Failure"},
    {0x8A0, "Control voltage error"},
    {0x8A1, "Control voltage drop"},
    {0x8A2, "Motor power over"},
    {0x8A3, "Motor power drop"},
    {0x8A6, "ENC voltage drop"},
    {0x8A7, "Low Power Supply Voltage for External Wiring"},
    {0x8AA, "Regeneration circuit err"},
    {0x8AB, "Regeneration power over"},
    {0x8AC, "Regeneration overwork"},
    {0x8AE, "Detect power failure"},
    {0x8AF, "Motor power RY error"},
    {0x8B0, "Control voltage drop of the connected unit detected"},
    {0x8B9, "Driver parameter reading error during initialization"},
    {0x8BA, "Mismatch Number of axis / Operation mode"},
    {0x8BB, "RCON-EC Unit initial communication error"},
    {0x8BC, "Attached axis communication error"},
    {0x8BD, "Motion driver communication error"},
    {0x8BE, "Option unit initial communication error"},
    {0x8BF, "Internal communication error(CLK)"},
    {0x8CA, "Overheat"},
    {0x8D0, "Option unit connection quantity error"},
    {0x8D1, "Unsupported option unit connection"},
    {0x8D2, "RCON-EC Unit number of connection units error"},
    {0x8DC, "RCON-EC Unit communication error"},
    {0x8DD, "Unit connection confirmation signal error"},
    {0x8DE, "Driver unit communication error"},
    {0x8DF, "Option Unit communication error"},
    {0x8F0, "Ethernet initialization process error"},
    {0x8FA, "CPU error"},
    {0xE30, "Motor speed not reached"},
    {0xE48, "Driver Overload Warning"},
    {0xE4E, "Exceeded Suction Count Threshold"},
    {0xE4F, "Exceeded Total Motor Rotation Time Threshold"},
    {0xE5C, "Receive Timeout Error"},
    {0xE5E, "Delimiter Error Packet Receival"},
    {0xE6B, "Maintenance Information Data Error"},
    {0xE81, "Dropping of workpiece"},
    {0xE83, "Release Time Out"},
    {0xE9A, "I/O 24V Overcurrent"},
    {0xEA1, "Parameter Data Error"},
    {0xEA8, "Unsupported Motor / Encoder Types"},
    {0xEAB, "Abnormal Instructed Speed"},
    {0xEB0, "Motor instantaneous current exceeded"},
    {0xEB1, "Abnormal detection of motor phase B current"},
    {0xEB2, "Abnormal detection of motor phase A current"},
    {0xEB3, "Temperature sensor error"},
    {0xEB4, "Pressure sensor error"},
    {0xEB8, "Magnetic Pole Detection Error"},
    {0xEB9, "Magnetic Pole Detection Error 2"},
    {0xEC0, "Actual Speed Excess"},
    {0xEC1, "Servo Error"},
    {0xEC5, "Motor speed control command value not reached error"},
    {0xEC8, "Abnormal motor rated current"},
    {0xEC9, "Over-Voltage Error"},
    {0xECA, "Overheat Error"},
    {0xECB, "Motor current calibration process error"},
    {0xECC, "Control power supply voltage error"},
    {0xECE, "Drop in Control Supply Voltage"},
    {0xED4, "Motor Power Error"},
    {0xED8, "Deviation Overflow"},
    {0xEE0, "Overload"},
    {0xEE5, "Encoder Receipt Error"},
    {0xEF5, "Non-Volatile Memory Writing Verify Error"},
    {0xEF6, "Non-Volatile Memory Writing Timeout"},
    {0xEF8, "Non-Volatile Memory Data Destroyed"},
    {0xEF9, "RAM error"},
    {0xEFA, "CPU error"},
    {0xEFC, "Logic Error"},
    {0xEFD, "Switch error processing"},
    {0xEFE, "Abnormality in the oscillator circuit"},
    {0xFFF, "PowerUP No Error"},
};

const NameEntry kCuratedNames[] = {
    {0x080, "Motion command while in SERVO-OFF condition"},
    {0x083, "Absolute positional movement command when Homing is not completed"},
    {0x084, "Motion command during Homing"},
    {0x085, "Position No. error when moving"},
    {0x0A3, "Positional instruction information error when executing"},
    {0x0A7, "The instructed deceleration is abnormal"},
    {0x0AB, "The instructed speed is abnormal"},
    {0x0BE, "Time-out error while Homing"},
    {0x0C0, "Over speed error"},
    {0x0C1, "Servo error"},
    {0x0D8, "Deviation error"},
    {0x0D9, "Software stroke limit over error"},
    {0x0E0, "Over load"},
    {0x0EE, "ABS error (2)"},
    {0xFFF, "PowerUP (no error)"},
};

const NameEntry kConNames[] = {
    {0x02C, "Monitoring data type change command during monitoring"},
    {0x02D, "Monitoring related command in monitoring function invalid status"},
    {0x02E, "RTC related command in RTC invalid status"},
};

const NameEntry kGroups[] = {
    {0x048, "Warning"},
    {0x04A, "Warning"},
    {0x04E, "Warning"},
    {0x04F, "Warning"},
    {0x085, "F"},
    {0x09A, "E"},
    {0x0A0, "F"},
    {0x0A1, "F"},
    {0x0A2, "F"},
    {0x0A7, "A"},
    {0x0AF, "F"},
    {0x0B8, "A"},
    {0x0BA, "A"},
    {0x0BE, "A"},
    {0x0BF, "A"},
    {0x0C0, "A"},
    {0x0C1, "A"},
    {0x0C2, "A"},
    {0x0C8, "B"},
    {0x0C9, "E"},
    {0x0CA, "B"},
    {0x0CB, "C"},
    {0x0CC, "E"},
    {0x0CE, "E"},
    {0x0D2, "E"},
    {0x0D4, "E"},
    {0x0D5, "A"},
    {0x0D8, "A"},
    {0x0DC, "F"},
    {0x0E0, "A"},
    {0x0E4, "D"},
    {0x0E5, "D"},
    {0x0E6, "D"},
    {0x0E8, "D"},
    {0x0EB, "D"},
    {0x0ED, "D"},
    {0x0EE, "D"},
    {0x0F0, "C"},
    {0x0F5, "C"},
    {0x0F6, "C"},
    {0x0F8, "C"},
    {0x0FA, "C"},
    {0x0FC, "C"},
    {0x0FD, "C"},
};

const NameEntry kHomeNeeded[] = {
    {0x082, "home first"},
    {0x083, "home first"},
    {0x096, "the absolute reset was interrupted: redo it"},
    {0x0D5, "home return after the reset"},
    {0x0EE, "home lost: a home return after the reset"},
};

const NameEntry kSpecificRemedy[] = {
    {0x02C, "a monitoring data-type change was sent during monitoring: stop the monitoring first (ME0342-4H p.8-14)"},
    {0x02D, "a monitoring command was sent with monitoring disabled: No.112 selects the monitoring mode (ME0342-4H p.8-14)"},
    {0x02E, "a calendar (RTC) command was sent with the calendar disabled: No.111 = 1 enables it (ME0342-4H p.8-14)"},
    {0x049, "collision warning: the axis keeps running"},
    {0x05C, "a Modbus frame header arrived but no valid frame within 5 s (ME0342-4H p.8-15): check the wiring, the baud rate and electrical noise"},
    {0x069, "the calendar (RTC) clock stopped and the time is lost: set the clock again, or switch the calendar off with No.111 = 0 if it is not used; not registered in the alarm list (ME0342-4H p.8-15; this driver does not set the clock)"},
    {0x084, "a move was sent during the home return: wait for homing to finish"},
    {0x090, "a software reset was sent with the servo on: alarm reset; switch the servo off before a software reset"},
    {0x0A1, "parameter data error: e.g. soft limits in the wrong order, a rotary axis switched from index to normal mode with the - soft limit at 0, or No.10 (default positioning band) below the minimum band; correct the parameter, then software reset (ME0342-4H p.8-22)"},
    {0x0A2, "bad position data: the detail is the row, the address the field (see detail); causes include an empty row, a target outside the soft limits and a relative row in solenoid mode 2 (ME0342-4H p.8-17)"},
    {0x0A3, "bad direct-move data: the address names the field (see detail)"},
    {0x0A7, "the deceleration was lowered during a move so that the axis would overrun the soft limit: send the next command earlier or keep the deceleration (ME0342-4H p.8-18)"},
    {0x0C1, "the axis did not move for 2 s after a move command (blocked or held; also a cable fault, a brake not released or high sliding resistance, ME0342-4H p.8-19): free it, then alarm reset"},
    {0x0CE, "control supply voltage too low: check the 24 V supply and its capacity (treated as cold start to be safe; an alarm reset may already clear it)"},
    {0x0D2, "motor supply voltage too high: check the 24 V supply (treated as cold start to be safe; an alarm reset may already clear it)"},
    {0x0D9, "outside the soft limits: after the alarm reset move back inside them (slowly, or by hand)"},
    {0x0DF, "collision detected: alarm reset; the EC may run its evacuation move"},
    {0x0EE, "absolute position lost: absolute reset = alarm reset, servo on, home return (ME0342-4H p.4-130, p.8-21)"},
    {0x0F1, "fieldbus link lost (fieldbus types): check the network parameters, the master and the cable (ME0342-4H p.8-21)"},
};

const NameEntry kWarningClearing[] = {
    {0x048, "maintenance warning 3 (overload warning level): clears with an alarm reset; check why the load rose (ME3816-6B p.8-19, 8-27)"},
    {0x04A, "maintenance warning 4 (capacitor assumed life, 200 V models only): clears with an alarm reset but returns after every reboot (ME3816-6B p.8-19, 8-28)"},
    {0x04E, "maintenance warning 1 (total travel count): clears when the travel-count target in the maintenance information is raised or set to 0 (off) (ME3816-6B p.8-19, 8-26)"},
    {0x04F, "maintenance warning 2 (total travel distance): clears when the travel-distance target in the maintenance information is raised or set to 0 (off) (ME3816-6B p.8-19, 8-26)"},
};

const NameEntry kAbsDetails[] = {
    {0x001, "absolute battery voltage low"},
    {0x002, "encoder cable broken or connector contact fault"},
    {0x006, "position data lost during the absolute reset (battery-less absolute)"},
    {0x00B, "motor replaced: the pairing ID registered in the controller changed (battery-less absolute)"},
};

const NameEntry kRowFields[] = {
    {0x000, "position"},
    {0x002, "positioning band"},
    {0x004, "speed"},
    {0x006, "zone +"},
    {0x008, "zone -"},
    {0x00A, "acceleration"},
    {0x00B, "deceleration"},
    {0x00C, "push current"},
    {0x00D, "threshold"},
    {0x00E, "control flags"},
};

const NameEntry kCommandFields[] = {
    {0xD0A, "control signal 1"},
    {0xF00, "position"},
    {0xF02, "speed"},
    {0xF04, "acceleration"},
    {0xF06, "deceleration"},
    {0xF08, "positioning band"},
    {0xF0A, "positioning current"},
    {0xF0C, "push current"},
    {0xF0D, "control flags"},
    {0xF0E, "threshold"},
};

}  // namespace

const char* controller_name(int code) { return find(kControllerNames, sizeof(kControllerNames) / sizeof(kControllerNames[0]), code); }
const char* curated_name(int code) { return find(kCuratedNames, sizeof(kCuratedNames) / sizeof(kCuratedNames[0]), code); }
const char* con_name(int code) { return find(kConNames, sizeof(kConNames) / sizeof(kConNames[0]), code); }
const char* group_letter(int code) { return find(kGroups, sizeof(kGroups) / sizeof(kGroups[0]), code); }
const char* home_needed(int code) { return find(kHomeNeeded, sizeof(kHomeNeeded) / sizeof(kHomeNeeded[0]), code); }
const char* specific_remedy(int code) { return find(kSpecificRemedy, sizeof(kSpecificRemedy) / sizeof(kSpecificRemedy[0]), code); }
const char* warning_clearing(int code) { return find(kWarningClearing, sizeof(kWarningClearing) / sizeof(kWarningClearing[0]), code); }
const char* abs_error_detail(int code) { return find(kAbsDetails, sizeof(kAbsDetails) / sizeof(kAbsDetails[0]), code); }
const char* row_field(int code) { return find(kRowFields, sizeof(kRowFields) / sizeof(kRowFields[0]), code); }
const char* command_field(int code) { return find(kCommandFields, sizeof(kCommandFields) / sizeof(kCommandFields[0]), code); }

const char* group_name(const std::string& group) {
    struct G {
        const char* key;
        const char* text;
    };
    static const G table[] = {
        {"A", "overload"},
        {"B", "motor error"},
        {"C", "controller error"},
        {"D", "controller-encoder link error"},
        {"E", "power supply voltage / current error"},
        {"F", "data setting error (a grouping the PC tool shows; ME3816-6B p.8-19 lists groups A-E only)"},
        {"Warning", "maintenance warning"},
        {"Other", "other"},
    };
    for (const G& g : table)
        if (group == g.key) return g.text;
    return nullptr;
}

const char* group_advice(const std::string& group) {
    struct G {
        const char* key;
        const char* text;
    };
    static const G table[] = {
        {"A", "check for interference, a blocked or overloaded axis"},
        {"B", "the motor may need service; contact IAI if it recurs"},
        {"C", "the controller may need service; contact IAI if it recurs"},
        {"D", "power-cycle; if it recurs, the controller or motor may need service"},
        {"E", "check the 24 V supply voltage and its capacity"},
        {"F", "check the parameter / position data that was sent"},
        {"Warning", "a maintenance threshold was reached: plan maintenance"},
    };
    for (const G& g : table)
        if (group == g.key) return g.text;
    return nullptr;
}

const char* family_clearing(int code, const std::string& family) {
    // What clears 0x0CE / 0x0D2 / 0x0EE per CON family (DOC). The driver's level stays conservative (cold start for
    // 0x0CE / 0x0D2, by encoder type for 0x0EE); this text says what the manual of that family allows.
    struct F {
        int code;
        const char* family;
        const char* text;
    };
    static const F table[] = {
        {0x0CE, "PCON", "on a PCON-CB an alarm reset is enough (operation-cancel level, ME0342-4H p.8-19)"},
        {0x0CE, "ACON/DCON", "on an ACON / DCON-CB an alarm reset is enough (operation-cancel level, ME0343-3B p.8-18)"},
        {0x0CE, "SCON", "on an SCON-CB it needs a cold start: software reset or power cycle (ME0340-9F p.8-20)"},
        {0x0D2, "ACON/DCON", "on an ACON / DCON-CB an alarm reset is enough (operation-cancel level, ME0343-3B p.8-18)"},
        {0x0D2, "SCON", "on an SCON-CB it needs a cold start: software reset or power cycle (ME0340-9F p.8-20)"},
        {0x0EE, "PCON", "on a PCON-CB an alarm reset is enough (operation-cancel level, ME0342-4H p.8-21), then a home "
                        "return"},
        {0x0EE, "ACON/DCON", "on an ACON / DCON-CB an alarm reset is enough (operation-cancel level, ME0343-3B p.8-19), "
                             "then a home return"},
        {0x0EE, "SCON", "on an SCON-CB it needs a cold start: software reset or power cycle (ME0340-9F p.8-23), then a "
                        "home return"},
    };
    for (const F& f : table)
        if (f.code == code && family == f.family) return f.text;
    return nullptr;
}

std::vector<std::pair<int, const char*>> table_entries(const std::string& name, bool* known) {
    std::vector<std::pair<int, const char*>> out;
    *known = true;
#define IAD_TABLE(key, table)                                \
    if (name == key) {                                       \
        for (const NameEntry& e : table) out.emplace_back(e.code, e.text); \
        return out;                                          \
    }
    IAD_TABLE("controller_names", kControllerNames)
    IAD_TABLE("curated_names", kCuratedNames)
    IAD_TABLE("con_names", kConNames)
    IAD_TABLE("groups", kGroups)
    IAD_TABLE("home_needed", kHomeNeeded)
    IAD_TABLE("specific_remedies", kSpecificRemedy)
    IAD_TABLE("warning_clearing", kWarningClearing)
    IAD_TABLE("abs_error_details", kAbsDetails)
    IAD_TABLE("row_fields", kRowFields)
    IAD_TABLE("command_fields", kCommandFields)
#undef IAD_TABLE
    *known = false;
    return out;
}

const char* family_clearing_any() { return "the manuals allow an alarm reset on PCON / ACON-CB and ask for a cold start on SCON-CB (ME0342-4H p.8-19 / 8-21; ME0343-3B p.8-18 / 8-19; ME0340-9F p.8-20 / 8-23)"; }

}  // namespace alarm_data
}  // namespace iai_actuator_driver
