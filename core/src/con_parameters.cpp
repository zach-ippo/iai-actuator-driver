// The numbered parameters of the CON-family controllers (PCON / ACON / SCON / DCON ...), as IAI's PC tool numbers them:
// the table, the per-family ranges, decode / encode / parse / text and check_value. Port of con_params.py (C#
// ConParameters.cs). No I/O. The table rows are generated from the Python edition; the encodings of items that were not
// changed on hardware are UNVERIFIED (read on a PCON-CB + RCP6: values and item locations VERIFIED in the Python
// edition).
//
// The functions of namespace con_detail are internal to the model unit (declared in src/model_detail.hpp, used by
// parameter_plan.cpp); the public ones are con_parameters(), con_parameter() and is_con_home_parameter().
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <map>
#include <tuple>
#include <stdexcept>

#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "model_detail.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

// internal API (con_detail): declared in model_detail.hpp

namespace {

using con_detail::UnitLimits;

ConParam P(int no, const char* name, const char* symbol, int class_id, int item_id, const char* kind, int entry,
           double scale, bool is_signed, std::vector<int> bits, const char* unit, std::optional<double> lo,
           std::optional<double> hi, std::map<int, std::string> choices, bool dangerous, bool writable) {
    ConParam p;
    p.no = no;
    p.name = name;
    p.symbol = symbol;
    p.class_id = class_id;
    p.item_id = item_id;
    p.kind = kind;
    p.entry = entry;
    p.scale = scale;
    p.is_signed = is_signed;
    p.bits = std::move(bits);
    p.unit = unit;
    p.lo = lo;
    p.hi = hi;
    p.choices = std::move(choices);
    p.dangerous = dangerous;
    p.writable = writable;
    return p;
}

/// Every CON parameter definition with the PCON-CB ranges (DOC ME0342-4H p.6-2 to 6-6; details p.6-7 to 6-57).
const std::vector<ConParam>& table() {
    static const std::vector<ConParam> rows = {
    P(1, "Zone Output Position(1) +", "ZNM", 12, 1, "scaled", 0, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(2, "Zone Output Position(1) -", "ZNL", 12, 2, "scaled", 0, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(3, "Soft limit +", "LIMM", 7, 1, "scaled", 0, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(4, "Soft limit -", "LIML", 7, 2, "scaled", 0, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(5, "Home direction", "ORG", 9, 16, "parity", 0, 1.0, false, {}, "", 0.0, 1.0, {{0, "Reverse"}, {1, "Forward"}}, true, true),
    P(6, "Push recognition time", "PSWT", 9, 3, "raw", 0, 1.0, false, {}, "ms", 0.0, 9999.0, {}, false, true),
    P(7, "Servo gain selection", "PLG0", 9, 6, "raw", 0, 1.0, false, {}, "", 0.0, 31.0, {}, true, true),
    P(8, "Default velocity", "VCMD", 8, 10, "scaled", 0, 0.01, false, {}, "mm/s", 1.0, 9999.0, {}, false, true),
    P(9, "Default ACC", "ACMD", 8, 11, "scaled", 0, 0.01, false, {}, "G", 0.01, 9.99, {}, false, true),
    P(10, "Default position band", "INP", 8, 9, "scaled", 0, 0.01, false, {}, "mm", 0.01, 999.99, {}, false, true),
    P(12, "Default positioning current limit", "SPOW", 8, 12, "pct255", 0, 1.0, false, {}, "%", 0.0, 70.0, {}, false, true),
    P(13, "Default homing current limit", "ODPW", 9, 19, "pct255", 0, 1.0, false, {}, "%", 0.0, 100.0, {}, false, true),
    P(15, "STOP' Input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {7}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(16, "SIO Baudrate", "BRSL", 8, 3, "baud", 0, 1.0, false, {}, "bps", 0.0, 230400.0, {}, true, true),
    P(17, "Min delay for activating local transmitter", "RTIM", 8, 4, "raw", 0, 1.0, false, {}, "ms", 0.0, 255.0, {}, false, true),
    P(18, "Home Input Polarity", "AIOF", 9, 27, "pair3", 0, 1.0, false, {0, 1}, "", 0.0, 2.0, {{0, "Not Used"}, {1, "Normally open"}, {2, "Normally closed"}}, true, true),
    P(21, "ServoON' Input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {6}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(22, "Home offset", "OFST", 9, 21, "scaled", 0, 0.01, false, {}, "mm", 0.0, 9999.99, {}, true, true),
    P(23, "Zone Output Position(2) +", "ZNM", 12, 1, "scaled", 1, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(24, "Zone Output Position(2) -", "ZNL", 12, 2, "scaled", 1, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, false, true),
    P(25, "PIO pattern", "IOPN", 8, 2, "raw", 0, 1.0, true, {}, "", -1.0, 32767.0, {}, false, true),
    P(26, "PIO Jog Velocity", "IOJV", 8, 16, "scaled", 0, 0.01, false, {}, "mm/s", 1.0, 9999.0, {}, false, true),
    P(27, "Move command type", "FPIO", 8, 1, "bits", 0, 1.0, false, {16}, "", 0.0, 1.0, {{0, "Level"}, {1, "Edge"}}, false, true),
    P(28, "Pole sense initial moving direction", "PHSP", 9, 1, "bits", 0, 1.0, false, {16}, "", 0.0, 1.0, {{0, "Reverse"}, {1, "Forward"}}, true, true),
    P(29, "Excitation phase signal detection time", "PHSP", 9, 1, "bits", 0, 1.0, false, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, "ms", 1.0, 999.0, {}, true, true),
    P(30, "Excitation phase signal detection type", "PHSP", 9, 1, "bits", 0, 1.0, false, {17, 18}, "", 0.0, 2.0, {{0, "Conventional method"}, {1, "New method 1"}, {2, "New method 2"}}, true, true),
    P(31, "Velocity loop proportional gain", "VLPG", 9, 7, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(32, "Velocity loop integral gain", "VLPT", 9, 8, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(33, "Torque filter time constant", "TRQF", 9, 9, "raw", 0, 1.0, false, {}, "", 0.0, 2500.0, {}, true, true),
    P(34, "Push velocity", "PSHV", 8, 15, "scaled", 0, 0.01, false, {}, "mm/s", 1.0, 9999.0, {}, false, true),
    P(35, "Safety velocity", "SAFV", 8, 21, "scaled", 0, 0.01, false, {}, "mm/s", 1.0, 250.0, {}, false, true),
    P(36, "Automatic servo OFF delay time", "ASO1", 8, 20, "raw", 0, 1.0, false, {}, "s", 0.0, 9999.0, {}, false, true),
    P(37, "Automatic servo off delay time2", "ASO2", 8, 44, "raw", 0, 1.0, false, {}, "s", 0.0, 9999.0, {}, false, true),
    P(38, "Automatic servo off delay time3", "ASO3", 8, 45, "raw", 0, 1.0, false, {}, "s", 0.0, 9999.0, {}, false, true),
    P(39, "Positioning complete signal output method", "FPIO", 8, 1, "bits", 0, 1.0, false, {0}, "", 0.0, 1.0, {{0, "PEND"}, {1, "INP"}}, false, true),
    P(40, "Home input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {8}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(41, "Operation mode input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {15}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(42, "Enable function", "FPIO", 8, 1, "bits", 0, 1.0, false, {22}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(43, "Home confirmation sensor input polarity", "AIOF", 9, 27, "pair3", 0, 1.0, false, {6, 7}, "", 0.0, 2.0, {{0, "Not Used"}, {1, "Normally open"}, {2, "Normally closed"}}, true, true),
    P(45, "Silent interval magnification", "SIVM", 8, 19, "raw", 0, 1.0, false, {}, "", 0.0, 10.0, {}, false, true),
    P(46, "Velocity override", "OVRD", 8, 18, "pct255", 0, 1.0, false, {}, "%", 1.0, 100.0, {}, false, true),
    P(47, "PIO Jog Velocity2", "IOV2", 8, 22, "scaled", 0, 0.01, false, {}, "mm/s", 1.0, 250.0, {}, false, true),
    P(48, "PIO Inching Distance", "IOID", 8, 23, "scaled", 0, 0.01, false, {}, "mm", 0.01, 1.0, {}, false, true),
    P(49, "PIO Inching Distance2", "IOD2", 8, 24, "scaled", 0, 0.01, false, {}, "mm", 0.01, 1.0, {}, false, true),
    P(50, "Load output judgment time", "LDWT", 9, 25, "raw", 0, 1.0, false, {}, "ms", 0.0, 9999.0, {}, false, true),
    P(51, "Disable 'Range of torque authorization'", "FPIO", 8, 1, "bits", 0, 1.0, false, {25}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, false, true),
    P(52, "Addition and subtraction velocity mode initial value", "CTLF", 8, 14, "bits", 0, 1.0, false, {6, 7}, "", 0.0, 2.0, {{0, "Trapezoid"}, {1, "S-shaped motion"}, {2, "first order lag filter"}}, false, true),
    P(53, "Stop mode initial value", "CTLF", 8, 14, "bits", 0, 1.0, false, {8, 9, 10}, "", 0.0, 7.0, {}, false, true),
    P(55, "Position command primary filter time constant", "PLPF", 9, 39, "scaled", 0, 0.1, false, {}, "ms", 0.0, 100.0, {}, false, true),
    P(56, "Sigmoid motion ratio setting", "SCRV", 9, 40, "raw", 0, 1.0, false, {}, "%", 0.0, 100.0, {}, false, true),
    P(57, "Torque limiting value", "TQLM", 9, 36, "pct255", 0, 1.0, false, {}, "%", 0.0, 70.0, {}, false, true),
    P(58, "Clear deviation at servo off and alarm happens", "FSTP", 9, 15, "bits", 0, 1.0, false, {7}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, true, true),
    P(59, "Deviation err monitor when the trq is being limited", "FSTP", 9, 15, "bits", 0, 1.0, false, {9}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, true, true),
    P(60, "Deviation counter clear input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {12}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(61, "Torque limitation instruction input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {13}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(62, "Pulse count direction", "FPIO", 8, 1, "bits", 0, 1.0, false, {29}, "", 0.0, 1.0, {{0, "Forward"}, {1, "Reverse"}}, true, true),
    P(63, "Command pulse input mode", "CPMD", 9, 29, "bits", 0, 1.0, false, {0, 1}, "", 0.0, 2.0, {}, true, true),
    P(64, "Command pulse input mode polarity", "CPMD", 9, 29, "bits", 0, 1.0, false, {4}, "", 0.0, 1.0, {{0, "Positive"}, {1, "Negative"}}, true, true),
    P(65, "Electronic gear numerator", "CNUM", 9, 30, "raw", 0, 1.0, false, {}, "", 1.0, 4096.0, {}, true, true),
    P(66, "Electronic gear denominator", "CDEN", 9, 31, "raw", 0, 1.0, false, {}, "", 1.0, 4096.0, {}, true, true),
    P(67, "Compulsion stop input enable/disable selection", "FPIO", 8, 1, "bits", 0, 1.0, false, {14}, "", 0.0, 1.0, {{0, "Enable"}, {1, "Disable"}}, true, true),
    P(71, "Positional feedforward gain", "PLFG", 9, 33, "raw", 0, 1.0, false, {}, "", 0.0, 100.0, {}, true, true),
    P(77, "Screw lead", "LEAD", 7, 3, "scaled", 0, 0.01, false, {}, "mm", 0.01, 999.99, {}, true, true),
    P(78, "Axis action type", "ATYP", 7, 7, "bits", 0, 1.0, false, {8}, "", 0.0, 1.0, {{0, "Linear axis"}, {1, "Rotary axis"}}, true, true),
    P(79, "Rotary axis mode", "ATYP", 7, 7, "bits", 0, 1.0, false, {9}, "", 0.0, 1.0, {{0, "Normal mode"}, {1, "Index Mode"}}, true, true),
    P(80, "Rotary axis shortcut select", "ATYP", 7, 7, "bits", 0, 1.0, false, {10}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, true, true),
    P(83, "Absolute unit", "ETYP", 9, 11, "bits", 0, 1.0, false, {3}, "", 0.0, 1.0, {{0, "Not Used"}, {1, "Use"}}, true, true),
    P(84, "Fieldbus operation mode", "FMOD", 8, 46, "raw", 0, 1.0, false, {}, "", 0.0, 4.0, {}, true, true),
    P(85, "Fieldbus node address", "NADR", 8, 47, "raw", 0, 1.0, false, {}, "", 0.0, 65535.0, {}, true, true),
    P(86, "Fieldbus communication speed", "FBRS", 8, 48, "raw", 0, 1.0, false, {}, "", 0.0, 4.0, {}, true, true),
    P(87, "Network type", "NTYP", 8, 49, "raw", 0, 1.0, false, {}, "", 0.0, 9.0, {{0, "None"}, {1, "CC-Link"}, {2, "DeviceNet"}, {3, "PROFIBUS-DP"}, {4, "CompoNet"}, {5, "MECHATROLINK"}, {6, "EtherCAT"}, {7, "EtherNet/IP"}, {8, "PROFINET IO"}, {9, "MECHATROLINK III SV"}, {10, "MECHATROLINK III I/O"}, {11, "CC-Link IE Field"}, {12, "EtherCAT DriveProfile"}}, true, true),
    P(88, "Software limit margin", "SLMA", 7, 10, "scaled", 0, 0.01, false, {}, "mm", 0.0, 9999.99, {}, true, true),
    P(90, "Fieldbus input/output format", "FPIO", 8, 1, "bits", 0, 1.0, false, {26, 27}, "", 0.0, 3.0, {{0, "Byte / word swap disabled"}, {1, "Only byte swap"}, {2, "Only for word swap"}, {3, "Byte swap + word swap"}}, true, true),
    P(91, "Current limit value during stop after push failure", "FSTP", 9, 15, "bits", 0, 1.0, false, {3}, "", 0.0, 1.0, {{0, "Current limit value during motion"}, {1, "Current limit value during pressing"}}, false, true),
    P(110, "Stop method at SrvOff", "FSTP", 9, 15, "bits", 0, 1.0, false, {19}, "", 0.0, 1.0, {{0, "Sudden stop"}, {1, "Deceleration stop"}}, false, true),
    P(111, "Calendar Function", "FRTC", 8, 53, "bits", 0, 1.0, false, {0}, "", 0.0, 1.0, {{0, "Not Used"}, {1, "Use"}}, false, true),
    P(112, "Monitoring mode", "FMNT", 8, 54, "bits", 0, 1.0, false, {0, 1, 2, 3}, "", 0.0, 3.0, {{0, "Not Used"}, {1, "4CH Record Mode"}, {2, "8CH Record Mode"}, {3, "2CH Record Mode"}}, false, true),
    P(113, "Monitoring cycle", "FMNT", 8, 54, "bits", 0, 1.0, false, {16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31}, "ms", 1.0, 60000.0, {}, false, true),
    P(140, "IP address", "IPAD", 8, 55, "ipv4", 0, 1.0, false, {}, "", 0.0, 4294967295.0, {}, true, true),
    P(141, "Subnet mask", "SNMK", 8, 56, "ipv4", 0, 1.0, false, {}, "", 0.0, 4294967295.0, {}, true, true),
    P(142, "Default gateway", "DFGW", 8, 57, "ipv4", 0, 1.0, false, {}, "", 0.0, 4294967295.0, {}, true, true),
    P(143, "Overload warning load level ratio", "OLWL", 9, 102, "raw", 0, 1.0, false, {}, "%", 50.0, 100.0, {}, false, true),
    P(144, "Gain scheduling limit magnification", "GSUL", 9, 99, "raw", 0, 1.0, false, {}, "%", 0.0, 1023.0, {}, true, true),
    P(145, "Gain scheduling velocity loop proportional gain", "GSPC", 9, 100, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(146, "Gain scheduling velocity loop integral gain", "GSIC", 9, 101, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(147, "Total moving count threshold", "TMCT", 8, 59, "raw", 0, 1.0, false, {}, "", 0.0, 999999999.0, {}, false, true),
    P(148, "Total moving distance threshold", "ODOT", 8, 60, "raw", 0, 1.0, false, {}, "m", 0.0, 999999999.0, {}, false, true),
    P(149, "Zone output switch", "FPIO", 8, 1, "bits", 0, 1.0, false, {1}, "", 0.0, 1.0, {{0, "PZONE output select"}, {1, "ZONE1 / ZONE2 output select"}}, false, true),
    P(151, "Minor fault alarm output", "FSTP", 9, 15, "bits", 0, 1.0, false, {24}, "", 0.0, 1.0, {{0, "Output at overload warning"}, {1, "Message level alarm output"}}, false, true),
    P(152, "High output setting", "ATYP", 7, 7, "bits", 0, 1.0, false, {18}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, true, true),
    P(153, "Boost up velocity loop proportional gain", "BUPC", 9, 105, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(154, "Boost up velocity loop integral gain", "BUIC", 9, 106, "raw", 0, 1.0, false, {}, "", 1.0, 99999999.0, {}, true, true),
    P(155, "Battery maintenance", "ATYP", 7, 7, "bits", 0, 1.0, false, {30, 31}, "", 0.0, 3.0, {{0, "20 days"}, {1, "15 days"}, {2, "10 days"}, {3, "5 days"}}, true, true),
    P(156, "(Torque authorization)/(Minor failure) output switch", "SPIO", 8, 50, "bits", 0, 1.0, false, {24}, "", 0.0, 1.0, {{0, "Load judgment / torque level status signal output"}, {1, "Message level alarm output"}}, false, true),
    P(159, "FB Half Direct Value Mode velocity unit", "SPIO", 8, 50, "bits", 0, 1.0, false, {25}, "", 0.0, 1.0, {{0, "1mm/s"}, {1, "0.1mm/s"}}, false, true),
    P(165, "Delay-time after exiting shutdown", "SDDT", 8, 66, "raw", 0, 1.0, false, {}, "ms", 0.0, 100.0, {}, false, true),
    P(166, "Current limit expansion Function", "DCET", 9, 50, "flag", 0, 1.0, false, {}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, true, false),
    P(167, "Pulse train datum position", "RPOS", 7, 30, "scaled", 0, 0.01, true, {}, "mm", -9999.99, 9999.99, {}, true, true),
    P(168, "Collision detecting function", "SPIO", 8, 50, "bits", 0, 1.0, false, {26, 27, 28}, "", 0.0, 7.0, {}, false, true),
    P(182, "Automatic current-down", "FSTP", 9, 15, "bits", 0, 1.0, false, {26}, "", 0.0, 1.0, {{0, "Disable"}, {1, "Enable"}}, false, true),
    P(187, "Non-positioner mode command output completion judgement time", "CPMD", 9, 29, "bits", 0, 1.0, false, {16, 17, 18, 19, 20, 21, 22, 23}, "ms", 0.0, 255.0, {}, false, true),
    P(188, "Network Number", "NWNO", 8, 90, "raw", 0, 1.0, false, {}, "", 1.0, 239.0, {}, false, false),
    P(226, "Motor control switching threshold level", "SDDM", 9, 51, "raw", 0, 1.0, false, {}, "%", 0.0, 99999999.0, {}, true, true),
    };
    return rows;
}

/// The table with the decimal flags of the bounds the Python edition writes as floats although whole.
const std::vector<ConParam>& typed_table() {
    static const std::vector<ConParam> rows = [] {
        std::vector<ConParam> out = table();
        for (ConParam& p : out) {
            if (p.no == 22 || p.no == 55 || p.no == 88) p.lo_decimal = true;  // 0.0
            if (p.no == 48 || p.no == 49 || p.no == 55) p.hi_decimal = true;  // 1.0, 100.0
        }
        return out;
    }();
    return rows;
}

struct NoteRow {
    int no;
    const char* note;
};

/// A documented caution shown with the parameter.
const NoteRow kNotes[] = {
    {34, "keep at least 5 mm/s; IAI asks to be consulted before it is changed; a row speed below it pushes at the row speed (ME0342-4H p.6-23)"},
    {35, "at most the actuator maximum speed when that is below 250 (ME0342-4H p.6-23)"},
    {39, "changes what PEND means (ME0342-4H p.4-37, 6-24)"},
    {46, "scales stored-position moves only, not 0x9900 direct moves (ME0342-4H p.6-26)"},
};

// IAI baud code <-> bit/s (the SIO baud-rate parameter). Every code decodes; a CON write takes the rates below only.
struct BaudCode {
    int code;
    int rate;
};
const BaudCode kBaudCodes[] = {{4, 9600},   {5, 19200},  {6, 38400},   {7, 76800},    {17, 14400},
                               {18, 28800}, {19, 57600}, {20, 115200}, {21, 230400},  {256, 460800}};
// The eight SIO rates No.16 takes on a CON-CB controller (DOC ME0342-4H p.6-14; ME0343-3B p.6-14; ME0340-9F p.6-16).
const int kConBaudRates[] = {9600, 14400, 19200, 28800, 38400, 76800, 115200, 230400};

struct Range {
    int no;
    double lo, hi;
};
// Ranges that differ from the PCON-CB table, per family (DOC ME0343-3B p.6-2 to 6-6; ME0340-9F p.6-2 to 6-9).
const Range kAconRanges[] = {{10, 0.01, 9999.99}, {13, 0, 300}, {29, 50, 999}, {31, 1, 27661},
                             {32, 1, 217270},     {53, 0, 3},   {87, 0, 11}};
const Range kSconRanges[] = {{10, 0.01, 9999.99}, {13, 0, 300}, {53, 0, 3}, {65, 1, 99999999}, {66, 1, 99999999},
                             {84, 0, 8},          {87, 0, 11},  {113, 1, 1000}, {165, 0, 10000}};

const char* const kUnknownFamilyNote =
    "the unit's family has no range table of its own in the driver: the PCON-CB range (ME0342-4H p.6-2 to 6-6) is used";
constexpr double kPushSpeedFloorMmS = 5.0;  // No.34: "the limit should be 5 mm/s" (ME0342-4H p.6-23): a warning
constexpr int kEncoderPulsesItem = 12;      // class 9 item 12 EPLS: encoder pulses / rev (8192 on the RCP6)

bool is_whole(double value) { return std::isfinite(value) && value == std::floor(value); }

int decimals_of(double scale) {
    const std::string text = util::format("%g", scale);
    const std::size_t dot = text.find('.');
    return dot == std::string::npos ? 0 : static_cast<int>(text.size() - dot - 1);
}

double round_to(double value, int decimals) {
    const double f = std::pow(10.0, decimals);
    return std::round(value * f) / f;
}

std::int32_t as_signed(std::uint32_t raw) { return static_cast<std::int32_t>(raw); }

/// pct255 field mask: the low byte for ranges up to 100 %, 16 bits above (300 % = 765; IAI's 255 = 100 % scale
/// continued, as PPOW does up to 0x1FE = 200 %; UNVERIFIED for these parameters).
std::uint32_t pct_mask(const ConParam& p) { return !p.hi || *p.hi <= 100 ? 0xFFu : 0xFFFFu; }

std::string choices_text(const ConParam& p) {
    std::string out = "{";
    bool first = true;
    for (const auto& kv : p.choices) {
        if (!first) out += ", ";
        first = false;
        out += std::to_string(kv.first) + ": '" + kv.second + "'";
    }
    return out + "}";
}

std::string range_text(const ConParam& p) {
    return util::py_number(p.lo.value_or(0), p.lo_decimal) + ".." + util::py_number(p.hi.value_or(0), p.hi_decimal) +
           (p.unit.empty() ? "" : " " + p.unit);
}

/// A parsed value as Python prints it: a float for a scaled parameter, an int otherwise.
std::string value_text(const ConParam& p, double number) { return util::py_number(number, p.kind == "scaled"); }

/// "No.16 takes only": the documented rates.
std::string baud_rates_text() {
    std::string out;
    for (std::size_t i = 0; i < sizeof(kConBaudRates) / sizeof(kConBaudRates[0]); ++i)
        out += (i ? ", " : "") + std::to_string(kConBaudRates[i]);
    return out;
}

/// IAI's category D / E (actuator-specific or manufacturer use: never change) of this family (DOC ME0342-4H p.6-2).
bool category_is_d_or_e(int no, const std::optional<std::string>& fam) {
    static const int kDE[] = {5, 77, 78, 88, 18, 58, 87};
    for (int n : kDE)
        if (n == no) return true;
    if (fam && *fam == "SCON") {  // ME0340-9F p.6-2: 14 / 19 / 20 / 72 (E), 73 / 75 / 76 / 89 (D)
        static const int kScon[] = {14, 19, 20, 72, 73, 75, 76, 89};
        for (int n : kScon)
            if (n == no) return true;
    }
    return false;
}

struct MinBand {
    double mm = 0.0;
    std::string formula;
    std::string doc;
    bool certain = true;
};

double ceil_hundredth(double value) { return std::ceil(std::round(value * 100.0 * 1e6) / 1e6) / 100.0; }

/// The smallest positioning band (No.10 and the rows' band) the unit accepts, in 0.01 mm, rounded up: lead / pulses x
/// factor (DOC ME0342-4H p.6-12, p.4-16; ME0343-3B p.4-13; ME0340-9F p.4-20, 4-93). nullopt when neither the lead nor
/// MINI is known; unknown encoder pulses -> 800 assumed and certain = false.
std::optional<MinBand> min_positioning_band(const std::optional<std::string>& fam, std::optional<double> lead_mm,
                                            std::optional<std::uint32_t> encoder_pulses,
                                            std::optional<double> mini_mm) {
    struct Floor {
        double mm;
        std::string formula;
        std::string doc;
    };
    std::vector<Floor> floors;
    floors.push_back({0.01, "0.01 mm (the input range)", "ME0342-4H p.6-12"});
    bool certain = true;
    if (mini_mm && *mini_mm != 0.0) floors.push_back({*mini_mm, "MINI (3.67), enforced by this unit", "the unit's own limit"});
    if (lead_mm && *lead_mm != 0.0) {
        const unsigned pulses = encoder_pulses && *encoder_pulses != 0 ? *encoder_pulses : 800u;
        certain = encoder_pulses.has_value();
        const bool low_res = pulses < 4096;
        int factor;
        std::string doc;
        if (fam && *fam == "SCON") {
            factor = 1;
            doc = "ME0340-9F p.4-20, 4-93";
        } else if (fam && *fam == "ACON/DCON") {
            factor = low_res ? 4 : 1;
            doc = low_res ? "ME0343-3B p.4-13" : "ME0343-3B p.8-21";
        } else {
            factor = low_res ? 3 : 1;
            doc = low_res ? "ME0342-4H p.6-12" : "ME0342-4H p.6-12, p.4-16";
            if (!fam) doc += "; the PCON rule (the family has no rule of its own in the driver)";
        }
        const double band = ceil_hundredth(*lead_mm / pulses * factor);
        std::string how = "lead " + util::format("%g", *lead_mm) + " / " + std::to_string(pulses) + " pulses" +
                          (factor != 1 ? " x " + std::to_string(factor) : std::string());
        if (!certain) how += " (encoder count not known: 800 assumed)";
        floors.push_back({band, how, doc});
    } else if (!(mini_mm && *mini_mm != 0.0)) {
        return std::nullopt;
    }
    const Floor* best = &floors[0];
    for (const Floor& f : floors)
        if (f.mm > best->mm) best = &f;
    MinBand out;
    out.mm = util::round2(best->mm);
    out.formula = best->formula;
    out.doc = best->doc;
    out.certain = certain || util::starts_with(best->formula, "0.01") || util::starts_with(best->formula, "MINI");
    return out;
}

}  // namespace

// ------------------------------------------------------------------------------------------ public

std::string ConParam::key() const {
    const std::string cls = class_id > 9 ? util::format("%X", class_id) : std::to_string(class_id);
    return cls + "." + std::to_string(item_id) + (entry != 0 ? "[" + std::to_string(entry) + "]" : std::string());
}

std::vector<ConParam> con_parameters(const std::string& family) {
    std::vector<ConParam> out;
    for (const ConParam& p : typed_table()) out.push_back(con_detail::for_family(p, family));
    return out;
}

const ConParam* con_parameter(int no) {
    for (const ConParam& p : typed_table())
        if (p.no == no) return &p;
    return nullptr;
}

bool is_con_home_parameter(int no) { return no == 5 || no == 22; }

// ------------------------------------------------------------------------------------------ internal
namespace con_detail {

std::optional<std::string> family_of(const std::string& family) {
    const std::string name = util::upper(util::trim(family));
    if (name == "ACON" || name == "DCON" || name == "ACON/DCON") return std::string("ACON/DCON");
    if (name == "PCON" || name == "SCON") return name;
    return std::nullopt;
}

ConParam for_family(const ConParam& p, const std::string& family) {
    const std::optional<std::string> fam = family_of(family);
    if (!fam) return p;
    ConParam out = p;
    const Range* ranges = *fam == "SCON" ? kSconRanges : *fam == "ACON/DCON" ? kAconRanges : nullptr;
    const std::size_t count = *fam == "SCON" ? sizeof(kSconRanges) / sizeof(kSconRanges[0])
                              : *fam == "ACON/DCON" ? sizeof(kAconRanges) / sizeof(kAconRanges[0])
                                                    : 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (ranges[i].no == p.no) {
            out.lo = ranges[i].lo;
            out.hi = ranges[i].hi;
            out.lo_decimal = false;  // the family ranges are written as ints, or as decimals that are not whole
            out.hi_decimal = false;
        }
    }
    if (category_is_d_or_e(p.no, fam) && !out.dangerous) out.dangerous = true;
    return out;
}

std::string note_of(int no) {
    for (const NoteRow& row : kNotes)
        if (row.no == no) return row.note;
    return std::string();
}

std::pair<int, int> baud_range() { return {kConBaudRates[0], kConBaudRates[sizeof(kConBaudRates) / sizeof(kConBaudRates[0]) - 1]}; }

ParamValue decode(const ConParam& p, std::uint32_t raw) {
    const std::string& kind = p.kind;
    if (kind == "scaled") {
        const double value = (p.is_signed ? static_cast<double>(as_signed(raw)) : static_cast<double>(raw)) * p.scale;
        return ParamValue::of(util::py_round(value, decimals_of(p.scale)));
    }
    // every kind but "scaled" is an int in the Python edition (of_whole: written as an integer in the JSON)
    if (kind == "raw")
        return ParamValue::of_whole(p.is_signed ? static_cast<double>(as_signed(raw)) : static_cast<double>(raw));
    if (kind == "pct255") return ParamValue::of_whole(std::nearbyint((raw & pct_mask(p)) * 100.0 / 255.0));
    if (kind == "bits") {
        std::uint32_t value = 0;
        for (std::size_t j = 0; j < p.bits.size(); ++j) value |= ((raw >> p.bits[j]) & 1u) << j;
        return ParamValue::of_whole(static_cast<double>(value));
    }
    if (kind == "pair3") {
        std::uint32_t pair = 0;
        for (std::size_t j = 0; j < 2 && j < p.bits.size(); ++j) pair |= ((raw >> p.bits[j]) & 1u) << j;
        return ParamValue::of_whole(pair == 3 ? 2.0 : pair == 1 ? 1.0 : 0.0);
    }
    if (kind == "baud") {
        for (const BaudCode& b : kBaudCodes)
            if (static_cast<std::uint32_t>(b.code) == raw) return ParamValue::of_whole(b.rate);
        return ParamValue();
    }
    if (kind == "parity") {
        if (raw == 0 || raw == 0xFFFFFFFFu) return ParamValue();
        return ParamValue::of_whole(raw & 1u);
    }
    if (kind == "ipv4") {
        ParamValue v;
        v.text = util::format("%u.%u.%u.%u", (raw >> 24) & 0xFF, (raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF);
        return v;
    }
    return ParamValue::of_whole(raw ? 1.0 : 0.0);  // flag
}

std::string text(const ConParam& p, const ParamValue& value) {
    if (value.empty()) return "not set";
    if (value.number) {
        const double n = *value.number;
        if (p.kind != "scaled" && is_whole(n)) {
            const auto it = p.choices.find(static_cast<int>(n));
            if (it != p.choices.end()) return std::to_string(it->first) + ":" + it->second;
        }
        if (p.kind == "scaled")
            return util::fixed(n, decimals_of(p.scale)) + (p.unit.empty() ? std::string() : " " + p.unit);
        return util::g(n) + (p.unit.empty() ? std::string() : " " + p.unit);
    }
    return value.text + (p.unit.empty() ? std::string() : " " + p.unit);
}

ParamValue parse(const ConParam& p, const ParamValue& value) {
    if (p.kind == "ipv4") {
        if (value.number) {
            const double n = *value.number;
            if (!is_whole(n) || n < 0 || n > 4294967295.0)
                throw std::invalid_argument(util::format("No.%d: %s is not an IPv4 address", p.no, util::g(n).c_str()));
            const std::uint32_t raw = static_cast<std::uint32_t>(n);
            ParamValue v;
            v.text = util::format("%u.%u.%u.%u", (raw >> 24) & 0xFF, (raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF);
            return v;
        }
        const std::vector<std::string> parts = util::split(util::trim(value.text), '.');
        bool ok = parts.size() == 4;
        std::string joined;
        for (std::size_t i = 0; ok && i < parts.size(); ++i) {
            const std::string part = util::trim(parts[i]);
            bool digits = !part.empty();
            for (char c : part)
                if (c < '0' || c > '9') digits = false;
            const auto n = digits ? util::parse_int(part) : std::nullopt;
            if (!n || *n < 0 || *n > 255) ok = false;
            else joined += (i ? "." : "") + std::to_string(*n);
        }
        if (!ok) throw std::invalid_argument(util::format("No.%d: '%s' is not an IPv4 address", p.no, value.text.c_str()));
        ParamValue v;
        v.text = joined;
        return v;
    }

    double number = 0.0;
    if (value.number) {
        number = *value.number;
    } else {
        const std::string text_value = util::trim(value.text);
        const std::size_t colon = text_value.find(':');
        const std::string head = util::trim(colon == std::string::npos ? text_value : text_value.substr(0, colon));
        const std::string label =
            util::lower(util::trim(colon == std::string::npos ? text_value : text_value.substr(colon + 1)));
        std::vector<int> matches;
        for (const auto& kv : p.choices) {
            const std::string name = util::lower(kv.second);
            if (name == label || (!label.empty() && util::starts_with(name, label))) matches.push_back(kv.first);
        }
        // the part before a colon is a number ("1:Forward"); a bare number was already parsed
        std::string digits = head;
        if (!digits.empty() && digits[0] == '-') digits.erase(0, 1);
        const std::size_t dot = digits.find('.');
        if (dot != std::string::npos) digits.erase(dot, 1);
        bool numeric = !digits.empty();
        for (char c : digits)
            if (c < '0' || c > '9') numeric = false;
        if (numeric) {
            number = util::parse_double(head).value_or(0.0);
        } else if (matches.size() == 1) {
            number = matches[0];
        } else {
            throw std::invalid_argument(util::format("No.%d: '%s' is not a number or one of %s", p.no, value.text.c_str(),
                                                     p.choices.empty() ? "the values" : choices_text(p).c_str()));
        }
    }
    if (!std::isfinite(number))
        throw std::invalid_argument(util::format("No.%d: %s is not a finite number", p.no,
                                                 value.number ? util::py_float(number).c_str()
                                                              : ("'" + value.text + "'").c_str()));
    if (p.kind == "scaled") {
        number = util::py_round(number, decimals_of(p.scale));
    } else if (!is_whole(number)) {
        throw std::invalid_argument(util::format("No.%d: %s must be a whole number", p.no,
                                                 value.number ? util::py_float(number).c_str()
                                                              : ("'" + value.text + "'").c_str()));
    }
    if (p.kind == "baud") {
        bool known = false;
        for (int rate : kConBaudRates)
            if (rate == static_cast<int>(number)) known = true;
        if (!known)
            throw std::invalid_argument(util::format(
                "No.%d: %s bit/s is not one of the documented SIO rates %s (ME0342-4H p.6-14)", p.no,
                value_text(p, number).c_str(), baud_rates_text().c_str()));
        return ParamValue::of(number);
    }
    if (!p.choices.empty() && (p.kind == "bits" || p.kind == "pair3" || p.kind == "parity" || p.kind == "raw") &&
        p.choices.find(static_cast<int>(number)) == p.choices.end())
        throw std::invalid_argument(util::format("No.%d: %s is not one of %s", p.no, value_text(p, number).c_str(),
                                                 choices_text(p).c_str()));
    if (p.lo && p.hi && !(*p.lo - 1e-9 <= number && number <= *p.hi + 1e-9))
        throw std::invalid_argument(
            util::format("No.%d: %s outside %s", p.no, value_text(p, number).c_str(), range_text(p).c_str()));
    return ParamValue::of(number);
}

std::uint32_t encode(const ConParam& p, std::uint32_t raw, const ParamValue& value) {
    const std::string& kind = p.kind;
    const double number = value.number.value_or(0.0);
    if (kind == "scaled") {
        const double counts = std::nearbyint(number / p.scale);
        if (!(counts >= -2147483648.0 && counts < 4294967296.0))
            throw std::invalid_argument(util::format("No.%d: %s cannot be encoded", p.no, util::g(number).c_str()));
        return static_cast<std::uint32_t>(static_cast<std::int64_t>(counts) & 0xFFFFFFFFll);
    }
    if (kind == "raw") return static_cast<std::uint32_t>(static_cast<std::int64_t>(number) & 0xFFFFFFFFll);
    if (kind == "pct255") {
        const std::uint32_t mask = pct_mask(p);
        return (raw & ~mask) | ((static_cast<std::uint32_t>(number) * 255u / 100u) & mask);
    }
    if (kind == "bits" || kind == "pair3") {
        std::vector<int> bits = p.bits;
        if (kind == "pair3" && bits.size() > 2) bits.resize(2);
        const std::uint32_t field = kind == "pair3" && static_cast<int>(number) == 2 ? 3u : static_cast<std::uint32_t>(number);
        if (bits.size() < 32 && (field >> bits.size()) != 0)
            throw std::invalid_argument(
                util::format("No.%d: %s does not fit %d bit(s)", p.no, util::g(number).c_str(), static_cast<int>(bits.size())));
        for (std::size_t j = 0; j < bits.size(); ++j)
            raw = (raw & ~(1u << bits[j])) | (((field >> j) & 1u) << bits[j]);
        return raw;
    }
    if (kind == "baud") {
        for (const BaudCode& b : kBaudCodes)
            if (b.rate == static_cast<int>(number)) return static_cast<std::uint32_t>(b.code);
        throw std::invalid_argument(util::format("No.%d: %s bit/s has no baud code", p.no, util::g(number).c_str()));
    }
    if (kind == "parity") {
        if (raw == 0 || raw == 0xFFFFFFFFu)
            throw std::invalid_argument(util::format("No.%d: the home pattern is 0x%08X: not settable", p.no, raw));
        if ((raw & 1u) == static_cast<std::uint32_t>(number)) return raw;
        return static_cast<int>(number) == 0 ? raw + 1 : raw - 1;
    }
    if (kind == "ipv4") {
        unsigned a = 0, b = 0, c = 0, d = 0;
        if (std::sscanf(value.text.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4)
            throw std::invalid_argument(util::format("No.%d: '%s' is not an IPv4 address", p.no, value.text.c_str()));
        return (a << 24) | (b << 16) | (c << 8) | d;
    }
    throw std::invalid_argument(util::format("No.%d (%s) is read-only", p.no, kind.c_str()));
}

UnitLimits unit_limits(const std::map<int, ClassBlock>& classes,
                       const std::optional<std::map<std::string, std::uint32_t>>& support) {
    // unset items (0 / FFFFFFFF) count as not known
    auto raw = [&](int class_id, int item_id) -> std::optional<std::uint32_t> {
        const auto it = classes.find(class_id);
        if (it == classes.end()) return std::nullopt;
        const ClassItem* item = it->second.item(item_id);
        if (item == nullptr || item->raw == 0 || item->raw == 0xFFFFFFFFu) return std::nullopt;
        return item->raw;
    };
    auto scaled = [&](int class_id, int item_id) -> std::optional<double> {
        const auto value = raw(class_id, item_id);
        if (!value || (*value & 0x80000000u) != 0) return std::nullopt;
        return util::round2(*value * 0.01);
    };
    const std::uint32_t atyp = raw(reg::kLimitParamsCategory, reg::kAxisItemAtyp).value_or(0);
    std::optional<std::uint32_t> sup2;
    if (support) {
        const auto it = support->find("SUP2");
        if (it != support->end()) sup2 = it->second;
    }
    const auto mini = raw(reg::kControllerMonitorCategory, reg::kControllerItemMini);
    const bool enforced = sup2 && (*sup2 & reg::kSup2Cinp) != 0 && (atyp & reg::kAtypRinp) != 0;
    UnitLimits out;
    out.max_speed = scaled(reg::kLimitParamsCategory, reg::kAxisItemMaxv);
    out.max_accel = scaled(reg::kLimitParamsCategory, reg::kAxisItemMaxa);
    out.max_push_speed = scaled(reg::kLimitParamsCategory, reg::kAxisItemMxph);
    out.lead_mm = scaled(reg::kLimitParamsCategory, reg::kAxisItemLead);
    out.encoder_pulses = raw(reg::kFactoryParamsCategory, kEncoderPulsesItem);
    if (enforced && mini) out.min_band_mm = util::round2(*mini * 0.01);
    out.rotary = (atyp & reg::kAtypRotary) != 0;
    return out;
}

Check check_value(const ConParam& p, const ParamValue& value, const std::string& family, const UnitLimits& unit) {
    const std::optional<std::string> fam = family_of(family);
    const ConParam q = for_family(p, family);
    const ParamValue number_value = parse(q, value);
    std::vector<std::string> warnings;
    if (!fam) warnings.push_back(kUnknownFamilyNote);
    if (number_value.number) {
        const double number = *number_value.number;
        // parameters capped by the unit's own maxima (DOC ME0342-4H p.6-2, 6-23, 6-26)
        std::optional<double> cap;
        const char* cap_text = "";
        switch (q.no) {
            case 8: case 26: case 35: case 47:
                cap = unit.max_speed;
                cap_text = "the actuator's maximum speed MAXV (7.4)";
                break;
            case 9:
                cap = unit.max_accel;
                cap_text = "the actuator's maximum acceleration MAXA (7.5)";
                break;
            case 34:
                cap = unit.max_push_speed;
                cap_text = "the actuator's maximum push speed MXPH (7.8)";
                break;
            default: break;
        }
        if (cap && number > *cap + 1e-9)
            throw std::invalid_argument(util::format("No.%d: %s %s is above %s %s %s (ME0342-4H p.6-2)", q.no,
                                                     value_text(q, number).c_str(), q.unit.c_str(), cap_text,
                                                     util::format("%g", *cap).c_str(), q.unit.c_str()));
        if (q.no == 10) {
            const std::optional<MinBand> band = min_positioning_band(fam, unit.lead_mm, unit.encoder_pulses, unit.min_band_mm);
            if (band && number < band->mm - 1e-9) {
                const std::string why = util::format(
                    "No.10: %.2f mm is below this unit's minimum positioning band %.2f mm (%s; %s); the next software "
                    "reset would raise cold-start alarm 0x0A1 (ME0342-4H p.8-22)",
                    number, band->mm, band->formula.c_str(), band->doc.c_str());
                if (band->certain) throw std::invalid_argument(why);
                warnings.push_back(why);
            }
        }
        if (q.no == 34) {
            if (number < kPushSpeedFloorMmS)
                warnings.push_back(util::format(
                    "No.34: %s mm/s is below %s mm/s, the slowest push speed IAI documents; the push force may not be "
                    "reached (ME0342-4H p.6-23)",
                    util::format("%g", number).c_str(), util::format("%g", kPushSpeedFloorMmS).c_str()));
            warnings.push_back("No.34: IAI asks to be consulted before the push speed is changed (ME0342-4H p.6-23)");
        }
    }
    const std::string note = note_of(q.no);
    if (!note.empty() && (q.no == 39 || q.no == 46 || q.no == 35))
        warnings.push_back("No." + std::to_string(q.no) + ": " + note);
    Check out;
    out.value = number_value;
    out.warnings = warnings;
    out.family = fam;
    out.lo = q.lo;
    out.hi = q.hi;
    return out;
}

// ------------------------------------------------------------------------------------------ the tables (bindings)

std::vector<std::pair<int, int>> baud_codes() {
    std::vector<std::pair<int, int>> out;
    for (const BaudCode& b : kBaudCodes) out.emplace_back(b.code, b.rate);
    return out;
}

std::vector<int> con_baud_rates() { return std::vector<int>(std::begin(kConBaudRates), std::end(kConBaudRates)); }

std::vector<std::string> families() { return {"PCON", "ACON/DCON", "SCON"}; }

std::vector<std::tuple<int, double, double>> family_ranges(const std::string& family) {
    std::vector<std::tuple<int, double, double>> out;
    if (family == "ACON/DCON")
        for (const Range& r : kAconRanges) out.emplace_back(r.no, r.lo, r.hi);
    else if (family == "SCON")
        for (const Range& r : kSconRanges) out.emplace_back(r.no, r.lo, r.hi);
    return out;
}

std::string range_doc(const std::optional<std::string>& fam) {
    const std::string f = fam.value_or("PCON");
    if (f == "ACON/DCON") return "ME0343-3B p.6-2 to 6-6 (No.53: p.4-17 and the No.53 detail page)";
    if (f == "SCON") return "ME0340-9F p.6-2 to 6-9";
    if (f == "PCON") return "ME0342-4H p.6-2 to 6-6";
    return std::string();
}

std::string unknown_family_note() { return kUnknownFamilyNote; }

namespace {
// IAI's category per parameter number (DOC ME0342-4H p.6-2 to 6-6).
const std::map<int, const char*>& category_table() {
    static const std::map<int, const char*> kCategories = {
        {1, "B"},   {2, "B"},   {3, "A"},   {4, "A"},   {5, "D"},   {6, "C"},   {7, "C"},   {8, "B"},   {9, "B"},
        {10, "B"},  {12, "B"},  {13, "C"},  {15, "B"},  {16, "B"},  {17, "B"},  {18, "E"},  {21, "B"},  {22, "C"},
        {23, "B"},  {24, "B"},  {25, "A"},  {26, "B"},  {27, "B"},  {28, "B"},  {29, "B"},  {30, "B"},  {31, "C"},
        {32, "C"},  {33, "C"},  {34, "C"},  {35, "C"},  {36, "B"},  {37, "B"},  {38, "B"},  {39, "B"},  {40, "C"},
        {41, "C"},  {42, "C"},  {43, "B"},  {45, "B"},  {46, "B"},  {47, "B"},  {48, "B"},  {49, "B"},  {50, "C"},
        {51, "B"},  {52, "B"},  {53, "B"},  {55, "B"},  {56, "B"},  {57, "B"},  {58, "E"},  {59, "C"},  {60, "B"},
        {61, "B"},  {62, "B"},  {63, "B"},  {64, "B"},  {65, "B"},  {66, "B"},  {67, "B"},  {71, "B"},  {77, "D"},
        {78, "D"},  {79, "B"},  {80, "B"},  {83, "B"},  {84, "A"},  {85, "A"},  {86, "A"},  {87, "E"},  {88, "D"},
        {90, "C"},  {91, "C"},  {110, "B"}, {111, "B"}, {112, "B"}, {113, "B"}, {140, "B"}, {141, "B"}, {142, "B"},
        {143, "B"}, {144, "B"}, {145, "C"}, {146, "C"}, {147, "B"}, {148, "B"}, {149, "B"}, {151, "B"}, {152, "B"},
        {153, "B"}, {154, "B"}, {155, "A"}, {156, "B"}, {159, "B"}, {165, "B"}, {166, "B"}, {167, "B"}, {168, "B"},
        {182, "B"}, {187, "C"}, {188, "A"},
    };
    return kCategories;
}

// The SCON-only numbers (ME0340-9F p.6-2): 14 / 19 / 20 / 72 (E), 73 / 75 / 76 / 89 (D).
const std::map<int, const char*>& scon_category_table() {
    static const std::map<int, const char*> kScon = {{14, "E"}, {19, "E"}, {20, "E"}, {72, "E"},
                                                     {73, "D"}, {75, "D"}, {76, "D"}, {89, "D"}};
    return kScon;
}

// PCON-CB factory defaults that do not depend on the actuator (DOC ME0342-4H p.6-2 to 6-6; No.42 reads 1 there but 0
// on p.6-25); `decimal` = a decimal value (No.48 1.00, No.49 0.10, No.55 0.0).
struct Default {
    int no;
    double value;
    bool decimal;
};
const Default kDefaults[] = {
    {15, 0, false},   {16, 38400, false}, {17, 5, false},     {21, 0, false},   {25, 0, false},   {27, 0, false},
    {29, 10, false},  {30, 1, false},     {35, 100, false},   {36, 0, false},   {37, 0, false},   {38, 0, false},
    {39, 0, false},   {40, 0, false},     {41, 0, false},     {42, 1, false},   {45, 0, false},   {46, 100, false},
    {48, 1.00, true}, {49, 0.10, true},   {50, 255, false},   {51, 0, false},   {52, 0, false},   {53, 0, false},
    {55, 0.0, true},  {56, 0, false},     {57, 70, false},    {58, 1, false},   {59, 0, false},   {60, 0, false},
    {61, 0, false},   {63, 1, false},     {64, 0, false},     {65, 200, false}, {66, 15, false},  {67, 0, false},
    {71, 0, false},   {110, 0, false},    {111, 1, false},    {112, 1, false},  {113, 1, false},  {143, 100, false},
    {144, 0, false},  {147, 0, false},    {148, 0, false},    {149, 0, false},  {151, 1, false},  {155, 0, false},
    {156, 0, false},  {165, 0, false},    {166, 0, false},    {167, 0, false},  {168, 0, false},  {182, 0, false},
    {187, 0, false},
};
const Default kAconDefaults[] = {{48, 0.10, true}, {49, 0.10, true}};
const Default kSconDefaults[] = {{48, 0.10, true}, {49, 0.10, true}, {65, 2048, false}, {66, 125, false}};
}  // namespace

std::string category(int no, const std::optional<std::string>& fam) {
    if (fam && *fam == "SCON") {
        const auto it = scon_category_table().find(no);
        if (it != scon_category_table().end()) return it->second;
    }
    const auto it = category_table().find(no);
    return it == category_table().end() ? std::string() : std::string(it->second);
}

std::vector<std::pair<int, std::string>> categories(const std::optional<std::string>& fam) {
    std::vector<std::pair<int, std::string>> out;
    if (!fam) {
        for (const auto& kv : category_table()) out.emplace_back(kv.first, kv.second);
    } else if (*fam == "SCON") {
        for (const auto& kv : scon_category_table()) out.emplace_back(kv.first, kv.second);
    }
    return out;
}

std::string category_text(const std::string& category) {
    if (category == "A") return "check before use";
    if (category == "B") return "set per application";
    if (category == "C") return "use as shipped";
    if (category == "D") return "actuator-specific, set at shipment (do not change)";
    if (category == "E") return "manufacturer use only (never change)";
    return std::string();
}

std::optional<Number> factory_default(int no, const std::optional<std::string>& fam) {
    if (fam && *fam == "ACON/DCON") {
        for (const Default& d : kAconDefaults)
            if (d.no == no) return Number{d.value, d.decimal};
    } else if (fam && *fam == "SCON") {
        for (const Default& d : kSconDefaults)
            if (d.no == no) return Number{d.value, d.decimal};
    }
    for (const Default& d : kDefaults)
        if (d.no == no) return Number{d.value, d.decimal};
    return std::nullopt;
}

std::vector<std::pair<int, Number>> factory_defaults(const std::optional<std::string>& fam) {
    std::vector<std::pair<int, Number>> out;
    if (!fam) {
        for (const Default& d : kDefaults) out.emplace_back(d.no, Number{d.value, d.decimal});
    } else if (*fam == "ACON/DCON") {
        for (const Default& d : kAconDefaults) out.emplace_back(d.no, Number{d.value, d.decimal});
    } else if (*fam == "SCON") {
        for (const Default& d : kSconDefaults) out.emplace_back(d.no, Number{d.value, d.decimal});
    }
    return out;
}

std::optional<Band> min_band(const std::optional<std::string>& fam, std::optional<double> lead_mm,
                             std::optional<std::uint32_t> encoder_pulses, std::optional<double> mini_mm) {
    const std::optional<MinBand> band = min_positioning_band(fam, lead_mm, encoder_pulses, mini_mm);
    if (!band) return std::nullopt;
    return Band{band->mm, band->formula, band->doc, band->certain};
}

}  // namespace con_detail

}  // namespace iai_actuator_driver
