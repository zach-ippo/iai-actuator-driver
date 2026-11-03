// What a serial line carries, and the guard every request passes. Port of the Python edition's gateway.Line /
// gateway.LineRules (iai_actuator_driver/gateway.py); the gateway objects and detect_line() are in gateway.hpp.
//
// Every ModbusRtu carries a Line (bus.line(), default "direct"). On a plain line (an EC or a CON controller on its own
// SIO port, or an SIO daisy chain) everything the driver knows is allowed. Behind a gateway the line carries less, and
// the bus refuses the rest BEFORE anything is sent (LineRuleError):
//
//   line kind          broadcast  0x45/0x48  FC05  other rules
//   "direct"           yes        yes        yes   one unit per port VERIFIED (EC, PCON-CB); daisy chain UNVERIFIED
//   "rcon"             no         no         no    FC 01/03/04/06/10/17 only; gateway in MANU before control and
//                                                  motion; position unit (No.207) known before mm commands;
//                                                  command strategy "register" (DRG1 over FC06)
//   "robonet"          no         no         yes   SIO through mode (SW2 ON): FC 01-07/0F/10/11/17; FC10 <= 75
//                                                  registers (160-byte buffer); host 38.4-230.4 kbps; strategy
//                                                  "coil"; a timed-out 0x0000 directory read = no directory
//   "robonet-gateway"  no         no         no    gateway mode (SW2 OFF, slave 3Fh): no axis access at all
//
// Sources: RCON DOC ME0413 p.3-1 (function codes), p.4-1 (addresses; "RCON-EC unit is not applicable"), p.6-1 #2 (no
// broadcast), #5 (Modbus control in MANU mode); ME0384 p.8-56 (No.207 position unit). ROBONET DOC ME0208 p.26 (through
// mode: FC list, 38.4-230.4 kbps, 160-byte buffer; gateway mode: 3Fh, FC 03/06/10), p.30 / p.118 (SW2), p.340 (fixed
// 230.4 kbps internal link). Every gateway rule is DOC only: no RCON or ROBONET has been tested with this driver
// (UNVERIFIED on hardware, in both editions). The direct line's rules are the Python edition's VERIFIED behaviour; this
// C++ code is UNVERIFIED on hardware.
//
// A Line is shared (std::shared_ptr) by the bus, the Port and the axes; it is thread-safe.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/errors.hpp"

namespace iai_actuator_driver {

class Gateway;      // gateway.hpp
struct LineSnapshot;  // gateway.hpp

// ------------------------------------------------------------------------------------------ names

/// The kinds of line.
enum class LineKind {
    Direct,          ///< "direct": an EC / CON controller on its own SIO port, or an SIO daisy chain
    Rcon,            ///< "rcon": behind an RCON gateway (D0h)
    Robonet,         ///< "robonet": a ROBONET in SIO through mode (SW2 ON), which cannot be probed
    RobonetGateway,  ///< "robonet-gateway": a ROBONET in gateway mode (SW2 OFF, 3Fh): no axis access
};

/// "direct", "rcon", "robonet", "robonet-gateway".
inline const char* to_string(LineKind kind) noexcept {
    switch (kind) {
        case LineKind::Rcon: return "rcon";
        case LineKind::Robonet: return "robonet";
        case LineKind::RobonetGateway: return "robonet-gateway";
        default: return "direct";
    }
}
/// The kind of a name, nullopt when unknown.
std::optional<LineKind> line_kind_from_string(const std::string& name);
/// Every kind, in the order above.
std::vector<LineKind> line_kinds();
/// "auto" -> nullopt, a kind name -> that kind; std::invalid_argument for anything else ("line must be 'auto' or one
/// of direct, rcon, robonet, robonet-gateway, got 'canbus'").
std::optional<LineKind> parse_line_kind(const std::string& text);

/// Operations an axis can ask about (Line::allows / Line::require).
enum class LineOperation {
    Broadcast,      ///< "broadcast": any frame to address 0
    BaudChange,     ///< "baud_change": the broadcast baud change (0x45 cmd 4 to address 0)
    IaiCommand,     ///< "iai_command": function 0x45
    IaiCommand2,    ///< "iai_command2": function 0x48
    SoftwareReset,  ///< "software_reset": 0x45 cmd 5 op1 1
    ActionMode,     ///< "action_mode": 0x45 cmd 0x0C
    HomeIai,        ///< "home_iai": 0x45 cmd 0x0A
    Coil,           ///< "coil": FC05 (servo / home / STOP / pause coils, PMSL)
    ReportSlaveId,  ///< "report_slave_id": FC11
    AxisControl,    ///< "axis_control": any command to an axis at all
};

/// "broadcast", "baud_change", "iai_command", ...
const char* to_string(LineOperation operation) noexcept;
/// The operation of a name, nullopt when unknown.
std::optional<LineOperation> line_operation_from_string(const std::string& name);
/// Every operation, in the order above.
std::vector<LineOperation> line_operations();

// ------------------------------------------------------------------------------------------ addresses and timing

/// RCON-GW unit identifier (D0h; also D1h or FFh). DOC ME0413 p.4-1.
inline constexpr int kRconGatewaySlave = 0xD0;
inline constexpr int kRconGatewayAddresses[] = {0xD0, 0xD1, 0xFF};
/// ROBONET RGW-SIO in gateway mode (SW2 OFF): fixed slave 3Fh. DOC ME0208 p.26, p.119.
inline constexpr int kRoboNetGatewaySlave = 0x3F;
/// Wait per detection probe (D0h, 3Fh). On a line without a gateway both cost ~0.1 s together, once per port.
inline constexpr double kGatewayProbeTimeoutS = 0.05;
/// Minimum response timeout per frame on a gateway line: a teaching-tool frame can be delayed by >= 40 ms per
/// exchange, more with EC units (DOC ME0394 p.5-17; ME0384 p.6-118).
inline constexpr double kGatewayFrameTimeoutS = 0.3;
/// Minimum wait for the first contact with an axis behind an RCON gateway (probe / scan of a linked axis).
inline constexpr double kGatewayFirstContactS = 0.65;
/// Probe wait in ROBONET through mode: the bytes are relayed over the 230.4 kbps internal link (UNVERIFIED).
inline constexpr double kRoboNetProbeTimeoutS = 0.2;
/// How long a read of the RCON gateway status (MANU flag) is trusted before it is read again.
inline constexpr double kGwsrCacheS = 2.0;
/// RCON parameter No.207: 0 = 0.01 mm, 1 = 0.001 mm (target, band, zones, monitored position). DOC ME0384 p.8-56.
inline constexpr double kPositionUnitsMm[] = {0.01, 0.001};

// ------------------------------------------------------------------------------------------ explanations

/// Why an ELECYLINDER behind an EC connection unit is not reachable.
inline constexpr const char* kEcBehindGateway =
    "an ELECYLINDER behind an EC connection unit (RCON-EC / REC) is not reachable over IAI's public Modbus "
    "(DOC ME0413 p.4-1: 'RCON-EC unit is not applicable'), and its own SIO port is disabled there (DOC ME0394 "
    "p.4-56; ME0384 p.5-67): not supported by this driver";
/// What to do about a ROBONET in gateway mode.
inline constexpr const char* kRoboNetSw2 =
    "the ROBONET gateway is in gateway mode (SW2 OFF, slave 3Fh): set SW2 ON (SIO through mode) for direct access "
    "to the axes at axis number + 1, then open the port with line='robonet' (DOC ME0208 p.30, p.118)";
/// Why an RCON axis needs its position unit stated.
inline constexpr const char* kPositionUnitText =
    "RCON driver units may count positions in 0.01 mm or 0.001 mm (parameter No.207, DOC ME0384 p.8-56), and No.207 "
    "is not readable over Modbus (the RCON parameter area is 'system, no access', DOC ME0413 p.5-10). State it: "
    "LineOptions::position_unit_mm = 0.01 or 0.001, or Port::set_position_unit(...)";

// ------------------------------------------------------------------------------------------ errors

/// The RCON gateway is not in MANU mode (or did not answer), so Modbus control is not possible (ME0413 p.6-1 #5).
/// Nothing was sent to the axis.
class GatewayModeError : public EcNotReadyError {
public:
    using EcNotReadyError::EcNotReadyError;
};

/// An mm-based command on an RCON driver unit whose position unit (No.207) is not known. Nothing was sent.
class PositionUnitError : public EcNotReadyError {
public:
    using EcNotReadyError::EcNotReadyError;
};

/// The line gives no axis access the driver supports (ROBONET gateway mode, an EC behind an RCON-EC / REC unit).
class UnsupportedLineError : public EcNotReadyError {
public:
    using EcNotReadyError::EcNotReadyError;
};

// ------------------------------------------------------------------------------------------ line rules

/// What one kind of line carries. function_codes is the documented list (nullopt: no restriction known). The bus
/// refuses only WRITES (and broadcasts) outside the list; a read of an undocumented function (e.g. FC11 on RCON) still
/// goes out: a read is harmless and the unit may answer it.
struct LineRules {
    LineKind kind = LineKind::Direct;
    std::string title;
    bool broadcast = true;
    bool iai_commands = true;
    std::optional<std::vector<int>> function_codes;  ///< ascending
    std::optional<std::string> command_strategy;     ///< "register" (RCON), "coil" (ROBONET), nullopt (the axis's own)
    bool requires_manu = false;
    bool requires_position_unit = false;
    bool axis_access = true;
    int max_write_registers = 123;
    double probe_timeout_s = kGatewayProbeTimeoutS;
    double min_frame_timeout_s = 0.2;
    std::optional<std::pair<int, int>> baud_range;   ///< documented host baud range (inclusive)
    std::vector<int> gateway_addresses;
    bool directory_timeout_means_absent = false;     ///< a timeout on the 0x0000 read = "no directory"
    std::string evidence;

    /// True if `function` is documented on this line (or the line has no documented list).
    bool allows_function(int function) const;
    /// FC05 is carried (false on RCON and in ROBONET gateway mode).
    bool coils() const { return allows_function(0x05); }
    /// FC11 is documented (false on RCON: identify the unit by read_serial_code instead).
    bool report_slave_id() const { return allows_function(0x11); }
    /// `rate` is inside the documented host baud range (always true when none is documented).
    bool baud_allowed(int rate) const {
        return !baud_range || (baud_range->first <= rate && rate <= baud_range->second);
    }
};

/// The rules of one kind of line.
const LineRules& line_rules(LineKind kind);
/// Every kind's rules, in line_kinds() order (for documentation and front ends).
std::vector<LineRules> line_rules_table();

// ------------------------------------------------------------------------------------------ the line

/// Where a position unit came from.
inline constexpr const char* kUnitStated = "stated";   ///< by the caller
inline constexpr const char* kUnitRead = "read";       ///< the axis read No.207 itself
inline constexpr const char* kUnitDefault = "default"; ///< the CON / EC unit 0.01 (direct and ROBONET lines)

/// `unit_mm` if it is one of kPositionUnitsMm (0.01, 0.001), else std::invalid_argument. Nothing is recorded.
double check_position_unit(double unit_mm);

/// (0.5) What a caller states the axes on a line may send, over the kind's rules (the Python edition's duck-typed
/// bus.line_rules). Each field that is set replaces the kind's answer in Line::allows / Line::require, which the axis
/// asks; the bus's request guard (Line::check_request) keeps the kind's rules. `name` is the line's name for the axis
/// (Axis::line_name, Status::line): "direct" = none.
struct StatedLineRules {
    std::optional<std::string> name;
    std::optional<bool> coils;            ///< FC05
    std::optional<bool> iai_commands;     ///< 0x45 / 0x48 (software reset, action mode, IAI home)
    std::optional<bool> broadcast;        ///< frames to address 0 (the broadcast baud change)
    std::optional<bool> report_slave_id;  ///< FC11
};

/// What one serial line carries, and the gateway behind it (if any). Attached to the bus (ModbusRtu::set_line).
class Line {
public:
    /// A line of `kind`. `gateway`: the gateway object found by detect_line (null when none answered). `detected`:
    /// a probe found the gateway (false when the kind was stated or is the default). `position_unit_mm`: the position
    /// unit of every axis on the line (RCON: 0.01 or 0.001), if stated (std::invalid_argument otherwise, and for 0.001
    /// on a direct line: set_position_unit).
    explicit Line(LineKind kind = LineKind::Direct, std::shared_ptr<Gateway> gateway = nullptr, bool detected = false,
                  std::optional<double> position_unit_mm = std::nullopt);
    ~Line();
    Line(const Line&) = delete;
    Line& operator=(const Line&) = delete;

    // ---- basics
    LineKind kind() const;
    const LineRules& rules() const;
    std::shared_ptr<Gateway> gateway() const;
    bool detected() const;
    /// True on every line except "direct".
    bool is_gateway() const;
    /// The command strategy the line requires: "register" (RCON), "coil" (ROBONET) or nullopt.
    std::optional<std::string> command_strategy() const;
    int max_write_registers() const;
    /// Line('rcon', detected, gateway=RconGateway(0xD0)).
    std::string to_string() const;

    // ---- what is carried
    /// Whether `operation` is carried / documented on this line.
    bool allows(LineOperation operation) const;
    /// The same by name; std::invalid_argument for an unknown name.
    bool allows(const std::string& operation) const;
    /// LineRuleError (nothing sent) with a line-specific message if `operation` is not carried. `what` labels the
    /// refused action (default: the operation's name with spaces).
    void require(LineOperation operation, const std::string& what = std::string()) const;

    /// The bus's guard, run before a frame is sent: LineRuleError if the line does not carry it. Refused on a gateway
    /// line: any broadcast; IAI functions 0x45 / 0x48; a write function outside the documented list (FC05 on RCON);
    /// any write in ROBONET gateway mode; any write to the gateway's own address; an FC10 longer than the line's
    /// buffer allows. Reads always pass.
    void check_request(int slave, const std::vector<std::uint8_t>& pdu) const;

    // ---- control and motion preconditions
    /// Run before taking control of an axis and before every motion command. ROBONET gateway mode:
    /// UnsupportedLineError (no axis access). RCON: the gateway must report MANU (GatewayModeError; the status read
    /// is cached for kGwsrCacheS, an AUTO is re-read before refusing). Other lines: nothing to check.
    void require_control(std::optional<int> slave = std::nullopt) const;
    /// The position unit (mm per count) of `slave`: stated / read for that slave, else for the whole line. Direct and
    /// ROBONET lines: 0.01 (reg::kPositionScale) unless stated. RCON: nullopt until stated (No.207 is not readable).
    std::optional<double> position_unit_mm(std::optional<int> slave = std::nullopt) const;
    /// Where position_unit_mm comes from: "stated", "read", "default" or nullopt (unknown).
    std::optional<std::string> position_unit_source(std::optional<int> slave = std::nullopt) const;
    /// Record the position unit (0.01 or 0.001 mm) of `slave` (nullopt = every axis on the line). `source` is
    /// "stated" or "read". std::invalid_argument for another unit, source or a slave outside 1..247, and on a direct
    /// line for any unit but the unit's own 0.01 (a unit on a direct line counts in 0.01 mm: ON-WIRE on a PCON-CB,
    /// where a stated 0.001 read P0 94.99 as 9.499 and would have sent targets x 10).
    void set_position_unit(double unit_mm, std::optional<int> slave = std::nullopt,
                           const std::string& source = kUnitStated);
    /// The position unit of `slave`; PositionUnitError on RCON when it is not known (nothing sent).
    double require_position_unit(int slave) const;
    /// require_control plus require_position_unit; returns the position unit (mm per count).
    double require_motion(int slave) const;
    /// Wire position counts per driver count (the driver computes in 0.01 units): 10 on a 0.001 mm RCON unit, else 1
    /// (also while the unit is not known: motion is refused then by require_motion).
    int wire_factor(int slave) const;

    // ---- explanations
    /// Record axes the gateway reports as linked that do not answer Modbus (e.g. ECs behind an RCON-EC unit).
    void note_unreachable(const std::vector<int>& slaves);
    std::vector<int> unreachable_slaves() const;
    /// Why linked axes (default: unreachable_slaves) do not answer; nullopt on a non-RCON line or with none.
    std::optional<std::string> explain_unreachable(const std::optional<std::vector<int>>& slaves = std::nullopt) const;
    /// A one-line explanation of what this line does not support (nullopt on a direct line).
    std::optional<std::string> explain() const;
    /// Everything above as plain data (the gateway's last status included), for to_json (gateway.hpp).
    LineSnapshot snapshot() const;

    // ---- (0.5) rules stated by the caller
    /// State what the axes on this line may send over the kind's rules (StatedLineRules); nullopt clears.
    void state_rules(std::optional<StatedLineRules> rules);
    std::optional<StatedLineRules> stated_rules() const;
    /// The line's name for the axis: the stated name, else the kind's; nullopt on a direct line.
    std::optional<std::string> axis_name() const;

private:
    std::string reason(LineOperation operation) const;
    /// The kind's own answer (the request guard's), without the stated rules.
    bool kind_allows(LineOperation operation) const;
    LineRuleError refusal(LineOperation operation, const std::string& what) const;

    const LineRules* rules_;
    std::shared_ptr<Gateway> gateway_;
    bool detected_;
    mutable std::mutex mutex_;
    std::map<int, std::pair<double, std::string>> units_;  ///< slave (0 = every axis) -> (unit, source)
    std::set<int> unreachable_;
    std::optional<StatedLineRules> stated_;  ///< guarded by mutex_
};

}  // namespace iai_actuator_driver
