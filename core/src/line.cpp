// Line rules and the request guard (port of the Python edition's gateway.LineRules / gateway.Line). Every gateway rule
// is DOC only (ME0413, ME0208) and UNVERIFIED on hardware; the direct line's behaviour is VERIFIED in the Python
// edition. This C++ code is UNVERIFIED on hardware.
#include "iai_actuator_driver/line.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr int kFcWriteMany = 0x10;
constexpr int kFcIai = 0x45;
constexpr int kFcIai2 = 0x48;

/// Function codes that change something in the slave (FC17 writes before it reads).
bool is_write_function(int function) {
    switch (function) {
        case 0x05: case 0x06: case 0x0F: case 0x10: case 0x17: case 0x45: case 0x48: return true;
        default: return false;
    }
}

bool is_iai_operation(LineOperation op) {
    return op == LineOperation::IaiCommand || op == LineOperation::IaiCommand2 || op == LineOperation::SoftwareReset ||
           op == LineOperation::ActionMode || op == LineOperation::HomeIai;
}

const char* const kKindNames[] = {"direct", "rcon", "robonet", "robonet-gateway"};
const char* const kOperationNames[] = {"broadcast", "baud_change", "iai_command", "iai_command2", "software_reset",
                                       "action_mode", "home_iai", "coil", "report_slave_id", "axis_control"};

std::string kinds_text() { return "direct, rcon, robonet, robonet-gateway"; }

std::vector<LineRules> build_rules() {
    std::vector<LineRules> table(4);

    LineRules& direct = table[0];
    direct.kind = LineKind::Direct;
    direct.title = "direct SIO line (EC / CON controller, one unit or a daisy chain)";
    direct.broadcast = true;
    direct.iai_commands = true;
    direct.max_write_registers = 123;  // 256-byte receive buffer (ME0162-12A p.5-4)
    direct.probe_timeout_s = 0.05;
    direct.min_frame_timeout_s = 0.2;
    direct.evidence = "VERIFIED (one EC or PCON-CB per port); several units on one line UNVERIFIED";

    LineRules& rcon = table[1];
    rcon.kind = LineKind::Rcon;
    rcon.title = "RCON gateway line";
    rcon.broadcast = false;                                       // ME0413 p.6-1 #2
    rcon.iai_commands = false;                                    // ME0413 p.3-1
    rcon.function_codes = std::vector<int>{0x01, 0x03, 0x04, 0x06, 0x10, 0x17};  // ME0413 p.3-1
    rcon.command_strategy = std::string("register");             // DRG1 over FC06 (ME0413 p.5-11, 5-13)
    rcon.requires_manu = true;                                    // ME0413 p.6-1 #5
    rcon.requires_position_unit = true;                           // ME0384 p.8-56
    rcon.axis_access = true;
    rcon.max_write_registers = 123;
    rcon.probe_timeout_s = kGatewayFirstContactS;
    rcon.min_frame_timeout_s = kGatewayFrameTimeoutS;
    // 115200 fixed, but a PC tool may leave it at its own rate (ME0413 p.6-1 #3): auto-detect (no range)
    rcon.gateway_addresses = {std::begin(kRconGatewayAddresses), std::end(kRconGatewayAddresses)};
    rcon.directory_timeout_means_absent = true;  // directory / parameters / table: "system, no access" (ME0413 p.5-10)
    rcon.evidence = "DOC ME0413 p.3-1, p.4-1, p.6-1; UNVERIFIED on hardware";

    LineRules& robonet = table[2];
    robonet.kind = LineKind::Robonet;
    robonet.title = "ROBONET line in SIO through mode";
    robonet.broadcast = false;      // not documented for through mode; fixed 230.4 kbps link (ME0208 p.26, p.340)
    robonet.iai_commands = false;   // ME0208 p.26
    robonet.function_codes =
        std::vector<int>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0F, 0x10, 0x11, 0x17};  // ME0208 p.26
    robonet.command_strategy = std::string("coil");
    robonet.max_write_registers = 75;  // 160-byte buffer (ME0208 p.26): 9 + 2 x 75 = 159 bytes
    robonet.probe_timeout_s = kRoboNetProbeTimeoutS;
    robonet.min_frame_timeout_s = kGatewayFrameTimeoutS;
    robonet.baud_range = std::make_pair(38400, 230400);  // ME0208 p.26
    robonet.directory_timeout_means_absent = true;  // RCM-era axis firmware: expect no directory (ME0391 p.1)
    robonet.evidence = "DOC ME0208 p.26, p.340; UNVERIFIED on hardware";

    LineRules& gateway = table[3];
    gateway.kind = LineKind::RobonetGateway;
    gateway.title = "ROBONET line in gateway mode";
    gateway.broadcast = false;
    gateway.iai_commands = false;
    gateway.function_codes = std::vector<int>{0x03, 0x06, 0x10};  // ME0208 p.26
    gateway.axis_access = false;  // a 1:1 link to the gateway's own image; the axes sit behind it
    gateway.max_write_registers = 75;
    gateway.probe_timeout_s = kRoboNetProbeTimeoutS;
    gateway.min_frame_timeout_s = kGatewayFrameTimeoutS;
    gateway.baud_range = std::make_pair(9600, 230400);  // ME0208 p.26
    gateway.gateway_addresses = {kRoboNetGatewaySlave};
    gateway.directory_timeout_means_absent = true;
    gateway.evidence = "DOC ME0208 p.26, p.30, p.118; UNVERIFIED on hardware";
    return table;
}

const std::vector<LineRules>& rules_table() {
    static const std::vector<LineRules> table = build_rules();
    return table;
}

// Why an operation is not carried, per line kind (line-specific refusal messages).
std::string iai_reason(LineKind kind) {
    switch (kind) {
        case LineKind::Rcon:
            return "it needs IAI command 0x45/0x48, and the RCON function-code list is 01/03/04/06/10/17 only "
                   "(DOC ME0413 p.3-1)";
        case LineKind::Robonet:
            return "it needs IAI command 0x45/0x48, which ROBONET through mode does not carry (DOC ME0208 p.26)";
        case LineKind::RobonetGateway: return kRoboNetSw2;
        default: return "IAI commands are not carried here";
    }
}

std::string broadcast_reason(LineKind kind) {
    switch (kind) {
        case LineKind::Rcon: return "RCON is not applicable for broadcast commands (DOC ME0413 p.6-1 #2)";
        case LineKind::Robonet:
            return "broadcast is not documented for ROBONET through mode, and the axes run a fixed 230.4 kbps internal "
                   "link (DOC ME0208 p.26, p.340): a broadcast baud change could leave every axis unreachable until a "
                   "power cycle";
        case LineKind::RobonetGateway: return kRoboNetSw2;
        default: return "broadcast is not allowed here";
    }
}

std::string function_reason(LineKind kind, const char* fallback) {
    switch (kind) {
        case LineKind::Rcon: return "the RCON function-code list is 01/03/04/06/10/17 only (DOC ME0413 p.3-1)";
        case LineKind::Robonet: return "ROBONET through mode carries FC 01-07, 0F, 10, 11 and 17 only (DOC ME0208 p.26)";
        case LineKind::RobonetGateway: return kRoboNetSw2;
        default: return fallback;
    }
}

std::string join_ints(const std::vector<int>& values) {
    std::vector<std::string> parts;
    for (int v : values) parts.push_back(std::to_string(v));
    return util::join(parts, ", ");
}

}  // namespace

// ------------------------------------------------------------------------------------------ names

std::optional<LineKind> line_kind_from_string(const std::string& name) {
    for (int i = 0; i < 4; ++i)
        if (name == kKindNames[i]) return static_cast<LineKind>(i);
    return std::nullopt;
}

std::vector<LineKind> line_kinds() {
    return {LineKind::Direct, LineKind::Rcon, LineKind::Robonet, LineKind::RobonetGateway};
}

std::optional<LineKind> parse_line_kind(const std::string& text) {
    if (text == "auto") return std::nullopt;
    const std::optional<LineKind> kind = line_kind_from_string(text);
    if (!kind) throw std::invalid_argument("line must be 'auto' or one of " + kinds_text() + ", got '" + text + "'");
    return kind;
}

const char* to_string(LineOperation operation) noexcept { return kOperationNames[static_cast<int>(operation)]; }

std::optional<LineOperation> line_operation_from_string(const std::string& name) {
    for (int i = 0; i < 10; ++i)
        if (name == kOperationNames[i]) return static_cast<LineOperation>(i);
    return std::nullopt;
}

std::vector<LineOperation> line_operations() {
    std::vector<LineOperation> out;
    for (int i = 0; i < 10; ++i) out.push_back(static_cast<LineOperation>(i));
    return out;
}

// ------------------------------------------------------------------------------------------ rules

bool LineRules::allows_function(int function) const {
    return !function_codes || std::find(function_codes->begin(), function_codes->end(), function) != function_codes->end();
}

const LineRules& line_rules(LineKind kind) { return rules_table().at(static_cast<std::size_t>(kind)); }

std::vector<LineRules> line_rules_table() { return rules_table(); }

// ------------------------------------------------------------------------------------------ Line

Line::Line(LineKind kind, std::shared_ptr<Gateway> gateway, bool detected, std::optional<double> position_unit_mm)
    : rules_(&line_rules(kind)), gateway_(std::move(gateway)), detected_(detected) {
    if (position_unit_mm) set_position_unit(*position_unit_mm);
}

Line::~Line() = default;

LineKind Line::kind() const { return rules_->kind; }
const LineRules& Line::rules() const { return *rules_; }
std::shared_ptr<Gateway> Line::gateway() const { return gateway_; }
bool Line::detected() const { return detected_; }
bool Line::is_gateway() const { return kind() != LineKind::Direct; }
std::optional<std::string> Line::command_strategy() const { return rules_->command_strategy; }
int Line::max_write_registers() const { return rules_->max_write_registers; }

std::string Line::to_string() const {
    return std::string("Line('") + iai_actuator_driver::to_string(kind()) + "', " + (detected_ ? "detected" : "stated") +
           ", gateway=" + (gateway_ ? gateway_->to_string() : std::string("None")) + ")";
}

bool Line::allows(LineOperation operation) const {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stated_) {  // (0.5) what the caller stated replaces the kind's answer
            std::optional<bool> stated;
            switch (operation) {
                case LineOperation::Broadcast:
                case LineOperation::BaudChange: stated = stated_->broadcast; break;
                case LineOperation::Coil: stated = stated_->coils; break;
                case LineOperation::ReportSlaveId: stated = stated_->report_slave_id; break;
                case LineOperation::AxisControl: break;
                default: stated = stated_->iai_commands; break;
            }
            if (stated) return *stated;
        }
    }
    return kind_allows(operation);
}

bool Line::kind_allows(LineOperation operation) const {
    const LineRules& r = *rules_;
    switch (operation) {
        case LineOperation::Broadcast:
        case LineOperation::BaudChange: return r.broadcast;
        case LineOperation::IaiCommand:
        case LineOperation::IaiCommand2:
        case LineOperation::SoftwareReset:
        case LineOperation::ActionMode:
        case LineOperation::HomeIai: return r.iai_commands;
        case LineOperation::Coil: return r.coils();
        case LineOperation::ReportSlaveId: return r.report_slave_id();
        case LineOperation::AxisControl: return r.axis_access;
    }
    return false;
}

bool Line::allows(const std::string& operation) const {
    const std::optional<LineOperation> op = line_operation_from_string(operation);
    if (!op) {
        std::vector<std::string> names(std::begin(kOperationNames), std::end(kOperationNames));
        throw std::invalid_argument("unknown line operation '" + operation + "'; choose from " +
                                    util::join(names, ", "));
    }
    return allows(*op);
}

std::string Line::reason(LineOperation operation) const {
    const LineKind k = kind();
    if (k == LineKind::RobonetGateway) return kRoboNetSw2;
    if (operation == LineOperation::Broadcast || operation == LineOperation::BaudChange) {
        std::string text = broadcast_reason(k);
        if (operation == LineOperation::BaudChange && k == LineKind::Rcon)
            text += "; the RCON line runs at 115200 (DOC ME0413 p.2-1)";
        return text;
    }
    if (is_iai_operation(operation)) return iai_reason(k);
    return function_reason(k, "not carried here");
}

void Line::require(LineOperation operation, const std::string& what) const {
    if (!allows(operation)) throw refusal(operation, what);
}

LineRuleError Line::refusal(LineOperation operation, const std::string& what) const {
    std::string label = what;
    if (label.empty()) {
        label = iai_actuator_driver::to_string(operation);
        std::replace(label.begin(), label.end(), '_', ' ');
    }
    return LineRuleError(label + ": not sent on this " + rules_->title + ": " + reason(operation) + "; nothing sent");
}

void Line::check_request(int slave, const std::vector<std::uint8_t>& pdu) const {
    if (pdu.empty()) return;
    const int function = pdu[0];
    // the kind's rules: what a caller states for the axes (state_rules) does not change what the bus sends
    if (slave == 0 && !kind_allows(LineOperation::Broadcast))
        throw refusal(LineOperation::Broadcast, util::format("broadcast of function 0x%02X", function));
    if (function == kFcIai || function == kFcIai2) {
        const LineOperation op = function == kFcIai ? LineOperation::IaiCommand : LineOperation::IaiCommand2;
        if (!kind_allows(op)) throw refusal(op, util::format("IAI command 0x%02X", function));
    }
    if (!is_write_function(function)) return;
    const LineRules& r = *rules_;
    if (!r.axis_access)
        throw LineRuleError(
            util::format("write function 0x%02X to slave %d: %s; nothing sent", function, slave, kRoboNetSw2));
    if (!r.allows_function(function))
        throw LineRuleError(util::format("function 0x%02X to slave %d: not sent on this %s: %s; nothing sent", function,
                                         slave, r.title.c_str(),
                                         function_reason(kind(), "not documented here").c_str()));
    if (std::find(r.gateway_addresses.begin(), r.gateway_addresses.end(), slave) != r.gateway_addresses.end())
        throw LineRuleError(util::format(
            "write function 0x%02X to the gateway's own address 0x%02X: the gateway is a read-only object in this "
            "driver; nothing sent",
            function, slave));
    if (function == kFcWriteMany && pdu.size() >= 5) {
        const int count = (pdu[3] << 8) | pdu[4];
        if (count > r.max_write_registers)
            throw LineRuleError(util::format(
                "FC10 of %d registers: this %s takes at most %d per frame (160-byte buffer, DOC ME0208 p.26); nothing "
                "sent",
                count, r.title.c_str(), r.max_write_registers));
    }
}

// ------------------------------------------------------------------------------------------ control and motion

void Line::require_control(std::optional<int> /*slave*/) const {
    if (!rules_->axis_access) throw UnsupportedLineError(kRoboNetSw2);
    if (!rules_->requires_manu) return;
    auto* gw = dynamic_cast<RconGateway*>(gateway_.get());
    if (!gw)
        throw GatewayModeError(
            "RCON line without an answering gateway (D0h): MANU mode cannot be confirmed, so Modbus control is "
            "refused (DOC ME0413 p.6-1 #5); nothing was sent");
    gw->require_manu();
}

std::optional<double> Line::position_unit_mm(std::optional<int> slave) const {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (slave) {
            const auto it = units_.find(*slave);
            if (it != units_.end()) return it->second.first;
        }
        const auto all = units_.find(0);
        if (all != units_.end()) return all->second.first;
    }
    if (rules_->requires_position_unit) return std::nullopt;
    return reg::kPositionScale;
}

std::optional<std::string> Line::position_unit_source(std::optional<int> slave) const {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (slave) {
            const auto it = units_.find(*slave);
            if (it != units_.end()) return it->second.second;
        }
        const auto all = units_.find(0);
        if (all != units_.end()) return all->second.second;
    }
    if (rules_->requires_position_unit) return std::nullopt;
    return std::string(kUnitDefault);
}

double check_position_unit(double unit_mm) {
    for (double u : kPositionUnitsMm)
        if (std::fabs(unit_mm - u) < 1e-12) return u;
    throw std::invalid_argument("position unit must be one of (0.01, 0.001) mm, got " + util::g(unit_mm));
}

void Line::set_position_unit(double unit_mm, std::optional<int> slave, const std::string& source) {
    const double unit = check_position_unit(unit_mm);
    if (source != kUnitStated && source != kUnitRead) throw std::invalid_argument("source must be 'stated' or 'read'");
    // a direct line: the unit's own 0.01 (ON-WIRE on a PCON-CB: a stated 0.001 read 94.99 as 9.499, targets x 10)
    if (kind() == LineKind::Direct && unit != reg::kPositionScale)
        throw std::invalid_argument(
            "position unit " + util::g(unit) +
            " mm refused on a direct line: a unit on a direct line counts positions in its own unit, 0.01 mm; the "
            "position unit is stated only for RCON driver units (No.207, DOC ME0384 p.8-56). Drop it (or state "
            "the line 'rcon'); nothing sent");
    if (slave && (*slave < 1 || *slave > 247))
        throw std::invalid_argument(util::format("slave address must be 1..247, got %d", *slave));
    std::lock_guard<std::mutex> lock(mutex_);
    units_[slave ? *slave : 0] = std::make_pair(unit, source);
}

double Line::require_position_unit(int slave) const {
    const std::optional<double> unit = position_unit_mm(slave);
    if (!unit) throw PositionUnitError(util::format("slave %d: %s; nothing was sent", slave, kPositionUnitText));
    return *unit;
}

double Line::require_motion(int slave) const {
    require_control(slave);
    return require_position_unit(slave);
}

int Line::wire_factor(int slave) const {
    const std::optional<double> unit = position_unit_mm(slave);
    if (!unit || !(*unit > 0.0)) return 1;
    return std::max(1, static_cast<int>(std::lround(reg::kPositionScale / *unit)));
}

// ------------------------------------------------------------------------------------------ explanations

void Line::note_unreachable(const std::vector<int>& slaves) {
    std::lock_guard<std::mutex> lock(mutex_);
    unreachable_.insert(slaves.begin(), slaves.end());
}

std::vector<int> Line::unreachable_slaves() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<int>(unreachable_.begin(), unreachable_.end());
}

std::optional<std::string> Line::explain_unreachable(const std::optional<std::vector<int>>& slaves) const {
    std::vector<int> items = slaves ? *slaves : unreachable_slaves();
    std::sort(items.begin(), items.end());
    items.erase(std::unique(items.begin(), items.end()), items.end());
    if (items.empty() || kind() != LineKind::Rcon) return std::nullopt;
    return "slave(s) " + join_ints(items) +
           " are linked at the RCON gateway but do not answer Modbus: if they are ELECYLINDER axes, " + kEcBehindGateway;
}

std::optional<std::string> Line::explain() const {
    if (kind() == LineKind::RobonetGateway) return std::string(kRoboNetSw2);
    return explain_unreachable();
}

LineSnapshot Line::snapshot() const {
    LineSnapshot s;
    s.kind = kind();
    s.detected = detected_;
    s.rules = *rules_;
    if (gateway_) s.gateway = gateway_->last_gateway_status();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        s.position_units = units_;
    }
    s.unreachable_slaves = unreachable_slaves();
    s.explanation = explain();
    return s;
}

// ------------------------------------------------------------------------------------------ (0.5) stated rules

void Line::state_rules(std::optional<StatedLineRules> rules) {
    std::lock_guard<std::mutex> lock(mutex_);
    stated_ = std::move(rules);
}

std::optional<StatedLineRules> Line::stated_rules() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stated_;
}

std::optional<std::string> Line::axis_name() const {
    std::string name = iai_actuator_driver::to_string(kind());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stated_ && stated_->name && !stated_->name->empty()) name = *stated_->name;
    }
    if (name == "direct") return std::nullopt;
    return name;
}

}  // namespace iai_actuator_driver
