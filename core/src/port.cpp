// One serial line shared by one or several units: baud detection once, scans, the line-wide baud change (port of the
// Python edition's cell.Port / cell.scan; C# Port.cs, PortManager.cs). Several units per port is UNVERIFIED on
// hardware; this edition is UNVERIFIED on hardware.
#include "iai_actuator_driver/port.hpp"

#include <algorithm>
#include <mutex>
#include <set>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "bus_stop_timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace detail {
int check_baud_change(int rate);  // baud.cpp
}  // namespace detail

namespace {

constexpr const char* kLogSource = "iai_actuator_driver.port";

void check_slave(int slave) {
    if (slave < 1 || slave > 247) throw std::invalid_argument(util::format("slave address must be 1..247, got %d", slave));
}

/// Why `slave` is not at rest, "" when it is: the status bits the model unit's Status::idle() / pushing() use (DSS1
/// STP, DSSE MOVE / GHMS / PUSH, and servo on + homed without PEND / PSFL / STP / a major alarm = still travelling).
/// Read here from the raw words so the bus does not depend on the model unit.
std::string not_at_rest(ModbusRtu& bus, int slave) {
    const std::vector<std::uint16_t> w = bus.read_registers(slave, reg::kDss1, 3);
    const std::uint16_t dss1 = w[0];
    const std::uint16_t dsse = w[2];
    if (dsse & reg::kDsseGhms) return "homing";
    if (dss1 & reg::kDss1Stp) return "paused";
    if (dsse & reg::kDssePush) return "pushing";
    if (dsse & reg::kDsseMove) return "moving";
    const bool travelling = (dss1 & reg::kDss1Sv) && (dss1 & reg::kDss1Hend) && !(dss1 & reg::kDss1Pend) &&
                            !(dss1 & reg::kDss1Psfl) && !(dss1 & reg::kDss1Almh);
    return travelling ? "moving" : std::string();
}

/// Baud detection that also asks the gateway addresses (`extra`: (slave, register) pairs, after the axes at each
/// rate), so a line where only a gateway answers (ROBONET gateway mode) is found too. One read per address and rate,
/// no retries; the current rate first; the start rate restored when nothing answers (ModbusTimeout).
int detect_rate_with_gateways(ModbusRtu& bus, const std::vector<int>& listed,
                              const std::vector<std::pair<int, int>>& extra, const std::vector<int>& candidates) {
    for (int s : listed) check_slave(s);
    const std::vector<int> slaves = detail::skip_ascii_start_slave(listed, kLogSource, "baud rate detection");
    const std::optional<int> start = bus.baud_rate();
    if (!start) throw std::logic_error(bus.name() + " has no baud rate to detect");
    std::vector<int> order{*start};
    for (int b : candidates)
        if (std::find(order.begin(), order.end(), b) == order.end()) order.push_back(b);
    const std::optional<double> plain_wait = slaves.size() > 1 ? std::optional<double>(kScanTimeoutS) : std::nullopt;
    for (int baud : order) {
        if (bus.baud_rate() != baud) bus.set_baud_rate(baud);
        for (int slave : slaves) {
            bool answered = false;
            if (plain_wait) {
                answered = bus.probe(slave, *plain_wait);
            } else {
                try {
                    bus.read_registers(slave, 0x9000, 1, 0);
                    answered = true;
                } catch (const ModbusSlaveError&) {
                    answered = true;  // a well-formed exception reply still proves the rate
                } catch (const ModbusTimeout&) {
                } catch (const ModbusCrcError&) {
                } catch (const ModbusFrameError&) {
                }
            }
            if (answered) {
                log_info(kLogSource, util::format("slave %d answers at %d baud", slave, baud));
                return baud;
            }
        }
        for (const auto& target : extra) {
            if (bus.probe(target.first, kScanTimeoutS, target.second)) {
                log_info(kLogSource, util::format("gateway 0x%02X answers at %d baud", target.first, baud));
                return baud;
            }
        }
    }
    bus.set_baud_rate(*start);
    std::vector<std::string> rates;
    for (int b : order) rates.push_back(std::to_string(b));
    std::vector<std::string> addresses;
    for (int s : slaves) addresses.push_back(std::to_string(s));
    for (const auto& target : extra) addresses.push_back(util::format("0x%02X", target.first));
    throw ModbusTimeout("none of slaves " + util::join(addresses, ", ") + " answered at any of " +
                        util::join(rates, ", ") + " baud");
}

}  // namespace

struct Port::Impl {
    std::string name;
    std::shared_ptr<ModbusRtu> bus;
    std::mutex mutex;
    bool baud_checked = false;
    bool baud_detected = false;
    // the line (line.hpp): stated or probed once (line_mutex)
    std::mutex line_mutex;
    std::optional<LineKind> line_kind;      ///< nullopt: "auto"
    std::optional<double> position_unit_mm;
    bool line_detected = false;
    LineDetector detector;  ///< (py-cell) the bus's owner works out the line (no probe of the port's own)
};

Port::Port(std::shared_ptr<Transport> transport, std::string name, BusOptions options) : impl_(new Impl) {
    if (!transport) throw std::invalid_argument("transport required");
    impl_->name = name.empty() ? transport->name() : std::move(name);
    if (impl_->name.empty()) impl_->name = "PORT";
    impl_->bus = std::make_shared<ModbusRtu>(std::move(transport), std::move(options));
    impl_->baud_checked = !impl_->bus->can_change_baud_rate();  // a transport without a rate has nothing to detect
}

Port::Port(std::shared_ptr<Transport> transport, std::string name, BusOptions options, LineOptions line)
    : Port(std::move(transport), std::move(name), std::move(options)) {
    const std::optional<LineKind> kind = parse_line_kind(line.kind);  // std::invalid_argument for an unknown kind
    impl_->line_kind = kind;
    impl_->position_unit_mm = line.position_unit_mm;
    if (kind && (*kind == LineKind::Direct || *kind == LineKind::Robonet)) {  // nothing to probe
        impl_->bus->set_line(std::make_shared<Line>(*kind, nullptr, false, line.position_unit_mm));
        impl_->line_detected = true;
    } else if (line.position_unit_mm) {
        check_position_unit(*line.position_unit_mm);  // validated now; set at detection (a direct line refuses 0.001)
    }
}

Port::Port(std::shared_ptr<ModbusRtu> bus, std::string name, LineOptions line, bool fixed_rate,
           LineDetector detector)
    : impl_(new Impl) {
    if (!bus) throw std::invalid_argument("bus required");
    impl_->name = name.empty() ? bus->name() : std::move(name);
    if (impl_->name.empty()) impl_->name = "PORT";
    impl_->bus = std::move(bus);
    impl_->baud_checked = fixed_rate || !impl_->bus->can_change_baud_rate();
    impl_->line_kind = parse_line_kind(line.kind);  // std::invalid_argument for an unknown kind
    impl_->position_unit_mm = line.position_unit_mm;
    impl_->detector = std::move(detector);
    if (impl_->detector) {  // the owner of the bus states or detects the line itself: nothing is set here
    } else if (impl_->line_kind &&
               (*impl_->line_kind == LineKind::Direct || *impl_->line_kind == LineKind::Robonet)) {
        impl_->bus->set_line(std::make_shared<Line>(*impl_->line_kind, nullptr, false, line.position_unit_mm));
        impl_->line_detected = true;
    } else if (line.position_unit_mm) {
        check_position_unit(*line.position_unit_mm);  // validated now; set at detection (a direct line refuses 0.001)
    }
}

Port::~Port() = default;

std::shared_ptr<Port> Port::open(const std::string& port_name, const PortOptions& options) {
    auto transport = std::make_shared<SerialTransport>(port_name, options.baud_rate.value_or(kDefaultBaudRate),
                                                       options.read_timeout_s);
    auto port = std::make_shared<Port>(transport, transport->name(), options.bus, options.line);
    if (options.baud_rate) port->impl_->baud_checked = true;  // a fixed rate: used as given, never probed
    return port;
}

const std::string& Port::name() const { return impl_->name; }
std::shared_ptr<ModbusRtu> Port::bus() const { return impl_->bus; }
std::optional<int> Port::baud_rate() const { return impl_->bus->baud_rate(); }

bool Port::baud_detected() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->baud_detected;
}

int Port::detect_baud_rate(const std::vector<int>& slaves, const std::vector<int>& candidates,
                           std::optional<double> probe_timeout_s) {
    int rate = 0;
    try {
        rate = impl_->bus->detect_baud_rate(slaves, candidates.empty() ? baud_detect_order() : candidates, 0x9000,
                                            true, probe_timeout_s);
    } catch (const ModbusTimeout& e) {
        ModbusTimeout error(e.message() + "; " + kNoAnswerHint);
        throw error;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->baud_detected = true;
    impl_->baud_checked = true;
    return rate;
}

std::optional<int> Port::ensure_baud_rate(const std::vector<int>& slaves) {
    bool checked = false;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        checked = impl_->baud_checked;
    }
    if (!checked) {
        // the bus mutex (held by detect_baud_rate) serialises concurrent callers; a second one re-checks first
        std::lock_guard<std::recursive_mutex> sequence(impl_->bus->sequence_mutex());
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            checked = impl_->baud_checked;
        }
        if (!checked) {
            // the line kind is fixed at construction: read without line_mutex (detect_line holds it while calling here)
            const std::optional<LineKind> kind = impl_->line_kind;
            if (kind && *kind == LineKind::Direct) {
                detect_baud_rate(slaves);
            } else {
                std::vector<int> order = baud_detect_order();
                if (kind && (*kind == LineKind::Robonet || *kind == LineKind::RobonetGateway)) {
                    const LineRules& rules = line_rules(*kind);  // ME0208 p.26: only the documented host rates
                    order.erase(std::remove_if(order.begin(), order.end(),
                                               [&](int rate) { return !rules.baud_allowed(rate); }),
                                order.end());
                }
                std::vector<std::pair<int, int>> extra;
                if (!kind || *kind == LineKind::Rcon) extra.emplace_back(kRconGatewaySlave, kRconGwStatus);
                if (!kind || *kind == LineKind::RobonetGateway) extra.emplace_back(kRoboNetGatewaySlave, kRoboNetGwStatus);
                if (extra.empty()) {
                    detect_baud_rate(slaves, order);
                } else {
                    try {
                        detect_rate_with_gateways(*impl_->bus, slaves, extra, order);
                    } catch (const ModbusTimeout& e) {
                        ModbusTimeout error(e.message() + "; " + kNoAnswerHint);
                        throw error;
                    }
                    std::lock_guard<std::mutex> lock(impl_->mutex);
                    impl_->baud_detected = true;
                    impl_->baud_checked = true;
                }
            }
        }
    }
    return baud_rate();
}

std::vector<int> Port::scan(const std::vector<int>& listed, std::optional<double> timeout_s, int address,
                            bool use_link_map) {
    if (listed.empty()) return {};
    // slave 58 (0x3A) would raise alarm 0x05E on every IAI unit of the line (modbus.hpp): never probed
    const std::vector<int> slaves = detail::skip_ascii_start_slave(listed, kLogSource, impl_->name + ": scan");
    try {
        ensure_baud_rate(slaves);
    } catch (const ModbusTimeout& e) {
        log_info(kLogSource, impl_->name + ": no address answered at any rate (" + e.what() + ")");
        return {};
    }
    // gateway lines (detected once, READ-ONLY): no axis in ROBONET gateway mode; longer first-contact waits; on RCON
    // only the axes the gateway reports as linked, and a linked axis that stays silent is recorded and explained
    const std::shared_ptr<Line> line = detect_line();
    if (!line->rules().axis_access) {
        log_warning(kLogSource, impl_->name + ": " + line->explain().value_or(kRoboNetSw2));
        return {};
    }
    double wait = timeout_s.value_or(kScanTimeoutS);
    if (line->is_gateway()) wait = std::max(wait, line->rules().probe_timeout_s);
    std::vector<int> candidates = slaves;
    std::vector<int> linked;
    if (line->kind() == LineKind::Rcon && use_link_map) {
        if (auto gw = std::dynamic_pointer_cast<RconGateway>(line->gateway())) {
            try {
                linked = gw->status().linked_slaves();  // the detection read, or a fresh one
            } catch (const ModbusError& e) {
                log_info(kLogSource, impl_->name + ": RCON gateway status not readable (" + e.what() +
                                         "): probing every address");
            }
            if (!linked.empty()) {
                candidates.clear();
                for (int s : slaves)
                    if (std::find(linked.begin(), linked.end(), s) != linked.end()) candidates.push_back(s);
            }
        }
    }
    std::vector<int> found;
    for (int slave : candidates)
        if (impl_->bus->probe(slave, wait, address)) found.push_back(slave);
    if (!linked.empty()) {
        std::vector<int> silent;
        for (int s : candidates)
            if (std::find(found.begin(), found.end(), s) == found.end()) silent.push_back(s);
        if (!silent.empty()) {
            line->note_unreachable(silent);
            log_warning(kLogSource, impl_->name + ": " + line->explain_unreachable(silent).value_or(""));
        }
    }
    return found;
}

std::vector<ScanHit> Port::scan_units(const std::vector<int>& slaves, std::optional<double> timeout_s, bool identify,
                                     bool use_link_map) {
    std::vector<int> unique;
    for (int s : slaves) {
        check_slave(s);
        if (std::find(unique.begin(), unique.end(), s) == unique.end()) unique.push_back(s);
    }
    if (unique.empty()) return {};
    // slave 58 (0x3A) would raise alarm 0x05E on every IAI unit of the line (modbus.hpp): never probed
    const std::vector<int> addresses = detail::skip_ascii_start_slave(unique, kLogSource, impl_->name + ": scan");
    try {
        ensure_baud_rate(addresses);
    } catch (const ModbusTimeout& e) {
        log_info(kLogSource, impl_->name + ": no address answered at any rate (" + e.what() + ")");
        return {};
    }
    std::vector<ScanHit> hits;
    for (int slave : scan(addresses, timeout_s, 0x9000, use_link_map)) {
        if (identify) {
            hits.push_back(this->identify(slave));
            continue;
        }
        ScanHit hit;
        hit.port = impl_->name;
        hit.slave = slave;
        hit.line = to_string(impl_->bus->line()->kind());
        hits.push_back(std::move(hit));
    }
    return hits;
}

ScanHit Port::identify(int slave) {
    check_slave(slave);
    ScanHit hit;
    hit.port = impl_->name;
    hit.slave = slave;
    const std::shared_ptr<Line> line = impl_->bus->line();
    hit.line = to_string(line->kind());
    if (!line->rules().report_slave_id()) {
        // FC11 is not documented on this line (RCON, ME0413 p.3-1): the 0x9400 serial code is the identity
        hit.serial_code = read_serial_code(*impl_->bus, slave);
        if (!hit.serial_code) hit.error = "serial code 0x9400 not readable";
        return hit;
    }
    try {
        hit.slave_id_raw = impl_->bus->report_slave_id(slave, 0);
        const std::vector<std::uint8_t>& sid = hit.slave_id_raw;
        if (sid.size() < 5) {
            hit.error = "report slave id: short answer " + hex_frame(sid);
        } else {
            hit.system_code = sid[0];
            hit.unit_code = sid[2];
            hit.firmware = util::format("%02X%02X", sid[3], sid[4]);
        }
    } catch (const ModbusError& e) {
        hit.slave_id_raw.clear();
        hit.error = std::string("report slave id: ") + e.what();
    }
    return hit;
}

PortBaudResult Port::set_baud_rate(int rate, const std::vector<int>& slaves, const BaudChangeOptions& options) {
    detail::check_baud_change(rate);  // the refusal first: nothing sent
    std::set<int> unique(slaves.begin(), slaves.end());
    const std::vector<int> addresses(unique.begin(), unique.end());
    if (addresses.empty())
        throw EcNotReadyError(impl_->name + ": no axis on this port to verify a baud change with; nothing sent");
    for (int s : addresses) check_slave(s);
    require_line(LineOperation::BaudChange, impl_->name + ": baud change");  // nothing is broadcast behind a gateway
    ModbusRtu& bus = *impl_->bus;
    std::lock_guard<std::recursive_mutex> sequence(bus.sequence_mutex());
    for (int s : addresses) {
        const std::string why = not_at_rest(bus, s);
        if (!why.empty())
            throw EcNotReadyError(axis_name(impl_->name, s) + " is not at rest (" + why +
                                  "): baud change refused (nothing sent)");
    }
    PortBaudResult result;
    result.port = impl_->name;
    result.rate = rate;
    result.change = change_baud_rate(bus, rate, {addresses.front()}, options);
    if (result.change.baud_rate != rate) {
        const int found = result.change.baud_rate;
        throw VerificationError(util::format(
            "baud change to %d (code 0x%02X) not taken: %s %s at %d baud (the port is left at %d)%s", rate,
            result.change.code, axis_name(impl_->name, addresses.front()).c_str(),
            found == result.change.previous ? "still answers" : "answers", found, found,
            baud_not_taken_hint(rate, found, result.change.previous).c_str()));
    }
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->baud_detected = false;
        impl_->baud_checked = true;
    }
    result.answered[addresses.front()] = true;
    const double wait = std::max(kScanTimeoutS, 0.2);
    std::vector<std::string> missing;
    for (std::size_t i = 1; i < addresses.size(); ++i) {
        const bool ok = bus.probe(addresses[i], wait);
        result.answered[addresses[i]] = ok;
        if (!ok) missing.push_back(std::to_string(addresses[i]));
    }
    if (!missing.empty())
        log_warning(kLogSource, util::format("%s: slaves %s did not answer at %d baud after the broadcast",
                                             impl_->name.c_str(), util::join(missing, ", ").c_str(), rate));
    return result;
}

void Port::close() { impl_->bus->close(); }

// ------------------------------------------------------------------------------------------ line and gateway

std::shared_ptr<Line> Port::line() const { return impl_->bus->line(); }

std::shared_ptr<Gateway> Port::gateway() const { return impl_->bus->line()->gateway(); }

std::shared_ptr<Line> Port::detect_line(bool force) {
    std::lock_guard<std::mutex> lock(impl_->line_mutex);
    if (impl_->line_detected && !force) return impl_->bus->line();
    bool checked = false;
    {
        std::lock_guard<std::mutex> state(impl_->mutex);
        checked = impl_->baud_checked;
    }
    if (!checked) {
        try {
            ensure_baud_rate();
        } catch (const ModbusTimeout& e) {
            log_info(kLogSource, impl_->name + ": no baud rate found, line not detected yet (" + e.what() + ")");
            return impl_->bus->line();
        }
    }
    if (impl_->detector) {  // (py-cell) the bus's owner probes and attaches the line; a failure is asked again later
        try {
            impl_->detector();
        } catch (const std::exception& e) {
            log_warning(kLogSource, impl_->name + ": line not detected (" + e.what() + ")");
            return impl_->bus->line();
        }
        impl_->line_detected = true;
        return impl_->bus->line();
    }
    const std::optional<LineKind> kind = impl_->line_kind;
    std::shared_ptr<Line> line;
    if (kind && (*kind == LineKind::Direct || *kind == LineKind::Robonet)) {
        line = std::make_shared<Line>(*kind, nullptr, false, impl_->position_unit_mm);
        impl_->bus->set_line(line);
    } else {
        line = iai_actuator_driver::detect_line(*impl_->bus, kind, kGatewayProbeTimeoutS, impl_->position_unit_mm);
    }
    impl_->line_detected = true;
    return line;
}

void Port::set_position_unit(double unit_mm, const std::optional<std::vector<int>>& slaves) {
    const std::shared_ptr<Line> line = detect_line();
    if (!slaves) {
        line->set_position_unit(unit_mm);
        std::lock_guard<std::mutex> lock(impl_->line_mutex);
        impl_->position_unit_mm = unit_mm;  // kept for a later detect_line(true)
    } else {
        for (int s : *slaves) line->set_position_unit(unit_mm, s);
    }
}

std::optional<GatewayStatus> Port::gateway_status() {
    const std::shared_ptr<Gateway> gw = detect_line()->gateway();
    if (!gw) return std::nullopt;
    return gw->read_gateway_status();
}

std::map<int, RconAxisMonitor> Port::gateway_monitor(const std::optional<std::vector<int>>& slaves) {
    const std::shared_ptr<Line> line = detect_line();
    auto gw = std::dynamic_pointer_cast<RconGateway>(line->gateway());
    if (!gw)
        throw EcNotReadyError(impl_->name + ": no RCON gateway on this line (line '" + to_string(line->kind()) + "')");
    std::set<int> wanted;
    if (slaves) {
        wanted.insert(slaves->begin(), slaves->end());
    } else {
        for (int s : gw->status().linked_slaves()) wanted.insert(s);
    }
    std::vector<int> axes;
    for (int s : wanted) {
        if (s < 1 || s > kRconMaxAxes)
            throw std::invalid_argument(util::format("an axis behind an RCON gateway is slave 1..16, got %d", s));
        axes.push_back(s - 1);
    }
    std::map<int, RconAxisMonitor> out;
    if (axes.empty()) return out;
    for (auto& kv : gw->read_monitor(axes)) out[kv.second.slave()] = kv.second;
    return out;
}

void Port::require_line(LineOperation operation, const std::string& what) { detect_line()->require(operation, what); }

std::vector<std::uint8_t> Port::broadcast_stop(bool on) {
    const StopPriority stop_priority;  // before every waiting normal request of the line (bus_lock.hpp)
    std::optional<detail::StopTiming> timing;  // the stop latency line (bus_stop_timing.hpp); none for the release
    if (on) timing.emplace(*impl_->bus, 0, nullptr, "broadcast STOP", false, kLogSource);
    require_line(LineOperation::Broadcast, name() + ": broadcast");
    return impl_->bus->broadcast_coil(reg::kCoilStop, on);
}

bool Port::closed() const { return impl_->bus->closed(); }

std::vector<ScanHit> scan_port(const std::string& port_name, const std::vector<int>& slaves, bool identify,
                               const PortOptions& options) {
    std::shared_ptr<Port> port = Port::open(port_name, options);
    struct Closer {
        Port& port;
        ~Closer() {
            try {
                port.close();
            } catch (...) {
            }
        }
    } closer{*port};
    return port->scan_units(slaves, std::nullopt, identify);
}

}  // namespace iai_actuator_driver
