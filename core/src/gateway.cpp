// The read-only gateway objects, the line probes and detect_line() (port of the Python edition's gateway.py). Every
// gateway register and rule is DOC only (ME0413, ME0208) and UNVERIFIED on hardware; this C++ code is UNVERIFIED on
// hardware. Nothing here writes to a gateway.
#include "iai_actuator_driver/gateway.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr const char* kLogSource = "iai_actuator_driver.gateway";

/// One FC03 read, no retries, at most `timeout_s`; nullopt on silence, a bad frame or an exception reply.
std::optional<std::vector<std::uint16_t>> try_read(ModbusRtu& bus, int slave, int address, int count,
                                                   double timeout_s) {
    ReadOptions options;
    options.retries = 0;
    options.max_wait_s = timeout_s;
    try {
        return bus.read_registers(slave, address, count, options);
    } catch (const TransportError&) {
        throw;  // the port failed: not a missing gateway
    } catch (const ModbusError& e) {
        log_debug(kLogSource, util::format("probe 0x%02X @0x%04X: %s", slave, address, e.what()));
        return std::nullopt;
    }
}

/// 8 words as ASCII, two characters per word (high byte first), NULs as spaces, trimmed.
std::string ascii(const std::vector<std::uint16_t>& words) {
    std::string text;
    for (std::uint16_t w : words) {
        for (int shift : {8, 0}) {
            const unsigned char c = static_cast<unsigned char>((w >> shift) & 0xFF);
            text += c == 0 ? ' ' : (c < 0x80 ? static_cast<char>(c) : '?');
        }
    }
    return util::trim(text);
}

}  // namespace

// ------------------------------------------------------------------------------------------ RconGateway

RconGateway::RconGateway(ModbusRtu& bus, int address, std::optional<RconGatewayStatus> status, double cache_s)
    : bus_(bus), address_(address), cache_s_(cache_s), status_(std::move(status)) {
    if (address < 1 || address > 255) throw std::invalid_argument(util::format("gateway address must be 1..255, got %d", address));
    if (!(cache_s >= 0.0)) throw std::invalid_argument("cache_s must be >= 0");
}

std::string RconGateway::to_string() const { return util::format("RconGateway(0x%02X)", address_); }

std::optional<GatewayStatus> RconGateway::last_gateway_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_) return std::nullopt;
    return GatewayStatus(*status_);
}

double RconGateway::cache_s() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_s_;
}

void RconGateway::set_cache_s(double seconds) {
    if (!(seconds >= 0.0)) throw std::invalid_argument("cache_s must be >= 0");
    std::lock_guard<std::mutex> lock(mutex_);
    cache_s_ = seconds;
}

std::optional<RconGatewayStatus> RconGateway::last_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

RconGatewayStatus RconGateway::read_status() {
    RconGatewayStatus st;
    st.words = bus_.read_registers(address_, kRconGwStatus, kRconGwStatusWords);
    st.read_at = now_s();
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = st;
    return st;
}

RconGatewayStatus RconGateway::status(std::optional<double> max_age_s) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double age = max_age_s ? *max_age_s : cache_s_;
        if (status_ && now_s() - status_->read_at <= age) return *status_;
    }
    return read_status();
}

std::uint32_t RconGateway::read_alarm() {
    const std::vector<std::uint16_t> w = bus_.read_registers(address_, kRconGwAlarm, 2);
    return (static_cast<std::uint32_t>(w[0]) << 16) | w[1];
}

RconGatewayStatus RconGateway::require_manu() {
    RconGatewayStatus st;
    try {
        st = status();
        if (!st.manu()) st = read_status();
    } catch (const ModbusError& e) {
        GatewayModeError error(util::format(
            "the RCON gateway (0x%02X) did not answer its status read, so MANU mode cannot be confirmed: %s",
            address_, e.what()));
        throw error;
    }
    if (!st.manu())
        throw GatewayModeError(
            "the RCON gateway is in AUTO mode: Modbus control works only in MANU mode (DOC ME0413 p.6-1 #5; ME0384 "
            "p.3-11). Set the gateway's AUTO/MANU switch to MANU; nothing was sent");
    return st;
}

std::map<int, RconAxisMonitor> RconGateway::read_monitor(const std::optional<std::vector<int>>& axes) {
    std::set<int> unique;
    if (axes) {
        unique.insert(axes->begin(), axes->end());
    } else {
        for (int a = 0; a < kRconMaxAxes; ++a) unique.insert(a);
    }
    const std::vector<int> wanted(unique.begin(), unique.end());
    for (int a : wanted)
        if (a < 0 || a >= kRconMaxAxes)
            throw std::invalid_argument(util::format("RCON axis number must be 0..%d, got %d", kRconMaxAxes - 1, a));
    std::map<int, RconAxisMonitor> out;
    std::size_t i = 0;
    while (i < wanted.size()) {
        const int first = wanted[i];
        std::vector<int> span;
        for (std::size_t j = i; j < wanted.size() && wanted[j] < first + kRconMonitorAxesPerRead; ++j)
            span.push_back(wanted[j]);
        const int last = span.back();
        const int count = (last - first + 1) * kRconMonitorStride;
        ReadOptions options;
        options.chunk = count;
        const std::vector<std::uint16_t> words =
            bus_.read_registers(address_, kRconMonitorBase + kRconMonitorStride * first, count, options);
        for (int a : span) {
            const std::size_t off = static_cast<std::size_t>((a - first) * kRconMonitorStride);
            RconAxisMonitor m;
            m.axis = a;
            m.words.assign(words.begin() + static_cast<std::ptrdiff_t>(off),
                           words.begin() + static_cast<std::ptrdiff_t>(off + kRconMonitorStride));
            out[a] = std::move(m);
        }
        i += span.size();
    }
    return out;
}

// ------------------------------------------------------------------------------------------ RoboNetGateway

RoboNetGateway::RoboNetGateway(ModbusRtu& bus, std::optional<RoboNetGatewayStatus> status)
    : bus_(bus), status_(std::move(status)) {}

std::optional<GatewayStatus> RoboNetGateway::last_gateway_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_) return std::nullopt;
    return GatewayStatus(*status_);
}

std::optional<RoboNetGatewayStatus> RoboNetGateway::last_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

RoboNetGatewayStatus RoboNetGateway::read_status() {
    RoboNetGatewayStatus st;
    st.words = bus_.read_registers(kRoboNetGatewaySlave, kRoboNetGwStatus, kRoboNetGwStatusWords);
    st.read_at = now_s();
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = st;
    return st;
}

RoboNetGatewayStatus RoboNetGateway::status(std::optional<double> max_age_s) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_ && max_age_s && now_s() - status_->read_at <= *max_age_s) return *status_;
    }
    return read_status();
}

// ------------------------------------------------------------------------------------------ probes and detection

std::optional<RconGatewayStatus> probe_rcon_gateway(ModbusRtu& bus, int address, double timeout_s) {
    const auto words = try_read(bus, address, kRconGwStatus, kRconGwStatusWords, timeout_s);
    if (!words || static_cast<int>(words->size()) != kRconGwStatusWords) return std::nullopt;
    RconGatewayStatus st;
    st.words = *words;
    st.read_at = now_s();
    return st;
}

std::optional<RoboNetGatewayStatus> probe_robonet_gateway(ModbusRtu& bus, double timeout_s) {
    const auto words = try_read(bus, kRoboNetGatewaySlave, kRoboNetGwStatus, kRoboNetGwStatusWords, timeout_s);
    if (!words || static_cast<int>(words->size()) != kRoboNetGwStatusWords) return std::nullopt;
    RoboNetGatewayStatus st;
    st.words = *words;
    st.read_at = now_s();
    return st;
}

std::shared_ptr<Line> detect_line(ModbusRtu& bus, std::optional<LineKind> kind, double timeout_s,
                                  std::optional<double> position_unit_mm) {
    std::shared_ptr<Gateway> gateway;
    LineKind found = kind ? *kind : LineKind::Direct;
    const bool any = !kind.has_value();
    if (any || *kind == LineKind::Rcon) {
        const double wait = any ? timeout_s : std::max(timeout_s, kGatewayFirstContactS);
        if (auto st = probe_rcon_gateway(bus, kRconGatewaySlave, wait)) {
            gateway = std::make_shared<RconGateway>(bus, kRconGatewaySlave, *st);
            found = LineKind::Rcon;
        }
    }
    if (!gateway && (any || *kind == LineKind::RobonetGateway)) {
        const double wait = any ? timeout_s : std::max(timeout_s, kRoboNetProbeTimeoutS);
        if (auto st = probe_robonet_gateway(bus, wait)) {
            gateway = std::make_shared<RoboNetGateway>(bus, *st);
            found = LineKind::RobonetGateway;
        }
    }
    auto line = std::make_shared<Line>(found, gateway, gateway != nullptr, position_unit_mm);
    if (found != LineKind::Direct)
        log_info(kLogSource, "line detected: " + line->rules().title + " (" + line->rules().evidence + ")");
    if (found == LineKind::RobonetGateway) log_warning(kLogSource, kRoboNetSw2);
    bus.set_line(line);
    return line;
}

std::optional<std::string> read_serial_code(ModbusRtu& bus, int slave) {
    std::vector<std::uint16_t> words;
    try {
        words = bus.read_registers(slave, kRconSerialBase, kRconSerialWords);
    } catch (const ModbusError& e) {
        log_debug(kLogSource, util::format("slave %d: serial code 0x9400 not readable: %s", slave, e.what()));
        return std::nullopt;
    }
    std::string text = ascii(words);
    if (text.empty()) return std::nullopt;
    return text;
}

std::optional<std::string> robonet_hint(ModbusRtu& bus, int slave, bool directory_timed_out) {
    if (bus.line()->kind() != LineKind::Direct || !directory_timed_out) return std::nullopt;
    if (!bus.unsupported_functions(slave).count(kFcIaiCommand)) return std::nullopt;
    return util::format(
        "slave %d answers 0x9000 but rejects IAI command 0x45 and has no 0x0000 directory: this may be a ROBONET axis "
        "in SIO through mode (DOC ME0208 p.26); open the port with line='robonet' (UNVERIFIED heuristic)",
        slave);
}

}  // namespace iai_actuator_driver
