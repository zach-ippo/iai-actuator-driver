// One serial line shared by one or several units (one axis per COM port, or several per port by slave address =
// axis number + 1 on a daisy chain; several per port is UNVERIFIED on hardware), scanning and naming.
// Port of the Python edition's cell.Port / cell.scan / cell.axis_name (C# Port.cs, PortManager.cs). Read-only except
// set_baud_rate. Axes are created by Axis(port, slave) (axis.hpp).
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

/// Most axes on one line (IAI axis numbers 0..15, addresses 1..16), and the default scan range 1..16 of each port.
/// A Cell has no total limit (CellOptions::max_axes caps it when given).
inline constexpr int kMaxAxes = 16;

/// "COM3:1" - the port name and the slave address.
inline std::string axis_name(const std::string& port, int slave) { return port + ":" + std::to_string(slave); }

/// 1..16.
inline std::vector<int> default_scan_slaves() {
    std::vector<int> slaves;
    for (int s = 1; s <= kMaxAxes; ++s) slaves.push_back(s);
    return slaves;
}

/// One answering address found by a scan.
struct ScanHit {
    std::string port;
    int slave = 1;
    std::vector<std::uint8_t> slave_id_raw;  ///< FC11 data bytes ([] when not identified or not answered)
    std::optional<int> system_code;          ///< FC11 byte 0 (0xEC EC, 0xAE PCON...)
    std::optional<int> unit_code;            ///< FC11 byte 2
    std::string firmware;                    ///< FC11 bytes 3-4 as 4 hex digits ("" when unknown)
    std::string error;                       ///< why identification failed ("" when it worked or was not asked)
    std::string line = "direct";             ///< the line kind of the port (line.hpp)
    std::optional<std::string> serial_code;  ///< 0x9400 serial code: the identity on an RCON line (no FC11 there)

    std::string name() const { return axis_name(port, slave); }
    int axis_number() const { return slave - 1; }
};

/// The line a port is on (line.hpp, gateway.hpp).
struct LineOptions {
    /// "auto" (default: probe once for an RCON gateway at D0h and a ROBONET gateway at 3Fh, ~0.1 s on a line without
    /// one), "direct" (no probe), "rcon", "robonet" (SIO through mode, which cannot be probed) or "robonet-gateway".
    std::string kind = "auto";
    /// The position unit of every axis on an RCON line (No.207: 0.01 or 0.001 mm). Without it mm-based motion on an
    /// RCON line is refused (Line::require_motion); see Port::set_position_unit.
    std::optional<double> position_unit_mm;
};

/// (py-cell) How the owner of a borrowed bus works out its line itself (a binding's own detect_line, which attaches
/// the line to the bus): called by Port::detect_line in place of the port's own probes, once the rate is settled.
using LineDetector = std::function<void()>;

/// Settings of Port::open.
struct PortOptions {
    std::optional<int> baud_rate;              ///< nullopt: detect in place (kBaudDetectOrder) on first use
    double read_timeout_s = kDefaultReadTimeoutS;
    BusOptions bus;                            ///< retries (3), gap, frame logger
    LineOptions line;                          ///< the line kind ("auto") and the RCON position unit
};

/// Outcome of Port::set_baud_rate.
struct PortBaudResult {
    std::string port;
    int rate = 0;
    BaudChangeResult change;
    std::map<int, bool> answered;  ///< slave -> answered at the new rate
    bool all_answered() const {
        for (const auto& kv : answered)
            if (!kv.second) return false;
        return true;
    }
};

/// One open line: the bus (one mutex), its name, the baud rate found once.
class Port {
public:
    /// Wrap a transport (a SerialTransport, a SimLine, a SimulatedUnit, a FakeTransport). `name` "" = transport name.
    explicit Port(std::shared_ptr<Transport> transport, std::string name = std::string(), BusOptions options = {});
    ~Port();
    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;

    /// Open a serial port once (TransportError when busy / absent). The baud rate is detected lazily
    /// (ensure_baud_rate) unless options.baud_rate is given.
    static std::shared_ptr<Port> open(const std::string& port_name, const PortOptions& options = {});

    const std::string& name() const;
    std::shared_ptr<ModbusRtu> bus() const;
    std::optional<int> baud_rate() const;
    /// The rate was found by detect_baud_rate / ensure_baud_rate.
    bool baud_detected() const;

    /// ModbusRtu::detect_baud_rate over several slaves (kScanTimeoutS per address).
    int detect_baud_rate(const std::vector<int>& slaves = default_scan_slaves(), const std::vector<int>& candidates = {},
                         std::optional<double> probe_timeout_s = std::nullopt);

    /// Detect once (no-op when the rate was given, already detected, or the transport has no baud rate). Returns the
    /// rate (nullopt without one). ModbusTimeout when nothing answers at any rate.
    std::optional<int> ensure_baud_rate(const std::vector<int>& slaves = default_scan_slaves());

    /// The addresses that answer (one FC03 probe each, no retries, `timeout_s` default kScanTimeoutS). READ-ONLY.
    /// On an RCON line only the axes the gateway reports as linked are probed unless `use_link_map` is false. Slave 58
    /// (kAsciiStartSlave) is never probed, here and in the baud detection: one warning line says so.
    std::vector<int> scan(const std::vector<int>& slaves = default_scan_slaves(),
                          std::optional<double> timeout_s = std::nullopt, int address = 0x9000,
                          bool use_link_map = true);

    /// scan(), then (identify) FC11 for each hit (errors kept in ScanHit::error, never thrown). READ-ONLY.
    std::vector<ScanHit> scan_units(const std::vector<int>& slaves = default_scan_slaves(),
                                    std::optional<double> timeout_s = std::nullopt, bool identify = true,
                                    bool use_link_map = true);

    /// (py-cell) What one address says about itself: FC11 (no retries, waited for up to the FC11 wait), or the 0x9400
    /// serial code on a line without FC11 (RCON). Failures are kept in ScanHit::error, never thrown. READ-ONLY.
    ScanHit identify(int slave);

    /// Change every listed unit's rate and the port's (change_baud_rate), then probe each slave at the new rate.
    PortBaudResult set_baud_rate(int rate, const std::vector<int>& slaves, const BaudChangeOptions& options = {});

    /// Close the bus (and the transport). Safe to call twice. A Port does not own its axes: release their STOP / pause
    /// coils first (Axis::close(); Cell::close() does both in that order).
    void close();
    bool closed() const;

    // ---- line and gateway (line.hpp, gateway.hpp). The line is detected at the first scan (READ-ONLY FC03 probes)
    /// A port on a stated or probed line (`line`; std::invalid_argument for an unknown kind or position unit).
    Port(std::shared_ptr<Transport> transport, std::string name, BusOptions options, LineOptions line);
    /// (py-cell) A port over a bus that is open already (a binding's bus, shared with its other users). `name` "" = the
    /// transport's. `fixed_rate`: the bus's rate is used as it is, never detected. `detector`: the owner works out the
    /// line (detect_line calls it instead of probing; the port then never sets the bus's line itself); line.kind still
    /// selects the gateway addresses and rates of the baud detection.
    Port(std::shared_ptr<ModbusRtu> bus, std::string name, LineOptions line, bool fixed_rate,
         LineDetector detector = nullptr);
    /// The line's rules and gateway (bus()->line()); "direct" until detect_line() has run.
    std::shared_ptr<Line> line() const;
    /// The gateway of an RCON / ROBONET gateway-mode line (null otherwise or before detection).
    std::shared_ptr<Gateway> gateway() const;
    /// Work out the line once (detect_line, gateway.hpp) and return it. The baud rate is settled first; when no rate
    /// answers, the line stays "direct" and is asked again next time. `force` probes again.
    std::shared_ptr<Line> detect_line(bool force = false);
    /// State the position unit (0.01 or 0.001 mm; RCON parameter No.207, DOC ME0384 p.8-56) of `slaves` (nullopt:
    /// every axis on the line). Needed before mm-based motion on an RCON line.
    void set_position_unit(double unit_mm, const std::optional<std::vector<int>>& slaves = std::nullopt);
    /// A fresh read of the gateway's status (nullopt on a line without one). READ-ONLY.
    std::optional<GatewayStatus> gateway_status();
    /// The RCON all-axes monitor for `slaves` (1..16; nullopt: the slaves the gateway reports as linked), keyed by
    /// slave. At most two FC03 frames for 16 axes. EcNotReadyError on a line without an RCON gateway. READ-ONLY.
    std::map<int, RconAxisMonitor> gateway_monitor(const std::optional<std::vector<int>>& slaves = std::nullopt);
    /// Refuse an operation the line does not carry (LineRuleError, nothing sent), after detect_line().
    void require_line(LineOperation operation, const std::string& what);
    /// STOP coil 0x042C to every unit on the line at once: 00 05 04 2C FF 00 4D 12 (`on`) or the release
    /// 00 05 04 2C 00 00 0C E2. No unit answers. LineRuleError (nothing sent) on a gateway line. Returns the frame.
    /// Whether every unit acts on it (and a CON unit with PMSS = 0) is UNVERIFIED on hardware.
    std::vector<std::uint8_t> broadcast_stop(bool on = true);
    /// Pause coil STP 0x040A to every unit: 00 05 04 0A FF 00 AC D9 (`on`) / the release 00 05 04 0A 00 00 ED 29.
    /// LineRuleError (nothing sent) on a gateway line. UNVERIFIED on hardware. (src/cell_extras.cpp)
    std::vector<std::uint8_t> broadcast_pause(bool on = true);
    /// Servo OFF (coil SON 0x0403 = 0) to every unit: 00 05 04 03 00 00 3D 2B. A vertical axis without a brake may
    /// drop. LineRuleError (nothing sent) on a gateway line. UNVERIFIED on hardware. (src/cell_extras.cpp)
    std::vector<std::uint8_t> broadcast_servo_off();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// Open one serial port, find its rate (a line where nothing answers at any rate gives an empty result), scan it and
/// close it. READ-ONLY. Throws TransportError when the port cannot be opened.
std::vector<ScanHit> scan_port(const std::string& port_name, const std::vector<int>& slaves = default_scan_slaves(),
                               bool identify = true, const PortOptions& options = {});

}  // namespace iai_actuator_driver
