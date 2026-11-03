// The read-only gateway objects (RCON, ROBONET gateway mode), the two line probes and detect_line(). Port of the
// Python edition's gateway.py (RconGateway, RoboNetGateway, detect_line, read_serial_code, robonet_hint); the line
// rules and the request guard are in line.hpp.
//
// Detection (detect_line, run by Port at the first scan): two read-only FC03 probes, each waiting at most
// kGatewayProbeTimeoutS (0.05 s), so a normal line pays about 0.1 s once:
//   D0 03 37 00 00 0B 18 38   the RCON gateway status block 0x3700 (GWSR .. NPLK, ME0413 p.5-2, 5-3);
//   3F 03 F7 00 00 02 F2 A1   the ROBONET gateway status words 0xF700 (ME0208 p.128-129).
// Only a normal reply (not an exception) counts. ROBONET THROUGH mode is transparent and cannot be probed: open the
// port with line "robonet" (robonet_hint gives an UNVERIFIED hint).
//
// The gateways are never written by this driver. Every gateway rule and register is DOC only (ME0413, ME0208):
// UNVERIFIED on hardware in both editions. A gateway object keeps a reference to its bus: it is valid while the bus
// lives (the bus's Line holds the gateway).
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"

namespace iai_actuator_driver {

// ------------------------------------------------------------------------------------------ registers

/// GWSR (0x3700) .. NPLK (0x370A): 11 words. DOC ME0413 p.5-2, 5-3.
inline constexpr int kRconGwStatus = 0x3700;
inline constexpr int kRconGwStatusWords = 0x0B;
/// Gateway alarm code, u32 high word first (e.g. 0x000008BA). DOC ME0413 p.5-2; ME0425 p.70.
inline constexpr int kRconGwAlarm = 0x3708;
/// All-axes monitor: 15 words per axis at 0x2900 + 15 x axis (axes 0-15). DOC ME0413 p.5-2.
inline constexpr int kRconMonitorBase = 0x2900;
inline constexpr int kRconMonitorStride = 15;
/// 8 axes = 120 words per FC03 frame (D0 03 29 00 00 78 5F F5), so 16 axes take two frames.
inline constexpr int kRconMonitorAxesPerRead = 8;
inline constexpr int kRconMaxAxes = 16;
/// Serial codes CTS1..CTS4, ASCII, 8 words, on each driver unit. DOC ME0413 p.5-10 to 5-23 (SCON-CB-RC from V000D).
inline constexpr int kRconSerialBase = 0x9400;
inline constexpr int kRconSerialWords = 8;
/// Gateway status 0 (RUN, counts) and status 1 (LNK15..LNK0). DOC ME0208 p.40, p.48, p.123, p.128-129.
inline constexpr int kRoboNetGwStatus = 0xF700;
inline constexpr int kRoboNetGwStatusWords = 2;

// ------------------------------------------------------------------------------------------ RCON

/// The RCON gateway status block 0x3700-0x370A (DOC ME0413 p.5-2, 5-3), as read. READ-ONLY. linked_axes decodes NLNK
/// (0x3703, active low) as bit n = axis n: the bit order is not stated in the manual (UNVERIFIED). auto_manu_status
/// is GSR2 b2 ATST (its polarity is not stated: UNVERIFIED).
struct RconGatewayStatus {
    std::vector<std::uint16_t> words;
    double read_at = 0.0;  ///< now_s() of the read

    int word(std::size_t offset) const { return offset < words.size() ? words[offset] : 0; }
    int gwsr() const { return word(0); }
    int gsr2() const { return word(1); }
    int nlnk() const { return word(3); }
    /// GWSR b15 GWMD: 1 = MANU (Modbus control possible), 0 = AUTO (field network / PLC).
    bool manu() const { return (gwsr() & 0x8000) != 0; }
    /// GWSR b14 MONS: field-network monitoring active.
    bool monitor_active() const { return (gwsr() & 0x4000) != 0; }
    /// GWSR b13 ERRT or b12 ERRC (communication errors; ERRC from firmware V00006).
    bool link_error() const { return (gwsr() & 0x3000) != 0; }
    /// GSR2 b2 ATST (UNVERIFIED polarity).
    bool auto_manu_status() const { return (gsr2() & 0x0004) != 0; }
    /// GSR2 b1 EMST: the emergency-stop input is active.
    bool emergency_stop() const { return (gsr2() & 0x0002) != 0; }
    /// STIM 0x3704 (u32).
    std::uint32_t system_timer_ms() const {
        return (static_cast<std::uint32_t>(word(4)) << 16) | static_cast<std::uint32_t>(word(5));
    }
    /// Gateway alarm code ALMC 0x3708 (u32), e.g. 0x8BA "axes / operation mode mismatch"; 0 = none.
    std::uint32_t alarm_code() const {
        return (static_cast<std::uint32_t>(word(8)) << 16) | static_cast<std::uint32_t>(word(9));
    }
    /// The alarm's name from the alarm tables (nullopt without an alarm). Implemented by the model unit.
    std::optional<std::string> alarm_name() const;
    /// Axis numbers whose NLNK bit is 0 (linked). Bit order UNVERIFIED (bit n = axis n assumed).
    std::vector<int> linked_axes() const {
        std::vector<int> out;
        for (int n = 0; n < kRconMaxAxes; ++n)
            if (!(nlnk() & (1 << n))) out.push_back(n);
        return out;
    }
    /// Slave addresses (axis number + 1) of linked_axes.
    std::vector<int> linked_slaves() const {
        std::vector<int> out;
        for (int n : linked_axes()) out.push_back(n + 1);
        return out;
    }
};

/// One axis's record of the RCON all-axes monitor (15 words at 0x2900 + 15 x axis, DOC ME0413 p.5-2). Positions are
/// raw counts in the unit's position unit (No.207: 0.01 or 0.001 mm); speeds are 0.01 mm/s. The record has no DSS2,
/// ZONS or POSS: read the axis's own 0x9000 block for those.
struct RconAxisMonitor {
    int axis = 0;
    std::vector<std::uint16_t> words;  ///< 15

    int slave() const { return axis + 1; }
    std::uint16_t word(std::size_t i) const { return i < words.size() ? words[i] : 0; }
    std::int32_t i32(std::size_t i) const {
        return static_cast<std::int32_t>((static_cast<std::uint32_t>(word(i)) << 16) | word(i + 1));
    }
    /// PNOW (s32), in the unit's position unit.
    std::int32_t position_counts() const { return i32(0); }
    /// VNOW (s32), 0.01 mm/s.
    std::int32_t velocity_counts() const { return i32(2); }
    /// CNOW (mA; command current on RCON-PC, feedback current on AC/DC/SC).
    std::int32_t current_ma() const { return i32(4); }
    int alarm_code() const { return word(6); }
    int dss1() const { return word(7); }
    int dsse() const { return word(8); }
    int sipm() const { return word(9); }
    /// OLLV, overload level in %.
    int overload_pct() const { return word(10); }
    /// DEVI (s32), pulses.
    std::int32_t deviation_pulses() const { return i32(11); }
    /// STAT (u32).
    std::uint32_t stat() const { return static_cast<std::uint32_t>(i32(13)); }
    /// The record in the 14-word layout of the axis's own 0x9000 block (PNOW .. CNOW), for Status decoding. DIPM,
    /// DOPM and DSS2 are not in the gateway record and are given as 0.
    std::vector<std::uint16_t> status_words() const {
        return {word(0), word(1), word(6), 0, 0, word(7), 0, word(8), word(13), word(14), word(2), word(3),
                word(4), word(5)};
    }
};

// ------------------------------------------------------------------------------------------ ROBONET (gateway mode)

/// ROBONET gateway status words 0xF700 / 0xF701 (DOC ME0208 p.40-52, p.128-129). READ-ONLY. Example reply
/// 3F 03 04 80 21 00 03 1C 3B: RUN, one 4-word axis, one 8-word axis, LNK0 + LNK1.
struct RoboNetGatewayStatus {
    std::vector<std::uint16_t> words;
    double read_at = 0.0;

    int status0() const { return words.empty() ? 0 : words[0]; }
    int link() const { return words.size() > 1 ? words[1] : 0; }
    /// b15 RUN: the gateway runs normally.
    bool running() const { return (status0() & 0x8000) != 0; }
    /// b12 MOD: the front switch is at MANU.
    bool manu() const { return (status0() & 0x1000) != 0; }
    /// b9-b5: number of direct-numerical (8-word) axes.
    int direct_axes() const { return (status0() >> 5) & 0x1F; }
    /// b4-b0: number of positioner 1 / simple direct (4-word) axes.
    int positioner_axes() const { return status0() & 0x1F; }
    /// Axis numbers whose LNK bit is set (bit n = axis n linked).
    std::vector<int> linked_axes() const {
        std::vector<int> out;
        for (int n = 0; n < 16; ++n)
            if (link() & (1 << n)) out.push_back(n);
        return out;
    }
};

/// The status of either gateway.
using GatewayStatus = std::variant<RconGatewayStatus, RoboNetGatewayStatus>;

// ------------------------------------------------------------------------------------------ gateway objects

/// A gateway on a line (RconGateway or RoboNetGateway). Read-only: never written by this driver.
class Gateway {
public:
    virtual ~Gateway() = default;
    /// LineKind::Rcon or LineKind::RobonetGateway.
    virtual LineKind kind() const = 0;
    /// Its slave address (D0h / 3Fh).
    virtual int address() const = 0;
    /// "RconGateway(0xD0)" / "RoboNetGateway(0x3F)".
    virtual std::string to_string() const = 0;
    /// The last status read (nullopt before the first read).
    virtual std::optional<GatewayStatus> last_gateway_status() const = 0;
    /// A fresh read of the status. READ-ONLY.
    virtual GatewayStatus read_gateway_status() = 0;
};

/// The RCON gateway (slave D0h): status, MANU flag, alarm, link map, all-axes monitor. Never written by this driver
/// (DOC ME0413 p.5-2 to 5-9; UNVERIFIED on hardware). The MANU flag is cached for cache_s seconds; a refusal always
/// re-reads it first. Thread-safe.
class RconGateway : public Gateway {
public:
    explicit RconGateway(ModbusRtu& bus, int address = kRconGatewaySlave,
                         std::optional<RconGatewayStatus> status = std::nullopt, double cache_s = kGwsrCacheS);

    LineKind kind() const override { return LineKind::Rcon; }
    int address() const override { return address_; }
    std::string to_string() const override;
    std::optional<GatewayStatus> last_gateway_status() const override;
    GatewayStatus read_gateway_status() override { return read_status(); }

    double cache_s() const;
    void set_cache_s(double seconds);  ///< >= 0 (0: every status() reads)
    /// The last status read (nullopt before the first read).
    std::optional<RconGatewayStatus> last_status() const;
    /// FC03 D0 03 37 00 00 0B: GWSR .. NPLK. READ-ONLY.
    RconGatewayStatus read_status();
    /// The cached status when younger than max_age_s (default cache_s()), else a fresh read.
    RconGatewayStatus status(std::optional<double> max_age_s = std::nullopt);
    /// FC03 D0 03 37 08 00 02: the gateway alarm code (u32; 0 = none). READ-ONLY.
    std::uint32_t read_alarm();
    /// GatewayModeError unless the gateway reports MANU (GWSR b15 GWMD = 1). A cached AUTO is re-read before refusing;
    /// a gateway that does not answer is a refusal too.
    RconGatewayStatus require_manu();
    /// The all-axes monitor for `axes` (axis numbers 0-15; default all 16) in as few frames as possible: at most 8
    /// axes (120 words) per FC03 frame, e.g. D0 03 29 00 00 78 5F F5 + D0 03 29 78 00 78 DF EC for all 16.
    /// std::invalid_argument for an axis outside 0..15. READ-ONLY.
    std::map<int, RconAxisMonitor> read_monitor(const std::optional<std::vector<int>>& axes = std::nullopt);

private:
    ModbusRtu& bus_;
    int address_;
    mutable std::mutex mutex_;
    double cache_s_;
    std::optional<RconGatewayStatus> status_;
};

/// A ROBONET gateway in gateway mode (3Fh): read-only status only. The driver never writes it (UNVERIFIED).
class RoboNetGateway : public Gateway {
public:
    explicit RoboNetGateway(ModbusRtu& bus, std::optional<RoboNetGatewayStatus> status = std::nullopt);

    LineKind kind() const override { return LineKind::RobonetGateway; }
    int address() const override { return kRoboNetGatewaySlave; }
    std::string to_string() const override { return "RoboNetGateway(0x3F)"; }
    std::optional<GatewayStatus> last_gateway_status() const override;
    GatewayStatus read_gateway_status() override { return read_status(); }

    std::optional<RoboNetGatewayStatus> last_status() const;
    /// FC03 3F 03 F7 00 00 02. READ-ONLY.
    RoboNetGatewayStatus read_status();
    /// The cached status when younger than max_age_s (nullopt: always a fresh read).
    RoboNetGatewayStatus status(std::optional<double> max_age_s = std::nullopt);

private:
    ModbusRtu& bus_;
    mutable std::mutex mutex_;
    std::optional<RoboNetGatewayStatus> status_;
};

// ------------------------------------------------------------------------------------------ probes and detection

/// D0 03 37 00 00 0B 18 38: the RCON gateway status block, or nullopt when no gateway answers (silence, a bad frame or
/// an exception reply). One read, no retries, at most `timeout_s`. READ-ONLY.
std::optional<RconGatewayStatus> probe_rcon_gateway(ModbusRtu& bus, int address = kRconGatewaySlave,
                                                    double timeout_s = kGatewayProbeTimeoutS);
/// 3F 03 F7 00 00 02 F2 A1: the ROBONET gateway status words, or nullopt when none answers. READ-ONLY.
std::optional<RoboNetGatewayStatus> probe_robonet_gateway(ModbusRtu& bus, double timeout_s = kGatewayProbeTimeoutS);

/// Work out the line `bus` is on, attach it (bus.set_line) and return it. READ-ONLY (FC03 probes only). `kind`
/// nullopt ("auto") probes for an RCON gateway (D0h) and then a ROBONET gateway in gateway mode (3Fh); when neither
/// answers the line is "direct". Rcon / RobonetGateway probe that gateway only (an RCON probe then waits
/// kGatewayFirstContactS; a stated kind is kept when its gateway does not answer); Direct and Robonet (through mode,
/// which cannot be probed) send nothing. Call it once the baud rate is settled.
std::shared_ptr<Line> detect_line(ModbusRtu& bus, std::optional<LineKind> kind = std::nullopt,
                                  double timeout_s = kGatewayProbeTimeoutS,
                                  std::optional<double> position_unit_mm = std::nullopt);

/// The unit's serial code, 01 03 94 00 00 08 68 3C (8 words ASCII, NULs as spaces, trimmed): the identity where FC11
/// is not documented. DOC ME0413 p.5-10 to 5-23 (RCON driver units; SCON-CB-RC from V000D). nullopt when the read is
/// refused or the text is empty. READ-ONLY.
std::optional<std::string> read_serial_code(ModbusRtu& bus, int slave);

/// An UNVERIFIED hint that a "direct" line may be a ROBONET in SIO through mode: given when IAI command 0x45 was
/// answered with exception 01 (bus.unsupported_functions(slave)) and the 0x0000 directory read timed out on an axis
/// that answers 0x9000. nullopt otherwise.
std::optional<std::string> robonet_hint(ModbusRtu& bus, int slave, bool directory_timed_out);

// ------------------------------------------------------------------------------------------ plain data and JSON

/// A Line as plain data (Line::snapshot).
struct LineSnapshot {
    LineKind kind = LineKind::Direct;
    bool detected = false;
    LineRules rules;
    std::optional<GatewayStatus> gateway;                              ///< the gateway's last status read
    std::map<int, std::pair<double, std::string>> position_units;      ///< slave (0 = every axis) -> (unit, source)
    std::vector<int> unreachable_slaves;
    std::optional<std::string> explanation;
};

// JSON forms (field names follow the Python edition's to_dict()). Implemented by the model unit (src/model_gateway.cpp).
Json to_json(const LineRules& value);
Json to_json(const LineSnapshot& value);   ///< position_units keyed "all" / "<slave>"
Json to_json(const RconGatewayStatus& value);
Json to_json(const RconAxisMonitor& value);
Json to_json(const RoboNetGatewayStatus& value);
Json to_json(const GatewayStatus& value);

}  // namespace iai_actuator_driver
