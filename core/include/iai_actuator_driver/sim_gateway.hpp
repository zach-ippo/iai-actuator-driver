// Simulated lines behind a gateway (DOC only, UNVERIFIED on hardware): a SimLine whose units sit behind an RCON gateway,
// a ROBONET in SIO through mode, or a ROBONET in gateway mode. Port of the Python edition's sim_line.SimGatewayLine,
// SimulatedRconGateway, SimulatedRoboNetGateway, sim_rcon_line and sim_robonet_line; implemented by the sim unit
// (src/sim_gateway.cpp).
//
//   auto line = sim_rcon_line({1, 2, 3});                        // RCON gateway at D0h + three driver units
//   SimGatewayOptions o; o.position_unit_mm = 0.001;
//   auto fine = sim_rcon_line({1}, o);                           // No.207 = 1: positions in 0.001 mm on the wire
//   auto net = sim_robonet_line({1, 2});                         // ROBONET in SIO through mode (SW2 ON)
//   auto gw = sim_robonet_line({1, 2}, true);                    // gateway mode (SW2 OFF): only 3Fh answers
//
// * Rcon: a SimulatedRconGateway answers at D0h (status block 0x3700 with the MANU flag, alarm 0x3708, link map NLNK,
//   all-axes monitor 0x2900 + 15 x axis); the units answer FC 01/03/04/06/10/17 and exception 01 to anything else
//   (FC05, FC11, 0x45, 0x48: ME0413 p.3-1); a broadcast reaches nobody (ME0413 p.6-1 #2); 0x9400 serves a serial code;
//   with position_unit_mm 0.001 PNOW / PCMD / INP are 0.001 mm on the wire (ME0384 p.8-56; the units count 0.01 mm).
// * Robonet: FC 01-07/0F/10/11/17 answered, 0x45 / 0x48 exception 01 (ME0208 p.26); no broadcast; reads below 0x0D00
//   (directory and class items) get no answer (RCM-era firmware, modelled); frames above 160 bytes get none.
// * RobonetGateway: only a SimulatedRoboNetGateway at 3Fh answers (status words 0xF700).
// * silent_no_access (0.6): the axes never answer an FC03 read of the areas that are "system, no access" on an RCON
//   axis (below DRG1 0x0D00 and 0x8000-0x8FFF, ME0413 p.5-10): the directory then times out.
// Every frame is recorded in requests() as sent (before any unit conversion). A frame the gateway passes on reaches the
// addressed unit through its own transport (write, then read() takes its answer), as the gateway's all-axes monitor
// reads do: the unit's request log, its hooks (sim_hooks.hpp), a closed / unplugged / powered-off unit and its FC11
// delay apply as on a plain line (0.6).
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"

namespace iai_actuator_driver {

class SimGatewayLine;

/// A simulated RCON gateway (slave D0h) on a SimGatewayLine: answers FC03 / FC04 reads only.
/// * 0x3700-0x370A: GWSR (b15 GWMD = manu), GSR2 (b2 ATST = manu, b1 EMST = emergency_stop), NLNK (active low: bit n
///   = 0 for every axis n on the line or in linked_only), STIM (ms), TIMN, ALMC (u32 alarm_code), NPLK (ME0413 p.5-2);
/// * 0x2900 + 15 x axis: the all-axes monitor, read from each unit's own 0x9000 block (OLLV given as 0);
/// * 0x3300: TIMC (s). Anything else: exception 02; any other function: exception 01.
class SimulatedRconGateway {
public:
    explicit SimulatedRconGateway(int slave = 0xD0, bool manu = true, int alarm_code = 0, bool emergency_stop = false,
                                  std::vector<int> linked_only = {});

    int slave() const;
    void set_slave(int slave);           ///< e.g. 0xD7: the gateway stops answering at D0h
    bool manu() const;
    void set_manu(bool manu);            ///< the AUTO / MANU switch
    int alarm_code() const;
    void set_alarm_code(int code);
    bool emergency_stop() const;
    void set_emergency_stop(bool on);
    /// Slave addresses linked at the gateway that never answer (ECs on an RCON-EC unit).
    std::vector<int> linked_only() const;
    void set_linked_only(std::vector<int> slaves);  ///< 0.6
    /// The units on the line plus linked_only, ascending.
    std::vector<int> linked_slaves() const;
    /// The 11 words of 0x3700.
    std::vector<std::uint16_t> status_block() const;
    /// Every frame addressed to the gateway, oldest first.
    std::vector<std::vector<std::uint8_t>> requests() const;
    /// The answer to one request frame (empty: no answer).
    std::vector<std::uint8_t> handle(const std::vector<std::uint8_t>& frame);

private:
    friend class SimGatewayLine;
    std::vector<std::uint16_t> monitor(int axis) const;

    mutable std::mutex mutex_;
    int slave_;
    bool manu_;
    int alarm_code_;
    bool emergency_stop_;
    std::vector<int> linked_only_;
    std::vector<std::vector<std::uint8_t>> requests_;
    double started_;
    SimGatewayLine* line_ = nullptr;  ///< set by the line that owns this gateway
};

/// A simulated ROBONET gateway in gateway mode (slave 3Fh): FC03 of the fixed status area 0xF700-0xF707. Status 0 =
/// RUN | 4-word axis count (b4-b0); status 1 = LNK (bit n = axis n) for every unit on the line (ME0208 p.40-52,
/// p.128-129). Writes get no answer ("invalid data: no response", ME0208 p.138; this driver never writes it); other
/// functions: exception 01.
class SimulatedRoboNetGateway {
public:
    explicit SimulatedRoboNetGateway(int slave = 0x3F, bool running = true);
    int slave() const;
    void set_slave(int slave);  ///< 0.6
    bool running() const;
    void set_running(bool running);
    std::vector<std::vector<std::uint8_t>> requests() const;
    std::vector<std::uint8_t> handle(const std::vector<std::uint8_t>& frame);

private:
    friend class SimGatewayLine;
    mutable std::mutex mutex_;
    int slave_;
    bool running_;
    std::vector<std::vector<std::uint8_t>> requests_;
    SimGatewayLine* line_ = nullptr;
};

/// Settings of a SimGatewayLine.
struct SimGatewayOptions {
    LineKind kind = LineKind::Rcon;          ///< Rcon, Robonet or RobonetGateway (Direct: std::invalid_argument)
    std::optional<std::string> name;         ///< default "GW" (sim_rcon_line "RCON", sim_robonet_line "ROBONET")
    bool manu = true;                        ///< RCON gateway state (SimulatedRconGateway)
    int gateway_alarm = 0;
    bool emergency_stop = false;
    std::vector<int> linked_only;
    double position_unit_mm = 0.01;          ///< RCON No.207 of every unit: 0.01 or 0.001
    std::map<int, double> position_units;    ///< per slave (overrides position_unit_mm)
    std::map<int, std::string> serials;      ///< RCON serial codes at 0x9400 (default "SIMRCON-<axis, 2 digits>")
    bool silent_no_access = false;           ///< 0.6: no answer to FC03 reads of an RCON axis's no-access areas
};

/// A simulated line behind a gateway. dropped_broadcasts() lists the broadcast frames nobody received; rejected() the
/// frames a unit answered with exception 01 because the gateway does not carry the function.
class SimGatewayLine : public SimLine {
public:
    explicit SimGatewayLine(std::vector<std::shared_ptr<SimulatedUnit>> units = {}, SimGatewayOptions options = {});
    ~SimGatewayLine() override;

    LineKind kind() const;
    /// The RCON gateway (null on a ROBONET line).
    std::shared_ptr<SimulatedRconGateway> rcon_gateway() const;
    /// The ROBONET gateway (gateway mode only; null otherwise).
    std::shared_ptr<SimulatedRoboNetGateway> robonet_gateway() const;
    /// The function codes the units answer (the rest: exception 01).
    std::vector<int> function_codes() const;
    double position_unit(int slave) const;
    /// Wire counts per simulated count: 10 for a 0.001 mm unit on RCON, else 1.
    int position_factor(int slave) const;
    std::string serial_code(int slave) const;
    std::vector<std::vector<std::uint8_t>> dropped_broadcasts() const;
    std::vector<std::vector<std::uint8_t>> rejected() const;
    bool silent_no_access() const;           ///< 0.6
    void set_silent_no_access(bool silent);  ///< 0.6

    // Transport
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;

private:
    void emit(const std::vector<std::uint8_t>& reply);
    std::vector<std::uint8_t> serial_reply(int slave, int start, int count) const;
    std::vector<std::uint8_t> to_unit(int slave, const std::vector<std::uint8_t>& frame) const;
    std::vector<std::uint8_t> from_unit(const std::vector<std::uint8_t>& request,
                                        const std::vector<std::uint8_t>& reply) const;

    LineKind kind_;
    std::shared_ptr<SimulatedRconGateway> rcon_;
    std::shared_ptr<SimulatedRoboNetGateway> robonet_;
    double unit_mm_;
    std::map<int, double> units_mm_;
    std::map<int, std::string> serials_;
    mutable std::mutex gw_mutex_;
    std::vector<std::uint8_t> out_;
    std::vector<std::vector<std::uint8_t>> dropped_;
    std::vector<std::vector<std::uint8_t>> rejected_;
    bool silent_no_access_ = false;
    std::shared_ptr<SimulatedUnit> pending_unit_;  ///< the unit whose answer read() takes next (with the request)
    std::vector<std::uint8_t> pending_request_;
};

/// The unit defaults of the gateway lines: the Pcon profile (the closest to an RCON driver unit).
inline SimulatedUnitOptions sim_gateway_unit_defaults() {
    SimulatedUnitOptions options;
    options.profile = SimProfile::Pcon;
    return options;
}

/// An RCON line: a simulated gateway at D0h and one SimulatedUnit per address (`unit` with its slave replaced;
/// default profile Pcon, the closest to an RCON driver unit). options.kind is set to Rcon.
std::shared_ptr<SimGatewayLine> sim_rcon_line(const std::vector<int>& slaves = {1}, SimGatewayOptions options = {},
                                              SimulatedUnitOptions unit = sim_gateway_unit_defaults());
/// A ROBONET line in SIO through mode (default) or, with gateway_mode, in gateway mode (only 3Fh answers).
std::shared_ptr<SimGatewayLine> sim_robonet_line(const std::vector<int>& slaves = {1}, bool gateway_mode = false,
                                                 SimulatedUnitOptions unit = sim_gateway_unit_defaults(),
                                                 const std::string& name = "ROBONET");

}  // namespace iai_actuator_driver
