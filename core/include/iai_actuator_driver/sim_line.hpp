// One simulated serial line carrying several SimulatedUnits by slave address, the way an SIO daisy chain puts several
// axes on one RS-485 line (slave address = axis number + 1; UNVERIFIED on hardware). Use it wherever a Transport is:
// Port(line), ModbusRtu(line), Cell::open({{"SIM1", line}}). Port of the Python edition's sim_line.py (C# SimLine.cs);
// implemented by the sim unit (src/sim_line.cpp).
//
// Every request frame is delivered to every unit: the addressed unit answers, the others stay silent, a frame to the
// broadcast address 0 reaches all of them and none answers, an address nobody has stays silent. Two units cannot share
// an address. A baud-rate change of the line and a BREAK reach every unit. requests() records (time, frame) of every
// request, so tests can check how the frames of several axes interleave. Reads never block (like the units); read()
// gathers the units' pending answers in address order; write() / read() on a closed line throw TransportError; the
// line's set_baud_rate() changes the PC-side rate of every unit that models one (std::logic_error when none does).
// Evidence: UNVERIFIED on hardware (this edition; the Python edition's line is UNVERIFIED on a multi-drop wire too).
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

class SimLine : public Transport {
public:
    /// A line named `name` with `units` on it. std::invalid_argument for an address outside 1..247 or taken twice.
    explicit SimLine(std::vector<std::shared_ptr<SimulatedUnit>> units = {}, std::string name = "SIM");
    ~SimLine() override;
    SimLine(const SimLine&) = delete;
    SimLine& operator=(const SimLine&) = delete;

    /// A line with one unit per slave address, each built from `options` with its slave replaced.
    static std::shared_ptr<SimLine> with_slaves(const std::vector<int>& slaves, SimulatedUnitOptions options = {},
                                                std::string name = "SIM");
    /// `n` lines "SIM1".."SIMn" ({name: line}), each with `slaves` (default {1}).
    static std::map<std::string, std::shared_ptr<SimLine>> sim_ports(int n, const std::vector<int>& slaves = {1},
                                                                     SimulatedUnitOptions options = {},
                                                                     const std::string& prefix = "SIM");

    /// Add a unit (std::invalid_argument when its address is taken). Returns it.
    std::shared_ptr<SimulatedUnit> add(std::shared_ptr<SimulatedUnit> unit);
    /// Remove the unit at `slave` (std::out_of_range when absent). Returns it.
    std::shared_ptr<SimulatedUnit> remove(int slave);
    /// The unit at `slave` (std::out_of_range when absent).
    std::shared_ptr<SimulatedUnit> unit(int slave) const;
    std::map<int, std::shared_ptr<SimulatedUnit>> units() const;
    std::vector<int> slaves() const;
    std::size_t size() const;

    /// (now_s(), frame) of every request, oldest first.
    std::vector<std::pair<double, std::vector<std::uint8_t>>> requests() const;
    /// The slave byte of every request, in order.
    std::vector<int> request_slaves() const;
    void clear_requests();

    // ---- power / link loss (recovery.hpp). Port of the Python edition's SimLine.unplug / replug / reopen /
    // power_cycle; remove() / add() while unplugged model a unit swapped for another.
    /// The USB adapter pulled: the port vanishes (present() false) and the open handle dies: write / read throw
    /// TransportError until reopen() after replug(). The units keep their power, state and baud rate.
    void unplug();
    /// Plugged back in: the port is listed again; the old handle stays dead until reopen().
    void replug();
    std::optional<bool> present() const override;
    bool can_reopen() const override { return true; }
    /// Open the line again after close() or unplug() (TransportError while unplugged), its units too. `baud_rate`:
    /// the PC side's rate afterwards (units that model one; nullopt: unchanged).
    void reopen(std::optional<int> baud_rate = std::nullopt) override;
    int reopens() const;
    /// A 24 V power cycle of the units at `slaves` (empty: every unit); `lose_home` as SimulatedUnit::power_on.
    void power_cycle(const std::vector<int>& slaves = {}, std::optional<bool> lose_home = std::nullopt);

    // Transport (the line's rate = the units' PC-side rate; 0 when no unit models one)
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override;
    void set_baud_rate(int rate) override;
    double read_timeout() const override;
    void set_read_timeout(double seconds) override;
    void send_break(double seconds) override;
    void close() override;
    bool is_open() const override;
    std::string name() const override;

protected:
    /// For a subclass that answers frames itself (SimGatewayLine, sim_gateway.hpp): the checks of write() (closed /
    /// unplugged: TransportError), then record `frame` in requests().
    void record_request(const std::uint8_t* data, std::size_t size);
    /// The checks of read(): TransportError when the line is closed or unplugged.
    void check_open() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iai_actuator_driver
