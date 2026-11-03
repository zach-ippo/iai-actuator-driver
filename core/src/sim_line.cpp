// SimLine: one simulated serial line carrying several SimulatedUnits by slave address (an SIO daisy chain: slave
// address = axis number + 1; UNVERIFIED on hardware). Every request frame is delivered to every unit; the addressed one
// answers, a broadcast (address 0) reaches all and nobody answers, an address nobody has stays silent. Port of the
// Python edition's sim_line.py (C# SimLine.cs). Unofficial: not an IAI product. Header-only dependencies only.
#include <algorithm>
#include <mutex>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

struct SimLine::Impl {
    mutable std::mutex mutex;
    std::string name;
    std::map<int, std::shared_ptr<SimulatedUnit>> units;
    std::vector<std::pair<double, std::vector<std::uint8_t>>> requests;
    double read_timeout = kDefaultReadTimeoutS;
    bool closed = false;
    bool present = true;       ///< the port is listed by the PC (unplug / replug)
    bool handle_lost = false;  ///< the open handle died with an unplug: every call throws until reopen()
    int reopens = 0;

    /// The units in address order (a copy, so a call never holds the line's mutex while a unit works).
    std::vector<std::shared_ptr<SimulatedUnit>> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::shared_ptr<SimulatedUnit>> list;
        for (const auto& kv : units) list.push_back(kv.second);
        return list;
    }
};

SimLine::SimLine(std::vector<std::shared_ptr<SimulatedUnit>> units, std::string name) : impl_(new Impl) {
    impl_->name = std::move(name);
    for (auto& unit : units) add(std::move(unit));
}

SimLine::~SimLine() = default;

std::shared_ptr<SimLine> SimLine::with_slaves(const std::vector<int>& slaves, SimulatedUnitOptions options,
                                              std::string name) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    for (int slave : slaves) {
        options.slave = slave;
        units.push_back(std::make_shared<SimulatedUnit>(options));
    }
    return std::make_shared<SimLine>(std::move(units), std::move(name));
}

std::map<std::string, std::shared_ptr<SimLine>> SimLine::sim_ports(int n, const std::vector<int>& slaves,
                                                                   SimulatedUnitOptions options,
                                                                   const std::string& prefix) {
    if (n < 1 || n > 64) throw std::invalid_argument("n must be 1..64");
    std::map<std::string, std::shared_ptr<SimLine>> ports;
    for (int i = 1; i <= n; ++i) {
        const std::string name = prefix + std::to_string(i);
        ports[name] = with_slaves(slaves, options, name);
    }
    return ports;
}

std::shared_ptr<SimulatedUnit> SimLine::add(std::shared_ptr<SimulatedUnit> unit) {
    if (!unit) throw std::invalid_argument("unit is null");
    const int slave = unit->slave();
    if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->units.count(slave))
        throw std::invalid_argument("slave " + std::to_string(slave) + " is already on line " + impl_->name);
    impl_->units[slave] = unit;
    return unit;
}

std::shared_ptr<SimulatedUnit> SimLine::remove(int slave) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->units.find(slave);
    if (it == impl_->units.end())
        throw std::out_of_range("no unit at slave " + std::to_string(slave) + " on line " + impl_->name);
    auto unit = it->second;
    impl_->units.erase(it);
    return unit;
}

std::shared_ptr<SimulatedUnit> SimLine::unit(int slave) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->units.find(slave);
    if (it == impl_->units.end())
        throw std::out_of_range("no unit at slave " + std::to_string(slave) + " on line " + impl_->name);
    return it->second;
}

std::map<int, std::shared_ptr<SimulatedUnit>> SimLine::units() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->units;
}

std::vector<int> SimLine::slaves() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<int> list;
    for (const auto& kv : impl_->units) list.push_back(kv.first);  // std::map: ascending
    return list;
}

std::size_t SimLine::size() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->units.size();
}

std::vector<std::pair<double, std::vector<std::uint8_t>>> SimLine::requests() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->requests;
}

std::vector<int> SimLine::request_slaves() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<int> list;
    for (const auto& r : impl_->requests)
        if (!r.second.empty()) list.push_back(r.second[0]);
    return list;
}

void SimLine::clear_requests() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->requests.clear();
}

// ---- Transport

void SimLine::write(const std::uint8_t* data, std::size_t size) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->handle_lost)
            throw TransportError("simulated line " + impl_->name + " vanished (USB unplugged): reopen it");
        if (impl_->closed) throw TransportError("simulated line " + impl_->name + " is closed");
        impl_->requests.emplace_back(now_s(), std::vector<std::uint8_t>(data, data + size));
    }
    units = impl_->snapshot();
    // every unit sees every frame; only the addressed one (none for the broadcast address 0) answers
    for (const auto& unit : units) unit->write(data, size);
}

std::size_t SimLine::read(std::uint8_t* buffer, std::size_t size) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->handle_lost)
            throw TransportError("simulated line " + impl_->name + " vanished (USB unplugged): reopen it");
        if (impl_->closed) throw TransportError("simulated line " + impl_->name + " is closed");
    }
    std::size_t got = 0;
    for (const auto& unit : impl_->snapshot()) {
        if (got >= size) break;
        got += unit->read(buffer + got, size - got);
    }
    return got;
}

void SimLine::reset_input() {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->handle_lost)
            throw TransportError("simulated line " + impl_->name + " vanished (USB unplugged): reopen it");
    }
    for (const auto& unit : impl_->snapshot()) unit->reset_input();
}

int SimLine::baud_rate() const {
    int rate = 0;
    for (const auto& unit : impl_->snapshot()) rate = std::max(rate, unit->baud_rate());
    return rate;
}

void SimLine::set_baud_rate(int rate) {
    const auto units = impl_->snapshot();
    if (units.empty() || baud_rate() == 0)
        throw std::logic_error("simulated line " + impl_->name +
                               " has no baud rate (set SimulatedUnitOptions::baud_rate on the units)");
    // the PC side's rate on the shared wire: every unit that models a line hears it at the new rate
    for (const auto& unit : units)
        if (unit->baud_rate() > 0) unit->set_baud_rate(rate);
}

double SimLine::read_timeout() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->read_timeout;
}

void SimLine::set_read_timeout(double seconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->read_timeout = seconds;  // reads never block: the value is only stored
}

void SimLine::send_break(double seconds) {
    for (const auto& unit : impl_->snapshot()) unit->send_break(seconds);  // a BREAK reaches every unit on the wire
}

void SimLine::close() {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->closed = true;
    }
    for (const auto& unit : impl_->snapshot()) unit->close();
}

bool SimLine::is_open() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return !impl_->closed;
}

std::string SimLine::name() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->name;
}

// ---- power / link loss

void SimLine::unplug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->present = false;
    impl_->handle_lost = true;
}

void SimLine::replug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->present = true;
}

std::optional<bool> SimLine::present() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->present;
}

void SimLine::reopen(std::optional<int> baud_rate) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->present)
            throw TransportError("could not open simulated line " + impl_->name + ": it is not present (unplugged)");
        impl_->handle_lost = false;
        impl_->closed = false;
        ++impl_->reopens;
    }
    for (const auto& unit : impl_->snapshot()) unit->reopen(baud_rate);
}

int SimLine::reopens() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->reopens;
}

void SimLine::power_cycle(const std::vector<int>& slaves, std::optional<bool> lose_home) {
    std::map<int, std::shared_ptr<SimulatedUnit>> list = units();
    std::vector<int> which = slaves;
    if (which.empty())
        for (const auto& kv : list) which.push_back(kv.first);
    for (int slave : which) {
        const auto it = list.find(slave);
        if (it == list.end())
            throw std::out_of_range("no unit at slave " + std::to_string(slave) + " on line " + name());
        if (lose_home) it->second->power_cycle(*lose_home);
        else it->second->power_cycle();
    }
}

// ---- hooks of a subclass that answers frames itself (SimGatewayLine)

void SimLine::record_request(const std::uint8_t* data, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle_lost)
        throw TransportError("simulated line " + impl_->name + " vanished (USB unplugged): reopen it");
    if (impl_->closed) throw TransportError("simulated line " + impl_->name + " is closed");
    impl_->requests.emplace_back(now_s(), std::vector<std::uint8_t>(data, data + size));
}

void SimLine::check_open() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->handle_lost)
        throw TransportError("simulated line " + impl_->name + " vanished (USB unplugged): reopen it");
    if (impl_->closed) throw TransportError("simulated line " + impl_->name + " is closed");
}

}  // namespace iai_actuator_driver
