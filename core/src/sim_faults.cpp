// SimulatedUnit fault injection for power / link-loss recovery (recovery.hpp): the 24 V supply cut and back
// (power_off / power_on / power_cycle(lose_home)), a pulled USB adapter (unplug / replug / reopen) and a different
// controller serial (SimulatedUnitOptions::serial_number, a replaced unit). Port of the Python edition's
// SimulatedEC.power_off / power_on / unplug / replug / reopen. What the hooks model is VERIFIED in the Python edition
// (the COM port vanishes when the adapter is pulled; a power-up restarts the uptime, drops PMSS and brings the unit
// back at 38400); this edition is UNVERIFIED on hardware. Unofficial: not an IAI product. Header-only dependencies
// only: the simulator links on its own.
#include <algorithm>
#include <mutex>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

namespace {

/// Words of the controller serial at the start of the identification block (CTS1-4: 16 ASCII characters).
constexpr int kSerialWords = 8;

}  // namespace

// ================================================================================================ Impl helpers

void SimulatedUnit::Impl::check_handle() const {
    if (handle_lost) throw TransportError("simulated port vanished (USB unplugged): reopen it");
}

void SimulatedUnit::Impl::set_serial_number(const std::string& text) {
    std::string padded = text.substr(0, 2 * kSerialWords);
    padded.resize(2 * kSerialWords, ' ');
    const std::size_t offset = static_cast<std::size_t>(reg::kIdentSerialOffset);
    if (identification.size() < offset + kSerialWords) identification.resize(offset + kSerialWords, 0);
    for (int i = 0; i < kSerialWords; ++i) {
        const auto hi = static_cast<unsigned char>(padded[static_cast<std::size_t>(2 * i)]);
        const auto lo = static_cast<unsigned char>(padded[static_cast<std::size_t>(2 * i + 1)]);
        identification[offset + static_cast<std::size_t>(i)] = static_cast<std::uint16_t>((hi << 8) | lo);
    }
}

// ================================================================================================ public hooks

void SimulatedUnit::power_cycle(bool lose_home) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.log_event("power cycle");
    s.power_off();
    s.log_event("power on");
    s.restart(true);
    if (lose_home) {
        s.homed = false;
        s.log_event("HEND lost at power-up");
    }
}

void SimulatedUnit::power_off() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->power_off();
    impl_->powered = false;
}

void SimulatedUnit::power_on(std::optional<bool> lose_home) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.powered = true;
    s.log_event("power on");
    s.restart(true);
    if (lose_home.value_or(!s.absolute)) {
        s.homed = false;
        s.log_event("HEND lost at power-up");
    }
}

bool SimulatedUnit::powered() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->powered;
}

void SimulatedUnit::unplug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.log_event("port unplugged");
    s.present = false;
    s.handle_lost = true;
    s.out.clear();
}

void SimulatedUnit::replug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->log_event("port replugged");
    impl_->present = true;
}

std::optional<bool> SimulatedUnit::present() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->present;
}

void SimulatedUnit::reopen(std::optional<int> baud_rate) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    if (!s.present) throw TransportError("could not open simulated port: it is not present (unplugged)");
    s.handle_lost = false;
    s.closed = false;
    s.out.clear();
    s.reply_ready_at = 0.0;
    if (baud_rate && s.line_baud) {
        s.line_baud = *baud_rate;
        s.baud_history.push_back(*baud_rate);
    }
    ++s.reopens;
    s.log_event("port reopened");
}

int SimulatedUnit::reopens() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->reopens;
}

std::string SimulatedUnit::serial_number() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const Impl& s = *impl_;
    std::string text;
    const std::size_t offset = static_cast<std::size_t>(reg::kIdentSerialOffset);
    for (std::size_t i = offset; i < offset + kSerialWords && i < s.identification.size(); ++i) {
        for (int shift : {8, 0}) {
            const int b = (s.identification[i] >> shift) & 0xFF;
            if (b >= 0x20 && b <= 0x7E) text += static_cast<char>(b);
            else if (b == 0) text += ' ';
        }
    }
    const auto first = text.find_first_not_of(' ');
    if (first == std::string::npos) return std::string();
    return text.substr(first, text.find_last_not_of(' ') - first + 1);
}

}  // namespace iai_actuator_driver
