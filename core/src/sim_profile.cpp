// SimulatedUnit: the command profile 0x0F00 (FC17 / FC10 writes read back as written, no motion until the PCRF coil
// 0x042E; separate acceleration and deceleration; CTLF b1 / b2 push, b3 incremental; a new profile + PCRF retargets;
// STOP stops it) and the acceleration pattern of the last move start (CTLF b6-b7). Port of the Python edition's sim.py
// (SimulatedEC._fc17, _active_profile, _run_profile, _set_accel_pattern): the command profile is VERIFIED there on the
// EC-GRBP8M gripper and the PCON-CB; the motion itself stays the simulator's trapezoid (an S-motion or filter move is
// recorded, not shaped). This edition is UNVERIFIED on hardware. Unofficial: not an IAI product.
#include <string>

#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

namespace {

using Bytes = std::vector<std::uint8_t>;
using Words = std::vector<std::uint16_t>;

int be16_at(const Bytes& b, std::size_t at) { return (b[at] << 8) | b[at + 1]; }

void put16(Bytes& b, int value) {
    b.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    b.push_back(static_cast<std::uint8_t>(value & 0xFF));
}

/// SPOW / DPOW of the block before any profile is written (live after a 15.00 mm/s, 0.05 G move: EC 114 / 255; the
/// PCON 0x59 / 0xFF; items 8.12 / 8.13).
std::pair<std::uint16_t, std::uint16_t> spow_dpow(int system_code) {
    if (system_code == reg::kSystemEc || system_code == reg::kSystemEc200) return {114, 255};
    return {0x59, 0xFF};
}

}  // namespace

// ================================================================================================ Impl

Bytes SimulatedUnit::Impl::fc17(const Bytes& body) {
    // read address u16 | read count u16 | write address u16 | write count u16 | bytes u8 | values
    if (body.size() < 9) throw sim_detail::SimException{0x03};
    const int r_addr = be16_at(body, 0);
    const int r_count = be16_at(body, 2);
    const int w_addr = be16_at(body, 4);
    const int w_count = be16_at(body, 6);
    const int nbytes = body[8];
    if (r_count < 1 || r_count > 125 || w_count < 1 || w_count > 121 || nbytes != 2 * w_count ||
        body.size() != static_cast<std::size_t>(9 + nbytes))
        throw sim_detail::SimException{0x03};
    Words values;
    for (int i = 0; i < w_count; ++i)
        values.push_back(static_cast<std::uint16_t>(be16_at(body, static_cast<std::size_t>(9 + 2 * i))));
    write_words(w_addr, values);  // the write goes first (Modbus FC17)
    advance();
    Bytes pdu = {0x17, static_cast<std::uint8_t>(2 * r_count)};
    for (int i = 0; i < r_count; ++i) put16(pdu, read_word(r_addr + i));
    return pdu;
}

std::uint16_t SimulatedUnit::Impl::active_profile(int offset) const {
    if (profile_words) return (*profile_words)[static_cast<std::size_t>(offset)];
    // What the last move applied (live after a 15.00 mm/s, 0.05 G move to 75.00: 75.00 / 15.00 / 5 / 5 / 0.10).
    const auto pcmd = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(target * 100.0)));
    const auto vcmd = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(vmax * 100.0)));
    const auto acmd = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(accel / reg::kGmms2 * 100.0)));
    const auto dcmd = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(decel / reg::kGmms2 * 100.0)));
    const auto inp = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(band * 100.0)));
    const auto power = spow_dpow(system_code);
    const std::uint16_t words[reg::kProfileWords] = {
        pcmd.first, pcmd.second, vcmd.first, vcmd.second, acmd.first, acmd.second, dcmd.first, dcmd.second,
        inp.first,  inp.second,  power.first, power.second, 0, 0, 0, 0, 0, 0, 0, 0};
    return words[offset];
}

void SimulatedUnit::Impl::write_profile(int address, const Words& values) {
    Words words(reg::kProfileWords);
    for (int i = 0; i < reg::kProfileWords; ++i) words[static_cast<std::size_t>(i)] = active_profile(i);
    for (std::size_t i = 0; i < values.size(); ++i)
        words[static_cast<std::size_t>(address - reg::kProfileBase) + i] = values[i];
    profile_words = words;  // accepted and read back; no motion until PCRF (VERIFIED on the EC-S6HH / PCON)
    log_event(util::format("profile write 0x%04x x%d", address, static_cast<int>(values.size())));
}

void SimulatedUnit::Impl::run_profile() {
    // PCRF coil: start the move the block describes (VERIFIED on the PCON: an FC17 write alone does not move; PCRF
    // then starts it; a retarget mid-move works; CTLF b3 = incremental).
    Words w(reg::kProfileWords);
    for (int i = 0; i < reg::kProfileWords; ++i) w[static_cast<std::size_t>(i)] = active_profile(i);
    const double pcmd = reg::from_i32_words(w[0], w[1]) / 100.0;
    const double speed = reg::from_u32_words(w[2], w[3]) / 100.0;
    const double accel_g = reg::from_u32_words(w[4], w[5]) / 100.0;
    const double decel_g = reg::from_u32_words(w[6], w[7]) / 100.0;
    const double band_mm = reg::from_u32_words(w[8], w[9]) / 100.0;
    const std::uint16_t ctlf = w[reg::kProfileCtlf];
    ++profile_moves;
    log_event("profile move (PCRF)");
    wake_from_asof("profile move");  // sim_controls.cpp
    row_move.reset();
    set_accel_pattern(ctlf);
    if (ctlf & reg::kCtlfPush) {
        start_push(pcmd, band_mm, speed, accel_g, (ctlf & reg::kCtlfPushAway) ? 1 : -1);
        return;
    }
    start_move((ctlf & reg::kCtlfInc) ? pcmd + pos : pcmd, band_mm, speed, accel_g, decel_g);
}

void SimulatedUnit::Impl::set_accel_pattern(std::uint16_t ctlf) {
    const std::uint16_t mod = static_cast<std::uint16_t>(ctlf & reg::kCtlfModMask);
    accel_pattern = mod == 0                      ? "trapezoid"
                    : mod == reg::kCtlfModSMotion ? "s_motion"
                    : mod == reg::kCtlfModFilter  ? "filter"
                                                  : "invalid";
    if (accel_pattern != "trapezoid")
        log_event(util::format("acceleration pattern %s (CTLF 0x%04x)", accel_pattern.c_str(),
                               static_cast<unsigned>(ctlf)));
}

// ================================================================================================ public

int SimulatedUnit::profile_moves() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->profile_moves;
}

std::string SimulatedUnit::accel_pattern() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->accel_pattern;
}

double SimulatedUnit::command_speed_mm_s() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->vmax;
}

double SimulatedUnit::push_speed_mm_s() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->push_speed;
}

}  // namespace iai_actuator_driver
