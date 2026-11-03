// SimulatedUnit: the rest of the Python edition's SimulatedEC test hooks and models (0.5): the EMG / drive-cut / enable
// inputs, the automatic servo OFF (STAT b17 ASOF), the cold boot with its slow first servo ON (DSSE PSNS), the teach
// coils MOD 0x0414 / TEAC 0x0415, IAI function 0x48, the function codes and broadcast a unit carries, and the settings
// and levels a test changes after the unit is made. Port of the Python edition's sim.py (set_inputs, _request_servo,
// _first_son_s, _wake_from_asof, _teach, _fc48 and the option attributes); modelled from the manuals there (DOC
// ME0162-12A p.4-20 / 4-21 / 5-97, DOC PCON p.4-24 / 5-5, ME0413 p.3-1 / 6-1), UNVERIFIED on an EC. Unofficial: not an
// IAI product. Header-only dependencies only.
#include <string>

#include "iai_actuator_driver/sim.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

using Bytes = std::vector<std::uint8_t>;

namespace {

constexpr int kFc48Reset = 0x23;  ///< alarm reset (op1 ignored)
constexpr int kFc48Servo = 0x30;  ///< op1 1 on / 0 off
constexpr int kFc48Home = 0x31;
constexpr int kFc48Stop = 0x34;   ///< decel stop
constexpr std::size_t kFc48Body = 19;  ///< group u8 | command u16 | op1..op4 u32

void put32(Bytes& b, std::uint32_t value) {
    for (int shift : {24, 16, 8, 0}) b.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFF));
}

}  // namespace

// ================================================================================================ Impl: settings

void SimulatedUnit::Impl::apply_options(const SimulatedUnitOptions& o) {
    // Every setting but the fixed facts (the constructor's init list sets those once).
    max_accel_g = o.max_accel_g;
    firmware = o.firmware.value_or(fx.firmware);
    dopm = o.dopm.value_or(fx.dopm);
    recompute_position_checksum = o.recompute_position_checksum;
    drop_reply_after_retarget = o.drop_reply_after_retarget;
    stop_latches = o.stop_latches;
    move_bit_clears_in_decel = o.move_bit_clears_in_decel;
    serve_soft_limits = o.soft_limits;
    ls_range_mm = o.ls_range_mm;
    early_in_position = o.early_in_position;
    settle_tail_mm = o.settle_tail_mm;
    settle_speed_mm_s = o.settle_speed_mm_s;
    push_blip_mm = o.push_blip_mm;
    push_blip_len_mm = o.push_blip_len_mm;
    slave_id_delay_s = o.slave_id_delay_s.value_or(fx.slave_id_delay_s);
    parw_busy_s = o.parw_busy_s;
    parw_dsse_busy = o.parw_dsse_busy;
    ignored_baud_codes = std::set<int>(o.ignored_baud_codes.begin(), o.ignored_baud_codes.end());
    boot_baud_rate = o.boot_baud_rate;
    reset_baud_rate = o.reset_baud_rate;
    reset_s = o.reset_s;
    auto_servo_off_s = o.auto_servo_off_s;
    cold_boot = o.cold_boot;
    ec_first_son_s = o.ec_first_son_s;
    function_codes.reset();
    if (o.function_codes) function_codes = std::set<int>(o.function_codes->begin(), o.function_codes->end());
    broadcast = o.broadcast.value_or(!o.function_codes);  // an RCON gateway line (function_codes) acts on none
    parw_latches = o.parw_latches;
    opt.move_patterns = o.move_patterns;
}

// ================================================================================================ Impl: servo and inputs

bool SimulatedUnit::Impl::son_blocked() const { return emg_input || drive_cut || enable_released; }

void SimulatedUnit::Impl::request_servo(bool on) {
    // A servo ON / OFF command (SON coil, DRG1 SON, 0x45 cmd 9, 0x48 cmd 0x30): ON is ignored while an input blocks it,
    // and the first ON after a cold boot completes only after the excitation-detection time.
    son_level = on;
    if (!on) {
        son_due.reset();
        set_servo(false);
        return;
    }
    if (son_blocked()) {
        log_event("servo on ignored (EMG / drive cut / enable switch)");
        return;
    }
    if (son_locked) {
        log_event("servo on ignored (DSSE b12 0 after the action mode PC; no alarm)");
        return;
    }
    if (!excited) {
        if (!son_due) {
            son_due = now() + first_son_s();
            log_event(util::format("first servo on: excitation detection %.3f s", first_son_s()));
        }
        return;
    }
    set_servo(true);
}

double SimulatedUnit::Impl::first_son_s() const {
    // CON: 0.006 + (T1 + No.29) x 10 + 0.020 s from item 9.1 (No.29 b0-15 ms, No.30 b17-18; T1 0.16 / 0.22 s; DOC PCON
    // p.4-24); EC: ec_first_son_s.
    if (system_code == reg::kSystemEc || system_code == reg::kSystemEc200) return ec_first_son_s;
    const auto address = item_address(0x0009, 1);
    const std::uint32_t raw = address ? reg::from_u32_words(read_word(*address), read_word(*address + 1)) : 0u;
    const double no29_ms = raw & 0xFFFFu;
    const unsigned no30 = (raw >> 17) & 0x3u;
    return 0.006 + ((no30 == 0 ? 0.16 : 0.22) + no29_ms / 1000.0) * 10.0 + 0.020;
}

void SimulatedUnit::Impl::wake_from_asof(const char* what) {
    // A move command while the automatic servo OFF holds the servo off re-energises it (DOC PCON p.5-5).
    if (asof && !servo_on && !son_blocked() && !son_locked && !alarm_major) {
        log_event(util::format("servo on (%s after automatic servo off)", what));
        set_servo(true);
    }
}

void SimulatedUnit::Impl::timers() {
    // The slow first servo ON and the automatic servo OFF fall due (advance(), at the time `t` just reached).
    if (son_due && t >= *son_due) {
        son_due.reset();
        if (!son_blocked()) set_servo(true, "excitation detection done");
    }
    if (asof_at && t >= *asof_at && !moving && servo_on) {
        asof_at.reset();
        set_servo(false, "automatic servo off");
    }
}

// ================================================================================================ Impl: teach, 0x48

void SimulatedUnit::Impl::teach() {
    // TEAC rising edge (DOC ME0162-12A p.5-97): with MOD on, PNOW into row POSR; alarm 0x093 before homing.
    if (!teach_mode) {
        log_event("TEAC ignored (MOD off)");
        return;
    }
    if (!homed) {
        raise_alarm(kSimAlarmTeachNotHomed);
        return;
    }
    const int n = posr;
    if (n < 0 || n >= sim_detail::kRowCount) {
        raise_alarm(kSimAlarmPositionNo);
        return;
    }
    const std::size_t row = static_cast<std::size_t>(n) * reg::kPositionStride;
    const auto p = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(pos * 100.0)));
    table[row] = p.first;
    table[row + 1] = p.second;
    table[row + reg::kPosChecksum] = position_checksum_of_row(n);
    ++nonvolatile_writes;
    teach_written = true;
    log_event(util::format("teach row %d = %.2f mm", n, pos));
}

Bytes SimulatedUnit::Impl::fc48(const Bytes& body) {
    // group u8 | command u16 | op1..op4 u32 (the group is ignored, as on 0x45); the answer: 0x48 | rejection u16 | four
    // zero u32.
    if (body.size() != kFc48Body) throw sim_detail::SimException{0x03};
    const int command = (body[1] << 8) | body[2];
    const int op1 = static_cast<int>((static_cast<std::uint32_t>(body[3]) << 24) | (body[4] << 16) | (body[5] << 8) |
                                     body[6]);
    int rejection;
    switch (command) {
        case kFc48Reset: rejection = iai_command(reg::kIaiCmdReset, reg::kIaiResetAlarm); break;
        case kFc48Servo: rejection = iai_command(reg::kIaiCmdServo, op1); break;
        case kFc48Home: rejection = iai_command(reg::kIaiCmdHome, op1); break;
        case kFc48Stop: rejection = iai_command(reg::kIaiCmdStop, op1); break;
        default: rejection = iai_command(-1, op1); break;
    }
    Bytes pdu = {0x48, static_cast<std::uint8_t>((rejection >> 8) & 0xFF), static_cast<std::uint8_t>(rejection & 0xFF)};
    for (int i = 0; i < 4; ++i) put32(pdu, 0);
    return pdu;
}

// ================================================================================================ SimulatedUnit: public

SimulatedUnitOptions SimulatedUnit::options() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const Impl& s = *impl_;
    SimulatedUnitOptions o = s.opt;  // the fixed facts as given
    o.max_accel_g = s.max_accel_g;
    o.firmware = s.firmware;
    o.dopm = s.dopm;
    o.recompute_position_checksum = s.recompute_position_checksum;
    o.drop_reply_after_retarget = s.drop_reply_after_retarget;
    o.stop_latches = s.stop_latches;
    o.move_bit_clears_in_decel = s.move_bit_clears_in_decel;
    o.soft_limits = s.serve_soft_limits;
    o.ls_range_mm = s.ls_range_mm;
    o.early_in_position = s.early_in_position;
    o.settle_tail_mm = s.settle_tail_mm;
    o.settle_speed_mm_s = s.settle_speed_mm_s;
    o.push_blip_mm = s.push_blip_mm;
    o.push_blip_len_mm = s.push_blip_len_mm;
    o.slave_id_delay_s = s.slave_id_delay_s;
    o.parw_busy_s = s.parw_busy_s;
    o.parw_dsse_busy = s.parw_dsse_busy;
    o.ignored_baud_codes.assign(s.ignored_baud_codes.begin(), s.ignored_baud_codes.end());
    o.boot_baud_rate = s.boot_baud_rate;
    o.reset_baud_rate = s.reset_baud_rate;
    o.reset_s = s.reset_s;
    o.auto_servo_off_s = s.auto_servo_off_s;
    o.cold_boot = s.cold_boot;
    o.ec_first_son_s = s.ec_first_son_s;
    o.function_codes.reset();
    if (s.function_codes) o.function_codes = std::vector<int>(s.function_codes->begin(), s.function_codes->end());
    o.broadcast = s.broadcast;
    o.parw_latches = s.parw_latches;
    return o;
}

void SimulatedUnit::set_options(const SimulatedUnitOptions& options) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    impl_->apply_options(options);
}

void SimulatedUnit::set_state(const SimStateLevels& levels) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.advance();
    if (levels.servo_on) s.servo_on = s.son_level = *levels.servo_on;
    if (levels.homed) s.homed = *levels.homed;
    if (levels.pmsl) s.pmsl = *levels.pmsl;
    if (levels.safety_speed) s.safety_speed = *levels.safety_speed;
}

void SimulatedUnit::set_inputs(std::optional<bool> emg, std::optional<bool> drive_cut, std::optional<bool> enable) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.advance();
    if (emg) s.emg_input = *emg;
    if (drive_cut) s.drive_cut = *drive_cut;
    if (enable) s.enable_released = !*enable;
    if (s.son_blocked() && s.servo_on) s.set_servo(false, "input");
}

bool SimulatedUnit::auto_servo_off_active() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->asof;
}

void SimulatedUnit::set_parameter_word(int address, std::uint16_t value, bool nv) {
    if (address < 0 || address > 0xFFFF)
        throw std::out_of_range("register address must be 0..0xFFFF, got " + std::to_string(address));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    (nv ? impl_->param_nv : impl_->param_ram)[address] = value;
}

double SimulatedUnit::ls_range_mm() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->ls_range_mm;
}

}  // namespace iai_actuator_driver
