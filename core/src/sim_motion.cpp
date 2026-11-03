// SimulatedUnit: the motion model and the axis behaviour (servo, alarms, home, push, pause, stop, restart) plus the
// status words derived from them (DSS1 / DSSE / ZONS / DOPM / the 0x9000 monitor block). Port of the Python edition's
// sim.py (SimulatedEC); the hardware behaviour it models is VERIFIED in the Python edition where sim.hpp says so, this
// edition is UNVERIFIED on hardware. Unofficial: not an IAI product. Header-only dependencies only.
//
// Motion is a real-time trapezoid (speed VCMD, acceleration ACMD, deceleration DCMD) integrated in 1 ms steps of
// simulated time (clock x time_scale). A normal move covers its last settle_tail_mm slowly (the servo's settling
// tail) and PEND / MEND rise inside the INP band while the axis still moves; the MOVE bit clears during the final
// deceleration: a driver must also wait for PNOW to stop changing (VERIFIED in the Python edition).
#include <algorithm>
#include <cmath>
#include <cstring>

#include "iai_actuator_driver/sim.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

using sim_detail::HomingPhase;
using sim_detail::PushPhase;

namespace {

inline long long labs_diff(long long a, long long b) { return a > b ? a - b : b - a; }

}  // namespace

// ================================================================================================ time and integration

double SimulatedUnit::Impl::now() const { return (clock() - t0) * time_scale; }

void SimulatedUnit::Impl::advance() {
    const double current = now();
    double elapsed = current - t;
    t = current;
    timers();  // the slow first servo ON, the automatic servo OFF (sim_controls.cpp)
    if (!moving) {
        vel = 0.0;
        accelerating = decelerating = false;
        return;
    }
    while (elapsed > 0.0 && moving) {
        const double dt = std::min(sim_detail::kIntegrationStepS, elapsed);
        step(dt);
        elapsed -= dt;
        if (paused && vel == 0.0) break;  // held at rest: nothing changes until the pause is released
    }
}

void SimulatedUnit::Impl::step(double dt) {
    const double a = decel;  // the braking curve
    const double to_go = target - pos;
    const double d = std::fabs(to_go);
    const double tail = settle_tail_mm;
    const double creep = settle_speed_mm_s;
    const bool tail_applies = tail > 0.0 && creep > 0.0 && creep < vmax && push_phase == PushPhase::Off &&
                              homing_phase == HomingPhase::Off && !stopping;
    settling = tail_applies && d <= tail;
    double v_des;
    if (paused) {
        v_des = 0.0;
    } else if (tail_applies) {
        // Decelerate to the creep speed at the start of the tail, then creep (servo settling).
        const double v_lim = d <= tail ? std::min(creep, std::sqrt(2.0 * a * d))
                                       : std::sqrt(creep * creep + 2.0 * a * (d - tail));
        v_des = std::copysign(std::min(vmax, v_lim), to_go);
    } else {
        v_des = std::copysign(std::min(vmax, std::sqrt(2.0 * a * std::fabs(to_go))), to_go);
    }
    const bool speeding_up = std::fabs(v_des) > std::fabs(vel) && v_des * vel >= 0.0;
    const double a_step = (speeding_up ? accel : decel) * dt;
    const double dv = std::max(-a_step, std::min(a_step, v_des - vel));
    const double v_new = vel + dv;
    const double new_pos = pos + (vel + v_new) / 2.0 * dt;
    accelerating = std::fabs(v_new) > std::fabs(vel) + 1e-12;
    decelerating = std::fabs(v_new) < std::fabs(vel) - 1e-12;
    if (push_phase == PushPhase::Push && obstacle_mm) {
        const double lo = std::min(pos, new_pos);
        const double hi = std::max(pos, new_pos);
        if (lo <= *obstacle_mm && *obstacle_mm <= hi) {
            pos = target = *obstacle_mm;
            vel = 0.0;
            moving = false;
            accelerating = decelerating = false;
            push_phase = PushPhase::Off;
            push_contact = true;     // holds (keeps pushing) until the next command
            target_reached = true;   // live: DSS1 0x7099 after contact
            log_event(util::format("push contact at %.2f mm", pos));
            motion_ended(true);
            return;
        }
    }
    const bool crossed = (to_go > 0.0 && new_pos >= target) || (to_go < 0.0 && new_pos <= target);
    if (!paused && (crossed || std::fabs(to_go) < 1e-6)) {
        pos = target;
        arrive();
        return;
    }
    if (paused && crossed) {
        pos = target;  // a pause while braking late rests at the target, not past it
        vel = 0.0;
        accelerating = decelerating = false;
        return;
    }
    pos = new_pos;
    vel = v_new;
}

void SimulatedUnit::Impl::begin_motion(double target_mm, double speed, double accel_mm_s2, double band_mm,
                                       std::optional<double> decel_mm_s2) {
    decel = decel_mm_s2 ? *decel_mm_s2 : accel_mm_s2;
    target = target_mm;
    target_reached = false;
    stopping = false;
    settling = false;
    push_contact = false;
    push_missed = false;
    vmax = speed;
    accel = accel_mm_s2;
    band = band_mm;
    paused = stp_input;  // STP is a level input: a new move waits while it is ON
    moving = true;
    asof_at.reset();
}

void SimulatedUnit::Impl::next_phase(double target_mm, double speed) {
    target = target_mm;
    vmax = speed;
    moving = true;
}

void SimulatedUnit::Impl::arrive() {
    // The axis reached `target`: finish the move or start its next phase.
    vel = 0.0;
    moving = false;
    settling = false;
    accelerating = decelerating = false;
    if (homing_phase == HomingPhase::Seek) {
        homing_phase = HomingPhase::Creep;
        next_phase(0.0, sim_detail::kCreepSpeedMmS);
    } else if (homing_phase == HomingPhase::Creep) {
        homing_phase = HomingPhase::Off;
        homed = true;
        home_params_at_home = home_parameters();
        abs_home_lost = false;
        log_event("home complete");
    } else if (push_phase == PushPhase::Approach) {
        push_phase = PushPhase::Push;
        next_phase(push_end, push_speed);
    } else if (push_phase == PushPhase::Push) {
        push_phase = PushPhase::Off;
        push_missed = true;
        log_event("push missed");
        motion_ended(true);
    } else if (!stopping) {
        target_reached = true;  // a decel stop does not count as reaching the target
        motion_ended(true);
    } else {
        motion_ended(false);
    }
}

void SimulatedUnit::Impl::motion_ended(bool completed) {
    // A move / push ended: POSS takes a stored row that completed (contact or miss included), a 0x9900 move reads 0
    // (DOC ME0162-12A p.4-24), and the automatic servo OFF timer starts.
    if (completed && row_move)
        poss = *row_move;
    else if (completed)
        poss = 0;
    row_move.reset();
    if (auto_servo_off_s && servo_on) asof_at = now() + *auto_servo_off_s;
}

// ================================================================================================ commands

bool SimulatedUnit::Impl::blocked(const char* what) {
    if (stop_held()) {
        log_event(util::format("%s ignored (STOP held)", what));
        return true;
    }
    return false;
}

bool SimulatedUnit::Impl::check_move(double target_mm, double band_mm, double speed, double accel_g) {
    // Raise the alarm the unit would raise for this command; true if it may start.
    const double lo = soft_limits.min;
    const double hi = soft_limits.max;
    if (!servo_on)
        raise_alarm(kSimAlarmServoOffMove);
    else if (homing_phase != HomingPhase::Off)
        raise_alarm(kSimAlarmMoveWhileHoming);
    else if (!homed)
        raise_alarm(kSimAlarmNotHomedMove);
    else if (band_mm <= 0.0)
        raise_alarm(kSimAlarmBadBand);
    else if (!(speed > 0.0 && speed <= max_speed_mm_s))
        raise_alarm(kSimAlarmBadSpeed);
    else if (!(accel_g > 0.0 && accel_g <= max_accel_g))
        raise_alarm(kSimAlarmBadDecel);
    else if (!(lo - 1e-9 <= target_mm && target_mm <= hi + 1e-9))
        raise_alarm(kSimAlarmSoftLimit);
    else
        return true;
    return false;
}

void SimulatedUnit::Impl::start_move(double target_mm, double band_mm, double speed, double accel_g,
                                     std::optional<double> decel_g) {
    const double dg = (!decel_g || *decel_g <= 0.0) ? accel_g : *decel_g;
    if (blocked("move") || !check_move(target_mm, band_mm, speed, std::max(accel_g, dg))) return;
    log_event(util::format("move %.2f mm @ %.2f mm/s, %.2f / %.2f G", target_mm, speed, accel_g, dg));
    if (move_counter_address) {  // TLMC (maintenance item 1) counts move starts, retargets included
        const int a = *move_counter_address;
        const auto word = [this](int addr) {
            const auto it = statics.find(addr);
            return it == statics.end() ? std::uint16_t(0) : it->second;
        };
        const std::uint32_t count = reg::from_u32_words(word(a), word(a + 1)) + 1u;
        const auto w = reg::to_u32_words(count);
        statics[a] = w.first;
        statics[a + 1] = w.second;
    }
    push_phase = PushPhase::Off;
    begin_motion(target_mm, speed, accel_g * reg::kGmms2, band_mm, dg * reg::kGmms2);
}

void SimulatedUnit::Impl::start_push(double target_mm, double band_mm, double speed, double accel_g, int direction) {
    if (blocked("push")) return;
    const double end = target_mm + direction * band_mm;
    if (!check_move(target_mm, band_mm, speed, accel_g) || !check_move(end, band_mm, speed, accel_g)) return;
    log_event(util::format("push %c from %.2f over %.2f mm", direction > 0 ? '+' : '-', target_mm, band_mm));
    if (move_counter_address) {
        const int a = *move_counter_address;
        const auto word = [this](int addr) {
            const auto it = statics.find(addr);
            return it == statics.end() ? std::uint16_t(0) : it->second;
        };
        const auto w = reg::to_u32_words(reg::from_u32_words(word(a), word(a + 1)) + 1u);
        statics[a] = w.first;
        statics[a + 1] = w.second;
    }
    push_end = end;
    push_dir = direction;
    push_speed = std::min(speed, unit_push_speed());
    if ((pos - target_mm) * direction >= 0.0) {
        // Hardware: from inside / past the push zone the unit skips the approach and pushes from here.
        if ((pos - end) * direction >= 0.0) push_end = pos + direction * band_mm;
        push_phase = PushPhase::Push;
        begin_motion(push_end, push_speed, accel_g * reg::kGmms2, band_mm);
    } else {
        push_phase = PushPhase::Approach;
        begin_motion(target_mm, speed, accel_g * reg::kGmms2, band_mm);
    }
}

double SimulatedUnit::Impl::unit_push_speed() const {
    // min(PSHV item 8.15, MXPH item 7.8) as this unit serves them (fallback 20 mm/s).
    std::optional<double> best;
    const std::pair<int, int> keys[2] = {{reg::kMotionParamsCategory, reg::kParamItemPshv},
                                         {reg::kLimitParamsCategory, reg::kAxisItemMxph}};
    for (const auto& key : keys) {
        const auto address = item_address(key.first, key.second);
        if (!address) continue;
        std::uint32_t raw;
        try {
            raw = reg::from_u32_words(read_word(*address), read_word(*address + 1));
        } catch (const sim_detail::SimException&) {
            continue;
        }
        if (raw > 0 && raw < 10000000u) {
            const double speed = raw / 100.0;
            if (!best || speed < *best) best = speed;
        }
    }
    return best ? *best : sim_detail::kPushSpeedFallbackMmS;
}

void SimulatedUnit::Impl::start_home() {
    if (!servo_on) {
        log_event("home ignored (servo off)");
        return;
    }
    if (blocked("home")) return;
    log_event("home");
    homed = false;
    push_phase = PushPhase::Off;
    homing_phase = HomingPhase::Seek;
    begin_motion(sim_detail::kHomingEndMm, sim_detail::kHomingSpeedMmS, 0.3 * reg::kGmms2, 0.10);
}

void SimulatedUnit::Impl::decel_stop() {
    log_event("stop");
    push_contact = false;  // a STOP ends the push hold (simplification)
    if (!moving) return;
    target_reached = false;
    stopping = true;
    push_phase = PushPhase::Off;
    homing_phase = HomingPhase::Off;  // home stays incomplete
    if (paused || vel == 0.0) {
        moving = false;
        paused = false;
        target = pos;
        return;
    }
    double stop_distance = vel * vel / (2.0 * decel);
    const double to_go = target - pos;
    if (to_go * vel > 0.0) stop_distance = std::min(stop_distance, std::fabs(to_go));  // braking late: ends at the target
    target = pos + std::copysign(stop_distance, vel);
}

void SimulatedUnit::Impl::move_to_row(int n, double speed_scale) {
    if (n < 0 || n >= sim_detail::kRowCount) {
        raise_alarm(kSimAlarmPositionNo);
        return;
    }
    const Words row = row_words(n);
    std::string text = util::format("move to position %d", n);
    if (speed_scale != 1.0) text += util::format(" at %.2f %%", speed_scale * 100.0);
    log_event(text);
    wake_from_asof("row move");
    const double band_mm = reg::from_u32_words(row[2], row[3]) / 100.0;
    const double speed = reg::from_u32_words(row[4], row[5]) / 100.0 * speed_scale;
    const double accel_g = row[10] / 100.0;
    const double target_mm = reg::from_i32_words(row[0], row[1]) / 100.0;
    if (row[reg::kPosCtlf] & reg::kCtlfPush) {  // a push row (gripper row 1, test G2: approach 4.10, push to 9.50)
        start_push(target_mm, band_mm, speed, accel_g, (row[reg::kPosCtlf] & reg::kCtlfPushAway) ? 1 : -1);
    } else {
        start_move(target_mm, band_mm, speed, accel_g, row[11] / 100.0);
    }
    if (moving) row_move = n;  // POSS reports it once the move (or push) ends
}

void SimulatedUnit::Impl::set_servo(bool on, const char* reason) {
    if (on && alarm_major) return;  // servo on refused while a major alarm is active
    if (!on && moving) {
        // Simplification: the real unit decelerates / brakes; here the axis just stops.
        vel = 0.0;
        moving = false;
        paused = false;
        if (homing_phase != HomingPhase::Off) {
            homing_phase = HomingPhase::Off;
            homed = false;
        }
        target = pos;
    }
    if (!on) {
        target_reached = false;
        push_phase = PushPhase::Off;
        push_contact = false;
        asof = reason && std::strcmp(reason, "automatic servo off") == 0;
        asof_at.reset();
    } else {
        asof = false;
        asof_at.reset();
        son_due.reset();
        excited = true;
    }
    servo_on = on;
    std::string text = on ? "servo on" : "servo off";
    if (reason && *reason) text += std::string(" (") + reason + ")";
    log_event(text);
}

void SimulatedUnit::Impl::set_pause(bool on) {
    stp_input = on;  // a level input: also holds moves commanded while it is ON
    paused = on && moving;
    log_event(on ? "pause" : "resume");
}

void SimulatedUnit::Impl::control_switched(bool pc) {
    // Nothing on the PIO connector (pio_unwired): in PIO mode the unwired *STP input reads as a pause and the servo goes
    // off; Modbus control back clears the pause, the servo stays off (VERIFIED on the PCON-CB, 2026-11-02).
    if (!pio_unwired) return;
    if (pc) {
        if (pio_pause) log_event("pause of the PIO *STP input released (Modbus control)");
        pio_pause = false;
        return;
    }
    pio_pause = true;
    log_event("PIO mode with nothing on the PIO connector: pause (*STP)");
    if (servo_on) set_servo(false, "PIO mode: *SON not wired");
}

void SimulatedUnit::Impl::raise_alarm(int code, std::optional<bool> major) {
    alarm_code = code;
    // alarm levels: a message (named codes of 0x011-0x07F) keeps the axis running, everything else stops it
    alarm_major = major ? *major : !sim_detail::is_message_alarm(code);
    log_event(util::format("alarm 0x%03x", code));
    const auto stamp = reg::to_u32_words(stim_ms());
    alarm_history.insert(alarm_history.begin(),
                         std::array<std::uint16_t, 6>{0, 0xFFFF, 0, static_cast<std::uint16_t>(code), stamp.first, stamp.second});
    alarm_history.resize(static_cast<std::size_t>(sim_detail::kHistoryEntries));
    if (alarm_major) set_servo(false, "alarm");
}

void SimulatedUnit::Impl::alarm_reset() {
    log_event("alarm reset");
    if (stp_input && moving) {
        // DOC ME0384 p.6-76; ME0162-12A p.4-11, 5-85: an ALRS edge while paused cancels the remaining travel.
        log_event("paused move cancelled (ALRS while paused)");
        decel_stop();
        row_move.reset();
    }
    const bool abs_error = alarm_code == kSimAlarmAbsError;
    const bool was_major = alarm_major;
    alarm_code = 0;
    alarm_major = false;
    if (abs_error) {
        servo_on = true;  // gripper recovery: after the ALRS pulse the servo came back on by itself (HEND 0)
        log_event("servo on (after ABS error reset)");
    } else if (was_major && alarm_reset_restores_servo && son_level && !servo_on && pmsl && !son_blocked() &&
               !son_locked) {
        set_servo(true);  // PCON-CB: the servo-on level was still set (ON-WIRE, 2026-11-03)
        log_event("servo on (the servo-on level was still set at the alarm reset)");
    }
    if (rtc_unset) raise_alarm(reg::kAlarmRtcStop);  // PCON: re-raised by itself while the clock is unset (message level)
}

void SimulatedUnit::Impl::software_reset_command() {
    // 0x45 cmd 5 op1 1 (sent after servo OFF by the IAI PC software, ON-WIRE) or the SFRS coil 0x0420 (test R1).
    if (servo_on) {
        log_event("software reset refused: servo on");
        raise_alarm(kSimAlarmResetServoOn, true);  // R1: alarm 0x090, servo off, no reset
        return;
    }
    ++software_resets;
    log_event("software reset");
    restart(false);
}

void SimulatedUnit::Impl::power_off() {
    advance();
    log_event("power off");
    out.clear();
    reply_ready_at = 0.0;
    vel = 0.0;
    moving = paused = false;
    homing_phase = HomingPhase::Off;
    push_phase = PushPhase::Off;
    settling = accelerating = decelerating = stopping = false;
    target = pos;
}

void SimulatedUnit::Impl::power_on() {
    log_event("power on");
    restart(true);
    if (!absolute) {
        homed = false;
        log_event("HEND lost at power-up");
    }
}

void SimulatedUnit::Impl::restart(bool power_up) {
    // Power-up or software reset: everything volatile starts afresh.
    parw_tick();
    if (parw_until) {  // worst case (unknown on the EC): a cut-off commit is lost
        log_event("parameter commit (PARW) interrupted by a restart: lost");
        parw_until.reset();
        parw_snapshot.clear();
    }
    param_ram.clear();  // N3: parameter writes were RAM only until PARW
    apply_parameters();
    out.clear();
    drop_next_reply = false;
    coils.clear();
    pio_start.reset();
    vel = 0.0;
    moving = paused = stp_input = pio_pause = false;
    homing_phase = HomingPhase::Off;
    push_phase = PushPhase::Off;
    settling = accelerating = decelerating = stopping = false;
    push_contact = push_missed = false;
    target = pos;
    target_reached = false;  // live after boot: DSS1 0x3098 (b0 = 0)
    drg1_other = drg2 = posr = posr_imm = 0;
    alarm_code = 0;
    alarm_major = false;
    servo_on = true;  // boot: servo on; after a software reset DSS1 read 0x7088 (servo on, HEND 0; ON-WIRE)
    // test 17: PMSS = 0 after boot. A software reset dropped it to 0 on the EC-R6S rod; the EC-GRBP8M gripper and the
    // PCON-CB keep PMSS 1 (reset_keeps_pmss; VERIFIED in the Python edition)
    pmsl = pmsl && (con_reset || fx.reset_keeps_pmss) && !power_up;
    safety_speed = false;  // test 17: SFTY 0
    asof = false;
    asof_at.reset();
    son_due.reset();
    son_locked = false;  // DSSE b12 set again by a restart: UNVERIFIED (never tried on the PCON-CB)
    teach_mode = teach_written = false;
    row_move.reset();
    profile_words.reset();  // the command profile is volatile (sim_profile.cpp)
    poss = 0;
    if (power_up && cold_boot) {
        servo_on = false;  // CON after power-up: servo off, PSNS 0 until the first servo ON
        excited = false;
    }
    son_level = servo_on;
    if (power_up) {
        homed = true;
        alarm_history.insert(alarm_history.begin(),
                             std::array<std::uint16_t, 6>{0, 0xFFFF, 0, static_cast<std::uint16_t>(reg::kAlarmPowerUp), 0xFFFF, 0xFFFF});
        alarm_history.resize(static_cast<std::size_t>(sim_detail::kHistoryEntries));
        t0 = clock();
        t = now();
        busy_until = -1.0;
        if (ec_baud) ec_baud = boot_baud_rate;  // test 17: 38400 after power-up
    } else {
        if (!absolute) homed = false;  // the EC-R6S rod (incremental) lost HEND at a software reset
        if (reset_baud_rate && ec_baud && !con_reset) ec_baud = reset_baud_rate;
        busy_until = now() + reset_s;  // uptime (0x9010) keeps counting
    }
    if (absolute && (abs_home_lost || home_parameters() != home_params_at_home)) {
        // Gripper 2026-10-27: committed P4 change + software reset -> 0x0EE, HEND 0, servo off; a second reset with P4
        // restored did not clear it (home stays "not established").
        abs_home_lost = true;
        homed = false;
        raise_alarm(kSimAlarmAbsError, true);
    }
    if (rtc_unset) raise_alarm(reg::kAlarmRtcStop);  // the PCON's clock is still unset after a restart
}

// ================================================================================================ derived status words

std::vector<std::uint16_t> SimulatedUnit::Impl::row_words(int n) const {
    const auto first = table.begin() + static_cast<std::ptrdiff_t>(n) * reg::kPositionStride;
    return Words(first, first + reg::kPositionStride);
}

std::pair<double, bool> SimulatedUnit::Impl::row_marker(int n) const {
    // (LS position, is a push row) of position row n: its position, or a push row's push end.
    const Words row = row_words(n);
    const double stored = reg::from_i32_words(row[0], row[1]) / 100.0;
    if (row[reg::kPosCtlf] & reg::kCtlfPush) {
        const double band_mm = reg::from_u32_words(row[2], row[3]) / 100.0;
        return {stored + ((row[reg::kPosCtlf] & reg::kCtlfPushAway) ? band_mm : -band_mm), true};
    }
    return {stored, false};
}

bool SimulatedUnit::Impl::at_row(int n) const {
    // PNOW within the LS range (EC parameter 2) of row n's marker, moving or not. Compared in 0.01 mm counts,
    // inclusive: the slider lit LS1 / LS0 at 150.10 / -0.10 mm and the gripper LS0 at 2.91 mm while moving.
    const double marker = row_marker(n).first;
    return labs_diff(sim_detail::py_round(pos * 100.0), sim_detail::py_round(marker * 100.0)) <=
           sim_detail::py_round(ls_range_mm * 100.0);
}

std::uint16_t SimulatedUnit::Impl::zons() const {
    // b0 / b1 inside zones 1 / 2 (both span the soft limits), b12-b14 position markers LS0-LS2 (the LS bits follow the
    // position of all three rows, also on the 2-position gripper: 0x4003 at 2.52 mm with row 2 = 2.50 mm).
    std::uint16_t value = (soft_limits.min - 1e-9 <= pos && pos <= soft_limits.max + 1e-9)
                              ? static_cast<std::uint16_t>(sim_detail::kZonsZ1 | sim_detail::kZonsZ2)
                              : std::uint16_t(0);
    if (at_row(0)) value |= reg::kZonsLs0;
    if (at_row(1)) value |= reg::kZonsLs1;
    if (at_row(2)) value |= reg::kZonsLs2;
    return value;
}

std::uint16_t SimulatedUnit::Impl::dopm_word() const {
    // PIO output image in the profile's layout: LS0 b0, LS1 b1, then LS2 b2 + *ALM b3 (3-position) or *ALM b2
    // (2-position). *ALM = 1 means no alarm. A push row's bit stays off after a missed push (gripper, G2: DOPM 0x0004
    // at 9.50 mm with PSFL; 0x0006 there after power-up).
    const bool three = dopm == DopmLayout::ThreePosition;
    std::uint16_t value = alarm_code ? std::uint16_t(0)
                                     : (three ? sim_detail::kDopm3PosNoAlarm : sim_detail::kDopm2PosNoAlarm);
    const std::pair<int, std::uint16_t> markers[3] = {{0, sim_detail::kDopmLs0}, {1, sim_detail::kDopmLs1},
                                                      {2, sim_detail::kDopm3PosLs2}};
    for (int i = 0; i < (three ? 3 : 2); ++i) {
        const int n = markers[i].first;
        if (at_row(n) && !(push_missed && row_marker(n).second)) value |= markers[i].second;
    }
    return value;
}

bool SimulatedUnit::Impl::early_band() const {
    // Still moving, but already inside the INP band of a normal move or a decel stop (seen live).
    return early_in_position && moving && !paused && push_phase == PushPhase::Off && homing_phase == HomingPhase::Off &&
           std::fabs(pos - target) <= band + 1e-9;
}

bool SimulatedUnit::Impl::push_blip() const {
    // Inside the push_blip_mm window of a push stroke (hardware test G5).
    if (!push_blip_mm || push_phase != PushPhase::Push || !moving) return false;
    const double along = (pos - *push_blip_mm) * push_dir;
    return -1e-9 <= along && along <= push_blip_len_mm + 1e-9;
}

std::uint16_t SimulatedUnit::Impl::dss1() const {
    std::uint16_t value = reg::kDss1Pwr;
    if (emg_input) value |= reg::kDss1Emgs;
    if (safety_speed) value |= reg::kDss1Sfty;
    if (servo_on) {
        value |= reg::kDss1Sv;
        if (!moving) value |= reg::kDss1Bkrl;  // live: b7 = 1 when stopped, 0 while moving
    }
    if (alarm_code) value |= alarm_major ? reg::kDss1Almh : reg::kDss1Alml;
    if (stp_input || pio_pause) value |= reg::kDss1Stp;
    if (homed) value |= reg::kDss1Hend;
    const bool early = early_band();  // inside the band, still moving (PEND / MEND early, as seen live)
    const bool blip = push_blip();    // hardware test G5: contact fingerprint for a moment during a push
    if (push_missed) {
        value |= reg::kDss1Psfl;
    } else if (servo_on && (push_contact || blip || early || (!moving && std::fabs(pos - target) <= band + 1e-9))) {
        value |= reg::kDss1Pend;
    }
    if (servo_on && (target_reached || blip || (early && !stopping))) value |= reg::kDss1TargetReached;
    return value;
}

bool SimulatedUnit::Impl::move_bit() const {
    if (!moving || homing_phase == HomingPhase::Creep || settling)
        return false;  // live: MOVE is 0 during the homing creep and the final settling (0x33C2 at 3.04 mm)
    const double braking = vmax * vmax / (2.0 * decel) + std::max(settle_tail_mm, 0.0);
    if (early_band() && std::fabs(target - pos) <= braking + 1e-9)
        return false;  // in the band and in the final deceleration (live: MOVE clears 0.2-1.2 mm before standstill)
    if (paused) return std::fabs(vel) > 0.0;  // 0 once the pause has stopped the axis
    return !(move_bit_clears_in_decel && decelerating);
}

std::uint16_t SimulatedUnit::Impl::dsse() const {
    const std::uint16_t dynamic = static_cast<std::uint16_t>(reg::kDssePmss | reg::kDsseMove | sim_detail::kDsseDst1 |
                                                             reg::kDsseGhms | sim_detail::kDsseSson | reg::kDssePush |
                                                             reg::kDssePsns | reg::kDssePushContact);
    std::uint16_t value = static_cast<std::uint16_t>(sim_detail::kDsseIdle & ~dynamic);  // live idle value, dynamic bits out
    const bool blip = push_blip();
    if (pmsl) value |= reg::kDssePmss;
    if (move_bit() && !blip) value |= static_cast<std::uint16_t>(reg::kDsseMove | sim_detail::kDsseDst1);  // live: 0x33F2 moving
    if (homing_phase != HomingPhase::Off) value |= reg::kDsseGhms;
    // live: 0x39C2 during the homing creep; 0 before a cold boot's first servo on
    if (homing_phase != HomingPhase::Creep && excited) value |= reg::kDssePsns;
    if (emg_input) value |= reg::kDsseEmgp;
    if (drive_cut) value |= reg::kDsseMpuv;
    if (action_mode_locks_son ? !son_locked : servo_on) value |= sim_detail::kDsseSson;  // PCON-CB: 1 unless locked
    if (push_phase == PushPhase::Push || push_contact) value |= reg::kDssePush;
    if (push_contact || blip) value |= reg::kDssePushContact;  // live: 0x37C3 after contact
    if (parw_dsse_busy && parw_until) value = static_cast<std::uint16_t>(value & ~reg::kDsseParw);
    return value;
}

std::uint16_t SimulatedUnit::Impl::dss2() const {
    // b15 ENBS (the enable switch; live 0x8000), b11 MODS (teach mode), b10 PWRT (teach write done)
    return static_cast<std::uint16_t>((enable_released ? 0 : reg::kDss2Enbs) | (teach_mode ? reg::kDss2Mods : 0) |
                                      (teach_written ? reg::kDss2Pwrt : 0));
}

std::uint16_t SimulatedUnit::Impl::ssse_word() const {
    // The profile's word, b11 while a cold-start-level alarm is present (DOC ME0162-12A p.4-25).
    const bool cold = alarm_code != 0 && sim_detail::is_cold_start_alarm(alarm_code);
    return static_cast<std::uint16_t>(ssse | (cold ? reg::kSsseAlmc : 0));
}

std::uint16_t SimulatedUnit::Impl::drg1() const {
    return static_cast<std::uint16_t>(drg1_other | (servo_on ? sim_detail::kDrg1Son : 0) |
                                      (safety_speed ? sim_detail::kDrg1Sfty : 0) | (stp_input ? sim_detail::kDrg1Stp : 0));
}

int SimulatedUnit::Impl::current_ma() const {
    if (!servo_on) return 0;
    if (push_contact) return 464;  // live plateau at 30 %
    if (moving && accelerating) return 450;
    return moving ? 250 : 120;
}

std::uint32_t SimulatedUnit::Impl::stim_ms() const {
    return static_cast<std::uint32_t>(static_cast<long long>(now() * 1000.0) & 0xFFFFFFFFLL);
}

std::uint16_t SimulatedUnit::Impl::status_image(int offset) const {
    // Status image 0x0200 (class 2, item ids 1-9 + 16): DSS1, DSS2, DSSE, POSS, ZONS, DIPM, DOPM, SIPM, SSSE, then
    // 0x0001 (live slider: 7099 8000 33C2 0000 4003 0000 000C 0300 1008 0001).
    const std::uint16_t image[10] = {dss1(), dss2(), dsse(), static_cast<std::uint16_t>(poss), zons(), 0, dopm_word(),
                                     0x0300, ssse_word(), 0x0001};
    return image[offset];
}

std::uint16_t SimulatedUnit::Impl::monitor_word(int address) const {
    const auto pnow = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(pos * 100.0)));
    const double steps = static_cast<double>(sim_detail::py_round(std::fabs(vel) / sim_detail::kVnowStep));  // coarse on the EC
    const auto vnow = reg::to_i32_words(
        static_cast<std::int32_t>(sim_detail::py_round(std::copysign(steps * sim_detail::kVnowStep, vel) * 100.0)));
    const auto cnow = reg::to_i32_words(current_ma());
    // live idle: 0x0000801F / 0x0008801F (MPOW, SON, SV, HEND, RMDS, b15; DOC ME0162-12A p.4-21)
    const std::uint32_t stat_value = (drive_cut ? 0u : reg::kStatMpow) | (servo_on || asof ? sim_detail::kStatSon : 0u) |
                                     (servo_on ? sim_detail::kStatSv : 0u) | (homed ? sim_detail::kStatHend : 0u) |
                                     sim_detail::kStatRmds | 0x8000u | (parw_until ? reg::kStatAeep : 0u) |
                                     (asof ? reg::kStatAsof : 0u);
    const auto stat = reg::to_u32_words(stat_value);
    const auto stim = reg::to_u32_words(stim_ms());
    const auto ollv_words = reg::to_u32_words(static_cast<std::uint32_t>(ollv));
    switch (address) {
        case 0x9000: return pnow.first;
        case 0x9001: return pnow.second;
        case 0x9002: return static_cast<std::uint16_t>(alarm_code);
        case 0x9003: return 0;
        case 0x9004: return dopm_word();
        case 0x9005: return dss1();
        case 0x9006: return dss2();
        case 0x9007: return dsse();
        case 0x9008: return stat.first;
        case 0x9009: return stat.second;
        case 0x900A: return vnow.first;
        case 0x900B: return vnow.second;
        case 0x900C: return cnow.first;
        case 0x900D: return cnow.second;
        case 0x9010: return stim.first;
        case 0x9011: return stim.second;
        case 0x9012: return 0x0300;
        case 0x9013: return zons();
        case 0x9014: return static_cast<std::uint16_t>(poss);
        case 0x9015: return ssse_word();
        case 0x9020: return ollv_words.first;
        case 0x9021: return ollv_words.second;
        default: return 0;
    }
}

}  // namespace iai_actuator_driver
