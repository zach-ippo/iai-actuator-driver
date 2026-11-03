// Decoded monitor block (0x9000...): Status::from_registers and the derived flags. Port of the Python edition's
// ec.Status (C# Status.cs). No I/O.
#include "iai_actuator_driver/status.hpp"

#include <cmath>
#include <stdexcept>

#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

bool bit(std::uint32_t value, std::uint32_t mask) { return (value & mask) != 0; }

}  // namespace

Status Status::from_registers(const std::vector<std::uint16_t>& words) {
    if (words.size() < static_cast<std::size_t>(reg::kStatusMinWords))
        throw std::invalid_argument("need at least " + std::to_string(reg::kStatusMinWords) + " words");
    const int alarm_code = words[2];
    const std::uint16_t dss1 = words[5];
    const std::uint16_t dss2 = words[6];
    const std::uint16_t dsse = words[7];

    StatusRaw raw;
    raw.words = reg::kStatusMinWords;
    raw.pnow = reg::from_u32_words(words[0], words[1]);
    raw.almc = words[2];
    raw.dipm = words[3];
    raw.dopm = words[4];
    raw.dss1 = dss1;
    raw.dss2 = dss2;
    raw.dsse = dsse;

    std::optional<int> current;
    std::optional<double> vnow;
    if (words.size() >= static_cast<std::size_t>(reg::kStatusShortWords)) {
        raw.words = reg::kStatusShortWords;
        raw.stat = reg::from_u32_words(words[8], words[9]);
        raw.vnow = reg::from_u32_words(words[10], words[11]);
        raw.cnow = reg::from_u32_words(words[12], words[13]);
        vnow = reg::from_counts(reg::from_i32_words(words[10], words[11]), reg::kVelocityScale);
        current = reg::from_i32_words(words[12], words[13]);
    }
    if (words.size() >= static_cast<std::size_t>(reg::kStatusBlockWords)) {
        raw.words = reg::kStatusBlockWords;
        raw.devi = reg::from_u32_words(words[14], words[15]);
        raw.stim = reg::from_u32_words(words[16], words[17]);
        raw.sipm = words[18];
        raw.zons = words[19];
    }
    if (words.size() >= static_cast<std::size_t>(reg::kStatusReadWords)) {
        raw.words = reg::kStatusReadWords;
        raw.poss = words[20];
        raw.ssse = words[21];
    }

    // Fault rule (alarm_is_fault): ALMH, or a code of level operation cancelled / cold start. A message-level code is a
    // warning - the axis keeps running - and never blocks motion. An unknown-level code without the warning bit ALML is
    // a fault (fail safe).
    const bool alarm = alarm_is_fault(alarm_code, bit(dss1, reg::kDss1Almh), bit(dss1, reg::kDss1Alml));
    const bool servo_on = bit(dss1, reg::kDss1Sv);
    // "Settled" states in which PEND legitimately stays 0 fall back to the raw bits. A warning does not settle the axis:
    // it keeps running.
    const bool settled = !servo_on || bit(dss1, reg::kDss1Pend | reg::kDss1Psfl | reg::kDss1Stp) || alarm ||
                         !bit(dss1, reg::kDss1Hend);
    const bool moving = bit(dsse, reg::kDsseMove | reg::kDsseGhms) || !settled;
    if (!moving) vnow = 0.0;

    Status st;
    st.position_mm = reg::from_counts(reg::from_i32_words(words[0], words[1]), reg::kPositionScale);
    st.alarm_code = alarm_code;
    st.servo_on = servo_on;
    st.homed = bit(dss1, reg::kDss1Hend);
    st.in_position = bit(dss1, reg::kDss1Pend);
    st.moving = moving;
    st.alarm = alarm;
    st.modbus_enabled = bit(dsse, reg::kDssePmss);
    st.emergency_stop = bit(dss1, reg::kDss1Emgs);
    st.velocity_mm_s = vnow;
    st.current_ma = current;
    st.raw = raw;
    return st;
}

// ------------------------------------------------------------------------------------------ derived flags

bool Status::warning() const {
    if (alarm) return false;
    return bit(raw.dss1, reg::kDss1Alml) ||
           (alarm_code != 0 && iai_actuator_driver::alarm_level(alarm_code) == AlarmLevel::Message);
}

std::optional<AlarmLevel> Status::alarm_level() const {
    if (alarm_code == 0) return std::nullopt;
    return iai_actuator_driver::alarm_level(alarm_code);
}

bool Status::move_bit() const { return bit(raw.dsse, reg::kDsseMove); }
bool Status::homing() const { return bit(raw.dsse, reg::kDsseGhms); }
bool Status::paused() const { return bit(raw.dss1, reg::kDss1Stp); }
bool Status::target_reached() const { return bit(raw.dss1, reg::kDss1TargetReached); }
bool Status::push_missed() const { return bit(raw.dss1, reg::kDss1Psfl); }
bool Status::pushing() const { return bit(raw.dsse, reg::kDssePush); }
bool Status::push_contact() const {
    // behind an RCON gateway DSSE b0 is not defined (DOC ME0413 p.5-15): PEND with DSSE PUSH and no PSFL (UNVERIFIED)
    if (rcon()) return in_position && bit(raw.dsse, reg::kDssePush) && !push_missed();
    return in_position && bit(raw.dsse, reg::kDssePushContact);
}
bool Status::safety_speed() const { return bit(raw.dss1, reg::kDss1Sfty); }

std::optional<bool> Status::at_backward_end() const {
    if (!raw.zons || rcon()) return std::nullopt;  // LS0-LS2 undefined on RCON (DOC ME0413 p.5-15 to 5-19)
    return bit(*raw.zons, reg::kZonsLs0);
}
std::optional<bool> Status::at_forward_end() const {
    if (!raw.zons || rcon()) return std::nullopt;
    return bit(*raw.zons, reg::kZonsLs1);
}
std::optional<bool> Status::at_middle() const {
    if (!raw.zons || rcon()) return std::nullopt;
    return bit(*raw.zons, reg::kZonsLs2);
}

std::optional<double> Status::vnow_mm_s() const {
    if (!raw.vnow) return std::nullopt;
    return reg::from_counts(static_cast<std::int32_t>(*raw.vnow), reg::kVelocityScale);
}

std::optional<double> Status::uptime_s() const {
    if (!raw.stim) return std::nullopt;
    return std::round(static_cast<double>(*raw.stim)) / 1000.0;  // ms since power-on; whole ms, so 3 decimals exact
}

bool Status::ready() const {
    return servo_on && homed && modbus_enabled && !alarm && !emergency_stop && !paused() && !homing();
}

// ------------------------------------------------------------------------------------------ condition flags

std::optional<int> Status::position_complete_no() const {
    if (!raw.poss) return std::nullopt;
    return static_cast<int>(*raw.poss & (rcon() ? reg::kPossMaskRcon : reg::kPossMask));  // 7 bits on RCON (ME0413 p.5-10)
}

std::optional<bool> Status::cold_start_alarm() const {
    if (!raw.ssse) return std::nullopt;
    return bit(*raw.ssse, reg::kSsseAlmc);
}

std::optional<bool> Status::rtc_in_use() const {
    if (!raw.ssse) return std::nullopt;
    return bit(*raw.ssse, reg::kSsseRtc);
}

std::optional<bool> Status::drive_power_cut() const {
    // DSSE is always read; STAT only with 14 or more words
    const bool mpuv = bit(raw.dsse, reg::kDsseMpuv);
    const bool mpow_off = raw.stat.has_value() && !bit(*raw.stat, reg::kStatMpow);
    return mpuv || mpow_off;
}

std::optional<bool> Status::emg_input() const { return bit(raw.dsse, reg::kDsseEmgp); }

std::optional<bool> Status::enable_off() const { return !bit(raw.dss2, reg::kDss2Enbs); }

std::optional<bool> Status::auto_servo_off() const {
    if (!raw.stat) return std::nullopt;
    return bit(*raw.stat, reg::kStatAsof);
}

std::optional<bool> Status::nv_busy() const {
    if (!raw.stat || rcon()) return std::nullopt;  // STAT AEEP undefined on RCON (DOC ME0413 p.5-19)
    return bit(*raw.stat, reg::kStatAeep);
}

std::optional<bool> Status::manual_mode() const { return bit(raw.dsse, reg::kDsseRmds); }

std::optional<bool> Status::battery_low() const {
    if (!raw.stat) return std::nullopt;
    return bit(*raw.stat, reg::kStatBatl);
}

bool Status::excitation_done() const { return bit(raw.dsse, reg::kDssePsns); }

std::vector<std::string> Status::servo_block_reasons() const {
    // Before a servo ON only the DSSE / DSS2 blockers count (DOC ME0162-12A p.4-11, 4-19 to 4-21, 5-83); STAT MPOW is
    // UNVERIFIED on an EC and is not used here.
    std::vector<std::string> reasons;
    if (bit(raw.dsse, reg::kDsseEmgp))
        reasons.push_back("the emergency-stop input is active (DSSE EMGP; DOC ME0162-12A p.4-20)");
    if (bit(raw.dsse, reg::kDsseMpuv))
        reasons.push_back(
            "the motor drive power is cut (DSSE MPUV / STAT MPOW = 0): check the drive cut-off circuit and the "
            "external stop wiring (DOC ME0162-12A p.4-20, 4-21; ME0155 p.66, 131)");
    if (!bit(raw.dss2, reg::kDss2Enbs))
        reasons.push_back(
            "the enable switch of a connected teaching tool is released (DSS2 ENBS = 0; DOC ME0162-12A p.4-19, 5-83; "
            "ME0155 p.35)");
    return reasons;
}

}  // namespace iai_actuator_driver
