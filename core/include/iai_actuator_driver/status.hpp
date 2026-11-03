// Decoded monitor block (0x9000...). Port of the Python edition's ec.Status (C# Status.cs); implemented by the model
// unit (src/status.cpp).
//
// `moving` is "the axis is in motion": DSSE MOVE, a home return in progress (GHMS), or servo on without PEND / PSFL /
// STP while homed and not in a fault. The raw MOVE bit alone is not reliable on the EC: it clears while still
// decelerating and during the homing creep (VERIFIED). `velocity_mm_s` is 0 when not moving (also with only 8 words
// read); while moving it is VNOW (coarse on the EC; nullopt when only 8 words were read) - the Axis replaces it with a
// speed derived from successive PNOW samples. The condition flags
// (emg_input, drive_power_cut, ...) are nullopt when their word was not read, never a guessed false; their meanings are
// DOC (ME0162-12A p.4-18 to 4-25) and UNVERIFIED on an EC.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/alarms.hpp"

namespace iai_actuator_driver {

/// The words a status read returned (22 by default; 20, 14 or 8 when a unit refuses the longer read).
struct StatusRaw {
    int words = 0;            ///< how many words were decoded (8, 14, 20 or 22)
    std::uint32_t pnow = 0;
    std::uint16_t almc = 0;
    std::uint16_t dipm = 0;
    std::uint16_t dopm = 0;
    std::uint16_t dss1 = 0;
    std::uint16_t dss2 = 0;
    std::uint16_t dsse = 0;
    std::optional<std::uint32_t> stat;  ///< >= 14 words
    std::optional<std::uint32_t> vnow;
    std::optional<std::uint32_t> cnow;
    std::optional<std::uint32_t> devi;  ///< >= 20 words
    std::optional<std::uint32_t> stim;
    std::optional<std::uint16_t> sipm;
    std::optional<std::uint16_t> zons;
    std::optional<std::uint16_t> poss;  ///< 22 words
    std::optional<std::uint16_t> ssse;
};

/// One decoded status sample.
struct Status {
    double position_mm = 0.0;           ///< PNOW (units: mm, or deg on a rotary axis), 0.01 resolution
    int alarm_code = 0;                 ///< ALMC, 0 = none
    bool servo_on = false;              ///< DSS1 SV
    bool homed = false;                 ///< DSS1 HEND
    bool in_position = false;           ///< DSS1 PEND
    bool moving = false;                ///< see the file comment
    bool alarm = false;                 ///< a fault that blocks motion (alarm_is_fault)
    bool modbus_enabled = false;        ///< DSSE PMSS: Modbus has control
    bool emergency_stop = false;        ///< DSS1 EMGS
    std::optional<double> velocity_mm_s;  ///< 0 when not moving; nullopt while moving when only 8 words were read
    std::optional<int> current_ma;      ///< CNOW (signed), nullopt with fewer than 14 words
    StatusRaw raw;
    /// "rcon" behind an RCON gateway (set by the Axis): the bits RCON driver units leave undefined ("-": DSSE b12, b7,
    /// b6, b4, b3, b0; STAT AEEP; ZONS LS0-LS2; DOC ME0413 p.5-15 to 5-19) read as nullopt, POSS has 7 bits and
    /// push_contact() is PEND + DSSE PUSH without PSFL (UNVERIFIED). nullopt on other lines.
    std::optional<std::string> line;
    /// line == "rcon".
    bool rcon() const { return line && *line == "rcon"; }

    /// Decode words read from 0x9000: at least 8 (std::invalid_argument otherwise); 14, 20 or 22 add fields.
    static Status from_registers(const std::vector<std::uint16_t>& words);

    // ---- derived flags
    bool warning() const;                    ///< a message-level alarm (or ALML) and no fault
    std::optional<AlarmLevel> alarm_level() const;  ///< nullopt when no code
    bool fault() const { return alarm; }
    bool move_bit() const;                   ///< raw DSSE MOVE
    bool homing() const;                     ///< DSSE GHMS
    bool paused() const;                     ///< DSS1 STP
    bool target_reached() const;             ///< DSS1 b0 MEND
    bool push_missed() const;                ///< DSS1 PSFL
    bool pushing() const;                    ///< DSSE b10
    bool push_contact() const;               ///< PEND and DSSE b0 in this one sample
    bool safety_speed() const;               ///< DSS1 SFTY
    std::optional<bool> at_backward_end() const;  ///< ZONS LS0
    std::optional<bool> at_forward_end() const;   ///< ZONS LS1
    std::optional<bool> at_middle() const;        ///< ZONS LS2
    std::optional<double> vnow_mm_s() const;      ///< VNOW (coarse) when read
    std::optional<double> uptime_s() const;       ///< STIM / 1000 when read
    /// Not moving, not paused, not homing: safe for relative decisions and non-volatile writes.
    bool idle() const { return !moving && !paused() && !homing(); }
    /// A move would be accepted as far as the bits tell: servo on, homed, Modbus control, no fault / EMG / pause / homing.
    bool ready() const;

    // ---- condition flags (nullopt when the word was not read)
    std::optional<int> position_complete_no() const;  ///< POSS & 0x03FF (& 0x007F on RCON)
    std::optional<bool> cold_start_alarm() const;     ///< SSSE b11
    std::optional<bool> rtc_in_use() const;           ///< SSSE b8
    std::optional<bool> drive_power_cut() const;      ///< STAT MPOW = 0 or DSSE MPUV
    std::optional<bool> emg_input() const;            ///< DSSE EMGP
    std::optional<bool> enable_off() const;           ///< DSS2 ENBS = 0
    std::optional<bool> auto_servo_off() const;       ///< STAT ASOF
    std::optional<bool> nv_busy() const;              ///< STAT AEEP: non-volatile write in progress
    /// DSSE b13 RMDS: the unit's MANU / AUTO mode bit (true = MANU; on a unit with a MANU / AUTO switch, AUTO gives the
    /// PIO control; DOC ME0162-12A p.4-20). Not PC vs PIO control: that is modbus_enabled (PMSS). On the EC it stays
    /// true in PC and in PIO mode alike (VERIFIED in the Python edition on the EC-GRBP8M gripper, PMSS 0 and 1).
    std::optional<bool> manual_mode() const;
    std::optional<bool> battery_low() const;          ///< STAT BATL
    bool excitation_done() const;                     ///< DSSE PSNS

    /// Why a servo ON cannot work, from the flags above (EMG input, drive power cut, enable switch released); empty
    /// when nothing blocks it.
    std::vector<std::string> servo_block_reasons() const;
};

}  // namespace iai_actuator_driver
