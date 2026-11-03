// Byte-level simulated unit on an RS-485 line, usable wherever a Transport is: it parses Modbus RTU request frames and
// answers like a slave (FC03 / 05 / 06 / 10 / 11 / 17 and IAI 0x45 / 0x48; other functions exception 01), so the whole library is
// testable without hardware: Axis::open(std::make_shared<SimulatedUnit>()). It never opens a port. Port of the Python
// edition's sim.py + sim_fixtures.py (C# Simulation/SimulatedUnit.cs, SimulatedUnitOptions.cs, SimFixtures.cs);
// implemented by the sim unit (src/sim_unit*.cpp, src/sim_fixtures.cpp).
//
// Each profile is seeded with the values read live from that unit (directory, identification block with a placeholder
// serial, soft limits, parameter classes, position rows, alarm history, FC11 answer). Motion is a real-time trapezoid
// (time_scale speeds it up). Behaviour follows the hardware tests of the Python edition:
// * PEND / MEND rise inside the band while still moving, and the last settle_tail_mm is covered slowly: a driver must
//   also wait for PNOW to stop changing. DSSE MOVE clears during the final deceleration.
// * STOP 0x042C latches until written OFF (stop_latches; worst case, UNVERIFIED): moves and home returns are ignored
//   while ON. 0x9900 moves run with PMSS = 0; 0x9800 row moves are silently ignored with PMSS = 0 (test N4).
// * Parameter writes (classes 7 / 8 / 9 / 0xC) land in RAM: they read back at once and are lost by a restart unless
//   the PARW coil committed them (DRGE b7 reads 1 for parw_busy_s).
// * STP is a level input. Home: HEND 0, GHMS 1, to the mechanical end, a creep back to 0.00 with MOVE 0 but GHMS 1,
//   then HEND 1 + PEND 1. Push rows (CTLF b1): approach, then push over the band; obstacle_mm gives contact, none PSFL.
// * IAI 0x45: servo cmd 9, home cmd 0x0A (op1 = the home pattern; 0 rejected 0x11), alarm reset cmd 5 op1 0, software
//   reset cmd 5 op1 1 (after servo off; with the servo on alarm 0x090), action mode 0x0C, stored row cmd 6 at op2/255
//   of its speed, broadcast baud change cmd 4 (codes in ignored_baud_codes are ignored, as the EC-R6S rod ignored 0x06).
//   Unknown commands are rejected 0x21. On an EC profile cmd 6 with a row the position mode does not run (row 2 in
//   2-position mode) is rejected 0x11 and nothing moves, as on the EC-GRBP8M gripper; 0x9800 still runs it (unverified).
//   After a software reset PMSS reads 0 (the EC-R6S rod), except on the gripper and the PCON profiles (kept at 1).
// * The baud rate is modelled only with options.baud_rate: the unit then answers only when the PC side matches its
//   rate; a BREAK of 150 ms or more drops it to 9600 (DOC).
// * Reads are bounded by the directory: 0 inside a block without recorded data, exception 02 outside every block.
// * Reads never block: write() answers at once (read() returns what is buffered; only the rod's FC11 answer becomes
//   readable after its 0.3 s), so has_real_timeout() stays false and the bus polls for its own deadline.
// * The command profile 0x0F00 (FC17 / FC10; read back as written, no motion) runs with the PCRF coil 0x042E (separate
//   acceleration and deceleration, push / incremental CTLF bits; a new profile + PCRF retargets). The acceleration
//   pattern CTLF b6-b7 of each start is recorded (accel_pattern()); the motion stays a trapezoid (src/sim_profile.cpp),
//   a 0x9900 S-motion / filter move at half the commanded acceleration (a smoothed profile, as in the Python edition).
// * IAI 0x48 (src/sim_controls.cpp): cmd 0x30 servo, 0x31 home, 0x34 decel stop, 0x23 alarm reset; others rejected
//   0x21. Teach coils MOD 0x0414 / TEAC 0x0415: with MOD on, TEAC writes PNOW into row POSR and sets DSS2 b10 PWRT
//   until released (alarm 0x093 before homing; DSS2 b11 MODS follows MOD).
// * set_inputs: EMG (DSS1 EMGS, DSSE EMGP), a drive cut (STAT MPOW 0, DSSE MPUV) and the enable switch (DSS2 ENBS)
//   switch the servo off; a servo ON is ignored while any is active. auto_servo_off_s: the servo goes off by itself
//   after that standstill (STAT b17 ASOF); SON or a move command switches it back on. cold_boot: servo off and DSSE
//   PSNS 0 at power-up, the first servo ON takes the excitation detection (CON: from item 9.1; EC: ec_first_son_s).
// * function_codes: only those are answered (exception 01 otherwise); broadcast: frames to address 0 are acted on.
// Not simulated (the Python edition shapes it): S-motion / filter shaping of the motion.
// Power / link loss (recovery.hpp) is simulated: power_off / power_on / power_cycle(lose_home), unplug / replug /
// reopen, SimulatedUnitOptions::serial_number (a replaced unit). Other function codes are answered with exception 01, coils the unit does not
// know with exception 02.
// What it is NOT: proof of a unit's behaviour. Where the tests and manuals are silent it makes a simple choice.
// Thread-safe: every call holds the unit's mutex. Evidence: the simulated behaviour follows what docs/COMPATIBILITY.md
// lists as VERIFIED on hardware; the rest is UNVERIFIED.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

/// The per-unit defaults: one profile per unit tested, plus two simulator-only units.
enum class SimProfile {
    Slider,   ///< EC-DS4H-150 in 3-position mode: 150 mm, 700 mm/s, FC11 EC FF 02 00 13, absolute (WA). The default.
    Gripper,  ///< EC-GRBP8M-20 in 2-position mode: 9.50 mm, 53 mm/s, MAXA / MAXD 0.30 G, FC11 EC FF 0A 00 02, absolute.
    Rod,      ///< EC-R6S-150: 150 mm, 800 mm/s, FC11 EC FF 00 00 02, incremental, FC11 answered after 0.3 s.
    Pcon,     ///< PCON-CB + RCP6-WSA10R (FC11 AE FF 44 00 00): 150 mm, 840 mm/s, 64 rows, message alarm 0x069 at power-up;
              ///< nothing on its PIO connector: PIO mode turns the servo off and shows a pause (DSS1 STP).
    Rotary,   ///< simulator-only: the rod's blocks with AKID 7.33 = 1, a rotary axis in degrees (0..360)
    Unknown,  ///< simulator-only: FC11 system 0x99 unit 0x7E, AKID 42, blank model string, on the rod's blocks
};

/// "slider", "gripper", "rod", "pcon", "rotary", "unknown".
const char* to_string(SimProfile profile);
/// The profile of a name (case-insensitive), nullopt when unknown.
std::optional<SimProfile> sim_profile_from_string(const std::string& name);
/// Every profile, in declaration order.
std::vector<SimProfile> sim_profiles();

/// Layout of the PIO output image DOPM (0x9004).
enum class DopmLayout {
    ThreePosition,  ///< b0 LS0, b1 LS1, b2 LS2, b3 *ALM (the slider)
    TwoPosition,    ///< b0 LS0, b1 LS1, b2 *ALM (the gripper, the rod)
};

/// A fault inject_fault applies to the next responses.
enum class SimFault {
    Timeout,     ///< no answer at all
    Crc,         ///< the last CRC byte flipped
    Partial,     ///< only the first half of the frame
    WrongSlave,  ///< the answer comes from another slave address
};

/// Settings of a SimulatedUnit. Every nullopt takes the profile's value.
struct SimulatedUnitOptions {
    int slave = 1;                           ///< address it answers to (axis 0 = 1)
    SimProfile profile = SimProfile::Slider;
    std::optional<double> stroke_mm;         ///< soft limits -0.30 .. stroke + 0.30 (alarm 0x0D9 outside)
    std::optional<double> position_mm;       ///< initial position
    double time_scale = 1.0;                 ///< simulated seconds per real second (e.g. 20 makes tests fast)
    std::function<double()> clock;           ///< monotonic seconds; empty = now_s()
    std::optional<double> max_speed_mm_s;    ///< MAXV 7.4 and the default speed 8.10; VCMD above it -> alarm
    double max_accel_g = 1.0;                ///< ACMD above this -> alarm
    std::optional<int> system_code;          ///< FC11 byte 0
    std::optional<int> unit_code;            ///< FC11 byte 2
    std::optional<std::pair<int, int>> firmware;  ///< FC11 bytes 3-4
    std::optional<DopmLayout> dopm;
    bool pmsl = true;                        ///< initial Modbus control (the hardware boots with 0)
    bool recompute_position_checksum = true; ///< update word +0xF after a row write (an assumption)
    bool drop_reply_after_retarget = false;  ///< no answer to the first request after a mid-move command (seen once)
    bool stop_latches = true;                ///< STOP stays ON until written OFF (worst case)
    bool move_bit_clears_in_decel = true;
    bool safety_speed = false;               ///< initial SFTY
    bool soft_limits = true;                 ///< serve the soft-limit block (false: exception 02 there)
    double ls_range_mm = 0.10;
    bool early_in_position = true;           ///< PEND / MEND rise inside the band while still moving
    double settle_tail_mm = 0.05;            ///< the last part of a move covered at settle_speed_mm_s (0 disables)
    double settle_speed_mm_s = 0.5;
    std::optional<double> push_blip_mm;      ///< hardware test G5: a contact fingerprint for a moment, then a miss
    double push_blip_len_mm = 0.10;
    std::optional<bool> absolute;            ///< keeps HEND at a software reset (absolute) or loses it
    std::optional<int> baud_rate;            ///< model the line at this PC-side rate (nullopt: no baud rate at all)
    std::optional<int> unit_baud_rate;       ///< the unit's own rate (default = baud_rate)
    int boot_baud_rate = 38400;              ///< after power_cycle() (VERIFIED)
    std::optional<int> reset_baud_rate;      ///< after a software reset (nullopt: unchanged)
    double reset_s = 1.0;                    ///< simulated seconds the unit is silent after a software reset
    std::optional<double> slave_id_delay_s;  ///< real seconds before an FC11 answer can be read (rod: 0.3)
    double parw_busy_s = 0.1;                ///< real seconds DRGE b7 reads 1 after PARW (ON-WIRE: +55 / +235 ms)
    bool parw_dsse_busy = false;             ///< DSSE b7 reads 0 while the commit runs (seen on a PCON)
    std::vector<int> ignored_baud_codes = {0x06};
    std::optional<std::string> serial_number; ///< controller serial at 0x0600 (up to 16 ASCII; a replaced unit)
    std::optional<std::uint16_t> move_patterns; ///< override of the MVPT support word 0x0103 (b1 S-motion, b2 filter)
    // 0.5: the rest of the Python edition's SimulatedEC options
    std::optional<double> auto_servo_off_s;  ///< automatic servo OFF after this standstill (simulated s; nullopt: off)
    bool cold_boot = false;                  ///< start (and power up) with the servo off and DSSE PSNS 0
    double ec_first_son_s = 0.3;             ///< on an EC with cold_boot: how long the first servo ON takes
    bool rtc = false;                        ///< SSSE b8: a real-time clock in use
    std::optional<std::vector<int>> function_codes;  ///< only these are answered (others: exception 01); nullopt: all
    std::optional<bool> broadcast;           ///< frames to address 0 are acted on (nullopt: unless function_codes)
    bool parw_latches = true;                ///< the PARW coil starts a commit (false: acknowledged and ignored)
    // 0.6
    std::optional<std::string> port_name;    ///< the port's name() (nullopt: "SIM"), e.g. "COM99" for a pretend port
};

/// Live state a test sets directly (the Python edition's attribute writes): the stored level only, no action is taken
/// (nullopt: as it is).
struct SimStateLevels {
    std::optional<bool> servo_on;
    std::optional<bool> homed;
    std::optional<bool> pmsl;
    std::optional<bool> safety_speed;
};

/// Alarm codes the simulator raises.
inline constexpr int kSimAlarmServoOffMove = 0x080;
inline constexpr int kSimAlarmNotHomedMove = 0x083;
inline constexpr int kSimAlarmMoveWhileHoming = 0x084;
inline constexpr int kSimAlarmPositionNo = 0x085;
inline constexpr int kSimAlarmBadBand = 0x0A3;
inline constexpr int kSimAlarmBadDecel = 0x0A7;
inline constexpr int kSimAlarmBadSpeed = 0x0AB;
inline constexpr int kSimAlarmSoftLimit = 0x0D9;
inline constexpr int kSimAlarmResetServoOn = 0x090;
inline constexpr int kSimAlarmTeachNotHomed = 0x093;
inline constexpr int kSimAlarmAbsError = 0x0EE;
/// 0x45 rejection codes the simulator gives.
inline constexpr int kSimRejectOperand1 = 0x11;
inline constexpr int kSimRejectCommandCode = 0x21;
inline constexpr int kSimRejectServoOff = 0x23;
inline constexpr int kSimRejectHoming = 0x24;
inline constexpr int kSimRejectMonitorMode = 0x3A;

class SimulatedUnit : public Transport {
public:
    explicit SimulatedUnit(const SimulatedUnitOptions& options = {});
    SimulatedUnit(SimProfile profile, int slave = 1, double time_scale = 1.0);
    ~SimulatedUnit() override;
    SimulatedUnit(const SimulatedUnit&) = delete;
    SimulatedUnit& operator=(const SimulatedUnit&) = delete;

    // ---- fixed facts
    SimProfile profile() const;
    int slave() const;
    double stroke_mm() const;
    double max_speed_mm_s() const;
    int system_code() const;
    int unit_code() const;
    int position_count() const;   ///< stored rows the unit reports (MAXP, item 3.2: slider 3, gripper 2, rod 2, PCON 64)
    bool absolute() const;
    double time_scale() const;

    // ---- live state (snapshots)
    bool servo_on() const;
    bool homed() const;
    bool pmsl() const;
    bool safety_speed() const;
    bool paused() const;
    bool stop_held() const;
    int alarm_code() const;
    double position_mm() const;
    bool moving() const;
    Span soft_limits_mm() const;
    std::optional<double> obstacle_mm() const;
    void set_obstacle_mm(std::optional<double> mm);   ///< a workpiece for push rows
    std::optional<int> unit_baud_rate() const;        ///< the rate the unit listens at (nullopt: not modelled)

    // ---- counters and logs
    int nonvolatile_writes() const;    ///< position-table row writes that changed or rewrote a row
    int parameter_commits() const;     ///< completed PARW commits
    int parameter_ram_writes() const;  ///< parameter words written to RAM
    int software_resets() const;
    int override_moves() const;        ///< moves started with 0x45 cmd 6
    int profile_moves() const;         ///< moves started with the PCRF coil (src/sim_profile.cpp)
    std::string accel_pattern() const; ///< "trapezoid" / "s_motion" / "filter" / "invalid": CTLF b6-b7 of the last start
    double command_speed_mm_s() const; ///< the speed of the move in progress (VCMD; after a speed change the new one)
    double push_speed_mm_s() const;    ///< the speed of the last push phase: min(VCMD, the unit's push speed)
    std::vector<std::vector<std::uint8_t>> requests() const;  ///< every request frame (the last 2000), oldest first
    void clear_requests();
    std::vector<std::string> events() const;  ///< what the unit did ("stop", "home complete"...), last 2000
    void clear_events();
    /// (0.6) The PC side's rates in order: the one the line is modelled at, then every set_baud_rate() and every
    /// reopen() at a given rate (empty: the line is not modelled). The Python edition's SimulatedEC.baud_history.
    std::vector<int> baud_history() const;
    std::map<int, std::uint16_t> parameter_ram() const;  ///< written since the last commit / restart
    std::map<int, std::uint16_t> parameter_nv() const;   ///< committed: kept across restarts
    bool parw_busy() const;

    // ---- direct access for tests
    bool coil_state(int address) const;
    void set_coil_state(int address, bool on);   ///< the stored level only: no action is taken (write a frame to act)
    std::vector<std::uint16_t> position_row(int row) const;       ///< 16 words (std::out_of_range outside 0..767)
    /// Replace a row without a write cycle (no checksum recompute, nonvolatile_writes() unchanged); 16 words.
    void set_position_row(int row, const std::vector<std::uint16_t>& words);
    /// What an FC03 of one word would return (std::out_of_range where the unit answers exception 02).
    std::uint16_t register_value(int address) const;

    // ---- 0.5: the rest of the Python edition's SimulatedEC test hooks (src/sim_controls.cpp)
    /// The settings as they are now: the options the unit was made with, as changed since by set_options, with the LS
    /// range the last restart applied.
    SimulatedUnitOptions options() const;
    /// Change settings later, as the Python edition's option attributes do: every setting is taken from `options` but
    /// the fixed facts (profile, slave, time_scale, stroke_mm, position_mm, max_speed_mm_s, the FC11 codes, absolute,
    /// the line's rates, serial_number, rtc, clock), which stay. An unset optional takes the profile's value.
    void set_options(const SimulatedUnitOptions& options);
    void set_state(const SimStateLevels& levels);
    /// The emergency-stop input (DSS1 EMGS, DSSE EMGP), a motor drive power cut (STAT MPOW 0, DSSE MPUV) and the
    /// teaching-tool enable switch (false: released, DSS2 ENBS 0); nullopt: as it is. Each that becomes active switches
    /// the servo off (a move stops at once); while any is active a servo ON is ignored.
    void set_inputs(std::optional<bool> emg, std::optional<bool> drive_cut, std::optional<bool> enable);
    bool auto_servo_off_active() const;  ///< STAT b17 ASOF: the servo is off because of the automatic servo OFF
    /// Set one parameter word as if written (nv false: the parameter RAM, lost by a restart) or committed (nv true: the
    /// non-volatile store). The unit does not act on it until a restart, as for a written word.
    void set_parameter_word(int address, std::uint16_t value, bool nv);
    double ls_range_mm() const;  ///< the LS range as applied (EC parameter 2 takes effect at a restart)

    // ---- events
    void inject_fault(SimFault kind, int count = 1);
    /// Raise an alarm now (a message-level code keeps the axis running; any other switches the servo off).
    void trigger_alarm(int code);
    /// A move commanded over PIO (PMSS 0): returns false when refused. hold_start keeps the start input held.
    bool pio_move(double target_mm, double speed_mm_s = 20.0, double accel_g = 0.3, bool hold_start = false);
    void pio_release();
    /// Power loss + power-up: boot baud rate, PC control off, uptime restarted, HEND lost on an incremental unit,
    /// a 0x0FFF record in the alarm history, uncommitted parameters lost.
    void power_cycle();

    // ---- power / link loss (recovery.hpp; implemented in src/sim_faults.cpp). Port of the Python edition's
    // SimulatedEC.power_off / power_on / unplug / replug / reopen.
    /// power_cycle() with HEND decided by `lose_home` (true: lost; false: kept, as one power-up of the EC-R6S rod
    /// still read HEND 1).
    void power_cycle(bool lose_home);
    /// The 24 V supply cut: the axis stops where it is (a move in progress is lost) and the unit stays silent until
    /// power_on(): frames go unanswered and the master times out.
    void power_off();
    /// The supply back: the power-up of power_cycle(). `lose_home`: drop HEND (nullopt: on an incremental unit).
    void power_on(std::optional<bool> lose_home = std::nullopt);
    bool powered() const;
    /// The USB adapter pulled: the PC's port vanishes (present() false) and the open handle dies: write / read /
    /// reset_input throw TransportError until reopen() after replug(). The unit keeps its power, state and baud rate.
    void unplug();
    /// Plugged back in: the port is listed again; the old handle stays dead until reopen().
    void replug();
    std::optional<bool> present() const override;
    bool can_reopen() const override { return true; }
    /// Open the port again after close() or unplug() (TransportError while unplugged). `baud_rate`: the PC side's
    /// rate afterwards (a modelled line only; nullopt keeps it). Pending replies are lost.
    void reopen(std::optional<int> baud_rate = std::nullopt) override;
    int reopens() const;
    /// The controller serial served at 0x0600.
    std::string serial_number() const;

    /// Answer one request frame (what a read would return after writing it; empty = no answer). Usable directly: the
    /// baud, silence, drop-reply and injected-fault rules apply as for write(); the FC11 delay and the pending output
    /// buffer do not. Throws TransportError once close()d.
    std::vector<std::uint8_t> handle_frame(const std::vector<std::uint8_t>& request);

    // ---- Transport
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override;            ///< 0 when the line is not modelled
    void set_baud_rate(int rate) override;     ///< std::logic_error when not modelled
    double read_timeout() const override;
    void set_read_timeout(double seconds) override;
    void send_break(double seconds) override;
    void close() override;
    bool is_open() const override;
    std::string name() const override;         ///< port_name, else "SIM" (axis names "SIM:1"); SimLine names its lines

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iai_actuator_driver
