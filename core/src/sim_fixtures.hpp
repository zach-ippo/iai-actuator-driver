// Private to the sim unit (src/sim_fixtures.cpp, sim_unit.cpp, sim_motion.cpp, sim_line.cpp and tests/test_sim_*.cpp):
//   1. ProfileFixture / fixture(): the per-profile data read live from the units tested (directory, identification
//      block with a placeholder serial, class blocks, position rows, alarm history, support words). Port of the
//      Python edition's sim_fixtures.py and the profile records of sim.py (C# SimFixtures.cs + SimulatedUnit's table).
//   2. SimulatedUnit::Impl: the state and behaviour of one simulated unit (a port of the Python edition's SimulatedEC,
//      C# SimulatedUnit.cs). Every Impl method assumes the unit's mutex is held by the caller (SimulatedUnit's public
//      members lock once and call in); sim_unit.cpp holds the frame, register, coil, parameter and IAI code,
//      sim_motion.cpp the motion model and the axis behaviour, sim_controls.cpp the inputs, the automatic servo OFF,
//      the cold boot, the teach coils, IAI 0x48 and the settings a test changes later.
//   3. sim_test: frame helpers shared by the three test files (a test needs no bus: raw frames through handle_frame).
// Header-only dependencies only (crc, registers constants, errors, timing, util): the sim unit links on its own.
// Unofficial: not an IAI product. Evidence words VERIFIED / ON-WIRE / DOC / UNVERIFIED as in the other editions; the
// behaviour ported here is "VERIFIED in the Python edition" where its hardware tests say so, this edition itself is
// UNVERIFIED on hardware.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace sim_detail {

// ------------------------------------------------------------------------------------------ profile data

/// Everything a profile seeds a SimulatedUnit with.
struct ProfileFixture {
    SimProfile profile = SimProfile::Slider;
    double stroke_mm = 150.0;
    double position_mm = 0.0;
    double max_speed_mm_s = 700.0;
    int system_code = 0xEC;
    int unit_code = 0x02;
    std::pair<int, int> firmware{0x00, 0x13};
    DopmLayout dopm = DopmLayout::ThreePosition;
    std::string index_table_hex;        ///< the directory words from 0x0000 (hex)
    std::string identification_hex;     ///< 0x0600, 72 words (hex)
    std::string limit_params_hex;       ///< class 7 at 0x0700 (hex; words 0-3 are replaced by the soft limits)
    std::string motion_params_hex;      ///< class 8 at 0x0800 (hex; 0x0812 is replaced by the maximum speed)
    std::map<int, std::string> blocks;  ///< start address -> its words (hex): ID maps, classes 1 / 3 / 4 / 9 / 0C, ...
    std::map<int, std::vector<std::uint16_t>> positions;    ///< row -> 16 words
    std::map<int, std::vector<std::uint16_t>> simple_rows;  ///< rows 256.. (an EC's simple-data store)
    std::vector<std::uint16_t> blank_row;                   ///< a row never written (PCON: speed 0, ...); empty = zeros
    std::string alarm_history_hex;      ///< 0x0500: entries 0.. (6 words each, hex); the rest are empty
    std::vector<std::uint16_t> support; ///< class-1 support words where the blocks have no 0x0100
    std::uint16_t ssse = 0;
    int ollv = 0;
    bool absolute = true;               ///< keeps HEND at a software reset
    double slave_id_delay_s = 0.0;      ///< FC11 answered after this (rod 0.3)
    std::optional<int> akid;            ///< override of AKID 7.33 (rotary / unknown profiles)
    bool con_reset = false;             ///< CON-style software reset (PCON: params reload, keeps baud / PMSS)
    /// PMSS kept across a software reset: the EC-GRBP8M gripper (VERIFIED in the Python edition); con_reset implies
    /// it; the EC-R6S rod dropped it to 0 (VERIFIED), which the other EC profiles keep doing
    bool reset_keeps_pmss = false;
    bool rtc_alarm = false;             ///< message alarm 0x069 at power-up (PCON)
    /// Nothing on the PIO connector (PCON): a switch to PIO mode turns the servo off and the unwired *STP input shows as
    /// a pause (DSS1 STP) until Modbus control is back (VERIFIED on the PCON-CB, 2026-11-02); not modelled at power-up
    bool pio_unwired = false;
    /// 0x45 cmd 0x0C with op1 = 1 clears DSSE b12 (from PIO and already in PC, either op2), and while b12 is 0 a servo
    /// ON is ignored (SON written, STAT b1 stays 0, no alarm); a PMSL coil OFF (1 to 0) sets b12 again, the only thing
    /// tried that did (PMSL ON written again did not). b12 reads 1 otherwise, the servo on or off (VERIFIED on the
    /// PCON-CB, 2026-11-02). Whether op1 = 0 sets it again is untested: not modelled (worst case); a restart clears it
    /// here (UNVERIFIED: a power cycle with b12 at 0 was never tried)
    bool action_mode_locks_son = false;
    /// An alarm that switches the servo off leaves the servo-on level set (the SON coil / DRG1 b12 / 0x45 cmd 9 written
    /// last), and the alarm reset then turns the servo back on by itself (ON-WIRE on the PCON-CB, 2026-11-03: after a
    /// push back error 0x0DC one alarm reset at rest re-energised the motor). The EC profiles: not modelled
    bool alarm_reset_restores_servo = false;
    std::vector<std::pair<int, int>> ranges;        ///< [start, end) of every directory block and ID map
    std::vector<std::pair<int, int>> param_ranges;  ///< [start, end) of the parameter classes 7 / 8 / 9 / 0C data
};

/// The fixture of a profile.
const ProfileFixture& fixture(SimProfile profile);

// ------------------------------------------------------------------------------------------ constants

inline constexpr double kHomingSpeedMmS = 30.0;   ///< toward the mechanical end (hardware: ~30 mm/s)
inline constexpr double kHomingEndMm = -2.00;     ///< the mechanical end seen while homing on the test EC
inline constexpr double kCreepSpeedMmS = 5.0;     ///< back to 0.00 (the hardware creep is slow; exact value irrelevant)
inline constexpr double kVnowStep = 12.48;        ///< VNOW quantization seen on the test EC (mm/s)
inline constexpr double kIntegrationStepS = 0.001;///< simulated seconds per integration step
inline constexpr double kBreakTo9600S = 0.15;     ///< DOC ME0162: a line break of 150 ms or more drops a unit to 9600
inline constexpr double kPushSpeedFallbackMmS = 20.0;  ///< push speed when neither PSHV nor MXPH is served
inline constexpr std::size_t kLogLimit = 2000;    ///< requests / events kept
inline constexpr int kRowCount = reg::kPositionMaxEntries;  ///< 768 rows
inline constexpr int kHistoryEntries = reg::kAlarmHistoryEntries;

// bits the shared register header does not name
inline constexpr std::uint16_t kDrg1Sfty = 1u << 14;
inline constexpr std::uint16_t kDrg1Son = 1u << 12;
inline constexpr std::uint16_t kDrg1Alrs = 1u << 8;
inline constexpr std::uint16_t kDrg1Stp = 1u << 5;
inline constexpr std::uint16_t kDrg1Home = 1u << 4;
inline constexpr std::uint16_t kDrg1Cstr = 1u << 3;
inline constexpr std::uint16_t kDsseSson = 1u << 12;
inline constexpr std::uint16_t kDsseDst1 = 1u << 4;
inline constexpr std::uint16_t kZonsZ1 = 1u << 0;
inline constexpr std::uint16_t kZonsZ2 = 1u << 1;
inline constexpr std::uint32_t kStatSon = 1u << 1;
inline constexpr std::uint32_t kStatSv = 1u << 2;
inline constexpr std::uint32_t kStatHend = 1u << 3;
inline constexpr std::uint32_t kStatRmds = 1u << 4;
inline constexpr std::uint16_t kDopmLs0 = 1u << 0;
inline constexpr std::uint16_t kDopmLs1 = 1u << 1;
inline constexpr std::uint16_t kDopm3PosLs2 = 1u << 2;
inline constexpr std::uint16_t kDopm3PosNoAlarm = 1u << 3;
inline constexpr std::uint16_t kDopm2PosNoAlarm = 1u << 2;
inline constexpr std::uint16_t kDsseIdle = 0x33C2;  ///< live idle value of DSSE (the dynamic bits are masked out)

/// Python round(): half to even (std::nearbyint in the default rounding mode).
inline long long py_round(double value) { return static_cast<long long>(std::nearbyint(value)); }

/// IAI baud code -> rate (DOC ME0162; 38400 = 6 as BRSL), nullopt when unknown. Own copy: the simulator does not link
/// the bus unit.
inline std::optional<int> baud_rate_of_code(int code) {
    switch (code) {
        case 0x04: return 9600;
        case 0x05: return 19200;
        case 0x06: return 38400;
        case 0x07: return 76800;
        case 0x11: return 14400;
        case 0x12: return 28800;
        case 0x13: return 57600;
        case 0x14: return 115200;
        case 0x15: return 230400;
        default: return std::nullopt;
    }
}

/// True for a message-level alarm (the named codes of 0x011-0x07F): ALML, the axis keeps running. Own copy of the model
/// layer's table (UNVERIFIED levels).
inline bool is_message_alarm(int code) {
    return code == 0x040 || code == 0x041 || (code >= 0x047 && code <= 0x04A) || (code >= 0x04C && code <= 0x04F) ||
           (code >= 0x05A && code <= 0x05E) || (code >= 0x061 && code <= 0x065) || (code >= 0x068 && code <= 0x06B) ||
           (code >= 0x070 && code <= 0x07C) || code == 0x07F;
}

/// True for a cold-start-level alarm (SSSE b11 ALMC): 0x001-0x010, and the codes of 0x080-0x0FF that do not merely
/// cancel the operation (0x0EE counts as operation cancelled on an absolute unit).
inline bool is_cold_start_alarm(int code) {
    if (code >= 0x001 && code <= 0x010) return true;
    if (code < 0x080 || code > 0x0FF || code == 0x0EE) return false;
    static const std::set<int> cancelled = {0x080, 0x082, 0x083, 0x084, 0x085, 0x086, 0x087, 0x088, 0x089, 0x08A, 0x090,
                                            0x091, 0x092, 0x093, 0x094, 0x095, 0x096, 0x0A2, 0x0A3, 0x0A4, 0x0A5, 0x0A7,
                                            0x0AB, 0x0AC, 0x0AD, 0x0AE, 0x0B5, 0x0B6, 0x0BA, 0x0BE, 0x0BF, 0x0C0, 0x0C1,
                                            0x0C2, 0x0C5, 0x0CF, 0x0D6, 0x0D8, 0x0D9, 0x0DA, 0x0DB, 0x0DC, 0x0DE, 0x0DF,
                                            0x0E1, 0x0ED, 0x0EF, 0x0F1};
    return cancelled.count(code) == 0;
}

/// A Modbus exception raised inside a handler (the code goes out as fc | 0x80, code).
struct SimException {
    int code;
};

enum class HomingPhase { Off, Seek, Creep };
enum class PushPhase { Off, Approach, Push };

/// A move a PLC started over PIO with its start input held (pio_move hold_start).
struct PioStart {
    double target_mm = 0.0;
    double speed_mm_s = 0.0;
    double accel_g = 0.0;
};

}  // namespace sim_detail

// ------------------------------------------------------------------------------------------ SimulatedUnit::Impl

struct SimulatedUnit::Impl {
    using Bytes = std::vector<std::uint8_t>;
    using Words = std::vector<std::uint16_t>;

    explicit Impl(const SimulatedUnitOptions& options);

    // ---- configuration
    SimulatedUnitOptions opt;
    const sim_detail::ProfileFixture& fx;
    SimProfile profile;
    int slave;
    double stroke_mm;
    double time_scale;
    double max_speed_mm_s;
    double max_accel_g;
    int system_code;
    int unit_code;
    std::pair<int, int> firmware;
    DopmLayout dopm;
    bool absolute;
    bool con_reset;
    bool rtc_unset;
    bool pio_unwired;
    bool action_mode_locks_son;
    bool alarm_reset_restores_servo;
    bool recompute_position_checksum;
    bool drop_reply_after_retarget;
    bool stop_latches;
    bool move_bit_clears_in_decel;
    bool serve_soft_limits;
    Span soft_limits;
    double ls_range_mm;
    bool early_in_position;
    double settle_tail_mm;
    double settle_speed_mm_s;
    std::optional<double> push_blip_mm;
    double push_blip_len_mm;
    std::optional<double> obstacle_mm;
    double slave_id_delay_s;
    double parw_busy_s;
    bool parw_dsse_busy;
    std::set<int> ignored_baud_codes;
    int boot_baud_rate;
    std::optional<int> reset_baud_rate;
    double reset_s;
    std::function<double()> clock;
    double t0 = 0.0;

    // ---- line, logs, counters
    mutable std::mutex mutex;
    std::deque<std::uint8_t> out;
    std::deque<SimFault> faults;
    bool closed = false;
    bool powered = true;       ///< 24 V present (power_off / power_on; sim_faults.cpp)
    bool present = true;       ///< the port is listed by the PC (unplug / replug)
    bool handle_lost = false;  ///< the open handle died with an unplug: every call throws until reopen()
    int reopens = 0;
    double read_timeout = kDefaultReadTimeoutS;
    std::deque<Bytes> requests;
    std::deque<std::string> events;
    int nonvolatile_writes = 0;
    int parameter_commits = 0;
    int parameter_ram_writes = 0;
    int software_resets = 0;
    int override_moves = 0;
    std::optional<int> line_baud;  ///< the PC side (nullopt: no line model)
    std::optional<int> ec_baud;    ///< the unit's own rate
    std::vector<int> baud_history; ///< (0.6) line_baud as modelled, then every change (baud_history())
    double reply_ready_at = 0.0;  ///< real time before which a delayed (FC11) answer is not readable
    double busy_until = -1.0;      ///< simulated time until which the unit is silent (software reset)
    bool drop_next_reply = false;
    std::optional<sim_detail::PioStart> pio_start;
    std::optional<double> parw_until;               ///< real time the running commit completes
    std::map<int, std::uint16_t> parw_snapshot;     ///< the parameter RAM as it was at the PARW coil

    // ---- read-only information and parameters
    Words index;
    Words identification;
    Words limit_params;
    Words motion_params;
    std::map<int, std::uint16_t> statics;
    std::map<int, std::uint16_t> param_ram;  ///< written since the last commit / restart
    std::map<int, std::uint16_t> param_nv;   ///< committed
    std::optional<int> alarm_item_address;
    std::optional<int> move_counter_address;
    std::uint16_t ssse = 0;
    int ollv = 0;
    std::vector<std::array<std::uint16_t, 6>> alarm_history;
    Words table;                              ///< 0x1000..0x3FFF: 768 rows of 16 words
    std::array<std::uint16_t, 10> direct{};   ///< 0x9900..0x9909
    std::uint16_t drg1_other = 0;
    std::uint16_t drg2 = 0;
    std::uint16_t posr = 0;
    std::uint16_t posr_imm = 0;
    std::map<int, bool> coils;
    std::vector<std::optional<std::uint32_t>> home_params_at_home;
    bool abs_home_lost = false;

    // ---- axis state
    double pos = 0.0;
    double vel = 0.0;
    double target = 0.0;
    double vmax = 0.0;
    double accel = 0.0;   ///< mm/s^2
    double decel = 0.0;   ///< mm/s^2
    double band = 0.10;
    bool moving = false;
    bool paused = false;       ///< a move is being held by STP
    bool stp_input = false;    ///< STP level (coil 0x040A / DRG1 bit 5)
    bool pio_pause = false;    ///< DSS1 STP from the unwired *STP input while in PIO mode (pio_unwired)
    sim_detail::HomingPhase homing_phase = sim_detail::HomingPhase::Off;
    sim_detail::PushPhase push_phase = sim_detail::PushPhase::Off;
    double push_end = 0.0;
    int push_dir = 1;
    double push_speed = 0.0;
    bool settling = false;     ///< inside the settling tail of a normal move
    bool push_contact = false;
    bool push_missed = false;
    bool accelerating = false;
    bool decelerating = false;
    bool target_reached = false;
    bool stopping = false;
    bool servo_on = true;
    bool homed = true;
    bool pmsl = true;
    bool safety_speed = false;
    int alarm_code = 0;
    bool alarm_major = false;
    double t = 0.0;            ///< simulated time of the last advance()
    int poss = 0;              ///< completed stored row
    std::optional<int> row_move;  ///< the stored row of the move in progress (for POSS)

    // ---- inputs, automatic servo OFF, cold boot, teach, function codes (sim_controls.cpp)
    std::optional<double> auto_servo_off_s;
    bool cold_boot = false;
    double ec_first_son_s = 0.3;
    std::optional<std::set<int>> function_codes;  ///< only these are answered (nullopt: all)
    bool broadcast = true;                        ///< frames to address 0 are acted on
    bool parw_latches = true;
    bool emg_input = false;        ///< set_inputs: emergency-stop input
    bool drive_cut = false;        ///< set_inputs: motor drive power cut
    bool enable_released = false;  ///< set_inputs: teaching-tool enable switch released
    bool asof = false;             ///< STAT b17: the servo was switched off by the automatic servo OFF
    std::optional<double> asof_at;  ///< simulated time the automatic servo OFF is due
    bool excited = true;           ///< DSSE PSNS: the excitation detection is done
    bool son_locked = false;       ///< DSSE b12 cleared by the action mode PC: servo ON ignored (action_mode_locks_son)
    bool son_level = true;         ///< the servo-on level written last (alarm_reset_restores_servo)
    std::optional<double> son_due;  ///< simulated time a slow (first) servo ON completes
    bool teach_mode = false;       ///< MOD coil (DSS2 b11 MODS)
    bool teach_written = false;    ///< TEAC wrote a row (DSS2 b10 PWRT) and is still held
    bool son_blocked() const;
    void request_servo(bool on);
    double first_son_s() const;
    void wake_from_asof(const char* what);
    void timers();
    void teach();
    Bytes fc48(const Bytes& body);
    void apply_options(const SimulatedUnitOptions& o);

    // ---- time and motion (sim_motion.cpp)
    double now() const;
    void advance();
    void step(double dt);
    void begin_motion(double target_mm, double speed, double accel_mm_s2, double band_mm,
                      std::optional<double> decel_mm_s2 = std::nullopt);
    void next_phase(double target_mm, double speed);
    void arrive();
    void motion_ended(bool completed);
    bool check_move(double target_mm, double band_mm, double speed, double accel_g);
    void start_move(double target_mm, double band_mm, double speed, double accel_g,
                    std::optional<double> decel_g = std::nullopt);
    void start_push(double target_mm, double band_mm, double speed, double accel_g, int direction);
    double unit_push_speed() const;
    void start_home();
    void decel_stop();
    void move_to_row(int n, double speed_scale = 1.0);
    bool blocked(const char* what);
    void set_servo(bool on, const char* reason = "");
    void set_pause(bool on);
    void control_switched(bool pc);  ///< PMSS changed by the PMSL coil or the action mode (pio_unwired)
    void raise_alarm(int code, std::optional<bool> major = std::nullopt);
    void alarm_reset();
    void restart(bool power_up);
    void power_off();
    void power_on();
    // power / link loss (sim_faults.cpp)
    void check_handle() const;                         ///< TransportError while the handle is dead (unplug)
    void set_serial_number(const std::string& text);   ///< the controller serial at 0x0600 (16 ASCII, space padded)
    void software_reset_command();
    void apply_parameters();
    // derived values (sim_motion.cpp)
    std::vector<std::uint16_t> row_words(int n) const;
    std::pair<double, bool> row_marker(int n) const;
    bool at_row(int n) const;
    std::uint16_t zons() const;
    std::uint16_t dopm_word() const;
    bool early_band() const;
    bool push_blip() const;
    std::uint16_t dss1() const;
    std::uint16_t dsse() const;
    std::uint16_t dss2() const;
    std::uint16_t ssse_word() const;
    bool move_bit() const;
    std::uint16_t drg1() const;
    int current_ma() const;
    std::uint32_t stim_ms() const;
    std::uint16_t monitor_word(int address) const;
    std::uint16_t status_image(int offset) const;

    // ---- frames, registers, coils, parameters, IAI (sim_unit.cpp)
    void log_event(const std::string& text);
    std::optional<Bytes> respond(const Bytes& request);            ///< one request frame in, the answer out
    std::optional<Bytes> handle_frame_core(const Bytes& frame);
    Bytes fc03(const Bytes& body);
    Bytes fc05(const Bytes& body);
    Bytes fc06(const Bytes& body);
    Bytes fc10(const Bytes& body);
    Bytes fc11(const Bytes& body);
    Bytes fc45(const Bytes& body);
    int move_position_override(int n, int override_value);
    int action_mode(int pc, int safety);
    int iai_command(int action, int op1);
    void start_commit();
    void parw_tick();
    void finish_commit();
    std::uint16_t read_word(int address) const;
    void write_words(int address, const Words& values);
    void write_direct(int address, const Words& values);
    void write_drg1(std::uint16_t value);
    void write_coil(int address, bool on);
    bool in_directory(int address) const;
    bool in_parameter_ram(int address, int count) const;
    std::optional<int> item_address(int class_id, int item_id) const;
    std::uint16_t base_word(int address) const;
    std::vector<std::optional<std::uint32_t>> home_parameters() const;
    std::uint16_t position_checksum_of_row(int row) const;
    bool stop_held() const;

    // ---- command profile 0x0F00 and acceleration patterns (sim_profile.cpp)
    std::optional<Words> profile_words;        ///< the block as last written (nullopt: what the last move applied)
    int profile_moves = 0;                     ///< moves started with the PCRF coil
    std::string accel_pattern = "trapezoid";   ///< CTLF b6-b7 of the last move start ("invalid" for 11)
    Bytes fc17(const Bytes& body);
    std::uint16_t active_profile(int offset) const;
    void write_profile(int address, const Words& values);
    void run_profile();
    void set_accel_pattern(std::uint16_t ctlf);
};

// ------------------------------------------------------------------------------------------ test helpers

/// Frame helpers shared by tests/test_sim_*.cpp (header-only, inline; not part of the library).
namespace sim_test {

using Bytes = std::vector<std::uint8_t>;
using Words = std::vector<std::uint16_t>;

/// A simulated clock the test advances by hand (SimulatedUnitOptions::clock).
struct FakeClock {
    double seconds = 100.0;
    double read() const { return seconds; }
    void advance(double s) { seconds += s; }
    std::function<double()> reader() {
        return [this] { return seconds; };
    }
};

/// A unit on a hand-advanced clock (SimulatedUnitOptions::clock = the FakeClock). Not movable: the clock lambda keeps
/// the address of `clock`.
struct Rig {
    FakeClock clock;
    std::unique_ptr<SimulatedUnit> unit;
    explicit Rig(SimulatedUnitOptions options = {}) {
        options.clock = [this] { return clock.seconds; };
        unit.reset(new SimulatedUnit(options));
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    SimulatedUnit& u() { return *unit; }
    SimulatedUnit* operator->() { return unit.get(); }
};

/// Options with the settling tail off: the pure trapezoid (what most motion tests check).
inline SimulatedUnitOptions pure_trapezoid(SimProfile profile = SimProfile::Slider) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.settle_speed_mm_s = 0.0;
    return o;
}

/// A complete request frame from "03 90 00 00 02" (the PDU in hex; slave and CRC added).
inline Bytes frame(int slave, const std::string& pdu_hex) { return build_frame(slave, util::hex_bytes(pdu_hex)); }

/// The answer to a request: the full response frame (empty = silence).
inline Bytes exchange(SimulatedUnit& unit, const std::string& pdu_hex, int slave = 1) {
    return unit.handle_frame(frame(slave, pdu_hex));
}

/// The first `n` words.
inline Words head(const Words& words, std::size_t n) { return Words(words.begin(), words.begin() + static_cast<std::ptrdiff_t>(std::min(n, words.size()))); }

/// Upper-case hex of bytes ("01 83 02 C0 F1"), for readable CHECK_EQ failures.
inline std::string hexs(const Bytes& bytes) {
    std::string text;
    for (std::size_t i = 0; i < bytes.size(); ++i) text += util::format(i ? " %02X" : "%02X", static_cast<unsigned>(bytes[i]));
    return text;
}

/// Upper-case hex of words ("1CE3 0000").
inline std::string hexs(const Words& words) {
    std::string text;
    for (std::size_t i = 0; i < words.size(); ++i) text += util::format(i ? " %04X" : "%04X", static_cast<unsigned>(words[i]));
    return text;
}

/// Big-endian ASCII words as text, trimmed (the model string of the identification block). Own copy: the simulator
/// tests do not link the model unit.
inline std::string ascii_of(const Words& words) {
    std::string text;
    for (std::uint16_t w : words)
        for (int shift : {8, 0}) {
            const int c = (w >> shift) & 0xFF;
            if (c >= 0x20 && c < 0x7F) text += static_cast<char>(c);
        }
    return util::trim(text);
}

/// build_frame of an answer, for comparisons: slave + hex PDU + CRC.
inline Bytes expect(int slave, const std::string& pdu_hex) { return frame(slave, pdu_hex); }

/// The PDU of an answer without slave and CRC.
inline Bytes pdu_of(const Bytes& response) {
    if (response.size() < 4) return {};
    return Bytes(response.begin() + 1, response.end() - 2);
}

inline Bytes be16(int value) { return {static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value & 0xFF)}; }

/// FC03 read of `count` words; throws std::runtime_error with the exception code on an exception answer.
inline Words read_regs(SimulatedUnit& unit, int address, int count, int slave = 1) {
    Bytes pdu = {0x03};
    for (int v : {address, count}) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) throw std::runtime_error("no answer to FC03");
    if (response[1] & 0x80) throw std::runtime_error("exception " + std::to_string(static_cast<int>(response[2])));
    Words words;
    for (std::size_t i = 0; i < static_cast<std::size_t>(response[2]) / 2; ++i)
        words.push_back(static_cast<std::uint16_t>((response[3 + 2 * i] << 8) | response[4 + 2 * i]));
    return words;
}

/// The exception code of an FC03 read (0 when it succeeded, -1 when silent).
inline int read_exception(SimulatedUnit& unit, int address, int count, int slave = 1) {
    Bytes pdu = {0x03};
    for (int v : {address, count}) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) return -1;
    return (response[1] & 0x80) ? response[2] : 0;
}

/// FC10 write; returns the exception code (0 = accepted).
inline int write_regs(SimulatedUnit& unit, int address, const Words& values, int slave = 1) {
    Bytes pdu = {0x10};
    for (int v : {address, static_cast<int>(values.size())}) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    pdu.push_back(static_cast<std::uint8_t>(2 * values.size()));
    for (std::uint16_t v : values) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) return -1;
    return (response[1] & 0x80) ? response[2] : 0;
}

/// FC06 write; returns the exception code (0 = accepted).
inline int write_reg(SimulatedUnit& unit, int address, int value, int slave = 1) {
    Bytes pdu = {0x06};
    for (int v : {address, value}) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) return -1;
    return (response[1] & 0x80) ? response[2] : 0;
}

/// FC05 write; returns the exception code (0 = accepted).
inline int coil(SimulatedUnit& unit, int address, bool on, int slave = 1) {
    Bytes pdu = {0x05};
    const Bytes a = be16(address);
    pdu.insert(pdu.end(), a.begin(), a.end());
    pdu.push_back(on ? 0xFF : 0x00);
    pdu.push_back(0x00);
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) return -1;
    return (response[1] & 0x80) ? response[2] : 0;
}

/// IAI 0x45: the rejection code of the answer (0 = accepted, -1 = silent, -2 = an exception response).
inline int iai(SimulatedUnit& unit, int command, int op1 = 0, int op2 = 0, int slave = 1) {
    Bytes pdu = {0x45, 0x00, static_cast<std::uint8_t>(command)};
    for (int v : {op1, op2}) {
        const Bytes w = be16(v);
        pdu.insert(pdu.end(), w.begin(), w.end());
    }
    const Bytes response = unit.handle_frame(build_frame(slave, pdu));
    if (response.size() < 5) return -1;
    if (response[1] & 0x80) return -2;
    return response[2];
}

/// The nine words of a 0x9900 move block: PCMD, INP, VCMD, ACMD, PPOW, CTLF (mm, mm, mm/s, G).
inline Words move_block(double mm, double band_mm, double speed_mm_s, double accel_g, int ppow = 0, int ctlf = 0) {
    const auto p = reg::to_i32_words(static_cast<std::int32_t>(reg::to_counts(mm)));
    const auto b = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(band_mm)));
    const auto v = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(speed_mm_s)));
    return {p.first, p.second, b.first, b.second, v.first, v.second,
            static_cast<std::uint16_t>(reg::to_counts(accel_g)), static_cast<std::uint16_t>(ppow),
            static_cast<std::uint16_t>(ctlf)};
}

/// The first `n` words of a move block (7 words = no PPOW / CTLF: the stale-CTLF hazard).
inline Words move_words(double mm, double band_mm, double speed_mm_s, double accel_g, std::size_t n) {
    Words w = move_block(mm, band_mm, speed_mm_s, accel_g);
    w.resize(n);
    return w;
}

/// Start a move over the 0x9900 block (all 9 words); returns the exception code (0 = accepted).
inline int begin_move(SimulatedUnit& unit, double mm, double speed_mm_s, double accel_g, double band_mm = 0.10,
                int ppow = 0, int ctlf = 0) {
    return write_regs(unit, 0x9900, move_block(mm, band_mm, speed_mm_s, accel_g, ppow, ctlf));
}

/// PNOW of an 0x9000 read (two words) in mm.
inline double pnow_mm(const Words& words) {
    return reg::from_i32_words(words.at(0), words.at(1)) * reg::kPositionScale;
}

/// True when `events` holds exactly `text`.
inline bool contains(const std::vector<std::string>& events, const std::string& text) {
    for (const auto& e : events)
        if (e == text) return true;
    return false;
}

/// `count` words from `address`, in FC03 reads of at most 100 words.
inline Words read_block(SimulatedUnit& unit, int address, int count, int slave = 1) {
    Words words;
    for (int done = 0; done < count; done += 100) {
        const Words part = read_regs(unit, address + done, std::min(100, count - done), slave);
        words.insert(words.end(), part.begin(), part.end());
    }
    return words;
}

/// A 32-bit item of a directory class found the way the driver finds it: through the unit's own directory and the
/// class's ID map (nullopt when the unit has no such item).
inline std::optional<std::uint32_t> read_item(SimulatedUnit& unit, int class_id, int item_id, int slave = 1) {
    const int count = read_regs(unit, 0, 8, slave)[2];
    const Words records = read_block(unit, 8, 8 * count, slave);
    for (int i = 0; i < count; ++i) {
        const int ci = records[8 * i], dq = records[8 * i + 2], da = records[8 * i + 4], ia = records[8 * i + 5];
        if (ci != class_id || ia == 0 || records[8 * i + 7] != 2) continue;
        const Words ids = read_block(unit, ia, 2 * dq, slave);
        for (int k = 0; k < dq; ++k) {
            if (ids[2 * k] != item_id) continue;
            const Words value = read_regs(unit, da + 2 * k, 2, slave);
            return reg::from_u32_words(value[0], value[1]);
        }
    }
    return std::nullopt;
}

}  // namespace sim_test
}  // namespace iai_actuator_driver
