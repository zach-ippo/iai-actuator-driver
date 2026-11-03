// Private to the core (internal: not installed, not part of the API yet): a simulated SEL controller (IAI Protocol B /
// Format B) on a serial line, usable wherever a Transport is. It reads command frames (sel_frame.hpp), answers them as
// the controller does, and moves its axes in simulated time. It never opens a port. The controls below are C++ only
// (no C hooks yet).
//
// Behaviour follows the serial Format B manual where it says something (DOC); where it is silent the simulator makes
// the worst-case choice a driver must survive, and says so. Everything is UNVERIFIED on a controller: no SEL controller
// has been tested.
// * A frame it cannot accept gets silence, never an error reply (DOC): a bad checksum, start character, terminator or
//   hex field, another station, a command longer than the controller's receive buffer. "@@" in place of the checksum
//   is accepted (DOC). Every LF ends a frame; bytes before it that do not form a command are dropped silently.
// * Execute commands (id 232H and up) are answered only while execute_enabled (a channel not enabled for execution
//   gets silence and nothing is done, DOC). They are refused with an error reply ('&', the SIM-only codes below) in
//   MANU mode, while the system is not ready, and (servo ON, home, moves, jog) during an emergency stop; 238H stop is
//   accepted in every state. Queries are answered in every mode (DOC).
// * The stale-bit-4 worst case is on by default: after the reply to an operation command (servo, home, move, jog) the
//   axis status keeps its old bits - bit 0 "in use" 0, bit 4 "completed" from the previous command - for
//   in_use_delay_s; only then does bit 0 turn ON, bit 4 clear and the motion start. The manual does not say when bit 0
//   turns ON (DOC); a driver that polls at once and trusts bit 4 sees the previous command's completion.
// * Completion (DOC): bit 0 OFF with bit 4 ON = done; bit 0 OFF with bit 4 OFF = cancelled (238H stop, emergency stop,
//   a mode switch to MANU, an axis error). Servo commands keep bit 4 as it is (UNVERIFIED).
// * Motion: every axis of one command follows one trapezoid and all finish together (CP); the axis with the longest
//   distance runs at the commanded speed. 0 in a speed / acceleration field takes the profile's parameter (or the
//   stored position's own value for 237H / 26BH); speeds above an axis' maximum and accelerations above
//   max_accel_g are clamped (the manual: the speed is limited by the axis parameters, DOC). 238H decelerates to a
//   stop; 262H re-plans the running move from where it is. Home runs to 0 at home_speed then waits home_dwell_s.
// * Refusals of a move: an axis not present, an axis with an error (until 252H), servo OFF, not homed (moves, jog),
//   an axis in use (including the in_use_delay_s window), a target outside the axis' soft limits. Nothing moves then.
// * Reply buffer: a position read (209H / 21FH) returns only the records that fit in max_reply_bytes, the whole reply
//   frame counted (DOC: the record count is clipped to the buffer). The TTA's serial buffers are 2051 bytes in, 2048
//   out; the older TT's (and the XSEL-J/K family's) 1023 in, 1024 out (DOC).
// * 25BH software reset: no reply (DOC), then silence for reset_s while it re-initialises: servos OFF, motion and
//   errors gone, homes lost unless `absolute`.
// * reply_latency_s (I/O parameter 97): a reply becomes readable only that long after its command (DOC: the
//   controller waits at least that long). Reads never block (has_real_timeout() is false): the caller polls.
// Implemented: 200H, 201H, the setting inquiry (202H + 090), 208H, 209H, 212H, 215H, 216H, 21EH, 21FH, 232H, 233H,
// 234H, 235H, 236H, 237H, 238H, 252H, 25BH, 262H, 26BH. Every other id is refused with kSelSimErrorUnsupported, as
// are the 4-digit position messages on the TT profile (they are not in its message list, DOC).
// Thread-safe: every call holds the controller's mutex.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/transport.hpp"
#include "sel_messages.hpp"

namespace iai_actuator_driver {
namespace sel {

/// The simulated controllers.
enum class SelSimProfile {
    tta,  ///< TTA table-top robot: model code B9h, 4 axes X/Y/Z (200 / 200 / 100 mm) + R (360 deg), 30000 positions,
          ///< 2051 / 2048-byte serial buffers. The default.
    tt,   ///< the older TT: model code BCh, the same 4 axes, 3000 positions, 1023 / 1024-byte buffers, no 4-digit
          ///< position messages (21EH, 21FH, 26BH)
};

/// "tta", "tt".
const char* to_string(SelSimProfile profile) noexcept;

/// One simulated axis.
struct SelSimAxis {
    double min = 0.0;          ///< soft limits, mm (deg on a rotary axis)
    double max = 200.0;
    double max_speed = 800.0;  ///< mm/s (deg/s): faster commands are clamped
    double position = 0.0;     ///< at power-up
    bool rotary = false;
};

/// Controller modes of 215H (DOC).
enum class SelSimMode { automatic = 1, manual = 2 };

/// Faults inject_fault applies to the next commands this station accepts (one per command).
enum class SelSimFault {
    silence,              ///< the command is lost: not acted on, no reply
    silent_after_acting,  ///< acted on, the reply lost (a re-sent execute would act twice)
    bad_checksum,         ///< the reply's checksum is wrong
    wrong_station,        ///< the reply names another station
    wrong_id,             ///< the reply names another message id (a late reply to something else)
    partial,              ///< only the first half of the reply
    error_reply,          ///< an error reply with kSelSimErrorInjected, nothing done
};

// Error-reply codes. The manual gives no error-code table (DOC); these are SIM-only codes, chosen not to collide with
// the four the manual names (684H, 236H, 23CH, CC1H). A driver must treat every code as opaque.
inline constexpr int kSelSimErrorManualMode = 0xF01;    ///< an execute command in MANU mode
inline constexpr int kSelSimErrorEmergencyStop = 0xF02; ///< servo ON, home, move or jog during an emergency stop
inline constexpr int kSelSimErrorNotReady = 0xF03;      ///< an execute command while the system is not ready
inline constexpr int kSelSimErrorServoOff = 0xF04;      ///< home, move or jog on an axis with its servo OFF
inline constexpr int kSelSimErrorNotHomed = 0xF05;      ///< a move or jog on an axis not homed
inline constexpr int kSelSimErrorAxisInUse = 0xF06;     ///< an operation on an axis already in use
inline constexpr int kSelSimErrorOutOfStroke = 0xF07;   ///< a target outside the soft limits
inline constexpr int kSelSimErrorNoAxis = 0xF08;        ///< an axis pattern naming an axis that is not present
inline constexpr int kSelSimErrorFormat = 0xF09;        ///< data that does not match the message's layout
inline constexpr int kSelSimErrorUnsupported = 0xF0A;   ///< a message id this profile does not answer
inline constexpr int kSelSimErrorNoPosition = 0xF0B;    ///< 237H / 26BH to an undefined position
inline constexpr int kSelSimErrorInjected = 0xF0C;      ///< SelSimFault::error_reply
inline constexpr int kSelSimErrorAxisError = 0xF0D;     ///< an operation on an axis with an error (until 252H)

/// Settings of a SimulatedSel. Every nullopt takes the profile's value.
struct SimulatedSelOptions {
    int station = 0x99;                        ///< I/O parameter 91 (the manual's examples use 99h)
    SelSimProfile profile = SelSimProfile::tta;
    std::function<double()> clock;             ///< monotonic seconds; empty = now_s() (the clock seam of timing.hpp)
    double time_scale = 1.0;                   ///< simulated seconds per clock second
    std::optional<std::vector<SelSimAxis>> axes;  ///< axes 1..n (at most 8); nullopt: the profile's
    std::optional<int> model_code;             ///< 201H
    std::optional<std::size_t> max_command_bytes;  ///< receive buffer: longer commands get silence
    std::optional<std::size_t> max_reply_bytes;    ///< transmit buffer: list replies are clipped to it
    std::optional<int> position_capacity;      ///< highest position number
    double in_use_delay_s = 0.05;              ///< the stale-bit-4 window after an operation command (0 disables)
    double reply_latency_s = 0.0;              ///< I/O parameter 97
    double servo_s = 0.05;                     ///< how long a servo ON / OFF keeps the axis in use
    double home_speed = 20.0;                  ///< mm/s (deg/s) when 233H gives 0
    double home_dwell_s = 0.1;                 ///< in use at 0 after the home run
    double default_speed = 100.0;              ///< mm/s when a move gives 0 (the axis parameter)
    double default_accel_g = 0.3;              ///< G when a move gives 0
    double max_accel_g = 1.0;                  ///< acceleration / deceleration clamp
    double reset_s = 1.0;                      ///< silent re-initialisation after 25BH or power-up
    bool execute_enabled = true;               ///< false: execute commands get silence and are not acted on
    bool absolute = false;                     ///< homes survive a software reset or power cycle
    bool servo_on = false;                     ///< initial servo state of every axis
    bool homed = false;                        ///< initial home state of every axis
    std::map<int, std::uint32_t> settings;     ///< setting inquiry values (others read 0); e.g. 185, 189
    std::map<int, PositionData> positions;     ///< the stored position table
    std::string port_name = "SIM-SEL";
};

/// One axis as the simulator sees it now.
struct SelSimAxisState {
    double position = 0.0;
    bool servo_on = false;
    int home_state = 0;      ///< 0 not homed, 1 homing, 2 homed (212H bits 1-2)
    bool in_use = false;     ///< 212H bit 0
    bool completed = false;  ///< 212H bit 4
    int error_code = 0;
};

class SimulatedSel : public Transport {
public:
    explicit SimulatedSel(const SimulatedSelOptions& options = {});
    ~SimulatedSel() override;
    SimulatedSel(const SimulatedSel&) = delete;
    SimulatedSel& operator=(const SimulatedSel&) = delete;

    // ---- Transport
    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override { return 0; }
    void set_baud_rate(int rate) override;  // std::logic_error: no line is modelled
    double read_timeout() const override;
    void set_read_timeout(double seconds) override;
    void send_break(double seconds) override;  // std::logic_error
    void close() override;
    bool is_open() const override;
    std::string name() const override;
    std::optional<bool> present() const override;
    bool can_reopen() const override { return true; }
    void reopen(std::optional<int> baud_rate = std::nullopt) override;

    // ---- fixed facts
    SelSimProfile profile() const;
    int station() const;
    int axis_count() const;
    std::size_t max_command_bytes() const;
    std::size_t max_reply_bytes() const;

    // ---- live state
    SelSimAxisState axis(int axis) const;  ///< axis 1..axis_count() (std::out_of_range otherwise)
    SelSimMode mode() const;
    bool emergency_stop() const;
    /// Simulated seconds since construction.
    double now() const;

    // ---- controls (each takes effect at once)
    /// AUTO / MANU (the mode switch). Switching to MANU cancels every motion (bit 4 OFF).
    void set_mode(SelSimMode mode);
    /// Press (true) or release the emergency stop. Pressing cancels every motion and switches every servo OFF;
    /// releasing leaves them OFF.
    void set_emergency_stop(bool on);
    /// 215H byte 3 bit 2 "system ready"; false refuses every execute command but 238H.
    void set_ready(bool ready);
    /// 215H byte 2 bit 0 "flash ROM writing" (reported only).
    void set_flash_writing(bool on);
    /// False: execute commands get silence and are not acted on (the channel is not the one enabled for them).
    void set_execute_enabled(bool enabled);
    /// An axis error: the axis' motion is cancelled, its servo goes OFF, 212H shows `code` and 215H / 216H the latest
    /// system error, until 252H.
    void set_axis_error(int axis, int code);
    /// Levels set directly (no motion; nullopt: as it is). Refused (std::logic_error) while the axis is in use.
    void set_axis(int axis, std::optional<bool> servo_on, std::optional<bool> homed = std::nullopt,
                  std::optional<double> position = std::nullopt);
    void set_position_data(int number, const PositionData& data);
    void clear_position_data(int number);
    void set_setting(int number, std::uint32_t value);
    /// The next `count` commands this station accepts get `fault`.
    void inject_fault(SelSimFault fault, int count = 1);
    /// Total silence (true) until released: frames are read and counted, nothing is answered or done.
    void set_silent(bool silent);
    /// Power off (silence, state lost) and on again: silent for reset_s, then as after a software reset.
    void power_cycle();
    /// The adapter pulled: present() false; write / read / reset_input throw TransportError until reopen() after
    /// replug(). The controller keeps running.
    void unplug();
    void replug();

    // ---- counters
    /// Command frames for this station that passed the frame check, per message id (every id: frames()).
    int frames(int message_id) const;
    int frames() const;
    /// Commands acted on, per message id (not refused, not lost to a fault or a disabled channel).
    int executed(int message_id) const;
    /// LF-terminated lines that were dropped silently (bad frame, another station, too long, while silent / resetting).
    int dropped() const;
    /// Replies sent (normal and error, faulty ones included).
    int replies() const;
    void reset_counters();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sel
}  // namespace iai_actuator_driver
