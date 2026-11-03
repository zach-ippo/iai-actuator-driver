// One axis: an ELECYLINDER or a CON-family controller (PCON / ACON / SCON / DCON with a RoboCylinder) at one slave
// address. Port of the Python edition's ec.Elecylinder (the source of truth; C# Axis.cs, Axis.Motion.cs,
// Axis.Config.cs); implemented by the axis unit (src/axis_*.cpp).
//
// Rules the Axis keeps (ported; VERIFIED in the Python edition unless marked):
// * Every motion input is checked against the unit's own limits (limits()) BEFORE anything is sent: SoftLimitError,
//   nothing written. Every unit is driven the same way.
// * Moves use the 9-word direct block at 0x9900 (FC10): the move starts at once and re-targets a running move.
//   move_by / jog are absolute targets computed from the measured position (the EC's incremental base is not verified;
//   the JOG coils run at ~100 mm/s from a parameter and are never used).
// * Completion (the safe rule): PEND (or PSFL) AND the position within the band of the target AND PNOW unchanged over
//   settle_s (PEND can rise before standstill; MOVE clears early). An alarm ends the wait with EcAlarmError, a timeout
//   sends a decel stop and raises MotionTimeoutError; settling away from the target raises MotionIncompleteError.
// * stop(): STOP coil 0x042C ON, held until standstill (PNOW unchanged over settle_s, at most the timeout), then OFF.
// * Modbus control (PMSL) is taken automatically at rest only (never switched during motion: that stops a move).
// * Non-volatile writes (position table, teach, parameter commits) need no opt-in and pass the non-volatile write
//   guard of results.hpp (unchanged -> skipped; nonvolatile_writes() counter; a warning above 60 per hour).
// * No watchdog: if the PC or cable fails, a move in progress finishes. This driver is not a safety function.
//
// Thread safety: the bus serialises frames (one mutex per port); the Axis guards its own state. stop() may be called
// from another thread while a thread waits for a move: that wait (move_to, push, a stored row, MoveHandle::wait /
// poll / done) ends at once with MotionIncompleteError ("stopped from outside this wait") and sends no STOP of its own
// (ON-WIRE on a PCON-CB + RCP6: before, it ended about 3 s later and sent a second STOP). A newer move of the axis from
// another thread ends it the same way ("superseded by a newer move", no STOP). Several axes may share one Port (daisy
// chain, UNVERIFIED).
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/results.hpp"
#include "iai_actuator_driver/status.hpp"

namespace iai_actuator_driver {

// ------------------------------------------------------------------------------------------ timing constants (s)
inline constexpr double kSettleSDefault = 0.1;      ///< "done" needs PNOW unchanged this long (PEND rises early)
inline constexpr double kPollIntervalSDefault = 0.02;
inline constexpr double kServoTimeoutS = 3.0;       ///< servo on / off must show in DSS1 within this
inline constexpr double kServoFirstOnMaxS = 13.3;   ///< first servo ON of a CON unit with No.29 / 30 unreadable
inline constexpr double kHomeStartS = 2.0;          ///< a home return must visibly start within this
inline constexpr double kStoredStartS = 0.5;        ///< a 0x9800 row move must visibly start within this
inline constexpr double kSettledAwayS = 0.5;        ///< in position away from the target this long -> incomplete
inline constexpr double kErrorStopHoldS = 3.0;      ///< STOP held at most this long on the error path
inline constexpr double kParwReadyS = 2.0;          ///< before a parameter write: DRGE b7 = 0 and DSSE b7 = 1
inline constexpr double kParwClearS = 2.0;          ///< after PARW: idle again within this (ON-WIRE: 0.24 s)
inline constexpr double kParwObserveS = 0.3;        ///< watch this long for the commit to show busy
inline constexpr double kResetWaitS = 1.0;          ///< after the software reset, before probing
inline constexpr double kResetTimeoutS = 10.0;      ///< give up when the unit has not answered this long
inline constexpr double kResetServoOffS = 2.0;      ///< servo OFF must show before the reset is sent
inline constexpr double kResetServoOnS = 1.0;       ///< wait for a servo that comes back on by itself
// distances (units)
inline constexpr double kSamePositionMm = 0.005;    ///< PNOW closer than this = unchanged (0.01 counts)
inline constexpr double kStartMovedMm = 0.015;      ///< PNOW moved more than this = motion started

/// Settings of an Axis. Every cap is optional: nullopt = the unit's own maximum (read from the unit). A configured
/// cap can only narrow the unit's own range, never widen it.
struct AxisOptions {
    std::optional<Span> stroke;               ///< application soft limits, intersected with the unit's
    std::optional<double> max_speed_mm_s;     ///< cap for any commanded speed (units/s)
    std::optional<double> max_accel_g;        ///< cap for any commanded acceleration / deceleration (G)
    std::optional<double> max_push_pct;       ///< cap for push power (%)
    /// How servo / home / alarm reset are sent (Axis::strategy()). Coil, the default, takes the line's strategy where it
    /// names one ("register" behind RCON); Axis::set_strategy(Coil) keeps coils explicitly.
    CommandMethod method = CommandMethod::Coil;
    double poll_interval_s = kPollIntervalSDefault;
    bool auto_modbus_control = true;          ///< take PMSL automatically at rest before a command
    double settle_s = kSettleSDefault;
    double push_settle_s = kPushSettleSDefault;  ///< a push outcome must hold this long (0: the first sample counts)
    Completion completion = Completion::Safe;  ///< when a move counts as done (monitor.hpp); MoveOptions may override
};

/// Settings of Axis::open.
struct OpenOptions {
    int slave = 1;                            ///< 1..247 (axis number + 1)
    std::optional<int> baud_rate;             ///< nullopt: detect in place (115200, 38400, 9600, 230400, 57600, 19200)
    double read_timeout_s = kDefaultReadTimeoutS;
    BusOptions bus;                           ///< retries (3 = IAI's recommendation), gap, frame logger
    AxisOptions axis;
    bool fast = false;                        ///< then enable_fast(): 230400 baud, Completion::Fast, the status monitor
    /// The line, worked out once the baud rate is settled (detect_line: two READ-ONLY probes, ~0.1 s on a plain
    /// line; "direct" / "robonet" send nothing). A ROBONET in gateway mode gives no axis: UnsupportedLineError.
    LineOptions line;
};

/// One absolute / relative move. Positions and speeds in the axis's units (mm, or deg on a rotary axis).
/// Timeouts here and in every command that starts motion or a servo change (servo, home, stored row, push, sequence,
/// change_speed / change_accel, Cell move_all / move_each / home_all / servo_all): nullopt is the default, a value must
/// be finite and above 0, else SoftLimitError before anything is sent (there is no "wait forever").
struct MoveOptions {
    double speed = 50.0;                      ///< units/s: 0.01 .. limits().speed.max
    double accel_g = 0.1;                     ///< G (= decel on a 0x9900 move): 0.01 .. limits().accel_decel.max
    double band = 0.1;                        ///< positioning band: limits().band
    bool wait = true;                         ///< block until done (else return right after the command)
    std::optional<double> timeout_s;          ///< default: 2 x the move's estimate + 2 s
    /// A deceleration other than accel_g: the command-profile path (FC17 0x0F00 + PCRF; accel checked against MAXA,
    /// decel against MAXD). nullopt or equal to accel_g: the 0x9900 block.
    std::optional<double> decel_g;
    /// CTLF b6-b7 (nullopt = trapezoid). Refused when the unit's MVPT lacks it, S-motion above a 2 s accel time.
    std::optional<AccelPattern> accel_pattern;
    /// The completion rule of this move (nullopt: AxisOptions::completion).
    std::optional<Completion> completion;

    /// The jog defaults: 10 units/s.
    static MoveOptions jog() {
        MoveOptions o;
        o.speed = 10.0;
        return o;
    }
};

/// A stored-row move.
struct PositionMoveOptions {
    bool wait = true;
    std::optional<double> timeout_s;
    std::optional<double> speed_pct;          ///< 1..100 % of the row's stored speed (0x45 cmd 6; nothing written)
    bool allow_above_caps = false;            ///< only the unit's own limits apply to the row's speed / accel
};

/// Options of a non-volatile write (position table, teach).
struct NvWriteOptions {
    bool allow_nonvolatile = false;           ///< accepted and ignored (no longer needed): every write goes through
    bool force = false;                       ///< write even when the row already holds these words
    bool allow_unsafe = false;                ///< accepted and ignored (no longer needed): rows past max_positions(),
                                              ///< a row the EC's position mode does not run, unknown flag bits and
                                              ///< role-bit changes are written with a warning (result.warnings)
    bool write_checksum = true;               ///< (0.5) false: write_position_table sends the 15 data words only (the
                                              ///< unit must update word +0xF itself; VerificationError when it does not)
};

/// Options of software_reset.
struct SoftwareResetOptions {
    double timeout_s = kResetTimeoutS;
    double reset_wait_s = kResetWaitS;
};

class MoveHandle;  // move_handle.hpp: a move started without waiting

/// (0.5) What an Axis knows without I/O (Axis::state()): the facts read from the unit so far and the command state,
/// for a binding that shows them as attributes (the Python edition's soft_limits_mm, ec_max_speed_mm_s, unit_code,
/// status_words ...). The caps are computed from what is known (no read; the fallbacks while nothing is).
struct AxisState {
    int status_words = 22;                    ///< words status() reads at 0x9000: 22, 20 or 8 (after a refusal)
    bool motion_pending = false;              ///< a motion command was sent and not seen to finish
    bool stop_held = false;                   ///< the STOP this axis wrote ON (begin_stop) is still held
    bool directory_absent = false;            ///< the directory read timed out on a line where that means "none"
    bool soft_limits_known = false;           ///< read_soft_limits() ran
    std::optional<Span> soft_limits;          ///< the unit's soft limits as stored (with the 0.30 margin)
    std::string soft_limits_note = "not read yet";
    std::optional<AxisLimits> limits;         ///< read_axis_limits(), once read
    std::optional<double> unit_max_speed;     ///< MAXV 7.4, else the default speed 8.10 (once the limits are read)
    std::string unit_max_speed_note = "not read yet";
    std::optional<int> system_code;           ///< FC11 / CTCL, once read
    std::optional<int> unit_code;
    std::optional<std::string> model_name;    ///< 0x0600, once read
    std::optional<int> boot_baud_rate;        ///< BRSL 8.3, once read
    std::optional<int> max_positions;         ///< stored rows, once read
    std::vector<IndexCategory> categories;    ///< the directory, once read (empty before)
    std::optional<double> p1_ceiling_mm;      ///< the largest P1 this axis has read
    double speed_cap = 0.0;                   ///< speed_cap() from what is known
    double accel_cap = 0.0;
    double accel_only_cap = 0.0;
    double decel_cap = 0.0;
    double push_cap = 0.0;
};

namespace detail {
struct AxisAccess;  // the library's own modules (coordinated motion, recovery) reach the axis state (src/axis_impl.hpp)
/// (0.5) Test hook of the C ABI (iad_testing_set_timing): shorten "home_start_s", "parw_ready_s" or "parw_clear_s"
/// process-wide; nullopt restores the default, an empty name all of them (src/axis_state.cpp).
void set_test_timing(const std::string& name, std::optional<double> seconds);
}  // namespace detail

class Axis {
public:
    /// One axis on an open bus (shared; several axes may use one bus). No I/O.
    explicit Axis(std::shared_ptr<ModbusRtu> bus, int slave = 1, AxisOptions options = {});
    /// One axis at `slave` on a shared Port. The port's line is worked out first (Port::detect_line: once per port,
    /// READ-ONLY probes; nothing more once it is known) and its command strategy taken ("register" behind RCON);
    /// UnsupportedLineError on a ROBONET in gateway mode (no axis access).
    explicit Axis(Port& port, int slave = 1, AxisOptions options = {});

    /// Open a serial port once and return an axis that owns it. With options.baud_rate nullopt the rate is found in
    /// place (the port is never closed and reopened). ModbusTimeout (port closed again) when no rate answers, with
    /// kNoAnswerHint; TransportError when the port is busy or absent.
    static Axis open(const std::string& port_name, const OpenOptions& options = {});
    /// The same on any transport (a SimulatedUnit, a SimLine, a FakeTransport); detection only when it has a baud rate.
    static Axis open(std::shared_ptr<Transport> transport, const OpenOptions& options = {});

    ~Axis();
    Axis(Axis&& other) noexcept;
    Axis& operator=(Axis&& other) noexcept;
    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;

    // ---------------------------------------------------------------- session
    /// Read identity and ranges (identify(), limits) and, with take_control, take Modbus control (at rest).
    DeviceInfo connect(bool take_control = true);
    /// Release a STOP / pause this object holds and close the port when the axis owns it. Safe to call twice.
    void close();
    bool owns_bus() const;
    /// (ABI 0.5, py-monitor) This axis is the one user of a bus it did not open (a binding's axis object that owns
    /// its bus: the Python edition's owns_bus): enable_fast may then change the port's baud rate. close() still
    /// leaves the bus open (its owner closes it).
    void set_port_owner(bool owner);
    std::shared_ptr<ModbusRtu> bus() const;
    int slave() const;
    std::optional<int> baud_rate() const;
    bool baud_detected() const;
    AxisOptions options() const;
    void set_options(const AxisOptions& options);
    /// The last identify() / connect() result.
    std::optional<DeviceInfo> device_info() const;
    /// "mm" or "deg" (read once).
    std::string units();
    /// "slider", "rod", "gripper", "rotary"... ("unknown" when not read).
    std::string kind();
    /// "EC", "PCON", ... kGenericFamily.
    std::string product_family();
    bool is_ec();
    std::string to_string() const;

    // ---------------------------------------------------------------- reading (READ-ONLY)
    /// One status read (22 words; falls back to 20 / 8 when refused). The velocity is derived from PNOW samples.
    Status status();
    /// Raw FC03 (diagnostics).
    std::vector<std::uint16_t> read_registers(int address, int count);
    /// Identity: family, model, serials, firmware, axis number, soft limits, kind, units, rows. Kept as device_info().
    DeviceInfo identify();
    /// Every range the driver enforces, read from the unit, with its source.
    AxisRanges limits();
    AxisLimits read_axis_limits();
    SoftLimits read_soft_limits();
    /// Stored rows on this unit (3 on the ECs tested, 64 on the PCON): rows it can read and store. Which of them an EC
    /// runs depends on its position mode (P9): rows 0 and 1 in 2-position mode (P9 0 / 3), rows 0-2 in 3-position mode;
    /// move_to_position, teach_position and write_position_table refuse a row the active mode does not use.
    int max_positions();
    /// The stroke every target is checked against (configured stroke intersected with the unit's).
    std::optional<Span> effective_stroke();
    /// The power-up baud rate (BRSL 8.3 / CON No.16), nullopt when not readable.
    std::optional<int> read_boot_baud_rate();
    /// Alarm history, newest first.
    std::vector<AlarmRecord> read_alarm_history(bool include_empty = false);
    /// One stored row.
    PositionEntry read_position_table(int row);

    // ---------------------------------------------------------------- control
    /// PMSL ON at rest; with verify, the status showing PMSS (EcNotReadyError when it stays 0). nullopt without verify.
    std::optional<Status> enable_modbus_control(bool verify = true);
    /// PMSL OFF: command authority back to PIO.
    void disable_modbus_control();
    /// Servo on / off. A servo ON the status shows cannot work (EMG input, drive power cut, enable switch released) is
    /// refused before anything is sent: ServoBlockedError with reasons(). timeout default kServoTimeoutS (a CON unit's
    /// first servo ON: what its parameters give, at most kServoFirstOnMaxS). nullopt when !wait.
    std::optional<Status> servo(bool on, bool wait = true, std::optional<double> timeout_s = std::nullopt);
    /// Home return (coil edge, or 0x45 cmd 0x0A with the unit's ORG). Waits for HEND and standstill when wait.
    std::optional<Status> home(bool wait = true, std::optional<double> timeout_s = std::nullopt,
                               std::optional<CommandMethod> method = std::nullopt);
    /// Clear the current alarm; returns the new status. A servo the alarm switched off stays off: its servo-on level
    /// (SON coil / DRG1 b12) is cleared first, since the reset alone re-energises the motor (ON-WIRE on a PCON-CB,
    /// 2026-11-03); call servo(true) afterwards. A servo that is on stays on.
    Status reset_alarm(std::optional<CommandMethod> method = std::nullopt);

    // ---------------------------------------------------------------- motion
    /// Every check of move_to (target, speed, accel, band against limits(); the axis ready to move), nothing sent.
    /// Returns the status read; throws what move_to would (SoftLimitError, EcNotReadyError, EcAlarmError).
    Status check_move(double position, const MoveOptions& options = {});
    /// Absolute move (0x9900). The final status when options.wait, else nullopt right after the command.
    std::optional<Status> move_to(double position, const MoveOptions& options = {});
    /// Absolute move at `speed` (waits).
    std::optional<Status> move_to(double position, double speed, double accel_g = 0.1, double band = 0.1);
    /// Relative to the measured position (sent as an absolute target).
    std::optional<Status> move_by(double delta, const MoveOptions& options = {});
    /// A small relative step at a low default speed (10 units/s): limits().jog_step.
    std::optional<Status> jog(double delta, const MoveOptions& options = MoveOptions::jog());
    /// Run stored row `row` (FC06 0x9800, or 0x45 cmd 6 with speed_pct). The row is read and checked first (stroke,
    /// caps, incremental, checksum); Modbus control is taken at rest (an EC ignores 0x9800 while PMSS = 0); no visible
    /// start within kStoredStartS -> decel stop + EcNotReadyError / MotionIncompleteError. On an EC only the rows its
    /// position mode uses (P9): a row it does not run (row 2 in 2-position mode) is refused with SoftLimitError,
    /// nothing sent (the EC-GRBP8M gripper rejected it with 0x45 cmd 6 code 0x11; VERIFIED in the Python edition). When
    /// P9 is not known the row is sent, and a 0x11 rejection of cmd 6 is thrown as that SoftLimitError (the rejection
    /// nested: std::rethrow_if_nested).
    std::optional<Status> move_to_position(int row, const PositionMoveOptions& options = {});
    /// Block until in position and still (the safe rule), optionally at `target` within `band`.
    Status wait_idle(std::optional<double> target = std::nullopt, std::optional<double> band = std::nullopt,
                     double timeout_s = 10.0);
    /// Decel stop (works with or without Modbus control). Blocks for the deceleration; with wait, MotionTimeoutError
    /// when still moving after timeout_s (otherwise a warning is logged). Returns the last status read. Its frames are
    /// stop-class (bus_lock.hpp): they go before every waiting request of the port, so the STOP waits for the one
    /// transaction in flight at most, whatever polls the port. pause(), begin_stop() and servo(false) too.
    Status stop(bool wait = false, double timeout_s = 5.0);
    /// Only the first frame of stop() (STOP ON) and return at once (Cell::stop_all sends it to every axis first, then
    /// calls stop() on each). Always followed by stop() or close().
    void begin_stop();
    /// End every blocking wait of this axis running now in another thread (a move, jog, home, push, stored row,
    /// wait_idle, MoveHandle::wait, the servo wait, Cell waits on this axis): it throws WaitCancelledError within
    /// kBusWaitSliceS plus at most one request on the wire, a motion wait after sending a decel stop. Calls started
    /// afterwards are not affected. Sends nothing itself; safe from any thread (a binding's Ctrl+C handler; cancel.hpp).
    void cancel_waits();
    /// (ABI 0.3) The cancel latch (CancelToken::set_latch): on = cancel_waits() now, and every wait of this axis that
    /// starts while it is on ends the same way at its first check point; off clears it. Safe from any thread.
    void set_cancel_latch(bool on);
    bool cancel_latched() const;
    /// Hold the move and keep the remaining travel (STP ON).
    void pause();
    /// Release the pause; a paused move continues by itself (STP OFF).
    void resume();
    /// Drop the remaining travel of a paused move and release the pause: waits for standstill, an ALRS edge with STP
    /// held, STP off. Refused (nothing sent) when no pause is active (EcNotReadyError) or a fault alarm is active
    /// (EcAlarmError). VERIFIED on the EC gripper in the Python edition.
    Status cancel_paused_move(double timeout_s = 5.0);
    /// Change the speed of the 0x9900 move in progress by re-sending the full block this object sent last with the new
    /// VCMD (same target, band, CTLF). Refused (nothing sent) with no move of this object running, while paused, or
    /// outside limits(). With wait, waits for the move (wait_idle).
    std::optional<Status> change_speed(double speed, bool wait = false, std::optional<double> timeout_s = std::nullopt);

    // ---------------------------------------------------------------- motion extras (src/axis_motion_extras.cpp)
    /// Push move: approach `target` at options.speed, then push over `band` at force_pct (PPOW = floor(pct x 255 /
    /// 100)) up to min(max_push_pct, MXPP 7.9); below MNPP 7.58 (or 20 %) allowed with a warning. Refused (nothing
    /// sent) when the unit cannot push, the band end is outside the stroke, the axis is moving, or it stands inside or
    /// past the push zone (the unit would skip the approach). Outcome contact / missed once it has held for
    /// push_settle_s with the position unchanged. nullopt when !options.wait.
    std::optional<PushResult> push(double target, double band, double force_pct, const PushOptions& options = {});
    /// (push speed in units/s, source): min(speed, PSHV 8.15, MXPH 7.8); kPushSpeedMaxMmS when neither is readable.
    std::pair<double, std::string> push_speed(std::optional<double> speed = std::nullopt);
    /// change_speed for the acceleration (= deceleration): ACMD in the re-sent 0x9900 block.
    std::optional<Status> change_accel(double accel_g, bool wait = false, std::optional<double> timeout_s = std::nullopt);
    /// Absolute moves through `points` in order (each a full 0x9900 block); every point is checked before the first
    /// frame (SoftLimitError names the point). One status per point; see SequenceOptions::blend_band.
    std::vector<Status> move_sequence(const std::vector<SequencePoint>& points, const SequenceOptions& options = {});

    // ---------------------------------------------------------------- position table (non-volatile)
    /// Write stored row `row` (16 words incl. the checksum) and verify it. At rest; rows 256-267 and a row the table
    /// cannot address are refused (NonVolatileWriteError); values inside the stroke / caps and the push range. Written
    /// with a warning (logged and in result.warnings): a row past max_positions(), on an EC a row its position mode
    /// (P9) does not run (row 2 in 2-position mode), an UNVERIFIED row layout, a push on the middle row of a
    /// 3-position EC, unknown flag bits or a change of the role bits 0x0030. Non-volatile guard: when the row already
    /// holds the 16 words nothing is written (skipped, kNvUnchangedNote) unless force. That check comes before the
    /// stroke / speed / accel / push checks: they only matter for a real write (a row stored at 53 mm/s is skipped on
    /// an axis opened with max_speed_mm_s 10); with force every check applies. A refused value names the row, the
    /// field, its unit and whose limit it breaks.
    PositionWriteResult write_position_table(int row, const PositionEntry& entry, const NvWriteOptions& options = {});
    /// Store the current position as row `row` (read the row, replace position_mm, fill empty speed / band / accel
    /// with the unit's defaults, write_position_table). Refused before homing, while moving and for a row outside
    /// 0 .. max_positions() - 1; a row the EC's position mode does not use is taught with a warning. The guard applies
    /// (a same-value teach is skipped before the caps).
    PositionWriteResult teach_position(int row, const NvWriteOptions& options = {});
    /// teach_position by `method`: Table (the above, the VERIFIED pieces) or Coil (EXPERIMENTAL; DOC for CON units,
    /// ME0162-12A p.5-97; UNVERIFIED on an EC): MOD on 01 05 04 14 FF 00, POSR = row 01 06 0D 03 00 nn, TEAC on
    /// 01 05 04 15 FF 00, wait for DSS2 b10 PWRT (VerificationError after kTeachWriteS), TEAC off, MOD off; needs coils
    /// (LineRuleError otherwise, nothing sent). Same refusals and warnings as the table method; nothing is written when
    /// the row is complete and already holds the position (skipped) unless force. Returns the row read back.
    PositionWriteResult teach_position(int row, TeachMethod method, const NvWriteOptions& options = {});

    // ---------------------------------------------------------------- parameters
    /// The unit's named parameters, decoded (EC: P1-P18; CON: No.x). READ-ONLY.
    std::vector<ParameterInfo> parameters();
    /// One parameter: a number ("2", "P2", "No.35") or a raw item ("8.15"). READ-ONLY.
    ParameterInfo read_parameter(const std::string& key);
    ParameterInfo read_parameter(int number);
    /// Write one parameter, verify it by read-back and (commit) save it with PARW and its handshake (VERIFIED in the
    /// Python edition on the EC and the PCON; applies after software_reset() or a power cycle). Guards: see
    /// parameters.hpp; the axis must be at rest. Non-volatile guard: with commit and the value unchanged nothing is
    /// written or committed (skipped) unless this axis has uncommitted RAM writes (then only PARW runs) or force.
    ParameterWriteResult write_parameter(const std::string& key, const ParamValue& value,
                                         const ParameterWriteOptions& options = {});
    ParameterWriteResult write_parameter(int number, double value, const ParameterWriteOptions& options = {});
    /// This unit's parameter list (the IA-OS list for its unit code, or every mapped item found on it).
    ParameterModel parameter_model();

    // ---------------------------------------------------------------- non-volatile write guard
    /// Non-volatile write cycles this Axis object caused (parameter commits + position-table writes).
    long nonvolatile_writes() const;
    /// Of those, the ones in the last kNvWindowS.
    int nonvolatile_writes_last_hour() const;
    /// A write_parameter changed a value that no confirmed commit has saved yet.
    bool has_uncommitted_parameters() const;
    /// (0.5) Share the guard's bookkeeping (the counter, the hour window and its warning, the parameters written to RAM
    /// and not committed) with every Axis of this process given the same `unit_key` (the Python edition keeps it per
    /// unit: a reconnect or a new object per job keeps counting); `label` names the axis in the warning ("COM3 slave
    /// 1"). An empty key gives this axis a fresh bookkeeping of its own.
    void share_nonvolatile_state(const std::string& unit_key, const std::string& label);
    /// (0.5) Drop the shared bookkeeping of every unit key starting with `prefix` ("" = all), as a new process starts.
    static void forget_nonvolatile_state(const std::string& unit_key_prefix);
    /// (0.7) Test hook of the guard (iad_testing_axis_nv_record): count one non-volatile write `what` at `now` (seconds
    /// on now_s()'s clock) as a real write is counted: the total, the hour window and its warning. True when it warned.
    /// Nothing is sent; never needed in production.
    bool testing_note_nonvolatile_write(double now, const std::string& what);
    /// (0.7) Test hook (iad_testing_axis_nv_last_hour): the writes of the kNvWindowS before `now` (now_s()'s clock).
    /// Changes nothing: a query for a later time does not drop the counts of the time before it.
    int testing_nonvolatile_writes_last_hour(double now) const;

    // ---------------------------------------------------------------- unit
    /// Restart the unit's firmware (0x45 cmd 5 op1 1 after servo off; at rest only), wait until it answers again
    /// (reset_wait_s, then probes until timeout_s; the rate re-detected if it changed), confirm the restart. Clears the
    /// uncommitted-parameter flag (the unit reloads its parameters).
    SoftwareResetResult software_reset(const SoftwareResetOptions& options = {});
    /// Change the unit's and the port's baud rate (volatile: until power-off). At rest only, and only when this axis
    /// is the only one using the bus (use Port::set_baud_rate for a shared line). SoftLimitError (nothing sent) for a
    /// rate without an IAI code; an UNVERIFIED code is sent with a warning (result.code_verified false);
    /// VerificationError when the unit answers at another rate afterwards (the port is left at the rate that answered).
    BaudChangeResult set_baud_rate(int rate, const BaudChangeOptions& options = {});

    // ---------------------------------------------------------------- fast path (src/axis_fast.cpp; monitor.hpp)
    /// connect(take_control), then with `fast` enable_fast() (idempotent).
    DeviceInfo connect(bool take_control, bool fast);
    /// The lean status: one read of 0x9000-0x9007 (8 words: PNOW, ALMC, DIPM, DOPM, DSS1, DSS2, DSSE), the smallest
    /// range with position, alarm code, DSS1 and DSSE. Flags from STAT, ZONS, POSS, SSSE are nullopt; velocity_mm_s is
    /// nullopt while moving. Used by the monitor and the fast completion. READ-ONLY.
    Status read_lean_status(std::optional<int> retries = std::nullopt);
    /// wait_idle by `completion`: Completion::Fast takes the lean block, one sample (fast_done), no settle window.
    /// With the status monitor running the samples come from it; the Windows timer runs at 1 ms meanwhile.
    Status wait_idle(std::optional<double> target, std::optional<double> band, double timeout_s, Completion completion);
    /// move_to without waiting: the command is sent (every check has passed) and a MoveHandle returned
    /// (options.wait is ignored). With the monitor running a status younger than kFreshS that shows the axis ready
    /// replaces the status read before the command (the move is then one frame).
    MoveHandle start_move_to(double position, const MoveOptions& options = {});
    /// move_by without waiting (the measured position: a monitor status younger than kFreshS when it runs).
    MoveHandle start_move_by(double delta, const MoveOptions& options = {});
    /// jog without waiting.
    MoveHandle start_jog(double delta, const MoveOptions& options = MoveOptions::jog());
    /// Start the background status monitor (AxisMonitor). Already running: the callbacks in `options` are added and
    /// the running monitor is returned. Without options a monitor that ended on an error (port gone) is restarted with
    /// its callbacks. Read-only; stopped by stop_monitor() and close(). std::invalid_argument for a bad option.
    std::shared_ptr<AxisMonitor> start_monitor(std::optional<MonitorOptions> options = std::nullopt);
    /// Stop the status monitor (no-op without one); monitor() is null afterwards.
    void stop_monitor(double timeout_s = 1.0);
    /// The status monitor of start_monitor() (null when none was started).
    std::shared_ptr<AxisMonitor> monitor() const;
    /// The fast path in one call: Completion::Fast, the status monitor on (with `monitor`) and the link raised to
    /// `baud_rate` (230400 by default, VERIFIED on the EC-R6S rod in the Python edition; nullopt keeps the rate). The
    /// baud change is set_baud_rate (a broadcast; volatile, 38400 again after a power cycle), tried only when this
    /// axis owns the port (it opened it, or set_port_owner), the transport has a baud rate and the axis is at rest; otherwise, or when the unit does
    /// not take it, the rate is kept and the reason is in notes (also fast_notes()). Safe to call again.
    FastModeSummary enable_fast(std::optional<int> baud_rate = kFastBaudRate, bool monitor = true);
    /// Undo enable_fast: Completion::Safe again and the status monitor stopped. The baud rate is kept.
    FastModeSummary disable_fast();
    /// enable_fast() ran (and no disable_fast() since).
    bool fast_mode() const;
    std::vector<std::string> fast_notes() const;

    // ---------------------------------------------------------------- command strategy, line, modes (src/axis_strategy.cpp)
    /// The command strategy in use for servo / home / alarm reset: set_strategy(); else AxisOptions::method when it is
    /// not Coil; else the line's (Register behind an RCON gateway, DOC ME0413 p.5-11); else Coil. A Coil strategy
    /// becomes Register by itself on a line without FC05 or when the unit answers an FC05 write with exception 01; an
    /// IAI strategy on a line without 0x45 / 0x48 becomes Coil (or Register). Port of the Python edition's strategies.
    CommandMethod strategy() const;
    /// Choose the strategy explicitly (no I/O; Iai2 logs an EXPERIMENTAL warning). set_strategy(Coil) keeps the coil
    /// strategy on a line that names another (it still adapts when the line carries no coils).
    void set_strategy(CommandMethod method);
    /// "coil" (the VERIFIED STOP coil sequence) or "register": the coil-free substitute used with the Register strategy,
    /// on a line without coils (RCON) or after an FC05 exception 01: DRG1 pause 01 06 0D 00 10 20, hold until
    /// standstill, an ALRS edge while paused 11 20 / 10 20 (drops the remaining travel: the old target is not resumed;
    /// DOC ME0384 p.6-76, ME0162-12A p.4-11, 5-85), release 10 00 (SON / SFTY kept). UNVERIFIED on hardware. With a
    /// fault alarm active the ALRS edge is left out (it would clear the alarm) and only the pause is released.
    std::string stop_path() const;
    /// The line kind ("rcon", "robonet", "robonet-gateway"), nullopt on a direct line.
    std::optional<std::string> line_name() const;
    /// The action mode: pc = PC / Modbus control (IA-OS "Teaching", PMSS = 1), false = PIO ("Monitoring", PMSS = 0).
    /// safety nullopt keeps the present safety speed (DSS1 SFTY read first). PIO is 0x45 cmd 0x0C op1 = 0 (VERIFIED on
    /// the EC-R6S rod in the Python edition, (0,0); (0,1) UNVERIFIED). PC is the PMSL coil (only when PMSS is 0) and, when
    /// the safety speed must change, the SFTY coil (DRG1 b14 on the register path): 0x45 cmd 0x0C op1 = 1 left a
    /// PCON-CB refusing servo ON (VERIFIED, 2026-11-02); a unit without coils still gets op1 = 1. Refused while the axis
    /// moves, pauses, homes or pushes (EcNotReadyError; taking PC control from PIO allows a pause at a standstill with
    /// the servo off) and on a line without 0x45 (LineRuleError), nothing sent. verify: PMSS (and SFTY when pc) read
    /// back, VerificationError otherwise.
    Status set_action_mode(bool pc = true, std::optional<bool> safety = std::nullopt, bool verify = true);
    /// Switch the safety-speed mode (~100 mm/s cap when on; volatile): the SFTY coil 0x0401 (VERIFIED on the EC-DS4H
    /// slider in the Python edition), or DRG1 b14 SFTY with FC06 on the register path (SON kept; DOC ME0162-12A
    /// p.5-124, UNVERIFIED). Takes Modbus control first if needed (at rest only). verify: VerificationError when DSS1
    /// b14 does not follow. The driver never changes the safety speed by itself.
    Status set_safety_speed(bool on, bool verify = true);

    // ---------------------------------------------------------------- home pieces, caps (src/axis_accessors.cpp)
    /// Release what a home start left set (the HOME coil of the coil strategy, the DRG1 HOME bit of the register
    /// path; nothing with the IAI strategies): needed after home(wait = false). Never cancelled.
    void finish_home();
    /// The unit's home pattern ORG (item 9.16 by item ID; 7 on the three ECs tested in the Python edition); the 0x45
    /// home takes op1 = ORG & 0xFFFF (VERIFIED on the EC-R6S rod). nullopt when the item is not in the directory.
    /// READ-ONLY.
    std::optional<std::uint32_t> read_home_pattern();
    /// (documented minimum speed in units/s, source): per EC series and lead, 5 / 10 mm/s on grippers and wire types,
    /// 20 deg/s on rotaries, else lead x 1.25 (DOC ME0155 p.139). Below it the driver warns, never refuses. nullopt
    /// with the reason when nothing tells. READ-ONLY.
    std::pair<std::optional<double>, std::string> recommended_min_speed();
    /// Highest speed the driver commands (units/s): min(max_speed_mm_s, MAXV 7.4), each only when known.
    double speed_cap();
    /// Highest acceleration (= deceleration on a 0x9900 move): min(max_accel_g, MAXA, MAXD), each only when known.
    double accel_cap();
    /// Highest acceleration of a move with its own deceleration (the command-profile path): min(max_accel_g, MAXA).
    double accel_only_cap();
    /// Highest deceleration of a move with its own deceleration: min(max_accel_g, MAXD).
    double decel_cap();
    /// Highest push force (%): min(max_push_pct, MXPP 7.9), each only when known.
    double push_cap();

    // ---------------------------------------------------------------- 0.5: state and reads of the bindings (src/axis_state.cpp)
    /// The facts read so far and the command state (AxisState). No I/O.
    AxisState state() const;
    /// The status samples this axis decoded last (status(), the samples of its waits, the lean reads of the status
    /// monitor), newest first, at most kRecentStatuses: a binding finds the raw words of a status it was handed. No I/O.
    std::vector<Status> recent_statuses() const;
    static constexpr std::size_t kRecentStatuses = 16;
    /// The unit's speed cap: MAXV 7.4, else the default speed 8.10 (read_axis_limits()); nullopt with the reason in
    /// state().unit_max_speed_note. READ-ONLY.
    std::optional<double> read_speed_limit();
    /// Class 1 support words by name (SUPT, DIAG, MVPT ... SUP3); nullopt when the class is not in the directory or not
    /// readable. READ-ONLY.
    std::optional<std::map<std::string, std::uint32_t>> read_support_words();
    /// The directory records (index table at 0x0000), read once. READ-ONLY.
    std::vector<IndexCategory> categories();
    /// What close() does to the unit, keeping this object usable: the status monitors stop and a STOP or pause this
    /// axis holds is released (a failed release is logged). The Python edition's Elecylinder.close keeps what the
    /// object read; the bindings close the port themselves.
    void release_held();
    /// Replace the unit's limits this axis keeps (read_axis_limits()'s; the Python edition's settable axis_limits):
    /// every later check uses them until the next read_axis_limits(). No I/O.
    void set_axis_limits(const AxisLimits& limits);
    /// (seconds, why) to wait for a servo ON (the servo() default): 3 s, or on a CON unit before its first servo ON
    /// after power-up 0.006 + (T1 + No.29) x 10 + 0.020 s + 1 s (DOC PCON p.4-24). `status` nullopt: read it.
    std::pair<double, std::string> servo_on_timeout(const std::optional<Status>& status = std::nullopt);
    /// The EC user parameters of this unit's list, decoded and annotated as the Python edition's
    /// Elecylinder.user_parameters (info.hpp user_parameter_rows). READ-ONLY.
    Json user_parameters();
    /// This unit's parameter list from the unit code and the model string only (FC11 + 0x0600; visibility false, the
    /// Python edition's parameter_model()), or with the unit's runtime visibility rules applied (classes 3, 7, 8, 9,
    /// 0xC, the support words, ACCL: what the parameter writes use; parameter_model() is that one). READ-ONLY.
    ParameterModel parameter_model(bool visibility);

private:
    friend class AxisMonitor;
    friend class MoveHandle;
    friend struct detail::AxisAccess;
    struct Impl;
    std::unique_ptr<Impl> impl_;
    /// The MoveHandle of the move just sent with options `o` (axis_fast.cpp).
    MoveHandle started_handle(const MoveOptions& o, double target);
};

}  // namespace iai_actuator_driver
