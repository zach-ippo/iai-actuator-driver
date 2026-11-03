// Private to the axis unit: the state an Axis keeps (shared by src/axis_core.cpp, axis_motion.cpp, axis_config.cpp,
// axis_nv.cpp, cell.cpp and the CLI) and the helpers of its waits. Nothing outside the axis unit includes this header.
//
// Axis::Impl does the work; the public Axis methods forward to it, so internal paths (servo off while moving calls
// stop(), teach calls write_position_table, ...) call the same code as the public API.
#pragma once

#include <atomic>
#include <cmath>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {
namespace detail {

/// Log source of the axis.
inline constexpr const char* kAxisLog = "iai_actuator_driver.axis";
/// Why a Modbus STOP is not offered for a move a PLC commanded.
inline constexpr const char* kPioStopHint =
    "stop() stopped a Modbus-commanded move with PMSS = 0 on hardware (VERIFIED in the Python edition), but is "
    "untested on a move a PLC commanded over PIO: stop that from the PLC or with the hardware stop";
/// Hardware test N4 of the Python edition.
inline constexpr const char* kN4Note =
    "the EC silently ignores 0x9800 position-number moves while PMSS = 0 (VERIFIED in the Python edition, test N4)";
/// Float noise allowed on positions (they are 0.01 counts).
inline constexpr double kPosEps = 1e-6;
/// Accel / decel time limit with EC P5 smooth acceleration on (DOC ME3816-6B p.6-11).
inline constexpr double kSmoothAccelMaxTimeS = 2.0;
/// After the software reset command: probe this often to see the unit go silent.
inline constexpr double kResetGapProbeS = 0.05;
/// The default command speed fallback when the unit's default speed is unknown (units/s).
inline constexpr double kTeachFallbackSpeed = kFallbackSpeedCap;
/// Automatic servo OFF: the servo must come back within this after SON (else a SON off / on cycle).
inline constexpr double kAsofWakeS = 1.5;
/// 0x45 cmd 6 (a stored row at a % of its speed) answered 0x11 "1st operand error": the unit does not run row op1
/// (VERIFIED in the Python edition on the EC-GRBP8M gripper in 2-position mode, P9 = 0: row 2 rejected, nothing moved).
inline constexpr int kRejectRowOperand = 0x11;
/// What the warning of a row write / teach to a row the EC's position mode does not run adds: it is written.
inline constexpr const char* kRowWrittenAnyway = "written anyway (e.g. to prepare a switch to 3-position mode)";

/// "1.00..150.00 mm", or `missing`.
std::string fmt_range(const std::optional<Span>& range, const std::string& units,
                      const std::string& missing = "unknown");

/// Has a condition held, with PNOW unchanged, for `hold` seconds? (PEND can rise before standstill.) Fed one poll at a
/// time; the window restarts whenever the condition drops, its key changes (a push outcome) or the position moves more
/// than the tolerance from where the window started. A hold of 0 accepts the first poll. Port of C# Settle / Python
/// _Settle.
class Settle {
public:
    explicit Settle(double hold_s, double tolerance = kSamePositionMm) : hold_(hold_s), tolerance_(tolerance) {}

    bool update(bool condition, double position, int key = 0, double now = now_s()) {
        if (!condition) {
            started_ = false;
            return false;
        }
        if (!started_ || key != key_ || std::fabs(position - position_) > tolerance_) {
            started_ = true;
            time_ = now;
            position_ = position;
            key_ = key;
            return hold_ <= 0.0;
        }
        return now - time_ >= hold_;
    }

private:
    double hold_;
    double tolerance_;
    bool started_ = false;
    double time_ = 0.0;
    double position_ = 0.0;
    int key_ = 0;
};

/// The non-volatile write counter of one unit, its one-hour window and its parameters written to RAM and not yet
/// committed (results.hpp). Each Axis has its own until Axis::share_nonvolatile_state gives it the one every axis of
/// the process on the same unit shares (0.5; the Python edition's per-unit bookkeeping). Thread-safe. Implemented in
/// src/axis_nv.cpp.
class NvWriteTracker {
public:
    explicit NvWriteTracker(std::string label = std::string());
    /// Count one non-volatile write cycle at `now`; above kNvWarnPerHour in the window log kNvRateWarning (source
    /// "iai_actuator_driver.axis", once per kNvWindowS). Returns true when it warned.
    /// The stamps are kept in time order and only those more than kNvWindowS before the newest one are dropped, so a
    /// stamp given out of order (a test) is neither lost nor leaves the list unsorted.
    bool record(double now = now_s());
    long total() const;
    /// The writes of the window ending at `now`; forgets the stamps older than it (the library's own time only moves on).
    int last_hour(double now = now_s()) const;
    /// The same count for any `now` (a test's), changing nothing: the writes not older than kNvWindowS before it. A query
    /// for a later time keeps the real counts.
    int count_in_window(double now) const;
    void set_label(std::string label);
    /// A parameter was written to RAM and no confirmed commit (or restart) has saved it since.
    bool uncommitted() const { return uncommitted_.load(); }
    void set_uncommitted(bool on) { uncommitted_.store(on); }

private:
    void prune(double now) const;
    int count_in_window_locked(double now) const;

    mutable std::mutex mutex_;
    mutable std::deque<double> stamps_;  // in time order
    long total_ = 0;
    double last_warning_ = -1e300;
    std::string label_;
    std::atomic<bool> uncommitted_{false};
};

/// (0.5) The time limits a test may shorten (detail::set_test_timing in axis.hpp, the Python edition's
/// testing.timings): a home return must visibly start within home_start_s (kHomeStartS), the parameter handshake must
/// be idle within parw_ready_s (kParwReadyS) and end within parw_clear_s (kParwClearS) after the PARW coil.
/// Process-wide; never set by the library itself. Implemented in src/axis_state.cpp.
double home_start_s();
double parw_ready_s();
double parw_clear_s();

/// (0.5) The tracker every axis of this process given `key` shares (made on first use, labelled `label` in its
/// warning); forget_nv_trackers drops the keys starting with `prefix` ("" = all), as a new process starts without them.
std::shared_ptr<NvWriteTracker> shared_nv_tracker(const std::string& key, const std::string& label);
void forget_nv_trackers(const std::string& prefix);

/// Trapezoid time estimate of a move (s): distance at `speed` with `accel_g` up and down.
double motion_time_s(double distance, double speed, double accel_g);
/// Default timeout of a move: 2 x motion_time_s + 2 s.
inline double default_move_timeout_s(double distance, double speed, double accel_g) {
    return 2.0 * motion_time_s(distance, speed, accel_g) + 2.0;
}
/// The timeout of a command that starts motion or a servo change (move, home, servo, stored row, push, sequence, change
/// of the running move): nullopt is the default; a value must be finite and above 0, else SoftLimitError before
/// anything is sent (0 or -1 is "wait forever" in some languages: refused, never a move cut short at once).
/// Implemented in src/axis_motion.cpp.
void check_timeout(const std::optional<double>& timeout_s, const std::string& what = "timeout_s");

/// The class name of a driver error ("SoftLimitError", "ModbusTimeout", ...; "invalid_argument" etc. for std errors).
std::string error_type_name(const std::exception& e);
/// "port_vanished" (TransportError), "unit_silent" (no valid answer after the retries: timeout, CRC, frame) or "".
std::string link_fault_kind(const std::exception& e);
/// "<Type>: <what>" of the exception in `error` ("unknown error" for a non-standard one).
std::string describe(const std::exception_ptr& error);
/// Add a note to a driver error (and log it at info level).
void add_note(EcDriverError* e, const std::string& note);

/// At rest for taking Modbus control from PIO (PMSL ON, the action mode PC): idle(), or paused (DSS1 STP) with the servo
/// off while standing still: no MOVE, no home return, no push, VNOW read and 0. A PCON in PIO mode with nothing on its PIO
/// connector turns the servo off and shows STP at rest (the unwired *STP input; VERIFIED on the PCON-CB, 2026-11-02);
/// 1.0.1 taking control there: VERIFIED (PCON-CB, 2026-11-02). A pause with the servo on is a hold, for example a PLC
/// keeping *STP on a move: taking control would cancel it, so it stays a refusal. A PLC pause with the servo off is
/// still indistinguishable from an unwired *STP. Handing control back to PIO keeps idle().
inline bool at_rest_for_control(const Status& st) {
    if (st.idle()) return true;
    return st.paused() && !st.servo_on && !st.moving && !st.homing() && !st.pushing() && st.raw.vnow &&
           *st.raw.vnow == 0;
}

/// Why a paused axis standing still is not at rest for a command, with the way out; "" when it is moving, pushing,
/// homing or not paused (the callers keep their own text). In PIO mode (PMSS 0) the pause is the PIO's: a PCON with
/// nothing on its PIO connector shows STP at rest (VERIFIED on a PCON-CB, 2026-11-03); resume() would not help there.
/// pc_helps: taking PC control is a way out too (only with the servo off, as at_rest_for_control).
inline std::string pause_blocker(const Status& st, bool pc_helps) {
    if (!st.paused() || st.moving || st.homing() || st.pushing()) return std::string();
    if (st.modbus_enabled) return "the axis is paused (DSS1 STP on): resume() or stop() it first";
    return std::string("the unit is in PIO mode (PMSS 0) and the PIO holds the pause (DSS1 STP on): ") +
           (pc_helps && !st.servo_on ? "take PC control first (set_action_mode) or release *STP on the PIO connector"
                                     : "release *STP on the PIO connector first");
}

/// The hook a coordinated run sets on each of its axes (coord.hpp, src/coord_run.cpp): called by Axis::Impl::note_stop
/// before a STOP, pause or servo OFF goes to the axis, so that none of the run's frames follows it.
struct StopHook {
    virtual ~StopHook() = default;
    virtual void external_stop() = 0;
};

}  // namespace detail

/// The state behind an Axis, and the work. Fields marked (mutex) are guarded by `mutex`; the atomics and `nv` guard
/// themselves; `bus`, `slave`, `owns_bus`, `on_port` are fixed.
struct Axis::Impl {
    Impl(std::shared_ptr<ModbusRtu> bus, int slave, AxisOptions options, bool owns_bus, bool on_port);

    std::shared_ptr<ModbusRtu> bus;
    int slave = 1;
    bool owns_bus = false;
    bool on_port = false;             ///< created on a Port (a shared line): set_baud_rate is refused
    std::atomic<bool> port_owner{false};  ///< (0.5, py-monitor) Axis::set_port_owner: enable_fast may change the rate
    std::atomic<bool> baud_detected{false};
    std::atomic<bool> closed{false};
    std::unique_ptr<UnitReader> reader;  ///< bus->reader(slave)

    mutable std::mutex mutex;
    AxisOptions options;              ///< (mutex)

    // facts read from the unit (mutex; cached; cleared by software_reset where they can change)
    std::optional<UnitDirectory> directory;
    std::optional<DeviceInfo> info;
    std::optional<AxisLimits> limits;
    SoftLimits soft;
    bool soft_known = false;
    std::optional<int> max_positions_cache;
    std::optional<double> p1_ceiling_mm;
    int status_words = reg::kStatusReadWords;     ///< 22, lowered when the unit refuses the long read
    bool status_size_checked = false;             ///< (0.5) the read size was checked against the directory once
    std::optional<std::pair<double, double>> last_sample;  ///< (time, position) for the derived velocity
    std::deque<Status> recent_statuses;           ///< (0.5) the last decoded samples, newest first (Axis::recent_statuses)
    std::optional<int> system_code;
    std::optional<int> unit_code;
    bool unit_code_known = false;
    std::optional<std::string> model_name;
    bool model_name_known = false;
    std::optional<int> boot_baud_rate;
    bool ctlf_item_known = false;               ///< EC item 8.14 read (P5 smooth acceleration, change_speed)
    std::optional<std::uint32_t> ctlf_item;

    // command state
    /// The stop mark of this unit now (modbus.hpp detail::SendGate): a command takes it before its checks and opens
    /// its SendGate with it, so a STOP from elsewhere after that refuses the command's frames (nothing sent).
    detail::StopMark decision_mark() const { return detail::stop_mark(*bus, slave); }
    /// A stop sequence of this object in progress (stop_sequence, try_stop and their register forms). While one runs
    /// in another thread, release_stale_stop refuses (nothing sent); the last one to end releases the STOP / pause, so
    /// no thread releases a STOP another thread still holds.
    class StopRun {
    public:
        explicit StopRun(Impl& a);
        ~StopRun();
        StopRun(const StopRun&) = delete;
        StopRun& operator=(const StopRun&) = delete;
        /// End this run: true when no other thread's run is left (this one releases what is held).
        bool leave();

    private:
        Impl& a_;
        bool left_ = false;
    };
    mutable std::mutex stop_runs_mutex;
    std::vector<std::thread::id> stop_runs;  ///< (stop_runs_mutex) one entry per StopRun alive
    /// A stop sequence of another thread is in progress on this object.
    bool stop_running_elsewhere() const;
    /// Group stops (Cell::stop_all, a coordinated group's stop) that hold this axis now (detail::GroupStopHold): from
    /// before its first STOP frame (begin_stop) until its full stop() ends, whichever thread sends them. Not keyed by
    /// thread: the first frame and the full stop run on different threads.
    std::atomic<int> group_stops{0};
    /// A STOP another thread still holds: a stop sequence of another thread, or a group stop of this axis.
    bool stop_held_elsewhere() const { return stop_running_elsewhere() || group_stops.load() > 0; }
    /// EcNotReadyError (stop_attempted set; nothing sent) while stop_held_elsewhere(): that STOP is not this
    /// command's to release.
    void refuse_while_stop_held_elsewhere() const;
    std::atomic<bool> stop_coil_on{false};    ///< this object holds STOP ON (released by stop() / close())
    std::atomic<bool> motion_pending{false};  ///< a move of this object may still run
    /// cancel_waits() cancels it: every wait of this axis open then ends (guard() and the servo wait open a scope on
    /// it; cancel.hpp)
    std::shared_ptr<CancelToken> cancel_token = std::make_shared<CancelToken>();
    std::optional<DirectMove> last_move;      ///< (mutex) the last 0x9900 block this object sent (change_speed)
    std::optional<double> last_target;        ///< (mutex)
    std::optional<CommandMethod> home_method; ///< (mutex) the method of the last home start (finish_home)

    // coordinated motion (coord.hpp, src/coord_run.cpp)
    std::atomic<long> stop_count{0};          ///< stops, pauses and servo OFFs sent so far (note_stop)
    std::mutex coord_mutex;                   ///< guards coord_run
    std::shared_ptr<detail::StopHook> coord_run;  ///< (coord_mutex) the coordinated run this axis is part of, else null
    /// A STOP, pause or servo OFF is about to go to this axis: count it and end a coordinated run on it first (the run
    /// sends no frame after it and waits out a frame in flight). Every stop path calls it before its first frame.
    void note_stop() {
        ++stop_count;
        std::shared_ptr<detail::StopHook> run;
        {
            std::lock_guard<std::mutex> lock(coord_mutex);
            run = coord_run;
        }
        if (run) run->external_stop();
    }

    // non-volatile write guard
    std::shared_ptr<detail::NvWriteTracker> nv_tracker = std::make_shared<detail::NvWriteTracker>();  ///< (mutex)
    /// This axis's tracker (its own, or the unit's shared one: Axis::share_nonvolatile_state).
    std::shared_ptr<detail::NvWriteTracker> nv() const {
        std::lock_guard<std::mutex> lock(mutex);
        return nv_tracker;
    }

    // ------------------------------------------------------------------ core (axis_core.cpp)
    UnitReader& rd() { return *reader; }
    AxisOptions opts() const;
    void sleep_poll() const;
    Status status();
    UnitDirectory ensure_directory();
    ClassBlock read_class_block(int class_id);
    std::optional<int> block_address(int category, int fallback);
    std::optional<int> read_system_code();
    std::optional<std::string> read_model_name();
    bool is_ec();
    std::string family();
    DeviceInfo identify();
    SoftLimits read_soft();
    AxisLimits read_limits();
    std::optional<AxisLimits> limits_read();
    void ensure_soft();
    int max_positions();
    RangeInputs cap_inputs();
    /// (0.5) cap_inputs from what is known, no read (the caps a refusal names before anything was read).
    RangeInputs known_cap_inputs() const;
    RangeInputs range_inputs();
    AxisRanges ranges();
    std::pair<std::optional<double>, std::string> recommended_min_speed();
    void warn_min_speed(double speed);
    double speed_cap();
    double accel_cap();
    double push_cap();
    std::optional<Span> stroke_now();
    Span motion_stroke();
    std::string units_label();
    void note_p1(double value);
    std::pair<int, std::string> position_address(int row);
    PositionEntry read_row(int row);
    std::vector<AlarmRecord> alarm_history(bool include_empty);
    std::optional<int> read_boot_baud();
    std::optional<Status> enable_control(bool verify);
    void disable_control();
    void close();
    /// (0.5) close() without closing: the monitors stop, a STOP / pause this axis holds is released (Axis::release_held).
    void release_held();
    std::string describe() const;

    // ------------------------------------------------------------------ motion (axis_motion.cpp)
    /// require_control's refusals (the gateway, auto_modbus_control, at rest, a STOP held elsewhere); nothing sent.
    void check_control(const Status& st);
    Status require_control(Status st);
    /// wake_asof: an automatic servo OFF (STAT ASOF) is not a refusal; the servo is switched back on first
    /// (wake_from_auto_servo_off). false (check_move): accepted, nothing sent. `refusals`: the caller's own checks of
    /// the status (they throw to refuse). With Modbus control off every refusal (these too) runs before auto Modbus
    /// control takes PC control, so a refused command writes nothing, not even PMSL; the pause is checked after it
    /// (taking control ends a PIO pause), and the checks run again on the status after it.
    Status require(Status st, bool servo = true, bool homed = true, bool allow_paused = false, bool wake_asof = true,
                   const std::function<void(const Status&)>& refusals = {});
    void check_during_motion(const Status& st);
    void release_stop();
    void release_stale_stop();
    std::pair<Status, bool> hold_until_stopped(double timeout_s);
    std::pair<Status, bool> stop_sequence(double timeout_s);
    void try_stop(EcDriverError* error, const std::string& when = "after this error");
    /// push_band: the band may be as long as the stroke (a push band), not only kMoveBandMax. accel_only: accel_g is
    /// an acceleration alone (the command-profile path): checked against min(max_accel_g, MAXA), not MAXD.
    DirectMove validate_motion(double position, double speed, double accel_g, double band, bool push_band = false,
                               bool accel_only = false);
    void check_unit_speed(double speed);
    void check_unit_accel(double accel, const std::string& what, bool decel);
    void send_direct(const DirectMove& move);
    std::vector<std::string> change_warnings(double speed, double accel);
    std::optional<Status> servo(bool on, bool wait, std::optional<double> timeout_s);
    std::vector<std::string> servo_blockers(const Status& st, bool after);
    double servo_on_timeout(const Status& st, std::string* why = nullptr);  ///< why: the reason in words (0.5)
    std::optional<std::uint32_t> read_home_pattern();
    std::optional<Status> home(bool wait, std::optional<double> timeout_s, std::optional<CommandMethod> method);
    void finish_home();
    bool home_seen_after_lost_reply(EcDriverError& error, const Status& before);
    Status reset_alarm(std::optional<CommandMethod> method);
    Status check_move(double position, const MoveOptions& options);
    std::optional<Status> move_to(double position, const MoveOptions& options);
    std::optional<Status> move_by(double delta, const MoveOptions& options);
    std::optional<Status> move_to_position(int row, const PositionMoveOptions& options);
    struct StoredRow {
        double target = 0.0;
        double band = 0.0;
        std::optional<Span> zone;  ///< push rows: the push zone
        int direction = 0;
    };
    StoredRow check_stored_row(int row, const PositionEntry& entry, bool allow_above_caps, double speed_scale);
    void wait_stored_start(int row, double start, double target);
    Status wait_push(double target, const Span& zone, double timeout_s);
    Status wait_idle(std::optional<double> target, std::optional<double> band, double timeout_s);
    Status stop(bool wait, double timeout_s);
    void begin_stop();
    void pause();
    void resume();
    Status cancel_paused_move(double timeout_s);
    std::optional<Status> change_speed(double speed, bool wait, std::optional<double> timeout_s);

    // ------------------------------------------------------------------ motion extras (axis_motion_extras.cpp)
    bool mvpt_known = false;                     ///< (mutex) the MVPT support word was read
    std::optional<std::uint32_t> mvpt;           ///< (mutex) MVPT (class 1), nullopt when the unit has none
    std::optional<std::uint32_t> move_patterns();
    std::optional<std::uint32_t> item_raw(int class_id, int item_id);
    /// CTLF b6-b7 of `pattern` for a move at speed / accel_g; SoftLimitError (nothing sent) when refused.
    std::uint16_t accel_pattern_ctlf(std::optional<AccelPattern> pattern, double speed, double accel_g);
    double accel_only_cap();                     ///< min(max_accel_g, MAXA): accel of a move with its own decel
    double decel_cap();                          ///< min(max_accel_g, MAXD)
    struct ProfileMove {
        DirectMove move;                         ///< PCMD / INP / VCMD / ACMD as checked
        std::uint32_t dcmd = 0;
        int address = 0;                         ///< the unit's command-profile block (class 0x0F)
    };
    ProfileMove check_profile_move(double position, double speed, double accel_g, double decel_g, double band);
    void send_profile(const ProfileMove& profile, std::uint16_t ppow, std::uint16_t ctlf);
    std::optional<Status> move_profile(double position, const MoveOptions& options);
    std::pair<double, std::string> push_speed(std::optional<double> speed);
    std::pair<int, std::vector<std::string>> check_push_force(double force_pct);
    std::vector<std::string> auto_servo_off_push_warnings();
    std::optional<PushResult> push(double target, double band, double force_pct, const PushOptions& options);
    std::optional<Status> change_move(std::optional<double> speed, std::optional<double> accel, bool wait,
                                      std::optional<double> timeout_s);
    /// The blend point of move_sequence: |position - target| <= band (no settle, no PEND), samples from `since` on.
    Status wait_band(double target, double band, double timeout_s, double since);
    std::vector<Status> move_sequence(const std::vector<SequencePoint>& points, const SequenceOptions& options);

    /// Send the decel stop for `error`, then throw it (its own type).
    template <class E>
    [[noreturn]] void stop_and_raise(E error) {
        try_stop(&error);
        throw error;
    }

    /// Run `body`; on any exception send a decel stop (unless one was already sent for it), then rethrow. The body is
    /// cancellable (cancel_waits: WaitCancelledError, after the decel stop).
    template <class F>
    auto guard(F&& body) -> decltype(body()) {
        const CancelScope cancel_scope(cancel_token);
        try {
            return body();
        } catch (EcDriverError& e) {
            if (!e.stop_attempted()) try_stop(&e);
            throw;
        } catch (...) {
            try_stop(nullptr);
            throw;
        }
    }

    // ------------------------------------------------------------------ configuration (axis_config.cpp)
    PositionWriteResult write_position_table(int row, const PositionEntry& entry, const NvWriteOptions& options);
    /// The words of `entry` once every value fits the stroke, the caps and the unit's limits (SoftLimitError naming
    /// the row, the field, its unit and whose limit it breaks; "stored" when the value equals `stored`'s). Unknown
    /// control-flag bits that differ from the unit's own are not refused: their warning goes to `warnings`.
    std::vector<std::uint16_t> validate_entry(const PositionEntry& entry, int unit_flags, int unit_push_power,
                                              std::vector<std::string>& warnings,
                                              std::optional<int> row = std::nullopt,
                                              const PositionEntry* stored = nullptr);
    PositionWriteResult teach_position(int row, const NvWriteOptions& options);
    // the stored rows an EC runs (axis_config.cpp; Python _active_position_mode / _check_active_row / _row_rejected)
    bool position_mode_known = false;                 ///< (mutex) P9 read (cleared by a commit and software_reset)
    std::optional<std::uint32_t> position_mode_raw;   ///< (mutex) P9 (EC item 8.2) as read, nullopt when not readable
    /// The position mode an EC runs: P9 0 / 3 = 2-position (rows 0 and 1), 4 = 3-position (rows 0-2).
    struct RowMode {
        int rows = 0;            ///< rows 0 .. rows - 1 are run
        std::uint32_t code = 0;  ///< P9
        std::string name;        ///< "2-position (double solenoid)"
    };
    /// The mode from P9 when it decodes to a known one, else nullopt (the open rule: every row 0 .. max_positions() - 1
    /// offered, the unit's own answer decides): a CON unit, a unit whose family is not known, P9 not readable or an
    /// unknown value. READ-ONLY; P9 is read once and cached.
    std::optional<RowMode> active_position_mode();
    /// "row 2 (middle)" on an EC (rows 0-2 have roles), else "row 7".
    std::string row_label(int row);
    /// "row 2 (middle) is not used in 2-position mode (P9 = 0: 2-position (double solenoid)): the unit runs row 0
    /// (backward end) and row 1 (forward end) only"
    std::string inactive_row_text(int row, const RowMode& mode);
    /// SoftLimitError "<row> is not used in N-position mode ...; <consequence>" (before anything is sent) when the
    /// active position mode does not run `row`; nothing is refused when the mode is not known.
    void check_active_row(int row, const std::string& consequence);
    /// A row write / teach to a row the active position mode does not run is written: "<row> is not used in
    /// N-position mode ...; written anyway ..." for its warning, "" when the mode runs it or is not known.
    std::string inactive_row_warning(int row);
    /// The 0x11 rejection of 0x45 cmd 6 for `row` in words: the SoftLimitError to throw in its place.
    SoftLimitError row_rejected(int row, const IaiCommandRejected& rejection);
    ParameterContext parameter_context(std::optional<double> position_mm);
    std::vector<ParameterInfo> parameters();
    ParameterInfo read_parameter(const ParameterKey& key);
    ParameterWriteResult write_parameter(const ParameterKey& key, const ParamValue& value,
                                         const ParameterWriteOptions& options);
    ParameterModel parameter_model();
    std::pair<int, int> parameter_handshake();
    void wait_parameter_idle(const std::string& consequence);
    std::pair<bool, std::string> commit_parameters();
    SoftwareResetResult software_reset(const SoftwareResetOptions& options);
    bool watch_reset_gap(double window_s);
    BaudChangeResult set_baud_rate(int rate, const BaudChangeOptions& options);

    // ------------------------------------------------------------------ non-volatile guard (axis_nv.cpp)
    /// Count one non-volatile write cycle (`what` for the debug log).
    void note_nonvolatile_write(const std::string& what);

    // ------------------------------------------------------------------ fast path (axis_fast.cpp; monitor.hpp)
    std::atomic<Axis*> owner{nullptr};        ///< the Axis holding this state now (set by its constructors and moves)
    std::shared_ptr<AxisMonitor> monitor;     ///< (mutex) the monitor of start_monitor()
    bool fast_mode = false;                   ///< (mutex) enable_fast() ran
    std::vector<std::string> fast_notes;      ///< (mutex) what enable_fast() could not do
    /// A move sent by move_to (start_move_to builds its MoveHandle from it).
    struct StartedMove {
        double target = 0.0;
        double band = 0.1;
        double timeout_s = 0.0;
        Completion completion = Completion::Safe;
        double sent_at = 0.0;                 ///< now_s() right after the command was answered
    };
    std::optional<StartedMove> started_move;  ///< (mutex) the last one
    /// Where a wait loop takes its statuses from (next_status).
    struct Feed {
        bool lean = false;                    ///< the 8-word block (Completion::Fast), else the full status
        double since = 0.0;                   ///< monitor samples whose request went out at or after this only
        double gap_s = 0.0;                   ///< pause between two own reads
        long seq = 0;                         ///< the last monitor sample taken
        bool first = true;
    };

    /// The completion rule of one call: `completion` or the axis's own.
    Completion completion_mode(std::optional<Completion> completion) const;
    /// The monitor of start_monitor() while it runs, else null.
    std::shared_ptr<AxisMonitor> live_monitor() const;
    /// Stop and detach every monitor of this axis; monitor is null afterwards (close()).
    void stop_monitors();
    /// One FC03 of 0x9000 x kLeanStatusWords (retries nullopt: the bus's).
    Status read_lean_status(std::optional<int> retries = std::nullopt);
    /// A lean sample that shows the servo off, an alarm or EMG is confirmed with the full block (STAT tells an
    /// automatic servo OFF from a fault).
    Status confirmed(const Status& st);
    /// The status a move command is checked against: the monitor's when younger than kFreshS and showing the axis
    /// ready, else a full read.
    Status pre_move_status();
    /// The measured position of a relative move: the monitor's status when younger than kFreshS, else a full read.
    Status measured_status();
    /// One sample against the completion rule `mode` (`settle` carries the safe rule's window).
    bool is_done(const Status& st, std::optional<double> target, double band, Completion mode, detail::Settle& settle);
    /// A feed for a wait loop: lean (the 8-word block, kFastPollS apart) or the full status (poll_interval_s apart).
    Feed make_feed(bool lean, double since) const;
    /// The next status of `feed` (never ends): the monitor's samples whose request went out at or after feed.since (a
    /// stalled or failed monitor falls back to a direct read), else own reads.
    Status next_status(Feed& feed);
    /// One status without blocking: the monitor's latest when its request went out at or after `since` and it is
    /// fresh, else one read (lean or full).
    Status sample_since(double since, bool lean);
    /// Record the move just sent (start_move_to).
    void note_move(double target, double band, double timeout_s, Completion mode, double sent_at);
    /// wait_idle by the rule `mode` with samples from `since` on (default: this call). Stops and raises like wait_idle.
    Status wait_done(std::optional<double> target, std::optional<double> band, double timeout_s, Completion mode,
                     std::optional<double> since = std::nullopt);

    // ------------------------------------------------------------------ command strategies (axis_strategy.cpp)
    // Port of ec.py _active_strategy / _run_strategy / _switch_to_register / stop_path and the register STOP substitute.
    std::optional<CommandMethod> strategy;    ///< (mutex) set by set_strategy() or the switch to "register"
    bool coils_refused = false;               ///< (mutex) an FC05 write of this axis was answered with exception 01
    RegisterCommands registers;               ///< the DRG1 image of the register path (thread-safe)
    std::atomic<bool> stop_pause_on{false};   ///< the register STOP's pause is ON and not yet cancelled + released

    /// The strategy in use: set_strategy() / the switch to "register"; else AxisOptions::method when not Coil; else
    /// the line's (Line::command_strategy(): "register" behind RCON); else Coil.
    CommandMethod current_strategy() const;
    void set_strategy(CommandMethod method);
    /// The line kind's name, nullopt on a direct line.
    std::optional<std::string> line_name() const;
    /// LineRuleError (nothing sent) for an action the line does not carry.
    LineRuleError line_refusal(const std::string& what, const std::string& action) const;
    /// No FC05 on this line / unit: the line rules, an exception 01 seen by this axis or remembered by the bus.
    bool no_coils() const;
    /// The register STOP substitute applies (the Register strategy, or no coils).
    bool register_stop_path() const;
    /// An FC05 write answered with exception 01 (illegal function): the unit / line has no coils.
    static bool coil_refused(const std::exception& e);
    /// No coils from now on: a coil strategy becomes Register (an IAI one is kept for servo / home / reset).
    void switch_to_register(const std::string& why);
    /// `requested` (nullopt: the strategy in use) adapted to the line: an IAI method on a line without 0x45 / 0x48 ->
    /// Coil, Coil without coils -> Register. `explicit_`: a per-call method is refused there instead (LineRuleError).
    CommandMethod active_strategy(std::optional<CommandMethod> requested = std::nullopt, bool explicit_ = false);
    /// The strategy for a per-call method (nullopt: the configured one, adapted to the line).
    CommandMethod strategy_for(std::optional<CommandMethod> method);
    /// Run `send(method)`; a Coil method refused with exception 01 switches to Register and runs it once more. Returns
    /// the method that ran it.
    template <class F>
    CommandMethod run_strategy(CommandMethod method, const std::string& action, F&& send) {
        try {
            send(method);
            return method;
        } catch (const ModbusSlaveError& e) {
            if (!(uses_coils(method) && coil_refused(e))) throw;
        }
        switch_to_register("FC05 refused with exception 01 (" + action + ")");
        send(CommandMethod::Register);
        return CommandMethod::Register;
    }
    /// STP on / off: the pause coil, or DRG1 STP on the register path (an exception 01 switches to it).
    void set_pause(bool on);
    /// ALRS edge with STP held, then STP off (coil or register path); with a fault alarm in `st` (read when nullopt)
    /// the ALRS edge is left out and only STP is released.
    void cancel_pause(std::optional<Status> st);
    /// The coil-free STOP: DRG1 pause, hold until standstill, ALRS edge while paused, release.
    /// `run`: the caller's StopRun (the last run to end cancels the pause).
    std::pair<Status, bool> register_stop_sequence(double timeout_s, StopRun& run);
    /// Best-effort register STOP after `error` (notes on the error).
    void try_register_stop(EcDriverError* error, const std::string& when, StopRun& run);
    /// STOP coil ON or the register path's DRG1 pause (begin_stop's frame).
    void send_stop_first_frame();
    Status set_safety_speed(bool on, bool verify);
    Status set_action_mode(bool pc, std::optional<bool> safety, bool verify);
    /// The experimental MOD / POSR / TEAC sequence (teach_position with TeachMethod::Coil).
    PositionWriteResult teach_by_coils(int row, const NvWriteOptions& options);
    /// The unit switched the servo off by itself (STAT ASOF, SV 0): servo ON by the strategy in use, then a SON off /
    /// on cycle when it does not come back within kAsofWakeS. ServoBlockedError (with the reasons) when it stays off.
    Status wake_from_auto_servo_off(Status st);

    // ------------------------------------------------------------------ the axis on a gateway line (axis_rcon.cpp)
    bool directory_absent = false;            ///< (mutex) the 0x0000 read timed out where that means "no directory"
    /// A timed-out 0x0000 read means "no directory" on this line (Line rules directory_timeout_means_absent).
    bool directory_timeout_is_absence() const;
    /// The directory read is settled as absent on this line (no I/O).
    bool no_directory() const;
    /// RCON driver units: the position table is 'system, no access' (DOC ME0413 p.5-10): a row write goes out only
    /// when the unit's own directory describes it. LineRuleError (nothing sent).
    void rows_writable_here(const std::string& action);
    /// What identify() needs on a gateway line: a reader that does not send FC11 where the line does not document it,
    /// the 0x9400 serial code read instead, the errors to report first.
    struct LineIdentity {
        std::unique_ptr<UnitReader> reader;     ///< null: rd()
        std::optional<std::string> serial_code;
        std::vector<std::string> errors;
    };
    /// Prepare identify() on this line (direct line: nothing, no I/O). A complete DeviceInfo when the unit has no
    /// directory here (then nothing else is read: every other area is 'system, no access').
    std::optional<DeviceInfo> identify_on_line(LineIdentity& prep);
    /// Merge the line's identity facts into what read_device_info returned (FC11 errors replaced, the serial code).
    void finish_line_identity(const LineIdentity& prep, DeviceInfo& info, std::vector<std::string>& errors);
    /// The line's strategy for a new axis (Axis::open, Axis(Port&)); no I/O. UnsupportedLineError without axis access.
    void apply_line(const std::shared_ptr<Line>& line);
    /// Status::from_registers with this line's position unit (PNOW x 0.001 on a 0.001 mm RCON unit, 3 decimals) and
    /// line kind (Status::line).
    Status decode_status(const std::vector<std::uint16_t>& words);
};

namespace detail {
/// The axis state for the library's own modules (Axis befriends this; nothing else names it): coordinated motion
/// (src/coord_run.cpp) and recovery (src/recovery.cpp).
struct AxisAccess {
    using Impl = Axis::Impl;
    static Impl& impl(Axis& axis) { return *axis.impl_; }
};

/// RAII: a group stop holds `axis` (Axis::Impl::group_stops) from before its first STOP frame until end() (its full
/// stop() ended) or the destructor, whichever comes first; end() may run on another thread. Not copyable.
class GroupStopHold {
public:
    explicit GroupStopHold(std::shared_ptr<Axis> axis) : axis_(std::move(axis)) {
        ++AxisAccess::impl(*axis_).group_stops;
    }
    ~GroupStopHold() { end(); }
    GroupStopHold(const GroupStopHold&) = delete;
    GroupStopHold& operator=(const GroupStopHold&) = delete;
    void end() noexcept {
        if (!ended_.exchange(true)) --AxisAccess::impl(*axis_).group_stops;
    }

private:
    std::shared_ptr<Axis> axis_;
    std::atomic<bool> ended_{false};
};

/// The GroupStopHold of every axis of one group stop, by name. Filled before the stop's threads start, never changed
/// while they run (end() reads it from them).
class GroupStopHolds {
public:
    void add(const std::string& name, std::shared_ptr<Axis> axis) {
        if (axis && !holds_.count(name)) holds_.emplace(name, std::make_unique<GroupStopHold>(std::move(axis)));
    }
    /// That axis's full stop ended.
    void end(const std::string& name) noexcept {
        const auto it = holds_.find(name);
        if (it != holds_.end()) it->second->end();
    }

private:
    std::map<std::string, std::unique_ptr<GroupStopHold>> holds_;
};
}  // namespace detail

}  // namespace iai_actuator_driver
