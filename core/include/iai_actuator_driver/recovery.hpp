// Power-loss and link-loss recovery: find out what happened to an axis's link, bring it back, say what changed. Port of
// the Python edition's recovery.py (the source of truth); implemented by src/recovery.cpp (the Cell side in
// src/cell.cpp).
//
// Three things take an axis away from the PC:
// * "port_vanished": the USB adapter was pulled; the COM port disappears from the PC (VERIFIED in the Python edition)
//   and every call on the open handle fails (TransportError), or a silent unit whose port is no longer listed.
// * "unit_silent": the port is there but the unit does not answer after the retries (24 V off, cable, or a unit that
//   came back at another baud rate).
// * "unit_restarted": the unit answers but restarted since track(): its uptime counter 0x9010 dropped (only a power-up
//   resets it, a software reset does not: VERIFIED), a new 0x0FFF power-up record is in its alarm history (VERIFIED; none
//   was written after a 3.2 s outage on the gripper, 2026-10-30),
//   Modbus control (PMSS) dropped while the PC held it (0 after every power-up: VERIFIED), HEND was lost (an
//   incremental unit: VERIFIED at a software reset, assumed at a power-up), or its baud rate fell back to 38400 (the
//   power-up rate: VERIFIED).
//
// Typical use:
//
//   Axis ax = Axis::open("COM3"); ax.connect(); ax.set_baud_rate(115200);
//   recovery::track(ax);                       // identity, baud rate, PC control, safety speed, HEND, uptime
//   ax.move_to(50.0);                          // the axis records its target for recover() itself
//   auto chk = recovery::check_link(ax);       // one status read: cheap enough for a monitor thread
//   if (!chk.ok()) {
//       recovery::RecoverOptions o; o.timeout_s = 60;
//       auto report = recovery::recover(ax, o);  // never moves the axis
//       if (report.needs_home) ax.home();          // the application decides what to do next
//   }
//
// recover(), in order: wait for the port to be listed again (by name), reopen it in place (ModbusRtu::reopen), find the
// unit's baud rate (38400 first when power was lost), re-identify the unit and REQUIRE the same serial number and model
// (else UnitReplacedError: nothing is written to the other unit), restore the session (the baud rate the application
// had set, PC control, the safety-speed choice), compare the ranges re-read, then assess: alarms (only message-level
// ones such as a PCON's 0x069 are reset automatically), servo, HEND (needs_home), the position against the last
// commanded target, and on request the stored rows (checksums) and the parameters against a cached copy. It NEVER
// starts motion: no servo ON, no home, no move; a move cut off by the fault is not resumed.
//
// Events (on_event callbacks get a RecoveryEvent): "link_lost", "unit_restarted", "unit_replaced", "recovered", each
// once per change of the axis's link state. Cell adds on_fault (by default every other axis is stopped),
// check_links(), monitor_links() and recover_all() (cell.hpp).
//
// The bookkeeping (session, link state, pending target) is kept per unit: per bus and slave address, so every Axis
// object on the same bus and slave shares it; it is dropped when the bus is destroyed (or by forget()).
//
// Evidence: the power-up facts and the vanishing COM port above are VERIFIED in the Python edition, and its recover()
// is VERIFIED on the EC-GRBP8M gripper after a real power cycle (a fresh process, session_from_backup) and after a line
// break (found at 9600, put back on 38400 / 230400; also through Cell monitor_links / recover_all). A power cycle seen
// live by a running process is VERIFIED on the same gripper with this core (2026-10-30, through the Python binding):
// the 24 V supply was off for 3.2 s; check_link reported the link lost (unit_silent) and then unit_restarted (the
// uptime counter dropped, 6481.8 s to 0.8 s, and PMSS dropped while the PC held it); recover() took PC control back
// in 1.44 s; the servo stayed on, HEND was kept, the 3 rows and 102 parameter items were unchanged and nothing was
// written to non-volatile memory. No new 0x0FFF power-up record was written after that 3.2 s outage. Beyond that run
// this C++ edition is tested on the simulator (SimulatedUnit::power_cycle / power_off / unplug, SimLine::unplug).
// Behind a gateway (RCON / ROBONET) the same steps run where the line carries them (UNVERIFIED).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
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
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

namespace detail {
class MonotonicCv;  // src/monotonic_cv.hpp (internal)
}  // namespace detail

namespace recovery {

// ------------------------------------------------------------------------------------------ states and events

inline constexpr const char* kLinkOk = "ok";
inline constexpr const char* kPortVanished = "port_vanished";
inline constexpr const char* kUnitSilent = "unit_silent";
inline constexpr const char* kUnitRestarted = "unit_restarted";
inline constexpr const char* kUnitReplaced = "unit_replaced";
/// recover() is running on the axis.
inline constexpr const char* kRecovering = "recovering";
/// Every link state, in that order.
const std::vector<std::string>& link_states();

inline constexpr const char* kEventLinkLost = "link_lost";
inline constexpr const char* kEventUnitRestarted = "unit_restarted";
inline constexpr const char* kEventUnitReplaced = "unit_replaced";
inline constexpr const char* kEventRecovered = "recovered";
/// Every event kind, in that order.
const std::vector<std::string>& event_kinds();

// ------------------------------------------------------------------------------------------ constants

/// The rate every unit tested powers up at (BRSL item 8.3 = code 6; VERIFIED): probed first after a power loss.
inline constexpr int kBootBaudRate = 38400;
inline constexpr double kRecoverTimeoutS = 30.0;
/// How often recover() looks for the port / the unit again while waiting.
inline constexpr double kPortPollS = 0.5;
inline constexpr double kPositionToleranceMm = 0.1;
/// The uptime counter must have advanced at least kUptimeRateMin x the PC's elapsed time minus kUptimeSlackS since
/// the last reading; less means the unit restarted meanwhile (it counts ms since power-up and tracks the wall clock:
/// VERIFIED).
inline constexpr double kUptimeSlackS = 2.0;
inline constexpr double kUptimeRateMin = 0.9;
/// Directory classes compared by the parameter integrity check (the writable parameter classes; classes 3 / 4 carry
/// live values such as the present alarm and are left out).
inline constexpr int kParameterClasses[] = {0x0007, 0x0008, 0x0009, 0x000C};

// ------------------------------------------------------------------------------------------ events

/// One change of an axis's link state: `kind` one of event_kinds(); `cause` the link state behind it
/// ("port_vanished" ...); `time` seconds since the epoch (wall clock).
struct RecoveryEvent {
    std::string kind;
    std::string axis;
    std::string message;
    std::string cause;
    double time = 0.0;

    Json to_json() const;
};

/// An event callback. An exception it throws is logged and ignored (a broken callback never breaks a recovery).
using EventCallback = std::function<void(const RecoveryEvent&)>;

/// Replaces list_ports() for the "port listed again?" check (tests, a custom adapter list).
using PortLister = std::function<std::vector<SerialPortInfo>()>;

// ------------------------------------------------------------------------------------------ session

/// The identity fields recover() compares (serial number, model, system / unit code) and the firmware.
struct Identity {
    std::optional<std::string> serial_number;
    std::optional<std::string> model_name;
    std::optional<int> system_code;
    std::optional<int> unit_code;
    std::optional<std::string> firmware;

    Json to_json() const;
    bool operator==(const Identity& o) const;
    bool operator!=(const Identity& o) const { return !(*this == o); }
};

/// (code, detail, address, time stamp ms or nullopt) of one alarm-history slot.
struct AlarmKey {
    int code = 0;
    int detail = 0;
    int address = 0;
    std::optional<std::int64_t> time_ms;

    bool operator==(const AlarmKey& o) const {
        return code == o.code && detail == o.detail && address == o.address && time_ms == o.time_ms;
    }
    bool operator!=(const AlarmKey& o) const { return !(*this == o); }
};

/// Which stored rows track() copies / RecoveryPolicy::check_rows checks: none (the default), all (`all`: rows 0 ..
/// max_positions() - 1, or the rows of the cached copy), or the listed row numbers.
struct RowSelection {
    bool all = false;
    std::vector<int> rows;

    static RowSelection none() { return RowSelection(); }
    static RowSelection every() {
        RowSelection r;
        r.all = true;
        return r;
    }
    static RowSelection of(std::vector<int> numbers) {
        RowSelection r;
        r.rows = std::move(numbers);
        return r;
    }
    bool selected() const { return all || !rows.empty(); }
};

/// What the application had established on an axis while its link was up (track()): the unit's identity, the baud
/// rate, PC control (PMSS), the safety-speed choice, servo, HEND, the uptime and alarm history used to spot a restart,
/// the last commanded target, and optional copies of the stored rows / parameters for the integrity checks.
/// check_link() refreshes the volatile fields while the link is healthy; recover() compares against them and restores
/// them. Shared (std::shared_ptr); lock `mutex` to change fields while another thread may check or recover the axis.
struct Session {
    std::string name;                            ///< "" = none (the axis is then named "<port>:<slave>")
    int slave = 1;
    std::string port;                            ///< the transport's name ("" when it has none)
    std::optional<int> baud_rate;
    std::optional<std::string> serial_number;
    std::optional<std::string> model_name;
    std::optional<int> system_code;
    std::optional<int> unit_code;
    std::optional<std::string> firmware;
    std::optional<std::string> product_family;
    std::optional<Span> soft_limits_mm;
    std::optional<double> max_speed_mm_s;
    std::string units = "mm";
    std::optional<bool> modbus_control;
    std::optional<bool> safety_speed;
    std::optional<bool> servo_on;
    std::optional<bool> homed;
    std::optional<std::int64_t> uptime_ms;
    std::optional<double> uptime_at;             ///< now_s() of the uptime reading
    std::optional<std::vector<AlarmKey>> alarm_history;
    std::optional<double> last_target_mm;
    std::optional<std::map<int, PositionEntry>> rows;
    std::optional<std::map<std::string, std::string>> parameters;  ///< "0x0008:8.15" -> upper-case hex
    std::optional<Json> backup;                  ///< a backup (info::backup layout) to compare with
    double taken_at = now_s();
    std::atomic<bool> recovering{false};
    mutable std::recursive_mutex mutex;          ///< guards the fields while a check or a recovery reads them
    std::mutex recover_mutex;                    ///< one recover() per axis at a time

    /// The fields recover() requires to match, and the firmware.
    Identity identity() const;
    /// The Python edition's Session.to_dict() layout (rows / parameters / has_backup; no backup).
    Json to_json() const;
};

/// Settings of track().
struct TrackOptions {
    std::string name;                 ///< "" keeps the earlier session's name
    std::optional<DeviceInfo> info;   ///< an identity already read (skips identify())
    bool alarm_history = true;        ///< keep the history to spot a new power-up record (192 words)
    RowSelection rows;                ///< also copy these stored rows
    bool parameters = false;          ///< also copy the items of kParameterClasses
    std::optional<Json> backup;       ///< a backup to compare with instead
};

/// A deliberate change the application (or the axis itself) made; nullopt = unchanged.
struct ControlChange {
    std::optional<bool> modbus_control;
    std::optional<bool> safety_speed;
    std::optional<int> baud_rate;
    std::optional<bool> homed;
};

/// Remember what the application has set up on `axis` while its link is up, for check_link() and recover().
/// READ-ONLY (FC03 / FC11). Call it after connect(), a baud change, homing or taking control, and again after any
/// deliberate change of these (or use note_control()). Replaces an earlier session of the unit. Throws the link error
/// when the axis does not answer.
std::shared_ptr<Session> track(Axis& axis, const TrackOptions& options = {});
/// A session from an identity already read (e.g. Cell::info()), without any I/O: recover() can then check the unit is
/// the same one, but has no PC-control / safety-speed / uptime baseline to restore or compare (those steps are skipped
/// with a note). Registered unless the unit already has one (that one is returned).
std::shared_ptr<Session> session_from_info(const Axis& axis, const DeviceInfo& info, const std::string& name = "");
/// A session from a backup (the info::backup / Python info.backup layout), without any I/O: recover() then requires
/// the unit to be the one the backup was taken from and compares the stored rows / parameters with the backup when the
/// policy asks for it. No PC-control / safety-speed / uptime baseline (state them with note_control()); the baud rate
/// is the one in use now. Replaces an earlier session. std::invalid_argument when the backup holds no identity.
std::shared_ptr<Session> session_from_backup(const Axis& axis, const Json& backup, const std::string& name = "");
/// The same from a backup file (UTF-8 JSON; std::runtime_error when it cannot be read).
std::shared_ptr<Session> session_from_backup_file(const Axis& axis, const std::string& path,
                                                  const std::string& name = "");
/// The unit's session, nullptr when none.
std::shared_ptr<Session> session_of(const Axis& axis);
/// Drop the unit's session, link state and pending target.
void forget(const Axis& axis);
/// Write volatile fields of the unit's session under its mutex (no I/O), as the Python edition's code sets the fields
/// of its Session: an object with any of "name", "baudrate", "modbus_control", "safety_speed", "servo_on", "homed"
/// (null: unknown), "uptime_ms", "uptime_age_s" (the uptime reading is that many seconds old: uptime_at = now_s() -
/// age), "last_target_mm" (null: none) and "recovering". EcNotReadyError without a session; std::invalid_argument for
/// another key or a value of the wrong type (nothing is written then).
void update_session(const Axis& axis, const Json& fields);
/// Record the last commanded target (axis units) so recover() can compare the position with it (nullopt: no target,
/// e.g. after a home return). The Axis records its own moves; Cell::move_all records its targets.
void note_target(const Axis& axis, std::optional<double> target_mm);
/// Tell the session about a deliberate change, so check_link() does not take it for a restart and recover() restores
/// the new choice. Without a session nothing happens.
void note_control(const Axis& axis, const ControlChange& change);

/// The unit's last known link state (link_states(); "ok" when nothing was seen).
std::string link_state(const Axis& axis);
/// Record the link state and emit the matching event when it changed: "link_lost" on entering "port_vanished" /
/// "unit_silent" from a working link, "unit_restarted", "unit_replaced" and "recovered" (back to "ok" from a fault).
/// Returns true when the state changed. std::invalid_argument for an unknown state.
bool set_link_state(const Axis& axis, const std::string& state, const std::string& message = "",
                    const EventCallback& on_event = EventCallback(), const std::string& name = "");

// ------------------------------------------------------------------------------------------ detection

/// The name of the axis's port ("COM3", a simulated line's name), "" for a transport without one.
std::string port_name(const Axis& axis);
/// Whether the axis's port is listed by the PC right now: the transport's own present() (the simulators, a serial
/// port by its name in list_ports()); with `lister`, a transport that cannot tell is looked up by name in it.
/// nullopt when it cannot be told. READ-ONLY: no port is opened.
std::optional<bool> port_present(const Axis& axis, const PortLister& lister = PortLister());
/// The link state an exception means: "port_vanished" (TransportError, or no answer while `axis`'s port is no longer
/// listed), "unit_silent" (no valid answer after the retries: timeout, CRC or frame errors), "unit_replaced"
/// (UnitReplacedError), or "" for anything that is not a link fault (a refused command, an alarm, a soft limit, a
/// closed bus ...).
std::string classify_failure(const std::exception& error, const Axis* axis = nullptr,
                             const PortLister& lister = PortLister());
std::string classify_failure(const std::exception_ptr& error, const Axis* axis = nullptr,
                             const PortLister& lister = PortLister());
/// The signs in `status` (and the alarm records / baud rate found) that the unit restarted since `session` was
/// refreshed, in words; empty when none. `now`: now_s() (default).
std::vector<std::string> restart_signs(const Session& session, const Status& status,
                                       const std::vector<AlarmKey>& new_records = {},
                                       std::optional<int> baud_rate_found = std::nullopt,
                                       int boot_baud_rate = kBootBaudRate, std::optional<double> now = std::nullopt);
/// The slots added at the top of the history since `before` (newest first, empty slots left out). History entries
/// shift down by one per new alarm.
std::vector<AlarmKey> new_alarm_records(const std::optional<std::vector<AlarmKey>>& before,
                                        const std::optional<std::vector<AlarmKey>>& after);

/// Result of check_link(): `state` (link_states()), `reasons` in words, the status read (when the unit answered), the
/// error (when it did not) and how long the check took.
struct LinkCheck {
    std::string state = kLinkOk;
    std::vector<std::string> reasons;
    std::optional<Status> status;
    std::exception_ptr error;
    std::string error_text;          ///< "<Type>: <message>" of `error` ("" without one)
    double elapsed_s = 0.0;
    std::string axis;

    bool ok() const { return state == kLinkOk; }
    Json to_json() const;
};

/// Settings of check_link().
struct CheckLinkOptions {
    std::shared_ptr<Session> session;  ///< nullptr: the unit's session
    EventCallback on_event;
    std::string name;
    PortLister lister;
};

/// One cheap link check, safe from a monitor thread (the bus mutex serialises it with the application's commands):
/// one status read (22 words), compared with the session (track()) for restart signs. READ-ONLY. Returns "ok" (the
/// session's volatile baseline of uptime, PC control, safety speed, servo, HEND and baud rate is moved forward),
/// "port_vanished" / "unit_silent" (the read failed; error), "unit_restarted" (reasons; the baseline is kept for
/// recover()) or "recovering" (a recover() is running). Emits "link_lost" / "unit_restarted" to on_event when the state
/// changes. Without a session only the link itself is checked. An error that is not a link fault is thrown.
LinkCheck check_link(Axis& axis, const CheckLinkOptions& options = {});

/// A background thread running check_link() on each axis every interval_s. on_change(name, check) is called when an
/// axis's state changes (the first check counts as a change only when it is not "ok"); states() holds the last
/// LinkCheck per axis. `check` replaces check_link (Cell::monitor_links routes faults to the cell this way). The
/// monitor never recovers or moves anything itself: call recover() (or Cell::recover_all) from the application when it
/// reports a fault. The axes must outlive the monitor (the destructor stops it).
class LinkMonitor {
public:
    using CheckFn = std::function<LinkCheck(const std::string& name, Axis& axis)>;
    using ChangeFn = std::function<void(const std::string& name, const LinkCheck& check)>;

    struct Options {
        double interval_s = 1.0;   ///< > 0, else std::invalid_argument
        ChangeFn on_change;
        EventCallback on_event;
        CheckFn check;
    };

    /// Named axes, checked in this order.
    LinkMonitor(std::vector<std::pair<std::string, Axis*>> axes, Options options);
    /// One axis (named like the events: the session's name, else "<port>:<slave>").
    LinkMonitor(Axis& axis, Options options);
    ~LinkMonitor();
    LinkMonitor(const LinkMonitor&) = delete;
    LinkMonitor& operator=(const LinkMonitor&) = delete;

    bool running() const;
    /// One pass over every axis, in the calling thread. Returns the checks of this pass (an axis whose check threw is
    /// left out and the error kept in errors()).
    std::map<std::string, LinkCheck> check_now();
    LinkMonitor& start();
    /// Stop and join the thread. A check of the pass in progress that waits for the bus ends at once (cancel.hpp), one
    /// on the wire within its response timeout. Safe to call twice.
    void stop(double timeout_s = 5.0);

    std::map<std::string, LinkCheck> states() const;
    long passes() const;
    /// The last (at most 50) errors of checks that threw ("<name>: <Type>: <message>").
    std::vector<std::string> errors() const;
    double interval_s() const { return options_.interval_s; }

private:
    void run();

    std::vector<std::pair<std::string, Axis*>> axes_;
    Options options_;
    mutable std::mutex mutex_;
    const std::unique_ptr<::iai_actuator_driver::detail::MonotonicCv> wake_;  ///< timed waits on the monotonic clock
    bool stop_requested_ = false;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::map<std::string, LinkCheck> states_;
    std::atomic<long> passes_{0};
    std::vector<std::string> errors_;
    std::mutex pass_mutex_;  ///< one pass at a time (check_now from the application and the thread)
    /// Cancelled by stop(): the thread's pass ends at its next check point (a sticky scope in run(); cancel.hpp)
    std::shared_ptr<CancelToken> cancel_ = std::make_shared<CancelToken>();
};

// ------------------------------------------------------------------------------------------ recovery

/// What recover() restores and checks. Motion is never part of it.
struct RecoveryPolicy {
    /// Switch the unit back to the rate the application had set (only a verified baud-change code, or the boot rate's
    /// code; a broadcast: every unit of a shared line switches).
    bool restore_baud_rate = true;
    /// Take PC control (PMSL) again when the application held it.
    bool restore_control = true;
    /// Put the safety-speed mode back as it was.
    bool restore_safety_speed = true;
    /// Reset a message-level alarm (a warning such as 0x069); other alarms are left for the operator.
    bool clear_message_alarms = true;
    /// Compare the stored rows with the session's copy / the backup and check their checksums.
    RowSelection check_rows;
    /// Compare the parameter classes with the session's copy / the backup.
    bool check_parameters = false;
    /// A backup to compare with (used when the session has no copy).
    std::optional<Json> backup;
    /// |position - last target| that still counts as at the target.
    double position_tolerance_mm = kPositionToleranceMm;
    /// How often to look for the port / the unit while waiting.
    double poll_s = kPortPollS;
    /// The power-up rate probed first after a power loss.
    int boot_baud_rate = kBootBaudRate;
    /// Replaces list_ports() for the "port listed again?" check.
    PortLister list_ports;
};

/// A new alarm-history record recover() found (newest first).
struct NewAlarmRecord {
    int code = 0;
    std::string hex;   ///< "0x0FFF"
    std::string name;
    std::optional<std::int64_t> time_ms;
};

/// What recover() found and did. `cause`: the link state it started from; `restart_signs` why the unit is taken to
/// have restarted; `actions` what was sent / done, in order; `notes` what the application should know or do (nothing in
/// a note was done automatically). needs_attention() sums up whether anything needs a decision before the next move;
/// ready_to_move() whether the status allows one (servo ON, homed, PC control, no alarm).
struct RecoveryReport {
    std::string axis;
    std::string cause = kLinkOk;
    bool restarted = false;
    std::vector<std::string> restart_signs;
    bool port_reopened = false;
    std::optional<int> baud_rate_found;
    std::optional<int> baud_rate;
    bool baud_restored = false;
    bool same_unit = false;
    Identity identity;
    bool control_restored = false;
    /// The application held PC control before the fault and the unit still did (a line break, a CON reset): nothing
    /// was written to take it again.
    bool control_kept = false;
    bool safety_speed_restored = false;
    std::vector<std::string> limits_changed;
    int alarm_code = 0;
    std::optional<std::string> alarm_name;
    std::optional<std::string> alarm_level;    ///< level_key() of the alarm ("message", ...)
    std::vector<int> alarms_cleared;
    std::vector<NewAlarmRecord> new_alarm_records;
    std::optional<bool> modbus_control;
    std::optional<bool> safety_speed;
    std::optional<bool> servo_on;
    std::optional<bool> servo_was_on;
    std::optional<bool> homed;
    bool needs_home = false;
    std::optional<double> position_mm;
    std::optional<double> last_target_mm;
    std::optional<double> position_error_mm;
    std::optional<bool> at_target;
    std::vector<int> rows_checked;
    std::vector<std::string> row_problems;
    int parameters_checked = 0;
    std::vector<std::string> parameter_differences;
    std::vector<std::string> actions;
    std::vector<std::string> notes;
    double elapsed_s = 0.0;
    std::optional<Status> status;

    /// An alarm that blocks motion is still active.
    bool fault_alarm() const;
    /// Anything the application must decide before the next move: home lost, a fault alarm, a move cut off (not at its
    /// target) or still running, changed ranges, row / parameter differences, or the servo off where it was on.
    bool needs_attention() const;
    /// The status allows a move now (servo ON, homed, PC control, no alarm, no E-stop).
    bool ready_to_move() const;
    /// "G: recovered from unit_silent, unit restarted, needs home".
    std::string summary() const;
    /// The Python edition's RecoveryReport.to_dict() layout.
    Json to_json() const;
};

/// recover() could not bring the axis back (the port or the unit did not return in time, the identity could not be
/// confirmed, no session to compare with). report(): what was found so far (nullptr when nothing).
class RecoveryError : public EcDriverError {
public:
    explicit RecoveryError(const std::string& message, std::shared_ptr<RecoveryReport> report = nullptr)
        : EcDriverError(message), report_(std::move(report)) {}
    const RecoveryReport* report() const noexcept { return report_.get(); }
    std::shared_ptr<RecoveryReport> report_ptr() const noexcept { return report_; }
    void set_report(std::shared_ptr<RecoveryReport> report) { report_ = std::move(report); }

private:
    std::shared_ptr<RecoveryReport> report_;
};

/// Another unit answers where the tracked one was (a different serial number or model). Nothing was written to it:
/// check the machine, then track() the new unit (and back it up) before using it. expected() / found(): the identity
/// fields compared.
class UnitReplacedError : public RecoveryError {
public:
    UnitReplacedError(const std::string& message, Identity expected, Identity found,
                      std::shared_ptr<RecoveryReport> report = nullptr)
        : RecoveryError(message, std::move(report)), expected_(std::move(expected)), found_(std::move(found)) {}
    const Identity& expected() const noexcept { return expected_; }
    const Identity& found() const noexcept { return found_; }

private:
    Identity expected_;
    Identity found_;
};

/// Settings of recover().
struct RecoverOptions {
    double timeout_s = kRecoverTimeoutS;   ///< seconds to wait for the port / the unit (>= 0)
    RecoveryPolicy policy;
    std::shared_ptr<Session> session;      ///< nullptr: the unit's session
    EventCallback on_event;
    std::string name;
    /// recover_axis() only: without a session (none given, none tracked) throw recover()'s RecoveryError instead of
    /// taking the identity the axis read last as the baseline (the Python edition's module-level recover()).
    bool require_session = false;
};

/// Bring `axis` back after a power or link loss and report what changed. NEVER starts motion.
///
/// Needs the session track() recorded while the link was up (the identity to compare with). Steps: find out what
/// happened (one status read); if the port vanished, wait until it is listed again (every policy.poll_s) and reopen it
/// in place; if the unit was silent, find it again until the timeout (by its baud rate: 38400 first after a power loss,
/// then the session's, then the usual rates; or, without a baud rate, by a status read); re-identify it and require the
/// same serial number / model / unit code (UnitReplacedError otherwise, thrown before anything is written); look for
/// restart signs (restart_signs(), the alarm history); restore the session (baud rate, PC control, safety speed; per
/// policy); compare the ranges re-read by identify(); assess the alarm (only a message-level one is reset), servo, HEND
/// (needs_home) and the position against the last target; optionally check the stored rows and parameters. The
/// session's baseline is then moved forward and the link state is "ok" (event "recovered").
///
/// Throws RecoveryError (with report()) when the port / unit does not come back within the timeout or the identity
/// cannot be confirmed, and UnitReplacedError for another unit. Only one recovery runs per unit at a time;
/// check_link() answers "recovering" meanwhile.
RecoveryReport recover(Axis& axis, const RecoverOptions& options = {});

/// recover() the way the Python edition's Elecylinder.recover does it: without a session (none given, none tracked)
/// the identity this axis read last (Axis::device_info(), from connect() / identify()) is the baseline (no I/O).
/// UNVERIFIED on hardware (simulator only).
RecoveryReport recover_axis(Axis& axis, const RecoverOptions& options = {});

// ------------------------------------------------------------------------------------------ hooks for the Axis

/// Called by the Axis implementation for its own commands (a target, a deliberate control / HEND / baud change), by
/// bus and slave; never throws (bookkeeping must never fail a command). No-ops without a session.
namespace hooks {
void note_target(const std::shared_ptr<ModbusRtu>& bus, int slave, std::optional<double> target_mm) noexcept;
void note_control(const std::shared_ptr<ModbusRtu>& bus, int slave, const ControlChange& change) noexcept;
}  // namespace hooks

}  // namespace recovery
}  // namespace iai_actuator_driver
