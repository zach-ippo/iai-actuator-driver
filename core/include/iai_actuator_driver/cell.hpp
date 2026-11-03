// Several axes over one or more ports, driven together: every axis found or given (no total limit unless max_axes; one
// per COM port, or up to 16 per port by slave address). Ports run in parallel threads; the axes of one port take turns
// (one bus mutex). Port of the Python edition's cell.Cell (C# Cell.cs); implemented by the axis unit (src/cell.cpp).
//
// Faults: when an axis fails during a group command with a link fault (TransportError, ModbusTimeout after its
// retries), on_fault(name, kind, message) is called once for that axis and - with stop_on_fault (default) - every
// other axis is stopped (stop_all). faults() lists the faulted axes until clear_fault(). Independently, a motion
// group command (home_all, move_all / move_each, wait_all) that fails on any axis stops the other axes it moves and
// raises CellError after handling every axis.
#pragma once

#include <exception>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/recovery.hpp"

namespace iai_actuator_driver {

namespace coord {
class CoordinatedGroup;  // coord.hpp: coordinated (interpolated) motion of several axes (Cell::coordinated)
}  // namespace coord

namespace info {
struct BackupRows;  // info.hpp: which position rows Cell::backup reads
}  // namespace info

/// on_fault(axis name, kind "port_vanished" | "unit_silent" (| "unit_restarted" | "unit_replaced" from the link checks
/// and recover_all; recovery.hpp), message).
using FaultCallback = std::function<void(const std::string& name, const std::string& kind, const std::string& message)>;

/// Settings of Cell::open.
struct CellOptions {
    std::optional<int> max_axes;                   ///< the most axes the cell takes (>= 1); nullopt = no limit
    std::optional<int> baud_rate;                  ///< fixed rate for every port; nullopt = detect each port once
    double read_timeout_s = kDefaultReadTimeoutS;
    BusOptions bus;                                ///< per port (3 retries)
    double scan_timeout_s = kScanTimeoutS;
    std::vector<int> scan_slaves = default_scan_slaves();
    bool identify = true;                          ///< identify every axis at open (READ-ONLY)
    AxisOptions axis;                              ///< for every axis
    bool stop_on_fault = true;
    FaultCallback on_fault;
    LineOptions line;                              ///< every port's line kind ("auto") and RCON position unit
    recovery::EventCallback on_event;              ///< every RecoveryEvent of the cell (also kept in events())
};

/// One axis's move of move_each.
struct AxisMove {
    std::string axis;            ///< "COM3:1"
    double position = 0.0;
    MoveOptions options;         ///< speed, accel, band, decel (options.wait is ignored: move_each's wait applies)
};

/// (py-cell) One axis of Cell::assemble: the cell's name for it ("COM3:1"), the port it is on and the axis (on that
/// port's bus).
struct CellMember {
    std::string name;
    std::shared_ptr<Port> port;
    std::shared_ptr<Axis> axis;
};

/// status_all takes a running status monitor's sample when it is younger than this (s).
inline constexpr double kMonitorStatusAgeS = 0.1;

/// move_all's per-axis values ({axis: value}, the Python edition's mapping form of speed_mm_s / accel_g / band_mm /
/// decel_g), each in that axis's own units. A map that is not empty must name every axis of the move and replaces
/// MoveOptions' value for each (std::out_of_range for a missing or unknown axis, nothing sent); an empty map keeps
/// MoveOptions' value.
struct PerAxisMoveValues {
    std::map<std::string, double> speed;
    std::map<std::string, double> accel_g;
    std::map<std::string, double> band;
    /// A deceleration other than the axis's accel_g: the command-profile move (FC17 0x0F00 + PCRF, Axis::move_to).
    std::map<std::string, double> decel_g;
};

/// One read of the whole 0x9000 monitor block of `axis`, decoded the way the Python edition's Elecylinder.read_monitor()
/// returns it: position, velocity (VNOW, coarse on an EC), current (CNOW raw), current_pct (CNOW / MCUR 4.5 x 100, null
/// when the rated current is not readable), deviation, alarm + alarm_name, uptime_s, units, raw. READ-ONLY (FC03).
/// EcNotReadyError when the block could not be read. Used by Cell::monitor_all (src/cell_extras.cpp).
Json monitor_reading(Axis& axis);

/// One or more axes failed in a group command. Every axis was handled before this was raised.
class CellError : public EcDriverError {
public:
    CellError(std::string operation, std::map<std::string, std::string> messages,
              std::map<std::string, std::exception_ptr> errors);
    const std::string& operation() const noexcept { return operation_; }
    /// Axis name -> error message.
    const std::map<std::string, std::string>& messages() const noexcept { return messages_; }
    /// Axis name -> the exception (rethrow with std::rethrow_exception).
    const std::map<std::string, std::exception_ptr>& errors() const noexcept { return errors_; }

private:
    std::string operation_;
    std::map<std::string, std::string> messages_;
    std::map<std::string, std::exception_ptr> errors_;
};

/// Per-axis results of a group command (with return_exceptions the failures are kept here instead of raising).
template <class T>
struct CellResults {
    std::string operation;
    std::vector<std::string> names;               ///< the axes asked, in cell order (stop_all / pause_all: then the
                                                  ///< listed names the cell does not have, each an error)
    std::map<std::string, T> results;             ///< the axes that succeeded
    std::map<std::string, std::string> messages;  ///< the axes that failed -> message
    std::map<std::string, std::exception_ptr> errors;

    bool succeeded() const { return errors.empty(); }
    bool contains(const std::string& name) const { return results.count(name) != 0; }
    /// The result of one axis; rethrows that axis's error when it failed (std::out_of_range when not asked).
    const T& at(const std::string& name) const {
        auto e = errors.find(name);
        if (e != errors.end()) std::rethrow_exception(e->second);
        return results.at(name);
    }
};

class Cell {
public:
    /// Every IAI USB adapter of the PC, scanned (READ-ONLY). Ports that cannot be opened or where nothing answers are
    /// skipped (port_errors()); EcNotReadyError when no axis at all is found.
    static Cell open(const CellOptions& options = {});
    /// These serial ports, scanned.
    static Cell open(const std::vector<std::string>& ports, const CellOptions& options = {});
    /// Cell::open({"COM3", "COM4"}) (a braced list of names would otherwise be ambiguous with the map overloads).
    static Cell open(std::initializer_list<std::string> ports, const CellOptions& options = {}) {
        return open(std::vector<std::string>(ports), options);
    }
    /// These slaves on these serial ports (each listed slave must answer).
    static Cell open(const std::map<std::string, std::vector<int>>& slaves_by_port, const CellOptions& options = {});
    /// Ready-made transports by name (SimLine, SimulatedUnit...): `slaves` empty = scan each, else these on every one.
    static Cell open(const std::map<std::string, std::shared_ptr<Transport>>& transports,
                     const std::vector<int>& slaves = {}, const CellOptions& options = {});
    /// The same for a map of a Transport subclass, e.g. Cell::open(SimLine::sim_ports(3)).
    template <class T, class = std::enable_if_t<std::is_base_of<Transport, T>::value && !std::is_same<T, Transport>::value>>
    static Cell open(const std::map<std::string, std::shared_ptr<T>>& transports, const std::vector<int>& slaves = {},
                     const CellOptions& options = {}) {
        const std::map<std::string, std::shared_ptr<Transport>> base(transports.begin(), transports.end());
        return open(base, slaves, options);
    }
    /// (py-cell) Ports that are open already (a binding's ports on its buses), in this order under their names:
    /// `slaves` nullopt = scan each (CellOptions::scan_slaves), else {port name: slaves} (each listed slave must
    /// answer; a port not named takes none). The rate of a port is detected unless it was given or found already; the
    /// line as the port has it. Ports where nothing answers or that fail are skipped when scanning (port_errors()) and
    /// closed, as are the ports left without an axis. Destroyed without close(), the cell leaves its axes and ports
    /// open (they are the caller's).
    static Cell open(const std::vector<std::shared_ptr<Port>>& ports,
                     const std::optional<std::map<std::string, std::vector<int>>>& slaves,
                     const CellOptions& options = {});
    /// (py-cell) A cell of axes that exist already (a binding's Port::axis objects): nothing is scanned or probed and
    /// nothing is sent unless options.identify. Names must be unique, each axis on its port's bus, at most
    /// options.max_axes when given (std::invalid_argument otherwise). The cell shares the axes and ports with their
    /// other users.
    static Cell assemble(const std::vector<CellMember>& members, const CellOptions& options = {});

    ~Cell();
    Cell(Cell&& other) noexcept;
    Cell& operator=(Cell&& other) noexcept;
    Cell(const Cell&) = delete;
    Cell& operator=(const Cell&) = delete;

    // ---- the axes ("COM3:1" = port:slave), in port order then slave order
    std::vector<std::string> names() const;
    std::size_t size() const;
    bool contains(const std::string& name) const;
    Axis& operator[](const std::string& name);   ///< std::out_of_range for an unknown name
    Axis& operator[](std::size_t index);
    std::shared_ptr<Axis> axis_ptr(const std::string& name) const;
    std::shared_ptr<Port> port_of(const std::string& name) const;
    std::map<std::string, std::shared_ptr<Port>> ports() const;
    /// Every port's line kind ("direct", "rcon", "robonet", "robonet-gateway"; line.hpp), keyed by port name.
    std::map<std::string, std::string> lines() const;
    /// The gateway of every port that has one (RCON, ROBONET gateway mode), keyed by port name.
    std::map<std::string, std::shared_ptr<Gateway>> gateways() const;
    /// A fresh read of every gateway's status, keyed by port name. READ-ONLY.
    std::map<std::string, GatewayStatus> gateway_status_all();
    std::string units_of(const std::string& name);
    /// identify() results kept from open / identify_all.
    std::map<std::string, DeviceInfo> info() const;
    /// Ports skipped by a scanning open, and why.
    std::map<std::string, std::string> port_errors() const;
    /// (py-cell) The exceptions of port_errors (rethrow with std::rethrow_exception).
    std::map<std::string, std::exception_ptr> port_error_details() const;
    /// Faulted axes -> kind, until clear_fault.
    std::map<std::string, std::string> faults() const;
    void clear_fault(const std::string& name);

    // ---- group commands: `axes` empty = all; ports in parallel. Without return_exceptions a failure on any axis
    // raises CellError after every axis was handled.
    CellResults<DeviceInfo> identify_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false);
    /// Every axis's status: ports in parallel, the axes of a port one after another. An axis whose status monitor runs
    /// (start_monitors, Axis::start_monitor) and has a sample younger than monitor_max_age_s is not read: its sample is
    /// returned instead, the lean status (PNOW .. DSSE: raw.words 8, no STAT / ZONS / POSS / SSSE / uptime).
    /// monitor_max_age_s nullopt: always read. READ-ONLY.
    CellResults<Status> status_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false,
                                   std::optional<double> monitor_max_age_s = kMonitorStatusAgeS);
    CellResults<AxisRanges> limits_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false);
    /// monitor_reading() of every axis (position, velocity, current, deviation, alarm, uptime). READ-ONLY (ports in
    /// parallel, the axes of a port one after another).
    CellResults<Json> monitor_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false);
    /// Read-only backup of every axis (info::backup_axes) into `directory`, one "<axis>.json" each (FC03 / FC11 only;
    /// nothing is written to a unit). Returns (axis name, file path) in cell order, or (name, "error: ...") for an axis
    /// that did not answer; the other axes are still saved. Rows "used" (info::BackupRows). (src/cell_extras.cpp)
    std::vector<std::pair<std::string, std::string>> backup(const std::string& directory,
                                                            const std::vector<std::string>& axes = {});
    std::vector<std::pair<std::string, std::string>> backup(const std::string& directory,
                                                            const std::vector<std::string>& axes,
                                                            const info::BackupRows& rows);
    CellResults<Status> servo_all(bool on, const std::vector<std::string>& axes = {}, double timeout_s = kServoTimeoutS,
                                  bool return_exceptions = false);
    /// Home every axis at once, wait for all. `method`: the home command's path (nullopt: each axis's strategy).
    /// `stop_others_on_error`: a failure on one axis stops the others.
    CellResults<Status> home_all(const std::vector<std::string>& axes = {}, std::optional<double> timeout_s = std::nullopt,
                                 bool return_exceptions = false, std::optional<CommandMethod> method = std::nullopt,
                                 bool stop_others_on_error = true);
    /// Start an absolute move on every axis of `targets` ({name: position}, one MoveOptions for all) with as little
    /// delay as possible, then (options.wait) wait for all. Every move is checked first (check_move on every axis): one
    /// bad value refuses the whole move, nothing sent (CellError "move_all (check; nothing started)"). A start that
    /// fails stops the axes already started. nullopt when !options.wait (call wait_all later).
    std::optional<CellResults<Status>> move_all(const std::map<std::string, double>& targets,
                                                const MoveOptions& options = {}, bool return_exceptions = false);
    /// move_all with per-axis speed / accel / band / decel ({axis: value} maps; see PerAxisMoveValues). An axis whose
    /// decel_g differs from its accel_g takes the command-profile move (checked against MAXA / MAXD first, nothing sent
    /// on a refusal); the wait estimate uses the slower of the two.
    std::optional<CellResults<Status>> move_all(const std::map<std::string, double>& targets, const MoveOptions& options,
                                                const PerAxisMoveValues& per_axis, bool return_exceptions = false);
    /// move_all with per-axis options (speed, accel, band, decel each from AxisMove::options; their `wait` is ignored).
    /// `stop_others_on_error`: a start that fails stops the axes already started, a failed wait the others.
    std::optional<CellResults<Status>> move_each(const std::vector<AxisMove>& moves, bool wait = true,
                                                 std::optional<double> timeout_s = std::nullopt,
                                                 bool return_exceptions = false, bool stop_others_on_error = true);
    /// Wait until every axis is in position and settled (an axis started by move_all: at its target and band; timeout
    /// per axis from the move's estimate, 10 s otherwise).
    CellResults<Status> wait_all(const std::vector<std::string>& axes = {}, std::optional<double> timeout_s = std::nullopt,
                                 bool return_exceptions = false, bool stop_others_on_error = true);
    /// Decel stop on every axis as fast as possible: begin_stop() on all first (skipped on a line without coils), then
    /// stop() on each. `broadcast`: on a daisy chain whose axes are all selected (two or more axes, a line that carries
    /// broadcasts: never behind a gateway) one broadcast STOP (Port::broadcast_stop) goes out first and stops every unit
    /// at once; the per-axis hold / release follows as the fallback, then the broadcast release once every axis of that
    /// line has stopped. Several units acting on one broadcast is UNVERIFIED on hardware, hence an opt-in. A listed
    /// name the cell does not have never holds back the STOP: every listed axis it has is stopped, then each unknown
    /// name is an error of its own (std::out_of_range; CellError unless return_exceptions).
    CellResults<Status> stop_all(const std::vector<std::string>& axes = {}, double timeout_s = 5.0,
                                 bool return_exceptions = false, bool broadcast = false);
    /// Pause every axis (keeps the remaining travel; Axis::pause), ports in parallel. `broadcast`: a broadcast pause
    /// (Port::broadcast_pause, 00 05 04 0A FF 00 AC D9) first on every daisy chain whose axes are all selected (as for
    /// stop_all; never behind a gateway; UNVERIFIED on hardware), then the per-axis pause as the fallback. Unknown
    /// names as for stop_all: the axes the cell has are paused, each unknown name is an error of its own.
    CellResults<bool> pause_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false,
                                bool broadcast = false);
    /// Release the pause on every axis (Axis::resume: the paused moves continue). `broadcast`: after the per-axis
    /// releases, a broadcast release (00 05 04 0A 00 00 ED 29) on every daisy chain whose axes are all selected.
    CellResults<bool> resume_all(const std::vector<std::string>& axes = {}, bool return_exceptions = false,
                                 bool broadcast = false);

    // ---- status monitors (monitor.hpp; Port of the Python edition's Cell.start_monitors / stop_monitors)
    /// Start the status monitor of every axis (Axis::start_monitor with `options`: gap_s, link_check_s, callbacks...).
    /// Each monitor's link events ("link_lost", "unit_restarted", ...) go to events() / CellOptions::on_event and to the
    /// fault handling (on_fault; with stop_on_fault every other axis is stopped, from a thread of its own). Calling it
    /// again for a running monitor adds the callbacks in `options` and routes its events once. READ-ONLY. Stopped by
    /// stop_monitors() and close(); recover_all() holds a monitor during the recovery and restarts it afterwards.
    std::map<std::string, std::shared_ptr<AxisMonitor>> start_monitors(const std::vector<std::string>& axes = {},
                                                                       const MonitorOptions& options = {});
    /// Stop the status monitor of every axis (no-op where none runs).
    void stop_monitors(const std::vector<std::string>& axes = {});

    // ---- power / link loss (recovery.hpp; Port of the Python edition's Cell.track_all / check_links / monitor_links /
    // recover_all). A link fault seen here (also a restarted unit) goes to on_fault and, with stop_on_fault, stops every
    // other axis once; the events ("link_lost", "unit_restarted", "unit_replaced", "recovered") go to events() and
    // CellOptions::on_event.
    /// recovery::track on every axis (ports in parallel; the identity from info() when it was read; options.name and
    /// options.info are set per axis). READ-ONLY.
    CellResults<std::shared_ptr<recovery::Session>> track_all(const std::vector<std::string>& axes = {},
                                                              const recovery::TrackOptions& options = {},
                                                              bool return_exceptions = false);
    /// recovery::check_link on every axis (one status read each, ports in parallel). An axis not tracked yet is tracked
    /// first (READ-ONLY). A fault goes to on_fault (and stops the other axes). Never throws for an axis: an unexpected
    /// error is kept in the result's errors.
    CellResults<recovery::LinkCheck> check_links(const std::vector<std::string>& axes = {});
    /// Start a background LinkMonitor that runs check_links' check on every axis each interval_s (faults go to on_fault
    /// as above; on_change(name, check) on every state change). Stopped by close(); it never recovers or moves anything.
    std::shared_ptr<recovery::LinkMonitor> monitor_links(double interval_s = 1.0, const std::vector<std::string>& axes = {},
                                                         recovery::LinkMonitor::ChangeFn on_change = {});
    /// recovery::recover on every axis (default: all): ports in parallel, the axes of one port one after another (the
    /// first reopens a shared port and restores its baud rate for all). An axis never tracked is checked against the
    /// identity read at open (info()). Recovered axes leave faults(). Never moves anything: no servo ON, no home, no
    /// move; a move cut off by the fault is not resumed (the report's at_target / needs_home say what is left). A
    /// failed or replaced axis (faults() then says "unit_replaced") raises CellError after every axis was handled.
    /// A stop of the other axes that a status monitor's fault event started is let finish first (up to 10 s); each
    /// axis's status monitor is held during its recovery and restarted afterwards.
    CellResults<recovery::RecoveryReport> recover_all(const std::vector<std::string>& axes = {},
                                                      double timeout_s = recovery::kRecoverTimeoutS,
                                                      const recovery::RecoveryPolicy& policy = {},
                                                      bool return_exceptions = false);
    /// The cell's last (at most 200) recovery events, oldest first.
    std::vector<recovery::RecoveryEvent> events() const;

    // ---- coordinated motion (coord.hpp; include it to use these; implemented in src/coord_run.cpp). No I/O.
    /// A coord::CoordinatedGroup of cell axes: {path name, cell axis name} in order (the first two are the default
    /// plane of a circle), e.g. cell.coordinated({{"x", "COM3:1"}, {"y", "COM4:1"}}); empty: every axis under its cell
    /// name. std::out_of_range for an unknown axis, std::invalid_argument for an axis or a name given twice. A failure
    /// during a run stops every axis of the group (stop_all on those axes only). The cell must outlive the group and stay
    /// where it is (not moved) while the group is used.
    coord::CoordinatedGroup coordinated(const std::vector<std::pair<std::string, std::string>>& axes = {});
    /// The same with the cell names as the path names (Python cell.coordinated("COM3:1", "COM4:1")).
    coord::CoordinatedGroup coordinated_names(const std::vector<std::string>& names);

    /// Close every port. Safe to call twice.
    void close();
    bool closed() const;

private:
    Cell();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iai_actuator_driver
