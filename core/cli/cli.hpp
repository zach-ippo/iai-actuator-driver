// Private header of the iai-actuator-driver-cpp command line (cli/*.cpp, axis unit). Port of the Python edition's cli.py (C#
// IaiActuatorDriver.Cli: CliArgs.cs, CliApp.cs, CliOutput.cs).
//
//   iai-actuator-driver-cpp ports                                  list serial ports, IAI adapters first (opens nothing)
//   iai-actuator-driver-cpp [--port COM3 | --ports COM3,COM4 | --sim] scan [--slaves 1-16] [--scan-timeout S]
//                                                        find the answering axes (READ-ONLY)
//   iai-actuator-driver-cpp stop-all [--ports COM3,COM4 | --sim] [--slaves 1-16] [--broadcast]
//                                                        decel STOP on every axis found (not a safety function)
//   iai-actuator-driver-cpp (--port COMx | --sim [--sim-profile NAME]) [--slave N | --axis N] [--baud RATE] [--json] COMMAND
//     status | identify | info [--section S]... | limits | alarms | line [--monitor]   READ-ONLY
//     monitor [--count N] [--interval S] | monitor --watch [--duration S]               READ-ONLY
//     backup [FILE] [--rows used|all|0,1,2] | check [--motion] [--report FILE]          read-only (check: unless --motion)
//     servo on|off | home | alarm-reset [--method coil|iai]
//     move POS [--speed V] [--accel G] [--decel G] [--accel-pattern P] [--band B] [--no-wait] [--fast]
//          [--change-speed V | --change-accel G] [--change-after S]   absolute move (mm, or deg on a rotary axis)
//     jog DELTA [--speed V] [--accel G] [--fast]                relative move from the measured position
//     push POS BAND PCT [--toward-home] [--speed V]             approach POS, then push over BAND at PCT %
//     stop | pause | resume | cancel-pause
//     goto N [--speed-pct P] [--allow-fast] [--no-wait]         run stored row N
//     teach N [--force] [--method table|coil]                   store the current position as row N (permanent)
//     mode pc|pio [--safety [on|off]]                           action mode: PC (PMSL coil) or PIO (0x45 cmd 0x0C)
//     param list | param get KEY | param set KEY VALUE [--force] [--no-commit]
//     baud RATE [--method direct|break] | swreset
//   The former opt-ins --allow-nonvolatile, --allow-dangerous, --allow-home-params, --allow-unlisted and
//   --allow-unverified-code are accepted and ignored (no longer needed; not in --help).
//     recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]   never moves
//   Caps: --stroke MIN MAX, --max-speed V, --max-accel G, --max-push PCT (default 100 units/s, 0.3 G, 70 %, always
//   lowered to the unit's own maxima), --unit-max (the unit's own maxima).
//   Line: --line auto|direct|rcon|robonet, --position-unit 0.01|0.001 (RCON parameter No.207).
//   Strategy: --strategy coil|iai|iai2|register (servo / home / alarm reset path; default: the line's, else coil).
//   Global: --timeout S, --retries N (default 3), --log-frames, --help, --version.
//   Hidden (tests / demos): --sim-time-scale X, --sim-baud RATE, --sim-slaves LIST (scan, stop-all), --sim-state FILE
//   (keep the simulated unit's position and position table across runs), --sim-line rcon|robonet (the unit behind a
//   simulated gateway), --sim-unit CODE (the simulated unit's FC11 unit code).
//
// Exit codes (as the Python CLI): 0 done; 1 the command ran but reports a failure (nothing found by scan); 2 a driver
// error or a usage error (the message on stderr); 130 interrupted (Ctrl+C: a decel stop is sent first).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace cli {

inline constexpr const char* kProgram = "iai-actuator-driver-cpp";
inline constexpr const char* kBanner = "iai-actuator-driver-cpp (UNOFFICIAL - not an IAI product)";

/// A command line that cannot be run as given (exit code 2).
class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// The parsed command line: positionals (command and its arguments) and --options anywhere on the line (--name value
/// or --name=value). A token such as -5 or -0.5 is a number, not an option. Unknown options are a UsageError.
class Args {
public:
    static Args parse(const std::vector<std::string>& argv);
    /// The arity of a flag that may take one value, on or off ("--safety", "--safety on", "--safety=off").
    static constexpr int kOnOff = -1;
    /// Every option and how many values it takes (kOnOff: none, or on / off).
    static const std::map<std::string, int>& arity();

    bool has(const std::string& name) const;
    std::optional<std::string> value(const std::string& name) const;   ///< the last occurrence
    std::vector<std::string> values(const std::string& name) const;    ///< all values of the last occurrence
    std::vector<std::string> every(const std::string& name) const;     ///< the values of every occurrence, in order
    std::optional<double> number(const std::string& name) const;       ///< UsageError when not a number
    std::optional<int> integer(const std::string& name) const;         ///< UsageError when not an integer
    const std::vector<std::string>& positionals() const { return positionals_; }
    /// UsageError for an option given that `command` does not take.
    void allow(const std::string& command, const std::vector<std::string>& allowed) const;

private:
    std::vector<std::string> positionals_;
    std::map<std::string, std::vector<std::string>> options_;
    std::map<std::string, std::vector<std::string>> every_;
};

/// A positional as a finite number / an integer (UsageError naming `what` otherwise).
double parse_number(const std::string& what, const std::string& text);
int parse_integer(const std::string& what, const std::string& text);
/// "1-4,7" -> {1, 2, 3, 4, 7} (addresses 1..247; UsageError otherwise).
std::vector<int> parse_slave_list(const std::string& what, const std::string& text);

/// The usage text.
std::string usage();

/// Run one command line (argv without the program name). Never throws: errors are printed to `err`.
int run(const std::vector<std::string>& argv, std::ostream& out, std::ostream& err);

/// Ctrl+C (main.cpp's handler): a running motion command stops its axis, sends no motion frame afterwards, and run()
/// returns 130. Signal-safe.
void request_interrupt() noexcept;
bool interrupted() noexcept;

// ------------------------------------------------------------------------------------------ text output
namespace text {

std::string hex(const std::vector<std::uint8_t>& bytes);
void ports(const std::vector<SerialPortInfo>& entries, std::ostream& out);
void scan(const std::string& name, std::optional<int> baud, const std::vector<ScanHit>& hits,
          const std::string& error, std::ostream& out);
/// `line`: the line kind printed as "line       rcon" (empty: not printed, a direct line).
void status(const Status& st, std::ostream& out, const std::string& units, const std::string& line = std::string());
/// One compact line of a (lean) status, for monitor --watch.
std::string status_line(const Status& st, const std::string& units);
/// The text form of info::unit_info() (--json carries every field). Positions in the axis's units.
void unit_info(const Json& data, std::ostream& out);
/// The text form of a recovery report.
void recovery(const recovery::RecoveryReport& report, const std::string& units, std::ostream& out);
void info(const DeviceInfo& info, Axis& axis, std::ostream& out);
void alarms(const std::vector<AlarmRecord>& records, std::ostream& out);
void limits(const AxisRanges& ranges, std::ostream& out);
void parameters(const std::vector<ParameterInfo>& list, const ParameterModel* model, std::ostream& out,
                const std::string& units, bool is_ec, const std::string& family);
void parameter_write(const ParameterWriteResult& result, std::ostream& out, const std::string& prefix);
void row_write(const PositionWriteResult& result, std::ostream& out, const std::string& units);
void reset(const SoftwareResetResult& result, std::ostream& out);
void baud(const BaudChangeResult& result, std::ostream& out);

}  // namespace text

// ------------------------------------------------------------------------------------------ commands of cli_extra.cpp
// The commands added for parity with the Python CLI. `label` is the session's name: "sim" or the port name. Each returns
// the exit code; driver errors propagate to run().
namespace extra {

int info(Axis& axis, const Args& args, std::ostream& out);
int monitor(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err);
int push(Axis& axis, const std::vector<std::string>& pos, const Args& args, std::ostream& out);
/// move --change-speed / --change-accel: start the move, wait --change-after s, change the running move, wait for it.
std::optional<Status> move_and_change(Axis& axis, double target, const MoveOptions& options, const Args& args,
                                      std::ostream& err);
int backup(Axis& axis, const std::string& label, const std::vector<std::string>& pos, const Args& args,
           std::ostream& out);
int recover(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err);
int line(Axis& axis, const Args& args, std::ostream& out);
int check(Axis& axis, const std::string& label, const Args& args, std::ostream& out, std::ostream& err);
/// "YYYYmmdd-HHMMSS" in local time (default file names of backup and check).
std::string file_stamp();

}  // namespace extra

}  // namespace cli
}  // namespace iai_actuator_driver
