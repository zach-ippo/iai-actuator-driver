// The commands of iai-actuator-driver-cpp: ports, scan and the axis commands, with the exit codes of the Python CLI (0 done, 1
// nothing found by scan, 2 a driver or usage error, 130 interrupted). Port of the Python edition's cli.py (C# CliApp.cs).
// Any IAI unit is driven the same way (ELECYLINDER or a CON controller): positions in mm, or deg on a rotary axis, every
// range read from the unit itself. This edition is UNVERIFIED on hardware.
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "axis_impl.hpp"
#include "cli.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/version.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace cli {

namespace {

std::atomic<bool> g_interrupt{false};

constexpr double kDefaultMaxPushPct = 70.0;  ///< the tool's conservative push cap without --max-push / --unit-max

const char* const kHelp =
    R"(iai-actuator-driver-cpp (UNOFFICIAL - not an IAI product)

usage: iai-actuator-driver-cpp (--port NAME | --sim [--sim-profile NAME]) [--slave N | --axis N] [options] COMMAND [args]
       iai-actuator-driver-cpp ports [--json]
       iai-actuator-driver-cpp scan [--ports COM3,COM4 | --sim] [--slaves 1-16] [--baud RATE] [--scan-timeout S] [--json]
       iai-actuator-driver-cpp stop-all [--ports COM3,COM4 | --sim] [--slaves 1-16] [--broadcast] [--json]

Port commands (no --port needed):
  ports                        list the serial ports, IAI USB adapters (10C4:81D7) first; opens nothing
  scan                         find the axes that answer on each port (default: every IAI adapter, slaves 1-16;
                               READ-ONLY)
  stop-all [--broadcast]       decel STOP on every axis found on the ports (not a safety function); --broadcast
                               also sends one broadcast STOP on each daisy chain (never behind a gateway;
                               several units acting on one broadcast is UNVERIFIED on hardware)

Axis commands (positions in mm, or deg on a rotary axis; every range is read from the unit):
  status                       position and status bits (read-only)
  identify                     identity, directory, soft limits, unit maxima (read-only)
  info [--section S]...        everything the unit tells about itself (read-only: FC03 + FC11); sections:
                               identity, parameters, monitor, alarms, maintenance, positions
  limits                       every range the driver enforces, with its source (read-only)
  monitor [--count N] [--interval S]
                               position, velocity, current, deviation, alarm, uptime (read-only)
  monitor --watch [--interval S] [--duration S]
                               the status monitor: a line per status change (at most one per --interval) and
                               every link event (link lost, unit restarted) until Ctrl+C or --duration
  alarms                       alarm history (read-only)
  line [--monitor]             the line (direct / rcon / robonet), its rules and the gateway status;
                               --monitor: this axis's RCON all-axes monitor record (read-only)
  backup [FILE] [--rows used|all|0,1,2]
                               read-only JSON backup: identity, directory, parameters, rows, alarms
  check [--motion] [--report FILE]
                               compatibility self-test (read-only unless --motion: a short move out and back)
  servo on|off                 servo on / off (on also takes Modbus control at rest)
  home [--method coil|iai]     home return and wait
  move POS [--speed V] [--accel G] [--band B] [--no-wait]
                               absolute move (default 20 units/s, 0.1 G, band 0.1)
       [--decel G]             deceleration when it differs from --accel (command-profile move)
       [--accel-pattern P]     trapezoid (default), s_motion or filter (CON units whose MVPT lists it)
       [--fast]                fast completion: done on the first in-position sample, the status monitor
                               feeding the wait; the baud rate is kept
       [--change-speed V | --change-accel G] [--change-after S]
                               change the running move after S s (default 1; plain waiting moves only;
                               UNVERIFIED on hardware)
  jog DELTA [--speed V] [--accel G] [--fast]
                               relative move by DELTA (default 10 units/s; never the unit's JOG coils)
  push POS BAND PCT [--toward-home] [--speed V]
                               approach POS, then push over BAND at PCT % force (capped by --max-push and
                               MXPP 7.9; below MNPP only warned); default away from home
  goto N [--speed-pct P] [--allow-fast] [--no-wait]
                               run stored row N at its stored speed (or P % of it; 0x45 cmd 6, in steps of
                               1/256, at least 0.78 %)
  stop                         decel stop (STOP coil on, wait until stopped, off); works without Modbus control
  pause | resume               hold the move keeping its remaining travel / let it continue
  cancel-pause                 drop the remaining travel of a paused move and release the pause
  alarm-reset [--method coil|iai]
                               alarm reset (not a software reset); 'reset' is a deprecated alias
  teach N [--force] [--method table|coil]
                               store the current position as row N (non-volatile); a row that already holds it
                               is not written (the memory lasts about 100,000 write cycles, IAI); --force writes;
                               --method coil: MOD / POSR / TEAC coil sequence (EXPERIMENTAL; default table)
  mode pc|pio [--safety [on|off]]
                               action mode: pc = PC / Modbus control (PMSS 1; the PMSL coil), pio = PIO
                               control (PMSS 0; 0x45 cmd 0x0C); --safety (or --safety on) switches the safety speed
                               on (~100 mm/s cap), without it (or off) off; only at rest
  swreset                      software reset (servo off, then 0x45 cmd 5): applies committed parameters;
                               an incremental unit loses its home
  baud RATE [--method direct|break]
                               change the baud rate (broadcast 0x45 cmd 4); volatile: power-up rate after a
                               power cycle; only 115200, 230400 and 38400 have codes verified on hardware
                               (38400 depends on the unit: the EC-R6S rod ignores it; another code is sent
                               with a warning)
  param list                   the unit's parameters, decoded (EC: P-numbers; CON: No.-numbers; read-only)
  param get KEY                one parameter or raw item (5, P5, No.35, 8.15, C.1[1]; read-only)
  param set KEY VALUE [--no-commit] [--force]
                               write, verify and commit (PARW); applies after swreset or a power cycle; a
                               home, dangerous or unlisted parameter is written with a warning.
                               A value the unit already holds is neither written nor committed (the
                               non-volatile memory lasts about 100,000 write cycles, IAI); --force commits it
  recover [--backup FILE] [--target POS] [--restore-baud RATE] [--no-control] [--wait S]
                               bring the axis back after a power or link loss: same unit, PC control, baud
                               rate, message alarms, and a report of home, alarm and position (never moves;
                               UNVERIFIED on hardware)

Connection and safety options:
  --port NAME                  serial port, e.g. COM3 (close IA-OS first: one program per port)
  --sim [--sim-profile NAME]   the simulator instead of hardware (slider, gripper, rod, pcon, rotary, unknown)
  --slave N | --axis N         Modbus address = axis number + 1 (default slave 1)
  --baud RATE                  fixed baud rate (default: detected in place: 115200, 38400, 9600, 230400, 57600, 19200)
  --timeout S                  read timeout in s (default 0.2)
  --retries N                  retries on timeout / CRC error (default 3, the manufacturer's recommendation)
  --stroke MIN MAX             narrow the unit's soft limits (never widened)
  --max-speed V                speed cap (default 100 units/s), always lowered to the unit's MAXV (7.4)
  --max-accel G                acceleration cap (default 0.3 G), always lowered to MAXA / MAXD (7.5 / 7.6)
  --max-push PCT               push force cap (default 70 %), always lowered to MXPP (7.9)
  --unit-max                   lift the default caps up to the unit's own maxima
  --line auto|direct|rcon|robonet
                               the line the unit sits on (default auto: probed once, read-only; robonet = SIO
                               through mode, which cannot be probed). Behind a gateway its rules apply
  --position-unit 0.01|0.001   RCON only: the unit's position unit (No.207), needed before mm-based motion
  --strategy coil|iai|iai2|register
                               servo / home / alarm reset path (default: coil, or register behind an RCON
                               gateway; register = FC06 DRG1 without coils; iai and iai2 are EXPERIMENTAL)
  --log-frames                 print every Modbus frame to stderr
  --json                       machine-readable output
  --version, --help

Commands that change the unit's state first read identity and ranges and take Modbus control while the axis
is at rest ('mode pio' gives control back to the PIO); read-only commands write nothing. Ctrl+C during a move
or home sends the decel STOP sequence.
This tool is not a safety function: keep a hardware emergency stop. This edition is UNVERIFIED on hardware.
Exit codes: 0 ok, 1 nothing found by scan / a failed check / a line error, 2 error, 130 interrupted.
)";

const std::vector<std::string> kConnectionOptions{
    "--port",      "--sim",       "--sim-profile", "--slave",     "--axis",          "--baud",
    "--timeout",   "--retries",   "--stroke",      "--max-speed", "--max-accel",     "--max-push",
    "--unit-max",  "--log-frames", "--sim-time-scale", "--sim-baud", "--sim-state", "--line",
    "--position-unit", "--sim-line", "--sim-unit", "--strategy"};

struct CommandSpec {
    int args;  ///< positional arguments after the command; -1: checked by the command
    std::vector<std::string> options;
    bool state_changing;
    bool motion;  ///< Ctrl+C stops the axis
};

const std::map<std::string, CommandSpec>& command_specs() {
    static const std::map<std::string, CommandSpec> specs{
        {"status", {0, {}, false, false}},
        {"identify", {0, {}, false, false}},
        {"info", {0, {"--section"}, false, false}},
        {"limits", {0, {}, false, false}},
        {"monitor", {0, {"--count", "--interval", "--watch", "--duration"}, false, false}},
        {"alarms", {0, {}, false, false}},
        {"line", {0, {"--monitor"}, false, false}},
        {"backup", {-1, {"--rows"}, false, false}},
        {"check", {0, {"--motion", "--report"}, false, true}},
        {"recover", {0, {"--backup", "--target", "--restore-baud", "--no-control", "--wait"}, false, false}},
        {"servo", {1, {}, true, true}},
        {"home", {0, {"--method"}, true, true}},
        {"alarm-reset", {0, {"--method"}, true, false}},
        {"reset", {0, {"--method"}, true, false}},
        {"move",
         {1,
          {"--speed", "--accel", "--band", "--no-wait", "--decel", "--accel-pattern", "--fast", "--change-speed",
           "--change-accel", "--change-after"},
          true,
          true}},
        {"jog", {1, {"--speed", "--accel", "--fast"}, true, true}},
        {"push", {3, {"--toward-home", "--speed"}, true, true}},
        {"stop", {0, {}, false, false}},
        {"pause", {0, {}, true, false}},
        {"resume", {0, {}, true, true}},
        {"cancel-pause", {0, {}, true, true}},
        {"goto", {1, {"--speed-pct", "--allow-fast", "--no-wait"}, true, true}},
        {"teach", {1, {"--allow-nonvolatile", "--force", "--method"}, true, false}},  // --allow-*: hidden no-ops
        {"mode", {1, {"--safety"}, true, false}},
        {"param",
         {-1, {"--no-commit", "--allow-dangerous", "--allow-home-params", "--allow-unlisted", "--force"}, false, false}},
        {"baud", {1, {"--method", "--allow-unverified-code"}, true, false}},
        {"swreset", {0, {}, true, false}},
    };
    return specs;
}

int usage_error(std::ostream& err, const std::string& message) {
    err << "error: " << message << '\n';
    err << "usage: " << kProgram
        << " (--port NAME | --sim [--sim-profile NAME]) [--slave N | --axis N] [options] COMMAND [args]\n";
    err << "       " << kProgram << " ports | scan [--ports COM3,COM4 | --sim] [--slaves 1-16]   (--help for every "
                                    "command)\n";
    return 2;
}

void report_error(std::ostream& err, const std::exception& e) {
    err << "error: " << detail::error_type_name(e) << ": " << e.what() << '\n';
}

void write_json(std::ostream& out, const Json& value) { out << value.dump(2) << '\n'; }

FrameLogger frame_logger(std::ostream& err) {
    auto gate = std::make_shared<std::mutex>();
    const double start = now_s();
    return [&err, gate, start](FrameDirection d, const std::vector<std::uint8_t>& frame) {
        std::lock_guard<std::mutex> lock(*gate);
        err << util::format("%.4f %-10s ", now_s() - start, to_string(d)) << text::hex(frame) << '\n';
    };
}

BusOptions bus_options(const Args& args, std::ostream& err) {
    BusOptions bus;
    if (const auto retries = args.integer("--retries")) {
        if (*retries < 0) throw UsageError("--retries must be >= 0");
        bus.retries = *retries;
    }
    if (args.has("--log-frames")) bus.frame_logger = frame_logger(err);
    return bus;
}

double read_timeout(const Args& args) {
    const auto t = args.number("--timeout");
    if (!t) return kDefaultReadTimeoutS;
    if (!(*t > 0)) throw UsageError("--timeout must be > 0");
    return *t;
}

SimProfile profile_of(const std::string& name) {
    const auto p = sim_profile_from_string(name);
    if (!p) {
        std::vector<std::string> names;
        for (SimProfile q : sim_profiles()) names.push_back(to_string(q));
        throw UsageError("--sim-profile: choose a profile from " + util::join(names, ", ") + ", got '" + name + "'");
    }
    return *p;
}

SimulatedUnitOptions sim_options(const Args& args, const std::string& profile, int slave) {
    SimulatedUnitOptions so;
    so.slave = slave;
    so.profile = profile_of(profile);
    if (const auto scale = args.number("--sim-time-scale")) {
        if (!(*scale > 0)) throw UsageError("--sim-time-scale must be > 0");
        so.time_scale = *scale;
    }
    if (const auto rate = args.integer("--sim-baud"))
        so.baud_rate = *rate;
    else if (const auto baud = args.integer("--baud"))
        so.baud_rate = *baud;
    if (const auto code = args.value("--sim-unit")) {
        const bool hex = util::starts_with(util::lower(*code), "0x");
        const auto n = util::parse_int(hex ? code->substr(2) : *code, hex ? 16 : 10);
        if (!n || *n < 0 || *n > 0xFF) throw UsageError("--sim-unit: a unit code 0..0xFF, got '" + *code + "'");
        so.unit_code = static_cast<int>(*n);
    }
    return so;
}

// Hidden --sim-state FILE (tests / demos): the simulated unit's position and its position table (the rows a teach
// writes to non-volatile memory) are loaded from FILE before the command and saved after it, so successive runs of
// the command line see one unit. Format: "position <mm>" and "row <n> <16 hex words>" lines.
std::vector<std::pair<int, std::vector<std::uint16_t>>> load_sim_state(const std::string& path,
                                                                      SimulatedUnitOptions& so) {
    std::vector<std::pair<int, std::vector<std::uint16_t>>> rows;
    std::ifstream in(path);
    if (!in) return rows;  // first run: the profile's state
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream fields(line);
        std::string what;
        fields >> what;
        if (what == "position") {
            double mm = 0.0;
            if (fields >> mm) so.position_mm = mm;
        } else if (what == "row") {
            int n = -1;
            fields >> n;
            std::vector<std::uint16_t> words;
            unsigned value = 0;
            while (fields >> std::hex >> value) words.push_back(static_cast<std::uint16_t>(value));
            if (n < 0 || words.size() != 16) throw UsageError("--sim-state " + path + ": bad line '" + line + "'");
            rows.emplace_back(n, words);
        }
    }
    return rows;
}

void save_sim_state(const std::string& path, const SimulatedUnit& unit) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) throw UsageError("--sim-state: cannot write " + path);
    out << "position " << util::format("%.2f", unit.position_mm()) << '\n';
    for (int n = 0; n < unit.position_count(); ++n) {
        out << "row " << n;
        for (std::uint16_t w : unit.position_row(n)) out << ' ' << util::format("%04x", w);
        out << '\n';
    }
}

AxisOptions axis_options(const Args& args) {
    AxisOptions o;
    const auto stroke = args.values("--stroke");
    if (stroke.size() == 2) {
        const double lo = parse_number("--stroke MIN", stroke[0]);
        const double hi = parse_number("--stroke MAX", stroke[1]);
        if (!(lo < hi)) throw UsageError("--stroke: MIN must be below MAX");
        o.stroke = Span{lo, hi};
    }
    // The configured caps: the explicit options, else the conservative defaults (100 units/s, 0.3 G, 70 %) unless
    // --unit-max. The driver always lowers each to the unit's own maximum (MAXV / MAXA / MAXD / MXPP).
    const bool unit_max = args.has("--unit-max");
    const auto speed = args.number("--max-speed");
    const auto accel = args.number("--max-accel");
    const auto push = args.number("--max-push");
    o.max_speed_mm_s = speed ? speed : (unit_max ? std::nullopt : std::optional<double>(kFallbackSpeedCap));
    o.max_accel_g = accel ? accel : (unit_max ? std::nullopt : std::optional<double>(kFallbackAccelCapG));
    o.max_push_pct = push ? push : (unit_max ? std::nullopt : std::optional<double>(kDefaultMaxPushPct));
    return o;
}

std::optional<CommandMethod> method_of(const Args& args) {
    const auto m = args.value("--method");
    if (!m) return std::nullopt;
    if (*m == "coil") return CommandMethod::Coil;
    if (*m == "iai") return CommandMethod::Iai;
    throw UsageError("--method: coil or iai, got '" + *m + "'");
}

TeachMethod teach_method_of(const Args& args) {
    const std::string m = args.value("--method").value_or("table");
    if (m == "table") return TeachMethod::Table;
    if (m == "coil") return TeachMethod::Coil;
    throw UsageError("--method: table or coil, got '" + m + "'");
}

/// --strategy: the command strategy for servo / home / alarm reset (nullopt: the line's, else coil).
std::optional<CommandMethod> strategy_of(const Args& args) {
    const auto name = args.value("--strategy");
    if (!name) return std::nullopt;
    if (const auto m = command_method_from_string(*name)) return m;
    throw UsageError("--strategy: coil, iai, iai2 or register, got '" + *name + "'");
}

/// mode --safety: on with the flag alone or "on", off with "off" or without the flag (the Python CLI's store_true).
bool safety_of(const Args& args) { return args.has("--safety") && args.value("--safety").value_or("on") == "on"; }

// ------------------------------------------------------------------------------------------ ports / scan

int run_ports(const Args& args, std::ostream& out) {
    args.allow("ports", {});
    if (args.positionals().size() != 1) throw UsageError("'ports' takes no argument");
    const std::vector<SerialPortInfo> entries = list_ports();
    if (args.has("--json")) {
        Json list = Json::array();
        for (const auto& e : entries) list.push_back(to_json(e));
        write_json(out, list);
    } else {
        text::ports(entries, out);
    }
    return 0;
}

int run_scan(const Args& args, std::ostream& out, std::ostream& err) {
    args.allow("scan", {"--ports", "--port", "--sim", "--sim-profile", "--sim-slaves", "--sim-baud", "--sim-time-scale",
                        "--slaves", "--baud", "--scan-timeout", "--timeout", "--log-frames", "--retries"});
    std::vector<std::string> pos = args.positionals();
    std::optional<std::string> profile = args.value("--sim-profile");
    if (args.has("--sim") && !profile) {  // "--sim gripper scan": a profile name taken as the profile
        for (auto it = pos.begin(); it != pos.end(); ++it) {
            if (*it != "scan" && sim_profile_from_string(*it)) {
                profile = *it;
                pos.erase(it);
                break;
            }
        }
    }
    if (pos.size() != 1 || pos[0] != "scan") throw UsageError("'scan' takes no argument (use --ports and --slaves)");
    const std::optional<std::string> port_list = args.has("--ports") ? args.value("--ports") : args.value("--port");
    const bool sim = args.has("--sim");
    if (port_list && sim) throw UsageError("--ports and --sim exclude each other");
    const std::vector<int> slaves =
        args.has("--slaves") ? parse_slave_list("--slaves", *args.value("--slaves")) : default_scan_slaves();
    const double scan_timeout = args.number("--scan-timeout").value_or(kScanTimeoutS);
    if (!(scan_timeout > 0)) throw UsageError("--scan-timeout must be > 0");
    const std::optional<int> baud = args.integer("--baud");
    const double timeout = read_timeout(args);
    const BusOptions bus = bus_options(args, err);
    std::vector<std::pair<std::string, std::function<std::shared_ptr<Port>()>>> targets;
    if (sim) {
        SimulatedUnitOptions so = sim_options(args, profile.value_or("slider"), 1);
        const std::vector<int> sim_slaves =
            args.has("--sim-slaves") ? parse_slave_list("--sim-slaves", *args.value("--sim-slaves")) : std::vector<int>{1};
        targets.emplace_back("sim", [so, sim_slaves, bus]() {
            auto line = SimLine::with_slaves(sim_slaves, so, "sim");
            return std::make_shared<Port>(line, "sim", bus);
        });
    } else {
        std::vector<std::string> names;
        if (port_list) {
            for (const auto& n : util::split(*port_list, ','))
                if (!util::trim(n).empty()) names.push_back(util::trim(n));
            if (names.empty()) throw UsageError("--ports: no port named");
        } else {
            names = iai_port_names();
            if (names.empty()) {
                err << "error: no IAI USB adapter found; name the ports with --ports COM3,COM4\n";
                return 2;
            }
        }
        for (const auto& name : names) {
            PortOptions po;
            po.baud_rate = baud;
            po.read_timeout_s = timeout;
            po.bus = bus;
            targets.emplace_back(name, [name, po]() { return Port::open(name, po); });
        }
    }
    Json report = Json::object();
    std::size_t found = 0;
    std::size_t failed = 0;
    for (const auto& target : targets) {
        std::optional<int> rate;
        std::vector<ScanHit> hits;
        std::string problem;
        try {
            std::shared_ptr<Port> port = target.second();
            try {
                bool answered = true;
                if (!baud) {
                    try {
                        port->ensure_baud_rate(slaves);
                    } catch (const ModbusTimeout&) {
                        answered = false;  // nothing answers at any rate: an empty result, not an error
                    }
                }
                if (answered) hits = port->scan_units(slaves, scan_timeout, true);
                rate = port->baud_rate();
            } catch (...) {
                port->close();
                throw;
            }
            port->close();
        } catch (const std::exception& e) {
            problem = detail::error_type_name(e) + ": " + e.what();
            ++failed;
        }
        found += hits.size();
        Json axes = Json::array();
        for (const auto& h : hits) axes.push_back(to_json(h));
        Json entry = Json::object();
        entry.set("baud", rate).set("axes", axes).set("error", problem.empty() ? Json() : Json(problem));
        report.set(target.first, entry);
        if (!args.has("--json")) text::scan(target.first, rate, hits, problem, out);
    }
    if (args.has("--json")) write_json(out, report);
    if (failed == targets.size()) return 2;
    return found > 0 ? 0 : 1;
}

// stop-all: open the ports (read-only scan), then Cell::stop_all (STOP to every axis first, then wait for each to stand
// still; --broadcast adds one broadcast STOP on daisy chains, UNVERIFIED on hardware).
int run_stop_all(const Args& args, std::ostream& out, std::ostream& err) {
    args.allow("stop-all", {"--ports", "--port", "--sim", "--sim-profile", "--sim-slaves", "--sim-baud",
                            "--sim-time-scale", "--slaves", "--baud", "--scan-timeout", "--timeout", "--log-frames",
                            "--retries", "--broadcast"});
    if (args.positionals().size() != 1) throw UsageError("'stop-all' takes no argument (use --ports and --slaves)");
    const std::optional<std::string> port_list = args.has("--ports") ? args.value("--ports") : args.value("--port");
    const bool sim = args.has("--sim");
    if (port_list && sim) throw UsageError("--ports and --sim exclude each other");
    CellOptions co;
    co.scan_slaves =
        args.has("--slaves") ? parse_slave_list("--slaves", *args.value("--slaves")) : default_scan_slaves();
    co.scan_timeout_s = args.number("--scan-timeout").value_or(kScanTimeoutS);
    if (!(co.scan_timeout_s > 0)) throw UsageError("--scan-timeout must be > 0");
    co.baud_rate = args.integer("--baud");
    co.read_timeout_s = read_timeout(args);
    co.bus = bus_options(args, err);
    co.identify = false;
    std::optional<Cell> cell;
    try {
        if (sim) {
            const SimulatedUnitOptions so = sim_options(args, args.value("--sim-profile").value_or("slider"), 1);
            const std::vector<int> sim_slaves = args.has("--sim-slaves")
                                                    ? parse_slave_list("--sim-slaves", *args.value("--sim-slaves"))
                                                    : std::vector<int>{1};
            const std::map<std::string, std::shared_ptr<Transport>> lines{
                {"sim", SimLine::with_slaves(sim_slaves, so, "sim")}};
            cell.emplace(Cell::open(lines, {}, co));
        } else {
            std::vector<std::string> names;
            if (port_list) {
                for (const auto& n : util::split(*port_list, ','))
                    if (!util::trim(n).empty()) names.push_back(util::trim(n));
                if (names.empty()) throw UsageError("--ports: no port named");
            } else {
                names = iai_port_names();
                if (names.empty()) {
                    err << "error: no IAI USB adapter found; name the ports with --ports COM3,COM4\n";
                    return 2;
                }
            }
            cell.emplace(Cell::open(names, co));
        }
    } catch (const UsageError&) {
        throw;
    } catch (const std::exception& e) {
        report_error(err, e);
        return 2;
    }
    const CellResults<Status> results = cell->stop_all({}, 5.0, true, args.has("--broadcast"));
    if (args.has("--json")) {
        Json report = Json::object();
        for (const auto& name : results.names) {
            Json entry = Json::object();
            const auto failed = results.errors.find(name);
            if (failed != results.errors.end())
                entry.set("error", detail::describe(failed->second));
            else
                entry.set("stopped", true).set("status", to_json(results.results.at(name)));
            report.set(name, entry);
        }
        write_json(out, report);
    } else {
        for (const auto& name : results.names) {
            const auto failed = results.errors.find(name);
            if (failed != results.errors.end())
                out << name << ": STOP FAILED: " << detail::describe(failed->second) << '\n';
            else
                out << name << ": stopped at " << util::fixed(results.results.at(name).position_mm) << ' '
                    << cell->units_of(name) << '\n';
        }
        out << "STOP is a decelerate-stop, not a safety function: use the hardware emergency stop for safety.\n";
    }
    cell->close();
    return results.errors.empty() ? 0 : 2;
}

// ------------------------------------------------------------------------------------------ axis commands

void check_numbers(const std::string& command, const std::vector<std::string>& pos, const Args& args) {
    for (const char* name : {"--timeout", "--max-speed", "--max-accel", "--max-push", "--speed", "--accel", "--band",
                             "--speed-pct", "--sim-time-scale", "--decel", "--change-speed", "--change-accel",
                             "--change-after", "--interval", "--duration", "--target", "--wait", "--position-unit"})
        args.number(name);
    for (const char* name : {"--slave", "--axis", "--baud", "--retries", "--sim-baud", "--count", "--restore-baud"})
        args.integer(name);
    if (command == "move") parse_number("move POS", pos.at(0));
    if (command == "jog") parse_number("jog DELTA", pos.at(0));
    if (command == "push") {
        parse_number("push POS", pos.at(0));
        parse_number("push BAND", pos.at(1));
        parse_number("push PCT", pos.at(2));
    }
    if (const auto pattern = args.value("--accel-pattern"))
        if (!accel_pattern_from_string(*pattern))
            throw UsageError("--accel-pattern: trapezoid, s_motion or filter, got '" + *pattern + "'");
    if (const auto unit = args.number("--position-unit"))
        if (*unit != 0.01 && *unit != 0.001) throw UsageError("--position-unit: 0.01 or 0.001");
    if (const auto line = args.value("--line"))
        if (*line != "auto" && *line != "direct" && *line != "rcon" && *line != "robonet")
            throw UsageError("--line: auto, direct, rcon or robonet, got '" + *line + "'");
    if (const auto sim_line = args.value("--sim-line"))
        if (*sim_line != "rcon" && *sim_line != "robonet")
            throw UsageError("--sim-line: rcon or robonet, got '" + *sim_line + "'");
    if (const auto rate = args.integer("--restore-baud")) {
        bool verified = false;
        for (int r : kVerifiedBaudRates) verified = verified || r == *rate;
        if (!verified)
            throw UsageError("--restore-baud: 115200 or 230400 (the verified codes), got " + std::to_string(*rate));
    }
    if (command == "info")
        for (const auto& s : args.every("--section")) {
            const auto& names = info::sections();
            if (std::find(names.begin(), names.end(), s) == names.end())
                throw UsageError("--section: choose from " + util::join(names, ", ") + ", got '" + s + "'");
        }
    if (command == "backup") {
        if (pos.size() > 1) throw UsageError("'backup' takes at most one argument (FILE)");
        try {
            info::BackupRows::parse(args.value("--rows").value_or("used"));
        } catch (const std::invalid_argument& e) {
            throw UsageError(e.what());
        }
    }
    if (command == "goto") parse_integer("goto N", pos.at(0));
    if (command == "teach") parse_integer("teach N", pos.at(0));
    if (command == "baud") parse_integer("baud RATE", pos.at(0));
}

void check_param_args(const std::vector<std::string>& pos, const Args& args) {
    if (pos.empty()) throw UsageError("param: list, get KEY or set KEY VALUE");
    std::size_t count = 0;
    if (pos[0] == "list")
        count = 1;
    else if (pos[0] == "get")
        count = 2;
    else if (pos[0] == "set")
        count = 3;
    else
        throw UsageError("param: list, get KEY or set KEY VALUE (got '" + pos[0] + "')");
    if (pos.size() != count)
        throw UsageError(std::string("usage: param ") + (count == 1 ? "list" : count == 2 ? "get KEY" : "set KEY VALUE"));
    if (pos[0] != "set") {
        std::vector<std::string> set_only;
        for (const char* o : {"--no-commit", "--allow-dangerous", "--allow-home-params", "--allow-unlisted", "--force"})
            if (args.has(o)) set_only.push_back(o);
        if (!set_only.empty())
            throw UsageError("'param " + pos[0] + "' does not take " + util::join(set_only, ", "));
    }
}

/// The line kind when the axis sits behind a gateway ("rcon", "robonet"), else "" (a direct line).
std::string gateway_line(Axis& axis) {
    const std::shared_ptr<ModbusRtu> bus = axis.bus();
    const std::shared_ptr<Line> line = bus ? bus->line() : nullptr;
    return line && line->is_gateway() ? to_string(line->kind()) : std::string();
}

int show(const std::optional<Status>& st, Axis& axis, bool json, std::ostream& out) {
    if (!st) return 0;
    if (json)
        write_json(out, to_json(*st));
    else
        text::status(*st, out, axis.units(), gateway_line(axis));
    return 0;
}

ParamValue param_value(const std::string& key, const std::string& value) {
    const std::string k = util::trim(key);
    const bool raw_item = k.find('.') != std::string::npos && !util::starts_with(util::lower(k), "no.");
    const std::string v = util::trim(value);
    if (raw_item && (util::starts_with(v, "0x") || util::starts_with(v, "0X"))) {
        if (const auto n = util::parse_int(v.substr(2), 16)) return ParamValue::of(static_cast<double>(*n));
    }
    return ParamValue::parse(v);
}

int execute(Axis& axis, const std::string& label, const std::string& command, const std::vector<std::string>& pos,
            const Args& args, std::ostream& out, std::ostream& err) {
    const bool json = args.has("--json");
    const CommandSpec& spec = command_specs().at(command);
    if (command == "check") return extra::check(axis, label, args, out, err);
    if (command == "line") return extra::line(axis, args, out);
    if (command == "backup") return extra::backup(axis, label, pos, args, out);
    if (command == "recover") return extra::recover(axis, label, args, out, err);
    if (command == "reset")
        err << "note: 'reset' is deprecated and means ALARM reset; use 'alarm-reset'. The software reset is "
               "'swreset'.\n";
    if (spec.state_changing || (command == "param" && pos.at(0) == "set")) {
        // Session start: identity, directory, soft limits and speed limit; Modbus control only at rest.
        const Status before = axis.status();
        if (!before.modbus_enabled && detail::at_rest_for_control(before))
            (json ? err : out) << "enabling Modbus control (PMSL) while the axis is at rest\n";
        axis.connect(true);
        // --fast: fast completion and the status monitor; the baud rate is kept (raise it with 'baud 230400').
        if (args.has("--fast")) axis.enable_fast(std::nullopt);
    }
    if (command == "info") return extra::info(axis, args, out);
    if (command == "monitor") return extra::monitor(axis, label, args, out, err);
    if (command == "push") return extra::push(axis, pos, args, out);
    if (command == "status") return show(axis.status(), axis, json, out);
    if (command == "identify") {
        const DeviceInfo info = axis.identify();
        if (json)
            write_json(out, to_json(info));
        else
            text::info(info, axis, out);
        return 0;
    }
    if (command == "limits") {
        const AxisRanges ranges = axis.limits();
        if (json)
            write_json(out, to_json(ranges));
        else
            text::limits(ranges, out);
        return 0;
    }
    if (command == "alarms") {
        const auto records = axis.read_alarm_history();
        if (json) {
            Json list = Json::array();
            for (const auto& r : records) list.push_back(to_json(r));
            write_json(out, list);
        } else {
            text::alarms(records, out);
        }
        return 0;
    }
    if (command == "servo") return show(axis.servo(pos.at(0) == "on"), axis, json, out);
    if (command == "home") return show(axis.home(true, std::nullopt, method_of(args)), axis, json, out);
    if (command == "alarm-reset" || command == "reset") return show(axis.reset_alarm(method_of(args)), axis, json, out);
    if (command == "move") {
        const double target = parse_number("move POS", pos.at(0));
        MoveOptions o;
        o.speed = args.number("--speed").value_or(20.0);
        o.accel_g = args.number("--accel").value_or(0.1);
        o.band = args.number("--band").value_or(0.1);
        o.wait = !args.has("--no-wait");
        o.decel_g = args.number("--decel");
        if (const auto pattern = args.value("--accel-pattern")) o.accel_pattern = accel_pattern_from_string(*pattern);
        const bool change = args.has("--change-speed") || args.has("--change-accel");
        const std::optional<Status> st = change ? extra::move_and_change(axis, target, o, args, err) : axis.move_to(target, o);
        if (!st) {
            if (json) {
                Json j = Json::object();
                j.set("sent", true).set("target", target).set("units", axis.units());
                write_json(out, j);
            } else {
                out << "move to " << util::fixed(target) << ' ' << axis.units() << " sent\n";
            }
            return 0;
        }
        return show(st, axis, json, out);
    }
    if (command == "jog") {
        MoveOptions o = MoveOptions::jog();
        o.speed = args.number("--speed").value_or(10.0);
        o.accel_g = args.number("--accel").value_or(0.1);
        return show(axis.jog(parse_number("jog DELTA", pos.at(0)), o), axis, json, out);
    }
    if (command == "stop") return show(axis.stop(true, 5.0), axis, json, out);
    if (command == "pause" || command == "resume") {
        const bool pausing = command == "pause";
        if (pausing)
            axis.pause();
        else
            axis.resume();
        if (json) {
            Json j = Json::object();
            j.set(pausing ? "paused" : "resumed", true);
            write_json(out, j);
        } else {
            out << (pausing ? "paused: the move is held and keeps its remaining travel; 'resume' continues it, "
                              "'cancel-pause' drops it (the pause stays set after this command ends)"
                            : "pause released: a paused move continues by itself")
                << '\n';
        }
        return 0;
    }
    if (command == "cancel-pause") return show(axis.cancel_paused_move(), axis, json, out);
    if (command == "goto") {
        const int row = parse_integer("goto N", pos.at(0));
        PositionMoveOptions o;
        o.speed_pct = args.number("--speed-pct");
        o.wait = !args.has("--no-wait");
        o.allow_above_caps = args.has("--allow-fast");
        if (o.speed_pct && *o.speed_pct > 0 && *o.speed_pct <= 100) {  // out of range: refused by move_to_position
            const int code = speed_override_code(*o.speed_pct);
            const double runs = speed_override_pct(code);
            if (std::fabs(runs - *o.speed_pct) > 0.05 * *o.speed_pct)
                err << util::format("note: --speed-pct %s: the row runs at %.2f %% of its stored speed "
                                    "(override code %d; the unit runs (code + 1) / 256)",
                                    util::g(*o.speed_pct).c_str(), runs, code)
                    << '\n';
        }
        const std::optional<Status> st = axis.move_to_position(row, o);
        if (!st) {
            if (json) {
                Json j = Json::object();
                j.set("started", true).set("row", row);
                write_json(out, j);
            } else {
                out << "stored position " << row << " started\n";
            }
            return 0;
        }
        return show(st, axis, json, out);
    }
    if (command == "mode") return show(axis.set_action_mode(pos.at(0) == "pc", safety_of(args)), axis, json, out);
    if (command == "teach") {
        NvWriteOptions o;  // --allow-nonvolatile: accepted and ignored (no longer needed)
        o.force = args.has("--force");
        const PositionWriteResult r =
            axis.teach_position(parse_integer("teach N", pos.at(0)), teach_method_of(args), o);
        if (json)
            write_json(out, to_json(r));
        else
            text::row_write(r, out, axis.units());
        return 0;
    }
    if (command == "param") {
        const bool is_ec = axis.is_ec();
        if (pos[0] == "list") {
            const std::vector<ParameterInfo> list = axis.parameters();
            if (json) {
                Json arr = Json::array();
                for (const auto& p : list) arr.push_back(to_json(p));
                write_json(out, arr);
            } else {
                const ParameterModel model = axis.parameter_model();
                text::parameters(list, is_ec ? &model : nullptr, out, axis.units(), is_ec, axis.product_family());
            }
            return 0;
        }
        if (pos[0] == "get") {
            write_json(out, to_json(axis.read_parameter(util::trim(pos.at(1)))));
            return 0;
        }
        ParameterWriteOptions o;  // --allow-dangerous / -unlisted / -home-params: ignored (no longer needed)
        o.commit = !args.has("--no-commit");
        o.force = args.has("--force");
        const ParameterWriteResult r =
            axis.write_parameter(util::trim(pos.at(1)), param_value(pos.at(1), pos.at(2)), o);
        if (json)
            write_json(out, to_json(r));
        else
            text::parameter_write(r, out, is_ec ? "P" : "No.");
        return 0;
    }
    if (command == "baud") {
        const int rate = parse_integer("baud RATE", pos.at(0));
        BaudChangeOptions o;
        const std::string method = args.value("--method").value_or("direct");
        if (method == "direct")
            o.method = BaudChangeMethod::Direct;
        else if (method == "break")
            o.method = BaudChangeMethod::Break;
        else
            throw UsageError("--method: direct or break, got '" + method + "'");
        const BaudChangeResult r = axis.set_baud_rate(rate, o);  // --allow-unverified-code: ignored (no longer needed)
        if (json)
            write_json(out, to_json(r));
        else
            text::baud(r, out);
        return 0;
    }
    if (command == "swreset") {
        const SoftwareResetResult r = axis.software_reset();
        if (json)
            write_json(out, to_json(r));
        else
            text::reset(r, out);
        return 0;
    }
    throw UsageError("unknown command '" + command + "'");
}

int interrupted_exit(std::ostream& err) {
    err << "interrupted (a decel stop was sent if motion had been commanded)\n";
    return 130;
}

int run_axis_command(const Args& args, std::ostream& out, std::ostream& err) {
    std::vector<std::string> positionals = args.positionals();
    std::optional<std::string> profile = args.value("--sim-profile");
    // "--sim gripper status": a profile name right after --sim is taken as the profile
    if (args.has("--sim") && !profile && positionals.size() >= 2 && sim_profile_from_string(positionals[0]) &&
        !command_specs().count(positionals[0])) {
        profile = positionals[0];
        positionals.erase(positionals.begin());
    }
    const std::string command = positionals.at(0);
    const auto it = command_specs().find(command);
    if (it == command_specs().end()) throw UsageError("unknown command '" + command + "'");
    const CommandSpec& spec = it->second;
    std::vector<std::string> allowed(kConnectionOptions);
    allowed.insert(allowed.end(), spec.options.begin(), spec.options.end());
    args.allow(command, allowed);
    const std::vector<std::string> pos(positionals.begin() + 1, positionals.end());
    if (spec.args >= 0 && static_cast<int>(pos.size()) != spec.args)
        throw UsageError(spec.args == 0 ? "'" + command + "' takes no argument"
                                        : "'" + command + "' takes " + std::to_string(spec.args) +
                                              " argument(s), got " + std::to_string(pos.size()));
    if (command == "param") check_param_args(pos, args);
    if (command == "servo" && pos[0] != "on" && pos[0] != "off") throw UsageError("servo: on or off");
    if (command == "mode" && pos[0] != "pc" && pos[0] != "pio") throw UsageError("mode: pc or pio");
    if (command == "home" || command == "alarm-reset" || command == "reset") method_of(args);
    if (command == "teach") teach_method_of(args);
    const std::optional<CommandMethod> strategy = strategy_of(args);
    check_numbers(command, pos, args);
    if (command == "move" && (args.has("--change-speed") || args.has("--change-accel")) &&
        (args.has("--no-wait") || args.has("--decel"))) {
        err << "error: --change-speed / --change-accel need a plain move that waits (no --no-wait, no --decel)\n";
        return 2;
    }
    const bool sim = args.has("--sim");
    const std::optional<std::string> port_name = args.value("--port");
    if (sim == port_name.has_value()) throw UsageError("give exactly one of --port NAME and --sim");
    if (!sim && profile) throw UsageError("--sim-profile needs --sim");
    if (args.has("--slave") && args.has("--axis")) throw UsageError("--slave and --axis exclude each other");
    const int slave = args.has("--axis") ? *args.integer("--axis") + 1 : args.integer("--slave").value_or(1);
    if (slave < 1 || slave > 247) {
        err << "error: slave address must be 1..247 (axis 0..246), got " << slave << '\n';
        return 2;
    }
    OpenOptions oo;
    oo.slave = slave;
    oo.baud_rate = args.integer("--baud");
    oo.read_timeout_s = read_timeout(args);
    oo.bus = bus_options(args, err);
    oo.axis = axis_options(args);
    oo.line.kind = args.value("--line").value_or("auto");
    oo.line.position_unit_mm = args.number("--position-unit");
    std::unique_ptr<Axis> axis;
    std::shared_ptr<SimulatedUnit> sim_unit;
    const std::optional<std::string> sim_state = args.value("--sim-state");
    const std::optional<std::string> sim_line = args.value("--sim-line");
    if (sim_state && !sim) throw UsageError("--sim-state needs --sim");
    if (sim_line && !sim) throw UsageError("--sim-line needs --sim");
    if (sim_line && sim_state) throw UsageError("--sim-line and --sim-state exclude each other");
    const std::string label = sim ? std::string("sim") : *port_name;
    try {
        if (sim && sim_line) {
            const SimulatedUnitOptions so = sim_options(args, profile.value_or("slider"), slave);
            std::shared_ptr<Transport> line;
            if (*sim_line == "rcon")
                line = sim_rcon_line({slave}, SimGatewayOptions(), so);
            else
                line = sim_robonet_line({slave}, false, so);
            axis = std::make_unique<Axis>(Axis::open(line, oo));
        } else if (sim) {
            SimulatedUnitOptions so = sim_options(args, profile.value_or("slider"), slave);
            const auto rows = sim_state ? load_sim_state(*sim_state, so)
                                        : std::vector<std::pair<int, std::vector<std::uint16_t>>>{};
            sim_unit = std::make_shared<SimulatedUnit>(so);
            for (const auto& row : rows) sim_unit->set_position_row(row.first, row.second);
            axis = std::make_unique<Axis>(Axis::open(sim_unit, oo));
        } else {
            axis = std::make_unique<Axis>(Axis::open(*port_name, oo));
        }
        if (strategy) axis->set_strategy(*strategy);  // explicit: coil stays coil on a line that names another
        const std::shared_ptr<ModbusRtu> bus = axis->bus();
        const std::shared_ptr<Line> line = bus ? bus->line() : nullptr;
        if (line && line->is_gateway())
            err << "note: line " << to_string(line->kind()) << " (" << line->rules().title << "); "
                << line->rules().evidence << '\n';
        if (!sim)
            err << "note: " << *port_name << " at " << axis->baud_rate().value_or(0) << " baud ("
                << (axis->baud_detected() ? "auto-detected" : "fixed by --baud") << ")\n";
    } catch (const UsageError&) {
        throw;
    } catch (const std::exception& e) {
        report_error(err, e);
        return 2;
    }
    std::atomic<bool> done{false};
    std::thread watcher;
    if (spec.motion) {
        Axis* ax = axis.get();
        watcher = std::thread([ax, &done]() {
            while (!done.load()) {
                if (interrupted()) {
                    try {
                        ax->stop(false, 5.0);
                    } catch (...) {
                    }
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }
    auto finish_watch = [&]() {
        done = true;
        if (watcher.joinable()) watcher.join();
    };
    int code = 2;
    auto keep_sim_state = [&]() {
        if (sim_state && sim_unit) save_sim_state(*sim_state, *sim_unit);
    };
    try {
        code = execute(*axis, label, command, pos, args, out, err);
        finish_watch();
        keep_sim_state();
    } catch (const UsageError&) {
        finish_watch();
        throw;
    } catch (const std::exception& e) {
        finish_watch();
        keep_sim_state();
        if (interrupted()) return interrupted_exit(err);
        report_error(err, e);
        return 2;
    }
    if (interrupted() && !(command == "monitor" && args.has("--watch"))) return interrupted_exit(err);
    try {
        axis->close();
    } catch (const std::exception& e) {
        report_error(err, e);
        return 2;
    }
    return code;
}

}  // namespace

void request_interrupt() noexcept { g_interrupt.store(true); }
bool interrupted() noexcept { return g_interrupt.load(); }

std::string usage() { return kHelp; }

int run(const std::vector<std::string>& argv, std::ostream& out, std::ostream& err) {
    g_interrupt.store(false);
    try {
        const Args args = Args::parse(argv);
        if (args.has("--version")) {
            out << kProgram << ' ' << kVersion << '\n' << kDisclaimer << '\n';
            return 0;
        }
        if (args.has("--help")) {
            out << usage();
            return 0;
        }
        if (args.positionals().empty()) return usage_error(err, "a command is required");
        const std::string& command = args.positionals()[0];
        if (command == "ports") return run_ports(args, out);
        if (command == "scan") return run_scan(args, out, err);
        if (command == "stop-all") return run_stop_all(args, out, err);
        if (args.has("--sim") && args.positionals().size() >= 2 && args.positionals()[1] == "scan" &&
            sim_profile_from_string(command))
            return run_scan(args, out, err);
        return run_axis_command(args, out, err);
    } catch (const UsageError& e) {
        return usage_error(err, e.what());
    } catch (const std::exception& e) {
        report_error(err, e);
        return 2;
    } catch (...) {
        err << "error: unknown error\n";
        return 2;
    }
}

}  // namespace cli
}  // namespace iai_actuator_driver
