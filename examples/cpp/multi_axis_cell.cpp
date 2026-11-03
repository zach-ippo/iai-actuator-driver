// multi_axis_cell.cpp - several axes as one Cell through the C++ API: open, read, servo, home, move together, stop
// during a move, and the per-axis results of a group command.
//
// Shows Cell::open over several ports (one axis per COM port is the normal setup, VERIFIED; several axes on one port,
// by address, are UNVERIFIED on hardware), limits_all, status_all, servo_all, home_all, move_all (one speed for every
// axis), move_each (a speed per axis), stop_all during a move, and the fault policy: when the link of one axis fails
// during a group command, on_fault is called once for that axis and, with stop_on_fault (the default), every other
// axis gets a decel STOP. Every position comes from the unit's own range, so the same program runs on any mix of
// units (slider, rod, gripper, a CON controller ...).
//
// Run it (close IA-OS first):
//
//     multi_axis_cell --sim                                  three simulated ports: slider, rod, gripper
//     multi_axis_cell --sim --sim-profiles slider,pcon       one simulated port per profile
//     multi_axis_cell --port COM3 --port COM4                two IAI USB adapters, each scanned for axes
//     multi_axis_cell --port COM3:1 --port COM3:2            exactly these addresses on one port
//
// Options: --speed UNITS_S (default 20, raised to a unit's documented minimum speed where that is higher), --step
// UNITS (default 10, at most a quarter of each stroke), --home (home every axis, not only those that report no home).
//
// Motion: around the middle of each unit's stroke, --step apart: values for a first run on unloaded units. Homing
// runs each axis to its mechanical end: keep the travel clear. At the end every servo is switched off. On Ctrl+C or
// an error every axis gets a decel STOP and the servos are left as they are (a vertical axis without a brake can drop
// when its servo goes off). Exit codes: 0 done, 2 error, 130 interrupted.
//
// Build: CMake (examples/CMakeLists.txt), or link the static library directly, e.g.
//
//     zig c++ -std=c++17 -I core/include examples/cpp/multi_axis_cell.cpp core/build/libiai_actuator_driver.a
//             -lwinmm -lsetupapi -ladvapi32 -o multi_axis_cell.exe
//
// Unofficial: not an IAI product. The driver is not a safety function: keep a hardware emergency stop.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "iai_actuator_driver/iai_actuator_driver.hpp"

namespace iad = iai_actuator_driver;

namespace {

const char* const kBanner = "iai-actuator-driver example: multi-axis cell, C++ API (UNOFFICIAL - not an IAI product)";
constexpr double kDefaultSpeed = 20.0;  // units/s for a first run
constexpr double kDefaultStep = 10.0;   // units between the low and the high position

// ------------------------------------------------------------------------------------------------ Ctrl+C

volatile std::sig_atomic_t g_signal = 0;

extern "C" void on_signal(int) { g_signal = 1; }

/// Calls `stop` from its own thread once Ctrl+C is pressed (the signal handler only sets a flag; Cell::stop_all may be
/// called while another thread waits in a group command).
class StopOnCtrlC {
public:
    explicit StopOnCtrlC(std::function<void()> stop) : stop_(std::move(stop)) {
        std::signal(SIGINT, on_signal);
#ifdef SIGBREAK
        std::signal(SIGBREAK, on_signal);  // Windows: Ctrl+Break
#endif
        thread_ = std::thread([this] { watch(); });
    }
    ~StopOnCtrlC() {
        done_ = true;
        thread_.join();
    }
    StopOnCtrlC(const StopOnCtrlC&) = delete;
    StopOnCtrlC& operator=(const StopOnCtrlC&) = delete;

    bool interrupted() const { return g_signal != 0; }
    /// Wait until the STOP returned (it holds until standstill), so the ports are closed after it.
    void wait_stopped() const {
        while (interrupted() && !stopped_) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    /// Throw between steps after Ctrl+C.
    void check() const {
        if (interrupted()) throw Interrupted{};
    }
    struct Interrupted {};

private:
    void watch() {
        while (!done_) {
            if (g_signal && !stopped_) {
                try {
                    stop_();
                } catch (const std::exception& e) {
                    std::cerr << "STOP after Ctrl+C failed: " << e.what() << "\n";
                }
                stopped_ = true;
                std::signal(SIGINT, SIG_DFL);  // a second Ctrl+C ends the program at once
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::function<void()> stop_;
    std::atomic<bool> done_{false};
    std::atomic<bool> stopped_{false};
    std::thread thread_;
};

// ------------------------------------------------------------------------------------------------ command line

struct Args {
    std::vector<std::string> ports;  // --port, repeated; empty = --sim
    std::vector<std::string> sim_profiles{"slider", "rod", "gripper"};
    std::optional<double> speed;
    double step = kDefaultStep;
    bool home = false;
};

[[noreturn]] void usage(const std::string& problem) {
    if (!problem.empty()) std::cerr << "error: " << problem << "\n";
    std::cerr << "usage: multi_axis_cell (--sim [--sim-profiles A,B,...] | --port PORT[:SLAVE] ...) [--speed UNITS_S]\n"
                 "                       [--step UNITS] [--home]\n";
    std::exit(problem.empty() ? 0 : 2);
}

std::vector<std::string> split(const std::string& text, char sep) {
    std::vector<std::string> out;
    std::stringstream in(text);
    for (std::string item; std::getline(in, item, sep);) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

double positive(const std::string& option, const std::string& text) {
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !(value > 0.0) || !std::isfinite(value)) {
        usage(option + " takes a number above 0");
    }
    return value;
}

Args parse_args(int argc, char** argv) {
    Args a;
    bool sim = false;
    for (int i = 1; i < argc; ++i) {
        const std::string opt = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) usage(opt + " needs a value");
            return argv[++i];
        };
        if (opt == "--sim") sim = true;
        else if (opt == "--home") a.home = true;
        else if (opt == "-h" || opt == "--help") usage("");
        else if (opt == "--port") a.ports.push_back(value());
        else if (opt == "--sim-profiles") a.sim_profiles = split(value(), ',');
        else if (opt == "--speed") a.speed = positive(opt, value());
        else if (opt == "--step") a.step = positive(opt, value());
        else usage("unknown option " + opt);
    }
    if (sim == !a.ports.empty()) usage("give --sim or one or more --port PORT");
    if (sim && (a.sim_profiles.empty() || a.sim_profiles.size() > 16)) usage("--sim-profiles takes 1..16 names");
    return a;
}

// ------------------------------------------------------------------------------------------------ the example

/// on_fault: called once per axis whose link fails during a group command (port_vanished or unit_silent).
void on_fault(const std::string& name, const std::string& kind, const std::string& message) {
    std::printf("on_fault: %s: %s (%s)\n", name.c_str(), kind.c_str(), message.c_str());
    std::fflush(stdout);
}

/// The cell: one simulated port per profile ("SIM1".."SIMn", one unit at address 1 each), or the given ports.
iad::Cell open_cell(const Args& args) {
    iad::CellOptions options;
    options.on_fault = on_fault;  // stop_on_fault stays on: a link fault on one axis stops the others
    if (args.ports.empty()) {
        std::map<std::string, std::shared_ptr<iad::Transport>> lines;
        for (std::size_t i = 0; i < args.sim_profiles.size(); ++i) {
            const auto profile = iad::sim_profile_from_string(args.sim_profiles[i]);
            if (!profile) usage("unknown simulator profile " + args.sim_profiles[i]);
            iad::SimulatedUnitOptions sim;
            sim.profile = *profile;
            sim.pmsl = false;  // PIO in control, as after a power-up: the driver takes Modbus control at rest
            lines["SIM" + std::to_string(i + 1)] = std::make_shared<iad::SimulatedUnit>(sim);
        }
        return iad::Cell::open(lines, {1}, options);
    }
    // "COM3:1" names one address; "COM3" scans the port (addresses 1..16). Either every entry names one or none does.
    std::map<std::string, std::vector<int>> slaves;
    for (const auto& entry : args.ports) {
        const auto colon = entry.rfind(':');
        if (colon == std::string::npos) continue;
        slaves[entry.substr(0, colon)].push_back(std::atoi(entry.c_str() + colon + 1));
    }
    if (slaves.empty()) return iad::Cell::open(args.ports, options);
    return iad::Cell::open(slaves, options);
}

std::string fmt(const char* format, double value) {
    char buf[64];
    std::snprintf(buf, sizeof buf, format, value);
    return buf;
}

/// Where one axis works: low, middle and high, around the middle of its own range.
struct Plan {
    double low = 0.0;
    double middle = 0.0;
    double high = 0.0;
    double speed = kDefaultSpeed;
};

Plan plan_for(const iad::AxisRanges& r, const Args& args, const std::string& name) {
    if (!r.position.min || !r.position.max) {
        throw iad::EcNotReadyError(name + ": the unit's soft limits are not readable: set AxisOptions::stroke");
    }
    Plan p;
    const double low = *r.position.min;
    const double high = *r.position.max;
    const double half = std::floor(std::min(args.step, (high - low) / 4.0) / 2.0 * 100.0) / 100.0;
    p.middle = std::round((low + high) / 2.0 * 100.0) / 100.0;
    p.low = p.middle - half;
    p.high = p.middle + half;
    // below the documented minimum speed a unit may run noisily or unsteadily (the driver only warns)
    p.speed = args.speed.value_or(std::max(kDefaultSpeed, r.speed.recommended_min.value_or(0.0)));
    return p;
}

void print_status(iad::Cell& cell, const std::string& title) {
    std::printf("%s\n", title.c_str());
    const auto all = cell.status_all();
    for (const auto& name : all.names) {
        const iad::Status& st = all.at(name);
        std::string flags;
        for (const auto& f : {std::make_pair("servo ON", st.servo_on), std::make_pair("homed", st.homed),
                              std::make_pair("in position", st.in_position), std::make_pair("moving", st.moving),
                              std::make_pair("PC control", st.modbus_enabled)}) {
            if (f.second) flags += std::string(flags.empty() ? "" : ", ") + f.first;
        }
        if ((st.alarm || st.warning()) && st.alarm_code != 0) {
            const iad::AlarmInfo info = iad::alarm_info(st.alarm_code);
            flags += std::string("; ") + (st.alarm ? "alarm " : "warning ") + info.hex + " " + info.name;
        }
        std::printf("  %-8s %8.2f %-3s %s\n", name.c_str(), st.position_mm, cell.units_of(name).c_str(), flags.c_str());
    }
    std::fflush(stdout);
}

/// "SIM1:1 65.00, SIM2:1 65.00": where each axis of a group result is.
std::string positions(const iad::CellResults<iad::Status>& results) {
    std::string text;
    for (const auto& name : results.names) {
        if (!text.empty()) text += ", ";
        text += name + " " + (results.contains(name) ? fmt("%.2f", results.at(name).position_mm) : "failed");
    }
    return text;
}

double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

/// move_each: every axis to its own target at its own speed, started together; waits for all.
void move_each(iad::Cell& cell, const std::map<std::string, Plan>& plan, double Plan::*target, const char* label) {
    std::vector<iad::AxisMove> moves;
    for (const auto& item : plan) {
        iad::AxisMove m;
        m.axis = item.first;
        m.position = item.second.*target;
        m.options.speed = item.second.speed;
        moves.push_back(m);
    }
    const auto started = std::chrono::steady_clock::now();
    const auto done = cell.move_each(moves);  // checked first: one bad value refuses every move, nothing sent
    std::printf("move_each to the %s positions: %s (%.2f s)\n", label, positions(*done).c_str(), seconds_since(started));
    std::fflush(stdout);
}

void run(iad::Cell& cell, const Args& args, const StopOnCtrlC& ctrl_c) {
    std::printf("cell: %zu %s\n", cell.size(), cell.size() == 1 ? "axis" : "axes");
    const auto info = cell.info();          // identify() of every axis, kept from open
    const auto limits = cell.limits_all();  // each unit's own ranges, units and kind
    std::map<std::string, Plan> plan;
    for (const auto& name : cell.names()) {
        plan[name] = plan_for(limits.at(name), args, name);
        const Plan& p = plan[name];
        const std::string units = cell.units_of(name);
        const auto i = info.find(name);
        std::printf("  %-8s %s (%s, %s); low %.2f, middle %.2f, high %.2f %s at %g %s/s\n", name.c_str(),
                    i != info.end() ? i->second.model().c_str() : "?", limits.at(name).family.c_str(),
                    limits.at(name).kind.c_str(), p.low, p.middle, p.high, units.c_str(), p.speed, units.c_str());
    }
    for (const auto& skipped : cell.port_errors()) {
        std::printf("  skipped port %s: %s\n", skipped.first.c_str(), skipped.second.c_str());
    }
    print_status(cell, "status_all");
    ctrl_c.check();

    std::printf("servo_all(true)\n");
    cell.servo_all(true);
    ctrl_c.check();
    std::vector<std::string> homes;
    const auto before = cell.status_all();
    for (const auto& name : before.names) {
        if (args.home || !before.at(name).homed) homes.push_back(name);
    }
    if (!homes.empty()) {
        std::printf("home_all: %zu %s (each runs to its mechanical end)\n", homes.size(),
                    homes.size() == 1 ? "axis" : "axes");
        std::fflush(stdout);
        cell.home_all(homes);
        ctrl_c.check();
    } else {
        std::printf("home_all: not needed, every axis reports its home (--home homes them anyway)\n");
    }

    // move_all: one set of options for every axis. The common speed must suit every unit: the highest first-run speed.
    std::map<std::string, double> middles;
    double common = 0.0;
    for (const auto& item : plan) {
        middles[item.first] = item.second.middle;
        common = std::max(common, item.second.speed);
    }
    iad::MoveOptions move;
    move.speed = common;
    auto started = std::chrono::steady_clock::now();
    const auto at_middle = cell.move_all(middles, move);
    std::printf("move_all to the middle at %g units/s: %s (%.2f s)\n", common, positions(*at_middle).c_str(),
                seconds_since(started));
    ctrl_c.check();

    move_each(cell, plan, &Plan::low, "low");
    ctrl_c.check();
    move_each(cell, plan, &Plan::high, "high");
    ctrl_c.check();

    // stop_all during a move: start the moves (wait false), then a decel STOP on every axis.
    std::vector<iad::AxisMove> moves;
    for (const auto& item : plan) {
        iad::AxisMove m;
        m.axis = item.first;
        m.position = item.second.low;
        m.options.speed = item.second.speed;
        moves.push_back(m);
    }
    cell.move_each(moves, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const auto stopped = cell.stop_all();
    std::printf("stop_all during a move to the low positions: stopped at %s\n", positions(stopped).c_str());
    ctrl_c.check();

    print_status(cell, "status_all");
    std::printf("servo_all(false)\n");
    cell.servo_all(false);
}

/// Print an error; a CellError lists the outcome of every axis that failed. Returns 2.
int report(const std::exception_ptr& error) {
    std::fflush(stdout);
    try {
        std::rethrow_exception(error);
    } catch (const iad::CellError& e) {
        std::cerr << "error: " << e.operation() << " failed on " << e.messages().size() << " axis(es)\n";
        for (const auto& m : e.messages()) std::cerr << "       " << m.first << ": " << m.second << "\n";
    } catch (const std::exception& e) {  // iad::EcDriverError and the rest (std::invalid_argument for a bad value)
        std::cerr << "error: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "error: unknown exception\n";
    }
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);
    std::puts(kBanner);
    std::optional<iad::Cell> cell;
    std::atomic<iad::Cell*> to_stop{nullptr};  // the cell the Ctrl+C thread stops, once open
    StopOnCtrlC ctrl_c([&to_stop] {
        if (iad::Cell* c = to_stop.load()) c->stop_all({}, 5.0, true);  // every axis, even when one fails
    });
    int code = 0;
    try {
        cell.emplace(open_cell(args));
        to_stop = &*cell;
        run(*cell, args, ctrl_c);
    } catch (...) {
        code = ctrl_c.interrupted() ? 130 : report(std::current_exception());
    }
    if (ctrl_c.interrupted()) {
        ctrl_c.wait_stopped();
        std::fflush(stdout);
        std::cerr << "interrupted (Ctrl+C): a decel STOP was sent to every axis; the servos are left as they are\n";
        code = 130;
    } else if (code != 0 && cell) {
        const auto results = cell->stop_all({}, 5.0, true);  // return_exceptions: try every axis
        std::cerr << "       decel STOP sent to every axis; the servos are left as they are\n";
        for (const auto& m : results.messages) std::cerr << "       STOP not confirmed on " << m.first << ": " << m.second << "\n";
    }
    if (cell) cell->close();
    if (code == 0) std::puts("done");
    return code;
}
