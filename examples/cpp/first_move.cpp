// first_move.cpp - one axis through the C++ API (include/iai_actuator_driver/iai_actuator_driver.hpp): open, connect,
// servo ON, home when needed, an absolute move and two relative moves around mid-stroke, the status, servo OFF.
//
// Run it (close IA-OS first: only one program can use a serial port):
//
//     first_move --sim                       the simulator (an ELECYLINDER slider), no hardware
//     first_move --sim --sim-profile rod     another simulated unit: slider, gripper, rod, pcon, rotary, unknown
//     first_move --port COM3                 the unit on COM3, address 1 (the baud rate is found automatically)
//     first_move --port COM3 --slave 2       the unit at address 2 on a shared port (address = axis number + 1)
//     first_move --port COM3 --home          a home return even when the status says homed
//
// Options: --speed UNITS_S (default 20, raised to the unit's documented minimum speed where that is higher),
// --distance UNITS (the step out and back, default 10, at most a quarter of the stroke).
//
// Every target and speed is checked against the unit's own ranges (Axis::limits()) before anything is sent: a value
// outside them throws SoftLimitError and nothing moves. Every driver error derives from EcDriverError; what() carries
// the driver's notes (for example whether a decel STOP was sent).
//
// Motion: to the middle of the stroke, then --distance further and back. The home return runs only when the status
// says the axis is not homed, or with --home: the axis runs to its mechanical end, so keep the travel clear. Test a
// new setup unloaded, and keep a hand on the emergency stop.
//
// End: the servo is switched OFF (a vertical axis without a brake can fall when the servo goes off: support it first)
// and Modbus control is handed back to PIO when PIO had it at the start. Ctrl+C: a watcher thread sends a decel STOP
// (Axis::stop may be called while another thread waits for a move; a signal handler must not call the driver), the
// waiting move then throws and the program closes the port with the servo left as it is. Exit codes: 0 done, 2 error,
// 130 interrupted.
//
// Build: CMake (examples/CMakeLists.txt), or link the static library directly, e.g.
//
//     zig c++ -std=c++17 -I core/include examples/cpp/first_move.cpp core/build/libiai_actuator_driver.a
//             -lwinmm -lsetupapi -ladvapi32 -o first_move.exe
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
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "iai_actuator_driver/iai_actuator_driver.hpp"

namespace iad = iai_actuator_driver;

namespace {

const char* const kBanner = "iai-actuator-driver example: first move, C++ API (UNOFFICIAL - not an IAI product)";
constexpr double kDefaultSpeed = 20.0;     // units/s: a first test (unloaded, mid-stroke, 20 mm/s or less)
constexpr double kDefaultDistance = 10.0;  // units: the step out and back

// ------------------------------------------------------------------------------------------------ Ctrl+C

volatile std::sig_atomic_t g_signal = 0;

extern "C" void on_signal(int) { g_signal = 1; }

/// Calls `stop` from its own thread once Ctrl+C is pressed. The signal handler only sets a flag; the driver is called
/// from this thread, which a blocking move on the main thread does not hold up.
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

    /// Ctrl+C was pressed (the STOP has been sent or is being sent).
    bool interrupted() const { return g_signal != 0; }
    /// Wait until the STOP returned (it holds until standstill), so the port is closed after it.
    void wait_stopped() const {
        while (interrupted() && !stopped_) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

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

/// Thrown between steps after Ctrl+C.
struct Interrupted {};

// ------------------------------------------------------------------------------------------------ command line

struct Args {
    std::optional<std::string> port;  // --port COM3; nullopt = --sim
    std::string sim_profile = "slider";
    int slave = 1;
    std::optional<double> speed;
    double distance = kDefaultDistance;
    bool home = false;
};

[[noreturn]] void usage(const std::string& problem) {
    if (!problem.empty()) std::cerr << "error: " << problem << "\n";
    std::cerr << "usage: first_move (--sim [--sim-profile NAME] | --port PORT) [--slave N] [--speed UNITS_S]\n"
                 "                  [--distance UNITS] [--home]\n";
    std::exit(problem.empty() ? 0 : 2);
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
        else if (opt == "--port") a.port = value();
        else if (opt == "--sim-profile") a.sim_profile = value();
        else if (opt == "--slave") a.slave = std::atoi(value().c_str());
        else if (opt == "--speed") a.speed = positive(opt, value());
        else if (opt == "--distance") a.distance = positive(opt, value());
        else usage("unknown option " + opt);
    }
    if (sim == a.port.has_value()) usage("give --sim or --port PORT (one of them)");
    if (a.slave < 1 || a.slave > 247) usage("--slave takes 1..247");
    return a;
}

// ------------------------------------------------------------------------------------------------ the example

/// The axis on --port (baud rate found automatically), or a simulated unit with --sim.
iad::Axis open_axis(const Args& args) {
    iad::OpenOptions open;
    open.slave = args.slave;
    if (args.port) return iad::Axis::open(*args.port, open);
    const auto profile = iad::sim_profile_from_string(args.sim_profile);
    if (!profile) usage("unknown --sim-profile " + args.sim_profile + " (slider, gripper, rod, pcon, rotary, unknown)");
    iad::SimulatedUnitOptions sim;
    sim.slave = args.slave;
    sim.profile = *profile;
    sim.pmsl = false;  // PIO in control, as after a power-up: connect() takes Modbus control as on hardware
    return iad::Axis::open(std::make_shared<iad::SimulatedUnit>(sim), open);
}

void show(const std::string& label, const std::string& text) {
    std::printf("%-10s %s\n", label.c_str(), text.c_str());
    std::fflush(stdout);
}

std::string fmt(const char* format, double value) {
    char buf[64];
    std::snprintf(buf, sizeof buf, format, value);
    return buf;
}

/// The status in one line: position and the flags that matter around a move.
std::string describe(const iad::Status& st, const std::string& units) {
    std::string text = "position " + fmt("%.2f", st.position_mm) + " " + units;
    const std::pair<const char*, bool> flags[] = {
        {"servo ON", st.servo_on}, {"homed", st.homed},          {"in position", st.in_position},
        {"moving", st.moving},     {"PC control", st.modbus_enabled}, {"emergency stop", st.emergency_stop}};
    for (const auto& f : flags) {
        if (f.second) text += std::string(", ") + f.first;
    }
    if ((st.alarm || st.warning()) && st.alarm_code != 0) {
        const iad::AlarmInfo info = iad::alarm_info(st.alarm_code);
        text += std::string("; ") + (st.alarm ? "alarm " : "warning ") + info.hex + " " + info.name;
    }
    return text;
}

void first_move(iad::Axis& ax, const Args& args, const StopOnCtrlC& ctrl_c, bool& motion) {
    auto step_done = [&] {
        if (ctrl_c.interrupted()) throw Interrupted{};
    };
    const iad::Status found = ax.status();       // read-only: the state before this program sent anything
    const iad::DeviceInfo info = ax.connect();   // identity and ranges; Modbus control is taken here, at rest only
    const iad::AxisRanges limits = ax.limits();  // every range the driver enforces, read from the unit itself
    const std::string units = limits.units;      // "mm", or "deg" on a rotary axis
    show("unit", info.model() + " (" + ax.product_family() + "; actuator: " + ax.kind() + "), firmware " +
                     info.firmware.value_or("?") + ", address " + std::to_string(ax.slave()) +
                     (ax.baud_rate() ? ", " + std::to_string(*ax.baud_rate()) + " baud" : std::string()) +
                     ", library " + iad::kVersion);
    if (!limits.position.min || !limits.position.max) {
        throw iad::EcNotReadyError("the unit's soft limits are not readable here: set AxisOptions::stroke");
    }
    const double low = *limits.position.min;
    const double high = *limits.position.max;
    // below the documented minimum speed a unit may run noisily or unsteadily (the driver only warns)
    const double speed = args.speed.value_or(std::max(kDefaultSpeed, limits.speed.recommended_min.value_or(0.0)));
    show("range", fmt("%.2f", low) + " .. " + fmt("%.2f", high) + " " + units + "; speed up to " +
                      fmt("%g", limits.speed.max.value_or(NAN)) + " " + units + "/s; " + fmt("%g", speed) + " " +
                      units + "/s for every move here");
    show("status", describe(found, units));
    step_done();

    // A servo ON the status shows cannot work (EMG, drive power cut) throws ServoBlockedError with reasons().
    iad::Status st = *ax.servo(true);
    show("servo", "ON");
    step_done();
    if (args.home || !st.homed) {
        show("home", "home return: the axis runs to its mechanical end, keep the travel clear");
        st = *ax.home();
        motion = true;
        show("", "homed at " + fmt("%.2f", st.position_mm) + " " + units);
        step_done();
    } else {
        show("home", "not needed: the status says homed (--home runs a home return anyway)");
    }

    const double middle = std::round((low + high) / 2.0 * 100.0) / 100.0;
    const double step = std::floor(std::min(args.distance, (high - low) / 4.0) * 100.0) / 100.0;
    iad::MoveOptions move;  // accel 0.1 G, band 0.1, wait until in position and settled: the defaults
    move.speed = speed;
    st = *ax.move_to(middle, move);
    motion = true;
    show("move", "to the middle of the stroke: at " + fmt("%.2f", st.position_mm) + " " + units);
    step_done();
    for (const double delta : {step, -step}) {
        st = *ax.move_by(delta, move);  // relative to the measured position
        show("move", "by " + fmt("%+.2f", delta) + " " + units + ": at " + fmt("%.2f", st.position_mm) + " " + units);
        step_done();
    }
    motion = false;

    show("status", describe(ax.status(), units));
    ax.servo(false);
    show("servo", "OFF");
    if (!found.modbus_enabled) {
        ax.disable_modbus_control();  // PIO had control when this program started: hand it back
        show("control", "handed back to PIO");
    }
}

/// Print an error with what the driver knows about it (the alarm, the reasons a servo cannot come on). Returns 2.
int report(const std::exception_ptr& error) {
    std::fflush(stdout);
    try {
        std::rethrow_exception(error);
    } catch (const iad::EcAlarmError& e) {
        const iad::AlarmInfo info = iad::alarm_info(e.alarm_code());
        std::cerr << "error: " << e.what() << "\n       alarm " << info.hex << " " << info.name << ": " << info.reset
                  << "\n";
    } catch (const iad::ServoBlockedError& e) {
        std::cerr << "error: " << e.what() << "\n";
        for (const auto& reason : e.reasons()) std::cerr << "       " << reason << "\n";
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
    std::optional<iad::Axis> ax;
    std::atomic<iad::Axis*> to_stop{nullptr};  // the axis the Ctrl+C thread stops, once open
    bool motion = false;  // a motion command ran: an error afterwards sends a decel STOP before the port closes
    StopOnCtrlC ctrl_c([&to_stop] {
        if (iad::Axis* a = to_stop.load()) a->stop();
    });
    int code = 0;
    try {
        ax.emplace(open_axis(args));
        to_stop = &*ax;
        first_move(*ax, args, ctrl_c, motion);
    } catch (...) {
        code = ctrl_c.interrupted() ? 130 : report(std::current_exception());
    }
    if (ctrl_c.interrupted()) {
        ctrl_c.wait_stopped();
        std::fflush(stdout);
        std::cerr << "interrupted (Ctrl+C): a decel STOP was sent; the servo is left as it is\n";
        code = 130;
    } else if (code != 0 && motion && ax) {
        try {
            ax->stop();  // an error after a motion command: decel STOP before the port closes
            std::cerr << "       a decel STOP was sent; the servo is left as it is\n";
        } catch (const std::exception& e) {
            std::cerr << "       decel STOP failed too: " << e.what() << "\n";
        }
    }
    if (ax) ax->close();
    if (code == 0) std::puts("done");
    return code;
}
