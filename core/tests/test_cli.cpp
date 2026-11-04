// The command line on the simulator (in process, through cli::run): read-only commands, a move, the non-volatile
// write guard as a user sees it (an unchanged parameter or row is not written; --force writes it), and Ctrl+C (raised
// from a bus hook as a frame is about to go). No COM port.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "harness.hpp"
#include "iai_actuator_driver/modbus.hpp"

using namespace iai_actuator_driver;

namespace {

struct CliRun {
    int code = -1;
    std::string out;
    std::string err;
    bool says(const std::string& text) const { return (out + err).find(text) != std::string::npos; }
};

CliRun cli_run(std::vector<std::string> argv) {
    std::ostringstream out, err;
    CliRun r;
    r.code = cli::run(argv, out, err);
    r.out = out.str();
    r.err = err.str();
    return r;
}

std::vector<std::string> sim_args(const std::string& state, std::vector<std::string> command) {
    std::vector<std::string> argv{"--sim", "--sim-profile", "gripper", "--sim-time-scale", "20", "--sim-state", state};
    argv.insert(argv.end(), command.begin(), command.end());
    return argv;
}

using Pdu = std::vector<std::uint8_t>;

bool is_direct_move(const Pdu& pdu) { return pdu.size() > 2 && pdu[0] == 0x10 && pdu[1] == 0x99 && pdu[2] == 0x00; }

/// The write frames on the wire (--log-frames "T tx HEX" lines; FC05 / FC06 / FC0F / FC10): hex, with the CRC.
std::vector<std::string> writes(const std::string& err) {
    std::vector<std::string> frames;
    std::istringstream lines(err);
    std::string line;
    while (std::getline(lines, line)) {
        const std::size_t at = line.find(" tx ");
        if (at == std::string::npos) continue;
        const std::string hex = line.substr(line.find_first_not_of(' ', at + 3));
        const std::string fc = hex.substr(3, 2);
        if (fc == "05" || fc == "06" || fc == "0f" || fc == "10") frames.push_back(hex);
    }
    return frames;
}

std::size_t count_prefix(const std::vector<std::string>& frames, const std::string& prefix) {
    return static_cast<std::size_t>(
        std::count_if(frames.begin(), frames.end(), [&](const std::string& f) { return f.rfind(prefix, 0) == 0; }));
}

constexpr const char* kDirectMove = "01 10 99 00";
constexpr const char* kStopOn = "01 05 04 2c ff 00";
constexpr const char* kStopOff = "01 05 04 2c 00 00";

/// Ctrl+C as the command line is about to send a request: `raise` runs once, at the first request `when`
/// picks (the bus's before_bus_wait hook, cleared at the end of the scope).
class InterruptAt {
public:
    InterruptAt(std::function<bool(const Pdu&)> when, std::function<void()> raise) {
        auto fired = std::make_shared<bool>(false);
        detail::bus_test_hooks().before_bus_wait = [when, raise, fired](int, const Pdu& pdu) {
            if (*fired || !when(pdu)) return;
            *fired = true;
            raise();
        };
    }
    ~InterruptAt() { detail::bus_test_hooks().before_bus_wait = nullptr; }
    InterruptAt(const InterruptAt&) = delete;
    InterruptAt& operator=(const InterruptAt&) = delete;
};

/// A move interrupted at its 0x9900 frame: exit 130, no 0x9900 on the wire, the STOP sent and released last.
void check_stopped_before_the_move(const CliRun& r) {
    CHECK_EQ(r.code, 130);
    CHECK(r.says("interrupted"));
    const std::vector<std::string> w = writes(r.err);
    CHECK_EQ(count_prefix(w, kDirectMove), std::size_t{0});
    CHECK(count_prefix(w, kStopOn) >= 1);
    CHECK(!w.empty() && w.back().rfind(kStopOff, 0) == 0);
}

}  // namespace

TEST(cli_read_only_commands_and_a_move_on_the_simulator) {
    CHECK_EQ(cli_run({"--sim", "status"}).code, 0);
    CHECK(cli_run({"--sim", "identify"}).says("soft limit"));
    CHECK(cli_run({"--sim", "--sim-profile", "gripper", "limits"}).says("recommended min 5"));
    const CliRun moved = cli_run({"--sim", "--sim-time-scale", "20", "move", "30", "--speed", "50"});
    CHECK_EQ(moved.code, 0);
    CHECK(moved.says("30.0"));
    const CliRun refused = cli_run({"--sim", "move", "99999"});
    CHECK_EQ(refused.code, 2);
    CHECK(refused.says("nothing sent"));
    CHECK_EQ(cli_run({"--sim", "frobnicate"}).code, 2);
    // the former opt-ins are accepted and ignored, and not in --help
    const CliRun help = cli_run({"--help"});
    CHECK(help.says("teach N [--force] [--method table|coil]"));
    for (const char* hidden : {"--allow-nonvolatile", "--allow-dangerous", "--allow-home-params", "--allow-unlisted",
                               "--allow-unverified-code"})
        CHECK(!help.says(hidden));
}

TEST(cli_goto_notes_a_rounded_speed_override) {
    // the unit runs (code + 1) / 256 of the row's stored speed (ON-WIRE on the PCON-CB): 0.1 % runs as code 1 = 0.78 %
    const CliRun slow = cli_run({"--sim", "--sim-profile", "pcon", "--unit-max", "--sim-time-scale", "50", "goto", "1",
                                 "--speed-pct", "0.1", "--no-wait"});
    CHECK_EQ(slow.code, 0);
    CHECK(slow.err.find("note: --speed-pct 0.1: the row runs at 0.78 % of its stored speed (override code 1;") !=
          std::string::npos);
    const CliRun exact = cli_run(
        {"--sim", "--sim-profile", "pcon", "--unit-max", "--sim-time-scale", "50", "goto", "1", "--speed-pct", "25"});
    CHECK_EQ(exact.code, 0);
    CHECK(!exact.says("note: --speed-pct"));  // 25 % runs as code 63 = 25.00 %
}

TEST(cli_position_unit_0_001_is_refused_on_a_direct_line) {
    // ON-WIRE on a PCON-CB: a stated 0.001 on a direct line read 94.99 as 9.499 (and targets would go x 10)
    for (const char* line : {"auto", "direct"}) {
        const CliRun r =
            cli_run({"--sim", "--sim-profile", "pcon", "--line", line, "--position-unit", "0.001", "status"});
        CHECK_EQ(r.code, 2);
        CHECK(r.says("position unit 0.001 mm refused on a direct line"));
    }
    CHECK_EQ(cli_run({"--sim", "--sim-profile", "pcon", "--position-unit", "0.01", "status"}).code, 0);
}

TEST(cli_unit_without_a_family_name_is_reported_without_a_label) {
    // the simulator's "unknown" unit (system 0x99, unit 0x7E): reported as what it says it is, and driven
    const CliRun ident = cli_run({"--sim", "--sim-profile", "unknown", "identify"});
    CHECK_EQ(ident.code, 0);
    CHECK(ident.says("unit 0x7E"));
    CHECK(!ident.says("UNVERIFIED"));
    CHECK(!ident.says("unit code not in"));  // no note on the unit code (the AKID kind note says what is done)
    CHECK(ident.says("AKID kind 42: commanded as a linear axis in mm"));
    CHECK(!ident.says("driver's table"));
    const CliRun info = cli_run({"--sim", "--sim-profile", "unknown", "info", "--section", "identity"});
    CHECK_EQ(info.code, 0);
    CHECK(!info.says("UNVERIFIED"));
    CHECK(!info.says("unit code not in"));
    const CliRun moved = cli_run({"--sim", "--sim-profile", "unknown", "--sim-time-scale", "20", "move", "5", "--speed",
                                  "20"});
    CHECK_EQ(moved.code, 0);
}

TEST(cli_nonvolatile_guard_skips_unchanged_writes) {
    const std::string state = (std::filesystem::temp_directory_path() / "iai_actuator_driver_cpp_test_cli_sim_state.txt").string();
    std::remove(state.c_str());
    CHECK_EQ(cli_run(sim_args(state, {"move", "7"})).code, 0);
    const CliRun first = cli_run(sim_args(state, {"teach", "1"}));  // no opt-in needed
    CHECK_EQ(first.code, 0);
    CHECK(first.says("taught"));
    const CliRun second = cli_run(sim_args(state, {"teach", "1", "--allow-nonvolatile"}));  // accepted, ignored
    CHECK_EQ(second.code, 0);
    CHECK(second.says("unchanged: nothing written (one non-volatile write cycle saved)"));
    const CliRun forced = cli_run(sim_args(state, {"teach", "1", "--allow-nonvolatile", "--force"}));
    CHECK(forced.says("written anyway (force)"));
    const CliRun same = cli_run(sim_args(state, {"param", "set", "P5", "0"}));
    CHECK_EQ(same.code, 0);
    CHECK(same.says("unchanged: nothing written (one non-volatile write cycle saved)"));
    const CliRun same_ram = cli_run(sim_args(state, {"param", "set", "P5", "0", "--no-commit"}));
    CHECK_EQ(same_ram.code, 0);
    CHECK(same_ram.says("unchanged, the unit already holds this value; nothing written (--no-commit)"));
    CHECK(!same_ram.says("RAM ONLY"));
    const CliRun changed = cli_run(sim_args(state, {"param", "set", "P5", "1", "--allow-dangerous"}));  // ignored
    CHECK_EQ(changed.code, 0);
    CHECK(changed.says("committed"));
    std::remove(state.c_str());
}

TEST(cli_teach_method_chooses_the_table_or_the_coil_sequence) {
    const std::string state =
        (std::filesystem::temp_directory_path() / "iai_actuator_driver_cpp_test_cli_teach_method.txt").string();
    std::remove(state.c_str());
    CHECK_EQ(cli_run(sim_args(state, {"move", "9"})).code, 0);
    auto teach = [&](const std::string& method) {
        // row 1: the gripper runs rows 0 and 1 only (2-position mode, P9 = 0)
        return cli_run(sim_args(state, {"--log-frames", "teach", "1", "--allow-nonvolatile", "--method", method}));
    };
    // --method coil: the EXPERIMENTAL MOD / POSR / TEAC sequence starts with MOD on (01 05 04 14 FF 00)
    CHECK(teach("coil").says("01 05 04 14 ff 00"));
    // --method table (the default): no teach coil, the row written and read back (from another position: the row the
    // simulator's teach coils just wrote holds 9.00 mm, which the table path would leave unchanged)
    CHECK_EQ(cli_run(sim_args(state, {"move", "5"})).code, 0);
    const CliRun table = teach("table");
    CHECK_EQ(table.code, 0);
    CHECK(table.says("taught"));
    CHECK(!table.says("01 05 04 14 ff 00"));
    CHECK(teach("row").says("--method: table or coil"));
    std::remove(state.c_str());
}

TEST(cli_ctrl_c_before_the_move_sends_no_motion_frame) {
    // ON-WIRE on a PCON-CB (driver 1.1.0): Ctrl+C before the 0x9900 frame sent STOP ON / OFF, then the move went out
    // ~1.3 s later and ran its full travel (exit 130)
    const std::vector<std::string> move{"--sim", "--sim-time-scale", "20", "--log-frames", "move", "30", "--speed",
                                        "50"};
    {
        const InterruptAt at([](const Pdu&) { return true; }, [] { cli::request_interrupt(); });  // the port opens
        const CliRun r = cli_run(move);
        CHECK_EQ(r.code, 130);
        CHECK(writes(r.err).empty());  // the command never started: nothing written
    }
    {
        const InterruptAt at(is_direct_move, [] { cli::request_interrupt(); });  // as the move frame is about to go
        check_stopped_before_the_move(cli_run(move));
    }
}
