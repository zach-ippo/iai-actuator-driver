// The commands of the command line added for parity with the Python CLI, on the simulator (in process, through
// cli::run): push, a change of the running move, --fast / --decel / --accel-pattern, monitor (--watch), info, backup,
// recover, line (direct and behind a simulated RCON gateway), check, stop-all (with the broadcast STOP checked on the
// Cell). Ports of the Python edition's test_cli.py / test_round2_cli.py cases. No COM port.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "harness.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/sim_line.hpp"

using namespace iai_actuator_driver;

namespace {

struct CliRun {
    int code = -1;
    std::string out;
    std::string err;
    bool says(const std::string& text) const { return (out + err).find(text) != std::string::npos; }
    bool out_says(const std::string& text) const { return out.find(text) != std::string::npos; }
    bool err_says(const std::string& text) const { return err.find(text) != std::string::npos; }
    /// The "position   NN.NN mm" of a status printout (NaN without one).
    double position() const {
        const std::size_t at = out.find("position ");
        double mm = 0;
        return at != std::string::npos && std::sscanf(out.c_str() + at, "position %lf", &mm) == 1 ? mm : std::nan("");
    }
};

CliRun cli_run(std::vector<std::string> argv) {
    std::ostringstream out, err;
    CliRun r;
    r.code = cli::run(argv, out, err);
    r.out = out.str();
    r.err = err.str();
    return r;
}

std::string temp_file(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("iai_actuator_driver_cpp_test_" + name)).string();
}

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

bool has_frame(const std::vector<std::pair<double, std::vector<std::uint8_t>>>& requests,
               const std::vector<std::uint8_t>& frame) {
    for (const auto& r : requests)
        if (r.second == frame) return true;
    return false;
}

}  // namespace

TEST(cli_parity_help_lists_the_new_commands_and_options) {
    const CliRun help = cli_run({"--help"});
    CHECK_EQ(help.code, 0);
    for (const char* word : {"push", "monitor --watch", "info", "backup", "recover", "line", "check", "stop-all",
                             "--decel", "--accel-pattern", "--fast", "--change-speed", "--line", "--position-unit",
                             "--broadcast", "'reset' is a deprecated alias", "mode pc|pio [--safety [on|off]]",
                             "--strategy coil|iai|iai2|register", "--method table|coil"})
        CHECK(help.out_says(word));
}

TEST(cli_parity_push_missed_and_refused_in_the_zone) {
    const CliRun missed = cli_run({"--sim", "--sim-time-scale", "20", "push", "80", "10", "30"});
    CHECK_EQ(missed.code, 0);
    CHECK(missed.out_says("push missed at 90.00 mm"));
    const CliRun inside = cli_run({"--sim", "push", "70", "10", "30"});
    CHECK_EQ(inside.code, 2);
    CHECK(inside.err_says("push zone"));
    const CliRun json = cli_run({"--sim", "--sim-time-scale", "20", "--json", "push", "80", "10", "30"});
    CHECK_EQ(json.code, 0);
    CHECK_EQ(Json::parse(json.out).find("outcome")->as_string(), std::string("missed"));
    CHECK_EQ(cli_run({"--sim", "push", "80", "10"}).code, 2);  // PCT missing
}

TEST(cli_parity_change_speed_of_the_running_move) {
    const CliRun r = cli_run({"--sim", "--sim-time-scale", "2", "--max-speed", "200", "move", "60", "--speed", "10",
                              "--accel", "0.3", "--change-speed", "80", "--change-after", "0.2"});
    CHECK_EQ(r.code, 0);
    CHECK(r.err_says("changing the speed of the running move to 80"));
    CHECK(r.out_says("60.00 mm"));
}

TEST(cli_parity_change_options_need_a_plain_waiting_move) {
    for (const auto& extra : std::vector<std::vector<std::string>>{{"--no-wait"}, {"--decel", "0.1"}}) {
        std::vector<std::string> argv{"--sim", "move", "60", "--change-speed", "50"};
        argv.insert(argv.end(), extra.begin(), extra.end());
        const CliRun r = cli_run(argv);
        CHECK_EQ(r.code, 2);
        CHECK(r.err_says("plain move that waits"));
    }
}

TEST(cli_parity_change_speed_outside_the_range_is_refused) {
    const CliRun r = cli_run({"--sim", "--sim-time-scale", "2", "move", "60", "--speed", "10", "--accel", "0.3",
                              "--change-speed", "5000", "--change-after", "0.2"});
    CHECK_EQ(r.code, 2);
    CHECK(r.err_says("SoftLimitError"));
}

TEST(cli_parity_fast_decel_and_accel_pattern_moves) {
    // --fast completes inside the 0.1 mm band, so the printed position is wherever the axis was then
    const CliRun fast = cli_run({"--sim", "--sim-time-scale", "20", "move", "40", "--fast"});
    CHECK_EQ(fast.code, 0);
    CHECK_NEAR(fast.position(), 40.0, 0.1);
    const CliRun decel = cli_run({"--sim", "--sim-time-scale", "20", "move", "40", "--accel", "0.2", "--decel", "0.1"});
    CHECK_EQ(decel.code, 0);
    CHECK(decel.out_says("40.00 mm"));
    const CliRun pattern =
        cli_run({"--sim", "--sim-profile", "pcon", "--sim-time-scale", "20", "move", "40", "--accel-pattern", "s_motion"});
    CHECK_EQ(pattern.code, 0);
    const CliRun bad = cli_run({"--sim", "move", "40", "--accel-pattern", "sine"});
    CHECK_EQ(bad.code, 2);
    CHECK(bad.err_says("--accel-pattern"));
    const CliRun jog = cli_run({"--sim", "--sim-time-scale", "20", "jog", "5", "--fast"});
    CHECK_EQ(jog.code, 0);
    CHECK_NEAR(jog.position(), 78.95, 0.1);
}

TEST(cli_parity_monitor_reads_and_json) {
    const CliRun two = cli_run({"--sim", "monitor", "--count", "2", "--interval", "0.05"});
    CHECK_EQ(two.code, 0);
    std::size_t lines = 0;
    for (std::size_t at = 0; (at = two.out.find("position 73.95 mm", at)) != std::string::npos; ++at) ++lines;
    CHECK_EQ(lines, std::size_t(2));
    CHECK(two.out_says("deviation 0"));
    const CliRun json = cli_run({"--sim", "--json", "monitor"});
    CHECK_EQ(json.code, 0);
    const Json m = Json::parse(json.out);
    CHECK_EQ(m.find("position")->as_number(), 73.95);
    CHECK_EQ(m.find("units")->as_string(), std::string("mm"));
}

TEST(cli_parity_monitor_watch_prints_status_and_a_summary) {
    const CliRun r = cli_run({"--sim", "monitor", "--watch", "--duration", "0.4"});
    CHECK_EQ(r.code, 0);
    CHECK(r.out_says("position     73.95 mm"));
    CHECK(r.out_says("polls"));
    CHECK(r.out_says("link ok"));
    const CliRun json = cli_run({"--sim", "--json", "monitor", "--watch", "--duration", "0.3"});
    CHECK_EQ(json.code, 0);
    std::istringstream lines(json.out);
    std::string line, last;
    bool status_seen = false;
    while (std::getline(lines, line)) {
        if (line.empty()) continue;
        const Json j = Json::parse(line);
        status_seen = status_seen || j.find("type")->as_string() == "status";
        last = line;
    }
    CHECK(status_seen);
    CHECK_EQ(Json::parse(last).find("type")->as_string(), std::string("summary"));
}

TEST(cli_parity_info_text_json_and_sections) {
    const CliRun all = cli_run({"--sim", "info"});
    CHECK_EQ(all.code, 0);
    for (const char* part : {"[identity]", "[user parameters]", "[monitor]", "[alarms]", "[maintenance]", "[positions]",
                             "serial        SIM-0000001"})
        CHECK(all.out_says(part));
    const CliRun some = cli_run({"--sim", "--json", "info", "--section", "identity", "--section", "monitor"});
    CHECK_EQ(some.code, 0);
    const Json data = Json::parse(some.out);
    CHECK(!data.find("identity")->is_null());
    CHECK(!data.find("monitor")->is_null());
    CHECK(data.find("alarms")->is_null());
    const CliRun bad = cli_run({"--sim", "info", "--section", "bogus"});
    CHECK_EQ(bad.code, 2);
    CHECK(bad.err_says("--section"));
    const CliRun rotary = cli_run({"--sim", "--sim-profile", "rotary", "info", "--section", "identity"});
    CHECK(rotary.out_says("deg/s (MAXV 7.4)"));
}

TEST(cli_parity_backup_writes_a_read_only_json_file) {
    const std::string path = temp_file("backup.json");
    std::remove(path.c_str());
    const CliRun r = cli_run({"--sim", "backup", path, "--rows", "0,1"});
    CHECK_EQ(r.code, 0);
    CHECK(r.out_says("backup written to"));
    CHECK(r.out_says("2 position rows"));
    const Json data = Json::parse(read_file(path));
    CHECK(data.find("meta")->find("read_only")->as_bool());
    CHECK_EQ(data.find("positions")->find("rows")->size(), std::size_t(2));
    std::remove(path.c_str());
    const CliRun bad = cli_run({"--sim", "backup", temp_file("x.json"), "--rows", "abc"});
    CHECK_EQ(bad.code, 2);
    CHECK(bad.err_says("--rows"));
    const CliRun wrapped = cli_run({"--sim", "backup", temp_file("w.json"), "--rows", "4294967296"});
    CHECK_EQ(wrapped.code, 2);
    CHECK(wrapped.err_says("--rows"));
}

TEST(cli_parity_recover_reports_and_never_moves) {
    const CliRun plain = cli_run({"--sim", "recover", "--target", "73.95"});
    CHECK_EQ(plain.code, 0);
    CHECK(plain.out_says("ready to move: yes; needs attention: no (nothing was moved)"));
    CHECK(plain.out_says("(target 73.95, +0.00)"));
    const std::string path = temp_file("recover_backup.json");
    std::remove(path.c_str());
    CHECK_EQ(cli_run({"--sim", "backup", path, "--rows", "0,1"}).code, 0);
    const CliRun with_backup = cli_run({"--sim", "--json", "recover", "--backup", path});
    CHECK_EQ(with_backup.code, 0);
    const Json report = Json::parse(with_backup.out);
    CHECK(report.find("same_unit")->as_bool());
    std::remove(path.c_str());
    const CliRun missing = cli_run({"--sim", "recover", "--backup", temp_file("no_such_backup.json")});
    CHECK_EQ(missing.code, 2);
    CHECK(missing.err_says("error: backup"));
    CHECK_EQ(cli_run({"--sim", "recover", "--restore-baud", "9600"}).code, 2);  // not a verified code
}

TEST(cli_parity_line_on_a_direct_unit) {
    const CliRun r = cli_run({"--sim", "line"});
    CHECK_EQ(r.code, 0);
    CHECK(r.out_says("line       direct"));
    CHECK(r.out_says("coils yes"));
    CHECK(r.out_says("stop path coil"));
    const CliRun monitor = cli_run({"--sim", "line", "--monitor"});
    CHECK_EQ(monitor.code, 1);
    CHECK(monitor.out_says("RCON gateway only"));
}

TEST(cli_parity_line_on_an_rcon_gateway) {
    const CliRun r = cli_run({"--sim", "--sim-line", "rcon", "--position-unit", "0.01", "line", "--monitor"});
    CHECK_EQ(r.code, 0);
    CHECK(r.out_says("line       rcon"));
    CHECK(r.out_says("coils NO"));
    CHECK(r.out_says("strategy   register; stop path register"));
    CHECK(r.out_says("pos. unit  0.01 mm (stated)"));
    CHECK(r.out_says("MANU"));
    CHECK(r.out_says("monitor    axis 0"));
    CHECK(r.err_says("line rcon"));  // noted on stderr when the port opens
    const CliRun json = cli_run({"--sim", "--sim-line", "rcon", "--position-unit", "0.001", "--json", "line", "--monitor"});
    CHECK_EQ(json.code, 0);
    const Json data = Json::parse(json.out);
    CHECK_EQ(data.find("line")->find("kind")->as_string(), std::string("rcon"));
    CHECK(data.find("gateway_status")->find("manu")->as_bool());
    CHECK_EQ(data.find("monitor")->find("slave")->as_int(), 1LL);
    CHECK_EQ(data.find("errors")->size(), std::size_t(0));
}

TEST(cli_parity_rcon_motion_needs_the_position_unit_and_status_names_the_line) {
    const CliRun move = cli_run({"--sim", "--sim-line", "rcon", "--stroke", "0", "100", "move", "10"});
    CHECK_EQ(move.code, 2);
    CHECK(move.err_says("PositionUnitError"));
    const CliRun status = cli_run({"--sim", "--sim-line", "rcon", "status"});
    CHECK_EQ(status.code, 0);
    CHECK(status.out_says("line       rcon"));
    CHECK(!cli_run({"--sim", "status"}).out_says("line       "));
}

TEST(cli_parity_check_read_only_and_with_motion) {
    const std::string path = temp_file("check_report.json");
    std::remove(path.c_str());
    const CliRun r = cli_run({"--sim", "check", "--report", path});
    CHECK_EQ(r.code, 0);
    CHECK(r.out_says("compatibility check on sim, slave 1, strategy coil"));
    CHECK(r.out_says("SKIP  motion"));
    CHECK(r.out_says("RESULT PASS - report written to"));
    CHECK_EQ(Json::parse(read_file(path)).find("result")->as_string(), std::string("PASS"));
    const CliRun motion = cli_run({"--sim", "--sim-time-scale", "10", "check", "--motion", "--report", path});
    CHECK_EQ(motion.code, 0);
    CHECK(motion.out_says("PASS  motion_back"));
    std::remove(path.c_str());
}

TEST(cli_parity_reset_is_a_deprecated_alias_of_alarm_reset) {
    const CliRun r = cli_run({"--sim", "reset"});
    CHECK_EQ(r.code, 0);
    CHECK(r.err_says("'reset' is deprecated"));
    CHECK(r.out_says("position"));
}

TEST(cli_parity_mode_switches_pc_and_pio_control_and_the_safety_speed) {
    // the Python CLI's "mode pc --safety" case: the status with the flags MODBUS and SAFETY-SPEED
    const CliRun safety = cli_run({"--sim", "mode", "pc", "--safety"});
    CHECK_EQ(safety.code, 0);
    CHECK(safety.out_says("MODBUS SAFETY-SPEED"));
    struct Case {
        std::vector<std::string> args;
        bool modbus;
        bool safety;
    };
    // without --safety (or with off) the safety speed is switched off, as the Python CLI's store_true flag does
    const std::vector<Case> cases{{{"mode", "pc", "--safety", "on"}, true, true},
                                  {{"mode", "pc", "--safety=on"}, true, true},
                                  {{"mode", "pc", "--safety", "off"}, true, false},
                                  {{"mode", "pc"}, true, false},
                                  {{"mode", "pio"}, false, false}};
    for (const Case& c : cases) {
        std::vector<std::string> argv{"--sim", "--json"};
        argv.insert(argv.end(), c.args.begin(), c.args.end());
        const CliRun r = cli_run(argv);
        REQUIRE(r.code == 0);
        const Json st = Json::parse(r.out);
        CHECK_EQ(st.find("modbus_enabled")->as_bool(), c.modbus);
        CHECK_EQ(st.find("safety_speed")->as_bool(), c.safety);
    }
    CHECK(cli_run({"--sim", "mode", "auto"}).err_says("mode: pc or pio"));
    CHECK(cli_run({"--sim", "mode", "pc", "--safety=maybe"}).err_says("--safety: on or off, got 'maybe'"));
    CHECK(cli_run({"--sim", "mode", "pc", "--safety", "maybe"}).err_says("'mode' takes 1 argument(s), got 2"));
    CHECK(cli_run({"--sim", "status", "--safety"}).err_says("'status' does not take --safety"));
    // an RCON gateway carries no 0x45: refused by the line rules, nothing sent
    const CliRun rcon = cli_run({"--sim", "--sim-line", "rcon", "mode", "pc"});
    CHECK_EQ(rcon.code, 2);
    CHECK(rcon.err_says("LineRuleError"));
    CHECK(rcon.err_says("nothing sent"));
}

TEST(cli_parity_strategy_option_chooses_the_command_path) {
    // --strategy iai: home with 0x45 cmd 0x0A (op1 = the home pattern ORG), not the HOME coil
    const CliRun iai = cli_run({"--sim", "--sim-time-scale", "50", "--strategy", "iai", "--log-frames", "home"});
    CHECK_EQ(iai.code, 0);
    CHECK(iai.out_says("HOMED"));
    CHECK(iai.err_says("01 45 00 0a"));
    CHECK(!iai.err_says("01 05 04 0b ff 00"));
    const CliRun coil = cli_run({"--sim", "--sim-time-scale", "50", "--log-frames", "home"});
    CHECK_EQ(coil.code, 0);
    CHECK(coil.err_says("01 05 04 0b ff 00"));
    CHECK(!coil.err_says("01 45 00 0a"));
    // the line and the check name the strategy in use; an explicit coil adapts on a line without coils
    CHECK(cli_run({"--sim", "--strategy", "register", "line"}).out_says("stop path register"));
    CHECK(cli_run({"--sim", "--sim-line", "rcon", "--strategy", "coil", "line"}).out_says("stop path register"));
    const std::string path = temp_file("check_strategy.json");
    CHECK(cli_run({"--sim", "--strategy", "iai", "check", "--report", path}).out_says("slave 1, strategy iai"));
    std::remove(path.c_str());
    const CliRun bogus = cli_run({"--sim", "--strategy", "bogus", "status"});
    CHECK_EQ(bogus.code, 2);
    CHECK(bogus.err_says("--strategy: coil, iai, iai2 or register, got 'bogus'"));
}

TEST(cli_parity_stop_all_on_a_simulated_line) {
    const CliRun r = cli_run({"stop-all", "--sim", "--sim-slaves", "1-3"});
    CHECK_EQ(r.code, 0);
    for (const char* name : {"sim:1: stopped", "sim:2: stopped", "sim:3: stopped"}) CHECK(r.out_says(name));
    CHECK(r.out_says("not a safety function"));
    const CliRun json = cli_run({"--json", "stop-all", "--sim", "--sim-slaves", "1-2", "--broadcast"});
    CHECK_EQ(json.code, 0);
    const Json data = Json::parse(json.out);
    CHECK(data.find("sim:1")->find("stopped")->as_bool());
    CHECK(data.find("sim:2")->find("stopped")->as_bool());
    CHECK_EQ(cli_run({"stop-all", "--sim", "--ports", "COM3"}).code, 2);
}

TEST(cli_parity_cell_stop_all_broadcast_sends_stop_and_its_release) {
    const std::vector<std::uint8_t> stop_on{0x00, 0x05, 0x04, 0x2C, 0xFF, 0x00, 0x4D, 0x12};
    const std::vector<std::uint8_t> stop_off{0x00, 0x05, 0x04, 0x2C, 0x00, 0x00, 0x0C, 0xE2};
    CellOptions co;
    co.identify = false;
    co.scan_slaves = {1, 2};
    {
        auto line = SimLine::with_slaves({1, 2}, SimulatedUnitOptions(), "sim");
        Cell cell = Cell::open(std::map<std::string, std::shared_ptr<SimLine>>{{"sim", line}}, {}, co);
        const auto results = cell.stop_all({}, 5.0, false, true);
        CHECK(results.succeeded());
        CHECK(has_frame(line->requests(), stop_on));
        CHECK(has_frame(line->requests(), stop_off));
        cell.close();
    }
    {
        auto line = SimLine::with_slaves({1, 2}, SimulatedUnitOptions(), "sim");
        Cell cell = Cell::open(std::map<std::string, std::shared_ptr<SimLine>>{{"sim", line}}, {}, co);
        CHECK(cell.stop_all().succeeded());
        bool broadcast_seen = false;
        for (const auto& r : line->requests()) broadcast_seen = broadcast_seen || (!r.second.empty() && r.second[0] == 0);
        CHECK(!broadcast_seen);
        cell.close();
    }
}
