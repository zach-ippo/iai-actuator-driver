// Axis unit: an axis behind an RCON gateway (and a ROBONET in through mode): the register strategy and the coil-free
// STOP, the identity from the 0x9400 serial code without FC11, a silent directory treated as absent (nothing else of
// the no-access areas is read), motion that needs an explicit stroke there, only the documented RCON function codes on
// the wire, the position-table refusal, the 0.001 mm unit in the status (3 decimals) and on the command-profile path,
// and the line rules in recover(). Port of the Python edition's tests/test_open_on_gateway_lines.py (OpenRconTest,
// OpenWithoutDirectoryTest) and the axis parts of tests/test_gateway.py. Simulator only: every gateway rule is DOC
// (ME0413, ME0208) and UNVERIFIED on hardware.
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "filter_transport.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;
using ec_test::Bytes;
using ec_test::FilterTransport;

namespace {

SimulatedUnitOptions fast_unit(double scale = 20.0) {
    SimulatedUnitOptions o = sim_gateway_unit_defaults();
    o.time_scale = scale;
    return o;
}

OpenOptions open_options(std::optional<double> unit = 0.01, const std::string& line = "auto") {
    OpenOptions o;
    o.slave = 1;
    o.bus.min_gap_s = 0.0;
    o.axis.poll_interval_s = 0.005;
    o.line.kind = line;
    o.line.position_unit_mm = unit;
    return o;
}

std::set<int> function_codes(const std::vector<Bytes>& frames) {
    std::set<int> out;
    for (const Bytes& f : frames)
        if (f.size() > 1) out.insert(f[1]);
    return out;
}

std::vector<Bytes> sent(const SimLine& line) {
    std::vector<Bytes> out;
    for (const auto& r : line.requests()) out.push_back(r.second);
    return out;
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool any_contains(const std::vector<std::string>& texts, const std::string& part) {
    for (const auto& t : texts)
        if (contains(t, part)) return true;
    return false;
}

/// The directory, identification, limit, parameter and table areas (below DRG1 0x0D00) and the 0x8000 system area:
/// 'system, no access' on an RCON axis (DOC ME0413 p.5-10).
bool no_access(int address) { return address < reg::kDrg1 || (0x8000 <= address && address < 0x9000); }

bool no_access_read(const Bytes& f) {
    return f.size() >= 6 && f[1] == 0x03 && f[0] >= 1 && f[0] <= 16 && no_access((f[2] << 8) | f[3]);
}

/// An RCON line whose axes never answer a read of the no-access areas.
std::shared_ptr<FilterTransport> silent_directory_line(std::shared_ptr<SimGatewayLine> line) {
    return std::make_shared<FilterTransport>(line, [](const Bytes& f, Transport&) -> std::optional<Bytes> {
        if (no_access_read(f)) return Bytes();  // silence
        return std::nullopt;
    });
}

std::vector<int> no_access_addresses(const std::vector<Bytes>& frames) {
    std::vector<int> out;
    for (const Bytes& f : frames)
        if (no_access_read(f)) out.push_back((f[2] << 8) | f[3]);
    return out;
}

bool wait_for(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_at_least(0.005);
    }
    return predicate();
}

}  // namespace

// ------------------------------------------------------------------------------------------ RCON: strategy, identity

TEST(axis_rcon_axis_takes_the_register_strategy) {
    SimGatewayOptions g;
    g.serials = {{1, "RC-TEST-01"}};
    Axis ax = Axis::open(sim_rcon_line({1}, g, fast_unit()), open_options());
    CHECK(ax.bus()->line()->kind() == LineKind::Rcon);
    CHECK(ax.line_name() == std::optional<std::string>("rcon"));
    CHECK(ax.strategy() == CommandMethod::Register);
    CHECK_EQ(ax.stop_path(), "register");
    CHECK(ax.bus()->line()->position_unit_mm(1) == std::optional<double>(0.01));
}

TEST(axis_rcon_an_explicit_strategy_is_kept) {
    Axis ax = Axis::open(sim_rcon_line({1}, {}, fast_unit()), open_options());
    ax.set_strategy(CommandMethod::Coil);
    CHECK(ax.strategy() == CommandMethod::Coil);
    CHECK_EQ(ax.stop_path(), "register");  // the stop path follows the line even when the strategy is coil
}

TEST(axis_rcon_identity_is_the_serial_code_and_fc11_is_not_sent) {
    SimGatewayOptions g;
    g.serials = {{1, "RC-TEST-01"}};
    auto line = sim_rcon_line({1}, g, fast_unit());
    Axis ax = Axis::open(line, open_options());
    const DeviceInfo info = ax.identify();
    // a directory that does answer gives the system / unit code (CTCL) after the serial code
    CHECK(info.identity_source == std::optional<std::string>("CTCL 0x0300"));
    CHECK(info.system_code.has_value());
    CHECK(any_contains(info.errors, "FC11"));
    CHECK(!any_contains(info.errors, "report slave id"));
    CHECK_EQ(function_codes(sent(*line)).count(0x11), 0u);
    bool serial_read = false;
    for (const Bytes& f : sent(*line))
        if (f.size() >= 4 && f[1] == 0x03 && f[2] == 0x94 && f[3] == 0x00) serial_read = true;
    CHECK(serial_read);
}

TEST(axis_rcon_unit_code_read_does_not_send_fc11) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    Axis ax = Axis::open(line, open_options());
    (void)ax.product_family();
    (void)ax.is_ec();
    CHECK_EQ(function_codes(sent(*line)).count(0x11), 0u);
}

TEST(axis_rcon_no_coil_is_written_when_opening_and_stopping) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    {
        Axis ax = Axis::open(line, open_options());
        ax.enable_modbus_control();
        ax.stop();
        ax.pause();
        ax.resume();
    }
    CHECK_EQ(function_codes(sent(*line)).count(0x05), 0u);
    CHECK_EQ(function_codes(sent(*line)).count(0x45), 0u);
}

TEST(axis_rcon_a_move_can_be_stopped_from_the_pc) {
    // the safety item: servo, home and a move go out over DRG1 / 0x9900 and the coil-free STOP ends the move there
    auto line = sim_rcon_line({1}, {}, fast_unit(5.0));
    OpenOptions o = open_options();
    o.axis.stroke = Span{0.0, 150.0};
    Axis ax = Axis::open(line, o);
    ax.servo(false);
    CHECK(!line->unit(1)->servo_on());
    ax.servo(true);
    CHECK(line->unit(1)->servo_on());
    const auto homed = ax.home();
    REQUIRE(homed.has_value());
    CHECK(homed->homed);
    MoveOptions m;
    m.speed = 20.0;
    m.wait = false;
    ax.move_to(140.0, m);
    CHECK(wait_for([&] { return line->unit(1)->position_mm() > 20.0; }, 10.0));
    const Status st = ax.stop(true);
    CHECK(!st.moving);
    const double stopped = line->unit(1)->position_mm();
    sleep_at_least(0.3);
    CHECK_NEAR(line->unit(1)->position_mm(), stopped, 0.005);  // the old target is not resumed
    CHECK(stopped < 140.0);
    CHECK(!ax.status().paused());
    std::set<int> codes = function_codes(sent(*line));
    for (int fc : {0x03, 0x06, 0x10, 0x17}) codes.erase(fc);
    CHECK(codes.empty());  // FC01/03/04/06/10/17 only on RCON (DOC ME0413 p.3-1)
}

TEST(axis_rcon_error_path_stops_with_the_substitute) {
    auto line = sim_rcon_line({1}, {}, fast_unit(5.0));
    OpenOptions o = open_options();
    o.axis.stroke = Span{0.0, 150.0};
    Axis ax = Axis::open(line, o);
    MoveOptions m;
    m.speed = 20.0;
    m.timeout_s = 0.1;
    CHECK_THROWS(ax.move_to(140.0, m), MotionTimeoutError);
    CHECK(!line->unit(1)->moving());
    CHECK(!ax.status().paused());
    CHECK_EQ(function_codes(sent(*line)).count(0x05), 0u);
}

TEST(axis_rcon_profile_move_is_refused_before_anything_is_sent) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    OpenOptions o = open_options();
    o.axis.stroke = Span{0.0, 150.0};
    Axis ax = Axis::open(line, o);
    line->clear_requests();
    MoveOptions m;
    m.speed = 20.0;
    m.decel_g = 0.2;
    bool threw = false;
    try {
        ax.move_to(50.0, m);
    } catch (const LineRuleError& e) {
        threw = true;
        CHECK(contains(e.what(), "PCRF"));
    }
    CHECK(threw);
    CHECK(sent(*line).empty());
}

TEST(axis_rcon_gateway_must_be_in_manu_before_control) {
    SimGatewayOptions g;
    g.manu = false;
    auto line = sim_rcon_line({1}, g, fast_unit());
    Axis ax = Axis::open(line, open_options());
    line->unit(1)->clear_requests();
    CHECK_THROWS(ax.servo(true), GatewayModeError);
    CHECK_THROWS(ax.home(), GatewayModeError);
    CHECK_THROWS(ax.set_safety_speed(true), GatewayModeError);
    for (const Bytes& f : line->unit(1)->requests()) CHECK_EQ(static_cast<int>(f[1]), 0x03);  // nothing written
}

// ------------------------------------------------------------------------------------------ the 0.001 mm unit

TEST(axis_rcon_a_0_001_unit_gives_3_decimals_in_the_status) {
    SimGatewayOptions g;
    g.position_unit_mm = 0.001;
    auto line = sim_rcon_line({1}, g, fast_unit());
    OpenOptions o = open_options(0.001);
    o.axis.stroke = Span{0.0, 150.0};
    Axis ax = Axis::open(line, o);
    MoveOptions m;
    m.speed = 50.0;
    m.timeout_s = 10.0;
    const auto st = ax.move_to(45.67, m);
    REQUIRE(st.has_value());
    const Status now = ax.status();
    CHECK_NEAR(now.position_mm, 45.67, 0.0015);
    CHECK(now.line == std::optional<std::string>("rcon"));
    CHECK_NEAR(now.position_mm * 1000.0, std::round(now.position_mm * 1000.0), 1e-6);  // 3 decimals
}

TEST(axis_rcon_profile_move_target_and_band_in_the_unit_counts) {
    // a ROBONET axis stated with a 0.001 mm unit: the command profile (FC17) carries target and band x 10
    auto line = sim_robonet_line({1}, false, fast_unit());
    OpenOptions o = open_options(std::nullopt, "robonet");
    o.axis.stroke = Span{0.0, 150.0};
    Axis ax = Axis::open(line, o);
    ax.bus()->line()->set_position_unit(0.001, 1);
    line->clear_requests();
    MoveOptions m;
    m.speed = 20.0;
    m.accel_g = 0.2;
    m.decel_g = 0.1;
    m.band = 0.2;
    m.wait = false;
    ax.move_to(5.0, m);
    ax.stop();
    Bytes fc17;
    for (const Bytes& f : sent(*line))
        if (f.size() > 30 && f[1] == 0x17) fc17 = f;
    REQUIRE(!fc17.empty());
    // 17 [read addr 2] [read count 2] [write addr 2] [write count 2] [byte count 1] then the 20 words
    const std::size_t at = 11;
    auto be32 = [&](std::size_t i) {
        return static_cast<std::int32_t>((static_cast<std::uint32_t>(fc17[i]) << 24) |
                                         (static_cast<std::uint32_t>(fc17[i + 1]) << 16) |
                                         (static_cast<std::uint32_t>(fc17[i + 2]) << 8) | fc17[i + 3]);
    };
    CHECK_EQ(be32(at), 5000);       // target 5.000 mm in 0.001 mm counts
    CHECK_EQ(be32(at + 4), 2000);   // the speed is not scaled (0.01 mm/s)
    CHECK_EQ(be32(at + 16), 200);   // band 0.200 mm in 0.001 mm counts
}

// ------------------------------------------------------------------------------------------ no directory on the line

TEST(axis_rcon_identify_treats_the_silent_directory_as_absent) {
    SimGatewayOptions g;
    g.serials = {{1, "RC-TEST-02"}};
    auto t = silent_directory_line(sim_rcon_line({1}, g, fast_unit()));
    Axis ax = Axis::open(t, open_options());
    const DeviceInfo info = ax.identify();
    CHECK(any_contains(info.errors, "treated as absent"));
    CHECK(!info.soft_limits_mm.has_value());
    CHECK(!info.max_speed_mm_s.has_value());
    CHECK(info.serial_number == std::optional<std::string>("RC-TEST-02"));  // the serial code is the identity
    CHECK(info.identity_source == std::optional<std::string>("serial code 0x9400"));
    const std::vector<int> below = no_access_addresses(t->frames());
    CHECK_EQ(below.size(), 4u);  // one read, 3 retries, then nothing more
    for (int a : below) CHECK_EQ(a, 0x0000);
    CHECK_EQ(function_codes(t->frames()).count(0x11), 0u);
}

TEST(axis_rcon_limits_are_not_read_without_a_directory) {
    auto t = silent_directory_line(sim_rcon_line({1}, {}, fast_unit()));
    Axis ax = Axis::open(t, open_options());
    const SoftLimits soft = ax.read_soft_limits();
    CHECK(!soft.limits.has_value());
    CHECK(contains(soft.note, "stroke"));
    (void)ax.read_axis_limits();
    CHECK_EQ(ax.max_positions(), reg::kPositionActiveRows);
    (void)ax.limits();
    for (int a : no_access_addresses(t->frames())) CHECK_EQ(a, 0x0000);
}

TEST(axis_rcon_motion_needs_an_explicit_stroke) {
    auto t = silent_directory_line(sim_rcon_line({1}, {}, fast_unit()));
    Axis ax = Axis::open(t, open_options());
    ax.enable_modbus_control();
    MoveOptions m;
    m.speed = 50.0;
    m.wait = false;
    bool threw = false;
    try {
        ax.move_to(10.0, m);
    } catch (const EcNotReadyError& e) {
        threw = true;
        CHECK(contains(e.what(), "stroke"));
    }
    CHECK(threw);
}

TEST(axis_rcon_a_move_with_an_explicit_stroke_uses_only_documented_functions) {
    auto t = silent_directory_line(sim_rcon_line({1}, {}, fast_unit()));
    OpenOptions o = open_options();
    o.axis.stroke = Span{0.0, 100.0};
    {
        Axis ax = Axis::open(t, o);
        ax.identify();
        ax.enable_modbus_control();
        ax.servo(true);
        MoveOptions m;
        m.speed = 50.0;
        m.timeout_s = 10.0;
        const auto st = ax.move_to(10.0, m);
        REQUIRE(st.has_value());
        CHECK_NEAR(st->position_mm, 10.0, 0.011);
        CHECK(st->line == std::optional<std::string>("rcon"));
        ax.stop();
    }
    for (int a : no_access_addresses(t->frames())) CHECK_EQ(a, 0x0000);
    std::set<int> codes = function_codes(t->frames());
    for (int fc : {0x03, 0x06, 0x10, 0x17}) codes.erase(fc);
    CHECK(codes.empty());
}

TEST(axis_rcon_rows_are_not_writable_without_the_table) {
    auto t = silent_directory_line(sim_rcon_line({1}, {}, fast_unit()));
    Axis ax = Axis::open(t, open_options());
    t->clear_frames();
    PositionEntry e;
    e.position_mm = 10.0;
    e.band_mm = 0.1;
    e.speed_mm_s = 20.0;
    e.accel_g = 0.3;
    e.decel_g = 0.3;
    NvWriteOptions nv;
    nv.allow_nonvolatile = true;
    bool threw = false;
    try {
        ax.write_position_table(0, e, nv);
    } catch (const LineRuleError& err) {
        threw = true;
        CHECK(contains(err.what(), "system, no access"));
    }
    CHECK(threw);
    for (const Bytes& f : t->frames()) CHECK_EQ(static_cast<int>(f[1]), 0x03);  // reads only
}

TEST(axis_rcon_a_directory_timeout_still_raises_on_a_direct_line) {
    auto line = SimLine::with_slaves({1}, fast_unit());
    OpenOptions o = open_options(std::nullopt, "direct");
    o.bus.retries = 0;
    Axis ax = Axis::open(line, o);
    line->remove(1);
    CHECK_THROWS(ax.read_soft_limits(), ModbusTimeout);
}

// ------------------------------------------------------------------------------------------ recover() on RCON

TEST(axis_rcon_recover_restores_the_safety_speed_over_drg1) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    Axis ax = Axis::open(line, open_options());
    ax.connect();
    ax.set_safety_speed(true);
    recovery::track(ax);
    ax.bus()->write_register(1, reg::kDrg1, 0x1000);  // SFTY off behind the driver's back (not a noted change)
    CHECK(!line->unit(1)->safety_speed());
    line->clear_requests();
    recovery::RecoverOptions ro;
    ro.timeout_s = 5.0;
    const recovery::RecoveryReport report = recovery::recover(ax, ro);
    CHECK(line->unit(1)->safety_speed());
    CHECK(report.safety_speed_restored);
    CHECK_EQ(function_codes(sent(*line)).count(0x05), 0u);
    CHECK_EQ(function_codes(sent(*line)).count(0x11), 0u);  // the quick identity check sends no FC11 there
}
