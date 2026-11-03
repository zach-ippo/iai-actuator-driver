// Axis::open on gateway lines: line detection, retries, the line's guards before control and motion, the position
// unit (No.207) on the wire, ROBONET gateway / through mode. Port of the Python edition's
// tests/test_open_on_gateway_lines.py (the parts this edition has; see the area notes for what is not ported yet).
// Simulator only. Every gateway rule is DOC (ME0413, ME0208) and UNVERIFIED on hardware.
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::vector<Bytes> sent(const SimLine& line) {
    std::vector<Bytes> out;
    for (const auto& r : line.requests()) out.push_back(r.second);
    return out;
}

std::set<int> function_codes(const SimLine& line) {
    std::set<int> out;
    for (const Bytes& f : sent(line))
        if (f.size() > 1) out.insert(f[1]);
    return out;
}

/// The two read-only line probes (D0 03 37 00 / 3F 03 F7 00).
int probes(const SimLine& line) {
    int n = 0;
    for (const Bytes& f : sent(line))
        if (f.size() >= 4 && ((f[0] == 0xD0 && f[2] == 0x37) || (f[0] == 0x3F && f[2] == 0xF7)) && f[1] == 0x03) ++n;
    return n;
}

SimulatedUnitOptions fast_unit(double scale = 20.0) {
    SimulatedUnitOptions o = sim_gateway_unit_defaults();
    o.time_scale = scale;
    return o;
}

OpenOptions open_options(const std::string& line = "auto", std::optional<double> unit = std::nullopt) {
    OpenOptions o;
    o.slave = 1;
    o.bus.min_gap_s = 0.0;
    o.axis.poll_interval_s = 0.005;
    o.line.kind = line;
    o.line.position_unit_mm = unit;
    return o;
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// Servo on over FC06 DRG1 (the RCON path: no coils behind the gateway; ME0413 p.5-11), and wait for it.
void servo_on_by_register(Axis& ax, const std::shared_ptr<SimulatedUnit>& unit) {
    ax.bus()->write_register(1, reg::kDrg1, 0x1000);
    const double deadline = now_s() + 3.0;
    while (!unit->servo_on() && now_s() < deadline) sleep_at_least(0.01);
}

/// The 9-word direct move blocks written to 0x9900.
std::vector<Bytes> move_blocks(const SimLine& line) {
    std::vector<Bytes> out;
    for (const Bytes& f : sent(line))
        if (f.size() > 19 && f[1] == 0x10 && f[2] == 0x99 && f[3] == 0x00) out.push_back(f);
    return out;
}

std::int64_t be32(const Bytes& f, std::size_t at, bool is_signed) {
    const std::uint32_t raw = (static_cast<std::uint32_t>(f[at]) << 24) | (static_cast<std::uint32_t>(f[at + 1]) << 16) |
                              (static_cast<std::uint32_t>(f[at + 2]) << 8) | f[at + 3];
    return is_signed ? static_cast<std::int64_t>(static_cast<std::int32_t>(raw)) : static_cast<std::int64_t>(raw);
}

}  // namespace

// ------------------------------------------------------------------------------------------ defaults

TEST(axis_gateway_open_three_retries_by_default) {
    Axis ax = Axis::open(SimLine::with_slaves({1}, fast_unit()), open_options());
    CHECK_EQ(ax.bus()->retries(), 3);
}

TEST(axis_gateway_open_retries_can_still_be_chosen) {
    OpenOptions o = open_options();
    o.bus.retries = 1;
    Axis ax = Axis::open(SimLine::with_slaves({1}, fast_unit()), o);
    CHECK_EQ(ax.bus()->retries(), 1);
}

TEST(axis_gateway_open_plain_line_is_probed_once_and_stays_direct) {
    auto line = SimLine::with_slaves({1}, fast_unit());
    {
        Axis ax = Axis::open(line, open_options());
        CHECK(ax.bus()->line()->kind() == LineKind::Direct);
        CHECK(!ax.bus()->line()->command_strategy().has_value());
    }
    CHECK_EQ(probes(*line), 2);
    std::set<int> codes = function_codes(*line);
    codes.erase(0x03);
    codes.erase(0x05);  // close() may release coils on the direct line; detection itself only reads
    CHECK(codes.empty());
    for (const Bytes& f : sent(*line))
        if (f[0] == 0xD0 || f[0] == 0x3F) CHECK_EQ(static_cast<int>(f[1]), 0x03);
}

TEST(axis_gateway_open_stated_direct_line_sends_no_probe) {
    auto line = SimLine::with_slaves({1}, fast_unit());
    { Axis ax = Axis::open(line, open_options("direct")); }
    CHECK_EQ(probes(*line), 0);
}

TEST(axis_gateway_open_unknown_line_kind_is_refused_and_the_port_closed) {
    auto line = SimLine::with_slaves({1}, fast_unit());
    CHECK_THROWS(Axis::open(line, open_options("canbus")), std::invalid_argument);
    CHECK(!line->is_open());
}

// ------------------------------------------------------------------------------------------ RCON

TEST(axis_gateway_open_rcon_line_and_unit) {
    SimGatewayOptions g;
    g.serials = {{1, "RC-TEST-01"}};
    auto line = sim_rcon_line({1}, g, fast_unit());
    Axis ax = Axis::open(line, open_options("auto", 0.01));
    CHECK(ax.bus()->line()->kind() == LineKind::Rcon);
    CHECK(ax.bus()->line()->command_strategy() == std::optional<std::string>("register"));
    CHECK(ax.bus()->line()->position_unit_mm(1) == std::optional<double>(0.01));
    CHECK(read_serial_code(*ax.bus(), 1) == std::optional<std::string>("RC-TEST-01"));
}

TEST(axis_gateway_open_gateway_must_be_in_manu_before_control) {
    SimGatewayOptions g;
    g.manu = false;
    auto line = sim_rcon_line({1}, g, fast_unit());
    Axis ax = Axis::open(line, open_options("auto", 0.01));
    const std::size_t mark = line->unit(1)->requests().size();
    CHECK_THROWS(ax.enable_modbus_control(), GatewayModeError);
    CHECK_THROWS(ax.enable_modbus_control(), EcNotReadyError);
    CHECK_EQ(line->unit(1)->requests().size(), mark);  // nothing reached the axis
}

TEST(axis_gateway_open_no_coil_is_written_behind_rcon) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    Axis ax = Axis::open(line, open_options("auto", 0.01));
    ax.enable_modbus_control();  // PMSS already 1: nothing to write
    CHECK_THROWS(ax.bus()->write_coil(1, reg::kCoilStop, true), LineRuleError);
    CHECK_EQ(function_codes(*line).count(0x05), 0u);
    CHECK_EQ(function_codes(*line).count(0x45), 0u);
}

TEST(axis_gateway_open_motion_is_refused_until_the_unit_is_stated) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    OpenOptions o = open_options();
    o.axis.stroke = Span{0.0, 100.0};
    Axis ax = Axis::open(line, o);
    servo_on_by_register(ax, line->unit(1));
    const std::size_t before = line->requests().size();
    MoveOptions m;
    m.speed = 50.0;
    m.wait = false;
    bool threw = false;
    try {
        ax.move_to(10.0, m);
    } catch (const PositionUnitError& e) {
        threw = true;
        CHECK(contains(e.what(), "No.207"));
    }
    CHECK(threw);
    CHECK_THROWS(ax.check_move(10.0, m), PositionUnitError);
    CHECK_EQ(line->requests().size(), before);  // nothing sent
    ax.bus()->line()->set_position_unit(0.01);
    m.wait = true;
    m.timeout_s = 10.0;
    const auto st = ax.move_to(10.0, m);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 10.0, 0.011);
    CHECK(!move_blocks(*line).empty());  // the move went out once the unit was stated
}

TEST(axis_gateway_open_a_0_001_unit_scales_target_and_band_on_the_wire) {
    SimGatewayOptions g;
    g.position_unit_mm = 0.001;
    auto line = sim_rcon_line({1}, g, fast_unit());
    OpenOptions o = open_options("auto", 0.001);
    o.axis.stroke = Span{0.0, 100.0};
    Axis ax = Axis::open(line, o);
    servo_on_by_register(ax, line->unit(1));
    MoveOptions m;
    m.speed = 50.0;
    m.band = 0.2;
    m.timeout_s = 10.0;
    const auto st = ax.move_to(45.67, m);
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 45.67, 0.006);
    CHECK_NEAR(ax.status().position_mm, 45.67, 0.006);
    const auto blocks = move_blocks(*line);
    REQUIRE(!blocks.empty());
    CHECK_EQ(be32(blocks.back(), 7, true), 45670);   // target in 0.001 mm counts
    CHECK_EQ(be32(blocks.back(), 11, false), 200);   // band in 0.001 mm counts
    CHECK_EQ(be32(blocks.back(), 15, false), 5000);  // the speed is not scaled
}

TEST(axis_gateway_open_a_0_01_unit_changes_nothing) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    OpenOptions o = open_options("auto", 0.01);
    o.axis.stroke = Span{0.0, 100.0};
    Axis ax = Axis::open(line, o);
    servo_on_by_register(ax, line->unit(1));
    MoveOptions m;
    m.speed = 50.0;
    m.timeout_s = 10.0;
    ax.move_to(45.67, m);
    const auto blocks = move_blocks(*line);
    REQUIRE(!blocks.empty());
    CHECK_EQ(be32(blocks.back(), 7, true), 4567);
}

// ------------------------------------------------------------------------------------------ ROBONET

TEST(axis_gateway_open_robonet_gateway_mode_gives_no_axis_and_closes_the_port) {
    auto line = sim_robonet_line({1}, true, fast_unit());
    CHECK_THROWS(Axis::open(line, open_options()), UnsupportedLineError);
    CHECK(!line->is_open());
}

TEST(axis_gateway_open_robonet_through_mode_refuses_iai_commands) {
    auto line = sim_robonet_line({1}, false, fast_unit());
    {
        Axis ax = Axis::open(line, open_options("robonet"));
        CHECK(ax.bus()->line()->kind() == LineKind::Robonet);
        CHECK(ax.bus()->line()->command_strategy() == std::optional<std::string>("coil"));
        const std::size_t mark = line->requests().size();
        CHECK_THROWS(ax.software_reset(), LineRuleError);
        CHECK_EQ(line->requests().size(), mark);  // refused before the servo off
    }
    CHECK_EQ(probes(*line), 0);  // a stated line is not probed
}
