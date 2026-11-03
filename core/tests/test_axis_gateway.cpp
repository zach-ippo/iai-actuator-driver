// Gateway lines (RCON, ROBONET): detection, line rules and bus guards, MANU, position unit, gateway monitor, identity.
// Port of the Python edition's tests/test_gateway.py. Simulator only (SimGatewayLine, sim_gateway.hpp): every gateway
// rule is DOC (ME0413, ME0208), UNVERIFIED on hardware. Built with the whole library (bus + sim + cell).
#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes hexbytes(const std::string& text) { return util::hex_bytes(text); }

std::vector<Bytes> frames(const SimLine& line) {
    std::vector<Bytes> out;
    for (const auto& r : line.requests()) out.push_back(r.second);
    return out;
}

std::vector<Bytes> frames_since(const SimLine& line, std::size_t mark) {
    std::vector<Bytes> all = frames(line);
    return std::vector<Bytes>(all.begin() + static_cast<std::ptrdiff_t>(std::min(mark, all.size())), all.end());
}

int probe_count(const SimLine& line) {
    int n = 0;
    for (const Bytes& f : frames(line))
        if (!f.empty() && (f[0] == 0xD0 || f[0] == 0x3F)) ++n;
    return n;
}

SimulatedUnitOptions fast_unit(double scale = 20.0) {
    SimulatedUnitOptions o = sim_gateway_unit_defaults();
    o.time_scale = scale;
    return o;
}

BusOptions quick_bus() {
    BusOptions b;
    b.min_gap_s = 0.0;
    return b;
}

LineOptions line_options(const std::string& kind, std::optional<double> unit = std::nullopt) {
    LineOptions o;
    o.kind = kind;
    o.position_unit_mm = unit;
    return o;
}

LineOptions unit_only(double unit) {
    LineOptions o;
    o.position_unit_mm = unit;
    return o;
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::shared_ptr<ModbusRtu> rcon_bus(const std::shared_ptr<SimGatewayLine>& line) {
    auto bus = std::make_shared<ModbusRtu>(line, quick_bus());
    detect_line(*bus);
    return bus;
}

std::shared_ptr<RconGateway> rcon_of(const std::shared_ptr<Line>& line) {
    return std::dynamic_pointer_cast<RconGateway>(line->gateway());
}

}  // namespace

// ------------------------------------------------------------------------------------------ frames

TEST(axis_gateway_probe_and_monitor_frames) {
    const std::vector<std::pair<std::pair<int, std::string>, std::string>> cases = {
        {{0xD0, "03 37 00 00 0B"}, "D0 03 37 00 00 0B 18 38"}, {{0x3F, "03 F7 00 00 02"}, "3F 03 F7 00 00 02 F2 A1"},
        {{0xD0, "03 37 08 00 02"}, "D0 03 37 08 00 02 59 FC"}, {{0xD0, "03 29 00 00 78"}, "D0 03 29 00 00 78 5F F5"},
        {{0xD0, "03 29 78 00 78"}, "D0 03 29 78 00 78 DF EC"}, {{0x01, "03 94 00 00 08"}, "01 03 94 00 00 08 68 3C"},
    };
    for (const auto& c : cases) CHECK(build_frame(c.first.first, hexbytes(c.first.second)) == hexbytes(c.second));
}

TEST(axis_gateway_16_axes_monitor_is_two_frames) {
    std::vector<int> slaves;
    for (int s = 1; s <= 16; ++s) slaves.push_back(s);
    auto line = sim_rcon_line(slaves, {}, fast_unit());
    auto bus = rcon_bus(line);
    const std::size_t mark = line->requests().size();
    const auto mons = rcon_of(bus->line())->read_monitor();
    const auto sent = frames_since(*line, mark);
    REQUIRE(sent.size() == 2);
    CHECK(sent[0] == hexbytes("D0 03 29 00 00 78 5F F5"));
    CHECK(sent[1] == hexbytes("D0 03 29 78 00 78 DF EC"));
    CHECK_EQ(mons.size(), 16u);
    CHECK_EQ(mons.begin()->first, 0);
    CHECK_EQ(mons.rbegin()->first, 15);
}

// ------------------------------------------------------------------------------------------ detection

TEST(axis_gateway_direct_line_costs_two_short_read_only_probes) {
    auto line = SimLine::with_slaves({1}, fast_unit(), "D");
    Port port(line, "D");
    const double started = now_s();
    const auto found = port.detect_line();
    const double elapsed = now_s() - started;
    CHECK(found->kind() == LineKind::Direct);
    CHECK(!found->detected());
    CHECK(elapsed < 0.3);
    const auto sent = frames(*line);
    REQUIRE(sent.size() == 2);
    CHECK(sent[0] == hexbytes("D0 03 37 00 00 0B 18 38"));
    CHECK(sent[1] == hexbytes("3F 03 F7 00 00 02 F2 A1"));
    port.scan_units({1}, std::nullopt, false);
    CHECK_EQ(probe_count(*line), 2);  // probed once per port
}

TEST(axis_gateway_stated_direct_line_sends_no_probe) {
    auto line = SimLine::with_slaves({1}, fast_unit(), "D");
    Port port(line, "D", BusOptions{}, line_options("direct"));
    port.scan_units({1}, std::nullopt, false);
    CHECK(port.line()->kind() == LineKind::Direct);
    CHECK_EQ(probe_count(*line), 0);
}

TEST(axis_gateway_unit_at_0x3f_answering_an_exception_is_not_a_gateway) {
    SimulatedUnitOptions a = fast_unit();
    a.slave = 1;
    SimulatedUnitOptions b = fast_unit();
    b.slave = 0x3F;  // 0xF700 is no unit register: exception 02
    auto line = std::make_shared<SimLine>(
        std::vector<std::shared_ptr<SimulatedUnit>>{std::make_shared<SimulatedUnit>(a), std::make_shared<SimulatedUnit>(b)},
        "D");
    Port port(line, "D");
    CHECK(port.detect_line()->kind() == LineKind::Direct);
}

TEST(axis_gateway_rcon_detected) {
    auto line = sim_rcon_line({1, 2, 3}, {}, fast_unit());
    Port port(line, "R");
    const auto hits = port.scan_units(default_scan_slaves(), std::nullopt, false);
    REQUIRE(hits.size() == 3);
    CHECK_EQ(hits[0].slave, 1);
    CHECK_EQ(hits[2].slave, 3);
    CHECK(port.line()->kind() == LineKind::Rcon);
    CHECK(port.line()->detected());
    auto gw = rcon_of(port.line());
    REQUIRE(gw != nullptr);
    const auto st = gw->last_status();
    REQUIRE(st.has_value());
    CHECK(st->manu());
    CHECK(st->linked_slaves() == std::vector<int>({1, 2, 3}));
    CHECK(port.line()->command_strategy() == std::optional<std::string>("register"));
    CHECK(port.bus()->line() == port.line());
}

TEST(axis_gateway_robonet_gateway_mode_gives_no_axis) {
    auto line = sim_robonet_line({1, 2}, true, fast_unit(), "G");
    {
        Port port(line, "G");
        CHECK(port.scan(default_scan_slaves()).empty());
        CHECK(port.line()->kind() == LineKind::RobonetGateway);
        const auto st = port.gateway_status();
        REQUIRE(st.has_value());
        const auto* r = std::get_if<RoboNetGatewayStatus>(&*st);
        REQUIRE(r != nullptr);
        CHECK(r->running());
        CHECK(r->linked_axes() == std::vector<int>({0, 1}));
        bool threw = false;
        try {
            port.line()->require_control(1);
        } catch (const UnsupportedLineError& e) {
            threw = true;
            CHECK(contains(e.what(), "SW2 ON"));
        }
        CHECK(threw);
    }
    for (const Bytes& f : frames(*line)) CHECK_EQ(static_cast<int>(f[1]), 0x03);  // nothing but reads
    CellOptions co;
    co.scan_timeout_s = 0.01;
    co.identify = false;
    bool refused = false;
    try {
        Cell::open(std::map<std::string, std::shared_ptr<SimGatewayLine>>{
                       {"G2", sim_robonet_line({1}, true, fast_unit(), "G2")}},
                   {}, co);
    } catch (const EcNotReadyError& e) {
        refused = true;
        CHECK(contains(e.what(), "SW2"));
    }
    CHECK(refused);
}

TEST(axis_gateway_robonet_gateway_status_decoding) {
    RoboNetGatewayStatus st;
    st.words = {0x8021, 0x0003};  // the ME0208 p.128-129 example reply
    CHECK(st.running());
    CHECK_EQ(st.positioner_axes(), 1);
    CHECK_EQ(st.direct_axes(), 1);
    CHECK(st.linked_axes() == std::vector<int>({0, 1}));
}

TEST(axis_gateway_stated_rcon_without_gateway_refuses_control) {
    auto line = SimLine::with_slaves({1}, fast_unit(), "X");
    Port port(line, "X", BusOptions{}, line_options("rcon", 0.01));
    CHECK(port.detect_line()->kind() == LineKind::Rcon);
    CHECK(port.gateway() == nullptr);
    CHECK_THROWS(port.line()->require_motion(1), GatewayModeError);
}

// ------------------------------------------------------------------------------------------ line rules

TEST(axis_gateway_operations_per_kind) {
    for (LineOperation op : line_operations()) CHECK(Line(LineKind::Direct).allows(op));
    const std::vector<std::tuple<LineKind, LineOperation, bool>> expect = {
        {LineKind::Rcon, LineOperation::Coil, false},
        {LineKind::Rcon, LineOperation::IaiCommand, false},
        {LineKind::Rcon, LineOperation::BaudChange, false},
        {LineKind::Rcon, LineOperation::ReportSlaveId, false},
        {LineKind::Rcon, LineOperation::AxisControl, true},
        {LineKind::Robonet, LineOperation::Coil, true},
        {LineKind::Robonet, LineOperation::IaiCommand, false},
        {LineKind::Robonet, LineOperation::BaudChange, false},
        {LineKind::Robonet, LineOperation::ReportSlaveId, true},
        {LineKind::Robonet, LineOperation::SoftwareReset, false},
        {LineKind::RobonetGateway, LineOperation::AxisControl, false},
        {LineKind::RobonetGateway, LineOperation::Coil, false},
    };
    for (const auto& e : expect) {
        Line line(std::get<0>(e));
        CHECK_EQ(line.allows(std::get<1>(e)), std::get<2>(e));
        if (!std::get<2>(e)) CHECK_THROWS(line.require(std::get<1>(e)), LineRuleError);
    }
    CHECK_THROWS(Line(LineKind::Direct).allows("teleport"), std::invalid_argument);
    CHECK_THROWS(parse_line_kind("canbus"), std::invalid_argument);
    CHECK(!parse_line_kind("auto").has_value());
    CHECK(parse_line_kind("robonet-gateway") == std::optional<LineKind>(LineKind::RobonetGateway));
}

TEST(axis_gateway_line_specific_messages) {
    const std::vector<std::tuple<LineKind, LineOperation, std::string>> cases = {
        {LineKind::Rcon, LineOperation::SoftwareReset, "ME0413 p.3-1"},
        {LineKind::Rcon, LineOperation::BaudChange, "ME0413 p.6-1 #2"},
        {LineKind::Robonet, LineOperation::BaudChange, "230.4 kbps"},
        {LineKind::Robonet, LineOperation::IaiCommand, "ME0208 p.26"},
        {LineKind::RobonetGateway, LineOperation::Coil, "SW2 ON"},
    };
    for (const auto& c : cases) {
        bool threw = false;
        try {
            Line(std::get<0>(c)).require(std::get<1>(c));
        } catch (const LineRuleError& e) {
            threw = true;
            CHECK(contains(e.what(), std::get<2>(c)));
            CHECK(contains(e.what(), "nothing sent"));
        }
        CHECK(threw);
    }
}

TEST(axis_gateway_to_json_is_json) {
    for (LineKind kind : line_kinds()) {
        Line line(kind);
        const Json j = to_json(line.snapshot());
        CHECK(Json::parse(j.dump()) == j);
        CHECK_EQ(j.find("kind")->as_string(), std::string(to_string(kind)));
    }
    CHECK_EQ(line_rules_table().size(), 4u);
}

TEST(axis_gateway_one_line_rule_error_class) {
    CHECK((std::is_base_of<UnsafeCommandError, LineRuleError>::value));
    CHECK((std::is_base_of<EcNotReadyError, GatewayModeError>::value));
    CHECK((std::is_base_of<EcNotReadyError, PositionUnitError>::value));
    CHECK((std::is_base_of<EcNotReadyError, UnsupportedLineError>::value));
}

// ------------------------------------------------------------------------------------------ RCON guard

TEST(axis_gateway_rcon_refused_requests_send_nothing) {
    auto line = sim_rcon_line({1, 2}, {}, fast_unit());
    auto bus = rcon_bus(line);
    const std::size_t mark = line->requests().size();
    CHECK_THROWS(bus->write_coil(1, reg::kCoilStop, true), LineRuleError);  // FC05
    CHECK_THROWS(bus->iai_command(1, 0, 0x09, 1), LineRuleError);          // 0x45 servo
    IaiOptions unsafe;
    unsafe.allow_unsafe = true;
    CHECK_THROWS(bus->iai_command(1, 0, 0x05, 1, 0, unsafe), LineRuleError);  // 0x45 reset
    CHECK_THROWS(bus->line()->check_request(1, {0x48, 0x00, 0x00, 0x30}), LineRuleError);  // 0x48
    CHECK_THROWS(bus->write_register(0xD0, 0x3700, 0), LineRuleError);  // a write to the gateway
    CHECK_THROWS(bus->broadcast_coil(reg::kCoilStop, true), LineRuleError);
    CHECK_EQ(line->requests().size(), mark);
}

TEST(axis_gateway_rcon_documented_writes_pass) {
    auto line = sim_rcon_line({1, 2}, {}, fast_unit());
    auto bus = rcon_bus(line);
    bus->write_register(1, reg::kDrg1, 0x1000);  // FC06 DRG1 servo on (ME0413 p.5-11)
    CHECK(frames(*line).back() == hexbytes("01 06 0D 00 10 00 86 A6"));
    CHECK_EQ(bus->read_registers(1, 0x9000, 8).size(), 8u);
}

TEST(axis_gateway_rcon_fc11_read_passes_the_guard_and_the_sim_rejects_it) {
    auto line = sim_rcon_line({1, 2}, {}, fast_unit());
    auto bus = rcon_bus(line);
    bool threw = false;
    try {
        bus->report_slave_id(1, 0);
    } catch (const ModbusSlaveError& e) {
        threw = true;
        CHECK_EQ(e.code(), 0x01);
    }
    CHECK(threw);
    CHECK(bus->unsupported_functions(1).count(0x11) == 1);
}

// ------------------------------------------------------------------------------------------ MANU

TEST(axis_gateway_auto_refuses_then_manu_allows) {
    SimGatewayOptions g;
    g.manu = false;
    auto line = sim_rcon_line({1}, g, fast_unit());
    Port port(line, "R", BusOptions{}, unit_only(0.01));
    port.scan({1});
    bool threw = false;
    try {
        port.line()->require_control(1);
    } catch (const GatewayModeError& e) {
        threw = true;
        CHECK(contains(e.what(), "MANU"));
        CHECK(contains(e.what(), "ME0413 p.6-1 #5"));
    }
    CHECK(threw);
    line->rcon_gateway()->set_manu(true);  // the switch is turned: a refusal re-reads, so the next call passes
    CHECK_EQ(port.line()->require_motion(1), 0.01);
}

TEST(axis_gateway_manu_read_is_cached) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    Port port(line, "R", BusOptions{}, unit_only(0.01));
    port.scan({1});
    port.line()->require_control(1);
    const std::size_t n = line->rcon_gateway()->requests().size();
    for (int i = 0; i < 5; ++i) port.line()->require_control(1);
    CHECK_EQ(line->rcon_gateway()->requests().size(), n);  // not read again inside the cache window
    rcon_of(port.line())->set_cache_s(0.0);
    port.line()->require_control(1);
    CHECK_EQ(line->rcon_gateway()->requests().size(), n + 1);
}

TEST(axis_gateway_gateway_alarm_and_emergency_stop) {
    SimGatewayOptions g;
    g.gateway_alarm = 0x8BA;
    g.emergency_stop = true;
    auto line = sim_rcon_line({1}, g, fast_unit());
    auto bus = rcon_bus(line);
    auto gw = rcon_of(bus->line());
    const RconGatewayStatus st = gw->read_status();
    CHECK_EQ(st.alarm_code(), 0x8BAu);
    CHECK(contains(st.alarm_name().value_or(""), "Operation mode"));
    CHECK(st.emergency_stop());
    CHECK_EQ(gw->read_alarm(), 0x8BAu);
    CHECK_EQ(to_json(st).find("alarm_code")->as_int(), 0x8BA);
}

// ------------------------------------------------------------------------------------------ position unit

TEST(axis_gateway_position_unit_refused_until_stated) {
    auto line = sim_rcon_line({1, 2}, {}, fast_unit());
    Port port(line, "R");
    port.scan({1, 2});
    CHECK(!port.line()->position_unit_mm(1).has_value());
    bool threw = false;
    try {
        port.line()->require_motion(1);
    } catch (const PositionUnitError& e) {
        threw = true;
        CHECK(contains(e.what(), "No.207"));
    }
    CHECK(threw);
    port.set_position_unit(0.001, std::vector<int>{1});
    CHECK_EQ(port.line()->require_motion(1), 0.001);
    CHECK(port.line()->position_unit_source(1) == std::optional<std::string>("stated"));
    CHECK_THROWS(port.line()->require_position_unit(2), PositionUnitError);
    port.set_position_unit(0.01);
    CHECK_EQ(port.line()->require_position_unit(2), 0.01);
    CHECK_EQ(port.line()->require_position_unit(1), 0.001);  // the per-axis statement wins
    CHECK_THROWS(port.set_position_unit(0.1), std::invalid_argument);
    CHECK_EQ(port.line()->wire_factor(1), 10);
    CHECK_EQ(port.line()->wire_factor(2), 1);
}

TEST(axis_gateway_direct_line_default_unit) {
    Line line(LineKind::Direct);
    CHECK_EQ(line.require_motion(1), reg::kPositionScale);
    CHECK(line.position_unit_source(1) == std::optional<std::string>("default"));
}

TEST(axis_gateway_sim_0_001_unit_on_the_wire) {
    SimGatewayOptions g;
    g.position_unit_mm = 0.001;
    auto line = sim_rcon_line({1}, g, fast_unit(50.0));
    auto bus = rcon_bus(line);
    auto unit = line->unit(1);
    bus->write_register(1, reg::kDrg1, 0x1000);  // servo on over FC06 (no coils behind RCON)
    const double deadline_on = now_s() + 3.0;
    while (!unit->servo_on() && now_s() < deadline_on) sleep_at_least(0.01);
    REQUIRE(unit->servo_on());
    const std::vector<std::uint16_t> pw = bus->read_registers(1, reg::kMonitorBase, 2);
    CHECK_EQ(reg::from_i32_words(pw[0], pw[1]), static_cast<std::int32_t>(std::llround(unit->position_mm() * 1000)));
    const auto t = reg::to_i32_words(45678);
    const auto band = reg::to_u32_words(100);
    const auto speed = reg::to_u32_words(2000);
    bus->write_registers(1, reg::kDirectMoveBase,
                         {t.first, t.second, band.first, band.second, speed.first, speed.second, 10, 0, 0});
    const double deadline = now_s() + 5.0;
    sleep_at_least(0.02);
    while (unit->moving() && now_s() < deadline) sleep_at_least(0.01);
    CHECK_NEAR(unit->position_mm(), 45.68, 0.005);  // the sim counts 0.01 mm
    const auto mon = rcon_of(bus->line())->read_monitor(std::vector<int>{0});
    CHECK_EQ(mon.at(0).position_counts(), static_cast<std::int32_t>(std::llround(unit->position_mm() * 1000)));
}

// ------------------------------------------------------------------------------------------ identity

TEST(axis_gateway_scan_uses_the_serial_code_not_fc11) {
    SimGatewayOptions g;
    g.serials = {{2, "RCON-PC-0042"}};
    auto line = sim_rcon_line({1, 2}, g, fast_unit());
    Port port(line, "R");
    const auto hits = port.scan_units(default_scan_slaves(), std::nullopt, true);
    REQUIRE(hits.size() == 2);
    CHECK(hits[0].serial_code == std::optional<std::string>("SIMRCON-00"));
    CHECK(hits[1].serial_code == std::optional<std::string>("RCON-PC-0042"));
    CHECK_EQ(hits[0].line, std::string("rcon"));
    const Json j = to_json(hits[0]);
    CHECK_EQ(j.find("product_family")->as_string(), std::string("RCON"));
    CHECK_EQ(j.find("line")->as_string(), std::string("rcon"));
    for (const Bytes& f : frames(*line)) CHECK(f[1] != 0x11);
}

TEST(axis_gateway_read_serial_code) {
    SimGatewayOptions g;
    g.serials = {{3, "AB"}};
    auto bus = rcon_bus(sim_rcon_line({3}, g, fast_unit()));
    CHECK(read_serial_code(*bus, 3) == std::optional<std::string>("AB"));
    ModbusRtu plain(std::make_shared<SimulatedUnit>(fast_unit()), quick_bus());
    CHECK(!read_serial_code(plain, 1).has_value());
}

// ------------------------------------------------------------------------------------------ ECs behind RCON-EC

TEST(axis_gateway_linked_silent_axes_are_explained) {
    SimGatewayOptions g;
    g.linked_only = {5};
    auto line = sim_rcon_line({1}, g, fast_unit());
    {
        Port port(line, "R");
        const auto hits = port.scan_units(default_scan_slaves(), std::nullopt, false);
        REQUIRE(hits.size() == 1);
        CHECK_EQ(hits[0].slave, 1);
        CHECK(port.line()->unreachable_slaves() == std::vector<int>({5}));
        const std::string why = port.line()->explain().value_or("");
        CHECK(contains(why, "ME0413 p.4-1"));
        CHECK(contains(why, "ME0394 p.4-56"));
    }
    std::set<int> probed;
    for (const Bytes& f : frames(*line))
        if (f[1] == 0x03 && f[2] == 0x90 && f[3] == 0x00) probed.insert(f[0]);
    CHECK(probed == std::set<int>({1, 5}));  // only the linked addresses were probed
}

TEST(axis_gateway_cell_open_with_listed_ec_axis) {
    SimGatewayOptions g;
    g.linked_only = {2};
    auto line = sim_rcon_line({1}, g, fast_unit());
    CellOptions co;
    co.scan_timeout_s = 0.01;
    co.identify = false;
    bool threw = false;
    try {
        Cell::open(std::map<std::string, std::shared_ptr<SimGatewayLine>>{{"RCON", line}}, {1, 2}, co);
    } catch (const ModbusTimeout& e) {
        threw = true;
        CHECK(contains(e.what(), "RCON-EC"));
    }
    CHECK(threw);
}

// ------------------------------------------------------------------------------------------ the simulators

TEST(axis_gateway_sim_rcon_units_reject_undocumented_functions) {
    auto line = sim_rcon_line({1}, {}, fast_unit());
    ModbusRtu bus(line, quick_bus());  // no detection: the bus guard does not know the gateway
    bool threw = false;
    try {
        bus.write_coil(1, reg::kCoilStop, true);
    } catch (const ModbusSlaveError& e) {
        threw = true;
        CHECK_EQ(e.code(), 0x01);
    }
    CHECK(threw);
    CHECK_EQ(line->rejected().size(), 1u);
    CHECK(!line->unit(1)->stop_held());
    bus.broadcast_coil(reg::kCoilStop, true, 0.0);
    CHECK_EQ(line->dropped_broadcasts().size(), 1u);
    CHECK(!line->unit(1)->stop_held());
}

TEST(axis_gateway_sim_robonet_through_mode_model) {
    auto line = sim_robonet_line({1}, false, fast_unit());
    BusOptions b = quick_bus();
    b.retries = 0;
    ModbusRtu bus(line, b);
    CHECK_THROWS(bus.iai_command(1, 0, 0x09, 1), ModbusSlaveError);
    CHECK_THROWS(bus.read_registers(1, 0x0000, 4), ModbusTimeout);  // no directory
    CHECK_EQ(bus.read_registers(1, 0x9000, 2).size(), 2u);
    bus.write_coil(1, reg::kCoilSon, true);  // coils are carried
    CHECK(contains(robonet_hint(bus, 1, true).value_or(""), "line='robonet'"));
    CHECK(!robonet_hint(bus, 1, false).has_value());
}

TEST(axis_gateway_robonet_stated_line_rules) {
    auto line = sim_robonet_line({1, 2}, false, fast_unit(), "N");
    Port port(line, "N", BusOptions{}, line_options("robonet"));
    const auto hits = port.scan_units({1, 2, 3}, std::nullopt, false);
    REQUIRE(hits.size() == 2);
    CHECK_EQ(hits[1].slave, 2);
    CHECK_EQ(probe_count(*line), 0);  // through mode is not probed
    auto bus = port.bus();
    CHECK_THROWS(bus->iai_command(1, 0, 0x09, 1), LineRuleError);
    bool threw = false;
    try {
        bus->write_registers(1, 0x1000, std::vector<std::uint16_t>(76, 0));
    } catch (const LineRuleError& e) {
        threw = true;
        CHECK(contains(e.what(), "160-byte"));
    }
    CHECK(threw);
    CHECK_THROWS(port.set_baud_rate(115200, {1}), LineRuleError);
    bus->write_coil(1, reg::kCoilSon, true);
    CHECK(port.line()->rules().directory_timeout_means_absent);
    CHECK(port.line()->command_strategy() == std::optional<std::string>("coil"));
}

TEST(axis_gateway_robonet_baud_detection_skips_slow_rates) {
    auto fake = std::make_shared<FakeTransport>(nullptr, 115200, "N");  // nothing answers
    Port port(fake, "N", BusOptions{}, line_options("robonet"));
    CHECK(port.scan({1}).empty());
    const std::vector<int> rates = fake->baud_rates_set();
    CHECK(!rates.empty());
    for (int r : rates) CHECK(r >= 38400 && r <= 230400);  // ME0208 p.26: 38.4-230.4 kbps only
    CHECK_EQ(fake->baud_rate(), 115200);                    // the starting rate is restored
}

TEST(axis_gateway_gateway_objects_on_the_cell) {
    CellOptions co;
    co.scan_timeout_s = 0.01;
    co.identify = false;
    const std::map<std::string, std::shared_ptr<Transport>> ports = {
        {"RCON", sim_rcon_line({1}, {}, fast_unit())}, {"D", SimLine::with_slaves({1}, fast_unit(), "D")}};
    Cell cell = Cell::open(ports, {}, co);
    CHECK(cell.lines() == (std::map<std::string, std::string>{{"D", "direct"}, {"RCON", "rcon"}}));
    const auto gws = cell.gateways();
    REQUIRE(gws.size() == 1);
    CHECK_EQ(gws.begin()->first, std::string("RCON"));
    const auto all = cell.gateway_status_all();
    REQUIRE(all.count("RCON") == 1);
    const auto* st = std::get_if<RconGatewayStatus>(&all.at("RCON"));
    REQUIRE(st != nullptr);
    CHECK(st->manu());
}
