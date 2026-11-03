// Tests of SimLine (sim unit): several SimulatedUnits on one simulated serial line by slave address: routing, the
// broadcast address, refused duplicate addresses, the request log, sim_ports, a baud rate / BREAK that reach every unit,
// closing. Port of the C# edition's SimLineTests (the parts that do not need the bus). The line is UNVERIFIED on
// hardware (an SIO daisy chain of several axes). 0.6: a gateway line reaches its units through their ports (their logs
// and hooks apply), silent_no_access, a hook that throws, the baud history. Unofficial: not an IAI product.
#include <set>
#include <stdexcept>

#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_hooks.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "harness.hpp"
#include "sim_fixtures.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::sim_test;

namespace {

/// Write one request to a line and read whatever came back.
Bytes line_exchange(SimLine& line, const std::string& pdu_hex, int slave) {
    const Bytes request = frame(slave, pdu_hex);
    line.write(request.data(), request.size());
    Bytes data(1024);
    const std::size_t n = line.read(data.data(), data.size());
    data.resize(n);
    return data;
}

std::shared_ptr<SimulatedUnit> make_unit(int slave, SimProfile profile = SimProfile::Slider) {
    SimulatedUnitOptions o;
    o.slave = slave;
    o.profile = profile;
    return std::make_shared<SimulatedUnit>(o);
}

}  // namespace

TEST(sim_line_only_the_addressed_unit_answers_and_an_unknown_address_is_silent) {
    SimLine line({make_unit(1), make_unit(2, SimProfile::Gripper), make_unit(5, SimProfile::Pcon)}, "SIM9");
    CHECK_EQ(line.name(), std::string("SIM9"));
    CHECK_EQ(hexs(line_exchange(line, "11", 1)), hexs(frame(1, "11 05 EC FF 02 00 13")));
    CHECK_EQ(hexs(line_exchange(line, "11", 2)), hexs(frame(2, "11 05 EC FF 0A 00 02")));
    CHECK_EQ(hexs(line_exchange(line, "11", 5)), hexs(frame(5, "11 05 AE FF 44 00 00")));
    CHECK(line_exchange(line, "11", 3).empty());  // an address nobody has: the master times out
    CHECK(line_exchange(line, "11", 247).empty());
    // reads of two units interleave on the one wire and each answers from its own state
    CHECK_EQ(read_regs(*line.unit(1), 0x9001, 1)[0], 0x1CE3);  // the slider: 73.95 mm
    CHECK_EQ(line.size(), 3u);
    CHECK(line.is_open());
    CHECK_EQ(line.read_timeout(), kDefaultReadTimeoutS);
    line.set_read_timeout(0.5);
    CHECK_EQ(line.read_timeout(), 0.5);
    CHECK(!line.has_real_timeout());
}

TEST(sim_line_a_duplicate_or_out_of_range_address_is_refused) {
    CHECK_THROWS(SimLine({make_unit(1), make_unit(1)}), std::invalid_argument);
    CHECK_THROWS(SimLine({make_unit(0)}), std::invalid_argument);
    CHECK_THROWS(SimLine({make_unit(248)}), std::invalid_argument);
    CHECK_THROWS(SimLine({std::shared_ptr<SimulatedUnit>()}), std::invalid_argument);
    SimLine line({make_unit(1)});
    CHECK_THROWS(line.add(make_unit(1)), std::invalid_argument);
    CHECK_EQ(line.size(), 1u);  // nothing was added
    CHECK_THROWS(SimLine::with_slaves({1, 2, 2}), std::invalid_argument);
    CHECK_EQ(line.add(make_unit(247))->slave(), 247);
    CHECK_EQ(line.size(), 2u);
}

TEST(sim_line_a_broadcast_reaches_every_unit_and_nobody_answers) {
    SimLine line({make_unit(1), make_unit(2), make_unit(3)});
    CHECK(line_exchange(line, "05 04 03 00 00", 0).empty());  // SON off to every unit
    for (int slave : {1, 2, 3}) CHECK(!line.unit(slave)->servo_on());
    CHECK(line_exchange(line, "05 04 03 FF 00", 0).empty());
    for (int slave : {1, 2, 3}) CHECK(line.unit(slave)->servo_on());
    // a broadcast read acts on nothing and answers nothing
    CHECK(line_exchange(line, "03 90 00 00 02", 0).empty());
    CHECK_EQ(line.request_slaves().size(), 3u);
}

TEST(sim_line_a_unit_taken_off_the_line_stops_answering) {
    SimLine line({make_unit(1), make_unit(2)});
    CHECK(!line_exchange(line, "11", 2).empty());
    const auto removed = line.remove(2);
    CHECK_EQ(removed->slave(), 2);
    CHECK(line_exchange(line, "11", 2).empty());  // a cable pulled
    CHECK_EQ(line.size(), 1u);
    CHECK_THROWS(line.remove(2), std::out_of_range);
    CHECK_THROWS(line.unit(2), std::out_of_range);
    line.add(removed);  // plugged back in
    CHECK(!line_exchange(line, "11", 2).empty());
    const std::vector<int> slaves_now = line.slaves();
    CHECK_EQ(hexs(Bytes(slaves_now.begin(), slaves_now.end())), "01 02");
}

TEST(sim_line_ports_are_named_and_hold_their_slaves) {
    const auto ports = SimLine::sim_ports(3);
    CHECK_EQ(ports.size(), 3u);
    int expected = 1;
    for (const auto& kv : ports) {
        CHECK_EQ(kv.first, "SIM" + std::to_string(expected));  // std::map: SIM1, SIM2, SIM3
        CHECK_EQ(kv.second->name(), kv.first);
        CHECK_EQ(kv.second->slaves().size(), 1u);
        CHECK_EQ(kv.second->slaves()[0], 1);
        ++expected;
    }
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.time_scale = 7.0;
    const auto daisy = SimLine::sim_ports(2, {1, 2, 3}, o, "AX");
    CHECK_EQ(daisy.size(), 2u);
    CHECK(daisy.count("AX1") == 1 && daisy.count("AX2") == 1);
    const auto line = daisy.at("AX2");
    const std::vector<int> daisy_slaves = line->slaves();
    CHECK_EQ(hexs(Bytes(daisy_slaves.begin(), daisy_slaves.end())), "01 02 03");
    for (int slave : {1, 2, 3}) {
        CHECK_EQ(line->unit(slave)->slave(), slave);  // each unit got its own address
        CHECK(line->unit(slave)->profile() == SimProfile::Gripper);
        CHECK_EQ(line->unit(slave)->time_scale(), 7.0);
    }
    CHECK(daisy.at("AX1")->unit(1) != daisy.at("AX2")->unit(1));  // separate units on separate ports
    CHECK_THROWS(SimLine::sim_ports(0), std::invalid_argument);
    CHECK_THROWS(SimLine::sim_ports(65), std::invalid_argument);
    CHECK_EQ(SimLine::sim_ports(64).size(), 64u);
    const auto one = SimLine::with_slaves({4, 9}, o, "ONE");
    CHECK_EQ(one->name(), std::string("ONE"));
    CHECK_EQ(one->unit(9)->slave(), 9);
}

TEST(sim_line_units_on_a_line_move_independently) {
    sim_test::FakeClock clock;
    SimulatedUnitOptions o;
    o.clock = clock.reader();
    o.settle_speed_mm_s = 0.0;
    const auto line = SimLine::with_slaves({1, 2}, o);
    SimulatedUnit& first = *line->unit(1);
    SimulatedUnit& second = *line->unit(2);
    const Bytes request = frame(1, "10 99 00 00 09 12 00 00 20 CB 00 00 00 0A 00 00 27 10 00 1E 00 00 00 00");
    line->write(request.data(), request.size());  // axis 0 only: PCMD 83.95 mm
    Bytes ack(64);
    CHECK_EQ(line->read(ack.data(), ack.size()), 8u);
    clock.advance(0.05);
    CHECK(first.moving());
    CHECK(!second.moving());
    clock.advance(5.0);
    CHECK_NEAR(first.position_mm(), 83.95, 1e-6);
    CHECK_NEAR(second.position_mm(), 73.95, 1e-9);
    // the request log shows the frames of the line in order
    const auto requests = line->requests();
    REQUIRE(requests.size() == 1u);
    CHECK_EQ(hexs(requests[0].second), hexs(request));
}

TEST(sim_line_requests_are_logged_with_time_and_order) {
    SimLine line({make_unit(1), make_unit(2)});
    CHECK(line.requests().empty());
    const double before = now_s();
    line_exchange(line, "03 90 00 00 02", 1);
    line_exchange(line, "03 90 00 00 02", 2);
    line_exchange(line, "03 90 00 00 02", 9);  // an address nobody has is logged too
    line_exchange(line, "05 04 03 FF 00", 0);
    const auto requests = line.requests();
    REQUIRE(requests.size() == 4u);
    CHECK(requests[0].first >= before);
    CHECK(requests[1].first >= requests[0].first);
    CHECK_EQ(hexs(requests[0].second), hexs(frame(1, "03 90 00 00 02")));
    const std::vector<int> slaves = line.request_slaves();
    CHECK_EQ(hexs(Bytes(slaves.begin(), slaves.end())), "01 02 09 00");
    // each unit also logs what it saw (every unit sees every frame)
    CHECK_EQ(line.unit(1)->requests().size(), 4u);
    CHECK_EQ(line.unit(2)->requests().size(), 4u);
    line.clear_requests();
    CHECK(line.requests().empty());
    CHECK(line.request_slaves().empty());
}

TEST(sim_line_has_a_baud_rate_only_when_its_units_model_one) {
    SimLine plain({make_unit(1)});
    CHECK_EQ(plain.baud_rate(), 0);
    CHECK_THROWS(plain.set_baud_rate(115200), std::logic_error);
    SimLine empty;
    CHECK_EQ(empty.baud_rate(), 0);
    CHECK_THROWS(empty.set_baud_rate(115200), std::logic_error);
    SimulatedUnitOptions o;
    o.baud_rate = 115200;
    const auto line = SimLine::with_slaves({1, 2}, o);
    CHECK_EQ(line->baud_rate(), 115200);
    CHECK(!line_exchange(*line, "11", 1).empty());
    line->set_baud_rate(38400);  // the PC side's rate on the shared wire: every unit hears it
    CHECK_EQ(line->baud_rate(), 38400);
    CHECK_EQ(line->unit(1)->baud_rate(), 38400);
    CHECK_EQ(line->unit(2)->baud_rate(), 38400);
    CHECK(line_exchange(*line, "11", 1).empty());  // the units are still at 115200
    CHECK(line_exchange(*line, "11", 2).empty());
    line->set_baud_rate(115200);
    CHECK(!line_exchange(*line, "11", 2).empty());
    // a broadcast baud change reaches every unit: both now listen at 230400
    CHECK(line_exchange(*line, "45 00 04 00 15 00 05", 0).empty());
    CHECK_EQ(line->unit(1)->unit_baud_rate().value(), 230400);
    CHECK_EQ(line->unit(2)->unit_baud_rate().value(), 230400);
    line->set_baud_rate(230400);
    CHECK(!line_exchange(*line, "11", 1).empty());
}

TEST(sim_line_a_break_reaches_every_unit_on_the_wire) {
    SimulatedUnitOptions o;
    o.baud_rate = 115200;
    const auto line = SimLine::with_slaves({1, 2, 3}, o);
    line->send_break(0.1);
    for (int slave : {1, 2, 3}) CHECK_EQ(line->unit(slave)->unit_baud_rate().value(), 115200);
    line->send_break(0.2);  // 150 ms or more drops every unit to 9600
    for (int slave : {1, 2, 3}) CHECK_EQ(line->unit(slave)->unit_baud_rate().value(), 9600);
    line->set_baud_rate(9600);
    CHECK(!line_exchange(*line, "11", 3).empty());
}

TEST(sim_line_close_closes_every_unit_and_refuses_new_frames) {
    SimLine line({make_unit(1), make_unit(2)});
    const auto first = line.unit(1);
    CHECK(line.is_open());
    line.close();
    line.close();  // twice is fine
    CHECK(!line.is_open());
    CHECK(!first->is_open());
    CHECK(!line.unit(2)->is_open());
    const Bytes request = frame(1, "11");
    CHECK_THROWS(line.write(request.data(), request.size()), TransportError);
    std::uint8_t buffer[16];
    CHECK_THROWS(line.read(buffer, sizeof buffer), TransportError);
}

TEST(sim_line_reset_input_clears_every_units_pending_answer) {
    SimLine line({make_unit(1), make_unit(2)});
    const Bytes request = frame(2, "03 90 00 00 02");
    line.write(request.data(), request.size());
    line.reset_input();
    std::uint8_t buffer[64];
    CHECK_EQ(line.read(buffer, sizeof buffer), 0u);
}

TEST(sim_line_reads_never_block_and_honour_the_buffer_size) {
    SimLine line({make_unit(1)});
    std::uint8_t buffer[64];
    CHECK_EQ(line.read(buffer, sizeof buffer), 0u);  // nothing pending
    const Bytes request = frame(1, "03 90 00 00 02");
    line.write(request.data(), request.size());
    CHECK_EQ(line.read(buffer, 5), 5u);
    CHECK_EQ(line.read(buffer, sizeof buffer), 4u);
    CHECK_EQ(line.read(buffer, sizeof buffer), 0u);
}

TEST(sim_line_the_rod_answers_fc11_late_through_the_line) {
    SimLine line({make_unit(1), make_unit(2, SimProfile::Rod)});
    const Bytes request = frame(2, "11");
    line.write(request.data(), request.size());
    std::uint8_t buffer[32];
    CHECK_EQ(line.read(buffer, sizeof buffer), 0u);  // the rod answers after 0.3 s
    sleep_at_least(0.35);
    CHECK_EQ(hexs(Bytes(buffer, buffer + line.read(buffer, sizeof buffer))), hexs(frame(2, "11 05 EC FF 00 00 02")));
}

TEST(sim_line_every_profile_on_one_line_keeps_its_own_identity_and_range) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    int slave = 1;
    for (SimProfile profile : sim_profiles()) units.push_back(make_unit(slave++, profile));
    SimLine line(units, "MIX");
    CHECK_EQ(line.size(), 6u);
    const double strokes[] = {150.0, 9.5, 150.0, 150.0, 360.0, 100.0};
    for (int i = 0; i < 6; ++i) {
        SimulatedUnit& unit = *line.unit(i + 1);
        CHECK_NEAR(unit.stroke_mm(), strokes[i], 1e-9);
        const std::uint32_t maxv = read_item(unit, 7, 4, i + 1).value();
        CHECK_EQ(maxv, static_cast<std::uint32_t>(std::lround(unit.max_speed_mm_s() * 100)));
    }
    const std::set<int> systems = {0xEC, 0xAE, 0x99};
    for (int i = 1; i <= 6; ++i) {
        const Bytes answer = line.unit(i)->handle_frame(frame(i, "11"));
        CHECK(systems.count(answer[3]) == 1);
    }
}

TEST(sim_line_frames_to_a_unit_that_fails_are_the_units_own_business) {
    SimLine line({make_unit(1), make_unit(2)});
    line.unit(1)->inject_fault(SimFault::Timeout);
    CHECK(line_exchange(line, "03 90 05 00 01", 1).empty());  // unit 1 swallows its answer
    CHECK_EQ(line_exchange(line, "03 90 05 00 01", 2).size(), 7u);
    line.unit(2)->inject_fault(SimFault::WrongSlave);
    const Bytes foreign = line_exchange(line, "03 90 05 00 01", 2);
    CHECK_EQ(foreign[0], 3);  // an answer from another address
    CHECK(check_crc(foreign));
    line.unit(1)->close();  // one unit's port is gone: the next frame on the line raises
    const Bytes request = frame(2, "11");
    CHECK_THROWS(line.write(request.data(), request.size()), TransportError);
}

TEST(sim_line_units_keep_the_history_of_the_pc_side_rate) {
    SimLine plain({make_unit(1)});
    CHECK(plain.unit(1)->baud_history().empty());  // no line model, no history
    CHECK_EQ(plain.unit(1)->name(), std::string("SIM"));
    SimulatedUnitOptions o;
    o.baud_rate = 115200;
    o.unit_baud_rate = 38400;
    o.port_name = std::string("COM99");
    const auto line = SimLine::with_slaves({1, 2}, o);
    CHECK_EQ(line->unit(1)->name(), std::string("COM99"));
    line->set_baud_rate(38400);  // every unit hears the new rate
    line->close();
    line->reopen();  // the rate as it was: nothing recorded
    line->reopen(9600);
    for (int slave : {1, 2}) CHECK(line->unit(slave)->baud_history() == std::vector<int>({115200, 38400, 9600}));
}

TEST(sim_gateway_frames_reach_the_unit_through_its_port_and_its_hooks) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Pcon;
    const auto unit = std::make_shared<HookedSimUnit>(o);
    SimGatewayLine line({unit});  // RCON
    CHECK_EQ(line_exchange(line, "03 90 00 00 02", 1).size(), 9u);
    CHECK_EQ(unit->request_log().size(), 1u);  // the unit logged the frame passed on
    // the all-axes monitor reads the unit's status block through its port too
    CHECK_EQ(line_exchange(line, "03 29 00 00 0F", 0xD0).size(), 35u);
    CHECK_EQ(unit->request_log().size(), 2u);
    CHECK_EQ(hexs(unit->request_log().back().second), hexs(frame(1, "03 90 00 00 14")));
    // the unit's hooks apply: an override, a hook answering an exception
    unit->set_word_override(0x9005, 0x1234);
    const Bytes word = line_exchange(line, "03 90 05 00 01", 1);
    REQUIRE(word.size() == 7u);
    CHECK_EQ((word[3] << 8) | word[4], 0x1234);
    unit->set_request_hook([](const Bytes&) {
        SimHookAnswer a;
        a.kind = SimHookAnswer::Kind::Exception;
        a.exception_code = 4;
        return a;
    });
    CHECK_EQ(hexs(line_exchange(line, "03 90 05 00 01", 1)), hexs(frame(1, "83 04")));
    // a hook that throws fails the line's write, as on the unit's own port
    unit->set_request_hook([](const Bytes&) -> SimHookAnswer { throw TransportError("the hook failed"); });
    const Bytes request = frame(1, "03 90 05 00 01");
    CHECK_THROWS(line.write(request.data(), request.size()), TransportError);
    unit->set_request_hook(SimRequestHook());
    unit->power_off();  // a unit without power stays silent behind the gateway
    CHECK(line_exchange(line, "03 90 05 00 01", 1).empty());
    unit->power_on();
    CHECK_EQ(line_exchange(line, "03 90 05 00 01", 1).size(), 7u);
    unit->close();  // a unit whose port is closed fails the line's write
    CHECK_THROWS(line.write(request.data(), request.size()), TransportError);
}

TEST(sim_gateway_silent_no_access_and_a_late_fc11_answer) {
    SimGatewayOptions o;
    o.silent_no_access = true;
    SimGatewayLine line({make_unit(1, SimProfile::Pcon)}, o);
    CHECK(line_exchange(line, "03 00 00 00 08", 1).empty());  // the directory: "system, no access" on RCON
    CHECK(line_exchange(line, "03 80 00 00 01", 1).empty());
    CHECK_EQ(line_exchange(line, "03 90 00 00 02", 1).size(), 9u);
    line.set_silent_no_access(false);
    CHECK(line.unit(1)->requests().size() == 1u);  // the silent reads never reached the unit
    CHECK_EQ(line_exchange(line, "03 00 00 00 08", 1).size(), 21u);
    // ROBONET carries FC11: the rod's late answer comes through the line once it is ready
    SimulatedUnitOptions rod;
    rod.profile = SimProfile::Rod;
    const auto net = sim_robonet_line({2}, false, rod);
    const Bytes request = frame(2, "11");
    net->write(request.data(), request.size());
    std::uint8_t buffer[32];
    CHECK_EQ(net->read(buffer, sizeof buffer), 0u);  // the rod answers after 0.3 s
    sleep_at_least(0.35);
    CHECK_EQ(hexs(Bytes(buffer, buffer + net->read(buffer, sizeof buffer))), hexs(frame(2, "11 05 EC FF 00 00 02")));
}
