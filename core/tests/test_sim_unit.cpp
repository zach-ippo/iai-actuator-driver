// Tests of the SimulatedUnit (sim unit): raw request frames through handle_frame / write / read, built with
// build_frame (no bus, no axis): the seeded live values of every profile, the directory and identification blocks,
// the protocol (exceptions, silence, echoes, faults), coils, IAI 0x45, the parameter RAM / non-volatile store and PARW,
// the position table, alarms, power cycle, software reset, baud modelling. The motion model is in test_sim_motion.cpp,
// SimLine in test_sim_line.cpp. Port of the C# edition's SimulatedUnitTests / SimProfileTests and the Python
// edition's test_sim.py. The simulator models what the Python edition's hardware tests saw (VERIFIED there); this
// edition is UNVERIFIED on hardware. Unofficial: not an IAI product.
#include <atomic>
#include <set>
#include <thread>

#include "harness.hpp"
#include "sim_fixtures.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::sim_test;

namespace {

Bytes read_bytes_all(SimulatedUnit& unit) {
    Bytes data(512);
    const std::size_t n = unit.read(data.data(), data.size());
    data.resize(n);
    return data;
}

}  // namespace

// ------------------------------------------------------------------------------------------ seeded live values

TEST(sim_seed_live_monitor_values) {
    SimulatedUnit unit;
    const Words words = read_regs(unit, 0x9000, 8);
    CHECK_EQ(hexs(Words(words.begin(), words.begin() + 2)), "0000 1CE3");  // 73.95 mm
    CHECK_EQ(words[2], 0);                                                  // no alarm
    CHECK_EQ(words[5], 0x3098);                                             // DSS1 (live)
    CHECK_EQ(words[7], 0x33C2);                                             // DSSE (live)
    CHECK_NEAR(pnow_mm(words), 73.95, 1e-9);
}

TEST(sim_every_profile_serves_the_22_word_status_block) {
    for (SimProfile profile : sim_profiles()) {
        Rig rig(pure_trapezoid(profile));
        SimulatedUnit& unit = rig.u();
        const Words w = read_regs(unit, reg::kMonitorBase, reg::kStatusReadWords);
        REQUIRE(w.size() == 22u);
        CHECK_NEAR(pnow_mm(w), unit.position_mm(), 0.0051);
        const bool pcon = profile == SimProfile::Pcon;
        CHECK_EQ(w[2], pcon ? reg::kAlarmRtcStop : 0);        // the PCON powers up with message alarm 0x069
        CHECK_EQ(w[5], pcon ? 0x3298 : 0x3098);               // PWR, SV, BKRL, HEND, PEND (+ ALML on the PCON)
        CHECK_EQ(w[7], 0x33C2);                               // DSSE: PMSS, PSNS, SSON, RMDS ...
        CHECK_EQ(hexs(Words{w[8], w[9]}), "0000 801F");        // STAT: MPOW SON SV HEND RMDS + b15 (live idle)
        CHECK_EQ(hexs(Words{w[0x10], w[0x11]}), "0000 0000");  // STIM: ms since power on (the clock has not moved)
        CHECK_EQ(w[0x12], 0x0300);
        CHECK_EQ(w[0x13] & 0x0003, 0x0003);                   // ZONS: inside both zones
        CHECK_EQ(w[0x14], 0);                                 // POSS: no stored row has completed
        CHECK_EQ(w[0x15], profile == SimProfile::Slider ? 0x1008 : 0x0000);  // SSSE as read live
        CHECK_EQ(hexs(Words{w[0x0E], w[0x0F]}), "0000 0000");  // DEVI
        // the shorter reads the driver falls back to are served too
        CHECK_EQ(read_regs(unit, reg::kMonitorBase, reg::kStatusBlockWords).size(), 20u);
        CHECK_EQ(read_regs(unit, reg::kMonitorBase, reg::kStatusShortWords).size(), 14u);
        CHECK_EQ(read_regs(unit, reg::kMonitorBase, reg::kStatusMinWords).size(), 8u);
        // the status image 0x0200 mirrors the monitor block (DSS1 DSS2 DSSE POSS ZONS DIPM DOPM SIPM SSSE 0001)
        const Words image = read_regs(unit, 0x0200, 10);
        CHECK_EQ(image[0], w[5]);
        CHECK_EQ(image[1], 0x8000);
        CHECK_EQ(image[2], w[7]);
        CHECK_EQ(image[4], w[0x13]);
        CHECK_EQ(image[6], w[4]);
        CHECK_EQ(image[8], w[0x15]);
        CHECK_EQ(image[9], 0x0001);
    }
}

TEST(sim_seed_live_direct_move_block_and_drg1) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(read_regs(unit, 0x9900, 7)), "0000 1CE3 0000 000A 0001 1170 001E");
    CHECK_EQ(read_regs(unit, 0x0D00, 1)[0], 0x1000);  // DRG1: servo on
    CHECK_EQ(read_regs(unit, 0x0D02, 1)[0], 0x0100);  // DRGE: PMSL
}

TEST(sim_seed_live_position_rows) {
    SimulatedUnit unit;
    const auto row = [&](int n) { return read_regs(unit, 0x1000 + 16 * n, 16); };
    CHECK_EQ(hexs(head(row(0), 6)), "0000 0000 0000 000A 0000 2A62");  // 0.00 mm, 108.50 mm/s
    CHECK_EQ(hexs(head(row(1), 6)), "0000 3A98 0000 000A 0000 6112");  // 150.00 mm, 248.50 mm/s
    CHECK_EQ(row(2)[1], 0x1D4C);                                                                 // 75.00 mm
    CHECK_EQ(row(1)[10], 0x1E);                                                                  // 0.30 G
    CHECK_EQ(row(3)[12], 0xB2);
    CHECK_EQ(hexs(unit.position_row(1)), hexs(row(1)));
}

TEST(sim_seed_directory) {
    SimulatedUnit unit;
    const Words header = read_regs(unit, 0, 8);
    CHECK_EQ(hexs(head(header, 5)), "0001 0008 001B 0100 0100");
    const Words records = read_block(unit, 8, 8 * header[2]);
    std::map<int, Words> by_class;
    for (int i = 0; i < header[2]; ++i)
        by_class[records[8 * i]] = Words(records.begin() + 8 * i, records.begin() + 8 * i + 8);
    // class id -> data address (the addresses the driver finds through the directory)
    const std::map<int, int> bases = {{0x0005, 0x0500}, {0x0006, 0x0600}, {0x0007, 0x0700}, {0x0008, 0x0800},
                                      {0x0009, 0x0900}, {0x000D, 0x0D00}, {0x0010, 0x1000}, {0x0101, 0x9000},
                                      {0x0102, 0x9800}, {0x0103, 0x9900}};
    for (const auto& kv : bases) {
        REQUIRE(by_class.count(kv.first) == 1);
        CHECK_EQ(by_class[kv.first][4], kv.second);
    }
    CHECK_EQ(by_class[0x000D][6], reg::kExpectedCoilBase);  // the coil base of the I/O control class
}

TEST(sim_seed_report_slave_id_matches_the_unit_tested) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(exchange(unit, "11")), "01 11 05 EC FF 02 00 13 C1 5D");  // byte for byte the test EC's answer
}

TEST(sim_every_profile_answers_its_own_fc11_and_names) {
    struct Case {
        SimProfile profile;
        const char* name;
        const char* fc11;
    };
    const Case cases[] = {{SimProfile::Slider, "slider", "EC FF 02 00 13"},   {SimProfile::Gripper, "gripper", "EC FF 0A 00 02"},
                          {SimProfile::Rod, "rod", "EC FF 00 00 02"},         {SimProfile::Pcon, "pcon", "AE FF 44 00 00"},
                          {SimProfile::Rotary, "rotary", "EC FF 00 00 02"},   {SimProfile::Unknown, "unknown", "99 FF 7E 01 00"}};
    CHECK_EQ(sim_profiles().size(), 6u);
    for (const Case& c : cases) {
        SimulatedUnit unit(c.profile);
        CHECK_EQ(std::string(to_string(c.profile)), std::string(c.name));
        CHECK_EQ(sim_profile_from_string(c.name).value(), c.profile);
        CHECK(unit.profile() == c.profile);
        CHECK_EQ(hexs(pdu_of(exchange(unit, "11"))), std::string("11 05 ") + c.fc11);
        CHECK_EQ(unit.name(), std::string("SIM"));
    }
    CHECK(sim_profile_from_string("GRIPPER").has_value());
    CHECK(!sim_profile_from_string("lathe").has_value());
}

TEST(sim_every_profile_directory_and_identification_match_its_fixture) {
    for (SimProfile profile : sim_profiles()) {
        SimulatedUnit unit(profile);
        const auto& fx = sim_detail::fixture(profile);
        const Words index = util::hex_words(fx.index_table_hex);
        CHECK_EQ(hexs(read_block(unit, 0, static_cast<int>(index.size()))), hexs(index));
        const Words ident = util::hex_words(fx.identification_hex);
        CHECK_EQ(ident.size(), 72u);
        CHECK_EQ(hexs(read_block(unit, 0x0600, 72)), hexs(ident));
        // the model string of the identification block, as text (the unknown unit has a blank one)
        const Words model = read_block(unit, 0x0620, 0x28);
        const std::string text = ascii_of(model);
        if (profile == SimProfile::Unknown || profile == SimProfile::Pcon) {  // read live: no model string
            CHECK(util::trim(text).empty());
        } else {
            CHECK(!util::trim(text).empty());
        }
    }
    SimulatedUnit slider(SimProfile::Slider);
    CHECK_EQ(ascii_of(read_block(slider, 0x0620, 0x28)), std::string("MWB-EC-DSRR4-WA-MF"));
    SimulatedUnit gripper(SimProfile::Gripper);
    CHECK_EQ(ascii_of(read_block(gripper, 0x0620, 0x28)),
             std::string("EC-GRBP8M-20-1-1-PN-WA-WL2"));
}

TEST(sim_reads_are_bounded_by_the_directory) {
    for (SimProfile profile : sim_profiles()) {
        SimulatedUnit unit(profile);
        CHECK_EQ(read_exception(unit, 0x7000, 1), 2);      // outside every block
        CHECK_EQ(read_exception(unit, 0x9020, 4), 2);      // spans the end of the monitor block
        CHECK_EQ(read_exception(unit, 0x0F00, 1), 0);      // inside a directory block without recorded data: 0
        CHECK_EQ(read_regs(unit, 0x0F00, 1)[0], 0);
        CHECK_EQ(read_exception(unit, 0x00E0, 1), 2);      // just past the index table
        CHECK_EQ(read_exception(unit, 0x9000, 0), 3);      // a zero count
        CHECK_EQ(read_exception(unit, 0x9000, 126), 3);    // more than 125 words
        CHECK_EQ(read_exception(unit, 0x1000, 125), 0);    // 125 words are fine
    }
}

TEST(sim_profile_values_come_from_the_units_own_directory) {
    struct Case {
        SimProfile profile;
        double stroke;
        double start;
        int rows;
        int akid;
    };
    // stroke, start position and stored rows as the driver reads them; AKID 7.33 is the rotary / unknown override
    const Case cases[] = {{SimProfile::Slider, 150.0, 73.95, 3, -1}, {SimProfile::Gripper, 9.5, 9.5, 2, -1},
                          {SimProfile::Rod, 150.0, 0.0, 2, -1},      {SimProfile::Pcon, 150.0, 60.0, 64, -1},
                          {SimProfile::Rotary, 360.0, 0.0, 2, 1},    {SimProfile::Unknown, 100.0, 0.0, 2, 42}};
    for (const Case& c : cases) {
        SimulatedUnit unit(c.profile);
        CHECK_NEAR(unit.stroke_mm(), c.stroke, 1e-9);
        CHECK_NEAR(unit.position_mm(), c.start, 1e-9);
        CHECK_EQ(unit.position_count(), c.rows);
        const Span limits = unit.soft_limits_mm();
        CHECK_NEAR(limits.min, -0.30, 1e-9);
        CHECK_NEAR(limits.max, c.stroke + 0.30, 1e-9);
        CHECK_EQ(hexs(read_regs(unit, 0x0700, 4)),
                 hexs(Words{reg::to_i32_words(static_cast<std::int32_t>(std::lround((c.stroke + 0.30) * 100))).first,
                            reg::to_i32_words(static_cast<std::int32_t>(std::lround((c.stroke + 0.30) * 100))).second,
                            reg::to_i32_words(-30).first, reg::to_i32_words(-30).second}));
        CHECK_EQ(read_item(unit, 7, 1).value(), static_cast<std::uint32_t>(std::lround((c.stroke + 0.30) * 100)));
        CHECK_EQ(read_item(unit, 7, 4).value(), static_cast<std::uint32_t>(std::lround(unit.max_speed_mm_s() * 100)));  // MAXV
        CHECK_EQ(read_item(unit, 8, 10).value(), static_cast<std::uint32_t>(std::lround(unit.max_speed_mm_s() * 100)));  // 8.10
        if (c.akid >= 0) CHECK_EQ(read_item(unit, 7, 33).value(), static_cast<std::uint32_t>(c.akid));
        CHECK_EQ(unit.slave(), 1);
    }
    CHECK(SimulatedUnit(SimProfile::Slider).absolute());
    CHECK(!SimulatedUnit(SimProfile::Rod).absolute());
    CHECK(SimulatedUnit(SimProfile::Pcon).absolute());
}

TEST(sim_options_override_the_profile) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.stroke_mm = 20.0;
    o.position_mm = 5.0;
    o.max_speed_mm_s = 40.0;
    o.system_code = 0xED;
    o.unit_code = 0x33;
    o.firmware = std::make_pair(1, 9);
    o.slave = 4;
    SimulatedUnit unit(o);
    CHECK_EQ(unit.slave(), 4);
    CHECK_NEAR(unit.stroke_mm(), 20.0, 1e-9);
    CHECK_NEAR(unit.position_mm(), 5.0, 1e-9);
    CHECK_EQ(hexs(pdu_of(exchange(unit, "11", 4))), "11 05 ED FF 33 01 09");
    CHECK_EQ(hexs(Words{read_regs(unit, 0x0700, 4, 4)[1]}), "07EE");  // 20.30 mm
    CHECK_EQ(read_item(unit, 7, 4, 4).value(), 4000u);
    CHECK_EQ(read_item(unit, 7, 1, 4).value(), 2030u);
    CHECK(exchange(unit, "11", 1).empty());  // another slave: silence
    CHECK_EQ(unit.system_code(), 0xED);
    CHECK_EQ(unit.unit_code(), 0x33);
    SimulatedUnitOptions hidden;
    hidden.soft_limits = false;
    SimulatedUnit no_limits(hidden);
    CHECK_EQ(read_exception(no_limits, 0x0700, 4), 2);  // the soft-limit block is not served
}

TEST(sim_options_are_validated) {
    SimulatedUnitOptions bad_scale;
    bad_scale.time_scale = 0.0;
    CHECK_THROWS(SimulatedUnit{bad_scale}, std::invalid_argument);
    SimulatedUnitOptions bad_profile;
    bad_profile.profile = static_cast<SimProfile>(42);
    CHECK_THROWS(SimulatedUnit{bad_profile}, std::invalid_argument);
    SimulatedUnitOptions bad_dopm;
    bad_dopm.dopm = static_cast<DopmLayout>(4);
    CHECK_THROWS(SimulatedUnit{bad_dopm}, std::invalid_argument);
    CHECK_THROWS(SimulatedUnit(SimProfile::Slider, 1, -1.0), std::invalid_argument);
}

TEST(sim_rod_answers_fc11_late) {
    SimulatedUnit rod(SimProfile::Rod);
    const Bytes request = frame(1, "11");
    rod.write(request.data(), request.size());
    std::uint8_t buffer[32];
    CHECK_EQ(rod.read(buffer, sizeof buffer), 0u);  // not yet: the rod answers after 0.3 s (0.2-0.4 s on hardware)
    sleep_at_least(0.35);
    CHECK_EQ(hexs(Bytes(buffer, buffer + rod.read(buffer, sizeof buffer))), hexs(frame(1, "11 05 EC FF 00 00 02")));
    // a request after a reset_input() supersedes a delayed answer still pending: only the new answer is readable
    rod.write(request.data(), request.size());
    rod.reset_input();
    const Bytes other = frame(1, "03 90 00 00 02");
    rod.write(other.data(), other.size());
    CHECK_EQ(rod.read(buffer, sizeof buffer), 9u);  // the FC03 answer only, at once
    // other profiles answer at once
    SimulatedUnit slider;
    slider.write(request.data(), request.size());
    CHECK_EQ(slider.read(buffer, sizeof buffer), 10u);
}

// ------------------------------------------------------------------------------------------ protocol

TEST(sim_protocol_exception_codes) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(exchange(unit, "03 70 00 00 01")), hexs(expect(1, "83 02")));
    CHECK_EQ(hexs(exchange(unit, "03 90 20 00 04")), hexs(expect(1, "83 02")));  // spans the end of the block
    CHECK_EQ(hexs(exchange(unit, "03 90 00 00 00")), hexs(expect(1, "83 03")));
    CHECK_EQ(hexs(exchange(unit, "01 00 00 00 01")), hexs(expect(1, "81 01")));  // other function codes
    // FC17: the write first (PCMD high word alone: no move), then the read (sim_profile.cpp)
    CHECK_EQ(hexs(exchange(unit, "17 90 00 00 02 99 00 00 01 02 00 00")), hexs(expect(1, "17 04 00 00 1C E3")));
    CHECK_EQ(hexs(exchange(unit, "17 90 00 00 02 99 00 00 01 04 00 00")), hexs(expect(1, "97 03")));  // byte count
    CHECK_EQ(hexs(exchange(unit, "48 00 00 34")), hexs(expect(1, "C8 03")));                           // a short 0x48
    CHECK_EQ(hexs(exchange(unit, "05 04 03 12 34")), hexs(expect(1, "85 03")));
    CHECK_EQ(hexs(exchange(unit, "05 04 99 FF 00")), hexs(expect(1, "85 02")));
    CHECK_EQ(hexs(exchange(unit, "06 90 00 00 01")), hexs(expect(1, "86 02")));  // read-only
    CHECK_EQ(hexs(exchange(unit, "03 90 00 00")), hexs(expect(1, "83 03")));      // a short body
    CHECK_EQ(hexs(exchange(unit, "10 99 00 00 02 03 00 00 00")), hexs(expect(1, "90 03")));  // byte count != 2 x count
    CHECK_EQ(hexs(exchange(unit, "11 00")), hexs(expect(1, "91 03")));            // FC11 takes no body
    CHECK_EQ(hexs(exchange(unit, "45 00 04 00")), hexs(expect(1, "C5 03")));      // a short 0x45
    CHECK_EQ(hexs(exchange(unit, "10 99 00 00 00 00")), hexs(expect(1, "90 03")));
    CHECK_EQ(hexs(exchange(unit, "10 90 00 00 01 02 00 00")), hexs(expect(1, "90 02")));  // writing the monitor block
}

TEST(sim_protocol_silent_cases) {
    SimulatedUnit unit;
    Bytes bad = frame(1, "03 90 00 00 02");
    bad.back() = static_cast<std::uint8_t>(bad.back() ^ 0xFF);
    CHECK(unit.handle_frame(bad).empty());                      // bad CRC
    CHECK(unit.handle_frame(Bytes{0x01, 0x03}).empty());        // too short
    CHECK(exchange(unit, "03 90 00 00 02", 5).empty());         // another slave
    CHECK(exchange(unit, "05 04 03 00 00", 0).empty());         // broadcast: no answer...
    CHECK(!unit.servo_on());                                    // ...but executed
    CHECK(exchange(unit, "03 90 00 00 02", 0).empty());         // a broadcast read acts on nothing
    CHECK(exchange(unit, "01 00 00 00 01", 0).empty());
}

TEST(sim_protocol_echoes) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(exchange(unit, "05 04 27 FF 00")), "01 05 04 27 FF 00 3D 01");
    CHECK_EQ(hexs(exchange(unit, "10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E")), "01 10 99 00 00 07 AF 57");
    CHECK_EQ(hexs(exchange(unit, "06 0D 03 00 02")), hexs(expect(1, "06 0D 03 00 02")));
}

TEST(sim_fault_kinds) {
    SimulatedUnit unit;
    const Bytes good = exchange(unit, "03 90 05 00 01");
    CHECK_EQ(good.size(), 7u);
    unit.inject_fault(SimFault::Timeout);
    CHECK(exchange(unit, "03 90 05 00 01").empty());
    unit.inject_fault(SimFault::Crc);
    const Bytes bad_crc = exchange(unit, "03 90 05 00 01");
    CHECK_EQ(bad_crc.size(), good.size());
    CHECK(bad_crc != good);
    CHECK(!check_crc(bad_crc));
    unit.inject_fault(SimFault::Partial);
    CHECK(exchange(unit, "03 90 05 00 01").size() < good.size());
    unit.inject_fault(SimFault::WrongSlave);
    const Bytes foreign = exchange(unit, "03 90 05 00 01");
    CHECK_EQ(foreign[0], 2);
    CHECK(check_crc(foreign));
    CHECK_EQ(hexs(exchange(unit, "03 90 05 00 01")), hexs(good));  // back to normal
    // count > 1 queues several; a fault is consumed only by an answer (a silent request leaves it for the next)
    unit.inject_fault(SimFault::Timeout, 2);
    CHECK(exchange(unit, "03 90 05 00 01", 5).empty());
    CHECK(exchange(unit, "03 90 05 00 01").empty());
    CHECK(exchange(unit, "03 90 05 00 01").empty());
    CHECK_EQ(hexs(exchange(unit, "03 90 05 00 01")), hexs(good));
    CHECK_THROWS(unit.inject_fault(static_cast<SimFault>(99)), std::invalid_argument);
}

TEST(sim_closed_unit_is_a_transport_error) {
    SimulatedUnit unit;
    CHECK(unit.is_open());
    const Bytes request = frame(1, "03 90 00 00 02");
    unit.close();
    unit.close();  // twice is fine
    CHECK(!unit.is_open());
    CHECK_THROWS(unit.write(request.data(), request.size()), TransportError);
    CHECK_THROWS(unit.handle_frame(request), TransportError);
    std::uint8_t b[8];
    CHECK_THROWS(unit.read(b, sizeof b), TransportError);
}

TEST(sim_transport_write_read_round_trip) {
    SimulatedUnit unit;
    const Bytes request = frame(1, "03 90 00 00 02");
    unit.write(request.data(), request.size());
    std::uint8_t part[4];
    CHECK_EQ(unit.read(part, sizeof part), 4u);  // reads return what is there, up to the buffer size
    CHECK_EQ(unit.read(part, sizeof part), 4u);  // 9 bytes: 4 + 4 + 1
    CHECK_EQ(unit.read(part, sizeof part), 1u);
    CHECK_EQ(unit.read(part, sizeof part), 0u);  // never blocks
    unit.write(request.data(), request.size());
    unit.reset_input();
    CHECK(read_bytes_all(unit).empty());
    CHECK_EQ(unit.read_timeout(), kDefaultReadTimeoutS);
    unit.set_read_timeout(0.5);
    CHECK_EQ(unit.read_timeout(), 0.5);
    CHECK_EQ(unit.baud_rate(), 0);  // the line is not modelled
    CHECK_THROWS(unit.set_baud_rate(9600), std::logic_error);
    CHECK(!unit.has_real_timeout());
}

TEST(sim_requests_and_events_are_logged) {
    SimulatedUnit unit;
    CHECK(unit.requests().empty());
    exchange(unit, "03 90 00 00 02");
    exchange(unit, "03 90 00 00 04", 5);  // another slave's request is logged too
    CHECK_EQ(unit.requests().size(), 2u);
    CHECK_EQ(hexs(unit.requests()[0]), hexs(frame(1, "03 90 00 00 02")));
    unit.clear_requests();
    CHECK(unit.requests().empty());
    for (int i = 0; i < 2100; ++i) exchange(unit, "03 90 00 00 01");
    CHECK_EQ(unit.requests().size(), 2000u);  // the last 2000
    coil(unit, reg::kCoilSon, false);
    CHECK(contains(unit.events(), "servo off"));
    unit.clear_events();
    CHECK(unit.events().empty());
}

TEST(sim_register_value_is_what_an_fc03_returns) {
    SimulatedUnit unit;
    CHECK_EQ(unit.register_value(0x9001), 0x1CE3);
    CHECK_EQ(unit.register_value(0x0002), 0x001B);
    CHECK_THROWS(unit.register_value(0x7000), std::out_of_range);
}

// ------------------------------------------------------------------------------------------ coils

TEST(sim_coils_servo_alarm_reset_and_unknown_addresses) {
    SimulatedUnit unit;
    CHECK_EQ(coil(unit, reg::kCoilSon, false), 0);
    CHECK(!unit.servo_on());
    CHECK_EQ(read_regs(unit, reg::kDrg1, 1)[0] & 0x1000, 0);
    CHECK_EQ(read_regs(unit, reg::kDss1, 1)[0] & reg::kDss1Sv, 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(unit.servo_on());
    CHECK_EQ(coil(unit, reg::kCoilSfty, true), 0);
    CHECK(unit.safety_speed());
    CHECK(read_regs(unit, reg::kDss1, 1)[0] & reg::kDss1Sfty);
    CHECK(read_regs(unit, reg::kDrg1, 1)[0] & 0x4000);
    CHECK_EQ(coil(unit, reg::kCoilSfty, false), 0);
    CHECK(!unit.safety_speed());
    // alarm reset is a rising edge
    unit.trigger_alarm(0x0E0);
    CHECK_EQ(unit.alarm_code(), 0x0E0);
    CHECK(!unit.servo_on());  // a major alarm switches the servo off
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(!unit.servo_on());  // refused while the alarm is active
    CHECK_EQ(coil(unit, reg::kCoilAlrs, true), 0);
    CHECK_EQ(unit.alarm_code(), 0);
    CHECK_EQ(coil(unit, reg::kCoilAlrs, false), 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(unit.servo_on());
    // coils the unit does not know
    CHECK_EQ(coil(unit, 0x0499, true), 2);
    CHECK_EQ(coil(unit, 0x042F, true), 2);  // (0x042E is PCRF: the command profile, sim_profile.cpp)
    CHECK(unit.coil_state(reg::kCoilSon));
    CHECK(!unit.coil_state(0x0499));
    unit.set_coil_state(0x0499, true);  // direct access: the stored level only
    CHECK(unit.coil_state(0x0499));
}

TEST(sim_pmsl_off_ignores_other_coils) {
    SimulatedUnit unit;
    CHECK_EQ(coil(unit, reg::kCoilPmsl, false), 0);
    CHECK(!unit.pmsl());
    CHECK_EQ(read_regs(unit, reg::kDsse, 1)[0] & reg::kDssePmss, 0);
    CHECK_EQ(read_regs(unit, reg::kDrge, 1)[0] & 0x0100, 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, false), 0);
    CHECK(unit.servo_on());  // other coils are ignored (DOC; UNVERIFIED on the EC)
    CHECK_EQ(iai(unit, reg::kIaiCmdServo, 0), kSimRejectMonitorMode);
    CHECK_EQ(write_regs(unit, 0x9900, move_block(80.0, 0.1, 8.0, 0.1)), 0);
    CHECK(unit.moving());  // hardware test 18: 0x9900 moves run with PMSS = 0
    CHECK_EQ(coil(unit, reg::kCoilStop, true), 0);  // hardware test 19: STOP stops a Modbus move with PMSS = 0
    CHECK(contains(unit.events(), "stop"));
    CHECK(unit.stop_held());
    CHECK_EQ(coil(unit, reg::kCoilStop, false), 0);
    CHECK(contains(unit.events(), "stop released"));
    CHECK(!unit.stop_held());
    CHECK_EQ(coil(unit, reg::kCoilPmsl, true), 0);
    CHECK(unit.pmsl());
}

TEST(sim_drg1_register_writes_act_like_the_coils) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(write_reg(unit, reg::kDrg1, 0x0000), 0);
    CHECK(!unit.servo_on());
    CHECK_EQ(write_reg(unit, reg::kDrg1, 0x1000), 0);
    CHECK(unit.servo_on());
    CHECK_EQ(write_reg(unit, reg::kDrg1, 0x1000 | 0x0010), 0);  // HOME rising
    CHECK(!unit.homed());
    CHECK(contains(unit.events(), "home"));
    CHECK_EQ(write_reg(unit, reg::kDrg1, 0x1000 | 0x4000), 0);  // SFTY
    CHECK(unit.safety_speed());
    CHECK_EQ(write_reg(unit, reg::kDrg2, 0x00AA), 0);
    CHECK_EQ(read_regs(unit, reg::kDrg2, 1)[0], 0x00AA);
    CHECK_EQ(write_reg(unit, reg::kPosr, 2), 0);
    CHECK_EQ(read_regs(unit, reg::kPosr, 1)[0], 2);
}

// ------------------------------------------------------------------------------------------ IAI 0x45

TEST(sim_iai_servo_and_rejections) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(exchange(unit, "45 00 09 00 00 00 00")), hexs(expect(1, "45 00 00")));  // servo off: accepted
    CHECK(!unit.servo_on());
    CHECK_EQ(iai(unit, reg::kIaiCmdServo, 1), 0);
    CHECK(unit.servo_on());
    CHECK_EQ(iai(unit, 0x0D), kSimRejectCommandCode);   // not simulated
    CHECK_EQ(iai(unit, 0x05, 2), kSimRejectCommandCode);  // reset with an unknown op1
    CHECK_EQ(iai(unit, 0x04, 0x99), kSimRejectCommandCode);  // an unknown baud code
    CHECK_EQ(iai(unit, 0x0B), 0);                        // decel stop
    CHECK(contains(unit.events(), "stop"));
}

TEST(sim_iai_home_operand_and_state_rejections) {
    SimulatedUnit unit;
    CHECK_EQ(iai(unit, reg::kIaiCmdHome, 0), kSimRejectOperand1);  // op1 = 0 rejected 0x11 (VERIFIED in the Python edition)
    CHECK_EQ(iai(unit, reg::kIaiCmdHome, 7), 0);                   // op1 = the home pattern accepted
    CHECK(!unit.homed());
    CHECK_EQ(iai(unit, reg::kIaiCmdHome, 7), kSimRejectHoming);    // already homing: 0x24
    CHECK_EQ(iai(unit, reg::kIaiCmdServo, 0), 0);                  // servo off stops the home
    CHECK_EQ(iai(unit, reg::kIaiCmdHome, 7), kSimRejectServoOff);  // servo off: 0x23
    CHECK_EQ(iai(unit, reg::kIaiCmdMovePosition, 1, 255), kSimRejectServoOff);
}

TEST(sim_iai_alarm_reset_and_software_reset) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    unit.trigger_alarm(0x0D9);
    CHECK_EQ(unit.alarm_code(), 0x0D9);
    CHECK_EQ(hexs(exchange(unit, "45 00 05 00 00 00 00")), hexs(expect(1, "45 00 00")));  // 01 45 00 05 00 00 00 00
    CHECK_EQ(unit.alarm_code(), 0);
    // op1 = 1 is the software reset: with the servo ON it raises alarm 0x090 (UNVERIFIED for 0x45; VERIFIED for the
    // SFRS coil in the Python edition, test R1) and nothing restarts
    CHECK_EQ(iai(unit, 0x09, 1), 0);  // the major alarm had switched the servo off
    CHECK(unit.servo_on());
    CHECK_EQ(iai(unit, 0x05, 1), 0);
    CHECK_EQ(unit.alarm_code(), kSimAlarmResetServoOn);
    CHECK(!unit.servo_on());
    CHECK_EQ(unit.software_resets(), 0);
    CHECK(contains(unit.events(), "software reset refused: servo on"));
    // with the servo off it restarts: the reply goes out, then the unit is silent for reset_s simulated seconds
    CHECK_EQ(hexs(exchange(unit, "45 00 05 00 01 00 00")), hexs(expect(1, "45 00 00")));
    CHECK_EQ(unit.software_resets(), 1);
    CHECK(exchange(unit, "03 90 00 00 02").empty());  // silent while it restarts
    CHECK(contains(unit.events(), "frame ignored (restarting after a software reset)"));
    rig.clock.advance(1.2);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    CHECK(unit.servo_on());  // servo ON after the restart
    CHECK(!unit.pmsl());     // PMSS 0 after an EC software reset
    CHECK(unit.homed());     // an absolute unit keeps HEND
    CHECK_EQ(unit.alarm_code(), 0);
}

TEST(sim_software_reset_depends_on_the_profile) {
    {
        Rig rod(pure_trapezoid(SimProfile::Rod));
        CHECK(rod->homed());
        iai(rod.u(), 0x09, 0);
        CHECK_EQ(iai(rod.u(), 0x05, 1), 0);
        rod.clock.advance(1.5);
        CHECK(!rod->homed());  // the EC-R6S rod (incremental) loses HEND at a software reset
        CHECK_EQ(read_regs(rod.u(), reg::kDss1, 1)[0] & reg::kDss1Hend, 0);
    }
    {
        Rig pcon(pure_trapezoid(SimProfile::Pcon));
        CHECK(pcon->pmsl());
        CHECK_EQ(pcon->alarm_code(), reg::kAlarmRtcStop);  // message level at power-up: the axis still runs
        iai(pcon.u(), 0x09, 0);
        CHECK_EQ(iai(pcon.u(), 0x05, 1), 0);
        pcon.clock.advance(1.5);
        CHECK(pcon->pmsl());   // the PCON keeps PMSS 1 at a reset (VERIFIED in the Python edition)
        CHECK(pcon->homed());
        CHECK_EQ(pcon->alarm_code(), reg::kAlarmRtcStop);  // and its clock is still unset
    }
    {
        Rig gripper(pure_trapezoid(SimProfile::Gripper));
        CHECK(gripper->pmsl());
        iai(gripper.u(), 0x09, 0);
        CHECK_EQ(iai(gripper.u(), 0x05, 1), 0);
        gripper.clock.advance(1.5);
        CHECK_EQ(gripper->software_resets(), 1);
        CHECK(gripper->pmsl());  // the EC-GRBP8M gripper keeps PMSS 1 (VERIFIED in the Python edition, 2026-10-29)
        CHECK(gripper->servo_on());
        CHECK(gripper->homed());
    }
}

TEST(sim_iai_cmd6_rejects_a_row_the_position_mode_does_not_run) {
    // the EC-GRBP8M gripper in 2-position mode (P9 = 0) rejected row 2 with 0x11 and nothing moved (VERIFIED in the
    // Python edition); rows 0 and 1 still run
    Rig gripper(pure_trapezoid(SimProfile::Gripper));
    const double before = gripper->position_mm();
    CHECK_EQ(iai(gripper.u(), reg::kIaiCmdMovePosition, 2, 46), kSimRejectOperand1);
    gripper.clock.advance(1.0);
    CHECK_EQ(gripper->override_moves(), 0);
    CHECK_NEAR(gripper->position_mm(), before, 1e-9);
    CHECK_EQ(gripper->alarm_code(), 0);
    CHECK(contains(gripper->events(), "0x45 cmd 6 row 2 rejected: not used in 2-position mode"));
    CHECK_EQ(iai(gripper.u(), reg::kIaiCmdMovePosition, 0, 46), 0);
    CHECK_EQ(gripper->override_moves(), 1);
    // the 3-position slider runs row 2
    Rig slider(pure_trapezoid(SimProfile::Slider));
    CHECK_EQ(iai(slider.u(), reg::kIaiCmdMovePosition, 2, 255), 0);
    CHECK_EQ(slider->override_moves(), 1);
}

TEST(sim_iai_action_mode) {
    SimulatedUnit unit;
    CHECK_EQ(iai(unit, 0x0C, 0, 0), 0);  // PIO control
    CHECK(!unit.pmsl());
    CHECK_EQ(read_regs(unit, reg::kDsse, 1)[0] & reg::kDssePmss, 0);
    CHECK_EQ(iai(unit, 0x0C, 1, 1), 0);  // PC control + safety speed (accepted with PMSS = 0 too)
    CHECK(unit.pmsl());
    CHECK(unit.safety_speed());
    CHECK_EQ(iai(unit, 0x0C, 1, 0), 0);
    CHECK(!unit.safety_speed());
    CHECK_EQ(iai(unit, 0x0C, 2, 0), kSimRejectOperand1);
    CHECK_EQ(iai(unit, 0x0C, 1, 2), 0x12);
    CHECK(contains(unit.events(), "action mode PC, safety on"));
    // switching to PC control mid-move stops the move (worst case, like the PMSL coil)
    Rig rig(pure_trapezoid());
    iai(rig.u(), 0x0C, 0, 0);
    write_regs(rig.u(), 0x9900, move_block(120.0, 0.1, 20.0, 0.1));
    rig.clock.advance(0.5);
    CHECK(rig->moving());
    iai(rig.u(), 0x0C, 1, 0);
    CHECK(contains(rig->events(), "move stopped by the action-mode change"));
    rig.clock.advance(2.0);
    CHECK(!rig->moving());
}

TEST(sim_iai_cmd6_runs_a_stored_row_at_op2_plus_1_of_256) {
    Rig full(pure_trapezoid());
    CHECK_EQ(iai(full.u(), reg::kIaiCmdMovePosition, 1, 255), 0);  // row 1: 150.00 mm at 248.50 mm/s
    full.clock.advance(0.2);
    const Words v_full = read_regs(full.u(), reg::kVnow, 2);
    CHECK_EQ(reg::from_i32_words(v_full[0], v_full[1]), 24960);  // 248.5 mm/s reads 19.9 x 12.48, quantized to 20 steps
    CHECK_EQ(full->override_moves(), 1);
    Rig half(pure_trapezoid());
    CHECK_EQ(iai(half.u(), reg::kIaiCmdMovePosition, 1, 127), 0);  // (127 + 1) / 256 of the stored speed
    half.clock.advance(0.2);
    const Words v_half = read_regs(half.u(), reg::kVnow, 2);
    CHECK_EQ(reg::from_i32_words(v_half[0], v_half[1]), 12480);
    CHECK(contains(half->events(), "move to position 1 at 50.00 %"));
    // the PCON-CB's steps (ON-WIRE): code 13 ran an 840 mm/s row at 46.0 mm/s = 14 / 256, not 13 / 255
    Rig slow(pure_trapezoid());
    CHECK_EQ(iai(slow.u(), reg::kIaiCmdMovePosition, 1, 13), 0);
    CHECK(contains(slow->events(), "move to position 1 at 5.47 %"));
    half.clock.advance(5.0);
    CHECK_NEAR(half->position_mm(), 150.0, 1e-6);
    CHECK_EQ(read_regs(half.u(), reg::kPoss, 1)[0], 1);  // POSS: the completed stored row
    // rejections
    CHECK_EQ(iai(half.u(), reg::kIaiCmdMovePosition, 1, 0), 0x12);
    // a row the EC's position mode does not run (the slider runs rows 0-2): 0x11, nothing moves, no alarm
    CHECK_EQ(iai(half.u(), reg::kIaiCmdMovePosition, 900, 255), kSimRejectOperand1);
    CHECK_EQ(half->alarm_code(), 0);
    CHECK_EQ(half->override_moves(), 1);
    // a CON unit runs the row it is given: a row past the table raises alarm 0x085
    Rig pcon(pure_trapezoid(SimProfile::Pcon));
    CHECK_EQ(iai(pcon.u(), reg::kIaiCmdMovePosition, 900, 255), 0);
    CHECK_EQ(pcon->alarm_code(), kSimAlarmPositionNo);
    coil(half.u(), reg::kCoilPmsl, false);
    CHECK_EQ(iai(half.u(), reg::kIaiCmdMovePosition, 1, 255), kSimRejectMonitorMode);
}

TEST(sim_broadcast_baud_change_and_ignored_code) {
    SimulatedUnitOptions o;
    o.baud_rate = 230400;  // the PC side and the unit start at 230400
    SimulatedUnit unit(o);
    CHECK_EQ(unit.baud_rate(), 230400);
    CHECK_EQ(unit.unit_baud_rate().value(), 230400);
    // 00 45 00 04 00 <code> 00 05: never answered
    CHECK(exchange(unit, "45 00 04 00 14 00 05", 0).empty());  // 115200 (ON-WIRE + VERIFIED in the Python edition)
    CHECK_EQ(unit.unit_baud_rate().value(), 115200);
    CHECK(contains(unit.events(), "baud change to 115200"));
    CHECK(exchange(unit, "03 90 00 00 02").empty());  // the PC is still at 230400: not understood
    CHECK(contains(unit.events(), "frame at 230400 baud not understood (EC at 115200)"));
    unit.set_baud_rate(115200);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    // code 0x06 (38400) is ignored, as the EC-R6S rod ignored it sent direct
    CHECK(exchange(unit, "45 00 04 00 06 00 05", 0).empty());
    CHECK_EQ(unit.unit_baud_rate().value(), 115200);
    CHECK(contains(unit.events(), "baud code 0x06 (38400) ignored"));
    SimulatedUnitOptions honest = o;
    honest.ignored_baud_codes = {};
    SimulatedUnit other(honest);
    exchange(other, "45 00 04 00 06 00 05", 0);
    CHECK_EQ(other.unit_baud_rate().value(), 38400);
    // a unicast baud change is answered at the old rate, then the unit moves
    SimulatedUnit direct(honest);
    CHECK_EQ(iai(direct, 0x04, 0x13), 0);
    CHECK_EQ(direct.unit_baud_rate().value(), 57600);
    // a unit without a modelled line ignores the rate but still accepts the command
    SimulatedUnit unmodelled;
    CHECK_EQ(iai(unmodelled, 0x04, 0x14), 0);
    CHECK(!unmodelled.unit_baud_rate().has_value());
}

TEST(sim_baud_modelling_and_break) {
    SimulatedUnitOptions o;
    o.baud_rate = 115200;
    o.unit_baud_rate = 38400;  // the unit is at a different rate than the PC side
    SimulatedUnit unit(o);
    CHECK(exchange(unit, "03 90 00 00 02").empty());  // silent: the PC is at 115200, the unit at 38400
    unit.set_baud_rate(38400);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    // a BREAK of 150 ms or more drops the unit to 9600 (DOC); a shorter one does nothing
    unit.send_break(0.1);
    CHECK_EQ(unit.unit_baud_rate().value(), 38400);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    unit.send_break(0.2);
    CHECK_EQ(unit.unit_baud_rate().value(), 9600);
    CHECK(exchange(unit, "03 90 00 00 02").empty());
    unit.set_baud_rate(9600);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    CHECK(contains(unit.events(), "EC at 9600 baud (break)"));
    // a power cycle brings it back to the boot rate (38400, VERIFIED in the Python edition, hardware test 17)
    unit.power_cycle();
    CHECK_EQ(unit.unit_baud_rate().value(), 38400);
    unit.set_baud_rate(38400);
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    // set_baud_rate clears unread bytes
    const Bytes request = frame(1, "03 90 00 00 02");
    unit.write(request.data(), request.size());
    unit.set_baud_rate(38400);
    CHECK(read_bytes_all(unit).empty());
    // a break on an unmodelled unit is only logged
    SimulatedUnit plain;
    plain.send_break(0.3);
    CHECK(!plain.unit_baud_rate().has_value());
    CHECK(contains(plain.events(), "break 0.300 s"));
}

TEST(sim_software_reset_baud_options) {
    Rig rig([] {
        SimulatedUnitOptions o;
        o.profile = SimProfile::Rod;
        o.baud_rate = 115200;
        o.reset_baud_rate = 38400;
        return o;
    }());
    iai(rig.u(), 0x09, 0);
    CHECK_EQ(iai(rig.u(), 0x05, 1), 0);
    CHECK_EQ(rig->unit_baud_rate().value(), 38400);  // the rate after a software reset (hardware: unknown)
    Rig pcon([] {
        SimulatedUnitOptions o;
        o.profile = SimProfile::Pcon;
        o.baud_rate = 115200;
        o.reset_baud_rate = 38400;
        return o;
    }());
    iai(pcon.u(), 0x09, 0);
    iai(pcon.u(), 0x05, 1);
    CHECK_EQ(pcon->unit_baud_rate().value(), 115200);  // the PCON keeps its baud rate at a reset (VERIFIED in Python)
}

// ------------------------------------------------------------------------------------------ parameters

TEST(sim_parameter_ram_then_commit_then_power_cycle) {
    SimulatedUnitOptions o;
    o.parw_busy_s = 0.2;  // real seconds: long enough that a slow machine still sees the busy window
    SimulatedUnit unit(o);
    const int address = 0x0811;  // a class-8 parameter word
    const std::uint16_t original = read_regs(unit, address, 1)[0];
    CHECK(unit.parameter_ram().empty());
    CHECK(unit.parameter_nv().empty());
    // writes land in RAM: they read back at once, but a restart loses them (hardware tests N2 / N3)
    CHECK_EQ(write_reg(unit, address, 0x0044), 0);
    CHECK_EQ(read_regs(unit, address, 1)[0], 0x0044);
    CHECK_EQ(unit.parameter_ram_writes(), 1);
    CHECK_EQ(unit.parameter_ram().at(address), 0x0044);
    CHECK(unit.parameter_nv().empty());
    // the PARW coil starts a commit: DRGE b7 reads 1 for parw_busy_s, then it completes
    CHECK_EQ(coil(unit, reg::kCoilParw, true), 0);
    CHECK(unit.parw_busy());
    CHECK(read_regs(unit, reg::kDrge, 1)[0] & reg::kDrgeParw);
    CHECK(read_regs(unit, reg::kStat, 2)[0] & (reg::kStatAeep >> 16));  // STAT b16 AEEP (high word) while it runs
    CHECK(!unit.coil_state(reg::kCoilParw));                            // self-clearing
    CHECK_EQ(unit.parameter_commits(), 0);
    sleep_at_least(0.25);
    CHECK(!unit.parw_busy());
    CHECK_EQ(read_regs(unit, reg::kDrge, 1)[0] & reg::kDrgeParw, 0);
    CHECK_EQ(unit.parameter_commits(), 1);
    CHECK_EQ(unit.parameter_nv().at(address), 0x0044);
    CHECK(unit.parameter_ram().empty());
    CHECK_EQ(read_regs(unit, address, 1)[0], 0x0044);
    // a later uncommitted write is lost by a power cycle, the committed value survives
    CHECK_EQ(write_reg(unit, address, 0x0055), 0);
    CHECK_EQ(read_regs(unit, address, 1)[0], 0x0055);
    unit.power_cycle();
    CHECK_EQ(read_regs(unit, address, 1)[0], 0x0044);
    CHECK(unit.parameter_ram().empty());
    CHECK(original != 0x0044);
}

TEST(sim_parameter_commit_edge_cases) {
    SimulatedUnitOptions o;
    o.parw_busy_s = 0.0;  // commit at once: DRGE b7 is never seen
    SimulatedUnit instant(o);
    write_reg(instant, 0x0811, 0x0021);
    coil(instant, reg::kCoilParw, true);
    CHECK(!instant.parw_busy());
    CHECK_EQ(instant.parameter_commits(), 1);
    CHECK_EQ(instant.parameter_nv().at(0x0811), 0x0021);
    CHECK_EQ(read_regs(instant, reg::kDrge, 1)[0] & reg::kDrgeParw, 0);
    // a word rewritten during a running commit stays RAM only
    SimulatedUnitOptions slow;
    slow.parw_busy_s = 0.2;
    SimulatedUnit unit(slow);
    write_reg(unit, 0x0811, 0x0031);
    coil(unit, reg::kCoilParw, true);
    write_reg(unit, 0x0811, 0x0032);
    sleep_at_least(0.25);
    CHECK_EQ(unit.parameter_nv().at(0x0811), 0x0031);
    CHECK_EQ(unit.parameter_ram().at(0x0811), 0x0032);
    CHECK_EQ(read_regs(unit, 0x0811, 1)[0], 0x0032);
    // a restart inside the commit window loses the commit (worst case; unknown on the EC)
    SimulatedUnit cut(slow);
    write_reg(cut, 0x0811, 0x0041);
    coil(cut, reg::kCoilParw, true);
    cut.power_cycle();
    CHECK(!cut.parw_busy());
    CHECK(cut.parameter_nv().empty());
    CHECK_EQ(cut.parameter_commits(), 0);
    CHECK(contains(cut.events(), "parameter commit (PARW) interrupted by a restart: lost"));
    // PARW is ignored while PMSS = 0 (like other coils)
    SimulatedUnit manual(slow);
    write_reg(manual, 0x0811, 0x0051);
    coil(manual, reg::kCoilPmsl, false);
    coil(manual, reg::kCoilParw, true);
    CHECK(!manual.parw_busy());
    // DSSE b7 (PARW idle) drops while the commit runs only with parw_dsse_busy
    SimulatedUnitOptions dsse;
    dsse.parw_busy_s = 0.2;
    dsse.parw_dsse_busy = true;
    SimulatedUnit watched(dsse);
    write_reg(watched, 0x0811, 0x0061);
    CHECK(read_regs(watched, reg::kDsse, 1)[0] & reg::kDsseParw);
    coil(watched, reg::kCoilParw, true);
    CHECK_EQ(read_regs(watched, reg::kDsse, 1)[0] & reg::kDsseParw, 0);
}

TEST(sim_parameter_writes_outside_the_parameter_classes_are_refused) {
    SimulatedUnit unit;
    CHECK_EQ(write_reg(unit, 0x0500, 1), 2);        // alarm history
    CHECK_EQ(write_reg(unit, 0x0600, 1), 2);        // identification
    CHECK_EQ(write_regs(unit, 0x0300, {1, 2}), 2);  // controller monitor
    CHECK_EQ(write_regs(unit, 0x7000, {1}), 2);
    CHECK_EQ(write_regs(unit, 0x07FF, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20}), 2);  // spans a block end
    CHECK_EQ(unit.parameter_ram_writes(), 0);
    // class 7 / 9 / 0C words are accepted too
    CHECK_EQ(write_reg(unit, 0x0701, 0x3AB6), 0);
    CHECK_EQ(write_reg(unit, 0x0901, 0x0001), 0);
    CHECK_EQ(write_reg(unit, 0x0C01, 0x0001), 0);
    CHECK_EQ(unit.parameter_ram_writes(), 3);
}

TEST(sim_committed_home_parameter_change_raises_the_abs_error_after_a_reset) {
    // Gripper 2026-10-27 (VERIFIED in the Python edition): a committed P4 change + software reset -> 0x0EE, HEND 0, servo
    // off; an alarm reset switches the servo back on, a home return clears it
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.parw_busy_s = 0.0;
    o.settle_speed_mm_s = 0.0;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    const auto p4 = read_item(unit, 9, 21);
    REQUIRE(p4.has_value());
    // find the address of item 9.21 the way the driver does and change its low word
    const int count = read_regs(unit, 0, 8)[2];
    const Words records = read_block(unit, 8, 8 * count);
    int address = -1;
    for (int i = 0; i < count; ++i) {
        if (records[8 * i] != 9) continue;
        const Words ids = read_block(unit, records[8 * i + 5], 2 * records[8 * i + 2]);
        for (int k = 0; k < records[8 * i + 2]; ++k)
            if (ids[2 * k] == 21) address = records[8 * i + 4] + 2 * k;
    }
    REQUIRE(address > 0);
    CHECK_EQ(write_reg(unit, address + 1, static_cast<int>((*p4 + 5) & 0xFFFF)), 0);
    CHECK_EQ(coil(unit, reg::kCoilParw, true), 0);
    CHECK_EQ(unit.parameter_commits(), 1);
    iai(unit, 0x09, 0);
    CHECK_EQ(iai(unit, 0x05, 1), 0);
    rig.clock.advance(1.5);
    CHECK_EQ(unit.alarm_code(), kSimAlarmAbsError);
    CHECK(!unit.homed());
    CHECK(!unit.servo_on());
    CHECK(read_regs(unit, reg::kDss1, 1)[0] & reg::kDss1Almh);
    coil(unit, reg::kCoilPmsl, true);
    coil(unit, reg::kCoilAlrs, true);
    coil(unit, reg::kCoilAlrs, false);
    CHECK(unit.servo_on());  // the servo came back by itself after the ALRS pulse
    CHECK(!unit.homed());    // HEND stays 0
    iai(unit, reg::kIaiCmdHome, 7);
    rig.clock.advance(10.0);
    CHECK(unit.homed());
}

// ------------------------------------------------------------------------------------------ position table

TEST(sim_position_table_write_counts_nonvolatile_and_recomputes_the_checksum) {
    SimulatedUnit unit;
    CHECK_EQ(unit.nonvolatile_writes(), 0);
    CHECK_EQ(write_regs(unit, 0x1030, {0, 1234}), 0);
    CHECK_EQ(unit.nonvolatile_writes(), 1);
    const Words row = unit.position_row(3);
    CHECK_EQ(hexs(head(row, 2)), "0000 04D2");
    // word +0xF = the checksum of the other fifteen (an assumption of the simulator: whether the unit does this is
    // unknown); the byte sum of the 32-byte row is 0 mod 256
    unsigned sum = 0;
    for (std::uint16_t w : row) sum += (w >> 8) + (w & 0xFF);
    CHECK_EQ(sum & 0xFF, 0u);
    CHECK_EQ(row[reg::kPosChecksum], reg::position_checksum(Words(row.begin(), row.begin() + 15)));
    // every write call counts once; a full 16-word row with its own checksum counts once too
    Words full = {0, 5000, 0, 10, 0, 0x2710, 0, 0, 0, 0, 30, 30, 0, 0, 0x0000, 0};
    full[15] = reg::position_checksum(full);
    CHECK_EQ(write_regs(unit, 0x1000 + 16 * 5, full), 0);
    CHECK_EQ(unit.nonvolatile_writes(), 2);
    CHECK_EQ(hexs(unit.position_row(5)), hexs(full));
    CHECK_EQ(write_reg(unit, 0x1000 + 16 * 6 + 1, 700), 0);
    CHECK_EQ(unit.nonvolatile_writes(), 3);
    CHECK_EQ(read_regs(unit, 0x1000 + 16 * 6, 2)[1], 700);
    // a write across two rows recomputes both checksums
    write_regs(unit, 0x1000 + 16 * 7 + 14, {0x0010, 0x0000, 0x0000, 0x0099});
    for (int n : {7, 8}) {
        const Words r = unit.position_row(n);
        CHECK_EQ(r[15], reg::position_checksum(Words(r.begin(), r.begin() + 15)));
    }
    // reads never count
    read_regs(unit, 0x1000, 16);
    CHECK_EQ(unit.nonvolatile_writes(), 4);
}

TEST(sim_position_table_checksum_recompute_can_be_switched_off) {
    SimulatedUnitOptions o;
    o.recompute_position_checksum = false;
    SimulatedUnit unit(o);
    const std::uint16_t before = unit.position_row(3)[15];
    write_regs(unit, 0x1030, {0, 4321});
    CHECK_EQ(unit.position_row(3)[15], before);  // a driver must write the checksum itself
    CHECK_EQ(unit.nonvolatile_writes(), 1);
}

TEST(sim_position_table_direct_access_and_pcon_blank_rows) {
    SimulatedUnit unit;
    Words row(16, 0);
    row[1] = 321;
    unit.set_position_row(9, row);
    CHECK_EQ(unit.nonvolatile_writes(), 0);  // a test hook, not a write cycle
    CHECK_EQ(read_regs(unit, 0x1000 + 16 * 9, 2)[1], 321);
    CHECK_THROWS(unit.position_row(768), std::out_of_range);
    CHECK_THROWS(unit.position_row(-1), std::out_of_range);
    CHECK_THROWS(unit.set_position_row(0, Words(3)), std::invalid_argument);
    CHECK_EQ(hexs(unit.position_row(100)), hexs(Words(16, 0)));  // slider: rows never written read 0
    SimulatedUnit pcon(SimProfile::Pcon);
    CHECK_EQ(hexs(unit.position_row(200)), hexs(Words(16, 0)));
    CHECK_EQ(pcon.position_row(100)[3], 0x000A);  // the PCON's blank row: band 0.10, accel 0.30 G
    CHECK_EQ(pcon.position_row(100)[10], 0x001E);
    CHECK_EQ(pcon.position_row(100)[12], 0x00B2);
    // the rows 256.. (an EC's simple-data store) are served from the 0x2000 page
    CHECK_EQ(read_exception(unit, 0x2000, 16), 0);
    CHECK_EQ(read_exception(unit, 0x4000, 1), 2);  // past the 768 rows
}

// ------------------------------------------------------------------------------------------ alarms, power cycle

TEST(sim_alarm_history_is_seeded_and_shifts) {
    SimulatedUnit unit;
    CHECK_EQ(hexs(read_regs(unit, 0x0500, 12)), "0000 FFFF 0000 0FFF FFFF FFFF 0003 FFFF 0000 00EE 000E 84DB");
    unit.trigger_alarm(0x0E0);
    CHECK_EQ(read_regs(unit, 0x0500, 6)[3], 0x0E0);
    CHECK_EQ(read_regs(unit, 0x0506, 6)[3], 0x0FFF);  // shifted down
    CHECK_EQ(hexs(read_regs(unit, 0x05BA, 6)), "0000 FFFF 0000 0000 FFFF FFFF");  // the last entry is empty
    CHECK_EQ(read_exception(unit, 0x05C0, 1), 2);  // the block ends after the 32 entries
}

TEST(sim_alarm_levels_message_keeps_the_servo_major_switches_it_off) {
    SimulatedUnit unit;
    unit.trigger_alarm(0x04E);  // "total moving count exceeded" (a maintenance warning): message level
    CHECK(unit.servo_on());
    CHECK_EQ(unit.alarm_code(), 0x04E);
    std::uint16_t dss1 = read_regs(unit, reg::kDss1, 1)[0];
    CHECK((dss1 & reg::kDss1Alml) != 0 && (dss1 & reg::kDss1Almh) == 0);
    CHECK_EQ(read_regs(unit, reg::kAlmc, 1)[0], 0x04E);
    CHECK_EQ(read_regs(unit, reg::kSsse, 1)[0] & reg::kSsseAlmc, 0);
    unit.trigger_alarm(0x0DA);  // operation cancelled: ALMH, servo off
    CHECK(!unit.servo_on());
    dss1 = read_regs(unit, reg::kDss1, 1)[0];
    CHECK((dss1 & reg::kDss1Almh) != 0 && (dss1 & reg::kDss1Alml) == 0);
    CHECK_EQ(read_regs(unit, reg::kSsse, 1)[0] & reg::kSsseAlmc, 0);  // not a cold-start alarm
    unit.trigger_alarm(0x0C8);  // a cold-start level code: SSSE b11 ALMC
    CHECK(read_regs(unit, reg::kSsse, 1)[0] & reg::kSsseAlmc);
    // DOPM *ALM follows the alarm (3-position layout b3)
    CHECK_EQ(read_regs(unit, reg::kDopm, 1)[0] & 0x08, 0);
    // the alarm is mirrored in the controller monitor class (item 3.10)
    CHECK_EQ(read_item(unit, 3, 10).value(), 0x0C8u);
}

TEST(sim_power_cycle_resets_the_volatile_state) {
    SimulatedUnit unit;
    coil(unit, reg::kCoilSfty, true);
    coil(unit, reg::kCoilStp, true);
    coil(unit, reg::kCoilStop, true);
    unit.trigger_alarm(0x0E0);
    CHECK(unit.stop_held());
    unit.power_cycle();
    CHECK(!unit.pmsl());  // PMSS 0 after power-up (hardware test 17)
    CHECK(!unit.safety_speed());
    CHECK(!unit.stop_held());
    CHECK(!unit.coil_state(reg::kCoilStp));
    CHECK_EQ(unit.alarm_code(), 0);
    CHECK(unit.servo_on());
    CHECK(unit.homed());  // an absolute unit stays homed
    CHECK_EQ(read_regs(unit, reg::kDss1, 1)[0] & (reg::kDss1Stp | reg::kDss1Sfty), 0);
    CHECK_EQ(read_regs(unit, 0x0500, 6)[3], 0x0FFF);  // the power-up marker
    CHECK_EQ(read_regs(unit, 0x0506, 6)[3], 0x0E0);   // the earlier alarm is one entry down
    CHECK_NEAR(unit.position_mm(), 73.95, 1e-9);      // the position is kept
    CHECK(contains(unit.events(), "power cycle"));
    // an incremental unit comes back without HEND (the safe assumption for a driver)
    SimulatedUnit rod(SimProfile::Rod);
    CHECK(rod.homed());
    rod.power_cycle();
    CHECK(!rod.homed());
    CHECK_EQ(read_regs(rod, reg::kDss1, 1)[0] & reg::kDss1Hend, 0);
    CHECK(contains(rod.events(), "HEND lost at power-up"));
}

TEST(sim_power_cycle_uptime_restarts_and_the_monitor_counts_moves) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    rig.clock.advance(5.0);
    const Words stim = read_regs(unit, reg::kStim, 2);
    CHECK_EQ(reg::from_u32_words(stim[0], stim[1]), 5000u);  // ms since power on
    unit.power_cycle();
    const Words again = read_regs(unit, reg::kStim, 2);
    CHECK_EQ(reg::from_u32_words(again[0], again[1]), 0u);
    // TLMC (maintenance item 1) counts the moves the simulator starts
    const std::uint32_t before = read_item(unit, 0x0200, 1).value();
    write_regs(unit, 0x9900, move_block(80.0, 0.1, 50.0, 0.3));
    CHECK_EQ(read_item(unit, 0x0200, 1).value(), before + 1);
}

TEST(sim_pcon_raises_its_message_alarm_again_after_a_reset) {
    SimulatedUnit pcon(SimProfile::Pcon);
    CHECK_EQ(pcon.alarm_code(), reg::kAlarmRtcStop);
    CHECK(pcon.servo_on());
    CHECK(read_regs(pcon, reg::kDss1, 1)[0] & reg::kDss1Alml);
    CHECK_EQ(coil(pcon, reg::kCoilAlrs, true), 0);
    CHECK_EQ(pcon.alarm_code(), reg::kAlarmRtcStop);  // re-raised by itself while the clock is unset
    CHECK(pcon.servo_on());
    pcon.power_cycle();
    CHECK_EQ(pcon.alarm_code(), reg::kAlarmRtcStop);
    CHECK_EQ(hexs(Words{read_regs(pcon, 0x0500, 6)[3]}), "0069");  // the newest history entry
}

TEST(sim_drop_reply_after_retarget_loses_the_next_answer) {
    SimulatedUnitOptions o;
    o.drop_reply_after_retarget = true;
    o.settle_speed_mm_s = 0.0;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(write_regs(unit, 0x9900, move_block(120.0, 0.1, 20.0, 0.1)), 0);  // starts the move: answered, nothing dropped
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);
    rig.clock.advance(0.5);
    CHECK_EQ(write_regs(unit, 0x9900, move_block(100.0, 0.1, 20.0, 0.1)), 0);  // a retarget mid-move: answered...
    CHECK_EQ(read_exception(unit, 0x9000, 2), -1);                              // ...but the next request is not (seen once)
    CHECK(unit.moving());
    CHECK_EQ(read_regs(unit, 0x9000, 2).size(), 2u);                            // the one after is
    rig.clock.advance(10.0);
    CHECK_NEAR(unit.position_mm(), 100.0, 1e-6);
}

TEST(sim_unit_is_thread_safe_under_concurrent_use) {
    SimulatedUnitOptions o;
    o.time_scale = 50.0;
    SimulatedUnit unit(o);
    std::atomic<int> failures{0};
    auto reader = [&unit, &failures] {
        for (int i = 0; i < 300; ++i) {
            try {
                const Words w = read_regs(unit, 0x9000, 8);
                if (w.size() != 8) ++failures;
                unit.position_mm();
                unit.moving();
            } catch (...) {
                ++failures;
            }
        }
    };
    auto writer = [&unit, &failures] {
        for (int i = 0; i < 100; ++i) {
            try {
                write_regs(unit, 0x9900, move_block(60.0 + (i % 2) * 40.0, 0.1, 400.0, 0.5));
                coil(unit, reg::kCoilStp, i % 3 == 0);
            } catch (...) {
                ++failures;
            }
        }
    };
    std::thread a(reader), b(reader), c(writer), d(reader);
    a.join();
    b.join();
    c.join();
    d.join();
    CHECK_EQ(failures.load(), 0);
}
