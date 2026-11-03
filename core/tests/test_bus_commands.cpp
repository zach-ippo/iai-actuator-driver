// Bus unit: the exact frames of every command sender (commands.hpp), over a FakeTransport that echoes writes and
// accepts IAI commands. Expected frames as the Python edition sends them (VERIFIED there unless noted in commands.hpp).
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes hx(const std::string& text) { return util::hex_bytes(text); }

/// FC05 / FC06 echo, FC10 echo of address and count, 0x45 accepted (01 45 00 00).
std::optional<Bytes> unit(const Bytes& frame) {
    const int slave = frame[0];
    switch (frame[1]) {
        case 0x05:
        case 0x06: return Bytes(frame.begin(), frame.end());
        case 0x10: return build_frame(slave, Bytes(frame.begin() + 1, frame.begin() + 6));
        case 0x45: return build_frame(slave, hx("45 00 00"));
        default: return std::nullopt;
    }
}

struct Rig {
    std::shared_ptr<FakeTransport> t = std::make_shared<FakeTransport>(unit);
    ModbusRtu bus{t, [] {
                      BusOptions o;
                      o.min_gap_s = 0.0;
                      return o;
                  }()};
    std::vector<std::string> frames() const {
        std::vector<std::string> out;
        for (const Bytes& w : t->writes()) out.push_back(hex_frame(w));
        return out;
    }
};

using Frames = std::vector<std::string>;

}  // namespace

TEST(bus_cmd_servo) {
    Rig coil;
    send_servo(coil.bus, 1, true);
    send_servo(coil.bus, 1, false);
    CHECK(coil.frames() == (Frames{"01 05 04 03 FF 00 7D 0A", "01 05 04 03 00 00 3C FA"}));
    Rig iai;
    send_servo(iai.bus, 1, true, CommandMethod::Iai);
    send_servo(iai.bus, 1, false, CommandMethod::Iai);
    CHECK(iai.frames() == (Frames{"01 45 00 09 00 01 00 00 18 C2", "01 45 00 09 00 00 00 00 49 02"}));
    CHECK_EQ(std::string(to_string(CommandMethod::Iai)), "iai");
}

TEST(bus_cmd_home_rising_edge_and_release) {
    Rig coil;
    start_home(coil.bus, 1);
    CHECK(coil.frames() == (Frames{"01 05 04 0B 00 00 BD 38", "01 05 04 0B FF 00 FC C8"}));  // 0000 then FF00
    coil.t->clear_writes();
    finish_home(coil.bus, 1);
    CHECK(coil.frames() == (Frames{"01 05 04 0B 00 00 BD 38"}));
}

TEST(bus_cmd_home_iai) {
    Rig r;
    start_home(r.bus, 1, CommandMethod::Iai, 0x00010007u);  // op1 = ORG & 0xFFFF
    CHECK(r.frames() == (Frames{"01 45 00 0A 00 07 00 00 BC C3"}));
    finish_home(r.bus, 1, CommandMethod::Iai);  // nothing for the 0x45 home
    CHECK_EQ(r.t->writes().size(), 1u);
    // no usable ORG: refused, nothing sent
    Rig none;
    CHECK_THROWS(start_home(none.bus, 1, CommandMethod::Iai), EcNotReadyError);
    CHECK_THROWS(start_home(none.bus, 1, CommandMethod::Iai, 0x00010000u), EcNotReadyError);
    CHECK(none.t->writes().empty());
    // sent once: a lost reply is not retried
    auto t = std::make_shared<FakeTransport>();
    ModbusRtu bus(t);
    CHECK_THROWS(start_home(bus, 1, CommandMethod::Iai, 7u), ModbusTimeout);
    CHECK_EQ(t->writes().size(), 1u);
}

TEST(bus_cmd_alarm_reset_pulse) {
    Rig coil;
    send_alarm_reset(coil.bus, 1);
    CHECK(coil.frames() == (Frames{"01 05 04 07 FF 00 3C CB", "01 05 04 07 00 00 7D 3B"}));  // FF00 then 0000
    Rig iai;
    send_alarm_reset(iai.bus, 1, CommandMethod::Iai);
    CHECK(iai.frames() == (Frames{"01 45 00 05 00 00 00 00 59 03"}));
}

TEST(bus_cmd_stop_pause_pmsl_safety_commit) {
    Rig r;
    send_stop_coil(r.bus, 1, true);
    send_pause(r.bus, 1, true);
    send_modbus_control(r.bus, 1, true);
    send_safety_speed(r.bus, 1, true);
    send_parameter_commit(r.bus, 1);
    CHECK(r.frames() == (Frames{"01 05 04 2C FF 00 4C C3", "01 05 04 0A FF 00 AD 08", "01 05 04 27 FF 00 3D 01",
                                "01 05 04 01 FF 00 DC CA", "01 05 04 28 FF 00 0D 02"}));
    r.t->clear_writes();
    send_stop_coil(r.bus, 1, false);
    send_pause(r.bus, 1, false);
    const Frames off = r.frames();
    REQUIRE(off.size() == 2);
    CHECK_EQ(off[0].substr(0, 17), "01 05 04 2C 00 00");
    CHECK_EQ(off[1].substr(0, 17), "01 05 04 0A 00 00");
}

TEST(bus_cmd_action_mode) {
    Rig r;
    const IaiResponse result = send_action_mode(r.bus, 1, true, false);
    CHECK_EQ(result.rejection, 0);
    send_action_mode(r.bus, 1, false, true);
    const Frames f = r.frames();
    REQUIRE(f.size() == 2);
    CHECK_EQ(f[0], "01 45 00 0C 00 01 00 00 D4 C2");
    CHECK_EQ(f[1].substr(0, 23), "01 45 00 0C 00 00 00 01");
}

TEST(bus_cmd_software_reset_sent_once) {
    Rig r;
    send_software_reset(r.bus, 1);
    CHECK(r.frames() == (Frames{"01 45 00 05 00 01 00 00 08 C3"}));
    auto t = std::make_shared<FakeTransport>();  // the reply lost while the unit restarts
    ModbusRtu bus(t);
    CHECK_THROWS(send_software_reset(bus, 1), ModbusTimeout);
    CHECK_EQ(t->writes().size(), 1u);
}

TEST(bus_cmd_position_moves) {
    Rig r;
    send_position_move(r.bus, 1, 3);
    CHECK(r.frames() == (Frames{"01 06 98 00 00 03 E6 AB"}));
    r.t->clear_writes();
    // the unit runs (code + 1) / 256 (ON-WIRE on the PCON-CB): code = round(256 x pct / 100) - 1, within 1..255
    send_position_move_override(r.bus, 1, 3, 50.0);   // 128 - 1 = 127
    send_position_move_override(r.bus, 1, 3, 0.1);    // max(1, 0 - 1)
    send_position_move_override(r.bus, 1, 3, 30.0);   // round(76.8) - 1 = 76
    send_position_move_override(r.bus, 1, 3, 100.0);  // 255
    send_position_move_override(r.bus, 1, 3, 5.0);    // round(12.8) - 1 = 12 (13 ran 5.47 % on the PCON-CB)
    const Frames f = r.frames();
    REQUIRE(f.size() == 5);
    CHECK_EQ(f[0].substr(0, 23), "01 45 00 06 00 03 00 7F");
    CHECK_EQ(f[1], "01 45 00 06 00 03 00 01 2C C3");
    CHECK_EQ(f[2].substr(0, 23), "01 45 00 06 00 03 00 4C");
    CHECK_EQ(f[3].substr(0, 23), "01 45 00 06 00 03 00 FF");
    CHECK_EQ(f[4].substr(0, 23), "01 45 00 06 00 03 00 0C");
    CHECK_EQ(speed_override_code(25.0), 63);
    CHECK_EQ(speed_override_code(1.0), 2);  // 3 / 256 = 1.17 %
    CHECK_NEAR(speed_override_pct(63), 25.0, 1e-12);
    CHECK_NEAR(speed_override_pct(1), 0.78125, 1e-12);
    CHECK_THROWS(send_position_move_override(r.bus, 1, 3, std::nan("")), std::invalid_argument);
}

TEST(bus_cmd_direct_move_block) {
    DirectMove move;
    move.pcmd = 5000;
    move.inp = 10;
    move.vcmd = 10000;
    move.acmd = 30;
    CHECK(move.words() == (std::vector<std::uint16_t>{0, 5000, 0, 10, 0, 10000, 30, 0, 0}));
    DirectMove back;
    back.pcmd = -1;
    back.inp = 0x00012345;
    CHECK(back.words() == (std::vector<std::uint16_t>{0xFFFF, 0xFFFF, 0x0001, 0x2345, 0, 0, 0, 0, 0}));
    CHECK(move == move);
    CHECK(!(move == back));
    Rig r;
    write_direct_move(r.bus, 1, move);
    REQUIRE(r.t->writes().size() == 1);
    const Bytes expected = build_frame(1, hx("10 99 00 00 09 12 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 00 00 00 00"));
    CHECK_EQ(hex_frame(r.t->writes()[0]), hex_frame(expected));
    CHECK_EQ(r.t->writes()[0].size(), 7u + 18u + 2u);  // 9 words: never past 0x9908
}
