// Bus unit: IAI function 0x48 (ModbusRtu::iai_command2) and the command senders of the strategies (0x48 servo /
// home / alarm reset, the DRG1 register path, the teach coils). Port of the Python edition's 0x48 frame tests
// (tests/test_modbus.py IaiCommand2 parts). Scripted frames only: 0x48 is UNVERIFIED on any unit, frame layout included.
#include <memory>
#include <string>
#include <vector>

#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

/// A 0x48 answer: [slave] 48 [rejection u16] [4 x u32 status].
Bytes reply2(int slave, int rejection, std::uint32_t status1 = 0) {
    Bytes pdu = {0x48, static_cast<std::uint8_t>(rejection >> 8), static_cast<std::uint8_t>(rejection & 0xFF)};
    for (int shift = 24; shift >= 0; shift -= 8) pdu.push_back(static_cast<std::uint8_t>((status1 >> shift) & 0xFF));
    pdu.resize(19, 0x00);
    return build_frame(slave, pdu);
}

struct Rig {
    std::shared_ptr<FakeTransport> fake = std::make_shared<FakeTransport>();
    ModbusRtu bus;
    Rig() : bus(fake, [] {
                BusOptions o;
                o.retries = 0;
                o.min_gap_s = 0.0;
                return o;
            }()) {}
};

}  // namespace

TEST(bus_iai2_frame_layout_and_reply) {
    Rig r;
    r.fake->enqueue(reply2(1, 0, 0x01020304u));
    const IaiResponse2 res = r.bus.iai_command2(1, reg::kIai2CmdServo, {1u});
    CHECK_EQ(res.rejection, 0);
    CHECK_EQ(res.status[0], 0x01020304u);
    const auto writes = r.fake->writes();
    REQUIRE(writes.size() == 1);
    const Bytes& f = writes[0];
    CHECK_EQ(f.size(), 23u);  // 21 bytes + CRC
    CHECK_EQ(static_cast<int>(f[0]), 1);
    CHECK_EQ(static_cast<int>(f[1]), 0x48);
    CHECK_EQ(static_cast<int>(f[2]), 0x00);  // group
    CHECK_EQ(static_cast<int>(f[3]), 0x00);  // command hi
    CHECK_EQ(static_cast<int>(f[4]), 0x30);  // command lo
    CHECK_EQ(static_cast<int>(f[8]), 0x01);  // op1 = 1 (u32, big-endian)
    for (std::size_t i = 9; i < 21; ++i) CHECK_EQ(static_cast<int>(f[i]), 0);
    CHECK(check_crc(f));
    CHECK_EQ(kFcIaiCommand2, 0x48);
}

TEST(bus_iai2_rejection_raises_and_is_named) {
    Rig r;
    r.fake->enqueue(reply2(1, 0x1006));
    bool threw = false;
    try {
        r.bus.iai_command2(1, reg::kIai2CmdHome);
    } catch (const IaiCommandRejected& e) {
        threw = true;
        CHECK_EQ(e.function(), 0x48);
        CHECK_EQ(e.rejection(), 0x1006);
        CHECK_EQ(e.reason(), "motion command during homing");
    }
    CHECK(threw);
    CHECK_EQ(iai2_rejection_name(0x1001), "1st operand error");
    CHECK_EQ(iai2_rejection_name(0x1FFF), "unknown rejection code");
}

TEST(bus_iai2_safe_list_and_never_send_refuse_before_sending) {
    Rig r;
    CHECK_THROWS(r.bus.iai_command2(1, 0x0010), UnsafeCommandError);  // non-volatile parameter write: never sent
    IaiOptions unsafe;
    unsafe.allow_unsafe = true;
    CHECK_THROWS(r.bus.iai_command2(1, 0x0010, {}, unsafe), UnsafeCommandError);  // allow_unsafe is not enough
    CHECK_THROWS(r.bus.iai_command2(1, 0x0050), UnsafeCommandError);              // not on the safe list
    CHECK_THROWS(r.bus.iai_command2(1, reg::kIai2CmdServo, {1u}, {}, 2), UnsafeCommandError);  // group 2
    CHECK_THROWS(r.bus.iai_command2(1, reg::kIai2CmdServo, {1u, 2u, 3u, 4u, 5u}), std::invalid_argument);
    CHECK(r.fake->writes().empty());
    CHECK_EQ(iai2_safe_command(0x34), "decel stop");
    CHECK(iai2_never_send(0x0020).find("baud") != std::string::npos);
}

TEST(bus_iai2_is_refused_on_an_rcon_line) {
    Rig r;
    r.bus.set_line(std::make_shared<Line>(LineKind::Rcon, nullptr, false, 0.01));
    CHECK_THROWS(r.bus.iai_command2(1, reg::kIai2CmdServo, {1u}), LineRuleError);
    CHECK(r.fake->writes().empty());
}

TEST(bus_iai2_strategy_senders) {
    Rig r;
    for (int i = 0; i < 4; ++i) r.fake->enqueue(reply2(1, 0));
    send_servo(r.bus, 1, true, CommandMethod::Iai2);
    send_servo(r.bus, 1, false, CommandMethod::Iai2);
    start_home(r.bus, 1, CommandMethod::Iai2);
    finish_home(r.bus, 1, CommandMethod::Iai2);  // nothing for the 0x48 home
    send_alarm_reset(r.bus, 1, CommandMethod::Iai2);
    std::vector<int> codes;
    for (const Bytes& f : r.fake->writes()) codes.push_back((f[3] << 8) | f[4]);
    CHECK(codes == (std::vector<int>{0x30, 0x30, 0x31, 0x23}));
}

TEST(bus_register_senders_keep_son_and_sfty) {
    Rig r;
    std::uint16_t drg1 = 0x5000;  // SON + SFTY
    std::vector<std::uint16_t> written;
    r.fake->set_responder([&](const Bytes& f) -> std::optional<Bytes> {
        if (f[1] == 0x03) return build_frame(1, {0x03, 0x02, static_cast<std::uint8_t>(drg1 >> 8),
                                                 static_cast<std::uint8_t>(drg1 & 0xFF)});
        if (f[1] == 0x06) {
            drg1 = static_cast<std::uint16_t>((f[4] << 8) | f[5]);
            written.push_back(drg1);
            return Bytes(f.begin(), f.end());
        }
        return std::nullopt;
    });
    RegisterCommands regs;
    regs.pause(r.bus, 1, true);
    regs.cancel_paused(r.bus, 1);
    regs.start_home(r.bus, 1);
    regs.finish_home(r.bus, 1);
    regs.reset_alarm(r.bus, 1);
    regs.safety_speed(r.bus, 1, false);
    regs.servo(r.bus, 1, false);
    CHECK(written == (std::vector<std::uint16_t>{0x5020, 0x5120, 0x5020, 0x5000, 0x5010, 0x5000, 0x5100, 0x5000,
                                                  0x1000, 0x0000}));
    // EMG and BKRL are never written
    drg1 = 0x1000;
    written.clear();
    regs.write(r.bus, 1, 0xFFFF);
    REQUIRE(written.size() == 1);
    CHECK_EQ(written[0] & (reg::kDrg1Emg | reg::kDrg1Bkrl), 0);
    // a stateless sender reads DRG1 first too (servo through the free function)
    drg1 = 0x4000;
    written.clear();
    send_servo(r.bus, 1, true, CommandMethod::Register);
    CHECK(written == (std::vector<std::uint16_t>{0x5000}));
}

TEST(bus_teach_coil_senders) {
    Rig r;
    r.fake->set_responder([](const Bytes& f) -> std::optional<Bytes> { return Bytes(f.begin(), f.end()); });
    send_teach_mode(r.bus, 1, true);
    send_position_select(r.bus, 1, 5);
    send_teach(r.bus, 1, true);
    const auto w = r.fake->writes();
    REQUIRE(w.size() == 3);
    CHECK(w[0] == build_frame(1, {0x05, 0x04, 0x14, 0xFF, 0x00}));
    CHECK(w[1] == build_frame(1, {0x06, 0x0D, 0x03, 0x00, 0x05}));
    CHECK(w[2] == build_frame(1, {0x05, 0x04, 0x15, 0xFF, 0x00}));
    CHECK_THROWS(send_position_select(r.bus, 1, -1), std::invalid_argument);
    CHECK_EQ(std::string(to_string(TeachMethod::Coil)), "coil");
}
