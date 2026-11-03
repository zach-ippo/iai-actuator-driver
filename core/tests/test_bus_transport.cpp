// Bus unit: FakeTransport behaviour, hex_frame, the IAI command tables and the baud tables, and one call of
// list_ports() (it must not throw; its contents depend on the PC). SerialTransport is never constructed here.
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "iai_actuator_driver/version.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {
using Bytes = std::vector<std::uint8_t>;
}  // namespace

TEST(bus_fake_script_then_responder) {
    int calls = 0;
    FakeTransport t([&calls](const Bytes& f) -> std::optional<Bytes> {
        ++calls;
        return Bytes{0xAA, f[0]};
    });
    t.enqueue(Bytes{0x01});
    t.enqueue(std::nullopt);
    const Bytes a{0x10}, b{0x20}, c{0x30};
    std::uint8_t buf[8];
    t.write(a);
    CHECK_EQ(t.read(buf, 8), 1u);
    CHECK_EQ(buf[0], 0x01);
    t.write(b);
    CHECK_EQ(t.read(buf, 8), 0u);  // scripted silence
    t.write(c);
    CHECK_EQ(t.read(buf, 1), 1u);  // partial reads
    CHECK_EQ(buf[0], 0xAA);
    CHECK_EQ(t.read(buf, 8), 1u);
    CHECK_EQ(buf[0], 0x30);
    CHECK_EQ(calls, 1);
    CHECK_EQ(t.writes().size(), 3u);
    CHECK_EQ(t.write_times().size(), 3u);
    t.inject({1, 2, 3});
    t.reset_input();
    CHECK_EQ(t.read(buf, 8), 0u);
    t.clear_writes();
    CHECK(t.writes().empty());
    CHECK_EQ(t.name(), "FAKE");
    CHECK(!t.has_real_timeout());
}

TEST(bus_fake_baud_rate_and_answer_rate) {
    FakeTransport none;
    CHECK_EQ(none.baud_rate(), 0);
    CHECK_THROWS(none.set_baud_rate(9600), std::logic_error);
    FakeTransport t([](const Bytes&) -> std::optional<Bytes> { return Bytes{0x55}; }, 38400, "F2");
    t.set_answer_at_baud_rate(115200);
    std::uint8_t buf[4];
    t.enqueue(Bytes{0x01});
    t.write(Bytes{0x00});
    CHECK_EQ(t.read(buf, 4), 0u);  // wrong rate: silent, the script entry is kept
    t.set_baud_rate(115200);
    t.write(Bytes{0x00});
    CHECK_EQ(t.read(buf, 4), 1u);
    CHECK_EQ(buf[0], 0x01);
    CHECK(t.baud_rates_set() == std::vector<int>{115200});
    t.send_break(0.2);
    CHECK(t.breaks() == std::vector<double>{0.2});
    t.set_read_timeout(0.05);
    CHECK_NEAR(t.read_timeout(), 0.05, 1e-12);
}

TEST(bus_fake_fail_close_reopen) {
    FakeTransport t;
    CHECK(t.is_open());
    t.fail();
    CHECK(!t.is_open());
    CHECK_THROWS(t.write(Bytes{1}), TransportError);
    std::uint8_t buf[2];
    CHECK_THROWS(t.read(buf, 2), TransportError);
    CHECK_THROWS(t.reset_input(), TransportError);
    t.reopen();
    t.write(Bytes{1});
    CHECK_EQ(t.writes().size(), 1u);
    t.close();
    t.close();
    CHECK(!t.is_open());
    CHECK_THROWS(t.write(Bytes{2}), TransportError);
    t.reopen();
    CHECK(t.is_open());
}

TEST(bus_hex_frame_and_crc_helpers) {
    CHECK_EQ(hex_frame({}), "");
    CHECK_EQ(hex_frame({0x01, 0xAB, 0x0F}), "01 AB 0F");
    Bytes frame{0x01, 0x03, 0x90, 0x00, 0x00, 0x16};
    append_crc(frame);
    CHECK_EQ(hex_frame(frame), "01 03 90 00 00 16 E9 04");
}

TEST(bus_iai_tables) {
    CHECK_EQ(iai_rejection_name(0x11), "1st operand error");
    CHECK_EQ(iai_rejection_name(0x3A), "command prohibited in monitor (PIO) mode");
    CHECK_EQ(iai_rejection_name(0x05), "unknown rejection code");
    CHECK(iai_rejection_hint(0x11, reg::kIaiCmdHome).find("ORG (item 9.16)") != std::string::npos);
    CHECK_EQ(iai_rejection_hint(0x11, reg::kIaiCmdServo), "");
    CHECK_EQ(iai_rejection_hint(0x23, reg::kIaiCmdServo), "turn the servo on first");
    for (int command : {0x05, 0x06, 0x07, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E}) CHECK(!iai_safe_command(command).empty());
    for (int command : {0x01, 0x04, 0x08, 0x15, 0x45}) CHECK(iai_safe_command(command).empty());
    CHECK(iai_never_send(0x04).find("baud") != std::string::npos);
    CHECK_EQ(iai_never_send(0x20), "write maintenance information");
    CHECK_EQ(iai_never_send(0x3C), "write maintenance information");
    CHECK_EQ(iai_never_send(0xE8), "factory inspection command");
    CHECK_EQ(iai_never_send(0xF3), "clear position data");
    CHECK_EQ(iai_never_send(0x09), "");
    CHECK_EQ(iai_never_send(0x3D), "");
}

TEST(bus_baud_tables) {
    CHECK_EQ(baud_code(115200).value_or(0), 0x14);
    CHECK_EQ(baud_code(230400).value_or(0), 0x15);
    CHECK(!baud_code(12345).has_value());
    CHECK_EQ(baud_by_code(6).value_or(0), 38400);
    CHECK(baud_rate_verified(115200));
    CHECK(!baud_rate_verified(38400));
    CHECK(baud_rate_unit_dependent(38400));  // taken by the gripper and the PCON-CB, ignored by the EC-R6S rod
    CHECK(baud_code_verified(38400) && baud_code_verified(230400));
    CHECK(!baud_code_verified(57600));
    CHECK(baud_detect_order() == (std::vector<int>{115200, 38400, 9600, 230400, 57600, 19200}));
    CHECK_EQ(std::string(to_string(BaudChangeMethod::Break)), "break");
}

TEST(bus_list_ports_does_not_throw) {
    // Opens nothing; the contents depend on the PC, so only the shape is checked.
    std::vector<SerialPortInfo> ports;
    bool threw = false;
    try {
        ports = list_ports();
    } catch (...) {
        threw = true;
    }
    CHECK(!threw);
    for (const SerialPortInfo& p : ports) CHECK(!p.port.empty());
    CHECK(std::string(kDisclaimer).find("UNVERIFIED") != std::string::npos);
}
