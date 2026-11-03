// SerialTransport on Linux and macOS (POSIX termios) against a pseudo terminal: a reply that arrives after the request
// is read (the read waits: Linux returns 0, not EAGAIN, from a VMIN = VTIME = 0 read while nothing has arrived yet,
// VERIFIED on a CP210x adapter), nothing sent is a timeout, and the other end closed is a vanished port; the termios
// flags of a character framing after the open and a reopen (8N1 by default, 2 stop bits; 7 data bits and a parity bit
// refused, a pty cannot carry them), and through the C ABI (iad_transport_open_serial_options). POSIX only.
#if !defined(_WIN32) && !defined(_GNU_SOURCE) && !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE 700  // posix_openpt, grantpt, unlockpt, ptsname
#endif
#ifndef _WIN32

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>  // std::this_thread::sleep_for
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

/// A pseudo terminal: the master's fd and the slave's path (master -1 when the system gives none).
struct Pty {
    int master = -1;
    std::string slave;

    Pty() {
        master = posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) return;
        const char* name = (grantpt(master) == 0 && unlockpt(master) == 0) ? ptsname(master) : nullptr;
        if (name == nullptr) {
            ::close(master);
            master = -1;
            return;
        }
        slave = name;
    }
    ~Pty() {
        if (master >= 0) ::close(master);
    }
    Pty(const Pty&) = delete;
    Pty& operator=(const Pty&) = delete;
};

/// The termios flags of the pseudo terminal's slave as the open left them (read through a second descriptor).
bool slave_termios(const std::string& slave, termios& tio) {
    const int fd = ::open(slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;
    const bool ok = tcgetattr(fd, &tio) == 0;
    ::close(fd);
    return ok;
}

}  // namespace

TEST(bus_serial_posix_framing_reaches_the_port_and_survives_a_reopen) {
    // a Linux pseudo terminal keeps the stop bits and the parity check (c_iflag) as set, but forces CS8 and PARENB
    // clear in c_cflag: a framing with 7 data bits or a parity bit is refused there (read back after the open, or
    // EINVAL from tcsetattr), and the mapping of those is checked on detail::apply_framing
    // (test_bus_serial_framing.cpp), the same function the open uses
    struct Case {
        SerialFraming framing;
        bool two_stop;
    };
    const Case kept[] = {
        {SerialFraming{}, false},
        {SerialFraming{8, Parity::None, 2}, true},
    };
    for (const Case& c : kept) {
        Pty pty;
        CHECK(pty.master >= 0);
        if (pty.master < 0) return;
        SerialTransport port(pty.slave, 38400, 0.1, c.framing);
        CHECK(port.framing() == c.framing);
        for (int pass = 0; pass < 2; ++pass) {  // after the open, then after a reopen (which keeps the framing)
            termios tio{};
            REQUIRE(slave_termios(pty.slave, tio));
            CHECK((tio.c_iflag & (INPCK | IGNPAR)) == 0);  // no parity, nothing checked
            CHECK((tio.c_cflag & PARODD) == 0);
            CHECK_EQ((tio.c_cflag & CSTOPB) != 0, c.two_stop);
            CHECK((tio.c_iflag & ISTRIP) == 0);  // a byte is never cut to 7 bits
            CHECK((tio.c_cflag & CLOCAL) != 0);
            if (pass == 0) port.reopen();
        }
        CHECK(port.framing() == c.framing);
    }
#ifdef __linux__
    for (const SerialFraming& framing : {SerialFraming{7, Parity::None, 1}, SerialFraming{7, Parity::Even, 1},
                                         SerialFraming{8, Parity::Odd, 2}, SerialFraming{8, Parity::Even, 1}}) {
        Pty pty;
        if (pty.master < 0) return;
        try {
            SerialTransport port(pty.slave, 38400, 0.1, framing);
            ::ec_test::fail(__FILE__, __LINE__, to_string(framing) + " opened on a pseudo terminal");
        } catch (const TransportError& e) {
            const std::string message = e.what();
            if (message.find("does not take the framing " + to_string(framing)) == std::string::npos)
                ::ec_test::fail(__FILE__, __LINE__, message);
        }
    }
#endif
}

TEST(capi11_serial_options_reach_the_port_through_the_c_abi) {
    // iad_transport_open_serial_options on a pseudo terminal: 0 data bits and 0 stop bits mean 8 and 1, 2 stop bits
    // reach CSTOPB; on Linux 7 data bits or a parity bit is IAD_ERR_TRANSPORT there (a pty forces CS8 and no parity
    // bit), so 0 = 8 data bits and NONE = no parity are seen end to end. Even against odd is checked on the C++ side
    // (to_serial_args in test_capi_serial_options.cpp).
    struct Case {
        int32_t data_bits, parity, stop_bits;
        bool opens, two_stop;
    };
    const Case cases[] = {
        {0, IAD_PARITY_NONE, 0, true, false},
        {8, IAD_PARITY_NONE, 1, true, false},
        {0, IAD_PARITY_NONE, 2, true, true},
#ifdef __linux__
        {7, IAD_PARITY_NONE, 0, false, false},
        {0, IAD_PARITY_EVEN, 0, false, false},
        {0, IAD_PARITY_ODD, 2, false, false},
#endif
    };
    for (const Case& c : cases) {
        Pty pty;
        CHECK(pty.master >= 0);
        if (pty.master < 0) return;
        iad_serial_options o{};
        o.struct_size = sizeof o;
        REQUIRE(iad_serial_options_init(&o) == IAD_OK);
        o.baud_rate = 38400;
        o.data_bits = c.data_bits;
        o.parity = c.parity;
        o.stop_bits = c.stop_bits;
        iad_transport* t = nullptr;
        const iad_result r = iad_transport_open_serial_options(pty.slave.c_str(), &o, &t);
        if (!c.opens) {
            CHECK_EQ(r, IAD_ERR_TRANSPORT);
            CHECK(t == nullptr);
            CHECK(std::string(iad_last_error_message()).find("does not take the framing") != std::string::npos);
            continue;
        }
        REQUIRE(r == IAD_OK);
        termios tio{};
        const bool read = slave_termios(pty.slave, tio);
        CHECK_EQ(iad_transport_close(t), IAD_OK);
        REQUIRE(read);
        CHECK((tio.c_iflag & INPCK) == 0);
        CHECK_EQ((tio.c_cflag & CSTOPB) != 0, c.two_stop);
    }
}

TEST(bus_serial_posix_reply_after_a_delay_is_read) {
    Pty pty;
    CHECK(pty.master >= 0);
    if (pty.master < 0) return;
    SerialTransport port(pty.slave, 38400, 1.0);
    const std::vector<std::uint8_t> request = {0x01, 0x03, 0x90, 0x00, 0x00, 0x02};
    port.write(request.data(), request.size());
    const int master = pty.master;
    ec_test::Thread unit([master] {  // the unit answers about 60 ms later, as a real one does
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        const std::uint8_t reply[5] = {0x01, 0x03, 0x04, 0x12, 0x34};
        (void)!::write(master, reply, sizeof reply);
    });
    std::uint8_t buffer[5] = {};
    const std::size_t got = port.read(buffer, sizeof buffer);
    unit.join();
    CHECK_EQ(got, std::size_t{5});
    CHECK_EQ(static_cast<int>(buffer[2]), 4);
    CHECK_EQ(static_cast<int>(buffer[4]), 0x34);
}

TEST(bus_serial_posix_nothing_sent_is_a_timeout_not_a_vanished_port) {
    Pty pty;
    CHECK(pty.master >= 0);
    if (pty.master < 0) return;
    SerialTransport port(pty.slave, 38400, 0.1);
    std::uint8_t buffer[4] = {};
    CHECK_EQ(port.read(buffer, sizeof buffer), std::size_t{0});
}

TEST(bus_serial_posix_closed_other_end_is_a_vanished_port) {
    Pty pty;
    CHECK(pty.master >= 0);
    if (pty.master < 0) return;
    SerialTransport port(pty.slave, 38400, 0.5);
    ::close(pty.master);
    pty.master = -1;
    std::uint8_t buffer[4] = {};
    CHECK_THROWS(port.read(buffer, sizeof buffer), TransportError);
}

#endif  // _WIN32
