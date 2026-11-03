// SerialTransport on Linux and macOS (POSIX termios) against a pseudo terminal: a reply that arrives after the request
// is read (the read waits: Linux returns 0, not EAGAIN, from a VMIN = VTIME = 0 read while nothing has arrived yet,
// VERIFIED on a CP210x adapter), nothing sent is a timeout, and the other end closed is a vanished port. POSIX only.
#if !defined(_WIN32) && !defined(_GNU_SOURCE) && !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE 700  // posix_openpt, grantpt, unlockpt, ptsname
#endif
#ifndef _WIN32

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>  // std::this_thread::sleep_for
#include <vector>

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

}  // namespace

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
