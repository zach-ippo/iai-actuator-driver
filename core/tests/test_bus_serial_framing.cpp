// Bus unit: the character framing of a serial port (SerialFraming). Its checks, its text, a SerialTransport that
// refuses an unsupported framing before it opens anything (on a port name that does not exist, so no port is ever
// opened), and the platform mapping of every supported framing: the DCB fields on Windows, the termios flags
// elsewhere.
#include <cstring>
#include <stdexcept>
#include <string>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "harness.hpp"
#include "serial_framing.hpp"

using namespace iai_actuator_driver;

namespace {

const char* const kNoSuchPort = "IAD-NO-SUCH-PORT";  // never a real port: opening it would be a TransportError

}  // namespace

TEST(bus_serial_framing_default_is_8n1_and_checks_its_values) {
    const SerialFraming d;
    CHECK_EQ(d.data_bits, 8);
    CHECK(d.parity == Parity::None);
    CHECK_EQ(d.stop_bits, 1);
    CHECK_EQ(to_string(d), std::string("8N1"));
    CHECK_EQ(to_string(SerialFraming{7, Parity::Even, 2}), std::string("7E2"));
    CHECK_EQ(to_string(SerialFraming{8, Parity::Odd, 1}), std::string("8O1"));
    CHECK(SerialFraming{} == d);
    CHECK((SerialFraming{7, Parity::None, 1} != d));
    for (int data_bits : {7, 8})
        for (Parity parity : {Parity::None, Parity::Even, Parity::Odd})
            for (int stop_bits : {1, 2}) SerialFraming{data_bits, parity, stop_bits}.check();  // no throw
    CHECK_THROWS(SerialFraming({6, Parity::None, 1}).check(), std::invalid_argument);
    CHECK_THROWS(SerialFraming({9, Parity::None, 1}).check(), std::invalid_argument);
    CHECK_THROWS(SerialFraming({0, Parity::None, 1}).check(), std::invalid_argument);
    CHECK_THROWS(SerialFraming({8, Parity::None, 0}).check(), std::invalid_argument);
    CHECK_THROWS(SerialFraming({8, Parity::None, 3}).check(), std::invalid_argument);
    CHECK_THROWS(SerialFraming({8, static_cast<Parity>(3), 1}).check(), std::invalid_argument);
}

TEST(bus_serial_framing_is_refused_before_the_port_is_opened) {
    // std::invalid_argument, not the TransportError an open of the absent port gives: the check ran first
    CHECK_THROWS(SerialTransport(kNoSuchPort, 115200, 0.2, SerialFraming{5, Parity::None, 1}), std::invalid_argument);
    CHECK_THROWS(SerialTransport(kNoSuchPort, 115200, 0.2, SerialFraming{8, static_cast<Parity>(7), 1}),
                 std::invalid_argument);
    CHECK_THROWS(SerialTransport(kNoSuchPort, 115200, 0.2, SerialFraming{8, Parity::Even, 3}), std::invalid_argument);
    try {
        SerialTransport bad(kNoSuchPort, 115200, 0.2, SerialFraming{8, Parity::None, 0});
        CHECK(false);
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("stop bits must be 1 or 2") != std::string::npos);
    }
    // a supported framing gets as far as the open, which finds no port
    CHECK_THROWS(SerialTransport(kNoSuchPort, 115200, 0.2, SerialFraming{7, Parity::Even, 2}), TransportError);
}

#ifdef _WIN32

TEST(bus_serial_framing_sets_the_dcb) {
    struct Case {
        SerialFraming framing;
        BYTE byte_size, parity, stop_bits;
        DWORD parity_check;
    };
    const Case cases[] = {
        {SerialFraming{}, 8, NOPARITY, ONESTOPBIT, FALSE},
        {SerialFraming{7, Parity::Even, 1}, 7, EVENPARITY, ONESTOPBIT, TRUE},
        {SerialFraming{8, Parity::Odd, 2}, 8, ODDPARITY, TWOSTOPBITS, TRUE},
        {SerialFraming{7, Parity::None, 2}, 7, NOPARITY, TWOSTOPBITS, FALSE},
    };
    for (const Case& c : cases) {
        DCB dcb;
        std::memset(&dcb, 0xFF, sizeof dcb);  // whatever the port had before
        dcb.fParity = TRUE;
        detail::apply_framing(dcb, c.framing);
        CHECK_EQ(static_cast<int>(dcb.ByteSize), static_cast<int>(c.byte_size));
        CHECK_EQ(static_cast<int>(dcb.Parity), static_cast<int>(c.parity));
        CHECK_EQ(static_cast<int>(dcb.StopBits), static_cast<int>(c.stop_bits));
        CHECK_EQ(static_cast<int>(dcb.fParity), static_cast<int>(c.parity_check));
        CHECK_EQ(static_cast<int>(dcb.fBinary), 1);  // the rest is left as it was
    }
}

#else

TEST(bus_serial_framing_sets_the_termios_flags) {
    struct Case {
        SerialFraming framing;
        tcflag_t size;
        bool parity, odd, two_stop;
    };
    const Case cases[] = {
        {SerialFraming{}, CS8, false, false, false},
        {SerialFraming{7, Parity::Even, 1}, CS7, true, false, false},
        {SerialFraming{8, Parity::Odd, 2}, CS8, true, true, true},
        {SerialFraming{7, Parity::None, 2}, CS7, false, false, true},
    };
    for (const Case& c : cases) {
        for (bool all_set : {false, true}) {  // whatever the port had before
            termios tio;
            std::memset(&tio, all_set ? 0xFF : 0x00, sizeof tio);
            detail::apply_framing(tio, c.framing);
            CHECK_EQ(static_cast<long>(tio.c_cflag & CSIZE), static_cast<long>(c.size));
            CHECK_EQ((tio.c_cflag & PARENB) != 0, c.parity);
            CHECK_EQ((tio.c_cflag & PARODD) != 0, c.odd);
            CHECK_EQ((tio.c_cflag & CSTOPB) != 0, c.two_stop);
            CHECK_EQ((tio.c_iflag & INPCK) != 0, c.parity);  // parity checked only with a parity bit
            CHECK((tio.c_iflag & IGNPAR) == 0);              // a byte with a parity error is read (as 0), not dropped
#ifdef CMSPAR
            CHECK((tio.c_cflag & CMSPAR) == 0);  // never mark / space parity
#endif
            CHECK_EQ((tio.c_cflag & CLOCAL) != 0, all_set);  // the other flags are left as they were
            CHECK(detail::has_framing(tio, c.framing));
        }
    }
}

TEST(bus_serial_framing_read_back_finds_a_dropped_flag) {
    // what a driver that cannot carry a framing does (a Linux pty: CS8 and PARENB clear whatever was asked for)
    for (const SerialFraming& framing : {SerialFraming{7, Parity::Even, 1}, SerialFraming{8, Parity::Odd, 2},
                                         SerialFraming{7, Parity::None, 1}, SerialFraming{8, Parity::Even, 1}}) {
        termios tio{};
        detail::apply_framing(tio, framing);
        tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB);
        tio.c_cflag |= static_cast<tcflag_t>(CS8);
        CHECK(!detail::has_framing(tio, framing));
    }
    termios odd{};
    detail::apply_framing(odd, SerialFraming{8, Parity::Odd, 1});
    odd.c_cflag &= ~static_cast<tcflag_t>(PARODD);  // even instead of odd
    CHECK(!detail::has_framing(odd, SerialFraming{8, Parity::Odd, 1}));
    termios stop{};
    detail::apply_framing(stop, SerialFraming{8, Parity::None, 2});
    stop.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    CHECK(!detail::has_framing(stop, SerialFraming{8, Parity::None, 2}));
    termios kept{};
    detail::apply_framing(kept, SerialFraming{8, Parity::None, 2});
    kept.c_cflag |= static_cast<tcflag_t>(PARODD);  // PARODD without a parity bit sends nothing: not compared
    CHECK(detail::has_framing(kept, SerialFraming{8, Parity::None, 2}));
}

#endif  // _WIN32
