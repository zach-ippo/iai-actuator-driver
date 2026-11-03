// C ABI 1.1: iad_transport_open_serial_options and its sized struct iad_serial_options (the character framing of a
// serial port). No port is opened: every call names a port that does not exist, so a refusal (IAD_ERR_INVALID_ARGUMENT,
// IAD_ERR_UNSUPPORTED) shows the check came first, and only a valid request reaches the open (IAD_ERR_TRANSPORT).
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "harness.hpp"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const char* const kNoSuchPort = "IAD-NO-SUCH-PORT";  // never a real port: CreateFile / open() finds nothing

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

iad_serial_options defaults() {
    iad_serial_options o;
    std::memset(&o, 0, sizeof o);
    o.struct_size = sizeof o;
    REQUIRE(iad_serial_options_init(&o) == IAD_OK);
    return o;
}

/// The result of opening kNoSuchPort with `o`; *out must stay NULL whatever happens (nothing is ever opened).
iad_result open_with(const iad_serial_options* o) {
    iad_transport* out = reinterpret_cast<iad_transport*>(std::uintptr_t{1});  // overwritten with NULL first
    const iad_result r = iad_transport_open_serial_options(kNoSuchPort, o, &out);
    CHECK(out == nullptr);
    if (out) iad_transport_close(out);
    return r;
}

}  // namespace

TEST(capi11_version_is_1_1) {
    CHECK_EQ(IAD_ABI_VERSION_MAJOR, 1);
    CHECK_EQ(IAD_ABI_VERSION_MINOR, 1);
    CHECK_EQ(iad_abi_version(), static_cast<std::uint32_t>(IAD_ABI_VERSION));
    CHECK(IAD_ABI_COMPATIBLE(iad_abi_version()));
    CHECK(!IAD_ABI_COMPATIBLE((1u << 16) | 0u));  // a caller of this header needs 1.1
    CHECK(!IAD_ABI_COMPATIBLE((2u << 16) | 1u));
}

TEST(capi11_serial_options_init_writes_8n1_and_its_size) {
    const iad_serial_options o = defaults();
    CHECK_EQ(o.struct_size, static_cast<std::uint32_t>(64));
    CHECK_EQ(o.reserved_header, 0u);
    CHECK_EQ(o.read_timeout_s, 0.2);
    CHECK_EQ(o.baud_rate, 115200);
    CHECK_EQ(o.data_bits, 8);
    CHECK_EQ(o.parity, IAD_PARITY_NONE);
    CHECK_EQ(o.stop_bits, 1);
    for (std::uint64_t slot : o.reserved) CHECK_EQ(slot, std::uint64_t{0});
    CHECK_EQ(iad_serial_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(IAD_PARITY_NONE, 0);
    CHECK_EQ(IAD_PARITY_EVEN, 1);
    CHECK_EQ(IAD_PARITY_ODD, 2);
}

TEST(capi11_serial_options_refuse_bad_values_before_opening) {
    struct Case {
        const char* what;
        void (*set)(iad_serial_options&);
        const char* message;
    };
    const Case cases[] = {
        {"data bits 6", [](iad_serial_options& o) { o.data_bits = 6; }, "data_bits must be 7 or 8"},
        {"data bits 9", [](iad_serial_options& o) { o.data_bits = 9; }, "data_bits must be 7 or 8"},
        {"data bits -1", [](iad_serial_options& o) { o.data_bits = -1; }, "data_bits must be 7 or 8"},
        {"parity 3", [](iad_serial_options& o) { o.parity = 3; }, "parity must be IAD_PARITY_NONE"},
        {"parity -1", [](iad_serial_options& o) { o.parity = -1; }, "parity must be IAD_PARITY_NONE"},
        {"stop bits 3", [](iad_serial_options& o) { o.stop_bits = 3; }, "stop_bits must be 1 or 2"},
        {"stop bits -2", [](iad_serial_options& o) { o.stop_bits = -2; }, "stop_bits must be 1 or 2"},
        {"baud rate -1", [](iad_serial_options& o) { o.baud_rate = -1; }, "baud_rate must be a rate"},
        {"read timeout -1", [](iad_serial_options& o) { o.read_timeout_s = -1.0; }, "read_timeout_s must be >= 0"},
        {"read timeout inf",
         [](iad_serial_options& o) { o.read_timeout_s = std::numeric_limits<double>::infinity(); },
         "read_timeout_s must be >= 0"},
    };
    for (const Case& c : cases) {
        iad_serial_options o = defaults();
        c.set(o);
        if (open_with(&o) != IAD_ERR_INVALID_ARGUMENT) {
            ::ec_test::fail(__FILE__, __LINE__, std::string(c.what) + ": not IAD_ERR_INVALID_ARGUMENT");
            continue;
        }
        const std::string message = iad_last_error_message();
        if (!has(message, c.message)) ::ec_test::fail(__FILE__, __LINE__, std::string(c.what) + ": " + message);
    }
}

TEST(capi11_serial_options_refuse_a_struct_they_cannot_read) {
    // the "Sized structs" rule: too small (0, below the first size), not a multiple of 8, far beyond the library's
    // own size, a set reserved_header: IAD_ERR_INVALID_ARGUMENT; a set reserved slot: IAD_ERR_UNSUPPORTED
    for (std::uint32_t size : {0u, 8u, 56u, 60u, 65u, 64u + 4096u + 8u}) {
        iad_serial_options o = defaults();
        o.struct_size = size;
        CHECK_EQ(open_with(&o), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(iad_last_error_message(), "struct_size"));
    }
    iad_serial_options header = defaults();
    header.reserved_header = 1;
    CHECK_EQ(open_with(&header), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(iad_last_error_message(), "reserved_header must be 0"));
    for (int slot = 0; slot < 4; ++slot) {
        iad_serial_options o = defaults();
        o.reserved[slot] = 1;
        CHECK_EQ(open_with(&o), IAD_ERR_UNSUPPORTED);
    }
    // a bigger caller struct with a set byte past the library's own: a field of a later ABI, never ignored
    alignas(8) unsigned char later[sizeof(iad_serial_options) + 8] = {};
    const iad_serial_options base = defaults();
    std::memcpy(later, &base, sizeof base);
    const std::uint32_t size = sizeof later;
    std::memcpy(later, &size, sizeof size);
    later[sizeof base] = 1;
    CHECK_EQ(open_with(reinterpret_cast<const iad_serial_options*>(later)), IAD_ERR_UNSUPPORTED);
    later[sizeof base] = 0;  // zeroed extra bytes are read as this library's struct
    CHECK_EQ(open_with(reinterpret_cast<const iad_serial_options*>(later)), IAD_ERR_TRANSPORT);
}

TEST(capi11_serial_options_refuse_null_and_empty_arguments) {
    iad_transport* out = nullptr;
    const iad_serial_options o = defaults();
    CHECK_EQ(iad_transport_open_serial_options(kNoSuchPort, &o, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_transport_open_serial_options(nullptr, &o, &out), IAD_ERR_INVALID_ARGUMENT);
    CHECK(out == nullptr);
    CHECK_EQ(iad_transport_open_serial_options("", &o, &out), IAD_ERR_INVALID_ARGUMENT);
    CHECK(out == nullptr);
}

TEST(capi11_serial_options_valid_framings_reach_the_open) {
    // every supported framing passes the checks and only then fails, at the port (absent: IAD_ERR_TRANSPORT)
    CHECK_EQ(open_with(nullptr), IAD_ERR_TRANSPORT);  // NULL: the defaults
    CHECK(has(iad_last_error_message(), kNoSuchPort));
    for (int data_bits : {0, 7, 8})
        for (int parity : {IAD_PARITY_NONE, IAD_PARITY_EVEN, IAD_PARITY_ODD})
            for (int stop_bits : {0, 1, 2}) {
                iad_serial_options o = defaults();
                o.data_bits = data_bits;
                o.parity = parity;
                o.stop_bits = stop_bits;
                CHECK_EQ(open_with(&o), IAD_ERR_TRANSPORT);
            }
    iad_serial_options zeroed_rate = defaults();
    zeroed_rate.baud_rate = 0;  // 0 = 115200
    zeroed_rate.read_timeout_s = kNaN;  // NaN = 0.2
    CHECK_EQ(open_with(&zeroed_rate), IAD_ERR_TRANSPORT);
}

TEST(capi11_serial_options_translate_to_the_framing_the_port_opens_with) {
    // the C values as the C++ SerialTransport receives them (capi::to_serial_args, the open's only translation; the
    // POSIX pty test in test_serial_posix.cpp checks the same values on the port's flags)
    namespace capi = iai_actuator_driver::capi;
    using iai_actuator_driver::Parity;
    using iai_actuator_driver::SerialFraming;
    struct Case {
        int32_t data_bits, parity, stop_bits;
        SerialFraming framing;
    };
    const Case cases[] = {
        {0, IAD_PARITY_NONE, 0, SerialFraming{8, Parity::None, 1}},
        {8, IAD_PARITY_NONE, 1, SerialFraming{8, Parity::None, 1}},
        {7, IAD_PARITY_EVEN, 1, SerialFraming{7, Parity::Even, 1}},
        {0, IAD_PARITY_ODD, 2, SerialFraming{8, Parity::Odd, 2}},
        {7, IAD_PARITY_NONE, 2, SerialFraming{7, Parity::None, 2}},
        {8, IAD_PARITY_EVEN, 0, SerialFraming{8, Parity::Even, 1}},
    };
    for (const Case& c : cases) {
        iad_serial_options o = defaults();
        o.data_bits = c.data_bits;
        o.parity = c.parity;
        o.stop_bits = c.stop_bits;
        const capi::SerialArgs a = capi::to_serial_args(&o);
        if (!(a.framing == c.framing))
            ::ec_test::fail(__FILE__, __LINE__, std::to_string(c.data_bits) + "/" + std::to_string(c.parity) + "/" +
                                                    std::to_string(c.stop_bits) + " -> " + to_string(a.framing));
        CHECK_EQ(a.baud_rate, 115200);
        CHECK_EQ(a.read_timeout_s, 0.2);
    }
    iad_serial_options zeroed_rate = defaults();
    zeroed_rate.baud_rate = 0;          // 0 = 115200
    zeroed_rate.read_timeout_s = kNaN;  // NaN = 0.2
    const capi::SerialArgs zeroed = capi::to_serial_args(&zeroed_rate);
    CHECK_EQ(zeroed.baud_rate, 115200);
    CHECK_EQ(zeroed.read_timeout_s, 0.2);
    iad_serial_options other = defaults();
    other.baud_rate = 38400;
    other.read_timeout_s = 0.05;
    const capi::SerialArgs given = capi::to_serial_args(&other);
    CHECK_EQ(given.baud_rate, 38400);
    CHECK_EQ(given.read_timeout_s, 0.05);
    const capi::SerialArgs none = capi::to_serial_args(nullptr);  // NULL: 8N1
    CHECK(none.framing == SerialFraming{});
    CHECK_EQ(none.baud_rate, 115200);
    CHECK_EQ(none.read_timeout_s, 0.2);
}
