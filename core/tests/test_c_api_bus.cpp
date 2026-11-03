// C ABI 0.3 additions (include/iai_actuator_driver.h "transports" to "axis and cell: cancel latch", src/c_api_bus.cpp,
// src/c_api_sim.cpp) on the simulator: the sized structs, transports (simulated, the binding's callbacks), buses with
// raw requests and the frame sink, the simulator hooks (word overrides, unreadable ranges, stale status polls, a
// stepped clock, the request hook), power and link loss, a line break, simulated lines, axes and cells on transports
// and buses, the clock seam and the cancel latch. The 0.4 additions (capi04_; "buses: bus lock" to "buses: response
// timing"): the bus lock across calls, the preemptible read scope, suspended monitors, the IAI flags, 0x48 (through a
// request hook that answers it), broadcasts, the line on a bus and the response timing. Simulator only: no COM port is
// opened.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 20; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

Json state(iad_sim* sim) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_sim_state_json(sim, b, s, r); }));
}

iad_sim* open_sim(const std::string& options) {
    iad_sim* sim = nullptr;
    CHECK_EQ(iad_sim_open(options.c_str(), &sim), IAD_OK);
    REQUIRE(sim != nullptr);
    return sim;
}

/// A bus without gaps on a transport handle (the transport handle is released: the bus keeps it).
iad_bus* bus_on(iad_transport* t, int retries = 0) {
    iad_bus_options o;
    iad_bus_options_init(&o);
    o.min_gap_s = 0.0;
    o.retries = retries;
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(t, &o, &bus), IAD_OK);
    REQUIRE(bus != nullptr);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    return bus;
}

iad_bus* bus_on_sim(iad_sim* sim, int retries = 0) {
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_transport(sim, &t), IAD_OK);
    return bus_on(t, retries);
}

iad_axis_options quick_axis() {
    iad_axis_options o;
    iad_axis_options_init(&o);
    o.poll_interval_s = 0.005;
    return o;
}

/// The frames a sink saw.
struct Frames {
    std::vector<std::pair<int32_t, std::vector<std::uint8_t>>> list;
    static void IAD_CALL sink(int32_t direction, const uint8_t* bytes, size_t length, void* user) {
        static_cast<Frames*>(user)->list.emplace_back(direction, std::vector<std::uint8_t>(bytes, bytes + length));
    }
};

/// The binding's own transport for the callback tests: an in-memory simulated unit behind C callbacks.
struct Host {
    std::shared_ptr<SimulatedUnit> unit = std::make_shared<SimulatedUnit>(SimProfile::Slider);
    bool fail_writes = false;
    int writes = 0;
    int closes = 0;
    std::atomic<int> releases{0};

    static int32_t IAD_CALL write(void* user, const uint8_t* data, size_t size) {
        auto* h = static_cast<Host*>(user);
        ++h->writes;
        if (h->fail_writes) {
            iad_callback_error("host port gone (test)");
            return -1;
        }
        h->unit->write(data, size);
        return 0;
    }
    static int32_t IAD_CALL read(void* user, uint8_t* buffer, size_t size, double, size_t* got) {
        *got = static_cast<Host*>(user)->unit->read(buffer, size);
        return 0;
    }
    static int32_t IAD_CALL reset_input(void* user) {
        static_cast<Host*>(user)->unit->reset_input();
        return 0;
    }
    static void IAD_CALL close(void* user) { ++static_cast<Host*>(user)->closes; }
    static void IAD_CALL release(void* user) { ++static_cast<Host*>(user)->releases; }
};

// a request hook: FC11 unanswered, FC03 at 0x0100 answered with exception 04, FC03 at 0x9000 answered by the unit and
// then replaced by the same answer with a changed word (act and replace)
int32_t IAD_CALL test_hook(const uint8_t* request, size_t size, uint8_t* reply, size_t capacity, size_t* length,
                           void* user) {
    auto* sim = static_cast<iad_sim*>(user);
    if (size > 1 && request[1] == 0x11) return IAD_SIM_DROP;
    if (size > 3 && request[1] == 0x03 && request[2] == 0x01 && request[3] == 0x00) return IAD_SIM_EXCEPTION(4);
    if (size > 3 && request[1] == 0x03 && request[2] == 0x90 && request[3] == 0x00) {
        if (iad_sim_handle_frame(sim, request, size, reply, capacity, length) != IAD_OK || *length < 5) return IAD_SIM_DROP;
        reply[3] = 0x12;  // the first word of the answer
        reply[4] = 0x34;
        std::vector<std::uint8_t> frame(reply, reply + *length - 2);
        append_crc(frame);
        std::memcpy(reply, frame.data(), frame.size());
        return IAD_SIM_REPLY;
    }
    return IAD_SIM_PASS;
}

}  // namespace

TEST(capi03_version_and_sized_structs) {
    CHECK_EQ(iad_abi_version(), (1u << 16) | 0u);
    iad_bus_options o;
    std::memset(&o, 0xAB, sizeof o);
    CHECK_EQ(iad_bus_options_init(&o), IAD_OK);
    CHECK_EQ(o.struct_size, static_cast<uint32_t>(sizeof o));
    CHECK_EQ(o.reserved_header, 0u);
    CHECK_EQ(o.retries, 3);
    CHECK_NEAR(o.min_gap_s, 0.005, 1e-12);
    iad_transport_callbacks cb;
    std::memset(&cb, 0xAB, sizeof cb);
    cb.struct_size = static_cast<uint32_t>(sizeof cb);  // 0.5: the caller says it has the whole struct
    CHECK_EQ(iad_transport_callbacks_init(&cb), IAD_OK);
    CHECK_EQ(cb.struct_size, static_cast<uint32_t>(sizeof cb));
    CHECK_EQ(cb.real_timeout, IAD_TRUE);
    CHECK(cb.write == nullptr && cb.release == nullptr && cb.set_read_timeout == nullptr);
    CHECK_EQ(iad_bus_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);

    iad_sim* sim = open_sim("{}");
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_transport(sim, &t), IAD_OK);
    iad_bus* bus = nullptr;
    iad_bus_options small = o;
    small.struct_size = 8;  // below the 0.3 size
    CHECK_EQ(iad_bus_open(t, &small, &bus), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "struct_size"));
    iad_bus_options header = o;
    header.reserved_header = 1;
    CHECK_EQ(iad_bus_open(t, &header, &bus), IAD_ERR_INVALID_ARGUMENT);
    iad_bus_options later = o;
    later.reserved[2] = 7;  // a later ABI's field
    CHECK_EQ(iad_bus_open(t, &later, &bus), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "reserved[2]"));
    struct Bigger {
        iad_bus_options base;
        uint64_t next;
    } bigger;
    std::memset(&bigger, 0, sizeof bigger);
    bigger.base = o;
    bigger.base.struct_size = static_cast<uint32_t>(sizeof bigger);
    CHECK_EQ(iad_bus_open(t, &bigger.base, &bus), IAD_OK);  // a bigger struct with zero tail: read as far as known
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    bigger.next = 1;
    CHECK_EQ(iad_bus_open(t, &bigger.base, &bus), IAD_ERR_UNSUPPORTED);
    CHECK_EQ(bus, static_cast<iad_bus*>(nullptr));
    // wrong kinds and closed handles
    CHECK_EQ(iad_bus_close(reinterpret_cast<iad_bus*>(t)), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "another kind of handle"));
    CHECK_EQ(iad_sim_unplug(reinterpret_cast<iad_sim*>(t)), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "a transport handle was passed as a simulator handle"));
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);  // twice: fine
    CHECK_EQ(iad_transport_reset_input(t), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
    CHECK_EQ(iad_sim_close(nullptr), IAD_OK);
}

TEST(capi03_bus_requests_frame_sink_and_info) {
    iad_sim* sim = open_sim(R"({"profile": "slider", "slave": 2})");
    iad_bus* bus = bus_on_sim(sim);
    Frames frames;
    CHECK_EQ(iad_bus_set_frame_sink(bus, &Frames::sink, &frames), IAD_OK);
    uint16_t words[8] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9000, 8, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(frames.list.size(), 2u);
    if (frames.list.size() == 2) {
        CHECK_EQ(frames.list[0].first, IAD_FRAME_TX);
        CHECK_EQ(frames.list[1].first, IAD_FRAME_RX);
        CHECK_EQ(static_cast<int>(frames.list[0].second[0]), 2);
        CHECK_EQ(frames.list[1].second.size(), 5u + 16u);
    }
    uint8_t id[64];
    size_t length = 0;
    CHECK_EQ(iad_bus_report_slave_id(bus, 2, 0, kNaN, id, sizeof id, &length), IAD_OK);
    CHECK(length >= 3);
    CHECK_EQ(iad_bus_report_slave_id(bus, 2, 0, kNaN, id, 1, &length), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK(length >= 3);
    iad_bool answered = IAD_FALSE;
    CHECK_EQ(iad_bus_probe(bus, 2, kNaN, 0x9000, &answered), IAD_OK);
    CHECK_EQ(answered, IAD_TRUE);
    CHECK_EQ(iad_bus_probe(bus, 9, 0.02, 0x9000, &answered), IAD_OK);
    CHECK_EQ(answered, IAD_FALSE);
    CHECK_EQ(iad_bus_read_registers(bus, 9, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_MODBUS_TIMEOUT);
    CHECK_EQ(iad_bus_read_registers(bus, 0, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_write_coil(bus, 2, reg::kCoilStop, IAD_TRUE, IAD_INT_UNKNOWN), IAD_OK);
    CHECK(state(sim).find("stop_held")->as_bool());
    CHECK_EQ(iad_bus_write_coil(bus, 2, reg::kCoilStop, IAD_FALSE, IAD_INT_UNKNOWN), IAD_OK);
    int32_t rejection = -1;
    uint32_t status = 0;
    CHECK_EQ(iad_bus_iai_command(bus, 2, 0x45, 0, 0x7F, nullptr, 0, 0, 0, &rejection, &status, 1),
             IAD_ERR_UNSAFE_REFUSED);  // not on the safe list: nothing sent
    CHECK_EQ(iad_bus_set_frame_sink(bus, nullptr, nullptr), IAD_OK);
    const std::size_t seen = frames.list.size();
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(frames.list.size(), seen);
    CHECK_EQ(iad_bus_set_retries(bus, 5), IAD_OK);
    CHECK_EQ(iad_bus_set_min_gap(bus, -1.0), IAD_ERR_INVALID_ARGUMENT);
    const Json info = Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_bus_info_json(bus, b, s, r); }));
    CHECK_EQ(info.find("retries")->as_int(), 5LL);
    CHECK_EQ(info.find("closed")->as_bool(), false);
    CHECK(info.find("baud_rate")->is_null());  // the simulator models no rate here
    CHECK_EQ(info.find("name")->as_string(), std::string("SIM"));
    iad_transport* t = nullptr;
    CHECK_EQ(iad_bus_transport(bus, &t), IAD_OK);
    const Json tinfo =
        Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_transport_info_json(t, b, s, r); }));
    CHECK_EQ(tinfo.find("open")->as_bool(), true);
    CHECK_EQ(tinfo.find("can_reopen")->as_bool(), true);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // owned: closes the bus and the simulated port
    CHECK_EQ(state(sim).find("open")->as_bool(), false);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_sim_hooks_overrides_unreadable_and_kinds) {
    iad_sim* sim = open_sim(R"({"word_overrides": {"0x9002": 4660}, "unreadable": [[256, 4]]})");
    iad_bus* bus = bus_on_sim(sim);
    uint16_t words[4] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 4, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(words[2], 0x1234);
    uint16_t value = 0;
    CHECK_EQ(iad_sim_register_value(sim, 0x9002, &value), IAD_OK);
    CHECK_EQ(value, 0x1234);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x0102, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_SLAVE_EXCEPTION);
    CHECK(has(last_json(), "\"exception_code\":2"));
    CHECK_EQ(iad_sim_register_value(sim, 0x0101, &value), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "\"kind\":\"std::out_of_range\""));
    CHECK_EQ(iad_sim_set_unreadable(sim, 0x0100, 1, IAD_FALSE), IAD_OK);  // the overlapping range goes
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x0102, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(iad_sim_set_word_override(sim, 0x9002, IAD_INT_UNKNOWN), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 4, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK(words[2] != 0x1234);
    CHECK_EQ(iad_sim_set_word_override(sim, 0x9002, 0x10000), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "std::out_of_range"));
    // invalid_argument stays its own kind
    iad_sim* bad = nullptr;
    CHECK_EQ(iad_sim_open(R"({"profile": "nope"})", &bad), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "\"kind\":\"std::invalid_argument\""));
    CHECK_EQ(iad_sim_open(R"({"colour": 1})", &bad), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "unknown simulator option 'colour'"));
    CHECK_EQ(iad_sim_open(R"({"slave": 300})", &bad), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "std::out_of_range"));
    CHECK_EQ(iad_sim_open("[1]", &bad), IAD_ERR_INVALID_ARGUMENT);
    // the JSON controls
    const std::string row = read_text([&](char* b, size_t s, size_t* r) {
        return iad_sim_control_json(sim, R"({"command": "position_row", "row": 0})", b, s, r);
    });
    CHECK(has(row, "\"words\":["));
    CHECK_EQ(read_text([&](char* b, size_t s, size_t* r) {
                 return iad_sim_control_json(sim, R"({"command": "set_coil_state", "address": 1068, "on": true})", b, s, r);
             }),
             std::string("{}"));
    CHECK(has(read_text([&](char* b, size_t s, size_t* r) {
              return iad_sim_control_json(sim, R"({"command": "coil_state", "address": 1068})", b, s, r);
          }),
          "\"on\":true"));
    std::size_t need = 0;
    CHECK_EQ(iad_sim_control_json(sim, R"({"command": "position_row", "row": 9999})", nullptr, 0, &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "std::out_of_range"));
    CHECK_EQ(iad_sim_control_json(sim, R"({"command": "dance"})", nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_control_json(sim, R"({"command": "inject_fault", "kind": "timeout"})", nullptr, 0, &need),
             IAD_ERR_BUFFER_TOO_SMALL);  // "{}" did not fit; the command ran
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_MODBUS_TIMEOUT);
    const std::string requests =
        read_text([&](char* b, size_t s, size_t* r) { return iad_sim_requests_json(sim, b, s, r); });
    CHECK(has(requests, "\"frame\":\"01 03 90 00"));
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_sim_stale_status_and_step_clock) {
    iad_sim* sim = open_sim(R"({"time_scale": 20, "stale_status_polls": 50, "step_s": 0.01})");
    iad_bus* bus = bus_on_sim(sim, 3);
    const iad_axis_options ao = quick_axis();
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, &ao, &ax), IAD_OK);
    REQUIRE(ax != nullptr);
    CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, nullptr), IAD_OK);
    const double before = state(sim).find("clock_s")->as_number();
    uint16_t words[8] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 8, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_NEAR(state(sim).find("clock_s")->as_number() - before, 0.01, 1e-9);  // one request, one step
    iad_move_options mo;
    iad_move_options_init(&mo);
    mo.speed = 100.0;
    mo.wait = IAD_FALSE;
    CHECK_EQ(iad_axis_move_to(ax, 100.0, &mo, nullptr), IAD_OK);  // FC10 at 0x9900 arms the stale polls
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 8, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK(words[reg::kDss1 - 0x9000] & reg::kDss1Pend);
    CHECK_EQ(words[reg::kDsse - 0x9000] & reg::kDsseMove, 0);
    CHECK(state(sim).find("stale_polls_served")->as_int() >= 1);
    CHECK_EQ(iad_axis_stop(ax, ptr_of(stop_opts(IAD_TRUE, 5.0)), nullptr), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);  // not the bus's owner: the bus stays open
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(iad_sim_set_stale_status_polls(sim, -1), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_sim_request_hook) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    CHECK_EQ(iad_sim_set_request_hook(sim, &test_hook, sim), IAD_OK);
    uint8_t id[64];
    size_t length = 0;
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, 0, kNaN, id, sizeof id, &length), IAD_ERR_MODBUS_TIMEOUT);  // dropped
    uint16_t words[2] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x0100, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_SLAVE_EXCEPTION);
    CHECK(has(last_json(), "\"exception_code\":4"));
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(words[0], 0x1234);  // the unit's answer, replaced
    CHECK(has(read_text([&](char* b, size_t s, size_t* r) { return iad_sim_requests_json(sim, b, s, r); }), "01 11"));
    CHECK_EQ(iad_sim_set_request_hook(sim, nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, 0, kNaN, id, sizeof id, &length), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_callback_transport) {
    Host host;
    iad_transport_callbacks cb;
    iad_transport_callbacks_init(&cb);
    cb.real_timeout = IAD_FALSE;  // in memory: the bus polls
    cb.write = &Host::write;
    cb.read = &Host::read;
    cb.reset_input = &Host::reset_input;
    cb.close = &Host::close;
    cb.release = &Host::release;
    iad_transport* t = nullptr;
    iad_transport_callbacks missing = cb;
    missing.read = nullptr;
    CHECK_EQ(iad_transport_open_callbacks(&missing, "HOST", &host, &t), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(host.releases.load(), 0);  // nothing was taken
    CHECK_EQ(iad_transport_open_callbacks(&cb, "HOST", &host, &t), IAD_OK);
    REQUIRE(t != nullptr);
    CHECK_EQ(iad_transport_set_baud_rate(t, 115200), IAD_ERR_UNSUPPORTED);  // no set_baud_rate callback
    CHECK_EQ(iad_transport_send_break(t, 0.2), IAD_ERR_UNSUPPORTED);
    iad_transport* t2 = nullptr;
    CHECK_EQ(iad_transport_open_callbacks(&cb, nullptr, &host, &t2), IAD_OK);
    const Json info = Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_transport_info_json(t2, b, s, r); }));
    CHECK_EQ(info.find("name")->as_string(), std::string("CALLBACK"));
    CHECK(info.find("present")->is_null());
    CHECK_EQ(iad_transport_release(t2), IAD_OK);
    CHECK_EQ(host.releases.load(), 1);  // the last owner let go
    // an axis on it, through the binding's transport
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    open.line = IAD_LINE_DIRECT;
    const iad_axis_options ao = quick_axis();
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_transport(t, &open, &ao, &ax), IAD_OK);
    REQUIRE(ax != nullptr);
    CHECK_EQ(iad_transport_release(t), IAD_OK);  // the axis keeps it
    iad_identity id;
    CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, &id), IAD_OK);
    CHECK(host.writes > 3);
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    host.fail_writes = true;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_ERR_TRANSPORT);
    CHECK(has(last_json(), "host port gone (test)"));
    host.fail_writes = false;
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_axis_bus(ax, &bus), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // borrowed: the handle only
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(host.releases.load(), 1);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);  // owns its bus: closes the transport, then lets go
    CHECK_EQ(host.closes, 1);
    CHECK_EQ(host.releases.load(), 2);
}

TEST(capi03_virtual_clock) {
    double now = 0.0;
    CHECK_EQ(iad_clock_advance(1.0), IAD_ERR_NOT_READY);  // the real clock
    CHECK_EQ(iad_clock_use_virtual(kNaN), IAD_OK);
    CHECK_EQ(iad_clock_now(&now), IAD_OK);
    CHECK_EQ(now, 1000.0);
    CHECK_EQ(iad_clock_advance(2.5), IAD_OK);
    CHECK_EQ(iad_clock_now(&now), IAD_OK);
    CHECK_EQ(now, 1002.5);
    // a silent port with a real read timeout: the bus waits for each attempt's deadline
    iad_transport_callbacks cb;
    iad_transport_callbacks_init(&cb);
    cb.write = [](void*, const uint8_t*, size_t) -> int32_t { return 0; };
    cb.read = [](void*, uint8_t*, size_t, double, size_t* got) -> int32_t {
        *got = 0;
        return 0;
    };
    iad_transport* t = nullptr;
    CHECK_EQ(iad_transport_open_callbacks(&cb, "SILENT", nullptr, &t), IAD_OK);
    iad_bus* bus = bus_on(t, 3);
    const double real0 = steady_now_s();
    uint16_t words[1] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_MODBUS_TIMEOUT);
    CHECK(steady_now_s() - real0 < 0.5);  // four attempts of 0.2 s at least: no real time spent
    CHECK_EQ(iad_clock_now(&now), IAD_OK);
    CHECK(now - 1002.5 >= 0.8);  // the virtual clock moved by the attempts' deadlines
    CHECK_EQ(iad_clock_use_real(), IAD_OK);
    CHECK_EQ(iad_clock_now(&now), IAD_OK);
    CHECK(std::fabs(now - steady_now_s()) < 1.0);
    CHECK_EQ(iad_clock_use_callbacks(nullptr, nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_clock_use_callbacks([](void*) { return 5.0; }, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}

TEST(capi03_power_unplug_and_line_break) {
    iad_sim* sim = open_sim(R"({"baud_rate": 115200})");
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_transport(sim, &t), IAD_OK);
    iad_bus* bus = bus_on(t, 0);
    uint16_t words[1] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(iad_sim_unplug(sim), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_TRANSPORT);
    CHECK_EQ(state(sim).find("present")->as_bool(), false);
    CHECK_EQ(iad_bus_reopen(bus, 0), IAD_ERR_TRANSPORT);  // still unplugged
    CHECK_EQ(iad_sim_replug(sim), IAD_OK);
    CHECK_EQ(iad_bus_reopen(bus, 0), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    // a BREAK of 150 ms or more drops the unit to 9600 baud: silent at 115200 until the rate is found again
    CHECK_EQ(iad_bus_send_break(bus, 0.2), IAD_OK);
    CHECK_EQ(state(sim).find("unit_baud_rate")->as_int(), 9600LL);
    int32_t rate = 0;
    CHECK_EQ(iad_bus_detect_baud_rate(bus, "1", nullptr, IAD_INT_UNKNOWN, IAD_TRUE, kNaN, &rate), IAD_OK);
    CHECK_EQ(rate, 9600);
    CHECK_EQ(iad_bus_set_baud_rate(bus, 115200), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_MODBUS_TIMEOUT);
    CHECK_EQ(iad_bus_send_break(bus, 3.0), IAD_ERR_INVALID_ARGUMENT);
    // a power cycle: back at the boot rate
    CHECK_EQ(iad_sim_power_cycle(sim, IAD_FALSE), IAD_OK);
    CHECK_EQ(state(sim).find("unit_baud_rate")->as_int(), 38400LL);
    CHECK_EQ(iad_sim_power_cycle(sim, 5), IAD_OK);  // (0.8) a tribool input: 0 false, 2 unknown, any other value true
    CHECK_EQ(state(sim).find("unit_baud_rate")->as_int(), 38400LL);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_sim_line_cell_on_transports) {
    iad_sim* s1 = open_sim(R"({"slave": 1, "time_scale": 20})");
    iad_sim* s2 = open_sim(R"({"slave": 2, "time_scale": 20, "profile": "gripper"})");
    iad_sim* twin = open_sim(R"({"slave": 2})");
    iad_sim* units[] = {s1, s2};
    iad_sim_line* line = nullptr;
    CHECK_EQ(iad_sim_line_open("LINE7", units, 2, &line), IAD_OK);
    REQUIRE(line != nullptr);
    CHECK_EQ(iad_sim_line_add(line, twin), IAD_ERR_INVALID_ARGUMENT);  // address 2 is taken
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_line_transport(line, &t), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.line = IAD_LINE_DIRECT;
    const iad_axis_options ao = quick_axis();
    iad_cell* cell = nullptr;
    iad_transport* list[] = {t};
    CHECK_EQ(iad_cell_open_transports(list, 1, nullptr, "1,2", &co, &ao, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    size_t count = 0;
    CHECK_EQ(iad_cell_axis_count(cell, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    iad_axis* a2 = nullptr;
    CHECK_EQ(iad_cell_open_axis(cell, "LINE7:2", &a2), IAD_OK);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_cell_bus(cell, "LINE7", &bus), IAD_OK);
    iad_bus* none = nullptr;
    CHECK_EQ(iad_cell_bus(cell, "COM9", &none), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "std::out_of_range"));
    uint16_t words[1] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    const std::string requests =
        read_text([&](char* b, size_t s, size_t* r) { return iad_sim_line_requests_json(line, b, s, r); });
    CHECK(has(requests, "\"slave\":2"));
    const Json st = Json::parse(read_text([&](char* b, size_t s, size_t* r) {
        return iad_sim_line_control_json(line, R"({"command": "state"})", b, s, r);
    }));
    CHECK_EQ(st.find("name")->as_string(), std::string("LINE7"));
    CHECK_EQ(st.find("slaves")->size(), 2u);
    CHECK_EQ(iad_sim_line_remove(line, 9), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "std::out_of_range"));
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // borrowed: the cell keeps its port
    CHECK_EQ(iad_axis_close(a2), IAD_OK);
    iad_transport* list2[] = {t, t};
    iad_cell* cell2 = nullptr;
    CHECK_EQ(iad_cell_open_transports(list2, 2, nullptr, nullptr, &co, &ao, &cell2), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_transports(list2, 2, "ONLYONE", nullptr, &co, &ao, &cell2), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(iad_sim_line_close(line), IAD_OK);
    for (iad_sim* s : {s1, s2, twin}) CHECK_EQ(iad_sim_close(s), IAD_OK);
}

TEST(capi03_cancel_latch) {
    iad_sim* sim = open_sim("{}");  // real time: a 10 mm/s move over 100 mm takes about 10 s
    iad_bus* bus = bus_on_sim(sim, 3);
    const iad_axis_options ao = quick_axis();
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, &ao, &ax), IAD_OK);
    REQUIRE(ax != nullptr);
    CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, nullptr), IAD_OK);
    iad_bool latched = IAD_TRUE;
    CHECK_EQ(iad_axis_cancel_latched(ax, &latched), IAD_OK);
    CHECK_EQ(latched, IAD_FALSE);
    CHECK_EQ(iad_axis_cancel_latch(ax), IAD_OK);
    CHECK_EQ(iad_axis_cancel_latched(ax, &latched), IAD_OK);
    CHECK_EQ(latched, IAD_TRUE);
    CHECK_EQ(iad_sim_control_json(sim, R"({"command": "clear_requests"})", nullptr, 0, nullptr),
             IAD_ERR_INVALID_ARGUMENT);  // required is mandatory
    std::size_t need = 0;
    iad_sim_control_json(sim, R"({"command": "clear_requests"})", nullptr, 0, &need);
    iad_move_options mo;
    iad_move_options_init(&mo);
    mo.speed = 10.0;
    CHECK_EQ(iad_axis_move_to(ax, 100.0, &mo, nullptr), IAD_ERR_CANCELLED);  // refused: nothing sent
    CHECK(has(last_json(), "cancel latch"));
    CHECK_EQ(read_text([&](char* b, size_t s, size_t* r) { return iad_sim_requests_json(sim, b, s, r); }),
             std::string("[]"));
    CHECK_EQ(iad_axis_home(ax, ptr_of(home_opts(IAD_TRUE, kNaN, IAD_METHOD_DEFAULT)), nullptr), IAD_ERR_CANCELLED);
    iad_status st;
    CHECK_EQ(iad_axis_stop(ax, ptr_of(stop_opts(IAD_FALSE, 5.0)), &st), IAD_OK);  // a stop is never refused
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(iad_axis_cancel_reset(ax), IAD_OK);
    // a move cancelled from another thread through the latch: ends with a decel stop
    std::thread canceller([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        iad_axis_cancel_latch(ax);
    });
    const double t0 = steady_now_s();
    CHECK_EQ(iad_axis_move_to(ax, 100.0, &mo, nullptr), IAD_ERR_CANCELLED);
    canceller.join();
    CHECK(steady_now_s() - t0 < 5.0);
    CHECK_EQ(iad_axis_cancel_reset(ax), IAD_OK);
    mo.speed = 200.0;
    CHECK_EQ(iad_axis_move_to(ax, 10.0, &mo, &st), IAD_OK);  // later calls work after the reset
    CHECK_NEAR(st.position_mm, 10.0, 0.11);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi03_log_level_and_callback_error) {
    CHECK_EQ(iad_set_log_level(7), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_set_log_level(IAD_LOG_ERROR), IAD_OK);
    CHECK_EQ(iad_set_log_level(IAD_LOG_DEBUG), IAD_OK);
    CHECK_EQ(iad_callback_error(nullptr), IAD_OK);  // outside a callback: no effect
}

// ============================================================================================ 0.4

namespace {

Json bus_info(iad_bus* bus) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_bus_info_json(bus, b, s, r); }));
}

Json monitor_info(iad_axis* ax) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_axis_monitor_info_json(ax, b, s, r); }));
}

bool eventually(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double end = steady_now_s() + timeout_s;
    while (steady_now_s() < end) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

/// The registers of each FC03 request a sink saw (-1 for another request).
std::vector<int> read_sizes(const Frames& frames) {
    std::vector<int> out;
    for (const auto& f : frames.list)
        if (f.first == IAD_FRAME_TX) out.push_back(f.second[1] == 0x03 ? ((f.second[4] << 8) | f.second[5]) : -1);
    return out;
}

/// A preemptible read's gate: counts its calls and ends the read after `stop_after` of them (-1: never).
struct Gate {
    std::atomic<int> calls{0};
    int stop_after = -1;
    static int32_t IAD_CALL fn(void* user) {
        auto* g = static_cast<Gate*>(user);
        const int n = ++g->calls;
        if (g->stop_after >= 0 && n > g->stop_after) {
            iad_callback_error("STOP pressed (test)");
            return 1;
        }
        return 0;
    }
};

/// Answers every 0x48 frame before the simulated unit sees it: [slave] 48 [rejection u16] [1, 2, 3, 4 as u32].
struct Iai2Hook {
    std::vector<std::uint8_t> last;
    int rejection = 0;
    static int32_t IAD_CALL fn(const uint8_t* request, size_t size, uint8_t* reply, size_t, size_t* length,
                               void* user) {
        auto* h = static_cast<Iai2Hook*>(user);
        if (size < 2 || request[1] != 0x48) return IAD_SIM_PASS;
        h->last.assign(request, request + size);
        std::vector<std::uint8_t> frame = {request[0], 0x48, static_cast<std::uint8_t>(h->rejection >> 8),
                                           static_cast<std::uint8_t>(h->rejection & 0xFF)};
        for (std::uint8_t v = 1; v <= 4; ++v) frame.insert(frame.end(), {0, 0, 0, v});
        append_crc(frame);
        std::memcpy(reply, frame.data(), frame.size());
        *length = frame.size();
        return IAD_SIM_REPLY;
    }
};

}  // namespace

TEST(capi04_bus_lock_across_calls) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    iad_bool locked = IAD_FALSE;
    CHECK_EQ(iad_bus_unlock(bus), IAD_ERR_INVALID_ARGUMENT);  // nothing held
    CHECK_EQ(iad_bus_lock(bus, -1.0, &locked), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_lock(bus, 0.0, &locked), IAD_OK);  // free: taken at once
    CHECK_EQ(locked, IAD_TRUE);
    CHECK_EQ(iad_bus_lock(bus, kNaN, &locked), IAD_OK);  // recursive
    CHECK_EQ(bus_info(bus).find("lock_depth")->as_int(), 2LL);
    uint16_t words[4] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 4, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);  // the holder's own requests
    std::atomic<bool> other_done{false};
    std::atomic<int> other_result{1};
    std::thread other([&] {
        uint16_t w[1];
        other_result = iad_bus_read_registers(bus, 1, 0x9000, 1, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, w);
        other_done = true;
    });
    REQUIRE(eventually([&] { return bus_info(bus).find("lock_waiting")->find("normal")->as_int() == 1; }));
    iad_bool timed = IAD_TRUE;
    iad_result timed_result = IAD_ERR_INTERNAL;
    double waited = 0.0;
    std::thread third([&] {  // a timed hold gives up while the bus is held
        const double t0 = steady_now_s();
        timed_result = iad_bus_lock(bus, 0.1, &timed);
        waited = steady_now_s() - t0;
    });
    third.join();
    CHECK_EQ(timed_result, IAD_OK);
    CHECK_EQ(timed, IAD_FALSE);
    CHECK(waited >= 0.09 && waited < 2.0);
    CHECK_EQ(iad_bus_unlock(bus), IAD_OK);
    CHECK(!other_done.load());  // still held once
    CHECK_EQ(iad_bus_unlock(bus), IAD_OK);
    other.join();
    CHECK_EQ(other_result.load(), static_cast<int>(IAD_OK));
    CHECK_EQ(bus_info(bus).find("lock_depth")->as_int(), 0LL);
    // a closed bus: refused on another handle of it
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, nullptr, &ax), IAD_OK);
    iad_bus* borrowed = nullptr;
    CHECK_EQ(iad_axis_bus(ax, &borrowed), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_bus_lock(borrowed, kNaN, &locked), IAD_ERR_TRANSPORT);
    CHECK_EQ(locked, IAD_FALSE);
    CHECK_EQ(iad_bus_close(borrowed), IAD_OK);
    iad_axis_close(ax);  // its bus is closed already
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi04_preemptible_read_scope) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    iad_sim* sim2 = open_sim("{}");
    iad_bus* other = bus_on_sim(sim2);
    Frames frames;
    Frames other_frames;
    CHECK_EQ(iad_bus_set_frame_sink(bus, &Frames::sink, &frames), IAD_OK);
    CHECK_EQ(iad_bus_set_frame_sink(other, &Frames::sink, &other_frames), IAD_OK);
    CHECK_EQ(iad_bus_preemptible_begin(bus, 126, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_preemptible_end(bus), IAD_ERR_INVALID_ARGUMENT);  // none begun
    Gate gate;
    CHECK_EQ(iad_bus_preemptible_begin(bus, 0, &Gate::fn, &gate), IAD_OK);
    CHECK_EQ(bus_info(bus).find("preemptible_words")->as_int(), 32LL);
    CHECK(bus_info(other).find("preemptible_words")->is_null());
    std::vector<uint16_t> words(34);  // the status block 0x9000-0x9021
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 34, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words.data()), IAD_OK);
    CHECK_EQ(gate.calls.load(), 2);  // the gate before every request
    CHECK_EQ(iad_bus_write_coil(bus, 1, reg::kCoilStop, IAD_TRUE, IAD_INT_UNKNOWN), IAD_ERR_UNSAFE_REFUSED);
    CHECK(has(last_json(), "preemptible"));
    CHECK_EQ(iad_bus_broadcast_coil(bus, reg::kCoilStop, IAD_TRUE), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(iad_bus_read_registers(other, 1, 0x9000, 34, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words.data()), IAD_OK);  // as usual
    CHECK_EQ(iad_bus_write_coil(other, 1, reg::kCoilStop, IAD_FALSE, IAD_INT_UNKNOWN), IAD_OK);
    CHECK_EQ(iad_bus_preemptible_end(other), IAD_ERR_INVALID_ARGUMENT);  // the innermost scope is on `bus`
    gate.stop_after = gate.calls.load() + 1;  // one more request, then STOP
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 34, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words.data()), IAD_ERR_CANCELLED);
    CHECK(has(last_json(), "STOP pressed (test)"));
    CHECK_EQ(iad_bus_preemptible_end(bus), IAD_OK);
    CHECK_EQ(iad_bus_preemptible_end(bus), IAD_ERR_INVALID_ARGUMENT);
    CHECK(bus_info(bus).find("preemptible_words")->is_null());
    CHECK_EQ(iad_bus_write_coil(bus, 1, reg::kCoilStop, IAD_FALSE, IAD_INT_UNKNOWN), IAD_OK);  // writes again
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 34, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words.data()), IAD_OK);
    CHECK(read_sizes(frames) == (std::vector<int>{32, 2, 32, -1, 34}));
    CHECK(read_sizes(other_frames) == (std::vector<int>{34, -1}));
    // a scope ended through the handle that began it, after that handle was closed
    iad_bus* borrowed = nullptr;
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, nullptr, &ax), IAD_OK);
    CHECK_EQ(iad_axis_bus(ax, &borrowed), IAD_OK);
    CHECK_EQ(iad_bus_preemptible_begin(borrowed, 8, nullptr, nullptr), IAD_OK);
    CHECK_EQ(bus_info(bus).find("preemptible_words")->as_int(), 8LL);  // any handle on the same bus sees it
    CHECK_EQ(iad_bus_close(borrowed), IAD_OK);
    CHECK_EQ(iad_bus_preemptible_end(borrowed), IAD_OK);
    CHECK(bus_info(bus).find("preemptible_words")->is_null());
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_bus_close(other), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim2), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi04_suspend_monitors_of_a_bus) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    int32_t count = -1;
    CHECK_EQ(iad_bus_suspend_monitors(bus, kNaN, &count), IAD_OK);  // no monitor: nothing held
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_bus_resume_monitors(bus), IAD_OK);
    CHECK_EQ(iad_bus_resume_monitors(bus), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_suspend_monitors(bus, -1.0, &count), IAD_ERR_INVALID_ARGUMENT);
    const iad_axis_options ao = quick_axis();
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, &ao, &ax), IAD_OK);
    CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_start_monitor(ax, nullptr), IAD_OK);
    auto polls = [&] { return monitor_info(ax).find("polls")->as_int(); };
    REQUIRE(eventually([&] { return polls() >= 3; }));
    CHECK_EQ(iad_bus_suspend_monitors(bus, 1.0, &count), IAD_OK);
    CHECK_EQ(count, 1);
    CHECK_EQ(bus_info(bus).find("suspended_monitors")->as_int(), 1LL);
    const long long held = polls();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    CHECK(polls() <= held + 1);  // no polls while suspended (one in flight at most)
    CHECK_EQ(iad_bus_resume_monitors(bus), IAD_OK);
    CHECK_EQ(bus_info(bus).find("suspended_monitors")->as_int(), 0LL);
    REQUIRE(eventually([&] { return polls() > held + 2; }));
    // a suspension kept on a handle ends with the handle
    iad_bus* borrowed = nullptr;
    CHECK_EQ(iad_axis_bus(ax, &borrowed), IAD_OK);
    CHECK_EQ(iad_bus_suspend_monitors(borrowed, 1.0, &count), IAD_OK);
    CHECK_EQ(count, 1);
    const long long held2 = polls();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CHECK(polls() <= held2 + 1);
    CHECK_EQ(iad_bus_close(borrowed), IAD_OK);  // releases the handle and its suspension
    REQUIRE(eventually([&] { return polls() > held2 + 2; }));
    CHECK_EQ(iad_axis_stop_monitor(ax, kNaN), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi04_iai_flags_0x48_and_broadcasts) {
    iad_sim* sim = open_sim("{}");
    Iai2Hook hook;
    CHECK_EQ(iad_sim_set_request_hook(sim, &Iai2Hook::fn, &hook), IAD_OK);
    iad_bus* bus = bus_on_sim(sim);
    int32_t rejection = -1;
    uint32_t status = 0;
    // 0x45 with flags: the never-send list needs its own opt-in; an unknown bit is refused; nothing sent
    const uint32_t baud[2] = {0x14, 5};
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x04, baud, 2, IAD_IAI_ALLOW_UNSAFE, 0, &rejection, &status, 1),
             IAD_ERR_UNSAFE_REFUSED);
    CHECK(has(last_json(), "never-send"));
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x0B, nullptr, 0, 0x100, 0, &rejection, &status, 1),
             IAD_ERR_UNSUPPORTED);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0, 0, 0x0B, nullptr, 0, IAD_IAI_NO_CHECK, 0, &rejection, &status, 1),
             IAD_OK);  // 0: 0x45
    CHECK_EQ(rejection, 0);  // decel stop at rest: accepted
    // (0.8) the function code and the operands of the one function: another code is refused, nothing sent
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x46, 0, 0x0B, nullptr, 0, 0, 0, &rejection, &status, 1),
             IAD_ERR_UNSUPPORTED);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x0B, baud, 3, 0, 0, &rejection, &status, 1),
             IAD_ERR_INVALID_ARGUMENT);  // 0x45 takes two operands
    const uint32_t wide[1] = {0x10000};
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x0B, wide, 1, 0, 0, &rejection, &status, 1),
             IAD_ERR_INVALID_ARGUMENT);  // a 0x45 operand is 16 bits
    // 0x48: the frame, the reply, a rejection
    const uint32_t ops[1] = {1};
    uint32_t words[4] = {0, 0, 0, 0};
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, ops, 1, 0, IAD_INT_UNKNOWN, &rejection, words, 4), IAD_OK);
    CHECK_EQ(rejection, 0);
    CHECK(words[0] == 1u && words[1] == 2u && words[2] == 3u && words[3] == 4u);
    std::vector<std::uint8_t> want = {0x01, 0x48, 0x00, 0x00, 0x30, 0, 0, 0, 1};
    want.resize(21, 0);
    append_crc(want);
    CHECK(hook.last == want);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x7777, nullptr, 0, 0, 0, &rejection, words, 4), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x0010, nullptr, 0, IAD_IAI_ALLOW_UNSAFE, 0, &rejection, words, 4),
             IAD_ERR_UNSAFE_REFUSED);  // never-send: allow_unsafe is not enough
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, nullptr, 5, 0, 0, &rejection, words, 4), IAD_ERR_INVALID_ARGUMENT);
    hook.rejection = 0x1005;
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, ops, 1, 0, 0, &rejection, words, 4), IAD_ERR_COMMAND_REJECTED);
    CHECK_EQ(Json::parse(last_json()).find("rejection")->as_int(), 0x1005LL);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, ops, 1, IAD_IAI_NO_CHECK, 0, &rejection, nullptr, 4), IAD_OK);
    CHECK_EQ(rejection, 0x1005);
    // broadcasts: the frame sent, a buffer too small refused before sending, never a read
    uint8_t frame[16] = {0};
    size_t length = 0;
    CHECK_EQ(iad_bus_broadcast_iai_command(bus, 0, 0x09, 0, 0, 0, 0.0, frame, sizeof frame, &length), IAD_OK);
    std::vector<std::uint8_t> servo_off = {0x00, 0x45, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00};
    append_crc(servo_off);
    CHECK(length == 10u && std::vector<std::uint8_t>(frame, frame + length) == servo_off);
    CHECK_EQ(iad_bus_broadcast_iai_command(bus, 0, 0x04, 0x14, 5, IAD_IAI_ALLOW_UNSAFE, 0.0, nullptr, 0, nullptr),
             IAD_ERR_UNSAFE_REFUSED);  // the baud change needs IAD_IAI_ALLOW_NEVER_SEND
    const uint8_t stop_on[5] = {0x05, 0x04, 0x2C, 0xFF, 0x00};
    CHECK_EQ(iad_bus_broadcast(bus, stop_on, sizeof stop_on, kNaN, frame, 4, &length), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(length, 8u);
    CHECK(!state(sim).find("stop_held")->as_bool());  // nothing sent
    CHECK_EQ(iad_bus_broadcast(bus, stop_on, sizeof stop_on, kNaN, frame, sizeof frame, &length), IAD_OK);
    CHECK(length == 8u && frame[0] == 0x00 && frame[6] == 0x4D && frame[7] == 0x12);  // 00 05 04 2C FF 00 4D 12
    CHECK(state(sim).find("stop_held")->as_bool());
    const uint8_t stop_off[5] = {0x05, 0x04, 0x2C, 0x00, 0x00};
    CHECK_EQ(iad_bus_broadcast(bus, stop_off, sizeof stop_off, 0.0, nullptr, 0, nullptr), IAD_OK);
    const uint8_t read[5] = {0x03, 0x90, 0x00, 0x00, 0x01};
    CHECK_EQ(iad_bus_broadcast(bus, read, sizeof read, 0.0, nullptr, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi04_line_on_a_bus_and_response_timing) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    auto line_json = [&] {
        return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_bus_line_json(bus, b, s, r); }));
    };
    CHECK_EQ(line_json().find("kind")->as_string(), std::string("direct"));
    const uint8_t fc03_1[5] = {0x03, 0x90, 0x00, 0x00, 0x01};
    double direct_s = 0.0;
    CHECK_EQ(iad_bus_response_timeout(bus, 1, fc03_1, sizeof fc03_1, 7, &direct_s), IAD_OK);
    CHECK_NEAR(direct_s, 0.2, 1e-9);  // the 0.2 s floor
    // a stated RCON line: the guard refuses broadcasts and IAI commands, the timeout floor is 0.3 s
    CHECK_EQ(iad_bus_set_line(bus, IAD_LINE_AUTO, kNaN), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_line(bus, 9, kNaN), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_line(bus, IAD_LINE_RCON, 0.005), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_line(bus, IAD_LINE_RCON, 0.001), IAD_OK);
    const Json rcon = line_json();
    CHECK_EQ(rcon.find("kind")->as_string(), std::string("rcon"));
    CHECK_EQ(rcon.find("detected")->as_bool(), false);
    CHECK_NEAR(rcon.find("position_units")->find("all")->find("unit_mm")->as_number(), 0.001, 1e-12);
    CHECK_EQ(bus_info(bus).find("line")->as_string(), std::string("rcon"));
    CHECK_EQ(iad_bus_broadcast_coil(bus, reg::kCoilStop, IAD_TRUE), IAD_ERR_UNSAFE_REFUSED);
    CHECK(has(last_json(), "LineRuleError"));
    int32_t rejection = 0;
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, nullptr, 0, 0, 0, &rejection, nullptr, 4), IAD_ERR_UNSAFE_REFUSED);
    double rcon_s = 0.0;
    CHECK_EQ(iad_bus_response_timeout(bus, 1, fc03_1, sizeof fc03_1, 7, &rcon_s), IAD_OK);
    CHECK_NEAR(rcon_s, 0.3, 1e-9);
    // probed: nothing answers D0h or 3Fh on the simulator, so the line is direct again
    int32_t found = -1;
    CHECK_EQ(iad_bus_detect_line(bus, IAD_LINE_AUTO, 0.02, kNaN, &found), IAD_OK);
    CHECK_EQ(found, IAD_LINE_DIRECT);
    CHECK_EQ(line_json().find("kind")->as_string(), std::string("direct"));
    CHECK_EQ(iad_bus_detect_line(bus, IAD_LINE_ROBONET, kNaN, kNaN, &found), IAD_OK);  // through mode: no probe
    CHECK_EQ(found, IAD_LINE_ROBONET);
    CHECK_EQ(iad_bus_set_line(bus, IAD_LINE_DIRECT, kNaN), IAD_OK);
    // the response delay RTIM
    double delay = 0.0;
    CHECK_EQ(iad_bus_response_delay(bus, 1, &delay), IAD_OK);
    CHECK_NEAR(delay, 5.0, 1e-12);
    std::vector<std::uint8_t> fc03_60 = {0x03, 0x90, 0x00, 0x00, 0x3C};
    double base = 0.0;
    CHECK_EQ(iad_bus_response_timeout(bus, 1, fc03_60.data(), fc03_60.size(), 125, &base), IAD_OK);
    CHECK_EQ(iad_bus_set_response_delay(bus, 1, 405.0), IAD_OK);
    CHECK_EQ(iad_bus_response_delay(bus, 1, &delay), IAD_OK);
    CHECK_NEAR(delay, 405.0, 1e-12);
    double slower = 0.0;
    CHECK_EQ(iad_bus_response_timeout(bus, 1, fc03_60.data(), fc03_60.size(), 125, &slower), IAD_OK);
    CHECK(slower > base + 0.2);
    CHECK_EQ(iad_bus_set_response_delay(bus, 1, -1.0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_response_delay(bus, 1, 1001.0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_response_delay(bus, 0, &delay), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_response_timeout(bus, 1, fc03_1, sizeof fc03_1, 300, &direct_s), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi04_per_call_read_options_and_baud_targets) {
    iad_sim* sim = open_sim(R"({"baud_rate": 38400, "unit_baud_rate": 115200})");
    iad_bus* bus = bus_on_sim(sim);
    Frames frames;
    CHECK_EQ(iad_bus_set_frame_sink(bus, &Frames::sink, &frames), IAD_OK);
    // baud detection over targets: a silent slave, the RCON gateway probe, then slave 1
    int32_t rate = 0;
    CHECK_EQ(iad_bus_detect_baud_rate(bus, "9, 0xD0@0x3700, 1", "115200,38400", IAD_INT_UNKNOWN, IAD_TRUE, 0.02,
                                         &rate),
             IAD_OK);
    CHECK_EQ(rate, 115200);
    bool gateway_probe = false;
    for (const auto& f : frames.list)
        if (f.first == IAD_FRAME_TX && f.second[0] == 0xD0 && f.second[2] == 0x37 && f.second[3] == 0x00)
            gateway_probe = true;
    CHECK(gateway_probe);
    CHECK_EQ(iad_bus_detect_baud_rate(bus, "0xD0@0x3700", "38400", IAD_INT_UNKNOWN, IAD_FALSE, 0.02, &rate),
             IAD_ERR_MODBUS_TIMEOUT);
    CHECK(has(last_json(), "slave 0xD0 did not answer"));
    CHECK_EQ(iad_bus_detect_baud_rate(bus, "1@x", nullptr, IAD_INT_UNKNOWN, IAD_TRUE, kNaN, &rate),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_detect_baud_rate(bus, nullptr, nullptr, IAD_INT_UNKNOWN, IAD_TRUE, kNaN, &rate), IAD_OK);
    CHECK_EQ(rate, 115200);  // NULL: slave 1, at the present rate first
    // reads in chunks, a capped wait, FC11 with the bus's retries
    frames.list.clear();
    uint16_t words[34] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 34, IAD_INT_UNKNOWN, kNaN, 10, words), IAD_OK);
    CHECK(read_sizes(frames) == (std::vector<int>{10, 10, 10, 4}));
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 34, 0, kNaN, 126, words), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, -1.0, IAD_INT_UNKNOWN, words), IAD_ERR_INVALID_ARGUMENT);
    const double t0 = steady_now_s();
    CHECK_EQ(iad_bus_read_registers(bus, 9, 0x9000, 1, 0, 0.02, IAD_INT_UNKNOWN, words), IAD_ERR_MODBUS_TIMEOUT);
    CHECK(steady_now_s() - t0 < 1.0);
    uint8_t id[64];
    size_t length = 0;
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, IAD_INT_UNKNOWN, kNaN, id, sizeof id, &length), IAD_OK);
    CHECK(length >= 3);
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, 0, 0.5, id, 1, &length), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, 0, -1.0, id, sizeof id, &length), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

// ============================================================================================ 0.5 (core-bus)

namespace {

/// The binding's port of the 0.5 tests: a Host that also waits on a read timeout of its own (as a serial port).
struct TimedHost : Host {
    double timeout = 0.2;
    std::vector<double> at_write;  // the port's timeout at each write
    std::vector<double> set;       // each set_read_timeout
    bool fail_set = false;

    static TimedHost* of(void* user) { return static_cast<TimedHost*>(user); }
    static int32_t IAD_CALL write(void* user, const uint8_t* data, size_t size) {
        of(user)->at_write.push_back(of(user)->timeout);
        return Host::write(static_cast<Host*>(of(user)), data, size);
    }
    static int32_t IAD_CALL read(void* user, uint8_t* buffer, size_t size, double timeout_s, size_t* got) {
        return Host::read(static_cast<Host*>(of(user)), buffer, size, timeout_s, got);
    }
    static int32_t IAD_CALL reset_input(void* user) { return Host::reset_input(static_cast<Host*>(of(user))); }
    static void IAD_CALL close(void* user) { Host::close(static_cast<Host*>(of(user))); }
    static void IAD_CALL release(void* user) { Host::release(static_cast<Host*>(of(user))); }
    static int32_t IAD_CALL set_read_timeout(void* user, double seconds) {
        TimedHost* h = of(user);
        if (h->fail_set) {
            iad_callback_error("the timeout is fixed (test)");
            return -1;
        }
        h->set.push_back(seconds);
        h->timeout = seconds;
        return 0;
    }
};

/// Callbacks on a TimedHost; `whole`: struct_size of this header (set_read_timeout is read), else a 0.3 caller's.
iad_transport_callbacks timed_callbacks(bool whole) {
    iad_transport_callbacks cb;
    std::memset(&cb, 0, sizeof cb);
    if (whole) cb.struct_size = static_cast<uint32_t>(sizeof cb);
    CHECK_EQ(iad_transport_callbacks_init(&cb), IAD_OK);
    cb.write = &TimedHost::write;
    cb.read = &TimedHost::read;
    cb.reset_input = &TimedHost::reset_input;
    cb.close = &TimedHost::close;
    cb.release = &TimedHost::release;
    cb.set_read_timeout = &TimedHost::set_read_timeout;  // past struct_size for a 0.3 caller: never read
    return cb;
}

/// Answers every 0x45 frame with rejection 0x21 and status byte 0x07.
int32_t IAD_CALL reject_45(const uint8_t* request, size_t size, uint8_t* reply, size_t, size_t* length, void*) {
    if (size < 2 || request[1] != 0x45) return IAD_SIM_PASS;
    std::vector<std::uint8_t> frame = {request[0], 0x45, 0x21, 0x07};
    append_crc(frame);
    std::memcpy(reply, frame.data(), frame.size());
    *length = frame.size();
    return IAD_SIM_REPLY;
}

}  // namespace

TEST(capi05_transport_set_read_timeout) {
    // a caller that does not say it has the 0.5 struct gets the 0.3 part filled, nothing past it
    iad_transport_callbacks old;
    std::memset(&old, 0xAB, sizeof old);
    old.struct_size = 0;
    CHECK_EQ(iad_transport_callbacks_init(&old), IAD_OK);
    CHECK(old.struct_size >= offsetof(iad_transport_callbacks, set_read_timeout) && old.struct_size <= sizeof old);
    if (sizeof(void*) == 8) {
        CHECK_EQ(old.struct_size, 96u);
        const auto* tail = reinterpret_cast<const unsigned char*>(&old.set_read_timeout);
        CHECK(tail[0] == 0xAB && tail[7] == 0xAB);  // past a 0.3 caller's struct: untouched
    }
    TimedHost host;
    iad_transport_callbacks cb = timed_callbacks(true);
    iad_transport* t = nullptr;
    CHECK_EQ(iad_transport_open_callbacks(&cb, "TIMED", &host, &t), IAD_OK);
    iad_bus* bus = nullptr;
    iad_bus_options o;
    iad_bus_options_init(&o);
    o.min_gap_s = 0.0;
    o.retries = 0;
    CHECK_EQ(iad_bus_open(t, &o, &bus), IAD_OK);
    // a probe shortens the port's timeout before the request goes out and puts it back after
    iad_bool answered = IAD_FALSE;
    CHECK_EQ(iad_bus_probe(bus, 1, 0.03, 0x9000, &answered), IAD_OK);
    CHECK(answered == IAD_TRUE);
    REQUIRE(!host.at_write.empty());
    CHECK_NEAR(host.at_write.back(), 0.03, 1e-12);
    CHECK(host.set == (std::vector<double>{0.03, 0.2}));
    CHECK_NEAR(host.timeout, 0.2, 1e-12);
    CHECK_EQ(iad_bus_probe(bus, 9, 0.03, 0x9000, &answered), IAD_OK);  // silence: the same, and put back too
    CHECK(answered == IAD_FALSE);
    CHECK_NEAR(host.at_write.back(), 0.03, 1e-12);
    CHECK_NEAR(host.timeout, 0.2, 1e-12);
    uint16_t word = 0;
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, 0.05, IAD_INT_UNKNOWN, &word), IAD_OK);  // a capped read
    CHECK_NEAR(host.at_write.back(), 0.05, 1e-12);
    CHECK_NEAR(host.timeout, 0.2, 1e-12);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, &word), IAD_OK);  // a plain read leaves it alone
    CHECK_NEAR(host.at_write.back(), 0.2, 1e-12);
    // set on the transport handle; a failing callback leaves the timeout as it was
    CHECK_EQ(iad_transport_set_read_timeout(t, 0.5), IAD_OK);
    CHECK_NEAR(host.timeout, 0.5, 1e-12);
    const Json info =
        Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_transport_info_json(t, b, s, r); }));
    CHECK_NEAR(info.find("read_timeout_s")->as_number(), 0.5, 1e-12);
    host.fail_set = true;
    CHECK_EQ(iad_transport_set_read_timeout(t, 0.1), IAD_ERR_TRANSPORT);
    CHECK(has(last_json(), "the timeout is fixed (test)"));
    CHECK_EQ(iad_bus_probe(bus, 1, 0.03, 0x9000, &answered), IAD_OK);  // the probe still runs, on the port's timeout
    CHECK(answered == IAD_TRUE);
    CHECK_NEAR(host.at_write.back(), 0.5, 1e-12);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    // a 0.3 caller: the field past its struct_size is never read, the bus keeps the timeout to itself
    TimedHost host03;
    iad_transport_callbacks cb03 = timed_callbacks(false);
    CHECK_EQ(iad_transport_open_callbacks(&cb03, "OLD", &host03, &t), IAD_OK);
    bus = bus_on(t);
    CHECK_EQ(iad_bus_probe(bus, 1, 0.03, 0x9000, &answered), IAD_OK);
    CHECK(answered == IAD_TRUE);
    CHECK(host03.set.empty());
    CHECK_NEAR(host03.at_write.back(), 0.2, 1e-12);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}

TEST(capi05_bus_shutdown_and_release) {
    // shutdown: closed as ModbusRtu::close, the handle kept
    Host host;
    iad_transport_callbacks cb;
    std::memset(&cb, 0, sizeof cb);
    iad_transport_callbacks_init(&cb);
    cb.real_timeout = IAD_FALSE;
    cb.write = &Host::write;
    cb.read = &Host::read;
    cb.reset_input = &Host::reset_input;
    cb.close = &Host::close;
    cb.release = &Host::release;
    iad_transport* t = nullptr;
    CHECK_EQ(iad_transport_open_callbacks(&cb, "HOST", &host, &t), IAD_OK);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(t, nullptr, &bus), IAD_OK);
    uint16_t word = 0;
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, &word), IAD_OK);
    CHECK_EQ(iad_bus_shutdown(bus), IAD_OK);
    CHECK_EQ(host.closes, 1);
    CHECK(bus_info(bus).find("closed")->as_bool());  // the handle is still there
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, &word), IAD_ERR_TRANSPORT);
    CHECK_EQ(iad_bus_shutdown(bus), IAD_OK);  // twice: fine, closed once
    CHECK_EQ(iad_bus_shutdown(nullptr), IAD_OK);
    CHECK_EQ(host.closes, 1);
    CHECK_EQ(iad_bus_release(reinterpret_cast<iad_bus*>(t)), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "another kind of handle"));
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_bus_shutdown(bus), IAD_ERR_INVALID_ARGUMENT);  // a released handle
    CHECK_EQ(host.closes, 1);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(host.releases.load(), 1);

    // release: a bus of iad_bus_open let go without closing its transport
    Host kept;
    CHECK_EQ(iad_transport_open_callbacks(&cb, "KEPT", &kept, &t), IAD_OK);
    bus = bus_on(t);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kNaN, IAD_INT_UNKNOWN, &word), IAD_OK);
    const iad_axis_options ao = quick_axis();
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, &ao, &ax), IAD_OK);
    CHECK_EQ(iad_bus_release(bus), IAD_OK);
    CHECK_EQ(iad_bus_release(bus), IAD_OK);  // twice: fine
    CHECK_EQ(iad_bus_release(nullptr), IAD_OK);
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);  // the axis keeps the bus and the transport
    CHECK_EQ(kept.closes, 0);
    CHECK_EQ(kept.releases.load(), 0);
    iad_bus* again = nullptr;
    CHECK_EQ(iad_axis_bus(ax, &again), IAD_OK);
    CHECK_EQ(iad_bus_release(again), IAD_OK);
    CHECK_EQ(kept.closes, 0);

    // shutdown stops the bus's status monitors; reopen brings the bus back
    iad_sim* sim = open_sim("{}");
    iad_bus* simbus = bus_on_sim(sim);
    iad_axis* mon = nullptr;
    CHECK_EQ(iad_axis_open_bus(simbus, 1, &ao, &mon), IAD_OK);
    CHECK_EQ(iad_axis_connect(mon, IAD_TRUE, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_start_monitor(mon, nullptr), IAD_OK);
    REQUIRE(eventually([&] { return monitor_info(mon).find("polls")->as_int() >= 2; }));
    CHECK_EQ(iad_bus_shutdown(simbus), IAD_OK);
    const Json after = monitor_info(mon);
    CHECK(after.is_null() || !after.find("running")->as_bool());
    CHECK_EQ(iad_axis_status(mon, &st), IAD_ERR_TRANSPORT);
    CHECK_EQ(iad_bus_reopen(simbus, 0), IAD_OK);
    CHECK_EQ(iad_axis_status(mon, &st), IAD_OK);
    CHECK_EQ(iad_axis_close(mon), IAD_OK);
    CHECK_EQ(iad_bus_close(simbus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK(eventually([&] { return kept.releases.load() == 1; }));  // the last owner let go
}

TEST(capi05_rejection_status_in_the_error) {
    iad_sim* sim = open_sim("{}");
    CHECK_EQ(iad_sim_set_request_hook(sim, &reject_45, nullptr), IAD_OK);
    iad_bus* bus = bus_on_sim(sim);
    int32_t rejection = -1;
    uint32_t status = 0;
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x0B, nullptr, 0, 0, 0, &rejection, &status, 1),
             IAD_ERR_COMMAND_REJECTED);
    Json j = Json::parse(last_json());
    CHECK_EQ(j.find("rejection")->as_int(), 0x21LL);
    CHECK_EQ(j.find("status")->as_int(), 0x07LL);
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x45, 0, 0x0B, nullptr, 0, IAD_IAI_NO_CHECK, 0, &rejection, &status, 1),
             IAD_OK);
    CHECK(rejection == 0x21 && status == 0x07u);  // the same reply, unchecked
    Iai2Hook hook;
    hook.rejection = 0x1005;
    CHECK_EQ(iad_sim_set_request_hook(sim, &Iai2Hook::fn, &hook), IAD_OK);
    const uint32_t ops[1] = {1};
    uint32_t words[4] = {0, 0, 0, 0};
    CHECK_EQ(iad_bus_iai_command(bus, 1, 0x48, 0, 0x30, ops, 1, 0, 0, &rejection, words, 4), IAD_ERR_COMMAND_REJECTED);
    j = Json::parse(last_json());
    const Json* got = j.find("status");
    REQUIRE(got != nullptr && got->size() == 4u);
    for (std::size_t i = 0; i < 4; ++i) CHECK_EQ(got->as_array()[i].as_int(), static_cast<long long>(i + 1));
    CHECK_EQ(j.find("hint")->as_string(), std::string("turn the servo on first"));  // 0x1005: as 0x45's 0x23
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}
