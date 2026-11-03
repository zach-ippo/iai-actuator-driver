// C ABI 0.5 additions of the simulators (include/iai_actuator_driver.h "simulators", src/c_api_sim.cpp):
// iad_sim_set_clock (a clock of the unit's own, set after iad_sim_open), the new iad_sim_open options, the state keys
// and the controls set_state / set_option / set_inputs / set_parameter_word; 0.6 ("simulators: ports, gateway lines"):
// the gateway lines, the port for a bus, the state keys and a failing request hook. Simulator only: no COM port is
// opened.
#include <cstring>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/crc.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

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

iad_result control(iad_sim* sim, const char* command) {
    char buffer[256];
    std::size_t need = 0;
    return iad_sim_control_json(sim, command, buffer, sizeof buffer, &need);
}

iad_sim* open_sim(const std::string& options) {
    iad_sim* sim = nullptr;
    CHECK_EQ(iad_sim_open(options.c_str(), &sim), IAD_OK);
    REQUIRE(sim != nullptr);
    return sim;
}

double fake_now(void* user) { return *static_cast<double*>(user); }

}  // namespace

TEST(capi05_sim_set_clock_runs_the_unit_on_a_clock_of_its_own) {
    iad_sim* sim = open_sim(R"({"settle_speed_mm_s": 0})");
    double fake = 50.0;
    const double before = state(sim).find("clock_s")->as_number();
    CHECK_EQ(iad_sim_set_clock(sim, &fake_now, &fake), IAD_OK);
    CHECK_NEAR(state(sim).find("clock_s")->as_number(), before, 0.05);  // no jump
    // a 0x9900 move: the fake clock stands still, so does the axis
    const uint8_t pdu[] = {0x10, 0x99, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x00, 0x20, 0xCB, 0x00, 0x00,
                           0x00, 0x0A, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x1E};  // 83.95 mm, 0.10, 10 mm/s, 0.30 G
    std::vector<std::uint8_t> request = build_frame(1, std::vector<std::uint8_t>(pdu, pdu + sizeof pdu));
    uint8_t answer[64];
    std::size_t length = 0;
    CHECK_EQ(iad_sim_handle_frame(sim, request.data(), request.size(), answer, sizeof answer, &length), IAD_OK);
    CHECK_EQ(length, 8u);
    CHECK_NEAR(state(sim).find("position_mm")->as_number(), 73.95, 1e-9);
    fake += 2.0;
    const Json s = state(sim);
    CHECK(!s.find("moving")->as_bool());
    CHECK_NEAR(s.find("position_mm")->as_number(), 83.95, 1e-9);
    CHECK_EQ(iad_sim_set_clock(sim, nullptr, nullptr), IAD_OK);  // the library's clock again
    CHECK_NEAR(state(sim).find("clock_s")->as_number(), before + 2.0, 0.05);
    CHECK_EQ(iad_sim_set_clock(nullptr, &fake_now, &fake), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi05_sim_options_and_controls) {
    iad_sim* sim = nullptr;
    CHECK_EQ(iad_sim_open(R"({"rtc": true, "function_codes": [3, 6, 16], "parw_latches": false, "cold_boot": true,
                              "ec_first_son_s": 0.3, "auto_servo_off_s": 2.5, "broadcast": null})", &sim),
             IAD_OK);
    Json s = state(sim);
    CHECK(!s.find("servo_on")->as_bool());  // cold boot
    CHECK(!s.find("auto_servo_off_active")->as_bool());
    CHECK_NEAR(s.find("ls_range_mm")->as_number(), 0.10, 1e-12);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
    CHECK_EQ(iad_sim_open(R"({"function_codes": 3})", &sim), IAD_ERR_INVALID_ARGUMENT);
    CHECK(sim == nullptr);

    sim = open_sim("{}");
    CHECK_EQ(control(sim, R"({"command": "set_state", "servo_on": false, "homed": false})"), IAD_OK);
    s = state(sim);
    CHECK(!s.find("servo_on")->as_bool() && !s.find("homed")->as_bool() && s.find("pmsl")->as_bool());
    CHECK_EQ(control(sim, R"({"command": "set_state", "dance": true})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "ls_range_mm", "value": 0.4})"), IAD_OK);
    CHECK_NEAR(state(sim).find("ls_range_mm")->as_number(), 0.4, 1e-12);
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "push_blip_mm", "value": null})"), IAD_OK);
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "profile", "value": "rod"})"),
             IAD_ERR_INVALID_ARGUMENT);  // a fixed fact
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "dance", "value": 1})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(control(sim, R"({"command": "set_inputs", "emg": true})"), IAD_OK);
    uint16_t dss1 = 0;
    CHECK_EQ(iad_sim_register_value(sim, 0x9005, &dss1), IAD_OK);
    CHECK_EQ(dss1 & 0x8000, 0x8000);  // EMGS
    CHECK_EQ(control(sim, R"({"command": "set_inputs", "emg": null, "drive_cut": false})"), IAD_OK);
    CHECK_EQ(control(sim, R"({"command": "set_parameter_word", "address": "0x0800", "value": 7, "nv": true})"), IAD_OK);
    s = state(sim);
    CHECK_EQ(s.find("parameter_nv")->find("2048")->as_int(), 7LL);
    CHECK_EQ(control(sim, R"({"command": "set_parameter_word", "address": 2049, "value": 70000})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

// ============================================================================================ 0.6 (core-sim-gw)

namespace {

std::string line_control(iad_sim_line* line, const char* command) {
    return read_text([&](char* b, size_t s, size_t* r) { return iad_sim_line_control_json(line, command, b, s, r); });
}

iad_bus* bus_on_line(iad_sim_line* line) {
    iad_transport* port = nullptr;
    CHECK_EQ(iad_sim_line_port(line, &port), IAD_OK);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(port, nullptr, &bus), IAD_OK);
    CHECK_EQ(iad_transport_release(port), IAD_OK);  // the bus keeps the port
    REQUIRE(bus != nullptr);
    return bus;
}

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

int32_t failing_hook(const uint8_t* request, size_t size, uint8_t* reply, size_t capacity, size_t* length, void*) {
    if (size > 1 && request[1] == 0x11) return IAD_SIM_FAIL;  // no text: the default one
    static const char why[] = "the cable is on fire";
    *length = sizeof why - 1 < capacity ? sizeof why - 1 : capacity;
    std::memcpy(reply, why, *length);
    return IAD_SIM_FAIL;
}

}  // namespace

TEST(capi06_sim_gateway_line_answers_and_its_controls) {
    iad_sim* one = open_sim(R"({"profile": "pcon", "slave": 1})");
    iad_sim* two = open_sim(R"({"profile": "pcon", "slave": 2})");
    iad_sim* units[] = {one, two};
    iad_sim_line* line = nullptr;
    CHECK_EQ(iad_sim_gateway_line_open("canbus", nullptr, units, 2, &line), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_gateway_line_open("direct", nullptr, units, 2, &line), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_gateway_line_open(nullptr, R"({"colour": 1})", units, 2, &line), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_gateway_line_open(nullptr, R"({"position_unit_mm": 0.1})", units, 2, &line),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(line == nullptr);
    CHECK_EQ(iad_sim_gateway_line_open(nullptr, R"({"name": "R", "linked_only": [5], "serials": {"2": "AB"},
                                                   "position_unit_mm": {"1": 0.001}})",
                                       units, 2, &line),
             IAD_OK);
    REQUIRE(line != nullptr);
    CHECK(has(line_control(line, R"({"command": "state"})"), "\"kind\":\"rcon\""));
    iad_bus* bus = bus_on_line(line);
    uint16_t status[11] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 0xD0, 0x3700, 11, 0, kUnset, IAD_INT_UNKNOWN, status), IAD_OK);
    CHECK_EQ(status[0], 0x8000);                             // GWSR: MANU
    CHECK_EQ(status[3], 0xFFFF & ~(1u | 2u | (1u << 4)));    // NLNK: axes 0, 1 and the linked-only slave 5
    uint16_t pnow[2] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, 0, kUnset, IAD_INT_UNKNOWN, pnow), IAD_OK);
    CHECK_EQ((pnow[0] << 16) | pnow[1], 60000);  // 60.00 mm in 0.001 mm on the wire
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9000, 2, 0, kUnset, IAD_INT_UNKNOWN, pnow), IAD_OK);
    CHECK_EQ((pnow[0] << 16) | pnow[1], 6000);
    uint16_t serial[1] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9400, 1, 0, kUnset, IAD_INT_UNKNOWN, serial), IAD_OK);
    CHECK_EQ(serial[0], 0x4142);  // "AB"
    CHECK_EQ(iad_bus_write_coil(bus, 1, 0x040A, IAD_TRUE, 0), IAD_ERR_SLAVE_EXCEPTION);  // FC05 is not carried
    CHECK_EQ(iad_bus_broadcast_coil(bus, 0x040A, IAD_TRUE), IAD_OK);                    // reaches nobody
    std::string gw = line_control(line, R"({"command": "gateway"})");
    CHECK(has(gw, "\"manu\":true"));
    CHECK(has(gw, "\"linked_slaves\":[1,2,5]"));
    CHECK(has(gw, "\"requests\":[\"D0 03 37 00 00 0B"));
    CHECK(has(gw, "\"rejected\":[\"01 05 04 0A FF 00"));
    CHECK(has(gw, "\"dropped_broadcasts\":[\"00 05 04 0A FF 00"));
    CHECK(has(gw, "\"function_codes\":[1,3,4,6,16,23]"));
    CHECK(!state(one).find("stop_held")->as_bool());
    CHECK(has(line_control(line, R"({"command": "set_gateway", "manu": false, "alarm_code": 2234,
                                     "emergency_stop": true})"),
              "{}"));
    CHECK_EQ(iad_bus_read_registers(bus, 0xD0, 0x3700, 11, 0, kUnset, IAD_INT_UNKNOWN, status), IAD_OK);
    CHECK_EQ(status[0], 0);
    CHECK_EQ(status[1], 0x0002);  // GSR2: EMST, no ATST
    CHECK_EQ(status[9], 2234);    // ALMC (low word)
    CHECK(has(line_control(line, R"({"command": "set_gateway", "slave": 215})"), "{}"));  // D7h
    CHECK_EQ(iad_bus_read_registers(bus, 0xD0, 0x3700, 1, 0, kUnset, IAD_INT_UNKNOWN, status), IAD_ERR_MODBUS_TIMEOUT);
    CHECK(has(line_control(line, R"({"command": "gateway_handle", "frame": "D7 03 37 00 00 01 99 88"})"),
              "\"reply\":\"D7 03 02 00 00"));
    std::size_t need = 0;
    CHECK_EQ(iad_sim_line_control_json(line, R"({"command": "set_gateway", "running": true})", nullptr, 0, &need),
             IAD_ERR_INVALID_ARGUMENT);  // an RCON gateway has no RUN flag
    CHECK_EQ(iad_sim_line_control_json(line, R"({"command": "set_gateway", "colour": 1})", nullptr, 0, &need),
             IAD_ERR_INVALID_ARGUMENT);
    // the silent no-access areas, switched on later
    uint16_t index[8] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x0000, 8, 0, kUnset, IAD_INT_UNKNOWN, index), IAD_OK);
    CHECK(has(line_control(line, R"({"command": "set_gateway", "silent_no_access": true})"), "{}"));
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x0000, 8, 0, kUnset, IAD_INT_UNKNOWN, index), IAD_ERR_MODBUS_TIMEOUT);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // closes the line and its units
    CHECK_EQ(state(two).find("open")->as_bool(), false);
    CHECK_EQ(iad_sim_line_close(line), IAD_OK);

    // a plain line has no gateway; a ROBONET in gateway mode has controls of its own
    iad_sim_line* plain = nullptr;
    CHECK_EQ(iad_sim_line_open("D", nullptr, 0, &plain), IAD_OK);
    CHECK(has(line_control(plain, R"({"command": "state"})"), "\"kind\":\"direct\""));
    CHECK_EQ(iad_sim_line_control_json(plain, R"({"command": "gateway"})", nullptr, 0, &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_line_close(plain), IAD_OK);
    iad_sim_line* net = nullptr;
    iad_sim* three = open_sim(R"({"slave": 3})");
    CHECK_EQ(iad_sim_gateway_line_open("robonet-gateway", nullptr, &three, 1, &net), IAD_OK);
    CHECK(has(line_control(net, R"({"command": "set_gateway", "running": false})"), "{}"));
    gw = line_control(net, R"({"command": "gateway"})");
    CHECK(has(gw, "\"running\":false"));
    CHECK(has(gw, "\"slave\":63"));
    CHECK(has(line_control(net, R"({"command": "gateway_handle", "frame": "3F 03 F7 00 00 02 F2 A1"})"),
              "\"reply\":\"3F 03 04 00 01 00 04"));  // stopped, one axis (axis 2: LNK bit 2)
    CHECK_EQ(iad_sim_line_close(net), IAD_OK);
    for (iad_sim* s : {one, two, three}) CHECK_EQ(iad_sim_close(s), IAD_OK);
}

TEST(capi06_sim_port_state_keys_and_a_failing_request_hook) {
    iad_sim* sim = open_sim(R"({"baud_rate": 115200, "unit_baud_rate": 38400, "port_name": "COM99",
                               "push_blip_mm": 6.76})");
    Json s = state(sim);
    CHECK_NEAR(s.find("push_blip_mm")->as_number(), 6.76, 1e-12);
    CHECK_NEAR(s.find("push_blip_len_mm")->as_number(), 0.10, 1e-12);
    CHECK_EQ(s.find("port_name")->as_string(), std::string("COM99"));
    CHECK_EQ(s.find("baud_history")->size(), 1u);
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "port_name", "value": "COM1"})"),
             IAD_ERR_INVALID_ARGUMENT);  // a fixed fact
    iad_transport* port = nullptr;
    CHECK_EQ(iad_sim_port(sim, &port), IAD_OK);
    CHECK(has(read_text([&](char* b, size_t n, size_t* r) { return iad_transport_info_json(port, b, n, r); }),
              "\"name\":\"COM99\""));
    CHECK_EQ(iad_transport_set_baud_rate(port, 38400), IAD_OK);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(port, nullptr, &bus), IAD_OK);
    CHECK_EQ(iad_transport_release(port), IAD_OK);
    uint16_t words[2] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(state(sim).find("baud_history")->size(), 2u);
    CHECK_EQ(control(sim, R"({"command": "set_option", "name": "push_blip_mm", "value": null})"), IAD_OK);
    CHECK(state(sim).find("push_blip_mm")->is_null());

    // a hook that fails: the bus call fails with the hook's text and is not retried
    CHECK_EQ(iad_sim_set_request_hook(sim, &failing_hook, nullptr), IAD_OK);
    const std::size_t before = Json::parse(read_text([&](char* b, size_t n, size_t* r) {
                                               return iad_sim_requests_json(sim, b, n, r);
                                           })).size();
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, 3, kUnset, IAD_INT_UNKNOWN, words), IAD_ERR_TRANSPORT);
    CHECK(has(iad_last_error_message(), "the cable is on fire"));
    CHECK_EQ(Json::parse(read_text([&](char* b, size_t n, size_t* r) { return iad_sim_requests_json(sim, b, n, r); }))
                 .size(),
             before + 1);
    uint8_t id[64];
    std::size_t length = 0;
    CHECK_EQ(iad_bus_report_slave_id(bus, 1, 0, kUnset, id, sizeof id, &length), IAD_ERR_TRANSPORT);
    CHECK(has(iad_last_error_message(), "request hook failed"));
    CHECK_EQ(iad_sim_set_request_hook(sim, nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // the port closes the unit
    CHECK_EQ(state(sim).find("open")->as_bool(), false);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi07_sim_slave_0_acts_on_broadcasts_and_never_answers) {
    // as the Python edition's SimulatedEC(slave=0): no address of its own, a broadcast still acts
    iad_sim* sim = open_sim(R"({"slave": 0})");
    CHECK_EQ(state(sim).find("slave")->as_int(), 0);
    uint8_t answer[64];
    std::size_t length = 99;
    const std::vector<std::uint8_t> read = build_frame(1, std::vector<std::uint8_t>{0x03, 0x90, 0x00, 0x00, 0x02});
    CHECK_EQ(iad_sim_handle_frame(sim, read.data(), read.size(), answer, sizeof answer, &length), IAD_OK);
    CHECK_EQ(length, 0u);  // slave 1 is another unit
    CHECK(state(sim).find("servo_on")->as_bool());
    const std::vector<std::uint8_t> off = build_frame(0, std::vector<std::uint8_t>{0x05, 0x04, 0x03, 0x00, 0x00});
    length = 99;
    CHECK_EQ(iad_sim_handle_frame(sim, off.data(), off.size(), answer, sizeof answer, &length), IAD_OK);
    CHECK_EQ(length, 0u);                           // a broadcast: no answer...
    CHECK(!state(sim).find("servo_on")->as_bool());  // ...but executed
    iad_sim* other = nullptr;
    CHECK_EQ(iad_sim_open(R"({"slave": 248})", &other), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_open(R"({"slave": -1})", &other), IAD_ERR_INVALID_ARGUMENT);
    CHECK(other == nullptr);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}
