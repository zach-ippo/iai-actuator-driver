// C ABI 0.6, csharp block (src/c_api_cell.cpp) on simulated lines: a port's baud detection with the caller's
// candidates, and one address's identity read on a bus without an axis. The C# Port and DeviceIdentity facades
// (bindings/dotnet) run the managed edition's tests over these.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A simulated line of EC sliders at `slaves` and a bus on it (closed by close_line).
struct SimBus {
    std::vector<iad_sim*> units;
    iad_sim_line* line = nullptr;
    iad_bus* bus = nullptr;
};

SimBus open_line(const char* name, const std::vector<int>& slaves, const std::string& more = "") {
    SimBus out;
    for (int s : slaves) {
        iad_sim* sim = nullptr;
        const std::string options = R"({"time_scale": 20, "slave": )" + std::to_string(s) + more + "}";
        REQUIRE(iad_sim_open(options.c_str(), &sim) == IAD_OK);
        out.units.push_back(sim);
    }
    REQUIRE(iad_sim_line_open(name, out.units.data(), out.units.size(), &out.line) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_line_transport(out.line, &t) == IAD_OK);
    REQUIRE(iad_bus_open(t, nullptr, &out.bus) == IAD_OK);
    iad_transport_release(t);
    return out;
}

void close_line(SimBus& b) {
    iad_bus_close(b.bus);
    iad_sim_line_close(b.line);
    for (iad_sim* s : b.units) iad_sim_close(s);
}

template <class F>
Json json_of(F call) {
    std::string buffer(64 * 1024, '\0');
    size_t required = 0;
    REQUIRE(call(&buffer[0], buffer.size(), &required) == IAD_OK);
    return Json::parse(buffer.c_str());
}

std::string last_message() {
    std::string buffer(8192, '\0');
    size_t required = 0;
    iad_last_error_json(&buffer[0], buffer.size(), &required);
    const Json error = Json::parse(buffer.c_str());  // named: find() points into it
    const Json* message = error.find("message");
    return message ? message->as_string() : std::string();
}

}  // namespace

TEST(capi_csharp_port_detects_its_rate_from_the_callers_candidates) {
    SimBus b = open_line("L", {1, 2}, R"(, "baud_rate": 115200, "unit_baud_rate": 38400)");
    CHECK_EQ(iad_bus_set_port(b.bus, "L", IAD_LINE_DIRECT, kNaN, IAD_FALSE, nullptr, nullptr), IAD_OK);
    int32_t rate = 0;
    CHECK_EQ(iad_bus_port_detect_baud_rate(b.bus, "1", "9600", 0.01, &rate), IAD_ERR_MODBUS_TIMEOUT);
    CHECK(last_message().find("24 V supply") != std::string::npos);  // the no-answer checklist
    CHECK_EQ(rate, IAD_INT_UNKNOWN);
    Json info = json_of([&](char* p, size_t n, size_t* r) { return iad_bus_port_info_json(b.bus, p, n, r); });
    CHECK(!info.find("baud_detected")->as_bool());
    CHECK_EQ(info.find("baud_rate")->as_int(), 115200LL);  // the starting rate is restored
    CHECK_EQ(iad_bus_port_detect_baud_rate(b.bus, "2", "9600,38400", kNaN, &rate), IAD_OK);
    CHECK_EQ(rate, 38400);
    info = json_of([&](char* p, size_t n, size_t* r) { return iad_bus_port_info_json(b.bus, p, n, r); });
    CHECK(info.find("baud_detected")->as_bool());
    iad_bool detected = IAD_FALSE;
    CHECK_EQ(iad_bus_ensure_baud_rate(b.bus, "1,2", &rate, &detected), IAD_OK);  // found already: nothing sent
    CHECK_EQ(rate, 38400);
    CHECK(detected == IAD_TRUE);
    CHECK_EQ(iad_bus_port_detect_baud_rate(b.bus, "0", nullptr, kNaN, &rate), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_port_detect_baud_rate(b.bus, "1", nullptr, -1.0, &rate), IAD_ERR_INVALID_ARGUMENT);
    close_line(b);
}

TEST(capi_csharp_device_info_on_a_bus_without_an_axis) {
    SimBus b = open_line("L", {1, 3});
    const Json info =
        json_of([&](char* p, size_t n, size_t* r) { return iad_bus_read_device_info_json(b.bus, 3, p, n, r); });
    CHECK_EQ(info.find("slave")->as_int(), 3LL);
    CHECK_EQ(info.find("system_code")->as_int(), 0xECLL);
    CHECK_EQ(info.find("identity_source")->as_string(), std::string("FC11"));
    CHECK(info.find("model_name")->type() == Json::Type::String);
    CHECK(info.find("categories")->size() > 0u);
    CHECK(info.find("soft_limits_mm")->type() == Json::Type::Null);  // no limits read
    CHECK_EQ(iad_bus_set_retries(b.bus, 0), IAD_OK);
    const Json silent =
        json_of([&](char* p, size_t n, size_t* r) { return iad_bus_read_device_info_json(b.bus, 9, p, n, r); });
    CHECK(silent.find("system_code")->type() == Json::Type::Null);
    bool fc11 = false;
    for (const Json& e : silent.find("errors")->as_array())
        if (e.as_string().rfind("report slave id: not answered", 0) == 0) fc11 = true;
    CHECK(fc11);  // kept in errors, never returned
    size_t required = 0;
    CHECK_EQ(iad_bus_read_device_info_json(b.bus, 0, nullptr, 0, &required), IAD_ERR_INVALID_ARGUMENT);
    close_line(b);
}
