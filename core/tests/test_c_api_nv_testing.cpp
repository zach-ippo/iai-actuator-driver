// C ABI 0.7 (csharp-fixes): the test hook of the non-volatile write guard (src/c_api_nv_testing.cpp). Simulator only.
#include <cmath>
#include <cstdint>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "harness.hpp"

using namespace iai_actuator_driver;

TEST(capi07_testing_axis_nv_record_counts_warns_once_and_follows_the_unit) {
    iad_sim* sim = nullptr;
    REQUIRE(iad_sim_open(R"({"time_scale": 20})", &sim) == IAD_OK);
    iad_transport* port = nullptr;
    REQUIRE(iad_sim_transport(sim, &port) == IAD_OK);
    iad_bus* bus = nullptr;
    REQUIRE(iad_bus_open(port, nullptr, &bus) == IAD_OK);
    CHECK_EQ(iad_transport_release(port), IAD_OK);
    iad_axis* one = nullptr;
    iad_axis* two = nullptr;
    REQUIRE(iad_axis_open_bus(bus, 1, nullptr, &one) == IAD_OK);
    REQUIRE(iad_axis_open_bus(bus, 1, nullptr, &two) == IAD_OK);
    CHECK_EQ(iad_axis_set_nv_unit(one, "test:capi07:1", "SIM slave 1"), IAD_OK);
    CHECK_EQ(iad_axis_set_nv_unit(two, "test:capi07:1", "SIM slave 1"), IAD_OK);

    double t0 = 0.0;
    CHECK_EQ(iad_clock_now(&t0), IAD_OK);
    iad_bool warned = IAD_TRUE;
    for (int i = 0; i < 60; ++i) {
        CHECK_EQ(iad_testing_axis_nv_record(one, t0, nullptr, &warned), IAD_OK);
        CHECK_EQ(warned, IAD_FALSE);
    }
    CHECK_EQ(iad_testing_axis_nv_record(two, t0, "row 2", &warned), IAD_OK);  // the 61st in the hour, same unit
    CHECK_EQ(warned, IAD_TRUE);
    CHECK_EQ(iad_testing_axis_nv_record(one, NAN, nullptr, &warned), IAD_OK);  // NaN: now; one warning per hour
    CHECK_EQ(warned, IAD_FALSE);
    int64_t total = 0;
    int32_t last_hour = 0;
    CHECK_EQ(iad_axis_nonvolatile_writes(two, &total, &last_hour, nullptr), IAD_OK);
    CHECK_EQ(total, 62);
    CHECK_EQ(last_hour, 62);
    int32_t count = -1;
    CHECK_EQ(iad_testing_axis_nv_last_hour(one, NAN, &count), IAD_OK);
    CHECK_EQ(count, 62);
    CHECK_EQ(iad_testing_axis_nv_last_hour(one, t0 + 3600.0 + 10.0, &count), IAD_OK);  // the window has moved on
    CHECK_EQ(count, 0);
    CHECK_EQ(iad_testing_axis_nv_record(one, t0 + 3600.0 + 20.0, nullptr, nullptr), IAD_OK);  // warned may be NULL
    CHECK_EQ(iad_testing_axis_nv_last_hour(two, t0 + 3600.0 + 20.0, &count), IAD_OK);
    CHECK_EQ(count, 1);

    CHECK_EQ(iad_testing_axis_nv_record(one, INFINITY, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_testing_axis_nv_last_hour(one, NAN, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_testing_axis_nv_record(nullptr, NAN, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(two), IAD_OK);
    CHECK_EQ(iad_axis_close(one), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
    CHECK_EQ(iad_nv_forget("test:capi07:"), IAD_OK);
}
