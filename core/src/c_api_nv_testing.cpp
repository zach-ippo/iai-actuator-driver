// C ABI 0.7 additions (include/iai_actuator_driver_testing.h): the test hook of the non-volatile
// write guard (Axis::testing_note_nonvolatile_write, src/axis_nv.cpp). A binding's tests count writes at a time of
// their choosing (the rate warning an hour later) without making real non-volatile writes. Never needed in production.

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/timing.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

/// A time on the library's clock (iad_clock_now): NaN = now; anything else must be finite.
double clock_time(double seconds) {
    if (std::isnan(seconds)) return now_s();
    if (!std::isfinite(seconds)) throw std::invalid_argument("now_s must be finite (NaN: now)");
    return seconds;
}

}  // namespace

extern "C" {

iad_result IAD_CALL iad_testing_axis_nv_record(iad_axis* axis, double now_s, const char* what, iad_bool* warned) {
    return guarded([&] {
        const auto e = find_axis(axis);
        const bool warning = e->axis->testing_note_nonvolatile_write(clock_time(now_s), what && *what ? what : "test");
        if (warned) *warned = warning ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_testing_axis_nv_last_hour(iad_axis* axis, double now_s, int32_t* count) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(count, "count");
        *count = static_cast<int32_t>(e->axis->testing_nonvolatile_writes_last_hour(clock_time(now_s)));
    });
}

}  // extern "C"
