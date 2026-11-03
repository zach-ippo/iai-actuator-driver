// The option structs of the C ABI's axis commands (0.8: iad_servo_options ... iad_cancel_paused_move_options), built
// for the tests: each from its _init function, then the fields the test sets. ptr_of passes a temporary's address for
// the duration of the call: iad_axis_stop(ax, ptr_of(stop_opts(IAD_TRUE, 5.0)), &st).
#pragma once

#include <cstdint>
#include <limits>

#include "iai_actuator_driver.h"

/// NaN: "not set" (the default the header names).
inline const double kUnset = std::numeric_limits<double>::quiet_NaN();

/// The address of `value`, valid until the end of the full expression that holds the call.
template <class T>
const T* ptr_of(const T& value) {
    return &value;
}

inline iad_servo_options servo_opts(iad_bool wait, double timeout_s) {
    iad_servo_options o;
    iad_servo_options_init(&o);
    o.wait = wait;
    o.timeout_s = timeout_s;
    return o;
}

inline iad_home_options home_opts(iad_bool wait, double timeout_s, std::int32_t method) {
    iad_home_options o;
    iad_home_options_init(&o);
    o.wait = wait;
    o.timeout_s = timeout_s;
    o.method = method;
    return o;
}

inline iad_stop_options stop_opts(iad_bool wait, double timeout_s) {
    iad_stop_options o;
    iad_stop_options_init(&o);
    o.wait = wait;
    o.timeout_s = timeout_s;
    return o;
}

inline iad_wait_idle_options wait_idle_opts(double target, double band, double timeout_s,
                                            std::int32_t completion = IAD_COMPLETION_DEFAULT) {
    iad_wait_idle_options o;
    iad_wait_idle_options_init(&o);
    o.target = target;
    o.band = band;
    o.timeout_s = timeout_s;
    o.completion = completion;
    return o;
}

inline iad_change_options change_opts(iad_bool wait, double timeout_s) {
    iad_change_options o;
    iad_change_options_init(&o);
    o.wait = wait;
    o.timeout_s = timeout_s;
    return o;
}

inline iad_cancel_paused_move_options cancel_paused_opts(double timeout_s) {
    iad_cancel_paused_move_options o;
    iad_cancel_paused_move_options_init(&o);
    o.timeout_s = timeout_s;
    return o;
}
