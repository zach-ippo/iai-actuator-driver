// Public Axis accessors of pieces the axis already computes: finish_home() after home(wait = false), the home pattern
// ORG, the documented minimum speed and the caps the driver checks moves against. Port of the Python edition's ec.py
// (finish_home, read_home_pattern, recommended_min_speed, speed_cap_mm_s, accel_cap_g, accel_only_cap_g, decel_cap_g,
// push_cap_pct). The work is in axis_core.cpp / axis_motion.cpp / axis_motion_extras.cpp.
#include <optional>
#include <string>
#include <utility>

#include "axis_impl.hpp"

namespace iai_actuator_driver {

void Axis::finish_home() { impl_->finish_home(); }
std::optional<std::uint32_t> Axis::read_home_pattern() { return impl_->read_home_pattern(); }
std::pair<std::optional<double>, std::string> Axis::recommended_min_speed() { return impl_->recommended_min_speed(); }
double Axis::speed_cap() { return impl_->speed_cap(); }
double Axis::accel_cap() { return impl_->accel_cap(); }
double Axis::accel_only_cap() { return impl_->accel_only_cap(); }
double Axis::decel_cap() { return impl_->decel_cap(); }
double Axis::push_cap() { return impl_->push_cap(); }

}  // namespace iai_actuator_driver
