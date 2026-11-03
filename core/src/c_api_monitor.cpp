// C ABI 0.5 status monitor additions (include/iai_actuator_driver.h, "axis: status monitor holds, port owner"): holding
// one axis's monitor (AxisMonitor::suspend, kept on the handle), the target and band its done rule checks a sample
// against (AxisMonitor::done_target) and the port's owner for enable_fast's baud change (Axis::set_port_owner). The
// Python facade (bindings/python/iai_actuator_driver/_native/monitor.py) runs the Python edition's monitor tests over
// these and the 0.2 monitor calls (src/c_api_ext.cpp). Evidence: UNVERIFIED on hardware (simulator tests only).
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/monitor.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

iad_result IAD_CALL iad_axis_suspend_monitor(iad_axis* axis, double timeout_s) {
    return guarded([&] {
        const auto e = find_axis(axis);
        if (!std::isnan(timeout_s) && !(timeout_s >= 0.0 && std::isfinite(timeout_s)))
            throw std::invalid_argument("timeout_s must be >= 0");
        const std::shared_ptr<AxisMonitor> mon = e->axis->monitor();
        if (!mon) throw EcNotReadyError("no status monitor: call iad_axis_start_monitor first");
        AxisMonitor::Suspension hold = mon->suspend(std::isnan(timeout_s) ? 1.0 : timeout_s);
        std::lock_guard<std::mutex> lock(e->monitor_mutex);
        e->monitor_holds.push_back(std::move(hold));
    });
}

iad_result IAD_CALL iad_axis_resume_monitor(iad_axis* axis) {
    return guarded([&] {
        const auto e = find_axis(axis);
        AxisMonitor::Suspension hold;
        {
            std::lock_guard<std::mutex> lock(e->monitor_mutex);
            if (e->monitor_holds.empty())
                throw std::invalid_argument("no iad_axis_suspend_monitor of this handle to resume");
            hold = std::move(e->monitor_holds.back());
            e->monitor_holds.pop_back();
        }
        hold.release();  // outside the lock: it wakes the monitor thread
    });
}

iad_result IAD_CALL iad_axis_monitor_target(iad_axis* axis, double* target_mm, double* band_mm) {
    return guarded([&] {
        const auto e = find_axis(axis);
        need(target_mm, "target_mm");
        need(band_mm, "band_mm");
        *target_mm = kNaN;
        *band_mm = kNaN;
        if (const auto target = AxisMonitor::done_target(*e->axis)) {
            *target_mm = target->first;
            *band_mm = target->second;
        }
    });
}

iad_result IAD_CALL iad_axis_set_port_owner(iad_axis* axis, iad_bool owner) {
    return guarded([&] {
        const auto e = find_axis(axis);
        e->axis->set_port_owner(owner != IAD_FALSE);
    });
}
