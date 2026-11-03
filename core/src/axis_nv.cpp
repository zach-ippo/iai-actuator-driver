// The non-volatile write guard's counter and rate warning, and the move-time estimate (axis unit).
//
// The units' non-volatile memory lasts about 100,000 write cycles (IAI). Every real non-volatile write of an Axis (a
// position-table row write, a parameter commit with PARW) is counted; above kNvWarnPerHour in the last kNvWindowS one
// warning per hour is logged (kNvRateWarning). Never a refusal: the driver warns, the integrator decides. The skip of
// unchanged writes is in axis_config.cpp. New in all editions; UNVERIFIED on hardware in this edition.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "axis_impl.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/results.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace detail {

NvWriteTracker::NvWriteTracker(std::string label) : label_(std::move(label)) {}

void NvWriteTracker::prune(double now) const {
    while (!stamps_.empty() && now - stamps_.front() > kNvWindowS) stamps_.pop_front();
}

namespace {
std::mutex& registry_mutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, std::shared_ptr<NvWriteTracker>>& registry() {
    static std::map<std::string, std::shared_ptr<NvWriteTracker>> trackers;
    return trackers;
}
}  // namespace

std::shared_ptr<NvWriteTracker> shared_nv_tracker(const std::string& key, const std::string& label) {
    std::lock_guard<std::mutex> lock(registry_mutex());
    std::shared_ptr<NvWriteTracker>& slot = registry()[key];
    if (!slot) slot = std::make_shared<NvWriteTracker>(label);
    slot->set_label(label);
    return slot;
}

void forget_nv_trackers(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(registry_mutex());
    auto& trackers = registry();
    for (auto it = trackers.begin(); it != trackers.end();)
        it = it->first.compare(0, prefix.size(), prefix) == 0 ? trackers.erase(it) : std::next(it);
}

bool NvWriteTracker::record(double now) {
    int count = 0;
    std::string label;
    bool warn = false;
    long total = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++total_;
        total = total_;
        stamps_.insert(std::upper_bound(stamps_.begin(), stamps_.end(), now), now);
        prune(stamps_.back());
        count = count_in_window_locked(now);
        warn = count > kNvWarnPerHour && !(now - last_warning_ < kNvWindowS);
        if (warn) last_warning_ = now;
        label = label_;
    }
    log_debug(kAxisLog, util::format("non-volatile write%s%s (%ld on this axis, %d in the last hour)",
                                     label.empty() ? "" : " on ", label.c_str(), total, count));
    if (!warn) return false;
    char buf[512];
    std::snprintf(buf, sizeof buf, kNvRateWarning, count, label.empty() ? "this axis" : label.c_str(),
                  static_cast<double>(kNvWriteCycles) / count / 24.0);
    log_warning(kAxisLog, buf);
    return true;
}

long NvWriteTracker::total() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_;
}

int NvWriteTracker::last_hour(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    prune(now);
    return count_in_window_locked(now);
}

int NvWriteTracker::count_in_window(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_in_window_locked(now);
}

int NvWriteTracker::count_in_window_locked(double now) const {
    return static_cast<int>(
        std::count_if(stamps_.begin(), stamps_.end(), [now](double stamp) { return now - stamp <= kNvWindowS; }));
}

void NvWriteTracker::set_label(std::string label) {
    std::lock_guard<std::mutex> lock(mutex_);
    label_ = std::move(label);
}

double motion_time_s(double distance, double speed, double accel_g) {
    const double accel = std::max(accel_g, 0.01) * reg::kGmms2;
    const double v = std::max(speed, 0.01);
    const double d = std::max(std::fabs(distance), 0.0);
    const double ramp = v * v / accel;  // distance spent accelerating + decelerating
    return d >= ramp ? d / v + v / accel : 2.0 * std::sqrt(d / accel);
}

}  // namespace detail

void Axis::Impl::note_nonvolatile_write(const std::string& what) {
    log_debug(detail::kAxisLog, "non-volatile write: " + what);
    nv()->record(now_s());
}

long Axis::nonvolatile_writes() const { return impl_->nv()->total(); }

int Axis::nonvolatile_writes_last_hour() const { return impl_->nv()->last_hour(now_s()); }

bool Axis::has_uncommitted_parameters() const { return impl_->nv()->uncommitted(); }

void Axis::share_nonvolatile_state(const std::string& unit_key, const std::string& label) {
    std::shared_ptr<detail::NvWriteTracker> tracker =
        unit_key.empty() ? std::make_shared<detail::NvWriteTracker>(label) : detail::shared_nv_tracker(unit_key, label);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->nv_tracker = std::move(tracker);
}

void Axis::forget_nonvolatile_state(const std::string& unit_key_prefix) { detail::forget_nv_trackers(unit_key_prefix); }

bool Axis::testing_note_nonvolatile_write(double now, const std::string& what) {
    log_debug(detail::kAxisLog, "non-volatile write: " + what);
    return impl_->nv()->record(now);
}

int Axis::testing_nonvolatile_writes_last_hour(double now) const { return impl_->nv()->count_in_window(now); }

}  // namespace iai_actuator_driver
