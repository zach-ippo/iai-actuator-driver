// The non-volatile write guard's counter: one tracker per unit, shared by every protocol stack that writes the unit's
// non-volatile memory (an Axis's position-table rows and parameter commits today), so they all count against the same
// budget. The units' non-volatile memory lasts about kNvWriteCycles write cycles (results.hpp); above kNvWarnPerHour
// in the last kNvWindowS one warning per hour is logged (kNvRateWarning). Never a refusal: the driver warns, the
// integrator decides. Callers skip unchanged writes themselves and count only the real ones. Implemented in
// src/axis_nv.cpp.
#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {

/// The non-volatile write counter of one unit, its one-hour window and its parameters written to RAM and not yet
/// committed (results.hpp). Each Axis has its own until Axis::share_nonvolatile_state gives it the one every axis of
/// the process on the same unit shares (0.5; the Python edition's per-unit bookkeeping). Thread-safe.
class NvWriteTracker {
public:
    explicit NvWriteTracker(std::string label = std::string());
    /// Count one non-volatile write cycle at `now`; above kNvWarnPerHour in the window log kNvRateWarning (source
    /// "iai_actuator_driver.axis", once per kNvWindowS). Returns true when it warned.
    /// The stamps are kept in time order and only those more than kNvWindowS before the newest one are dropped, so a
    /// stamp given out of order (a test) is neither lost nor leaves the list unsorted.
    bool record(double now = now_s());
    long total() const;
    /// The writes of the window ending at `now`; forgets the stamps older than it (the library's own time only moves
    /// on).
    int last_hour(double now = now_s()) const;
    /// The same count for any `now` (a test's), changing nothing: the writes not older than kNvWindowS before it. A
    /// query for a later time keeps the real counts.
    int count_in_window(double now) const;
    void set_label(std::string label);
    /// A parameter was written to RAM and no confirmed commit (or restart) has saved it since.
    bool uncommitted() const { return uncommitted_.load(); }
    void set_uncommitted(bool on) { uncommitted_.store(on); }

private:
    void prune(double now) const;
    int count_in_window_locked(double now) const;

    mutable std::mutex mutex_;
    mutable std::deque<double> stamps_;  // in time order
    long total_ = 0;
    double last_warning_ = -1e300;
    std::string label_;
    std::atomic<bool> uncommitted_{false};
};

/// (0.5) The tracker every user of this process given `key` shares (made on first use, labelled `label` in its
/// warning); forget_nv_trackers drops the keys starting with `prefix` ("" = all), as a new process starts without them.
std::shared_ptr<NvWriteTracker> shared_nv_tracker(const std::string& key, const std::string& label);
void forget_nv_trackers(const std::string& prefix);

}  // namespace iai_actuator_driver
