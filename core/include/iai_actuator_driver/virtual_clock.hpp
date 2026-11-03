// A virtual clock for tests (header-only; ABI 0.3 iad_clock_*): it only moves when slept on or advanced, so a timeout
// costs no real time and a run is the same on any PC. Installed through the clock seam of timing.hpp (set_clock), it
// stands in for now_s() and sleep_at_least() of the whole library. Port of the Python edition's
// testing.VirtualClock. Not for production code. Thread-safe.
#pragma once

#include <memory>
#include <mutex>

#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {

class VirtualClock {
public:
    explicit VirtualClock(double start_s = 1000.0) : now_(start_s) {}

    double now() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return now_;
    }
    /// Move the clock forward by `seconds` (no-op for 0, a negative value or NaN).
    void advance(double seconds) {
        if (!(seconds > 0.0)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        now_ += seconds;
    }
    /// What sleep_at_least() does while installed: advance at once.
    void sleep(double seconds) { advance(seconds); }

    /// Install `clock` for the whole library (set_clock); it stays alive while installed.
    static void install(const std::shared_ptr<VirtualClock>& clock) {
        set_clock([clock] { return clock->now(); }, [clock](double s) { clock->sleep(s); });
    }
    /// Back to the steady clock.
    static void uninstall() { set_clock(nullptr, nullptr); }

private:
    mutable std::mutex mutex_;
    double now_;
};

/// RAII for tests: a new VirtualClock installed while it lives (the steady clock again afterwards).
class VirtualClockScope {
public:
    explicit VirtualClockScope(double start_s = 1000.0) : clock_(std::make_shared<VirtualClock>(start_s)) {
        VirtualClock::install(clock_);
    }
    ~VirtualClockScope() { VirtualClock::uninstall(); }
    VirtualClockScope(const VirtualClockScope&) = delete;
    VirtualClockScope& operator=(const VirtualClockScope&) = delete;
    VirtualClock& clock() { return *clock_; }

private:
    std::shared_ptr<VirtualClock> clock_;
};

}  // namespace iai_actuator_driver
