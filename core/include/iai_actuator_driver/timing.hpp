// Monotonic clock and minimum-length sleeps (header-only).
//
// A sleep may last longer than asked (Windows rounds up to the timer resolution, about 15.6 ms by default; the serial
// transport raises it to 1 ms while a port is open); sleep_at_least() never returns early, which is what the bus gap
// and the settle times need (a minimum, never a maximum).
//
// Clock seam (ABI 0.3, for tests): set_clock() replaces now_s() and sleep_at_least() process-wide, e.g. with a virtual
// clock that only moves when slept on (virtual_clock.hpp), so a timeout costs no real time and a run is the same on any
// PC. The bus (gaps, response deadlines, retries), the axis (polls, waits, the non-volatile write window), the
// cancellable sleeps and the simulator's default clock all read it. Waits on a condition variable (a status monitor
// waiting for its next poll, the bus lock, the event queues) keep real-time slices but take their deadlines from
// now_s(): a virtual clock is meant for single-threaded tests, as in the Python edition's testing.virtual_clock().
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace iai_actuator_driver {

namespace detail {
/// The installed clock (null: the steady clock). Replaced hooks are never freed: another thread may still be reading
/// them (a few bytes per set_clock() call; test-only). They are kept in retire_clock_hooks()'s list, so that a leak
/// checker (LeakSanitizer) finds them reachable at exit instead of reporting them as lost memory.
struct ClockHooks {
    std::function<double()> now;
    std::function<void(double)> sleep;
};
inline std::atomic<const ClockHooks*>& clock_hooks() noexcept {
    static std::atomic<const ClockHooks*> hooks{nullptr};
    return hooks;
}
/// Keep a replaced hook for the rest of the process (never freed; the list and its mutex are never destroyed either,
/// a thread may still read a clock while the process exits).
inline void retire_clock_hooks(const ClockHooks* replaced) {
    if (replaced == nullptr) return;
    static std::mutex* const mutex = new std::mutex;
    static std::vector<const ClockHooks*>* const retired = new std::vector<const ClockHooks*>;
    std::lock_guard<std::mutex> lock(*mutex);
    retired->push_back(replaced);
}
}  // namespace detail

/// Seconds on the steady clock (arbitrary origin), whatever clock is installed.
inline double steady_now_s() noexcept {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

/// Seconds on a monotonic clock (arbitrary origin): the installed clock (set_clock), else the steady clock.
inline double now_s() noexcept {
    if (const detail::ClockHooks* hooks = detail::clock_hooks().load(std::memory_order_acquire)) {
        try {
            return hooks->now();
        } catch (...) {
            // a broken test clock must never break communication: fall back to real time
        }
    }
    return steady_now_s();
}

/// Sleep for at least `seconds` (no-op for 0, a negative value or NaN). With a clock installed: its sleep.
inline void sleep_at_least(double seconds) {
    if (!(seconds > 0.0)) return;
    if (const detail::ClockHooks* hooks = detail::clock_hooks().load(std::memory_order_acquire)) {
        hooks->sleep(seconds);
        return;
    }
    const double end = steady_now_s() + seconds;
    for (;;) {
        const double remaining = end - steady_now_s();
        if (remaining <= 0.0) return;
        std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    }
}

/// (ABI 0.3) Install a clock for now_s() / sleep_at_least() (both functions, or both empty to restore the steady
/// clock). Process-wide; for tests (virtual_clock.hpp).
inline void set_clock(std::function<double()> now, std::function<void(double)> sleep) {
    if (!now || !sleep) {
        detail::retire_clock_hooks(detail::clock_hooks().exchange(nullptr, std::memory_order_acq_rel));
        return;
    }
    const detail::ClockHooks* fresh = new detail::ClockHooks{std::move(now), std::move(sleep)};
    detail::retire_clock_hooks(detail::clock_hooks().exchange(fresh, std::memory_order_acq_rel));
}

/// True while a clock other than the steady clock is installed.
inline bool clock_installed() noexcept { return detail::clock_hooks().load(std::memory_order_acquire) != nullptr; }

}  // namespace iai_actuator_driver
