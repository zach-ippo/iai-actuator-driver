// Private to the core (internal: not installed, not part of the API): a condition variable whose timed waits run on
// the monotonic clock. The Linux release library is built against glibc 2.17 (manylinux2014), where libc++ has no
// pthread_cond_clockwait: there std::condition_variable::wait_for / wait_until turn into an absolute CLOCK_REALTIME
// deadline for pthread_cond_timedwait, so a backward step of the system clock during a wait (an NTP step, a VM's time
// sync) stretches the wait by the step. MonotonicCv is used wherever the core waits with a timeout.
//
// * Where pthread_condattr_setclock exists (_POSIX_CLOCK_SELECTION: Linux with glibc 2.3.3 or later, musl, the BSDs) it
//   is a pthread condition variable on CLOCK_MONOTONIC, used with the native handle of the std::mutex of the
//   std::unique_lock; every deadline is taken on CLOCK_MONOTONIC.
// * Elsewhere it is std::condition_variable: on Windows libc++ and the MSVC STL hand Windows a relative timeout for each
//   wait (SleepConditionVariableSRW), which a clock step does not stretch. (macOS has no pthread_condattr_setclock:
//   there it stays std::condition_variable, as before; no macOS library is shipped.)
//
// The calls are those of std::condition_variable with std::unique_lock<std::mutex>: wait, wait_for (any duration),
// wait_until (steady_clock time points only), each with and without a predicate; notify_one / notify_all. A timeout of
// 0, a negative one or NaN does not wait (with a predicate: its value); a timeout longer than kMaxWaitS waits
// kMaxWaitS (about 31 years, so a deadline never overflows). Spurious wake-ups and EINTR return from the calls without
// a predicate (as std::condition_variable may) and are waited out by the calls with one. Not copyable.
#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <system_error>

#if !defined(_WIN32) && defined(__has_include)
#if __has_include(<unistd.h>)
#include <unistd.h>
#endif
#endif
#if !defined(_WIN32) && defined(_POSIX_CLOCK_SELECTION) && _POSIX_CLOCK_SELECTION > 0
#define IAD_MONOTONIC_CV_PTHREAD 1
#include <errno.h>
#include <pthread.h>
#include <time.h>
#else
#define IAD_MONOTONIC_CV_PTHREAD 0
#endif

namespace iai_actuator_driver {
namespace detail {

class MonotonicCv {
public:
    /// The longest wait (s): a longer timeout is cut to it.
    static constexpr double kMaxWaitS = 1e9;
    /// True when the timed waits are a pthread condition variable on CLOCK_MONOTONIC (see the file comment).
    static constexpr bool kPthreadMonotonic = IAD_MONOTONIC_CV_PTHREAD != 0;

    MonotonicCv(const MonotonicCv&) = delete;
    MonotonicCv& operator=(const MonotonicCv&) = delete;

    /// Seconds of `d` for a wait: 0 for 0, a negative value or NaN, at most kMaxWaitS.
    template <class Rep, class Period>
    static double wait_seconds(const std::chrono::duration<Rep, Period>& d) noexcept {
        const double s = std::chrono::duration<double>(d).count();
        if (!(s > 0.0)) return 0.0;
        return s < kMaxWaitS ? s : kMaxWaitS;
    }

    template <class Predicate>
    void wait(std::unique_lock<std::mutex>& lock, Predicate pred) {
        while (!pred()) wait(lock);
    }

    template <class Rep, class Period>
    std::cv_status wait_for(std::unique_lock<std::mutex>& lock, const std::chrono::duration<Rep, Period>& d) {
        return wait_seconds_for(lock, wait_seconds(d));
    }

    template <class Rep, class Period, class Predicate>
    bool wait_for(std::unique_lock<std::mutex>& lock, const std::chrono::duration<Rep, Period>& d, Predicate pred) {
        return wait_seconds_for(lock, wait_seconds(d), pred);
    }

    template <class Duration>
    std::cv_status wait_until(std::unique_lock<std::mutex>& lock,
                              const std::chrono::time_point<std::chrono::steady_clock, Duration>& t) {
        return wait_seconds_for(lock, seconds_until(t));
    }

    template <class Duration, class Predicate>
    bool wait_until(std::unique_lock<std::mutex>& lock,
                    const std::chrono::time_point<std::chrono::steady_clock, Duration>& t, Predicate pred) {
        return wait_seconds_for(lock, seconds_until(t), pred);
    }

#if IAD_MONOTONIC_CV_PTHREAD
    MonotonicCv() {
        pthread_condattr_t attr;
        int rc = pthread_condattr_init(&attr);
        if (rc != 0) throw std::system_error(rc, std::system_category(), "pthread_condattr_init");
        rc = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        if (rc == 0) rc = pthread_cond_init(&cv_, &attr);
        pthread_condattr_destroy(&attr);
        if (rc != 0) throw std::system_error(rc, std::system_category(), "pthread_cond_init (CLOCK_MONOTONIC)");
    }
    ~MonotonicCv() { pthread_cond_destroy(&cv_); }

    void notify_one() noexcept { pthread_cond_signal(&cv_); }
    void notify_all() noexcept { pthread_cond_broadcast(&cv_); }

    /// Until notified (or a spurious wake-up).
    void wait(std::unique_lock<std::mutex>& lock) {
        const int rc = pthread_cond_wait(&cv_, lock.mutex()->native_handle());
        if (rc != 0 && rc != EINTR) throw std::system_error(rc, std::system_category(), "pthread_cond_wait");
    }

private:
    pthread_cond_t cv_;

    static timespec monotonic_now() noexcept {
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        return now;
    }
    /// CLOCK_MONOTONIC + `seconds` (0 <= seconds <= kMaxWaitS: no overflow).
    static timespec deadline_after(double seconds) noexcept {
        timespec t = monotonic_now();
        const auto whole = static_cast<time_t>(seconds);
        t.tv_sec += whole;
        t.tv_nsec += static_cast<long>((seconds - static_cast<double>(whole)) * 1e9);
        if (t.tv_nsec >= 1000000000L) {
            t.tv_sec += 1;
            t.tv_nsec -= 1000000000L;
        }
        return t;
    }
    static bool passed(const timespec& deadline) noexcept {
        const timespec now = monotonic_now();
        return now.tv_sec > deadline.tv_sec || (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec);
    }
    /// One wait up to `deadline`: false once it has passed (ETIMEDOUT), true on a wake-up (notified, spurious, EINTR).
    bool wait_deadline(std::unique_lock<std::mutex>& lock, const timespec& deadline) {
        const int rc = pthread_cond_timedwait(&cv_, lock.mutex()->native_handle(), &deadline);
        if (rc == ETIMEDOUT) return false;
        if (rc != 0 && rc != EINTR) throw std::system_error(rc, std::system_category(), "pthread_cond_timedwait");
        return true;
    }

    std::cv_status wait_seconds_for(std::unique_lock<std::mutex>& lock, double seconds) {
        if (!(seconds > 0.0)) return std::cv_status::timeout;
        const timespec deadline = deadline_after(seconds);
        if (!wait_deadline(lock, deadline)) return std::cv_status::timeout;
        return passed(deadline) ? std::cv_status::timeout : std::cv_status::no_timeout;
    }
    template <class Predicate>
    bool wait_seconds_for(std::unique_lock<std::mutex>& lock, double seconds, Predicate& pred) {
        if (!(seconds > 0.0)) return pred();
        const timespec deadline = deadline_after(seconds);
        while (!pred()) {
            if (passed(deadline) || !wait_deadline(lock, deadline)) return pred();
        }
        return true;
    }
#else
    MonotonicCv() = default;

    void notify_one() noexcept { cv_.notify_one(); }
    void notify_all() noexcept { cv_.notify_all(); }

    /// Until notified (or a spurious wake-up).
    void wait(std::unique_lock<std::mutex>& lock) { cv_.wait(lock); }

private:
    std::condition_variable cv_;

    std::cv_status wait_seconds_for(std::unique_lock<std::mutex>& lock, double seconds) {
        if (!(seconds > 0.0)) return std::cv_status::timeout;
        return cv_.wait_for(lock, std::chrono::duration<double>(seconds));
    }
    template <class Predicate>
    bool wait_seconds_for(std::unique_lock<std::mutex>& lock, double seconds, Predicate& pred) {
        if (!(seconds > 0.0)) return pred();
        return cv_.wait_for(lock, std::chrono::duration<double>(seconds), pred);
    }
#endif

    /// Seconds from now to `t` on the steady clock (for a wait: see wait_seconds), without integer overflow.
    template <class Duration>
    static double seconds_until(const std::chrono::time_point<std::chrono::steady_clock, Duration>& t) noexcept {
        const double s = std::chrono::duration<double>(t.time_since_epoch()).count() -
                         std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        return wait_seconds(std::chrono::duration<double>(s));
    }
};

}  // namespace detail
}  // namespace iai_actuator_driver

#undef IAD_MONOTONIC_CV_PTHREAD
