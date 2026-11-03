// Bus unit: the condition variable of the core's timed waits (src/monotonic_cv.hpp: on Linux a pthread one on
// CLOCK_MONOTONIC) and the clock-step regression of a core wait. The timings are generous upper bounds and lower bounds
// a wait can never undercut. No serial port is opened.
//
// bus_monotonic_cv_clock_step (Linux only, and only with CAP_SYS_TIME, e.g. as root; otherwise it says so and passes)
// steps CLOCK_REALTIME back during 200 ms waits (CancelToken::wait_changed, MonotonicCv::wait_for / wait_until) and
// checks each returns within 300 ms of steady time. The step is 1 s, or 0.3 s when a time service undid the 1 s step
// before the wait returned (WSL does so within about 0.1 s); afterwards the clock is put back (the steady time that
// passed is added to the time read before the step). Before 1.0.1 the Linux release library (libc++, glibc 2.17)
// waited on an absolute CLOCK_REALTIME deadline there, and such a step stretched the wait by the step. The file builds
// against a core without monotonic_cv.hpp too (the CancelToken check then runs alone), to show that.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/timing.hpp"

#if __has_include("monotonic_cv.hpp")
#include "monotonic_cv.hpp"
#define IAD_TEST_HAS_MONOTONIC_CV 1
#else
#define IAD_TEST_HAS_MONOTONIC_CV 0
#endif

#if defined(__linux__)
#include <time.h>

#include <fstream>
#include <string>
#endif

using namespace iai_actuator_driver;

namespace {

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

#if defined(__linux__)
/// CAP_SYS_TIME (bit 25) in the effective capabilities of this process: it may set CLOCK_REALTIME.
bool may_set_clock() {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("CapEff:", 0) != 0) continue;
        try {
            return ((std::stoull(line.substr(7), nullptr, 16) >> 25) & 1ULL) != 0;
        } catch (...) {
            return false;
        }
    }
    return false;
}

double seconds_of(const timespec& t) { return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9; }

timespec timespec_of(double s) {
    timespec t{};
    t.tv_sec = static_cast<time_t>(std::floor(s));
    t.tv_nsec = static_cast<long>((s - std::floor(s)) * 1e9);
    if (t.tv_nsec >= 1000000000L) {
        t.tv_sec += 1;
        t.tv_nsec -= 1000000000L;
    }
    return t;
}

/// RAII: notes CLOCK_REALTIME and CLOCK_MONOTONIC; step_back() sets the realtime clock `seconds` behind; offset_s() is
/// how far it is from the time noted plus the monotonic time that passed (-seconds while a step holds); the destructor
/// puts it back there.
class ClockStep {
public:
    ClockStep() {
        clock_gettime(CLOCK_REALTIME, &real0_);
        clock_gettime(CLOCK_MONOTONIC, &mono0_);
    }
    ~ClockStep() {
        if (!stepped_.load()) return;
        const timespec back = timespec_of(seconds_of(real0_) + since_s());
        if (clock_settime(CLOCK_REALTIME, &back) != 0) std::fprintf(stderr, "could not put CLOCK_REALTIME back\n");
    }
    ClockStep(const ClockStep&) = delete;
    ClockStep& operator=(const ClockStep&) = delete;

    bool step_back(double seconds) {
        const timespec t = timespec_of(seconds_of(real0_) + since_s() - seconds);
        if (clock_settime(CLOCK_REALTIME, &t) != 0) return false;
        stepped_ = true;
        return true;
    }
    bool stepped() const { return stepped_.load(); }
    double offset_s() const {
        const double since = since_s();
        timespec now{};
        clock_gettime(CLOCK_REALTIME, &now);
        return seconds_of(now) - seconds_of(real0_) - since;
    }

private:
    double since_s() const {
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        return seconds_of(now) - seconds_of(mono0_);
    }
    timespec real0_{};
    timespec mono0_{};
    std::atomic<bool> stepped_{false};
};

/// Run `wait` (a 200 ms wait) while another thread steps CLOCK_REALTIME back by `step_s` 50 ms into it and watches the
/// step until the wait returns; the steady seconds the wait took, or -1 when the step did not hold that long (a time
/// service put the clock forward again). The clock is put back before it returns.
template <class Wait>
double steady_seconds_with_a_clock_step(double step_s, Wait wait) {
    ClockStep clock;
    std::atomic<bool> done{false};
    bool held = true;
    double took = 0.0;
    {
        ec_test::Thread stepper([&] {
            sleep_s(0.05);
            CHECK(clock.step_back(step_s));
            while (!done.load()) {
                if (std::fabs(clock.offset_s() + step_s) > 0.05) held = false;
                sleep_s(0.001);
            }
        });
        const double t0 = steady_now_s();
        wait();
        took = steady_now_s() - t0;
        done = true;
    }
    CHECK(clock.stepped());
    return held ? took : -1.0;
}

/// The steady seconds of `wait` with a clock step that held: 1 s, else 0.3 s (WSL's time sync puts a step of 0.5 s or
/// more forward again within about 0.1 s, a step of 0.3 s it keeps); -1 when neither held (said, not measured).
template <class Wait>
double steady_seconds_with_a_held_clock_step(const char* what, Wait wait) {
    for (const double step_s : {1.0, 0.3}) {
        const double took = steady_seconds_with_a_clock_step(step_s, wait);
        if (took < 0.0) continue;
        std::printf("  (%s with CLOCK_REALTIME stepped back %.1f s during it: %.3f s)\n", what, step_s, took);
        return took;
    }
    std::printf("  (%s: the system's time service undid every clock step, not measured)\n", what);
    return -1.0;
}

/// A 200 ms wait that returned within 300 ms of steady time (or was not measured: negative).
void check_200_ms(double took) {
    if (took < 0.0) return;
    CHECK(took >= 0.19);
    CHECK(took < 0.3);
}
#endif

}  // namespace

// ============================================================================================ the clock-step regression

TEST(bus_monotonic_cv_clock_step) {
#if defined(__linux__)
    if (!may_set_clock()) {
        std::printf("  (bus_monotonic_cv_clock_step: not run here, it needs CAP_SYS_TIME to step CLOCK_REALTIME)\n");
        return;
    }
    // a core wait: CancelToken::wait_changed (cancellable sleeps, the bus lock's waits for a cancel)
    auto token = std::make_shared<CancelToken>();
    bool changed = true;
    check_200_ms(steady_seconds_with_a_held_clock_step("CancelToken::wait_changed(0.2 s)", [&] {
        changed = token->wait_changed(token->generation(), 0.2);
    }));
    CHECK(!changed);
#if IAD_TEST_HAS_MONOTONIC_CV
    detail::MonotonicCv cv;
    std::mutex m;
    std::cv_status status = std::cv_status::no_timeout;
    check_200_ms(steady_seconds_with_a_held_clock_step("MonotonicCv::wait_for(200 ms)", [&] {
        std::unique_lock<std::mutex> lock(m);
        const double end = steady_now_s() + 0.2;
        status = cv.wait_for(lock, std::chrono::milliseconds(200));
        while (status == std::cv_status::no_timeout && steady_now_s() < end)  // a spurious wake-up: wait out the rest
            status = cv.wait_for(lock, std::chrono::duration<double>(end - steady_now_s()));
    }));
    CHECK(status == std::cv_status::timeout);
    bool pred_result = true;
    check_200_ms(steady_seconds_with_a_held_clock_step("MonotonicCv::wait_for(200 ms, predicate)", [&] {
        std::unique_lock<std::mutex> lock(m);
        pred_result = cv.wait_for(lock, std::chrono::milliseconds(200), [] { return false; });
    }));
    CHECK(!pred_result);
    bool until_result = true;
    check_200_ms(steady_seconds_with_a_held_clock_step("MonotonicCv::wait_until(+200 ms, predicate)", [&] {
        std::unique_lock<std::mutex> lock(m);
        until_result = cv.wait_until(lock, std::chrono::steady_clock::now() + std::chrono::milliseconds(200),
                                     [] { return false; });
    }));
    CHECK(!until_result);
#endif
#endif
}

#if IAD_TEST_HAS_MONOTONIC_CV
// ============================================================================================ MonotonicCv

namespace {

struct Shared {
    std::mutex m;
    detail::MonotonicCv cv;
    bool flag = false;
    int woken = 0;
};

}  // namespace

TEST(bus_monotonic_cv_times_out) {
    Shared s;
    std::unique_lock<std::mutex> lock(s.m);
    double t0 = steady_now_s();
    std::cv_status status = s.cv.wait_for(lock, std::chrono::milliseconds(50));
    while (status == std::cv_status::no_timeout && steady_now_s() - t0 < 0.05)  // spurious: allowed without a predicate
        status = s.cv.wait_for(lock, std::chrono::milliseconds(50));
    double took = steady_now_s() - t0;
    CHECK(status == std::cv_status::timeout);
    CHECK(took >= 0.049);
    CHECK(took < 2.0);

    t0 = steady_now_s();
    CHECK(!s.cv.wait_for(lock, std::chrono::duration<double>(0.05), [&] { return s.flag; }));
    took = steady_now_s() - t0;
    CHECK(took >= 0.049);
    CHECK(took < 2.0);

    t0 = steady_now_s();
    CHECK(!s.cv.wait_until(lock, std::chrono::steady_clock::now() + std::chrono::milliseconds(50), [&] { return s.flag; }));
    took = steady_now_s() - t0;
    CHECK(took >= 0.049);
    CHECK(took < 2.0);
}

TEST(bus_monotonic_cv_zero_negative_and_nan_timeouts_do_not_wait) {
    Shared s;
    std::unique_lock<std::mutex> lock(s.m);
    const double t0 = steady_now_s();
    CHECK(s.cv.wait_for(lock, std::chrono::seconds(0)) == std::cv_status::timeout);
    CHECK(s.cv.wait_for(lock, std::chrono::milliseconds(-5)) == std::cv_status::timeout);
    CHECK(s.cv.wait_for(lock, std::chrono::duration<double>(std::nan(""))) == std::cv_status::timeout);
    CHECK(s.cv.wait_until(lock, std::chrono::steady_clock::now() - std::chrono::seconds(1)) == std::cv_status::timeout);
    CHECK(!s.cv.wait_for(lock, std::chrono::seconds(0), [&] { return s.flag; }));
    CHECK(!s.cv.wait_for(lock, std::chrono::duration<double>(-1.0), [&] { return s.flag; }));
    CHECK(!s.cv.wait_until(lock, std::chrono::steady_clock::time_point::min(), [&] { return s.flag; }));
    s.flag = true;
    CHECK(s.cv.wait_for(lock, std::chrono::seconds(0), [&] { return s.flag; }));  // the predicate's value
    CHECK(s.cv.wait_for(lock, std::chrono::duration<double>(std::nan("")), [&] { return s.flag; }));
    CHECK(steady_now_s() - t0 < 0.5);
    CHECK(lock.owns_lock());
}

TEST(bus_monotonic_cv_predicate_true_returns_at_once) {
    Shared s;
    s.flag = true;
    std::unique_lock<std::mutex> lock(s.m);
    const double t0 = steady_now_s();
    CHECK(s.cv.wait_for(lock, std::chrono::seconds(10), [&] { return s.flag; }));
    CHECK(s.cv.wait_until(lock, std::chrono::steady_clock::now() + std::chrono::seconds(10), [&] { return s.flag; }));
    s.cv.wait(lock, [&] { return s.flag; });
    CHECK(steady_now_s() - t0 < 0.5);
}

TEST(bus_monotonic_cv_notify_one_wakes_a_waiter) {
    Shared s;
    bool result = false;
    double took = 0.0;
    ec_test::Thread waiter([&] {
        std::unique_lock<std::mutex> lock(s.m);
        const double t0 = steady_now_s();
        result = s.cv.wait_for(lock, std::chrono::seconds(10), [&] { return s.flag; });
        took = steady_now_s() - t0;
    });
    sleep_s(0.05);
    {
        std::lock_guard<std::mutex> lock(s.m);
        s.flag = true;
    }
    s.cv.notify_one();
    waiter.join();
    CHECK(result);
    CHECK(took < 5.0);
}

TEST(bus_monotonic_cv_notify_all_wakes_every_waiter) {
    Shared s;
    std::atomic<int> done{0};
    {
        std::vector<std::unique_ptr<ec_test::Thread>> waiters;
        for (int i = 0; i < 4; ++i) {
            waiters.push_back(std::make_unique<ec_test::Thread>([&] {
                std::unique_lock<std::mutex> lock(s.m);
                ++s.woken;
                s.cv.wait(lock, [&] { return s.flag; });  // untimed, with a predicate
                ++done;
            }));
        }
        for (int i = 0; i < 500; ++i) {  // all four waiting (each counted itself under the mutex)
            {
                std::lock_guard<std::mutex> lock(s.m);
                if (s.woken == 4) break;
            }
            sleep_s(0.01);
        }
        {
            std::lock_guard<std::mutex> lock(s.m);
            s.flag = true;
        }
        s.cv.notify_all();
        for (auto& w : waiters) w->join();
    }
    CHECK_EQ(done.load(), 4);
}

TEST(bus_monotonic_cv_wait_without_a_predicate_returns_when_notified) {
    Shared s;
    std::cv_status status = std::cv_status::timeout;
    ec_test::Thread waiter([&] {
        std::unique_lock<std::mutex> lock(s.m);
        s.woken = 1;
        while (!s.flag) status = s.cv.wait_for(lock, std::chrono::seconds(10));
    });
    for (int i = 0; i < 500; ++i) {
        {
            std::lock_guard<std::mutex> lock(s.m);
            if (s.woken == 1) break;
        }
        sleep_s(0.01);
    }
    {
        std::lock_guard<std::mutex> lock(s.m);
        s.flag = true;
    }
    s.cv.notify_one();
    waiter.join();
    CHECK(status == std::cv_status::no_timeout);
}

TEST(bus_monotonic_cv_long_timeouts_do_not_overflow) {
    // each must wait (not time out at once from an overflowed deadline) until the flag is set
    Shared s;
    const auto run = [&](auto wait) {
        {
            std::lock_guard<std::mutex> lock(s.m);
            s.flag = false;
        }
        bool result = false;
        ec_test::Thread waiter([&] {
            std::unique_lock<std::mutex> lock(s.m);
            result = wait(lock);
        });
        sleep_s(0.05);
        {
            std::lock_guard<std::mutex> lock(s.m);
            s.flag = true;
        }
        s.cv.notify_all();
        waiter.join();
        return result;
    };
    const auto pred = [&] { return s.flag; };
    CHECK(run([&](std::unique_lock<std::mutex>& l) { return s.cv.wait_for(l, std::chrono::hours(24 * 365 * 100), pred); }));
    CHECK(run([&](std::unique_lock<std::mutex>& l) { return s.cv.wait_for(l, std::chrono::nanoseconds::max(), pred); }));
    CHECK(run([&](std::unique_lock<std::mutex>& l) { return s.cv.wait_for(l, std::chrono::seconds::max(), pred); }));
    CHECK(run([&](std::unique_lock<std::mutex>& l) {
        return s.cv.wait_for(l, std::chrono::duration<double>(std::numeric_limits<double>::infinity()), pred);
    }));
    CHECK(run([&](std::unique_lock<std::mutex>& l) { return s.cv.wait_for(l, std::chrono::duration<double>(1e300), pred); }));
    CHECK(run([&](std::unique_lock<std::mutex>& l) {
        return s.cv.wait_until(l, std::chrono::steady_clock::time_point::max(), pred);
    }));
    CHECK_NEAR(detail::MonotonicCv::wait_seconds(std::chrono::nanoseconds::max()), detail::MonotonicCv::kMaxWaitS, 0.0);
    CHECK_NEAR(detail::MonotonicCv::wait_seconds(std::chrono::milliseconds(1500)), 1.5, 1e-12);
    CHECK_NEAR(detail::MonotonicCv::wait_seconds(std::chrono::seconds(-3)), 0.0, 0.0);
}

TEST(bus_monotonic_cv_is_pthread_monotonic_on_linux) {
#if defined(__linux__)
    CHECK(detail::MonotonicCv::kPthreadMonotonic);
#elif defined(_WIN32)
    CHECK(!detail::MonotonicCv::kPthreadMonotonic);  // std::condition_variable: a relative timeout per wait
#endif
}
#endif
