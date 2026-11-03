// Cancellable waits (cancel.hpp). UNVERIFIED on hardware (simulator tests only).
#include "iai_actuator_driver/cancel.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "monotonic_cv.hpp"

namespace iai_actuator_driver {

namespace {

thread_local CancelScope* innermost_scope = nullptr;

}  // namespace

// ============================================================================================ CancelToken

CancelToken::CancelToken() : cv_(std::make_unique<detail::MonotonicCv>()) {}

CancelToken::~CancelToken() = default;

void CancelToken::cancel() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_;
    }
    cv_->notify_all();
}

void CancelToken::set_latch(bool on) {
    latched_.store(on);  // before the generation: a scope opening meanwhile sees one or the other
    if (on) cancel();
}

bool CancelToken::wait_changed(std::uint64_t seen, double seconds) const {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!(seconds > 0.0)) return generation_.load() != seen;
    return cv_->wait_for(lock, std::chrono::duration<double>(seconds), [&] { return generation_.load() != seen; });
}

// ============================================================================================ CancelScope

CancelScope::CancelScope(std::shared_ptr<CancelToken> token, bool sticky) : token_(std::move(token)), sticky_(sticky) {
    if (!token_) return;
    seen_ = token_->generation();
    if (token_->latched()) --seen_;  // latched (0.3): starts cancelled
    outer_ = innermost_scope;
    innermost_scope = this;
}

CancelScope::~CancelScope() {
    if (!token_) return;
    innermost_scope = outer_;
}

bool CancelScope::cancelled() const noexcept { return token_ && token_->generation() != seen_; }

// ============================================================================================ check points

bool cancel_pending() noexcept {
    if (StopPriority::active()) return false;
    for (const CancelScope* s = innermost_scope; s; s = s->outer_)
        if (s->cancelled()) return true;
    return false;
}

void check_cancel() {
    if (!cancel_pending()) return;
    for (CancelScope* s = innermost_scope; s; s = s->outer_)
        if (s->cancelled() && !s->sticky_) s->seen_ = s->token_->generation();  // delivered once: the clean-up runs
    throw WaitCancelledError("wait cancelled from another thread (cancel_waits)");
}

void cancellable_sleep(double seconds) {
    const CancelScope* s = StopPriority::active() ? nullptr : innermost_scope;
    if (!s) {
        sleep_at_least(seconds);
        return;
    }
    if (clock_installed()) {  // a test clock (timing.hpp): its sleep, with the check points around it
        check_cancel();
        sleep_at_least(seconds);
        check_cancel();
        return;
    }
    const double end = now_s() + (seconds > 0.0 ? seconds : 0.0);
    for (;;) {
        check_cancel();
        const double left = end - now_s();
        if (left <= 0.0) return;
        s->token_->wait_changed(s->seen_, std::min(left, kBusWaitSliceS));  // wakes at once on its own token
    }
}

}  // namespace iai_actuator_driver
