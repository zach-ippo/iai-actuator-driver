// The fair bus lock with stop priority and the StopPriority / PreemptibleReads scopes (bus_lock.hpp). UNVERIFIED on
// hardware (simulator and scripted-frame tests only).
#include "iai_actuator_driver/bus_lock.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "monotonic_cv.hpp"

namespace iai_actuator_driver {

namespace {

thread_local int stop_priority_depth = 0;
thread_local const PreemptibleReads* preemptible_reads = nullptr;  // the innermost scope of the thread

}  // namespace

// ============================================================================================ BusLock

struct BusLock::Waiter {
    BusPriority priority;
};

BusLock::BusLock() : cv_(std::make_unique<detail::MonotonicCv>()) {}

BusLock::~BusLock() = default;

bool BusLock::is_next(const Waiter* w) const {
    if (owner_ != std::thread::id()) return false;
    if (!stop_queue_.empty()) return stop_queue_.front() == w;
    return !normal_queue_.empty() && normal_queue_.front() == w;
}

void BusLock::remove(const Waiter* w) {
    auto& queue = w->priority == BusPriority::Stop ? stop_queue_ : normal_queue_;
    queue.erase(std::remove(queue.begin(), queue.end(), w), queue.end());
}

void BusLock::wait_turn(std::unique_lock<std::mutex>& lock, Waiter& w, const std::function<void()>& check) {
    const auto slice = std::chrono::duration<double>(kBusWaitSliceS);
    while (!is_next(&w)) {
        cv_->wait_for(lock, slice);
        if (is_next(&w) || !check) continue;
        lock.unlock();  // the check never runs under the lock's own mutex (it may log, read flags, throw)
        try {
            check();
        } catch (...) {
            lock.lock();
            remove(&w);
            lock.unlock();
            cv_->notify_all();  // the next waiter may be at the head now
            throw;
        }
        lock.lock();
    }
    remove(&w);
    owner_ = std::this_thread::get_id();
    depth_ = 1;
    ++handovers_;
}

void BusLock::lock(BusPriority priority, const std::function<void()>& check) {
    const std::thread::id me = std::this_thread::get_id();
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (owner_ == me) {
            ++depth_;
            return;
        }
    }
    if (check) check();  // before queueing: a cancelled call or a closed bus never waits
    std::unique_lock<std::mutex> lock(mutex_);
    Waiter w{priority};
    (priority == BusPriority::Stop ? stop_queue_ : normal_queue_).push_back(&w);
    wait_turn(lock, w, check);
}

bool BusLock::try_lock() {
    std::lock_guard<std::mutex> guard(mutex_);
    const std::thread::id me = std::this_thread::get_id();
    if (owner_ == me) {
        ++depth_;
        return true;
    }
    if (owner_ != std::thread::id() || !stop_queue_.empty() || !normal_queue_.empty()) return false;
    owner_ = me;
    depth_ = 1;
    ++handovers_;
    return true;
}

void BusLock::unlock() {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (owner_ != std::this_thread::get_id() || depth_ <= 0)
            throw std::logic_error("BusLock::unlock by a thread that does not own the lock");
        if (--depth_ > 0) return;
        owner_ = std::thread::id();
    }
    cv_->notify_all();
}

bool BusLock::yield_to_stop(const std::function<void()>& check) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (owner_ != std::this_thread::get_id() || depth_ != 1 || stop_queue_.empty()) return false;
    owner_ = std::thread::id();
    depth_ = 0;
    Waiter w{BusPriority::Normal};
    normal_queue_.push_front(&w);  // back before every waiting normal request, after every stop-class one
    cv_->notify_all();
    wait_turn(lock, w, check);
    return true;
}

bool BusLock::owned_by_this_thread() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return owner_ == std::this_thread::get_id();
}

int BusLock::depth() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return owner_ == std::this_thread::get_id() ? depth_ : 0;
}

std::size_t BusLock::waiting(BusPriority priority) const {
    std::lock_guard<std::mutex> guard(mutex_);
    return priority == BusPriority::Stop ? stop_queue_.size() : normal_queue_.size();
}

std::uint64_t BusLock::handovers() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return handovers_;
}

void BusLock::wake_all() { cv_->notify_all(); }

// ============================================================================================ the scopes

StopPriority::StopPriority() noexcept { ++stop_priority_depth; }
StopPriority::~StopPriority() { --stop_priority_depth; }
bool StopPriority::active() noexcept { return stop_priority_depth > 0; }

PreemptibleReads::PreemptibleReads(PreemptibleReadOptions options)
    : options_(std::move(options)), previous_(preemptible_reads) {
    if (options_.words < 1 || options_.words > 125)
        throw std::invalid_argument("PreemptibleReads: words must be 1..125, got " + std::to_string(options_.words));
    preemptible_reads = this;
}

PreemptibleReads::~PreemptibleReads() { preemptible_reads = previous_; }

const PreemptibleReadOptions* PreemptibleReads::current() noexcept {
    if (StopPriority::active() || !preemptible_reads) return nullptr;
    return &preemptible_reads->options_;
}

const PreemptibleReadOptions* PreemptibleReads::current(const BusLock& bus) noexcept {
    if (StopPriority::active()) return nullptr;
    for (const PreemptibleReads* scope = preemptible_reads; scope; scope = scope->previous_)
        if (!scope->options_.bus || scope->options_.bus == &bus) return &scope->options_;
    return nullptr;
}

}  // namespace iai_actuator_driver
