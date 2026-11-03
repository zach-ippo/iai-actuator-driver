// Cancellable waits: another thread ends a blocking call soon (Axis::cancel_waits; a status monitor's stop() ends its
// own poll the same way). The bindings turn Ctrl+C into it, so that a long move, home or wait ends with a decel stop
// instead of running on. Implemented by src/bus_cancel.cpp (the bus unit: every request checks it).
//
// A CancelToken belongs to an axis (or to a monitor thread). A blocking call opens a CancelScope on it in the thread
// that waits (Axis: every motion wait, the home wait and the servo wait). CancelToken::cancel(), from any thread, ends
// every scope open at that moment: the waiting thread throws WaitCancelledError at its next check point, which is
// * before each bus request and each retry, and while it waits for the bus lock (at least every kBusWaitSliceS),
// * in cancellable_sleep() (at once) and in the status monitor's waits (at least every kBusWaitSliceS).
// A request already on the wire is finished first (its response timeout at most; a few ms on a healthy line). The
// error is delivered once per scope: the clean-up that follows it (the decel stop of a motion wait, the HOME coil
// release) runs normally. Scopes opened after cancel() are not affected. Nothing inside a StopPriority scope
// (bus_lock.hpp) is ever cancelled, and a sequence that holds the bus (baud detection) is not interrupted midway.
//
// Rules for the bindings (Python ctypes, C# P/Invoke; the ABI's iad_axis_cancel calls Axis::cancel_waits):
// * Ctrl+C: a blocking call cannot see a signal while it runs in native code (Python defers KeyboardInterrupt until
//   the ctypes call returns). Python: run the blocking call in a helper thread and wait for it in short joins on the
//   calling thread; on KeyboardInterrupt call the cancel entry, wait for the call to return (it has sent the decel
//   stop) and re-raise. C#: Console.CancelKeyPress (its own thread) calls the cancel entry. WaitCancelledError maps to
//   KeyboardInterrupt / OperationCanceledException.
// * Process exit: close every cell, port and axis before the runtime tears down (Python: atexit, which runs before
//   module teardown, plus weakref.finalize; C#: AppDomain.ProcessExit plus SafeHandle). Close stops and joins every
//   core thread of the object: status monitors (a poll waiting for the bus ends at once, one on the wire within its
//   timeout), link watches, the cell's worker threads. A call blocked on an object being closed ends with
//   TransportError (bus closed) or WaitCancelledError: cancel its waits first, then close. Never unload the library
//   while a core thread runs, and never call into it from a destructor that may run after unloading.
//
// Evidence: UNVERIFIED on hardware (simulator tests only).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>

namespace iai_actuator_driver {

namespace detail {
class MonotonicCv;  // src/monotonic_cv.hpp (internal)
}  // namespace detail

/// The cancellation state shared by an axis (or a monitor thread) and the calls that wait on it. Thread-safe.
class CancelToken {
public:
    CancelToken();
    ~CancelToken();
    CancelToken(const CancelToken&) = delete;
    CancelToken& operator=(const CancelToken&) = delete;

    /// End every CancelScope open on this token now (their threads throw WaitCancelledError at the next check point).
    void cancel();
    /// Counts cancel() calls.
    std::uint64_t generation() const noexcept { return generation_.load(); }
    /// Block up to `seconds`, or until the generation differs from `seen`; true when it does.
    bool wait_changed(std::uint64_t seen, double seconds) const;
    /// (ABI 0.3) The latch: while set, every scope opened on this token starts cancelled (its first check point
    /// throws), so a cancel that comes before a call opens its scope is not lost. set_latch(true) also cancels the
    /// scopes open now; set_latch(false) clears it (scopes open keep their state).
    void set_latch(bool on);
    bool latched() const noexcept { return latched_.load(); }

private:
    std::atomic<std::uint64_t> generation_{0};
    std::atomic<bool> latched_{false};
    mutable std::mutex mutex_;
    const std::unique_ptr<detail::MonotonicCv> cv_;  ///< its timed waits run on the monotonic clock
};

/// RAII: the calling thread waits on `token` while it lives (a null token: no effect). Nestable; not copyable.
/// `sticky`: after cancel() every later check point of the scope throws too (a thread that is ending: a monitor's poll
/// loop), instead of once.
class CancelScope {
public:
    explicit CancelScope(std::shared_ptr<CancelToken> token, bool sticky = false);
    ~CancelScope();
    CancelScope(const CancelScope&) = delete;
    CancelScope& operator=(const CancelScope&) = delete;
    /// cancel() was called since this scope opened (or since its error was delivered).
    bool cancelled() const noexcept;

private:
    friend void check_cancel();
    friend bool cancel_pending() noexcept;
    friend void cancellable_sleep(double seconds);
    std::shared_ptr<CancelToken> token_;
    std::uint64_t seen_ = 0;
    bool sticky_ = false;
    CancelScope* outer_ = nullptr;
};

/// Throw WaitCancelledError when a scope of the calling thread was cancelled (then marked delivered). No-op inside a
/// StopPriority scope or without a scope.
void check_cancel();
/// Would check_cancel() throw now? (no side effect)
bool cancel_pending() noexcept;
/// Sleep at least `seconds` (sleep_at_least), ending early with WaitCancelledError when the calling thread's scope is
/// cancelled meanwhile.
void cancellable_sleep(double seconds);

}  // namespace iai_actuator_driver
