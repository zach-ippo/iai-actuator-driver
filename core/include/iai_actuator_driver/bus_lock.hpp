// The bus lock of a ModbusRtu (one transaction at a time per port), handed over fairly and with STOP priority, and
// the per-thread scopes that steer it: StopPriority and PreemptibleReads. Implemented by src/bus_lock.cpp; ModbusRtu
// takes it for every request (modbus.hpp).
//
// Why: std::mutex is not fair. A status monitor polling back to back (AxisMonitor with gap 0, several axes on one
// line) re-takes it at once and can make a STOP from another thread wait behind many polls. The rules:
// * FIFO: requests waiting for the bus get it in the order they asked for it.
// * Stop priority: a stop-class request goes before every waiting normal request (FIFO among stop-class requests), so
//   it waits for the one transaction in flight at most. Stop-class are the requests a thread makes inside a
//   StopPriority scope (Axis::stop / begin_stop / pause / servo OFF / cancel_paused_move and the decel stop sent after
//   an error, Port::broadcast_stop) and, whoever sends them, the frames that stop motion: FC05 to the STOP coil 0x042C
//   (ON and OFF), the pause coil STP 0x040A ON and the servo coil SON 0x0403 OFF, addressed or broadcast. A request
//   that retries lets a waiting stop-class request go between two of its attempts; a write is then not sent again
//   (STOP first, modbus.hpp: EcNotReadyError "a STOP went first; not re-sent"), a read is.
// * Recursive for the owning thread: a multi-frame sequence that holds the bus (baud detection) keeps it throughout,
//   and a STOP waits for the whole sequence there (sending it at a probe rate would not reach the unit anyway).
// * A wait for the bus runs the caller's check (cancellation, cancel.hpp; the bus closed) before waiting and at least
//   every kBusWaitSliceS while waiting: a closed bus or a cancelled wait ends it at once, the lock not taken.
// * PreemptibleReads (port of the Python edition's GUI _GatedBus, apps/gui/backend.py): inside it, the thread's FC03 reads go
//   out in frames of at most `words` registers (32, the GUI's GATED_READ_WORDS), every attempt takes the bus on its own
//   (retries outside the lock, the optional gate before each) and every write, IAI command, broadcast and baud change of
//   that thread is refused (UnsafeCommandError, nothing sent). A long read-only read (unit info, a backup, a register
//   browser) then never makes a STOP wait for more than one short request. A StopPriority scope inside it wins: a stop
//   always goes out. A scope covers every bus of the thread, or one bus (PreemptibleReadOptions::bus; the C ABI's
//   iad_bus_preemptible_begin): the requests on other buses go out as usual.
//
// Evidence: UNVERIFIED on hardware (simulator and scripted-frame tests only). The default frames on the wire are
// unchanged: reads outside PreemptibleReads still go out in kMaxReadRegisters (60) words, retries stay inside the lock
// unless a stop-class request waits (the Python edition's modbus.py rule).
#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace iai_actuator_driver {

namespace detail {
class MonotonicCv;  // src/monotonic_cv.hpp (internal)
}  // namespace detail

/// Registers per FC03 frame inside PreemptibleReads (the Python GUI's GATED_READ_WORDS).
inline constexpr int kPreemptibleReadWords = 32;
/// A thread waiting for the bus runs its check at least this often (s): cancellation and close are seen this soon.
inline constexpr double kBusWaitSliceS = 0.02;

/// Who goes first when the bus is free.
enum class BusPriority {
    Normal,  ///< FIFO
    Stop,    ///< before every waiting Normal request (FIFO among themselves)
};

/// "normal" / "stop".
inline const char* to_string(BusPriority p) noexcept { return p == BusPriority::Stop ? "stop" : "normal"; }

/// The fair, recursive bus lock with stop priority (see the file comment). Thread-safe; not copyable.
class BusLock {
public:
    BusLock();
    ~BusLock();
    BusLock(const BusLock&) = delete;
    BusLock& operator=(const BusLock&) = delete;

    /// Take the lock for the calling thread (recursive: the owner only counts up). `check` (optional) runs before the
    /// wait and at least every kBusWaitSliceS while waiting, never with the lock's own mutex held; whatever it throws
    /// ends the wait (the lock is not taken, the queue place given up) and propagates.
    void lock(BusPriority priority, const std::function<void()>& check = {});
    /// BasicLockable: lock(Normal) without a check.
    void lock() { lock(BusPriority::Normal); }
    /// Take the lock when it is free and nobody waits (or the caller owns it); never blocks.
    bool try_lock();
    /// Give one level back; the last one hands the bus to the next waiter (stop-class first, then FIFO).
    /// std::logic_error when the calling thread does not own the lock.
    void unlock();

    /// Between two attempts of one request of the owner (held exactly once): when a stop-class request waits, let it
    /// (and every stop-class request queued meanwhile) go first, then take the lock back before every waiting normal
    /// request. Returns true when it yielded. No-op (false) when nothing stop-class waits or the lock is held
    /// recursively (a sequence that must not be interrupted). `check` as for lock(), while waiting to take it back: when
    /// it throws, the lock is NOT held any more (the caller must not unlock).
    bool yield_to_stop(const std::function<void()>& check = {});

    /// True when the calling thread owns the lock.
    bool owned_by_this_thread() const;
    /// The calling thread's recursion depth (0 when it does not own the lock).
    int depth() const;
    /// Threads waiting with this priority now.
    std::size_t waiting(BusPriority priority) const;
    /// How often the lock went to a new owner (for tests and diagnostics).
    std::uint64_t handovers() const;
    /// Wake every waiter now, so that it runs its check (ModbusRtu::close: they end with TransportError at once).
    void wake_all();

private:
    struct Waiter;
    bool is_next(const Waiter* w) const;   // with mutex_ held
    void remove(const Waiter* w);          // with mutex_ held
    /// Wait (mutex_ held through `lock`) until `w` is next; runs `check` every slice. Takes the lock on success.
    void wait_turn(std::unique_lock<std::mutex>& lock, Waiter& w, const std::function<void()>& check);

    mutable std::mutex mutex_;
    std::unique_ptr<detail::MonotonicCv> cv_;  ///< its timed waits run on the monotonic clock
    std::thread::id owner_;
    int depth_ = 0;
    std::deque<Waiter*> stop_queue_;
    std::deque<Waiter*> normal_queue_;
    std::uint64_t handovers_ = 0;
};

/// RAII: while it lives, every bus request of the calling thread is stop-class (BusPriority::Stop) and no
/// cancellation (cancel.hpp) interrupts it. Nestable; not copyable. Used by every stop path of the axis.
class StopPriority {
public:
    StopPriority() noexcept;
    ~StopPriority();
    StopPriority(const StopPriority&) = delete;
    StopPriority& operator=(const StopPriority&) = delete;
    /// True inside a StopPriority scope of the calling thread.
    static bool active() noexcept;
};

/// Options of a PreemptibleReads scope.
struct PreemptibleReadOptions {
    int words = kPreemptibleReadWords;  ///< registers per FC03 frame, 1..125 (std::invalid_argument otherwise)
    /// Runs before every request attempt, outside the bus lock (the GUI's STOP-epoch check): throw to end the read.
    std::function<void()> gate;
    /// Only the requests of the bus with this lock (ModbusRtu::bus_lock()); null: every bus of the thread. Compared,
    /// never dereferenced.
    const BusLock* bus = nullptr;
};

/// RAII: preemptible read-only reads for the calling thread (see the file comment). Nestable (the innermost scope
/// that covers a bus applies to it); not copyable. No effect inside a StopPriority scope.
class PreemptibleReads {
public:
    explicit PreemptibleReads(PreemptibleReadOptions options = {});
    ~PreemptibleReads();
    PreemptibleReads(const PreemptibleReads&) = delete;
    PreemptibleReads& operator=(const PreemptibleReads&) = delete;
    /// The innermost scope's options of the calling thread; null outside one or inside a StopPriority scope.
    static const PreemptibleReadOptions* current() noexcept;
    /// The innermost scope of the calling thread that covers `bus` (its options' bus null or this one); null when none
    /// does or inside a StopPriority scope. ModbusRtu asks this before every request.
    static const PreemptibleReadOptions* current(const BusLock& bus) noexcept;
    /// This scope's options.
    const PreemptibleReadOptions& options() const noexcept { return options_; }

private:
    PreemptibleReadOptions options_;
    const PreemptibleReads* previous_;
};

}  // namespace iai_actuator_driver
