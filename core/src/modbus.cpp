// Modbus RTU master (port of the Python edition's modbus.ModbusRTU; C# ModbusRtu.cs): one transaction at a time per
// port, the minimum gap, retries on timeouts / CRC / frame errors, the IAI 0x45 commands with their safe and
// never-send lists, broadcasts, baud detection in place and the per-request response timeout. The behaviour is VERIFIED
// in the Python edition; this edition is UNVERIFIED on hardware.
#include "iai_actuator_driver/modbus.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

constexpr const char* kLogSource = "iai_actuator_driver.modbus";

void check_u16(const char* name, long long value) {
    if (value < 0 || value > 0xFFFF)
        throw std::invalid_argument(util::format("%s must be 0..65535, got %lld", name, value));
}

void check_slave(int slave) {
    if (slave < 1 || slave > 247)
        throw std::invalid_argument(
            util::format("slave address must be 1..247 (broadcast: use broadcast()), got %d", slave));
    if (slave == kAsciiStartSlave) throw std::invalid_argument(std::string(kAsciiStartSlaveNote) + "; nothing sent");
}

/// PDU of FC03 / FC05 / FC06: function, two big-endian words.
std::vector<std::uint8_t> pdu_words(int function, int a, int b) {
    return {static_cast<std::uint8_t>(function), static_cast<std::uint8_t>((a >> 8) & 0xFF),
            static_cast<std::uint8_t>(a & 0xFF), static_cast<std::uint8_t>((b >> 8) & 0xFF),
            static_cast<std::uint8_t>(b & 0xFF)};
}

int be16(const std::vector<std::uint8_t>& pdu, std::size_t at) { return (pdu[at] << 8) | pdu[at + 1]; }

/// Registers a request touches (the manual's internal time is 1 ms per register): FC03 / 04 / 10 count, FC17 read +
/// write count, coils / inputs in 16-bit words, 1 for everything else.
int register_count(const std::vector<std::uint8_t>& pdu) {
    if (pdu.empty()) return 1;
    const int function = pdu[0];
    if ((function == 0x03 || function == 0x04 || function == 0x10) && pdu.size() >= 5)
        return std::max(1, be16(pdu, 3));
    if ((function == 0x01 || function == 0x02 || function == 0x0F) && pdu.size() >= 5)
        return std::max(1, (be16(pdu, 3) + 15) / 16);
    if (function == 0x17 && pdu.size() >= 9) return std::max(1, be16(pdu, 3) + be16(pdu, 7));
    return 1;
}

std::vector<std::uint8_t> concat(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    std::vector<std::uint8_t> out(a);
    out.insert(out.end(), b.begin(), b.end());
    return out;
}

bool is_retryable(const std::exception& e) {
    return dynamic_cast<const ModbusTimeout*>(&e) || dynamic_cast<const ModbusCrcError*>(&e) ||
           dynamic_cast<const ModbusFrameError*>(&e);
}

std::string join_ints(const std::vector<int>& values) {
    std::vector<std::string> parts;
    for (int v : values) parts.push_back(std::to_string(v));
    return util::join(parts, ", ");
}

/// A frame that stops motion, whoever sends it (bus_lock.hpp): FC05 to the STOP coil (ON and OFF), STP ON (pause),
/// SON OFF (servo off). It goes before every waiting normal request.
bool is_stop_frame(const std::vector<std::uint8_t>& pdu) {
    if (pdu.size() < 5 || pdu[0] != kFcWriteCoil) return false;
    const int address = be16(pdu, 1);
    const int value = be16(pdu, 3);
    return address == reg::kCoilStop || (address == reg::kCoilStp && value == kCoilOn) ||
           (address == reg::kCoilSon && value == kCoilOff);
}

/// The bus priority of a request of the calling thread: stop-class inside a StopPriority scope or for a stop frame.
BusPriority priority_of(const std::vector<std::uint8_t>& pdu) {
    return StopPriority::active() || is_stop_frame(pdu) ? BusPriority::Stop : BusPriority::Normal;
}

/// FC03 / FC11: what a preemptible read may send.
bool is_read_request(const std::vector<std::uint8_t>& pdu) {
    return !pdu.empty() && (pdu[0] == kFcReadHoldingRegisters || pdu[0] == kFcReportSlaveId);
}

/// A request that changes nothing on the unit: FC01 / 02 / 03 / 04 / 11. Everything else counts as a write.
bool is_read_only(const std::vector<std::uint8_t>& pdu) {
    return !pdu.empty() && (pdu[0] == 0x01 || pdu[0] == 0x02 || pdu[0] == kFcReadHoldingRegisters || pdu[0] == 0x04 ||
                            pdu[0] == kFcReportSlaveId);
}

/// A frame that brings motion to a halt: FC05 STOP coil ON, STP ON, SON OFF (not the STOP coil OFF that releases it).
bool engages_stop(const std::vector<std::uint8_t>& pdu) {
    if (!is_stop_frame(pdu)) return false;
    return !(be16(pdu, 1) == reg::kCoilStop && be16(pdu, 3) == kCoilOff);
}

/// A write the STOP-first gate counts (detail::StopMark): a stop frame, or any write inside a StopPriority scope.
bool is_stop_write(const std::vector<std::uint8_t>& pdu) {
    return !is_read_only(pdu) && (StopPriority::active() || is_stop_frame(pdu));
}

/// A write the STOP-first gate checks: not read-only, not a frame that engages a stop, not inside a StopPriority scope.
bool is_gated_write(const std::vector<std::uint8_t>& pdu) {
    return !is_read_only(pdu) && !StopPriority::active() && !engages_stop(pdu);
}

thread_local detail::SendGate* send_gates = nullptr;  // the innermost SendGate scope of the thread
thread_local detail::StopWireStamp* stop_wire_stamps = nullptr;  // the innermost StopWireStamp scope of the thread

/// What is left of the minimum gap a stop-class request waits with the bus held is spun on the clock for at most this
/// long (a sleep can be stretched by the timer resolution or the scheduler); a longer gap (min_gap_s set high) is slept
/// down to it first.
constexpr double kGapSpinS = 0.005;

/// A write, IAI command, broadcast or baud change from inside a PreemptibleReads scope (bus_lock.hpp).
[[noreturn]] void refuse_in_preemptible(const std::string& what) {
    throw UnsafeCommandError(what +
                             " refused: a preemptible read-only read runs in this thread (PreemptibleReads); nothing sent");
}

/// Registers per FC03 frame: `wanted`, at most the words of the PreemptibleReads scope covering the bus of `lock`.
int read_chunk(const BusLock& lock, int wanted) {
    const PreemptibleReadOptions* pre = PreemptibleReads::current(lock);
    return pre ? std::min(wanted, pre->words) : wanted;
}

}  // namespace

// ------------------------------------------------------------------------------------------ the STOP-first gate

namespace detail {

/// The scopes of the calling thread (modbus.hpp SendGate), for ModbusRtu::Impl.
struct SendGateAccess {
    template <class F>
    static void each(const BusLock& bus, int slave, F&& f) {  // slave 0: every scope of the bus
        for (SendGate* g = send_gates; g; g = g->previous_)
            if (g->bus_ == &bus && (slave == 0 || g->slave_ == slave)) f(*g);
    }
    static StopMark& expected(SendGate& g) { return g.expected_; }
    static const std::function<void()>& check(const SendGate& g) { return g.check_; }
    static void passed(SendGate& g) { ++g.passed_; }
};

SendGate::SendGate(const ModbusRtu& bus, int slave, StopMark expected, std::function<void()> check)
    : bus_(&bus.bus_lock()), slave_(slave), expected_(expected), check_(std::move(check)), previous_(send_gates) {
    send_gates = this;
}

SendGate::~SendGate() { send_gates = previous_; }

/// The StopWireStamp scopes of the calling thread, for ModbusRtu::Impl.
struct StopWireAccess {
    /// A stop-class write of this thread goes to the wire on `bus` now: every open scope of the bus without a wire
    /// time yet notes it.
    static void wire(const BusLock& bus) {
        double now = 0.0;
        bool taken = false;
        for (StopWireStamp* s = stop_wire_stamps; s; s = s->previous_) {
            if (s->bus_ != &bus || s->wire_) continue;
            if (!taken) {
                now = now_s();
                taken = true;
            }
            s->wire_ = now;
        }
    }
};

StopWireStamp::StopWireStamp(const ModbusRtu& bus) : bus_(&bus.bus_lock()), previous_(stop_wire_stamps) {
    stop_wire_stamps = this;
}

StopWireStamp::~StopWireStamp() { stop_wire_stamps = previous_; }

BusTestHooks& bus_test_hooks() {
    static BusTestHooks hooks;
    return hooks;
}

std::vector<int> skip_ascii_start_slave(const std::vector<int>& slaves, const std::string& log_source,
                                        const std::string& where) {
    std::vector<int> kept;
    kept.reserve(slaves.size());
    for (int s : slaves)
        if (s != kAsciiStartSlave) kept.push_back(s);
    if (kept.empty()) return slaves;  // 58 alone: refused by the request itself
    if (kept.size() != slaves.size()) log_warning(log_source, where + ": " + kAsciiStartSlaveNote + ": skipped");
    return kept;
}

}  // namespace detail

std::string hex_frame(const std::vector<std::uint8_t>& bytes) {
    std::string out;
    out.reserve(bytes.size() * 3);
    static const char digits[] = "0123456789ABCDEF";
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) out += ' ';
        out += digits[bytes[i] >> 4];
        out += digits[bytes[i] & 0x0F];
    }
    return out;
}

// ------------------------------------------------------------------------------------------ Impl

struct ModbusRtu::Impl {
    // replaced only by set_transport, with the bus lock and transport_mutex held: read it freely under the bus lock,
    // else through current_transport()
    std::shared_ptr<Transport> transport;
    mutable std::mutex transport_mutex;
    std::vector<std::shared_ptr<Transport>> retired;  // replaced transports, kept while the bus lives (transport_mutex)
    // the bus lock: fair (FIFO), stop-class requests first, recursive (detect_baud_rate holds it around probes);
    // bus_lock.hpp. Taken through Hold.
    mutable BusLock mutex;
    std::recursive_mutex sequence;       // multi-frame sequences of one axis
    mutable std::mutex delay_mutex;      // guards response_delay_ms (never waits behind the bus)
    std::atomic<int> retries{kRecommendedRetries};
    std::atomic<double> min_gap_s{kDefaultMinGapS};
    std::atomic<bool> closed{false};
    // now_s() at the end of the last frame: written under the bus lock, read outside it too (the gap is waited before
    // the bus is taken)
    std::atomic<double> last_io{0.0};
    std::map<int, double> response_delay_ms;    // guarded by delay_mutex
    std::mutex logger_mutex;
    FrameLogger frame_logger;                   // guarded by logger_mutex
    mutable std::mutex line_mutex;
    std::shared_ptr<Line> line = std::make_shared<Line>();  // guarded by line_mutex (line.hpp)
    std::map<int, std::set<int>> unsupported;   // slave -> functions answered with exception 01 (line_mutex)
    // stop-class writes sent so far (detail::StopMark): [slave] for 1..247, [0] broadcasts; bumped under the bus lock
    std::array<std::atomic<std::uint64_t>, 248> stops{};
    std::atomic<std::uint64_t> stops_total{0};  // all of them, to every slave and broadcast (a broadcast write's gate)
    // the stops of each slave every Axis object on this bus counted (detail::halt_count); [0] unused
    std::array<std::atomic<std::uint64_t>, 248> halts{};

    detail::StopMark mark(int slave) const {
        detail::StopMark m;
        m.slave = slave >= 1 && slave <= 247 ? stops[static_cast<std::size_t>(slave)].load() : 0;
        m.broadcast = stops[0].load();
        return m;
    }

    /// A stop-class write to `slave` (0: broadcast) goes to the wire now (bus lock held): count it, and move on the
    /// calling thread's own scopes that had seen everything before it (its own STOP never refuses its own frames).
    void count_stop_write(int slave) {
        const std::size_t at = static_cast<std::size_t>(slave);
        const std::uint64_t before = stops[at].load();
        detail::SendGateAccess::each(mutex, slave, [&](detail::SendGate& g) {
            detail::StopMark& e = detail::SendGateAccess::expected(g);
            std::uint64_t& seen = slave == 0 ? e.broadcast : e.slave;
            if (seen == before) seen = before + 1;
        });
        stops[at].store(before + 1);
        stops_total.fetch_add(1);
    }

    /// The checks of the calling thread's scopes on `slave` (while a gated write waits for the bus, and at the gate).
    void scope_checks(int slave) const {
        detail::SendGateAccess::each(mutex, slave, [](detail::SendGate& g) {
            if (const auto& check = detail::SendGateAccess::check(g)) check();
        });
    }

    /// The STOP-first gate of a write about to go to the wire (bus lock held, after the gap): refused when a stop-class
    /// write to `slave` or a broadcast one went since a scope of this thread decided on it, or since this request began
    /// (`since`: the stop mark taken when the request was made, before it waited for the bus, and moved on past its own
    /// stop-class frames). EcNotReadyError, stop_attempted set; nothing sent.
    void gate(int slave, int function, int attempt, const detail::StopMark& since,
              const std::exception_ptr& last_error) {
        scope_checks(slave);
        const detail::StopMark now = mark(slave);
        bool refused = now != since;
        detail::SendGateAccess::each(mutex, slave, [&](detail::SendGate& g) {
            if (now != detail::SendGateAccess::expected(g)) refused = true;
        });
        if (!refused) {
            detail::SendGateAccess::each(mutex, slave, [](detail::SendGate& g) { detail::SendGateAccess::passed(g); });
            return;
        }
        std::string message;
        if (attempt == 0) {
            message = util::format("function 0x%02X to slave %d: a STOP arrived meanwhile (to this unit or broadcast) "
                                   "after this command was decided; nothing sent",
                                   function, slave);
        } else {
            std::string why = "no valid answer";
            try {
                if (last_error) std::rethrow_exception(last_error);
            } catch (const std::exception& e) {
                why = e.what();
            } catch (...) {
            }
            message = util::format("function 0x%02X to slave %d: a STOP went first (to this unit or broadcast) after "
                                   "attempt %d got no valid answer (%s); not re-sent (an earlier attempt may have "
                                   "reached the unit before the STOP)",
                                   function, slave, attempt, why.c_str());
        }
        log_info(kLogSource, message);
        EcNotReadyError error(message);
        error.set_stop_attempted();
        throw error;
    }

    std::shared_ptr<Line> current_line() const {
        std::lock_guard<std::mutex> lock(line_mutex);
        return line;
    }

    std::shared_ptr<Transport> current_transport() const {
        std::lock_guard<std::mutex> lock(transport_mutex);
        return transport;
    }

    // ---- helpers (call with `mutex` held unless noted)

    /// A frame for the frame logger: passed on right after this thread lets the bus go (Hold defers the calls into
    /// the caller's code; log.hpp), to the logger installed then.
    void log_frame(FrameDirection direction, const std::vector<std::uint8_t>& frame) {
        {
            std::lock_guard<std::mutex> lock(logger_mutex);
            if (!frame_logger) return;
        }
        detail::call_or_defer([this, direction, frame] { call_frame_logger(direction, frame); });
    }

    void call_frame_logger(FrameDirection direction, const std::vector<std::uint8_t>& frame) {
        FrameLogger logger;
        {
            std::lock_guard<std::mutex> lock(logger_mutex);
            logger = frame_logger;
        }
        if (!logger) return;
        try {
            logger(direction, frame);
        } catch (const std::exception& e) {
            log_error(kLogSource, std::string("frame logger raised: ") + e.what());  // never breaks communication
        } catch (...) {
            log_error(kLogSource, "frame logger raised");
        }
    }

    /// Stamps last_io when a frame's scope ends (also on an error): the gap counts from the end of every frame.
    struct IoStamp {
        std::atomic<double>& last_io;
        ~IoStamp() { last_io.store(now_s()); }
    };

    /// When the minimum gap after the last frame ends (now_s()).
    double gap_end() const { return last_io.load() + min_gap_s.load(); }

    /// The gap of a normal request, waited before it takes the bus: a STOP never waits behind that sleep. A stop-class
    /// request does not wait here (it queues at once and waits the rest with the bus held, wait_gap_held), nor does a
    /// thread that holds the bus already (a sequence: the rest is waited there).
    void wait_gap_unlocked(bool stop_class) const {
        if (stop_class || mutex.owned_by_this_thread()) return;
        const double left = gap_end() - now_s();
        if (!(left > 0.0)) return;
        if (const auto& hook = detail::bus_test_hooks().before_gap_wait) hook(left, false);
        sleep_at_least(gap_end() - now_s());
    }

    /// What is left of the gap with the bus held (the bus was handed over right after a frame, or a sequence holds
    /// it): a stop-class request spins on the clock for it, a normal one sleeps. Then, with `may_yield`, a stop-class
    /// request that waits for the bus goes first (BusLock::yield_to_stop) and what is left of the gap after its
    /// frame is waited again.
    class Hold;
    void wait_gap_held(Hold& hold, bool stop_class, bool may_yield) {
        for (;;) {
            const double left = gap_end() - now_s();
            if (left > 0.0) {
                if (const auto& hook = detail::bus_test_hooks().before_gap_wait) hook(left, true);
                if (stop_class)
                    spin_until_gap_end();
                else
                    sleep_at_least(gap_end() - now_s());
            }
            if (!may_yield || !hold.yield_to_stop()) return;
        }
    }

    /// A stop-class request's rest of the gap: the last kGapSpinS spun on the clock (no oversleep), anything before it
    /// slept. A clock installed through the seam (a test's virtual clock) only moves when slept on: slept through it.
    void spin_until_gap_end() const {
        const double until = gap_end();
        if (clock_installed()) {
            sleep_at_least(until - now_s());
            return;
        }
        sleep_at_least(until - kGapSpinS - now_s());
        while (now_s() < until) {
        }
    }

    void throw_if_closed() const {
        if (closed.load()) throw TransportError("bus is closed");
    }

    int require_baud_rate(const char* what) const {
        const int rate = transport->baud_rate();
        if (rate <= 0) throw std::logic_error(transport->name() + " has no baud rate to " + what);
        return rate;
    }

    /// The bus lock for the scope (bus_lock.hpp). `checked`: the wait for it ends with WaitCancelledError when the
    /// calling thread's wait is cancelled (cancel.hpp) and with TransportError when the bus is closed (also when it
    /// already is); a thread that holds the lock already only counts up. While it holds the bus, the thread's log
    /// lines and frame logger entries wait (detail::DeferSinkCalls, log.hpp): they are passed on, in order and on
    /// this thread, right after the bus is let go, so a sink never runs with the bus held (a STOP never waits for it).
    class Hold {
    public:
        Hold(const Impl& impl, BusPriority priority, bool checked = true, std::function<void()> extra = {})
            : impl_(impl), priority_(priority), checked_(checked), extra_(std::move(extra)) {
            acquire();
        }
        ~Hold() {
            if (held_) impl_.mutex.unlock();
            end_deferring();  // after the unlock
        }
        Hold(const Hold&) = delete;
        Hold& operator=(const Hold&) = delete;
        void acquire() {
            if (checked_)
                impl_.mutex.lock(priority_, [this] { check(); });
            else
                impl_.mutex.lock(priority_);
            held_ = true;
            if (!deferring_) deferring_.emplace();
        }
        void release() {
            if (!held_) return;
            held_ = false;
            impl_.mutex.unlock();
            end_deferring();
        }
        /// Let a waiting stop-class request go first (BusLock::yield_to_stop); true when one did.
        bool yield_to_stop() {
            held_ = false;  // a check that throws while it waits to take the bus back leaves it free
            const bool yielded = impl_.mutex.yield_to_stop([this] { check(); });
            held_ = true;
            return yielded;
        }

    private:
        void check() const {
            check_cancel();
            impl_.throw_if_closed();
            if (extra_) extra_();
        }
        void end_deferring() noexcept { deferring_.reset(); }  // the kept calls are made (outermost scope only)
        const Impl& impl_;
        BusPriority priority_;
        bool checked_;
        std::function<void()> extra_;  // the STOP-first gate's scope checks of a gated write (detail::SendGate)
        bool held_ = false;
        std::optional<detail::DeferSinkCalls> deferring_;  // from the first acquire to the bus let go
    };

    double delay_ms(int slave) const {
        std::lock_guard<std::mutex> lock(delay_mutex);
        const auto it = response_delay_ms.find(slave);
        return it == response_delay_ms.end() ? kDefaultResponseDelayMs : it->second;
    }

    double response_timeout(int slave, const std::vector<std::uint8_t>& pdu, std::optional<int> expected_len) const {
        const int registers = register_count(pdu);
        const int response_bytes = expected_len ? *expected_len : 64;
        const int rate = transport->baud_rate();
        const int baud = rate > 0 ? rate : kDefaultBaudRate;
        const double kbps = baud / 1000.0;
        const double tout_ms = 3.0 * registers + delay_ms(slave) + 10.0 * (response_bytes + 8) / kbps;
        // Slow rates: VERIFIED on the EC gripper at 9600 (after a line break) - replies alternate ~65 ms / ~285 ms,
        // so the 0.2 s floor missed every other one and baud detection failed. Longer floor below 38400.
        const double slow_floor = baud <= 9600 ? kSlowFloor9600S : baud < 38400 ? kSlowFloorBelow38400S : 0.0;
        // a gateway line waits at least its own minimum (0.3 s; line.hpp)
        return std::max({kMinFrameTimeoutS, slow_floor, current_line()->rules().min_frame_timeout_s,
                         tout_ms / 1000.0 + kUsbMarginS});
    }

    bool set_read_timeout(double value) {
        try {
            transport->set_read_timeout(value);
            return true;
        } catch (const std::exception& e) {
            log_debug(kLogSource, std::string("cannot change the transport timeout: ") + e.what());
            return false;
        }
    }

    /// Lower the transport's read timeout to max_wait; the old value to restore (nullopt: nothing changed).
    std::optional<double> shorten_read_timeout(std::optional<double> max_wait) {
        if (!max_wait) return std::nullopt;
        double current = 0.0;
        try {
            current = transport->read_timeout();
        } catch (const std::exception&) {
            return std::nullopt;
        }
        if (current <= *max_wait || !set_read_timeout(*max_wait)) return std::nullopt;
        return current;
    }

    void drain(const std::vector<std::uint8_t>& head) {
        log_frame(FrameDirection::RxPartial, head);
        transport->reset_input();
    }

    /// What a real line (has_real_timeout: a serial port) has buffered already, up to n bytes, without waiting (its
    /// read timeout set to 0 for the call). 0 when the timeout cannot be changed, and for any other transport: an
    /// in-memory one answers at once anyway, and a binding's own transport may wait its own timeout whatever the core
    /// asks.
    std::size_t read_buffered(std::uint8_t* out, std::size_t n) {
        if (!transport->has_real_timeout()) return 0;
        double saved = 0.0;
        try {
            saved = transport->read_timeout();
        } catch (const std::exception&) {
            return 0;
        }
        if (saved > 0.0 && !set_read_timeout(0.0)) return 0;
        struct Restore {
            Impl* self;
            double saved;
            ~Restore() {
                if (saved > 0.0) self->set_read_timeout(saved);
            }
        } restore{this, saved};
        return transport->read(out, n);
    }

    std::vector<std::uint8_t> read_exact(std::size_t n, const std::vector<std::uint8_t>& so_far,
                                         std::optional<double> deadline) {
        std::vector<std::uint8_t> data(n);
        std::size_t got = transport->read(data.data(), n);
        while (got < n && deadline && now_s() < *deadline) {
            const std::size_t more = transport->read(data.data() + got, n - got);  // a serial port blocks here
            if (more == 0) sleep_at_least(0.001);  // through the clock seam (timing.hpp): a virtual clock moves on
            got += more;
        }
        // The deadline passed (also when this thread was not scheduled for a while after its last read): the bytes that
        // arrived meanwhile are taken before a timeout is declared, never reported missing.
        if (got < n && deadline) got += read_buffered(data.data() + got, n - got);
        if (got < n) {
            if (!so_far.empty() || got > 0) {
                std::vector<std::uint8_t> partial(so_far);
                partial.insert(partial.end(), data.begin(), data.begin() + static_cast<std::ptrdiff_t>(got));
                log_frame(FrameDirection::RxPartial, partial);
            }
            throw ModbusTimeout(util::format("timeout: got %d bytes, wanted %d", static_cast<int>(so_far.size() + got),
                                             static_cast<int>(so_far.size() + n)));
        }
        return data;
    }

    std::vector<std::uint8_t> read_response(int slave, int function, std::optional<int> expected_len,
                                            std::optional<double> deadline) {
        const std::vector<std::uint8_t> head = read_exact(2, {}, deadline);
        if (head[0] != slave) {
            drain(head);
            throw ModbusFrameError(util::format("response from slave %d while talking to slave %d", head[0], slave));
        }
        if (head[1] == (function | 0x80)) {
            const std::vector<std::uint8_t> frame = concat(head, read_exact(3, head, deadline));
            log_frame(FrameDirection::Rx, frame);
            if (!check_crc(frame)) throw ModbusCrcError("bad CRC in exception response " + hex_frame(frame));
            throw ModbusSlaveError(frame[2], function);
        }
        if (head[1] != function) {
            drain(head);
            throw ModbusFrameError(util::format("response function 0x%02x to request 0x%02x", head[1], function));
        }
        std::vector<std::uint8_t> frame;
        if (!expected_len) {
            const std::vector<std::uint8_t> with_count = concat(head, read_exact(1, head, deadline));
            frame = concat(with_count, read_exact(static_cast<std::size_t>(with_count[2]) + 2, with_count, deadline));
        } else {
            frame = concat(head, read_exact(static_cast<std::size_t>(*expected_len - 2), head, deadline));
        }
        log_frame(FrameDirection::Rx, frame);
        if (!check_crc(frame)) throw ModbusCrcError("bad CRC in response " + hex_frame(frame));
        return frame;
    }

    /// Send one request and return the validated response frame. `expected_len`: the full normal response length
    /// (nullopt: a byte count follows). `timeout`: wait this long per attempt (on a real line at least
    /// response_timeout()). `max_wait`: wait at most this long (the transport's read timeout is shortened to it).
    /// `check` validates the response (an echo, a byte count): a ModbusFrameError it throws is retried like any other
    /// mismatched frame (a late reply to an earlier request can sit in front of the right one).
    using ResponseCheck = std::function<void(const std::vector<std::uint8_t>&)>;
    std::vector<std::uint8_t> transaction(int slave, const std::vector<std::uint8_t>& pdu,
                                          std::optional<int> expected_len, std::optional<int> retries_override,
                                          std::optional<double> timeout = std::nullopt,
                                          std::optional<double> max_wait = std::nullopt,
                                          const ResponseCheck& check = ResponseCheck()) {
        check_slave(slave);
        // STOP first: the stop mark when the request is made, before it waits for the gap and the bus. A write that a
        // stop-class write overtakes while it waits (or between its attempts) is refused (gate)
        detail::StopMark since = mark(slave);
        const int extra = retries_override ? *retries_override : retries.load();
        if (extra < 0) throw std::invalid_argument("retries must be >= 0");
        current_line()->check_request(slave, pdu);  // the line's guard: LineRuleError, nothing sent
        const PreemptibleReadOptions* pre = PreemptibleReads::current(mutex);  // null inside a StopPriority scope
        if (pre && !is_read_request(pdu))
            refuse_in_preemptible(util::format("function 0x%02X to slave %d", pdu[0], slave));
        const std::vector<std::uint8_t> frame = build_frame(slave, pdu);
        const int function = pdu[0];
        std::exception_ptr last_error;
        if (pre && pre->gate) pre->gate();  // outside the bus lock (the GUI's gate)
        const bool gated = is_gated_write(pdu);      // the STOP-first gate checks it (modbus.hpp SendGate)
        const bool stop_write = is_stop_write(pdu);  // the gate counts it
        const BusPriority priority = priority_of(pdu);
        const bool stop_class = priority == BusPriority::Stop;
        wait_gap_unlocked(stop_class);  // a normal request: the gap before the bus is taken
        if (const auto& hook = detail::bus_test_hooks().before_bus_wait) hook(slave, pdu);
        std::function<void()> scope_wait_checks;
        if (gated) scope_wait_checks = [this, slave] { scope_checks(slave); };
        // fair, stop-class first, cancellable while waiting (bus_lock.hpp); a gated write's scope checks run too
        Hold hold(*this, priority, true, std::move(scope_wait_checks));
        const bool alone = mutex.depth() == 1;  // not inside a sequence that holds the bus (baud detection)
        throw_if_closed();
        if (max_wait) {
            timeout = max_wait;
        } else if (transport->has_real_timeout()) {
            const double automatic = response_timeout(slave, pdu, expected_len);
            timeout = timeout ? std::max(*timeout, automatic) : automatic;
        }
        struct Restore {
            Impl* self;
            std::optional<double> saved;
            ~Restore() {
                if (saved) self->set_read_timeout(*saved);
            }
        };
        for (int attempt = 0; attempt <= extra; ++attempt) {
            if (attempt > 0 && alone && pre) {
                // a preemptible read gives the bus back altogether between two attempts (retries outside the lock),
                // runs its gate again and waits the gap before it takes the bus back
                hold.release();
                if (pre->gate) pre->gate();
                wait_gap_unlocked(false);
                hold.acquire();
            }
            // What is left of the gap; then a waiting stop-class request goes first: between two attempts (a STOP waits
            // for one attempt at most) and before the first attempt of a normal request (a STOP that came while it
            // waited the gap with the bus held). Not inside a sequence that holds the bus, not among stop-class
            // requests at their first attempt (FIFO among them).
            wait_gap_held(hold, stop_class, alone && (attempt > 0 || !stop_class));
            if (attempt > 0 && alone) {
                check_cancel();
                throw_if_closed();
            }
            const Restore restore{this, shorten_read_timeout(max_wait)};  // put back before the bus is let go
            // STOP first: a write that a STOP overtook (since its command decided on it, while it waited for the bus,
            // or between two attempts) never goes out after it; under the bus lock, so a frame is wholly before the
            // STOP frame or refused
            if (gated) gate(slave, function, attempt, since, last_error);
            if (stop_write) {
                count_stop_write(slave);
                since = mark(slave);  // its own frame: never a reason to refuse its own retry
            }
            const IoStamp stamp{last_io};
            try {
                transport->reset_input();
                log_frame(FrameDirection::Tx, frame);  // passed on once the bus is let go, after the write
                if (stop_write) detail::StopWireAccess::wire(mutex);  // the stop latency line (C1)
                transport->write(frame);
                const std::optional<double> deadline =
                    timeout ? std::optional<double>(now_s() + *timeout) : std::nullopt;
                std::vector<std::uint8_t> response = read_response(slave, function, expected_len, deadline);
                if (check) check(response);
                return response;
            } catch (const ModbusSlaveError& e) {
                if (e.code() == 0x01) {  // Illegal Function: remembered for line / strategy decisions
                    std::lock_guard<std::mutex> line_lock(line_mutex);
                    unsupported[slave].insert(function);
                }
                throw;  // a valid answer: never retried
            } catch (const TransportError&) {
                throw;  // the port failed underneath the bus: never retried
            } catch (const ModbusError& e) {
                if (!is_retryable(e)) throw;
                last_error = std::current_exception();
                log_debug(kLogSource, util::format("attempt %d/%d failed: %s", attempt + 1, extra + 1, e.what()));
            } catch (const EcDriverError&) {
                throw;
            } catch (const std::exception& e) {
                throw TransportError(std::string("transport error: ") + e.what());
            }
        }
        try {
            std::rethrow_exception(last_error);
        } catch (ModbusError& e) {
            e.set_attempts(extra + 1);
            throw;
        }
    }

    std::vector<std::uint8_t> send_broadcast(const std::vector<std::uint8_t>& frame, double settle_s) {
        // STOP first: a broadcast reaches every unit, so a stop-class write to any of them (or broadcast) that went
        // while it waited refuses it (not one that engages a stop or comes from a StopPriority scope)
        const std::uint64_t since = stops_total.load();
        if (PreemptibleReads::current(mutex)) refuse_in_preemptible("a broadcast");
        const std::vector<std::uint8_t> pdu(frame.begin() + 1, frame.end() - 2);  // the frame without address and CRC
        const BusPriority priority = priority_of(pdu);  // a broadcast STOP / pause / servo OFF is stop-class too
        const bool stop_write = is_stop_write(pdu);
        const bool gated = is_gated_write(pdu);
        wait_gap_unlocked(priority == BusPriority::Stop);
        Hold hold(*this, priority);
        throw_if_closed();
        wait_gap_held(hold, priority == BusPriority::Stop, false);
        if (gated && stops_total.load() != since) {
            const std::string message = util::format("broadcast function 0x%02X: a STOP arrived meanwhile (to a unit of "
                                                     "this line or broadcast) after this command was decided; nothing "
                                                     "sent",
                                                     pdu.empty() ? 0 : pdu[0]);
            log_info(kLogSource, message);
            EcNotReadyError error(message);
            error.set_stop_attempted();
            throw error;
        }
        if (stop_write) count_stop_write(0);  // every slave's STOP-first gate sees it (detail::StopMark)
        const IoStamp stamp{last_io};
        try {
            transport->reset_input();
            log_frame(FrameDirection::Tx, frame);  // passed on once the bus is let go, after the write
            if (stop_write) detail::StopWireAccess::wire(mutex);  // the stop latency line (C1)
            transport->write(frame);
            sleep_at_least(settle_s);
            transport->reset_input();  // a broadcast is never answered: drop any line noise
        } catch (const EcDriverError&) {
            throw;
        } catch (const std::exception& e) {
            throw TransportError(std::string("transport error: ") + e.what());
        }
        return frame;
    }

    static std::vector<std::uint8_t> iai_pdu(int group, int command, int op1, int op2, bool allow_unsafe,
                                             bool allow_never_send) {
        if (group < 0 || group > 0xFF || command < 0 || command > 0xFF)
            throw std::invalid_argument("group and command must be 0..255");
        check_u16("op1", op1);
        check_u16("op2", op2);
        const std::string never = iai_never_send(command);
        if (!never.empty() && !allow_never_send)
            throw UnsafeCommandError(util::format(
                "IAI command 0x45/0x%02x (%s) is on the never-send list; nothing sent (pass allow_never_send only on "
                "the user's explicit request)",
                command, never.c_str()));
        if (!allow_unsafe) {
            if (group != 0)
                throw UnsafeCommandError(util::format(
                    "IAI command 0x45 to ID group 0x%02x: only group 0 is allowed without allow_unsafe; nothing sent",
                    group));
            if (iai_safe_command(command).empty())
                throw UnsafeCommandError(util::format(
                    "IAI command 0x45/0x%02x is not on the safe list; pass allow_unsafe only if you know exactly what "
                    "it does; nothing sent",
                    command));
            if (command == 0x05 && op1 != 0)
                throw UnsafeCommandError("0x45/0x05 with op1 != 0 is a software reset; pass allow_unsafe; nothing sent");
        }
        return {static_cast<std::uint8_t>(kFcIaiCommand), static_cast<std::uint8_t>(group),
                static_cast<std::uint8_t>(command), static_cast<std::uint8_t>(op1 >> 8),
                static_cast<std::uint8_t>(op1 & 0xFF), static_cast<std::uint8_t>(op2 >> 8),
                static_cast<std::uint8_t>(op2 & 0xFF)};
    }

    /// Response check for a write: the reply must echo the first `length` bytes of `expected` (function, address and
    /// value or count).
    static ResponseCheck echo_check(const std::vector<std::uint8_t>& expected, std::size_t length) {
        return [expected, length](const std::vector<std::uint8_t>& frame) {
            for (std::size_t i = 0; i < length; ++i) {
                if (frame.size() <= 1 + i || frame[1 + i] != expected[i])
                    throw ModbusFrameError("unexpected echo " + hex_frame(frame));
            }
        };
    }

    /// Response check for a read: the byte count must be `want`.
    static ResponseCheck byte_count_check(const char* name, int want) {
        return [name, want](const std::vector<std::uint8_t>& frame) {
            if (frame[2] != want) throw ModbusFrameError(util::format("%s byte count %d != %d", name, frame[2], want));
        };
    }
};

// ------------------------------------------------------------------------------------------ the reader adapter

namespace {

class SlaveReader final : public UnitReader {
public:
    SlaveReader(ModbusRtu* bus, int slave) : bus_(bus), slave_(slave) {}
    int slave() const override { return slave_; }
    std::vector<std::uint16_t> read_registers(int address, int count) override {
        return bus_->read_registers(slave_, address, count);
    }
    std::vector<std::uint8_t> report_slave_id() override { return bus_->report_slave_id(slave_); }

private:
    ModbusRtu* bus_;
    int slave_;
};

}  // namespace

// ------------------------------------------------------------------------------------------ ModbusRtu

ModbusRtu::ModbusRtu(std::shared_ptr<Transport> transport, BusOptions options) : impl_(new Impl) {
    if (!transport) throw std::invalid_argument("transport required");
    if (options.retries < 0) throw std::invalid_argument("retries must be >= 0");
    if (!(options.min_gap_s >= 0.0)) throw std::invalid_argument("min_gap_s must be >= 0");
    impl_->transport = std::move(transport);
    impl_->retries = options.retries;
    impl_->min_gap_s = options.min_gap_s;
    impl_->frame_logger = std::move(options.frame_logger);
}

ModbusRtu::~ModbusRtu() = default;

Transport& ModbusRtu::transport() { return *impl_->current_transport(); }  // replaced ones are kept (retired)
std::shared_ptr<Transport> ModbusRtu::transport_ptr() const { return impl_->current_transport(); }
std::string ModbusRtu::name() const { return impl_->current_transport()->name(); }

void ModbusRtu::set_transport(std::shared_ptr<Transport> transport) {
    if (!transport) throw std::invalid_argument("transport required");
    const Impl::Hold hold(*impl_, BusPriority::Normal, false);  // after the transaction in flight, also on a closed bus
    const std::string now = transport->name();
    {
        std::lock_guard<std::mutex> lock(impl_->transport_mutex);
        if (transport == impl_->transport) return;
        impl_->retired.push_back(std::move(impl_->transport));
        impl_->transport = std::move(transport);
    }
    log_info(kLogSource, "transport replaced (now " + now + ")");  // outside transport_mutex: a log sink may ask name()
}
int ModbusRtu::retries() const { return impl_->retries.load(); }

void ModbusRtu::set_retries(int retries) {
    if (retries < 0) throw std::invalid_argument("retries must be >= 0");
    impl_->retries = retries;
}

double ModbusRtu::min_gap_s() const { return impl_->min_gap_s.load(); }

void ModbusRtu::set_min_gap_s(double seconds) {
    if (!(seconds >= 0.0)) throw std::invalid_argument("min_gap_s must be >= 0");
    impl_->min_gap_s = seconds;
}

void ModbusRtu::set_frame_logger(FrameLogger logger) {
    std::lock_guard<std::mutex> lock(impl_->logger_mutex);
    impl_->frame_logger = std::move(logger);
}

void ModbusRtu::set_response_delay_ms(int slave, double delay_ms) {
    if (!(delay_ms >= 0.0 && delay_ms <= 1000.0))
        throw std::invalid_argument(util::format("response delay must be 0..1000 ms, got %g", delay_ms));
    std::lock_guard<std::mutex> lock(impl_->delay_mutex);
    impl_->response_delay_ms[slave] = delay_ms;
}

double ModbusRtu::response_delay_ms(int slave) const { return impl_->delay_ms(slave); }

double ModbusRtu::response_timeout(int slave, const std::vector<std::uint8_t>& pdu,
                                   std::optional<int> expected_len) const {
    const Impl::Hold hold(*impl_, BusPriority::Normal, false);
    return impl_->response_timeout(slave, pdu, expected_len);
}

namespace {
std::mutex& close_hook_mutex() {
    static std::mutex m;
    return m;
}
std::function<void(const ModbusRtu&)>& close_hook() {
    static std::function<void(const ModbusRtu&)> hook;
    return hook;
}
}  // namespace

detail::StopMark detail::stop_mark(const ModbusRtu& bus, int slave) { return bus.impl_->mark(slave); }

std::uint64_t detail::halt_count(const ModbusRtu& bus, int slave) {
    return slave >= 1 && slave <= 247 ? bus.impl_->halts[static_cast<std::size_t>(slave)].load() : 0;
}

void detail::count_halt(const ModbusRtu& bus, int slave) {
    if (slave >= 1 && slave <= 247) bus.impl_->halts[static_cast<std::size_t>(slave)].fetch_add(1);
}

void detail::set_bus_close_hook(std::function<void(const ModbusRtu&)> hook) {
    std::lock_guard<std::mutex> lock(close_hook_mutex());
    close_hook() = std::move(hook);
}

void ModbusRtu::close() {
    if (impl_->closed.load()) return;
    std::function<void(const ModbusRtu&)> hook;
    {
        std::lock_guard<std::mutex> lock(close_hook_mutex());
        hook = close_hook();
    }
    if (hook) {
        try {
            hook(*this);  // the status monitors of this bus stop and join first: no "link lost" from a deliberate close
        } catch (const std::exception& e) {
            log_warning(kLogSource, std::string("stopping the status monitors of the bus failed: ") + e.what());
        }
    }
    // Closed first: every thread waiting for the bus ends at once with TransportError (no queued request is sent);
    // then the transaction in flight finishes and the transport closes (stop-class: no queue in front of it).
    if (impl_->closed.exchange(true)) return;
    impl_->mutex.wake_all();
    const Impl::Hold hold(*impl_, BusPriority::Stop, false);
    impl_->transport->close();
}

bool ModbusRtu::closed() const { return impl_->closed.load(); }

bool ModbusRtu::can_reopen() const { return impl_->current_transport()->can_reopen(); }

void ModbusRtu::reopen(std::optional<int> baud_rate) {
    if (!can_reopen()) throw std::logic_error(name() + " cannot be reopened");
    const Impl::Hold hold(*impl_, BusPriority::Normal, false);
    try {
        impl_->transport->reopen(baud_rate);
    } catch (const EcDriverError&) {
        throw;
    } catch (const std::logic_error&) {
        throw;
    } catch (const std::exception& e) {
        throw TransportError(std::string("cannot reopen the port: ") + e.what());
    }
    impl_->closed = false;
    impl_->last_io = now_s();
    log_info(kLogSource, "port reopened (" + impl_->transport->name() + ")");
}

std::recursive_mutex& ModbusRtu::sequence_mutex() { return impl_->sequence; }

BusLock& ModbusRtu::bus_lock() const { return impl_->mutex; }

std::optional<int> ModbusRtu::baud_rate() const {
    const Impl::Hold hold(*impl_, BusPriority::Normal, false);
    const int rate = impl_->transport->baud_rate();
    return rate > 0 ? std::optional<int>(rate) : std::nullopt;
}

bool ModbusRtu::can_change_baud_rate() const { return baud_rate().has_value(); }

void ModbusRtu::set_baud_rate(int rate) {
    if (rate <= 0) throw std::invalid_argument(util::format("baud rate must be > 0, got %d", rate));
    if (PreemptibleReads::current(impl_->mutex)) refuse_in_preemptible("a baud rate change");
    const Impl::Hold hold(*impl_, BusPriority::Normal, false);
    impl_->require_baud_rate("change");
    const Impl::IoStamp stamp{impl_->last_io};
    impl_->transport->set_baud_rate(rate);
}

void ModbusRtu::send_break(double seconds) {
    if (!(seconds > 0.0 && seconds <= 2.0)) throw std::invalid_argument("break duration must be in (0, 2] s");
    if (PreemptibleReads::current(impl_->mutex)) refuse_in_preemptible("a line break");
    impl_->wait_gap_unlocked(false);
    Impl::Hold hold(*impl_, BusPriority::Normal);
    impl_->throw_if_closed();
    impl_->wait_gap_held(hold, false, false);
    const Impl::IoStamp stamp{impl_->last_io};
    impl_->transport->send_break(seconds);
}

int ModbusRtu::detect_baud_rate(const std::vector<int>& slaves, const std::vector<int>& candidates, int probe_address,
                                bool current_first, std::optional<double> probe_timeout_s) {
    std::vector<BaudProbe> targets;
    targets.reserve(slaves.size());
    for (int s : slaves) targets.push_back(BaudProbe{s, std::nullopt});
    return detect_baud_rate_targets(targets, candidates, probe_address, current_first, probe_timeout_s);
}

int ModbusRtu::detect_baud_rate_targets(const std::vector<BaudProbe>& listed, const std::vector<int>& candidates,
                                        int probe_address, bool current_first,
                                        std::optional<double> probe_timeout_s) {
    if (listed.empty()) throw std::invalid_argument("no slave address to probe");
    std::vector<BaudProbe> targets;  // slave 58 skipped (kAsciiStartSlave), unless it is the only one: refused below
    for (const BaudProbe& t : listed)
        if (t.slave != kAsciiStartSlave) targets.push_back(t);
    if (targets.empty())
        targets = listed;
    else if (targets.size() != listed.size())
        log_warning(kLogSource, std::string("baud rate detection: ") + kAsciiStartSlaveNote + ": skipped");
    check_u16("probe_address", probe_address);
    std::size_t plain = 0;
    for (const BaudProbe& t : targets) {
        check_slave(t.slave);
        if (t.address)
            check_u16("address", *t.address);
        else
            ++plain;
    }
    // the plain entries: probe_timeout_s, else kScanTimeoutS for several, else a full read with the transport's timeout;
    // an entry with its own address: probe_timeout_s, else kScanTimeoutS
    const std::optional<double> plain_wait =
        probe_timeout_s ? probe_timeout_s : plain > 1 ? std::optional<double>(kScanTimeoutS) : std::nullopt;
    const double own_wait = probe_timeout_s ? *probe_timeout_s : kScanTimeoutS;
    if (PreemptibleReads::current(impl_->mutex)) refuse_in_preemptible("baud rate detection");
    const Impl::Hold hold(*impl_, BusPriority::Normal);  // held throughout: the rate changes under every request
    impl_->throw_if_closed();
    const int start = impl_->require_baud_rate("detect");
    const std::vector<int> rates = candidates.empty() ? baud_detect_order() : candidates;
    std::vector<int> order;
    if (current_first) {
        order.push_back(start);
        for (int b : rates)
            if (b != start) order.push_back(b);
    } else {
        std::vector<int> all(rates);
        all.push_back(start);
        for (int b : all)
            if (std::find(order.begin(), order.end(), b) == order.end()) order.push_back(b);
    }
    Transport& t = *impl_->transport;
    for (int baud : order) {
        if (baud != t.baud_rate()) t.set_baud_rate(baud);
        for (const BaudProbe& target : targets) {
            const int address = target.slave;
            const int reg = target.address ? *target.address : probe_address;
            const std::optional<double> wait = target.address ? std::optional<double>(own_wait) : plain_wait;
            if (wait) {
                if (!probe(address, *wait, reg)) continue;
            } else {
                try {
                    read_registers(address, reg, 1, 0);
                } catch (const ModbusSlaveError&) {
                    // a well-formed exception reply still proves the rate
                } catch (const ModbusError& e) {
                    if (!is_retryable(e)) throw;
                    log_debug(kLogSource,
                              util::format("no answer from slave %d at %d baud: %s", address, baud, e.what()));
                    continue;
                }
            }
            log_info(kLogSource, util::format("slave %d answers at %d baud", address, baud));
            return baud;
        }
    }
    t.set_baud_rate(start);
    const std::string list = join_ints(order);
    std::vector<std::string> names;  // as the Python edition: a gateway probe (own address) in hex
    for (const BaudProbe& target : targets)
        names.push_back(target.address ? util::format("0x%02X", target.slave) : std::to_string(target.slave));
    if (names.size() == 1)
        throw ModbusTimeout("slave " + names[0] + " did not answer at any of " + list + " baud");
    throw ModbusTimeout("none of slaves " + util::join(names, ", ") + " answered at any of " + list + " baud");
}

int ModbusRtu::detect_baud_rate(int slave, const std::vector<int>& candidates, int probe_address, bool current_first,
                                std::optional<double> probe_timeout_s) {
    return detect_baud_rate(std::vector<int>{slave}, candidates, probe_address, current_first, probe_timeout_s);
}

bool ModbusRtu::probe(int slave, double timeout_s, int address) {
    check_u16("address", address);
    if (!(timeout_s > 0.0)) throw std::invalid_argument("probe timeout must be > 0");
    try {
        impl_->transaction(slave, pdu_words(kFcReadHoldingRegisters, address, 1), 7, 0, std::nullopt, timeout_s);
    } catch (const ModbusSlaveError&) {
        return true;
    } catch (const ModbusError& e) {
        if (!is_retryable(e)) throw;
        log_debug(kLogSource, util::format("probe: no answer from slave %d: %s", slave, e.what()));
        return false;
    }
    return true;
}

std::vector<std::uint16_t> ModbusRtu::read_registers(int slave, int address, int count, std::optional<int> retries) {
    check_u16("address", address);
    if (count < 1 || static_cast<long long>(address) + count > 0x10000)
        throw std::invalid_argument(util::format("invalid register count %d at 0x%04X", count, address));
    std::vector<std::uint16_t> values;
    values.reserve(static_cast<std::size_t>(count));
    int at = address;
    int left = count;
    while (left > 0) {
        const int n = std::min(left, read_chunk(impl_->mutex, kMaxReadRegisters));  // shorter inside PreemptibleReads
        const std::vector<std::uint8_t> frame =
            impl_->transaction(slave, pdu_words(kFcReadHoldingRegisters, at, n), 5 + 2 * n, retries, std::nullopt,
                               std::nullopt, Impl::byte_count_check("FC03", 2 * n));
        for (int i = 0; i < n; ++i)
            values.push_back(static_cast<std::uint16_t>((frame[3 + 2 * i] << 8) | frame[4 + 2 * i]));
        at += n;
        left -= n;
    }
    return values;
}

void ModbusRtu::write_coil(int slave, int address, bool on, std::optional<int> retries) {
    check_u16("address", address);
    const std::vector<std::uint8_t> pdu = pdu_words(kFcWriteCoil, address, on ? kCoilOn : kCoilOff);
    impl_->transaction(slave, pdu, 8, retries, std::nullopt, std::nullopt, Impl::echo_check(pdu, pdu.size()));
}

void ModbusRtu::write_register(int slave, int address, std::uint16_t value) {
    check_u16("address", address);
    const std::vector<std::uint8_t> pdu = pdu_words(kFcWriteRegister, address, value);
    impl_->transaction(slave, pdu, 8, std::nullopt, std::nullopt, std::nullopt, Impl::echo_check(pdu, pdu.size()));
}

void ModbusRtu::write_registers(int slave, int address, const std::vector<std::uint16_t>& values) {
    check_u16("address", address);
    const int n = static_cast<int>(values.size());
    if (n < 1 || n > kMaxWriteRegisters)
        throw std::invalid_argument(
            util::format("can write 1..%d registers per frame, got %d", kMaxWriteRegisters, n));
    std::vector<std::uint8_t> pdu = pdu_words(kFcWriteRegisters, address, n);
    pdu.push_back(static_cast<std::uint8_t>(2 * n));
    for (std::uint16_t v : values) {
        pdu.push_back(static_cast<std::uint8_t>(v >> 8));
        pdu.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    impl_->transaction(slave, pdu, 8, std::nullopt, std::nullopt, std::nullopt, Impl::echo_check(pdu, 5));
}

void ModbusRtu::write_registers(int slave, int address, const std::vector<std::uint16_t>& values,
                                const WriteOptions& options) {
    check_u16("address", address);
    const int n = static_cast<int>(values.size());
    if (n < 1 || n > kMaxWriteRegisters)
        throw std::invalid_argument(
            util::format("can write 1..%d registers per frame, got %d", kMaxWriteRegisters, n));
    if (options.max_wait_s && !(*options.max_wait_s > 0.0)) throw std::invalid_argument("max_wait must be > 0");
    std::vector<std::uint8_t> pdu = pdu_words(kFcWriteRegisters, address, n);
    pdu.push_back(static_cast<std::uint8_t>(2 * n));
    for (std::uint16_t v : values) {
        pdu.push_back(static_cast<std::uint8_t>(v >> 8));
        pdu.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    impl_->transaction(slave, pdu, 8, options.retries, std::nullopt, options.max_wait_s, Impl::echo_check(pdu, 5));
}

std::vector<std::uint16_t> ModbusRtu::read_write_registers(int slave, int read_address, int read_count,
                                                           int write_address, const std::vector<std::uint16_t>& values) {
    check_u16("read_address", read_address);
    check_u16("write_address", write_address);
    if (read_count < 1 || read_count > kMaxReadRegisters)
        throw std::invalid_argument(util::format("read_count must be 1..%d, got %d", kMaxReadRegisters, read_count));
    const int n = static_cast<int>(values.size());
    if (n < 1 || n > 60) throw std::invalid_argument(util::format("can write 1..60 registers with FC17, got %d", n));
    std::vector<std::uint8_t> pdu = pdu_words(kFcReadWriteRegisters, read_address, read_count);
    pdu.push_back(static_cast<std::uint8_t>(write_address >> 8));
    pdu.push_back(static_cast<std::uint8_t>(write_address & 0xFF));
    pdu.push_back(static_cast<std::uint8_t>(n >> 8));
    pdu.push_back(static_cast<std::uint8_t>(n & 0xFF));
    pdu.push_back(static_cast<std::uint8_t>(2 * n));
    for (std::uint16_t v : values) {
        pdu.push_back(static_cast<std::uint8_t>(v >> 8));
        pdu.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    const std::vector<std::uint8_t> frame =
        impl_->transaction(slave, pdu, 5 + 2 * read_count, std::nullopt, std::nullopt, std::nullopt,
                           Impl::byte_count_check("FC17", 2 * read_count));
    std::vector<std::uint16_t> out;
    out.reserve(static_cast<std::size_t>(read_count));
    for (int i = 0; i < read_count; ++i)
        out.push_back(static_cast<std::uint16_t>((frame[3 + 2 * i] << 8) | frame[4 + 2 * i]));
    return out;
}

std::vector<std::uint8_t> ModbusRtu::report_slave_id(int slave, std::optional<int> retries,
                                                     std::optional<double> timeout_s) {
    const double wait = timeout_s ? *timeout_s : kSlaveIdTimeoutS;
    const std::vector<std::uint8_t> frame =
        impl_->transaction(slave, {static_cast<std::uint8_t>(kFcReportSlaveId)}, std::nullopt, retries,
                           wait > 0.0 ? std::optional<double>(wait) : std::nullopt);
    return std::vector<std::uint8_t>(frame.begin() + 3, frame.end() - 2);
}

IaiResponse ModbusRtu::iai_command(int slave, int group, int command, int op1, int op2, const IaiOptions& options) {
    impl_->current_line()->require(LineOperation::IaiCommand,
                                   util::format("IAI command 0x45/0x%02x to slave %d", command, slave));
    const std::vector<std::uint8_t> pdu =
        Impl::iai_pdu(group, command, op1, op2, options.allow_unsafe, options.allow_never_send);
    const std::vector<std::uint8_t> frame = impl_->transaction(slave, pdu, 6, options.retries);
    IaiResponse result;
    result.rejection = frame[2];
    result.status = frame[3];
    if (options.check && result.rejection != 0)
        throw IaiCommandRejected(kFcIaiCommand, command, result.rejection, iai_rejection_name(result.rejection),
                                 iai_rejection_hint(result.rejection, command), result.status);
    return result;
}

IaiResponse2 ModbusRtu::iai_command2(int slave, int command, const std::vector<std::uint32_t>& operands,
                                     const IaiOptions& options, int group) {
    check_u16("command", command);
    impl_->current_line()->require(LineOperation::IaiCommand2,
                                   util::format("IAI command 0x48/0x%04x to slave %d", command, slave));
    if (group < 0 || group > 0xFF) throw std::invalid_argument("group must be 0..255");
    if (operands.size() > 4) throw std::invalid_argument("at most 4 operands");
    const std::string never = iai2_never_send(command);
    if (!never.empty() && !options.allow_never_send)
        throw UnsafeCommandError(util::format("IAI command 0x48/0x%04x (%s) is on the never-send list; nothing sent "
                                              "(pass allow_never_send only on the user's explicit request)",
                                              command, never.c_str()));
    if (!options.allow_unsafe) {
        if (group != 0)
            throw UnsafeCommandError(util::format(
                "IAI command 0x48 to ID group 0x%02x: only group 0 is allowed without allow_unsafe; nothing sent", group));
        if (iai2_safe_command(command).empty())
            throw UnsafeCommandError(util::format("IAI command 0x48/0x%04x is not on the safe list; pass allow_unsafe "
                                                  "only if you know exactly what it does; nothing sent",
                                                  command));
    }
    std::vector<std::uint8_t> pdu = {static_cast<std::uint8_t>(kFcIaiCommand2), static_cast<std::uint8_t>(group),
                                     static_cast<std::uint8_t>(command >> 8), static_cast<std::uint8_t>(command & 0xFF)};
    for (std::size_t i = 0; i < 4; ++i) {
        const std::uint32_t op = i < operands.size() ? operands[i] : 0u;
        for (int shift = 24; shift >= 0; shift -= 8) pdu.push_back(static_cast<std::uint8_t>((op >> shift) & 0xFF));
    }
    const std::vector<std::uint8_t> frame = impl_->transaction(slave, pdu, 22, options.retries);
    IaiResponse2 result;
    result.rejection = (frame[2] << 8) | frame[3];
    for (std::size_t i = 0; i < 4; ++i) {
        const std::size_t at = 4 + 4 * i;
        result.status[i] = (static_cast<std::uint32_t>(frame[at]) << 24) | (static_cast<std::uint32_t>(frame[at + 1]) << 16) |
                           (static_cast<std::uint32_t>(frame[at + 2]) << 8) | frame[at + 3];
    }
    if (options.check && result.rejection != 0)
        throw IaiCommandRejected(kFcIaiCommand2, command, result.rejection, iai2_rejection_name(result.rejection),
                                 iai2_rejection_hint(result.rejection, command),
                                 -1, std::vector<std::uint32_t>(result.status, result.status + 4));
    return result;
}

std::vector<std::uint8_t> ModbusRtu::broadcast_iai_command(int group, int command, int op1, int op2,
                                                           const IaiOptions& options, double settle_s) {
    if (!(settle_s >= 0.0)) throw std::invalid_argument("settle_s must be >= 0");
    // the line's refusal comes first: no opt-in makes a gateway carry a broadcast or an IAI command
    {
        const std::shared_ptr<Line> line = impl_->current_line();
        const std::string what = util::format("broadcast IAI command 0x45/0x%02x", command);
        if (command == 0x04) line->require(LineOperation::BaudChange, "broadcast baud change (0x45 cmd 4)");
        line->require(LineOperation::Broadcast, what);
        line->require(LineOperation::IaiCommand, what);
    }
    const std::vector<std::uint8_t> pdu =
        Impl::iai_pdu(group, command, op1, op2, options.allow_unsafe, options.allow_never_send);
    return impl_->send_broadcast(build_frame(0, pdu), settle_s);
}

std::vector<std::uint8_t> ModbusRtu::broadcast(const std::vector<std::uint8_t>& pdu, double settle_s) {
    if (!(settle_s >= 0.0)) throw std::invalid_argument("settle_s must be >= 0");
    if (pdu.empty() || (pdu[0] != kFcWriteCoil && pdu[0] != kFcWriteRegister && pdu[0] != kFcWriteRegisters)) {
        const std::string fc = pdu.empty() ? std::string("(empty)") : util::format("0x%02X", pdu[0]);
        throw std::invalid_argument("function " + fc +
                                    " cannot be broadcast: only FC05 / FC06 / FC10 (a broadcast is not answered)");
    }
    impl_->current_line()->check_request(0, pdu);  // refused on a gateway line (LineRuleError, nothing sent)
    return impl_->send_broadcast(build_frame(0, pdu), settle_s);
}

std::vector<std::uint8_t> ModbusRtu::broadcast_coil(int address, bool on, double settle_s) {
    check_u16("address", address);
    return broadcast(pdu_words(kFcWriteCoil, address, on ? kCoilOn : kCoilOff), settle_s);
}

std::unique_ptr<UnitReader> ModbusRtu::reader(int slave) {
    check_slave(slave);
    return std::unique_ptr<UnitReader>(new SlaveReader(this, slave));
}

// ------------------------------------------------------------------------------------------ the line (line.hpp)

std::shared_ptr<Line> ModbusRtu::line() const { return impl_->current_line(); }

void ModbusRtu::set_line(std::shared_ptr<Line> line) {
    if (!line) throw std::invalid_argument("line required");
    std::lock_guard<std::mutex> lock(impl_->line_mutex);
    impl_->line = std::move(line);
}

std::set<int> ModbusRtu::unsupported_functions(int slave) const {
    std::lock_guard<std::mutex> lock(impl_->line_mutex);
    const auto it = impl_->unsupported.find(slave);
    return it == impl_->unsupported.end() ? std::set<int>() : it->second;
}

std::vector<std::uint16_t> ModbusRtu::read_registers(int slave, int address, int count, const ReadOptions& options) {
    check_u16("address", address);
    if (count < 1 || static_cast<long long>(address) + count > 0x10000)
        throw std::invalid_argument(util::format("invalid register count %d at 0x%04X", count, address));
    const int size = options.chunk ? *options.chunk : kMaxReadRegisters;
    if (size < 1 || size > kMaxFc03Registers)
        throw std::invalid_argument(util::format("chunk must be 1..%d, got %d", kMaxFc03Registers, size));
    if (options.max_wait_s && !(*options.max_wait_s > 0.0)) throw std::invalid_argument("max_wait must be > 0");
    std::vector<std::uint16_t> values;
    values.reserve(static_cast<std::size_t>(count));
    int at = address;
    int left = count;
    while (left > 0) {
        const int n = std::min(left, read_chunk(impl_->mutex, size));  // shorter inside PreemptibleReads
        const std::vector<std::uint8_t> frame =
            impl_->transaction(slave, pdu_words(kFcReadHoldingRegisters, at, n), 5 + 2 * n, options.retries,
                               std::nullopt, options.max_wait_s, Impl::byte_count_check("FC03", 2 * n));
        for (int i = 0; i < n; ++i)
            values.push_back(static_cast<std::uint16_t>((frame[3 + 2 * i] << 8) | frame[4 + 2 * i]));
        at += n;
        left -= n;
    }
    return values;
}

}  // namespace iai_actuator_driver
