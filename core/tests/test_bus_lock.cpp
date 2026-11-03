// Bus unit: the fair bus lock with stop priority, StopPriority / PreemptibleReads and the cancellable waits
// (bus_lock.hpp, cancel.hpp) on their own and under ModbusRtu over a FakeTransport. Ordering is checked with the bus
// held on purpose (deterministic on a busy PC); the only timings are generous upper bounds. No serial port is opened.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

void sleep_s(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

bool wait_until(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = now_s() + timeout_s;
    while (now_s() < deadline) {
        if (predicate()) return true;
        sleep_s(0.001);
    }
    return predicate();
}

/// Records the order in which threads got something.
struct Order {
    std::mutex mutex;
    std::vector<std::string> items;
    void add(const std::string& s) {
        std::lock_guard<std::mutex> lock(mutex);
        items.push_back(s);
    }
    std::vector<std::string> all() {
        std::lock_guard<std::mutex> lock(mutex);
        return items;
    }
};

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool is_move_block(const Bytes& f) { return f.size() > 4 && f[1] == 0x10 && ((f[2] << 8) | f[3]) == reg::kDirectMoveBase; }

bool is_stop_coil(const Bytes& f, bool on = true) {
    return f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop && f[4] == (on ? 0xFF : 0x00);
}

/// A unit behind a FakeTransport: FC03 answers zeros, FC05 / FC06 echo, FC10 its short echo; `delay_s` of "wire time"
/// per request; `silent` FC03 requests, `silent_writes` FC10 and `silent_coils` FC05 requests left unanswered first (a
/// timeout each); while
/// `held`, a request stays "on the wire" (up to 5 s). Records (frame, thread) of every request.
struct Unit {
    struct Entry {
        Bytes frame;
        std::thread::id thread;
    };
    std::mutex mutex;
    std::vector<Entry> log;
    std::atomic<double> delay_s{0.0};
    std::atomic<int> silent{0};
    std::atomic<int> silent_writes{0};
    std::atomic<int> silent_coils{0};
    std::atomic<int> in_flight{0};
    std::atomic<bool> held{false};

    FakeTransport::Responder responder() {
        return [this](const Bytes& req) -> std::optional<Bytes> {
            ++in_flight;
            {
                std::lock_guard<std::mutex> lock(mutex);
                log.push_back({req, std::this_thread::get_id()});
            }
            if (delay_s.load() > 0.0) sleep_s(delay_s.load());
            const double until = now_s() + 5.0;
            while (held.load() && now_s() < until) sleep_s(0.001);
            --in_flight;
            if (req.size() > 1 && req[1] == 0x03 && silent.load() > 0) {
                --silent;
                return std::nullopt;
            }
            if (req.size() > 1 && req[1] == 0x10 && silent_writes.load() > 0) {
                --silent_writes;
                return std::nullopt;
            }
            if (req.size() > 1 && req[1] == 0x05 && silent_coils.load() > 0) {
                --silent_coils;
                return std::nullopt;
            }
            if (req.size() >= 9 && req[1] == 0x10) return build_frame(req[0], Bytes(req.begin() + 1, req.begin() + 6));
            if (req.size() >= 8 && req[1] == 0x03) {
                const int n = (req[4] << 8) | req[5];
                Bytes pdu{0x03, static_cast<std::uint8_t>(2 * n)};
                pdu.resize(2 + 2 * static_cast<std::size_t>(n), 0);
                return build_frame(req[0], pdu);
            }
            return Bytes(req.begin(), req.end());  // FC05 / FC06: the echo
        };
    }
    std::vector<Entry> all() {
        std::lock_guard<std::mutex> lock(mutex);
        return log;
    }
    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return log.size();
    }
};

/// Parks the first thread that reaches it while armed, until released (10 s at most).
struct Latch {
    std::atomic<bool> armed{false};
    std::atomic<bool> parked{false};
    std::atomic<bool> released{false};
    /// Park here when armed (only the first caller); false when it was not armed.
    bool park() {
        if (!armed.exchange(false)) return false;
        parked = true;
        const double until = now_s() + 10.0;
        while (!released.load() && now_s() < until) sleep_s(0.001);
        return true;
    }
};

/// The bus test hooks for one test (bus_test_hooks is global): cleared again at the end.
struct BusHooks {
    BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
    ~BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
};

thread_local bool park_my_gap = false;  // this thread's gap waits park at the test's latch

/// Parks a thread that set park_my_gap at its gap wait (once, while `latch` is armed) and records whether it held the
/// bus there.
void park_at_gap_waits(Latch& latch, std::atomic<int>& held_at_park) {
    detail::bus_test_hooks().before_gap_wait = [&latch, &held_at_park](double, bool held) {
        if (!park_my_gap || !latch.armed.load()) return;
        held_at_park = held ? 1 : 0;
        latch.park();
    };
}

struct BusRig {
    std::shared_ptr<Unit> unit = std::make_shared<Unit>();
    std::shared_ptr<FakeTransport> t;
    std::unique_ptr<ModbusRtu> bus;
    explicit BusRig(int retries = kRecommendedRetries) {
        t = std::make_shared<FakeTransport>(unit->responder());
        BusOptions o;
        o.retries = retries;
        o.min_gap_s = 0.0;
        bus.reset(new ModbusRtu(t, o));
    }
};

}  // namespace

// ============================================================================================ the lock alone

TEST(bus_lock_hands_the_bus_over_in_request_order) {
    BusLock lock;
    Order order;
    lock.lock();
    std::vector<std::thread> threads;
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([&, i] {
            lock.lock(BusPriority::Normal);
            order.add("n" + std::to_string(i));
            lock.unlock();
        });
        REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == static_cast<std::size_t>(i + 1); }));
    }
    lock.unlock();
    for (auto& t : threads) t.join();
    CHECK(order.all() == (std::vector<std::string>{"n0", "n1", "n2", "n3"}));
    CHECK_EQ(lock.handovers(), std::uint64_t(5));
}

TEST(bus_lock_stop_class_goes_before_every_waiting_normal_request) {
    BusLock lock;
    Order order;
    lock.lock();
    std::vector<std::thread> threads;
    const auto queue = [&](const std::string& name, BusPriority p, std::size_t normal, std::size_t stop) {
        threads.emplace_back([&, name, p] {
            lock.lock(p);
            order.add(name);
            lock.unlock();
        });
        REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == normal && lock.waiting(BusPriority::Stop) == stop; }));
    };
    queue("n1", BusPriority::Normal, 1, 0);
    queue("n2", BusPriority::Normal, 2, 0);
    queue("s1", BusPriority::Stop, 2, 1);
    queue("s2", BusPriority::Stop, 2, 2);
    lock.unlock();
    for (auto& t : threads) t.join();
    CHECK(order.all() == (std::vector<std::string>{"s1", "s2", "n1", "n2"}));
}

TEST(bus_lock_is_recursive_and_only_its_owner_unlocks) {
    BusLock lock;
    lock.lock();
    lock.lock(BusPriority::Stop);
    CHECK_EQ(lock.depth(), 2);
    CHECK(lock.owned_by_this_thread());
    bool try_refused = false;
    bool unlock_refused = false;
    std::thread other([&] {
        try_refused = !lock.try_lock();
        try {
            lock.unlock();
        } catch (const std::logic_error&) {
            unlock_refused = true;
        }
    });
    other.join();
    CHECK(try_refused);
    CHECK(unlock_refused);
    lock.unlock();
    lock.unlock();
    CHECK(!lock.owned_by_this_thread());
    CHECK(lock.try_lock());
    lock.unlock();
}

TEST(bus_lock_a_throwing_check_ends_the_wait_and_gives_the_place_up) {
    BusLock lock;
    lock.lock();
    std::atomic<bool> give_up{false};
    std::atomic<bool> threw{false};
    std::thread waiter([&] {
        try {
            lock.lock(BusPriority::Normal, [&] {
                if (give_up.load()) throw std::runtime_error("given up");
            });
            lock.unlock();
        } catch (const std::runtime_error&) {
            threw = true;
        }
    });
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    give_up = true;
    CHECK(wait_until([&] { return threw.load(); }, 2.0));  // within a slice (20 ms); 2 s for a busy PC
    waiter.join();
    CHECK_EQ(lock.waiting(BusPriority::Normal), std::size_t(0));
    lock.unlock();
    CHECK(lock.try_lock());  // nobody left in the queue
    lock.unlock();
}

TEST(bus_lock_yield_lets_a_waiting_stop_in_and_comes_back_before_normal_requests) {
    BusLock lock;
    Order order;
    lock.lock();
    CHECK(!lock.yield_to_stop());  // nothing stop-class waits: kept
    std::thread normal([&] {
        lock.lock(BusPriority::Normal);
        order.add("normal");
        lock.unlock();
    });
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    std::thread stop([&] {
        lock.lock(BusPriority::Stop);
        order.add("stop");
        lock.unlock();
    });
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
    CHECK(lock.yield_to_stop());
    order.add("owner again");
    CHECK_EQ(lock.depth(), 1);
    lock.unlock();
    stop.join();
    normal.join();
    CHECK(order.all() == (std::vector<std::string>{"stop", "owner again", "normal"}));
    lock.lock();
    lock.lock();
    std::thread stop2([&] {
        lock.lock(BusPriority::Stop);
        lock.unlock();
    });
    REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
    CHECK(!lock.yield_to_stop());  // held recursively: a sequence that must not be interrupted
    lock.unlock();
    lock.unlock();
    stop2.join();
}

// ============================================================================================ cancellation alone

TEST(bus_cancel_scope_is_delivered_once_sticky_stays_and_stop_priority_is_exempt) {
    auto token = std::make_shared<CancelToken>();
    {
        token->cancel();  // before the scope: no effect on it
        const CancelScope scope(token);
        CHECK(!cancel_pending());
        token->cancel();
        CHECK(scope.cancelled());
        {
            const StopPriority stop;
            CHECK(!cancel_pending());
            check_cancel();  // never inside StopPriority
        }
        CHECK_THROWS(check_cancel(), WaitCancelledError);
        check_cancel();  // delivered once: the clean-up runs normally
        CHECK(!cancel_pending());
    }
    {
        const CancelScope sticky(token, true);
        token->cancel();
        CHECK_THROWS(check_cancel(), WaitCancelledError);
        CHECK_THROWS(check_cancel(), WaitCancelledError);
    }
    check_cancel();  // no scope: nothing to cancel
}

TEST(bus_cancellable_sleep_ends_at_once_on_cancel) {
    auto token = std::make_shared<CancelToken>();
    std::atomic<bool> threw{false};
    std::atomic<double> ended{0.0};
    std::thread sleeper([&] {
        const CancelScope scope(token);
        try {
            cancellable_sleep(30.0);
        } catch (const WaitCancelledError&) {
            threw = true;
        }
        ended = now_s();
    });
    sleep_s(0.05);
    const double t0 = now_s();
    token->cancel();
    sleeper.join();
    CHECK(threw.load());
    CHECK(ended.load() - t0 < 2.0);  // at once (a slice at most); generous for a busy PC
}

// ============================================================================================ under ModbusRtu

TEST(bus_stop_frame_goes_first_while_pollers_queue_flat_out) {
    BusRig rig;
    std::atomic<bool> run{true};
    std::vector<std::thread> pollers;
    for (int i = 0; i < 4; ++i) {
        pollers.emplace_back([&] {
            while (run.load()) {
                try {
                    rig.bus->read_registers(1, reg::kMonitorBase, 8);
                } catch (const std::exception&) {
                }
            }
        });
    }
    REQUIRE(wait_until([&] { return rig.unit->size() >= 20; }));
    for (int round = 0; round < 5; ++round) {
        BusLock& lock = rig.bus->bus_lock();
        lock.lock();  // hold the bus: every poller queues behind it
        REQUIRE(wait_until([&] { return lock.waiting(BusPriority::Normal) == 4; }));
        ec_test::Thread stopper([&] { rig.bus->write_coil(1, reg::kCoilStop, true); });  // a stop frame: stop-class
        // CHECK, not REQUIRE: the stop waits for the bus held here, so the test goes on to give it back and join
        CHECK(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
        const std::size_t marker = rig.unit->size();
        lock.unlock();
        stopper.join();
        const auto log = rig.unit->all();
        REQUIRE(log.size() > marker);
        CHECK(is_stop_coil(log[marker].frame));  // the very next frame on the wire, before 4 queued polls
    }
    run = false;
    for (auto& t : pollers) t.join();
    // and FIFO kept every poller going (none starved)
    std::vector<std::thread::id> seen;
    for (const auto& e : rig.unit->all())
        if (std::find(seen.begin(), seen.end(), e.thread) == seen.end()) seen.push_back(e.thread);
    CHECK(seen.size() >= 5);  // 4 pollers + the stoppers
}

TEST(bus_a_retrying_request_lets_a_waiting_stop_in_between_attempts) {
    BusRig rig(3);
    rig.unit->silent = 2;  // the read's first two attempts get no answer
    rig.unit->held = true;
    ec_test::Thread reader([&] { rig.bus->read_registers(1, reg::kMonitorBase, 8); });
    REQUIRE(wait_until([&] { return rig.unit->in_flight.load() == 1; }));  // attempt 1 on the wire
    ec_test::Thread stopper([&] { rig.bus->write_coil(1, reg::kCoilStop, true); });
    REQUIRE(wait_until([&] { return rig.bus->bus_lock().waiting(BusPriority::Stop) == 1; }));
    rig.unit->held = false;
    reader.join();
    stopper.join();
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 4u);  // read, STOP, read, read
    CHECK(log[0].frame[1] == 0x03);
    CHECK(is_stop_coil(log[1].frame));  // after one attempt, not after all three
    CHECK(log[2].frame[1] == 0x03);
    CHECK(log[3].frame[1] == 0x03);
}

/// A 0x9900 block (target, band, speed, accel: values do not matter to the bus) whose first attempt is held on the wire
/// and gets no answer while `stop` waits for the bus (stop-class): the STOP goes between the attempts. Returns the
/// writer's error text ("" when it did not end with EcNotReadyError) and whether it said a STOP was attempted.
std::pair<std::string, bool> write_overtaken_by_a_stop(BusRig& rig, const std::function<void()>& stop) {
    rig.unit->silent_writes = 1;  // attempt 1 gets no answer
    rig.unit->held = true;
    std::string what;
    bool stop_attempted = false;
    ec_test::Thread mover([&] {
        try {
            rig.bus->write_registers(1, reg::kDirectMoveBase, {0, 5000, 0, 10, 0, 1000, 30, 0, 0});
        } catch (const EcNotReadyError& e) {
            what = e.what();
            stop_attempted = e.stop_attempted();
        }
    });
    // CHECK, not REQUIRE: the writer must be let go and joined whatever happens
    CHECK(wait_until([&] { return rig.unit->in_flight.load() == 1; }));  // attempt 1 on the wire, the bus held
    ec_test::Thread stopper(stop);
    CHECK(wait_until([&] { return rig.bus->bus_lock().waiting(BusPriority::Stop) == 1; }));
    rig.unit->held = false;
    mover.join();
    stopper.join();
    return {what, stop_attempted};
}

TEST(bus_a_write_is_never_sent_again_after_a_stop_went_between_its_attempts) {
    // A4: the retry path. Before, the yield between two attempts let the STOP out and the same block went out again
    // after it (3 retries by default): a move after the STOP. Now: block, STOP, nothing else; the caller is told.
    BusRig rig(3);
    const auto result = write_overtaken_by_a_stop(rig, [&] { rig.bus->write_coil(1, reg::kCoilStop, true); });
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 2u);
    CHECK(is_move_block(log[0].frame));
    CHECK(is_stop_coil(log[1].frame));
    CHECK(contains(result.first, "a STOP went first"));
    CHECK(contains(result.first, "not re-sent"));
    CHECK(result.second);  // that STOP was the stop: guard() sends no second one
}

TEST(bus_a_write_is_never_sent_again_after_a_broadcast_stop_went_between_its_attempts) {
    // the same with the STOP broadcast to every unit of the line (Port::broadcast_stop): it counts for slave 1 too
    BusRig rig(3);
    const auto result = write_overtaken_by_a_stop(rig, [&] {
        const StopPriority stop_priority;
        rig.bus->broadcast_coil(reg::kCoilStop, true, 0.0);
    });
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 2u);
    CHECK(is_move_block(log[0].frame));
    CHECK(log[1].frame[0] == 0x00);  // broadcast
    CHECK(is_stop_coil(log[1].frame));
    CHECK(contains(result.first, "a STOP went first"));
}

TEST(bus_a_send_gate_refuses_a_write_after_a_stop_it_did_not_see_and_never_its_own) {
    // A4 (ii): a command takes the stop mark before its checks; a STOP from another thread after that refuses its
    // write (nothing sent), a STOP to another slave does not, its own STOP frames do not
    BusRig rig(0);
    {
        detail::SendGate gate(*rig.bus, 1, detail::stop_mark(*rig.bus, 1));
        ec_test::Thread other([&] { rig.bus->write_coil(2, reg::kCoilStop, true); });  // another unit
        other.join();
        rig.bus->write_register(1, 0x0D00, 0x1000);  // goes out
        CHECK_EQ(gate.passed(), 1);
        ec_test::Thread stopper([&] { rig.bus->write_coil(1, reg::kCoilStop, true); });
        stopper.join();
        const std::size_t before = rig.unit->size();
        bool refused = false;
        try {
            rig.bus->write_register(1, 0x0D00, 0x1000);
        } catch (const EcNotReadyError& e) {
            refused = contains(e.what(), "a STOP arrived meanwhile") && contains(e.what(), "nothing sent");
        }
        CHECK(refused);
        CHECK_EQ(rig.unit->size(), before);  // nothing sent
        CHECK_EQ(gate.passed(), 1);
        rig.bus->read_registers(1, reg::kMonitorBase, 2);  // reads are never refused
    }
    {
        detail::SendGate gate(*rig.bus, 1, detail::stop_mark(*rig.bus, 1));
        rig.bus->write_coil(1, reg::kCoilStop, false);  // this thread's own STOP release: counted, moves its mark on
        rig.bus->write_register(1, 0x0D00, 0x1000);    // goes out
        CHECK_EQ(gate.passed(), 2);
    }
}

/// A write of `write` (another thread) queued for the bus held here, then a STOP coil ON to `stop_slave` that goes
/// first. Returns the writer's error text ("" when it did not end with EcNotReadyError) and whether it said a STOP was
/// attempted.
std::pair<std::string, bool> write_queued_behind_a_stop(BusRig& rig, int stop_slave, const std::function<void()>& write) {
    BusLock& lock = rig.bus->bus_lock();
    std::string what;
    bool stop_attempted = false;
    lock.lock();  // hold the bus: the write queues behind it
    ec_test::Thread writer([&] {
        try {
            write();
        } catch (const EcNotReadyError& e) {
            what = e.what();
            stop_attempted = e.stop_attempted();
        }
    });
    // CHECK, not REQUIRE: the bus must be given back and the threads joined whatever happens
    CHECK(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    ec_test::Thread stopper([&] { rig.bus->write_coil(stop_slave, reg::kCoilStop, true); });
    CHECK(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
    lock.unlock();
    writer.join();
    stopper.join();
    return {what, stop_attempted};
}

TEST(bus_a_write_queued_before_a_stop_is_not_sent_after_it) {
    // A write without a SendGate scope (a raw register write) waits for the bus when a STOP queues: the STOP goes
    // first. Before: the write's stop mark was taken once it had the bus, after the STOP, so it went out right after
    // it. Now it counts from when it was made: refused, nothing sent.
    BusRig rig(3);
    const auto result = write_queued_behind_a_stop(rig, 1, [&] { rig.bus->write_register(1, 0x0D00, 0x1000); });
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 1u);
    CHECK(is_stop_coil(log[0].frame));
    CHECK(contains(result.first, "a STOP arrived meanwhile"));
    CHECK(contains(result.first, "nothing sent"));
    CHECK(result.second);
    // a write to another unit is not held back by that unit's STOP
    const auto other = write_queued_behind_a_stop(rig, 2, [&] { rig.bus->write_register(1, 0x0D00, 0x1000); });
    CHECK_EQ(other.first, std::string());
    CHECK_EQ(rig.unit->size(), std::size_t(3));
}

TEST(bus_a_broadcast_write_queued_before_a_stop_to_any_unit_is_not_sent_after_it) {
    // a broadcast reaches every unit of the line: a STOP to any of them that went while it waited refuses it
    BusRig rig(3);
    const auto result = write_queued_behind_a_stop(rig, 2, [&] { rig.bus->broadcast({0x06, 0x0D, 0x00, 0x10, 0x00}, 0.0); });
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 1u);
    CHECK(is_stop_coil(log[0].frame));
    CHECK(contains(result.first, "a STOP arrived meanwhile"));
    CHECK(contains(result.first, "nothing sent"));
    CHECK(result.second);
    rig.bus->broadcast({0x06, 0x0D, 0x00, 0x10, 0x00}, 0.0);  // no STOP meanwhile: sent
    CHECK_EQ(rig.unit->size(), std::size_t(2));
}

TEST(bus_a_stop_release_outside_a_stop_scope_is_retried_after_a_lost_answer) {
    // A STOP coil OFF sent outside a StopPriority scope (the release of a stale STOP before the next command) is
    // counted as a stop-class write and checked by the gate. Before: its own count refused its retry after one lost
    // answer ("a STOP went first ... not re-sent"). Now its own frame never counts against it: sent again.
    BusRig rig(3);
    rig.unit->silent_coils = 1;  // the first release gets no answer
    rig.bus->write_coil(1, reg::kCoilStop, false);
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 2u);
    CHECK(is_stop_coil(log[0].frame, false));
    CHECK(is_stop_coil(log[1].frame, false));
}

TEST(bus_a_stop_is_never_kept_waiting_by_the_gap_of_a_normal_request) {
    // A5: a read right after a frame is parked in its wait for the minimum gap; meanwhile a STOP from another thread
    // goes out. Before: the gap was slept with the bus held, so the STOP waited for that sleep and then for the read.
    BusRig rig(0);
    rig.bus->set_min_gap_s(0.5);  // long: the second read finds the gap open however busy the PC is
    BusHooks hooks;
    Latch latch;
    std::atomic<int> held_at_park{-1};
    park_at_gap_waits(latch, held_at_park);
    latch.armed = true;
    ec_test::Thread reader([&] {
        park_my_gap = true;
        rig.bus->read_registers(1, reg::kMonitorBase, 2);  // the first frame: no gap before it
        rig.bus->read_registers(1, reg::kMonitorBase, 2);  // parked in its gap wait
    });
    // CHECK, not REQUIRE: the reader must be let go and joined whatever happens
    CHECK(wait_until([&] { return latch.parked.load(); }));
    CHECK_EQ(held_at_park.load(), 0);  // the gap is waited before the bus is taken
    std::atomic<bool> stopped{false};
    ec_test::Thread stopper([&] {
        rig.bus->write_coil(1, reg::kCoilStop, true);
        stopped = true;
    });
    CHECK(wait_until([&] { return stopped.load(); }));  // while the read is still parked
    latch.released = true;
    reader.join();
    stopper.join();
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 3u);
    CHECK(log[0].frame[1] == 0x03);
    CHECK(is_stop_coil(log[1].frame));
    CHECK(log[2].frame[1] == 0x03);
}

TEST(bus_a_request_handed_the_bus_lets_a_stop_that_came_during_its_gap_go_first) {
    // A5: a read handed the bus right after a frame waits what is left of the gap with the bus held (parked there); a
    // STOP that queues meanwhile goes first, then the read. Before: the read went first and the STOP waited for its
    // reply too.
    BusRig rig(0);
    rig.bus->set_min_gap_s(0.5);
    BusHooks hooks;
    Latch latch;
    std::atomic<int> held_at_park{-1};
    park_at_gap_waits(latch, held_at_park);
    BusLock& lock = rig.bus->bus_lock();
    lock.lock();  // hold the bus: the reader queues behind it
    ec_test::Thread reader([&] {
        park_my_gap = true;
        rig.bus->read_registers(1, reg::kMonitorBase, 2);
    });
    CHECK(wait_until([&] { return lock.waiting(BusPriority::Normal) == 1; }));
    rig.bus->read_registers(1, reg::kMonitorBase, 2);  // a frame of the holder: the gap starts now
    latch.armed = true;
    lock.unlock();  // the reader takes the bus and waits the gap with it held: parked there
    CHECK(wait_until([&] { return latch.parked.load(); }));
    CHECK_EQ(held_at_park.load(), 1);
    ec_test::Thread stopper([&] { rig.bus->write_coil(1, reg::kCoilStop, true); });
    CHECK(wait_until([&] { return lock.waiting(BusPriority::Stop) == 1; }));
    latch.released = true;
    reader.join();
    stopper.join();
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 3u);
    CHECK(log[0].frame[1] == 0x03);  // the holder's
    CHECK(is_stop_coil(log[1].frame));
    CHECK(log[2].frame[1] == 0x03);  // the reader's, after the STOP
    const std::vector<double> times = rig.t->write_times();
    REQUIRE(times.size() == 3u);
    for (std::size_t i = 1; i < times.size(); ++i) CHECK(times[i] - times[i - 1] >= 0.5 - 1e-9);  // the gap held
}

TEST(bus_preemptible_reads_split_short_and_refuse_writes) {
    BusRig rig;
    {
        const PreemptibleReads scope;
        const std::vector<std::uint16_t> words = rig.bus->read_registers(1, 0x9000, 100);
        CHECK_EQ(words.size(), std::size_t(100));
        ReadOptions ro;
        ro.chunk = 120;
        CHECK_EQ(rig.bus->read_registers(1, 0x9000, 40, ro).size(), std::size_t(40));
        CHECK_THROWS(rig.bus->write_coil(1, reg::kCoilSon, true), UnsafeCommandError);
        CHECK_THROWS(rig.bus->write_register(1, 0x0D00, 1), UnsafeCommandError);
        CHECK_THROWS(rig.bus->broadcast_coil(reg::kCoilStop, true), UnsafeCommandError);
        CHECK_THROWS(rig.bus->detect_baud_rate(1), UnsafeCommandError);
        {
            const StopPriority stop;  // a stop always goes out
            rig.bus->write_coil(1, reg::kCoilStop, true);
        }
    }
    rig.bus->read_registers(1, 0x9000, 100);  // outside: the usual 60-word frames
    std::vector<int> counts;
    for (const auto& e : rig.unit->all())
        counts.push_back(e.frame[1] == 0x03 ? ((e.frame[4] << 8) | e.frame[5]) : -1);
    CHECK(counts == (std::vector<int>{32, 32, 32, 4, 32, 8, -1, 60, 40}));
    CHECK_THROWS(PreemptibleReads(PreemptibleReadOptions{0, {}}), std::invalid_argument);
}

TEST(bus_preemptible_read_lets_a_stop_in_between_two_short_frames) {
    BusRig rig;
    std::atomic<int> gates{0};
    rig.unit->held = true;  // the first frame stays on the wire until the STOP is queued
    ec_test::Thread reader([&] {
        PreemptibleReadOptions o;
        o.gate = [&] { ++gates; };
        const PreemptibleReads scope(o);
        rig.bus->read_registers(1, 0x9000, 32 * 8);  // 8 frames of 32 words
    });
    REQUIRE(wait_until([&] { return rig.unit->in_flight.load() == 1; }));
    ec_test::Thread stopper([&] { rig.bus->write_coil(1, reg::kCoilStop, true); });
    REQUIRE(wait_until([&] { return rig.bus->bus_lock().waiting(BusPriority::Stop) == 1; }));
    rig.unit->held = false;
    reader.join();
    stopper.join();
    const auto log = rig.unit->all();
    REQUIRE(log.size() == 9u);
    CHECK(is_stop_coil(log[1].frame));  // right after the one short frame in flight, not after the whole read
    for (std::size_t i = 0; i < log.size(); ++i)
        if (i != 1) CHECK_EQ((log[i].frame[4] << 8) | log[i].frame[5], 32);
    CHECK_EQ(gates.load(), 8);  // the gate before every request
}

TEST(bus_close_ends_every_waiter_at_once_and_sends_nothing_queued) {
    BusRig rig;
    rig.unit->delay_s = 0.3;  // the transaction in flight takes 300 ms
    ec_test::Thread busy([&] {
        try {
            rig.bus->read_registers(1, 0x9000, 8);
        } catch (const std::exception&) {
        }
    });
    REQUIRE(wait_until([&] { return rig.unit->in_flight.load() == 1; }));
    std::atomic<bool> transport_error{false};
    std::atomic<double> waiter_end{0.0};
    ec_test::Thread waiter([&] {
        try {
            rig.bus->read_registers(1, 0x9100, 8);
        } catch (const TransportError&) {
            transport_error = true;
        }
        waiter_end = now_s();
    });
    REQUIRE(wait_until([&] { return rig.bus->bus_lock().waiting(BusPriority::Normal) == 1; }));
    rig.bus->close();  // returns once the transaction in flight is done
    const double closed_at = now_s();
    busy.join();
    waiter.join();
    CHECK(transport_error.load());
    CHECK(waiter_end.load() <= closed_at);  // ended while the transaction in flight was still running
    CHECK_EQ(rig.unit->size(), std::size_t(1));  // the queued request never went out
    CHECK(rig.bus->closed());
}

TEST(bus_cancelled_wait_for_the_bus_ends_with_wait_cancelled) {
    BusRig rig;
    rig.unit->delay_s = 0.3;
    ec_test::Thread busy([&] { rig.bus->read_registers(1, 0x9000, 8); });
    REQUIRE(wait_until([&] { return rig.unit->in_flight.load() == 1; }));
    auto token = std::make_shared<CancelToken>();
    std::atomic<bool> cancelled{false};
    std::atomic<double> waiter_end{0.0};
    ec_test::Thread waiter([&] {
        const CancelScope scope(token);
        try {
            rig.bus->read_registers(1, 0x9100, 8);
        } catch (const WaitCancelledError&) {
            cancelled = true;
        }
        waiter_end = now_s();
    });
    REQUIRE(wait_until([&] { return rig.bus->bus_lock().waiting(BusPriority::Normal) == 1; }));
    token->cancel();
    waiter.join();
    busy.join();
    const double busy_end = now_s();
    CHECK(cancelled.load());
    CHECK(waiter_end.load() <= busy_end);
    CHECK_EQ(rig.unit->size(), std::size_t(1));  // the cancelled request never went out
    // the bus works on: the next request of the same thread scope is not cancelled again
    rig.bus->read_registers(1, 0x9100, 8);
}
