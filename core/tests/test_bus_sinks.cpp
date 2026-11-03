// Bus unit: the calls into the caller's code (the frame logger, the log handler) never run with the bus held (the load
// plan's A14): a thread's frame entries and log lines are passed on right after it lets the bus go, in order and on
// that same thread, so a sink that blocks (a console write, a binding waiting for its interpreter's lock) never keeps
// a STOP from another thread waiting for the bus. The interleavings are forced with latches on real time (a sink
// parked until released); the only timings are generous upper bounds on those latches. No serial port is opened.
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/log.hpp"
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

bool is_stop_coil(const Bytes& f) {
    return f.size() >= 6 && f[1] == 0x05 && ((f[2] << 8) | f[3]) == reg::kCoilStop && f[4] == 0xFF;
}

/// Parks the first thread that reaches it while armed, until released (10 s at most).
struct Latch {
    std::atomic<bool> armed{false};
    std::atomic<bool> parked{false};
    std::atomic<bool> released{false};
    bool park() {
        if (!armed.exchange(false)) return false;
        parked = true;
        const double until = now_s() + 10.0;
        while (!released.load() && now_s() < until) sleep_s(0.001);
        return true;
    }
};

/// A unit behind a FakeTransport: FC03 answers zeros (the first `silent` FC03 requests get no answer), FC05 echoes.
struct Rig {
    std::atomic<int> silent{0};
    std::shared_ptr<FakeTransport> t;
    std::unique_ptr<ModbusRtu> bus;
    explicit Rig(int retries) {
        t = std::make_shared<FakeTransport>([this](const Bytes& req) -> std::optional<Bytes> {
            if (req.size() >= 8 && req[1] == 0x03) {
                if (silent.load() > 0) {
                    --silent;
                    return std::nullopt;
                }
                const int n = (req[4] << 8) | req[5];
                Bytes pdu{0x03, static_cast<std::uint8_t>(2 * n)};
                pdu.resize(2 + 2 * static_cast<std::size_t>(n), 0);
                return build_frame(req[0], pdu);
            }
            return Bytes(req.begin(), req.end());
        });
        BusOptions o;
        o.retries = retries;
        o.min_gap_s = 0.0;
        bus.reset(new ModbusRtu(t, o));
    }
    bool stop_written() const {
        for (const Bytes& f : t->writes())
            if (is_stop_coil(f)) return true;
        return false;
    }
};

/// The process-wide log handler for one test: put back at the end.
struct LogScope {
    LogHandler previous;
    explicit LogScope(LogHandler handler) { previous = set_log_handler(std::move(handler)); }
    ~LogScope() { set_log_handler(previous); }
};

}  // namespace

TEST(bus_a_frame_sink_that_blocks_never_keeps_a_stop_waiting) {
    // The reader's frame sink is parked on its first entry. Before, the Tx entry was handed over with the bus held
    // (before the write): the STOP from the other thread waited for the sink. Now the sink runs after the bus is let
    // go: the STOP goes out while the sink is still parked.
    Rig rig(0);
    Latch latch;
    std::mutex mutex;
    std::vector<std::pair<FrameDirection, std::thread::id>> calls;
    std::atomic<int> written_when_tx{-1};
    rig.bus->set_frame_logger([&](FrameDirection d, const Bytes&) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            calls.emplace_back(d, std::this_thread::get_id());
        }
        if (d == FrameDirection::Tx && written_when_tx.load() < 0)
            written_when_tx = static_cast<int>(rig.t->writes().size());
        latch.park();
    });
    latch.armed = true;
    std::thread::id reader_id;
    ec_test::Thread reader([&] {
        reader_id = std::this_thread::get_id();
        rig.bus->read_registers(1, reg::kMonitorBase, 8);
    });
    // CHECK, not REQUIRE: the reader must be let go and joined whatever happens
    CHECK(wait_until([&] { return latch.parked.load(); }));
    std::atomic<bool> stopped{false};
    ec_test::Thread stopper([&] {
        rig.bus->write_coil(1, reg::kCoilStop, true);
        stopped = true;
    });
    CHECK(wait_until([&] { return stopped.load(); }));  // while the reader's sink is still parked
    CHECK(rig.stop_written());
    latch.released = true;
    reader.join();
    stopper.join();
    CHECK_EQ(written_when_tx.load(), 1);  // the Tx entry came after its frame was written
    // each thread's own entries, Tx then Rx, on that thread (the stopper's came while the reader's sink was parked)
    std::lock_guard<std::mutex> lock(mutex);
    REQUIRE(calls.size() == 4u);
    std::vector<FrameDirection> mine, others;
    for (const auto& c : calls) (c.second == reader_id ? mine : others).push_back(c.first);
    CHECK(mine == (std::vector<FrameDirection>{FrameDirection::Tx, FrameDirection::Rx}));
    CHECK(others == (std::vector<FrameDirection>{FrameDirection::Tx, FrameDirection::Rx}));
    CHECK(calls[0].second == reader_id);  // the parked one
    CHECK(calls[3].second == reader_id);  // its Rx after the stopper's pair
}

TEST(bus_a_log_sink_that_blocks_never_keeps_a_stop_waiting) {
    // A read's first attempt gets no answer: "attempt 1/2 failed" is logged between the two attempts, with the bus
    // held. Before, the handler ran right there (parked here: the STOP waited). Now the line is passed on after the
    // read let the bus go, on the reader's thread: the STOP goes out while the handler is parked.
    Rig rig(1);
    rig.silent = 1;
    Latch latch;
    std::atomic<bool> on_reader_thread{false};
    std::thread::id reader_id;
    std::atomic<bool> reader_started{false};
    LogScope scope([&](LogLevel, const std::string&, const std::string& message) {
        if (message.find("attempt 1/2 failed") == std::string::npos || !reader_started.load()) return;
        on_reader_thread = std::this_thread::get_id() == reader_id;
        latch.park();
    });
    latch.armed = true;
    ec_test::Thread reader([&] {
        reader_id = std::this_thread::get_id();
        reader_started = true;
        rig.bus->read_registers(1, reg::kMonitorBase, 8);
    });
    CHECK(wait_until([&] { return latch.parked.load(); }));
    CHECK(on_reader_thread.load());
    std::atomic<bool> stopped{false};
    ec_test::Thread stopper([&] {
        rig.bus->write_coil(1, reg::kCoilStop, true);
        stopped = true;
    });
    CHECK(wait_until([&] { return stopped.load(); }));
    CHECK(rig.stop_written());
    latch.released = true;
    reader.join();
    stopper.join();
    const auto writes = rig.t->writes();
    REQUIRE(writes.size() == 3u);  // read, read again (its retry), then the STOP: the read had ended first
    CHECK(is_stop_coil(writes[2]));
}

TEST(bus_sink_calls_after_the_bus_is_let_go_reach_the_sink_installed_then) {
    // A sink removed before the deferred entries are passed on gets none of them (a C caller frees its user data
    // after iad_axis_set_frame_sink(NULL) returns); a thread that never holds a bus logs at once.
    Rig rig(0);
    std::atomic<int> entries{0};
    rig.bus->set_frame_logger([&](FrameDirection, const Bytes&) { ++entries; });
    rig.bus->read_registers(1, reg::kMonitorBase, 2);
    CHECK_EQ(entries.load(), 2);
    {
        detail::DeferSinkCalls defer;  // as with the bus held
        rig.bus->read_registers(1, reg::kMonitorBase, 2);
        CHECK_EQ(entries.load(), 2);   // kept
        rig.bus->set_frame_logger(FrameLogger());
    }
    CHECK_EQ(entries.load(), 2);  // the sink was gone when they were passed on
    std::vector<std::string> lines;
    LogScope scope([&](LogLevel, const std::string&, const std::string& message) { lines.push_back(message); });
    log_info("test", "at once");
    CHECK_EQ(lines.size(), std::size_t(1));
    {
        detail::DeferSinkCalls outer;
        {
            detail::DeferSinkCalls inner;
            log_info("test", "kept");
        }
        CHECK_EQ(lines.size(), std::size_t(1));  // the outermost scope passes them on
    }
    REQUIRE(lines.size() == 2u);
    CHECK_EQ(lines[1], std::string("kept"));
}
