// Bus unit: the Modbus RTU master over a FakeTransport (frames, echo and length checks, retries, IAI 0x45, broadcasts,
// the gap, the response timeout, the frame logger, threads). Ported from the C# edition's ModbusRtuTests.cs and the
// Python edition's test_modbus.py. No serial port is opened.
#include <algorithm>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes hx(const std::string& text) { return util::hex_bytes(text); }
Bytes resp(int slave, const std::string& pdu) { return build_frame(slave, hx(pdu)); }

Bytes read_resp(int slave, const std::vector<std::uint16_t>& values) {
    Bytes pdu{0x03, static_cast<std::uint8_t>(2 * values.size())};
    for (std::uint16_t v : values) {
        pdu.push_back(static_cast<std::uint8_t>(v >> 8));
        pdu.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    return build_frame(slave, pdu);
}

struct Rig {
    std::shared_ptr<FakeTransport> t;
    std::unique_ptr<ModbusRtu> bus;
};

Rig scripted(const std::vector<std::optional<Bytes>>& script, int retries = kRecommendedRetries, double gap = 0.0) {
    Rig rig;
    rig.t = std::make_shared<FakeTransport>();
    for (const auto& r : script) rig.t->enqueue(r);
    BusOptions options;
    options.retries = retries;
    options.min_gap_s = gap;
    rig.bus.reset(new ModbusRtu(rig.t, options));
    return rig;
}

/// A register map answering FC03 (and FC06 / FC10 by storing), per slave; records the reads.
struct FakeUnits {
    std::mutex mutex;
    std::map<int, std::map<int, std::uint16_t>> regs;
    std::vector<std::pair<int, int>> reads;

    std::optional<Bytes> respond(const Bytes& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        const int slave = frame[0];
        if (slave == 0 || !regs.count(slave)) return std::nullopt;
        auto& r = regs[slave];
        const int fc = frame[1];
        const int a = (frame[2] << 8) | frame[3];
        const int b = (frame[4] << 8) | frame[5];
        if (fc == 0x03) {
            reads.emplace_back(a, b);
            std::vector<std::uint16_t> values;
            for (int i = 0; i < b; ++i) values.push_back(r.count(a + i) ? r[a + i] : 0);
            return read_resp(slave, values);
        }
        if (fc == 0x06) {
            r[a] = static_cast<std::uint16_t>(b);
            return Bytes(frame.begin(), frame.end());
        }
        return std::nullopt;
    }
};

/// A Transport with a real read timeout (like a serial port) over a FakeTransport: the bus then waits up to
/// response_timeout() per request.
class SlowLine final : public Transport {
public:
    explicit SlowLine(std::shared_ptr<FakeTransport> inner) : inner_(std::move(inner)) {}
    void write(const std::uint8_t* d, std::size_t n) override { inner_->write(d, n); }
    std::size_t read(std::uint8_t* b, std::size_t n) override { return inner_->read(b, n); }
    void reset_input() override { inner_->reset_input(); }
    int baud_rate() const override { return inner_->baud_rate(); }
    void set_baud_rate(int rate) override { inner_->set_baud_rate(rate); }
    double read_timeout() const override { return inner_->read_timeout(); }
    void set_read_timeout(double s) override { inner_->set_read_timeout(s); }
    bool has_real_timeout() const override { return true; }
    void send_break(double s) override { inner_->send_break(s); }
    void close() override { inner_->close(); }
    bool is_open() const override { return inner_->is_open(); }
    std::string name() const override { return "SLOW"; }

private:
    std::shared_ptr<FakeTransport> inner_;
};

/// A real line (has_real_timeout) whose reply to every request arrives in two chunks: the first `first` bytes at once,
/// the rest while the reader stalls right after the read that took the first chunk's last byte (delay injection on the
/// virtual clock: the clock moves on by `stall_s`, as when this thread is not scheduled between the read and its
/// deadline check). Reads never block.
class StallingLine final : public Transport {
public:
    StallingLine(Bytes reply, std::size_t first, double stall_s, VirtualClock& clock)
        : reply_(std::move(reply)), first_(first), stall_s_(stall_s), clock_(clock) {}
    int writes() const { return writes_; }
    void write(const std::uint8_t*, std::size_t) override {
        ++writes_;
        pending_.assign(reply_.begin(), reply_.begin() + static_cast<std::ptrdiff_t>(first_));
        rest_.assign(reply_.begin() + static_cast<std::ptrdiff_t>(first_), reply_.end());
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override {
        const std::size_t n = std::min(size, pending_.size());
        std::copy(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
        pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n));
        if (n > 0 && pending_.empty() && !rest_.empty()) {
            clock_.advance(stall_s_);  // the reader stalls; meanwhile the rest of the reply arrives
            pending_.swap(rest_);
        }
        return n;
    }
    void reset_input() override {
        pending_.clear();
        rest_.clear();
    }
    int baud_rate() const override { return 38400; }
    void set_baud_rate(int) override {}
    double read_timeout() const override { return timeout_; }
    void set_read_timeout(double s) override { timeout_ = s; }
    bool has_real_timeout() const override { return true; }
    void send_break(double) override {}
    void close() override {}
    bool is_open() const override { return true; }
    std::string name() const override { return "STALL"; }

private:
    Bytes reply_;
    std::size_t first_;
    double stall_s_;
    VirtualClock& clock_;
    Bytes pending_, rest_;
    double timeout_ = 0.2;
    int writes_ = 0;
};

/// The bus test hooks for one test (bus_test_hooks is global): cleared again at the end.
struct BusHooks {
    BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
    ~BusHooks() { detail::bus_test_hooks() = detail::BusTestHooks{}; }
};

}  // namespace

// ------------------------------------------------------------------------------------------ CRC

TEST(bus_crc_vectors) {
    // STOP broadcast quoted by the Python edition and the 22-word status read
    CHECK_EQ(hex_frame(build_frame(0, hx("05 04 2C FF 00"))), "00 05 04 2C FF 00 4D 12");
    CHECK_EQ(hex_frame(build_frame(1, hx("03 90 00 00 16"))), "01 03 90 00 00 16 E9 04");
    CHECK(check_crc(hx("00 05 04 2C FF 00 4D 12")));
    CHECK(check_crc(hx("01 03 90 00 00 16 E9 04")));
    CHECK(!check_crc(hx("01 03 90 00 00 16 E9 05")));
    const std::string text = "123456789";
    CHECK_EQ(crc16(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()), 0x4B37);
    CHECK_EQ(crc16(Bytes{}), 0xFFFF);
}

// ------------------------------------------------------------------------------------------ frames

TEST(bus_fc03_frame_and_decode) {
    Rig r = scripted({hx("01 03 04 00 00 0B FE 7C 83")});
    const std::vector<std::uint16_t> values = r.bus->read_registers(1, 0x9000, 2);
    REQUIRE(values.size() == 2);
    CHECK_EQ(values[0], 0x0000);
    CHECK_EQ(values[1], 0x0BFE);
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]), "01 03 90 00 00 02 E9 0B");
}

TEST(bus_fc05_frames_and_echo) {
    Rig r = scripted({hx("01 05 04 03 FF 00 7D 0A"), hx("01 05 04 03 00 00 3C FA")});
    r.bus->write_coil(1, 0x0403, true);
    r.bus->write_coil(1, 0x0403, false);
    const auto w = r.t->writes();
    REQUIRE(w.size() == 2);
    CHECK_EQ(hex_frame(w[0]), "01 05 04 03 FF 00 7D 0A");
    CHECK_EQ(hex_frame(w[1]), "01 05 04 03 00 00 3C FA");
    // an OFF echo to an ON request is a frame error ...
    Rig bad = scripted({hx("01 05 04 03 00 00 3C FA")}, 0);
    CHECK_THROWS(bad.bus->write_coil(1, 0x0403, true), ModbusFrameError);
    // ... retried like any mismatched frame: wrong on every attempt -> an error after retries + 1 requests
    const std::vector<std::optional<Bytes>> wrong(kRecommendedRetries + 1, hx("01 05 04 03 00 00 3C FA"));
    Rig all = scripted(wrong);
    CHECK_THROWS(all.bus->write_coil(1, 0x0403, true), ModbusFrameError);
    CHECK_EQ(all.t->writes().size(), static_cast<std::size_t>(kRecommendedRetries + 1));
    // a stale reply first, then the right echo: success on the second request
    Rig late = scripted({hx("01 05 04 03 00 00 3C FA"), hx("01 05 04 03 FF 00 7D 0A")});
    late.bus->write_coil(1, 0x0403, true);
    CHECK_EQ(late.t->writes().size(), 2u);
}

TEST(bus_fc06_frame_and_echo) {
    Rig r = scripted({hx("01 06 0D 00 10 00 86 A6")});
    r.bus->write_register(1, 0x0D00, 0x1000);
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]), "01 06 0D 00 10 00 86 A6");
    Rig bad = scripted({resp(1, "06 0D 00 10 01")}, 0);
    CHECK_THROWS(bad.bus->write_register(1, 0x0D00, 0x1000), ModbusFrameError);
    Rig late = scripted({resp(1, "06 0D 00 10 01"), hx("01 06 0D 00 10 00 86 A6")});
    late.bus->write_register(1, 0x0D00, 0x1000);
    CHECK_EQ(late.t->writes().size(), 2u);
}

TEST(bus_fc10_frame_and_echo) {
    Rig r = scripted({hx("01 10 99 00 00 07 AF 57")});
    r.bus->write_registers(1, 0x9900, {0, 5000, 0, 10, 0, 10000, 30});
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]),
             "01 10 99 00 00 07 0E 00 00 13 88 00 00 00 0A 00 00 27 10 00 1E 50 CF");
    // echo with another count
    Rig bad = scripted({resp(1, "10 99 00 00 06")}, 0);
    CHECK_THROWS(bad.bus->write_registers(1, 0x9900, {0, 5000, 0, 10, 0, 10000, 30}), ModbusFrameError);
    Rig late = scripted({resp(1, "10 99 00 00 06"), hx("01 10 99 00 00 07 AF 57")});
    late.bus->write_registers(1, 0x9900, {0, 5000, 0, 10, 0, 10000, 30});
    CHECK_EQ(late.t->writes().size(), 2u);
}

TEST(bus_fc11_variable_length) {
    Rig r = scripted({resp(1, "11 05 EC FF 00 00 20")});
    CHECK_EQ(hex_frame(r.bus->report_slave_id(1)), "EC FF 00 00 20");
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]), "01 11 C0 2C");
}

TEST(bus_fc03_length_checks) {
    // a valid frame with the wrong byte count
    Rig r = scripted({resp(1, "03 02 00 07 00 00")}, 0);
    CHECK_THROWS(r.bus->read_registers(1, 0x9000, 2), ModbusFrameError);
    // retried like any mismatched frame: a stale reply first, then the right one
    Rig late = scripted({resp(1, "03 02 00 07 00 00"), read_resp(1, {7, 0})});
    CHECK_EQ(late.bus->read_registers(1, 0x9000, 2)[0], 7);
    CHECK_EQ(late.t->writes().size(), 2u);
    // a frame too short for the request: incomplete = timeout
    Rig shortr = scripted({resp(1, "03 02 00 07")}, 0);
    CHECK_THROWS(shortr.bus->read_registers(1, 0x9000, 2), ModbusTimeout);
}

TEST(bus_argument_validation_sends_nothing) {
    Rig r = scripted({});
    CHECK_THROWS(r.bus->read_registers(0, 0x9000, 1), std::invalid_argument);  // broadcast read refused
    CHECK_THROWS(r.bus->read_registers(248, 0x9000, 1), std::invalid_argument);
    CHECK_THROWS(r.bus->read_registers(1, 0x9000, 0), std::invalid_argument);
    CHECK_THROWS(r.bus->read_registers(1, 0xFFFF, 2), std::invalid_argument);
    CHECK_THROWS(r.bus->write_register(1, 0x10000, 1), std::invalid_argument);
    CHECK_THROWS(r.bus->write_registers(1, 0x9900, {}), std::invalid_argument);
    CHECK_THROWS(r.bus->write_registers(1, 0x9900, std::vector<std::uint16_t>(124)), std::invalid_argument);
    CHECK_THROWS(r.bus->iai_command(1, 0, 0x09, 0x10000), std::invalid_argument);
    CHECK_THROWS(r.bus->read_registers(1, 0x9000, 1, -1), std::invalid_argument);
    CHECK_THROWS(r.bus->broadcast(hx("03 90 00 00 01"), 0.0), std::invalid_argument);  // never a broadcast read
    CHECK_THROWS(r.bus->broadcast({}, 0.0), std::invalid_argument);
    CHECK(r.t->writes().empty());
}

TEST(bus_large_reads_are_split) {
    auto units = std::make_shared<FakeUnits>();
    for (int i = 0; i < 130; ++i) units->regs[1][0x1000 + i] = static_cast<std::uint16_t>(i);
    auto t = std::make_shared<FakeTransport>([units](const Bytes& f) { return units->respond(f); });
    BusOptions options;
    options.min_gap_s = 0.0;
    ModbusRtu bus(t, options);
    const std::vector<std::uint16_t> values = bus.read_registers(1, 0x1000, 130);
    REQUIRE(values.size() == 130);
    CHECK_EQ(values[0], 0);
    CHECK_EQ(values[129], 129);
    REQUIRE(units->reads.size() == 3);
    CHECK(units->reads[0] == std::make_pair(0x1000, 60));
    CHECK(units->reads[1] == std::make_pair(0x103C, 60));
    CHECK(units->reads[2] == std::make_pair(0x1078, 10));
}

// ------------------------------------------------------------------------------------------ retries

TEST(bus_exception_reply_not_retried) {
    Rig r = scripted({resp(1, "83 02")});
    bool caught = false;
    try {
        r.bus->read_registers(1, 0x7000, 1);
    } catch (const ModbusSlaveError& e) {
        caught = true;
        CHECK_EQ(e.code(), 2);
        CHECK_EQ(e.function(), 3);
        CHECK(std::string(e.what()).find("Illegal Data Address") != std::string::npos);
        CHECK_EQ(e.attempts(), 1);
    }
    CHECK(caught);
    CHECK_EQ(r.t->writes().size(), 1u);
}

TEST(bus_timeout_retried_then_success) {
    Rig r = scripted({std::nullopt, read_resp(1, {7})});
    CHECK_EQ(r.bus->read_registers(1, 0x9002, 1)[0], 7);
    CHECK_EQ(r.t->writes().size(), 2u);
}

TEST(bus_timeout_after_default_three_retries) {
    Rig r = scripted({std::nullopt, std::nullopt, std::nullopt, std::nullopt, read_resp(1, {7})});
    CHECK_EQ(r.bus->retries(), 3);
    bool caught = false;
    try {
        r.bus->read_registers(1, 0x9002, 1);
    } catch (const ModbusTimeout& e) {
        caught = true;
        CHECK_EQ(e.attempts(), 4);
        CHECK(std::string(e.what()).find("gave up after 4 attempts") != std::string::npos);
    }
    CHECK(caught);
    CHECK_EQ(r.t->writes().size(), 4u);
}

TEST(bus_retries_zero_and_per_call_override) {
    Rig r = scripted({std::nullopt, read_resp(1, {7})});
    CHECK_THROWS(r.bus->read_registers(1, 0x9002, 1, 0), ModbusTimeout);
    CHECK_EQ(r.t->writes().size(), 1u);
    CHECK_EQ(r.bus->read_registers(1, 0x9002, 1)[0], 7);  // the default is untouched
    CHECK_EQ(r.bus->retries(), 3);
    Rig z = scripted({std::nullopt}, 0);
    try {
        z.bus->read_registers(1, 0x9002, 1);
        CHECK(false);
    } catch (const ModbusTimeout& e) {
        CHECK(std::string(e.what()).find("gave up") == std::string::npos);
    }
}

TEST(bus_crc_error_retried) {
    Bytes bad = read_resp(1, {7});
    bad.back() ^= 0xFF;
    Rig r = scripted({bad, read_resp(1, {7})});
    CHECK_EQ(r.bus->read_registers(1, 0x9002, 1)[0], 7);
    CHECK_EQ(r.t->writes().size(), 2u);
    Rig all = scripted({bad, bad, bad, bad});
    try {
        all.bus->read_registers(1, 0x9002, 1);
        CHECK(false);
    } catch (const ModbusCrcError& e) {
        CHECK_EQ(e.attempts(), 4);
    }
    CHECK_EQ(all.t->writes().size(), 4u);
    // a bad CRC on an exception frame is retried too
    Bytes bad_exc = resp(1, "83 02");
    bad_exc.back() ^= 0xFF;
    Rig exc = scripted({bad_exc, resp(1, "83 02")});
    CHECK_THROWS(exc.bus->read_registers(1, 0x7000, 1), ModbusSlaveError);
    CHECK_EQ(exc.t->writes().size(), 2u);
}

TEST(bus_wrong_slave_and_function_retried) {
    Rig r = scripted({read_resp(2, {1}), resp(1, "06 90 02 00 01"), read_resp(1, {9})});
    CHECK_EQ(r.bus->read_registers(1, 0x9002, 1)[0], 9);
    CHECK_EQ(r.t->writes().size(), 3u);
    Rig all = scripted({read_resp(2, {1}), read_resp(2, {1}), read_resp(2, {1}), read_resp(2, {1})});
    try {
        all.bus->read_registers(1, 0x9002, 1);
        CHECK(false);
    } catch (const ModbusFrameError& e) {
        CHECK_EQ(e.attempts(), 4);
    }
}

TEST(bus_partial_frame_is_timeout_and_retried) {
    const Bytes full = read_resp(1, {1, 2});
    Rig r = scripted({Bytes(full.begin(), full.begin() + 4), full});
    std::vector<std::pair<FrameDirection, Bytes>> log;
    r.bus->set_frame_logger([&log](FrameDirection d, const Bytes& f) { log.emplace_back(d, f); });
    const std::vector<std::uint16_t> v = r.bus->read_registers(1, 0x9000, 2);
    CHECK_EQ(v[1], 2);
    CHECK_EQ(r.t->writes().size(), 2u);
    bool partial = false;
    for (const auto& e : log)
        if (e.first == FrameDirection::RxPartial && e.second.size() == 4) partial = true;
    CHECK(partial);
}

TEST(bus_transport_error_not_retried) {
    Rig r = scripted({read_resp(1, {1})});
    r.t->fail();
    try {
        r.bus->read_registers(1, 0x9000, 1);
        CHECK(false);
    } catch (const TransportError& e) {
        CHECK_EQ(e.attempts(), 1);
    }
    CHECK(r.t->writes().empty());
    r.t->reopen();
    CHECK_EQ(r.bus->read_registers(1, 0x9000, 1)[0], 1);
}

TEST(bus_closed_bus) {
    Rig r = scripted({});
    r.bus->close();
    r.bus->close();
    CHECK(r.bus->closed());
    CHECK(!r.t->is_open());
    CHECK_THROWS(r.bus->read_registers(1, 0x9000, 2), TransportError);
    CHECK_THROWS(r.bus->broadcast_coil(0x042C, true, 0.0), TransportError);
}

TEST(bus_waits_for_a_late_answer_within_the_timeout) {
    // FC11 waits up to its timeout: an answer arriving after the (instant) fake read still counts
    auto t = std::make_shared<FakeTransport>();
    std::optional<ec_test::Thread> late;  // joined even when the read throws (a raw std::thread would end the process)
    FakeTransport* raw = t.get();
    t->set_responder([&late, raw](const Bytes&) -> std::optional<Bytes> {
        late.emplace(
            [raw] {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                raw->inject(resp(1, "11 05 EC FF 00 00 02"));
            },
            __FILE__, __LINE__);
        return std::nullopt;
    });
    BusOptions options;
    options.retries = 0;
    options.min_gap_s = 0.0;
    ModbusRtu bus(t, options);
    CHECK_EQ(hex_frame(bus.report_slave_id(1, 0, 2.0)), "EC FF 00 00 02");
    if (late) late->join();
}

// ------------------------------------------------------------------------------------------ IAI 0x45

TEST(bus_iai_home_frame_and_accepted_reply) {
    Rig r = scripted({hx("01 45 00 00 10 0D")});
    const IaiResponse result = r.bus->iai_command(1, 0, 0x0A, 7);
    CHECK_EQ(result.rejection, 0);
    CHECK_EQ(result.status, 0);
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]), "01 45 00 0A 00 07 00 00 BC C3");
    Rig servo = scripted({resp(1, "45 00 00")});
    servo.bus->iai_command(1, 0, 0x09, 1);
    CHECK_EQ(hex_frame(servo.t->writes()[0]), "01 45 00 09 00 01 00 00 18 C2");
}

TEST(bus_iai_software_reset_needs_allow_unsafe) {
    Rig r = scripted({resp(1, "45 00 00")});
    CHECK_THROWS(r.bus->iai_command(1, 0, 0x05, 1), UnsafeCommandError);
    CHECK(r.t->writes().empty());
    IaiOptions options;
    options.allow_unsafe = true;
    r.bus->iai_command(1, 0, 0x05, 1, 0, options);
    REQUIRE(r.t->writes().size() == 1);
    CHECK_EQ(hex_frame(r.t->writes()[0]), "01 45 00 05 00 01 00 00 08 C3");
}

TEST(bus_iai_never_send_and_safe_lists) {
    Rig r = scripted({resp(1, "45 00 00"), resp(1, "45 00 00")});
    IaiOptions unsafe;
    unsafe.allow_unsafe = true;
    for (int command : {0x01, 0x02, 0x03, 0x04, 0x0F, 0x10, 0x11, 0x1D, 0x20, 0x3C, 0x4F, 0x73, 0x90, 0x91, 0xDD, 0xE8,
                        0xF3, 0xFF}) {
        try {
            r.bus->iai_command(1, 0, command, 0, 0, unsafe);
            CHECK(false);
        } catch (const UnsafeCommandError& e) {
            CHECK(std::string(e.what()).find("never-send") != std::string::npos);
        }
        CHECK_THROWS(r.bus->broadcast_iai_command(0, command, 0, 0, unsafe, 0.0), UnsafeCommandError);
    }
    CHECK_THROWS(r.bus->iai_command(1, 0, 0x15), UnsafeCommandError);  // not on the safe list
    try {
        r.bus->iai_command(1, 0x05, 0x09, 1);  // ID group other than 0
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("group") != std::string::npos);
    }
    CHECK(r.t->writes().empty());
    r.bus->iai_command(1, 0, 0x15, 0, 2, unsafe);  // neither safe nor never-send: allow_unsafe is enough
    IaiOptions never;
    never.allow_unsafe = true;
    never.allow_never_send = true;
    r.bus->iai_command(1, 0, 0x0F, 0, 0, never);
    CHECK_EQ(r.t->writes().size(), 2u);
}

TEST(bus_iai_rejection_named_and_not_retried) {
    Rig r = scripted({resp(1, "45 11 00"), resp(1, "45 00 00")});
    bool caught = false;
    try {
        r.bus->iai_command(1, 0, reg::kIaiCmdHome, 0x10);
    } catch (const IaiCommandRejected& e) {
        caught = true;
        CHECK_EQ(e.rejection(), 0x11);
        CHECK_EQ(e.command(), 0x0A);
        CHECK_EQ(e.function(), 0x45);
        CHECK_EQ(e.reason(), "1st operand error");
        CHECK(e.hint().find("ORG (item 9.16)") != std::string::npos);
        CHECK(std::string(e.what()).find("1st operand error") != std::string::npos);
    }
    CHECK(caught);
    CHECK_EQ(r.t->writes().size(), 1u);  // a rejection is an answer: never retried
    for (int code : {0x23, 0x24, 0x25, 0x26, 0x27, 0x3A}) {
        Rig c = scripted({resp(1, util::format("45 %02X 07", code))});
        try {
            c.bus->iai_command(1, 0, 0x09, 1);
            CHECK(false);
        } catch (const IaiCommandRejected& e) {
            CHECK_EQ(e.rejection(), code);
            CHECK_EQ(e.status_byte(), 7);
            CHECK(e.reason() != "unknown rejection code");
        }
    }
    Rig unchecked = scripted({resp(1, "45 21 07")});
    IaiOptions no_check;
    no_check.check = false;
    CHECK_EQ(unchecked.bus->iai_command(1, 0, 0x0A, 7, 0, no_check).rejection, 0x21);
}

// ------------------------------------------------------------------------------------------ broadcasts

TEST(bus_broadcast_baud_frame_nothing_read_back) {
    Rig r = scripted({});
    r.t->inject({0x55, 0xAA});  // line noise before
    IaiOptions options;
    options.allow_unsafe = true;
    options.allow_never_send = true;
    const Bytes frame = r.bus->broadcast_iai_command(0, reg::kIaiCmdBaud, 0x14, reg::kIaiBaudOp2, options, 0.0);
    CHECK_EQ(hex_frame(frame), "00 45 00 04 00 14 00 05 25 08");
    REQUIRE(r.t->writes().size() == 1);
    CHECK(r.t->writes()[0] == frame);
    std::uint8_t rest[4];
    CHECK_EQ(r.t->read(rest, 4), 0u);  // the noise was dropped and nothing was waited for or read back
    // the baud change needs allow_never_send even with allow_unsafe
    IaiOptions unsafe;
    unsafe.allow_unsafe = true;
    CHECK_THROWS(r.bus->broadcast_iai_command(0, reg::kIaiCmdBaud, 0x14, 5, unsafe, 0.0), UnsafeCommandError);
    CHECK_EQ(r.t->writes().size(), 1u);
}

TEST(bus_broadcast_coil_and_settle) {
    Rig r = scripted({});
    const double t0 = now_s();
    const Bytes stop = r.bus->broadcast_coil(reg::kCoilStop, true, 0.03);
    CHECK(now_s() - t0 >= 0.029);
    CHECK_EQ(hex_frame(stop), "00 05 04 2C FF 00 4D 12");
    const Bytes reg6 = r.bus->broadcast(hx("06 0D 00 10 00"), 0.0);
    CHECK_EQ(reg6[0], 0);
    CHECK(check_crc(reg6));
    CHECK_EQ(r.t->writes().size(), 2u);
    CHECK_THROWS(r.bus->broadcast_coil(0x042C, true, -1.0), std::invalid_argument);
}

// ------------------------------------------------------------------------------------------ timing

TEST(bus_minimum_gap_between_requests) {
    auto t = std::make_shared<FakeTransport>();
    for (int i = 0; i < 5; ++i) t->enqueue(read_resp(1, {0}));
    ModbusRtu bus(t);  // default 5 ms gap
    CHECK_NEAR(bus.min_gap_s(), 0.005, 1e-12);
    for (int i = 0; i < 5; ++i) bus.read_registers(1, 0x9002, 1);
    const std::vector<double> times = t->write_times();
    REQUIRE(times.size() == 5);
    for (std::size_t i = 1; i < times.size(); ++i) CHECK(times[i] - times[i - 1] >= 0.0049);
    CHECK_THROWS(bus.set_min_gap_s(-0.1), std::invalid_argument);
    CHECK_THROWS(bus.set_retries(-1), std::invalid_argument);
}

TEST(bus_the_gap_is_waited_before_the_bus_is_taken_a_stop_waits_it_with_the_bus) {
    // A5 on the virtual clock: a normal request right after a frame sleeps the gap before it takes the bus (a STOP from
    // another thread is never kept waiting by that sleep); a stop-class request queues at once and waits what is left
    // of the gap with the bus held (spun on a real clock, slept through the seam here). The gap holds either way.
    VirtualClockScope clock;
    BusHooks hooks;
    std::vector<std::pair<double, bool>> waits;  // (seconds left, bus held)
    detail::bus_test_hooks().before_gap_wait = [&waits](double left, bool held) { waits.emplace_back(left, held); };
    auto t = std::make_shared<FakeTransport>();
    for (int i = 0; i < 2; ++i) t->enqueue(read_resp(1, {0}));
    t->enqueue(resp(1, "05 04 2C FF 00"));
    ModbusRtu bus(t);  // default 5 ms gap
    bus.read_registers(1, 0x9002, 1);
    CHECK(waits.empty());  // nothing before the first frame
    bus.read_registers(1, 0x9002, 1);
    REQUIRE(waits.size() == 1u);
    CHECK_NEAR(waits[0].first, kDefaultMinGapS, 1e-9);
    CHECK(!waits[0].second);  // before the bus lock (before: slept with it held)
    bus.write_coil(1, reg::kCoilStop, true);  // a stop frame: stop-class
    REQUIRE(waits.size() == 2u);
    CHECK_NEAR(waits[1].first, kDefaultMinGapS, 1e-9);
    CHECK(waits[1].second);
    const std::vector<double> times = t->write_times();
    REQUIRE(times.size() == 3u);
    for (std::size_t i = 1; i < times.size(); ++i) CHECK(times[i] - times[i - 1] >= kDefaultMinGapS - 1e-9);
}

TEST(bus_a_reply_that_arrived_while_the_reader_stalled_past_the_deadline_is_taken) {
    // A1 on the virtual clock: the reply comes in two chunks and the reader stalls past the response deadline between
    // them (the second chunk is in the buffer by then). Before: a timeout and the request sent again. Now the buffered
    // bytes are read before a timeout is declared: one frame on the wire, the value read.
    VirtualClockScope clock;
    auto line = std::make_shared<StallingLine>(read_resp(1, {7}), 3, 1.0, clock.clock());
    BusOptions o;
    o.min_gap_s = 0.0;
    ModbusRtu bus(line, o);
    CHECK_EQ(bus.read_registers(1, 0x9002, 1)[0], 7);
    CHECK_EQ(line->writes(), 1);
    CHECK_NEAR(line->read_timeout(), 0.2, 1e-12);  // put back after the read that waits for nothing
}

TEST(bus_response_timeout_floors_and_formula) {
    const Bytes fc03_1 = hx("03 90 00 00 01");
    const Bytes fc03_22 = hx("03 90 00 00 16");
    // the 9600 slow-reply floor and the floor below 38400
    ModbusRtu at9600(std::make_shared<FakeTransport>(nullptr, 9600));
    CHECK_NEAR(at9600.response_timeout(1, fc03_1, 7), kSlowFloor9600S, 1e-12);
    ModbusRtu at19200(std::make_shared<FakeTransport>(nullptr, 19200));
    CHECK_NEAR(at19200.response_timeout(1, fc03_1, 7), kSlowFloorBelow38400S, 1e-12);
    // 115200, 22 registers: 3 x 22 + 5 + 10 x (49 + 8) / 115.2 ms = 75.9 ms + 50 ms margin -> the 0.2 s floor
    ModbusRtu fast(std::make_shared<FakeTransport>(nullptr, 115200));
    CHECK_NEAR(fast.response_timeout(1, fc03_22, 5 + 2 * 22), kMinFrameTimeoutS, 1e-12);
    // the formula itself with RTIM 200 ms: 66 + 200 + 570 / 115.2 = 270.948 ms, + 0.05 s
    fast.set_response_delay_ms(1, 200.0);
    CHECK_NEAR(fast.response_delay_ms(1), 200.0, 1e-12);
    CHECK_NEAR(fast.response_delay_ms(2), kDefaultResponseDelayMs, 1e-12);
    CHECK_NEAR(fast.response_timeout(1, fc03_22, 5 + 2 * 22), (66.0 + 200.0 + 570.0 / 115.2) / 1000.0 + 0.05, 1e-9);
    CHECK_NEAR(fast.response_timeout(2, fc03_22, 5 + 2 * 22), kMinFrameTimeoutS, 1e-12);
    // FC10 of 123 registers at 19200: 369 + 5 + 160 / 19.2 ms = 382.33 ms + 0.05 s (above the 0.35 s floor)
    Bytes fc10{0x10, 0x99, 0x00, 0x00, 123};
    CHECK_NEAR(at19200.response_timeout(1, fc10, 8), (369.0 + 5.0 + 160.0 / 19.2) / 1000.0 + 0.05, 1e-9);
    // no baud rate: 115200 assumed
    ModbusRtu none(std::make_shared<FakeTransport>());
    CHECK_NEAR(none.response_timeout(1, fc03_1, 7), kMinFrameTimeoutS, 1e-12);
    CHECK_THROWS(fast.set_response_delay_ms(1, 1001.0), std::invalid_argument);
}

TEST(bus_auto_timeout_only_on_a_real_line) {
    // A real line (has_real_timeout) waits up to response_timeout() (0.2 s): an answer 60 ms late is taken.
    auto inner = std::make_shared<FakeTransport>(nullptr, 115200);
    std::optional<ec_test::Thread> late;  // joined even when a busy PC makes the read time out
    FakeTransport* raw = inner.get();
    inner->set_responder([&late, raw](const Bytes&) -> std::optional<Bytes> {
        late.emplace(
            [raw] {
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                raw->inject(read_resp(1, {42}));
            },
            __FILE__, __LINE__);
        return std::nullopt;
    });
    BusOptions options;
    options.retries = 0;
    options.min_gap_s = 0.0;
    ModbusRtu slow(std::make_shared<SlowLine>(inner), options);
    CHECK_EQ(slow.read_registers(1, 0x9000, 1)[0], 42);
    if (late) late->join();
    // An in-memory transport answers at once: no wait, a missing answer times out immediately.
    Rig r = scripted({std::nullopt}, 0);
    const double t0 = now_s();
    CHECK_THROWS(r.bus->read_registers(1, 0x9000, 1), ModbusTimeout);
    CHECK(now_s() - t0 < 0.1);
}

TEST(bus_probe_shortens_and_restores_the_read_timeout) {
    std::vector<double> seen;
    auto t = std::make_shared<FakeTransport>();
    t->set_read_timeout(0.2);
    FakeTransport* raw = t.get();
    t->set_responder([&seen, raw](const Bytes&) -> std::optional<Bytes> {
        seen.push_back(raw->read_timeout());
        return std::nullopt;
    });
    BusOptions options;
    options.min_gap_s = 0.0;
    ModbusRtu bus(t, options);
    CHECK(!bus.probe(1, 0.02));
    REQUIRE(seen.size() == 1);  // no retries
    CHECK_NEAR(seen[0], 0.02, 1e-12);
    CHECK_NEAR(t->read_timeout(), 0.2, 1e-12);
    CHECK_THROWS(bus.probe(1, 0.0), std::invalid_argument);
}

TEST(bus_send_break_uses_the_transport) {
    Rig r = scripted({});
    r.bus->send_break(0.2);
    REQUIRE(r.t->breaks().size() == 1);
    CHECK_NEAR(r.t->breaks()[0], 0.2, 1e-12);
    CHECK_THROWS(r.bus->send_break(3.0), std::invalid_argument);
    CHECK_THROWS(r.bus->send_break(0.0), std::invalid_argument);
}

// ------------------------------------------------------------------------------------------ logger, reader, threads

TEST(bus_frame_logger_sees_tx_and_rx) {
    std::vector<std::pair<FrameDirection, Bytes>> frames;
    auto t = std::make_shared<FakeTransport>();
    t->enqueue(hx("01 03 04 00 00 0B FE 7C 83"));
    BusOptions options;
    options.frame_logger = [&frames](FrameDirection d, const Bytes& f) { frames.emplace_back(d, f); };
    ModbusRtu bus(t, options);
    bus.read_registers(1, 0x9000, 2);
    REQUIRE(frames.size() == 2);
    CHECK(frames[0].first == FrameDirection::Tx);
    CHECK_EQ(hex_frame(frames[0].second), "01 03 90 00 00 02 E9 0B");
    CHECK(frames[1].first == FrameDirection::Rx);
    CHECK_EQ(hex_frame(frames[1].second), "01 03 04 00 00 0B FE 7C 83");
    CHECK_EQ(std::string(to_string(FrameDirection::RxPartial)), "rx-partial");
}

TEST(bus_broken_logger_does_not_break_io) {
    std::vector<std::string> logged;
    std::mutex m;
    LogHandler previous = set_log_handler([&logged, &m](LogLevel, const std::string&, const std::string& message) {
        std::lock_guard<std::mutex> lock(m);
        logged.push_back(message);
    });
    Rig r = scripted({read_resp(1, {0})});
    r.bus->set_frame_logger([](FrameDirection, const Bytes&) { throw std::runtime_error("logger bug"); });
    CHECK_EQ(r.bus->read_registers(1, 0x9002, 1)[0], 0);
    set_log_handler(previous);
    bool seen = false;
    for (const auto& line : logged)
        if (line.find("frame logger raised") != std::string::npos) seen = true;
    CHECK(seen);
}

TEST(bus_reader_adapter) {
    Rig r = scripted({read_resp(3, {0x1234, 0x5678}), resp(3, "11 05 EC FF 02 00 13")});
    std::unique_ptr<UnitReader> reader = r.bus->reader(3);
    CHECK_EQ(reader->slave(), 3);
    const std::vector<std::uint16_t> v = reader->read_registers(0x0300, 2);
    CHECK_EQ(v[1], 0x5678);
    CHECK_EQ(hex_frame(reader->report_slave_id()), "EC FF 02 00 13");
    CHECK_THROWS(r.bus->reader(0), std::invalid_argument);
}

TEST(bus_two_threads_never_interleave_frames) {
    auto units = std::make_shared<FakeUnits>();
    for (int s = 1; s <= 4; ++s) units->regs[s][0x9005] = static_cast<std::uint16_t>(0x3000 + s);
    auto t = std::make_shared<FakeTransport>([units](const Bytes& f) { return units->respond(f); });
    BusOptions options;
    options.retries = 0;  // an interleaved frame would show as a timeout or a wrong answer
    options.min_gap_s = 0.0;
    ModbusRtu bus(t, options);
    std::atomic<int> wrong{0};
    std::atomic<int> errors{0};
    std::vector<std::thread> threads;
    for (int s = 1; s <= 4; ++s) {
        threads.emplace_back([&bus, &wrong, &errors, s] {
            for (int i = 0; i < 40; ++i) {
                try {
                    if (bus.read_registers(s, 0x9005, 1)[0] != 0x3000 + s) ++wrong;
                } catch (...) {
                    ++errors;
                }
            }
        });
    }
    for (auto& th : threads) th.join();
    CHECK_EQ(wrong.load(), 0);
    CHECK_EQ(errors.load(), 0);
    CHECK_EQ(units->reads.size(), 160u);
    for (const Bytes& w : t->writes()) CHECK(w.size() == 8 && check_crc(w));
}
