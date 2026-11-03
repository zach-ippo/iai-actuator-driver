// Bus unit: baud detection in place, probe, the broadcast baud change and Port (scan, scan_units, ensure_baud_rate,
// set_baud_rate) over FakeTransport responders. Ported from the C# edition's ModbusRtuTests.cs (BaudDetectTests),
// DirectoryAndPortTests.cs and the Python edition's baud / cell tests. No serial port is opened.
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"
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

/// Units on one line: registers per slave, FC03 / FC11 answers, each unit answering only at its own rate, and the
/// broadcast baud change (every unit not in `stay` then answers at the rate the code names, or at `goes_to` when set:
/// a unit whose code table differs).
struct Line {
    std::mutex mutex;
    FakeTransport* t = nullptr;
    std::map<int, std::map<int, std::uint16_t>> regs;
    std::map<int, Bytes> slave_id;  // FC11 data; absent = silent to FC11
    std::map<int, int> rate;        // the rate each unit answers at (0: a line without a baud rate)
    std::set<int> stay;             // units that ignore the broadcast
    std::optional<int> goes_to;

    std::optional<Bytes> respond(const Bytes& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        const int slave = frame[0];
        if (slave == 0) {
            if (frame.size() == 10 && frame[1] == 0x45 && frame[3] == 0x04) {
                const std::optional<int> to = goes_to ? goes_to : baud_by_code(frame[5]);
                for (auto& kv : rate)
                    if (to && !stay.count(kv.first)) kv.second = *to;
            }
            return std::nullopt;
        }
        if (!regs.count(slave) || t->baud_rate() != rate[slave]) return std::nullopt;
        if (frame[1] == 0x11) {
            if (!slave_id.count(slave)) return std::nullopt;
            Bytes pdu{0x11, static_cast<std::uint8_t>(slave_id[slave].size())};
            pdu.insert(pdu.end(), slave_id[slave].begin(), slave_id[slave].end());
            return build_frame(slave, pdu);
        }
        if (frame[1] == 0x03) {
            const int a = (frame[2] << 8) | frame[3];
            const int n = (frame[4] << 8) | frame[5];
            std::vector<std::uint16_t> values;
            for (int i = 0; i < n; ++i) values.push_back(regs[slave].count(a + i) ? regs[slave][a + i] : 0);
            return read_resp(slave, values);
        }
        return std::nullopt;
    }
};

struct LineRig {
    std::shared_ptr<Line> line;
    std::shared_ptr<FakeTransport> t;
};

/// A line with units at `slaves`, the port at `port_rate`, the units answering at `unit_rate` (0, 0: no baud rate).
LineRig line_rig(const std::vector<int>& slaves, int port_rate, int unit_rate) {
    LineRig rig;
    rig.line = std::make_shared<Line>();
    for (int s : slaves) {
        rig.line->regs[s][reg::kPnow + 1] = static_cast<std::uint16_t>(100 * s);
        rig.line->slave_id[s] = {0xEC, 0xFF, 0x02, 0x00, 0x13};
        rig.line->rate[s] = unit_rate;
    }
    auto line = rig.line;
    rig.t = std::make_shared<FakeTransport>([line](const Bytes& f) { return line->respond(f); }, port_rate, "LINE");
    rig.line->t = rig.t.get();
    return rig;
}

BusOptions quick() {
    BusOptions options;
    options.min_gap_s = 0.0;
    return options;
}

bool has_slave0_write(const FakeTransport& t) {
    for (const Bytes& w : t.writes())
        if (!w.empty() && w[0] == 0) return true;
    return false;
}

}  // namespace

// ------------------------------------------------------------------------------------------ detect / probe

TEST(bus_detect_baud_rate_in_place_without_retries) {
    auto t = std::make_shared<FakeTransport>(nullptr, 115200);
    t->set_answer_at_baud_rate(38400);
    t->enqueue(read_resp(1, {0x1CE3}));
    BusOptions options = quick();
    options.retries = 2;
    ModbusRtu bus(t, options);
    CHECK_EQ(bus.detect_baud_rate(1), 38400);
    CHECK_EQ(bus.baud_rate().value_or(0), 38400);
    CHECK(t->baud_rates_set() == std::vector<int>{38400});  // 115200 tried first (no answer, no retry), then 38400
    CHECK_EQ(t->writes().size(), 2u);
    CHECK_EQ(bus.retries(), 2);
    CHECK(t->is_open());  // never closed while detecting
}

TEST(bus_detect_exception_reply_counts_as_answer) {
    auto t = std::make_shared<FakeTransport>(nullptr, 115200);
    t->set_answer_at_baud_rate(115200);
    t->enqueue(resp(1, "83 02"));
    ModbusRtu bus(t, quick());
    CHECK_EQ(bus.detect_baud_rate(1), 115200);
}

TEST(bus_detect_order_and_restore_on_failure) {
    auto t = std::make_shared<FakeTransport>(nullptr, 115200);
    t->set_answer_at_baud_rate(1);  // never answers
    ModbusRtu bus(t, quick());
    try {
        bus.detect_baud_rate(1);
        CHECK(false);
    } catch (const ModbusTimeout& e) {
        CHECK(std::string(e.what()).find("slave 1 did not answer at any of 115200, 38400, 9600") != std::string::npos);
    }
    // 115200 first (the current rate), then 38400, 9600, 230400, 57600, 19200, then the start rate restored
    CHECK(t->baud_rates_set() == (std::vector<int>{38400, 9600, 230400, 57600, 19200, 115200}));
    CHECK_EQ(t->baud_rate(), 115200);
    CHECK(baud_probe_all() == (std::vector<int>{115200, 38400, 9600, 230400, 57600, 19200, 14400, 28800, 76800}));
    // explicit candidates: one read per rate
    auto t2 = std::make_shared<FakeTransport>(nullptr, 115200);
    t2->set_answer_at_baud_rate(1);
    ModbusRtu bus2(t2, quick());
    CHECK_THROWS(bus2.detect_baud_rate(1, {38400, 9600}), ModbusTimeout);
    CHECK_EQ(t2->baud_rate(), 115200);
    CHECK_EQ(t2->writes().size(), 3u);
}

TEST(bus_detect_current_last_when_not_first) {
    auto t = std::make_shared<FakeTransport>(nullptr, 38400);
    t->set_answer_at_baud_rate(1);
    ModbusRtu bus(t, quick());
    CHECK_THROWS(bus.detect_baud_rate(1, {9600, 115200}, 0x9000, false), ModbusTimeout);
    CHECK(t->baud_rates_set() == (std::vector<int>{9600, 115200, 38400, 38400}));  // last entry: the restore
}

TEST(bus_detect_several_slaves_first_answer_settles) {
    LineRig rig = line_rig({3}, 115200, 230400);
    ModbusRtu bus(rig.t, quick());
    try {
        CHECK_EQ(bus.detect_baud_rate(std::vector<int>{1, 2, 3}, {}, 0x9000, true, 0.005), 230400);
    } catch (const std::exception& e) {
        CHECK(false);
    }
    auto none = std::make_shared<FakeTransport>(nullptr, 115200);
    none->set_answer_at_baud_rate(1);
    ModbusRtu bus2(none, quick());
    try {
        bus2.detect_baud_rate(std::vector<int>{1, 2}, {38400}, 0x9000, true, 0.001);
        CHECK(false);
    } catch (const ModbusTimeout& e) {
        CHECK(std::string(e.what()).find("none of slaves 1, 2 answered") != std::string::npos);
    }
}

TEST(bus_detect_needs_a_baud_rate) {
    ModbusRtu bus(std::make_shared<FakeTransport>());
    CHECK(!bus.baud_rate().has_value());
    CHECK(!bus.can_change_baud_rate());
    CHECK_THROWS(bus.detect_baud_rate(1), std::logic_error);
    CHECK_THROWS(bus.set_baud_rate(38400), std::logic_error);
    CHECK_THROWS(bus.detect_baud_rate(std::vector<int>{}), std::invalid_argument);
}

TEST(bus_probe_answers_and_silence) {
    LineRig rig = line_rig({2}, 0, 0);
    ModbusRtu bus(rig.t, quick());
    CHECK(bus.probe(2));
    CHECK(!bus.probe(1, 0.005));
    REQUIRE(rig.t->writes().size() == 2);
    CHECK_EQ(hex_frame(rig.t->writes()[0]), hex_frame(build_frame(2, hx("03 90 00 00 01"))));
    // an exception reply counts; a foreign or corrupt reply does not
    auto t = std::make_shared<FakeTransport>();
    t->enqueue(resp(1, "83 02"));
    t->enqueue(read_resp(5, {1}));
    Bytes bad = read_resp(1, {1});
    bad.back() ^= 0xFF;
    t->enqueue(bad);
    ModbusRtu bus2(t, quick());
    CHECK(bus2.probe(1));
    CHECK(!bus2.probe(1, 0.005));
    CHECK(!bus2.probe(1, 0.005));
    CHECK_EQ(t->writes().size(), 3u);  // never retried
}

// ------------------------------------------------------------------------------------------ change_baud_rate

TEST(bus_change_baud_rate_frame_settle_and_verify) {
    LineRig rig = line_rig({1}, 115200, 115200);
    ModbusRtu bus(rig.t, quick());
    BaudChangeOptions options;
    options.settle_s = 0.05;
    const BaudChangeResult r = change_baud_rate(bus, 230400, {1}, options);
    CHECK_EQ(r.previous, 115200);
    CHECK_EQ(r.baud_rate, 230400);
    CHECK_EQ(r.code, 0x15);
    CHECK_EQ(r.frame, "00 45 00 04 00 15 00 05 74 C8");
    CHECK(r.verified);
    CHECK(!r.probed);
    CHECK(r.code_verified);
    CHECK(r.method == BaudChangeMethod::Direct);
    CHECK_EQ(r.note, std::string(kBaudVolatileNote));
    const auto writes = rig.t->writes();
    REQUIRE(writes.size() == 2);
    CHECK_EQ(hex_frame(writes[0]), "00 45 00 04 00 15 00 05 74 C8");
    CHECK_EQ(hex_frame(writes[1]), hex_frame(build_frame(1, hx("03 90 00 00 01"))));
    CHECK(rig.t->baud_rates_set() == std::vector<int>{230400});
    // the port switched after the settle time: the verifying read (at the new rate) came at least settle_s later
    const auto times = rig.t->write_times();
    CHECK(times[1] - times[0] >= 0.049);
    CHECK_EQ(bus.baud_rate().value_or(0), 230400);
}

TEST(bus_change_baud_rate_probed_path) {
    // the unit went to 9600 instead of 230400: the read at 230400 fails, the probe finds it
    LineRig rig = line_rig({1}, 115200, 115200);
    rig.line->goes_to = 9600;
    ModbusRtu bus(rig.t, quick());
    BaudChangeOptions options;
    options.settle_s = 0.0;
    const BaudChangeResult r = change_baud_rate(bus, 230400, {1}, options);
    CHECK(!r.verified);
    CHECK(r.probed);
    CHECK_EQ(r.baud_rate, 9600);
    CHECK_EQ(bus.baud_rate().value_or(0), 9600);
    // probe order: the new rate, the previous rate, then every IAI rate
    const std::vector<int> rates = rig.t->baud_rates_set();
    REQUIRE(rates.size() >= 3);
    CHECK_EQ(rates[0], 230400);
    CHECK_EQ(rates[1], 115200);
    CHECK_EQ(rates[2], 38400);
    // nothing answers anywhere afterwards: ModbusTimeout
    LineRig gone = line_rig({1}, 115200, 115200);
    gone.line->goes_to = 1;
    ModbusRtu bus2(gone.t, quick());
    CHECK_THROWS(change_baud_rate(bus2, 115200, {1}, options), ModbusTimeout);
}

TEST(bus_change_baud_rate_refusals_send_nothing) {
    LineRig rig = line_rig({1}, 115200, 115200);
    ModbusRtu bus(rig.t, quick());
    try {
        change_baud_rate(bus, 12345, {1});
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("baud rate 12345 has no IAI code") == 0);
        CHECK(std::string(e.what()).find("nothing sent") != std::string::npos);
    }
    CHECK_THROWS(change_baud_rate(bus, 115200, {}), std::invalid_argument);
    ModbusRtu no_rate(std::make_shared<FakeTransport>(), quick());
    CHECK_THROWS(change_baud_rate(no_rate, 115200, {1}), std::logic_error);
    CHECK(rig.t->writes().empty());
    // an unverified code needs no opt-in (allow_unverified_code is ignored): sent, with a logged warning
    std::vector<std::string> warnings;
    const LogHandler previous = set_log_handler([&warnings](LogLevel level, const std::string&, const std::string& m) {
        if (level >= LogLevel::Warning) warnings.push_back(m);
    });
    BaudChangeOptions options;
    options.settle_s = 0.0;
    const BaudChangeResult r = change_baud_rate(bus, 57600, {1}, options);
    const BaudChangeResult never = change_baud_rate(bus, 14400, {1}, options);
    set_log_handler(previous);
    CHECK_EQ(r.code, 0x13);
    CHECK(!r.code_verified);
    CHECK(r.verified);
    CHECK_EQ(r.frame.substr(0, 23), "00 45 00 04 00 13 00 05");
    CHECK(!never.code_verified);
    REQUIRE(warnings.size() == 2u);
    CHECK(warnings[0].find("baud rate 57600: its code is UNVERIFIED") == 0);
    CHECK(warnings[0].find("Sent anyway") != std::string::npos);
    CHECK(warnings[1].find("never probes it") != std::string::npos);
}

TEST(bus_change_baud_rate_38400_without_the_opt_in) {
    // 38400 (code 0x06) is unit-dependent: sent without allow_unverified_code, the check after the switch decides
    LineRig takes = line_rig({1}, 115200, 115200);  // a unit that takes the code (EC-GRBP8M, PCON-CB)
    ModbusRtu bus(takes.t, quick());
    BaudChangeOptions options;
    options.settle_s = 0.0;
    const BaudChangeResult r = change_baud_rate(bus, 38400, {1}, options);
    CHECK_EQ(r.code, 0x06);
    CHECK_EQ(r.baud_rate, 38400);
    CHECK(r.verified);
    CHECK(!r.probed);
    CHECK(r.code_verified);  // so the CLI prints no "not verified" warning
    CHECK_EQ(bus.baud_rate().value_or(0), 38400);
    // a unit that ignores it (EC-R6S rod) keeps its rate: the port is left there and the error says so
    LineRig ignores = line_rig({1}, 115200, 115200);
    ignores.line->stay.insert(1);
    Port port(ignores.t, "", quick());
    try {
        port.set_baud_rate(38400, {1}, options);
        CHECK(false);
    } catch (const VerificationError& e) {
        CHECK(std::string(e.what()).find("still answers at 115200") != std::string::npos);
        CHECK(std::string(e.what()).find("this unit ignores the 38400 code (the EC-R6S rod does too): a power cycle "
                                         "brings it back to 38400") != std::string::npos);
    }
    CHECK_EQ(port.baud_rate().value_or(0), 115200);
}

TEST(bus_change_baud_rate_break_method) {
    LineRig rig = line_rig({1}, 38400, 9600);  // the break drops the unit to 9600
    ModbusRtu bus(rig.t, quick());
    BaudChangeOptions options;
    options.settle_s = 0.0;
    options.method = BaudChangeMethod::Break;
    const BaudChangeResult r = change_baud_rate(bus, 115200, {1}, options);
    CHECK(r.verified);
    CHECK(r.method == BaudChangeMethod::Break);
    REQUIRE(rig.t->breaks().size() == 1);
    CHECK_NEAR(rig.t->breaks()[0], kBreakS, 1e-12);
    CHECK(rig.t->baud_rates_set() == (std::vector<int>{9600, 115200}));
}

// ------------------------------------------------------------------------------------------ Port

TEST(bus_port_scan_finds_answering_slaves_only) {
    LineRig rig = line_rig({1, 3}, 0, 0);
    Port port(rig.t, "", quick());
    CHECK_EQ(port.name(), "LINE");
    CHECK(port.scan({1, 2, 3, 4}, 0.005) == (std::vector<int>{1, 3}));
    CHECK(!port.baud_rate().has_value());
    CHECK(!port.ensure_baud_rate().has_value());  // nothing to detect without a baud rate
}

TEST(bus_port_scan_units_identifies) {
    LineRig rig = line_rig({1, 3, 4}, 0, 0);
    rig.line->slave_id.erase(3);                    // answers FC03, silent to FC11
    rig.line->slave_id[4] = {0xAE, 0xFF};           // a short FC11 answer
    Port port(rig.t, "COMX", quick());
    const std::vector<ScanHit> hits = port.scan_units({1, 2, 3, 4, 1}, 0.005);
    REQUIRE(hits.size() == 3);
    CHECK_EQ(hits[0].name(), "COMX:1");
    CHECK_EQ(hits[0].axis_number(), 0);
    CHECK_EQ(hits[0].system_code.value_or(0), 0xEC);
    CHECK_EQ(hits[0].unit_code.value_or(-1), 0x02);
    CHECK_EQ(hits[0].firmware, "0013");
    CHECK(hits[0].error.empty());
    CHECK_EQ(hits[1].slave, 3);
    CHECK(hits[1].error.find("report slave id") != std::string::npos);
    CHECK(!hits[1].system_code.has_value());
    CHECK(hits[2].error.find("short answer AE FF") != std::string::npos);
    const std::vector<ScanHit> plain = port.scan_units({1}, 0.005, false);
    REQUIRE(plain.size() == 1);
    CHECK(plain[0].slave_id_raw.empty());
    CHECK_THROWS(port.scan_units({0}), std::invalid_argument);
}

TEST(bus_port_ensure_baud_rate_once) {
    LineRig rig = line_rig({2}, 38400, 115200);
    Port port(rig.t, "", quick());
    CHECK(!port.baud_detected());
    CHECK_EQ(port.ensure_baud_rate({1, 2}).value_or(0), 115200);
    CHECK(port.baud_detected());
    const std::size_t writes = rig.t->writes().size();
    CHECK_EQ(port.ensure_baud_rate({1, 2}).value_or(0), 115200);
    CHECK_EQ(rig.t->writes().size(), writes);  // detected once
    const std::vector<ScanHit> hits = port.scan_units({1, 2}, 0.005);
    REQUIRE(hits.size() == 1);
    CHECK_EQ(hits[0].slave, 2);
}

TEST(bus_port_silent_line) {
    LineRig rig = line_rig({}, 115200, 115200);
    Port port(rig.t, "", quick());
    CHECK(port.scan_units({1, 2}, 0.001).empty());  // nothing answers at any rate: an empty result
    try {
        port.detect_baud_rate({1}, {38400});
        CHECK(false);
    } catch (const ModbusTimeout& e) {
        CHECK(std::string(e.what()).find("24 V supply") != std::string::npos);
    }
}

TEST(bus_port_scan_never_sends_to_slave_58) {
    // slave 58 = 0x3A = ':', the Modbus ASCII start: a request to it raises alarm 0x05E on every IAI unit of the line
    // (ON-WIRE on a PCON-CB, 2026-11-03). A 1-247 scan and the baud detection before it skip it, with one warning.
    std::vector<std::string> warnings;
    const LogHandler previous = set_log_handler([&warnings](LogLevel level, const std::string&, const std::string& m) {
        if (level >= LogLevel::Warning) warnings.push_back(m);
    });
    std::vector<int> all;
    for (int s = 1; s <= 247; ++s) all.push_back(s);
    LineRig rig = line_rig({1, 57, 59, 200}, 38400, 38400);
    Port port(rig.t, "", quick());
    const std::vector<int> found = port.scan(all, 0.001);
    // the baud detection over the list alone, at a rate where nothing answers: 58 skipped there too
    LineRig away = line_rig({59}, 115200, 38400);
    Port port2(away.t, "", quick());
    CHECK_EQ(port2.ensure_baud_rate({57, 58, 59}).value_or(0), 38400);
    set_log_handler(previous);
    CHECK(found == (std::vector<int>{1, 57, 59, 200}));
    for (const auto* t : {rig.t.get(), away.t.get()}) {
        CHECK(!t->writes().empty());
        for (const Bytes& w : t->writes()) CHECK(!w.empty() && w[0] != 0x3A);
    }
    int notes = 0;
    for (const std::string& w : warnings)
        if (w.find("slave 58 (0x3A) is never used") != std::string::npos) ++notes;
    CHECK_EQ(notes, 2);  // one line per scan / detection
    // asked for alone or directly: refused, nothing sent
    rig.t->clear_writes();
    CHECK_THROWS(port.scan({58}, 0.001), std::invalid_argument);
    CHECK_THROWS(port.bus()->probe(58, 0.001), std::invalid_argument);
    CHECK_THROWS(port.bus()->read_registers(58, reg::kPnow, 2), std::invalid_argument);
    CHECK_THROWS(port.bus()->detect_baud_rate(58), std::invalid_argument);
    CHECK(rig.t->writes().empty());
}

TEST(bus_port_set_baud_rate) {
    LineRig rig = line_rig({1, 2}, 115200, 115200);
    Port port(rig.t, "", quick());
    BaudChangeOptions options;
    options.settle_s = 0.0;
    const PortBaudResult r = port.set_baud_rate(230400, {2, 1}, options);
    CHECK_EQ(r.port, "LINE");
    CHECK_EQ(r.rate, 230400);
    CHECK(r.change.verified);
    CHECK(r.all_answered());
    CHECK_EQ(r.answered.size(), 2u);
    CHECK_EQ(port.baud_rate().value_or(0), 230400);
    CHECK(!port.baud_detected());
    // a unit that does not follow: reported, not thrown
    LineRig rig2 = line_rig({1, 2}, 115200, 115200);
    Port port2(rig2.t, "", quick());
    rig2.line->stay.insert(2);
    const PortBaudResult r2 = port2.set_baud_rate(230400, {1, 2}, options);
    CHECK(!r2.all_answered());
    CHECK(!r2.answered.at(2));
}

TEST(bus_port_set_baud_rate_refusals) {
    LineRig rig = line_rig({1, 2}, 115200, 115200);
    Port port(rig.t, "", quick());
    CHECK_THROWS(port.set_baud_rate(230400, {}), EcNotReadyError);
    CHECK_THROWS(port.set_baud_rate(12345, {1}), SoftLimitError);  // no IAI code
    CHECK(rig.t->writes().empty());
    rig.line->regs[2][reg::kDsse] = reg::kDsseMove;  // slave 2 is moving
    try {
        port.set_baud_rate(230400, {1, 2});
        CHECK(false);
    } catch (const EcNotReadyError& e) {
        CHECK(std::string(e.what()).find("LINE:2 is not at rest") != std::string::npos);
    }
    CHECK(!has_slave0_write(*rig.t));  // no broadcast went out
    // the unit went elsewhere: VerificationError, the port left where it answers
    LineRig away = line_rig({1}, 115200, 115200);
    away.line->goes_to = 38400;
    Port port2(away.t, "", quick());
    BaudChangeOptions options;
    options.settle_s = 0.0;
    CHECK_THROWS(port2.set_baud_rate(230400, {1}, options), VerificationError);
    CHECK_EQ(port2.baud_rate().value_or(0), 38400);
}

TEST(bus_port_close) {
    LineRig rig = line_rig({1}, 0, 0);
    Port port(rig.t, "P", quick());
    CHECK(!port.closed());
    port.close();
    port.close();
    CHECK(port.closed());
    CHECK(!rig.t->is_open());
    CHECK(port.bus() != nullptr);
}
