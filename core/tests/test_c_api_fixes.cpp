// C ABI 0.6 (fixes) on the simulator: a bus moved onto another transport in place (iad_bus_set_transport), a cell's
// group command refused inside a hold of its bus lock (it would wait for that hold forever), iad_bus_lock with a huge
// finite timeout, and iad_call_json's argument checks (a wrongly typed nested argument, an integer outside its field's
// range). Simulator only: no COM port is opened.
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <thread>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver_testing.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 20; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

iad_sim* open_sim(const std::string& options) {
    iad_sim* sim = nullptr;
    CHECK_EQ(iad_sim_open(options.c_str(), &sim), IAD_OK);
    REQUIRE(sim != nullptr);
    return sim;
}

/// A bus without gaps on a simulator (the transport handle is released: the bus keeps it).
iad_bus* bus_on_sim(iad_sim* sim) {
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_transport(sim, &t), IAD_OK);
    iad_bus_options o;
    iad_bus_options_init(&o);
    o.min_gap_s = 0.0;
    o.retries = 0;
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_bus_open(t, &o, &bus), IAD_OK);
    REQUIRE(bus != nullptr);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    return bus;
}

std::size_t requests(iad_sim* sim) {
    return Json::parse(read_text([&](char* b, size_t s, size_t* r) { return iad_sim_requests_json(sim, b, s, r); }))
        .size();
}

bool eventually(const std::function<bool()>& predicate, double timeout_s = 5.0) {
    const double deadline = steady_now_s() + timeout_s;
    while (steady_now_s() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

struct Frames {
    std::atomic<int> count{0};
    static void IAD_CALL sink(int32_t, const uint8_t*, size_t, void* user) { ++static_cast<Frames*>(user)->count; }
};

/// iad_call_json's result code and output for `function` with `args`.
iad_result call(const char* function, const char* args, std::string* out = nullptr) {
    std::size_t need = 0;
    iad_result r = iad_call_json(function, args, nullptr, 0, &need);
    if (r != IAD_ERR_BUFFER_TOO_SMALL) return r;
    std::string text(need, 'x');
    r = iad_call_json(function, args, &text[0], text.size(), &need);
    text.resize(std::strlen(text.c_str()));
    if (out) *out = text;
    return r;
}

}  // namespace

TEST(capi06_bus_set_transport_moves_the_same_bus) {
    iad_sim* a = open_sim(R"({"serial_number": "UNIT-A"})");
    iad_sim* b = open_sim(R"({"serial_number": "UNIT-B"})");
    iad_bus* bus = bus_on_sim(a);
    CHECK_EQ(iad_bus_set_response_delay(bus, 1, 7.0), IAD_OK);
    Frames frames;
    CHECK_EQ(iad_bus_set_frame_sink(bus, &Frames::sink, &frames), IAD_OK);
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_bus(bus, 1, nullptr, &ax), IAD_OK);
    uint16_t words[2] = {0, 0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    const std::size_t on_a = requests(a);
    CHECK(on_a >= 1);
    CHECK_EQ(requests(b), 0u);

    iad_transport* tb = nullptr;
    CHECK_EQ(iad_sim_transport(b, &tb), IAD_OK);
    CHECK_EQ(iad_bus_set_transport(bus, tb), IAD_OK);
    CHECK_EQ(iad_transport_release(tb), IAD_OK);  // the bus keeps its own reference
    const int seen = frames.count.load();
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(requests(a), on_a);  // the old pipe gets nothing more
    CHECK_EQ(requests(b), 1u);
    CHECK(frames.count.load() >= seen + 2);  // the frame sink stays on the bus
    double delay = 0.0;
    CHECK_EQ(iad_bus_response_delay(bus, 1, &delay), IAD_OK);
    CHECK_EQ(delay, 7.0);  // the settings stay
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);  // the axis opened on the bus follows it
    CHECK_EQ(requests(b), 2u);
    CHECK_EQ(requests(a), on_a);

    // the old transport stays open (a handle of its own still writes to it); the same transport again changes nothing
    iad_transport* ta = nullptr;
    CHECK_EQ(iad_sim_transport(a, &ta), IAD_OK);
    const Json info = Json::parse(read_text([&](char* p, size_t s, size_t* r) {
        return iad_transport_info_json(ta, p, s, r);
    }));
    CHECK(info.find("open")->as_bool());
    CHECK_EQ(iad_bus_set_transport(bus, ta), IAD_OK);  // and back onto the first unit
    CHECK_EQ(iad_bus_set_transport(bus, ta), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);
    CHECK_EQ(requests(a), on_a + 1);
    CHECK_EQ(iad_bus_set_transport(bus, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_transport(nullptr, ta), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_bus_set_transport(bus, reinterpret_cast<iad_transport*>(bus)), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_transport_release(ta), IAD_OK);

    // on a closed bus: moved, still closed (as the Python edition's attribute)
    iad_transport* tb2 = nullptr;
    CHECK_EQ(iad_sim_transport(b, &tb2), IAD_OK);
    CHECK_EQ(iad_bus_shutdown(bus), IAD_OK);
    CHECK_EQ(iad_bus_set_transport(bus, tb2), IAD_OK);
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 2, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_ERR_TRANSPORT);
    CHECK_EQ(iad_transport_release(tb2), IAD_OK);
    iad_axis_close(ax);
    CHECK_EQ(iad_bus_release(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(a), IAD_OK);
    CHECK_EQ(iad_sim_close(b), IAD_OK);
}

TEST(capi06_cell_group_command_refused_inside_a_hold) {
    iad_sim* s1 = open_sim(R"({"slave": 1, "time_scale": 20})");
    iad_sim* s2 = open_sim(R"({"slave": 2, "time_scale": 20})");
    iad_sim* units[] = {s1, s2};
    iad_sim_line* line = nullptr;
    CHECK_EQ(iad_sim_line_open("LINE6", units, 2, &line), IAD_OK);
    REQUIRE(line != nullptr);
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_line_transport(line, &t), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.line = IAD_LINE_DIRECT;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    iad_transport* list[] = {t};
    CHECK_EQ(iad_cell_open_transports(list, 1, nullptr, "1,2", &co, &ao, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_cell_bus(cell, "LINE6", &bus), IAD_OK);
    iad_cell_result results[2];
    size_t count = 0;

    iad_bool locked = IAD_FALSE;
    CHECK_EQ(iad_bus_lock(bus, kNaN, &locked), IAD_OK);
    CHECK_EQ(locked, IAD_TRUE);
    const double t0 = steady_now_s();
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_ERR_NOT_READY);
    CHECK(has(last_json(), "bus lock of port LINE6"));
    CHECK_EQ(iad_cell_home_all(cell, nullptr, kNaN, IAD_METHOD_DEFAULT, IAD_TRUE, results, 2, &count), IAD_ERR_NOT_READY);
    CHECK_EQ(iad_cell_wait_all(cell, nullptr, 1.0, IAD_FALSE, results, 2, &count), IAD_ERR_NOT_READY);
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, 1.0, IAD_FALSE, results, 2, &count), IAD_ERR_NOT_READY);
    CHECK(steady_now_s() - t0 < 2.0);  // refused at once, not stuck behind the hold
    CHECK_EQ(count, 0u);
    uint16_t words[1] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 2, 0x9000, 1, IAD_INT_UNKNOWN, kNaN, IAD_INT_UNKNOWN, words), IAD_OK);  // its own requests go
    size_t axes = 0;
    CHECK_EQ(iad_cell_axis_count(cell, &axes), IAD_OK);  // calls without I/O are not refused
    CHECK_EQ(axes, 2u);
    CHECK_EQ(iad_bus_unlock(bus), IAD_OK);

    // another thread's hold is no reason: the group command waits for it, then runs
    std::atomic<bool> held{false};
    std::atomic<bool> release{false};
    std::thread holder([&] {
        iad_bool l = IAD_FALSE;
        iad_bus_lock(bus, kNaN, &l);
        held = true;
        while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        iad_bus_unlock(bus);
    });
    REQUIRE(eventually([&] { return held.load(); }));
    std::thread releaser([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        release = true;
    });
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(count, 2u);
    releaser.join();
    holder.join();
    CHECK_EQ(iad_bus_close(bus), IAD_OK);  // borrowed: the cell keeps its port
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(iad_sim_line_close(line), IAD_OK);
    for (iad_sim* s : {s1, s2}) CHECK_EQ(iad_sim_close(s), IAD_OK);
}

TEST(polish_link_monitor_check_now_refused_inside_a_hold) {
    // check_now runs the checks on the caller's thread; a fault there stops the other axes on core threads, which would
    // wait for the hold forever (a deadlock before: found by the round-5 review)
    iad_sim* s1 = open_sim(R"({"slave": 1, "time_scale": 20})");
    iad_sim* s2 = open_sim(R"({"slave": 2, "time_scale": 20})");
    iad_sim* s3 = open_sim(R"({"slave": 3, "time_scale": 20})");
    iad_sim* units[] = {s1, s2, s3};
    iad_sim_line* line = nullptr;
    CHECK_EQ(iad_sim_line_open("LINE7", units, 3, &line), IAD_OK);
    REQUIRE(line != nullptr);
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_line_transport(line, &t), IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.line = IAD_LINE_DIRECT;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    iad_transport* list[] = {t};
    CHECK_EQ(iad_cell_open_transports(list, 1, nullptr, "1,2,3", &co, &ao, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    iad_bus* bus = nullptr;
    CHECK_EQ(iad_cell_bus(cell, "LINE7", &bus), IAD_OK);
    CHECK_EQ(iad_cell_monitor_links(cell, 3600.0, nullptr), IAD_OK);
    iad_link_check checks[3];
    size_t count = 9;
    iad_bool locked = IAD_FALSE;
    CHECK_EQ(iad_bus_lock(bus, kNaN, &locked), IAD_OK);
    CHECK_EQ(locked, IAD_TRUE);
    const double t0 = steady_now_s();
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, nullptr, checks, 3, &count), IAD_ERR_NOT_READY);
    CHECK_EQ(count, 0u);
    CHECK(has(last_json(), "bus lock of port LINE7"));
    CHECK(steady_now_s() - t0 < 2.0);
    CHECK_EQ(iad_bus_unlock(bus), IAD_OK);
    // outside a hold it runs
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, nullptr, checks, 3, &count), IAD_OK);
    CHECK_EQ(count, 3u);
    CHECK_EQ(iad_cell_stop_link_monitor(cell), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    CHECK_EQ(iad_sim_line_close(line), IAD_OK);
    for (iad_sim* s : {s1, s2, s3}) CHECK_EQ(iad_sim_close(s), IAD_OK);
}

TEST(capi06_bus_lock_huge_finite_timeout) {
    iad_sim* sim = open_sim("{}");
    iad_bus* bus = bus_on_sim(sim);
    // a free bus is taken for any finite timeout (a steady_clock deadline of 1e12 s would overflow into the past)
    for (double timeout : {1e12, 1e300, DBL_MAX}) {
        iad_bool locked = IAD_FALSE;
        CHECK_EQ(iad_bus_lock(bus, timeout, &locked), IAD_OK);
        CHECK_EQ(locked, IAD_TRUE);
        CHECK_EQ(iad_bus_unlock(bus), IAD_OK);
    }
    // held by another thread: a huge timeout waits as long as it takes
    std::atomic<bool> held{false};
    std::atomic<bool> release{false};
    std::thread holder([&] {
        iad_bool l = IAD_FALSE;
        iad_bus_lock(bus, kNaN, &l);
        held = true;
        while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        iad_bus_unlock(bus);
    });
    REQUIRE(eventually([&] { return held.load(); }));
    std::atomic<bool> done{false};
    iad_bool got = IAD_FALSE;
    iad_result result = IAD_ERR_INTERNAL;
    std::thread waiter([&] {
        result = iad_bus_lock(bus, 9.3e9, &got);
        done = true;
        if (got == IAD_TRUE) iad_bus_unlock(bus);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    CHECK(!done.load());  // still waiting, not timed out at once
    release = true;
    holder.join();
    waiter.join();
    CHECK_EQ(result, IAD_OK);
    CHECK_EQ(got, IAD_TRUE);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi06_call_json_wrongly_typed_nested_argument) {
    const char* p = R"("p": {"no": 15, "name": "x", "class_id": 8, "item_id": 1, "kind": "bits", "bits": [7], )"
                    R"("choices": [[0, "Enable"], [1, "Disable"]]})";
    std::string out;
    CHECK_EQ(call("con_params.decode", (std::string("{") + p + R"(, "raw": 128})").c_str(), &out), IAD_OK);
    CHECK_EQ(out, std::string("1"));
    // a choice label that is no text, a bit that is no number: bad arguments, never "function missing"
    CHECK_EQ(call("con_params.decode",
                  R"({"p": {"no": 1, "class_id": 1, "item_id": 1, "choices": [[0, 1]]}, "raw": 0})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "choices"));
    CHECK_EQ(call("con_params.decode", R"({"p": {"no": 1, "class_id": 1, "item_id": 1, "bits": ["x"]}, "raw": 0})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "bits"));
    CHECK_EQ(call("con_params.encode",
                  R"({"p": {"no": 1, "class_id": 1, "item_id": 1, "choices": [["a", "b"]]}, "raw": 0, "value": 1})"),
             IAD_ERR_INVALID_ARGUMENT);
    // a nested object where a list belongs (Json's accessors): the same
    CHECK_EQ(call("info.decode_alarm_history", R"({"record": {"category_id": 1}, "words": {"a": 1}})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("no.such_function", "{}"), IAD_ERR_UNSUPPORTED);  // kept for what the library lacks
}

TEST(capi06_call_json_integers_out_of_range) {
    std::string out;
    CHECK_EQ(call("alarms.alarm_name", R"({"code": 257})", &out), IAD_OK);
    const std::string name_257 = out;
    // 4294967553 = 0x100000101: refused, never narrowed to 0x101
    CHECK_EQ(call("alarms.alarm_name", R"({"code": 4294967553})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "32-bit"));
    CHECK_EQ(call("alarms.alarm_name", R"({"code": 1e300})"), IAD_ERR_INVALID_ARGUMENT);  // no undefined cast
    CHECK_EQ(call("alarms.alarm_name", R"({"code": -1e19})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("alarms.alarm_name", R"({"code": 257.0})", &out), IAD_OK);  // a whole float in range still counts
    CHECK_EQ(out, name_257);
    CHECK_EQ(call("alarms.alarm_info", R"({"code": 257, "detail": 4294967296})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("con_params.category", R"({"no": 4294967297})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("info.decode_bits", R"({"word": 1, "table": "DSS1", "width": 4294967312})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("con_params.min_positioning_band", R"({"lead_mm": 2.0, "encoder_pulses": 4294967296})"),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("info.io_bit_tables", R"({"mode": {"layout": "2pos", "code": 1e300}})"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(call("info.io_bit_tables", R"({"mode": {"layout": "2pos", "code": 3}})", &out), IAD_OK);
    CHECK(has(out, "DIPM_SINGLE"));
    // where the Python edition answers "none" for an unknown number, a huge one is unknown too
    CHECK_EQ(call("con_params.con_family", R"({"family": 4294967553})", &out), IAD_OK);
    CHECK_EQ(out, std::string("null"));
    CHECK_EQ(call("con_params.audit", R"({"values": [[4294967297, 5]]})", &out), IAD_OK);  // not No.1
    CHECK_EQ(out, std::string("[]"));
}
