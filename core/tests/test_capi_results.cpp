// C ABI 0.7 (py-fixes) on the simulator (include/iai_actuator_driver.h "library: per-thread results; cell: link monitor
// pass", src/c_api_results.cpp): iad_last_result_json gives the calling thread's own result after a call on another
// thread replaced the handle's, iad_cell_check_links keeps each non-link failure's error record ("error_details"), and
// iad_cell_link_monitor_check_now runs one pass of the cell's link monitor (its states, passes and state events).
// iad_sim_request_hook's IAD_SIM_ABORT and a transport callback's IAD_CALLBACK_ABORT end a call with the caller's own
// error (IAD_ERR_DRIVER), not the port's failure (IAD_ERR_TRANSPORT); a link check does not take it for a link fault.
// Simulator only: no COM port is opened. Timing: time_scale 50, generous deadlines (the PC may be busy).
#include <atomic>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// A JSON / text output read the way a binding does (size query, then the text); "<IAD_...>" when it failed.
template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 50; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

/// iad_last_result_json of the calling thread.
std::string thread_result_text() {
    return read_text([](char* b, size_t n, size_t* r) { return iad_last_result_json(b, n, r); });
}

iad_axis* sim_axis(const char* profile) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 50.0;
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim(profile, &sim, &open, &ao, &ax) == IAD_OK);
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    return ax;
}

iad_recovery_policy quick_policy() {
    iad_recovery_policy p;
    iad_recovery_policy_init(&p);
    p.poll_s = 0.02;
    return p;
}

/// The events queued on the cell within `seconds`.
std::vector<iad_event> cell_events(iad_cell* cell, double seconds) {
    std::vector<iad_event> out;
    const double end = now_s() + seconds;
    while (now_s() < end) {
        iad_event ev;
        if (iad_cell_next_event(cell, 0.02, &ev) != IAD_OK) break;
        if (ev.kind != IAD_EVENT_NONE) out.push_back(ev);
    }
    return out;
}

/// A request hook that fails FC03 at 0x9000 with an error of its own while armed (IAD_SIM_ABORT, or IAD_SIM_FAIL).
struct AbortHook {
    std::atomic<int> armed_calls{0};
    bool armed = false;
    int32_t answer = IAD_SIM_ABORT;
    static int32_t IAD_CALL fn(const uint8_t* request, size_t size, uint8_t* reply, size_t, size_t* length,
                               void* user) {
        auto* h = static_cast<AbortHook*>(user);
        if (!h->armed || size < 4 || request[1] != 0x03 || request[2] != 0x90 || request[3] != 0x00)
            return IAD_SIM_PASS;
        ++h->armed_calls;
        const char text[] = "unexpected frame (test)";
        std::memcpy(reply, text, sizeof text - 1);
        *length = sizeof text - 1;
        return h->answer;
    }
};

/// A callback transport over nothing: write fails with `result` (after iad_callback_error), read gets no byte.
struct DeadPort {
    std::atomic<int> writes{0};
    int32_t result = IAD_CALLBACK_ABORT;
    static int32_t IAD_CALL write(void* user, const uint8_t*, size_t) {
        auto* p = static_cast<DeadPort*>(user);
        ++p->writes;
        iad_callback_error("assert in the test transport");
        return p->result;
    }
    static int32_t IAD_CALL read(void*, uint8_t*, size_t, double, size_t* got) {
        *got = 0;
        return 0;
    }
};

}  // namespace

TEST(capi_results_a_request_hooks_own_error_is_not_a_link_fault) {
    iad_sim* sim = nullptr;
    REQUIRE(iad_sim_open(R"({"slave": 1, "time_scale": 50})", &sim) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_transport(sim, &t) == IAD_OK);
    iad_bus_options bo;
    iad_bus_options_init(&bo);
    bo.min_gap_s = 0.0;
    iad_bus* bus = nullptr;
    REQUIRE(iad_bus_open(t, &bo, &bus) == IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_bus(bus, 1, &ao, &ax) == IAD_OK);
    CHECK_EQ(iad_axis_connect(ax, IAD_TRUE, nullptr), IAD_OK);
    AbortHook hook;
    CHECK_EQ(iad_sim_set_request_hook(sim, &AbortHook::fn, &hook), IAD_OK);
    uint16_t words[2] = {0};
    // the hook's own error: IAD_ERR_DRIVER with its text, asked once
    hook.armed = true;
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_ERR_DRIVER);
    CHECK(has(iad_last_error_message(), "unexpected frame (test)"));
    CHECK_EQ(hook.armed_calls.load(), 1);
    // a link check ends with that error: not "port_vanished", and the link state stays as it was
    iad_link_check lc;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_ERR_DRIVER);
    const auto link_state = [&] {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_axis_link_state(ax, b, n, r); });
    };
    CHECK(!has(link_state(), "port_vanished"));
    // IAD_SIM_FAIL stays the line's failure: IAD_ERR_TRANSPORT, and a link fault of the check
    hook.answer = IAD_SIM_FAIL;
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_ERR_TRANSPORT);
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(std::string(lc.state), std::string("port_vanished"));
    hook.armed = false;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi_results_a_transport_callbacks_own_error_is_not_the_ports) {
    DeadPort port;
    iad_transport_callbacks cb;
    iad_transport_callbacks_init(&cb);
    cb.write = &DeadPort::write;
    cb.read = &DeadPort::read;
    iad_transport* t = nullptr;
    REQUIRE(iad_transport_open_callbacks(&cb, "DEAD", &port, &t) == IAD_OK);
    iad_bus_options bo;
    iad_bus_options_init(&bo);
    bo.min_gap_s = 0.0;
    iad_bus* bus = nullptr;
    REQUIRE(iad_bus_open(t, &bo, &bus) == IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    uint16_t words[2] = {0};
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_ERR_DRIVER);
    CHECK(has(iad_last_error_message(), "assert in the test transport"));
    CHECK_EQ(port.writes.load(), 1);
    port.result = -1;  // any other failure is the port's
    CHECK_EQ(iad_bus_read_registers(bus, 1, 0x9000, 1, 0, kUnset, IAD_INT_UNKNOWN, words), IAD_ERR_TRANSPORT);
    CHECK(has(iad_last_error_message(), "assert in the test transport"));
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
}

TEST(capi_results_last_result_json_is_the_calls_own) {
    // a thread whose calls kept nothing: not ready
    std::string fresh;
    std::thread([&] { fresh = thread_result_text(); }).join();
    CHECK_EQ(fresh, std::string("<IAD_ERR_NOT_READY>"));
    iad_axis* ax = sim_axis("slider");
    CHECK_EQ(iad_axis_track(ax, nullptr, "S", nullptr, nullptr), IAD_OK);
    // a recovery on one thread, a link check on another after it: the handle keeps the check, the thread its report
    std::promise<void> recovered, checked;
    std::future<void> checked_done = checked.get_future();
    std::string own, after_check;
    iad_result code = IAD_ERR_INTERNAL;
    std::thread worker([&] {
        const iad_recovery_policy policy = quick_policy();
        iad_recovery_summary rs;
        code = iad_axis_recover(ax, 5.0, &policy, nullptr, nullptr, nullptr, &rs);
        own = thread_result_text();
        recovered.set_value();
        checked_done.wait();
        after_check = thread_result_text();
    });
    recovered.get_future().wait();
    iad_link_check lc;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    checked.set_value();
    worker.join();
    CHECK_EQ(code, IAD_OK);
    CHECK(has(own, "\"same_unit\":true"));
    CHECK_EQ(after_check, own);
    const std::string handle =
        read_text([&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(handle, "\"state\":\"ok\"") && !has(handle, "same_unit"));
    CHECK_EQ(thread_result_text(), handle);  // this thread's own: the check
    // the buffer rules of a text output; the last error is left as it was
    CHECK_EQ(iad_axis_recover(nullptr, 1.0, nullptr, nullptr, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    const std::string message = iad_last_error_message();
    size_t need = 0;
    CHECK_EQ(iad_last_result_json(nullptr, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(need, handle.size() + 1);
    char small[4];
    CHECK_EQ(iad_last_result_json(small, sizeof small, &need), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(iad_last_result_json(small, sizeof small, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(std::string(iad_last_error_message()), message);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_results_cell_check_links_keeps_each_failures_error_record) {
    iad_sim* sim = nullptr;
    REQUIRE(iad_sim_open(R"({"slave": 1, "time_scale": 50})", &sim) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_transport(sim, &t) == IAD_OK);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.line = IAD_LINE_DIRECT;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    iad_transport* list[] = {t};
    REQUIRE(iad_cell_open_transports(list, 1, "UNIT", "1", &co, &ao, &cell) == IAD_OK);
    iad_link_check checks[1];
    size_t count = 0;
    CHECK_EQ(iad_cell_check_links(cell, nullptr, checks, 1, &count), IAD_OK);
    CHECK_EQ(std::string(checks[0].state), std::string("ok"));
    CHECK(has(thread_result_text(), "\"error_details\":{}"));
    // the status block answers exception 02: not a link fault, the check fails with the unit's exception
    CHECK_EQ(iad_sim_set_unreadable(sim, 0x9000, 22, IAD_TRUE), IAD_OK);
    CHECK_EQ(iad_cell_check_links(cell, nullptr, checks, 1, &count), IAD_OK);
    CHECK_EQ(count, size_t(1));
    CHECK_EQ(std::string(checks[0].state), std::string("error"));
    const std::string kept = thread_result_text();
    CHECK(has(kept, "\"error_details\":{\"UNIT:1\":{"));
    CHECK(has(kept, "\"kind\":\"ModbusSlaveError\""));
    CHECK(has(kept, "\"code\":" + std::to_string(IAD_ERR_SLAVE_EXCEPTION)));
    CHECK(has(kept, "\"exception_code\":2"));
    CHECK(has(kept, "\"errors\":{\"UNIT:1\":\"ModbusSlaveError: "));
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi_results_cell_link_monitor_check_now_is_one_pass_of_the_monitor) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 50.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, &ao, &cell) == IAD_OK);
    auto monitor_json = [&] {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_cell_link_monitor_json(cell, b, n, r); });
    };
    iad_link_check checks[3];
    size_t count = 7;
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, nullptr, checks, 3, &count), IAD_ERR_NOT_READY);
    CHECK_EQ(count, size_t(0));
    CHECK_EQ(iad_cell_monitor_links(cell, 3600.0, nullptr), IAD_OK);  // one pass now, the next in an hour
    const double end = now_s() + 5.0;
    while (now_s() < end && !has(monitor_json(), "\"passes\":1")) sleep_at_least(0.01);
    REQUIRE(has(monitor_json(), "\"passes\":1"));
    cell_events(cell, 0.1);
    iad_axis* first = nullptr;
    REQUIRE(iad_cell_open_axis(cell, "SIM1:1", &first) == IAD_OK);
    auto line = std::dynamic_pointer_cast<SimLine>(capi::find_axis(first)->axis->bus()->transport_ptr());
    REQUIRE(line != nullptr);
    line->power_cycle();
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, "SIM2:1,SIM1:1,NOPE:1", checks, 3, &count), IAD_OK);
    CHECK_EQ(count, size_t(3));
    CHECK(checks[0].ok == IAD_TRUE);
    CHECK_EQ(std::string(checks[1].state), std::string("unit_restarted"));
    CHECK_EQ(std::string(checks[2].state), std::string("error"));
    CHECK(checks[2].ok == IAD_FALSE);
    const std::string j = monitor_json();
    CHECK(has(j, "\"passes\":2"));
    CHECK(has(j, "\"state\":\"unit_restarted\""));
    bool changed = false;
    for (const auto& e : cell_events(cell, 0.3))
        changed = changed || (e.kind == IAD_EVENT_LINK_STATE && std::string(e.axis) == "SIM1:1" &&
                              std::string(e.name) == "unit_restarted");
    CHECK(changed);
    // every axis of the pass by name when none is named
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, nullptr, checks, 3, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK(has(monitor_json(), "\"passes\":3"));
    CHECK_EQ(iad_cell_link_monitor_check_now(nullptr, nullptr, checks, 3, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_stop_link_monitor(cell), IAD_OK);
    CHECK_EQ(iad_cell_link_monitor_check_now(cell, nullptr, checks, 3, &count), IAD_ERR_NOT_READY);
    CHECK_EQ(iad_axis_close(first), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}
