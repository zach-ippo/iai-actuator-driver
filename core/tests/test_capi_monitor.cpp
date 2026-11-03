// C ABI 0.5 status monitor additions (src/c_api_monitor.cpp) on the simulator: holding one axis's monitor, the target
// and band of the done rule, the port's owner for enable_fast, the info's new keys and iad_move_handle_poll on the
// monitor's samples. The Python facade (bindings/python/iai_actuator_driver/_native/monitor.py) runs the Python
// edition's monitor tests over these.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

double seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool wait_until(const std::function<bool()>& predicate, double timeout_s = 3.0) {
    const double end = seconds() + timeout_s;
    while (seconds() < end) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
}

iad_axis* sim_axis() {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.002;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim("slider", &sim, &open, &ao, &ax) == IAD_OK);
    REQUIRE(ax != nullptr);
    return ax;
}

void start_monitor(iad_axis* ax) {
    iad_monitor_options o;
    REQUIRE(iad_monitor_options_init(&o) == IAD_OK);
    REQUIRE(iad_axis_start_monitor(ax, &o) == IAD_OK);
}

int64_t polls(iad_axis* ax) {
    iad_monitor_sample s;
    REQUIRE(iad_axis_monitor_latest(ax, &s) == IAD_OK);
    return s.polls;
}

Json info(iad_axis* ax) {
    std::string buffer(16 * 1024, '\0');
    size_t required = 0;
    REQUIRE(iad_axis_monitor_info_json(ax, &buffer[0], buffer.size(), &required) == IAD_OK);
    return Json::parse(buffer.c_str());
}

iad_move_options moves(double speed) {
    iad_move_options o;
    iad_move_options_init(&o);
    o.speed = speed;
    o.accel_g = 0.3;
    return o;
}

/// The requests a bus sent, with the thread that sent each.
struct Sent {
    std::mutex mutex;
    std::vector<std::pair<std::thread::id, std::vector<std::uint8_t>>> frames;
    static void IAD_CALL sink(int32_t direction, const uint8_t* bytes, size_t length, void* user) {
        if (direction != IAD_FRAME_TX) return;
        auto* self = static_cast<Sent*>(user);
        std::lock_guard<std::mutex> lock(self->mutex);
        self->frames.emplace_back(std::this_thread::get_id(), std::vector<std::uint8_t>(bytes, bytes + length));
    }
    std::vector<std::vector<std::uint8_t>> of(std::thread::id thread) {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::vector<std::uint8_t>> out;
        for (const auto& f : frames)
            if (f.first == thread) out.push_back(f.second);
        return out;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        frames.clear();
    }
};

bool is_status_read(const std::vector<std::uint8_t>& f) {
    return f.size() >= 4 && f[1] == 0x03 && ((f[2] << 8) | f[3]) == reg::kMonitorBase;
}

}  // namespace

TEST(capi_monitor_suspend_and_resume_one_axis) {
    iad_axis* ax = sim_axis();
    CHECK_EQ(iad_axis_suspend_monitor(ax, NAN), IAD_ERR_NOT_READY);  // no monitor yet
    CHECK_EQ(iad_axis_resume_monitor(ax), IAD_ERR_INVALID_ARGUMENT);
    start_monitor(ax);
    CHECK(wait_until([&] { return polls(ax) >= 3; }));
    Json j = info(ax);
    CHECK(!j.find("suspended")->as_bool());
    CHECK(j.find("started_at_s")->type() == Json::Type::Number);
    CHECK(j.find("error_detail")->is_null());
    CHECK_EQ(iad_axis_suspend_monitor(ax, -1.0), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_suspend_monitor(ax, NAN), IAD_OK);
    CHECK_EQ(iad_axis_suspend_monitor(ax, 0.5), IAD_OK);  // nests
    CHECK(info(ax).find("suspended")->as_bool());
    const int64_t held = polls(ax);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK_EQ(polls(ax), held);
    CHECK_EQ(iad_axis_resume_monitor(ax), IAD_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    CHECK_EQ(polls(ax), held);  // the outer hold still holds
    CHECK_EQ(iad_axis_resume_monitor(ax), IAD_OK);
    CHECK(wait_until([&] { return polls(ax) > held + 2; }));
    CHECK(!info(ax).find("suspended")->as_bool());
    CHECK_EQ(iad_axis_resume_monitor(ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_suspend_monitor(ax, NAN), IAD_OK);  // a hold left on the handle goes with it
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_monitor_target_of_the_done_rule) {
    iad_axis* ax = sim_axis();
    double target = 0.0, band = 0.0;
    CHECK_EQ(iad_axis_monitor_target(ax, &target, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_monitor_target(ax, &target, &band), IAD_OK);
    CHECK(std::isnan(target) && std::isnan(band));  // no move yet; no monitor needed
    const iad_move_options o = moves(100.0);
    CHECK_EQ(iad_axis_move_to(ax, 80.0, &o, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_monitor_target(ax, &target, &band), IAD_OK);
    CHECK(std::fabs(target - 80.0) < 1e-9);
    CHECK(std::fabs(band - 0.1) < 1e-9);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_monitor_port_owner_lets_enable_fast_change_the_rate) {
    iad_sim* sim = nullptr;
    REQUIRE(iad_sim_open(R"({"baud_rate": 38400, "time_scale": 20})", &sim) == IAD_OK);
    iad_transport* t = nullptr;
    REQUIRE(iad_sim_transport(sim, &t) == IAD_OK);
    iad_bus_options bo;
    iad_bus_options_init(&bo);
    bo.min_gap_s = 0.0;
    iad_bus* bus = nullptr;
    REQUIRE(iad_bus_open(t, &bo, &bus) == IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_bus(bus, 1, nullptr, &ax) == IAD_OK);
    iad_fast_summary s;
    CHECK_EQ(iad_axis_enable_fast(ax, IAD_FAST_BAUD_RATE, IAD_FALSE, &s), IAD_OK);
    CHECK_EQ(s.baud_rate, 38400);  // not the port's owner: the rate is kept
    CHECK(std::string(s.notes).find("shared port") != std::string::npos);
    CHECK_EQ(iad_axis_set_port_owner(ax, IAD_TRUE), IAD_OK);
    CHECK_EQ(iad_axis_enable_fast(ax, IAD_FAST_BAUD_RATE, IAD_FALSE, &s), IAD_OK);
    CHECK_EQ(s.baud_rate, IAD_FAST_BAUD_RATE);
    CHECK_EQ(s.note_count, 0);
    CHECK_EQ(s.monitor, IAD_FALSE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    std::string buffer(4096, '\0');
    size_t required = 0;
    CHECK_EQ(iad_bus_info_json(bus, &buffer[0], buffer.size(), &required), IAD_OK);
    const Json bus_info = Json::parse(buffer.c_str());
    REQUIRE(bus_info.find("closed") != nullptr);
    CHECK(!bus_info.find("closed")->as_bool());  // the owner's close is the binding's: the bus stays open
    CHECK_EQ(iad_bus_close(bus), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);
}

TEST(capi_monitor_move_poll_takes_the_monitors_samples) {
    iad_axis* ax = sim_axis();
    Sent sent;
    REQUIRE(iad_axis_set_frame_sink(ax, &Sent::sink, &sent) == IAD_OK);
    const iad_move_options o = moves(100.0);
    CHECK_EQ(iad_axis_move_to(ax, 60.0, &o, nullptr), IAD_OK);
    start_monitor(ax);
    CHECK(wait_until([&] { return polls(ax) >= 5; }));
    sent.clear();
    iad_move_handle* move = nullptr;
    REQUIRE(iad_axis_start_move_to(ax, 80.0, &o, &move) == IAD_OK);
    iad_bool done = IAD_FALSE;
    iad_status st;
    const double end = seconds() + 10.0;
    while (!done && seconds() < end) CHECK_EQ(iad_move_handle_poll(move, 0.05, &done, &st), IAD_OK);
    CHECK_EQ(done, IAD_TRUE);
    CHECK(std::fabs(st.position_mm - 80.0) <= 0.1 + 1e-6);
    const auto own = sent.of(std::this_thread::get_id());
    CHECK_EQ(own.size(), std::size_t(1));  // the move write only: the status came from the monitor
    for (const auto& f : own) CHECK(!is_status_read(f));
    CHECK_EQ(iad_move_handle_close(move), IAD_OK);
    CHECK_EQ(iad_axis_set_frame_sink(ax, nullptr, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}
