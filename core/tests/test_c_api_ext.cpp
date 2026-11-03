// C ABI 0.2 additions (include/iai_actuator_driver.h "library: log sink, option defaults" to "cell: coordinated
// motion", src/c_api_ext*.cpp) on the simulator: the new result codes and error details, the option defaults and the
// reserved slots given a meaning, push, command-profile moves, acceleration patterns, change_accel, move_sequence, move
// handles (poll in slices, cancel from another thread), the status monitor and the event queue, the sinks, fast mode,
// recovery (track, check_link, recover, a replaced unit, backups), unit info / backup / the self-test, the line, the
// command strategy, the action mode, the safety speed, the cancel entry, the Cell extras, the motion caps and
// coordinated motion. Simulator only: no COM port is opened. Timing: time_scale >= 20 where a wait is measured,
// generous deadlines (the PC may be busy).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver_testing.h"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string message() { return iad_last_error_message(); }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

/// A JSON / text output read the way a binding does: size query, then the text. Live JSON (a monitor's poll count and
/// age) may grow between two calls: IAD_ERR_BUFFER_TOO_SMALL is then answered with a read at the size that call
/// reported, until the text fits (bounded). "<IAD_...>" when it failed (`read`, when given: whether the text was read).
template <class F>
std::string read_text(F call, bool* read = nullptr) {
    if (read != nullptr) *read = false;
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 50; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            if (read != nullptr) *read = true;
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

/// read_text for a call that must succeed: when it fails, a failure of the test that names the result and the last
/// error's message (a refusal prints its reason), and "<IAD_...: message>" as the text.
template <class F>
std::string read_ok(F call, const char* file = EC_TEST_CALLER_FILE, int line = EC_TEST_CALLER_LINE) {
    bool read = false;
    const std::string text = read_text(call, &read);
    if (read) return text;
    const std::string why = text.substr(0, text.size() - 1) + ": " + iad_last_error_message() + ">";
    ::ec_test::fail(file, line, "read_ok: " + why);
    return why;
}

/// An axis on a simulated unit: no gap between frames, 5 ms polls; connected unless told otherwise. The simulated unit
/// starts with its servo on and homed.
iad_axis* sim_axis(const char* profile = "slider", double scale = 20.0, bool connect = true) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = scale;
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    const iad_result r = iad_axis_open_sim(profile, &sim, &open, &ao, &ax);
    CHECK_EQ(std::string(iad_result_name(r)), std::string("IAD_OK"));
    REQUIRE(ax != nullptr);
    if (connect) CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    return ax;
}

/// sim_axis() on a slider with a stepped clock of its own (iad_sim_open "step_s"): each request moves the unit's time
/// on by step_s. A wait fed by the status monitor sends no request of its own, so the monitor sees every step of a
/// move whatever the PC's load (on the library's clock a move at time_scale 20 can end between two of its polls).
iad_axis* stepped_axis(double step_s) {
    iad_sim* sim = nullptr;
    const std::string options = R"({"profile": "slider", "step_s": )" + std::to_string(step_s) + "}";
    REQUIRE(iad_sim_open(options.c_str(), &sim) == IAD_OK);
    iad_transport* t = nullptr;
    CHECK_EQ(iad_sim_transport(sim, &t), IAD_OK);
    CHECK_EQ(iad_sim_close(sim), IAD_OK);  // the transport keeps the unit
    REQUIRE(t != nullptr);
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_transport(t, &open, &ao, &ax), IAD_OK);
    CHECK_EQ(iad_transport_release(t), IAD_OK);  // the axis keeps it
    REQUIRE(ax != nullptr);
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    return ax;
}

/// The simulated unit behind an axis handle of iad_axis_open_sim.
std::shared_ptr<SimulatedUnit> sim_of(iad_axis* ax) {
    return std::dynamic_pointer_cast<SimulatedUnit>(capi::find_axis(ax)->axis->bus()->transport_ptr());
}

iad_move_options moves(double speed, bool wait = true) {
    iad_move_options o;
    iad_move_options_init(&o);
    o.speed = speed;
    o.wait = wait ? IAD_TRUE : IAD_FALSE;
    return o;
}

/// moves() for a stepped_axis(): the unit's time advances with the requests, not with the real time a move's timeout
/// counts, so the timeout is generous (a busy PC slows the polls, not the move they see).
iad_move_options stepped_moves(double speed) {
    iad_move_options o = moves(speed);
    o.timeout_s = 60.0;
    return o;
}

/// Events of a queue until `enough` says so or `seconds` passed.
std::vector<iad_event> drain(const std::function<iad_result(double, iad_event*)>& next, double seconds,
                             const std::function<bool(const std::vector<iad_event>&)>& enough = {}) {
    std::vector<iad_event> out;
    const double end = now_s() + seconds;
    while (now_s() < end) {
        iad_event ev;
        if (next(0.05, &ev) != IAD_OK) break;
        if (ev.kind != IAD_EVENT_NONE) out.push_back(ev);
        if (enough && enough(out)) break;
    }
    return out;
}

bool any_kind(const std::vector<iad_event>& events, int32_t kind, const char* name = nullptr) {
    for (const auto& e : events)
        if (e.kind == kind && (!name || std::string(e.name) == name)) return true;
    return false;
}

/// A fresh temporary directory for the test's files.
std::filesystem::path temp_dir(const std::string& tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path p = std::filesystem::temp_directory_path() / ("iad_capi_ext_" + tag + "_" + std::to_string(stamp));
    std::filesystem::create_directories(p);
    return p;
}

iad_recovery_policy quick_policy() {
    iad_recovery_policy p;
    iad_recovery_policy_init(&p);
    p.poll_s = 0.02;
    return p;
}

}  // namespace

// ============================================================================================ codes and errors

TEST(capi_ext_codes_kinds_and_error_details) {
    CHECK_EQ(iad_abi_version(), (uint32_t(1) << 16) | uint32_t(0));  // 1.0 keeps every 0.2 function
    CHECK_EQ(std::string(iad_result_name(IAD_ERR_RECOVERY)), std::string("IAD_ERR_RECOVERY"));
    CHECK_EQ(std::string(iad_result_name(IAD_ERR_UNIT_REPLACED)), std::string("IAD_ERR_UNIT_REPLACED"));
    CHECK_EQ(std::string(iad_result_name(IAD_ERR_FILE)), std::string("IAD_ERR_FILE"));
    CHECK_EQ(std::string(iad_result_name(IAD_ERR_CANCELLED)), std::string("IAD_ERR_CANCELLED"));
    auto describe = [](auto e) { return capi::describe_error(std::make_exception_ptr(e)); };
    // recovery: the report travels with the error
    auto report = std::make_shared<recovery::RecoveryReport>();
    report->axis = "COM3:1";
    report->cause = "unit_silent";
    const capi::ErrorDescription r = describe(recovery::RecoveryError("not back in time", report));
    CHECK_EQ(r.code, IAD_ERR_RECOVERY);
    CHECK_EQ(r.kind, std::string("RecoveryError"));
    CHECK(has(r.json.dump(), "\"report\":{"));
    CHECK(has(r.json.dump(), "\"cause\":\"unit_silent\""));
    CHECK(has(describe(recovery::RecoveryError("no report")).json.dump(), "\"report\":null"));
    recovery::Identity expected;
    expected.serial_number = "A-1";
    recovery::Identity found;
    found.serial_number = "B-2";
    const capi::ErrorDescription u = describe(recovery::UnitReplacedError("another unit", expected, found, report));
    CHECK_EQ(u.code, IAD_ERR_UNIT_REPLACED);
    CHECK(has(u.json.dump(), "\"expected\":{"));
    CHECK(has(u.json.dump(), "\"A-1\""));
    CHECK(has(u.json.dump(), "\"B-2\""));
    // the cancel entry, files, the line's own kinds (codes unchanged, the class named)
    CHECK_EQ(describe(WaitCancelledError("cancelled")).code, IAD_ERR_CANCELLED);
    CHECK_EQ(describe(WaitCancelledError("cancelled")).kind, std::string("WaitCancelledError"));
    CHECK_EQ(describe(std::ios_base::failure("could not open")).code, IAD_ERR_FILE);
    CHECK_EQ(describe(LineRuleError("no coils")).code, IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(describe(LineRuleError("no coils")).kind, std::string("LineRuleError"));
    CHECK_EQ(describe(GatewayModeError("AUTO")).code, IAD_ERR_NOT_READY);
    CHECK_EQ(describe(GatewayModeError("AUTO")).kind, std::string("GatewayModeError"));
    CHECK_EQ(describe(PositionUnitError("x")).kind, std::string("PositionUnitError"));
    CHECK_EQ(describe(UnsupportedLineError("x")).kind, std::string("UnsupportedLineError"));
}

// ============================================================================================ options

TEST(capi_ext_option_defaults_and_reserved_slots) {
    iad_push_options po;
    CHECK_EQ(iad_push_options_init(&po), IAD_OK);
    CHECK_NEAR(po.speed, 10.0, 1e-12);
    CHECK_NEAR(po.accel_g, 0.1, 1e-12);
    CHECK(std::isnan(po.timeout_s));
    CHECK_EQ(po.direction, 1);
    CHECK_EQ(po.wait, IAD_TRUE);
    iad_sequence_options so;
    CHECK_EQ(iad_sequence_options_init(&so), IAD_OK);
    CHECK(so.speed == 50.0 && so.accel_g == 0.1 && so.band == 0.1);
    CHECK(std::isnan(so.blend_band) && std::isnan(so.timeout_s));
    CHECK_EQ(so.completion, IAD_COMPLETION_DEFAULT);
    iad_monitor_options mo;
    CHECK_EQ(iad_monitor_options_init(&mo), IAD_OK);
    CHECK_NEAR(mo.gap_s, 0.002, 1e-12);
    CHECK_NEAR(mo.link_check_s, 1.0, 1e-12);
    CHECK_EQ(mo.retries, 0);
    CHECK_EQ(mo.events, IAD_EVENTS_ALL);
    CHECK_EQ(mo.queue_capacity, 1024);
    iad_track_options to;
    CHECK_EQ(iad_track_options_init(&to), IAD_OK);
    CHECK(to.alarm_history == IAD_TRUE && to.parameters == IAD_FALSE);
    iad_recovery_policy rp;
    CHECK_EQ(iad_recovery_policy_init(&rp), IAD_OK);
    CHECK(rp.position_tolerance_mm == 0.1 && rp.poll_s == 0.5 && rp.boot_baud_rate == 38400);
    CHECK(rp.restore_baud_rate && rp.restore_control && rp.restore_safety_speed && rp.clear_message_alarms);
    CHECK_EQ(rp.check_parameters, IAD_FALSE);
    iad_check_options co;
    CHECK_EQ(iad_check_options_init(&co), IAD_OK);
    CHECK(co.step_mm == 5.0 && co.speed_mm_s == 10.0 && co.accel_g == 0.1 && co.motion == IAD_FALSE);
    CHECK_EQ(iad_push_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sequence_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_monitor_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_track_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_recovery_policy_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_check_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    // the reserved slots given a meaning: the init writes "not set", and zero is the earlier behaviour
    iad_move_options m;
    iad_move_options_init(&m);
    CHECK(m.accel_pattern == IAD_ACCEL_DEFAULT && std::isnan(m.decel_g) && m.completion == IAD_COMPLETION_DEFAULT);
    m.decel_g = 0.0;
    CHECK(!capi::to_move_options(&m, MoveOptions{}).decel_g.has_value());
    m.decel_g = 0.2;
    m.accel_pattern = IAD_ACCEL_S_MOTION;
    m.completion = IAD_COMPLETION_FAST;
    const MoveOptions mm = capi::to_move_options(&m, MoveOptions{});
    CHECK(mm.decel_g && *mm.decel_g == 0.2);
    CHECK(mm.accel_pattern == std::optional<AccelPattern>(AccelPattern::SMotion));
    CHECK(mm.completion == std::optional<Completion>(Completion::Fast));
    m.completion = 7;
    CHECK_THROWS(capi::to_move_options(&m, MoveOptions{}), std::invalid_argument);
    iad_open_options oo;
    iad_open_options_init(&oo);
    CHECK(oo.line == IAD_LINE_AUTO && std::isnan(oo.position_unit_mm) && oo.fast == IAD_FALSE);
    iad_cell_options ce;
    iad_cell_options_init(&ce);
    CHECK(ce.line == IAD_LINE_AUTO && std::isnan(ce.position_unit_mm));
    const LineOptions line = capi::to_line_options(IAD_LINE_RCON, 0.001);
    CHECK_EQ(line.kind, std::string("rcon"));
    CHECK(line.position_unit_mm && *line.position_unit_mm == 0.001);
    CHECK_THROWS(capi::to_line_options(IAD_LINE_DIRECT, 0.005), std::invalid_argument);
    CHECK_THROWS(capi::to_line_options(9, kNaN), std::invalid_argument);
    oo.line = 9;
    iad_axis* bad = nullptr;
    CHECK_EQ(iad_axis_open_sim("slider", nullptr, &oo, nullptr, &bad), IAD_ERR_INVALID_ARGUMENT);
    CHECK(bad == nullptr);
    // the axis options round trip: completion and push settle (0 in the core = no settle = negative here)
    iad_axis* ax = sim_axis("slider", 20.0, false);
    iad_axis_options ao;
    CHECK_EQ(iad_axis_get_options(ax, &ao), IAD_OK);
    CHECK_EQ(ao.completion, IAD_COMPLETION_SAFE);
    CHECK_NEAR(ao.push_settle_s, 0.15, 1e-12);
    ao.completion = IAD_COMPLETION_FAST;
    ao.push_settle_s = -1.0;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    CHECK_EQ(iad_axis_get_options(ax, &ao), IAD_OK);
    CHECK_EQ(ao.completion, IAD_COMPLETION_FAST);
    CHECK(ao.push_settle_s < 0.0);
    ao.push_settle_s = 0.0;  // zero: the default again
    ao.completion = 0;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    CHECK_EQ(iad_axis_get_options(ax, &ao), IAD_OK);
    CHECK_NEAR(ao.push_settle_s, 0.15, 1e-12);
    CHECK_EQ(ao.completion, IAD_COMPLETION_SAFE);
    ao.method = IAD_METHOD_REGISTER;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    CHECK_EQ(iad_axis_get_options(ax, &ao), IAD_OK);
    CHECK_EQ(ao.method, IAD_METHOD_REGISTER);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ motion extras

TEST(capi_ext_push_profile_moves_patterns_and_sequences) {
    iad_axis* ax = sim_axis("slider");
    iad_push_result pr;
    CHECK_EQ(iad_axis_push(ax, 80.0, 10.0, 30.0, nullptr, &pr), IAD_OK);
    CHECK_EQ(pr.outcome, IAD_PUSH_MISSED);
    CHECK_EQ(std::string(pr.outcome_name), std::string("missed"));
    CHECK_NEAR(pr.position_mm, 90.0, 1e-9);
    CHECK_EQ(pr.status.push_missed, IAD_TRUE);
    const std::string kept = read_text([&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(kept, "\"outcome\":\"missed\""));
    CHECK_EQ(iad_axis_push(ax, 80.0, 10.0, 75.0, nullptr, &pr), IAD_ERR_SOFT_LIMIT);  // above the force cap
    iad_push_options po;
    iad_push_options_init(&po);
    po.direction = 2;
    CHECK_EQ(iad_axis_push(ax, 80.0, 10.0, 30.0, &po, &pr), IAD_ERR_INVALID_ARGUMENT);
    // (0.8) a direction of 0 is the default, +1 (capi08_defaults_zero_in_a_move_is_the_default)
    // an acceleration pattern (CTLF b6-b7) through the move options
    iad_move_options s = moves(50.0);
    s.accel_pattern = IAD_ACCEL_S_MOTION;
    iad_status st;
    CHECK_EQ(iad_axis_move_to(ax, 100.0, &s, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 100.0, 0.11);
    CHECK_EQ(sim_of(ax)->accel_pattern(), std::string("s_motion"));
    s.accel_pattern = 9;
    CHECK_EQ(iad_axis_move_to(ax, 90.0, &s, &st), IAD_ERR_INVALID_ARGUMENT);
    // change_accel of a running move; refused without one
    CHECK_EQ(iad_axis_change_accel(ax, 0.2, ptr_of(change_opts(0, kNaN)), nullptr), IAD_ERR_NOT_READY);
    iad_move_options slow = moves(20.0, false);
    CHECK_EQ(iad_axis_move_to(ax, 140.0, &slow, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_change_accel(ax, 0.2, ptr_of(change_opts(1, 30.0)), &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 140.0, 0.11);
    // move_sequence: one status per point; a too small array never stops it
    iad_sequence_point pts[3];
    for (auto& p : pts) {
        std::memset(&p, 0, sizeof p);
        p.struct_size = sizeof p;
        p.speed = p.accel_g = p.band = kNaN;
    }
    pts[0].position = 20.0;
    pts[1].position = 40.0;
    pts[1].speed = 80.0;
    pts[2].position = 60.0;
    iad_sequence_options so;
    iad_sequence_options_init(&so);
    so.speed = 100.0;
    iad_status statuses[3];
    size_t count = 99;
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, statuses, 3, &count), IAD_OK);
    CHECK_EQ(count, size_t(3));
    CHECK_NEAR(statuses[2].position_mm, 60.0, 0.11);
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, statuses, 1, &count), IAD_OK);
    CHECK_EQ(count, size_t(3));
    pts[1].position = 1e6;  // checked before the first frame
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, statuses, 3, &count), IAD_ERR_SOFT_LIMIT);
    CHECK_EQ(count, size_t(0));
    CHECK_EQ(iad_axis_move_sequence(ax, nullptr, 2, &so, nullptr, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // the command-profile move: a deceleration of its own (FC17 0x0F00 + PCRF)
    iad_axis* pcon = sim_axis("pcon");
    iad_move_options profile = moves(100.0);
    profile.accel_g = 0.3;
    profile.decel_g = 0.1;
    CHECK_EQ(iad_axis_move_to(pcon, 120.0, &profile, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 120.0, 0.11);
    CHECK_EQ(sim_of(pcon)->profile_moves(), 1);
    profile.decel_g = 1.5;  // above MAXD: refused, nothing sent
    CHECK_EQ(iad_axis_move_to(pcon, 100.0, &profile, &st), IAD_ERR_SOFT_LIMIT);
    CHECK(has(message(), "MAXD"));
    CHECK_EQ(iad_axis_close(pcon), IAD_OK);
}

// ============================================================================================ move handles

TEST(capi_ext_move_handles_poll_wait_and_close) {
    iad_axis* ax = sim_axis("slider");
    iad_move_options o = moves(50.0);
    iad_move_handle* h = nullptr;
    CHECK_EQ(iad_axis_start_move_to(ax, 100.0, &o, &h), IAD_OK);
    REQUIRE(h != nullptr);
    iad_move_info info;
    CHECK_EQ(iad_move_handle_result(h, &info), IAD_OK);
    CHECK_NEAR(info.target, 100.0, 1e-9);
    CHECK(info.deadline_s > info.sent_at_s);
    CHECK_EQ(info.completion, IAD_COMPLETION_SAFE);
    // short slices: the move runs on between them
    iad_bool done = IAD_FALSE;
    iad_status st;
    for (int i = 0; i < 400 && !done; ++i) CHECK_EQ(iad_move_handle_poll(h, 0.05, &done, &st), IAD_OK);
    REQUIRE(done == IAD_TRUE);
    CHECK_NEAR(st.position_mm, 100.0, 0.11);
    CHECK_EQ(iad_move_handle_done(h, &done, nullptr), IAD_OK);
    CHECK_EQ(done, IAD_TRUE);
    CHECK_EQ(iad_move_handle_wait(h, kNaN, &st), IAD_OK);  // a late wait returns the result
    CHECK_EQ(iad_move_handle_result(h, &info), IAD_OK);
    CHECK(info.has_result == IAD_TRUE && info.cancelled == IAD_FALSE);
    CHECK_NEAR(info.result.position_mm, 100.0, 0.11);
    const std::string j = read_text([&](char* b, size_t n, size_t* r) { return iad_move_handle_json(h, b, n, r); });
    CHECK(has(j, "\"done\":true"));
    CHECK(has(j, "\"target_mm\":100"));
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
    CHECK_EQ(iad_move_handle_close(nullptr), IAD_OK);
    CHECK_EQ(iad_move_handle_done(h, &done, nullptr), IAD_ERR_INVALID_ARGUMENT);
    // move_by and jog, waited
    CHECK_EQ(iad_axis_start_move_by(ax, -50.0, &o, &h), IAD_OK);
    CHECK_EQ(iad_move_handle_wait(h, 30.0, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 50.0, 0.11);
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
    CHECK_EQ(iad_axis_start_jog(ax, 1.0, nullptr, &h), IAD_OK);
    CHECK_EQ(iad_move_handle_wait(h, 30.0, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 51.0, 0.11);
    // a refused start gives no handle; handle kinds are not mixed up
    iad_move_handle* none = reinterpret_cast<iad_move_handle*>(1);
    CHECK_EQ(iad_axis_start_move_to(ax, 1e6, &o, &none), IAD_ERR_SOFT_LIMIT);
    CHECK(none == nullptr);
    CHECK_EQ(iad_axis_status(reinterpret_cast<iad_axis*>(h), &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "a move handle was passed as an axis handle"));
    CHECK_EQ(iad_move_handle_close(reinterpret_cast<iad_move_handle*>(ax)), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(reinterpret_cast<iad_axis*>(h)), IAD_ERR_INVALID_ARGUMENT);
    // the handle keeps its axis object alive: a finished move still answers after the axis handle is closed
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_move_handle_wait(h, kNaN, &st), IAD_OK);
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
}

TEST(capi_ext_move_handle_cancel_here_and_from_another_thread) {
    iad_axis* ax = sim_axis("slider", 1.0);  // real time: 120 mm at 20 mm/s = 6 s
    iad_move_options slow = moves(20.0);
    iad_move_handle* h = nullptr;
    CHECK_EQ(iad_axis_start_move_to(ax, 120.0, &slow, &h), IAD_OK);
    sleep_at_least(0.2);
    iad_status st;
    CHECK_EQ(iad_move_handle_cancel(h, 1, kNaN, &st), IAD_OK);
    CHECK_EQ(st.moving, IAD_FALSE);
    CHECK(st.position_mm < 120.0);
    iad_move_info info;
    CHECK_EQ(iad_move_handle_result(h, &info), IAD_OK);
    CHECK(info.cancelled == IAD_TRUE && info.has_result == IAD_TRUE);
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
    // a cancel while another thread polls the handle: the axis stops at once, the poll returns
    CHECK_EQ(iad_axis_start_move_to(ax, 0.0, &slow, &h), IAD_OK);
    std::atomic<bool> returned{false};
    iad_result polled = IAD_ERR_INTERNAL;
    std::thread poller([&] {
        iad_bool done = IAD_FALSE;
        polled = iad_move_handle_poll(h, 2.0, &done, nullptr);
        returned = true;
    });
    sleep_at_least(0.3);
    const double t0 = now_s();
    CHECK_EQ(iad_move_handle_cancel(h, 0, kNaN, &st), IAD_OK);
    poller.join();
    CHECK(returned.load());
    CHECK(now_s() - t0 < 10.0);
    (void)polled;  // done, not done, or the settled-away error: the stop is what counts
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(st.moving, IAD_FALSE);
    CHECK_EQ(iad_move_handle_result(h, &info), IAD_OK);
    CHECK_EQ(info.cancelled, IAD_TRUE);
    CHECK_EQ(iad_move_handle_close(h), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ cancel entry

TEST(capi_ext_cancel_ends_a_blocking_move_on_another_thread) {
    iad_axis* ax = sim_axis("slider", 1.0);
    iad_result moved = IAD_ERR_INTERNAL;
    std::string kind;
    std::thread mover([&] {
        iad_move_options slow = moves(20.0);
        iad_status st;
        moved = iad_axis_move_to(ax, 140.0, &slow, &st);
        kind = last_json();
    });
    sleep_at_least(0.3);
    const double t0 = now_s();
    CHECK_EQ(iad_axis_cancel(ax), IAD_OK);
    mover.join();
    CHECK(now_s() - t0 < 5.0);
    CHECK_EQ(std::string(iad_result_name(moved)), std::string("IAD_ERR_CANCELLED"));
    CHECK(has(kind, "\"kind\":\"WaitCancelledError\""));
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.position_mm < 140.0);
    CHECK_EQ(iad_axis_cancel(ax), IAD_OK);  // nothing waits: nothing happens
    CHECK_EQ(iad_axis_move_to(ax, 30.0, nullptr, &st), IAD_OK);  // later calls are not affected
    CHECK_EQ(iad_axis_cancel(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ monitor and events

TEST(capi_ext_status_monitor_and_event_queue) {
    // the stepped clock: the waits below are fed by the monitor, so it sees each move from its start to its end
    iad_axis* ax = stepped_axis(0.05);
    iad_event ev;
    CHECK_EQ(iad_axis_next_event(ax, 0.0, &ev), IAD_OK);
    CHECK_EQ(ev.kind, IAD_EVENT_NONE);
    iad_monitor_sample sample;
    CHECK_EQ(iad_axis_monitor_latest(ax, &sample), IAD_ERR_NOT_READY);
    iad_monitor_options mo;
    iad_monitor_options_init(&mo);
    mo.queue_capacity = 0;
    CHECK_EQ(iad_axis_start_monitor(ax, &mo), IAD_ERR_INVALID_ARGUMENT);  // nothing started
    CHECK_EQ(iad_axis_monitor_latest(ax, &sample), IAD_ERR_NOT_READY);
    iad_monitor_options_init(&mo);
    mo.gap_s = -1.0;
    CHECK_EQ(iad_axis_start_monitor(ax, &mo), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_start_monitor(ax, nullptr), IAD_OK);
    iad_bool got = IAD_FALSE;
    CHECK_EQ(iad_axis_monitor_wait_update(ax, 0, 5.0, &got, &sample), IAD_OK);
    CHECK_EQ(got, IAD_TRUE);
    CHECK(sample.seq > 0);
    CHECK_EQ(sample.running, IAD_TRUE);
    CHECK_EQ(iad_axis_monitor_wait_update(ax, sample.seq, 5.0, &got, &sample), IAD_OK);
    CHECK_EQ(got, IAD_TRUE);
    // a move: status changes and the done edge arrive as events named by the axis
    iad_status st;
    iad_move_options o = stepped_moves(100.0);
    CHECK_EQ(iad_axis_move_to(ax, 60.0, &o, &st), IAD_OK);
    auto next = [&](double t, iad_event* e) { return iad_axis_next_event(ax, t, e); };
    std::vector<iad_event> events =
        drain(next, 10.0, [](const std::vector<iad_event>& v) { return any_kind(v, IAD_EVENT_DONE); });
    CHECK(any_kind(events, IAD_EVENT_STATUS));
    CHECK(any_kind(events, IAD_EVENT_DONE));
    REQUIRE(!events.empty());
    for (const auto& e : events) CHECK_EQ(e.source, IAD_EVENT_SOURCE_MONITOR);  // (0.7) its polls are the monitor's
    CHECK_EQ(std::string(events.front().axis), std::string("SIM:1"));
    CHECK(events.front().sequence >= 1);
    CHECK_EQ(iad_axis_monitor_wait_done(ax, 5.0, &got, &st), IAD_OK);
    CHECK_EQ(got, IAD_TRUE);
    CHECK_EQ(iad_axis_monitor_latest(ax, &sample), IAD_OK);
    CHECK(sample.polls > 0);
    const std::string info = read_text([&](char* b, size_t n, size_t* r) { return iad_axis_monitor_info_json(ax, b, n, r); });
    CHECK(has(info, "\"running\":true"));
    CHECK(has(info, "\"link_state\":\"ok\""));
    // a running monitor keeps its options; the queue takes new settings: done events only
    iad_monitor_options_init(&mo);
    mo.events = IAD_EVENTS_DONE;
    CHECK_EQ(iad_axis_start_monitor(ax, &mo), IAD_OK);
    drain(next, 0.2);
    CHECK_EQ(iad_axis_move_to(ax, 20.0, &o, &st), IAD_OK);
    events = drain(next, 10.0, [](const std::vector<iad_event>& v) { return any_kind(v, IAD_EVENT_DONE); });
    CHECK(any_kind(events, IAD_EVENT_DONE));
    for (const auto& e : events) CHECK_EQ(e.kind, IAD_EVENT_DONE);
    int done_events = 0;  // hooked once: one done edge per move
    for (const auto& e : events) done_events += e.kind == IAD_EVENT_DONE;
    CHECK_EQ(done_events, 1);
    // a full queue drops the oldest and says so
    mo.events = IAD_EVENTS_STATUS;
    mo.queue_capacity = 1;
    CHECK_EQ(iad_axis_start_monitor(ax, &mo), IAD_OK);
    CHECK_EQ(iad_axis_move_to(ax, 80.0, &o, &st), IAD_OK);  // a status event per step of the move
    CHECK_EQ(iad_axis_next_event(ax, 1.0, &ev), IAD_OK);
    CHECK_EQ(ev.kind, IAD_EVENT_STATUS);
    CHECK(ev.dropped > 0);
    CHECK_EQ(iad_axis_stop_monitor(ax, kNaN), IAD_OK);
    CHECK_EQ(iad_axis_stop_monitor(ax, kNaN), IAD_OK);  // no monitor: no-op
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // closing the handle wakes a thread waiting for an event
    iad_axis* other = sim_axis("slider", 20.0, false);
    iad_result waited = IAD_OK;
    std::thread waiter([&] {
        iad_event e;
        waited = iad_axis_next_event(other, 30.0, &e);
    });
    sleep_at_least(0.2);
    const double t0 = now_s();
    CHECK_EQ(iad_axis_close(other), IAD_OK);
    waiter.join();
    CHECK(now_s() - t0 < 5.0);
    CHECK_EQ(waited, IAD_ERR_INVALID_ARGUMENT);
}

namespace {
struct SinkLog {
    std::atomic<int> frames{0};
    std::atomic<int> tx{0};
    std::atomic<int> lines{0};
    std::string last_line;
};
void IAD_CALL frame_sink(int32_t direction, const uint8_t* bytes, size_t length, void* user) {
    auto* log = static_cast<SinkLog*>(user);
    if (bytes && length > 0) ++log->frames;
    if (direction == IAD_FRAME_TX) ++log->tx;
}
void IAD_CALL log_sink(int32_t level, const char* source, const char* message, void* user) {
    auto* log = static_cast<SinkLog*>(user);
    ++log->lines;
    log->last_line = std::to_string(level) + " " + source + " " + message;
}
}  // namespace

TEST(capi_ext_frame_and_log_sinks) {
    SinkLog log;
    iad_axis* ax = sim_axis("slider", 20.0, false);
    CHECK_EQ(iad_axis_set_frame_sink(ax, frame_sink, &log), IAD_OK);
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(log.frames.load() >= 2);  // the request and the answer
    CHECK(log.tx.load() >= 1);
    CHECK_EQ(iad_axis_set_frame_sink(ax, nullptr, nullptr), IAD_OK);
    const int before = log.frames.load();
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(log.frames.load(), before);
    CHECK_EQ(iad_set_log_sink(log_sink, &log), IAD_OK);
    log_warning("iai_actuator_driver.test", "hello sink");
    CHECK_EQ(log.lines.load(), 1);
    CHECK_EQ(log.last_line, std::string("2 iai_actuator_driver.test hello sink"));
    CHECK_EQ(iad_set_log_sink(nullptr, nullptr), IAD_OK);
    log_warning("iai_actuator_driver.test", "nobody listens");
    CHECK_EQ(log.lines.load(), 1);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ fast mode

TEST(capi_ext_fast_mode) {
    // the stepped clock: the move's wait is fed by the monitor, so the monitor sees the move and its done edge
    iad_axis* ax = stepped_axis(0.05);
    iad_fast_summary s;
    CHECK_EQ(iad_axis_enable_fast(ax, -5, 1, &s), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_enable_fast(ax, IAD_FAST_BAUD_RATE, 1, &s), IAD_OK);
    CHECK_EQ(s.completion, IAD_COMPLETION_FAST);
    CHECK_EQ(s.monitor, IAD_TRUE);
    CHECK_EQ(s.baud_rate, 0);      // a simulated unit without a baud rate: the rate is kept ...
    CHECK(s.note_count >= 1);      // ... and the reason noted
    CHECK(std::strlen(s.notes) > 0);
    iad_bool on = IAD_FALSE;
    CHECK_EQ(iad_axis_fast_mode(ax, &on), IAD_OK);
    CHECK_EQ(on, IAD_TRUE);
    CHECK(has(read_text([&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); }),
              "\"completion\":\"fast\""));
    // the monitor enable_fast started feeds this handle's queue
    iad_move_options o = stepped_moves(100.0);
    iad_status st;
    CHECK_EQ(iad_axis_move_to(ax, 50.0, &o, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 50.0, 0.11);
    auto next = [&](double t, iad_event* e) { return iad_axis_next_event(ax, t, e); };
    CHECK(any_kind(drain(next, 10.0, [](const std::vector<iad_event>& v) { return any_kind(v, IAD_EVENT_DONE); }),
                   IAD_EVENT_DONE));
    CHECK_EQ(iad_axis_disable_fast(ax, &s), IAD_OK);
    CHECK_EQ(s.completion, IAD_COMPLETION_SAFE);
    CHECK_EQ(s.monitor, IAD_FALSE);
    CHECK_EQ(iad_axis_fast_mode(ax, &on), IAD_OK);
    CHECK_EQ(on, IAD_FALSE);
    CHECK_EQ(iad_axis_fast_mode(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // fast at open
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_open_options open;
    iad_open_options_init(&open);
    open.fast = IAD_TRUE;
    CHECK_EQ(iad_axis_open_sim("slider", &sim, &open, nullptr, &ax), IAD_OK);
    CHECK_EQ(iad_axis_fast_mode(ax, &on), IAD_OK);
    CHECK_EQ(on, IAD_TRUE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ recovery

TEST(capi_ext_recovery_after_a_power_cycle) {
    iad_axis* ax = sim_axis("gripper", 50.0);
    auto text = [&](iad_result (*f)(iad_axis*, char*, size_t, size_t*)) {
        return read_text([&](char* b, size_t n, size_t* r) { return f(ax, b, n, r); });
    };
    CHECK_EQ(text(iad_axis_link_state), std::string("ok"));
    CHECK_EQ(text(iad_axis_session_json), std::string("<IAD_ERR_NOT_READY>"));
    CHECK_EQ(iad_axis_track(ax, nullptr, "G", "x", nullptr), IAD_ERR_INVALID_ARGUMENT);  // a bad row list
    CHECK_EQ(iad_axis_track(ax, nullptr, "G", nullptr, nullptr), IAD_OK);
    CHECK(has(text(iad_axis_session_json), "\"name\":\"G\""));
    iad_link_check lc;
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(lc.ok, IAD_TRUE);
    CHECK_EQ(std::string(lc.state), std::string("ok"));
    CHECK(lc.status.raw_words > 0);
    sim_of(ax)->power_cycle();
    CHECK_EQ(iad_axis_check_link(ax, &lc), IAD_OK);
    CHECK_EQ(std::string(lc.state), std::string("unit_restarted"));
    CHECK_EQ(lc.ok, IAD_FALSE);
    CHECK(lc.reason_count > 0);
    CHECK(has(lc.reasons, "uptime"));
    CHECK(has(text(iad_axis_last_result_json), "\"state\":\"unit_restarted\""));
    CHECK_EQ(text(iad_axis_link_state), std::string("unit_restarted"));
    auto next = [&](double t, iad_event* e) { return iad_axis_next_event(ax, t, e); };
    std::vector<iad_event> events = drain(next, 2.0, [](const std::vector<iad_event>& v) { return !v.empty(); });
    CHECK(any_kind(events, IAD_EVENT_LINK, "unit_restarted"));
    // recover: never moves anything; the report is kept, the link is "ok" again
    const iad_recovery_policy policy = quick_policy();
    iad_recovery_summary rs;
    CHECK_EQ(iad_axis_recover(ax, 10.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_OK);
    CHECK_EQ(rs.restarted, IAD_TRUE);
    CHECK_EQ(rs.same_unit, IAD_TRUE);
    CHECK_EQ(std::string(rs.cause), std::string("unit_restarted"));
    CHECK(std::strlen(rs.summary) > 0);
    CHECK(has(text(iad_axis_last_result_json), "\"restarted\":true"));
    CHECK_EQ(text(iad_axis_link_state), std::string("ok"));
    events = drain(next, 2.0, [](const std::vector<iad_event>& v) { return any_kind(v, IAD_EVENT_LINK, "recovered"); });
    CHECK(any_kind(events, IAD_EVENT_LINK, "recovered"));
    // bookkeeping entries
    CHECK_EQ(iad_axis_note_target(ax, 12.5), IAD_OK);
    CHECK_EQ(iad_axis_note_target(ax, kNaN), IAD_OK);
    CHECK_EQ(iad_axis_note_target(ax, INFINITY), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_note_control(ax, IAD_TRUE, IAD_UNKNOWN, 0, IAD_UNKNOWN), IAD_OK);
    CHECK_EQ(iad_axis_note_control(ax, 5, IAD_UNKNOWN, 0, IAD_UNKNOWN), IAD_OK);  // (0.8) any value but 0 and 2 is true
    CHECK_EQ(iad_axis_note_control(ax, -1, IAD_FALSE, 0, IAD_UNKNOWN), IAD_OK);   // a Visual Basic True
    CHECK_EQ(iad_axis_note_control(ax, IAD_UNKNOWN, IAD_UNKNOWN, -9600, IAD_UNKNOWN), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_forget(ax), IAD_OK);
    CHECK_EQ(text(iad_axis_session_json), std::string("<IAD_ERR_NOT_READY>"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // no session and no identity read: the recovery cannot confirm the unit
    iad_axis* fresh = sim_axis("gripper", 50.0, false);
    CHECK_EQ(iad_axis_recover(fresh, 1.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_ERR_RECOVERY);
    CHECK(has(last_json(), "\"kind\":\"RecoveryError\""));
    CHECK_EQ(iad_axis_close(fresh), IAD_OK);
}

TEST(capi_ext_backups_sessions_from_them_and_a_replaced_unit) {
    const std::filesystem::path dir = temp_dir("backup");
    const std::string file = (dir / "gripper.json").string();
    iad_axis* g = sim_axis("gripper", 50.0);
    // the JSON is kept even when the buffer was too small: no second read
    std::size_t need = 0;
    CHECK_EQ(iad_axis_backup_json(g, "used", file.c_str(), nullptr, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    REQUIRE(need > 100);
    std::string backup(need, 'x');
    CHECK_EQ(iad_axis_last_result_json(g, &backup[0], backup.size(), &need), IAD_OK);
    backup.resize(std::strlen(backup.c_str()));
    CHECK(has(backup, "ec_driver.backup/1"));
    CHECK(std::filesystem::exists(file));
    CHECK_EQ(iad_axis_backup_json(g, "bogus rows", nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(g), IAD_OK);
    // the gripper's backup on a slider: the recovery refuses the other unit, nothing written
    iad_axis* s = sim_axis("slider", 50.0);
    // backup_axes: several handles into one directory, one file each, the names given or "port:slave"
    {
        iad_axis* g2 = sim_axis("gripper", 50.0);
        iad_axis* both[2] = {s, g2};
        const std::string many = (dir / "many").string();
        const std::string listed = read_text([&](char* b, size_t n, size_t* r) {
            return iad_backup_axes_json(both, 2, "slider, grip:1", many.c_str(), "0,1", b, n, r);
        });
        CHECK(has(listed, "\"slider\":"));
        CHECK(has(listed, "\"grip:1\":"));
        CHECK(!has(listed, "error:"));
        CHECK(std::filesystem::exists(dir / "many" / "slider.json"));
        CHECK(std::filesystem::exists(dir / "many" / "grip_1.json"));
        const std::string named = read_text([&](char* b, size_t n, size_t* r) {
            return iad_backup_axes_json(both + 1, 1, nullptr, many.c_str(), nullptr, b, n, r);
        });
        CHECK(has(named, ":1\":"));
        CHECK(!has(named, "error:"));
        iad_axis* twice[2] = {s, s};
        CHECK_EQ(iad_backup_axes_json(both, 2, "one", many.c_str(), nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_backup_axes_json(twice, 2, nullptr, many.c_str(), nullptr, nullptr, 0, &need),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_backup_axes_json(both, 0, nullptr, many.c_str(), nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_backup_axes_json(both, 2, nullptr, nullptr, nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_backup_axes_json(both, 2, nullptr, many.c_str(), "bogus rows", nullptr, 0, &need),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_axis_close(g2), IAD_OK);
        CHECK_EQ(iad_backup_axes_json(both, 2, nullptr, many.c_str(), nullptr, nullptr, 0, &need),
                 IAD_ERR_INVALID_ARGUMENT);  // a closed handle, nothing read
    }
    CHECK_EQ(iad_axis_session_from_backup(s, "not json", nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_from_backup(s, "{}", nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_from_backup(s, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_from_backup(s, backup.c_str(), "S"), IAD_OK);
    const iad_recovery_policy policy = quick_policy();
    iad_recovery_summary rs;
    CHECK_EQ(iad_axis_recover(s, 2.0, &policy, nullptr, nullptr, nullptr, &rs), IAD_ERR_UNIT_REPLACED);
    const std::string err = last_json();
    CHECK(has(err, "\"kind\":\"UnitReplacedError\""));
    CHECK(has(err, "\"expected\":{"));
    CHECK(has(err, "\"found\":{"));
    CHECK_EQ(iad_axis_session_from_backup_file(s, file.c_str(), "S"), IAD_OK);
    CHECK_EQ(iad_axis_session_from_backup_file(s, (dir / "missing.json").string().c_str(), "S"), IAD_ERR_FILE);
    CHECK_EQ(iad_axis_close(s), IAD_OK);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ============================================================================================ information, line, modes

TEST(capi_ext_unit_info_self_test_line_strategy_and_modes) {
    iad_axis* ax = sim_axis("slider", 50.0);
    auto text = [&](iad_result (*f)(iad_axis*, char*, size_t, size_t*)) {
        return read_text([&](char* b, size_t n, size_t* r) { return f(ax, b, n, r); });
    };
    const std::string info =
        read_text([&](char* b, size_t n, size_t* r) { return iad_axis_read_unit_info_json(ax, "identity", b, n, r); });
    CHECK(has(info, "ec_driver.unit_info/1"));
    CHECK(has(info, "\"identity\":{"));
    std::size_t need = 0;
    CHECK_EQ(iad_axis_read_unit_info_json(ax, "bogus", nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    const std::string reading =
        read_text([&](char* b, size_t n, size_t* r) { return iad_axis_read_monitor_json(ax, b, n, r); });
    CHECK(has(reading, "\"position\":"));
    if (!has(reading, "\"position\":")) std::printf("    monitor reading: %s (%s)\n", reading.c_str(), message().c_str());
    // the self-test: a summary, the report kept, each line queued
    iad_check_summary cs;
    CHECK_EQ(iad_axis_self_test(ax, nullptr, "SIM", &cs), IAD_OK);
    CHECK(cs.items > 5);
    CHECK_EQ(cs.failed, 0);
    CHECK_EQ(cs.passed, IAD_TRUE);
    const std::string report = text(iad_axis_last_result_json);
    CHECK(has(report, "\"tool\":\"iai-check (unofficial)\""));
    CHECK(has(report, "\"port\":\"SIM\""));
    auto next = [&](double t, iad_event* e) { return iad_axis_next_event(ax, t, e); };
    const std::vector<iad_event> lines = drain(next, 1.0);
    REQUIRE(static_cast<int>(lines.size()) == cs.items);
    CHECK_EQ(lines.front().kind, IAD_EVENT_CHECK_ITEM);
    CHECK_EQ(std::string(lines.front().name), std::string("link"));
    CHECK_EQ(std::string(lines.front().cause), std::string("PASS"));
    iad_check_options co;
    iad_check_options_init(&co);
    co.step_mm = -1.0;
    CHECK_EQ(iad_axis_self_test(ax, &co, nullptr, &cs), IAD_ERR_INVALID_ARGUMENT);
    // the line: a direct line here
    CHECK(has(text(iad_axis_line_json), "\"kind\":\"direct\""));
    CHECK_EQ(text(iad_axis_gateway_status_json), std::string("null"));
    CHECK_EQ(text(iad_axis_line_name), std::string(""));
    CHECK_EQ(iad_axis_set_position_unit(ax, 0.01), IAD_OK);
    CHECK_EQ(iad_axis_set_position_unit(ax, 0.5), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_position_unit(ax, kNaN), IAD_ERR_INVALID_ARGUMENT);
    // the command strategy
    int32_t method = -99;
    CHECK_EQ(iad_axis_strategy(ax, &method), IAD_OK);
    CHECK_EQ(method, IAD_METHOD_COIL);
    CHECK_EQ(text(iad_axis_stop_path), std::string("coil"));
    CHECK_EQ(iad_axis_set_strategy(ax, IAD_METHOD_IAI), IAD_OK);
    CHECK_EQ(iad_axis_strategy(ax, &method), IAD_OK);
    CHECK_EQ(method, IAD_METHOD_IAI);
    CHECK_EQ(iad_axis_set_strategy(ax, IAD_METHOD_DEFAULT), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_strategy(ax, 9), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_strategy(ax, -1), IAD_ERR_INVALID_ARGUMENT);  // the 0.7 value of IAD_METHOD_DEFAULT
    CHECK_EQ(iad_axis_set_strategy(ax, IAD_METHOD_COIL), IAD_OK);
    // safety speed and the action mode
    iad_status st;
    CHECK_EQ(iad_axis_set_safety_speed(ax, 1, 1, &st), IAD_OK);
    CHECK_EQ(st.safety_speed, IAD_TRUE);
    CHECK_EQ(iad_axis_set_safety_speed(ax, 0, 1, &st), IAD_OK);
    CHECK_EQ(st.safety_speed, IAD_FALSE);
    CHECK_EQ(iad_axis_set_action_mode(ax, 1, 7, 1, &st), IAD_OK);  // (0.8) any value but 0 and IAD_UNKNOWN is true
    CHECK_EQ(st.safety_speed, IAD_TRUE);
    CHECK_EQ(iad_axis_set_action_mode(ax, 1, -1, 1, &st), IAD_OK);  // a Visual Basic True
    CHECK_EQ(st.safety_speed, IAD_TRUE);
    CHECK_EQ(iad_axis_set_action_mode(ax, 1, IAD_FALSE, 1, &st), IAD_OK);
    CHECK_EQ(st.safety_speed, IAD_FALSE);
    CHECK_EQ(iad_axis_set_action_mode(ax, 1, IAD_UNKNOWN, 1, &st), IAD_OK);
    CHECK_EQ(st.modbus_enabled, IAD_TRUE);
    CHECK_EQ(st.safety_speed, IAD_FALSE);  // kept
    // the coil-free teach is experimental; the table teach writes the row (no opt-in needed)
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    iad_position_write_result pw;
    CHECK_EQ(iad_axis_teach_position(ax, 0, 5, &nv, &pw), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_teach_position(ax, 0, IAD_TEACH_TABLE, &nv, &pw), IAD_OK);
    CHECK_EQ(pw.row, 0);
    // the push speed entry of the limits
    iad_limits lim;
    CHECK_EQ(iad_axis_limits(ax, &lim), IAD_OK);
    CHECK(std::isnan(lim.push_speed) || lim.push_speed > 0.0);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ cell

TEST(capi_ext_cell_extras) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,pcon", 1, &sim, nullptr, nullptr, &cell) == IAD_OK);
    auto cell_text = [&](iad_result (*f)(iad_cell*, char*, size_t, size_t*)) {
        return read_text([&](char* b, size_t n, size_t* r) { return f(cell, b, n, r); });
    };
    iad_cell_result results[4];
    size_t count = 0;
    CHECK_EQ(cell_text(iad_cell_last_result_json), std::string("<IAD_ERR_NOT_READY>"));
    // move_each: each axis with its own options, the pcon with a deceleration of its own
    iad_axis_move moves_each[2];
    std::memset(moves_each, 0, sizeof moves_each);
    moves_each[0].struct_size = moves_each[1].struct_size = sizeof moves_each[0];
    moves_each[0].speed = 50.0;
    moves_each[0].position = 30.0;
    std::strcpy(moves_each[0].axis, "SIM1:1");
    moves_each[1].speed = 50.0;
    moves_each[1].accel_g = 0.3;
    moves_each[1].decel_g = 0.1;
    moves_each[1].position = 60.0;
    std::strcpy(moves_each[1].axis, "SIM2:1");
    CHECK_EQ(iad_cell_move_each(cell, moves_each, 2, 1, kNaN, IAD_TRUE, results, 4, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK_NEAR(results[0].status.position_mm, 30.0, 0.11);
    CHECK_NEAR(results[1].status.position_mm, 60.0, 0.11);
    moves_each[1].position = 1e6;  // one bad value refuses all, nothing sent
    CHECK_EQ(iad_cell_move_each(cell, moves_each, 2, 1, kNaN, IAD_TRUE, results, 4, &count), IAD_ERR_CELL);
    CHECK_EQ(count, size_t(2));
    CHECK_EQ(results[1].code, IAD_ERR_SOFT_LIMIT);
    std::strcpy(moves_each[1].axis, "SIM9:1");
    CHECK_EQ(iad_cell_move_each(cell, moves_each, 2, 1, kNaN, IAD_TRUE, results, 4, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_move_each(cell, moves_each, 0, 1, kNaN, IAD_TRUE, results, 4, &count), IAD_ERR_INVALID_ARGUMENT);
    // per-axis values (Cell::move_all with PerAxisMoveValues): NaN = the options' value; the pcon decelerates at a value
    // of its own, so its move takes the command-profile path (FC 0x17), the slider's the 0x9900 block
    {
        iad_axis* pcon = nullptr;
        REQUIRE(iad_cell_open_axis(cell, "SIM2:1", &pcon) == IAD_OK);
        std::atomic<int> profile_frames{0};
        CHECK_EQ(iad_axis_set_frame_sink(
                     pcon,
                     [](int32_t direction, const uint8_t* bytes, size_t length, void* user) {
                         if (direction == IAD_FRAME_TX && length > 2 && bytes[1] == 0x17)
                             ++*static_cast<std::atomic<int>*>(user);
                     },
                     &profile_frames),
                 IAD_OK);
        const double pos[2] = {20.0, 50.0};
        const double speeds[2] = {40.0, kNaN};
        const double accels[2] = {kNaN, 0.3};
        const double decels[2] = {kNaN, 0.1};
        iad_move_options mv;
        iad_move_options_init(&mv);
        mv.speed = 50.0;
        CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", pos, 2, &mv, speeds, accels, nullptr, decels, results, 4,
                                          &count),
                 IAD_OK);
        CHECK_EQ(count, size_t(2));
        CHECK_NEAR(results[0].status.position_mm, 20.0, 0.11);
        CHECK_NEAR(results[1].status.position_mm, 50.0, 0.11);
        CHECK(profile_frames.load() > 0);
        const double far_off[2] = {20.0, 1e6};  // one bad value refuses all, nothing sent
        CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", far_off, 2, &mv, speeds, accels, nullptr, decels, results,
                                          4, &count),
                 IAD_ERR_CELL);
        CHECK_EQ(count, size_t(2));
        CHECK_EQ(results[1].code, IAD_ERR_SOFT_LIMIT);
        const double bad_band[2] = {0.1, std::numeric_limits<double>::infinity()};
        CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", pos, 2, &mv, nullptr, nullptr, bad_band, nullptr, results,
                                          4, &count),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", pos, 1, &mv, nullptr, nullptr, nullptr, nullptr, results,
                                          4, &count),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM1:1", pos, 2, &mv, nullptr, nullptr, nullptr, nullptr, results,
                                          4, &count),
                 IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(iad_axis_set_frame_sink(pcon, nullptr, nullptr), IAD_OK);
        CHECK_EQ(iad_axis_close(pcon), IAD_OK);
    }
    // pause / resume all, the cancel entry
    CHECK_EQ(iad_cell_pause_all(cell, nullptr, 0, results, 4, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK_EQ(results[0].code, IAD_OK);
    CHECK_EQ(iad_cell_resume_all(cell, nullptr, 0, results, 4, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK_EQ(iad_cell_cancel(cell, nullptr), IAD_OK);
    CHECK_EQ(iad_cell_cancel(cell, "SIM9:1"), IAD_ERR_CELL);  // an unknown name is an error of its own, as in stop_all
    // monitors: every axis's events on the cell handle
    auto next = [&](double t, iad_event* e) { return iad_cell_next_event(cell, t, e); };
    CHECK_EQ(iad_cell_start_monitors(cell, nullptr, nullptr), IAD_OK);
    const double targets[2] = {50.0, 40.0};
    iad_move_options mo;
    iad_move_options_init(&mo);
    mo.speed = 50.0;
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", targets, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &count), IAD_OK);
    std::vector<iad_event> events = drain(next, 10.0, [](const std::vector<iad_event>& v) {
        bool one = false, two = false;
        for (const auto& e : v) {
            one = one || (e.kind == IAD_EVENT_DONE && std::string(e.axis) == "SIM1:1");
            two = two || (e.kind == IAD_EVENT_DONE && std::string(e.axis) == "SIM2:1");
        }
        return one && two;
    });
    CHECK(any_kind(events, IAD_EVENT_STATUS));
    CHECK(any_kind(events, IAD_EVENT_DONE));
    CHECK_EQ(iad_cell_start_monitors(cell, "SIM1:1", nullptr), IAD_OK);  // already fed: not twice
    CHECK_EQ(iad_cell_stop_monitors(cell, nullptr), IAD_OK);
    CHECK(has(cell_text(iad_cell_lines_json), "\"SIM1\":{"));
    CHECK_EQ(cell_text(iad_cell_gateway_status_json), std::string("{}"));
    CHECK_EQ(iad_cell_set_position_unit(cell, "SIM1", 0.01, nullptr), IAD_OK);
    CHECK_EQ(iad_cell_set_position_unit(cell, "SIM1", 0.01, "1"), IAD_OK);
    CHECK_EQ(iad_cell_set_position_unit(cell, "NOPE", 0.01, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_set_position_unit(cell, "SIM1", 0.01, "one"), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(read_text([&](char* b, size_t n, size_t* r) { return iad_cell_monitor_all_json(cell, nullptr, b, n, r); }),
              "\"readings\":{\"SIM1:1\""));
    // recovery on the cell: track, a restarted unit seen by the link check (queued as a fault), recover
    CHECK_EQ(iad_cell_track_all(cell, nullptr, nullptr, nullptr, results, 4, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK(has(cell_text(iad_cell_last_result_json), "\"SIM1:1\":{"));
    iad_link_check checks[2];
    CHECK_EQ(iad_cell_check_links(cell, nullptr, checks, 2, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK(checks[0].ok == IAD_TRUE && checks[1].ok == IAD_TRUE);
    drain(next, 0.2);
    iad_axis* first = nullptr;
    REQUIRE(iad_cell_open_axis(cell, "SIM1:1", &first) == IAD_OK);
    auto line = std::dynamic_pointer_cast<SimLine>(capi::find_axis(first)->axis->bus()->transport_ptr());
    REQUIRE(line != nullptr);
    line->power_cycle();
    CHECK_EQ(iad_cell_check_links(cell, nullptr, checks, 2, &count), IAD_OK);
    CHECK_EQ(std::string(checks[0].state), std::string("unit_restarted"));
    CHECK_EQ(checks[1].ok, IAD_TRUE);
    events = drain(next, 2.0, [](const std::vector<iad_event>& v) { return any_kind(v, IAD_EVENT_FAULT); });
    CHECK(any_kind(events, IAD_EVENT_FAULT, "unit_restarted"));
    const iad_recovery_policy policy = quick_policy();
    CHECK_EQ(iad_cell_recover_all(cell, "SIM1:1", 10.0, &policy, nullptr, results, 4, &count), IAD_OK);
    CHECK_EQ(count, size_t(1));
    CHECK(has(cell_text(iad_cell_last_result_json), "\"reports\":{\"SIM1:1\""));
    CHECK_EQ(iad_cell_clear_fault(cell, "SIM1:1"), IAD_OK);
    CHECK_EQ(iad_cell_monitor_links(cell, 0.05, nullptr), IAD_OK);
    CHECK_EQ(iad_cell_monitor_links(cell, -1.0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    sleep_at_least(0.1);
    CHECK_EQ(iad_cell_stop_link_monitor(cell), IAD_OK);
    CHECK_EQ(iad_cell_stop_link_monitor(cell), IAD_OK);
    // a backup of one axis into a directory, the result kept
    const std::filesystem::path dir = temp_dir("cell");
    const std::string saved = read_text(
        [&](char* b, size_t n, size_t* r) { return iad_cell_backup_json(cell, "SIM2:1", dir.string().c_str(), "used", b, n, r); });
    CHECK(has(saved, "\"SIM2:1\":"));
    CHECK(!has(saved, "error:"));
    CHECK_EQ(cell_text(iad_cell_last_result_json), saved);
    CHECK_EQ(iad_axis_close(first), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    iad_event ev;
    CHECK_EQ(iad_cell_next_event(cell, 0.0, &ev), IAD_ERR_INVALID_ARGUMENT);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ============================================================================================ lean reads, completion, RCON monitor

TEST(capi_ext_lean_status_boot_baud_wait_by_completion) {
    iad_axis* ax = sim_axis("gripper", 20.0);
    iad_status full;
    iad_status lean;
    CHECK_EQ(iad_axis_status(ax, &full), IAD_OK);
    CHECK_EQ(iad_axis_read_lean_status(ax, IAD_INT_UNKNOWN, &lean), IAD_OK);
    CHECK_EQ(lean.raw_words, 8);
    CHECK(std::isnan(lean.current_ma));
    CHECK_NEAR(lean.position_mm, full.position_mm, 0.011);
    CHECK_EQ(lean.servo_on, full.servo_on);
    CHECK_EQ(iad_axis_read_lean_status(ax, 2, &lean), IAD_OK);
    CHECK_EQ(iad_axis_read_lean_status(ax, -1, &lean), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_read_lean_status(ax, IAD_INT_UNKNOWN, nullptr), IAD_ERR_INVALID_ARGUMENT);
    int32_t rate = 0;
    CHECK_EQ(iad_axis_read_boot_baud_rate(ax, &rate), IAD_OK);
    CHECK_EQ(rate, 38400);
    CHECK_EQ(iad_axis_read_boot_baud_rate(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    // wait by a completion rule: a move started without waiting, then each rule
    iad_move_options mv = moves(10.0, false);
    iad_status st;
    CHECK_EQ(iad_axis_move_to(ax, 6.0, &mv, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(6.0, kNaN, 10.0, IAD_COMPLETION_FAST)), &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 6.0, 0.11);
    CHECK_EQ(st.in_position, IAD_TRUE);
    CHECK_EQ(iad_axis_move_to(ax, 4.0, &mv, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(4.0, 0.1, kNaN, IAD_COMPLETION_SAFE)), &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 4.0, 0.11);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(kNaN, kNaN, kNaN, IAD_COMPLETION_DEFAULT)), nullptr), IAD_OK);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(kNaN, kNaN, kNaN, 7)), &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(kNaN, kNaN, -1.0, IAD_COMPLETION_SAFE)), &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(std::numeric_limits<double>::infinity(), kNaN, kNaN,
                                           IAD_COMPLETION_SAFE)), &st),
             IAD_ERR_INVALID_ARGUMENT);
    // no RCON gateway on a direct line
    std::size_t need = 0;
    CHECK_EQ(iad_axis_gateway_monitor_json(ax, nullptr, nullptr, 0, &need), IAD_ERR_NOT_READY);
    CHECK(has(message(), "no RCON gateway"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_axis_read_lean_status(ax, IAD_INT_UNKNOWN, &lean), IAD_ERR_INVALID_ARGUMENT);  // closed
}

TEST(capi_ext_rcon_all_axes_monitor) {
    SimulatedUnitOptions unit = sim_gateway_unit_defaults();
    unit.time_scale = 20.0;
    OpenOptions oo;
    oo.slave = 1;
    oo.bus.min_gap_s = 0.0;
    oo.axis.poll_interval_s = 0.005;
    oo.line.position_unit_mm = 0.01;
    // an axis handle on an RCON line (the C ABI has no simulated gateway: the handle is registered directly)
    auto line = sim_rcon_line({1, 2}, {}, unit);
    iad_axis* ax = capi::register_axis(std::make_shared<Axis>(Axis::open(line, oo)));
    auto monitor = [&](iad_axis* a, const char* slaves) {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_axis_gateway_monitor_json(a, slaves, b, n, r); });
    };
    const std::string both = monitor(ax, nullptr);  // the linked slaves
    CHECK(has(both, "\"1\":{\"axis\":0,"));
    CHECK(has(both, "\"2\":{\"axis\":1,"));
    CHECK(has(both, "\"position_counts\":"));
    const std::string two = monitor(ax, "2");
    CHECK(has(two, "\"2\":{"));
    CHECK(!has(two, "\"1\":{"));
    CHECK_EQ(monitor(ax, "2,2"), two);
    CHECK_EQ(monitor(ax, "17"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(monitor(ax, "one"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    // the same through a cell on that port
    CellOptions co;
    co.bus.min_gap_s = 0.0;
    co.axis.poll_interval_s = 0.005;
    co.line.position_unit_mm = 0.01;
    auto cell_line = sim_rcon_line({1, 2}, {}, unit);
    std::map<std::string, std::shared_ptr<Transport>> lines{{"R", cell_line}};
    iad_cell* cell = capi::register_cell(std::make_shared<Cell>(Cell::open(lines, {1, 2}, co)));
    auto cell_monitor = [&](const char* port, const char* slaves) {
        return read_text(
            [&](char* b, size_t n, size_t* r) { return iad_cell_gateway_monitor_json(cell, port, slaves, b, n, r); });
    };
    const std::string from_cell = cell_monitor("R", nullptr);
    CHECK(has(from_cell, "\"1\":{"));
    CHECK(has(from_cell, "\"2\":{"));
    CHECK(has(cell_monitor("R", "1"), "\"slave\":1"));
    CHECK_EQ(cell_monitor("NOPE", nullptr), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    std::size_t need = 0;
    CHECK_EQ(iad_cell_gateway_monitor_json(cell, nullptr, nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_gateway_monitor_json(cell, "R", nullptr, nullptr, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    // a cell of direct lines has no RCON gateway
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    iad_cell* direct = nullptr;
    REQUIRE(iad_cell_open_sim("slider", 1, &sim, nullptr, nullptr, &direct) == IAD_OK);
    CHECK_EQ(iad_cell_gateway_monitor_json(direct, "SIM1", nullptr, nullptr, 0, &need), IAD_ERR_NOT_READY);
    CHECK_EQ(iad_cell_close(direct), IAD_OK);
}

// ============================================================================================ handles and pointers

TEST(capi_ext_null_handles_and_pointers_are_refused) {
    iad_status st;
    iad_event ev;
    iad_bool b;
    int32_t i32;
    std::size_t need;
    char buf[8];
    iad_move_handle* mh = nullptr;
    CHECK_EQ(iad_axis_push(nullptr, 1, 1, 30, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_change_accel(nullptr, 0.1, ptr_of(change_opts(0, kNaN)), nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_start_move_to(nullptr, 1, nullptr, &mh), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_start_move_to(nullptr, 1, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_move_handle_wait(nullptr, kNaN, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "move handle is NULL"));
    CHECK_EQ(iad_move_handle_poll(nullptr, 0.0, &b, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_move_handle_cancel(nullptr, 0, kNaN, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_start_monitor(nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_next_event(nullptr, 0.0, &ev), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_enable_fast(nullptr, 0, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_track(nullptr, nullptr, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_recover(nullptr, kNaN, nullptr, nullptr, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_link_state(nullptr, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_action_mode(nullptr, 1, IAD_UNKNOWN, 1, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_strategy(nullptr, &i32), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_next_event(nullptr, 0.0, &ev), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_pause_all(nullptr, nullptr, 0, nullptr, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_last_result_json(nullptr, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    const double pos[1] = {1.0};
    CHECK_EQ(iad_cell_move_all(nullptr, "SIM1:1", pos, 1, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 0,
                                      nullptr),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_backup_axes_json(nullptr, 1, nullptr, ".", nullptr, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    iad_axis* none[1] = {nullptr};
    CHECK_EQ(iad_backup_axes_json(none, 1, nullptr, ".", nullptr, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_backup_axes_json(none, 1, nullptr, ".", nullptr, buf, sizeof buf, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_axis* ax = sim_axis("slider", 20.0, false);
    CHECK_EQ(iad_axis_next_event(ax, 0.0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_start_move_to(ax, 1.0, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_strategy(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_link_state(ax, buf, sizeof buf, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_session_from_backup_file(ax, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_monitor_wait_update(ax, 0, 0.0, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_axis_next_event(ax, 0.0, &ev), IAD_ERR_INVALID_ARGUMENT);  // closed
}

// ============================================================================================ caps, home, coordinated

TEST(capi_ext_motion_caps_home_pattern_and_finish_home) {
    iad_axis* ax = sim_axis();
    const std::shared_ptr<Axis> core = capi::find_axis(ax)->axis;
    iad_motion_caps caps;
    std::memset(&caps, 0x55, sizeof caps);
    CHECK_EQ(iad_axis_motion_caps(ax, &caps), IAD_OK);
    CHECK_EQ(caps.speed_cap, core->speed_cap());
    CHECK_EQ(caps.accel_cap, core->accel_cap());
    CHECK_EQ(caps.accel_only_cap, core->accel_only_cap());
    CHECK_EQ(caps.decel_cap, core->decel_cap());
    CHECK_EQ(caps.push_cap, core->push_cap());
    CHECK(caps.speed_cap > 0.0 && caps.accel_cap > 0.0 && caps.push_cap > 0.0);
    const auto min_speed = core->recommended_min_speed();
    CHECK(min_speed.first ? caps.min_speed == *min_speed.first : std::isnan(caps.min_speed));
    CHECK_EQ(std::string(caps.min_speed_source), min_speed.second);
    CHECK_EQ(caps.reserved[0], uint64_t(0));
    iad_bool known = IAD_UNKNOWN;
    uint32_t pattern = 99;
    CHECK_EQ(iad_axis_read_home_pattern(ax, &known, &pattern), IAD_OK);
    const std::optional<std::uint32_t> org = core->read_home_pattern();
    CHECK_EQ(known, org ? IAD_TRUE : IAD_FALSE);
    CHECK_EQ(pattern, org ? *org : 0u);
    iad_status st;
    CHECK_EQ(iad_axis_home(ax, ptr_of(home_opts(IAD_FALSE, kNaN, IAD_METHOD_DEFAULT)), nullptr), IAD_OK);
    const double deadline = now_s() + 20.0;  // the home return ends by itself (a busy PC: a generous deadline)
    sleep_at_least(0.05);
    while (now_s() < deadline && (iad_axis_status(ax, &st) != IAD_OK || !st.homed || !st.idle)) sleep_at_least(0.02);
    CHECK_EQ(iad_axis_finish_home(ax), IAD_OK);
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.homed == IAD_TRUE);
    CHECK_EQ(iad_axis_motion_caps(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_read_home_pattern(ax, nullptr, &pattern), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_motion_caps(nullptr, &caps), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_finish_home(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_ext_move_poll_ends_on_cancel) {
    iad_axis* ax = sim_axis("slider", 1.0);
    iad_move_options slow = moves(20.0, false);
    iad_move_handle* mh = nullptr;
    REQUIRE(iad_axis_start_move_to(ax, 140.0, &slow, &mh) == IAD_OK);
    iad_result polled = IAD_ERR_INTERNAL;
    iad_bool done = IAD_UNKNOWN;
    std::string kind;
    std::thread poller([&] {
        iad_status st;
        polled = iad_move_handle_poll(mh, 60.0, &done, &st);  // one long slice: only the cancel ends it early
        kind = last_json();
    });
    sleep_at_least(0.3);
    const double t0 = now_s();
    CHECK_EQ(iad_axis_cancel(ax), IAD_OK);
    poller.join();
    CHECK(now_s() - t0 < 5.0);
    CHECK_EQ(std::string(iad_result_name(polled)), std::string("IAD_ERR_CANCELLED"));
    CHECK(has(kind, "\"kind\":\"WaitCancelledError\""));
    CHECK_EQ(done, IAD_FALSE);
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.position_mm < 140.0);  // stopped short (a decel stop before the error)
    CHECK_EQ(iad_move_handle_close(mh), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

namespace {

/// Two simulated sliders on two ports (SIM1:1, SIM2:1), both at `start`.
iad_cell* coord_cell(double start, double scale) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = scale;
    sim.position_mm = start;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_cell* cell = nullptr;
    const iad_result r = iad_cell_open_sim("slider,slider", 1, &sim, nullptr, &ao, &cell);
    CHECK_EQ(std::string(iad_result_name(r)), std::string("IAD_OK"));
    return cell;
}

double cell_position(iad_cell* cell, const char* name) {
    iad_axis* ax = nullptr;
    if (iad_cell_open_axis(cell, name, &ax) != IAD_OK) return kNaN;
    iad_status st;
    const double p = iad_axis_status(ax, &st) == IAD_OK ? st.position_mm : kNaN;
    iad_axis_close(ax);
    return p;
}

std::string kept_result(iad_cell* cell) {
    return read_text([&](char* b, size_t n, size_t* r) { return iad_cell_last_result_json(cell, b, n, r); });
}

/// A circle of radius 10 at 20 mm/s on SIM1:1 / SIM2:1 (steps of about 98 ms), for a dry run. Without "min_step_s" the
/// plan measures the line now: a busy PC stretches that past 33 ms per transaction and the planner (rightly) refuses
/// the steps. Tests whose subject is not the measured line pin min_step_s (0.045 s, about the model's shortest step for
/// one axis per port), as test_coord_run.cpp's step_options() and test_coord.py's STEP do.
const char* const kCirclePlanMeasured = R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                                            "path": {"shape": "circle", "center": [50, 40], "radius": 10},
                                            "speed": 20, "accel_g": 0.3})";
const char* const kCirclePlan = R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                                    "path": {"shape": "circle", "center": [50, 40], "radius": 10},
                                    "speed": 20, "accel_g": 0.3, "min_step_s": 0.045})";

/// The library's clock with every sleep ending `late_s` late (a loaded PC), through the C ABI's clock seam; the real
/// clock again when the scope ends, however it ends.
class LateSleeps {
public:
    explicit LateSleeps(double late_s) : late_s_(late_s) {
        CHECK_EQ(iad_clock_use_callbacks(&now, &sleep, this), IAD_OK);
    }
    LateSleeps(const LateSleeps&) = delete;
    LateSleeps& operator=(const LateSleeps&) = delete;
    ~LateSleeps() { iad_clock_use_real(); }

private:
    static double IAD_CALL now(void*) { return steady_now_s(); }
    static void IAD_CALL sleep(double seconds, void* user) {
        const double end = steady_now_s() + seconds + static_cast<const LateSleeps*>(user)->late_s_;
        for (double left = end - steady_now_s(); left > 0.0; left = end - steady_now_s())
            std::this_thread::sleep_for(std::chrono::duration<double>(left));
    }
    double late_s_;
};

}  // namespace

TEST(capi_ext_coordinated_plan_and_sync_run) {
    iad_cell* cell = coord_cell(50.0, 1.0);
    REQUIRE(cell != nullptr);
    auto call = [&](iad_result (*f)(iad_cell*, const char*, char*, size_t, size_t*), const std::string& request) {
        return read_ok([&](char* b, size_t n, size_t* r) { return f(cell, request.c_str(), b, n, r); });
    };
    // a dry run of a circle: streamed, nothing moves
    const std::string circle = call(iad_cell_coord_plan_json, kCirclePlan);
    CHECK(has(circle, "\"mode\":\"stream\""));
    CHECK(has(circle, "\"axes\":[\"x\",\"y\"]"));
    CHECK(has(circle, "\"summary\":"));
    CHECK_NEAR(cell_position(cell, "SIM1:1"), 50.0, 0.01);
    // a synchronized line run, the cell names as the path names; the result is kept on the handle
    const std::string line = call(iad_cell_coord_run_json,
                                  R"({"axes": ["SIM1:1", "SIM2:1"],
                                      "path": {"segments": [{"line": {"SIM1:1": 70, "SIM2:1": 60}}]},
                                      "speed": 40, "accel_g": 0.3, "band": 0.1})");
    CHECK(has(line, "\"completed\":true"));
    CHECK(has(line, "\"mode\":\"sync\""));
    CHECK_NEAR(cell_position(cell, "SIM1:1"), 70.0, 0.11);
    CHECK_NEAR(cell_position(cell, "SIM2:1"), 60.0, 0.11);
    CHECK(has(kept_result(cell), "\"completed\":true"));
    // a too small buffer: the run happens once and its result is kept
    size_t need = 0;
    CHECK_EQ(iad_cell_coord_run_json(cell, R"({"axes": {"x": "SIM1:1"}, "path": {"segments": [{"line": {"x": 60}}]},
                                               "speed": 40, "accel_g": 0.3})",
                                     nullptr, 0, &need),
             IAD_ERR_BUFFER_TOO_SMALL);
    CHECK(need > 1);
    CHECK_NEAR(cell_position(cell, "SIM1:1"), 60.0, 0.11);
    CHECK(has(kept_result(cell), "\"completed\":true"));
    // refusals: nothing sent
    char buf[64];
    const char* too_fast = R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                               "path": {"segments": [{"line": {"x": 10, "y": 20}}]}, "speed": 100000})";
    CHECK_EQ(iad_cell_coord_plan_json(cell, too_fast, buf, sizeof buf, &need), IAD_ERR_SOFT_LIMIT);
    const std::string infeasible = last_json();
    CHECK(has(infeasible, "\"kind\":\"InfeasiblePathError\""));
    CHECK(has(infeasible, "\"reasons\":["));
    CHECK(has(infeasible, "\"max_speed\":"));
    CHECK_EQ(iad_cell_coord_run_json(cell, too_fast, buf, sizeof buf, &need), IAD_ERR_SOFT_LIMIT);
    CHECK_NEAR(cell_position(cell, "SIM1:1"), 60.0, 0.11);
    CHECK_EQ(iad_cell_coord_plan_json(cell, R"({"axes": {"x": "SIM1:1"}, "path": {"segments": [{"line": {"x": 60}}]},
                                                "speed": 10})",
                                      buf, sizeof buf, &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "\"kind\":\"NoMotionError\""));
    CHECK_EQ(iad_cell_coord_plan_json(cell, R"({"path": {"segments": [{"line": {"SIM1:1": 10}}]}, "speed": 10, "sped": 1})",
                                      buf, sizeof buf, &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "unknown key 'sped'"));
    CHECK_EQ(iad_cell_coord_plan_json(cell, R"({"path": {"segments": [{"spiral": {}}]}, "speed": 10})", buf, sizeof buf,
                                      &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_coord_plan_json(cell, R"({"axes": {"x": "SIM9:1"}, "path": {"segments": [{"line": {"x": 1}}]},
                                                "speed": 10})",
                                      buf, sizeof buf, &need),
             IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_coord_plan_json(cell, "not json", buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_coord_plan_json(cell, nullptr, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_coord_run_json(nullptr, "{}", buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi_ext_coordinated_plan_on_a_loaded_pc) {
    // every sleep of the library ends 30 ms late, as on a loaded PC: the line measured now is slow. The pinned circle
    // is planned all the same; the measured one is refused with its reason, never IAD_OK with a part of a plan
    iad_cell* cell = coord_cell(50.0, 1.0);
    REQUIRE(cell != nullptr);
    {
        const LateSleeps loaded(0.03);
        const std::string pinned =
            read_ok([&](char* b, size_t n, size_t* r) { return iad_cell_coord_plan_json(cell, kCirclePlan, b, n, r); });
        CHECK(has(pinned, "\"mode\":\"stream\""));
        CHECK(has(pinned, "\"axes\":[\"x\",\"y\"]"));
        CHECK(has(pinned, "\"summary\":"));
        bool read = true;
        const std::string measured = read_text(
            [&](char* b, size_t n, size_t* r) { return iad_cell_coord_plan_json(cell, kCirclePlanMeasured, b, n, r); },
            &read);
        const std::string refused = last_json();
        CHECK(!read);  // no plan text at all
        CHECK_EQ(measured, std::string("<IAD_ERR_SOFT_LIMIT>"));
        CHECK(has(refused, "\"kind\":\"InfeasiblePathError\""));
        CHECK(has(refused, "\"reasons\":["));
    }
    CHECK_NEAR(cell_position(cell, "SIM1:1"), 50.0, 0.01);  // dry runs: nothing moved
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi_ext_coordinated_run_stopped_from_outside) {
    iad_cell* cell = coord_cell(20.0, 1.0);
    REQUIRE(cell != nullptr);
    iad_result ran = IAD_ERR_INTERNAL;
    std::string error;
    std::thread runner([&] {
        char buf[64];
        size_t need = 0;
        // 90 mm at 10 mm/s: about 9 s unless stopped
        ran = iad_cell_coord_run_json(cell, R"({"axes": {"x": "SIM1:1", "y": "SIM2:1"},
                                                "path": {"segments": [{"line": {"x": 110, "y": 110}}]},
                                                "speed": 10, "accel_g": 0.3})",
                                      buf, sizeof buf, &need);
        error = last_json();
    });
    // stop once the run is under way (the axes moving), however busy the PC is
    const double deadline = now_s() + 20.0;
    while (now_s() < deadline && cell_position(cell, "SIM1:1") < 21.0) sleep_at_least(0.02);
    CHECK(cell_position(cell, "SIM1:1") >= 21.0);
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_FALSE, results, 2, &count), IAD_OK);
    runner.join();
    CHECK_EQ(std::string(iad_result_name(ran)), std::string("IAD_ERR_DRIVER"));
    CHECK(has(error, "\"kind\":\"CoordinatedMotionError\""));
    CHECK(has(error, "\"result\":{"));
    CHECK(has(error, "\"stop\":{"));
    CHECK(has(error, "\"cause\":"));
    CHECK(has(kept_result(cell), "\"completed\":false"));
    CHECK(cell_position(cell, "SIM1:1") < 109.0);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}
