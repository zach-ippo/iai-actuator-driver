// C ABI (include/iai_actuator_driver.h, src/c_api*.cpp) on the simulator: every exported function, the error codes and
// messages, the last error (thread-local, JSON), NULL / closed / wrong-kind handles, closing twice, NULL pointers, the
// buffer-too-small rule with its required size, the group commands with too small results arrays, and the mapping of
// every C++ exception class. Simulator only: no COM port is opened (the argument checks of iad_axis_open /
// iad_cell_open refuse before any port is touched, and those tests use names that are no port).
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/version.hpp"
#include "capi_options.hpp"
#include "harness.hpp"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string message() { return iad_last_error_message(); }

/// The last error as JSON (two calls: the size, then the text).
std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

/// A JSON / text output read the way a binding does: size query, then the text. Text that grew between two calls
/// (live JSON) is read again at the size the failed call reported, until it fits (bounded). "<IAD_...>" when it failed.
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

iad_axis* open_sim(const char* profile = "slider", double scale = 20.0, int slave = 1) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = scale;
    iad_open_options open;
    iad_open_options_init(&open);
    open.slave = slave;
    iad_axis* ax = nullptr;
    const iad_result r = iad_axis_open_sim(profile, &sim, &open, nullptr, &ax);
    CHECK_EQ(std::string(iad_result_name(r)), std::string("IAD_OK"));
    REQUIRE(ax != nullptr);
    return ax;
}

iad_move_options moves(double speed, bool wait = true) {
    iad_move_options o;
    iad_move_options_init(&o);
    o.speed = speed;
    o.wait = wait ? IAD_TRUE : IAD_FALSE;
    return o;
}

/// Every function that takes an axis handle first, called with `ax` and valid other arguments.
std::vector<std::pair<const char*, std::function<iad_result(iad_axis*)>>> axis_calls() {
    static iad_identity id;
    static iad_axis_options ao;
    static iad_status st;
    static iad_limits lim;
    static iad_position_entry entry;
    static iad_position_write_result pw;
    static iad_parameter par;
    static iad_param_write_result pr;
    static iad_baud_result br;
    static iad_reset_result rr;
    static char buf[64];
    static std::size_t need;
    static int32_t i32;
    static int64_t i64;
    static iad_bool b;
    iad_axis_options_init(&ao);
    iad_position_entry_init(&entry);
    return {
        {"connect", [](iad_axis* a) { return iad_axis_connect(a, 1, &id); }},
        {"identify", [](iad_axis* a) { return iad_axis_identify(a, &id); }},
        {"identity_json", [](iad_axis* a) { return iad_axis_identity_json(a, buf, sizeof buf, &need); }},
        {"get_options", [](iad_axis* a) { return iad_axis_get_options(a, &ao); }},
        {"set_options", [](iad_axis* a) { return iad_axis_set_options(a, &ao); }},
        {"baud_rate", [](iad_axis* a) { return iad_axis_baud_rate(a, &i32, &b); }},
        {"modbus_control", [](iad_axis* a) { return iad_axis_modbus_control(a, 1); }},
        {"status", [](iad_axis* a) { return iad_axis_status(a, &st); }},
        {"limits", [](iad_axis* a) { return iad_axis_limits(a, &lim); }},
        {"limits_json", [](iad_axis* a) { return iad_axis_limits_json(a, buf, sizeof buf, &need); }},
        {"alarm_history_json", [](iad_axis* a) { return iad_axis_read_alarm_history_json(a, 0, buf, sizeof buf, &need); }},
        {"max_positions", [](iad_axis* a) { return iad_axis_max_positions(a, &i32); }},
        {"read_position_table", [](iad_axis* a) { return iad_axis_read_position_table(a, 0, &entry); }},
        {"servo", [](iad_axis* a) { return iad_axis_servo(a, 1, ptr_of(servo_opts(1, kNaN)), &st); }},
        {"home", [](iad_axis* a) { return iad_axis_home(a, ptr_of(home_opts(1, kNaN, IAD_METHOD_DEFAULT)), &st); }},
        {"reset_alarm", [](iad_axis* a) { return iad_axis_reset_alarm(a, IAD_METHOD_DEFAULT, &st); }},
        {"check_move", [](iad_axis* a) { return iad_axis_check_move(a, 10.0, nullptr, &st); }},
        {"move_to", [](iad_axis* a) { return iad_axis_move_to(a, 10.0, nullptr, &st); }},
        {"move_by", [](iad_axis* a) { return iad_axis_move_by(a, 1.0, nullptr, &st); }},
        {"jog", [](iad_axis* a) { return iad_axis_jog(a, 1.0, nullptr, &st); }},
        {"move_to_position", [](iad_axis* a) { return iad_axis_move_to_position(a, 0, nullptr, &st); }},
        {"change_speed", [](iad_axis* a) { return iad_axis_change_speed(a, 10.0, ptr_of(change_opts(0, kNaN)), &st); }},
        {"wait_idle", [](iad_axis* a) { return iad_axis_wait_idle(a, ptr_of(wait_idle_opts(kNaN, kNaN, kNaN)), &st); }},
        {"stop", [](iad_axis* a) { return iad_axis_stop(a, ptr_of(stop_opts(0, kNaN)), &st); }},
        {"pause", [](iad_axis* a) { return iad_axis_pause(a); }},
        {"resume", [](iad_axis* a) { return iad_axis_resume(a); }},
        {"cancel_paused_move", [](iad_axis* a) { return iad_axis_cancel_paused_move(a, ptr_of(cancel_paused_opts(kNaN)), &st); }},
        {"write_position_table", [](iad_axis* a) { return iad_axis_write_position_table(a, 0, &entry, nullptr, &pw); }},
        {"teach_position", [](iad_axis* a) { return iad_axis_teach_position(a, 0, IAD_TEACH_TABLE, nullptr, &pw); }},
        {"parameters_json", [](iad_axis* a) { return iad_axis_parameters_json(a, buf, sizeof buf, &need); }},
        {"read_parameter", [](iad_axis* a) { return iad_axis_read_parameter(a, "2", &par); }},
        {"read_parameter_json", [](iad_axis* a) { return iad_axis_read_parameter_json(a, "2", buf, sizeof buf, &need); }},
        {"write_parameter", [](iad_axis* a) { return iad_axis_write_parameter(a, "2", "0.2", nullptr, &pr); }},
        {"write_parameter_number", [](iad_axis* a) { return iad_axis_write_parameter_number(a, "2", 0.2, nullptr, &pr); }},
        {"nonvolatile_writes", [](iad_axis* a) { return iad_axis_nonvolatile_writes(a, &i64, &i32, &b); }},
        {"set_baud_rate", [](iad_axis* a) { return iad_axis_set_baud_rate(a, 115200, nullptr, &br); }},
        {"software_reset", [](iad_axis* a) { return iad_axis_software_reset(a, nullptr, &rr); }},
        {"last_result_json", [](iad_axis* a) { return iad_axis_last_result_json(a, buf, sizeof buf, &need); }},
    };
}

/// Every function that takes a cell handle first (except close), called with `cell` and valid other arguments.
std::vector<std::pair<const char*, std::function<iad_result(iad_cell*)>>> cell_calls() {
    static iad_cell_result results[4];
    static std::size_t n;
    static char buf[64];
    static iad_axis* ax;
    static const double positions[1] = {10.0};
    return {
        {"axis_count", [](iad_cell* c) { return iad_cell_axis_count(c, &n); }},
        {"axis_name", [](iad_cell* c) { return iad_cell_axis_name(c, 0, buf, sizeof buf, &n); }},
        {"axis", [](iad_cell* c) { return iad_cell_open_axis(c, "SIM1:1", &ax); }},
        {"info_json", [](iad_cell* c) { return iad_cell_info_json(c, buf, sizeof buf, &n); }},
        {"clear_fault", [](iad_cell* c) { return iad_cell_clear_fault(c, "SIM1:1"); }},
        {"status_all", [](iad_cell* c) { return iad_cell_status_all(c, nullptr, kNaN, results, 4, &n); }},
        {"servo_all", [](iad_cell* c) { return iad_cell_servo_all(c, 1, nullptr, kNaN, results, 4, &n); }},
        {"home_all", [](iad_cell* c) { return iad_cell_home_all(c, nullptr, kNaN, IAD_METHOD_DEFAULT, IAD_TRUE, results, 4, &n); }},
        {"move_all", [](iad_cell* c) { return iad_cell_move_all(c, "SIM1:1", positions, 1, nullptr, nullptr, nullptr, nullptr, nullptr, results, 4, &n); }},
        {"wait_all", [](iad_cell* c) { return iad_cell_wait_all(c, nullptr, kNaN, IAD_TRUE, results, 4, &n); }},
        {"stop_all", [](iad_cell* c) { return iad_cell_stop_all(c, nullptr, kNaN, IAD_FALSE, results, 4, &n); }},
    };
}

}  // namespace

// ============================================================================================ library

TEST(capi_versions_and_result_names) {
    CHECK_EQ(iad_abi_version(), (uint32_t(IAD_ABI_VERSION_MAJOR) << 16) | uint32_t(IAD_ABI_VERSION_MINOR));
    // the macro, the loader rule (the same major, a library minor at least the header's) and the testing header's own
    // version
    CHECK_EQ(iad_abi_version(), uint32_t(IAD_ABI_VERSION));
    CHECK_EQ(IAD_ABI_VERSION_MAJOR, 1);
    CHECK(IAD_ABI_COMPATIBLE(iad_abi_version()));
    CHECK(IAD_ABI_COMPATIBLE(iad_abi_version() + 1u));        // a higher minor only adds
    CHECK(!IAD_ABI_COMPATIBLE(iad_abi_version() - 1u));       // 0.65535: another major
    CHECK(!IAD_ABI_COMPATIBLE(uint32_t(0x00000008u)));        // 0.8, the same layout under its development label
    CHECK(!IAD_ABI_COMPATIBLE(iad_abi_version() + 0x10000u)); // the next major
    CHECK_EQ(iad_testing_abi_version(), uint32_t(IAD_TESTING_ABI_VERSION));
    CHECK_EQ(std::string(iad_library_version()), std::string(iai_actuator_driver::kVersion));
    std::vector<std::string> names;
    for (int code = 0; code >= IAD_ERR_CANCELLED; --code) {  // 0.2: the codes go on to -25
        const std::string name = iad_result_name(code);
        CHECK(name.rfind("IAD_", 0) == 0);
        CHECK(name != "IAD_ERR_UNKNOWN");
        for (const auto& n : names) CHECK(n != name);
        names.push_back(name);
    }
    CHECK_EQ(names.size(), std::size_t(26));
    CHECK_EQ(std::string(iad_result_name(IAD_OK)), std::string("IAD_OK"));
    CHECK_EQ(std::string(iad_result_name(IAD_ERR_BUFFER_TOO_SMALL)), std::string("IAD_ERR_BUFFER_TOO_SMALL"));
    CHECK_EQ(std::string(iad_result_name(-26)), std::string("IAD_ERR_UNKNOWN"));
    CHECK_EQ(std::string(iad_result_name(5)), std::string("IAD_ERR_UNKNOWN"));
    CHECK(iad_last_error_message() != nullptr);
}

TEST(capi_option_defaults_follow_the_core) {
    iad_axis_options ao;
    CHECK_EQ(iad_axis_options_init(&ao), IAD_OK);
    CHECK(std::isnan(ao.stroke_min) && std::isnan(ao.stroke_max) && std::isnan(ao.max_speed_mm_s));
    CHECK(std::isnan(ao.max_accel_g) && std::isnan(ao.max_push_pct));
    CHECK_NEAR(ao.poll_interval_s, 0.02, 1e-12);
    CHECK_NEAR(ao.settle_s, 0.1, 1e-12);
    CHECK_EQ(ao.method, IAD_METHOD_COIL);
    CHECK_EQ(ao.auto_modbus_control, IAD_TRUE);
    iad_open_options oo;
    CHECK_EQ(iad_open_options_init(&oo), IAD_OK);
    CHECK_NEAR(oo.read_timeout_s, 0.2, 1e-12);
    CHECK_NEAR(oo.min_gap_s, 0.005, 1e-12);
    CHECK_EQ(oo.slave, 1);
    CHECK_EQ(oo.baud_rate, 0);
    CHECK_EQ(oo.retries, 3);
    iad_sim_options so;
    CHECK_EQ(iad_sim_options_init(&so), IAD_OK);
    CHECK_NEAR(so.time_scale, 1.0, 1e-12);
    CHECK(std::isnan(so.position_mm) && std::isnan(so.stroke_mm) && std::isnan(so.max_speed_mm_s));
    CHECK_EQ(so.baud_rate, 0);
    CHECK_EQ(so.pmsl, IAD_TRUE);
    iad_move_options mo;
    CHECK_EQ(iad_move_options_init(&mo), IAD_OK);
    CHECK_NEAR(mo.speed, 50.0, 1e-12);
    CHECK_NEAR(mo.accel_g, 0.1, 1e-12);
    CHECK_NEAR(mo.band, 0.1, 1e-12);
    CHECK(std::isnan(mo.timeout_s));
    CHECK_EQ(mo.wait, IAD_TRUE);
    CHECK_EQ(iad_jog_options_init(&mo), IAD_OK);
    CHECK_NEAR(mo.speed, 10.0, 1e-12);
    iad_position_move_options pm;
    CHECK_EQ(iad_position_move_options_init(&pm), IAD_OK);
    CHECK(std::isnan(pm.timeout_s) && std::isnan(pm.speed_pct));
    CHECK_EQ(pm.wait, IAD_TRUE);
    CHECK_EQ(pm.allow_above_caps, IAD_FALSE);
    iad_nv_write_options nv;
    std::memset(&nv, 0xFF, sizeof nv);
    CHECK_EQ(iad_nv_write_options_init(&nv), IAD_OK);
    CHECK(nv.allow_nonvolatile == 0 && nv.force == 0 && nv.allow_unsafe == 0 && nv.reserved[0] == 0);
    iad_param_write_options pw;
    CHECK_EQ(iad_param_write_options_init(&pw), IAD_OK);
    CHECK(pw.commit == 1 && pw.allow_dangerous == 0 && pw.allow_unlisted == 0 && pw.allow_home_params == 0 &&
          pw.force == 0);
    iad_baud_options bo;
    CHECK_EQ(iad_baud_options_init(&bo), IAD_OK);
    CHECK_NEAR(bo.settle_s, 0.2, 1e-12);
    CHECK_EQ(bo.method, IAD_BAUD_DIRECT);
    CHECK_EQ(bo.allow_unverified_code, IAD_FALSE);
    iad_reset_options ro;
    CHECK_EQ(iad_reset_options_init(&ro), IAD_OK);
    CHECK_NEAR(ro.timeout_s, 10.0, 1e-12);
    CHECK_NEAR(ro.reset_wait_s, 1.0, 1e-12);
    iad_cell_options co;
    CHECK_EQ(iad_cell_options_init(&co), IAD_OK);
    CHECK_NEAR(co.scan_timeout_s, 0.05, 1e-12);
    CHECK_EQ(co.max_axes, 0);  // no limit
    CHECK_EQ(co.scan_first, 1);
    CHECK_EQ(co.scan_last, 16);
    CHECK_EQ(co.retries, 3);
    CHECK(co.identify == 1 && co.stop_on_fault == 1 && co.baud_rate == 0);
    iad_position_entry pe;
    CHECK_EQ(iad_position_entry_init(&pe), IAD_OK);
    CHECK_NEAR(pe.band_mm, 0.10, 1e-12);
    CHECK_NEAR(pe.speed_mm_s, 50.0, 1e-12);
    CHECK_NEAR(pe.accel_g, 0.30, 1e-12);
    CHECK_NEAR(pe.decel_g, 0.30, 1e-12);
    CHECK_EQ(pe.checksum, IAD_INT_UNKNOWN);
    CHECK_EQ(pe.checksum_ok, IAD_UNKNOWN);
    // NULL: IAD_ERR_INVALID_ARGUMENT, with a message
    CHECK_EQ(iad_axis_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "options must not be NULL"));
    CHECK_EQ(iad_open_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_sim_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_move_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_jog_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_position_move_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_nv_write_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_param_write_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_baud_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_reset_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_options_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_position_entry_init(nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "entry must not be NULL"));
}

TEST(capi_last_error_is_per_thread_and_cleared_by_success) {
    iad_status st;
    CHECK_EQ(iad_axis_status(nullptr, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("axis handle is NULL"));
    const std::string json = last_json();
    CHECK(has(json, "\"code\":-10"));
    CHECK(has(json, "\"name\":\"IAD_ERR_INVALID_ARGUMENT\""));
    CHECK(has(json, "\"kind\":\"std::invalid_argument\""));
    CHECK(has(json, "\"message\":\"axis handle is NULL\""));
    CHECK(has(json, "\"notes\":[]"));
    CHECK(has(json, "\"stop_attempted\":false"));
    // reading the last error never changes it, not even when that read fails
    char small[4];
    std::size_t need = 0;
    CHECK_EQ(iad_last_error_json(small, sizeof small, &need), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(need, json.size() + 1);
    CHECK_EQ(small[0], '\0');
    CHECK_EQ(iad_last_error_json(small, sizeof small, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_last_error_json(nullptr, 8, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("axis handle is NULL"));
    CHECK_EQ(last_json(), json);
    // another thread has its own (empty) last error
    std::string other_message = "unset";
    std::string other_json;
    std::thread t([&] {
        other_message = iad_last_error_message();
        other_json = last_json();
    });
    t.join();
    CHECK_EQ(other_message, std::string(""));
    CHECK_EQ(other_json, std::string("{\"code\":0}"));
    CHECK_EQ(message(), std::string("axis handle is NULL"));
    // a call that succeeds clears it
    iad_move_options mo;
    CHECK_EQ(iad_move_options_init(&mo), IAD_OK);
    CHECK_EQ(message(), std::string(""));
    CHECK_EQ(last_json(), std::string("{\"code\":0}"));
}

TEST(capi_list_ports_json_opens_nothing) {
    const std::string ports = read_text([](char* b, std::size_t n, std::size_t* r) { return iad_list_ports_json(b, n, r); });
    CHECK(!ports.empty() && ports[0] == '[');
}

// ============================================================================================ handles

TEST(capi_null_closed_and_wrong_kind_handles_are_refused) {
    CHECK_EQ(iad_axis_close(nullptr), IAD_OK);
    CHECK_EQ(iad_cell_close(nullptr), IAD_OK);
    const auto calls = axis_calls();
    CHECK_EQ(calls.size(), std::size_t(38));
    for (const auto& c : calls) {
        const iad_result r = c.second(nullptr);
        if (r != IAD_ERR_INVALID_ARGUMENT) std::printf("    NULL handle: iad_axis_%s gave %s\n", c.first, iad_result_name(r));
        CHECK_EQ(r, IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(message(), std::string("axis handle is NULL"));
    }
    for (const auto& c : cell_calls()) {
        CHECK_EQ(c.second(nullptr), IAD_ERR_INVALID_ARGUMENT);
        CHECK_EQ(message(), std::string("cell handle is NULL"));
    }
    // closed: refused; closing again is safe; handles are never reused
    iad_axis* a = open_sim();
    CHECK_EQ(iad_axis_close(a), IAD_OK);
    CHECK_EQ(iad_axis_close(a), IAD_OK);
    for (const auto& c : calls) {
        CHECK_EQ(c.second(a), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(message(), "invalid axis handle"));
    }
    iad_axis* b = open_sim();
    CHECK(b != a);
    // the wrong kind: an axis passed as a cell and the reverse
    iad_cell* as_cell = reinterpret_cast<iad_cell*>(b);
    for (const auto& c : cell_calls()) {
        CHECK_EQ(c.second(as_cell), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(message(), "an axis handle was passed as a cell handle"));
    }
    CHECK_EQ(iad_cell_close(as_cell), IAD_ERR_INVALID_ARGUMENT);
    iad_status st;
    CHECK_EQ(iad_axis_status(b, &st), IAD_OK);  // still open
    CHECK_EQ(iad_axis_close(b), IAD_OK);
    // a token that was never issued
    iad_axis* unknown = reinterpret_cast<iad_axis*>(static_cast<std::uintptr_t>(0x7FFFFFF0));
    CHECK_EQ(iad_axis_status(unknown, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(unknown), IAD_OK);
}

TEST(capi_null_output_pointers_are_refused_before_any_io) {
    iad_axis* ax = open_sim("slider");
    CHECK_EQ(iad_axis_status(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("out must not be NULL"));
    CHECK_EQ(iad_axis_identify(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_limits(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_get_options(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_baud_rate(ax, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_max_positions(ax, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_read_position_table(ax, 0, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_parameter par;
    CHECK_EQ(iad_axis_read_parameter(ax, nullptr, &par), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("key must not be NULL"));
    CHECK_EQ(iad_axis_read_parameter(ax, "2", nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_write_parameter(ax, "2", nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("value must not be NULL"));
    CHECK_EQ(iad_axis_write_parameter(ax, nullptr, "1", nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    nv.allow_nonvolatile = IAD_TRUE;
    CHECK_EQ(iad_axis_write_position_table(ax, 0, nullptr, &nv, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("entry must not be NULL"));
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    int64_t total = -1;
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, nullptr, nullptr), IAD_OK);  // the others may be NULL
    CHECK_EQ(total, 0);
    char buf[16];
    std::size_t need = 0;
    CHECK_EQ(iad_axis_limits_json(ax, buf, sizeof buf, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("required must not be NULL"));
    CHECK_EQ(iad_axis_limits_json(ax, nullptr, 8, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "buffer is NULL"));
    // values that are not numbers are refused before anything is sent
    iad_status st;
    CHECK_EQ(iad_axis_move_to(ax, kNaN, nullptr, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("position must be a finite number"));
    CHECK_EQ(iad_axis_move_by(ax, std::numeric_limits<double>::infinity(), nullptr, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_home(ax, ptr_of(home_opts(1, kNaN, 7)), &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "method must be"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_json_outputs_report_the_required_size) {
    iad_axis* ax = open_sim("slider");
    std::size_t need = 0;
    CHECK_EQ(iad_axis_limits_json(ax, nullptr, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK(need > 2);
    CHECK(has(message(), "bytes needed"));
    CHECK(has(last_json(), "\"kind\":\"BufferTooSmall\""));
    std::vector<char> buf(need - 1, 'x');
    std::size_t need2 = 0;
    CHECK_EQ(iad_axis_limits_json(ax, buf.data(), buf.size(), &need2), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(need2, need);
    CHECK_EQ(buf[0], '\0');
    buf.assign(need, 'x');
    CHECK_EQ(iad_axis_limits_json(ax, buf.data(), buf.size(), &need2), IAD_OK);
    CHECK_EQ(need2, need);
    CHECK_EQ(std::strlen(buf.data()), need - 1);
    CHECK_EQ(buf[0], '{');
    CHECK_EQ(message(), std::string(""));
    // a larger buffer: the same text, *required unchanged
    std::vector<char> big(need + 100, 'x');
    CHECK_EQ(iad_axis_limits_json(ax, big.data(), big.size(), &need2), IAD_OK);
    CHECK_EQ(need2, need);
    CHECK_EQ(std::string(big.data()), std::string(buf.data()));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ axis

TEST(capi_axis_session_and_reading_on_the_simulator) {
    iad_axis* ax = open_sim("gripper");
    char buf[32];
    std::size_t need = 0;
    CHECK_EQ(iad_axis_identity_json(ax, buf, sizeof buf, &need), IAD_ERR_NOT_READY);  // before connect
    CHECK(has(message(), "iad_axis_connect"));
    iad_identity id;
    CHECK_EQ(iad_axis_connect(ax, 1, &id), IAD_OK);
    CHECK_EQ(std::string(id.product_family), std::string("EC"));
    CHECK_EQ(id.slave, 1);
    CHECK_EQ(id.system_code, 0xEC);
    CHECK_EQ(id.unit_code, 0x0A);
    CHECK_EQ(std::string(id.units), std::string("mm"));
    CHECK_EQ(std::string(id.kind), std::string("gripper"));
    CHECK_EQ(std::string(id.firmware), std::string("0002"));
    CHECK(std::strlen(id.model) > 0);
    CHECK_EQ(id.rotary, IAD_FALSE);
    CHECK(id.max_positions >= 2);
    CHECK(std::isfinite(id.soft_max_mm) && id.soft_max_mm > 9.0);
    CHECK_EQ(iad_axis_connect(ax, 0, nullptr), IAD_OK);  // out may be NULL
    iad_identity again;
    CHECK_EQ(iad_axis_identify(ax, &again), IAD_OK);
    CHECK_EQ(std::string(again.model), std::string(id.model));
    const std::string info =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_identity_json(ax, b, n, r); });
    CHECK(has(info, "\"slave\":1"));
    CHECK(has(info, "\"system_code\":236"));
    // status, limits, rows, alarm history
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.raw_words >= 8);
    CHECK_EQ(st.modbus_enabled, IAD_TRUE);
    CHECK(st.alarm_level == IAD_ALARM_LEVEL_NONE || st.alarm_code != 0);
    iad_limits lim;
    CHECK_EQ(iad_axis_limits(ax, &lim), IAD_OK);
    CHECK_EQ(std::string(lim.units), std::string("mm"));
    CHECK_EQ(std::string(lim.kind), std::string("gripper"));
    CHECK_NEAR(lim.position.max, 9.5, 0.01);
    CHECK_NEAR(lim.speed.max, 53.0, 0.01);
    CHECK_EQ(std::string(lim.speed.unit), std::string("mm/s"));
    CHECK(lim.stored_rows.count >= 2);
    CHECK_EQ(lim.position.count, IAD_INT_UNKNOWN);
    const std::string limits =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_limits_json(ax, b, n, r); });
    CHECK(has(limits, "\"position\""));
    CHECK(has(limits, "\"source\""));
    const std::string history = read_text(
        [ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_read_alarm_history_json(ax, 0, b, n, r); });
    CHECK(!history.empty() && history[0] == '[');
    const std::string all_slots = read_text(
        [ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_read_alarm_history_json(ax, 1, b, n, r); });
    CHECK(all_slots.size() >= history.size());
    int32_t rows = 0;
    CHECK_EQ(iad_axis_max_positions(ax, &rows), IAD_OK);
    CHECK_EQ(rows, id.max_positions);
    iad_position_entry row;
    CHECK_EQ(iad_axis_read_position_table(ax, 0, &row), IAD_OK);
    CHECK(row.checksum != IAD_INT_UNKNOWN);
    CHECK(row.checksum_ok != IAD_UNKNOWN);
    CHECK(iad_axis_read_position_table(ax, 100000, &row) != IAD_OK);
    // the port's rate: none on an in-memory unit
    int32_t rate = -1;
    iad_bool detected = -1;
    CHECK_EQ(iad_axis_baud_rate(ax, &rate, &detected), IAD_OK);
    CHECK_EQ(rate, 0);
    CHECK_EQ(detected, IAD_FALSE);
    // options round trip
    iad_axis_options ao;
    CHECK_EQ(iad_axis_get_options(ax, &ao), IAD_OK);
    CHECK(std::isnan(ao.max_speed_mm_s));
    ao.max_speed_mm_s = 20.0;
    ao.stroke_min = 1.0;
    ao.stroke_max = 8.0;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    iad_axis_options back;
    CHECK_EQ(iad_axis_get_options(ax, &back), IAD_OK);
    CHECK_NEAR(back.max_speed_mm_s, 20.0, 1e-12);
    CHECK_NEAR(back.stroke_min, 1.0, 1e-12);
    CHECK_NEAR(back.stroke_max, 8.0, 1e-12);
    CHECK_EQ(iad_axis_limits(ax, &lim), IAD_OK);
    CHECK_NEAR(lim.speed.max, 20.0, 1e-9);
    CHECK_NEAR(lim.speed.configured, 20.0, 1e-9);
    back.stroke_max = kNaN;  // both or neither
    CHECK_EQ(iad_axis_set_options(ax, &back), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "both or neither"));
    back.stroke_max = 0.5;  // below min
    CHECK_EQ(iad_axis_set_options(ax, &back), IAD_ERR_INVALID_ARGUMENT);
    back.stroke_max = 8.0;
    back.method = 5;
    CHECK_EQ(iad_axis_set_options(ax, &back), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_options(ax, nullptr), IAD_OK);  // NULL = the defaults
    CHECK_EQ(iad_axis_get_options(ax, &back), IAD_OK);
    CHECK(std::isnan(back.max_speed_mm_s) && std::isnan(back.stroke_min));
    // Modbus control
    CHECK_EQ(iad_axis_modbus_control(ax, 0), IAD_OK);
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(st.modbus_enabled, IAD_FALSE);
    CHECK_EQ(iad_axis_modbus_control(ax, 1), IAD_OK);
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(st.modbus_enabled, IAD_TRUE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_axis_open_sim_checks_its_arguments) {
    iad_axis* ax = reinterpret_cast<iad_axis*>(static_cast<std::uintptr_t>(1234));
    CHECK_EQ(iad_axis_open_sim("nope", nullptr, nullptr, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK(ax == nullptr);  // *out cleared on failure
    CHECK(has(message(), "unknown simulator profile 'nope'"));
    CHECK_EQ(iad_axis_open_sim(nullptr, nullptr, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_open_options oo;
    iad_open_options_init(&oo);
    oo.slave = 0;
    CHECK_EQ(iad_axis_open_sim("rod", nullptr, &oo, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "slave address must be 1..247"));
    iad_sim_options so;
    iad_sim_options_init(&so);
    so.time_scale = -1.0;
    CHECK_EQ(iad_axis_open_sim("rod", &so, nullptr, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    // NULL profile and options: a slider with the defaults; another slave address; every profile
    CHECK_EQ(iad_axis_open_sim(nullptr, nullptr, nullptr, nullptr, &ax), IAD_OK);
    iad_identity id;
    CHECK_EQ(iad_axis_connect(ax, 1, &id), IAD_OK);
    CHECK(has(id.kind, "slider"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    iad_axis* third = open_sim("pcon", 20.0, 3);
    CHECK_EQ(iad_axis_identify(third, &id), IAD_OK);
    CHECK_EQ(id.slave, 3);
    CHECK_EQ(std::string(id.product_family), std::string("PCON"));
    CHECK_EQ(iad_axis_close(third), IAD_OK);
    iad_axis* rotary = open_sim("ROTARY");  // case-insensitive
    CHECK_EQ(iad_axis_identify(rotary, &id), IAD_OK);
    CHECK_EQ(std::string(id.units), std::string("deg"));
    CHECK_EQ(id.rotary, IAD_TRUE);
    CHECK_EQ(iad_axis_close(rotary), IAD_OK);
    iad_axis* unknown = open_sim("unknown");
    CHECK_EQ(iad_axis_identify(unknown, &id), IAD_OK);
    CHECK_EQ(id.unit_known, IAD_FALSE);
    CHECK_EQ(iad_axis_close(unknown), IAD_OK);
    // a modelled baud rate is detected when the open options leave it at 0
    so.time_scale = 20.0;
    so.baud_rate = 38400;
    CHECK_EQ(iad_axis_open_sim("rod", &so, nullptr, nullptr, &ax), IAD_OK);
    int32_t rate = 0;
    iad_bool detected = 0;
    CHECK_EQ(iad_axis_baud_rate(ax, &rate, &detected), IAD_OK);
    CHECK_EQ(rate, 38400);
    CHECK_EQ(detected, IAD_TRUE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_axis_open_and_cell_open_refuse_bad_arguments_before_any_port) {
    // None of these may open a port: the checks come first (and the names are no port anyway).
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open(nullptr, nullptr, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("port must not be NULL"));
    CHECK_EQ(iad_axis_open("", nullptr, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_open("NOPORT", nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_open_options oo;
    iad_open_options_init(&oo);
    oo.slave = 248;
    CHECK_EQ(iad_axis_open("NOPORT", &oo, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    iad_open_options_init(&oo);
    oo.read_timeout_s = -1.0;
    CHECK_EQ(iad_axis_open("NOPORT", &oo, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    iad_open_options_init(&oo);
    oo.retries = -1;
    CHECK_EQ(iad_axis_open("NOPORT", &oo, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    iad_open_options_init(&oo);
    oo.baud_rate = -9600;
    CHECK_EQ(iad_axis_open("NOPORT", &oo, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.stroke_min = 5.0;
    CHECK_EQ(iad_axis_open("NOPORT", nullptr, &ao, &ax), IAD_ERR_INVALID_ARGUMENT);
    iad_axis_options_init(&ao);
    ao.max_accel_g = -1.0;
    CHECK_EQ(iad_axis_open("NOPORT", nullptr, &ao, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK(ax == nullptr);
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open("NOPORT1:1,NOPORT2", nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "either every entry names a slave"));
    CHECK_EQ(iad_cell_open("NOPORT1:0", nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open("NOPORT1:x", nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open(":1", nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open("NOPORT1,,NOPORT2", nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "empty name"));
    CHECK_EQ(iad_cell_open("NOPORT1", nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.scan_first = 5;
    co.scan_last = 2;
    CHECK_EQ(iad_cell_open("NOPORT1", &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    iad_cell_options_init(&co);
    co.max_axes = -1;
    CHECK_EQ(iad_cell_open("NOPORT1", &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK(cell == nullptr);
}

TEST(capi_axis_motion_on_the_simulator) {
    iad_axis* ax = open_sim("slider");
    iad_status st;
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_servo(ax, 1, ptr_of(servo_opts(1, kNaN)), &st), IAD_OK);
    CHECK_EQ(st.servo_on, IAD_TRUE);
    CHECK_EQ(iad_axis_home(ax, ptr_of(home_opts(1, kNaN, IAD_METHOD_DEFAULT)), &st), IAD_OK);
    CHECK_EQ(st.homed, IAD_TRUE);
    CHECK_EQ(iad_axis_home(ax, ptr_of(home_opts(1, kNaN, IAD_METHOD_IAI)), &st), IAD_OK);  // 0x45 cmd 0x0A
    CHECK_EQ(st.homed, IAD_TRUE);
    iad_move_options mo = moves(100.0);
    CHECK_EQ(iad_axis_move_to(ax, 50.0, &mo, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 50.0, 0.11);
    CHECK_EQ(st.in_position, IAD_TRUE);
    CHECK_EQ(st.idle, IAD_TRUE);
    CHECK_EQ(iad_axis_move_by(ax, 10.0, &mo, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 60.0, 0.11);
    CHECK_EQ(iad_axis_jog(ax, -1.0, nullptr, &st), IAD_OK);  // the jog defaults
    CHECK_NEAR(st.position_mm, 59.0, 0.11);
    CHECK_EQ(iad_axis_move_to(ax, 40.0, nullptr, nullptr), IAD_OK);  // the move defaults, no status wanted
    // checks: nothing sent, the core's error kinds and messages
    CHECK_EQ(iad_axis_check_move(ax, 30.0, &mo, &st), IAD_OK);
    CHECK_EQ(iad_axis_check_move(ax, 1e6, &mo, &st), IAD_ERR_SOFT_LIMIT);
    CHECK(!message().empty());
    CHECK(has(last_json(), "\"kind\":\"SoftLimitError\""));
    CHECK_EQ(iad_axis_move_to(ax, 1e6, &mo, &st), IAD_ERR_SOFT_LIMIT);
    iad_move_options fast = moves(1e6);
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &fast, &st), IAD_ERR_SOFT_LIMIT);
    CHECK_EQ(iad_axis_change_speed(ax, 20.0, ptr_of(change_opts(0, kNaN)), &st), IAD_ERR_NOT_READY);  // no move running
    CHECK_EQ(iad_axis_cancel_paused_move(ax, ptr_of(cancel_paused_opts(kNaN)), &st), IAD_ERR_NOT_READY);     // no pause
    // without waiting: `out` is left unwritten, then wait_idle
    iad_move_options nowait = moves(100.0, false);
    st.position_mm = -12345.0;
    CHECK_EQ(iad_axis_move_to(ax, 100.0, &nowait, &st), IAD_OK);
    CHECK_EQ(st.position_mm, -12345.0);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(100.0, 0.1, kNaN)), &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 100.0, 0.11);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(kNaN, kNaN, 2.0)), &st), IAD_OK);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(kNaN, kNaN, 2.0)), nullptr), IAD_OK);
    // change speed and pause / resume of a running move
    iad_move_options slow = moves(20.0, false);
    CHECK_EQ(iad_axis_move_to(ax, 120.0, &slow, nullptr), IAD_OK);
    CHECK_EQ(iad_axis_change_speed(ax, 40.0, ptr_of(change_opts(0, kNaN)), nullptr), IAD_OK);
    CHECK_EQ(iad_axis_pause(ax), IAD_OK);
    iai_actuator_driver::sleep_at_least(0.05);
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(st.paused, IAD_TRUE);
    CHECK_EQ(iad_axis_resume(ax), IAD_OK);
    CHECK_EQ(iad_axis_wait_idle(ax, ptr_of(wait_idle_opts(120.0, 0.1, kNaN)), &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 120.0, 0.11);
    // cancel a paused move: the remaining travel is dropped
    CHECK_EQ(iad_axis_move_to(ax, 20.0, &slow, nullptr), IAD_OK);
    iai_actuator_driver::sleep_at_least(0.05);
    CHECK_EQ(iad_axis_pause(ax), IAD_OK);
    CHECK_EQ(iad_axis_cancel_paused_move(ax, ptr_of(cancel_paused_opts(kNaN)), &st), IAD_OK);
    CHECK_EQ(st.paused, IAD_FALSE);
    // a decel stop at rest
    CHECK_EQ(iad_axis_stop(ax, ptr_of(stop_opts(1, kNaN)), &st), IAD_OK);
    CHECK_EQ(st.moving, IAD_FALSE);
    CHECK_EQ(iad_axis_stop(ax, ptr_of(stop_opts(0, 1.0)), nullptr), IAD_OK);
    // a stored row (read and checked first)
    iad_position_entry row;
    CHECK_EQ(iad_axis_read_position_table(ax, 1, &row), IAD_OK);
    CHECK_EQ(iad_axis_move_to_position(ax, 1, nullptr, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, row.position_mm, row.band_mm + 0.02);
    iad_position_move_options pm;
    iad_position_move_options_init(&pm);
    pm.speed_pct = 50.0;
    CHECK_EQ(iad_axis_move_to_position(ax, 0, &pm, &st), IAD_OK);
    // alarm reset; servo off
    CHECK_EQ(iad_axis_reset_alarm(ax, IAD_METHOD_DEFAULT, &st), IAD_OK);
    CHECK_EQ(st.alarm, IAD_FALSE);
    CHECK_EQ(iad_axis_servo(ax, 0, ptr_of(servo_opts(1, kNaN)), &st), IAD_OK);
    CHECK_EQ(st.servo_on, IAD_FALSE);
    CHECK_EQ(iad_axis_move_to(ax, 50.0, &mo, &st), IAD_ERR_NOT_READY);
    CHECK(has(last_json(), "\"kind\":\"EcNotReadyError\""));
    CHECK_EQ(iad_axis_servo(ax, 1, ptr_of(servo_opts(0, kNaN)), nullptr), IAD_OK);  // no wait
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_stop_from_another_thread_ends_a_waiting_move) {
    iad_axis* ax = open_sim("slider", 1.0);
    iad_move_options to_start = moves(200.0);
    CHECK_EQ(iad_axis_move_to(ax, 10.0, &to_start, nullptr), IAD_OK);
    iad_result stop_result = IAD_ERR_INTERNAL;
    std::string stop_thread_message = "unset";
    std::thread stopper([&] {
        iai_actuator_driver::sleep_at_least(0.2);
        stop_result = iad_axis_stop(ax, ptr_of(stop_opts(0, kNaN)), nullptr);
        stop_thread_message = iad_last_error_message();
    });
    iad_move_options slow = moves(20.0);
    iad_status st;
    const iad_result moved = iad_axis_move_to(ax, 140.0, &slow, &st);
    stopper.join();
    CHECK_EQ(stop_result, IAD_OK);
    CHECK_EQ(stop_thread_message, std::string(""));
    CHECK_EQ(moved, IAD_ERR_MOTION_INCOMPLETE);
    CHECK(has(last_json(), "\"kind\":\"MotionIncompleteError\""));
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK(st.position_mm < 30.0);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_position_table_writes_need_no_opt_in) {
    iad_axis* ax = open_sim("slider");
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    char buf[16];
    std::size_t need = 0;
    CHECK_EQ(iad_axis_last_result_json(ax, buf, sizeof buf, &need), IAD_ERR_NOT_READY);  // nothing written yet
    iad_position_entry row;
    CHECK_EQ(iad_axis_read_position_table(ax, 2, &row), IAD_OK);
    CHECK_NEAR(row.position_mm, 75.0, 0.001);
    row.position_mm = 60.0;
    row.reserved[0] = 0;
    iad_position_write_result out;
    // the simple-data store stays refused; a stored row needs no opt-in (allow_nonvolatile: accepted and ignored)
    CHECK_EQ(iad_axis_write_position_table(ax, 256, &row, nullptr, &out), IAD_ERR_NONVOLATILE_REFUSED);
    CHECK(has(last_json(), "\"kind\":\"NonVolatileWriteError\""));
    int64_t total = -1;
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, nullptr, nullptr), IAD_OK);
    CHECK_EQ(total, 0);
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    nv.allow_nonvolatile = IAD_TRUE;
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &row, nullptr, &out), IAD_OK);
    CHECK_EQ(out.row, 2);
    CHECK_EQ(out.skipped, IAD_FALSE);
    CHECK_EQ(out.warning_count, 0);
    CHECK_NEAR(out.position_mm, 60.0, 0.001);
    CHECK_EQ(out.checksum_ok, IAD_TRUE);
    const std::string written =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(written, "\"row\":2"));
    CHECK(has(written, "\"skipped\":false"));
    // unchanged: nothing written (one write cycle saved), unless forced
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &row, &nv, &out), IAD_OK);
    CHECK_EQ(out.skipped, IAD_TRUE);
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, nullptr, nullptr), IAD_OK);
    CHECK_EQ(total, 1);
    nv.force = IAD_TRUE;
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &row, &nv, nullptr), IAD_OK);  // out may be NULL
    const std::string forced =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(forced, "\"forced\":true"));
    int32_t hour = -1;
    iad_bool uncommitted = -1;
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, &hour, &uncommitted), IAD_OK);
    CHECK_EQ(total, 2);
    CHECK_EQ(hour, 2);
    CHECK_EQ(uncommitted, IAD_FALSE);
    // a row past the unit's rows: written, with warnings in the result (past the rows; not a row of the 3-position
    // mode; row 2's role bit 0x0020 on row 5, which holds 0: a role-bit change)
    CHECK_EQ(iad_axis_write_position_table(ax, 5, &row, nullptr, &out), IAD_OK);
    CHECK_EQ(out.skipped, IAD_FALSE);
    CHECK_EQ(out.warning_count, 3);
    const std::string past =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(past, "row 5 is past them: written anyway"));
    // refusals: outside the stroke, a non-finite value
    row.position_mm = 400.0;
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &row, &nv, &out), IAD_ERR_SOFT_LIMIT);
    row.position_mm = kNaN;
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &row, &nv, &out), IAD_ERR_INVALID_ARGUMENT);
    // teach: the current position into a row
    iad_move_options mo = moves(100.0);
    CHECK_EQ(iad_axis_move_to(ax, 40.0, &mo, nullptr), IAD_OK);
    nv.force = IAD_FALSE;
    CHECK_EQ(iad_axis_teach_position(ax, 0, IAD_TEACH_TABLE, nullptr, &out), IAD_OK);  // no opt-in needed
    CHECK_NEAR(out.position_mm, 40.0, 0.011);
    CHECK_EQ(iad_axis_teach_position(ax, 0, IAD_TEACH_TABLE, &nv, &out), IAD_OK);
    CHECK_EQ(out.skipped, IAD_TRUE);  // the row now holds it
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_parameters_list_read_and_guarded_writes) {
    iad_axis* ax = open_sim("gripper");
    CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    const std::string list =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_parameters_json(ax, b, n, r); });
    CHECK(!list.empty() && list[0] == '[');
    CHECK(has(list, "\"number\":2"));
    iad_parameter p;
    CHECK_EQ(iad_axis_read_parameter(ax, "P2", &p), IAD_OK);
    CHECK_EQ(p.number, 2);
    CHECK_NEAR(p.value, 0.10, 1e-9);
    CHECK_EQ(std::string(p.family), std::string("EC"));
    CHECK(std::strlen(p.name) > 0);
    CHECK(std::string(p.address).rfind("0x", 0) == 0);
    const std::string one = read_text(
        [ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_read_parameter_json(ax, "2", b, n, r); });
    CHECK(has(one, "\"number\":2"));
    CHECK(has(one, "\"choices\""));
    // guards refuse before anything is written
    iad_param_write_result r;
    CHECK_EQ(iad_axis_write_parameter(ax, "P99", "1", nullptr, &r), IAD_ERR_UNSAFE_REFUSED);  // not a parameter
    CHECK(has(last_json(), "\"kind\":\"UnsafeCommandError\""));
    CHECK_EQ(iad_axis_write_parameter(ax, "P2", "5.0", nullptr, &r), IAD_ERR_SOFT_LIMIT);
    CHECK_EQ(iad_axis_write_parameter(ax, "P2", "  ", nullptr, &r), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(message(), std::string("value is empty"));
    CHECK_EQ(iad_axis_write_parameter_number(ax, "P2", kNaN, nullptr, &r), IAD_ERR_INVALID_ARGUMENT);
    int64_t total = -1;
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, nullptr, nullptr), IAD_OK);
    CHECK_EQ(total, 0);
    // a write: verified and committed
    CHECK_EQ(iad_axis_write_parameter_number(ax, "2", 0.20, nullptr, &r), IAD_OK);
    CHECK_EQ(r.number, 2);
    CHECK_EQ(r.changed, IAD_TRUE);
    CHECK_EQ(r.verified, IAD_TRUE);
    CHECK_EQ(r.committed, IAD_TRUE);
    CHECK_EQ(r.saved, IAD_TRUE);
    CHECK_EQ(r.skipped, IAD_FALSE);
    CHECK(std::string(r.address).rfind("0x", 0) == 0);
    CHECK(std::strlen(r.after) > 0);
    const std::string last =
        read_text([ax](char* b, std::size_t n, std::size_t* r2) { return iad_axis_last_result_json(ax, b, n, r2); });
    CHECK(has(last, "\"takes_effect\""));
    CHECK_EQ(iad_axis_read_parameter(ax, "2", &p), IAD_OK);
    CHECK_NEAR(p.value, 0.20, 1e-9);
    // unchanged: skipped; the text form; RAM only
    CHECK_EQ(iad_axis_write_parameter(ax, "P2", "0.2", nullptr, &r), IAD_OK);
    CHECK_EQ(r.skipped, IAD_TRUE);
    iad_param_write_options ram;
    iad_param_write_options_init(&ram);
    ram.commit = IAD_FALSE;
    CHECK_EQ(iad_axis_write_parameter(ax, "P2", "0.3", &ram, &r), IAD_OK);
    CHECK_EQ(r.committed, IAD_FALSE);
    iad_bool uncommitted = 0;
    CHECK_EQ(iad_axis_nonvolatile_writes(ax, &total, nullptr, &uncommitted), IAD_OK);
    CHECK_EQ(total, 1);
    CHECK_EQ(uncommitted, IAD_TRUE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_software_reset_and_baud_change) {
    iad_axis* rod = open_sim("rod");
    CHECK_EQ(iad_axis_connect(rod, 1, nullptr), IAD_OK);
    iad_reset_options ro;
    iad_reset_options_init(&ro);
    ro.reset_wait_s = 0.1;
    ro.timeout_s = 5.0;
    iad_reset_result rr;
    CHECK_EQ(iad_axis_software_reset(rod, &ro, &rr), IAD_OK);
    CHECK_EQ(rr.confirmed, IAD_TRUE);
    CHECK(std::strlen(rr.confirmed_by) > 0);
    CHECK_EQ(rr.needs_home, IAD_TRUE);  // the rod is incremental
    CHECK_EQ(rr.status.homed, IAD_FALSE);
    CHECK_EQ(rr.baud_rate, 0);
    const std::string reset =
        read_text([rod](char* b, std::size_t n, std::size_t* r) { return iad_axis_last_result_json(rod, b, n, r); });
    CHECK(has(reset, "\"accepted\""));
    iad_baud_result br;
    CHECK_EQ(iad_axis_set_baud_rate(rod, 115200, nullptr, &br), IAD_ERR_NOT_READY);  // no rate on this transport
    CHECK_EQ(iad_axis_close(rod), IAD_OK);
    // a unit with a modelled line
    iad_sim_options so;
    iad_sim_options_init(&so);
    so.time_scale = 20.0;
    so.baud_rate = 38400;
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_sim("rod", &so, nullptr, nullptr, &ax), IAD_OK);
    CHECK_EQ(iad_axis_set_baud_rate(ax, 12345, nullptr, &br), IAD_ERR_SOFT_LIMIT);  // no IAI code
    iad_baud_options bo;
    iad_baud_options_init(&bo);
    bo.method = 9;
    CHECK_EQ(iad_axis_set_baud_rate(ax, 115200, &bo, &br), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_set_baud_rate(ax, 115200, nullptr, &br), IAD_OK);
    CHECK_EQ(br.previous, 38400);
    CHECK_EQ(br.baud_rate, 115200);
    CHECK_EQ(br.method, IAD_BAUD_DIRECT);
    CHECK_EQ(br.verified, IAD_TRUE);
    CHECK(std::strlen(br.frame) > 0);
    int32_t rate = 0;
    CHECK_EQ(iad_axis_baud_rate(ax, &rate, nullptr), IAD_OK);
    CHECK_EQ(rate, 115200);
    const std::string baud =
        read_text([ax](char* b, std::size_t n, std::size_t* r) { return iad_axis_last_result_json(ax, b, n, r); });
    CHECK(has(baud, "\"baudrate\":115200"));
    iad_status st;
    CHECK_EQ(iad_axis_status(ax, &st), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

// ============================================================================================ cell

TEST(capi_cell_group_commands_on_simulated_lines) {
    iad_sim_options so;
    iad_sim_options_init(&so);
    so.time_scale = 20.0;
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open_sim("slider, gripper", 2, &so, nullptr, nullptr, &cell), IAD_OK);
    REQUIRE(cell != nullptr);
    std::size_t n = 0;
    CHECK_EQ(iad_cell_axis_count(cell, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(4));
    const char* expected[] = {"SIM1:1", "SIM1:2", "SIM2:1", "SIM2:2"};
    for (std::size_t i = 0; i < 4; ++i) {
        const std::string name = read_text(
            [cell, i](char* b, std::size_t size, std::size_t* r) { return iad_cell_axis_name(cell, i, b, size, r); });
        CHECK_EQ(name, std::string(expected[i]));
    }
    char buf[8];
    std::size_t need = 0;
    CHECK_EQ(iad_cell_axis_name(cell, 4, buf, sizeof buf, &need), IAD_ERR_INVALID_ARGUMENT);
    const std::string info =
        read_text([cell](char* b, std::size_t size, std::size_t* r) { return iad_cell_info_json(cell, b, size, r); });
    CHECK(has(info, "\"axes\":[\"SIM1:1\",\"SIM1:2\",\"SIM2:1\",\"SIM2:2\"]"));
    CHECK(has(info, "\"port_errors\":{}"));
    CHECK(has(info, "\"faults\":{}"));
    CHECK(has(info, "\"SIM2:1\":{\"slave\":1"));
    iad_cell_result results[4];
    CHECK_EQ(iad_cell_servo_all(cell, 1, nullptr, kNaN, results, 4, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(4));
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK_EQ(std::string(results[i].axis), std::string(expected[i]));
        CHECK_EQ(results[i].code, IAD_OK);
        CHECK_EQ(results[i].status.servo_on, IAD_TRUE);
        CHECK_EQ(std::string(results[i].message), std::string(""));
    }
    CHECK_EQ(iad_cell_home_all(cell, nullptr, kNaN, IAD_METHOD_DEFAULT, IAD_TRUE, results, 4, &n), IAD_OK);
    CHECK_EQ(results[3].status.homed, IAD_TRUE);
    // a move of two axes: results in the order of the list
    iad_move_options mo = moves(50.0);
    const double targets[2] = {5.0, 20.0};
    CHECK_EQ(iad_cell_move_all(cell, "SIM2:1,SIM1:1", targets, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(2));
    CHECK_EQ(std::string(results[0].axis), std::string("SIM2:1"));
    CHECK_NEAR(results[0].status.position_mm, 5.0, 0.11);
    CHECK_NEAR(results[1].status.position_mm, 20.0, 0.11);
    // a results array that is too small: every axis is still handled, *count says how many
    iad_cell_result one[2];
    std::memset(one, 0x5A, sizeof one);
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, one, 1, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(4));
    CHECK_EQ(std::string(one[0].axis), std::string("SIM1:1"));
    CHECK_NEAR(one[0].status.position_mm, 20.0, 0.11);
    CHECK_EQ(static_cast<unsigned char>(one[1].axis[0]), 0x5A);  // not written
    CHECK_EQ(iad_cell_status_all(cell, "SIM2:2, SIM1:2", kNaN, nullptr, 0, &n), IAD_OK);  // results may be NULL
    CHECK_EQ(n, std::size_t(2));
    CHECK_EQ(iad_cell_status_all(cell, "SIM9:1", kNaN, results, 4, &n), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "no axis 'SIM9:1'"));
    CHECK_EQ(n, std::size_t(0));
    // one bad target refuses the whole move: nothing started, every axis reported
    const double bad[2] = {10.0, 500.0};
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM2:1", bad, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_ERR_CELL);
    CHECK_EQ(n, std::size_t(2));
    CHECK_EQ(results[0].code, IAD_OK);
    CHECK_EQ(results[0].status.raw_words, 0);  // not read: the move was refused for another axis
    CHECK_EQ(results[1].code, IAD_ERR_SOFT_LIMIT);
    CHECK(std::strlen(results[1].message) > 0);
    const std::string cell_error = last_json();
    CHECK(has(cell_error, "\"kind\":\"CellError\""));
    CHECK(has(cell_error, "\"operation\":\"move_all (check; nothing started)\""));
    CHECK(has(cell_error, "\"SIM2:1\":{\"code\":-1,\"name\":\"IAD_ERR_SOFT_LIMIT\",\"kind\":\"SoftLimitError\""));
    CHECK_EQ(iad_cell_status_all(cell, "SIM1:1", kNaN, results, 4, &n), IAD_OK);
    CHECK_NEAR(results[0].status.position_mm, 20.0, 0.11);  // did not move
    // argument errors of move_all
    CHECK_EQ(iad_cell_move_all(cell, nullptr, targets, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1", targets, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1,SIM1:1", targets, 2, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "two moves for axis SIM1:1"));
    CHECK_EQ(iad_cell_move_all(cell, "SIM1:1", nullptr, 1, &mo, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_ERR_INVALID_ARGUMENT);
    // without waiting, then wait_all
    const double home[2] = {1.0, 1.0};
    CHECK_EQ(iad_cell_move_all(cell, "SIM2:1,SIM1:1", home, 2, &mo, nullptr, nullptr, nullptr, nullptr, nullptr, 0, nullptr), IAD_OK);
    iad_move_options nowait = moves(50.0, false);
    CHECK_EQ(iad_cell_move_all(cell, "SIM2:1,SIM1:1", targets, 2, &nowait, nullptr, nullptr, nullptr, nullptr, results, 4, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(0));
    CHECK_EQ(iad_cell_wait_all(cell, "SIM2:1,SIM1:1", kNaN, IAD_TRUE, results, 4, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(2));
    CHECK_NEAR(results[0].status.position_mm, 5.0, 0.11);
    CHECK_NEAR(results[1].status.position_mm, 20.0, 0.11);
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_FALSE, results, 4, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(4));
    CHECK_EQ(iad_cell_stop_all(cell, nullptr, kNaN, IAD_FALSE, nullptr, 0, nullptr), IAD_OK);  // results and count may be NULL
    // faults
    CHECK_EQ(iad_cell_clear_fault(cell, "SIM1:1"), IAD_OK);
    CHECK_EQ(iad_cell_clear_fault(cell, "SIM7:1"), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_clear_fault(cell, nullptr), IAD_ERR_INVALID_ARGUMENT);
    // an axis handle of the cell: every single-axis function works on it; closing it leaves the axis in the cell
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_cell_open_axis(cell, "SIM9:9", &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_axis(cell, nullptr, &ax), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_axis(cell, "SIM2:2", &ax), IAD_OK);
    iad_status st;
    CHECK_EQ(iad_axis_move_to(ax, 3.0, &mo, &st), IAD_OK);
    CHECK_NEAR(st.position_mm, 3.0, 0.11);
    iad_identity id;
    CHECK_EQ(iad_axis_identify(ax, &id), IAD_OK);
    CHECK_EQ(id.slave, 2);
    CHECK_EQ(std::string(id.kind), std::string("gripper"));
    CHECK_EQ(iad_cell_close(reinterpret_cast<iad_cell*>(ax)), IAD_ERR_INVALID_ARGUMENT);  // wrong kind
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_axis_status(ax, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_status_all(cell, "SIM2:2", kNaN, results, 4, &n), IAD_OK);  // still in the cell, still open
    CHECK_NEAR(results[0].status.position_mm, 3.0, 0.11);
    // closing the cell ends its axis handles; closing them and the cell again stays safe
    iad_axis* kept = nullptr;
    CHECK_EQ(iad_cell_open_axis(cell, "SIM1:2", &kept), IAD_OK);
    CHECK_EQ(iad_axis_close(reinterpret_cast<iad_axis*>(cell)), IAD_ERR_INVALID_ARGUMENT);  // wrong kind
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_axis_status(kept, &st), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_axis_close(kept), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    CHECK_EQ(iad_cell_axis_count(cell, &n), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(message(), "invalid cell handle"));
}

TEST(capi_cell_open_sim_checks_its_arguments) {
    iad_cell* cell = nullptr;
    CHECK_EQ(iad_cell_open_sim(nullptr, 0, nullptr, nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_sim(nullptr, 17, nullptr, nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_sim("slider,nope", 1, nullptr, nullptr, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_open_sim(nullptr, 1, nullptr, nullptr, nullptr, nullptr), IAD_ERR_INVALID_ARGUMENT);
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.max_axes = 2;
    CHECK_EQ(iad_cell_open_sim("slider,slider", 2, nullptr, &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK(cell == nullptr);
    // NULL profiles: one slider line; the same profile twice gives two lines
    CHECK_EQ(iad_cell_open_sim(nullptr, 1, nullptr, nullptr, nullptr, &cell), IAD_OK);
    std::size_t n = 0;
    CHECK_EQ(iad_cell_axis_count(cell, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(1));
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    co.max_axes = 0;  // no limit
    co.identify = IAD_FALSE;
    CHECK_EQ(iad_cell_open_sim("rod,rod", 1, nullptr, &co, nullptr, &cell), IAD_OK);
    CHECK_EQ(iad_cell_axis_count(cell, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(2));
    const std::string info =
        read_text([cell](char* b, std::size_t size, std::size_t* r) { return iad_cell_info_json(cell, b, size, r); });
    CHECK(has(info, "\"info\":{}"));  // identify off
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    // no total limit by default: 2 lines x 9 = 18 axes; max_axes still caps (listed slaves: refused)
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    CHECK_EQ(iad_cell_open_sim("slider,slider", 9, nullptr, &co, nullptr, &cell), IAD_OK);
    CHECK_EQ(iad_cell_axis_count(cell, &n), IAD_OK);
    CHECK_EQ(n, std::size_t(18));
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    co.max_axes = 17;
    CHECK_EQ(iad_cell_open_sim("slider,slider", 9, nullptr, &co, nullptr, &cell), IAD_ERR_INVALID_ARGUMENT);
    CHECK(cell == nullptr);
}

// ============================================================================================ error mapping

TEST(capi_every_exception_class_has_its_code) {
    using namespace iai_actuator_driver;
    auto code = [](auto e) { return capi::describe_error(std::make_exception_ptr(e)).code; };
    auto kind = [](auto e) { return capi::describe_error(std::make_exception_ptr(e)).kind; };
    CHECK_EQ(code(SoftLimitError("x")), IAD_ERR_SOFT_LIMIT);
    CHECK_EQ(code(EcNotReadyError("x")), IAD_ERR_NOT_READY);
    CHECK_EQ(code(EcAlarmError(0x0D9)), IAD_ERR_ALARM);
    CHECK_EQ(code(MotionTimeoutError("x")), IAD_ERR_MOTION_TIMEOUT);
    CHECK_EQ(code(ModbusTimeout("x")), IAD_ERR_MODBUS_TIMEOUT);
    CHECK_EQ(code(ModbusCrcError("x")), IAD_ERR_FRAME);
    CHECK_EQ(code(ModbusFrameError("x")), IAD_ERR_FRAME);
    CHECK_EQ(code(TransportError("x")), IAD_ERR_TRANSPORT);
    CHECK_EQ(code(VerificationError("x")), IAD_ERR_VERIFICATION);
    CHECK_EQ(code(NonVolatileWriteError("x")), IAD_ERR_NONVOLATILE_REFUSED);
    CHECK_EQ(code(std::invalid_argument("x")), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(code(std::out_of_range("x")), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(code(std::logic_error("x")), IAD_ERR_UNSUPPORTED);
    CHECK_EQ(code(std::runtime_error("x")), IAD_ERR_INTERNAL);
    CHECK_EQ(code(std::bad_alloc()), IAD_ERR_INTERNAL);
    CHECK_EQ(code(42), IAD_ERR_INTERNAL);
    CHECK_EQ(kind(42), std::string("unknown"));
    CHECK_EQ(code(capi::BufferTooSmall("x")), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(code(ServoBlockedError("x", {"EMG input"})), IAD_ERR_SERVO_BLOCKED);
    CHECK_EQ(code(MotionIncompleteError("x")), IAD_ERR_MOTION_INCOMPLETE);
    CHECK_EQ(code(ModbusSlaveError(2, 3)), IAD_ERR_SLAVE_EXCEPTION);
    CHECK_EQ(code(IaiCommandRejected(0x45, 0x0A, 0x11, "1st operand error", "")), IAD_ERR_COMMAND_REJECTED);
    CHECK_EQ(code(UnsafeCommandError("x")), IAD_ERR_UNSAFE_REFUSED);
    CHECK_EQ(code(CellError("op", {}, {})), IAD_ERR_CELL);
    CHECK_EQ(code(ModbusError("x")), IAD_ERR_MODBUS);
    CHECK_EQ(code(EcError("x")), IAD_ERR_DRIVER);
    CHECK_EQ(code(EcDriverError("x")), IAD_ERR_DRIVER);
    CHECK_EQ(kind(ModbusCrcError("x")), std::string("ModbusCrcError"));
    CHECK_EQ(kind(EcDriverError("x")), std::string("EcDriverError"));
    CHECK_EQ(kind(std::out_of_range("x")), std::string("std::out_of_range"));
    // the details of iad_last_error_json
    EcAlarmError alarm(0x0D9);
    alarm.add_note("decel stop sent after this error");
    alarm.set_stop_attempted();
    const capi::ErrorDescription a = capi::describe_error(std::make_exception_ptr(alarm));
    CHECK_EQ(a.message, std::string(alarm.what()));
    CHECK(has(a.message, "[decel stop sent after this error]"));
    const std::string aj = a.json.dump();
    CHECK(has(aj, "\"notes\":[\"decel stop sent after this error\"]"));
    CHECK(has(aj, "\"stop_attempted\":true"));
    CHECK(has(aj, "\"alarm_code\":217"));
    ModbusTimeout timeout("no answer");
    timeout.set_attempts(4);
    CHECK(has(capi::describe_error(std::make_exception_ptr(timeout)).json.dump(), "\"attempts\":4"));
    const std::string sj = capi::describe_error(std::make_exception_ptr(ModbusSlaveError(2, 3))).json.dump();
    CHECK(has(sj, "\"exception_code\":2"));
    CHECK(has(sj, "\"function\":3"));
    CHECK(has(sj, "\"reason\":\"Illegal Data Address\""));
    const std::string rj = capi::describe_error(std::make_exception_ptr(
                                                    IaiCommandRejected(0x45, 0x0A, 0x11, "1st operand error", "hint")))
                               .json.dump();
    CHECK(has(rj, "\"command\":10"));
    CHECK(has(rj, "\"rejection\":17"));
    CHECK(has(rj, "\"reason\":\"1st operand error\""));
    CHECK(has(rj, "\"hint\":\"hint\""));
    const std::string bj =
        capi::describe_error(std::make_exception_ptr(ServoBlockedError("blocked", {"EMG input", "drive power cut"})))
            .json.dump();
    CHECK(has(bj, "\"reasons\":[\"EMG input\",\"drive power cut\"]"));
    std::map<std::string, std::exception_ptr> errors{{"COM3:1", std::make_exception_ptr(TransportError("gone"))}};
    const std::string cj =
        capi::describe_error(std::make_exception_ptr(CellError("stop_all", {{"COM3:1", "gone"}}, errors))).json.dump();
    CHECK(has(cj, "\"operation\":\"stop_all\""));
    CHECK(has(cj, "\"axes\":{\"COM3:1\":{\"code\":-7,\"name\":\"IAD_ERR_TRANSPORT\",\"kind\":\"TransportError\""));
    CHECK_EQ(capi::error_code(std::make_exception_ptr(VerificationError("x"))), IAD_ERR_VERIFICATION);
}

TEST(capi_text_fields_are_cut_at_a_character_boundary) {
    using iai_actuator_driver::capi::copy_text;
    char small[5];
    copy_text(small, "abcdef");
    CHECK_EQ(std::string(small), std::string("abcd"));
    // "ab" + U+00E9 (2 bytes): a 4-byte array holds 3 bytes of text, which would split the 2-byte character
    char four[4];
    copy_text(four, "ab\xC3\xA9");
    CHECK_EQ(std::string(four), std::string("ab"));
    char exact[5];
    copy_text(exact, "ab\xC3\xA9");
    CHECK_EQ(std::string(exact), std::string("ab\xC3\xA9"));
    // a 3-byte character (U+20AC) cut in its middle is dropped whole
    char three[5];
    copy_text(three, "ab\xE2\x82\xAC");
    CHECK_EQ(std::string(three), std::string("ab"));
}
