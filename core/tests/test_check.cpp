// The compatibility self-test (check.hpp) against the simulator: the read-only check (nothing but FC03 / FC11), the
// report fields and JSON, the short motion check out and back (and its step adapting to a short stroke), the failure
// paths (alarm, no link, a configured stroke the unit does not have), the warnings (FC11 silent, safety speed, rows
// outside the stroke, a CON No.10 below the minimum band), degrees on a rotary axis and the report file fallback.
// Ported from the Python edition's tests/test_cli.py (CheckTest), test_adaptive.py (CheckStepTest),
// test_hardware_fixes.py, test_settle.py, test_parameter_rules.py and test_info.py (the report's unit_info).
#include <filesystem>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/sim.hpp"

using namespace iai_actuator_driver;

namespace {

std::shared_ptr<SimulatedUnit> make_sim(SimulatedUnitOptions o) { return std::make_shared<SimulatedUnit>(o); }

SimulatedUnitOptions sim_options(SimProfile profile, double scale = 20.0) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    return o;
}

Axis open_axis(const std::shared_ptr<Transport>& transport, int slave = 1, AxisOptions axis = {},
               double read_timeout_s = kDefaultReadTimeoutS) {
    OpenOptions o;
    o.slave = slave;
    axis.poll_interval_s = 0.005;
    o.axis = axis;
    o.read_timeout_s = read_timeout_s;
    return Axis::open(transport, o);
}

CheckOptions label(bool motion = false) {
    CheckOptions o;
    o.motion = motion;
    o.port_label = "sim";
    return o;
}

std::set<int> functions(const SimulatedUnit& sim) {
    std::set<int> out;
    for (const auto& frame : sim.requests())
        if (frame.size() > 1) out.insert(frame[1]);
    return out;
}

bool read_only(const SimulatedUnit& sim) {
    for (int f : functions(sim))
        if (f != 0x03 && f != 0x11) return false;
    return true;
}

const CheckItem& item(const CheckReport& r, const std::string& name) {
    const CheckItem* i = r.item(name);
    if (i == nullptr) throw std::out_of_range("no check item '" + name + "'");
    return *i;
}

std::string result(const CheckReport& r, const std::string& name) {
    const CheckItem* i = r.item(name);
    return i ? i->result : std::string("<missing>");
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

double data_number(const CheckItem& i, const char* key) { return i.data.find(key)->as_number(); }

std::string dump_items(const CheckReport& r) {
    std::string out;
    for (const CheckItem& i : r.items) out += "\n    " + format_check_item(i);
    return out;
}

/// The simulator with FC11 silenced (firmware that does not answer Report Slave ID).
class SilentFc11 : public Transport {
public:
    explicit SilentFc11(std::shared_ptr<SimulatedUnit> sim) : sim_(std::move(sim)) {}
    void write(const std::uint8_t* data, std::size_t size) override {
        drop_ = size > 1 && data[1] == 0x11;
        if (!drop_) sim_->write(data, size);
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override { return drop_ ? 0 : sim_->read(buffer, size); }
    void reset_input() override { sim_->reset_input(); }
    int baud_rate() const override { return sim_->baud_rate(); }
    void set_baud_rate(int rate) override { sim_->set_baud_rate(rate); }
    double read_timeout() const override { return sim_->read_timeout(); }
    void set_read_timeout(double seconds) override { sim_->set_read_timeout(seconds); }
    void send_break(double seconds) override { sim_->send_break(seconds); }
    void close() override { sim_->close(); }
    bool is_open() const override { return sim_->is_open(); }
    std::string name() const override { return sim_->name(); }

private:
    std::shared_ptr<SimulatedUnit> sim_;
    bool drop_ = false;
};

}  // namespace

// ======================================================================================= read-only

TEST(check_read_only_on_the_slider) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    AxisOptions cap;
    cap.max_speed_mm_s = 100.0;  // the command line's conservative default cap
    Axis ax = open_axis(sim, 1, cap);
    sim->clear_requests();
    std::vector<std::string> seen;
    CheckOptions o = label();
    o.on_item = [&](const CheckItem& i) { seen.push_back(i.name); };
    const CheckReport report = run_check(ax, o);
    CHECK(report.passed());
    if (!report.passed()) std::printf("%s\n", dump_items(report).c_str());
    for (const char* name : {"link", "slave_id", "index_table", "identification", "soft_limits", "speed_limit", "status",
                             "alarm", "alarm_history", "position_table", "direct_move_block", "modbus_control",
                             "safety_speed", "unit_info", "parameter_ranges"})
        CHECK_EQ(result(report, name), kCheckPass);
    CHECK_EQ(result(report, "motion"), kCheckSkip);
    CHECK_EQ(seen.size(), report.items.size());
    CHECK_EQ(report.model.value_or(""), "MWB-EC-DSRR4-WA-MF");
    CHECK_EQ(report.controller.value_or(""), "EC 24 V, unit 0x02 (EC dust/drip-proof)");
    CHECK_NEAR(report.ec_max_speed_mm_s.value_or(0), 700.0, 1e-9);
    CHECK_NEAR(report.speed_cap_mm_s.value_or(0), 100.0, 1e-9);
    CHECK_EQ(report.firmware.value_or(""), "0013");
    CHECK_EQ(report.serial_number.value_or(""), "SIM-0000001");
    CHECK_EQ(report.model_name.value_or(""), "MWB-EC-DSRR4-WA-MF");
    REQUIRE(report.ec_soft_limits_mm && report.stroke);
    CHECK(*report.ec_soft_limits_mm == (Span{-0.3, 150.3}));
    CHECK(*report.stroke == (Span{0.0, 150.0}));
    CHECK(!report.configured_stroke.has_value());
    CHECK(!report.motion_requested);
    CHECK(read_only(*sim));  // nothing but reads and slave id
    CHECK_EQ(sim->nonvolatile_writes(), 0);
    CHECK(contains(item(report, "parameter_ranges").detail, "P4 home position adjustment is"));
    const Json json = to_json(report);
    CHECK_EQ(json.find("result")->as_string(), "PASS");
    CHECK_EQ(json.find("tool")->as_string(), "iai-check (unofficial)");
    CHECK(json.find("timestamp") != nullptr);
    CHECK(*json.find("ec_soft_limits_mm") == Json(std::vector<double>{-0.3, 150.3}));
    CHECK(json.find("configured_stroke")->is_null());
    CHECK(Json::parse(json.dump(2)) == json);
    // the report carries the unit information (test_info.py: test_check_report_carries_unit_info)
    const Json& info = report.unit_info;
    CHECK_EQ(info.find("meta")->find("schema")->as_string(), "ec_driver.unit_info/1");
    CHECK_EQ(info.find("maintenance")->find("total_moves")->as_int(), 101);
}

TEST(check_fc11_silent_is_a_warning) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    auto silent = std::make_shared<SilentFc11>(sim);
    Axis ax = open_axis(silent);
    const CheckReport report = run_check(ax, label());
    CHECK_EQ(result(report, "slave_id"), kCheckWarn);
    CHECK(contains(item(report, "slave_id").detail, "identity read from CTCL 0x0300"));  // class 3 item 1
    CHECK_EQ(result(report, "index_table"), kCheckPass);
    CHECK(report.passed());
}

TEST(check_no_link) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    Axis ax = open_axis(sim, 1, AxisOptions(), 0.05);
    sim->inject_fault(SimFault::Timeout, 100000);
    const CheckReport report = run_check(ax, label());
    CHECK_EQ(result(report, "link"), kCheckFail);
    CHECK(contains(item(report, "link").detail, "no answer"));
    CHECK_EQ(result(report, "slave_id"), kCheckSkip);
    CHECK_EQ(result(report, "motion"), kCheckSkip);
    CHECK(!report.passed());
    CHECK_EQ(to_json(report).find("result")->as_string(), "FAIL");
}

TEST(check_fails_on_alarm_and_skips_motion) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    Axis ax = open_axis(sim);
    sim->trigger_alarm(0x0E0);
    const CheckReport report = run_check(ax, label(true));
    CHECK_EQ(result(report, "alarm"), kCheckFail);
    CHECK_EQ(item(report, "alarm").detail, "alarm 0x0e0 active");
    CHECK_EQ(result(report, "motion"), kCheckSkip);
    CHECK(!report.passed());
}

TEST(check_configured_stroke_checked_against_the_unit) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    AxisOptions wide;
    wide.stroke = Span{0.0, 300.0};
    Axis ax = open_axis(sim, 1, wide);
    CheckReport report = run_check(ax, label());
    CHECK(report.passed());
    CHECK_EQ(result(report, "soft_limits"), kCheckWarn);
    CHECK(contains(item(report, "soft_limits").detail, "narrows it to 0.00..150.00"));
    auto sim2 = make_sim(sim_options(SimProfile::Slider));
    AxisOptions apart;
    apart.stroke = Span{200.0, 300.0};
    Axis ax2 = open_axis(sim2, 1, apart);
    report = run_check(ax2, label(true));
    CHECK(!report.passed());
    CHECK_EQ(result(report, "soft_limits"), kCheckFail);
    CHECK_EQ(result(report, "motion"), kCheckSkip);
}

TEST(check_short_unit_with_a_150_mm_configured_stroke) {
    SimulatedUnitOptions o = sim_options(SimProfile::Slider);
    o.stroke_mm = 50.0;
    o.position_mm = 25.0;
    auto sim = make_sim(o);
    AxisOptions a;
    a.stroke = Span{0.0, 150.0};
    Axis ax = open_axis(sim, 1, a);
    const CheckReport report = run_check(ax, label());
    CHECK(report.passed());
    CHECK_EQ(result(report, "soft_limits"), kCheckWarn);
    CHECK_EQ(result(report, "position_table"), kCheckWarn);  // rows at 150 and 75 mm lie outside 0..50
    REQUIRE(report.stroke.has_value());
    CHECK(*report.stroke == (Span{0.0, 50.0}));
}

TEST(check_safety_speed_reported) {
    SimulatedUnitOptions o = sim_options(SimProfile::Slider);
    o.safety_speed = true;
    auto sim = make_sim(o);
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label());
    CHECK(report.passed());
    CHECK_EQ(result(report, "safety_speed"), kCheckWarn);
    CHECK(item(report, "safety_speed").data.find("safety_speed")->as_bool());
}

// ======================================================================================= motion

TEST(check_motion_returns_to_start) {
    auto sim = make_sim(sim_options(SimProfile::Slider, 10.0));
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK(report.passed());
    if (!report.passed()) std::printf("%s\n", dump_items(report).c_str());
    CHECK_EQ(result(report, "motion_out"), kCheckPass);
    CHECK_EQ(result(report, "motion_back"), kCheckPass);
    CHECK_NEAR(data_number(item(report, "motion_out"), "reached_mm"), 78.95, 0.1);
    CHECK_NEAR(sim->position_mm(), 73.95, 1e-6);
    CHECK(report.motion_requested);
}

TEST(check_motion_near_the_end_goes_backwards) {
    SimulatedUnitOptions o = sim_options(SimProfile::Slider, 10.0);
    o.position_mm = 148.0;
    auto sim = make_sim(o);
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK(report.passed());
    CHECK_NEAR(data_number(item(report, "motion_out"), "target_mm"), 143.0, 0.001);
    CHECK_NEAR(sim->position_mm(), 148.0, 1e-6);
}

TEST(check_passes_on_the_gripper) {
    auto sim = make_sim(sim_options(SimProfile::Gripper, 20.0));
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK(report.passed());
    if (!report.passed()) std::printf("%s\n", dump_items(report).c_str());
    CHECK_EQ(report.model.value_or(""), "EC-GRBP8M-20-1-1-PN-WA-WL2");
    CHECK_EQ(report.controller.value_or(""), "EC 24 V, unit 0x0A (EC(MS) \"2-size EC\")");
    CHECK(contains(item(report, "slave_id").detail, "gripper (AKID 4)"));
    CHECK_NEAR(report.ec_max_speed_mm_s.value_or(0), 53.0, 1e-9);
    CHECK_NEAR(report.speed_cap_mm_s.value_or(0), 53.0, 1e-9);
    REQUIRE(report.stroke.has_value());
    CHECK(*report.stroke == (Span{0.0, 9.5}));
    CHECK_EQ(result(report, "speed_limit"), kCheckPass);
    CHECK(contains(item(report, "speed_limit").detail, "53.00 mm/s"));
    CHECK_EQ(result(report, "index_table"), kCheckPass);
    // the step adapts to the 9.50 mm stroke: a quarter of it (2.38 mm), toward the side with more room
    CHECK_NEAR(data_number(item(report, "motion_out"), "target_mm"), 7.12, 1e-9);
    CHECK_NEAR(data_number(item(report, "motion_back"), "reached_mm"), 9.5, 1e-9);
}

TEST(check_reports_the_settled_position) {
    SimulatedUnitOptions o = sim_options(SimProfile::Gripper, 20.0);
    o.position_mm = 3.0;
    auto sim = make_sim(o);
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK(report.passed());
    // the step is at most a quarter of the 9.50 mm stroke: 3.00 + 2.38, toward the side with more room
    const std::pair<const char*, double> legs[] = {{"motion_out", 5.38}, {"motion_back", 3.0}};
    for (const auto& leg : legs) {
        const CheckItem& i = item(report, leg.first);
        CHECK_EQ(i.result, kCheckPass);
        CHECK_NEAR(data_number(i, "target_mm"), leg.second, 1e-9);
        CHECK_NEAR(data_number(i, "reached_mm"), leg.second, 1e-9);
        CHECK_NEAR(data_number(i, "error_mm"), 0.0, 1e-9);
        CHECK(i.data.find("settled")->as_bool());
        CHECK(contains(i.detail, "settled"));
    }
}

TEST(check_motion_step_adapts_to_a_short_stroke) {
    SimulatedUnitOptions o = sim_options(SimProfile::Unknown, 20.0);
    o.stroke_mm = 4.0;
    o.position_mm = 2.0;
    auto sim = make_sim(o);
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK_EQ(result(report, "motion_out"), kCheckPass);
    if (result(report, "motion_out") != kCheckPass) std::printf("%s\n", dump_items(report).c_str());
    CHECK_NEAR(data_number(item(report, "motion_out"), "target_mm"), 3.0, 1e-9);  // a quarter of the 4 mm stroke
}

TEST(check_messages_use_deg_on_a_rotary_axis) {
    auto sim = make_sim(sim_options(SimProfile::Rotary, 20.0));
    Axis ax = open_axis(sim);
    const CheckReport report = run_check(ax, label(true));
    CHECK(contains(item(report, "soft_limits").detail, " deg"));
    CHECK(!contains(item(report, "position_table").detail, " mm"));
    CHECK(contains(item(report, "motion_out").detail, "deg/s"));
}

// ======================================================================================= parameters, report file

TEST(check_warns_on_a_con_band_below_the_minimum) {
    auto sim = make_sim(sim_options(SimProfile::Pcon));
    Axis ax = open_axis(sim);
    CheckReport report = run_check(ax, label());
    CHECK_EQ(result(report, "parameter_ranges"), kCheckPass);
    CHECK(contains(item(report, "parameter_ranges").detail, "minimum positioning band 0.01 mm"));
    ParameterWriteOptions w;
    w.commit = false;  // raw item write: No.10 = 0.00 in RAM
    ax.write_parameter("8.9", ParamValue::of(0), w);
    report = run_check(ax, label());
    CHECK_EQ(result(report, "parameter_ranges"), kCheckWarn);
    CHECK(contains(item(report, "parameter_ranges").detail, "No.10"));
    CHECK(report.passed());  // a WARN never fails the check
}

TEST(check_unwritable_report_falls_back_and_stays_read_only) {
    auto sim = make_sim(sim_options(SimProfile::Slider));
    Axis ax = open_axis(sim);
    sim->clear_requests();
    const CheckReport report = run_check(ax, label());
    CHECK(read_only(*sim));  // no STOP (or any coil) written by the read-only check
    const std::string name = "iad-check-fallback-test.json";
    const auto fallback = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(fallback);
    const std::string missing =
        (std::filesystem::temp_directory_path() / "iad-no-such-dir" / "deeper" / name).string();
    std::ostringstream out, err;
    const std::string where = save_check_report(report, missing, out, err);
    CHECK(contains(where, "fallback"));
    CHECK(std::filesystem::is_regular_file(fallback));
    CHECK(contains(err.str(), "could not write"));
    std::filesystem::remove(fallback);
    CHECK_EQ(format_check_item(report.items.front()).substr(0, 10), "PASS  link");
}
