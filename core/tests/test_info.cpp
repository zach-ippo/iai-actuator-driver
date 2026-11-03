// Unit info and backup (info.hpp): the bit decoders, position mode, unit_info on the simulator profiles (the slider
// and gripper baselines read live by the Python edition), sections and error handling, the live decode of a status
// poll, the read-only backup in the Python edition's schema (every profile, row selections, files, several axes) and
// the backup readers a recovery session takes its input from. Ported from the Python edition's tests/test_info.py,
// tests/test_parameter_rules.py (BackupTest, the CON range check) and tests/test_fast_recovery.py
// (session_from_backup input). Simulator only (no COM port).
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_hooks.hpp"

using namespace iai_actuator_driver;

namespace {

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile, double scale = 50.0) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim, AxisOptions axis = {}) {
    OpenOptions o;
    o.slave = sim->slave();
    axis.poll_interval_s = 0.005;
    o.axis = axis;
    return Axis::open(sim, o);
}

/// The function codes the simulator received.
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

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

const Json& at(const Json& object, const std::string& key) {
    const Json* v = object.find(key);
    if (v == nullptr) throw std::out_of_range("no key '" + key + "'");
    return *v;
}

std::vector<std::string> strings(const Json& array) {
    std::vector<std::string> out;
    for (const Json& j : array.as_array()) out.push_back(j.as_string());
    return out;
}

/// The element of an array of objects whose `key` equals `value`.
const Json& where(const Json& array, const std::string& key, const Json& value) {
    for (const Json& j : array.as_array()) {
        const Json* v = j.find(key);
        if (v != nullptr && *v == value) return j;
    }
    throw std::out_of_range("no element with " + key + " = " + value.dump());
}

std::vector<std::string> errors_of(const Json& data) { return strings(at(data, "errors")); }

std::string temp_dir(const std::string& name) {
    const auto path = std::filesystem::temp_directory_path() / ("iad_info_test_" + name);
    std::filesystem::remove_all(path);
    std::filesystem::create_directories(path);
    return path.string();
}

/// Every FC03 the simulator saw lies inside one directory block's certain extent, an ID map or the directory.
void check_reads_inside_directory(Axis& ax, const SimulatedUnit& sim) {
    const DeviceInfo dev = ax.identify();
    const UnitDirectory dir(dev.index_header, dev.categories);
    std::vector<std::pair<int, int>> allowed = {{0, 8 + 8 * dir.header()->record_count}};
    for (const IndexCategory& rec : dir.records()) {
        allowed.emplace_back(rec.data_address, rec.data_address + rec.extent().words);
        if (rec.id_list_address) allowed.emplace_back(rec.id_list_address, rec.id_list_address + 2 * rec.member_count);
    }
    int reads = 0;
    for (const auto& frame : sim.requests()) {
        if (frame.size() < 6 || frame[1] != 0x03) continue;
        ++reads;
        const int start = (frame[2] << 8) | frame[3];
        const int count = (frame[4] << 8) | frame[5];
        bool inside = false;
        for (const auto& a : allowed)
            if (a.first <= start && start + count <= a.second) inside = true;
        if (!inside) CHECK(inside && "an FC03 outside every directory block");
    }
    CHECK(reads > 20);
}

}  // namespace

// ======================================================================================= decoders

TEST(info_decode_bits_dss1_after_a_move) {
    const Json word = info::decode_bits(0x7099, info::bit_table("DSS1"));  // live: idle after a move, SFTY on
    CHECK(strings(at(word, "on")) == std::vector<std::string>({"SFTY", "PWR", "SV", "BKRL", "HEND", "PEND", "MEND"}));
    CHECK_EQ(at(word, "hex").as_string(), "0x7099");
    const auto& bits = at(word, "bits").as_array();
    CHECK_EQ(at(bits[0], "bit").as_int(), 15);  // high bit first
    CHECK_EQ(at(bits[1], "bit").as_int(), 14);
    CHECK_EQ(at(bits[2], "bit").as_int(), 13);
    const Json& pend = where(at(word, "bits"), "name", Json("PEND"));
    CHECK(at(pend, "on").as_bool() && at(pend, "confirmed").as_bool());
    CHECK_EQ(at(pend, "evidence").as_string(), "VERIFIED-LIVE");
    const Json& emgs = where(at(word, "bits"), "name", Json("EMGS"));
    CHECK(!at(emgs, "on").as_bool() && !at(emgs, "confirmed").as_bool());
    CHECK_EQ(at(word, "unknown_set").size(), 0u);
}

TEST(info_decode_bits_table_by_name_and_unknown_bits) {
    const Json word = info::decode_bits(0x4043, "ZONS");  // b6 is not named in the ZONS table
    CHECK(strings(at(word, "on")) == std::vector<std::string>({"LS2", "Z2", "Z1", "b6"}));
    CHECK_EQ(at(word, "unknown_set").as_array().at(0).as_int(), 6);
    CHECK_EQ(at(at(word, "bits").as_array().back(), "evidence").as_string(), "not named");
    CHECK_THROWS(info::decode_bits(0x10000, "DSS1"), std::invalid_argument);
    CHECK_THROWS(info::decode_bits(0, "NOPE"), std::out_of_range);
}

TEST(info_decode_bits_dsse_push_contact_and_stat_32_bit) {
    const auto dsse = strings(at(info::decode_bits(0x37C3, "DSSE"), "on"));  // live: holding after contact
    CHECK(std::find(dsse.begin(), dsse.end(), "PHDT") != dsse.end());
    CHECK(std::find(dsse.begin(), dsse.end(), "PMSS") != dsse.end());
    const Json stat = info::decode_bits(0x0008801F, info::bit_table("STAT"), 32);  // live gripper STAT
    CHECK_EQ(at(stat, "hex").as_string(), "0x0008801F");
    CHECK(strings(at(stat, "on")) == std::vector<std::string>({"SFD", "CARS", "RMDS", "HEND", "SV", "SON", "MPOW"}));
    for (const Json& b : at(stat, "bits").as_array()) CHECK(!at(b, "confirmed").as_bool());  // names only
}

TEST(info_dopm_layouts) {
    const info::IoTables three = info::io_bit_tables(std::string("3pos"));
    const info::IoTables two = info::io_bit_tables(std::string("2pos"));
    auto on = [](std::uint32_t w, const std::vector<info::BitDef>& t) { return strings(at(info::decode_bits(w, t), "on")); };
    CHECK(on(0x000C, *three.dopm) == std::vector<std::string>({"*ALM", "LS2"}));  // slider at 75.00 mm
    CHECK(on(0x0009, *three.dopm) == std::vector<std::string>({"*ALM", "LS0"}));  // slider at 0.00 mm
    CHECK(on(0x0006, *two.dopm) == std::vector<std::string>({"*ALM", "LS1"}));    // gripper at 9.50 mm
    CHECK(on(0x0005, *two.dopm) == std::vector<std::string>({"*ALM", "LS0"}));    // gripper open
    const Json decoded = info::decode_bits(0x0004, *two.dopm);
    const Json& alm = where(at(decoded, "bits"), "name", Json("*ALM"));
    CHECK(at(alm, "active_low").as_bool() && at(alm, "on").as_bool());  // 1 = no alarm
    CHECK(on(0x0004, *three.dopm) == std::vector<std::string>({"LS2"}));  // same word, other layout
    for (const auto* table : {three.dipm, &info::bit_table("DIPM_SINGLE")})
        for (const info::BitDef& b : *table) CHECK(!b.confirmed);  // no PIO wired on the slider: unconfirmed
    // the gripper's inputs pressed by hand (2026-10-30): ST0 = bit 0 and ST1 = bit 1 seen, RES (bit 2) not yet
    CHECK_EQ(two.dipm->at(0).name, "RES");
    CHECK(!two.dipm->at(0).confirmed);
    CHECK_EQ(two.dipm->at(1).name, "ST1");
    CHECK(two.dipm->at(1).confirmed);
    CHECK_EQ(two.dipm->at(2).name, "ST0");
    CHECK(two.dipm->at(2).confirmed);
    CHECK_EQ(three.dipm->at(0).name, "RES");
    CHECK_EQ(three.dipm->at(1).name, "ST1");
    CHECK_EQ(three.dipm->at(2).name, "ST0");
    CHECK_EQ(three.dipm->at(0).bit, 3);  // 3-position: RES moves to IN3
    CHECK_EQ(two.dipm->at(0).bit, 2);
}

TEST(info_position_mode) {
    Json mode = info::position_mode(4u);
    CHECK_EQ(at(mode, "name").as_string(), "3-position");
    CHECK_EQ(at(mode, "positions").as_int(), 3);
    CHECK_EQ(at(mode, "layout").as_string(), "3pos");
    CHECK(!at(mode, "confirmed").as_bool());
    mode = info::position_mode(0u);
    CHECK_EQ(at(mode, "layout").as_string(), "2pos");
    CHECK(at(mode, "confirmed").as_bool());
    CHECK(at(mode, "name").as_string().find("double") != std::string::npos);
    const Json single = info::position_mode(3u);
    CHECK(at(single, "name").as_string().find("single") != std::string::npos);
    CHECK(info::io_bit_tables(single).dipm == &info::bit_table("DIPM_SINGLE"));
    CHECK_EQ(at(info::position_mode(std::nullopt, std::string("MWB-EC-DSRR4-WA-MF")), "layout").as_string(), "3pos");
    CHECK_EQ(at(info::position_mode(std::nullopt, std::string("EC-GRBP8M-20-1-1-PN-WA-WL2")), "source").as_string(),
             "assumed");
    CHECK(at(info::position_mode(7u), "note").as_string().find("unknown user parameter 9 value 7") != std::string::npos);
}

TEST(info_alarm_time_and_rtc) {
    CHECK_EQ(at(info::alarm_time(951515, false), "time_after_power_on").as_string(), "00:15:51.515");
    CHECK_EQ(at(info::alarm_time(951515, std::nullopt), "clock").as_string(), "power_on");
    const Json rtc = info::alarm_time(86400 + 3661, true);  // DOC: seconds since 2000-01-01 (UNVERIFIED on hardware)
    CHECK_EQ(at(rtc, "occurred_at").as_string(), "2000-01-02 01:01:01");
    CHECK(at(rtc, "time_after_power_on").is_null());
    CHECK(at(info::alarm_time(std::nullopt, true), "clock").is_null());
    CHECK(info::rtc_in_use(0x0100u).value());
    CHECK(!info::rtc_in_use(0x1008u).value());
    CHECK(!info::rtc_in_use(std::nullopt).has_value());
}

// ======================================================================================= unit_info: slider

TEST(info_unit_info_slider_complete_read_only) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    sim->clear_requests();
    const Json data = info::unit_info(ax);
    CHECK(errors_of(data).empty());
    for (const auto& e : errors_of(data)) std::printf("    error: %s\n", e.c_str());
    CHECK_EQ(at(at(data, "meta"), "schema").as_string(), "ec_driver.unit_info/1");
    CHECK(strings(at(at(data, "meta"), "sections")) == info::sections());
    CHECK(Json::parse(data.dump()) == data);
    CHECK_EQ(at(at(data, "identity"), "model").as_string(), "MWB-EC-DSRR4-WA-MF");
    CHECK(read_only(*sim));
    CHECK_EQ(sim->nonvolatile_writes(), 0);
    check_reads_inside_directory(ax, *sim);
}

TEST(info_unit_info_slider_identity) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const Json ident = at(info::unit_info(ax, std::vector<std::string>{"identity"}), "identity");
    CHECK_EQ(at(ident, "serial_number").as_string(), "SIM-0000001");  // placeholder, never the real serial
    CHECK_EQ(at(ident, "unit_code").as_int(), 0x02);
    CHECK_EQ(at(ident, "firmware").as_string(), "0013");
    CHECK_EQ(at(ident, "slave").as_int(), 1);
    CHECK(at(ident, "soft_limits_mm") == Json(std::vector<double>{-0.3, 150.3}));
    CHECK(at(ident, "stroke_mm") == Json(std::vector<double>{0.0, 150.0}));
    CHECK_NEAR(at(ident, "ec_max_speed_mm_s").as_number(), 700.0, 1e-9);
    CHECK_NEAR(at(ident, "speed_cap_mm_s").as_number(), 700.0, 1e-9);  // cap = MAXV
    CHECK(at(ident, "baudrate").is_null());  // simulator: no serial port
    CHECK_EQ(at(at(ident, "position_mode"), "layout").as_string(), "3pos");
    CHECK_EQ(at(at(ident, "directory"), "record_count").as_int(), 27);
    CHECK_EQ(at(at(ident, "identification_extra"), "date_guess").as_string(), "2025-09-26");
    const Json& caps = at(ident, "capabilities");
    CHECK_EQ(at(at(caps, "collision"), "reported").as_string(), "not reported (MVPT b13 CDAS = 0)");
    CHECK_EQ(at(at(caps, "link"), "boot_baudrate").as_int(), 38400);
    // the directory is summarised, not listed raw
    CHECK(ident.find("categories") == nullptr && ident.find("index_header") == nullptr && ident.find("errors") == nullptr);
}

TEST(info_unit_info_slider_parameters) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const Json params = at(info::unit_info(ax, std::vector<std::string>{"parameters"}), "parameters");
    std::vector<std::string> names;
    for (const Json& c : at(params, "classes").as_array()) names.push_back(at(c, "class").as_string());
    CHECK(names == std::vector<std::string>({"0x0003", "0x0004", "0x0007", "0x0008", "0x0009", "0x000C"}));
    const Json& c8 = where(at(params, "classes"), "class", Json("0x0008"));
    CHECK_EQ(at(c8, "items").size(), 39u);
    const Json& p14 = where(at(c8, "items"), "key", Json("8.14"));
    CHECK(strings(at(p14, "params")) == std::vector<std::string>({"P5", "P6"}));
    const Json& speed = where(at(c8, "items"), "key", Json("8.10"));
    CHECK_NEAR(at(speed, "scaled").as_number(), 700.0, 1e-9);
    CHECK(at(speed, "note").as_string().find("default command speed") != std::string::npos);
    const Json& c7 = where(at(params, "classes"), "class", Json("0x0007"));
    const Json& maxv = where(at(c7, "items"), "key", Json("7.4"));
    CHECK_NEAR(at(maxv, "scaled").as_number(), 700.0, 1e-9);
    CHECK_EQ(at(maxv, "address").as_string(), "0x0706");
    CHECK(at(maxv, "note").as_string().find("MAXV") != std::string::npos);
    const Json& unknown = where(at(c8, "items"), "key", Json("8.105"));  // unmapped items are still listed raw
    CHECK_EQ(at(unknown, "params").size(), 0u);
    const Json& p3 = where(at(params, "named"), "no", Json(3));
    CHECK_EQ(at(p3, "text").as_string(), "1:Forward");
    const Json& p9 = where(at(params, "named"), "no", Json(9));
    CHECK_EQ(at(p9, "text").as_string(), "4:3-position");
    CHECK_EQ(at(p9, "category").as_string(), "A");
    CHECK(!at(p9, "iaos_only").as_bool());
    CHECK(at(where(at(params, "named"), "no", Json(13)), "iaos_only").as_bool());
    CHECK(at(params, "range_check").is_null());  // an EC: no CON range table
    CHECK(at(at(params, "model"), "known").as_bool());
}

TEST(info_unit_info_slider_io_status_monitor) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const Json data = info::unit_info(ax, std::vector<std::string>{"monitor"});
    const Json& io = at(data, "io");
    CHECK_EQ(at(io, "layout").as_string(), "3pos");
    std::vector<std::string> dopm;
    for (const Json& b : at(at(io, "DOPM"), "bits").as_array()) dopm.push_back(at(b, "name").as_string());
    CHECK(dopm == std::vector<std::string>({"*ALM", "LS2", "LS1", "LS0"}));
    CHECK(strings(at(at(io, "DOPM"), "on")) == std::vector<std::string>({"*ALM"}));  // 73.95 mm: no marker, no alarm
    Json lamps = Json::object();
    for (const char* k : {"backward_end", "forward_end", "middle", "alarm", "warning"}) lamps.set(k, false);
    CHECK(at(io, "lamps") == lamps);
    const Json& words = at(data, "status_words");
    CHECK(strings(at(at(words, "DSS1"), "on")) == std::vector<std::string>({"PWR", "SV", "BKRL", "HEND", "PEND"}));
    const auto dsse = strings(at(at(words, "DSSE"), "on"));
    CHECK(std::find(dsse.begin(), dsse.end(), "PMSS") != dsse.end());
    CHECK_EQ(at(at(words, "SSSE"), "value").as_int(), 0x1008);
    CHECK_EQ(at(at(words, "STAT"), "hex").as_string(), "0x0000801F");
    const Json& mon = at(data, "monitor");
    CHECK_NEAR(at(mon, "position_mm").as_number(), 73.95, 1e-9);
    CHECK_EQ(at(mon, "alarm_code").as_int(), 0);
    CHECK_EQ(at(mon, "overload_level").as_int(), 18);
    CHECK(mon.find("uptime_ms") != nullptr);
    const Json& control = at(data, "control_words");
    CHECK(strings(at(at(control, "DRG1"), "on")) == std::vector<std::string>({"SON"}));
    CHECK(strings(at(at(control, "DRGE"), "on")) == std::vector<std::string>({"PMSL"}));
}

TEST(info_unit_info_slider_alarms) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const Json alarms = at(info::unit_info(ax, std::vector<std::string>{"alarms"}), "alarms");
    CHECK_EQ(at(alarms, "slots").as_int(), 32);
    CHECK_EQ(at(alarms, "alarm_count").as_int(), 5);
    CHECK_EQ(at(alarms, "power_up_markers").as_int(), 4);
    CHECK_EQ(at(alarms, "empty_slots").as_int(), 23);
    std::vector<std::string> names;
    for (const Json& a : at(alarms, "history").as_array()) names.push_back(at(a, "name").as_string());
    CHECK(names == std::vector<std::string>({"ABS error (2)", "ABS error (2)", "Servo error", "Servo error", "Servo error"}));
    const Json& first = at(alarms, "history").as_array().at(0);
    CHECK_EQ(at(first, "index").as_int(), 1);
    CHECK_EQ(at(first, "hex").as_string(), "0x0EE");
    CHECK_EQ(at(first, "detail").as_int(), 3);
    CHECK_EQ(at(first, "time_raw").as_int(), 0x000E84DB);
    CHECK_EQ(at(first, "newer_power_ups").as_int(), 1);
    // UNVERIFIED: the time stamp is ms since power-on; 0x0EE is operation cancelled on an absolute (WA) unit
    CHECK_EQ(at(first, "time_ms").as_int(), 951515);
    CHECK_EQ(at(first, "time_after_power_on").as_string(), "00:15:51.515");
    CHECK_EQ(at(first, "level").as_string(), "operation_cancelled");
    CHECK_EQ(at(first, "group").as_string(), "D");
    const Json& servo = at(alarms, "history").as_array().at(2);
    CHECK_EQ(at(servo, "hex").as_string(), "0x0C1");
    CHECK_EQ(at(servo, "level").as_string(), "operation_cancelled");
    CHECK_EQ(at(servo, "group").as_string(), "A");
    const Json& cur = at(alarms, "current");
    CHECK_EQ(at(cur, "code").as_int(), 0);
    CHECK(!at(cur, "active").as_bool());
    CHECK(at(cur, "level").is_null());
    CHECK(!at(cur, "fault").as_bool() && !at(cur, "warning").as_bool());
    CHECK_EQ(at(cur, "class3_code").as_int(), 0);
    CHECK(at(at(alarms, "notes"), "order").as_string().find("newest") != std::string::npos);
}

TEST(info_unit_info_slider_maintenance_and_positions) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const Json data = info::unit_info(ax, std::vector<std::string>{"maintenance", "positions"});
    CHECK(errors_of(data).empty());
    const Json& maint = at(data, "maintenance");
    CHECK_EQ(at(maint, "total_moves").as_int(), 101);
    CHECK_EQ(at(maint, "distance_m").as_int(), 2);
    const Json& temp = where(at(maint, "items"), "symbol", Json("TEMP"));
    CHECK_EQ(at(temp, "address").as_string(), "0x840E");
    CHECK_EQ(at(temp, "hi16").as_int(), 39);
    CHECK_EQ(at(temp, "lo16").as_int(), 48);
    CHECK(!at(temp, "confirmed").as_bool());
    CHECK(at(where(at(maint, "items"), "id", Json(10)), "unset").as_bool());
    const Json& pos = at(data, "positions");
    const auto& rows = at(pos, "rows").as_array();
    REQUIRE(rows.size() == 3);
    const char* roles[] = {"backward end", "forward end", "middle"};
    const double positions[] = {0.0, 150.0, 75.0};
    const double speeds[] = {108.5, 248.5, 248.5};
    for (int i = 0; i < 3; ++i) {
        CHECK_EQ(at(rows[i], "role").as_string(), roles[i]);
        CHECK_NEAR(at(rows[i], "position_mm").as_number(), positions[i], 1e-9);
        CHECK_NEAR(at(rows[i], "speed_mm_s").as_number(), speeds[i], 1e-9);
        CHECK(at(rows[i], "checksum_ok").as_bool());
    }
    CHECK(strings(at(rows[1], "ctlf_on")) == std::vector<std::string>({"GSL0"}));  // the unknown 0x0010 role bit
    const auto& simple = at(pos, "simple_data_rows").as_array();
    REQUIRE(simple.size() == 5);
    const int numbers[] = {260, 261, 262, 263, 266};  // 3-position: the Middle % is 266
    const int pct[][3] = {{16, 30, 30}, {71, 60, 60}, {71, 60, 60}, {71, 60, 60}, {71, 60, 60}};
    for (int i = 0; i < 5; ++i) {
        CHECK_EQ(at(simple[i], "n").as_int(), numbers[i]);
        CHECK_EQ(at(simple[i], "speed_pct").as_int(), pct[i][0]);
        CHECK_EQ(at(simple[i], "accel_pct").as_int(), pct[i][1]);
        CHECK_EQ(at(simple[i], "decel_pct").as_int(), pct[i][2]);
        CHECK(at(simple[i], "checksum_ok").as_bool());
    }
    CHECK_EQ(at(simple[3], "role").as_string(), "forward end, previous (IA-OS %)");  // 263 is not the Middle
    CHECK_EQ(at(simple[4], "role").as_string(), "middle, present (IA-OS %)");
    CHECK_NEAR(at(simple[4], "position_mm").as_number(), 75.0, 1e-9);
    const Json& dm = at(pos, "direct_move");
    CHECK_NEAR(at(dm, "target_mm").as_number(), 73.95, 1e-9);
    CHECK_NEAR(at(dm, "band_mm").as_number(), 0.1, 1e-9);
    CHECK_NEAR(at(dm, "speed_mm_s").as_number(), 700.0, 1e-9);
    CHECK_NEAR(at(dm, "accel_g").as_number(), 0.3, 1e-9);
    CHECK_EQ(at(at(pos, "active_profile"), "dpow").as_int(), 255);
}

// ======================================================================================= unit_info: gripper

TEST(info_unit_info_gripper) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    sim->clear_requests();
    const Json data = info::unit_info(ax);
    CHECK(errors_of(data).empty());
    for (const auto& e : errors_of(data)) std::printf("    error: %s\n", e.c_str());
    CHECK(read_only(*sim));
    const Json& ident = at(data, "identity");
    CHECK_EQ(at(ident, "model").as_string(), "EC-GRBP8M-20-1-1-PN-WA-WL2");
    CHECK_EQ(at(ident, "serial_number").as_string(), "SIM-0000002");
    CHECK_EQ(at(ident, "unit_code").as_int(), 0x0A);
    CHECK_EQ(at(ident, "firmware").as_string(), "0002");
    CHECK_EQ(at(at(ident, "directory"), "record_count").as_int(), 25);
    CHECK_EQ(at(at(ident, "position_mode"), "name").as_string(), "2-position (double solenoid)");
    const Json& named = at(at(data, "parameters"), "named");
    CHECK_EQ(at(where(named, "no", Json(3)), "text").as_string(), "1:Forward");
    CHECK_EQ(at(where(named, "no", Json(4)), "text").as_string(), "0.50 mm");
    CHECK_EQ(at(where(named, "no", Json(6)), "text").as_string(), "1:Enable");
    CHECK_EQ(at(where(named, "no", Json(7)), "text").as_string(), "0:Enable");
    const Json& io = at(data, "io");
    CHECK_EQ(at(io, "layout").as_string(), "2pos");
    CHECK(strings(at(at(io, "DOPM"), "on")) == std::vector<std::string>({"*ALM", "LS1"}));  // 9.50 mm (live 0x0006)
    CHECK(at(at(io, "lamps"), "forward_end").as_bool());
    std::vector<std::string> dipm;
    for (const Json& b : at(at(io, "DIPM"), "bits").as_array()) dipm.push_back(at(b, "name").as_string());
    CHECK(dipm == std::vector<std::string>({"RES", "ST1", "ST0"}));
    CHECK_EQ(at(at(data, "maintenance"), "total_moves").as_int(), 9752);
    CHECK_EQ(at(at(data, "maintenance"), "distance_m").as_int(), 88);
    const Json& alarms = at(data, "alarms");
    CHECK_EQ(at(alarms, "alarm_count").as_int(), 6);
    CHECK_EQ(at(alarms, "power_up_markers").as_int(), 4);
    for (const Json& a : at(alarms, "history").as_array()) CHECK_EQ(at(a, "name").as_string(), "Servo error");
    const auto& rows = at(at(data, "positions"), "rows").as_array();
    CHECK(at(rows[1], "push").as_bool());
    CHECK_NEAR(at(rows[1], "push_end_mm").as_number(), 9.5, 1e-9);
    CHECK_EQ(at(rows[1], "push_direction").as_string(), "away from home");
    CHECK_EQ(at(rows[2], "role").as_string(), "unused in 2-position mode");
    const auto& simple = at(at(data, "positions"), "simple_data_rows").as_array();
    CHECK_EQ(at(simple[3], "role").as_string(), "forward end, previous (IA-OS %)");
    CHECK_EQ(simple.size(), 4u);  // 2-position: no Middle row 266
    check_reads_inside_directory(ax, *sim);
}

TEST(info_unit_info_alarm_and_move_counter_follow_the_unit) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    MoveOptions o;
    o.speed = 20.0;
    ax.move_to(5.0, o);
    sim->trigger_alarm(0x0E0);
    const Json data = info::unit_info(ax, std::vector<std::string>{"alarms", "maintenance"});
    CHECK(at(data, "identity").is_null());
    CHECK_EQ(at(at(data, "maintenance"), "total_moves").as_int(), 9753);  // TLMC counts move starts
    const Json& cur = at(at(data, "alarms"), "current");
    CHECK_EQ(at(cur, "code").as_int(), 0x0E0);
    CHECK_EQ(at(cur, "name").as_string(), "Over load");
    CHECK(at(cur, "active").as_bool());
    CHECK_EQ(at(cur, "level").as_string(), "cold_start");
    CHECK_EQ(at(cur, "class3_code").as_int(), 0x0E0);
    CHECK(at(cur, "fault").as_bool() && !at(cur, "warning").as_bool());
    CHECK(at(cur, "reset").as_string().find("software reset") != std::string::npos);
    CHECK_EQ(at(cur, "group").as_string(), "A");
    const Json& newest = at(at(data, "alarms"), "history").as_array().at(0);
    CHECK_EQ(at(newest, "index").as_int(), 0);
    CHECK_EQ(at(newest, "code").as_int(), 0x0E0);
    CHECK_EQ(at(newest, "newer_power_ups").as_int(), 0);
}

// ======================================================================================= unit_info: edges

TEST(info_unit_info_sections) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    Json data = info::unit_info(ax, std::vector<std::string>{"maintenance"});
    CHECK(strings(at(at(data, "meta"), "sections")) == std::vector<std::string>({"maintenance"}));
    CHECK(at(data, "parameters").is_null());
    CHECK_EQ(at(at(data, "maintenance"), "total_moves").as_int(), 101);
    data = info::unit_info(ax, std::vector<std::string>{"monitor"});
    CHECK_EQ(at(at(data, "io"), "layout").as_string(), "3pos");  // P9 is read for the I/O layout
    CHECK(at(data, "alarms").is_null());
    CHECK_THROWS(info::unit_info(ax, std::vector<std::string>{"everything"}), std::invalid_argument);
}

TEST(info_unit_info_unreadable_block_is_reported_not_raised) {
    SimulatedUnitOptions o;
    o.time_scale = 50.0;
    o.soft_limits = false;  // class 7 (0x0700) answers exception 02
    auto sim = std::make_shared<SimulatedUnit>(o);
    Axis ax = open_sim(sim);
    const Json data = info::unit_info(ax);
    bool reported = false;
    for (const auto& e : errors_of(data))
        if (e.find("class 0x0007") != std::string::npos) reported = true;
    CHECK(reported);
    const Json& named = at(at(data, "parameters"), "named");
    CHECK(!at(where(named, "no", Json(1)), "found").as_bool());
    CHECK_EQ(at(where(named, "no", Json(2)), "text").as_string(), "0.10 mm");  // the other classes are still there
    CHECK(!at(data, "alarms").is_null());
}

TEST(info_unit_info_no_link_raises) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    sim->inject_fault(SimFault::Timeout, 1000);
    CHECK_THROWS(info::unit_info(ax), ModbusTimeout);
}

TEST(info_decode_monitor_from_a_status_poll) {
    // The GUI's live view: Status::raw (22 words) decodes like unit_info's monitor sections.
    struct Case {
        SimProfile profile;
        const char* layout;
        const char* dopm_b2;
        unsigned p9;
    };
    for (const Case& c : {Case{SimProfile::Slider, "3pos", "LS2", 4u}, Case{SimProfile::Gripper, "2pos", "*ALM", 0u}}) {
        auto sim = make_sim(c.profile);
        Axis ax = open_sim(sim);
        const Json data = info::unit_info(ax, std::vector<std::string>{"monitor"});
        const Json mode = info::position_mode(c.p9);
        sim->clear_requests();
        const Json live = info::decode_monitor(info::monitor_words(ax.status().raw), mode);
        CHECK(functions(*sim) == std::set<int>({0x03}));  // the status read only
        CHECK_EQ(live.size(), 3u);
        for (const char* name : {"DSS1", "DSS2", "DSSE", "ZONS", "STAT", "SIPM"})
            CHECK(at(at(at(live, "status_words"), name), "bits") == at(at(at(data, "status_words"), name), "bits"));
        CHECK(at(live, "status_words").find("SSSE") != nullptr);  // in the 22-word poll (0x9000-0x9015)
        CHECK(at(live, "status_words").find("POSS") != nullptr);
        CHECK_EQ(at(at(live, "io"), "layout").as_string(), c.layout);
        CHECK_EQ(at(where(at(at(at(live, "io"), "DOPM"), "bits"), "bit", Json(2)), "name").as_string(), c.dopm_b2);
        CHECK(at(at(live, "io"), "lamps") == at(at(data, "io"), "lamps"));
        CHECK_NEAR(at(at(live, "monitor"), "position_mm").as_number(), at(at(data, "monitor"), "position_mm").as_number(),
                   1e-9);
        CHECK(at(live, "monitor").find("uptime") != nullptr);
        CHECK(at(live, "monitor").find("overload_level") == nullptr);
    }
    const Json empty = info::decode_monitor(info::MonitorWords());
    CHECK_EQ(at(empty, "status_words").size(), 0u);
    CHECK_EQ(at(at(empty, "io"), "lamps").size(), 0u);
    CHECK_EQ(at(at(empty, "io"), "layout").as_string(), "2pos");
}

TEST(info_unit_info_pcon_range_check) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    const Json data = info::unit_info(ax, std::vector<std::string>{"identity", "parameters"});
    CHECK(Json::parse(data.dump()) == data);
    const Json& params = at(data, "parameters");
    const Json& check = at(params, "range_check");
    CHECK_EQ(at(check, "family").as_string(), "PCON");
    CHECK_EQ(at(check, "findings").size(), 0u);
    CHECK_NEAR(at(at(check, "min_band"), "mm").as_number(), 0.01, 1e-9);
    CHECK_EQ(at(params, "named").size(), 0u);  // the EC P-numbers do not apply to a CON controller
    const Json& no10 = where(at(params, "numbered"), "number", Json(10));
    CHECK_EQ(at(no10, "category").as_string(), "B");
    CHECK_NEAR(at(no10, "family_min").as_number(), 0.01, 1e-9);
    const Json& no88 = where(at(params, "numbered"), "number", Json(88));
    CHECK_EQ(at(no88, "category").as_string(), "D");
    CHECK(at(no88, "dangerous").as_bool());
}

// ======================================================================================= backup

TEST(info_backup_every_profile_round_trips_read_only) {
    for (SimProfile p : sim_profiles()) {
        auto sim = make_sim(p);
        Axis ax = open_sim(sim);
        sim->clear_requests();
        const Json data = info::backup(ax);
        CHECK(read_only(*sim));
        CHECK(Json::parse(data.dump(2)) == data);
        CHECK(errors_of(data).empty());
        for (const auto& e : errors_of(data)) std::printf("    %s: %s\n", to_string(p), e.c_str());
        CHECK_EQ(at(at(data, "meta"), "schema").as_string(), info::kBackupSchema);
        CHECK(at(at(data, "meta"), "read_only").as_bool());
        CHECK(at(at(data, "directory"), "records").size() > 0);
        CHECK(at(at(data, "parameters"), "classes").size() > 0);
        CHECK(at(at(data, "parameters"), "named").size() > 0);
        for (const Json& c : at(at(data, "parameters"), "classes").as_array())
            for (const Json& it : at(c, "items").as_array()) CHECK(it.find("address") && it.find("hex"));
        CHECK(at(at(data, "positions"), "rows").size() > 0);
        CHECK(!at(data, "alarms").is_null());
        CHECK(!at(data, "maintenance").is_null());
    }
}

TEST(info_backup_rows_used_all_and_listed) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    const Json used = at(info::backup(ax), "positions");
    const Json listed = at(info::backup(ax, info::BackupRows::list({5, 1, 2})), "positions");
    const Json everything = at(info::backup(ax, info::BackupRows::all()), "positions");
    const Json bad = info::backup(ax, info::BackupRows::list({99999}));
    std::vector<int> used_rows, listed_rows;
    for (const Json& r : at(used, "rows").as_array()) used_rows.push_back(static_cast<int>(at(r, "n").as_int()));
    for (const Json& r : at(listed, "rows").as_array()) listed_rows.push_back(static_cast<int>(at(r, "n").as_int()));
    std::vector<int> expected;
    for (int n = 0; n < 64; ++n) expected.push_back(n);
    CHECK(used_rows == expected);
    CHECK(listed_rows == std::vector<int>({1, 2, 5}));
    CHECK_EQ(at(listed, "selection").as_string(), "list");
    CHECK(at(at(info::backup(ax, info::BackupRows::list({5, 1, 2})), "meta"), "rows") == Json(std::vector<int>{1, 2, 5}));
    CHECK_EQ(static_cast<long long>(at(everything, "rows").size()), at(everything, "rows_in_table").as_int());
    bool value_error = false;
    for (const auto& e : errors_of(bad))
        if (e.rfind("positions: ValueError", 0) == 0) value_error = true;
    CHECK(value_error);
    const Json& row = at(used, "rows").as_array().at(1);
    CHECK_EQ(at(row, "words").size(), 16u);
    CHECK(at(row, "decoded").find("position_mm") != nullptr);
    auto slider = make_sim(SimProfile::Slider);
    Axis sx = open_sim(slider);
    std::vector<int> rows;
    const Json slider_backup = info::backup(sx);
    for (const Json& r : at(at(slider_backup, "positions"), "rows").as_array())
        rows.push_back(static_cast<int>(at(r, "n").as_int()));
    std::vector<int> slider_rows = {0, 1, 2};
    for (int n = reg::kSimpleDataFirstRow; n <= reg::kSimpleDataLastRow; ++n) slider_rows.push_back(n);
    CHECK(rows == slider_rows);
    CHECK_THROWS(info::BackupRows::parse("0,x"), std::invalid_argument);
    CHECK_THROWS(info::BackupRows::parse("4294967296"), std::invalid_argument);  // not wrapped to row 0
    CHECK_THROWS(info::BackupRows::parse("0,2147483648"), std::invalid_argument);
    CHECK(info::BackupRows::parse("0, 1,2").numbers == std::vector<int>({0, 1, 2}));
    CHECK_EQ(info::BackupRows::parse("all").selection, "all");
}

TEST(info_backup_files_and_several_axes) {
    const std::string dir = temp_dir("files");
    auto gripper = make_sim(SimProfile::Gripper);
    Axis gx = open_sim(gripper);
    const std::string file = (std::filesystem::path(dir) / "gripper.json").string();
    const Json data = info::backup(gx, info::BackupRows(), file);
    const Json saved = info::load_backup(file);
    CHECK(at(saved, "identity") == at(data, "identity"));
    CHECK_EQ(at(at(saved, "meta"), "path").as_string(), file);
    auto rod = make_sim(SimProfile::Rod);
    auto pcon = make_sim(SimProfile::Pcon);
    Axis ax1 = open_sim(rod);
    Axis ax2 = open_sim(pcon);
    const auto paths = info::backup_axes({{"rod/1", &ax1}, {"pcon", &ax2}},
                                         (std::filesystem::path(dir) / "cell").string());
    REQUIRE(paths.size() == 2);
    CHECK_EQ(paths[0].first, "rod/1");
    CHECK_EQ(paths[1].first, "pcon");
    CHECK(paths[0].second.size() > 10 && paths[0].second.substr(paths[0].second.size() - 10) == "rod_1.json");
    for (const auto& p : paths) CHECK_EQ(at(info::load_backup(p.second), "errors").size(), 0u);
    std::filesystem::remove_all(dir);
}

TEST(info_backup_is_the_input_of_a_session) {
    // recovery.session_from_backup: identity, rows and parameter items come back from the file (no I/O).
    const std::string dir = temp_dir("session");
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    const std::string file = (std::filesystem::path(dir) / "g.json").string();
    const Json data = info::backup(ax, info::BackupRows(), file);
    const Json loaded = info::load_backup(file);
    const info::BackupIdentity ident = info::backup_identity(loaded);
    CHECK_EQ(ident.model_name.value_or(""), at(at(data, "identity"), "model_name").as_string());
    CHECK_EQ(ident.serial_number.value_or(""), "SIM-0000002");
    CHECK_EQ(ident.system_code.value_or(0), 0xEC);
    CHECK_EQ(ident.unit_code.value_or(0), 0x0A);
    CHECK(ident.soft_limits_mm.has_value());
    CHECK_EQ(ident.units, "mm");
    const auto rows = info::backup_rows(loaded);
    REQUIRE(rows.count(1) == 1);
    CHECK(rows.at(1) == ax.read_position_table(1));
    const auto params = info::backup_parameters(loaded);
    CHECK(params.count("0x0008:8.14") == 1);
    CHECK(params.count("0x0007:7.4") == 1);
    CHECK_THROWS(info::backup_identity(Json::parse("{\"identity\": {}}")), std::invalid_argument);
    CHECK_THROWS(info::backup_identity(Json::parse("{\"identity\": null}")), std::invalid_argument);
    CHECK_THROWS(info::load_backup((std::filesystem::path(dir) / "missing.json").string()), std::ios_base::failure);
    std::filesystem::remove_all(dir);
}

TEST(info_unit_info_and_backup_end_at_the_next_request_after_cancel_waits) {
    // (0.7) Ctrl+C (Axis::cancel_waits; iad_axis_cancel) ends a long read promptly: the readers open the axis's cancel
    // scope, and no section takes WaitCancelledError for a read problem of its own
    SimulatedUnitOptions o;
    o.profile = SimProfile::Slider;
    o.time_scale = 50.0;
    const auto sim = std::make_shared<HookedSimUnit>(o);
    Axis ax = open_sim(sim);
    std::atomic<int> requests{0};
    int cancel_at = 5;
    sim->set_request_hook([&](const std::vector<std::uint8_t>&) {
        if (++requests == cancel_at) ax.cancel_waits();
        return SimHookAnswer{};
    });
    CHECK_THROWS(info::backup(ax, info::BackupRows::all()), WaitCancelledError);
    CHECK(requests.load() <= cancel_at + 1);  // the request on the wire is finished, nothing after it
    requests = 0;
    cancel_at = 12;
    CHECK_THROWS(info::unit_info(ax), WaitCancelledError);
    CHECK(requests.load() <= cancel_at + 1);
    CHECK(read_only(*sim));
    // the cancel was delivered: the next read runs to its end
    cancel_at = -1;
    const Json backup = info::backup(ax);
    CHECK(at(backup, "positions").type() == Json::Type::Object);
    CHECK(at(info::unit_info(ax, std::vector<std::string>{"identity"}), "identity").type() == Json::Type::Object);
}

TEST(info_capabilities_and_parameter_wording_follow_the_python_edition) {
    const auto sim = make_sim(SimProfile::Rod);
    Axis ax = open_sim(sim);
    const Json data = info::unit_info(ax, std::vector<std::string>{"identity", "parameters"});
    const Json& limits = at(at(at(data, "identity"), "capabilities"), "axis_limits");
    CHECK(limits.find("notes") == nullptr);  // AxisLimits.to_dict() has none
    CHECK(at(limits, "max_speed_mm_s").type() == Json::Type::Number);
    CHECK(has(at(limits, "max_speed_mm_s").dump(), "."));  // a float for the reader, as in the Python edition
    const Json& named = at(at(data, "parameters"), "named");
    REQUIRE(named.size() > 0);
    CHECK(has(at(named.as_array()[0], "evidence").as_string(), "EC-GRBP8M gripper (fw 0002): changed one at a time in "
                                                                "IA-OS"));
    CHECK(has(at(at(data, "parameters"), "model").dump(), "the parameters IA-OS lists for unit 0x00"));
}

TEST(info_backup_identity_and_rows_refuse_a_number_outside_the_int_range_and_a_wrong_type) {
    // (0.7) a double outside int was narrowed (undefined: x86 and ARM64 differ), a wrong type threw std::logic_error
    // ("not in this build")
    const Json fine = Json::parse("{\"identity\":{\"serial_number\":\"S1\",\"system_code\":236.0,\"unit_code\":10,"
                                  "\"soft_limits_mm\":[0,9.5]}}");
    const info::BackupIdentity id = info::backup_identity(fine);
    CHECK(id.system_code == std::optional<int>(236));
    CHECK(id.unit_code == std::optional<int>(10));
    for (const char* codes : {"\"system_code\":1e10", "\"system_code\":3e9", "\"system_code\":236.5",
                              "\"unit_code\":-1e30"}) {
        const Json odd = Json::parse(std::string("{\"identity\":{\"serial_number\":\"S1\",") + codes + "}}");
        const info::BackupIdentity got = info::backup_identity(odd);  // not taken: no value, never a narrowed one
        CHECK(!got.system_code && !got.unit_code);
    }
    CHECK_THROWS(info::backup_identity(Json::parse("{\"identity\":{\"serial_number\":\"S1\","
                                                   "\"soft_limits_mm\":[null,9.5]}}")),
                 std::invalid_argument);
    // a row number or a class number outside the int range is no row, no class
    const Json rows = Json::parse("{\"positions\":{\"rows\":[{\"n\":1e10,\"words\":[]},{\"n\":0.5,\"words\":[]}]},"
                                  "\"parameters\":{\"classes\":[{\"class_id\":4e9,\"items\":[]}]}}");
    CHECK(info::backup_rows(rows).empty());
    CHECK(info::backup_parameters(rows).empty());
    CHECK_THROWS(info::backup_rows(Json::parse("{\"positions\":{\"rows\":[{\"n\":0,\"words\":[7]}]}}")),
                 std::invalid_argument);
}
