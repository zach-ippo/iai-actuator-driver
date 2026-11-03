// Model unit: the JSON form of the result types. Field names follow the Python edition's to_dict() so the outputs of the
// editions compare; values the unit did not give are null.
#include <string>
#include <vector>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/results.hpp"
#include "iai_actuator_driver/status.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

const Json& get(const Json& object, const std::string& key) {
    const Json* value = object.find(key);
    if (value == nullptr) throw std::logic_error("missing key " + key);
    return *value;
}

bool has(const Json& object, const std::string& key) { return object.find(key) != nullptr; }

bool is_object(const Json& value) { return value.type() == Json::Type::Object; }

std::vector<std::string> keys_of(const Json& object) {
    std::vector<std::string> keys;
    for (const auto& kv : object.as_object()) keys.push_back(kv.first);
    return keys;
}

std::vector<std::uint16_t> status_words(std::size_t count) {
    std::vector<std::uint16_t> w(count, 0);
    w[1] = 7395;                                                               // PNOW 73.95 mm
    w[2] = 0;                                                                  // ALMC
    w[3] = 0x0001;                                                             // DIPM
    w[4] = 0x0005;                                                             // DOPM
    w[5] = reg::kDss1Pwr | reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Pend;     // DSS1
    w[6] = reg::kDss2Enbs;                                                     // DSS2
    w[7] = reg::kDssePmss | reg::kDsseRmds | reg::kDssePsns;                   // DSSE
    if (count > 9) w[9] = 1;                                                   // STAT MPOW
    if (count > 13) w[13] = 464;                                               // CNOW
    if (count > 17) w[17] = 5000;                                              // STIM
    if (count > 19) w[19] = reg::kZonsLs1;                                     // ZONS
    if (count > 20) w[20] = 3;                                                 // POSS
    if (count > 21) w[21] = reg::kSsseRtc;                                     // SSSE
    return w;
}

}  // namespace

// ------------------------------------------------------------------------------------------ Status

TEST(model_json_status_has_the_python_keys) {
    const Json j = to_json(Status::from_registers(status_words(22)));
    for (const char* key :
         {"position_mm", "alarm_code", "servo_on", "homed", "in_position", "moving", "alarm", "modbus_enabled",
          "emergency_stop", "velocity_mm_s", "current_ma", "raw", "warning", "alarm_level", "move_bit", "homing", "paused",
          "target_reached", "push_missed", "pushing", "push_contact", "safety_speed", "at_backward_end", "at_forward_end",
          "at_middle", "vnow_mm_s", "idle", "ready", "position_complete_no", "cold_start_alarm", "rtc_in_use",
          "drive_power_cut", "emg_input", "enable_off", "auto_servo_off", "nv_busy", "manual_mode", "battery_low",
          "excitation_done"})
        CHECK(has(j, key));
    CHECK_NEAR(get(j, "position_mm").as_number(), 73.95, 1e-9);
    CHECK_EQ(get(j, "alarm_code").as_int(), 0);
    CHECK(get(j, "servo_on").as_bool());
    CHECK(get(j, "homed").as_bool());
    CHECK(!get(j, "moving").as_bool());
    CHECK(!get(j, "alarm").as_bool());
    CHECK(get(j, "alarm_level").is_null());
    CHECK_EQ(get(j, "current_ma").as_int(), 464);
    CHECK(get(j, "at_forward_end").as_bool());
    CHECK(!get(j, "at_backward_end").as_bool());
    CHECK_EQ(get(j, "position_complete_no").as_int(), 3);
    CHECK(get(j, "rtc_in_use").as_bool());
    CHECK(get(j, "idle").as_bool());
    CHECK(get(j, "ready").as_bool());
    // the raw words by name
    const Json& raw = get(j, "raw");
    CHECK_EQ(keys_of(raw), (std::vector<std::string>{"PNOW", "ALMC", "DIPM", "DOPM", "DSS1", "DSS2", "DSSE", "STAT", "VNOW",
                                                     "CNOW", "DEVI", "STIM", "SIPM", "ZONS", "POSS", "SSSE"}));
    CHECK_EQ(get(raw, "PNOW").as_int(), 7395);
    CHECK_EQ(get(raw, "STIM").as_int(), 5000);
    CHECK_EQ(get(raw, "SSSE").as_int(), reg::kSsseRtc);
    // it survives a trip through the text form
    CHECK(Json::parse(j.dump()) == j);
    CHECK(Json::parse(j.dump(2)) == j);
}

TEST(model_json_status_short_reads_give_null_flags_and_fewer_raw_words) {
    const Json j = to_json(Status::from_registers(status_words(8)));
    CHECK(get(j, "current_ma").is_null());
    CHECK(get(j, "at_forward_end").is_null());
    CHECK(get(j, "position_complete_no").is_null());
    CHECK(get(j, "cold_start_alarm").is_null());
    CHECK(get(j, "nv_busy").is_null());
    CHECK(get(j, "vnow_mm_s").is_null());
    CHECK_NEAR(get(j, "velocity_mm_s").as_number(), 0.0, 1e-12);   // not moving: 0, never a guess while moving
    CHECK_EQ(keys_of(get(j, "raw")), (std::vector<std::string>{"PNOW", "ALMC", "DIPM", "DOPM", "DSS1", "DSS2", "DSSE"}));
    // a fault carries its level key
    std::vector<std::uint16_t> w = status_words(22);
    w[2] = 0x090;
    const Json fault = to_json(Status::from_registers(w));
    CHECK(get(fault, "alarm").as_bool());
    CHECK_EQ(get(fault, "alarm_level").as_string(), "operation_cancelled");
    w[2] = 0x04E;
    CHECK(get(to_json(Status::from_registers(w)), "warning").as_bool());
    CHECK_EQ(get(to_json(Status::from_registers(w)), "alarm_level").as_string(), "message");
}

// ------------------------------------------------------------------------------------------ alarms

TEST(model_json_alarm_record_and_info) {
    AlarmRecord r;
    r.index = 1;
    r.code = 0x0C1;
    r.detail = 0;
    r.address = 0xFFFF;
    r.time_ms = 2324745;
    const Json j = to_json(r);
    CHECK_EQ(get(j, "index").as_int(), 1);
    CHECK_EQ(get(j, "code").as_int(), 0x0C1);
    CHECK_EQ(get(j, "name").as_string(), "Servo error");
    CHECK_EQ(get(j, "address").as_int(), 0xFFFF);
    CHECK_EQ(get(j, "time_ms").as_int(), 2324745);
    CHECK_EQ(get(j, "time_raw").as_int(), 2324745);
    CHECK_EQ(get(j, "time_after_power_on").as_string(), "00:38:44.745");
    CHECK(!get(j, "power_up").as_bool());
    CHECK_EQ(get(j, "level").as_string(), "operation_cancelled");
    r.time_ms = std::nullopt;
    CHECK(get(to_json(r), "time_ms").is_null());
    CHECK(get(to_json(r), "time_after_power_on").is_null());

    const Json info = to_json(alarm_info(0x0EE, true));
    CHECK_EQ(get(info, "hex").as_string(), "0x0EE");
    CHECK_EQ(get(info, "level").as_string(), "operation_cancelled");
    CHECK(get(info, "fault").as_bool());
    CHECK(!get(info, "warning").as_bool());
    CHECK_EQ(get(info, "group").as_string(), "D");
    CHECK_EQ(get(info, "home_needed").as_string(), "home lost: alarm reset, then a home return");
    CHECK(has(info, "remedy"));
    CHECK(has(info, "detail_text"));
    CHECK(has(info, "level_text"));
    CHECK(has(info, "group_name"));
}

// ------------------------------------------------------------------------------------------ identity and limits

TEST(model_json_device_info_keys) {
    DeviceInfo d;
    d.slave = 3;
    d.system_code = 0xEC;
    d.system_name = "EC 24 V";
    d.run_status = 0xFF;
    d.unit_code = 0x0A;
    d.unit_name = "EC(MS) \"2-size EC\"";
    d.firmware = "0002";
    d.slave_id_raw = {0xEC, 0xFF, 0x0A, 0x00, 0x02};
    d.model_name = "EC-GRBP8M-20-1-1-PN-WA-WL2";
    d.user_name = "EC-GRBP8M-20";
    d.serial_number = "SIM-0000002";
    d.soft_limits_mm = Span{-0.30, 9.80};
    d.max_speed_mm_s = 53.0;
    d.max_accel_g = 0.30;
    d.actuator_kind = 4;
    d.actuator_kind_name = "gripper";
    d.units = "mm";
    d.accl = 0x0100;
    d.errors = {"identity read from CTCL (class 3 item 1) instead of FC11"};
    d.product_family = "EC";
    d.identity_source = "CTCL 0x0300";
    d.unit_known = true;
    d.axis_number = 2;
    d.max_positions = 3;
    IndexHeader header;
    header.model_code = 1;
    header.words_per_record = 8;
    header.record_count = 1;
    d.index_header = header;
    IndexCategory category;
    category.category_id = 7;
    category.data_address = 0x0700;
    category.member_count = 26;
    category.list_count = 1;
    category.member_size = 2;
    d.categories = {category};
    const Json j = to_json(d);
    CHECK_EQ(get(j, "slave").as_int(), 3);
    CHECK_EQ(get(j, "system_code").as_int(), 0xEC);
    CHECK_EQ(get(j, "slave_id_raw").as_string(), "ec ff 0a 00 02");
    CHECK_EQ(get(j, "model").as_string(), "EC-GRBP8M-20-1-1-PN-WA-WL2");
    CHECK_EQ(get(j, "controller").as_string(), "EC 24 V, unit 0x0A (EC(MS) \"2-size EC\")");
    CHECK_EQ(get(j, "family").as_string(), "EC(MS) \"2-size EC\"");
    CHECK_EQ(get(j, "short_model").as_string(), "EC-GRBP8M-20");
    CHECK_EQ(get(j, "actuator").as_string(), "gripper (AKID 4)");
    CHECK_EQ(get(j, "kind").as_string(), "gripper");
    CHECK(!get(j, "rotary").as_bool());
    CHECK_EQ(get(j, "extensibility_level").as_int(), 1);
    CHECK_EQ(get(j, "accl").as_int(), 0x0100);
    REQUIRE(get(j, "soft_limits_mm").size() == 2u);
    CHECK_NEAR(get(j, "soft_limits_mm").as_array()[1].as_number(), 9.80, 1e-9);
    CHECK_EQ(get(j, "errors").size(), 1u);
    CHECK_EQ(get(j, "identity_source").as_string(), "CTCL 0x0300");
    CHECK(get(j, "unit_known").as_bool());
    CHECK_EQ(get(j, "max_positions").as_int(), 3);
    CHECK_EQ(get(get(j, "index_header"), "record_count").as_int(), 1);
    REQUIRE(get(j, "categories").size() == 1u);
    CHECK_EQ(get(get(j, "categories").as_array()[0], "category_id").as_int(), 7);
    CHECK(get(j, "controller_name").is_null());
    // a bare DeviceInfo: nulls, no throw
    const Json bare = to_json(DeviceInfo());
    CHECK(get(bare, "system_code").is_null());
    CHECK(get(bare, "slave_id_raw").is_null());
    CHECK(get(bare, "soft_limits_mm").is_null());
    CHECK_EQ(get(bare, "controller").as_string(), "unknown");
    CHECK_EQ(get(bare, "kind").as_string(), "unknown");
    CHECK(Json::parse(j.dump()) == j);
}

TEST(model_json_limits_ranges_and_span) {
    CHECK(to_json(Span{1.5, 9.0}) == Json::parse("[1.5,9]"));
    AxisLimits l;
    l.max_speed_mm_s = 800.0;
    l.actuator_kind = 3;
    l.max_push_pct = 70.0;
    const Json lj = to_json(l);
    CHECK_NEAR(get(lj, "max_speed_mm_s").as_number(), 800.0, 1e-9);
    CHECK_EQ(get(lj, "kind").as_string(), "rod");
    CHECK_EQ(get(lj, "units").as_string(), "mm");
    CHECK(get(lj, "push_available").as_bool());
    CHECK(get(lj, "max_accel_g").is_null());
    CHECK(!get(lj, "min_push_from_ec").as_bool());
    CHECK_NEAR(get(lj, "min_push_pct").as_number(), 20.0, 1e-9);

    // a range leaves out the keys that do not apply to it
    LimitRange r;
    r.min = 0.01;
    r.max = 800.0;
    r.unit = "mm/s";
    r.source = "MAXV item 7.4";
    Json rj = to_json(r);
    CHECK_EQ(keys_of(rj), (std::vector<std::string>{"min", "max", "unit", "source"}));
    r.unit_max = 800.0;
    rj = to_json(r);
    CHECK_EQ(keys_of(rj), (std::vector<std::string>{"min", "max", "unit", "source", "unit_max", "configured"}));
    CHECK(get(rj, "configured").is_null());
    r.available = true;
    r.recommended_min = 20.0;
    r.count = 3;
    rj = to_json(r);
    CHECK(get(rj, "available").as_bool());
    CHECK_NEAR(get(rj, "recommended_min").as_number(), 20.0, 1e-9);
    CHECK_EQ(get(rj, "count").as_int(), 3);

    AxisRanges ranges;
    ranges.units = "deg";
    ranges.kind = "rotary";
    const Json aj = to_json(ranges);
    CHECK_EQ(keys_of(aj), (std::vector<std::string>{"units", "kind", "family", "position", "speed", "accel", "decel",
                                                    "accel_decel", "push", "band", "push_band", "stored_rows", "jog_step",
                                                    "speed_override"}));
    CHECK_EQ(get(aj, "units").as_string(), "deg");
    CHECK_EQ(get(aj, "family").as_string(), reg::kGenericFamily);
    CHECK(get(get(aj, "position"), "min").is_null());
}

// ------------------------------------------------------------------------------------------ position entry

TEST(model_json_position_entry) {
    PositionEntry e = PositionEntry::from_registers({0x0000, 0x0BB8, 0x0000, 0x2EE0, 0x0000, 0x0002, 0x0000, 0x0000,
                                                     0x0000, 0x0000, 0x0032, 0x0032, 0x000A, 0x0000, 0x0016, 0x00A9});
    const Json j = to_json(e);
    CHECK_EQ(keys_of(j), (std::vector<std::string>{"position_mm", "band_mm", "speed_mm_s", "zone_plus_mm", "zone_minus_mm",
                                                    "accel_g", "decel_g", "push_power", "load_threshold", "control_flags",
                                                    "checksum", "checksum_ok"}));
    CHECK_NEAR(get(j, "position_mm").as_number(), 30.0, 1e-9);
    CHECK_EQ(get(j, "push_power").as_int(), 10);
    CHECK_EQ(get(j, "control_flags").as_int(), 0x16);
    CHECK_EQ(get(j, "checksum").as_int(), 0xA9);
    CHECK(get(j, "checksum_ok").as_bool());
    e.checksum = std::nullopt;
    CHECK(get(to_json(e), "checksum").is_null());
    CHECK(get(to_json(e), "checksum_ok").is_null());
}

// ------------------------------------------------------------------------------------------ parameters

TEST(model_json_param_value_info_and_model) {
    CHECK(to_json(ParamValue::of(2.5)) == Json(2.5));
    CHECK(to_json(ParamValue::parse("1:Forward")) == Json("1:Forward"));
    CHECK(to_json(ParamValue()).is_null());
    CHECK(to_json(ParamValue::parse("7")) == Json(7.0));

    ParameterInfo p;
    p.number = 5;
    p.name = "Smooth accel/decel setting";
    p.item = "8.14";
    p.address = "0x081A";
    p.value = ParamValue::of(0);
    p.text = "0:Disable";
    p.choices = {{"0", "0:Disable"}, {"1", "1:Enable"}};
    p.writable = true;
    p.raw = "00000000";
    const Json pj = to_json(p);
    CHECK_EQ(get(pj, "number").as_int(), 5);
    CHECK_EQ(get(pj, "item").as_string(), "8.14");
    CHECK_EQ(get(pj, "address").as_string(), "0x081A");
    CHECK_EQ(get(pj, "text").as_string(), "0:Disable");
    CHECK_EQ(get(get(pj, "choices"), "1").as_string(), "1:Enable");
    CHECK(get(pj, "writable").as_bool());
    CHECK(!get(pj, "dangerous").as_bool());
    CHECK_EQ(get(pj, "family").as_string(), "EC");
    CHECK(get(pj, "symbol").is_null());
    CHECK(get(pj, "min").is_null());
    CHECK(get(pj, "unsigned").is_null());

    ParameterModel m;
    m.unit_code = 0;
    m.system_code = 0xEC;
    m.model_name = "EC-R6S-150-1-PN";
    m.family = "EC standard";
    m.known = true;
    m.candidates = {1, 2, 7};
    m.listed = {1, 2};
    m.hidden = {{7, "shown only with a wireless module (SUP2 CNWL)"}};
    m.verified = true;
    m.seen_on = {"EC-R6S-"};  // kept for compatibility (the driver leaves it empty); still serialized
    m.note = "the parameters IA-OS lists for unit 0x00 (EC standard)";
    const Json mj = to_json(m);
    CHECK_EQ(get(mj, "family").as_string(), "EC standard");
    CHECK(get(mj, "known").as_bool());
    CHECK(get(mj, "verified").as_bool());
    CHECK_EQ(get(mj, "candidates").size(), 3u);
    CHECK_EQ(get(mj, "listed").size(), 2u);
    REQUIRE(get(mj, "hidden").size() == 1u);
    CHECK_EQ(get(get(mj, "hidden").as_array()[0], "no").as_int(), 7);
    CHECK_EQ(get(get(mj, "hidden").as_array()[0], "reason").as_string(), "shown only with a wireless module (SUP2 CNWL)");
    CHECK_EQ(get(mj, "seen_on").as_array()[0].as_string(), "EC-R6S-");
}

TEST(model_json_parameter_write_result_saved_needs_reset_takes_effect) {
    ParameterWriteResult r;
    r.number = 2;
    r.name = "LS signal detection range adjustment";
    r.key = "8.9";
    r.address = "0x0810";
    r.unit = "mm";
    r.before = "0.10 mm";
    r.after = "0.20 mm";
    r.value = ParamValue::of(0.2);
    r.raw_before = "0000000A";
    r.raw_after = "00000014";
    r.changed = true;
    r.verified = true;
    r.committed = true;
    r.commit_confirmed = true;
    r.listed_for_model = true;
    r.verified_for_model = true;
    r.warnings = {"a warning"};
    r.note = "committed";
    Json j = to_json(r);
    CHECK_EQ(keys_of(j), (std::vector<std::string>{"no", "name", "key", "address", "unit", "before", "after", "value",
                                                   "raw_before", "raw_after", "changed", "verified", "committed",
                                                   "listed_for_model", "verified_for_model", "home_parameter", "warnings",
                                                   "note", "commit_confirmed", "skipped", "forced", "saved", "needs_reset",
                                                   "takes_effect"}));
    CHECK_EQ(get(j, "no").as_int(), 2);
    CHECK_EQ(get(j, "key").as_string(), "8.9");
    CHECK_NEAR(get(j, "value").as_number(), 0.2, 1e-9);
    CHECK(get(j, "saved").as_bool());                // committed and the commit was seen
    CHECK(get(j, "needs_reset").as_bool());          // always: a parameter applies after a software reset
    CHECK_EQ(get(j, "takes_effect").as_string(), "after a software reset (software_reset()) or a power cycle");
    CHECK_EQ(get(j, "warnings").as_array()[0].as_string(), "a warning");
    CHECK(!get(j, "skipped").as_bool());

    // committed but never seen acting on the coil
    r.commit_confirmed = false;
    j = to_json(r);
    CHECK(!get(j, "saved").as_bool());
    CHECK(get(j, "needs_reset").as_bool());
    CHECK(get(j, "takes_effect").as_string().find("IF the commit took - it was NOT confirmed") != std::string::npos);

    // RAM only
    r.committed = false;
    j = to_json(r);
    CHECK(!get(j, "saved").as_bool());
    CHECK(get(j, "takes_effect").as_string().find("RAM only: lost at the next power-up") == 0);

    // skipped by the non-volatile write guard: nothing was written
    r.skipped = true;
    r.changed = false;
    r.note = kNvUnchangedNote;
    j = to_json(r);
    CHECK(get(j, "skipped").as_bool());
    CHECK_EQ(get(j, "note").as_string(), "unchanged: nothing written (one non-volatile write cycle saved)");
    CHECK_EQ(get(j, "takes_effect").as_string(), "unchanged: the unit already holds this value");
    r.skipped = false;
    r.forced = true;
    CHECK(get(to_json(r), "forced").as_bool());
}

TEST(model_json_position_write_and_software_reset_results) {
    PositionWriteResult w;
    w.row = 2;
    w.skipped = true;
    w.note = kNvUnchangedNote;
    w.warnings = {"row 2 is the middle point"};
    const Json wj = to_json(w);
    CHECK_EQ(get(wj, "row").as_int(), 2);
    CHECK(get(wj, "skipped").as_bool());
    CHECK(!get(wj, "forced").as_bool());
    CHECK_EQ(get(wj, "note").as_string(), kNvUnchangedNote);
    CHECK(is_object(get(wj, "entry")));
    CHECK_EQ(get(wj, "warnings").size(), 1u);

    SoftwareResetResult s;
    s.accepted = true;
    s.baud_rate = 38400;
    s.status = Status::from_registers(status_words(22));
    s.uptime_before_s = 12.5;
    s.elapsed_s = 1.25;
    s.notes = {"a note"};
    s.confirmed = true;
    s.confirmed_by = "PMSS 1 -> 0";
    const Json sj = to_json(s);
    CHECK(get(sj, "accepted").as_bool());
    CHECK(!get(sj, "reply_lost").as_bool());
    CHECK_EQ(get(sj, "baudrate").as_int(), 38400);
    CHECK(get(sj, "modbus_enabled").as_bool());
    CHECK(get(sj, "servo_on").as_bool());
    CHECK(get(sj, "homed").as_bool());
    CHECK(!get(sj, "needs_home").as_bool());
    CHECK_EQ(get(sj, "alarm_code").as_int(), 0);
    CHECK_NEAR(get(sj, "uptime_before_s").as_number(), 12.5, 1e-9);
    CHECK(get(sj, "uptime_after_s").is_null());
    CHECK_EQ(get(sj, "confirmed_by").as_string(), "PMSS 1 -> 0");
    CHECK(is_object(get(sj, "status")));
    CHECK_NEAR(get(get(sj, "status"), "position_mm").as_number(), 73.95, 1e-9);
}

// ------------------------------------------------------------------------------------------ bus-side records

TEST(model_json_baud_port_scan_and_serial_records) {
    BaudChangeResult b;
    b.previous = 38400;
    b.baud_rate = 115200;
    b.method = BaudChangeMethod::Break;
    b.code = 20;
    b.frame = "00 45 00 04 00 14 00 05";
    b.verified = true;
    const Json bj = to_json(b);
    CHECK_EQ(get(bj, "previous").as_int(), 38400);
    CHECK_EQ(get(bj, "baudrate").as_int(), 115200);
    CHECK_EQ(get(bj, "method").as_string(), "break");
    CHECK_EQ(get(bj, "code").as_int(), 20);
    CHECK_EQ(get(bj, "frame").as_string(), "00 45 00 04 00 14 00 05");
    CHECK(get(bj, "verified").as_bool());
    CHECK(!get(bj, "probed").as_bool());
    CHECK(get(bj, "code_verified").as_bool());
    CHECK(has(bj, "note"));
    b.method = BaudChangeMethod::Direct;
    CHECK_EQ(get(to_json(b), "method").as_string(), "direct");

    PortBaudResult p;
    p.port = "COM3";
    p.rate = 115200;
    p.change = b;
    p.answered = {{1, true}, {2, false}};
    const Json pj = to_json(p);
    CHECK_EQ(get(pj, "port").as_string(), "COM3");
    CHECK_EQ(get(pj, "rate").as_int(), 115200);
    CHECK_EQ(get(get(pj, "change"), "code").as_int(), 20);
    CHECK(get(get(pj, "answered"), "1").as_bool());
    CHECK(!get(get(pj, "answered"), "2").as_bool());

    ScanHit h;
    h.port = "COM3";
    h.slave = 2;
    h.slave_id_raw = {0xEC, 0xFF, 0x0A, 0x00, 0x02};
    h.system_code = 0xEC;
    h.unit_code = 0x0A;
    h.firmware = "0002";
    const Json hj = to_json(h);
    CHECK_EQ(get(hj, "name").as_string(), "COM3:2");
    CHECK_EQ(get(hj, "axis_number").as_int(), 1);
    CHECK_EQ(get(hj, "product_family").as_string(), "EC");
    CHECK_EQ(get(hj, "system_name").as_string(), "EC 24 V");
    CHECK_EQ(get(hj, "family").as_string(), "EC(MS) \"2-size EC\"");
    CHECK_EQ(get(hj, "slave_id_raw").as_string(), "ec ff 0a 00 02");
    CHECK_EQ(get(hj, "firmware").as_string(), "0002");
    CHECK(get(hj, "error").is_null());
    ScanHit silent;
    silent.port = "COM4";
    silent.slave = 5;
    silent.error = "not answered";
    const Json sj = to_json(silent);
    CHECK(get(sj, "system_code").is_null());
    CHECK(get(sj, "product_family").is_null());
    CHECK(get(sj, "slave_id_raw").is_null());
    CHECK(get(sj, "firmware").is_null());
    CHECK_EQ(get(sj, "error").as_string(), "not answered");

    SerialPortInfo s;
    s.port = "COM3";
    s.description = "IAI USB to UART Bridge Controller";
    s.vid = 0x10C4;
    s.pid = 0x81D7;
    const Json spj = to_json(s);
    CHECK_EQ(get(spj, "port").as_string(), "COM3");
    CHECK(get(spj, "is_iai").as_bool());
    CHECK_EQ(get(spj, "vid").as_int(), 0x10C4);
    CHECK(!get(to_json(SerialPortInfo()), "is_iai").as_bool());
    CHECK(get(to_json(SerialPortInfo()), "vid").is_null());
}

TEST(model_json_a_whole_float_keeps_its_decimal_point) {
    // (0.7) as Python's json.dumps: 1.0 stays a float for the reader (Python, C#), an integer stays an integer
    CHECK_EQ(Json(1.0).dump(), std::string("1.0"));
    CHECK_EQ(Json(150.0).dump(), std::string("150.0"));
    CHECK_EQ(Json(-2.0).dump(), std::string("-2.0"));
    CHECK_EQ(Json(0.0).dump(), std::string("0.0"));
    CHECK_EQ(Json(0.25).dump(), std::string("0.25"));
    CHECK_EQ(Json(1e20).dump(), std::string("1e+20"));
    CHECK_EQ(Json(3).dump(), std::string("3"));
    CHECK_EQ(Json(std::vector<double>{1.0, 2.5}).dump(), std::string("[1.0,2.5]"));
    CHECK(Json::parse("1.0").type() == Json::Type::Number);
    CHECK_EQ(Json::parse("[1.0,7]").dump(), std::string("[1.0,7]"));
    // as_whole: an integer, or a whole float inside the long long range; nothing else (no undefined narrowing)
    CHECK(Json(9600.0).as_whole() == std::optional<long long>(9600));
    CHECK(Json(42).as_whole() == std::optional<long long>(42));
    CHECK(!Json(9600.5).as_whole());
    CHECK(!Json(1e300).as_whole());
    CHECK(!Json(-1e19).as_whole());
    CHECK(!Json("1").as_whole());
    CHECK(!Json().as_whole());
    CHECK_EQ(Json(1e300).as_int(), 9223372036854775807LL);  // held at the range, not undefined
    CHECK_EQ(Json(-1e300).as_int(), -9223372036854775807LL - 1);
    CHECK_EQ(Json(-7.9).as_int(), -7LL);
}
