// Model unit: the monitor block decode (Status), the fault / warning / unknown-level alarm rule, alarm names, levels,
// groups and texts, and the alarm history decode. Ported from the Python edition's status / alarm tests; the rules marked
// VERIFIED there (ALMH servo off for 0x090, 0x0EE on an absolute unit) are VERIFIED in the Python edition, this edition
// is UNVERIFIED on hardware.
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/status.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

namespace {

/// A monitor block of `count` words (8, 14, 20 or 22): position in counts, the alarm code, DSS1 / DSS2 / DSSE and the rest.
struct Block {
    std::int32_t pnow = 7395;  // 73.95 mm
    int almc = 0;
    std::uint16_t dss1 = reg::kDss1Pwr | reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Pend | reg::kDss1Bkrl;
    std::uint16_t dss2 = reg::kDss2Enbs;
    std::uint16_t dsse = reg::kDssePmss | reg::kDsseParw | reg::kDssePsns | reg::kDsseRmds;
    std::uint32_t stat = reg::kStatMpow;
    std::int32_t vnow = 0;
    std::int32_t cnow = 464;
    std::uint32_t devi = 0;
    std::uint32_t stim = 123456;
    std::uint16_t zons = 0;
    std::uint16_t poss = 0;
    std::uint16_t ssse = 0;

    std::vector<std::uint16_t> words(std::size_t count = 22) const {
        std::vector<std::uint16_t> w(count, 0);
        // every store is bounded by `count`: a short block (the 7-word case) has no word 7 to write
        auto put16 = [&](std::size_t i, std::uint16_t v) {
            if (i < count) w[i] = v;
        };
        auto put32 = [&](std::size_t i, std::uint32_t v) {
            if (i + 1 < count) {
                w[i] = static_cast<std::uint16_t>(v >> 16);
                w[i + 1] = static_cast<std::uint16_t>(v & 0xFFFF);
            }
        };
        put32(0, static_cast<std::uint32_t>(pnow));
        put16(2, static_cast<std::uint16_t>(almc));
        put16(3, 0x0001);  // DIPM
        put16(4, 0x0005);  // DOPM
        put16(5, dss1);
        put16(6, dss2);
        put16(7, dsse);
        put32(8, stat);
        put32(10, static_cast<std::uint32_t>(vnow));
        put32(12, static_cast<std::uint32_t>(cnow));
        put32(14, devi);
        put32(16, stim);
        put16(18, 0);
        put16(19, zons);
        put16(20, poss);
        put16(21, ssse);
        return w;
    }
};

}  // namespace

// ------------------------------------------------------------------------------------------ from_registers

TEST(model_status_decodes_the_full_22_word_block) {
    Block b;
    b.zons = reg::kZonsLs1;
    b.poss = 0x0002;
    b.ssse = reg::kSsseRtc;
    const Status st = Status::from_registers(b.words(22));
    CHECK_NEAR(st.position_mm, 73.95, 1e-9);
    CHECK_EQ(st.alarm_code, 0);
    CHECK(st.servo_on);
    CHECK(st.homed);
    CHECK(st.in_position);
    CHECK(!st.moving);
    CHECK(!st.alarm);
    CHECK(st.modbus_enabled);
    CHECK(!st.emergency_stop);
    REQUIRE(st.velocity_mm_s.has_value());
    CHECK_NEAR(*st.velocity_mm_s, 0.0, 1e-9);
    REQUIRE(st.current_ma.has_value());
    CHECK_EQ(*st.current_ma, 464);
    CHECK_EQ(st.raw.words, 22);
    CHECK_EQ(st.raw.pnow, 7395u);
    CHECK_EQ(st.raw.dipm, 1);
    CHECK_EQ(st.raw.dopm, 5);
    CHECK_EQ(st.raw.dss1, b.dss1);
    CHECK_EQ(*st.raw.stat, 1u);
    CHECK_EQ(*st.raw.stim, 123456u);
    CHECK_EQ(*st.raw.zons, reg::kZonsLs1);
    CHECK_EQ(*st.raw.poss, 2);
    CHECK_EQ(*st.raw.ssse, reg::kSsseRtc);
    CHECK(*st.at_forward_end());
    CHECK(!*st.at_backward_end());
    CHECK(!*st.at_middle());
    CHECK_EQ(*st.position_complete_no(), 2);
    CHECK(*st.rtc_in_use());
    CHECK(!*st.cold_start_alarm());
    CHECK_NEAR(*st.uptime_s(), 123.456, 1e-9);
    CHECK_NEAR(*st.vnow_mm_s(), 0.0, 1e-9);
    CHECK(st.idle());
    CHECK(st.ready());
}

TEST(model_status_every_word_count_decodes_what_it_has) {
    const Block b;
    const Status s8 = Status::from_registers(b.words(8));
    CHECK_EQ(s8.raw.words, 8);
    CHECK(!s8.current_ma.has_value());
    CHECK(!s8.raw.stat.has_value());
    CHECK(!s8.raw.vnow.has_value());
    CHECK(!s8.vnow_mm_s().has_value());
    CHECK(!s8.uptime_s().has_value());
    // not moving: the velocity is 0 even without VNOW
    REQUIRE(s8.velocity_mm_s.has_value());
    CHECK_NEAR(*s8.velocity_mm_s, 0.0, 1e-9);

    const Status s14 = Status::from_registers(b.words(14));
    CHECK_EQ(s14.raw.words, 14);
    REQUIRE(s14.raw.stat.has_value());
    CHECK(s14.raw.cnow.has_value());
    CHECK(!s14.raw.devi.has_value());
    CHECK(!s14.raw.stim.has_value());
    CHECK(!s14.raw.zons.has_value());
    CHECK_EQ(*s14.current_ma, 464);

    const Status s20 = Status::from_registers(b.words(20));
    CHECK_EQ(s20.raw.words, 20);
    CHECK(s20.raw.devi.has_value());
    CHECK(s20.raw.stim.has_value());
    CHECK(s20.raw.sipm.has_value());
    CHECK(s20.raw.zons.has_value());
    CHECK(!s20.raw.poss.has_value());
    CHECK(!s20.raw.ssse.has_value());

    // a count between the sizes decodes what is complete
    CHECK_EQ(Status::from_registers(b.words(17)).raw.words, 14);
    CHECK_EQ(Status::from_registers(b.words(21)).raw.words, 20);
    CHECK_EQ(Status::from_registers(b.words(30)).raw.words, 22);
    // fewer than 8 words is not a status block
    CHECK_THROWS(Status::from_registers(b.words(7)), std::invalid_argument);
    CHECK_THROWS(Status::from_registers({}), std::invalid_argument);
}

TEST(model_status_condition_flags_are_nullopt_for_short_reads) {
    const Block b;
    const Status s8 = Status::from_registers(b.words(8));
    CHECK(!s8.at_backward_end().has_value());
    CHECK(!s8.at_forward_end().has_value());
    CHECK(!s8.at_middle().has_value());
    CHECK(!s8.position_complete_no().has_value());
    CHECK(!s8.cold_start_alarm().has_value());
    CHECK(!s8.rtc_in_use().has_value());
    CHECK(!s8.auto_servo_off().has_value());
    CHECK(!s8.nv_busy().has_value());
    CHECK(!s8.battery_low().has_value());
    // DSSE / DSS2 are always read, so these are known
    CHECK(s8.emg_input().has_value());
    CHECK(s8.enable_off().has_value());
    CHECK(s8.manual_mode().has_value());
    CHECK(s8.drive_power_cut().has_value());
    const Status s20 = Status::from_registers(b.words(20));
    CHECK(s20.at_middle().has_value());
    CHECK(!s20.position_complete_no().has_value());  // POSS and SSSE need 22 words
    CHECK(!s20.cold_start_alarm().has_value());
    CHECK(s20.nv_busy().has_value());                // STAT needs 14
}

TEST(model_status_negative_positions_and_signed_values) {
    Block b;
    b.pnow = -30;
    b.cnow = -150;
    b.vnow = -1250;
    b.dsse |= reg::kDsseMove;
    const Status st = Status::from_registers(b.words(22));
    CHECK_NEAR(st.position_mm, -0.30, 1e-9);
    CHECK_EQ(*st.current_ma, -150);
    CHECK(st.moving);
    CHECK_NEAR(*st.velocity_mm_s, -12.5, 1e-9);  // VNOW while moving (coarse on the EC)
    CHECK_NEAR(*st.vnow_mm_s(), -12.5, 1e-9);
    CHECK_EQ(st.raw.vnow.value(), static_cast<std::uint32_t>(-1250));
}

// ------------------------------------------------------------------------------------------ the moving rule

TEST(model_status_moving_rule_move_ghms_and_the_settled_states) {
    Block b;
    // servo on, homed, PEND = 0 and MOVE = 0: still moving (MOVE clears early while decelerating)
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend;
    CHECK(Status::from_registers(b.words()).moving);
    // the MOVE bit alone
    b = Block();
    b.dsse |= reg::kDsseMove;
    CHECK(Status::from_registers(b.words()).moving);
    CHECK(Status::from_registers(b.words()).move_bit());
    // a home return in progress (GHMS), also during the creep when MOVE is 0
    b = Block();
    b.dsse |= reg::kDsseGhms;
    Status st = Status::from_registers(b.words());
    CHECK(st.moving);
    CHECK(st.homing());
    CHECK(!st.move_bit());
    CHECK(!st.idle());
    CHECK(!st.ready());
    // servo off: not moving even without PEND
    b = Block();
    b.dss1 = reg::kDss1Hend;
    CHECK(!Status::from_registers(b.words()).moving);
    // not homed: PEND legitimately stays 0
    b = Block();
    b.dss1 = reg::kDss1Sv;
    CHECK(!Status::from_registers(b.words()).moving);
    // PSFL (push missed) and STP (pause) settle the axis
    b = Block();
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Psfl;
    st = Status::from_registers(b.words());
    CHECK(!st.moving);
    CHECK(st.push_missed());
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Stp;
    st = Status::from_registers(b.words());
    CHECK(!st.moving);
    CHECK(st.paused());
    CHECK(!st.idle());
    CHECK(!st.ready());
    // a fault settles it too
    b = Block();
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Almh;
    st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(!st.moving);
    CHECK(!st.ready());
}

TEST(model_status_a_warning_does_not_settle_the_axis) {
    Block b;
    b.almc = 0x04E;  // message level
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Alml;  // no PEND
    const Status st = Status::from_registers(b.words());
    CHECK(!st.alarm);
    CHECK(st.warning());
    CHECK(st.moving);  // it keeps running
}

// ------------------------------------------------------------------------------------------ the fault rule

TEST(model_status_fault_rule_operation_cancelled_and_cold_start) {
    Block b;
    b.almc = 0x090;  // software reset sent with the servo on: ALMH, servo off (VERIFIED in the Python edition)
    b.dss1 = reg::kDss1Hend | reg::kDss1Almh;
    Status st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(st.fault());
    CHECK(!st.warning());
    CHECK(!st.servo_on);
    REQUIRE(st.alarm_level().has_value());
    CHECK(*st.alarm_level() == AlarmLevel::OperationCancelled);
    // the code alone is enough, also without ALMH
    b.dss1 = reg::kDss1Sv | reg::kDss1Hend | reg::kDss1Pend;
    st = Status::from_registers(b.words());
    CHECK(st.alarm);
    // a cold-start code (0x0F4)
    b.almc = 0x0F4;
    st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(*st.alarm_level() == AlarmLevel::ColdStart);
    // ALMH without any code
    b = Block();
    b.dss1 |= reg::kDss1Almh;
    st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(!st.alarm_level().has_value());
}

TEST(model_status_0ee_is_operation_cancelled_unless_the_unit_is_incremental) {
    Block b;
    b.almc = 0x0EE;
    b.dss1 = reg::kDss1Hend | reg::kDss1Almh;
    const Status st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(*st.alarm_level() == AlarmLevel::OperationCancelled);   // absolute (WA) unit or unknown: VERIFIED on the WA gripper
    CHECK(alarm_level(0x0EE, true) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x0EE) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x0EE, false) == AlarmLevel::ColdStart);    // an incremental unit
    CHECK(alarm_is_fault(0x0EE));
}

TEST(model_status_message_level_codes_are_warnings_never_faults) {
    Block b;
    b.almc = 0x04E;  // total moving count exceeded (maintenance warning)
    Status st = Status::from_registers(b.words());
    CHECK(!st.alarm);
    CHECK(st.warning());
    CHECK(*st.alarm_level() == AlarmLevel::Message);
    CHECK(st.ready());  // a warning does not block a move
    // the warning bit without a code
    b = Block();
    b.dss1 |= reg::kDss1Alml;
    st = Status::from_registers(b.words());
    CHECK(!st.alarm);
    CHECK(st.warning());
    // PCON at power-up: real-time clock stop 0x069 is message level
    b = Block();
    b.almc = reg::kAlarmRtcStop;
    CHECK(!Status::from_registers(b.words()).alarm);
    // ALMH makes any code a fault
    b = Block();
    b.almc = 0x04E;
    b.dss1 |= reg::kDss1Almh;
    st = Status::from_registers(b.words());
    CHECK(st.alarm);
    CHECK(!st.warning());
}

TEST(model_status_unknown_codes_are_faults_unless_the_warning_bit_is_set) {
    Block b;
    b.almc = 0x123;   // no known level
    Status st = Status::from_registers(b.words());
    CHECK(st.alarm);  // fail safe
    CHECK(*st.alarm_level() == AlarmLevel::Unknown);
    b.dss1 |= reg::kDss1Alml;
    st = Status::from_registers(b.words());
    CHECK(!st.alarm);
    CHECK(st.warning());
    // a code of 0x011-0x07F that is not a named message code is unknown as well
    b = Block();
    b.almc = 0x030;
    CHECK(Status::from_registers(b.words()).alarm);
    // the power-up marker is not an alarm
    b = Block();
    b.almc = reg::kAlarmPowerUp;
    st = Status::from_registers(b.words());
    CHECK(!st.alarm);
    CHECK(*st.alarm_level() == AlarmLevel::None);
    CHECK(!st.warning());
}

// ------------------------------------------------------------------------------------------ derived flags

TEST(model_status_derived_push_and_mode_flags) {
    Block b;
    b.dsse |= reg::kDssePush | reg::kDssePushContact;
    Status st = Status::from_registers(b.words());
    CHECK(st.pushing());
    CHECK(st.push_contact());  // PEND and DSSE b0 in this one sample
    b.dss1 &= static_cast<std::uint16_t>(~reg::kDss1Pend);
    CHECK(!Status::from_registers(b.words()).push_contact());
    b = Block();
    b.dss1 |= reg::kDss1TargetReached | reg::kDss1Sfty | reg::kDss1Emgs;
    st = Status::from_registers(b.words());
    CHECK(st.target_reached());
    CHECK(st.safety_speed());
    CHECK(st.emergency_stop);
    CHECK(!st.ready());
    b = Block();
    b.dsse &= static_cast<std::uint16_t>(~reg::kDssePmss);  // PIO has control
    st = Status::from_registers(b.words());
    CHECK(!st.modbus_enabled);
    CHECK(!st.ready());
}

TEST(model_status_condition_flags_from_stat_dsse_dss2_ssse) {
    Block b;
    b.stat = reg::kStatMpow | reg::kStatAeep | reg::kStatAsof | reg::kStatBatl;
    b.dsse |= reg::kDsseEmgp;
    b.ssse = reg::kSsseAlmc;
    Status st = Status::from_registers(b.words());
    CHECK(*st.nv_busy());
    CHECK(*st.auto_servo_off());
    CHECK(*st.battery_low());
    CHECK(*st.emg_input());
    CHECK(*st.cold_start_alarm());
    CHECK(!*st.rtc_in_use());
    CHECK(*st.manual_mode());
    CHECK(st.excitation_done());
    CHECK(!*st.enable_off());
    CHECK(!*st.drive_power_cut());
    // drive power cut: STAT MPOW = 0 or DSSE MPUV
    b = Block();
    b.stat = 0;
    CHECK(*Status::from_registers(b.words()).drive_power_cut());
    b = Block();
    b.dsse |= reg::kDsseMpuv;
    CHECK(*Status::from_registers(b.words()).drive_power_cut());
    // the enable switch of a teaching tool released: DSS2 ENBS = 0
    b = Block();
    b.dss2 = 0;
    CHECK(*Status::from_registers(b.words()).enable_off());
    // the mode switch in AUTO: RMDS = 0
    b = Block();
    b.dsse &= static_cast<std::uint16_t>(~reg::kDsseRmds);
    CHECK(!*Status::from_registers(b.words()).manual_mode());
    // PSNS (excitation detection) not complete
    b = Block();
    b.dsse &= static_cast<std::uint16_t>(~reg::kDssePsns);
    CHECK(!Status::from_registers(b.words()).excitation_done());
}

TEST(model_status_servo_block_reasons) {
    Block b;
    CHECK(Status::from_registers(b.words()).servo_block_reasons().empty());
    b.dsse |= reg::kDsseEmgp;
    std::vector<std::string> reasons = Status::from_registers(b.words()).servo_block_reasons();
    REQUIRE(reasons.size() == 1u);
    CHECK(reasons[0].find("emergency-stop input") != std::string::npos);
    b.dsse |= reg::kDsseMpuv;
    b.dss2 = 0;  // enable switch released
    reasons = Status::from_registers(b.words()).servo_block_reasons();
    REQUIRE(reasons.size() == 3u);
    CHECK(reasons[1].find("motor drive power is cut") != std::string::npos);
    CHECK(reasons[2].find("enable switch") != std::string::npos);
    // STAT MPOW = 0 alone is no reason before a servo ON (UNVERIFIED on an EC)
    b = Block();
    b.stat = 0;
    CHECK(Status::from_registers(b.words()).servo_block_reasons().empty());
}

// ------------------------------------------------------------------------------------------ alarm names, levels, groups

TEST(model_alarm_names_curated_then_controller_then_hex) {
    CHECK_EQ(alarm_name(0x0C1), "Servo error");
    CHECK_EQ(alarm_name(0x0EE), "ABS error (2)");
    CHECK_EQ(alarm_name(0x0D9), "Software stroke limit over error");
    CHECK_EQ(alarm_name(reg::kAlarmPowerUp), "PowerUP (no error)");   // the driver's curated name wins
    CHECK_EQ(alarm_name(0x04E), "Total moving count is exceeded threshold.");
    CHECK_EQ(alarm_name(0x000), "No alarm (in normal condition)");
    CHECK_EQ(alarm_name(0x02C), "Monitoring data type change command during monitoring");  // CON-only name
    CHECK_EQ(alarm_name(0x7777), "alarm 0x7777");
    CHECK_EQ(alarm_name(0x001), "alarm 0x001");
    CHECK_EQ(alarm_name(0x0AB), "The instructed speed is abnormal");
}

TEST(model_alarm_levels) {
    CHECK(alarm_level(0) == AlarmLevel::None);
    CHECK(alarm_level(reg::kAlarmPowerUp) == AlarmLevel::None);
    CHECK(alarm_level(0x005) == AlarmLevel::ColdStart);          // 0x001-0x010
    CHECK(alarm_level(0x040) == AlarmLevel::Message);
    CHECK(alarm_level(0x048) == AlarmLevel::Message);
    CHECK(alarm_level(0x07F) == AlarmLevel::Message);
    CHECK(alarm_level(0x030) == AlarmLevel::Unknown);            // in 0x011-0x07F but not a named message code
    CHECK(alarm_level(0x02C) == AlarmLevel::Unknown);            // CON-only message code: no family known here
    CHECK(alarm_level(0x02C, std::nullopt, 0xAE) == AlarmLevel::Message);  // PCON system code: message level
    CHECK(alarm_level(0x02E, std::nullopt, 0xAA) == AlarmLevel::Message);  // SCON
    CHECK(alarm_level(0x02D, std::nullopt, 0xEC) == AlarmLevel::Unknown);  // EC
    CHECK(!alarm_is_fault(0x02D, false, false, 0xAC));
    CHECK(alarm_is_fault(0x02D, false, false));
    CHECK(alarm_info(0x02C, std::nullopt, std::nullopt, std::nullopt, 0xAE).warning);
    CHECK(alarm_level(0x080) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x090) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x0C1) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x0D9) == AlarmLevel::OperationCancelled);
    CHECK(alarm_level(0x0F1) == AlarmLevel::OperationCancelled);  // fieldbus link (DOC)
    CHECK(alarm_level(0x0CE) == AlarmLevel::ColdStart);          // conservative on every unit
    CHECK(alarm_level(0x0D2) == AlarmLevel::ColdStart);
    CHECK(alarm_level(0x0F4) == AlarmLevel::ColdStart);
    CHECK(alarm_level(0x0FF) == AlarmLevel::ColdStart);
    CHECK(alarm_level(0x100) == AlarmLevel::Unknown);            // outside the controller ranges
    CHECK(alarm_level(0x842) == AlarmLevel::Unknown);
    CHECK_EQ(std::string(level_key(AlarmLevel::None)), "none");
    CHECK_EQ(std::string(level_key(AlarmLevel::Message)), "message");
    CHECK_EQ(std::string(level_key(AlarmLevel::OperationCancelled)), "operation_cancelled");
    CHECK_EQ(std::string(level_key(AlarmLevel::ColdStart)), "cold_start");
    CHECK_EQ(std::string(level_key(AlarmLevel::Unknown)), "unknown");
    CHECK_EQ(level_text(AlarmLevel::OperationCancelled), "operation cancelled (stop, servo off; alarm reset)");
    CHECK_EQ(level_text(AlarmLevel::None), "no alarm");
}

TEST(model_alarm_message_codes_are_exactly_the_named_ones) {
    // the named codes of 0x011-0x07F are message level, every other code of the range has an unknown level (the name
    // table and the rule are kept in step); the CON-only codes 0x02C-0x02E are named but need the family to be messages
    int named = 0;
    for (int code = 0x011; code <= 0x07F; ++code) {
        const bool has_name = alarm_name(code).rfind("alarm 0x", 0) != 0;
        if (code == 0x02C || code == 0x02D || code == 0x02E) {
            CHECK(alarm_level(code) == AlarmLevel::Unknown);
            continue;
        }
        if (has_name) ++named;
        CHECK_EQ(alarm_level(code) == AlarmLevel::Message, has_name);
    }
    CHECK(named > 30);
}

TEST(model_alarm_fault_rule) {
    CHECK(!alarm_is_fault(0));
    CHECK(alarm_is_fault(0, true));                 // ALMH
    CHECK(!alarm_is_fault(0x04E));                  // message: never a fault by itself
    CHECK(alarm_is_fault(0x04E, true));
    CHECK(alarm_is_fault(0x090));                   // operation cancelled
    CHECK(alarm_is_fault(0x0F4));                   // cold start
    CHECK(alarm_is_fault(0x123));                   // unknown level: fail safe
    CHECK(!alarm_is_fault(0x123, false, true));     // unless ALML says it is a warning
    CHECK(alarm_is_fault(0x090, false, true));      // ALML does not excuse a known fault
    CHECK(!alarm_is_fault(reg::kAlarmPowerUp));
}

TEST(model_alarm_groups) {
    CHECK_EQ(alarm_group(0x048), "Warning");
    CHECK_EQ(alarm_group(0x085), "F");
    CHECK_EQ(alarm_group(0x0C1), "A");
    CHECK_EQ(alarm_group(0x0D8), "A");
    CHECK_EQ(alarm_group(0x0C8), "B");
    CHECK_EQ(alarm_group(0x0F0), "C");
    CHECK_EQ(alarm_group(0x0EE), "D");               // the PC tool showed 0x0EE as group D
    CHECK_EQ(alarm_group(0x0CE), "E");
    CHECK_EQ(alarm_group(0x090), "Other");
    CHECK_EQ(alarm_group(0x7777), "Other");
}

TEST(model_alarm_support_decides_the_encoder_type) {
    CHECK(!absolute_encoder_from_support(nullptr).has_value());
    std::map<std::string, std::uint32_t> empty;
    CHECK(!absolute_encoder_from_support(&empty).has_value());
    std::map<std::string, std::uint32_t> wa = {{"SUPT", 0x60002BD7u}, {"SUP2", 0x4384009Au}};
    CHECK(*absolute_encoder_from_support(&wa));      // the WA gripper: SUPT b13 ABSE set
    std::map<std::string, std::uint32_t> mabs = {{"SUPT", 0x60000BD7u}, {"SUP2", 0x00040000u}};
    CHECK(*absolute_encoder_from_support(&mabs));    // SUP2 b18 MABS
    std::map<std::string, std::uint32_t> incremental = {{"SUPT", 0x60000BD7u}, {"SUP2", 0x4300009Au}};
    CHECK(!*absolute_encoder_from_support(&incremental));
    std::map<std::string, std::uint32_t> only_sup2 = {{"SUP2", 0x4300009Au}};
    CHECK(!absolute_encoder_from_support(&only_sup2).has_value());  // cannot be told
}

TEST(model_alarm_info_texts) {
    const AlarmInfo info = alarm_info(0x090);
    CHECK_EQ(info.code, 0x090);
    CHECK_EQ(info.hex, "0x090");
    CHECK_EQ(info.name, "Software reset command while in SERVO-ON condition");
    CHECK(info.level == AlarmLevel::OperationCancelled);
    CHECK_EQ(info.level_text, "operation cancelled (stop, servo off; alarm reset)");
    CHECK_EQ(info.group, "Other");
    CHECK_EQ(info.group_name, "other");
    CHECK(info.fault);
    CHECK(!info.warning);
    CHECK_EQ(info.reset, "alarm reset, then servo on");
    CHECK_EQ(info.home_needed, "");
    CHECK(info.remedy.find("remove the cause, alarm reset, servo on. A software reset was sent with the servo on") != std::string::npos);

    const AlarmInfo warning = alarm_info(0x04E);
    CHECK(warning.warning);
    CHECK(!warning.fault);
    CHECK(warning.reset.find("maintenance warning 1") == 0);       // the EC's maintenance warnings say how each clears
    CHECK_EQ(warning.group, "Warning");
    CHECK_EQ(warning.group_name, "maintenance warning");
    const AlarmInfo plain_warning = alarm_info(0x069);
    CHECK_EQ(plain_warning.reset, "alarm reset (the axis keeps running meanwhile)");

    const AlarmInfo none = alarm_info(0);
    CHECK(none.level == AlarmLevel::None);
    CHECK(!none.fault);
    CHECK_EQ(none.reset, "");
    CHECK_EQ(none.remedy, "");

    const AlarmInfo unknown = alarm_info(0x123);
    CHECK(unknown.fault);
    CHECK(unknown.level == AlarmLevel::Unknown);
    CHECK(unknown.remedy.find("unknown alarm level") == 0);
    CHECK_EQ(unknown.name, "alarm 0x123");

    const AlarmInfo cold = alarm_info(0x0F4);
    CHECK(cold.level == AlarmLevel::ColdStart);
    CHECK_EQ(cold.home_needed, "incremental units: home return after the reset");
    CHECK(cold.reset.find("software reset (servo off first) or 24 V power cycle") == 0);
}

TEST(model_alarm_info_0ee_by_encoder_type) {
    const AlarmInfo unknown = alarm_info(0x0EE);
    CHECK(unknown.level == AlarmLevel::OperationCancelled);
    CHECK_EQ(unknown.level_text, "operation cancelled on absolute units / cold start on incremental units");
    CHECK(unknown.reset.find("absolute units: alarm reset, then servo on; incremental units:") == 0);
    CHECK(unknown.home_needed.find("home lost: a home return after the reset") == 0);
    CHECK_EQ(unknown.group, "D");
    CHECK_EQ(unknown.group_name, "controller-encoder link error");
    CHECK(unknown.remedy.find("Absolute position lost") != std::string::npos);

    const AlarmInfo absolute = alarm_info(0x0EE, true);
    CHECK(absolute.level == AlarmLevel::OperationCancelled);
    CHECK_EQ(absolute.reset, "alarm reset, then servo on");
    CHECK_EQ(absolute.home_needed, "home lost: alarm reset, then a home return");

    const AlarmInfo incremental = alarm_info(0x0EE, false);
    CHECK(incremental.level == AlarmLevel::ColdStart);
    CHECK_EQ(incremental.home_needed, "home lost: software reset (servo off first) or power cycle, then a home return");
}

TEST(model_alarm_info_detail_text) {
    CHECK_EQ(alarm_info(0x0A2, std::nullopt, 2, 0x1000 + 16 * 2 + 4).detail_text, "row 2, field speed");
    CHECK_EQ(alarm_info(0x0A2, std::nullopt, std::nullopt, 0x1000 + 16 * 3 + 0xA).detail_text, "row 3, field acceleration");
    CHECK_EQ(alarm_info(0x0A2, std::nullopt, 1, 0x1000 + 16 * 1 + 5).detail_text, "row 1, field +0x5");
    CHECK_EQ(alarm_info(0x0A3, std::nullopt, std::nullopt, 0x0F02).detail_text, "direct-move field speed (0x0F02)");
    CHECK_EQ(alarm_info(0x0A1, std::nullopt, std::nullopt, 0x0810).detail_text, "parameter at 0x0810");
    CHECK_EQ(alarm_info(0x0A1, std::nullopt, std::nullopt, 0xFFFF).detail_text, "");
    CHECK_EQ(alarm_info(0x0EE, std::nullopt, 1).detail_text, "detail 0001: absolute battery voltage low (ME0342-4H p.8-21)");
    CHECK_EQ(alarm_info(0x0EE, std::nullopt, 0x0007).detail_text, "");
    CHECK_EQ(alarm_info(0x0C1, std::nullopt, 5, 6).detail_text, "");
}

// ------------------------------------------------------------------------------------------ alarm history

namespace {

// The gripper's alarm history as read live: slot 0 the power-up marker, slot 1 servo error 0x0C1 at 2324745 ms, ...
const char* const kGripperHistory =
    "0000FFFF00000FFFFFFFFFFF0000FFFF000000C1002379090000FFFF00000FFFFFFFFFFF0000FFFF000000C103844A58"
    "0000FFFF000000C10383B4560000FFFF000000C1000964770000FFFF00000FFFFFFFFFFF0000FFFF000000C10003B375"
    "0000FFFF000000C1000355FD0000FFFF00000FFFFFFFFFFF";

}  // namespace

TEST(model_alarm_history_decode_as_read_on_hardware) {
    const std::vector<AlarmRecord> slots = alarm_history_slots(util::hex_words(kGripperHistory));
    REQUIRE(slots.size() == 10u);
    CHECK_EQ(slots[0].index, 0);
    CHECK_EQ(slots[0].code, 0x0FFF);
    CHECK(slots[0].power_up());
    CHECK(!slots[0].time_ms.has_value());           // FFFFFFFF
    CHECK_EQ(slots[0].detail, 0);
    CHECK_EQ(slots[0].address, 0xFFFF);
    CHECK_EQ(slots[0].name(), "PowerUP (no error)");
    CHECK(slots[0].level() == AlarmLevel::None);
    CHECK_EQ(slots[1].code, 0x0C1);
    CHECK(!slots[1].power_up());
    CHECK_EQ(*slots[1].time_ms, 2324745);
    CHECK_EQ(slots[1].name(), "Servo error");
    CHECK(slots[1].level() == AlarmLevel::OperationCancelled);
    CHECK_EQ(*slots[1].time_after_power_on(), "00:38:44.745");
    CHECK(!slots[0].time_after_power_on().has_value());
    CHECK_EQ(*slots[3].time_ms, 59001432);
    CHECK_EQ(*slots[3].time_after_power_on(), "16:23:21.432");
    CHECK_EQ(slots[9].index, 9);
}

TEST(model_alarm_history_ends_at_the_first_empty_slot) {
    // slot 0: code 0x0C1 at 1 s; slot 1 empty; slot 2 would be an alarm but is not read as one
    std::vector<std::uint16_t> words = {0x0000, 0xFFFF, 0x0000, 0x00C1, 0x0000, 0x03E8,   //
                                        0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000,   //
                                        0x0000, 0xFFFF, 0x0000, 0x0048, 0x0000, 0x0001};
    const std::vector<AlarmRecord> slots = alarm_history_slots(words);
    REQUIRE(slots.size() == 1u);
    CHECK_EQ(slots[0].code, 0x0C1);
    CHECK_EQ(*slots[0].time_ms, 1000);
    CHECK_EQ(*slots[0].time_after_power_on(), "00:00:01.000");
    const std::vector<AlarmRecord> all = alarm_history_slots(words, true);
    REQUIRE(all.size() == 3u);
    CHECK_EQ(all[1].code, 0);
    CHECK_EQ(all[2].code, 0x048);
    CHECK(all[0].time_ms.has_value());
    // a trailing partial slot is ignored; no words, no slots
    words.push_back(0x1234);
    CHECK_EQ(alarm_history_slots(words, true).size(), 3u);
    CHECK(alarm_history_slots({}).empty());
}

TEST(model_alarm_time_text_with_days) {
    AlarmRecord r;
    r.time_ms = 90061001;  // 1 d 01:01:01.001
    CHECK_EQ(*r.time_after_power_on(), "1 d 01:01:01.001");
    r.time_ms = 86400000;
    CHECK_EQ(*r.time_after_power_on(), "1 d 00:00:00.000");
    r.time_ms = 86399999;
    CHECK_EQ(*r.time_after_power_on(), "23:59:59.999");
    r.time_ms = 0;
    CHECK_EQ(*r.time_after_power_on(), "00:00:00.000");
    r.time_ms = 4294967294LL;  // the largest stamp the unit stores (FFFFFFFE ms)
    CHECK_EQ(*r.time_after_power_on(), "49 d 17:02:47.294");
    r.time_ms = std::nullopt;
    CHECK(!r.time_after_power_on().has_value());
}
