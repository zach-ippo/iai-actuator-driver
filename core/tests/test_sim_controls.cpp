// Tests of the SimulatedUnit's 0.5 models and test hooks (sim unit; src/sim_controls.cpp): IAI 0x48, the teach coils,
// the EMG / drive-cut / enable inputs, the automatic servo OFF, the cold boot with its slow first servo ON, the
// function codes / broadcast / parw_latches / rtc options, the settings and levels a test changes later and the clock
// a HookedSimUnit is given later. Port of the Python edition's test_sim.py models; raw frames through handle_frame (no
// bus). The models are the Python edition's (DOC, UNVERIFIED on an EC). Unofficial: not an IAI product.
#include "harness.hpp"
#include "iai_actuator_driver/sim_hooks.hpp"
#include "sim_fixtures.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::sim_test;

namespace {

/// IAI 0x48 with op1: the rejection code of the answer (0 = accepted, -1 = silent, -2 = an exception response).
int iai48(SimulatedUnit& unit, int command, std::uint32_t op1 = 0) {
    Bytes pdu = {0x48, 0x00, static_cast<std::uint8_t>(command >> 8), static_cast<std::uint8_t>(command & 0xFF)};
    for (int shift : {24, 16, 8, 0}) pdu.push_back(static_cast<std::uint8_t>((op1 >> shift) & 0xFF));
    pdu.resize(20, 0);  // op2..op4
    const Bytes response = unit.handle_frame(build_frame(1, pdu));
    if (response.size() < 5) return -1;
    if (response[1] & 0x80) return -2;
    return (response[2] << 8) | response[3];
}

/// The first register of 32-bit item (class, item id), found through the unit's directory and ID map.
std::optional<int> item_address_of(SimulatedUnit& unit, int class_id, int item_id) {
    const int count = read_regs(unit, 0, 8)[2];
    const Words records = read_block(unit, 8, 8 * count);
    for (int i = 0; i < count; ++i) {
        const int ci = records[8 * i], dq = records[8 * i + 2], da = records[8 * i + 4], ia = records[8 * i + 5];
        if (ci != class_id || ia == 0 || records[8 * i + 7] != 2) continue;
        const Words ids = read_block(unit, ia, 2 * dq);
        for (int k = 0; k < dq; ++k)
            if (ids[2 * k] == item_id) return da + 2 * k;
    }
    return std::nullopt;
}

}  // namespace

// ------------------------------------------------------------------------------------------ IAI 0x48, teach coils

TEST(sim_iai_0x48_commands_and_rejections) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    // the answer: 0x48 | rejection u16 | four zero u32
    CHECK_EQ(hexs(exchange(unit, "48 00 00 30 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00")),
             hexs(expect(1, "48 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00")));
    CHECK(!unit.servo_on());  // cmd 0x30 op1 0: servo off
    CHECK_EQ(iai48(unit, 0x30, 1), 0);
    CHECK(unit.servo_on());
    CHECK_EQ(iai48(unit, 0x7777), kSimRejectCommandCode);
    CHECK_EQ(hexs(exchange(unit, "48 00 00 34")), hexs(expect(1, "C8 03")));  // a short 0x48
    // 0x34: decel stop
    CHECK_EQ(begin_move(unit, 140.0, 20.0, 0.3), 0);
    rig.clock.advance(0.5);
    CHECK(unit.moving());
    CHECK_EQ(iai48(unit, 0x34), 0);
    rig.clock.advance(2.0);
    CHECK(!unit.moving());
    CHECK(unit.position_mm() < 90.0);
    // 0x23: alarm reset (op1 ignored); 0x31: home
    unit.trigger_alarm(kSimAlarmBadBand);
    CHECK_EQ(iai48(unit, 0x23, 7), 0);
    CHECK_EQ(unit.alarm_code(), 0);
    CHECK_EQ(iai48(unit, 0x30, 1), 0);
    CHECK_EQ(iai48(unit, 0x31), 0);
    CHECK(!unit.homed());
    CHECK(unit.moving());
    // PMSS 0: rejected as on 0x45
    CHECK_EQ(coil(unit, reg::kCoilPmsl, false), 0);
    CHECK_EQ(iai48(unit, 0x30, 0), kSimRejectMonitorMode);
}

TEST(sim_teach_coils_write_pnow_into_row_posr) {
    SimulatedUnit unit;  // 73.95 mm, homed
    CHECK_EQ(write_reg(unit, reg::kPosr, 1), 0);
    CHECK_EQ(coil(unit, reg::kCoilTeac, true), 0);
    CHECK(contains(unit.events(), "TEAC ignored (MOD off)"));
    CHECK_EQ(coil(unit, reg::kCoilTeac, false), 0);
    CHECK_EQ(coil(unit, reg::kCoilMod, true), 0);
    CHECK_EQ(read_regs(unit, 0x9006, 1)[0], reg::kDss2Enbs | reg::kDss2Mods);
    const int writes = unit.nonvolatile_writes();
    CHECK_EQ(coil(unit, reg::kCoilTeac, true), 0);
    const Words row = unit.position_row(1);
    CHECK_NEAR(reg::from_i32_words(row[0], row[1]) / 100.0, 73.95, 1e-9);
    CHECK_EQ(row[reg::kPosChecksum], reg::position_checksum(head(row, reg::kPositionWordsWritable)));
    CHECK_EQ(unit.nonvolatile_writes(), writes + 1);
    CHECK_EQ(read_regs(unit, 0x9006, 1)[0], reg::kDss2Enbs | reg::kDss2Mods | reg::kDss2Pwrt);
    CHECK_EQ(coil(unit, reg::kCoilTeac, false), 0);  // PWRT until TEAC is released
    CHECK_EQ(read_regs(unit, 0x9006, 1)[0], reg::kDss2Enbs | reg::kDss2Mods);
    SimStateLevels levels;
    levels.homed = false;
    unit.set_state(levels);
    CHECK_EQ(coil(unit, reg::kCoilTeac, true), 0);
    CHECK_EQ(unit.alarm_code(), kSimAlarmTeachNotHomed);  // TEAC before homing
}

// ------------------------------------------------------------------------------------------ inputs, ASOF, cold boot

TEST(sim_inputs_switch_the_servo_off_and_block_servo_on) {
    SimulatedUnit unit;
    unit.set_inputs(true, std::nullopt, std::nullopt);
    CHECK(!unit.servo_on());
    Words w = read_regs(unit, 0x9005, 3);  // DSS1 DSS2 DSSE
    CHECK_EQ(w[0] & reg::kDss1Emgs, reg::kDss1Emgs);
    CHECK_EQ(w[2] & reg::kDsseEmgp, reg::kDsseEmgp);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(!unit.servo_on());
    CHECK(contains(unit.events(), "servo on ignored (EMG / drive cut / enable switch)"));
    unit.set_inputs(false, true, std::nullopt);
    CHECK_EQ(read_regs(unit, 0x9009, 1)[0] & reg::kStatMpow, 0);  // STAT low word: MPOW 0
    CHECK_EQ(read_regs(unit, 0x9007, 1)[0] & reg::kDsseMpuv, reg::kDsseMpuv);
    unit.set_inputs(std::nullopt, false, false);  // the enable switch released
    CHECK_EQ(read_regs(unit, 0x9006, 1)[0] & reg::kDss2Enbs, 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(!unit.servo_on());
    unit.set_inputs(std::nullopt, std::nullopt, true);
    w = read_regs(unit, 0x9005, 5);
    CHECK_EQ(w[0] & reg::kDss1Emgs, 0);
    CHECK_EQ(w[1], reg::kDss2Enbs);
    CHECK_EQ(w[2] & (reg::kDsseEmgp | reg::kDsseMpuv), 0);
    CHECK_EQ(w[4] & reg::kStatMpow, reg::kStatMpow);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(unit.servo_on());
}

TEST(sim_auto_servo_off_after_a_standstill_and_a_move_switches_it_back_on) {
    SimulatedUnitOptions o = pure_trapezoid();
    o.auto_servo_off_s = 0.5;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    rig.clock.advance(1.0);
    CHECK(unit.servo_on());  // no move has ended yet: the timer has not started
    CHECK_EQ(begin_move(unit, 80.0, 50.0, 0.3), 0);
    rig.clock.advance(1.0);
    CHECK(!unit.moving());
    CHECK(unit.servo_on());
    rig.clock.advance(0.6);
    CHECK(!unit.servo_on());
    CHECK(unit.auto_servo_off_active());
    const Words stat = read_regs(unit, 0x9008, 2);
    CHECK_EQ(stat[0] & 0x0002, 0x0002);  // b17 ASOF
    CHECK_EQ(stat[1] & 0x0006, 0x0002);  // b1 SON still commanded, b2 SV off
    CHECK_EQ(begin_move(unit, 70.0, 50.0, 0.3), 0);  // a move command switches it back on (DOC PCON p.5-5)
    CHECK(unit.servo_on());
    CHECK(unit.moving());
    CHECK(!unit.auto_servo_off_active());
}

TEST(sim_cold_boot_servo_off_until_the_slow_first_servo_on) {
    SimulatedUnitOptions o;
    o.cold_boot = true;
    o.ec_first_son_s = 0.4;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    CHECK(!unit.servo_on());
    CHECK_EQ(read_regs(unit, 0x9007, 1)[0] & reg::kDssePsns, 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    rig.clock.advance(0.3);
    CHECK(!unit.servo_on());  // the excitation detection is still running
    rig.clock.advance(0.2);
    CHECK(unit.servo_on());
    CHECK_EQ(read_regs(unit, 0x9007, 1)[0] & reg::kDssePsns, reg::kDssePsns);
    CHECK_EQ(coil(unit, reg::kCoilSon, false), 0);
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(unit.servo_on());  // excited: a later servo ON is immediate
    unit.power_cycle();
    CHECK(!unit.servo_on());  // the next power-up starts cold again
    CHECK_EQ(read_regs(unit, 0x9007, 1)[0] & reg::kDssePsns, 0);
}

TEST(sim_cold_boot_first_servo_on_of_a_con_unit_follows_item_9_1) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Pcon;
    o.cold_boot = true;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    const auto address = item_address_of(unit, 0x0009, 1);
    REQUIRE(address.has_value());
    unit.set_parameter_word(*address, 0x0002, true);  // No.30 = 1 (b17-18): T1 0.22 s
    unit.set_parameter_word(*address + 1, 50, true);  // No.29 = 50 ms
    CHECK_EQ(coil(unit, reg::kCoilSon, true), 0);
    CHECK(contains(unit.events(), "first servo on: excitation detection 2.726 s"));  // 0.006 + 0.27 x 10 + 0.020
    rig.clock.advance(2.7);
    CHECK(!unit.servo_on());
    rig.clock.advance(0.05);
    CHECK(unit.servo_on());
}

// ------------------------------------------------------------------------------------------ options, settings, levels

TEST(sim_function_codes_broadcast_parw_latches_and_rtc_options) {
    SimulatedUnitOptions o;
    o.function_codes = std::vector<int>{0x03, 0x06, 0x10};
    o.parw_latches = false;
    o.rtc = true;
    SimulatedUnit unit(o);
    CHECK_EQ(coil(unit, reg::kCoilSon, false), 0x01);  // FC05 is not carried: exception 01
    CHECK(unit.servo_on());
    CHECK_EQ(read_regs(unit, 0x9015, 1)[0] & reg::kSsseRtc, reg::kSsseRtc);
    CHECK(exchange(unit, "06 0D 03 00 02", 0).empty());  // a broadcast (off by default with function_codes)
    CHECK_EQ(read_regs(unit, 0x0D03, 1)[0], 0);         // ...is not acted on
    CHECK(contains(unit.events(), "broadcast function 0x06 ignored"));
    SimulatedUnitOptions later = unit.options();
    later.function_codes.reset();
    unit.set_options(later);
    CHECK(!unit.options().broadcast.value_or(true));  // broadcast stays as it was
    CHECK_EQ(coil(unit, reg::kCoilParw, true), 0);    // PARW: acknowledged and ignored
    CHECK(!unit.parw_busy());
    CHECK(contains(unit.events(), "PARW ignored"));
    later.broadcast = true;
    unit.set_options(later);
    CHECK(exchange(unit, "06 0D 03 00 02", 0).empty());
    CHECK_EQ(read_regs(unit, 0x0D03, 1)[0], 2);
}

TEST(sim_settings_and_levels_change_later) {
    SimulatedUnit unit;
    SimulatedUnitOptions o = unit.options();
    CHECK_NEAR(o.slave_id_delay_s.value_or(-1.0), 0.0, 1e-12);
    CHECK(o.dopm == std::optional<DopmLayout>(DopmLayout::ThreePosition));
    o.ls_range_mm = 0.50;
    o.stop_latches = false;
    o.push_blip_mm = 3.0;
    unit.set_options(o);
    CHECK_NEAR(unit.ls_range_mm(), 0.50, 1e-12);
    CHECK(!unit.options().stop_latches);
    CHECK(unit.options().push_blip_mm == std::optional<double>(3.0));
    const std::size_t events = unit.events().size();
    SimStateLevels levels;
    levels.servo_on = false;
    levels.pmsl = false;
    levels.safety_speed = true;
    unit.set_state(levels);
    CHECK(!unit.servo_on());
    CHECK(!unit.pmsl());
    CHECK(unit.safety_speed());
    CHECK(unit.homed());
    CHECK_EQ(unit.events().size(), events);  // the level only: nothing done, nothing logged
    // a committed LS range (EC parameter 2, item 8.9) applies at the next restart
    const auto address = item_address_of(unit, reg::kMotionParamsCategory, 9);
    REQUIRE(address.has_value());
    unit.set_parameter_word(*address, 0, true);
    unit.set_parameter_word(*address + 1, 25, true);
    CHECK_NEAR(unit.ls_range_mm(), 0.50, 1e-12);
    unit.power_cycle();
    CHECK_NEAR(unit.ls_range_mm(), 0.25, 1e-12);
    unit.set_parameter_word(*address + 1, 30, false);  // RAM only: lost by the restart
    unit.power_cycle();
    CHECK_NEAR(unit.ls_range_mm(), 0.25, 1e-12);
}

TEST(sim_hooked_unit_runs_on_a_clock_set_later_without_a_jump) {
    HookedSimUnit unit(pure_trapezoid());
    FakeClock clock;
    const double before = unit.clock_now();
    unit.set_clock(clock.reader());
    CHECK_NEAR(unit.clock_now(), before, 0.05);  // the unit's time goes on from where it was
    CHECK_EQ(begin_move(unit, 83.95, 10.0, 0.3), 0);
    CHECK_NEAR(unit.position_mm(), 73.95, 1e-9);  // the clock stands still: so does the axis
    clock.advance(2.0);
    CHECK(!unit.moving());
    CHECK_NEAR(unit.position_mm(), 83.95, 1e-9);
    unit.set_clock({});
    CHECK_NEAR(unit.clock_now(), before + 2.0, 0.05);  // the library's clock again, still no jump
}
