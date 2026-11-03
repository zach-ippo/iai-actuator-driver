// Tests of the SimulatedUnit's motion model and axis behaviour (sim unit), on an injected clock and raw frames: the
// trapezoid and its settling tail, PEND / MEND rising early, the MOVE bit clearing in deceleration, re-targeting, the
// STOP latch, pause / resume, the home sequence, push contact and miss, soft-limit / speed / acceleration alarms, row
// moves (0x9800 ignored while PMSS = 0), the position markers, the PIO test hook. Port of the C# edition's
// SimulatedUnitTests (motion part) and the Python edition's test_sim.py, with a few values taken from a trace of the
// Python simulator (VERIFIED in the Python edition against the hardware logs; this edition is UNVERIFIED on hardware).
// Unofficial: not an IAI product.
#include <cmath>

#include "harness.hpp"
#include "sim_fixtures.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::sim_test;

namespace {

/// DSS1 and DSSE of an 8-word monitor read.
struct Bits {
    int dss1;
    int dsse;
};

Bits bits(SimulatedUnit& unit) {
    const Words w = read_regs(unit, 0x9000, 8);
    return {w[5], w[7]};
}

/// A 7-word move (no PPOW / CTLF): the way the Python tests start a plain move.
void move7(SimulatedUnit& unit, double mm, double speed, double accel_g, double band = 0.1) {
    write_regs(unit, 0x9900, move_words(mm, band, speed, accel_g, 7));
}

}  // namespace

// ------------------------------------------------------------------------------------------ the pure trapezoid

TEST(sim_motion_trapezoid) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 83.95, 10.0, 0.1);  // 10 mm at 10 mm/s, 0.1 G: about 1.01 s
    rig.clock.advance(0.5);
    CHECK_NEAR(unit.position_mm(), 73.95 + 5.0 - 0.051, 0.02);
    Bits b = bits(unit);
    CHECK(b.dsse & reg::kDsseMove);
    CHECK_EQ(b.dss1 & reg::kDss1Pend, 0);
    const Words vnow = read_regs(unit, reg::kVnow, 2);
    CHECK_EQ(reg::from_i32_words(vnow[0], vnow[1]), 1248);  // a 10 mm/s cruise reads 12.48: quantized like the EC
    rig.clock.advance(0.49);
    CHECK(unit.moving());
    rig.clock.advance(0.03);
    CHECK(!unit.moving());
    CHECK_EQ(hexs(read_regs(unit, 0x9000, 2)), "0000 20CB");  // 83.95 mm
    b = bits(unit);
    CHECK(b.dss1 & reg::kDss1Pend);
    CHECK_EQ(b.dsse & reg::kDsseMove, 0);
    CHECK(b.dss1 & reg::kDss1TargetReached);  // MEND: the move ended at its target
}

TEST(sim_motion_matches_the_python_simulators_trace) {
    Rig rig(pure_trapezoid());
    move7(rig.u(), 83.95, 10.0, 0.1);
    rig.clock.advance(0.5);
    CHECK_NEAR(rig->position_mm(), 78.8989, 0.0005);  // the Python edition's value after 0.5 s
}

TEST(sim_motion_triangular_profile_never_reaches_the_commanded_speed) {
    Rig rig(pure_trapezoid());
    move7(rig.u(), 73.45, 100.0, 0.1);  // 0.5 mm backwards: never reaches 100 mm/s
    rig.clock.advance(2.0 * std::sqrt(0.5 / 980.0) + 0.01);
    CHECK(!rig->moving());
    CHECK_NEAR(rig->position_mm(), 73.45, 1e-6);
}

TEST(sim_motion_decel_stop_holds_until_standstill) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 140.0, 50.0, 0.1);
    rig.clock.advance(0.5);
    CHECK_EQ(coil(unit, reg::kCoilStop, true), 0);
    const double at_stop = unit.position_mm();
    rig.clock.advance(1.0);
    CHECK(!unit.moving());
    CHECK_NEAR(unit.position_mm(), at_stop + 50.0 * 50.0 / (2.0 * 980.0), 0.05);
    CHECK(unit.position_mm() < 140.0);
    const Bits b = bits(unit);
    CHECK(b.dss1 & reg::kDss1Pend);                 // PEND also after a STOP ...
    CHECK_EQ(b.dss1 & reg::kDss1TargetReached, 0);  // ... but MEND (the target was reached) stays 0
}

TEST(sim_motion_pause_resume) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 100.0, 20.0, 0.3);
    rig.clock.advance(0.3);
    CHECK_EQ(coil(unit, reg::kCoilStp, true), 0);
    rig.clock.advance(1.0);
    const double paused_at = unit.position_mm();
    const Bits b = bits(unit);
    CHECK(b.dss1 & reg::kDss1Stp);
    CHECK_EQ(b.dsse & reg::kDsseMove, 0);
    CHECK(unit.paused());
    CHECK(unit.moving());
    rig.clock.advance(1.0);
    CHECK_EQ(unit.position_mm(), paused_at);  // held at rest, exactly
    CHECK_EQ(coil(unit, reg::kCoilStp, false), 0);
    CHECK(!unit.paused());
    rig.clock.advance(5.0);
    CHECK_EQ(unit.position_mm(), 100.0);
    CHECK(!unit.moving());
}

TEST(sim_motion_a_long_pause_costs_no_time) {
    Rig rig(pure_trapezoid());
    move7(rig.u(), 100.0, 20.0, 0.3);
    rig.clock.advance(0.3);
    coil(rig.u(), reg::kCoilStp, true);
    rig.clock.advance(0.5);
    const double paused_at = rig->position_mm();
    rig.clock.advance(1.0e6);  // a million simulated seconds held at rest
    CHECK_EQ(rig->position_mm(), paused_at);
}

TEST(sim_motion_soft_limit_alarm_0x0D9) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 150.5, 10.0, 0.1);  // the unit's own limit is 150.30
    const Words w = read_regs(unit, 0x9000, 8);
    CHECK_EQ(w[2], 0x0D9);
    CHECK(w[5] & reg::kDss1Almh);
    CHECK(!unit.servo_on());
    CHECK(!unit.moving());
    CHECK_EQ(coil(unit, reg::kCoilAlrs, true), 0);
    CHECK_EQ(coil(unit, reg::kCoilAlrs, false), 0);
    CHECK_EQ(read_regs(unit, reg::kAlmc, 1)[0], 0);
    // limits are inclusive: 150.30 is accepted, -0.30 too (the servo is off after the alarm: switch it on first)
    iai(unit, reg::kIaiCmdServo, 1);
    move7(unit, 150.30, 50.0, 0.3);
    CHECK_EQ(unit.alarm_code(), 0);
    rig.clock.advance(5.0);
    CHECK_EQ(unit.position_mm(), 150.30);
    move7(unit, -0.30, 100.0, 0.3);
    CHECK_EQ(unit.alarm_code(), 0);
    move7(unit, -0.31, 100.0, 0.3);
    CHECK_EQ(unit.alarm_code(), kSimAlarmSoftLimit);
}

TEST(sim_motion_speed_and_acceleration_alarms) {
    {
        Rig rig;
        move7(rig.u(), 80.0, 800.0, 0.1);  // above the unit's 700 mm/s
        CHECK_EQ(rig->alarm_code(), 0x0AB);
        CHECK(!rig->servo_on());
    }
    {
        Rig rig;
        move7(rig.u(), 80.0, 10.0, 2.0);  // above the unit's 1.00 G
        CHECK_EQ(rig->alarm_code(), 0x0A7);
    }
    {
        Rig rig;
        write_regs(rig.u(), 0x9900, move_words(80.0, 0.0, 10.0, 0.1, 7));  // a zero INP band
        CHECK_EQ(rig->alarm_code(), kSimAlarmBadBand);
    }
    {
        Rig rig;
        move7(rig.u(), 80.0, 0.0, 0.1);  // a zero speed
        CHECK_EQ(rig->alarm_code(), kSimAlarmBadSpeed);
    }
    {
        SimulatedUnitOptions o;
        o.max_speed_mm_s = 100.0;  // the options move the limit
        o.max_accel_g = 0.5;
        Rig rig(o);
        move7(rig.u(), 80.0, 120.0, 0.3);
        CHECK_EQ(rig->alarm_code(), kSimAlarmBadSpeed);
    }
}

TEST(sim_motion_not_homed_is_operation_cancelled_0x083) {
    // 0x083 is "operation cancelled" in the alarm levels (0x080-0x0FF; UNVERIFIED): ALMH, the servo switches off. An
    // incremental unit (the rod) comes back from a power cycle without HEND.
    Rig rig(pure_trapezoid(SimProfile::Rod));
    rig->power_cycle();
    CHECK(!rig->homed());
    move7(rig.u(), 80.0, 10.0, 0.1);
    CHECK_EQ(rig->alarm_code(), 0x083);
    CHECK(read_regs(rig.u(), reg::kDss1, 1)[0] & reg::kDss1Almh);
    CHECK(!rig->servo_on());
}

TEST(sim_motion_servo_off_move_alarm_0x080) {
    Rig rig(pure_trapezoid());
    iai(rig.u(), reg::kIaiCmdServo, 0);
    move7(rig.u(), 80.0, 10.0, 0.1);
    CHECK_EQ(rig->alarm_code(), kSimAlarmServoOffMove);
}

TEST(sim_motion_message_level_alarm_keeps_the_axis_running) {
    // 0x04E "total moving count exceeded" (a maintenance warning) is message level: ALML, the move continues
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 100.0, 20.0, 0.3);
    rig.clock.advance(0.5);
    unit.trigger_alarm(0x04E);
    const std::uint16_t dss1 = read_regs(unit, reg::kDss1, 1)[0];
    CHECK((dss1 & reg::kDss1Alml) != 0 && (dss1 & reg::kDss1Almh) == 0);
    CHECK(unit.servo_on() && unit.moving());
    rig.clock.advance(5.0);
    CHECK_EQ(unit.position_mm(), 100.0);
}

TEST(sim_motion_a_major_alarm_stops_the_move) {
    Rig rig(pure_trapezoid());
    move7(rig.u(), 100.0, 20.0, 0.3);
    rig.clock.advance(0.5);
    const double here = rig->position_mm();
    rig->trigger_alarm(0x0E0);
    CHECK(!rig->moving());
    CHECK(!rig->servo_on());
    rig.clock.advance(5.0);
    CHECK_EQ(rig->position_mm(), here);  // the axis just stops (a simplification of the unit's braking)
}

// ------------------------------------------------------------------------------------------ home

TEST(sim_motion_home_pure_trapezoid) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(coil(unit, reg::kCoilHome, false), 0);
    CHECK_EQ(coil(unit, reg::kCoilHome, true), 0);
    Bits b = bits(unit);
    CHECK_EQ(b.dss1 & reg::kDss1Hend, 0);
    CHECK(b.dsse & reg::kDsseGhms);
    rig.clock.advance(4.2);
    b = bits(unit);
    CHECK(b.dss1 & reg::kDss1Hend);
    CHECK_EQ(b.dsse & reg::kDsseGhms, 0);
    CHECK_EQ(unit.position_mm(), 0.0);
    CHECK(contains(unit.events(), "home complete"));
}

TEST(sim_motion_home_phases_match_the_hardware_values) {
    Rig rig;  // the default model, settling tail included
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilHome, false);
    coil(unit, reg::kCoilHome, true);
    rig.clock.advance(1.0);
    Bits b = bits(unit);
    CHECK_EQ(b.dsse, 0x3BF2);  // live: homing (MOVE + GHMS)
    CHECK_EQ(b.dss1 & (reg::kDss1Hend | reg::kDss1Pend), 0);
    CHECK_NEAR(unit.position_mm(), 44.1033, 0.001);  // the Python edition's trace after 1.0 s
    rig.clock.advance(1.6);  // the seek to the mechanical end takes about 2.54 s
    CHECK(unit.position_mm() < -1.0);
    b = bits(unit);
    CHECK_EQ(b.dsse, 0x39C2);  // live: the creep back, MOVE 0 but GHMS 1
    CHECK_EQ(b.dss1 & reg::kDss1Hend, 0);
    CHECK_NEAR(unit.position_mm(), -1.6996, 0.001);
    rig.clock.advance(1.0);
    b = bits(unit);
    CHECK_EQ(b.dsse, 0x33C2);
    CHECK_EQ(b.dss1 & (reg::kDss1Hend | reg::kDss1Pend), reg::kDss1Hend | reg::kDss1Pend);
    CHECK_EQ(unit.position_mm(), 0.0);
    CHECK_EQ(b.dss1, 0x3098);
}

TEST(sim_motion_home_is_ignored_with_the_servo_off_and_while_stop_is_held) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilSon, false);
    coil(unit, reg::kCoilHome, true);
    CHECK(unit.homed());
    CHECK(contains(unit.events(), "home ignored (servo off)"));
    coil(unit, reg::kCoilHome, false);
    coil(unit, reg::kCoilSon, true);
    coil(unit, reg::kCoilStop, true);
    coil(unit, reg::kCoilHome, true);
    CHECK(unit.homed());
    CHECK(contains(unit.events(), "home ignored (STOP held)"));
}

TEST(sim_motion_a_stop_ends_the_home_and_leaves_it_incomplete) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilHome, false);
    coil(unit, reg::kCoilHome, true);
    rig.clock.advance(0.5);
    coil(unit, reg::kCoilStop, true);
    rig.clock.advance(2.0);
    CHECK(!unit.moving());
    CHECK(!unit.homed());  // home stays incomplete
    CHECK_EQ(bits(unit).dsse & reg::kDsseGhms, 0);
    // a move command during the home raises 0x084
    Rig other;
    coil(other.u(), reg::kCoilHome, true);
    other.clock.advance(0.3);
    move7(other.u(), 80.0, 10.0, 0.1);
    CHECK_EQ(other->alarm_code(), kSimAlarmMoveWhileHoming);
}

// ------------------------------------------------------------------------------------------ position moves

TEST(sim_motion_incremental_and_position_moves) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(write_regs(unit, 0x9900, {0, 500, 0, 10, 0, 5000, 30, 0, reg::kCtlfInc}), 0);  // +5.00 mm
    rig.clock.advance(2.0);
    CHECK_NEAR(unit.position_mm(), 78.95, 1e-6);
    CHECK_EQ(read_regs(unit, reg::kCtlf, 1)[0], 0);  // CTLF resets after the start
    CHECK_EQ(write_reg(unit, 0x9800, 2), 0);          // immediate move to position 2 (75.00 mm)
    rig.clock.advance(2.0);
    CHECK_NEAR(unit.position_mm(), 75.0, 1e-6);
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 2);   // POSS: the completed stored row
    CHECK_EQ(write_reg(unit, reg::kPosr, 1), 0);
    CHECK_EQ(coil(unit, reg::kCoilCstr, true), 0);    // CSTR starts a move to POSR
    rig.clock.advance(3.0);
    CHECK_NEAR(unit.position_mm(), 150.0, 1e-6);
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 1);
    // a 0x9900 move reads POSS 0 again
    move7(unit, 50.0, 100.0, 0.3);
    rig.clock.advance(3.0);
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 0);
    CHECK(contains(unit.events(), "move to position 2"));
}

TEST(sim_motion_stale_ctlf_hazard_exists) {
    // why the driver writes all 9 words: a pending CTLF changes the next 7-word move
    Rig rig;
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(write_reg(unit, reg::kCtlf, reg::kCtlfInc), 0);
    CHECK(!unit.moving());  // a CTLF written on its own starts nothing
    CHECK_EQ(write_regs(unit, 0x9900, {0, 3000, 0, 10, 0, 10000, 30}), 0);  // "absolute 30 mm"
    rig.clock.advance(3.0);
    CHECK_NEAR(unit.position_mm(), 103.95, 1e-6);
}

TEST(sim_motion_0x9800_row_moves_are_ignored_while_pmss_is_0_and_run_with_pmss_1) {
    // hardware test N4: accepted (a normal reply), no motion, no exception, no alarm
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilPmsl, false);
    CHECK_EQ(write_reg(unit, 0x9800, 1), 0);
    CHECK(!unit.moving());
    CHECK_EQ(unit.alarm_code(), 0);
    CHECK_EQ(read_regs(unit, 0x9800, 1)[0], 1);  // the register keeps what was written
    CHECK(contains(unit.events(), "0x9800 = 1 ignored (PMSS = 0)"));
    rig.clock.advance(1.0);
    CHECK_NEAR(unit.position_mm(), 73.95, 1e-9);
    coil(unit, reg::kCoilPmsl, true);
    CHECK_EQ(write_reg(unit, 0x9800, 1), 0);  // with PMSS = 1: row 1, 150.00 mm at 248.50 mm/s
    CHECK(unit.moving());
    rig.clock.advance(1.0);
    CHECK_NEAR(unit.position_mm(), 150.0, 1e-6);
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 1);
    // a row past the table: alarm 0x085 "Position No. error"
    CHECK_EQ(write_reg(unit, 0x9800, 900), 0);
    CHECK_EQ(unit.alarm_code(), kSimAlarmPositionNo);
}

TEST(sim_motion_retargeting_mid_move) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 140.0, 50.0, 0.3);
    rig.clock.advance(0.5);
    const double at_retarget = unit.position_mm();
    CHECK(at_retarget > 90.0 && at_retarget < 100.0);
    move7(unit, 60.0, 50.0, 0.3);  // a new PCMD / VCMD / ACMD write re-targets: back past the start
    CHECK(unit.moving());
    rig.clock.advance(10.0);
    CHECK(!unit.moving());
    CHECK_EQ(unit.position_mm(), 60.0);
    // a re-target to the same place just ends there
    move7(unit, 61.0, 50.0, 0.3);
    move7(unit, 61.0, 50.0, 0.3);
    rig.clock.advance(5.0);
    CHECK_EQ(unit.position_mm(), 61.0);
}

TEST(sim_motion_servo_off_stops_the_move_and_servo_on_returns) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 140.0, 50.0, 0.1);
    rig.clock.advance(0.2);
    CHECK_EQ(iai(unit, reg::kIaiCmdServo, 0), 0);
    CHECK(!unit.moving());
    CHECK(!unit.servo_on());
    CHECK_EQ(iai(unit, reg::kIaiCmdServo, 1), 0);
    CHECK(unit.servo_on());
    const Bits b = bits(unit);
    CHECK_EQ(b.dss1 & reg::kDss1TargetReached, 0);
}

// ------------------------------------------------------------------------------------------ hardware findings

TEST(sim_motion_move_bit_clears_during_deceleration) {
    Rig rig;  // the default model: settling tail on
    SimulatedUnit& unit = rig.u();
    begin_move(unit, 93.95, 100.0, 0.1);  // about 0.30 s; decel from about 0.20 s
    rig.clock.advance(0.1);
    Bits b = bits(unit);
    CHECK(b.dsse & reg::kDsseMove);
    CHECK_EQ(b.dss1, 0x3010);  // moving: BKRL 0, PEND 0
    rig.clock.advance(0.15);
    b = bits(unit);
    CHECK(unit.moving());
    CHECK_EQ(b.dsse & reg::kDsseMove, 0);  // cleared while still decelerating...
    CHECK_EQ(b.dss1 & reg::kDss1Pend, 0);  // ...but PEND is not set yet
    CHECK_NEAR(unit.position_mm(), 92.6472, 0.001);  // the Python edition's trace at 0.25 s
    rig.clock.advance(0.1);
    b = bits(unit);
    CHECK(b.dss1 & reg::kDss1Pend);
    CHECK_EQ(unit.position_mm(), 93.95);
    CHECK_EQ(b.dss1, 0x3099);
    CHECK_EQ(b.dsse, 0x33C2);
    // with the option off the MOVE bit stays set while decelerating
    SimulatedUnitOptions o;
    o.move_bit_clears_in_decel = false;
    Rig other(o);
    write_regs(other.u(), 0x9900, {0, 9395, 0, 10, 0, 10000, 10, 0, 0});
    other.clock.advance(0.25);
    CHECK(read_regs(other.u(), reg::kDsse, 1)[0] & reg::kDsseMove);
}

TEST(sim_motion_stop_latches_until_written_off) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    begin_move(unit, 140.0, 50.0, 0.1);
    rig.clock.advance(0.5);
    coil(unit, reg::kCoilStop, true);
    rig.clock.advance(1.0);
    const double stopped_at = unit.position_mm();
    CHECK(!unit.moving());
    CHECK(unit.stop_held());
    CHECK(unit.coil_state(reg::kCoilStop));
    begin_move(unit, 80.0, 50.0, 0.1);  // ignored while STOP is held
    rig.clock.advance(1.0);
    CHECK_EQ(unit.position_mm(), stopped_at);
    CHECK(contains(unit.events(), "move ignored (STOP held)"));
    CHECK_EQ(unit.alarm_code(), 0);
    coil(unit, reg::kCoilStop, false);
    CHECK(!unit.stop_held());
    begin_move(unit, 80.0, 50.0, 0.1);
    rig.clock.advance(3.0);
    CHECK_EQ(unit.position_mm(), 80.0);
}

TEST(sim_motion_stop_auto_reset_option) {
    SimulatedUnitOptions o;
    o.stop_latches = false;  // the CON behaviour: the controller resets STOP itself
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilStop, true);
    CHECK(!unit.stop_held());
    CHECK(!unit.coil_state(reg::kCoilStop));
    write_regs(unit, 0x9900, {0, 8000, 0, 10, 0, 5000, 30, 0, 0});
    rig.clock.advance(2.0);
    CHECK_EQ(unit.position_mm(), 80.0);
}

TEST(sim_motion_stp_is_a_level_input) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    coil(unit, reg::kCoilStp, true);  // paused while idle
    CHECK(bits(unit).dss1 & reg::kDss1Stp);
    begin_move(unit, 90.0, 50.0, 0.1);  // waits while STP is ON
    rig.clock.advance(2.0);
    CHECK_EQ(unit.position_mm(), 73.95);
    CHECK(unit.moving());
    coil(unit, reg::kCoilStp, false);  // resumes by itself on release (live)
    rig.clock.advance(2.0);
    CHECK_EQ(unit.position_mm(), 90.0);
}

TEST(sim_motion_an_alarm_reset_while_paused_cancels_the_move) {
    // DOC ME0384 p.6-76; ME0162-12A p.4-11, 5-85: an ALRS edge while paused cancels the remaining travel
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    move7(unit, 120.0, 20.0, 0.3);
    rig.clock.advance(0.5);
    coil(unit, reg::kCoilStp, true);
    rig.clock.advance(0.5);
    const double here = unit.position_mm();
    coil(unit, reg::kCoilAlrs, true);
    coil(unit, reg::kCoilAlrs, false);
    CHECK(contains(unit.events(), "paused move cancelled (ALRS while paused)"));
    coil(unit, reg::kCoilStp, false);
    rig.clock.advance(10.0);
    CHECK_EQ(unit.position_mm(), here);  // releasing the pause does not resume the old target
}

TEST(sim_motion_push_missed) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    begin_move(unit, 80.0, 10.0, 0.1, 10.0, 76, reg::kCtlfPush | reg::kCtlfPushAway);
    rig.clock.advance(0.3);
    CHECK_EQ(bits(unit).dsse & reg::kDssePush, 0);  // approach
    rig.clock.advance(0.8);
    CHECK(bits(unit).dsse & reg::kDssePush);  // pushing beyond 80 mm
    CHECK(unit.position_mm() > 80.0);
    rig.clock.advance(1.0);
    const Bits b = bits(unit);
    CHECK_EQ(unit.position_mm(), 90.0);  // PCMD + band
    CHECK(b.dss1 & reg::kDss1Psfl);
    CHECK_EQ(b.dss1 & reg::kDss1Pend, 0);
    CHECK(contains(unit.events(), "push missed"));
    begin_move(unit, 75.0, 50.0, 0.1);  // the next move clears PSFL
    CHECK_EQ(bits(unit).dss1 & reg::kDss1Psfl, 0);
}

TEST(sim_motion_push_contact) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    unit.set_obstacle_mm(85.0);
    CHECK_EQ(unit.obstacle_mm().value(), 85.0);
    begin_move(unit, 80.0, 10.0, 0.1, 20.0, 76, reg::kCtlfPush | reg::kCtlfPushAway);
    rig.clock.advance(2.0);
    const Bits b = bits(unit);
    CHECK_EQ(unit.position_mm(), 85.0);
    CHECK_EQ(b.dss1, 0x3099);  // live 0x7099 (b14 SFTY aside)
    CHECK_EQ(b.dsse, 0x37C3);  // live after contact
    CHECK(contains(unit.events(), "push contact at 85.00 mm"));
    const Words cnow = read_regs(unit, reg::kCnow, 2);
    CHECK_EQ(reg::from_i32_words(cnow[0], cnow[1]), 464);  // live plateau at 30 %
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 0);
    // a STOP ends the push hold
    coil(unit, reg::kCoilStop, true);
    coil(unit, reg::kCoilStop, false);
    CHECK_EQ(bits(unit).dsse & reg::kDssePushContact, 0);
    unit.set_obstacle_mm(std::nullopt);
    CHECK(!unit.obstacle_mm().has_value());
}

TEST(sim_motion_push_toward_home_and_skipped_approach) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    begin_move(unit, 60.0, 10.0, 0.1, 10.0, 76, reg::kCtlfPush);  // DIR 0 = toward home
    rig.clock.advance(4.0);
    CHECK_EQ(unit.position_mm(), 50.0);
    // at 73.95 and already past PCMD 70 for a + push: the push starts from where it is, no approach back to 70
    Rig past;
    write_regs(past.u(), 0x9900, {0, 7000, 0, 1000, 0, 1000, 10, 76, reg::kCtlfPush | reg::kCtlfPushAway});
    past.clock.advance(0.2);
    CHECK(past->position_mm() > 73.95);
    past.clock.advance(2.0);
    CHECK_EQ(past->position_mm(), 80.0);  // pushes to PCMD + band, no approach back to 70
}

TEST(sim_motion_push_rows_run_with_row_moves) {
    // the gripper's row 1 is a push row (4.10 mm approach, 5.40 mm push, 50 %): at 5 mm/s on the gripper
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 3.0;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    unit.set_obstacle_mm(7.5);
    CHECK_EQ(write_reg(unit, 0x9800, 1), 0);
    rig.clock.advance(10.0);
    CHECK_EQ(unit.position_mm(), 7.5);  // contact at the obstacle
    CHECK_EQ(read_regs(unit, reg::kPoss, 1)[0], 1);
    CHECK(bits(unit).dsse & reg::kDssePushContact);
    // no workpiece: a miss at the band end (4.10 + 5.40 = 9.50 mm)
    Rig empty(o);
    write_reg(empty.u(), 0x9800, 1);
    empty.clock.advance(10.0);
    CHECK_EQ(empty->position_mm(), 9.5);
    CHECK(bits(empty.u()).dss1 & reg::kDss1Psfl);
}

TEST(sim_motion_push_blip_then_miss) {
    // hardware test G5: the full contact fingerprint for a moment during the push stroke, then a miss at 7.00 mm
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 1.0;
    o.push_blip_mm = 6.76;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    // push + (closing) 30 %, PCMD 2.00, band 5.00, 10 mm/s
    write_regs(unit, 0x9900, {0, 200, 0, 500, 0, 1000, 10, 76, reg::kCtlfPush | reg::kCtlfPushAway});
    std::vector<double> blips;
    int steps = 0;
    for (; steps < 3000; ++steps) {
        rig.clock.advance(0.001);
        const Words w = read_regs(unit, 0x9000, 8);
        if (w[7] & reg::kDssePushContact) {
            blips.push_back(pnow_mm(w));
            CHECK(unit.moving());
            CHECK_EQ(w[5] & 0x0809, 0x0009);  // PEND + MEND, PSFL 0 (live 0x3099)
            CHECK_EQ(w[7], 0x37C3);           // live: PUSH + b0, MOVE 0
        }
        if (!unit.moving()) break;
    }
    REQUIRE(!blips.empty());
    for (double p : blips) {
        CHECK(p >= 6.76 - 1e-9);
        CHECK(p <= 6.86 + 1e-9);
    }
    CHECK_EQ(blips.size(), 20u);  // the Python edition's trace: 20 samples at 1 ms
    CHECK_EQ(unit.position_mm(), 7.0);
    CHECK(read_regs(unit, reg::kDss1, 1)[0] & reg::kDss1Psfl);
    CHECK_EQ(read_regs(unit, reg::kDsse, 1)[0] & reg::kDssePushContact, 0);
    CHECK_EQ(steps + 1, 1110);
}

TEST(sim_motion_soft_limit_block_and_identification_follow_the_stroke) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(hexs(read_regs(unit, 0x0700, 4)), "0000 3AB6 FFFF FFE2");  // live values: +150.30 / -0.30
    begin_move(unit, 150.2, 50.0, 0.1);  // inside the unit's own +150.30 limit
    rig.clock.advance(3.0);
    CHECK_EQ(unit.alarm_code(), 0);
    SimulatedUnitOptions o;
    o.stroke_mm = 50.0;
    SimulatedUnit shorter(o);
    CHECK_EQ(hexs(read_regs(shorter, 0x0700, 2)), "0000 13A6");  // 50.30 mm
    Rig narrow(o);
    begin_move(narrow.u(), 60.0, 50.0, 0.1);
    CHECK_EQ(narrow->alarm_code(), kSimAlarmSoftLimit);
}

TEST(sim_motion_safety_speed_coil_sets_the_status_bits) {
    SimulatedUnit unit;
    CHECK_EQ(coil(unit, reg::kCoilSfty, true), 0);
    CHECK(bits(unit).dss1 & reg::kDss1Sfty);
    CHECK(read_regs(unit, reg::kDrg1, 1)[0] & 0x4000);
    SimulatedUnitOptions o;
    o.safety_speed = true;
    SimulatedUnit start_on(o);
    CHECK(start_on.safety_speed());
}

// ------------------------------------------------------------------------------------------ position markers

TEST(sim_markers_follow_stored_positions) {
    Rig rig;
    SimulatedUnit& unit = rig.u();
    const auto dopm = [&] { return read_regs(unit, reg::kDopm, 1)[0]; };
    const auto zons = [&] { return read_regs(unit, 0x9013, 1)[0]; };
    CHECK_EQ(dopm(), sim_detail::kDopm3PosNoAlarm);  // 73.95 mm: no marker
    CHECK_EQ(zons(), 0x0003);
    // hardware test 20 (slider, 3-position mode): DOPM 0x000C / 0x0009 / 0x000A, ZONS 0x4003 / 0x1003 / 0x2003
    struct Row {
        double target;
        int dopm;
        int zons;
    };
    for (const Row& row : {Row{75.0, 0x000C, 0x4003}, Row{0.0, 0x0009, 0x1003}, Row{150.0, 0x000A, 0x2003}}) {
        begin_move(unit, row.target, 100.0, 0.3);
        rig.clock.advance(0.3);
        if (std::fabs(unit.position_mm() - row.target) > 1.0) {
            CHECK_EQ(dopm() & 0x7, 0);  // mid-stroke: no row nearby, all markers dark
            CHECK_EQ(zons() & 0x7000, 0);
        }
        rig.clock.advance(30.0);
        CHECK_EQ(dopm(), row.dopm);
        CHECK_EQ(zons(), row.zons);
    }
}

TEST(sim_markers_ls_range_edges) {
    // slider: ZONS 0x2003 at 150.10 and 0x1003 at -0.10 (motion log), dark 0.11 away
    struct Edge {
        double position;
        int zons;
    };
    for (const Edge& e : {Edge{150.10, 0x2003}, Edge{149.89, 0x0003}, Edge{-0.10, 0x1003}, Edge{0.11, 0x0003},
                          Edge{74.90, 0x4003}, Edge{75.11, 0x0003}}) {
        SimulatedUnitOptions o;
        o.position_mm = e.position;
        SimulatedUnit unit(o);
        CHECK_EQ(read_regs(unit, 0x9013, 1)[0], e.zons);
    }
    // outside the soft limits the zone bits drop
    SimulatedUnitOptions far;
    far.position_mm = 151.0;
    SimulatedUnit out(far);
    CHECK_EQ(read_regs(out, 0x9013, 1)[0] & 0x0003, 0);
}

TEST(sim_markers_lit_while_moving_through_a_row) {
    // slider 73.95 -> 76.00 at 10 mm/s: LS2 is lit while passing 74.90..75.10 mm, then dark again
    Rig rig;
    SimulatedUnit& unit = rig.u();
    write_regs(unit, 0x9900, {0, 7600, 0, 10, 0, 1000, 10, 0, 0});
    int lit_while_moving = 0;
    for (int i = 0; i < 400; ++i) {
        rig.clock.advance(0.005);
        const double pos = unit.position_mm();
        const std::uint16_t zons = read_regs(unit, 0x9013, 1)[0];
        CHECK_EQ(std::llabs(sim_detail::py_round(pos * 100) - 7500) <= 10, (zons & reg::kZonsLs2) != 0);
        if ((zons & reg::kZonsLs2) && unit.moving()) ++lit_while_moving;
    }
    CHECK(lit_while_moving > 0);
    CHECK(!unit.moving());
    CHECK_EQ(read_regs(unit, 0x9013, 1)[0], 0x0003);  // 76.00: dark
}

TEST(sim_markers_gripper_two_position_layout) {
    // gripper after a missed push: DOPM 0x0004 at 9.50 mm (the push row's bit stays off), ZONS 0x2003
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 3.0;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(read_regs(unit, reg::kDopm, 1)[0], 0x0005);  // 3.00 mm: LS0 + *ALM (2-position layout)
    write_reg(unit, 0x9800, 1);
    rig.clock.advance(10.0);
    CHECK_EQ(unit.position_mm(), 9.5);
    CHECK(read_regs(unit, reg::kDss1, 1)[0] & reg::kDss1Psfl);
    CHECK_EQ(read_regs(unit, reg::kDopm, 1)[0], 0x0004);
    CHECK_EQ(read_regs(unit, 0x9013, 1)[0], 0x2003);
    SimulatedUnit start(SimProfile::Gripper);  // the live start value at 9.50 mm
    CHECK_EQ(read_regs(start, 0x9013, 1)[0], 0x2003);
}

// ------------------------------------------------------------------------------------------ settling tail

namespace {

struct Sample {
    double pos;
    int dss1;
    int dsse;
    bool moving;
};

std::vector<Sample> run_move(Rig& rig, double target) {
    SimulatedUnit& unit = rig.u();
    write_regs(unit, 0x9900, move_block(target, 0.10, 10.0, 0.10));
    std::vector<Sample> samples;
    for (int i = 0; i < 1000; ++i) {
        rig.clock.advance(0.002);
        const Words w = read_regs(unit, 0x9000, 8);
        samples.push_back({pnow_mm(w), w[5], w[7], unit.moving()});
        if (!unit.moving()) break;
    }
    return samples;
}

}  // namespace

TEST(sim_settling_pend_and_mend_rise_before_standstill) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 3.0;
    Rig rig(o);
    const std::vector<Sample> samples = run_move(rig, 8.0);  // the self-check move: 3.00 -> 8.00 at 10 mm/s
    std::vector<Sample> early;
    for (const Sample& s : samples)
        if (s.moving && (s.dss1 & 0x0809) == 0x0009) early.push_back(s);  // MEND + PEND, PSFL 0, while still moving
    REQUIRE(!early.empty());
    bool before_8 = false;
    for (const Sample& s : early) {
        CHECK(std::fabs(s.pos - 8.0) <= 0.10 + 1e-9);
        CHECK_EQ(s.dsse & reg::kDsseMove, 0);  // live 0x33C2 in the settling tail
        before_8 = before_8 || s.pos < 7.99;
    }
    CHECK(before_8);
    int tail = 0;
    for (const Sample& s : samples)
        if (s.moving && s.pos >= 7.95 && s.pos < 8.0) ++tail;
    CHECK(tail * 0.002 >= 0.05);  // the tail is long enough for a poller to see
    CHECK_EQ(samples.back().pos, 8.0);
    CHECK_EQ(samples.back().dss1 & 0x0009, 0x0009);
    // the Python edition's trace of the same move: 293 samples of 2 ms, 45 of them in-band while moving, the first at
    // 7.91 mm; it ends with DSS1 0x3099 / DSSE 0x33C2
    CHECK_EQ(samples.size(), 293u);
    CHECK_EQ(early.size(), 45u);
    CHECK_NEAR(early.front().pos, 7.91, 0.0051);
    CHECK_EQ(samples.back().dss1, 0x3099);
    CHECK_EQ(samples.back().dsse, 0x33C2);
}

TEST(sim_settling_options_off_give_the_pure_trapezoid) {
    SimulatedUnitOptions o;
    o.profile = SimProfile::Gripper;
    o.position_mm = 3.0;
    o.early_in_position = false;
    o.settle_speed_mm_s = 0.0;
    Rig rig(o);
    const std::vector<Sample> samples = run_move(rig, 8.0);
    for (const Sample& s : samples) CHECK(!(s.moving && (s.dss1 & reg::kDss1Pend)));
    CHECK_EQ(samples.back().pos, 8.0);
    // a different tail: settle_tail_mm = 0 disables it, the move still ends exactly on target
    SimulatedUnitOptions no_tail = o;
    no_tail.early_in_position = true;
    no_tail.settle_speed_mm_s = 0.5;
    no_tail.settle_tail_mm = 0.0;
    Rig other(no_tail);
    CHECK_EQ(run_move(other, 8.0).back().pos, 8.0);
}

// ------------------------------------------------------------------------------------------ time scale

TEST(sim_time_scale_speeds_the_model_up) {
    SimulatedUnitOptions o;
    o.time_scale = 100.0;  // 100 simulated seconds per real second
    SimulatedUnit unit(o);
    CHECK_EQ(unit.time_scale(), 100.0);
    begin_move(unit, 120.0, 100.0, 0.5);  // about 0.5 s of simulated time
    sleep_at_least(0.15);
    CHECK(!unit.moving());
    CHECK_EQ(unit.position_mm(), 120.0);
}

// ------------------------------------------------------------------------------------------ PIO test hook

TEST(sim_pio_held_start_input_restarts_after_stop_release) {
    SimulatedUnitOptions o;
    o.pmsl = false;
    Rig rig(o);
    SimulatedUnit& unit = rig.u();
    CHECK(unit.pio_move(120.0, 20.0, 0.3, true));
    rig.clock.advance(0.5);
    coil(unit, reg::kCoilStop, true);
    rig.clock.advance(1.0);
    CHECK(!unit.moving());
    const double stopped = unit.position_mm();
    coil(unit, reg::kCoilStop, false);
    CHECK(unit.moving());  // the held start input restarts the move (a WORST-CASE model)
    CHECK(contains(unit.events(), "pio start input still on: move restarted"));
    rig.clock.advance(0.5);
    CHECK(unit.position_mm() > stopped + 1.0);
    unit.pio_release();
    coil(unit, reg::kCoilStop, true);
    rig.clock.advance(2.0);
    coil(unit, reg::kCoilStop, false);
    CHECK(!unit.moving());
}

TEST(sim_pio_is_ignored_with_modbus_control) {
    SimulatedUnitOptions o;
    o.pmsl = true;
    SimulatedUnit unit(o);
    CHECK(!unit.pio_move(120.0));
    CHECK(!unit.moving());
    CHECK(contains(unit.events(), "pio start ignored (PMSS = 1)"));
    SimulatedUnitOptions manual;
    manual.pmsl = false;
    SimulatedUnit other(manual);
    other.pio_move(120.0, 20.0, 0.3, true);
    coil(other, reg::kCoilPmsl, true);  // PMSL on: stops the move, PIO inputs ignored
    coil(other, reg::kCoilStop, true);
    coil(other, reg::kCoilStop, false);
    CHECK(!contains(other.events(), "pio start input still on: move restarted"));
    CHECK(contains(other.events(), "move stopped by PMSL"));
}

TEST(sim_pmsl_on_mid_move_stops_it) {
    // hardware test 18: PMSL ON mid-move stopped the move (PEND = 1, MEND = 0)
    SimulatedUnitOptions o;
    o.pmsl = false;
    o.settle_speed_mm_s = 0.0;
    Rig rig(o);
    write_regs(rig.u(), 0x9900, move_block(120.0, 0.1, 20.0, 0.3));
    rig.clock.advance(0.5);
    CHECK(rig->moving());
    coil(rig.u(), reg::kCoilPmsl, true);
    rig.clock.advance(2.0);
    CHECK(!rig->moving());
    CHECK(rig->position_mm() < 120.0);
    const std::uint16_t dss1 = read_regs(rig.u(), reg::kDss1, 1)[0];
    CHECK(dss1 & reg::kDss1Pend);
    CHECK_EQ(dss1 & reg::kDss1TargetReached, 0);
}
