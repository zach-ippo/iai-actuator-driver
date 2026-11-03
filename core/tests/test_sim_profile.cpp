// Sim unit: the command profile 0x0F00 (written with FC10 / FC17, read back as written, no motion until the PCRF coil;
// separate acceleration and deceleration; STOP stops it; volatile across a restart) and the acceleration pattern of
// each move start. Port of the Python edition's sim.py behaviour (VERIFIED there on the EC-GRBP8M gripper and the
// PCON-CB for the profile); raw frames through handle_frame on a hand-advanced clock.
#include <string>

#include "harness.hpp"
#include "sim_fixtures.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::sim_test;

namespace {

/// The 20 words of a command profile: PCMD, VCMD, ACMD, DCMD, INP (mm, mm/s, G, G, mm), SPOW, DPOW, PPOW, CTLF.
Words profile_block(double mm, double speed, double accel_g, double decel_g, double band, int ctlf = 0) {
    const auto p = reg::to_i32_words(static_cast<std::int32_t>(reg::to_counts(mm)));
    const auto v = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(speed)));
    const auto a = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(accel_g)));
    const auto d = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(decel_g)));
    const auto b = reg::to_u32_words(static_cast<std::uint32_t>(reg::to_counts(band)));
    return {p.first, p.second, v.first, v.second, a.first, a.second, d.first, d.second, b.first, b.second,
            114,     255,      0,       static_cast<std::uint16_t>(ctlf), 0, 0, 0, 0, 0, 0};
}

}  // namespace

TEST(sim_profile_reads_back_and_runs_only_with_pcrf) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    const Words block = profile_block(100.0, 50.0, 0.30, 0.10, 0.10);
    CHECK_EQ(write_regs(unit, reg::kProfileBase, block), 0);
    CHECK_EQ(hexs(read_regs(unit, reg::kProfileBase, reg::kProfileWords)), hexs(block));
    rig.clock.advance(1.0);
    CHECK(!unit.moving());  // a write alone does not move (VERIFIED on the PCON in the Python edition)
    CHECK_EQ(unit.profile_moves(), 0);
    CHECK_EQ(coil(unit, reg::kCoilPcrf, true), 0);
    CHECK(unit.moving());
    CHECK(!unit.coil_state(reg::kCoilPcrf));  // self-clearing
    CHECK_EQ(unit.profile_moves(), 1);
    CHECK(contains(unit.events(), "profile move (PCRF)"));
    CHECK_NEAR(unit.command_speed_mm_s(), 50.0, 1e-9);
    rig.clock.advance(10.0);
    CHECK_NEAR(unit.position_mm(), 100.0, 1e-6);
    CHECK(!unit.moving());
}

TEST(sim_profile_fc17_writes_then_reads_the_monitor) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    std::string pdu = "17 90 00 00 02 0F 00 00 14 28";
    for (std::uint16_t w : profile_block(20.0, 100.0, 0.30, 0.20, 0.10))
        pdu += util::format(" %02X %02X", w >> 8, w & 0xFF);
    const Bytes answer = sim_test::exchange(unit, pdu);  // (not std::exchange by ADL)
    CHECK_EQ(hexs(pdu_of(answer)), hexs(Bytes{0x17, 0x04, 0x00, 0x00, 0x1C, 0xE3}));  // PNOW 73.95: still at rest
    CHECK_NEAR(pnow_mm(read_regs(unit, reg::kProfileBase, 2)), 20.0, 1e-9);
    CHECK_EQ(coil(unit, reg::kCoilPcrf, true), 0);
    rig.clock.advance(0.05);
    CHECK(unit.moving());
    CHECK_EQ(coil(unit, reg::kCoilStop, true), 0);  // STOP stops it
    rig.clock.advance(2.0);
    CHECK(!unit.moving());
    CHECK(unit.position_mm() > 20.5);
    CHECK_EQ(coil(unit, reg::kCoilStop, false), 0);
}

TEST(sim_profile_defaults_and_volatility) {
    Rig rig(pure_trapezoid(SimProfile::Pcon));
    SimulatedUnit& unit = rig.u();
    const Words before = read_regs(unit, reg::kProfileBase, reg::kProfileWords);
    CHECK_EQ(before[reg::kProfileSpow], 0x59);  // the PCON's SPOW / DPOW
    CHECK_EQ(before[reg::kProfileDpow], 0xFF);
    Rig ec(pure_trapezoid(SimProfile::Slider));
    const Words ec_block = read_regs(ec.u(), reg::kProfileBase, reg::kProfileWords);
    CHECK_EQ(ec_block[reg::kProfileSpow], 114);  // an EC's (live)
    CHECK_EQ(ec_block[reg::kProfileDpow], 255);
    CHECK_EQ(write_regs(unit, reg::kProfileBase, profile_block(30.0, 40.0, 0.2, 0.1, 0.1)), 0);
    CHECK_NEAR(pnow_mm(read_regs(unit, reg::kProfileBase, 2)), 30.0, 1e-9);
    unit.power_cycle();  // the block is volatile: it shows what the last move applied again
    CHECK(pnow_mm(read_regs(unit, reg::kProfileBase, 2)) != 30.0);
    CHECK_EQ(read_regs(unit, reg::kProfileBase, reg::kProfileWords)[reg::kProfileSpow], 0x59);
}

TEST(sim_profile_accel_pattern_of_each_start) {
    Rig rig(pure_trapezoid());
    SimulatedUnit& unit = rig.u();
    CHECK_EQ(unit.accel_pattern(), std::string("trapezoid"));
    CHECK_EQ(begin_move(unit, 80.0, 50.0, 0.3, 0.1, 0, reg::kCtlfModSMotion), 0);
    CHECK_EQ(unit.accel_pattern(), std::string("s_motion"));
    rig.clock.advance(5.0);
    CHECK_EQ(begin_move(unit, 90.0, 50.0, 0.3, 0.1, 0, reg::kCtlfModFilter), 0);
    CHECK_EQ(unit.accel_pattern(), std::string("filter"));
    rig.clock.advance(5.0);
    CHECK_EQ(begin_move(unit, 70.0, 50.0, 0.3, 0.1, 0, reg::kCtlfModMask), 0);
    CHECK_EQ(unit.accel_pattern(), std::string("invalid"));
    rig.clock.advance(5.0);
    CHECK_NEAR(unit.position_mm(), 70.0, 1e-6);  // the motion itself stays a trapezoid
    CHECK_EQ(begin_move(unit, 60.0, 50.0, 0.3), 0);
    CHECK_EQ(unit.accel_pattern(), std::string("trapezoid"));
}
