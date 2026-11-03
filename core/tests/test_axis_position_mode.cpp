// Axis unit: the findings of the full feature test on the EC-GRBP8M gripper (2-position mode, P9 = 0), on the
// simulator. Port of the Python edition's tests/test_gripper_fulltest.py (the conformance cases):
//   1. Stored rows follow the position mode: row 2 is not run in 2-position mode (the gripper rejected 0x45 cmd 6 row 2
//      with 0x11 "1st operand error"); the driver refuses it before anything is sent and translates the rejection.
//   2. A same-value teach / row write is skipped before the speed cap (a row stored at 53 mm/s on a 10 mm/s axis); a
//      refused value names the row, the field, its unit and whose cap it is.
//   3. A software reset keeps PMSS 1 on the gripper; the restart is confirmed by the servo coming back on by itself.
// VERIFIED in the Python edition on the EC-GRBP8M gripper (2026-10-29); this edition is UNVERIFIED on hardware.
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "harness.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/sim.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

const std::string kInactive = "row 2 (middle) is not used in 2-position mode (P9 = 0";

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile = SimProfile::Gripper) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = 50.0;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim, AxisOptions axis = {}) {
    OpenOptions o;
    o.slave = sim->slave();
    axis.poll_interval_s = 0.005;
    o.axis = axis;
    o.bus.min_gap_s = 0.0;
    return Axis::open(sim, o);
}

AxisOptions capped(double max_speed) {
    AxisOptions a;
    a.max_speed_mm_s = max_speed;
    return a;
}

/// Frames that command the unit: FC05 / FC06 / FC10 writes and 0x45 / 0x48 commands.
int command_frames(const std::vector<Bytes>& requests) {
    int n = 0;
    for (const auto& f : requests)
        if (f.size() > 1 && (f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x10 || f[1] == 0x45 || f[1] == 0x48)) ++n;
    return n;
}

NvWriteOptions nv(bool unsafe = false, bool force = false) {
    NvWriteOptions o;
    o.allow_nonvolatile = true;
    o.allow_unsafe = unsafe;
    o.force = force;
    return o;
}

PositionMoveOptions pct(double speed_pct, bool above_caps = false) {
    PositionMoveOptions o;
    o.speed_pct = speed_pct;
    o.allow_above_caps = above_caps;
    return o;
}

/// The message of the SoftLimitError `f` throws ("<no SoftLimitError>" when it returns).
template <class F>
std::string refusal(F&& f) {
    try {
        f();
    } catch (const SoftLimitError& e) {
        return e.message();
    }
    return "<no SoftLimitError>";
}

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

bool starts(const std::string& text, const std::string& prefix) { return text.rfind(prefix, 0) == 0; }

/// Write `raw` into the unit's RAM copy of item `key` (no commit) with a raw FC10, as a unit set up by IA-OS would
/// hold it: the driver reads it like any other value.
void poke_item(Axis& ax, SimulatedUnit& sim, const std::string& key, std::uint32_t raw) {
    const ParameterInfo item = ax.read_parameter(key);
    REQUIRE(item.address.has_value());
    const int address = std::stoi(*item.address, nullptr, 16);
    const int words = item.words.value_or(2);
    Bytes pdu{0x10, static_cast<std::uint8_t>(address >> 8), static_cast<std::uint8_t>(address & 0xFF), 0x00,
              static_cast<std::uint8_t>(words), static_cast<std::uint8_t>(2 * words)};
    const std::vector<std::uint16_t> data =
        words == 2 ? std::vector<std::uint16_t>{static_cast<std::uint16_t>(raw >> 16), static_cast<std::uint16_t>(raw)}
                   : std::vector<std::uint16_t>{static_cast<std::uint16_t>(raw)};
    for (std::uint16_t w : data) {
        pdu.push_back(static_cast<std::uint8_t>(w >> 8));
        pdu.push_back(static_cast<std::uint8_t>(w & 0xFF));
    }
    const Bytes answer = sim.handle_frame(build_frame(sim.slave(), pdu));
    REQUIRE(answer.size() > 1 && answer[1] == 0x10);
}

}  // namespace

// ------------------------------------------------------------------------------------------ 1. rows and the mode

TEST(axis_position_mode_row_2_refused_before_anything_is_sent) {
    auto sim = make_sim();
    Axis ax = open_sim(sim);
    ax.connect(true);
    for (const PositionMoveOptions& o : {PositionMoveOptions{}, pct(18.0)}) {
        sim->clear_requests();
        const std::string text = refusal([&] { ax.move_to_position(2, o); });
        CHECK(has(text, kInactive));
        CHECK(has(text, "runs row 0 (backward end) and row 1 (forward end) only"));
        CHECK(has(text, "nothing sent"));
        CHECK_EQ(command_frames(sim->requests()), 0);
    }
    const auto st = ax.move_to_position(0, pct(18.0));  // an active row still runs
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 3.0, 0.01);
    CHECK_EQ(sim->override_moves(), 1);
}

TEST(axis_position_mode_rejection_translated_when_the_mode_is_unknown) {
    auto sim = make_sim();
    Axis ax = open_sim(sim);
    ax.connect(true);
    poke_item(ax, *sim, "8.2", 7);  // P9 holds a value the driver does not know: the row is sent
    const double before = ax.status().position_mm;
    std::string text;
    bool nested = false;
    std::string hint;
    try {
        ax.move_to_position(2, pct(18.0, true));
    } catch (const SoftLimitError& e) {
        text = e.message();
        try {
            std::rethrow_if_nested(e);
        } catch (const IaiCommandRejected& r) {
            nested = r.rejection() == 0x11;
            hint = r.hint();
        }
    }
    CHECK(has(text, "row 2 (middle) is not used in the unit's position mode"));
    CHECK(has(text, "0x45 cmd 6 code 0x11 1st operand error"));
    CHECK(has(text, "read P9 with read_parameter(9)"));
    CHECK(nested);  // the rejection is kept, nested
    CHECK(has(hint, "2-position mode"));
    CHECK_NEAR(ax.status().position_mm, before, 0.01);  // nothing moved
    CHECK_EQ(sim->override_moves(), 0);
}

TEST(axis_position_mode_three_position_slider_runs_row_2) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const auto st = ax.move_to_position(2, pct(50.0, true));
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, 75.0, 0.01);
}

TEST(axis_position_mode_con_rows_are_not_limited_by_p9) {
    // class 8 item 2 is another parameter on a CON unit: its rows are what MAXP / the directory say
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.connect(true);
    CHECK_EQ(ax.max_positions(), 64);
    const PositionEntry row2 = ax.read_position_table(2);
    CHECK(ax.write_position_table(2, row2, nv()).skipped);  // not refused: skipped as unchanged
    const auto st = ax.move_to_position(2, pct(10.0, true));
    REQUIRE(st.has_value());
    CHECK_NEAR(st->position_mm, row2.position_mm, 0.1);
}

TEST(axis_position_mode_teach_and_write_row_2_with_a_warning) {
    // allow_unsafe is no longer needed: the row the 2-position mode does not run is written, with a warning
    auto sim = make_sim();
    Axis ax = open_sim(sim);
    ax.connect(true);
    const PositionEntry row2 = ax.read_position_table(2);
    PositionEntry moved = row2;
    moved.position_mm = 5.0;
    const PositionWriteResult back = ax.write_position_table(2, moved, nv());
    CHECK(!back.skipped);
    CHECK_NEAR(back.entry.position_mm, 5.0, 0.001);
    REQUIRE(back.warnings.size() == 1u);
    CHECK(has(back.warnings[0], kInactive));
    CHECK(has(back.warnings[0], "written anyway (e.g. to prepare a switch to 3-position mode)"));
    CHECK_EQ(ax.nonvolatile_writes(), 1);
    const PositionWriteResult taught = ax.teach_position(2, nv(true));  // the old opt-in: accepted, ignored
    CHECK(!taught.skipped);
    REQUIRE(!taught.warnings.empty());
    CHECK(has(taught.warnings[0], kInactive));
    CHECK_EQ(ax.nonvolatile_writes(), 2);
}

TEST(axis_position_mode_cli_goto_2_is_refused) {
    std::ostringstream out, err;
    const int code = cli::run({"--sim", "--sim-profile", "gripper", "--sim-time-scale", "50", "goto", "2",
                               "--speed-pct", "18"},
                              out, err);
    CHECK_EQ(code, 2);
    CHECK(has(err.str(), kInactive));
}

// ------------------------------------------------------------------------------------------ 2. unchanged rows

// The gripper's row 0 is stored at 53 mm/s; the axis is opened with a 10 mm/s cap.

TEST(axis_position_mode_same_row_write_is_skipped_before_the_cap) {
    auto sim = make_sim();
    Axis ax = open_sim(sim, capped(10.0));
    ax.connect(true);
    const PositionEntry row0 = ax.read_position_table(0);
    CHECK_NEAR(row0.speed_mm_s, 53.0, 1e-9);
    sim->clear_requests();
    CHECK(ax.write_position_table(0, row0, nv()).skipped);
    CHECK_EQ(command_frames(sim->requests()), 0);
    PositionEntry moved = row0;
    moved.position_mm = 3.5;  // a real change: the cap applies, with context
    CHECK_EQ(refusal([&] { ax.write_position_table(0, moved, nv()); }),
             std::string("row 0 stored speed 53 mm/s is above this axis's max_speed_mm_s 10 mm/s; open the axis with "
                         "a higher max_speed_mm_s (command line: --max-speed or --unit-max) to write it"));
    PositionEntry faster = row0;
    faster.speed_mm_s = 20.0;
    CHECK(starts(refusal([&] { ax.write_position_table(0, faster, nv()); }),
                 "row 0 speed 20 mm/s is above this axis's max_speed_mm_s"));
    CHECK_EQ(ax.nonvolatile_writes(), 0);
}

TEST(axis_position_mode_same_value_teach_is_skipped_before_the_cap) {
    auto sim = make_sim();
    Axis ax = open_sim(sim, capped(10.0));
    ax.connect(true);
    ax.move_to(3.0, 10.0);  // row 0 holds 3.00 mm
    sim->clear_requests();
    CHECK(ax.teach_position(0, nv()).skipped);
    CHECK_EQ(command_frames(sim->requests()), 0);
    ax.move_to(3.5, 10.0);
    const std::string text = refusal([&] { ax.teach_position(0, nv()); });
    CHECK(has(text, "row 0 stored speed 53 mm/s"));
    CHECK(has(text, "--unit-max"));
    CHECK_EQ(ax.nonvolatile_writes(), 0);
}

TEST(axis_position_mode_force_still_checks_every_value) {
    auto sim = make_sim();
    Axis ax = open_sim(sim, capped(10.0));
    ax.connect(true);
    const PositionEntry row0 = ax.read_position_table(0);
    CHECK(has(refusal([&] { ax.write_position_table(0, row0, nv(false, true)); }), "max_speed_mm_s 10"));
    PositionEntry slow = row0;
    slow.speed_mm_s = 0.0;
    slow.position_mm = 3.5;
    CHECK_EQ(refusal([&] { ax.write_position_table(0, slow, nv()); }),
             std::string("row 0 speed 0 mm/s: a row needs a speed above 0"));
    PositionEntry outside = row0;
    outside.position_mm = 12.0;
    CHECK(has(refusal([&] { ax.write_position_table(0, outside, nv()); }),
              "row 0 position 12 mm is outside the stroke"));
}

// ------------------------------------------------------------------------------------------ 3. software reset

TEST(axis_position_mode_gripper_reset_keeps_pmss_and_is_confirmed_by_the_servo) {
    auto sim = make_sim();
    Axis ax = open_sim(sim);
    ax.connect(true);
    SoftwareResetOptions o;
    o.reset_wait_s = 0.0;
    const SoftwareResetResult r = ax.software_reset(o);
    CHECK(r.confirmed);
    CHECK(r.modbus_enabled());
    CHECK(!has(r.confirmed_by, "PMSS 1 -> 0"));
    CHECK(has(r.confirmed_by, "the servo came back on by itself"));
    bool named = false;
    for (const auto& note : r.notes) named = named || has(note, "EC-GRBP8M");
    CHECK(named);
}
