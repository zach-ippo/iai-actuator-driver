// Axis unit: configuration on the simulator - the position-table guards and verify, teach (refused before homing),
// the parameter guards and the commit handshake, P1 narrowing the stroke, the software reset (the rod loses HEND) and
// the baud change on a simulator with baud modelling.
#include <memory>
#include <string>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile, double scale = 20.0) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = scale;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim, AxisOptions axis = {}) {
    OpenOptions o;
    o.slave = sim->slave();
    o.axis = axis;
    return Axis::open(sim, o);
}

int write_frames(const std::vector<Bytes>& requests) {
    int n = 0;
    for (const auto& f : requests)
        if (f.size() > 1 && (f[1] == 0x05 || f[1] == 0x06 || f[1] == 0x10 || f[1] == 0x45)) ++n;
    return n;
}

int fc10_frames(const std::vector<Bytes>& requests) {
    int n = 0;
    for (const auto& f : requests)
        if (f.size() > 1 && f[1] == 0x10) ++n;
    return n;
}

NvWriteOptions nv(bool allow, bool force = false) {
    NvWriteOptions o;
    o.allow_nonvolatile = allow;  // accepted and ignored (no longer needed)
    o.force = force;
    return o;
}

template <class F>
std::string what_of(F&& f) {
    try {
        f();
    } catch (const std::exception& e) {
        return e.what();
    }
    return std::string();
}

bool any_has(const std::vector<std::string>& texts, const std::string& part) {
    for (const auto& t : texts)
        if (t.find(part) != std::string::npos) return true;
    return false;
}

}  // namespace

TEST(axis_config_position_table_guards_and_verify) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    ax.connect();
    const PositionEntry row2 = ax.read_position_table(2);
    CHECK_NEAR(row2.position_mm, 75.0, 0.001);
    sim->clear_requests();
    PositionEntry changed = row2;
    changed.position_mm = 60.0;
    CHECK_THROWS(ax.write_position_table(256, changed), NonVolatileWriteError);  // simple-data store
    CHECK_THROWS(ax.write_position_table(-1, changed), NonVolatileWriteError);   // a row the table cannot address
    PositionEntry outside = row2;
    outside.position_mm = 400.0;
    CHECK_THROWS(ax.write_position_table(2, outside), SoftLimitError);
    PositionEntry fast = row2;
    fast.speed_mm_s = 5000.0;
    CHECK_THROWS(ax.write_position_table(2, fast), SoftLimitError);
    NvWriteOptions unsafe = nv(true);
    unsafe.allow_unsafe = true;  // accepted and ignored: it lifts no range check
    PositionEntry strong = row2;
    strong.push_power = 0xFFFF;
    CHECK_THROWS(ax.write_position_table(2, strong, unsafe), SoftLimitError);
    PositionEntry heavy = row2;
    heavy.load_threshold = 300;
    CHECK_THROWS(ax.write_position_table(2, heavy, unsafe), SoftLimitError);
    CHECK_EQ(write_frames(sim->requests()), 0);
    CHECK_EQ(sim->nonvolatile_writes(), 0);
    const PositionWriteResult r = ax.write_position_table(2, changed);  // no opt-in needed
    CHECK(!r.skipped);
    CHECK_EQ(r.row, 2);
    CHECK(r.warnings.empty());
    CHECK_NEAR(r.entry.position_mm, 60.0, 0.001);
    CHECK(r.entry.checksum_ok().value_or(false));
    CHECK_EQ(sim->nonvolatile_writes(), 1);
    CHECK_EQ(ax.nonvolatile_writes(), 1);
    CHECK_NEAR(ax.read_position_table(2).position_mm, 60.0, 0.001);
    CHECK_EQ(sim->position_row(2).size(), 16u);
    // beyond the unit's rows and a change of the unknown role bits: written, with a warning
    const PositionWriteResult past = ax.write_position_table(5, changed);
    CHECK(!past.skipped);
    CHECK(any_has(past.warnings, "row 5 is past them: written anyway"));
    PositionEntry role = changed;
    role.control_flags = 0x0010;  // row 2 carries 0x0020: the unknown role bits change
    const PositionWriteResult roles = ax.write_position_table(2, role, nv(true));  // the old opt-in: ignored
    CHECK(!roles.skipped);
    CHECK(any_has(roles.warnings, "change the unknown role bits 0x0030 of the unit's 0x0020"));
    CHECK_EQ(ax.read_position_table(2).control_flags, 0x0010);
    CHECK_EQ(sim->nonvolatile_writes(), 3);
}

TEST(axis_config_teach_refused_before_homing) {
    auto sim = make_sim(SimProfile::Rod);
    sim->power_cycle();  // an incremental unit loses its home
    Axis ax = open_sim(sim);
    CHECK(!ax.status().homed);
    sim->clear_requests();
    CHECK_THROWS(ax.teach_position(0), EcNotReadyError);  // no opt-in needed: not homed
    CHECK_THROWS(ax.teach_position(0, nv(true)), EcNotReadyError);
    CHECK_EQ(write_frames(sim->requests()), 0);
    ax.connect();
    ax.servo(true);
    ax.home();
    ax.move_to(40.0, 100.0);
    const PositionWriteResult r = ax.teach_position(0);
    CHECK(!r.skipped);
    CHECK_NEAR(r.entry.position_mm, 40.0, 0.011);
    CHECK_EQ(sim->nonvolatile_writes(), 1);
    CHECK_THROWS(ax.teach_position(7, nv(true)), SoftLimitError);
}

TEST(axis_config_parameter_guards_send_nothing) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    sim->clear_requests();
    CHECK_THROWS(ax.write_parameter("P2", ParamValue::parse("5.0")), SoftLimitError);    // outside 0.01..1.00
    CHECK_THROWS(ax.write_parameter("P99", ParamValue::parse("1")), UnsafeCommandError);
    CHECK_THROWS(ax.write_parameter("not a key", ParamValue::parse("1")), UnsafeCommandError);
    CHECK_EQ(fc10_frames(sim->requests()), 0);
    CHECK_EQ(sim->parameter_commits(), 0);
    CHECK_EQ(sim->parameter_ram_writes(), 0);
    // a home parameter needs no opt-in: written and committed, with the home warning
    const ParameterWriteResult home = ax.write_parameter("P3", ParamValue::parse("0"));
    CHECK(home.changed);
    CHECK(any_has(home.warnings, "P3 / P4 are home parameters"));
    CHECK_EQ(sim->parameter_commits(), 1);
}

TEST(axis_config_parameter_write_is_verified_and_committed) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    const ParameterInfo before = ax.read_parameter("P2");
    REQUIRE(before.value.number.has_value());
    CHECK_NEAR(*before.value.number, 0.10, 1e-9);
    const ParameterWriteResult r = ax.write_parameter(2, 0.20);
    CHECK(r.changed);
    CHECK(r.verified);
    CHECK(r.committed);
    CHECK(r.commit_confirmed);
    CHECK(r.saved());
    CHECK(!r.skipped);
    CHECK_EQ(r.number, 2);
    CHECK(r.address.rfind("0x", 0) == 0);
    CHECK_EQ(sim->parameter_commits(), 1);
    CHECK_EQ(ax.nonvolatile_writes(), 1);
    const ParameterInfo after = ax.read_parameter(2);
    REQUIRE(after.value.number.has_value());
    CHECK_NEAR(*after.value.number, 0.20, 1e-9);
    // RAM only: written and verified, nothing committed
    const ParameterWriteResult ram = ax.write_parameter(2, 0.30, [] {
        ParameterWriteOptions o;
        o.commit = false;
        return o;
    }());
    CHECK(ram.changed);
    CHECK(!ram.committed);
    CHECK(ax.has_uncommitted_parameters());
    CHECK_EQ(sim->parameter_commits(), 1);
}

TEST(axis_config_unchanged_ram_only_write_is_not_called_ram_only) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    ParameterWriteOptions o;
    o.commit = false;
    const ParameterWriteResult first = ax.write_parameter(2, 0.30, o);
    CHECK(first.changed);
    CHECK(first.note.rfind("RAM ONLY - NOT SAVED", 0) == 0);
    sim->clear_requests();
    const ParameterWriteResult again = ax.write_parameter(2, 0.30, o);  // the same value: nothing is written
    CHECK(!again.changed);
    CHECK_EQ(fc10_frames(sim->requests()), 0);
    CHECK_EQ(again.note, std::string(kRamUnchangedNote));
}

TEST(axis_config_con_pio_pattern_above_mxpt_is_refused) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.connect();
    const ParameterInfo mxpt = ax.read_parameter("3.16");
    REQUIRE(mxpt.value.number.has_value());
    REQUIRE(*mxpt.value.number > 0);
    sim->clear_requests();
    const std::string text = what_of([&] { ax.write_parameter(25, *mxpt.value.number + 1); });
    CHECK(text.find("MXPT") != std::string::npos);
    CHECK_THROWS(ax.write_parameter(25, 32767), SoftLimitError);
    CHECK_EQ(fc10_frames(sim->requests()), 0);
}

TEST(axis_config_con_pio_pattern_below_zero_is_refused) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.connect();
    sim->clear_requests();
    const std::string text = what_of([&] { ax.write_parameter(25, -1); });
    CHECK(text.find("below the lowest pattern 0") != std::string::npos);
    CHECK_THROWS(ax.write_parameter(25, -1), SoftLimitError);
    CHECK_EQ(fc10_frames(sim->requests()), 0);
}

TEST(axis_config_con_inverted_soft_limits_are_refused) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.connect();
    const double minus = *ax.read_parameter(4).value.number;
    sim->clear_requests();
    CHECK_THROWS(ax.write_parameter(3, minus - 1.0), SoftLimitError);   // No.3 below No.4
    CHECK_THROWS(ax.write_parameter(4, 100000.0), SoftLimitError);
    const std::string text = what_of([&] { ax.write_parameter(3, minus - 1.0); });
    CHECK(text.find("inverted") != std::string::npos);
    CHECK_EQ(fc10_frames(sim->requests()), 0);
}

TEST(axis_config_con_soft_limit_below_the_position_warns) {
    auto sim = make_sim(SimProfile::Pcon);
    Axis ax = open_sim(sim);
    ax.connect();
    ax.move_to(5.0, 20.0);
    ParameterWriteOptions o;
    o.commit = false;
    const ParameterWriteResult r = ax.write_parameter(3, 2.0, o);
    CHECK(r.changed);
    CHECK(any_has(r.warnings, "0x0D9"));
}

TEST(axis_config_parameters_list_ec_and_con) {
    auto gripper = make_sim(SimProfile::Gripper);
    Axis ec = open_sim(gripper);
    const auto list = ec.parameters();
    REQUIRE(!list.empty());
    bool p2 = false;
    for (const auto& p : list)
        if (p.number == 2) p2 = true;
    CHECK(p2);
    const ParameterModel model = ec.parameter_model();
    CHECK(!model.listed.empty());
    auto pcon = make_sim(SimProfile::Pcon);
    Axis con = open_sim(pcon);
    const auto con_list = con.parameters();
    REQUIRE(!con_list.empty());
    CHECK_EQ(con_list.front().family, "CON");
    const ParameterInfo raw = con.read_parameter("8.4");
    CHECK(raw.raw.has_value());
}

TEST(axis_config_p1_narrows_the_stroke_at_once) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    ax.move_to(5.0, 20.0);
    ParameterWriteOptions o;
    o.commit = false;
    const ParameterWriteResult r = ax.write_parameter(1, 8.0, o);
    CHECK(r.changed);
    const auto eff = ax.effective_stroke();
    REQUIRE(eff.has_value());
    CHECK_NEAR(eff->max, 8.0, 0.011);
    CHECK_THROWS(ax.move_to(9.0, 20.0), SoftLimitError);
    CHECK_THROWS(ax.write_parameter(1, 20.0, o), SoftLimitError);  // P1 can only narrow (up to the largest P1 read)
    CHECK_THROWS(ax.write_parameter(1, 4.0, o), SoftLimitError);   // not below the current position
}

TEST(axis_config_software_reset_rod_loses_hend) {
    auto sim = make_sim(SimProfile::Rod);
    Axis ax = open_sim(sim);
    ax.connect();
    CHECK(ax.status().homed);
    ParameterWriteOptions ram;
    ram.commit = false;
    ax.write_parameter(2, 0.20, ram);
    CHECK(ax.has_uncommitted_parameters());
    SoftwareResetOptions o;
    o.reset_wait_s = 0.1;
    o.timeout_s = 5.0;
    const SoftwareResetResult r = ax.software_reset(o);
    CHECK(r.confirmed);
    CHECK(!r.confirmed_by.empty());
    CHECK(!r.homed());
    CHECK(r.needs_home());
    CHECK(!ax.has_uncommitted_parameters());
    CHECK_EQ(sim->software_resets(), 1);
    CHECK(!ax.status().homed);
    CHECK_THROWS(ax.move_to(10.0, 20.0), EcNotReadyError);
}

TEST(axis_config_software_reset_refused_while_moving) {
    auto sim = make_sim(SimProfile::Slider, 1.0);
    Axis ax = open_sim(sim);
    MoveOptions o;
    o.speed = 50.0;
    o.wait = false;
    ax.move_to(140.0, o);
    sleep_at_least(0.05);
    CHECK_THROWS(ax.software_reset(), EcNotReadyError);
    ax.stop();
    CHECK_EQ(sim->software_resets(), 0);
}

TEST(axis_config_set_baud_rate_38400_without_the_opt_in) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Rod;
    so.time_scale = 20.0;
    so.baud_rate = 115200;
    so.ignored_baud_codes = {};  // a unit that takes code 0x06 (EC-GRBP8M, PCON-CB)
    auto takes = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(takes);
    const BaudChangeResult r = ax.set_baud_rate(38400);  // no allow_unverified_code
    CHECK_EQ(r.code, 0x06);
    CHECK_EQ(r.baud_rate, 38400);
    CHECK(r.verified);
    CHECK(r.code_verified);
    CHECK_EQ(ax.baud_rate().value_or(0), 38400);
    CHECK_EQ(takes->unit_baud_rate().value_or(0), 38400);
    // the default simulated unit ignores the code (EC-R6S rod): sent anyway, the check after the switch refuses
    so.ignored_baud_codes = {0x06};
    auto ignores = std::make_shared<SimulatedUnit>(so);
    Axis rod = Axis::open(ignores);
    try {
        rod.set_baud_rate(38400);
        CHECK(false);
    } catch (const VerificationError& e) {
        CHECK(std::string(e.what()).find("this unit ignores the 38400 code") != std::string::npos);
    }
    CHECK_EQ(rod.baud_rate().value_or(0), 115200);
}

TEST(axis_config_set_baud_rate_on_a_modelled_line) {
    SimulatedUnitOptions so;
    so.profile = SimProfile::Rod;
    so.time_scale = 20.0;
    so.baud_rate = 38400;
    auto sim = std::make_shared<SimulatedUnit>(so);
    Axis ax = Axis::open(sim);
    CHECK_EQ(ax.baud_rate().value_or(0), 38400);
    sim->clear_requests();
    CHECK_THROWS(ax.set_baud_rate(12345), SoftLimitError);  // no IAI code
    CHECK_EQ(write_frames(sim->requests()), 0);
    const BaudChangeResult r = ax.set_baud_rate(115200);
    CHECK_EQ(r.previous, 38400);
    CHECK_EQ(r.baud_rate, 115200);
    CHECK(r.verified);
    CHECK_EQ(ax.baud_rate().value_or(0), 115200);
    CHECK_EQ(sim->unit_baud_rate().value_or(0), 115200);
    CHECK_NEAR(ax.status().position_mm, sim->position_mm(), 0.011);
    // an UNVERIFIED code is sent (no opt-in needed): code_verified false
    const BaudChangeResult unverified = ax.set_baud_rate(57600);
    CHECK(!unverified.code_verified);
    CHECK_EQ(unverified.baud_rate, 57600);
    CHECK_EQ(sim->unit_baud_rate().value_or(0), 57600);
    // no baud rate to change on an in-memory unit; a shared Port uses Port::set_baud_rate
    auto plain = make_sim(SimProfile::Rod);
    Axis other = open_sim(plain);
    CHECK_THROWS(other.set_baud_rate(115200), EcNotReadyError);
    Port port(sim, "LINE");
    Axis shared(port, 1);
    CHECK_THROWS(shared.set_baud_rate(230400), EcNotReadyError);
}
