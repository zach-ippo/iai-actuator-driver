// Axis unit: the non-volatile write guard (new in all editions). The units' non-volatile memory lasts about 100,000
// write cycles (IAI): an unchanged row or parameter is not written (skipped, kNvUnchangedNote) unless force; a parameter
// commit still runs for this axis's uncommitted RAM writes (PARW only); every real write is counted; above 60 per hour
// one warning per hour, never a refusal. Simulator only.
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::shared_ptr<SimulatedUnit> make_sim(SimProfile profile) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.time_scale = 20.0;
    return std::make_shared<SimulatedUnit>(o);
}

Axis open_sim(const std::shared_ptr<SimulatedUnit>& sim) {
    OpenOptions o;
    o.slave = sim->slave();
    return Axis::open(sim, o);
}

int count(const std::vector<Bytes>& requests, int fc, int address = -1) {
    int n = 0;
    for (const auto& f : requests)
        if (f.size() >= 4 && f[1] == fc && (address < 0 || ((f[2] << 8) | f[3]) == address)) ++n;
    return n;
}

NvWriteOptions nv(bool force = false) {
    NvWriteOptions o;
    o.allow_nonvolatile = true;
    o.force = force;
    return o;
}

ParameterWriteOptions pw(bool commit = true, bool force = false) {
    ParameterWriteOptions o;
    o.commit = commit;
    o.force = force;
    return o;
}

/// Captures the warnings of the axis while alive.
struct WarningCapture {
    std::mutex mutex;
    std::vector<std::string> messages;
    LogHandler previous;
    WarningCapture() {
        previous = set_log_handler([this](LogLevel level, const std::string& source, const std::string& message) {
            if (level != LogLevel::Warning || source != detail::kAxisLog) return;
            std::lock_guard<std::mutex> lock(mutex);
            messages.push_back(message);
        });
    }
    ~WarningCapture() { set_log_handler(previous); }
    std::vector<std::string> take() {
        std::lock_guard<std::mutex> lock(mutex);
        auto out = messages;
        messages.clear();
        return out;
    }
};

}  // namespace

TEST(axis_nv_guard_unchanged_row_is_not_written) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const PositionEntry stored = ax.read_position_table(2);
    sim->clear_requests();
    const PositionWriteResult r = ax.write_position_table(2, stored, nv());
    CHECK(r.skipped);
    CHECK(!r.forced);
    CHECK_EQ(r.note, kNvUnchangedNote);
    CHECK(r.entry == stored);
    CHECK_EQ(count(sim->requests(), 0x10), 0);
    CHECK_EQ(sim->nonvolatile_writes(), 0);
    CHECK_EQ(ax.nonvolatile_writes(), 0);
    // force writes anyway, and counts
    const PositionWriteResult forced = ax.write_position_table(2, stored, nv(true));
    CHECK(!forced.skipped);
    CHECK(forced.forced);
    CHECK_EQ(forced.note, kNvForcedNote);
    CHECK_EQ(sim->nonvolatile_writes(), 1);
    CHECK_EQ(ax.nonvolatile_writes(), 1);
    CHECK_EQ(ax.nonvolatile_writes_last_hour(), 1);
    // a changed row is written and counted
    PositionEntry moved = stored;
    moved.position_mm = 61.0;
    const PositionWriteResult w = ax.write_position_table(2, moved, nv());
    CHECK(!w.skipped);
    CHECK_EQ(w.note, "");
    CHECK_EQ(ax.nonvolatile_writes(), 2);
}

TEST(axis_nv_guard_the_guards_still_come_first) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    const PositionEntry stored = ax.read_position_table(2);
    CHECK_THROWS(ax.write_position_table(256, stored), NonVolatileWriteError);  // simple-data store: refused first
    CHECK(ax.write_position_table(2, stored).skipped);  // no opt-in needed: unchanged, skipped
    CHECK_EQ(sim->nonvolatile_writes(), 0);
}

TEST(axis_nv_guard_teach_at_the_stored_position_is_skipped) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    ax.move_to(75.0, 50.0);  // row 2 holds 75.00
    const PositionWriteResult r = ax.teach_position(2, nv());
    CHECK(r.skipped);
    CHECK_EQ(r.note, kNvUnchangedNote);
    CHECK_EQ(sim->nonvolatile_writes(), 0);
    const PositionWriteResult f = ax.teach_position(2, nv(true));
    CHECK(f.forced);
    CHECK_EQ(sim->nonvolatile_writes(), 1);
}

TEST(axis_nv_guard_unchanged_parameter_commits_nothing) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    sim->clear_requests();
    const ParameterWriteResult r = ax.write_parameter(2, 0.10);  // P2 is 0.10 on the simulated gripper
    CHECK(r.skipped);
    CHECK(!r.changed);
    CHECK(!r.committed);
    CHECK(r.verified);
    CHECK_EQ(r.note, kNvUnchangedNote);
    CHECK(r.takes_effect().find("already holds") != std::string::npos);
    CHECK_EQ(count(sim->requests(), 0x10), 0);
    CHECK_EQ(count(sim->requests(), 0x05, reg::kCoilParw), 0);
    CHECK_EQ(sim->parameter_commits(), 0);
    CHECK_EQ(ax.nonvolatile_writes(), 0);
    // commit = false with an unchanged value: nothing asked of the memory (not "skipped"), nothing written
    const ParameterWriteResult ram = ax.write_parameter(2, 0.10, pw(false));
    CHECK(!ram.skipped);
    CHECK(!ram.committed);
    CHECK(!ax.has_uncommitted_parameters());
    CHECK_EQ(count(sim->requests(), 0x10), 0);
}

TEST(axis_nv_guard_force_commits_an_unchanged_value) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    sim->clear_requests();
    const ParameterWriteResult r = ax.write_parameter(2, 0.10, pw(true, true));
    CHECK(!r.skipped);
    CHECK(r.forced);
    CHECK(r.committed);
    CHECK(r.commit_confirmed);
    CHECK_EQ(count(sim->requests(), 0x10), 0);  // committed anyway, the unchanged item is not written
    CHECK_EQ(sim->parameter_commits(), 1);
    CHECK_EQ(ax.nonvolatile_writes(), 1);
}

TEST(axis_nv_guard_uncommitted_ram_write_then_commit_runs_parw_only) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    const ParameterWriteResult ram = ax.write_parameter(2, 0.20, pw(false));  // RAM only: waits for a commit
    CHECK(ram.changed);
    CHECK(!ram.committed);
    CHECK(ax.has_uncommitted_parameters());
    CHECK_EQ(sim->parameter_commits(), 0);
    sim->clear_requests();
    const ParameterWriteResult r = ax.write_parameter(2, 0.20);  // unchanged now, but the commit saves the RAM write
    CHECK(!r.skipped);
    CHECK(!r.changed);
    CHECK(r.committed);
    CHECK(r.commit_confirmed);
    CHECK_EQ(count(sim->requests(), 0x10), 0);                 // the item is not written again
    CHECK_EQ(count(sim->requests(), 0x05, reg::kCoilParw), 1);  // only PARW
    CHECK_EQ(sim->parameter_commits(), 1);
    CHECK(!ax.has_uncommitted_parameters());
    CHECK(ax.write_parameter(2, 0.20).skipped);  // nothing left uncommitted
    CHECK_EQ(sim->parameter_commits(), 1);
    CHECK_EQ(ax.nonvolatile_writes(), 1);
}

TEST(axis_nv_guard_software_reset_drops_pending_ram_writes) {
    auto sim = make_sim(SimProfile::Gripper);
    Axis ax = open_sim(sim);
    ax.connect();
    ax.write_parameter(2, 0.20, pw(false));
    CHECK(ax.has_uncommitted_parameters());
    SoftwareResetOptions o;
    o.reset_wait_s = 0.1;
    CHECK(ax.software_reset(o).confirmed);  // the restart reloaded the parameters: the RAM write is gone
    CHECK(!ax.has_uncommitted_parameters());
    CHECK(ax.write_parameter(2, 0.10).skipped);
    CHECK_EQ(sim->parameter_commits(), 0);
}

TEST(axis_nv_guard_one_warning_per_hour_never_a_refusal) {
    WarningCapture capture;
    detail::NvWriteTracker t("SIM slave 1");
    const double t0 = 1000.0;
    for (int i = 0; i < kNvWarnPerHour; ++i) CHECK(!t.record(t0 + i));
    CHECK(capture.take().empty());
    CHECK(t.record(t0 + 100.0));  // the 61st in the hour
    const auto messages = capture.take();
    REQUIRE(messages.size() == 1);
    CHECK(messages[0].find("61 non-volatile writes in the last hour") != std::string::npos);
    CHECK(messages[0].find("100,000 write cycles") != std::string::npos);
    CHECK(messages[0].find("use move_to (direct moves are RAM only)") != std::string::npos);
    CHECK(messages[0].find("SIM slave 1") != std::string::npos);
    for (int i = 0; i < 10; ++i) CHECK(!t.record(t0 + 200.0 + i));  // once per hour
    CHECK(capture.take().empty());
    CHECK_EQ(t.total(), kNvWarnPerHour + 11L);
    CHECK_EQ(t.last_hour(t0 + 300.0), kNvWarnPerHour + 11);
    // an hour later the old writes have left the window
    const double later = t0 + 100.0 + kNvWindowS + 300.0;
    CHECK_EQ(t.last_hour(later), 0);
    CHECK(!t.record(later));
    CHECK_EQ(t.last_hour(later), 1);
    int warned = 0;
    for (int i = 0; i < kNvWarnPerHour; ++i) warned += t.record(later + 1.0 + i) ? 1 : 0;
    CHECK_EQ(warned, 1);
    CHECK_EQ(capture.take().size(), 1u);
}

TEST(axis_nv_guard_axis_counter_warns_but_writes) {
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    WarningCapture capture;
    PositionEntry row = ax.read_position_table(2);
    int warnings = 0;
    for (int i = 0; i <= kNvWarnPerHour; ++i) {  // 61 real writes
        row.position_mm = 60.0 + (i % 2);
        const PositionWriteResult r = ax.write_position_table(2, row, nv());
        CHECK(!r.skipped);
    }
    for (const auto& m : capture.take())
        if (m.find("non-volatile writes in the last hour") != std::string::npos) ++warnings;
    CHECK_EQ(warnings, 1);
    CHECK_EQ(ax.nonvolatile_writes(), kNvWarnPerHour + 1L);
    CHECK_EQ(sim->nonvolatile_writes(), kNvWarnPerHour + 1);
}

TEST(axis_nv_guard_testing_hook_counts_like_a_real_write_and_is_shared_per_unit) {
    // (0.7) a binding's tests count writes at a time of their choosing; the unit's bookkeeping is shared
    auto sim = make_sim(SimProfile::Slider);
    Axis ax = open_sim(sim);
    Axis other = open_sim(sim);
    ax.share_nonvolatile_state("test:hook:1", "SIM slave 1");
    other.share_nonvolatile_state("test:hook:1", "SIM slave 1");
    WarningCapture capture;
    const double t0 = now_s();
    for (int i = 0; i < kNvWarnPerHour - 1; ++i) CHECK(!ax.testing_note_nonvolatile_write(t0, "test"));
    PositionEntry row = other.read_position_table(2);
    row.position_mm = 61.0;
    CHECK(!other.write_position_table(2, row, nv()).skipped);  // the 60th: no warning yet
    CHECK(capture.take().empty());
    CHECK(other.testing_note_nonvolatile_write(t0, "test"));  // the 61st, on the other object of the unit
    REQUIRE(capture.take().size() == 1u);
    CHECK_EQ(ax.nonvolatile_writes(), kNvWarnPerHour + 1L);
    CHECK_EQ(ax.testing_nonvolatile_writes_last_hour(t0 + 1.0), kNvWarnPerHour + 1);
    const double later = t0 + kNvWindowS + 10.0;
    CHECK_EQ(ax.testing_nonvolatile_writes_last_hour(later), 0);
    CHECK(!ax.testing_note_nonvolatile_write(later, "test"));
    CHECK_EQ(other.testing_nonvolatile_writes_last_hour(later), 1);
    CHECK_EQ(sim->nonvolatile_writes(), 1);  // the noted writes never reached the unit
    Axis::forget_nonvolatile_state("test:hook:");
}
