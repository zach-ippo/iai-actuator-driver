// C ABI 1.0 (the 0.8 layout reset): the structs (docs/NATIVE_ABI.md, "Sized structs"), on the simulator only:
//  - every input struct is sized (struct_size + reserved_header): a short struct, a set reserved_header, a set reserved
//    slot and a set byte past the library's own struct are refused, a bigger caller struct with zeroed extra bytes is
//    read;
//  - the _init functions and the sized outputs write the size they know (the first size for an unknown struct_size);
//  - arrays take their stride from the first element's struct_size;
//  - iad_axis_move has fields of its own (NaN / 0 = the defaults), a name that does not fit is refused, never cut;
//  - iad_cell_result.index and iad_event.index.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "iai_actuator_driver.h"
#include "harness.hpp"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

iad_axis* open_slider() {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_axis* ax = nullptr;
    REQUIRE(iad_axis_open_sim("slider", &sim, nullptr, nullptr, &ax) == IAD_OK);
    REQUIRE(iad_axis_connect(ax, 1, nullptr) == IAD_OK);
    return ax;
}

iad_cell* open_cell() {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 20.0;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_sim("slider,slider", 1, &sim, nullptr, nullptr, &cell) == IAD_OK);
    return cell;
}

}  // namespace

TEST(capi08_sized_inputs_refuse_what_they_cannot_read) {
    iad_axis* ax = open_slider();
    iad_move_options mo;
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    CHECK_EQ(mo.struct_size, static_cast<uint32_t>(sizeof mo));
    CHECK_EQ(mo.reserved_header, 0u);
    mo.speed = 100.0;
    CHECK_EQ(iad_axis_move_to(ax, 20.0, &mo, nullptr), IAD_OK);
    // below the first size (an uninitialised or a foreign struct): refused before anything is sent
    iad_move_options bad = mo;
    bad.struct_size = 8;
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &bad, nullptr), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "iad_move_options.struct_size is 8"));
    bad = mo;
    bad.reserved_header = 1;
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &bad, nullptr), IAD_ERR_INVALID_ARGUMENT);
    // a reserved slot set: a later ABI's field, never ignored
    bad = mo;
    bad.reserved0 = 1;
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &bad, nullptr), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "iad_move_options.reserved0"));
    bad = mo;
    bad.reserved[1] = 7;
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &bad, nullptr), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "iad_move_options.reserved[1]"));
    // a caller built with a later header: its extra bytes zero (the defaults) are fine, a set one is refused
    struct Later {
        iad_move_options base;
        uint64_t added;
    } later;
    std::memset(&later, 0, sizeof later);
    later.base.struct_size = static_cast<uint32_t>(sizeof later);
    REQUIRE(iad_move_options_init(&later.base) == IAD_OK);
    CHECK_EQ(later.base.struct_size, static_cast<uint32_t>(sizeof(iad_move_options)));  // a size it does not know
    later.base.struct_size = static_cast<uint32_t>(sizeof later);
    later.base.speed = 100.0;
    CHECK_EQ(iad_axis_move_to(ax, 30.0, &later.base, nullptr), IAD_OK);
    later.added = 1;
    CHECK_EQ(iad_axis_move_to(ax, 40.0, &later.base, nullptr), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "a field of a later ABI"));
    // the same rule on every input: options of the axis, an NV opt-in, a policy
    iad_axis_options ao;
    REQUIRE(iad_axis_get_options(ax, &ao) == IAD_OK);
    CHECK_EQ(ao.struct_size, static_cast<uint32_t>(sizeof ao));
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_OK);
    ao.struct_size = 0;
    CHECK_EQ(iad_axis_set_options(ax, &ao), IAD_ERR_INVALID_ARGUMENT);
    iad_nv_write_options nv;
    REQUIRE(iad_nv_write_options_init(&nv) == IAD_OK);
    nv.reserved[3] = 1;
    CHECK_EQ(iad_axis_teach_position(ax, 0, IAD_TEACH_TABLE, &nv, nullptr), IAD_ERR_UNSUPPORTED);
    iad_recovery_policy policy;
    REQUIRE(iad_recovery_policy_init(&policy) == IAD_OK);
    CHECK_EQ(policy.struct_size, static_cast<uint32_t>(sizeof policy));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_init_and_sized_outputs_write_the_size_they_know) {
    // a zeroed struct (struct_size 0: the caller did not set it) and a size the library does not know: the first size,
    // struct_size set to it; the caller that set struct_size = sizeof before _init gets the same
    iad_axis_options ao;
    std::memset(&ao, 0, sizeof ao);
    REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
    CHECK_EQ(ao.struct_size, static_cast<uint32_t>(sizeof ao));
    std::memset(&ao, 0xAB, sizeof ao);
    ao.struct_size = static_cast<uint32_t>(sizeof ao);
    REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
    CHECK_EQ(ao.struct_size, static_cast<uint32_t>(sizeof ao));
    std::memset(&ao, 0xAB, sizeof ao);  // struct_size 0xABABABAB: not a size it knows
    REQUIRE(iad_axis_options_init(&ao) == IAD_OK);
    CHECK_EQ(ao.struct_size, static_cast<uint32_t>(sizeof ao));
    CHECK_EQ(ao.reserved_header, 0u);
    CHECK_EQ(ao.reserved0, 0);
    CHECK(std::isnan(ao.max_speed_mm_s));
    // a bigger caller struct with a size the library does not know: nothing past the first size is written
    struct Bigger {
        iad_position_entry entry;
        uint64_t tail;
    } b;
    std::memset(&b, 0xCD, sizeof b);
    b.entry.struct_size = static_cast<uint32_t>(sizeof b);
    REQUIRE(iad_position_entry_init(&b.entry) == IAD_OK);
    CHECK_EQ(b.entry.struct_size, static_cast<uint32_t>(sizeof(iad_position_entry)));
    CHECK_EQ(b.tail, UINT64_C(0xCDCDCDCDCDCDCDCD));
    CHECK_NEAR(b.entry.speed_mm_s, 50.0, 1e-9);
    // the outputs into a sized struct follow the same rule
    iad_axis* ax = open_slider();
    std::memset(&b, 0xCD, sizeof b);
    REQUIRE(iad_axis_read_position_table(ax, 2, &b.entry) == IAD_OK);
    CHECK_EQ(b.entry.struct_size, static_cast<uint32_t>(sizeof(iad_position_entry)));
    CHECK_EQ(b.tail, UINT64_C(0xCDCDCDCDCDCDCDCD));
    CHECK_NEAR(b.entry.position_mm, 75.0, 0.001);
    // read back and written again as it came (the entry is an input too)
    iad_nv_write_options nv;
    iad_nv_write_options_init(&nv);
    nv.allow_nonvolatile = IAD_TRUE;
    iad_position_write_result out;
    CHECK_EQ(iad_axis_write_position_table(ax, 2, &b.entry, &nv, &out), IAD_OK);
    CHECK_EQ(out.skipped, IAD_TRUE);  // unchanged: the non-volatile guard
    CHECK_EQ(out.row, 2);
    CHECK_NEAR(out.position_mm, 75.0, 0.001);
    CHECK_EQ(out.checksum_ok, IAD_TRUE);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_arrays_take_the_stride_from_the_first_element) {
    iad_axis* ax = open_slider();
    iad_sequence_options so;
    iad_sequence_options_init(&so);
    so.speed = 100.0;
    // a later caller's element (8 bytes more): the stride is its struct_size
    struct Point {
        iad_sequence_point p;
        uint64_t added;
    } pts[2];
    std::memset(pts, 0, sizeof pts);
    for (int i = 0; i < 2; ++i) {
        pts[i].p.struct_size = static_cast<uint32_t>(sizeof pts[i]);
        pts[i].p.speed = pts[i].p.accel_g = pts[i].p.band = kNaN;
        pts[i].p.position = 20.0 + 20.0 * i;
    }
    iad_status st[2];
    size_t count = 0;
    CHECK_EQ(iad_axis_move_sequence(ax, &pts[0].p, 2, &so, st, 2, &count), IAD_OK);
    CHECK_EQ(count, size_t(2));
    CHECK_NEAR(st[1].position_mm, 40.0, 0.11);
    pts[1].added = 1;  // a later field set on the second point
    CHECK_EQ(iad_axis_move_sequence(ax, &pts[0].p, 2, &so, st, 2, &count), IAD_ERR_UNSUPPORTED);
    CHECK(has(last_json(), "points[1]"));
    pts[1].added = 0;
    pts[1].p.struct_size = static_cast<uint32_t>(sizeof(iad_sequence_point));  // one size for the whole array
    CHECK_EQ(iad_axis_move_sequence(ax, &pts[0].p, 2, &so, st, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "points[1].struct_size"));
    iad_sequence_point zero[1];
    std::memset(zero, 0, sizeof zero);  // struct_size 0
    CHECK_EQ(iad_axis_move_sequence(ax, zero, 1, &so, st, 1, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "points[0].struct_size is 0"));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi08_axis_moves_have_their_own_fields_and_names_are_never_cut) {
    iad_cell* cell = open_cell();
    iad_axis_move moves[2];
    std::memset(moves, 0, sizeof moves);  // NaN or 0 in a double, 0 in an enum: the defaults
    for (int i = 0; i < 2; ++i) moves[i].struct_size = static_cast<uint32_t>(sizeof moves[i]);
    moves[0].position = 30.0;
    std::strcpy(moves[0].axis, "SIM2:1");
    moves[1].position = 40.0;
    moves[1].speed = 100.0;
    moves[1].timeout_s = kNaN;
    std::strcpy(moves[1].axis, "SIM1:1");
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, 1, kNaN, IAD_TRUE, results, 2, &count), IAD_OK);
    REQUIRE(count == size_t(2));
    // the requested order, each with its position in the request
    CHECK_EQ(std::string(results[0].axis), std::string("SIM2:1"));
    CHECK_EQ(results[0].index, 0);
    CHECK_EQ(results[1].index, 1);
    CHECK_NEAR(results[0].status.position_mm, 30.0, 0.11);
    CHECK_NEAR(results[1].status.position_mm, 40.0, 0.11);
    // all axes: the cell order
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(std::string(results[1].axis), std::string("SIM2:1"));
    CHECK_EQ(results[1].index, 1);
    // a name that does not fit its 128 bytes with the NUL: refused, never cut to a shorter name
    std::memset(moves[0].axis, 'x', sizeof moves[0].axis);
    CHECK_EQ(iad_cell_move_each(cell, moves, 2, 1, kNaN, IAD_TRUE, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "moves[0].axis does not fit"));
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi08_events_carry_the_axis_index_of_the_cell) {
    iad_cell* cell = open_cell();
    REQUIRE(iad_cell_start_monitors(cell, nullptr, nullptr) == IAD_OK);
    const double targets[2] = {35.0, 45.0};
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_move_all(cell, "SIM2:1,SIM1:1", targets, 2, nullptr, nullptr, nullptr, nullptr, nullptr, results, 2, &count), IAD_OK);
    bool done1 = false, done2 = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!(done1 && done2) && std::chrono::steady_clock::now() < deadline) {
        iad_event ev;
        REQUIRE(iad_cell_next_event(cell, 0.2, &ev) == IAD_OK);
        if (ev.kind == IAD_EVENT_NONE) {
            CHECK_EQ(ev.index, IAD_INT_UNKNOWN);
            continue;
        }
        const std::string axis = ev.axis;
        if (axis == "SIM1:1") CHECK_EQ(ev.index, 0);  // the cell order, not the order of the request
        if (axis == "SIM2:1") CHECK_EQ(ev.index, 1);
        done1 = done1 || (ev.kind == IAD_EVENT_DONE && axis == "SIM1:1");
        done2 = done2 || (ev.kind == IAD_EVENT_DONE && axis == "SIM2:1");
    }
    CHECK(done1 && done2);
    CHECK_EQ(iad_cell_stop_monitors(cell, nullptr), IAD_OK);
    // an axis's own queue: no cell position
    iad_axis* ax = nullptr;
    REQUIRE(iad_cell_open_axis(cell, "SIM2:1", &ax) == IAD_OK);
    REQUIRE(iad_axis_start_monitor(ax, nullptr) == IAD_OK);
    CHECK_EQ(iad_axis_move_to(ax, 25.0, nullptr, nullptr), IAD_OK);
    iad_event ev;
    bool seen = false;
    for (int i = 0; i < 50 && !seen; ++i) {
        REQUIRE(iad_axis_next_event(ax, 0.1, &ev) == IAD_OK);
        seen = ev.kind != IAD_EVENT_NONE;
        CHECK_EQ(ev.index, IAD_INT_UNKNOWN);
    }
    CHECK(seen);
    CHECK_EQ(iad_axis_stop_monitor(ax, 2.0), IAD_OK);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
}

TEST(capi08_a_cut_axis_name_keeps_its_index) {
    // two simulated axes in a cell under names of the caller's choice, one longer than IAD_AXIS_NAME_SIZE
    iad_axis* axes[2] = {open_slider(), open_slider()};
    const std::string long_name = std::string(150, 'p') + ":1";
    const std::string names = "A:1," + long_name;
    iad_cell_options co;
    iad_cell_options_init(&co);
    co.identify = IAD_FALSE;
    iad_cell* cell = nullptr;
    REQUIRE(iad_cell_open_axes(axes, 2, names.c_str(), &co, &cell) == IAD_OK);
    iad_cell_result results[2];
    size_t count = 0;
    CHECK_EQ(iad_cell_status_all(cell, nullptr, kNaN, results, 2, &count), IAD_OK);
    REQUIRE(count == size_t(2));
    CHECK_EQ(std::string(results[1].axis), long_name.substr(0, IAD_AXIS_NAME_SIZE - 1));  // cut, NUL-terminated
    CHECK_EQ(results[1].index, 1);
    char full[256];
    size_t need = 0;
    CHECK_EQ(iad_cell_axis_name(cell, static_cast<size_t>(results[1].index), full, sizeof full, &need), IAD_OK);
    CHECK_EQ(std::string(full), long_name);  // the index leads to the whole name
    // the requested list with the whole name: found, reported at its position in the request
    const std::string wanted = long_name + ",A:1";
    CHECK_EQ(iad_cell_status_all(cell, wanted.c_str(), kNaN, results, 2, &count), IAD_OK);
    CHECK_EQ(results[0].index, 0);
    CHECK_EQ(std::string(results[1].axis), std::string("A:1"));
    // as a move's input it does not fit: refused, never cut to another axis's name
    iad_axis_move m;
    std::memset(&m, 0, sizeof m);
    m.struct_size = static_cast<uint32_t>(sizeof m);
    m.position = 10.0;
    std::memcpy(m.axis, long_name.data(), sizeof m.axis);
    CHECK_EQ(iad_cell_move_each(cell, &m, 1, 1, kNaN, IAD_TRUE, results, 2, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(iad_cell_close(cell), IAD_OK);
    iad_axis_close(axes[0]);
    iad_axis_close(axes[1]);
}

TEST(capi08_a_garbage_struct_size_is_refused_before_it_is_used) {
    iad_axis* ax = open_slider();
    iad_move_options mo;
    mo.struct_size = static_cast<uint32_t>(sizeof mo);
    REQUIRE(iad_move_options_init(&mo) == IAD_OK);
    mo.speed = 100.0;
    // stack garbage in struct_size (never set): far beyond the struct, not a multiple of its alignment, just over the bound
    const uint32_t garbage[] = {0x7FFF0010u, 0xFFFFFFF8u, static_cast<uint32_t>(sizeof mo) + 4u,
                                static_cast<uint32_t>(sizeof mo) + 4096u + 8u, 0xABABABABu};
    for (const uint32_t g : garbage) {
        iad_move_options bad = mo;
        bad.struct_size = g;
        CHECK_EQ(iad_axis_move_to(ax, 30.0, &bad, nullptr), IAD_ERR_INVALID_ARGUMENT);
        CHECK(has(last_json(), "iad_move_options.struct_size is " + std::to_string(g)));
        CHECK(has(last_json(), "not the size of a struct this library can read"));
    }
    // the same for the input that is read as an array: the garbage must not become the stride (a pointer 2 GB away)
    iad_sequence_point pts[3];
    std::memset(pts, 0, sizeof pts);
    for (int i = 0; i < 3; ++i) {
        pts[i].struct_size = static_cast<uint32_t>(sizeof pts[i]);
        pts[i].speed = pts[i].accel_g = pts[i].band = kNaN;
        pts[i].position = 20.0 + 10.0 * i;
    }
    iad_sequence_options so;
    so.struct_size = static_cast<uint32_t>(sizeof so);
    REQUIRE(iad_sequence_options_init(&so) == IAD_OK);
    so.speed = 100.0;
    iad_status st[3];
    size_t count = 0;
    pts[0].struct_size = 0x7FFF0010u;
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, st, 3, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "points[0].struct_size is 2147418128"));
    pts[0].struct_size = static_cast<uint32_t>(sizeof pts[0]) + 4u;
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, st, 3, &count), IAD_ERR_INVALID_ARGUMENT);
    CHECK(has(last_json(), "not the size of a struct this library can read"));
    pts[0].struct_size = static_cast<uint32_t>(sizeof pts[0]);
    CHECK_EQ(iad_axis_move_sequence(ax, pts, 3, &so, st, 3, &count), IAD_OK);
    CHECK_EQ(count, size_t(3));
    // the largest struct_size it still reads: exactly 4096 bytes past its struct, all zero, is a later ABI's defaults
    std::vector<unsigned char> big(sizeof(iad_move_options) + 4096, 0);
    std::memcpy(big.data(), &mo, sizeof mo);
    const uint32_t most = static_cast<uint32_t>(big.size());
    std::memcpy(big.data(), &most, sizeof most);
    CHECK_EQ(iad_axis_move_to(ax, 40.0, reinterpret_cast<const iad_move_options*>(big.data()), nullptr), IAD_OK);
    big[big.size() - 1] = 1;  // a set byte at its end: a field of a later ABI, still told so
    CHECK_EQ(iad_axis_move_to(ax, 50.0, reinterpret_cast<const iad_move_options*>(big.data()), nullptr),
             IAD_ERR_UNSUPPORTED);
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}
