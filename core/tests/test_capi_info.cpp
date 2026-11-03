// C ABI 0.6 information additions (include/iai_actuator_driver.h "axis: directory readers, self-test lines",
// src/c_api_info.cpp) on the simulator: the directory readers by name (read_directory, read_block, read_class,
// read_support_words, read_accl and the backup with the Python edition's lenient rows), the lines of a self-test, and
// the input ranges per product line of iad_call_json. Simulator only: no COM port is opened.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

namespace {

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string last_json() {
    std::size_t need = 0;
    if (iad_last_error_json(nullptr, 0, &need) != IAD_ERR_BUFFER_TOO_SMALL) return "<size query failed>";
    std::string text(need, 'x');
    if (iad_last_error_json(&text[0], text.size(), &need) != IAD_OK) return "<read failed>";
    text.resize(std::strlen(text.c_str()));
    return text;
}

/// A JSON output read the way a binding does: size query, then the text; "<IAD_...>" when it failed.
template <class F>
std::string read_text(F call) {
    std::size_t need = 0;
    iad_result r = call(nullptr, 0, &need);
    for (int attempt = 0; r == IAD_ERR_BUFFER_TOO_SMALL && attempt < 5; ++attempt) {
        std::string text(need, 'x');
        r = call(&text[0], text.size(), &need);
        if (r == IAD_OK) {
            text.resize(std::strlen(text.c_str()));
            return text;
        }
    }
    return std::string("<") + iad_result_name(r) + ">";
}

/// A connected axis on a simulated unit (no gap between frames, 5 ms polls).
iad_axis* sim_axis(const char* profile, bool connect = true) {
    iad_sim_options sim;
    iad_sim_options_init(&sim);
    sim.time_scale = 50.0;
    iad_open_options open;
    iad_open_options_init(&open);
    open.min_gap_s = 0.0;
    iad_axis_options ao;
    iad_axis_options_init(&ao);
    ao.poll_interval_s = 0.005;
    iad_axis* ax = nullptr;
    CHECK_EQ(iad_axis_open_sim(profile, &sim, &open, &ao, &ax), IAD_OK);
    REQUIRE(ax != nullptr);
    if (connect) CHECK_EQ(iad_axis_connect(ax, 1, nullptr), IAD_OK);
    return ax;
}

std::shared_ptr<SimulatedUnit> sim_of(iad_axis* ax) {
    return std::dynamic_pointer_cast<SimulatedUnit>(capi::find_axis(ax)->axis->bus()->transport_ptr());
}

/// iad_axis_info_call_json as a binding calls it; "<IAD_...>" when it failed.
std::string info_text(iad_axis* ax, const char* function, const char* args = nullptr) {
    return read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_info_call_json(ax, function, args, b, n, r); });
}

Json info_json(iad_axis* ax, const char* function, const char* args = nullptr) {
    const std::string text = info_text(ax, function, args);
    if (!text.empty() && text[0] == '<') {
        std::printf("    %s %s: %s %s\n", function, args ? args : "", text.c_str(), last_json().c_str());
        return Json();
    }
    return Json::parse(text);
}

/// The result of an iad_call_json function.
std::string call(const char* function, const char* args) {
    return read_text([&](char* b, size_t n, size_t* r) { return iad_call_json(function, args, b, n, r); });
}

}  // namespace

TEST(capi_info_directory_readers) {
    iad_axis* ax = sim_axis("gripper");
    const Json dir = info_json(ax, "read_directory");
    REQUIRE(dir.type() == Json::Type::Object);
    const Json& header = *dir.find("header");
    CHECK_EQ(header.find("words_per_record")->as_int(), 8LL);
    const auto& records = dir.find("records")->as_array();
    CHECK_EQ(static_cast<long long>(records.size()), header.find("record_count")->as_int());
    REQUIRE(!records.empty());
    CHECK_EQ(records[0].size(), static_cast<std::size_t>(8));

    // read_block: inside the block, the default to the end of the certain extent; out of it nothing is sent
    const std::string invalid = "<IAD_ERR_INVALID_ARGUMENT>";
    CHECK_EQ(info_json(ax, "read_block", "{\"class_id\": 259}").size(), static_cast<std::size_t>(10));  // at 0x9900
    CHECK_EQ(info_text(ax, "read_block", "{\"class_id\": 512, \"offset\": 0, \"count\": 4}"),
             std::string("[0,9752,0,88]"));
    const std::size_t sent = sim_of(ax)->requests().size();
    for (const char* args : {"{\"class_id\": 259, \"offset\": 9, \"count\": 2}", "{\"class_id\": 259, \"offset\": -1}",
                             "{\"class_id\": 259, \"offset\": 0, \"count\": 0}"}) {
        CHECK_EQ(info_text(ax, "read_block", args), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
        CHECK(has(last_json(), "is outside it"));
        CHECK(has(last_json(), "std::out_of_range"));
    }
    CHECK_EQ(info_text(ax, "read_block", "{\"class_id\": 274}"), invalid);  // the gripper has no 0x0112
    CHECK(has(last_json(), "class 0x0112 is not in this"));
    CHECK_EQ(sim_of(ax)->requests().size(), sent);

    // read_class: items by ID; too large or not in the directory refused
    const Json c8 = info_json(ax, "read_class", "{\"class_id\": 8}");
    CHECK_EQ(c8.find("class_id")->as_int(), 8LL);
    CHECK(!c8.find("id_map_address")->is_null());
    const auto& items = c8.find("items")->as_array();
    REQUIRE(!items.empty());
    CHECK(!items[0].find("item_id")->is_null());
    const Json monitor = info_json(ax, "read_class", "{\"class_id\": 257}");  // 0x0101: no ID map, word by word
    CHECK(monitor.find("id_map_address")->is_null());
    CHECK(monitor.find("items")->as_array()[0].find("item_id")->is_null());
    CHECK_EQ(info_text(ax, "read_class", "{\"class_id\": 16}"), invalid);  // the position table: 12288 words
    CHECK(has(last_json(), "read it in parts"));
    CHECK_EQ(info_text(ax, "read_class", "{\"class_id\": 274}"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));

    // the class-1 words in member order, ACCL
    const std::string support = info_text(ax, "read_support_words");
    CHECK(support.rfind("{\"SUPT\":", 0) == 0);
    CHECK_EQ(info_json(ax, "read_accl").type(), Json::Type::Int);

    // records given: exactly those (a directory without class 1 has no support words)
    std::string only7 = "{\"records\": [";
    for (const Json& r : records)
        if (r.as_array()[0].as_int() == 7) only7 += r.dump();
    only7 += "]}";
    CHECK_EQ(info_text(ax, "read_support_words", only7.c_str()), std::string("null"));
    CHECK_EQ(info_text(ax, "read_accl", only7.c_str()), std::string("null"));
    const std::string class7 = only7.substr(0, only7.size() - 1) + ", \"class_id\": 7}";
    CHECK(has(info_text(ax, "read_class", class7.c_str()), "\"items\""));

    // the result is kept; bad arguments and an unknown function
    std::size_t need = 0;
    CHECK_EQ(iad_axis_info_call_json(ax, "read_accl", nullptr, nullptr, 0, &need), IAD_ERR_BUFFER_TOO_SMALL);
    CHECK_EQ(read_text([&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); }),
             info_text(ax, "read_accl"));
    CHECK_EQ(info_text(ax, "read_block", "{\"class_id\": \"8\"}"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(info_text(ax, "read_block", "[1]"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(info_text(ax, "read_block", "{\"class_id\": 8, \"records\": [[1, 2]]}"),
             std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK_EQ(info_text(ax, "write_block"), std::string("<IAD_ERR_UNSUPPORTED>"));
    CHECK_EQ(iad_axis_info_call_json(ax, nullptr, nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    iad_axis_close(ax);
}

TEST(capi_info_directory_without_identify) {
    iad_axis* ax = sim_axis("slider", false);
    const Json c7 = info_json(ax, "read_class", "{\"class_id\": 7}");  // reads the index table itself
    REQUIRE(c7.type() == Json::Type::Object);
    CHECK_EQ(c7.find("items")->as_array()[0].find("raw")->as_int(), 15030LL);
    iad_axis_close(ax);
}

TEST(capi_info_backup_rows_are_lenient) {
    iad_axis* ax = sim_axis("pcon");
    const Json listed = info_json(ax, "backup", "{\"rows\": [5, 1, 2, 2]}");
    REQUIRE(listed.type() == Json::Type::Object);
    CHECK_EQ(listed.find("meta")->find("rows")->dump(), std::string("[1,2,5]"));
    CHECK_EQ(listed.find("positions")->find("rows")->as_array().size(), static_cast<std::size_t>(3));
    CHECK_EQ(listed.find("errors")->dump(), std::string("[]"));
    CHECK(!listed.find("meta")->find("path"));
    for (const char* args : {"{\"rows\": [99999]}", "{\"rows\": [-1]}"}) {
        const Json bad = info_json(ax, "backup", args);
        REQUIRE(bad.type() == Json::Type::Object);
        CHECK(has(bad.find("errors")->dump(), "positions: ValueError: rows ["));
        CHECK(has(bad.find("errors")->dump(), "are outside 0.."));
        CHECK(!bad.find("parameters")->is_null());  // the other sections are still read
    }
    const Json text = info_json(ax, "backup", "{\"rows\": \"bad\"}");
    REQUIRE(text.type() == Json::Type::Object);
    CHECK_EQ(text.find("meta")->find("rows")->as_string(), std::string("bad"));
    CHECK(has(text.find("errors")->dump(),
              "positions: ValueError: rows must be one of ('used', 'all') or row numbers, got 'bad'"));
    const Json used = info_json(ax, "backup");
    CHECK_EQ(used.find("meta")->find("rows")->as_string(), std::string("used"));
    CHECK_EQ(used.find("positions")->find("rows")->as_array().size(), static_cast<std::size_t>(64));
    // iad_axis_backup_json keeps its strict rows (the CLI's --rows)
    std::size_t need = 0;
    CHECK_EQ(iad_axis_backup_json(ax, "bad", nullptr, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    CHECK_EQ(info_text(ax, "backup", "{\"rows\": [\"1\"]}"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    iad_axis_close(ax);
}

TEST(capi_info_self_test_items) {
    iad_axis* ax = sim_axis("slider");
    auto items_from = [&](int first) {
        return read_text(
            [&](char* b, size_t n, size_t* r) { return iad_axis_self_test_items_json(ax, first, b, n, r); });
    };
    CHECK_EQ(items_from(0), std::string("[]"));
    std::size_t need = 0;
    CHECK_EQ(iad_axis_self_test_items_json(ax, -1, nullptr, 0, &need), IAD_ERR_INVALID_ARGUMENT);
    // watched from another thread while the test runs: the lines so far, in order, the same as the report's
    std::atomic<bool> done{false};
    std::vector<std::string> seen;
    std::thread watcher([&] {
        while (!done.load()) {
            const std::string text = items_from(0);
            if (text[0] == '[' && text != "[]" && (seen.empty() || seen.back() != text)) seen.push_back(text);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    iad_check_options co;
    iad_check_options_init(&co);
    co.motion = IAD_TRUE;
    iad_check_summary cs;
    const iad_result r = iad_axis_self_test(ax, &co, "SIM", &cs);
    done.store(true);
    watcher.join();
    CHECK_EQ(r, IAD_OK);
    CHECK_EQ(items_from(0), std::string("[]"));  // none runs now: the lines are in the report
    const Json report = Json::parse(
        read_text([&](char* b, size_t n, size_t* r2) { return iad_axis_last_result_json(ax, b, n, r2); }));
    const auto& lines = report.find("items")->as_array();
    REQUIRE(static_cast<int>(lines.size()) == cs.items);
    CHECK(!seen.empty());
    for (const std::string& text : seen) {  // every view is a prefix of the report's lines
        const Json view = Json::parse(text);
        const auto& part = view.as_array();
        CHECK(part.size() <= lines.size());
        for (std::size_t i = 0; i < part.size() && i < lines.size(); ++i) CHECK_EQ(part[i].dump(), lines[i].dump());
    }
    const Json first = Json::parse(seen.front()).as_array()[0];
    CHECK_EQ(first.find("name")->as_string(), std::string("link"));
    CHECK_EQ(first.find("result")->as_string(), std::string("PASS"));
    CHECK(first.find("data")->type() == Json::Type::Object);
    iad_axis_close(ax);
}

TEST(capi_info_unit_info_sections_as_a_list) {
    iad_axis* ax = sim_axis("slider");
    const Json none = info_json(ax, "unit_info", "{\"sections\": []}");
    REQUIRE(none.type() == Json::Type::Object);
    CHECK_EQ(none.find("meta")->find("sections")->dump(), std::string("[]"));
    CHECK(none.find("identity")->is_null());
    const Json some = info_json(ax, "unit_info", "{\"sections\": [\"alarms\", \"identity\", \"alarms\"]}");
    CHECK_EQ(some.find("meta")->find("sections")->dump(), std::string("[\"alarms\",\"identity\"]"));
    CHECK(!some.find("identity")->is_null());
    CHECK(some.find("parameters")->is_null());
    CHECK_EQ(info_text(ax, "unit_info", "{\"sections\": [\"bogus\"]}"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK(has(last_json(), "unknown section(s) ['bogus']; choose from ['identity', "));
    const Json all = info_json(ax, "unit_info");
    CHECK_EQ(all.find("meta")->find("sections")->as_array().size(), static_cast<std::size_t>(6));
    iad_axis_close(ax);
}

TEST(capi_info_input_ranges_and_product_line) {
    CHECK_EQ(call("info.product_line", "{\"family\": \"EC\"}"), std::string("\"EC\""));
    CHECK_EQ(call("info.product_line", "{\"family\": \"ec\"}"), std::string("\"EC\""));
    CHECK_EQ(call("info.product_line", "{\"family\": \"PCON\"}"), std::string("\"CON-CB\""));
    CHECK_EQ(call("info.product_line", "{\"family\": \"IAI controller\"}"), std::string("\"CON-CB\""));
    CHECK_EQ(call("info.product_line", "{\"family\": \"PCON\", \"gateway\": \"rcon\"}"), std::string("\"RCON\""));
    CHECK_EQ(call("info.product_line", "{\"gateway\": \"RoboNet-gateway\"}"), std::string("\"ROBONET\""));
    CHECK_EQ(call("info.product_line", "{}"), std::string("\"CON-CB\""));

    const Json all = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_call_json("info.input_ranges", "{}", b, n, r); }));
    REQUIRE(all.type() == Json::Type::Object);
    std::vector<std::string> names;
    for (const auto& kv : all.as_object()) names.push_back(kv.first);
    CHECK(names == (std::vector<std::string>{"EC", "CON-CB", "RCON", "ROBONET"}));
    CHECK_EQ(all.find("EC")->find("parameters")->as_array().size(), static_cast<std::size_t>(19));
    CHECK(all.find("RCON")->find("parameters")->is_null());
    CHECK_EQ(all.find("ROBONET")->find("moves")->as_array().size(), static_cast<std::size_t>(8));
    for (const auto& kv : all.as_object())
        for (const Json& row : kv.second.find("other")->as_array()) CHECK(!row.find("doc")->as_string().empty());
    CHECK(has(call("info.ec_parameter_table", "{}"), "\"driver_range\":{\"min\":0.01,\"max\":1.0}"));  // floats

    const Json acon = Json::parse(read_text([&](char* b, size_t n, size_t* r) {
        return iad_call_json("info.input_ranges", "{\"line\": \"CON-CB\", \"family\": \"ACON\"}", b, n, r);
    }));
    CHECK_EQ(acon.find("family")->as_string(), std::string("ACON/DCON"));
    bool found13 = false;
    for (const Json& row : acon.find("parameters")->as_array())
        if (row.find("number")->as_int() == 13) found13 = row.find("max")->as_int() == 300;
    CHECK(found13);
    CHECK_EQ(call("info.input_ranges", "{\"line\": \"RCON-EC\"}"), std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    CHECK(has(last_json(), "unknown product line 'RCON-EC'; choose from ['EC', 'CON-CB', 'RCON', 'ROBONET']"));
    CHECK(has(call("info.ec_parameter_table", "{}"), "{\"no\":143,"));
}

TEST(capi_info_caller_records_past_the_register_space_are_refused) {
    iad_axis* ax = sim_axis("slider");
    // DQ 65535 x MS 1 x LQ 65535 passes 2^31: the size wrapped negative and read_class gave a bogus block with IAD_OK
    for (const char* function : {"read_class", "read_block"}) {
        CHECK_EQ(info_text(ax, function, "{\"class_id\":7,\"records\":[[7,0,65535,65535,4096,0,0,1]]}"),
                 std::string("<IAD_ERR_INVALID_ARGUMENT>"));
        CHECK(has(last_json(), "65535 registers"));
        // a wrap to a small positive size (65534) read outside every declared block
        CHECK_EQ(info_text(ax, function, "{\"class_id\":7,\"records\":[[7,0,65535,21846,4096,0,0,3]]}"),
                 std::string("<IAD_ERR_INVALID_ARGUMENT>"));
    }
    // the extent itself never overflows: held at the int range, so read_class refuses it as too large
    CHECK_EQ(reg::category_extent(7, 65535, 65535, 1).words, 2147483647);
    CHECK_EQ(reg::category_extent(7, 65535, 21846, 3).words, 2147483647);
    CHECK_EQ(reg::category_extent(0x63, 65535, 65535, 0).words_max, 2147483647);
    CHECK_EQ(reg::category_extent(7, 10, 2, 2).words, 40);
    // a record inside the register space still reads
    CHECK(has(info_text(ax, "read_block", "{\"class_id\":7,\"count\":2,\"records\":[[7,0,2,1,4096,0,0,2]]}"), "["));
    CHECK_EQ(iad_axis_close(ax), IAD_OK);
}

TEST(capi_parameter_rows_follow_the_python_editions_keys_and_number_types) {
    // (0.7) a named row has none of the raw-item fields and no empty symbol / note; a code, a count and a flag is an
    // integer, a measure a float even when whole (1.0), as the Python edition's parameters() writes them
    iad_axis* ax = sim_axis("gripper");
    const Json rows = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_parameters_json(ax, b, n, r); }));
    REQUIRE(rows.type() == Json::Type::Array);
    const Json* p5 = nullptr;
    const Json* p2 = nullptr;
    for (const Json& row : rows.as_array()) {
        for (const char* gone : {"unsigned", "signed", "words", "named", "symbol"}) CHECK(row.find(gone) == nullptr);
        const Json* note = row.find("note");
        CHECK(note == nullptr || !(note->type() == Json::Type::String && note->as_string().empty()));
        const long long number = row.find("number")->as_int();
        if (number == 5) p5 = &row;
        if (number == 2) p2 = &row;
    }
    REQUIRE(p5 != nullptr);
    REQUIRE(p2 != nullptr);
    for (const char* key : {"value", "min", "max"}) {
        CHECK(p5->find(key)->type() == Json::Type::Int);
        CHECK(p2->find(key)->type() == Json::Type::Number);
        CHECK(has(p2->find(key)->dump(), "."));
    }
    // a raw item keeps its fields; a named parameter read by number has the same row as the list
    const std::string raw = read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_read_parameter_json(ax, "8.15", b, n, r); });
    CHECK(has(raw, "\"unsigned\":") && has(raw, "\"words\":") && has(raw, "\"family\":\"raw\""));
    const Json one = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_read_parameter_json(ax, "P5", b, n, r); }));
    CHECK(one.find("unsigned") == nullptr);
    CHECK(one.find("value")->type() == Json::Type::Int);
    // not offered: the number is named as the Python edition names it (parameter 15), a text key quoted ('P15')
    auto read_one = [&](const char* key) {
        return read_text([&](char* b, size_t n, size_t* r) { return iad_axis_read_parameter_json(ax, key, b, n, r); });
    };
    CHECK_EQ(read_one("15"), std::string("<IAD_ERR_UNSAFE_REFUSED>"));
    CHECK(has(last_json(), "\"message\":\"parameter 15 is not offered on this unit\""));
    CHECK_EQ(read_one("P15"), std::string("<IAD_ERR_UNSAFE_REFUSED>"));
    CHECK(has(last_json(), "\"message\":\"parameter 'P15' is not offered on this unit\""));
    // the limits' integer ranges
    const Json limits = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_limits_json(ax, b, n, r); }));
    for (const char* name : {"stored_rows", "speed_override"})
        for (const char* key : {"min", "max"}) CHECK(limits.find(name)->find(key)->type() == Json::Type::Int);
    CHECK(limits.find("position")->find("max")->type() == Json::Type::Number);
    iad_axis_close(ax);
}

TEST(capi_con_parameter_rows_keep_the_tables_number_types) {
    iad_axis* ax = sim_axis("pcon");
    const Json rows = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_parameters_json(ax, b, n, r); }));
    const Json info = info_json(ax, "unit_info", "{\"sections\": [\"parameters\"]}");
    auto row_of = [](const Json& list, long long number) -> const Json& {
        for (const Json& row : list.as_array())
            if (row.find("number")->as_int() == number) return row;
        throw std::out_of_range("no parameter row " + std::to_string(number));
    };
    const Json& numbered = *info.find("parameters")->find("numbered");
    struct Expect {
        long long number;
        const char* key;
        Json::Type type;
    };
    for (const Expect& e : {Expect{8, "min", Json::Type::Int},      Expect{8, "max", Json::Type::Int},
                            Expect{8, "value", Json::Type::Number}, Expect{16, "value", Json::Type::Int},
                            Expect{16, "min", Json::Type::Int},     Expect{22, "min", Json::Type::Number},
                            Expect{22, "max", Json::Type::Number},  Expect{55, "min", Json::Type::Number},
                            Expect{55, "max", Json::Type::Number}}) {
        CHECK(row_of(rows, e.number).find(e.key)->type() == e.type);
        CHECK(row_of(numbered, e.number).find(e.key)->type() == e.type);
    }
    CHECK_EQ(row_of(rows, 8).find("symbol")->as_string(), std::string("VCMD"));
    for (const char* gone : {"unsigned", "signed", "words", "named"}) CHECK(row_of(numbered, 8).find(gone) == nullptr);
    for (const Expect& e : {Expect{48, "factory_default", Json::Type::Number}, Expect{65, "factory_default", Json::Type::Int},
                            Expect{55, "family_min", Json::Type::Number}, Expect{55, "family_max", Json::Type::Number},
                            Expect{8, "family_min", Json::Type::Int}, Expect{16, "family_max", Json::Type::Int}})
        CHECK(row_of(numbered, e.number).find(e.key)->type() == e.type);
    iad_axis_close(ax);
}

TEST(capi_parameter_write_notes_and_warnings_follow_the_python_edition) {
    iad_axis* ax = sim_axis("gripper");
    iad_param_write_options o;
    iad_param_write_options_init(&o);
    o.commit = IAD_FALSE;
    o.allow_unlisted = IAD_TRUE;
    iad_param_write_result out;
    CHECK_EQ(iad_axis_write_parameter(ax, "P12", "1", &o, &out), IAD_OK);
    const Json ram = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); }));
    CHECK(has(ram.find("note")->as_string(), "RAM ONLY - NOT SAVED (commit=False): a parameter written over Modbus (FC10) "
                                              "changes the EC's RAM only and reverts"));
    CHECK_EQ(ram.find("warnings")->as_array().front().as_string(),
             std::string("P12 is not in ME3816-6B (IA-OS lists it): its meaning is UNVERIFIED"));
    iad_param_write_options_init(&o);
    o.allow_unlisted = IAD_TRUE;
    CHECK_EQ(iad_axis_write_parameter(ax, "P12", "0", &o, &out), IAD_OK);
    const Json committed = Json::parse(read_text(
        [&](char* b, size_t n, size_t* r) { return iad_axis_last_result_json(ax, b, n, r); }));
    CHECK(has(committed.find("note")->as_string(), "Committed to non-volatile memory with coil 0x0428 (DRGE b7 PARW), the "
                                                    "coil IA-OS's Transfer writes (ON-WIRE: USB capture of IA-OS)."));
    iad_axis_close(ax);
}
