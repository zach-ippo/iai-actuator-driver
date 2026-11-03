// Model unit: the named parameters. ParamValue / ParameterKey parsing, the EC user parameters P1-P18 decoded from the
// class blocks of the units tested (rod EC-R6S, gripper EC-GRBP8M: the lists VERIFIED against the PC tool's screen in
// the Python edition), the visibility rules, every guard of plan_parameter_write (nothing is sent: the plan is what the
// Axis would carry out) and the CON parameters of a PCON-CB + RCP6 (item locations VERIFIED in the Python edition).
// This edition itself is UNVERIFIED on hardware.
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;

// The register words of a unit profile ("rod", "gripper", "pcon"): defined in test_model_identity.cpp.
std::map<int, std::uint16_t> model_fixture_words(const std::string& profile);

namespace {

class MapReader : public UnitReader {
public:
    explicit MapReader(std::map<int, std::uint16_t> words, int slave = 1) : words_(std::move(words)), slave_(slave) {}
    int slave() const override { return slave_; }
    std::vector<std::uint16_t> read_registers(int address, int count) override {
        std::vector<std::uint16_t> out;
        for (int i = 0; i < count; ++i) {
            const auto it = words_.find(address + i);
            out.push_back(it == words_.end() ? 0 : it->second);
        }
        return out;
    }
    std::vector<std::uint8_t> report_slave_id() override { return slave_id; }
    void set32(int address, std::uint32_t value) {
        words_[address] = static_cast<std::uint16_t>(value >> 16);
        words_[address + 1] = static_cast<std::uint16_t>(value & 0xFFFF);
    }
    std::vector<std::uint8_t> slave_id;

private:
    std::map<int, std::uint16_t> words_;
    int slave_;
};

MapReader make_unit(const std::string& profile) {
    MapReader reader(model_fixture_words(profile));
    if (profile == "rod") reader.slave_id = {0xEC, 0xFF, 0x00, 0x00, 0x02};
    if (profile == "gripper") reader.slave_id = {0xEC, 0xFF, 0x0A, 0x00, 0x02};
    if (profile == "pcon") reader.slave_id = {0xAE, 0xFF, 0x44, 0x00, 0x00};
    return reader;
}

/// What the Axis reads before it asks for a decision: the classes, the support words, ACCL and the identity.
ParameterContext make_context(MapReader& reader) {
    const UnitDirectory dir = UnitDirectory::read(reader);
    const DeviceInfo info = read_device_info(reader, &dir);
    ParameterContext ctx;
    ctx.system_code = info.system_code;
    ctx.unit_code = info.unit_code;
    ctx.model_name = info.model_name;
    ctx.family = info.product_family.value_or("IAI controller");
    ctx.is_ec = reg::is_ec_family(info.system_code);
    for (int class_id : parameter_classes_needed(ctx.is_ec)) ctx.classes[class_id] = dir.read_class(reader, class_id);
    ctx.support = dir.read_support_words(reader);
    ctx.accl = dir.read_accl(reader);
    return ctx;
}

const ClassItem& item_of(const ParameterContext& ctx, int class_id, int item_id, int entry = 0) {
    return *ctx.classes.at(class_id).item(item_id, entry);
}

const ParameterInfo* find_row(const std::vector<ParameterInfo>& rows, int number) {
    for (const ParameterInfo& r : rows)
        if (r.number == number) return &r;
    return nullptr;
}

std::vector<int> numbers_of(const std::vector<ParameterInfo>& rows) {
    std::vector<int> out;
    for (const ParameterInfo& r : rows) out.push_back(r.number);
    return out;
}

bool has_warning(const ParameterWritePlan& plan, const std::string& part) {
    for (const std::string& w : plan.warnings)
        if (w.find(part) != std::string::npos) return true;
    return false;
}

/// The former opt-ins: accepted and ignored (no longer needed); a plan with them equals one without.
ParameterWriteOptions opts(bool home = false, bool dangerous = false, bool unlisted = false) {
    ParameterWriteOptions o;
    o.allow_home_params = home;
    o.allow_dangerous = dangerous;
    o.allow_unlisted = unlisted;
    return o;
}

ParameterWritePlan plan(const ParameterContext& ctx, const std::string& key, const std::string& value,
                        const ParameterWriteOptions& options = ParameterWriteOptions()) {
    return plan_parameter_write(ctx, ParameterKey::parse(key), ParamValue::parse(value), options);
}

}  // namespace

// ------------------------------------------------------------------------------------------ values and keys

TEST(model_param_value_parse) {
    ParamValue v = ParamValue::parse("5");
    REQUIRE(v.number.has_value());
    CHECK_NEAR(*v.number, 5.0, 1e-12);
    CHECK_EQ(v.to_string(), "5");
    v = ParamValue::parse("  -2.5 ");
    CHECK_NEAR(*v.number, -2.5, 1e-12);
    CHECK_EQ(v.to_string(), "-2.5");
    v = ParamValue::parse("1e3");
    CHECK_NEAR(*v.number, 1000.0, 1e-9);
    v = ParamValue::parse("0x1F");
    REQUIRE(v.number.has_value());
    CHECK_NEAR(*v.number, 31.0, 1e-12);
    v = ParamValue::parse("1:Forward");
    CHECK(!v.number.has_value());
    CHECK_EQ(v.text, "1:Forward");
    CHECK_EQ(v.to_string(), "1:Forward");
    v = ParamValue::parse("disable");
    CHECK(!v.number.has_value());
    CHECK_EQ(v.text, "disable");
    v = ParamValue::parse("192.168.0.1");
    CHECK(!v.number.has_value());          // an address is a label
    CHECK(ParamValue::parse("").empty());
    CHECK(ParamValue::parse("   ").empty());
    CHECK(!ParamValue::parse("5").empty());
    CHECK(!ParamValue::parse("nan").number.has_value());
    CHECK(!ParamValue::parse("inf").number.has_value());
    const ParamValue of = ParamValue::of(2.5);
    CHECK_NEAR(*of.number, 2.5, 1e-12);
    CHECK_EQ(of.to_string(), "2.5");
    CHECK_EQ(ParamValue::of(300.0).to_string(), "300");
}

TEST(model_param_key_named_and_raw_items) {
    for (const char* text : {"5", "P5", "p5", "No.5", "no.5", " P5 "}) {
        const ParameterKey k = ParameterKey::parse(text);
        REQUIRE(k.number.has_value());
        CHECK_EQ(*k.number, 5);
        CHECK(!k.is_item());
    }
    CHECK_EQ(*ParameterKey::parse("No.143").number, 143);
    const ParameterKey item = ParameterKey::parse("8.15");
    CHECK(item.is_item());
    CHECK_EQ(item.class_id, 8);
    CHECK_EQ(item.item_id, 15);
    CHECK_EQ(item.entry, 0);
    CHECK_EQ(item.text, "8.15");
    const ParameterKey zone = ParameterKey::parse("C.1[1]");   // the class is hexadecimal
    CHECK_EQ(zone.class_id, 12);
    CHECK_EQ(zone.item_id, 1);
    CHECK_EQ(zone.entry, 1);
    CHECK_EQ(ParameterKey::parse("0x0C.2").class_id, 12);
    CHECK_EQ(ParameterKey::parse("7.58").item_id, 58);
    const ParameterKey named = ParameterKey::of(3);
    CHECK_EQ(*named.number, 3);
    CHECK_EQ(named.text, "3");
    for (const char* bad : {"", "abc", "8.", ".5", "8.x", "8.15[x]", "Z.1", "8..1", "P", "No."})
        CHECK_THROWS(ParameterKey::parse(bad), std::invalid_argument);
}

TEST(model_param_classes_needed) {
    for (bool ec : {true, false}) {
        const std::vector<int> classes = parameter_classes_needed(ec);
        const std::set<int> set(classes.begin(), classes.end());
        for (int c : {3, 7, 8, 9, 0x0C}) CHECK(set.count(c) == 1u);
    }
}

// ------------------------------------------------------------------------------------------ the EC table

TEST(model_param_ec_table_and_flags) {
    const std::vector<ParamDef>& table = ec_user_parameters();
    REQUIRE(table.size() == 18u);
    for (std::size_t i = 0; i < table.size(); ++i) CHECK_EQ(table[i].no, static_cast<int>(i) + 1);
    CHECK_EQ(ec_user_parameter(1)->key(), "7.1");
    CHECK_EQ(ec_user_parameter(1)->offset_mm, -0.30);
    CHECK_EQ(ec_user_parameter(3)->key(), "9.16");
    CHECK_EQ(ec_user_parameter(3)->kind, "parity");
    CHECK_EQ(ec_user_parameter(5)->key(), "8.14");
    CHECK_EQ(*ec_user_parameter(5)->bit, 6);
    CHECK_EQ(*ec_user_parameter(6)->bit, 10);
    CHECK_EQ(*ec_user_parameter(7)->bit, 23);
    CHECK_EQ(ec_user_parameter(7)->choices.at(1), "1:Disable");   // inverted sense
    CHECK_EQ(ec_user_parameter(17)->key(), "C.1[1]");
    CHECK_EQ(ec_user_parameter(18)->key(), "C.2[1]");
    CHECK_EQ(ec_user_parameter(11)->key(), "8.25+8.29");
    CHECK_EQ(*ec_user_parameter(11)->item_id2, 29);
    for (int no : {8, 11, 13, 14, 15, 16, 17, 18}) CHECK(ec_user_parameter(no)->read_only);
    for (int no : {1, 2, 3, 4, 5, 6, 7, 9, 10, 12}) CHECK(!ec_user_parameter(no)->read_only);
    CHECK(ec_user_parameter(0) == nullptr);
    CHECK(ec_user_parameter(19) == nullptr);
    CHECK(is_ec_home_parameter(3));
    CHECK(is_ec_home_parameter(4));
    CHECK(!is_ec_home_parameter(2));
    CHECK(!is_ec_home_parameter(1));
    CHECK(!ec_user_parameter(8)->note.empty());
    CHECK(ec_user_parameter(1)->evidence.find("VERIFIED") != std::string::npos);
}

// ------------------------------------------------------------------------------------------ decode and visibility

TEST(model_param_rod_list_matches_the_pc_tool_screen) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    // the PC tool lists P1-P6, P8, P9, P12 on the EC-R6S rod (VERIFIED in the Python edition)
    CHECK(numbers_of(rows) == (std::vector<int>{1, 2, 3, 4, 5, 6, 8, 9, 12}));

    const ParameterInfo& p1 = *find_row(rows, 1);
    CHECK_EQ(p1.name, "Operation range adjustment");
    CHECK_EQ(p1.item, "7.1");
    CHECK_EQ(*p1.address, "0x0700");
    CHECK_NEAR(*p1.value.number, 150.0, 1e-9);
    CHECK_EQ(*p1.text, "150.00 mm");
    CHECK_EQ(p1.unit, "mm");
    CHECK_NEAR(*p1.min, 1.0, 1e-9);
    CHECK_NEAR(*p1.max, 150.0, 1e-9);   // P1 can only narrow: up to the value read
    CHECK(p1.writable);
    CHECK(!p1.dangerous);
    CHECK_EQ(p1.family, "EC");
    CHECK_EQ(*p1.raw, "00003AB6");
    CHECK(p1.note.empty());             // no per-model note

    const ParameterInfo& p2 = *find_row(rows, 2);
    CHECK_EQ(*p2.address, "0x0810");
    CHECK_NEAR(*p2.value.number, 0.10, 1e-9);
    CHECK_NEAR(*p2.min, 0.01, 1e-9);
    CHECK_NEAR(*p2.max, 1.00, 1e-9);

    const ParameterInfo& p3 = *find_row(rows, 3);
    CHECK_EQ(*p3.address, "0x091E");
    CHECK_NEAR(*p3.value.number, 1.0, 1e-9);
    CHECK_EQ(*p3.text, "1:Forward");
    CHECK(p3.dangerous);                // a home parameter
    CHECK(p3.writable);
    CHECK_EQ(p3.choices.at("1"), "1:Forward");
    CHECK_EQ(p3.choices.at("0"), "0:Reverse");
    CHECK_NEAR(*p3.min, 0.0, 1e-9);
    CHECK_NEAR(*p3.max, 1.0, 1e-9);

    const ParameterInfo& p4 = *find_row(rows, 4);
    CHECK_EQ(*p4.address, "0x0928");
    CHECK_NEAR(*p4.value.number, 3.0, 1e-9);
    CHECK(p4.dangerous);

    // P5 and P6 share item 8.14
    CHECK_EQ(*find_row(rows, 5)->address, "0x081A");
    CHECK_EQ(*find_row(rows, 6)->address, "0x081A");
    CHECK_EQ(*find_row(rows, 5)->text, "0:Disable");

    const ParameterInfo& p8 = *find_row(rows, 8);
    CHECK_EQ(*p8.address, "0x070C");
    CHECK_EQ(*p8.text, "1:Disable");
    CHECK(!p8.writable);                // read-only until hardware-verified
    CHECK(p8.note.find("read-only until hardware-verified") == 0);

    CHECK_EQ(*find_row(rows, 9)->address, "0x0802");
    CHECK_EQ(*find_row(rows, 9)->text, "0:Double");
    CHECK_EQ(*find_row(rows, 12)->address, "0x0800");
    CHECK_EQ(*find_row(rows, 12)->text, "0:Continuous energization");
}

TEST(model_param_gripper_list_matches_the_pc_tool_screen) {
    MapReader reader = make_unit("gripper");
    const ParameterContext ctx = make_context(reader);
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    // P1-P7, P9, P10, P12 on the EC-GRBP8M gripper (VERIFIED in the Python edition)
    CHECK(numbers_of(rows) == (std::vector<int>{1, 2, 3, 4, 5, 6, 7, 9, 10, 12}));
    CHECK_NEAR(*find_row(rows, 1)->value.number, 9.5, 1e-9);
    CHECK_NEAR(*find_row(rows, 4)->value.number, 0.5, 1e-9);
    CHECK_EQ(*find_row(rows, 3)->address, "0x0918");
    CHECK_EQ(*find_row(rows, 6)->text, "1:Enable");
    CHECK_EQ(*find_row(rows, 7)->text, "0:Enable");        // P7 is inverted: 0 = Enable
    CHECK_EQ(*find_row(rows, 7)->address, "0x083A");
    CHECK_EQ(*find_row(rows, 10)->address, "0x083A");      // P7 and P10 share item 8.50
}

TEST(model_param_decode_all_eighteen_parameters_of_an_unknown_unit) {
    // an unknown unit code lists every mapped parameter that the unit has: P1-P18 decoded by class + item ID
    MapReader reader = make_unit("rod");
    ParameterContext ctx = make_context(reader);
    ctx.unit_code = 0x7E;
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    // P11 (the pair FI00 / FO00) needs both items and the rod's directory does not have FO00
    std::vector<int> expected;
    for (int no = 1; no <= 18; ++no)
        if (no != 11) expected.push_back(no);
    CHECK(numbers_of(rows) == expected);
    CHECK_EQ(*find_row(rows, 7)->text, "0:Enable");
    CHECK_EQ(*find_row(rows, 7)->address, "0x083A");
    CHECK_EQ(*find_row(rows, 8)->text, "1:Disable");
    CHECK_EQ(*find_row(rows, 8)->raw, "05350000");
    CHECK_EQ(*find_row(rows, 10)->text, "0:Disable");
    CHECK_EQ(*find_row(rows, 12)->raw, "20400000");
    CHECK_EQ(*find_row(rows, 13)->address, "0x091C");
    CHECK_EQ(*find_row(rows, 13)->raw, "0901008C");
    CHECK_EQ(*find_row(rows, 13)->text, "0:Disable");
    CHECK_EQ(*find_row(rows, 14)->text, "150.30 mm");
    CHECK_EQ(*find_row(rows, 15)->text, "-0.30 mm");
    CHECK_EQ(*find_row(rows, 16)->address, "0x083A");
    CHECK_EQ(*find_row(rows, 17)->address, "0x0C04");
    CHECK_EQ(find_row(rows, 17)->item, "C.1[1]");
    CHECK_EQ(*find_row(rows, 18)->text, "-0.30 mm");
    CHECK_NEAR(*find_row(rows, 14)->value.number, 150.3, 1e-9);
    CHECK_NEAR(*find_row(rows, 15)->value.number, -0.3, 1e-9);
    for (int no : {8, 13, 14, 15, 16, 17, 18}) {
        CHECK(!find_row(rows, no)->writable);                 // located but never written on hardware: read-only
        CHECK(find_row(rows, no)->note.find("read-only") == 0);
    }
    for (int no : {1, 2, 3, 4, 5, 6, 7, 9, 10, 12}) CHECK(find_row(rows, no)->writable);
}

TEST(model_param_p11_multi_io_pair_decodes_from_both_items) {
    // the gripper has FI00 (8.25) and FO00 (8.29): bytes 1 of 18191A10 / 292A2B10 = (0x19, 0x2A)... the pair of the
    // 3-position mode (26, 43) is "7:Move middle point" (VERIFIED in the Python edition)
    MapReader reader = make_unit("gripper");
    ParameterContext ctx = make_context(reader);
    ctx.unit_code = 0x7E;   // list every mapped parameter
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    const ParameterInfo* p11 = find_row(rows, 11);
    REQUIRE(p11 != nullptr);
    CHECK_EQ(p11->item, "8.25+8.29");
    CHECK_EQ(*p11->address, "0x0830/0x0832");
    CHECK_EQ(*p11->raw, "18191A10/292A2B10");
    CHECK_NEAR(*p11->value.number, 7.0, 1e-9);
    CHECK_EQ(*p11->text, "7:Move middle point");
    CHECK(!p11->writable);                       // located but never written on hardware
    // the other read-only parameters of the gripper
    CHECK_EQ(*find_row(rows, 14)->text, "10.30 mm");
    CHECK_EQ(*find_row(rows, 17)->text, "10.30 mm");
    CHECK_EQ(*find_row(rows, 15)->text, "-0.30 mm");
    CHECK_EQ(*find_row(rows, 13)->raw, "0D01208C");
    // an unknown pair is reported, not guessed
    const UnitDirectory dir = UnitDirectory::read(reader);
    const ClassBlock c8 = dir.read_class(reader, 8);
    reader.set32(c8.item(29)->address, 0x00FF0000u);
    ParameterContext odd = make_context(reader);
    odd.unit_code = 0x7E;
    CHECK(find_row(list_parameters(odd), 11)->text->find("unknown pair") == 0);
    CHECK(find_row(list_parameters(odd), 11)->value.empty());
    // without FO00 the pair cannot be decoded: P11 is not found (the rod's directory has neither item)
    MapReader rod = make_unit("rod");
    ParameterContext rod_ctx = make_context(rod);
    rod_ctx.unit_code = 0x7E;
    CHECK(find_row(list_parameters(rod_ctx), 11) == nullptr);
}

TEST(model_param_visibility_reasons_and_the_unit_code_list) {
    MapReader reader = make_unit("rod");
    ParameterContext ctx = make_context(reader);
    const ParameterModel model = model_parameter_list(ctx);
    CHECK(model.known);
    CHECK(model.verified);
    CHECK_EQ(*model.family, "EC standard");
    CHECK_EQ(model.candidates.size(), 17u);
    CHECK(model.listed == (std::vector<int>{1, 2, 3, 4, 5, 6, 8, 9, 12}));
    REQUIRE(model.hidden.size() == 8u);
    std::map<int, std::string> hidden(model.hidden.begin(), model.hidden.end());
    CHECK_EQ(hidden.at(7), "shown only with a wireless module (SUP2 CNWL)");
    CHECK(hidden.at(11).find("needs level > Position3") == 0);
    CHECK(hidden.at(13).find("needs level > Position3 and EXFN = 1 (level 0)") == 0);
    CHECK_EQ(hidden.at(16), "shown only with SUP3 SORS");
    CHECK(hidden.at(17).find("needs level > Phase1 and EXFN = 1") == 0);
    CHECK_EQ(model.note, "the parameters IA-OS lists for unit 0x00 (EC standard)");
    CHECK(model.seen_on.empty());  // kept for compatibility; no longer used

    // the same unit code but another model type: the same list, the same note, no model label
    ctx.model_name = std::string("EC-S6SAH-100-1-PN");
    const ParameterModel other = model_parameter_list(ctx);
    CHECK(other.known);
    CHECK(other.verified);
    CHECK(other.listed == model.listed);
    CHECK_EQ(other.note, model.note);
    ctx.model_name = std::nullopt;
    CHECK(model_parameter_list(ctx).verified);
    CHECK_EQ(model_parameter_list(ctx).note, model.note);
}

TEST(model_param_unknown_unit_lists_every_mapped_parameter) {
    MapReader reader = make_unit("rod");
    ParameterContext ctx = make_context(reader);
    ctx.unit_code = 0x7E;   // a unit code the driver has never seen
    ParameterModel model = model_parameter_list(ctx);
    CHECK(!model.known);
    CHECK(!model.verified);
    CHECK_EQ(model.listed.size(), 18u);
    CHECK(model.hidden.empty());
    CHECK_EQ(model.note, "unit code 0x7E: every mapped item found on the unit is shown");
    CHECK(model.note.find("UNVERIFIED") == std::string::npos);
    ctx.unit_code = std::nullopt;
    model = model_parameter_list(ctx);
    CHECK(!model.known);
    CHECK(model.note.find("unit code not readable") == 0);
    // only what the unit has is shown for an unknown unit
    ctx.unit_code = 0x7E;
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    CHECK(!rows.empty());
    CHECK(rows.size() <= 18u);
    CHECK(find_row(rows, 1)->note.empty());  // no per-model note
    CHECK(find_row(rows, 2)->writable);
}

TEST(model_param_p3_is_not_settable_without_a_home_pattern) {
    MapReader reader = make_unit("rod");
    reader.set32(0x091E, 0);   // ORG = 0
    const ParameterContext ctx = make_context(reader);
    const std::vector<ParameterInfo> rows = list_parameters(ctx);   // keep the rows alive: p3 refers into them
    const ParameterInfo& p3 = *find_row(rows, 3);
    CHECK(p3.value.empty());
    CHECK_EQ(*p3.text, "not settable (home pattern ORG = 0x00000000)");
    CHECK(!p3.writable);
    CHECK_EQ(p3.note, "not settable: home pattern ORG (item 9.16) is 0 / FFFFFFFF");
    CHECK_THROWS(plan(ctx, "P3", "1:Forward", opts(true)), SoftLimitError);
    MapReader unset = make_unit("rod");
    unset.set32(0x091E, 0xFFFFFFFFu);
    CHECK_THROWS(plan(make_context(unset), "P3", "0", opts(true)), SoftLimitError);
}

// ------------------------------------------------------------------------------------------ plans of the EC parameters

TEST(model_param_plan_p2_with_every_field) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    const ParameterWritePlan p = plan(ctx, "P2", "0.20");
    CHECK_EQ(p.number, 2);
    CHECK_EQ(p.name, "Auto-switch \"LS\" signal detection range adjustment");
    CHECK_EQ(p.key, "8.9");
    CHECK_EQ(p.unit, "mm");
    CHECK_EQ(p.class_id, 8);
    CHECK_EQ(p.item_id, 9);
    CHECK_EQ(p.entry, 0);
    CHECK_EQ(p.address, 0x0810);
    CHECK_EQ(p.size, 2);
    CHECK_EQ(p.raw_before, 10u);
    CHECK_EQ(p.raw_after, 20u);
    CHECK(p.changed());
    CHECK_EQ(p.before, "0.10 mm");
    CHECK_EQ(p.after, "0.20 mm");
    CHECK_NEAR(*p.value.number, 0.20, 1e-9);
    CHECK(p.listed_for_model);
    CHECK(p.verified_for_model);
    CHECK(!p.home_parameter);
    CHECK(!p.dangerous);
    CHECK(!p.rereads_soft_limits);
    CHECK((p.words() == std::vector<std::uint16_t>{0x0000, 0x0014}));
    CHECK(p.warnings.empty());          // nothing to warn about: no per-model warning
}

TEST(model_param_plan_unchanged_value_is_not_changed) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    const ParameterWritePlan p = plan(ctx, "P2", "0.10");
    CHECK(!p.changed());                // the input of the non-volatile write guard
    CHECK_EQ(p.raw_before, p.raw_after);
    CHECK_EQ(p.before, p.after);
    const ParameterWritePlan q = plan(ctx, "No.2", "0.1");
    CHECK(!q.changed());
    // an unchanged choice
    CHECK(!plan(ctx, "P3", "1:Forward", opts(true)).changed());
    CHECK(!plan(ctx, "P5", "0").changed());
    CHECK(plan(ctx, "P5", "1").changed());
}

TEST(model_param_plan_shared_items_flip_only_their_own_bit) {
    MapReader reader = make_unit("rod");
    reader.set32(0x081A, 0x00000400u);   // item 8.14: P6 (bit 10) is on, P5 (bit 6) is off
    const ParameterContext ctx = make_context(reader);
    const ParameterWritePlan p5 = plan(ctx, "P5", "1:Enable");
    CHECK_EQ(p5.raw_before, 0x00000400u);
    CHECK_EQ(p5.raw_after, 0x00000440u);   // only bit 6 changed
    CHECK_EQ(p5.before, "0:Disable");
    CHECK_EQ(p5.after, "1:Enable");
    CHECK(has_warning(p5, "P5 enabled"));
    const ParameterWritePlan p6 = plan(ctx, "P6", "0");
    CHECK_EQ(p6.raw_after, 0x00000000u);   // bit 6 untouched, bit 10 cleared
    MapReader both = make_unit("rod");
    both.set32(0x081A, 0xFFFFFFFFu);
    const ParameterWritePlan off = plan(make_context(both), "P5", "0");
    CHECK_EQ(off.raw_after, 0xFFFFFFBFu);
}

TEST(model_param_plan_choice_labels_and_numbers) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    for (const char* forward : {"1", "1:Forward", "forward", "FORWARD", "1:forward", "fo"})
        CHECK_EQ(plan(ctx, "P3", forward, opts(true)).after, "1:Forward");
    const ParameterWritePlan reverse = plan(ctx, "P3", "reverse", opts(true));
    CHECK_EQ(reverse.after, "0:Reverse");
    CHECK_EQ(reverse.raw_after, reverse.raw_before + 1);   // ORG 7 -> 8: the parity is the direction
    CHECK_THROWS(plan(ctx, "P3", "sideways", opts(true)), SoftLimitError);
    CHECK_THROWS(plan(ctx, "P3", "2", opts(true)), SoftLimitError);
    CHECK_THROWS(plan(ctx, "P3", "1.5", opts(true)), SoftLimitError);
    CHECK_THROWS(plan(ctx, "P5", "maybe"), SoftLimitError);
    CHECK_EQ(plan(ctx, "P12", "instant").after, "1:Instantaneous energization");   // a unique prefix
}

TEST(model_param_plan_mm_ranges) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    CHECK_THROWS(plan(ctx, "P2", "2.0"), SoftLimitError);       // 0.01 .. 1.00
    CHECK_THROWS(plan(ctx, "P2", "0.0"), SoftLimitError);
    CHECK_THROWS(plan(ctx, "P2", "-0.5"), SoftLimitError);
    CHECK_THROWS(plan(ctx, "P2", "wide"), SoftLimitError);
    // the rod enforces MINI 3.67 (SUP2 CINP and ATYP RINP): P2 may not go below 0.08 mm there
    try {
        plan(ctx, "P2", "0.07");
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("below this EC's minimum positioning band MINI (3.67) 0.08 mm") != std::string::npos);
    }
    CHECK_NEAR(*plan(ctx, "P2", "0.08").value.number, 0.08, 1e-9);
    CHECK_NEAR(*plan(ctx, "P2", "1.00").value.number, 1.0, 1e-9);
    // the gripper does not (no MINI): the driver's sanity range starts at 0.01 mm
    MapReader gripper = make_unit("gripper");
    CHECK_NEAR(*plan(make_context(gripper), "P2", "0.01").value.number, 0.01, 1e-9);
    CHECK_NEAR(*plan(ctx, "P2", "0.123").value.number, 0.12, 1e-9);   // rounded to 0.01 mm
    CHECK_THROWS(plan(ctx, "P4", "6.0", opts(true)), SoftLimitError);  // 0 .. 5
    CHECK_THROWS(plan(ctx, "P4", "-1", opts(true)), SoftLimitError);
    const ParameterWritePlan p4 = plan(ctx, "P4", "3.5", opts(true));
    CHECK_EQ(p4.raw_after, 350u);
    CHECK_EQ(p4.after, "3.50 mm");
    CHECK(has_warning(p4, "P4: adjust P1"));
}

TEST(model_param_plan_p1_can_only_narrow_and_not_below_the_position) {
    MapReader reader = make_unit("rod");
    ParameterContext ctx = make_context(reader);
    const ParameterWritePlan narrow = plan(ctx, "P1", "100.0");
    CHECK_EQ(narrow.raw_before, 15030u);
    CHECK_EQ(narrow.raw_after, 10030u);       // raw = (P1 + 0.30) x 100: the upper soft limit
    CHECK_EQ(narrow.before, "150.00 mm");
    CHECK_EQ(narrow.after, "100.00 mm");
    CHECK(narrow.rereads_soft_limits);
    CHECK(has_warning(narrow, "P1 is the upper soft limit"));
    CHECK_THROWS(plan(ctx, "P1", "160.0"), SoftLimitError);   // widening is for the PC tool
    CHECK_THROWS(plan(ctx, "P1", "0.5"), SoftLimitError);     // below 1.00 mm
    CHECK_NEAR(*plan(ctx, "P1", "150.0").value.number, 150.0, 1e-9);
    // never below the current position
    ctx.position_mm = 120.0;
    CHECK_THROWS(plan(ctx, "P1", "100.0"), SoftLimitError);
    CHECK_NEAR(*plan(ctx, "P1", "125.0").value.number, 125.0, 1e-9);
    // a larger ceiling read earlier in the session lets it grow back up to it
    ctx.position_mm = std::nullopt;
    ctx.p1_ceiling_mm = 200.0;
    CHECK_NEAR(*plan(ctx, "P1", "180.0").value.number, 180.0, 1e-9);
    CHECK_THROWS(plan(ctx, "P1", "210.0"), SoftLimitError);
    try {
        plan(ctx, "P1", "210.0");
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("allowed 1.00..200.00 mm") != std::string::npos);
        CHECK(std::string(e.what()).find("nothing written") != std::string::npos);
    }
}

namespace {

/// The address of MXPT (item 3.16, the highest allowed P9 value) on a unit.
int mxpt_address(MapReader& reader) {
    const UnitDirectory dir = UnitDirectory::read(reader);
    return dir.read_class(reader, 3).item(reg::kControllerItemMxpt)->address;
}

}  // namespace

TEST(model_param_plan_p9_and_p12_value_rules) {
    // the rod's MXPT allows only 0: Single (3) is not a valid P9 value there
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    try {
        plan(ctx, "P9", "3:Single");
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("this unit accepts only 0 (extensibility level, MXPT; UNVERIFIED rule)") !=
              std::string::npos);
        CHECK(std::string(e.what()).find("nothing written") != std::string::npos);
    }
    CHECK(!plan(ctx, "P9", "0").changed());
    CHECK_THROWS(plan(ctx, "P9", "9"), SoftLimitError);               // not a choice at all
    CHECK(has_warning(plan(ctx, "P9", "0:Double"), "changes how the EC reads its PIO inputs"));
    CHECK(has_warning(plan(ctx, "P12", "1"), "changes how the EC reads its PIO inputs"));

    // with MXPT 5 the valid set of extensibility level 0 is {0, 3}
    MapReader wide = make_unit("rod");
    wide.set32(mxpt_address(wide), 5);
    const ParameterContext wide_ctx = make_context(wide);
    CHECK_EQ(plan(wide_ctx, "P9", "3:Single").raw_after, 3u);
    CHECK_THROWS(plan(wide_ctx, "P9", "4"), SoftLimitError);
    CHECK_THROWS(plan(wide_ctx, "P9", "5"), SoftLimitError);

    // P12 != 0 is refused while P9 is 3 or 4
    MapReader single = make_unit("rod");
    single.set32(mxpt_address(single), 5);
    single.set32(0x0802, 3);
    const ParameterContext single_ctx = make_context(single);
    CHECK_THROWS(plan(single_ctx, "P12", "1"), SoftLimitError);
    CHECK(!plan(single_ctx, "P12", "0").changed());

    // 4 / 5 change the PIO layout: accepted only on a unit already in that mode (extensibility level 2: 0, 3, 4, 5)
    MapReader phase = make_unit("rod");
    phase.set32(mxpt_address(phase), 5);
    phase.set32(0x0802, 4);
    const UnitDirectory phase_dir = UnitDirectory::read(phase);
    const ClassBlock c8 = phase_dir.read_class(phase, 8);  // named: item() points into it
    const ClassItem* spio = c8.item(reg::kParamItemSpio);
    phase.set32(spio->address, spio->raw | (1u << 13));   // EXFN (8.50 b13): the extended functions are on
    ParameterContext phase_ctx = make_context(phase);
    phase_ctx.accl = 0x0200u;
    CHECK(!plan(phase_ctx, "P9", "4").changed());
    try {
        plan(phase_ctx, "P9", "5");
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("5 is accepted only on a unit already in that mode") != std::string::npos);
    }
    // a present value outside the valid set hides P9 from the list (it is still written, with a warning)
    MapReader hidden = make_unit("rod");
    hidden.set32(0x0802, 4);
    const ParameterContext hidden_ctx = make_context(hidden);
    CHECK(model_parameter_list(hidden_ctx).listed == (std::vector<int>{1, 2, 3, 4, 5, 6, 8, 12}));
    const ParameterWritePlan hidden_p9 = plan(hidden_ctx, "P9", "0");
    CHECK(!hidden_p9.listed_for_model);
    CHECK(has_warning(hidden_p9, "is not in the parameter list for unit 0x00 (hidden by the visibility rule"));
}

TEST(model_param_plan_p9_single_on_an_acr_unit_is_refused) {
    MapReader reader = make_unit("rod");
    reader.set32(mxpt_address(reader), 5);
    ParameterContext ctx = make_context(reader);
    ctx.model_name = std::string("EC-S6SAH-100-1-PN-ACR");   // option ACR: an RCON-EC connection
    try {
        plan(ctx, "P9", "3:Single");
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("P9 = Single on a unit with option ACR") == 0);
        CHECK(std::string(e.what()).find("nothing written") != std::string::npos);
    }
    // the ACR inside a clean-room type name is no option
    ctx.model_name = std::string("EC-S6SACR-100-1-PN");
    CHECK_NEAR(*plan(ctx, "P9", "3:Single").value.number, 3.0, 1e-9);
    // Double stays fine on an ACR unit
    ctx.model_name = std::string("EC-S6SAH-100-1-PN-ACR");
    CHECK(!plan(ctx, "P9", "0:Double").changed());
}

TEST(model_param_plan_unlisted_and_unknown_parameters) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    // P7 is not in the rod's list (no wireless module): written, with the reason as a warning (no opt-in needed)
    const ParameterWritePlan p7 = plan(ctx, "P7", "1:Disable");
    CHECK(!p7.listed_for_model);
    CHECK(has_warning(p7, "P7 (Wireless function setting) is not in the parameter list for unit 0x00"));
    CHECK(has_warning(p7, "hidden by the visibility rule: shown only with a wireless module (SUP2 CNWL)"));
    CHECK(has_warning(p7, "(listed: P1, P2, P3, P4, P5, P6, P8, P9, P12); written anyway"));
    CHECK_EQ(p7.raw_after & (1u << 23), 1u << 23);
    CHECK_EQ(plan(ctx, "P7", "1:Disable", opts(false, false, true)).raw_after, p7.raw_after);  // the old opt-in
    // not a parameter of the table at all
    try {
        plan(ctx, "P99", "1");
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("P99 is not a confirmed EC user parameter (mapped: P1, P2,") == 0);
    }
    CHECK_THROWS(plan(ctx, "P0", "1"), UnsafeCommandError);
}

TEST(model_param_plan_read_only_parameters_are_refused_even_with_every_opt_in) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    for (int no : {8, 11, 13, 14, 15, 16, 17, 18}) {
        const std::string key = "P" + std::to_string(no);
        CHECK_THROWS(plan(ctx, key, "1", opts(true, true, true)), UnsafeCommandError);
    }
    try {
        plan(ctx, "P8", "0", opts(true, true, true));
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("P8 (Power-saving setting) is read-only in this driver: read-only until") == 0);
        CHECK(std::string(e.what()).find("Nothing written") != std::string::npos);
    }
}

TEST(model_param_plan_home_parameters_are_written_with_the_home_warning) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    for (const char* key : {"P3", "P4"}) {  // no opt-in needed
        const ParameterWritePlan home = plan(ctx, key, key[1] == '3' ? "0" : "3.5");
        CHECK(home.home_parameter);
        CHECK(has_warning(home, "P3 / P4 are home parameters"));
        CHECK(has_warning(home, "alarm 0x0EE 'ABS error (2)'"));
    }
    const ParameterWritePlan p3 = plan(ctx, "P3", "0:Reverse", opts(true));
    CHECK(p3.home_parameter);
    CHECK(p3.dangerous);
    CHECK_EQ(p3.key, "9.16");
    CHECK_EQ(p3.address, 0x091E);
    CHECK_EQ(p3.raw_before, 7u);   // the EC-R6S rod holds ORG 7: odd = 1:Forward
    CHECK_EQ(p3.raw_after, 8u);
    CHECK_EQ(p3.before, "1:Forward");
    CHECK_EQ(p3.after, "0:Reverse");
    CHECK(has_warning(p3, "P3: the direction of every move reverses"));
    CHECK(has_warning(p3, "home pattern ORG 7 -> 8 (the parity is the direction)"));
    CHECK(has_warning(p3, "P3 / P4 are home parameters"));
    // allow_dangerous stands for allow_home_params on an EC
    CHECK_EQ(plan(ctx, "P3", "0", opts(false, true)).raw_after, 8u);
    // no home pattern change, no ORG note
    CHECK(!has_warning(plan(ctx, "P3", "1", opts(true)), "home pattern ORG"));
    // a TCP2 / ZRMK caution when the direction changes
    MapReader caution = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(caution);
    const ClassBlock c7 = dir.read_class(caution, 7);
    const ClassBlock c8 = dir.read_class(caution, 8);
    caution.set32(c7.item(62) ? c7.item(62)->address : 0x0752, 5);
    caution.set32(c8.item(8)->address, 0x100);
    const ParameterWritePlan cautioned = plan(make_context(caution), "P3", "0", opts(true));
    CHECK(has_warning(cautioned, "ZRMK (8.8) b8 CORG is set"));
    if (c7.item(62) != nullptr) CHECK(has_warning(cautioned, "TCP2 (7.62) != 0"));
}

TEST(model_param_plan_p3_on_a_rotary_axis_is_refused) {
    MapReader reader = make_unit("rod");
    const UnitDirectory dir = UnitDirectory::read(reader);
    reader.set32(dir.read_class(reader, 7).item(reg::kAxisItemAkid)->address, 1);   // AKID 1 = rotary
    const ParameterContext ctx = make_context(reader);
    try {
        plan(ctx, "P3", "0", opts(true));
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("P3 (home direction) on a rotary axis") == 0);
        CHECK(std::string(e.what()).find("nothing written") != std::string::npos);
    }
    // the mm parameters of a rotary axis are in degrees
    const ParameterWritePlan p2 = plan(ctx, "P2", "0.2");
    CHECK_EQ(p2.unit, "deg");
    CHECK(has_warning(p2, "P2 is in deg on this rotary axis"));
    ParameterContext deg = ctx;
    deg.units = "deg";
    CHECK_EQ(*find_row(list_parameters(deg), 2)->text, "0.10 deg");
    CHECK_EQ(find_row(list_parameters(deg), 2)->unit, "deg");
}

TEST(model_param_plan_any_model_is_written_without_a_model_warning) {
    MapReader reader = make_unit("rod");
    ParameterContext ctx = make_context(reader);
    ctx.model_name = std::string("EC-S6SAH-100-1-PN");   // another model type with unit code 0x00: the same list
    const ParameterWritePlan p = plan(ctx, "P2", "0.20");
    CHECK(p.verified_for_model);                          // kept for compatibility: unit 0x00 has an IA-OS list
    CHECK(p.warnings.empty());
    CHECK_EQ(p.raw_after, 20u);
    // a unit code without an IA-OS list here is written the same way, with no warning about the code
    ctx.unit_code = 0x7E;
    const ParameterWritePlan unknown = plan(ctx, "P2", "0.20");
    CHECK(!unknown.verified_for_model);
    CHECK(unknown.listed_for_model);
    CHECK(unknown.warnings.empty());
    CHECK_EQ(unknown.raw_after, 20u);
}

TEST(model_param_plan_p7_inverted_sense_on_the_gripper) {
    MapReader reader = make_unit("gripper");
    const ParameterContext ctx = make_context(reader);
    const ParameterWritePlan p = plan(ctx, "P7", "disable");
    CHECK_EQ(p.before, "0:Enable");
    CHECK_EQ(p.after, "1:Disable");
    CHECK_EQ(p.raw_after, p.raw_before | (1u << 23));
    CHECK_EQ(p.address, 0x083A);
    CHECK(p.verified_for_model);   // unit 0x0A has an IA-OS list
    const ParameterWritePlan p10 = plan(ctx, "P10", "1:Enable");
    CHECK_EQ(p10.raw_after, p10.raw_before | (1u << 18));
    CHECK(has_warning(p10, "P10: may have no effect"));
}

// ------------------------------------------------------------------------------------------ raw items

TEST(model_param_describe_named_and_raw_items) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    const ParameterInfo named = describe_parameter(ctx, ParameterKey::parse("P2"));
    CHECK_EQ(named.number, 2);
    CHECK_EQ(*named.text, "0.10 mm");
    CHECK_EQ(describe_parameter(ctx, ParameterKey::of(3)).number, 3);
    CHECK_THROWS(describe_parameter(ctx, ParameterKey::parse("P7")), UnsafeCommandError);    // not listed
    CHECK_THROWS(describe_parameter(ctx, ParameterKey::parse("P99")), UnsafeCommandError);

    const ParameterInfo raw = describe_parameter(ctx, ParameterKey::parse("8.15"));
    CHECK_EQ(raw.number, 0);
    CHECK_EQ(raw.item, "8.15");
    CHECK_EQ(raw.family, "raw");
    CHECK(raw.address.has_value());
    CHECK_EQ(raw.words.value(), 2);
    CHECK_EQ(*raw.raw, util::format("%08X", static_cast<unsigned>(item_of(ctx, 8, 15).raw)));
    CHECK_EQ(raw.unsigned_value.value(), item_of(ctx, 8, 15).raw);
    CHECK_EQ(*raw.named, 34);       // CON No.34 lives at this item
    CHECK(!raw.dangerous);
    CHECK(raw.note.find("PSHV") == 0);
    CHECK(raw.writable);
    // an item that is not in the ID map, a class that was not read, and a class that is not a parameter class
    CHECK_THROWS(describe_parameter(ctx, ParameterKey::parse("8.250")), UnsafeCommandError);
    CHECK_THROWS(describe_parameter(ctx, ParameterKey::parse("5.1")), UnsafeCommandError);    // class not read
    const ParameterInfo class3 = describe_parameter(ctx, ParameterKey::parse("3.2"));
    CHECK(!class3.writable);
    CHECK_EQ(class3.note, "MAXP: maximum position count (3 = 3-position)");
    CHECK_EQ(describe_parameter(ctx, ParameterKey::parse("C.1[1]")).item, "C.1[1]");
}

TEST(model_param_plan_raw_items) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    // 8.15 (PSHV) is a known whole-item parameter that is not dangerous: no opt-in
    const ParameterWritePlan p = plan(ctx, "8.15", "3000");
    CHECK_EQ(p.number, 34);
    CHECK_EQ(p.name, "8.15");
    CHECK_EQ(p.key, "8.15");
    CHECK_EQ(p.unit, "raw");
    CHECK_EQ(p.raw_after, 3000u);
    CHECK_EQ(p.after, "00000BB8");
    CHECK_EQ(p.before, util::format("%08X", static_cast<unsigned>(p.raw_before)));
    CHECK_EQ(p.size, 2);
    CHECK(!p.dangerous);
    CHECK(has_warning(p, "raw item write: its meaning is UNVERIFIED on this unit"));
    CHECK(p.listed_for_model);
    CHECK((p.words() == std::vector<std::uint16_t>{0x0000, 0x0BB8}));
    CHECK_EQ(plan(ctx, "8.15", "0x0BB8").raw_after, 3000u);
    // a shared item (8.1 holds several bit fields) is dangerous: written with a warning (no opt-in needed)
    const ParameterWritePlan shared = plan(ctx, "8.1", "0");
    CHECK(shared.dangerous);
    CHECK(has_warning(shared, "raw write of '8.1': No."));
    CHECK(has_warning(shared, "; written anyway"));
    CHECK_EQ(plan(ctx, "8.1", "0", opts(false, true)).raw_after, shared.raw_after);   // the old opt-in: ignored
    // class 3 is not a parameter class
    try {
        plan(ctx, "3.2", "5", opts(true, true, true));
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("class 0x0003 is not a parameter class (writable: 7, 8, 9, 0x0C)") == 0);
    }
    CHECK_THROWS(plan(ctx, "8.250", "5", opts(true, true, true)), UnsafeCommandError);   // not in the ID map
    CHECK_THROWS(plan(ctx, "8.15", "1.5"), SoftLimitError);                               // a raw item takes an integer
    CHECK_THROWS(plan(ctx, "8.15", "label"), SoftLimitError);
    CHECK_THROWS(plan(ctx, "8.15", "8589934592"), SoftLimitError);                        // 2^33 does not fit 32 bits
    CHECK_THROWS(plan(ctx, "8.15", "-2147483649"), SoftLimitError);
    CHECK_EQ(plan(ctx, "8.15", "-1").raw_after, 0xFFFFFFFFu);
    CHECK_EQ(plan(ctx, "8.15", "4294967295").raw_after, 0xFFFFFFFFu);
}

TEST(model_param_plan_raw_item_value_is_whole_in_the_json) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    // the write result of a raw item says 3000, as the read of the same item does (and the Python edition's int)
    const ParameterWritePlan p = plan(ctx, "8.15", "3000");
    CHECK(to_json(p.value).type() == Json::Type::Int);
    CHECK_EQ(to_json(p.value).dump(), "3000");
    CHECK(to_json(describe_parameter(ctx, ParameterKey::parse("8.15")).value).type() == Json::Type::Int);
    CHECK_EQ(to_json(plan(ctx, "8.15", "0xFFFFFFFF").value).dump(), "4294967295");
}

TEST(model_param_plan_raw_soft_limit_widening_is_written_with_a_warning) {
    MapReader reader = make_unit("rod");
    const ParameterContext ctx = make_context(reader);
    // 7.1 is 15030 (150.30 mm); a larger value widens the + soft limit: a stroke change (no opt-in needed)
    const ParameterWritePlan widened = plan(ctx, "7.1", "16000");
    CHECK(widened.dangerous);
    CHECK(has_warning(widened, "widens the + soft limit 7.1 (150.30 -> 160.00): a stroke change"));
    CHECK(!has_warning(widened, "written anyway"));  // the widening is the warning, once
    CHECK(widened.rereads_soft_limits);
    // narrowing it is a whole-item write of a known parameter (No.3)
    const ParameterWritePlan narrowed = plan(ctx, "7.1", "10000");
    CHECK_EQ(narrowed.raw_after, 10000u);
    CHECK(narrowed.rereads_soft_limits);
    CHECK_EQ(narrowed.number, 3);
    // the - soft limit 7.2 widens when it goes more negative
    CHECK(has_warning(plan(ctx, "7.2", "-500"), "widens the - soft limit 7.2"));
    CHECK_EQ(plan(ctx, "7.2", "-500", opts(false, true)).raw_after, 0xFFFFFE0Cu);
    CHECK_EQ(plan(ctx, "7.2", "-10").raw_after, 0xFFFFFFF6u);
}

// ------------------------------------------------------------------------------------------ the CON table

TEST(model_con_table_and_families) {
    const ConParam* vcmd = con_parameter(8);
    REQUIRE(vcmd != nullptr);
    CHECK_EQ(vcmd->symbol, "VCMD");
    CHECK_EQ(vcmd->name, "Default velocity");
    CHECK_EQ(vcmd->key(), "8.10");
    CHECK_EQ(vcmd->kind, "scaled");
    CHECK_NEAR(vcmd->scale, 0.01, 1e-12);
    CHECK_EQ(vcmd->unit, "mm/s");
    CHECK_NEAR(*vcmd->lo, 1.0, 1e-12);
    CHECK_NEAR(*vcmd->hi, 9999.0, 1e-12);
    CHECK(!vcmd->dangerous);
    CHECK(vcmd->writable);
    CHECK_EQ(con_parameter(23)->key(), "C.1[1]");
    CHECK(con_parameter(15)->dangerous);
    CHECK_EQ(con_parameter(15)->bits.size(), 1u);
    CHECK_EQ(con_parameter(15)->bits[0], 7);
    CHECK(!con_parameter(166)->writable);
    CHECK(!con_parameter(188)->writable);
    CHECK(con_parameter(9999) == nullptr);
    CHECK(con_parameter(11) == nullptr);   // not a parameter number
    CHECK(is_con_home_parameter(5));
    CHECK(is_con_home_parameter(22));
    CHECK(!is_con_home_parameter(15));

    const std::vector<ConParam> pcon = con_parameters("PCON");
    const std::vector<ConParam> unknown = con_parameters("");
    CHECK_EQ(pcon.size(), unknown.size());
    CHECK(pcon.size() > 100u);
    auto find = [](const std::vector<ConParam>& v, int no) -> const ConParam& {
        for (const ConParam& p : v)
            if (p.no == no) return p;
        throw std::logic_error("no such parameter");
    };
    // the range tables differ per family (DOC): ACON / DCON and SCON
    const std::vector<ConParam> acon = con_parameters("ACON");
    CHECK_NEAR(*find(pcon, 29).lo, 1.0, 1e-12);
    CHECK_NEAR(*find(acon, 29).lo, 50.0, 1e-12);
    CHECK_NEAR(*find(con_parameters("DCON"), 29).lo, 50.0, 1e-12);
    CHECK_NEAR(*find(con_parameters("ACON/DCON"), 87).hi, 11.0, 1e-12);
    CHECK_NEAR(*find(pcon, 84).hi, 4.0, 1e-12);
    CHECK_NEAR(*find(con_parameters("scon"), 84).hi, 8.0, 1e-12);
    CHECK_NEAR(*find(con_parameters("SCON"), 65).hi, 99999999.0, 1e-3);
    CHECK_NEAR(*find(unknown, 84).hi, 4.0, 1e-12);   // an unknown family keeps the PCON-CB range
}

// ------------------------------------------------------------------------------------------ the CON parameters of a PCON

TEST(model_con_pcon_parameter_list) {
    MapReader reader = make_unit("pcon");
    const ParameterContext ctx = make_context(reader);
    CHECK(!ctx.is_ec);
    CHECK_EQ(ctx.family, "PCON");
    const std::vector<ParameterInfo> rows = list_parameters(ctx);
    CHECK(rows.size() > 90u);
    const ParameterInfo& vcmd = *find_row(rows, 8);
    CHECK_EQ(vcmd.name, "Default velocity");
    CHECK_EQ(*vcmd.symbol, "VCMD");
    CHECK_EQ(vcmd.item, "8.10");
    CHECK_EQ(*vcmd.address, "0x0812");
    CHECK_NEAR(*vcmd.value.number, 840.0, 1e-9);
    CHECK_EQ(*vcmd.text, "840.00 mm/s");
    CHECK_EQ(vcmd.unit, "mm/s");
    CHECK_NEAR(*vcmd.min, 1.0, 1e-9);
    CHECK_NEAR(*vcmd.max, 9999.0, 1e-9);
    CHECK(vcmd.writable);
    CHECK(!vcmd.dangerous);
    CHECK_EQ(vcmd.family, "CON");
    CHECK_EQ(*vcmd.raw, "00014820");
    CHECK_EQ(*find_row(rows, 5)->text, "0:Reverse");
    CHECK(find_row(rows, 5)->dangerous);
    CHECK_EQ(*find_row(rows, 3)->text, "150.30 mm");
    CHECK_EQ(*find_row(rows, 4)->text, "-0.30 mm");
    CHECK_EQ(*find_row(rows, 16)->text, "38400 bps");
    CHECK_NEAR(*find_row(rows, 16)->min, 9600.0, 1e-9);
    CHECK_NEAR(*find_row(rows, 16)->max, 230400.0, 1e-9);
    CHECK_EQ(*find_row(rows, 42)->text, "1:Disable");
    CHECK_EQ(*find_row(rows, 87)->text, "7:EtherNet/IP");
    CHECK_EQ(*find_row(rows, 140)->text, "192.168.0.1");
    CHECK_EQ(*find_row(rows, 141)->text, "255.255.255.0");
    CHECK_EQ(*find_row(rows, 46)->text, "100 %");
    CHECK_EQ(*find_row(rows, 12)->text, "35 %");
    CHECK_EQ(*find_row(rows, 30)->text, "1:New method 1");
    CHECK(!find_row(rows, 166)->writable);
    CHECK(find_row(rows, 188) == nullptr);        // its item is not in the PCON's ID map
    CHECK(find_row(rows, 34)->note.find("keep at least 5 mm/s") == 0);
    CHECK_EQ(find_row(rows, 23)->item, "C.1[1]");
    // a rotary axis shows degrees
    ParameterContext deg = ctx;
    deg.units = "deg";
    CHECK_EQ(*find_row(list_parameters(deg), 8)->text, "840.00 deg/s");
    CHECK_EQ(find_row(list_parameters(deg), 8)->unit, "deg/s");
}

TEST(model_con_plan_default_velocity_with_unit_maxima) {
    MapReader reader = make_unit("pcon");
    const ParameterContext ctx = make_context(reader);
    const ParameterWritePlan p = plan(ctx, "No.8", "500");
    CHECK_EQ(p.number, 8);
    CHECK_EQ(p.name, "Default velocity");
    CHECK_EQ(p.key, "8.10");
    CHECK_EQ(p.unit, "mm/s");
    CHECK_EQ(p.address, 0x0812);
    CHECK_EQ(p.raw_before, 84000u);
    CHECK_EQ(p.raw_after, 50000u);
    CHECK_EQ(p.before, "840.00 mm/s");
    CHECK_EQ(p.after, "500.00 mm/s");
    CHECK_NEAR(*p.value.number, 500.0, 1e-9);
    CHECK(p.changed());
    CHECK(p.listed_for_model);
    CHECK(!p.verified_for_model);
    CHECK(!p.home_parameter);
    CHECK(!p.dangerous);
    // the reset warning is added by the Axis to a write that is not skipped (not by the plan)
    CHECK(!has_warning(p, "applies a committed parameter after a software reset or a power cycle"));
    CHECK(!has_warning(p, "no range table"));          // the PCON's family has one
    CHECK(!plan(ctx, "8", "840").changed());
    // above the actuator's maximum speed MAXV 7.4 (840 mm/s)
    try {
        plan(ctx, "No.8", "900");
        CHECK(false);
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("is above the actuator's maximum speed MAXV (7.4) 840 mm/s") != std::string::npos);
        CHECK(std::string(e.what()).find("nothing written") != std::string::npos);
    }
    CHECK_THROWS(plan(ctx, "No.8", "0"), SoftLimitError);       // below the table's range 1 .. 9999
    CHECK_THROWS(plan(ctx, "No.8", "fast"), SoftLimitError);
    CHECK_THROWS(plan(ctx, "No.9", "1.5"), SoftLimitError);     // above MAXA 7.5 (1.00 G)
    CHECK_THROWS(plan(ctx, "No.34", "25"), SoftLimitError);     // above MXPH 7.8 (20 mm/s)
    // No.34 below the documented 5 mm/s only warns
    const ParameterWritePlan push = plan(ctx, "No.34", "3");
    CHECK(has_warning(push, "No.34: 3 mm/s is below 5 mm/s"));
    CHECK(has_warning(push, "IAI asks to be consulted"));
}

TEST(model_con_plan_dangerous_parameters_are_written_with_a_warning) {
    MapReader reader = make_unit("pcon");
    const ParameterContext ctx = make_context(reader);
    // No.15 (STOP' input enable) is a bit field of item 8.1 and dangerous: written with a warning (no opt-in needed)
    const ParameterWritePlan p15 = plan(ctx, "No.15", "Disable");
    CHECK(p15.dangerous);
    CHECK(has_warning(p15, "No.15 (STOP' Input enable/disable selection) is a dangerous parameter"));
    CHECK_EQ(p15.raw_after, p15.raw_before | 0x80u);       // only bit 7 changed
    CHECK_EQ(p15.after, "1:Disable");
    CHECK_EQ(p15.key, "8.1");
    CHECK_EQ(plan(ctx, "No.15", "Disable", opts(true, true)).raw_after, p15.raw_after);  // the old opt-ins: ignored
    const ParameterWritePlan home = plan(ctx, "No.5", "Forward", opts(true));
    CHECK(home.home_parameter);
    CHECK(home.dangerous);
    CHECK_EQ(home.before, "0:Reverse");
    CHECK_EQ(home.after, "1:Forward");
    CHECK_EQ(home.raw_after, home.raw_before - 1);         // the pattern parity is the direction
    const ParameterWritePlan offset = plan(ctx, "No.22", "5.0", opts(true));
    CHECK(offset.home_parameter);
    CHECK_EQ(offset.raw_after, 500u);
    CHECK_EQ(offset.after, "5.00 mm");
    const ParameterWritePlan home_plain = plan(ctx, "No.5", "1");  // no opt-in needed
    CHECK(home_plain.home_parameter);
    CHECK(has_warning(home_plain, "No.5 (Home direction) is a dangerous parameter"));
    CHECK(plan(ctx, "No.5", "1", opts(false, true)).home_parameter);
    // a parameter that is shared by bit fields keeps the other fields
    const ParameterWritePlan sibling = plan(ctx, "No.27", "Edge");
    CHECK_EQ(sibling.raw_after, sibling.raw_before | (1u << 16));
}

TEST(model_con_plan_baud_and_read_only_and_missing) {
    MapReader reader = make_unit("pcon");
    const ParameterContext ctx = make_context(reader);
    CHECK(has_warning(plan(ctx, "No.16", "115200"), "is a dangerous parameter"));  // dangerous: a warning
    const ParameterWritePlan baud = plan(ctx, "No.16", "115200", opts(false, true));
    CHECK_EQ(baud.raw_after, 20u);                                         // IAI baud code
    CHECK_EQ(baud.before, "38400 bps");
    CHECK_EQ(baud.after, "115200 bps");
    CHECK_THROWS(plan(ctx, "No.16", "57600", opts(false, true)), SoftLimitError);   // not one of the eight rates
    CHECK_THROWS(plan(ctx, "No.16", "12345", opts(false, true)), SoftLimitError);
    try {
        plan(ctx, "No.16", "57600", opts(false, true));
    } catch (const SoftLimitError& e) {
        CHECK(std::string(e.what()).find("is not one of the documented SIO rates 9600, 14400, 19200") != std::string::npos);
    }
    // read-only in this driver
    try {
        plan(ctx, "No.166", "1", opts(true, true, true));
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("No.166 (Current limit expansion Function) is read-only in this driver") == 0);
    }
    // not offered on this unit
    try {
        plan(ctx, "No.188", "5", opts(true, true, true));
        CHECK(false);
    } catch (const UnsafeCommandError& e) {
        CHECK(std::string(e.what()).find("parameter No.188 is not offered on this unit") == 0);
    }
    CHECK_THROWS(plan(ctx, "No.9999", "5"), UnsafeCommandError);
    CHECK_THROWS(plan(ctx, "No.11", "5"), UnsafeCommandError);
    // an address: the dotted text is kept
    const ParameterWritePlan ip = plan(ctx, "No.140", "10.0.0.7", opts(false, true));
    CHECK_EQ(ip.raw_after, 0x0A000007u);
    CHECK_EQ(ip.after, "10.0.0.7");
    CHECK_THROWS(plan(ctx, "No.140", "300.1.1.1", opts(false, true)), SoftLimitError);
    CHECK_THROWS(plan(ctx, "No.140", "10.0.0", opts(false, true)), SoftLimitError);
}

TEST(model_con_plan_family_ranges_and_unknown_family) {
    MapReader reader = make_unit("pcon");
    ParameterContext ctx = make_context(reader);
    // No.29 (excitation detection time): 1..999 on a PCON, 50..999 on an ACON / DCON
    CHECK_EQ(plan(ctx, "No.29", "30", opts(false, true)).raw_after & 0xFFFF, 30u);
    ctx.family = "ACON/DCON";
    CHECK_THROWS(plan(ctx, "No.29", "30", opts(false, true)), SoftLimitError);
    CHECK_EQ(plan(ctx, "No.29", "60", opts(false, true)).raw_after & 0xFFFF, 60u);
    // No.84 (fieldbus mode): 0..4 on a PCON, 0..8 on an SCON
    ctx.family = "PCON";
    CHECK_THROWS(plan(ctx, "No.84", "6", opts(false, true)), SoftLimitError);
    ctx.family = "SCON";
    CHECK_EQ(plan(ctx, "No.84", "6", opts(false, true)).raw_after, 6u);
    // a family without a range table: the PCON-CB range with a note saying so, never a refusal
    ctx.family = "IAI controller";
    const ParameterWritePlan unknown = plan(ctx, "No.8", "500");
    CHECK(has_warning(unknown, "the unit's family has no range table of its own in the driver"));
    CHECK(!has_warning(unknown, "UNVERIFIED"));
    CHECK_EQ(unknown.raw_after, 50000u);
}

TEST(model_con_plan_minimum_positioning_band) {
    // No.10: a band below the unit's minimum makes the next software reset raise cold-start alarm 0x0A1
    MapReader low_res = make_unit("pcon");
    const UnitDirectory dir = UnitDirectory::read(low_res);
    const ClassBlock c9 = dir.read_class(low_res, 9);
    const int epls = c9.item(12)->address;           // EPLS 9.12: encoder pulses per revolution
    low_res.set32(epls, 800);                         // a low-resolution encoder: lead 16 / 800 x 3 = 0.06 mm
    const ParameterContext ctx = make_context(low_res);
    try {
        plan(ctx, "No.10", "0.05");
        CHECK(false);
    } catch (const SoftLimitError& e) {
        const std::string text = e.what();
        CHECK(text.find("No.10: 0.05 mm is below this unit's minimum positioning band 0.06 mm") == 0);
        CHECK(text.find("cold-start alarm 0x0A1") != std::string::npos);
        CHECK(text.find("nothing written") != std::string::npos);
    }
    CHECK_EQ(plan(ctx, "No.10", "0.06").raw_after, 6u);
    CHECK_EQ(plan(ctx, "No.10", "1.0").raw_after, 100u);
    // the 8192-pulse RCP6 encoder: 0.01 mm is accepted
    MapReader high_res = make_unit("pcon");
    high_res.set32(epls, 8192);
    CHECK_EQ(plan(make_context(high_res), "No.10", "0.01").raw_after, 1u);
    // an unknown encoder count (800 assumed) is only a warning
    MapReader unknown_res = make_unit("pcon");
    unknown_res.set32(epls, 0);
    const ParameterWritePlan warned = plan(make_context(unknown_res), "No.10", "0.05");
    CHECK(has_warning(warned, "encoder count not known: 800 assumed"));
    CHECK_EQ(warned.raw_after, 5u);
}

TEST(model_con_describe_named_parameter) {
    MapReader reader = make_unit("pcon");
    const ParameterContext ctx = make_context(reader);
    const ParameterInfo info = describe_parameter(ctx, ParameterKey::parse("No.34"));
    CHECK_EQ(info.number, 34);
    CHECK_EQ(*info.symbol, "PSHV");
    CHECK_EQ(*info.text, "20.00 mm/s");
    CHECK_THROWS(describe_parameter(ctx, ParameterKey::parse("No.188")), UnsafeCommandError);
    const ParameterInfo raw = describe_parameter(ctx, ParameterKey::parse("8.21"));
    CHECK_EQ(*raw.named, 35);
    CHECK(!raw.dangerous);
    CHECK_EQ(describe_parameter(ctx, ParameterKey::parse("9.6")).dangerous, true);   // servo gain selection
}
