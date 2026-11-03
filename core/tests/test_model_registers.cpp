// Model unit: the register lookups (families, kinds, categories, text decoding, EC series, documented minimum speeds),
// the inline word helpers and the position-table row. Ported from the Python edition's registers / position tests; the
// values are VERIFIED in the Python edition, this edition is UNVERIFIED on hardware.
#include <cmath>
#include <limits>
#include <stdexcept>

#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "harness.hpp"

using namespace iai_actuator_driver;

// ------------------------------------------------------------------------------------------ word helpers

TEST(model_reg_word_helpers_round_trip) {
    const auto minus_one = reg::to_i32_words(-1);
    CHECK_EQ(minus_one.first, 0xFFFF);
    CHECK_EQ(minus_one.second, 0xFFFF);
    CHECK_EQ(reg::from_i32_words(0xFFFF, 0xFFFF), -1);
    CHECK_EQ(reg::from_i32_words(0x8000, 0x0000), std::numeric_limits<std::int32_t>::min());
    const auto big = reg::to_u32_words(0x12345678u);
    CHECK_EQ(big.first, 0x1234);
    CHECK_EQ(big.second, 0x5678);
    CHECK_EQ(reg::from_u32_words(0x1234, 0x5678), 0x12345678u);
    CHECK_EQ(reg::from_u32_words(0xFFFF, 0xFFFF), 0xFFFFFFFFu);
    for (std::int32_t v : {0, 1, -1, 7395, -30, 15030, 2147483647, -2147483647 - 1}) {
        const auto w = reg::to_i32_words(v);
        CHECK_EQ(reg::from_i32_words(w.first, w.second), v);
    }
}

TEST(model_reg_counts_and_scales) {
    CHECK_EQ(reg::to_counts(73.95), 7395);
    CHECK_EQ(reg::to_counts(-0.30), -30);
    CHECK_EQ(reg::to_counts(800.0, reg::kVelocityScale), 80000);
    CHECK_EQ(reg::to_counts(0.30, reg::kAccelScale), 30);
    CHECK_THROWS(reg::to_counts(std::nan("")), std::invalid_argument);
    CHECK_THROWS(reg::to_counts(std::numeric_limits<double>::infinity()), std::invalid_argument);
    CHECK_NEAR(reg::from_counts(7395), 73.95, 1e-12);
    CHECK_NEAR(reg::from_counts(-30), -0.30, 1e-12);
    CHECK_NEAR(reg::from_counts(12450, reg::kVelocityScale), 124.5, 1e-12);
}

TEST(model_reg_position_address) {
    CHECK_EQ(reg::position_address(0), 0x1000);
    CHECK_EQ(reg::position_address(2), 0x1020);
    CHECK_EQ(reg::position_address(767), 0x1000 + 16 * 767);
    CHECK_THROWS(reg::position_address(768), std::out_of_range);
    CHECK_THROWS(reg::position_address(-1), std::out_of_range);
}

TEST(model_reg_position_checksum_matches_the_rows_read_on_hardware) {
    // rod row 0 as read live (EC-R6S rod): the +0xF word makes the byte sum of the 32-byte row 0 mod 256
    const std::vector<std::uint16_t> row0 = {0x0000, 0x0000, 0x0000, 0x000A, 0x0000, 0x5DC0, 0x0000, 0x0000,
                                             0x0000, 0x0000, 0x000A, 0x002F, 0x0000, 0x0000, 0x0000, 0x00A0};
    CHECK_EQ(reg::position_checksum(row0), 0x00A0);
    unsigned total = 0;
    for (std::uint16_t w : row0) total += (w >> 8) + (w & 0xFF);
    CHECK_EQ(total & 0xFF, 0u);
    CHECK_THROWS(reg::position_checksum({1, 2, 3}), std::invalid_argument);
    // the checksum of 15 words ignores a 16th word
    CHECK_EQ(reg::position_checksum(std::vector<std::uint16_t>(row0.begin(), row0.begin() + 15)), 0x00A0);
}

TEST(model_reg_push_power_scale) {
    CHECK_EQ(reg::push_power(20), 0x33);
    CHECK_EQ(reg::push_power(70), 0xB2);
    CHECK_EQ(reg::push_power(100), 255);
    CHECK_EQ(reg::push_power(30), 76);
    CHECK_EQ(reg::push_power(50), 127);
    CHECK_EQ(reg::push_power(60), 153);
    CHECK_EQ(reg::push_power(0), 0);
    CHECK_EQ(reg::push_power(30.5), 77);  // a fraction is truncated the same way
    CHECK_THROWS(reg::push_power(std::nan("")), std::invalid_argument);
    CHECK_NEAR(reg::push_pct(0xB2), 69.8, 0.05);
    CHECK_NEAR(reg::push_pct(255), 100.0, 1e-9);
    CHECK_EQ(reg::kPpowMin, reg::push_power(20));
    CHECK_EQ(reg::kPpowMax, reg::push_power(70));
}

// ------------------------------------------------------------------------------------------ identity lookups

TEST(model_reg_product_family_and_system) {
    CHECK_EQ(reg::product_family(0xEC), "EC");
    CHECK_EQ(reg::product_family(0xED), "EC");
    CHECK_EQ(reg::product_family(0xAE), "PCON");
    CHECK_EQ(reg::product_family(0xAC), "ACON/DCON");
    CHECK_EQ(reg::product_family(0xAA), "SCON");
    CHECK_EQ(reg::product_family(0x99), reg::kGenericFamily);   // any other code works the same way
    CHECK_EQ(reg::product_family(std::nullopt), reg::kGenericFamily);
    CHECK_EQ(std::string(reg::kGenericFamily), "IAI controller");
    CHECK(reg::is_ec_family(0xEC));
    CHECK(reg::is_ec_family(0xED));
    CHECK(!reg::is_ec_family(0xAE));
    CHECK(!reg::is_ec_family(std::nullopt));
    CHECK_EQ(reg::system_name(0xEC), "EC 24 V");
    CHECK_EQ(reg::system_name(0xED), "EC 200 V");
    CHECK_EQ(reg::system_name(0xAE), "PCON");
    CHECK_EQ(reg::system_name(0xAC), "ACON / DCON");
    CHECK_EQ(reg::system_name(0xAA), "SCON");
    CHECK_EQ(reg::system_name(0x99), "");
    CHECK_EQ(reg::system_name(std::nullopt), "");
}

TEST(model_reg_ec_family_of_system_and_unit) {
    CHECK_EQ(*reg::ec_family(0xEC, 0x00), "EC standard");
    CHECK_EQ(*reg::ec_family(0xEC, 0x01), "EC slim/small");
    CHECK_EQ(*reg::ec_family(0xEC, 0x02), "EC dust/drip-proof");
    CHECK_EQ(*reg::ec_family(0xEC, 0x03), "EC high thrust");
    CHECK_EQ(*reg::ec_family(0xEC, 0x08), "EC(UM) ultra-compact");
    CHECK_EQ(*reg::ec_family(0xEC, 0x09), "EC(T) thin");
    CHECK_EQ(*reg::ec_family(0xEC, 0x0A), "EC(MS) \"2-size EC\"");
    CHECK_EQ(*reg::ec_family(0xEC, 0x0B), "EC(UC) ultra-ultra-compact");
    CHECK_EQ(*reg::ec_family(0xED, 0x00), "EC(200V)");
    CHECK(!reg::ec_family(0xEC, 0x7E).has_value());
    CHECK(!reg::ec_family(0xAE, 0x44).has_value());
    CHECK(!reg::ec_family(std::nullopt, 0x00).has_value());
    CHECK(!reg::ec_family(0xEC, std::nullopt).has_value());
}

TEST(model_reg_axis_units_and_kind) {
    CHECK_EQ(reg::axis_units(std::nullopt), "mm");
    CHECK_EQ(reg::axis_units(3), "mm");
    CHECK_EQ(reg::axis_units(4), "mm");
    CHECK_EQ(reg::axis_units(1), "deg");
    CHECK_EQ(reg::axis_units(42), "mm");  // an unknown kind is commanded as a linear axis
    CHECK_EQ(reg::axis_kind_name(std::nullopt), "unknown");
    CHECK_EQ(reg::axis_kind_name(0), "slider");
    CHECK_EQ(reg::axis_kind_name(1), "rotary");
    CHECK_EQ(reg::axis_kind_name(3), "rod");
    CHECK_EQ(reg::axis_kind_name(4), "gripper");
    CHECK_EQ(reg::axis_kind_name(9), "slider (small)");
    CHECK_EQ(reg::axis_kind_name(10), "rod (small)");
    CHECK_EQ(reg::axis_kind_name(11), "wire");
    CHECK_EQ(reg::axis_kind_name(42), "kind 42");
    // a unit without AKID (a CON controller): the ATYP TYPE bit decides, AKID wins when present
    CHECK_EQ(reg::axis_units(std::nullopt, true), "deg");
    CHECK_EQ(reg::axis_units(std::nullopt, false), "mm");
    CHECK_EQ(reg::axis_kind_name(std::nullopt, true), "rotary");
    CHECK_EQ(reg::axis_kind_name(std::nullopt, false), "unknown");
    CHECK_EQ(reg::axis_units(3, true), "mm");
    CHECK_EQ(reg::axis_kind_name(3, true), "rod");
    CHECK_EQ(reg::axis_units(1, false), "deg");
}

// ------------------------------------------------------------------------------------------ directory lookups

TEST(model_reg_category_names) {
    CHECK_EQ(reg::category_name(0x0010), "position table");
    CHECK_EQ(reg::category_name(0x0005), "alarm history");
    CHECK_EQ(reg::category_name(0x0101), "monitor");
    CHECK_EQ(reg::category_name(0x0012), "?");
    CHECK_EQ(reg::category_name(0x7777), "?");
    CHECK(reg::category_name(0x0007).find("axis parameters") == 0);
}

TEST(model_reg_category_extent_rules) {
    // one size for every member: DQ x MS x LQ, exact
    const reg::BlockExtent ordinary = reg::category_extent(0x0007, 26, 1, 2);
    CHECK_EQ(ordinary.words, 52);
    CHECK(ordinary.exact);
    CHECK_EQ(ordinary.words_max, 52);
    const reg::BlockExtent rows = reg::category_extent(0x0010, 8, 768, 2);
    CHECK_EQ(rows.words, 12288);
    // class 1 (support) has a fixed member order with known sizes: 6 members = 8 words, 11 members = 18
    CHECK_EQ(reg::category_extent(1, 6, 1, 0).words, 8);
    CHECK_EQ(reg::category_extent(1, 11, 1, 0).words, 18);
    CHECK(reg::category_extent(1, 11, 1, 0).exact);
    // a known mixed block (identification 36 members = 72 words, monitor 22 members = 34 words)
    CHECK_EQ(reg::category_extent(6, 36, 1, 0).words, 72);
    CHECK_EQ(reg::category_extent(0x0101, 22, 1, 0).words, 34);
    CHECK(reg::category_extent(0x0101, 22, 1, 0).exact);
    // the same block with another member count is not the known one: uncertain
    const reg::BlockExtent odd = reg::category_extent(0x0101, 23, 1, 0);
    CHECK_EQ(odd.words, 23);
    CHECK(!odd.exact);
    CHECK_EQ(odd.words_max, 46);
    // an unknown mixed block: at least one word per member, at most two
    const reg::BlockExtent unknown = reg::category_extent(0x0012, 7, 1, 0);
    CHECK_EQ(unknown.words, 7);
    CHECK(!unknown.exact);
    CHECK_EQ(unknown.words_max, 14);
}

TEST(model_reg_support_members_order) {
    const auto& members = reg::support_members();
    REQUIRE(members.size() == 11u);
    CHECK_EQ(members[0].first, "SUPT");
    CHECK_EQ(members[0].second, 2);
    CHECK_EQ(members[1].first, "DIAG");
    CHECK_EQ(members[1].second, 1);
    CHECK_EQ(members[2].first, "MVPT");
    CHECK_EQ(members[5].first, "SUP2");
    CHECK_EQ(members[5].second, 2);
    CHECK_EQ(members[10].first, "SUP3");
    int words = 0;
    for (const auto& m : members) words += m.second;
    CHECK_EQ(words, 18);
}

// ------------------------------------------------------------------------------------------ text

TEST(model_reg_words_to_ascii_strips_padding_and_junk) {
    CHECK_EQ(reg::words_to_ascii({0x4543, 0x2D52, 0x3653}), "EC-R6S");
    CHECK_EQ(reg::words_to_ascii({0x4543, 0x0000, 0x0000}), "EC");                // NUL padding
    CHECK_EQ(reg::words_to_ascii({0x2020, 0x4543, 0x2020}), "EC");                // space padding both sides
    CHECK_EQ(reg::words_to_ascii({0x4501, 0x4300}), "EC");                        // a control byte is dropped
    CHECK_EQ(reg::words_to_ascii({0x45FF, 0x8043}), "EC");                        // so are bytes above 0x7E
    CHECK_EQ(reg::words_to_ascii({0x4500, 0x4300}), "E C");                       // NUL inside becomes a space
    CHECK_EQ(reg::words_to_ascii({}), "");
    CHECK_EQ(reg::words_to_ascii({0x0000, 0x0000}), "");
}

TEST(model_reg_words_to_text_rejects_binary) {
    CHECK_EQ(*reg::words_to_text({0x4543, 0x2D52}), "EC-R");
    CHECK_EQ(*reg::words_to_text({0x5043, 0x4F4E, 0x2020, 0x2020}), "PCON");
    CHECK(!reg::words_to_text({0x4543, 0x0001}).has_value());   // a control byte: not a text field
    CHECK(!reg::words_to_text({0x8B85, 0x6362}).has_value());   // binary data
    CHECK(!reg::words_to_text({0x0000, 0x0000}).has_value());   // blank
    CHECK(!reg::words_to_text({0x2020, 0x2020}).has_value());   // blank
    CHECK(!reg::words_to_text({}).has_value());
}

TEST(model_reg_printable_ascii) {
    CHECK_EQ(reg::printable("EC 24 V"), "EC 24 V");
    CHECK_EQ(reg::printable("caf\xC3\xA9"), "caf");
    CHECK_EQ(reg::printable(std::string("a\tb\nc\x7F") + "d"), "abcd");
    CHECK_EQ(reg::printable(""), "");
}

// ------------------------------------------------------------------------------------------ EC series / minimum speed

TEST(model_reg_ec_series_from_the_model_string) {
    CHECK_EQ(*reg::ec_series(std::string("EC-DS4H-150-MOT-WA-WL2")), "S4");
    CHECK_EQ(*reg::ec_series(std::string("EC-R6S-150-1-PN")), "R6");
    CHECK_EQ(*reg::ec_series(std::string("EC-GRBP8M-20-1-1-PN-WA-WL2")), "GRB");
    CHECK_EQ(*reg::ec_series(std::string("EC-RR8-10-100")), "RR8");
    CHECK_EQ(*reg::ec_series(std::string("EC-RR10-5-100")), "RR10");
    CHECK_EQ(*reg::ec_series(std::string("EC-RTC12-20")), "RTC");
    CHECK_EQ(*reg::ec_series(std::string("EC-RTB10")), "RTB");
    CHECK_EQ(*reg::ec_series(std::string("EC-GRC6M-10")), "GRC");
    CHECK_EQ(*reg::ec_series(std::string("EC-WEGR4")), "WEGR");
    CHECK_EQ(*reg::ec_series(std::string("EC-WER3")), "WER");
    CHECK_EQ(*reg::ec_series(std::string("ec-s3sa-100")), "S3");   // case-insensitive
    CHECK_EQ(*reg::ec_series(std::string("EC-S6SAH-100-1-PN")), "S6");
    CHECK_EQ(*reg::ec_series(std::string("EC-DR6S-150")), "R6");   // the optional D of the 200 V models
    CHECK(!reg::ec_series(std::string("PCON-CB")).has_value());
    CHECK(!reg::ec_series(std::string("EC-XYZ")).has_value());
    CHECK(!reg::ec_series(std::string("")).has_value());
    CHECK(!reg::ec_series(std::nullopt).has_value());
}

TEST(model_reg_recommended_min_speed_by_series_and_lead) {
    auto r = reg::recommended_min_speed(std::string("EC-R6S-150-1-PN"), 20.0);
    CHECK_NEAR(*r.first, 25.0, 1e-9);
    CHECK_EQ(r.second, "series R6, lead 20 mm (DOC)");
    r = reg::recommended_min_speed(std::string("EC-DS4H-150"), 5.0);
    CHECK_NEAR(*r.first, 7.0, 1e-9);
    CHECK_EQ(r.second, "series S4, lead 5 mm (DOC)");
    r = reg::recommended_min_speed(std::string("EC-DS4H-150"), 2.5);
    CHECK_NEAR(*r.first, 4.0, 1e-9);
    CHECK_EQ(r.second, "series S4, lead 2.5 mm (DOC)");
    // grippers and wire types have one value per series, the lead does not matter
    r = reg::recommended_min_speed(std::string("EC-GRBP8M-20-1-1-PN-WA-WL2"), 1.05);
    CHECK_NEAR(*r.first, 5.0, 1e-9);
    CHECK_EQ(r.second, "series GRB (DOC)");
    r = reg::recommended_min_speed(std::string("EC-GRCP6M"), std::nullopt);
    CHECK_NEAR(*r.first, 10.0, 1e-9);
    r = reg::recommended_min_speed(std::string("EC-RTC12"), std::nullopt);
    CHECK_NEAR(*r.first, 20.0, 1e-9);
}

TEST(model_reg_recommended_min_speed_fallbacks_warn_never_refuse) {
    // a lead the series table does not list: lead x 1.25 (lead / 800 pulses / 1 ms, DOC ME0155 p.139)
    auto r = reg::recommended_min_speed(std::string("EC-DS4H-150"), 7.0);
    CHECK_NEAR(*r.first, 8.75, 1e-9);
    CHECK_EQ(r.second, "lead 7 mm x 1.25 (DOC ME0155 p.139)");
    // an unknown model with a lead
    r = reg::recommended_min_speed(std::string("PCON-CB (FB)"), 16.0);
    CHECK_NEAR(*r.first, 20.0, 1e-9);
    r = reg::recommended_min_speed(std::nullopt, 4.0);
    CHECK_NEAR(*r.first, 5.0, 1e-9);
    // nothing tells
    r = reg::recommended_min_speed(std::nullopt, std::nullopt);
    CHECK(!r.first.has_value());
    CHECK_EQ(r.second, "no documented minimum known for this unit");
    r = reg::recommended_min_speed(std::string("EC-DS4H-150"), 0.0);
    CHECK(!r.first.has_value());
}

// ------------------------------------------------------------------------------------------ position entry

namespace {

const std::vector<std::uint16_t> kRodRow1 = {0x0000, 0x0BB8, 0x0000, 0x2EE0, 0x0000, 0x0002, 0x0000, 0x0000,
                                             0x0000, 0x0000, 0x0032, 0x0032, 0x000A, 0x0000, 0x0016, 0x00A9};

}  // namespace

TEST(model_position_decode_a_push_row_read_on_hardware) {
    // rod simple-data row 261 as read live: 30.00 mm, band 120 mm, 0.02 mm/s, 0.50 G, PPOW 10, CTLF 0x0016
    const PositionEntry e = PositionEntry::from_registers(kRodRow1);
    CHECK_NEAR(e.position_mm, 30.00, 1e-9);
    CHECK_NEAR(e.band_mm, 120.00, 1e-9);
    CHECK_NEAR(e.speed_mm_s, 0.02, 1e-9);
    CHECK_NEAR(e.zone_plus_mm, 0.0, 1e-9);
    CHECK_NEAR(e.accel_g, 0.50, 1e-9);
    CHECK_NEAR(e.decel_g, 0.50, 1e-9);
    CHECK_EQ(e.push_power, 10);
    CHECK_EQ(e.load_threshold, 0);
    CHECK_EQ(e.control_flags, 0x0016);
    REQUIRE(e.checksum.has_value());
    CHECK_EQ(*e.checksum, 0x00A9);
    CHECK(e.is_push());           // CTLF bit 1
    CHECK(!e.incremental());      // CTLF bit 3
    REQUIRE(e.checksum_ok().has_value());
    CHECK(*e.checksum_ok());
}

TEST(model_position_round_trip_including_negative_positions) {
    PositionEntry e;
    e.position_mm = -12.34;
    e.band_mm = 0.25;
    e.speed_mm_s = 123.45;
    e.zone_plus_mm = 100.5;
    e.zone_minus_mm = -5.0;
    e.accel_g = 0.30;
    e.decel_g = 0.75;
    e.push_power = 0x33;
    e.load_threshold = 7;
    e.control_flags = 0x0030;
    const std::vector<std::uint16_t> words = e.to_registers_with_checksum();
    REQUIRE(words.size() == 16u);
    CHECK_EQ(words[0], 0xFFFF);                      // -1234 as int32, high word first
    CHECK_EQ(words[1], static_cast<std::uint16_t>(-1234 & 0xFFFF));
    CHECK_EQ(words[10], 30);
    CHECK_EQ(words[11], 75);
    CHECK_EQ(words[14], 0x0030);
    CHECK_EQ(words[15], reg::position_checksum(words));
    const PositionEntry back = PositionEntry::from_registers(words);
    CHECK_NEAR(back.position_mm, -12.34, 1e-9);
    CHECK_NEAR(back.zone_minus_mm, -5.0, 1e-9);
    CHECK_NEAR(back.speed_mm_s, 123.45, 1e-9);
    CHECK(*back.checksum_ok());
    CHECK(back.checksum.has_value());
    PositionEntry same = back;
    same.checksum = std::nullopt;
    CHECK(!same.checksum_ok().has_value());   // an entry built in code has no checksum to compare
    CHECK(PositionEntry::from_registers(e.to_registers()) != back);  // the checksum is part of what was read
    CHECK(PositionEntry::from_registers(e.to_registers()) == same);
}

TEST(model_position_checksum_mismatch_is_visible) {
    std::vector<std::uint16_t> words = kRodRow1;
    words[15] = 0x00AA;
    const PositionEntry e = PositionEntry::from_registers(words);
    CHECK(!*e.checksum_ok());
    // 15 words decode too (no checksum read)
    const PositionEntry fifteen = PositionEntry::from_registers(std::vector<std::uint16_t>(kRodRow1.begin(), kRodRow1.begin() + 15));
    CHECK(!fifteen.checksum.has_value());
    CHECK_THROWS(PositionEntry::from_registers(std::vector<std::uint16_t>(14, 0)), std::invalid_argument);
}

TEST(model_position_unencodable_values_are_refused_before_anything_is_sent) {
    PositionEntry e;
    e.push_power = 70000;
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.control_flags = -1;
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.accel_g = 700.0;   // 70000 counts do not fit a word
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.decel_g = -0.5;
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.position_mm = 1e9;   // does not fit int32 counts
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.band_mm = -1.0;      // the band is unsigned
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    e = PositionEntry();
    e.speed_mm_s = std::nan("");
    CHECK_THROWS(e.to_registers(), std::invalid_argument);
    CHECK_EQ(PositionEntry().to_registers().size(), 15u);
}

TEST(model_position_defaults_and_flags) {
    const PositionEntry d;
    CHECK_NEAR(d.band_mm, 0.10, 1e-12);
    CHECK_NEAR(d.speed_mm_s, 50.0, 1e-12);
    CHECK_NEAR(d.accel_g, 0.30, 1e-12);
    CHECK(!d.is_push());
    PositionEntry e;
    e.control_flags = reg::kCtlfPush | reg::kCtlfInc;
    CHECK(e.is_push());
    CHECK(e.incremental());
    e.control_flags = reg::kCtlfPushAway;
    CHECK(!e.is_push());
}
