// SEL unit: the frames of the SEL protocol (src/sel_frame.hpp). The two checksum vectors are the manuals' own
// examples (DOC): the 209H query `!99209001005` with checksum 54 (serial Format B manual) and the test-call reply
// `#992001234567890` with checksum 34 (the Ethernet manual's telnet example, whose request carries "@@"). Every other
// case is built from those rules; nothing here ran against a SEL controller (UNVERIFIED). No serial port is opened.
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "harness.hpp"
#include "sel_frame.hpp"

using namespace iai_actuator_driver;
using sel::FrameKind;
using sel::FrameStatus;

namespace {

/// `body` (start .. data) with its correct checksum and CR LF.
std::string with_checksum(const std::string& body) { return body + sel::hex(sel::checksum(body), 2) + "\r\n"; }

}  // namespace

TEST(sel_frame_checksum_manual_vector_209h_query) {
    CHECK_EQ(static_cast<int>(sel::checksum("!99209001005")), 0x54);
    CHECK_EQ(sel::encode_command(0x99, 0x209, "001005"), std::string("!9920900100554\r\n"));
    const sel::DecodeResult r = sel::decode("!9920900100554\r\n");
    REQUIRE(r.ok());
    CHECK(r.frame.kind == FrameKind::command);
    CHECK_EQ(r.frame.station, 0x99);
    CHECK_EQ(r.frame.id, 0x209);
    CHECK_EQ(r.frame.data, std::string("001005"));
    CHECK(!r.frame.checksum_skipped);
}

TEST(sel_frame_checksum_manual_vector_test_call_reply) {
    CHECK_EQ(static_cast<int>(sel::checksum("#992001234567890")), 0x34);
    CHECK_EQ(sel::encode_reply(0x99, 0x200, "1234567890"), std::string("#99200123456789034\r\n"));
    const sel::DecodeResult r = sel::decode_reply("#99200123456789034\r\n", 0x99, 0x200);
    REQUIRE(r.ok());
    CHECK(r.frame.kind == FrameKind::reply);
    CHECK_EQ(r.frame.data, std::string("1234567890"));
}

TEST(sel_frame_no_checksum_is_accepted_on_a_command_only) {
    // the telnet example's request, typed with "@@" in place of the checksum
    CHECK_EQ(sel::encode_command(0x99, 0x200, "1234567890", false), std::string("!992001234567890@@\r\n"));
    const sel::DecodeResult r = sel::decode("!992001234567890@@\r\n");
    REQUIRE(r.ok());
    CHECK(r.frame.checksum_skipped);
    CHECK_EQ(r.frame.id, 0x200);
    CHECK_EQ(r.frame.data, std::string("1234567890"));
    // a reply always carries its checksum: "@@" there is a checksum that does not match
    CHECK(sel::decode("#992001234567890@@\r\n").status == FrameStatus::bad_checksum);
    CHECK(sel::decode_reply("#992001234567890@@\r\n", 0x99, 0x200).status == FrameStatus::bad_checksum);
    CHECK(sel::decode("&99001@@\r\n").status == FrameStatus::bad_checksum);
    // half of "@@" is neither the marker nor hex
    CHECK(sel::decode("!992001234567890@4\r\n").status == FrameStatus::bad_hex);
}

TEST(sel_frame_error_reply_is_ten_bytes_and_matches_any_id) {
    const std::string frame = sel::encode_error_reply(0x99, 0xC1A);
    CHECK_EQ(frame.size(), std::size_t{10});
    CHECK_EQ(frame, with_checksum("&99C1A"));
    const sel::DecodeResult r = sel::decode_reply(frame, 0x99, 0x234);
    REQUIRE(r.ok());
    CHECK(r.frame.kind == FrameKind::error_reply);
    CHECK_EQ(r.frame.id, 0xC1A);
    CHECK(r.frame.data.empty());
    CHECK(sel::decode_reply(frame, 0x98, 0x234).status == FrameStatus::wrong_station);
    CHECK(sel::decode(with_checksum("&99C1A00")).status == FrameStatus::bad_format);  // an error reply has no data
}

TEST(sel_frame_hex_case_is_written_upper_and_read_either) {
    CHECK_EQ(sel::encode_command(0xAB, 0x2A1, "0f"), with_checksum("!AB2A10f"));
    const std::string lower = "#ab2a10a";
    REQUIRE(sel::checksum(lower) == 0x3B);
    const std::string frame = lower + "3b\r\n";  // lower case in every hex field, the checksum too
    const sel::DecodeResult r = sel::decode_reply(frame, 0xAB, 0x2A1);
    REQUIRE(r.ok());
    CHECK_EQ(r.frame.station, 0xAB);
    CHECK_EQ(r.frame.id, 0x2A1);
    CHECK_EQ(sel::parse_hex("fF"), std::optional<std::uint32_t>(0xFF));
}

TEST(sel_frame_negative_too_short) {
    CHECK(sel::decode("").status == FrameStatus::too_short);
    CHECK(sel::decode("#9920034\r").status == FrameStatus::too_short);  // 9 bytes
    CHECK(sel::decode("&99001").status == FrameStatus::too_short);
    CHECK(sel::decode(with_checksum("#99200")).ok());                // 10 bytes: the shortest normal reply
}

TEST(sel_frame_negative_bad_checksum) {
    CHECK(sel::decode("!9920900100555\r\n").status == FrameStatus::bad_checksum);
    CHECK(sel::decode("#99200123456789043\r\n").status == FrameStatus::bad_checksum);
    CHECK(sel::decode("!9920900200554\r\n").status == FrameStatus::bad_checksum);  // one data byte changed
    CHECK(sel::decode_reply("#99200123456789035\r\n", 0x99, 0x200).status == FrameStatus::bad_checksum);
}

TEST(sel_frame_negative_wrong_terminator) {
    CHECK(sel::decode("!9920900100554\n").status == FrameStatus::bad_terminator);
    CHECK(sel::decode("!9920900100554\r").status == FrameStatus::bad_terminator);
    CHECK(sel::decode("!9920900100554\n\r").status == FrameStatus::bad_terminator);
    CHECK(sel::decode("!9920900100554").status == FrameStatus::bad_terminator);
    CHECK(sel::decode("!9920900100554\r\n\r\n").status == FrameStatus::bad_terminator);  // two frames' worth
    CHECK(sel::decode(with_checksum("#99200\r\n12")).status == FrameStatus::bad_terminator);
    CHECK(sel::decode(with_checksum("#99200\t12")).status == FrameStatus::bad_format);
}

TEST(sel_frame_negative_bad_hex) {
    CHECK(sel::decode(with_checksum("!9G209001005")).status == FrameStatus::bad_hex);  // station
    CHECK(sel::decode(with_checksum("!9920Z001005")).status == FrameStatus::bad_hex);  // message id
    CHECK(sel::decode(with_checksum("& 9001")).status == FrameStatus::bad_hex);         // error-reply station
    CHECK(sel::decode("!99209001005G4\r\n").status == FrameStatus::bad_hex);            // checksum
    CHECK(sel::decode("!99209001005-4\r\n").status == FrameStatus::bad_hex);
    CHECK(!sel::parse_hex(""));
    CHECK(!sel::parse_hex("123456789"));
    CHECK(!sel::parse_hex("1 2"));
    CHECK(!sel::parse_hex("+1"));
    CHECK(!sel::parse_hex_signed("x0"));
}

TEST(sel_frame_negative_wrong_station_and_id_and_start) {
    const std::string reply = sel::encode_reply(0x99, 0x200, "1234567890");
    CHECK(sel::decode_reply(reply, 0x01, 0x200).status == FrameStatus::wrong_station);
    const sel::DecodeResult other = sel::decode_reply(reply, 0x99, 0x201);
    CHECK(other.status == FrameStatus::wrong_id);  // a late reply to the previous command
    CHECK_EQ(other.frame.id, 0x200);
    // the host never accepts a command as a reply (its own echo on a two-wire line, say)
    CHECK(sel::decode_reply("!9920900100554\r\n", 0x99, 0x209).status == FrameStatus::bad_start);
    CHECK(sel::decode(with_checksum("$99200")).status == FrameStatus::bad_start);
    CHECK(sel::decode(with_checksum("99200x")).status == FrameStatus::bad_start);
}

TEST(sel_frame_hex_fields_unsigned_and_signed) {
    CHECK_EQ(sel::hex(0x64, 4), std::string("0064"));
    CHECK_EQ(sel::hex(0x1388, 8), std::string("00001388"));
    CHECK_EQ(sel::hex(0xFFFFFFFFu, 8), std::string("FFFFFFFF"));
    CHECK_EQ(sel::hex(0, 1), std::string("0"));
    CHECK_THROWS(sel::hex(0x100, 2), std::out_of_range);
    CHECK_THROWS(sel::hex(1, 0), std::out_of_range);
    CHECK_THROWS(sel::hex(1, 9), std::out_of_range);
    // signed: two's complement over the field (UNVERIFIED: the manuals show no negative example)
    CHECK_EQ(sel::hex_signed(5000), std::string("00001388"));
    CHECK_EQ(sel::hex_signed(-1), std::string("FFFFFFFF"));
    CHECK_EQ(sel::hex_signed(-5000), std::string("FFFFEC78"));
    CHECK_EQ(sel::hex_signed(std::numeric_limits<std::int32_t>::min()), std::string("80000000"));
    CHECK_EQ(sel::hex_signed(std::numeric_limits<std::int32_t>::max()), std::string("7FFFFFFF"));
    CHECK_EQ(sel::hex_signed(-1, 4), std::string("FFFF"));
    CHECK_EQ(sel::hex_signed(-32768, 4), std::string("8000"));
    CHECK_THROWS(sel::hex_signed(32768, 4), std::out_of_range);
    CHECK_THROWS(sel::hex_signed(-32769, 4), std::out_of_range);
    CHECK_EQ(sel::parse_hex_signed("FFFFEC78"), std::optional<std::int32_t>(-5000));
    CHECK_EQ(sel::parse_hex_signed("80000000"), std::optional<std::int32_t>(std::numeric_limits<std::int32_t>::min()));
    CHECK_EQ(sel::parse_hex_signed("7FFFFFFF"), std::optional<std::int32_t>(std::numeric_limits<std::int32_t>::max()));
    CHECK_EQ(sel::parse_hex_signed("8000"), std::optional<std::int32_t>(-32768));
    CHECK_EQ(sel::parse_hex_signed("7FFF"), std::optional<std::int32_t>(32767));
    CHECK_EQ(sel::parse_hex_signed("00001388"), std::optional<std::int32_t>(5000));
}

TEST(sel_frame_encode_refuses_what_cannot_be_framed) {
    CHECK_THROWS(sel::encode_command(0x100, 0x200, ""), std::out_of_range);
    CHECK_THROWS(sel::encode_command(-1, 0x200, ""), std::out_of_range);
    CHECK_THROWS(sel::encode_command(0x99, 0x1000, ""), std::out_of_range);
    CHECK_THROWS(sel::encode_error_reply(0x99, -1), std::out_of_range);
    CHECK_THROWS(sel::encode_command(0x99, 0x200, "12\r\n"), std::invalid_argument);
    CHECK_THROWS(sel::encode_command(0x99, 0x200, std::string("1\0", 2)), std::invalid_argument);
    // round trip of every station byte
    for (int station = 0; station <= 0xFF; ++station) {
        const sel::DecodeResult r = sel::decode_reply(sel::encode_reply(station, 0x212, "01"), station, 0x212);
        CHECK(r.ok());
        CHECK_EQ(r.frame.station, station);
    }
    CHECK_EQ(std::string(sel::to_string(FrameStatus::wrong_station)), std::string("wrong_station"));
}
