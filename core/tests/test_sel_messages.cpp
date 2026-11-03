// SEL unit: the phase-1 messages (src/sel_messages.hpp). Requests are checked byte for byte against the field layouts
// of the serial Format B manual (DOC; the 209H query is the manual's own checksum example), replies are written by hand
// from the same layouts and parsed back. Nothing here ran against a SEL controller (UNVERIFIED). No serial port is
// opened.
#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "harness.hpp"
#include "sel_frame.hpp"
#include "sel_messages.hpp"

using namespace iai_actuator_driver;
using sel::MessageKind;
using sel::ReplyFormatError;

namespace {

constexpr int kStation = 0x99;

/// The command frame of `r` for station 99h.
std::string frame_of(const sel::Request& r) { return sel::encode_request(r, kStation); }

/// `data` sent back as station 99h's normal reply to `id`, accepted by the host's frame check, and its data returned.
std::string reply(int id, const std::string& data) {
    const sel::DecodeResult r = sel::decode_reply(sel::encode_reply(kStation, id, data), kStation, id);
    REQUIRE(r.ok());
    return r.frame.data;
}

double from_bits(std::uint64_t bits) {
    double v = 0.0;
    std::memcpy(&v, &bits, sizeof v);
    return v;
}

std::uint64_t to_bits(double v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    return bits;
}

}  // namespace

TEST(sel_messages_manual_vector_and_layout_checksums) {
    // the manual's own example: 209H, head position 001, 005 records, checksum 54
    const sel::Request q = sel::position_query(1, 5);
    CHECK_EQ(q.id, sel::kPositionQuery);
    CHECK(q.kind == MessageKind::query);
    CHECK_EQ(frame_of(q), std::string("!9920900100554\r\n"));
    // built from the layouts: servo ON axis 1, and a 5.000 mm absolute move at 100 mm/s with the parameter's acc/dec
    CHECK_EQ(frame_of(sel::servo(sel::axis_bit(1), true)), std::string("!99232011BC\r\n"));
    const sel::Request move = sel::move_absolute(sel::axis_bit(1), {0.0, 0.0, 100.0}, {5.0});
    CHECK_EQ(frame_of(move), std::string("!9923401000000000064000013886B\r\n"));
    CHECK_EQ(frame_of(sel::version_query()), std::string("!99201000B6\r\n"));
    CHECK_EQ(sel::encode_request(q, kStation, false), std::string("!99209001005@@\r\n"));
}

TEST(sel_messages_axes_and_kinds) {
    CHECK_EQ(static_cast<int>(sel::axis_bit(1)), 0x01);
    CHECK_EQ(static_cast<int>(sel::axis_bit(8)), 0x80);
    CHECK_THROWS(sel::axis_bit(0), std::out_of_range);
    CHECK_THROWS(sel::axis_bit(9), std::out_of_range);
    CHECK_EQ(sel::axis_count(0xA5), 4);
    CHECK_EQ(sel::axis_count(0), 0);
    CHECK(sel::axes_of(0xA5) == std::vector<int>({1, 3, 6, 8}));
    CHECK(sel::axes_of(0).empty());
    CHECK(sel::system_status_query().kind == MessageKind::query);
    CHECK(sel::stop(0x01).kind == MessageKind::execute);
    CHECK(sel::set_output(300, true).kind == MessageKind::execute);
    CHECK(sel::alarm_reset().reply_expected);
    const sel::Request reset = sel::software_reset();
    CHECK_EQ(reset.id, sel::kSoftwareReset);
    CHECK(reset.kind == MessageKind::execute);
    CHECK(!reset.reply_expected);
    CHECK(reset.data.empty());
}

TEST(sel_messages_no_data_replies) {
    sel::parse_no_data(sel::kServo, reply(sel::kServo, ""));
    CHECK_THROWS(sel::parse_no_data(sel::kServo, "0"), ReplyFormatError);
    try {
        sel::parse_no_data(sel::kStop, "00");
        CHECK(false);
    } catch (const ReplyFormatError& e) {
        CHECK_EQ(e.message_id(), sel::kStop);
        CHECK(std::string(e.what()).find("238H") != std::string::npos);
    }
}

TEST(sel_messages_200h_test_call) {
    const sel::Request r = sel::test_call("1234567890");
    CHECK_EQ(frame_of(sel::test_call("1234567890")), std::string("!992001234567890") +
                                                         sel::hex(sel::checksum("!992001234567890"), 2) + "\r\n");
    CHECK_EQ(sel::parse_test_call(reply(sel::kTestCall, "1234567890"), r.data), std::string("1234567890"));
    CHECK_THROWS(sel::parse_test_call("1234567891", r.data), ReplyFormatError);
    CHECK_THROWS(sel::parse_test_call("123456789", r.data), ReplyFormatError);
    CHECK_THROWS(sel::parse_test_call("12345678900", r.data), ReplyFormatError);
    CHECK_THROWS(sel::test_call("123456789"), std::invalid_argument);
    CHECK_THROWS(sel::test_call("123456789\n"), std::invalid_argument);
}

TEST(sel_messages_201h_version) {
    CHECK_EQ(sel::version_query(2, 1).data, std::string("021"));
    CHECK_THROWS(sel::version_query(0x100, 0), std::out_of_range);
    CHECK_THROWS(sel::version_query(0, 0x10), std::out_of_range);
    // unit type 00, device 0, model B9 (TTA), unit 00, version 0123, 2025-11-03 15:30:45 (every field hex)
    const std::string data = "000B9000123" "07E90B030F1E2D";
    const sel::VersionInfo v = sel::parse_version(reply(sel::kVersionQuery, data), 0, 0);
    CHECK_EQ(v.model_code, 0xB9);
    CHECK_EQ(v.unit_code, 0);
    CHECK_EQ(v.version, 0x0123);
    CHECK_EQ(v.year, 2025);
    CHECK_EQ(v.month, 11);
    CHECK_EQ(v.day, 3);
    CHECK_EQ(v.hour, 15);
    CHECK_EQ(v.minute, 30);
    CHECK_EQ(v.second, 45);
    CHECK_EQ(sel::parse_version("000bd000123" "07e90b030f1e2d", 0, 0).model_code, 0xBD);  // lower case read too
    CHECK_THROWS(sel::parse_version(data, 1, 0), ReplyFormatError);  // the echoed unit type differs
    CHECK_THROWS(sel::parse_version(data, 0, 1), ReplyFormatError);  // the echoed device differs
    CHECK_THROWS(sel::parse_version(data.substr(0, 24), 0, 0), ReplyFormatError);
    CHECK_THROWS(sel::parse_version(data + "0", 0, 0), ReplyFormatError);
    CHECK_THROWS(sel::parse_version("000B9000123" "07E90B030F1E2G", 0, 0), ReplyFormatError);
}

TEST(sel_messages_setting_inquiry) {
    const sel::Request q = sel::setting_query(sel::kSettingPositionOutput);
    CHECK_EQ(q.id, 0x202);
    CHECK_EQ(q.data, std::string("0900B9001"));  // setting 185 = 0B9h
    CHECK_EQ(sel::setting_query(sel::kSettingCoordinateSystem).data, std::string("0900BD001"));
    CHECK_EQ(sel::setting_query(200).data, std::string("0900C8001"));  // the manual's example number
    CHECK_THROWS(sel::setting_query(0x1000), std::out_of_range);
    CHECK_EQ(sel::parse_setting(reply(0x202, "0900000000E")), 0x0000000Eu);
    CHECK_EQ(sel::parse_setting("09087654321"), 0x87654321u);
    CHECK_THROWS(sel::parse_setting("0910000000E"), ReplyFormatError);
    CHECK_THROWS(sel::parse_setting("0900000000"), ReplyFormatError);
    CHECK_THROWS(sel::parse_setting("0900000000E0"), ReplyFormatError);
}

TEST(sel_messages_215h_system_status) {
    CHECK(sel::system_status_query().data.empty());
    // AUTO, no critical error, latest error 0A1h, E-stop on, program running + flash writing, ready + operating
    const sel::SystemStatus s = sel::parse_system_status(reply(sel::kSystemStatusQuery, "10000A108210600"));
    CHECK_EQ(s.mode, 1);
    CHECK(s.auto_mode());
    CHECK_EQ(s.critical_error, 0);
    CHECK_EQ(s.latest_error, 0xA1);
    CHECK(s.emergency_stop());
    CHECK(!s.safety_gate_open());
    CHECK(!s.mode_switch_manual());
    CHECK(s.program_running());
    CHECK(s.flash_writing());
    CHECK(!s.servo_interlock());
    CHECK(s.system_ready());
    CHECK(s.operating());
    CHECK(!s.drive_source_cutoff());
    const sel::SystemStatus manual = sel::parse_system_status("200000001000100");
    CHECK(!manual.auto_mode());
    CHECK(manual.mode_switch_manual());
    CHECK(!manual.system_ready());
    CHECK(manual.drive_source_cutoff());
    CHECK_THROWS(sel::parse_system_status("10000A10821060"), ReplyFormatError);
    CHECK_THROWS(sel::parse_system_status("10000A1082106000"), ReplyFormatError);
    CHECK_THROWS(sel::parse_system_status("1000-A108210600"), ReplyFormatError);
}

TEST(sel_messages_212h_axis_status) {
    CHECK_EQ(sel::axis_status_query(0xFF).data, std::string("FF"));
    CHECK_THROWS(sel::axis_status_query(0), std::invalid_argument);
    // axes 1 and 3 present: axis 1 homed, servo on, completed, home sensor, at 5.000 mm; axis 3 in use, servo on,
    // not homed, error 0A2h, encoder 82h, at -5.000 mm
    const std::string data = "05" "1C40000000001388" "0900A282FFFFEC78";
    const sel::AxisStatusReply r = sel::parse_axis_status(reply(sel::kAxisStatusQuery, data), 0xFF);
    CHECK_EQ(static_cast<int>(r.present), 0x05);
    REQUIRE(r.axes.size() == 2);
    const sel::AxisStatus& a1 = r.axes[0];
    CHECK_EQ(a1.axis, 1);
    CHECK(!a1.in_use());
    CHECK(a1.homed());
    CHECK_EQ(a1.home_state(), 2);
    CHECK(a1.servo_on());
    CHECK(a1.completed());
    CHECK(!a1.push_error());
    CHECK_EQ(static_cast<int>(a1.sensor), 4);
    CHECK_NEAR(a1.position, 5.0, 1e-12);
    const sel::AxisStatus& a3 = r.axes[1];
    CHECK_EQ(a3.axis, 3);
    CHECK(a3.in_use());
    CHECK_EQ(a3.home_state(), 0);
    CHECK(!a3.completed());
    CHECK_EQ(a3.error_code, 0xA2);
    CHECK_EQ(static_cast<int>(a3.encoder), 0x82);
    CHECK_NEAR(a3.position, -5.0, 1e-12);
    CHECK_EQ(sel::parse_axis_status("00", 0xFF).axes.size(), std::size_t{0});  // no driver on any asked-for axis
    CHECK_EQ(sel::parse_axis_status("01" "2a40000080000000", 0x01).axes[0].home_state(), 1);  // lower case, homing
    CHECK_NEAR(sel::parse_axis_status("01" "0000000080000000", 0x01).axes[0].position, -2147483.648, 1e-9);
    CHECK_THROWS(sel::parse_axis_status(data, 0x01), ReplyFormatError);  // axis 3 was not asked for
    CHECK_THROWS(sel::parse_axis_status(data.substr(0, data.size() - 1), 0xFF), ReplyFormatError);
    CHECK_THROWS(sel::parse_axis_status(data + "0", 0xFF), ReplyFormatError);
    CHECK_THROWS(sel::parse_axis_status("07" "1C40000000001388" "0900A282FFFFEC78", 0xFF), ReplyFormatError);
    CHECK_THROWS(sel::parse_axis_status("01" "1C4000000000138X", 0x01), ReplyFormatError);
    CHECK_THROWS(sel::parse_axis_status("", 0x01), ReplyFormatError);
}

TEST(sel_messages_216h_error_detail) {
    CHECK_EQ(sel::error_detail_query(1, 2).data, std::string("102000"));
    CHECK_EQ(sel::error_detail_query(5, 0, 0x123).data, std::string("500123"));
    CHECK_THROWS(sel::error_detail_query(0x10, 0), std::out_of_range);
    const std::string head = "0A1" "00000001" "0000000C" "00000002" "FFFFFFFE" "00000000" "00000000" "00000000"
                             "00000000" "000000000000000";
    const sel::ErrorDetail e = sel::parse_error_detail(reply(sel::kErrorDetailQuery, head + "03ABC"));
    CHECK_EQ(e.error_number, 0xA1);
    CHECK_EQ(e.detail[0], 1);
    CHECK_EQ(e.detail[1], 12);
    CHECK_EQ(e.detail[2], 2);
    CHECK_EQ(e.detail[3], -2);  // a point number: negative at an interpolation point
    CHECK_EQ(e.message, std::string("ABC"));
    CHECK(sel::parse_error_detail(head + "00").message.empty());  // the newer models send no message
    CHECK_THROWS(sel::parse_error_detail(head + "03AB"), ReplyFormatError);
    CHECK_THROWS(sel::parse_error_detail(head + "03ABCD"), ReplyFormatError);
    CHECK_THROWS(sel::parse_error_detail(head), ReplyFormatError);
    CHECK_THROWS(sel::parse_error_detail(head.substr(1) + "00"), ReplyFormatError);
}

TEST(sel_messages_motion_requests) {
    const sel::MotionProfile profile{0.3, 0.25, 100.0};
    CHECK_EQ(sel::servo(0x0F, false).data, std::string("0F0"));
    CHECK_EQ(sel::home(0x03).data, std::string("03000000"));
    CHECK_EQ(sel::home(0x01, 20.0, 3.0).data, std::string("01014003"));
    CHECK_EQ(sel::move_absolute(0x05, profile, {12.345, -0.001}).data,
             std::string("05" "001E" "0019" "0064" "00003039" "FFFFFFFF"));
    CHECK_EQ(sel::move_relative(0x02, profile, {-5.0}).data, std::string("02" "001E" "0019" "0064" "FFFFEC78"));
    // jog (distance 0) negative in base coordinates; inching 0.5 mm positive in the work system
    CHECK_EQ(sel::jog(0x01, {}, 0.0, false).data, std::string("01" "000000000000" "00000000" "0"));
    CHECK_EQ(sel::jog(0x01, profile, 0.5, true, 1).data, std::string("01" "001E00190064" "000001F4" "3"));
    CHECK_EQ(sel::move_to_position(0xFF, {}, 0xFFF).data, std::string("FF" "000000000000" "FFF"));
    CHECK_EQ(sel::move_to_position_2(0x01, profile, 0x1234).data, std::string("01" "001E00190064" "1234"));
    CHECK_EQ(sel::stop(0xFF).data, std::string("FF00"));
    CHECK_EQ(sel::stop(0x01, true).data, std::string("0101"));
    CHECK_EQ(sel::change_speed(0x03, 50.0).data, std::string("030032"));
    CHECK(sel::alarm_reset().data.empty());
    CHECK_EQ(sel::move_to_position(0x01, {}, 1).id, sel::kMoveToPosition);
    CHECK_EQ(sel::move_to_position_2(0x01, {}, 1).id, sel::kMoveToPosition2);
    // full-scale fields
    CHECK_EQ(sel::move_absolute(0x01, {655.35, 655.35, 65535.0}, {2147483.647}).data,
             std::string("01" "FFFF" "FFFF" "FFFF" "7FFFFFFF"));
    CHECK_EQ(sel::move_absolute(0x01, {}, {-2147483.648}).data, std::string("01" "000000000000" "80000000"));
}

TEST(sel_messages_motion_refusals) {
    const sel::MotionProfile profile{0.3, 0.3, 100.0};
    const std::vector<double> one{1.0};
    const std::vector<double> two{1.0, 2.0};
    CHECK_THROWS(sel::move_absolute(0, profile, {}), std::invalid_argument);       // no axis
    CHECK_THROWS(sel::move_absolute(0x03, profile, one), std::invalid_argument);   // one value for two axes
    CHECK_THROWS(sel::move_relative(0x01, profile, two), std::invalid_argument);
    CHECK_THROWS(sel::move_absolute(0x01, profile, std::vector<double>{2147483.648}), std::out_of_range);
    CHECK_THROWS(sel::move_absolute(0x01, profile, std::vector<double>{-1e300}), std::out_of_range);
    CHECK_THROWS(sel::move_absolute(0x01, profile, std::vector<double>{std::nan("")}), std::invalid_argument);
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{inf, 0.0, 0.0}, one), std::invalid_argument);
    // a speed or acceleration that is not 0 but rounds to 0 would read as "the controller's parameter"
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{0.0, 0.0, 0.4}, one), std::invalid_argument);
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{0.004, 0.0, 0.0}, one), std::invalid_argument);
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{0.0, 0.0, -1.0}, one), std::out_of_range);
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{655.36, 0.0, 0.0}, one), std::out_of_range);
    CHECK_THROWS(sel::move_absolute(0x01, sel::MotionProfile{0.0, 0.0, 65536.0}, one), std::out_of_range);
    CHECK_THROWS(sel::home(0x01, 4096.0, 0.0), std::out_of_range);
    CHECK_THROWS(sel::home(0x01, 0.0, 0.2), std::invalid_argument);
    // an inching distance that rounds to 0 would jog until stopped
    CHECK_THROWS(sel::jog(0x01, profile, 0.0004, true), std::invalid_argument);
    CHECK_THROWS(sel::jog(0x01, profile, -1.0, true), std::out_of_range);
    CHECK_THROWS(sel::jog(0x01, profile, 1.0, true, 4), std::out_of_range);
    CHECK_THROWS(sel::move_to_position(0x01, profile, 0x1000), std::out_of_range);
    CHECK_THROWS(sel::move_to_position(0x01, profile, -1), std::out_of_range);
    CHECK_THROWS(sel::move_to_position_2(0x01, profile, 0x10000), std::out_of_range);
    CHECK_THROWS(sel::stop(0), std::invalid_argument);
    CHECK_THROWS(sel::change_speed(0x01, 0.0), std::invalid_argument);
    CHECK_THROWS(sel::change_speed(0x01, 0.4), std::invalid_argument);
}

TEST(sel_messages_programs) {
    CHECK_EQ(sel::program_control(sel::ProgramAction::run, 1).id, sel::kProgramRun);
    CHECK_EQ(sel::program_control(sel::ProgramAction::run, 0xFF).data, std::string("FF"));
    CHECK_EQ(sel::program_control(sel::ProgramAction::end, 0).id, sel::kProgramEnd);
    CHECK_EQ(sel::program_control(sel::ProgramAction::end, 0).data, std::string("00"));  // every running program
    CHECK_EQ(sel::program_control(sel::ProgramAction::pause, 0).id, sel::kProgramPause);
    CHECK_EQ(sel::program_control(sel::ProgramAction::step, 2).id, sel::kProgramStep);
    CHECK_EQ(sel::program_control(sel::ProgramAction::resume, 0).id, sel::kProgramResume);
    CHECK(sel::program_control(sel::ProgramAction::run, 1).kind == MessageKind::execute);
    CHECK_THROWS(sel::program_control(sel::ProgramAction::run, 0), std::out_of_range);
    CHECK_THROWS(sel::program_control(sel::ProgramAction::step, 0), std::out_of_range);
    CHECK_THROWS(sel::program_control(sel::ProgramAction::end, 0x100), std::out_of_range);
    CHECK_EQ(sel::program_status_query(0x0C).data, std::string("0C"));
    CHECK_THROWS(sel::program_status_query(0), std::out_of_range);
    const sel::ProgramStatus p = sel::parse_program_status(reply(sel::kProgramStatusQuery, "0C1002A0000000"), 0x0C);
    CHECK_EQ(p.program, 12);
    CHECK(p.started);
    CHECK_EQ(p.step, 0x2A);
    CHECK_EQ(p.error_code, 0);
    const sel::ProgramStatus err = sel::parse_program_status("0C00000A220031", 0x0C);
    CHECK(!err.started);
    CHECK_EQ(err.error_code, 0xA22);
    CHECK_EQ(err.error_step, 0x31);
    CHECK_THROWS(sel::parse_program_status("0D1002A0000000", 0x0C), ReplyFormatError);
    CHECK_THROWS(sel::parse_program_status("0C1002A000000", 0x0C), ReplyFormatError);
}

TEST(sel_messages_ports_and_flags) {
    CHECK_EQ(sel::input_query(0, 16).data, std::string("00000010"));
    CHECK_EQ(sel::output_query(300, 8).data, std::string("012C0008"));
    CHECK_THROWS(sel::input_query(0, 0), std::out_of_range);
    CHECK_THROWS(sel::input_query(0x10000, 8), std::out_of_range);
    // ports 0, 2 and 15 on (bit 0 = the lowest number)
    const sel::BitStates in = sel::parse_ports(sel::kInputQuery, reply(sel::kInputQuery, "0000001005" "80"), 0, 16);
    REQUIRE(in.on.size() == 16);
    CHECK(in.on[0]);
    CHECK(!in.on[1]);
    CHECK(in.on[2]);
    CHECK(in.on[15]);
    CHECK_EQ(in.start, 0);
    const sel::BitStates odd = sel::parse_ports(sel::kOutputQuery, "012C000AFF03", 300, 10);
    REQUIRE(odd.on.size() == 10);
    CHECK(odd.on[9]);
    CHECK_EQ(sel::parse_ports(sel::kInputQuery, "0000000805", 0, 16).on.size(), std::size_t{8});  // clipped
    CHECK_THROWS(sel::parse_ports(sel::kInputQuery, "0000001105" "80" "01", 0, 16), ReplyFormatError);  // more
    CHECK_THROWS(sel::parse_ports(sel::kInputQuery, "0008000805", 0, 8), ReplyFormatError);   // another start
    CHECK_THROWS(sel::parse_ports(sel::kInputQuery, "0000001005", 0, 16), ReplyFormatError);  // a byte short
    CHECK_THROWS(sel::parse_ports(sel::kInputQuery, "000000080500", 0, 8), ReplyFormatError);
    CHECK_THROWS(sel::parse_ports(sel::kFlagQuery, "0000000805", 0, 8), std::invalid_argument);
    CHECK_EQ(sel::set_output(300, true).data, std::string("012C1"));
    CHECK_EQ(sel::set_output(0x10, false).data, std::string("00100"));
    CHECK_THROWS(sel::set_output(0x10000, true), std::out_of_range);

    CHECK_EQ(sel::flag_query(0, 600, 16).data, std::string("00" "0258" "0010"));
    const sel::BitStates f = sel::parse_flags(reply(sel::kFlagQuery, "01" "0258" "0008" "81"), 1, 600, 8);
    CHECK_EQ(f.program, 1);
    CHECK_EQ(f.start, 600);
    CHECK(f.on[0]);
    CHECK(f.on[7]);
    CHECK(!f.on[1]);
    CHECK_THROWS(sel::parse_flags("00" "0258" "0008" "81", 1, 600, 8), ReplyFormatError);
    CHECK_EQ(sel::set_flag(0, 600, true).data, std::string("00" "0258" "1"));
    CHECK_THROWS(sel::set_flag(0x100, 600, true), std::out_of_range);
}

TEST(sel_messages_integer_and_string_variables) {
    CHECK_EQ(sel::variable_query(sel::VariableKind::integer, 0, 1, 3).id, sel::kIntegerVariableQuery);
    CHECK_EQ(sel::variable_query(sel::VariableKind::integer, 0, 1, 3).data, std::string("00" "001" "03"));
    CHECK_EQ(sel::variable_query(sel::VariableKind::string, 2, 0xFFF, 0xFF).data, std::string("02" "FFF" "FF"));
    CHECK_THROWS(sel::variable_query(sel::VariableKind::integer, 0, 1, 0), std::out_of_range);
    CHECK_THROWS(sel::variable_query(sel::VariableKind::integer, 0, 0x1000, 1), std::out_of_range);
    const sel::IntegerVariables iv = sel::parse_integer_variables(
        reply(sel::kIntegerVariableQuery, "00" "001" "03" "00000005" "FFFFFFFF" "80000000"), 0, 1, 3);
    REQUIRE(iv.values.size() == 3);
    CHECK_EQ(iv.values[0], 5);
    CHECK_EQ(iv.values[1], -1);
    CHECK_EQ(iv.values[2], std::numeric_limits<std::int32_t>::min());
    CHECK_THROWS(sel::parse_integer_variables("00" "001" "02" "00000005", 0, 1, 3), ReplyFormatError);
    CHECK_THROWS(sel::parse_integer_variables("00" "002" "01" "00000005", 0, 1, 3), ReplyFormatError);
    CHECK_THROWS(sel::parse_integer_variables("00" "001" "04" "00000005" "00000005" "00000005" "00000005", 0, 1, 3),
                 ReplyFormatError);
    const sel::Request set = sel::set_integer_variables(0, 1, {5, -1});
    CHECK_EQ(set.id, sel::kSetIntegerVariables);
    CHECK_EQ(set.data, std::string("00" "001" "02" "00000005" "FFFFFFFF"));
    CHECK_EQ(sel::parse_variable_write(sel::kSetIntegerVariables, reply(sel::kSetIntegerVariables, "0000102"), 0, 1, 2),
             2);
    CHECK_THROWS(sel::parse_variable_write(sel::kSetIntegerVariables, "0000103", 0, 1, 2), ReplyFormatError);
    CHECK_THROWS(sel::parse_variable_write(sel::kSetStringVariables, "0000102", 0, 1, 2), std::invalid_argument);
    CHECK_THROWS(sel::set_integer_variables(0, 1, {}), std::invalid_argument);
    CHECK_THROWS(sel::set_integer_variables(0, 1, std::vector<std::int32_t>(0x100, 0)), std::out_of_range);

    const sel::StringVariables sv =
        sel::parse_string_variables(reply(sel::kStringVariableQuery, "05" "010" "02" "4869"), 5, 0x10, 2);
    CHECK_EQ(sv.bytes, std::string("Hi"));
    CHECK_THROWS(sel::parse_string_variables("05" "010" "02" "486", 5, 0x10, 2), ReplyFormatError);
    CHECK_EQ(sel::set_string_variables(5, 0x10, std::string("Hi\0", 3)).data, std::string("05" "010" "03" "486900"));
    sel::parse_no_data(sel::kSetStringVariables, reply(sel::kSetStringVariables, ""));
}

TEST(sel_messages_real_variables_both_word_orders) {
    // the manual's example bit pattern 0123456789ABCDEFh in both orders
    const double v = from_bits(0x0123456789ABCDEFull);
    CHECK_EQ(sel::set_real_variables(0, 1, {v}).data, std::string("00" "001" "01" "0123456789ABCDEF"));
    CHECK_EQ(sel::set_real_variables(0, 1, {v}, sel::RealWordOrder::low_word_first).data,
             std::string("00" "001" "01" "89ABCDEF01234567"));
    const sel::RealVariables normal =
        sel::parse_real_variables(reply(sel::kRealVariableQuery, "00" "001" "02" "0123456789ABCDEF" "C004000000000000"),
                                  0, 1, 2);
    REQUIRE(normal.values.size() == 2);
    CHECK_EQ(to_bits(normal.values[0]), 0x0123456789ABCDEFull);
    CHECK_EQ(normal.values[1], -2.5);
    const sel::RealVariables low =
        sel::parse_real_variables("00" "001" "01" "89ABCDEF01234567", 0, 1, 1, sel::RealWordOrder::low_word_first);
    CHECK_EQ(to_bits(low.values[0]), 0x0123456789ABCDEFull);
    CHECK_THROWS(sel::parse_real_variables("00" "001" "01" "0123456789ABCDE", 0, 1, 1), ReplyFormatError);
    CHECK_THROWS(sel::set_real_variables(0, 1, {std::nan("")}), std::invalid_argument);
    CHECK_EQ(sel::parse_variable_write(sel::kSetRealVariables, "0000101", 0, 1, 1), 1);
}

TEST(sel_messages_position_table_reads) {
    CHECK(sel::position_count_query().data.empty());
    CHECK_EQ(sel::parse_position_count(sel::kPositionCountQuery, reply(sel::kPositionCountQuery, "0FA")), 250);
    CHECK_EQ(sel::parse_position_count(sel::kPositionCountQuery2, reply(sel::kPositionCountQuery2, "7530")), 30000);
    CHECK_THROWS(sel::parse_position_count(sel::kPositionCountQuery, "0FA0"), ReplyFormatError);
    CHECK_THROWS(sel::parse_position_count(sel::kPositionCountQuery2, "0FA"), ReplyFormatError);
    CHECK_THROWS(sel::parse_position_count(sel::kPositionQuery, "0FA"), std::invalid_argument);

    // two defined records from head 1: no. 1 on axes 1-2, no. 5 on axis 1 with every value from the parameters
    const std::string data = "002" "001" "03" "001E" "001E" "0064" "00001388" "FFFFEC78"
                             "005" "01" "0000" "0000" "0000" "00000000";
    const std::vector<sel::PositionRecord> rows =
        sel::parse_positions(sel::kPositionQuery, reply(sel::kPositionQuery, data), 1, 5);
    REQUIRE(rows.size() == 2);
    CHECK_EQ(rows[0].number, 1);
    CHECK_EQ(static_cast<int>(rows[0].data.axes), 0x03);
    CHECK_NEAR(rows[0].data.profile.accel, 0.3, 1e-12);
    CHECK_NEAR(rows[0].data.profile.decel, 0.3, 1e-12);
    CHECK_NEAR(rows[0].data.profile.speed, 100.0, 1e-12);
    REQUIRE(rows[0].data.positions.size() == 2);
    CHECK_NEAR(rows[0].data.positions[0], 5.0, 1e-12);
    CHECK_NEAR(rows[0].data.positions[1], -5.0, 1e-12);
    CHECK_EQ(rows[1].number, 5);
    REQUIRE(rows[1].data.positions.size() == 1);
    CHECK_NEAR(rows[1].data.positions[0], 0.0, 1e-12);
    CHECK(sel::parse_positions(sel::kPositionQuery, "000", 1, 5).empty());
    CHECK_THROWS(sel::parse_positions(sel::kPositionQuery, data, 2, 5), ReplyFormatError);   // no. 1 before head 2
    CHECK_THROWS(sel::parse_positions(sel::kPositionQuery, data, 1, 1), ReplyFormatError);   // 2 records for 1
    CHECK_THROWS(sel::parse_positions(sel::kPositionQuery, data.substr(0, data.size() - 8), 1, 5), ReplyFormatError);
    CHECK_THROWS(sel::parse_positions(sel::kPositionQuery, data + "0", 1, 5), ReplyFormatError);
    CHECK_THROWS(sel::parse_positions(sel::kPositionQuery, "003" + data.substr(3), 1, 5), ReplyFormatError);

    CHECK_EQ(sel::position_query_2(0x1000, 2).data, std::string("1000" "0002"));
    CHECK_EQ(sel::position_query_2(0x1000, 2).id, sel::kPositionQuery2);
    const std::string wide_data = "0001" "1000" "04" "0000" "0000" "0032" "000186A0";
    const std::vector<sel::PositionRecord> wide =
        sel::parse_positions(sel::kPositionQuery2, reply(sel::kPositionQuery2, wide_data), 0x1000, 2);
    REQUIRE(wide.size() == 1);
    CHECK_EQ(wide[0].number, 0x1000);
    CHECK_EQ(static_cast<int>(wide[0].data.axes), 0x04);
    CHECK_NEAR(wide[0].data.positions[0], 100.0, 1e-12);
    CHECK_THROWS(sel::position_query(0x1000, 1), std::out_of_range);
    CHECK_THROWS(sel::position_query(1, 0), std::out_of_range);
    CHECK_THROWS(sel::position_query_2(0x10000, 1), std::out_of_range);
}

TEST(sel_messages_position_table_writes) {
    const sel::PositionData two_axes{0x03, {0.3, 0.3, 100.0}, {5.0, -5.0}};
    const sel::PositionData one_axis{0x01, {}, {0.0}};
    const sel::Request range = sel::position_range_write(1, {two_axes, one_axis});
    CHECK_EQ(range.id, sel::kPositionRangeWrite);
    CHECK(range.kind == MessageKind::execute);
    CHECK_EQ(range.data, std::string("001" "002" "03" "001E001E0064" "00001388" "FFFFEC78" "01" "000000000000"
                                     "00000000"));
    const sel::Request change = sel::position_change_write({{7, two_axes}, {9, one_axis}});
    CHECK_EQ(change.data, std::string("002" "007" "03" "001E001E0064" "00001388" "FFFFEC78" "009" "01" "000000000000"
                                      "00000000"));
    CHECK_EQ(sel::position_range_write_2(0x1000, {one_axis}).data, std::string("1000" "0001" "01" "000000000000"
                                                                                 "00000000"));
    CHECK_EQ(sel::position_change_write_2({{0x1234, one_axis}}).data, std::string("0001" "1234" "01" "000000000000"
                                                                                    "00000000"));
    CHECK_EQ(sel::position_change_write_2({{0x1234, one_axis}}).id, sel::kPositionChangeWrite2);
    CHECK_EQ(sel::position_clear(1, 0x10).data, std::string("001" "010"));
    CHECK_EQ(sel::position_clear_2(0x1000, 1).data, std::string("1000" "0001"));
    CHECK_EQ(sel::position_clear_2(0x1000, 1).id, sel::kPositionClear2);
    sel::parse_no_data(sel::kPositionClear, reply(sel::kPositionClear, ""));

    CHECK_EQ(sel::parse_position_write(sel::kPositionRangeWrite, reply(sel::kPositionRangeWrite, "001002"), 1, 2), 2);
    CHECK_EQ(sel::parse_position_write(sel::kPositionChangeWrite, reply(sel::kPositionChangeWrite, "002"), 0, 2), 2);
    CHECK_EQ(sel::parse_position_write(sel::kPositionRangeWrite2, "10000001", 0x1000, 1), 1);
    CHECK_EQ(sel::parse_position_write(sel::kPositionChangeWrite2, "0001", 0, 1), 1);
    CHECK_EQ(sel::parse_position_write(sel::kPositionRangeWrite, "001001", 1, 2), 1);  // fewer: the caller sees it
    CHECK_THROWS(sel::parse_position_write(sel::kPositionRangeWrite, "002002", 1, 2), ReplyFormatError);
    CHECK_THROWS(sel::parse_position_write(sel::kPositionRangeWrite, "001003", 1, 2), ReplyFormatError);
    CHECK_THROWS(sel::parse_position_write(sel::kPositionChangeWrite, "0002", 0, 2), ReplyFormatError);
    CHECK_THROWS(sel::parse_position_write(sel::kPositionClear, "", 0, 2), std::invalid_argument);

    const sel::PositionData wrong{0x03, {}, {1.0}};
    const std::vector<sel::PositionData> wrong_rows{wrong};
    CHECK_THROWS(sel::position_range_write(1, wrong_rows), std::invalid_argument);
    CHECK_THROWS(sel::position_range_write(1, std::vector<sel::PositionData>{}), std::invalid_argument);
    const std::vector<sel::PositionData> rows2{one_axis, one_axis};
    CHECK_THROWS(sel::position_range_write(0xFFF, rows2), std::out_of_range);  // the second row is beyond FFFh
    CHECK_EQ(sel::position_range_write(0xFFE, rows2).data.substr(0, 6), std::string("FFE002"));
    const std::vector<sel::PositionRecord> beyond{{0x1000, one_axis}};
    CHECK_THROWS(sel::position_change_write(beyond), std::out_of_range);
    CHECK_THROWS(sel::position_clear(0xFFF, 2), std::out_of_range);
    CHECK_THROWS(sel::position_clear(1, 0), std::out_of_range);
}
