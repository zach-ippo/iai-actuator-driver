// Private to the core (internal: not installed, not part of the API yet): the phase-1 messages of the SEL controllers'
// serial protocol, on top of the frames of sel_frame.hpp. One builder per command or query (the message id and its
// fixed-width data fields, units converted here), one parser per reply (its data fields into a typed struct).
//
// Field layouts follow the serial Format B manual's tables (DOC). Units: positions in mm (deg on a rotary axis) as
// signed 0.001 steps, acceleration and deceleration in G as 0.01 G steps, speeds in mm/s; 0 in a speed or
// acceleration field means "the controller's parameter" (or the stored position's own value), so a value that is not
// 0 but rounds to 0 is refused. An axis pattern selects up to 8 axes, bit 0 = axis 1 (UNVERIFIED: the manual never
// says it in words), and per-axis fields follow in ascending axis order. Bit order inside a port or flag byte (bit 0 =
// the lowest number) is UNVERIFIED too. Everything here is UNVERIFIED on a controller: no SEL controller has been
// tested.
//
// Builders throw std::invalid_argument or std::out_of_range for what cannot be encoded (nothing is sent then).
// Parsers take a reply's data (Frame::data of an accepted normal reply, sel_frame.hpp decode_reply) and throw
// ReplyFormatError for a reply that does not match its layout: wrong length, a field that is not hex, or an echoed
// field (start number, program, axis pattern) that differs from the request.
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace iai_actuator_driver {
namespace sel {

// ---- message ids (phase 1) --------------------------------------------------------------------------------------
inline constexpr int kTestCall = 0x200;
inline constexpr int kVersionQuery = 0x201;
inline constexpr int kSettingQuery = 0x202;  // the controller-setting inquiry: id 202 + fixed "090"
inline constexpr int kPositionCountQuery = 0x208;
inline constexpr int kPositionQuery = 0x209;
inline constexpr int kInputQuery = 0x20B;
inline constexpr int kOutputQuery = 0x20C;
inline constexpr int kFlagQuery = 0x20D;
inline constexpr int kIntegerVariableQuery = 0x20E;
inline constexpr int kRealVariableQuery = 0x20F;
inline constexpr int kStringVariableQuery = 0x210;
inline constexpr int kAxisStatusQuery = 0x212;
inline constexpr int kProgramStatusQuery = 0x213;
inline constexpr int kSystemStatusQuery = 0x215;
inline constexpr int kErrorDetailQuery = 0x216;
inline constexpr int kPositionCountQuery2 = 0x21E;
inline constexpr int kPositionQuery2 = 0x21F;
inline constexpr int kServo = 0x232;
inline constexpr int kHome = 0x233;
inline constexpr int kMoveAbsolute = 0x234;
inline constexpr int kMoveRelative = 0x235;
inline constexpr int kJog = 0x236;
inline constexpr int kMoveToPosition = 0x237;
inline constexpr int kStop = 0x238;
inline constexpr int kPositionRangeWrite = 0x244;
inline constexpr int kPositionChangeWrite = 0x245;
inline constexpr int kPositionClear = 0x246;
inline constexpr int kSetOutput = 0x24A;
inline constexpr int kSetFlag = 0x24B;
inline constexpr int kSetIntegerVariables = 0x24C;
inline constexpr int kSetRealVariables = 0x24D;
inline constexpr int kSetStringVariables = 0x24E;
inline constexpr int kAlarmReset = 0x252;
inline constexpr int kProgramRun = 0x253;
inline constexpr int kProgramEnd = 0x254;
inline constexpr int kProgramPause = 0x255;
inline constexpr int kProgramStep = 0x256;
inline constexpr int kProgramResume = 0x257;
inline constexpr int kSoftwareReset = 0x25B;
inline constexpr int kChangeSpeed = 0x262;
inline constexpr int kMoveToPosition2 = 0x26B;
inline constexpr int kPositionRangeWrite2 = 0x26C;
inline constexpr int kPositionChangeWrite2 = 0x26D;
inline constexpr int kPositionClear2 = 0x26E;

/// Setting 185: bit 3 = the position output operation feature is on (TTA, MSEL). Plain position writes then clear
/// the written positions' output data (DOC).
inline constexpr int kSettingPositionOutput = 185;
/// Setting 189: bit 0 = the linear-axis work/tool coordinate system feature is on (TTA, MSEL) (DOC).
inline constexpr int kSettingCoordinateSystem = 189;

/// A query reads; an execute command changes data or moves axes. The controller answers an execute command only on
/// a channel enabled for execution (silence otherwise), and a silent execute may still have been acted on (DOC).
enum class MessageKind { query, execute };

/// A command's message id and data fields, ready for encode_command (sel_frame.hpp).
struct Request {
    int id = 0;
    std::string data;
    MessageKind kind = MessageKind::query;
    bool reply_expected = true;  // false only for the software reset (25BH): the controller restarts silently (DOC)
};

/// The command frame of `request` for `station` (encode_command).
std::string encode_request(const Request& request, int station, bool with_checksum = true);

/// A reply that does not match its message's layout. Nothing in it is used.
class ReplyFormatError : public std::runtime_error {
public:
    ReplyFormatError(int message_id, const std::string& what);
    int message_id() const noexcept { return message_id_; }

private:
    int message_id_;
};

/// A reply that carries no data (most execute commands): anything else is refused.
void parse_no_data(int message_id, std::string_view data);

// ---- axes --------------------------------------------------------------------------------------------------------
using AxisPattern = std::uint8_t;

/// The pattern bit of `axis` (1..8; std::out_of_range otherwise).
AxisPattern axis_bit(int axis);
/// The axes (1..8) of `pattern`, ascending.
std::vector<int> axes_of(AxisPattern pattern);
/// How many axes `pattern` selects.
int axis_count(AxisPattern pattern) noexcept;

/// Acceleration, deceleration (G) and speed (mm/s) of a move or a stored position; 0 = the controller's parameter
/// (for a move to a stored position: the position's own value first).
struct MotionProfile {
    double accel = 0.0;  // G, 0.01 G steps, 0..655.35
    double decel = 0.0;  // G
    double speed = 0.0;  // mm/s, whole steps, 0..65535
};

// ---- identification -------------------------------------------------------------------------------------------------
/// 200H: `text` must be exactly 10 printable characters; the reply echoes them.
Request test_call(std::string_view text);
/// The echoed text: refused unless it is `sent`.
std::string parse_test_call(std::string_view data, std::string_view sent);

/// 201H. Unit type: 0 main CPU application, 1 main CPU core, 2 driver CPU, 3 mounted SIO; device number from 0.
Request version_query(int unit_type = 0, int device = 0);
struct VersionInfo {
    int unit_type = 0;
    int device = 0;
    int model_code = 0;  // e.g. B9h, BDh, BFh on a TTA (DOC)
    int unit_code = 0;
    int version = 0;     // the 4-digit version field, raw
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
};
VersionInfo parse_version(std::string_view data, int unit_type, int device);

/// The controller-setting inquiry (id 202 + "090" + setting number + "001"; DOC, not on RSEL).
Request setting_query(int setting);
std::uint32_t parse_setting(std::string_view data);

// ---- status ------------------------------------------------------------------------------------------------------
Request system_status_query();  // 215H
struct SystemStatus {
    int mode = 0;            // 0 undetermined, 1 AUTO, 2 MANUAL, 3 slave update, 4 core update
    int critical_error = 0;  // the critical-level system error number (0: none)
    int latest_error = 0;    // the latest system error number
    std::array<std::uint8_t, 4> bytes{};  // system status bytes 1..4

    bool auto_mode() const noexcept { return mode == 1; }
    bool mode_switch_manual() const noexcept { return bytes[0] & 0x01; }
    bool emergency_stop() const noexcept { return bytes[0] & 0x08; }
    bool safety_gate_open() const noexcept { return bytes[0] & 0x04; }
    bool power_error() const noexcept { return bytes[0] & 0x10; }
    bool battery_low() const noexcept { return bytes[0] & 0x20; }
    bool battery_error() const noexcept { return bytes[0] & 0x40; }
    bool flash_writing() const noexcept { return bytes[1] & 0x01; }
    bool servo_interlock() const noexcept { return bytes[1] & 0x04; }
    bool io_interlock() const noexcept { return bytes[1] & 0x08; }
    bool restart_wait() const noexcept { return bytes[1] & 0x10; }
    bool program_running() const noexcept { return bytes[1] & 0x20; }
    bool drive_source_cutoff() const noexcept { return bytes[2] & 0x01; }
    bool operating() const noexcept { return bytes[2] & 0x02; }
    bool system_ready() const noexcept { return bytes[2] & 0x04; }
    bool positioner_mode() const noexcept { return bytes[2] & 0x10; }
};
SystemStatus parse_system_status(std::string_view data);

/// 212H for the axes of `axes` (not 0); FFh asks for every present axis.
Request axis_status_query(AxisPattern axes);
struct AxisStatus {
    int axis = 0;             // 1..8
    std::uint8_t status = 0;  // the axis status byte
    std::uint8_t sensor = 0;  // bit 0 creep, bit 1 overrun, bit 2 home sensor
    int error_code = 0;       // the axis error code (0: none)
    std::uint8_t encoder = 0; // the encoder status byte
    double position = 0.0;    // mm (deg on a rotary axis)

    /// Bit 0: a task owns the axis (moving, servo starting or stopping, paused).
    bool in_use() const noexcept { return status & 0x01; }
    /// Bits 1-2: 0 not homed, 1 homing, 2 homed.
    int home_state() const noexcept { return (status >> 1) & 0x03; }
    bool homed() const noexcept { return home_state() == 2; }
    bool servo_on() const noexcept { return status & 0x08; }
    /// Bit 4: the last operation command completed normally (meaningful once in_use() is false).
    bool completed() const noexcept { return status & 0x10; }
    bool push_error() const noexcept { return status & 0x20; }
};
struct AxisStatusReply {
    AxisPattern present = 0;  // the asked-for axes that have a driver
    std::vector<AxisStatus> axes;
};
/// Refused when the reply names an axis that was not asked for.
AxisStatusReply parse_axis_status(std::string_view data, AxisPattern requested);

/// 216H. Type 1: 0 system error (type 2: 0 critical, 1 latest), 1 axis error (type 2: axis), 2 program error (type 2:
/// program), 3 error-list record (type 2: record from 1), 5 error-list record by the 3-digit record number.
Request error_detail_query(int type1, int type2, int record = 0);
struct ErrorDetail {
    int error_number = 0;
    /// Detail information 1..8; below the system-down level: program, step, axis, point (negative at an
    /// interpolation point).
    std::array<std::int32_t, 8> detail{};
    std::string message;                   // empty on the newer models
};
ErrorDetail parse_error_detail(std::string_view data);

// ---- servo, home, motion -----------------------------------------------------------------------------------------
Request servo(AxisPattern axes, bool on);                                          // 232H
/// 233H; speeds in mm/s (0..4095, 0 = parameter).
Request home(AxisPattern axes, double search_speed = 0.0, double creep_speed = 0.0);
/// 234H: `positions` holds one target per selected axis, ascending.
Request move_absolute(AxisPattern axes, const MotionProfile& profile, const std::vector<double>& positions);
/// 235H: `offsets` holds one signed distance per selected axis, ascending.
Request move_relative(AxisPattern axes, const MotionProfile& profile, const std::vector<double>& offsets);
/// 236H: `distance` 0 jogs until a stop (238H), otherwise inches by it (mm, not negative); `coordinate_system` 0 base,
/// 1 work, 2 tool, 3 each axis (SCARA and coordinate-system unit axes only).
Request jog(AxisPattern axes, const MotionProfile& profile, double distance, bool positive, int coordinate_system = 0);
/// 237H (position 0..FFFh) and 26BH (0..FFFFh): `axes` is ANDed with the stored position's own pattern.
Request move_to_position(AxisPattern axes, const MotionProfile& profile, int position);
Request move_to_position_2(AxisPattern axes, const MotionProfile& profile, int position);
/// 238H; `cancel_held_outputs` also cancels the outputs held by an interlock.
Request stop(AxisPattern axes, bool cancel_held_outputs = false);
Request change_speed(AxisPattern axes, double speed);  // 262H
Request alarm_reset();     // 252H
Request software_reset();  // 25BH: no reply

// ---- programs ----------------------------------------------------------------------------------------------------
enum class ProgramAction { run = kProgramRun, end = kProgramEnd, pause = kProgramPause, step = kProgramStep,
                           resume = kProgramResume };
/// 253H..257H for program 1..FFh; end, pause and resume also take 0 = every running program.
Request program_control(ProgramAction action, int program);
Request program_status_query(int program);  // 213H, program 1..FFh
struct ProgramStatus {
    int program = 0;
    bool started = false;
    int step = 0;          // the executing step (0: none)
    int error_code = 0;
    int error_step = 0;
};
ProgramStatus parse_program_status(std::string_view data, int program);

// ---- I/O ports, flags, variables ---------------------------------------------------------------------------------
/// 20BH / 20CH: `count` ports from `start` (a category's head + a multiple of 8).
Request input_query(int start, int count);
Request output_query(int start, int count);
/// 20DH: `count` flags from `start` of `program` (0: the global flags).
Request flag_query(int program, int start, int count);
/// Ports or flags read, one entry per number from `start`.
struct BitStates {
    int program = 0;  // flags only
    int start = 0;
    std::vector<bool> on;
};
/// The reply to input_query / output_query (`message_id` says which). The controller may return fewer than asked
/// (its buffer), never more.
BitStates parse_ports(int message_id, std::string_view data, int start, int count);
BitStates parse_flags(std::string_view data, int program, int start, int count);
Request set_output(int port, bool on);             // 24AH
Request set_flag(int program, int flag, bool on);  // 24BH

enum class VariableKind { integer = kIntegerVariableQuery, real = kRealVariableQuery, string = kStringVariableQuery };
/// Word order of a real variable: IEEE double as 16 hex digits, or (XSEL-J/K V0.41 and older, Other parameter 46
/// bits 0-3 = 0) its low 4 bytes first (DOC).
enum class RealWordOrder { normal, low_word_first };
/// 20EH / 20FH / 210H: `count` (1..FFh) variables from `start` (0..FFFh) of `program` (0: global). A string
/// variable is one character.
Request variable_query(VariableKind kind, int program, int start, int count);
struct IntegerVariables {
    int program = 0;
    int start = 0;
    std::vector<std::int32_t> values;
};
struct RealVariables {
    int program = 0;
    int start = 0;
    std::vector<double> values;
};
struct StringVariables {
    int program = 0;
    int start = 0;
    std::string bytes;  // one byte per string variable
};
IntegerVariables parse_integer_variables(std::string_view data, int program, int start, int count);
RealVariables parse_real_variables(std::string_view data, int program, int start, int count,
                                   RealWordOrder order = RealWordOrder::normal);
StringVariables parse_string_variables(std::string_view data, int program, int start, int count);
/// 24CH / 24DH / 24EH: values from `start` (1..FFh of them).
Request set_integer_variables(int program, int start, const std::vector<std::int32_t>& values);
Request set_real_variables(int program, int start, const std::vector<double>& values,
                           RealWordOrder order = RealWordOrder::normal);
Request set_string_variables(int program, int start, std::string_view bytes);
/// The reply to set_integer_variables / set_real_variables: the count the controller changed (at most `count`).
/// 24EH replies with no data (parse_no_data).
int parse_variable_write(int message_id, std::string_view data, int program, int start, int count);

// ---- position table ----------------------------------------------------------------------------------------------
/// One stored position: its axes, profile and one coordinate per axis of the pattern (ascending; mm or deg).
struct PositionData {
    AxisPattern axes = 0;
    MotionProfile profile;
    std::vector<double> positions;
};
struct PositionRecord {
    int number = 0;
    PositionData data;
};

/// 208H (count 0..FFFh) and 21EH (0..FFFFh): how many positions are defined.
Request position_count_query();
Request position_count_query_2();
int parse_position_count(int message_id, std::string_view data);

/// 209H (3-digit numbers) and 21FH (4-digit): `count` records from `head`. The reply lists the defined ones only,
/// clipped to the controller's buffer.
Request position_query(int head, int count);
Request position_query_2(int head, int count);
/// The reply to position_query / position_query_2 (`message_id` says which): refused when a record lies before
/// `head`, or there are more records than `count`.
std::vector<PositionRecord> parse_positions(int message_id, std::string_view data, int head, int count);

/// 244H / 26CH: `rows` written to consecutive numbers from `start`.
Request position_range_write(int start, const std::vector<PositionData>& rows);
Request position_range_write_2(int start, const std::vector<PositionData>& rows);
/// 245H / 26DH: each record to its own number.
Request position_change_write(const std::vector<PositionRecord>& rows);
Request position_change_write_2(const std::vector<PositionRecord>& rows);
/// The count written (at most `count`); for a range write the echoed start must be `start` (ignored for 245H/26DH).
int parse_position_write(int message_id, std::string_view data, int start, int count);
/// 246H / 26EH: clear `count` positions from `start` (comments stay). No reply data.
Request position_clear(int start, int count);
Request position_clear_2(int start, int count);

}  // namespace sel
}  // namespace iai_actuator_driver
