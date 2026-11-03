// The SEL protocol's phase-1 messages (sel_messages.hpp).
#include "sel_messages.hpp"

#include <cmath>
#include <cstring>
#include <optional>
#include <utility>

#include "sel_frame.hpp"

namespace iai_actuator_driver {
namespace sel {

namespace {

std::string id_name(int message_id) { return hex(static_cast<std::uint32_t>(message_id & 0xFFF), 3) + "H"; }

void check_range(const char* name, long long value, long long low, long long high) {
    if (value < low || value > high) {
        throw std::out_of_range(std::string("SEL ") + name + " " + std::to_string(value) + " not " +
                                std::to_string(low) + ".." + std::to_string(high));
    }
}

std::string field(const char* name, long long value, int width, long long low = 0) {
    const long long high = (1LL << (4 * width)) - 1;
    check_range(name, value, low, high);
    return hex(static_cast<std::uint32_t>(value), width);
}

std::string pattern_field(AxisPattern axes) {
    if (axes == 0) throw std::invalid_argument("SEL axis pattern: no axis selected");
    return hex(axes, 2);
}

void check_finite(const char* name, double value) {
    if (!std::isfinite(value)) throw std::invalid_argument(std::string("SEL ") + name + ": not a finite number");
}

/// `value` in steps of 1 / `scale`, 0..`high`. A value that is not 0 but rounds to 0 is refused when 0 means "the
/// controller's parameter" (or, for an inching distance, "jog until stopped").
std::uint32_t steps(const char* name, double value, double scale, std::uint32_t high, bool zero_is_special = true) {
    check_finite(name, value);
    if (value < 0.0) throw std::out_of_range(std::string("SEL ") + name + ": negative");
    const double scaled = value * scale;
    if (scaled > static_cast<double>(high) + 0.5) {
        throw std::out_of_range(std::string("SEL ") + name + " " + std::to_string(value) + ": above the field's range");
    }
    const auto rounded = static_cast<std::uint32_t>(std::llround(scaled));
    if (rounded > high) throw std::out_of_range(std::string("SEL ") + name + ": above the field's range");
    if (zero_is_special && rounded == 0 && value != 0.0) {
        throw std::invalid_argument(std::string("SEL ") + name + " " + std::to_string(value) +
                                    ": rounds to 0, which the controller reads as its own setting");
    }
    return rounded;
}

/// A coordinate in mm (deg) as signed 0.001 steps.
std::int32_t position_steps(const char* name, double value) {
    check_finite(name, value);
    const double scaled = value * 1000.0;
    if (scaled < -2147483648.0 - 0.5 || scaled > 2147483647.0 + 0.5) {
        throw std::out_of_range(std::string("SEL ") + name + " " + std::to_string(value) +
                                ": out of the field's range");
    }
    const long long rounded = std::llround(scaled);
    check_range(name, rounded, -2147483648LL, 2147483647LL);
    return static_cast<std::int32_t>(rounded);
}

std::string profile_fields(const MotionProfile& p) {
    return hex(steps("acceleration", p.accel, 100.0, 0xFFFF), 4) +
           hex(steps("deceleration", p.decel, 100.0, 0xFFFF), 4) + hex(steps("speed", p.speed, 1.0, 0xFFFF), 4);
}

std::string position_fields(AxisPattern axes, const std::vector<double>& values, const char* name) {
    if (values.size() != static_cast<std::size_t>(axis_count(axes))) {
        throw std::invalid_argument(std::string("SEL ") + name + ": " + std::to_string(values.size()) +
                                    " values for " + std::to_string(axis_count(axes)) + " axes");
    }
    std::string text;
    for (double v : values) text += hex_signed(position_steps(name, v));
    return text;
}

Request make(int id, std::string data, MessageKind kind) {
    Request r;
    r.id = id;
    r.data = std::move(data);
    r.kind = kind;
    return r;
}

/// Reads a reply's fields in order; any mismatch is a ReplyFormatError.
class Reader {
public:
    Reader(int message_id, std::string_view data) : id_(message_id), data_(data) {}

    [[noreturn]] void fail(const std::string& what) const { throw ReplyFormatError(id_, what); }

    std::string_view take(std::size_t width, const char* name) {
        if (data_.size() - pos_ < width) fail(std::string("too short for the ") + name + " field");
        const std::string_view text = data_.substr(pos_, width);
        pos_ += width;
        return text;
    }
    std::uint32_t unsigned_field(int width, const char* name) {
        const std::optional<std::uint32_t> v = parse_hex(take(static_cast<std::size_t>(width), name));
        if (!v) fail(std::string("the ") + name + " field is not hex");
        return *v;
    }
    int number(int width, const char* name) { return static_cast<int>(unsigned_field(width, name)); }
    std::int32_t signed_field(int width, const char* name) {
        const std::optional<std::int32_t> v = parse_hex_signed(take(static_cast<std::size_t>(width), name));
        if (!v) fail(std::string("the ") + name + " field is not hex");
        return *v;
    }
    /// An echoed field: refused unless it is `expected`.
    void echo(int width, const char* name, long long expected) {
        const std::uint32_t v = unsigned_field(width, name);
        if (static_cast<long long>(v) != expected) {
            fail(std::string("the ") + name + " " + std::to_string(v) + " is not the " + std::to_string(expected) +
                 " asked for");
        }
    }
    /// A count field: at most `limit`.
    int count(int width, const char* name, long long limit) {
        const int n = number(width, name);
        if (n > limit) {
            fail(std::string("the ") + name + " " + std::to_string(n) + " exceeds the " + std::to_string(limit) +
                 " asked for");
        }
        return n;
    }
    double position(const char* name) { return signed_field(8, name) / 1000.0; }
    MotionProfile profile() {
        MotionProfile p;
        p.accel = unsigned_field(4, "acceleration") / 100.0;
        p.decel = unsigned_field(4, "deceleration") / 100.0;
        p.speed = unsigned_field(4, "speed");
        return p;
    }
    void end() const {
        if (pos_ != data_.size()) fail(std::to_string(data_.size() - pos_) + " bytes beyond its fields");
    }

private:
    int id_;
    std::string_view data_;
    std::size_t pos_ = 0;
};

std::vector<bool> read_bits(Reader& in, int count) {
    std::vector<bool> on(static_cast<std::size_t>(count));
    for (int byte_index = 0; byte_index * 8 < count; ++byte_index) {
        const std::uint32_t byte = in.unsigned_field(2, "data byte");
        for (int bit = 0; bit < 8 && byte_index * 8 + bit < count; ++bit) {
            on[static_cast<std::size_t>(byte_index * 8 + bit)] = (byte >> bit) & 1u;
        }
    }
    return on;
}

std::string real_field(double value, RealWordOrder order) {
    check_finite("real variable", value);
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    const std::string high = hex(static_cast<std::uint32_t>(bits >> 32), 8);
    const std::string low = hex(static_cast<std::uint32_t>(bits & 0xFFFFFFFFu), 8);
    return order == RealWordOrder::normal ? high + low : low + high;
}

double read_real(Reader& in, RealWordOrder order) {
    const std::uint32_t first = in.unsigned_field(8, "real variable");
    const std::uint32_t second = in.unsigned_field(8, "real variable");
    const std::uint32_t high = order == RealWordOrder::normal ? first : second;
    const std::uint32_t low = order == RealWordOrder::normal ? second : first;
    const std::uint64_t bits = (std::uint64_t{high} << 32) | low;
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

std::string variable_head(int program, int start, std::size_t count) {
    if (count == 0) throw std::invalid_argument("SEL variable change: no values");
    check_range("variable count", static_cast<long long>(count), 1, 0xFF);
    return field("program", program, 2) + field("variable", start, 3) + hex(static_cast<std::uint32_t>(count), 2);
}

void variable_echo(Reader& in, int program, int start) {
    in.echo(2, "program", program);
    in.echo(3, "start variable", start);
}

/// The fields of one stored position after its number: pattern, profile, one coordinate per axis.
std::string position_data_fields(const PositionData& row) {
    return hex(row.axes, 2) + profile_fields(row.profile) + position_fields(row.axes, row.positions, "position");
}

PositionData read_position_data(Reader& in) {
    PositionData d;
    d.axes = static_cast<AxisPattern>(in.unsigned_field(2, "axis pattern"));
    d.profile = in.profile();
    for (int i = 0; i < axis_count(d.axes); ++i) d.positions.push_back(in.position("position"));
    return d;
}

int number_width(int message_id) {
    switch (message_id) {
        case kPositionCountQuery: case kPositionQuery: case kPositionRangeWrite: case kPositionChangeWrite:
        case kPositionClear: case kMoveToPosition:
            return 3;
        case kPositionCountQuery2: case kPositionQuery2: case kPositionRangeWrite2: case kPositionChangeWrite2:
        case kPositionClear2: case kMoveToPosition2:
            return 4;
        default:
            throw std::invalid_argument("SEL message " + id_name(message_id) + " is not a position-table message");
    }
}

long long number_max(int width) { return (1LL << (4 * width)) - 1; }

Request range_write(int id, int start, const std::vector<PositionData>& rows) {
    const int w = number_width(id);
    if (rows.empty()) throw std::invalid_argument("SEL position write: no rows");
    check_range("position count", static_cast<long long>(rows.size()), 1, number_max(w));
    std::string data = field("start position", start, w);
    check_range("last position", start + static_cast<long long>(rows.size()) - 1, 0, number_max(w));
    data += hex(static_cast<std::uint32_t>(rows.size()), w);
    for (const PositionData& row : rows) data += position_data_fields(row);
    return make(id, std::move(data), MessageKind::execute);
}

Request change_write(int id, const std::vector<PositionRecord>& rows) {
    const int w = number_width(id);
    if (rows.empty()) throw std::invalid_argument("SEL position write: no rows");
    check_range("position count", static_cast<long long>(rows.size()), 1, number_max(w));
    std::string data = hex(static_cast<std::uint32_t>(rows.size()), w);
    for (const PositionRecord& row : rows) data += field("position", row.number, w) + position_data_fields(row.data);
    return make(id, std::move(data), MessageKind::execute);
}

Request clear(int id, int start, int count) {
    const int w = number_width(id);
    std::string data = field("start position", start, w) + field("position count", count, w, 1);
    check_range("last position", static_cast<long long>(start) + count - 1, 0, number_max(w));
    return make(id, std::move(data), MessageKind::execute);
}

Request position_read(int id, int head, int count) {
    const int w = number_width(id);
    return make(id, field("head position", head, w) + field("record count", count, w, 1), MessageKind::query);
}

}  // namespace

std::string encode_request(const Request& request, int station, bool with_checksum) {
    return encode_command(station, request.id, request.data, with_checksum);
}

ReplyFormatError::ReplyFormatError(int message_id, const std::string& what)
    : std::runtime_error("SEL " + id_name(message_id) + " reply: " + what), message_id_(message_id) {}

void parse_no_data(int message_id, std::string_view data) { Reader(message_id, data).end(); }

// ---- axes --------------------------------------------------------------------------------------------------------
AxisPattern axis_bit(int axis) {
    check_range("axis", axis, 1, 8);
    return static_cast<AxisPattern>(1u << (axis - 1));
}

std::vector<int> axes_of(AxisPattern pattern) {
    std::vector<int> axes;
    for (int axis = 1; axis <= 8; ++axis) {
        if (pattern & (1u << (axis - 1))) axes.push_back(axis);
    }
    return axes;
}

int axis_count(AxisPattern pattern) noexcept {
    int n = 0;
    for (unsigned p = pattern; p; p &= p - 1) ++n;
    return n;
}

// ---- identification -------------------------------------------------------------------------------------------------
Request test_call(std::string_view text) {
    if (text.size() != 10) throw std::invalid_argument("SEL test call: the text must be 10 characters");
    for (char c : text) {
        if (c < 0x20 || c > 0x7E) throw std::invalid_argument("SEL test call: a character outside 20h..7Eh");
    }
    return make(kTestCall, std::string(text), MessageKind::query);
}

std::string parse_test_call(std::string_view data, std::string_view sent) {
    Reader in(kTestCall, data);
    const std::string_view echo = in.take(10, "string");
    in.end();
    if (echo != sent) in.fail("the echo is not the text sent");
    return std::string(echo);
}

Request version_query(int unit_type, int device) {
    return make(kVersionQuery, field("unit type", unit_type, 2) + field("device number", device, 1),
                MessageKind::query);
}

VersionInfo parse_version(std::string_view data, int unit_type, int device) {
    Reader in(kVersionQuery, data);
    VersionInfo v;
    in.echo(2, "unit type", unit_type);
    in.echo(1, "device number", device);
    v.unit_type = unit_type;
    v.device = device;
    v.model_code = in.number(2, "model code");
    v.unit_code = in.number(2, "unit code");
    v.version = in.number(4, "version");
    v.year = in.number(4, "year");
    v.month = in.number(2, "month");
    v.day = in.number(2, "day");
    v.hour = in.number(2, "hour");
    v.minute = in.number(2, "minute");
    v.second = in.number(2, "second");
    in.end();
    return v;
}

Request setting_query(int setting) {
    return make(kSettingQuery, "090" + field("setting number", setting, 3) + "001", MessageKind::query);
}

std::uint32_t parse_setting(std::string_view data) {
    Reader in(kSettingQuery, data);
    if (in.take(3, "fixed") != "090") in.fail("the fixed field is not 090");
    const std::uint32_t value = in.unsigned_field(8, "setting value");
    in.end();
    return value;
}

// ---- status ------------------------------------------------------------------------------------------------------
Request system_status_query() { return make(kSystemStatusQuery, {}, MessageKind::query); }

SystemStatus parse_system_status(std::string_view data) {
    Reader in(kSystemStatusQuery, data);
    SystemStatus s;
    s.mode = in.number(1, "system mode");
    s.critical_error = in.number(3, "critical error");
    s.latest_error = in.number(3, "latest error");
    for (auto& b : s.bytes) b = static_cast<std::uint8_t>(in.unsigned_field(2, "status byte"));
    in.end();
    return s;
}

Request axis_status_query(AxisPattern axes) { return make(kAxisStatusQuery, pattern_field(axes), MessageKind::query); }

AxisStatusReply parse_axis_status(std::string_view data, AxisPattern requested) {
    Reader in(kAxisStatusQuery, data);
    AxisStatusReply r;
    r.present = static_cast<AxisPattern>(in.unsigned_field(2, "axis pattern"));
    if (r.present & ~requested) in.fail("it names an axis that was not asked for");
    for (int axis : axes_of(r.present)) {
        AxisStatus a;
        a.axis = axis;
        a.status = static_cast<std::uint8_t>(in.unsigned_field(2, "axis status"));
        a.sensor = static_cast<std::uint8_t>(in.unsigned_field(1, "sensor input"));
        a.error_code = in.number(3, "axis error code");
        a.encoder = static_cast<std::uint8_t>(in.unsigned_field(2, "encoder status"));
        a.position = in.position("current position");
        r.axes.push_back(a);
    }
    in.end();
    return r;
}

Request error_detail_query(int type1, int type2, int record) {
    return make(kErrorDetailQuery, field("type 1", type1, 1) + field("type 2", type2, 2) + field("record", record, 3),
                MessageKind::query);
}

ErrorDetail parse_error_detail(std::string_view data) {
    Reader in(kErrorDetailQuery, data);
    ErrorDetail e;
    e.error_number = in.number(3, "error number");
    for (auto& d : e.detail) d = in.signed_field(8, "detail information");
    in.take(15, "reserved");  // seven fields reserved for system use
    const int length = in.number(2, "message byte count");
    e.message = std::string(in.take(static_cast<std::size_t>(length), "message"));
    in.end();
    return e;
}

// ---- servo, home, motion -----------------------------------------------------------------------------------------
Request servo(AxisPattern axes, bool on) {
    return make(kServo, pattern_field(axes) + (on ? "1" : "0"), MessageKind::execute);
}

Request home(AxisPattern axes, double search_speed, double creep_speed) {
    return make(kHome,
                pattern_field(axes) + hex(steps("search speed", search_speed, 1.0, 0xFFF), 3) +
                    hex(steps("creep speed", creep_speed, 1.0, 0xFFF), 3),
                MessageKind::execute);
}

Request move_absolute(AxisPattern axes, const MotionProfile& profile, const std::vector<double>& positions) {
    return make(kMoveAbsolute,
                pattern_field(axes) + profile_fields(profile) + position_fields(axes, positions, "target"),
                MessageKind::execute);
}

Request move_relative(AxisPattern axes, const MotionProfile& profile, const std::vector<double>& offsets) {
    return make(kMoveRelative, pattern_field(axes) + profile_fields(profile) + position_fields(axes, offsets, "offset"),
                MessageKind::execute);
}

Request jog(AxisPattern axes, const MotionProfile& profile, double distance, bool positive, int coordinate_system) {
    check_range("jog coordinate system", coordinate_system, 0, 3);
    const std::uint32_t inching = steps("inching distance", distance, 1000.0, 0x7FFFFFFF);
    const unsigned type = (positive ? 1u : 0u) | static_cast<unsigned>(coordinate_system) << 1;
    return make(kJog, pattern_field(axes) + profile_fields(profile) + hex(inching, 8) + hex(type, 1),
                MessageKind::execute);
}

Request move_to_position(AxisPattern axes, const MotionProfile& profile, int position) {
    return make(kMoveToPosition, pattern_field(axes) + profile_fields(profile) + field("position number", position, 3),
                MessageKind::execute);
}

Request move_to_position_2(AxisPattern axes, const MotionProfile& profile, int position) {
    return make(kMoveToPosition2, pattern_field(axes) + profile_fields(profile) + field("position number", position, 4),
                MessageKind::execute);
}

Request stop(AxisPattern axes, bool cancel_held_outputs) {
    return make(kStop, pattern_field(axes) + (cancel_held_outputs ? "01" : "00"), MessageKind::execute);
}

Request change_speed(AxisPattern axes, double speed) {
    check_finite("speed", speed);
    if (speed <= 0.0) throw std::invalid_argument("SEL speed change: the speed must be above 0");
    return make(kChangeSpeed, pattern_field(axes) + hex(steps("speed", speed, 1.0, 0xFFFF), 4), MessageKind::execute);
}

Request alarm_reset() { return make(kAlarmReset, {}, MessageKind::execute); }

Request software_reset() {
    Request r = make(kSoftwareReset, {}, MessageKind::execute);
    r.reply_expected = false;
    return r;
}

// ---- programs ----------------------------------------------------------------------------------------------------
Request program_control(ProgramAction action, int program) {
    const bool all_allowed = action == ProgramAction::end || action == ProgramAction::pause ||
                             action == ProgramAction::resume;
    switch (action) {
        case ProgramAction::run: case ProgramAction::end: case ProgramAction::pause: case ProgramAction::step:
        case ProgramAction::resume:
            break;
        default:
            throw std::invalid_argument("SEL program control: unknown action");
    }
    return make(static_cast<int>(action), field("program", program, 2, all_allowed ? 0 : 1), MessageKind::execute);
}

Request program_status_query(int program) {
    return make(kProgramStatusQuery, field("program", program, 2, 1), MessageKind::query);
}

ProgramStatus parse_program_status(std::string_view data, int program) {
    Reader in(kProgramStatusQuery, data);
    ProgramStatus p;
    in.echo(2, "program", program);
    p.program = program;
    p.started = in.unsigned_field(1, "status") & 0x1u;
    p.step = in.number(4, "executing step");
    p.error_code = in.number(3, "program error code");
    p.error_step = in.number(4, "error step");
    in.end();
    return p;
}

// ---- I/O ports, flags, variables ---------------------------------------------------------------------------------
Request input_query(int start, int count) {
    return make(kInputQuery, field("start port", start, 4) + field("port count", count, 4, 1), MessageKind::query);
}

Request output_query(int start, int count) {
    return make(kOutputQuery, field("start port", start, 4) + field("port count", count, 4, 1), MessageKind::query);
}

Request flag_query(int program, int start, int count) {
    return make(kFlagQuery,
                field("program", program, 2) + field("start flag", start, 4) + field("flag count", count, 4, 1),
                MessageKind::query);
}

BitStates parse_ports(int message_id, std::string_view data, int start, int count) {
    if (message_id != kInputQuery && message_id != kOutputQuery) {
        throw std::invalid_argument("SEL message " + id_name(message_id) + " is not a port query");
    }
    Reader in(message_id, data);
    BitStates s;
    in.echo(4, "start port", start);
    s.start = start;
    s.on = read_bits(in, in.count(4, "port count", count));
    in.end();
    return s;
}

BitStates parse_flags(std::string_view data, int program, int start, int count) {
    Reader in(kFlagQuery, data);
    BitStates s;
    in.echo(2, "program", program);
    in.echo(4, "start flag", start);
    s.program = program;
    s.start = start;
    s.on = read_bits(in, in.count(4, "flag count", count));
    in.end();
    return s;
}

Request set_output(int port, bool on) {
    return make(kSetOutput, field("output port", port, 4) + (on ? "1" : "0"), MessageKind::execute);
}

Request set_flag(int program, int flag, bool on) {
    return make(kSetFlag, field("program", program, 2) + field("flag", flag, 4) + (on ? "1" : "0"),
                MessageKind::execute);
}

Request variable_query(VariableKind kind, int program, int start, int count) {
    switch (kind) {
        case VariableKind::integer: case VariableKind::real: case VariableKind::string: break;
        default: throw std::invalid_argument("SEL variable query: unknown kind");
    }
    return make(static_cast<int>(kind),
                field("program", program, 2) + field("start variable", start, 3) + field("variable count", count, 2, 1),
                MessageKind::query);
}

IntegerVariables parse_integer_variables(std::string_view data, int program, int start, int count) {
    Reader in(kIntegerVariableQuery, data);
    IntegerVariables v;
    variable_echo(in, program, start);
    v.program = program;
    v.start = start;
    const int n = in.count(2, "variable count", count);
    for (int i = 0; i < n; ++i) v.values.push_back(in.signed_field(8, "integer variable"));
    in.end();
    return v;
}

RealVariables parse_real_variables(std::string_view data, int program, int start, int count, RealWordOrder order) {
    Reader in(kRealVariableQuery, data);
    RealVariables v;
    variable_echo(in, program, start);
    v.program = program;
    v.start = start;
    const int n = in.count(2, "variable count", count);
    for (int i = 0; i < n; ++i) v.values.push_back(read_real(in, order));
    in.end();
    return v;
}

StringVariables parse_string_variables(std::string_view data, int program, int start, int count) {
    Reader in(kStringVariableQuery, data);
    StringVariables v;
    variable_echo(in, program, start);
    v.program = program;
    v.start = start;
    const int n = in.count(2, "variable count", count);
    for (int i = 0; i < n; ++i) v.bytes += static_cast<char>(in.unsigned_field(2, "string variable"));
    in.end();
    return v;
}

Request set_integer_variables(int program, int start, const std::vector<std::int32_t>& values) {
    std::string data = variable_head(program, start, values.size());
    for (std::int32_t v : values) data += hex_signed(v);
    return make(kSetIntegerVariables, std::move(data), MessageKind::execute);
}

Request set_real_variables(int program, int start, const std::vector<double>& values, RealWordOrder order) {
    std::string data = variable_head(program, start, values.size());
    for (double v : values) data += real_field(v, order);
    return make(kSetRealVariables, std::move(data), MessageKind::execute);
}

Request set_string_variables(int program, int start, std::string_view bytes) {
    std::string data = variable_head(program, start, bytes.size());
    for (char c : bytes) data += hex(static_cast<unsigned char>(c), 2);
    return make(kSetStringVariables, std::move(data), MessageKind::execute);
}

int parse_variable_write(int message_id, std::string_view data, int program, int start, int count) {
    if (message_id != kSetIntegerVariables && message_id != kSetRealVariables) {
        throw std::invalid_argument("SEL message " + id_name(message_id) + " has no variable-write reply");
    }
    Reader in(message_id, data);
    variable_echo(in, program, start);
    const int n = in.count(2, "changed count", count);
    in.end();
    return n;
}

// ---- position table ----------------------------------------------------------------------------------------------
Request position_count_query() { return make(kPositionCountQuery, {}, MessageKind::query); }
Request position_count_query_2() { return make(kPositionCountQuery2, {}, MessageKind::query); }

int parse_position_count(int message_id, std::string_view data) {
    if (message_id != kPositionCountQuery && message_id != kPositionCountQuery2) {
        throw std::invalid_argument("SEL message " + id_name(message_id) + " is not a position-count query");
    }
    Reader in(message_id, data);
    const int n = in.number(number_width(message_id), "position count");
    in.end();
    return n;
}

Request position_query(int head, int count) { return position_read(kPositionQuery, head, count); }
Request position_query_2(int head, int count) { return position_read(kPositionQuery2, head, count); }

std::vector<PositionRecord> parse_positions(int message_id, std::string_view data, int head, int count) {
    if (message_id != kPositionQuery && message_id != kPositionQuery2) {
        throw std::invalid_argument("SEL message " + id_name(message_id) + " is not a position query");
    }
    const int w = number_width(message_id);
    Reader in(message_id, data);
    const int n = in.count(w, "record count", count);
    std::vector<PositionRecord> rows;
    for (int i = 0; i < n; ++i) {
        PositionRecord row;
        row.number = in.number(w, "position number");
        if (row.number < head) in.fail("position " + std::to_string(row.number) + " lies before the head asked for");
        row.data = read_position_data(in);
        rows.push_back(std::move(row));
    }
    in.end();
    return rows;
}

Request position_range_write(int start, const std::vector<PositionData>& rows) {
    return range_write(kPositionRangeWrite, start, rows);
}
Request position_range_write_2(int start, const std::vector<PositionData>& rows) {
    return range_write(kPositionRangeWrite2, start, rows);
}
Request position_change_write(const std::vector<PositionRecord>& rows) {
    return change_write(kPositionChangeWrite, rows);
}
Request position_change_write_2(const std::vector<PositionRecord>& rows) {
    return change_write(kPositionChangeWrite2, rows);
}

int parse_position_write(int message_id, std::string_view data, int start, int count) {
    const bool range = message_id == kPositionRangeWrite || message_id == kPositionRangeWrite2;
    if (!range && message_id != kPositionChangeWrite && message_id != kPositionChangeWrite2) {
        throw std::invalid_argument("SEL message " + id_name(message_id) + " is not a position write");
    }
    const int w = number_width(message_id);
    Reader in(message_id, data);
    if (range) in.echo(w, "start position", start);
    const int n = in.count(w, "written count", count);
    in.end();
    return n;
}

Request position_clear(int start, int count) { return clear(kPositionClear, start, count); }
Request position_clear_2(int start, int count) { return clear(kPositionClear2, start, count); }

}  // namespace sel
}  // namespace iai_actuator_driver
