// The SEL protocol's frames (sel_frame.hpp).
#include "sel_frame.hpp"

#include <stdexcept>

namespace iai_actuator_driver {
namespace sel {

namespace {

constexpr char kDigits[] = "0123456789ABCDEF";

int digit_value(char c) noexcept {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool printable(char c) noexcept { return c >= 0x20 && c <= 0x7E; }

void check_width(int width) {
    if (width < 1 || width > 8) throw std::out_of_range("SEL hex field: width " + std::to_string(width) + " not 1..8");
}

/// start + station + id (or error code) + data + checksum or "@@" + CR LF.
std::string encode(char start, int station, int id, std::string_view data, bool with_checksum) {
    if (station < 0 || station > 0xFF) {
        throw std::out_of_range("SEL station " + std::to_string(station) + " not 0..FFh");
    }
    if (id < 0 || id > 0xFFF) {
        throw std::out_of_range("SEL message id / error code " + std::to_string(id) + " not 0..FFFh");
    }
    for (char c : data) {
        if (!printable(c)) throw std::invalid_argument("SEL frame data: a byte outside 20h..7Eh");
    }
    std::string frame;
    frame.reserve(kMinFrameSize + data.size());
    frame += start;
    frame += hex(static_cast<std::uint32_t>(station), 2);
    frame += hex(static_cast<std::uint32_t>(id), 3);
    frame += data;
    frame += with_checksum ? hex(checksum(frame), 2) : std::string(kNoChecksum);
    frame += kTerminator;
    return frame;
}

}  // namespace

std::uint8_t checksum(std::string_view bytes) noexcept {
    unsigned sum = 0;
    for (char c : bytes) sum += static_cast<unsigned char>(c);
    return static_cast<std::uint8_t>(sum & 0xFFu);
}

std::string hex(std::uint32_t value, int width) {
    check_width(width);
    if (width < 8 && value >> (4 * width)) {
        throw std::out_of_range("SEL hex field: " + std::to_string(value) + " does not fit " + std::to_string(width) +
                                " digits");
    }
    std::string text(static_cast<std::size_t>(width), '0');
    for (int i = width - 1; i >= 0; --i, value >>= 4) text[static_cast<std::size_t>(i)] = kDigits[value & 0xFu];
    return text;
}

std::string hex_signed(std::int32_t value, int width) {
    check_width(width);
    if (width < 8) {
        const std::int64_t half = std::int64_t{1} << (4 * width - 1);
        if (value < -half || value >= half) {
            throw std::out_of_range("SEL signed hex field: " + std::to_string(value) + " does not fit " +
                                    std::to_string(width) + " digits");
        }
    }
    const std::uint32_t bits = static_cast<std::uint32_t>(value);  // two's complement (C++17: modulo 2^32)
    return hex(width < 8 ? bits & ((std::uint32_t{1} << (4 * width)) - 1u) : bits, width);
}

std::optional<std::uint32_t> parse_hex(std::string_view text) noexcept {
    if (text.empty() || text.size() > 8) return std::nullopt;
    std::uint32_t value = 0;
    for (char c : text) {
        const int d = digit_value(c);
        if (d < 0) return std::nullopt;
        value = (value << 4) | static_cast<std::uint32_t>(d);
    }
    return value;
}

std::optional<std::int32_t> parse_hex_signed(std::string_view text) noexcept {
    const std::optional<std::uint32_t> raw = parse_hex(text);
    if (!raw) return std::nullopt;
    const int bits = static_cast<int>(text.size()) * 4;
    const std::int64_t span = std::int64_t{1} << bits;
    std::int64_t value = *raw;
    if (value >= span / 2) value -= span;
    return static_cast<std::int32_t>(value);
}

std::string encode_command(int station, int message_id, std::string_view data, bool with_checksum) {
    return encode(kCommandStart, station, message_id, data, with_checksum);
}

std::string encode_reply(int station, int message_id, std::string_view data) {
    return encode(kReplyStart, station, message_id, data, true);
}

std::string encode_error_reply(int station, int error_code) {
    return encode(kErrorStart, station, error_code, {}, true);
}

const char* to_string(FrameStatus status) noexcept {
    switch (status) {
        case FrameStatus::ok: return "ok";
        case FrameStatus::too_short: return "too_short";
        case FrameStatus::bad_start: return "bad_start";
        case FrameStatus::bad_terminator: return "bad_terminator";
        case FrameStatus::bad_format: return "bad_format";
        case FrameStatus::bad_hex: return "bad_hex";
        case FrameStatus::bad_checksum: return "bad_checksum";
        case FrameStatus::wrong_station: return "wrong_station";
        case FrameStatus::wrong_id: return "wrong_id";
    }
    return "unknown";
}

DecodeResult decode(std::string_view bytes) {
    DecodeResult result;
    if (bytes.size() < kMinFrameSize) return result;  // too_short
    Frame& f = result.frame;
    switch (bytes.front()) {
        case kCommandStart: f.kind = FrameKind::command; break;
        case kReplyStart: f.kind = FrameKind::reply; break;
        case kErrorStart: f.kind = FrameKind::error_reply; break;
        default: result.status = FrameStatus::bad_start; return result;
    }
    const std::size_t body_size = bytes.size() - kTerminator.size();
    if (bytes.substr(body_size) != kTerminator) {
        result.status = FrameStatus::bad_terminator;
        return result;
    }
    const std::string_view body = bytes.substr(0, body_size);  // start .. checksum
    for (char c : body) {
        if (c == '\r' || c == '\n') {
            result.status = FrameStatus::bad_terminator;
            return result;
        }
        if (!printable(c)) {
            result.status = FrameStatus::bad_format;
            return result;
        }
    }
    const std::optional<std::uint32_t> station = parse_hex(body.substr(1, 2));
    const std::optional<std::uint32_t> id = parse_hex(body.substr(3, 3));
    if (!station || !id) {
        result.status = FrameStatus::bad_hex;
        return result;
    }
    const std::string_view summed = body.substr(0, body.size() - 2);
    const std::string_view sc = body.substr(body.size() - 2);
    if (sc == kNoChecksum) {
        if (f.kind != FrameKind::command) {
            result.status = FrameStatus::bad_checksum;
            return result;
        }
        f.checksum_skipped = true;
    } else {
        const std::optional<std::uint32_t> value = parse_hex(sc);
        if (!value) {
            result.status = FrameStatus::bad_hex;
            return result;
        }
        if (*value != checksum(summed)) {
            result.status = FrameStatus::bad_checksum;
            return result;
        }
    }
    f.station = static_cast<int>(*station);
    f.id = static_cast<int>(*id);
    f.data = std::string(summed.substr(6));
    if (f.kind == FrameKind::error_reply && !f.data.empty()) {
        result.status = FrameStatus::bad_format;
        return result;
    }
    result.status = FrameStatus::ok;
    return result;
}

DecodeResult decode_reply(std::string_view bytes, int station, int message_id) {
    DecodeResult result = decode(bytes);
    if (!result.ok()) return result;
    if (result.frame.kind == FrameKind::command) {
        result.status = FrameStatus::bad_start;
    } else if (result.frame.station != station) {
        result.status = FrameStatus::wrong_station;
    } else if (result.frame.kind == FrameKind::reply && result.frame.id != message_id) {
        result.status = FrameStatus::wrong_id;
    }
    return result;
}

}  // namespace sel
}  // namespace iai_actuator_driver
