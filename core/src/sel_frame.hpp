// Private to the core (internal: not installed, not part of the API yet): the frames of the SEL controllers' serial
// protocol (IAI Protocol B / Format B: XSEL, SSEL, ASEL, PSEL, MSEL, the TTA table-top robots). ASCII, one command
// then one reply, half duplex (DOC):
//
//   command        '!' station(2) id(3)   data... checksum(2) CR LF
//   normal reply   '#' station(2) id(3)   data... checksum(2) CR LF
//   error reply    '&' station(2) code(3)         checksum(2) CR LF      (10 bytes)
//
// Every number is fixed-width big-endian ASCII hex. The checksum is the low byte of the plain byte sum from the start
// character through the last data byte, as 2 hex digits; "@@" in its place turns the check off for that command
// (DOC). Hex digits are written upper case and read in either case, signed fields are two's complement over their
// width: both UNVERIFIED (the manuals show no lower-case or negative example). Everything here is UNVERIFIED on a
// controller: no SEL controller has been tested.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace iai_actuator_driver {
namespace sel {

inline constexpr char kCommandStart = '!';
inline constexpr char kReplyStart = '#';
inline constexpr char kErrorStart = '&';
/// In place of the checksum of a command: the controller skips the check (DOC).
inline constexpr std::string_view kNoChecksum = "@@";
inline constexpr std::string_view kTerminator = "\r\n";
/// The shortest frame: start, station, id or code, checksum, CR LF (an error reply is exactly this long).
inline constexpr std::size_t kMinFrameSize = 10;

/// The checksum of `bytes`: the low byte of their plain sum.
std::uint8_t checksum(std::string_view bytes) noexcept;

/// `value` as `width` upper-case hex digits (1..8). Throws std::out_of_range when it does not fit.
std::string hex(std::uint32_t value, int width);
/// A signed field: `value` as `width` hex digits (1..8), two's complement over width * 4 bits. Throws
/// std::out_of_range when it does not fit (an 8-digit field takes every int32_t).
std::string hex_signed(std::int32_t value, int width = 8);
/// A field of 1..8 hex digits, either case (nullopt: empty, too long or not hex).
std::optional<std::uint32_t> parse_hex(std::string_view text) noexcept;
/// A signed field of 1..8 hex digits, sign-extended from its width (nullopt as parse_hex).
std::optional<std::int32_t> parse_hex_signed(std::string_view text) noexcept;

/// A command frame: '!' + station (0..FFh) + message id (0..FFFh) + `data` + checksum (or "@@" when `with_checksum`
/// is false) + CR LF. `data` is the message's fields, already encoded; it must be printable ASCII (20h..7Eh).
/// Throws std::out_of_range for a station or id out of range, std::invalid_argument for other data bytes.
std::string encode_command(int station, int message_id, std::string_view data, bool with_checksum = true);
/// A normal reply frame ('#'), as encode_command (always with its checksum). For the simulator and the tests.
std::string encode_reply(int station, int message_id, std::string_view data);
/// An error reply frame ('&' + station + error code (0..FFFh) + checksum + CR LF). For the simulator and the tests.
std::string encode_error_reply(int station, int error_code);

enum class FrameKind { command, reply, error_reply };

/// Why a frame was not accepted. The controller stays silent on a command it cannot accept (DOC); the host drops such
/// a reply and keeps reading until its deadline.
enum class FrameStatus {
    ok,
    too_short,       // fewer than kMinFrameSize bytes
    bad_start,       // not '!', '#' or '&' (or not the kind asked for)
    bad_terminator,  // does not end with CR LF, or a CR or LF before the end
    bad_format,      // a byte outside 20h..7Eh, or an error reply with data
    bad_hex,         // the station, the id / error code or the checksum is not hex
    bad_checksum,    // the checksum does not match (or "@@" where it is not allowed)
    wrong_station,   // a valid frame from another station
    wrong_id,        // a valid normal reply to another message id
};

/// The name of a status ("bad_checksum").
const char* to_string(FrameStatus status) noexcept;

/// A decoded frame.
struct Frame {
    FrameKind kind = FrameKind::command;
    int station = 0;
    int id = 0;                    // the message id, or the error code of an error reply
    std::string data;              // the bytes between the id and the checksum, still encoded
    bool checksum_skipped = false; // a command with "@@" in place of its checksum
};

struct DecodeResult {
    FrameStatus status = FrameStatus::too_short;
    Frame frame;  // filled when status is ok (and for wrong_station / wrong_id: the frame that was dropped)

    bool ok() const noexcept { return status == FrameStatus::ok; }
};

/// Any one frame of `bytes` (exactly one, CR LF included). "@@" in place of the checksum is accepted on a command
/// only (checksum_skipped), as the controller does.
DecodeResult decode(std::string_view bytes);

/// The host's check of a reply to a command sent to `station` with `message_id` (DOC): a normal or an error reply,
/// its checksum correct, the same station, and for a normal reply the same message id (an error reply carries its
/// error code instead). A command frame is bad_start.
DecodeResult decode_reply(std::string_view bytes, int station, int message_id);

}  // namespace sel
}  // namespace iai_actuator_driver
