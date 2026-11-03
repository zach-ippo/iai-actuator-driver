// Modbus RTU framing helpers (header-only: the bus, the simulator and the tests all need them).
// CRC-16/MODBUS (polynomial 0xA001 reflected, initial 0xFFFF), appended low byte first (DOC ME0162).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace iai_actuator_driver {

/// CRC-16/MODBUS of `size` bytes.
inline std::uint16_t crc16(const std::uint8_t* data, std::size_t size) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001u) : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

/// CRC-16/MODBUS of a byte vector.
inline std::uint16_t crc16(const std::vector<std::uint8_t>& data) noexcept { return crc16(data.data(), data.size()); }

/// Append the CRC of `frame` to it, low byte first.
inline void append_crc(std::vector<std::uint8_t>& frame) {
    const std::uint16_t crc = crc16(frame);
    frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<std::uint8_t>(crc >> 8));
}

/// slave + pdu + CRC.
inline std::vector<std::uint8_t> build_frame(int slave, const std::vector<std::uint8_t>& pdu) {
    std::vector<std::uint8_t> frame;
    frame.reserve(pdu.size() + 3);
    frame.push_back(static_cast<std::uint8_t>(slave));
    frame.insert(frame.end(), pdu.begin(), pdu.end());
    append_crc(frame);
    return frame;
}

/// A complete frame (at least 4 bytes) whose last two bytes are the CRC of the rest.
inline bool check_crc(const std::uint8_t* frame, std::size_t size) noexcept {
    if (size < 4) return false;
    const std::uint16_t crc = crc16(frame, size - 2);
    return frame[size - 2] == (crc & 0xFF) && frame[size - 1] == (crc >> 8);
}

/// Vector overload of check_crc.
inline bool check_crc(const std::vector<std::uint8_t>& frame) noexcept { return check_crc(frame.data(), frame.size()); }

}  // namespace iai_actuator_driver
