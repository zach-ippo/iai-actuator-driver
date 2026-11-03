// One row of the position table (0x1000 + 16 x n, 16 words, non-volatile: an FC10 write reaches non-volatile memory at
// once and the unit uses it at once - VERIFIED in the Python edition). Port of ec.PositionEntry (C# AxisTypes.cs);
// implemented by the model unit (src/position.cpp).
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace iai_actuator_driver {

/// A stored row, in the axis's units (mm or deg).
struct PositionEntry {
    double position_mm = 0.0;   ///< PCMD
    double band_mm = 0.10;      ///< INP (push rows: the push band)
    double speed_mm_s = 50.0;   ///< VCMD
    double zone_plus_mm = 0.0;  ///< ZNMP
    double zone_minus_mm = 0.0; ///< ZNLP
    double accel_g = 0.30;      ///< ACMD
    double decel_g = 0.30;      ///< DCMD
    int push_power = 0;         ///< PPOW counts (255 = 100 %)
    int load_threshold = 0;     ///< LPOW
    int control_flags = 0;      ///< CTLF (push / direction / role bits)
    std::optional<int> checksum;  ///< +0xF as read (nullopt for an entry built in code)

    bool is_push() const;
    bool incremental() const;
    /// The stored checksum matches the data (nullopt without one).
    std::optional<bool> checksum_ok() const;

    /// Decode 15 or 16 words (std::invalid_argument for fewer than 15).
    static PositionEntry from_registers(const std::vector<std::uint16_t>& words);
    /// The 15 data words +0x0..+0xE (std::invalid_argument for a value that does not fit its field).
    std::vector<std::uint16_t> to_registers() const;
    /// to_registers() plus the checksum word: the 16 words a write sends.
    std::vector<std::uint16_t> to_registers_with_checksum() const;

    bool operator==(const PositionEntry& other) const;
    bool operator!=(const PositionEntry& other) const { return !(*this == other); }
};

}  // namespace iai_actuator_driver
