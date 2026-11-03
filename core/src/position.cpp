// One row of the position table (0x1000 + 16 x n, 16 words). Port of ec.PositionEntry (C# AxisTypes.cs). No I/O.
#include "iai_actuator_driver/position.hpp"

#include <limits>
#include <stdexcept>
#include <string>

#include "iai_actuator_driver/registers.hpp"

namespace iai_actuator_driver {

namespace {

void require_field(const char* name, int value) {
    if (value < 0 || value > 0xFFFF) throw std::invalid_argument(std::string(name) + " must be an int in 0..65535");
}

std::int32_t to_i32(std::int64_t counts, const char* what) {
    if (counts < std::numeric_limits<std::int32_t>::min() || counts > std::numeric_limits<std::int32_t>::max())
        throw std::invalid_argument(std::string(what) + " does not fit a signed 32-bit field");
    return static_cast<std::int32_t>(counts);
}

std::uint32_t to_u32(std::int64_t counts, const char* what) {
    if (counts < 0 || counts > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
        throw std::invalid_argument(std::string(what) + " does not fit an unsigned 32-bit field");
    return static_cast<std::uint32_t>(counts);
}

void add(std::vector<std::uint16_t>& words, std::pair<std::uint16_t, std::uint16_t> pair) {
    words.push_back(pair.first);
    words.push_back(pair.second);
}

}  // namespace

bool PositionEntry::is_push() const { return (control_flags & reg::kCtlfPush) != 0; }

bool PositionEntry::incremental() const { return (control_flags & reg::kCtlfInc) != 0; }

std::optional<bool> PositionEntry::checksum_ok() const {
    if (!checksum) return std::nullopt;
    return *checksum == static_cast<int>(reg::position_checksum(to_registers()));
}

PositionEntry PositionEntry::from_registers(const std::vector<std::uint16_t>& words) {
    if (words.size() < static_cast<std::size_t>(reg::kPositionWordsWritable))
        throw std::invalid_argument("need at least " + std::to_string(reg::kPositionWordsWritable) + " words");
    PositionEntry e;
    e.position_mm = reg::from_counts(reg::from_i32_words(words[0], words[1]), reg::kPositionScale);
    e.band_mm = reg::from_counts(reg::from_u32_words(words[2], words[3]), reg::kPositionScale);
    e.speed_mm_s = reg::from_counts(reg::from_u32_words(words[4], words[5]), reg::kVelocityScale);
    e.zone_plus_mm = reg::from_counts(reg::from_i32_words(words[6], words[7]), reg::kPositionScale);
    e.zone_minus_mm = reg::from_counts(reg::from_i32_words(words[8], words[9]), reg::kPositionScale);
    e.accel_g = reg::from_counts(words[10], reg::kAccelScale);
    e.decel_g = reg::from_counts(words[11], reg::kAccelScale);
    e.push_power = words[12];
    e.load_threshold = words[13];
    e.control_flags = words[14];
    if (words.size() > 15) e.checksum = words[15];
    return e;
}

std::vector<std::uint16_t> PositionEntry::to_registers() const {
    require_field("push_power", push_power);
    require_field("load_threshold", load_threshold);
    require_field("control_flags", control_flags);
    const std::int64_t accel = reg::to_counts(accel_g, reg::kAccelScale);
    const std::int64_t decel = reg::to_counts(decel_g, reg::kAccelScale);
    if (accel < 0 || accel > 0xFFFF || decel < 0 || decel > 0xFFFF) throw std::invalid_argument("accel/decel out of range");
    std::vector<std::uint16_t> words;
    words.reserve(static_cast<std::size_t>(reg::kPositionWordsWritable));
    add(words, reg::to_i32_words(to_i32(reg::to_counts(position_mm), "position")));
    add(words, reg::to_u32_words(to_u32(reg::to_counts(band_mm), "band")));
    add(words, reg::to_u32_words(to_u32(reg::to_counts(speed_mm_s, reg::kVelocityScale), "speed")));
    add(words, reg::to_i32_words(to_i32(reg::to_counts(zone_plus_mm), "zone +")));
    add(words, reg::to_i32_words(to_i32(reg::to_counts(zone_minus_mm), "zone -")));
    words.push_back(static_cast<std::uint16_t>(accel));
    words.push_back(static_cast<std::uint16_t>(decel));
    words.push_back(static_cast<std::uint16_t>(push_power));
    words.push_back(static_cast<std::uint16_t>(load_threshold));
    words.push_back(static_cast<std::uint16_t>(control_flags));
    return words;
}

std::vector<std::uint16_t> PositionEntry::to_registers_with_checksum() const {
    std::vector<std::uint16_t> words = to_registers();
    words.push_back(reg::position_checksum(words));
    return words;
}

bool PositionEntry::operator==(const PositionEntry& o) const {
    return position_mm == o.position_mm && band_mm == o.band_mm && speed_mm_s == o.speed_mm_s &&
           zone_plus_mm == o.zone_plus_mm && zone_minus_mm == o.zone_minus_mm && accel_g == o.accel_g &&
           decel_g == o.decel_g && push_power == o.push_power && load_threshold == o.load_threshold &&
           control_flags == o.control_flags && checksum == o.checksum;
}

}  // namespace iai_actuator_driver
