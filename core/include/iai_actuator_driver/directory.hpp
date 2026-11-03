// The unit's self-description: the index table at 0x0000 (header + one 8-word record per class) and the class blocks
// it points to, with their item-ID maps. Port of the directory parts of info.py (C# UnitDirectory.cs); implemented by
// the model unit (src/directory.cpp). READ-ONLY. Addresses of the class blocks differ per model, so every class item is
// found by class + item ID through this directory, never by a fixed address (VERIFIED on the units tested).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/registers.hpp"

namespace iai_actuator_driver {

/// Index-table header (0x0000): MODL, DQNT (words per record), LQNT (records), BFRD, BFTD.
struct IndexHeader {
    int model_code = 0;
    int words_per_record = 0;
    int record_count = 0;
    int rx_buffer = 0;
    int tx_buffer = 0;

    /// Needs 5 words (std::invalid_argument).
    static IndexHeader from_words(const std::vector<std::uint16_t>& words);
    /// 8 words per record and 1..64 records.
    bool layout_ok() const { return words_per_record == 8 && record_count > 0 && record_count <= 64; }
};

/// One 8-word category record: CI MD DQ LQ DA IA BA MS.
struct IndexCategory {
    int category_id = 0;     ///< CI
    int model_code = 0;      ///< MD
    int member_count = 0;    ///< DQ
    int list_count = 0;      ///< LQ (rows: 768 position-table rows on an EC)
    int data_address = 0;    ///< DA
    int id_list_address = 0; ///< IA (0 = no item-ID map)
    int coil_base = 0;       ///< BA
    int member_size = 0;     ///< MS (0 = mixed sizes)

    reg::BlockExtent extent() const;
    std::string name() const;
    /// Needs 8 words from `offset` (std::invalid_argument).
    static IndexCategory from_words(const std::vector<std::uint16_t>& words, std::size_t offset = 0);
};

/// One item of a class block, found by its ID.
struct ClassItem {
    int class_id = 0;
    std::optional<int> item_id;  ///< nullopt for a block without an ID map
    int entry = 0;               ///< list entry (0 for single-entry classes)
    int index = 0;               ///< member index inside the entry
    int address = 0;             ///< register address of the item
    int size = 1;                ///< words (1 or 2)
    std::uint32_t raw = 0;       ///< value, high word first

    std::int32_t signed_value() const;  ///< raw as int16 / int32 by size
    std::string hex() const;            ///< 4 or 8 upper-case hex digits
    std::string key() const;            ///< "8.15", "C.1[1]" (class in hex above 9; "[entry]" when not 0)
    double scaled() const;              ///< signed_value() x 0.01, rounded to 0.01
};

/// A class block read through the directory.
struct ClassBlock {
    int class_id = 0;
    int address = 0;
    int words = 0;
    int member_size = 0;
    int entries = 0;
    std::optional<int> id_map_address;
    std::vector<ClassItem> items;

    /// The item with this ID (and entry), nullptr when absent.
    const ClassItem* item(int item_id, int entry = 0) const;
};

/// The index table of one unit.
class UnitDirectory {
public:
    /// Most words read for one class.
    static constexpr int kMaxClassWords = 1024;

    UnitDirectory() = default;
    UnitDirectory(std::optional<IndexHeader> header, std::vector<IndexCategory> records);

    /// Read the header and every record. A unit without a directory (silent / zero header) gives an empty directory.
    static UnitDirectory read(UnitReader& reader);
    /// Decode header + records from one block of words starting at 0x0000.
    static UnitDirectory from_words(const std::vector<std::uint16_t>& words);
    /// Decode consecutive 8-word records.
    static std::vector<IndexCategory> parse_records(const std::vector<std::uint16_t>& words);

    const std::optional<IndexHeader>& header() const { return header_; }
    const std::vector<IndexCategory>& records() const { return records_; }
    bool empty() const { return records_.empty(); }

    /// The record of a class, nullptr when absent.
    const IndexCategory* find(int class_id) const;
    /// The record of a class; std::out_of_range when absent.
    const IndexCategory& get(int class_id) const;
    /// Words of a block that are safe to read (never past the next block).
    int readable_words(const IndexCategory& record) const;

    /// Raw words of a class block from `offset` (count: nullopt = readable_words).
    std::vector<std::uint16_t> read_block(UnitReader& reader, int class_id, int offset = 0,
                                          std::optional<int> count = std::nullopt) const;
    /// A class block decoded by its ID map (items of 1 or 2 words).
    ClassBlock read_class(UnitReader& reader, int class_id, int max_words = kMaxClassWords) const;
    /// The class-1 support words by name (SUPT, DIAG, MVPT, ...), nullopt when the unit has no class 1.
    std::optional<std::map<std::string, std::uint32_t>> read_support_words(UnitReader& reader) const;
    /// ACCL (identification +0x12, u32), nullopt when unreadable.
    std::optional<std::uint32_t> read_accl(UnitReader& reader) const;

private:
    std::optional<IndexHeader> header_;
    std::vector<IndexCategory> records_;
};

}  // namespace iai_actuator_driver
