// The unit's self-description: the index table at 0x0000 and the class blocks it points to, with their item-ID maps.
// Port of the directory parts of info.py (C# UnitDirectory.cs). READ-ONLY: only FC03 goes through the UnitReader, and
// every read stays inside a block the unit itself declares (the unit does not reject unmapped addresses, it returns
// 0000, so these bounds are the only guard against reading past a block).
#include "iai_actuator_driver/directory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "util.hpp"

namespace iai_actuator_driver {

// ------------------------------------------------------------------------------------------ IndexHeader / IndexCategory

IndexHeader IndexHeader::from_words(const std::vector<std::uint16_t>& words) {
    if (words.size() < 5) throw std::invalid_argument("need 5 header words");
    IndexHeader header;
    header.model_code = words[0];
    header.words_per_record = words[1];
    header.record_count = words[2];
    header.rx_buffer = words[3];
    header.tx_buffer = words[4];
    return header;
}

reg::BlockExtent IndexCategory::extent() const {
    return reg::category_extent(category_id, member_count, list_count, member_size);
}

std::string IndexCategory::name() const { return reg::category_name(category_id); }

IndexCategory IndexCategory::from_words(const std::vector<std::uint16_t>& words, std::size_t offset) {
    if (offset + 8 > words.size()) throw std::invalid_argument("need 8 record words");
    IndexCategory rec;
    rec.category_id = words[offset];
    rec.model_code = words[offset + 1];
    rec.member_count = words[offset + 2];
    rec.list_count = words[offset + 3];
    rec.data_address = words[offset + 4];
    rec.id_list_address = words[offset + 5];
    rec.coil_base = words[offset + 6];
    rec.member_size = words[offset + 7];
    return rec;
}

// ------------------------------------------------------------------------------------------ ClassItem / ClassBlock

std::int32_t ClassItem::signed_value() const {
    if (size == 1) return static_cast<std::int16_t>(raw & 0xFFFF);
    return static_cast<std::int32_t>(raw);
}

std::string ClassItem::hex() const { return util::format("%0*X", 4 * size, static_cast<unsigned>(raw)); }

std::string ClassItem::key() const {
    const std::string cls = class_id > 9 ? util::format("%X", class_id) : std::to_string(class_id);
    const std::string item = item_id ? std::to_string(*item_id) : "?";
    return cls + "." + item + (entry != 0 ? "[" + std::to_string(entry) + "]" : std::string());
}

double ClassItem::scaled() const { return util::round2(static_cast<double>(signed_value()) * 0.01); }

const ClassItem* ClassBlock::item(int item_id, int entry_no) const {
    for (const ClassItem& it : items)
        if (it.item_id && *it.item_id == item_id && it.entry == entry_no) return &it;
    return nullptr;
}

// ------------------------------------------------------------------------------------------ UnitDirectory

UnitDirectory::UnitDirectory(std::optional<IndexHeader> header, std::vector<IndexCategory> records)
    : header_(std::move(header)), records_(std::move(records)) {}

UnitDirectory UnitDirectory::read(UnitReader& reader) {
    const std::vector<std::uint16_t> head = reader.read_registers(reg::kIndexHeader, reg::kIndexHeaderWords);
    const IndexHeader header = IndexHeader::from_words(head);
    if (!header.layout_ok()) return UnitDirectory(header, {});  // a unit without a directory (zero / odd header)
    const std::vector<std::uint16_t> words = reader.read_registers(reg::kIndexRecords, 8 * header.record_count);
    return UnitDirectory(header, parse_records(words));
}

UnitDirectory UnitDirectory::from_words(const std::vector<std::uint16_t>& words) {
    const IndexHeader header = IndexHeader::from_words(words);
    if (!header.layout_ok()) return UnitDirectory(header, {});
    const std::size_t need = static_cast<std::size_t>(reg::kIndexHeaderWords + 8 * header.record_count);
    if (words.size() < need)
        throw std::invalid_argument("index table: " + std::to_string(words.size()) + " words, the header announces " +
                                    std::to_string(need));
    const std::vector<std::uint16_t> body(words.begin() + reg::kIndexHeaderWords,
                                          words.begin() + static_cast<std::ptrdiff_t>(need));
    return UnitDirectory(header, parse_records(body));
}

std::vector<IndexCategory> UnitDirectory::parse_records(const std::vector<std::uint16_t>& words) {
    std::vector<IndexCategory> records;
    for (std::size_t i = 0; i + 8 <= words.size(); i += 8) records.push_back(IndexCategory::from_words(words, i));
    return records;
}

const IndexCategory* UnitDirectory::find(int class_id) const {
    for (const IndexCategory& rec : records_)
        if (rec.category_id == class_id) return &rec;
    return nullptr;
}

const IndexCategory& UnitDirectory::get(int class_id) const {
    const IndexCategory* rec = find(class_id);
    if (rec == nullptr)
        throw std::out_of_range(util::format("class 0x%04X is not in this unit's directory", class_id));
    return *rec;
}

int UnitDirectory::readable_words(const IndexCategory& record) const {
    const reg::BlockExtent extent = record.extent();
    if (extent.exact) return extent.words;
    int words_max = extent.words_max;
    int next = -1;
    for (const IndexCategory& other : records_)
        if (other.data_address > record.data_address && (next < 0 || other.data_address < next))
            next = other.data_address;
    if (next >= 0) words_max = std::min(words_max, next - record.data_address);
    return std::max(extent.words, words_max);
}

std::vector<std::uint16_t> UnitDirectory::read_block(UnitReader& reader, int class_id, int offset,
                                                     std::optional<int> count) const {
    const IndexCategory& rec = get(class_id);
    const int words = rec.extent().words;
    const int readable = readable_words(rec);
    const int n = count ? *count : words - offset;
    if (offset < 0 || n < 1 || offset + n > readable) {
        const std::string size = readable == words ? std::to_string(words) + " registers"
                                                   : std::to_string(words) + "-" + std::to_string(readable) +
                                                         " registers (size uncertain)";
        throw std::out_of_range(util::format("class 0x%04X has %s at 0x%04X: offset %d + count %d is outside it",
                                             class_id, size.c_str(), rec.data_address, offset, n));
    }
    return reader.read_registers(rec.data_address + offset, n);
}

ClassBlock UnitDirectory::read_class(UnitReader& reader, int class_id, int max_words) const {
    const IndexCategory& rec = get(class_id);
    const int words = rec.extent().words;
    if (words > max_words)
        throw std::invalid_argument(util::format("class 0x%04X has %d registers (> %d); read it in parts", class_id,
                                                 words, max_words));
    std::vector<std::uint16_t> data;
    if (words > 0) data = reader.read_registers(rec.data_address, words);
    const bool uniform = rec.member_size == 1 || rec.member_size == 2;
    const int size = uniform ? rec.member_size : 1;
    const int per_entry = uniform ? rec.member_count * size : words;
    std::vector<int> ids;
    bool have_ids = false;
    if (rec.id_list_address != 0 && uniform && rec.member_count > 0) {
        const std::vector<std::uint16_t> id_words = reader.read_registers(rec.id_list_address, 2 * rec.member_count);
        for (int i = 0; i < rec.member_count && static_cast<std::size_t>(2 * i) < id_words.size(); ++i)
            ids.push_back(id_words[static_cast<std::size_t>(2 * i)]);
        have_ids = true;
    }

    ClassBlock block;
    block.class_id = class_id;
    block.address = rec.data_address;
    block.words = words;
    block.member_size = rec.member_size;
    block.entries = rec.list_count;
    if (rec.id_list_address != 0) block.id_map_address = rec.id_list_address;
    for (std::size_t n = 0; n + static_cast<std::size_t>(size) <= data.size(); n += static_cast<std::size_t>(size)) {
        ClassItem item;
        item.class_id = class_id;
        const int pos = static_cast<int>(n);
        item.entry = per_entry > 0 ? pos / per_entry : 0;
        item.index = (per_entry > 0 ? pos % per_entry : pos) / size;
        item.address = rec.data_address + pos;
        item.size = size;
        item.raw = size == 1 ? data[n] : reg::from_u32_words(data[n], data[n + 1]);
        if (have_ids && static_cast<std::size_t>(item.index) < ids.size())
            item.item_id = ids[static_cast<std::size_t>(item.index)];
        block.items.push_back(item);
    }
    return block;
}

std::optional<std::map<std::string, std::uint32_t>> UnitDirectory::read_support_words(UnitReader& reader) const {
    const IndexCategory* rec = find(reg::kSupportCategory);
    if (rec == nullptr || rec->member_count < 1) return std::nullopt;
    const auto& all = reg::support_members();
    const std::size_t count = std::min(static_cast<std::size_t>(rec->member_count), all.size());
    int total = 0;
    for (std::size_t i = 0; i < count; ++i) total += all[i].second;
    const std::vector<std::uint16_t> words = reader.read_registers(rec->data_address, total);
    std::map<std::string, std::uint32_t> result;
    std::size_t offset = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const int size = all[i].second;
        if (offset + static_cast<std::size_t>(size) > words.size()) break;
        result[all[i].first] = size == 1 ? words[offset] : reg::from_u32_words(words[offset], words[offset + 1]);
        offset += static_cast<std::size_t>(size);
    }
    return result;
}

std::optional<std::uint32_t> UnitDirectory::read_accl(UnitReader& reader) const {
    const IndexCategory* rec = find(reg::kIdentificationCategory);
    if (rec == nullptr) return std::nullopt;
    const std::vector<std::uint16_t> words = reader.read_registers(rec->data_address + reg::kIdentAcclOffset, 2);
    if (words.size() < 2) return std::nullopt;
    return reg::from_u32_words(words[0], words[1]);
}

}  // namespace iai_actuator_driver
