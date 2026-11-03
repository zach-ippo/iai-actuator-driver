// C ABI 0.6 information additions (include/iai_actuator_driver.h, "axis: directory readers, self-test lines"): the
// readers of a unit's directory behind one entry point (the Python edition's info.read_directory, read_block,
// read_class, read_support_words, read_accl, unit_info with its sections as a list, and backup with its lenient row
// selection) and the lines of a running self-test (kept on the handle by iad_axis_self_test, src/c_api_ext.cpp). The
// Python facades (bindings/python/iai_actuator_driver/_native/info.py and check.py) run the Python edition's info and
// check tests over these and the 0.2 information calls. READ-ONLY: FC03 only, every read inside a block the unit's
// directory declares. Evidence: UNVERIFIED on hardware (simulator tests only).
#include <algorithm>
#include <climits>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

void write_json(const Json& json, char* buffer, std::size_t buffer_size, std::size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

[[noreturn]] void bad(const std::string& key, const char* what) {
    throw std::invalid_argument("iad_axis_info_call_json: argument '" + key + "' must be " + what);
}

/// An integer argument. A number past +/-1e9 is held there (every address and count of a unit is far smaller, so it
/// is refused by the bounds that follow, never wrapped).
std::optional<int> opt_int(const Json& args, const char* key) {
    const Json* v = args.find(key);
    if (v == nullptr || v->is_null()) return std::nullopt;
    if (v->type() != Json::Type::Int) bad(key, "an integer or null");
    return static_cast<int>(std::max(-1000000000LL, std::min(1000000000LL, v->as_int())));
}

int need_int(const Json& args, const char* key) {
    const std::optional<int> v = opt_int(args, key);
    if (!v) bad(key, "an integer");
    return *v;
}

/// The index table read from the unit (info.read_directory): header, then LQNT records of DQNT words. Unlike
/// UnitDirectory::read, a header that is not 8 words per record and 1..64 records is refused (ValueError in Python).
UnitDirectory read_directory(UnitReader& rd) {
    const IndexHeader header = IndexHeader::from_words(rd.read_registers(reg::kIndexHeader, reg::kIndexHeaderWords));
    if (!header.layout_ok())
        throw std::invalid_argument(util::format("index table: unexpected layout IndexHeader(model_code=%d, "
                                                 "words_per_record=%d, record_count=%d, rx_buffer=%d, tx_buffer=%d)",
                                                 header.model_code, header.words_per_record, header.record_count,
                                                 header.rx_buffer, header.tx_buffer));
    const std::vector<std::uint16_t> words = rd.read_registers(reg::kIndexRecords, 8 * header.record_count);
    return UnitDirectory(header, UnitDirectory::parse_records(words));
}

/// The records a reader works from (info._records): the given ones, else the directory the axis has read, else the
/// unit's index table read now.
UnitDirectory records_of(Axis& axis, UnitReader& rd, const Json& args) {
    const Json* given = args.find("records");
    if (given != nullptr && !given->is_null()) {
        if (given->type() != Json::Type::Array) bad("records", "a list of 8-number records or null");
        std::vector<IndexCategory> records;
        for (const Json& r : given->as_array()) {
            if (r.type() != Json::Type::Array || r.size() != 8) bad("records", "a list of 8-number records or null");
            std::vector<std::uint16_t> words;
            for (const Json& w : r.as_array()) {
                if (w.type() != Json::Type::Int || w.as_int() < 0 || w.as_int() > 0xFFFF)
                    bad("records", "a list of records of 8 numbers 0..65535");
                words.push_back(static_cast<std::uint16_t>(w.as_int()));
            }
            records.push_back(IndexCategory::from_words(words));
            // a block of more words than the 16-bit register space holds is no unit's (DQ x MS x LQ can pass 2^31)
            const IndexCategory& rec = records.back();
            if (rec.extent().words > 0xFFFF)
                throw std::invalid_argument(util::format(
                    "iad_axis_info_call_json: argument 'records': class 0x%04X declares a block of more than the "
                    "65535 registers a unit has (DQ %d, MS %d, LQ %d)",
                    rec.category_id, rec.member_count, rec.member_size, rec.list_count));
        }
        return UnitDirectory(std::nullopt, records);
    }
    std::vector<IndexCategory> known = axis.state().categories;
    if (!known.empty()) return UnitDirectory(std::nullopt, std::move(known));
    return UnitDirectory(std::nullopt, read_directory(rd).records());
}

Json record_json(const IndexCategory& r) {
    return Json(std::vector<int>{r.category_id, r.model_code, r.member_count, r.list_count, r.data_address,
                                 r.id_list_address, r.coil_base, r.member_size});
}

Json words_json(const std::vector<std::uint16_t>& words) {
    Json out = Json::array();
    for (std::uint16_t w : words) out.push_back(Json(static_cast<int>(w)));
    return out;
}

/// A class block with its raw items (info.ClassBlock / ClassItem fields).
Json block_json(const ClassBlock& b) {
    Json items = Json::array();
    for (const ClassItem& it : b.items) {
        Json i = Json::object();
        i.set("class_id", it.class_id);
        i.set("item_id", it.item_id);
        i.set("entry", it.entry);
        i.set("index", it.index);
        i.set("address", it.address);
        i.set("size", it.size);
        i.set("raw", static_cast<long long>(it.raw));
        items.push_back(i);
    }
    Json out = Json::object();
    out.set("class_id", b.class_id);
    out.set("address", b.address);
    out.set("words", b.words);
    out.set("member_size", b.member_size);
    out.set("entries", b.entries);
    out.set("id_map_address", b.id_map_address);
    out.set("items", items);
    return out;
}

/// The row selection of a backup as the Python edition takes it: "used", "all", row numbers; any other text and any
/// number outside the unit's table become a positions error of the backup (info::BackupRows), never a refusal here.
info::BackupRows backup_rows_arg(const Json& args) {
    const Json* rows = args.find("rows");
    if (rows == nullptr || rows->is_null()) return info::BackupRows::used();
    if (rows->type() == Json::Type::String) {
        if (rows->as_string() == "all") return info::BackupRows::all();
        info::BackupRows r;
        r.selection = rows->as_string();  // "used", or reported by the backup
        return r;
    }
    if (rows->type() != Json::Type::Array) bad("rows", "\"used\", \"all\", a list of row numbers or null");
    std::vector<int> numbers;
    for (const Json& n : rows->as_array()) {
        if (n.type() != Json::Type::Int) bad("rows", "\"used\", \"all\", a list of row numbers or null");
        numbers.push_back(static_cast<int>(std::max<long long>(INT_MIN, std::min<long long>(INT_MAX, n.as_int()))));
    }
    return info::BackupRows::list(numbers);
}

/// The sections of unit_info: null = all, else a list of names (an empty list reads none).
std::optional<std::vector<std::string>> sections_arg(const Json& args) {
    const Json* given = args.find("sections");
    if (given == nullptr || given->is_null()) return std::nullopt;
    if (given->type() != Json::Type::Array) bad("sections", "a list of section names or null");
    std::vector<std::string> names;
    for (const Json& s : given->as_array()) {
        if (s.type() != Json::Type::String) bad("sections", "a list of section names or null");
        names.push_back(s.as_string());
    }
    return names;
}

Json info_call(Axis& axis, const std::string& function, const Json& args) {
    if (function == "unit_info") return info::unit_info(axis, sections_arg(args));
    if (function == "backup") return info::backup(axis, backup_rows_arg(args));
    std::unique_ptr<UnitReader> reader = axis.bus()->reader(axis.slave());
    UnitReader& rd = *reader;
    if (function == "read_directory") {
        const UnitDirectory dir = read_directory(rd);
        const IndexHeader& h = *dir.header();
        Json header = Json::object();
        header.set("model_code", h.model_code);
        header.set("words_per_record", h.words_per_record);
        header.set("record_count", h.record_count);
        header.set("rx_buffer", h.rx_buffer);
        header.set("tx_buffer", h.tx_buffer);
        Json records = Json::array();
        for (const IndexCategory& r : dir.records()) records.push_back(record_json(r));
        Json out = Json::object();
        out.set("header", header);
        out.set("records", records);
        return out;
    }
    if (function == "read_block") {
        const int class_id = need_int(args, "class_id");
        const UnitDirectory dir = records_of(axis, rd, args);
        return words_json(dir.read_block(rd, class_id, opt_int(args, "offset").value_or(0), opt_int(args, "count")));
    }
    if (function == "read_class") {
        const int class_id = need_int(args, "class_id");
        const UnitDirectory dir = records_of(axis, rd, args);
        return block_json(
            dir.read_class(rd, class_id, opt_int(args, "max_words").value_or(UnitDirectory::kMaxClassWords)));
    }
    if (function == "read_support_words") {
        const std::optional<std::map<std::string, std::uint32_t>> words =
            records_of(axis, rd, args).read_support_words(rd);
        if (!words) return Json();
        Json out = Json::object();
        for (const auto& member : reg::support_members()) {  // in member order, as the Python edition's dict
            const auto it = words->find(member.first);
            if (it != words->end()) out.set(it->first, static_cast<long long>(it->second));
        }
        return out;
    }
    if (function == "read_accl") {
        const std::optional<std::uint32_t> accl = records_of(axis, rd, args).read_accl(rd);
        return accl ? Json(static_cast<long long>(*accl)) : Json();
    }
    throw std::logic_error("iad_axis_info_call_json: no function '" + function + "' in this library");
}

}  // namespace

iad_result IAD_CALL iad_axis_info_call_json(iad_axis* axis, const char* function, const char* args_json, char* buffer,
                                            size_t buffer_size, size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        const std::string name = need(function, "function");
        const Json args = args_json && *args_json ? Json::parse(args_json) : Json::object();
        if (args.type() != Json::Type::Object)
            throw std::invalid_argument("iad_axis_info_call_json: the arguments must be a JSON object");
        std::string text = info_call(*e->axis, name, args).dump();
        keep_thread_result(text);  // (0.7, py-fixes) iad_last_result_json
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            e->last_result = text;  // kept even when the buffer is too small (no second read)
        }
        write_text_output(text, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_axis_self_test_items_json(iad_axis* axis, int32_t first, char* buffer, size_t buffer_size,
                                                  size_t* required) {
    return guarded([&] {
        const auto e = find_axis(axis);
        check_text_output(buffer, buffer_size, required);
        if (first < 0) throw std::invalid_argument("first must be >= 0");
        Json items = Json::array();
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            for (std::size_t i = static_cast<std::size_t>(first); i < e->check_items.size(); ++i)
                items.push_back(to_json(e->check_items[i]));
        }
        write_json(items, buffer, buffer_size, required);
    });
}
