// Private helpers of the C ABI (c_api_impl.hpp): the handle registry, the last error and the exception mapping, the
// text outputs and the struct conversions. The exported functions are in c_api.cpp.
//
// Evidence: UNVERIFIED on hardware (the C ABI is tested on the simulator only).

#include "c_api_impl.hpp"

#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <ios>
#include <limits>
#include <new>
#include <unordered_map>
#include <utility>

#include "iai_actuator_driver/coord.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace capi {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ------------------------------------------------------------------------------------------ small conversions

double num(const std::optional<double>& value) { return value ? *value : kNaN; }
std::int32_t inum(const std::optional<int>& value) { return value ? static_cast<std::int32_t>(*value) : IAD_INT_UNKNOWN; }
iad_tribool tri(const std::optional<bool>& value) { return value ? (*value ? IAD_TRUE : IAD_FALSE) : IAD_UNKNOWN; }
iad_bool flag(bool value) { return value ? IAD_TRUE : IAD_FALSE; }
bool is_set(std::int32_t value) { return value != IAD_INT_UNKNOWN; }

}  // namespace

// ============================================================================================ errors

namespace {

/// The details of one exception for iad_last_error_json (filled only when asked: classify(e, nullptr) allocates
/// nothing, so the code survives an out-of-memory condition).
struct Detail {
    std::string kind;
    std::string message;
    std::vector<std::string> notes;
    bool stop_attempted = false;
    Json extra = Json::object();
};

iad_result noted(Detail* d, iad_result code, const char* kind, const std::exception& e) {
    if (d) {
        d->kind = kind;
        d->message = e.what();
        if (const auto* driver = dynamic_cast<const EcDriverError*>(&e)) {
            d->notes = driver->notes();
            d->stop_attempted = driver->stop_attempted();
        }
        if (const auto* modbus = dynamic_cast<const ModbusError*>(&e)) d->extra.set("attempts", modbus->attempts());
    }
    return code;
}

Json build_json(iad_result code, const Detail& d);

/// The code of an exception and (d != nullptr) its details. The most specific class is caught first. May throw
/// std::bad_alloc while filling d.
iad_result classify(const std::exception_ptr& error, Detail* d) {
    try {
        std::rethrow_exception(error);
    } catch (const BufferTooSmall& e) {
        return noted(d, IAD_ERR_BUFFER_TOO_SMALL, "BufferTooSmall", e);
    } catch (const CellError& e) {
        if (d) {
            d->extra.set("operation", e.operation());
            Json axes = Json::object();
            for (const auto& kv : e.errors()) {
                Detail one;
                const iad_result code = classify(kv.second, &one);
                // (0.5, py-cell) every detail of that axis's error (notes, alarm_code ...), as for a call of its own
                axes.set(kv.first, build_json(code, one));
            }
            d->extra.set("axes", std::move(axes));
        }
        return noted(d, IAD_ERR_CELL, "CellError", e);
    } catch (const coord::CoordinatedMotionError& e) {  // 0.2: a coordinated run stopped (the driver code, its own kind)
        if (d) {
            d->extra.set("axis", Json(e.axis()));
            Json cause;
            if (e.cause()) {
                Detail one;
                const iad_result code = classify(e.cause(), &one);
                cause = Json::object();
                cause.set("code", code);
                cause.set("name", iad_result_name(code));
                cause.set("kind", one.kind);
                cause.set("message", one.message);
            }
            d->extra.set("cause", std::move(cause));
            Json stop = Json::object();
            Json stopped = Json::array();
            for (const auto& kv : e.stop().results) stopped.push_back(kv.first);
            Json failed = Json::object();
            for (const auto& kv : e.stop().messages) failed.set(kv.first, kv.second);
            stop.set("stopped", std::move(stopped));
            stop.set("errors", std::move(failed));
            d->extra.set("stop", std::move(stop));
            Json result = e.result().to_json();
            result.set("summary", e.result().summary());
            d->extra.set("result", std::move(result));
        }
        return noted(d, IAD_ERR_DRIVER, "CoordinatedMotionError", e);
    } catch (const coord::InfeasiblePathError& e) {  // a soft-limit kind: nothing was sent
        if (d) {
            d->extra.set("reasons", Json(e.reasons()));
            d->extra.set("max_speed", Json(e.max_speed()));
        }
        return noted(d, IAD_ERR_SOFT_LIMIT, "InfeasiblePathError", e);
    } catch (const coord::NoMotionError& e) {
        return noted(d, IAD_ERR_INVALID_ARGUMENT, "NoMotionError", e);
    } catch (const recovery::UnitReplacedError& e) {
        if (d) {
            d->extra.set("expected", e.expected().to_json());
            d->extra.set("found", e.found().to_json());
            d->extra.set("report", e.report() ? e.report()->to_json() : Json());
        }
        return noted(d, IAD_ERR_UNIT_REPLACED, "UnitReplacedError", e);
    } catch (const recovery::RecoveryError& e) {
        if (d) d->extra.set("report", e.report() ? e.report()->to_json() : Json());
        return noted(d, IAD_ERR_RECOVERY, "RecoveryError", e);
    } catch (const WaitCancelledError& e) {
        return noted(d, IAD_ERR_CANCELLED, "WaitCancelledError", e);
    } catch (const ServoBlockedError& e) {
        if (d) d->extra.set("reasons", Json(e.reasons()));
        return noted(d, IAD_ERR_SERVO_BLOCKED, "ServoBlockedError", e);
    } catch (const GatewayModeError& e) {  // the line's not-ready kinds: their own class name, the not-ready code
        return noted(d, IAD_ERR_NOT_READY, "GatewayModeError", e);
    } catch (const PositionUnitError& e) {
        return noted(d, IAD_ERR_NOT_READY, "PositionUnitError", e);
    } catch (const UnsupportedLineError& e) {
        return noted(d, IAD_ERR_NOT_READY, "UnsupportedLineError", e);
    } catch (const EcNotReadyError& e) {
        return noted(d, IAD_ERR_NOT_READY, "EcNotReadyError", e);
    } catch (const SoftLimitError& e) {
        // (0.6) a row the unit rejected (Axis::Impl::row_rejected) carries the rejection nested: its details as "cause"
        const auto* nested = dynamic_cast<const std::nested_exception*>(&e);
        if (d && nested && nested->nested_ptr()) {
            Detail one;
            const iad_result code = classify(nested->nested_ptr(), &one);
            d->extra.set("cause", build_json(code, one));
        }
        return noted(d, IAD_ERR_SOFT_LIMIT, "SoftLimitError", e);
    } catch (const EcAlarmError& e) {
        if (d) d->extra.set("alarm_code", e.alarm_code());
        return noted(d, IAD_ERR_ALARM, "EcAlarmError", e);
    } catch (const MotionTimeoutError& e) {
        return noted(d, IAD_ERR_MOTION_TIMEOUT, "MotionTimeoutError", e);
    } catch (const MotionIncompleteError& e) {
        return noted(d, IAD_ERR_MOTION_INCOMPLETE, "MotionIncompleteError", e);
    } catch (const NonVolatileWriteError& e) {
        return noted(d, IAD_ERR_NONVOLATILE_REFUSED, "NonVolatileWriteError", e);
    } catch (const VerificationError& e) {
        return noted(d, IAD_ERR_VERIFICATION, "VerificationError", e);
    } catch (const EcError& e) {
        return noted(d, IAD_ERR_DRIVER, "EcError", e);
    } catch (const ModbusTimeout& e) {
        return noted(d, IAD_ERR_MODBUS_TIMEOUT, "ModbusTimeout", e);
    } catch (const ModbusCrcError& e) {
        return noted(d, IAD_ERR_FRAME, "ModbusCrcError", e);
    } catch (const ModbusFrameError& e) {
        return noted(d, IAD_ERR_FRAME, "ModbusFrameError", e);
    } catch (const ModbusSlaveError& e) {
        if (d) {
            d->extra.set("exception_code", e.code());
            d->extra.set("function", e.function());
            d->extra.set("reason", e.code_name());
        }
        return noted(d, IAD_ERR_SLAVE_EXCEPTION, "ModbusSlaveError", e);
    } catch (const IaiCommandRejected& e) {
        if (d) {
            d->extra.set("function", e.function());
            d->extra.set("command", e.command());
            d->extra.set("rejection", e.rejection());
            d->extra.set("reason", e.reason());
            d->extra.set("hint", e.hint());
            // 0.5 (core-bus): the reply's status, as the call returns it unchecked (0x48: its four words)
            if (!e.status_words().empty()) {
                Json words = Json::array();
                for (std::uint32_t w : e.status_words()) words.push_back(Json(static_cast<long long>(w)));
                d->extra.set("status", std::move(words));
            } else {
                d->extra.set("status", e.status_byte() >= 0 ? Json(e.status_byte()) : Json());
            }
        }
        return noted(d, IAD_ERR_COMMAND_REJECTED, "IaiCommandRejected", e);
    } catch (const TransportError& e) {
        return noted(d, IAD_ERR_TRANSPORT, "TransportError", e);
    } catch (const ModbusError& e) {
        return noted(d, IAD_ERR_MODBUS, "ModbusError", e);
    } catch (const LineRuleError& e) {  // an unsafe-refused kind: its own class name, the same code
        return noted(d, IAD_ERR_UNSAFE_REFUSED, "LineRuleError", e);
    } catch (const UnsafeCommandError& e) {
        return noted(d, IAD_ERR_UNSAFE_REFUSED, "UnsafeCommandError", e);
    } catch (const EcDriverError& e) {
        return noted(d, IAD_ERR_DRIVER, "EcDriverError", e);
    } catch (const std::invalid_argument& e) {
        return noted(d, IAD_ERR_INVALID_ARGUMENT, "std::invalid_argument", e);
    } catch (const std::out_of_range& e) {
        return noted(d, IAD_ERR_INVALID_ARGUMENT, "std::out_of_range", e);
    } catch (const std::logic_error& e) {
        return noted(d, IAD_ERR_UNSUPPORTED, "std::logic_error", e);
    } catch (const std::ios_base::failure& e) {  // 0.2: a file that could not be read or written
        return noted(d, IAD_ERR_FILE, "std::ios_base::failure", e);
    } catch (const std::filesystem::filesystem_error& e) {
        return noted(d, IAD_ERR_FILE, "std::filesystem::filesystem_error", e);
    } catch (const std::bad_alloc& e) {
        return noted(d, IAD_ERR_INTERNAL, "std::bad_alloc", e);
    } catch (const std::exception& e) {
        return noted(d, IAD_ERR_INTERNAL, "std::exception", e);
    } catch (...) {
        if (d) {
            d->kind = "unknown";
            d->message = "unknown exception (not derived from std::exception)";
        }
        return IAD_ERR_INTERNAL;
    }
}

Json build_json(iad_result code, const Detail& d) {
    Json j = Json::object();
    j.set("code", code);
    j.set("name", iad_result_name(code));
    j.set("kind", d.kind);
    j.set("message", d.message);
    j.set("notes", Json(d.notes));
    j.set("stop_attempted", d.stop_attempted);
    for (const auto& kv : d.extra.as_object()) j.set(kv.first, kv.second);
    return j;
}

// The calling thread's last error: code, message (what()), JSON ("" until built).
thread_local iad_result t_code = IAD_OK;
thread_local std::string t_message;
thread_local std::string t_json;

}  // namespace

ErrorDescription describe_error(const std::exception_ptr& error) {
    Detail d;
    ErrorDescription out;
    out.code = classify(error, &d);
    out.kind = d.kind;
    out.message = d.message;
    out.json = build_json(out.code, d);
    return out;
}

iad_result error_code(const std::exception_ptr& error) noexcept {
    try {
        return classify(error, nullptr);
    } catch (...) {
        return IAD_ERR_INTERNAL;
    }
}

void clear_last_error() noexcept {
    t_code = IAD_OK;
    t_message.clear();
    t_json.clear();
}

iad_result set_last_error(const std::exception_ptr& error) noexcept {
    const iad_result code = error_code(error);
    t_code = code;
    try {
        ErrorDescription d = describe_error(error);
        t_message = std::move(d.message);
        t_json = d.json.dump();
    } catch (...) {
        // out of memory while describing it: the code stays, the texts are dropped
        t_message.clear();
        t_json.clear();
    }
    return code;
}

const char* last_error_message() noexcept { return t_message.c_str(); }

std::string last_error_json() {
    if (t_code == IAD_OK) return "{\"code\":0}";
    if (!t_json.empty()) return t_json;
    // the details were lost (out of memory while describing the error): the code and its name only
    return "{\"code\":" + std::to_string(t_code) + ",\"name\":\"" + iad_result_name(t_code) + "\"}";
}

// (0.7, py-fixes) the result the last call on this thread kept (iad_last_result_json)
namespace {
thread_local std::optional<std::string> t_result;
}  // namespace

void keep_thread_result(const std::string& text) { t_result = text; }

std::optional<std::string> thread_result() { return t_result; }

// ============================================================================================ handles

namespace {

const char* object_kind_name(ObjectKind kind) {
    switch (kind) {
        case ObjectKind::Transport: return "a transport handle";
        case ObjectKind::Bus: return "a bus handle";
        case ObjectKind::Sim: return "a simulator handle";
        case ObjectKind::Line: return "a line handle";        // 0.5 (py-gateway)
        case ObjectKind::Gateway: return "a gateway handle";  // 0.5 (py-gateway)
        default: return "a simulated line handle";
    }
}

/// Live handles. Tokens come from one counter (never reused, never 0), so a token is live in at most one map. The
/// registry is never destroyed: handles still open when the library unloads leave their ports open until the
/// process exits (docs/NATIVE_ABI.md), instead of closing ports from a DLL detach.
struct Registry {
    std::mutex mutex;
    std::uintptr_t next = 1;
    std::unordered_map<std::uintptr_t, std::shared_ptr<AxisEntry>> axes;
    std::unordered_map<std::uintptr_t, std::shared_ptr<CellEntry>> cells;
    std::unordered_map<std::uintptr_t, std::shared_ptr<MoveEntry>> moves;  ///< 0.2
    std::unordered_map<std::uintptr_t, std::pair<ObjectKind, std::shared_ptr<void>>> objects;  ///< 0.3

    std::uintptr_t issue() {
        if (next == 0) throw std::runtime_error("handle tokens exhausted");
        return next++;
    }
    /// The token is a live handle of another kind than the one looked up.
    const char* kind_of(std::uintptr_t t) const {
        if (axes.count(t)) return "an axis handle";
        if (cells.count(t)) return "a cell handle";
        if (moves.count(t)) return "a move handle";
        auto o = objects.find(t);
        if (o != objects.end()) return object_kind_name(o->second.first);
        return nullptr;
    }
};

Registry& registry() {
    static Registry* r = new Registry();
    return *r;
}

std::uintptr_t token(const void* handle) { return reinterpret_cast<std::uintptr_t>(handle); }

}  // namespace

iad_axis* register_axis(std::shared_ptr<Axis> axis) {
    auto entry = std::make_shared<AxisEntry>();
    entry->axis = std::move(axis);
    entry->owned = true;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const std::uintptr_t t = r.issue();
    r.axes.emplace(t, std::move(entry));
    return reinterpret_cast<iad_axis*>(t);
}

iad_axis* register_cell_axis(std::shared_ptr<Axis> axis, iad_cell* cell) {
    auto entry = std::make_shared<AxisEntry>();
    entry->axis = std::move(axis);
    entry->owned = false;
    entry->cell = token(cell);
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    if (!r.cells.count(entry->cell)) throw std::invalid_argument("invalid cell handle (closed meanwhile)");
    const std::uintptr_t t = r.issue();
    r.axes.emplace(t, std::move(entry));
    return reinterpret_cast<iad_axis*>(t);
}

iad_cell* register_cell(std::shared_ptr<Cell> cell) {
    auto entry = std::make_shared<CellEntry>();
    entry->cell = std::move(cell);
    return register_cell(std::move(entry));
}

iad_cell* register_cell(std::shared_ptr<CellEntry> entry) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const std::uintptr_t t = r.issue();
    r.cells.emplace(t, std::move(entry));
    return reinterpret_cast<iad_cell*>(t);
}

iad_move_handle* register_move(std::shared_ptr<MoveEntry> entry) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const std::uintptr_t t = r.issue();
    r.moves.emplace(t, std::move(entry));
    return reinterpret_cast<iad_move_handle*>(t);
}

std::shared_ptr<AxisEntry> find_axis(iad_axis* handle) {
    if (!handle) throw std::invalid_argument("axis handle is NULL");
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.axes.find(token(handle));
    if (it == r.axes.end()) {
        if (const char* other = r.kind_of(token(handle)))
            throw std::invalid_argument(std::string(other) + " was passed as an axis handle");
        throw std::invalid_argument("invalid axis handle (closed, its cell closed, or never opened)");
    }
    return it->second;
}

std::shared_ptr<CellEntry> find_cell(iad_cell* handle) {
    if (!handle) throw std::invalid_argument("cell handle is NULL");
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.cells.find(token(handle));
    if (it == r.cells.end()) {
        if (const char* other = r.kind_of(token(handle)))
            throw std::invalid_argument(std::string(other) + " was passed as a cell handle");
        throw std::invalid_argument("invalid cell handle (closed or never opened)");
    }
    return it->second;
}

std::shared_ptr<MoveEntry> find_move(iad_move_handle* handle) {
    if (!handle) throw std::invalid_argument("move handle is NULL");
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.moves.find(token(handle));
    if (it == r.moves.end()) {
        if (const char* other = r.kind_of(token(handle)))
            throw std::invalid_argument(std::string(other) + " was passed as a move handle");
        throw std::invalid_argument("invalid move handle (closed or never started)");
    }
    return it->second;
}

std::shared_ptr<AxisEntry> release_axis(iad_axis* handle, bool* other_kind) {
    *other_kind = false;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.axes.find(token(handle));
    if (it == r.axes.end()) {
        *other_kind = r.kind_of(token(handle)) != nullptr;
        return nullptr;
    }
    std::shared_ptr<AxisEntry> entry = std::move(it->second);
    r.axes.erase(it);
    return entry;
}

std::shared_ptr<CellEntry> release_cell(iad_cell* handle, bool* other_kind,
                                        std::vector<std::shared_ptr<AxisEntry>>* released_axes) {
    *other_kind = false;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const std::uintptr_t t = token(handle);
    auto it = r.cells.find(t);
    if (it == r.cells.end()) {
        *other_kind = r.kind_of(t) != nullptr;
        return nullptr;
    }
    std::shared_ptr<CellEntry> entry = std::move(it->second);
    r.cells.erase(it);
    for (auto a = r.axes.begin(); a != r.axes.end();) {
        if (a->second->cell == t) {
            if (released_axes) released_axes->push_back(a->second);
            a = r.axes.erase(a);
        } else {
            ++a;
        }
    }
    return entry;
}

std::shared_ptr<MoveEntry> release_move(iad_move_handle* handle, bool* other_kind) {
    *other_kind = false;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.moves.find(token(handle));
    if (it == r.moves.end()) {
        *other_kind = r.kind_of(token(handle)) != nullptr;
        return nullptr;
    }
    std::shared_ptr<MoveEntry> entry = std::move(it->second);
    r.moves.erase(it);
    return entry;
}

// ============================================================================================ 0.3 object handles

std::uintptr_t register_object(ObjectKind kind, std::shared_ptr<void> entry) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const std::uintptr_t t = r.issue();
    r.objects.emplace(t, std::make_pair(kind, std::move(entry)));
    return t;
}

std::shared_ptr<void> find_object(const void* handle, ObjectKind kind) {
    const std::string what = std::string(object_kind_name(kind)).substr(2);  // "transport handle"
    if (!handle) throw std::invalid_argument(what + " is NULL");
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.objects.find(token(handle));
    if (it == r.objects.end() || it->second.first != kind) {
        if (const char* other = r.kind_of(token(handle)))
            throw std::invalid_argument(std::string(other) + " was passed as a" +
                                        (what[0] == 'a' ? "n " : " ") + what);
        throw std::invalid_argument("invalid " + what + " (closed or never opened)");
    }
    return it->second.second;
}

std::shared_ptr<void> release_object(const void* handle, ObjectKind kind, bool* other_kind) {
    *other_kind = false;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    auto it = r.objects.find(token(handle));
    if (it == r.objects.end() || it->second.first != kind) {
        *other_kind = r.kind_of(token(handle)) != nullptr;
        return nullptr;
    }
    std::shared_ptr<void> entry = std::move(it->second.second);
    r.objects.erase(it);
    return entry;
}

void check_reserved(const std::uint64_t* slots, std::size_t count, const char* field) {
    for (std::size_t i = 0; i < count; ++i)
        if (slots[i] != 0)
            throw std::logic_error(std::string(field) + "[" + std::to_string(i) +
                                   "] is set: a field of a later ABI than this library's");
}

// ============================================================================================ text outputs

void copy_text(char* dst, std::size_t size, const std::string& text) noexcept {
    if (!dst || size == 0) return;
    std::size_t n = text.size() < size - 1 ? text.size() : size - 1;
    if (n < text.size()) {
        // never end inside a UTF-8 sequence: back up while the first byte left out is a continuation byte
        while (n > 0 && (static_cast<unsigned char>(text[n]) & 0xC0) == 0x80) --n;
    }
    std::memcpy(dst, text.data(), n);
    std::memset(dst + n, 0, size - n);
}

void check_text_output(const char* buffer, std::size_t buffer_size, const std::size_t* required) {
    need(required, "required");
    if (!buffer && buffer_size > 0) throw std::invalid_argument("buffer is NULL with buffer_size > 0");
}

void write_text_output(const std::string& text, char* buffer, std::size_t buffer_size, std::size_t* required) {
    *required = text.size() + 1;
    if (buffer_size < text.size() + 1) {
        if (buffer_size > 0) buffer[0] = '\0';
        throw BufferTooSmall("buffer of " + std::to_string(buffer_size) + " bytes is too small: " +
                             std::to_string(text.size() + 1) + " bytes needed (see required)");
    }
    std::memcpy(buffer, text.data(), text.size());
    buffer[text.size()] = '\0';
}

// ============================================================================================ inputs

std::vector<std::string> split_list(const char* text, bool unique) {
    std::vector<std::string> out;
    if (!text) return out;
    const std::string all(text);
    auto trim = [](const std::string& s) {
        std::size_t a = 0, b = s.size();
        while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
        while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t')) --b;
        return s.substr(a, b - a);
    };
    if (trim(all).empty()) return out;
    std::size_t start = 0;
    for (;;) {
        const std::size_t comma = all.find(',', start);
        const std::string item = trim(all.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
        if (item.empty()) throw std::invalid_argument("empty name in the list '" + all + "'");
        bool seen = false;
        for (const auto& s : out) seen = seen || s == item;
        if (!unique || !seen) out.push_back(item);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out;
}

void cancel_listed(Cell& cell, const char* axes, const char* operation, const std::function<void(Axis&)>& act) {
    const std::vector<std::string> listed = split_list(axes);
    std::vector<std::string> known = listed.empty() ? cell.names() : std::vector<std::string>();
    std::map<std::string, std::string> messages;
    std::map<std::string, std::exception_ptr> errors;
    for (const auto& name : listed) {
        if (cell.contains(name)) {
            known.push_back(name);
            continue;
        }
        const std::out_of_range unknown("no axis '" + name + "' in this cell (axes: " + util::join(cell.names(), ", ") +
                                        ")");
        messages[name] = unknown.what();
        errors[name] = std::make_exception_ptr(unknown);
    }
    for (const auto& name : known) {  // every axis the cell has first: the unknown names are reported afterwards
        try {
            act(*cell.axis_ptr(name));
        } catch (const std::exception& e) {
            messages[name] = e.what();
            errors[name] = std::current_exception();
        }
    }
    if (!errors.empty()) throw CellError(operation, std::move(messages), std::move(errors));
}

std::optional<CommandMethod> to_method(std::int32_t method) {
    switch (method) {
        case IAD_METHOD_DEFAULT: return std::nullopt;
        case IAD_METHOD_COIL: return CommandMethod::Coil;
        case IAD_METHOD_IAI: return CommandMethod::Iai;
        case IAD_METHOD_IAI2: return CommandMethod::Iai2;
        case IAD_METHOD_REGISTER: return CommandMethod::Register;
        default:
            throw std::invalid_argument(
                "method must be IAD_METHOD_DEFAULT, IAD_METHOD_COIL, IAD_METHOD_IAI, IAD_METHOD_IAI2 or "
                "IAD_METHOD_REGISTER, got " +
                std::to_string(method));
    }
}

std::int32_t from_method(CommandMethod method) noexcept {
    switch (method) {
        case CommandMethod::Iai: return IAD_METHOD_IAI;
        case CommandMethod::Iai2: return IAD_METHOD_IAI2;
        case CommandMethod::Register: return IAD_METHOD_REGISTER;
        default: return IAD_METHOD_COIL;
    }
}

std::optional<Completion> to_completion(std::int32_t completion) {
    switch (completion) {
        case IAD_COMPLETION_DEFAULT: return std::nullopt;
        case IAD_COMPLETION_SAFE: return Completion::Safe;
        case IAD_COMPLETION_FAST: return Completion::Fast;
        default:
            throw std::invalid_argument(
                "completion must be IAD_COMPLETION_DEFAULT, IAD_COMPLETION_SAFE or IAD_COMPLETION_FAST, got " +
                std::to_string(completion));
    }
}

std::int32_t from_completion(Completion completion) noexcept {
    return completion == Completion::Fast ? IAD_COMPLETION_FAST : IAD_COMPLETION_SAFE;
}

std::optional<AccelPattern> to_accel_pattern(std::int32_t pattern) {
    switch (pattern) {
        case IAD_ACCEL_DEFAULT: return std::nullopt;
        case IAD_ACCEL_TRAPEZOID: return AccelPattern::Trapezoid;
        case IAD_ACCEL_S_MOTION: return AccelPattern::SMotion;
        case IAD_ACCEL_FILTER: return AccelPattern::Filter;
        default:
            throw std::invalid_argument("accel_pattern must be IAD_ACCEL_DEFAULT, _TRAPEZOID, _S_MOTION or _FILTER, got " +
                                        std::to_string(pattern));
    }
}

std::optional<double> to_position_unit(double unit_mm) {
    if (std::isnan(unit_mm) || unit_mm == 0.0) return std::nullopt;
    if (std::fabs(unit_mm - 0.01) < 1e-9) return 0.01;
    if (std::fabs(unit_mm - 0.001) < 1e-9) return 0.001;
    throw std::invalid_argument("position_unit_mm must be 0.01 or 0.001 (NaN or 0: not stated), got " +
                                std::to_string(unit_mm));
}

LineOptions to_line_options(std::int32_t line, double position_unit_mm) {
    LineOptions o;
    switch (line) {
        case IAD_LINE_AUTO: o.kind = "auto"; break;
        case IAD_LINE_DIRECT: o.kind = "direct"; break;
        case IAD_LINE_RCON: o.kind = "rcon"; break;
        case IAD_LINE_ROBONET: o.kind = "robonet"; break;
        case IAD_LINE_ROBONET_GATEWAY: o.kind = "robonet-gateway"; break;
        default:
            throw std::invalid_argument("line must be IAD_LINE_AUTO, _DIRECT, _RCON, _ROBONET or _ROBONET_GATEWAY, got " +
                                        std::to_string(line));
    }
    o.position_unit_mm = to_position_unit(position_unit_mm);
    return o;
}

std::optional<double> opt(double value) {
    if (std::isnan(value)) return std::nullopt;
    return value;
}

double or_default(double value, double fallback) { return std::isnan(value) ? fallback : value; }

std::optional<double> timeout_arg(double timeout_s, const char* what) {
    if (std::isnan(timeout_s)) return std::nullopt;
    if (!(timeout_s > 0.0) || !std::isfinite(timeout_s))
        throw std::invalid_argument(std::string(what) + " must be a finite number of seconds > 0 (NaN: the default), got " +
                                    util::g(timeout_s) + "; nothing sent");
    return timeout_s;
}

std::optional<double> opt_nonzero(double value) {
    if (std::isnan(value) || value == 0.0) return std::nullopt;
    return value;
}

double finite(double value, const char* name) {
    if (!std::isfinite(value)) throw std::invalid_argument(std::string(name) + " must be a finite number");
    return value;
}

namespace {

/// The checks of Axis::set_options (axis_core.cpp), here so that a bad value is refused before a port opens.
void validate_axis_options(const AxisOptions& o) {
    if (o.stroke && (!std::isfinite(o.stroke->min) || !std::isfinite(o.stroke->max) || !(o.stroke->min < o.stroke->max)))
        throw std::invalid_argument("stroke_min must be below stroke_max (both finite)");
    if (o.max_speed_mm_s && !(*o.max_speed_mm_s > 0.0 && std::isfinite(*o.max_speed_mm_s)))
        throw std::invalid_argument("max_speed_mm_s must be > 0 (NaN: the unit's own maximum)");
    if (o.max_accel_g && !(*o.max_accel_g > 0.0 && std::isfinite(*o.max_accel_g)))
        throw std::invalid_argument("max_accel_g must be > 0 (NaN: the unit's own maximum)");
    if (o.max_push_pct && !(*o.max_push_pct > 0.0 && *o.max_push_pct <= 100.0))
        throw std::invalid_argument("max_push_pct must be in (0, 100] (NaN: the unit's own maximum)");
    if (!(o.poll_interval_s >= 0.0) || !std::isfinite(o.poll_interval_s))
        throw std::invalid_argument("poll_interval_s must be >= 0");
    if (!(o.settle_s >= 0.0) || !std::isfinite(o.settle_s)) throw std::invalid_argument("settle_s must be >= 0");
}

double positive(double value, const char* name) {
    if (!(value > 0.0) || !std::isfinite(value)) throw std::invalid_argument(std::string(name) + " must be > 0");
    return value;
}

double non_negative(double value, const char* name) {
    if (!(value >= 0.0) || !std::isfinite(value)) throw std::invalid_argument(std::string(name) + " must be >= 0");
    return value;
}

std::optional<int> baud_input(std::int32_t rate, const char* name) {
    if (rate == 0 || rate == IAD_INT_UNKNOWN) return std::nullopt;
    if (rate < 0) throw std::invalid_argument(std::string(name) + " must be 0 (detect) or a positive rate");
    return rate;
}

int retries_input(std::int32_t retries, int fallback) {
    if (!is_set(retries)) return fallback;
    if (retries < 0 || retries > 100) throw std::invalid_argument("retries must be 0..100");
    return retries;
}

int slave_input(std::int32_t slave) {
    if (!is_set(slave)) return 1;
    if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
    return slave;
}

}  // namespace

AxisOptions to_axis_options(const iad_axis_options* input) {
    AxisOptions o;
    if (input) {
        const iad_axis_options in = read_input(input, "iad_axis_options");
        const iad_axis_options* p = &in;
        const bool has_min = !std::isnan(p->stroke_min);
        const bool has_max = !std::isnan(p->stroke_max);
        if (has_min != has_max) throw std::invalid_argument("stroke_min and stroke_max: set both or neither (NaN)");
        if (has_min) o.stroke = Span{p->stroke_min, p->stroke_max};
        o.max_speed_mm_s = opt(p->max_speed_mm_s);
        o.max_accel_g = opt(p->max_accel_g);
        o.max_push_pct = opt(p->max_push_pct);
        o.poll_interval_s = or_default(p->poll_interval_s, o.poll_interval_s);
        o.settle_s = or_default(p->settle_s, o.settle_s);
        if (const auto m = to_method(p->method)) o.method = *m;
        o.auto_modbus_control = p->auto_modbus_control != 0;
        // 0.2 slots (zero = the earlier behaviour: the core's defaults)
        if (std::isnan(p->push_settle_s) || p->push_settle_s == 0.0)
            o.push_settle_s = kPushSettleSDefault;
        else if (p->push_settle_s < 0.0)
            o.push_settle_s = 0.0;  // no settle: the first sample counts
        else
            o.push_settle_s = finite(p->push_settle_s, "push_settle_s");
        if (const auto c = to_completion(p->completion)) o.completion = *c;
    }
    validate_axis_options(o);
    return o;
}

OpenOptions to_open_options(const iad_open_options* input, const iad_axis_options* axis_options) {
    OpenOptions o;
    o.axis = to_axis_options(axis_options);
    if (input) {
        const iad_open_options in = read_input(input, "iad_open_options");
        const iad_open_options* p = &in;
        o.read_timeout_s = positive(or_default(p->read_timeout_s, o.read_timeout_s), "read_timeout_s");
        o.bus.min_gap_s = non_negative(or_default(p->min_gap_s, o.bus.min_gap_s), "min_gap_s");
        o.slave = slave_input(p->slave);
        o.baud_rate = baud_input(p->baud_rate, "baud_rate");
        o.bus.retries = retries_input(p->retries, o.bus.retries);
        o.line = to_line_options(p->line, p->position_unit_mm);
        o.fast = p->fast != 0;
    }
    return o;
}

SimProfile to_sim_profile(const char* profile) {
    if (!profile) return SimProfile::Slider;
    const auto p = sim_profile_from_string(profile);
    if (!p)
        throw std::invalid_argument(std::string("unknown simulator profile '") + profile +
                                    "' (slider, gripper, rod, pcon, rotary, unknown)");
    return *p;
}

SimulatedUnitOptions to_sim_options(const iad_sim_options* input, SimProfile profile, int slave) {
    SimulatedUnitOptions o;
    o.profile = profile;
    o.slave = slave;
    if (input) {
        const iad_sim_options in = read_input(input, "iad_sim_options");
        const iad_sim_options* p = &in;
        o.time_scale = positive(or_default(p->time_scale, o.time_scale), "time_scale");
        if (!std::isnan(p->position_mm)) o.position_mm = finite(p->position_mm, "position_mm");
        if (!std::isnan(p->stroke_mm)) o.stroke_mm = positive(p->stroke_mm, "stroke_mm");
        if (!std::isnan(p->max_speed_mm_s)) o.max_speed_mm_s = positive(p->max_speed_mm_s, "max_speed_mm_s");
        o.baud_rate = baud_input(p->baud_rate, "sim baud_rate");
        o.pmsl = p->pmsl != 0;
    }
    return o;
}

MoveOptions to_move_options(const iad_move_options* input, const MoveOptions& defaults) {
    MoveOptions o = defaults;
    if (input) {
        const iad_move_options in = read_input(input, "iad_move_options");
        const iad_move_options* p = &in;
        o.speed = or_default(p->speed, defaults.speed);
        o.accel_g = or_default(p->accel_g, defaults.accel_g);
        o.band = or_default(p->band, defaults.band);
        o.timeout_s = timeout_arg(p->timeout_s, "timeout_s");
        o.wait = p->wait != 0;
        // 0.2 slots (zero = the earlier behaviour)
        o.accel_pattern = to_accel_pattern(p->accel_pattern);
        if (!std::isnan(p->decel_g) && p->decel_g != 0.0) o.decel_g = finite(p->decel_g, "decel_g");
        o.completion = to_completion(p->completion);
    }
    return o;
}

PositionMoveOptions to_position_move_options(const iad_position_move_options* input) {
    PositionMoveOptions o;
    if (input) {
        const iad_position_move_options in = read_input(input, "iad_position_move_options");
        const iad_position_move_options* p = &in;
        o.timeout_s = timeout_arg(p->timeout_s, "timeout_s");
        o.speed_pct = opt(p->speed_pct);
        o.wait = p->wait != 0;
        o.allow_above_caps = p->allow_above_caps != 0;
    }
    return o;
}

NvWriteOptions to_nv_options(const iad_nv_write_options* input) {
    NvWriteOptions o;
    if (input) {
        const iad_nv_write_options in = read_input(input, "iad_nv_write_options");
        const iad_nv_write_options* p = &in;
        o.allow_nonvolatile = p->allow_nonvolatile != 0;
        o.force = p->force != 0;
        o.allow_unsafe = p->allow_unsafe != 0;
    }
    return o;
}

ParameterWriteOptions to_param_options(const iad_param_write_options* input) {
    ParameterWriteOptions o;
    if (input) {
        const iad_param_write_options in = read_input(input, "iad_param_write_options");
        const iad_param_write_options* p = &in;
        o.commit = p->commit != 0;
        o.allow_dangerous = p->allow_dangerous != 0;
        o.allow_unlisted = p->allow_unlisted != 0;
        o.allow_home_params = p->allow_home_params != 0;
        o.force = p->force != 0;
    }
    return o;
}

BaudChangeOptions to_baud_options(const iad_baud_options* input) {
    BaudChangeOptions o;
    if (input) {
        const iad_baud_options in = read_input(input, "iad_baud_options");
        const iad_baud_options* p = &in;
        o.settle_s = non_negative(or_default(p->settle_s, o.settle_s), "settle_s");
        if (p->method == IAD_BAUD_DIRECT)
            o.method = BaudChangeMethod::Direct;
        else if (p->method == IAD_BAUD_BREAK)
            o.method = BaudChangeMethod::Break;
        else if (is_set(p->method))
            throw std::invalid_argument("method must be IAD_BAUD_DIRECT or IAD_BAUD_BREAK, got " +
                                        std::to_string(p->method));
        o.allow_unverified_code = p->allow_unverified_code != 0;
    }
    return o;
}

SoftwareResetOptions to_reset_options(const iad_reset_options* input) {
    SoftwareResetOptions o;
    if (input) {
        const iad_reset_options in = read_input(input, "iad_reset_options");
        const iad_reset_options* p = &in;
        o.timeout_s = positive(or_default(p->timeout_s, o.timeout_s), "timeout_s");
        o.reset_wait_s = non_negative(or_default(p->reset_wait_s, o.reset_wait_s), "reset_wait_s");
    }
    return o;
}

// (0.8) The options of the axis commands that took scalars before 0.8: the same conversions as those scalars had.

ServoArgs to_servo_args(const iad_servo_options* input) {
    ServoArgs a;
    if (input) {
        const iad_servo_options in = read_input(input, "iad_servo_options");
        a.wait = in.wait != 0;
        a.timeout_s = timeout_arg(in.timeout_s, "timeout_s");
    }
    return a;
}

HomeArgs to_home_args(const iad_home_options* input) {
    HomeArgs a;
    if (input) {
        const iad_home_options in = read_input(input, "iad_home_options");
        a.wait = in.wait != 0;
        a.timeout_s = timeout_arg(in.timeout_s, "timeout_s");
        a.method = to_method(in.method);
    }
    return a;
}

StopArgs to_stop_args(const iad_stop_options* input) {
    StopArgs a;
    if (input) {
        const iad_stop_options in = read_input(input, "iad_stop_options");
        a.wait = in.wait != 0;
        a.timeout_s = or_default(in.timeout_s, a.timeout_s);
    }
    return a;
}

WaitIdleArgs to_wait_idle_args(const iad_wait_idle_options* input) {
    WaitIdleArgs a;
    if (input) {
        const iad_wait_idle_options in = read_input(input, "iad_wait_idle_options");
        a.target = opt(in.target);
        a.band = opt(in.band);
        if (a.target) finite(*a.target, "target");
        if (a.band) finite(*a.band, "band");
        if (!std::isnan(in.timeout_s)) {
            if (!(in.timeout_s >= 0.0) || !std::isfinite(in.timeout_s))
                throw std::invalid_argument("timeout_s must be >= 0");
            a.timeout_s = in.timeout_s;
        }
        a.completion = to_completion(in.completion);
    }
    return a;
}

ChangeArgs to_change_args(const iad_change_options* input) {
    ChangeArgs a;
    if (input) {
        const iad_change_options in = read_input(input, "iad_change_options");
        a.wait = in.wait != 0;
        a.timeout_s = timeout_arg(in.timeout_s, "timeout_s");
    }
    return a;
}

double to_cancel_paused_timeout(const iad_cancel_paused_move_options* input) {
    if (!input) return kCancelPausedTimeoutS;
    const iad_cancel_paused_move_options in = read_input(input, "iad_cancel_paused_move_options");
    return or_default(in.timeout_s, kCancelPausedTimeoutS);
}

CellOptions to_cell_options(const iad_cell_options* input, const iad_axis_options* axis_options) {
    CellOptions o;
    o.axis = to_axis_options(axis_options);
    if (input) {
        const iad_cell_options in = read_input(input, "iad_cell_options");
        const iad_cell_options* p = &in;
        o.read_timeout_s = positive(or_default(p->read_timeout_s, o.read_timeout_s), "read_timeout_s");
        o.bus.min_gap_s = non_negative(or_default(p->min_gap_s, o.bus.min_gap_s), "min_gap_s");
        o.scan_timeout_s = positive(or_default(p->scan_timeout_s, o.scan_timeout_s), "scan_timeout_s");
        if (is_set(p->max_axes) && p->max_axes != 0) {  // 0 = no limit
            if (p->max_axes < 0) throw std::invalid_argument("max_axes must be >= 1, or 0 for no limit");
            o.max_axes = p->max_axes;
        }
        o.baud_rate = baud_input(p->baud_rate, "baud_rate");
        o.bus.retries = retries_input(p->retries, o.bus.retries);
        const int first = is_set(p->scan_first) ? p->scan_first : 1;
        const int last = is_set(p->scan_last) ? p->scan_last : kMaxAxes;
        if (first < 1 || last > 247 || first > last)
            throw std::invalid_argument("scan_first .. scan_last must be within 1..247 with scan_first <= scan_last");
        o.scan_slaves.clear();
        for (int s = first; s <= last; ++s) o.scan_slaves.push_back(s);
        o.identify = p->identify != 0;
        o.stop_on_fault = p->stop_on_fault != 0;
        o.line = to_line_options(p->line, p->position_unit_mm);
    }
    return o;
}

PositionEntry from_c(const iad_position_entry& input) {
    const iad_position_entry c = read_input(&input, "iad_position_entry");
    PositionEntry e;
    e.position_mm = finite(c.position_mm, "position_mm");
    e.band_mm = finite(c.band_mm, "band_mm");
    e.speed_mm_s = finite(c.speed_mm_s, "speed_mm_s");
    e.zone_plus_mm = finite(c.zone_plus_mm, "zone_plus_mm");
    e.zone_minus_mm = finite(c.zone_minus_mm, "zone_minus_mm");
    e.accel_g = finite(c.accel_g, "accel_g");
    e.decel_g = finite(c.decel_g, "decel_g");
    e.push_power = c.push_power;
    e.load_threshold = c.load_threshold;
    e.control_flags = c.control_flags;
    return e;  // checksum: computed by the write (the input's is ignored)
}

// ============================================================================================ outputs

void to_c(const AxisOptions& o, iad_axis_options* out) {
    iad_axis_options c = zeroed<iad_axis_options>();
    c.stroke_min = o.stroke ? o.stroke->min : kNaN;
    c.stroke_max = o.stroke ? o.stroke->max : kNaN;
    c.max_speed_mm_s = num(o.max_speed_mm_s);
    c.max_accel_g = num(o.max_accel_g);
    c.max_push_pct = num(o.max_push_pct);
    c.poll_interval_s = o.poll_interval_s;
    c.settle_s = o.settle_s;
    c.method = from_method(o.method);
    c.auto_modbus_control = flag(o.auto_modbus_control);
    c.push_settle_s = o.push_settle_s > 0.0 ? o.push_settle_s : -1.0;  // 0 in the core = "no settle" = negative here
    c.completion = from_completion(o.completion);
    write_sized(out, c);  // (0.8) a sized struct: as much as the caller's struct_size says
}

void to_c(const Status& s, iad_status* out) {
    std::memset(out, 0, sizeof *out);
    out->position_mm = s.position_mm;
    out->velocity_mm_s = num(s.velocity_mm_s);
    out->current_ma = s.current_ma ? static_cast<double>(*s.current_ma) : kNaN;
    out->uptime_s = num(s.uptime_s());
    out->alarm_code = s.alarm_code;
    const auto level = s.alarm_level();
    out->alarm_level = level ? static_cast<std::int32_t>(*level) : IAD_ALARM_LEVEL_NONE;
    out->servo_on = flag(s.servo_on);
    out->homed = flag(s.homed);
    out->in_position = flag(s.in_position);
    out->moving = flag(s.moving);
    out->alarm = flag(s.alarm);
    out->warning = flag(s.warning());
    out->modbus_enabled = flag(s.modbus_enabled);
    out->emergency_stop = flag(s.emergency_stop);
    out->paused = flag(s.paused());
    out->homing = flag(s.homing());
    out->safety_speed = flag(s.safety_speed());
    out->ready = flag(s.ready());
    out->idle = flag(s.idle());
    out->target_reached = flag(s.target_reached());
    out->push_missed = flag(s.push_missed());
    out->pushing = flag(s.pushing());
    out->emg_input = tri(s.emg_input());
    out->drive_power_cut = tri(s.drive_power_cut());
    out->enable_off = tri(s.enable_off());
    out->raw_words = s.raw.words;
    out->raw_dss1 = s.raw.dss1;
    out->raw_dss2 = s.raw.dss2;
    out->raw_dsse = s.raw.dsse;
    out->raw_dipm = s.raw.dipm;
    out->raw_dopm = s.raw.dopm;
    out->rcon = flag(s.rcon());
}

void unknown_status(iad_status* out) {
    std::memset(out, 0, sizeof *out);
    out->position_mm = kNaN;
    out->velocity_mm_s = kNaN;
    out->current_ma = kNaN;
    out->uptime_s = kNaN;
    out->emg_input = IAD_UNKNOWN;
    out->drive_power_cut = IAD_UNKNOWN;
    out->enable_off = IAD_UNKNOWN;
}

namespace {

void to_c(const LimitRange& r, iad_range* out) {
    std::memset(out, 0, sizeof *out);
    out->min = num(r.min);
    out->max = num(r.max);
    out->unit_max = num(r.unit_max);
    out->configured = num(r.configured);
    out->recommended_min = num(r.recommended_min);
    out->available = tri(r.available);
    out->count = inum(r.count);
    copy_text(out->unit, r.unit);
}

}  // namespace

void to_c(const AxisRanges& r, iad_limits* out) {
    std::memset(out, 0, sizeof *out);
    to_c(r.position, &out->position);
    to_c(r.speed, &out->speed);
    to_c(r.accel, &out->accel);
    to_c(r.decel, &out->decel);
    to_c(r.accel_decel, &out->accel_decel);
    to_c(r.push, &out->push);
    to_c(r.band, &out->band);
    to_c(r.push_band, &out->push_band);
    to_c(r.stored_rows, &out->stored_rows);
    to_c(r.jog_step, &out->jog_step);
    to_c(r.speed_override, &out->speed_override);
    copy_text(out->units, r.units);
    copy_text(out->kind, r.kind);
    copy_text(out->family, r.family);
    out->push_speed = num(r.push.speed);
}

void to_c(const DeviceInfo& d, iad_identity* out) {
    std::memset(out, 0, sizeof *out);
    out->soft_min_mm = d.soft_limits_mm ? d.soft_limits_mm->min : kNaN;
    out->soft_max_mm = d.soft_limits_mm ? d.soft_limits_mm->max : kNaN;
    out->max_speed_mm_s = num(d.max_speed_mm_s);
    out->max_accel_g = num(d.max_accel_g);
    out->max_decel_g = num(d.max_decel_g);
    out->slave = d.slave;
    out->system_code = inum(d.system_code);
    out->unit_code = inum(d.unit_code);
    out->run_status = inum(d.run_status);
    out->axis_number = inum(d.axis_number);
    out->max_positions = inum(d.max_positions);
    out->actuator_kind = inum(d.actuator_kind);
    out->unit_known = tri(d.unit_known);
    out->rotary = tri(d.rotary());
    copy_text(out->product_family, d.product_family.value_or(""));
    copy_text(out->model, d.model());
    copy_text(out->model_name, d.model_name.value_or(""));
    copy_text(out->controller_name, d.controller_name.value_or(""));
    copy_text(out->controller, d.controller());
    copy_text(out->unit_name, d.unit_name.value_or(""));
    copy_text(out->system_name, d.system_name.value_or(""));
    copy_text(out->firmware, d.firmware.value_or(""));
    copy_text(out->serial_number, d.serial_number.value_or(""));
    copy_text(out->actuator_serial, d.actuator_serial.value_or(""));
    copy_text(out->user_name, d.user_name.value_or(""));
    copy_text(out->units, d.units);
    copy_text(out->kind, d.kind());
    copy_text(out->identity_source, d.identity_source.value_or(""));
}

namespace {

/// The fields of a position row, shared by iad_position_entry and (0.8) iad_position_write_result.
template <class C>
void fill_row(const PositionEntry& e, C* out) {
    out->position_mm = e.position_mm;
    out->band_mm = e.band_mm;
    out->speed_mm_s = e.speed_mm_s;
    out->zone_plus_mm = e.zone_plus_mm;
    out->zone_minus_mm = e.zone_minus_mm;
    out->accel_g = e.accel_g;
    out->decel_g = e.decel_g;
    out->push_power = e.push_power;
    out->load_threshold = e.load_threshold;
    out->control_flags = e.control_flags;
    out->checksum = inum(e.checksum);
    out->checksum_ok = tri(e.checksum_ok());
    out->is_push = flag(e.is_push());
    out->incremental = flag(e.incremental());
}

}  // namespace

void to_c(const PositionEntry& e, iad_position_entry* out) {
    iad_position_entry c = zeroed<iad_position_entry>();
    fill_row(e, &c);
    write_sized(out, c);  // (0.8) a sized struct: as much as the caller's struct_size says
}

void to_c(const ParameterInfo& p, iad_parameter* out) {
    std::memset(out, 0, sizeof *out);
    out->value = num(p.value.number);
    out->min = num(p.min);
    out->max = num(p.max);
    out->number = p.number;
    out->writable = flag(p.writable);
    out->dangerous = flag(p.dangerous);
    copy_text(out->name, p.name);
    copy_text(out->symbol, p.symbol.value_or(""));
    copy_text(out->item, p.item);
    copy_text(out->address, p.address.value_or(""));
    copy_text(out->text, p.text ? *p.text : p.value.to_string());
    copy_text(out->unit, p.unit);
    copy_text(out->raw, p.raw.value_or(""));
    copy_text(out->family, p.family);
}

void to_c(const ParameterWriteResult& r, iad_param_write_result* out) {
    std::memset(out, 0, sizeof *out);
    out->number = r.number;
    out->changed = flag(r.changed);
    out->verified = flag(r.verified);
    out->committed = flag(r.committed);
    out->commit_confirmed = flag(r.commit_confirmed);
    out->saved = flag(r.saved());
    out->skipped = flag(r.skipped);
    out->forced = flag(r.forced);
    out->listed_for_model = flag(r.listed_for_model);
    out->verified_for_model = flag(r.verified_for_model);
    out->home_parameter = flag(r.home_parameter);
    out->warning_count = static_cast<std::int32_t>(r.warnings.size());
    copy_text(out->key, r.key);
    copy_text(out->address, r.address);
    copy_text(out->name, r.name);
    copy_text(out->before, r.before);
    copy_text(out->after, r.after);
    copy_text(out->unit, r.unit);
}

void to_c(const PositionWriteResult& r, iad_position_write_result* out) {
    std::memset(out, 0, sizeof *out);
    fill_row(r.entry, out);  // (0.8) the row's fields in the result itself (no embedded input struct)
    out->row = r.row;
    out->skipped = flag(r.skipped);
    out->forced = flag(r.forced);
    out->warning_count = static_cast<std::int32_t>(r.warnings.size());
}

void to_c(const SoftwareResetResult& r, iad_reset_result* out) {
    std::memset(out, 0, sizeof *out);
    to_c(r.status, &out->status);
    out->uptime_before_s = num(r.uptime_before_s);
    out->uptime_after_s = num(r.uptime_after_s);
    out->elapsed_s = r.elapsed_s;
    out->accepted = flag(r.accepted);
    out->reply_lost = flag(r.reply_lost);
    out->confirmed = flag(r.confirmed);
    out->baud_rate = r.baud_rate.value_or(0);
    out->baud_changed = flag(r.baud_changed);
    out->needs_home = flag(r.needs_home());
    copy_text(out->confirmed_by, r.confirmed_by);
}

void to_c(const BaudChangeResult& r, iad_baud_result* out) {
    std::memset(out, 0, sizeof *out);
    out->previous = r.previous;
    out->baud_rate = r.baud_rate;
    out->method = r.method == BaudChangeMethod::Break ? IAD_BAUD_BREAK : IAD_BAUD_DIRECT;
    out->code = r.code;
    out->verified = flag(r.verified);
    out->probed = flag(r.probed);
    out->code_verified = flag(r.code_verified);
    copy_text(out->frame, r.frame);
}

bool fill_cell_results(const std::vector<std::string>& names, const std::map<std::string, Status>& ok,
                       const std::map<std::string, std::exception_ptr>& errors, iad_cell_result* results,
                       std::size_t capacity, std::size_t* count) {
    if (count) *count = names.size();
    for (std::size_t i = 0; results && i < names.size() && i < capacity; ++i) {
        iad_cell_result* r = &results[i];
        std::memset(r, 0, sizeof *r);
        r->index = static_cast<std::int32_t>(i);  // (0.8) the position in the requested list (cell order for all)
        copy_text(r->axis, names[i]);
        auto e = errors.find(names[i]);
        auto s = ok.find(names[i]);
        if (e != errors.end()) {
            unknown_status(&r->status);
            r->code = error_code(e->second);
            try {
                std::rethrow_exception(e->second);
            } catch (const std::exception& ex) {
                copy_text(r->message, ex.what());
            } catch (...) {
                copy_text(r->message, "unknown exception");
            }
        } else if (s != ok.end()) {
            to_c(s->second, &r->status);
            r->code = IAD_OK;
        } else {
            unknown_status(&r->status);
            r->code = IAD_OK;
        }
    }
    return !errors.empty();
}

}  // namespace capi
}  // namespace iai_actuator_driver
