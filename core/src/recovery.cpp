// Power-loss and link-loss recovery (recovery.hpp): the per-unit bookkeeping (session, link state, pending target),
// detection (classify_failure, restart_signs, check_link, LinkMonitor) and recover(). Port of the Python edition's
// recovery.py (the source of truth): the same checks before any frame, the same order (the same unit is confirmed
// before anything is written; the boot rate is probed first after a power loss, then the session's rate), the same
// messages and notes. Never starts motion. UNVERIFIED on hardware in this edition (simulator only); the behaviour it
// ports is VERIFIED in the Python edition where recovery.hpp says so. Unofficial: not an IAI product.
#include "iai_actuator_driver/recovery.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "axis_impl.hpp"
#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "monotonic_cv.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace recovery {

namespace {

constexpr const char* kLog = "iai_actuator_driver.recovery";
constexpr std::int64_t kUptimeWrapMs = std::int64_t(1) << 32;
constexpr std::int64_t kHourMs = 3600000;
constexpr int kSerialWords = 8;
/// Words a backup row needs to be decoded (the 15 writable words; the checksum is optional).
constexpr std::size_t kPositionWordsWritable = 15;

// ------------------------------------------------------------------------------------------ registry

/// The bookkeeping of one unit (bus + slave). Python: the weak-key dictionaries _SESSIONS / _STATES / _PENDING_TARGETS.
struct Entry {
    std::weak_ptr<ModbusRtu> bus;           ///< expired: the entry is stale and dropped
    std::shared_ptr<Session> session;
    std::string state;                       ///< "" = never set ("ok")
    std::optional<double> pending_target;    ///< a target noted before a session existed
};

using Key = std::pair<const ModbusRtu*, int>;

std::recursive_mutex& registry_mutex() {
    static std::recursive_mutex m;
    return m;
}

std::map<Key, Entry>& registry() {
    static std::map<Key, Entry> entries;
    return entries;
}

/// Drop the entries of destroyed buses (call with the registry mutex held).
void prune() {
    auto& entries = registry();
    for (auto it = entries.begin(); it != entries.end();) {
        if (it->second.bus.expired()) it = entries.erase(it);
        else ++it;
    }
}

/// The entry of a unit, nullptr when none (call with the registry mutex held).
Entry* find_entry(const ModbusRtu* bus, int slave) {
    prune();
    auto it = registry().find(Key(bus, slave));
    return it == registry().end() ? nullptr : &it->second;
}

/// The entry of a unit, created when missing (call with the registry mutex held).
Entry& entry_for(const std::shared_ptr<ModbusRtu>& bus, int slave) {
    prune();
    Entry& e = registry()[Key(bus.get(), slave)];
    if (e.bus.expired()) e.bus = bus;
    return e;
}

Entry& entry_for(const Axis& axis) { return entry_for(axis.bus(), axis.slave()); }

void set_state_quietly(const Axis& axis, const std::string& state) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    entry_for(axis).state = state;
}

// ------------------------------------------------------------------------------------------ small helpers

bool is_lost(const std::string& state) { return state == kPortVanished || state == kUnitSilent; }

double round3(double value) { return std::round(value * 1000.0) / 1000.0; }

std::string type_name(const std::exception& e) {
    if (dynamic_cast<const UnitReplacedError*>(&e)) return "UnitReplacedError";
    if (dynamic_cast<const RecoveryError*>(&e)) return "RecoveryError";
    return detail::error_type_name(e);
}

std::string describe(const std::exception& e) { return type_name(e) + ": " + e.what(); }

bool is_link_error(const std::exception& e) {
    return dynamic_cast<const TransportError*>(&e) || dynamic_cast<const ModbusTimeout*>(&e) ||
           dynamic_cast<const ModbusCrcError*>(&e) || dynamic_cast<const ModbusFrameError*>(&e);
}

/// Python repr() of an identity value: 'text' or the number.
std::string repr(const std::optional<std::string>& value) { return value ? "'" + *value + "'" : "None"; }
std::string repr(const std::optional<int>& value) { return value ? std::to_string(*value) : "None"; }

std::string span_text(const Span& s) { return "(" + util::g(s.min) + ", " + util::g(s.max) + ")"; }

double wall_time() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/// Sleep up to `seconds` without passing `deadline`; false when the deadline has already passed.
bool sleep_until_deadline(double seconds, double deadline) {
    const double remaining = deadline - now_s();
    if (remaining <= 0.0) return false;
    sleep_at_least(std::min(seconds, remaining));
    return true;
}

void emit(const EventCallback& on_event, const std::string& kind, const std::string& axis, const std::string& message,
          const std::string& cause) {
    RecoveryEvent event;
    event.kind = kind;
    event.axis = axis;
    event.message = message;
    event.cause = cause;
    event.time = wall_time();
    const std::string line = (axis.empty() ? std::string("axis") : axis) + ": " + kind + ": " + message;
    if (kind == kEventRecovered) log_info(kLog, line);
    else log_warning(kLog, line);
    if (!on_event) return;
    try {
        on_event(event);
    } catch (const std::exception& e) {
        log_error(kLog, std::string("recovery event callback raised: ") + e.what());  // never breaks the recovery
    } catch (...) {
        log_error(kLog, "recovery event callback raised");
    }
}

std::string default_name(const Axis& axis) {
    {
        std::lock_guard<std::recursive_mutex> lock(registry_mutex());
        const Entry* e = find_entry(axis.bus().get(), axis.slave());
        if (e && e->session && !e->session->name.empty()) return e->session->name;
    }
    const std::string port = port_name(axis);
    return port.empty() ? "slave " + std::to_string(axis.slave()) : port + ":" + std::to_string(axis.slave());
}

std::optional<std::int64_t> uptime_ms(const Status& st) {
    if (!st.raw.stim) return std::nullopt;
    return static_cast<std::int64_t>(*st.raw.stim);
}

/// Every alarm-history slot (empty ones too) as comparable keys; nullopt when the unit has no readable history. A link
/// error is thrown.
std::optional<std::vector<AlarmKey>> read_history(Axis& axis) {
    std::vector<AlarmRecord> records;
    try {
        records = axis.read_alarm_history(true);
    } catch (const std::exception& e) {
        if (is_link_error(e) || dynamic_cast<const CallerAbortError*>(&e)) throw;
        if (!dynamic_cast<const EcDriverError*>(&e) && !dynamic_cast<const std::out_of_range*>(&e) &&
            !dynamic_cast<const std::invalid_argument*>(&e))
            throw;
        log_info(kLog, std::string("alarm history not readable: ") + e.what());
        return std::nullopt;
    }
    std::vector<AlarmKey> keys;
    for (const auto& r : records) keys.push_back(AlarmKey{r.code, r.detail, r.address, r.time_ms});
    return keys;
}

std::vector<int> row_numbers(Axis& axis, const RowSelection& rows) {
    std::vector<int> out;
    if (rows.all) {
        const int n = axis.max_positions();
        for (int i = 0; i < n; ++i) out.push_back(i);
        return out;
    }
    std::set<int> unique(rows.rows.begin(), rows.rows.end());
    return std::vector<int>(unique.begin(), unique.end());
}

std::map<int, PositionEntry> read_rows(Axis& axis, const RowSelection& rows) {
    std::map<int, PositionEntry> out;
    for (int n : row_numbers(axis, rows)) out[n] = axis.read_position_table(n);
    return out;
}

/// {"0x0008:8.15": raw hex} of every item of kParameterClasses the unit's directory has (FC03 only).
std::map<std::string, std::string> read_parameters(Axis& axis) {
    std::unique_ptr<UnitReader> reader = axis.bus()->reader(axis.slave());
    UnitDirectory directory;
    const std::optional<DeviceInfo> info = axis.device_info();
    if (info && !info->categories.empty()) directory = UnitDirectory(info->index_header, info->categories);
    else directory = UnitDirectory::read(*reader);
    std::map<std::string, std::string> out;
    for (int class_id : kParameterClasses) {
        if (!directory.find(class_id)) continue;  // not in this unit's directory
        const ClassBlock block = directory.read_class(*reader, class_id);
        for (const auto& item : block.items) out[util::format("0x%04X:", class_id) + item.key()] = item.hex();
    }
    return out;
}

// ---- backups (the info::backup / Python info.backup layout)

std::optional<std::string> json_string(const Json* j) {
    if (!j || j->type() != Json::Type::String) return std::nullopt;
    return j->as_string();
}

/// A whole number in the int range, else nullopt like a value of another type (a fraction or 1e10 is never narrowed:
/// converting a double outside int is undefined).
std::optional<int> json_int(const Json* j) {
    const std::optional<long long> n = j ? j->as_whole() : std::nullopt;
    if (!n || *n < INT_MIN || *n > INT_MAX) return std::nullopt;
    return static_cast<int>(*n);
}

std::optional<double> json_number(const Json* j) {
    if (!j || (j->type() != Json::Type::Int && j->type() != Json::Type::Number)) return std::nullopt;
    return j->as_number();
}

std::optional<Span> json_span(const Json* j) {
    if (!j) return std::nullopt;
    if (j->type() == Json::Type::Array && j->size() == 2) {
        const auto& a = j->as_array();
        const auto lo = json_number(&a[0]);
        const auto hi = json_number(&a[1]);
        if (lo && hi) return Span{*lo, *hi};
    }
    if (j->type() == Json::Type::Object) {
        const auto lo = json_number(j->find("min"));
        const auto hi = json_number(j->find("max"));
        if (lo && hi) return Span{*lo, *hi};
    }
    return std::nullopt;
}

const Json* backup_identity(const Json& backup) {
    const Json* ident = backup.find("identity");
    return ident && ident->type() == Json::Type::Object ? ident : nullptr;
}

std::map<int, PositionEntry> backup_rows(const Json& backup) {
    std::map<int, PositionEntry> rows;
    const Json* positions = backup.find("positions");
    const Json* list = positions ? positions->find("rows") : nullptr;
    if (!list || list->type() != Json::Type::Array) return rows;
    for (const Json& row : list->as_array()) {
        const auto n = json_int(row.find("n"));
        const Json* words = row.find("words");
        if (!n || !words || words->type() != Json::Type::Array) continue;
        std::vector<std::uint16_t> values;
        for (const Json& w : words->as_array()) {
            const auto text = json_string(&w);
            if (!text) continue;
            const auto v = util::parse_int(*text, 16);
            if (v) values.push_back(static_cast<std::uint16_t>(*v & 0xFFFF));
        }
        if (values.size() >= kPositionWordsWritable) rows[*n] = PositionEntry::from_registers(values);
    }
    return rows;
}

std::map<std::string, std::string> backup_parameters(const Json& backup) {
    std::map<std::string, std::string> out;
    const Json* parameters = backup.find("parameters");
    const Json* classes = parameters ? parameters->find("classes") : nullptr;
    if (!classes || classes->type() != Json::Type::Array) return out;
    for (const Json& block : classes->as_array()) {
        const int class_id = json_int(block.find("class_id")).value_or(-1);
        if (std::find(std::begin(kParameterClasses), std::end(kParameterClasses), class_id) ==
            std::end(kParameterClasses))
            continue;
        const Json* items = block.find("items");
        if (!items || items->type() != Json::Type::Array) continue;
        for (const Json& item : items->as_array()) {
            const auto key = json_string(item.find("key"));
            const auto hex = json_string(item.find("hex"));
            if (key && hex) out[util::format("0x%04X:", class_id) + *key] = util::upper(*hex);
        }
    }
    return out;
}

Json read_json_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::stringstream text;
    text << in.rdbuf();
    return Json::parse(text.str());
}

// ---- the session from what the unit tells

void fill_identity(Session& s, const DeviceInfo& info) {
    s.serial_number = info.serial_number;
    s.model_name = info.model_name;
    s.system_code = info.system_code;
    s.unit_code = info.unit_code;
    s.firmware = info.firmware;
    s.product_family = info.product_family;
    s.soft_limits_mm = info.soft_limits_mm;
    s.max_speed_mm_s = info.max_speed_mm_s;
    s.units = info.units;
}

Identity identity_of(const DeviceInfo& info) {
    Identity id;
    id.serial_number = info.serial_number;
    id.model_name = info.model_name;
    id.system_code = info.system_code;
    id.unit_code = info.unit_code;
    id.firmware = info.firmware;
    return id;
}

/// A healthy reading: move the volatile baseline forward (call with the session mutex held).
void refresh(Session& sess, const Axis& axis, const Status& st) {
    sess.uptime_ms = uptime_ms(st);
    sess.uptime_at = now_s();
    if (st.modbus_enabled) sess.modbus_control = true;
    sess.safety_speed = st.safety_speed();
    sess.servo_on = st.servo_on;
    if (st.homed) sess.homed = true;
    if (const auto rate = axis.baud_rate()) sess.baud_rate = rate;
}

}  // namespace

// ================================================================================================ names and events

const std::vector<std::string>& link_states() {
    static const std::vector<std::string> states{kLinkOk,        kPortVanished, kUnitSilent,
                                                 kUnitRestarted, kUnitReplaced, kRecovering};
    return states;
}

const std::vector<std::string>& event_kinds() {
    static const std::vector<std::string> kinds{kEventLinkLost, kEventUnitRestarted, kEventUnitReplaced,
                                                kEventRecovered};
    return kinds;
}

Json RecoveryEvent::to_json() const {
    Json j = Json::object();
    j.set("kind", kind);
    j.set("axis", axis.empty() ? Json() : Json(axis));
    j.set("message", message);
    j.set("cause", cause.empty() ? Json() : Json(cause));
    j.set("time", time);
    return j;
}

Json Identity::to_json() const {
    Json j = Json::object();
    j.set("serial_number", serial_number);
    j.set("model_name", model_name);
    j.set("system_code", system_code);
    j.set("unit_code", unit_code);
    j.set("firmware", firmware);
    return j;
}

bool Identity::operator==(const Identity& o) const {
    return serial_number == o.serial_number && model_name == o.model_name && system_code == o.system_code &&
           unit_code == o.unit_code && firmware == o.firmware;
}

// ================================================================================================ session

Identity Session::identity() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    Identity id;
    id.serial_number = serial_number;
    id.model_name = model_name;
    id.system_code = system_code;
    id.unit_code = unit_code;
    id.firmware = firmware;
    return id;
}

Json Session::to_json() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    Json j = Json::object();
    j.set("name", name.empty() ? Json() : Json(name));
    j.set("slave", slave);
    j.set("port", port.empty() ? Json() : Json(port));
    j.set("baudrate", baud_rate);
    j.set("serial_number", serial_number);
    j.set("model_name", model_name);
    j.set("system_code", system_code);
    j.set("unit_code", unit_code);
    j.set("firmware", firmware);
    j.set("product_family", product_family);
    j.set("soft_limits_mm", soft_limits_mm ? iai_actuator_driver::to_json(*soft_limits_mm) : Json());
    j.set("max_speed_mm_s", max_speed_mm_s);
    j.set("units", units);
    j.set("modbus_control", modbus_control);
    j.set("safety_speed", safety_speed);
    j.set("servo_on", servo_on);
    j.set("homed", homed);
    j.set("uptime_ms", uptime_ms ? Json(static_cast<long long>(*uptime_ms)) : Json());
    j.set("uptime_at", uptime_at);
    if (alarm_history) {
        Json list = Json::array();
        for (const auto& a : *alarm_history) {
            Json one = Json::array();
            one.push_back(a.code);
            one.push_back(a.detail);
            one.push_back(a.address);
            one.push_back(a.time_ms ? Json(static_cast<long long>(*a.time_ms)) : Json());
            list.push_back(std::move(one));
        }
        j.set("alarm_history", std::move(list));
    } else {
        j.set("alarm_history", Json());
    }
    j.set("last_target_mm", last_target_mm);
    if (rows) {
        Json r = Json::object();
        for (const auto& kv : *rows) r.set(std::to_string(kv.first), iai_actuator_driver::to_json(kv.second));
        j.set("rows", std::move(r));
    } else {
        j.set("rows", Json());
    }
    if (parameters) {
        Json p = Json::object();
        for (const auto& kv : *parameters) p.set(kv.first, kv.second);
        j.set("parameters", std::move(p));
    } else {
        j.set("parameters", Json());
    }
    j.set("taken_at", taken_at);
    j.set("recovering", recovering.load());
    j.set("has_backup", backup.has_value());
    return j;
}

std::shared_ptr<Session> track(Axis& axis, const TrackOptions& options) {
    const DeviceInfo info = options.info ? *options.info : axis.identify();
    const Status st = axis.status();
    std::optional<std::vector<AlarmKey>> history;
    if (options.alarm_history) history = read_history(axis);
    std::optional<double> pending;
    std::shared_ptr<Session> previous;
    {
        std::lock_guard<std::recursive_mutex> lock(registry_mutex());
        Entry& e = entry_for(axis);
        pending = e.pending_target;
        e.pending_target.reset();
        previous = e.session;
    }
    auto sess = std::make_shared<Session>();
    sess->name = !options.name.empty() ? options.name : (previous ? previous->name : std::string());
    sess->slave = axis.slave();
    sess->port = port_name(axis);
    sess->baud_rate = axis.baud_rate();
    fill_identity(*sess, info);
    sess->modbus_control = st.modbus_enabled;
    sess->safety_speed = st.safety_speed();
    sess->servo_on = st.servo_on;
    sess->homed = st.homed;
    sess->uptime_ms = uptime_ms(st);
    sess->uptime_at = now_s();
    sess->alarm_history = history;
    if (pending) {
        sess->last_target_mm = pending;
    } else if (previous) {
        std::lock_guard<std::recursive_mutex> lock(previous->mutex);
        sess->last_target_mm = previous->last_target_mm;
    }
    if (options.rows.selected()) sess->rows = read_rows(axis, options.rows);
    if (options.parameters) sess->parameters = read_parameters(axis);
    if (options.backup) sess->backup = options.backup;
    {
        std::lock_guard<std::recursive_mutex> lock(registry_mutex());
        Entry& e = entry_for(axis);
        e.session = sess;
        e.state = kLinkOk;
    }
    return sess;
}

std::shared_ptr<Session> session_from_info(const Axis& axis, const DeviceInfo& info, const std::string& name) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    Entry& e = entry_for(axis);
    if (e.session) return e.session;
    auto sess = std::make_shared<Session>();
    sess->name = name;
    sess->slave = axis.slave();
    sess->port = port_name(axis);
    sess->baud_rate = axis.baud_rate();
    fill_identity(*sess, info);
    sess->last_target_mm = e.pending_target;
    e.pending_target.reset();
    e.session = sess;
    return sess;
}

std::shared_ptr<Session> session_from_backup(const Axis& axis, const Json& backup, const std::string& name) {
    const Json* ident = backup_identity(backup);
    const auto serial = json_string(ident ? ident->find("serial_number") : nullptr);
    const auto model = json_string(ident ? ident->find("model_name") : nullptr);
    const auto system = json_int(ident ? ident->find("system_code") : nullptr);
    const auto unit = json_int(ident ? ident->find("unit_code") : nullptr);
    if (!serial && !model && !system && !unit)
        throw std::invalid_argument("the backup holds no identity (serial number, model, system / unit code)");
    auto sess = std::make_shared<Session>();
    sess->name = name;
    sess->slave = axis.slave();
    sess->port = port_name(axis);
    sess->baud_rate = axis.baud_rate();
    sess->serial_number = serial;
    sess->model_name = model;
    sess->system_code = system;
    sess->unit_code = unit;
    sess->firmware = json_string(ident->find("firmware"));
    sess->product_family = json_string(ident->find("product_family"));
    sess->soft_limits_mm = json_span(ident->find("soft_limits_mm"));
    sess->max_speed_mm_s = json_number(ident->find("max_speed_mm_s"));
    const auto units = json_string(ident->find("units"));
    sess->units = units && !units->empty() ? *units : "mm";
    sess->backup = backup;
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    Entry& e = entry_for(axis);
    sess->last_target_mm = e.pending_target;
    e.pending_target.reset();
    e.session = sess;
    e.state = kLinkOk;
    return sess;
}

std::shared_ptr<Session> session_from_backup_file(const Axis& axis, const std::string& path, const std::string& name) {
    return session_from_backup(axis, read_json_file(path), name);
}

std::shared_ptr<Session> session_of(const Axis& axis) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    const Entry* e = find_entry(axis.bus().get(), axis.slave());
    return e ? e->session : nullptr;
}

void forget(const Axis& axis) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    prune();
    registry().erase(Key(axis.bus().get(), axis.slave()));
}

void update_session(const Axis& axis, const Json& fields) {
    const std::shared_ptr<Session> sess = session_of(axis);
    if (!sess) throw EcNotReadyError("no recovery session: call recovery::track(axis) first");
    if (fields.type() != Json::Type::Object) throw std::invalid_argument("the session fields are a JSON object");
    auto wrong = [](const std::string& key, const char* what) {
        return std::invalid_argument("session field '" + key + "' must be " + what);
    };
    auto number = [&](const std::string& key, const Json& v) {
        if (v.type() != Json::Type::Int && v.type() != Json::Type::Number) throw wrong(key, "a number");
        const double x = v.as_number();
        if (!std::isfinite(x)) throw wrong(key, "a finite number");
        return x;
    };
    auto tri = [&](const std::string& key, const Json& v) -> std::optional<bool> {
        if (v.is_null()) return std::nullopt;
        if (v.type() != Json::Type::Bool) throw wrong(key, "true, false or null");
        return v.as_bool();
    };
    // every value checked before anything is written
    std::vector<std::function<void(Session&)>> writes;
    for (const auto& kv : fields.as_object()) {
        const std::string& key = kv.first;
        const Json& v = kv.second;
        if (key == "name") {
            if (v.type() != Json::Type::String && !v.is_null()) throw wrong(key, "a string or null");
            const std::string name = v.is_null() ? std::string() : v.as_string();
            writes.push_back([name](Session& s) { s.name = name; });
        } else if (key == "baudrate") {
            // a whole number in 1..INT_MAX (9600.0 too); a fraction or 3e9 is refused, never narrowed
            const std::optional<long long> whole = v.is_null() ? std::nullopt : v.as_whole();
            if (!v.is_null() && (!whole || *whole <= 0 || *whole > INT_MAX))
                throw wrong(key, "a whole positive rate (at most 2147483647) or null");
            const std::optional<int> rate = whole ? std::optional<int>(static_cast<int>(*whole)) : std::nullopt;
            writes.push_back([rate](Session& s) { s.baud_rate = rate; });
        } else if (key == "modbus_control" || key == "safety_speed" || key == "servo_on" || key == "homed") {
            const std::optional<bool> value = tri(key, v);
            std::optional<bool> Session::*member = key == "modbus_control" ? &Session::modbus_control
                                                   : key == "safety_speed" ? &Session::safety_speed
                                                   : key == "servo_on"     ? &Session::servo_on
                                                                           : &Session::homed;
            writes.push_back([value, member](Session& s) { s.*member = value; });
        } else if (key == "uptime_ms") {
            const std::optional<long long> whole = v.is_null() ? std::nullopt : v.as_whole();
            if (!v.is_null() && !whole) throw wrong(key, "a whole number of milliseconds (64-bit) or null");
            const std::optional<std::int64_t> up =
                whole ? std::optional<std::int64_t>(static_cast<std::int64_t>(*whole)) : std::nullopt;
            writes.push_back([up](Session& s) { s.uptime_ms = up; });
        } else if (key == "uptime_age_s") {
            const std::optional<double> age = v.is_null() ? std::nullopt : std::optional<double>(number(key, v));
            writes.push_back([age](Session& s) {
                s.uptime_at = age ? std::optional<double>(now_s() - *age) : std::nullopt;
            });
        } else if (key == "last_target_mm") {
            const std::optional<double> target = v.is_null() ? std::nullopt : std::optional<double>(number(key, v));
            writes.push_back([target](Session& s) { s.last_target_mm = target; });
        } else if (key == "recovering") {
            if (v.type() != Json::Type::Bool) throw wrong(key, "true or false");
            const bool on = v.as_bool();
            writes.push_back([on](Session& s) { s.recovering = on; });
        } else {
            throw std::invalid_argument("'" + key + "' is not a session field that can be written (name, baudrate, "
                                        "modbus_control, safety_speed, servo_on, homed, uptime_ms, uptime_age_s, "
                                        "last_target_mm, recovering)");
        }
    }
    std::lock_guard<std::recursive_mutex> lock(sess->mutex);
    for (const auto& write : writes) write(*sess);
}

void note_target(const Axis& axis, std::optional<double> target_mm) { hooks::note_target(axis.bus(), axis.slave(), target_mm); }

void note_control(const Axis& axis, const ControlChange& change) {
    hooks::note_control(axis.bus(), axis.slave(), change);
}

namespace hooks {

void note_target(const std::shared_ptr<ModbusRtu>& bus, int slave, std::optional<double> target_mm) noexcept {
    try {
        if (!bus) return;
        std::shared_ptr<Session> sess;
        {
            std::lock_guard<std::recursive_mutex> lock(registry_mutex());
            Entry& e = entry_for(bus, slave);
            if (!e.session) {
                e.pending_target = target_mm;
                return;
            }
            sess = e.session;
        }
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        sess->last_target_mm = target_mm;
    } catch (const std::exception& e) {
        log_debug(kLog, std::string("target not recorded: ") + e.what());
    } catch (...) {
    }
}

void note_control(const std::shared_ptr<ModbusRtu>& bus, int slave, const ControlChange& change) noexcept {
    try {
        if (!bus) return;
        std::shared_ptr<Session> sess;
        {
            std::lock_guard<std::recursive_mutex> lock(registry_mutex());
            const Entry* e = find_entry(bus.get(), slave);
            if (!e || !e->session) return;
            sess = e->session;
        }
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        if (change.modbus_control) sess->modbus_control = *change.modbus_control;
        if (change.safety_speed) sess->safety_speed = *change.safety_speed;
        if (change.baud_rate) sess->baud_rate = *change.baud_rate;
        if (change.homed) sess->homed = *change.homed;
    } catch (const std::exception& e) {
        log_debug(kLog, std::string("session not updated: ") + e.what());
    } catch (...) {
    }
}

}  // namespace hooks

std::string link_state(const Axis& axis) {
    std::lock_guard<std::recursive_mutex> lock(registry_mutex());
    const Entry* e = find_entry(axis.bus().get(), axis.slave());
    return e && !e->state.empty() ? e->state : std::string(kLinkOk);
}

bool set_link_state(const Axis& axis, const std::string& state, const std::string& message,
                    const EventCallback& on_event, const std::string& name) {
    const auto& states = link_states();
    if (std::find(states.begin(), states.end(), state) == states.end())
        throw std::invalid_argument("state must be one of (ok, port_vanished, unit_silent, unit_restarted, "
                                    "unit_replaced, recovering), got '" + state + "'");
    std::string previous;
    {
        std::lock_guard<std::recursive_mutex> lock(registry_mutex());
        Entry& e = entry_for(axis);
        previous = e.state.empty() ? std::string(kLinkOk) : e.state;
        if (previous == state) return false;
        e.state = state;
    }
    const std::string label = name.empty() ? default_name(axis) : name;
    const std::string text = message.empty() ? state : message;
    if (is_lost(state) && !is_lost(previous)) {
        emit(on_event, kEventLinkLost, label, text, state);
    } else if (state == kUnitRestarted) {
        emit(on_event, kEventUnitRestarted, label, text, state);
    } else if (state == kUnitReplaced) {
        emit(on_event, kEventUnitReplaced, label, text, state);
    } else if (state == kLinkOk && previous != kRecovering) {
        emit(on_event, kEventRecovered, label, message.empty() ? "link back after " + previous : message, previous);
    }
    return true;
}

// ================================================================================================ detection

std::string port_name(const Axis& axis) { return axis.bus()->name(); }

std::optional<bool> port_present(const Axis& axis, const PortLister& lister) {
    const std::shared_ptr<Transport> transport = axis.bus()->transport_ptr();
    if (!transport) return std::nullopt;
    const bool serial = dynamic_cast<const SerialTransport*>(transport.get()) != nullptr;
    try {
        if (!lister || !serial) {
            const std::optional<bool> own = transport->present();
            if (own || !lister) return own;
        }
        const std::string name = util::upper(transport->name());
        if (name.empty()) return std::nullopt;
        for (const auto& p : lister())
            if (util::upper(p.port) == name) return true;
        return false;
    } catch (const std::exception& e) {  // listing must never break a recovery
        log_debug(kLog, std::string("listing the ports failed: ") + e.what());
        return std::nullopt;
    }
}

std::string classify_failure(const std::exception& error, const Axis* axis, const PortLister& lister) {
    if (dynamic_cast<const UnitReplacedError*>(&error)) return kUnitReplaced;
    if (dynamic_cast<const TransportError*>(&error)) {
        // a bus closed by close() is not a link fault (the Python edition raises a plain ModbusError there)
        if (axis && axis->bus()->closed()) return std::string();
        if (std::string(error.what()) == "bus is closed") return std::string();
        return kPortVanished;
    }
    if (dynamic_cast<const ModbusTimeout*>(&error) || dynamic_cast<const ModbusCrcError*>(&error) ||
        dynamic_cast<const ModbusFrameError*>(&error)) {
        if (axis && port_present(*axis, lister) == std::optional<bool>(false)) return kPortVanished;
        return kUnitSilent;
    }
    if (dynamic_cast<const EcDriverError*>(&error)) return std::string();  // a refused command, an alarm, a limit
    if (dynamic_cast<const std::system_error*>(&error)) return kPortVanished;  // an OS error on a dead handle
    return std::string();
}

std::string classify_failure(const std::exception_ptr& error, const Axis* axis, const PortLister& lister) {
    if (!error) return std::string();
    try {
        std::rethrow_exception(error);
    } catch (const std::exception& e) {
        return classify_failure(e, axis, lister);
    } catch (...) {
        return std::string();
    }
}

std::vector<std::string> restart_signs(const Session& session, const Status& status,
                                       const std::vector<AlarmKey>& new_records, std::optional<int> baud_rate_found,
                                       int boot_baud_rate, std::optional<double> now) {
    std::lock_guard<std::recursive_mutex> lock(session.mutex);
    std::vector<std::string> signs;
    const auto up = uptime_ms(status);
    const auto before = session.uptime_ms;
    if (up && before) {
        const bool wrapped = *before > kUptimeWrapMs - kHourMs && *up < kHourMs;
        if (!wrapped) {
            if (*up < *before) {
                signs.push_back(util::format("uptime counter dropped (%.1f s -> %.1f s)", *before / 1000.0,
                                             *up / 1000.0));
            } else if (session.uptime_at) {
                const double elapsed = now.value_or(now_s()) - *session.uptime_at;
                const double expected =
                    static_cast<double>(*before) + 1000.0 * (kUptimeRateMin * elapsed - kUptimeSlackS);
                if (static_cast<double>(*up) < expected)
                    signs.push_back(util::format("uptime counter %.1f s is behind the %.1f s it should show at least "
                                                 "(restarted while the PC was away)",
                                                 *up / 1000.0, expected / 1000.0));
            }
        }
    }
    for (const auto& rec : new_records)
        if (rec.code == reg::kAlarmPowerUp) {
            signs.push_back("new power-up record (0x0FFF) in the alarm history");
            break;
        }
    if (session.modbus_control.value_or(false) && !status.modbus_enabled)
        signs.push_back("Modbus control (PMSS) dropped while the PC held it");
    if (session.homed.value_or(false) && !status.homed && !status.homing()) signs.push_back("home (HEND) lost");
    if (baud_rate_found && session.baud_rate && *baud_rate_found != *session.baud_rate &&
        *baud_rate_found == boot_baud_rate)
        signs.push_back(util::format("baud rate fell back to %d (the power-up rate) from %d", boot_baud_rate,
                                     *session.baud_rate));
    return signs;
}

std::vector<AlarmKey> new_alarm_records(const std::optional<std::vector<AlarmKey>>& before,
                                        const std::optional<std::vector<AlarmKey>>& after) {
    std::vector<AlarmKey> out;
    if (!before || !after) return out;
    const std::size_t n = after->size();
    for (std::size_t k = 0; k <= n; ++k) {
        // the newest k slots are new when the rest equals `before` shifted by k
        const std::size_t keep = n - k;
        bool same = before->size() >= keep;
        for (std::size_t i = 0; same && i < keep; ++i) same = (*after)[k + i] == (*before)[i];
        if (same) {
            for (std::size_t i = 0; i < k; ++i)
                if ((*after)[i].code != 0) out.push_back((*after)[i]);
            return out;
        }
    }
    for (const auto& a : *after)
        if (a.code != 0) out.push_back(a);
    return out;
}

Json LinkCheck::to_json() const {
    Json j = Json::object();
    j.set("axis", axis.empty() ? Json() : Json(axis));
    j.set("state", state);
    j.set("ok", ok());
    Json r = Json::array();
    for (const auto& reason : reasons) r.push_back(reason);
    j.set("reasons", std::move(r));
    j.set("error", error_text.empty() ? Json() : Json(error_text));
    j.set("elapsed_s", elapsed_s);
    j.set("position_mm", status ? Json(status->position_mm) : Json());
    return j;
}

LinkCheck check_link(Axis& axis, const CheckLinkOptions& options) {
    const std::shared_ptr<Session> sess = options.session ? options.session : session_of(axis);
    std::string label = options.name;
    if (label.empty() && sess) {
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        label = sess->name;
    }
    if (label.empty()) label = default_name(axis);
    LinkCheck chk;
    chk.axis = label;
    if (sess && sess->recovering.load()) {
        chk.state = kRecovering;
        chk.reasons = {"recovery in progress"};
        return chk;
    }
    const double started = now_s();
    Status st;
    try {
        st = axis.status();
    } catch (const std::exception& e) {
        const std::string kind = classify_failure(e, &axis, options.lister);
        if (kind.empty()) throw;
        chk.state = kind;
        chk.reasons = {describe(e)};
        chk.error = std::current_exception();
        chk.error_text = describe(e);
        chk.elapsed_s = round3(now_s() - started);
        set_link_state(axis, kind, chk.reasons[0], options.on_event, label);
        return chk;
    }
    chk.elapsed_s = round3(now_s() - started);
    chk.status = st;
    if (!sess) {
        set_link_state(axis, kLinkOk, "", options.on_event, label);
        return chk;
    }
    std::vector<std::string> signs;
    {
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        signs = restart_signs(*sess, st);
        if (signs.empty()) refresh(*sess, axis, st);
    }
    if (!signs.empty()) {
        chk.state = kUnitRestarted;
        chk.reasons = signs;
        set_link_state(axis, kUnitRestarted, util::join(signs, "; "), options.on_event, label);
        return chk;
    }
    const std::string current = link_state(axis);
    if (current != kLinkOk && current != kUnitReplaced) {
        // the link answers again with no restart sign (e.g. a cable re-seated): still recover() to confirm the unit
        chk.state = current;
        chk.reasons = {"answers again: run recover() to confirm the unit"};
    }
    return chk;
}

// ================================================================================================ LinkMonitor

LinkMonitor::LinkMonitor(std::vector<std::pair<std::string, Axis*>> axes, Options options)
    : axes_(std::move(axes)),
      options_(std::move(options)),
      wake_(std::make_unique<::iai_actuator_driver::detail::MonotonicCv>()) {
    if (!(options_.interval_s > 0.0)) throw std::invalid_argument("interval_s must be > 0");
    for (const auto& kv : axes_)
        if (!kv.second) throw std::invalid_argument("axis '" + kv.first + "' is null");
}

LinkMonitor::LinkMonitor(Axis& axis, Options options)
    : LinkMonitor(std::vector<std::pair<std::string, Axis*>>{{default_name(axis), &axis}}, std::move(options)) {}

LinkMonitor::~LinkMonitor() { stop(); }

bool LinkMonitor::running() const { return running_.load(); }

std::map<std::string, LinkCheck> LinkMonitor::check_now() {
    std::lock_guard<std::mutex> pass(pass_mutex_);
    std::map<std::string, LinkCheck> out;
    for (const auto& kv : axes_) {
        const std::string& name = kv.first;
        LinkCheck chk;
        try {
            if (options_.check) {
                chk = options_.check(name, *kv.second);
            } else {
                CheckLinkOptions o;
                o.name = name;
                o.on_event = options_.on_event;
                chk = check_link(*kv.second, o);
            }
        } catch (const WaitCancelledError&) {
            throw;  // stop() ended the pass of the thread (run)
        } catch (const std::exception& e) {  // e.g. the bus was closed: keep monitoring the others
            std::lock_guard<std::mutex> lock(mutex_);
            errors_.push_back(name + ": " + describe(e));
            if (errors_.size() > 50) errors_.erase(errors_.begin(), errors_.end() - 50);
            continue;
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            errors_.push_back(name + ": unknown error");
            continue;
        }
        std::string previous = kLinkOk;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = states_.find(name);
            if (it != states_.end()) previous = it->second.state;
            states_[name] = chk;
        }
        out[name] = chk;
        if (previous != chk.state && options_.on_change) {
            try {
                options_.on_change(name, chk);
            } catch (const std::exception& e) {
                log_error(kLog, std::string("link monitor callback raised: ") + e.what());
            } catch (...) {
                log_error(kLog, "link monitor callback raised");
            }
        }
    }
    ++passes_;
    return out;
}

LinkMonitor& LinkMonitor::start() {
    if (running_.load()) return *this;
    if (thread_.joinable()) {
        if (thread_.get_id() == std::this_thread::get_id()) return *this;
        thread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = false;
    }
    running_ = true;
    try {
        thread_ = std::thread([this] { run(); });
    } catch (...) {  // not started: not running
        running_ = false;
        throw;
    }
    return *this;
}

void LinkMonitor::run() {
    // Every way out clears running_; an unexpected error ends the thread with the error in errors(), not the process.
    struct Exit {
        std::atomic<bool>& running;
        ~Exit() { running = false; }
    } done{running_};
    try {
        const CancelScope cancel_scope(cancel_, true);  // stop(): a check waiting for the bus ends at once
        for (;;) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stop_requested_) break;
            }
            try {
                check_now();
            } catch (const WaitCancelledError&) {
                break;  // stop() during the pass
            }
            std::unique_lock<std::mutex> lock(mutex_);
            wake_->wait_for(lock, std::chrono::duration<double>(options_.interval_s),
                            [this] { return stop_requested_; });
            if (stop_requested_) break;
        }
    } catch (...) {  // e.g. out of memory while keeping a result
        try {
            const std::string message =
                "link monitor stopped: " + ::iai_actuator_driver::detail::describe(std::current_exception());
            log_error(kLog, message);
            std::lock_guard<std::mutex> lock(mutex_);
            errors_.push_back(message);
            if (errors_.size() > 50) errors_.erase(errors_.begin(), errors_.end() - 50);
        } catch (...) {
        }
    }
}

void LinkMonitor::stop(double timeout_s) {
    (void)timeout_s;  // the join waits for a read on the wire (its response timeout); a wait for the bus ends at once
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
    }
    cancel_->cancel();
    wake_->notify_all();
    if (thread_.joinable()) {
        if (thread_.get_id() == std::this_thread::get_id()) thread_.detach();  // stop() from a callback of the thread
        else thread_.join();
    }
    running_ = false;
}

std::map<std::string, LinkCheck> LinkMonitor::states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_;
}

long LinkMonitor::passes() const { return passes_.load(); }

std::vector<std::string> LinkMonitor::errors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return errors_;
}

// ================================================================================================ report

bool RecoveryReport::fault_alarm() const { return status ? status->alarm : alarm_code != 0; }

bool RecoveryReport::needs_attention() const {
    return needs_home || fault_alarm() || at_target == std::optional<bool>(false) || !limits_changed.empty() ||
           !row_problems.empty() || !parameter_differences.empty() ||
           (servo_was_on.value_or(false) && !servo_on.value_or(false)) || (status && status->moving);
}

bool RecoveryReport::ready_to_move() const { return status && status->ready(); }

std::string RecoveryReport::summary() const {
    std::vector<std::string> parts{(axis.empty() ? std::string("axis") : axis) + ": recovered from " + cause};
    if (restarted) parts.push_back("unit restarted");
    if (port_reopened) parts.push_back("port reopened");
    if (baud_restored)
        parts.push_back(util::format("baud %d -> %d", baud_rate_found.value_or(0), baud_rate.value_or(0)));
    if (needs_home) parts.push_back("needs home");
    if (fault_alarm()) parts.push_back(util::format("alarm 0x%03X", alarm_code));
    if (status && status->moving) parts.push_back("still moving");
    if (at_target == std::optional<bool>(false))
        parts.push_back(util::format("%+.2f from the last target", position_error_mm.value_or(0.0)));
    return util::join(parts, ", ");
}

Json RecoveryReport::to_json() const {
    auto strings = [](const std::vector<std::string>& v) {
        Json a = Json::array();
        for (const auto& s : v) a.push_back(s);
        return a;
    };
    auto ints = [](const std::vector<int>& v) {
        Json a = Json::array();
        for (int i : v) a.push_back(i);
        return a;
    };
    Json j = Json::object();
    j.set("axis", axis.empty() ? Json() : Json(axis));
    j.set("cause", cause);
    j.set("restarted", restarted);
    j.set("restart_signs", strings(restart_signs));
    j.set("port_reopened", port_reopened);
    j.set("baudrate_found", baud_rate_found);
    j.set("baudrate", baud_rate);
    j.set("baud_restored", baud_restored);
    j.set("same_unit", same_unit);
    j.set("identity", identity.to_json());
    j.set("control_restored", control_restored);
    j.set("control_kept", control_kept);
    j.set("safety_speed_restored", safety_speed_restored);
    j.set("limits_changed", strings(limits_changed));
    j.set("alarm_code", alarm_code);
    j.set("alarm_name", alarm_name);
    j.set("alarm_level", alarm_level);
    j.set("alarms_cleared", ints(alarms_cleared));
    Json records = Json::array();
    for (const auto& r : new_alarm_records) {
        Json one = Json::object();
        one.set("code", r.code);
        one.set("hex", r.hex);
        one.set("name", r.name);
        one.set("time_ms", r.time_ms ? Json(static_cast<long long>(*r.time_ms)) : Json());
        records.push_back(std::move(one));
    }
    j.set("new_alarm_records", std::move(records));
    j.set("modbus_control", modbus_control);
    j.set("safety_speed", safety_speed);
    j.set("servo_on", servo_on);
    j.set("servo_was_on", servo_was_on);
    j.set("homed", homed);
    j.set("needs_home", needs_home);
    j.set("position_mm", position_mm);
    j.set("last_target_mm", last_target_mm);
    j.set("position_error_mm", position_error_mm);
    j.set("at_target", at_target);
    j.set("rows_checked", ints(rows_checked));
    j.set("row_problems", strings(row_problems));
    j.set("parameters_checked", parameters_checked);
    j.set("parameter_differences", strings(parameter_differences));
    j.set("actions", strings(actions));
    j.set("notes", strings(notes));
    j.set("elapsed_s", elapsed_s);
    j.set("status", status ? iai_actuator_driver::to_json(*status) : Json());
    j.set("fault_alarm", fault_alarm());
    j.set("needs_attention", needs_attention());
    j.set("ready_to_move", ready_to_move());
    j.set("summary", summary());
    return j;
}

// ================================================================================================ recover()

namespace {

/// One recovery: the state its steps share.
struct Run {
    Axis& axis;
    Session& sess;
    const RecoveryPolicy& policy;
    RecoveryReport& report;
    std::shared_ptr<RecoveryReport> report_ptr;
    double deadline;
    double timeout;
    const EventCallback& on_event;
    std::string label;
    std::string prior;

    std::vector<int> baud_order(bool boot_first) const {
        std::vector<int> first;
        const std::optional<int> session_rate = [this] {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            return sess.baud_rate;
        }();
        if (boot_first) {
            first.push_back(policy.boot_baud_rate);
            if (session_rate) first.push_back(*session_rate);
        } else {
            if (session_rate) first.push_back(*session_rate);
            first.push_back(policy.boot_baud_rate);
        }
        for (int r : kBaudDetectOrder) first.push_back(r);
        std::vector<int> out;
        for (int r : first)
            if (r > 0 && std::find(out.begin(), out.end(), r) == out.end()) out.push_back(r);
        return out;
    }

    [[noreturn]] void fail(const std::string& message) { throw RecoveryError(message, report_ptr); }

    /// Wait for the port, reopen it, and find the unit (its baud rate when the transport has one). Returns the rate.
    std::optional<int> reconnect(bool reopen) {
        ModbusRtu& bus = *axis.bus();
        const bool boot_first = report.cause != kPortVanished;  // a silent unit has likely lost power: 38400 first
        const std::vector<int> order = baud_order(boot_first);
        std::string last;
        bool waiting_noted = false;
        for (;;) {
            if (now_s() > deadline) {
                const std::string where = reopen && !sess.port.empty() ? "port " + sess.port : "the unit";
                const std::string why = !last.empty()
                                            ? "last: " + last
                                            : std::string("the port is still not listed: is the adapter plugged in?");
                fail(where + " did not come back within " + util::format("%g", timeout) + " s (" + why + ")");
            }
            if (reopen) {
                if (!bus.can_reopen())
                    fail("the port vanished and this transport cannot be reopened (" + bus.name() + ")");
                if (port_present(axis, policy.list_ports) == std::optional<bool>(false)) {
                    if (!waiting_noted) {
                        std::string shown = !sess.port.empty() ? sess.port : port_name(axis);
                        if (shown.empty()) shown = "?";
                        report.notes.push_back("waited for port " + shown + " to be listed again");
                        waiting_noted = true;
                    }
                    sleep_until_deadline(policy.poll_s, deadline);
                    continue;
                }
                try {
                    bus.reopen();
                } catch (const ModbusError& e) {
                    last = e.what();
                    sleep_until_deadline(policy.poll_s, deadline);
                    continue;
                }
                reopen = false;
                report.port_reopened = true;
                const std::string name = port_name(axis);
                report.actions.push_back("port " + (name.empty() ? std::string("(transport)") : name) + " reopened");
            }
            try {
                if (bus.can_change_baud_rate()) return bus.detect_baud_rate(axis.slave(), order, 0x9000, false);
                axis.status();
                return std::nullopt;
            } catch (const std::exception& e) {
                const std::string kind = classify_failure(e, &axis, policy.list_ports);
                if (kind.empty()) throw;
                last = e.what();
                if (kind == kPortVanished) reopen = true;
            }
            sleep_until_deadline(policy.poll_s, deadline);
        }
    }

    /// (same unit, or nullopt when that cannot be told yet; the differences; the fields the unit did not give).
    struct Comparison {
        std::optional<bool> same;
        std::vector<std::string> diffs;
        std::vector<std::string> missing;
    };

    Comparison compare_identity(const DeviceInfo& info) const {
        std::lock_guard<std::recursive_mutex> lock(sess.mutex);
        Comparison c;
        int compared = 0;
        auto text_field = [&](const char* key, const std::optional<std::string>& before,
                              const std::optional<std::string>& after) {
            if (!before) return;
            if (!after) {
                c.missing.push_back(key);
                return;
            }
            ++compared;
            if (*before != *after) c.diffs.push_back(std::string(key) + " " + repr(before) + " -> " + repr(after));
        };
        auto int_field = [&](const char* key, const std::optional<int>& before, const std::optional<int>& after) {
            if (!before) return;
            if (!after) {
                c.missing.push_back(key);
                return;
            }
            ++compared;
            if (*before != *after) c.diffs.push_back(std::string(key) + " " + repr(before) + " -> " + repr(after));
        };
        text_field("serial_number", sess.serial_number, info.serial_number);
        text_field("model_name", sess.model_name, info.model_name);
        int_field("system_code", sess.system_code, info.system_code);
        int_field("unit_code", sess.unit_code, info.unit_code);
        if (!c.diffs.empty()) {
            c.same = false;
            return c;
        }
        const bool serial_missing =
            sess.serial_number && std::find(c.missing.begin(), c.missing.end(), "serial_number") != c.missing.end();
        if (compared == 0 || serial_missing) return c;  // nullopt: not told yet
        c.same = true;
        return c;
    }

    DeviceInfo identify_same() {
        for (;;) {
            forget_directory();
            const DeviceInfo info = axis.identify();
            report.identity = identity_of(info);
            const Comparison c = compare_identity(info);
            if (c.same == std::optional<bool>(false)) {
                const std::string message =
                    "another unit answers at slave " + std::to_string(axis.slave()) +
                    (sess.port.empty() ? std::string() : " on " + sess.port) + ": " + util::join(c.diffs, "; ") +
                    ". Nothing was written to it: check the machine, then track() the new unit (and back it up)";
                set_link_state(axis, kUnitReplaced, message, on_event, label);
                throw UnitReplacedError(message, sess.identity(), report.identity, report_ptr);
            }
            if (c.same) {
                report.same_unit = true;
                std::lock_guard<std::recursive_mutex> lock(sess.mutex);
                if (sess.firmware && info.firmware && *sess.firmware != *info.firmware)
                    report.notes.push_back("firmware " + *sess.firmware + " -> " + *info.firmware +
                                           " (same serial number)");
                return info;
            }
            if (!sleep_until_deadline(policy.poll_s, deadline)) {
                const std::string what = c.missing.empty() ? "no identity was recorded" : util::join(c.missing, ", ");
                fail("the unit's identity could not be confirmed (" + what +
                     "): nothing restored; check it is the same unit");
            }
        }
    }

    /// FC11 and the identification block only (two reads): true when they show the session's unit (its serial number
    /// is required). Anything else (an error, a field that differs or is missing) is false: the full identify decides.
    /// READ-ONLY. Lets recover() put a unit found at a slow rate (9600 after a line break) back on the session's rate
    /// before the many reads of identify().
    /// The unit may have restarted or been replaced: its directory is read again by identify(), as the Python edition's
    /// identify() always does, not taken from what the axis read before (a line settled as having none stays so).
    void forget_directory() {
        detail::AxisAccess::Impl& impl = detail::AxisAccess::impl(axis);
        std::lock_guard<std::mutex> lock(impl.mutex);
        impl.directory.reset();
    }

    bool quick_same() {
        std::optional<std::string> serial, model;
        std::optional<int> system, unit;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            serial = sess.serial_number;
            model = sess.model_name;
            system = sess.system_code;
            unit = sess.unit_code;
        }
        if (!serial) return false;
        if (!axis.bus()->line()->rules().report_slave_id()) return false;  // RCON: no FC11 (the full identify decides)
        int address = reg::kIdentificationBase;
        if (const auto di = axis.device_info()) {
            if (!di->categories.empty()) {
                const IndexCategory* c = di->category(reg::kIdentificationCategory);
                if (!c) return false;
                address = c->data_address;
            }
        }
        std::vector<std::uint8_t> sid;
        std::vector<std::uint16_t> words;
        try {
            sid = axis.bus()->report_slave_id(axis.slave(), 0);
            if (sid.size() < 5) return false;
            words = axis.bus()->read_registers(axis.slave(), address, reg::kIdentificationWords);
        } catch (const CallerAbortError&) {
            throw;  // the caller's own error ends the call
        } catch (const EcDriverError&) {
            return false;
        } catch (const std::out_of_range&) {
            return false;
        } catch (const std::invalid_argument&) {
            return false;
        }
        auto text = [&words](int offset, int count) -> std::optional<std::string> {
            if (static_cast<int>(words.size()) < offset + count) return std::nullopt;
            return reg::words_to_text(std::vector<std::uint16_t>(words.begin() + offset, words.begin() + offset + count));
        };
        return text(reg::kIdentSerialOffset, kSerialWords) == serial &&
               (!model || text(reg::kIdentModelOffset, reg::kIdentModelWords) == model) &&
               (!system || *system == sid[0]) && (!unit || *unit == sid[2]);
    }

    /// The session's baud rate back, best effort (a failure becomes a note); nothing when the link is already at it.
    void restore_baud_rate() {
        ModbusRtu& bus = *axis.bus();
        std::optional<int> target;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            target = sess.baud_rate;
        }
        const std::optional<int> now = bus.baud_rate();
        if (policy.restore_baud_rate && target && now && *now != *target) {
            if (!bus.can_change_baud_rate()) {
                report.notes.push_back(util::format("baud rate not restored: this transport cannot change it (link at %d)",
                                                    *now));
            } else if (!baud_rate_verified(*target) && *target != policy.boot_baud_rate) {
                report.notes.push_back(util::format(
                    "baud rate not restored: %d baud has no verified baud-change code; the link stays at %d", *target,
                    *now));
            } else {
                // the boot rate's code is tried too (VERIFIED on the EC-GRBP8M gripper, ignored by the EC-R6S rod in
                // the Python edition): a unit that ignores it only leaves the link where it answers (a note below)
                try {
                    // nothing is broadcast behind a gateway (line.hpp): refused before anything is sent, as a note
                    bus.line()->require(LineOperation::BaudChange, "set_baudrate (broadcast 0x45 cmd 4)");
                    const Status st = axis.status();
                    if (!st.idle() || st.pushing())
                        throw EcNotReadyError("baud change only at rest (not moving, pushing, paused or homing)");
                    const BaudChangeResult result = change_baud_rate(bus, *target, {axis.slave()});
                    if (result.baud_rate != *target)
                        throw VerificationError(util::format(
                            "baud change to %d (code 0x%02X) not taken: the unit %s at %d baud (the port is left at %d)",
                            *target, result.code, result.baud_rate == result.previous ? "still answers" : "answers",
                            result.baud_rate, result.baud_rate));
                } catch (const CallerAbortError&) {
                    throw;  // the caller's own error ends the call
                } catch (const EcDriverError& e) {
                    const std::optional<int> at = bus.baud_rate();
                    report.notes.push_back(util::format("baud rate %d not restored (", *target) + describe(e) +
                                           util::format("); link at %d", at.value_or(0)));
                    report.baud_rate = bus.baud_rate();
                    return;
                }
                report.baud_restored = true;
                report.actions.push_back(
                    util::format("baud rate restored %d -> %d (a broadcast: every unit of this line)", *now, *target));
            }
        }
        report.baud_rate = bus.baud_rate();
    }

    /// Axis::set_safety_speed (the SFTY coil 0x0401, or DRG1 SFTY on the register path: RCON), read back.
    Status set_safety_speed(bool on) { return axis.set_safety_speed(on, true); }

    /// The session's baud rate, PC control and safety speed, each best effort (a failure becomes a note). The baud
    /// rate is skipped when baud_tried: the quick path already tried it (a second try would repeat the broadcast and
    /// the note).
    Status restore(bool baud_tried) {
        if (baud_tried)
            report.baud_rate = axis.bus()->baud_rate();
        else
            restore_baud_rate();
        Status st = axis.status();
        std::optional<bool> control, safety;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            control = sess.modbus_control;
            safety = sess.safety_speed;
        }
        if (!control) {
            report.notes.push_back("PC control before the fault unknown (no track()): not restored");
        } else if (policy.restore_control && *control && !st.modbus_enabled) {
            try {
                const std::optional<Status> taken = axis.enable_modbus_control();
                st = taken ? *taken : axis.status();
                report.control_restored = true;
                report.actions.push_back("PC control (PMSL) taken again");
            } catch (const CallerAbortError&) {
                throw;  // the caller's own error ends the call
            } catch (const EcDriverError& e) {
                report.notes.push_back("PC control (PMSL) not taken again: " + describe(e));
            }
        } else if (*control && st.modbus_enabled) {
            report.control_kept = true;  // held before the fault and still held (a line break, a CON reset)
        }
        if (policy.restore_safety_speed && safety && st.safety_speed() != *safety) {
            if (!st.modbus_enabled) {
                report.notes.push_back("safety speed not restored: the PC does not hold control");
            } else {
                try {
                    st = set_safety_speed(*safety);
                    report.safety_speed_restored = true;
                    report.actions.push_back(std::string("safety speed ") + (*safety ? "ON" : "OFF") + " again");
                } catch (const CallerAbortError&) {
                    throw;  // the caller's own error ends the call
                } catch (const EcDriverError& e) {
                    report.notes.push_back("safety speed not restored: " + describe(e));
                }
            }
        }
        return st;
    }

    void compare_limits(const DeviceInfo& info) {
        std::lock_guard<std::recursive_mutex> lock(sess.mutex);
        if (sess.soft_limits_mm && info.soft_limits_mm && !(*sess.soft_limits_mm == *info.soft_limits_mm))
            report.limits_changed.push_back("soft limits " + span_text(*sess.soft_limits_mm) + " -> " +
                                            span_text(*info.soft_limits_mm));
        if (sess.max_speed_mm_s && info.max_speed_mm_s && *sess.max_speed_mm_s != *info.max_speed_mm_s)
            report.limits_changed.push_back("max speed " + util::g(*sess.max_speed_mm_s) + " -> " +
                                            util::g(*info.max_speed_mm_s));
        if (!sess.units.empty() && !info.units.empty() && sess.units != info.units)
            report.limits_changed.push_back("units " + sess.units + " -> " + info.units);
        if (!report.limits_changed.empty())
            report.notes.push_back("ranges changed (a committed parameter applied at the restart): " +
                                   util::join(report.limits_changed, "; "));
    }

    Status assess_alarm(Status st) {
        const int code = st.alarm_code;
        if (!code) return st;
        std::optional<int> system;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            system = sess.system_code;
        }
        const AlarmLevel level = alarm_level(code, std::nullopt, system);
        const std::string name = alarm_name(code);
        if (level == AlarmLevel::Message && !st.alarm) {
            if (!policy.clear_message_alarms) {
                report.notes.push_back(util::format("message-level alarm 0x%03X (", code) + name +
                                       ") left (a warning: the axis still runs)");
            } else if (!st.modbus_enabled) {
                report.notes.push_back(util::format("message-level alarm 0x%03X (", code) + name +
                                       ") left: the PC does not hold control");
            } else {
                try {
                    st = axis.reset_alarm();
                    report.alarms_cleared.push_back(code);
                    report.actions.push_back(util::format("message-level alarm 0x%03X (", code) + name + ") reset");
                    if (st.alarm_code == code)
                        report.notes.push_back(util::format("0x%03X came back after the reset (e.g. a PCON whose clock "
                                                            "is not set): a warning only, the axis still runs",
                                                            code));
                } catch (const CallerAbortError&) {
                    throw;  // the caller's own error ends the call
                } catch (const EcDriverError& e) {
                    report.notes.push_back(util::format("alarm 0x%03X not reset: ", code) + describe(e));
                }
            }
        } else {
            report.notes.push_back(util::format("alarm 0x%03X (", code) + name + ", " + level_key(level) +
                                   ") left for the operator: check the machine, then reset_alarm() (a cold-start alarm "
                                   "needs a software reset or a power cycle)");
        }
        return st;
    }

    std::optional<Json> usable_backup() {
        const std::optional<Json>& source = policy.backup ? policy.backup : sess.backup;
        if (!source) return std::nullopt;
        const Json* ident = backup_identity(*source);
        const auto serial = json_string(ident ? ident->find("serial_number") : nullptr);
        std::optional<std::string> mine;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            mine = sess.serial_number;
        }
        if (serial && mine && *serial != *mine) {
            report.notes.push_back("backup is of another unit (serial " + *serial + "): not compared");
            return std::nullopt;
        }
        return source;
    }

    void check_rows() {
        std::map<int, PositionEntry> cached;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            if (sess.rows) cached = *sess.rows;
        }
        if (cached.empty()) {
            if (const auto backup = usable_backup()) cached = backup_rows(*backup);
        }
        const RowSelection& wanted = policy.check_rows;
        std::vector<int> rows;
        if (wanted.all && !cached.empty()) {
            for (const auto& kv : cached) rows.push_back(kv.first);
        } else {
            rows = row_numbers(axis, wanted);
        }
        for (int n : rows) {
            PositionEntry entry;
            try {
                entry = axis.read_position_table(n);
            } catch (const std::exception& e) {
                if (is_link_error(e) || dynamic_cast<const CallerAbortError*>(&e)) throw;
                if (!dynamic_cast<const EcDriverError*>(&e) && !dynamic_cast<const std::logic_error*>(&e)) throw;
                report.row_problems.push_back(util::format("row %d: not readable (", n) + describe(e) + ")");
                continue;
            }
            report.rows_checked.push_back(n);
            if (entry.checksum_ok() == std::optional<bool>(false)) {
                const int expected = reg::position_checksum(entry.to_registers());
                report.row_problems.push_back(util::format("row %d: checksum 0x%04X does not match its data (0x%04X)",
                                                           n, entry.checksum.value_or(0), expected));
            }
            const auto ref = cached.find(n);
            if (ref != cached.end() && ref->second != entry) {
                const PositionEntry& a = ref->second;
                std::vector<std::string> changed;
                if (a.position_mm != entry.position_mm) changed.push_back("position_mm");
                if (a.band_mm != entry.band_mm) changed.push_back("band_mm");
                if (a.speed_mm_s != entry.speed_mm_s) changed.push_back("speed_mm_s");
                if (a.zone_plus_mm != entry.zone_plus_mm) changed.push_back("zone_plus_mm");
                if (a.zone_minus_mm != entry.zone_minus_mm) changed.push_back("zone_minus_mm");
                if (a.accel_g != entry.accel_g) changed.push_back("accel_g");
                if (a.decel_g != entry.decel_g) changed.push_back("decel_g");
                if (a.push_power != entry.push_power) changed.push_back("push_power");
                if (a.load_threshold != entry.load_threshold) changed.push_back("load_threshold");
                if (a.control_flags != entry.control_flags) changed.push_back("control_flags");
                if (a.checksum != entry.checksum) changed.push_back("checksum");
                report.row_problems.push_back(util::format("row %d: differs from the cached copy (", n) +
                                              util::join(changed, ", ") + ")");
            }
        }
        if (cached.empty() && wanted.all) report.notes.push_back("no cached rows to compare with: checksums only");
    }

    void check_parameters() {
        std::map<std::string, std::string> cached;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            if (sess.parameters) cached = *sess.parameters;
        }
        if (cached.empty()) {
            if (const auto backup = usable_backup()) cached = backup_parameters(*backup);
        }
        if (cached.empty()) {
            report.notes.push_back("no cached parameters to compare with (track(parameters=True) or a backup)");
            return;
        }
        const std::map<std::string, std::string> now = read_parameters(axis);
        report.parameters_checked = static_cast<int>(cached.size());
        for (const auto& kv : cached) {
            const auto after = now.find(kv.first);
            if (after == now.end())
                report.parameter_differences.push_back(kv.first + ": missing now (was " + kv.second + ")");
            else if (util::upper(after->second) != util::upper(kv.second))
                report.parameter_differences.push_back(kv.first + ": " + kv.second + " -> " + after->second);
        }
        if (!report.parameter_differences.empty())
            report.notes.push_back("parameters differ from the cached copy (a write not committed is lost at "
                                   "power-off): check them before the next move");
    }

    void steps() {
        // 1. what happened?
        bool link_down = false;
        try {
            const Status st = axis.status();
            const std::vector<std::string> signs = restart_signs(sess, st);
            report.cause = !signs.empty() ? std::string(kUnitRestarted)
                                          : (is_lost(prior) || prior == kUnitRestarted ? prior : std::string(kLinkOk));
        } catch (const std::exception& e) {
            const std::string kind = classify_failure(e, &axis, policy.list_ports);
            if (kind.empty()) throw;
            link_down = true;
            report.cause = kind;
            report.notes.push_back("link: " + describe(e));
            set_link_state(axis, kind, describe(e), on_event, label);
        }
        set_state_quietly(axis, kRecovering);

        // 2. port back, reopened, unit found again
        std::optional<int> found;
        if (link_down) {
            found = reconnect(report.cause == kPortVanished);
            report.baud_rate_found = found;
        } else {
            report.baud_rate_found = axis.bus()->baud_rate();
        }

        // 3. the same unit? (nothing is written to the unit before this). Found at another rate than the session's
        // (9600 after a line break): a quick identity check, the session's rate back, then the full identify there
        std::optional<int> session_rate;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            session_rate = sess.baud_rate;
        }
        const bool baud_tried = found && found != session_rate && policy.restore_baud_rate && quick_same();
        if (baud_tried) restore_baud_rate();
        const DeviceInfo info = identify_same();

        // 4. restart signs
        Status st = axis.status();
        std::optional<std::vector<AlarmKey>> baseline;
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            baseline = sess.alarm_history;
        }
        std::optional<std::vector<AlarmKey>> history;
        if (baseline) history = read_history(axis);
        const std::vector<AlarmKey> fresh = new_alarm_records(baseline, history);
        for (const auto& r : fresh)
            report.new_alarm_records.push_back(
                NewAlarmRecord{r.code, util::format("0x%03X", r.code), alarm_name(r.code), r.time_ms});
        const std::vector<std::string> signs = restart_signs(sess, st, fresh, found, policy.boot_baud_rate);
        report.restart_signs = signs;
        report.restarted = !signs.empty();
        if (report.restarted) {
            if (report.cause == kLinkOk) report.cause = kUnitRestarted;
            if (prior != kUnitRestarted)
                emit(on_event, kEventUnitRestarted, label, util::join(signs, "; "), kUnitRestarted);
        }

        // 5. restore the session, compare the ranges
        st = restore(baud_tried);
        compare_limits(info);

        // 6. assess (nothing here moves the axis)
        st = assess_alarm(st);
        report.status = st;
        report.alarm_code = st.alarm_code;
        if (st.alarm_code) report.alarm_name = alarm_name(st.alarm_code);
        if (const auto level = st.alarm_level()) report.alarm_level = level_key(*level);
        report.modbus_control = st.modbus_enabled;
        report.safety_speed = st.safety_speed();
        report.servo_on = st.servo_on;
        if (report.servo_was_on.value_or(false) && !st.servo_on)
            report.notes.push_back("the servo was ON before the fault and is OFF now: servo(true) when it is safe");
        report.homed = st.homed;
        report.needs_home = !st.homed;
        if (report.needs_home)
            report.notes.push_back("HEND is 0 (an incremental unit loses its home at a power loss or reset): home() "
                                   "before the next move");
        report.position_mm = st.position_mm;
        if (st.moving) {
            // the link went, the unit did not (e.g. the USB adapter was pulled): it has no communication watchdog, so
            // the move commanded before the fault runs on. recover() neither stops nor resumes it
            report.notes.push_back(util::format("the axis is still moving (at %.2f): a move commanded before the fault "
                                                "runs on (the unit has no communication watchdog); wait for it or "
                                                "stop() it before the next command",
                                                st.position_mm));
        }
        const std::optional<double> target = report.last_target_mm;
        if (target) {
            const double error = round3(st.position_mm - *target);
            report.position_error_mm = error;
            report.at_target = std::fabs(error) <= policy.position_tolerance_mm;
            if (!*report.at_target && !st.moving)
                report.notes.push_back(util::format("position %.2f is %+.2f from the last commanded target %.2f: the "
                                                    "move was cut off and is NOT resumed",
                                                    st.position_mm, error, *target) +
                                       (report.needs_home ? " (position not meaningful until homed)" : ""));
        }

        // 7. integrity (optional, read-only)
        if (policy.check_rows.selected()) check_rows();
        if (policy.check_parameters) check_parameters();

        // 8. new baseline, link state "ok"
        if (!report.alarms_cleared.empty() && history) history = read_history(axis);
        {
            std::lock_guard<std::recursive_mutex> lock(sess.mutex);
            sess.uptime_ms = uptime_ms(st);
            sess.uptime_at = now_s();
            if (history) sess.alarm_history = history;
            if (sess.modbus_control || st.modbus_enabled) sess.modbus_control = st.modbus_enabled;
            sess.safety_speed = st.safety_speed();
            sess.servo_on = st.servo_on;
            sess.homed = st.homed;
            if (const auto rate = axis.baud_rate()) sess.baud_rate = rate;
            sess.soft_limits_mm = info.soft_limits_mm;
            sess.max_speed_mm_s = info.max_speed_mm_s;
            sess.firmware = info.firmware;
        }
        set_state_quietly(axis, kLinkOk);
        if (report.cause != kLinkOk || prior != kLinkOk)
            emit(on_event, kEventRecovered, label, report.summary(), report.cause);
    }
};

}  // namespace

RecoveryReport recover(Axis& axis, const RecoverOptions& options) {
    const RecoveryPolicy& policy = options.policy;
    const std::shared_ptr<Session> sess = options.session ? options.session : session_of(axis);
    if (!sess)
        throw RecoveryError("no session for this axis: call recovery::track(axis) while the link is up (the identity "
                            "to compare with is unknown)");
    if (options.timeout_s < 0) throw std::invalid_argument("timeout must be >= 0");
    if (axis.bus()->closed()) throw RecoveryError("the bus is closed (close() was called): open the axis again instead");
    std::string label = options.name;
    if (label.empty()) {
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        label = sess->name;
    }
    if (label.empty()) label = default_name(axis);
    const double started = now_s();
    auto report = std::make_shared<RecoveryReport>();
    report->axis = label;
    {
        std::lock_guard<std::recursive_mutex> lock(sess->mutex);
        report->last_target_mm = sess->last_target_mm;
        report->servo_was_on = sess->servo_on;
    }
    std::lock_guard<std::mutex> one_at_a_time(sess->recover_mutex);
    const std::string prior = link_state(axis);
    sess->recovering = true;
    auto finish = [&] {
        sess->recovering = false;
        report->elapsed_s = round3(now_s() - started);
        if (link_state(axis) == kRecovering) {  // it failed: back to the fault it started from
            const std::string fault = report->cause != kLinkOk ? report->cause : prior;
            set_state_quietly(axis, fault != kRecovering ? fault : std::string(kLinkOk));
        }
    };
    Run run{axis, *sess, policy, *report, report, started + options.timeout_s, options.timeout_s, options.on_event,
            label, prior};
    try {
        run.steps();
    } catch (RecoveryError& e) {
        if (!e.report()) e.set_report(report);
        finish();
        throw;
    } catch (const CallerAbortError&) {
        finish();
        throw;  // the caller's own error ends the call as it is (no RecoveryError around it)
    } catch (const EcDriverError& e) {
        const std::string kind = classify_failure(e, &axis, policy.list_ports);
        if (!kind.empty()) report->cause = kind;
        finish();
        throw RecoveryError("recovery interrupted: " + describe(e), report);
    } catch (...) {
        finish();
        throw;
    }
    finish();
    return *report;
}

RecoveryReport recover_axis(Axis& axis, const RecoverOptions& options) {
    RecoverOptions o = options;
    if (!o.session && !o.require_session && !session_of(axis)) {
        const std::optional<DeviceInfo> info = axis.device_info();
        if (info && (info->serial_number || info->model_name || info->system_code))
            o.session = session_from_info(axis, *info, o.name);
    }
    // the status monitor is held while the recovery runs (with every other running monitor on this port: the baud-rate
    // search and a reopen change the line under them) and restarted afterwards when it was running or ended on the
    // fault (the port vanished); a failed recovery leaves it as it was
    const std::shared_ptr<AxisMonitor> monitor = axis.monitor();
    const bool restart = monitor && (monitor->running() || monitor->error());
    RecoveryReport report;
    {
        BusSuspension hold = suspend_bus(*axis.bus());
        report = recover(axis, o);
    }
    if (restart && monitor == axis.monitor() && !monitor->running()) {
        try {
            monitor->start();
        } catch (const std::exception& e) {
            report.notes.push_back(std::string("the status monitor was not restarted: ") + e.what());
        }
    }
    return report;
}

// ================================================================================================ the monitor's link watch

namespace {

/// The status monitor's link watch (monitor.hpp): its lost polls and its periodic link check go through the recovery
/// bookkeeping, as the Python edition's monitor does through recovery.py. Installed once per process (below).
EventCallback bridge(const LinkEventCallback& emit) {
    if (!emit) return EventCallback();
    return [emit](const RecoveryEvent& e) {
        LinkEvent event;
        event.kind = e.kind;
        event.axis = e.axis;
        event.message = e.message;
        event.cause = e.cause;
        event.time = e.time;
        emit(event);
    };
}

bool watch_recovering(Axis& axis) {
    const std::shared_ptr<Session> sess = session_of(axis);
    return sess && sess->recovering.load();
}

MonitorLinkWatch make_monitor_link_watch() {
    MonitorLinkWatch w;
    w.recovering = watch_recovering;
    w.link_fault = [](Axis& axis, const std::exception& error, const LinkEventCallback& emit) {
        try {  // the link watch must never break the monitor
            if (watch_recovering(axis)) return;
            const std::string kind = classify_failure(error, &axis);
            if (kind == kPortVanished || kind == kUnitSilent) set_link_state(axis, kind, describe(error), bridge(emit));
        } catch (const std::exception& e) {
            log_debug(kLog, std::string("link state not recorded: ") + e.what());
        }
    };
    w.link_state = [](Axis& axis) { return link_state(axis); };
    w.check_link = [](Axis& axis, const LinkEventCallback& emit) -> std::optional<MonitorLinkCheck> {
        // the axis's session; made without I/O from the identity the axis already read, else no check
        std::shared_ptr<Session> sess = session_of(axis);
        if (!sess) {
            const std::optional<DeviceInfo> info = axis.device_info();
            if (!info || !(info->serial_number || info->model_name || info->system_code || info->unit_code))
                return std::nullopt;
            sess = session_from_info(axis, *info);
        }
        if (sess->recovering.load()) return std::nullopt;
        CheckLinkOptions o;
        o.session = sess;
        o.on_event = bridge(emit);
        const LinkCheck chk = check_link(axis, o);
        MonitorLinkCheck out;
        out.state = chk.state;
        out.reasons = chk.reasons;
        out.error = chk.error;
        return out;
    };
    return w;
}

/// Installs the watch at start-up (monitor.hpp keeps it in a function-local static, so the order of the static
/// initialisers does not matter).
const bool kMonitorWatchInstalled = [] {
    set_monitor_link_watch(make_monitor_link_watch());
    return true;
}();

}  // namespace

}  // namespace recovery
}  // namespace iai_actuator_driver
