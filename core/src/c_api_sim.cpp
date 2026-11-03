// C ABI 0.3 simulators (include/iai_actuator_driver.h "simulators", with the 0.5 additions: a clock of the unit's own,
// the other SimulatedEC options and controls): simulated units with the test hooks of the Python edition's SimulatedEC
// (HookedSimUnit, sim_hooks.hpp) and simulated lines (SimLine). 0.6 ("simulators: ports, gateway lines"): the gateway
// lines (SimGatewayLine, sim_gateway.hpp) and their controls, a port for a bus that holds the timer resolution, a
// request hook that fails the request. Test and demo tooling; never opens a port. Evidence: simulator tests only.
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
#include "util.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

constexpr std::size_t kHookReplyCapacity = 256;

// ============================================================================================ JSON inputs

/// A JSON object (NULL or "" = {}); std::invalid_argument otherwise.
Json parse_object(const char* text, const char* what) {
    if (!text || !*text) return Json::object();
    Json j;
    try {
        j = Json::parse(text);
    } catch (const std::exception& e) {
        throw std::invalid_argument(std::string(what) + " is not valid JSON: " + e.what());
    }
    if (j.type() != Json::Type::Object) throw std::invalid_argument(std::string(what) + " must be a JSON object");
    return j;
}

bool is_number(const Json& v) { return v.type() == Json::Type::Int || v.type() == Json::Type::Number; }

double number(const Json& v, const std::string& key) {
    if (!is_number(v)) throw std::invalid_argument("'" + key + "' must be a number");
    const double d = v.as_number();
    if (!std::isfinite(d)) throw std::invalid_argument("'" + key + "' must be a finite number");
    return d;
}

long long integer(const Json& v, const std::string& key) {
    if (v.type() == Json::Type::Int) return v.as_int();
    if (v.type() == Json::Type::Number && std::floor(v.as_number()) == v.as_number()) return v.as_int();
    if (v.type() == Json::Type::String) {  // "0x9005", "36869"
        try {
            std::size_t used = 0;
            const long long n = std::stoll(v.as_string(), &used, 0);
            if (used == v.as_string().size()) return n;
        } catch (const std::exception&) {
        }
    }
    throw std::invalid_argument("'" + key + "' must be an integer");
}

int in_range(long long value, long long low, long long high, const std::string& key) {
    if (value < low || value > high)
        throw std::out_of_range("'" + key + "' must be " + std::to_string(low) + ".." + std::to_string(high) + ", got " +
                                std::to_string(value));
    return static_cast<int>(value);
}

bool boolean(const Json& v, const std::string& key) {
    if (v.type() != Json::Type::Bool) throw std::invalid_argument("'" + key + "' must be true or false");
    return v.as_bool();
}

std::string text(const Json& v, const std::string& key) {
    if (v.type() != Json::Type::String) throw std::invalid_argument("'" + key + "' must be a string");
    return v.as_string();
}

const Json& array(const Json& v, const std::string& key) {
    if (v.type() != Json::Type::Array) throw std::invalid_argument("'" + key + "' must be an array");
    return v;
}

/// A register address from a number or a string ("0x9005").
int address_of(const Json& v, const std::string& key) { return in_range(integer(v, key), 0, 0xFFFF, key); }

std::map<int, std::uint16_t> word_overrides_of(const Json& v) {
    std::map<int, std::uint16_t> out;
    if (v.type() == Json::Type::Object) {
        for (const auto& kv : v.as_object()) {
            const int address = address_of(Json(kv.first), "word_overrides key");
            out[address] = static_cast<std::uint16_t>(in_range(integer(kv.second, "word_overrides value"), 0, 0xFFFF,
                                                               "word_overrides value"));
        }
        return out;
    }
    for (const auto& pair : array(v, "word_overrides").as_array()) {
        if (pair.type() != Json::Type::Array || pair.size() != 2)
            throw std::invalid_argument("'word_overrides' entries must be [address, value]");
        out[address_of(pair.as_array()[0], "word_overrides address")] = static_cast<std::uint16_t>(
            in_range(integer(pair.as_array()[1], "word_overrides value"), 0, 0xFFFF, "word_overrides value"));
    }
    return out;
}

std::vector<std::pair<int, int>> ranges_of(const Json& v) {
    std::vector<std::pair<int, int>> out;
    for (const auto& pair : array(v, "unreadable").as_array()) {
        if (pair.type() != Json::Type::Array || pair.size() != 2)
            throw std::invalid_argument("'unreadable' entries must be [start, count]");
        out.emplace_back(address_of(pair.as_array()[0], "unreadable start"),
                         in_range(integer(pair.as_array()[1], "unreadable count"), 1, 0x10000, "unreadable count"));
    }
    return out;
}

std::vector<std::uint16_t> words_of(const Json& v, const std::string& key) {
    std::vector<std::uint16_t> out;
    for (const auto& w : array(v, key).as_array()) out.push_back(static_cast<std::uint16_t>(in_range(integer(w, key), 0, 0xFFFF, key)));
    return out;
}

/// One key of the options JSON into `o` / `h` (null = the default, also when changing a set value); false when `k` is
/// no simulator option.
bool set_option_key(SimulatedUnitOptions& o, SimHookOptions& h, const std::string& k, const Json& v) {
    if (k == "profile") {
        const std::string name = text(v, k);
        const auto p = sim_profile_from_string(name);
        if (!p)
            throw std::invalid_argument("unknown simulator profile '" + name +
                                        "' (slider, gripper, rod, pcon, rotary, unknown)");
        o.profile = *p;
    } else if (k == "slave") {
        o.slave = in_range(integer(v, k), 0, 247, k);  // 0.7: 0 as the Python edition, a unit that hears only broadcasts
    } else if (k == "time_scale") {
        o.time_scale = number(v, k);
        if (!(o.time_scale > 0.0)) throw std::invalid_argument("'time_scale' must be > 0");
    } else if (k == "position_mm") {
        o.position_mm = number(v, k);
    } else if (k == "stroke_mm") {
        o.stroke_mm = number(v, k);
    } else if (k == "max_speed_mm_s") {
        o.max_speed_mm_s = number(v, k);
    } else if (k == "max_accel_g") {
        o.max_accel_g = number(v, k);
    } else if (k == "system_code") {
        o.system_code = in_range(integer(v, k), 0, 255, k);
    } else if (k == "unit_code") {
        o.unit_code = in_range(integer(v, k), 0, 255, k);
    } else if (k == "firmware") {
        if (array(v, k).size() != 2) throw std::invalid_argument("'firmware' must be [major, minor]");
        o.firmware = std::make_pair(in_range(integer(v.as_array()[0], k), 0, 255, k),
                                    in_range(integer(v.as_array()[1], k), 0, 255, k));
    } else if (k == "dopm") {
        const std::string d = text(v, k);
        if (d == "three_position")
            o.dopm = DopmLayout::ThreePosition;
        else if (d == "two_position")
            o.dopm = DopmLayout::TwoPosition;
        else
            throw std::invalid_argument("'dopm' must be \"three_position\" or \"two_position\"");
    } else if (k == "pmsl") {
        o.pmsl = boolean(v, k);
    } else if (k == "recompute_position_checksum") {
        o.recompute_position_checksum = boolean(v, k);
    } else if (k == "drop_reply_after_retarget") {
        o.drop_reply_after_retarget = boolean(v, k);
    } else if (k == "stop_latches") {
        o.stop_latches = boolean(v, k);
    } else if (k == "move_bit_clears_in_decel") {
        o.move_bit_clears_in_decel = boolean(v, k);
    } else if (k == "safety_speed") {
        o.safety_speed = boolean(v, k);
    } else if (k == "soft_limits") {
        o.soft_limits = boolean(v, k);
    } else if (k == "ls_range_mm") {
        o.ls_range_mm = number(v, k);
    } else if (k == "early_in_position") {
        o.early_in_position = boolean(v, k);
    } else if (k == "settle_tail_mm") {
        o.settle_tail_mm = number(v, k);
    } else if (k == "settle_speed_mm_s") {
        o.settle_speed_mm_s = number(v, k);
    } else if (k == "push_blip_mm") {
        o.push_blip_mm = v.is_null() ? std::nullopt : std::optional<double>(number(v, k));
    } else if (k == "push_blip_len_mm") {
        o.push_blip_len_mm = number(v, k);
    } else if (k == "absolute") {
        o.absolute = v.is_null() ? std::nullopt : std::optional<bool>(boolean(v, k));
    } else if (k == "baud_rate") {
        o.baud_rate = v.is_null() ? std::nullopt : std::optional<int>(in_range(integer(v, k), 1, 10000000, k));
    } else if (k == "unit_baud_rate") {
        o.unit_baud_rate = v.is_null() ? std::nullopt : std::optional<int>(in_range(integer(v, k), 1, 10000000, k));
    } else if (k == "boot_baud_rate") {
        o.boot_baud_rate = in_range(integer(v, k), 1, 10000000, k);
    } else if (k == "reset_baud_rate") {
        o.reset_baud_rate = v.is_null() ? std::nullopt : std::optional<int>(in_range(integer(v, k), 1, 10000000, k));
    } else if (k == "reset_s") {
        o.reset_s = number(v, k);
    } else if (k == "slave_id_delay_s") {
        o.slave_id_delay_s = v.is_null() ? std::nullopt : std::optional<double>(number(v, k));
    } else if (k == "parw_busy_s") {
        o.parw_busy_s = number(v, k);
    } else if (k == "parw_dsse_busy") {
        o.parw_dsse_busy = boolean(v, k);
    } else if (k == "ignored_baud_codes") {
        o.ignored_baud_codes.clear();
        for (const auto& c : array(v, k).as_array()) o.ignored_baud_codes.push_back(in_range(integer(c, k), 0, 255, k));
    } else if (k == "serial_number") {
        o.serial_number = v.is_null() ? std::nullopt : std::optional<std::string>(text(v, k));
    } else if (k == "move_patterns") {
        o.move_patterns = v.is_null() ? std::nullopt
                                      : std::optional<std::uint16_t>(static_cast<std::uint16_t>(
                                            in_range(integer(v, k), 0, 0xFFFF, k)));
    } else if (k == "auto_servo_off_s") {  // 0.5
        o.auto_servo_off_s = v.is_null() ? std::nullopt : std::optional<double>(number(v, k));
    } else if (k == "cold_boot") {
        o.cold_boot = boolean(v, k);
    } else if (k == "ec_first_son_s") {
        o.ec_first_son_s = number(v, k);
    } else if (k == "rtc") {
        o.rtc = boolean(v, k);
    } else if (k == "function_codes") {
        o.function_codes.reset();
        if (!v.is_null()) {
            o.function_codes.emplace();
            for (const auto& f : array(v, k).as_array()) o.function_codes->push_back(in_range(integer(f, k), 0, 255, k));
        }
    } else if (k == "broadcast") {
        o.broadcast = v.is_null() ? std::nullopt : std::optional<bool>(boolean(v, k));
    } else if (k == "parw_latches") {
        o.parw_latches = boolean(v, k);
    } else if (k == "port_name") {  // 0.6
        o.port_name = v.is_null() ? std::nullopt : std::optional<std::string>(text(v, k));
    } else if (k == "word_overrides") {
        h.word_overrides = word_overrides_of(v);
    } else if (k == "unreadable") {
        h.unreadable = ranges_of(v);
    } else if (k == "stale_status_polls") {
        h.stale_status_polls = in_range(integer(v, k), 0, 1000000, k);
    } else if (k == "step_s") {
        h.step_s = v.is_null() ? std::nullopt : std::optional<double>(number(v, k));
    } else {
        return false;
    }
    return true;
}

/// The options JSON of iad_sim_open.
std::pair<SimulatedUnitOptions, SimHookOptions> sim_options_of(const char* options_json) {
    const Json j = parse_object(options_json, "simulator options");
    SimulatedUnitOptions o;
    SimHookOptions h;
    for (const auto& kv : j.as_object())
        if (!set_option_key(o, h, kv.first, kv.second))
            throw std::invalid_argument("unknown simulator option '" + kv.first + "'");
    return {o, h};
}

/// Options "set_option" cannot change: the unit's fixed facts, its initial state (set_state changes the state) and the
/// hooks with controls of their own.
bool fixed_option(const std::string& k) {
    static const char* const fixed[] = {
        "profile", "slave", "time_scale", "position_mm", "stroke_mm", "max_speed_mm_s", "system_code", "unit_code",
        "absolute", "baud_rate", "unit_baud_rate", "serial_number", "rtc", "pmsl", "safety_speed", "word_overrides",
        "unreadable", "stale_status_polls", "step_s", "port_name"};
    for (const char* f : fixed)
        if (k == f) return true;
    return false;
}

/// An optional boolean argument of a control (absent or null: nullopt).
std::optional<bool> flag_arg(const Json& c, const std::string& key) {
    const Json* v = c.find(key);
    if (!v || v->is_null()) return std::nullopt;
    return boolean(*v, key);
}

/// Refuse the arguments of a control other than `known` (and "command").
void only_arguments(const Json& c, std::initializer_list<const char*> known) {
    for (const auto& kv : c.as_object()) {
        if (kv.first == "command") continue;
        bool ok = false;
        for (const char* k : known) ok = ok || kv.first == k;
        if (!ok) throw std::invalid_argument("unknown argument '" + kv.first + "' of the command");
    }
}

std::optional<bool> tribool_arg(iad_tribool value, const char* /*name*/) {  // 0.8: 0 false, IAD_UNKNOWN unknown, else true
    if (value == IAD_UNKNOWN) return std::nullopt;
    return value != IAD_FALSE;
}

std::optional<bool> optional_bool(const Json* v, const std::string& key) {
    if (!v || v->is_null()) return std::nullopt;
    return boolean(*v, key);
}

const Json& required(const Json& j, const std::string& key) {
    const Json* v = j.find(key);
    if (!v) throw std::invalid_argument("the command needs '" + key + "'");
    return *v;
}

// ============================================================================================ handles

std::shared_ptr<HookedSimUnit> find_sim(iad_sim* handle) {
    return std::static_pointer_cast<SimEntry>(find_object(handle, ObjectKind::Sim))->unit;
}

std::shared_ptr<SimLine> find_line(iad_sim_line* handle) {
    return std::static_pointer_cast<LineEntry>(find_object(handle, ObjectKind::SimLine))->line;
}

void write_json(const Json& json, char* buffer, std::size_t buffer_size, std::size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

Json words_json(const std::map<int, std::uint16_t>& words) {
    Json j = Json::object();
    for (const auto& kv : words) j.set(std::to_string(kv.first), static_cast<int>(kv.second));
    return j;
}

Json state_of(const HookedSimUnit& u) {
    Json j = Json::object();
    j.set("profile", to_string(u.profile()));
    j.set("slave", u.slave());
    j.set("stroke_mm", u.stroke_mm());
    j.set("max_speed_mm_s", u.max_speed_mm_s());
    j.set("system_code", u.system_code());
    j.set("unit_code", u.unit_code());
    j.set("position_count", u.position_count());
    j.set("absolute", u.absolute());
    j.set("time_scale", u.time_scale());
    j.set("servo_on", u.servo_on());
    j.set("homed", u.homed());
    j.set("pmsl", u.pmsl());
    j.set("safety_speed", u.safety_speed());
    j.set("paused", u.paused());
    j.set("stop_held", u.stop_held());
    j.set("alarm_code", u.alarm_code());
    j.set("position_mm", u.position_mm());
    j.set("moving", u.moving());
    const Span limits = u.soft_limits_mm();
    Json span = Json::array();
    span.push_back(limits.min);
    span.push_back(limits.max);
    j.set("soft_limits_mm", span);
    j.set("obstacle_mm", Json(u.obstacle_mm()));
    j.set("unit_baud_rate", Json(u.unit_baud_rate()));
    const int rate = u.baud_rate();
    j.set("baud_rate", rate > 0 ? Json(rate) : Json());
    j.set("powered", u.powered());
    j.set("present", Json(u.present()));
    j.set("open", u.is_open());
    j.set("reopens", u.reopens());
    j.set("serial_number", u.serial_number());
    j.set("nonvolatile_writes", u.nonvolatile_writes());
    j.set("parameter_commits", u.parameter_commits());
    j.set("parameter_ram_writes", u.parameter_ram_writes());
    j.set("software_resets", u.software_resets());
    j.set("override_moves", u.override_moves());
    j.set("profile_moves", u.profile_moves());
    j.set("accel_pattern", u.accel_pattern());
    j.set("command_speed_mm_s", u.command_speed_mm_s());
    j.set("push_speed_mm_s", u.push_speed_mm_s());
    j.set("parw_busy", u.parw_busy());
    j.set("parameter_ram", words_json(u.parameter_ram()));
    j.set("parameter_nv", words_json(u.parameter_nv()));
    j.set("word_overrides", words_json(u.word_overrides()));
    Json ranges = Json::array();
    for (const auto& r : u.unreadable()) {
        Json pair = Json::array();
        pair.push_back(r.first);
        pair.push_back(r.second);
        ranges.push_back(pair);
    }
    j.set("unreadable", ranges);
    j.set("stale_status_polls", u.stale_status_polls());
    j.set("stale_polls_served", u.stale_polls_served());
    j.set("step_s", Json(u.step_s()));
    j.set("clock_s", u.clock_now());
    j.set("ls_range_mm", u.ls_range_mm());  // 0.5
    j.set("auto_servo_off_active", u.auto_servo_off_active());
    const SimulatedUnitOptions o = u.options();  // 0.6
    j.set("push_blip_mm", Json(o.push_blip_mm));
    j.set("push_blip_len_mm", o.push_blip_len_mm);
    j.set("port_name", o.port_name ? Json(*o.port_name) : Json());
    j.set("baud_history", Json(u.baud_history()));
    return j;
}

SimFault fault_of(const std::string& kind) {
    if (kind == "timeout") return SimFault::Timeout;
    if (kind == "crc") return SimFault::Crc;
    if (kind == "partial") return SimFault::Partial;
    if (kind == "wrong_slave") return SimFault::WrongSlave;
    throw std::invalid_argument("fault kind must be timeout, crc, partial or wrong_slave, got '" + kind + "'");
}

Json control(HookedSimUnit& u, const Json& c) {
    const std::string command = text(required(c, "command"), "command");
    Json out = Json::object();
    if (command == "inject_fault") {
        const Json* count = c.find("count");
        u.inject_fault(fault_of(text(required(c, "kind"), "kind")),
                       count ? in_range(integer(*count, "count"), 1, 1000000, "count") : 1);
    } else if (command == "trigger_alarm") {
        u.trigger_alarm(in_range(integer(required(c, "code"), "code"), 0, 0xFFFF, "code"));
    } else if (command == "set_obstacle") {
        const Json& mm = required(c, "mm");
        u.set_obstacle_mm(mm.is_null() ? std::nullopt : std::optional<double>(number(mm, "mm")));
    } else if (command == "pio_move") {
        const Json* speed = c.find("speed_mm_s");
        const Json* accel = c.find("accel_g");
        const Json* hold = c.find("hold_start");
        out.set("started", u.pio_move(number(required(c, "target_mm"), "target_mm"),
                                      speed ? number(*speed, "speed_mm_s") : 20.0,
                                      accel ? number(*accel, "accel_g") : 0.3, hold ? boolean(*hold, "hold_start") : false));
    } else if (command == "pio_release") {
        u.pio_release();
    } else if (command == "coil_state") {
        out.set("on", u.coil_state(address_of(required(c, "address"), "address")));
    } else if (command == "set_coil_state") {
        u.set_coil_state(address_of(required(c, "address"), "address"), boolean(required(c, "on"), "on"));
    } else if (command == "position_row") {
        const std::vector<std::uint16_t> words = u.position_row(static_cast<int>(integer(required(c, "row"), "row")));
        out.set("words", Json(std::vector<int>(words.begin(), words.end())));
    } else if (command == "set_position_row") {
        const std::vector<std::uint16_t> words = words_of(required(c, "words"), "words");
        if (words.size() != 16) throw std::invalid_argument("'words' must hold 16 words");
        u.set_position_row(static_cast<int>(integer(required(c, "row"), "row")), words);
    } else if (command == "clear_requests") {
        u.clear_requests();
        u.clear_request_log();
    } else if (command == "clear_events") {
        u.clear_events();
    } else if (command == "set_step_s") {
        u.set_step_s(number(required(c, "step_s"), "step_s"));
    } else if (command == "set_state") {  // 0.5
        only_arguments(c, {"servo_on", "homed", "pmsl", "safety_speed"});
        SimStateLevels levels;
        levels.servo_on = flag_arg(c, "servo_on");
        levels.homed = flag_arg(c, "homed");
        levels.pmsl = flag_arg(c, "pmsl");
        levels.safety_speed = flag_arg(c, "safety_speed");
        u.set_state(levels);
    } else if (command == "set_option") {
        const std::string name = text(required(c, "name"), "name");
        if (fixed_option(name))
            throw std::invalid_argument("simulator option '" + name + "' cannot be changed after the unit is made");
        SimulatedUnitOptions o = u.options();
        SimHookOptions unused;
        if (!set_option_key(o, unused, name, required(c, "value")))
            throw std::invalid_argument("unknown simulator option '" + name + "'");
        u.set_options(o);
    } else if (command == "set_inputs") {
        only_arguments(c, {"emg", "drive_cut", "enable"});
        u.set_inputs(flag_arg(c, "emg"), flag_arg(c, "drive_cut"), flag_arg(c, "enable"));
    } else if (command == "set_parameter_word") {
        const Json* nv = c.find("nv");
        u.set_parameter_word(address_of(required(c, "address"), "address"),
                             static_cast<std::uint16_t>(in_range(integer(required(c, "value"), "value"), 0, 0xFFFF, "value")),
                             nv && !nv->is_null() && boolean(*nv, "nv"));
    } else {
        throw std::invalid_argument("unknown simulator command '" + command + "'");
    }
    return out;
}

Json hex_list(const std::vector<std::vector<std::uint8_t>>& frames) {
    Json list = Json::array();
    for (const auto& f : frames) list.push_back(hex_frame(f));
    return list;
}

/// The line as a gateway line (std::invalid_argument for a plain line).
SimGatewayLine& gateway_line(SimLine& line) {
    auto* gw = dynamic_cast<SimGatewayLine*>(&line);
    if (!gw)
        throw std::invalid_argument("line " + line.name() + " has no gateway (iad_sim_gateway_line_open makes one)");
    return *gw;
}

std::vector<int> slave_list(const Json& v, const std::string& key) {
    std::vector<int> out;
    for (const auto& s : array(v, key).as_array()) out.push_back(in_range(integer(s, key), 0, 255, key));
    return out;
}

/// (0.6) The "gateway" control: the gateway's state and the frames the line kept.
Json gateway_state(const SimGatewayLine& line) {
    Json j = Json::object();
    j.set("kind", to_string(line.kind()));
    j.set("silent_no_access", line.silent_no_access());
    j.set("function_codes", Json(line.function_codes()));
    j.set("dropped_broadcasts", hex_list(line.dropped_broadcasts()));
    j.set("rejected", hex_list(line.rejected()));
    Json g;
    if (const auto rcon = line.rcon_gateway()) {
        g = Json::object();
        g.set("slave", rcon->slave());
        g.set("manu", rcon->manu());
        g.set("alarm_code", rcon->alarm_code());
        g.set("emergency_stop", rcon->emergency_stop());
        g.set("linked_only", Json(rcon->linked_only()));
        g.set("linked_slaves", Json(rcon->linked_slaves()));
        const std::vector<std::uint16_t> words = rcon->status_block();
        g.set("status_block", Json(std::vector<int>(words.begin(), words.end())));
        g.set("requests", hex_list(rcon->requests()));
    } else if (const auto robonet = line.robonet_gateway()) {
        g = Json::object();
        g.set("slave", robonet->slave());
        g.set("running", robonet->running());
        g.set("requests", hex_list(robonet->requests()));
    }
    j.set("gateway", g);
    return j;
}

/// (0.6) The "set_gateway" control.
void set_gateway(SimGatewayLine& line, const Json& c) {
    only_arguments(c, {"manu", "alarm_code", "emergency_stop", "slave", "linked_only", "running", "silent_no_access"});
    const auto rcon = line.rcon_gateway();
    const auto robonet = line.robonet_gateway();
    for (const auto& kv : c.as_object()) {
        const std::string& k = kv.first;
        const Json& v = kv.second;
        if (k == "command") continue;
        if (k == "silent_no_access") {
            line.set_silent_no_access(boolean(v, k));
        } else if (k == "slave") {
            const int slave = in_range(integer(v, k), 0, 255, k);
            if (rcon)
                rcon->set_slave(slave);
            else if (robonet)
                robonet->set_slave(slave);
            else
                throw std::invalid_argument("line " + line.name() + " has no gateway station (ROBONET through mode)");
        } else if (k == "running") {
            if (!robonet) throw std::invalid_argument("'running' is a ROBONET gateway's (gateway mode)");
            robonet->set_running(boolean(v, k));
        } else if (!rcon) {
            throw std::invalid_argument("'" + k + "' is an RCON gateway's");
        } else if (k == "manu") {
            rcon->set_manu(boolean(v, k));
        } else if (k == "alarm_code") {
            rcon->set_alarm_code(in_range(integer(v, k), 0, 0x7FFFFFFF, k));
        } else if (k == "emergency_stop") {
            rcon->set_emergency_stop(boolean(v, k));
        } else {  // linked_only
            rcon->set_linked_only(slave_list(v, k));
        }
    }
}

/// (0.6) The "gateway_handle" control: the gateway's own answer to one frame.
Json gateway_handle(SimGatewayLine& line, const Json& c) {
    only_arguments(c, {"frame"});
    const std::vector<std::uint8_t> frame = util::hex_bytes(text(required(c, "frame"), "frame"));
    if (frame.size() < 2) throw std::invalid_argument("'frame' must hold at least an address and a function code");
    std::vector<std::uint8_t> reply;
    if (const auto rcon = line.rcon_gateway())
        reply = rcon->handle(frame);
    else if (const auto robonet = line.robonet_gateway())
        reply = robonet->handle(frame);
    else
        throw std::invalid_argument("line " + line.name() + " has no gateway station (ROBONET through mode)");
    Json out = Json::object();
    out.set("reply", reply.empty() ? Json() : Json(hex_frame(reply)));
    return out;
}

/// (0.6) The options of iad_sim_gateway_line_open.
SimGatewayOptions gateway_options_of(const char* kind, const char* options_json) {
    SimGatewayOptions o;
    const std::string name = kind && *kind ? kind : "rcon";
    const std::optional<LineKind> k = line_kind_from_string(name);
    if (!k || *k == LineKind::Direct)
        throw std::invalid_argument("kind must be one of rcon, robonet, robonet-gateway, got '" + name + "'");
    o.kind = *k;
    const Json j = parse_object(options_json, "gateway line options");
    for (const auto& kv : j.as_object()) {
        const std::string& key = kv.first;
        const Json& v = kv.second;
        if (key == "name") {
            if (!v.is_null()) o.name = text(v, key);
        } else if (key == "manu") {
            o.manu = boolean(v, key);
        } else if (key == "gateway_alarm") {
            o.gateway_alarm = in_range(integer(v, key), 0, 0x7FFFFFFF, key);
        } else if (key == "emergency_stop") {
            o.emergency_stop = boolean(v, key);
        } else if (key == "linked_only") {
            o.linked_only = slave_list(v, key);
        } else if (key == "position_unit_mm") {
            if (v.type() == Json::Type::Object) {
                for (const auto& unit : v.as_object())
                    o.position_units[in_range(integer(Json(unit.first), key), 1, 247, key)] = number(unit.second, key);
            } else {
                o.position_unit_mm = number(v, key);
            }
        } else if (key == "serials") {
            if (v.type() != Json::Type::Object) throw std::invalid_argument("'serials' must be {\"slave\": code}");
            for (const auto& s : v.as_object())
                o.serials[in_range(integer(Json(s.first), key), 1, 247, key)] = text(s.second, key);
        } else if (key == "silent_no_access") {
            o.silent_no_access = boolean(v, key);
        } else {
            throw std::invalid_argument("unknown gateway line option '" + key + "'");
        }
    }
    return o;
}

Json line_control(SimLine& line, const Json& c) {
    const std::string command = text(required(c, "command"), "command");
    Json out = Json::object();
    if (command == "unplug") {
        line.unplug();
    } else if (command == "replug") {
        line.replug();
    } else if (command == "power_cycle") {
        std::vector<int> slaves;
        if (const Json* s = c.find("slaves"))
            for (const auto& v : array(*s, "slaves").as_array()) slaves.push_back(in_range(integer(v, "slaves"), 1, 247, "slaves"));
        line.power_cycle(slaves, optional_bool(c.find("lose_home"), "lose_home"));
    } else if (command == "clear_requests") {
        line.clear_requests();
    } else if (command == "state") {
        out.set("name", line.name());
        out.set("slaves", Json(line.slaves()));
        out.set("present", Json(line.present()));
        out.set("open", line.is_open());
        out.set("reopens", line.reopens());
        const int rate = line.baud_rate();
        out.set("baud_rate", rate > 0 ? Json(rate) : Json());
        const auto* gw = dynamic_cast<const SimGatewayLine*>(&line);  // 0.6
        out.set("kind", to_string(gw ? gw->kind() : LineKind::Direct));
    } else if (command == "gateway") {  // 0.6
        only_arguments(c, {});
        out = gateway_state(gateway_line(line));
    } else if (command == "set_gateway") {
        set_gateway(gateway_line(line), c);
    } else if (command == "gateway_handle") {
        out = gateway_handle(gateway_line(line), c);
    } else {
        throw std::invalid_argument("unknown line command '" + command + "'");
    }
    return out;
}

/// (0.6) The 1 ms timer resolution the bus ports of the simulators hold while open (SimPort): its own count, apart from
/// the waits' and monitors' process_timer().
TimerResolution& sim_port_timer() {
    static TimerResolution timer;
    return timer;
}

/// (0.6) The port a bus gets on a simulated unit or line (iad_sim_port, iad_sim_line_port): every call goes to the unit
/// or line, and while open it holds the 1 ms timer resolution, as a serial port and a binding's callback transport do
/// (Windows), so the bus's gap and its polls of the simulator sleep in 1 ms steps, not the default 15.6 ms.
class SimPort final : public Transport {
public:
    explicit SimPort(std::shared_ptr<Transport> target)
        : target_(std::move(target)), timer_(std::make_unique<ScopedTimerResolution>(sim_port_timer())) {}

    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override { target_->write(data, size); }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override { return target_->read(buffer, size); }
    void reset_input() override { target_->reset_input(); }
    int baud_rate() const override { return target_->baud_rate(); }
    void set_baud_rate(int rate) override { target_->set_baud_rate(rate); }
    double read_timeout() const override { return target_->read_timeout(); }
    void set_read_timeout(double seconds) override { target_->set_read_timeout(seconds); }
    bool has_real_timeout() const override { return target_->has_real_timeout(); }
    void send_break(double seconds) override { target_->send_break(seconds); }
    void close() override {
        target_->close();
        hold_timer(false);
    }
    bool is_open() const override { return target_->is_open(); }
    std::string name() const override { return target_->name(); }
    std::optional<bool> present() const override { return target_->present(); }
    bool can_reopen() const override { return target_->can_reopen(); }
    void reopen(std::optional<int> baud_rate) override {
        target_->reopen(baud_rate);
        hold_timer(true);
    }

private:
    void hold_timer(bool hold) {
        std::lock_guard<std::mutex> lock(timer_mutex_);
        if (!hold) {
            timer_.reset();
        } else if (!timer_) {
            timer_ = std::make_unique<ScopedTimerResolution>(sim_port_timer());
        }
    }

    std::shared_ptr<Transport> target_;
    std::mutex timer_mutex_;
    std::unique_ptr<ScopedTimerResolution> timer_;
};

}  // namespace

extern "C" {

// ============================================================================================ simulated units

iad_result IAD_CALL iad_sim_open(const char* options_json, iad_sim** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto options = sim_options_of(options_json);
        auto entry = std::make_shared<SimEntry>();
        entry->unit = std::make_shared<HookedSimUnit>(options.first, options.second);
        *out = reinterpret_cast<iad_sim*>(register_object(ObjectKind::Sim, entry));
    });
}

iad_result IAD_CALL iad_sim_close(iad_sim* sim) {
    return guarded([&] {
        if (!sim) return;
        bool other_kind = false;
        release_object(sim, ObjectKind::Sim, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_sim_close");
    });
}

iad_result IAD_CALL iad_sim_transport(iad_sim* sim, iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = register_transport(find_sim(sim));
    });
}

iad_result IAD_CALL iad_sim_port(iad_sim* sim, iad_transport** out) {  // 0.6
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = register_transport(std::make_shared<SimPort>(find_sim(sim)));
    });
}

iad_result IAD_CALL iad_sim_state_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        write_json(state_of(*find_sim(sim)), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_sim_requests_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        Json list = Json::array();
        for (const auto& r : find_sim(sim)->request_log()) {
            Json one = Json::object();
            one.set("t", r.first);
            one.set("frame", hex_frame(r.second));
            list.push_back(one);
        }
        write_json(list, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_sim_events_json(iad_sim* sim, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        write_json(Json(find_sim(sim)->events()), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_sim_power_cycle(iad_sim* sim, iad_tribool lose_home) {
    return guarded([&] {
        const auto u = find_sim(sim);
        const std::optional<bool> lose = tribool_arg(lose_home, "lose_home");
        if (lose)
            u->power_cycle(*lose);
        else
            u->power_cycle();
    });
}

iad_result IAD_CALL iad_sim_power_off(iad_sim* sim) {
    return guarded([&] { find_sim(sim)->power_off(); });
}

iad_result IAD_CALL iad_sim_power_on(iad_sim* sim, iad_tribool lose_home) {
    return guarded([&] {
        const auto u = find_sim(sim);
        u->power_on(tribool_arg(lose_home, "lose_home"));
    });
}

iad_result IAD_CALL iad_sim_unplug(iad_sim* sim) {
    return guarded([&] { find_sim(sim)->unplug(); });
}

iad_result IAD_CALL iad_sim_replug(iad_sim* sim) {
    return guarded([&] { find_sim(sim)->replug(); });
}

iad_result IAD_CALL iad_sim_set_word_override(iad_sim* sim, int32_t address, int32_t value) {
    return guarded([&] {
        const auto u = find_sim(sim);
        if (value != IAD_INT_UNKNOWN && (value < 0 || value > 0xFFFF))
            throw std::out_of_range("value must be 0..0xFFFF (IAD_INT_UNKNOWN: model it again), got " +
                                    std::to_string(value));
        u->set_word_override(address, value == IAD_INT_UNKNOWN ? std::nullopt
                                                               : std::optional<std::uint16_t>(static_cast<std::uint16_t>(value)));
    });
}

iad_result IAD_CALL iad_sim_set_unreadable(iad_sim* sim, int32_t start, int32_t count, iad_bool unreadable) {
    return guarded([&] { find_sim(sim)->set_unreadable(start, count, unreadable != 0); });
}

iad_result IAD_CALL iad_sim_set_stale_status_polls(iad_sim* sim, int32_t polls) {
    return guarded([&] { find_sim(sim)->set_stale_status_polls(polls); });
}

iad_result IAD_CALL iad_sim_register_value(iad_sim* sim, int32_t address, uint16_t* value) {
    return guarded([&] {
        need(value, "value");
        const auto u = find_sim(sim);
        if (address < 0 || address > 0xFFFF)
            throw std::out_of_range("register address must be 0..0xFFFF, got " + std::to_string(address));
        *value = u->hooked_register_value(address);
    });
}

iad_result IAD_CALL iad_sim_handle_frame(iad_sim* sim, const uint8_t* request, size_t size, uint8_t* buffer,
                                         size_t buffer_size, size_t* length) {
    return guarded([&] {
        need(length, "length");
        *length = 0;
        if (!buffer && buffer_size > 0) throw std::invalid_argument("buffer is NULL with buffer_size > 0");
        if (size) need(request, "request");
        const auto u = find_sim(sim);
        const std::vector<std::uint8_t> answer = u->handle(std::vector<std::uint8_t>(request, request + size));
        *length = answer.size();
        if (answer.size() > buffer_size)
            throw BufferTooSmall("the answer is " + std::to_string(answer.size()) + " bytes, the buffer " +
                                 std::to_string(buffer_size));
        if (!answer.empty()) std::memcpy(buffer, answer.data(), answer.size());
    });
}

iad_result IAD_CALL iad_sim_set_request_hook(iad_sim* sim, iad_sim_request_hook hook, void* user) {
    return guarded([&] {
        const auto u = find_sim(sim);
        if (!hook) {
            u->set_request_hook(SimRequestHook());
            return;
        }
        u->set_request_hook([hook, user](const std::vector<std::uint8_t>& frame) {
            std::uint8_t reply[kHookReplyCapacity];
            std::size_t length = 0;
            const int32_t r = hook(frame.data(), frame.size(), reply, sizeof reply, &length, user);
            SimHookAnswer a;
            if (r == IAD_SIM_DROP) {
                a.kind = SimHookAnswer::Kind::Drop;
            } else if (r == IAD_SIM_REPLY) {
                a.kind = SimHookAnswer::Kind::Reply;
                a.reply.assign(reply, reply + (length < sizeof reply ? length : sizeof reply));
            } else if ((r & ~0xFF) == 0x100 && (r & 0xFF) != 0) {
                a.kind = SimHookAnswer::Kind::Exception;
                a.exception_code = r & 0xFF;
            } else if (r == IAD_SIM_FAIL) {  // 0.6: the request fails as on a port that fails, with the hook's text
                const std::size_t n = length < sizeof reply ? length : sizeof reply;
                const std::string why(reinterpret_cast<const char*>(reply), n);
                throw TransportError(why.empty() ? "the simulator's request hook failed" : why);
            } else if (r == IAD_SIM_ABORT) {  // (0.7, py-fixes) the caller's own error: not the line's, not retried
                const std::size_t n = length < sizeof reply ? length : sizeof reply;
                const std::string why(reinterpret_cast<const char*>(reply), n);
                throw CallerAbortError(why.empty() ? "the simulator's request hook raised" : why);
            }
            return a;
        });
    });
}

iad_result IAD_CALL iad_sim_control_json(iad_sim* sim, const char* command_json, char* buffer, size_t buffer_size,
                                         size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto u = find_sim(sim);
        need(command_json, "command_json");
        write_json(control(*u, parse_object(command_json, "the command")), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_sim_set_clock(iad_sim* sim, iad_clock_now_fn now, void* user) {
    return guarded([&] {
        const auto u = find_sim(sim);
        if (!now) {
            u->set_clock(std::function<double()>());
            return;
        }
        u->set_clock([now, user] { return now(user); });
    });
}

// ============================================================================================ simulated lines

iad_result IAD_CALL iad_sim_line_open(const char* name, iad_sim* const* units, size_t unit_count, iad_sim_line** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (unit_count) need(units, "units");
        std::vector<std::shared_ptr<SimulatedUnit>> list;
        for (std::size_t i = 0; i < unit_count; ++i) list.push_back(find_sim(units[i]));
        auto entry = std::make_shared<LineEntry>();
        entry->line = std::make_shared<SimLine>(std::move(list), name && *name ? name : "SIM");
        *out = reinterpret_cast<iad_sim_line*>(register_object(ObjectKind::SimLine, entry));
    });
}

iad_result IAD_CALL iad_sim_line_close(iad_sim_line* line) {
    return guarded([&] {
        if (!line) return;
        bool other_kind = false;
        release_object(line, ObjectKind::SimLine, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_sim_line_close");
    });
}

iad_result IAD_CALL iad_sim_line_add(iad_sim_line* line, iad_sim* sim) {
    return guarded([&] {
        const auto l = find_line(line);
        l->add(find_sim(sim));
    });
}

iad_result IAD_CALL iad_sim_line_remove(iad_sim_line* line, int32_t slave) {
    return guarded([&] { find_line(line)->remove(slave); });
}

iad_result IAD_CALL iad_sim_line_transport(iad_sim_line* line, iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = register_transport(find_line(line));
    });
}

iad_result IAD_CALL iad_sim_line_port(iad_sim_line* line, iad_transport** out) {  // 0.6
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = register_transport(std::make_shared<SimPort>(find_line(line)));
    });
}

iad_result IAD_CALL iad_sim_gateway_line_open(const char* kind, const char* options_json, iad_sim* const* units,
                                              size_t unit_count, iad_sim_line** out) {  // 0.6
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (unit_count) need(units, "units");
        SimGatewayOptions options = gateway_options_of(kind, options_json);
        std::vector<std::shared_ptr<SimulatedUnit>> list;
        for (std::size_t i = 0; i < unit_count; ++i) list.push_back(find_sim(units[i]));
        auto entry = std::make_shared<LineEntry>();
        entry->line = std::make_shared<SimGatewayLine>(std::move(list), std::move(options));
        *out = reinterpret_cast<iad_sim_line*>(register_object(ObjectKind::SimLine, entry));
    });
}

iad_result IAD_CALL iad_sim_line_requests_json(iad_sim_line* line, char* buffer, size_t buffer_size,
                                               size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        Json list = Json::array();
        for (const auto& r : find_line(line)->requests()) {
            Json one = Json::object();
            one.set("t", r.first);
            one.set("slave", r.second.empty() ? Json() : Json(static_cast<int>(r.second[0])));
            one.set("frame", hex_frame(r.second));
            list.push_back(one);
        }
        write_json(list, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_sim_line_control_json(iad_sim_line* line, const char* command_json, char* buffer,
                                              size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto l = find_line(line);
        need(command_json, "command_json");
        write_json(line_control(*l, parse_object(command_json, "the command")), buffer, buffer_size, required);
    });
}

}  // extern "C"
