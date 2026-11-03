// C ABI 0.5, py-cell block: the port of a bus (a C++ Port over a bus that is open already: the baud rate found once,
// the line, the scan, one address's identity, the line-wide baud change) and cells over buses or over axes that exist
// already, the way the Python and C# Cell / Port facades hold them (a Port object per bus, Port::axis objects in the
// cell), with the group commands' options those facades have. Simulator tests only; several axes per port is
// UNVERIFIED on hardware.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/reader.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {

/// The port of a bus lives with the bus handle it was made for (BusEntry::port: the handle of the binding's bus
/// object) and in the cells that use it. Other handles on the same bus (an axis's, a cell's) find it through this
/// list, which never keeps a port alive (nothing of a bus outlives iad_shutdown).
std::mutex g_ports_mutex;
std::vector<std::weak_ptr<Port>> g_ports;  // guarded by g_ports_mutex

std::shared_ptr<BusEntry> bus_entry(iad_bus* handle) {
    std::shared_ptr<void> entry = find_object(handle, ObjectKind::Bus);
    if (!entry) throw std::invalid_argument("invalid bus handle");
    return std::static_pointer_cast<BusEntry>(entry);
}

/// The live port made for `bus` (on any handle), or null; drops the ports that are gone (g_ports_mutex held).
std::shared_ptr<Port> find_port_locked(const std::shared_ptr<ModbusRtu>& bus) {
    std::shared_ptr<Port> found;
    g_ports.erase(std::remove_if(g_ports.begin(), g_ports.end(),
                                 [&](const std::weak_ptr<Port>& w) {
                                     const std::shared_ptr<Port> p = w.lock();
                                     if (p && !found && p->bus() == bus) found = p;
                                     return !p;
                                 }),
                  g_ports.end());
    return found;
}

std::shared_ptr<Port> known_port(const std::shared_ptr<ModbusRtu>& bus) {
    std::lock_guard<std::mutex> lock(g_ports_mutex);
    return find_port_locked(bus);
}

/// The port of a bus handle: the one iad_bus_set_port made (or an earlier call, also on another handle of the bus),
/// else one with the defaults (the transport's name, a probed line, the rate detected once), kept by this handle.
std::shared_ptr<Port> port_of(const std::shared_ptr<BusEntry>& entry) {
    std::lock_guard<std::mutex> lock(g_ports_mutex);
    if (entry->port) return entry->port;
    if (std::shared_ptr<Port> port = find_port_locked(entry->bus)) return port;
    entry->port = std::make_shared<Port>(entry->bus, std::string(), LineOptions{}, false);
    g_ports.push_back(entry->port);
    return entry->port;
}

int32_t kind_code(LineKind kind) {
    switch (kind) {
        case LineKind::Rcon: return IAD_LINE_RCON;
        case LineKind::Robonet: return IAD_LINE_ROBONET;
        case LineKind::RobonetGateway: return IAD_LINE_ROBONET_GATEWAY;
        default: return IAD_LINE_DIRECT;
    }
}

std::vector<int> slave_list(const char* slaves) {
    std::vector<int> list = to_int_list(slaves, "slaves");
    if (list.empty()) return default_scan_slaves();
    return list;
}

void write_json(const Json& json, char* buffer, size_t buffer_size, size_t* required) {
    write_text_output(json.dump(), buffer, buffer_size, required);
}

std::vector<int> int_values(const Json& list, const std::string& what) {
    if (list.type() != Json::Type::Array) throw std::invalid_argument(what + " must be a list of slave addresses");
    std::vector<int> out;
    for (const Json& v : list.as_array()) {
        if (v.type() != Json::Type::Int) throw std::invalid_argument(what + " must be a list of slave addresses");
        out.push_back(static_cast<int>(v.as_int()));
    }
    return out;
}

/// iad_cell_open_buses' `slaves_json`: nullopt = scan; a list = these on every port; {port: list}.
std::optional<std::map<std::string, std::vector<int>>> slaves_by_port(const char* slaves_json,
                                                                      const std::vector<std::shared_ptr<Port>>& ports) {
    const std::string text = slaves_json ? std::string(slaves_json) : std::string();
    if (text.find_first_not_of(" \t\r\n") == std::string::npos || text == "scan" || text == "\"scan\"")
        return std::nullopt;
    const Json j = Json::parse(text);
    std::map<std::string, std::vector<int>> out;
    if (j.type() == Json::Type::Array) {
        const std::vector<int> every = int_values(j, "slaves");
        for (const auto& p : ports) out[p->name()] = every;
    } else if (j.type() == Json::Type::Object) {
        for (const auto& kv : j.as_object()) out[kv.first] = int_values(kv.second, "slaves of " + kv.first);
    } else {
        throw std::invalid_argument("slaves_json must be \"scan\", a list of slaves or {port: [slaves]}");
    }
    return out;
}

/// A new cell handle on `make`'s cell, its on_fault / on_event queued on the handle (iad_cell_next_event).
template <class Make>
iad_cell* new_cell(const iad_cell_options* options, const iad_axis_options* axis_options, Make make) {
    auto entry = std::make_shared<CellEntry>();
    const CellOptions o = queueing_cell_options(to_cell_options(options, axis_options), entry->events);
    entry->cell = std::make_shared<Cell>(make(o));
    return register_cell(std::move(entry));
}

/// A group command's per-axis results into the results array; CellError (IAD_ERR_CELL) when an axis failed.
void finish_results(const CellResults<Status>& r, iad_cell_result* results, size_t capacity, size_t* count) {
    if (fill_cell_results(r.names, r.results, r.errors, results, capacity, count))
        throw CellError(r.operation, r.messages, r.errors);
}

/// {"results": {axis: value}, "errors": {axis: the error as iad_last_error_json gives it}} of a read-only group read.
template <class T, class ToJson>
Json group_json(const CellResults<T>& r, ToJson to) {
    Json results = Json::object();
    for (const auto& kv : r.results) results.set(kv.first, to(kv.second));
    Json errors = Json::object();
    for (const auto& kv : r.errors) errors.set(kv.first, describe_error(kv.second).json);
    Json j = Json::object();
    j.set("results", std::move(results));
    j.set("errors", std::move(errors));
    return j;
}

}  // namespace

namespace iai_actuator_driver {
namespace capi {

std::shared_ptr<CellEntry> find_cell_to_run(iad_cell* handle) {
    std::shared_ptr<CellEntry> e = find_cell(handle);
    for (const auto& kv : e->cell->ports()) {
        const std::shared_ptr<ModbusRtu> bus = kv.second->bus();
        if (bus && bus->bus_lock().owned_by_this_thread())
            throw EcNotReadyError("the calling thread holds the bus lock of port " + kv.first +
                                  " (iad_bus_lock): a cell's group command runs its axes on threads of the core, which "
                                  "would wait for that hold forever; nothing sent (iad_bus_unlock first)");
    }
    return e;
}

}  // namespace capi
}  // namespace iai_actuator_driver

extern "C" {

// ============================================================================================ the port of a bus

iad_result IAD_CALL iad_bus_set_port(iad_bus* bus, const char* name, int32_t line, double position_unit_mm,
                                     iad_bool fixed_rate, iad_port_line_fn detect_line, void* user) {
    return guarded([&] {
        const auto entry = bus_entry(bus);
        const LineOptions o = to_line_options(line, position_unit_mm);
        LineDetector detector;
        if (detect_line) {
            detector = [detect_line, user]() {
                const iad_result r = detect_line(user);
                if (r != IAD_OK)
                    throw EcDriverError(std::string("the owner's line detection failed (") + iad_result_name(r) + ")");
            };
        }
        auto port = std::make_shared<Port>(entry->bus, name ? std::string(name) : std::string(), o, fixed_rate != 0,
                                           std::move(detector));
        std::lock_guard<std::mutex> lock(g_ports_mutex);
        g_ports.erase(std::remove_if(g_ports.begin(), g_ports.end(),
                                     [&](const std::weak_ptr<Port>& w) {
                                         const std::shared_ptr<Port> p = w.lock();
                                         return !p || p->bus() == entry->bus;  // the earlier port of this bus goes
                                     }),
                      g_ports.end());
        entry->port = port;
        g_ports.push_back(std::move(port));
    });
}

iad_result IAD_CALL iad_bus_port_info_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        Json j = Json::object();
        j.set("name", port->name());
        const std::optional<int> rate = port->baud_rate();
        j.set("baud_rate", rate ? Json(*rate) : Json());
        j.set("baud_detected", port->baud_detected());
        j.set("line", to_string(port->line()->kind()));
        j.set("closed", port->closed());
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_bus_ensure_baud_rate(iad_bus* bus, const char* slaves, int32_t* rate, iad_bool* detected) {
    return guarded([&] {
        if (rate) *rate = IAD_INT_UNKNOWN;
        if (detected) *detected = IAD_FALSE;
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const std::optional<int> found = port->ensure_baud_rate(slave_list(slaves));
        if (rate && found) *rate = *found;
        if (detected) *detected = port->baud_detected() ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_bus_port_detect_line(iad_bus* bus, iad_bool force, int32_t* kind) {
    return guarded([&] {
        if (kind) *kind = IAD_LINE_DIRECT;
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const std::shared_ptr<Line> line = port->detect_line(force != 0);
        if (kind) *kind = kind_code(line->kind());
    });
}

iad_result IAD_CALL iad_bus_scan(iad_bus* bus, const char* slaves, double timeout_s, iad_bool use_link_map,
                                 int32_t* found, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        if (!found && capacity > 0) throw std::invalid_argument("found is NULL with capacity > 0");
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const std::vector<int> hits =
            port->scan(slave_list(slaves), timeout_arg(timeout_s, "timeout_s"), 0x9000, use_link_map != 0);
        if (count) *count = hits.size();
        for (size_t i = 0; found && i < hits.size() && i < capacity; ++i) found[i] = hits[i];
    });
}

iad_result IAD_CALL iad_bus_scan_units_json(iad_bus* bus, const char* slaves, double timeout_s, iad_bool identify,
                                            iad_bool use_link_map, char* buffer, size_t buffer_size,
                                            size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const std::vector<ScanHit> hits = port->scan_units(slave_list(slaves), timeout_arg(timeout_s, "timeout_s"),
                                                           identify != 0, use_link_map != 0);
        Json list = Json::array();
        for (const auto& h : hits) list.push_back(to_json(h));
        write_json(list, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_bus_identify_json(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                          size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        write_json(to_json(port->identify(slave)), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_bus_set_baud_rate_all(iad_bus* bus, int32_t rate, const char* slaves,
                                              const iad_baud_options* options, char* buffer, size_t buffer_size,
                                              size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const PortBaudResult r = port->set_baud_rate(rate, to_int_list(slaves, "slaves"), to_baud_options(options));
        write_json(to_json(r), buffer, buffer_size, required);
    });
}

// ============================================================================================ cells

iad_result IAD_CALL iad_cell_open_buses(iad_bus* const* buses, size_t bus_count, const char* slaves_json,
                                        const char* scan_slaves, const iad_cell_options* options,
                                        const iad_axis_options* axis_options, iad_cell** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (bus_count < 1) throw std::invalid_argument("bus_count must be >= 1");
        need(buses, "buses");
        std::vector<std::shared_ptr<Port>> ports;
        for (size_t i = 0; i < bus_count; ++i) {
            const std::shared_ptr<Port> port = port_of(bus_entry(buses[i]));
            for (const auto& p : ports)
                if (p == port || p->name() == port->name())
                    throw std::invalid_argument("two buses are port " + port->name() + " (give each its own name)");
            ports.push_back(port);
        }
        const auto slaves = slaves_by_port(slaves_json, ports);
        const std::vector<int> scan = to_int_list(scan_slaves, "scan_slaves");
        *out = new_cell(options, axis_options, [&](CellOptions o) {
            if (!scan.empty()) o.scan_slaves = scan;
            return Cell::open(ports, slaves, o);
        });
    });
}

iad_result IAD_CALL iad_cell_open_axes(iad_axis* const* axes, size_t axis_count, const char* names,
                                       const iad_cell_options* options, iad_cell** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (axis_count < 1) throw std::invalid_argument("axis_count must be >= 1");
        need(axes, "axes");
        const std::vector<std::string> given = split_list(names, false);
        if (!given.empty() && given.size() != axis_count)
            throw std::invalid_argument("names lists " + std::to_string(given.size()) + " names for " +
                                        std::to_string(axis_count) + " axes");
        std::vector<CellMember> members;
        std::map<const ModbusRtu*, std::shared_ptr<Port>> made;  // buses without a port: one each, kept by the cell
        for (size_t i = 0; i < axis_count; ++i) {
            const std::shared_ptr<Axis> axis = find_axis(axes[i])->axis;
            const std::shared_ptr<ModbusRtu> bus = axis->bus();
            std::shared_ptr<Port> port = known_port(bus);
            if (!port) {
                std::shared_ptr<Port>& slot = made[bus.get()];
                if (!slot) {
                    std::string pname;
                    if (!given.empty()) pname = given[i].substr(0, given[i].rfind(':'));
                    slot = std::make_shared<Port>(bus, pname, LineOptions{}, true, [] {});  // the line as the bus has it
                }
                port = slot;
            }
            CellMember m;
            m.name = given.empty() ? axis_name(port->name(), axis->slave()) : given[i];
            m.port = port;
            m.axis = axis;
            members.push_back(std::move(m));
        }
        *out = new_cell(options, nullptr, [&](const CellOptions& o) { return Cell::assemble(members, o); });
    });
}

iad_result IAD_CALL iad_cell_axis_ref(iad_cell* cell, const char* name, iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        need(name, "name");
        const auto e = find_cell(cell);
        iad_axis* handle = register_axis(e->cell->axis_ptr(name));
        find_axis(handle)->owned = false;  // closing this handle leaves the axis as it is (the cell's)
        *out = handle;
    });
}

iad_result IAD_CALL iad_cell_release(iad_cell* cell) {
    return guarded([&] {
        if (!cell) return;
        bool other_kind = false;
        std::vector<std::shared_ptr<AxisEntry>> axes;
        const std::shared_ptr<CellEntry> entry = release_cell(cell, &other_kind, &axes);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_cell_release");
        for (const auto& a : axes) a->events->close();
        if (!entry) return;
        entry->events->close();
        std::shared_ptr<recovery::LinkMonitor> links;
        {
            std::lock_guard<std::mutex> lock(entry->mutex);
            links = std::move(entry->link_monitor);
        }
        if (links) links->stop();
        // the last reference goes with `entry`: a cell of iad_cell_open_axes leaves its axes and ports open
    });
}

iad_result IAD_CALL iad_cell_port_errors_json(iad_cell* cell, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto e = find_cell(cell);
        Json j = Json::object();
        for (const auto& kv : e->cell->port_error_details()) j.set(kv.first, describe_error(kv.second).json);
        write_json(j, buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_cell_read_all_json(iad_cell* cell, const char* what, const char* axes, char* buffer,
                                           size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        need(what, "what");
        const auto e = find_cell_to_run(cell);
        const std::string kind(what);
        Json j;
        if (kind == "identify") {
            j = group_json(e->cell->identify_all(split_list(axes), true), [](const DeviceInfo& d) { return to_json(d); });
        } else if (kind == "limits") {
            j = group_json(e->cell->limits_all(split_list(axes), true), [](const AxisRanges& r) { return to_json(r); });
        } else if (kind == "monitor") {
            j = group_json(e->cell->monitor_all(split_list(axes), true), [](const Json& r) { return r; });
        } else {
            throw std::invalid_argument("what must be \"identify\", \"limits\" or \"monitor\", got \"" + kind + "\"");
        }
        write_json(j, buffer, buffer_size, required);
    });
}

// (0.8) The group commands with the options of the Python and C# facades (0.5 added them as _ex functions; folded
// into the base functions at 0.8).

iad_result IAD_CALL iad_cell_status_all(iad_cell* cell, const char* axes, double monitor_max_age_s,
                                        iad_cell_result* results, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        std::optional<double> age;
        if (!std::isnan(monitor_max_age_s)) {
            if (!(monitor_max_age_s >= 0.0) || !std::isfinite(monitor_max_age_s))
                throw std::invalid_argument("monitor_max_age_s must be >= 0 (NaN: always read)");
            age = monitor_max_age_s;
        }
        finish_results(e->cell->status_all(split_list(axes), true, age), results, capacity, count);
    });
}

iad_result IAD_CALL iad_cell_home_all(iad_cell* cell, const char* axes, double timeout_s, int32_t method,
                                      iad_bool stop_others_on_error, iad_cell_result* results, size_t capacity,
                                      size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        finish_results(e->cell->home_all(split_list(axes), timeout_arg(timeout_s, "timeout_s"), true, to_method(method),
                                         stop_others_on_error != 0),
                       results, capacity, count);
    });
}

iad_result IAD_CALL iad_cell_move_each(iad_cell* cell, const iad_axis_move* moves, size_t move_count, iad_bool wait,
                                       double timeout_s, iad_bool stop_others_on_error, iad_cell_result* results,
                                       size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        if (move_count == 0) throw std::invalid_argument("moves: iad_cell_move_each needs at least one move");
        need(moves, "moves");
        const std::vector<AxisMove> list = to_axis_moves(moves, move_count);
        std::vector<std::string> names;
        for (const AxisMove& a : list) names.push_back(a.axis);
        std::optional<CellResults<Status>> r;
        try {
            r = e->cell->move_each(list, wait != 0, timeout_arg(timeout_s, "timeout_s"), true,
                                   stop_others_on_error != 0);
        } catch (const CellError& refused) {
            // refused at the check (nothing started) or a start failed (the others stopped): every axis is reported
            fill_cell_results(names, {}, refused.errors(), results, capacity, count);
            throw;
        }
        if (!r) return;  // started without waiting: *count stays 0, results unwritten
        if (fill_cell_results(names, r->results, r->errors, results, capacity, count))
            throw CellError(r->operation, r->messages, r->errors);
    });
}

iad_result IAD_CALL iad_cell_wait_all(iad_cell* cell, const char* axes, double timeout_s,
                                      iad_bool stop_others_on_error, iad_cell_result* results, size_t capacity,
                                      size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        finish_results(e->cell->wait_all(split_list(axes), timeout_arg(timeout_s, "timeout_s"), true,
                                         stop_others_on_error != 0),
                       results, capacity, count);
    });
}

iad_result IAD_CALL iad_cell_stop_all(iad_cell* cell, const char* axes, double timeout_s, iad_bool broadcast,
                                      iad_cell_result* results, size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);
        finish_results(e->cell->stop_all(split_list(axes), or_default(timeout_s, 5.0), true, broadcast != 0), results,
                       capacity, count);
    });
}

// ============================================================================================ ABI 0.6 (csharp)

iad_result IAD_CALL iad_bus_port_detect_baud_rate(iad_bus* bus, const char* slaves, const char* candidates,
                                                  double probe_timeout_s, int32_t* rate) {
    return guarded([&] {
        if (rate) *rate = IAD_INT_UNKNOWN;
        const std::shared_ptr<Port> port = port_of(bus_entry(bus));
        const int found = port->detect_baud_rate(slave_list(slaves), to_int_list(candidates, "candidates"),
                                                 timeout_arg(probe_timeout_s, "probe_timeout_s"));
        if (rate) *rate = found;
    });
}

iad_result IAD_CALL iad_bus_read_device_info_json(iad_bus* bus, int32_t slave, char* buffer, size_t buffer_size,
                                                  size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        if (slave < 1 || slave > 247) throw std::out_of_range("slave address must be 1..247");
        const std::unique_ptr<UnitReader> reader = bus_entry(bus)->bus->reader(slave);
        write_json(to_json(read_device_info(*reader)), buffer, buffer_size, required);
    });
}

}  // extern "C"
