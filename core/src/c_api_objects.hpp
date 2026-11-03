// Private helpers of the C ABI 0.3 handles (transports, buses, simulators, simulated lines) and the cancel latch,
// shared by src/c_api_bus.cpp and src/c_api_sim.cpp. Internal: not installed, not part of the ABI.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/sim_hooks.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace capi {

/// iad_transport: a transport shared with whatever uses it (a bus, an axis, a cell, a line).
struct TransportEntry {
    std::shared_ptr<Transport> transport;
};

/// iad_bus: `owned` from iad_bus_open (closing the handle closes the bus); otherwise borrowed from an axis or cell.
/// (0.4) `suspensions`: this handle's iad_bus_suspend_monitors, the last one last; released with the handle.
struct BusEntry {
    std::shared_ptr<ModbusRtu> bus;
    bool owned = false;
    std::mutex suspend_mutex;
    std::vector<BusSuspension> suspensions;  // guarded by suspend_mutex
    /// (0.5, py-cell) the port of iad_bus_set_port (or of the first port call on this handle): it lives with the handle
    std::shared_ptr<Port> port;              // guarded by c_api_cell.cpp's port mutex
};

/// iad_sim.
struct SimEntry {
    std::shared_ptr<HookedSimUnit> unit;
};

/// iad_sim_line.
struct LineEntry {
    std::shared_ptr<SimLine> line;
};

/// The transport of a live iad_transport (std::invalid_argument otherwise).
std::shared_ptr<Transport> find_transport(iad_transport* handle);
/// A new iad_transport on `transport`.
iad_transport* register_transport(std::shared_ptr<Transport> transport);

/// iad_set_log_level: lines below it are not passed to the log sink.
std::atomic<int32_t>& log_level();

/// find_axis for a call that waits or moves: WaitCancelledError (IAD_ERR_CANCELLED, nothing sent) while the axis's
/// cancel latch is set (iad_axis_cancel_latch).
std::shared_ptr<AxisEntry> find_axis_to_wait(iad_axis* handle);
/// The same check on an axis object (a move handle's axis).
void check_cancel_latch(const Axis& axis);

}  // namespace capi
}  // namespace iai_actuator_driver
