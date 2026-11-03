// iai_actuator_driver - unofficial C++ driver for IAI actuators over serial Modbus RTU (ELECYLINDER units and CON controllers
// with RoboCylinder), so PC software can command up to 16 axes instead of PLC I/O. Not an IAI product.
//
// Umbrella header: everything a customer program needs.
//
//   #include "iai_actuator_driver/iai_actuator_driver.hpp"
//   using namespace iai_actuator_driver;
//   Axis ax = Axis::open("COM3");          // baud rate found automatically
//   ax.connect();                          // identity, ranges, Modbus control
//   ax.servo(true);
//   ax.home();
//   ax.move_to(50.0, 20.0);                // 50 mm at 20 mm/s, waits until in position
//   ax.stop();
//   ax.servo(false);
//
// No hardware: Axis::open(std::make_shared<SimulatedUnit>(SimProfile::Gripper)).
#pragma once

#include "iai_actuator_driver/version.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/fake_transport.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/alarms.hpp"
#include "iai_actuator_driver/status.hpp"
#include "iai_actuator_driver/directory.hpp"
#include "iai_actuator_driver/device_info.hpp"
#include "iai_actuator_driver/position.hpp"
#include "iai_actuator_driver/parameters.hpp"
#include "iai_actuator_driver/results.hpp"
#include "iai_actuator_driver/nonvolatile.hpp"
#include "iai_actuator_driver/json.hpp"
#include "iai_actuator_driver/axis.hpp"
#include "iai_actuator_driver/info.hpp"
#include "iai_actuator_driver/check.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/move_handle.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/cell.hpp"
#include "iai_actuator_driver/coord.hpp"
#include "iai_actuator_driver/sim.hpp"
#include "iai_actuator_driver/sim_line.hpp"
#include "iai_actuator_driver/sim_gateway.hpp"
