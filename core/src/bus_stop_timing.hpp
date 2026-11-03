// Private to the core (internal: not installed, not part of the API): the stop latency line. Every stop of an axis
// (stop(), begin_stop(), the decel stop after an error) and every broadcast STOP stamps the call and its first
// stop-class frame on the wire (detail::StopWireStamp, modbus.hpp) and, where the stop holds until standstill, the
// standstill the driver confirmed; one info line says how long each took, e.g. "COM3 slave 1: STOP on the wire 7 ms
// after the call, standstill confirmed 180 ms later". The line is logged when the stop ends, on the thread that stopped
// (log.hpp: handlers run on the thread that logs), never between a STOP frame and the next one of a loop that stops
// several axes (StopLinesLater).
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/modbus.hpp"

namespace iai_actuator_driver {
namespace detail {

/// RAII: while it lives, the stop latency lines of the calling thread are kept and logged, in order and on this
/// thread, when it ends: a loop that sends the first STOP frame to several axes back to back runs no log handler
/// between two of them. Nestable (the outermost scope logs). Not copyable.
class StopLinesLater {
public:
    StopLinesLater();
    ~StopLinesLater();
    StopLinesLater(const StopLinesLater&) = delete;
    StopLinesLater& operator=(const StopLinesLater&) = delete;

private:
    std::vector<std::pair<std::string, std::string>> lines_;  // (source, message)
    StopLinesLater* previous_;
    friend class StopTiming;
};

/// RAII: the timing of one stop (see the file comment) on `bus` to `slave` (0: a broadcast); `what` names the stop
/// ("STOP", "decel stop", "broadcast STOP"); `holds`: the stop waits for standstill, which hold_until_stopped notes for
/// `owner` (the axis). Logs its line at info, from `source`, when it ends (also when the stop throws). Nestable; not
/// copyable.
class StopTiming {
public:
    StopTiming(const ModbusRtu& bus, int slave, const void* owner, const char* what, bool holds, const char* source);
    ~StopTiming();
    StopTiming(const StopTiming&) = delete;
    StopTiming& operator=(const StopTiming&) = delete;

    /// Standstill confirmed now for `owner` (an axis): every open timing of this thread for it notes the first one.
    static void standstill(const void* owner);
    /// The line as it would be logged now.
    std::string line() const;

private:
    StopWireStamp wire_;
    const ModbusRtu& bus_;
    int slave_;
    const void* owner_;
    const char* what_;
    bool holds_;
    const char* source_;
    double call_s_;
    std::optional<double> still_s_;
    StopTiming* previous_;
};

}  // namespace detail
}  // namespace iai_actuator_driver
