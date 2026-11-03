// The read-only view of one unit that the model layer (directory, identify, limits, parameters) works through. The bus
// implements it (ModbusRtu::reader); tests implement it over a map of words. Nothing here writes to a unit.
#pragma once

#include <cstdint>
#include <vector>

namespace iai_actuator_driver {

/// Read-only access to one slave. Implementations throw the ModbusError family (ModbusTimeout, ModbusSlaveError...).
class UnitReader {
public:
    virtual ~UnitReader() = default;

    /// The slave address (1..247).
    virtual int slave() const = 0;

    /// FC03: `count` holding registers from `address` (a long read is split by the bus).
    virtual std::vector<std::uint16_t> read_registers(int address, int count) = 0;

    /// FC11 Report Slave ID: the data bytes after the byte count ([system, run status, unit, fw hi, fw lo] on every
    /// unit tested). Sent once, no retries, waits up to kSlaveIdTimeoutS (0.65 s: the EC rod answers after 0.2-0.4 s,
    /// VERIFIED in the Python edition). Throws ModbusTimeout when silent, ModbusSlaveError when refused.
    virtual std::vector<std::uint8_t> report_slave_id() = 0;
};

}  // namespace iai_actuator_driver
