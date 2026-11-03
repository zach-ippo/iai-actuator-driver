// Modbus RTU master for IAI units over a Transport, plus the IAI function 0x45 control commands the editions use,
// baud-rate detection and the broadcast baud change.
//
// Design rules (ported from the Python edition, iai_actuator_driver/modbus.py; VERIFIED there unless marked):
// * One transaction at a time per port (a mutex), at least min_gap_s (default 5 ms) of silence between requests. A
//   request waits out the gap before it takes the bus; what is left once it has it (the bus was handed over right
//   after a frame) it waits with the bus held, and a stop-class request that queued meanwhile then goes first. A
//   stop-class request (bus_lock.hpp) queues at once and spins on the clock for what is left of the gap (at most
//   5 ms) rather than sleeping, so a STOP is never kept waiting by another request's gap or by a stretched sleep.
// * Timeouts, CRC errors and mismatched frames are retried: kRecommendedRetries = 3, "setting of the number of
//   retries is mandatory" (DOC ME0162-12A p.4-2). Exception responses and IAI rejections are valid answers and are
//   never retried; TransportError is never retried. Every driver write is idempotent (absolute targets, coil levels),
//   so a retry after a lost answer is harmless; pass retries 0 for a command that must not be sent twice (a reset).
//   A write is never sent again after a STOP that went between its attempts (STOP first, below).
// * Response timeout per request (DOC ME0162-12A p.4-1, 4-2): Tout = 3 x Tint + RTIM + 10 x (response bytes + 8) /
//   kbps [ms] + kUsbMarginS, Tint = 1 ms per register, RTIM = the unit's response delay (default 5 ms); never below
//   kMinFrameTimeoutS (0.2 s), and never below 0.5 s at 9600 baud or less / 0.35 s below 38400 (the 9600 slow-reply
//   floor: replies alternated ~65 ms / ~285 ms on the EC gripper at 9600, VERIFIED). Applies to a transport with
//   has_real_timeout(); an in-memory transport answers at once.
// * FC11 waits up to kSlaveIdTimeoutS (0.65 s): the EC-R6S rod answers it only after 0.2-0.4 s.
// * Broadcast (slave 0) is never answered: the IAI baud change 00 45 00 04 00 <code> 00 05 (ON-WIRE), FC05 / FC06 /
//   FC10 to every unit of a daisy chain (DOC; UNVERIFIED on a multi-drop line).
// * IAI commands not on the safe list need allow_unsafe; commands on the never-send list (non-volatile writes, axis
//   number, baud, alarm-list clear, initialisation, maintenance writes, firmware update, inspection) need
//   allow_never_send (UnsafeCommandError, nothing sent).
#pragma once

#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/reader.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

// ------------------------------------------------------------------------------------------ function codes / sizes

inline constexpr int kFcReadHoldingRegisters = 0x03;
inline constexpr int kFcWriteCoil = 0x05;
inline constexpr int kFcWriteRegister = 0x06;
inline constexpr int kFcWriteRegisters = 0x10;
inline constexpr int kFcReportSlaveId = 0x11;
inline constexpr int kFcReadWriteRegisters = 0x17;  ///< FC17: write, then read, in one frame (the command profile)
inline constexpr int kFcIaiCommand = 0x45;

/// Largest single FC03 read the driver issues; bigger reads are split.
inline constexpr int kMaxReadRegisters = 60;
/// FC10 must fit the controller's 256-byte receive buffer (DOC ME0162 p.5-4).
inline constexpr int kMaxWriteRegisters = 123;
/// Largest FC03 read Modbus allows in one frame (ReadOptions::chunk up to this; the RCON all-axes monitor reads 120).
inline constexpr int kMaxFc03Registers = 125;

class Line;  // line.hpp: what the line carries and the guard every request passes

/// Per-call options of ModbusRtu::read_registers (gateway probes, the RCON monitor).
struct ReadOptions {
    std::optional<int> retries;     ///< extra attempts (default: the bus's retries())
    std::optional<double> max_wait_s;  ///< wait at most this long per attempt (a probe: a missing unit costs about this)
    std::optional<int> chunk;       ///< registers per FC03 frame, 1..kMaxFc03Registers (default kMaxReadRegisters)
};

/// Per-call options of ModbusRtu::write_registers (a coordinated run's frames, coord.hpp: no retry inside the bus and
/// a bounded wait for the echo).
struct WriteOptions {
    std::optional<int> retries;        ///< extra attempts (default: the bus's retries())
    std::optional<double> max_wait_s;  ///< wait at most this long for the echo per attempt (> 0)
};

inline constexpr std::uint16_t kCoilOn = 0xFF00;
inline constexpr std::uint16_t kCoilOff = 0x0000;

// ------------------------------------------------------------------------------------------ timing

/// Retries the IAI Modbus manual asks for (DOC ME0162-12A p.4-2). The default of every bus in this edition.
inline constexpr int kRecommendedRetries = 3;
/// Default minimum silence between transactions (s).
inline constexpr double kDefaultMinGapS = 0.005;
/// FC11 answer wait (s).
inline constexpr double kSlaveIdTimeoutS = 0.65;
/// probe() / scan wait per address (s): 16 addresses in about 1 s.
inline constexpr double kScanTimeoutS = 0.05;
/// Slave 58 = 0x3A = ':', the Modbus ASCII start character. IAI units tell RTU from ASCII frame by frame, so every unit
/// on the line takes a request to slave 58 for an ASCII frame and raises alarm 0x05E "Delimiter error" (ON-WIRE on a
/// PCON-CB, 2026-11-03: slaves 33 and 57 no alarm, 58 alarm 0x05E). No IAI unit can use it: the bus refuses a request
/// to it (std::invalid_argument, nothing sent) and scans and baud detections skip it.
inline constexpr int kAsciiStartSlave = 0x3A;
/// Why kAsciiStartSlave is never sent to (the refusal and the scans' log line).
inline constexpr const char* kAsciiStartSlaveNote =
    "slave 58 (0x3A) is never used: a request to it starts with ':', the Modbus ASCII start, and every IAI unit on the "
    "line takes it for an ASCII frame and raises alarm 0x05E 'Delimiter error' (no IAI unit can use address 58)";
namespace detail {
/// `slaves` without kAsciiStartSlave, with one warning line (`log_source`, `where`) when it was listed. A list of
/// kAsciiStartSlave alone is returned as it is: the request to it is then refused with kAsciiStartSlaveNote.
std::vector<int> skip_ascii_start_slave(const std::vector<int>& slaves, const std::string& log_source,
                                        const std::string& where);
}  // namespace detail
/// Silence kept after a broadcast frame (driver choice, UNVERIFIED on a multi-drop line).
inline constexpr double kBroadcastTurnaroundS = 0.02;
/// After the broadcast baud change, before the port switches (ON-WIRE: IA-OS waited ~0.2 s).
inline constexpr double kBaudSettleS = 0.2;
/// Floor of response_timeout() (s): the read timeout every verified EC / PCON session used.
inline constexpr double kMinFrameTimeoutS = 0.2;
/// Floor at 9600 baud or less (s): the 9600 slow-reply floor (VERIFIED on the EC gripper after a line break).
inline constexpr double kSlowFloor9600S = 0.5;
/// Floor between 9600 and 38400 baud (s).
inline constexpr double kSlowFloorBelow38400S = 0.35;
/// Added to the manual's Tout for the USB adapter's latency and the PC's scheduling (s; driver choice).
inline constexpr double kUsbMarginS = 0.05;
/// RTIM default: EC item 8.4 / CON parameter No.17 (ms, DOC ME0162-12A p.4-1).
inline constexpr double kDefaultResponseDelayMs = 5.0;
/// Line BREAK used by the "break" baud-change method (s; DOC: >= 150 ms drops the unit to 9600; UNVERIFIED on an EC).
inline constexpr double kBreakS = 0.2;

// ------------------------------------------------------------------------------------------ baud rates

/// Every unit tested boots at 38400 (BRSL item 8.3 = code 6, VERIFIED); IA-OS switches it to 115200 (volatile).
inline constexpr int kPowerUpBaudRate = 38400;

/// One entry of the IAI baud-code table (op1 of 0x45 cmd 4; also BRSL item 8.3 / CON No.16 values).
struct BaudCode {
    int rate;
    int code;
    const char* evidence;
};

/// IAI baud codes. Only 0x14 (115200, ON-WIRE + VERIFIED) and 0x15 (230400, VERIFIED) are confirmed on every unit for the
/// direct broadcast change; 0x06 (38400) is unit-dependent (kUnitDependentBaudRates). 14400 / 28800 / 76800 are never
/// probed at connect.
inline constexpr BaudCode kIaiBaudCodes[] = {
    {9600, 0x04, "UNVERIFIED"},
    {19200, 0x05, "UNVERIFIED"},
    {38400, 0x06, "unit-dependent: taken by the EC-GRBP8M gripper and the PCON-CB (VERIFIED), ignored by the EC-R6S rod"},
    {57600, 0x13, "UNVERIFIED"},
    {115200, 0x14, "ON-WIRE + VERIFIED (EC-R6S rod, both ways)"},
    {230400, 0x15, "VERIFIED (EC-R6S rod)"},
    {14400, 0x11, "UNVERIFIED"},
    {28800, 0x12, "UNVERIFIED"},
    {76800, 0x07, "UNVERIFIED"},
};

/// Rates whose baud-change code is verified on hardware (the EC-R6S rod, sent direct at the current rate);
/// set_baud_rate sends any other rate that has a code with a warning (BaudChangeResult::code_verified false).
inline constexpr int kVerifiedBaudRates[] = {115200, 230400};

/// Rates whose code some units take and others ignore: 0x06 (38400) switched the EC-GRBP8M gripper and the PCON-CB
/// (VERIFIED) and was IGNORED by the EC-R6S rod. set_baud_rate sends them without the opt-in because it checks the
/// result: a unit that ignores the code keeps answering at its rate and the port is left there (VerificationError).
inline constexpr int kUnitDependentBaudRates[] = {38400};

/// Rates detect_baud_rate() tries after the current one (one read per rate).
inline constexpr int kBaudDetectOrder[] = {115200, 38400, 9600, 230400, 57600, 19200};

/// Every rate an IAI baud code names, kBaudDetectOrder first: what to probe when a unit does not answer at a new rate.
inline constexpr int kBaudProbeAll[] = {115200, 38400, 9600, 230400, 57600, 19200, 14400, 28800, 76800};

/// IAI baud code of `rate`, nullopt when the rate has none.
inline constexpr std::optional<int> baud_code(int rate) {
    for (const auto& entry : kIaiBaudCodes)
        if (entry.rate == rate) return entry.code;
    return std::nullopt;
}

/// Rate of an IAI baud code (e.g. BRSL 6 -> 38400), nullopt when unknown.
inline constexpr std::optional<int> baud_by_code(int code) {
    for (const auto& entry : kIaiBaudCodes)
        if (entry.code == code) return entry.rate;
    return std::nullopt;
}

/// True for a rate of kVerifiedBaudRates.
inline constexpr bool baud_rate_verified(int rate) {
    for (int verified : kVerifiedBaudRates)
        if (verified == rate) return true;
    return false;
}

/// True for a rate of kUnitDependentBaudRates.
inline constexpr bool baud_rate_unit_dependent(int rate) {
    for (int dependent : kUnitDependentBaudRates)
        if (dependent == rate) return true;
    return false;
}

/// True when the rate's code is verified on hardware on at least some units (kVerifiedBaudRates or
/// kUnitDependentBaudRates): what BaudChangeResult::code_verified reports.
inline constexpr bool baud_code_verified(int rate) { return baud_rate_verified(rate) || baud_rate_unit_dependent(rate); }

/// What a "baud change not taken" VerificationError appends: for 38400 with the unit still answering at the previous
/// rate, a note that this unit ignores the code (empty otherwise).
inline std::string baud_not_taken_hint(int rate, int found, int previous) {
    if (rate == 38400 && found == previous)
        return "; this unit ignores the 38400 code (the EC-R6S rod does too): a power cycle brings it back to 38400";
    return std::string();
}

/// kBaudDetectOrder as a vector.
inline std::vector<int> baud_detect_order() { return {std::begin(kBaudDetectOrder), std::end(kBaudDetectOrder)}; }

/// kBaudProbeAll as a vector.
inline std::vector<int> baud_probe_all() { return {std::begin(kBaudProbeAll), std::end(kBaudProbeAll)}; }

// ------------------------------------------------------------------------------------------ IAI command tables
// Implemented by the bus unit (src/modbus_tables.cpp), ported from registers.py (IAI_REJECTIONS, IAI_REJECTION_HINTS,
// IAI_COMMAND_REJECTION_HINTS, IAI_NEVER_SEND) and modbus.py (IAI_SAFE_COMMANDS).

/// Name of a 0x45 rejection code ("unknown rejection code" when not in the table).
std::string iai_rejection_name(int code);
/// The driver's short advice for a rejection of `command` ("" when none).
std::string iai_rejection_hint(int code, int command);
/// Description of a 0x45 command allowed without allow_unsafe, "" when not on the safe list.
std::string iai_safe_command(int command);
/// Why a 0x45 command is never sent without allow_never_send, "" when it is not on that list.
std::string iai_never_send(int command);

// ------------------------------------------------------------------------------------------ the master

/// Direction of a logged frame.
enum class FrameDirection { Tx, Rx, RxPartial };

/// "tx", "rx", "rx-partial".
inline const char* to_string(FrameDirection d) noexcept {
    return d == FrameDirection::Tx ? "tx" : d == FrameDirection::Rx ? "rx" : "rx-partial";
}

/// Optional (direction, frame) callback for debugging and captures. An exception it throws is logged and ignored.
using FrameLogger = std::function<void(FrameDirection, const std::vector<std::uint8_t>&)>;

/// Answer of an IAI 0x45 command: [slave] 45 [rejection] [status] (01 45 00 00 = accepted, ON-WIRE + VERIFIED).
struct IaiResponse {
    int rejection = 0;
    int status = 0;
};

/// IAI function 0x48 "ControlCommand2" (UNVERIFIED: the command set of units whose SUP3 IVC1 bit is set; none of the
/// ECs tested has it, they all answer 0x45).
inline constexpr int kFcIaiCommand2 = 0x48;

/// Answer of an IAI 0x48 command: [slave] 48 [rejection u16] [status1..4 u32] (UNVERIFIED layout).
struct IaiResponse2 {
    int rejection = 0;                       ///< 0 = accepted; 0x1000-0x13FF = a rejection
    std::uint32_t status[4] = {0, 0, 0, 0};
};

/// Name of a 0x48 rejection code ("unknown rejection code" when not in the table). Implemented in src/modbus_iai2.cpp.
std::string iai2_rejection_name(int code);
/// The driver's short advice for a 0x48 rejection of `command` ("" when none; registers.py IAI2_REJECTION_HINTS and
/// IAI2_COMMAND_REJECTION_HINTS).
std::string iai2_rejection_hint(int code, int command);
/// Description of a 0x48 command allowed without allow_unsafe, "" when not on the safe list.
std::string iai2_safe_command(int command);
/// Why a 0x48 command is never sent without allow_never_send, "" when it is not on that list.
std::string iai2_never_send(int command);

/// Settings of a ModbusRtu master.
struct BusOptions {
    int retries = kRecommendedRetries;  ///< extra attempts after a timeout / CRC error / mismatched frame
    double min_gap_s = kDefaultMinGapS; ///< minimum silence between transactions
    FrameLogger frame_logger;           ///< optional
};

/// Opt-ins and options of an IAI 0x45 command.
struct IaiOptions {
    bool allow_unsafe = false;       ///< a command not on the safe list
    bool allow_never_send = false;   ///< a command on the never-send list (the baud change opts in itself)
    bool check = true;               ///< a non-zero rejection raises IaiCommandRejected
    std::optional<int> retries;      ///< timeouts / CRC only; 0 for a command that must not be sent twice
};

class ModbusRtu;

namespace detail {
/// How many stop-class writes a bus has sent so far to one slave and as broadcasts: a frame that stops motion (FC05
/// STOP coil ON or OFF, STP ON, SON OFF; bus_lock.hpp) or any write inside a StopPriority scope. Counted under the bus
/// lock as the frame goes to the wire. Private (the STOP-first gate, SendGate).
struct StopMark {
    std::uint64_t slave = 0;
    std::uint64_t broadcast = 0;
    bool operator==(const StopMark& o) const noexcept { return slave == o.slave && broadcast == o.broadcast; }
    bool operator!=(const StopMark& o) const noexcept { return !(*this == o); }
};
/// The stop mark of `slave` (1..247) on `bus` now. No lock, no I/O. Private.
StopMark stop_mark(const ModbusRtu& bus, int slave);
}  // namespace detail

/// Thread-safe Modbus RTU master on one port. Owns (shares) its transport; close() closes it.
/// Slave addresses 1..247 (axis n = slave n + 1); broadcast (0) only through the broadcast_* methods.
///
/// STOP first: a write that does not itself stop motion (and is not sent from a StopPriority scope) never reaches the
/// wire after a stop-class write to the same slave (or a broadcast one) that it did not see. A write counts from when
/// it is made: one that a stop-class write overtakes while it waits for the bus is refused (EcNotReadyError: "a STOP
/// arrived meanwhile ...; nothing sent"). Between two attempts of a request a waiting stop-class request goes first
/// (bus_lock.hpp); when one did, a write is not sent again ("a STOP went first; not re-sent"); its own stop-class frame
/// never counts against its retry. Reads are retried as before. A broadcast write is refused when a stop-class write
/// to any slave of the bus, or a broadcast one, went while it waited. A command that decides on a frame before it
/// sends it opens a detail::SendGate scope with the stop mark it decided on: a STOP that arrives meanwhile refuses the
/// frame under the bus lock.
class ModbusRtu {
public:
    explicit ModbusRtu(std::shared_ptr<Transport> transport, BusOptions options = {});
    ~ModbusRtu();
    ModbusRtu(const ModbusRtu&) = delete;
    ModbusRtu& operator=(const ModbusRtu&) = delete;

    // ---- configuration
    Transport& transport();
    std::shared_ptr<Transport> transport_ptr() const;
    /// Move the bus onto another byte pipe in place (the Python edition's `bus.transport = other`): settings, response
    /// delays, the line, the frame logger, the closed state and everything keyed by this bus (axes, recovery sessions)
    /// stay. Waits for the transaction in flight; nothing is sent. The old transport is not closed; the bus keeps it
    /// until it goes, so a reference from transport() stays valid.
    void set_transport(std::shared_ptr<Transport> transport);
    /// The transport's name ("COM3", "SIM1").
    std::string name() const;
    int retries() const;
    void set_retries(int retries);                 ///< >= 0, else std::invalid_argument
    double min_gap_s() const;
    void set_min_gap_s(double seconds);            ///< >= 0, else std::invalid_argument
    void set_frame_logger(FrameLogger logger);

    /// The unit's response delay RTIM (EC item 8.4 / CON No.17, 0..1000 ms) used by response_timeout().
    void set_response_delay_ms(int slave, double delay_ms);
    double response_delay_ms(int slave) const;

    /// How long to wait for the answer to `pdu` (s), from IAI's formula and the floors above. `expected_len` is the
    /// whole response frame length when known (nullopt: 64 bytes assumed).
    double response_timeout(int slave, const std::vector<std::uint8_t>& pdu, std::optional<int> expected_len) const;

    // ---- lifecycle
    /// Close the transport. Safe to call twice; later requests throw TransportError. The status monitors of the bus are
    /// stopped and joined first (monitor.hpp), every thread still waiting for the bus ends at once with TransportError
    /// (nothing queued is sent) and the transaction in flight finishes before the transport closes (bus_lock.hpp).
    void close();
    bool closed() const;
    /// True when the transport can be opened again in place (reopen()).
    bool can_reopen() const;
    /// Open the transport again in place after its port vanished (USB adapter pulled; VERIFIED in the Python edition:
    /// the COM port disappears) and came back, or after close(). Every object holding this bus keeps working with it;
    /// nothing is sent. `baud_rate`: the PC side's rate afterwards (nullopt: as before). The unit may have lost power
    /// meanwhile: recovery::recover (recovery.hpp) does the whole sequence. TransportError while the port is still
    /// missing; std::logic_error when the transport cannot be reopened.
    void reopen(std::optional<int> baud_rate = std::nullopt);

    /// Serialises multi-frame sequences of one axis on a shared port (a parameter write + commit, a software reset, a
    /// baud change) against the other axes' sequences. Recursive; single requests do not need it.
    std::recursive_mutex& sequence_mutex();

    /// The bus lock every request takes (bus_lock.hpp: FIFO, stop-class requests first, recursive). Holding it keeps
    /// the other threads off the bus across several requests of this thread; a STOP from another thread then waits for
    /// the whole hold, so keep holds short. Also for tests and diagnostics (waiting(), handovers()).
    BusLock& bus_lock() const;

    // ---- baud rate
    /// The transport's baud rate, nullopt when it has none (an in-memory simulator).
    std::optional<int> baud_rate() const;
    bool can_change_baud_rate() const;
    /// Change the transport's rate in place (the port is never closed). std::logic_error without a baud rate.
    void set_baud_rate(int rate);
    /// Hold BREAK for 0 < seconds <= 2 (>= 150 ms drops a unit to 9600, DOC). Nothing else is sent meanwhile.
    void send_break(double seconds);

    /// Find the rate one of `slaves` answers at and leave the transport set to it. Tries the current rate first
    /// (unless !current_first: then `candidates` in order, the current rate where it appears or last), one FC03 read
    /// of `probe_address` per address and rate, no retries; the first answer (an exception response counts) settles
    /// it. `candidates` empty = kBaudDetectOrder. `probe_timeout_s` default: the transport's timeout for one address,
    /// kScanTimeoutS for several. The rate is changed in place. The bus mutex is held throughout.
    /// Throws ModbusTimeout (start rate restored) when nothing answers; std::logic_error without a baud rate.
    int detect_baud_rate(const std::vector<int>& slaves, const std::vector<int>& candidates = {},
                         int probe_address = 0x9000, bool current_first = true,
                         std::optional<double> probe_timeout_s = std::nullopt);
    int detect_baud_rate(int slave, const std::vector<int>& candidates = {}, int probe_address = 0x9000,
                         bool current_first = true, std::optional<double> probe_timeout_s = std::nullopt);
    /// One target of detect_baud_rate_targets: `slave` asked at `address` (nullopt: the probe_address).
    struct BaudProbe {
        int slave = 1;
        std::optional<int> address;
    };
    /// detect_baud_rate over targets (the Python edition's detect_baudrate with (slave, address) entries: cell.Port adds
    /// the gateway probes (0xD0, 0x3700) and (0x3F, 0xF700) after the axes). An entry with its own address waits at most
    /// kScanTimeoutS (or probe_timeout_s); the plain entries as detect_baud_rate. The targets are asked in order at
    /// each rate.
    int detect_baud_rate_targets(const std::vector<BaudProbe>& targets, const std::vector<int>& candidates = {},
                                 int probe_address = 0x9000, bool current_first = true,
                                 std::optional<double> probe_timeout_s = std::nullopt);

    /// Ask `slave` once (FC03, one register), no retries, wait at most `timeout_s`. True for a valid frame
    /// (an exception response counts); false for silence, a bad CRC or another address. READ-ONLY.
    bool probe(int slave, double timeout_s = kScanTimeoutS, int address = 0x9000);

    // ---- requests (slave 1..247)
    /// FC03. Reads longer than kMaxReadRegisters are split. `retries` overrides retries() for this call.
    std::vector<std::uint16_t> read_registers(int slave, int address, int count,
                                              std::optional<int> retries = std::nullopt);
    /// FC05: FF00 / 0000; the echo is checked.
    void write_coil(int slave, int address, bool on, std::optional<int> retries = std::nullopt);
    /// FC06; the echo is checked.
    void write_register(int slave, int address, std::uint16_t value);
    /// FC10, 1..kMaxWriteRegisters values; the echo (address, count) is checked.
    void write_registers(int slave, int address, const std::vector<std::uint16_t>& values);
    /// FC10 with per-call options (retries, max_wait_s); the echo (address, count) is checked.
    void write_registers(int slave, int address, const std::vector<std::uint16_t>& values, const WriteOptions& options);
    /// FC17: write 1..60 `values` at write_address, then read 1..kMaxReadRegisters registers at read_address, in one
    /// frame (the byte count is checked). Used by the command-profile move (0x0F00 x20 written, 0x9000 x34 read).
    std::vector<std::uint16_t> read_write_registers(int slave, int read_address, int read_count, int write_address,
                                                    const std::vector<std::uint16_t>& values);
    /// FC11: the data bytes after the byte count. Default: no retries and kSlaveIdTimeoutS.
    std::vector<std::uint8_t> report_slave_id(int slave, std::optional<int> retries = 0,
                                              std::optional<double> timeout_s = std::nullopt);
    /// IAI 0x45 "control command": PDU 45 [group] [cmd] [op1 hi lo] [op2 hi lo]. See IaiOptions.
    IaiResponse iai_command(int slave, int group, int command, int op1 = 0, int op2 = 0, const IaiOptions& options = {});
    /// IAI 0x48 "ControlCommand2": PDU 48 [group u8] [cmd u16] [op1..op4 u32] (up to 4 operands, missing ones 0),
    /// 21 bytes + CRC, e.g. 01 48 00 00 30 + 16 x 00 + CRC; reply [slave] 48 [rejection u16] [4 x u32] (22 bytes). A
    /// rejection raises IaiCommandRejected (never retried). The 16-bit code must be on the 0x48 safe list and the group
    /// 0 unless allow_unsafe; never-send codes need allow_never_send. Refused before anything is sent on a line without
    /// 0x48 (LineRuleError). UNVERIFIED: untested on any unit, frame layout included.
    IaiResponse2 iai_command2(int slave, int command, const std::vector<std::uint32_t>& operands = {},
                              const IaiOptions& options = {}, int group = 0);

    // ---- broadcast (slave 0: every unit acts, none answers; the bus mutex is held for settle_s)
    /// 0x45 to address 0 (the same safe / never-send rules). Returns the frame sent.
    std::vector<std::uint8_t> broadcast_iai_command(int group, int command, int op1, int op2,
                                                    const IaiOptions& options = {}, double settle_s = kBaudSettleS);
    /// FC05 / FC06 / FC10 PDU to address 0 (never a read: std::invalid_argument). Returns the frame sent.
    std::vector<std::uint8_t> broadcast(const std::vector<std::uint8_t>& pdu, double settle_s = kBroadcastTurnaroundS);
    /// FC05 coil to address 0, e.g. STOP 00 05 04 2C FF 00 4D 12.
    std::vector<std::uint8_t> broadcast_coil(int address, bool on, double settle_s = kBroadcastTurnaroundS);

    /// A read-only UnitReader for one slave (FC03 + FC11 through this bus), for the model layer.
    std::unique_ptr<UnitReader> reader(int slave);

    // ---- the line (line.hpp, gateway.hpp)
    /// What the line carries (default: a "direct" Line). Every request passes its guard (Line::check_request) before
    /// anything is sent; IAI commands and broadcasts pass Line::require first (LineRuleError, nothing sent).
    std::shared_ptr<Line> line() const;
    /// Attach the line found by detect_line (or stated). std::invalid_argument for null.
    void set_line(std::shared_ptr<Line> line);
    /// Function codes `slave` answered with exception 01 (Illegal Function) so far on this bus.
    std::set<int> unsupported_functions(int slave) const;
    /// FC03 with per-call options (retries, max_wait_s, chunk). READ-ONLY.
    std::vector<std::uint16_t> read_registers(int slave, int address, int count, const ReadOptions& options);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend detail::StopMark detail::stop_mark(const ModbusRtu& bus, int slave);
};

/// "01 03 90 00 ..." (upper-case hex, space separated): frames in messages and logs.
std::string hex_frame(const std::vector<std::uint8_t>& bytes);

// ------------------------------------------------------------------------------------------ baud change

/// How set_baud_rate switches the rate.
enum class BaudChangeMethod {
    Direct,  ///< broadcast at the current rate, then switch the port in place (VERIFIED for 115200 / 230400)
    Break,   ///< port to 9600, line BREAK, broadcast at 9600 (the IA-OS sequence, ON-WIRE; UNVERIFIED on an EC)
};

/// "direct" / "break".
inline const char* to_string(BaudChangeMethod m) noexcept { return m == BaudChangeMethod::Direct ? "direct" : "break"; }

/// Options of change_baud_rate().
struct BaudChangeOptions {
    BaudChangeMethod method = BaudChangeMethod::Direct;
    double settle_s = kBaudSettleS;      ///< after the broadcast, before the port switches
    bool allow_unverified_code = false;  ///< accepted and ignored (no longer needed): an UNVERIFIED code is sent
};

/// The note every baud change carries.
inline constexpr const char* kBaudVolatileNote =
    "volatile: the EC answers at 38400 baud again after every power-up (VERIFIED); a broadcast changes the rate of "
    "every EC on the RS-485 line";  // the Python edition's BAUD_VOLATILE_NOTE

/// Outcome of a baud change.
struct BaudChangeResult {
    int previous = 0;            ///< the rate before
    int baud_rate = 0;           ///< the rate the port is at now and the unit answered at
    BaudChangeMethod method = BaudChangeMethod::Direct;
    int code = 0;                ///< op1 of the broadcast (IAI baud code)
    std::string frame;           ///< the broadcast frame sent, hex
    bool verified = false;       ///< the first read at the new rate was answered
    bool probed = false;         ///< that read failed and the rate was found by probing kBaudProbeAll instead
    std::string note = kBaudVolatileNote;
    bool code_verified = true;   ///< the rate's code is in kVerifiedBaudRates or kUnitDependentBaudRates
};

/// Change the baud rate of every unit on the bus (broadcast 00 45 00 04 00 <code> 00 05, allow_never_send) and the
/// port's, then confirm with one FC03 read of the first of `slaves` at the new rate; when that fails, probe
/// kBaudProbeAll and report where the unit went (probed = true). Holds sequence_mutex(). Throws SoftLimitError for a
/// rate without a code (nothing sent); an unverified rate (not in kVerifiedBaudRates / kUnitDependentBaudRates) is
/// sent with a logged warning (code_verified false),
/// std::logic_error without a baud rate, ModbusTimeout when no rate answers afterwards.
/// A unit found at another rate is not an error here: result.baud_rate != rate tells the caller (Axis::set_baud_rate and
/// Port::set_baud_rate throw VerificationError then, as the Python edition does).
BaudChangeResult change_baud_rate(ModbusRtu& bus, int rate, const std::vector<int>& slaves,
                                  const BaudChangeOptions& options = {});

namespace detail {
/// Run by ModbusRtu::close() before anything else (outside the bus lock): monitor.cpp installs the stop of the bus's
/// status monitors here when it makes its first monitor (the bus unit cannot call the axis unit). Private.
void set_bus_close_hook(std::function<void(const ModbusRtu&)> hook);

struct SendGateAccess;

/// RAII: the STOP-first gate of the calling thread for one slave of one bus. While it lives, every write of this
/// thread to that slave is checked under the bus lock right before it goes to the wire (reads, frames that stop
/// motion and writes inside a StopPriority scope are never checked): when a stop-class write went to the slave or as
/// a broadcast since `expected` (the stop mark taken when the caller decided, before its checks), the write is
/// refused with EcNotReadyError (stop_attempted set: a STOP went first) and nothing is sent. `check` (optional) runs
/// there too, and while the write waits for the bus: whatever it throws refuses the write. The thread's own
/// stop-class writes move `expected` on. Nestable: every scope that covers the slave must agree. Not copyable.
/// Private (the axis commands and the coordinated runner).
class SendGate {
public:
    SendGate(const ModbusRtu& bus, int slave, StopMark expected, std::function<void()> check = {});
    ~SendGate();
    SendGate(const SendGate&) = delete;
    SendGate& operator=(const SendGate&) = delete;
    /// How many writes went past this gate to the wire so far.
    int passed() const noexcept { return passed_; }

private:
    friend struct SendGateAccess;  // modbus.cpp
    const BusLock* bus_;
    int slave_;
    StopMark expected_;
    std::function<void()> check_;
    int passed_ = 0;
    SendGate* previous_;
};

struct StopWireAccess;

/// RAII: when the calling thread's first stop-class write on one bus went to the wire while the scope lived (a frame
/// that stops motion or any write inside a StopPriority scope, to any slave or broadcast; bus_lock.hpp): now_s() just
/// before the transport's write. Retries do not move it (the first attempt may be the one that reached the unit).
/// For the stop latency line (src/bus_stop_timing.hpp). Nestable (each scope keeps its own first); not copyable.
/// Private.
class StopWireStamp {
public:
    explicit StopWireStamp(const ModbusRtu& bus);
    ~StopWireStamp();
    StopWireStamp(const StopWireStamp&) = delete;
    StopWireStamp& operator=(const StopWireStamp&) = delete;
    /// The wire time of the first stop-class write; nullopt while none went out.
    std::optional<double> wire_s() const noexcept { return wire_; }

private:
    friend struct StopWireAccess;  // modbus.cpp
    const BusLock* bus_;
    std::optional<double> wire_;
    StopWireStamp* previous_;
};

/// Test hooks of the bus (empty: none). Set only while no request runs. Private.
struct BusTestHooks {
    /// Every request, before it waits for the bus (outside the bus lock): a test parks the thread here (a latch).
    std::function<void(int slave, const std::vector<std::uint8_t>& pdu)> before_bus_wait;
    /// Every wait for the minimum gap between two frames, before it sleeps or spins: the seconds left and whether the
    /// calling thread holds the bus then. A test records or parks here.
    std::function<void(double seconds, bool bus_held)> before_gap_wait;
};
BusTestHooks& bus_test_hooks();
}  // namespace detail

}  // namespace iai_actuator_driver
