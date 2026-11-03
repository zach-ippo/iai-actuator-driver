// Byte pipes under the Modbus master: the Transport interface, the serial port (Win32 or POSIX termios behind one
// class) and serial-port discovery.
//
// Design rules (VERIFIED in the Python edition): a port is opened once and kept open - opening / closing in a loop, or
// a line BREAK of 150 ms or more, drops an EC back to its power-up baud rate - so the baud rate is changed in place and
// no break is ever sent implicitly. 8 data bits, no parity, 1 stop bit, DTR and RTS asserted, no handshake (the line
// state pyserial sets, i.e. the configuration the Python edition was verified with). 8N1 is the default everywhere;
// another framing (SerialFraming) is for a controller set to one.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace iai_actuator_driver {

/// Rate a serial port is opened at when none is given (IA-OS switches ECs to it; volatile). ECs boot at 38400.
inline constexpr int kDefaultBaudRate = 115200;

/// Default per-read timeout of a serial port (s). The bus sets the real per-request timeout (response_timeout()).
inline constexpr double kDefaultReadTimeoutS = 0.2;

/// A byte pipe. Implementations need not be thread-safe: the ModbusRtu master serialises every call under its mutex.
/// Errors of the underlying port (closed, unplugged) are thrown as TransportError; a read timeout is never an error.
class Transport {
public:
    virtual ~Transport() = default;

    /// Send all `size` bytes.
    virtual void write(const std::uint8_t* data, std::size_t size) = 0;

    /// Read up to `size` bytes, blocking at most read_timeout(); returns the number read (fewer, or 0, = timeout).
    virtual std::size_t read(std::uint8_t* buffer, std::size_t size) = 0;

    /// Discard received bytes not yet read.
    virtual void reset_input() = 0;

    /// Current baud rate, 0 when the transport has none (an in-memory simulator or fake).
    virtual int baud_rate() const = 0;

    /// Change the rate in place (the port stays open, so no line break reaches the unit). Throws std::logic_error on a
    /// transport without a baud rate.
    virtual void set_baud_rate(int rate) = 0;

    /// Per-read timeout in seconds.
    virtual double read_timeout() const = 0;
    virtual void set_read_timeout(double seconds) = 0;

    /// True for a real line with a real read timeout (a serial port; a simulated line with latency): the bus then
    /// waits up to response_timeout() per request. False for an in-memory transport that answers at once.
    virtual bool has_real_timeout() const { return false; }

    /// Hold the line in BREAK for `seconds`. A break of 150 ms or more drops a unit to 9600 baud (DOC ME0162):
    /// never used by the driver's normal paths. Throws std::logic_error where unsupported.
    virtual void send_break(double seconds) = 0;

    /// Release the resource. Safe to call twice.
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    /// A short name for messages and axis names ("COM3", "/dev/ttyUSB0", "SIM1").
    virtual std::string name() const = 0;

    // ---- power / link loss (recovery.hpp). Defaults: unknown / not reopenable.
    /// Whether the PC lists this port right now (a USB adapter pulled: the COM port vanishes, VERIFIED in the Python
    /// edition); nullopt when it cannot be told. Opens nothing.
    virtual std::optional<bool> present() const { return std::nullopt; }
    /// True when reopen() can open the same port again in place.
    virtual bool can_reopen() const { return false; }
    /// Open the same port again in place after it vanished and came back, or after close() (nothing is sent).
    /// `baud_rate`: the PC side's rate afterwards (nullopt: as before). TransportError while the port is absent;
    /// std::logic_error where unsupported (can_reopen() false).
    virtual void reopen(std::optional<int> baud_rate = std::nullopt) {
        (void)baud_rate;
        throw std::logic_error(name() + " cannot be reopened");
    }

    /// Convenience overload.
    void write(const std::vector<std::uint8_t>& data) { write(data.data(), data.size()); }
};

/// The parity bit of a serial character (SerialFraming).
enum class Parity {
    None,  ///< no parity bit (the default)
    Even,
    Odd,
};

/// The character framing of a serial port. The default, 8 data bits, no parity, 1 stop bit (8N1), is what the Modbus
/// units use; 7 data bits, a parity bit and 2 stop bits are for a controller set to another framing (UNVERIFIED on
/// hardware: no unit here uses them). A Modbus RTU bus needs 8 data bits; 7 is only for a caller's own 7-bit protocol
/// on Transport::write / read.
struct SerialFraming {
    int data_bits = 8;             ///< 7 or 8
    Parity parity = Parity::None;  ///< None, Even or Odd
    int stop_bits = 1;             ///< 1 or 2

    /// std::invalid_argument for data bits other than 7 or 8, stop bits other than 1 or 2 or an unknown parity.
    void check() const;

    bool operator==(const SerialFraming& other) const noexcept {
        return data_bits == other.data_bits && parity == other.parity && stop_bits == other.stop_bits;
    }
    bool operator!=(const SerialFraming& other) const noexcept { return !(*this == other); }
};

/// "8N1", "7E2", "8O1" (data bits, parity letter, stop bits).
std::string to_string(const SerialFraming& framing);

/// A real serial port: Win32 (CreateFile / DCB / COMMTIMEOUTS, "\\.\COMn" for any n) or POSIX termios (Linux; custom
/// rates such as 230400 through the standard Bxxx constants). Opened once in the constructor. While at least one
/// port is open on Windows the process timer resolution is raised to 1 ms (timeBeginPeriod; -lwinmm) so sleeps and
/// the inter-frame gap are not rounded up to 15.6 ms.
class SerialTransport final : public Transport {
public:
    /// Open `port` ("COM3", "\\.\COM12", "/dev/ttyUSB0"). Throws TransportError: "<port> is in use by another program
    /// (IA-OS, another iai-actuator-driver, a terminal)..." / "<port> does not exist (unplugged?)..." / other OS errors.
    explicit SerialTransport(const std::string& port, int baud_rate = kDefaultBaudRate,
                             double read_timeout_s = kDefaultReadTimeoutS);
    /// The same with another character framing (8N1 above). An unsupported framing (SerialFraming::check) is
    /// std::invalid_argument before the port is opened; a port whose driver does not keep the framing (POSIX: read
    /// back after the open) is a TransportError.
    SerialTransport(const std::string& port, int baud_rate, double read_timeout_s, const SerialFraming& framing);
    ~SerialTransport() override;
    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    using Transport::write;  // keep the write(vector) convenience overload visible
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override;
    void set_baud_rate(int rate) override;
    double read_timeout() const override;
    void set_read_timeout(double seconds) override;
    bool has_real_timeout() const override { return true; }
    void send_break(double seconds) override;
    void close() override;
    bool is_open() const override;
    std::string name() const override;

    /// The character framing the port was opened with (reopen() keeps it).
    SerialFraming framing() const;

    /// Open the same port again in place after it vanished (nothing is sent). Throws TransportError while absent.
    void reopen(std::optional<int> baud_rate = std::nullopt) override;
    bool can_reopen() const override { return true; }
    /// Its name in list_ports() (opens nothing).
    std::optional<bool> present() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// USB IDs of the IAI USB-to-serial adapter (Silicon Labs CP210x as "IAI USB to UART Bridge Controller").
inline constexpr int kIaiUsbVid = 0x10C4;
inline constexpr int kIaiUsbPid = 0x81D7;

/// One serial port of the PC.
struct SerialPortInfo {
    std::string port;           ///< "COM3" / "/dev/ttyUSB0"
    std::string description;    ///< friendly name, "" if unknown
    std::string manufacturer;   ///< "" if unknown
    std::string hardware_id;    ///< e.g. "USB\VID_10C4&PID_81D7\..." ("" if unknown)
    std::optional<int> vid;
    std::optional<int> pid;

    /// The IAI USB adapter (VID 0x10C4 / PID 0x81D7, VERIFIED), or a port whose manufacturer starts with "IAI" or whose
    /// description starts with "IAI " (UNVERIFIED for other IAI adapters). Header-only (the JSON form uses it).
    bool is_iai() const {
        if (vid && pid && *vid == kIaiUsbVid && *pid == kIaiUsbPid) return true;
        auto starts = [](const std::string& text, const char* prefix) {
            std::size_t i = 0;
            while (i < text.size() && (text[i] == ' ' || text[i] == '	')) ++i;
            for (std::size_t k = 0; prefix[k]; ++k, ++i) {
                if (i >= text.size()) return false;
                char c = text[i];
                if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
                if (c != prefix[k]) return false;
            }
            return true;
        };
        return starts(manufacturer, "IAI") || starts(description, "IAI ");
    }
};

/// The PC's serial ports, IAI adapters first, then by name (COM2 before COM10). Opens nothing. Windows: SetupAPI
/// (GUID_DEVCLASS_PORTS; -lsetupapi); Linux: /sys/class/tty/*/device (ttyUSB*, ttyACM*, ttyS* with a device).
std::vector<SerialPortInfo> list_ports();

namespace detail {
/// (0.7) Test hook of the C ABI (iad_testing_set_port_list; the Python edition's testing.fake_ports): list_ports() -
/// and so iai_port_names(), port_present() and a serial transport's present() - gives `ports`, ordered as the PC's,
/// instead of the PC's enumeration, process-wide; nullopt restores the PC's. Opens nothing.
void set_test_port_list(std::optional<std::vector<SerialPortInfo>> ports);
}  // namespace detail

/// The names of the IAI adapters of list_ports().
std::vector<std::string> iai_port_names();

/// True when `port` is currently listed by the PC (opens nothing).
bool port_present(const std::string& port);

inline std::optional<bool> SerialTransport::present() const { return port_present(name()); }

}  // namespace iai_actuator_driver
