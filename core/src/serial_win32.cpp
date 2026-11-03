// SerialTransport and the port listing on Windows: CreateFile("\\.\COMn") / DCB / COMMTIMEOUTS, SetupAPI for
// discovery. Line state as the Python edition (pyserial) was VERIFIED with: 8N1, DTR and RTS asserted, no handshake;
// the port is opened once, the rate changed in place, a BREAK only on request. This edition is UNVERIFIED on hardware,
// and so is another framing (7 data bits, parity, 2 stop bits: SerialFraming).
#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#include <setupapi.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "serial_framing.hpp"

namespace iai_actuator_driver {
namespace detail {
std::vector<SerialPortInfo> platform_list_ports();
}  // namespace detail

namespace {

constexpr DWORD kWriteTimeoutMs = 2000;  // a stuck adapter fails a write instead of hanging the bus

// ---- process timer resolution: 1 ms while at least one port is open (reference counted)
std::mutex& timer_mutex() {
    static std::mutex m;
    return m;
}
int& timer_users() {
    static int users = 0;
    return users;
}
void raise_timer_resolution() {
    std::lock_guard<std::mutex> lock(timer_mutex());
    if (timer_users()++ == 0) timeBeginPeriod(1);
}
void lower_timer_resolution() {
    std::lock_guard<std::mutex> lock(timer_mutex());
    if (timer_users() > 0 && --timer_users() == 0) timeEndPeriod(1);
}

std::string error_text(DWORD code) {
    char buffer[512] = {0};
    const DWORD n = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
                                   buffer, static_cast<DWORD>(sizeof buffer), nullptr);
    std::string text(buffer, n);
    while (!text.empty() && (text.back() == '\r' || text.back() == '\n' || text.back() == ' ' || text.back() == '.'))
        text.pop_back();
    char head[32];
    std::snprintf(head, sizeof head, "Windows error %lu", static_cast<unsigned long>(code));
    return text.empty() ? std::string(head) : std::string(head) + ": " + text;
}

/// "COM3" / "\\.\COM12" -> ("\\.\COM3", "COM3").
void split_port_name(const std::string& port, std::string& path, std::string& name) {
    const std::string prefix = "\\\\.\\";
    if (port.compare(0, prefix.size(), prefix) == 0) {
        path = port;
        name = port.substr(prefix.size());
    } else {
        path = prefix + port;
        name = port;
    }
}

bool starts_with_ci(const std::string& text, const char* prefix) {
    for (std::size_t i = 0; prefix[i]; ++i) {
        if (i >= text.size()) return false;
        if (std::toupper(static_cast<unsigned char>(text[i])) != std::toupper(static_cast<unsigned char>(prefix[i])))
            return false;
    }
    return true;
}

}  // namespace

void detail::apply_framing(DCB& dcb, const SerialFraming& framing) {
    dcb.ByteSize = static_cast<BYTE>(framing.data_bits);
    dcb.Parity = framing.parity == Parity::Even ? EVENPARITY : framing.parity == Parity::Odd ? ODDPARITY : NOPARITY;
    dcb.fParity = framing.parity == Parity::None ? FALSE : TRUE;
    dcb.StopBits = framing.stop_bits == 2 ? TWOSTOPBITS : ONESTOPBIT;
}

// ------------------------------------------------------------------------------------------ SerialTransport

struct SerialTransport::Impl {
    std::string path;  // "\\.\COM3"
    std::string name;  // "COM3"
    HANDLE handle = INVALID_HANDLE_VALUE;
    int baud = kDefaultBaudRate;
    double read_timeout = kDefaultReadTimeoutS;
    SerialFraming framing;  // 8N1 unless the constructor was given another
    bool timer_raised = false;

    bool open() const { return handle != INVALID_HANDLE_VALUE; }

    [[noreturn]] void fail(const char* what) const {
        const DWORD code = GetLastError();
        throw TransportError(name + ": " + what + " failed (" + error_text(code) + ")");
    }

    void require_open() const {
        if (!open()) throw TransportError(name + " is closed");
    }

    void apply_timeouts(DWORD read_constant_ms, bool non_blocking) {
        COMMTIMEOUTS t = {};
        if (non_blocking) {
            t.ReadIntervalTimeout = MAXDWORD;  // return at once with what is buffered
            t.ReadTotalTimeoutMultiplier = 0;
            t.ReadTotalTimeoutConstant = 0;
        } else {
            // return as soon as any byte is there, else wait up to the constant
            t.ReadIntervalTimeout = MAXDWORD;
            t.ReadTotalTimeoutMultiplier = MAXDWORD;
            t.ReadTotalTimeoutConstant = read_constant_ms;
        }
        t.WriteTotalTimeoutMultiplier = 0;
        t.WriteTotalTimeoutConstant = kWriteTimeoutMs;
        if (!SetCommTimeouts(handle, &t)) fail("SetCommTimeouts");
    }

    void configure(int rate) {
        DCB dcb = {};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(handle, &dcb)) fail("GetCommState");
        dcb.BaudRate = static_cast<DWORD>(rate);
        detail::apply_framing(dcb, framing);  // 8N1 by default
        dcb.fBinary = TRUE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fTXContinueOnXoff = TRUE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fErrorChar = FALSE;
        dcb.fNull = FALSE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fAbortOnError = FALSE;
        if (!SetCommState(handle, &dcb))
            fail(("setting " + std::to_string(rate) + " baud " + to_string(framing)).c_str());
        SetupComm(handle, 4096, 4096);
        apply_timeouts(1, false);
        PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    }

    void open_port(int rate) {
        handle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                             FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            const DWORD code = GetLastError();
            if (code == ERROR_ACCESS_DENIED || code == ERROR_SHARING_VIOLATION)
                throw TransportError(name +
                                     " is in use by another program (IA-OS, another iai-actuator-driver, a terminal). Close it "
                                     "and try again - only one program can own a serial port");
            if (code == ERROR_FILE_NOT_FOUND || code == ERROR_PATH_NOT_FOUND)
                throw TransportError(name + " does not exist (unplugged?): check the USB adapter and the port name");
            throw TransportError(name + ": cannot open the port (" + error_text(code) + ")");
        }
        try {
            configure(rate);
        } catch (...) {
            CloseHandle(handle);
            handle = INVALID_HANDLE_VALUE;
            throw;
        }
        baud = rate;
        if (!timer_raised) {
            raise_timer_resolution();
            timer_raised = true;
        }
    }

    void close_port() {
        if (handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
            handle = INVALID_HANDLE_VALUE;
        }
        if (timer_raised) {
            lower_timer_resolution();
            timer_raised = false;
        }
    }
};

SerialTransport::SerialTransport(const std::string& port, int baud_rate, double read_timeout_s)
    : SerialTransport(port, baud_rate, read_timeout_s, SerialFraming{}) {}

SerialTransport::SerialTransport(const std::string& port, int baud_rate, double read_timeout_s,
                                 const SerialFraming& framing)
    : impl_(new Impl) {
    if (port.empty()) throw std::invalid_argument("port name required");
    if (baud_rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    if (!(read_timeout_s >= 0.0)) throw std::invalid_argument("read timeout must be >= 0");
    framing.check();  // before anything is opened
    split_port_name(port, impl_->path, impl_->name);
    impl_->read_timeout = read_timeout_s;
    impl_->framing = framing;
    impl_->open_port(baud_rate);
}

SerialTransport::~SerialTransport() {
    try {
        impl_->close_port();
    } catch (...) {
    }
}

void SerialTransport::write(const std::uint8_t* data, std::size_t size) {
    impl_->require_open();
    std::size_t done = 0;
    while (done < size) {
        DWORD n = 0;
        const DWORD chunk = static_cast<DWORD>(std::min<std::size_t>(size - done, 0x10000));
        if (!WriteFile(impl_->handle, data + done, chunk, &n, nullptr)) impl_->fail("write");
        if (n == 0) throw TransportError(impl_->name + ": write timed out (the adapter accepts no data)");
        done += n;
    }
}

std::size_t SerialTransport::read(std::uint8_t* buffer, std::size_t size) {
    impl_->require_open();
    if (size == 0) return 0;
    std::size_t got = 0;
    if (!(impl_->read_timeout > 0.0)) {
        impl_->apply_timeouts(0, true);
        DWORD n = 0;
        if (!ReadFile(impl_->handle, buffer, static_cast<DWORD>(std::min<std::size_t>(size, 0x10000)), &n, nullptr))
            impl_->fail("read");
        return n;
    }
    const double deadline = now_s() + impl_->read_timeout;
    while (got < size) {
        const double remaining = deadline - now_s();
        if (remaining <= 0.0) {
            // The deadline passed (also when this thread was not scheduled for a while after its last ReadFile): take
            // what is buffered already, without waiting, before the caller hears that bytes are missing.
            impl_->apply_timeouts(0, true);
            while (got < size) {
                DWORD n = 0;
                const DWORD want = static_cast<DWORD>(std::min<std::size_t>(size - got, 0x10000));
                if (!ReadFile(impl_->handle, buffer + got, want, &n, nullptr)) impl_->fail("read");
                if (n == 0) break;
                got += n;
            }
            break;
        }
        const double ms = std::ceil(remaining * 1000.0);
        const DWORD constant = static_cast<DWORD>(std::max(1.0, std::min(ms, 3600000.0)));
        impl_->apply_timeouts(constant, false);
        DWORD n = 0;
        const DWORD want = static_cast<DWORD>(std::min<std::size_t>(size - got, 0x10000));
        if (!ReadFile(impl_->handle, buffer + got, want, &n, nullptr)) impl_->fail("read");
        got += n;
    }
    return got;
}

void SerialTransport::reset_input() {
    impl_->require_open();
    if (!PurgeComm(impl_->handle, PURGE_RXCLEAR | PURGE_RXABORT)) impl_->fail("PurgeComm");
}

int SerialTransport::baud_rate() const { return impl_->baud; }

void SerialTransport::set_baud_rate(int rate) {
    if (rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    impl_->require_open();
    DCB dcb = {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(impl_->handle, &dcb)) impl_->fail("GetCommState");
    dcb.BaudRate = static_cast<DWORD>(rate);
    if (!SetCommState(impl_->handle, &dcb)) impl_->fail(("setting " + std::to_string(rate) + " baud").c_str());
    impl_->baud = rate;
}

double SerialTransport::read_timeout() const { return impl_->read_timeout; }

void SerialTransport::set_read_timeout(double seconds) {
    if (!(seconds >= 0.0) || !std::isfinite(seconds)) throw std::invalid_argument("read timeout must be >= 0");
    impl_->read_timeout = seconds;
}

void SerialTransport::send_break(double seconds) {
    impl_->require_open();
    if (!SetCommBreak(impl_->handle)) impl_->fail("SetCommBreak");
    struct Release {
        HANDLE handle;
        ~Release() { ClearCommBreak(handle); }
    } release{impl_->handle};
    sleep_at_least(seconds);
}

void SerialTransport::close() { impl_->close_port(); }

bool SerialTransport::is_open() const { return impl_->open(); }

std::string SerialTransport::name() const { return impl_->name; }

SerialFraming SerialTransport::framing() const { return impl_->framing; }

void SerialTransport::reopen(std::optional<int> baud_rate) {
    const int rate = baud_rate.value_or(impl_->baud);
    if (rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    if (impl_->handle != INVALID_HANDLE_VALUE) {
        CloseHandle(impl_->handle);
        impl_->handle = INVALID_HANDLE_VALUE;
    }
    impl_->open_port(rate);
}

// ------------------------------------------------------------------------------------------ discovery

namespace {

std::string device_property(HDEVINFO devs, SP_DEVINFO_DATA& info, DWORD property) {
    std::vector<BYTE> buffer(1024);
    DWORD type = 0;
    DWORD needed = 0;
    if (!SetupDiGetDeviceRegistryPropertyA(devs, &info, property, &type, buffer.data(),
                                           static_cast<DWORD>(buffer.size()), &needed)) {
        if (GetLastError() != ERROR_INSUFFICIENT_BUFFER || needed == 0) return std::string();
        buffer.resize(needed);
        if (!SetupDiGetDeviceRegistryPropertyA(devs, &info, property, &type, buffer.data(),
                                               static_cast<DWORD>(buffer.size()), &needed))
            return std::string();
    }
    if (type != REG_SZ && type != REG_MULTI_SZ && type != REG_EXPAND_SZ) return std::string();
    const char* text = reinterpret_cast<const char*>(buffer.data());
    std::size_t len = 0;
    while (len < buffer.size() && text[len]) ++len;  // REG_MULTI_SZ: the first string
    return std::string(text, len);
}

std::string device_port_name(HDEVINFO devs, SP_DEVINFO_DATA& info) {
    HKEY key = SetupDiOpenDevRegKey(devs, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
    if (key == nullptr || key == reinterpret_cast<HKEY>(INVALID_HANDLE_VALUE)) return std::string();
    char value[256] = {0};
    DWORD size = sizeof(value) - 1;
    DWORD type = 0;
    std::string name;
    if (RegQueryValueExA(key, "PortName", nullptr, &type, reinterpret_cast<BYTE*>(value), &size) == ERROR_SUCCESS &&
        type == REG_SZ) {
        value[std::min<DWORD>(size, sizeof(value) - 1)] = 0;
        name = value;
    }
    RegCloseKey(key);
    return name;
}

/// The 4 hex digits after `tag` ("VID_") in a hardware ID, nullopt when absent.
std::optional<int> usb_id(const std::string& hardware_id, const char* tag) {
    std::string upper(hardware_id);
    for (char& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const std::size_t at = upper.find(tag);
    if (at == std::string::npos || at + 4 + 4 > upper.size()) return std::nullopt;
    const std::string digits = upper.substr(at + 4, 4);
    for (char c : digits)
        if (!std::isxdigit(static_cast<unsigned char>(c))) return std::nullopt;
    return static_cast<int>(std::strtol(digits.c_str(), nullptr, 16));
}

}  // namespace

std::vector<SerialPortInfo> detail::platform_list_ports() {
    std::vector<SerialPortInfo> ports;
    // GUID_DEVCLASS_PORTS {4D36E978-E325-11CE-BFC1-08002BE10318}, spelled out (no initguid.h needed)
    static const GUID kPortsClass = {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
    HDEVINFO devs = SetupDiGetClassDevsA(&kPortsClass, nullptr, nullptr, DIGCF_PRESENT);
    if (devs != INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA info = {};
        info.cbSize = sizeof(info);
        for (DWORD i = 0; SetupDiEnumDeviceInfo(devs, i, &info); ++i) {
            const std::string port = device_port_name(devs, info);
            if (port.empty() || !starts_with_ci(port, "COM")) continue;  // LPT ports share the class
            SerialPortInfo entry;
            entry.port = port;
            entry.description = device_property(devs, info, SPDRP_FRIENDLYNAME);
            if (entry.description.empty()) entry.description = device_property(devs, info, SPDRP_DEVICEDESC);
            entry.manufacturer = device_property(devs, info, SPDRP_MFG);
            entry.hardware_id = device_property(devs, info, SPDRP_HARDWAREID);
            entry.vid = usb_id(entry.hardware_id, "VID_");
            entry.pid = usb_id(entry.hardware_id, "PID_");
            ports.push_back(entry);
        }
        SetupDiDestroyDeviceInfoList(devs);
    }
    // ports the device manager does not class as "Ports" (some virtual / driverless ones): the SERIALCOMM map
    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) == ERROR_SUCCESS) {
        for (DWORD i = 0;; ++i) {
            char value_name[256];
            DWORD name_len = sizeof(value_name);
            BYTE data[256];
            DWORD data_len = sizeof(data) - 1;
            DWORD type = 0;
            const LONG r = RegEnumValueA(key, i, value_name, &name_len, nullptr, &type, data, &data_len);
            if (r == ERROR_NO_MORE_ITEMS) break;
            if (r != ERROR_SUCCESS || type != REG_SZ) continue;
            data[std::min<DWORD>(data_len, sizeof(data) - 1)] = 0;
            const std::string port(reinterpret_cast<const char*>(data));
            if (port.empty()) continue;
            const bool known = std::any_of(ports.begin(), ports.end(), [&port](const SerialPortInfo& p) {
                return p.port.size() == port.size() && starts_with_ci(p.port, port.c_str());
            });
            if (!known) {
                SerialPortInfo entry;
                entry.port = port;
                ports.push_back(entry);
            }
        }
        RegCloseKey(key);
    }
    return ports;
}

}  // namespace iai_actuator_driver

#endif  // _WIN32
