// SerialTransport and the port listing on Linux (POSIX termios): raw 8N1, DTR and RTS asserted, no handshake, poll()
// timeouts, the rate changed in place, a BREAK only on request; discovery from /sys/class/tty. VERIFIED on Linux with
// the IAI USB adapter (CP210x, cp210x driver, passed into WSL 2 with usbipd) on a PCON-CB, 2026-11-03: the harness's
// Python and C++ command-line editions and the SDK's C example; macOS UNVERIFIED.
#ifndef _WIN32

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace detail {
std::vector<SerialPortInfo> platform_list_ports();
}  // namespace detail

namespace {

/// The termios constant of a standard rate, 0 when the system has none.
speed_t speed_constant(int rate) {
    switch (rate) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return 0;
    }
}

std::string errno_text(int code) { return std::string(std::strerror(code)) + " (errno " + std::to_string(code) + ")"; }

/// "/dev/ttyUSB0" -> "ttyUSB0" (for messages: the full path stays the name).
std::string base_name(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

// ------------------------------------------------------------------------------------------ SerialTransport

struct SerialTransport::Impl {
    std::string path;  // "/dev/ttyUSB0"
    int fd = -1;
    int baud = kDefaultBaudRate;
    double read_timeout = kDefaultReadTimeoutS;

    bool open() const { return fd >= 0; }

    [[noreturn]] void fail(const char* what, int code) const {
        throw TransportError(path + ": " + what + " failed (" + errno_text(code) + ")");
    }

    void require_open() const {
        if (!open()) throw TransportError(path + " is closed");
    }

    void apply_speed(int rate) {
        const speed_t speed = speed_constant(rate);
        if (speed == 0)
            throw TransportError(path + ": " + std::to_string(rate) +
                                 " baud is not a standard rate on this system (no B" + std::to_string(rate) + ")");
        termios tio;
        if (tcgetattr(fd, &tio) != 0) fail("tcgetattr", errno);
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        if (tcsetattr(fd, TCSANOW, &tio) != 0) fail("tcsetattr (baud rate)", errno);
        baud = rate;
    }

    void configure(int rate) {
        termios tio;
        if (tcgetattr(fd, &tio) != 0) fail("tcgetattr", errno);
        tio.c_iflag &= ~static_cast<tcflag_t>(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF |
                                              IXANY | INPCK);
        tio.c_oflag &= ~static_cast<tcflag_t>(OPOST);
        tio.c_lflag &= ~static_cast<tcflag_t>(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | CSTOPB);
#ifdef CRTSCTS
        tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
        tio.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &tio) != 0) fail("tcsetattr (raw 8N1)", errno);
        apply_speed(rate);
        int bits = TIOCM_DTR | TIOCM_RTS;
        if (ioctl(fd, TIOCMBIS, &bits) != 0) {
            // not every driver (a pseudo terminal, some virtual ports) supports the modem lines: not fatal
        }
        tcflush(fd, TCIOFLUSH);
    }

    void open_port(int rate) {
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            const int code = errno;
            if (code == EBUSY)
                throw TransportError(path +
                                     " is in use by another program (another iai-actuator-driver, a terminal). Close it and try "
                                     "again - only one program can own a serial port");
            if (code == ENOENT || code == ENODEV || code == ENXIO)
                throw TransportError(path + " does not exist (unplugged?): check the USB adapter and the port name");
            if (code == EACCES)
                throw TransportError(path + ": permission denied (on Linux add the user to the 'dialout' group)");
            throw TransportError(path + ": cannot open the port (" + errno_text(code) + ")");
        }
        if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
            const int code = errno;
            ::close(fd);
            fd = -1;
            if (code == EWOULDBLOCK)
                throw TransportError(path +
                                     " is in use by another program (another iai-actuator-driver, a terminal). Close it and try "
                                     "again - only one program can own a serial port");
            throw TransportError(path + ": cannot lock the port (" + errno_text(code) + ")");
        }
        try {
            configure(rate);
        } catch (...) {
            ::close(fd);
            fd = -1;
            throw;
        }
    }

    void close_port() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
};

SerialTransport::SerialTransport(const std::string& port, int baud_rate, double read_timeout_s) : impl_(new Impl) {
    if (port.empty()) throw std::invalid_argument("port name required");
    if (baud_rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    if (!(read_timeout_s >= 0.0)) throw std::invalid_argument("read timeout must be >= 0");
    impl_->path = port.find('/') == std::string::npos ? "/dev/" + port : port;
    impl_->read_timeout = read_timeout_s;
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
    const double deadline = now_s() + 2.0;  // a stuck adapter fails a write instead of hanging the bus
    while (done < size) {
        const ssize_t n = ::write(impl_->fd, data + done, size - done);
        if (n > 0) {
            done += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) impl_->fail("write", errno);
        const double remaining = deadline - now_s();
        if (remaining <= 0.0) throw TransportError(impl_->path + ": write timed out (the adapter accepts no data)");
        pollfd p = {impl_->fd, POLLOUT, 0};
        const int r = ::poll(&p, 1, static_cast<int>(std::ceil(remaining * 1000.0)));
        if (r < 0 && errno != EINTR) impl_->fail("poll (write)", errno);
        if (r > 0 && (p.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw TransportError(impl_->path + ": the port vanished (unplugged?)");
    }
}

std::size_t SerialTransport::read(std::uint8_t* buffer, std::size_t size) {
    impl_->require_open();
    if (size == 0) return 0;
    std::size_t got = 0;
    const double deadline = now_s() + impl_->read_timeout;
    bool readable = false;  // the last poll() said POLLIN
    for (;;) {
        const ssize_t n = ::read(impl_->fd, buffer + got, size - got);
        if (n > 0) {
            got += static_cast<std::size_t>(n);
            if (got >= size) break;
            readable = false;
            continue;
        }
        if (n == 0) {
            // VMIN = VTIME = 0: Linux returns 0, not EAGAIN, while nothing has arrived yet (VERIFIED on a CP210x
            // adapter). Only a 0 right after poll() said readable means the device is gone (a USB adapter pulled).
            if (readable) throw TransportError(impl_->path + ": the port vanished (unplugged?)");
        } else {
            if (errno == EINTR) continue;
            if (errno != EAGAIN && errno != EWOULDBLOCK) impl_->fail("read", errno);
        }
        const double remaining = deadline - now_s();
        if (remaining <= 0.0) break;
        pollfd p = {impl_->fd, POLLIN, 0};
        const int r = ::poll(&p, 1, std::max(1, static_cast<int>(std::ceil(remaining * 1000.0))));
        if (r < 0) {
            if (errno == EINTR) continue;
            impl_->fail("poll (read)", errno);
        }
        if (r == 0) break;  // timeout
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
            throw TransportError(impl_->path + ": the port vanished (unplugged?)");
        readable = (p.revents & POLLIN) != 0;
    }
    return got;
}

void SerialTransport::reset_input() {
    impl_->require_open();
    if (tcflush(impl_->fd, TCIFLUSH) != 0) impl_->fail("tcflush", errno);
}

int SerialTransport::baud_rate() const { return impl_->baud; }

void SerialTransport::set_baud_rate(int rate) {
    if (rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    impl_->require_open();
    impl_->apply_speed(rate);
}

double SerialTransport::read_timeout() const { return impl_->read_timeout; }

void SerialTransport::set_read_timeout(double seconds) {
    if (!(seconds >= 0.0) || !std::isfinite(seconds)) throw std::invalid_argument("read timeout must be >= 0");
    impl_->read_timeout = seconds;
}

void SerialTransport::send_break(double seconds) {
    impl_->require_open();
    if (ioctl(impl_->fd, TIOCSBRK) != 0) impl_->fail("TIOCSBRK", errno);
    struct Release {
        int fd;
        ~Release() { ioctl(fd, TIOCCBRK); }
    } release{impl_->fd};
    sleep_at_least(seconds);
}

void SerialTransport::close() { impl_->close_port(); }

bool SerialTransport::is_open() const { return impl_->open(); }

std::string SerialTransport::name() const { return impl_->path; }

void SerialTransport::reopen(std::optional<int> baud_rate) {
    const int rate = baud_rate.value_or(impl_->baud);
    if (rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    impl_->close_port();
    impl_->open_port(rate);
}

// ------------------------------------------------------------------------------------------ discovery

namespace {

std::string read_line(const std::string& file) {
    std::ifstream in(file);
    std::string line;
    if (in) std::getline(in, line);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ')) line.pop_back();
    return line;
}

bool exists(const std::string& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

std::string real_path(const std::string& path) {
    char buffer[PATH_MAX];
    if (::realpath(path.c_str(), buffer) == nullptr) return std::string();
    return buffer;
}

bool starts_with(const std::string& text, const char* prefix) { return text.rfind(prefix, 0) == 0; }

std::optional<int> hex_value(const std::string& text) {
    if (text.empty()) return std::nullopt;
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 16);
    if (end == text.c_str() || *end != '\0') return std::nullopt;
    return static_cast<int>(value);
}

}  // namespace

std::vector<SerialPortInfo> detail::platform_list_ports() {
    std::vector<SerialPortInfo> ports;
    DIR* dir = ::opendir("/sys/class/tty");
    if (dir == nullptr) return ports;
    std::vector<std::string> names;
    while (dirent* entry = ::readdir(dir)) {
        const std::string name = entry->d_name;
        if (starts_with(name, "ttyUSB") || starts_with(name, "ttyACM") || starts_with(name, "ttyS") ||
            starts_with(name, "ttyAMA") || starts_with(name, "ttyXRUSB") || starts_with(name, "rfcomm"))
            names.push_back(name);
    }
    ::closedir(dir);
    for (const std::string& name : names) {
        const std::string sys = "/sys/class/tty/" + name;
        const std::string device = real_path(sys + "/device");
        if (device.empty()) continue;  // no device behind it (a placeholder ttyS)
        const std::string subsystem = base_name(real_path(device + "/subsystem"));
        if (starts_with(name, "ttyS") && subsystem == "platform") continue;  // legacy placeholders, as pyserial
        SerialPortInfo info;
        info.port = "/dev/" + name;
        // walk up to the USB device (the directory with idVendor / idProduct)
        std::string dir_path = device;
        for (int depth = 0; depth < 6 && !dir_path.empty() && dir_path != "/"; ++depth) {
            if (exists(dir_path + "/idVendor") && exists(dir_path + "/idProduct")) {
                info.vid = hex_value(read_line(dir_path + "/idVendor"));
                info.pid = hex_value(read_line(dir_path + "/idProduct"));
                info.manufacturer = read_line(dir_path + "/manufacturer");
                const std::string product = read_line(dir_path + "/product");
                const std::string serial = read_line(dir_path + "/serial");
                info.description = product;
                char id[64];
                std::snprintf(id, sizeof id, "USB VID:PID=%04X:%04X", info.vid.value_or(0), info.pid.value_or(0));
                info.hardware_id = id;
                if (!serial.empty()) info.hardware_id += " SER=" + serial;
                break;
            }
            const std::size_t slash = dir_path.find_last_of('/');
            if (slash == std::string::npos || slash == 0) break;
            dir_path = dir_path.substr(0, slash);
        }
        if (info.description.empty()) info.description = name;
        ports.push_back(info);
    }
    return ports;
}

}  // namespace iai_actuator_driver

#endif  // !_WIN32
