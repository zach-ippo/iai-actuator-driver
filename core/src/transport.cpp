// Serial-port discovery shared by both platforms (port of the Python edition's cell.list_ports; C# PortManager.cs):
// IAI adapters first, then natural order of the name (COM2 before COM10). The platform listing itself is in
// serial_win32.cpp (SetupAPI) and serial_posix.cpp (/sys/class/tty). Opens nothing.
#include <algorithm>
#include <cctype>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace detail {
/// The platform's serial ports, unsorted (serial_win32.cpp / serial_posix.cpp). May throw; list_ports() logs it.
std::vector<SerialPortInfo> platform_list_ports();
}  // namespace detail

namespace {

/// (name without trailing digits, trailing number or -1, name): "COM10" -> ("COM", 10, "COM10").
std::tuple<std::string, long long, std::string> natural_key(const std::string& name) {
    std::size_t end = name.size();
    while (end > 0 && std::isdigit(static_cast<unsigned char>(name[end - 1]))) --end;
    const std::string head = name.substr(0, end);
    const std::string tail = name.substr(end);
    long long number = -1;
    if (!tail.empty()) {
        number = 0;
        for (char c : tail.substr(0, 18)) number = number * 10 + (c - '0');
    }
    return std::make_tuple(head, number, name);
}

/// "\\.\COM12" -> "COM12".
std::string short_name(const std::string& port) {
    const std::string prefix = "\\\\.\\";
    return port.compare(0, prefix.size(), prefix) == 0 ? port.substr(prefix.size()) : port;
}

/// The list of iad_testing_set_port_list (nullopt: the PC's enumeration).
std::mutex& test_ports_mutex() {
    static std::mutex m;
    return m;
}
std::optional<std::vector<SerialPortInfo>>& test_ports() {
    static std::optional<std::vector<SerialPortInfo>> ports;
    return ports;
}

bool same_port(const std::string& a, const std::string& b) {
    const std::string x = short_name(a), y = short_name(b);
#ifdef _WIN32
    if (x.size() != y.size()) return false;
    for (std::size_t i = 0; i < x.size(); ++i)
        if (std::toupper(static_cast<unsigned char>(x[i])) != std::toupper(static_cast<unsigned char>(y[i])))
            return false;
    return true;
#else
    return x == y;
#endif
}

}  // namespace

void detail::set_test_port_list(std::optional<std::vector<SerialPortInfo>> ports) {
    std::lock_guard<std::mutex> lock(test_ports_mutex());
    test_ports() = std::move(ports);
}

std::vector<SerialPortInfo> list_ports() {
    std::vector<SerialPortInfo> ports;
    std::optional<std::vector<SerialPortInfo>> fake;
    {
        std::lock_guard<std::mutex> lock(test_ports_mutex());
        fake = test_ports();
    }
    try {
        ports = fake ? *fake : detail::platform_list_ports();
    } catch (const std::exception& e) {
        log_warning("iai_actuator_driver.transport", std::string("no serial ports listed: ") + e.what());
        return {};
    }
    std::stable_sort(ports.begin(), ports.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        const bool ia = a.is_iai(), ib = b.is_iai();
        if (ia != ib) return ia;
        return natural_key(a.port) < natural_key(b.port);
    });
    return ports;
}

std::vector<std::string> iai_port_names() {
    std::vector<std::string> names;
    for (const SerialPortInfo& info : list_ports())
        if (info.is_iai()) names.push_back(info.port);
    return names;
}

bool port_present(const std::string& port) {
    for (const SerialPortInfo& info : list_ports())
        if (same_port(info.port, port)) return true;
    return false;
}

}  // namespace iai_actuator_driver
