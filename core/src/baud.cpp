// The broadcast baud change (port of the Python edition's Elecylinder.set_baudrate, broadcast / switch / verify / probe
// part; C# Axis.SetBaudRate): 00 45 00 04 00 <code> 00 05 to address 0 (ON-WIRE), wait, switch the port in place,
// verify with one read, probe every IAI rate when that read fails. 115200 (0x14) and 230400 (0x15) are VERIFIED;
// 38400 (0x06) is unit-dependent (taken by the EC-GRBP8M gripper and the PCON-CB, ignored by the EC-R6S rod) and is sent
// without the opt-in because the check after the switch decides.
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace detail {

/// The IAI code of `rate` after the refusal of change_baud_rate (nothing is sent by this check): SoftLimitError for
/// a rate without a code. An UNVERIFIED code is sent (change_baud_rate logs its warning; allow_unverified_code is no
/// longer needed). Shared with port.cpp.
int check_baud_change(int rate);

}  // namespace detail

namespace {

constexpr const char* kLogSource = "iai_actuator_driver.modbus";

std::string rates_text(const std::vector<int>& rates) {
    std::vector<std::string> parts;
    for (int r : rates) parts.push_back(std::to_string(r));
    return util::join(parts, ", ");
}

std::vector<int> verified_rates() { return {std::begin(kVerifiedBaudRates), std::end(kVerifiedBaudRates)}; }

}  // namespace

int detail::check_baud_change(int rate) {
    const std::optional<int> code = baud_code(rate);
    if (!code) {
        std::vector<int> all;
        for (const auto& entry : kIaiBaudCodes) all.push_back(entry.rate);
        std::sort(all.begin(), all.end());
        throw SoftLimitError(util::format("baud rate %d has no IAI code; choose from %s (verified) or %s (every "
                                          "code); nothing sent",
                                          rate, rates_text(verified_rates()).c_str(), rates_text(all).c_str()));
    }
    return *code;
}

BaudChangeResult change_baud_rate(ModbusRtu& bus, int rate, const std::vector<int>& slaves,
                                  const BaudChangeOptions& options) {
    const int code = detail::check_baud_change(rate);  // options.allow_unverified_code: ignored (no longer needed)
    if (slaves.empty()) throw std::invalid_argument("no slave address to verify the baud change with; nothing sent");
    for (int s : slaves)
        if (s < 1 || s > 247) throw std::invalid_argument(util::format("slave address must be 1..247, got %d", s));
    if (!(options.settle_s >= 0.0)) throw std::invalid_argument("settle_s must be >= 0");
    if (options.method != BaudChangeMethod::Direct && options.method != BaudChangeMethod::Break)
        throw std::invalid_argument("unknown baud change method");

    std::lock_guard<std::recursive_mutex> sequence(bus.sequence_mutex());
    const std::optional<int> start = bus.baud_rate();
    if (!start) throw std::logic_error(bus.name() + " has no baud rate to change; nothing sent");
    const int previous = *start;
    log_info(kLogSource, util::format("changing the baud rate %d -> %d (%s, broadcast 0x45 cmd 4, code 0x%02X)",
                                      previous, rate, to_string(options.method), code));
    if (!baud_code_verified(rate)) {
        std::string why = "its code is UNVERIFIED (untested on an EC)";
        const std::vector<int> detect = baud_detect_order();
        if (std::find(detect.begin(), detect.end(), rate) == detect.end())
            why += "; the auto-baud at connect never probes it (reconnect with that baud rate)";
        log_warning(kLogSource, util::format("baud rate %d: %s. Verified: %s. Sent anyway; a power cycle brings the "
                                             "unit back to its power-up rate",
                                             rate, why.c_str(), rates_text(verified_rates()).c_str()));
    }
    if (options.method == BaudChangeMethod::Break) {
        bus.set_baud_rate(9600);
        try {
            bus.send_break(kBreakS);
        } catch (const std::logic_error& e) {
            bus.set_baud_rate(previous);
            throw std::logic_error(std::string("this transport cannot send a line break (") + e.what() +
                                   "); use BaudChangeMethod::Direct");
        }
        sleep_at_least(kBreakS);  // ON-WIRE: IA-OS left ~0.22 s between the end of the break and the broadcast
    }
    IaiOptions iai;
    iai.allow_unsafe = true;
    iai.allow_never_send = true;  // the baud change is on the never-send list: this function opts in
    const std::vector<std::uint8_t> frame =
        bus.broadcast_iai_command(0, reg::kIaiCmdBaud, code, reg::kIaiBaudOp2, iai, options.settle_s);
    bus.set_baud_rate(rate);

    BaudChangeResult result;
    result.previous = previous;
    result.baud_rate = rate;
    result.method = options.method;
    result.code = code;
    result.frame = hex_frame(frame);
    result.code_verified = baud_code_verified(rate);
    try {
        bus.read_registers(slaves.front(), reg::kPnow, 1);
        result.verified = true;
    } catch (const ModbusTimeout&) {
    } catch (const ModbusCrcError&) {
    } catch (const ModbusFrameError&) {
    }
    if (!result.verified) {
        std::vector<int> order;
        auto add = [&order](int r) {
            if (std::find(order.begin(), order.end(), r) == order.end()) order.push_back(r);
        };
        add(rate);
        add(previous);
        for (int r : baud_probe_all()) add(r);
        const int found = bus.detect_baud_rate(slaves.front(), order);  // ModbusTimeout when nothing answers
        result.probed = true;
        result.baud_rate = found;
        if (found != rate)
            log_warning(kLogSource,
                        util::format("baud change to %d (code 0x%02X) not taken: slave %d %s at %d baud (the port is "
                                     "left at %d)",
                                     rate, code, slaves.front(), found == previous ? "still answers" : "answers",
                                     found, found));
    }
    return result;
}

}  // namespace iai_actuator_driver
