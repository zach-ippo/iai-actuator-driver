// C ABI 0.3 (include/iai_actuator_driver.h "transports" to "axis and cell: cancel latch"): transports (serial, the
// binding's callbacks), buses and their raw requests, the clock seam, the log level, axes and cells opened on
// transports and buses, the cancel latch. The simulators are in c_api_sim.cpp. C ABI 0.4 ("buses: bus lock" to "buses:
// response timing"): the bus lock held across calls, the preemptible read scope, suspending the monitors of a bus, the
// 0x45 flags, 0x48, broadcasts, the line on a bus and the response timing. C ABI 0.5 ("buses: shutdown, release"):
// closing a bus without releasing its handle, releasing it without closing, the transport's set_read_timeout callback.
// C ABI 0.6 ("buses: another transport"): a bus moved onto another transport in place. C ABI 1.1: a serial port opened
// with another character framing (iad_transport_open_serial_options). Evidence: simulator tests only (UNVERIFIED on
// hardware).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "c_api_objects.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/gateway.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/modbus.hpp"
#include "iai_actuator_driver/monitor.hpp"
#include "iai_actuator_driver/port.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "iai_actuator_driver/virtual_clock.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

namespace {


// 0.3 sized structs: the size each was introduced with (read_sized refuses less). iad_transport_callbacks grew in 0.5
// (set_read_timeout appended): its 0.3 size is the struct up to that field, padded as the 0.3 struct was.
constexpr std::size_t kTransportCallbacks03 =
    (offsetof(iad_transport_callbacks, set_read_timeout) + alignof(iad_transport_callbacks) - 1) /
    alignof(iad_transport_callbacks) * alignof(iad_transport_callbacks);
constexpr std::size_t kBusOptions03 = sizeof(iad_bus_options);
static_assert(sizeof(iad_bus_options) == 56 && alignof(iad_bus_options) == 8, "iad_bus_options: 56 bytes, align 8");
static_assert(offsetof(iad_transport_callbacks, write) == 24, "iad_transport_callbacks: the pointers start at 24");
static_assert(sizeof(void*) != 8 || (kTransportCallbacks03 == 96 && sizeof(iad_transport_callbacks) == 104),
              "iad_transport_callbacks: 96 bytes in 0.3, 104 from 0.5 on a 64-bit build");

thread_local std::string t_callback_error;  // iad_callback_error: the text of the next callback failure

/// The text a callback gave iad_callback_error (cleared), or `fallback`.
std::string take_callback_error(const std::string& fallback) {
    std::string text;
    text.swap(t_callback_error);
    return text.empty() ? fallback : text;
}

/// 0.5: the 1 ms timer resolution a binding's transport holds while open, as a serial port does (serial_win32.cpp):
/// the bus's gap and its polls of an in-memory transport then sleep in 1 ms steps, not the default 15.6 ms (Windows;
/// a no-op elsewhere). Its own count, apart from the waits' and monitors' process_timer().
TimerResolution& callback_timer() {
    static TimerResolution timer;
    return timer;
}

/// A Transport on the binding's callbacks (iad_transport_callbacks). Not thread-safe itself: the bus serialises it.
class CallbackTransport final : public Transport {
public:
    CallbackTransport(const iad_transport_callbacks& callbacks, std::string name, void* user)
        : cb_(callbacks), name_(std::move(name)), user_(user),
          timeout_(std::isnan(callbacks.read_timeout_s) ? kDefaultReadTimeoutS : callbacks.read_timeout_s),
          rate_(callbacks.baud_rate), timer_(std::make_unique<ScopedTimerResolution>(callback_timer())) {}
    ~CallbackTransport() override {
        if (cb_.release) cb_.release(user_);
    }

    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override {
        check_open();
        t_callback_error.clear();
        check(cb_.write(user_, data, size), "write");
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override {
        check_open();
        std::size_t got = 0;
        t_callback_error.clear();
        check(cb_.read(user_, buffer, size, timeout_, &got), "read");
        return got < size ? got : size;
    }
    void reset_input() override {
        check_open();
        if (!cb_.reset_input) return;
        t_callback_error.clear();
        check(cb_.reset_input(user_), "reset_input");
    }
    int baud_rate() const override { return cb_.set_baud_rate ? rate_ : 0; }
    void set_baud_rate(int rate) override {
        if (!cb_.set_baud_rate) throw std::logic_error(name_ + " has no baud rate (no set_baud_rate callback)");
        t_callback_error.clear();
        check(cb_.set_baud_rate(user_, rate), "set_baud_rate");
        rate_ = rate;
    }
    double read_timeout() const override { return timeout_; }
    void set_read_timeout(double seconds) override {
        if (cb_.set_read_timeout) {  // 0.5: the port takes it before the next write (a probe's shorter wait)
            t_callback_error.clear();
            check(cb_.set_read_timeout(user_, seconds), "set_read_timeout");
        }
        timeout_ = seconds;
    }
    bool has_real_timeout() const override { return cb_.real_timeout != 0; }
    void send_break(double seconds) override {
        if (!cb_.send_break) throw std::logic_error(name_ + " cannot send a line break (no send_break callback)");
        t_callback_error.clear();
        check(cb_.send_break(user_, seconds), "send_break");
    }
    void close() override {
        if (closed_.exchange(true)) return;
        if (cb_.close) cb_.close(user_);
        hold_timer(false);
    }
    bool is_open() const override { return !closed_.load(); }
    std::string name() const override { return name_; }
    std::optional<bool> present() const override {
        if (!cb_.present) return std::nullopt;
        const int32_t r = cb_.present(user_);
        if (r == IAD_TRUE) return true;
        if (r == IAD_FALSE) return false;
        return std::nullopt;
    }
    bool can_reopen() const override { return cb_.reopen != nullptr; }
    void reopen(std::optional<int> baud_rate) override {
        if (!cb_.reopen) throw std::logic_error(name_ + " cannot be reopened (no reopen callback)");
        t_callback_error.clear();
        check(cb_.reopen(user_, baud_rate ? *baud_rate : 0), "reopen");
        closed_ = false;
        hold_timer(true);
        if (baud_rate && cb_.set_baud_rate) rate_ = *baud_rate;
    }

private:
    void check_open() const {
        if (closed_.load()) throw TransportError(name_ + " is closed");
    }
    void check(int32_t result, const char* what) const {
        if (result == 0) return;
        if (result == IAD_CALLBACK_ABORT)  // (0.7, py-fixes) the callback's own error, not the port's
            throw CallerAbortError(take_callback_error(name_ + ": the transport's " + what + " callback raised"));
        throw TransportError(take_callback_error(name_ + ": the transport's " + what + " callback failed (" +
                                                 std::to_string(result) + ")"));
    }
    void hold_timer(bool hold) {
        std::lock_guard<std::mutex> lock(timer_mutex_);
        if (!hold) {
            timer_.reset();
        } else if (!timer_) {
            timer_ = std::make_unique<ScopedTimerResolution>(callback_timer());
        }
    }

    iad_transport_callbacks cb_;
    std::string name_;
    void* user_;
    double timeout_;
    int rate_;
    std::atomic<bool> closed_{false};
    std::mutex timer_mutex_;
    std::unique_ptr<ScopedTimerResolution> timer_;  // while open
};

int slave_arg(int32_t slave) {
    if (slave < 1 || slave > 247) throw std::invalid_argument("slave address must be 1..247, got " + std::to_string(slave));
    return slave;
}

int address_arg(int32_t address) {
    if (address < 0 || address > 0xFFFF)
        throw std::invalid_argument("register address must be 0..0xFFFF, got " + std::to_string(address));
    return address;
}

std::optional<int> retries_arg(int32_t retries) {
    if (retries == IAD_INT_UNKNOWN) return std::nullopt;
    if (retries < 0 || retries > 100) throw std::invalid_argument("retries must be 0..100 (IAD_INT_UNKNOWN: the bus's)");
    return retries;
}

std::optional<int> rate_arg(int32_t rate) {
    if (rate < 0) throw std::invalid_argument("baud_rate must be a rate, or 0 to keep the present one");
    return rate > 0 ? std::optional<int>(rate) : std::nullopt;
}

iad_transport* new_transport(std::shared_ptr<Transport> transport) {
    auto entry = std::make_shared<TransportEntry>();
    entry->transport = std::move(transport);
    return reinterpret_cast<iad_transport*>(register_object(ObjectKind::Transport, entry));
}

iad_bus* new_bus(std::shared_ptr<ModbusRtu> bus, bool owned) {
    auto entry = std::make_shared<BusEntry>();
    entry->bus = std::move(bus);
    entry->owned = owned;
    return reinterpret_cast<iad_bus*>(register_object(ObjectKind::Bus, entry));
}

std::shared_ptr<BusEntry> find_bus(iad_bus* handle) {
    return std::static_pointer_cast<BusEntry>(find_object(handle, ObjectKind::Bus));
}

// the virtual clock iad_clock_use_virtual installed (null otherwise)
std::mutex g_clock_mutex;
std::shared_ptr<VirtualClock> g_virtual_clock;

// ---- 0.4

/// Ends iad_bus_lock's wait when its time is up (caught there; never leaves the library).
struct LockTimedOut {};

/// One iad_bus_preemptible_begin scope of a thread. `keep` holds the bus, so its lock's address (the scope's key) is
/// never reused by another bus while the scope lives.
struct PreemptibleScope {
    const void* handle = nullptr;
    std::shared_ptr<ModbusRtu> keep;
    std::unique_ptr<PreemptibleReads> reads;
};

/// The calling thread's scopes, innermost last; ended innermost first (also when the thread ends).
struct PreemptibleStack {
    std::vector<PreemptibleScope> scopes;
    ~PreemptibleStack() {
        while (!scopes.empty()) scopes.pop_back();
    }
};

thread_local PreemptibleStack t_preemptible;

/// IAD_IAI_* flags -> IaiOptions; a bit this ABI does not know: std::logic_error (IAD_ERR_UNSUPPORTED).
IaiOptions iai_flags(uint32_t flags) {
    constexpr uint32_t known = IAD_IAI_ALLOW_UNSAFE | IAD_IAI_ALLOW_NEVER_SEND | IAD_IAI_NO_CHECK;
    if (flags & ~known) {
        char hex[16];
        std::snprintf(hex, sizeof hex, "0x%X", static_cast<unsigned>(flags));
        throw std::logic_error(std::string("IAI flags ") + hex +
                               " have bits this library does not know (a later ABI's); nothing sent");
    }
    IaiOptions o;
    o.allow_unsafe = (flags & IAD_IAI_ALLOW_UNSAFE) != 0;
    o.allow_never_send = (flags & IAD_IAI_ALLOW_NEVER_SEND) != 0;
    o.check = (flags & IAD_IAI_NO_CHECK) == 0;
    return o;
}

/// A broadcast's settle time: NaN -> `fallback`; otherwise >= 0 and finite.
double settle_arg(double settle_s, double fallback) {
    const double s = std::isnan(settle_s) ? fallback : settle_s;
    if (!(s >= 0.0) || !std::isfinite(s)) throw std::invalid_argument("settle_s must be >= 0 (NaN: the default)");
    return s;
}

/// The frame output of a broadcast, checked before anything is sent: room for `size` bytes when frame is not NULL.
void frame_room(const uint8_t* frame, size_t frame_size, size_t size, size_t* frame_length) {
    if (frame_length) *frame_length = size;
    if (!frame && frame_size > 0) throw std::invalid_argument("frame is NULL with frame_size > 0");
    if (frame && frame_size < size)
        throw BufferTooSmall("the frame is " + std::to_string(size) + " bytes, the buffer " + std::to_string(frame_size) +
                             "; nothing sent");
}

void copy_frame(const std::vector<std::uint8_t>& sent, uint8_t* frame, size_t* frame_length) {
    if (frame_length) *frame_length = sent.size();
    if (frame && !sent.empty()) std::memcpy(frame, sent.data(), sent.size());
}

/// IAD_LINE_DIRECT .. IAD_LINE_ROBONET_GATEWAY -> the kind (IAD_LINE_AUTO is not a kind).
LineKind line_kind_arg(int32_t line) {
    switch (line) {
        case IAD_LINE_DIRECT: return LineKind::Direct;
        case IAD_LINE_RCON: return LineKind::Rcon;
        case IAD_LINE_ROBONET: return LineKind::Robonet;
        case IAD_LINE_ROBONET_GATEWAY: return LineKind::RobonetGateway;
        default: break;
    }
    throw std::invalid_argument("line must be IAD_LINE_DIRECT, _RCON, _ROBONET or _ROBONET_GATEWAY, got " +
                                std::to_string(line));
}

int32_t line_code(LineKind kind) {
    switch (kind) {
        case LineKind::Rcon: return IAD_LINE_RCON;
        case LineKind::Robonet: return IAD_LINE_ROBONET;
        case LineKind::RobonetGateway: return IAD_LINE_ROBONET_GATEWAY;
        default: return IAD_LINE_DIRECT;
    }
}

}  // namespace

namespace iai_actuator_driver {
namespace capi {

std::shared_ptr<Transport> find_transport(iad_transport* handle) {
    return std::static_pointer_cast<TransportEntry>(find_object(handle, ObjectKind::Transport))->transport;
}

iad_transport* register_transport(std::shared_ptr<Transport> transport) { return new_transport(std::move(transport)); }

SerialArgs to_serial_args(const iad_serial_options* options) {
    iad_serial_options o = zeroed<iad_serial_options>();  // NULL: the defaults (0 / NaN below)
    if (options)
        o = read_input(options, "iad_serial_options");
    else
        o.read_timeout_s = std::numeric_limits<double>::quiet_NaN();
    if (o.baud_rate < 0) throw std::invalid_argument("iad_serial_options.baud_rate must be a rate, or 0 for 115200");
    SerialArgs a;
    if (o.baud_rate > 0) a.baud_rate = o.baud_rate;
    a.read_timeout_s = std::isnan(o.read_timeout_s) ? kDefaultReadTimeoutS : o.read_timeout_s;
    if (!(a.read_timeout_s >= 0.0) || !std::isfinite(a.read_timeout_s))
        throw std::invalid_argument("iad_serial_options.read_timeout_s must be >= 0 (NaN: 0.2)");
    if (o.data_bits != 0) a.framing.data_bits = o.data_bits;
    if (o.stop_bits != 0) a.framing.stop_bits = o.stop_bits;
    if (a.framing.data_bits != 7 && a.framing.data_bits != 8)
        throw std::invalid_argument("iad_serial_options.data_bits must be 7 or 8 (0: 8), not " +
                                    std::to_string(o.data_bits));
    if (a.framing.stop_bits != 1 && a.framing.stop_bits != 2)
        throw std::invalid_argument("iad_serial_options.stop_bits must be 1 or 2 (0: 1), not " +
                                    std::to_string(o.stop_bits));
    switch (o.parity) {
        case IAD_PARITY_NONE: a.framing.parity = Parity::None; break;
        case IAD_PARITY_EVEN: a.framing.parity = Parity::Even; break;
        case IAD_PARITY_ODD: a.framing.parity = Parity::Odd; break;
        default:
            throw std::invalid_argument("iad_serial_options.parity must be IAD_PARITY_NONE, IAD_PARITY_EVEN or "
                                        "IAD_PARITY_ODD, not " + std::to_string(o.parity));
    }
    return a;
}

std::atomic<int32_t>& log_level() {
    static std::atomic<int32_t> level{IAD_LOG_DEBUG};
    return level;
}

void check_cancel_latch(const Axis& axis) {
    if (axis.cancel_latched())
        throw WaitCancelledError("cancelled before it started (the cancel latch is set: iad_axis_cancel_latch; clear it "
                                 "with iad_axis_cancel_reset)");
}

std::shared_ptr<AxisEntry> find_axis_to_wait(iad_axis* handle) {
    std::shared_ptr<AxisEntry> e = find_axis(handle);
    check_cancel_latch(*e->axis);
    return e;
}

}  // namespace capi
}  // namespace iai_actuator_driver

extern "C" {

// ============================================================================================ library

iad_result IAD_CALL iad_callback_error(const char* message) {
    try {
        t_callback_error = message ? message : "";
        return IAD_OK;  // the thread's last error is left alone: this runs inside another call
    } catch (...) {
        return IAD_ERR_INTERNAL;
    }
}

iad_result IAD_CALL iad_set_log_level(int32_t level) {
    return guarded([&] {
        if (level < IAD_LOG_DEBUG || level > IAD_LOG_ERROR)
            throw std::invalid_argument("level must be IAD_LOG_DEBUG .. IAD_LOG_ERROR, got " + std::to_string(level));
        log_level().store(level);
    });
}

iad_result IAD_CALL iad_clock_use_real(void) {
    return guarded([&] {
        std::lock_guard<std::mutex> lock(g_clock_mutex);
        VirtualClock::uninstall();
        g_virtual_clock.reset();
    });
}

iad_result IAD_CALL iad_clock_use_virtual(double start_s) {
    return guarded([&] {
        const double start = std::isnan(start_s) ? 1000.0 : finite(start_s, "start_s");
        std::lock_guard<std::mutex> lock(g_clock_mutex);
        auto clock = std::make_shared<VirtualClock>(start);
        VirtualClock::install(clock);
        g_virtual_clock = std::move(clock);
    });
}

iad_result IAD_CALL iad_clock_use_callbacks(iad_clock_now_fn now, iad_clock_sleep_fn sleep, void* user) {
    return guarded([&] {
        if (!now != !sleep) throw std::invalid_argument("give both clock callbacks (now and sleep) or neither");
        std::lock_guard<std::mutex> lock(g_clock_mutex);
        g_virtual_clock.reset();
        if (!now) {
            set_clock(nullptr, nullptr);
            return;
        }
        set_clock([now, user] { return now(user); }, [sleep, user](double seconds) { sleep(seconds, user); });
    });
}

iad_result IAD_CALL iad_clock_advance(double seconds) {
    return guarded([&] {
        if (!(seconds >= 0.0) || !std::isfinite(seconds)) throw std::invalid_argument("seconds must be >= 0");
        std::lock_guard<std::mutex> lock(g_clock_mutex);
        if (!g_virtual_clock) throw EcNotReadyError("the virtual clock is not installed (iad_clock_use_virtual)");
        g_virtual_clock->advance(seconds);
    });
}

iad_result IAD_CALL iad_clock_now(double* out) {
    return guarded([&] { *need(out, "out") = now_s(); });
}

iad_result IAD_CALL iad_transport_callbacks_init(iad_transport_callbacks* callbacks) {
    return guarded([&] {
        need(callbacks, "callbacks");
        // 0.5: the struct grew, and the caller's sizeof is not known here: struct_size set to this header's size before
        // the call says the caller has it all; anything else (0, a zeroed struct) is taken as a 0.3 / 0.4
        // caller, whose struct ends before set_read_timeout, so nothing is written past its end
        std::uint32_t size = 0;
        std::memcpy(&size, callbacks, sizeof size);
        if (size != sizeof(iad_transport_callbacks)) size = static_cast<std::uint32_t>(kTransportCallbacks03);
        iad_transport_callbacks d;
        std::memset(&d, 0, sizeof d);
        d.struct_size = size;
        d.read_timeout_s = kDefaultReadTimeoutS;
        d.real_timeout = IAD_TRUE;
        std::memcpy(callbacks, &d, size);
    });
}

iad_result IAD_CALL iad_bus_options_init(iad_bus_options* options) {
    return guarded([&] {
        need(options, "options");
        const BusOptions d;
        std::memset(options, 0, sizeof *options);
        options->struct_size = static_cast<uint32_t>(sizeof *options);
        options->min_gap_s = d.min_gap_s;
        options->retries = d.retries;
    });
}

iad_result IAD_CALL iad_serial_options_init(iad_serial_options* options) {
    return guarded([&] {
        need(options, "options");
        const SerialFraming d;
        iad_serial_options c = zeroed<iad_serial_options>();
        c.read_timeout_s = kDefaultReadTimeoutS;
        c.baud_rate = kDefaultBaudRate;
        c.data_bits = d.data_bits;
        c.parity = IAD_PARITY_NONE;
        c.stop_bits = d.stop_bits;
        write_sized(options, c);  // a sized struct: as much as the caller's struct_size says
    });
}

// ============================================================================================ transports

iad_result IAD_CALL iad_transport_open_callbacks(const iad_transport_callbacks* callbacks, const char* name, void* user,
                                                 iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const iad_transport_callbacks cb =
            read_sized(need(callbacks, "callbacks"), "iad_transport_callbacks", kTransportCallbacks03);
        if (!cb.write || !cb.read) throw std::invalid_argument("iad_transport_callbacks needs write and read");
        if (!(cb.read_timeout_s >= 0.0) && !std::isnan(cb.read_timeout_s))
            throw std::invalid_argument("read_timeout_s must be >= 0 (NaN: 0.2)");
        if (cb.baud_rate < 0) throw std::invalid_argument("baud_rate must be >= 0 (0: none)");
        const std::string label = name && *name ? name : "CALLBACK";
        *out = new_transport(std::make_shared<CallbackTransport>(cb, label, user));
    });
}

iad_result IAD_CALL iad_transport_open_serial(const char* port, int32_t baud_rate, double read_timeout_s,
                                              iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        need(port, "port");
        if (!*port) throw std::invalid_argument("port name is empty");
        if (baud_rate < 0) throw std::invalid_argument("baud_rate must be a rate, or 0 for 115200");
        const double timeout = std::isnan(read_timeout_s) ? kDefaultReadTimeoutS : read_timeout_s;
        if (!(timeout >= 0.0) || !std::isfinite(timeout)) throw std::invalid_argument("read_timeout_s must be >= 0");
        *out = new_transport(
            std::make_shared<SerialTransport>(std::string(port), baud_rate > 0 ? baud_rate : kDefaultBaudRate, timeout));
    });
}

iad_result IAD_CALL iad_transport_open_serial_options(const char* port, const iad_serial_options* options,
                                                      iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        need(port, "port");
        if (!*port) throw std::invalid_argument("port name is empty");
        const SerialArgs a = to_serial_args(options);
        *out = new_transport(std::make_shared<SerialTransport>(std::string(port), a.baud_rate, a.read_timeout_s,
                                                               a.framing));
    });
}

iad_result IAD_CALL iad_transport_close(iad_transport* transport) {
    return guarded([&] {
        if (!transport) return;
        bool other_kind = false;
        auto entry = std::static_pointer_cast<TransportEntry>(release_object(transport, ObjectKind::Transport, &other_kind));
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_transport_close");
        if (entry) entry->transport->close();
    });
}

iad_result IAD_CALL iad_transport_release(iad_transport* transport) {
    return guarded([&] {
        if (!transport) return;
        bool other_kind = false;
        release_object(transport, ObjectKind::Transport, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_transport_release");
    });
}

iad_result IAD_CALL iad_transport_info_json(iad_transport* transport, char* buffer, size_t buffer_size,
                                            size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto t = find_transport(transport);
        Json j = Json::object();
        j.set("name", t->name());
        j.set("open", t->is_open());
        const int rate = t->baud_rate();
        j.set("baud_rate", rate > 0 ? Json(rate) : Json());
        j.set("read_timeout_s", t->read_timeout());
        j.set("real_timeout", t->has_real_timeout());
        j.set("present", Json(t->present()));
        j.set("can_reopen", t->can_reopen());
        write_text_output(j.dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_transport_write(iad_transport* transport, const uint8_t* data, size_t size) {
    return guarded([&] {
        const auto t = find_transport(transport);
        if (size) need(data, "data");
        t->write(data, size);
    });
}

iad_result IAD_CALL iad_transport_read(iad_transport* transport, uint8_t* buffer, size_t size, size_t* got) {
    return guarded([&] {
        need(got, "got");
        *got = 0;
        const auto t = find_transport(transport);
        if (size) need(buffer, "buffer");
        *got = t->read(buffer, size);
    });
}

iad_result IAD_CALL iad_transport_reset_input(iad_transport* transport) {
    return guarded([&] { find_transport(transport)->reset_input(); });
}

iad_result IAD_CALL iad_transport_set_baud_rate(iad_transport* transport, int32_t rate) {
    return guarded([&] {
        const auto t = find_transport(transport);
        if (rate <= 0) throw std::invalid_argument("rate must be > 0");
        t->set_baud_rate(rate);
    });
}

iad_result IAD_CALL iad_transport_set_read_timeout(iad_transport* transport, double seconds) {
    return guarded([&] {
        const auto t = find_transport(transport);
        if (!(seconds >= 0.0) || !std::isfinite(seconds)) throw std::invalid_argument("seconds must be >= 0");
        t->set_read_timeout(seconds);
    });
}

iad_result IAD_CALL iad_transport_send_break(iad_transport* transport, double seconds) {
    return guarded([&] {
        const auto t = find_transport(transport);
        if (!(seconds > 0.0) || seconds > 2.0) throw std::invalid_argument("a line break lasts 0 < seconds <= 2");
        t->send_break(seconds);
    });
}

iad_result IAD_CALL iad_transport_reopen(iad_transport* transport, int32_t baud_rate) {
    return guarded([&] {
        const auto t = find_transport(transport);
        t->reopen(rate_arg(baud_rate));
    });
}

// ============================================================================================ buses

iad_result IAD_CALL iad_bus_open(iad_transport* transport, const iad_bus_options* options, iad_bus** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        auto t = find_transport(transport);
        BusOptions o;
        if (options) {
            const iad_bus_options in = read_sized(options, "iad_bus_options", kBusOptions03);
            check_reserved(in.reserved, 4, "iad_bus_options.reserved");
            if (in.reserved0 != 0) throw std::logic_error("iad_bus_options.reserved0 is set: a field of a later ABI");
            if (!(in.min_gap_s >= 0.0) || !std::isfinite(in.min_gap_s))
                throw std::invalid_argument("min_gap_s must be >= 0");
            if (in.retries < 0 || in.retries > 100) throw std::invalid_argument("retries must be 0..100");
            o.min_gap_s = in.min_gap_s;
            o.retries = in.retries;
        }
        *out = new_bus(std::make_shared<ModbusRtu>(std::move(t), o), true);
    });
}

iad_result IAD_CALL iad_bus_close(iad_bus* bus) {
    return guarded([&] {
        if (!bus) return;
        bool other_kind = false;
        auto entry = std::static_pointer_cast<BusEntry>(release_object(bus, ObjectKind::Bus, &other_kind));
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_bus_close");
        if (entry && entry->owned) entry->bus->close();
    });
}

iad_result IAD_CALL iad_bus_transport(iad_bus* bus, iad_transport** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = new_transport(find_bus(bus)->bus->transport_ptr());
    });
}

iad_result IAD_CALL iad_bus_info_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const auto e = find_bus(bus);
        ModbusRtu& b = *e->bus;
        Json j = Json::object();
        j.set("name", b.transport().name());
        j.set("retries", b.retries());
        j.set("min_gap_s", b.min_gap_s());
        // the transport's rate without the bus lock (ModbusRtu::baud_rate waits for it): an info read never waits for
        // another thread's hold (iad_bus_lock)
        const int rate = b.transport().baud_rate();
        j.set("baud_rate", rate > 0 ? Json(rate) : Json());
        j.set("can_change_baud_rate", rate > 0);  // ModbusRtu::can_change_baud_rate, without the lock
        j.set("closed", b.closed());
        const std::shared_ptr<Line> line = b.line();
        j.set("line", line ? Json(to_string(line->kind())) : Json());
        Json unsupported = Json::object();
        for (int slave = 1; slave <= 247; ++slave) {
            const std::set<int> codes = b.unsupported_functions(slave);
            if (!codes.empty()) unsupported.set(std::to_string(slave), Json(std::vector<int>(codes.begin(), codes.end())));
        }
        j.set("unsupported_functions", unsupported);
        // 0.4: the lock, the calling thread's preemptible scope and this handle's suspensions
        const BusLock& lock = b.bus_lock();
        j.set("lock_depth", lock.depth());
        Json waiting = Json::object();
        waiting.set("normal", static_cast<long long>(lock.waiting(BusPriority::Normal)));
        waiting.set("stop", static_cast<long long>(lock.waiting(BusPriority::Stop)));
        j.set("lock_waiting", waiting);
        const PreemptibleReadOptions* pre = PreemptibleReads::current(lock);
        j.set("preemptible_words", pre ? Json(pre->words) : Json());
        {
            std::lock_guard<std::mutex> guard(e->suspend_mutex);
            long long held = 0;
            for (const auto& s : e->suspensions) held += static_cast<long long>(s.monitors().size());
            j.set("suspended_monitors", held);
        }
        write_text_output(j.dump(), buffer, buffer_size, required);
    });
}

iad_result IAD_CALL iad_bus_set_retries(iad_bus* bus, int32_t retries) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (retries < 0 || retries > 100) throw std::invalid_argument("retries must be 0..100");
        e->bus->set_retries(retries);
    });
}

iad_result IAD_CALL iad_bus_set_min_gap(iad_bus* bus, double seconds) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (!(seconds >= 0.0) || !std::isfinite(seconds)) throw std::invalid_argument("seconds must be >= 0");
        e->bus->set_min_gap_s(seconds);
    });
}

iad_result IAD_CALL iad_bus_set_frame_sink(iad_bus* bus, iad_frame_sink sink, void* user) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (!sink) {
            e->bus->set_frame_logger(FrameLogger());
            return;
        }
        e->bus->set_frame_logger([sink, user](FrameDirection direction, const std::vector<std::uint8_t>& bytes) {
            sink(static_cast<int32_t>(direction), bytes.data(), bytes.size(), user);
        });
    });
}

iad_result IAD_CALL iad_bus_read_registers(iad_bus* bus, int32_t slave, int32_t address, int32_t count, int32_t retries,
                                           double max_wait_s, int32_t chunk, uint16_t* values) {
    return guarded([&] {
        const auto e = find_bus(bus);
        need(values, "values");
        if (count < 1 || count > 0x10000) throw std::invalid_argument("count must be >= 1");
        ReadOptions o;
        o.retries = retries_arg(retries);
        if (!std::isnan(max_wait_s)) {
            if (!(max_wait_s > 0.0) || !std::isfinite(max_wait_s))
                throw std::invalid_argument("max_wait_s must be > 0 (NaN: no cap)");
            o.max_wait_s = max_wait_s;
        }
        if (chunk != IAD_INT_UNKNOWN) o.chunk = chunk;  // 1..125, checked by the bus
        const std::vector<std::uint16_t> words =
            e->bus->read_registers(slave_arg(slave), address_arg(address), count, o);
        std::memcpy(values, words.data(), words.size() * sizeof(std::uint16_t));
    });
}

iad_result IAD_CALL iad_bus_write_coil(iad_bus* bus, int32_t slave, int32_t address, iad_bool on, int32_t retries) {
    return guarded([&] {
        const auto e = find_bus(bus);
        e->bus->write_coil(slave_arg(slave), address_arg(address), on != 0, retries_arg(retries));
    });
}

iad_result IAD_CALL iad_bus_write_register(iad_bus* bus, int32_t slave, int32_t address, uint16_t value) {
    return guarded([&] { find_bus(bus)->bus->write_register(slave_arg(slave), address_arg(address), value); });
}

iad_result IAD_CALL iad_bus_write_registers(iad_bus* bus, int32_t slave, int32_t address, const uint16_t* values,
                                            size_t count) {
    return guarded([&] {
        const auto e = find_bus(bus);
        need(values, "values");
        if (count < 1) throw std::invalid_argument("count must be >= 1");
        e->bus->write_registers(slave_arg(slave), address_arg(address), std::vector<std::uint16_t>(values, values + count));
    });
}

iad_result IAD_CALL iad_bus_read_write_registers(iad_bus* bus, int32_t slave, int32_t read_address, int32_t read_count,
                                                 int32_t write_address, const uint16_t* values, size_t count,
                                                 uint16_t* out) {
    return guarded([&] {
        const auto e = find_bus(bus);
        need(values, "values");
        need(out, "out");
        if (count < 1 || read_count < 1) throw std::invalid_argument("count and read_count must be >= 1");
        const std::vector<std::uint16_t> words =
            e->bus->read_write_registers(slave_arg(slave), address_arg(read_address), read_count,
                                         address_arg(write_address), std::vector<std::uint16_t>(values, values + count));
        std::memcpy(out, words.data(), words.size() * sizeof(std::uint16_t));
    });
}

iad_result IAD_CALL iad_bus_report_slave_id(iad_bus* bus, int32_t slave, int32_t retries, double timeout_s,
                                            uint8_t* buffer, size_t buffer_size, size_t* length) {
    return guarded([&] {
        need(length, "length");
        *length = 0;
        if (!buffer && buffer_size > 0) throw std::invalid_argument("buffer is NULL with buffer_size > 0");
        const auto e = find_bus(bus);
        if (!std::isnan(timeout_s) && (!(timeout_s > 0.0) || !std::isfinite(timeout_s)))
            throw std::invalid_argument("timeout_s must be > 0 (NaN: the transport's timeout only)");
        // ModbusRtu::report_slave_id: a wait of 0 = the transport's own timeout only
        const double wait = std::isnan(timeout_s) ? 0.0 : timeout_s;
        const std::vector<std::uint8_t> bytes = e->bus->report_slave_id(slave_arg(slave), retries_arg(retries), wait);
        *length = bytes.size();
        if (bytes.size() > buffer_size)
            throw BufferTooSmall("the slave id is " + std::to_string(bytes.size()) + " bytes, the buffer " +
                                 std::to_string(buffer_size));
        if (!bytes.empty()) std::memcpy(buffer, bytes.data(), bytes.size());
    });
}

iad_result IAD_CALL iad_bus_iai_command(iad_bus* bus, int32_t slave, int32_t function_code, int32_t group,
                                        int32_t command, const uint32_t* operands, size_t operand_count, uint32_t flags,
                                        int32_t retries, int32_t* rejection, uint32_t* status, size_t status_count) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (function_code != 0 && function_code != 0x45 && function_code != 0x48)
            throw std::logic_error("function_code must be 0x45 (or 0) or 0x48, got " + std::to_string(function_code) +
                                   "; nothing sent");
        const bool command2 = function_code == 0x48;
        const std::size_t most = command2 ? 4 : 2;
        if (operand_count > most)
            throw std::invalid_argument("at most " + std::to_string(most) + " operands for 0x" +
                                        (command2 ? std::string("48") : std::string("45")) + ", got " +
                                        std::to_string(operand_count));
        if (operand_count) need(operands, "operands");
        IaiOptions o = iai_flags(flags);
        o.retries = retries_arg(retries);
        std::vector<std::uint32_t> words;
        if (command2) {
            const std::vector<std::uint32_t> ops(operands, operands + operand_count);
            const IaiResponse2 r = e->bus->iai_command2(slave_arg(slave), command, ops, o, group);
            if (rejection) *rejection = r.rejection;
            words.assign(r.status, r.status + 4);
        } else {
            int op[2] = {0, 0};
            for (std::size_t i = 0; i < operand_count; ++i) {
                if (operands[i] > 0xFFFFu)  // the message of ModbusRtu::iai_command's own check
                    throw std::invalid_argument(std::string(i == 0 ? "op1" : "op2") + " must be 0..65535, got " +
                                                std::to_string(operands[i]));
                op[i] = static_cast<int>(operands[i]);
            }
            const IaiResponse r = e->bus->iai_command(slave_arg(slave), group, command, op[0], op[1], o);
            if (rejection) *rejection = r.rejection;
            words.push_back(static_cast<std::uint32_t>(r.status));
        }
        if (status)
            for (std::size_t i = 0; i < words.size() && i < status_count; ++i) status[i] = words[i];
    });
}

iad_result IAD_CALL iad_bus_broadcast_coil(iad_bus* bus, int32_t address, iad_bool on) {
    return guarded([&] { find_bus(bus)->bus->broadcast_coil(address_arg(address), on != 0); });
}

iad_result IAD_CALL iad_bus_probe(iad_bus* bus, int32_t slave, double timeout_s, int32_t address, iad_bool* answered) {
    return guarded([&] {
        need(answered, "answered");
        *answered = IAD_FALSE;
        const auto e = find_bus(bus);
        const double timeout = std::isnan(timeout_s) ? kScanTimeoutS : timeout_s;
        if (!(timeout > 0.0) || !std::isfinite(timeout)) throw std::invalid_argument("timeout_s must be > 0");
        *answered = e->bus->probe(slave_arg(slave), timeout, address_arg(address)) ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_bus_detect_baud_rate(iad_bus* bus, const char* targets, const char* candidates,
                                             int32_t probe_address, iad_bool current_first, double probe_timeout_s,
                                             int32_t* rate) {
    return guarded([&] {
        need(rate, "rate");
        *rate = 0;
        const auto e = find_bus(bus);
        // "1,2,0xD0@0x3700": a slave, or slave@register; decimal or 0x hex
        auto number = [](const std::string& text) {
            const bool hex = text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X');
            std::size_t used = 0;
            long value = -1;
            try {
                value = std::stol(text, &used, hex ? 16 : 10);
            } catch (const std::exception&) {
                used = 0;
            }
            if (used == 0 || used != text.size() || value < 0 || value > 0xFFFF)
                throw std::invalid_argument("targets: '" + text + "' is not a number (decimal or 0x hex)");
            return static_cast<int>(value);
        };
        std::vector<ModbusRtu::BaudProbe> list;
        for (const std::string& item : split_list(targets, false)) {
            const std::size_t at = item.find('@');
            ModbusRtu::BaudProbe probe;
            probe.slave = slave_arg(number(item.substr(0, at)));
            if (at != std::string::npos) probe.address = address_arg(number(item.substr(at + 1)));
            list.push_back(probe);
        }
        if (list.empty()) list.push_back(ModbusRtu::BaudProbe{1, std::nullopt});
        const std::vector<int> c = to_int_list(candidates, "candidates");
        const int address = probe_address == IAD_INT_UNKNOWN ? 0x9000 : address_arg(probe_address);
        std::optional<double> wait;
        if (!std::isnan(probe_timeout_s)) {
            if (!(probe_timeout_s > 0.0) || !std::isfinite(probe_timeout_s))
                throw std::invalid_argument("probe_timeout_s must be > 0 (NaN: the default)");
            wait = probe_timeout_s;
        }
        *rate = e->bus->detect_baud_rate_targets(list, c, address, current_first != 0, wait);
    });
}

iad_result IAD_CALL iad_bus_set_baud_rate(iad_bus* bus, int32_t rate) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (rate <= 0) throw std::invalid_argument("rate must be > 0");
        e->bus->set_baud_rate(rate);
    });
}

iad_result IAD_CALL iad_bus_send_break(iad_bus* bus, double seconds) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (!(seconds > 0.0) || seconds > 2.0) throw std::invalid_argument("a line break lasts 0 < seconds <= 2");
        e->bus->send_break(seconds);
    });
}

iad_result IAD_CALL iad_bus_reopen(iad_bus* bus, int32_t baud_rate) {
    return guarded([&] { find_bus(bus)->bus->reopen(rate_arg(baud_rate)); });
}

// ============================================================================================ axes and cells

iad_result IAD_CALL iad_axis_open_bus(iad_bus* bus, int32_t slave, const iad_axis_options* axis_options,
                                      iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        const auto e = find_bus(bus);
        const AxisOptions o = to_axis_options(axis_options);
        *out = register_axis(std::make_shared<Axis>(e->bus, slave_arg(slave), o));
    });
}

iad_result IAD_CALL iad_axis_open_transport(iad_transport* transport, const iad_open_options* options,
                                            const iad_axis_options* axis_options, iad_axis** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        auto t = find_transport(transport);
        const OpenOptions o = to_open_options(options, axis_options);
        *out = register_axis(std::make_shared<Axis>(Axis::open(std::move(t), o)));
    });
}

iad_result IAD_CALL iad_axis_bus(iad_axis* axis, iad_bus** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        *out = new_bus(find_axis(axis)->axis->bus(), false);
    });
}

iad_result IAD_CALL iad_cell_open_transports(iad_transport* const* transports, size_t transport_count,
                                             const char* names, const char* slaves, const iad_cell_options* options,
                                             const iad_axis_options* axis_options, iad_cell** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        if (transport_count < 1) throw std::invalid_argument("transport_count must be >= 1");
        need(transports, "transports");
        const std::vector<std::string> given = split_list(names, false);
        if (!given.empty() && given.size() != transport_count)
            throw std::invalid_argument("names lists " + std::to_string(given.size()) + " names for " +
                                        std::to_string(transport_count) + " transports");
        std::map<std::string, std::shared_ptr<Transport>> by_name;
        for (std::size_t i = 0; i < transport_count; ++i) {
            auto t = find_transport(transports[i]);
            const std::string name = given.empty() ? t->name() : given[i];
            if (!by_name.emplace(name, std::move(t)).second)
                throw std::invalid_argument("two transports are named '" + name + "' (give names)");
        }
        const std::vector<int> s = to_int_list(slaves, "slaves");
        for (int v : s) slave_arg(v);
        auto entry = std::make_shared<CellEntry>();
        const CellOptions o = queueing_cell_options(to_cell_options(options, axis_options), entry->events);
        entry->cell = std::make_shared<Cell>(Cell::open(by_name, s, o));
        *out = register_cell(std::move(entry));
    });
}

iad_result IAD_CALL iad_cell_bus(iad_cell* cell, const char* port, iad_bus** out) {
    return guarded([&] {
        need(out, "out");
        *out = nullptr;
        need(port, "port");
        const auto e = find_cell(cell);
        const auto ports = e->cell->ports();
        auto it = ports.find(port);
        if (it == ports.end()) throw std::out_of_range("no port '" + std::string(port) + "' in this cell");
        *out = new_bus(it->second->bus(), false);
    });
}

// ============================================================================================ cancel latch

iad_result IAD_CALL iad_axis_cancel_latch(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->set_cancel_latch(true); });
}

iad_result IAD_CALL iad_axis_cancel_reset(iad_axis* axis) {
    return guarded([&] { find_axis(axis)->axis->set_cancel_latch(false); });
}

iad_result IAD_CALL iad_axis_cancel_latched(iad_axis* axis, iad_bool* latched) {
    return guarded([&] {
        need(latched, "latched");
        *latched = find_axis(axis)->axis->cancel_latched() ? IAD_TRUE : IAD_FALSE;
    });
}

iad_result IAD_CALL iad_cell_cancel_latch(iad_cell* cell, const char* axes) {
    return guarded([&] {
        const auto e = find_cell(cell);
        cancel_listed(*e->cell, axes, "cancel_latch", [](Axis& axis) { axis.set_cancel_latch(true); });
    });
}

iad_result IAD_CALL iad_cell_cancel_reset(iad_cell* cell, const char* axes) {
    return guarded([&] {
        const auto e = find_cell(cell);
        cancel_listed(*e->cell, axes, "cancel_reset", [](Axis& axis) { axis.set_cancel_latch(false); });
    });
}

// ============================================================================================ 0.4: bus lock

iad_result IAD_CALL iad_bus_lock(iad_bus* bus, double timeout_s, iad_bool* locked) {
    return guarded([&] {
        need(locked, "locked");
        *locked = IAD_FALSE;
        const auto e = find_bus(bus);
        if (!std::isnan(timeout_s) && !(timeout_s >= 0.0))
            throw std::invalid_argument("timeout_s must be >= 0 (NaN: no limit)");
        ModbusRtu& b = *e->bus;
        if (b.closed()) throw TransportError("bus is closed");
        BusLock& lock = b.bus_lock();
        if (timeout_s == 0.0) {
            *locked = lock.try_lock() ? IAD_TRUE : IAD_FALSE;
            return;
        }
        // the time on the real clock: the wait runs in real-time slices (bus_lock.hpp), whatever clock is set. The
        // elapsed seconds are compared as a double: a steady_clock deadline cannot hold a huge finite timeout (above
        // about 9.2e9 s it overflows into the past), which then waits as long as it takes
        const bool limited = std::isfinite(timeout_s);
        const auto start = std::chrono::steady_clock::now();
        const auto elapsed_s = [start] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };
        try {
            lock.lock(BusPriority::Normal, [&] {
                if (b.closed()) throw TransportError("bus is closed");
                if (limited && elapsed_s() >= timeout_s) throw LockTimedOut{};
            });
        } catch (const LockTimedOut&) {
            return;  // not held: *locked stays IAD_FALSE
        }
        if (b.closed()) {  // closed while this thread took it
            lock.unlock();
            throw TransportError("bus is closed");
        }
        *locked = IAD_TRUE;
    });
}

iad_result IAD_CALL iad_bus_unlock(iad_bus* bus) {
    return guarded([&] {
        BusLock& lock = find_bus(bus)->bus->bus_lock();
        if (!lock.owned_by_this_thread())
            throw std::invalid_argument("the calling thread does not hold the bus lock (iad_bus_lock)");
        lock.unlock();
    });
}

// ============================================================================================ 0.4: preemptible reads

iad_result IAD_CALL iad_bus_preemptible_begin(iad_bus* bus, int32_t words, iad_read_gate_fn gate, void* user) {
    return guarded([&] {
        const auto e = find_bus(bus);
        const int n = words == 0 ? kPreemptibleReadWords : words;
        if (n < 1 || n > 125) throw std::invalid_argument("words must be 1..125 (0: 32), got " + std::to_string(words));
        PreemptibleReadOptions o;
        o.words = n;
        o.bus = &e->bus->bus_lock();
        if (gate) {
            o.gate = [gate, user] {
                t_callback_error.clear();
                if (gate(user) != 0)
                    throw WaitCancelledError(
                        take_callback_error("the preemptible read's gate ended the read; nothing more sent"));
            };
        }
        PreemptibleScope scope;
        scope.handle = bus;
        scope.keep = e->bus;
        scope.reads = std::make_unique<PreemptibleReads>(std::move(o));  // the thread's innermost scope from here on
        t_preemptible.scopes.push_back(std::move(scope));
    });
}

iad_result IAD_CALL iad_bus_preemptible_end(iad_bus* bus) {
    return guarded([&] {
        auto& scopes = t_preemptible.scopes;
        if (scopes.empty())
            throw std::invalid_argument("the calling thread has no preemptible scope to end (iad_bus_preemptible_begin)");
        // by the handle that began it (also after that handle was closed), else any handle on the same bus
        const bool same = scopes.back().handle == static_cast<const void*>(bus) ||
                          &find_bus(bus)->bus->bus_lock() == scopes.back().reads->options().bus;
        if (!same)
            throw std::invalid_argument(
                "the calling thread's innermost preemptible scope is on another bus: end that one first");
        scopes.pop_back();
    });
}

// ============================================================================================ 0.4: status monitors

iad_result IAD_CALL iad_bus_suspend_monitors(iad_bus* bus, double timeout_s, int32_t* monitors) {
    return guarded([&] {
        if (monitors) *monitors = 0;
        const auto e = find_bus(bus);
        const double timeout = std::isnan(timeout_s) ? 1.0 : timeout_s;
        if (!(timeout >= 0.0) || !std::isfinite(timeout)) throw std::invalid_argument("timeout_s must be >= 0 (NaN: 1 s)");
        BusSuspension held = suspend_bus(*e->bus, timeout);
        const auto count = static_cast<int32_t>(held.monitors().size());
        {
            std::lock_guard<std::mutex> guard(e->suspend_mutex);
            e->suspensions.push_back(std::move(held));
        }
        if (monitors) *monitors = count;
    });
}

iad_result IAD_CALL iad_bus_resume_monitors(iad_bus* bus) {
    return guarded([&] {
        const auto e = find_bus(bus);
        BusSuspension last;
        {
            std::lock_guard<std::mutex> guard(e->suspend_mutex);
            if (e->suspensions.empty())
                throw std::invalid_argument("this bus handle holds no suspension (iad_bus_suspend_monitors)");
            last = std::move(e->suspensions.back());
            e->suspensions.pop_back();
        }
        last.release();  // outside the handle's mutex: the monitors poll again
    });
}

// ============================================================================================ 0.4: broadcasts

iad_result IAD_CALL iad_bus_broadcast_iai_command(iad_bus* bus, int32_t group, int32_t command, int32_t op1,
                                                  int32_t op2, uint32_t flags, double settle_s, uint8_t* frame,
                                                  size_t frame_size, size_t* frame_length) {
    return guarded([&] {
        if (frame_length) *frame_length = 0;
        const auto e = find_bus(bus);
        const IaiOptions o = iai_flags(flags);
        const double settle = settle_arg(settle_s, kBaudSettleS);
        frame_room(frame, frame_size, 10, frame_length);  // 00 45 group cmd op1 op2 + CRC
        copy_frame(e->bus->broadcast_iai_command(group, command, op1, op2, o, settle), frame, frame_length);
    });
}

iad_result IAD_CALL iad_bus_broadcast(iad_bus* bus, const uint8_t* pdu, size_t size, double settle_s, uint8_t* frame,
                                      size_t frame_size, size_t* frame_length) {
    return guarded([&] {
        if (frame_length) *frame_length = 0;
        const auto e = find_bus(bus);
        if (size) need(pdu, "pdu");
        if (size > 252) throw std::invalid_argument("a PDU is at most 252 bytes, got " + std::to_string(size));
        const double settle = settle_arg(settle_s, kBroadcastTurnaroundS);
        frame_room(frame, frame_size, size + 3, frame_length);  // address 0 + the PDU + CRC
        const std::vector<std::uint8_t> p(pdu, pdu + size);
        copy_frame(e->bus->broadcast(p, settle), frame, frame_length);
    });
}

// ============================================================================================ 0.4: the line

iad_result IAD_CALL iad_bus_set_line(iad_bus* bus, int32_t line, double position_unit_mm) {
    return guarded([&] {
        const auto e = find_bus(bus);
        if (line == IAD_LINE_AUTO)
            throw std::invalid_argument("IAD_LINE_AUTO probes the line: use iad_bus_detect_line (iad_bus_set_line states "
                                        "a kind)");
        const LineKind kind = line_kind_arg(line);
        e->bus->set_line(std::make_shared<Line>(kind, nullptr, false, to_position_unit(position_unit_mm)));
    });
}

iad_result IAD_CALL iad_bus_detect_line(iad_bus* bus, int32_t line, double timeout_s, double position_unit_mm,
                                        int32_t* found) {
    return guarded([&] {
        if (found) *found = IAD_INT_UNKNOWN;
        const auto e = find_bus(bus);
        const std::optional<LineKind> kind =
            line == IAD_LINE_AUTO ? std::nullopt : std::optional<LineKind>(line_kind_arg(line));
        const double timeout = std::isnan(timeout_s) ? kGatewayProbeTimeoutS : timeout_s;
        if (!(timeout > 0.0) || !std::isfinite(timeout)) throw std::invalid_argument("timeout_s must be > 0 (NaN: 0.05)");
        const std::shared_ptr<Line> attached = detect_line(*e->bus, kind, timeout, to_position_unit(position_unit_mm));
        if (found) *found = line_code(attached->kind());
    });
}

iad_result IAD_CALL iad_bus_line_json(iad_bus* bus, char* buffer, size_t buffer_size, size_t* required) {
    return guarded([&] {
        check_text_output(buffer, buffer_size, required);
        const std::shared_ptr<Line> line = find_bus(bus)->bus->line();
        write_text_output((line ? to_json(line->snapshot()) : Json()).dump(), buffer, buffer_size, required);
    });
}

// ============================================================================================ 0.4: response timing

iad_result IAD_CALL iad_bus_set_response_delay(iad_bus* bus, int32_t slave, double delay_ms) {
    return guarded([&] {
        const auto e = find_bus(bus);
        e->bus->set_response_delay_ms(slave_arg(slave), finite(delay_ms, "delay_ms"));
    });
}

iad_result IAD_CALL iad_bus_response_delay(iad_bus* bus, int32_t slave, double* delay_ms) {
    return guarded([&] {
        need(delay_ms, "delay_ms");
        *delay_ms = std::numeric_limits<double>::quiet_NaN();
        *delay_ms = find_bus(bus)->bus->response_delay_ms(slave_arg(slave));
    });
}

iad_result IAD_CALL iad_bus_response_timeout(iad_bus* bus, int32_t slave, const uint8_t* pdu, size_t size,
                                             int32_t expected_length, double* seconds) {
    return guarded([&] {
        need(seconds, "seconds");
        *seconds = std::numeric_limits<double>::quiet_NaN();
        const auto e = find_bus(bus);
        if (size) need(pdu, "pdu");
        if (expected_length < 0 || expected_length > 256)
            throw std::invalid_argument("expected_length must be 0..256 (0: 64 bytes assumed), got " +
                                        std::to_string(expected_length));
        const std::vector<std::uint8_t> p(pdu, pdu + size);
        const std::optional<int> expected = expected_length ? std::optional<int>(expected_length) : std::nullopt;
        *seconds = e->bus->response_timeout(slave_arg(slave), p, expected);
    });
}

// ============================================================================================ 0.5 (core-bus): buses

iad_result IAD_CALL iad_bus_shutdown(iad_bus* bus) {
    return guarded([&] {
        if (!bus) return;
        find_bus(bus)->bus->close();  // the monitors stop, the transaction in flight ends, the transport closes
    });
}

iad_result IAD_CALL iad_bus_release(iad_bus* bus) {
    return guarded([&] {
        if (!bus) return;
        bool other_kind = false;
        release_object(bus, ObjectKind::Bus, &other_kind);
        if (other_kind) throw std::invalid_argument("another kind of handle was passed to iad_bus_release");
    });
}

// ============================================================================================ 0.6 (fixes): buses

iad_result IAD_CALL iad_bus_set_transport(iad_bus* bus, iad_transport* transport) {
    return guarded([&] {
        const auto e = find_bus(bus);
        e->bus->set_transport(find_transport(transport));
    });
}

}  // extern "C"
