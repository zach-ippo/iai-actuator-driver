// Diagnostic log hook (header-only). With no handler installed nothing is logged. A handler that throws is ignored:
// logging must never break communication. One process-wide handler, guarded by a mutex (handlers run in the thread
// that logs, possibly a Cell worker thread; a line logged while that thread holds a bus comes right after it lets the
// bus go).
#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace iai_actuator_driver {

/// Severity of a log line.
enum class LogLevel { Debug, Info, Warning, Error };

/// "debug", "info", "warning", "error".
inline const char* to_string(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Debug: return "debug";
        case LogLevel::Info: return "info";
        case LogLevel::Warning: return "warning";
        default: return "error";
    }
}

/// (level, source such as "iai_actuator_driver.axis", message).
using LogHandler = std::function<void(LogLevel, const std::string& source, const std::string& message)>;

namespace detail {
inline std::mutex& log_mutex() {
    static std::mutex m;
    return m;
}
inline LogHandler& log_slot() {
    static LogHandler handler;
    return handler;
}

/// The calls into the caller's code (log lines, frame logger entries) a thread makes while it holds a bus: kept in
/// order and made right after it lets the bus go, on the same thread (DeferSinkCalls). A handler that blocks or takes
/// an interpreter's lock then never delays a STOP that waits for the bus.
struct DeferredSinkCalls {
    int depth = 0;
    std::vector<std::function<void()>> calls;
};
inline DeferredSinkCalls& deferred_sink_calls() {
    thread_local DeferredSinkCalls state;
    return state;
}

/// `call` now, or (while the thread defers: DeferSinkCalls) right after the outermost scope ends. Never throws; a
/// call that cannot be kept is dropped.
inline void call_or_defer(std::function<void()> call) noexcept {
    try {
        DeferredSinkCalls& state = deferred_sink_calls();
        if (state.depth > 0) {
            state.calls.push_back(std::move(call));
            return;
        }
        call();
    } catch (...) {
        // a broken handler must never break communication
    }
}

/// While one lives (nested: the outermost counts), the thread's log lines and frame logger entries wait; when the
/// outermost ends they are passed on in order. ModbusRtu holds one for as long as it holds the bus lock.
class DeferSinkCalls {
public:
    DeferSinkCalls() noexcept { ++deferred_sink_calls().depth; }
    ~DeferSinkCalls() { end(); }
    DeferSinkCalls(const DeferSinkCalls&) = delete;
    DeferSinkCalls& operator=(const DeferSinkCalls&) = delete;
    /// End the scope early (idempotent); the kept calls are made when it was the outermost.
    void end() noexcept {
        if (ended_) return;
        ended_ = true;
        DeferredSinkCalls& state = deferred_sink_calls();
        if (--state.depth > 0) return;
        while (!state.calls.empty()) {
            std::vector<std::function<void()>> calls;
            calls.swap(state.calls);
            for (auto& call : calls) {
                try {
                    call();
                } catch (...) {
                }
            }
        }
    }

private:
    bool ended_ = false;
};
}  // namespace detail

/// Install (or, with an empty function, remove) the process-wide log handler. Returns the previous one.
inline LogHandler set_log_handler(LogHandler handler) {
    std::lock_guard<std::mutex> lock(detail::log_mutex());
    LogHandler previous = std::move(detail::log_slot());
    detail::log_slot() = std::move(handler);
    return previous;
}

/// Send one line to the handler (no-op without one), on the calling thread; a line logged while the thread holds a
/// bus is passed right after it lets the bus go (detail::DeferSinkCalls). Never throws.
inline void log(LogLevel level, const std::string& source, const std::string& message) noexcept {
    try {
        LogHandler handler;
        {
            std::lock_guard<std::mutex> lock(detail::log_mutex());
            handler = detail::log_slot();
        }
        if (!handler) return;
        detail::DeferredSinkCalls& deferred = detail::deferred_sink_calls();
        if (deferred.depth > 0) {
            deferred.calls.push_back([level, source, message] {
                LogHandler current;  // the handler installed when the line is passed on
                {
                    std::lock_guard<std::mutex> lock(detail::log_mutex());
                    current = detail::log_slot();
                }
                if (current) current(level, source, message);
            });
            return;
        }
        handler(level, source, message);
    } catch (...) {
        // a broken log handler must never break communication
    }
}

inline void log_debug(const std::string& source, const std::string& message) noexcept { log(LogLevel::Debug, source, message); }
inline void log_info(const std::string& source, const std::string& message) noexcept { log(LogLevel::Info, source, message); }
inline void log_warning(const std::string& source, const std::string& message) noexcept { log(LogLevel::Warning, source, message); }
inline void log_error(const std::string& source, const std::string& message) noexcept { log(LogLevel::Error, source, message); }

}  // namespace iai_actuator_driver
