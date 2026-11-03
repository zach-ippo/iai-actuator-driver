// Private helpers for the threads the library starts (internal: not installed, not part of the API). A std::thread
// destroyed while still joinable ends the process (std::terminate), so every such thread is joined on every path: also
// when starting a later thread of the same group throws.
#pragma once

#include <thread>
#include <vector>

namespace iai_actuator_driver {
namespace util {

/// Join every joinable thread of `threads`. Never throws: a join that fails detaches the thread instead.
inline void join_all(std::vector<std::thread>& threads) noexcept {
    for (auto& t : threads) {
        if (!t.joinable()) continue;
        try {
            t.join();
        } catch (...) {
            try {
                t.detach();
            } catch (...) {
            }
        }
    }
}

/// join_all() when the scope ends, an exception included (a later thread that could not start).
class JoinThreads {
public:
    explicit JoinThreads(std::vector<std::thread>& threads) noexcept : threads_(threads) {}
    ~JoinThreads() { join_all(threads_); }
    JoinThreads(const JoinThreads&) = delete;
    JoinThreads& operator=(const JoinThreads&) = delete;

private:
    std::vector<std::thread>& threads_;
};

}  // namespace util
}  // namespace iai_actuator_driver
