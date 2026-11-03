// HookedSimUnit (sim_hooks.hpp): the Python edition's SimulatedEC test hooks on top of a SimulatedUnit. Test tooling;
// UNVERIFIED on hardware.
#include "iai_actuator_driver/sim_hooks.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {

namespace {

constexpr std::size_t kLogLimit = 2000;
constexpr int kFcRead = 0x03;
constexpr int kFcWriteMany = 0x10;
constexpr int kFcReadWrite = 0x17;
constexpr std::uint16_t kDsseDst1 = 1u << 4;  // seen set while moving (live); cleared with MOVE by a stale poll

int word_at(const std::vector<std::uint8_t>& frame, std::size_t offset) {
    return (frame[offset] << 8) | frame[offset + 1];
}

bool overlaps(int start, int count, const std::pair<int, int>& range) {
    return start < range.first + range.second && range.first < start + count;
}

}  // namespace

struct HookedSimUnit::State {
    // clock (its own mutex: the unit calls the clock with its own lock held)
    mutable std::mutex clock_mutex;
    std::function<double()> base_clock;
    double offset = 0.0;  ///< added to base_clock since set_clock, so the unit's time does not jump
    std::optional<double> step;
    double stepped_now = 1000.0;

    // hooks
    mutable std::mutex mutex;
    SimHookOptions hooks;
    int stale_left = 0;
    int stale_served = 0;
    SimRequestHook request_hook;
    std::deque<std::pair<double, std::vector<std::uint8_t>>> log;
    std::vector<std::uint8_t> pending;  ///< a hooked or patched answer not read yet

    double now() const {
        std::lock_guard<std::mutex> lock(clock_mutex);
        if (step) return stepped_now;
        return base_now();
    }

    double base_now() const { return (base_clock ? base_clock() : now_s()) + offset; }  // clock_mutex held
};

HookedSimUnit::HookedSimUnit(const SimulatedUnitOptions& options, const SimHookOptions& hooks)
    : HookedSimUnit(options, hooks, std::make_shared<State>()) {}

HookedSimUnit::HookedSimUnit(SimulatedUnitOptions options, const SimHookOptions& hooks, std::shared_ptr<State> state)
    : SimulatedUnit([&] {
          if (hooks.step_s && (!(*hooks.step_s >= 0.0) || !std::isfinite(*hooks.step_s)))
              throw std::invalid_argument("step_s must be a number of seconds >= 0");
          state->base_clock = options.clock;
          state->step = hooks.step_s;
          const std::shared_ptr<State> s = state;
          options.clock = [s] { return s->now(); };
          return options;
      }()),
      state_(std::move(state)) {
    for (const auto& range : hooks.unreadable) set_unreadable(range.first, range.second, true);
    for (const auto& kv : hooks.word_overrides) set_word_override(kv.first, kv.second);
    set_stale_status_polls(hooks.stale_status_polls);
}

HookedSimUnit::~HookedSimUnit() = default;

// ---- hooks

void HookedSimUnit::set_word_override(int address, std::optional<std::uint16_t> value) {
    if (address < 0 || address > 0xFFFF)
        throw std::invalid_argument("register address must be 0..0xFFFF, got " + std::to_string(address));
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (value)
        state_->hooks.word_overrides[address] = *value;
    else
        state_->hooks.word_overrides.erase(address);
}

std::map<int, std::uint16_t> HookedSimUnit::word_overrides() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->hooks.word_overrides;
}

void HookedSimUnit::set_unreadable(int start, int count, bool unreadable) {
    if (count < 1 || start < 0 || start + count > 0x10000)
        throw std::invalid_argument("an unreadable range needs start 0..0xFFFF and count >= 1 inside it, got start " +
                                    std::to_string(start) + " count " + std::to_string(count));
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto& ranges = state_->hooks.unreadable;
    if (unreadable) {
        ranges.emplace_back(start, count);
        return;
    }
    ranges.erase(std::remove_if(ranges.begin(), ranges.end(),
                                [&](const std::pair<int, int>& r) { return overlaps(start, count, r); }),
                 ranges.end());
}

std::vector<std::pair<int, int>> HookedSimUnit::unreadable() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->hooks.unreadable;
}

void HookedSimUnit::set_stale_status_polls(int polls) {
    if (polls < 0) throw std::invalid_argument("stale_status_polls must be >= 0, got " + std::to_string(polls));
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->hooks.stale_status_polls = polls;
    if (polls == 0) state_->stale_left = 0;
}

int HookedSimUnit::stale_status_polls() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->hooks.stale_status_polls;
}

int HookedSimUnit::stale_polls_served() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->stale_served;
}

void HookedSimUnit::set_step_s(std::optional<double> step_s) {
    std::lock_guard<std::mutex> lock(state_->clock_mutex);
    if (!step_s) {
        if (state_->step) throw std::logic_error("the stepped clock cannot be turned off (set a step of 0 instead)");
        return;
    }
    if (!(*step_s >= 0.0) || !std::isfinite(*step_s)) throw std::invalid_argument("step_s must be a number of seconds >= 0");
    if (!state_->step)
        throw std::logic_error("the stepped clock is chosen when the unit is made (step_s in its options)");
    state_->step = step_s;
}

std::optional<double> HookedSimUnit::step_s() const {
    std::lock_guard<std::mutex> lock(state_->clock_mutex);
    return state_->step;
}

double HookedSimUnit::clock_now() const { return state_->now(); }

void HookedSimUnit::set_clock(std::function<double()> clock) {
    std::lock_guard<std::mutex> lock(state_->clock_mutex);
    const double before = state_->base_now();
    state_->base_clock = std::move(clock);
    state_->offset = 0.0;
    state_->offset = before - state_->base_now();
}

void HookedSimUnit::set_request_hook(SimRequestHook hook) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->request_hook = std::move(hook);
}

std::vector<std::pair<double, std::vector<std::uint8_t>>> HookedSimUnit::request_log() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return {state_->log.begin(), state_->log.end()};
}

void HookedSimUnit::clear_request_log() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->log.clear();
}

std::uint16_t HookedSimUnit::hooked_register_value(int address) const {
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        for (const auto& r : state_->hooks.unreadable)
            if (overlaps(address, 1, r))
                throw std::out_of_range("register " + std::to_string(address) + " is unreadable (exception 02)");
        auto it = state_->hooks.word_overrides.find(address);
        if (it != state_->hooks.word_overrides.end()) return it->second;
    }
    return register_value(address);
}

// ---- Transport

std::vector<std::uint8_t> HookedSimUnit::handle(const std::vector<std::uint8_t>& request) {
    return patch(request, handle_frame(request), false);
}

std::vector<std::uint8_t> HookedSimUnit::patch(const std::vector<std::uint8_t>& request,
                                               std::vector<std::uint8_t> answer, bool count_stale) {
    if (answer.size() < 5 || request.size() < 6 || answer[0] != request[0] || answer[1] != request[1]) return answer;
    if (!check_crc(answer.data(), answer.size())) return answer;  // an injected fault: passed on as it is
    const int start = word_at(request, 2);
    const int count = word_at(request, 4);
    const std::size_t words = std::min<std::size_t>(static_cast<std::size_t>(count), answer[2] / 2u);
    if (answer.size() < 3 + 2 * words + 2) return answer;
    bool stale = false;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        for (const auto& r : state_->hooks.unreadable)
            if (overlaps(start, count, r))
                return build_frame(answer[0], {static_cast<std::uint8_t>(request[1] | 0x80), 0x02});
        if (count_stale && request[1] == kFcRead && start == reg::kMonitorBase && state_->stale_left > 0) {
            --state_->stale_left;
            ++state_->stale_served;
            stale = true;
        }
        for (std::size_t i = 0; i < words; ++i) {
            auto it = state_->hooks.word_overrides.find(start + static_cast<int>(i));
            if (it == state_->hooks.word_overrides.end()) continue;
            answer[3 + 2 * i] = static_cast<std::uint8_t>(it->second >> 8);
            answer[4 + 2 * i] = static_cast<std::uint8_t>(it->second & 0xFF);
        }
    }
    if (stale) {
        for (std::size_t i = 0; i < words; ++i) {
            const int address = start + static_cast<int>(i);
            if (address != reg::kDss1 && address != reg::kDsse) continue;
            std::uint16_t value = static_cast<std::uint16_t>((answer[3 + 2 * i] << 8) | answer[4 + 2 * i]);
            if (address == reg::kDss1)
                value = static_cast<std::uint16_t>(value | reg::kDss1Pend);  // not taken yet: still in position
            else
                value = static_cast<std::uint16_t>(value & ~(reg::kDsseMove | kDsseDst1));  // MOVE still clear
            answer[3 + 2 * i] = static_cast<std::uint8_t>(value >> 8);
            answer[4 + 2 * i] = static_cast<std::uint8_t>(value & 0xFF);
        }
    }
    answer.resize(answer.size() - 2);
    append_crc(answer);
    return answer;
}

void HookedSimUnit::write(const std::uint8_t* data, std::size_t size) {
    const std::vector<std::uint8_t> frame(data, data + size);
    {
        std::lock_guard<std::mutex> lock(state_->clock_mutex);
        if (state_->step) state_->stepped_now += *state_->step;
    }
    const double t = state_->now();
    SimRequestHook hook;
    bool hooked_reads = false;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->log.emplace_back(t, frame);
        while (state_->log.size() > kLogLimit) state_->log.pop_front();
        state_->pending.clear();  // a new request supersedes an answer not read
        hook = state_->request_hook;
        hooked_reads = !state_->hooks.word_overrides.empty() || !state_->hooks.unreadable.empty() ||
                       state_->stale_left > 0;
    }
    if (hook) {
        if (!is_open()) throw TransportError("simulated port is closed");
        SimulatedUnit::reset_input();  // TransportError while unplugged; an answer not read is superseded
        const SimHookAnswer answer = hook(frame);
        if (answer.kind != SimHookAnswer::Kind::Pass) {
            const std::optional<int> unit_rate = unit_baud_rate();
            const bool hears = powered() && (!unit_rate || baud_rate() == 0 || *unit_rate == baud_rate());
            if (!hears || answer.kind == SimHookAnswer::Kind::Drop) return;
            std::vector<std::uint8_t> bytes = answer.reply;
            if (answer.kind == SimHookAnswer::Kind::Exception)
                bytes = build_frame(slave(), {static_cast<std::uint8_t>((size > 1 ? data[1] : 0) | 0x80),
                                              static_cast<std::uint8_t>(answer.exception_code & 0xFF)});
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->pending = std::move(bytes);
            return;
        }
    }
    SimulatedUnit::write(data, size);
    const bool to_me = size >= 6 && data[0] == slave();
    if (to_me && data[1] == kFcWriteMany && word_at(frame, 2) == reg::kDirectMoveBase) {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->hooks.stale_status_polls > 0) state_->stale_left = state_->hooks.stale_status_polls;
    }
    if (!(hooked_reads && to_me && (data[1] == kFcRead || data[1] == kFcReadWrite))) return;
    std::vector<std::uint8_t> raw;  // the unit's answer to a read is complete at once: take it and patch it
    std::uint8_t chunk[260];
    for (std::size_t n; (n = SimulatedUnit::read(chunk, sizeof chunk)) > 0;) raw.insert(raw.end(), chunk, chunk + n);
    std::vector<std::uint8_t> patched = patch(frame, std::move(raw), true);
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->pending = std::move(patched);
}

std::size_t HookedSimUnit::read(std::uint8_t* buffer, std::size_t size) {
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->pending.empty()) {
            if (!is_open()) throw TransportError("simulated port is closed");
            const std::size_t n = std::min(size, state_->pending.size());
            std::copy(state_->pending.begin(), state_->pending.begin() + static_cast<std::ptrdiff_t>(n), buffer);
            state_->pending.erase(state_->pending.begin(), state_->pending.begin() + static_cast<std::ptrdiff_t>(n));
            return n;
        }
    }
    return SimulatedUnit::read(buffer, size);
}

void HookedSimUnit::reset_input() {
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->pending.clear();
    }
    SimulatedUnit::reset_input();
}

void HookedSimUnit::close() {
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->pending.clear();
    }
    SimulatedUnit::close();
}

void HookedSimUnit::reopen(std::optional<int> baud_rate) {
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->pending.clear();
    }
    SimulatedUnit::reopen(baud_rate);
}

}  // namespace iai_actuator_driver
