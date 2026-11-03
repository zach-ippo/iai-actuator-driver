// A SimulatedUnit with the test hooks of the Python edition's SimulatedEC (ABI 0.3 iad_sim_*): word overrides,
// unreadable register ranges, stale status polls after a move command, a stepped clock and a request hook. It is a
// SimulatedUnit (usable on a SimLine too); with no hook set it behaves exactly like one. Implemented by the sim unit
// (src/sim_hooks.cpp).
//
// * word_overrides {address: value}: an FC03 / FC17 read serves `value` for these registers instead of the modelled
//   value (a unit whose register reads otherwise, e.g. AKID = 1 for a rotary unit).
// * unreadable [(start, count)]: an FC03 / FC17 read touching these registers answers exception 02 (a unit without
//   them).
// * stale_status_polls N: after each FC10 move command at 0x9900 to this unit, the next N FC03 reads at 0x9000 still
//   show PEND set and MOVE (and DSSE b4) clear: the unit has not taken the command yet (a modelled worst case for the
//   completion rules, as in the Python edition).
// * step_s: the unit runs on its own clock, which starts at 1000.0 s and advances step_s at every request written to
//   it, so each poll sees the axis exactly step_s later whatever the host's speed (options.clock is then not used).
// * request hook: sees every frame written to the unit (on a line: the frames to other slaves too) and answers it as
//   usual, not at all, with given bytes or with a Modbus exception (the Python edition's on_request); handle_frame()
//   is the unit's own answer, so a hook can act and then drop or replace the reply. The hook runs after the line
//   checks it can see (closed, unplugged, powered off, a baud rate the unit does not listen at), without the unit's
//   lock held. A hook that throws fails the write with that exception (0.6: the Python edition's on_request raising;
//   the C ABI's IAD_SIM_FAIL throws TransportError), and the frame reaches the unit no further.
// The patched answers keep the base unit's faults: an answer with a bad CRC (inject_fault) is passed on unchanged.
// Evidence: test tooling; the behaviour each hook models is the Python edition's (UNVERIFIED on hardware).
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "iai_actuator_driver/sim.hpp"

namespace iai_actuator_driver {

/// What a request hook decides for one frame.
struct SimHookAnswer {
    enum class Kind {
        Pass,       ///< the unit answers as usual
        Drop,       ///< no answer (the frame does not reach the unit)
        Reply,      ///< answer with `reply` (the frame does not reach the unit)
        Exception,  ///< answer the Modbus exception `exception_code` (the frame does not reach the unit)
    };
    Kind kind = Kind::Pass;
    std::vector<std::uint8_t> reply;
    int exception_code = 0;
};

/// The frame written -> what to answer.
using SimRequestHook = std::function<SimHookAnswer(const std::vector<std::uint8_t>& frame)>;

/// Hooks of a HookedSimUnit.
struct SimHookOptions {
    std::map<int, std::uint16_t> word_overrides;
    std::vector<std::pair<int, int>> unreadable;  ///< (start, count)
    int stale_status_polls = 0;
    std::optional<double> step_s;                 ///< >= 0 (nullopt: options.clock / now_s())
};

class HookedSimUnit : public SimulatedUnit {
public:
    explicit HookedSimUnit(const SimulatedUnitOptions& options = {}, const SimHookOptions& hooks = {});
    ~HookedSimUnit() override;

    // ---- hooks (thread-safe; take effect at the next request)
    /// Serve `value` for `address` (nullopt: model it again).
    void set_word_override(int address, std::optional<std::uint16_t> value);
    std::map<int, std::uint16_t> word_overrides() const;
    /// Mark [start, start + count) unreadable (or readable again with `unreadable` false: every range overlapping
    /// it is removed). std::invalid_argument for a count < 1 or a range outside 0..0xFFFF.
    void set_unreadable(int start, int count, bool unreadable = true);
    std::vector<std::pair<int, int>> unreadable() const;
    /// 0 = off. std::invalid_argument when negative.
    void set_stale_status_polls(int polls);
    int stale_status_polls() const;
    /// Status reads answered stale so far.
    int stale_polls_served() const;
    /// The stepped clock (nullopt: off, the unit's clock is options.clock / now_s() again). std::invalid_argument
    /// when negative or not finite.
    void set_step_s(std::optional<double> step_s);
    std::optional<double> step_s() const;
    /// The unit's clock now (the stepped clock when on).
    double clock_now() const;
    /// (0.5) Run the unit on `clock` (monotonic seconds; empty: now_s(), the library's clock again) from now on. The
    /// unit's time goes on from where it is: the new clock is offset so nothing jumps. The stepped clock, when on,
    /// still wins.
    void set_clock(std::function<double()> clock);
    void set_request_hook(SimRequestHook hook);
    /// (time on the unit's clock, frame) of every frame written to the unit, oldest first (the last 2000), also the
    /// ones the request hook answered or dropped.
    std::vector<std::pair<double, std::vector<std::uint8_t>>> request_log() const;
    void clear_request_log();
    /// register_value() with the overrides and the unreadable ranges applied (std::out_of_range where a read is
    /// refused).
    std::uint16_t hooked_register_value(int address) const;
    /// The unit's own answer to `request` (SimulatedUnit::handle_frame: it acts on it) with the word overrides and the
    /// unreadable ranges applied (not the stale polls): the Python edition's SimulatedEC.handle, for request hooks.
    std::vector<std::uint8_t> handle(const std::vector<std::uint8_t>& request);

    // ---- Transport (the hooks)
    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    void close() override;
    void reopen(std::optional<int> baud_rate = std::nullopt) override;

private:
    struct State;
    HookedSimUnit(SimulatedUnitOptions options, const SimHookOptions& hooks, std::shared_ptr<State> state);
    std::vector<std::uint8_t> patch(const std::vector<std::uint8_t>& request, std::vector<std::uint8_t> answer,
                                    bool count_stale);
    std::shared_ptr<State> state_;
};

}  // namespace iai_actuator_driver
