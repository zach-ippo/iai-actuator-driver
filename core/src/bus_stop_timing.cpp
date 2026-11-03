// The stop latency line (bus_stop_timing.hpp).
#include "bus_stop_timing.hpp"

#include <utility>

#include "iai_actuator_driver/log.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace detail {

namespace {

thread_local StopLinesLater* lines_later = nullptr;  // the innermost StopLinesLater scope of the thread
thread_local StopTiming* timings = nullptr;          // the innermost StopTiming scope of the thread

double ms(double seconds) { return seconds * 1000.0; }

}  // namespace

StopLinesLater::StopLinesLater() : previous_(lines_later) { lines_later = this; }

StopLinesLater::~StopLinesLater() {
    lines_later = previous_;
    if (previous_) {  // an outer scope logs them
        try {
            for (auto& l : lines_) previous_->lines_.push_back(std::move(l));
        } catch (...) {
        }
        return;
    }
    for (const auto& l : lines_) log_info(l.first, l.second);
}

StopTiming::StopTiming(const ModbusRtu& bus, int slave, const void* owner, const char* what, bool holds,
                       const char* source)
    : wire_(bus),
      bus_(bus),
      slave_(slave),
      owner_(owner),
      what_(what),
      holds_(holds),
      source_(source),
      call_s_(now_s()),
      previous_(timings) {
    timings = this;
}

StopTiming::~StopTiming() {
    timings = previous_;
    try {
        std::string message = line();
        if (lines_later)
            lines_later->lines_.emplace_back(source_, std::move(message));
        else
            log_info(source_, message);
    } catch (...) {  // a stop never fails for its log line
    }
}

void StopTiming::standstill(const void* owner) {
    double now = 0.0;
    bool taken = false;
    for (StopTiming* t = timings; t; t = t->previous_) {
        if (t->owner_ != owner || !t->holds_ || t->still_s_) continue;
        if (!taken) {
            now = now_s();
            taken = true;
        }
        t->still_s_ = now;
    }
}

std::string StopTiming::line() const {
    const std::string who = slave_ > 0 ? util::format("%s slave %d", bus_.name().c_str(), slave_) : bus_.name();
    const std::optional<double> wire = wire_.wire_s();
    if (!wire)
        return util::format("%s: %s not sent (it ended %.0f ms after the call)", who.c_str(), what_,
                            ms(now_s() - call_s_));
    std::string out =
        util::format("%s: %s on the wire %.0f ms after the call", who.c_str(), what_, ms(*wire - call_s_));
    if (holds_) {
        if (still_s_)
            out += util::format(", standstill confirmed %.0f ms later", ms(*still_s_ - *wire));
        else
            out += ", standstill not confirmed";
    }
    return out;
}

}  // namespace detail
}  // namespace iai_actuator_driver
