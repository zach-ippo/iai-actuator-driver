// A test transport that wraps another one (a SimulatedUnit, a SimLine) and lets a test answer or swallow chosen
// frames: a unit without FC05 coils (exception 01), axes whose no-access areas stay silent (RCON), IAI 0x48 frames
// translated into coils, the teach coils. Every frame written is logged. Header-only, for
// the axis tests (test_axis_strategy.cpp, test_axis_rcon.cpp).
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/transport.hpp"

namespace ec_test {

using Bytes = std::vector<std::uint8_t>;

class FilterTransport final : public iai_actuator_driver::Transport {
public:
    /// The frame written -> nullopt: pass it to the inner transport; a value: these bytes are the answer (empty =
    /// silence; the frame does not reach the inner transport).
    using Hook = std::function<std::optional<Bytes>(const Bytes&, iai_actuator_driver::Transport& inner)>;

    FilterTransport(std::shared_ptr<iai_actuator_driver::Transport> inner, Hook hook)
        : inner_(std::move(inner)), hook_(std::move(hook)) {}

    /// Every frame written, in order (also the ones the hook answered or swallowed).
    std::vector<Bytes> frames() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_;
    }
    void clear_frames() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
    }
    iai_actuator_driver::Transport& inner() { return *inner_; }

    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override {
        const Bytes frame(data, data + size);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frames_.push_back(frame);
        }
        std::optional<Bytes> answer = hook_ ? hook_(frame, *inner_) : std::nullopt;
        if (!answer) {
            inner_->write(data, size);
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.insert(pending_.end(), answer->begin(), answer->end());
    }
    std::size_t read(std::uint8_t* buffer, std::size_t size) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!pending_.empty()) {
                const std::size_t n = std::min(size, pending_.size());
                std::copy(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
                pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(n));
                return n;
            }
        }
        return inner_->read(buffer, size);
    }
    void reset_input() override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_.clear();
        }
        inner_->reset_input();
    }
    int baud_rate() const override { return inner_->baud_rate(); }
    void set_baud_rate(int rate) override { inner_->set_baud_rate(rate); }
    double read_timeout() const override { return inner_->read_timeout(); }
    void set_read_timeout(double seconds) override { inner_->set_read_timeout(seconds); }
    bool has_real_timeout() const override { return inner_->has_real_timeout(); }
    void send_break(double seconds) override { inner_->send_break(seconds); }
    void close() override { inner_->close(); }
    bool is_open() const override { return inner_->is_open(); }
    std::string name() const override { return inner_->name(); }

    /// The whole answer of `inner` to `frame` (the simulator answers at once on write).
    static Bytes exchange(iai_actuator_driver::Transport& inner, const Bytes& frame) {
        inner.write(frame.data(), frame.size());
        Bytes out(512);
        const std::size_t n = inner.read(out.data(), out.size());
        out.resize(n);
        return out;
    }

    /// An exception response `slave` (function | 0x80) `code`, with its CRC.
    static Bytes exception(int slave, int function, int code) {
        return iai_actuator_driver::build_frame(slave, {static_cast<std::uint8_t>(function | 0x80),
                                                        static_cast<std::uint8_t>(code)});
    }

private:
    std::shared_ptr<iai_actuator_driver::Transport> inner_;
    Hook hook_;
    mutable std::mutex mutex_;
    std::vector<Bytes> frames_;
    Bytes pending_;
};

}  // namespace ec_test
