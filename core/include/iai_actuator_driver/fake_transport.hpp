// In-memory Transport for tests (port of C# FakeTransport.cs): records every frame written and answers it from a
// script of responses (enqueue; an empty optional = no answer) or, when the script is empty, from a responder
// callback. Reads never block: what is buffered is returned at once, so a missing answer times out immediately.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {

class FakeTransport final : public Transport {
public:
    /// Request frame in, response bytes out (nullopt = silence).
    using Responder = std::function<std::optional<std::vector<std::uint8_t>>(const std::vector<std::uint8_t>&)>;

    /// `baud_rate` 0 = no baud rate (like the simulator); otherwise the starting rate, settable in place.
    explicit FakeTransport(Responder responder = nullptr, int baud_rate = 0, std::string name = "FAKE");
    ~FakeTransport() override;

    /// Queue responses for the next writes, in order (nullopt = no answer to that write).
    void enqueue(std::optional<std::vector<std::uint8_t>> response);
    /// Make bytes readable now (unsolicited / partial data).
    void inject(const std::vector<std::uint8_t>& bytes);
    void set_responder(Responder responder);
    /// Only answer while baud_rate() equals this (a unit at another rate is silent); nullopt = always.
    void set_answer_at_baud_rate(std::optional<int> rate);

    /// Every frame written, in order.
    std::vector<std::vector<std::uint8_t>> writes() const;
    /// now_s() of every write.
    std::vector<double> write_times() const;
    /// Every rate set with set_baud_rate, in order.
    std::vector<int> baud_rates_set() const;
    /// Every send_break duration.
    std::vector<double> breaks() const;
    void clear_writes();

    /// Make every later call throw TransportError (a pulled USB adapter) until reopen().
    void fail();
    /// Usable again (after fail() or close()); `baud_rate` sets the rate when the fake has one.
    void reopen(std::optional<int> baud_rate = std::nullopt) override;
    bool can_reopen() const override { return true; }

    // Transport
    using Transport::write;  // keep the write(vector) convenience overload visible
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override;
    void set_baud_rate(int rate) override;
    double read_timeout() const override;
    void set_read_timeout(double seconds) override;
    void send_break(double seconds) override;
    void close() override;
    bool is_open() const override;
    std::string name() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iai_actuator_driver
