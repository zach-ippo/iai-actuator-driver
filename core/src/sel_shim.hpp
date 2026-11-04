// Private to the core (internal: not installed, not part of the API yet): StreamShim, a Transport in front of another
// that delivers bytes as a byte stream does (a TCP socket, a USB adapter's bursts). Reads hand out what the inner
// transport has in segments of the sizes in `read_segments`, taken in turn: a small size splits a reply over several
// reads, a large one coalesces it with whatever else is waiting (a late reply, the next line), and 0 returns nothing
// that time though bytes wait. Writes reach the inner transport in pieces of `write_segments`, in turn. It tests the
// SEL link's LF framing before any TCP transport exists. For tests; it never opens a port.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace sel {

class StreamShim : public Transport {
public:
    /// `read_segments` empty = everything waiting at once; `write_segments` empty = each write whole. A write segment
    /// of 0 is std::invalid_argument.
    StreamShim(std::shared_ptr<Transport> inner, std::vector<std::size_t> read_segments,
               std::vector<std::size_t> write_segments = {});
    ~StreamShim() override;
    StreamShim(const StreamShim&) = delete;
    StreamShim& operator=(const StreamShim&) = delete;

    Transport& inner() const noexcept { return *inner_; }
    /// Calls of read() and write() pieces passed on so far.
    int reads() const noexcept { return reads_; }
    int write_pieces() const noexcept { return write_pieces_; }

    // ---- Transport (everything not about segments goes to the inner transport)
    using Transport::write;
    void write(const std::uint8_t* data, std::size_t size) override;
    std::size_t read(std::uint8_t* buffer, std::size_t size) override;
    void reset_input() override;
    int baud_rate() const override { return inner_->baud_rate(); }
    void set_baud_rate(int rate) override { inner_->set_baud_rate(rate); }
    double read_timeout() const override { return inner_->read_timeout(); }
    void set_read_timeout(double seconds) override { inner_->set_read_timeout(seconds); }
    bool has_real_timeout() const override { return inner_->has_real_timeout(); }
    void send_break(double seconds) override { inner_->send_break(seconds); }
    void close() override { inner_->close(); }
    bool is_open() const override { return inner_->is_open(); }
    std::string name() const override { return inner_->name(); }
    std::optional<bool> present() const override { return inner_->present(); }
    bool can_reopen() const override { return inner_->can_reopen(); }
    void reopen(std::optional<int> baud_rate = std::nullopt) override;

private:
    std::shared_ptr<Transport> inner_;
    std::vector<std::size_t> read_segments_;
    std::vector<std::size_t> write_segments_;
    std::size_t next_read_ = 0;
    std::size_t next_write_ = 0;
    std::string buffered_;
    int reads_ = 0;
    int write_pieces_ = 0;
};

}  // namespace sel
}  // namespace iai_actuator_driver
