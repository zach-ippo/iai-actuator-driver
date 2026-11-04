// StreamShim: a byte-stream view of another Transport (sel_shim.hpp).
#include "sel_shim.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace iai_actuator_driver {
namespace sel {

StreamShim::StreamShim(std::shared_ptr<Transport> inner, std::vector<std::size_t> read_segments,
                       std::vector<std::size_t> write_segments)
    : inner_(std::move(inner)), read_segments_(std::move(read_segments)), write_segments_(std::move(write_segments)) {
    if (!inner_) throw std::invalid_argument("StreamShim needs a transport");
    for (std::size_t size : write_segments_) {
        if (size == 0) throw std::invalid_argument("StreamShim: a write segment of 0 bytes");
    }
}

StreamShim::~StreamShim() = default;

void StreamShim::write(const std::uint8_t* data, std::size_t size) {
    std::size_t done = 0;
    while (done < size) {
        std::size_t piece = size - done;
        if (!write_segments_.empty()) {
            piece = std::min(piece, write_segments_[next_write_]);
            next_write_ = (next_write_ + 1) % write_segments_.size();
        }
        inner_->write(data + done, piece);
        ++write_pieces_;
        done += piece;
    }
}

std::size_t StreamShim::read(std::uint8_t* buffer, std::size_t size) {
    ++reads_;
    std::uint8_t chunk[512];
    for (;;) {  // everything the inner transport has now
        const std::size_t n = inner_->read(chunk, sizeof chunk);
        if (n == 0) break;
        buffered_.append(reinterpret_cast<const char*>(chunk), n);
    }
    std::size_t take = std::min(size, buffered_.size());
    if (!read_segments_.empty()) {
        take = std::min(take, read_segments_[next_read_]);
        next_read_ = (next_read_ + 1) % read_segments_.size();
    }
    std::copy_n(buffered_.data(), take, buffer);
    buffered_.erase(0, take);
    return take;
}

void StreamShim::reset_input() {
    buffered_.clear();
    inner_->reset_input();
}

void StreamShim::reopen(std::optional<int> baud_rate) {
    buffered_.clear();
    inner_->reopen(baud_rate);
}

}  // namespace sel
}  // namespace iai_actuator_driver
