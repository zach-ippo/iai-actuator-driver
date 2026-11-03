// In-memory Transport for tests (port of C# FakeTransport.cs): records every frame written and answers from a script
// of responses or a responder callback. Reads never block.
#include "iai_actuator_driver/fake_transport.hpp"

#include <algorithm>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {

struct FakeTransport::Impl {
    mutable std::mutex mutex;
    Responder responder;
    std::deque<std::optional<std::vector<std::uint8_t>>> script;
    std::deque<std::uint8_t> rx;
    std::vector<std::vector<std::uint8_t>> writes;
    std::vector<double> write_times;
    std::vector<int> baud_rates;
    std::vector<double> breaks;
    std::optional<int> answer_at;
    int baud_rate = 0;
    double read_timeout = kDefaultReadTimeoutS;
    bool closed = false;
    bool failed = false;
    std::string name;

    void check_usable() const {
        if (failed) throw TransportError(name + ": the port vanished (simulated USB adapter pull)");
        if (closed) throw TransportError(name + ": fake transport is closed");
    }
};

FakeTransport::FakeTransport(Responder responder, int baud_rate, std::string name) : impl_(new Impl) {
    impl_->responder = std::move(responder);
    impl_->baud_rate = baud_rate > 0 ? baud_rate : 0;
    impl_->name = std::move(name);
}

FakeTransport::~FakeTransport() = default;

void FakeTransport::enqueue(std::optional<std::vector<std::uint8_t>> response) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->script.push_back(std::move(response));
}

void FakeTransport::inject(const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->rx.insert(impl_->rx.end(), bytes.begin(), bytes.end());
}

void FakeTransport::set_responder(Responder responder) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->responder = std::move(responder);
}

void FakeTransport::set_answer_at_baud_rate(std::optional<int> rate) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->answer_at = rate;
}

std::vector<std::vector<std::uint8_t>> FakeTransport::writes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->writes;
}

std::vector<double> FakeTransport::write_times() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->write_times;
}

std::vector<int> FakeTransport::baud_rates_set() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->baud_rates;
}

std::vector<double> FakeTransport::breaks() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->breaks;
}

void FakeTransport::clear_writes() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->writes.clear();
    impl_->write_times.clear();
}

void FakeTransport::fail() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->failed = true;
    impl_->rx.clear();
}

void FakeTransport::reopen(std::optional<int> baud_rate) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (baud_rate && impl_->baud_rate > 0) impl_->baud_rate = *baud_rate;
    impl_->failed = false;
    impl_->closed = false;
    impl_->rx.clear();
}

void FakeTransport::write(const std::uint8_t* data, std::size_t size) {
    std::vector<std::uint8_t> frame(data, data + size);
    Responder responder;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->check_usable();
        impl_->writes.push_back(frame);
        impl_->write_times.push_back(now_s());
        if (impl_->answer_at && *impl_->answer_at != impl_->baud_rate) return;  // garbled at the wrong rate: silence
        if (!impl_->script.empty()) {
            std::optional<std::vector<std::uint8_t>> response = std::move(impl_->script.front());
            impl_->script.pop_front();
            if (response) impl_->rx.insert(impl_->rx.end(), response->begin(), response->end());
            return;
        }
        responder = impl_->responder;
    }
    if (!responder) return;
    std::optional<std::vector<std::uint8_t>> answer = responder(frame);  // outside the lock: it may inject()
    if (answer) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->rx.insert(impl_->rx.end(), answer->begin(), answer->end());
    }
}

std::size_t FakeTransport::read(std::uint8_t* buffer, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->check_usable();
    const std::size_t n = std::min(size, impl_->rx.size());
    std::copy(impl_->rx.begin(), impl_->rx.begin() + static_cast<std::ptrdiff_t>(n), buffer);
    impl_->rx.erase(impl_->rx.begin(), impl_->rx.begin() + static_cast<std::ptrdiff_t>(n));
    return n;
}

void FakeTransport::reset_input() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->check_usable();
    impl_->rx.clear();
}

int FakeTransport::baud_rate() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->baud_rate;
}

void FakeTransport::set_baud_rate(int rate) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->baud_rate == 0) throw std::logic_error(impl_->name + ": this fake transport has no baud rate");
    if (rate <= 0) throw std::invalid_argument("baud rate must be > 0");
    impl_->check_usable();
    impl_->baud_rate = rate;
    impl_->baud_rates.push_back(rate);
    impl_->rx.clear();
}

double FakeTransport::read_timeout() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->read_timeout;
}

void FakeTransport::set_read_timeout(double seconds) {
    if (!(seconds >= 0.0)) throw std::invalid_argument("read timeout must be >= 0");
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->read_timeout = seconds;
}

void FakeTransport::send_break(double seconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->check_usable();
    impl_->breaks.push_back(seconds);
    impl_->rx.clear();
}

void FakeTransport::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->closed = true;
}

bool FakeTransport::is_open() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return !impl_->closed && !impl_->failed;
}

std::string FakeTransport::name() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->name;
}

}  // namespace iai_actuator_driver
