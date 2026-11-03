// Simulated gateway lines (RCON, ROBONET through mode, ROBONET gateway mode): port of the Python edition's
// sim_line.SimGatewayLine and its gateways. DOC only (ME0413, ME0208), UNVERIFIED on hardware. Unofficial: not an IAI
// product. Header-only dependencies only (sim unit).
#include "iai_actuator_driver/sim_gateway.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/crc.hpp"
#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"

namespace iai_actuator_driver {

namespace {

using Bytes = std::vector<std::uint8_t>;

const std::vector<int> kRconFunctions = {0x01, 0x03, 0x04, 0x06, 0x10, 0x17};  // ME0413 p.3-1
const std::vector<int> kRoboNetFunctions = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                            0x07, 0x0F, 0x10, 0x11, 0x17};  // ME0208 p.26
constexpr std::size_t kRoboNetBufferBytes = 160;  // ME0208 p.26
constexpr int kRoboNetNoAnswerBelow = 0x0D00;     // directory and class items: none on RCM-era firmware (modelled)
constexpr int kSerialBase = 0x9400;
constexpr int kSerialWords = 8;
constexpr int kMonitorBase = 0x2900;
constexpr int kMonitorStride = 15;
constexpr int kGwStatus = 0x3700;
constexpr int kGwStatusWords = 0x0B;
constexpr int kDrg1 = 0x0D00;
constexpr std::size_t kUnitReadBytes = 4096;  // an answer is read whole, as the Python edition's unit.read(4096)

/// PNOW, PCMD (signed) and INP (unsigned): the 32-bit position fields scaled on a 0.001 mm RCON unit.
struct PositionField {
    int address;
    bool is_signed;
};
const PositionField kPositionFields[] = {{0x9000, true}, {0x9900, true}, {0x9902, false}};

Bytes exception_frame(int slave, int function, int code) {
    return build_frame(slave, {static_cast<std::uint8_t>(function | 0x80), static_cast<std::uint8_t>(code)});
}

int be16(const Bytes& b, std::size_t at) { return (b[at] << 8) | b[at + 1]; }

std::vector<std::uint16_t> words_of(const Bytes& data, std::size_t from, std::size_t count_bytes) {
    std::vector<std::uint16_t> out;
    for (std::size_t i = 0; i + 1 < count_bytes && from + i + 1 < data.size(); i += 2)
        out.push_back(static_cast<std::uint16_t>((data[from + i] << 8) | data[from + i + 1]));
    return out;
}

void push_words(Bytes& out, const std::vector<std::uint16_t>& words) {
    for (std::uint16_t w : words) {
        out.push_back(static_cast<std::uint8_t>(w >> 8));
        out.push_back(static_cast<std::uint8_t>(w & 0xFF));
    }
}

Bytes read_reply(int slave, int function, const std::vector<std::uint16_t>& words) {
    Bytes pdu{static_cast<std::uint8_t>(function), static_cast<std::uint8_t>(2 * words.size())};
    push_words(pdu, words);
    return build_frame(slave, pdu);
}

/// Convert the 32-bit position fields inside `words` (read / written from `start`) between the wire's 0.001 mm
/// (`factor` 10) and the simulated unit's 0.01 mm.
std::vector<std::uint16_t> scale_words(int start, std::vector<std::uint16_t> words, int factor, bool to_unit) {
    for (const PositionField& field : kPositionFields) {
        const long i = field.address - start;
        if (i < 0 || i + 1 >= static_cast<long>(words.size())) continue;
        long long value = (static_cast<long long>(words[i]) << 16) | words[i + 1];
        if (field.is_signed && (value & 0x80000000LL)) value -= (1LL << 32);
        value = to_unit ? std::llround(static_cast<double>(value) / factor) : value * factor;
        const std::uint32_t raw = static_cast<std::uint32_t>(value & 0xFFFFFFFFLL);
        words[i] = static_cast<std::uint16_t>(raw >> 16);
        words[i + 1] = static_cast<std::uint16_t>(raw & 0xFFFF);
    }
    return words;
}

bool contains(const std::vector<int>& list, int value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

/// The directory, identification, limit, parameter and table areas (below the DRG1 registers at 0x0D00) and the 0x8000
/// system area: "system, no access" on an RCON axis (ME0413 p.5-10).
bool rcon_no_access(int address) { return address < kDrg1 || (address >= 0x8000 && address < 0x9000); }

/// Whatever the unit has to answer now (its read(): empty when nothing, or not yet).
Bytes read_unit(SimulatedUnit& unit) {
    std::vector<std::uint8_t> buffer(kUnitReadBytes);
    buffer.resize(unit.read(buffer.data(), buffer.size()));
    return buffer;
}

}  // namespace

// ------------------------------------------------------------------------------------------ RCON gateway

SimulatedRconGateway::SimulatedRconGateway(int slave, bool manu, int alarm_code, bool emergency_stop,
                                           std::vector<int> linked_only)
    : slave_(slave), manu_(manu), alarm_code_(alarm_code), emergency_stop_(emergency_stop),
      linked_only_(std::move(linked_only)), started_(now_s()) {}

int SimulatedRconGateway::slave() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slave_;
}
void SimulatedRconGateway::set_slave(int slave) {
    std::lock_guard<std::mutex> lock(mutex_);
    slave_ = slave;
}
bool SimulatedRconGateway::manu() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return manu_;
}
void SimulatedRconGateway::set_manu(bool manu) {
    std::lock_guard<std::mutex> lock(mutex_);
    manu_ = manu;
}
int SimulatedRconGateway::alarm_code() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return alarm_code_;
}
void SimulatedRconGateway::set_alarm_code(int code) {
    std::lock_guard<std::mutex> lock(mutex_);
    alarm_code_ = code;
}
bool SimulatedRconGateway::emergency_stop() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return emergency_stop_;
}
void SimulatedRconGateway::set_emergency_stop(bool on) {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stop_ = on;
}
std::vector<int> SimulatedRconGateway::linked_only() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return linked_only_;
}
void SimulatedRconGateway::set_linked_only(std::vector<int> slaves) {
    std::lock_guard<std::mutex> lock(mutex_);
    linked_only_ = std::move(slaves);
}

std::vector<int> SimulatedRconGateway::linked_slaves() const {
    std::set<int> all;
    if (line_) {
        for (int s : line_->slaves()) all.insert(s);
    }
    for (int s : linked_only()) all.insert(s);
    return std::vector<int>(all.begin(), all.end());
}

std::vector<std::uint16_t> SimulatedRconGateway::status_block() const {
    unsigned nlnk = 0xFFFF;
    for (int s : linked_slaves())
        if (s >= 1 && s <= 16) nlnk &= ~(1u << (s - 1));
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint32_t stim = static_cast<std::uint32_t>(static_cast<long long>((now_s() - started_) * 1000.0));
    const std::uint16_t gwsr = manu_ ? 0x8000 : 0;
    const std::uint16_t gsr2 = static_cast<std::uint16_t>((manu_ ? 0x0004 : 0) | (emergency_stop_ ? 0x0002 : 0));
    const std::uint32_t alm = static_cast<std::uint32_t>(alarm_code_);
    return {gwsr,
            gsr2,
            0,
            static_cast<std::uint16_t>(nlnk & 0xFFFF),
            static_cast<std::uint16_t>(stim >> 16),
            static_cast<std::uint16_t>(stim & 0xFFFF),
            0,
            0,
            static_cast<std::uint16_t>(alm >> 16),
            static_cast<std::uint16_t>(alm & 0xFFFF),
            0xFFFE};
}

std::vector<std::vector<std::uint8_t>> SimulatedRconGateway::requests() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
}

std::vector<std::uint16_t> SimulatedRconGateway::monitor(int axis) const {
    const std::vector<std::uint16_t> none(kMonitorStride, 0);
    if (!line_) return none;
    std::shared_ptr<SimulatedUnit> unit;
    try {
        unit = line_->unit(axis + 1);
    } catch (const std::out_of_range&) {
        return none;
    }
    // through the unit's own port, as any request (its log, hooks and silence apply; a closed unit throws)
    const Bytes request = build_frame(unit->slave(), {0x03, 0x90, 0x00, 0x00, 0x14});
    unit->reset_input();
    unit->write(request.data(), request.size());
    const Bytes reply = read_unit(*unit);
    if (reply.size() < 45 || !check_crc(reply) || reply[1] != 0x03) return none;
    std::vector<std::uint16_t> w = words_of(reply, 3, reply.size() - 5);
    const int factor = line_->position_factor(unit->slave());
    if (factor != 1) w = scale_words(0x9000, w, factor, false);
    // PNOW(2) VNOW(2) CNOW(2) ALMC DSS1 DSSE SIPM OLLV DEVI(2) STAT(2)
    return {w[0], w[1], w[10], w[11], w[12], w[13], w[2], w[5], w[7], w[18], 0, w[14], w[15], w[8], w[9]};
}

std::vector<std::uint8_t> SimulatedRconGateway::handle(const std::vector<std::uint8_t>& frame) {
    int slave = 0;
    double started = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.push_back(frame);
        slave = slave_;
        started = started_;
    }
    const int function = frame[1];
    if (function != 0x03 && function != 0x04) return exception_frame(slave, function, 0x01);
    if (frame.size() < 8) return exception_frame(slave, function, 0x03);
    const int start = be16(frame, 2);
    const int count = be16(frame, 4);
    if (count < 1 || count > 125) return exception_frame(slave, function, 0x03);
    const std::vector<std::uint16_t> status = status_block();
    std::map<int, std::vector<std::uint16_t>> monitors;
    std::vector<std::uint16_t> words;
    for (int address = start; address < start + count; ++address) {
        if (address >= kGwStatus && address < kGwStatus + kGwStatusWords) {
            words.push_back(status[static_cast<std::size_t>(address - kGwStatus)]);
        } else if (address >= kMonitorBase && address < kMonitorBase + 16 * kMonitorStride) {
            const int axis = (address - kMonitorBase) / kMonitorStride;
            const int offset = (address - kMonitorBase) % kMonitorStride;
            if (!monitors.count(axis)) monitors[axis] = monitor(axis);
            words.push_back(monitors[axis][static_cast<std::size_t>(offset)]);
        } else if (address == 0x3300 || address == 0x3301) {
            const std::uint32_t timc = static_cast<std::uint32_t>(static_cast<long long>(now_s() - started));
            words.push_back(static_cast<std::uint16_t>(address == 0x3300 ? timc >> 16 : timc & 0xFFFF));
        } else {
            return exception_frame(slave, function, 0x02);
        }
    }
    return read_reply(slave, function, words);
}

// ------------------------------------------------------------------------------------------ ROBONET gateway

SimulatedRoboNetGateway::SimulatedRoboNetGateway(int slave, bool running) : slave_(slave), running_(running) {}

int SimulatedRoboNetGateway::slave() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slave_;
}
void SimulatedRoboNetGateway::set_slave(int slave) {
    std::lock_guard<std::mutex> lock(mutex_);
    slave_ = slave;
}
bool SimulatedRoboNetGateway::running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}
void SimulatedRoboNetGateway::set_running(bool running) {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = running;
}
std::vector<std::vector<std::uint8_t>> SimulatedRoboNetGateway::requests() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
}

std::vector<std::uint8_t> SimulatedRoboNetGateway::handle(const std::vector<std::uint8_t>& frame) {
    int slave = 0;
    bool running = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.push_back(frame);
        slave = slave_;
        running = running_;
    }
    const int function = frame[1];
    if (function == 0x06 || function == 0x10) return {};
    if (function != 0x03) return exception_frame(slave, function, 0x01);
    if (frame.size() < 8) return exception_frame(slave, function, 0x03);
    const int start = be16(frame, 2);
    const int count = be16(frame, 4);
    if (start < 0xF700 || start + count > 0xF708 || count < 1) return exception_frame(slave, function, 0x02);
    std::vector<int> axes;
    if (line_)
        for (int s : line_->slaves())
            if (s >= 1 && s <= 16) axes.push_back(s - 1);
    unsigned link = 0;
    for (int a : axes) link |= 1u << a;
    const std::uint16_t status0 = static_cast<std::uint16_t>((running ? 0x8000 : 0) | (axes.size() & 0x1F));
    const std::vector<std::uint16_t> area = {status0, static_cast<std::uint16_t>(link), 0, 0, 0, 0, 0, 0};
    const std::vector<std::uint16_t> words(area.begin() + (start - 0xF700), area.begin() + (start - 0xF700 + count));
    return read_reply(slave, 0x03, words);
}

// ------------------------------------------------------------------------------------------ the line

SimGatewayLine::SimGatewayLine(std::vector<std::shared_ptr<SimulatedUnit>> units, SimGatewayOptions options)
    : SimLine(std::move(units), options.name ? *options.name : std::string("GW")), kind_(options.kind),
      unit_mm_(options.position_unit_mm), units_mm_(options.position_units), serials_(options.serials),
      silent_no_access_(options.silent_no_access) {
    if (kind_ == LineKind::Direct)
        throw std::invalid_argument("kind must be one of rcon, robonet, robonet-gateway, got direct");
    auto check_unit = [](double value) {
        if (value != 0.01 && value != 0.001)
            throw std::invalid_argument("position_unit_mm must be 0.01 or 0.001, got " + std::to_string(value));
    };
    check_unit(unit_mm_);
    for (const auto& kv : units_mm_) check_unit(kv.second);
    if (kind_ == LineKind::Rcon) {
        rcon_ = std::make_shared<SimulatedRconGateway>(0xD0, options.manu, options.gateway_alarm,
                                                       options.emergency_stop, options.linked_only);
        rcon_->line_ = this;
    } else if (kind_ == LineKind::RobonetGateway) {
        robonet_ = std::make_shared<SimulatedRoboNetGateway>();
        robonet_->line_ = this;
    }
}

SimGatewayLine::~SimGatewayLine() {
    if (rcon_) rcon_->line_ = nullptr;
    if (robonet_) robonet_->line_ = nullptr;
}

LineKind SimGatewayLine::kind() const { return kind_; }
std::shared_ptr<SimulatedRconGateway> SimGatewayLine::rcon_gateway() const { return rcon_; }
std::shared_ptr<SimulatedRoboNetGateway> SimGatewayLine::robonet_gateway() const { return robonet_; }
std::vector<int> SimGatewayLine::function_codes() const {
    return kind_ == LineKind::Rcon ? kRconFunctions : kRoboNetFunctions;
}

double SimGatewayLine::position_unit(int slave) const {
    const auto it = units_mm_.find(slave);
    return it != units_mm_.end() ? it->second : unit_mm_;
}

int SimGatewayLine::position_factor(int slave) const {
    return kind_ == LineKind::Rcon && position_unit(slave) == 0.001 ? 10 : 1;
}

std::string SimGatewayLine::serial_code(int slave) const {
    const auto it = serials_.find(slave);
    if (it != serials_.end()) return it->second;
    char buf[32];
    std::snprintf(buf, sizeof buf, "SIMRCON-%02d", slave - 1);
    return buf;
}

std::vector<std::vector<std::uint8_t>> SimGatewayLine::dropped_broadcasts() const {
    std::lock_guard<std::mutex> lock(gw_mutex_);
    return dropped_;
}

std::vector<std::vector<std::uint8_t>> SimGatewayLine::rejected() const {
    std::lock_guard<std::mutex> lock(gw_mutex_);
    return rejected_;
}

bool SimGatewayLine::silent_no_access() const {
    std::lock_guard<std::mutex> lock(gw_mutex_);
    return silent_no_access_;
}

void SimGatewayLine::set_silent_no_access(bool silent) {
    std::lock_guard<std::mutex> lock(gw_mutex_);
    silent_no_access_ = silent;
}

void SimGatewayLine::emit(const std::vector<std::uint8_t>& reply) {
    if (reply.empty()) return;
    std::lock_guard<std::mutex> lock(gw_mutex_);
    out_.insert(out_.end(), reply.begin(), reply.end());
}

void SimGatewayLine::write(const std::uint8_t* data, std::size_t size) {
    record_request(data, size);  // TransportError when closed / unplugged
    const Bytes frame(data, data + size);
    {
        std::lock_guard<std::mutex> lock(gw_mutex_);
        out_.clear();
        pending_unit_.reset();
        pending_request_.clear();
    }
    if (size < 4 || !check_crc(frame)) return;  // garbage: every station stays silent
    const int address = frame[0];
    const int function = frame[1];
    if (silent_no_access() && address >= 1 && address <= 16 && function == 0x03 && size >= 6 &&
        rcon_no_access(be16(frame, 2)))
        return;
    if (address == 0) {
        std::lock_guard<std::mutex> lock(gw_mutex_);
        dropped_.push_back(frame);  // not carried (RCON: ME0413 p.6-1 #2; ROBONET: not documented)
        return;
    }
    if (rcon_ && address == rcon_->slave()) {
        emit(rcon_->handle(frame));
        return;
    }
    if (robonet_ && address == robonet_->slave()) {
        emit(robonet_->handle(frame));
        return;
    }
    if (kind_ == LineKind::RobonetGateway) return;  // the axes sit behind the gateway's internal link
    std::shared_ptr<SimulatedUnit> unit;
    try {
        unit = SimLine::unit(address);
    } catch (const std::out_of_range&) {
        return;
    }
    if (!contains(function_codes(), function)) {
        {
            std::lock_guard<std::mutex> lock(gw_mutex_);
            rejected_.push_back(frame);
        }
        emit(exception_frame(address, function, 0x01));
        return;
    }
    if (kind_ == LineKind::Robonet) {
        if (size > kRoboNetBufferBytes) return;
        if ((function == 0x03 || function == 0x04) && size >= 4 && be16(frame, 2) < kRoboNetNoAnswerBelow) return;
    }
    if (kind_ == LineKind::Rcon && function == 0x03 && size >= 8) {
        const int start = be16(frame, 2);
        if (start >= kSerialBase && start < kSerialBase + kSerialWords) {
            emit(serial_reply(address, start, be16(frame, 4)));
            return;
        }
    }
    const Bytes request = to_unit(address, frame);
    unit->write(request.data(), request.size());
    std::lock_guard<std::mutex> lock(gw_mutex_);
    pending_unit_ = unit;  // read() takes its answer (an FC11 answer may come late)
    pending_request_ = frame;
}

std::size_t SimGatewayLine::read(std::uint8_t* buffer, std::size_t size) {
    check_open();
    std::shared_ptr<SimulatedUnit> unit;
    Bytes request;
    {
        std::lock_guard<std::mutex> lock(gw_mutex_);
        unit = pending_unit_;
        request = pending_request_;
    }
    const Bytes reply = unit ? read_unit(*unit) : Bytes();  // without the line's lock: the unit takes its own
    std::lock_guard<std::mutex> lock(gw_mutex_);
    if (!reply.empty()) {
        pending_unit_.reset();
        pending_request_.clear();
        const Bytes wire = from_unit(request, reply);
        out_.insert(out_.end(), wire.begin(), wire.end());
    }
    const std::size_t n = std::min(size, out_.size());
    std::copy(out_.begin(), out_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
    out_.erase(out_.begin(), out_.begin() + static_cast<std::ptrdiff_t>(n));
    return n;
}

void SimGatewayLine::reset_input() {
    check_open();
    {
        std::lock_guard<std::mutex> lock(gw_mutex_);
        out_.clear();
        pending_unit_.reset();
        pending_request_.clear();
    }
    SimLine::reset_input();
}

std::vector<std::uint8_t> SimGatewayLine::serial_reply(int slave, int start, int count) const {
    std::string text = serial_code(slave).substr(0, 2 * kSerialWords);
    text.resize(2 * kSerialWords, '\0');
    if (start + count > kSerialBase + kSerialWords || count < 1) return exception_frame(slave, 0x03, 0x02);
    std::vector<std::uint16_t> words;
    for (int i = 0; i < kSerialWords; ++i)
        words.push_back(static_cast<std::uint16_t>((static_cast<unsigned char>(text[2 * i]) << 8) |
                                                   static_cast<unsigned char>(text[2 * i + 1])));
    const std::vector<std::uint16_t> part(words.begin() + (start - kSerialBase),
                                          words.begin() + (start - kSerialBase + count));
    return read_reply(slave, 0x03, part);
}

std::vector<std::uint8_t> SimGatewayLine::to_unit(int slave, const std::vector<std::uint8_t>& frame) const {
    const int factor = position_factor(slave);
    const int function = frame[1];
    if (factor == 1 || (function != 0x10 && function != 0x17)) return frame;
    const Bytes body(frame.begin() + 2, frame.end() - 2);
    Bytes pdu;
    if (function == 0x10) {
        if (body.size() < 5) return frame;
        const int start = be16(body, 0);
        const std::size_t nbytes = body[4];
        const std::vector<std::uint16_t> words = scale_words(start, words_of(body, 5, nbytes), factor, true);
        pdu.assign(body.begin(), body.begin() + 5);
        pdu.insert(pdu.begin(), 0x10);
        push_words(pdu, words);
    } else {
        if (body.size() < 9) return frame;
        const int write_address = be16(body, 4);
        const std::size_t nbytes = body[8];
        const std::vector<std::uint16_t> words = scale_words(write_address, words_of(body, 9, nbytes), factor, true);
        pdu.assign(body.begin(), body.begin() + 9);
        pdu.insert(pdu.begin(), 0x17);
        push_words(pdu, words);
    }
    return build_frame(slave, pdu);
}

std::vector<std::uint8_t> SimGatewayLine::from_unit(const std::vector<std::uint8_t>& request,
                                                    const std::vector<std::uint8_t>& reply) const {
    const int slave = request[0];
    const int function = request[1];
    const int factor = position_factor(slave);
    if (factor == 1 || reply.size() < 5 || reply[1] != function ||
        (function != 0x03 && function != 0x04 && function != 0x17))
        return reply;
    const int start = be16(request, 2);  // FC03 / 04 start, or the FC17 read address
    const std::vector<std::uint16_t> words = scale_words(start, words_of(reply, 3, reply.size() - 5), factor, false);
    return read_reply(slave, function, words);
}

// ------------------------------------------------------------------------------------------ factories

std::shared_ptr<SimGatewayLine> sim_rcon_line(const std::vector<int>& slaves, SimGatewayOptions options,
                                              SimulatedUnitOptions unit) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    for (int slave : slaves) {
        unit.slave = slave;
        units.push_back(std::make_shared<SimulatedUnit>(unit));
    }
    options.kind = LineKind::Rcon;
    if (!options.name) options.name = std::string("RCON");
    return std::make_shared<SimGatewayLine>(std::move(units), std::move(options));
}

std::shared_ptr<SimGatewayLine> sim_robonet_line(const std::vector<int>& slaves, bool gateway_mode,
                                                 SimulatedUnitOptions unit, const std::string& name) {
    std::vector<std::shared_ptr<SimulatedUnit>> units;
    for (int slave : slaves) {
        unit.slave = slave;
        units.push_back(std::make_shared<SimulatedUnit>(unit));
    }
    SimGatewayOptions options;
    options.kind = gateway_mode ? LineKind::RobonetGateway : LineKind::Robonet;
    options.name = name;
    return std::make_shared<SimGatewayLine>(std::move(units), std::move(options));
}

}  // namespace iai_actuator_driver
