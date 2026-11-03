// SimulatedUnit (a Transport): construction, the public API, request frames and their answers, registers, coils, the
// IAI 0x45 commands, parameter RAM / non-volatile store and PARW. The motion model and the axis behaviour are in
// sim_motion.cpp, the per-profile data in sim_fixtures.cpp. Port of the Python edition's sim.py (SimulatedEC); the
// hardware behaviour it models is VERIFIED in the Python edition where sim.hpp says so, this edition is UNVERIFIED on
// hardware. Unofficial: not an IAI product. Header-only dependencies only: the simulator links on its own.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "iai_actuator_driver/sim.hpp"
#include "sim_fixtures.hpp"

namespace iai_actuator_driver {

using sim_detail::SimException;
using Bytes = std::vector<std::uint8_t>;
using Words = std::vector<std::uint16_t>;

namespace {

inline int be16(const Bytes& b, std::size_t at) { return (b[at] << 8) | b[at + 1]; }

inline void push16(Bytes& b, int value) {
    b.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    b.push_back(static_cast<std::uint8_t>(value & 0xFF));
}

/// The fixture of a profile after checking every option the constructor validates.
const sim_detail::ProfileFixture& checked_fixture(const SimulatedUnitOptions& o) {
    if (!(o.time_scale > 0.0)) throw std::invalid_argument("time_scale must be > 0");
    bool known = false;
    for (SimProfile p : sim_profiles()) known = known || p == o.profile;
    if (!known) throw std::invalid_argument("unknown simulator profile");
    if (o.dopm && *o.dopm != DopmLayout::ThreePosition && *o.dopm != DopmLayout::TwoPosition)
        throw std::invalid_argument("unknown DOPM layout");
    return sim_detail::fixture(o.profile);
}

}  // namespace

// ================================================================================================ profile helpers

const char* to_string(SimProfile profile) {
    switch (profile) {
        case SimProfile::Slider: return "slider";
        case SimProfile::Gripper: return "gripper";
        case SimProfile::Rod: return "rod";
        case SimProfile::Pcon: return "pcon";
        case SimProfile::Rotary: return "rotary";
        case SimProfile::Unknown: return "unknown";
    }
    return "?";
}

std::optional<SimProfile> sim_profile_from_string(const std::string& name) {
    const std::string wanted = util::lower(util::trim(name));
    for (SimProfile p : sim_profiles())
        if (wanted == to_string(p)) return p;
    return std::nullopt;
}

std::vector<SimProfile> sim_profiles() {
    return {SimProfile::Slider, SimProfile::Gripper, SimProfile::Rod,
            SimProfile::Pcon,   SimProfile::Rotary,  SimProfile::Unknown};
}

// ================================================================================================ Impl: construction

SimulatedUnit::Impl::Impl(const SimulatedUnitOptions& options)
    : opt(options),
      fx(checked_fixture(options)),
      profile(options.profile),
      slave(options.slave),
      stroke_mm(options.stroke_mm.value_or(fx.stroke_mm)),
      time_scale(options.time_scale),
      max_speed_mm_s(options.max_speed_mm_s.value_or(fx.max_speed_mm_s)),
      max_accel_g(options.max_accel_g),
      system_code(options.system_code.value_or(fx.system_code)),
      unit_code(options.unit_code.value_or(fx.unit_code)),
      firmware(options.firmware.value_or(fx.firmware)),
      dopm(options.dopm.value_or(fx.dopm)),
      absolute(options.absolute.value_or(fx.absolute)),
      con_reset(fx.con_reset),
      rtc_unset(fx.rtc_alarm),
      pio_unwired(fx.pio_unwired),
      action_mode_locks_son(fx.action_mode_locks_son),
      recompute_position_checksum(options.recompute_position_checksum),
      drop_reply_after_retarget(options.drop_reply_after_retarget),
      stop_latches(options.stop_latches),
      move_bit_clears_in_decel(options.move_bit_clears_in_decel),
      serve_soft_limits(options.soft_limits),
      soft_limits{-reg::kSoftLimitMarginMm, stroke_mm + reg::kSoftLimitMarginMm},
      ls_range_mm(options.ls_range_mm),
      early_in_position(options.early_in_position),
      settle_tail_mm(options.settle_tail_mm),
      settle_speed_mm_s(options.settle_speed_mm_s),
      push_blip_mm(options.push_blip_mm),
      push_blip_len_mm(options.push_blip_len_mm),
      slave_id_delay_s(options.slave_id_delay_s.value_or(fx.slave_id_delay_s)),
      parw_busy_s(options.parw_busy_s),
      parw_dsse_busy(options.parw_dsse_busy),
      ignored_baud_codes(options.ignored_baud_codes.begin(), options.ignored_baud_codes.end()),
      boot_baud_rate(options.boot_baud_rate),
      reset_baud_rate(options.reset_baud_rate),
      reset_s(options.reset_s),
      clock(options.clock ? options.clock : std::function<double()>([] { return now_s(); })) {
    t0 = clock();
    pmsl = options.pmsl;
    safety_speed = options.safety_speed;
    apply_options(options);  // the settings a test may change later (sim_controls.cpp)
    if (options.baud_rate) {
        line_baud = options.baud_rate;
        ec_baud = options.unit_baud_rate.value_or(*options.baud_rate);
        baud_history.push_back(*options.baud_rate);
    }

    // read-only information blocks
    index = util::hex_words(fx.index_table_hex);
    identification = util::hex_words(fx.identification_hex);
    if (options.serial_number) set_serial_number(*options.serial_number);
    limit_params = util::hex_words(fx.limit_params_hex);
    {
        const auto hi = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(soft_limits.max * 100.0)));
        const auto lo = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(soft_limits.min * 100.0)));
        limit_params[0] = hi.first;
        limit_params[1] = hi.second;
        limit_params[2] = lo.first;
        limit_params[3] = lo.second;
    }
    motion_params = util::hex_words(fx.motion_params_hex);
    {
        const auto v = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(max_speed_mm_s * 100.0)));
        motion_params[0x12] = v.first;  // 0x0812-0x0813 = item 8.10, the default speed (equal to MAXV on every unit)
        motion_params[0x13] = v.second;
    }
    for (const auto& block : fx.blocks) {
        const Words words = util::hex_words(block.second);
        for (std::size_t i = 0; i < words.size(); ++i) statics[block.first + static_cast<int>(i)] = words[i];
    }
    for (std::size_t i = 0; i < fx.support.size(); ++i)
        statics.emplace(0x0100 + static_cast<int>(i), fx.support[i]);  // class 1 where the blocks have none
    if (const auto akid = item_address(reg::kLimitParamsCategory, reg::kAxisItemAkid); akid && fx.akid) {
        const int offset = *akid - reg::kSoftLimits;
        const auto w = reg::to_u32_words(static_cast<std::uint32_t>(*fx.akid));
        limit_params[static_cast<std::size_t>(offset)] = w.first;
        limit_params[static_cast<std::size_t>(offset) + 1] = w.second;
    }
    if (const auto maxv = item_address(reg::kLimitParamsCategory, reg::kAxisItemMaxv);
        maxv && *maxv >= reg::kSoftLimits && *maxv < reg::kSoftLimits + static_cast<int>(limit_params.size()) - 1) {
        const auto w = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(max_speed_mm_s * 100.0)));
        limit_params[static_cast<std::size_t>(*maxv - reg::kSoftLimits)] = w.first;
        limit_params[static_cast<std::size_t>(*maxv - reg::kSoftLimits) + 1] = w.second;
    }
    alarm_item_address = item_address(reg::kControllerMonitorCategory, 10);
    move_counter_address = item_address(0x0200, 1);  // maintenance block 0x8400: TLMC, item 1
    ssse = static_cast<std::uint16_t>(fx.ssse | (options.rtc ? reg::kSsseRtc : 0));
    ollv = fx.ollv;
    {
        const Words seeded = util::hex_words(fx.alarm_history_hex);
        for (std::size_t i = 0; i + 6 <= seeded.size() && alarm_history.size() < sim_detail::kHistoryEntries; i += 6) {
            std::array<std::uint16_t, 6> entry{};
            std::copy(seeded.begin() + static_cast<std::ptrdiff_t>(i), seeded.begin() + static_cast<std::ptrdiff_t>(i) + 6,
                      entry.begin());
            alarm_history.push_back(entry);
        }
        while (alarm_history.size() < static_cast<std::size_t>(sim_detail::kHistoryEntries))
            alarm_history.push_back({0x0000, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0xFFFF});
    }

    // position table 0x1000-0x3FFF: the rows read live and the 0x2000 page (rows 256-267)
    table.assign(static_cast<std::size_t>(reg::kPositionStride * sim_detail::kRowCount), 0);
    if (fx.blank_row.size() == static_cast<std::size_t>(reg::kPositionStride))
        for (int n = 0; n < sim_detail::kRowCount; ++n)
            std::copy(fx.blank_row.begin(), fx.blank_row.end(),
                      table.begin() + static_cast<std::ptrdiff_t>(n) * reg::kPositionStride);
    for (const auto* rows : {&fx.positions, &fx.simple_rows})
        for (const auto& row : *rows)
            std::copy(row.second.begin(), row.second.end(),
                      table.begin() + static_cast<std::ptrdiff_t>(row.first) * reg::kPositionStride);

    // 0x9900 block (live readback: 73.95 mm / 0.10 mm / 700.00 mm/s / 0.30 G, PPOW 0, CTLF 0)
    const double start_mm = options.position_mm.value_or(fx.position_mm);
    {
        const auto p = reg::to_i32_words(static_cast<std::int32_t>(sim_detail::py_round(start_mm * 100.0)));
        const auto b = reg::to_u32_words(10);
        const auto v = reg::to_u32_words(static_cast<std::uint32_t>(sim_detail::py_round(max_speed_mm_s * 100.0)));
        direct = {p.first, p.second, b.first, b.second, v.first, v.second, 30, 0, 0, 0};
    }

    // axis state (live: servo on, homed, no alarm)
    pos = start_mm;
    target = start_mm;
    accel = 0.3 * reg::kGmms2;
    decel = accel;
    t = now();
    home_params_at_home = home_parameters();
    if (rtc_unset) raise_alarm(reg::kAlarmRtcStop);  // PCON at power-up: message level, the axis still runs
    if (cold_boot) {
        servo_on = false;  // CON after power-up: servo off, PSNS 0 until the first servo ON
        excited = false;
    }
}

// ================================================================================================ SimulatedUnit: public API

SimulatedUnit::SimulatedUnit(const SimulatedUnitOptions& options) : impl_(new Impl(options)) {}

SimulatedUnit::SimulatedUnit(SimProfile profile, int slave, double time_scale) {
    SimulatedUnitOptions options;
    options.profile = profile;
    options.slave = slave;
    options.time_scale = time_scale;
    impl_.reset(new Impl(options));
}

SimulatedUnit::~SimulatedUnit() = default;

SimProfile SimulatedUnit::profile() const { return impl_->profile; }
int SimulatedUnit::slave() const { return impl_->slave; }
double SimulatedUnit::stroke_mm() const { return impl_->stroke_mm; }
double SimulatedUnit::max_speed_mm_s() const { return impl_->max_speed_mm_s; }
int SimulatedUnit::system_code() const { return impl_->system_code; }
int SimulatedUnit::unit_code() const { return impl_->unit_code; }
int SimulatedUnit::position_count() const {
    // The stored rows the unit reports (MAXP, item 3.2 of its controller monitor class): the slider 3, the gripper and
    // the rod 2, the PCON 64. Falls back to 3 / 2 (3-position / 2-position mode) when the directory has no such item.
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const Impl& s = *impl_;
    if (const auto address = s.item_address(reg::kControllerMonitorCategory, reg::kControllerItemMaxp)) {
        const auto word = [&s](int a) {
            const auto it = s.statics.find(a);
            return it == s.statics.end() ? std::uint16_t(0) : it->second;
        };
        const std::uint32_t rows = reg::from_u32_words(word(*address), word(*address + 1));
        if (rows > 0) return static_cast<int>(rows);
    }
    return s.dopm == DopmLayout::ThreePosition ? 3 : 2;
}
bool SimulatedUnit::absolute() const { return impl_->absolute; }
double SimulatedUnit::time_scale() const { return impl_->time_scale; }

bool SimulatedUnit::servo_on() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->servo_on;
}
bool SimulatedUnit::homed() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->homed;
}
bool SimulatedUnit::pmsl() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->pmsl;
}
bool SimulatedUnit::safety_speed() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->safety_speed;
}
bool SimulatedUnit::paused() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->paused;
}
bool SimulatedUnit::stop_held() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stop_held();
}
int SimulatedUnit::alarm_code() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->alarm_code;
}
double SimulatedUnit::position_mm() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->pos;
}
bool SimulatedUnit::moving() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    return impl_->moving;
}
Span SimulatedUnit::soft_limits_mm() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->soft_limits;
}
std::optional<double> SimulatedUnit::obstacle_mm() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->obstacle_mm;
}
void SimulatedUnit::set_obstacle_mm(std::optional<double> mm) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->obstacle_mm = mm;
}
std::optional<int> SimulatedUnit::unit_baud_rate() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->ec_baud;
}

int SimulatedUnit::nonvolatile_writes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->nonvolatile_writes;
}
int SimulatedUnit::parameter_commits() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->parw_tick();
    return impl_->parameter_commits;
}
int SimulatedUnit::parameter_ram_writes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->parameter_ram_writes;
}
int SimulatedUnit::software_resets() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->software_resets;
}
int SimulatedUnit::override_moves() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->override_moves;
}
std::vector<std::vector<std::uint8_t>> SimulatedUnit::requests() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {impl_->requests.begin(), impl_->requests.end()};
}
void SimulatedUnit::clear_requests() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->requests.clear();
}
std::vector<std::string> SimulatedUnit::events() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {impl_->events.begin(), impl_->events.end()};
}
void SimulatedUnit::clear_events() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->events.clear();
}
std::vector<int> SimulatedUnit::baud_history() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->baud_history;
}
std::map<int, std::uint16_t> SimulatedUnit::parameter_ram() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->param_ram;
}
std::map<int, std::uint16_t> SimulatedUnit::parameter_nv() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->parw_tick();
    return impl_->param_nv;
}
bool SimulatedUnit::parw_busy() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->parw_tick();
    return impl_->parw_until.has_value();
}

bool SimulatedUnit::coil_state(int address) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->coils.find(address);
    return it != impl_->coils.end() && it->second;
}
void SimulatedUnit::set_coil_state(int address, bool on) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->coils[address] = on;  // the stored level only: no action is taken (use a frame to act)
}
std::vector<std::uint16_t> SimulatedUnit::position_row(int row) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (row < 0 || row >= sim_detail::kRowCount) throw std::out_of_range("position row must be 0..767");
    return impl_->row_words(row);
}
void SimulatedUnit::set_position_row(int row, const std::vector<std::uint16_t>& words) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (row < 0 || row >= sim_detail::kRowCount) throw std::out_of_range("position row must be 0..767");
    if (words.size() != static_cast<std::size_t>(reg::kPositionStride)) throw std::invalid_argument("a row has 16 words");
    std::copy(words.begin(), words.end(), impl_->table.begin() + static_cast<std::ptrdiff_t>(row) * reg::kPositionStride);
}
std::uint16_t SimulatedUnit::register_value(int address) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->parw_tick();
    impl_->advance();
    try {
        return impl_->read_word(address);
    } catch (const SimException&) {
        throw std::out_of_range("simulated unit answers exception 02 at this address");
    }
}

void SimulatedUnit::inject_fault(SimFault kind, int count) {
    switch (kind) {
        case SimFault::Timeout:
        case SimFault::Crc:
        case SimFault::Partial:
        case SimFault::WrongSlave:
            break;
        default:
            throw std::invalid_argument("unknown fault kind");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (int i = 0; i < count; ++i) impl_->faults.push_back(kind);
}

void SimulatedUnit::trigger_alarm(int code) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->advance();
    impl_->raise_alarm(code);
}

bool SimulatedUnit::pio_move(double target_mm, double speed_mm_s, double accel_g, bool hold_start) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.advance();
    if (s.pmsl) {
        s.log_event("pio start ignored (PMSS = 1)");
        return false;
    }
    if (hold_start)
        s.pio_start = sim_detail::PioStart{target_mm, speed_mm_s, accel_g};
    else
        s.pio_start.reset();
    s.log_event(util::format("pio start %.2f mm%s", target_mm, hold_start ? " (start input held)" : ""));
    s.start_move(target_mm, 0.10, speed_mm_s, accel_g);
    return s.moving;
}

void SimulatedUnit::pio_release() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pio_start.reset();
}

void SimulatedUnit::power_cycle() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->log_event("power cycle");
    impl_->power_off();
    impl_->power_on();
}

// ---- Transport

std::vector<std::uint8_t> SimulatedUnit::handle_frame(const std::vector<std::uint8_t>& request) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->closed) throw TransportError("simulated port is closed");
    auto response = impl_->respond(request);
    return response ? *response : std::vector<std::uint8_t>{};
}

void SimulatedUnit::write(const std::uint8_t* data, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.check_handle();
    if (s.closed) throw TransportError("simulated port is closed");
    const Bytes request(data, data + size);
    auto response = s.respond(request);
    if (!response) return;
    if (size > 1 && data[1] == 0x11 && s.slave_id_delay_s > 0.0) s.reply_ready_at = now_s() + s.slave_id_delay_s;
    s.out.insert(s.out.end(), response->begin(), response->end());  // the rod answers FC11 late
}

std::size_t SimulatedUnit::read(std::uint8_t* buffer, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.check_handle();
    if (s.closed) throw TransportError("simulated port is closed");
    if (now_s() < s.reply_ready_at) return 0;
    const std::size_t n = std::min(size, s.out.size());
    for (std::size_t i = 0; i < n; ++i) buffer[i] = s.out[i];
    s.out.erase(s.out.begin(), s.out.begin() + static_cast<std::ptrdiff_t>(n));
    return n;
}

void SimulatedUnit::reset_input() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->check_handle();
    impl_->out.clear();
}

int SimulatedUnit::baud_rate() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->line_baud.value_or(0);
}

void SimulatedUnit::set_baud_rate(int rate) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->line_baud)
        throw std::logic_error("this SimulatedUnit has no baud rate (set SimulatedUnitOptions::baud_rate to model the line)");
    impl_->line_baud = rate;
    impl_->baud_history.push_back(rate);
    impl_->out.clear();
}

double SimulatedUnit::read_timeout() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->read_timeout;
}

void SimulatedUnit::set_read_timeout(double seconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->read_timeout = seconds;  // reads never block: the value is only stored
}

void SimulatedUnit::send_break(double seconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->log_event(util::format("break %.3f s", seconds));
    if (impl_->ec_baud && seconds >= sim_detail::kBreakTo9600S) {
        impl_->ec_baud = 9600;
        impl_->log_event("EC at 9600 baud (break)");
    }
}

void SimulatedUnit::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->closed = true;
}

bool SimulatedUnit::is_open() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return !impl_->closed;
}

std::string SimulatedUnit::name() const {
    return impl_->opt.port_name ? *impl_->opt.port_name : "SIM";  // a fixed fact: never changed after construction
}

// ================================================================================================ Impl: logs

void SimulatedUnit::Impl::log_event(const std::string& text) {
    events.push_back(text);
    if (events.size() > sim_detail::kLogLimit) events.pop_front();
}

// ================================================================================================ Impl: frames

std::optional<Bytes> SimulatedUnit::Impl::respond(const Bytes& request) {
    requests.push_back(request);
    if (requests.size() > sim_detail::kLogLimit) requests.pop_front();
    reply_ready_at = 0.0;  // a new request supersedes a delayed answer still pending
    if (!powered) {
        log_event("frame ignored (unit powered off)");
        return std::nullopt;
    }
    if (ec_baud && line_baud != ec_baud) {
        log_event(util::format("frame at %d baud not understood (EC at %d)", line_baud.value_or(0), *ec_baud));
        return std::nullopt;
    }
    if (now() < busy_until) {
        log_event("frame ignored (restarting after a software reset)");
        return std::nullopt;
    }
    const bool drop = drop_next_reply;
    drop_next_reply = false;
    std::optional<Bytes> response = handle_frame_core(request);
    if (!response || drop) return std::nullopt;
    if (faults.empty()) return response;
    const SimFault fault = faults.front();
    faults.pop_front();
    switch (fault) {
        case SimFault::Timeout:
            return std::nullopt;
        case SimFault::Crc:
            response->back() = static_cast<std::uint8_t>(response->back() ^ 0xFF);
            break;
        case SimFault::Partial:
            response->resize(std::max<std::size_t>(1, response->size() / 2));
            break;
        case SimFault::WrongSlave: {
            const Bytes pdu(response->begin() + 1, response->end() - 2);
            response = build_frame(((*response)[0] % 247) + 1, pdu);
            break;
        }
    }
    return response;
}

std::optional<Bytes> SimulatedUnit::Impl::handle_frame_core(const Bytes& frame) {
    if (frame.size() < 4 || !check_crc(frame)) return std::nullopt;  // a real slave stays silent on CRC / framing errors
    const int address = frame[0];
    const int function = frame[1];
    if (address != slave && address != 0) return std::nullopt;
    const Bytes body(frame.begin() + 2, frame.end() - 2);
    parw_tick();
    advance();
    if (address == 0 && !broadcast) {
        log_event(util::format("broadcast function 0x%02x ignored", function));
        return std::nullopt;
    }
    // a unit / line that does not carry a function (e.g. an RCON gateway: 01 / 03 / 04 / 06 / 10 / 17) answers 01
    const bool carried = !function_codes || function_codes->count(function) != 0;
    Bytes pdu;
    try {
        if (address == 0 && carried && function != 0x05 && function != 0x06 && function != 0x10 && function != 0x45 &&
            function != 0x48)
            return std::nullopt;  // broadcast: only writes and commands act (a broadcast read acts on nothing)
        switch (carried ? function : -1) {
            case 0x03: pdu = fc03(body); break;
            case 0x05: pdu = fc05(body); break;
            case 0x06: pdu = fc06(body); break;
            case 0x10: pdu = fc10(body); break;
            case 0x11: pdu = fc11(body); break;
            case 0x17: pdu = fc17(body); break;  // the command profile (sim_profile.cpp)
            case 0x45: pdu = fc45(body); break;
            case 0x48: pdu = fc48(body); break;  // sim_controls.cpp
            default: throw SimException{0x01};   // other function codes are not simulated
        }
    } catch (const SimException& e) {
        pdu = {static_cast<std::uint8_t>(function | 0x80), static_cast<std::uint8_t>(e.code)};
    }
    if (address == 0) return std::nullopt;  // broadcast: act, never answer
    return build_frame(slave, pdu);
}

Bytes SimulatedUnit::Impl::fc03(const Bytes& body) {
    if (body.size() != 4) throw SimException{0x03};
    const int address = be16(body, 0);
    const int count = be16(body, 2);
    if (count < 1 || count > 125) throw SimException{0x03};
    Words words;
    for (int i = 0; i < count; ++i) words.push_back(read_word(address + i));
    Bytes pdu = {0x03, static_cast<std::uint8_t>(2 * count)};
    for (std::uint16_t w : words) push16(pdu, w);
    return pdu;
}

Bytes SimulatedUnit::Impl::fc05(const Bytes& body) {
    if (body.size() != 4) throw SimException{0x03};
    const int address = be16(body, 0);
    const int value = be16(body, 2);
    if (value != 0xFF00 && value != 0x0000) throw SimException{0x03};
    write_coil(address, value == 0xFF00);
    Bytes pdu = {0x05};
    pdu.insert(pdu.end(), body.begin(), body.end());
    return pdu;
}

Bytes SimulatedUnit::Impl::fc06(const Bytes& body) {
    if (body.size() != 4) throw SimException{0x03};
    write_words(be16(body, 0), {static_cast<std::uint16_t>(be16(body, 2))});
    Bytes pdu = {0x06};
    pdu.insert(pdu.end(), body.begin(), body.end());
    return pdu;
}

Bytes SimulatedUnit::Impl::fc10(const Bytes& body) {
    if (body.size() < 5) throw SimException{0x03};
    const int address = be16(body, 0);
    const int count = be16(body, 2);
    const int nbytes = body[4];
    if (count < 1 || count > 123 || nbytes != 2 * count || body.size() != static_cast<std::size_t>(5 + nbytes))
        throw SimException{0x03};
    Words values;
    for (int i = 0; i < count; ++i) values.push_back(static_cast<std::uint16_t>(be16(body, static_cast<std::size_t>(5 + 2 * i))));
    write_words(address, values);
    Bytes pdu = {0x10};
    push16(pdu, address);
    push16(pdu, count);
    return pdu;
}

Bytes SimulatedUnit::Impl::fc11(const Bytes& body) {
    if (!body.empty()) throw SimException{0x03};
    return {0x11, 5, static_cast<std::uint8_t>(system_code), 0xFF, static_cast<std::uint8_t>(unit_code),
            static_cast<std::uint8_t>(firmware.first), static_cast<std::uint8_t>(firmware.second)};
}

Bytes SimulatedUnit::Impl::fc45(const Bytes& body) {
    // group u8 | command u8 | op1 u16 | op2 u16 (the group is ignored)
    if (body.size() != 6) throw SimException{0x03};
    const int command = body[1];
    const int op1 = be16(body, 2);
    const int op2 = be16(body, 4);
    if (command == reg::kIaiCmdActionMode) return {0x45, static_cast<std::uint8_t>(action_mode(op1, op2)), 0};
    if (command == reg::kIaiCmdMovePosition)
        return {0x45, static_cast<std::uint8_t>(move_position_override(op1, op2)), 0};
    int action = -1;
    switch (command) {
        case reg::kIaiCmdBaud:
        case reg::kIaiCmdReset:
        case reg::kIaiCmdServo:
        case reg::kIaiCmdHome:
        case reg::kIaiCmdStop:
            action = command;
            break;
        default:
            break;
    }
    if (action == reg::kIaiCmdHome && pmsl) {
        // op1 = ORG (item 9.16) & 0xFFFF: op1 = 7 (= ORG) accepted on the EC-R6S rod, op1 = 0 rejected 0x11 (VERIFIED in the
        // Python edition, test I3). Which other values the EC accepts is unknown: the simulator accepts any non-zero op1.
        if (op1 == 0) return {0x45, static_cast<std::uint8_t>(kSimRejectOperand1), 0};
        if (!servo_on) return {0x45, static_cast<std::uint8_t>(kSimRejectServoOff), 0};
        if (homing_phase != sim_detail::HomingPhase::Off) return {0x45, static_cast<std::uint8_t>(kSimRejectHoming), 0};
    }
    return {0x45, static_cast<std::uint8_t>(iai_command(action, op1)), 0};
}

int SimulatedUnit::Impl::move_position_override(int n, int override_value) {
    // 0x45 cmd 6: run stored row n at override / 255 of its stored speed (VERIFIED in the Python edition on the EC
    // gripper and the PCON). On an EC a row its position mode does not run (row 2 in 2-position mode) is rejected 0x11
    // "1st operand error" and nothing moves (VERIFIED in the Python edition on the EC-GRBP8M gripper, P9 = 0; 0x9800
    // with such a row is unverified: it still runs). Rejected 0x3A while PMSS = 0, 0x23 with the servo off, 0x24 while
    // homing, 0x12 for op2 = 0 (simulator choices, as the order of the checks).
    if (!pmsl) return kSimRejectMonitorMode;
    if (!servo_on) return kSimRejectServoOff;
    if (homing_phase != sim_detail::HomingPhase::Off) return kSimRejectHoming;
    const int mode_rows = dopm == DopmLayout::ThreePosition ? 3 : 2;  // the rows the position mode runs
    if ((system_code == reg::kSystemEc || system_code == reg::kSystemEc200) && (n < 0 || n >= mode_rows)) {
        log_event(util::format("0x45 cmd 6 row %d rejected: not used in %d-position mode", n, mode_rows));
        return kSimRejectOperand1;
    }
    if (override_value < 1 || override_value > 0xFF) return 0x12;
    ++override_moves;
    move_to_row(n, override_value / 255.0);
    return 0;
}

int SimulatedUnit::Impl::action_mode(int pc, int safety) {
    // 0x45 cmd 0x0C: op1 1 = PC / Modbus control (PMSS 1), 0 = PIO (PMSS 0); op2 = safety speed. VERIFIED in the Python
    // edition on the EC-R6S rod (1,1) (1,0) (0,0), accepted with PMSS 0 too. Switching to PC mode mid-move stops the move like
    // the PMSL coil (test 18; unverified for this command: worst case).
    if (pc > 1 || safety > 1) return pc > 1 ? kSimRejectOperand1 : 0x12;
    const bool switched_on = pc != 0 && !pmsl;
    const bool switched = (pc != 0) != pmsl;
    pmsl = pc != 0;
    safety_speed = safety != 0;
    log_event(util::format("action mode %s, safety %s", pc ? "PC" : "PIO", safety ? "on" : "off"));
    if (switched) control_switched(pc != 0);
    if (pc && action_mode_locks_son) {
        son_locked = true;
        log_event("DSSE b12 cleared by the action mode PC: servo ON is ignored until a PMSL coil OFF");
    }
    if (pc) pio_start.reset();
    if (switched_on && moving) {
        log_event("move stopped by the action-mode change");
        decel_stop();
    }
    return 0;
}

int SimulatedUnit::Impl::iai_command(int action, int op1) {
    if (action < 0) return kSimRejectCommandCode;
    if (action == reg::kIaiCmdReset) {
        if (op1 == reg::kIaiResetAlarm) {
            alarm_reset();
            return 0;
        }
        if (op1 == reg::kIaiResetSoftware) {
            software_reset_command();
            return 0;  // the reply goes out, then the unit restarts (ON-WIRE: 01 45 00 00)
        }
        return kSimRejectCommandCode;
    }
    if (action == reg::kIaiCmdBaud) {
        const std::optional<int> rate = sim_detail::baud_rate_of_code(op1);
        if (!rate) return kSimRejectCommandCode;
        if (ignored_baud_codes.count(op1)) {
            // The EC-R6S rod (VERIFIED in the Python edition): code 0x06 sent at the current rate was ignored.
            log_event(util::format("baud code 0x%02X (%d) ignored", op1, *rate));
            return 0;
        }
        log_event(util::format("baud change to %d", *rate));
        if (ec_baud) ec_baud = rate;  // a broadcast is not answered; a unicast answer still goes out at the old rate
        return 0;
    }
    if (!pmsl) return kSimRejectMonitorMode;
    if (action == reg::kIaiCmdServo)
        request_servo(op1 != 0);
    else if (action == reg::kIaiCmdHome)
        start_home();
    else
        decel_stop();
    return 0;
}

// ================================================================================================ Impl: parameters

void SimulatedUnit::Impl::start_commit() {
    // PARW coil ON: DRGE b7 reads 1 for parw_busy_s, then the RAM (as now) becomes non-volatile.
    if (parw_until) return;  // a commit is already running (the driver never sends the coil then: it waits for idle)
    parw_snapshot = param_ram;
    if (parw_busy_s <= 0.0) {
        finish_commit();
        return;
    }
    parw_until = now_s() + parw_busy_s;
    log_event("parameter commit (PARW) started");
}

void SimulatedUnit::Impl::parw_tick() {
    if (parw_until && now_s() >= *parw_until) finish_commit();
}

void SimulatedUnit::Impl::finish_commit() {
    // The parameters in RAM at the coil become the non-volatile values (ON-WIRE: the IAI PC software's "Transfer").
    for (const auto& kv : parw_snapshot) param_nv[kv.first] = kv.second;
    for (const auto& kv : parw_snapshot) {
        const auto it = param_ram.find(kv.first);
        if (it != param_ram.end() && it->second == kv.second) param_ram.erase(it);  // a word rewritten meanwhile stays RAM
    }
    parw_until.reset();
    parw_snapshot.clear();
    ++parameter_commits;
    log_event("parameters committed (PARW)");
}

bool SimulatedUnit::Impl::in_directory(int address) const {
    for (const auto& r : fx.ranges)
        if (r.first <= address && address < r.second) return true;
    return false;
}

bool SimulatedUnit::Impl::in_parameter_ram(int address, int count) const {
    for (const auto& r : fx.param_ranges)
        if (r.first <= address && address + count <= r.second) return true;
    return false;
}

std::optional<int> SimulatedUnit::Impl::item_address(int class_id, int item_id) const {
    // Address of the 32-bit item (class, item id) from the served directory and ID map, if any.
    const auto idx = [this](int a) { return a >= 0 && a < static_cast<int>(index.size()) ? static_cast<int>(index[static_cast<std::size_t>(a)]) : 0; };
    for (int i = 0; i < idx(2); ++i) {
        const int base = 8 + 8 * i;
        const int ci = idx(base), dq = idx(base + 2), da = idx(base + 4), ia = idx(base + 5), ms = idx(base + 7);
        if (ci != class_id || ia == 0 || ms != 2) continue;
        for (int k = 0; k < dq; ++k) {
            const auto it = statics.find(ia + 2 * k);
            if (it != statics.end() && it->second == item_id) return da + 2 * k;
        }
    }
    return std::nullopt;
}

std::uint16_t SimulatedUnit::Impl::base_word(int address) const {
    // A parameter word as served before any write (fixtures / the computed soft-limit and 0x0800 blocks).
    if (address >= reg::kSoftLimits && address < reg::kSoftLimits + static_cast<int>(limit_params.size()))
        return limit_params[static_cast<std::size_t>(address - reg::kSoftLimits)];
    if (address >= reg::kMotionParamsBase && address < reg::kMotionParamsBase + static_cast<int>(motion_params.size()))
        return motion_params[static_cast<std::size_t>(address - reg::kMotionParamsBase)];
    const auto it = statics.find(address);
    return it == statics.end() ? std::uint16_t(0) : it->second;
}

std::vector<std::optional<std::uint32_t>> SimulatedUnit::Impl::home_parameters() const {
    // Committed (non-volatile) raw values of P3 / P4, the home direction and offset (items 9.16 and 9.21).
    std::vector<std::optional<std::uint32_t>> values;
    for (const int item : {reg::kDriverItemOrg, 21}) {
        const auto address = item_address(reg::kFactoryParamsCategory, item);
        if (!address) {
            values.push_back(std::nullopt);
            continue;
        }
        std::uint16_t words[2];
        for (int k = 0; k < 2; ++k) {
            const auto it = param_nv.find(*address + k);
            words[k] = it != param_nv.end() ? it->second : base_word(*address + k);
        }
        values.push_back(reg::from_u32_words(words[0], words[1]));
    }
    return values;
}

void SimulatedUnit::Impl::apply_parameters() {
    // At a restart the unit applies its committed parameters: soft limits (P1 is item 7.1) and the LS range (P2).
    if (serve_soft_limits) {
        const std::int32_t hi = reg::from_i32_words(read_word(reg::kSoftLimits), read_word(reg::kSoftLimits + 1));
        const std::int32_t lo = reg::from_i32_words(read_word(reg::kSoftLimits + 2), read_word(reg::kSoftLimits + 3));
        if (lo / 100.0 < hi / 100.0) soft_limits = Span{lo / 100.0, hi / 100.0};
    }
    if (const auto address = item_address(reg::kMotionParamsCategory, 9)) {
        const std::uint32_t raw = reg::from_u32_words(read_word(*address), read_word(*address + 1));
        if (raw > 0 && raw < 10000) ls_range_mm = raw / 100.0;
    }
}

// ================================================================================================ Impl: registers

std::uint16_t SimulatedUnit::Impl::read_word(int address) const {
    if (address >= 0 && address < static_cast<int>(index.size())) return index[static_cast<std::size_t>(address)];
    if (const auto it = param_ram.find(address); it != param_ram.end()) return it->second;  // written since the last commit
    if (const auto it = param_nv.find(address); it != param_nv.end()) return it->second;    // a committed parameter
    if (address >= reg::kPositionTableBase && address < reg::kPositionTableBase + static_cast<int>(table.size()))
        return table[static_cast<std::size_t>(address - reg::kPositionTableBase)];
    if (address >= 0x9000 && address <= 0x9021) return monitor_word(address);
    if (address >= 0x9900 && address <= 0x9909)
        return address == 0x9909 ? drg1() : direct[static_cast<std::size_t>(address - 0x9900)];
    if (address >= 0x9800 && address <= 0x9802) {
        const std::uint16_t block[3] = {posr_imm, drg1(), drg2};
        return block[address - 0x9800];
    }
    if (address >= 0x9A00 && address <= 0x9A03) {  // PCMD mirror (live: = 0x9900 PCMD), PPOS, DRG1 mirror
        const std::uint16_t block[4] = {direct[0], direct[1], 0, drg1()};
        return block[address - 0x9A00];
    }
    if (address >= 0x0D00 && address <= 0x0D09) {
        const std::uint16_t drge = static_cast<std::uint16_t>((pmsl ? 0x0100 : 0) | (parw_until ? reg::kDrgeParw : 0));
        switch (address) {
            case reg::kDrg1: return drg1();
            case reg::kDrg2: return drg2;
            case reg::kDrge: return drge;
            case reg::kPosr: return posr;
            default: return 0;
        }
    }
    const int history_offset = address - reg::kAlarmHistoryBase;
    if (history_offset >= 0 && history_offset < reg::kAlarmHistoryStride * sim_detail::kHistoryEntries)
        return alarm_history[static_cast<std::size_t>(history_offset / reg::kAlarmHistoryStride)]
                            [static_cast<std::size_t>(history_offset % reg::kAlarmHistoryStride)];
    if (address >= reg::kIdentificationBase && address < reg::kIdentificationBase + static_cast<int>(identification.size()))
        return identification[static_cast<std::size_t>(address - reg::kIdentificationBase)];
    if (address >= reg::kSoftLimits && address < reg::kSoftLimits + static_cast<int>(limit_params.size())) {
        if (!serve_soft_limits) throw SimException{0x02};
        return limit_params[static_cast<std::size_t>(address - reg::kSoftLimits)];
    }
    if (address >= reg::kMotionParamsBase && address < reg::kMotionParamsBase + static_cast<int>(motion_params.size()))
        return motion_params[static_cast<std::size_t>(address - reg::kMotionParamsBase)];
    if (address >= 0x0200 && address <= 0x0209) return status_image(address - 0x0200);
    if (address >= reg::kProfileBase && address < reg::kProfileBase + reg::kProfileWords)
        return active_profile(address - reg::kProfileBase);  // the command profile (sim_profile.cpp)
    if (address == 0x0103 && opt.move_patterns) return *opt.move_patterns;  // MVPT override (test option)
    if (alarm_item_address && (address == *alarm_item_address || address == *alarm_item_address + 1)) {
        const auto w = reg::to_u32_words(static_cast<std::uint32_t>(alarm_code));
        return address == *alarm_item_address ? w.first : w.second;
    }
    if (const auto it = statics.find(address); it != statics.end()) return it->second;
    if (in_directory(address)) return 0;  // inside a directory block the simulator has no recorded data for
    throw SimException{0x02};             // stricter than the unit, which answers 0 everywhere: reads past a block show up
}

std::uint16_t SimulatedUnit::Impl::position_checksum_of_row(int row) const {
    const auto first = table.begin() + static_cast<std::ptrdiff_t>(row) * reg::kPositionStride;
    return reg::position_checksum(Words(first, first + reg::kPositionWordsWritable));
}

void SimulatedUnit::Impl::write_words(int address, const Words& values) {
    const int end = address + static_cast<int>(values.size());
    const int table_end = reg::kPositionTableBase + static_cast<int>(table.size());
    if (address >= reg::kPositionTableBase && end <= table_end) {
        const int offset = address - reg::kPositionTableBase;
        std::copy(values.begin(), values.end(), table.begin() + offset);
        ++nonvolatile_writes;
        if (recompute_position_checksum) {
            for (int n = offset / reg::kPositionStride; n <= (offset + static_cast<int>(values.size()) - 1) / reg::kPositionStride; ++n)
                table[static_cast<std::size_t>(n * reg::kPositionStride + reg::kPosChecksum)] = position_checksum_of_row(n);
        }
        log_event(util::format("table write 0x%04x x%d", address, static_cast<int>(values.size())));
        return;
    }
    if (address >= 0x9900 && end <= 0x990A) {
        write_direct(address, values);
        return;
    }
    if (values.size() == 1 && (address == reg::kDrg1 || address == reg::kDrg2 || address == reg::kPosr || address == 0x9800 ||
                               address == 0x9801 || address == 0x9802)) {
        const std::uint16_t value = values[0];
        if (address == reg::kDrg1 || address == 0x9801) {
            write_drg1(value);
        } else if (address == reg::kDrg2 || address == 0x9802) {
            drg2 = value;
        } else if (address == reg::kPosr) {
            posr = value;
        } else {
            posr_imm = value;
            if (pmsl) {
                move_to_row(value);
            } else {
                // Hardware test N4: accepted (normal reply), no motion, no exception, no alarm.
                log_event(util::format("0x9800 = %d ignored (PMSS = 0)", static_cast<int>(value)));
            }
        }
        return;
    }
    if (address >= reg::kProfileBase && end <= reg::kProfileBase + reg::kProfileWords) {
        write_profile(address, values);  // the command profile: no motion until PCRF (sim_profile.cpp)
        return;
    }
    if (in_parameter_ram(address, static_cast<int>(values.size()))) {
        // Hardware tests N2 / N3: accepted and read back at once, but RAM only until PARW commits it.
        for (std::size_t i = 0; i < values.size(); ++i) param_ram[address + static_cast<int>(i)] = values[i];
        ++parameter_ram_writes;
        log_event(util::format("parameter RAM write 0x%04x x%d", address, static_cast<int>(values.size())));
        return;
    }
    throw SimException{0x02};
}

void SimulatedUnit::Impl::write_direct(int address, const Words& values) {
    std::optional<std::uint16_t> new_drg1;
    for (std::size_t i = 0; i < values.size(); ++i) {
        const int r = address + static_cast<int>(i);
        if (r == 0x9909)
            new_drg1 = values[i];
        else
            direct[static_cast<std::size_t>(r - 0x9900)] = values[i];
    }
    const auto written = [&](int r) { return r >= address && r < address + static_cast<int>(values.size()); };
    if (new_drg1) write_drg1(*new_drg1);
    // ME0162 p.5-129: writing the low word of PCMD, VCMD, ACMD or PPOW starts / updates the move. A CTLF written on its
    // own stays pending until the next start (the stale-CTLF hazard). Hardware test 18: the move runs whatever PMSS is
    // (not gated by PMSL, contrary to ME0162).
    if (!(written(0x9901) || written(0x9905) || written(0x9906) || written(0x9907))) return;
    if (moving && drop_reply_after_retarget) drop_next_reply = true;
    const std::uint16_t ctlf = direct[8];
    direct[8] = 0;  // CTLF resets after every move start
    const double pcmd = reg::from_i32_words(direct[0], direct[1]) / 100.0;
    const double band_mm = reg::from_u32_words(direct[2], direct[3]) / 100.0;
    const double speed = reg::from_u32_words(direct[4], direct[5]) / 100.0;
    double accel_g = direct[6] / 100.0;
    wake_from_asof("0x9900 move");
    row_move.reset();
    set_accel_pattern(ctlf);  // CTLF b6-b7 (sim_profile.cpp)
    if (accel_pattern != "trapezoid") accel_g /= 2.0;  // smoothed profile: half the mean acceleration (simulator choice)
    if (ctlf & reg::kCtlfPush) {
        start_push(pcmd, band_mm, speed, accel_g, (ctlf & reg::kCtlfPushAway) ? 1 : -1);
        return;
    }
    start_move((ctlf & reg::kCtlfInc) ? pcmd + pos : pcmd, band_mm, speed, accel_g);
}

void SimulatedUnit::Impl::write_drg1(std::uint16_t value) {
    const std::uint16_t previous = drg1();
    drg1_other = static_cast<std::uint16_t>(value & ~(sim_detail::kDrg1Son | sim_detail::kDrg1Sfty | sim_detail::kDrg1Stp));
    if (!pmsl) return;
    const std::uint16_t rising = static_cast<std::uint16_t>(value & ~previous);
    safety_speed = (value & sim_detail::kDrg1Sfty) != 0;
    if (((value & sim_detail::kDrg1Son) != 0) != servo_on) request_servo((value & sim_detail::kDrg1Son) != 0);
    if (rising & sim_detail::kDrg1Alrs) alarm_reset();
    if (((value & sim_detail::kDrg1Stp) != 0) != stp_input) set_pause((value & sim_detail::kDrg1Stp) != 0);
    if (rising & sim_detail::kDrg1Home) start_home();
    if (rising & sim_detail::kDrg1Cstr) move_to_row(posr);
}

bool SimulatedUnit::Impl::stop_held() const {
    if (!stop_latches) return false;
    const auto it = coils.find(reg::kCoilStop);
    return it != coils.end() && it->second;
}

void SimulatedUnit::Impl::write_coil(int address, bool on) {
    switch (address) {
        case reg::kCoilSfty:
        case reg::kCoilSon:
        case reg::kCoilAlrs:
        case reg::kCoilStp:
        case reg::kCoilHome:
        case reg::kCoilCstr:
        case reg::kCoilPmsl:
        case reg::kCoilStop:
        case reg::kCoilParw:
        case reg::kCoilSfrs:
        case reg::kCoilPcrf:
        case reg::kCoilMod:
        case reg::kCoilTeac:
            break;
        default:
            throw SimException{0x02};
    }
    const bool previous = coils[address];
    coils[address] = on;
    const bool rising = on && !previous;
    if (address == reg::kCoilPmsl) {
        const bool switched_on = on && !pmsl;
        const bool switched = on != pmsl;
        if (switched && !on && son_locked) {
            son_locked = false;
            log_event("DSSE b12 set again by the PMSL coil OFF");
        }
        pmsl = on;
        log_event(on ? "pmsl on" : "pmsl off");
        if (switched) control_switched(on);
        if (on) pio_start.reset();  // PIO inputs are ignored while Modbus has control
        if (switched_on && moving) {
            // Hardware test 18: PMSL ON mid-move stopped the move (PEND = 1, MEND = 0).
            log_event("move stopped by PMSL");
            decel_stop();
        }
        return;
    }
    if (address == reg::kCoilStop) {
        // Hardware test 19: STOP stopped a Modbus-commanded move with PMSS = 0, so it is handled before the PMSL gate.
        // That it also stops a PIO move (pio_move) is the simulator's assumption.
        if (rising) {
            decel_stop();
        } else if (!on && previous) {
            log_event("stop released");
            if (pio_start && !pmsl) {
                // Worst-case model (untested on an EC): the PLC's start input is still on.
                const sim_detail::PioStart start = *pio_start;
                log_event("pio start input still on: move restarted");
                start_move(start.target_mm, 0.10, start.speed_mm_s, start.accel_g);
            }
        }
        if (!stop_latches) coils[address] = false;  // CON behaviour: the controller resets STOP itself
        return;
    }
    if (!pmsl) return;  // PMSS = 0: other coils are ignored (ME0162 p.7-8; unverified on the EC)
    switch (address) {
        case reg::kCoilSfty: safety_speed = on; break;
        case reg::kCoilSon: request_servo(on); break;
        case reg::kCoilMod:
            teach_mode = on;
            log_event(on ? "teach mode on" : "teach mode off");
            break;
        case reg::kCoilTeac:  // sim_controls.cpp
            if (rising)
                teach();
            else if (!on)
                teach_written = false;
            break;
        case reg::kCoilAlrs:
            if (rising) alarm_reset();
            break;
        case reg::kCoilStp: set_pause(on); break;
        case reg::kCoilHome:
            if (rising) start_home();
            break;
        case reg::kCoilCstr:
            if (rising) move_to_row(posr);
            break;
        case reg::kCoilParw:
            coils[address] = false;  // self-clearing: DRGE b7 follows the running commit (read_word)
            if (on && !parw_latches)
                log_event("PARW ignored");
            else if (on)
                start_commit();
            break;
        case reg::kCoilSfrs:
            if (rising) {
                coils[address] = false;
                software_reset_command();  // test R1: with the servo on -> alarm 0x090, no reset
            }
            break;
        case reg::kCoilPcrf:
            coils[address] = false;  // self-clearing (DRGE b1 reads 0 again; VERIFIED on the PCON)
            if (on) run_profile();   // sim_profile.cpp
            break;
        default:
            break;
    }
}

}  // namespace iai_actuator_driver
