// The simulated SEL controller (sel_sim.hpp).
#include "sel_sim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/timing.hpp"
#include "sel_frame.hpp"

namespace iai_actuator_driver {
namespace sel {

namespace {

constexpr double kMmPerS2PerG = 9806.65;
constexpr int kMaxAxes = 8;
constexpr double kInchingLimit = 1000.0;  // a longer inching distance is taken as 1000 mm (DOC)
constexpr double kStrokeEpsilon = 1e-6;

struct ProfileFacts {
    int model_code;
    std::size_t command_bytes;
    std::size_t reply_bytes;
    int position_capacity;
    bool four_digit_positions;  // 21EH, 21FH, 26BH
};

ProfileFacts facts_of(SelSimProfile profile) {
    switch (profile) {
        case SelSimProfile::tt: return {0xBC, 1023, 1024, 3000, false};
        case SelSimProfile::tta: break;
    }
    return {0xB9, 2051, 2048, 30000, true};
}

/// X / Y / Z / R: strokes, speeds and power-up positions are the simulator's own values (UNVERIFIED for a TTA).
std::vector<SelSimAxis> profile_axes(SelSimProfile) {
    return {{0.0, 200.0, 800.0, 25.0, false},
            {0.0, 200.0, 800.0, 25.0, false},
            {0.0, 100.0, 400.0, 10.0, false},
            {0.0, 360.0, 720.0, 0.0, true}};
}

bool is_execute(int id) noexcept { return id >= kServo; }

/// A command's data fields read in order; any mismatch clears ok().
class Fields {
public:
    explicit Fields(std::string_view data) : data_(data) {}
    std::uint32_t u(int width) {
        const std::string_view text = take(width);
        const std::optional<std::uint32_t> v = parse_hex(text);
        if (!v) ok_ = false;
        return v.value_or(0);
    }
    std::int32_t s(int width) {
        const std::string_view text = take(width);
        const std::optional<std::int32_t> v = parse_hex_signed(text);
        if (!v) ok_ = false;
        return v.value_or(0);
    }
    std::string_view take(int width) {
        const auto w = static_cast<std::size_t>(width);
        if (!ok_ || data_.size() - pos_ < w) {
            ok_ = false;
            return {};
        }
        const std::string_view text = data_.substr(pos_, w);
        pos_ += w;
        return text;
    }
    /// Every field read and nothing left over.
    bool done() const noexcept { return ok_ && pos_ == data_.size(); }

private:
    std::string_view data_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

/// One trapezoid over a distance S >= 0 from speed v0 >= 0: phase 1 to the cruise speed vc (accelerating at `up`, or
/// decelerating at `down` when v0 is above it), cruise, then down to 0 at `down`. When S is too short to stop from v0
/// at `down`, a single deceleration over S (a harder stop).
struct Ramp {
    double S = 0.0, v0 = 0.0, vc = 0.0, rate1 = 0.0, down = 1.0;
    double t1 = 0.0, t2 = 0.0, t3 = 0.0;

    static Ramp plan(double S, double v0, double vmax, double up, double down) {
        Ramp r;
        r.S = std::max(S, 0.0);
        r.v0 = std::max(v0, 0.0);
        r.down = down;
        if (r.S <= 0.0) return r;
        if (r.v0 > 0.0 && r.v0 * r.v0 / (2.0 * down) >= r.S) {
            r.vc = r.v0;
            r.down = r.v0 * r.v0 / (2.0 * r.S);
            r.t3 = r.v0 / r.down;
            return r;
        }
        double dist1 = 0.0;
        if (r.v0 <= vmax) {
            r.vc = vmax;
            r.rate1 = up;
            dist1 = (r.vc * r.vc - r.v0 * r.v0) / (2.0 * up);
            if (dist1 + r.vc * r.vc / (2.0 * down) > r.S) {
                r.vc = std::sqrt((r.S + r.v0 * r.v0 / (2.0 * up)) / (1.0 / (2.0 * up) + 1.0 / (2.0 * down)));
                dist1 = (r.vc * r.vc - r.v0 * r.v0) / (2.0 * up);
            }
            r.t1 = (r.vc - r.v0) / up;
        } else {
            r.vc = vmax;
            r.rate1 = -down;
            dist1 = (r.v0 * r.v0 - r.vc * r.vc) / (2.0 * down);
            r.t1 = (r.v0 - r.vc) / down;
        }
        const double cruise = std::max(0.0, r.S - dist1 - r.vc * r.vc / (2.0 * down));
        r.t2 = r.t1 + cruise / r.vc;
        r.t3 = r.t2 + r.vc / down;
        return r;
    }

    double s(double t) const {
        if (t <= 0.0) return 0.0;
        if (t >= t3) return S;
        if (t < t1) return v0 * t + 0.5 * rate1 * t * t;
        const double s1 = v0 * t1 + 0.5 * rate1 * t1 * t1;
        if (t < t2) return s1 + vc * (t - t1);
        const double tau = t - t2;
        return std::min(S, s1 + vc * (t2 - t1) + vc * tau - 0.5 * down * tau * tau);
    }
    double v(double t) const {
        if (t <= 0.0) return v0;
        if (t >= t3) return 0.0;
        if (t < t1) return v0 + rate1 * t;
        if (t < t2) return vc;
        return std::max(0.0, vc - down * (t - t2));
    }
};

enum class OpKind { servo_on, servo_off, home, move };

/// One accepted operation command over its axes (CP: one ramp for all of them).
struct Op {
    OpKind kind = OpKind::move;
    AxisPattern axes = 0;
    double start_t = 0.0;  // bit 0 turns ON then (the reply time + in_use_delay_s)
    bool started = false;
    bool cancelled = false;
    std::array<double, kMaxAxes> from{}, to{};
    double distance = 0.0;  // the longest axis' distance: the ramp's unit
    double speed = 0.0, accel = 0.0, decel = 0.0;  // in that unit (/s, /s^2)
    Ramp ramp;
    double ramp_t0 = 0.0;
    double dwell = 0.0;  // in use after the ramp (the home's creep)
    double end_t = 0.0;  // servo commands: when they finish; motion: ramp_t0 + ramp.t3 + dwell

    bool motion() const noexcept { return kind == OpKind::home || kind == OpKind::move; }
    bool has(int index) const noexcept { return axes & (1u << index); }
    double fraction(double t) const {
        if (!started || !motion()) return 0.0;
        if (distance <= 0.0) return 1.0;
        return std::min(1.0, ramp.s(t - ramp_t0) / distance);
    }
    double position(int index, double t) const { return from[index] + (to[index] - from[index]) * fraction(t); }
    void replan(double S, double v0, double vmax, double t) {
        ramp = Ramp::plan(S, v0, vmax, accel, decel);
        ramp_t0 = t;
        end_t = t + ramp.t3 + dwell;
    }
};

struct AxisLive {
    SelSimAxis spec;
    double position = 0.0;
    bool servo = false;
    int home_state = 0;
    bool completed = false;
    int error_code = 0;
};

struct Outcome {
    bool reply = true;
    int error = 0;  // an error reply with this code (0: a normal reply with `data`)
    std::string data;
    bool acted = false;
};

Outcome refused(int code) {
    Outcome o;
    o.error = code;
    return o;
}

Outcome accepted(std::string data = {}) {
    Outcome o;
    o.data = std::move(data);
    o.acted = true;
    return o;
}

std::uint32_t round_steps(double value, double scale, std::uint32_t high) {
    const double scaled = std::max(0.0, value * scale);
    return static_cast<std::uint32_t>(std::min<double>(std::llround(scaled), high));
}

std::string position_field(double value) {
    const long long steps = std::llround(value * 1000.0);
    return hex_signed(static_cast<std::int32_t>(std::clamp<long long>(steps, INT32_MIN, INT32_MAX)));
}

}  // namespace

const char* to_string(SelSimProfile profile) noexcept {
    switch (profile) {
        case SelSimProfile::tta: return "tta";
        case SelSimProfile::tt: return "tt";
    }
    return "?";
}

struct SimulatedSel::Impl {
    mutable std::mutex mutex;
    SimulatedSelOptions options;
    ProfileFacts facts;
    std::function<double()> clock;
    double origin = 0.0;

    std::vector<AxisLive> axes;
    std::vector<Op> ops;
    std::map<int, PositionData> positions;
    std::map<int, std::uint32_t> settings;
    SelSimMode mode = SelSimMode::automatic;
    bool estop = false;
    bool ready = true;
    bool flash_writing = false;
    bool execute_enabled = true;
    bool silent = false;
    double dead_until = -1.0;
    int latest_error = 0;
    std::deque<SelSimFault> faults;

    // the line
    std::string rx;
    bool rx_overflow = false;
    std::deque<std::pair<double, std::string>> tx;  // replies with the simulated time they become readable
    std::size_t tx_offset = 0;
    bool open = true;
    bool present = true;
    bool handle_lost = false;
    double read_timeout = kDefaultReadTimeoutS;

    // counters
    std::map<int, int> frames;
    std::map<int, int> executed;
    int dropped = 0;
    int replies = 0;

    explicit Impl(const SimulatedSelOptions& o) : options(o), facts(facts_of(o.profile)) {
        if (o.station < 0 || o.station > 0xFF) throw std::out_of_range("SimulatedSel: station not 0..FFh");
        if (o.time_scale <= 0.0) throw std::invalid_argument("SimulatedSel: time_scale must be above 0");
        if (o.model_code) facts.model_code = *o.model_code;
        if (o.max_command_bytes) facts.command_bytes = *o.max_command_bytes;
        if (o.max_reply_bytes) facts.reply_bytes = *o.max_reply_bytes;
        if (o.position_capacity) facts.position_capacity = *o.position_capacity;
        if (facts.command_bytes < kMinFrameSize || facts.reply_bytes < kMinFrameSize) {
            throw std::invalid_argument("SimulatedSel: a buffer shorter than a frame");
        }
        const std::vector<SelSimAxis> specs = o.axes ? *o.axes : profile_axes(o.profile);
        if (specs.empty() || specs.size() > kMaxAxes) throw std::invalid_argument("SimulatedSel: 1..8 axes");
        for (const SelSimAxis& spec : specs) {
            AxisLive a;
            a.spec = spec;
            a.position = spec.position;
            axes.push_back(a);
        }
        clock = o.clock ? o.clock : [] { return now_s(); };
        origin = clock();
        positions = o.positions;
        settings = o.settings;
        execute_enabled = o.execute_enabled;
        power_up_levels(true);
    }

    double now() const { return (clock() - origin) * options.time_scale; }

    AxisPattern present_mask() const { return static_cast<AxisPattern>((1u << axes.size()) - 1u); }

    /// Servo and home as at power-up (`first`: the options' initial levels; later: servos OFF, homes kept only on an
    /// absolute machine).
    void power_up_levels(bool first) {
        ops.clear();
        for (AxisLive& a : axes) {
            a.servo = first && options.servo_on;
            if (first) {
                a.home_state = options.homed ? 2 : 0;
            } else if (!options.absolute) {
                a.home_state = 0;
            }
            a.completed = false;
            a.error_code = 0;
        }
        latest_error = 0;
    }

    // ---- simulated time ------------------------------------------------------------------------------------------
    void start(Op& op) {
        op.started = true;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!op.has(i) || !op.motion()) continue;
            axes[i].completed = false;
            if (op.kind == OpKind::home) axes[i].home_state = 1;
        }
    }

    void finish(Op& op, double t) {
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!op.has(i)) continue;
            AxisLive& a = axes[i];
            switch (op.kind) {
                case OpKind::servo_on: a.servo = !op.cancelled; break;
                case OpKind::servo_off: a.servo = false; break;
                case OpKind::home:
                case OpKind::move:
                    a.position = op.started ? op.position(i, t) : a.position;
                    a.completed = !op.cancelled;
                    if (op.kind == OpKind::home) a.home_state = op.cancelled ? 0 : 2;
                    break;
            }
        }
    }

    void advance(double t) {
        for (std::size_t k = 0; k < ops.size();) {
            Op& op = ops[k];
            if (!op.started && t >= op.start_t) start(op);
            if (op.started && t >= op.end_t) {
                finish(op, op.end_t);
                ops.erase(ops.begin() + static_cast<std::ptrdiff_t>(k));
                continue;
            }
            ++k;
        }
    }

    const Op* op_of(int index) const {
        for (const Op& op : ops) {
            if (op.has(index)) return &op;
        }
        return nullptr;
    }

    double position_of(int index, double t) const {
        const Op* op = op_of(index);
        return op && op->motion() && op->started ? op->position(index, t) : axes[index].position;
    }

    SelSimAxisState state_of(int index, double t) const {
        const AxisLive& a = axes[index];
        const Op* op = op_of(index);
        SelSimAxisState s;
        s.position = position_of(index, t);
        s.servo_on = a.servo;
        s.home_state = a.home_state;
        s.in_use = op && op->started;
        s.completed = a.completed;
        s.error_code = a.error_code;
        return s;
    }

    /// Every operation on `mask` ends now (an emergency stop, a mode switch, an axis error): bit 4 OFF.
    void cancel_now(AxisPattern mask, double t) {
        for (std::size_t k = 0; k < ops.size();) {
            Op& op = ops[k];
            if ((op.axes & mask) == 0) {
                ++k;
                continue;
            }
            if (op.motion()) {
                op.cancelled = true;
                for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                    if (op.has(i)) axes[i].completed = false;
                }
            } else if (op.kind == OpKind::servo_on) {
                op.cancelled = true;
            }
            finish(op, t);
            ops.erase(ops.begin() + static_cast<std::ptrdiff_t>(k));
        }
    }

    /// 238H: a pending operation is dropped (bit 4 OFF), a running motion decelerates to a stop and ends cancelled.
    void stop(AxisPattern mask, double t) {
        for (std::size_t k = 0; k < ops.size();) {
            Op& op = ops[k];
            if ((op.axes & mask) == 0 || !op.motion() || op.cancelled) {
                ++k;
                continue;
            }
            if (!op.started) {
                op.cancelled = true;
                finish(op, t);
                ops.erase(ops.begin() + static_cast<std::ptrdiff_t>(k));
                continue;
            }
            op.cancelled = true;
            const double elapsed = t - op.ramp_t0;
            if (op.distance <= 0.0 || elapsed >= op.ramp.t3) {
                op.end_t = t;  // at the target already (the home's dwell): ends now
            } else {
                const double v = op.ramp.v(elapsed);
                const double s = op.ramp.s(elapsed);
                const double S = v * v / (2.0 * op.decel);
                for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                    if (!op.has(i)) continue;
                    const double now_pos = op.from[i] + (op.to[i] - op.from[i]) * s / op.distance;
                    const double dir = (op.to[i] - op.from[i]) / op.distance;
                    op.from[i] = now_pos;
                    op.to[i] = now_pos + dir * std::min(S, op.distance - s);
                }
                op.distance = std::min(S, op.distance - s);
                op.dwell = 0.0;
                op.replan(op.distance, v, std::max(v, 1e-9), t);
            }
            ++k;
        }
    }

    /// 262H: the running moves of `mask` continue to their targets at `speed` (units of their own distance).
    void change_speed(AxisPattern mask, double speed_mm_s, double t) {
        for (Op& op : ops) {
            if ((op.axes & mask) == 0 || op.kind != OpKind::move || op.cancelled) continue;
            const double speed = std::min(speed_mm_s, max_speed(op.axes));
            if (!op.started) {
                op.speed = speed;
                op.replan(op.distance, 0.0, speed, op.start_t);
                continue;
            }
            const double elapsed = t - op.ramp_t0;
            if (op.distance <= 0.0 || elapsed >= op.ramp.t3) continue;
            const double v = op.ramp.v(elapsed);
            const double s = op.ramp.s(elapsed);
            for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                if (op.has(i)) op.from[i] = op.from[i] + (op.to[i] - op.from[i]) * s / op.distance;
            }
            op.distance -= s;
            op.speed = speed;
            op.replan(op.distance, v, speed, t);
        }
    }

    double max_speed(AxisPattern mask) const {
        double v = 1e300;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (mask & (1u << i)) v = std::min(v, axes[i].spec.max_speed);
        }
        return v;
    }

    // ---- refusals ------------------------------------------------------------------------------------------------
    /// The checks every axis of an operation passes (0: none failed).
    int check_axes(AxisPattern mask, bool need_servo, bool need_home) const {
        if (mask & ~present_mask()) return kSelSimErrorNoAxis;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!(mask & (1u << i))) continue;
            if (axes[i].error_code != 0) return kSelSimErrorAxisError;
            if (op_of(i)) return kSelSimErrorAxisInUse;
            if (need_servo && !axes[i].servo) return kSelSimErrorServoOff;
            if (need_home && axes[i].home_state != 2) return kSelSimErrorNotHomed;
        }
        return 0;
    }

    bool in_stroke(int index, double target) const {
        const SelSimAxis& spec = axes[index].spec;
        return target >= spec.min - kStrokeEpsilon && target <= spec.max + kStrokeEpsilon;
    }

    /// Queue a motion over `mask` to `targets` (by axis index); speed in mm/s, accelerations in G (0: parameter).
    void add_motion(OpKind kind, AxisPattern mask, const std::array<double, kMaxAxes>& targets, double speed,
                    double accel_g, double decel_g, double dwell, double t) {
        Op op;
        op.kind = kind;
        op.axes = mask;
        op.start_t = t + options.in_use_delay_s;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!op.has(i)) continue;
            op.from[i] = axes[i].position;
            op.to[i] = targets[i];
            op.distance = std::max(op.distance, std::fabs(targets[i] - axes[i].position));
        }
        op.speed = std::min(speed > 0.0 ? speed : options.default_speed, max_speed(mask));
        const double a = std::min(accel_g > 0.0 ? accel_g : options.default_accel_g, options.max_accel_g);
        const double d = std::min(decel_g > 0.0 ? decel_g : options.default_accel_g, options.max_accel_g);
        op.accel = a * kMmPerS2PerG;
        op.decel = d * kMmPerS2PerG;
        op.dwell = dwell;
        op.replan(op.distance, 0.0, op.speed, op.start_t);
        ops.push_back(op);
    }

    void add_servo(AxisPattern mask, bool on, double t) {
        Op op;
        op.kind = on ? OpKind::servo_on : OpKind::servo_off;
        op.axes = mask;
        op.start_t = t + options.in_use_delay_s;
        op.end_t = op.start_t + options.servo_s;
        ops.push_back(op);
    }

    // ---- messages ------------------------------------------------------------------------------------------------
    bool supported(int id) const {
        switch (id) {
            case kTestCall: case kVersionQuery: case kSettingQuery: case kPositionCountQuery: case kPositionQuery:
            case kAxisStatusQuery: case kSystemStatusQuery: case kErrorDetailQuery: case kServo: case kHome:
            case kMoveAbsolute: case kMoveRelative: case kJog: case kMoveToPosition: case kStop: case kAlarmReset:
            case kSoftwareReset: case kChangeSpeed:
                return true;
            case kPositionCountQuery2: case kPositionQuery2: case kMoveToPosition2:
                return facts.four_digit_positions;
            default:
                return false;
        }
    }

    /// The mode / ready checks of an execute command (0: may run).
    int check_execute(int id) const {
        if (id == kStop) return 0;
        if (mode == SelSimMode::manual) return kSelSimErrorManualMode;
        if (!ready) return kSelSimErrorNotReady;
        return 0;
    }

    Outcome dispatch(int id, std::string_view data, double t) {
        if (!supported(id)) return refused(kSelSimErrorUnsupported);
        if (is_execute(id)) {
            if (const int code = check_execute(id)) return refused(code);
        }
        switch (id) {
            case kTestCall:
                return data.size() == 10 ? accepted(std::string(data)) : refused(kSelSimErrorFormat);
            case kVersionQuery: return version(data);
            case kSettingQuery: return setting(data);
            case kPositionCountQuery: case kPositionCountQuery2: return position_count(id, data);
            case kPositionQuery: case kPositionQuery2: return position_read(id, data);
            case kAxisStatusQuery: return axis_status(data, t);
            case kSystemStatusQuery: return system_status(data);
            case kErrorDetailQuery: return error_detail(data);
            case kServo: return servo(data, t);
            case kHome: return home(data, t);
            case kMoveAbsolute: case kMoveRelative: return move(id, data, t);
            case kJog: return jog(data, t);
            case kMoveToPosition: case kMoveToPosition2: return move_to_position(id, data, t);
            case kStop: return stop_command(data, t);
            case kAlarmReset: return alarm_reset(data);
            case kSoftwareReset: return software_reset(data, t);
            case kChangeSpeed: return speed_change(data, t);
            default: return refused(kSelSimErrorUnsupported);
        }
    }

    Outcome version(std::string_view data) const {
        Fields in(data);
        const std::uint32_t type = in.u(2);
        const std::uint32_t device = in.u(1);
        if (!in.done() || type > 3 || device != 0) return refused(kSelSimErrorFormat);
        static constexpr std::uint32_t kVersions[] = {0x0203, 0x0105, 0x0110, 0x0100};  // simulator values
        return accepted(hex(type, 2) + hex(device, 1) + hex(static_cast<std::uint32_t>(facts.model_code), 2) +
                        hex(0, 2) + hex(kVersions[type], 4) + hex(2019, 4) + hex(5, 2) + hex(14, 2) + hex(10, 2) +
                        hex(30, 2) + hex(0, 2));
    }

    Outcome setting(std::string_view data) const {
        Fields in(data);
        const bool fixed = in.take(3) == "090";
        const std::uint32_t number = in.u(3);
        const bool tail = in.take(3) == "001";
        if (!in.done() || !fixed || !tail) return refused(kSelSimErrorFormat);
        const auto it = settings.find(static_cast<int>(number));
        return accepted("090" + hex(it == settings.end() ? 0u : it->second, 8));
    }

    int width_of(int id) const {
        return id == kPositionCountQuery || id == kPositionQuery || id == kMoveToPosition ? 3 : 4;
    }

    Outcome position_count(int id, std::string_view data) const {
        if (!data.empty()) return refused(kSelSimErrorFormat);
        const int w = width_of(id);
        const int highest = w == 3 ? 0xFFF : 0xFFFF;
        std::uint32_t n = 0;
        for (const auto& [number, row] : positions) {
            if (number <= highest) ++n;
        }
        return accepted(hex(std::min<std::uint32_t>(n, static_cast<std::uint32_t>(highest)), w));
    }

    std::string record(int number, const PositionData& row, int w) const {
        std::string text = hex(static_cast<std::uint32_t>(number), w) + hex(row.axes, 2) +
                           hex(round_steps(row.profile.accel, 100.0, 0xFFFF), 4) +
                           hex(round_steps(row.profile.decel, 100.0, 0xFFFF), 4) +
                           hex(round_steps(row.profile.speed, 1.0, 0xFFFF), 4);
        for (std::size_t k = 0; k < static_cast<std::size_t>(sel::axis_count(row.axes)); ++k) {
            text += position_field(k < row.positions.size() ? row.positions[k] : 0.0);
        }
        return text;
    }

    /// 209H / 21FH: the defined records from the head, as many as fit in the reply buffer (DOC: clipped).
    Outcome position_read(int id, std::string_view data) const {
        const int w = width_of(id);
        Fields in(data);
        const std::uint32_t head = in.u(w);
        const std::uint32_t count = in.u(w);
        if (!in.done() || count == 0) return refused(kSelSimErrorFormat);
        // '#' + station + id + count field + records + checksum + CR LF
        const std::size_t fixed = 1 + 2 + 3 + static_cast<std::size_t>(w) + 2 + 2;
        std::size_t size = fixed;
        std::string records;
        std::uint32_t n = 0;
        const long long last = static_cast<long long>(head) + count - 1;
        for (auto it = positions.lower_bound(static_cast<int>(head)); it != positions.end() && it->first <= last;
             ++it) {
            const std::string one = record(it->first, it->second, w);
            if (size + one.size() > facts.reply_bytes) break;
            size += one.size();
            records += one;
            ++n;
        }
        return accepted(hex(n, w) + records);
    }

    Outcome axis_status(std::string_view data, double t) const {
        Fields in(data);
        const auto asked = static_cast<AxisPattern>(in.u(2));
        if (!in.done() || asked == 0) return refused(kSelSimErrorFormat);
        const auto present = static_cast<AxisPattern>(asked & present_mask());
        std::string text = hex(present, 2);
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!(present & (1u << i))) continue;
            const SelSimAxisState s = state_of(i, t);
            unsigned status = (s.in_use ? 0x01u : 0u) | static_cast<unsigned>(s.home_state) << 1 |
                              (s.servo_on ? 0x08u : 0u) | (s.completed ? 0x10u : 0u);
            const unsigned sensor = s.home_state == 2 && std::fabs(s.position) < 1e-9 ? 0x4u : 0u;
            text += hex(status, 2) + hex(sensor, 1) + hex(static_cast<std::uint32_t>(s.error_code & 0xFFF), 3) +
                    hex(0, 2) + position_field(s.position);
        }
        return accepted(text);
    }

    Outcome system_status(std::string_view data) const {
        if (!data.empty()) return refused(kSelSimErrorFormat);
        const unsigned byte1 = (mode == SelSimMode::manual ? 0x01u : 0u) | (estop ? 0x08u : 0u);
        const unsigned byte2 = flash_writing ? 0x01u : 0u;
        const unsigned byte3 = (estop ? 0x01u : 0u) | (mode == SelSimMode::automatic ? 0x02u : 0u) |
                               (ready ? 0x04u : 0u);
        return accepted(hex(static_cast<std::uint32_t>(mode), 1) + hex(0, 3) +
                        hex(static_cast<std::uint32_t>(latest_error & 0xFFF), 3) + hex(byte1, 2) + hex(byte2, 2) +
                        hex(byte3, 2) + hex(0, 2));
    }

    Outcome error_detail(std::string_view data) const {
        Fields in(data);
        const std::uint32_t type1 = in.u(1);
        const std::uint32_t type2 = in.u(2);
        in.u(3);
        if (!in.done()) return refused(kSelSimErrorFormat);
        int number = 0;
        std::array<std::int32_t, 8> detail{};
        if (type1 == 0 && type2 == 1) number = latest_error;
        if (type1 == 1 && type2 >= 1 && type2 <= axes.size()) {
            number = axes[type2 - 1].error_code;
            if (number) detail[2] = static_cast<std::int32_t>(type2);  // detail 3: the axis (DOC)
        }
        std::string text = hex(static_cast<std::uint32_t>(number & 0xFFF), 3);
        for (std::int32_t d : detail) text += hex_signed(d);
        return accepted(text + std::string(15, '0') + "00");  // reserved, an empty message (the newer models)
    }

    Outcome servo(std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const std::uint32_t type = in.u(1);
        if (!in.done() || mask == 0) return refused(kSelSimErrorFormat);
        const bool on = type & 1u;
        if (on && estop) return refused(kSelSimErrorEmergencyStop);
        if (mask & ~present_mask()) return refused(kSelSimErrorNoAxis);
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!(mask & (1u << i))) continue;
            if (on && axes[i].error_code != 0) return refused(kSelSimErrorAxisError);
            if (op_of(i)) return refused(kSelSimErrorAxisInUse);
        }
        add_servo(mask, on, t);
        return accepted();
    }

    Outcome home(std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const std::uint32_t search = in.u(3);
        in.u(3);  // the creep speed: the creep is home_dwell_s
        if (!in.done() || mask == 0) return refused(kSelSimErrorFormat);
        if (estop) return refused(kSelSimErrorEmergencyStop);
        if (const int code = check_axes(mask, true, false)) return refused(code);
        std::array<double, kMaxAxes> targets{};
        add_motion(OpKind::home, mask, targets, search ? search : options.home_speed, 0.0, 0.0, options.home_dwell_s,
                   t);
        return accepted();
    }

    /// The checks and the queueing shared by every move (targets by axis index).
    Outcome start_move(AxisPattern mask, const std::array<double, kMaxAxes>& targets, double speed, double accel,
                       double decel, double t) {
        if (estop) return refused(kSelSimErrorEmergencyStop);
        if (const int code = check_axes(mask, true, true)) return refused(code);
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if ((mask & (1u << i)) && !in_stroke(i, targets[i])) return refused(kSelSimErrorOutOfStroke);
        }
        add_motion(OpKind::move, mask, targets, speed, accel, decel, 0.0, t);
        return accepted();
    }

    Outcome move(int id, std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const double accel = in.u(4) / 100.0;
        const double decel = in.u(4) / 100.0;
        const double speed = in.u(4);
        std::array<double, kMaxAxes> values{};
        for (int axis : axes_of(mask)) values[axis - 1] = in.s(8) / 1000.0;
        if (!in.done() || mask == 0) return refused(kSelSimErrorFormat);
        if (mask & ~present_mask()) return refused(kSelSimErrorNoAxis);
        if (id == kMoveRelative) {
            for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                if (mask & (1u << i)) values[i] += axes[i].position;
            }
        }
        return start_move(mask, values, speed, accel, decel, t);
    }

    Outcome jog(std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const double accel = in.u(4) / 100.0;
        const double decel = in.u(4) / 100.0;
        const double speed = in.u(4);
        const double distance = std::min(in.u(8) / 1000.0, kInchingLimit);
        const bool positive = in.u(1) & 1u;
        if (!in.done() || mask == 0) return refused(kSelSimErrorFormat);
        if (mask & ~present_mask()) return refused(kSelSimErrorNoAxis);
        std::array<double, kMaxAxes> targets{};
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!(mask & (1u << i))) continue;
            const SelSimAxis& spec = axes[i].spec;
            if (distance == 0.0) {
                targets[i] = positive ? spec.max : spec.min;  // a jog runs to the soft limit unless stopped
            } else {
                targets[i] = axes[i].position + (positive ? distance : -distance);
            }
        }
        return start_move(mask, targets, speed, accel, decel, t);
    }

    Outcome move_to_position(int id, std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const double accel = in.u(4) / 100.0;
        const double decel = in.u(4) / 100.0;
        const double speed = in.u(4);
        const int number = static_cast<int>(in.u(width_of(id)));
        if (!in.done() || mask == 0) return refused(kSelSimErrorFormat);
        if (mask & ~present_mask()) return refused(kSelSimErrorNoAxis);
        const auto it = positions.find(number);
        if (it == positions.end()) return refused(kSelSimErrorNoPosition);
        const PositionData& row = it->second;
        const auto moving = static_cast<AxisPattern>(mask & row.axes);  // ANDed with the position's pattern (DOC)
        if (moving == 0 || (moving & ~present_mask())) return refused(kSelSimErrorNoAxis);
        std::array<double, kMaxAxes> targets{};
        std::size_t k = 0;
        for (int axis : axes_of(row.axes)) {
            if (k < row.positions.size()) targets[axis - 1] = row.positions[k];
            ++k;
        }
        return start_move(moving, targets, speed ? speed : row.profile.speed, accel ? accel : row.profile.accel,
                          decel ? decel : row.profile.decel, t);
    }

    Outcome stop_command(std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        in.u(2);
        if (!in.done()) return refused(kSelSimErrorFormat);
        stop(mask, t);
        return accepted();
    }

    Outcome alarm_reset(std::string_view data) {
        if (!data.empty()) return refused(kSelSimErrorFormat);
        for (AxisLive& a : axes) a.error_code = 0;
        latest_error = 0;
        return accepted();
    }

    Outcome software_reset(std::string_view data, double t) {
        if (!data.empty()) return refused(kSelSimErrorFormat);
        power_up_levels(false);
        dead_until = t + options.reset_s;
        Outcome o = accepted();
        o.reply = false;  // no reply (DOC)
        return o;
    }

    Outcome speed_change(std::string_view data, double t) {
        Fields in(data);
        const auto mask = static_cast<AxisPattern>(in.u(2));
        const std::uint32_t speed = in.u(4);
        if (!in.done() || mask == 0 || speed == 0) return refused(kSelSimErrorFormat);
        change_speed(mask, speed, t);
        return accepted();
    }

    // ---- the line ------------------------------------------------------------------------------------------------
    void check_handle() const {
        if (handle_lost) throw TransportError(options.port_name + ": simulated port vanished (unplugged): reopen it");
        if (!open) throw TransportError(options.port_name + ": port is closed");
    }

    void queue(std::string bytes, double t) {
        tx.emplace_back(t + options.reply_latency_s, std::move(bytes));
        ++replies;
    }

    std::string apply_fault(SelSimFault fault, const std::string& bytes, int id, bool error) const {
        switch (fault) {
            case SelSimFault::bad_checksum: {
                std::string bad = bytes;
                const std::size_t at = bad.size() - 3;  // the checksum's last digit
                bad[at] = bad[at] == '0' ? '1' : '0';
                return bad;
            }
            case SelSimFault::wrong_station: {
                const int other = (options.station + 1) & 0xFF;
                const DecodeResult d = decode(bytes);
                return error ? encode_error_reply(other, d.frame.id) : encode_reply(other, id, d.frame.data);
            }
            case SelSimFault::wrong_id: {
                if (error) return bytes;  // an error reply carries no id
                const DecodeResult d = decode(bytes);
                return encode_reply(options.station, id == kTestCall ? kVersionQuery : kTestCall, d.frame.data);
            }
            case SelSimFault::partial: return bytes.substr(0, bytes.size() / 2);
            default: return bytes;
        }
    }

    void handle_line(const std::string& line, double t) {
        if (line.size() > facts.command_bytes) {
            ++dropped;
            return;
        }
        const DecodeResult d = decode(line);
        if (!d.ok() || d.frame.kind != FrameKind::command || d.frame.station != options.station) {
            ++dropped;
            return;
        }
        const int id = d.frame.id;
        ++frames[id];
        if (silent || t < dead_until) {
            ++dropped;
            return;
        }
        std::optional<SelSimFault> fault;
        if (!faults.empty()) {
            fault = faults.front();
            faults.pop_front();
        }
        if (fault == SelSimFault::silence) return;
        if (fault == SelSimFault::error_reply) {
            queue(encode_error_reply(options.station, kSelSimErrorInjected), t);
            return;
        }
        if (is_execute(id) && !execute_enabled) return;  // a channel not enabled for execution: silence (DOC)
        const Outcome o = dispatch(id, d.frame.data, t);
        if (o.acted) ++executed[id];
        if (!o.reply || fault == SelSimFault::silent_after_acting) return;
        const bool error = o.error != 0;
        std::string bytes = error ? encode_error_reply(options.station, o.error)
                                  : encode_reply(options.station, id, o.data);
        if (fault) bytes = apply_fault(*fault, bytes, id, error);
        queue(std::move(bytes), t);
    }
};

SimulatedSel::SimulatedSel(const SimulatedSelOptions& options) : impl_(std::make_unique<Impl>(options)) {}
SimulatedSel::~SimulatedSel() = default;

void SimulatedSel::write(const std::uint8_t* data, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.check_handle();
    const double t = s.now();
    s.advance(t);
    for (std::size_t i = 0; i < size; ++i) {
        const char c = static_cast<char>(data[i]);
        if (!s.rx_overflow) {
            s.rx.push_back(c);
            if (s.rx.size() > s.facts.command_bytes) s.rx_overflow = true;  // a receive error: dropped at its LF
        }
        if (c != '\n') continue;
        if (s.rx_overflow) {
            ++s.dropped;
        } else {
            s.handle_line(s.rx, t);
        }
        s.rx.clear();
        s.rx_overflow = false;
    }
}

std::size_t SimulatedSel::read(std::uint8_t* buffer, std::size_t size) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.check_handle();
    const double t = s.now();
    std::size_t n = 0;
    while (n < size && !s.tx.empty() && s.tx.front().first <= t) {
        const std::string& bytes = s.tx.front().second;
        const std::size_t take = std::min(size - n, bytes.size() - s.tx_offset);
        std::copy_n(bytes.data() + s.tx_offset, take, buffer + n);
        n += take;
        s.tx_offset += take;
        if (s.tx_offset == bytes.size()) {
            s.tx.pop_front();
            s.tx_offset = 0;
        }
    }
    return n;
}

void SimulatedSel::reset_input() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    s.check_handle();
    // only what has arrived is discarded: a reply still inside its latency arrives later (a late reply)
    const double t = s.now();
    while (!s.tx.empty() && s.tx.front().first <= t) {
        s.tx.pop_front();
        s.tx_offset = 0;
    }
}

void SimulatedSel::set_baud_rate(int) { throw std::logic_error("SimulatedSel models no baud rate"); }

double SimulatedSel::read_timeout() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->read_timeout;
}

void SimulatedSel::set_read_timeout(double seconds) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->read_timeout = seconds;
}

void SimulatedSel::send_break(double) { throw std::logic_error("SimulatedSel models no line break"); }

void SimulatedSel::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->open = false;
}

bool SimulatedSel::is_open() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->open && !impl_->handle_lost;
}

std::string SimulatedSel::name() const { return impl_->options.port_name; }

std::optional<bool> SimulatedSel::present() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->present;
}

void SimulatedSel::reopen(std::optional<int>) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    if (!s.present) throw TransportError("could not open " + s.options.port_name + ": it is not present (unplugged)");
    s.handle_lost = false;
    s.open = true;
    s.rx.clear();
    s.rx_overflow = false;
    s.tx.clear();
    s.tx_offset = 0;
}

SelSimProfile SimulatedSel::profile() const { return impl_->options.profile; }
int SimulatedSel::station() const { return impl_->options.station; }
int SimulatedSel::axis_count() const { return static_cast<int>(impl_->axes.size()); }
std::size_t SimulatedSel::max_command_bytes() const { return impl_->facts.command_bytes; }
std::size_t SimulatedSel::max_reply_bytes() const { return impl_->facts.reply_bytes; }

SelSimAxisState SimulatedSel::axis(int axis) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    if (axis < 1 || axis > static_cast<int>(s.axes.size())) throw std::out_of_range("SimulatedSel: no such axis");
    const double t = s.now();
    s.advance(t);
    return s.state_of(axis - 1, t);
}

SelSimMode SimulatedSel::mode() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->mode;
}

bool SimulatedSel::emergency_stop() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->estop;
}

double SimulatedSel::now() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->now();
}

void SimulatedSel::set_mode(SelSimMode mode) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    const double t = s.now();
    s.advance(t);
    if (mode == SelSimMode::manual && s.mode != SelSimMode::manual) s.cancel_now(s.present_mask(), t);
    s.mode = mode;
}

void SimulatedSel::set_emergency_stop(bool on) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    const double t = s.now();
    s.advance(t);
    if (on && !s.estop) {
        s.cancel_now(s.present_mask(), t);
        for (AxisLive& a : s.axes) {
            a.servo = false;
            if (a.home_state == 1) a.home_state = 0;
        }
    }
    s.estop = on;
}

void SimulatedSel::set_ready(bool ready) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->ready = ready;
}

void SimulatedSel::set_flash_writing(bool on) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->flash_writing = on;
}

void SimulatedSel::set_execute_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->execute_enabled = enabled;
}

void SimulatedSel::set_axis_error(int axis, int code) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    if (axis < 1 || axis > static_cast<int>(s.axes.size())) throw std::out_of_range("SimulatedSel: no such axis");
    if (code <= 0 || code > 0xFFF) throw std::out_of_range("SimulatedSel: error code not 1..FFFh");
    const double t = s.now();
    s.advance(t);
    s.cancel_now(axis_bit(axis), t);
    AxisLive& a = s.axes[axis - 1];
    a.servo = false;
    if (a.home_state == 1) a.home_state = 0;
    a.error_code = code;
    s.latest_error = code;
}

void SimulatedSel::set_axis(int axis, std::optional<bool> servo_on, std::optional<bool> homed,
                            std::optional<double> position) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    if (axis < 1 || axis > static_cast<int>(s.axes.size())) throw std::out_of_range("SimulatedSel: no such axis");
    s.advance(s.now());
    if (s.op_of(axis - 1)) throw std::logic_error("SimulatedSel: the axis is in use");
    AxisLive& a = s.axes[axis - 1];
    if (servo_on) a.servo = *servo_on;
    if (homed) a.home_state = *homed ? 2 : 0;
    if (position) a.position = *position;
}

void SimulatedSel::set_position_data(int number, const PositionData& data) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (number < 0 || number > impl_->facts.position_capacity) {
        throw std::out_of_range("SimulatedSel: position number beyond the table");
    }
    impl_->positions[number] = data;
}

void SimulatedSel::clear_position_data(int number) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->positions.erase(number);
}

void SimulatedSel::set_setting(int number, std::uint32_t value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->settings[number] = value;
}

void SimulatedSel::inject_fault(SelSimFault fault, int count) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (int i = 0; i < count; ++i) impl_->faults.push_back(fault);
}

void SimulatedSel::set_silent(bool silent) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->silent = silent;
}

void SimulatedSel::power_cycle() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl& s = *impl_;
    const double t = s.now();
    s.power_up_levels(false);  // the mode switch and the emergency stop keep their positions
    s.faults.clear();
    s.tx.clear();
    s.tx_offset = 0;
    s.rx.clear();
    s.rx_overflow = false;
    s.dead_until = t + s.options.reset_s;
}

void SimulatedSel::unplug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->present = false;
    impl_->handle_lost = true;
}

void SimulatedSel::replug() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->present = true;
}

int SimulatedSel::frames(int message_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->frames.find(message_id);
    return it == impl_->frames.end() ? 0 : it->second;
}

int SimulatedSel::frames() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    int n = 0;
    for (const auto& [id, count] : impl_->frames) n += count;
    return n;
}

int SimulatedSel::executed(int message_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->executed.find(message_id);
    return it == impl_->executed.end() ? 0 : it->second;
}

int SimulatedSel::dropped() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->dropped;
}

int SimulatedSel::replies() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->replies;
}

void SimulatedSel::reset_counters() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->frames.clear();
    impl_->executed.clear();
    impl_->dropped = 0;
    impl_->replies = 0;
}

}  // namespace sel
}  // namespace iai_actuator_driver
