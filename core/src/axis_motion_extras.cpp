// Axis motion extras: push (approach, push band, force %, contact / miss with the hold rule, the PPOW limits), the
// command-profile move with its own deceleration (FC17: 0x0F00 x20 written, 0x9000 x34 read; then the PCRF coil), the
// acceleration patterns (CTLF b6-b7), change_speed / change_accel of a running 0x9900 move, and move_sequence with
// blending. Port of ec.py Elecylinder (push, _check_push_force, _auto_servo_off_push_warnings, push_speed_mm_s,
// _accel_pattern_ctlf, _move_profile / _check_profile_move / _send_profile, _change_move / _change_warnings, _wait_band,
// move_sequence). The behaviour is VERIFIED in the Python edition where motion.hpp says so; this edition is UNVERIFIED
// on hardware.
#include <algorithm>
#include <cmath>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/motion.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

using detail::kAxisLog;
using detail::kPosEps;

constexpr int kConAsofModeItem = 14;   ///< CON 8.14: No.53 stop mode b8-10 (P5 / P6 bits on an EC)
/// CON 8.20 / 8.44 / 8.45: No.36 / No.37 / No.38 automatic servo OFF delay 1 / 2 / 3, s (No.53 stop mode 1 / 2 / 3)
constexpr int kConAsofDelayItems[] = {20, 44, 45};
constexpr double kPushStartSideMm = 0.005;  ///< a push start further into the zone than this is refused

std::string f2(double v) { return util::fixed(v, 2); }
std::string gs(double v) { return util::g(v); }

template <class T>
std::string shown(const std::optional<T>& v) {
    if (!v) return "None";
    if constexpr (std::is_same<T, bool>::value) return *v ? "True" : "False";
    if constexpr (std::is_same<T, double>::value) return util::g(*v);
    return std::to_string(*v);
}

/// A read refusal / absence (as in axis_motion.cpp): a value the unit does not give.
bool not_readable(const std::exception& e) {
    return dynamic_cast<const ModbusError*>(&e) || dynamic_cast<const std::out_of_range*>(&e) ||
           dynamic_cast<const std::invalid_argument*>(&e) || dynamic_cast<const EcNotReadyError*>(&e);
}

}  // namespace

// ============================================================================================ unit facts

std::optional<std::uint32_t> Axis::Impl::move_patterns() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (mvpt_known) return mvpt;
    }
    std::optional<std::uint32_t> value;
    try {
        const UnitDirectory dir = ensure_directory();
        if (!dir.empty()) {
            const auto words = dir.read_support_words(rd());
            if (words) {
                const auto it = words->find("MVPT");
                if (it != words->end()) value = it->second;
            }
        }
    } catch (const TransportError&) {
        throw;
    } catch (const std::exception& e) {
        if (!not_readable(e)) throw;
        log_info(kAxisLog, std::string("MVPT support word not readable: ") + e.what());
    }
    std::lock_guard<std::mutex> lock(mutex);
    mvpt_known = true;
    mvpt = value;
    return value;
}

std::optional<std::uint32_t> Axis::Impl::item_raw(int class_id, int item_id) {
    try {
        const ClassBlock block = read_class_block(class_id);
        if (const ClassItem* item = block.item(item_id)) return item->raw;
    } catch (const TransportError&) {
        throw;
    } catch (const std::exception& e) {
        if (!not_readable(e)) throw;
    }
    return std::nullopt;
}

double Axis::Impl::accel_only_cap() {
    const std::optional<AxisLimits> lim = limits_read();
    const AxisOptions o = opts();
    std::vector<double> caps;
    if (o.max_accel_g) caps.push_back(*o.max_accel_g);
    if (lim && lim->max_accel_g) caps.push_back(*lim->max_accel_g);
    return caps.empty() ? kFallbackAccelCapG : *std::min_element(caps.begin(), caps.end());
}

double Axis::Impl::decel_cap() {
    const std::optional<AxisLimits> lim = limits_read();
    const AxisOptions o = opts();
    std::vector<double> caps;
    if (o.max_accel_g) caps.push_back(*o.max_accel_g);
    if (lim && lim->max_decel_g) caps.push_back(*lim->max_decel_g);
    return caps.empty() ? kFallbackAccelCapG : *std::min_element(caps.begin(), caps.end());
}

// ============================================================================================ acceleration patterns

std::uint16_t Axis::Impl::accel_pattern_ctlf(std::optional<AccelPattern> pattern, double speed, double accel_g) {
    if (!pattern || *pattern == AccelPattern::Trapezoid) return 0;
    const std::uint32_t bit = *pattern == AccelPattern::SMotion ? reg::kMvptSMotion : reg::kMvptFilter;
    const std::optional<std::uint32_t> mv = move_patterns();
    if (mv && !(*mv & bit))
        throw SoftLimitError(util::format("accel_pattern '%s': this unit does not report it (MVPT b%d = 0); nothing sent",
                                          iai_actuator_driver::to_string(*pattern),
                                          bit == reg::kMvptSMotion ? 1 : 2));
    if (*pattern == AccelPattern::SMotion && accel_g > 0) {
        const double accel_time = speed / (accel_g * reg::kGmms2);
        if (accel_time > reg::kSMotionMaxAccelTimeS + 1e-9)
            throw SoftLimitError(util::format("S-motion is invalid for an acceleration time above %g s (this move: %.2f s; "
                                              "DOC PCON p.4-18): raise the acceleration or use 'trapezoid'; nothing sent",
                                              reg::kSMotionMaxAccelTimeS, accel_time));
    }
    return accel_pattern_bits(*pattern);
}

// ============================================================================================ command-profile move

Axis::Impl::ProfileMove Axis::Impl::check_profile_move(double position, double speed, double accel_g, double decel_g,
                                                       double band) {
    if (!std::isfinite(decel_g)) throw SoftLimitError("invalid deceleration " + gs(decel_g) + "; nothing sent");
    // the profile move starts with the PCRF coil: refused on a line / unit without FC05 (RCON; axis_strategy.cpp)
    if (no_coils()) throw line_refusal("fc05", "a move with its own deceleration (it starts with the PCRF coil 0x042E)");
    ProfileMove p;
    p.move = validate_motion(position, speed, accel_g, band, false, true);  // accel against MAXA only
    const std::int64_t dcmd = reg::to_counts(decel_g, reg::kAccelScale);
    const double dcap = decel_cap();
    if (dcmd < 1 || decel_g > dcap + 1e-9)
        throw SoftLimitError("deceleration " + gs(decel_g) + " G outside 0.01.." + gs(dcap) +
                             " G (MAXD 7.6 / max_accel_g); nothing written");
    const std::optional<int> address = block_address(reg::kActiveProfileCategory, reg::kProfileBase);
    if (!address)
        throw EcNotReadyError("this unit's directory has no command-profile block (class 0x0F); move without decel_g; "
                              "nothing sent");
    p.dcmd = static_cast<std::uint32_t>(dcmd);
    p.address = *address;
    return p;
}

Axis::Impl::MoveWatch Axis::Impl::send_profile(const ProfileMove& p, std::uint16_t ppow, std::uint16_t ctlf) {
    // SPOW / DPOW are kept from the unit's own block; LPOW, CTF2 and the zones are 0.
    const std::vector<std::uint16_t> current = bus->read_registers(slave, p.address, reg::kProfileWords);
    // target and band in the unit's own position counts: x10 on a 0.001 mm RCON unit (No.207; line.hpp)
    const int factor = bus->line()->wire_factor(slave);
    const auto pcmd = reg::to_i32_words(p.move.pcmd * factor);
    const auto vcmd = reg::to_u32_words(p.move.vcmd);
    const auto acmd = reg::to_u32_words(p.move.acmd);
    const auto dcmd = reg::to_u32_words(p.dcmd);
    const auto inp = reg::to_u32_words(p.move.inp * static_cast<std::uint32_t>(factor));
    const std::vector<std::uint16_t> words = {
        pcmd.first, pcmd.second, vcmd.first, vcmd.second, acmd.first, acmd.second, dcmd.first, dcmd.second,
        inp.first,  inp.second,  current[reg::kProfileSpow], current[reg::kProfileDpow], ppow, ctlf, 0, 0, 0, 0, 0, 0};
    release_stale_stop();
    motion_pending = true;
    {
        std::lock_guard<std::mutex> lock(mutex);
        last_move.reset();  // a profile move: change_speed / change_accel re-send 0x9900 blocks only
        last_target = ppow ? std::nullopt : std::optional<double>(reg::from_counts(p.move.pcmd));
    }
    bus->read_write_registers(slave, reg::kMonitorBase, reg::kProfileMonitorWords, p.address, words);
    const MoveWatch watch = watch_now();  // the PCRF edge starts the move
    bus->write_coil(slave, reg::kCoilPcrf, true);  // self-clearing: no OFF is sent
    return watch;
}

std::optional<Status> Axis::Impl::move_profile(double position, const MoveOptions& o) {
    const double decel = *o.decel_g;
    const ProfileMove p = check_profile_move(position, o.speed, o.accel_g, decel, o.band);
    const double slower = std::min(o.accel_g, decel);
    const std::uint16_t mod = accel_pattern_ctlf(o.accel_pattern, o.speed, slower);
    const Completion mode = completion_mode(o.completion);
    const Status st = require(pre_move_status());
    const double target = reg::from_counts(p.move.pcmd);
    const double timeout =
        o.timeout_s ? *o.timeout_s : detail::default_move_timeout_s(std::fabs(target - st.position_mm), o.speed, slower);
    return guard([&]() -> std::optional<Status> {
        const MoveWatch watch = send_profile(p, 0, mod);
        const double sent = now_s();
        note_move(target, o.band, timeout, mode, sent, watch);
        if (!o.wait) return std::nullopt;
        return wait_done(target, o.band, timeout, mode, sent, &watch);
    });
}

// ============================================================================================ push

std::pair<double, std::string> Axis::Impl::push_speed(std::optional<double> speed) {
    const std::optional<AxisLimits> lim = limits_read();
    std::vector<std::pair<double, std::string>> candidates;
    const std::optional<std::uint32_t> pshv = item_raw(reg::kMotionParamsCategory, reg::kParamItemPshv);
    if (pshv && *pshv > 0 && *pshv < 10000000u)
        candidates.emplace_back(util::round2(*pshv * reg::kVelocityScale), "PSHV 8.15");
    if (lim && lim->max_push_speed_mm_s && *lim->max_push_speed_mm_s > 0)
        candidates.emplace_back(*lim->max_push_speed_mm_s, "MXPH 7.8");
    if (candidates.empty())
        candidates.emplace_back(reg::kPushSpeedMaxMmS, "fallback 20: PSHV 8.15 / MXPH 7.8 not readable");
    if (speed) candidates.emplace_back(*speed, "the commanded speed");
    std::pair<double, std::string> best = candidates.front();
    for (const auto& c : candidates)
        if (c.first < best.first) best = c;
    return best;
}

std::pair<int, std::vector<std::string>> Axis::Impl::check_push_force(double force_pct) {
    const std::optional<AxisLimits> cached = limits_read();
    const AxisLimits lim = cached ? *cached : read_limits();
    if (lim.push_available() == std::optional<bool>(false))
        throw SoftLimitError("this unit cannot push (SUPT PUSH " + shown(lim.supports_push) + ", MXPP " +
                             shown(lim.max_push_pct) + ", TCPC " + shown(lim.push_force_factor) + "); nothing written");
    const double cap = push_cap();
    const bool over = force_pct > cap + 1e-9;
    const int ppow = over || force_pct < 0 ? 0 : reg::push_power(force_pct);  // 70 % -> 0xB2 (DOC ME0162-12A p.5-132)
    if (over || ppow < 1 || ppow > std::min(reg::kPpowRawMax, reg::push_power(cap))) {
        const std::optional<double> configured = opts().max_push_pct;
        const std::string source = lim.max_push_pct && (!configured || *lim.max_push_pct < *configured)
                                       ? "MXPP, item 7.9"
                                       : configured ? "max_push_pct" : "the fallback cap: MXPP not readable";
        throw SoftLimitError("push force " + gs(force_pct) + " % outside 1.." + gs(cap) + " % (" + source +
                             "); nothing written");
    }
    std::vector<std::string> warnings;
    if (force_pct < lim.min_push_pct - 1e-9) {
        const std::string where = lim.min_push_from_unit ? "MNPP, item 7.58" : "the default when MNPP is absent";
        warnings.push_back("push force " + gs(force_pct) + " % is below this unit's recommended minimum " +
                           gs(lim.min_push_pct) + " % (" + where + "): allowed, but the force may be inaccurate");
    }
    return {ppow, warnings};
}

std::vector<std::string> Axis::Impl::auto_servo_off_push_warnings() {
    std::optional<int> system;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!unit_code_known) return {};  // the identity not read yet: no FC11 just for a warning
        system = system_code;
    }
    if (!system || reg::is_ec_family(system)) return {};  // an EC: its delay register is not known
    const std::optional<std::uint32_t> mode = item_raw(reg::kMotionParamsCategory, kConAsofModeItem);
    if (!mode || !((*mode >> 8) & 0x7)) return {};
    // the stop mode selects the delay: 1 -> No.36, 2 -> No.37, 3 -> No.38 (ON-WIRE: mode 2 with No.37 = 5 s on a PCON-CB)
    const unsigned stop_mode = static_cast<unsigned>((*mode >> 8) & 0x7);
    std::string which = "the delay No.53 selects";
    std::string after;
    if (stop_mode >= 1 && stop_mode <= 3) {
        const std::optional<std::uint32_t> delay =
            item_raw(reg::kMotionParamsCategory, kConAsofDelayItems[stop_mode - 1]);
        which = "the No." + std::to_string(35 + stop_mode) + " delay";
        if (delay) after = " (" + std::to_string(*delay) + " s)";
    }
    return {util::format("automatic servo OFF is enabled (No.53 stop mode %u): after the push the servo switches off "
                         "once ",
                         stop_mode) +
            which + after + " has passed and the work is no longer held (DOC PCON p.5-5)"};
}

std::optional<PushResult> Axis::Impl::push(double target, double band, double force_pct, const PushOptions& o) {
    detail::check_timeout(o.timeout_s);
    const detail::SendGate gate(*bus, slave, decision_mark());  // a STOP from elsewhere after this refuses the push
    if (o.direction != 1 && o.direction != -1)
        throw SoftLimitError("direction must be +1 (away from home) or -1 (toward home), got " +
                             std::to_string(o.direction) + "; nothing sent");
    if (!std::isfinite(force_pct)) throw SoftLimitError("invalid push force " + gs(force_pct) + "; nothing sent");
    DirectMove move = validate_motion(target, o.speed, o.accel_g, band, true);
    const std::pair<int, std::vector<std::string>> force = check_push_force(force_pct);
    std::vector<std::string> warnings = force.second;
    const Span stroke = motion_stroke();
    const std::string u = units_label();
    const double end = target + o.direction * band;
    if (!(stroke.min - kPosEps <= end && end <= stroke.max + kPosEps))
        throw SoftLimitError("push band end " + f2(end) + " " + u + " outside the stroke " +
                             detail::fmt_range(stroke, u) + "; nothing sent");
    const Span zone{std::min(target, end), std::max(target, end)};
    const Status st = require(status(), true, true, false, true, [&](const Status& s) {  // before PMSL (require)
        if (!s.idle()) throw EcNotReadyError("push needs a stopped axis (wait for the current move to finish)");
        if ((s.position_mm - target) * o.direction > kPushStartSideMm)
            throw EcNotReadyError("start position " + f2(s.position_mm) + " " + u +
                                  " is inside or past the push zone " + detail::fmt_range(zone, u) +
                                  ": the unit would skip the approach and push from where it is. Move to the approach "
                                  "side first.");
    });
    const double start = st.position_mm;
    double timeout = 0.0;
    if (o.timeout_s) {
        timeout = *o.timeout_s;
    } else {
        const double pushing = push_speed(o.speed).first;
        timeout = 2.0 * detail::motion_time_s(std::fabs(target - start), o.speed, o.accel_g) +
                  2.0 * band / std::max(pushing, 0.01) + 3.0;
    }
    move.ppow = static_cast<std::uint16_t>(force.first);
    move.ctlf = static_cast<std::uint16_t>(reg::kCtlfPush | (o.direction > 0 ? reg::kCtlfPushAway : 0));
    for (const auto& w : auto_servo_off_push_warnings()) warnings.push_back(w);
    for (const auto& w : warnings) log_warning(kAxisLog, "push: " + w);
    return guard([&]() -> std::optional<PushResult> {
        const MoveWatch watch = send_direct(move);
        {
            std::lock_guard<std::mutex> lock(mutex);
            last_target.reset();  // a push ends where it meets the work
        }
        if (!o.wait) return std::nullopt;
        const Status done = wait_push(target, zone, timeout, &watch);
        PushResult result;
        result.outcome = done.push_missed() ? kPushMissed : kPushContact;
        result.position_mm = done.position_mm;
        result.status = done;
        result.warnings = warnings;
        return result;
    });
}

// ============================================================================================ mid-move changes

std::optional<Status> Axis::Impl::change_move(std::optional<double> speed, std::optional<double> accel, bool wait,
                                              std::optional<double> timeout_s) {
    detail::check_timeout(timeout_s);
    // before the running move is read: a STOP from elsewhere after this refuses the changed frame (modbus.hpp)
    const detail::SendGate gate(*bus, slave, decision_mark());
    std::optional<DirectMove> move;
    {
        std::lock_guard<std::mutex> lock(mutex);
        move = last_move;
    }
    const Status st = status();
    if (st.paused() && !st.moving && !st.homing() && !st.pushing())  // a pause at a standstill: name it, not the missing move
        throw EcNotReadyError(detail::pause_blocker(st, false) + "; nothing sent");
    if (!move || !st.moving || !motion_pending.load())
        throw EcNotReadyError("no 0x9900 move of this object is in progress (a push, a stored-row or a command-profile "
                              "move cannot be changed this way); nothing sent");
    if (st.paused()) throw EcNotReadyError("the move is paused (STP on): resume() first; nothing sent");
    if (move->ctlf & reg::kCtlfPush)
        throw EcNotReadyError("a push cannot be changed mid-move (the unit would restart it); nothing sent");
    const double new_speed = speed ? *speed : reg::from_counts(move->vcmd, reg::kVelocityScale);
    const double new_accel = accel ? *accel : reg::from_counts(move->acmd, reg::kAccelScale);
    const double target = reg::from_counts(move->pcmd);
    const double band = reg::from_counts(move->inp);
    DirectMove changed = validate_motion(target, new_speed, new_accel, band);
    changed.ppow = move->ppow;
    changed.ctlf = move->ctlf;
    std::vector<std::string> warnings = change_warnings(new_speed, new_accel);  // EC: P5 smooth acceleration
    if (!is_ec() && (move->ctlf & reg::kCtlfModMask) == reg::kCtlfModSMotion)
        warnings.push_back("an S-motion move changed while moving runs on as a trapezoid (DOC PCON p.4-18, 4-19)");
    for (const auto& w : warnings) log_warning(kAxisLog, "change of the running move: " + w);
    require(st);
    return guard([&]() -> std::optional<Status> {
        send_direct(changed);
        if (!wait) return std::nullopt;
        const double timeout = timeout_s ? *timeout_s
                                         : detail::default_move_timeout_s(std::fabs(target - st.position_mm), new_speed,
                                                                          new_accel);
        return wait_idle(target, band, timeout);
    });
}

// ============================================================================================ sequences

Status Axis::Impl::wait_band(double target, double band, double timeout_s, double since, const MoveWatch* watch) {
    const double deadline = now_s() + timeout_s;
    std::optional<double> away_since;
    Feed feed = make_feed(true, since);  // the lean block, or the monitor's samples from `since` on
    for (;;) {
        const Status st = next_status(feed);
        check_during_motion(st);
        if (std::fabs(st.position_mm - target) <= band + kPosEps) return st;
        if (watch && ended_from_outside(*watch, st, target, band)) return st;
        const double now = now_s();
        const std::string u = units_label();
        if (st.in_position && !st.moving) {
            if (!away_since) away_since = now;
            if (now - *away_since >= kSettledAwayS)
                stop_and_raise(MotionIncompleteError("axis at rest at " + f2(st.position_mm) + " " + u +
                                                     ", outside the blend band " + gs(band) + " of " + f2(target) +
                                                     " " + u));
        } else {
            away_since.reset();
        }
        if (now >= deadline)
            stop_and_raise(MotionTimeoutError("blend point " + f2(target) + " " + u +
                                              util::format(" not reached within %.1f s: position ", timeout_s) +
                                              f2(st.position_mm) + " " + u));
    }
}

std::vector<Status> Axis::Impl::move_sequence(const std::vector<SequencePoint>& points, const SequenceOptions& o) {
    detail::check_timeout(o.timeout_s);
    // one decision for the whole sequence: a STOP from elsewhere refuses every point not yet sent (modbus.hpp)
    const detail::SendGate gate(*bus, slave, decision_mark());
    if (o.blend_band) {
        if (!std::isfinite(*o.blend_band))
            throw SoftLimitError("invalid blend_band " + gs(*o.blend_band) + "; nothing sent");
        if (!(0.01 - kPosEps <= *o.blend_band && *o.blend_band <= kMoveBandMax))
            throw SoftLimitError("blend_band " + gs(*o.blend_band) + " outside 0.01.." + gs(kMoveBandMax) +
                                 "; nothing sent");
    }
    struct Planned {
        DirectMove move;
        double target;
        double band;
        double speed;
        double accel;
    };
    std::vector<Planned> plan;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const SequencePoint& p = points[i];
        const double speed = p.speed.value_or(o.speed);
        const double accel = p.accel_g.value_or(o.accel_g);
        const double band = p.band.value_or(o.band);
        try {
            const DirectMove m = validate_motion(p.position, speed, accel, band);
            plan.push_back({m, reg::from_counts(m.pcmd), band, speed, accel});
        } catch (SoftLimitError& e) {
            detail::add_note(&e, "point " + std::to_string(i) + " of the sequence; nothing sent");
            throw;
        }
    }
    if (plan.empty()) throw std::invalid_argument("move_sequence needs at least one point");
    const Completion mode = completion_mode(o.completion);
    Status st = require(pre_move_status());
    std::vector<Status> results;
    return guard([&]() -> std::vector<Status> {
        for (std::size_t i = 0; i < plan.size(); ++i) {
            const Planned& p = plan[i];
            if (i) st = require(st);
            const double timeout = o.timeout_s ? *o.timeout_s
                                               : detail::default_move_timeout_s(std::fabs(p.target - st.position_mm),
                                                                                p.speed, p.accel);
            const MoveWatch watch = send_direct(p.move);
            const double sent = now_s();
            if (!o.blend_band || i + 1 == plan.size())
                st = wait_done(p.target, p.band, timeout, mode, sent, &watch);
            else
                st = wait_band(p.target, *o.blend_band, timeout, sent, &watch);
            results.push_back(st);
        }
        return results;
    });
}

// ============================================================================================ JSON

Json to_json(const PushResult& r) {
    Json out = Json::object();
    out.set("outcome", r.outcome);
    out.set("position_mm", r.position_mm);
    out.set("status", to_json(r.status));
    out.set("warnings", r.warnings);
    return out;
}

// ============================================================================================ Axis: public forwarding

std::optional<PushResult> Axis::push(double target, double band, double force_pct, const PushOptions& options) {
    return impl_->push(target, band, force_pct, options);
}

std::pair<double, std::string> Axis::push_speed(std::optional<double> speed) { return impl_->push_speed(speed); }

std::optional<Status> Axis::change_accel(double accel_g, bool wait, std::optional<double> timeout_s) {
    return impl_->change_move(std::nullopt, accel_g, wait, timeout_s);
}

std::vector<Status> Axis::move_sequence(const std::vector<SequencePoint>& points, const SequenceOptions& options) {
    return impl_->move_sequence(points, options);
}

}  // namespace iai_actuator_driver
