// Axis motion: Modbus control at rest, servo (ServoBlockedError), home (coil / 0x45), alarm reset, check_move and the
// 0x9900 moves (checked before anything is sent), the safe completion rule, the STOP sequence (held until standstill,
// then released), pause / resume / cancel_paused_move, change_speed and the stored-row moves. Port of ec.py Elecylinder
// (servo 2970, home 3196, check_move / move_to / move_by / jog, move_to_position 3659, stop / begin_stop / pause /
// resume / cancel_paused_move / change_speed, wait_idle 4043 and the stop internals; C# Axis.Motion.cs). The behaviour is
// VERIFIED in the Python edition where its documentation says so; this edition is UNVERIFIED on hardware.
#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/bus_lock.hpp"
#include "iai_actuator_driver/cancel.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "bus_stop_timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

using detail::kAxisLog;
using detail::kPosEps;

constexpr double kPushCreepMm = 0.05;   ///< a holding contact may creep this much over the push settle
constexpr double kPushStartS = 1.0;     ///< a push row must visibly start within this
constexpr double kPushSpeedDefault = 20.0;  ///< push speed for the timeout estimate when MXPH is unknown
constexpr int kRejectHomingIai = reg::kRejectHoming;
constexpr int kRejectHomingFc48 = 0x1006;
constexpr std::uint32_t kEcSmoothAccelBit = 1u << 6;  ///< EC 8.14 b6 = P5 smooth acceleration (VERIFIED item mapping)
constexpr int kConPhspItem = 1;                       ///< CON 9.1 PHSP: No.29 b0-15 (ms), No.30 b17-18

std::string f2(double v) { return util::fixed(v, 2); }
std::string gs(double v) { return util::g(v); }
[[maybe_unused]] std::string tf(bool v) { return v ? "true" : "false"; }  // the wait moved to axis_fast.cpp

bool not_readable(const std::exception& e) {
    return dynamic_cast<const ModbusSlaveError*>(&e) || dynamic_cast<const std::out_of_range*>(&e) ||
           dynamic_cast<const std::invalid_argument*>(&e) || dynamic_cast<const EcNotReadyError*>(&e);
}

}  // namespace

void detail::check_timeout(const std::optional<double>& timeout_s, const std::string& what) {
    if (!timeout_s || (std::isfinite(*timeout_s) && *timeout_s > 0.0)) return;
    throw SoftLimitError(what + " " + gs(*timeout_s) +
                         " s: a timeout must be a finite number of seconds above 0 (none: the default); nothing sent");
}

// ============================================================================================ preconditions

Status Axis::Impl::require_control(Status st) {
    bus->line()->require_control(slave);  // gateway lines first (RCON: MANU; line.hpp); nothing sent when refused
    if (st.modbus_enabled) return st;
    if (!opts().auto_modbus_control)
        throw EcNotReadyError("Modbus control is off (PMSS = 0, as after every power-up); call enable_modbus_control() "
                              "while the axis is at rest");
    if (!detail::at_rest_for_control(st))
        throw EcNotReadyError(std::string("Modbus control is off (PMSS = 0) and the axis is moving: taking control now "
                                          "would stop the move. Wait until it is at rest (") +
                              detail::kPioStopHint + ")");
    // a STOP / group stop another thread holds: refused before PMSL, so the control mode never changes under that
    // stop (PIO -> PC would also clear a PLC's *STP pause); nothing sent
    refuse_while_stop_held_elsewhere();
    log_info(kAxisLog, "taking Modbus control (PMSL) while the axis is at rest");
    return *enable_control(true);
}

Status Axis::Impl::require(Status st, bool servo_needed, bool homed_needed, bool allow_paused, bool wake_asof) {
    if (st.emergency_stop) throw EcNotReadyError("emergency stop is active");
    if (st.alarm)
        throw EcAlarmError(st.alarm_code, util::format("alarm 0x%03x is active; call reset_alarm()", st.alarm_code));
    st = require_control(st);
    // an automatic servo OFF (STAT ASOF) is not a refusal: the servo goes back on first (axis_strategy.cpp)
    bool asof = servo_needed && !st.servo_on && st.auto_servo_off().value_or(false);
    if (asof && wake_asof) {
        st = wake_from_auto_servo_off(st);
        asof = false;
    }
    if (servo_needed && !st.servo_on && !asof) throw EcNotReadyError("servo is off; call servo(true)");
    if (st.homing()) throw EcNotReadyError("a home return is in progress; wait for it to finish");
    if (homed_needed && !st.homed) throw EcNotReadyError("axis is not homed; call home()");
    if (st.paused() && !allow_paused) throw EcNotReadyError("axis is paused (STP is on); call resume() first");
    return st;
}

void Axis::Impl::check_during_motion(const Status& st) {
    if (st.alarm) stop_and_raise(EcAlarmError(st.alarm_code, util::format("alarm 0x%03x during motion", st.alarm_code)));
    if (st.emergency_stop) stop_and_raise(EcNotReadyError("emergency stop during motion"));
    // an automatic servo OFF (STAT ASOF) comes only after a standstill: the move has ended
    if (!st.servo_on && !st.auto_servo_off().value_or(false))
        stop_and_raise(EcNotReadyError("servo turned off during motion"));
}

// ============================================================================================ the STOP sequence

void Axis::Impl::release_stop() {
    send_stop_coil(*bus, slave, false);
    stop_coil_on = false;
}

// ---- the stop runs: no thread releases a STOP another thread still holds

Axis::Impl::StopRun::StopRun(Impl& a) : a_(a) {
    std::lock_guard<std::mutex> lock(a_.stop_runs_mutex);
    a_.stop_runs.push_back(std::this_thread::get_id());
}

Axis::Impl::StopRun::~StopRun() { leave(); }

bool Axis::Impl::StopRun::leave() {
    std::lock_guard<std::mutex> lock(a_.stop_runs_mutex);
    if (!left_) {
        left_ = true;
        const auto it = std::find(a_.stop_runs.begin(), a_.stop_runs.end(), std::this_thread::get_id());
        if (it != a_.stop_runs.end()) a_.stop_runs.erase(it);
    }
    return a_.stop_runs.empty();
}

bool Axis::Impl::stop_running_elsewhere() const {
    std::lock_guard<std::mutex> lock(stop_runs_mutex);
    const std::thread::id me = std::this_thread::get_id();
    return std::any_of(stop_runs.begin(), stop_runs.end(), [&](const std::thread::id& t) { return t != me; });
}

void Axis::Impl::refuse_while_stop_held_elsewhere() const {
    if (!stop_held_elsewhere()) return;
    EcNotReadyError error("a STOP is held on this axis (a stop in progress in another thread): wait for it to end; "
                          "nothing sent");
    error.set_stop_attempted();  // that stop is the decel stop: no second one from guard()
    throw error;
}

void Axis::Impl::release_stale_stop() {
    // a STOP / pause held by a stop sequence of another thread (or by a group stop: Cell::stop_all, a coordinated
    // group's stop, between the first STOP frame and the axis's full stop) is that stop's to release, never this
    // command's
    refuse_while_stop_held_elsewhere();
    if (stop_coil_on.load()) release_stop();
    if (stop_pause_on.load()) {  // a register STOP's pause left ON: cancelled first, so the old travel never resumes
        cancel_pause(std::nullopt);
        stop_pause_on = false;
    }
}

std::pair<Status, bool> Axis::Impl::hold_until_stopped(double timeout_s) {
    const double deadline = now_s() + timeout_s;
    std::optional<Status> previous;
    detail::Settle settle(opts().settle_s);
    for (;;) {
        Status st = status();
        const bool steady = previous && !st.move_bit() &&
                            std::fabs(st.position_mm - previous->position_mm) < kSamePositionMm;
        if (!st.servo_on || settle.update(!st.moving || steady, st.position_mm)) {
            detail::StopTiming::standstill(this);  // the stop latency line of a stop in progress (bus_stop_timing.hpp)
            return {st, true};
        }
        if (now_s() >= deadline) return {st, false};
        previous = st;
        sleep_poll();
    }
}

std::pair<Status, bool> Axis::Impl::stop_sequence(double timeout_s) {
    StopRun run(*this);  // the last stop sequence to end releases the STOP
    note_stop();
    if (register_stop_path()) return register_stop_sequence(timeout_s, run);  // the coil-free substitute (axis_strategy.cpp)
    stop_coil_on = true;
    std::pair<Status, bool> result;
    bool refused = false;  // FC05 answered with exception 01: this unit / line has no coils
    try {
        try {
            send_stop_coil(*bus, slave, true);
        } catch (const ModbusSlaveError& e) {
            if (!coil_refused(e)) throw;
            refused = true;
            stop_coil_on = false;
        }
        if (!refused) result = hold_until_stopped(timeout_s);
    } catch (EcDriverError& e) {
        try {
            if (run.leave()) release_stop();
        } catch (const std::exception& r) {
            detail::add_note(&e, std::string("releasing the STOP coil failed too: ") + r.what());
        }
        throw;
    } catch (...) {
        try {
            if (run.leave()) release_stop();
        } catch (...) {
        }
        throw;
    }
    if (refused) {
        switch_to_register("FC05 refused with exception 01 (STOP)");
        return register_stop_sequence(timeout_s, run);
    }
    if (run.leave()) release_stop();  // else another thread's stop still holds it and releases it when it ends
    return result;
}

void Axis::Impl::try_stop(EcDriverError* error, const std::string& when) {
    const StopPriority stop_priority;  // first in the bus queue, never cancelled (bus_lock.hpp, cancel.hpp)
    const detail::StopTiming timing(*bus, slave, this, "decel stop", true, kAxisLog);
    if (error) error->set_stop_attempted();
    StopRun run(*this);  // the last stop sequence to end releases the STOP
    note_stop();
    if (register_stop_path()) {
        try_register_stop(error, when, run);
        return;
    }
    stop_coil_on = true;
    bool sent = false;
    try {
        send_stop_coil(*bus, slave, true);
        sent = true;
    } catch (const std::exception& e) {
        if (coil_refused(e)) {
            stop_coil_on = false;
            switch_to_register("FC05 refused with exception 01 (STOP)");
            try_register_stop(error, when, run);
            return;
        }
        detail::add_note(error, "decel stop " + when + " failed too: " + e.what());
    }
    if (sent) {
        detail::add_note(error, "decel stop sent " + when);
        try {
            hold_until_stopped(kErrorStopHoldS);
        } catch (const std::exception& e) {
            log_debug(kAxisLog, std::string("status unavailable while holding STOP: ") + e.what());
        }
    }
    try {
        if (run.leave()) release_stop();  // also when ON failed: the unit may have received it anyway
    } catch (const std::exception& e) {
        detail::add_note(error, std::string("STOP coil not released (") + e.what() + "); the next command retries it");
    }
}

Status Axis::Impl::stop(bool wait, double timeout_s) {
    const StopPriority stop_priority;  // first in the bus queue, never cancelled (bus_lock.hpp, cancel.hpp)
    std::optional<detail::StopTiming> timing;  // one line when the sequence ends, before a timeout is reported
    timing.emplace(*bus, slave, this, "STOP", true, kAxisLog);
    const auto result = stop_sequence(timeout_s);
    timing.reset();
    if (result.second) {
        motion_pending = false;
    } else if (wait) {
        throw MotionTimeoutError("axis still moving " + gs(timeout_s) + " s after STOP");
    } else {
        log_warning(kAxisLog, util::format("axis still moving %.1f s after STOP", timeout_s));
    }
    return result.first;
}

void Axis::Impl::begin_stop() {
    const StopPriority stop_priority;
    const detail::StopTiming timing(*bus, slave, this, "STOP", false, kAxisLog);
    note_stop();
    send_stop_first_frame();  // STOP coil ON, or the register path's DRG1 pause (axis_strategy.cpp)
}

void Axis::Impl::pause() {
    std::optional<DirectMove> move;
    {
        std::lock_guard<std::mutex> lock(mutex);
        move = last_move;
    }
    if (move && (move->ctlf & reg::kCtlfModMask) == reg::kCtlfModSMotion)
        log_warning(kAxisLog, "pause during an S-motion move: do not pause while it accelerates (DOC PCON p.4-19)");
    const StopPriority stop_priority;
    note_stop();
    set_pause(true);  // the STP coil, or DRG1 STP on the register path (axis_strategy.cpp)
}

void Axis::Impl::resume() {
    // STOP first: the stop mark before the checks, so a STOP from elsewhere after it refuses the STP OFF / DRG1 write
    // (nothing sent; modbus.hpp SendGate); and a STOP another thread still holds is never released by it
    const detail::SendGate gate(*bus, slave, decision_mark());
    refuse_while_stop_held_elsewhere();
    if (stop_pause_on.load()) {  // a register STOP whose cancel did not complete is never resumed: cancel + release
        release_stale_stop();
        return;
    }
    motion_pending = true;
    set_pause(false);
}

Status Axis::Impl::cancel_paused_move(double timeout_s) {
    const StopPriority stop_priority;
    const Status st = status();
    if (!st.paused()) throw EcNotReadyError("no pause is active (DSS1 STP = 0): pause() first, or use stop(); nothing sent");
    if (st.alarm)
        throw EcAlarmError(st.alarm_code,
                           util::format("alarm 0x%03x is active: the ALRS edge would also clear it, and the alarm has "
                                        "already ended the move. Call reset_alarm() explicitly, then resume(); nothing "
                                        "sent",
                                        st.alarm_code));
    if (!hold_until_stopped(timeout_s).second)
        log_warning(kAxisLog, util::format("paused axis still moving after %.1f s: cancelling anyway", timeout_s));
    // a rising ALRS edge while STP is held drops the remaining travel, then STP off (coil or DRG1; axis_strategy.cpp)
    cancel_pause(std::nullopt);
    motion_pending = false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        last_move.reset();
    }
    return status();
}

// ============================================================================================ mode / servo / alarm

std::vector<std::string> Axis::Impl::servo_blockers(const Status& st, bool after) {
    std::vector<std::string> reasons = st.servo_block_reasons();
    if (!after) return reasons;
    bool power = false;
    for (const auto& r : reasons)
        if (r.find("drive power") != std::string::npos) power = true;
    if (!power && st.drive_power_cut().value_or(false))
        reasons.push_back("the motor drive power is cut (DSSE MPUV / STAT MPOW = 0): check the drive cut-off circuit "
                          "and the external stop wiring (DOC ME0162-12A p.4-20, 4-21)");
    if (st.alarm) reasons.push_back(util::format("alarm 0x%03x ", st.alarm_code) + alarm_name(st.alarm_code));
    if (!st.modbus_enabled)
        reasons.push_back(st.manual_mode() == std::optional<bool>(false)
                              ? "the mode switch is in AUTO: the PIO has control (DSSE RMDS = 0, PMSS = 0; DOC "
                                "ME0162-12A p.4-20)"
                              : "Modbus control is off (DSSE PMSS = 0): the PIO has control");
    if (!st.excitation_done())
        reasons.push_back("the excitation detection of a first servo ON is still running (DSSE PSNS = 0; DOC PCON "
                          "p.4-24)");
    return reasons;
}

double Axis::Impl::servo_on_timeout(const Status& st, std::string* why) {
    if (why) *why = "default";
    if (st.excitation_done() || is_ec()) return kServoTimeoutS;
    std::optional<std::uint32_t> raw;
    try {
        const ClassBlock block = read_class_block(reg::kFactoryParamsCategory);
        if (const ClassItem* item = block.item(kConPhspItem)) raw = item->raw;
    } catch (const TransportError&) {
        throw;
    } catch (const std::exception& e) {
        if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
    }
    if (!raw) {  // No.29 / No.30 not readable: the longest case
        if (why) *why = "first servo ON after power-up; No.29 / No.30 not readable: the longest case";
        return kServoFirstOnMaxS;
    }
    const double no29_ms = static_cast<double>(*raw & 0xFFFF);
    const unsigned no30 = (*raw >> 17) & 0x3;
    const double t1 = no30 == 0 ? 0.16 : 0.22;
    const double first = 0.006 + (t1 + no29_ms / 1000.0) * 10.0 + 0.020;  // DOC PCON p.4-24
    if (why)
        *why = util::format("first servo ON after power-up: 0.006 + (%s + No.29 %d ms) x 10 + 0.020 s + 1 s (No.30 = %u; "
                            "DOC PCON p.4-24)",
                            no30 == 0 ? "0.16" : "0.22", static_cast<int>(*raw & 0xFFFF), no30);
    return std::max(kServoTimeoutS, std::round((first + 1.0) * 1000.0) / 1000.0);
}

std::optional<Status> Axis::Impl::servo(bool on, bool wait, std::optional<double> timeout_s) {
    detail::check_timeout(timeout_s);  // before anything is sent (either direction)
    std::optional<detail::SendGate> gate;  // servo ON: a STOP from elsewhere after this refuses it (modbus.hpp)
    if (on) gate.emplace(*bus, slave, decision_mark());
    std::optional<StopPriority> stop_priority;  // servo OFF is a stop path: first in the bus queue, never cancelled
    if (!on) stop_priority.emplace();
    const CancelScope cancel_scope(on ? cancel_token : nullptr);  // the servo ON wait is cancellable (cancel_waits)
    if (!on) note_stop();
    Status st = status();
    double timeout = 0.0;
    if (on) {
        require(st, false, false, true);
        const std::vector<std::string> reasons = servo_blockers(st, false);
        if (!reasons.empty()) throw ServoBlockedError("servo ON not sent: " + util::join(reasons, "; "), reasons);
        release_stale_stop();
        motion_pending = true;
        timeout = timeout_s ? *timeout_s : servo_on_timeout(st);
    } else {
        if (st.moving) {
            stop(true, 5.0);
            st = status();
        }
        if (!st.modbus_enabled && opts().auto_modbus_control && detail::at_rest_for_control(st)) enable_control(true);
        timeout = timeout_s.value_or(kServoTimeoutS);
    }
    // the strategy in use, adapted to the line (Register behind RCON; an FC05 exception 01 switches; axis_strategy.cpp)
    run_strategy(active_strategy(), "servo", [&](CommandMethod m) { send_servo(*bus, slave, on, m, &registers); });
    if (!wait) return std::nullopt;
    const double deadline = now_s() + timeout;
    for (;;) {
        st = status();
        if (st.servo_on == on) return st;
        if (st.alarm)
            throw EcAlarmError(st.alarm_code, util::format("alarm 0x%03x while switching servo", st.alarm_code));
        if (now_s() >= deadline) {
            if (!on) throw EcNotReadyError("servo did not turn off within " + gs(timeout) + " s");
            const std::vector<std::string> reasons = servo_blockers(st, true);
            const std::string why =
                reasons.empty() ? " (the status shows no known cause)" : ": " + util::join(reasons, "; ");
            throw ServoBlockedError("servo did not turn on within " + gs(timeout) + " s" + why, reasons);
        }
        sleep_poll();
    }
}

Status Axis::Impl::reset_alarm(std::optional<CommandMethod> method) {
    const CommandMethod strategy_ = strategy_for(method);  // an explicit method the line refuses: nothing sent
    const Status st = status();
    if (!st.modbus_enabled && opts().auto_modbus_control && detail::at_rest_for_control(st)) enable_control(true);
    run_strategy(strategy_, "alarm reset", [&](CommandMethod m) { send_alarm_reset(*bus, slave, m, &registers); });
    return status();
}

// ============================================================================================ home

std::optional<std::uint32_t> Axis::Impl::read_home_pattern() {
    const ClassBlock block = read_class_block(reg::kFactoryParamsCategory);
    const ClassItem* item = block.item(reg::kDriverItemOrg);
    if (!item) return std::nullopt;
    return item->raw;
}

void Axis::Impl::finish_home() {
    const StopPriority no_cancel;  // the HOME coil release is clean-up: never cancelled, never queued behind polls
    std::optional<CommandMethod> started;
    {
        std::lock_guard<std::mutex> lock(mutex);
        started = home_method;
    }
    const CommandMethod m = started ? *started : active_strategy();
    iai_actuator_driver::finish_home(*bus, slave, m, &registers);
}

bool Axis::Impl::home_seen_after_lost_reply(EcDriverError& error, const Status& before) {
    log_warning(kAxisLog, std::string("no valid answer to the home command (") + error.what() +
                              "): checking whether the home return started");
    const double deadline = now_s() + detail::home_start_s();
    for (;;) {
        const Status st = status();
        check_during_motion(st);
        if (st.homing() || st.moving || (before.homed && !st.homed) ||
            std::fabs(st.position_mm - before.position_mm) > kStartMovedMm) {
            log_warning(kAxisLog, "home return seen running although its command was not answered: waiting for it");
            return true;
        }
        if (now_s() >= deadline) {
            detail::add_note(&error, "the home command was not answered and no home return was seen within " +
                                         gs(detail::home_start_s()) + " s; the frame may still have reached the unit");
            return false;
        }
        sleep_poll();
    }
}

std::optional<Status> Axis::Impl::home(bool wait, std::optional<double> timeout_s, std::optional<CommandMethod> method) {
    detail::check_timeout(timeout_s);
    const detail::SendGate gate(*bus, slave, decision_mark());  // a STOP from elsewhere after this refuses it
    const CommandMethod m = strategy_for(method);  // adapted to the line; an explicit method it refuses: nothing sent
    const Status st = require(status(), true, false);
    std::optional<std::uint32_t> org;
    if (m == CommandMethod::Iai) {
        try {
            org = read_home_pattern();
        } catch (const TransportError&) {
            throw;
        } catch (const std::exception& e) {
            if (!not_readable(e)) throw;
        }
        if (!org || *org == 0 || *org == 0xFFFFFFFFu || (*org & 0xFFFF) == 0) {
            const std::string shown = org ? util::format("0x%08X", static_cast<unsigned>(*org)) : "not found";
            throw EcNotReadyError("0x45 home needs a valid home pattern ORG (item 9.16); this unit's is " + shown +
                                  " (op1 = 0 is rejected 0x11). Use the coil method. Nothing sent");
        }
    }
    double timeout = 0.0;
    if (timeout_s) {
        timeout = *timeout_s;
    } else {
        const std::optional<Span> eff = stroke_now();
        const double span = eff ? eff->max - eff->min : 300.0;
        timeout = 2.0 * span / 20.0 + 10.0;
    }
    release_stale_stop();
    motion_pending = true;
    {
        std::lock_guard<std::mutex> lock(mutex);
        home_method = m;
        last_move.reset();
        last_target.reset();
    }
    // HEND drops while homing and stays 0 when the home return is stopped: a deliberate loss of home, not a restart
    // sign for the link check (recovery.hpp; the next healthy check records HEND 1 again)
    recovery::hooks::note_target(bus, slave, std::nullopt);
    {
        recovery::ControlChange change;
        change.homed = false;
        recovery::hooks::note_control(bus, slave, change);
    }
    auto release_after = [this](EcDriverError* e) {
        try {
            finish_home();
        } catch (const std::exception& r) {
            detail::add_note(e, std::string("releasing the HOME coil failed too: ") + r.what());
        }
    };
    std::exception_ptr refused;
    bool seen_running = false;
    try {
        guard([&]() {
            try {
                const CommandMethod ran = run_strategy(m, "home", [&](CommandMethod s) {
                    iai_actuator_driver::start_home(*bus, slave, s, org, &registers);
                });
                if (ran != m) {
                    std::lock_guard<std::mutex> lock(mutex);
                    home_method = ran;  // finish_home releases what was started
                }
            } catch (IaiCommandRejected& e) {
                if (e.rejection() != kRejectHomingIai && e.rejection() != kRejectHomingFc48) {
                    refused = std::current_exception();  // answered with a refusal: this frame did not run
                    return;
                }
                e.add_note(util::format("the unit reports a home return already in progress (0x%x) that this call "
                                        "did not start; it is stopped (",
                                        e.rejection()) +
                           detail::kPioStopHint + ")");
                throw;  // the guard sends a decel stop
            } catch (const ModbusSlaveError&) {
                refused = std::current_exception();
            } catch (const UnsafeCommandError&) {
                refused = std::current_exception();
            } catch (const EcNotReadyError&) {
                refused = std::current_exception();
            } catch (ModbusTimeout& e) {
                if (!home_seen_after_lost_reply(e, st)) throw;
                seen_running = true;
            } catch (ModbusCrcError& e) {
                if (!home_seen_after_lost_reply(e, st)) throw;
                seen_running = true;
            } catch (ModbusFrameError& e) {
                if (!home_seen_after_lost_reply(e, st)) throw;
                seen_running = true;
            }
        });
    } catch (EcDriverError& e) {
        release_after(&e);
        throw;
    } catch (...) {
        release_after(nullptr);
        throw;
    }
    if (refused) {
        motion_pending = false;  // refused: nothing moves
        std::rethrow_exception(refused);
    }
    if (!wait) return std::nullopt;
    Status result;
    try {
        result = guard([&]() -> Status {
            const double start = now_s();
            bool seen_start = seen_running;
            detail::Settle settle(opts().settle_s);
            for (;;) {
                const Status s = status();
                check_during_motion(s);
                // The start shows in the home-return flags (GHMS, HEND dropped, MOVE). Polls too sparse to catch a
                // short home return (several axes polled in turn on one line, a fast simulator) still see its result:
                // HEND risen since the command, or the axis moved from where it was.
                seen_start = seen_start || s.homing() || !s.homed || s.move_bit() || (!st.homed && s.homed) ||
                             std::fabs(s.position_mm - st.position_mm) > kStartMovedMm;
                const bool home_end = s.homed && s.in_position && !s.homing();
                if (settle.update(seen_start && home_end, s.position_mm)) return s;
                const double elapsed = now_s() - start;
                if (!seen_start && elapsed >= detail::home_start_s()) {
                    // Nothing changed: HEND was 1 before and still is, and the axis stands at the home position (0)
                    // where it started. A home return that began and ended between two polls leaves exactly this, and
                    // so does a HOME ignored on an axis already at home: the axis is homed at its home position either
                    // way, so it is done (UNVERIFIED on hardware, where a home return outlasts many polls).
                    if (home_end && !s.moving && st.homed && std::fabs(s.position_mm) <= kStartMovedMm &&
                        std::fabs(st.position_mm) <= kStartMovedMm) {
                        log_warning(kAxisLog, "home return not seen between the polls; the axis reports home complete "
                                              "at the home position as before the command (HEND 1, PEND 1): taken as "
                                              "done");
                        return s;
                    }
                    stop_and_raise(EcNotReadyError("home return did not start within " + gs(detail::home_start_s()) +
                                                   " s (HOME ignored?)"));
                }
                if (elapsed >= timeout)
                    stop_and_raise(MotionTimeoutError(util::format("homing did not finish within %.1f s", timeout)));
                sleep_poll();
            }
        });
    } catch (EcDriverError& e) {
        release_after(&e);
        throw;
    } catch (...) {
        release_after(nullptr);
        throw;
    }
    finish_home();
    motion_pending = false;
    return result;
}

// ============================================================================================ checks of a move

void Axis::Impl::check_unit_speed(double speed) {
    const std::optional<AxisLimits> lim = limits_read();
    if (!lim) return;
    const auto cap = lim->unit_speed_cap();
    if (cap.first && speed > *cap.first + 1e-9) {
        const std::string u = units_label();
        throw SoftLimitError("speed " + gs(speed) + " " + u + "/s exceeds this EC's maximum speed " + f2(*cap.first) +
                             " " + u + "/s (" + cap.second + "); use <= " + gs(speed_cap()) + " " + u + "/s");
    }
}

void Axis::Impl::check_unit_accel(double accel, const std::string& what, bool decel) {
    const std::optional<AxisLimits> lim = limits_read();
    if (!lim) return;
    std::vector<std::pair<std::optional<double>, const char*>> caps{{lim->max_accel_g, "MAXA 7.5"}};
    if (decel) caps.emplace_back(lim->max_decel_g, "MAXD 7.6");
    for (const auto& c : caps) {
        if (c.first && accel > *c.first + 1e-9)
            throw SoftLimitError(what + " " + gs(accel) + " G exceeds this EC's maximum " + f2(*c.first) + " G (" +
                                 c.second + "); use <= " + gs(accel_cap()) + " G");
    }
}

DirectMove Axis::Impl::validate_motion(double position, double speed, double accel_g, double band, bool push_band,
                                       bool accel_only) {
    // the line's motion check first: RCON gateway in MANU and the position unit (No.207) known (line.hpp)
    bus->line()->require_motion(slave);
    const std::pair<const char*, double> values[] = {
        {"target", position}, {"speed", speed}, {"accel", accel_g}, {"band", band}};
    for (const auto& v : values)
        if (!std::isfinite(v.second)) throw SoftLimitError(std::string("invalid ") + v.first + " " + gs(v.second));
    // the configured caps and stroke first, from what is known (no read: a refused value sends nothing at all, as the
    // Python edition); the unit's limits are read by motion_stroke() below
    std::string u = units_label();
    const AxisOptions o = opts();
    const std::int64_t vcmd = reg::to_counts(speed, reg::kVelocityScale);
    if (o.max_speed_mm_s && speed > *o.max_speed_mm_s + 1e-9)
        throw SoftLimitError("speed " + gs(speed) + " " + u + "/s outside 0.01.." + gs(*o.max_speed_mm_s) + " " + u +
                             "/s (max_speed_mm_s); nothing sent");
    if (vcmd < 1)
        throw SoftLimitError("speed " + gs(speed) + " " + u + "/s outside 0.01.." +
                             gs(iai_actuator_driver::speed_cap(known_cap_inputs())) + " " + u + "/s; nothing sent");
    const std::int64_t acmd = reg::to_counts(accel_g, reg::kAccelScale);
    if (o.max_accel_g && accel_g > *o.max_accel_g + 1e-9)
        throw SoftLimitError("acceleration " + gs(accel_g) + " G outside 0.01.." + gs(*o.max_accel_g) +
                             " G (max_accel_g); nothing sent");
    if (acmd < 1 || acmd > reg::kAcmdMaxRaw)
        throw SoftLimitError("acceleration " + gs(accel_g) + " G outside 0.01.." +
                             gs(iai_actuator_driver::accel_cap(known_cap_inputs())) + " G; nothing sent");
    const std::int64_t inp = reg::to_counts(band);
    if (inp < 1 || (!push_band && band > kMoveBandMax))
        throw SoftLimitError("band " + gs(band) + " " + u + " outside 0.01 " + u + ".." +
                             (push_band ? std::string("the stroke") : gs(kMoveBandMax) + " " + u) + "; nothing sent");
    std::optional<double> min_band;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (limits) min_band = limits->min_band;
    }
    if (min_band && band < *min_band - kPosEps)
        throw SoftLimitError("band " + gs(band) + " " + u + " below this unit's minimum positioning band MINI (3.67) " +
                             gs(*min_band) + "; nothing sent");
    const std::int64_t pcmd = reg::to_counts(position);
    if (o.stroke && !(reg::to_counts(o.stroke->min) <= pcmd && pcmd <= reg::to_counts(o.stroke->max)))
        throw SoftLimitError("target " + gs(position) + " " + u + " outside the configured stroke " +
                             detail::fmt_range(o.stroke, u) + "; nothing sent");
    const Span stroke = motion_stroke();
    if (!(reg::to_counts(stroke.min) <= pcmd && pcmd <= reg::to_counts(stroke.max))) {
        std::optional<Span> soft_limits;
        {
            std::lock_guard<std::mutex> lock(mutex);
            soft_limits = soft.limits;
        }
        throw SoftLimitError("target " + gs(position) + " " + u + " outside the effective stroke " +
                             detail::fmt_range(stroke, u) + " (soft limits " + detail::fmt_range(soft_limits, u) +
                             "); nothing sent");
    }
    if (push_band && band > stroke.max - stroke.min)
        throw SoftLimitError("band " + gs(band) + " " + u + " longer than the stroke; nothing sent");
    check_unit_speed(speed);
    check_unit_accel(accel_g, accel_only ? "acceleration" : "acceleration / deceleration", !accel_only);
    u = units_label();
    const double scap = speed_cap();
    if (speed > scap + 1e-9)
        throw SoftLimitError("speed " + gs(speed) + " " + u + "/s above " + gs(scap) + " " + u +
                             "/s: this unit's maximum speed is not readable and no max_speed_mm_s is configured; "
                             "nothing sent");
    const double acap = accel_only ? accel_only_cap() : accel_cap();
    if (accel_g > acap + 1e-9)
        throw SoftLimitError((accel_only ? "acceleration " : "acceleration / deceleration ") + gs(accel_g) +
                             " G above " + gs(acap) +
                             (accel_only ? " G (min(max_accel_g, MAXA 7.5))"
                                         : " G (min(max_accel_g, MAXA 7.5, MAXD 7.6))") +
                             "; nothing sent");
    warn_min_speed(speed);
    DirectMove move;
    move.pcmd = static_cast<std::int32_t>(pcmd);
    move.inp = static_cast<std::uint32_t>(inp);
    move.vcmd = static_cast<std::uint32_t>(vcmd);
    move.acmd = static_cast<std::uint16_t>(acmd);
    move.ppow = 0;
    move.ctlf = reg::kCtlfAbsolute;
    return move;
}

void Axis::Impl::send_direct(const DirectMove& move) {
    // the caller's SendGate (taken before its checks) refuses the frame after a STOP from elsewhere; this one covers
    // a caller without one from here on
    const detail::SendGate decided(*bus, slave, decision_mark());
    release_stale_stop();
    // target and band in the unit's own position counts: x10 on a 0.001 mm RCON unit (No.207; line.hpp)
    DirectMove wire = move;
    const int factor = bus->line()->wire_factor(slave);
    wire.pcmd *= factor;
    wire.inp *= static_cast<std::uint32_t>(factor);
    // recorded once the frame went past the STOP-first gate (also when its answer was lost: it may have arrived), never
    // for a frame that was refused: no target that was never sent
    auto record = [&] {
        motion_pending = true;
        {
            std::lock_guard<std::mutex> lock(mutex);
            last_move = move;
            last_target = reg::from_counts(move.pcmd);
        }
        // the target recover() compares the position with (recovery.hpp); a push ends where it meets the work
        recovery::hooks::note_target(bus, slave,
                                     move.ppow ? std::nullopt : std::optional<double>(reg::from_counts(move.pcmd)));
    };
    const detail::SendGate sent(*bus, slave, decision_mark());  // only tells whether the frame went past the gate
    try {
        write_direct_move(*bus, slave, wire);
    } catch (...) {
        if (sent.passed() > 0) record();
        throw;
    }
    record();
}

// ============================================================================================ moves

Status Axis::Impl::check_move(double position, const MoveOptions& options_) {
    detail::check_timeout(options_.timeout_s);
    if (options_.decel_g && *options_.decel_g != options_.accel_g)  // the command-profile path (motion extras)
        check_profile_move(position, options_.speed, options_.accel_g, *options_.decel_g, options_.band);
    else
        validate_motion(position, options_.speed, options_.accel_g, options_.band);
    accel_pattern_ctlf(options_.accel_pattern, options_.speed, options_.accel_g);
    return require(status(), true, true, false, false);  // a check only: an automatic servo OFF is accepted, nothing sent
}

std::optional<Status> Axis::Impl::move_to(double position, const MoveOptions& o) {
    detail::check_timeout(o.timeout_s);  // also for the command-profile move
    const detail::SendGate gate(*bus, slave, decision_mark());  // a STOP from elsewhere after this refuses the move
    if (o.decel_g && *o.decel_g != o.accel_g) return move_profile(position, o);  // motion extras
    DirectMove move = validate_motion(position, o.speed, o.accel_g, o.band);
    move.ctlf = static_cast<std::uint16_t>(move.ctlf | accel_pattern_ctlf(o.accel_pattern, o.speed, o.accel_g));
    const Completion mode = completion_mode(o.completion);
    const Status st = require(pre_move_status());  // a fresh monitor status, else a read (axis_fast.cpp)
    const double target = reg::from_counts(move.pcmd);
    const double timeout =
        o.timeout_s ? *o.timeout_s : detail::default_move_timeout_s(std::fabs(target - st.position_mm), o.speed, o.accel_g);
    return guard([&]() -> std::optional<Status> {
        send_direct(move);
        const double sent = now_s();
        note_move(target, o.band, timeout, mode, sent);
        if (!o.wait) return std::nullopt;
        return wait_done(target, o.band, timeout, mode, sent);
    });
}

std::optional<Status> Axis::Impl::move_by(double delta, const MoveOptions& o) {
    detail::check_timeout(o.timeout_s);
    const detail::SendGate gate(*bus, slave, decision_mark());  // before the position it adds to is read
    if (!std::isfinite(delta)) throw SoftLimitError("invalid distance " + gs(delta) + "; nothing sent");
    const double target = util::round2(measured_status().position_mm + delta);
    return move_to(target, o);
}

Status Axis::Impl::wait_idle(std::optional<double> target, std::optional<double> band, double timeout_s) {
    // the rule of the axis (AxisOptions::completion); the loop, the safe and the fast rule are in axis_fast.cpp
    return wait_done(target, band, timeout_s, completion_mode(std::nullopt));
}

// ============================================================================================ change_speed

std::vector<std::string> Axis::Impl::change_warnings(double speed, double accel) {
    std::vector<std::string> warnings;
    if (!is_ec()) return warnings;
    bool known;
    std::optional<std::uint32_t> ctlf;
    {
        std::lock_guard<std::mutex> lock(mutex);
        known = ctlf_item_known;
        ctlf = ctlf_item;
    }
    if (!known) {
        try {
            const ClassBlock block = read_class_block(reg::kMotionParamsCategory);
            if (const ClassItem* item = block.item(reg::kParamItemCtlf)) ctlf = item->raw;
        } catch (const TransportError&) {
            throw;
        } catch (const std::exception& e) {
            if (!not_readable(e) && !dynamic_cast<const ModbusError*>(&e)) throw;
        }
        std::lock_guard<std::mutex> lock(mutex);
        ctlf_item_known = true;
        ctlf_item = ctlf;
    }
    if (ctlf && (*ctlf & kEcSmoothAccelBit)) {
        const double accel_time = accel > 0 ? speed / (accel * reg::kGmms2) : HUGE_VAL;
        if (accel_time > detail::kSmoothAccelMaxTimeS + 1e-9)
            throw SoftLimitError(util::format("P5 smooth acceleration is on: accel / decel time %.2f s above %g s is not "
                                              "allowed (DOC ME3816-6B p.6-11); nothing sent",
                                              accel_time, detail::kSmoothAccelMaxTimeS));
        warnings.push_back("P5 smooth acceleration is on: a command issued too soon during a move can raise an "
                           "overload alarm (DOC ME3816-6B p.8-21 cause 7)");
    }
    return warnings;
}

std::optional<Status> Axis::Impl::change_speed(double speed, bool wait, std::optional<double> timeout_s) {
    return change_move(speed, std::nullopt, wait, timeout_s);  // motion extras (also change_accel, the CON warning)
}

// ============================================================================================ stored rows

Axis::Impl::StoredRow Axis::Impl::check_stored_row(int row, const PositionEntry& entry, bool allow_above_caps,
                                                    double speed_scale) {
    if (entry.checksum_ok() == std::optional<bool>(false))
        throw SoftLimitError("stored position " + std::to_string(row) +
                             ": its checksum word +0xF does not match the row's data; not run (re-read it, or restore "
                             "the row from the IA-OS backup); nothing sent");
    const Span stroke = motion_stroke();
    const std::string u = units_label();
    std::optional<Span> soft_limits;
    {
        std::lock_guard<std::mutex> lock(mutex);
        soft_limits = soft.limits;
    }
    StoredRow out;
    out.target = entry.position_mm;
    out.band = entry.band_mm;
    const std::int64_t t = reg::to_counts(out.target);
    if (!(reg::to_counts(stroke.min) <= t && t <= reg::to_counts(stroke.max)))
        throw SoftLimitError("stored position " + std::to_string(row) + " is " + f2(out.target) + " " + u +
                             ", outside the effective stroke " + detail::fmt_range(stroke, u) + " (soft limits " +
                             detail::fmt_range(soft_limits, u) + "); nothing sent");
    if (!(out.band > 0 && out.band <= stroke.max - stroke.min))
        throw SoftLimitError("stored position " + std::to_string(row) + ": band " + gs(out.band) + " " + u +
                             " out of range; nothing sent");
    const std::optional<AxisLimits> lim = limits_read();
    const AxisOptions o = opts();
    std::optional<double> unit_max;
    std::string unit_max_note;
    if (lim) {
        const auto cap = lim->unit_speed_cap();
        unit_max = cap.first;
        unit_max_note = cap.second;
    }
    const double cap = allow_above_caps ? unit_max.value_or(HUGE_VAL) : speed_cap();
    const double run_speed = util::round2(entry.speed_mm_s * speed_scale);
    if (!(run_speed > 0 && run_speed <= cap + kPosEps)) {
        const std::string shown = unit_max ? f2(*unit_max) + " " + u + "/s" : "not readable";
        const std::string configured = o.max_speed_mm_s ? gs(*o.max_speed_mm_s) : "none";
        const std::string how = speed_scale == 1.0
                                    ? "its stored speed"
                                    : util::format("%.2f %% of its stored speed", speed_scale * 100.0);
        std::string text = "stored position " + std::to_string(row) + " runs at " + how + " " + f2(run_speed) + " " +
                           u + "/s, above ";
        text += allow_above_caps ? std::string("the unit's own maximum")
                                 : "the speed cap " + gs(cap) + " " + u + "/s (min(max_speed_mm_s " + configured +
                                       ", the unit's maximum " + shown + "))";
        text += "; nothing sent.";
        if (!allow_above_caps)
            text += " Pass allow_above_caps to run the row at its own stored speed / acceleration (still within the "
                    "unit's own maximum), or use move_to() at a lower speed";
        throw SoftLimitError(text);
    }
    const double acmd_max = reg::kAcmdMaxRaw * reg::kAccelScale;
    const double unit_accel = lim && lim->max_accel_g ? *lim->max_accel_g : acmd_max;
    const double unit_decel = lim && lim->max_decel_g ? *lim->max_decel_g : acmd_max;
    const double configured_accel = o.max_accel_g.value_or(HUGE_VAL);
    const double accel_cap_ = allow_above_caps ? unit_accel : std::min(configured_accel, unit_accel);
    const double decel_cap_ = allow_above_caps ? unit_decel : std::min(configured_accel, unit_decel);
    if (!(std::min(entry.accel_g, entry.decel_g) > 0) || entry.accel_g > accel_cap_ + 1e-9 ||
        entry.decel_g > decel_cap_ + 1e-9)
        throw SoftLimitError("stored position " + std::to_string(row) + ": accel / decel " + f2(entry.accel_g) + " / " +
                             f2(entry.decel_g) + " G outside 0.." + gs(accel_cap_) + " / " + gs(decel_cap_) + " G" +
                             (allow_above_caps ? " (the unit's MAXA / MAXD, items 7.5 / 7.6)"
                                               : " (max_accel_g and the unit's MAXA / MAXD; allow_above_caps runs the "
                                                 "row's own values)") +
                             "; nothing sent");
    if (entry.incremental())
        throw SoftLimitError("stored position " + std::to_string(row) +
                             " is an incremental row (CTLF bit 3): its end point depends on where the axis is, so it "
                             "is not run; nothing sent");
    if (!entry.is_push()) return out;
    out.direction = (entry.control_flags & reg::kCtlfPushAway) ? 1 : -1;
    const double end = out.target + out.direction * out.band;
    if (!(stroke.min - kPosEps <= end && end <= stroke.max + kPosEps))
        throw SoftLimitError("stored position " + std::to_string(row) + " pushes to " + f2(end) + " " + u +
                             ", outside the effective stroke " + detail::fmt_range(stroke, u) + "; nothing sent");
    // The row's own force is the unit's configuration: only the upper caps are enforced, in PPOW counts:
    // floor(cap x 255 / 100), so a 70 % cap admits 0xB2 and refuses 0xB3 (DOC ME0162-12A p.5-132).
    const double force = entry.push_power * 100.0 / 255.0;
    const std::optional<double> mxpp = lim ? lim->max_push_pct : std::nullopt;
    const double push_cap_ = allow_above_caps ? mxpp.value_or(100.0) : push_cap();
    if (!(entry.push_power > 0 && entry.push_power <= std::min(reg::kPpowRawMax, reg::push_power(push_cap_))))
        throw SoftLimitError("stored position " + std::to_string(row) + util::format(": push force %.0f %% (PPOW %d)", force, entry.push_power) +
                             " outside 1.." + gs(push_cap_) + " % " +
                             (allow_above_caps ? "(the EC's MXPP, item 7.9)" : "(min(max_push_pct, the EC's MXPP))") +
                             "; nothing sent");
    out.zone = Span{std::min(out.target, end), std::max(out.target, end)};
    return out;
}

void Axis::Impl::wait_stored_start(int row, double start, double target) {
    const double deadline = now_s() + kStoredStartS;
    Status st;
    for (;;) {
        st = status();
        check_during_motion(st);
        if (st.moving || st.move_bit() || std::fabs(st.position_mm - start) > kStartMovedMm) return;
        if (now_s() >= deadline) break;
        sleep_poll();
    }
    const std::string u = units_label();
    const std::string n = std::to_string(row);
    const std::string where = "the axis stayed at " + f2(st.position_mm) + " " + u + " (target " + f2(target) + " " + u + ")";
    if (!st.modbus_enabled)
        stop_and_raise(EcNotReadyError("stored position " + n + " (0x9800 = " + n +
                                       ") was not executed: Modbus control is off (PMSS = 0) and " + detail::kN4Note +
                                       "; " + where +
                                       ". Something handed control back to PIO; call enable_modbus_control() at rest "
                                       "and retry"));
    stop_and_raise(MotionIncompleteError("stored position " + n + " (0x9800 = " + n + ") was ignored: no motion within " +
                                         gs(kStoredStartS) + " s with PMSS = 1; " + where +
                                         ". Possible causes: a STOP or pause held by another master, the unit's "
                                         "Monitor mode, or a firmware that does not run 0x9800"));
}

Status Axis::Impl::wait_push(double target, const Span& zone, double timeout_s) {
    const double start = now_s();
    bool seen_start = false;
    detail::Settle settle(opts().push_settle_s, kPushCreepMm);  // hardware test G5 (kPushSettleSDefault)
    for (;;) {
        const Status st = status();
        check_during_motion(st);
        seen_start = seen_start || st.move_bit() || (!st.in_position && !st.push_missed());
        const double elapsed = now_s() - start;
        int outcome = 0;  // 1 missed, 2 contact
        if (seen_start || elapsed >= kPushStartS) {
            if (st.push_missed() && !st.moving)
                outcome = 1;
            else if (st.push_contact() && !st.push_missed())
                outcome = 2;
            else if (!seen_start)
                stop_and_raise(MotionIncompleteError("push did not start within " + gs(kPushStartS) + " s"));
        }
        if (settle.update(outcome != 0, st.position_mm, outcome)) {
            if (outcome == 1) motion_pending = false;  // contact: still pushing until the next command
            return st;
        }
        if (elapsed >= timeout_s) {
            const std::string u = units_label();
            stop_and_raise(MotionTimeoutError("push (approach " + f2(target) + " " + u + ", zone " +
                                              detail::fmt_range(zone, u) + util::format(") not finished after %.1f s", timeout_s) +
                                              ": position " + f2(st.position_mm) + " " + u));
        }
        sleep_poll();
    }
}

std::optional<Status> Axis::Impl::move_to_position(int row, const PositionMoveOptions& o) {
    detail::check_timeout(o.timeout_s);
    const detail::SendGate gate(*bus, slave, decision_mark());  // a STOP from elsewhere after this refuses the start
    const int rows = max_positions();
    if (row < 0 || row >= rows)
        throw SoftLimitError("stored position must be 0.." + std::to_string(rows - 1) +
                             " (this unit's stored rows: MAXP 3.2 / the directory), got " + std::to_string(row) +
                             "; nothing sent");
    check_active_row(row, "nothing sent");  // an EC runs rows 0 and 1 only in 2-position mode (axis_config.cpp)
    std::optional<int> override_code;
    if (o.speed_pct) {
        bus->line()->require(LineOperation::IaiCommand, "move_to_position(speed_pct) (0x45 cmd 6)");  // line.hpp
        if (!std::isfinite(*o.speed_pct) || !(*o.speed_pct > 0 && *o.speed_pct <= 100))
            throw SoftLimitError("speed_pct must be in (0, 100], got " + gs(*o.speed_pct) + "; nothing sent");
        override_code = speed_override_code(*o.speed_pct);  // the unit runs (code + 1) / 256 (commands.hpp)
        const double runs = speed_override_pct(*override_code);
        if (std::fabs(runs - *o.speed_pct) > 0.05 * *o.speed_pct)
            log_warning(kAxisLog, util::format("speed_pct %s %%: stored position %d runs at %.2f %% of its stored "
                                               "speed (override code %d; the unit runs (code + 1) / 256)",
                                               gs(*o.speed_pct).c_str(), row, runs, *override_code));
    }
    const PositionEntry entry = read_row(row);
    const double scale = override_code ? speed_override_pct(*override_code) / 100.0 : 1.0;
    const StoredRow checked = check_stored_row(row, entry, o.allow_above_caps, scale);
    Status st = status();
    if (!st.modbus_enabled && !opts().auto_modbus_control)
        throw EcNotReadyError(std::string("Modbus control is off (PMSS = 0, as after every power-up) and ") +
                              detail::kN4Note + "; call enable_modbus_control() while the axis is at rest");
    st = require(st);  // takes Modbus control here, at rest only
    const std::string u = units_label();
    if (!st.idle())
        throw EcNotReadyError("stored position " + std::to_string(row) +
                              ": 0x9800 is only sent from rest (the axis is moving); wait for the current move to "
                              "finish or stop() it");
    const double start = st.position_mm;
    if (checked.zone && (start - checked.target) * checked.direction > 0.005)
        throw EcNotReadyError("stored position " + std::to_string(row) + " is a push row (approach " +
                              f2(checked.target) + " " + u + ", push zone " + detail::fmt_range(checked.zone, u) +
                              ") and the axis at " + f2(start) + " " + u +
                              " is inside or past the zone: the unit would skip the approach and push from here. Move "
                              "to the approach side first (e.g. the other stored position).");
    const double accel = std::min(entry.accel_g, entry.decel_g);
    const double run_speed = std::max(entry.speed_mm_s * scale, 0.01);
    double timeout = 0.0;
    if (o.timeout_s) {
        timeout = *o.timeout_s;
    } else {
        timeout = detail::default_move_timeout_s(std::fabs(checked.target - start), run_speed, accel);
        if (checked.zone) {
            const std::optional<AxisLimits> lim = limits_read();
            const double push_speed =
                std::min(run_speed, lim && lim->max_push_speed_mm_s && *lim->max_push_speed_mm_s > 0
                                        ? *lim->max_push_speed_mm_s
                                        : kPushSpeedDefault);
            timeout += 2.0 * checked.band / std::max(push_speed, 0.01) + 1.0;
        }
    }
    const bool needs_start = checked.zone || std::fabs(start - checked.target) > checked.band + kPosEps;
    return guard([&]() -> std::optional<Status> {
        release_stale_stop();
        motion_pending = true;
        {
            std::lock_guard<std::mutex> lock(mutex);
            last_move.reset();  // a stored row: change_speed re-sends 0x9900 blocks only
            last_target = checked.zone ? std::nullopt : std::optional<double>(checked.target);
        }
        recovery::hooks::note_target(bus, slave, checked.zone ? std::nullopt : std::optional<double>(checked.target));
        if (!override_code) {
            send_position_move(*bus, slave, row);
        } else {
            try {
                send_position_move_override(*bus, slave, row, *o.speed_pct);
            } catch (const IaiCommandRejected& e) {
                // 0x11: the unit does not run this row in its position mode (the rejection stays nested)
                if (e.rejection() != detail::kRejectRowOperand) throw;
                std::throw_with_nested(row_rejected(row, e));
            }
        }
        if (needs_start) wait_stored_start(row, start, checked.target);
        if (!o.wait) return std::nullopt;
        if (checked.zone) return wait_push(checked.target, *checked.zone, timeout);
        return wait_idle(checked.target, checked.band, timeout);
    });
}

// ============================================================================================ Axis: public forwarding

std::optional<Status> Axis::servo(bool on, bool wait, std::optional<double> timeout_s) {
    return impl_->servo(on, wait, timeout_s);
}

std::optional<Status> Axis::home(bool wait, std::optional<double> timeout_s, std::optional<CommandMethod> method) {
    return impl_->home(wait, timeout_s, method);
}

Status Axis::reset_alarm(std::optional<CommandMethod> method) { return impl_->reset_alarm(method); }

Status Axis::check_move(double position, const MoveOptions& options) { return impl_->check_move(position, options); }

std::optional<Status> Axis::move_to(double position, const MoveOptions& options) {
    return impl_->move_to(position, options);
}

std::optional<Status> Axis::move_to(double position, double speed, double accel_g, double band) {
    MoveOptions o;
    o.speed = speed;
    o.accel_g = accel_g;
    o.band = band;
    return impl_->move_to(position, o);
}

std::optional<Status> Axis::move_by(double delta, const MoveOptions& options) { return impl_->move_by(delta, options); }

std::optional<Status> Axis::jog(double delta, const MoveOptions& options) { return impl_->move_by(delta, options); }

std::optional<Status> Axis::move_to_position(int row, const PositionMoveOptions& options) {
    return impl_->move_to_position(row, options);
}

Status Axis::wait_idle(std::optional<double> target, std::optional<double> band, double timeout_s) {
    return impl_->wait_idle(target, band, timeout_s);
}

Status Axis::stop(bool wait, double timeout_s) { return impl_->stop(wait, timeout_s); }
void Axis::begin_stop() { impl_->begin_stop(); }
void Axis::pause() { impl_->pause(); }
void Axis::cancel_waits() { impl_->cancel_token->cancel(); }
void Axis::set_cancel_latch(bool on) { impl_->cancel_token->set_latch(on); }
bool Axis::cancel_latched() const { return impl_->cancel_token->latched(); }
void Axis::resume() { impl_->resume(); }
Status Axis::cancel_paused_move(double timeout_s) { return impl_->cancel_paused_move(timeout_s); }

std::optional<Status> Axis::change_speed(double speed, bool wait, std::optional<double> timeout_s) {
    return impl_->change_speed(speed, wait, timeout_s);
}

}  // namespace iai_actuator_driver
