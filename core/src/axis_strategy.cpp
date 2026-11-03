// Axis command strategies: how servo / home / alarm reset reach the unit (coil, IAI 0x45, IAI 0x48 or FC06 DRG1), the
// line's say in it (an RCON gateway carries no FC05 coils and no IAI commands), the automatic switch to the register
// path when the unit answers an FC05 write with exception 01, and the coil-free STOP substitute (pause, wait for
// standstill, ALRS edge while paused, release). Also the action mode (PIO: 0x45 cmd 0x0C; PC: the PMSL / SFTY coils),
// the safety speed (SFTY coil or DRG1 SFTY) and the experimental teach with the MOD / TEAC coils. Port of the Python
// edition's ec.py (CommandStrategy and its subclasses, Elecylinder._active_strategy / _run_strategy /
// _switch_to_register / stop_path, _register_stop_sequence / _try_register_stop / _cancel_pause / _set_pause,
// set_action_mode, set_safety_speed, _teach_by_coils). The register path is DOC (ME0162-12A p.5-124 to 5-128; RCON
// ME0413 p.5-11, 5-13; ME0384 p.6-76) and UNVERIFIED on hardware in both editions. The action mode's PIO form (0x45 cmd
// 0x0C op1 = 0) and the SFTY coil are VERIFIED in the Python edition; its PC form by the PMSL / SFTY coils in
// set_action_mode ran on the simulator, and the PCON check of it passed on the PCON-CB (2026-11-02: the PMSL coil from
// PIO, servo ON taken; the safety speed stayed off, so the SFTY coil there is UNVERIFIED in this form). The rest of
// this C++ code is UNVERIFIED on hardware.
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "axis_impl.hpp"
#include "iai_actuator_driver/commands.hpp"
#include "iai_actuator_driver/line.hpp"
#include "iai_actuator_driver/recovery.hpp"
#include "iai_actuator_driver/registers.hpp"
#include "util.hpp"

namespace iai_actuator_driver {

namespace {

using detail::kAxisLog;

/// What a line refusal says the line does not carry.
std::string carried_text(const std::string& what) {
    if (what == "fc05") return "FC05 coils";
    if (what == "iai") return "IAI 0x45 / 0x48 commands";
    if (what == "broadcast") return "broadcast frames";
    if (what == "slave_id") return "Report Slave ID (FC11)";
    return what;
}

}  // namespace

// ============================================================================================ the strategy in use

CommandMethod Axis::Impl::current_strategy() const {
    const std::optional<std::string> wanted = bus->line()->command_strategy();  // before the axis mutex
    std::lock_guard<std::mutex> lock(mutex);
    if (strategy) return *strategy;
    if (options.method != CommandMethod::Coil) return options.method;
    if (wanted) {
        if (const std::optional<CommandMethod> m = command_method_from_string(*wanted)) return *m;
    }
    return CommandMethod::Coil;
}

void Axis::Impl::set_strategy(CommandMethod method) {
    if (method == CommandMethod::Iai2)
        log_warning(kAxisLog, "command strategy 'iai2' is EXPERIMENTAL (IAI 0x48, untested on any unit)");
    std::lock_guard<std::mutex> lock(mutex);
    strategy = method;
}

std::optional<std::string> Axis::Impl::line_name() const {
    return bus->line()->axis_name();  // the kind, or the name stated for the axes (Line::state_rules)
}

LineRuleError Axis::Impl::line_refusal(const std::string& what, const std::string& action) const {
    return LineRuleError(action + ": " + line_name().value_or("this line") + " does not carry " + carried_text(what) +
                         " (the line rules); nothing sent");
}

bool Axis::Impl::no_coils() const {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (coils_refused) return true;
    }
    if (!bus->line()->allows(LineOperation::Coil)) return true;
    return bus->unsupported_functions(slave).count(kFcWriteCoil) > 0;  // exception 01 remembered by the bus
}

bool Axis::Impl::register_stop_path() const {
    return current_strategy() == CommandMethod::Register || no_coils();
}

bool Axis::Impl::coil_refused(const std::exception& e) {
    const auto* refused = dynamic_cast<const ModbusSlaveError*>(&e);
    return refused != nullptr && refused->code() == 0x01 && refused->function() == kFcWriteCoil;
}

void Axis::Impl::switch_to_register(const std::string& why) {
    const CommandMethod before = current_strategy();
    {
        std::lock_guard<std::mutex> lock(mutex);
        coils_refused = true;
        if (uses_coils(before)) strategy = CommandMethod::Register;
    }
    if (uses_coils(before))
        log_warning(kAxisLog, why + ": switching to the 'register' command strategy (FC06 DRG1, no coils)");
    else
        log_warning(kAxisLog, why + ": STOP / pause / safety speed use FC06 DRG1 from now on");
}

CommandMethod Axis::Impl::active_strategy(std::optional<CommandMethod> requested, bool explicit_) {
    const CommandMethod configured = current_strategy();
    CommandMethod method = requested.value_or(configured);
    const bool is_configured = !requested || *requested == configured;
    const std::shared_ptr<Line> line = bus->line();
    const LineOperation iai_op = method == CommandMethod::Iai2 ? LineOperation::IaiCommand2 : LineOperation::IaiCommand;
    if (uses_iai(method) && !line->allows(iai_op)) {
        if (explicit_) throw line_refusal("iai", std::string("method '") + iai_actuator_driver::to_string(method) + "'");
        log_info(kAxisLog, std::string("strategy '") + iai_actuator_driver::to_string(method) +
                               "': this line carries no IAI commands; using the coil / register path");
        if (is_configured) {
            std::lock_guard<std::mutex> lock(mutex);
            strategy = CommandMethod::Coil;
        }
        method = CommandMethod::Coil;
    }
    if (uses_coils(method) && no_coils()) {
        bool refused = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            refused = coils_refused;
        }
        if (explicit_ && !refused) throw line_refusal("fc05", std::string("method '") + iai_actuator_driver::to_string(method) + "'");
        if (is_configured && current_strategy() != CommandMethod::Register)
            switch_to_register("this line carries no FC05 coils");
        return CommandMethod::Register;
    }
    return method;
}

CommandMethod Axis::Impl::strategy_for(std::optional<CommandMethod> method) {
    if (!method) return active_strategy();
    return active_strategy(method, true);
}

// ============================================================================================ pause and the register STOP

void Axis::Impl::set_pause(bool on) {
    if (register_stop_path()) {
        registers.pause(*bus, slave, on);
        return;
    }
    try {
        send_pause(*bus, slave, on);
    } catch (const ModbusSlaveError& e) {
        if (!coil_refused(e)) throw;
        switch_to_register("FC05 refused with exception 01 (pause)");
        registers.pause(*bus, slave, on);
    }
}

void Axis::Impl::cancel_pause(std::optional<Status> st) {
    if (!st) {
        try {
            st = status();
        } catch (const ModbusError& e) {
            log_info(kAxisLog, std::string("status before the pause cancel not readable (") + e.what() + "): cancelling");
        }
    }
    const bool fault = st && st->alarm;  // the ALRS edge would also clear it; the alarm has already ended the move
    if (register_stop_path()) {
        if (fault)
            registers.pause(*bus, slave, false);
        else
            registers.cancel_paused(*bus, slave);  // 11 20, 10 20, 10 00 (SON / SFTY kept)
        return;
    }
    if (!fault) {  // a rising ALRS edge while STP is held drops the remaining travel
        bus->write_coil(slave, reg::kCoilAlrs, true);
        bus->write_coil(slave, reg::kCoilAlrs, false);
    }
    send_pause(*bus, slave, false);
}

std::pair<Status, bool> Axis::Impl::register_stop_sequence(double timeout_s, StopRun& run) {
    stop_pause_on = true;
    std::pair<Status, bool> held;
    try {
        HaltNote halt(*this);  // the waits of this axis end on this stop (ended_from_outside)
        registers.pause(*bus, slave, true);
        halt.note();
        held = hold_until_stopped(timeout_s);
    } catch (EcDriverError& e) {
        try {
            if (run.leave()) {
                cancel_pause(std::nullopt);
                stop_pause_on = false;
            }
        } catch (const std::exception& r) {
            detail::add_note(&e, std::string("cancelling the pause of the register STOP failed too: ") + r.what());
        }
        throw;
    } catch (...) {
        try {
            if (run.leave()) {
                cancel_pause(std::nullopt);
                stop_pause_on = false;
            }
        } catch (...) {
        }
        throw;
    }
    if (!run.leave()) return held;  // another thread's stop still holds the pause: it cancels it when it ends
    cancel_pause(held.first);
    stop_pause_on = false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        last_move.reset();  // the remaining travel is dropped: change_speed has no move to re-send
    }
    return held;
}

void Axis::Impl::try_register_stop(EcDriverError* error, const std::string& when, StopRun& run) {
    stop_pause_on = true;
    std::optional<Status> st;
    bool sent = false;
    HaltNote halt(*this);  // the other waits of this axis end on this stop (ended_from_outside)
    try {
        registers.pause(*bus, slave, true);
        sent = true;
        halt.note();
    } catch (const std::exception& e) {
        detail::add_note(error, "decel stop (DRG1 pause) " + when + " failed too: " + e.what());
    }
    if (sent) {
        detail::add_note(error, "decel stop (DRG1 pause + cancel) sent " + when);
        try {
            st = hold_until_stopped(kErrorStopHoldS).first;
        } catch (const std::exception& e) {
            log_debug(kAxisLog, std::string("status unavailable while holding the pause: ") + e.what());
        }
    }
    try {
        if (!run.leave()) return;  // another thread's stop still holds the pause: it cancels it when it ends
        cancel_pause(st);  // also when the pause failed: the unit may have received it anyway
        stop_pause_on = false;
        std::lock_guard<std::mutex> lock(mutex);
        last_move.reset();
    } catch (const std::exception& e) {
        detail::add_note(error, std::string("pause not cancelled / released (") + e.what() +
                                    "); the next command retries it");
    }
}

void Axis::Impl::send_stop_first_frame() {
    const HaltNote halt(*this);  // the waits of this axis end on this stop (ended_from_outside)
    if (register_stop_path()) {
        stop_pause_on = true;
        registers.pause(*bus, slave, true);
        return;
    }
    stop_coil_on = true;
    try {
        send_stop_coil(*bus, slave, true);
    } catch (const ModbusSlaveError& e) {
        if (!coil_refused(e)) throw;
        stop_coil_on = false;
        switch_to_register("FC05 refused with exception 01 (STOP)");
        stop_pause_on = true;
        registers.pause(*bus, slave, true);
    }
}

// ============================================================================================ safety speed, action mode

Status Axis::Impl::set_safety_speed(bool on, bool verify) {
    require_control(status());
    if (register_stop_path()) {
        registers.safety_speed(*bus, slave, on);  // DRG1 b14, SON kept (DOC ME0162-12A p.5-124)
    } else {
        try {
            send_safety_speed(*bus, slave, on);
        } catch (const ModbusSlaveError& e) {
            if (!coil_refused(e)) throw;
            switch_to_register("FC05 refused with exception 01 (safety speed)");
            registers.safety_speed(*bus, slave, on);
        }
    }
    const Status st = status();
    recovery::ControlChange change;  // deliberate: not a restart sign for the link check (recovery.hpp)
    change.safety_speed = st.safety_speed();
    recovery::hooks::note_control(bus, slave, change);
    if (verify && st.safety_speed() != on)
        throw VerificationError(std::string("SFTY written ") + (on ? "ON" : "OFF") + " but DSS1 bit 14 did not follow");
    return st;
}

Status Axis::Impl::set_action_mode(bool pc, std::optional<bool> safety, bool verify) {
    bus->line()->require(LineOperation::ActionMode, "set_action_mode (0x45 cmd 0x0C)");  // nothing sent when refused
    Status st = status();
    // taking PC control from PIO: a pause at a standstill with the servo off is no refusal (axis_impl.hpp); the other ways need idle()
    const bool at_rest = pc && !st.modbus_enabled ? detail::at_rest_for_control(st) : st.idle();
    if (!at_rest || st.pushing()) {
        const std::string blocker = detail::pause_blocker(st, pc);
        if (!blocker.empty())
            throw EcNotReadyError("the action mode is only switched at rest: " + blocker + "; nothing sent");
    }
    if (!at_rest || st.pushing())
        throw EcNotReadyError(std::string("the action mode is only switched at rest (switching control mid-move stops "
                                          "the move); wait or stop() first (") +
                              detail::kPioStopHint + "); nothing sent");
    const bool sfty = safety.value_or(st.safety_speed());  // nullopt: keep what the unit has now
    if (!pc) {
        send_action_mode(*bus, slave, false, sfty);  // op1 = 0: PIO, unchanged since 1.0.0
    } else {
        // PC: the PMSL coil and the SFTY coil, not 0x45 cmd 0x0C op1 = 1. On the PCON-CB every op1 = 1 cleared DSSE b12
        // (from PIO and already in PC, either op2; name unknown, "cannot be used" in ME0162-12A p.4-20), and while b12
        // is 0 the unit ignores servo ON (SON written, STAT b1 stays 0, no alarm); of what was tried, only a PMSL coil
        // OFF set it back (PMSL ON written again did not; op1 = 0 with b12 at 0 and a power cycle were never tried).
        // PMSL ON from PIO keeps b12 (VERIFIED on the PCON-CB, 2026-11-02). PMSL ON is VERIFIED-LIVE on the EC too.
        // Only a unit without coils (FC05 exception 01) still gets op1 = 1, the one way left.
        bool by_command = no_coils();
        bool taken = false;
        if (!by_command && !st.modbus_enabled) {
            try {
                send_modbus_control(*bus, slave, true);  // FF00 only when PMSS is 0; nothing for control otherwise
                taken = true;
            } catch (const ModbusSlaveError& e) {
                if (!coil_refused(e)) throw;
                by_command = true;
            }
        }
        if (by_command) {
            send_action_mode(*bus, slave, true, sfty);
        } else {
            if (taken) st = status();
            // the safety speed by its own coil (DRG1 b14 on the register path); on the PCON UNVERIFIED that it keeps b12
            if (st.modbus_enabled && st.safety_speed() != sfty) set_safety_speed(sfty, false);
        }
    }
    st = status();
    recovery::ControlChange change;
    change.modbus_control = st.modbus_enabled;
    if (pc) change.safety_speed = st.safety_speed();
    recovery::hooks::note_control(bus, slave, change);
    if (verify) {
        if (st.modbus_enabled != pc)
            throw VerificationError(util::format("action mode %s written but DSSE PMSS reads %d", pc ? "PC" : "PIO",
                                                 st.modbus_enabled ? 1 : 0));
        if (pc && st.safety_speed() != sfty)
            throw VerificationError(util::format("action mode safety speed %d written but DSS1 SFTY reads %d",
                                                 sfty ? 1 : 0, st.safety_speed() ? 1 : 0));
    }
    return st;
}

// ============================================================================================ teach with the coils

PositionWriteResult Axis::Impl::teach_by_coils(int row, const NvWriteOptions& o) {
    // o.allow_nonvolatile / o.allow_unsafe are ignored (no longer needed): a row the position mode does not run is
    // taught with a warning
    const int rows = max_positions();
    if (row < 0 || row >= rows)
        throw SoftLimitError("row must be 0.." + std::to_string(rows - 1) + " (this unit's stored rows), got " +
                             std::to_string(row) + "; nothing written");
    const std::string inactive = inactive_row_warning(row);  // row 2 of an EC in 2-position mode
    const Status st = status();
    if (!st.homed)
        throw EcNotReadyError("the axis is not homed: a taught position would be meaningless; home() first; nothing "
                              "written");
    if (!st.idle()) throw EcNotReadyError("teach at rest only (the axis moves, pauses or homes); nothing written");
    bus->line()->require(LineOperation::Coil, "teach_position (method coil: MOD / TEAC coils)");
    rows_writable_here("teach_position (method coil)");
    PositionWriteResult result;
    result.row = row;
    if (!inactive.empty()) {
        log_warning(kAxisLog, "teach row " + std::to_string(row) + ": " + inactive);
        result.warnings.push_back(inactive);
    }
    if (!o.force) {
        // The unit fills an empty field with its defaults when it teaches (DOC ME0208 p.90): only a complete row whose
        // position already equals PNOW and whose checksum is not bad is left as it is.
        const PositionEntry stored = read_row(row);
        const bool complete = std::min({stored.speed_mm_s, stored.band_mm, stored.accel_g, stored.decel_g}) > 0;
        const bool same = reg::to_counts(stored.position_mm) == reg::to_counts(st.position_mm);
        if (complete && same && stored.checksum_ok() != std::optional<bool>(false)) {
            log_info(kAxisLog, "teach row " + std::to_string(row) + ": " + kNvUnchangedNote);
            result.entry = stored;
            result.skipped = true;
            result.note = kNvUnchangedNote;
            return result;
        }
    }
    require_control(st);
    log_warning(kAxisLog, "teach_position (method coil): EXPERIMENTAL (DOC for CON units; UNVERIFIED on an EC)");
    send_teach_mode(*bus, slave, true);
    try {
        send_position_select(*bus, slave, row);  // POSR 0x0D03 (not 0x9800, which starts a move)
        send_teach(*bus, slave, true);
        note_nonvolatile_write("stored row " + std::to_string(row) + " (teach)");
        try {
            const double deadline = now_s() + kTeachWriteS;
            sleep_at_least(0.02);  // TEAC held >= 20 ms (DOC ME0162-12A p.5-97)
            for (;;) {
                const std::vector<std::uint16_t> words = bus->read_registers(slave, reg::kDss2, 1);
                if (!words.empty() && (words[0] & reg::kDss2Pwrt)) break;
                if (now_s() >= deadline)
                    throw VerificationError(util::format("teach: DSS2 b10 PWRT did not rise within %g s; row %d may be "
                                                         "unchanged",
                                                         kTeachWriteS, row));
                sleep_poll();
            }
        } catch (EcDriverError& e) {
            try {
                send_teach(*bus, slave, false);
            } catch (const std::exception& r) {
                detail::add_note(&e, std::string("releasing the TEAC coil failed too: ") + r.what());
            }
            throw;
        }
        send_teach(*bus, slave, false);
    } catch (EcDriverError& e) {
        try {
            send_teach_mode(*bus, slave, false);
        } catch (const std::exception& r) {
            detail::add_note(&e, std::string("releasing the MOD coil failed too: ") + r.what());
        }
        throw;
    } catch (...) {
        try {
            send_teach_mode(*bus, slave, false);
        } catch (...) {
        }
        throw;
    }
    send_teach_mode(*bus, slave, false);
    result.entry = read_row(row);
    result.forced = o.force;
    return result;
}

// ============================================================================================ automatic servo OFF

Status Axis::Impl::wake_from_auto_servo_off(Status st) {
    // On CON the next move command re-energises the axis by itself (DOC PCON p.5-5; ME0155 p.81), on an EC with PIO
    // too (ME3816-6B p.4-18 to 4-21); for a Modbus 0x9900 write that is not documented, so the driver does not rely on
    // it (UNVERIFIED): servo ON first, and when SON is already a held level that does not re-energise, a SON off / on
    // cycle (the axis is de-energised already).
    log_info(kAxisLog, "automatic servo OFF (STAT ASOF): switching the servo back on before the move (UNVERIFIED path)");
    auto send = [this](bool on) {
        run_strategy(active_strategy(), "servo", [&](CommandMethod m) { send_servo(*bus, slave, on, m, &registers); });
    };
    send(true);
    for (int attempt = 0; attempt < 2; ++attempt) {
        const double deadline = now_s() + (attempt == 0 ? detail::kAsofWakeS : kServoTimeoutS);
        for (;;) {
            st = status();
            if (st.servo_on) return st;
            if (st.alarm)
                throw EcAlarmError(st.alarm_code,
                                   util::format("alarm 0x%03x while leaving automatic servo OFF", st.alarm_code));
            if (now_s() >= deadline) break;
            sleep_poll();
        }
        if (attempt == 0) {
            note_stop();  // a servo OFF frame (the axis is de-energised already)
            send(false);
            send(true);
        }
    }
    const std::vector<std::string> reasons = servo_blockers(st, true);
    throw ServoBlockedError("the servo did not come back on after the automatic servo OFF" +
                                (reasons.empty() ? std::string() : ": " + util::join(reasons, "; ")),
                            reasons);
}

// ============================================================================================ Axis: public forwarding

CommandMethod Axis::strategy() const { return impl_->current_strategy(); }
void Axis::set_strategy(CommandMethod method) { impl_->set_strategy(method); }
std::string Axis::stop_path() const { return impl_->register_stop_path() ? "register" : "coil"; }
std::optional<std::string> Axis::line_name() const { return impl_->line_name(); }

Status Axis::set_action_mode(bool pc, std::optional<bool> safety, bool verify) {
    return impl_->set_action_mode(pc, safety, verify);
}

Status Axis::set_safety_speed(bool on, bool verify) { return impl_->set_safety_speed(on, verify); }

PositionWriteResult Axis::teach_position(int row, TeachMethod method, const NvWriteOptions& options) {
    if (method == TeachMethod::Coil) return impl_->teach_by_coils(row, options);
    return impl_->teach_position(row, options);
}

}  // namespace iai_actuator_driver
