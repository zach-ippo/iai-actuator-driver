// Exception hierarchy (header-only). Every error the library raises on purpose derives from EcDriverError, so a
// caller can catch one type. Names follow the Python edition (C# names in brackets):
//
//   EcDriverError [EcDriverException]                  every error raised on purpose
//   +- ModbusError [ModbusException]                   a Modbus transaction failed (link layer)
//   |  +- ModbusTimeout                                no (complete) answer within the timeout, after all retries
//   |  +- ModbusCrcError                               an answer with a bad CRC, after all retries
//   |  +- ModbusFrameError                             an answer that does not match the request, after all retries
//   |  +- ModbusSlaveError [ModbusSlaveException]      the slave answered with an exception response (fc | 0x80)
//   |  +- IaiCommandRejected                           IAI function 0x45 answered with a rejection code
//   |  +- TransportError                               the port failed underneath the bus (USB adapter pulled)
//   +- UnsafeCommandError                              refused to send a dangerous command; nothing was sent
//   |  +- LineRuleError                                the line (a gateway) does not carry it; nothing was sent
//   +- EcError                                         the actuator cannot do what was asked
//      +- SoftLimitError                               a value is outside the unit's range; nothing was sent
//      +- EcNotReadyError                              a precondition is not met (servo off, not homed, moving...)
//      |  +- ServoBlockedError                         the servo cannot come on (EMG input, drive power cut...)
//      +- EcAlarmError                                 the unit reports an alarm
//      +- MotionTimeoutError                           motion did not finish in time (a decel stop was sent)
//      +- MotionIncompleteError                        settled away from the target
//      +- NonVolatileWriteError                        a protected row (simple-data store, not addressable)
//      +- VerificationError                            read-back after a write did not match
//   +- WaitCancelledError                              a blocking call was cancelled from another thread (cancel.hpp)
//   +- CallerAbortError                                the caller's own error (a binding's callback or hook) came back; not the port's
//   CellError (cell.hpp)                               one or more axes failed in a group command
#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace iai_actuator_driver {

/// What to check when a unit does not answer at all (appended to the no-answer errors of open / scan / the CLI).
inline constexpr const char* kNoAnswerHint =
    "check the 24 V supply, the cable, the slave address (axis number + 1) and that IA-OS (or any other program) has "
    "disconnected. Not supported, whatever the wiring: the controllers with the SEL protocol (XSEL, SSEL, ASEL, PSEL, "
    "MSEL and the TTA table-top robots) do not speak Modbus RTU, and an ELECYLINDER on an RCON-EC / REC connection "
    "unit has its serial port disabled";

/// Base class of every error raised on purpose by this library.
///
/// what() is the message plus, when the driver added notes while handling the error (for example "decel stop sent
/// after this error"), " [note; note]". Not thread-safe: add notes before the error is shared between threads.
class EcDriverError : public std::runtime_error {
public:
    explicit EcDriverError(const std::string& message) : std::runtime_error(message), message_(message), what_(message) {}

    const char* what() const noexcept override { return what_.c_str(); }

    /// The message without notes or suffixes.
    const std::string& message() const noexcept { return message_; }

    /// Notes the driver added while handling the error.
    const std::vector<std::string>& notes() const noexcept { return notes_; }

    /// Add a note (shown in what()).
    void add_note(std::string note) {
        notes_.push_back(std::move(note));
        refresh();
    }

    /// The axis already sent its decel stop for this error (so an outer handler does not stop twice).
    bool stop_attempted() const noexcept { return stop_attempted_; }
    void set_stop_attempted(bool value = true) noexcept { stop_attempted_ = value; }

protected:
    /// Text appended to the message before the notes (ModbusError: the attempts).
    virtual std::string suffix() const { return {}; }

    /// Rebuild what() after a change.
    void refresh() {
        std::string text = message_ + suffix();
        if (!notes_.empty()) {
            text += " [";
            for (std::size_t i = 0; i < notes_.size(); ++i) {
                if (i) text += "; ";
                text += notes_[i];
            }
            text += "]";
        }
        what_ = std::move(text);
    }

private:
    std::string message_;
    std::vector<std::string> notes_;
    std::string what_;
    bool stop_attempted_ = false;
};

// ------------------------------------------------------------------------------------------ Modbus / link layer

/// A Modbus transaction failed. Timeouts, CRC and frame errors are retried before they are raised.
class ModbusError : public EcDriverError {
public:
    using EcDriverError::EcDriverError;

    /// How many attempts were made before this error was raised (1 = no retry).
    int attempts() const noexcept { return attempts_; }
    void set_attempts(int attempts) {
        attempts_ = attempts;
        refresh();
    }

protected:
    std::string suffix() const override {
        return attempts_ > 1 ? " (gave up after " + std::to_string(attempts_) + " attempts)" : std::string();
    }

private:
    int attempts_ = 1;
};

/// No (complete) response arrived within the timeout, after all retries.
class ModbusTimeout : public ModbusError {
public:
    using ModbusError::ModbusError;
};

/// A response arrived but its CRC did not match, after all retries.
class ModbusCrcError : public ModbusError {
public:
    using ModbusError::ModbusError;
};

/// A response arrived but did not match the request (wrong slave, function or echo), after all retries.
class ModbusFrameError : public ModbusError {
public:
    using ModbusError::ModbusError;
};

/// Name of a Modbus exception code (DOC): 1 Illegal Function, 2 Illegal Data Address, 3 Illegal Data Value,
/// 4 Slave Device Failure, otherwise "Unknown exception".
inline const char* modbus_exception_name(int code) noexcept {
    switch (code) {
        case 0x01: return "Illegal Function";
        case 0x02: return "Illegal Data Address";
        case 0x03: return "Illegal Data Value";
        case 0x04: return "Slave Device Failure";
        default: return "Unknown exception";
    }
}

/// The slave answered with a Modbus exception response (function code | 0x80). A valid answer: never retried.
class ModbusSlaveError : public ModbusError {
public:
    ModbusSlaveError(int code, int function, const std::string& message = std::string())
        : ModbusError(message.empty() ? default_message(code, function) : message), code_(code), function_(function) {}

    /// Exception code (1 illegal function, 2 illegal address, 3 illegal value, 4 failure).
    int code() const noexcept { return code_; }
    /// Function code of the rejected request.
    int function() const noexcept { return function_; }
    /// Name of code().
    const char* code_name() const noexcept { return modbus_exception_name(code_); }

private:
    static std::string default_message(int code, int function) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "Modbus exception %02Xh (%s) for function %02Xh", code & 0xFF,
                      modbus_exception_name(code), function & 0xFF);
        return buf;
    }
    int code_;
    int function_;
};

/// An IAI control command (function 0x45) was answered with a non-zero rejection code: the unit understood the frame
/// and refused the command. Never retried. reason() names the code (0x11 "1st operand error" is VERIFIED, the other
/// names UNVERIFIED), hint() is the driver's short advice ("" if none). The bus fills both from its rejection tables
/// (iai_rejection_name / iai_rejection_hint in modbus.hpp).
class IaiCommandRejected : public ModbusError {
public:
    IaiCommandRejected(int function, int command, int rejection, std::string reason, std::string hint,
                       int status_byte = -1, std::vector<std::uint32_t> status_words = {})
        : ModbusError(build_message(function, command, rejection, reason, hint)),
          function_(function), command_(command), rejection_(rejection), status_byte_(status_byte),
          status_words_(std::move(status_words)), reason_(std::move(reason)), hint_(std::move(hint)) {}

    int function() const noexcept { return function_; }        ///< 0x45
    int command() const noexcept { return command_; }          ///< the rejected command code
    int rejection() const noexcept { return rejection_; }      ///< 0x45: one byte 0x01-0x3F
    int status_byte() const noexcept { return status_byte_; }  ///< the reply's status byte, -1 when unknown
    /// 0x48: the reply's four status words (empty for 0x45)
    const std::vector<std::uint32_t>& status_words() const noexcept { return status_words_; }
    const std::string& reason() const noexcept { return reason_; }
    const std::string& hint() const noexcept { return hint_; }

private:
    static std::string build_message(int function, int command, int rejection, const std::string& reason,
                                     const std::string& hint) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "IAI command 0x%02x (function %02Xh) rejected with code 0x%02x (", command,
                      function & 0xFF, rejection);
        std::string text = buf + reason + ")";
        if (!hint.empty()) text += ": " + hint;
        return text;
    }
    int function_;
    int command_;
    int rejection_;
    int status_byte_;
    std::vector<std::uint32_t> status_words_;
    std::string reason_;
    std::string hint_;
};

/// The port failed underneath the bus (a USB adapter pulled: the COM port vanishes, VERIFIED in the Python edition).
/// Never retried.
class TransportError : public ModbusError {
public:
    using ModbusError::ModbusError;
};

// ------------------------------------------------------------------------------------------ refusals

/// Refused to send a command that could damage configuration (baud / axis change, initialisation, a dangerous
/// parameter, an unknown raw item...). Nothing was sent.
class UnsafeCommandError : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};

/// The line the unit sits on does not carry this command (no FC05 coils, no IAI 0x45 / 0x48 commands or no broadcast
/// behind a gateway; line.hpp). Nothing was sent.
class LineRuleError : public UnsafeCommandError {
public:
    using UnsafeCommandError::UnsafeCommandError;
};

// ------------------------------------------------------------------------------------------ actuator layer

/// The actuator cannot do what was asked.
class EcError : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};

/// A target or motion parameter is outside the unit's range (or a configured cap). Nothing was sent.
class SoftLimitError : public EcError {
public:
    using EcError::EcError;
};

/// A precondition is not met (servo off, not homed, Modbus control off, moving...).
class EcNotReadyError : public EcError {
public:
    using EcError::EcError;
};

/// The servo cannot come on: the status shows an EMG input, a drive power cut or a released enable switch. reasons()
/// lists them (also after a timeout). Nothing was sent when refused up front.
class ServoBlockedError : public EcNotReadyError {
public:
    ServoBlockedError(const std::string& message, std::vector<std::string> reasons)
        : EcNotReadyError(message), reasons_(std::move(reasons)) {}
    const std::vector<std::string>& reasons() const noexcept { return reasons_; }

private:
    std::vector<std::string> reasons_;
};

/// The actuator reports an alarm.
class EcAlarmError : public EcError {
public:
    explicit EcAlarmError(int alarm_code, const std::string& message = std::string())
        : EcError(message.empty() ? default_message(alarm_code) : message), alarm_code_(alarm_code) {}
    /// The alarm code (ALMC 0x9002).
    int alarm_code() const noexcept { return alarm_code_; }

private:
    static std::string default_message(int code) {
        char buf[48];
        std::snprintf(buf, sizeof buf, "actuator alarm 0x%03x", code);
        return buf;
    }
    int alarm_code_;
};

/// Motion did not complete in time. A decel stop was requested before this was raised.
class MotionTimeoutError : public EcError {
public:
    using EcError::EcError;
};

/// The axis settled (in position, not moving) away from the commanded target: a STOP from another master, the unit
/// ignoring the command (STOP held, PIO in control) or the unit clamping the target to its own soft limit.
class MotionIncompleteError : public EcError {
public:
    using EcError::EcError;
};

/// A write to a protected row (the simple-data rows 256-267, a row the table cannot address). Nothing was written.
class NonVolatileWriteError : public EcError {
public:
    using EcError::EcError;
};

/// Read-back after a write did not match what was written.
class VerificationError : public EcError {
public:
    using EcError::EcError;
};

/// A blocking call was ended by a cancellation from another thread (Axis::cancel_waits; cancel.hpp). A motion wait
/// sent a decel stop before this was raised (its notes say so). Bindings map it to KeyboardInterrupt /
/// OperationCanceledException.
class WaitCancelledError : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};

/// The caller's own error, handed back by a transport callback (IAD_CALLBACK_ABORT) or a simulator request hook
/// (IAD_SIM_ABORT): it is not the port's fault and not the unit's. It is never retried, never taken for a link fault,
/// and the best-effort parts of a recovery, a unit-info read, a backup or a self-test (which note an error and go on)
/// let it through, so the call ends with it. Reaches the C ABI as IAD_ERR_DRIVER (the kind "EcDriverError").
class CallerAbortError : public EcDriverError {
public:
    using EcDriverError::EcDriverError;
};

}  // namespace iai_actuator_driver
