// Private to the core (internal: not installed, not part of the API yet): the SEL link, the command / reply exchange
// with one SEL controller (IAI Protocol B / Format B) over any Transport, and the retry rules of a protocol whose only
// failure signal is silence.
//
// * One command in flight per link (a mutex); at least gap_s (1 ms; 3 ms on RS-485) after the last frame; bytes
//   received and not yet read are dropped before each command. The reply is read up to its LF, so a stream transport
//   that splits or coalesces segments works (StreamShim, sel_shim.hpp). A line that is not the reply - a bad checksum
//   or terminator, another station, another message id (a late reply to an earlier command) - is dropped and reading
//   goes on until the deadline (timeout_s, 3 s).
// * An error reply ('&') is never retried: SelRejectedError with its code (the manual gives no code table, DOC).
// * Silence (no accepted reply by the deadline; DOC: a frame the controller cannot accept, and an execute command on a
//   channel not enabled for execution, get silence too):
//   - a query is sent again, up to `retries` (3) more times, then SelSilentError;
//   - 238H stop and 252H alarm reset change nothing when repeated: sent again the same way, up to `hard_retries` (5);
//   - any other execute command is NEVER sent again blindly: the controller may have acted on it and lost only the
//     reply. The caller's Verifier re-queries the state (212H for servo, home and moves; 213H for a program run): "took
//     effect" is success; any other verdict is SelSilentError with nothing re-sent, unless retry_execute is set. A data
//     write (positions, outputs, flags, variables: repeating it writes the same data) is also sent again when its
//     verifier re-reads the data and finds it not written (not_taken).
//   - 25BH software reset has no reply (DOC): sent once, then 200H test calls until the controller answers again.
// * Completion of an operation (CompletionTracker, wait_done): the reply to a move means "accepted", not "done"; the
//   manual does not say when 212H bit 0 "in use" turns ON after the reply (DOC), so bit 4 "completed" can still be the
//   previous command's. Done needs bit 0 OFF with bit 4 ON after bit 0 was seen ON, or the position within the band of
//   the target and steady for settle_s: a stale bit 4 never reports done early.
// Counters: command frames sent per message id and per call (one execute frame per command, even under silence).
// Everything here is UNVERIFIED on a controller: no SEL controller has been tested.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "iai_actuator_driver/errors.hpp"
#include "iai_actuator_driver/transport.hpp"
#include "sel_frame.hpp"
#include "sel_messages.hpp"

namespace iai_actuator_driver {
namespace sel {

/// A SEL exchange failed.
class SelError : public EcDriverError {
public:
    SelError(const std::string& message, int message_id) : EcDriverError(message), message_id_(message_id) {}
    /// The message id of the command.
    int message_id() const noexcept { return message_id_; }

private:
    int message_id_;
};

/// No accepted reply by the deadline, after the retries the command's rule allows.
class SelSilentError : public SelError {
public:
    SelSilentError(const std::string& message, int message_id, int frames)
        : SelError(message, message_id), frames_(frames) {}
    /// Command frames sent for the call.
    int frames() const noexcept { return frames_; }

private:
    int frames_;
};

/// The controller answered with an error reply ('&'). Its code is opaque: the manual gives no meaning (DOC).
class SelRejectedError : public SelError {
public:
    SelRejectedError(const std::string& message, int message_id, int code)
        : SelError(message, message_id), code_(code) {}
    int code() const noexcept { return code_; }

private:
    int code_;
};

/// What a command that met silence may do next.
enum class SilenceRule {
    resend,              ///< queries, 238H stop, 252H alarm reset: sent again
    verify,              ///< operations (servo, home, moves, jog, programs, speed change): re-queried, never re-sent
    verify_then_resend,  ///< data writes (24AH-24EH, 244H-246H, 26CH-26EH): re-sent when the re-read says not written
    no_reply,            ///< 25BH software reset: no reply expected (DOC)
};

/// The rule of `request` (by its message id and kind).
SilenceRule silence_rule(const Request& request) noexcept;

/// A verifier's reading of the state after an execute command met silence.
enum class Verdict {
    took_effect,   ///< the command was acted on (or its aim already holds): success
    not_taken,     ///< it surely was not (a data re-read shows the old data)
    inconclusive,  ///< cannot tell (a stale bit 4 looks like nothing happened)
};

class SelLink;
/// Re-queries the controller through the link after silence. An EcDriverError it throws counts as inconclusive.
using Verifier = std::function<Verdict(SelLink&)>;

/// Settings of a SelLink.
struct SelLinkOptions {
    int station = 0x99;           ///< I/O parameter 91 (the manual's examples use 99h)
    double timeout_s = 3.0;       ///< per command frame (DOC: 3 s)
    double gap_s = 0.001;         ///< after the last frame before a command (0.003 on RS-485)
    double poll_s = 0.001;        ///< sleep between two empty reads
    int retries = 3;              ///< extra frames for a query on silence (and with retry_execute for an execute)
    int hard_retries = 5;         ///< extra frames for 238H stop and 252H alarm reset
    bool retry_execute = false;   ///< send an execute command again when its re-query does not confirm it
    bool with_checksum = true;    ///< false: "@@" in place of the checksum (the controller skips the check, DOC)
    /// Every frame sent (true) and every line received (false), as bytes; called with the link's lock held.
    std::function<void(bool sent, const std::string& bytes)> frame_sink;
};

/// The result of SelLink::send.
struct SendOutcome {
    std::optional<Frame> reply;  ///< the accepted normal reply; nullopt when confirmed by re-query or none expected
    int frames = 0;              ///< command frames this call sent
    int requeries = 0;           ///< verifier calls

    bool confirmed_by_requery() const noexcept { return !reply && requeries > 0; }
};

class SelLink {
public:
    explicit SelLink(std::shared_ptr<Transport> transport, SelLinkOptions options = {});
    ~SelLink();
    SelLink(const SelLink&) = delete;
    SelLink& operator=(const SelLink&) = delete;

    const SelLinkOptions& options() const noexcept;
    Transport& transport() const noexcept;

    /// One command frame, one reply, no retry: the reply (nullopt on silence, or at once when none is expected).
    /// SelRejectedError on an error reply; TransportError from the port; the request's encoding errors before anything
    /// is sent.
    std::optional<Frame> exchange(const Request& request);

    /// Send `request` under its SilenceRule. `verify` is used for the verify rules only (none: inconclusive).
    /// SelSilentError when the rule gives up; SelRejectedError on an error reply (never retried).
    SendOutcome send(const Request& request, const Verifier& verify = nullptr);

    /// A query (MessageKind::query; std::invalid_argument otherwise): the data of its reply.
    std::string query(const Request& request);

    /// 25BH, then 200H test calls (timeout_s each) until one is answered; SelSilentError when none is within
    /// `timeout_s` (the controller's re-initialisation). Nothing is resumed.
    void software_reset(double timeout_s = 10.0);

    // ---- counters
    int frames_sent() const;                ///< command frames, every id
    int frames_sent(int message_id) const;  ///< command frames of one id
    int lines_dropped() const;              ///< received lines that were not the reply
    int silences() const;                   ///< frames that met silence
    void reset_counters();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---- verifiers (re-query after silence) ---------------------------------------------------------------------------
/// 232H: every axis of `axes` shows the servo state asked for.
Verifier verify_servo(AxisPattern axes, bool on);
/// 233H: an axis of `axes` is homing or in use, or every one is homed though none of them was in `homed_before`.
Verifier verify_home(AxisPattern axes, AxisPattern homed_before = 0);
/// 234H, 237H, 26BH (and 235H / 236H): an axis of `axes` is in use, or every axis is within `band` of its target
/// (`targets`: one per axis, ascending; empty = unknown). A stale bit 4 is inconclusive.
Verifier verify_motion(AxisPattern axes, std::vector<double> targets = {}, double band = 0.05);
/// 253H: the program shows started.
Verifier verify_program_started(int program);

// ---- completion ---------------------------------------------------------------------------------------------------
enum class Completion {
    pending,     ///< still running, or not started as far as 212H tells
    done,        ///< bit 0 OFF, bit 4 ON
    cancelled,   ///< bit 0 OFF, bit 4 OFF after it ran (stop, emergency stop, MANU, an axis error)
    push_error,  ///< bit 5
};

/// "pending", "done", "cancelled", "push_error".
const char* to_string(Completion completion) noexcept;

/// The completion rule of one operation over successive 212H replies.
class CompletionTracker {
public:
    /// `targets`: one per axis of `axes`, ascending; empty = unknown (done then needs bit 0 seen ON).
    CompletionTracker(AxisPattern axes, std::vector<double> targets = {}, double band = 0.05, double settle_s = 0.1);

    /// The state after `reply` (a 212H reply that covers `axes`), read at `now` (s).
    Completion update(const AxisStatusReply& reply, double now);

private:
    struct AxisWatch {
        int axis = 0;
        std::optional<double> target;
        bool seen_in_use = false;
        std::optional<double> in_band_since;
    };
    std::vector<AxisWatch> watches_;
    double band_;
    double settle_s_;
};

/// Poll 212H every `poll_s` until the operation on `axes` completes (the link's lock is let go between polls).
/// MotionTimeoutError after `timeout_s` (nothing is sent to stop the axes: the caller decides).
Completion wait_done(SelLink& link, AxisPattern axes, std::vector<double> targets = {}, double timeout_s = 30.0,
                     double band = 0.05, double settle_s = 0.1, double poll_s = 0.02);

}  // namespace sel
}  // namespace iai_actuator_driver
