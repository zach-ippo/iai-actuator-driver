// The SEL link: one controller's command / reply exchange and the retry rules (sel_link.hpp). UNVERIFIED on a
// controller.
#include "sel_link.hpp"

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "iai_actuator_driver/timing.hpp"
#include "util.hpp"

namespace iai_actuator_driver {
namespace sel {

namespace {

/// A line longer than this without its LF is noise: dropped (the largest reply buffer is 2048 bytes, DOC).
constexpr std::size_t kMaxLineBytes = 8192;

/// "234H".
std::string id_text(int id) { return util::format("%03XH", static_cast<unsigned>(id)); }

std::string frames_text(int frames) { return frames == 1 ? "1 frame" : util::format("%d frames", frames); }

bool is_data_write(int id) noexcept {
    return (id >= kSetOutput && id <= kSetStringVariables) || (id >= kPositionRangeWrite && id <= kPositionClear) ||
           (id >= kPositionRangeWrite2 && id <= kPositionClear2);
}

/// The 212H statuses of `axes`, in ascending order (an axis the reply leaves out is not present).
AxisStatusReply axis_statuses(SelLink& link, AxisPattern axes) {
    return parse_axis_status(link.query(axis_status_query(axes)), axes);
}

}  // namespace

SilenceRule silence_rule(const Request& request) noexcept {
    if (!request.reply_expected) return SilenceRule::no_reply;
    if (request.kind == MessageKind::query) return SilenceRule::resend;
    if (request.id == kStop || request.id == kAlarmReset) return SilenceRule::resend;
    if (is_data_write(request.id)) return SilenceRule::verify_then_resend;
    return SilenceRule::verify;
}

struct SelLink::Impl {
    std::shared_ptr<Transport> transport;
    SelLinkOptions options;
    mutable std::mutex mutex;
    double last_io = -1e9;
    std::map<int, int> frames;
    int frames_total = 0;
    int dropped = 0;
    int silences = 0;

    void sink(bool sent, const std::string& bytes) {
        if (!options.frame_sink) return;
        try {
            options.frame_sink(sent, bytes);
        } catch (...) {
            // a frame logger must never break communication
        }
    }

    std::optional<Frame> exchange(const Request& request) {
        const std::string command = encode_request(request, options.station, options.with_checksum);
        std::lock_guard<std::mutex> lock(mutex);
        sleep_at_least(last_io + options.gap_s - now_s());
        transport->reset_input();
        sink(true, command);
        transport->write(reinterpret_cast<const std::uint8_t*>(command.data()), command.size());
        ++frames[request.id];
        ++frames_total;
        last_io = now_s();
        if (!request.reply_expected) return std::nullopt;
        const double deadline = now_s() + options.timeout_s;
        std::string pending;
        std::uint8_t buffer[512];
        for (;;) {
            const std::size_t n = transport->read(buffer, sizeof buffer);
            if (n > 0) {
                last_io = now_s();
                pending.append(reinterpret_cast<const char*>(buffer), n);
                std::size_t lf;
                while ((lf = pending.find('\n')) != std::string::npos) {
                    const std::string line = pending.substr(0, lf + 1);
                    pending.erase(0, lf + 1);
                    sink(false, line);
                    const DecodeResult d = decode_reply(line, options.station, request.id);
                    if (!d.ok()) {
                        ++dropped;  // damaged, another station's, or a late reply to an earlier command
                        continue;
                    }
                    if (d.frame.kind == FrameKind::error_reply) {
                        throw SelRejectedError(util::format("station %02Xh refused %s with error code %03Xh (the "
                                                            "manual gives no meaning for the codes)",
                                                            static_cast<unsigned>(options.station),
                                                            id_text(request.id).c_str(),
                                                            static_cast<unsigned>(d.frame.id)),
                                               request.id, d.frame.id);
                    }
                    return d.frame;
                }
                if (pending.size() > kMaxLineBytes) {
                    pending.clear();
                    ++dropped;
                }
            }
            if (now_s() >= deadline) break;
            if (n == 0) sleep_at_least(options.poll_s);  // through the clock seam: a virtual clock moves on
        }
        if (!pending.empty()) sink(false, pending);
        ++silences;
        return std::nullopt;
    }
};

SelLink::SelLink(std::shared_ptr<Transport> transport, SelLinkOptions options)
    : impl_(std::make_unique<Impl>()) {
    if (!transport) throw std::invalid_argument("SelLink needs a transport");
    if (options.station < 0 || options.station > 0xFF) throw std::out_of_range("SEL station must be 00h..FFh");
    if (!(options.timeout_s > 0.0)) throw std::invalid_argument("SEL timeout_s must be above 0");
    if (options.retries < 0 || options.hard_retries < 0) {
        throw std::invalid_argument("SEL retries must not be negative");
    }
    impl_->transport = std::move(transport);
    impl_->options = std::move(options);
}

SelLink::~SelLink() = default;

const SelLinkOptions& SelLink::options() const noexcept { return impl_->options; }

Transport& SelLink::transport() const noexcept { return *impl_->transport; }

std::optional<Frame> SelLink::exchange(const Request& request) { return impl_->exchange(request); }

SendOutcome SelLink::send(const Request& request, const Verifier& verify) {
    const SelLinkOptions& o = impl_->options;
    const SilenceRule rule = silence_rule(request);
    const std::string where = util::format("station %02Xh", static_cast<unsigned>(o.station));
    SendOutcome out;
    if (rule == SilenceRule::no_reply) {
        impl_->exchange(request);
        out.frames = 1;
        return out;
    }
    const int extra = rule == SilenceRule::resend && request.kind == MessageKind::execute ? o.hard_retries : o.retries;
    for (int attempt = 0;; ++attempt) {
        out.reply = impl_->exchange(request);
        ++out.frames;
        if (out.reply) return out;
        if (rule == SilenceRule::resend) {
            if (attempt < extra) continue;
            throw SelSilentError(util::format("no reply from %s to %s after %s (%.1f s each): check the cable, the "
                                              "baud rate and framing, and the station number (I/O parameter 91)",
                                              where.c_str(), id_text(request.id).c_str(),
                                              frames_text(out.frames).c_str(), o.timeout_s),
                                 request.id, out.frames);
        }
        Verdict verdict = Verdict::inconclusive;
        std::string why;
        if (verify) {
            ++out.requeries;
            try {
                verdict = verify(*this);
            } catch (const EcDriverError& e) {
                why = std::string("; the re-query failed: ") + e.what();
            }
        }
        if (verdict == Verdict::took_effect) return out;
        const bool rewrite = rule == SilenceRule::verify_then_resend && verdict == Verdict::not_taken;
        const bool again = attempt < o.retries && (o.retry_execute || rewrite);
        if (again) continue;
        const char* reading = !verify                            ? "nothing re-queried it"
                              : verdict == Verdict::not_taken    ? "the re-query shows it not acted on"
                                                                 : "the re-query could not confirm it";
        throw SelSilentError(util::format("no reply from %s to %s (%s sent, %.1f s each) and %s%s; not sent again: "
                                          "an execute command may have been acted on with only its reply lost (a "
                                          "channel not enabled for execution commands also gets silence)",
                                          where.c_str(), id_text(request.id).c_str(), frames_text(out.frames).c_str(),
                                          o.timeout_s, reading, why.c_str()),
                             request.id, out.frames);
    }
}

std::string SelLink::query(const Request& request) {
    if (request.kind != MessageKind::query) {
        throw std::invalid_argument("SelLink::query is for queries; " + id_text(request.id) + " is an execute command");
    }
    return send(request).reply->data;
}

void SelLink::software_reset(double timeout_s) {
    impl_->exchange(sel::software_reset());
    const Request probe = test_call("IAD-RESET0");
    const double deadline = now_s() + timeout_s;
    int probes = 0;
    while (now_s() < deadline) {
        ++probes;
        if (const std::optional<Frame> reply = impl_->exchange(probe)) {
            parse_test_call(reply->data, "IAD-RESET0");
            return;
        }
    }
    throw SelSilentError(util::format("station %02Xh did not answer within %.1f s of a software reset (%d test calls)",
                                      static_cast<unsigned>(impl_->options.station), timeout_s, probes),
                         kSoftwareReset, 1);
}

int SelLink::frames_sent() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->frames_total;
}

int SelLink::frames_sent(int message_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->frames.find(message_id);
    return it == impl_->frames.end() ? 0 : it->second;
}

int SelLink::lines_dropped() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->dropped;
}

int SelLink::silences() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->silences;
}

void SelLink::reset_counters() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->frames.clear();
    impl_->frames_total = 0;
    impl_->dropped = 0;
    impl_->silences = 0;
}

// ---- verifiers ----------------------------------------------------------------------------------------------------
Verifier verify_servo(AxisPattern axes, bool on) {
    return [axes, on](SelLink& link) {
        const AxisStatusReply r = axis_statuses(link, axes);
        if (r.axes.empty()) return Verdict::inconclusive;
        for (const AxisStatus& s : r.axes) {
            if (s.servo_on() != on) return Verdict::inconclusive;
        }
        return Verdict::took_effect;
    };
}

Verifier verify_home(AxisPattern axes, AxisPattern homed_before) {
    return [axes, homed_before](SelLink& link) {
        const AxisStatusReply r = axis_statuses(link, axes);
        if (r.axes.empty()) return Verdict::inconclusive;
        bool all_homed = true;
        for (const AxisStatus& s : r.axes) {
            if (s.in_use() || s.home_state() == 1) return Verdict::took_effect;
            all_homed = all_homed && s.homed();
        }
        return all_homed && (homed_before & axes) == 0 ? Verdict::took_effect : Verdict::inconclusive;
    };
}

Verifier verify_motion(AxisPattern axes, std::vector<double> targets, double band) {
    if (!targets.empty() && targets.size() != static_cast<std::size_t>(axis_count(axes))) {
        throw std::invalid_argument("verify_motion: one target per axis of the pattern");
    }
    return [axes, targets = std::move(targets), band](SelLink& link) {
        const AxisStatusReply r = axis_statuses(link, axes);
        if (r.axes.empty()) return Verdict::inconclusive;
        for (const AxisStatus& s : r.axes) {
            if (s.in_use()) return Verdict::took_effect;
        }
        if (targets.empty()) return Verdict::inconclusive;  // a stale bit 4 looks like nothing happened
        const std::vector<int> order = axes_of(axes);
        for (const AxisStatus& s : r.axes) {
            for (std::size_t i = 0; i < order.size(); ++i) {
                if (order[i] == s.axis && !(std::fabs(s.position - targets[i]) <= band)) return Verdict::inconclusive;
            }
        }
        return r.axes.size() == order.size() ? Verdict::took_effect : Verdict::inconclusive;
    };
}

Verifier verify_program_started(int program) {
    return [program](SelLink& link) {
        const ProgramStatus p = parse_program_status(link.query(program_status_query(program)), program);
        return p.started ? Verdict::took_effect : Verdict::inconclusive;
    };
}

// ---- completion ---------------------------------------------------------------------------------------------------
const char* to_string(Completion completion) noexcept {
    switch (completion) {
        case Completion::pending: return "pending";
        case Completion::done: return "done";
        case Completion::cancelled: return "cancelled";
        default: return "push_error";
    }
}

CompletionTracker::CompletionTracker(AxisPattern axes, std::vector<double> targets, double band, double settle_s)
    : band_(band), settle_s_(settle_s) {
    const std::vector<int> order = axes_of(axes);
    if (order.empty()) throw std::invalid_argument("CompletionTracker: no axis");
    if (!targets.empty() && targets.size() != order.size()) {
        throw std::invalid_argument("CompletionTracker: one target per axis of the pattern");
    }
    for (std::size_t i = 0; i < order.size(); ++i) {
        AxisWatch w;
        w.axis = order[i];
        if (!targets.empty()) w.target = targets[i];
        watches_.push_back(w);
    }
}

Completion CompletionTracker::update(const AxisStatusReply& reply, double now) {
    bool any_in_use = false;
    bool any_pending = false;
    bool any_cancelled = false;
    bool any_push = false;
    for (AxisWatch& w : watches_) {
        const AxisStatus* s = nullptr;
        for (const AxisStatus& candidate : reply.axes) {
            if (candidate.axis == w.axis) s = &candidate;
        }
        if (s == nullptr) {  // not in the reply: nothing to tell
            any_pending = true;
            continue;
        }
        if (s->in_use()) {
            w.seen_in_use = true;
            w.in_band_since.reset();
            any_in_use = true;
            continue;
        }
        if (w.seen_in_use) {  // it ran and stopped: bit 4 / bit 5 are this command's
            if (s->push_error()) {
                any_push = true;
            } else if (!s->completed()) {
                any_cancelled = true;
            }
            continue;
        }
        // Never seen in use: bit 4 may still be the previous command's. Done only at the target, steady, bit 4 ON.
        if (w.target && std::fabs(s->position - *w.target) <= band_ && s->completed()) {
            if (!w.in_band_since) w.in_band_since = now;
            if (now - *w.in_band_since >= settle_s_) continue;
        } else {
            w.in_band_since.reset();
        }
        any_pending = true;
    }
    if (any_in_use || any_pending) return Completion::pending;
    if (any_push) return Completion::push_error;
    if (any_cancelled) return Completion::cancelled;
    return Completion::done;
}

Completion wait_done(SelLink& link, AxisPattern axes, std::vector<double> targets, double timeout_s, double band,
                     double settle_s, double poll_s) {
    CompletionTracker tracker(axes, std::move(targets), band, settle_s);
    const double deadline = now_s() + timeout_s;
    for (;;) {
        const Completion c = tracker.update(axis_statuses(link, axes), now_s());
        if (c != Completion::pending) return c;
        if (now_s() >= deadline) {
            throw MotionTimeoutError(util::format("SEL axes %02Xh not done within %.1f s", static_cast<unsigned>(axes),
                                                  timeout_s));
        }
        sleep_at_least(poll_s);
    }
}

}  // namespace sel
}  // namespace iai_actuator_driver
